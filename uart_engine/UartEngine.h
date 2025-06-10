#ifndef __STM32_UARTENGINE_H__
#define __STM32_UARTENGINE_H__


#include "main.h"  // Must contain UART_HandleTypeDef definition

#if defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED)
#include "Uartsettings.h"

#if UART_ENGINE_ON

#include "status.h"
#include "uart_regs.h"
#include "UartBase.h"
#include "sbuf/RingBuffer.h"
#include <functional>
#include <cstring>   // for std::memcpy

using namespace buffers;

enum class UartType : u8 {
	UART_IT,

#if defined(HAL_DMA_MODULE_ENABLED)
	UART_DMA,
	UART_DMA_CIRCULAR
#else
#	warning "[UART ENGINE]: DMA is disabled"
#endif /* HAL_DMA_MODULE_ENABLED */

};

// Class for cyclic UART data reception using DMA (ReceiveToIdle mode).
// Template parameters allow configuration for the UART mode, buffer size, and number of buffers.
template <UartType mode, reg BufferSize = 128>
#if UART_ENGINE_RS485_ON
class UartEngine : public UartBase
#else
class UartEngine final : public UartBase
#endif /* UART_ENGINE_RS485_ON */
{
	static_assert(BufferSize > 16,
			"BufferSize 32 or more");

public:
	struct Init {
		UART_HandleTypeDef* huart;  // UART descriptor
	};

	UartEngine();
	virtual ~UartEngine() override;


	// Initializes the handler by storing the UART descriptor and starting DMA reception.
	bool init(const Init &initData);
	// Processes buffered data and restarts reception if needed.
	bool proceed(const u32 time);

	using rx_it_handler_t   	= std::function<bool(const u8* const data, const u16 size)>;
	using rx_loop_handler_t 	= std::function<u16(const u8* data, u16 size, const u32 time)>;
	using tx_handler_t   		= std::function<void(const status_t status)>;
	using error_handler_t   	= std::function<void(const status_t status)>;

	void subscribeRxIt(const rx_it_handler_t handler)   { m_rxItHandler = handler; }
	void subscribeRxLoop(const rx_loop_handler_t handler) { m_rxLoopHandler = handler; }
	void subscribeTx(const tx_handler_t handler) { m_txHandler = handler; }
	void subscribeError(const error_handler_t handler) { m_errorHandler = handler; }

	inline bool isTxBusy() const { return const_cast<volatile bool&>(m_isTxBusy); }
	bool send(const u8* const data, const u16 size);
protected:
	// Friend declaration for the HAL UART event callbacks.
	friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
	friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
	friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

	// Called from HAL_UARTEx_RxEventCallback.
	// newPos - byte count position where DMA has written the data.
	virtual void onRxCplt(const u16 newPos) override;
	virtual inline void onTxCplt() override;
	virtual void onErrorEvent() override;

	inline void receiveRestart();

protected:
	/*
	 * Receiver part
	 */
	// DMA reception buffer (circular)
	std::array<u8, BufferSize> m_rxRing;
	//Receive buffer for data processing (message extraction)
	sbuf::ArrayRB<BufferSize << 1> m_rxBuffer;
	// Stores the last processed byte position.
	u16 m_oldPos = 0;

	// Callback functions for immediate (IT) and loop (delayed) processing.
	rx_it_handler_t m_rxItHandler = nullptr;
	rx_loop_handler_t m_rxLoopHandler = nullptr;

	bool m_isStarted = false;

	/*
	 * Tranceiver part
	 */
	volatile bool m_isTxBusy = false;
	tx_handler_t m_txHandler = nullptr;

	/*
	 * Error part
	 */
	reg error = 0;
	error_handler_t m_errorHandler = nullptr;

	/*
	 * Ckeck part
	 */
	u32 m_lastCheckTime = 0;
	reg checkCounter = 0;
};

template<UartType mode, reg BufferSize>
bool UartEngine<mode, BufferSize>::init(const Init &initData)
{
	// Check if the UART descriptor is valid.
	if (!initData.huart) {
		Error_Handler();
		return false;
	}

	m_isStarted = false;

	// Check if the UART dma descriptor is valid.
	if constexpr (mode == UartType::UART_DMA || mode == UartType::UART_DMA_CIRCULAR) {
		if (!initData.huart->hdmarx || !initData.huart->hdmatx) {
			Error_Handler();
			return false;
		}
	}

	// Register this handler in the registry.
	UartBase::attachUart(initData.huart);

	// Start cyclic reception using the appropriate HAL function based on mode.
	receiveRestart();
	return true;
}

template<UartType mode, reg BufferSize>
UartEngine<mode, BufferSize>::UartEngine()
{
	// Initialize the buffer collection.
	m_rxBuffer.clear();
}

template<UartType mode, reg BufferSize>
UartEngine<mode, BufferSize>::~UartEngine()
{
	HAL_UART_DMAStop(m_huart);
	HAL_UART_Abort(m_huart);
}



template<UartType mode, reg BufferSize>
bool UartEngine<mode, BufferSize>::proceed(const u32 time)
{
	/*
	 * Read from packet queue
	 */
	{
		// If a loop handler is subscribed and there is buffered data, process it.
		if (m_rxLoopHandler && !m_rxBuffer.isEmpty()) {
			auto [data, size] = m_rxBuffer.front();

			if(data && size) {
				const auto popSize = m_rxLoopHandler(data, size, time);
				m_rxBuffer.pop(popSize);
			}
		}
	}

	/*
	 * Check uart state
	 */

	bool isRestarted = false;
	{
		if ((time - m_lastCheckTime) > 200) {
			m_lastCheckTime = time;

			// Get UART status
			const HAL_UART_StateTypeDef uartState = HAL_UART_GetState(m_huart);
			const u32 errorCode = HAL_UART_GetError(m_huart);

			// Determine active states
			const bool isRxActive = (uartState == HAL_UART_STATE_BUSY_RX) ||
					(uartState == HAL_UART_STATE_BUSY_TX_RX);

			const bool isTxActive = (uartState == HAL_UART_STATE_BUSY_TX) ||
					(uartState == HAL_UART_STATE_BUSY_TX_RX);

			// Check for DMA errors
			bool isDmaRxError = false;
			if constexpr (mode == UartType::UART_DMA || mode == UartType::UART_DMA_CIRCULAR) {
				isDmaRxError = (m_huart->hdmarx &&
						(HAL_DMA_GetError(m_huart->hdmarx) != HAL_DMA_ERROR_NONE));
			}

			const bool isDmaLocked = (isRxActive && (isDmaRxError || (errorCode & HAL_UART_ERROR_DMA))) ||
					(isTxActive && (errorCode & HAL_UART_ERROR_DMA));

			// If the state is false or hung, restart the receiver and transmitter.
			if (uartState == HAL_UART_STATE_ERROR ||
					uartState == HAL_UART_STATE_TIMEOUT ||
					errorCode != HAL_UART_ERROR_NONE ||
					isDmaLocked)
			{
				++checkCounter;

				if(checkCounter > 15) {
					HAL_UART_Abort(m_huart);
					m_isTxBusy = false;
					receiveRestart();
					checkCounter = 0;
					isRestarted = true;
				}
			} else {
				checkCounter = 0;
			}
		}
	}


	/*
	 * Reception restart logic
	 */
	{
		// If reception is not started, restart it.
		if (!m_isStarted) {
			receiveRestart();
		}
	}

	return isRestarted;
}

template<UartType mode, reg BufferSize>
void UartEngine<mode, BufferSize>::onRxCplt(const u16 newPos)
{
	/*
	 * UART_IT mode section.
	 */
	if constexpr (mode == UartType::UART_IT) {

		bool saveMsg          = true;
		const u8* const rxPtr = m_rxRing.data();
		const u16 rxSize      = newPos;

		if (m_rxItHandler) {
			saveMsg = !m_rxItHandler(rxPtr, rxSize);
		}

		// Save message if the buffer is not full.
		if (saveMsg) {
			m_rxBuffer.put(rxPtr, rxSize);
		}
	}
	/*
	 * UART_DMA mode section.
	 */
	else if constexpr (mode == UartType::UART_DMA) {

		// HAL UART Rx Event types:
		// HAL_UART_RXEVENT_TC   = 0x00U,
		// HAL_UART_RXEVENT_HT   = 0x01U,
		// HAL_UART_RXEVENT_IDLE = 0x02U.
		const HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(m_huart);

		if (event == HAL_UART_RXEVENT_HT) {
			bool saveMsg          = true;
			const u8* const rxPtr = m_rxRing.data();
			const u16 rxSize      = newPos;

			if (m_rxItHandler) {
				saveMsg = !m_rxItHandler(rxPtr, rxSize);
			}

			// Save message if the buffer is not full.
			if (saveMsg) {
				m_rxBuffer.put(rxPtr, rxSize);
			}

			m_oldPos = newPos;

		} else if (event == HAL_UART_RXEVENT_TC || event == HAL_UART_RXEVENT_IDLE) {

			if (newPos != m_oldPos) {

				const u8* const rxData = m_rxRing.data();
				bool saveMsg           = true;
				const u8* const rxPtr  = rxData + m_oldPos;
				const u16 rxSize       = newPos - m_oldPos;

				if (m_rxItHandler) {
					saveMsg = !m_rxItHandler(rxPtr, rxSize);
				}

				// Save message if the buffer is not full.
				if (saveMsg) {
					m_rxBuffer.put(rxPtr, rxSize);
				}
			}

			m_oldPos = 0;
		}
	}
	/*
	 * UART_DMA_CIRCULAR mode section.
	 */
	else {

		if (newPos != m_oldPos) {

			const u8* const rxData = m_rxRing.data();

			if (newPos > m_oldPos) {

				bool saveMsg          = true;
				const u8* const rxPtr = rxData + m_oldPos;
				const u16 rxSize      = newPos - m_oldPos;

				if (m_rxItHandler) {
					saveMsg = !m_rxItHandler(rxPtr, rxSize);
				}

				// Save message if the buffer is not full.
				if (saveMsg) {
					m_rxBuffer.put(rxPtr, rxSize);
				}

			} else { // newPos < m_oldPos -> wrap-around.

				bool saveMsg          = true;
				const u8* const rxPtr = rxData + m_oldPos;
				const u16 elemToEnd   = BufferSize - m_oldPos;

				if (m_rxItHandler) {

					if (elemToEnd) {
						const bool saveMsg1 = !m_rxItHandler(rxPtr, elemToEnd);
						const bool saveMsg2 = !m_rxItHandler(rxData, newPos);
						saveMsg = saveMsg1 || saveMsg2;
					} else {
						saveMsg = !m_rxItHandler(rxData, newPos);
					}
				}

				// Save message if the buffer is not full.
				if (saveMsg) {
					if (elemToEnd) {
						m_rxBuffer.put(rxPtr, elemToEnd);
						m_rxBuffer.put(rxData, newPos);
					} else {
						m_rxBuffer.put(rxData, newPos);
					}
				}
			}
		}

		m_oldPos = newPos;
	}

	// Continue reception if needed.
	if constexpr (mode == UartType::UART_DMA) {
		m_isStarted = (HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rxRing.data(), BufferSize) == HAL_OK);
	} else if constexpr (mode == UartType::UART_IT) {
		m_isStarted = (HAL_UARTEx_ReceiveToIdle_IT(m_huart, m_rxRing.data(), BufferSize) == HAL_OK);
	}
}

template<UartType mode, reg BufferSize>
inline void UartEngine<mode, BufferSize>::onTxCplt()
{
	const HAL_UART_StateTypeDef gState = HAL_UART_GetState(m_huart);
	status_t status = STATUS_OK; // Assume success by default

	if constexpr (mode == UartType::UART_DMA || mode == UartType::UART_DMA_CIRCULAR) {
		const u32 dmaError = (m_huart->hdmatx) ?
				HAL_DMA_GetError(m_huart->hdmatx) : HAL_DMA_ERROR_NONE;

		if ((gState == HAL_UART_STATE_ERROR ||
				gState == HAL_UART_STATE_TIMEOUT) ||
				(dmaError != HAL_DMA_ERROR_NONE))
		{
			status = ERROR_FAIL;
		}
	} else {
		if (gState == HAL_UART_STATE_ERROR ||
				gState == HAL_UART_STATE_TIMEOUT)
		{
			status = ERROR_FAIL;
		}
	}

	if(m_txHandler) {
		m_txHandler(status);
	}

	if(status == ERROR_FAIL) {
		++error;
	}

	m_isTxBusy = false;
}


template<UartType mode, reg BufferSize>
void UartEngine<mode, BufferSize>::onErrorEvent()
{
	status_t status = ERROR_FAIL;
	const u32 ErrorCode = HAL_UART_GetError(m_huart);
	const u32 State = HAL_UART_GetState(m_huart);

	// read data registers for clear all pending bits
	{
		volatile reg sr = UART_SR_REG(m_huart);
		volatile reg dr = UART_DR_REG(m_huart);
		__HAL_UART_CLEAR_FLAG(m_huart, UART_ENGINE_RX_PART | HAL_UART_ERROR_DMA);
		m_huart->ErrorCode = HAL_UART_ERROR_NONE;
		(void)sr; (void)dr;
	}

	// critical errors
	if ((ErrorCode & HAL_UART_ERROR_DMA) ||
			State == HAL_UART_STATE_ERROR ||
			State == HAL_UART_STATE_TIMEOUT)
	{
		HAL_UART_Abort(m_huart);
		receiveRestart();
		status = (ErrorCode & HAL_UART_ERROR_DMA)
											? static_cast<status_t>(StatusUART::ERROR_UART_DMA_ERR)
													: static_cast<status_t>(StatusUART::ERROR_UART_STATE);
		m_isTxBusy = false;
	}
	// If there is an overflow, parity, framing or noise error – just restart reception
	else if (ErrorCode & UART_ENGINE_RX_PART) {
		// Special handling for Overrun Error
		if (ErrorCode & HAL_UART_ERROR_ORE) {
			__HAL_UART_CLEAR_OREFLAG(m_huart);
			status = static_cast<status_t>(StatusUART::ERROR_UART_ORE);
		} else {
			status = static_cast<status_t>(StatusUART::ERROR_UART_RX);
		}

		receiveRestart();
	}
	// If only transmission or transmission and reception, we do not change anything
	else if (State != HAL_UART_STATE_READY) {
		status = STATUS_BUSY;
	}

	if(m_errorHandler && status != STATUS_BUSY) {
		m_errorHandler(status);
	}

	++error;
}


template<UartType mode, reg BufferSize>
bool UartEngine<mode, BufferSize>::send(const u8 *const data, const u16 size)
{
	// For DMA or DMA_CIRCULAR modes, check the data size
	if constexpr (mode == UartType::UART_DMA || mode == UartType::UART_DMA_CIRCULAR) {

		static constexpr u16 DMA_THRESHOLD = 16; // Threshold for using DMA (in bytes)

		if (size > DMA_THRESHOLD) {
			if (HAL_UART_Transmit_DMA(m_huart, data, size) != HAL_OK) {
				return false;
			}
		} else {
			if (HAL_UART_Transmit_IT(m_huart, data, size) != HAL_OK) {
				return false;
			}
		}
	}
	// For IT we use interrupts
	else if constexpr (mode == UartType::UART_IT) {
		if (HAL_UART_Transmit_IT(m_huart, data, size) != HAL_OK) {
			return false;
		}
	}

	/* Set Tx Busy Status. */
	m_isTxBusy = true;
	return true;
}

template<UartType mode, reg BufferSize>
inline void UartEngine<mode, BufferSize>::receiveRestart()
{
	// Stop only data reception
	HAL_UART_AbortReceive(m_huart);

	// Start reception regardless of TX state
	if constexpr (mode == UartType::UART_DMA || mode == UartType::UART_DMA_CIRCULAR) {
		m_isStarted = (HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rxRing.data(), BufferSize) == HAL_OK);
	} else if constexpr (mode == UartType::UART_IT) {
		m_isStarted = (HAL_UARTEx_ReceiveToIdle_IT(m_huart, m_rxRing.data(), BufferSize) == HAL_OK);
	}

	// Reset buffer position
	m_oldPos = 0;
}

#endif /* UART_ENGINE_ON */
#endif /* if defined USART or UART */
#endif /* __STM32_UARTENGINE_H__ */
