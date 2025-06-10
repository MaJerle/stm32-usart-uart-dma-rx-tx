/*
 * UartBase.h
 *
 *  Created on: Mar 15, 2025
 *      Author: admin
 */

#ifndef STM32_TOOLS_UART_ENGINE_UARTBASE_H_
#define STM32_TOOLS_UART_ENGINE_UARTBASE_H_

#include "main.h"  // Must contain UART_HandleTypeDef definition

#if defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED)

#include "Uartsettings.h"

#if UART_ENGINE_ON

#include "basic_types.h"
#include <vector>

class UartBase {
protected:
	UartBase() = default;
	virtual ~UartBase();

	void attachUart(UART_HandleTypeDef* const huart);
private:
	// Friend declaration for the HAL UART event callbacks.
	friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
	friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
	friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

	// Called from HAL_UARTEx_RxEventCallback.
	// newPos - byte count position where DMA has written the data.
	virtual void onRxCplt(const u16 newPos) = 0;
	virtual inline void onTxCplt() = 0;
	virtual void onErrorEvent() = 0;

	static inline bool rxCpltHandler(const UART_HandleTypeDef* const huart, const u16 pos) {
		for(auto it : m_uarts) {
			if(it->matched(huart)) {
				it->onRxCplt(pos);
				return true;
			}
		}
		return false;
	}

	static inline bool txCpltHandler(const UART_HandleTypeDef* const huart) {
		for(auto it : m_uarts) {
			if(it->matched(huart)) {
				it->onTxCplt();
				return true;
			}
		}
		return false;
	}

	static inline bool errorHandler(const UART_HandleTypeDef* const huart) {
		for(auto it : m_uarts) {
			if(it->matched(huart)) {
				it->onErrorEvent();
				return true;
			}
		}
		return false;
	}

public:
	inline bool matched(const UART_HandleTypeDef* const huart) {
		return m_huart == huart;
	}

	inline UART_HandleTypeDef* const instance() { return m_huart; }
	static inline void setMaximumInstances(const u8 n) {
		if (n == 0) {
			return;
		}
		m_uarts.reserve(n);
	}

protected:
	UART_HandleTypeDef* m_huart = nullptr;
private:
	static inline std::vector<UartBase*> m_uarts = {};
};
#else
#	warning "[UART ENGINE]: UART ENGINE is disabled"
#endif /* UART_ENGINE_ON */
#endif /* if defined USART or UART  */
#endif /* STM32_TOOLS_UART_ENGINE_UARTBASE_H_ */
