/**
  ******************************************************************************
  * @file    stm32c5xx_hal_smartcard.c
  * @brief   SMARTCARD HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the SMARTCARD peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Error functions.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) \
 || defined(UART7)
#if defined(USE_HAL_SMARTCARD_MODULE) && (USE_HAL_SMARTCARD_MODULE == 1)
/** @addtogroup SMARTCARD SMARTCARD
  * @{
  */
/** @defgroup SMARTCARD_Introduction SMARTCARD Introduction
  * @{

  - The SMARTCARD hardware abstraction layer provides a set of APIs to interface with the STM32 **USART** (Universal
    Synchronous/Asynchronous Receiver Transmitter) peripheral to support the T = 0 and T = 1 asynchronous protocols
    for smartcards as defined in the ISO/IEC 7816-3 standard.

  - It simplifies the configuration, initialization, and management of SMARTCARD communication by supporting
    various modes such as polling, interrupt, and DMA for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series.

  */
/**
  * @}
  */

/** @defgroup SMARTCARD_How_To_Use SMARTCARD How To Use
  * @{

# How to use the SMARTCARD HAL module driver

The SMARTCARD HAL driver can be used as follows:

1. Declare a hal_smartcard_handle_t handle structure, for example:
   hal_smartcard_handle_t hsmartcard;

2. Initialize the SMARTCARDx driver with a USART HW instance by calling the HAL_SMARTCARD_Init().
   The SMARTCARDx clock is enabled inside the HAL_SMARTCARD_Init() if USE_HAL_SMARTCARD_CLK_ENABLE_MODEL >
   HAL_CLK_ENABLE_NO.

3. Configure the low-level hardware (GPIO, CLOCK, NVIC, etc.):
    - Enable the SMARTCARDx interface clock when USE_HAL_SMARTCARD_CLK_ENABLE_MODEL is set to HAL_CLK_ENABLE_NO.
    - Configure SMARTCARDx pins:
        - Enable the clock for the SMARTCARDx GPIOs
        - Configure SMARTCARDx pins as alternate-function open-drain
    - Configure the NVIC for interrupt processing:
        - Configure the SMARTCARDx interrupt priority
        - Enable the NVIC SMARTCARDx IRQ Channel

4. Configure the communication baud rate, stop bit, first bit, parity mode, NACK, smartcard clock prescaler,
   source clock prescaler, clock polarity, clock phase, clock output enabling, guard time and auto retry count by
   calling HAL_SMARTCARD_SetConfig().

  @note In SMARTCARD mode, ETU (Elementary Time Unit) is equivalent to the baud period duration

  Configure or enable advanced features:
    - HAL_SMARTCARD_EnableIOInvert() to invert the IO pin active level logic
    - HAL_SMARTCARD_EnableDataInvert() to invert the binary data logic
    - HAL_SMARTCARD_EnableTxRxSwap() to change the GPIO used (USART Tx by default)
    - HAL_SMARTCARD_EnableRxOverRunDetection() to detect Rx Overrun errors
    - HAL_SMARTCARD_EnableDMAStopOnRxError() to stop DMA on Rx error
    - HAL_SMARTCARD_SetReceiverTimeout() to set the Rx timeout value
    - HAL_SMARTCARD_EnableReceiverTimeout() to detect Rx timeout
    - HAL_SMARTCARD_SetTxCpltIndication() to change the Tx complete indication
    - HAL_SMARTCARD_EnableFifoMode() to change the FIFO mode status
    - HAL_SMARTCARD_SetTxFifoThreshold() to set the Tx FIFO threshold
    - HAL_SMARTCARD_SetRxFifoThreshold() to set the Rx FIFO threshold
    - HAL_SMARTCARD_SetBlockLength() to set the block length (in bytes)

  All these advanced configurations are optional. If not called, default values apply.

5. For SMARTCARDx I/O operations, polling, interrupt, and DMA are available in this driver.

  - Polling-mode I/O operation
    - Send an amount of data in blocking mode using HAL_SMARTCARD_Transmit()
    - Receive an amount of data in blocking mode using HAL_SMARTCARD_Receive()
    - The communication is performed in polling mode. The HAL status of all data processing is returned by the same
      function after finishing transfer.

  - Interrupt-mode I/O operation
    - Send an amount of data in non-blocking mode using HAL_SMARTCARD_Transmit_IT()
    - At the end of transmission, execute HAL_SMARTCARD_TxCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - Receive an amount of data in non-blocking mode using HAL_SMARTCARD_Receive_IT()
    - At the end of reception, execute HAL_SMARTCARD_RxCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - On transfer error, execute HAL_SMARTCARD_ErrorCallback().
      Customize the associated function pointer to add application-specific code.

  - DMA-mode I/O operation
    - Send an amount of data in non-blocking mode (DMA) using HAL_SMARTCARD_Transmit_DMA()
    - At the half-transfer point, execute HAL_SMARTCARD_TxHalfCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - At the end of transmission, execute HAL_SMARTCARD_TxCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - Receive an amount of data in non-blocking mode (DMA) using HAL_SMARTCARD_Receive_DMA()
    - At the half-transfer point, execute HAL_SMARTCARD_RxHalfCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - At the end of reception, execute HAL_SMARTCARD_RxCpltCallback().
      Customize the associated function pointer to add application-specific code.
    - On transfer error, execute HAL_SMARTCARD_ErrorCallback().
      Customize the associated function pointer to add application-specific code.

  - The following sequential SMARTCARD interfaces are available:
    - Abort a polling SMARTCARD communication process using HAL_SMARTCARD_Abort().
    - Abort an IT SMARTCARD communication process using interrupts with HAL_SMARTCARD_Abort_IT().
    - At the end of the abort IT process, execute HAL_SMARTCARD_AbortCpltCallback().
      Customize the associated function pointer to add application-specific code.

6. Callback registration
  - When the compilation flag USE_HAL_SMARTCARD_REGISTER_CALLBACKS is set to 1, it allows the user to configure
    the driver callbacks dynamically via its own method:

    Callback name             | Default value                       | Callback registration function
  ----------------------------| ----------------------------------- | ---------------------------
  TxHalfCpltCallback          | HAL_SMARTCARD_TxHalfCpltCallback()  | HAL_SMARTCARD_RegisterTxHalfCpltCallback()
  TxCpltCallback              | HAL_SMARTCARD_TxCpltCallback()      | HAL_SMARTCARD_RegisterTxCpltCallback()
  RxHalfCpltCallback          | HAL_SMARTCARD_RxHalfCpltCallback()  | HAL_SMARTCARD_RegisterRxHalfCpltCallback()
  RxCpltCallback              | HAL_SMARTCARD_RxCpltCallback()      | HAL_SMARTCARD_RegisterRxCpltCallback()
  ErrorCallback               | HAL_SMARTCARD_ErrorCallback()       | HAL_SMARTCARD_RegisterErrorCallback()
  AbortCpltCallback           | HAL_SMARTCARD_AbortCpltCallback()   | HAL_SMARTCARD_RegisterAbortCpltCallback()
  RxFifoFullCallback          | HAL_SMARTCARD_RxFifoFullCallback()  | HAL_SMARTCARD_RegisterRxFifoFullCallback()
  TxFifoEmptyCallback         | HAL_SMARTCARD_TxFifoEmptyCallback() | HAL_SMARTCARD_RegisterTxFifoEmptyCallback()

  - To unregister a callback, register the default callback via the registration function.

  - By default, after the HAL_SMARTCARD_Init() and when the state is HAL_SMARTCARD_STATE_INIT, all callbacks are set to
    the corresponding default weak functions.

  - Callbacks can be registered in the handle global_state HAL_SMARTCARD_STATE_INIT and HAL_SMARTCARD_STATE_CONFIGURED.

  - When the compilation flag USE_HAL_SMARTCARD_REGISTER_CALLBACKS is set to 0 or not defined, the callback registration
    feature is not available and all callbacks are set to the corresponding weak functions.

7. Acquire/Release the HAL SMARTCARD handle
  - When the compilation flag USE_HAL_MUTEX is set to 1, a multi-threaded user application is allowed to acquire the
    SMARTCARD HAL handle to execute a transmit or receive process, or a sequence of transmit/receive operations.
    Release the SMARTCARD HAL handle when the process or sequence ends.
  - HAL acquire/release operations are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
     - HAL_SMARTCARD_AcquireBus() allows acquiring the HAL SMARTCARD handle.
     - HAL_SMARTCARD_ReleaseBus() allows releasing the HAL SMARTCARD handle.

  - When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_SMARTCARD_AcquireBus() and
    HAL_SMARTCARD_ReleaseBus() are not available.
  */
/**
  * @}
  */

/** @defgroup SMARTCARD_Configuration_Table SMARTCARD Configuration Table
  * @{
## Configuration inside the SMARTCARD driver

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags                   |   Default value   | Comment
------------------------------------ | ----------------- | ------------------------------------------------
USE_HAL_SMARTCARD_MODULE             |         1         | Enable HAL SMARTCARD driver module
USE_HAL_SMARTCARD_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_SMARTCARD_DMA                |         1         | Enable DMA code inside SMARTCARD
USE_HAL_CHECK_PARAM                  |         0         | Enable runtime parameter checks
USE_HAL_SMARTCARD_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE          |         0         | Enable atomicity of process state checks
USE_HAL_MUTEX                        |         0         | Enable semaphore creation for OS
USE_HAL_SMARTCARD_USER_DATA          |         0         | Add user data inside the HAL SMARTCARD handle
USE_HAL_SMARTCARD_GET_LAST_ERRORS    |         0         | Enable retrieval of last process error codes
USE_HAL_SMARTCARD_FIFO               |         1         | Enable the FIFO feature

Software configuration defined in preprocessor environment:
Preprocessor flags                   |   Default value   | Comment
------------------------------------ | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM                 |    Not defined    | Enable parameter checks for HAL and LL
USE_ASSERT_DBG_STATE                 |    Not defined    | Enable state checks for HAL

  */
/**
  * @}
  */
/* Private define ------------------------------------------------------------*/
/** @defgroup SMARTCARD_Private_Constants SMARTCARD Private Constants
  * @{
  */
#define USART_BRR_MIN 0x10U         /*!< USART BRR minimum authorized value */

#define USART_BRR_MAX 0xFFFFU       /*!< USART BRR maximum authorized value */

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
#define RX_FIFO_DEPTH 8U           /*!< SMARTCARD RX FIFO depth */

#define TX_FIFO_DEPTH 8U           /*!< SMARTCARD TX FIFO depth */
#endif /* USE_HAL_SMARTCARD_FIFO */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SMARTCARD_Private_Macros SMARTCARD Private Macros
  * @{
  */

/** @brief  Check SMARTCARD baud rate.
  * @param  baud_rate baud rate specified by the user.
  *         The maximum baud rate is derived from the maximum clock on C5 (144 MHz)
  *         divided by the oversampling method used on the USART (that is, 16)
  * @retval 1U (baud_rate is valid) or 0U (baud_rate is invalid)
  */
#define IS_SMARTCARD_BAUD_RATE(baud_rate) ((baud_rate) <= 9000000U && ((baud_rate) != 0U))

/** @brief  Check SMARTCARD Receiver Timeout value.
  * @param  timeout_etu Timeout value.
  * @retval 1U (timeout_etu is valid) or 0U (timeout_etu is invalid)
  */
#define IS_SMARTCARD_TIMEOUT_VALUE(timeout_etu) ((timeout_etu) <= 0xFFFFFFU)

/**
  * @brief Ensure that SMARTCARD frame number of stop bits is valid.
  * @param stopbits SMARTCARD frame number of stop bits.
  * @retval 1U (stopbits is valid) or 0U (stopbits is invalid)
  */
#define IS_SMARTCARD_STOP_BITS(stopbits) (((stopbits) == HAL_SMARTCARD_STOP_BIT_0_5) \
                                          || ((stopbits) == HAL_SMARTCARD_STOP_BIT_1_5))

/**
  * @brief Ensure that SMARTCARD first bit sent (MSB or LSB) is valid.
  * @param first_bit SMARTCARD first bit sent (MSB or LSB) parameter.
  * @retval 1U (first_bit is valid) or 0U (first_bit is invalid)
  */
#define IS_SMARTCARD_FIRST_BIT(first_bit) (((first_bit) == HAL_SMARTCARD_BIT_ORDER_LSB_FIRST) \
                                           || ((first_bit) == HAL_SMARTCARD_BIT_ORDER_MSB_FIRST))

/**
  * @brief Ensure that SMARTCARD nack management setting is valid.
  * @param nack SMARTCARD nack management setting.
  * @retval 1U (nack is valid) or 0U (nack is invalid)
  */
#define IS_SMARTCARD_NACK(nack) (((nack) == HAL_SMARTCARD_NACK_DISABLE) \
                                 || ((nack) == HAL_SMARTCARD_NACK_ENABLE))

/**
  * @brief Ensure that SMARTCARD clock output is valid.
  * @param clockoutput SMARTCARD clock output.
  * @retval 1U (clockoutput is valid) or 0U (clockoutput is invalid)
  */
#define IS_SMARTCARD_CLOCK_OUTPUT(clockoutput) (((clockoutput) == HAL_SMARTCARD_CLOCK_OUTPUT_ENABLE) \
                                                || ((clockoutput) == HAL_SMARTCARD_CLOCK_OUTPUT_DISABLE))

/**
  * @brief Ensure that SMARTCARD clock polarity is valid.
  * @param polarity SMARTCARD clock polarity.
  * @retval 1U (polarity is valid) or 0U (polarity is invalid)
  */
#define IS_SMARTCARD_CLOCK_POLARITY(polarity) (((polarity) == HAL_SMARTCARD_CLOCK_POLARITY_HIGH) \
                                               || ((polarity) == HAL_SMARTCARD_CLOCK_POLARITY_LOW))

/**
  * @brief Ensure that SMARTCARD clock phase is valid.
  * @param clock_phase SMARTCARD clock phase.
  * @retval 1U (clock_phase is valid) or 0U (clock_phase is invalid)
  */
#define IS_SMARTCARD_CLOCK_PHASE(clock_phase) (((clock_phase) == HAL_SMARTCARD_CLOCK_PHASE_1_EDGE) \
                                               || ((clock_phase) == HAL_SMARTCARD_CLOCK_PHASE_2_EDGE))

/**
  * @brief Ensure that SMARTCARD frame parity is valid.
  * @param parity SMARTCARD frame parity.
  * @retval 1U (parity is valid) or 0U (parity is invalid)
  */
#define IS_SMARTCARD_PARITY(parity) (((parity) == HAL_SMARTCARD_PARITY_EVEN) \
                                     || ((parity) == HAL_SMARTCARD_PARITY_ODD))

/**
  * @brief Ensure that SMARTCARD retry count is valid.
  * @param retry_count SMARTCARD retry count.
  * @retval 1U (retry count is valid) or 0U (retry count is invalid)
  */
#define IS_SMARTCARD_RETRY_COUNT(retry_count) (((retry_count) <= 7UL))

/**
  * @brief Ensure that SMARTCARD guard time is valid.
  * @param guard_time_etu SMARTCARD guard time (Elementary Time Unit).
  * @retval 1U (guard time is valid) or 0U (guard time is invalid)
  */
#define IS_SMARTCARD_GUARD_TIME(guard_time_etu) ((guard_time_etu) <= 0xFFU)

/**
  * @brief Ensure that SMARTCARD Tx complete indication is valid.
  * @param indication SMARTCARD frame Tx complete indication.
  * @retval 1U (indication is valid) or 0U (indication is invalid)
  */
#define IS_SMARTCARD_TX_CPLT(indication) (((indication) == HAL_SMARTCARD_TX_CPLT_BEFORE_GUARD_TIME) \
                                          || ((indication) == HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME))

/**
  * @brief Ensure that SMARTCARD Prescaler is valid.
  * @param clock_prescaler SMARTCARD Prescaler value.
  * @retval 1U (clock_prescaler is valid) or 0U (clock_prescaler is invalid)
  */
#define IS_SMARTCARD_CLOCK_PRESCALER(clock_prescaler) ((clock_prescaler) <= HAL_SMARTCARD_CLOCK_PRESC_DIV256)

/**
  * @brief Ensure that SMARTCARD block length is valid.
  * @param block_length_byte SMARTCARD block length value.
  * @retval 1U (block_length_byte is valid) or 0U (block_length_byte is invalid)
  */
#define IS_SMARTCARD_BLOCK_LENGTH(block_length_byte) ((block_length_byte) <= 0xFFUL)

/**
  * @brief Ensure that SMARTCARD clock Prescaler is valid.
  * @param sclk_prescaler SMARTCARD Prescaler value.
  * @retval 1U (sclk_prescaler is valid) or 0U (sclk_prescaler is invalid)
  */
#define IS_SMARTCARD_SCLK_PRESCALER(sclk_prescaler) (((sclk_prescaler) >= HAL_SMARTCARD_SCLK_PRESC_DIV2)       \
                                                     && ((sclk_prescaler) <= HAL_SMARTCARD_SCLK_PRESC_DIV62))

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)

/**
  * @brief Ensure that SMARTCARD FIFO threshold level is valid.
  * @param threshold SMARTCARD FIFO threshold level.
  * @retval 1U (threshold is valid) or 0U (threshold is invalid)
  */
#define IS_SMARTCARD_FIFO_THRESHOLD(threshold) (((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_1_8)         \
                                                || ((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_1_4)      \
                                                || ((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_1_2)      \
                                                || ((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_3_4)      \
                                                || ((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_7_8)      \
                                                || ((threshold) == HAL_SMARTCARD_FIFO_THRESHOLD_8_8))

/**
  * @brief Ensure that SMARTCARD Optional Interrupts for IT in Transmit is valid.
  * @param interrupt SMARTCARD Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */

#define IS_SMARTCARD_OPT_TX_IT(interrupt)  (((interrupt) == HAL_SMARTCARD_OPT_TX_IT_NONE)                 \
                                            || ((interrupt) == HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY)        \
                                            || ((interrupt) == HAL_SMARTCARD_OPT_TX_IT_DEFAULT))

/**
  * @brief Ensure that SMARTCARD Optional Interrupts for IT in Receive is valid.
  * @param interrupt SMARTCARD Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#define IS_SMARTCARD_OPT_RX_IT(interrupt)    (((interrupt) == HAL_SMARTCARD_OPT_RX_IT_NONE)               \
                                              || ((interrupt) == HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL)       \
                                              || ((interrupt) == HAL_SMARTCARD_OPT_RX_IT_DEFAULT))
#endif /* USE_HAL_SMARTCARD_FIFO */

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/**
  * @brief Ensure that SMARTCARD Optional Interrupts for DMA in Receive is valid.
  * @param interrupt SMARTCARD Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */

#define IS_SMARTCARD_OPT_RX_DMA(interrupt)    (((interrupt) == HAL_SMARTCARD_OPT_DMA_RX_IT_NONE)          \
                                               || ((interrupt) == HAL_SMARTCARD_OPT_DMA_RX_IT_HT)         \
                                               || ((interrupt) == HAL_SMARTCARD_OPT_DMA_RX_IT_DEFAULT))

/**
  * @brief Ensure that SMARTCARD Optional Interrupts for DMA in Transmit is valid.
  * @param interrupt SMARTCARD Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */

#define IS_SMARTCARD_OPT_TX_DMA(interrupt)  (((interrupt) == HAL_SMARTCARD_OPT_DMA_TX_IT_NONE)            \
                                             || ((interrupt) == HAL_SMARTCARD_OPT_DMA_TX_IT_HT)           \
                                             || ((interrupt) == HAL_SMARTCARD_OPT_DMA_TX_IT_DEFAULT))

#endif /* USE_HAL_SMARTCARD_DMA */
/**
  * @brief Check if USART instance is enabled. If yes, disable it.
  * @param handle specifies the USART Handle
  */
#define SMARTCARD_ENSURE_INSTANCE_DISABLED(handle)           \
  uint32_t instance_enabled;                                 \
  do                                                         \
  {                                                          \
    instance_enabled = LL_USART_IsEnabled(handle);           \
    if (instance_enabled != 0U)                              \
    {                                                        \
      LL_USART_Disable(handle);                              \
    }                                                        \
  }  while(0U)

/**
  * @brief Check if USART instance needs to be re-enabled.
  * @param handle specifies the USART Handle
  */
#define SMARTCARD_ENSURE_INSTANCE_ENABLED(handle)            \
  do                                                         \
  {                                                          \
    if (instance_enabled != 0U)                              \
    {                                                        \
      LL_USART_Enable(handle);                               \
    }                                                        \
  }  while(0U)

/** @brief  Return the transmission completion flag.
  * @param  handle specifies the SMARTCARD Handle.
  * @note   Based on tx_cplt_indication setting, return TC or TCBGT flag.
  *         When TCBGT flag (Transmission Complete Before Guard Time) is not available, TC flag is
  *         reported.
  * @retval Transmission completion flag
  */
#define SMARTCARD_TRANSMISSION_COMPLETION_FLAG(handle) \
  (((handle)->tx_cplt_indication == HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME) ? (LL_USART_ISR_TC) : (LL_USART_ISR_TCBGT))

/**
  * @brief Retrieve SMARTCARD instance from handle.
  * @param handle specifies the USART Handle
  */
#define SMARTCARD_GET_INSTANCE(handle)   ((USART_TypeDef *)((uint32_t)(handle)->instance))

/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/** @defgroup SMARTCARD_Private_Functions SMARTCARD Private Functions
  * @{
  */
static void SMARTCARD_Abort(hal_smartcard_handle_t *hsmartcard);
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
static void SMARTCARD_InitCallbacksToDefault(hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
static hal_status_t SMARTCARD_WaitOnFlagUntilTimeout(hal_smartcard_handle_t *hsmartcard, uint32_t flag,
                                                     uint32_t status, uint32_t tickstart, uint32_t timeout_ms);
static void SMARTCARD_EndTxTransfer(hal_smartcard_handle_t *hsmartcard);
static void SMARTCARD_EndRxTransfer(hal_smartcard_handle_t *hsmartcard);
hal_status_t SMARTCARD_Start_Transmit_IT(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint32_t size,
                                         uint32_t interrupts);
hal_status_t SMARTCARD_Start_Receive_IT(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint32_t size,
                                        uint32_t interrupts);
#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
hal_status_t SMARTCARD_Start_Transmit_DMA(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint32_t size,
                                          uint32_t interrupts);
hal_status_t SMARTCARD_Start_Receive_DMA(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint32_t size,
                                         uint32_t interrupts);
static void SMARTCARD_DMATxHalfCplt(hal_dma_handle_t *hdma);
static void SMARTCARD_DMATransmitCplt(hal_dma_handle_t *hdma);
static void SMARTCARD_DMARxHalfCplt(hal_dma_handle_t *hdma);
static void SMARTCARD_DMAReceiveCplt(hal_dma_handle_t *hdma);
static void SMARTCARD_DMAError(hal_dma_handle_t *hdma);
static void SMARTCARD_DMAAbortOnError(hal_dma_handle_t *hdma);
static void SMARTCARD_DMATxAbortCallback(hal_dma_handle_t *hdma);
static void SMARTCARD_DMARxAbortCallback(hal_dma_handle_t *hdma);
#endif /* USE_HAL_SMARTCARD_DMA */
static void SMARTCARD_TxISR(hal_smartcard_handle_t *hsmartcard);
static void SMARTCARD_EndTransmit_IT(hal_smartcard_handle_t *hsmartcard);
static void SMARTCARD_RxISR(hal_smartcard_handle_t *hsmartcard);
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
static void SMARTCARD_TxISR_FIFOEN(hal_smartcard_handle_t *hsmartcard);
static void SMARTCARD_RxISR_FIFOEN(hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_FIFO */
#if defined(USE_HAL_SMARTCARD_CLK_ENABLE_MODEL) && (USE_HAL_SMARTCARD_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
static void SMARTCARD_EnableClock(const hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_CLK_ENABLE_MODEL */

static void SMARTCARD_FLUSH_DRREGISTER(hal_smartcard_handle_t *hsmartcard);
#if defined(USE_ASSERT_DBG_PARAM)
hal_status_t SMARTCARD_Check_uart_baudrate_validity(uint32_t instance_clock_freq, uint32_t instance_clock_prescaler,
                                                    uint32_t baud_rate);
#endif /* USE_ASSERT_DBG_PARAM */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup SMARTCARD_Exported_Functions
  * @{
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  This subsection provides a set of functions allowing to initialize and de-initialize the USARTx peripheral:
  - Call the function HAL_SMARTCARD_Init() to initialize the selected HAL_SMARTCARD handle and associate an instance.
  - Call the function HAL_SMARTCARD_DeInit() to restore the default initialization of the selected USARTx peripheral.
  */

/**
  * @brief Initialize the SMARTCARD according to the associated handle.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *               the configuration information for SMARTCARD module.
  * @param  instance SMARTCARD instance.
  * @retval HAL_INVALID_PARAM When the handle is NULL.
  * @retval HAL_ERROR When the MUTEX cannot be created.
  * @retval HAL_OK HAL SMARTCARD driver correctly Initialized for the given SMARTCARD instance.
  */
hal_status_t HAL_SMARTCARD_Init(hal_smartcard_handle_t *hsmartcard, hal_smartcard_t instance)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(((IS_SMARTCARD_INSTANCE((USART_TypeDef *)((uint32_t)instance)))));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check input parameters */
  if (hsmartcard == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->instance      = instance;
  hsmartcard->tx_xfer_size  = 0;
  hsmartcard->rx_xfer_size  = 0;
  hsmartcard->tx_xfer_count = 0;
  hsmartcard->rx_xfer_count = 0;

#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  SMARTCARD_InitCallbacksToDefault(hsmartcard);
#endif /* (USE_HAL_SMARTCARD_REGISTER_CALLBACKS) */

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /* Initialize the number of data to process during RX/TX ISR execution */
  hsmartcard->nb_tx_data_to_process = 1;
  hsmartcard->nb_rx_data_to_process = 1;
  hsmartcard->fifo_status = HAL_SMARTCARD_FIFO_MODE_DISABLED;
#endif /* USE_HAL_SMARTCARD_FIFO */

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
  hsmartcard->hdma_tx = (hal_dma_handle_t *)NULL;
  hsmartcard->hdma_rx = (hal_dma_handle_t *)NULL;
#endif /* USE_HAL_SMARTCARD_DMA */

#if defined (USE_HAL_SMARTCARD_USER_DATA) && (USE_HAL_SMARTCARD_USER_DATA == 1)
  /* Reset the user data pointer to NULL */
  hsmartcard->p_user_data = NULL;
#endif /* USE_HAL_SMARTCARD_USER_DATA */

#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = 0;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */

#if defined(USE_HAL_SMARTCARD_CLK_ENABLE_MODEL) && (USE_HAL_SMARTCARD_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  SMARTCARD_EnableClock(hsmartcard);
#endif /* USE_HAL_SMARTCARD_CLK_ENABLE_MODEL */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Create the SMARTCARD semaphore */
  if (HAL_OS_SemaphoreCreate(&hsmartcard->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  hsmartcard->global_state = HAL_SMARTCARD_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief De-Initialize the HAL SMARTCARD driver for the given handle.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                   the configuration information for SMARTCARD module.
  * @retval HAL_OK HAL SMARTCARD driver correctly deinitialized for the given SMARTCARD handle.
  */
hal_status_t HAL_SMARTCARD_DeInit(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  ASSERT_DBG_PARAM(IS_SMARTCARD_INSTANCE(p_smartcardx));

  const hal_smartcard_state_t temp_state = hsmartcard->global_state;
  /* Check if any transfer ongoing */
  if ((temp_state == HAL_SMARTCARD_STATE_RX_ACTIVE) || (temp_state == HAL_SMARTCARD_STATE_TX_ACTIVE))
  {
    /* Stop current process/operation(s) */
    SMARTCARD_Abort(hsmartcard);

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
    if (hsmartcard->hdma_tx != NULL)
    {
      if (LL_USART_IsEnabledDMAReq_TX(p_smartcardx) != 0U)
      {
        LL_USART_DisableDMAReq_TX(p_smartcardx);

        /* No call back execution at end of DMA abort procedure */
        (void)HAL_DMA_Abort(hsmartcard->hdma_tx);
      }
    }
    if (hsmartcard->hdma_rx != NULL)
    {
      LL_USART_DisableDMAReq_RX(p_smartcardx);

      /* Abort the SMARTCARD DMA Rx channel : use blocking DMA Abort API (no callback) */
      if (LL_USART_IsEnabledDMAReq_RX(p_smartcardx) != 0U)
      {
        (void)HAL_DMA_Abort(hsmartcard->hdma_rx);
      }
    }
#endif /* USE_HAL_SMARTCARD_DMA */

    /* Clear the Error flags in the ICR register */
    LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF
                       | LL_USART_ICR_FECF | LL_USART_ICR_RTOCF | LL_USART_ICR_EOBCF);
  }

  LL_USART_Disable(p_smartcardx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Delete the SMARTCARD semaphore */
  (void)HAL_OS_SemaphoreDelete(&hsmartcard->semaphore);
#endif /* USE_HAL_MUTEX */

  /* Reset the global state */
  hsmartcard->global_state = HAL_SMARTCARD_STATE_RESET;

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group2 General Config functions
  * @{
  This subsection provides a set of functions allowing configuration of the USARTx peripheral in SMARTCARD mode:

  - Global configuration:
    - HAL_SMARTCARD_SetConfig(), set the minimum required configuration into the handler instance registers.
    - HAL_SMARTCARD_GetConfig(), fetch the minimum required configuration from the handler instance registers.
  */
/**
  * @brief  Configure the SMARTCARD according to user parameters in the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_config Pointer to the configuration structure
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_SetConfig(hal_smartcard_handle_t *hsmartcard, const hal_smartcard_config_t *p_config)
{
  USART_TypeDef *p_smartcardx;
  uint32_t instance_clock_freq;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_BAUD_RATE(p_config->baud_rate));
  ASSERT_DBG_PARAM(IS_SMARTCARD_STOP_BITS(p_config->stop_bits));
  ASSERT_DBG_PARAM(IS_SMARTCARD_FIRST_BIT(p_config->first_bit));
  ASSERT_DBG_PARAM(IS_SMARTCARD_PARITY(p_config->parity));
  ASSERT_DBG_PARAM(IS_SMARTCARD_NACK(p_config->nack));
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_PRESCALER(p_config->clock_prescaler));
  ASSERT_DBG_PARAM(IS_SMARTCARD_SCLK_PRESCALER(p_config->sclk_prescaler));
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_OUTPUT(p_config->clock_output));
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_POLARITY(p_config->clock_polarity));
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_PHASE(p_config->clock_phase));
  ASSERT_DBG_PARAM(IS_SMARTCARD_GUARD_TIME(p_config->guard_time_etu));
  ASSERT_DBG_PARAM(IS_SMARTCARD_RETRY_COUNT(p_config->auto_retry_count));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_INIT | (uint32_t)HAL_SMARTCARD_STATE_IDLE));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetPrescaler(p_smartcardx, (uint32_t)p_config->clock_prescaler);

  LL_USART_SetSmartcardGuardTime(p_smartcardx, p_config->guard_time_etu);
  LL_USART_SetSmartcardPrescaler(p_smartcardx, (uint32_t)p_config->sclk_prescaler);

  LL_USART_SetSmartcardConfig(p_smartcardx, (uint32_t)p_config->stop_bits, (uint32_t)p_config->first_bit,
                              (uint32_t)p_config->parity, (uint32_t)p_config->nack, p_config->auto_retry_count);
  LL_USART_SetSmartcardClockConfig(p_smartcardx, (uint32_t)p_config->clock_output, (uint32_t)p_config->clock_polarity,
                                   (uint32_t)p_config->clock_phase);

  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_smartcardx);
  ASSERT_DBG_PARAM(instance_clock_freq != 0U);

  ASSERT_DBG_PARAM(SMARTCARD_Check_uart_baudrate_validity(instance_clock_freq, p_config->clock_prescaler,
                                                          p_config->baud_rate) == HAL_OK);
  LL_USART_SetBaudRate(p_smartcardx, instance_clock_freq, (uint32_t)p_config->clock_prescaler, LL_USART_OVERSAMPLING_16,
                       p_config->baud_rate);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the SMARTCARD configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_config Pointer to the configuration structure
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMARTCARD_GetConfig(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_config_t *p_config)
{
  USART_TypeDef *p_smartcardx;
  uint32_t reg_temp;
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  p_config->parity = (hal_smartcard_parity_t)LL_USART_GetParity(p_smartcardx);
  reg_temp = LL_USART_READ_REG(p_smartcardx, CR2);
  p_config->first_bit = (hal_smartcard_bit_order_t)(uint32_t)(reg_temp & (USART_CR2_MSBFIRST));
  p_config->stop_bits = (hal_smartcard_stop_bits_t)(uint32_t)(reg_temp & (USART_CR2_STOP));
  p_config->clock_output = (hal_smartcard_clock_output_t)(uint32_t)(reg_temp & (USART_CR2_CLKEN));
  p_config->clock_phase = (hal_smartcard_clock_phase_t)(uint32_t)(reg_temp & (USART_CR2_CPHA));
  p_config->clock_polarity = (hal_smartcard_clock_polarity_t)(uint32_t)(reg_temp & (USART_CR2_CPOL));
  reg_temp = LL_USART_READ_REG(p_smartcardx, CR3);
  p_config->auto_retry_count = (uint32_t)((reg_temp & (USART_CR3_SCARCNT)) >> USART_CR3_SCARCNT_Pos);
  if (((reg_temp & (uint32_t)USART_CR3_NACK)) != 0U)
  {
    p_config->nack = HAL_SMARTCARD_NACK_ENABLE;
  }
  else
  {
    p_config->nack = HAL_SMARTCARD_NACK_DISABLE;
  }
  reg_temp = LL_USART_READ_REG(p_smartcardx, GTPR);
  p_config->guard_time_etu = (uint32_t)((reg_temp & (USART_GTPR_GT)) >> USART_GTPR_GT_Pos);
  p_config->sclk_prescaler = (hal_smartcard_source_clock_prescaler_t)(uint32_t)(reg_temp & (USART_GTPR_PSC));
  uint32_t instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_smartcardx);

  p_config->clock_prescaler = (hal_smartcard_prescaler_t)LL_USART_GetPrescaler(p_smartcardx);
  p_config->baud_rate = LL_USART_GetBaudRate(p_smartcardx, instance_clock_freq, (uint32_t)p_config->clock_prescaler,
                                             LL_USART_OVERSAMPLING_16);
  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group3 Unitary basic config functions
  * @{
  This subsection provides a set of unitary functions allowing to configure the USARTx peripheral in SMARTCARD mode:

  - Unitary configuration :
    - HAL_SMARTCARD_SetBaudRate(), set the baud rate value into the handler instance registers.
    - HAL_SMARTCARD_GetBaudRate(), fetch the baud rate value from the handler instance registers.
    - HAL_SMARTCARD_SetStopBits(), set the stop bits value into the handler instance registers.
    - HAL_SMARTCARD_GetStopBits(), fetch the stop bits value from the handler instance registers.
    - HAL_SMARTCARD_SetFirstBit(), set the first bit sent (MSB or LSB) value into the handler instance registers.
    - HAL_SMARTCARD_GetFirstBit(), fetch the first bit sent (MSB or LSB) value from the handler instance registers.
    - HAL_SMARTCARD_SetParity(), set the parity value into the handler instance registers.
    - HAL_SMARTCARD_GetParity(), fetch the parity value from the handler instance registers.
    - HAL_SMARTCARD_SetNack(), set the nack value (NACK transmission is enabled/disabled in case of parity error)
      into the handler instance registers.
    - HAL_SMARTCARD_GetNack(), fetch the nack value from the handler instance registers.
    - HAL_SMARTCARD_SetClockOutput(), set the clock output value into the handler instance registers.
    - HAL_SMARTCARD_GetClockOutput(), fetch the clock output value from the handler instance registers.
    - HAL_SMARTCARD_SetClockPolarity(), set the clock polarity value into the handler instance registers.
    - HAL_SMARTCARD_GetClockPolarity(), fetch the clock polarity value from the handler instance registers.
    - HAL_SMARTCARD_SetClockPhase(), set the clock phase value into the handler instance registers.
    - HAL_SMARTCARD_GetClockPhase(), fetch the clock phase value from the handler instance registers.
    - HAL_SMARTCARD_SetGuardTime(), set the guard time value into the handler instance registers.
    - HAL_SMARTCARD_GetGuardTime(), fetch the guard time value from the handler instance registers.
    - HAL_SMARTCARD_SetAutoRetryCount(),set the auto retry value into the handler instance registers.
    - HAL_SMARTCARD_GetAutoRetryCount(), fetch the auto retry count value from the handler instance registers.

    Those parameters have the following default state :

    | Parameter           | Default register state                  |
    |---------------------|:---------------------------------------:|
    | Baud rate           |     0U (This value must be changed)     |
    | StopBits            |  HAL_SMARTCARD_STOP_BIT_1(unavailable)  |
    | FirstBit            |    HAL_SMARTCARD_BIT_ORDER_LSB_FIRST    |
    | Parity              |         HAL_SMARTCARD_PARITY_ODD        |
    | Nack                |       HAL_SMARTCARD_NACK_DISABLE        |
    | ClockOutput         |    HAL_SMARTCARD_CLOCK_OUTPUT_DISABLE   |
    | ClockPolarity       |     HAL_SMARTCARD_CLOCK_POLARITY_LOW    |
    | ClockPhase          |     HAL_SMARTCARD_CLOCK_PHASE_1_EDGE    |
    | GuardTime           |                   0U                    |
    | AutoRetryCount      |                   0U                    |
  */

/**
  * @brief  Set the SMARTCARD baud rate configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  baud_rate SMARTCARD baud rate
  * @retval HAL_OK SMARTCARD baud rate set successfully
  * @retval HAL_INVALID_PARAM Invalid baud rate parameter
  */
hal_status_t HAL_SMARTCARD_SetBaudRate(const hal_smartcard_handle_t *hsmartcard, uint32_t baud_rate)
{
  USART_TypeDef *p_smartcardx;
  uint32_t instance_clock_freq;
  uint32_t instance_clock_prescaler;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_BAUD_RATE(baud_rate));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check input parameters */
  if (baud_rate == 0U)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_smartcardx);
  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  instance_clock_prescaler = LL_USART_GetPrescaler(p_smartcardx);
  ASSERT_DBG_PARAM(SMARTCARD_Check_uart_baudrate_validity(instance_clock_freq, instance_clock_prescaler, baud_rate)
                   == HAL_OK);
  LL_USART_SetBaudRate(p_smartcardx, instance_clock_freq, instance_clock_prescaler, LL_USART_OVERSAMPLING_16,
                       baud_rate);


  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);
  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD baud rate configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval uint32_t SMARTCARD baud rate value
  */
uint32_t HAL_SMARTCARD_GetBaudRate(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;
  uint32_t instance_clock_freq;
  uint32_t baud_rate;
  uint32_t prescaler;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_smartcardx);
  ASSERT_DBG_PARAM(instance_clock_freq != 0U);

  prescaler = LL_USART_GetPrescaler(p_smartcardx);
  baud_rate = LL_USART_GetBaudRate(p_smartcardx, instance_clock_freq, prescaler, LL_USART_OVERSAMPLING_16);
  return baud_rate;
}

/**
  * @brief  Set the Stop Bits configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  stop_bits SMARTCARD stop bit
  * @retval HAL_OK SMARTCARD stop bit value set successfully
  */
hal_status_t HAL_SMARTCARD_SetStopBits(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_stop_bits_t stop_bits)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_STOP_BITS(stop_bits));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetStopBitsLength(p_smartcardx, (uint32_t)stop_bits);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD Stop Bits configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_stop_bits_t SMARTCARD stop bit value
  */
hal_smartcard_stop_bits_t HAL_SMARTCARD_GetStopBits(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_stop_bits_t)LL_USART_GetStopBitsLength(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD first bit sent (MSB or LSB) configuration passed in parameters into
  *         the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  first_bit SMARTCARD first bit sent (MSB or LSB)
  * @retval HAL_OK SMARTCARD first bit set successfully
  */
hal_status_t HAL_SMARTCARD_SetFirstBit(const hal_smartcard_handle_t *hsmartcard,
                                       hal_smartcard_bit_order_t first_bit)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_FIRST_BIT(first_bit));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTransferBitOrder(p_smartcardx, (uint32_t)first_bit);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD first bit sent (MSB or LSB) configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_bit_order_t SMARTCARD first bit sent (MSB or LSB)
  */
hal_smartcard_bit_order_t HAL_SMARTCARD_GetFirstBit(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  return (hal_smartcard_bit_order_t)LL_USART_GetTransferBitOrder(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD parity configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  parity SMARTCARD parity mode
  * @retval HAL_OK SMARTCARD parity mode set successfully
  */
hal_status_t HAL_SMARTCARD_SetParity(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_parity_t parity)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_PARITY(parity));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetParity(p_smartcardx, (uint32_t)parity);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD parity configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_parity_t SMARTCARD parity mode
  */
hal_smartcard_parity_t HAL_SMARTCARD_GetParity(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_parity_t)LL_USART_GetParity(p_smartcardx);
}

/**
  * @brief  Set SMARTCARD nack management setting configuration passed in parameters into
  *         the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  nack SMARTCARD nack management setting
  * @retval HAL_OK nack management set successfully
  */
hal_status_t HAL_SMARTCARD_SetNack(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_nack_state_t nack)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_NACK(nack));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  if (nack != HAL_SMARTCARD_NACK_DISABLE)
  {
    LL_USART_EnableSmartcardNACK(p_smartcardx);
  }
  else
  {
    LL_USART_DisableSmartcardNACK(p_smartcardx);
  }

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get SMARTCARD nack management setting configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_nack_state_t nack status
  */
hal_smartcard_nack_state_t HAL_SMARTCARD_GetNack(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  if (LL_USART_IsEnabledSmartcardNACK(p_smartcardx) != 0U)
  {
    return HAL_SMARTCARD_NACK_ENABLE;
  }
  else
  {
    return HAL_SMARTCARD_NACK_DISABLE;
  }
}

/**
  * @brief  Enable the SMARTCARD clock output.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  clock_output SMARTCARD clock output setting from \ref hal_smartcard_clock_output_t
  * @retval HAL_OK SMARTCARD clock output enabled successfully
  */
hal_status_t HAL_SMARTCARD_SetClockOutput(const hal_smartcard_handle_t *hsmartcard,
                                          hal_smartcard_clock_output_t clock_output)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_OUTPUT(clock_output));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  if (clock_output != HAL_SMARTCARD_CLOCK_OUTPUT_DISABLE)
  {
    LL_USART_EnableSCLKOutput(p_smartcardx);
  }
  else
  {
    LL_USART_DisableSCLKOutput(p_smartcardx);
  }

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get SMARTCARD clock output configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_clock_output_t clock output status
  */
hal_smartcard_clock_output_t HAL_SMARTCARD_GetClockOutput(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  if (LL_USART_IsEnabledSCLKOutput(p_smartcardx) != 0U)
  {
    return HAL_SMARTCARD_CLOCK_OUTPUT_ENABLE;
  }
  else
  {
    return HAL_SMARTCARD_CLOCK_OUTPUT_DISABLE;
  }
}

/**
  * @brief  Set the SMARTCARD clock polarity configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  clock_polarity SMARTCARD clock polarity
  * @retval HAL_OK SMARTCARD clock polarity set successfully
  */
hal_status_t HAL_SMARTCARD_SetClockPolarity(const hal_smartcard_handle_t *hsmartcard,
                                            hal_smartcard_clock_polarity_t clock_polarity)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_POLARITY(clock_polarity));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetClockPolarity(p_smartcardx, (uint32_t)clock_polarity);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD clock polarity configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_clock_polarity_t SMARTCARD clock polarity
  */
hal_smartcard_clock_polarity_t HAL_SMARTCARD_GetClockPolarity(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_clock_polarity_t)LL_USART_GetClockPolarity(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD clock phase configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  clock_phase SMARTCARD clock phase
  * @retval HAL_OK SMARTCARD clock phase set successfully
  */
hal_status_t HAL_SMARTCARD_SetClockPhase(const hal_smartcard_handle_t *hsmartcard,
                                         hal_smartcard_clock_phase_t clock_phase)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_CLOCK_PHASE(clock_phase));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetClockPhase(p_smartcardx, (uint32_t)clock_phase);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD clock phase configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_clock_phase_t SMARTCARD clock phase
  */
hal_smartcard_clock_phase_t HAL_SMARTCARD_GetClockPhase(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_clock_phase_t)LL_USART_GetClockPhase(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD guard time configuration passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  guard_time_etu SMARTCARD guard time (Elementary Time Unit)
  * @note   guard time is expressed in etu (Elementary Time Unit), in SMARTCARD case etu is the baud period duration.
  * @retval HAL_OK SMARTCARD guard time set successfully
  */
hal_status_t HAL_SMARTCARD_SetGuardTime(const hal_smartcard_handle_t *hsmartcard, uint32_t guard_time_etu)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_GUARD_TIME(guard_time_etu));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetSmartcardGuardTime(p_smartcardx, guard_time_etu);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD guard time configuration from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @note   guard time is expressed in etu (Elementary Time Unit), in SMARTCARD case etu is the baud period duration.
  * @retval uint32_t SMARTCARD guard time
  */
uint32_t HAL_SMARTCARD_GetGuardTime(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return LL_USART_GetSmartcardGuardTime(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD auto retry count feature into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  retry_count SMARTCARD retry count
  * @retval HAL_OK SMARTCARD retry count set successfully
  */
hal_status_t HAL_SMARTCARD_SetAutoRetryCount(const hal_smartcard_handle_t *hsmartcard, uint32_t retry_count)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_RETRY_COUNT(retry_count));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetSmartcardAutoRetryCount(p_smartcardx, retry_count);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD auto retry count feature from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval uint32_t SMARTCARD retry count
  */
uint32_t HAL_SMARTCARD_GetAutoRetryCount(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return LL_USART_GetSmartcardAutoRetryCount(p_smartcardx);
}

/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group4 Advanced config functions
  * @{
  This subsection provides a set of functions allowing to configure optional USARTx parameters for SMARTCARD mode:

  - Pin inversion:
    - HAL_SMARTCARD_EnableIOInvert(), enable pin active level logic inversion into the handler instance registers.
    - HAL_SMARTCARD_DisableIOInvert(), disable pin active level logic inversion into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledIOInvert(), fetch pin active level logic inversion status from the handler
      instance registers.
    - HAL_SMARTCARD_EnableDataInvert(), enable binary data logic inversion into the handler instance registers.
    - HAL_SMARTCARD_DisableDataInvert(), disable binary data logic inversion into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledDataInvert(), fetch binary data logic inversion status from the handler
      instance registers.
    - HAL_SMARTCARD_EnableTxRxSwap(), enable USART GPIO swap into the handler instance registers.
    - HAL_SMARTCARD_DisableTxRxSwap(), disable USART GPIO swap into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledTxRxSwap(), fetch USART GPIO swap status from the handler instance registers.

  - Rx overrun :
    - HAL_SMARTCARD_EnableRxOverRunDetection(), enable Rx overrun errors detection into the handler instance registers.
    - HAL_SMARTCARD_DisableRxOverRunDetection(), disable Rx overrun errors detection into the handler
      instance registers.
    - HAL_SMARTCARD_IsEnabledRxOverRunDetection(), fetch Rx overrun error detection status from the handler
      instance registers.

  - DMA disable on Rx error:
    - HAL_SMARTCARD_EnableDMAStopOnRxError(), enable DMA stop on Rx error into the handler instance registers.
    - HAL_SMARTCARD_DisableDMAStopOnRxError(), disable DMA stop on Rx error into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledDMAStopOnRxError(), fetch DMA stop on Rx error status from the handler instance registers.

  - Timeout:
    - HAL_SMARTCARD_SetReceiverTimeout(), set the Rx timeout value into the handler instance registers.
    - HAL_SMARTCARD_GetReceiverTimeout(), fetch the Rx timeout value from the handler instance registers.
    - HAL_SMARTCARD_EnableReceiverTimeout(), enable Rx timeout detection into the handler instance registers.
    - HAL_SMARTCARD_DisableReceiverTimeout(), disable Rx timeout detection into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledReceiverTimeout(), fetch Rx timeout detection status from the handler instance registers.

  - Tx complete indication:
    - HAL_SMARTCARD_SetTxCpltIndication(), set the guard time Tx complete indication into the handler
      instance registers.
    - HAL_SMARTCARD_GetTxCpltIndication(), fetch the guard time Tx complete indication from the handler
      instance registers.

  - Block length:
    - HAL_SMARTCARD_SetBlockLength(), set the block length value into the handler instance registers.
    - HAL_SMARTCARD_GetBlockLength(), fetch the block length value from the handler instance registers.
    - HAL_SMARTCARD_EnableEndOfBlockIT(), enable end of block interrupt into the handler instance registers.
    - HAL_SMARTCARD_DisableEndOfBlockIT(), disable end of block interrupt into the handler instance registers.
    - HAL_SMARTCARD_IsEnabledEndOfBlockIT(), fetch end of block interrupt status from the handler instance registers.

  Those optional parameters have the following default state:

  | Parameter           | Default register state                  |
  |---------------------|:---------------------------------------:|
  | IOInversion         |    HAL_SMARTCARD_IO_INVERT_DISABLED     |
  | DataInvert          |   HAL_SMARTCARD_DATA_INVERT_DISABLED    |
  | TxRxSwap            |    HAL_SMARTCARD_TX_RX_SWAP_DISABLED    |
  | RxOverRunDetection  |   HAL_SMARTCARD_OVERRUN_DETECT_ENABLED  |
  | DMAStopOnRxError    |       HAL_SMARTCARD_DMA_STOP_NONE       |
  | ReceiverTimeout     |     HAL_SMARTCARD_TIMEOUT_DISABLED      |
  | TxCpltIndication    | HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME  |
  | BlockLength         |                   0U                    |
  */

/**
  * @brief  Enable SMARTCARD pin active level logic inversion into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Tx inversion feature enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableIOInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTXPinLevel(p_smartcardx, LL_USART_TXPIN_LEVEL_INVERTED);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD pin active level logic inversion into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Tx inversion feature disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableIOInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTXPinLevel(p_smartcardx, LL_USART_TXPIN_LEVEL_STANDARD);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD pin active level logic inversion status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_inversion_status_t SMARTCARD Tx inversion feature status
  */
hal_smartcard_io_invert_status_t HAL_SMARTCARD_IsEnabledIOInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  if (LL_USART_GetTXPinLevel(p_smartcardx) != 0U)
  {
    return HAL_SMARTCARD_IO_INVERT_ENABLED;
  }
  else
  {
    return HAL_SMARTCARD_IO_INVERT_DISABLED;
  }
}

/**
  * @brief Enable the binary Data Inversion into the handler instance registers, (1=L, 0=H).
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval HAL_OK SMARTCARD instance has been correctly configured.
  */
hal_status_t HAL_SMARTCARD_EnableDataInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetBinaryDataLogic(p_smartcardx, LL_USART_BINARY_LOGIC_NEGATIVE);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief Disable the binary Data Inversion into the handler instance registers (1=H, 0=L).
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval HAL_OK SMARTCARD instance has been correctly configured.
  */
hal_status_t HAL_SMARTCARD_DisableDataInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetBinaryDataLogic(p_smartcardx, LL_USART_BINARY_LOGIC_POSITIVE);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief Return the binary Data Inversion status according to the handler instance registers.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval hal_smartcard_data_invert_status_t Current Data Inversion status.
  */
hal_smartcard_data_invert_status_t HAL_SMARTCARD_IsEnabledDataInvert(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  if (LL_USART_GetBinaryDataLogic(p_smartcardx) == LL_USART_BINARY_LOGIC_POSITIVE)
  {
    return HAL_SMARTCARD_DATA_INVERT_DISABLED;
  }
  else
  {
    return HAL_SMARTCARD_DATA_INVERT_ENABLED;
  }
}

/**
  * @brief Enable the Swap between Tx and Rx Pin into the handler instance registers.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval HAL_OK SMARTCARD instance has been correctly configured.
  */
hal_status_t HAL_SMARTCARD_EnableTxRxSwap(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTXRXSwap(p_smartcardx, LL_USART_TXRX_SWAPPED);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief Disable the Swap between Tx and Rx Pin into the handler instance registers.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval HAL_OK SMARTCARD instance has been correctly configured.
  */
hal_status_t HAL_SMARTCARD_DisableTxRxSwap(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTXRXSwap(p_smartcardx, LL_USART_TXRX_STANDARD);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief Return the Swap between Tx and Rx Pin status according to the handler instance registers.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval hal_smartcard_tx_rx_swap_status_t Current Tx Rx Swap status.
  */
hal_smartcard_tx_rx_swap_status_t HAL_SMARTCARD_IsEnabledTxRxSwap(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  if (LL_USART_GetTXRXSwap(p_smartcardx) == LL_USART_TXRX_STANDARD)
  {
    return HAL_SMARTCARD_TX_RX_SWAP_DISABLED;
  }
  else
  {
    return HAL_SMARTCARD_TX_RX_SWAP_ENABLED;
  }
}

/**
  * @brief  Enable SMARTCARD RxOverrun detection into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Rx overrun feature enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableRxOverRunDetection(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_EnableOverrunDetect(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD RxOverrun detection into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Rx overrun feature disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableRxOverRunDetection(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_DisableOverrunDetect(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD RxOverrun detection status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_rx_overrun_detection_status_t SMARTCARD Rx overrun feature status
  */
hal_smartcard_rx_overrun_detection_status_t HAL_SMARTCARD_IsEnabledRxOverRunDetection(const
    hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_rx_overrun_detection_status_t)LL_USART_IsEnabledOverrunDetect(p_smartcardx);
}

/**
  * @brief  Enable SMARTCARD DMA stop on Rx error into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK DMA stop on Rx error feature enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_EnableDMADeactOnRxErr(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD DMA stop on Rx error into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK DMA stop on Rx error feature disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_DisableDMADeactOnRxErr(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD DMA stop on Rx error status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_dma_stop_status_t SMARTCARD DMA stop on Rx error feature status
  */
hal_smartcard_dma_stop_status_t HAL_SMARTCARD_IsEnabledDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  if (LL_USART_IsEnabledDMADeactOnRxErr(p_smartcardx) != 0U)
  {
    return HAL_SMARTCARD_DMA_STOP_ON_RX_ERROR;
  }
  else
  {
    return HAL_SMARTCARD_DMA_STOP_NONE;
  }
}

/**
  * @brief  Set the SMARTCARD receiver timeout value passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  timeout_etu SMARTCARD receiver timeout value
  * @note   timeout is expressed in etu (Elementary Time Unit), in SMARTCARD case etu is the baud period duration.
  * @retval HAL_OK SMARTCARD receiver timeout value set successfully
  */
hal_status_t HAL_SMARTCARD_SetReceiverTimeout(const hal_smartcard_handle_t *hsmartcard, uint32_t timeout_etu)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_TIMEOUT_VALUE(timeout_etu));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_SetRxTimeout(p_smartcardx, timeout_etu);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD receiver timeout value from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @note   timeout is expressed in etu (Elementary Time Unit), in SMARTCARD case etu is the baud period duration.
  * @retval uint32_t SMARTCARD receiver timeout value
  */
uint32_t HAL_SMARTCARD_GetReceiverTimeout(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  return LL_USART_GetRxTimeout(p_smartcardx);
}

/**
  * @brief  Enable SMARTCARD Receiver Timeout feature into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Receiver Timeout feature enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableReceiverTimeout(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_EnableRxTimeout(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD Receiver Timeout feature into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK Receiver Timeout feature disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableReceiverTimeout(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_DisableRxTimeout(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD Receiver Timeout feature status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_timeout_status_t SMARTCARD Receiver Timeout feature status
  */
hal_smartcard_timeout_status_t HAL_SMARTCARD_IsEnabledReceiverTimeout(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_timeout_status_t)LL_USART_IsEnabledRxTimeout(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD Pre guard time Tx complete indication passed in parameters
  *         into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  tx_cplt_indication SMARTCARD Pre guard time Tx complete indication
  * @retval HAL_OK SMARTCARD Tx completion indication set successfully
  */
hal_status_t HAL_SMARTCARD_SetTxCpltIndication(hal_smartcard_handle_t *hsmartcard,
                                               const hal_smartcard_tx_cplt_guard_time_indication_t tx_cplt_indication)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_TX_CPLT(tx_cplt_indication));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  hsmartcard->tx_cplt_indication = tx_cplt_indication;

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD Pre guard time Tx complete indication feature from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_tx_cplt_guard_time_indication_t  SMARTCARD Pre guard time Tx complete indication
  */
hal_smartcard_tx_cplt_guard_time_indication_t HAL_SMARTCARD_GetTxCpltIndication(
  const hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));

  return hsmartcard->tx_cplt_indication;
}

/**
  * @brief  Set the SMARTCARD block length for T=1 smartcard protocol passed in parameters into
  *         the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  block_length_byte block length
  * @retval HAL_OK SMARTCARD block length set successfully
  */
hal_status_t HAL_SMARTCARD_SetBlockLength(const hal_smartcard_handle_t *hsmartcard, uint32_t block_length_byte)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_BLOCK_LENGTH(block_length_byte));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_SetBlockLength(p_smartcardx, block_length_byte);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD block length for T=1 smartcard protocol from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval uint32_t SMARTCARD block length
  */
uint32_t HAL_SMARTCARD_GetBlockLength(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return LL_USART_GetBlockLength(p_smartcardx);
}
/**
  * @brief  Enable SMARTCARD End of block interrupt into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK End of block interrupt enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableEndOfBlockIT(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_EnableIT_EOB(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD End of block interrupt into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK End of block interrupt disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableEndOfBlockIT(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  LL_USART_DisableIT_EOB(p_smartcardx);

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD End of block interrupt status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_end_of_block_interrupt_status_t SMARTCARD End of block interrupt status
  */
hal_smartcard_end_of_block_interrupt_status_t HAL_SMARTCARD_IsEnabledEndOfBlockIT(const hal_smartcard_handle_t
    *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_end_of_block_interrupt_status_t)LL_USART_IsEnabledIT_EOB(p_smartcardx);
}

/**
  * @}
  */
/** @addtogroup SMARTCARD_Exported_Functions_Group5 FIFO config functions
  * @{
  This subsection provides a set of functions allowing use of the FIFO mode feature for the USARTx instance.
  Before using the FIFO mode feature, configure the instance with HAL_SMARTCARD_SetConfig().
  The following functions are provided to use the FIFO mode feature:
    - HAL_SMARTCARD_EnableFifoMode(), enable FIFO in the handler instance registers.
    - HAL_SMARTCARD_DisableFifoMode(), disable FIFO in the handler instance registers.
    - HAL_SMARTCARD_IsEnabledFifoMode(), fetch FIFO status from the handler instance registers.
    - HAL_SMARTCARD_SetTxFifoThreshold(), set the Tx FIFO threshold value in the handler instance registers.
    - HAL_SMARTCARD_GetTxFifoThreshold(), fetch the Tx FIFO threshold value from the handler instance registers.
    - HAL_SMARTCARD_SetRxFifoThreshold(), set the Rx FIFO threshold value in the handler instance registers.
    - HAL_SMARTCARD_GetRxFifoThreshold(), fetch the Rx FIFO threshold value from the handler instance registers.

    Use the following procedure:
    - HAL_SMARTCARD_SetTxFifoThreshold()
    - HAL_SMARTCARD_SetRxFifoThreshold()
    - HAL_SMARTCARD_EnableFifoMode()
    - Start the process, for example: HAL_SMARTCARD_Receive()
  */
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Enable SMARTCARD FIFO mode into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK FIFO feature enabled successfully
  */
hal_status_t HAL_SMARTCARD_EnableFifoMode(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_EnableFIFO(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  hsmartcard->fifo_status = HAL_SMARTCARD_FIFO_MODE_ENABLED;

  return HAL_OK;
}

/**
  * @brief  Disable SMARTCARD FIFO mode into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval HAL_OK FIFO feature disabled successfully
  */
hal_status_t HAL_SMARTCARD_DisableFifoMode(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_DisableFIFO(p_smartcardx);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  hsmartcard->fifo_status = HAL_SMARTCARD_FIFO_MODE_DISABLED;

  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD FIFO status from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_fifo_mode_status_t SMARTCARD FIFO feature status
  */
hal_smartcard_fifo_mode_status_t HAL_SMARTCARD_IsEnabledFifoMode(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_fifo_mode_status_t)LL_USART_IsEnabledFIFO(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD Tx FIFO threshold value passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  tx_fifo_threshold SMARTCARD Tx FIFO threshold value
  * @retval HAL_OK SMARTCARD Tx FIFO threshold value set successfully
  */
hal_status_t HAL_SMARTCARD_SetTxFifoThreshold(hal_smartcard_handle_t *hsmartcard,
                                              const hal_smartcard_fifo_threshold_t tx_fifo_threshold)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_FIFO_THRESHOLD(tx_fifo_threshold));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetTXFIFOThreshold(p_smartcardx, (uint32_t)tx_fifo_threshold);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  uint8_t numerator[]   = {1U, 1U, 1U, 3U, 7U, 1U, 0U, 0U};
  uint8_t shift_amount[] = {3U, 2U, 1U, 2U, 3U, 0U, 0U, 0U};

  hsmartcard->nb_tx_data_to_process = ((uint16_t)TX_FIFO_DEPTH * numerator[tx_fifo_threshold])
                                      >> shift_amount[tx_fifo_threshold];
  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD Tx FIFO threshold value from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_fifo_threshold_t SMARTCARD Tx FIFO threshold value
  */
hal_smartcard_fifo_threshold_t HAL_SMARTCARD_GetTxFifoThreshold(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_fifo_threshold_t)LL_USART_GetTXFIFOThreshold(p_smartcardx);
}

/**
  * @brief  Set the SMARTCARD Rx FIFO threshold value passed in parameters into the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  rx_fifo_threshold SMARTCARD Rx FIFO threshold value
  * @retval HAL_OK SMARTCARD Rx FIFO threshold value set successfully
  */
hal_status_t HAL_SMARTCARD_SetRxFifoThreshold(hal_smartcard_handle_t *hsmartcard,
                                              const hal_smartcard_fifo_threshold_t rx_fifo_threshold)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_FIFO_THRESHOLD(rx_fifo_threshold));

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)HAL_SMARTCARD_STATE_IDLE);
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  SMARTCARD_ENSURE_INSTANCE_DISABLED(p_smartcardx);

  LL_USART_SetRXFIFOThreshold(p_smartcardx, (uint32_t)rx_fifo_threshold);

  SMARTCARD_ENSURE_INSTANCE_ENABLED(p_smartcardx);

  uint8_t numerator[]   = {1U, 1U, 1U, 3U, 7U, 1U, 0U, 0U};
  uint8_t shift_amount[] = {3U, 2U, 1U, 2U, 3U, 0U, 0U, 0U};

  hsmartcard->nb_rx_data_to_process = ((uint16_t)RX_FIFO_DEPTH * numerator[rx_fifo_threshold])
                                      >> shift_amount[rx_fifo_threshold];
  return HAL_OK;
}

/**
  * @brief  Get the SMARTCARD Rx FIFO threshold value from the handler instance registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @retval hal_smartcard_fifo_threshold_t SMARTCARD Rx FIFO threshold
  */
hal_smartcard_fifo_threshold_t HAL_SMARTCARD_GetRxFifoThreshold(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;

  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return (hal_smartcard_fifo_threshold_t)LL_USART_GetRXFIFOThreshold(p_smartcardx);
}
#endif /* USE_HAL_SMARTCARD_FIFO */

/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group6 IO operation functions
  * @{
  This subsection provides a set of functions that allow managing SMARTCARD data transfers.

 There are two transfer modes:
    - Blocking mode: The communication is performed in polling mode.
      The HAL status of all data processing is returned by the same function
      after finishing transfer.
    - Non-blocking mode: The communication is performed using interrupts
      or DMA. These APIs return the HAL status.
      The end of the data processing is indicated through the
      dedicated SMARTCARD IRQ when using interrupt mode or the DMA IRQ when
      using DMA mode.
      The HAL_SMARTCARD_TxCpltCallback() and HAL_SMARTCARD_RxCpltCallback() user callbacks
      are executed, respectively, at the end of the transmit or receive process.
      The HAL_SMARTCARD_ErrorCallback() user callback is executed when a communication error is detected.

  Polling APIs:
    - HAL_SMARTCARD_Transmit(), transmit an amount of data in blocking mode.
    - HAL_SMARTCARD_Receive(), receive an amount of data in blocking mode.
    - HAL_SMARTCARD_Abort(), abort data transfer.

  IT APIs:
    - HAL_SMARTCARD_Transmit_IT(), transmit an amount of data in interrupt mode.
    - HAL_SMARTCARD_Transmit_IT_Opt(), transmit an amount of data in interrupt mode,
      enabling given optional interrupts.
    - HAL_SMARTCARD_Receive_IT(), receive an amount of data in interrupt mode.
    - HAL_SMARTCARD_Receive_IT_Opt(), receive an amount of data in interrupt mode, enabling given optional interrupts.
    - HAL_SMARTCARD_Abort_IT(), abort data transfer and call HAL_SMARTCARD_AbortCpltCallback().

  DMA APIs:
    - HAL_SMARTCARD_Transmit_DMA(), transmit an amount of data in DMA mode.
    - HAL_SMARTCARD_Transmit_DMA_Opt(), transmit an amount of data in DMA mode, enabling given optional interrupts.
    - HAL_SMARTCARD_Receive_DMA(), receive an amount of data in DMA mode.
    - HAL_SMARTCARD_Receive_DMA_Opt(), receive an amount of data in DMA mode, enabling given optional interrupts.
  */

/**
  * @brief  Send an amount of data in blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data pointer to data buffer.
  * @param  size_byte amount of data to be sent.
  * @param  timeout_ms Timeout duration.
  * @note   When FIFO mode is enabled, writing a data in the TDR register adds one
  *         data to the TXFIFO. Write operations to the TDR register are performed
  *         when TXFNF flag is set. From hardware perspective, TXFNF flag and
  *         TXE are mapped on the same bit-field.
  * @retval HAL_OK Operation started successfully
  * @retval HAL_TIMEOUT Transfer timeout
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Transmit(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data,
                                    uint16_t size_byte, uint32_t timeout_ms)
{
  USART_TypeDef *p_smartcardx;
  uint32_t tickstart;
  const uint8_t *ptmpdata = p_data;

  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_TX_ACTIVE);

#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */

  uint32_t nack_enabled = LL_USART_IsEnabledSmartcardNACK(p_smartcardx);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /* In case of TX only mode, if NACK is enabled, the USART must be able to monitor
      the bidirectional line to detect a NACK signal in case of parity error.
      Therefore, the receiver block must be enabled as well (RE bit in register CR1 must be set). */
  if (nack_enabled != 0U)
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX_RX);
  }
  else
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX);
  }

  LL_USART_Enable(p_smartcardx);

  hsmartcard->tx_xfer_size = size_byte;
  hsmartcard->tx_xfer_count = size_byte;

  while (hsmartcard->tx_xfer_count > 0U)
  {
    hsmartcard->tx_xfer_count--;
    if (SMARTCARD_WaitOnFlagUntilTimeout(hsmartcard, LL_USART_ISR_TXE_TXFNF, 0U, tickstart, timeout_ms) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    LL_USART_TransmitData8(p_smartcardx, (uint8_t)(*ptmpdata & 0xFFU));
    ptmpdata++;
  }

  if (SMARTCARD_WaitOnFlagUntilTimeout(hsmartcard, SMARTCARD_TRANSMISSION_COMPLETION_FLAG(hsmartcard), 0U,
                                       tickstart, timeout_ms) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if (nack_enabled != 0U)
  {
    /* In case of NACK enabled, USART is disabled to empty RDR register */
    LL_USART_Disable(p_smartcardx);
    LL_USART_Enable(p_smartcardx);
  }

  /* Perform a TX/RX FIFO Flush at end of Tx phase, as all sent bytes are appearing in Rx Data register */
  SMARTCARD_FLUSH_DRREGISTER(hsmartcard);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data Pointer to data buffer.
  * @param  size_byte Amount of data to be received.
  * @param  timeout_ms Timeout duration.
  * @note   When FIFO mode is enabled, the RXFNE flag is set as long as the RXFIFO
  *         is not empty. Read operations from the RDR register are performed when
  *         RXFNE flag is set. From hardware perspective, RXFNE flag and
  *         RXNE are mapped on the same bit-field.
  * @retval HAL_OK Operation started successfully
  * @retval HAL_TIMEOUT Transfer timeout
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Receive(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                   uint16_t size_byte, uint32_t timeout_ms)
{
  USART_TypeDef *p_smartcardx;
  uint32_t tickstart;
  uint8_t *ptmpdata = p_data;
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check input parameters */
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_RX_ACTIVE);

#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */

  LL_USART_EnableDirectionRx(p_smartcardx);

  LL_USART_Enable(p_smartcardx);

  LL_USART_ClearFlag_ORE(p_smartcardx);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  hsmartcard->rx_xfer_size = size_byte;
  hsmartcard->rx_xfer_count = size_byte;

  /* Check the remain data to be received */
  while (hsmartcard->rx_xfer_count > 0U)
  {
    hsmartcard->rx_xfer_count--;

    if (SMARTCARD_WaitOnFlagUntilTimeout(hsmartcard, LL_USART_ISR_RXNE_RXFNE, 0U, tickstart, timeout_ms) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    *ptmpdata = LL_USART_ReceiveData8(p_smartcardx);
    ptmpdata++;
  }

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

  return HAL_OK;
}
/**
  * @brief  Send an amount of data in interrupt mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data pointer to data buffer.
  * @param  size_byte amount of data to be sent.
  * @note   When FIFO mode is disabled, USART interrupt is generated whenever
  *         USART_TDR register is empty, i.e one interrupt per data to transmit.
  * @note   When FIFO mode is enabled, USART interrupt is generated whenever
  *         TXFIFO threshold reached. In that case the interrupt rate depends on
  *         TXFIFO threshold configuration.
  * @note   This function sets the hsmartcard->TxIsr function pointer according to
  *         the FIFO mode (data transmission processing depends on FIFO mode).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_BUSY Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Transmit_IT(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint16_t size_byte)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_TX_ACTIVE);

  return SMARTCARD_Start_Transmit_IT(hsmartcard, p_data, size_byte, HAL_SMARTCARD_OPT_TX_IT_NONE);
}

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Send an amount of data in interrupt mode, allows the user to enable the optional interrupts part of
  *         \ref SMARTCARD_Transmit_IT_Optional_Interrupts.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data Pointer to data buffer.
  * @param  size_byte Amount of data to be sent.
  * @param  interrupts Optional interrupts part of \ref SMARTCARD_Transmit_IT_Optional_Interrupts.
  * @note   When FIFO mode is disabled, USART interrupt is generated whenever
  *         USART_TDR register is empty, i.e., one interrupt per data to transmit.
  * @note   When FIFO mode is enabled, USART interrupt is generated whenever
  *         TXFIFO threshold is reached. In that case the interrupt rate depends on
  *         TXFIFO threshold configuration.
  * @note   This function sets the hsmartcard->TxIsr function pointer according to
  *         the FIFO mode (data transmission processing depends on FIFO mode).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_BUSY Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Transmit_IT_Opt(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data,
                                           uint16_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_SMARTCARD_OPT_TX_IT(interrupts));

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_TX_ACTIVE);

  return SMARTCARD_Start_Transmit_IT(hsmartcard, p_data, size_byte, interrupts);
}
#endif /* USE_HAL_SMARTCARD_FIFO */

/**
  * @brief  Receive an amount of data in interrupt mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data pointer to data buffer.
  * @param  size_byte amount of data to be received.
  * @note   When FIFO mode is disabled, USART interrupt is generated whenever
  *         USART_RDR register can be read, i.e one interrupt per data to receive.
  * @note   When FIFO mode is enabled, USART interrupt is generated whenever
  *         RXFIFO threshold reached. In that case the interrupt rate depends on
  *         RXFIFO threshold configuration.
  * @note   This function sets the hsmartcard->RxIsr function pointer according to
  *         the FIFO mode (data reception processing depends on FIFO mode).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_BUSY Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Receive_IT(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint16_t size_byte)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_RX_ACTIVE);

  return SMARTCARD_Start_Receive_IT(hsmartcard, (uint8_t *)p_data, size_byte, HAL_SMARTCARD_OPT_RX_IT_NONE);
}
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Receive an amount of data in interrupt mode, allows the user to enable the optional interrupts part of
  *         \ref SMARTCARD_Receive_IT_Optional_Interrupts.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data Pointer to data buffer.
  * @param  size_byte Amount of data to be received.
  * @param  interrupts Optional interrupts part of \ref SMARTCARD_Receive_IT_Optional_Interrupts.
  * @note   When FIFO mode is disabled, USART interrupt is generated whenever
  *         USART_RDR register can be read, i.e., one interrupt per data to receive.
  * @note   When FIFO mode is enabled, USART interrupt is generated whenever
  *         RXFIFO threshold is reached. In that case the interrupt rate depends on
  *         RXFIFO threshold configuration.
  * @note   This function sets the hsmartcard->RxIsr function pointer according to
  *         the FIFO mode (data reception processing depends on FIFO mode).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_BUSY Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Receive_IT_Opt(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                          uint16_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_SMARTCARD_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_RX_ACTIVE);

  return SMARTCARD_Start_Receive_IT(hsmartcard, (uint8_t *)p_data, size_byte, interrupts);
}
#endif /* USE_HAL_SMARTCARD_FIFO */

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/**
  * @brief  Send an amount of data in DMA mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data pointer to data buffer.
  * @param  size_byte amount of data to be sent.
  * @retval HAL_OK Operation started successfully
  * @retval HAL_ERROR DMA did not start successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Transmit_DMA(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint16_t size_byte)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(hsmartcard->hdma_tx != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (hsmartcard->hdma_tx == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_TX_ACTIVE);

  return SMARTCARD_Start_Transmit_DMA(hsmartcard, p_data, size_byte, HAL_SMARTCARD_OPT_DMA_TX_IT_HT);
}

/**
  * @brief  Send an amount of data in DMA mode, allows the user to enable the optional interrupts part of
  *         \ref SMARTCARD_Transmit_DMA_Optional_Interrupts.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data Pointer to data buffer.
  * @param  size_byte Amount of data to be sent.
  * @param  interrupts Optional interrupts part of \ref SMARTCARD_Transmit_DMA_Optional_Interrupts.
  * @retval HAL_OK Operation started successfully
  * @retval HAL_ERROR DMA did not start successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Transmit_DMA_Opt(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data,
                                            uint16_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(hsmartcard->hdma_tx != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_OPT_TX_DMA(interrupts));

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (hsmartcard->hdma_tx == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_TX_ACTIVE);

  return SMARTCARD_Start_Transmit_DMA(hsmartcard, p_data, size_byte, interrupts);

}

/**
  * @brief  Receive an amount of data in DMA mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data pointer to data buffer.
  * @param  size_byte amount of data to be received.
  * @note   The SMARTCARD-associated USART parity is enabled (PCE = 1),
  *         the received data contains the parity bit (MSB position).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_ERROR DMA did not start successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Receive_DMA(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint16_t size_byte)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(hsmartcard->hdma_rx != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (hsmartcard->hdma_rx == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_RX_ACTIVE);

  return (SMARTCARD_Start_Receive_DMA(hsmartcard, (uint8_t *)p_data, size_byte, HAL_SMARTCARD_OPT_DMA_RX_IT_HT));
}

/**
  * @brief  Receive an amount of data in DMA mode, allows the user to enable the optional interrupts part of
  *         \ref SMARTCARD_Receive_DMA_Optional_Interrupts.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  p_data Pointer to data buffer.
  * @param  size_byte Amount of data to be received.
  * @param  interrupts Optional interrupts part of \ref SMARTCARD_Receive_DMA_Optional_Interrupts.
  * @note   The SMARTCARD-associated USART parity is enabled (PCE = 1),
  *         the received data contains the parity bit (MSB position).
  * @retval HAL_OK Operation started successfully
  * @retval HAL_ERROR DMA did not start successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_Receive_DMA_Opt(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                           uint16_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(hsmartcard->hdma_rx != NULL);
  ASSERT_DBG_PARAM(IS_SMARTCARD_OPT_RX_DMA(interrupts));

  ASSERT_DBG_STATE(hsmartcard->global_state, HAL_SMARTCARD_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (hsmartcard->hdma_rx == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  HAL_CHECK_UPDATE_STATE(hsmartcard, global_state, HAL_SMARTCARD_STATE_IDLE, HAL_SMARTCARD_STATE_RX_ACTIVE);

  return (SMARTCARD_Start_Receive_DMA(hsmartcard, (uint8_t *)p_data, size_byte, interrupts));
}
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @brief  Abort ongoing transfers either Tx or Rx (blocking mode).
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   This procedure can be used to abort any ongoing transfer started in interrupt or DMA mode.
  *         This procedure performs the following operations:
  *           - Disable SMARTCARD interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to HAL_SMARTCARD_STATE_IDLE
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL_OK Operation successfully aborted
  * @retval HAL_TIMEOUT Abort timeout
  */
hal_status_t HAL_SMARTCARD_Abort(hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_ABORT;
  SMARTCARD_Abort(hsmartcard);

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_smartcardx) != 0U)
  {
    if (hsmartcard->hdma_tx != NULL)
    {
      LL_USART_DisableDMAReq_TX(p_smartcardx);

      /* No call back execution at end of DMA abort procedure */
      (void)HAL_DMA_Abort(hsmartcard->hdma_tx);
    }
  }
  if (LL_USART_IsEnabledDMAReq_RX(p_smartcardx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_smartcardx);

    /* Abort the SMARTCARD DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (hsmartcard->hdma_rx != NULL)
    {
      (void)HAL_DMA_Abort(hsmartcard->hdma_rx);
    }
  }
#endif /* USE_HAL_SMARTCARD_DMA */

  /* Reset Tx and Rx transfer counters */
  hsmartcard->tx_xfer_count = 0U;
  hsmartcard->rx_xfer_count = 0U;

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF
                     | LL_USART_ICR_FECF | LL_USART_ICR_RTOCF | LL_USART_ICR_EOBCF);

  /* Restore hsmartcard->global_state to Idle */
  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Abort ongoing transfers either Tx or Rx (Interrupt mode).
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SMARTCARD Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle state to HAL_SMARTCARD_STATE_IDLE
  *           - At abort completion, call the user abort complete callback
  * @note   This procedure is executed in interrupt mode, meaning that the abort procedure is
  *         considered complete only when the user abort complete callback is executed (not when exiting the function).
  * @retval HAL_OK Operation successfully aborted
  */
hal_status_t HAL_SMARTCARD_Abort_IT(hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_ABORT;
  SMARTCARD_Abort(hsmartcard);

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
  uint32_t abortcplt = 1U;
  /* If DMA Tx and/or DMA Rx Handles are associated to SMARTCARD Handle,
     DMA Abort complete callbacks must be initialised before any call
     to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if (hsmartcard->hdma_tx != NULL)
  {
    /* Set DMA Abort Complete callback if SMARTCARD DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if (LL_USART_IsEnabledDMAReq_TX(p_smartcardx) != 0U)
    {
      hsmartcard->hdma_tx->p_xfer_abort_cb = SMARTCARD_DMATxAbortCallback;

      LL_USART_DisableDMAReq_TX(p_smartcardx);

      /* Abort the SMARTCARD DMA Tx channel : use non blocking DMA Abort API (callback) */
      if (HAL_DMA_Abort_IT(hsmartcard->hdma_tx) == HAL_OK)
      {
        abortcplt = 0U;
      }
    }
  }
  /* DMA Rx Handle is valid */
  if (hsmartcard->hdma_rx != NULL)
  {
    /* Set DMA Abort Complete callback if SMARTCARD DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if (LL_USART_IsEnabledDMAReq_RX(p_smartcardx) != 0U)
    {
      hsmartcard->hdma_rx->p_xfer_abort_cb = SMARTCARD_DMARxAbortCallback;
      LL_USART_DisableDMAReq_RX(p_smartcardx);

      /* SMARTCARD Rx DMA abort callback has already been initialized:
        it will lead to a call to HAL_SMARTCARD_AbortCpltCallback() at the end of the DMA abort procedure */
      if (HAL_DMA_Abort_IT(hsmartcard->hdma_rx) != HAL_OK)
      {
        abortcplt = 1U;
      }
      else
      {
        abortcplt = 0U;
      }
    }
  }

  /* If no DMA abort complete callback execution is required, call the user abort complete callback. */
  if (abortcplt != 0U)
#endif /* USE_HAL_SMARTCARD_DMA */
  {
    /* Reset Tx and Rx transfer counters */
    hsmartcard->tx_xfer_count = 0U;
    hsmartcard->rx_xfer_count = 0U;

    /* Clear ISR function pointers */
    hsmartcard->p_rx_isr = NULL;
    hsmartcard->p_tx_isr = NULL;

    /* Clear the Error flags in the ICR register */
    LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF
                       | LL_USART_ICR_FECF | LL_USART_ICR_RTOCF | LL_USART_ICR_EOBCF);

    hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

    /* As no DMA is to be aborted, call the user abort complete callback directly. */
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    hsmartcard->p_abort_cplt_callback(hsmartcard);
#else
    HAL_SMARTCARD_AbortCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group7
  * @{
  This subsection provides a set of functions allowing to link the HAL SMARTCARD handle to a Tx and Rx DMA handler
  for the USARTx instance.
  A set of functions is provided to use the DMA feature:
    - HAL_SMARTCARD_SetTxDMA(): Link a DMA instance to the Tx channel
    - HAL_SMARTCARD_SetRxDMA(): Link a DMA instance to the Rx channel
 */
#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/**
  * @brief Set DMA channel for Transmission.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param hdma_tx Pointer to a hal_dma_handle_t structure which contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_tx is NULL.
  */
hal_status_t HAL_SMARTCARD_SetTxDMA(hal_smartcard_handle_t *hsmartcard, hal_dma_handle_t *hdma_tx)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(hdma_tx != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_tx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->hdma_tx = hdma_tx;
  hdma_tx->p_parent = hsmartcard;

  return HAL_OK;
}

/**
  * @brief Set DMA channel for Reception.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param hdma_rx Pointer to a hal_dma_handle_t structure which contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_rx is NULL.
  */
hal_status_t HAL_SMARTCARD_SetRxDMA(hal_smartcard_handle_t *hsmartcard, hal_dma_handle_t *hdma_rx)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(hdma_rx != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_rx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->hdma_rx = hdma_rx;
  hdma_rx->p_parent = hsmartcard;

  return HAL_OK;
}
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group8 IRQHandler and Default Callbacks
  * @{
This subsection provides the function handling the interruption of the SMARTCARDx in asynchronous mode.

  - HAL_SMARTCARD_IRQHandler(): process the interruption of an instance

  HAL_SMARTCARD_IRQHandler() is designed to process the different interruptions in the following order:
    - Error on Rx side (PE, FE, NE, ORE, RTOF)
    - Error on DMA side
    - Data on Rx side
    - Data on Tx side
    - FIFO Empty (Tx)
    - FIFO Full (Rx)

  Depending on the process function one's use, different callback might be triggered:

| Process API \n \ \n Callbacks      | HAL_SMARTCARD_Transmit_IT | HAL_SMARTCARD_Receive_IT |
|------------------------------------|:-------------------------:|:------------------------:|
| HAL_SMARTCARD_TxCpltCallback       |             x             |                          |
| HAL_SMARTCARD_RxCpltCallback       |                           |             x            |
| HAL_SMARTCARD_ErrorCallback        |             x             |             x            |

| Process API \n \ \n Callbacks      | HAL_SMARTCARD_Transmit_IT_Opt | HAL_SMARTCARD_Receive_IT_Opt |
|------------------------------------|:-----------------------------:|:----------------------------:|
| HAL_SMARTCARD_TxCpltCallback       |               x               |                              |
| HAL_SMARTCARD_RxCpltCallback       |                               |               x              |
| HAL_SMARTCARD_ErrorCallback        |               x               |               x              |
| HAL_SMARTCARD_TxFifoEmptyCallback* |               x               |                              |
| HAL_SMARTCARD_RxFifoFullCallback** |                               |               x              |
@note * with HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY arguments value for interrupts parameter
@note ** with HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL arguments value for interrupts parameter

| Process API \n \ \n Callbacks       | HAL_SMARTCARD_Transmit_DMA | HAL_SMARTCARD_Receive_DMA |
|-------------------------------------|:--------------------------:|:-------------------------:|
| HAL_SMARTCARD_TxHalfCpltCallback*   |               x            |                           |
| HAL_SMARTCARD_TxCpltCallback        |               x            |                           |
| HAL_SMARTCARD_RxHalfCpltCallback*   |                            |              x            |
| HAL_SMARTCARD_RxCpltCallback        |                            |              x            |
| HAL_SMARTCARD_ErrorCallback**       |               x            |              x            |
@note * these callbacks might be called following DMA IRQ management, not SMARTCARDx IRQ management.
@note ** these callbacks might be called following DMA IRQ management, or SMARTCARDx IRQ management.

| Process API \n \ \n Callbacks       | HAL_SMARTCARD_Transmit_DMA_Opt | HAL_SMARTCARD_Receive_DMA_Opt |
|-------------------------------------|:------------------------------:|:-----------------------------:|
| HAL_SMARTCARD_TxHalfCpltCallback    |               x                |                               |
| HAL_SMARTCARD_TxCpltCallback        |               x                |                               |
| HAL_SMARTCARD_RxHalfCpltCallback    |                                |                x              |
| HAL_SMARTCARD_RxCpltCallback        |                                |                x              |
| HAL_SMARTCARD_TxFifoEmptyCallback*  |               x                |                               |
| HAL_SMARTCARD_RxFifoFullCallback**  |                                |                x              |
| HAL_SMARTCARD_ErrorCallback         |               x                |                x              |
@note * with HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY arguments value for interrupts parameter
@note ** with HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL arguments value for interrupts parameter

| Process API \n \ \n Callbacks           | HAL_SMARTCARD_Abort_IT |
|-----------------------------------------|:----------------------:|
| HAL_SMARTCARD_AbortCpltCallback         |            x           |
| HAL_SMARTCARD_ErrorCallback             |            x           |
  */

/**
  * @brief Handle SMARTCARD interrupt request.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains the configuration
  * information for SMARTCARD module.
  */
void HAL_SMARTCARD_IRQHandler(hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  uint32_t isrflags   = LL_USART_READ_REG(p_smartcardx, ISR);
  uint32_t cr1its     = LL_USART_READ_REG(p_smartcardx, CR1);
  uint32_t cr3its     = LL_USART_READ_REG(p_smartcardx, CR3);
  uint32_t errorflags;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
  if (errorflags == 0U)
  {
    /* SMARTCARD in mode Receiver ---------------------------------------------------*/
    if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3its & USART_CR3_RXFTIE) != 0U)))
    {
      if (hsmartcard->p_rx_isr != NULL)
      {
        hsmartcard->p_rx_isr(hsmartcard);
      }
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
      /* As RXFF ISR is delayed compared to RXFT ISR we have to use the RXFT ISR to use the Fifo Full callback */
      if (((cr1its & USART_CR1_RXFFIE) != 0U) && ((cr3its & USART_CR3_RXFTIE) != 0U)
          && (((cr3its & USART_CR3_RXFTCFG) >> USART_CR3_RXFTCFG_Pos) == LL_USART_FIFO_THRESHOLD_8_8))
      {
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
        hsmartcard->p_rx_fifo_full_callback(hsmartcard);
#else
        HAL_SMARTCARD_RxFifoFullCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
      }
#endif /* USE_HAL_SMARTCARD_FIFO */
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != 0U)
      && ((((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U)
           || ((cr1its & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE)) != 0U))))
  {
    /* SMARTCARD parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
    {
      LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_PECF);

#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
      hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_PE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    }

    /* SMARTCARD frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_FECF);

#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
      if (hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
      {
        hsmartcard->last_error_codes |= HAL_SMARTCARD_TRANSMIT_ERROR_NACK;
      }
      else
      {
        hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_FE;
      }
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    }

    /* SMARTCARD noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_NECF);

#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
      hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_NE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    }

    /* SMARTCARD Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_ORE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3its & USART_CR3_RXFTIE) != 0U)
            || ((cr3its & USART_CR3_EIE) != 0U)))
    {
      /* Discard Overrun Error occurring in Transmit phase */
      if (LL_USART_IsEnabledDirectionTx(p_smartcardx) != 0U)
      {
        errorflags = (errorflags & ~USART_ISR_ORE);
      }
#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
      else
      {
        hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_ORE;
      }
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
      LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF);

    }

    /* SMARTCARD receiver timeout interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U))
    {
      LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_RTOCF);

#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
      hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_RTO;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    }

    /* Call SMARTCARD Error Call back function if need be --------------------------*/
    if (errorflags != 0U)
    {
      /* SMARTCARD in mode Receiver ---------------------------------------------------*/
      if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
          && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
              || ((cr3its & USART_CR3_RXFTIE) != 0U)))
      {
        if (hsmartcard->p_rx_isr != NULL)
        {
          hsmartcard->p_rx_isr(hsmartcard);
        }
      }

      /* If Error is to be considered as blocking :
          - Receiver Timeout error in Reception
          - Overrun error in Reception
          - any error occurs in DMA mode reception
      */
      if ((LL_USART_IsEnabledDMAReq_RX(p_smartcardx) != 0U)
          || ((errorflags & (USART_ISR_RTOF | USART_ISR_ORE)) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the SMARTCARD state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
        if (LL_USART_IsEnabledDMAReq_RX(p_smartcardx) != 0U)
        {
          SMARTCARD_EndRxTransfer(hsmartcard);

          /* Abort the SMARTCARD DMA Rx channel */
          if (hsmartcard->hdma_rx != NULL)
          {
            /* Set the SMARTCARD DMA Abort callback :
               will lead to call HAL_SMARTCARD_ErrorCallback() at end of DMA abort procedure */
            hsmartcard->hdma_rx->p_xfer_abort_cb = SMARTCARD_DMAAbortOnError;

            /* Abort DMA RX */
            if (HAL_DMA_Abort_IT(hsmartcard->hdma_rx) != HAL_OK)
            {
              /* Call Directly hsmartcard->hdma_rx->p_xfer_abort_cb function in case of error */
              hsmartcard->hdma_rx->p_xfer_abort_cb(hsmartcard->hdma_rx);
            }
          }
          else
          {
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
            hsmartcard->p_error_callback(hsmartcard);
#else
            HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
          }
        }
        else
#endif /* USE_HAL_SMARTCARD_DMA */
        {
          SMARTCARD_EndRxTransfer(hsmartcard);
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
          hsmartcard->p_error_callback(hsmartcard);
#else
          HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
        }
      }
      /* other error type to be considered as blocking :
          - Frame error flag in Transmission (No ack despite trials)
      */
      else if ((hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
               && ((errorflags & USART_ISR_FE) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the SMARTCARD state ready to be able to start again the process,
           Disable Tx Interrupts, and disable Tx DMA request, if ongoing */

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
        if (LL_USART_IsEnabledDMAReq_TX(p_smartcardx) != 0U)
        {
          SMARTCARD_EndTxTransfer(hsmartcard);

          /* Abort the SMARTCARD DMA Tx channel */
          if (hsmartcard->hdma_tx != NULL)
          {
            /* Set the SMARTCARD DMA Abort callback :
               will lead to call HAL_SMARTCARD_ErrorCallback() at end of DMA abort procedure */
            hsmartcard->hdma_tx->p_xfer_abort_cb = SMARTCARD_DMAAbortOnError;

            if (HAL_DMA_Abort_IT(hsmartcard->hdma_tx) != HAL_OK)
            {
              /* Call Directly hsmartcard->hdma_tx->p_xfer_abort_cb function in case of error */
              hsmartcard->hdma_tx->p_xfer_abort_cb(hsmartcard->hdma_tx);
            }
          }
          else
          {
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
            hsmartcard->p_error_callback(hsmartcard);
#else
            HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
          }
        }
        else
#endif /* USE_HAL_SMARTCARD_DMA */
        {
          SMARTCARD_EndTxTransfer(hsmartcard);
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
          hsmartcard->p_error_callback(hsmartcard);
#else
          HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
        hsmartcard->p_error_callback(hsmartcard);
#else
        HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
      }
    }
    return;

  } /* End if some error occurs */

  /* SMARTCARD in mode Receiver, end of block interruption ------------------------*/
  if (((isrflags & USART_ISR_EOBF) != 0U) && ((cr1its & USART_CR1_EOBIE) != 0U))
  {
    hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    hsmartcard->p_rx_cplt_callback(hsmartcard);
#else
    HAL_SMARTCARD_RxCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
    /* Clear EOBF interrupt after HAL_SMARTCARD_RxCpltCallback() call for the End of Block information
       to be available during HAL_SMARTCARD_RxCpltCallback() processing */
    LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_EOBCF);
    return;
  }

  /* SMARTCARD in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_ISR_TXE_TXFNF) != 0U)
      && (((cr1its & USART_CR1_TXEIE_TXFNFIE) != 0U)
          || ((cr3its & USART_CR3_TXFTIE) != 0U)))
  {
    if (hsmartcard->p_tx_isr != NULL)
    {
      hsmartcard->p_tx_isr(hsmartcard);
    }
    return;
  }

  /* SMARTCARD in mode Transmitter (transmission end) ------------------------*/
  if (hsmartcard->tx_cplt_indication != HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME)
  {
    if (LL_USART_IsEnabledIT_TCBGT(p_smartcardx) != 0U)
    {
      if (LL_USART_IsActiveFlag_TCBGT(p_smartcardx) != 0U)
      {
        SMARTCARD_EndTransmit_IT(hsmartcard);
        return;
      }
    }
  }
  else
  {
    if (LL_USART_IsEnabledIT_TC(p_smartcardx) != 0U)
    {
      if (LL_USART_IsActiveFlag_TC(p_smartcardx) != 0U)
      {
        SMARTCARD_EndTransmit_IT(hsmartcard);
        return;
      }
    }
  }

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /* SMARTCARD TX FIFO Empty occurred ----------------------------------------------*/
  if (((isrflags & USART_ISR_TXFE) != 0U) && ((cr1its & USART_CR1_TXFEIE) != 0U))
  {
    hsmartcard->p_tx_isr(hsmartcard);
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    hsmartcard->p_tx_fifo_empty_callback(hsmartcard);
#else
    HAL_SMARTCARD_TxFifoEmptyCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
    return;
  }
#endif /* USE_HAL_SMARTCARD_FIFO */
}

/**
  * @brief  SMARTCARD Tx completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_TxCpltCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_TxCpltCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Tx Half completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_TxHalfCpltCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_TxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Rx completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
             the HAL_SMARTCARD_RxCpltCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_RxCpltCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Rx Half completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the @ref HAL_SMARTCARD_RxHalfCpltCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_RxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Error callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_ErrorCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_ErrorCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Abort completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_AbortCpltCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_AbortCpltCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);

  /**
   */
}

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  SMARTCARD Rx FIFO full callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_RxFifoFullCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_RxFifoFullCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

/**
  * @brief  SMARTCARD Tx FIFO empty callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @warning This function must not be modified, when the callback is needed,
  *          the HAL_SMARTCARD_TxFifoEmptyCallback() can be implemented in the user file.
  */
__WEAK void HAL_SMARTCARD_TxFifoEmptyCallback(hal_smartcard_handle_t *hsmartcard)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmartcard);
}

#endif /* USE_HAL_SMARTCARD_FIFO */

/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group9 Callbacks Register functions
  * @{
  This subsection provides a set of functions allowing to configure the Callbacks for the USARTx instance.
  Prior to configure the Callbacks, one has to configure one's with HAL_SMARTCARD_SetConfig().
  A set of functions is provided to configure the callbacks:
    - HAL_SMARTCARD_RegisterTxHalfCpltCallback(): Set the Tx half complete callback
    - HAL_SMARTCARD_RegisterTxCpltCallback(): Set the Tx complete callback
    - HAL_SMARTCARD_RegisterRxHalfCpltCallback(): Set the Rx half complete callback
    - HAL_SMARTCARD_RegisterRxCpltCallback(): Set the Rx complete callback
    - HAL_SMARTCARD_RegisterErrorCallback(): Set the error callback
    - HAL_SMARTCARD_RegisterAbortCpltCallback(): Set the abort complete callback
    - HAL_SMARTCARD_RegisterRxFifoFullCallback(): Set the Rx Fifo full callback
    - HAL_SMARTCARD_RegisterTxFifoEmptyCallback(): Set the Tx Fifo empty callback
  */
#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register the SMARTCARD Tx Transfer completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the Tx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterTxCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_tx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD Tx Transfer Half completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the Tx Transfer Half completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterTxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_tx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD Rx Transfer completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the Rx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterRxCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_rx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD Rx Transfer Half completed callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the Rx Transfer Half completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterRxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_rx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD error callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the error callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterErrorCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_error_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD abort complete callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the abort complete function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterAbortCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_abort_cplt_callback = p_callback;

  return HAL_OK;
}

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Register the SMARTCARD Rx FIFO empty callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the abort Rx FIFO empty function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterRxFifoFullCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_rx_fifo_full_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMARTCARD Tx FIFO empty callback.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t
  * @param  p_callback Pointer to the Tx FIFO empty function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMARTCARD_RegisterTxFifoEmptyCallback(hal_smartcard_handle_t *hsmartcard,
                                                       hal_smartcard_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, ((uint32_t)HAL_SMARTCARD_STATE_IDLE | (uint32_t)HAL_SMARTCARD_STATE_INIT));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmartcard->p_tx_fifo_empty_callback = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_SMARTCARD_FIFO */

#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group10 State, Error and Clock Frequency functions
  * @{
  This subsection provides 2 functions allowing to read peripheral state and last occurred errors.
  - HAL_SMARTCARD_GetState() API can be helpful to check in run-time the state of the SMARTCARD peripheral
  - HAL_SMARTCARD_GetLastErrorCodes() API to retrieve the error codes in case of HAL_ERROR return
    available under the compilation switch USE_HAL_SMARTCARD_GET_LAST_ERRORS
  - HAL_SMARTCARD_GetClockFreq(), report the SMARTCARD clock frequency from the RCC configuration.
 */

/**
  * @brief Retrieve the SMARTCARD handle state.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *               the configuration information for SMARTCARD module.
  * @retval hal_smartcard_state_t SMARTCARD state
  */
hal_smartcard_state_t HAL_SMARTCARD_GetState(const hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  return hsmartcard->global_state;
}

#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
/**
  * @brief Retrieve the SMARTCARD errors codes.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                   the configuration information for SMARTCARD module.
  * @retval uint32_t Returned value can be a combination of the following values:
  *         @arg @ref HAL_SMARTCARD_ERROR_NONE
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_PE
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_NE
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_FE
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_ORE
  *         @arg @ref HAL_SMARTCARD_TRANSMIT_ERROR_NACK
  *         @arg @ref HAL_SMARTCARD_TRANSMIT_ERROR_DMA
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_DMA
  *         @arg @ref HAL_SMARTCARD_RECEIVE_ERROR_RTO
  */
uint32_t HAL_SMARTCARD_GetLastErrorCodes(const hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  return hsmartcard->last_error_codes;
}
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */

/** @brief  Report the SMARTCARD clock frequency from the RCC configuration.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the USART instance.
  * @retval uint32_t clock frequency
  */
uint32_t HAL_SMARTCARD_GetClockFreq(const hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx;
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  ASSERT_DBG_STATE(hsmartcard->global_state, (uint32_t)(HAL_SMARTCARD_STATE_IDLE | HAL_SMARTCARD_STATE_RX_ACTIVE
                                                        | HAL_SMARTCARD_STATE_TX_ACTIVE | HAL_SMARTCARD_STATE_ABORT));
  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  return HAL_RCC_USART_GetKernelClkFreq(p_smartcardx);
}

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup SMARTCARD_Exported_Functions_Group11 Acquire/Release Bus functions
  * @{
  This subsection provides functions allowing to control the bus of the USARTx instance:
    - HAL_SMARTCARD_AcquireBus(): Acquire the bus
    - HAL_SMARTCARD_ReleaseBus(): Release the bus.

  For multi task application, it is strongly recommended to use the bus operation functions to avoid race concurrency.
  */
/**
  * @brief Acquire the current instance bus.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param  timeout_ms Timeout in milliseconds for the Acquire to expire.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SMARTCARD_AcquireBus(hal_smartcard_handle_t *hsmartcard, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hsmartcard != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreTake(&hsmartcard->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief Release the current instance bus.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SMARTCARD_ReleaseBus(hal_smartcard_handle_t *hsmartcard)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hsmartcard != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreRelease(&hsmartcard->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /*USE_HAL_MUTEX */

#if defined (USE_HAL_SMARTCARD_USER_DATA) && (USE_HAL_SMARTCARD_USER_DATA == 1)

/** @addtogroup SMARTCARD_Exported_Functions_Group12 UserData functions
  * @{
  This subsection provides functions allowing to set user specific data to a SMARTCARDx instance:
  - HAL_SMARTCARD_SetUserData(): Set user data in handler.
  - HAL_SMARTCARD_GetUserData(): Get user data from handler.
  */

/**
  * @brief Store User Data pointer into the handle.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param p_user_data Pointer to the user data.
  */
void HAL_SMARTCARD_SetUserData(hal_smartcard_handle_t *hsmartcard, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  hsmartcard->p_user_data = p_user_data;
}

/**
  * @brief Retrieve User Data pointer from the handle.
  * @param hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @retval Pointer to the user data.
  */
const void *HAL_SMARTCARD_GetUserData(const hal_smartcard_handle_t *hsmartcard)
{
  ASSERT_DBG_PARAM(hsmartcard != NULL);

  return (hsmartcard->p_user_data);
}

/**
  * @}
  */
#endif /* USE_HAL_SMARTCARD_USER_DATA */

/**
  * @}
  */

/** @addtogroup SMARTCARD_Private_Functions SMARTCARD Private Functions
  * @{
  */
/**
  * @brief  Abort smartcard communication by disabling interruptions.
  * @param  hsmartcard SMARTCARD handle.
  */
static void SMARTCARD_Abort(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Disable RTOIE, EOBIE, TXEIE, TCIE, RXNE, PE, RXFT, TXFT and
     ERR (Frame error, noise error, overrun error) interrupts */
  LL_USART_DisableIT_CR1(p_smartcardx, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE
                                        | USART_CR1_TCIE | USART_CR1_RTOIE | USART_CR1_EOBIE));
  LL_USART_DisableIT_CR3(p_smartcardx, (USART_CR3_EIE | USART_CR3_RXFTIE | USART_CR3_TXFTIE));
}

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  Initialize the callbacks to their default values.
  * @param  hsmartcard SMARTCARD handle.
  */
static void SMARTCARD_InitCallbacksToDefault(hal_smartcard_handle_t *hsmartcard)
{
  /* Init the SMARTCARD Callback settings */
  hsmartcard->p_tx_cplt_callback          = HAL_SMARTCARD_TxCpltCallback;     /* Legacy weak p_tx_cplt_callback      */
  hsmartcard->p_rx_cplt_callback          = HAL_SMARTCARD_RxCpltCallback;     /* Legacy weak p_rx_cplt_callback      */
  hsmartcard->p_tx_half_cplt_callback     = HAL_SMARTCARD_TxHalfCpltCallback; /* Legacy weak p_tx_half_cplt_callback */
  hsmartcard->p_rx_half_cplt_callback     = HAL_SMARTCARD_RxHalfCpltCallback; /* Legacy weak p_rx_half_cplt_callback */
  hsmartcard->p_error_callback            = HAL_SMARTCARD_ErrorCallback;      /* Legacy weak p_error_callback        */
  hsmartcard->p_abort_cplt_callback       = HAL_SMARTCARD_AbortCpltCallback;  /* Legacy weak p_abort_cplt_callback   */

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  hsmartcard->p_rx_fifo_full_callback     = HAL_SMARTCARD_RxFifoFullCallback;      /* Legacy weak
                                                                                      p_rx_fifo_full_callback */
  hsmartcard->p_tx_fifo_empty_callback    = HAL_SMARTCARD_TxFifoEmptyCallback;     /* Legacy weak
                                                                                      p_tx_fifo_empty_callback */
#endif /* USE_HAL_SMARTCARD_FIFO */

}
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @brief  Handle SMARTCARD Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @param  flag Specifies the SMARTCARD flag to check.
  * @param  status The actual Flag status (1U or 0U).
  * @param  tickstart Tick start value
  * @param  timeout_ms Timeout duration.
  * @retval HAL status
  */
static hal_status_t SMARTCARD_WaitOnFlagUntilTimeout(hal_smartcard_handle_t *hsmartcard, uint32_t flag,
                                                     uint32_t status, uint32_t tickstart, uint32_t timeout_ms)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Wait until flag is set */
  while (LL_USART_IsActiveFlag(p_smartcardx, flag) == status)
  {
    /* Check for the Timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        if (LL_USART_IsActiveFlag(p_smartcardx, flag) == status)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error)
            interrupts for the interrupt process */
          LL_USART_DisableIT_CR1(p_smartcardx, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE));
          LL_USART_DisableIT_ERROR(p_smartcardx);

          hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Start Transmit operation in interrupt mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param  p_data Pointer to data buffer (u8 data elements).
  * @param  size Amount of data elements (u8) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL SMARTCARD API providing transmission in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         SMARTCARD Handle is assumed as Locked.
  * @retval HAL_OK Transmit started in IT mode.
  */
hal_status_t SMARTCARD_Start_Transmit_IT(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint32_t size,
                                         uint32_t interrupts)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->p_tx_buff         = p_data;
  hsmartcard->tx_xfer_size      = size;
  hsmartcard->tx_xfer_count     = size;
  hsmartcard->p_tx_isr       = NULL;
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */

  /* In case of TX only mode, if NACK is enabled, the USART must be able to monitor
      the bidirectional line to detect a NACK signal in case of parity error.
      Therefore, the receiver block must be enabled as well (RE bit in register CR1 must be set). */
  if (LL_USART_IsEnabledSmartcardNACK(p_smartcardx) != 0U)
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX_RX);
  }
  else
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX);
  }

  LL_USART_Enable(p_smartcardx);

  /* Perform a TX/RX FIFO Flush */
  SMARTCARD_FLUSH_DRREGISTER(hsmartcard);

  /* Configure Tx interrupt processing */
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)

  if (hsmartcard->fifo_status == HAL_SMARTCARD_FIFO_MODE_ENABLED)
  {
    hsmartcard->p_tx_isr = SMARTCARD_TxISR_FIFOEN;

    LL_USART_EnableIT_ERROR(p_smartcardx);

    LL_USART_EnableIT_TXFT(p_smartcardx);
  }
  else
  {
    hsmartcard->p_tx_isr = SMARTCARD_TxISR;

    LL_USART_EnableIT_ERROR(p_smartcardx);

    LL_USART_EnableIT_TXE_TXFNF(p_smartcardx);
  }

  if ((interrupts & HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY) == HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY)
  {
    LL_USART_EnableIT_TXFE(p_smartcardx);
  }
#else /* USE_HAL_SMARTCARD_FIFO */

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(interrupts);

  hsmartcard->p_tx_isr = SMARTCARD_TxISR;

  LL_USART_EnableIT_ERROR(p_smartcardx);
  LL_USART_EnableIT_TXE_TXFNF(p_smartcardx);
#endif /* USE_HAL_SMARTCARD_FIFO */

  return HAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param  p_data Pointer to data buffer (u8 data elements).
  * @param  size Amount of data elements (u8) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL SMARTCARD API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         SMARTCARD Handle is assumed as Locked.
  * @retval HAL_OK Receive started in IT mode.
  */
hal_status_t SMARTCARD_Start_Receive_IT(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint32_t size,
                                        uint32_t interrupts)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_RX_ACTIVE;

  hsmartcard->p_rx_buff = p_data;
  hsmartcard->rx_xfer_size = size;
  hsmartcard->rx_xfer_count = size;
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
  LL_USART_EnableDirectionRx(p_smartcardx);

  LL_USART_Enable(p_smartcardx);

  /* Configure Rx interrupt processing */
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  if ((hsmartcard->fifo_status == HAL_SMARTCARD_FIFO_MODE_ENABLED) && (size >= hsmartcard->nb_rx_data_to_process))
  {
    hsmartcard->p_rx_isr = SMARTCARD_RxISR_FIFOEN;

    LL_USART_EnableIT_PE(p_smartcardx);
    LL_USART_EnableIT_RXFT(p_smartcardx);
  }
  else
  {
    hsmartcard->p_rx_isr = SMARTCARD_RxISR;

    LL_USART_EnableIT_RXNE_RXFNE(p_smartcardx);
    LL_USART_EnableIT_PE(p_smartcardx);
  }

  if ((interrupts & HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL) == HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL)
  {
    LL_USART_EnableIT_RXFF(p_smartcardx);
  }
#else
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(interrupts);

  hsmartcard->p_rx_isr = SMARTCARD_RxISR;

  LL_USART_EnableIT_RXNE_RXFNE(p_smartcardx);
  LL_USART_EnableIT_PE(p_smartcardx);
#endif /* USE_HAL_SMARTCARD_FIFO */

  LL_USART_EnableIT_ERROR(p_smartcardx);

  return HAL_OK;
}

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)

/**
  * @brief  Start Transmit operation in DMA mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL SMARTCARD API providing transmission in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         SMARTCARD Handle is assumed as Locked.
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t SMARTCARD_Start_Transmit_DMA(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint32_t size,
                                          uint32_t interrupts)
{
  USART_TypeDef *p_smartcardx;
  uint32_t interrupts_dma;

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  hsmartcard->p_tx_buff     = p_data;
  hsmartcard->tx_xfer_size  = size;
  hsmartcard->tx_xfer_count = size;
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
  interrupts_dma = (interrupts & HAL_SMARTCARD_OPT_DMA_TX_IT_HT);

  /* In case of TX only mode, if NACK is enabled, the USART must be able to monitor
      the bidirectional line to detect a NACK signal in case of parity error.
      Therefore, the receiver block must be enabled as well (RE bit must be set). */
  if (LL_USART_IsEnabledSmartcardNACK(p_smartcardx) != 0U)
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX_RX);
  }
  else
  {
    LL_USART_SetTransferDirection(p_smartcardx, LL_USART_DIRECTION_TX);
  }

  LL_USART_Enable(p_smartcardx);

  /* Perform a TX/RX FIFO Flush */
  SMARTCARD_FLUSH_DRREGISTER(hsmartcard);

  hsmartcard->hdma_tx->p_xfer_cplt_cb = SMARTCARD_DMATransmitCplt;

  hsmartcard->hdma_tx->p_xfer_halfcplt_cb = SMARTCARD_DMATxHalfCplt;

  hsmartcard->hdma_tx->p_xfer_error_cb = SMARTCARD_DMAError;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hsmartcard->hdma_tx, (uint32_t)hsmartcard->p_tx_buff, (uint32_t)&p_smartcardx->TDR,
                                     size, interrupts_dma) != HAL_OK)
  {
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
    hsmartcard->last_error_codes |= HAL_SMARTCARD_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_ClearFlag_TC(p_smartcardx);
  LL_USART_EnableIT_ERROR(p_smartcardx);
  LL_USART_EnableDMAReq_TX(p_smartcardx);

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  if (((interrupts & HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY) == HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY))
  {
    LL_USART_EnableIT_TXFE(p_smartcardx);
  }
#endif /* USE_HAL_SMARTCARD_FIFO */

  return HAL_OK;
}

/**
  * @brief  Start Receive operation in DMA mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL SMARTCARD API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         SMARTCARD Handle is assumed as Locked.
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t SMARTCARD_Start_Receive_DMA(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                         uint32_t size, uint32_t interrupts)
{
  USART_TypeDef *p_smartcardx;
  uint32_t interrupts_dma;

  p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  LL_USART_EnableDirectionRx(p_smartcardx);

  LL_USART_Enable(p_smartcardx);

  hsmartcard->p_rx_buff = p_data;
  hsmartcard->rx_xfer_size = size;
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  hsmartcard->last_error_codes = HAL_SMARTCARD_ERROR_NONE;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
  interrupts_dma = (interrupts & HAL_SMARTCARD_OPT_DMA_RX_IT_HT);

  hsmartcard->hdma_rx->p_xfer_cplt_cb = SMARTCARD_DMAReceiveCplt;

  hsmartcard->hdma_rx->p_xfer_halfcplt_cb = SMARTCARD_DMARxHalfCplt;

  hsmartcard->hdma_rx->p_xfer_error_cb = SMARTCARD_DMAError;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hsmartcard->hdma_rx, (uint32_t)&p_smartcardx->RDR,
                                     (uint32_t)hsmartcard->p_rx_buff, size, interrupts_dma) != HAL_OK)
  {
#if defined (USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
    hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_EnableIT_PE(p_smartcardx);
  LL_USART_EnableIT_ERROR(p_smartcardx);
  LL_USART_EnableDMAReq_RX(p_smartcardx);

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  if (((interrupts & HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL) == HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL))
  {
    LL_USART_EnableIT_RXFF(p_smartcardx);
  }
#endif /* USE_HAL_SMARTCARD_FIFO */
  return HAL_OK;
}
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @brief  End ongoing Tx transfer on SMARTCARD peripheral (following error detection or Transmit completion).
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  */
static void SMARTCARD_EndTxTransfer(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Disable TXEIE, TCIE and ERR (Frame error, noise error, overrun error) interrupts */
  LL_USART_DisableIT_CR1(p_smartcardx, (USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE));
  LL_USART_DisableIT_ERROR(p_smartcardx);

  LL_USART_DisableDMAReq_TX(p_smartcardx);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;
}

/**
  * @brief  End ongoing Rx transfer on SMARTCARD peripheral (following error detection or Reception completion).
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  */
static void SMARTCARD_EndRxTransfer(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  LL_USART_DisableIT_CR1(p_smartcardx, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));
  LL_USART_DisableIT_ERROR(p_smartcardx);

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /* if Rx FIFO full Optional IT have been activated, clear status */
  LL_USART_DisableIT_RXFT(p_smartcardx);
  if (LL_USART_IsEnabledIT_RXFF(p_smartcardx) != 0U)
  {
    LL_USART_DisableIT_RXFF(p_smartcardx);
  }
#endif /* USE_HAL_SMARTCARD_FIFO */

  LL_USART_DisableDMAReq_RX(p_smartcardx);


  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;
}

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/**
  * @brief DMA SMARTCARD receive process half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void SMARTCARD_DMATxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_rx_half_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_RxHalfCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SMARTCARD transmit process complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void SMARTCARD_DMATransmitCplt(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->tx_xfer_count = 0U;

  /* Disable the DMA transfer for transmit request by resetting the DMAT bit
  in the SMARTCARD associated USART CR3 register */
  LL_USART_DisableDMAReq_TX(p_smartcardx);

  /* Enable the SMARTCARD Transmit Complete Interrupt */
  if (hsmartcard->tx_cplt_indication != HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME)
  {
    LL_USART_EnableIT_TCBGT(p_smartcardx);
  }
  else
  {
    LL_USART_EnableIT_TC(p_smartcardx);
  }
}

/**
  * @brief DMA SMARTCARD receive process half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void SMARTCARD_DMARxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_rx_half_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_RxHalfCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SMARTCARD receive process complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void SMARTCARD_DMAReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  hsmartcard->rx_xfer_count = 0U;

  /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
  LL_USART_DisableIT_PE(p_smartcardx);
  LL_USART_DisableIT_ERROR(p_smartcardx);

  /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
     in the SMARTCARD associated USART CR3 register */
  LL_USART_DisableDMAReq_RX(p_smartcardx);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_rx_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_RxCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SMARTCARD communication error callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void SMARTCARD_DMAError(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  /* Stop SMARTCARD DMA Tx request if ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
  {
    hsmartcard->tx_xfer_count = 0U;
#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
    hsmartcard->last_error_codes |= HAL_SMARTCARD_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    SMARTCARD_EndTxTransfer(hsmartcard);
  }

  /* Stop SMARTCARD DMA Rx request if ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_RX_ACTIVE)
  {
    hsmartcard->rx_xfer_count = 0U;
#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
    hsmartcard->last_error_codes |= HAL_SMARTCARD_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
    SMARTCARD_EndRxTransfer(hsmartcard);
  }
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_error_callback(hsmartcard);
#else
  HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SMARTCARD communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  */
static void SMARTCARD_DMAAbortOnError(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  hsmartcard->tx_xfer_count = 0U;
  hsmartcard->rx_xfer_count = 0U;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_error_callback(hsmartcard);
#else
  HAL_SMARTCARD_ErrorCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @brief  Send an amount of data in non-blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_SMARTCARD_Transmit_IT()
  *         and when the FIFO mode is disabled.
  */
static void SMARTCARD_TxISR(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Check that a Tx process is ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
  {
    if (hsmartcard->tx_xfer_count == 0U)
    {
      LL_USART_DisableIT_TXE_TXFNF(p_smartcardx);

      /* Enable the SMARTCARD Transmit Complete Interrupt */
      if (hsmartcard->tx_cplt_indication != HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME)
      {
        LL_USART_EnableIT_TCBGT(p_smartcardx);
      }
      else
      {
        LL_USART_EnableIT_TC(p_smartcardx);
      }
    }
    else
    {
      LL_USART_TransmitData8(p_smartcardx, (uint8_t)(*hsmartcard->p_tx_buff & 0xFFU));
      hsmartcard->p_tx_buff++;
      hsmartcard->tx_xfer_count--;
    }
  }
}

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Send an amount of data in non-blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_SMARTCARD_Transmit_IT()
  *         and when the FIFO mode is enabled.
  */
static void SMARTCARD_TxISR_FIFOEN(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  uint16_t nb_tx_data;

  /* Check that a Tx process is ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
  {
    for (nb_tx_data = hsmartcard->nb_tx_data_to_process ; nb_tx_data > 0U ; nb_tx_data--)
    {
      if (hsmartcard->tx_xfer_count == 0U)
      {
        LL_USART_DisableIT_TXE_TXFNF(p_smartcardx);
        LL_USART_DisableIT_TXFT(p_smartcardx);

        /* Enable the SMARTCARD Transmit Complete Interrupt */
        if (hsmartcard->tx_cplt_indication != HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME)
        {
          LL_USART_EnableIT_TCBGT(p_smartcardx);
        }
        else
        {
          LL_USART_EnableIT_TC(p_smartcardx);
        }
      }
      else if (LL_USART_IsActiveFlag_TXE_TXFNF(p_smartcardx) != 0U)
      {
        LL_USART_TransmitData8(p_smartcardx, (uint8_t)(*hsmartcard->p_tx_buff & 0xFFU));
        hsmartcard->p_tx_buff++;
        hsmartcard->tx_xfer_count--;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }
}

#endif /* USE_HAL_SMARTCARD_FIFO */

/**
  * @brief  Wrap up transmission in non-blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  */
static void SMARTCARD_EndTransmit_IT(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  uint32_t nack_enabled = LL_USART_IsEnabledSmartcardNACK(p_smartcardx);
  /* Disable the SMARTCARD Transmit Complete Interrupt */
  if (hsmartcard->tx_cplt_indication != HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME)
  {
    LL_USART_DisableIT_TCBGT(p_smartcardx);
  }
  else
  {
    LL_USART_DisableIT_TC(p_smartcardx);
  }

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /* if Tx FIFO empty or Rx FIFO Full Optional IT have been activated, clear status */
  if (LL_USART_IsEnabledIT_TXFE(p_smartcardx) != 0U)
  {
    LL_USART_DisableIT_TXFE(p_smartcardx);
    LL_USART_ClearFlag_TXFE(p_smartcardx);
  }
  if (LL_USART_IsEnabledIT_RXFF(p_smartcardx) != 0U)
  {
    LL_USART_DisableIT_RXFF(p_smartcardx);
  }
#endif /* USE_HAL_SMARTCARD_FIFO */

  /* Disable the Peripheral first to update mode */
  if ((hsmartcard->global_state == HAL_SMARTCARD_STATE_TX_ACTIVE)
      && (nack_enabled != 0U))
  {
    /* In case of NACK enabled, USART is disabled to empty RDR register */
    LL_USART_Disable(p_smartcardx);
    LL_USART_Enable(p_smartcardx);

    /* In case of TX only mode, if NACK is enabled, receiver block has been enabled
       for Transmit phase. Disable this receiver block. */
    LL_USART_DisableDirectionRx(p_smartcardx);

    /* Perform a TX FIFO Flush at end of Tx phase, as all sent bytes are appearing in Rx Data register */
    SMARTCARD_FLUSH_DRREGISTER(hsmartcard);
  }

  hsmartcard->p_tx_isr = NULL;

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_tx_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_TxCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/**
  * @brief  DMA SMARTCARD Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @param  hdma DMA handle.
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  */
static void SMARTCARD_DMATxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  /* Check if an Abort process is still ongoing */
  if (hsmartcard->hdma_rx != NULL)
  {
    if (hsmartcard->hdma_rx->global_state == HAL_DMA_STATE_ABORT)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hsmartcard->tx_xfer_count = 0U;

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF
                     | LL_USART_ICR_FECF | LL_USART_ICR_RTOCF | LL_USART_ICR_EOBCF);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_abort_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_AbortCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SMARTCARD Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @param  hdma DMA handle.
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  */
static void SMARTCARD_DMARxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_smartcard_handle_t *hsmartcard = (hal_smartcard_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);

  /* Check if an Abort process is still ongoing */
  if (hsmartcard->hdma_tx != NULL)
  {
    if (hsmartcard->hdma_tx->global_state == HAL_DMA_STATE_ABORT)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hsmartcard->rx_xfer_count = 0U;

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_smartcardx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF
                     | LL_USART_ICR_FECF | LL_USART_ICR_RTOCF | LL_USART_ICR_EOBCF);

  hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  hsmartcard->p_abort_cplt_callback(hsmartcard);
#else
  HAL_SMARTCARD_AbortCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
}
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @brief  Receive an amount of data in non-blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_SMARTCARD_Receive_IT()
  *         and when the FIFO mode is disabled.
  */
static void SMARTCARD_RxISR(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  /* Check that a Rx process is ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_RX_ACTIVE)
  {
    *hsmartcard->p_rx_buff = LL_USART_ReceiveData8(p_smartcardx);
    hsmartcard->p_rx_buff++;

    hsmartcard->rx_xfer_count--;
    if (hsmartcard->rx_xfer_count == 0U)
    {
      LL_USART_DisableIT_RXNE_RXFNE(p_smartcardx);

      /* Check if a transmit process is ongoing or not. If not disable ERR IT */
      if (hsmartcard->global_state == HAL_SMARTCARD_STATE_IDLE)
      {
        LL_USART_DisableIT_ERROR(p_smartcardx);
      }

      LL_USART_DisableIT_PE(p_smartcardx);

      hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

      /* Clear RxISR function pointer */
      hsmartcard->p_rx_isr = NULL;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
      hsmartcard->p_rx_cplt_callback(hsmartcard);
#else
      HAL_SMARTCARD_RxCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_smartcardx);
  }
}

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/**
  * @brief  Receive an amount of data in non-blocking mode.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure that contains
  *                    the configuration information for the specified SMARTCARD module.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_SMARTCARD_Receive_IT()
  *         and when the FIFO mode is enabled.
  */
static void SMARTCARD_RxISR_FIFOEN(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  uint16_t nb_rx_data;
  uint32_t rxdatacount;

  /* Check that a Rx process is ongoing */
  if (hsmartcard->global_state == HAL_SMARTCARD_STATE_RX_ACTIVE)
  {
    for (nb_rx_data = hsmartcard->nb_rx_data_to_process ; nb_rx_data > 0U ; nb_rx_data--)
    {
      *hsmartcard->p_rx_buff = LL_USART_ReceiveData8(p_smartcardx);
      hsmartcard->p_rx_buff++;

      hsmartcard->rx_xfer_count--;
      if (hsmartcard->rx_xfer_count == 0U)
      {
        LL_USART_DisableIT_CR1(p_smartcardx, USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE);
        LL_USART_DisableIT_RXFT(p_smartcardx);

        /* Check if a transmit process is ongoing or not. If not disable ERR IT */
        if (hsmartcard->global_state == HAL_SMARTCARD_STATE_IDLE)
        {
          LL_USART_DisableIT_ERROR(p_smartcardx);
        }

        hsmartcard->global_state = HAL_SMARTCARD_STATE_IDLE;

        /* Clear RxISR function pointer */
        hsmartcard->p_rx_isr = NULL;

#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
        hsmartcard->p_rx_cplt_callback(hsmartcard);
#else
        HAL_SMARTCARD_RxCpltCallback(hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
      }
    }

    /* When remaining number of bytes to receive is less than the RX FIFO
    threshold, next incoming frames are processed as if FIFO mode was
    disabled (i.e. one interrupt per received frame).
    */
    rxdatacount = hsmartcard->rx_xfer_count;
    if (((rxdatacount != 0U)) && (rxdatacount < hsmartcard->nb_rx_data_to_process))
    {
      LL_USART_DisableIT_RXFT(p_smartcardx);

      /* Update the RxISR function pointer */
      hsmartcard->p_rx_isr = SMARTCARD_RxISR;

      LL_USART_EnableIT_RXNE_RXFNE(p_smartcardx);
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_smartcardx);
  }
}
#endif /* USE_HAL_SMARTCARD_FIFO */

/** @brief  Flush the SMARTCARD Data registers.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the USART instance.
  */
static void SMARTCARD_FLUSH_DRREGISTER(hal_smartcard_handle_t *hsmartcard)
{
  USART_TypeDef *p_smartcardx = SMARTCARD_GET_INSTANCE(hsmartcard);
  do
  {
    LL_USART_SetRequest(p_smartcardx, LL_USART_REQUEST_RX_DATA_FLUSH);
    LL_USART_SetRequest(p_smartcardx, LL_USART_REQUEST_TX_DATA_FLUSH);
  } while (0U);
}

#if defined(USE_HAL_SMARTCARD_CLK_ENABLE_MODEL) && (USE_HAL_SMARTCARD_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
/** @brief  Set the SMARTCARD clock frequency.
  * @param  hsmartcard Pointer to a \ref hal_smartcard_handle_t structure which contains the SMARTCARD instance.
  */
static void SMARTCARD_EnableClock(const hal_smartcard_handle_t *hsmartcard)
{
  /*! Instance USART1 */
  if (hsmartcard->instance == HAL_SMARTCARD1)
  {
    HAL_RCC_USART1_EnableClock();
  }
  /*! Instance USART2 */
  if (hsmartcard->instance == HAL_SMARTCARD2)
  {
    HAL_RCC_USART2_EnableClock();
  }
#if defined(USART3)
  /*! Instance USART3 */
  if (hsmartcard->instance == HAL_SMARTCARD3)
  {
    HAL_RCC_USART3_EnableClock();
  }
#endif /* USART3 */
#if defined(USART6)
  /*! Instance USART6 */
  if (hsmartcard->instance == HAL_SMARTCARD6)
  {
    HAL_RCC_USART6_EnableClock();
  }
#endif /* USART6 */
}
#endif /* USE_HAL_SMARTCARD_CLK_ENABLE_MODEL */

#if defined(USE_ASSERT_DBG_PARAM)
/**
  * @brief Calculate and check baudrate validity.
  * @param instance_clock_freq Clock frequency of the uart instance used
  * @param instance_clock_prescaler Clock prescaler of the uart instance used
  * @param baud_rate Baud rate to be tested
  * @retval HAL_OK baudrate value is valid
  * @retval HAL_ERROR baudrate value is invalid
  */
hal_status_t SMARTCARD_Check_uart_baudrate_validity(uint32_t instance_clock_freq, uint32_t instance_clock_prescaler,
                                                    uint32_t baud_rate)
{
  uint32_t div_temp;
  div_temp = LL_USART_DIV_SAMPLING16(instance_clock_freq, instance_clock_prescaler, baud_rate);
  if ((div_temp >= USART_BRR_MIN) && (div_temp <= USART_BRR_MAX))
  {
    return HAL_OK;
  }
  return HAL_ERROR;
}
#endif /* USE_ASSERT_DBG_PARAM */

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_HAL_SMARTCARD_MODULE */
#endif /* USART1 || USART2 || USART3 || UART4 || UART5 || USART6 || UART7 */
/**
  * @}
  */
