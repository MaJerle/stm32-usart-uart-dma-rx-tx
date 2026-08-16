/**
  ******************************************************************************
  * @file    stm32c5xx_hal_uart.c
  * @brief   UART HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Asynchronous Receiver Transmitter Peripheral (UART).
  *           + Initialization and deinitialization functions
  *           + I/O operation functions
  *           + Peripheral control functions.
  *
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
 || defined(UART7) || defined (LPUART1)
/** @addtogroup UART
  * @{
  */

/** @defgroup UART_Introduction UART Introduction
  * @{
   - The UART hardware abstraction layer provides a set of APIs to interface with STM32 peripherals such as **UART**
     (Universal Asynchronous Receiver Transmitter), **LPUART** (Low-Power UART) and **USART** (Universal
     Synchronous/Asynchronous Receiver Transmitter) supporting the UART communications (asynchronous, half-duplex
     single-wire, multiprocessor, LIN, ModBus, ...).

  - It simplifies the configuration, initialization, and management of asynchronous UART communications by supporting
    various modes, such as polling, interrupt, and DMA, for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series, as well as multiple
    peripherals supporting the UART communications (**USART**, **UART**, and **LPUART**).
  * @}
  */

/** @defgroup UART_How_To_Use UART How To Use
  * @{

  # How to use the UART HAL module driver

    @note In the following documentation, consider USARTx as a placeholder for every UART instance,
      USART instance, and LPUART instance.

Use the UART HAL driver as follows:

## 1- Declare a hal_uart_handle_t handle structure, for example:
   hal_uart_handle_t huart;

## 2- Configure the low-level hardware (GPIO, CLOCK, NVIC, etc.):
    - Enable the USARTx interface clock if you have not set USE_HAL_UART_CLK_ENABLE_MODEL to HAL_CLK_ENABLE_PERIPH_ONLY
      or HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM (in those cases HAL_UART_Init() will enable the clock).
    - UART pins configuration:
        - Enable the clock for the UART GPIOs
        - Configure these UART pins as alternate function.
    - Configure the NVIC when using the interrupt process (HAL_UART_Transmit_IT() and HAL_UART_Receive_IT(), ...
      APIs):
        - Configure the USARTx interrupt priority.
        - Enable the NVIC USARTx IRQ Channel.
    - Configure the DMA when using the DMA process (HAL_UART_Transmit_DMA() and HAL_UART_Receive_DMA(), ... APIs):
        - Declare a DMA handle structure for the Tx or Rx channel.
        - Enable the DMAx interface clock.
        - Configure the declared DMA handle structure with the required Tx or Rx parameters.
        - Associate the initialized DMA handle with the UART handle with HAL_UART_SetTxDMA() or HAL_UART_SetRxDMA().
        - For each DMA channel (Tx and Rx), configure the corresponding NVIC line priority and enable it.
      @note In DMA Tx configuration, also enable the USARTx IRQ to complete the DMA transfer.

## 3- Initialize the UART driver by selecting a USARTx instance and calling HAL_UART_Init().
  Depending on USE_HAL_UART_CLK_ENABLE_MODEL, HAL_UART_Init() can enable the USARTx clock.
  For example:
      HAL_UART_Init(&huart, HAL_UART1);

## 4- Declare a hal_uart_config_t structure, fill it, and then call HAL_UART_SetConfig(). For example:
        hal_uart_config_t my_config;

   In the configuration structure,
    Program the baud rate, word length, stop bit, parity, prescaler value, hardware
    flow control, direction (Receiver/Transmitter), oversampling, and one-bit sampling.

  Apply the configuration by calling HAL_UART_SetConfig(&huart, &my_config).

## 5- If required, enable a specific mode on the UART:
  - Half-duplex mode with HAL_UART_EnableHalfDuplexMode()
  - Multiprocessor mode with HAL_UART_EnableMultiProcessorMode()
    - LIN mode with HAL_UART_EnableLINMode()
    - RS-485 mode with HAL_UART_EnableRS485Mode()

  Or call UART advanced features (TX/RX pins swap, auto baud rate detection, ...) with a set of
  different configuration functions.

## 6- Transfer APIs (Transmit and Receive)

  For USARTx I/O operations, polling, interrupt, and DMA are available within this driver.

  - Polling mode I/O operation
    - Send an amount of data in blocking mode using HAL_UART_Transmit()
    - Receive an amount of data in blocking mode using HAL_UART_Receive()
    - The communication is performed in polling mode. The HAL status for all data processing is returned by the same
      function after the transfer finishes or the timeout expires.

  - Interrupt mode I/O operation
    - Send an amount of data in non-blocking mode using HAL_UART_Transmit_IT() or HAL_UART_Transmit_IT_Opt()
    - At the end of transmission, HAL_UART_TxCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_TxCpltCallback().
    - Receive an amount of data in non-blocking mode using HAL_UART_Receive_IT() or HAL_UART_Receive_IT_Opt()
    - At the end of reception, HAL_UART_RxCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_RxCpltCallback().
    - In case of transfer error, HAL_UART_ErrorCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_ErrorCallback().
    - For interrupt transfers, call HAL_UART_IRQHandler() inside the USARTx IRQ handler

  - DMA mode I/O operation
    - Send an amount of data in non-blocking mode (DMA) using HAL_UART_Transmit_DMA() or HAL_UART_Transmit_DMA_Opt()
    - At the transmission half-transfer, HAL_UART_TxHalfCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_TxHalfCpltCallback().
    - At the end of transmission, HAL_UART_TxCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_TxCpltCallback().
    - Receive an amount of data in non-blocking mode (DMA) using HAL_UART_Receive_DMA() or HAL_UART_Receive_DMA_Opt()
    - At the reception half-transfer, HAL_UART_RxHalfCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_RxHalfCpltCallback().
    - At the end of reception, HAL_UART_RxCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_RxCpltCallback().
    - In case of transfer error, HAL_UART_ErrorCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_ErrorCallback().
    - Before starting DMA transfers, associate the DMA handle(s) with the UART handle using HAL_UART_SetTxDMA()
      and/or HAL_UART_SetRxDMA()

  - Advanced receive operations (optional)
    - Receive data until IDLE condition: HAL_UART_ReceiveToIdle(), HAL_UART_ReceiveToIdle_IT()
    - Receive data until timeout: HAL_UART_ReceiveUntilTMO(), HAL_UART_ReceiveUntilTMO_IT()
    - Receive data until character match: HAL_UART_ReceiveUntilCM(), HAL_UART_ReceiveUntilCM_IT()
    - When USE_HAL_UART_DMA is set to 1, also use the *_DMA() and *_DMA_Opt() variants

  - The sequential UART interface abort operations are listed below:
    - Abort a polling UART process communication using HAL_UART_Abort()
    - Abort an IT UART process communication with Interrupt using HAL_UART_Abort_IT()
    - Abort a transmit process using HAL_UART_AbortTransmit() or HAL_UART_AbortTransmit_IT()
    - Abort a receive process using HAL_UART_AbortReceive() or HAL_UART_AbortReceive_IT()
    - At the end of the abort IT process, HAL_UART_AbortCpltCallback() is executed, and you can add your own code by
      customizing the function pointer HAL_UART_AbortCpltCallback().

## 7- Callback registration
  When the compilation define USE_HAL_UART_REGISTER_CALLBACKS is set to 1U, configure the driver callbacks
  dynamically via your own method:

  Callback name               | Default value                       | Callback registration function
  ----------------------------| ----------------------------------- | ---------------------------
  TxHalfCpltCallback          | HAL_UART_TxHalfCpltCallback()       | HAL_UART_RegisterTxHalfCpltCallback()
  TxCpltCallback              | HAL_UART_TxCpltCallback()           | HAL_UART_RegisterTxCpltCallback()
  RxHalfCpltCallback          | HAL_UART_RxHalfCpltCallback()       | HAL_UART_RegisterRxHalfCpltCallback()
  RxCpltCallback              | HAL_UART_RxCpltCallback()           | HAL_UART_RegisterRxCpltCallback()
  ErrorCallback               | HAL_UART_ErrorCallback()            | HAL_UART_RegisterErrorCallback()
  AbortCpltCallback           | HAL_UART_AbortCpltCallback()        | HAL_UART_RegisterAbortCpltCallback()
  AbortTransmitCpltCallback   | HAL_UART_AbortTransmitCpltCallback()| HAL_UART_RegisterAbortTransmitCpltCallback()
  AbortReceiveCpltCallback    | HAL_UART_AbortReceiveCpltCallback() | HAL_UART_RegisterAbortReceiveCpltCallback()
  WakeupCallback              | HAL_UART_WakeupCallback()           | HAL_UART_RegisterWakeupCallback()
  RxFifoFullCallback          | HAL_UART_RxFifoFullCallback()        | HAL_UART_RegisterRxFifoFullCallback()
  TxFifoEmptyCallback         | HAL_UART_TxFifoEmptyCallback()       | HAL_UART_RegisterTxFifoEmptyCallback()
  LINBreakCallback            | HAL_UART_LINBreakCallback()          | HAL_UART_RegisterLINBreakCallback()
  ClearToSendCallback         | HAL_UART_ClearToSendCallback()       | HAL_UART_RegisterClearToSendCallback()

  To unregister a callback, register the default callback via the registration function.

  By default, after the HAL_UART_Init() and when the state is HAL_UART_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Register callbacks when the handle global_state is HAL_UART_STATE_INIT or HAL_UART_STATE_CONFIGURED.

  When the compilation define USE_HAL_UART_REGISTER_CALLBACKS is set to 0 or not defined, the callback registration
  feature is not available and weak callbacks are used, represented by the default value in the table above.

## 8- Acquire/Release the HAL UART handle
  - When the compilation flag USE_HAL_MUTEX is set to 1, a multi-thread user application can acquire the
    whole UART HAL handle to execute a transmit or a receive process or a sequence of transmit/receive.
    When the given process or sequence ends, release the UART HAL handle.
    - The HAL acquire/release operations are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
      - Use HAL_UART_AcquireBus() to acquire the HAL UART handle.
      - Use HAL_UART_ReleaseBus() to release the HAL UART handle.
  */

/**
  * @}
  */

/** @defgroup UART_Configuration_Table UART Configuration Table
  * @{

## 9- Configuration inside the UART driver:

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags              |   Default value   | Comment
------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_UART_MODULE             |         1         | Enable HAL UART driver module
USE_HAL_UART_REGISTER_CALLBACKS |         0         | Allows you to define your own callback
USE_HAL_UART_DMA                |         1         | Enable DMA code inside the HAL UART
USE_HAL_CHECK_PARAM             |         0         | Enable runtime parameter checking
USE_HAL_UART_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE     |         0         | Enable atomicity of process state checking
USE_HAL_MUTEX                   |         0         | Enable semaphore creation for OS
USE_HAL_UART_USER_DATA          |         0         | Add user data inside the HAL UART handle
USE_HAL_UART_GET_LAST_ERRORS    |         0         | Enable retrieval of the last process error codes

Software configuration defined in preprocessor environment:
Preprocessor flags              |   Default value   | Comment
------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM            |    Not defined    | Enable parameter checking for HAL and LL
USE_ASSERT_DBG_STATE            |    Not defined    | Enable state checking for HAL

  */
/**
  * @}
  */

#if defined(USE_HAL_UART_MODULE) && (USE_HAL_UART_MODULE == 1)

/* Private constants ------------------------------------------------------------*/
/** @defgroup UART_Private_Constants UART Private Constants
  * @{
  */

/*! LPUART BRR minimum authorized value */
#define LPUART_BRR_MIN  0x300U
/*! LPUART BRR maximum authorized value */
#define LPUART_BRR_MAX  0xFFFFFU
/*! UART BRR minimum authorized value */
#define UART_BRR_MIN    0x10U
/*! UART BRR maximum authorized value */
#define UART_BRR_MAX    0xFFFFU

/*! UART mask for 9-bit data length used for RDR reading */
#define UART_RDR_MASK_9BITS                 0x1FFU
/*! UART mask for 8-bit data length used for RDR reading */
#define UART_RDR_MASK_8BITS                 0xFFU
/*! UART mask for 7-bit data length used for RDR reading */
#define UART_RDR_MASK_7BITS                 0x7FU
/*! UART mask for 6-bit data length used for RDR reading */
#define UART_RDR_MASK_6BITS                 0x3FU

/*! Timeout value for UART instance enabling checks */
#define UART_ENABLE_TIMEOUT_MS              100U

/*! UART RX FIFO depth */
#define UART_RX_FIFO_DEPTH 8U

/*! UART TX FIFO depth */
#define UART_TX_FIFO_DEPTH 8U

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup UART_Private_Variables UART Private variables
  * @{
  */
/*! UART Prescaler Table preset */
#if defined(USE_ASSERT_DBG_PARAM)
const uint16_t UARTPrescTable[16] = {1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U, 256U, 256U, 256U, 256U};
#endif /* USE_ASSERT_DBG_PARAM */
/**
  * @}
  */

/* Private macros -----------------------------------------------------------*/
/** @defgroup UART_Private_Macros UART Private Macros
  * @{
  */

/** @brief  Check UART Baud rate.
  * @param  baud_rate Baud rate specified by the user.
  *         The maximum Baud Rate is derived from the maximum clock on C5 (i.e. 144 MHz)
  *         divided by the smallest oversampling used on the USART (i.e. 8)
  * @retval SET (baud_rate is valid) or RESET (baud_rate is invalid)
  */
#define IS_UART_BAUD_RATE(baud_rate) ((baud_rate) <= 18000000U && ((baud_rate) != 0U))

/** @brief  Check UART assertion time.
  * @param  time 5-bit value assertion time.
  * @retval SET (time is valid) or RESET (time is invalid)
  */
#define IS_UART_ASSERTION_TIME(time) ((time) <= 0x1FU)

/** @brief  Check UART deassertion time.
  * @param  time 5-bit value deassertion time.
  * @retval SET (time is valid) or RESET (time is invalid)
  */
#define IS_UART_DEASSERTION_TIME(time) ((time) <= 0x1FU)

/** @brief  Check UART Receiver Timeout value.
  * @param  timeoutvalue Timeout value.
  * @retval SET (timeoutvalue is valid) or RESET (timeoutvalue is invalid)
  */
#define IS_UART_RECEIVER_TIMEOUT_VALUE(timeoutvalue) ((timeoutvalue) <= 0xFFFFFFU)

/**
  * @brief Ensure that the number of transferred data is valid.
  * @param datasize UART TX data size.
  * @retval SET (datasize is valid) or RESET (datasize is invalid)
  */
#define IS_UART_TX_DATA_SIZE(datasize) ((datasize) <= 0xFFFFU)

/**
  * @brief Ensure that UART frame length is valid.
  * @param length UART frame length.
  * @retval SET (length is valid) or RESET (length is invalid)
  */
#define IS_UART_WORD_LENGTH(length) (((length) == HAL_UART_WORD_LENGTH_7_BIT)     \
                                     || ((length) == HAL_UART_WORD_LENGTH_8_BIT)  \
                                     || ((length) == HAL_UART_WORD_LENGTH_9_BIT))

/**
  * @brief Ensure that UART frame number of stop bits is valid.
  * @param stopbits UART frame number of stop bits.
  * @retval SET (stopbits is valid) or RESET (stopbits is invalid)
  */
#define IS_UART_STOP_BITS(stopbits) (((stopbits) == HAL_UART_STOP_BIT_0_5)    \
                                     || ((stopbits) == HAL_UART_STOP_BIT_1)   \
                                     || ((stopbits) == HAL_UART_STOP_BIT_1_5) \
                                     || ((stopbits) == HAL_UART_STOP_BIT_2))

/**
  * @brief Ensure that LPUART frame number of stop bits is valid.
  * @param stopbits LPUART frame number of stop bits.
  * @retval SET (stopbits is valid) or RESET (stopbits is invalid)
  */
#define IS_LPUART_STOP_BITS(stopbits) (((stopbits) == HAL_UART_STOP_BIT_1)     \
                                       || ((stopbits) == HAL_UART_STOP_BIT_2))

/**
  * @brief Ensure that UART frame parity is valid.
  * @param parity UART frame parity.
  * @retval SET (parity is valid) or RESET (parity is invalid)
  */
#define IS_UART_PARITY(parity) (((parity) == HAL_UART_PARITY_NONE)     \
                                || ((parity) == HAL_UART_PARITY_EVEN)  \
                                || ((parity) == HAL_UART_PARITY_ODD))

/**
  * @brief Ensure that UART hardware flow control is valid.
  * @param flowcontrol UART hardware flow control.
  * @retval SET (flowcontrol is valid) or RESET (flowcontrol is invalid)
  */
#define IS_UART_HARDWARE_FLOW_CONTROL(flowcontrol) (((flowcontrol) == HAL_UART_HW_CONTROL_NONE)        \
                                                    || ((flowcontrol) == HAL_UART_HW_CONTROL_RTS)      \
                                                    || ((flowcontrol) == HAL_UART_HW_CONTROL_CTS)      \
                                                    || ((flowcontrol) == HAL_UART_HW_CONTROL_RTS_CTS))

/**
  * @brief Ensure that UART direction is valid.
  * @param direction UART direction.
  * @retval SET (direction is valid) or RESET (direction is invalid)
  */
#define IS_UART_DIRECTION(direction) (((direction) == HAL_UART_DIRECTION_RX)        \
                                      || ((direction) == HAL_UART_DIRECTION_TX)     \
                                      || ((direction) == HAL_UART_DIRECTION_TX_RX))

/**
  * @brief Ensure that UART oversampling is valid.
  * @param sampling UART oversampling.
  * @retval SET (sampling is valid) or RESET (sampling is invalid)
  */
#define IS_UART_OVERSAMPLING(sampling) (((sampling) == HAL_UART_OVERSAMPLING_16)    \
                                        || ((sampling) == HAL_UART_OVERSAMPLING_8))

/**
  * @brief Ensure that LPUART oversampling is valid.
  * @param sampling LPUART oversampling.
  * @retval SET (sampling is valid) or RESET (sampling is invalid)
  */
#define IS_LPUART_OVERSAMPLING(sampling) ((sampling) == HAL_UART_OVERSAMPLING_16)

/**
  * @brief Ensure that UART frame sampling is valid.
  * @param onebit UART frame sampling.
  * @retval SET (onebit is valid) or RESET (onebit is invalid)
  */
#define IS_UART_ONE_BIT_SAMPLE(onebit) (((onebit) == HAL_UART_ONE_BIT_SAMPLE_DISABLE)    \
                                        || ((onebit) == HAL_UART_ONE_BIT_SAMPLE_ENABLE))

/**
  * @brief Ensure that UART Prescaler is valid.
  * @param clockprescaler UART Prescaler value.
  * @retval SET (clockprescaler is valid) or RESET (clockprescaler is invalid)
  */
#define IS_UART_PRESCALER(clockprescaler) (((clockprescaler) == HAL_UART_PRESCALER_DIV1)       \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV2)    \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV4)    \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV6)    \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV8)    \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV10)   \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV12)   \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV16)   \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV32)   \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV64)   \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV128)  \
                                           || ((clockprescaler) == HAL_UART_PRESCALER_DIV256))

/**
  * @brief Ensure that the UART wakeup method is valid.
  * @param wakeup UART wakeup method.
  * @retval SET (wakeup is valid) or RESET (wakeup is invalid)
  */
#define IS_UART_WAKEUP_METHOD(wakeup) (((wakeup) == HAL_UART_WAKEUP_METHOD_IDLE_LINE)        \
                                       || ((wakeup) == HAL_UART_WAKEUP_METHOD_ADDRESS_MARK))

/**
  * @brief Ensure that IRDA power mode is valid.
  * @param power_mode IRDA power mode.
  * @retval SET (power_mode is valid) or RESET (power_mode is invalid)
  */
#define IS_UART_IRDA_POWER_MODE(power_mode) (((power_mode) == HAL_UART_IRDA_POWER_MODE_NORMAL)  \
                                             || ((power_mode) == HAL_UART_IRDA_POWER_MODE_LOW))

/**
  * @brief Ensure that IRDA prescaler is valid.
  * @param prescaler IRDA Prescaler.
  * @retval SET (prescaler is valid) or RESET (prescaler is invalid)
  */
#define IS_UART_IRDA_PRESCALER(prescaler) (((prescaler) <= 0xFFU) && (prescaler != 0U))

/**
  * @brief Ensure that UART LIN break detection length is valid.
  * @param length UART LIN break detection length.
  * @retval SET (length is valid) or RESET (length is invalid)
  */
#define IS_UART_LIN_BREAK_DETECT_LENGTH(length) (((length) == HAL_UART_LIN_BREAK_DETECT_10_BIT)     \
                                                 || ((length) == HAL_UART_LIN_BREAK_DETECT_11_BIT))

/**
  * @brief Ensure that UART driver enable polarity is valid.
  * @param polarity UART driver enable polarity.
  * @retval SET (polarity is valid) or RESET (polarity is invalid)
  */
#define IS_UART_DE_POLARITY(polarity)    (((polarity) == HAL_UART_DE_POLARITY_HIGH)    \
                                          || ((polarity) == HAL_UART_DE_POLARITY_LOW))

/**
  * @brief Ensure that UART request parameter is valid.
  * @param request UART request parameter.
  * @retval SET (request is valid) or RESET (request is invalid)
  */
#define IS_UART_REQUEST_PARAMETER(request) (((request) == HAL_UART_REQUEST_AUTO_BAUD_RATE)     \
                                            || ((request) == HAL_UART_REQUEST_SEND_BREAK)      \
                                            || ((request) == HAL_UART_REQUEST_MUTE_MODE)       \
                                            || ((request) == HAL_UART_REQUEST_RX_DATA_FLUSH)   \
                                            || ((request) == HAL_UART_REQUEST_TX_DATA_FLUSH))

/**
  * @brief Ensure that UART wake-up selection is valid.
  * @param wakesel UART wake-up selection.
  * @retval SET (wakesel is valid) or RESET (wakesel is invalid)
  */
#define IS_UART_WAKEUP_SELECTION(wakesel) (((wakesel) == HAL_UART_WAKEUP_ON_ADDRESS)               \
                                           || ((wakesel) == HAL_UART_WAKEUP_ON_STARTBIT)           \
                                           || ((wakesel) == HAL_UART_WAKEUP_ON_READDATA_NONEMPTY))

/**
  * @brief Ensure that UART wake-up address length is valid.
  * @param address UART wake-up address length.
  * @retval SET (address is valid) or RESET (address is invalid)
  */
#define IS_UART_ADDRESS_LENGTH_DETECT(address) (((address) == HAL_UART_ADDRESS_DETECT_4_BIT)     \
                                                || ((address) == HAL_UART_ADDRESS_DETECT_7_BIT))

/**
  * @brief Ensure that UART FIFO threshold level is valid.
  * @param threshold UART FIFO threshold level.
  * @retval SET (threshold is valid) or RESET (threshold is invalid)
  */
#define IS_UART_FIFO_THRESHOLD(threshold) (((threshold) == HAL_UART_FIFO_THRESHOLD_1_8)     \
                                           || ((threshold) == HAL_UART_FIFO_THRESHOLD_1_4)  \
                                           || ((threshold) == HAL_UART_FIFO_THRESHOLD_1_2)  \
                                           || ((threshold) == HAL_UART_FIFO_THRESHOLD_3_4)  \
                                           || ((threshold) == HAL_UART_FIFO_THRESHOLD_7_8)  \
                                           || ((threshold) == HAL_UART_FIFO_THRESHOLD_8_8))

/**
  * @brief Ensure that UART Auto Baud Rate Mode is valid.
  * @param mode UART Auto Baud Rate Mode.
  * @retval SET (mode is valid) or RESET (mode is invalid)
  */
#define IS_UART_AUTO_BAUD_RATE_MODE(mode)  (((mode) == HAL_UART_AUTO_BAUD_DET_ON_START_BIT)       \
                                            || ((mode) == HAL_UART_AUTO_BAUD_DET_ON_FALLING_EDGE) \
                                            || ((mode) == HAL_UART_AUTO_BAUD_DET_ON_0X7F_FRAME)   \
                                            || ((mode) == HAL_UART_AUTO_BAUD_DET_ON_0X55_FRAME))

/**
  * @brief Ensure that UART Optional Interrupts for IT in Transmit is valid.
  * @param interrupt UART Optional Interrupts.
  * @retval SET (interrupt is valid) or RESET (interrupt is invalid)
  */

#define IS_UART_OPT_TX_IT(interrupt)  (((interrupt) == HAL_UART_OPT_TX_IT_NONE)             \
                                       || ((interrupt) == HAL_UART_OPT_TX_IT_FIFO_EMPTY)    \
                                       || ((interrupt) == HAL_UART_OPT_TX_IT_CLEAR_TO_SEND) \
                                       || ((interrupt) == HAL_UART_OPT_TX_IT_DEFAULT))

/**
  * @brief Ensure that UART Optional Interrupts for IT in Receive is valid.
  * @param interrupt UART Optional Interrupts.
  * @retval SET (interrupt is valid) or RESET (interrupt is invalid)
  */

#define IS_UART_OPT_RX_IT(interrupt)    (((interrupt) == HAL_UART_OPT_RX_IT_NONE)         \
                                         || ((interrupt) == HAL_UART_OPT_RX_IT_FIFO_FULL) \
                                         || ((interrupt) == HAL_UART_OPT_RX_IT_LIN_BREAK) \
                                         || ((interrupt) == HAL_UART_OPT_RX_IT_DEFAULT))

#if defined(USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Ensure that UART Optional Interrupts for DMA in Transmit is valid.
  * @param interrupt UART Optional Interrupts.
  * @retval SET (interrupt is valid) or RESET (interrupt is invalid)
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_UART_OPT_TX_DMA(interrupt)  (((interrupt) == HAL_UART_OPT_DMA_TX_IT_NONE)        \
                                        || ((interrupt) == HAL_UART_OPT_DMA_TX_IT_HT)       \
                                        || ((interrupt) == HAL_UART_OPT_DMA_TX_IT_SILENT)   \
                                        || ((interrupt) == HAL_UART_OPT_DMA_TX_IT_DEFAULT))

#define IS_UART_DMA_TX_VALID_SILENT_MODE(handle_dma, interrupt)                \
  (((interrupt) == HAL_UART_OPT_DMA_TX_IT_SILENT)                              \
   && (handle_dma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)

#else
#define IS_UART_OPT_TX_DMA(interrupt)  (((interrupt) == HAL_UART_OPT_DMA_TX_IT_NONE)        \
                                        || ((interrupt) == HAL_UART_OPT_DMA_TX_IT_HT)       \
                                        || ((interrupt) == HAL_UART_OPT_DMA_TX_IT_DEFAULT))

#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief Ensure that UART Optional Interrupts for DMA in Receive is valid.
  * @param interrupt UART Optional Interrupts.
  * @retval SET (interrupt is valid) or RESET (interrupt is invalid)
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_UART_OPT_RX_DMA(interrupt)    (((interrupt) == HAL_UART_OPT_DMA_RX_IT_NONE)        \
                                          || ((interrupt) == HAL_UART_OPT_DMA_RX_IT_HT)       \
                                          || ((interrupt) == HAL_UART_OPT_DMA_RX_IT_SILENT)   \
                                          || ((interrupt) == HAL_UART_OPT_DMA_RX_IT_DEFAULT))

#define IS_UART_DMA_RX_VALID_SILENT_MODE(handle_dma ,interrupts)                  \
  (((interrupts) == HAL_UART_OPT_DMA_RX_IT_SILENT)                                \
   && (handle_dma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)

#else
#define IS_UART_OPT_RX_DMA(interrupt)    (((interrupt) == HAL_UART_OPT_DMA_RX_IT_NONE)        \
                                          || ((interrupt) == HAL_UART_OPT_DMA_RX_IT_HT)       \
                                          || ((interrupt) == HAL_UART_OPT_DMA_RX_IT_DEFAULT))

#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_UART_DMA */

/**
  * @brief Check whether the UART instance is enabled. If it is, disable it.
  * @param handle specifies the UART handle.
  */
#define UART_ENSURE_INSTANCE_DISABLED(handle)                \
  uint32_t instance_enabled;                                 \
  do                                                         \
  {                                                          \
    instance_enabled = LL_USART_IsEnabled(handle);           \
    if (instance_enabled != 0U)                              \
    {                                                        \
      LL_USART_Disable(handle);                              \
    }                                                        \
  } while(0U)

/**
  * @brief Check whether the UART instance needs to be re-enabled.
  * @param handle specifies the UART handle.
  */
#define UART_ENSURE_INSTANCE_ENABLED(handle)                 \
  do                                                         \
  {                                                          \
    if (instance_enabled != 0U)                              \
    {                                                        \
      LL_USART_Enable(handle);                               \
    }                                                        \
  } while(0U)

/**
  * @brief Retrieve the UART instance from the handle.
  * @param handle specifies the USART handle.
  */
#define UART_GET_INSTANCE(handle)   ((USART_TypeDef *)((uint32_t)(handle)->instance))

/**
  * @}
  */

/* Private functions --------------------------------------------------------*/
/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */
#if defined(USE_HAL_UART_CLK_ENABLE_MODEL) && (USE_HAL_UART_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
/** @brief  Set the UART clock frequency.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__STATIC_INLINE void UART_SetClockFrequency(const hal_uart_handle_t *huart)
{
  /*! Instance USART1 */
  if (huart->instance == HAL_UART1)
  {
    HAL_RCC_USART1_EnableClock();
  }
  /*! Instance USART2 */
  if (huart->instance == HAL_UART2)
  {
    HAL_RCC_USART2_EnableClock();
  }
#if defined(USART3)
  /*! Instance USART3 */
  if (huart->instance == HAL_UART3)
  {
    HAL_RCC_USART3_EnableClock();
  }
#endif /* USART3 */
  /*! Instance UART4 */
  if (huart->instance == HAL_UART4)
  {
    HAL_RCC_UART4_EnableClock();
  }
  /*! Instance UART5 */
  if (huart->instance == HAL_UART5)
  {
    HAL_RCC_UART5_EnableClock();
  }
#if defined(USART6)
  /*! Instance USART6 */
  if (huart->instance == HAL_UART6)
  {
    HAL_RCC_USART6_EnableClock();
  }
#endif /* USART6 */
#if defined(UART7)
  /*! Instance UART7 */
  if (huart->instance == HAL_UART7)
  {
    HAL_RCC_UART7_EnableClock();
  }
#endif /* UART7 */
  /*! Instance LPUART1 */
  if (huart->instance == HAL_LPUART1)
  {
    HAL_RCC_LPUART1_EnableClock();
  }
}
#endif /* USE_HAL_UART_CLK_ENABLE_MODEL */

/** @brief  Report the UART mask to apply to retrieve the received data
  *         according to the word length and parity bit activation.
  * @param  huart specifies the UART Handle.
  * @note   If PCE bit from the CR1 register = 1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.
  * @retval HAL_OK UART RDR mask has been correctly calculated.
  * @retval HAL_ERROR UART RDR mask cannot be calculated because of a mismatch
  *         between parity and word length.
  */
__STATIC_INLINE hal_status_t UART_RDRMaskComputation(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);
  uint32_t data_width = LL_USART_GetDataWidth(p_uartx);
  uint32_t parity = LL_USART_GetParity(p_uartx);

  if (data_width == LL_USART_DATAWIDTH_9_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      huart->rdr_mask = UART_RDR_MASK_9BITS;
    }
    else
    {
      huart->rdr_mask = UART_RDR_MASK_8BITS;
    }
  }
  else if (data_width == LL_USART_DATAWIDTH_8_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      huart->rdr_mask = UART_RDR_MASK_8BITS;
    }
    else
    {
      huart->rdr_mask = UART_RDR_MASK_7BITS;
    }
  }
  else if (data_width == LL_USART_DATAWIDTH_7_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      huart->rdr_mask = UART_RDR_MASK_7BITS;
    }
    else
    {
      huart->rdr_mask = UART_RDR_MASK_6BITS;
    }
  }
  else
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
static void UART_EndTxTransfer(hal_uart_handle_t *huart);
#endif /* USE_HAL_UART_DMA */
static void UART_EndRxTransfer(hal_uart_handle_t *huart);
#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
static void UART_DMAAbortOnError(hal_dma_handle_t *hdma);
static void UART_DMATransmitCplt(hal_dma_handle_t *hdma);
static void UART_DMAReceiveCplt(hal_dma_handle_t *hdma);
static void UART_DMARxHalfCplt(hal_dma_handle_t *hdma);
static void UART_DMATxHalfCplt(hal_dma_handle_t *hdma);
static void UART_DMAError(hal_dma_handle_t *hdma);

static void UART_DMATxAbortCallback(hal_dma_handle_t *hdma);
static void UART_DMARxAbortCallback(hal_dma_handle_t *hdma);
static void UART_DMATxOnlyAbortCallback(hal_dma_handle_t *hdma);
static void UART_DMARxOnlyAbortCallback(hal_dma_handle_t *hdma);
static void UART_DMAAbortOnSuccessCallback(hal_dma_handle_t *hdma);
static hal_status_t UART_Start_Receive_DMA(hal_uart_handle_t *huart, uint8_t *p_data, uint32_t size,
                                           hal_uart_rx_modes_t rx_mode, uint32_t interrupts);
static hal_status_t UART_Start_Transmit_DMA(hal_uart_handle_t *huart, const uint8_t *p_data, uint32_t size,
                                            uint32_t interrupts);
#endif /* USE_HAL_UART_DMA */
static void UART_TxISR_8BIT_FIFOEN(hal_uart_handle_t *huart);
static void UART_TxISR_16BIT_FIFOEN(hal_uart_handle_t *huart);
static void UART_SetNbDataToProcess(hal_uart_handle_t *huart);
static void UART_TxISR_8BIT(hal_uart_handle_t *huart);
static void UART_TxISR_16BIT(hal_uart_handle_t *huart);
static void UART_EndTransmit_IT(hal_uart_handle_t *huart);
static void UART_RxISR_8BIT(hal_uart_handle_t *huart);
static void UART_RxISR_16BIT(hal_uart_handle_t *huart);
static void UART_RxISR_8BIT_FIFOEN(hal_uart_handle_t *huart);
static void UART_RxISR_16BIT_FIFOEN(hal_uart_handle_t *huart);
static hal_status_t UART_Abort(hal_uart_handle_t *huart);
static hal_status_t UART_CheckEnabledState(hal_uart_handle_t *huart);

static hal_status_t UART_Start_Receive_IT(hal_uart_handle_t *huart, uint8_t *p_data, uint32_t size,
                                          hal_uart_rx_modes_t rx_mode, uint32_t interrupts);
static hal_status_t UART_Start_Transmit_IT(hal_uart_handle_t *huart, const uint8_t *p_data, uint32_t size,
                                           uint32_t interrupts);
static hal_status_t UART_Start_Receive_Polling(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                               uint32_t *p_rx_size_byte, uint32_t timeout_ms,
                                               hal_uart_rx_modes_t rx_mode);
static hal_status_t UART_WaitOnFlagUntilTimeout(hal_uart_handle_t *huart, uint32_t flag, uint32_t status,
                                                uint32_t tick_start, uint32_t timeout_ms);
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
static void UART_InitCallbacksToDefault(hal_uart_handle_t *huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
#if defined(USE_ASSERT_DBG_PARAM)
hal_status_t UART_Check_lpuart_baudrate_validity(uint32_t instance_clock_freq, uint32_t baud_rate,
                                                 uint32_t instance_clock_prescaler);
hal_status_t UART_Check_uart_baudrate_validity(uint32_t instance_clock_freq, uint32_t baud_rate,
                                               uint32_t instance_clock_prescaler,
                                               hal_uart_oversampling_t oversampling);
#endif /* USE_ASSERT_DBG_PARAM */
void UART_Parity_Computation(hal_uart_handle_t *huart, uint8_t *p_character);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup UART_Exported_Functions
  * @{
  */

/** @addtogroup UART_Exported_Functions_Group1
  * @{
This subsection provides a set of functions to initialize and deinitialize the USARTx in
asynchronous mode.
  - Call the function HAL_UART_Init() to initialize the selected USARTx handle and associate an instance.
  - Call the function HAL_UART_DeInit() to deinitialize the given HAL UART instance by stopping any ongoing process
  and resetting the state machine.
  */

/**
  * @brief Initialize the UART handler for the associated instance.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which will contain the UART instance.
  * @param instance USARTx instance.
  * @retval HAL_OK UART instance has been correctly initialized
  * @retval HAL_INVALID_PARAM UART instance is NULL
  * @retval HAL_ERROR UART semaphore creation is failed (USE_HAL_MUTEX is set to 1)
  */
hal_status_t HAL_UART_Init(hal_uart_handle_t *huart, hal_uart_t instance)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(((IS_UART_INSTANCE((USART_TypeDef *)((uint32_t)instance))))
                   || (IS_LPUART_INSTANCE((USART_TypeDef *)((uint32_t)instance))));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (huart == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->rx_state = HAL_UART_RX_STATE_RESET;
  huart->tx_state = HAL_UART_TX_STATE_RESET;
  huart->reception_type = HAL_UART_RX_STANDARD;

  huart->instance = instance;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  UART_InitCallbacksToDefault(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */

  huart->nb_tx_data_to_process = 1;
  huart->nb_rx_data_to_process = 1;
  huart->fifo_mode = HAL_UART_FIFO_MODE_DISABLED;

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  huart->hdma_tx = NULL;
  huart->hdma_rx = NULL;
#endif /* USE_HAL_UART_DMA */

#if defined (USE_HAL_UART_USER_DATA) && (USE_HAL_UART_USER_DATA == 1)
  huart->p_user_data = NULL;
#endif /* USE_HAL_UART_USER_DATA */

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0;
  huart->last_transmission_error_codes = 0;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

#if defined(USE_HAL_UART_CLK_ENABLE_MODEL) && (USE_HAL_UART_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  UART_SetClockFrequency(huart);
#endif /* USE_HAL_UART_CLK_ENABLE_MODEL */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  if (HAL_OS_SemaphoreCreate(&huart->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  huart->global_state = HAL_UART_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief Deinitializes the UART handler, reset the flags, states and counters.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
void HAL_UART_DeInit(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  hal_uart_rx_state_t temp_rx_state;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM((IS_UART_INSTANCE(p_uartx))
                   || (IS_LPUART_INSTANCE(p_uartx)));

  temp_rx_state = huart->rx_state;
  /* Check if any transfer ongoing */
  if ((huart->tx_state == HAL_UART_TX_STATE_ACTIVE) || (temp_rx_state == HAL_UART_RX_STATE_ACTIVE))
  {
    /* Stop current process/operation(s) */
#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
    if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
    {
      LL_USART_DisableDMAReq_TX(p_uartx);
      /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
      if (huart->hdma_tx != NULL)
      {
        (void)HAL_DMA_Abort(huart->hdma_tx);
      }
    }
    if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
    {
      LL_USART_DisableDMAReq_RX(p_uartx);
      /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
      if (huart->hdma_rx != NULL)
      {
        (void)HAL_DMA_Abort(huart->hdma_rx);
      }
    }
#endif /* USE_HAL_UART_DMA */
    (void)UART_Abort(huart);
  }

  LL_USART_Disable(p_uartx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&huart->semaphore);
#endif /* USE_HAL_MUTEX */

  huart->reception_type = HAL_UART_RX_STANDARD;
  huart->rx_state = HAL_UART_RX_STATE_RESET;
  huart->tx_state = HAL_UART_TX_STATE_RESET;
  huart->global_state = HAL_UART_STATE_RESET;
}
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group2
  * @{
This subsection provides a set of functions to configure the USARTx in asynchronous mode.
  - Call HAL_UART_SetConfig() to configure the initialized instance with a set of parameters containing:
    - Baud Rate
    - Prescaler
    - Word Length
    - Stop Bits
    - Parity: If the parity is enabled, then the MSB bit of the data written in the data register is transmitted but
    is changed by the parity bit.
    - Hardware flow control
    - Direction (Receiver/Transmitter)
    - Oversampling method
    - One-bit sampling method
  - Call HAL_UART_GetConfig() to retrieve the current configuration (optional).
  - If needed, after calling HAL_UART_SetConfig(), modify the configuration by using the unitary
  configuration functions:
    - HAL_UART_SetBaudRate()
    - HAL_UART_SetStopBits()
    - HAL_UART_SetWordLength()
    - HAL_UART_SetParity()
    - HAL_UART_SetHwFlowCtl()
    - HAL_UART_SetXferDirection()
    - HAL_UART_SetOneBitSample()

  @note
    - __Prescaler__: cannot be modified with a unitary configuration function as it impacts other parameters. Call
    HAL_UART_SetConfig() to modify it.
    - __Over Sampling__: cannot be modified with a unitary configuration function as it impacts other parameters. Call
    HAL_UART_SetConfig() to modify it.

  - If needed, retrieve the different parameters by calling:
    - HAL_UART_GetBaudRate()
    - HAL_UART_GetStopBits()
    - HAL_UART_GetWordLength()
    - HAL_UART_GetParity()
    - HAL_UART_GetHwFlowCtl()
    - HAL_UART_GetXferDirection()
    - HAL_UART_GetOneBitSample()

  @note
    - __Prescaler__: As there is no unitary configuration function for this parameter, there is no unitary
    getter.
    - __Over Sampling__: As there is no unitary configuration function for this parameter, there is no unitary
    getter.

  - Possible frame format:
    Depending on the frame length defined by the M1 and M0 bits from the CR1 register (7-bit, 8-bit or 9-bit),
    the possible UART formats are listed in the following table.

~~~
  UART frame format.
    +-----------------------------------------------------------------------+
    |  M1 bit |  M0 bit |  PCE bit  |             UART frame                |
    |---------|---------|-----------|---------------------------------------|
    |    0    |    0    |    0      |    | SB |    8 bit data   | STB |     |
    |---------|---------|-----------|---------------------------------------|
    |    0    |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
    |---------|---------|-----------|---------------------------------------|
    |    0    |    1    |    0      |    | SB |    9 bit data   | STB |     |
    |---------|---------|-----------|---------------------------------------|
    |    0    |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
    |---------|---------|-----------|---------------------------------------|
    |    1    |    0    |    0      |    | SB |    7 bit data   | STB |     |
    |---------|---------|-----------|---------------------------------------|
    |    1    |    0    |    1      |    | SB | 6 bit data | PB | STB |     |
    +-----------------------------------------------------------------------+
~~~
  */

/**
  * @brief Set the basic configuration to enable the use of the UART instance.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer to a hal_uart_config_t structure containing the UART configuration.
  * @retval HAL_OK UART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  * @retval HAL_ERROR Kernel clock is not set.
  */
hal_status_t HAL_UART_SetConfig(hal_uart_handle_t *huart, const hal_uart_config_t *p_config)
{
  USART_TypeDef *p_uartx;
  uint32_t instance_clock_freq;
  uint32_t reg_temp;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(p_config != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_UART_PRESCALER(p_config->clock_prescaler));
  ASSERT_DBG_PARAM(IS_UART_WORD_LENGTH(p_config->word_length));
  if (IS_LPUART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(IS_LPUART_STOP_BITS(p_config->stop_bits));
  }
  if (IS_UART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(IS_UART_STOP_BITS(p_config->stop_bits));
  }

  ASSERT_DBG_PARAM(IS_UART_PARITY(p_config->parity));
  ASSERT_DBG_PARAM(IS_UART_BAUD_RATE(p_config->baud_rate));
  ASSERT_DBG_PARAM(IS_UART_DIRECTION(p_config->direction));
  ASSERT_DBG_PARAM(IS_UART_HARDWARE_FLOW_CONTROL(p_config->hw_flow_ctl));
  ASSERT_DBG_PARAM(IS_UART_ONE_BIT_SAMPLE(p_config->one_bit_sampling));
  if (!IS_LPUART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(IS_UART_OVERSAMPLING(p_config->oversampling));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_INIT | HAL_UART_STATE_CONFIGURED));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_RESET | HAL_UART_RX_STATE_IDLE));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_RESET | HAL_UART_TX_STATE_IDLE));

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_ConfigAsyncMode(p_uartx);

  if (IS_LPUART_INSTANCE(p_uartx))
  {
    reg_temp = ((uint32_t)p_config->word_length | (uint32_t)p_config->parity | (uint32_t)p_config->direction);

    LL_LPUART_ConfigXfer(p_uartx, reg_temp, (uint32_t)p_config->stop_bits);
  }
  else
  {
    reg_temp = ((uint32_t)p_config->word_length | (uint32_t)p_config->parity
                | (uint32_t)p_config->direction | (uint32_t)p_config->oversampling);

    LL_USART_ConfigXfer(p_uartx, reg_temp, (uint32_t)p_config->stop_bits);
  }
  LL_USART_SetHWFlowCtrl(p_uartx, (uint32_t)p_config->hw_flow_ctl);
  if (p_config->one_bit_sampling != HAL_UART_ONE_BIT_SAMPLE_DISABLE)
  {
    LL_USART_EnableOneBitSample(p_uartx);
  }
  else
  {
    LL_USART_DisableOneBitSample(p_uartx);
  }

  LL_USART_SetPrescaler(p_uartx, (uint32_t)p_config->clock_prescaler);

  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);
  if (instance_clock_freq != 0U)
  {
    if (IS_LPUART_INSTANCE(p_uartx))
    {
      ASSERT_DBG_PARAM(UART_Check_lpuart_baudrate_validity(instance_clock_freq, p_config->clock_prescaler,
                                                           p_config->baud_rate) == HAL_OK);
      LL_LPUART_SetBaudRate(p_uartx, instance_clock_freq, (uint32_t)p_config->clock_prescaler, p_config->baud_rate);
    }
    else
    {
      ASSERT_DBG_PARAM(UART_Check_uart_baudrate_validity(instance_clock_freq, p_config->clock_prescaler,
                                                         p_config->baud_rate, p_config->oversampling) == HAL_OK);
      LL_USART_SetBaudRate(p_uartx, instance_clock_freq, (uint32_t)p_config->clock_prescaler,
                           (uint32_t)p_config->oversampling, p_config->baud_rate);
    }
  }
  else
  {
    /* Kernel clock not set */
    return HAL_ERROR;
  }

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  huart->rx_state = HAL_UART_RX_STATE_IDLE;
  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  huart->global_state = HAL_UART_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Get the current basic configuration set in the current UART instance.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer to a hal_uart_config_t structure to store the UART configuration.
  */
void HAL_UART_GetConfig(const hal_uart_handle_t *huart, hal_uart_config_t *p_config)
{
  USART_TypeDef *p_uartx;
  uint32_t reg_temp;
  uint32_t instance_clock_freq;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);
  p_config->word_length = (hal_uart_word_length_t)(uint32_t)
                          (reg_temp & (LL_USART_DATAWIDTH_7_BIT | LL_USART_DATAWIDTH_9_BIT));
  p_config->parity = (hal_uart_parity_t)(uint32_t)(reg_temp & (LL_USART_PARITY_ODD));
  p_config->direction = (hal_uart_direction_t)(uint32_t)(reg_temp & (LL_USART_DIRECTION_TX_RX));
  p_config->oversampling = (hal_uart_oversampling_t)(uint32_t)(reg_temp & LL_USART_OVERSAMPLING_8);

  p_config->stop_bits = (hal_uart_stop_bits_t)LL_USART_GetStopBitsLength(p_uartx);

  reg_temp = LL_USART_READ_REG(p_uartx, CR3);
  p_config->hw_flow_ctl = (hal_uart_hw_control_t)(uint32_t)(reg_temp & (LL_USART_HWCONTROL_RTS_CTS));
  p_config->one_bit_sampling = (hal_uart_one_bit_sample_t)(uint32_t)(reg_temp & LL_USART_ONE_BIT_SAMPLE_ENABLE);

  p_config->clock_prescaler = (hal_uart_prescaler_t)LL_USART_GetPrescaler(p_uartx);

  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);
  if (IS_LPUART_INSTANCE(p_uartx))
  {
    p_config->baud_rate = LL_LPUART_GetBaudRate(p_uartx, instance_clock_freq, (uint32_t)p_config->clock_prescaler);
  }
  else
  {
    p_config->baud_rate = LL_USART_GetBaudRate(p_uartx, instance_clock_freq,
                                               (uint32_t)p_config->clock_prescaler, (uint32_t)p_config->oversampling);
  }
}

/**
  * @brief Set the Word Length configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param word_length Word length to be applied.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetWordLength(const hal_uart_handle_t *huart, hal_uart_word_length_t word_length)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(IS_UART_WORD_LENGTH(word_length));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetDataWidth(p_uartx, (uint32_t)word_length);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Word Length configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_word_length_t Current Word length configuration.
  */
hal_uart_word_length_t HAL_UART_GetWordLength(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_word_length_t)LL_USART_GetDataWidth(p_uartx);
}

/**
  * @brief Set the Parity configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param parity Parity to be applied.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetParity(const hal_uart_handle_t *huart, hal_uart_parity_t parity)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(IS_UART_PARITY(parity));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetParity(p_uartx, (uint32_t)parity);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Parity configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_parity_t Current Parity configuration.
  */
hal_uart_parity_t HAL_UART_GetParity(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
}

/**
  * @brief Set the Stop Bits configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param stop_bits Stop Bits to be applied.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetStopBits(const hal_uart_handle_t *huart, hal_uart_stop_bits_t stop_bits)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
#if defined(USE_ASSERT_DBG_PARAM)
  if (IS_LPUART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(IS_LPUART_STOP_BITS(stop_bits));
  }
  if (IS_UART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(IS_UART_STOP_BITS(stop_bits));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetStopBitsLength(p_uartx, (uint32_t)stop_bits);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Stop Bits configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_stop_bits_t Current Stop Bits configuration.
  */
hal_uart_stop_bits_t HAL_UART_GetStopBits(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_stop_bits_t)LL_USART_GetStopBitsLength(p_uartx);
}

/**
  * @brief Set the XFer Direction configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param direction XFer Direction to be applied.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetXferDirection(const hal_uart_handle_t *huart, hal_uart_direction_t direction)
{
  USART_TypeDef *p_uartx;
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(IS_UART_DIRECTION(direction));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_SetTransferDirection(p_uartx, (uint32_t)direction);

  return HAL_OK;
}

/**
  * @brief Get the XFer Direction configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_direction_t Current XFer Direction configuration.
  */
hal_uart_direction_t HAL_UART_GetXferDirection(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_direction_t)LL_USART_GetTransferDirection(p_uartx);
}

/**
  * @brief Set the Hardwre Flow Control configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param hw_flow_ctl Hardware Flow Control to be applied.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetHwFlowCtl(const hal_uart_handle_t *huart, hal_uart_hw_control_t hw_flow_ctl)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(IS_UART_HARDWARE_FLOW_CONTROL(hw_flow_ctl));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetHWFlowCtrl(p_uartx, (uint32_t)hw_flow_ctl);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Hardware Flow Control configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_hw_control_t Current Hardware Flow Control configuration.
  */
hal_uart_hw_control_t HAL_UART_GetHwFlowCtl(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_hw_control_t)LL_USART_GetHWFlowCtrl(p_uartx);
}

/**
  * @brief Set the One Bit Sample configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param one_bit_sample One Bit Sample to be applied
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetOneBitSample(const hal_uart_handle_t *huart, hal_uart_one_bit_sample_t one_bit_sample)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  p_uartx = UART_GET_INSTANCE(huart);

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));

#endif /* USE_ASSERT_DBG_PARAM */
  ASSERT_DBG_PARAM(IS_UART_ONE_BIT_SAMPLE(one_bit_sample));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  if (one_bit_sample == HAL_UART_ONE_BIT_SAMPLE_ENABLE)
  {
    LL_USART_EnableOneBitSample(p_uartx);
  }
  else
  {
    LL_USART_DisableOneBitSample(p_uartx);
  }

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the One Bit Sample configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval hal_uart_one_bit_sample_t Current One Bit Sampling configuration.
  */
hal_uart_one_bit_sample_t HAL_UART_GetOneBitSample(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  hal_uart_one_bit_sample_t one_bit_sample;

  ASSERT_DBG_PARAM(huart != NULL);

  p_uartx = UART_GET_INSTANCE(huart);

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));

#endif /* USE_ASSERT_DBG_PARAM */
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  if (LL_USART_IsEnabledOneBitSample(p_uartx) != 0U)
  {
    one_bit_sample = HAL_UART_ONE_BIT_SAMPLE_ENABLE;
  }
  else
  {
    one_bit_sample = HAL_UART_ONE_BIT_SAMPLE_DISABLE;
  }
  return (one_bit_sample);
}

/**
  * @brief Set the Baud Rate configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param baud_rate Baud rate value to set.
  * @retval HAL_OK UART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM Required baud rate value can't be set with current config.
  */
hal_status_t HAL_UART_SetBaudRate(const hal_uart_handle_t *huart, uint32_t baud_rate)
{
  USART_TypeDef *p_uartx;
  hal_uart_oversampling_t oversampling;
  uint32_t instance_clock_freq;
  uint32_t instance_clock_prescaler;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(IS_UART_BAUD_RATE(baud_rate));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);
  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);
  instance_clock_prescaler = LL_USART_GetPrescaler(p_uartx);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  if (IS_LPUART_INSTANCE(p_uartx))
  {
    ASSERT_DBG_PARAM(UART_Check_lpuart_baudrate_validity(instance_clock_freq, instance_clock_prescaler,
                                                         baud_rate) == HAL_OK);
    LL_LPUART_SetBaudRate(p_uartx, instance_clock_freq, instance_clock_prescaler, baud_rate);
  }
  else
  {
    oversampling = (hal_uart_oversampling_t)LL_USART_GetOverSampling(p_uartx);
    ASSERT_DBG_PARAM(UART_Check_uart_baudrate_validity(instance_clock_freq, instance_clock_prescaler, baud_rate,
                                                       oversampling) == HAL_OK);
    LL_USART_SetBaudRate(p_uartx, instance_clock_freq, instance_clock_prescaler, (uint32_t)oversampling, baud_rate);
  }

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Baud Rate configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval uint32_t Current baud rate value.
  */
uint32_t HAL_UART_GetBaudRate(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t instance_clock_freq;
  uint32_t baud_rate = 0;
  uint32_t oversampling;
  uint32_t prescaler;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);
  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);

  prescaler = LL_USART_GetPrescaler(p_uartx);
  if (IS_LPUART_INSTANCE(p_uartx))
  {
    baud_rate = LL_LPUART_GetBaudRate(p_uartx, instance_clock_freq, prescaler);
  }
  if (IS_UART_INSTANCE(p_uartx))
  {
    oversampling = LL_USART_GetOverSampling(p_uartx);
    baud_rate = LL_USART_GetBaudRate(p_uartx, instance_clock_freq, prescaler, oversampling);
  }

  return (baud_rate);
}
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group3
  * @{
  This subsection provides a set of functions to configure the USARTx instance in IRDA mode.
  Use the following functions to use the IRDA feature:
    - HAL_UART_IRDA_SetConfig(): Set the configuration and enable IRDA mode
    - HAL_UART_IRDA_SetPrescaler(): Set the IRDA prescaler (different from clock prescaler)
    - HAL_UART_IRDA_SetPowerMode(): Set the desired IRDA power mode (low power or normal)

  A set of getter functions is also provided to check the current configuration:
    - HAL_UART_IRDA_GetConfig(): Get the configuration for IRDA mode
    - HAL_UART_IRDA_GetPrescaler(): Get the IRDA prescaler (different from clock prescaler)
    - HAL_UART_IRDA_GetPowerMode(): Get the desired IRDA power mode (low power or normal)

  Note that the HAL_UART_IRDA_SetConfig API can be called without calling
  HAL_UART_SetConfig beforehand. The HAL_UART_xxxx API are still available in IRDA and are used to communicate.

  @warning Please note that while in IRDA mode LIN mode cannot be enabled, the stop bit configuration cannot
           be changed (1 bit is locked) and FIFO cannot be enabled.
  */

/**
  * @brief Set the basic configuration to enable the use of the UART instance.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer to a hal_uart_irda_config_t structure containing the IRDA configuration.
  * @retval HAL_OK UART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  */
hal_status_t HAL_UART_IRDA_SetConfig(hal_uart_handle_t *huart, const hal_uart_irda_config_t *p_config)
{
  USART_TypeDef *p_uartx;
  uint32_t instance_clock_freq;
  uint32_t div_temp;
  uint32_t reg_temp;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_UART_BAUD_RATE(p_config->baud_rate));
  ASSERT_DBG_PARAM(IS_UART_PRESCALER(p_config->clock_prescaler));
  ASSERT_DBG_PARAM(IS_UART_IRDA_POWER_MODE(p_config->irda_power_mode));
  ASSERT_DBG_PARAM(IS_UART_IRDA_PRESCALER(p_config->irda_prescaler));
  ASSERT_DBG_PARAM(IS_UART_WORD_LENGTH(p_config->word_length));
  ASSERT_DBG_PARAM(IS_UART_PARITY(p_config->parity));
  ASSERT_DBG_PARAM(IS_UART_DIRECTION(p_config->direction));
  ASSERT_DBG_PARAM(IS_UART_ONE_BIT_SAMPLE(p_config->one_bit_sampling));

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_INIT | HAL_UART_STATE_CONFIGURED));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_RESET | HAL_UART_RX_STATE_IDLE));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_RESET | HAL_UART_TX_STATE_IDLE));

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_ConfigIrdaMode(p_uartx);

  reg_temp = ((uint32_t)p_config->word_length | (uint32_t)p_config->parity
              | (uint32_t)p_config->direction | (uint32_t)HAL_UART_OVERSAMPLING_16);

  LL_USART_ConfigXfer(p_uartx, reg_temp, (uint32_t)HAL_UART_STOP_BIT_1);

  reg_temp = LL_USART_READ_REG(p_uartx, CR3);
  reg_temp = (reg_temp & ~(uint32_t)(HAL_UART_ONE_BIT_SAMPLE_ENABLE)) | (uint32_t)p_config->one_bit_sampling;
  reg_temp = (reg_temp & ~(uint32_t)(HAL_UART_IRDA_POWER_MODE_LOW)) | (uint32_t)p_config->irda_power_mode;
  LL_USART_WRITE_REG(p_uartx, CR3, reg_temp);

  LL_USART_SetIrdaPrescaler(p_uartx, p_config->irda_prescaler);

  LL_USART_SetPrescaler(p_uartx, (uint32_t)p_config->clock_prescaler);

  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);
  if (instance_clock_freq != 0UL)
  {
    div_temp = LL_USART_DIV_SAMPLING16(instance_clock_freq, (uint32_t)p_config->clock_prescaler, p_config->baud_rate);
    ASSERT_DBG_PARAM((div_temp >= UART_BRR_MIN) && (div_temp <= UART_BRR_MAX));
  }
  else
  {
    return HAL_ERROR;
  }
  LL_USART_WRITE_REG(p_uartx, BRR, (uint16_t)div_temp);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  huart->rx_state = HAL_UART_RX_STATE_IDLE;
  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  huart->global_state = HAL_UART_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Get the current IRDA configuration set in the current UART instance.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer to a \ref hal_uart_irda_config_t structure to store the IRDA configuration.
  */
void HAL_UART_IRDA_GetConfig(const hal_uart_handle_t *huart, hal_uart_irda_config_t *p_config)
{
  USART_TypeDef *p_uartx;
  uint32_t reg_temp;
  uint32_t instance_clock_freq;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);
  p_config->word_length = (hal_uart_word_length_t)(uint32_t)
                          (reg_temp & (LL_USART_DATAWIDTH_7_BIT | LL_USART_DATAWIDTH_9_BIT));
  p_config->parity = (hal_uart_parity_t)(uint32_t)(reg_temp & (LL_USART_PARITY_ODD));
  p_config->direction = (hal_uart_direction_t)(uint32_t)(reg_temp & (LL_USART_DIRECTION_TX_RX));

  reg_temp = LL_USART_READ_REG(p_uartx, CR3);
  p_config->one_bit_sampling = (hal_uart_one_bit_sample_t)(uint32_t)(reg_temp & LL_USART_ONE_BIT_SAMPLE_ENABLE);
  p_config->irda_power_mode = (hal_uart_irda_power_mode_t)(uint32_t)(reg_temp & LL_USART_IRDA_POWER_MODE_LOW);

  p_config->clock_prescaler = (hal_uart_prescaler_t)LL_USART_GetPrescaler(p_uartx);

  p_config->irda_prescaler = LL_USART_GetIrdaPrescaler(p_uartx);

  instance_clock_freq = HAL_RCC_UART_GetKernelClkFreq(p_uartx);

  p_config->baud_rate = LL_USART_GetBaudRate(p_uartx, instance_clock_freq, (uint32_t)p_config->clock_prescaler,
                                             (uint32_t)HAL_UART_OVERSAMPLING_16);
}

/**
  * @brief Set the IRDA prescaler value.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param irda_prescaler IRDA prescaler value to set (must be between 0x00 and 0xFF).
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_IRDA_SetPrescaler(const hal_uart_handle_t *huart, uint32_t irda_prescaler)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_IRDA_PRESCALER(irda_prescaler));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetIrdaPrescaler(p_uartx, irda_prescaler);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the IRDA prescaler value according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval uint32_t Current IRDA prescaler value.
  */
uint32_t HAL_UART_IRDA_GetPrescaler(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return LL_USART_GetIrdaPrescaler(p_uartx);
}

/**
  * @brief Get the IRDA power mode according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param irda_power_mode Irda power mode to set from \ref hal_uart_irda_power_mode_t
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_IRDA_SetPowerMode(const hal_uart_handle_t *huart,
                                        hal_uart_irda_power_mode_t irda_power_mode)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_IRDA_POWER_MODE(irda_power_mode));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetIrdaPowerMode(p_uartx, (uint32_t)irda_power_mode);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the IRDA power mode according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_irda_power_mode_t Current IRDA power mode.
  */
hal_uart_irda_power_mode_t HAL_UART_IRDA_GetPowerMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_IRDA_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);


  return (hal_uart_irda_power_mode_t)LL_USART_GetIrdaPowerMode(p_uartx);
}
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group4
  * @{
  This subsection provides a set of functions to configure the USARTx instance in particular
  asynchronous modes.
  Before enabling one of the following particular asynchronous modes, configure the instance in
  asynchronous mode with HAL_UART_SetConfig().

  Some modes require configuration that you can set by calling the following APIs:
    - HAL_UART_SetLINModeBreakDetectLength()
    - HAL_UART_SetConfigRS485Mode()
    - HAL_UART_SetConfigMultiProcessorMode()

  Enable a mode by calling the associated functions:
    - HAL_UART_EnableLINMode()
    - HAL_UART_EnableRS485Mode()
    - HAL_UART_EnableHalfDuplexMode()
    - HAL_UART_EnableMultiProcessorMode()

  The different modes are not compatible with each other, so do not enable two modes on the same instance.

  Disable each mode by calling the associated functions:
    - HAL_UART_DisableLINMode()
    - HAL_UART_DisableRS485Mode()
    - HAL_UART_DisableHalfDuplexMode()
    - HAL_UART_DisableMultiProcessorMode()

  Check whether a mode is enabled by calling the associated functions:
    - HAL_UART_IsEnabledLINMode()
    - HAL_UART_IsEnabledRS485Mode()
    - HAL_UART_IsEnabledHalfDuplexMode()
    - HAL_UART_IsEnabledMultiProcessorMode()

  For the multiprocessor mode, once you enable the mode, it is not active yet. Call
  HAL_UART_EnterMultiProcessorMuteMode() to enter mute. Check whether the UART is in mute with
  HAL_UART_IsEnteredMultiProcessorMuteMode().

  */

/**
  * @brief Enable the LIN Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableLINMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  ASSERT_DBG_PARAM(LL_USART_GetOverSampling(p_uartx) == LL_USART_OVERSAMPLING_16);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) == LL_USART_DATAWIDTH_8_BIT);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_ConfigLINMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the LIN Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableLINMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableLIN(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the LIN Mode status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval hal_uart_lin_mode_status_t Current LIN mode status.
  */
hal_uart_lin_mode_status_t HAL_UART_IsEnabledLINMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_lin_mode_status_t)LL_USART_IsEnabledLIN(p_uartx);
}

/**
  * @brief In LIN mode, set the Break Detection Length configuration passed in parameters into the handler instance
  *        registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param break_detect_length LIN Break Detection Length.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetLINModeBreakDetectLength(const hal_uart_handle_t *huart,
                                                  hal_uart_lin_break_detect_length_t break_detect_length)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_LIN_BREAK_DETECT_LENGTH(break_detect_length));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetLINBrkDetectionLen(p_uartx, (uint32_t)break_detect_length);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief In LIN mode, get the Break Detection Length configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval hal_uart_lin_break_detect_length_t Current Break Detection Length configuration.
  */
hal_uart_lin_break_detect_length_t HAL_UART_GetLINModeBreakDetectLength(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_lin_break_detect_length_t)LL_USART_GetLINBrkDetectionLen(p_uartx);
}

/**
  * @brief Enable the RS485 Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableRS485Mode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_DRIVER_ENABLE_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_EnableDEMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the RS485 Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableRS485Mode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_DRIVER_ENABLE_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableDEMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the RS485 Mode status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_rs485_mode_status_t Current RS485 Mode status.
  */
hal_uart_rs485_mode_status_t HAL_UART_IsEnabledRS485Mode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_DRIVER_ENABLE_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_rs485_mode_status_t)LL_USART_IsEnabledDEMode(p_uartx);
}

/**
  * @brief In RS485 mode, set the configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer on a hal_uart_rs485_config_t structure.
  * @retval HAL_OK UART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  */
hal_status_t HAL_UART_SetConfigRS485Mode(const hal_uart_handle_t *huart, const hal_uart_rs485_config_t *p_config)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_DRIVER_ENABLE_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_ASSERTION_TIME(p_config->assertion_time_samples));
  ASSERT_DBG_PARAM(IS_UART_DEASSERTION_TIME(p_config->deassertion_time_samples));
  ASSERT_DBG_PARAM(IS_UART_DE_POLARITY(p_config->polarity));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  if (p_config->polarity == HAL_UART_DE_POLARITY_HIGH)
  {
    LL_USART_SetDESignalPolarity(p_uartx, LL_USART_DE_POLARITY_HIGH);
  }
  else
  {
    LL_USART_SetDESignalPolarity(p_uartx, LL_USART_DE_POLARITY_LOW);
  }

  LL_USART_ConfigDETime(p_uartx, p_config->assertion_time_samples, p_config->deassertion_time_samples);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief In RS485 mode, get the configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_config Pointer on a hal_uart_rs485_config_t structure.
  */
void HAL_UART_GetConfigRS485Mode(const hal_uart_handle_t *huart, hal_uart_rs485_config_t *p_config)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_DRIVER_ENABLE_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_config->polarity = (hal_uart_de_polarity_t)LL_USART_GetDESignalPolarity(p_uartx);

  p_config->assertion_time_samples = LL_USART_GetDEAssertionTime(p_uartx);

  p_config->deassertion_time_samples = LL_USART_GetDEDeassertionTime(p_uartx);
}

/**
  * @brief Enable the Half Duplex Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableHalfDuplexMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_HALFDUPLEX_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_ConfigHalfDuplexMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Half Duplex Mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableHalfDuplexMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_HALFDUPLEX_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableHalfDuplex(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Half Duplex Mode status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_half_duplex_mode_status_t Current Half Duplex Mode status.
  */
hal_uart_half_duplex_mode_status_t HAL_UART_IsEnabledHalfDuplexMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_HALFDUPLEX_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_half_duplex_mode_status_t)LL_USART_IsEnabledHalfDuplex(p_uartx);
}

/**
  * @brief Enable the multiprocessor mode.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This does not make the instance enter mute mode. For this, use HAL_UART_EnterMultiProcessorMuteMode.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableMultiProcessorMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_ConfigMultiProcessMode(p_uartx);

  LL_USART_EnableMuteMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the multiprocessor mode in the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableMultiProcessorMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableMuteMode(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the multiprocessor mode status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_multi_processor_mode_status_t Current multiprocessor mode status.
  */
hal_uart_multi_processor_mode_status_t HAL_UART_IsEnabledMultiProcessorMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_multi_processor_mode_status_t)LL_USART_IsEnabledMuteMode(p_uartx);
}

/**
  * @brief For multiprocessor mode, set the mute configuration passed in parameters into the handler instance
  *        registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_wakeup_config Pointer to a hal_uart_multi_processor_mode_wakeup_config_t.
  * @retval HAL_OK UART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM p_wakeup_config is NULL.
  */
hal_status_t HAL_UART_SetConfigMultiProcessorMode(const hal_uart_handle_t *huart,
                                                  const hal_uart_multi_processor_mode_wakeup_config_t *p_wakeup_config)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_wakeup_config != NULL);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_METHOD(p_wakeup_config->wakeup_method));
  ASSERT_DBG_PARAM(IS_UART_ADDRESS_LENGTH_DETECT(p_wakeup_config->address_length));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_wakeup_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetWakeUpMethod(p_uartx, (uint32_t) p_wakeup_config->wakeup_method);
  LL_USART_ConfigNodeAddress(p_uartx, (uint32_t) p_wakeup_config->address_length, (uint32_t) p_wakeup_config->address);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief For multiprocessor mode, get the mute configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_wakeup_config Pointer to a hal_uart_multi_processor_mode_wakeup_config_t.
  */
void HAL_UART_GetConfigMultiProcessorMode(const hal_uart_handle_t *huart,
                                          hal_uart_multi_processor_mode_wakeup_config_t *p_wakeup_config)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_wakeup_config != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  p_wakeup_config->wakeup_method = (hal_uart_wakeup_method_t)LL_USART_GetWakeUpMethod(p_uartx);
  p_wakeup_config->address_length = (hal_uart_address_detect_length_t)LL_USART_GetNodeAddressLength(p_uartx);
  p_wakeup_config->address = (uint8_t)LL_USART_GetNodeAddress(p_uartx);
}

/**
  * @brief For multiprocessor mode, request instance to enter in mute.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note Requires HAL_UART_EnableMultiProcessorMode to be called first.
  * @retval HAL_OK Request has been sent.
  */
hal_status_t HAL_UART_EnterMultiProcessorMuteMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_RequestEnterMuteMode(p_uartx);

  return HAL_OK;
}

/**
  * @brief For multiprocessor mode, return if the instance is in mute.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_multi_processor_mode_mute_status_t Status of mute.
  */
hal_uart_multi_processor_mode_mute_status_t HAL_UART_IsEnteredMultiProcessorMuteMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_multi_processor_mode_mute_status_t)LL_USART_IsActiveFlag_RWU(p_uartx);
}

/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group5
  * @{
  This subsection provides a set of functions to configure advanced features for the USARTx instance.
  Note that not all advanced features are supported on all instances.
  Before configuring advanced features, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().

  Enable a set of advanced features by calling the associated functions:
    - HAL_UART_EnableTxPinLevelInvert(): Enable the Tx pin inverted logical level
    - HAL_UART_EnableRxPinLevelInvert(): Enable the Rx pin inverted logical level
    - HAL_UART_EnableDataInvert(): Enable the binary data inversion (1=L, 0=H)
    - HAL_UART_EnableTxRxSwap(): Enable the swap between Tx and Rx pins
    - HAL_UART_EnableRxOverRunDetection(): Enable the Rx overrun detection
    - HAL_UART_EnableDMAStopOnRxError(): Enable the stop of the DMA in case of Rx error
    - HAL_UART_EnableMSBFirst(): Enable the most significant bit first
    - HAL_UART_EnableReceiverTimeout(): Enable the hardware timeout on the receiver side
    - HAL_UART_EnableTransmitter(): Enable the transmitter side
    - HAL_UART_EnableReceiver(): Enable the receiver side

  Disable a set of advanced features by calling the associated functions:
    - HAL_UART_DisableTxPinLevelInvert(): Disable the Tx pin inverted logical level
    - HAL_UART_DisableRxPinLevelInvert(): Disable the Rx pin inverted logical level
    - HAL_UART_DisableDataInvert(): Disable the binary data inversion (1=H, 0=L)
    - HAL_UART_DisableTxRxSwap(): Disable the swap between Tx and Rx pins
    - HAL_UART_DisableRxOverRunDetection(): Disable the Rx overrun detection
    - HAL_UART_DisableDMAStopOnRxError(): Disable the stop of the DMA in case of Rx error
    - HAL_UART_DisableMSBFirst(): Disable the Most Significant Bit first
    - HAL_UART_DisableReceiverTimeout(): Disable the hardware timeout on the receiver side
    - HAL_UART_DisableTransmitter(): Disable the transmitter side
    - HAL_UART_DisableReceiver(): Disable the receiver side

  Check whether a feature is enabled by calling the associated functions:
    - HAL_UART_IsEnabledTxPinLevelInvert(): Check whether the Tx pin inverted logical level is enabled
    - HAL_UART_IsEnabledRxPinLevelInvert(): Check whether the Rx pin inverted logical level is enabled
    - HAL_UART_IsEnabledDataInvert(): Check whether the binary data inversion is enabled
    - HAL_UART_IsEnabledTxRxSwap(): Check whether the swap between Tx and Rx pins is enabled
    - HAL_UART_IsEnabledRxOverRunDetection(): Check whether the Rx overrun detection is enabled
    - HAL_UART_IsEnabledDMAStopOnRxError(): Check whether the stop of the DMA in case of Rx error is enabled
    - HAL_UART_IsEnabledMSBFirst(): Check whether the Most Significant Bit first is enabled
    - HAL_UART_IsEnabledReceiverTimeout(): Check whether the hardware timeout on the receiver side is enabled
    - HAL_UART_IsEnabledTransmitter(): Check whether the transmitter side is enabled
    - HAL_UART_IsEnabledReceiver(): Check whether the receiver side is enabled

  Configure some features by calling the associated functions:
    - HAL_UART_SetConfigReceiverTimeout(): Set a hardware timeout on the receiver side
    - HAL_UART_GetConfigReceiverTimeout(): Get the hardware timeout on the receiver side
  */

/**
  * @brief Enable the Tx pin level inversion into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableTxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTXPinLevel(p_uartx, LL_USART_TXPIN_LEVEL_INVERTED);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Tx pin level inversion into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableTxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTXPinLevel(p_uartx, LL_USART_TXPIN_LEVEL_STANDARD);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Tx pin level inversion status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_tx_pin_level_invert_status_t Current Tx pin level inversion status.
  */
hal_uart_tx_pin_level_invert_status_t HAL_UART_IsEnabledTxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (LL_USART_GetTXPinLevel(p_uartx) == LL_USART_TXPIN_LEVEL_STANDARD)
  {
    return HAL_UART_TX_PIN_LEVEL_INVERT_DISABLED;
  }
  else
  {
    return HAL_UART_TX_PIN_LEVEL_INVERT_ENABLED;
  }
}

/**
  * @brief Enable the Rx pin level inversion into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableRxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetRXPinLevel(p_uartx, LL_USART_RXPIN_LEVEL_INVERTED);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Rx pin level inversion into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableRxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetRXPinLevel(p_uartx, LL_USART_RXPIN_LEVEL_STANDARD);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Rx pin level inversion status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_rx_pin_level_invert_status_t Current Rx pin level inversion status.
  */
hal_uart_rx_pin_level_invert_status_t HAL_UART_IsEnabledRxPinLevelInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (LL_USART_GetRXPinLevel(p_uartx) == LL_USART_RXPIN_LEVEL_STANDARD)
  {
    return HAL_UART_RX_PIN_LEVEL_INVERT_DISABLED;
  }
  else
  {
    return HAL_UART_RX_PIN_LEVEL_INVERT_ENABLED;
  }
}

/**
  * @brief Enable the binary Data Inversion into the handler instance registers, (1=L, 0=H).
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableDataInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetBinaryDataLogic(p_uartx, LL_USART_BINARY_LOGIC_NEGATIVE);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the binary Data Inversion into the handler instance registers (1=H, 0=L).
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableDataInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetBinaryDataLogic(p_uartx, LL_USART_BINARY_LOGIC_POSITIVE);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the binary Data Inversion status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_data_invert_status_t Current Data Inversion status.
  */
hal_uart_data_invert_status_t HAL_UART_IsEnabledDataInvert(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (LL_USART_GetBinaryDataLogic(p_uartx) == LL_USART_BINARY_LOGIC_POSITIVE)
  {
    return HAL_UART_DATA_INVERT_DISABLED;
  }
  else
  {
    return HAL_UART_DATA_INVERT_ENABLED;
  }
}

/**
  * @brief Enable the swap between Tx and Rx pins into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableTxRxSwap(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTXRXSwap(p_uartx, LL_USART_TXRX_SWAPPED);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the swap between Tx and Rx pins into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableTxRxSwap(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTXRXSwap(p_uartx, LL_USART_TXRX_STANDARD);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the swap between Tx and Rx pins status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_tx_rx_swap_status_t Current Tx/Rx swap status.
  */
hal_uart_tx_rx_swap_status_t HAL_UART_IsEnabledTxRxSwap(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (LL_USART_GetTXRXSwap(p_uartx) == LL_USART_TXRX_STANDARD)
  {
    return HAL_UART_TX_RX_SWAP_DISABLED;
  }
  else
  {
    return HAL_UART_TX_RX_SWAP_ENABLED;
  }
}

/**
  * @brief Enable the Rx Overrun detection into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  When UART is initialised and configured with basic configuration parameters, this feature is enabled by
  *        default.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableRxOverRunDetection(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_EnableOverrunDetect(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Rx Overrun detection into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  When UART is initialised and configured with basic configuration parameters, this feature is enabled by
  *        default.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableRxOverRunDetection(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableOverrunDetect(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Rx Overrun detection status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  When UART is initialised and configured with basic configuration parameters, this feature is enabled by
  *        default.
  * @retval hal_uart_rx_overrun_detection_status_t Current Rx Overrun detection status.
  */
hal_uart_rx_overrun_detection_status_t HAL_UART_IsEnabledRxOverRunDetection(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_rx_overrun_detection_status_t)LL_USART_IsEnabledOverrunDetect(p_uartx);
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Enable the DMA Disabling On a Rx Error into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableDMAStopOnRxError(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_EnableDMADeactOnRxErr(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the DMA Disabling On a Rx Error into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableDMAStopOnRxError(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableDMADeactOnRxErr(p_uartx);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the DMA Disabling On a Rx Error status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_dma_stop_on_rx_error_status_t Current DMA Stopping On a Rx Error status.
  */
hal_uart_dma_stop_on_rx_error_status_t HAL_UART_IsEnabledDMAStopOnRxError(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return (hal_uart_dma_stop_on_rx_error_status_t) LL_USART_IsEnabledDMADeactOnRxErr(p_uartx);
}

#endif /* USE_HAL_UART_DMA */
/**
  * @brief Enable the MSB First into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableMSBFirst(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTransferBitOrder(p_uartx, LL_USART_BITORDER_MSB_FIRST);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;

}

/**
  * @brief Disable the MSB First into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableMSBFirst(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetTransferBitOrder(p_uartx, LL_USART_BITORDER_LSB_FIRST);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the MSB First status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_msb_first_status_t Current MSB First status.
  */
hal_uart_msb_first_status_t HAL_UART_IsEnabledMSBFirst(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (LL_USART_GetTransferBitOrder(p_uartx) == LL_USART_BITORDER_LSB_FIRST)
  {
    return HAL_UART_MSB_FIRST_DISABLED;
  }
  else
  {
    return HAL_UART_MSB_FIRST_ENABLED;
  }
}

/**
  * @brief Set the Receiver Timeout configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param timeout_bit Value between 0x00 -> 0xFFFFFFU in number of bit.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetConfigReceiverTimeout(const hal_uart_handle_t *huart, uint32_t timeout_bit)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(timeout_bit));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_SetRxTimeout(p_uartx, timeout_bit);

  return HAL_OK;
}

/**
  * @brief Get the Receiver Timeout configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval uint32_t Receiver Timeout between 0x00 -> 0xFFFFFFU in number of bit.
  */
uint32_t HAL_UART_GetConfigReceiverTimeout(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return LL_USART_GetRxTimeout(p_uartx);
}

/**
  * @brief Enable the Receiver Timeout into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableReceiverTimeout(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_EnableRxTimeout(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Receiver Timeout into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableReceiverTimeout(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_DisableRxTimeout(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Receiver Timeout status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_receiver_timeout_status_t Current Receiver Timeout status.
  */
hal_uart_receiver_timeout_status_t HAL_UART_IsEnabledReceiverTimeout(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_receiver_timeout_status_t) LL_USART_IsEnabledRxTimeout(p_uartx);
}

/**
  * @brief Enable the Transmitter into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableTransmitter(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_EnableDirectionTx(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Transmitter into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableTransmitter(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_DisableDirectionTx(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Transmitter status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval hal_uart_transmitter_status_t Current Transmitter status.
  */
hal_uart_transmitter_status_t HAL_UART_IsEnabledTransmitter(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t transfer_dir;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  transfer_dir = LL_USART_GetTransferDirection(p_uartx);

  if ((transfer_dir == (LL_USART_DIRECTION_TX)) || (transfer_dir == (LL_USART_DIRECTION_TX_RX)))
  {
    return HAL_UART_TRANSMITTER_ENABLED;
  }
  else
  {
    return HAL_UART_TRANSMITTER_DISABLED;
  }
}

/**
  * @brief Enable the Receiver into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableReceiver(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_EnableDirectionRx(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Receiver into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableReceiver(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_DisableDirectionRx(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Receiver status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  Refer to Half Duplex mode to use this API.
  * @retval hal_uart_receiver_status_t Current Receiver status.
  */
hal_uart_receiver_status_t HAL_UART_IsEnabledReceiver(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t transfer_dir;

  ASSERT_DBG_PARAM(huart != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  transfer_dir = LL_USART_GetTransferDirection(p_uartx);

  if ((transfer_dir == (LL_USART_DIRECTION_RX)) || (transfer_dir == (LL_USART_DIRECTION_TX_RX)))
  {
    return HAL_UART_RECEIVER_ENABLED;
  }
  else
  {
    return HAL_UART_RECEIVER_DISABLED;
  }
}
/**
  * @}
  */
/** @addtogroup UART_Exported_Functions_Group6
  * @{
  This subsection provides a set of functions to use the auto baud rate feature for the USARTx instance.
  Before using the auto baud rate feature, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().

  Use the following functions to use the auto baud rate feature:
    - HAL_UART_EnableAutoBaudRate(): Enable the auto baud rate feature
    - HAL_UART_DisableAutoBaudRate(): Disable the auto baud rate feature
    - HAL_UART_IsEnabledAutoBaudRate(): Check if the auto baud rate feature is enabled
    - HAL_UART_GetAutoBaudRateStatus(): Get the result of the auto baud rate operation
    - HAL_UART_SetConfigAutoBaudRateMode(): Set the configuration of the auto baud rate feature
    - HAL_UART_GetConfigAutoBaudRateMode(): Retrieve the configuration of the auto baud rate feature
    - HAL_UART_GetBaudRate(): Retrieve the current baud rate

  Use the following procedure:
    - HAL_UART_SetConfigAutoBaudRateMode()
    - HAL_UART_EnableAutoBaudRate()
    - Start a receive process, for example: HAL_UART_Receive()
    - HAL_UART_GetAutoBaudRateStatus() returns HAL_UART_AUTO_BAUD_RATE_DET_SUCCESS
    - HAL_UART_GetBaudRate()
  */

/**
  * @brief Enable the Auto Baud Rate feature.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableAutoBaudRate(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  if (LL_USART_IsEnabledAutoBaud(p_uartx) == 0U)
  {
    LL_USART_EnableAutoBaudRate(p_uartx);
  }
  else
  {
    LL_USART_RequestAutoBaudRate(p_uartx);
  }
  return HAL_OK;
}

/**
  * @brief Disable the Auto Baud Rate feature.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableAutoBaudRate(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_DisableAutoBaudRate(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Auto Baud Rate activation status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval hal_uart_auto_baud_rate_status_t Current Auto Baud Rate activation status.
  */
hal_uart_auto_baud_rate_status_t HAL_UART_IsEnabledAutoBaudRate(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_auto_baud_rate_status_t)LL_USART_IsEnabledAutoBaud(p_uartx);
}

/**
  * @brief Return the Auto Baud Rate Detection state according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @note  Baud Rate Value is available though HAL_UART_GetBaudRate().
  * @retval hal_uart_auto_baud_rate_detection_status_t Current Auto Baud Rate detection state.
  */
hal_uart_auto_baud_rate_detection_status_t HAL_UART_GetAutoBaudRateStatus(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  if (LL_USART_IsEnabledAutoBaud(p_uartx) == 0U)
  {
    return HAL_UART_AUTO_BAUD_RATE_DET_NOT_ENABLED;
  }

  if (LL_USART_IsActiveFlag_ABR(p_uartx) == 0U)
  {
    return HAL_UART_AUTO_BAUD_RATE_DET_ONGOING;
  }

  if (LL_USART_IsActiveFlag_ABRE(p_uartx) != 0U)
  {
    return HAL_UART_AUTO_BAUD_RATE_DET_ERROR;
  }

  return HAL_UART_AUTO_BAUD_RATE_DET_SUCCESS;
}

/**
  * @brief Set the Auto Baud Rate detection configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param auto_baud_rate_mode Auto Baud Rate Mode to set.
  * @note  This feature is not available for LPUART instances.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetConfigAutoBaudRateMode(const hal_uart_handle_t *huart,
                                                hal_uart_auto_baud_rate_mode_t auto_baud_rate_mode)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_AUTO_BAUD_RATE_MODE(auto_baud_rate_mode));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  /* Configure Auto Baud Rate only when it is not enabled */
  ASSERT_DBG_PARAM(LL_USART_IsEnabledAutoBaud(p_uartx) == 0U);

  LL_USART_SetAutoBaudRateMode(p_uartx, (uint32_t)auto_baud_rate_mode);

  return HAL_OK;
}

/**
  * @brief Get the Auto Baud Rate detection configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available for LPUART instances.
  * @retval hal_uart_auto_baud_rate_mode_t Current Auto Baud Rate detection configuration.
  */
hal_uart_auto_baud_rate_mode_t HAL_UART_GetConfigAutoBaudRateMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_auto_baud_rate_mode_t) LL_USART_GetAutoBaudRateMode(p_uartx);
}

/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group7
  * @{
  This subsection provides a set of functions to use the stop mode feature for the USARTx instance.
  Before using the stop mode feature, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().
  Use the following functions to use the stop mode feature:
    - HAL_UART_EnableStopMode(): Allow the instance to operate in stop mode
    - HAL_UART_DisableStopMode(): Disallow the instance to operate in stop mode
    - HAL_UART_IsEnabledStopMode(): Check whether the instance is allowed to operate in stop mode
    - HAL_UART_SetStopModeWkUpSource(): Set the source of the Wakeup flag
    - HAL_UART_GetStopModeWkUpSource(): Get the source of the Wakeup flag
    - HAL_UART_SetStopModeWkUpAddrLength(): Set the length of the address if the Wakeup source is an address
    - HAL_UART_GetStopModeWkUpAddrLength(): Get the length of the address
    - HAL_UART_SetStopModeWkUpAddr(): Set the address if the Wakeup source is an address
    - HAL_UART_GetStopModeWkUpAddr(): Get the address set for the Wakeup source

  Use the following procedure:
    - HAL_UART_SetStopModeWkUpSource()
    - HAL_UART_EnableStopMode()
    - Start a process, for example: HAL_UART_Receive_IT()
    - Call the PWR driver to enter low-power mode
    - Sleep until the Wakeup source is triggered
  */

/**
  * @brief Enable the Stop Mode into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableStopMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_EnableInStopMode(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the Stop Mode into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableStopMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_DisableInStopMode(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the Stop Mode status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_stop_mode_status_t Current Stop Mode status.
  */
hal_uart_stop_mode_status_t HAL_UART_IsEnabledStopMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_stop_mode_status_t) LL_USART_IsEnabledInStopMode(p_uartx);
}

/**
  * @brief Set the Stop Mode Wakeup source passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param source Stop Mode Wakeup source.
  * @note This API sets the source of the WUF flag. It does not mean other Stop Mode sources will not wake up the
  *       USARTx instance.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetStopModeWkUpSource(const hal_uart_handle_t *huart, const hal_uart_wakeup_source_t source)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_SELECTION(source));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetWKUPType(p_uartx, (uint32_t)source);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Stop Mode Wakeup source configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_wakeup_source_t Source of the Stop Mode Wakeup.
  */
hal_uart_wakeup_source_t HAL_UART_GetStopModeWkUpSource(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_wakeup_source_t)LL_USART_GetWKUPType(p_uartx);
}

/**
  * @brief Set the Stop Mode Wakeup address length passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param address_length Address Length to set.
  * @note Use this API with HAL_UART_SetStopModeWkUpAddr().
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetStopModeWkUpAddrLength(const hal_uart_handle_t *huart,
                                                const hal_uart_address_detect_length_t address_length)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(IS_UART_ADDRESS_LENGTH_DETECT(address_length));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetNodeAddressLength(p_uartx, (uint32_t)address_length);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Stop Mode Wakeup address length according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_address_detect_length_t Address length.
  */
hal_uart_address_detect_length_t HAL_UART_GetStopModeWkUpAddrLength(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_address_detect_length_t)LL_USART_GetNodeAddressLength(p_uartx);
}

/**
  * @brief Set the Stop Mode Wakeup address passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param address Address to set.
  * @note Use this API with HAL_UART_SetStopModeWkUpAddrLength().
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetStopModeWkUpAddr(const hal_uart_handle_t *huart, uint8_t address)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_SetNodeAddress(p_uartx, (uint32_t)address);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Get the Stop Mode Wakeup address according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval uint8_t address.
  */
uint8_t HAL_UART_GetStopModeWkUpAddr(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_uartx));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (uint8_t)LL_USART_GetNodeAddress(p_uartx);
}

/**
  * @}
  */
/** @addtogroup UART_Exported_Functions_Group8
  * @{
  This subsection provides a set of functions to use the FIFO mode feature for the USARTx instance.
  Before using the FIFO mode feature, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().
  Use the following functions to use the FIFO mode feature:
    - HAL_UART_EnableFifoMode(): Enable the fifo mode feature
    - HAL_UART_DisableFifoMode(): Disable the fifo mode feature
    - HAL_UART_IsEnabledFifoMode(): Check if the fifo mode feature is enabled
    - HAL_UART_SetTxFifoThreshold(): Set the configuration of the Tx FIFO
    - HAL_UART_GetTxFifoThreshold(): Retrieve the configuration of the Tx FIFO
    - HAL_UART_SetRxFifoThreshold(): Set the configuration of the Rx FIFO
    - HAL_UART_GetRxFifoThreshold(): Retrieve the configuration of the Rx FIFO

  Use the following procedure:
    - HAL_UART_SetTxFifoThreshold()
    - HAL_UART_SetRxFifoThreshold()
    - HAL_UART_EnableFifoMode()
    - Start a process, for example: HAL_UART_Receive()
  */

/**
  * @brief Enable the FIFO into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available in LIN mode.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_EnableFifoMode(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_EnableFIFO(p_uartx);

  huart->fifo_mode = HAL_UART_FIFO_MODE_ENABLED;

  UART_SetNbDataToProcess(huart);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Disable the FIFO into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available in LIN mode.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_DisableFifoMode(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);

  LL_USART_DisableFIFO(p_uartx);

  huart->fifo_mode = HAL_UART_FIFO_MODE_DISABLED;

  UART_SetNbDataToProcess(huart);

  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return HAL_OK;
}

/**
  * @brief Return the FIFO status according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available in LIN mode.
  * @retval hal_uart_fifo_mode_status_t Current FIFO status.
  */
hal_uart_fifo_mode_status_t HAL_UART_IsEnabledFifoMode(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_fifo_mode_status_t)LL_USART_IsEnabledFIFO(p_uartx);
}

/**
  * @brief Set the Tx FIFO threshold configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param tx_fifo_threshold Tx FIFO threshold to apply.
  * @note  This feature is not available in LIN mode.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetTxFifoThreshold(hal_uart_handle_t *huart, hal_uart_fifo_threshold_t tx_fifo_threshold)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_FIFO_THRESHOLD(tx_fifo_threshold));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_SetTXFIFOThreshold(p_uartx, (uint32_t)tx_fifo_threshold);

  UART_SetNbDataToProcess(huart);

  return HAL_OK;
}

/**
  * @brief Get the Tx FIFO threshold configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available in LIN mode.
  * @retval hal_uart_fifo_threshold_t Current Tx FIFO threshold configuration.
  */
hal_uart_fifo_threshold_t HAL_UART_GetTxFifoThreshold(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_fifo_threshold_t) LL_USART_GetTXFIFOThreshold(p_uartx);
}

/**
  * @brief Set the Rx FIFO threshold configuration passed in parameters into the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param rx_fifo_threshold Rx FIFO threshold to apply.
  * @note  This feature is not available in LIN mode.
  * @retval HAL_OK UART instance has been correctly configured.
  */
hal_status_t HAL_UART_SetRxFifoThreshold(hal_uart_handle_t *huart, hal_uart_fifo_threshold_t rx_fifo_threshold)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_FIFO_THRESHOLD(rx_fifo_threshold));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_SetRXFIFOThreshold(p_uartx, (uint32_t)rx_fifo_threshold);

  UART_SetNbDataToProcess(huart);

  return HAL_OK;
}

/**
  * @brief Get the Rx FIFO threshold configuration according to the handler instance registers.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note  This feature is not available in LIN mode.
  * @retval hal_uart_fifo_threshold_t Current Rx FIFO threshold configuration.
  */
hal_uart_fifo_threshold_t HAL_UART_GetRxFifoThreshold(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  return (hal_uart_fifo_threshold_t) LL_USART_GetRXFIFOThreshold(p_uartx);
}
/**
  * @}
  */


/** @addtogroup UART_Exported_Functions_Group10
  * @{
  This subsection provides a set of functions to link the HAL UART handle to a Tx and Rx DMA handler
  for the USARTx instance.
  Use the following functions to use the DMA feature:
    - HAL_UART_SetTxDMA(): Link a DMA instance to the Tx channel
    - HAL_UART_SetRxDMA(): Link a DMA instance to the Rx channel
  */
#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Set DMA channel for Transmission.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param hdma_tx Pointer to a hal_dma_handle_t structure which contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_tx is NULL.
  */
hal_status_t HAL_UART_SetTxDMA(hal_uart_handle_t *huart, hal_dma_handle_t *hdma_tx)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(hdma_tx != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT);
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_tx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->hdma_tx = hdma_tx;
  hdma_tx->p_parent = huart;

  return HAL_OK;
}

/**
  * @brief Set DMA channel for Reception.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param hdma_rx Pointer to a hal_dma_handle_t structure which contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_rx is NULL.
  */
hal_status_t HAL_UART_SetRxDMA(hal_uart_handle_t *huart, hal_dma_handle_t *hdma_rx)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(hdma_rx != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT);
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_rx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->hdma_rx = hdma_rx;
  hdma_rx->p_parent = huart;

  return HAL_OK;
}
#endif /* USE_HAL_UART_DMA */

/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group11
  * @{
  This subsection provides a set of functions to configure the callbacks for the USARTx instance.
  Before configuring the callbacks, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().
  Use the following functions to configure the callbacks:
    - HAL_UART_RegisterTxHalfCpltCallback(): Set the Tx half complete callback
    - HAL_UART_RegisterTxCpltCallback(): Set the Tx complete callback
    - HAL_UART_RegisterRxHalfCpltCallback(): Set the Rx half complete callback
    - HAL_UART_RegisterRxCpltCallback(): Set the Rx complete callback
    - HAL_UART_RegisterErrorCallback(): Set the error callback
    - HAL_UART_RegisterAbortCpltCallback(): Set the abort complete callback
    - HAL_UART_RegisterAbortTransmitCpltCallback(): Set the abort transmit complete callback
    - HAL_UART_RegisterAbortReceiveCpltCallback(): Set the abort receive complete callback
    - HAL_UART_RegisterWakeupCallback(): Set the wakeup callback
    - HAL_UART_RegisterRxFifoFullCallback(): Set the Rx FIFO full callback
    - HAL_UART_RegisterTxFifoEmptyCallback(): Set the Tx FIFO empty callback
    - HAL_UART_RegisterClearToSendCallback(): Set the clear to send callback
    - HAL_UART_RegisterLINBreakCallback(): Set the LIN break callback
  */
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)

/**
  * @brief  Register the UART Tx Half Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Tx Half Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterTxHalfCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_tx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Tx Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Tx Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterTxCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_tx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Rx Half Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Rx Half Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterRxHalfCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_rx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Rx Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Rx Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterRxCpltCallback(hal_uart_handle_t *huart, hal_uart_rx_cplt_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_rx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Error Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Error Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterErrorCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_error_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Abort Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Abort Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterAbortCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_abort_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Abort Transmit Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Abort Transmit Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterAbortTransmitCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_abort_transmit_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Abort Receive Complete Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Abort Receive Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterAbortReceiveCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_abort_receive_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART wakeup callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback Pointer to the wakeup callback function.
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterWakeupCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_wakeup_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Rx FIFO Full Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Rx FIFO Full Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterRxFifoFullCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_rx_fifo_full_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Tx FIFO Empty Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Tx FIFO Empty Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterTxFifoEmptyCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_tx_fifo_empty_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART Clear To Send Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the Clear To Send Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterClearToSendCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_clear_to_send_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the UART LIN Break Callback.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_callback pointer to the LIN Break Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_UART_RegisterLINBreakCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(huart->global_state, (uint32_t)(HAL_UART_STATE_CONFIGURED | HAL_UART_STATE_INIT));
  ASSERT_DBG_STATE(huart->rx_state, (uint32_t)(HAL_UART_RX_STATE_IDLE | HAL_UART_RX_STATE_RESET));
  ASSERT_DBG_STATE(huart->tx_state, (uint32_t)(HAL_UART_TX_STATE_IDLE | HAL_UART_TX_STATE_RESET));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  huart->p_lin_break_callback = p_callback;

  return HAL_OK;
}

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group12
  *  @{
  This subsection provides a set of functions to manage the UART asynchronous
  and Half Duplex data transfers.

 There are two modes of transfer:
    - Blocking mode: The communication is performed in polling mode.
      The HAL status of all data processing is returned by the same function
      after finishing transfer.
    - Non-blocking mode: The communication is performed using interrupts
      or DMA. These APIs return the HAL status.
      The end of the data processing will be indicated through the
      dedicated UART IRQ when using Interrupt mode or the DMA IRQ when
      using DMA mode.
      The HAL_UART_TxCpltCallback() and HAL_UART_RxCpltCallback() user callbacks
      are executed at the end of the transmit or receive process.
      The HAL_UART_ErrorCallback() user callback is executed when a communication error is detected.

  Blocking mode APIs are:
    - HAL_UART_Transmit()
    - HAL_UART_Receive()

  Non-blocking mode APIs with interrupt are:
    - HAL_UART_Transmit_IT()
    - HAL_UART_Transmit_IT_Opt()
    - HAL_UART_Receive_IT()
    - HAL_UART_Receive_IT_Opt()

  Non-blocking mode APIs with DMA are:
    - HAL_UART_Transmit_DMA()
    - HAL_UART_Transmit_DMA_Opt()
    - HAL_UART_Receive_DMA()
    - HAL_UART_Receive_DMA_Opt()
    - HAL_UART_Pause_DMA()
    - HAL_UART_PauseReceive_DMA()
    - HAL_UART_PauseTransmit_DMA()
    - HAL_UART_Resume_DMA()
    - HAL_UART_ResumeReceive_DMA()
    - HAL_UART_ResumeTransmit_DMA()

  A set of Transfer Complete Callbacks are provided in non-blocking mode:
    - HAL_UART_TxHalfCpltCallback()
    - HAL_UART_TxCpltCallback()
    - HAL_UART_RxHalfCpltCallback()
    - HAL_UART_RxCpltCallback()
    - HAL_UART_ErrorCallback()

  Abort non-blocking mode transfers using the Abort APIs:
    - HAL_UART_Abort()
    - HAL_UART_AbortTransmit()
    - HAL_UART_AbortReceive()
    - HAL_UART_Abort_IT()
    - HAL_UART_AbortTransmit_IT()
    - HAL_UART_AbortReceive_IT()

  For abort services based on interrupts (HAL_UART_Abortxxx_IT), a set of Abort Complete callbacks are provided:
    - HAL_UART_AbortCpltCallback()
    - HAL_UART_AbortTransmitCpltCallback()
    - HAL_UART_AbortReceiveCpltCallback()

  In non-blocking mode transfers, possible errors are split into two categories:
    - Error is considered recoverable and non-blocking: Transfer can run to completion, but error severity is
      to be evaluated by the user. This concerns Frame Error, Parity Error, or Noise Error
      in interrupt mode reception.
      The received character is then retrieved and stored in the Rx buffer, the error code is set to allow the user
      to identify the error type, and the HAL_UART_ErrorCallback() user callback is executed.
      Transfer remains ongoing on the UART side.
      If you want to abort it, call the Abort services.
    - Error is considered blocking: Transfer cannot be completed properly and is aborted.
      This concerns Overrun Error in interrupt mode reception and all errors in DMA mode.
      The error code is set to allow the user to identify the error type, and the HAL_UART_ErrorCallback()
      user callback is executed.

  In Half Duplex communication, do not run the transmit and receive process in parallel.
  */

/**
  * @brief Send an amount of data in blocking mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be sent.
  * @param timeout_ms         Timeout duration.
  * @warning When UART parity is not enabled (PCE bit from register CR1 = 0), and Word Length is configured to 9 bits
  *          (M1-M0 from register CR1 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Transmit(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte, uint32_t timeout_ms)
{
  uint32_t tick_start;
  uint32_t reg_temp;
  USART_TypeDef *p_uartx;
  const uint8_t  *p_data_8_bits;
  const uint16_t *p_data_16_bits;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_uartx = UART_GET_INSTANCE(huart);

  HAL_CHECK_UPDATE_STATE(huart, tx_state, HAL_UART_TX_STATE_IDLE, HAL_UART_TX_STATE_ACTIVE);

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->tx_state = HAL_UART_TX_STATE_IDLE;
    return HAL_ERROR;
  }

  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_SetTransferDirection(p_uartx, LL_USART_DIRECTION_TX);
  }

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    p_data_16_bits = (const uint16_t *)p_data;
    p_data_8_bits = NULL;
  }
  else
  {
    p_data_8_bits = (const uint8_t *)p_data;
    p_data_16_bits = NULL;
  }

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  huart->tx_xfer_size  = size_byte;
  huart->tx_xfer_count = size_byte;

  while (huart->tx_xfer_count > 0U)
  {
    if (UART_WaitOnFlagUntilTimeout(huart, LL_USART_ISR_TXE_TXFNF, 0U, tick_start, timeout_ms) != HAL_OK)
    {
      huart->tx_state = HAL_UART_TX_STATE_IDLE;
      return HAL_TIMEOUT;
    }
    if (p_data_8_bits == NULL)
    {
      LL_USART_TransmitData9(p_uartx, *p_data_16_bits);
      p_data_16_bits++;
    }
    else
    {
      LL_USART_TransmitData8(p_uartx, *p_data_8_bits);
      p_data_8_bits++;
    }
    huart->tx_xfer_count--;
  }

  if (UART_WaitOnFlagUntilTimeout(huart, LL_USART_ISR_TC, 0U, tick_start, timeout_ms) != HAL_OK)
  {
    huart->tx_state = HAL_UART_TX_STATE_IDLE;
    return HAL_TIMEOUT;
  }

  huart->tx_state = HAL_UART_TX_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Receive an amount of data in blocking mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @param timeout_ms         Timeout duration.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Receive(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  status = UART_Start_Receive_Polling(huart, p_data, size_byte, NULL, timeout_ms, HAL_UART_RX_STANDARD);

  huart->rx_state = HAL_UART_RX_STATE_IDLE;
  return status;
}

/**
  * @brief Send an amount of data in interrupt mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be sent.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Transmit_IT(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, tx_state, HAL_UART_TX_STATE_IDLE, HAL_UART_TX_STATE_ACTIVE);

  return UART_Start_Transmit_IT(huart, (const uint8_t *)p_data, size_byte, HAL_UART_OPT_TX_IT_NONE);
}

/**
  * @brief Send an amount of data in interrupt mode, allow user to enable Optional Interrupts part of
  *        \ref UART_Transmit_IT_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be sent.
  * @param interrupts         Optional interrupts part of \ref UART_Transmit_IT_Optional_Interrupts.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Transmit_IT_Opt(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte,
                                      uint32_t interrupts)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_OPT_TX_IT(interrupts));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, tx_state, HAL_UART_TX_STATE_IDLE, HAL_UART_TX_STATE_ACTIVE);

  return UART_Start_Transmit_IT(huart, (const uint8_t *)p_data, size_byte, interrupts);
}

/**
  * @brief Receive an amount of data in interrupt mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Receive_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_STANDARD;

  return (UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_STANDARD,
                                HAL_UART_OPT_RX_IT_NONE));
}

/**
  * @brief Receive an amount of data in interrupt mode, allow user to enable Optional Interrupts part of
  *        \ref UART_Receive_IT_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_Receive_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_STANDARD;

  return (UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_STANDARD,
                                interrupts));
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Send an amount of data in DMA mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be sent.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         TX DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_Transmit_DMA(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_tx != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, tx_state, HAL_UART_TX_STATE_IDLE, HAL_UART_TX_STATE_ACTIVE);

  return UART_Start_Transmit_DMA(huart, (const uint8_t *)p_data, size_byte, HAL_UART_OPT_DMA_TX_IT_HT);
}

/**
  * @brief Send an amount of data in DMA mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be sent.
  * @param interrupts         Optional interrupts part of \ref UART_Transmit_DMA_Optional_Interrupts.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         TX DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_Transmit_DMA_Opt(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte,
                                       uint32_t interrupts)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_tx != NULL);
  ASSERT_DBG_PARAM(IS_UART_OPT_TX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_UART_DMA_TX_VALID_SILENT_MODE(huart->hdma_tx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, tx_state, HAL_UART_TX_STATE_IDLE, HAL_UART_TX_STATE_ACTIVE);

  return UART_Start_Transmit_DMA(huart, (const uint8_t *)p_data, size_byte, interrupts);
}

/**
  * @brief Receive an amount of data in DMA mode.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain the
  *          parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01),
  *          the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         RX DMA handler not set or Error during instance enabling.

  */
hal_status_t HAL_UART_Receive_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_STANDARD;

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_STANDARD,
                                 HAL_UART_OPT_DMA_RX_IT_HT));
}

/**
  * @brief Receive an amount of data in DMA mode, allow user to enable Optional Interrupts part of
  *        \ref UART_Receive_DMA_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         RX DMA handler not set or Error during instance enabling.

  */
hal_status_t HAL_UART_Receive_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t interrupts)
{

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_UART_DMA_RX_VALID_SILENT_MODE(huart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_STANDARD;

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_STANDARD, interrupts));
}

/**
  * @brief Pause the DMA transfer.
  * @param huart        Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_Pause_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);
  if (huart->tx_state == HAL_UART_TX_STATE_ACTIVE)
  {
    if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
    {
      if (huart->hdma_tx != NULL)
      {
        LL_USART_DisableDMAReq_TX(p_uartx);
        huart->tx_state = HAL_UART_TX_STATE_PAUSED;
      }
    }
  }

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
    {
      if (huart->hdma_rx != NULL)
      {
        LL_USART_DisableIT_PE(p_uartx);
        LL_USART_DisableIT_ERROR(p_uartx);
        LL_USART_DisableDMAReq_RX(p_uartx);
        huart->rx_state = HAL_UART_RX_STATE_PAUSED;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief Pause the DMA receive transfer.
  * @param huart        Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_PauseReceive_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
    {
      if (huart->hdma_rx != NULL)
      {
        LL_USART_DisableIT_PE(p_uartx);
        LL_USART_DisableIT_ERROR(p_uartx);
        LL_USART_DisableDMAReq_RX(p_uartx);
        huart->rx_state = HAL_UART_RX_STATE_PAUSED;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief Pause the DMA transmit transfer.
  * @param huart        Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_PauseTransmit_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->tx_state == HAL_UART_TX_STATE_ACTIVE)
  {
    if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
    {
      LL_USART_DisableDMAReq_TX(p_uartx);
      huart->tx_state = HAL_UART_TX_STATE_PAUSED;
    }
  }
  return HAL_OK;
}

/**
  * @brief Resume DMA transfer.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK    Operation completed successfully.
  */
hal_status_t HAL_UART_Resume_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->tx_state == HAL_UART_TX_STATE_PAUSED)
  {
    if (huart->hdma_tx != NULL)
    {
      LL_USART_EnableDMAReq_TX(p_uartx);
      huart->tx_state = HAL_UART_TX_STATE_ACTIVE;
    }
  }

  if (huart->rx_state == HAL_UART_RX_STATE_PAUSED)
  {
    if (huart->hdma_rx != NULL)
    {
      LL_USART_ClearFlag_ORE(p_uartx);

      if (LL_USART_GetParity(p_uartx) != LL_USART_PARITY_NONE)
      {
        LL_USART_EnableIT_PE(p_uartx);
      }
      LL_USART_RequestRxDataFlush(p_uartx);
      LL_USART_EnableIT_ERROR(p_uartx);
      LL_USART_EnableDMAReq_RX(p_uartx);
      huart->rx_state = HAL_UART_RX_STATE_ACTIVE;
    }
  }
  return HAL_OK;
}

/**
  * @brief Resume the DMA receive transfer.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK    Operation completed successfully.
  */
hal_status_t HAL_UART_ResumeReceive_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->rx_state == HAL_UART_RX_STATE_PAUSED)
  {
    if (huart->hdma_rx != NULL)
    {
      LL_USART_ClearFlag_ORE(p_uartx);

      if (LL_USART_GetParity(p_uartx) != LL_USART_PARITY_NONE)
      {
        LL_USART_EnableIT_PE(p_uartx);
      }
      LL_USART_RequestRxDataFlush(p_uartx);
      LL_USART_EnableIT_ERROR(p_uartx);
      LL_USART_EnableDMAReq_RX(p_uartx);
      huart->rx_state = HAL_UART_RX_STATE_ACTIVE;
    }
  }
  return HAL_OK;
}

/**
  * @brief Resume the DMA transmit transfer.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK    Operation completed successfully.
  */
hal_status_t HAL_UART_ResumeTransmit_DMA(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->tx_state == HAL_UART_TX_STATE_PAUSED)
  {
    if (huart->hdma_tx != NULL)
    {
      LL_USART_EnableDMAReq_TX(p_uartx);
      huart->tx_state = HAL_UART_TX_STATE_ACTIVE;
    }
  }
  return HAL_OK;
}
#endif /* USE_HAL_UART_DMA */

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle rx_state to HAL_UART_RX_STATE_IDLE and tx_state to HAL_UART_TX_STATE_IDLE
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_Abort(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  huart->tx_state = HAL_UART_TX_STATE_ABORT;
  huart->rx_state = HAL_UART_RX_STATE_ABORT;

  (void)UART_Abort(huart);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_TX(p_uartx);
    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_tx != NULL)
    {
      (void)HAL_DMA_Abort(huart->hdma_tx);
    }
  }
  if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_uartx);
    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_rx != NULL)
    {
      (void)HAL_DMA_Abort(huart->hdma_rx);
    }
  }
#endif /* USE_HAL_UART_DMA */

  huart->rx_xfer_count = 0U;
  huart->tx_xfer_count = 0U;
  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  /* Flush the whole FIFO */
  LL_USART_RequestTxDataFlush(p_uartx);
  LL_USART_RequestRxDataFlush(p_uartx);

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0;
  huart->last_transmission_error_codes = 0;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  huart->reception_type = HAL_UART_RX_STANDARD;
  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  huart->rx_state = HAL_UART_RX_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle tx_state to HAL_UART_TX_STATE_IDLE
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_AbortTransmit(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  huart->tx_state = HAL_UART_TX_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_TXEIE_TXFNFIE | LL_USART_CR1_TCIE | LL_USART_CR1_TXFEIE));
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_TXFTIE | LL_USART_CR3_CTSIE));
  LL_USART_ClearFlag(p_uartx, (LL_USART_ICR_TXFECF | LL_USART_ICR_CTSCF));

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_TX(p_uartx);
    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_tx != NULL)
    {
      (void)HAL_DMA_Abort(huart->hdma_tx);
    }
  }
#endif /* USE_HAL_UART_DMA */
  huart->tx_xfer_count = 0U;

  LL_USART_RequestTxDataFlush(p_uartx);


#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_transmission_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle rx_state to HAL_UART_RX_STATE_IDLE
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL_OK      Operation completed successfully.
  */
hal_status_t HAL_UART_AbortReceive(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  huart->rx_state = HAL_UART_RX_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_RXFFIE
                                   | LL_USART_CR1_IDLEIE | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
  LL_USART_DisableIT_CR2(p_uartx, LL_USART_CR2_LBDIE);
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_LBDCF | LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_uartx);
    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_rx != NULL)
    {
      (void)HAL_DMA_Abort(huart->hdma_rx);
    }
  }
#endif /* USE_HAL_UART_DMA */

  huart->rx_xfer_count = 0U;
  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  LL_USART_RequestRxDataFlush(p_uartx);

  huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  huart->rx_state = HAL_UART_RX_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Abort ongoing transfers (Interrupt mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle rx_state to HAL_UART_RX_STATE_IDLE and tx_state to HAL_UART_TX_STATE_IDLE
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_UART_Abort_IT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t abort_cplt;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);
  abort_cplt = 1U;

  huart->tx_state = HAL_UART_TX_STATE_ABORT;
  huart->rx_state = HAL_UART_RX_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_TXEIE_TXFNFIE
                                   | LL_USART_CR1_TCIE | LL_USART_CR1_RXFFIE | LL_USART_CR1_TXFEIE | LL_USART_CR1_IDLEIE
                                   | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
  LL_USART_DisableIT_CR2(p_uartx, LL_USART_CR2_LBDIE);
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE | LL_USART_CR3_TXFTIE | LL_USART_CR3_CTSIE));

  LL_USART_ClearFlag(p_uartx, (LL_USART_ICR_TXFECF | LL_USART_ICR_LBDCF | LL_USART_ICR_CTSCF));

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_TX(p_uartx);
    if (huart->hdma_tx != NULL)
    {
      if (huart->hdma_tx->global_state == HAL_DMA_STATE_ACTIVE)
      {
        huart->hdma_tx->p_xfer_abort_cb = UART_DMATxAbortCallback;
        if (HAL_DMA_Abort_IT(huart->hdma_tx) == HAL_OK)
        {
          abort_cplt = 0U;
        }
      }
    }
  }

  if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_uartx);
    if (huart->hdma_rx != NULL)
    {
      if (huart->hdma_rx->global_state == HAL_DMA_STATE_ACTIVE)
      {
        huart->hdma_rx->p_xfer_abort_cb = UART_DMARxAbortCallback;
        if (HAL_DMA_Abort_IT(huart->hdma_rx) == HAL_OK)
        {
          abort_cplt = 0U;
        }
      }
    }
  }

#endif /* USE_HAL_UART_DMA */

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if (abort_cplt != 0U)
  {
    huart->rx_xfer_count = 0U;
    huart->tx_xfer_count = 0U;

    huart->p_rx_isr = NULL;
    huart->p_tx_isr = NULL;
    /* Clear the Error flags in the ICR register */
    LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

    LL_USART_RequestTxDataFlush(p_uartx);
    LL_USART_RequestRxDataFlush(p_uartx);

    huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
    huart->last_reception_error_codes = 0U;
    huart->last_transmission_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

    huart->tx_state = HAL_UART_TX_STATE_IDLE;
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_abort_cplt_callback(huart);
#else
    HAL_UART_AbortCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
  return HAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (Interrupt mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle tx_state to HAL_UART_TX_STATE_IDLE
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_UART_AbortTransmit_IT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t abort_cplt;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);
  abort_cplt = 1U;

  huart->tx_state = HAL_UART_TX_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_TXEIE_TXFNFIE | LL_USART_CR1_TCIE | LL_USART_CR1_TXFEIE));
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_TXFTIE | LL_USART_CR3_CTSIE));
  LL_USART_ClearFlag(p_uartx, (LL_USART_ICR_TXFECF | LL_USART_ICR_CTSCF));

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_TX(p_uartx);
    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_tx != NULL)
    {
      if (huart->hdma_tx->global_state == HAL_DMA_STATE_ACTIVE)
      {
        huart->hdma_tx->p_xfer_abort_cb = UART_DMATxOnlyAbortCallback;
        if (HAL_DMA_Abort_IT(huart->hdma_tx) == HAL_OK)
        {
          abort_cplt = 0U;
        }
      }
    }
  }
#endif /* USE_HAL_UART_DMA */

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if (abort_cplt != 0U)
  {
    huart->tx_xfer_count = 0U;

    huart->p_tx_isr = NULL;

    LL_USART_RequestTxDataFlush(p_uartx);

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
    huart->last_transmission_error_codes = 0;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

    huart->tx_state = HAL_UART_TX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_abort_transmit_cplt_callback(huart);
#else
    HAL_UART_AbortTransmitCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
  return HAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (Interrupt mode).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle rx_state to HAL_UART_RX_STATE_IDLE
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_UART_AbortReceive_IT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t abort_cplt;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);
  abort_cplt = 1U;

  huart->rx_state = HAL_UART_RX_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_RXFFIE
                                   | LL_USART_CR1_IDLEIE | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
  LL_USART_DisableIT_CR2(p_uartx, LL_USART_CR2_LBDIE);
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_LBDCF | LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_LBDCF | LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_uartx);
    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (huart->hdma_rx != NULL)
    {
      if (huart->hdma_rx->global_state == HAL_DMA_STATE_ACTIVE)
      {
        huart->hdma_rx->p_xfer_abort_cb = UART_DMARxOnlyAbortCallback;
        if (HAL_DMA_Abort_IT(huart->hdma_rx) == HAL_OK)
        {
          abort_cplt = 0U;
        }
      }
    }
  }

#endif /* USE_HAL_UART_DMA */

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if (abort_cplt != 0U)
  {
    huart->rx_xfer_count = 0U;

    huart->p_rx_isr = NULL;
    /* Clear the Error flags in the ICR register */
    LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

    LL_USART_RequestRxDataFlush(p_uartx);

    huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
    huart->last_reception_error_codes = 0;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

    huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_abort_receive_cplt_callback(huart);
#else
    HAL_UART_AbortReceiveCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group17
  * @{
  This subsection provides the function handling the interruption of the USARTx in asynchronous mode.

  - HAL_UART_IRQHandler(): process the interruption of an instance

  HAL_UART_IRQHandler() is designed to process the different interruptions in the following order:
    - Error on Rx side (PE, FE, ORE, NE, RTOF)
    - Error on DMA side
    - Data on Rx side
    - Data on Tx side

  Depending on the process function (_Opt included) one's use, different callback might be triggered:

| Process API \n \ \n Callbacks | HAL_UART_Transmit_IT | HAL_UART_Receive_IT |
|-------------------------------|:--------------------:|:-------------------:|
| HAL_UART_TxCpltCallback       |           x          |                     |
| HAL_UART_RxCpltCallback       |                      |          x          |
| HAL_UART_ErrorCallback        |           x          |          x          |

| Process API \n \ \n Callbacks  | HAL_UART_Transmit_DMA | HAL_UART_Receive_DMA |
|--------------------------------|:---------------------:|:--------------------:|
| HAL_UART_TxHalfCpltCallback*   |           x           |                      |
| HAL_UART_TxCpltCallback        |           x           |                      |
| HAL_UART_RxHalfCpltCallback*   |                       |           x          |
| HAL_UART_RxCpltCallback        |                       |           x          |
| HAL_UART_ErrorCallback**       |           x           |           x          |
@note * these callbacks might be called following DMA IRQ management, not USARTx IRQ management.
@note ** these callbacks might be called following DMA IRQ management, or USARTx IRQ management.

| Process API \n \ \n Callbacks      | HAL_UART_Abort_IT | HAL_UART_AbortTransmit_IT | HAL_UART_AbortReceive_IT |
|------------------------------------|:-----------------:|:-------------------------:|:-----------------------: |
| HAL_UART_AbortCpltCallback         |          x        |                           |                          |
| HAL_UART_AbortTransmitCpltCallback |                   |           x               |                          |
| HAL_UART_AbortReceiveCpltCallback  |                   |                           |           x              |
| HAL_UART_ErrorCallback             |          x        |           x               |           x              |

| Process API \n \ \n Callbacks | HAL_UART_ReceiveToIdle_IT | HAL_UART_ReceiveUntilTMO_IT |
|-------------------------------|:-------------------------:|:--------------------------: |
| HAL_UART_RxCpltCallback       |          x                |           x                 |
| HAL_UART_ErrorCallback        |          x                |           x                 |

| Process API \n \ \n Callbacks | HAL_UART_ReceiveUntilCM_IT |
|-------------------------------|:--------------------------------------:|
| HAL_UART_RxCpltCallback       |                   x                    |
| HAL_UART_ErrorCallback        |                   x                    |

| Process API \n \ \n Callbacks | HAL_UART_ReceiveToIdle_DMA      | HAL_UART_ReceiveUntilTMO_DMA |
|-------------------------------|:-------------------------------:|:---------------------------: |
| HAL_UART_RxHalfCpltCallback   |                x                |               x              |
| HAL_UART_RxCpltCallback       |                x                |               x              |
| HAL_UART_ErrorCallback        |                x                |               x              |

| Process API \n \ \n Callbacks | HAL_UART_ReceiveUntilCM_DMA |
|-------------------------------|:--------------------------------------: |
| HAL_UART_RxHalfCpltCallback   |                   x                     |
| HAL_UART_RxCpltCallback       |                   x                     |
| HAL_UART_ErrorCallback        |                   x                     |

| Process API \n \ \n Callbacks | HAL_UART_EnableLINMode |
|-------------------------------|:---------------------: |
|  HAL_UART_LINBreakCallback    |           x            |

  */

/**
  * @brief Handle UART interrupt request.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
void HAL_UART_IRQHandler(hal_uart_handle_t *huart)
{
  ASSERT_DBG_PARAM(huart != NULL);

  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  uint32_t isr_flags = LL_USART_READ_REG(p_uartx, ISR);
  uint32_t cr1_its   = LL_USART_READ_REG(p_uartx, CR1);
  uint32_t cr2_its   = LL_USART_READ_REG(p_uartx, CR2);
  uint32_t cr3_its   = LL_USART_READ_REG(p_uartx, CR3);

  uint32_t error_flags;
  uint32_t error_code = 0U;
  hal_uart_rx_modes_t reception_type;

  reception_type = huart->reception_type;

  if (reception_type != HAL_UART_RX_TO_RTO)
  {
    error_flags = (isr_flags & (uint32_t)(LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_ORE |
                                          LL_USART_ISR_NE | LL_USART_ISR_RTOF));
  }
  else
  {
    error_flags = (isr_flags & (uint32_t)(LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_ORE |
                                          LL_USART_ISR_NE));
  }

  if (error_flags == 0U)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if (((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U)
        && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3_its & LL_USART_CR3_RXFTIE) != 0U)))
    {
      if (huart->p_rx_isr != NULL)
      {
        huart->p_rx_isr(huart);
      }
      return;
    }
  }

  /* If some errors occur */
  if ((error_flags != 0U)
      && ((((cr3_its & (LL_USART_CR3_RXFTIE | LL_USART_CR3_EIE)) != 0U)
           || ((cr1_its & (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_RTOIE)) != 0U))))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isr_flags & LL_USART_ISR_PE) != 0U) && ((cr1_its & LL_USART_CR1_PEIE) != 0U))
    {
      LL_USART_ClearFlag_PE(p_uartx);

      error_code |= HAL_UART_RECEIVE_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isr_flags & LL_USART_ISR_FE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag_FE(p_uartx);

      error_code |= HAL_UART_RECEIVE_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isr_flags & LL_USART_ISR_NE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag_NE(p_uartx);

      error_code |= HAL_UART_RECEIVE_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isr_flags & LL_USART_ISR_ORE) != 0U)
        && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3_its & (LL_USART_CR3_RXFTIE | LL_USART_CR3_EIE)) != 0U)))
    {
      LL_USART_ClearFlag_ORE(p_uartx);

      error_code |= HAL_UART_RECEIVE_ERROR_ORE;
    }

    /* UART Receiver Timeout interrupt occurred ---------------------------------*/
    if (((isr_flags & LL_USART_ISR_RTOF) != 0U) && ((cr1_its & LL_USART_CR1_RTOIE) != 0U)
        && (reception_type != HAL_UART_RX_TO_RTO))
    {
      LL_USART_ClearFlag_RTO(p_uartx);

      error_code |= HAL_UART_RECEIVE_ERROR_RTO;
    }

    /* Call UART Error callback function if need be ----------------------------*/
    if (error_code != 0U)
    {
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
      huart->last_reception_error_codes = error_code;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
      /* UART in mode Receiver --------------------------------------------------*/
      if (((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U)
          && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
              || ((cr3_its & LL_USART_CR3_RXFTIE) != 0U)))
      {
        if (huart->p_rx_isr != NULL)
        {
          huart->p_rx_isr(huart);
        }
      }

      /* If Error is to be considered as blocking :
          - Receiver Timeout error in Reception
          - Overrun error in Reception
          - any error occurs in DMA mode reception
      */
      if ((LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
          || ((uint32_t)(error_code & (HAL_UART_RECEIVE_ERROR_RTO | HAL_UART_RECEIVE_ERROR_ORE)) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        UART_EndRxTransfer(huart);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
        if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
        {
          LL_USART_DisableDMAReq_RX(p_uartx);
          /* Abort the UART DMA Rx channel */
          if (huart->hdma_rx != NULL)
          {
            /* Set the UART DMA Abort callback :
                will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
            huart->hdma_rx->p_xfer_abort_cb = UART_DMAAbortOnError;

            if (HAL_DMA_Abort_IT(huart->hdma_rx) != HAL_OK)
            {
              /* Call Directly huart->hdmarx->p_xfer_abort_cb function in case of error */
              huart->hdma_rx->p_xfer_abort_cb(huart->hdma_rx);
            }
          }
          else
          {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            huart->p_error_callback(huart);
#else
            HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

          }
        }
        else
#endif /* USE_HAL_UART_DMA */
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_error_callback(huart);
#else
          HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Non-blocking error: transfer can continue.
           Error is notified to the user through the error callback. */
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_error_callback(huart);
#else
        HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
    }
    return;
  } /* End if some error occurs */

  if (reception_type != HAL_UART_RX_STANDARD)
  {
    hal_uart_rx_event_types_t rx_type = HAL_UART_RX_EVENT_TC;
    uint32_t it_to_clear = 0U;

    if ((reception_type == HAL_UART_RX_TO_IDLE)
        && ((isr_flags & LL_USART_ISR_IDLE) != 0U)
        && ((cr1_its & LL_USART_CR1_IDLEIE) != 0U))
    {
      rx_type = HAL_UART_RX_EVENT_IDLE;
      it_to_clear = LL_USART_CR1_IDLEIE;
      LL_USART_ClearFlag_IDLE(p_uartx);
    }
    else if ((reception_type == HAL_UART_RX_TO_RTO)
             && ((isr_flags & LL_USART_ISR_RTOF) != 0U)
             && ((cr1_its & LL_USART_CR1_RTOIE) != 0U))
    {
      rx_type = HAL_UART_RX_EVENT_RTO;
      it_to_clear = LL_USART_CR1_RTOIE;
      LL_USART_ClearFlag_RTO(p_uartx);
    }
    else if ((reception_type == HAL_UART_RX_TO_CHAR_MATCH)
             && ((isr_flags & LL_USART_ISR_CMF) != 0U)
             && ((cr1_its & LL_USART_CR1_CMIE) != 0U))
    {
      rx_type = HAL_UART_RX_EVENT_CHAR_MATCH;
      it_to_clear = LL_USART_CR1_CMIE;
      LL_USART_ClearFlag_CM(p_uartx);
    }
    else
    {
      /* do nothing */
    }
    if (rx_type != HAL_UART_RX_EVENT_TC)
    {
#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
      if (LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U)
      {
        uint32_t nb_remaining_rx_data =
          LL_DMA_GetBlkDataLength((DMA_Channel_TypeDef *)(uint32_t)huart->hdma_rx->instance);
        uint32_t rx_size = huart->rx_xfer_size;
        huart->rx_xfer_count = nb_remaining_rx_data;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
        if (huart->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
        {
          /* DMA mode enabled */
          /* Check received length : If all expected data are received, do nothing,
            (DMA cplt callback will be called).
            Otherwise, if at least one data has already been received, IDLE/CM/RTO events are to be notified to user */

          if ((nb_remaining_rx_data > 0U)
              && (nb_remaining_rx_data < rx_size))
          {
            /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
            LL_USART_DisableIT_PE(p_uartx);
            LL_USART_DisableIT_ERROR(p_uartx);

            /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
              in the UART CR3 register */
            LL_USART_DisableDMAReq_RX(p_uartx);

            LL_USART_DisableIT_CR1(p_uartx, it_to_clear);

            huart->hdma_rx->p_xfer_abort_cb = UART_DMAAbortOnSuccessCallback;
            /* Last bytes received, so no need as the abort is immediate */
            (void)HAL_DMA_Abort_IT(huart->hdma_rx);
          }
          return;
        }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
        else
        {
          if ((nb_remaining_rx_data > 0U) && (nb_remaining_rx_data < rx_size))
          {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            huart->p_rx_cplt_callback(huart, (rx_size - nb_remaining_rx_data), rx_type);
#else
            HAL_UART_RxCpltCallback(huart, (rx_size - nb_remaining_rx_data), rx_type);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
            return;
          }
        }
#endif /* USE_HAL_DMA_LINKEDLIST */
      }
      else
#endif /* USE_HAL_UART_DMA */
      {
        /* DMA mode not enabled */
        /* Check received length : If all expected data are received, do nothing.
          Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
        uint32_t rx_size = huart->rx_xfer_size;
        uint16_t nb_rx_data = (uint16_t)(rx_size - huart->rx_xfer_count);
        if ((huart->rx_xfer_count > 0U) && (nb_rx_data > 0U))
        {
          /* Disable the UART Parity Error Interrupt and RXNE interrupts */
          LL_USART_DisableIT_CR1(p_uartx, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));

          /* Disable the UART Error Interrupt(Frame, noise, overrun) and RX FIFO Threshold interrupt */
          LL_USART_DisableIT_CR3(p_uartx, (USART_CR3_EIE | USART_CR3_RXFTIE));

          huart->reception_type = HAL_UART_RX_STANDARD;

          huart->p_rx_isr = NULL;

          LL_USART_DisableIT_CR1(p_uartx, it_to_clear);

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_rx_cplt_callback(huart, nb_rx_data, rx_type);
#else
          HAL_UART_RxCpltCallback(huart, nb_rx_data, rx_type);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
          huart->rx_state = HAL_UART_RX_STATE_IDLE;
        }
        return;
      }
    }
  }

  /* UART wakeup from Stop mode interrupt occurred. ---------------------------*/
  if (((isr_flags & LL_USART_ISR_WUF) != 0U) && ((cr3_its & LL_USART_CR3_WUFIE) != 0U))
  {
    LL_USART_ClearFlag_WKUP(p_uartx);

    /* UART Rx state is not reset as a reception process might be ongoing.
       If UART handle state fields need to be reset to READY, do this in the wakeup callback. */

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_wakeup_callback(huart);
#else
    HAL_UART_WakeupCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_TXE_TXFNF) != 0U)
      && (((cr1_its & LL_USART_CR1_TXEIE_TXFNFIE) != 0U)
          || ((cr3_its & LL_USART_CR3_TXFTIE) != 0U)))
  {
    if (huart->p_tx_isr != NULL)
    {
      huart->p_tx_isr(huart);
    }
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if (((isr_flags & LL_USART_ISR_TC) != 0U) && ((cr1_its & LL_USART_CR1_TCIE) != 0U))
  {
    UART_EndTransmit_IT(huart);
    return;
  }

  /* UART TX FIFO Empty occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_TXFE) != 0U) && ((cr1_its & LL_USART_CR1_TXFEIE) != 0U))
  {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_tx_fifo_empty_callback(huart);
#else
    HAL_UART_TxFifoEmptyCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }

  /* UART RX FIFO Full occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_RXFF) != 0U) && ((cr1_its & LL_USART_CR1_RXFFIE) != 0U))
  {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_rx_fifo_full_callback(huart);
#else
    HAL_UART_RxFifoFullCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }
  /* UART CTS occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_CTSIF) != 0U) && ((cr3_its & LL_USART_CR3_CTSIE) != 0U))
  {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_clear_to_send_callback(huart);
#else
    HAL_UART_ClearToSendCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }
  /* UART LIN break occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_LBDF) != 0U) && ((cr2_its & LL_USART_CR2_LBDIE) != 0U))
  {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_lin_break_callback(huart);
#else
    HAL_UART_LINBreakCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }
}
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group13
  * @{
  This subsection provides a set of functions to use advanced I/O operations on the USARTx
  instance in asynchronous modes.
  Before using advanced I/O operations, configure the instance in asynchronous
  mode with HAL_UART_SetConfig().

  Three specific reception methods are provided:
    - Receive until Idle event is received or specified amount of data is received:
      - HAL_UART_ReceiveToIdle(): in polling mode
      - HAL_UART_ReceiveToIdle_IT(): in interrupt mode
      - HAL_UART_ReceiveToIdle_IT_Opt(): in interrupt mode, with optional interrupt selection
      - HAL_UART_ReceiveToIdle_DMA(): in DMA mode
      - HAL_UART_ReceiveToIdle_DMA_Opt(): in DMA mode, with Optional interrupts selection

    - Receive until Timeout (TMO) event is received or specified amount of data is received:
      - HAL_UART_ReceiveUntilTMO(): in polling mode
      - HAL_UART_ReceiveUntilTMO_IT(): in interrupt mode
      - HAL_UART_ReceiveUntilTMO_IT_Opt(): in interrupt mode, with optional interrupt selection
      - HAL_UART_ReceiveUntilTMO_DMA(): in DMA mode
      - HAL_UART_ReceiveUntilTMO_DMA_Opt(): in DMA mode, with optional interrupt selection

    - Receive until Character Match (CM) event is received or specified amount of data is received:
      - HAL_UART_ReceiveUntilCM(): in polling mode
      - HAL_UART_ReceiveUntilCM_IT(): in interrupt mode
      - HAL_UART_ReceiveUntilCM_IT()_Opt: in interrupt mode, with optional interrupt selection
      - HAL_UART_ReceiveUntilCM_DMA(): in DMA mode
      - HAL_UART_ReceiveUntilCM_DMA_Opt(): in DMA mode, with optional interrupt selection

  To send break character in LIN mode:
    - HAL_UART_SendLINBreak()

  To send a specific request:
    - HAL_UART_SendRequest()

  A set of Transfer Complete Callbacks are provided in non-blocking mode (IT and DMA):
    - HAL_UART_RxHalfCpltCallback()
    - HAL_UART_RxCpltCallback()
    - HAL_UART_ErrorCallback()

  Abort non-blocking mode transfers using the Abort APIs:
    - HAL_UART_Abort()
    - HAL_UART_AbortReceive()
    - HAL_UART_Abort_IT()
    - HAL_UART_AbortReceive_IT()

  For abort services based on interrupts (HAL_UART_Abortxxx_IT), a set of Abort Complete callbacks are provided:
    - HAL_UART_AbortCpltCallback()
    - HAL_UART_AbortReceiveCpltCallback()

  In non-blocking mode transfers, possible errors are split into two categories:
    - Error is considered recoverable and non-blocking: Transfer can run to completion, but error severity is
      to be evaluated by the user. This concerns Frame Error, Parity Error, or Noise Error
      in interrupt mode reception.
      The received character is then retrieved and stored in the Rx buffer, the error code is set to allow the user
      to identify the error type, and the HAL_UART_ErrorCallback() user callback is executed.
      Transfer remains ongoing on the UART side.
      If you want to abort it, call the Abort services.
    - Error is considered blocking: Transfer cannot be completed properly and is aborted.
      This concerns Overrun Error in interrupt mode reception and all errors in DMA mode.
      The error code is set to allow the user to identify the error type, and the HAL_UART_ErrorCallback()
      user callback is executed.

  In Half Duplex communication, do not run the transmit and receive process in parallel.
  */

/**
  * @brief Send Break Character on the line.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK The Break has been sent.
  * @retval HAL_BUSY Concurrent process ongoing.
  */
hal_status_t HAL_UART_SendLINBreak(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(IS_UART_LIN_INSTANCE(p_uartx));
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_RequestBreakSending(p_uartx);

  return HAL_OK;
}

/**
  * @brief Send specific UART request.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param request request type to process.
  * @retval HAL_OK The request has been sent.
  * @retval HAL_BUSY Concurrent process ongoing.
  */
hal_status_t HAL_UART_SendRequest(hal_uart_handle_t *huart, hal_uart_request_t request)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(IS_UART_REQUEST_PARAMETER(request));
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx) || (request != HAL_UART_REQUEST_AUTO_BAUD_RATE));
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  LL_USART_SetRequest(p_uartx, (uint32_t)request);

  return HAL_OK;
}

/**
  * @brief Receive an amount of data in blocking mode till either the expected number of data
  *        is received or an IDLE event occurs.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param size_byte          Amount of data elements (uint8_t or uint16_t) to be received.
  * @param p_rx_size_byte     Pointer to the number of data elements finally received
  *                           (could be lower than size, in case reception ends on IDLE event).
  * @param timeout_ms         Timeout duration expressed in ms (covers the whole reception sequence).
  * @note  OK is returned if reception is completed (expected number of data has been received)
  *        or if reception is stopped after IDLE event (less than the expected number of data has been received)
  *        In this case, p_rx_size_byte output parameter indicates number of data available in reception buffer.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  *          In this case, size must indicate the number of uint16_t available through p_data.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveToIdle(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                    uint32_t *p_rx_size_byte, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(p_rx_size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  status = UART_Start_Receive_Polling(huart, p_data, size_byte, p_rx_size_byte, timeout_ms, HAL_UART_RX_TO_IDLE);

  huart->rx_state = HAL_UART_RX_STATE_IDLE;
  return status;
}

/**
  * @brief Receive an amount of data in interrupt mode till either the expected number of data
  *        is received or an IDLE event occurs.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param size_byte          Amount of data elements (uint8_t or uint16_t) to be received.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  *          In this case, size must indicate the number of uint16_t available through p_data.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveToIdle_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_IDLE;

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte,
                               HAL_UART_RX_TO_IDLE, HAL_UART_OPT_RX_IT_NONE);
}

/**
  * @brief Receive an amount of data in interrupt mode till either the expected number of data
  *        is received or an IDLE event occurs, with optional interrupts selection. Allows the user to enable
  *        optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param size_byte          Amount of data elements (uint8_t or uint16_t) to be received.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  *          In this case, size must indicate the number of uint16_t available through p_data.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveToIdle_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                           uint32_t interrupts)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_IDLE;

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_IDLE, interrupts);
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Receive an amount of data in DMA mode till either the expected number
  *        of data is received or an IDLE event occurs.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param size_byte          Amount of data elements (uint8_t or uint16_t) to be received.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  *          In this case, size must indicate the number of uint16_t available through p_data.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveToIdle_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_IDLE;

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_IDLE,
                                 HAL_UART_OPT_DMA_RX_IT_HT));
}

/**
  * @brief Receive an amount of data in DMA mode till either the expected number
  *        of data is received or an IDLE event occurs, with optional interrupts selection.
  *        Allows the user to enable optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param size_byte          Amount of data elements (uint8_t or uint16_t) to be received.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  *          In this case, size must indicate the number of uint16_t available through p_data.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveToIdle_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                            uint32_t interrupts)
{
  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_UART_DMA_RX_VALID_SILENT_MODE(huart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_IDLE;

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_IDLE,
                                 interrupts));
}
#endif /* USE_HAL_UART_DMA */

/**
  * @brief Receive an amount of data in blocking mode till the timeout(TMO) expires or an amount of data is received.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param p_rx_size_byte     Pointer to the number of data elements finally received
  *                           (could be lower than size_byte, in case reception ends on IDLE event).
  * @param char_timeout_bit   Timeout duration expressed in bit.
  * @note  HAL_OK is returned if the timeout expires.
  * @note This feature is not available for LPUART instances.
  *        In this case, p_rx_size_byte output parameter indicates number of data available in reception buffer.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilTMO(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                      uint32_t *p_rx_size_byte, uint32_t char_timeout_bit)
{
  hal_status_t status;
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(p_rx_size_byte != NULL);
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(char_timeout_bit));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  LL_USART_SetRxTimeout(p_uartx, char_timeout_bit);
  LL_USART_EnableRxTimeout(p_uartx);

  status = UART_Start_Receive_Polling(huart, p_data, size_byte, p_rx_size_byte, 0xFFFFFFFFU, HAL_UART_RX_TO_RTO);

  LL_USART_DisableRxTimeout(p_uartx);
  huart->rx_state = HAL_UART_RX_STATE_IDLE;

  return status;
}

/**
  * @brief Receive an amount of data in interrupt mode till the timeout(TMO) expires or an amount of data is received.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param char_timeout_bit   Timeout duration expressed in bit.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @note This feature is not available for LPUART instances.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilTMO_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                         uint32_t char_timeout_bit)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(char_timeout_bit));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_RTO;

  LL_USART_SetRxTimeout(p_uartx, char_timeout_bit);
  LL_USART_EnableRxTimeout(p_uartx);

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_RTO, HAL_UART_OPT_RX_IT_NONE);
}

/**
  * @brief Receive an amount of data in interrupt mode till the timeout(TMO) expires or an amount of data is received,
  *        with optional interrupts selection. Allows the user to enable optional interrupts part of
  *        \ref UART_Receive_IT_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param char_timeout_bit   Timeout duration expressed in bit.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @note This feature is not available for LPUART instances.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilTMO_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                             uint32_t char_timeout_bit, uint32_t interrupts)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(char_timeout_bit));
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_RTO;

  LL_USART_SetRxTimeout(p_uartx, char_timeout_bit);
  LL_USART_EnableRxTimeout(p_uartx);

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_RTO, interrupts);
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Receive an amount of data in DMA mode till the timeout(TMO) expires or an amount of data is received.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param char_timeout_bit   Timeout duration expressed in bit.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @note This feature is not available for LPUART instances.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilTMO_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                          uint32_t char_timeout_bit)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(char_timeout_bit));
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_RTO;

  LL_USART_SetRxTimeout(p_uartx, char_timeout_bit);
  LL_USART_EnableRxTimeout(p_uartx);

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_RTO,
                                 HAL_UART_OPT_DMA_RX_IT_HT));
}

/**
  * @brief Receive an amount of data in DMA mode till the timeout(TMO) expires or an amount of data is received,
  *        with optional interrupts selection. Allows the user to enable optional interrupts part of
  *        \ref UART_Receive_DMA_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param char_timeout_bit   Timeout duration expressed in bit.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @note This feature is not available for LPUART instances.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilTMO_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                              uint32_t char_timeout_bit, uint32_t interrupts)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM(huart != NULL);
  p_uartx = UART_GET_INSTANCE(huart);
  ASSERT_DBG_PARAM(!IS_LPUART_INSTANCE(p_uartx));
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_RECEIVER_TIMEOUT_VALUE(char_timeout_bit));
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_DMA(interrupts));
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_UART_DMA_RX_VALID_SILENT_MODE(huart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_RTO;

  LL_USART_SetRxTimeout(p_uartx, char_timeout_bit);
  LL_USART_EnableRxTimeout(p_uartx);

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_RTO,
                                 interrupts));
}

#endif /* USE_HAL_UART_DMA */

/**
  * @brief Receive an amount of data in blocking mode till the character passed in parameters match
  *        the received sequence or an amount of data is received.
  *        In this case, p_rx_size_byte output parameter indicates number of data available in reception buffer.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param character          Character to match in the received sequence.
  * @param p_rx_size_byte     Pointer to the number of data elements finally received.
  * @param timeout_ms         Timeout duration expressed in ms (covers the whole reception sequence).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilCM(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                     uint8_t character, uint32_t *p_rx_size_byte, uint32_t timeout_ms)
{
  hal_status_t status;
  USART_TypeDef *p_uartx;
  hal_uart_parity_t parity;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(p_rx_size_byte != NULL);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(LL_USART_IsEnabledMuteMode(p_uartx) == 0U);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) != LL_USART_DATAWIDTH_9_BIT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);
  parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  if (parity != HAL_UART_PARITY_NONE)
  {
    UART_Parity_Computation(huart, &character);
  }

  UART_ENSURE_INSTANCE_DISABLED(p_uartx);
  LL_USART_SetNodeAddress(p_uartx, (uint32_t)character);
  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  status = UART_Start_Receive_Polling(huart, p_data, size_byte, p_rx_size_byte, timeout_ms,
                                      HAL_UART_RX_TO_CHAR_MATCH);

  huart->rx_state = HAL_UART_RX_STATE_IDLE;

  return status;
}

/**
  * @brief Receive an amount of data in interrupt mode till the character passed in parameters match
  *        the received sequence or an amount of data is received.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data     Pointer to data buffer.
  * @param size_byte  Amount of data elements to be received.
  * @param character  Character to match in the received sequence.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilCM_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                        uint8_t character)
{
  USART_TypeDef *p_uartx;
  hal_uart_parity_t parity;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(LL_USART_IsEnabledMuteMode(p_uartx) == 0U);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) != LL_USART_DATAWIDTH_9_BIT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_CHAR_MATCH;
  parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  if (parity != HAL_UART_PARITY_NONE)
  {
    UART_Parity_Computation(huart, &character);
  }
  UART_ENSURE_INSTANCE_DISABLED(p_uartx);
  LL_USART_SetNodeAddress(p_uartx, (uint32_t)character);
  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_CHAR_MATCH,
                               HAL_UART_OPT_RX_IT_NONE);
}

/**
  * @brief Receive an amount of data in interrupt mode till the character passed in parameters match
  *        the received sequence or an amount of data is received, with optional interrupts selection.
  *        Allows the user to enable optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data     Pointer to data buffer.
  * @param size_byte  Amount of data elements to be received.
  * @param character  Character to match in the received sequence.
  * @param interrupts Optional interrupts part of \ref UART_Receive_IT_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *        number of received data elements.
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilCM_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                            uint8_t character, uint32_t interrupts)
{
  USART_TypeDef *p_uartx;
  hal_uart_parity_t parity;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(LL_USART_IsEnabledMuteMode(p_uartx) == 0U);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) != LL_USART_DATAWIDTH_9_BIT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);


  huart->reception_type = HAL_UART_RX_TO_CHAR_MATCH;
  parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  if (parity != HAL_UART_PARITY_NONE)
  {
    UART_Parity_Computation(huart, &character);
  }
  UART_ENSURE_INSTANCE_DISABLED(p_uartx);
  LL_USART_SetNodeAddress(p_uartx, (uint32_t)character);
  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return UART_Start_Receive_IT(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_CHAR_MATCH,
                               interrupts);
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief Receive an amount of data in DMA mode till the character passed in parameters match
  *        the received sequence or an amount of data is received.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param character          Character to match in the received sequence.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilCM_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                         uint8_t character)
{
  USART_TypeDef *p_uartx;
  hal_uart_parity_t parity;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(LL_USART_IsEnabledMuteMode(p_uartx) == 0U);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) != LL_USART_DATAWIDTH_9_BIT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_CHAR_MATCH;

  parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  if (parity != HAL_UART_PARITY_NONE)
  {
    UART_Parity_Computation(huart, &character);
  }
  UART_ENSURE_INSTANCE_DISABLED(p_uartx);
  LL_USART_SetNodeAddress(p_uartx, (uint32_t)character);
  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_CHAR_MATCH,
                                 HAL_UART_OPT_DMA_RX_IT_HT));
}

/**
  * @brief Receive an amount of data in DMA mode till the character passed in parameters match
  *        the received sequence or an amount of data is received, with optional interrupts selection.
  *        Allows the user to enable optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @param huart              Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of data elements to be received.
  * @param character          Character to match in the received sequence.
  * @param interrupts         Optional interrupts part of \ref UART_Receive_DMA_Optional_Interrupts.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @warning When the UART parity is enabled (PCE bit from the CR1 register = 1), the received data contain
  *          the parity bit (MSB position).
  * @warning When UART parity is not enabled (PCE bit from the CR1 register = 0), and Word Length is configured to
  *          9 bits (M1-M0 from the CR1 register = 01), the received data is handled as a set of uint16_t.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         DMA handler not set or Error during instance enabling.
  */
hal_status_t HAL_UART_ReceiveUntilCM_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                             uint8_t character, uint32_t interrupts)
{
  USART_TypeDef *p_uartx;
  hal_uart_parity_t parity;

  ASSERT_DBG_PARAM(huart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_UART_OPT_RX_DMA(interrupts));
  ASSERT_DBG_PARAM(huart->hdma_rx != NULL);
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_UART_DMA_RX_VALID_SILENT_MODE(huart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);
  ASSERT_DBG_STATE(huart->rx_state, HAL_UART_RX_STATE_IDLE);
  ASSERT_DBG_STATE(huart->tx_state, HAL_UART_TX_STATE_IDLE);

  p_uartx = UART_GET_INSTANCE(huart);

  ASSERT_DBG_PARAM(LL_USART_IsEnabledMuteMode(p_uartx) == 0U);
  ASSERT_DBG_PARAM(LL_USART_GetDataWidth(p_uartx) != LL_USART_DATAWIDTH_9_BIT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(huart, rx_state, HAL_UART_RX_STATE_IDLE, HAL_UART_RX_STATE_ACTIVE);

  huart->reception_type = HAL_UART_RX_TO_CHAR_MATCH;

  parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  if (parity != HAL_UART_PARITY_NONE)
  {
    UART_Parity_Computation(huart, &character);
  }
  UART_ENSURE_INSTANCE_DISABLED(p_uartx);
  LL_USART_SetNodeAddress(p_uartx, (uint32_t)character);
  UART_ENSURE_INSTANCE_ENABLED(p_uartx);

  return (UART_Start_Receive_DMA(huart, (uint8_t *)p_data, size_byte, HAL_UART_RX_TO_CHAR_MATCH,
                                 interrupts));
}

/**
  * @}
  */

#endif /* USE_HAL_UART_DMA */
/** @addtogroup UART_Exported_Functions_Group14
  * @{
  This subsection provides functions to read the current frequency, state, and last error codes
  of the USARTx in asynchronous mode:

    - HAL_UART_GetClockFreq(): Return the current clock frequency of the UART peripheral.
    - HAL_UART_GetState(): Return the UART handle state.
    - HAL_UART_GetTxState(): Return the HAL UART Tx process state.
    - HAL_UART_GetRxState(): Return the HAL UART Rx process state.
    - HAL_UART_GetLastErrorCodes(): Return the last error of the UART handle.
  */

/** @brief  Return the peripheral clock frequency for UART.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure that contains
  *               the configuration information for UART module.
  * @retval uint32_t Frequency in Hz.
  * @retval 0 source clock of the huart not configured or not ready.
  */
uint32_t HAL_UART_GetClockFreq(const hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  ASSERT_DBG_PARAM((huart != NULL));

  ASSERT_DBG_STATE(huart->global_state, HAL_UART_STATE_CONFIGURED);

  p_uartx = UART_GET_INSTANCE(huart);

  return HAL_RCC_UART_GetKernelClkFreq(p_uartx);
}

/**
  * @brief Return the UART handle state.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_state_t UART state
  */
hal_uart_state_t HAL_UART_GetState(const hal_uart_handle_t *huart)
{
  ASSERT_DBG_PARAM(huart != NULL);

  return huart->global_state;
}

/**
  * @brief Return the HAL UART Tx process state.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_tx_state_t UART Tx process state
  */
hal_uart_tx_state_t HAL_UART_GetTxState(const hal_uart_handle_t *huart)
{
  ASSERT_DBG_PARAM(huart != NULL);

  return huart->tx_state;
}

/**
  * @brief  Return the HAL UART Rx process state.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval hal_uart_rx_state_t UART Rx process state
  */
hal_uart_rx_state_t HAL_UART_GetRxState(const hal_uart_handle_t *huart)
{
  ASSERT_DBG_PARAM(huart != NULL);

  return huart->rx_state;
}

/**
  * @brief  Return the UART last errors.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval Current last error codes.
  */
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
uint32_t HAL_UART_GetLastErrorCodes(const hal_uart_handle_t *huart)
{
  uint32_t tmp;

  ASSERT_DBG_PARAM(huart != NULL);

  tmp = huart->last_reception_error_codes;
  return (huart->last_transmission_error_codes | tmp);
}
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

/**
  * @}
  */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup UART_Exported_Functions_Group15
  * @{
  This subsection provides functions to control the bus of the USARTx instance:
    - HAL_UART_AcquireBus(): Acquire the bus
    - HAL_UART_ReleaseBus(): Release the bus.

  For multi-task applications, use the bus operation functions to avoid race conditions.
  */
/**
  * @brief  Acquire the current instance bus.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  timeout_ms Timeout in milliseconds for the Acquire to expire.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_UART_AcquireBus(hal_uart_handle_t *huart, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((huart != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreTake(&huart->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the current instance bus.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_UART_ReleaseBus(hal_uart_handle_t *huart)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((huart != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreRelease(&huart->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /*USE_HAL_MUTEX */
#if defined (USE_HAL_UART_USER_DATA) && (USE_HAL_UART_USER_DATA == 1)

/** @addtogroup UART_Exported_Functions_Group16
  * @{
  This subsection provides functions to set user-specific data for a USARTx instance:
    - HAL_UART_SetUserData(): Set user data in the handler.
    - HAL_UART_GetUserData(): Get user data from the handler.
  */

/**
  * @brief Store the user data pointer into the handle.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param p_user_data Pointer to the user data.
  */
void HAL_UART_SetUserData(hal_uart_handle_t *huart, const void *p_user_data)
{
  ASSERT_DBG_PARAM(huart != NULL);

  huart->p_user_data = p_user_data;
}

/**
  * @brief Retrieve the user data pointer from the handle.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval Pointer to the user data.
  */
const void *HAL_UART_GetUserData(const hal_uart_handle_t *huart)
{
  ASSERT_DBG_PARAM(huart != NULL);

  return (huart->p_user_data);
}

/**
  * @}
  */
#endif /* USE_HAL_UART_USER_DATA */

/** @addtogroup UART_Exported_Functions_Group18
  * @{
  This subsection provides the default weak callbacks of the USARTx instance.
  Refer to HAL_UART_IRQHandler() documentation to get the details of which callback is triggered for each process
  function. Refer to the "How to use the UART HAL module driver" section to find the association between
  callbacks, registration functions, and default callback values.
  */
/**
  * @brief Tx Transfer completed callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_TxCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief Tx Half Transfer completed callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_TxHalfCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_TxHalfCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief Rx Transfer completed callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param size_byte Number of bytes received.
  * @param rx_event Event that triggered the callback.
  */
__WEAK void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte, hal_uart_rx_event_types_t rx_event)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);
  STM32_UNUSED(size_byte);
  STM32_UNUSED(rx_event);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief Rx Half Transfer completed callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_RxHalfCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_RxHalfCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief UART error callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_ErrorCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_ErrorCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Abort Complete callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_AbortCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Abort Transmit Complete callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_AbortTransmitCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Abort Receive Complete callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_AbortReceiveCpltCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_AbortReceiveCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief UART wakeup from Stop mode callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_WakeupCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_WakeupCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Rx FIFO full callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_RxFifoFullCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_RxFifoFullCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Tx FIFO empty callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_TxFifoEmptyCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_TxFifoEmptyCallback can be implemented in the user file.
   */
}

/**
  * @brief UART LIN break callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_LINBreakCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_LINBreakCallback can be implemented in the user file.
   */
}

/**
  * @brief UART Clear to send callback.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
__WEAK void HAL_UART_ClearToSendCallback(hal_uart_handle_t *huart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(huart);

  /** @warning This function must not be modified. When the callback is needed,
               the HAL_UART_ClearToSendCallback can be implemented in the user file.
   */
}

/**
  * @}
  */
/**
  * @}
  */
/** @addtogroup UART_Private_Functions UART Private Functions
  * @{
  */

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
/**
  * @brief Initialize the callbacks to their default values.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
void UART_InitCallbacksToDefault(hal_uart_handle_t *huart)
{
  /* Init the UART Callback settings */
  huart->p_tx_half_cplt_callback        = HAL_UART_TxHalfCpltCallback;       /* Legacy weak TxHalfCpltCallback        */
  huart->p_tx_cplt_callback             = HAL_UART_TxCpltCallback;           /* Legacy weak TxCpltCallback            */
  huart->p_rx_half_cplt_callback        = HAL_UART_RxHalfCpltCallback;       /* Legacy weak RxHalfCpltCallback        */
  huart->p_rx_cplt_callback             = HAL_UART_RxCpltCallback;           /* Legacy weak RxCpltCallback            */
  huart->p_error_callback               = HAL_UART_ErrorCallback;            /* Legacy weak ErrorCallback             */
  huart->p_abort_cplt_callback          = HAL_UART_AbortCpltCallback;        /* Legacy weak AbortCpltCallback         */
  huart->p_abort_transmit_cplt_callback = HAL_UART_AbortTransmitCpltCallback;/* Legacy weak AbortTransmitCpltCallback */
  huart->p_abort_receive_cplt_callback  = HAL_UART_AbortReceiveCpltCallback; /* Legacy weak AbortReceiveCpltCallback  */
  huart->p_wakeup_callback              = HAL_UART_WakeupCallback;           /* Legacy weak WakeUpCallback            */
  huart->p_rx_fifo_full_callback        = HAL_UART_RxFifoFullCallback;       /* Legacy weak RxFifoFullCallback        */
  huart->p_tx_fifo_empty_callback       = HAL_UART_TxFifoEmptyCallback;      /* Legacy weak TxFifoEmptyCallback       */
  huart->p_clear_to_send_callback       = HAL_UART_ClearToSendCallback;      /* Legacy weak ClearToSendCallback       */
  huart->p_lin_break_callback           = HAL_UART_LINBreakCallback;         /* Legacy weak LINBreakCallback          */
}
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

/**
  * @brief Abort current UART exchange.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART Abort completed.
  */
hal_status_t UART_Abort(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_TXEIE_TXFNFIE
                                   | LL_USART_CR1_TCIE | LL_USART_CR1_RXFFIE | LL_USART_CR1_TXFEIE | LL_USART_CR1_IDLEIE
                                   | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
  LL_USART_DisableIT_CR2(p_uartx, LL_USART_CR2_LBDIE);
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE | LL_USART_CR3_TXFTIE | LL_USART_CR3_CTSIE));
  LL_USART_ClearFlag(p_uartx, (LL_USART_ICR_TXFECF | LL_USART_ICR_LBDCF | LL_USART_ICR_CTSCF));

  return HAL_OK;
}
/**
  * @brief If not enabled, enable the UART instance and check acknowledge bits.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @retval HAL_OK UART Ready for processing.
  * @retval HAL_TIMEOUT UART took too long to acknowledge.
  */
hal_status_t UART_CheckEnabledState(hal_uart_handle_t *huart)
{
  uint32_t count;
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  /* Check if Instance is enabled */
  /* - If Instance is already enabled : nothing to do */
  /* - If not, enable instance and check TEACK and REACK bits if needed */
  if (LL_USART_IsEnabled(p_uartx) == 0U)
  {
    LL_USART_Enable(p_uartx);

    if (LL_USART_IsEnabledDirectionTx(p_uartx) != 0U)
    {
      /** 8 is the number of required instructions cycles for the below loop statement.
        * The UART_ENABLE_TIMEOUT_MS is expressed in ms.
        */
      count = UART_ENABLE_TIMEOUT_MS * (SystemCoreClock / 8U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          /* Timeout occurred */
          return HAL_TIMEOUT;
        }
      } while (LL_USART_IsActiveFlag_TEACK(p_uartx) == 0U);
    }

    if (LL_USART_IsEnabledDirectionRx(p_uartx) != 0U)
    {
      /** 8 is the number of required instructions cycles for the below loop statement.
        * The UART_ENABLE_TIMEOUT_MS is expressed in ms.
        */
      count = UART_ENABLE_TIMEOUT_MS * (SystemCoreClock / 8U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          /* Timeout occurred */
          return HAL_TIMEOUT;
        }
      } while (LL_USART_IsActiveFlag_REACK(p_uartx) == 0U);
    }
  }

  return HAL_OK;
}

/**
  * @brief This function handles UART communication timeout. It waits
  *        until a flag is no longer in the specified status.
  * @param huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param flag       Specifies the UART flag to check
  * @param status     The actual flag status (1U or 0U)
  * @param tick_start Tick start value
  * @param timeout_ms Timeout duration
  * @retval HAL_OK UART Flag unset.
  * @retval HAL_TIMEOUT UART took to long to acknowledge.
  */
hal_status_t UART_WaitOnFlagUntilTimeout(hal_uart_handle_t *huart, uint32_t flag, uint32_t status,
                                         uint32_t tick_start, uint32_t timeout_ms)
{
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  /* Wait until flag is set */
  while (((LL_USART_READ_REG(p_uartx, ISR) & flag) == status))
  {
    if (huart->reception_type != HAL_UART_RX_TO_RTO)
    {
      if (LL_USART_IsEnabledDirectionRx(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_RTO(p_uartx) != 0U)
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error)
             interrupts for the interrupt process */
          LL_USART_DisableIT_CR1(p_uartx, LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE |
                                 LL_USART_CR1_TXEIE_TXFNFIE);
          LL_USART_DisableIT_CR3(p_uartx, LL_USART_CR3_EIE);

          LL_USART_ClearFlag_RTO(p_uartx);

          return HAL_TIMEOUT;
        }
      }
    }
    /* Check for the timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        if (((LL_USART_READ_REG(p_uartx, ISR) & flag) == status))
        {
          /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error)
           interrupts for the interrupt process */
          LL_USART_DisableIT_CR1(p_uartx, LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE |
                                 LL_USART_CR1_TXEIE_TXFNFIE);
          LL_USART_DisableIT_CR3(p_uartx, LL_USART_CR3_EIE);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Start receive operation in interrupt mode.
  * @param  huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_data     Pointer to data buffer (u8 or u16 data elements).
  * @param  size       Amount of data elements (u8 or u16) to be received.
  * @param  rx_mode    Reception mode
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @retval HAL_OK Receive started in IT mode.
  */
hal_status_t UART_Start_Receive_IT(hal_uart_handle_t *huart, uint8_t *p_data, uint32_t size,
                                   hal_uart_rx_modes_t rx_mode, uint32_t interrupts)
{
  uint32_t reg_temp;
  uint32_t nine_bits_data;
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  nine_bits_data = 0U;
  huart->p_rx_buff = p_data;
  huart->rx_xfer_size = size;
  huart->rx_xfer_count = size;
  huart->p_rx_isr = NULL;

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
    return HAL_ERROR;
  }

  /* If HalfDuplex mode selected, enable RE */
  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_EnableDirectionRx(p_uartx);
  }

  if (!IS_LPUART_INSTANCE(p_uartx))
  {
    if (LL_USART_IsEnabledRxTimeout(p_uartx) != 0U)
    {
      LL_USART_EnableIT_RTO(p_uartx);
    }
  }

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    nine_bits_data = 1U;
  }

  /* Computation of UART mask to apply to RDR register */
  if (UART_RDRMaskComputation(huart) != HAL_OK)
  {
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_EnableIT_ERROR(p_uartx);
  if ((huart->fifo_mode == HAL_UART_FIFO_MODE_ENABLED) && (size >= huart->nb_rx_data_to_process))
  {
    if (nine_bits_data != 0U)
    {
      huart->p_rx_isr = UART_RxISR_16BIT_FIFOEN;
    }
    else
    {
      huart->p_rx_isr = UART_RxISR_8BIT_FIFOEN;
    }
    if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
    {
      LL_USART_EnableIT_PE(p_uartx);
    }
    LL_USART_EnableIT_RXFT(p_uartx);
  }
  else
  {
    if (nine_bits_data != 0U)
    {
      huart->p_rx_isr = UART_RxISR_16BIT;
    }
    else
    {
      huart->p_rx_isr = UART_RxISR_8BIT;
    }
    if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
    {
      LL_USART_EnableIT_PE(p_uartx);
    }
    LL_USART_EnableIT_RXNE_RXFNE(p_uartx);
  }

  if (huart->rx_state != HAL_UART_RX_STATE_ACTIVE)
  {
    huart->p_rx_isr = NULL;
    return HAL_ERROR;
  }

  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

  if (rx_mode == HAL_UART_RX_TO_IDLE)
  {
    LL_USART_EnableIT_IDLE(p_uartx);
  }
  else if (rx_mode == HAL_UART_RX_TO_RTO)
  {
    LL_USART_EnableIT_RTO(p_uartx);
  }
  else if (rx_mode == HAL_UART_RX_TO_CHAR_MATCH)
  {
    LL_USART_EnableIT_CM(p_uartx);
  }
  else
  {
    /* do nothing */
  }

  if ((interrupts & HAL_UART_OPT_RX_IT_FIFO_FULL) == HAL_UART_OPT_RX_IT_FIFO_FULL)
  {
    LL_USART_EnableIT_RXFF(p_uartx);
  }
  if ((interrupts & HAL_UART_OPT_RX_IT_LIN_BREAK) == HAL_UART_OPT_RX_IT_LIN_BREAK)
  {
    LL_USART_EnableIT_LBD(p_uartx);
  }
  return HAL_OK;
}

/**
  * @brief  Start transmit operation in interrupt mode.
  * @param  huart      Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_data     Pointer to data buffer (u8 or u16 data elements).
  * @param  size       Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL UART API providing transmission in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @retval HAL_OK Transmit started in IT mode.
  */
hal_status_t UART_Start_Transmit_IT(hal_uart_handle_t *huart, const uint8_t *p_data, uint32_t size,
                                    uint32_t interrupts)
{
  uint32_t reg_temp;
  uint32_t nine_bits_data;
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  nine_bits_data = 0U;

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->tx_state = HAL_UART_TX_STATE_IDLE;
    return HAL_ERROR;
  }

  /* If HalfDuplex mode selected, disable RE to avoid overrun */
  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_SetTransferDirection(p_uartx, LL_USART_DIRECTION_TX);
  }

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    nine_bits_data = 1U;
  }

  huart->tx_xfer_size  = size;
  huart->tx_xfer_count = size;
  huart->p_tx_buff = p_data;
  huart->p_tx_isr = NULL;

  if (huart->fifo_mode == HAL_UART_FIFO_MODE_ENABLED)
  {
    if (nine_bits_data != 0U)
    {
      huart->p_tx_isr = UART_TxISR_16BIT_FIFOEN;
    }
    else
    {
      huart->p_tx_isr = UART_TxISR_8BIT_FIFOEN;
    }
    LL_USART_EnableIT_TXFT(p_uartx);
  }
  else
  {
    if (nine_bits_data != 0U)
    {
      huart->p_tx_isr = UART_TxISR_16BIT;
    }
    else
    {
      huart->p_tx_isr = UART_TxISR_8BIT;
    }
    LL_USART_EnableIT_TXE_TXFNF(p_uartx);
  }

  if ((interrupts & HAL_UART_OPT_TX_IT_FIFO_EMPTY) == HAL_UART_OPT_TX_IT_FIFO_EMPTY)
  {
    LL_USART_EnableIT_TXFE(p_uartx);
  }
  if ((interrupts & HAL_UART_OPT_TX_IT_CLEAR_TO_SEND) == HAL_UART_OPT_TX_IT_CLEAR_TO_SEND)
  {
    LL_USART_EnableIT_CTS(p_uartx);
  }
  return HAL_OK;
}

/**
  * @brief  Start receive operation in polling mode.
  * @param  huart          Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_data         Pointer to data buffer (u8 or u16 data elements).
  * @param  size_byte      Amount of data elements (u8 or u16) to be received.
  * @param  p_rx_size_byte Pointer on actual number of received data
  * @param  timeout_ms     Timeout value before exiting
  * @param  rx_mode        Reception mode
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @retval HAL_OK Receive finished in polling mode.
  * @retval HAL_TIMEOUT Timeout expired
  * @retval HAL_ERROR Error during instance enabling.
  */
hal_status_t UART_Start_Receive_Polling(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                        uint32_t *p_rx_size_byte, uint32_t timeout_ms,
                                        hal_uart_rx_modes_t rx_mode)
{
  uint32_t reg_temp;
  uint32_t flags_until_timeout;
  uint32_t tick_start;
  uint16_t uh_mask;
  uint16_t *p_data_16_bits;
  uint8_t  *p_data_8_bits;
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  flags_until_timeout = 0;

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
    return HAL_ERROR;
  }

  /* If HalfDuplex mode selected, enable RE */
  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_EnableDirectionRx(p_uartx);
  }

  reg_temp = LL_USART_READ_REG(p_uartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    p_data_16_bits = (uint16_t *)p_data;
    p_data_8_bits = NULL;
  }
  else
  {
    p_data_8_bits = (uint8_t *)p_data;
    p_data_16_bits = NULL;
  }

  huart->reception_type = rx_mode;

  huart->rx_xfer_size  = size_byte;
  huart->rx_xfer_count = size_byte;

  /* Computation of UART mask to apply to RDR register */
  if (UART_RDRMaskComputation(huart) != HAL_OK)
  {
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
    return HAL_ERROR;
  }
  uh_mask = huart->rdr_mask;

  if (huart->reception_type == HAL_UART_RX_STANDARD)
  {
    flags_until_timeout = LL_USART_ISR_RXNE_RXFNE;
  }
  else if (huart->reception_type == HAL_UART_RX_TO_IDLE)
  {
    flags_until_timeout = LL_USART_ISR_RXNE_RXFNE | LL_USART_ISR_IDLE;
  }
  else if (huart->reception_type == HAL_UART_RX_TO_RTO)
  {
    flags_until_timeout = LL_USART_ISR_RXNE_RXFNE | LL_USART_ISR_RTOF;
  }
  else if (huart->reception_type == HAL_UART_RX_TO_CHAR_MATCH)
  {
    flags_until_timeout = LL_USART_ISR_RXNE_RXFNE | LL_USART_ISR_CMF;
    LL_USART_ClearFlag_CM(p_uartx);
  }
  else
  {
    /* do nothing */
  }

  if (p_rx_size_byte != NULL)
  {
    *p_rx_size_byte = 0U;
  }

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  /* as long as data have to be received */
  while (huart->rx_xfer_count > 0U)
  {
    if (UART_WaitOnFlagUntilTimeout(huart, flags_until_timeout, 0U, tick_start, timeout_ms) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    if ((LL_USART_IsActiveFlag_IDLE(p_uartx) != 0U) && (rx_mode == HAL_UART_RX_TO_IDLE))
    {
      LL_USART_ClearFlag_IDLE(p_uartx);
      if (huart->rx_xfer_count != size_byte)
      {
        return HAL_OK;
      }
    }
    if ((LL_USART_IsActiveFlag_RTO(p_uartx) != 0U) && (rx_mode == HAL_UART_RX_TO_RTO))
    {
      LL_USART_ClearFlag_RTO(p_uartx);
      if (huart->rx_xfer_count != size_byte)
      {
        return HAL_OK;
      }
    }
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(p_uartx) != 0U)
    {
      if (p_data_8_bits == NULL)
      {
        *p_data_16_bits = LL_USART_ReceiveData9(p_uartx) & uh_mask;
        p_data_16_bits++;
      }
      else
      {
        *p_data_8_bits = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_uartx) & uh_mask);
        p_data_8_bits++;
      }
      if (p_rx_size_byte != NULL)
      {
        *p_rx_size_byte += 1U;
      }
      huart->rx_xfer_count--;
    }
    if ((LL_USART_IsActiveFlag_CM(p_uartx) != 0U) && (rx_mode == HAL_UART_RX_TO_CHAR_MATCH))
    {
      LL_USART_ClearFlag_CM(p_uartx);
      if (huart->rx_xfer_count != size_byte)
      {
        return HAL_OK;
      }
    }
  }
  huart->reception_type = HAL_UART_RX_STANDARD;
  return HAL_OK;
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief  Start receive operation in DMA mode.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @param  rx_mode Mode of the current reception (ToIdle, ToCM, UntilTimeout)
  * @note   This function could be called by all HAL UART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t UART_Start_Receive_DMA(hal_uart_handle_t *huart, uint8_t *p_data, uint32_t size,
                                    hal_uart_rx_modes_t rx_mode, uint32_t interrupts)
{
  USART_TypeDef *p_uartx;
  uint32_t interrupts_dma;

  p_uartx = UART_GET_INSTANCE(huart);
  huart->p_rx_buff = p_data;
  huart->rx_xfer_size = size;

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->rx_state = HAL_UART_RX_STATE_IDLE;
    return HAL_ERROR;
  }

  /* If HalfDuplex mode selected, enable RE */
  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_EnableDirectionRx(p_uartx);
  }

  if (!IS_LPUART_INSTANCE(p_uartx))
  {
    if (LL_USART_IsEnabledRxTimeout(p_uartx) != 0U)
    {
      LL_USART_EnableIT_RTO(p_uartx);
    }
  }
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (interrupts == HAL_UART_OPT_DMA_RX_IT_SILENT)
  {
    interrupts_dma = HAL_UART_OPT_DMA_RX_IT_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    interrupts_dma = (interrupts & HAL_UART_OPT_DMA_RX_IT_HT);
  }

  if (huart->hdma_rx != NULL)
  {
    huart->hdma_rx->p_xfer_cplt_cb = UART_DMAReceiveCplt;

    huart->hdma_rx->p_xfer_halfcplt_cb = UART_DMARxHalfCplt;

    huart->hdma_rx->p_xfer_error_cb = UART_DMAError;

    if (HAL_DMA_StartPeriphXfer_IT_Opt(huart->hdma_rx, (uint32_t)&p_uartx->RDR,
                                       (uint32_t)huart->p_rx_buff, size, interrupts_dma) != HAL_OK)
    {
      huart->rx_state = HAL_UART_RX_STATE_IDLE;
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
      huart->last_reception_error_codes |= HAL_UART_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
      return HAL_ERROR;
    }
  }

  LL_USART_EnableDMAReq_RX(p_uartx);
  LL_USART_EnableIT_ERROR(p_uartx);
  LL_USART_EnableIT_PE(p_uartx);

  if (rx_mode == HAL_UART_RX_TO_IDLE)
  {
    LL_USART_ClearFlag_IDLE(p_uartx);
    LL_USART_EnableIT_IDLE(p_uartx);
  }
  else if (rx_mode == HAL_UART_RX_TO_RTO)
  {
    LL_USART_ClearFlag_RTO(p_uartx);
    LL_USART_EnableIT_RTO(p_uartx);
  }
  else if (rx_mode == HAL_UART_RX_TO_CHAR_MATCH)
  {
    LL_USART_ClearFlag_CM(p_uartx);
    LL_USART_EnableIT_CM(p_uartx);
  }
  else
  {
    /* do nothing */
  }

  if (((interrupts & HAL_UART_OPT_RX_IT_LIN_BREAK) == HAL_UART_OPT_RX_IT_LIN_BREAK)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
      && (interrupts_dma != HAL_UART_OPT_DMA_RX_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
     )
  {
    LL_USART_EnableIT_LBD(p_uartx);
  }
  return HAL_OK;
}

/**
  * @brief  Start transmit operation in DMA mode.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL UART API providing transmission in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t UART_Start_Transmit_DMA(hal_uart_handle_t *huart, const uint8_t *p_data, uint32_t size,
                                     uint32_t interrupts)
{
  USART_TypeDef *p_uartx;
  uint32_t interrupts_dma;

  p_uartx = UART_GET_INSTANCE(huart);
  huart->p_tx_buff     = p_data;
  huart->tx_xfer_size  = size;
  huart->tx_xfer_count = size;

  if (UART_CheckEnabledState(huart) != HAL_OK)
  {
    huart->tx_state = HAL_UART_TX_STATE_IDLE;
    return HAL_ERROR;
  }

  /* If HalfDuplex mode selected, disable RE to avoid overrun */
  if (LL_USART_IsEnabledHalfDuplex(p_uartx) != 0U)
  {
    LL_USART_SetTransferDirection(p_uartx, LL_USART_DIRECTION_TX);
  }

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (interrupts == HAL_UART_OPT_DMA_TX_IT_SILENT)
  {
    interrupts_dma = HAL_UART_OPT_DMA_TX_IT_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    interrupts_dma = (interrupts & HAL_UART_OPT_DMA_TX_IT_HT);
  }

  if (huart->hdma_tx != NULL)
  {
    huart->hdma_tx->p_xfer_cplt_cb = UART_DMATransmitCplt;

    huart->hdma_tx->p_xfer_halfcplt_cb = UART_DMATxHalfCplt;

    huart->hdma_tx->p_xfer_error_cb = UART_DMAError;

    if (HAL_DMA_StartPeriphXfer_IT_Opt(huart->hdma_tx, (uint32_t)huart->p_tx_buff, (uint32_t)&p_uartx->TDR,
                                       size, interrupts_dma) != HAL_OK)
    {
      huart->tx_state = HAL_UART_TX_STATE_IDLE;
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
      huart->last_transmission_error_codes |= HAL_UART_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
      return HAL_ERROR;
    }
  }

  LL_USART_ClearFlag_TC(p_uartx);
  LL_USART_EnableDMAReq_TX(p_uartx);

  if (((interrupts & HAL_UART_OPT_TX_IT_CLEAR_TO_SEND) == HAL_UART_OPT_TX_IT_CLEAR_TO_SEND)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
      && (interrupts_dma != HAL_UART_OPT_DMA_TX_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
     )
  {
    LL_USART_EnableIT_CTS(p_uartx);
  }

  return HAL_OK;
}

/**
  * @brief End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
void UART_EndTxTransfer(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);
  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_TXEIE_TXFNFIE | LL_USART_CR1_TCIE));
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_TXFTIE));
  if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
  {
    LL_USART_DisableIT_CTS(p_uartx);
    LL_USART_ClearFlag_nCTS(p_uartx);
  }
  huart->tx_state = HAL_UART_TX_STATE_IDLE;
}
#endif /* USE_HAL_UART_DMA */

/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains the UART instance.
  */
static void UART_EndRxTransfer(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE));
  LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));

  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
  if (huart->reception_type == HAL_UART_RX_TO_IDLE)
  {
    LL_USART_DisableIT_IDLE(p_uartx);
    LL_USART_ClearFlag_IDLE(p_uartx);
  }
  if (huart->reception_type == HAL_UART_RX_TO_CHAR_MATCH)
  {
    LL_USART_DisableIT_CM(p_uartx);
    LL_USART_ClearFlag_CM(p_uartx);
  }
  if (huart->reception_type == HAL_UART_RX_TO_RTO)
  {
    LL_USART_DisableRxTimeout(p_uartx);
    LL_USART_DisableIT_RTO(p_uartx);
    LL_USART_ClearFlag_RTO(p_uartx);
  }
  if (LL_USART_IsEnabledIT_RXFF(p_uartx) != 0U)
  {
    LL_USART_DisableIT_RXFF(p_uartx);
  }
  if (LL_USART_IsEnabledIT_LBD(p_uartx) != 0U)
  {
    LL_USART_DisableIT_LBD(p_uartx);
    LL_USART_ClearFlag_LBD(p_uartx);
  }

  huart->reception_type = HAL_UART_RX_STANDARD;
  huart->p_rx_isr = NULL;
  huart->rx_state = HAL_UART_RX_STATE_IDLE;
}

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
/**
  * @brief DMA UART transmit process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMATransmitCplt(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    huart->tx_xfer_count = 0U;
    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
        in the UART CR3 register */
    LL_USART_DisableDMAReq_TX(p_uartx);
    LL_USART_EnableIT_TC(p_uartx);
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* DMA Circular mode */
  else
  {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    huart->p_tx_cplt_callback(huart);
#else
    HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
}

/**
  * @brief DMA UART transmit process half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMATxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_tx_half_cplt_callback(huart);
#else
  HAL_UART_TxHalfCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART receive process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMAReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    huart->rx_xfer_count = 0U;
    LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
    LL_USART_DisableIT_ERROR(p_uartx);
    LL_USART_DisableRxTimeout(p_uartx);
    /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
       in the UART CR3 register */
    LL_USART_DisableDMAReq_RX(p_uartx);

    if (LL_USART_IsEnabledIT_LBD(p_uartx) != 0U)
    {
      if (LL_USART_IsActiveFlag_LBD(p_uartx) != 0U)
      {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_lin_break_callback(huart);
#else
        HAL_UART_LINBreakCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
      LL_USART_DisableIT_LBD(p_uartx);
    }
    LL_USART_ClearFlag(p_uartx, LL_USART_ICR_LBDCF | LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);
    LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

    huart->rx_state = HAL_UART_RX_STATE_IDLE;
  }
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_rx_cplt_callback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#else
  HAL_UART_RxCpltCallback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART receive process half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMARxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_rx_half_cplt_callback(huart);
#else
  HAL_UART_RxHalfCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART communication error callback.
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMAError(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);
  const hal_uart_rx_state_t rx_state = huart->rx_state;
  const hal_uart_tx_state_t tx_state = huart->tx_state;

  /* Stop UART DMA Tx request if ongoing */
  if ((LL_USART_IsEnabledDMAReq_TX(p_uartx) != 0U) && (tx_state == HAL_UART_TX_STATE_ACTIVE))
  {
    huart->tx_xfer_count = 0U;
    UART_EndTxTransfer(huart);
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
    huart->last_transmission_error_codes |= HAL_UART_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
  }

  if ((LL_USART_IsEnabledDMAReq_RX(p_uartx) != 0U) && (rx_state == HAL_UART_RX_STATE_ACTIVE))
  {
    huart->rx_xfer_count = 0U;
    UART_EndRxTransfer(huart);
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
    huart->last_reception_error_codes |= HAL_UART_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
  }

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_error_callback(huart);
#else
  HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART communication abort callback, when initiated by HAL services on Error
  *        (To be called at end of DMA Abort procedure following error occurrence).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMAAbortOnError(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  huart->rx_xfer_count = 0U;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_error_callback(huart);
#else
  HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART Tx communication abort callback, when initiated by user
  *        (To be called at end of DMA Tx Abort procedure following user abort request).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  * @note  When this callback is executed, User Abort complete callback is called only if no
  *        Abort still ongoing for Rx DMA Handle.
  */
static void UART_DMATxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  /* Check if an Abort process is still ongoing */
  if (huart->hdma_rx != NULL)
  {
    if (huart->hdma_rx->global_state == HAL_DMA_STATE_ABORT)
    {
      return;
    }
  }
  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  huart->rx_xfer_count = 0U;
  huart->tx_xfer_count = 0U;
  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  if (huart->fifo_mode == HAL_UART_FIFO_MODE_ENABLED)
  {
    LL_USART_RequestTxDataFlush(p_uartx);
  }

  huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0U;
  huart->last_transmission_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_abort_cplt_callback(huart);
#else
  HAL_UART_AbortCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART Rx communication abort callback, when initiated by user
  *        (To be called at end of DMA Rx Abort procedure following user abort request).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  * @note  When this callback is executed, User Abort complete callback is called only if no
  *        Abort still ongoing for Tx DMA handle.
  */
static void UART_DMARxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  /* Check if an Abort process is still ongoing */
  if (huart->hdma_tx != NULL)
  {
    if (huart->hdma_tx->global_state == HAL_DMA_STATE_ABORT)
    {
      return;
    }
  }
  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  huart->rx_xfer_count = 0U;
  huart->tx_xfer_count = 0U;
  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0U;
  huart->last_transmission_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  LL_USART_RequestRxDataFlush(p_uartx);

  huart->tx_state = HAL_UART_TX_STATE_IDLE;
  huart->rx_state = HAL_UART_RX_STATE_IDLE;
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_abort_cplt_callback(huart);
#else
  HAL_UART_AbortCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART Tx communication abort callback, when initiated by user by a call to
  *        HAL_UART_AbortTransmit_IT API (Abort only Tx transfer)
  *        (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *        and leads to user Tx Abort Complete callback execution).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMATxOnlyAbortCallback(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  huart->tx_xfer_count = 0U;


  LL_USART_RequestTxDataFlush(p_uartx);

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_transmission_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  huart->tx_state = HAL_UART_TX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_abort_transmit_cplt_callback(huart);
#else
  HAL_UART_AbortTransmitCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART Rx communication abort callback, when initiated by user by a call to
  *        HAL_UART_AbortReceive_IT API (Abort only Rx transfer)
  *        (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *        and leads to user Rx Abort Complete callback execution).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMARxOnlyAbortCallback(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  huart->rx_xfer_count = 0U;
  huart->tx_xfer_count = 0U;
  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  huart->reception_type = HAL_UART_RX_STANDARD;

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  huart->last_reception_error_codes = 0U;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

  LL_USART_RequestRxDataFlush(p_uartx);
  huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_abort_receive_cplt_callback(huart);
#else
  HAL_UART_AbortReceiveCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief DMA UART Rx on communication abort callback, when initiated by
  *        IRQ handler API for reception to CHAR_MATCH, IDLE, RTO modes.
  *        (This callback is executed by the IRQ handler when a CM, RTO or IDLE IT is received and DMA transfer is
  *        not complete).
  * @param hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void UART_DMAAbortOnSuccessCallback(hal_dma_handle_t *hdma)
{
  hal_uart_handle_t *huart = (hal_uart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);
  hal_uart_rx_event_types_t rx_type = HAL_UART_RX_EVENT_TC;
  uint32_t rx_size = huart->rx_xfer_size;
  uint16_t nb_remaining_rx_data =
    (uint16_t)LL_DMA_GetBlkDataLength((DMA_Channel_TypeDef *)(uint32_t)huart->hdma_rx->instance);

  LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));

  if (huart->reception_type == HAL_UART_RX_TO_IDLE)
  {
    rx_type = HAL_UART_RX_EVENT_IDLE;
  }
  else if (huart->reception_type == HAL_UART_RX_TO_RTO)
  {
    rx_type = HAL_UART_RX_EVENT_RTO;
    LL_USART_DisableRxTimeout(p_uartx);
  }
  else if (huart->reception_type == HAL_UART_RX_TO_CHAR_MATCH)
  {
    rx_type = HAL_UART_RX_EVENT_CHAR_MATCH;
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_USART_IsEnabledIT_LBD(p_uartx) != 0U)
  {
    LL_USART_DisableIT_LBD(p_uartx);
  }
  if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
  {
    LL_USART_DisableIT_CTS(p_uartx);
  }
  LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF | LL_USART_ICR_LBDCF
                     | LL_USART_ICR_CTSCF);

  huart->reception_type = HAL_UART_RX_STANDARD;
  huart->rx_state = HAL_UART_RX_STATE_IDLE;
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_rx_cplt_callback(huart, (rx_size - nb_remaining_rx_data), rx_type);
#else
  HAL_UART_RxCpltCallback(huart, (rx_size - nb_remaining_rx_data), rx_type);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}
#endif /* USE_HAL_UART_DMA */

/**
  * @brief Tx interrupt handler for 7-bit or 8-bit data word length and FIFO mode is enabled.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Transmit_IT().
  */
static void UART_TxISR_8BIT_FIFOEN(hal_uart_handle_t *huart)
{
  uint16_t nb_tx_data;
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);

  for (nb_tx_data = huart->nb_tx_data_to_process ; nb_tx_data > 0U ; nb_tx_data--)
  {
    if (huart->tx_xfer_count == 0U)
    {
      LL_USART_DisableIT_TXFT(p_uartx);

      LL_USART_EnableIT_TC(p_uartx);

      if (LL_USART_IsEnabledIT_TXFE(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_TXFE(p_uartx) != 0U)
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_tx_fifo_empty_callback(huart);
#else
          HAL_UART_TxFifoEmptyCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_TXFE(p_uartx);
        LL_USART_ClearFlag_TXFE(p_uartx);
      }

      if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_CTS(p_uartx) != 0U)
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_clear_to_send_callback(huart);
#else
          HAL_UART_ClearToSendCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_CTS(p_uartx);
        LL_USART_ClearFlag_nCTS(p_uartx);
      }
      break;
    }
    else if (LL_USART_IsActiveFlag_TXE_TXFNF(p_uartx) != 0U)
    {
      LL_USART_TransmitData8(p_uartx, *huart->p_tx_buff);
      huart->p_tx_buff++;
      huart->tx_xfer_count--;
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
  * @brief TX interrupt handler for 9-bit data word length and FIFO mode is enabled.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Transmit_IT().
  */
static void UART_TxISR_16BIT_FIFOEN(hal_uart_handle_t *huart)
{
  const uint16_t *p_tmp;
  uint16_t nb_tx_data;
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);

  for (nb_tx_data = huart->nb_tx_data_to_process ; nb_tx_data > 0U ; nb_tx_data--)
  {
    if (huart->tx_xfer_count == 0U)
    {
      LL_USART_DisableIT_TXFT(p_uartx);

      LL_USART_EnableIT_TC(p_uartx);
      if (LL_USART_IsEnabledIT_TXFE(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_TXFE(p_uartx) != 0U)
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_tx_fifo_empty_callback(huart);
#else
          HAL_UART_TxFifoEmptyCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_TXFE(p_uartx);
        LL_USART_ClearFlag_TXFE(p_uartx);
      }

      if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_CTS(p_uartx) != 0U)
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_clear_to_send_callback(huart);
#else
          HAL_UART_ClearToSendCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_CTS(p_uartx);
        LL_USART_ClearFlag_nCTS(p_uartx);
      }

      break;
    }
    else if (LL_USART_IsActiveFlag_TXE_TXFNF(p_uartx) != 0U)
    {
      p_tmp = (const uint16_t *) huart->p_tx_buff;
      LL_USART_TransmitData9(p_uartx, *p_tmp);
      huart->p_tx_buff += 2U;
      huart->tx_xfer_count--;
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
  * @brief TX interrupt handler for 7-bit or 8-bit data word length.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Transmit_IT().
  */
static void UART_TxISR_8BIT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->tx_xfer_count == 0U)
  {
    LL_USART_DisableIT_TXE_TXFNF(p_uartx);

    LL_USART_EnableIT_TC(p_uartx);

    if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
    {
      if (LL_USART_IsActiveFlag_CTS(p_uartx) != 0U)
      {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_clear_to_send_callback(huart);
#else
        HAL_UART_ClearToSendCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
      LL_USART_DisableIT_CTS(p_uartx);
      LL_USART_ClearFlag_nCTS(p_uartx);
    }
  }
  else
  {
    LL_USART_TransmitData8(p_uartx, *huart->p_tx_buff);
    huart->p_tx_buff++;
    huart->tx_xfer_count--;
  }
}

/**
  * @brief TX interrupt handler for 9-bit data word length.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Transmit_IT().
  */
static void UART_TxISR_16BIT(hal_uart_handle_t *huart)
{
  const uint16_t *p_tmp;
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->tx_xfer_count == 0U)
  {
    LL_USART_DisableIT_TXE_TXFNF(p_uartx);

    LL_USART_EnableIT_TC(p_uartx);

    if (LL_USART_IsEnabledIT_CTS(p_uartx) != 0U)
    {
      if (LL_USART_IsActiveFlag_CTS(p_uartx) != 0U)
      {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_clear_to_send_callback(huart);
#else
        HAL_UART_ClearToSendCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
      LL_USART_DisableIT_CTS(p_uartx);
      LL_USART_ClearFlag_nCTS(p_uartx);
    }
  }
  else
  {
    p_tmp = (const uint16_t *) huart->p_tx_buff;
    LL_USART_TransmitData9(p_uartx, *p_tmp);
    huart->p_tx_buff += 2U;
    huart->tx_xfer_count--;
  }
}

/**
  * @brief  Complete transmission in non-blocking mode.
  * @param  huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  */
static void UART_EndTransmit_IT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  LL_USART_DisableIT_TC(p_uartx);

  huart->p_tx_isr = NULL;

  huart->tx_state = HAL_UART_TX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  huart->p_tx_cplt_callback(huart);
#else
  HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief RX interrupt handler for 7-bit or 8-bit data word length.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  */
static void UART_RxISR_8BIT(hal_uart_handle_t *huart)
{
  uint16_t uh_mask = huart->rdr_mask;
  USART_TypeDef *p_uartx;

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    *huart->p_rx_buff = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_uartx) & uh_mask);
    huart->p_rx_buff++;
    huart->rx_xfer_count--;
    if (huart->rx_xfer_count == 0U)
    {
      /* Disable the UART Rx interrupts */
      LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE
                                       | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));
      LL_USART_DisableIT_ERROR(p_uartx);
      LL_USART_DisableRxTimeout(p_uartx);

      huart->p_rx_isr = NULL;

      LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

      if (LL_USART_IsEnabledIT_LBD(p_uartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_LBD(p_uartx) != 0U)
        {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_lin_break_callback(huart);
#else
          HAL_UART_LINBreakCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_LBD(p_uartx);
        LL_USART_ClearFlag_LBD(p_uartx);
      }

      huart->reception_type = HAL_UART_RX_STANDARD;
      huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
      huart->p_rx_cplt_callback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#else
      HAL_UART_RxCpltCallback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_uartx);
  }
}

/**
  * @brief RX interrupt handler for 9-bit data word length.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Receive_IT().
  */
static void UART_RxISR_16BIT(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint16_t uh_mask = huart->rdr_mask;
  uint16_t *p_tmp;
  uint16_t  uh_data;

  p_uartx = UART_GET_INSTANCE(huart);

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    uh_data = LL_USART_ReceiveData9(p_uartx);
    p_tmp = (uint16_t *)huart->p_rx_buff;
    *p_tmp = (uint16_t)(uh_data & uh_mask);
    huart->p_rx_buff += 2U;
    huart->rx_xfer_count--;
    if (huart->rx_xfer_count == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupts */
      LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE
                                       | LL_USART_CR1_RTOIE));
      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      LL_USART_DisableIT_ERROR(p_uartx);

      LL_USART_DisableRxTimeout(p_uartx);

      huart->p_rx_isr = NULL;

      LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF);

      huart->reception_type = HAL_UART_RX_STANDARD;
      huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
      huart->p_rx_cplt_callback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#else
      HAL_UART_RxCpltCallback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_uartx);
  }
}

/**
  * @brief RX interrupt handler for 7-bit or 8-bit data word length and FIFO mode is enabled.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Receive_IT().
  */
static void UART_RxISR_8BIT_FIFOEN(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t isr_flags;
  uint32_t cr1_its;
  uint32_t cr3_its;
  uint32_t error_code;
  uint16_t uh_mask = huart->rdr_mask;
  uint16_t nb_rx_data;
  uint16_t rx_data_count;

  p_uartx = UART_GET_INSTANCE(huart);
  isr_flags = LL_USART_READ_REG(p_uartx, ISR);
  cr1_its   = LL_USART_READ_REG(p_uartx, CR1);
  cr3_its   = LL_USART_READ_REG(p_uartx, CR3);
  error_code = 0U;

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    nb_rx_data = huart->nb_rx_data_to_process;
    while ((nb_rx_data > 0U) && ((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U))
    {

      /* Char Match interrupt occurred --------------------------------------*/
      if (((isr_flags & LL_USART_ISR_CMF) != 0U) && ((cr1_its & LL_USART_CR1_CMIE) != 0U))
      {
        LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_RXNEIE_RXFNEIE));
        LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_RXFTIE));
        *huart->p_rx_buff = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_uartx) & uh_mask);
        huart->p_rx_buff++;
        huart->rx_xfer_count--;
        return;
      }
      else
      {
        *huart->p_rx_buff = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_uartx) & uh_mask);
        huart->p_rx_buff++;
        huart->rx_xfer_count--;
      }

      isr_flags = LL_USART_READ_REG(p_uartx, ISR);

      /* If some non-blocking errors occurred */
      if ((isr_flags & (LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_NE)) != 0U)
      {
        /* UART parity error interrupt occurred -------------------------------------*/
        if (((isr_flags & LL_USART_ISR_PE) != 0U) && ((cr1_its & LL_USART_CR1_PEIE) != 0U))
        {
          LL_USART_ClearFlag_PE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_PE;
        }

        /* UART frame error interrupt occurred --------------------------------------*/
        if (((isr_flags & LL_USART_ISR_FE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
        {
          LL_USART_ClearFlag_FE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_FE;
        }

        /* UART noise error interrupt occurred --------------------------------------*/
        if (((isr_flags & LL_USART_ISR_NE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
        {
          LL_USART_ClearFlag_NE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_NE;
        }
        /* Call UART Error callback function if needed ----------------------------*/
        if (error_code != HAL_UART_RECEIVE_ERROR_NONE)
        {
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
          huart->last_reception_error_codes = error_code;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
          /* Non-blocking error: transfer can continue.
             Error is notified to the user through the error callback. */
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_error_callback(huart);
#else
          HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
      }

      if (huart->rx_xfer_count == 0U)
      {
        LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE
                                         | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error)
          and RX FIFO Threshold interrupt */
        LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));

        LL_USART_DisableRxTimeout(p_uartx);

        huart->p_rx_isr = NULL;

        LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF | LL_USART_ICR_CMCF);

        if (LL_USART_IsEnabledIT_RXFF(p_uartx) != 0U)
        {
          if (LL_USART_IsActiveFlag_RXFF(p_uartx) != 0U)
          {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            huart->p_rx_fifo_full_callback(huart);
#else
            HAL_UART_RxFifoFullCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
          }
          LL_USART_DisableIT_RXFF(p_uartx);
        }

        if (LL_USART_IsEnabledIT_LBD(p_uartx) != 0U)
        {
          if (LL_USART_IsActiveFlag_LBD(p_uartx) != 0U)
          {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            huart->p_lin_break_callback(huart);
#else
            HAL_UART_LINBreakCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
          }
          LL_USART_DisableIT_LBD(p_uartx);
          LL_USART_ClearFlag_LBD(p_uartx);
        }

        huart->reception_type = HAL_UART_RX_STANDARD;
        huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_rx_cplt_callback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#else
        HAL_UART_RxCpltCallback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
    }

    /* When remaining number of bytes to receive is less than the RX FIFO
    threshold, next incoming frames are processed as if FIFO mode was
    disabled (i.e. one interrupt per received frame).
    */
    rx_data_count = (uint16_t)huart->rx_xfer_count;
    if ((rx_data_count != 0U) && (rx_data_count < huart->nb_rx_data_to_process))
    {
      LL_USART_DisableIT_RXFT(p_uartx);

      huart->p_rx_isr = UART_RxISR_8BIT;

      LL_USART_EnableIT_RXNE_RXFNE(p_uartx);
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_uartx);
  }
}

/**
  * @brief RX interrupt handler for 9-bit data word length and FIFO mode is enabled.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @note  Function is called under interruption only, once
  *        interruptions have been enabled by HAL_UART_Receive_IT().
  */
static void UART_RxISR_16BIT_FIFOEN(hal_uart_handle_t *huart)
{
  USART_TypeDef *p_uartx;
  uint32_t isr_flags;
  uint32_t cr1_its;
  uint32_t cr3_its;
  uint32_t error_code;
  uint16_t uh_mask = huart->rdr_mask;
  uint16_t nb_rx_data;
  uint16_t rx_data_count;
  uint16_t *p_tmp;
  uint16_t  uh_data;

  p_uartx = UART_GET_INSTANCE(huart);
  isr_flags = LL_USART_READ_REG(p_uartx, ISR);
  cr1_its   = LL_USART_READ_REG(p_uartx, CR1);
  cr3_its   = LL_USART_READ_REG(p_uartx, CR3);
  error_code = 0U;

  if (huart->rx_state == HAL_UART_RX_STATE_ACTIVE)
  {
    nb_rx_data = huart->nb_rx_data_to_process;
    while ((nb_rx_data > 0U) && ((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U))
    {
      uh_data = LL_USART_ReceiveData9(p_uartx);
      p_tmp = (uint16_t *)huart->p_rx_buff;
      *p_tmp = (uint16_t)(uh_data & uh_mask);
      huart->p_rx_buff += 2U;
      huart->rx_xfer_count--;

      isr_flags = LL_USART_READ_REG(p_uartx, ISR);

      /* If some non-blocking errors occurred */
      if ((isr_flags & (LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_NE)) != 0U)
      {
        /* UART parity error interrupt occurred -------------------------------------*/
        if (((isr_flags & LL_USART_ISR_PE) != 0U) && ((cr1_its & LL_USART_CR1_PEIE) != 0U))
        {
          LL_USART_ClearFlag_PE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_PE;
        }

        /* UART frame error interrupt occurred --------------------------------------*/
        if (((isr_flags & LL_USART_ISR_FE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
        {
          LL_USART_ClearFlag_FE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_FE;
        }

        /* UART noise error interrupt occurred --------------------------------------*/
        if (((isr_flags & LL_USART_ISR_NE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
        {
          LL_USART_ClearFlag_NE(p_uartx);
          error_code |= HAL_UART_RECEIVE_ERROR_NE;
        }

        /* Call UART Error callback function if needed ----------------------------*/
        if (error_code != HAL_UART_RECEIVE_ERROR_NONE)
        {
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
          huart->last_reception_error_codes = error_code;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
          /* Non-blocking error: transfer can continue.
             Error is notified to the user through the error callback. */
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          huart->p_error_callback(huart);
#else
          HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
      }
      if (huart->rx_xfer_count == 0U)
      {
        LL_USART_DisableIT_CR1(p_uartx, (LL_USART_CR1_PEIE | LL_USART_CR1_IDLEIE
                                         | LL_USART_CR1_RTOIE | LL_USART_CR1_CMIE));

        LL_USART_DisableRxTimeout(p_uartx);

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error)
         and RX FIFO Threshold interrupt */
        LL_USART_DisableIT_CR3(p_uartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));

        huart->p_rx_isr = NULL;

        LL_USART_ClearFlag(p_uartx, LL_USART_ICR_IDLECF | LL_USART_ICR_RTOCF);
        if (LL_USART_IsEnabledIT_RXFF(p_uartx) != 0U)
        {
          if (LL_USART_IsActiveFlag_RXFF(p_uartx) != 0U)
          {
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            huart->p_rx_fifo_full_callback(huart);
#else
            HAL_UART_RxFifoFullCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
          }
          LL_USART_DisableIT_RXFF(p_uartx);
        }
        huart->reception_type = HAL_UART_RX_STANDARD;
        huart->rx_state = HAL_UART_RX_STATE_IDLE;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->p_rx_cplt_callback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#else
        HAL_UART_RxCpltCallback(huart, huart->rx_xfer_size, HAL_UART_RX_EVENT_TC);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
    }
    /* When remaining number of bytes to receive is less than the RX FIFO
    threshold, next incoming frames are processed as if FIFO mode was
    disabled (i.e. one interrupt per received frame).
    */
    rx_data_count = (uint16_t)huart->rx_xfer_count;
    if ((rx_data_count != 0U) && (rx_data_count < huart->nb_rx_data_to_process))
    {
      LL_USART_DisableIT_RXFT(p_uartx);

      huart->p_rx_isr = UART_RxISR_16BIT;

      LL_USART_EnableIT_RXNE_RXFNE(p_uartx);
    }
  }
  else
  {
    LL_USART_RequestRxDataFlush(p_uartx);
  }
}

/**
  * @brief Calculate FIFO data to process depending on threshold.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  */
static void UART_SetNbDataToProcess(hal_uart_handle_t *huart)
{
  uint8_t rx_fifo_depth;
  uint8_t tx_fifo_depth;
  uint8_t rx_fifo_threshold;
  uint8_t tx_fifo_threshold;
  static const uint8_t numerator[] = {1U, 1U, 1U, 3U, 7U, 1U, 0U, 0U};
  static const uint8_t denominator[] = {8U, 4U, 2U, 4U, 8U, 1U, 1U, 1U};
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);

  if (huart->fifo_mode == HAL_UART_FIFO_MODE_DISABLED)
  {
    huart->nb_tx_data_to_process = 1U;
    huart->nb_rx_data_to_process = 1U;
  }
  else
  {
    rx_fifo_depth = UART_RX_FIFO_DEPTH;
    tx_fifo_depth = UART_TX_FIFO_DEPTH;
    rx_fifo_threshold = (uint8_t)LL_USART_GetRXFIFOThreshold(p_uartx);
    tx_fifo_threshold = (uint8_t)LL_USART_GetTXFIFOThreshold(p_uartx);
    huart->nb_tx_data_to_process = ((uint16_t)tx_fifo_depth * numerator[tx_fifo_threshold]) /
                                   (uint16_t)denominator[tx_fifo_threshold];
    huart->nb_rx_data_to_process = ((uint16_t)rx_fifo_depth * numerator[rx_fifo_threshold]) /
                                   (uint16_t)denominator[rx_fifo_threshold];
  }
}

#if defined(USE_ASSERT_DBG_PARAM)
/**
  * @brief Calculate and check baud rate validity.
  * @param instance_clock_freq Clock frequency of the lpuart instance used
  * @param instance_clock_prescaler Clock prescaler of the lpuart instance used
  * @param baud_rate Baud rate to be tested
  * @retval HAL_OK baud rate value is valid
  * @retval HAL_ERROR baud rate value is invalid
  */
hal_status_t UART_Check_lpuart_baudrate_validity(uint32_t instance_clock_freq, uint32_t instance_clock_prescaler,
                                                 uint32_t baud_rate)
{
  uint32_t lpuart_clock_freq_div = (instance_clock_freq / UARTPrescTable[instance_clock_prescaler]);
  if ((lpuart_clock_freq_div > (3U * baud_rate)) && (lpuart_clock_freq_div < (4096U * baud_rate)))
  {
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief Calculate and check baud rate validity.
  * @param instance_clock_freq Clock frequency of the uart instance used
  * @param instance_clock_prescaler Clock prescaler of the uart instance used
  * @param baud_rate Baud rate to be tested
  * @param oversampling Oversampling of the uart instance used
  * @retval HAL_OK baud rate value is valid
  * @retval HAL_ERROR baud rate value is invalid
  */
hal_status_t UART_Check_uart_baudrate_validity(uint32_t instance_clock_freq, uint32_t instance_clock_prescaler,
                                               uint32_t baud_rate, hal_uart_oversampling_t oversampling)
{
  uint32_t div_temp;
  if (oversampling == HAL_UART_OVERSAMPLING_8)
  {
    div_temp = LL_USART_DIV_SAMPLING8(instance_clock_freq, instance_clock_prescaler, baud_rate);
    if ((div_temp >= UART_BRR_MIN) && (div_temp <= UART_BRR_MAX))
    {
      return HAL_OK;
    }
  }
  else
  {
    div_temp = LL_USART_DIV_SAMPLING16(instance_clock_freq, instance_clock_prescaler, baud_rate);
    if ((div_temp >= UART_BRR_MIN) && (div_temp <= UART_BRR_MAX))
    {
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}
#endif /* USE_ASSERT_DBG_PARAM */

/**
  * @brief Set parity on the current character.
  * @param huart Pointer to a \ref hal_uart_handle_t structure which contains a UART instance.
  * @param p_character Character to set the parity on.
  */
void UART_Parity_Computation(hal_uart_handle_t *huart, uint8_t *p_character)
{
  uint8_t mask = 0x1;
  uint8_t c_length;
  uint8_t ones = 0;
  uint8_t i;
  USART_TypeDef *p_uartx = UART_GET_INSTANCE(huart);
  hal_uart_parity_t parity = (hal_uart_parity_t)LL_USART_GetParity(p_uartx);
  hal_uart_word_length_t length = (hal_uart_word_length_t)LL_USART_GetDataWidth(p_uartx);

  ASSERT_DBG_PARAM(length != HAL_UART_WORD_LENGTH_9_BIT);

  if (length == HAL_UART_WORD_LENGTH_7_BIT)
  {
    c_length = 7U;
  }
  else
  {
    c_length = 8U;
  }
  if (parity != HAL_UART_PARITY_NONE)
  {
    i = c_length;
    while (i != 0U)
    {
      if ((*p_character & mask) == mask)
      {
        ones ++;
      }
      mask = mask << 1U;
      i --;
    }
    if (((parity == HAL_UART_PARITY_EVEN) && ((ones % 2U) != 0U))
        || ((parity == HAL_UART_PARITY_ODD) && ((ones % 2U) == 0U)))
    {
      *p_character = (*p_character ^ (1U << (c_length - 1U)));
    }
  }
}
/**
  * @}
  */
#endif /* USE_HAL_UART_MODULE */
/**
  * @}
  */
#endif /* USART1 || USART2 || USART3 || UART4 || UART5 || USART6 || UART7 || LPUART1 */
/**
  * @}
  */
