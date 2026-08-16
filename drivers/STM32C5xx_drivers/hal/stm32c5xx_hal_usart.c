/**
  ******************************************************************************
  * @file    stm32c5xx_hal_usart.c
  * @brief   USART HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Synchronous/Asynchronous Receiver Transmitter
  *          Peripheral (USART).
  *           + Initialization and deinitialization functions
  *           + I/O operation functions
  *           + Peripheral control functions
  *           + Peripheral state and error functions.
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
#if defined(USE_HAL_USART_MODULE) && (USE_HAL_USART_MODULE == 1)
/** @addtogroup USART
  * @{
  */

/** @defgroup USART_Introduction USART Introduction
  * @{
  - The USART hardware abstraction layer provides a set of APIs to interface with the STM32 **USART** (Universal
    Synchronous/Asynchronous Receiver Transmitter) peripheral using the SPI communication protocol.

  - It simplifies the configuration, initialization, and management of synchronous USART communication by supporting
    various modes such as polling, interrupt, and DMA for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series.
  * @}
  */

/** @defgroup USART_How_To_Use USART How To Use
  * @{

  # How to use the USART HAL module driver

The USART Synchronous SPI HAL driver in synchronous SPI master/slave mode can be used as follows:

1. Declare a hal_usart_handle_t handle structure, for example:
   hal_usart_handle_t husart;

2. Configure the low-level hardware (GPIO, CLOCK, NVIC, etc.):
    - Enable the USART interface clock if you have not set USE_HAL_USART_CLK_ENABLE_MODEL to HAL_CLK_ENABLE_PERIPH_ONLY
      or HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM (in those cases HAL_USART_Init() will enable the clock).
    - USART pins configuration:
        - Enable the clock for the USART GPIOs
        - Configure these USART pins as an alternate function.
    - NVIC configuration when using interrupt processing (HAL_USART_Transmit_IT(), HAL_USART_Receive_IT(),
      HAL_USART_TransmitReceive_IT() and their _Opt equivalent APIs):
        - Configure the USART interrupt priority.
        - Enable the NVIC USART IRQ channel.
    - DMA configuration when using DMA processing (HAL_USART_Transmit_DMA(), HAL_USART_Receive_DMA(),
      HAL_USART_TransmitReceive_DMA() and their _Opt equivalent APIs):
        - Declare a DMA handle structure for the Tx or Rx channel.
        - Enable the DMAx interface clock.
        - Configure the declared DMA handle structure with the required Tx or Rx parameters.
        - Associate the initialized DMA handle to the USART handle with HAL_USART_SetTxDMA() or HAL_USART_SetRxDMA().
        - For each DMA channel (Tx and Rx), configure the corresponding NVIC line priority and enable it.
      @warning In DMA configuration, also enable the USART IRQ to complete the DMA transfer.

3. Initialize the USART driver by selecting a USART instance and calling HAL_USART_Init().
  Depending on USE_HAL_USART_CLK_ENABLE_MODEL, HAL_USART_Init() can enable the USART clock.
   For example:
        HAL_USART_Init(&husart, HAL_USART1);

4. Declare a hal_usart_config_t structure, fill it, and call HAL_USART_SetConfig(). For example:
        hal_usart_config_t my_config;

   In the configuration structure,
    program the baud rate, Word Length, Stop Bit, Parity, Prescaler value, Device Mode,
    Direction (Receiver/Transmitter), Clock Polarity, Clock Phase, and Last Bit Clock Pulse.

   Apply the configuration by calling HAL_USART_SetConfig(&husart, &my_config).

  If needed, configure and enable or disable the USART to wake up the MCU from stop mode. Use the UART APIs
  HAL_UART_SetStopModeWakeUpAddress(), HAL_UART_EnableStopMode() and HAL_UART_DisableStopMode() by casting the
  USART handle to UART type hal_usart_handle_t.

5. Callback registration
  When the compilation define USE_HAL_USART_REGISTER_CALLBACKS is set to 1U, configure the driver callbacks
  dynamically using a user-defined method:

  Callback name               | Default value                       | Callback registration function
  ----------------------------| ----------------------------------- | ---------------------------
  TxHalfCpltCallback          | HAL_USART_TxHalfCpltCallback()      | HAL_USART_RegisterTxHalfCpltCallback()
  TxCpltCallback              | HAL_USART_TxCpltCallback()          | HAL_USART_RegisterTxCpltCallback()
  RxHalfCpltCallback          | HAL_USART_RxHalfCpltCallback()      | HAL_USART_RegisterRxHalfCpltCallback()
  RxCpltCallback              | HAL_USART_RxCpltCallback()          | HAL_USART_RegisterRxCpltCallback()
  ErrorCallback               | HAL_USART_ErrorCallback()           | HAL_USART_RegisterErrorCallback()
  AbortCpltCallback           | HAL_USART_AbortCpltCallback()       | HAL_USART_RegisterAbortCpltCallback()
  TxRxCpltCallback            | HAL_USART_TxRxCpltCallback()        | HAL_USART_RegisterTxRxCpltCallback()
  RxFifoFullCallback          | HAL_USART_RxFifoFullCallback()      | HAL_USART_RegisterRxFifoFullCallback()
  TxFifoEmptyCallback         | HAL_USART_TxFifoEmptyCallback()     | HAL_USART_RegisterTxFifoEmptyCallback()

  If you need to unregister a callback, register the default callback via the registration function.

  By default, after the HAL_USART_Init() and when the state is HAL_USART_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Register callbacks when handle global_state is HAL_USART_STATE_INIT or HAL_USART_STATE_IDLE.

  When the compilation define USE_HAL_USART_REGISTER_CALLBACKS is set to 0U or not defined, the callback registration
  feature is not available and weak callbacks are used, as listed by the default values in the table above.

6. Acquire/Release the HAL USART handle
  - When the compilation flag USE_HAL_MUTEX is set to 1, a multithreaded user application can acquire the
    USART HAL handle to execute a transmit or a receive process or a sequence of transmit/receive.
    When the process or sequence ends, release the USART HAL handle.
  - The HAL acquire/release functions are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
     - Use HAL_USART_AcquireBus() to acquire/take the HAL USART handle.
     - Use HAL_USART_ReleaseBus() to release the HAL USART handle.

  - When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_USART_AcquireBus() and
    HAL_USART_ReleaseBus() are not available.

  */

/**
  * @}
  */

/** @defgroup USART_Configuration_Table USART Configuration Table
  * @{

## Configuration inside the USART driver:

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_USART_MODULE             |         1         | Enable HAL USART driver module
USE_HAL_USART_REGISTER_CALLBACKS |         0         | Allow the user to define a callback
USE_HAL_USART_DMA                |         1         | Enable DMA code in USART
USE_HAL_CHECK_PARAM              |         0         | Enable runtime parameter checks
USE_HAL_USART_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE      |         0         | Enable atomicity of process state checks
USE_HAL_MUTEX                    |         0         | Enable semaphore creation for the OS
USE_HAL_USART_USER_DATA          |         0         | Add user data inside the HAL USART handle
USE_HAL_USART_GET_LAST_ERRORS    |         0         | Enable retrieval of last process error codes
USE_HAL_USART_FIFO               |         0         | Enable FIFO code in HAL USART

Software configuration defined in preprocessor environment:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM             |    Not defined    | Enable parameter checks for HAL and LL
USE_ASSERT_DBG_STATE             |    Not defined    | Enable state checks for HAL

  */
/**
  * @}
  */

/* Private constants -----------------------------------------------------------*/
/** @defgroup USART_Private_Constants USART Private Constants
  * @{
  */
/*! USART transmitted dummy data                          */
#define USART_DUMMY_DATA                     (0xF)

/*! USART TX or RX enable acknowledge timeout value       */
#define USART_ENABLE_TIMEOUT_MS              100U

/*! USART BRR minimum authorized value                    */
#define USART_BRR_MIN                        0x10U

/*! USART BRR maximum authorized value                    */
#define USART_BRR_MAX                        0xFFFFU

/*! USART RX FIFO depth                                   */
#define RX_FIFO_DEPTH                        8U

/*! USART TX FIFO depth                                   */
#define TX_FIFO_DEPTH                        8U

/*! USART mask for 9-bit data length used for RDR reading */
#define USART_RDR_MASK_9_BIT                 0x01FFU

/*! USART mask for 8-bit data length used for RDR reading */
#define USART_RDR_MASK_8_BIT                 0x00FFU

/*! USART mask for 7-bit data length used for RDR reading */
#define USART_RDR_MASK_7_BIT                 0x007FU

/*! USART mask for 6-bit data length used for RDR reading */
#define USART_RDR_MASK_6_BIT                 0x003FU

/**
  * @}
  */
/* Private macros -----------------------------------------------------------*/
/** @defgroup USART_Private_Macros USART Private Macros
  * @{
  */

/** @brief  Check USART baud rate.
  * @param  baud_rate Baud rate specified by the user.
  *         The maximum baud rate is derived from the maximum clock on C5 (i.e. 144 MHz)
  *         divided by the smallest oversampling used on the USART (i.e. 8).
  * @retval 1U (baud_rate is valid) or 0U (baud_rate is invalid)
  */
#define IS_USART_BAUD_RATE(baud_rate) ((baud_rate) <= 18000000U     \
                                       && (baud_rate) > 0U)

/**
  * @brief Ensure that the number of transferred data is valid.
  * @param data_size USART TX data size.
  * @retval 1U (data_size is valid) or 0U (data_size is invalid)
  */
#define IS_USART_TX_DATA_SIZE(data_size)  ((data_size) <= 0xFFFFU)

/**
  * @brief Ensure that USART frame length is valid.
  * @param length USART frame length.
  * @retval 1U (length is valid) or 0U (length is invalid)
  */
#define IS_USART_WORD_LENGTH(length) (((length) == HAL_USART_WORD_LENGTH_7_BIT)    \
                                      || ((length) == HAL_USART_WORD_LENGTH_8_BIT) \
                                      || ((length) == HAL_USART_WORD_LENGTH_9_BIT))

/**
  * @brief Ensure that USART frame number of stop bits is valid.
  * @param stopbits USART frame number of stop bits.
  * @retval 1U (stopbits is valid) or 0U (stopbits is invalid)
  */
#define IS_USART_STOP_BITS(stopbits) (((stopbits) == HAL_USART_STOP_BIT_0_5)    \
                                      || ((stopbits) == HAL_USART_STOP_BIT_1)   \
                                      || ((stopbits) == HAL_USART_STOP_BIT_1_5) \
                                      || ((stopbits) == HAL_USART_STOP_BIT_2))
/**
  * @brief Ensure that USART frame parity is valid.
  * @param parity USART frame parity.
  * @retval 1U (parity is valid) or 0U (parity is invalid)
  */
#define IS_USART_PARITY(parity) (((parity) == HAL_USART_PARITY_NONE)     \
                                 || ((parity) == HAL_USART_PARITY_EVEN)  \
                                 || ((parity) == HAL_USART_PARITY_ODD))

/**
  * @brief Ensure that USART direction is valid.
  * @param direction USART direction.
  * @retval 1U (direction is valid) or 0U (direction is invalid)
  */
#define IS_USART_DIRECTION(direction) (((direction) == HAL_USART_DIRECTION_RX)    \
                                       || ((direction) == HAL_USART_DIRECTION_TX) \
                                       || ((direction) == HAL_USART_DIRECTION_TX_RX))

/**
  * @brief Ensure that USART Prescaler is valid.
  * @param clock_prescaler USART Prescaler value.
  * @retval 1U (clock_prescaler is valid) or 0U (clock_prescaler is invalid)
  */
#define IS_USART_PRESCALER(clock_prescaler) (((clock_prescaler) == HAL_USART_PRESCALER_DIV1)      \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV2)   \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV4)   \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV6)   \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV8)   \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV10)  \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV12)  \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV16)  \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV32)  \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV64)  \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV128) \
                                             || ((clock_prescaler) == HAL_USART_PRESCALER_DIV256))
/**
  * @brief Ensure that USART Clock Polarity is valid.
  * @param clock_polarity USART Clock Polarity value.
  * @retval 1U (clock_polarity is valid) or 0U (clock_polarity is invalid)
  */
#define IS_USART_CLOCK_POLARITY(clock_polarity) (((clock_polarity) == HAL_USART_CLOCK_POLARITY_LOW)  \
                                                 || ((clock_polarity) == HAL_USART_CLOCK_POLARITY_HIGH))
/**
  * @brief Ensure that USART Clock Phase is valid.
  * @param clock_phase USART Clock Phase value.
  * @retval 1U (clock_phase is valid) or 0U (clock_phase is invalid)
  */
#define IS_USART_CLOCK_PHASE(clock_phase) (((clock_phase) == HAL_USART_CLOCK_PHASE_1_EDGE)  \
                                           || ((clock_phase) == HAL_USART_CLOCK_PHASE_2_EDGE))
/**
  * @brief Ensure that USART Last Bit Clock Pulse is valid.
  * @param clock_last_bit USART Last Bit Clock Pulse value.
  * @retval 1U (clock_last_bit is valid) or 0U (clock_last_bit is invalid)
  */
#define IS_USART_CLOCK_LAST_BIT(clock_last_bit) (((clock_last_bit) == HAL_USART_CLOCK_LAST_BIT_DISABLED)  \
                                                 || ((clock_last_bit) == HAL_USART_CLOCK_LAST_BIT_ENABLED))

/**
  * @brief Ensure that USART mode is valid.
  * @param mode USART mode value.
  * @retval 1U (mode is valid) or 0U (mode is invalid)
  */
#define IS_USART_MODE(mode) (((mode) == HAL_USART_MODE_MASTER) \
                             || ((mode) == HAL_USART_MODE_SLAVE))

/**
  * @brief Ensure that USART Slave Select configuration is valid.
  * @param ss_config USART Slave Select configuration value.
  * @retval 1U (ss_config is valid) or 0U (ss_config is invalid)
  */
#define IS_USART_SLAVE_SELECT_CONFIG(ss_config) (((ss_config) == HAL_USART_SLAVE_SELECT_PIN_IGNORED)  \
                                                 || ((ss_config) == HAL_USART_SLAVE_SELECT_PIN_USED))

/**
  * @brief Ensure that USART request parameter is valid.
  * @param request USART request parameter.
  * @retval 1U (request is valid) or 0U (request is invalid)
  */
#define IS_USART_REQUEST_PARAMETER(request) (((request) == HAL_USART_REQUEST_RX_DATA_FLUSH)   \
                                             || ((request) == HAL_USART_REQUEST_TX_DATA_FLUSH))

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief Ensure that USART FIFO threshold level is valid.
  * @param threshold USART FIFO threshold level.
  * @retval 1U (threshold is valid) or 0U (threshold is invalid)
  */
#define IS_USART_FIFO_THRESHOLD(threshold) (((threshold) == HAL_USART_FIFO_THRESHOLD_1_8)    \
                                            || ((threshold) == HAL_USART_FIFO_THRESHOLD_1_4) \
                                            || ((threshold) == HAL_USART_FIFO_THRESHOLD_1_2) \
                                            || ((threshold) == HAL_USART_FIFO_THRESHOLD_3_4) \
                                            || ((threshold) == HAL_USART_FIFO_THRESHOLD_7_8) \
                                            || ((threshold) == HAL_USART_FIFO_THRESHOLD_8_8))

#endif /* USE_HAL_USART_FIFO */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief Ensure that USART Optional Interrupts for IT in Transmit are valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#define IS_USART_OPT_TX_IT(interrupts) (((interrupts) == HAL_USART_OPT_TX_IT_NONE)           \
                                        || ((interrupts) == HAL_USART_OPT_TX_IT_FIFO_EMPTY)  \
                                        || ((interrupts) == HAL_USART_OPT_TX_IT_DEFAULT))

/**
  * @brief Ensure that USART Optional Interrupts for IT in Receive are valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#define IS_USART_OPT_RX_IT(interrupts) (((interrupts) == HAL_USART_OPT_RX_IT_NONE)           \
                                        || ((interrupts) == HAL_USART_OPT_RX_IT_FIFO_FULL)   \
                                        || ((interrupts) == HAL_USART_OPT_RX_IT_DEFAULT))

/**
  * @brief Ensure that USART Optional Interrupts for IT in TransmitReceive are valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#define IS_USART_OPT_TXRX_IT(interrupts) (((interrupts) == HAL_USART_OPT_TXRX_IT_NONE)               \
                                          || ((interrupts) == HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY)   \
                                          || ((interrupts) == HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL)    \
                                          || ((interrupts) == HAL_USART_OPT_TXRX_IT_DEFAULT))
#endif /* USE_HAL_USART_FIFO */

#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
/**
  * @brief Ensure that USART Optional Interrupts for DMA in Transmit are valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_USART_OPT_TX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_TX_IT_NONE)         \
                                         || ((interrupts) == HAL_USART_OPT_DMA_TX_IT_HT)        \
                                         || ((interrupts) == HAL_USART_OPT_DMA_TX_IT_DEFAULT)   \
                                         || ((interrupts) == HAL_USART_OPT_DMA_TX_IT_SILENT))

#define IS_USART_DMA_TX_VALID_SILENT_MODE(handle_dma ,interrupts)                \
  (((interrupts) == HAL_USART_OPT_DMA_TX_IT_SILENT)                              \
   && (handle_dma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)

#else
#define IS_USART_OPT_TX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_TX_IT_NONE)        \
                                         || ((interrupts) == HAL_USART_OPT_DMA_TX_IT_HT)       \
                                         || ((interrupts) == HAL_USART_OPT_DMA_TX_IT_DEFAULT))
#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief Ensure that USART Optional Interrupts for DMA in Receive is valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_USART_OPT_RX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_RX_IT_NONE)         \
                                         || ((interrupts) == HAL_USART_OPT_DMA_RX_IT_HT)        \
                                         || ((interrupts) == HAL_USART_OPT_DMA_RX_IT_DEFAULT)   \
                                         || ((interrupts) == HAL_USART_OPT_DMA_RX_IT_SILENT))

#define IS_USART_DMA_RX_VALID_SILENT_MODE(handle_dma ,interrupts)                \
  (((interrupts) == HAL_USART_OPT_DMA_RX_IT_SILENT)                              \
   && (handle_dma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)

#else
#define IS_USART_OPT_RX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_RX_IT_NONE)         \
                                         || ((interrupts) == HAL_USART_OPT_DMA_RX_IT_HT)        \
                                         || ((interrupts) == HAL_USART_OPT_DMA_RX_IT_DEFAULT))

#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief Ensure that USART Optional Interrupts for DMA in TransmitReceive is valid.
  * @param interrupts USART Optional Interrupts.
  * @retval 1U (interrupt is valid) or 0U (interrupt is invalid)
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_USART_OPT_TXRX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_NONE)        \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_TX_IT_HT)    \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_RX_IT_HT)    \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_DEFAULT)  \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_SILENT))

#define IS_USART_DMA_TXRX_VALID_SILENT_MODE(handle_dmatx, handle_dmarx ,interrupts)              \
  (((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_SILENT)                                            \
   && ( (handle_dmatx->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)                       \
        || (handle_dmarx->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)) ? 0U : 1U)

#else
#define IS_USART_OPT_TXRX_DMA(interrupts) (((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_NONE)        \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_TX_IT_HT)    \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_RX_IT_HT)    \
                                           || ((interrupts) == HAL_USART_OPT_DMA_TXRX_IT_DEFAULT))
#endif /* USE_HAL_DMA_LINKEDLIST */

#endif /* USE_HAL_USART_DMA */

/**
  * @brief Check if USART instance is enabled. If yes, disable it.
  * @param handle specifies the USART Handle
  */
#define USART_ENSURE_INSTANCE_DISABLED(handle)               \
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
#define USART_ENSURE_INSTANCE_ENABLED(handle)                \
  do                                                         \
  {                                                          \
    if (instance_enabled != 0U)                              \
    {                                                        \
      LL_USART_Enable(handle);                               \
    }                                                        \
  }  while(0U)

/**
  * @brief Retrieve USART instance from handle.
  * @param handle specifies the USART Handle
  */
#define USART_GET_INSTANCE(handle)   ((USART_TypeDef *)((uint32_t)(handle)->instance))

/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/** @defgroup USART_Private_Functions USART Private Functions
  * @{
  */
#if defined(USE_HAL_USART_CLK_ENABLE_MODEL) && (USE_HAL_USART_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
/** @brief  Enable the USART clock.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__STATIC_INLINE void USART_EnableClock(const hal_usart_handle_t *husart)
{
  /*! Instance USART1 */
  if (husart->instance == HAL_USART1)
  {
    HAL_RCC_USART1_EnableClock();
  }
  /*! Instance USART2 */
  if (husart->instance == HAL_USART2)
  {
    HAL_RCC_USART2_EnableClock();
  }
#if defined(USART3)
  /*! Instance USART3 */
  if (husart->instance == HAL_USART3)
  {
    HAL_RCC_USART3_EnableClock();
  }
#endif /* USART3 */
}
#endif /* USE_HAL_USART_CLK_ENABLE_MODEL */

/** @brief  Report the USART mask to apply to retrieve the received data
  *         according to the word length and to the parity bits activation.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @warning If PCE = 1 in register USART_CR1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.
  * @retval HAL_ERROR Current configuration is incorrect.
  * @retval HAL_OK RDR successfully computed.
  */
__STATIC_INLINE hal_status_t USART_RDRMaskComputation(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
  uint32_t data_width = LL_USART_GetDataWidth(p_usartx);
  uint32_t parity = LL_USART_GetParity(p_usartx);

  if (data_width == LL_USART_DATAWIDTH_9_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      husart->rdr_register_mask = USART_RDR_MASK_9_BIT;
    }
    else
    {
      husart->rdr_register_mask = USART_RDR_MASK_8_BIT;
    }
  }
  else if (data_width == LL_USART_DATAWIDTH_8_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      husart->rdr_register_mask = USART_RDR_MASK_8_BIT;
    }
    else
    {
      husart->rdr_register_mask = USART_RDR_MASK_7_BIT;
    }
  }
  else if (data_width == LL_USART_DATAWIDTH_7_BIT)
  {
    if (parity == LL_USART_PARITY_NONE)
    {
      husart->rdr_register_mask = USART_RDR_MASK_7_BIT;
    }
    else
    {
      husart->rdr_register_mask = USART_RDR_MASK_6_BIT;
    }
  }
  else
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

static void USART_Abort(hal_usart_handle_t *husart);
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
static void USART_InitCallbacksToDefault(hal_usart_handle_t *husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
static hal_status_t USART_CheckEnabledState(hal_usart_handle_t *husart);
static hal_status_t USART_CheckCommunicationReady(hal_usart_handle_t *husart);
#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
static void USART_EndTransfer(hal_usart_handle_t *husart);
static void USART_DMATransmitCplt(hal_dma_handle_t *hdma);
static void USART_DMAReceiveCplt(hal_dma_handle_t *hdma);
static void USART_DMATxHalfCplt(hal_dma_handle_t *hdma);
static void USART_DMARxHalfCplt(hal_dma_handle_t *hdma);
static void USART_DMAError(hal_dma_handle_t *hdma);
static void USART_DMAAbortOnError(hal_dma_handle_t *hdma);
static void USART_DMATxAbortCallback(hal_dma_handle_t *hdma);
static void USART_DMARxAbortCallback(hal_dma_handle_t *hdma);
static void USART_DMADummy(hal_dma_handle_t *hdma);

#endif /* USE_HAL_USART_DMA */
static hal_status_t USART_WaitOnFlagUntilTimeout(hal_usart_handle_t *husart, uint32_t flag, uint32_t status,
                                                 uint32_t tick_start, uint32_t timeout_ms);
static void USART_TxISR_8BIT(hal_usart_handle_t *husart);
static void USART_TxISR_16BIT(hal_usart_handle_t *husart);
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
static void USART_TxISR_8BIT_FIFOEN(hal_usart_handle_t *husart);
static void USART_TxISR_16BIT_FIFOEN(hal_usart_handle_t *husart);
#endif /* USE_HAL_USART_FIFO */
static void USART_RxISR_8BIT(hal_usart_handle_t *husart);
static void USART_RxISR_16BIT(hal_usart_handle_t *husart);
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
static void USART_RxISR_8BIT_FIFOEN(hal_usart_handle_t *husart);
static void USART_RxISR_16BIT_FIFOEN(hal_usart_handle_t *husart);
static void USART_SetNbDataToProcess(hal_usart_handle_t *husart);
#endif /* USE_HAL_USART_FIFO */

static void USART_EndTransmit_IT(hal_usart_handle_t *husart);

static hal_status_t USART_Start_Transmit_IT(hal_usart_handle_t *husart, const uint8_t *p_data, uint32_t size,
                                            uint32_t interrupts);
static hal_status_t USART_Start_Receive_IT(hal_usart_handle_t *husart, uint8_t *p_data, uint32_t size,
                                           uint32_t interrupts);
static hal_status_t USART_Start_TransmitReceive_IT(hal_usart_handle_t *husart, const uint8_t *p_tx_data,
                                                   uint8_t *p_rx_data, uint32_t size, uint32_t interrupts);
#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
static hal_status_t USART_Start_Transmit_DMA(hal_usart_handle_t *husart, const uint8_t *p_data, uint32_t size,
                                             uint32_t interrupts);
static hal_status_t USART_Start_Receive_DMA(hal_usart_handle_t *husart, uint8_t *p_data, uint32_t size,
                                            uint32_t interrupts);
static hal_status_t USART_Start_TransmitReceive_DMA(hal_usart_handle_t *husart, const uint8_t *p_tx_data,
                                                    uint8_t *p_rx_data, uint32_t size, uint32_t interrupts);
#endif /* USE_HAL_USART_DMA */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup USART_Exported_Functions
  * @{
  */

/** @addtogroup USART_Exported_Functions_Group1
  * @{
This subsection provides a set of functions to initialize and deinitialize the USART in
synchronous mode.
  - Call the function HAL_USART_Init() to initialize the selected USART handle and associate an instance.
  - Call the function HAL_USART_DeInit() to deinitialize the given HAL USART instance by stopping any ongoing process
  and resetting the state machine.
  */

/**
  * @brief Initialize the USART handler for the associated instance.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which will contain the USART instance.
  * @param instance USART instance.
  * @retval HAL_OK USART instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM USART instance is NULL.
  * @retval HAL_ERROR USART semaphore creation failed (USE_HAL_MUTEX is set to 1).
  */
hal_status_t HAL_USART_Init(hal_usart_handle_t *husart, hal_usart_t instance)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_INSTANCE((USART_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (husart == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->instance = instance;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  USART_InitCallbacksToDefault(husart);
#endif /* (USE_HAL_USART_REGISTER_CALLBACKS) */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  /* Initialize the number of data to process during RX/TX ISR execution */
  husart->nb_tx_data_to_process = 1;
  husart->nb_rx_data_to_process = 1;
  husart->fifo_mode = HAL_USART_FIFO_MODE_DISABLED;
#endif /* USE_HAL_USART_FIFO */

#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
  husart->hdma_tx = (hal_dma_handle_t *) NULL;
  husart->hdma_rx = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_USART_DMA */

#if defined (USE_HAL_USART_USER_DATA) && (USE_HAL_USART_USER_DATA == 1)
  /* Reset the user data pointer to NULL */
  husart->p_user_data = NULL;
#endif /* USE_HAL_USART_USER_DATA */

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  husart->last_error_codes = 0;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */

#if defined(USE_HAL_USART_CLK_ENABLE_MODEL) && (USE_HAL_USART_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  USART_EnableClock(husart);
#endif /* USE_HAL_USART_CLK_ENABLE_MODEL */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Create the USART semaphore */
  if (HAL_OS_SemaphoreCreate(&husart->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  husart->global_state = HAL_USART_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief Deinitialize the USART handler, reset the flags, states, and counters.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
void HAL_USART_DeInit(hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
  ASSERT_DBG_PARAM(IS_USART_INSTANCE(p_usartx));

  const hal_usart_state_t temp_state = husart->global_state;

  if ((temp_state == HAL_USART_STATE_RX_ACTIVE) || (temp_state == HAL_USART_STATE_TX_ACTIVE)
      || (temp_state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    husart->global_state = HAL_USART_STATE_ABORT;
    USART_Abort(husart);
  }

  LL_USART_Disable(p_usartx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&husart->semaphore);
#endif /* USE_HAL_MUTEX  */

  husart->global_state = HAL_USART_STATE_RESET;
}
/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group2
  * @{
This subsection provides a set of functions to configure the USART in synchronous mode.
  - Call HAL_USART_SetConfig() to configure the initialized instance with a set of parameters containing:
    - baud rate
    - Prescaler
    - Word Length
    - Stop Bits
    - Parity: If the parity is enabled, then the MSB bit of the data written in the data register is transmitted but
    is changed by the parity bit.
    - Direction (Receiver/Transmitter)
    - Clock polarity
    - Clock phase
    - Last Bit Clock Pulse
    - Mode (Slave or Master)
  - Call HAL_USART_GetConfig() to retrieve the current configuration (not mandatory)
  - If needed, after calling HAL_USART_SetConfig(), modify the configuration using unitary configuration
  functions:
    - HAL_USART_SetBaudRate()
    - HAL_USART_SetStopBits()
    - HAL_USART_SetWordLength()
    - HAL_USART_SetParity()
    - HAL_USART_SetXferDirection()
    - HAL_USART_SetClockPolarity()
    - HAL_USART_SetClockPhase()
    - HAL_USART_SetLastBitClockPulse()
    - HAL_USART_SetMode()

  @warning
    - __Prescaler__: cannot be modified with a unitary configuration function as it impacts other parameters. Call
    HAL_USART_SetConfig() to modify it.

  - If needed, retrieve the different parameters by calling:
    - HAL_USART_GetBaudRate()
    - HAL_USART_GetStopBits()
    - HAL_USART_GetWordLength()
    - HAL_USART_GetParity()
    - HAL_USART_GetXferDirection()
    - HAL_USART_GetClockPolarity()
    - HAL_USART_GetClockPhase()
    - HAL_USART_GetLastBitClockPulse()
    - HAL_USART_GetMode()

  @warning
    - __Prescaler__: As there is no unitary configuration function for this parameter, there is no unitary
    getter as well.
  - Possible frame format:
    Depending on the frame length defined by the M1 and M0 bits (7-bit,
    8-bit or 9-bit), the possible USART formats are listed in the
    following table.

~~~
  USART frame format.
    +-----------------------------------------------------------------------+
    |  M1 bit |  M0 bit |  PCE bit  |             USART frame               |
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
  Acronym definition :
  - STB (Stop Bit)
  - SB (Start Bit)
  - PB (Parity Bit)
~~~
  */

/**
  * @brief Set the basic configuration to enable the use of the USART instance.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param p_config Pointer to a hal_usart_config_t structure containing the USART configuration.
  * @retval HAL_OK USART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  * @retval HAL_ERROR Error during instance enabling or kernel clock not enabled.
  */
hal_status_t HAL_USART_SetConfig(hal_usart_handle_t *husart, const hal_usart_config_t *p_config)
{
  USART_TypeDef *p_usartx;
  uint32_t instance_clock_freq;
  uint32_t div_temp;
  uint32_t brr_temp;
  uint32_t cr1_config;
  uint32_t cr2_config;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_USART_PRESCALER(p_config->clock_prescaler));
  ASSERT_DBG_PARAM(IS_USART_BAUD_RATE(p_config->baud_rate));
  ASSERT_DBG_PARAM(IS_USART_WORD_LENGTH(p_config->word_length));
  ASSERT_DBG_PARAM(IS_USART_STOP_BITS(p_config->stop_bits));
  ASSERT_DBG_PARAM(IS_USART_PARITY(p_config->parity));
  ASSERT_DBG_PARAM(IS_USART_DIRECTION(p_config->direction));
  ASSERT_DBG_PARAM(IS_USART_CLOCK_POLARITY(p_config->clock_polarity));
  ASSERT_DBG_PARAM(IS_USART_CLOCK_PHASE(p_config->clock_phase));
  ASSERT_DBG_PARAM(IS_USART_CLOCK_LAST_BIT(p_config->clock_last_bit));
  ASSERT_DBG_PARAM(IS_USART_MODE(p_config->mode));

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));
  p_usartx = USART_GET_INSTANCE(husart);

  LL_USART_Disable(p_usartx);

  if (p_config->mode == HAL_USART_MODE_SLAVE)
  {
    LL_USART_ConfigSyncSlaveMode(p_usartx);
  }
  else
  {
    LL_USART_ConfigSyncMasterMode(p_usartx);
  }
  husart->usart_mode = p_config->mode;

  cr1_config = ((uint32_t)p_config->word_length | (uint32_t)p_config->parity
                | (uint32_t)p_config->direction | LL_USART_OVERSAMPLING_8);

  cr2_config = ((uint32_t)p_config->stop_bits | (uint32_t)p_config->clock_polarity
                | (uint32_t)p_config->clock_phase | (uint32_t)p_config->clock_last_bit);

  LL_USART_ConfigXfer(p_usartx, cr1_config, cr2_config);

  LL_USART_SetPrescaler(p_usartx, (uint32_t)p_config->clock_prescaler);

  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_usartx);
  if (instance_clock_freq == 0U)
  {
    return HAL_ERROR;
  }

  div_temp = LL_USART_DIV_SAMPLING8(instance_clock_freq, (uint32_t)p_config->clock_prescaler, p_config->baud_rate);
  ASSERT_DBG_PARAM((div_temp >= USART_BRR_MIN) && (div_temp <= USART_BRR_MAX));
  brr_temp = div_temp & 0xFFF0U;
  brr_temp |= (uint16_t)((div_temp & (uint16_t)0x000FU) >> 1U);
  div_temp = brr_temp;

  LL_USART_WRITE_REG(p_usartx, BRR, (uint16_t)div_temp);

  husart->global_state = HAL_USART_STATE_IDLE;

  /* Enable USART instance */
  if (USART_CheckEnabledState(husart) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}
/**
  * @brief Get the current basic configuration set in the current USART instance.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param p_config Pointer to a hal_usart_config_t structure to store the USART configuration.
  */
void HAL_USART_GetConfig(const hal_usart_handle_t *husart, hal_usart_config_t *p_config)
{
  USART_TypeDef *p_usartx;
  uint32_t reg_temp;
  uint32_t instance_clock_freq;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);
  p_config->word_length = (hal_usart_word_length_t)(uint32_t)
                          (reg_temp & (LL_USART_DATAWIDTH_7_BIT | LL_USART_DATAWIDTH_9_BIT));
  p_config->parity = (hal_usart_parity_t)(uint32_t)(reg_temp & (LL_USART_PARITY_ODD));
  p_config->direction = (hal_usart_direction_t)(uint32_t)(reg_temp & (LL_USART_DIRECTION_TX_RX));

  reg_temp = LL_USART_READ_REG(p_usartx, CR2);

  p_config->stop_bits = (hal_usart_stop_bits_t)(uint32_t)(reg_temp & (LL_USART_STOP_BIT_1_5));
  p_config->clock_polarity = (hal_usart_clock_polarity_t)(uint32_t)(reg_temp & (LL_USART_CLOCK_POLARITY_HIGH));
  p_config->clock_phase = (hal_usart_clock_phase_t)(uint32_t)(reg_temp & (LL_USART_CLOCK_PHASE_2_EDGE));
  p_config->clock_last_bit = (hal_usart_clock_last_bit_state_t)(uint32_t)(reg_temp & (LL_USART_LASTCLKPULSE_ENABLED));

  p_config->clock_prescaler = (hal_usart_prescaler_t)LL_USART_GetPrescaler(p_usartx);

  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq(p_usartx);
  p_config->baud_rate = LL_USART_GetBaudRate(p_usartx, instance_clock_freq, (uint32_t)p_config->clock_prescaler,
                                             LL_USART_OVERSAMPLING_8);
  if (LL_USART_IsEnabledSPISlave(p_usartx) != 0U)
  {
    p_config->mode = HAL_USART_MODE_SLAVE;
  }
  else
  {
    p_config->mode = HAL_USART_MODE_MASTER;
  }
}

/**
  * @brief Set the Word Length configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param word_length Word length to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetWordLength(const hal_usart_handle_t *husart, hal_usart_word_length_t word_length)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_WORD_LENGTH(word_length));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetDataWidth(p_usartx, (uint32_t)word_length);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Word Length configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_word_length_t Current Word length configuration.
  */
hal_usart_word_length_t HAL_USART_GetWordLength(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_word_length_t)LL_USART_GetDataWidth(p_usartx);
}

/**
  * @brief Set the Parity configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param parity Parity to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetParity(const hal_usart_handle_t *husart, hal_usart_parity_t parity)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_PARITY(parity));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetParity(p_usartx, (uint32_t)parity);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Parity configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_parity_t Current Parity configuration.
  */
hal_usart_parity_t HAL_USART_GetParity(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_parity_t)LL_USART_GetParity(p_usartx);
}

/**
  * @brief Set the Stop Bits configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param stop_bits Stop Bits to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetStopBits(const hal_usart_handle_t *husart, hal_usart_stop_bits_t stop_bits)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_STOP_BITS(stop_bits));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetStopBitsLength(p_usartx, (uint32_t)stop_bits);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Stop Bits configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_stop_bits_t Current Stop Bits configuration.
  */
hal_usart_stop_bits_t HAL_USART_GetStopBits(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_stop_bits_t)LL_USART_GetStopBitsLength(p_usartx);
}

/**
  * @brief Set the XFer Direction configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param xfer_direction XFer Direction to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetXferDirection(const hal_usart_handle_t *husart, hal_usart_direction_t xfer_direction)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_DIRECTION(xfer_direction));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  LL_USART_SetTransferDirection(p_usartx, (uint32_t)xfer_direction);

  return HAL_OK;
}

/**
  * @brief Get the XFer Direction configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_direction_t Current XFer Direction configuration.
  */
hal_usart_direction_t HAL_USART_GetXferDirection(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_direction_t)LL_USART_GetTransferDirection(p_usartx);
}

/**
  * @brief Set the Clock polarity configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param clock_polarity Clock polarity to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetClockPolarity(const hal_usart_handle_t *husart, hal_usart_clock_polarity_t clock_polarity)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_CLOCK_POLARITY(clock_polarity));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetClockPolarity(p_usartx, (uint32_t)clock_polarity);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Clock polarity configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_clock_polarity_t Current Clock polarity configuration.
  */
hal_usart_clock_polarity_t HAL_USART_GetClockPolarity(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_clock_polarity_t)LL_USART_GetClockPolarity(p_usartx);
}

/**
  * @brief Set the Clock phase configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param clock_phase Clock phase to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetClockPhase(const hal_usart_handle_t *husart, hal_usart_clock_phase_t clock_phase)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_CLOCK_PHASE(clock_phase));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetClockPhase(p_usartx, (uint32_t)clock_phase);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Clock phase configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_clock_phase_t Current Clock phase configuration.
  */
hal_usart_clock_phase_t HAL_USART_GetClockPhase(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_clock_phase_t)LL_USART_GetClockPhase(p_usartx);
}

/**
  * @brief Set the last bit clock pulse configuration set as parameter into the handler instance registers
  *        (used in USART Synchronous SPI master mode only).
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param clock_last_bit Last bit clock pulse configuration to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetLastBitClockPulse(const hal_usart_handle_t *husart,
                                            hal_usart_clock_last_bit_state_t clock_last_bit)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_CLOCK_LAST_BIT(clock_last_bit));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetLastClkPulseOutput(p_usartx, (uint32_t)clock_last_bit);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the last bit clock pulse configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_clock_last_bit_state_t Current last bit clock pulse configuration.
  */
hal_usart_clock_last_bit_state_t HAL_USART_GetLastBitClockPulse(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  return (hal_usart_clock_last_bit_state_t)LL_USART_GetLastClkPulseOutput(p_usartx);
}

/**
  * @brief Set the baud rate configuration from the parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param baud_rate Baud rate value to set.
  * @retval HAL_OK USART instance has been correctly configured.
  * @retval HAL_INVALID_PARAM Required baud rate value cannot be set with the current configuration.
  */
hal_status_t HAL_USART_SetBaudRate(const hal_usart_handle_t *husart, uint32_t baud_rate)
{
  USART_TypeDef *p_usartx;
  uint32_t instance_clock_freq;
  uint32_t div_temp;
  uint32_t brr_temp;
  uint32_t instance_clock_prescaler;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_BAUD_RATE(baud_rate));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);
  instance_clock_freq = HAL_RCC_USART_GetKernelClkFreq((USART_TypeDef *)((uint32_t)(husart->instance)));
  instance_clock_prescaler = LL_USART_GetPrescaler(p_usartx);

  div_temp = LL_USART_DIV_SAMPLING8(instance_clock_freq, instance_clock_prescaler, baud_rate);
  ASSERT_DBG_PARAM((div_temp >= USART_BRR_MIN) && (div_temp <= USART_BRR_MAX));
  brr_temp = div_temp & 0xFFF0U;
  brr_temp |= (uint16_t)((div_temp & (uint16_t)0x000FU) >> 1U);
  div_temp = brr_temp;

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_WRITE_REG(p_usartx, BRR, div_temp);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the baud rate configuration from the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval uint32_t Current baud rate value.
  */
uint32_t HAL_USART_GetBaudRate(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;
  uint32_t instance_clock_freq;
  uint32_t prescaler;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);
  instance_clock_freq = HAL_USART_GetClockFreq(husart);

  prescaler = LL_USART_GetPrescaler(p_usartx);
  return LL_USART_GetBaudRate(p_usartx, instance_clock_freq, prescaler, LL_USART_OVERSAMPLING_8);
}

/**
  * @brief  Set the Mode configuration set as parameter into the handler instance registers.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  mode Mode value to set.
  * @warning When the USART operates in SPI slave mode, it handles data flow using
  *          the serial interface clock derived from the external SCLK signal
  *          provided by the external master SPI device.
  * @warning In SPI slave mode, the USART must be enabled before starting the master
  *          communications (or between frames while the clock is stable). Otherwise,
  *          if the USART slave is enabled while the master is in the middle of a
  *          frame, it will become desynchronized with the master.
  * @warning The data register of the slave needs to be ready before the first edge
  *          of the communication clock or before the end of the ongoing communication,
  *          otherwise the SPI slave will transmit zeros.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetMode(hal_usart_handle_t *husart, hal_usart_mode_t mode)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_MODE(mode));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  if (mode == HAL_USART_MODE_SLAVE)
  {
    LL_USART_ConfigSyncSlaveMode(p_usartx);
  }
  else
  {
    LL_USART_ConfigSyncMasterMode(p_usartx);
  }
  husart->usart_mode = mode;
  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Mode configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_mode_t Current mode value.
  */
hal_usart_mode_t HAL_USART_GetMode(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  if (LL_USART_IsEnabledSPISlave(p_usartx) != 0U)
  {
    return HAL_USART_MODE_SLAVE;
  }
  else
  {
    return HAL_USART_MODE_MASTER;
  }
}
/**
  * @}
  */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/** @addtogroup USART_Exported_Functions_Group3
  * @{
  This subsection provides a set of functions to use the FIFO mode feature for the USARTx instance.
  Before using the FIFO mode feature, configure the instance in synchronous
  mode with HAL_USART_SetConfig(). All these functions are available only if USE_HAL_USART_FIFO is set to 1.
  A set of functions is provided to use the FIFO mode feature:
    - HAL_USART_EnableFifoMode(): Enable the FIFO mode feature
    - HAL_USART_DisableFifoMode(): Disable the FIFO mode feature
    - HAL_USART_IsEnabledFifoMode(): Check if the FIFO mode feature is enabled
    - HAL_USART_SetTxFifoThreshold(): Set the configuration of the Tx FIFO
    - HAL_USART_GetTxFifoThreshold(): Retrieve the configuration of the Tx FIFO
    - HAL_USART_SetRxFifoThreshold(): Set the configuration of the Rx FIFO
    - HAL_USART_GetRxFifoThreshold(): Retrieve the configuration of the Rx FIFO

  The feature is designed to be used following the procedure:
    - HAL_USART_SetTxFifoThreshold()
    - HAL_USART_SetRxFifoThreshold()
    - HAL_USART_EnableFifoMode()
    - Start a process, e.g.: HAL_USART_Receive()
  */

/**
  * @brief Enable the FIFO into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_EnableFifoMode(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_EnableFIFO(p_usartx);

  husart->fifo_mode = HAL_USART_FIFO_MODE_ENABLED;

  /* Update Tx and Rx numbers of data to process */
  USART_SetNbDataToProcess(husart);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Disable the FIFO into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_DisableFifoMode(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_DisableFIFO(p_usartx);

  husart->fifo_mode = HAL_USART_FIFO_MODE_DISABLED;

  /* Update Tx and Rx numbers of data to process */
  USART_SetNbDataToProcess(husart);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Return the FIFO status according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_fifo_mode_status_t Current FIFO status.
  */
hal_usart_fifo_mode_status_t HAL_USART_IsEnabledFifoMode(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  return (hal_usart_fifo_mode_status_t)LL_USART_IsEnabledFIFO(p_usartx);
}

/**
  * @brief Set the Transmit FIFO Threshold configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param tx_fifo_threshold Transmit FIFO Threshold to applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetTxFifoThreshold(hal_usart_handle_t *husart, hal_usart_fifo_threshold_t tx_fifo_threshold)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_PARAM(IS_USART_FIFO_THRESHOLD(tx_fifo_threshold));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetTXFIFOThreshold(p_usartx, (uint32_t)tx_fifo_threshold);

  /* Update Tx numbers of data to process */
  USART_SetNbDataToProcess(husart);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Transmit FIFO Threshold configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_fifo_threshold_t Current Transmit FIFO Threshold configuration.
  */
hal_usart_fifo_threshold_t HAL_USART_GetTxFifoThreshold(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  return (hal_usart_fifo_threshold_t) LL_USART_GetTXFIFOThreshold(p_usartx);
}

/**
  * @brief Set the Receive FIFO Threshold configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param rx_fifo_threshold Receive FIFO Threshold to applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetRxFifoThreshold(hal_usart_handle_t *husart, hal_usart_fifo_threshold_t rx_fifo_threshold)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_PARAM(IS_USART_FIFO_THRESHOLD(rx_fifo_threshold));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  LL_USART_SetRXFIFOThreshold(p_usartx, (uint32_t)rx_fifo_threshold);

  /* Update Rx numbers of data to process */
  USART_SetNbDataToProcess(husart);

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Receive FIFO Threshold configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_fifo_threshold_t Current Receive FIFO Threshold configuration.
  */
hal_usart_fifo_threshold_t HAL_USART_GetRxFifoThreshold(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  p_usartx = USART_GET_INSTANCE(husart);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  return (hal_usart_fifo_threshold_t) LL_USART_GetRXFIFOThreshold(p_usartx);
}

/**
  * @}
  */

#endif /* USE_HAL_USART_FIFO */

/** @addtogroup USART_Exported_Functions_Group5
  * @{
  This subsection provides a set of functions to configure advanced features for the USARTx instance.
  Note that advanced features might not be supported on all instances.
  Before configuring advanced features, configure the instance in synchronous
  mode with HAL_USART_SetConfig().

  Configure some features by calling the associated functions:
    - HAL_USART_SetSlaveSelect(): Set the slave select to software or hardware using the USART NSS pin
    - HAL_USART_GetSlaveSelect(): Get the slave select configuration using the USART NSS pin
  */
/**
  * @brief Set the Slave select configuration set as parameter into the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param slave_select Slave select to be applied.
  * @retval HAL_OK USART instance has been correctly configured.
  */
hal_status_t HAL_USART_SetSlaveSelect(const hal_usart_handle_t *husart, hal_usart_slave_select_config_t slave_select)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_SLAVE_SELECT_CONFIG(slave_select));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  USART_ENSURE_INSTANCE_DISABLED(p_usartx);

  if (slave_select == HAL_USART_SLAVE_SELECT_PIN_USED)
  {
    LL_USART_EnableSPISlaveSelect(p_usartx);
  }
  else
  {
    LL_USART_DisableSPISlaveSelect(p_usartx);
  }

  USART_ENSURE_INSTANCE_ENABLED(p_usartx);

  return HAL_OK;
}

/**
  * @brief Get the Slave select configuration according to the handler instance registers.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_slave_select_config_t Current Slave select configuration.
  */
hal_usart_slave_select_config_t HAL_USART_GetSlaveSelect(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                                                    | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE
                                                    | HAL_USART_STATE_ABORT));

  p_usartx = USART_GET_INSTANCE(husart);

  /* condition inverted in LL */
  if (LL_USART_IsEnabledSPISlaveSelect(p_usartx) != 1U)
  {
    return HAL_USART_SLAVE_SELECT_PIN_IGNORED;
  }
  else
  {
    return HAL_USART_SLAVE_SELECT_PIN_USED;
  }
}

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group6 DMA Configuration functions
  * @{
  This subsection provides a set of functions to link the HAL USART handle to a Tx and Rx DMA handler
  for the USARTx instance.
  A set of functions is provided to use the DMA feature:
    - HAL_USART_SetTxDMA(): Link a DMA instance to the Tx channel
    - HAL_USART_SetRxDMA(): Link a DMA instance to the Rx channel
  */
#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
/**
  * @brief Set DMA channel for transmission.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param hdma_tx Pointer to a hal_dma_handle_t structure that contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_tx is NULL.
  */
hal_status_t HAL_USART_SetTxDMA(hal_usart_handle_t *husart, hal_dma_handle_t *hdma_tx)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(hdma_tx != NULL);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_tx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->hdma_tx = hdma_tx;
  hdma_tx->p_parent = husart;

  return HAL_OK;
}

/**
  * @brief Set DMA channel for Reception.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param hdma_rx Pointer to a hal_dma_handle_t structure which contains the DMA instance
  * @retval HAL_OK The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_rx is NULL.
  */
hal_status_t HAL_USART_SetRxDMA(hal_usart_handle_t *husart, hal_dma_handle_t *hdma_rx)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(hdma_rx != NULL);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_rx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->hdma_rx = hdma_rx;
  hdma_rx->p_parent = husart;

  return HAL_OK;
}
#endif /* USE_HAL_USART_DMA */

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group7 Callbacks Register functions
  * @{
  This subsection provides a set of functions to configure the callbacks for the USARTx instance.
  Before configuring the callbacks, configure the instance in synchronous
  mode with HAL_USART_SetConfig().
  A set of functions is provided to configure the callbacks:
    - HAL_USART_RegisterTxHalfCpltCallback(): Set the Tx half complete callback
    - HAL_USART_RegisterTxCpltCallback(): Set the Tx complete callback
    - HAL_USART_RegisterRxHalfCpltCallback(): Set the Rx half complete callback
    - HAL_USART_RegisterRxCpltCallback(): Set the Rx complete callback
    - HAL_USART_RegisterTxRxCpltCallback(): Set the TxRx complete callback
    - HAL_USART_RegisterErrorCallback(): Set the error callback
    - HAL_USART_RegisterAbortCpltCallback(): Set the abort complete callback
    - HAL_USART_RegisterRxFifoFullCallback(): Set the Rx FIFO full callback
    - HAL_USART_RegisterTxFifoEmptyCallback(): Set the Tx FIFO empty callback
  */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)

/**
  * @brief  Register the USART Tx Half Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback Pointer to the Tx Half Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterTxHalfCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_tx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Tx Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Tx Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterTxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_tx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Rx Half Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Rx Half Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterRxHalfCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_rx_half_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Rx Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Rx Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterRxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_rx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Tx/Rx Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Rx Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterTxRxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_tx_rx_cplt_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Error Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Error Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterErrorCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_error_callback = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the USART Abort Complete Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Abort Complete Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterAbortCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_abort_cplt_callback = p_callback;

  return HAL_OK;
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  Register the USART Rx Fifo Full Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Rx Fifo Full Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterRxFifoFullCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_rx_fifo_full_callback = p_callback;

  return HAL_OK;
}
/**
  * @brief  Register the USART Tx Fifo Empty Callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_callback pointer to the Tx Fifo Empty Callback function
  * @retval HAL_OK The function has been registered.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_USART_RegisterTxFifoEmptyCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(husart->global_state, (uint32_t)(HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  husart->p_tx_fifo_empty_callback = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_USART_FIFO */
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group8 IO operation functions
  *  @{
    This subsection provides a set of functions to manage the USART synchronous
      data transfers.

  The USART Synchronous SPI supports master and slave modes.
  In Master mode, SCLK is always an output, and is generated by transmission. This means that in order
  to achieve a reception in Master mode, a transmission (0x0F) must be performed simultaneously (full duplex).
  In Slave mode, SCLK is an input.

  There are two modes of transfer:
    - Blocking mode: The communication is performed in polling mode.
      The HAL status of all data processing is returned by the same function
      after finishing transfer.
    - Non-Blocking mode: The communication is performed using interrupts
      or DMA. These APIs return the HAL status.
      The end of the data processing will be indicated through the
      dedicated USART IRQ when using Interrupt mode or the DMA IRQ when
      using DMA mode.
      The HAL_USART_TxCpltCallback(), HAL_USART_RxCpltCallback() and HAL_USART_TxRxCpltCallback() user callbacks
      are executed respectively at the end of the transmit or receive process.
      The HAL_USART_ErrorCallback() user callback is executed when a communication error is detected.

  Blocking mode APIs are:
    - HAL_USART_Transmit()
    - HAL_USART_Receive()
    - HAL_USART_TransmitReceive()

  Non-Blocking mode APIs with Interrupt are:
    - HAL_USART_Transmit_IT()
    - HAL_USART_Transmit_IT_Opt()
    - HAL_USART_Receive_IT()
    - HAL_USART_Receive_IT_Opt()
    - HAL_USART_TransmitReceive_IT()
    - HAL_USART_TransmitReceive_IT_Opt()
    - HAL_USART_IRQHandler()

  Non-Blocking mode APIs with DMA are:
    - HAL_USART_Transmit_DMA()
    - HAL_USART_Transmit_DMA_Opt()
    - HAL_USART_Receive_DMA()
    - HAL_USART_Receive_DMA_Opt()
    - HAL_USART_TransmitReceive_DMA()
    - HAL_USART_TransmitReceive_DMA_Opt()
    - HAL_USART_Pause_DMA()
    - HAL_USART_Resume_DMA()

  A set of Transfer Complete Callbacks are provided in Non-Blocking mode:
    - HAL_USART_TxCpltCallback()
    - HAL_USART_RxCpltCallback()
    - HAL_USART_TxHalfCpltCallback()
    - HAL_USART_RxHalfCpltCallback()
    - HAL_USART_TxFifoEmptyCallback()
    - HAL_USART_RxFifoFullCallback()
    - HAL_USART_ErrorCallback()
    - HAL_USART_TxRxCpltCallback()

  Non-Blocking mode transfers can be aborted using Abort APIs:
    - HAL_USART_Abort()
    - HAL_USART_Abort_IT()

  For abort services based on interrupts (HAL_USART_Abort_IT()), an Abort Complete Callback is provided:
    - HAL_USART_AbortCpltCallback()

  In Non-Blocking mode transfers, possible errors are split into 2 categories.
  - Error is considered recoverable and non-blocking: transfer can proceed to the end, but error severity is
    evaluated by the user:
    - If Parity Error flag is detected in interrupt mode reception: Received character is then retrieved and stored
      in Rx buffer. Error code is set to allow the user to identify the error type. HAL_USART_ErrorCallback() user
      callback is then executed.
  - Error is considered blocking: transfer cannot be completed properly and is aborted.
    - If global state is HAL_USART_STATE_RX_ACTIVE: This concerns Overrun Error in Interrupt mode and all errors
      in DMA mode.
    - If global state is HAL_USART_STATE_TX_ACTIVE: This concerns Underrun Error in Interrupt mode and in DMA mode.
    - If global state is HAL_USART_STATE_TX_RX_ACTIVE: This concerns Overrun Error in Interrupt mode and in DMA mode.
    In all cases, HAL_USART_ErrorCallback() user callback is executed and error code is set to allow the user to
    identify the error type if USE_HAL_USART_GET_LAST_ERRORS=1.
  */

/**
  * @brief  Send an amount of data in blocking mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes to be sent.
  * @param  timeout_ms        Timeout duration.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Transmit(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                uint32_t timeout_ms)
{
  const uint8_t  *p_data_8_bits;
  const uint16_t *p_data_16_bits;
  uint32_t tick_start;
  uint32_t reg_temp;
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_usartx = USART_GET_INSTANCE(husart);

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_ACTIVE);

  /* Ensure Instance is ready */
  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }
  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

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

  husart->tx_xfer_size = size_byte;
  husart->tx_xfer_count = size_byte;

  /* Check the remaining data to be sent */
  while (husart->tx_xfer_count > 0U)
  {
    if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_TXE_TXFNF, 0U, tick_start, timeout_ms) != HAL_OK)
    {
      husart->global_state = HAL_USART_STATE_IDLE;
      return HAL_TIMEOUT;
    }
    if (p_data_8_bits == NULL)
    {
      LL_USART_TransmitData9(p_usartx, *p_data_16_bits);
      p_data_16_bits++;
    }
    else
    {
      LL_USART_TransmitData8(p_usartx, *p_data_8_bits);
      p_data_8_bits++;
    }
    husart->tx_xfer_count--;
  }

  if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_TC, 0U, tick_start, timeout_ms) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_TIMEOUT;
  }

  /* Clear Transmission Complete Flag */
  LL_USART_ClearFlag_TC(p_usartx);

  /* Clear overrun flag and discard the received data */
  LL_USART_ClearFlag_ORE(p_usartx);
  LL_USART_SetRequest(p_usartx, LL_USART_REQUEST_RX_DATA_FLUSH);
  LL_USART_SetRequest(p_usartx, LL_USART_REQUEST_TX_DATA_FLUSH);

  /* At end of Tx process, restore husart->global_state to Idle */
  husart->global_state = HAL_USART_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Receive an amount of data in blocking mode.
  * @param husart             Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param p_data             Pointer to data buffer.
  * @param size_byte          Amount of bytes to be received.
  * @param timeout_ms         Timeout duration.
  * @warning If USART is configured in Master mode, to receive synchronous data, dummy data are simultaneously
  *         transmitted.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Receive(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte, uint32_t timeout_ms)
{
  uint8_t  *p_data_8_bits;
  uint16_t *p_data_16_bits;
  uint16_t uh_mask;
  uint32_t tick_start;
  uint32_t reg_temp;
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_usartx = USART_GET_INSTANCE(husart);
  /* Ensure Instance is enabled */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

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


  /* Init tickstart for timeout management */
  tick_start = HAL_GetTick();

  husart->rx_xfer_size  = size_byte;
  husart->rx_xfer_count = size_byte;

  /* Computation of USART mask to apply to RDR register */
  if (USART_RDRMaskComputation(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }
  uh_mask = husart->rdr_register_mask;

  /* As long as data have to be received */
  while (husart->rx_xfer_count > 0U)
  {
    if (husart->usart_mode == HAL_USART_MODE_MASTER)
    {
      /* Wait until TXE flag is set to send dummy byte in order to generate the
      * clock for the slave to send data.
      * Whatever the frame length (7, 8 or 9-bit long), the same dummy value
      * can be written for all the cases. */
      if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_TXE_TXFNF, 0U, tick_start, timeout_ms) != HAL_OK)
      {
        husart->global_state = HAL_USART_STATE_IDLE;
        return HAL_TIMEOUT;
      }
      LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
    }

    /* Wait for RXNE Flag */
    if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_RXNE_RXFNE, 0U, tick_start, timeout_ms) != HAL_OK)
    {
      husart->global_state = HAL_USART_STATE_IDLE;
      return HAL_TIMEOUT;
    }

    if (p_data_8_bits == NULL)
    {
      *p_data_16_bits = LL_USART_ReceiveData9(p_usartx) & uh_mask;
      p_data_16_bits++;
    }
    else
    {
      *p_data_8_bits = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_usartx) & uh_mask);
      p_data_8_bits++;
    }

    husart->rx_xfer_count--;

  }

  /* Clear SPI slave underrun flag and discard transmit data */
  if (husart->usart_mode == HAL_USART_MODE_SLAVE)
  {
    LL_USART_ClearFlag_UDR(p_usartx);
    LL_USART_SetRequest(p_usartx, LL_USART_REQUEST_TX_DATA_FLUSH);
  }

  /* At end of Rx process, restore husart->global_state to Idle */
  husart->global_state = HAL_USART_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Full-duplex send and receive an amount of data in blocking mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data         Pointer to TX data buffer.
  * @param  p_rx_data         Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be sent (same amount to be received).
  * @param  timeout_ms        Timeout duration.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data and the received data are handled as sets of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       Operation exceeds user timeout.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_TransmitReceive(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                       uint32_t size_byte, uint32_t timeout_ms)
{
  uint8_t  *p_rx_data_8_bits;
  const uint8_t  *p_tx_data_8_bits;
  uint16_t *p_rx_data_16_bits;
  const uint16_t *p_tx_data_16_bits;
  USART_TypeDef *p_usartx;
  uint16_t uh_mask;
  uint32_t rx_data_count;
  uint32_t tick_start;
  uint32_t reg_temp;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_usartx = USART_GET_INSTANCE(husart);

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    p_rx_data_16_bits = (uint16_t *)p_rx_data;
    p_tx_data_16_bits = (const uint16_t *)p_tx_data;
    p_rx_data_8_bits = NULL;
    p_tx_data_8_bits = NULL;
  }
  else
  {
    p_rx_data_8_bits = (uint8_t *)p_rx_data;
    p_tx_data_8_bits = (const uint8_t *)p_tx_data;
    p_rx_data_16_bits = NULL;
    p_tx_data_16_bits = NULL;
  }

  /* Init tickstart for timeout management */
  tick_start = HAL_GetTick();

  husart->rx_xfer_size = size_byte;
  husart->rx_xfer_count = size_byte;
  husart->tx_xfer_size = size_byte;
  husart->tx_xfer_count = size_byte;

  /* Computation of USART mask to apply to RDR register */
  if (USART_RDRMaskComputation(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }
  uh_mask = husart->rdr_register_mask;

  if ((husart->tx_xfer_count == 0x01U) || (husart->usart_mode == HAL_USART_MODE_SLAVE))
  {
    /* Wait until TXE flag is set to send data */
    if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_TXE_TXFNF, 0U, tick_start, timeout_ms) != HAL_OK)
    {
      husart->global_state = HAL_USART_STATE_IDLE;
      return HAL_TIMEOUT;
    }
    if (p_tx_data_8_bits == NULL)
    {
      LL_USART_TransmitData9(p_usartx, (*p_tx_data_16_bits & uh_mask));
      p_tx_data_16_bits++;
    }
    else
    {
      LL_USART_TransmitData8(p_usartx, (*p_tx_data_8_bits & (uint8_t)(uh_mask & 0xFFU)));
      p_tx_data_8_bits++;
    }

    husart->tx_xfer_count--;
  }

  /* Check the remain data to be sent */
  rx_data_count = husart->rx_xfer_count;
  while ((husart->rx_xfer_count > 0U) || (rx_data_count > 0U))
  {
    if (husart->tx_xfer_count > 0U)
    {
      /* Wait until TXE flag is set to send data */
      if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_TXE_TXFNF, 0U, tick_start, timeout_ms) != HAL_OK)
      {
        husart->global_state = HAL_USART_STATE_IDLE;
        return HAL_TIMEOUT;
      }
      if (p_tx_data_8_bits == NULL)
      {
        LL_USART_TransmitData9(p_usartx, (*p_tx_data_16_bits & uh_mask));
        p_tx_data_16_bits++;
      }
      else
      {
        LL_USART_TransmitData8(p_usartx, (*p_tx_data_8_bits & (uint8_t)(uh_mask & 0xFFU)));
        p_tx_data_8_bits++;
      }

      husart->tx_xfer_count--;
    }

    if (husart->rx_xfer_count > 0U)
    {
      /* Wait for RXNE Flag */
      if (USART_WaitOnFlagUntilTimeout(husart, LL_USART_ISR_RXNE_RXFNE, 0U, tick_start, timeout_ms) != HAL_OK)
      {
        husart->global_state = HAL_USART_STATE_IDLE;
        return HAL_TIMEOUT;
      }

      if (p_rx_data_8_bits == NULL)
      {
        *p_rx_data_16_bits = LL_USART_ReceiveData9(p_usartx) & uh_mask;
        p_rx_data_16_bits++;
      }
      else
      {
        *p_rx_data_8_bits = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_usartx) & uh_mask);
        p_rx_data_8_bits++;
      }

      husart->rx_xfer_count--;
    }
    rx_data_count = husart->rx_xfer_count;
  }

  /* At end of TxRx process, restore husart->global_state to Idle */
  husart->global_state = HAL_USART_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Send an amount of data in interrupt mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes to be sent.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Transmit_IT(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Transmit_IT(husart, (const uint8_t *)p_data, size_byte, HAL_USART_OPT_TX_IT_NONE);
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  Send an amount of data in interrupt mode, allow user to enable Optional Interrupts part of
  *         \ref USART_Transmit_IT_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes to be sent.
  * @param  interrupts        Optional interrupts part of \ref USART_Transmit_IT_Optional_Interrupts.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Transmit_IT_Opt(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                       uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_USART_OPT_TX_IT(interrupts));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Transmit_IT(husart, (const uint8_t *)p_data, size_byte, interrupts);
}
#endif /* USE_HAL_USART_FIFO */

/**
  * @brief  Receive an amount of data in interrupt mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes to be received.
  * @warning If USART is configured in Master mode, to receive synchronous data, dummy data are simultaneously
  *         transmitted.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Receive_IT(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Receive_IT(husart, (uint8_t *)p_data, size_byte, HAL_USART_OPT_RX_IT_NONE);
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief Receive an amount of data in interrupt mode, allow user to enable Optional Interrupts part of
  *         \ref USART_Receive_IT_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes to be received.
  * @param  interrupts        Optional interrupts part of \ref USART_Receive_IT_Optional_Interrupts.
  * @warning If USART is configured in Master mode, to receive synchronous data, dummy data are simultaneously
  *         transmitted.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Receive_IT_Opt(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte,
                                      uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_USART_OPT_RX_IT(interrupts));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Receive_IT(husart, (uint8_t *)p_data, size_byte, interrupts);
}
#endif /* USE_HAL_USART_FIFO */

/**
  * @brief Full-duplex send and receive an amount of data in interrupt mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data         Pointer to TX data buffer.
  * @param  p_rx_data         Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be sent (same amount to be received).
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data and the received data are handled as sets of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_TransmitReceive_IT(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                          uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_TransmitReceive_IT(husart, (const uint8_t *)p_tx_data, (uint8_t *)p_rx_data, size_byte,
                                        HAL_USART_OPT_TXRX_IT_NONE);
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  Full-duplex send and receive an amount of data in interrupt mode, allow user to enable Optional
  *         Interrupts part of \ref USART_TransmitReceive_IT_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data         Pointer to TX data buffer.
  * @param  p_rx_data         Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be sent (same amount to be received).
  * @param  interrupts        Optional interrupts part of \ref USART_TransmitReceive_IT_Optional_Interrupts.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data and the received data are handled as sets of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_TransmitReceive_IT_Opt(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                              uint32_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(IS_USART_OPT_TXRX_IT(interrupts));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_TransmitReceive_IT(husart, (const uint8_t *)p_tx_data, (uint8_t *)p_rx_data, size_byte,
                                        interrupts);
}
#endif /* USE_HAL_USART_FIFO */
#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
/**
  * @brief Send an amount of data in DMA mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Transmit_DMA(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_tx != NULL);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Transmit_DMA(husart, (const uint8_t *)p_data, size_byte, HAL_USART_OPT_DMA_TX_IT_HT);
}

/**
  * @brief  Send an amount of data in DMA mode, allow user to enable Optional Interrupts part of
  *         \ref USART_Transmit_DMA_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to data buffer.
  * @param  size_byte         Amount of bytes.
  * @param  interrupts        Optional interrupts part of \ref USART_Transmit_DMA_Optional_Interrupts.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_Transmit_DMA_Opt(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                        uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_tx != NULL);
  ASSERT_DBG_PARAM(IS_USART_OPT_TX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_USART_DMA_TX_VALID_SILENT_MODE(husart->hdma_tx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Transmit_DMA(husart, (const uint8_t *)p_data, size_byte, interrupts);
}

/**
  * @brief  Receive an amount of data in DMA mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be received.
  * @warning When the USART parity is enabled (PCE bit in register USART_CR1 = 1), the received data contains
  *         the parity bit (MSB position).
  * @warning If USART is configured in Master mode, the USART DMA transmit channel must be configured in order to
  *         generate the clock for the slave.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling or missing Tx DMA handle when acting as master.
  */
hal_status_t HAL_USART_Receive_DMA(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_rx != NULL);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Receive_DMA(husart, (uint8_t *)p_data, size_byte, HAL_USART_OPT_DMA_RX_IT_HT);
}

/**
  * @brief  Receive an amount of data in DMA mode, allow user to enable Optional Interrupts part of
  *         \ref USART_Receive_DMA_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data            Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be received.
  * @param  interrupts        Optional interrupts part of \ref USART_Receive_DMA_Optional_Interrupts.
  * @warning When the USART parity is enabled (PCE bit in register USART_CR1 = 1), the received data contains
  *         the parity bit (MSB position).
  * @warning If USART is configured in Master mode, the USART DMA transmit channel must be configured in order to
  *         generate the clock for the slave.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the received data is handled as a set of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling or missing Tx DMA handle when acting as master.
  */
hal_status_t HAL_USART_Receive_DMA_Opt(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte,
                                       uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_rx != NULL);
  ASSERT_DBG_PARAM(IS_USART_OPT_RX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_USART_DMA_RX_VALID_SILENT_MODE(husart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_Receive_DMA(husart, (uint8_t *)p_data, size_byte, interrupts);
}

/**
  * @brief  Full-duplex transmit and receive an amount of data in non-blocking mode.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data         Pointer to TX data buffer.
  * @param  p_rx_data         Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be sent (same amount to be received).
  * @warning When the USART parity is enabled (PCE bit in register USART_CR1 = 1) the data received contains the
  *          parity bit.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data and the received data are handled as sets of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_TransmitReceive_DMA(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                           uint32_t size_byte)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_rx != NULL);
  ASSERT_DBG_PARAM(husart->hdma_tx != NULL);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_TransmitReceive_DMA(husart, (const uint8_t *)p_tx_data, (uint8_t *)p_rx_data, size_byte,
                                         (HAL_USART_OPT_DMA_TXRX_TX_IT_HT | HAL_USART_OPT_DMA_TXRX_RX_IT_HT));

}

/**
  * @brief  Full-duplex transmit and receive an amount of data in non-blocking mode, allow user to enable Optional
  *         Interrupts part of \ref USART_TransmitReceive_DMA_Optional_Interrupts.
  * @param  husart            Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data         Pointer to TX data buffer.
  * @param  p_rx_data         Pointer to RX data buffer.
  * @param  size_byte         Amount of bytes to be sent (same amount to be received).
  * @param  interrupts        Optional interrupts part of \ref USART_TransmitReceive_DMA_Optional_Interrupts.
  * @warning When the USART parity is enabled (PCE bit in register USART_CR1 = 1) the data received contains the
  *          parity bit.
  * @warning When USART parity is not enabled (PCE bit in register USART_CR1 = 0), and Word Length is configured
  *         to 9 bits (M1-M0 = 01), the sent data and the received data are handled as sets of u16.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Error during instance enabling.
  */
hal_status_t HAL_USART_TransmitReceive_DMA_Opt(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                               uint32_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0);
  ASSERT_DBG_PARAM(husart->hdma_rx != NULL);
  ASSERT_DBG_PARAM(husart->hdma_tx != NULL);
  ASSERT_DBG_PARAM(IS_USART_OPT_TXRX_DMA(interrupts));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_USART_DMA_TXRX_VALID_SILENT_MODE(husart->hdma_tx, husart->hdma_rx, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(husart, global_state, HAL_USART_STATE_IDLE, HAL_USART_STATE_TX_RX_ACTIVE);

  if (USART_CheckCommunicationReady(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  return USART_Start_TransmitReceive_DMA(husart, (const uint8_t *)p_tx_data, (uint8_t *)p_rx_data, size_byte,
                                         interrupts);
}

/**
  * @brief  Pause ongoing DMA transfers (Tx, Rx or both).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_USART_Pause_DMA(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_RX_ACTIVE | HAL_USART_STATE_TX_ACTIVE |
                   HAL_USART_STATE_TX_RX_ACTIVE);

  p_usartx = USART_GET_INSTANCE(husart);
  const hal_usart_state_t temp_state = husart->global_state;
  if ((temp_state == HAL_USART_STATE_TX_ACTIVE) || (temp_state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    if (LL_USART_IsEnabledDMAReq_TX(p_usartx) != 0U)
    {
      LL_USART_DisableDMAReq_TX(p_usartx);
    }
  }

  if ((temp_state == HAL_USART_STATE_RX_ACTIVE) || (temp_state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    if (LL_USART_IsEnabledDMAReq_RX(p_usartx) != 0U)
    {
      LL_USART_DisableIT_PE(p_usartx);
      LL_USART_DisableIT_ERROR(p_usartx);
      LL_USART_DisableDMAReq_RX(p_usartx);
    }
  }
  return HAL_OK;
}

/**
  * @brief Resume ongoing DMA transfers (Tx, Rx or both).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_USART_Resume_DMA(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;
  hal_usart_state_t state;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_RX_ACTIVE | HAL_USART_STATE_TX_ACTIVE |
                   HAL_USART_STATE_TX_RX_ACTIVE);

  p_usartx = USART_GET_INSTANCE(husart);
  state = husart->global_state;
  if ((state == HAL_USART_STATE_TX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    if (husart->hdma_tx != NULL)
    {
      LL_USART_EnableDMAReq_TX(p_usartx);
    }
  }

  if ((state == HAL_USART_STATE_RX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    if (husart->hdma_rx != NULL)
    {
      LL_USART_ClearFlag_ORE(p_usartx);

      if (LL_USART_GetParity(p_usartx) != LL_USART_PARITY_NONE)
      {
        LL_USART_EnableIT_PE(p_usartx);
      }
      LL_USART_RequestRxDataFlush(p_usartx);
      LL_USART_EnableIT_ERROR(p_usartx);
      LL_USART_EnableDMAReq_RX(p_usartx);
    }
  }
  return HAL_OK;
}

#endif /* USE_HAL_USART_DMA */

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  * @warning This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_USART_Abort(hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE | HAL_USART_STATE_TX_ACTIVE
                   | HAL_USART_STATE_TX_RX_ACTIVE);

  if (husart->global_state != HAL_USART_STATE_IDLE)
  {
    husart->global_state = HAL_USART_STATE_ABORT;
    USART_Abort(husart);

    husart->global_state  = HAL_USART_STATE_IDLE;
  }

  return HAL_OK;
}

/**
  * @brief  Abort ongoing transfers (Interrupt mode).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note    This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  * @warning This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_USART_Abort_IT(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;
  uint32_t abort_cplt = 1U;

  ASSERT_DBG_PARAM(husart != NULL);

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE | HAL_USART_STATE_TX_ACTIVE
                   | HAL_USART_STATE_TX_RX_ACTIVE);

  p_usartx = USART_GET_INSTANCE(husart);

  husart->global_state = HAL_USART_STATE_ABORT;

  LL_USART_DisableIT_CR1(p_usartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_TXEIE_TXFNFIE |
                                    LL_USART_CR1_TCIE));
  LL_USART_DisableIT_CR3(p_usartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE | LL_USART_CR3_TXFTIE));

  if (husart->global_state != HAL_USART_STATE_IDLE)
  {
    husart->global_state = HAL_USART_STATE_ABORT;

    /* if Rx FIFO full or Tx FIFO empty Optional IT have been activated, clear status */
    if (LL_USART_IsEnabledIT_TXFE(p_usartx) != 0U)
    {
      LL_USART_DisableIT_TXFE(p_usartx);
      LL_USART_ClearFlag_TXFE(p_usartx);
    }
    if (LL_USART_IsEnabledIT_RXFF(p_usartx) != 0U)
    {
      LL_USART_DisableIT_RXFF(p_usartx);
    }
#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
    if (LL_USART_IsEnabledDMAReq_TX(p_usartx) != 0U)
    {
      LL_USART_DisableDMAReq_TX(p_usartx);
      if (husart->hdma_tx != NULL)
      {
        if (husart->hdma_tx->global_state == HAL_DMA_STATE_ACTIVE)
        {
          husart->hdma_tx->p_xfer_abort_cb = USART_DMATxAbortCallback;
          if (HAL_DMA_Abort_IT(husart->hdma_tx) == HAL_OK)
          {
            abort_cplt = 0U;
          }
        }
      }
    }

    if (LL_USART_IsEnabledDMAReq_RX(p_usartx) != 0U)
    {
      LL_USART_DisableDMAReq_RX(p_usartx);
      if (husart->hdma_rx != NULL)
      {
        if (husart->hdma_rx->global_state == HAL_DMA_STATE_ACTIVE)
        {
          husart->hdma_rx->p_xfer_abort_cb = USART_DMARxAbortCallback;
          if (HAL_DMA_Abort_IT(husart->hdma_rx) == HAL_OK)
          {
            abort_cplt = 0U;
          }
        }
      }
    }

#endif /* USE_HAL_USART_DMA */

    /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
    if (abort_cplt != 0U)
    {
      /* Reset Tx and Rx transfer counters */
      husart->rx_xfer_count = 0U;
      husart->tx_xfer_count = 0U;

      husart->p_rx_isr = NULL;
      husart->p_tx_isr = NULL;

      /* Clear the Error flags in the ICR register */
      LL_USART_ClearFlag(p_usartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
      /* Flush the whole TX FIFO (if needed) */
      if (husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED)
      {
        LL_USART_RequestTxDataFlush(p_usartx);
      }
#endif /* USE_HAL_USART_FIFO */

      LL_USART_RequestRxDataFlush(p_usartx);

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Complete Callback */
      husart->p_abort_cplt_callback(husart);
#else
      /* Call legacy weak Abort Complete Callback */
      HAL_USART_AbortCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Even if Abort has done nothing as no transfer callback is called */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Complete Callback */
    husart->p_abort_cplt_callback(husart);
#else
    /* Call legacy weak Abort Complete Callback */
    HAL_USART_AbortCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
  }
  husart->global_state  = HAL_USART_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief Send Specific USART Request.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param request Request to process.
  * @retval HAL_OK The request has been sent.
  */
hal_status_t HAL_USART_SendRequest(hal_usart_handle_t *husart, hal_usart_request_t request)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM(husart != NULL);
  ASSERT_DBG_PARAM(IS_USART_REQUEST_PARAMETER(request));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_IDLE);

  p_usartx = USART_GET_INSTANCE(husart);

  LL_USART_SetRequest(p_usartx, (uint16_t)request);

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group12
  * @{
  This subsection provides the function handling the interruption of the USARTx in synchronous mode.

  - HAL_USART_IRQHandler(): process the interruption of an instance

  HAL_USART_IRQHandler() is designed to process the different interruptions in the following order:
    - Error on Rx side (PE, ORE, RTOF, UDR)
    - Error on DMA side
    - Data on Rx side
    - Data on Tx side

Depending on the process function one's use, different callback might be triggered:

| Process API \n \ \n Callbacks  | HAL_USART_Transmit_IT | HAL_USART_Receive_IT | HAL_USART_TransmitReceive_IT |
|--------------------------------|:---------------------:|:--------------------:|:----------------------------:|
| HAL_USART_TxCpltCallback       |           x           |                      |                              |
| HAL_USART_RxCpltCallback       |                       |          x           |                              |
| HAL_USART_ErrorCallback        |           x           |          x           |               x              |
| HAL_USART_TxRxCpltCallback     |                       |                      |               x              |

| Process API \n \ \n Callbacks   | HAL_USART_Transmit_IT_Opt       | HAL_USART_Receive_IT_Opt   |
|---------------------------------|:-------------------------------:|:--------------------------:|
| HAL_USART_TxCpltCallback        |                x                |                            |
| HAL_USART_RxCpltCallback        |                                 |               x            |
| HAL_USART_ErrorCallback         |                x                |               x            |
| HAL_USART_TxFifoEmptyCallback*  |                x                |                            |
| HAL_USART_RxFifoFullCallback**  |                                 |               x            |
@note * with HAL_USART_OPT_TX_IT_FIFO_EMPTY arguments value for interrupts parameter
@note ** with HAL_USART_OPT_RX_IT_FIFO_FULL arguments value for interrupts parameter

| Process API \n \ \n Callbacks  | HAL_USART_TransmitReceive_IT_Opt    |
|--------------------------------|:-----------------------------------:|
| HAL_USART_TxRxCpltCallback     |                   x                 |
| HAL_USART_ErrorCallback        |                   x                 |
| HAL_USART_TxFifoEmptyCallback* |                   x                 |
| HAL_USART_RxFifoFullCallback** |                   x                 |
@note * with HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY argument value for interrupts parameter
@note ** with HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL argument value for interrupts parameter

| Process API \n \ \n Callbacks   | HAL_USART_Transmit_DMA | HAL_USART_Receive_DMA | HAL_USART_TransmitReceive_DMA |
|---------------------------------|:----------------------:|:---------------------:|:-----------------------------:|
| HAL_USART_TxHalfCpltCallback*   |           x            |                       |               x               |
| HAL_USART_TxCpltCallback        |           x            |                       |                               |
| HAL_USART_RxHalfCpltCallback*   |                        |           x           |               x               |
| HAL_USART_RxCpltCallback        |                        |           x           |                               |
| HAL_USART_ErrorCallback**       |           x            |           x           |               x               |
| HAL_USART_TxRxCpltCallback      |                        |                       |               x               |
@note * these callbacks might be called following DMA IRQ management, not USARTx IRQ management.
@note ** these callbacks might be called following DMA IRQ management, or USARTx IRQ management.

| Process API \n \ \n Callbacks    | HAL_USART_Transmit_DMA_Opt()    | HAL_USART_Receive_DMA_Opt()    |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_USART_TxCpltCallback         |                x                |                                |
| HAL_USART_RxCpltCallback         |                                 |               x                |
| HAL_USART_ErrorCallback          |                x                |               x                |
| HAL_USART_TxFifoEmptyCallback*   |                x                |                                |
| HAL_USART_RxFifoFullCallback**   |                                 |               x                |
| HAL_USART_TxHalfCpltCallback***  |                x                |                                |
| HAL_USART_RxHalfCpltCallback**** |                                 |               x                |
@note * with HAL_USART_OPT_TX_IT_FIFO_EMPTY arguments value for interrupts parameter
@note ** with HAL_USART_OPT_RX_IT_FIFO_FULL arguments value for interrupts parameter
@note *** with HAL_USART_OPT_DMA_TX_IT_HT arguments value for interrupts parameter
@note **** with HAL_USART_OPT_DMA_RX_IT_HT arguments value for interrupts parameter

| Process API \n \ \n Callbacks    | HAL_USART_TransmitReceive_DMA_Opt() |
|----------------------------------|:-----------------------------------:|
| HAL_USART_TxRxCpltCallback       |                   x                 |
| HAL_USART_ErrorCallback          |                   x                 |
| HAL_USART_TxFifoEmptyCallback*   |                   x                 |
| HAL_USART_RxFifoFullCallback**   |                   x                 |
| HAL_USART_TxHalfCpltCallback***  |                   x                 |
| HAL_USART_RxHalfCpltCallback**** |                   x                 |
@note * with HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY arguments value for interrupts parameter
@note ** with HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL arguments value for interrupts parameter
@note *** with HAL_USART_OPT_DMA_TXRX_TX_IT_HT arguments value for interrupts parameter
@note **** with HAL_USART_OPT_DMA_TXRX_RX_IT_HT arguments value for interrupts parameter

| Process API \n \ \n Callbacks       | HAL_USART_Abort_IT |
|-------------------------------------|:------------------:|
| HAL_USART_AbortCpltCallback         |          x         |
  */

/**
  * @brief  Handle USART interrupt request.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
void HAL_USART_IRQHandler(hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);

  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  uint32_t isr_flags = LL_USART_READ_REG(p_usartx, ISR);
  uint32_t cr1_its   = LL_USART_READ_REG(p_usartx, CR1);
  uint32_t cr3_its   = LL_USART_READ_REG(p_usartx, CR3);

  uint32_t error_flags;
  uint32_t error_code = 0U;

  /* If no error occurs */
  error_flags = (isr_flags & (uint32_t)(LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_ORE | LL_USART_ISR_NE |
                                        LL_USART_ISR_RTOF | LL_USART_ISR_UDR));
  if (error_flags == 0U)
  {
    /* USART in mode Receiver ---------------------------------------------------*/
    if (((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U)
        && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3_its & LL_USART_CR3_RXFTIE) != 0U)))
    {
      if (husart->p_rx_isr != NULL)
      {
        husart->p_rx_isr(husart);
      }
      return;
    }
  }

  /* If some errors occur */
  if ((error_flags != 0U)
      && (((cr3_its & (LL_USART_CR3_RXFTIE | LL_USART_CR3_EIE)) != 0U)
          || ((cr1_its & (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE)) != 0U)))
  {
    /* USART parity error interrupt occurred -------------------------------------*/
    if (((isr_flags & LL_USART_ISR_PE) != 0U) && ((cr1_its & LL_USART_CR1_PEIE) != 0U))
    {
      LL_USART_ClearFlag_PE(p_usartx);

      error_code |= HAL_USART_RECEIVE_ERROR_PE;
    }

    /* USART frame error interrupt occurred --------------------------------------*/
    if (((isr_flags & LL_USART_ISR_FE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag_FE(p_usartx);

      error_code |= HAL_USART_RECEIVE_ERROR_FE;
    }

    /* USART noise error interrupt occurred --------------------------------------*/
    if (((isr_flags & LL_USART_ISR_NE) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
    {
      LL_USART_ClearFlag_NE(p_usartx);

      error_code |= HAL_USART_RECEIVE_ERROR_NE;
    }

    /* USART Over-Run interrupt occurred -----------------------------------------*/
    if (((isr_flags & LL_USART_ISR_ORE) != 0U)
        && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3_its & (LL_USART_CR3_RXFTIE | LL_USART_CR3_EIE)) != 0U)))
    {
      LL_USART_ClearFlag_ORE(p_usartx);

      error_code |= HAL_USART_RECEIVE_ERROR_ORE;
    }

    /* USART Receiver Timeout interrupt occurred ---------------------------------*/
    if (((isr_flags & LL_USART_ISR_RTOF) != 0U) && ((cr1_its & LL_USART_CR1_RTOIE) != 0U))
    {
      LL_USART_ClearFlag_RTO(p_usartx);

      error_code |= HAL_USART_RECEIVE_ERROR_RTO;
    }

    /* USART SPI slave underrun error interrupt occurred -------------------------*/
    if (((isr_flags & LL_USART_ISR_UDR) != 0U) && ((cr3_its & LL_USART_CR3_EIE) != 0U))
    {
      /* Ignore SPI slave underrun errors when reception is going on */
      if (husart->global_state == HAL_USART_STATE_RX_ACTIVE)
      {
        LL_USART_ClearFlag_UDR(p_usartx);
        return;
      }
      else
      {
        LL_USART_ClearFlag_UDR(p_usartx);
        error_code |= HAL_USART_TRANSMIT_ERROR_UDR;
      }
    }

    /* Call USART Error callback function if need be --------------------------*/
    if (error_code != 0U)
    {
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
      /* update error codes */
      husart->last_error_codes = error_code;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
      /* USART in mode Receiver ---------------------------------------------------*/
      if (((isr_flags & LL_USART_ISR_RXNE_RXFNE) != 0U)
          && (((cr1_its & LL_USART_CR1_RXNEIE_RXFNEIE) != 0U)
              || ((cr3_its & LL_USART_CR3_RXFTIE) != 0U)))
      {
        if (husart->p_rx_isr != NULL)
        {
          husart->p_rx_isr(husart);
        }
      }

#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
      /* If Overrun error occurs, or if any error occurs in DMA mode reception, consider error as blocking */
      if ((LL_USART_IsEnabledDMAReq_RX(p_usartx) != 0U)
          || ((uint32_t)(error_code & HAL_USART_RECEIVE_ERROR_ORE) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the USART state ready to be able to start again the process,
           Disable Interrupts, and disable DMA requests, if ongoing */
        USART_EndTransfer(husart);

        /* Abort the USART DMA Rx channel if enabled */
        if (LL_USART_IsEnabledDMAReq_RX(p_usartx) != 0U)
        {
          /* Disable the USART DMA Rx request if enabled */
          LL_USART_DisableDMAReq_RX(p_usartx);

          /* Abort the USART DMA Tx channel */
          if (husart->hdma_tx != NULL)
          {
            /* Set the USART Tx DMA Abort callback to NULL : no callback executed at end of DMA abort procedure */
            husart->hdma_tx->p_xfer_abort_cb = USART_DMADummy;

            (void)HAL_DMA_Abort_IT(husart->hdma_tx);
          }

          /* Abort the USART DMA Rx channel */
          if (husart->hdma_rx != NULL)
          {
            /* Set the USART Rx DMA Abort callback :
               It leads to call HAL_USART_ErrorCallback() at end of DMA abort procedure */
            husart->hdma_rx->p_xfer_abort_cb = USART_DMAAbortOnError;

            if (HAL_DMA_Abort_IT(husart->hdma_rx) != HAL_OK)
            {
              /* Call directly husart->hdma_rx->p_xfer_abort_cb function in case of error */
              husart->hdma_rx->p_xfer_abort_cb(husart->hdma_rx);
            }
          }
          else
          {
            /* Call user error callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Error Callback */
            husart->p_error_callback(husart);
#else
            /* Call legacy weak Error Callback */
            HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
          }
        }
        else
        {
          /* Call user error callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
          /* Call registered Error Callback */
          husart->p_error_callback(husart);
#else
          /* Call legacy weak Error Callback */
          HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
        }
      }
      else
#endif /* USE_HAL_USART_DMA */
      {
        /* Non Blocking error : transfer could go on. Error is notified to user through user error callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Error Callback */
        husart->p_error_callback(husart);
#else
        /* Call legacy weak Error Callback */
        HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
      }
    }
    return;

  } /* End if some error occurs */

  /* USART in mode Transmitter ------------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_TXE_TXFNF) != 0U)
      && (((cr1_its & LL_USART_CR1_TXEIE_TXFNFIE) != 0U)
          || ((cr3_its & LL_USART_CR3_TXFTIE) != 0U)))
  {
    if (husart->p_tx_isr != NULL)
    {
      husart->p_tx_isr(husart);
    }
    return;
  }

  /* USART in mode Transmitter (transmission end) -----------------------------*/
  if (((isr_flags & LL_USART_ISR_TC) != 0U) && ((cr1_its & LL_USART_CR1_TCIE) != 0U))
  {
    USART_EndTransmit_IT(husart);
    return;
  }

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  /* USART TX Fifo Empty occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_TXFE) != 0U) && ((cr1_its & LL_USART_CR1_TXFEIE) != 0U))
  {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered Tx Fifo Empty Callback */
    husart->p_tx_fifo_empty_callback(husart);
#else
    /* Call legacy weak Tx Fifo Empty Callback */
    HAL_USART_TxFifoEmptyCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    return;
  }

  /* USART RX Fifo Full occurred ----------------------------------------------*/
  if (((isr_flags & LL_USART_ISR_RXFF) != 0U) && ((cr1_its & LL_USART_CR1_RXFFIE) != 0U))
  {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered Rx Fifo Full Callback */
    husart->p_rx_fifo_full_callback(husart);
#else
    /* Call legacy weak Rx Fifo Full Callback */
    HAL_USART_RxFifoFullCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    return;
  }
#endif /* USE_HAL_USART_FIFO */
}

/**
  * @}
  */
/** @addtogroup USART_Exported_Functions_Group13
  * @{
  This subsection provides the default weak callbacks of the USARTx instance.
  Refer to HAL_USART_IRQHandler() documentation to get the details of which callback is triggered for each process
  functions. One can refer to the "How to use the USART HAL module driver" section to find the association between
  callbacks, registration function and default callback values. Here is the table of the default weak callbacks:

  Callback name               | Default value
  ----------------------------| -----------------------------------
  TxHalfCpltCallback          | HAL_USART_TxHalfCpltCallback()
  TxCpltCallback              | HAL_USART_TxCpltCallback()
  RxHalfCpltCallback          | HAL_USART_RxHalfCpltCallback()
  RxCpltCallback              | HAL_USART_RxCpltCallback()
  ErrorCallback               | HAL_USART_ErrorCallback()
  AbortCpltCallback           | HAL_USART_AbortCpltCallback()
  TxRxCpltCallback            | HAL_USART_TxRxCpltCallback()
  RxFifoFullCallback          | HAL_USART_RxFifoFullCallback()
  TxFifoEmptyCallback         | HAL_USART_TxFifoEmptyCallback()

  */
/**
  * @brief Tx Transfer completed callback.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_TxCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_TxCpltCallback() can be implemented in the user file.
    */
}

/**
  * @brief  Tx Half Transfer completed callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_TxHalfCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_TxHalfCpltCallback() can be implemented in the user file.
    */
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_RxCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_RxCpltCallback() can be implemented in the user file.
    */
}

/**
  * @brief  Rx Half Transfer completed callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_RxHalfCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_RxHalfCpltCallback() can be implemented in the user file
    */
}

/**
  * @brief Tx/Rx Transfers completed callback for the non-blocking process.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_TxRxCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_TxRxCpltCallback() can be implemented in the user file
    */
}

/**
  * @brief USART error callback.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_ErrorCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_ErrorCallback() can be implemented in the user file.
    */
}

/**
  * @brief  USART Abort Complete callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_AbortCpltCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_AbortCpltCallback() can be implemented in the user file.
    */
}
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  USART RX Fifo full callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_RxFifoFullCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_RxFifoFullCallback() can be implemented in the user file.
    */
}

/**
  * @brief  USART TX Fifo empty callback.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
__WEAK void HAL_USART_TxFifoEmptyCallback(hal_usart_handle_t *husart)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(husart);

  /** @warning This function must not be modified, when the callback is needed,
    *          the HAL_USART_TxFifoEmptyCallback() can be implemented in the user file.
    */
}
#endif /* USE_HAL_USART_FIFO */

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group9
  * @{
    A set of functions is provided to control the states and errors:
      - HAL_USART_GetState(): Return the USART handle state.
      - HAL_USART_GetClockFreq(): Return the peripheral clock frequency.
      - HAL_USART_GetLastErrorCodes(): Return the last error of the USART handle.
  */

/**
  * @brief  Return the USART handle state.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval hal_usart_state_t USART state
  */
hal_usart_state_t HAL_USART_GetState(const hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);

  return husart->global_state;
}

/**
  * @brief  Return the peripheral clock frequency.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval uint32_t Frequency in Hz.
  * @retval 0 source clock of the USART instance not configured or not ready
  */
uint32_t HAL_USART_GetClockFreq(const hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  ASSERT_DBG_PARAM((husart != NULL));

  ASSERT_DBG_STATE(husart->global_state, HAL_USART_STATE_INIT | HAL_USART_STATE_IDLE | HAL_USART_STATE_RX_ACTIVE
                   | HAL_USART_STATE_TX_ACTIVE | HAL_USART_STATE_TX_RX_ACTIVE | HAL_USART_STATE_ABORT);

  p_usartx = USART_GET_INSTANCE(husart);

  return HAL_RCC_USART_GetKernelClkFreq(p_usartx);
}
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)

/**
  * @brief  Return the USART last errors.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval Current Last Errors Codes
  */
uint32_t HAL_USART_GetLastErrorCodes(const hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);
  return husart->last_error_codes;
}
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup USART_Exported_Functions_Group10
  * @{
  This subsection provides functions to control the bus of the USARTx instance:
    - HAL_USART_AcquireBus(): Acquire the bus
    - HAL_USART_ReleaseBus(): Release the bus.

  For multitask applications, use the bus operation functions to avoid race conditions.
  */
/**
  * @brief Acquire the current instance bus.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  timeout_ms Timeout in milliseconds for the acquire to expire.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_USART_AcquireBus(hal_usart_handle_t *husart, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((husart != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreTake(&husart->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the current instance bus.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_USART_ReleaseBus(hal_usart_handle_t *husart)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((husart != NULL));

  status = HAL_ERROR;

  if (HAL_OS_SemaphoreRelease(&husart->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /*USE_HAL_MUTEX */
#if defined (USE_HAL_USART_USER_DATA) && (USE_HAL_USART_USER_DATA == 1)

/** @addtogroup USART_Exported_Functions_Group11
  * @{
  This subsection provides functions to set user-specific data to a USARTx instance:
    - HAL_USART_SetUserData(): Set user data in the handler.
    - HAL_USART_GetUserData(): Get user data from the handler.
  */

/**
  * @brief Store user data pointer into the handle.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param p_user_data Pointer to the user data.
  */
void HAL_USART_SetUserData(hal_usart_handle_t *husart, const void *p_user_data)
{
  ASSERT_DBG_PARAM(husart != NULL);

  husart->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve user data pointer from the handle.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval Pointer to the user data.
  */
const void *HAL_USART_GetUserData(const hal_usart_handle_t *husart)
{
  ASSERT_DBG_PARAM(husart != NULL);

  return (husart->p_user_data);
}

/**
  * @}
  */
#endif /* USE_HAL_USART_USER_DATA */

/**
  * @}
  */
/** @addtogroup USART_Private_Functions USART Private Functions
  * @{
  */

/**
  * @brief  Private function to abort ongoing transfers (blocking mode).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  * @warning This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  */
static void USART_Abort(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  LL_USART_DisableIT_CR1(p_usartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_TXEIE_TXFNFIE |
                                    LL_USART_CR1_TCIE | LL_USART_CR1_RXFFIE | LL_USART_CR1_TXFEIE));
  LL_USART_DisableIT_CR3(p_usartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE | LL_USART_CR3_TXFTIE));

#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
  if (LL_USART_IsEnabledDMAReq_TX(p_usartx) != 0U)
  {
    LL_USART_DisableDMAReq_TX(p_usartx);
    /* Abort the USART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (husart->hdma_tx != NULL)
    {
      (void)HAL_DMA_Abort(husart->hdma_tx);
    }
  }
  if (LL_USART_IsEnabledDMAReq_RX(p_usartx) != 0U)
  {
    LL_USART_DisableDMAReq_RX(p_usartx);
    /* Abort the USART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (husart->hdma_rx != NULL)
    {
      (void)HAL_DMA_Abort(husart->hdma_rx);
    }
  }
#endif /* USE_HAL_USART_DMA */

  husart->rx_xfer_count = 0U;
  husart->tx_xfer_count = 0U;

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_usartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if (husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED)
  {
    LL_USART_RequestTxDataFlush(p_usartx);
  }
#endif /* USE_HAL_USART_FIFO */

  LL_USART_RequestRxDataFlush(p_usartx);

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  husart->last_error_codes = HAL_USART_ERROR_NONE;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */

}

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
/**
  * @brief  Initialize the callbacks to their default values.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
static void USART_InitCallbacksToDefault(hal_usart_handle_t *husart)
{
  /* Init the USART Callback settings */
  husart->p_tx_half_cplt_callback       = HAL_USART_TxHalfCpltCallback;    /* Legacy weak TxHalfCpltCallback        */
  husart->p_tx_cplt_callback            = HAL_USART_TxCpltCallback;        /* Legacy weak TxCpltCallback            */
  husart->p_rx_half_cplt_callback       = HAL_USART_RxHalfCpltCallback;    /* Legacy weak RxHalfCpltCallback        */
  husart->p_rx_cplt_callback            = HAL_USART_RxCpltCallback;        /* Legacy weak RxCpltCallback            */
  husart->p_tx_rx_cplt_callback         = HAL_USART_TxRxCpltCallback;      /* Legacy weak TxRxCpltCallback          */
  husart->p_error_callback              = HAL_USART_ErrorCallback;         /* Legacy weak ErrorCallback             */
  husart->p_abort_cplt_callback         = HAL_USART_AbortCpltCallback;     /* Legacy weak AbortCpltCallback         */
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  husart->p_rx_fifo_full_callback       = HAL_USART_RxFifoFullCallback;    /* Legacy weak RxFifoFullCallback        */
  husart->p_tx_fifo_empty_callback      = HAL_USART_TxFifoEmptyCallback;   /* Legacy weak TxFifoEmptyCallback       */
#endif /* USE_HAL_USART_FIFO */
}
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
/**
  * @brief  End ongoing transfer on USART peripheral (following error detection or Transfer completion).
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
static void USART_EndTransfer(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
  /* Disable TXEIE, TCIE, RXNE, RXFT, TXFT, PE and ERR (Frame error, noise error, overrun error) interrupts */
  LL_USART_DisableIT_CR1(p_usartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE | LL_USART_CR1_TXEIE_TXFNFIE |
                                    LL_USART_CR1_TCIE));
  LL_USART_DisableIT_CR3(p_usartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE | LL_USART_CR3_TXFTIE));

  husart->p_rx_isr = NULL;
  husart->p_tx_isr = NULL;

  husart->global_state = HAL_USART_STATE_IDLE;
}

/**
  * @brief  DMA USART transmit process complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMATransmitCplt(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    husart->tx_xfer_count = 0U;

    if (husart->global_state == HAL_USART_STATE_TX_ACTIVE)
    {
      /* Disable the DMA transfer for transmit request by resetting the DMAT bit
         in the USART CR3 register */
      LL_USART_DisableDMAReq_TX(p_usartx);

      LL_USART_EnableIT_TC(p_usartx);
    }
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* DMA Circular mode */
  else
  {
    if (husart->global_state == HAL_USART_STATE_TX_ACTIVE)
    {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Complete Callback */
      husart->p_tx_cplt_callback(husart);
#else
      /* Call legacy weak Tx Complete Callback */
      HAL_USART_TxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
}

/**
  * @brief  DMA USART transmit process half complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMATxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Tx Half Complete Callback */
  husart->p_tx_half_cplt_callback(husart);
#else
  /* Call legacy weak Tx Half Complete Callback */
  HAL_USART_TxHalfCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART receive process complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMAReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    husart->rx_xfer_count = 0U;

    LL_USART_DisableIT_PE(p_usartx);
    LL_USART_DisableIT_ERROR(p_usartx);

    /* Disable the DMA RX transfer for the receiver request by resetting the DMAR bit
       in USART CR3 register */
    LL_USART_DisableDMAReq_RX(p_usartx);
    /* similarly, disable the DMA TX transfer that was started to provide the
       clock to the slave device */
    LL_USART_DisableDMAReq_TX(p_usartx);

    if (husart->global_state == HAL_USART_STATE_RX_ACTIVE)
    {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Rx Complete Callback */
      husart->p_rx_cplt_callback(husart);
#else
      /* Call legacy weak Rx Complete Callback */
      HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
    /* The USART state is HAL_USART_STATE_BUSY_TX_RX */
    else
    {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Rx Complete Callback */
      husart->p_tx_rx_cplt_callback(husart);
#else
      /* Call legacy weak Tx Rx Complete Callback */
      HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
    husart->global_state = HAL_USART_STATE_IDLE;
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* DMA circular mode */
  else
  {
    if (husart->global_state == HAL_USART_STATE_RX_ACTIVE)
    {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Rx Complete Callback */
      husart->p_rx_cplt_callback(husart);
#else
      /* Call legacy weak Rx Complete Callback */
      HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
    /* The USART state is HAL_USART_STATE_BUSY_TX_RX */
    else
    {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Rx Complete Callback */
      husart->p_tx_rx_cplt_callback(husart);
#else
      /* Call legacy weak Tx Rx Complete Callback */
      HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
    }
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
}

/**
  * @brief  DMA USART receive process half complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMARxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Rx Half Complete Callback */
  husart->p_rx_half_cplt_callback(husart);
#else
  /* Call legacy weak Rx Half Complete Callback */
  HAL_USART_RxHalfCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART communication error callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMAError(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  husart->rx_xfer_count = 0U;
  husart->tx_xfer_count = 0U;

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  const hal_usart_state_t temp_state = husart->global_state;

  if ((temp_state == HAL_USART_STATE_RX_ACTIVE) || (temp_state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    husart->last_error_codes |= HAL_USART_RECEIVE_ERROR_DMA;
  }
  if ((temp_state == HAL_USART_STATE_TX_ACTIVE) || (temp_state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    husart->last_error_codes |= HAL_USART_TRANSMIT_ERROR_DMA;
  }
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
  USART_EndTransfer(husart);
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Error Callback */
  husart->p_error_callback(husart);
#else
  /* Call legacy weak Error Callback */
  HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMAAbortOnError(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  husart->rx_xfer_count = 0U;
  husart->tx_xfer_count = 0U;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Error Callback */
  husart->p_error_callback(husart);
#else
  /* Call legacy weak Error Callback */
  HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  * @warning When this callback is executed, User Abort complete callback is called only if no
  *          Abort still ongoing for Rx DMA Handle.
  */
static void USART_DMATxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  /* Check if an Abort process is still ongoing */
  if (husart->hdma_rx != NULL)
  {
    if ((husart->hdma_rx->global_state == HAL_DMA_STATE_ABORT) && (husart->hdma_rx->p_xfer_abort_cb != NULL))
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  husart->rx_xfer_count = 0U;
  husart->tx_xfer_count = 0U;

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  husart->last_error_codes = 0;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_usartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  husart->global_state = HAL_USART_STATE_IDLE;

  /* Call user Abort complete callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Complete Callback */
  husart->p_abort_cplt_callback(husart);
#else
  /* Call legacy weak Abort Complete Callback */
  HAL_USART_AbortCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  * @warning When this callback is executed, User Abort complete callback is called only if no
  *          Abort still ongoing for Tx DMA Handle.
  */
static void USART_DMARxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_usart_handle_t *husart = (hal_usart_handle_t *)(hdma->p_parent);
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  /* Check if an Abort process is still ongoing */
  if (husart->hdma_tx != NULL)
  {
    if ((husart->hdma_tx->global_state == HAL_DMA_STATE_ABORT) && (husart->hdma_tx->p_xfer_abort_cb != NULL))
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  husart->rx_xfer_count = 0U;
  husart->tx_xfer_count = 0U;

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  husart->last_error_codes = 0;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */

  /* Clear the Error flags in the ICR register */
  LL_USART_ClearFlag(p_usartx, LL_USART_ICR_ORECF | LL_USART_ICR_NECF | LL_USART_ICR_PECF | LL_USART_ICR_FECF);

  husart->global_state = HAL_USART_STATE_IDLE;

  /* Call user Abort complete callback */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Complete Callback */
  husart->p_abort_cplt_callback(husart);
#else
  /* Call legacy weak Abort Complete Callback */
  HAL_USART_AbortCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART Dummy callback to prevent a call to a null pointer on DMA side.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void USART_DMADummy(hal_dma_handle_t *hdma)
{
  STM32_UNUSED(hdma);
}

#endif /* USE_HAL_USART_DMA */
/**
  * @brief If not enabled, enables the USART instance and check acknowledge bits.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK USART Ready for processing.
  * @retval HAL_TIMEOUT USART took to long to acknowledge.
  */
hal_status_t USART_CheckEnabledState(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
  hal_status_t tmp_status = HAL_OK;
  /* Check if Instance is enabled */
  /* - If Instance is already enabled : nothing to do */
  /* - If not, enable instance and check TEACK and REACK bits if needed */
  if (LL_USART_IsEnabled(p_usartx) == 0U)
  {
    LL_USART_Enable(p_usartx);
    tmp_status = USART_CheckCommunicationReady(husart);
  }
  return tmp_status;
}

/**
  * @brief  Check acknowledge bits.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @retval HAL_OK USART Ready for processing.
  * @retval HAL_TIMEOUT USART took to long to acknowledge.
  */
hal_status_t USART_CheckCommunicationReady(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
  uint32_t count;
  /* Init tickstart for timeout management */
  /* Check if the Transmitter is enabled */
  if (LL_USART_IsEnabledDirectionTx(p_usartx) != 0U)
  {
    /** 8 is the number of required instructions cycles for the below loop statement.
      * The USART_ENABLE_TIMEOUT_MS is expressed in ms.
      */
    count = USART_ENABLE_TIMEOUT_MS * (SystemCoreClock / 8U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        /* Timeout occurred */
        return HAL_TIMEOUT;
      }
      /* Wait until TEACK flag is set */
    } while (LL_USART_IsActiveFlag_TEACK(p_usartx) == 0U);
  }

  /* Check if the Receiver is enabled */
  if (LL_USART_IsEnabledDirectionRx(p_usartx) != 0U)
  {
    /** 8 is the number of required instructions cycles for the below loop statement.
      * The USART_ENABLE_TIMEOUT_MS is expressed in ms.
      */
    count = USART_ENABLE_TIMEOUT_MS * (SystemCoreClock / 8U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        /* Timeout occurred */
        return HAL_TIMEOUT;
      }
      /* Wait until TEACK flag is set */
    } while (LL_USART_IsActiveFlag_REACK(p_usartx) == 0U);
  }
  return HAL_OK;
}

/**
  * @brief  Handle USART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  flag Specifies the USART flag to check.
  * @param  status the actual Flag status (1U or 0U).
  * @param  tick_start Tick start value
  * @param  timeout_ms timeout duration.
  * @retval HAL status
  */
static hal_status_t USART_WaitOnFlagUntilTimeout(hal_usart_handle_t *husart, uint32_t flag, uint32_t status,
                                                 uint32_t tick_start, uint32_t timeout_ms)
{
  const USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  /* Wait until flag is set */
  while (((LL_USART_READ_REG(p_usartx, ISR) & flag) == status))
  {
    /* Check for the Timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        if ((LL_USART_IsActiveFlag(p_usartx, flag) == status))
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Interrupt service routine for sending 8bit data.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note    Function called under interruption only, once
  *          interruptions have been enabled by HAL_USART_Transmit_IT() or HAL_USART_TransmitReceive_IT().
  * @note ISR function executed when FIFO mode is disabled and when the
  *          data word length is less than 9 bits long.
  * @warning The USART errors are not managed to avoid the overrun error.
  */
static void USART_TxISR_8BIT(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  if (husart->tx_xfer_count == 0U)
  {
    LL_USART_DisableIT_TXE_TXFNF(p_usartx);

    LL_USART_EnableIT_TC(p_usartx);
  }
  else
  {
    LL_USART_TransmitData8(p_usartx, *husart->p_tx_buff);
    husart->p_tx_buff++;
    husart->tx_xfer_count--;
  }
}

/**
  * @brief  Interrupt service routine for sending 16bit data.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note    Function called under interruption only, once
  *          interruptions have been enabled by HAL_USART_Transmit_IT() or HAL_USART_TransmitReceive_IT().
  * @warning The USART errors are not managed to avoid the overrun error.
  * @note ISR function executed when FIFO mode is disabled and when the
  *         data word length is 9 bits long.
  */
static void USART_TxISR_16BIT(hal_usart_handle_t *husart)
{
  const uint16_t *tmp;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  if (husart->tx_xfer_count == 0U)
  {
    LL_USART_DisableIT_TXE_TXFNF(p_usartx);

    LL_USART_EnableIT_TC(p_usartx);
  }
  else
  {
    tmp = (const uint16_t *) husart->p_tx_buff;
    LL_USART_TransmitData9(p_usartx, *tmp);
    husart->p_tx_buff += 2U;
    husart->tx_xfer_count--;
  }
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  Interrupt service routine for sending 8bit data using FIFO.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note    Function called under interruption only, once
  *          interruptions have been enabled by HAL_USART_Transmit_IT() or HAL_USART_TransmitReceive_IT().
  * @warning The USART errors are not managed to avoid the overrun error.
  * @note ISR function executed when FIFO mode is enabled and when the
  *          data word length is less than 9 bits long.
  */
static void USART_TxISR_8BIT_FIFOEN(hal_usart_handle_t *husart)
{
  uint16_t nb_tx_data;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  for (nb_tx_data = husart->nb_tx_data_to_process ; nb_tx_data > 0U ; nb_tx_data--)
  {
    if (husart->tx_xfer_count == 0U)
    {
      LL_USART_DisableIT_TXFT(p_usartx);

      LL_USART_EnableIT_TC(p_usartx);
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
      /* if Tx FIFO empty Optional IT has been activated, check if we can call the callback */
      if (LL_USART_IsEnabledIT_TXFE(p_usartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_TXFE(p_usartx) != 0U)
        {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
          /* Call registered Tx Fifo Empty Callback */
          husart->p_tx_fifo_empty_callback(husart);
#else
          /* Call legacy weak Tx Fifo Empty Callback */
          HAL_USART_TxFifoEmptyCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_TXFE(p_usartx);
        LL_USART_ClearFlag_TXFE(p_usartx);
      }
#endif /* USE_HAL_USART_FIFO */
      break;
    }
    else if (LL_USART_IsActiveFlag_TXE_TXFNF(p_usartx) != 0U)
    {
      LL_USART_TransmitData8(p_usartx, *husart->p_tx_buff);
      husart->p_tx_buff++;
      husart->tx_xfer_count--;
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
  * @brief  Interrupt service routine for sending 16bit data using FIFO.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note    Function called under interruption only, once
  *          interruptions have been enabled by HAL_USART_Transmit_IT() or HAL_USART_TransmitReceive_IT().
  * @warning The USART errors are not managed to avoid the overrun error.
  * @note ISR function executed when FIFO mode is enabled and when the
  *         data word length is 9 bits long.
  */
static void USART_TxISR_16BIT_FIFOEN(hal_usart_handle_t *husart)
{
  const uint16_t *tmp;
  uint16_t  nb_tx_data;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  for (nb_tx_data = husart->nb_tx_data_to_process ; nb_tx_data > 0U ; nb_tx_data--)
  {
    if (husart->tx_xfer_count == 0U)
    {
      LL_USART_DisableIT_TXFT(p_usartx);

      LL_USART_EnableIT_TC(p_usartx);
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
      /* if Tx FIFO empty Optional IT has been activated, check if we can call the callback */
      if (LL_USART_IsEnabledIT_TXFE(p_usartx) != 0U)
      {
        if (LL_USART_IsActiveFlag_TXFE(p_usartx) != 0U)
        {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
          /* Call registered Tx Fifo Empty Callback */
          husart->p_tx_fifo_empty_callback(husart);
#else
          /* Call legacy weak Tx Fifo Empty Callback */
          HAL_USART_TxFifoEmptyCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
        }
        LL_USART_DisableIT_TXFE(p_usartx);
        LL_USART_ClearFlag_TXFE(p_usartx);
      }
#endif /* USE_HAL_USART_FIFO */
      break;
    }
    else if (LL_USART_IsActiveFlag_TXE_TXFNF(p_usartx) != 0U)
    {
      tmp = (const uint16_t *) husart->p_tx_buff;
      LL_USART_TransmitData9(p_usartx, *tmp);
      husart->p_tx_buff += 2U;
      husart->tx_xfer_count--;
    }
    else
    {
      /* Nothing to do */
    }
  }
}
#endif /* USE_HAL_USART_FIFO */

/**
  * @brief  Interrupt service routine for receiving 8bit data.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Receive_IT() or HAL_USART_TransmitReceive_IT().
  * @note ISR function executed when FIFO mode is disabled and when the
  *          data word length is less than 9 bits long.
  */
static void USART_RxISR_8BIT(hal_usart_handle_t *husart)
{
  const hal_usart_state_t state = husart->global_state;
  uint32_t tx_data_count;
  uint32_t tx_ftie;
  uint16_t uh_mask = husart->rdr_register_mask;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  if ((state == HAL_USART_STATE_RX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    *husart->p_rx_buff = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_usartx) & uh_mask);
    husart->p_rx_buff++;
    husart->rx_xfer_count--;

    if (husart->rx_xfer_count == 0U)
    {
      /* Disable the USART Parity Error Interrupt and RXNE interrupt*/
      LL_USART_DisableIT_CR1(p_usartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE));

      /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
      LL_USART_DisableIT_ERROR(p_usartx);

      /* Clear RxISR function pointer */
      husart->p_rx_isr = NULL;

      tx_ftie = LL_USART_IsEnabledIT_TXFT(p_usartx);
      tx_data_count = husart->tx_xfer_count;

      if (state == HAL_USART_STATE_RX_ACTIVE)
      {
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
        /* Clear SPI slave underrun flag and discard transmit data */
        if (husart->usart_mode == HAL_USART_MODE_SLAVE)
        {
          LL_USART_ClearFlag_UDR(p_usartx);
          LL_USART_RequestTxDataFlush(p_usartx);
        }
#endif /* USE_HAL_USART_FIFO */

        /* Rx process is completed, restore husart->global_state to Idle */
        husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Rx Complete Callback */
        husart->p_rx_cplt_callback(husart);
#else
        /* Call legacy weak Rx Complete Callback */
        HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
      }
      else if ((LL_USART_IsEnabledIT_TC(p_usartx) == 0U) && (tx_ftie == 0U) && (tx_data_count == 0U))
      {
        /* TxRx process is completed, restore husart->global_state to Idle */
        husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Tx Rx Complete Callback */
        husart->p_tx_rx_cplt_callback(husart);
#else
        /* Call legacy weak Tx Rx Complete Callback */
        HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
      }
      else
      {
        /* Nothing to do */
      }
    }
    else if ((state == HAL_USART_STATE_RX_ACTIVE) && (husart->usart_mode == HAL_USART_MODE_MASTER))
    {
      /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
      LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
  * @brief  Interrupt service routine for receiving 16bit data.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Receive_IT() or HAL_USART_TransmitReceive_IT().
  * @note ISR function executed when FIFO mode is disabled and when the
  *          data word length is 9 bits long.
  */
static void USART_RxISR_16BIT(hal_usart_handle_t *husart)
{
  const hal_usart_state_t state = husart->global_state;
  uint32_t tx_data_count;
  uint32_t tx_ftie;
  uint16_t *tmp;
  uint16_t uh_mask = husart->rdr_register_mask;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  if ((state == HAL_USART_STATE_RX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    tmp = (uint16_t *) husart->p_rx_buff;
    *tmp = (LL_USART_ReceiveData9(p_usartx) & uh_mask);
    husart->p_rx_buff += 2U;
    husart->rx_xfer_count--;

    if (husart->rx_xfer_count == 0U)
    {
      /* Disable the USART Parity Error Interrupt and RXNE interrupt*/
      LL_USART_DisableIT_CR1(p_usartx, (LL_USART_CR1_RXNEIE_RXFNEIE | LL_USART_CR1_PEIE));

      /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
      LL_USART_DisableIT_ERROR(p_usartx);

      /* Clear p_rx_isr function pointer */
      husart->p_rx_isr = NULL;

      tx_ftie = LL_USART_IsEnabledIT_TXFT(p_usartx);
      tx_data_count = husart->tx_xfer_count;

      if (state == HAL_USART_STATE_RX_ACTIVE)
      {
        /* Clear SPI slave underrun flag and discard transmit data */
        if (husart->usart_mode == HAL_USART_MODE_SLAVE)
        {
          LL_USART_ClearFlag_UDR(p_usartx);
          LL_USART_RequestTxDataFlush(p_usartx);
        }

        /* Rx process is completed, restore husart->global_state to Idle */
        husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Rx Complete Callback */
        husart->p_rx_cplt_callback(husart);
#else
        /* Call legacy weak Rx Complete Callback */
        HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
      }
      else if ((LL_USART_IsEnabledIT_TC(p_usartx) == 0U) && (tx_ftie == 0U) && (tx_data_count == 0U))
      {
        /* TxRx process is completed, restore husart->global_state to Idle */
        husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Tx Rx Complete Callback */
        husart->p_tx_rx_cplt_callback(husart);
#else
        /* Call legacy weak Tx Rx Complete Callback */
        HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
      }
      else
      {
        /* Nothing to do */
      }
    }
    else if ((state == HAL_USART_STATE_RX_ACTIVE) && (husart->usart_mode == HAL_USART_MODE_MASTER))
    {
      /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
      LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/**
  * @brief  Interrupt service routine for receiving 8bit data using FIFO.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Receive_IT() or HAL_USART_TransmitReceive_IT().
  * @note ISR function executed when FIFO mode is enabled and when the
  *          data word length is less than 9 bits long.
  */
static void USART_RxISR_8BIT_FIFOEN(hal_usart_handle_t *husart)
{
  hal_usart_state_t state = husart->global_state;
  uint32_t tx_data_count;
  uint32_t rx_data_count;
  uint32_t tx_ftie;
  uint16_t uh_mask = husart->rdr_register_mask;
  uint16_t nb_rx_data;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  /* Check that a Rx process is ongoing */
  if ((state == HAL_USART_STATE_RX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    rx_data_count = husart->rx_xfer_count;
    for (nb_rx_data = husart->nb_rx_data_to_process ; nb_rx_data > 0U ; nb_rx_data--)
    {
      if (LL_USART_IsActiveFlag_RXNE_RXFNE(p_usartx) != 0U)
      {
        *husart->p_rx_buff = (uint8_t)((uint16_t)LL_USART_ReceiveData8(p_usartx) & uh_mask);
        husart->p_rx_buff++;
        husart->rx_xfer_count--;

        if (husart->rx_xfer_count == 0U)
        {
          /* Disable the USART Parity Error Interrupt */
          LL_USART_DisableIT_PE(p_usartx);

          /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error)
             and RX FIFO Threshold interrupt */
          LL_USART_DisableIT_CR3(p_usartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));

          /* Clear p_rx_isr function pointer */
          husart->p_rx_isr = NULL;

          tx_ftie = LL_USART_IsEnabledIT_TXFT(p_usartx);
          tx_data_count = husart->tx_xfer_count;

          if (state == HAL_USART_STATE_RX_ACTIVE)
          {
            /* Clear SPI slave underrun flag and discard transmit data */
            if (husart->usart_mode == HAL_USART_MODE_SLAVE)
            {
              LL_USART_ClearFlag_UDR(p_usartx);
              LL_USART_RequestTxDataFlush(p_usartx);
            }
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
            /* if Rx FIFO full Optional IT has been activated, check if we can call the callback */
            if (LL_USART_IsEnabledIT_RXFF(p_usartx) != 0U)
            {
              if (LL_USART_IsActiveFlag_RXFF(p_usartx) != 0U)
              {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
                /* Call registered Rx FIFO Full Callback */
                husart->p_rx_fifo_full_callback(husart);
#else
                /* Call legacy weak Rx FIFO Full Callback */
                HAL_USART_RxFifoFullCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
              }
            }
#endif /* USE_HAL_USART_FIFO */
            /* Rx process is completed, restore husart->global_state to Idle */
            husart->global_state = HAL_USART_STATE_IDLE;
            state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Rx Complete Callback */
            husart->p_rx_cplt_callback(husart);
#else
            /* Call legacy weak Rx Complete Callback */
            HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
          }
          else if ((LL_USART_IsEnabledIT_TC(p_usartx) == 0U) && (tx_ftie == 0U) && (tx_data_count == 0U))
          {
            /* TxRx process is completed, restore husart->global_state to Idle */
            husart->global_state = HAL_USART_STATE_IDLE;
            state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Tx Rx Complete Callback */
            husart->p_tx_rx_cplt_callback(husart);
#else
            /* Call legacy weak Tx Rx Complete Callback */
            HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
          }
          else
          {
            /* Nothing to do */
          }
        }
        else if ((state == HAL_USART_STATE_RX_ACTIVE) && (husart->usart_mode == HAL_USART_MODE_MASTER))
        {
          /* As data to process has already been transmitted by the reception start (but not yet deducted from count)
             comparison must be done against 2*data_to_process */
          if (rx_data_count >= ((uint32_t)(husart->nb_rx_data_to_process) << 1))
          {
            /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
            LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
          }
        }
        else
        {
          /* Nothing to do */
        }
      }
    }

    /* When remaining number of bytes to receive is less than the RX FIFO
    threshold, next incoming frames are processed as if FIFO mode was
    disabled (i.e. one interrupt per received frame).
    */
    rx_data_count = husart->rx_xfer_count;
    if (((rx_data_count != 0U)) && (rx_data_count < husart->nb_rx_data_to_process))
    {
      /* Disable the USART RXFT interrupt*/
      LL_USART_DisableIT_RXFT(p_usartx);

      /* Update the RxISR function pointer */
      husart->p_rx_isr = USART_RxISR_8BIT;

      /* Enable the USART Data Register Not Empty interrupt */
      LL_USART_EnableIT_RXNE_RXFNE(p_usartx);

      if ((husart->tx_xfer_count == 0U) && (husart->usart_mode == HAL_USART_MODE_MASTER))
      {
        /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
        LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
      }
    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    LL_USART_RequestRxDataFlush(p_usartx);
  }
}

/**
  * @brief  Interrupt service routine for receiving 16bit data using FIFO.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Receive_IT() or HAL_USART_TransmitReceive_IT().
  * @note ISR function executed when FIFO mode is enabled and when the
  *          data word length is 9 bits long.
  */
static void USART_RxISR_16BIT_FIFOEN(hal_usart_handle_t *husart)
{
  hal_usart_state_t state = husart->global_state;
  uint32_t tx_data_count;
  uint32_t rx_data_count;
  uint32_t tx_ftie;
  uint16_t *tmp;
  uint16_t uh_mask = husart->rdr_register_mask;
  uint16_t nb_rx_data;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);

  /* Check that a Tx process is ongoing */
  if ((state == HAL_USART_STATE_RX_ACTIVE) || (state == HAL_USART_STATE_TX_RX_ACTIVE))
  {
    rx_data_count = husart->rx_xfer_count;
    for (nb_rx_data = husart->nb_rx_data_to_process ; nb_rx_data > 0U ; nb_rx_data--)
    {
      if (LL_USART_IsActiveFlag_RXNE_RXFNE(p_usartx) != 0U)
      {
        tmp = (uint16_t *) husart->p_rx_buff;
        *tmp = (LL_USART_ReceiveData9(p_usartx) & uh_mask);
        husart->p_rx_buff += 2U;
        husart->rx_xfer_count--;

        if (husart->rx_xfer_count == 0U)
        {
          /* Disable the USART Parity Error Interrupt */
          LL_USART_DisableIT_PE(p_usartx);

          /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error)
             and RX FIFO Threshold interrupt */
          LL_USART_DisableIT_CR3(p_usartx, (LL_USART_CR3_EIE | LL_USART_CR3_RXFTIE));

          /* Clear p_rx_isr function pointer */
          husart->p_rx_isr = NULL;

          tx_ftie = LL_USART_IsEnabledIT_TXFT(p_usartx);
          tx_data_count = husart->tx_xfer_count;

          if (state == HAL_USART_STATE_RX_ACTIVE)
          {
            /* Clear SPI slave underrun flag and discard transmit data */
            if (husart->usart_mode == HAL_USART_MODE_SLAVE)
            {
              LL_USART_ClearFlag_UDR(p_usartx);
              LL_USART_RequestTxDataFlush(p_usartx);
            }
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
            /* if Rx FIFO full Optional IT has been activated, check if we can call the callback */
            if (LL_USART_IsEnabledIT_RXFF(p_usartx) != 0U)
            {
              if (LL_USART_IsActiveFlag_RXFF(p_usartx) != 0U)
              {
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
                /* Call registered Rx FIFO Full Callback */
                husart->p_rx_fifo_full_callback(husart);
#else
                /* Call legacy weak Rx FIFO Full Callback */
                HAL_USART_RxFifoFullCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
              }
            }
#endif /* USE_HAL_USART_FIFO */
            /* Rx process is completed, restore husart->global_state to Idle */
            husart->global_state = HAL_USART_STATE_IDLE;
            state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Rx Complete Callback */
            husart->p_rx_cplt_callback(husart);
#else
            /* Call legacy weak Rx Complete Callback */
            HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
          }
          else if ((LL_USART_IsEnabledIT_TC(p_usartx) == 0U) && (tx_ftie == 0U) && (tx_data_count == 0U))
          {
            /* TxRx process is completed, restore husart->global_state to Idle */
            husart->global_state = HAL_USART_STATE_IDLE;
            state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Tx Rx Complete Callback */
            husart->p_tx_rx_cplt_callback(husart);
#else
            /* Call legacy weak Tx Rx Complete Callback */
            HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
          }
          else
          {
            /* Nothing to do */
          }
        }
        else if ((state == HAL_USART_STATE_RX_ACTIVE) && (husart->usart_mode == HAL_USART_MODE_MASTER))
        {
          /* As data to process has already been transmitted by the reception start (but not yet deducted from count)
             comparison must be done against 2*data_to_process */
          if (rx_data_count >= ((uint32_t)(husart->nb_rx_data_to_process) << 1))
          {
            /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
            LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
          }
        }
        else
        {
          /* Nothing to do */
        }
      }
    }

    /* When remaining number of bytes to receive is less than the RX FIFO
    threshold, next incoming frames are processed as if FIFO mode was
    disabled (i.e. one interrupt per received frame).
    */
    rx_data_count = husart->rx_xfer_count;
    if (((rx_data_count != 0U)) && (rx_data_count < husart->nb_rx_data_to_process))
    {
      /* Disable the USART RXFT interrupt*/
      LL_USART_DisableIT_RXFT(p_usartx);

      /* Update the RxISR function pointer */
      husart->p_rx_isr = USART_RxISR_16BIT;

      /* Enable the USART Data Register Not Empty interrupt */
      LL_USART_EnableIT_RXNE_RXFNE(p_usartx);

      if ((husart->tx_xfer_count == 0U) && (husart->usart_mode == HAL_USART_MODE_MASTER))
      {
        /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
        LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
      }
    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    LL_USART_RequestRxDataFlush(p_usartx);
  }
}

/**
  * @brief Calculate the number of data to process in RX/TX ISR.
  * @param husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @note The RX FIFO depth and the TX FIFO depth are extracted from the USART configuration registers.
  */
static void USART_SetNbDataToProcess(hal_usart_handle_t *husart)
{
  uint8_t rx_fifo_depth;
  uint8_t tx_fifo_depth;
  uint8_t rx_fifo_threshold;
  uint8_t tx_fifo_threshold;
  static const uint8_t numerator[]   = {1U, 1U, 1U, 3U, 7U, 1U, 0U, 0U};
  static const uint8_t denominator[] = {8U, 4U, 2U, 4U, 8U, 1U, 1U, 1U};
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  if (husart->fifo_mode == HAL_USART_FIFO_MODE_DISABLED)
  {
    husart->nb_tx_data_to_process = 1U;
    husart->nb_rx_data_to_process = 1U;
  }
  else
  {
    rx_fifo_depth = RX_FIFO_DEPTH;
    tx_fifo_depth = TX_FIFO_DEPTH;
    rx_fifo_threshold = (uint8_t)LL_USART_GetRXFIFOThreshold(p_usartx);
    tx_fifo_threshold = (uint8_t)LL_USART_GetTXFIFOThreshold(p_usartx);
    husart->nb_tx_data_to_process = ((uint16_t)tx_fifo_depth * numerator[tx_fifo_threshold]) /
                                    (uint16_t)denominator[tx_fifo_threshold];
    husart->nb_rx_data_to_process = ((uint16_t)rx_fifo_depth * numerator[rx_fifo_threshold]) /
                                    (uint16_t)denominator[rx_fifo_threshold];
  }
}

#endif /* USE_HAL_USART_FIFO */

/**
  * @brief  Wrap up transmission in non-blocking mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  */
static void USART_EndTransmit_IT(hal_usart_handle_t *husart)
{
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);

  /* Disable the USART Transmit Complete Interrupt */
  LL_USART_DisableIT_TC(p_usartx);

  /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
  LL_USART_DisableIT_ERROR(p_usartx);

  /* Clear p_tx_isr function pointer */
  husart->p_tx_isr = NULL;

  if (husart->global_state == HAL_USART_STATE_TX_ACTIVE)
  {
    /* Clear overrun flag and discard the received data */
    LL_USART_ClearFlag_ORE(p_usartx);
    LL_USART_RequestRxDataFlush(p_usartx);

    /* Tx process is completed */
    husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call p_tx_cplt_callback Tx Complete Callback */
    husart->p_tx_cplt_callback(husart);
#else
    /* Call legacy weak Tx Complete Callback */
    HAL_USART_TxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
  }
  else if (husart->rx_xfer_count == 0U)
  {
    /* TxRx process is completed */
    husart->global_state = HAL_USART_STATE_IDLE;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered Tx Rx Complete Callback */
    husart->p_tx_rx_cplt_callback(husart);
#else
    /* Call legacy weak Tx Rx Complete Callback */
    HAL_USART_TxRxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Start Transmit operation in interrupt mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing transmission in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Transmit started in IT mode.
  */
hal_status_t USART_Start_Transmit_IT(hal_usart_handle_t *husart, const uint8_t *p_data, uint32_t size,
                                     uint32_t interrupts)
{
  uint32_t reg_temp;
  uint32_t nine_bits_data;
  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);
#if !defined(USE_HAL_USART_FIFO) || (USE_HAL_USART_FIFO == 0)
  STM32_UNUSED(interrupts);
#endif /* USE_HAL_USART_FIFO */
  nine_bits_data = 0U;

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    nine_bits_data = 1U;
  }

  husart->p_tx_buff     = p_data;
  husart->tx_xfer_size  = size;
  husart->tx_xfer_count = size;
  husart->p_tx_isr        = NULL;

  /* The USART Error Interrupts: (Frame error, noise error, overrun error)
  are not managed by the USART Transmit Process to avoid the overrun interrupt.
  Note that when the usart mode is configured for transmit and receive ( State equal to "HAL_USART_STATE_TX_RX_ACTIVE")
  it is recommended to configure the usart mode to "HAL_USART_STATE_TX_ACTIVE",
  to benefit for the frame error and noise interrupts  */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if (husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED)
  {
    if (nine_bits_data != 0U)
    {
      husart->p_tx_isr = USART_TxISR_16BIT_FIFOEN;
    }
    else
    {
      husart->p_tx_isr = USART_TxISR_8BIT_FIFOEN;
    }
    LL_USART_EnableIT_TXFT(p_usartx);
  }
  else
#endif /* USE_HAL_USART_FIFO */
  {
    if (nine_bits_data != 0U)
    {
      husart->p_tx_isr = USART_TxISR_16BIT;
    }
    else
    {
      husart->p_tx_isr = USART_TxISR_8BIT;
    }
    LL_USART_EnableIT_TXE_TXFNF(p_usartx);
  }
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if ((interrupts & HAL_USART_OPT_TX_IT_FIFO_EMPTY) == HAL_USART_OPT_TX_IT_FIFO_EMPTY)
  {
    LL_USART_EnableIT_TXFE(p_usartx);
  }
#endif /* USE_HAL_USART_FIFO */
  return HAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Receive started in IT mode.
  */
hal_status_t USART_Start_Receive_IT(hal_usart_handle_t *husart, uint8_t *p_data, uint32_t size, uint32_t interrupts)
{
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  uint16_t nb_dummy_data;
#endif /* USE_HAL_USART_FIFO */
  uint32_t reg_temp;
  uint32_t nine_bits_data;
#if !defined(USE_HAL_USART_FIFO) || (USE_HAL_USART_FIFO == 0)
  STM32_UNUSED(interrupts);
#endif /* USE_HAL_USART_FIFO */

  USART_TypeDef *p_usartx = USART_GET_INSTANCE(husart);


  nine_bits_data = 0U;
  husart->p_rx_buff     = p_data;
  husart->rx_xfer_size  = size;
  husart->rx_xfer_count = size;
  husart->p_rx_isr        = NULL;

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    nine_bits_data = 1U;
  }

  if (USART_RDRMaskComputation(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_EnableIT_ERROR(p_usartx);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if ((husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED) && (size >= husart->nb_rx_data_to_process))
  {
    if (nine_bits_data != 0U)
    {
      husart->p_rx_isr = USART_RxISR_16BIT_FIFOEN;
    }
    else
    {
      husart->p_rx_isr = USART_RxISR_8BIT_FIFOEN;
    }

    if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
    {
      LL_USART_EnableIT_PE(p_usartx);
    }
    LL_USART_EnableIT_RXFT(p_usartx);
  }
  else
#endif /* USE_HAL_USART_FIFO */
  {
    if (nine_bits_data != 0U)
    {
      husart->p_rx_isr = USART_RxISR_16BIT;
    }
    else
    {
      husart->p_rx_isr = USART_RxISR_8BIT;
    }

    if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
    {
      LL_USART_EnableIT_PE(p_usartx);
    }
    LL_USART_EnableIT_RXNE_RXFNE(p_usartx);
  }
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if ((interrupts & HAL_USART_OPT_RX_IT_FIFO_FULL) == HAL_USART_OPT_RX_IT_FIFO_FULL)
  {
    LL_USART_EnableIT_RXFF(p_usartx);
  }
#endif /* USE_HAL_USART_FIFO */

  if (husart->usart_mode == HAL_USART_MODE_MASTER)
  {
    /* Send dummy data in order to generate the clock for the Slave to send the next data.
       When FIFO mode is disabled only one data must be transferred.
       When FIFO mode is enabled data must be transmitted until the RX FIFO reaches its threshold.
    */
#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
    if ((husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED) && (size >= husart->nb_rx_data_to_process))
    {
      for (nb_dummy_data = husart->nb_rx_data_to_process ; nb_dummy_data > 0U ; nb_dummy_data--)
      {
        LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
      }
    }
    else
#endif /* USE_HAL_USART_FIFO */
    {
      LL_USART_TransmitData8(p_usartx, USART_DUMMY_DATA);
    }
  }
  return HAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_tx_data Pointer to rx data buffer (u8 or u16 data elements).
  * @param  p_rx_data Pointer to rx data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing transmission/reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Receive started in IT mode.
  */
hal_status_t USART_Start_TransmitReceive_IT(hal_usart_handle_t *husart, const uint8_t *p_tx_data, uint8_t *p_rx_data,
                                            uint32_t size, uint32_t interrupts)
{
  USART_TypeDef *p_usartx;
  uint32_t reg_temp;
  uint32_t nine_bits_data;
#if !defined(USE_HAL_USART_FIFO) || (USE_HAL_USART_FIFO == 0)
  STM32_UNUSED(interrupts);
#endif /* USE_HAL_USART_FIFO */

  p_usartx = USART_GET_INSTANCE(husart);

  nine_bits_data = 0U;
  husart->p_rx_buff     = p_rx_data;
  husart->rx_xfer_size  = size;
  husart->rx_xfer_count = size;
  husart->p_tx_buff     = p_tx_data;
  husart->tx_xfer_size  = size;
  husart->tx_xfer_count = size;

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  if (((reg_temp & USART_CR1_M) == LL_USART_DATAWIDTH_9_BIT) && ((reg_temp & USART_CR1_PCE) == LL_USART_PARITY_NONE))
  {
    nine_bits_data = 1U;
  }

  if (USART_RDRMaskComputation(husart) != HAL_OK)
  {
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_EnableIT_ERROR(p_usartx);
  if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
  {
    LL_USART_EnableIT_PE(p_usartx);
  }

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  /* Configure TxRx interrupt processing */
  if ((husart->fifo_mode == HAL_USART_FIFO_MODE_ENABLED) && (size >= husart->nb_rx_data_to_process))
  {
    /* Set the Rx ISR function pointer according to the data word length */
    if (nine_bits_data != 0U)
    {
      husart->p_tx_isr = USART_TxISR_16BIT_FIFOEN;
      husart->p_rx_isr = USART_RxISR_16BIT_FIFOEN;
    }
    else
    {
      husart->p_tx_isr = USART_TxISR_8BIT_FIFOEN;
      husart->p_rx_isr = USART_RxISR_8BIT_FIFOEN;
    }
    LL_USART_EnableIT_RXFT(p_usartx);
    LL_USART_EnableIT_TXFT(p_usartx);
  }
  else
#endif /* USE_HAL_USART_FIFO */
  {
    if (nine_bits_data != 0U)
    {
      husart->p_tx_isr = USART_TxISR_16BIT;
      husart->p_rx_isr = USART_RxISR_16BIT;
    }
    else
    {
      husart->p_tx_isr = USART_TxISR_8BIT;
      husart->p_rx_isr = USART_RxISR_8BIT;
    }
    LL_USART_EnableIT_RXNE_RXFNE(p_usartx);
    LL_USART_EnableIT_TXE_TXFNF(p_usartx);
  }

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  if ((interrupts & HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY) == HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY)
  {
    LL_USART_EnableIT_TXFE(p_usartx);
  }
  if ((interrupts & HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL) == HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL)
  {
    LL_USART_EnableIT_RXFF(p_usartx);
  }
#endif /* USE_HAL_USART_FIFO */
  return HAL_OK;
}

#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)

/**
  * @brief  Start Transmit operation in DMA mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing transmission in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t USART_Start_Transmit_DMA(hal_usart_handle_t *husart, const uint8_t *p_data, uint32_t size,
                                      uint32_t interrupts)
{
  USART_TypeDef *p_usartx;
  uint32_t interrupts_dma;

  p_usartx = USART_GET_INSTANCE(husart);
  husart->p_tx_buff     = p_data;
  husart->tx_xfer_size  = size;
  husart->tx_xfer_count = size;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (interrupts == HAL_USART_OPT_DMA_TX_IT_SILENT)
  {
    interrupts_dma = HAL_USART_OPT_DMA_TX_IT_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    interrupts_dma = (interrupts & HAL_USART_OPT_DMA_TX_IT_HT);
  }

  /* Set the USART DMA transfer complete callback */
  husart->hdma_tx->p_xfer_cplt_cb = USART_DMATransmitCplt;

  /* Set the USART DMA Half transfer complete callback */
  husart->hdma_tx->p_xfer_halfcplt_cb = USART_DMATxHalfCplt;

  /* Set the DMA error callback */
  husart->hdma_tx->p_xfer_error_cb = USART_DMAError;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(husart->hdma_tx, (uint32_t)husart->p_tx_buff, (uint32_t)&p_usartx->TDR,
                                     size, interrupts_dma) != HAL_OK)
  {
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
    husart->last_error_codes |= HAL_USART_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  LL_USART_ClearFlag_TC(p_usartx);
  LL_USART_EnableDMAReq_TX(p_usartx);

  return HAL_OK;
}

/**
  * @brief  Start Receive operation in DMA mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t USART_Start_Receive_DMA(hal_usart_handle_t *husart, uint8_t *p_data, uint32_t size, uint32_t interrupts)
{
  uint32_t reg_temp;
  uint32_t interrupts_dma;
  USART_TypeDef *p_usartx;

  p_usartx = USART_GET_INSTANCE(husart);
  husart->p_rx_buff     = p_data;
  husart->rx_xfer_size  = size;
  husart->p_tx_buff     = p_data;
  husart->tx_xfer_size  = size;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (interrupts == HAL_USART_OPT_DMA_RX_IT_SILENT)
  {
    interrupts_dma = HAL_USART_OPT_DMA_RX_IT_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    interrupts_dma = (interrupts & HAL_USART_OPT_DMA_RX_IT_HT);
  }
  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  /* Set the USART DMA Rx transfer complete callback */
  husart->hdma_rx->p_xfer_cplt_cb = USART_DMAReceiveCplt;

  /* Set the USART DMA Half transfer complete callback */
  husart->hdma_rx->p_xfer_halfcplt_cb = USART_DMARxHalfCplt;

  /* Set the USART DMA Rx transfer error callback */
  husart->hdma_rx->p_xfer_error_cb = USART_DMAError;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(husart->hdma_rx, (uint32_t)&p_usartx->RDR, (uint32_t)husart->p_rx_buff,
                                     size, interrupts_dma) != HAL_OK)
  {
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
    husart->last_error_codes |= HAL_USART_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
    (void)HAL_DMA_Abort(husart->hdma_rx);
    husart->global_state = HAL_USART_STATE_IDLE;
    return HAL_ERROR;
  }

  if (husart->usart_mode == HAL_USART_MODE_MASTER)
  {
    /* Enable the USART transmit DMA channel: the transmit channel is used in order
        to generate in the non-blocking mode the clock to the slave device */

    /* Set the USART DMA Error callback to Null */
    /* Need to set Tx Complete callback because the DMA does not check the callback before calling it */
    if (husart->hdma_tx != NULL)
    {
      husart->hdma_tx->p_xfer_error_cb = USART_DMADummy;
      husart->hdma_tx->p_xfer_cplt_cb = USART_DMADummy;

      if (HAL_DMA_StartPeriphXfer_IT_Opt(husart->hdma_tx, (uint32_t)husart->p_tx_buff, (uint32_t)&p_usartx->TDR,
                                         size, HAL_DMA_OPT_IT_NONE) != HAL_OK)
      {
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
        husart->last_error_codes |= HAL_USART_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
        (void)HAL_DMA_Abort(husart->hdma_rx);
        husart->global_state = HAL_USART_STATE_IDLE;
        return HAL_ERROR;
      }
    }
    else
    {
      (void)HAL_DMA_Abort(husart->hdma_rx);
      husart->global_state = HAL_USART_STATE_IDLE;
      return HAL_ERROR;
    }
  }
  LL_USART_EnableIT_ERROR(p_usartx);
  if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
  {
    LL_USART_EnableIT_PE(p_usartx);
  }
  LL_USART_EnableDMAReq_TX(p_usartx);
  LL_USART_EnableDMAReq_RX(p_usartx);

  return HAL_OK;
}


/**
  * @brief  Start Transmit Receive operation in DMA mode.
  * @param  husart Pointer to a \ref hal_usart_handle_t structure which contains the USART instance.
  * @param  p_rx_data Pointer to data buffer (u8 or u16 data elements).
  * @param  p_tx_data Pointer to data buffer (u8 or u16 data elements).
  * @param  size  Amount of data elements (u8 or u16) to be received.
  * @param  interrupts List of optional interruptions to activate.
  * @note   This function could be called by all HAL USART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  * @retval HAL_OK Receive started in DMA mode.
  * @retval HAL_ERROR DMA did not start.
  */
hal_status_t USART_Start_TransmitReceive_DMA(hal_usart_handle_t *husart, const uint8_t *p_tx_data, uint8_t *p_rx_data,
                                             uint32_t size, uint32_t interrupts)
{
  USART_TypeDef *p_usartx;
  uint32_t reg_temp;
  uint32_t interrupts_dma_rx;
  uint32_t interrupts_dma_tx;

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (interrupts == HAL_USART_OPT_DMA_TXRX_IT_SILENT)
  {
    interrupts_dma_rx = HAL_USART_OPT_DMA_TXRX_IT_SILENT;
    interrupts_dma_tx = HAL_USART_OPT_DMA_TXRX_IT_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    interrupts_dma_rx = (interrupts & HAL_USART_OPT_DMA_TXRX_RX_IT_HT);
    interrupts_dma_tx = (interrupts & HAL_USART_OPT_DMA_TXRX_TX_IT_HT);
  }

  p_usartx = USART_GET_INSTANCE(husart);
  husart->p_rx_buff     = p_rx_data;
  husart->rx_xfer_size  = size;
  husart->p_tx_buff     = p_tx_data;
  husart->tx_xfer_size  = size;

  reg_temp = LL_USART_READ_REG(p_usartx, CR1);

  if ((husart->hdma_rx != NULL) && (husart->hdma_tx != NULL))
  {
    /* Set the USART DMA Rx transfer complete callback */
    husart->hdma_rx->p_xfer_cplt_cb = USART_DMAReceiveCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdma_rx->p_xfer_halfcplt_cb = USART_DMARxHalfCplt;

    /* Set the USART DMA Tx transfer complete callback */
    husart->hdma_tx->p_xfer_cplt_cb = USART_DMATransmitCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdma_tx->p_xfer_halfcplt_cb = USART_DMATxHalfCplt;

    /* Set the USART DMA Tx transfer error callback */
    husart->hdma_tx->p_xfer_error_cb = USART_DMAError;

    /* Set the USART DMA Rx transfer error callback */
    husart->hdma_rx->p_xfer_error_cb = USART_DMAError;

    if (HAL_DMA_StartPeriphXfer_IT_Opt(husart->hdma_rx, (uint32_t)&p_usartx->RDR, (uint32_t)husart->p_rx_buff,
                                       size, interrupts_dma_rx) != HAL_OK)
    {
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
      husart->last_error_codes |= HAL_USART_RECEIVE_ERROR_DMA;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
      if (husart->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
      {
        (void)HAL_DMA_Abort(husart->hdma_rx);
      }
#endif /* USE_HAL_DMA_LINKEDLIST */
      husart->global_state = HAL_USART_STATE_IDLE;
      return HAL_ERROR;
    }

    if (HAL_DMA_StartPeriphXfer_IT_Opt(husart->hdma_tx, (uint32_t)husart->p_tx_buff, (uint32_t)&p_usartx->TDR,
                                       size, interrupts_dma_tx) != HAL_OK)
    {
      husart->global_state = HAL_USART_STATE_IDLE;
#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
      husart->last_error_codes |= HAL_USART_TRANSMIT_ERROR_DMA;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
      if (husart->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
      {
        (void)HAL_DMA_Abort(husart->hdma_tx);
      }
#endif /* USE_HAL_DMA_LINKEDLIST */
      return HAL_ERROR;
    }
  }
  LL_USART_EnableIT_ERROR(p_usartx);
  if ((reg_temp & USART_CR1_PCE) != LL_USART_PARITY_NONE)
  {
    LL_USART_EnableIT_PE(p_usartx);
  }
  LL_USART_ClearFlag_TC(p_usartx);

  LL_USART_EnableDMAReq_TX(p_usartx);
  LL_USART_EnableDMAReq_RX(p_usartx);

  return HAL_OK;
}

#endif /* USE_HAL_USART_DMA */
/**
  * @}
  */

/**
  * @}
  */
#endif /* USART1 || USART2 || USART3 || UART4 || UART5 || USART6 || UART7 */
#endif /* USE_HAL_USART_MODULE */

/**
  * @}
  */
