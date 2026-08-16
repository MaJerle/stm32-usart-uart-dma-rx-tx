/**
  ******************************************************************************
  * @file    stm32c5xx_hal_spi.c
  * @brief   SPI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions.
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

/** @addtogroup SPI
  * @{
  */
/** @defgroup SPI_Introduction SPI Introduction
  * @{

  - The **SPI** hardware abstraction layer provides a set of APIs to interface with the STM32 SPI (Serial
    Peripheral Interface) peripheral.

  - It simplifies the configuration, initialization, and management of **SPI** communication, by supporting
    various modes such as polling, interrupt, and DMA for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series.

  */
/**
  * @}
  */

/** @defgroup SPI_How_To_Use SPI How To Use
  * @{


  The serial peripheral interface (SPI) can be used to communicate with external devices
  while using the specific synchronous protocol. The SPI protocol supports half-duplex, full-duplex
  and simplex synchronous, serial communication with external devices. The interface can be configured as master
  or slave and is capable of operating in multi slave or multi master configurations.
  The device configured as master provides communication clock (SCK) to the slave device. The Slave select (SS)
  and ready (RDY) signals can be applied optionally just to setup communication with concrete slave and to assure
  it handles the data flow properly.
  The Motorola data format is used by default, but some other specific modes are supported as well.

  ## Main features
  - Full-duplex synchronous transfers on three lines
  - Half-duplex synchronous transfer on two lines (with bidirectional data line)
  - Simplex synchronous transfers on two lines (with unidirectional data line)
  - From 4-bit up to 32-bit data size selection or fixed to 8-bit multiples
  - Multi master or multi slave mode capability
  - Dual clock domain, the peripheral kernel clock is independent from the APB bus clock
  - Baud rate prescaler up to kernel frequency/2 or bypass from RCC in Master mode
  - Protection of configuration and setting
  - Hardware or software management of SS for both master and slave
  - Adjustable minimum delays between data and between SS and data flow
  - Configurable SS signal polarity and timing, MISO x MOSI swap capability
  - Programmable clock polarity and phase with optional sampling delay when reading master data
  - Programmable data order with MSB-first or LSB-first shifting
  - Programmable number of data within a transaction to control SS and CRC
  - Dedicated transmission and reception flags with interrupt capability
  - SPI Motorola and TI formats support
  - Hardware CRC feature can verify integrity of the communication at the end of transaction by:
    - Adding CRC value in Tx mode
    - Automatic CRC error checking for Rx mode
  - Error detection with interrupt capability in case of data overrun, CRC error, data
  underrun, the mode fault and frame error, depending upon the operating mode
  - Two multiples of 8-bit embedded Rx and Tx FIFOs (FIFO size depends on instance)
  - Configurable FIFO thresholds (data packing)
  - Capability to handle data streams by system DMA controller
  - Configurable behavior for slave underrun condition (support of cascaded circular
  buffers)
  - Optional status pin RDY signalizing that the slave device is ready to handle the data flow

  ## How to use
      The SPI HAL driver can be used as follows:

  - Declare a hal_spi_handle_t handle structure, for example:
      hal_spi_handle_t  hspi;

  - Initialize the SPI according to the associated handle with HAL_SPI_Init()
    Note: Enable the SPI interface clock if you have set USE_HAL_SPI_CLK_ENABLE_MODEL to HAL_CLK_ENABLE_PERIPH_ONLY
          or HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM (in those cases HAL_SPI_Init() will enable the clock).
  - Initialize the SPI low level resources:
    - Enable the SPIx interface clock
    - SPI pins configuration
      - Enable the clock for the SPI GPIOs
      - Configure these SPI pins as alternate function push-pull
    - NVIC configuration if you need to use interrupt process or DMA process
      - Configure the SPIx interrupt priority
      - Enable the NVIC SPI IRQ handle
    - DMA Configuration if you need to use DMA process
      - Declare a hal_dma_handle_t handle structure for the transmit or receive Stream/Channel
      - Enable the DMAx clock
      - Configure the DMA handle parameters
      - Configure the DMA Tx or Rx Stream/Channel
      - Associate the initialized hdma_tx handle to the hspi DMA Tx or Rx handle
      - For each DMA channel (Tx and Rx), configure the corresponding NVIC line priority and enable Tx
                    or Rx Stream/Channel
  - Set the generic configuration of the SPI with HAL_SPI_SetConfig() to choose:
    - The mode
    - The direction
    - The data width
    - The clock polarity
    - The clock phase
    - The baud rate prescaler
    - The first bit
    - The NSS pin management
  - For an advanced configuration, use the following functions:
    - HAL_SPI_SetConfigCRC() to configure the CRC feature
    - HAL_SPI_SetConfigNSS() to configure the NSS feature
    - HAL_SPI_SLAVE_SetConfigUnderrun() to configure the Underrun Detection feature
    - HAL_SPI_EnableTIMode() to enable the TI mode feature
    - HAL_SPI_MASTER_EnableReceiverAutoSuspend() to enable the Master Receiver automatic suspension feature
    - HAL_SPI_MASTER_EnableKeepIOState() to enable the Master Keep IO State feature
    - HAL_SPI_EnableMosiMisoSwap() to enable the IO Swap feature
    - HAL_SPI_EnableReadyPin() to enable the Ready Pin management feature
    - HAL_SPI_LockIOConfig() to enable the Lock of IO configuration feature
    - HAL_SPI_EnableDelayReadDataSampling() to enable the Delay Read Data Sampling feature

  ### Circular mode restriction:
  - The DMA circular mode cannot be used when the SPI is configured in these modes:
    - Master Simplex Rx
    - Master Half duplex Rx
  - The CRC feature is not managed when the DMA circular mode is enabled

  ### Callback registration:
  The compilation flag USE_HAL_SPI_REGISTER_CALLBACKS allows the user to configure dynamically
  the driver callbacks, via its own method:

  Callback name               | Default value                       | Callback registration function
  ----------------------------| ----------------------------------- | ---------------------------
  ErrorCallback               | HAL_SPI_ErrorCallback()             | HAL_SPI_RegisterErrorCallback()
  TxCpltCallback              | HAL_SPI_TxCpltCallback()            | HAL_SPI_RegisterTxCpltCallback()
  RxCpltCallback              | HAL_SPI_RxCpltCallback()            | HAL_SPI_RegisterRxCpltCallback()
  TxRxCpltCallback            | HAL_SPI_TxRxCpltCallback()          | HAL_SPI_RegisterTxRxCpltCallback()
  TxHalfCpltCallback          | HAL_SPI_TxHalfCpltCallback()        | HAL_SPI_RegisterTxHalfCpltCallback()
  RxHalfCpltCallback          | HAL_SPI_RxHalfCpltCallback()        | HAL_SPI_RegisterRxHalfCpltCallback()
  TxRxHalfCpltCallback        | HAL_SPI_TxRxHalfCpltCallback()      | HAL_SPI_RegisterTxRxHalfCpltCallback()
  AbortCpltCallback           | HAL_SPI_AbortCpltCallback()         | HAL_SPI_RegisterAbortCpltCallback()
  SuspendCallback             | HAL_SPI_SuspendCallback()           | HAL_SPI_RegisterSuspendCallback()

  If one needs to unregister a callback, register the default callback via the registration function.

  By default, after the HAL_SPI_Init() and when the state is HAL_SPI_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Callbacks can be registered in handle global_state HAL_SPI_STATE_INIT and HAL_SPI_STATE_IDLE.

  When the compilation define USE_HAL_SPI_REGISTER_CALLBACKS is set to 0U or not defined, the callback registration
  feature is not available and weak callbacks are used, represented by the default value in the table
  above.

    Note: HAL_SPI_RegisterTxHalfCpltCallback(), HAL_SPI_RegisterRxHalfCpltCallback() and
      HAL_SPI_RegisterTxRxHalfCpltCallback() apply only in DMA mode.

  SuspendCallback restriction:
  SuspendCallback is called only when MasterReceiverAutoSuspend is enabled and
  End Of Transfer (EOF) interrupt is activated. SuspendCallback is used in relation with functions
  HAL_SPI_Transmit_IT, HAL_SPI_Receive_IT and HAL_SPI_TransmitReceive_IT.
  */
/**
  * @}
  */

/** @defgroup SPI_Configuration_Table SPI Configuration Table
  * @{
  ## Configuration inside the SPI driver:

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_HAL_SPI_MODULE             |         1         | Enable HAL SPI driver module
USE_HAL_SPI_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_SPI_DMA                |         1         | Enable DMA code inside SPI
USE_HAL_CHECK_PARAM            |         0         | Enable runtime parameter check
USE_HAL_SPI_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE    |         0         | Enable atomicity of process state check
USE_HAL_MUTEX                  |         0         | Enable semaphore creation for OS
USE_HAL_SPI_USER_DATA          |         0         | Add a user data inside HAL SPI handle
USE_HAL_SPI_GET_LAST_ERRORS    |         0         | Enable retrieval of last processes error codes
USE_HAL_SPI_CRC                |         1         | Enable the use of CRC feature inside driver

Software configuration defined in preprocessor environment:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM           |    Not defined    | Enable check param for HAL and LL
USE_ASSERT_DBG_STATE           |    Not defined    | Enable check state for HAL

  */
/**
  * @}
  */
#if defined(SPI1) || defined(SPI2) || defined(SPI3)
#if defined(USE_HAL_SPI_MODULE) && (USE_HAL_SPI_MODULE == 1)

/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
#define SPI_DEFAULT_TIMEOUT             100UL  /*!< Timeout default value     */

#define SPI_FIFO_SIZE                   16UL   /*!< Standard size 16-Bytes */


/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */

/**
  * @brief Convert bits to Bytes.
  */
#define CONVERT_TO_BYTES(value)        (((value)>>3UL)+1UL)

/**
  * @brief Get PacketSize from Datasize and Fifo Threshold.
  */
#define GET_PACKET_SIZE(data_width, fifo_threshold)   ((((fifo_threshold)>>SPI_CFG1_FTHLV_Pos)+1UL) *         \
                                                       (CONVERT_TO_BYTES((data_width) >> SPI_CFG1_DSIZE_Pos)))

/**
  * @brief Check if the packet length is supported by FIFO capacity.
  */
#define IS_SPI_PACKET_SIZE(packet_length) ((packet_length) <= SPI_FIFO_SIZE)

/**
  * @brief Check if the mode is type of hal_spi_mode_t.
  */
#define IS_SPI_MODE(mode)    (((mode) == HAL_SPI_MODE_SLAVE)                   \
                              || ((mode) == HAL_SPI_MODE_MASTER))

/**
  * @brief Check if the direction is type of hal_spi_direction_t.
  */
#define IS_SPI_DIRECTION(dir)    (((dir) == HAL_SPI_DIRECTION_FULL_DUPLEX)              \
                                  || ((dir) == HAL_SPI_DIRECTION_SIMPLEX_TX)            \
                                  || ((dir) == HAL_SPI_DIRECTION_SIMPLEX_RX)            \
                                  || ((dir) == HAL_SPI_DIRECTION_HALF_DUPLEX))

/**
  * @brief Check if the data width is type of hal_spi_data_width_t.
  */
#define IS_SPI_DATA_WIDTH(width)    (((width) == HAL_SPI_DATA_WIDTH_4_BIT)                    \
                                     || ((width) == HAL_SPI_DATA_WIDTH_5_BIT)                 \
                                     || ((width) == HAL_SPI_DATA_WIDTH_6_BIT)                 \
                                     || ((width) == HAL_SPI_DATA_WIDTH_7_BIT)                 \
                                     || ((width) == HAL_SPI_DATA_WIDTH_8_BIT)                 \
                                     || ((width) == HAL_SPI_DATA_WIDTH_9_BIT)                 \
                                     || ((width) == HAL_SPI_DATA_WIDTH_10_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_11_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_12_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_13_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_14_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_15_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_16_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_17_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_18_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_19_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_20_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_21_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_22_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_23_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_24_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_25_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_26_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_27_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_28_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_29_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_30_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_31_BIT)                \
                                     || ((width) == HAL_SPI_DATA_WIDTH_32_BIT))

/**
  * @brief Check if the clock polarity is type of hal_spi_clock_polarity_t.
  */
#define IS_SPI_POLARITY(polarity)    (((polarity) == HAL_SPI_CLOCK_POLARITY_LOW)             \
                                      || ((polarity) == HAL_SPI_CLOCK_POLARITY_HIGH))

/**
  * @brief Check if the clock phase is type of hal_spi_clock_phase_t.
  */
#define IS_SPI_PHASE(phase)    (((phase) == HAL_SPI_CLOCK_PHASE_1_EDGE)                      \
                                || ((phase) == HAL_SPI_CLOCK_PHASE_2_EDGE))

/**
  * @brief Check if the prescaler is type of hal_spi_baud_rate_prescaler_t.
  */
#define IS_SPI_PRESCALER(prescaler)    (((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_2)        \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_4)     \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_8)     \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_16)    \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_32)    \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_64)    \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_128)   \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_256)   \
                                        || ((prescaler) == HAL_SPI_BAUD_RATE_PRESCALER_BYPASS))

/**
  * @brief Check if the first bit is type of hal_spi_first_bit_t.
  */
#define IS_SPI_FIRST_BIT(first_bit)    (((first_bit) == HAL_SPI_MSB_FIRST)                \
                                        || ((first_bit) == HAL_SPI_LSB_FIRST))
#if defined(USE_HAL_SPI_CRC) && (USE_HAL_SPI_CRC == 1)

/**
  * @brief Check if the crc length is type of hal_spi_crc_length_t.
  */
#define IS_SPI_CRC_LENGTH(length)    (((length) == HAL_SPI_CRC_LENGTH_DATASIZE)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_4_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_5_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_6_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_7_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_8_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_9_BIT)               \
                                      || ((length) == HAL_SPI_CRC_LENGTH_10_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_11_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_12_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_13_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_14_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_15_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_16_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_17_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_18_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_19_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_20_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_21_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_22_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_23_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_24_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_25_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_26_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_27_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_28_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_29_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_30_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_31_BIT)              \
                                      || ((length) == HAL_SPI_CRC_LENGTH_32_BIT))

/**
  * @brief Check if the crc init pattern is type of hal_spi_crc_tx_init_pattern_t.
  */
#define IS_SPI_CRC_TX_INIT_PATTERN(pattern)    (((pattern) == HAL_SPI_CRC_TX_INIT_PATTERN_ALL_ZERO)    \
                                                || ((pattern) == HAL_SPI_CRC_TX_INIT_PATTERN_ALL_ONE))

/**
  * @brief Check if the crc init pattern is type of hal_spi_crc_rx_init_pattern_t.
  */
#define IS_SPI_CRC_RX_INIT_PATTERN(pattern)    (((pattern) == HAL_SPI_CRC_RX_INIT_PATTERN_ALL_ZERO)    \
                                                || ((pattern) == HAL_SPI_CRC_RX_INIT_PATTERN_ALL_ONE))

#endif /* USE_HAL_SPI_CRC */
/**
  * @brief Check if the nss management is type of hal_spi_nss_pin_management_t.
  */
#define IS_SPI_NSS_PIN_MANAGEMENT(management)  (((management) == HAL_SPI_NSS_PIN_MGMT_INTERNAL)        \
                                                || ((management) == HAL_SPI_NSS_PIN_MGMT_INPUT)        \
                                                || ((management) == HAL_SPI_NSS_PIN_MGMT_OUTPUT))

/**
  * @brief Check if the nss pulse state is type of hal_spi_nss_pulse_t.
  */
#define IS_SPI_NSS_PULSE(state)    (((state) == HAL_SPI_NSS_PULSE_DISABLE)                            \
                                    || ((state) == HAL_SPI_NSS_PULSE_ENABLE))


/**
  * @brief Check if the nss polarity is type of hal_spi_nss_polarity_t.
  */
#define IS_SPI_NSS_POLARITY(polarity)    (((polarity) == HAL_SPI_NSS_POLARITY_LOW)                    \
                                          || ((polarity) == HAL_SPI_NSS_POLARITY_HIGH))

/**
  * @brief Check if the mssi cycle is type of hal_spi_nss_master_slave_signal_idleness_delay_t.
  */
#define IS_SPI_NSS_MSSI_DELAY(cycle)  (((cycle) == HAL_SPI_NSS_MSSI_DELAY_0_CYCLE)                        \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_1_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_2_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_3_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_4_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_5_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_6_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_7_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_8_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_9_CYCLE)                     \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_10_CYCLE)                    \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_11_CYCLE)                    \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_12_CYCLE)                    \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_13_CYCLE)                    \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_14_CYCLE)                    \
                                       || ((cycle) == HAL_SPI_NSS_MSSI_DELAY_15_CYCLE))

/**
  * @brief Check if the midi delay is type of hal_spi_master_inter_data_idleness_delay_t.
  */
#define IS_SPI_MIDI_DELAY(delay)    (((delay) == HAL_SPI_MIDI_DELAY_0_CYCLE)                              \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_1_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_2_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_3_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_4_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_5_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_6_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_7_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_8_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_9_CYCLE)                           \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_10_CYCLE)                          \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_11_CYCLE)                          \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_12_CYCLE)                          \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_13_CYCLE)                          \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_14_CYCLE)                          \
                                     || ((delay) == HAL_SPI_MIDI_DELAY_15_CYCLE))

/**
  * @brief Check if the threshold is type of hal_spi_underrun_behavior_t.
  */
#define IS_SPI_UNDERRUN_BEHAV(behavior)    (((behavior) == HAL_SPI_UNDERRUN_BEHAV_REGISTER_PATTERN)        \
                                            || ((behavior) == HAL_SPI_UNDERRUN_BEHAV_LAST_RECEIVED))


/**
  * @brief Check if the ready pin polarity is type of hal_spi_ready_pin_polarity_t.
  */
#define IS_SPI_RDY_PIN_POLARITY(polarity)    (((polarity) == HAL_SPI_READY_PIN_POLARITY_HIGH)               \
                                              || ((polarity) == HAL_SPI_READY_PIN_POLARITY_LOW))

/**
  * @brief Check if the transfer size is valid when crc is disabled.
  */
#define IS_SPI_TRANSFER_SIZE(size)        (((size) < 0xFFFF) && (size) != 0)

/**
  * @brief Check if the threshold is type of hal_spi_fifo threshold_t.
  */
#define IS_SPI_FIFO_THRESHOLD(threshold)    (((threshold) == HAL_SPI_FIFO_THRESHOLD_1_DATA)                   \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_2_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_3_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_4_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_5_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_6_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_7_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_8_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_9_DATA)                \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_10_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_11_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_12_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_13_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_14_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_15_DATA)               \
                                             || ((threshold) == HAL_SPI_FIFO_THRESHOLD_16_DATA))

/**
  * @brief Check if the direction is Full-duplex.
  */
#define IS_SPI_DIRECTION_FULL_DUPLEX(mode) ((mode) == HAL_SPI_DIRECTION_FULL_DUPLEX)

/**
  * @brief Check if the direction is possible for TX transfer.
  */
#define IS_SPI_DIRECTION_TX_AVAILABLE(mode) (((mode) == HAL_SPI_DIRECTION_FULL_DUPLEX)                      \
                                             || ((mode) == HAL_SPI_DIRECTION_HALF_DUPLEX)                   \
                                             || ((mode) == HAL_SPI_DIRECTION_SIMPLEX_TX))

/**
  * @brief Check if the direction is possible for RX transfer.
  */
#define IS_SPI_DIRECTION_RX_AVAILABLE(mode) (((mode) == HAL_SPI_DIRECTION_FULL_DUPLEX)                     \
                                             || ((mode) == HAL_SPI_DIRECTION_HALF_DUPLEX)                  \
                                             || ((mode) == HAL_SPI_DIRECTION_SIMPLEX_RX))


/**
  * @brief Check if the CRC polynomial exists.
  */
#define IS_SPI_CRC_POLYNOMIAL(polynomial)                ((polynomial) > 0x0UL)
/**
  * @brief Check the consistency between polynomial and its size.
  */
#define IS_SPI_CRC_POLYNOMIAL_SIZE(polynomial, length)\
  (((polynomial) >> (((length) >> SPI_CFG1_CRCSIZE_Pos) + 1UL)) == 0UL)

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
static void SPI_DMATransmitCplt(hal_dma_handle_t *hdma);
static void SPI_DMAReceiveCplt(hal_dma_handle_t *hdma);
static void SPI_DMATransmitReceiveCplt(hal_dma_handle_t *hdma);
static void SPI_DMAHalfTransmitCplt(hal_dma_handle_t *hdma);
static void SPI_DMAHalfReceiveCplt(hal_dma_handle_t *hdma);
static void SPI_DMAHalfTransmitReceiveCplt(hal_dma_handle_t *hdma);
static void SPI_DMAError(hal_dma_handle_t *hdma);
static void SPI_DMAAbortOnError(hal_dma_handle_t *hdma);
static void SPI_DMATxAbortCallback(hal_dma_handle_t *hdma);
static void SPI_DMARxAbortCallback(hal_dma_handle_t *hdma);
static void SPI_DMAEmptyCallback(hal_dma_handle_t *hdma);
#endif /* USE_HAL_SPI_DMA  */
static hal_status_t SPI_WaitEndOfTransfer(hal_spi_handle_t *hspi,
                                          uint32_t timeout_ms, uint32_t tick_start);
static void SPI_TxISR_8BIT(hal_spi_handle_t *hspi);
static void SPI_TxISR_16BIT(hal_spi_handle_t *hspi);
static void SPI_TxISR_32BIT(hal_spi_handle_t *hspi);
static void SPI_RxISR_8BIT(hal_spi_handle_t *hspi);
static void SPI_RxISR_16BIT(hal_spi_handle_t *hspi);
static void SPI_RxISR_32BIT(hal_spi_handle_t *hspi);
static void SPI_AbortTransfer(hal_spi_handle_t *hspi);
static hal_status_t SPI_CloseTransfer(hal_spi_handle_t *hspi);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @addtogroup SPI_Exported_Functions_Group1 Initialization / De-Initialization functions
  * @{

  This subsection provides a set of functions allowing to initialize and de-initialize the SPIx peripheral:
    - Call the function HAL_SPI_Init() to initialize the selected SPI handle and associate an instance.
    - Call the function HAL_SPI_DeInit() to restore the default initialization of the selected SPIx peripheral.

  */

/**
  * @brief Initialize the SPI according to the associated handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param instance SPI instance.
  * @retval HAL_INVALID_PARAM When the handle is NULL.
  * @retval HAL_ERROR When the MUTEX cannot be created.
  * @retval HAL_OK HAL SPI driver correctly Initialized for the given SPI instance.
  */
hal_status_t HAL_SPI_Init(hal_spi_handle_t *hspi, hal_spi_t instance)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_ALL_INSTANCE((SPI_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hspi == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->instance = instance;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)

  /* Init the SPI Callback settings */
  hspi->p_error_cb           = HAL_SPI_ErrorCallback;        /* Legacy weak ErrorCallback        */
  hspi->p_tx_cplt_cb         = HAL_SPI_TxCpltCallback;       /* Legacy weak TxCpltCallback       */
  hspi->p_rx_cplt_cb         = HAL_SPI_RxCpltCallback;       /* Legacy weak RxCpltCallback       */
  hspi->p_tx_rx_cplt_cb      = HAL_SPI_TxRxCpltCallback;     /* Legacy weak TxRxCpltCallback     */
  hspi->p_tx_half_cplt_cb    = HAL_SPI_TxHalfCpltCallback;   /* Legacy weak TxHalfCpltCallback   */
  hspi->p_rx_half_cplt_cb    = HAL_SPI_RxHalfCpltCallback;   /* Legacy weak RxHalfCpltCallback   */
  hspi->p_tx_rx_half_cplt_cb = HAL_SPI_TxRxHalfCpltCallback; /* Legacy weak TxRxHalfCpltCallback */
  hspi->p_abort_cplt_cb      = HAL_SPI_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
  hspi->p_suspend_cb         = HAL_SPI_SuspendCallback;      /* Legacy weak SuspendCallback      */
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

  /* Other internal fields */
  hspi->p_tx_buff            = (uint8_t *)  NULL;
  hspi->tx_xfer_size         = (uint16_t)   0UL;
  hspi->tx_xfer_count        = (uint16_t)   0UL;
  hspi->p_rx_buff            = (uint8_t *)  NULL;
  hspi->rx_xfer_size         = (uint16_t)   0UL;
  hspi->rx_xfer_count        = (uint16_t)   0UL;

#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  hspi->hdma_tx              = (hal_dma_handle_t *) NULL;
  hspi->hdma_rx              = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_SPI_DMA  */

#if defined(USE_HAL_SPI_USER_DATA) && (USE_HAL_SPI_USER_DATA == 1)
  hspi->p_user_data          = NULL;
#endif /* USE_HAL_SPI_USER_DATA */

#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  /* Reset the last_error_codes variable storing the last errors */
  hspi->last_error_codes    = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

#if defined(USE_HAL_SPI_CLK_ENABLE_MODEL) && (USE_HAL_SPI_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  /* Enable the SPI peripheral clock */
  switch (hspi->instance)
  {
#if defined(SPI1)
    case HAL_SPI1:
      HAL_RCC_SPI1_EnableClock();
      break;
#endif /* SPI1 */
#if defined(SPI2)
    case HAL_SPI2:
      HAL_RCC_SPI2_EnableClock();
      break;
#endif /* SPI2 */
#if defined(SPI3)
    case HAL_SPI3:
      HAL_RCC_SPI3_EnableClock();
      break;
#endif /* SPI3 */
    default:
      break;
  }

#endif /* USE_HAL_SPI_CLK_ENABLE_MODEL */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Create the SPI semaphore */
  if (HAL_OS_SemaphoreCreate(&hspi->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_SPI_MUTEX  */

  hspi->global_state         = HAL_SPI_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief De-Initialize the HAL SPI driver for the given handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
void HAL_SPI_DeInit(hal_spi_handle_t *hspi)
{
  hal_spi_state_t temp_state;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_ALL_INSTANCE((SPI_TypeDef *)((uint32_t)hspi->instance)));

  temp_state = hspi->global_state;
  /* Check if any transfer ongoing */
  if ((temp_state == HAL_SPI_STATE_TX_ACTIVE) || (temp_state == HAL_SPI_STATE_RX_ACTIVE)
      || (temp_state == HAL_SPI_STATE_TX_RX_ACTIVE))
  {
#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
    /* Disable the SPI DMA Tx request if enabled */
    if (LL_SPI_IsEnabledDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
    {
      if (hspi->hdma_tx != NULL)
      {
        /* Abort DMA Tx Handle linked to SPI peripheral */
        (void)HAL_DMA_Abort(hspi->hdma_tx);
      }
    }

    /* Disable the SPI DMA Rx request if enabled */
    if (LL_SPI_IsEnabledDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
    {
      if (hspi->hdma_rx != NULL)
      {
        /* Abort DMA Rx Handle linked to SPI peripheral */
        (void)HAL_DMA_Abort(hspi->hdma_rx);
      }
    }
#endif /* USE_HAL_SPI_DMA */

    SPI_AbortTransfer(hspi);
  }

  LL_SPI_Disable((SPI_TypeDef *)((uint32_t)hspi->instance));

#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  /* Reset the last_error_codes variable storing the last errors */
  hspi->last_error_codes    = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&hspi->semaphore);
#endif /* USE_SPI_MUTEX  */

  hspi->global_state         = HAL_SPI_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group2 General Config functions
  * @{
  This subsection provides a set of functions allowing to configure the SPIx peripheral:
    - Call the function HAL_SPI_SetConfig() to configure the selected device with the selected configuration:
      - Mode
      - Direction
      - Data Width
      - Clock Polarity and Phase
      - Baud Rate Prescaler
      - FirstBit
      - Chip select management
    - Call the function HAL_SPI_GetConfig() to retrieve the current global configuration set by the user.
  */

/**
  * @brief Set the configuration to the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to hal_spi_config_t configuration structure.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  * @retval HAL_ERROR When io locked.
  * @retval HAL_OK SPI instance has been correctly configured.
  */
hal_status_t HAL_SPI_SetConfig(hal_spi_handle_t *hspi, const hal_spi_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_SPI_MODE(p_config->mode));
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION(p_config->direction));
  ASSERT_DBG_PARAM(IS_SPI_DATA_WIDTH(p_config->data_width));
  ASSERT_DBG_PARAM(IS_SPI_POLARITY(p_config->clock_polarity));
  ASSERT_DBG_PARAM(IS_SPI_PHASE(p_config->clock_phase));
  ASSERT_DBG_PARAM(IS_SPI_PRESCALER(p_config->baud_rate_prescaler));
  ASSERT_DBG_PARAM(IS_SPI_FIRST_BIT(p_config->first_bit));
  ASSERT_DBG_PARAM(IS_SPI_NSS_PIN_MANAGEMENT(p_config->nss_pin_management));


  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_INIT);

  /* Check for IOLock */
  if ((LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance))) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Set SPI generic configuration */
  LL_SPI_SetConfig((SPI_TypeDef *)((uint32_t)hspi->instance),
                   (uint32_t)p_config->data_width | (uint32_t)p_config->baud_rate_prescaler,
                   (uint32_t)p_config->mode | (uint32_t)p_config->direction |
                   (uint32_t)p_config->clock_polarity | (uint32_t)p_config->clock_phase |
                   (uint32_t)p_config->first_bit | (uint32_t)p_config->nss_pin_management);


  /* Store SPI direction into the handle */
  hspi->direction = p_config->direction;

  hspi->global_state = HAL_SPI_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Retrieve the configuration from the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to hal_spi_config_t configuration structure.
  */
void HAL_SPI_GetConfig(const hal_spi_handle_t *hspi, hal_spi_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  /* Get SPI basic configuration */
  uint32_t cfg1_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG1);
  uint32_t cfg2_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG2);

  p_config->mode  = ((hal_spi_mode_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_MASTER)));
  p_config->direction = hspi->direction;
  p_config->data_width = (hal_spi_data_width_t)((uint32_t)(cfg1_reg_value & SPI_CFG1_DSIZE));
  p_config->clock_polarity = (hal_spi_clock_polarity_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_CPOL));
  p_config->clock_phase = (hal_spi_clock_phase_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_CPHA));
  p_config->baud_rate_prescaler = (hal_spi_baud_rate_prescaler_t)
                                  ((uint32_t)(cfg1_reg_value & (SPI_CFG1_MBR | SPI_CFG1_BPASS)));
  p_config->first_bit = (hal_spi_first_bit_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_LSBFRST));
  p_config->nss_pin_management = (hal_spi_nss_pin_management_t)
                                 ((uint32_t)(cfg2_reg_value & (SPI_CFG2_SSM | SPI_CFG2_SSOE)));
}

/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group3 Features functions
  * @{
  This subsection provides a set of functions allowing to configure some additional
  features for the selected SPIx peripheral.

  There are two types of features, features with configuration parameters and features without
  configuration parameters.

  For each feature that has a configuration structure,
  there are those dedicated APIs:
    - HAL_SPI_SetConfigCRC() : Configure the CRC feature.
    - HAL_SPI_GetConfigCRC() : Retrieve the current CRC feature configuration.
    - HAL_SPI_EnableCRC() : Enable the CRC feature with user defined configuration.
    - HAL_SPI_DisableCRC() : Disable the CRC feature for the dedicated SPIx instance.
    - HAL_SPI_IsEnabledCRC() : Retrieve CRC feature status for the dedicated SPIx instance.

  There is a specific case for always-on features which cannot be disabled (NSS and Underrun detection),
  there are those dedicated APIs:
    - HAL_SPI_SetConfigNSS() : Configure the NSS feature.
    - HAL_SPI_GetConfigNSS() : Retrieve the current NSS feature configuration.
    - HAL_SPI_SLAVE_SetConfigUnderrun() : Configure the Underrun detection feature.
    - HAL_SPI_SLAVE_GetConfigUnderrun() : Retrieve the current underrun detection feature configuration.

  For each feature without parameters (TI Mode, Master Receiver Auto Suspend, Master Keep IO State, IO Swap,
  Delay Read Data Sampling and Ready Pin management), there are those dedicated APIs:
    - HAL_SPI_EnableTIMode()    : Enable the TI Mode feature for the dedicated SPIx instance.
    - HAL_SPI_DisableTIMode()   : Disable the TI Mode feature for the dedicated SPIx instance.
    - HAL_SPI_IsEnabledTIMode() : Retrieve the TI Mode feature status for the dedicated SPIx instance.
    - HAL_SPI_MASTER_EnableReceiverAutoSuspend()    : Enable the Master Receiver Automatic Suspension feature
                                                  for the dedicated SPIx instance.
    - HAL_SPI_MASTER_DisableReceiverAutoSuspend()   : Disable the Master Receiver Automatic Suspension feature
                                                  for the dedicated SPIx instance.
    - HAL_SPI_MASTER_IsEnabledReceiverAutoSuspend() : Retrieve the Master Receiver Automatic Suspension feature
                                                  status for the dedicated SPIx instance.
    - HAL_SPI_MASTER_EnableKeepIOState()    : Enable the Master Keep IO State feature for the dedicated SPIx instance.
    - HAL_SPI_MASTER_DisableKeepIOState()   : Disable the Master Keep IO State feature for the dedicated SPIx instance.
    - HAL_SPI_MASTER_IsEnabledKeepIOState() : Retrieve the Master Keep IO State feature status for the dedicated
                                             SPIx instance.
    - HAL_SPI_EnableMosiMisoSwap()    : Enable the IO Swap feature for the dedicated SPIx instance.
    - HAL_SPI_DisableMosiMisoSwap()   : Disable the IO Swap feature for the dedicated SPIx instance.
    - HAL_SPI_IsEnabledMosiMisoSwap() : Retrieve the IO Swap feature status for the dedicated SPIx instance.
    - HAL_SPI_EnableDelayReadDataSampling()    : Enable the Delay Read Data Sampling feature for the dedicated
                                                 SPIx instance.
    - HAL_SPI_DisableDelayReadDataSampling()   : Disable the Delay Read Data Sampling feature for the dedicated
                                                 SPIx instance.
    - HAL_SPI_IsEnabledDelayReadDataSampling() : Retrieve the Delay Read Data Sampling feature status for the
                                                 dedicated SPIx instance.
    - HAL_SPI_EnableReadyPin()    : Enable the Ready Pin management feature for the dedicated SPIx instance.
    - HAL_SPI_DisableReadyPin()   : Disable the Ready Pin management feature for the dedicated SPIx instance.
    - HAL_SPI_IsEnabledReadyPin() : Retrieve the Ready Pin management feature status for the dedicated SPIx instance.
  There are two other specific functions for the IO config feature which are:
    - HAL_SPI_LockIOConfig() : Lock the IO configuration for the dedicated SPIx instance.
    - HAL_SPI_IsLockedIOConfig() : Retrieve the IO configuration status for the dedicated SPIx instance.
    When this bit is set, the configuration register linked to IO configuration (SPI_CFG2) cannot be modified.
    This lock can be enabled only when SPI is disabled, otherwise it is write protected.
    It is cleared and cannot be set when a Mode Fault is detected (SPI_SR/MODF bit is set).

  */

#if defined(USE_HAL_SPI_CRC) && (USE_HAL_SPI_CRC == 1)

/**
  * @brief Configure the CRC feature.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to hal_spi_crc_config_t configuration structure.
  * @retval HAL_INVALID_PARAM in case of invalid parameter allocation such as a null config pointer.
  * @retval HAL_OK in case of valid configuration.
  */
hal_status_t HAL_SPI_SetConfigCRC(hal_spi_handle_t *hspi, const hal_spi_crc_config_t *p_config)
{
  /* Local variables*/
  uint32_t length_crc;
  uint32_t crc_poly_msb_mask;
  uint32_t crc_polynomial;
  uint32_t data_width;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_SPI_CRC_POLYNOMIAL(p_config->crc_polynomial));
  ASSERT_DBG_PARAM(IS_SPI_CRC_POLYNOMIAL_SIZE(p_config->crc_polynomial, p_config->crc_length));
  ASSERT_DBG_PARAM(IS_SPI_CRC_LENGTH(p_config->crc_length));
  ASSERT_DBG_PARAM(IS_SPI_CRC_TX_INIT_PATTERN(p_config->crc_tx_init_pattern));
  ASSERT_DBG_PARAM(IS_SPI_CRC_RX_INIT_PATTERN(p_config->crc_rx_init_pattern));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Local variables*/
  length_crc = (uint32_t)p_config->crc_length;
  crc_polynomial = (uint32_t)p_config->crc_polynomial;
  data_width = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Align the CRC length on the data size if HAL_SPI_CRC_LENGTH_DATASIZE */
  if (length_crc == ((uint32_t)HAL_SPI_CRC_LENGTH_DATASIZE))
  {
    length_crc = (((data_width) >> SPI_CFG1_DSIZE_Pos) << SPI_CFG1_CRCSIZE_Pos);
  }

  /* Enable 33/17 bits CRC computation in case maximum CRC size is used.
   * Maximum CRC size depends on SPIx capabilities (refer into the reference
   * manual on CRC computation feature)
   */
  if (length_crc == ((uint32_t)HAL_SPI_CRC_LENGTH_32_BIT))
  {
    LL_SPI_EnableFullSizeCRC((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
  else
  {
    LL_SPI_DisableFullSizeCRC((SPI_TypeDef *)((uint32_t)hspi->instance));

    /* Set MSB of CRC polynomial at 1 in SPI Register      */
    /* Set MSB is mandatory for a correct CRC computation  */
    crc_poly_msb_mask = (0x1UL << ((length_crc >> SPI_CFG1_CRCSIZE_Pos) + 0x1U));
    crc_polynomial |= crc_poly_msb_mask;
  }

  LL_SPI_SetCRCPolynomial((SPI_TypeDef *)((uint32_t)hspi->instance), crc_polynomial);

  LL_SPI_SetCRCWidth((SPI_TypeDef *)((uint32_t)hspi->instance), length_crc);

  LL_SPI_SetCRCInitPattern((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)p_config->crc_tx_init_pattern,
                           (uint32_t)p_config->crc_rx_init_pattern);
  return HAL_OK;
}

/**
  * @brief Retrieve the current CRC configuration.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to the hal_spi_crc_config_t configuration structure.
  */
void HAL_SPI_GetConfigCRC(const hal_spi_handle_t *hspi, hal_spi_crc_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  uint32_t init_pattern = LL_SPI_GetCRCInitPattern((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Get SPI CRC configuration */
  p_config->crc_polynomial       = LL_SPI_GetCRCPolynomial((SPI_TypeDef *)((uint32_t)hspi->instance));
  p_config->crc_length           = (hal_spi_crc_length_t)
                                   ((uint32_t)LL_SPI_GetCRCWidth((SPI_TypeDef *)((uint32_t)hspi->instance)));
  p_config->crc_tx_init_pattern  = (hal_spi_crc_tx_init_pattern_t)((uint32_t)(init_pattern & SPI_CR1_TCRCINI));
  p_config->crc_rx_init_pattern  = (hal_spi_crc_rx_init_pattern_t)((uint32_t)(init_pattern & SPI_CR1_RCRCINI));
}

/**
  * @brief Enable the CRC feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_OK CRC feature enabled successfully.
  */
hal_status_t HAL_SPI_EnableCRC(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  LL_SPI_EnableCRC((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Disable the CRC feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_OK CRC feature disabled successfully.
  */
hal_status_t HAL_SPI_DisableCRC(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  LL_SPI_DisableCRC((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve CRC status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_crc_status_t SPI CRC feature status.
  */
hal_spi_crc_status_t HAL_SPI_IsEnabledCRC(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_crc_status_t) LL_SPI_IsEnabledCRC((SPI_TypeDef *)((uint32_t)hspi->instance));
}
#endif /* USE_HAL_SPI_CRC */

/**
  * @brief Configure the NSS feature.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to hal_spi_nss_config_t configuration structure.
  * @retval HAL_INVALID_PARAM in case of invalid parameter allocation.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK in case of valid configuration.
  */
hal_status_t HAL_SPI_SetConfigNSS(hal_spi_handle_t *hspi, const hal_spi_nss_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_SPI_NSS_PULSE(p_config->nss_pulse));
  ASSERT_DBG_PARAM(IS_SPI_NSS_POLARITY(p_config->nss_polarity));
  ASSERT_DBG_PARAM(IS_SPI_NSS_MSSI_DELAY(p_config->nss_mssi_delay));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetNSSConfig(((SPI_TypeDef *)((uint32_t)hspi->instance)),
                      ((uint32_t)p_config->nss_pulse) | ((uint32_t)p_config->nss_polarity) |
                      ((uint32_t)p_config->nss_mssi_delay));

  return HAL_OK;
}

/**
  * @brief Retrieve the current NSS configuration.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to the hal_spi_nss_config_t configuration structure.
  */
void HAL_SPI_GetConfigNSS(const hal_spi_handle_t *hspi, hal_spi_nss_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  uint32_t cfg2_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG2);

  /* Get SPI NSS configuration */
  p_config->nss_pulse       = (hal_spi_nss_pulse_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_SSOM));
  p_config->nss_polarity    = (hal_spi_nss_polarity_t)((uint32_t)(cfg2_reg_value & SPI_CFG2_SSIOP));
  p_config->nss_mssi_delay  = (hal_spi_nss_master_slave_signal_idleness_delay_t)
                              ((uint32_t)(cfg2_reg_value & SPI_CFG2_MSSI));
}

/**
  * @brief Configure the Underrun detection mode feature.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to the hal_spi_underrun_config_t data structure.
  * @retval HAL_INVALID_PARAM in case of invalid parameter allocation.
  * @retval HAL_OK in case of valid configuration.
  */
hal_status_t HAL_SPI_SLAVE_SetConfigUnderrun(const hal_spi_handle_t *hspi, const hal_spi_underrun_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_SPI_UNDERRUN_BEHAV(p_config->underrun_behavior));
  /* Ensure that Underrun configuration is managed only by Slave */
  ASSERT_DBG_PARAM(LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance)) == HAL_SPI_MODE_SLAVE);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Configure Underrun fields */
  LL_SPI_SetUDRConfiguration((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)p_config->underrun_behavior);

  return HAL_OK;
}

/**
  * @brief Retrieve the current underrun detection configuration.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_config Pointer to the hal_spi_underrun_config_t data structure.
  */
void HAL_SPI_SLAVE_GetConfigUnderrun(const hal_spi_handle_t *hspi, hal_spi_underrun_config_t *p_config)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  /* Retrieve the current Autonomous mode configuration */
  uint32_t cfg1_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG1);
  uint32_t underrun_behavior = cfg1_reg_value & SPI_CFG1_UDRCFG;
  p_config->underrun_behavior  = (hal_spi_underrun_behavior_t)underrun_behavior;
}

/**
  * @brief Enable the TI mode feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK TI mode feature enabled successfully.
  */
hal_status_t HAL_SPI_EnableTIMode(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Enable the TI mode */
  LL_SPI_SetStandard((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_PROTOCOL_TI);

  return HAL_OK;
}

/**
  * @brief Disable the TI mode feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK TI mode feature disabled successfully.
  */
hal_status_t HAL_SPI_DisableTIMode(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Disable the TI mode */
  LL_SPI_SetStandard((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_PROTOCOL_MOTOROLA);

  return HAL_OK;
}

/**
  * @brief Retrieve the TI mode status for the dedicated SPI.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_ti_mode_status_t SPI TI mode feature status.
  */
hal_spi_ti_mode_status_t HAL_SPI_IsEnabledTIMode(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return ((LL_SPI_GetStandard((SPI_TypeDef *)((uint32_t)hspi->instance)) == LL_SPI_PROTOCOL_TI)
          ? HAL_SPI_TI_MODE_ENABLED : HAL_SPI_TI_MODE_DISABLED);
}

/**
  * @brief Enable the master automatic suspension in Receive mode feature for the dedicated SPIx.
  * The automatic suspension is not quite reliable when size of data drops below 8
  * bits. In this case, a safe suspension can be achieved by combination with delay inserted
  * between data frames applied when MIDI parameter keeps a non zero value; sum of data size
  * and the interleaved SPI cycles must always produce interval at length of 8 SPI clock
  * periods at minimum.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_OK Master Receiver Automatic Suspension feature enabled successfully.
  */
hal_status_t HAL_SPI_MASTER_EnableReceiverAutoSuspend(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_PARAM(LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance)) == HAL_SPI_MODE_MASTER);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Enable MasterRxAutoSuspend */
  LL_SPI_EnableMasterRxAutoSuspend((SPI_TypeDef *)((uint32_t)hspi->instance));
  return HAL_OK;
}

/**
  * @brief Disable the master automatic suspension in Receive mode feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_OK Master Receiver Automatic Suspension feature disabled successfully.
  */
hal_status_t HAL_SPI_MASTER_DisableReceiverAutoSuspend(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  LL_SPI_DisableMasterRxAutoSuspend((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the master automatic suspension in Receive mode feature status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_master_rx_auto_suspend_status_t SPI master receiver automatic suspension feature status.
  */
hal_spi_master_rx_auto_suspend_status_t HAL_SPI_MASTER_IsEnabledReceiverAutoSuspend(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_master_rx_auto_suspend_status_t)
         LL_SPI_IsEnabledMasterRxAutoSuspend((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Enable the alternate function GPIOs control feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @note Enabling the "Master Keep IO State" prevents any glitches on the associated outputs configured at
  *       alternate function mode by keeping them forced at state corresponding the current SPI configuration.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Master Keep IO State feature enabled successfully.
  */
hal_status_t HAL_SPI_MASTER_EnableKeepIOState(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_EnableGPIOControl((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Disable the alternate function GPIOs control feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Master Keep IO State feature disabled successfully.
  */
hal_status_t HAL_SPI_MASTER_DisableKeepIOState(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_DisableGPIOControl((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the alternate function GPIOs control feature status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_master_keep_io_state_status_t SPI master keep IO state feature status.
  */
hal_spi_master_keep_io_state_status_t HAL_SPI_MASTER_IsEnabledKeepIOState(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_master_keep_io_state_status_t)LL_SPI_IsEnabledGPIOControl((SPI_TypeDef *)((uint32_t)hspi->instance));
}


/**
  * @brief Enable the MISO/MOSI alternate functions inversion feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK IO Swap feature enabled successfully.
  */
hal_status_t HAL_SPI_EnableMosiMisoSwap(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_EnableMosiMisoSwap((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Disable the MISO/MOSI alternate functions inversion feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK IO Swap feature disabled successfully.
  */
hal_status_t HAL_SPI_DisableMosiMisoSwap(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_DisableMosiMisoSwap((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the MISO/MOSI alternate functions inversion feature status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_mosi_miso_swap_status_t SPI IO swap feature status.
   */
hal_spi_mosi_miso_swap_status_t HAL_SPI_IsEnabledMosiMisoSwap(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_mosi_miso_swap_status_t)LL_SPI_IsEnabledMosiMisoSwap((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Enable the ready pin feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Ready Pin feature enabled successfully.
  */
hal_status_t HAL_SPI_EnableReadyPin(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_EnableReadyPin((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Disable the ready pin feature for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Ready Pin feature disabled successfully.
  */
hal_status_t HAL_SPI_DisableReadyPin(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_DisableReadyPin((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the ready pin feature status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_ready_pin_status_t SPI ready pin management feature status.
   */
hal_spi_ready_pin_status_t HAL_SPI_IsEnabledReadyPin(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_ready_pin_status_t)LL_SPI_IsEnabledReadyPin((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the ready pin polarity for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param polarity This parameter must be a value of hal_spi_ready_pin_polarity_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Ready Pin polarity set successfully.
  */
hal_status_t HAL_SPI_SetReadyPinPolarity(hal_spi_handle_t *hspi, hal_spi_ready_pin_polarity_t polarity)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_RDY_PIN_POLARITY(polarity));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetReadyPinPolarity((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t) polarity);

  return HAL_OK;
}

/**
  * @brief Retrieve the ready pin polarity of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_ready_pin_polarity_t Current ready pin polarity.
  */
hal_spi_ready_pin_polarity_t HAL_SPI_GetReadyPinPolarity(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_ready_pin_polarity_t)LL_SPI_GetReadyPinPolarity((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Enable the Delay Read Data Sampling on Master Input IO.
  *        DRDS setting has no impact on the other SCK management.
  *        When CRC is enabled, CRC computation and evaluation is delayed too.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Delay Read Data Sampling enabled successfully.
  */
hal_status_t HAL_SPI_EnableDelayReadDataSampling(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Enable Delay Read Data Sampling feature */
  LL_SPI_EnableDelayReadDataSampling((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Disable the Delay Read Data Sampling on Master Input IO.
  *        DRDS setting has no impact on the other SCK management.
  *        When CRC is enabled, CRC computation and evaluation is delayed too.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Delay Read Data Sampling enabled successfully.
  */
hal_status_t HAL_SPI_DisableDelayReadDataSampling(hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_DisableDelayReadDataSampling((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the Delay Read Data Sampling feature status for the dedicated SPIx.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_drds_status_t Delay Read Data Sampling feature status.
   */
hal_spi_drds_status_t HAL_SPI_IsEnabledDelayReadDataSampling(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_drds_status_t)LL_SPI_IsEnabledDelayReadDataSampling((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Lock the IO configuration for the dedicated SPI.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @note The reset of the IOLock bit is done by hardware.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_LockIOConfig(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  LL_SPI_EnableIOLock((SPI_TypeDef *)((uint32_t)hspi->instance));

  return HAL_OK;
}

/**
  * @brief Retrieve the IO configuration lock status for the dedicated SPI.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_io_cfg_status_t.
  */
hal_spi_io_cfg_status_t HAL_SPI_IsLockedIOConfig(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_io_cfg_status_t)LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @}
  */


/** @addtogroup SPI_Exported_Functions_Group5 Items functions
  * @{
  This subsection provides a set of functions allowing to change and retrieve a single configuration item
  in the IDLE state.
    - HAL_SPI_SetMode() : Set the mode of the SPI peripheral.
    - HAL_SPI_GetMode() : Retrieve the mode of the SPI peripheral.
    - HAL_SPI_SetDirection() : Set the direction of the SPI peripheral.
    - HAL_SPI_GetDirection() : Retrieve the direction of the SPI peripheral.
    - HAL_SPI_SetDataWidth() : Set the data width for the SPI peripheral.
    - HAL_SPI_GetDataWidth() : Retrieve the data width of the SPI peripheral.
    - HAL_SPI_SetClockPolarity() : Set the clock polarity of the SPI peripheral.
    - HAL_SPI_GetClockPolarity() : Retrieve the clock polarity of the SPI peripheral.
    - HAL_SPI_SetClockPhase() : Set the active clock edge for the bit capture of the SPI peripheral.
    - HAL_SPI_GetClockPhase() : Retrieve the active clock edge for the bit capture of the SPI peripheral.
    - HAL_SPI_SetBaudRatePrescaler() : Set the Baud Rate prescaler value which is used to configure the transmit
                                       and receive SCK clock of the SPI peripheral.
    - HAL_SPI_GetBaudRatePrescaler() : Retrieve the baud rate prescaler of the SPI peripheral.
    - HAL_SPI_SetFirstBit() : Set whether data transfers start from MSB or LSB bit.
    - HAL_SPI_GetFirstBit() : Retrieve the first bit (MSB or LSB bit) of the SPI peripheral.
    - HAL_SPI_SetNSSPinManagement() : Set the NSS pin management mode of the SPI peripheral.
    - HAL_SPI_GetNSSPinManagement() : Retrieve the NSS pin management mode of the SPI peripheral.
    - HAL_SPI_SetFifoThreshold() : Set the FIFO threshold level of the SPI peripheral.
    - HAL_SPI_GetFifoThreshold() : Retrieve the FIFO threshold level of the SPI peripheral.
    - HAL_SPI_MASTER_SetInterDataIdlenessDelay() : Set an extra delay, expressed in number of SPI clock cycle periods,
                                                   inserted additionally between active edge of Slave Select signal
                                                   and first data transaction start in master mode.
    - HAL_SPI_MASTER_GetInterDataIdlenessDelay() : Retrieve the extra delay, expressed in number of SPI clock cycle
                                                   periods, inserted additionally between active edge of Slave
                                                   Select signal and first data transaction start in master mode.
  */

/**
  * @brief Set the mode of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param mode This parameter must be a value of hal_spi_mode_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetMode(hal_spi_handle_t *hspi, const hal_spi_mode_t mode)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_MODE(mode));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Get SPI basic configuration */
  uint32_t cfg2_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG2);

  /* SPIx NSS Internal Management Configuration */
  if ((STM32_IS_BIT_SET(cfg2_reg_value, SPI_CFG2_SSM))
      && (((mode == HAL_SPI_MODE_MASTER) && (STM32_IS_BIT_CLR(cfg2_reg_value, SPI_CFG2_SSIOP)))
          || ((mode == HAL_SPI_MODE_SLAVE)  && (STM32_IS_BIT_SET(cfg2_reg_value, SPI_CFG2_SSIOP)))))
  {
    LL_SPI_SetInternalSSLevel((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SS_LEVEL_HIGH);
  }
  else
  {
    LL_SPI_SetInternalSSLevel((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SS_LEVEL_LOW);
  }

  LL_SPI_SetMode((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)mode);

  return HAL_OK;
}

/**
  * @brief Retrieve the mode of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_mode_t Current SPI mode configuration.
  */
hal_spi_mode_t HAL_SPI_GetMode(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_mode_t)LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the direction of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param direction This parameter must be a value of hal_spi_direction_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetDirection(hal_spi_handle_t *hspi, const hal_spi_direction_t direction)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION(direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  /* Store the transfer direction in the handle, to check it later in operational functions */
  hspi->direction = direction;
  LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)direction);

  return HAL_OK;
}

/**
  * @brief Retrieve the direction of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_direction_t Current SPI direction configuration.
  */
hal_spi_direction_t HAL_SPI_GetDirection(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_direction_t)LL_SPI_GetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the data width for the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param data_width This parameter must be a value of hal_spi_data_width_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetDataWidth(const hal_spi_handle_t *hspi, const hal_spi_data_width_t data_width)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_PARAM(IS_SPI_DATA_WIDTH(data_width));
#if defined(USE_ASSERT_DBG_PARAM)
  /* Get PacketSize */
  uint32_t packet_length = GET_PACKET_SIZE(data_width,
                                           LL_SPI_GetFIFOThreshold((SPI_TypeDef *)((uint32_t)hspi->instance)));
  ASSERT_DBG_PARAM(IS_SPI_PACKET_SIZE(packet_length));
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  LL_SPI_SetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)data_width);

  return HAL_OK;
}

/**
  * @brief Retrieve the data width of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_data_width_t Current SPI data width configuration.
  */
hal_spi_data_width_t HAL_SPI_GetDataWidth(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_data_width_t)LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the clock polarity of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param clock_polarity This parameter must be a value of hal_spi_clock_polarity_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetClockPolarity(hal_spi_handle_t *hspi, const hal_spi_clock_polarity_t clock_polarity)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_POLARITY(clock_polarity));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetClockPolarity((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)clock_polarity);

  return HAL_OK;
}

/**
  * @brief Retrieve the clock polarity of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_clock_polarity_t Current SPI clock polarity configuration.
  */
hal_spi_clock_polarity_t HAL_SPI_GetClockPolarity(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_clock_polarity_t)LL_SPI_GetClockPolarity((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the active clock edge for the bit capture.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param clock_phase This parameter must be a value of hal_spi_clock_phase_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetClockPhase(hal_spi_handle_t *hspi, const hal_spi_clock_phase_t clock_phase)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_PHASE(clock_phase));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetClockPhase((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)clock_phase);

  return HAL_OK;
}

/**
  * @brief Retrieve the active clock edge for the bit capture of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_clock_phase_t Current SPI clock phase configuration.
  */
hal_spi_clock_phase_t HAL_SPI_GetClockPhase(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_clock_phase_t)LL_SPI_GetClockPhase((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the Baud Rate prescaler value which is used to configure the transmit and receive SCK clock.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param baud_rate_prescaler This parameter must be a value of hal_spi_baud_rate_prescaler_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetBaudRatePrescaler(const hal_spi_handle_t *hspi,
                                          const hal_spi_baud_rate_prescaler_t baud_rate_prescaler)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_PRESCALER(baud_rate_prescaler));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  LL_SPI_SetBaudRatePrescaler((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)baud_rate_prescaler);

  return HAL_OK;
}

/**
  * @brief Retrieve the baud rate prescaler of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_baud_rate_prescaler_t Current SPI clock baud rate prescaler configuration.
  */
hal_spi_baud_rate_prescaler_t HAL_SPI_GetBaudRatePrescaler(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_baud_rate_prescaler_t)LL_SPI_GetBaudRatePrescaler((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set whether data transfers start from MSB or LSB bit.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param first_bit This parameter must be a value of hal_spi_first_bit_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetFirstBit(hal_spi_handle_t *hspi, const hal_spi_first_bit_t first_bit)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_FIRST_BIT(first_bit));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetTransferBitOrder((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)first_bit);

  return HAL_OK;
}

/**
  * @brief Retrieve the first bit (MSB or LSB bit) of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_first_bit_t Current SPI first bit configuration.
  */
hal_spi_first_bit_t HAL_SPI_GetFirstBit(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_first_bit_t)LL_SPI_GetTransferBitOrder((SPI_TypeDef *)((uint32_t)hspi->instance));
}
/**
  * @brief Set the management configuration of the NSS Pin.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param nss_pin_management This parameter must be a value of hal_spi_nss_pin_management_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetNSSPinManagement(hal_spi_handle_t *hspi, hal_spi_nss_pin_management_t nss_pin_management)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_NSS_PIN_MANAGEMENT(nss_pin_management));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetNSSMode((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)nss_pin_management);

  return HAL_OK;
}

/**
  * @brief Retrieve the NSS Pin management of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_nss_pin_management_t Current SPI NSS Pin management configuration.
  */
hal_spi_nss_pin_management_t HAL_SPI_GetNSSPinManagement(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_nss_pin_management_t)LL_SPI_GetNSSMode((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set the FIFO threshold level.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param fifo_threshold This parameter can be a value of hal_spi_fifo_threshold_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_SetFifoThreshold(const hal_spi_handle_t *hspi, const hal_spi_fifo_threshold_t fifo_threshold)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_PARAM(IS_SPI_FIFO_THRESHOLD(fifo_threshold));
#if defined(USE_ASSERT_DBG_PARAM)
  /* Get PacketSize */
  uint32_t packet_length = GET_PACKET_SIZE(LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance)),
                                           (uint32_t)fifo_threshold);
  ASSERT_DBG_PARAM(IS_SPI_PACKET_SIZE(packet_length));
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  LL_SPI_SetFIFOThreshold((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)fifo_threshold);

  return HAL_OK;
}

/**
  * @brief Retrieve the FIFO threshold level of the SPI peripheral.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_fifo_threshold_t Current FIFO threshold configuration.
  */
hal_spi_fifo_threshold_t HAL_SPI_GetFifoThreshold(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_fifo_threshold_t)LL_SPI_GetFIFOThreshold((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Set an extra delay, expressed in number of SPI clock cycle periods, inserted additionally.
  *        between active edge of SS and first data transaction start in master mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param nb_cycles This parameter can be a value of hal_spi_master_inter_data_idleness_delay_t.
  * @retval HAL_ERROR when the IO configuration register (SPI_CFG2) is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SPI_MASTER_SetInterDataIdlenessDelay(hal_spi_handle_t *hspi,
                                                      const hal_spi_master_inter_data_idleness_delay_t nb_cycles)
{
  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(IS_SPI_MIDI_DELAY(nb_cycles));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Check for IOLock */
  if (LL_SPI_IsEnabledIOLock((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    return HAL_ERROR;
  }

  LL_SPI_SetInterDataIdleness((SPI_TypeDef *)((uint32_t)hspi->instance), (uint32_t)nb_cycles);

  return HAL_OK;
}

/**
  * @brief Retrieve the extra delay, expressed in number of SPI clock cycle periods, inserted additionally between.
  *        active edge of SS and first data transaction start in master mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_master_inter_data_idleness_delay_t Current inter data idleness delay.
  */
hal_spi_master_inter_data_idleness_delay_t HAL_SPI_MASTER_GetInterDataIdlenessDelay(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT);

  return (hal_spi_master_inter_data_idleness_delay_t)LL_SPI_GetInterDataIdleness((SPI_TypeDef *)((
      uint32_t)hspi->instance));
}

#if defined(USE_HAL_SPI_USER_DATA) && (USE_HAL_SPI_USER_DATA == 1)

/**
  * @brief Store User Data pointer into the handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_user_data Pointer to the user data.
  */
void HAL_SPI_SetUserData(hal_spi_handle_t *hspi, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  hspi->p_user_data = p_user_data;
}

/**
  * @brief Retrieve User Data pointer from the handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval Pointer to the user data.
  */
const void *HAL_SPI_GetUserData(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != NULL);

  return (hspi->p_user_data);
}

#endif /* USE_HAL_SPI_USER_DATA */

#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
/**
  * @brief Link the Transmit DMA handle to the SPI handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure.
  * @param hdma Pointer to a hal_dma_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_SPI_SetTxDMA(hal_spi_handle_t *hspi, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Link the DMA handle to the SPI handle */
  hspi->hdma_tx = hdma;
  hdma->p_parent = hspi;

  return HAL_OK;
}

/**
  * @brief Link the Receive DMA handle to the SPI handle.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure.
  * @param hdma Pointer to a hal_dma_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_SPI_SetRxDMA(hal_spi_handle_t *hspi, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Link the DMA handle to the SPI handle */
  hspi->hdma_rx = hdma;
  hdma->p_parent = hspi;

  return HAL_OK;
}
#endif /* USE_HAL_SPI_DMA  */
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group6 IO operation functions
  * @{
    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

  The SPI supports master and slave mode :

  - There are two modes of transfer:
    - Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
    - Non-blocking mode: The communication is performed using Interrupts
            or DMA, these APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The HAL_SPI_TxCpltCallback(), HAL_SPI_RxCpltCallback() and HAL_SPI_TxRxCpltCallback() user callbacks
      will be executed respectively at the end of the Transmit or Receive process.
      The HAL_SPI_ErrorCallback() user callback will be executed when a communication error is detected.

  - APIs provided for these 2 transfer modes (Blocking mode or Non-blocking mode using either Interrupt or DMA)
     exist for simplex, half-duplex and full-duplex modes.
  */

/**
  * @brief Transmit an amount of data in blocking mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be sent.
  * @param timeout_ms Timeout duration.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  * @retval HAL_TIMEOUT Operation exceeds user timeout.
  */
hal_status_t HAL_SPI_Transmit(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet, uint32_t timeout_ms)
{
  uint32_t tickstart;
  uint32_t mode;
  uint32_t fifo_threshold;
  uint32_t data_width;
  hal_status_t status;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_TX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_ACTIVE);

  mode           = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
  fifo_threshold = LL_SPI_GetFIFOThreshold((SPI_TypeDef *)((uint32_t)hspi->instance));
  data_width     = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));

  tickstart = HAL_GetTick();

  /* Set the transaction information */
  hspi->p_tx_buff         = (const uint8_t *)p_data;
  hspi->tx_xfer_size      = (uint16_t)count_packet;
  hspi->tx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_rx_buff         = NULL;
  hspi->rx_xfer_size      = (uint16_t)0UL;
  hspi->rx_xfer_count     = (uint16_t)0UL;
  hspi->p_tx_isr          = NULL;
  hspi->p_rx_isr          = NULL;

  /* Configure communication direction : 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_TX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_TX);
  }

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  /* Transmit data in 32 Bit mode */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    /* Transmit data in 32 Bit mode */
    while (hspi->tx_xfer_count > 0UL)
    {
      /* Wait until TXP flag is set to send data */
      if (LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
      {
        LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
        hspi->p_tx_buff += sizeof(uint32_t);
        hspi->tx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Transmit data in 16 Bit mode */
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    /* Transmit data in 16 Bit mode */
    while (hspi->tx_xfer_count > 0UL)
    {
      /* Wait until TXP flag is set to send data */
      if (LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
      {
        if ((hspi->tx_xfer_count > 1UL) && (fifo_threshold > LL_SPI_FIFO_THRESHOLD_1_DATA))
        {
          LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
          hspi->p_tx_buff += sizeof(uint32_t);
          hspi->tx_xfer_count -= (uint16_t)2UL;
        }
        else
        {
          LL_SPI_TransmitData16(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint16_t *)hspi->p_tx_buff));
          hspi->p_tx_buff += sizeof(uint16_t);
          hspi->tx_xfer_count--;
        }
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Transmit data in 8 Bit mode */
  else
  {
    while (hspi->tx_xfer_count > 0UL)
    {
      /* Wait until TXP flag is set to send data */
      if (LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
      {
        if ((hspi->tx_xfer_count > 3UL) && (fifo_threshold > LL_SPI_FIFO_THRESHOLD_3_DATA))
        {
          LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
          hspi->p_tx_buff += sizeof(uint32_t);
          hspi->tx_xfer_count -= (uint16_t)4UL;
        }
        else if ((hspi->tx_xfer_count > 1UL) && (fifo_threshold > LL_SPI_FIFO_THRESHOLD_1_DATA))
        {
          LL_SPI_TransmitData16(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint16_t *)hspi->p_tx_buff));
          hspi->p_tx_buff += sizeof(uint16_t);
          hspi->tx_xfer_count -= (uint16_t)2UL;
        }
        else
        {
          LL_SPI_TransmitData8(((SPI_TypeDef *)((uint32_t)hspi->instance)), *hspi->p_tx_buff);
          hspi->p_tx_buff += sizeof(uint8_t);
          hspi->tx_xfer_count--;
        }
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Wait End Of Transfer flag */
  if (SPI_WaitEndOfTransfer(hspi, timeout_ms, tickstart) != HAL_OK)
  {
    status = HAL_TIMEOUT;
    return status;
  }
  /* Call standard close procedure with error check */
  status = SPI_CloseTransfer(hspi);
  return status;
}

/**
  * @brief Receive an amount of data in blocking mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be received.
  * @param timeout_ms Timeout duration in milliseconds.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  * @retval HAL_TIMEOUT Operation exceeds user timeout.
  */
hal_status_t HAL_SPI_Receive(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet, uint32_t timeout_ms)
{
  uint32_t tickstart;
  uint32_t mode;
  uint32_t data_width;
  hal_status_t status;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_RX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_RX_ACTIVE);

  mode       = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
  data_width = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));

  tickstart = HAL_GetTick();

  /* Set the transaction information */
  hspi->p_rx_buff         = (uint8_t *)p_data;
  hspi->rx_xfer_size      = (uint16_t)count_packet;
  hspi->rx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_tx_buff         = NULL;
  hspi->tx_xfer_size      = (uint16_t)0UL;
  hspi->tx_xfer_count     = (uint16_t)0UL;
  hspi->p_tx_isr          = NULL;
  hspi->p_rx_isr          = NULL;

  /* Configure communication direction: 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_RX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_RX);
  }

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  /* Receive data in 32 Bit mode */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    /* Transfer loop */
    while (hspi->rx_xfer_count > 0UL)
    {
      /* Check the RXWNE/EOT flag */
      if ((((SPI_TypeDef *)((uint32_t)hspi->instance))->SR & (SPI_SR_RXWNE | SPI_SR_EOT))  != 0UL)
      {
        *((uint32_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData32((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint32_t);
        hspi->rx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Receive data in 16 Bit mode */
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    /* Transfer loop */
    while (hspi->rx_xfer_count > 0UL)
    {
      /* Check the RXP flag */
      if (LL_SPI_IsActiveFlag_RXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
      {
        *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint16_t);
        hspi->rx_xfer_count--;
      }
      /* At the end of transfer, remove last packets from RX FIFO */
      else if ((hspi->rx_xfer_count > 0UL)
               && (LL_SPI_GetRxFIFOPackingLevel((SPI_TypeDef *)((uint32_t)hspi->instance)) != LL_SPI_RX_FIFO_0PACKET))
      {
        *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint16_t);
        hspi->rx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Receive data in 8 Bit mode */
  else
  {
    /* Transfer loop */
    while (hspi->rx_xfer_count > 0UL)
    {
      /* Check the RXP flag */
      if (LL_SPI_IsActiveFlag_RXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
      {
        *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint8_t);
        hspi->rx_xfer_count--;
      }
      /* At the end of transfer, remove last packets from RX FIFO */
      else if ((hspi->rx_xfer_count > 0UL)
               && (LL_SPI_GetRxFIFOPackingLevel((SPI_TypeDef *)((uint32_t)hspi->instance)) != LL_SPI_RX_FIFO_0PACKET))
      {
        *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint8_t);
        hspi->rx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          /* Call standard close procedure with error check */
          (void)SPI_CloseTransfer(hspi);
          return HAL_TIMEOUT;
        }
      }
    }
  }

#if (USE_HAL_SPI_CRC != 0UL)
  if (LL_SPI_IsEnabledCRC((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    /* Wait for crc data to be received */
    if (SPI_WaitEndOfTransfer(hspi, timeout_ms, tickstart)  != HAL_OK)
    {
      status = HAL_TIMEOUT;
      return status;
    }
  }
#endif /* USE_HAL_SPI_CRC */

  /* Call standard close procedure with error check */
  status = SPI_CloseTransfer(hspi);
  return status;
}

/**
  * @brief Transmit and Receive an amount of data in blocking mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_tx_data Pointer to transmission data buffer.
  * @param p_rx_data Pointer to reception data buffer.
  * @param count_packet Amount of data to be exchanged in full-duplex. The process
  *                      manages the same number of data for rx and tx transfer.
  * @param timeout_ms Timeout duration.
  * @retval HAL_OK      Operation started successfully.
  * @retval HAL_BUSY    Concurrent process ongoing.
  * @retval HAL_TIMEOUT Operation exceeds user timeout.
  */
hal_status_t HAL_SPI_TransmitReceive(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                     uint32_t count_packet, uint32_t timeout_ms)
{
  uint32_t tickstart;
  uint16_t initial_tx_xfer_count;
  uint16_t initial_rx_xfer_count;
  uint32_t mode;
  uint32_t data_width;
  hal_status_t status;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));

  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_FULL_DUPLEX(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_RX_ACTIVE);

  mode        = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
  data_width  = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));

  tickstart = HAL_GetTick();

  /* Set the transaction information */
  initial_tx_xfer_count   = (uint16_t) count_packet;
  initial_rx_xfer_count   = (uint16_t) count_packet;
  hspi->p_rx_buff         = (uint8_t *)p_rx_data;
  hspi->rx_xfer_count     = (uint16_t) count_packet;
  hspi->rx_xfer_size      = (uint16_t) count_packet;
  hspi->p_tx_buff         = (const uint8_t *)p_tx_data;
  hspi->tx_xfer_count     = (uint16_t) count_packet;
  hspi->tx_xfer_size      = (uint16_t) count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_rx_isr          = NULL;
  hspi->p_tx_isr          = NULL;

  LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_FULL_DUPLEX);

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  /* Transmit and Receive data in 32 Bit mode */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    while ((initial_tx_xfer_count > 0UL) || (initial_rx_xfer_count > 0UL))
    {
      /* Check TXP flag */
      if ((LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (initial_tx_xfer_count > 0UL))
      {
        LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
        hspi->p_tx_buff += sizeof(uint32_t);
        hspi->tx_xfer_count --;
        initial_tx_xfer_count = hspi->tx_xfer_count;
      }
      /* Check RXWNE/EOT flag */
      if (((((SPI_TypeDef *)((uint32_t)hspi->instance))->SR & (SPI_SR_RXWNE | SPI_SR_EOT))  != 0UL)
          && (initial_rx_xfer_count > 0UL))
      {
        *((uint32_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData32((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint32_t);
        hspi->rx_xfer_count --;
        initial_rx_xfer_count = hspi->rx_xfer_count;
      }
      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
      {
        /* Call standard close procedure with error check */
        (void)SPI_CloseTransfer(hspi);
        return HAL_TIMEOUT;
      }
    }
  }
  /* Transmit and Receive data in 16 Bit mode */
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    while ((initial_tx_xfer_count > 0UL) || (initial_rx_xfer_count > 0UL))
    {
      /* Check the TXP flag */
      if ((LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (initial_tx_xfer_count > 0UL))
      {
        LL_SPI_TransmitData16(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint16_t *)hspi->p_tx_buff));
        hspi->p_tx_buff += sizeof(uint16_t);
        hspi->tx_xfer_count--;
        initial_tx_xfer_count = hspi->tx_xfer_count;
      }

      /* Check the RXP flag */
      if ((LL_SPI_IsActiveFlag_RXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (initial_rx_xfer_count > 0UL))
      {
        *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint16_t);
        hspi->rx_xfer_count--;
        initial_rx_xfer_count = hspi->rx_xfer_count;
      }
      /* At the end of transfer, remove last packets from RX FIFO */
      else if ((hspi->rx_xfer_count > 0UL)
               && (LL_SPI_GetRxFIFOPackingLevel((SPI_TypeDef *)((uint32_t)hspi->instance)) != LL_SPI_RX_FIFO_0PACKET))
      {
        *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint16_t);
        hspi->rx_xfer_count--;
        initial_rx_xfer_count = hspi->rx_xfer_count;
      }
      else
      {
        /* Nothing to do */
      }

      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
      {
        /* Call standard close procedure with error check */
        (void)SPI_CloseTransfer(hspi);
        return HAL_TIMEOUT;
      }
    }
  }
  /* Transmit and Receive data in 8 Bit mode */
  else
  {
    while ((initial_tx_xfer_count > 0UL) || (initial_rx_xfer_count > 0UL))
    {
      /* Check the TXP flag */
      if ((LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (initial_tx_xfer_count > 0UL))
      {
        LL_SPI_TransmitData8(((SPI_TypeDef *)((uint32_t)hspi->instance)), *hspi->p_tx_buff);
        hspi->p_tx_buff += sizeof(uint8_t);
        hspi->tx_xfer_count--;
        initial_tx_xfer_count = hspi->tx_xfer_count;
      }

      /* Check the RXP flag */
      if ((LL_SPI_IsActiveFlag_RXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (initial_rx_xfer_count > 0UL))
      {
        *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint8_t);
        hspi->rx_xfer_count--;
        initial_rx_xfer_count = hspi->rx_xfer_count;
      }
      /* At the end of transfer, remove last packets from RX FIFO */
      else if ((hspi->rx_xfer_count > 0UL)
               && (LL_SPI_GetRxFIFOPackingLevel((SPI_TypeDef *)((uint32_t)hspi->instance)) != LL_SPI_RX_FIFO_0PACKET))
      {
        *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
        hspi->p_rx_buff += sizeof(uint8_t);
        hspi->rx_xfer_count--;
        initial_rx_xfer_count = hspi->rx_xfer_count;
      }
      else
      {
        /* Nothing to do */
      }

      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >=  timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
      {
        /* Call standard close procedure with error check */
        (void)SPI_CloseTransfer(hspi);
        return HAL_TIMEOUT;
      }
    }
  }

  /* Wait for Tx/Rx (and CRC) data to be sent/received */
  if (SPI_WaitEndOfTransfer(hspi, timeout_ms, tickstart) != HAL_OK)
  {
    status = HAL_TIMEOUT;
    return status;
  }

  /* Call standard close procedure with error check */
  status = SPI_CloseTransfer(hspi);
  return status;
}

/**
  * @brief Transmit an amount of data in non-blocking mode with Interrupt.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be sent.
  * @retval HAL_OK      Operation started successfully.
  * @retval HAL_BUSY    Concurrent process ongoing.
  */
hal_status_t HAL_SPI_Transmit_IT(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  /* Check the transfer size */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));

  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_TX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_ACTIVE);

  status      = HAL_OK;
  data_width  = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode        = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Set the transaction information */
  hspi->p_tx_buff         = (const uint8_t *)p_data;
  hspi->tx_xfer_size      = (uint16_t)count_packet;
  hspi->tx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_rx_buff         = NULL;
  hspi->p_rx_isr          = NULL;
  hspi->rx_xfer_size      = (uint16_t)0UL;
  hspi->rx_xfer_count     = (uint16_t)0UL;

  /* Configure communication direction : 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_TX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_TX);
  }

  /* Set the function for IT treatment */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->p_tx_isr = SPI_TxISR_32BIT;
  }
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->p_tx_isr = SPI_TxISR_16BIT;
  }
  else
  {
    hspi->p_tx_isr = SPI_TxISR_8BIT;
  }

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Enable EOT, TXP, FRE, MODF and UDR interrupts */
  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                  LL_SPI_IT_EOT | LL_SPI_IT_TXP | LL_SPI_IT_UDR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

/**
  * @brief Receive an amount of data in non-blocking mode with Interrupt.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be sent.
  * @retval HAL_OK      Operation started successfully.
  * @retval HAL_BUSY    Concurrent process ongoing.
  */
hal_status_t HAL_SPI_Receive_IT(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));

  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_RX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_RX_ACTIVE);

  status      = HAL_OK;
  data_width  = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode        = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Set the transaction information */
  hspi->p_rx_buff         = (uint8_t *)p_data;
  hspi->rx_xfer_size      = (uint16_t)count_packet;
  hspi->rx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_tx_buff         = NULL;
  hspi->p_tx_isr          = NULL;
  hspi->tx_xfer_size      = (uint16_t)0UL;
  hspi->tx_xfer_count     = (uint16_t)0UL;

  /* Configure communication direction : 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_RX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_RX);
  }

  /* Set the function for IT treatment */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->p_rx_isr = SPI_RxISR_32BIT;
  }
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->p_rx_isr = SPI_RxISR_16BIT;
  }
  else
  {
    hspi->p_rx_isr = SPI_RxISR_8BIT;
  }

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Enable EOT, RXP, FRE, MODF and OVR interrupts */
  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                  LL_SPI_IT_EOT | LL_SPI_IT_RXP | LL_SPI_IT_OVR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

/**
  * @brief Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_tx_data Pointer to transmission data buffer.
  * @param p_rx_data Pointer to reception data buffer.
  * @param count_packet Amount of data to be exchanged in full-duplex. The process
  *                      manages the same number of data for rx and tx transfer.
  * @retval HAL_OK    Operation started successfully.
  * @retval HAL_BUSY  Concurrent process ongoing.
  */
hal_status_t HAL_SPI_TransmitReceive_IT(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                        uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;
  uint32_t tmp_tx_xfer_count;

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));

  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_FULL_DUPLEX(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_RX_ACTIVE);

  status  = HAL_OK;
  data_width     = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode           = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Set the transaction information */
  hspi->p_tx_buff         = (const uint8_t *)p_tx_data;
  hspi->tx_xfer_size      = (uint16_t)count_packet;
  hspi->tx_xfer_count     = (uint16_t)count_packet;
  hspi->p_rx_buff         = (uint8_t *)p_rx_data;
  hspi->rx_xfer_size      = (uint16_t)count_packet;
  hspi->rx_xfer_count     = (uint16_t)count_packet;
  tmp_tx_xfer_count       = hspi->tx_xfer_count;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Set the function for IT treatment */
  if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->p_tx_isr        = SPI_TxISR_32BIT;
    hspi->p_rx_isr        = SPI_RxISR_32BIT;
  }
  else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->p_rx_isr        = SPI_RxISR_16BIT;
    hspi->p_tx_isr        = SPI_TxISR_16BIT;
  }
  else
  {
    hspi->p_rx_isr        = SPI_RxISR_8BIT;
    hspi->p_tx_isr        = SPI_TxISR_8BIT;
  }

  LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_FULL_DUPLEX);

  /* Set the number of data at current transfer */
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Fill in the TxFIFO */
  while ((LL_SPI_IsActiveFlag_TXP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL) && (tmp_tx_xfer_count != 0UL))
  {
    /* Transmit data in 32 Bit mode */
    if (data_width > LL_SPI_DATA_WIDTH_16_BIT)
    {
      LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
      hspi->p_tx_buff += sizeof(uint32_t);
      hspi->tx_xfer_count--;
      tmp_tx_xfer_count = hspi->tx_xfer_count;
    }
    /* Transmit data in 16 Bit mode */
    else if (data_width > LL_SPI_DATA_WIDTH_8_BIT)
    {
      LL_SPI_TransmitData16(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint16_t *)hspi->p_tx_buff));
      hspi->p_tx_buff += sizeof(uint16_t);
      hspi->tx_xfer_count--;
      tmp_tx_xfer_count = hspi->tx_xfer_count;
    }
    /* Transmit data in 8 Bit mode */
    else
    {
      LL_SPI_TransmitData8(((SPI_TypeDef *)((uint32_t)hspi->instance)), *hspi->p_tx_buff);
      hspi->p_tx_buff += sizeof(uint8_t);
      hspi->tx_xfer_count--;
      tmp_tx_xfer_count = hspi->tx_xfer_count;
    }
  }

  /* Enable EOT, DXP, UDR, OVR, FRE and MODF interrupts */
  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                  LL_SPI_IT_EOT | LL_SPI_IT_DXP | LL_SPI_IT_OVR | LL_SPI_IT_UDR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
/**
  * @brief Transmit an amount of data in non-blocking mode with DMA.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be sent.
  * @retval HAL_OK      Operation started successfully.
  * @retval HAL_BUSY    Concurrent process ongoing.
  * @retval HAL_ERROR   Operation completed with error.
  */
hal_status_t HAL_SPI_Transmit_DMA(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;
  hal_dma_direct_xfer_config_t p_dma_tx_config;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_tx_node_config;
  hal_dma_node_type_t  p_node_type;
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  /* Check the transfer size */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));

  /* Check Direction parameter */
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_TX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_ACTIVE);

  status      = HAL_OK;
  data_width  = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode        = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set the transaction information */
  hspi->p_tx_buff         = (const uint8_t *)p_data;
  hspi->tx_xfer_size      = (uint16_t)count_packet;
  hspi->tx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_rx_buff         = NULL;
  hspi->p_tx_isr          = NULL;
  hspi->p_rx_isr          = NULL;
  hspi->rx_xfer_size      = (uint16_t)0UL;
  hspi->rx_xfer_count     = (uint16_t)0UL;

  /* Configure communication direction : 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_TX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_TX);
  }

  /* Get DMA channel basic transfer configuration */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hspi->hdma_tx->p_head_node, &p_dma_tx_node_config, &p_node_type);
    p_dma_tx_config.src_data_width = p_dma_tx_node_config.xfer.src_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hspi->hdma_tx, &p_dma_tx_config);
  }
#else /* USE_HAL_DMA_LINKEDLIST */
  HAL_DMA_GetConfigDirectXfer(hspi->hdma_tx, &p_dma_tx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Packing mode management is enabled by the DMA settings */
  if (((data_width > LL_SPI_DATA_WIDTH_16_BIT) && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD))
      || ((data_width > LL_SPI_DATA_WIDTH_8_BIT)  && (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_BYTE)))
  {
    /* Restriction the DMA data received is not allowed in this mode */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Adjust XferCount according to DMA alignment / Data size */
  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_HALFWORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
    if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_WORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 3UL) >> 2UL;
    }
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_WORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
  }
  else
  {
    /* Adjustment done */
  }

  hspi->hdma_tx->p_xfer_halfcplt_cb = SPI_DMAHalfTransmitCplt;
  hspi->hdma_tx->p_xfer_cplt_cb = SPI_DMATransmitCplt;
  hspi->hdma_tx->p_xfer_error_cb = SPI_DMAError;

  /* Clear TXDMAEN bit */
  LL_SPI_DisableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->tx_xfer_count = (uint16_t)count_packet;
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->tx_xfer_count = (uint16_t)(count_packet * 2U);
  }
  else
  {
    hspi->tx_xfer_count = (uint16_t)(count_packet * 4U);
  }

  /* Enable the Tx DMA Stream/Channel */
  if (HAL_OK != HAL_DMA_StartPeriphXfer_IT_Opt(hspi->hdma_tx,
                                               (uint32_t)hspi->p_tx_buff,
                                               (uint32_t) &((SPI_TypeDef *)((uint32_t)hspi->instance))->TXDR,
                                               hspi->tx_xfer_count, HAL_DMA_OPT_IT_DEFAULT))
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Set the number of data at current transfer */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), 0UL);
  }
  else
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
  }
#else
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_SPI_EnableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));

  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_IT_UDR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

/**
  * @brief Receive an amount of data in non-blocking mode with DMA.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_data Pointer to data buffer.
  * @param count_packet Amount of data to be sent.
  * @retval HAL_OK   Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_Receive_DMA(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;
  hal_dma_direct_xfer_config_t p_dma_rx_config;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_rx_node_config;
  hal_dma_node_type_t  p_node_type;
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_RX_AVAILABLE(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_RX_ACTIVE);

  status  = HAL_OK;
  data_width     = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode           = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set the transaction information */
  hspi->p_rx_buff         = (uint8_t *)p_data;
  hspi->rx_xfer_size      = (uint16_t)count_packet;
  hspi->rx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_tx_buff         = NULL;
  hspi->p_rx_isr          = NULL;
  hspi->p_tx_isr          = NULL;
  hspi->tx_xfer_size      = (uint16_t)0UL;
  hspi->tx_xfer_count     = (uint16_t)0UL;

  /* Configure communication direction : 1Line */
  if (LL_SPI_IsHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    LL_SPI_SetHalfDuplexDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_HALF_DUPLEX_RX);
  }
  else
  {
    LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_SIMPLEX_RX);
  }

  /* Get DMA channel basic transfer configuration */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hspi->hdma_rx->p_head_node, &p_dma_rx_node_config, &p_node_type);
    p_dma_rx_config.dest_data_width = p_dma_rx_node_config.xfer.dest_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hspi->hdma_rx, &p_dma_rx_config);
  }
#else /* USE_HAL_DMA_LINKEDLIST */
  HAL_DMA_GetConfigDirectXfer(hspi->hdma_rx, &p_dma_rx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Packing mode management is enabled by the DMA settings */
  if (((data_width > LL_SPI_DATA_WIDTH_16_BIT) && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD))
      || ((data_width > LL_SPI_DATA_WIDTH_8_BIT)  && (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_BYTE)))
  {
    /* Restriction the DMA data received is not allowed in this mode */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Adjust XferCount according to DMA alignment / Data size */
  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_HALFWORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
    if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_WORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 3UL) >> 2UL;
    }
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_WORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
  }
  else
  {
    /* Adjustment done */
  }

  hspi->hdma_rx->p_xfer_halfcplt_cb = SPI_DMAHalfReceiveCplt;
  hspi->hdma_rx->p_xfer_cplt_cb = SPI_DMAReceiveCplt;
  hspi->hdma_rx->p_xfer_error_cb = SPI_DMAError;

  /* Clear RXDMAEN bit */
  LL_SPI_DisableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->rx_xfer_count = (uint16_t)count_packet;
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->rx_xfer_count = (uint16_t)(count_packet * 2U);
  }
  else
  {
    hspi->rx_xfer_count = (uint16_t)(count_packet * 4U);
  }

  /* Enable the Rx DMA Stream/Channel */
  if (HAL_OK != HAL_DMA_StartPeriphXfer_IT_Opt(hspi->hdma_rx,
                                               (uint32_t) &((SPI_TypeDef *)((uint32_t)hspi->instance))->RXDR,
                                               (uint32_t)hspi->p_rx_buff,
                                               hspi->rx_xfer_count, HAL_DMA_OPT_IT_DEFAULT))
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Set the number of data at current transfer */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), 0UL);
  }
  else
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
  }
#else
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_SPI_EnableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));

  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_IT_OVR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

/**
  * @brief Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_tx_data Pointer to transmission data buffer.
  * @param p_rx_data Pointer to reception data buffer.
  * @param count_packet Amount of data to be exchanged in full-duplex. The process
  *                      manages the same number of data for rx and tx transfer.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_TransmitReceive_DMA(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                         uint32_t count_packet)
{
  hal_status_t status;
  uint32_t data_width;
  uint32_t mode;
  hal_dma_direct_xfer_config_t p_dma_tx_config;
  hal_dma_direct_xfer_config_t p_dma_rx_config;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_tx_node_config;
  hal_dma_node_config_t p_dma_rx_node_config;
  hal_dma_node_type_t  p_node_type;
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_PARAM(hspi != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (count_packet == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_PARAM(IS_SPI_TRANSFER_SIZE(count_packet));
  ASSERT_DBG_PARAM(IS_SPI_DIRECTION_FULL_DUPLEX(hspi->direction));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE);

  /* Critical section */
  HAL_CHECK_UPDATE_STATE(hspi, global_state, HAL_SPI_STATE_IDLE, HAL_SPI_STATE_TX_RX_ACTIVE);

  status     = HAL_OK;
  data_width = LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance));
  mode       = LL_SPI_GetMode((SPI_TypeDef *)((uint32_t)hspi->instance));
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set the transaction information */
  hspi->p_tx_buff         = (const uint8_t *)p_tx_data;
  hspi->tx_xfer_size      = (uint16_t)count_packet;
  hspi->tx_xfer_count     = (uint16_t)count_packet;
  hspi->p_rx_buff         = (uint8_t *)p_rx_data;
  hspi->rx_xfer_size      = (uint16_t)count_packet;
  hspi->rx_xfer_count     = (uint16_t)count_packet;
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_NONE;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  /* Init field not used in handle to zero */
  hspi->p_rx_isr          = NULL;
  hspi->p_tx_isr          = NULL;

  LL_SPI_SetTransferDirection((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_FULL_DUPLEX);

  /* Get DMA channel basic transfer configuration */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hspi->hdma_tx->p_head_node, &p_dma_tx_node_config, &p_node_type);
    p_dma_tx_config.src_data_width = p_dma_tx_node_config.xfer.src_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hspi->hdma_tx, &p_dma_tx_config);
  }
#else /* USE_HAL_DMA_LINKEDLIST */
  HAL_DMA_GetConfigDirectXfer(hspi->hdma_tx, &p_dma_tx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hspi->hdma_rx->p_head_node, &p_dma_rx_node_config, &p_node_type);
    p_dma_rx_config.dest_data_width = p_dma_rx_node_config.xfer.dest_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hspi->hdma_rx, &p_dma_rx_config);
  }
#else /* USE_HAL_DMA_LINKEDLIST */
  HAL_DMA_GetConfigDirectXfer(hspi->hdma_rx, &p_dma_rx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Reset the Tx/Rx DMA bits */
  LL_SPI_DisableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));
  LL_SPI_DisableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Packing mode management is enabled by the DMA settings */
  if (((data_width > LL_SPI_DATA_WIDTH_16_BIT)
       && ((p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD)
           || (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD)))
      || ((data_width > LL_SPI_DATA_WIDTH_8_BIT)
          && ((p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_BYTE)
              || (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_BYTE))))
  {
    /* Restriction the DMA data received is not allowed in this mode */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return status;
  }

  /* Adjust XferCount according to DMA alignment / Data size */
  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_HALFWORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
    else if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_WORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 3UL) >> 2UL;
    }
    else
    {
      /* Nothing to do */
    }
    if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_HALFWORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
    else if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_WORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 3UL) >> 2UL;
    }
    else
    {
      /* Nothing to do */
    }
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    if (p_dma_tx_config.src_data_width == HAL_DMA_SRC_DATA_WIDTH_WORD)
    {
      hspi->tx_xfer_count = (hspi->tx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
    if (p_dma_rx_config.dest_data_width == HAL_DMA_DEST_DATA_WIDTH_WORD)
    {
      hspi->rx_xfer_count = (hspi->rx_xfer_count + (uint16_t) 1UL) >> 1UL;
    }
  }
  else
  {
    /* Adjustment done */
  }

  hspi->hdma_rx->p_xfer_cplt_cb     = SPI_DMATransmitReceiveCplt;
  hspi->hdma_rx->p_xfer_halfcplt_cb = SPI_DMAHalfTransmitReceiveCplt;
  hspi->hdma_rx->p_xfer_error_cb    = SPI_DMAError;

  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->rx_xfer_count = (uint16_t)count_packet;
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->rx_xfer_count = (uint16_t)(count_packet * 2U);
  }
  else
  {
    hspi->rx_xfer_count = (uint16_t)(count_packet * 4U);
  }
  /* Enable the Rx DMA Stream/Channel  */
  if (HAL_OK != HAL_DMA_StartPeriphXfer_IT_Opt(hspi->hdma_rx,
                                               (uint32_t) &((SPI_TypeDef *)((uint32_t)hspi->instance))->RXDR,
                                               (uint32_t)hspi->p_rx_buff,
                                               hspi->rx_xfer_count, HAL_DMA_OPT_IT_DEFAULT))
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Set the SPI Tx DMA transfer complete callback because the communication closing
  is performed in DMA reception complete callback  */
  hspi->hdma_tx->p_xfer_cplt_cb     = SPI_DMAEmptyCallback;
  hspi->hdma_tx->p_xfer_halfcplt_cb = SPI_DMAEmptyCallback;
  hspi->hdma_tx->p_xfer_error_cb    = SPI_DMAError;

  if (data_width <= LL_SPI_DATA_WIDTH_8_BIT)
  {
    hspi->tx_xfer_count = (uint16_t)count_packet;
  }
  else if (data_width <= LL_SPI_DATA_WIDTH_16_BIT)
  {
    hspi->tx_xfer_count = (uint16_t)(count_packet * 2U);
  }
  else
  {
    hspi->tx_xfer_count = (uint16_t)(count_packet * 4U);
  }

  /* Enable the Tx DMA Stream/Channel  */
  if (HAL_OK != HAL_DMA_StartPeriphXfer_IT_Opt(hspi->hdma_tx,
                                               (uint32_t)hspi->p_tx_buff,
                                               (uint32_t) &((SPI_TypeDef *)((uint32_t)hspi->instance))->TXDR,
                                               hspi->tx_xfer_count, HAL_DMA_OPT_IT_DEFAULT))
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    hspi->last_error_codes = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    hspi->global_state = HAL_SPI_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Set the number of data at current transfer */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hspi->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), 0UL);
  }
  else
  {
    LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
  }
#else
  LL_SPI_SetTransferSize((SPI_TypeDef *)((uint32_t)hspi->instance), count_packet);
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_SPI_EnableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));

  LL_SPI_EnableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));

  LL_SPI_EnableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                  LL_SPI_IT_OVR | LL_SPI_IT_UDR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  LL_SPI_Enable((SPI_TypeDef *)((uint32_t)hspi->instance));

  if (mode == LL_SPI_MODE_MASTER)
  {
    LL_SPI_StartMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));
  }

  return status;
}

#endif /* USE_HAL_SPI_DMA */

/**
  * @brief Abort ongoing transfer (blocking mode).
  * @param hspi SPI handle.
  * @note  This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *        started in Interrupt or DMA mode.
  * @note  This procedure performs following operations :
  *         + Disable SPI Interrupts (depending of transfer direction).
  *         + Disable the DMA transfer in the peripheral register (if enabled).
  *         + Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode).
  *         + Set handle State to READY.
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @note   After the abort, other process (Tx, Rx or TxRx) can be started.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_Abort(hal_spi_handle_t *hspi)
{
  hal_status_t status;

  volatile uint32_t count;

  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE | HAL_SPI_STATE_TX_RX_ACTIVE);

  /* Set hspi->state to aborting to avoid any interaction */
  hspi->global_state = HAL_SPI_STATE_ABORT;

  status = HAL_OK;
  count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);

  /* If master communication on going, make sure current frame is done before closing the connection */
  if (LL_SPI_IsActiveMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
  {
    LL_SPI_DisableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes  = HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsEnabledIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U);

    LL_SPI_SuspendMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));

    count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes  = HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsActiveMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U);

    LL_SPI_ClearFlag_SUSP((SPI_TypeDef *)((uint32_t)hspi->instance));
    count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes  = HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsActiveFlag_SUSP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL);
  }

#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  /* Disable the SPI DMA Tx request if enabled */
  if (LL_SPI_IsEnabledDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
  {
    if (hspi->hdma_tx != NULL)
    {
      /* Abort DMA Tx Handle linked to SPI peripheral */
      if (HAL_DMA_Abort(hspi->hdma_tx) != HAL_OK)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes  = HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
  }

  /* Disable the SPI DMA Rx request if enabled */
  if (LL_SPI_IsEnabledDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
  {
    if (hspi->hdma_rx != NULL)
    {
      /* Abort DMA Rx Handle linked to SPI peripheral */
      if (HAL_DMA_Abort(hspi->hdma_rx) != HAL_OK)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes  = HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
  }
#endif /* USE_HAL_SPI_DMA */

  SPI_AbortTransfer(hspi);

  hspi->global_state = HAL_SPI_STATE_IDLE;

  return status;
}

/**
  * @brief Abort ongoing transfer (Interrupt mode).
  * @param hspi SPI handle.
  * @note  This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *        started in Interrupt or DMA mode.
  * @note  This procedure performs following operations :
  *         + Disable SPI Interrupts (depending of transfer direction).
  *         + Disable the DMA transfer in the peripheral register (if enabled).
  *         + Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode).
  *         + Set handle State to READY.
  *         + At abort completion, call user abort complete callback.
  * @note  This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *        considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_Abort_IT(hal_spi_handle_t *hspi)
{
  hal_status_t status;
  volatile uint32_t count;
#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  uint8_t dma_used;
#endif /* USE_HAL_SPI_DMA */

  ASSERT_DBG_PARAM(hspi != NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE | HAL_SPI_STATE_TX_RX_ACTIVE);

  /* Set hspi->state to aborting to avoid any interaction */
  hspi->global_state = HAL_SPI_STATE_ABORT;

  status = HAL_OK;
  count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);
#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  dma_used = 0U;
#endif /* USE_HAL_SPI_DMA */

  /* If master communication on going, make sure current frame is done before closing the connection */
  if (LL_SPI_IsActiveMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
  {
    LL_SPI_DisableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes =  HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsEnabledIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U);

    LL_SPI_SuspendMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance));

    count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes =  HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsActiveMasterTransfer((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U);

    LL_SPI_ClearFlag_SUSP((SPI_TypeDef *)((uint32_t)hspi->instance));

    count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24UL / 1000UL);
    do
    {
      count--;
      if (count == 0UL)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes =  HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
        break;
      }
    } while (LL_SPI_IsActiveFlag_SUSP((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL);
  }

#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  /* If DMA Tx and/or DMA Rx Handles are associated to SPI Handle, DMA Abort complete callbacks must be initialized
     before any call to DMA Abort functions */

  if (hspi->hdma_rx != NULL)
  {
    if (LL_SPI_IsEnabledDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
    {
      /* Set DMA Abort Complete callback if SPI DMA Rx request if enabled */
      hspi->hdma_rx->p_xfer_abort_cb = SPI_DMARxAbortCallback;
    }
    else
    {
      hspi->hdma_rx->p_xfer_abort_cb = NULL;
    }
  }

  if (hspi->hdma_tx != NULL)
  {
    if (LL_SPI_IsEnabledDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
    {
      /* Set DMA Abort Complete callback if SPI DMA Tx request if enabled */
      hspi->hdma_tx->p_xfer_abort_cb = SPI_DMATxAbortCallback;

      /* Set dma_used to 1 to make sure abort complete callback is called through DMA */
      dma_used = 1U;

      /* Abort DMA Tx Handle linked to SPI peripheral */
      if (HAL_DMA_Abort_IT(hspi->hdma_tx) != HAL_OK)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes =  HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
    else
    {
      hspi->hdma_tx->p_xfer_abort_cb = NULL;
    }
  }

  if (hspi->hdma_rx != NULL)
  {
    if (LL_SPI_IsEnabledDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0U)
    {
      /* Set dma_used to 1 to make sure abort complete callback is called through DMA */
      dma_used = 1U;

      /* Abort DMA Rx Handle linked to SPI peripheral */
      if (HAL_DMA_Abort_IT(hspi->hdma_rx) != HAL_OK)
      {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
        hspi->last_error_codes =  HAL_SPI_ERROR_ABORT;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
  }

  if ((hspi->global_state != HAL_SPI_STATE_IDLE) && (dma_used == 0U))
  {
    SPI_AbortTransfer(hspi);

    hspi->global_state = HAL_SPI_STATE_IDLE;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
    hspi->p_abort_cplt_cb(hspi);
#else
    HAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
  }
#else /* USE_HAL_SPI_DMA */
  /* Proceed with abort procedure */
  SPI_AbortTransfer(hspi);

  hspi->global_state = HAL_SPI_STATE_IDLE;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_abort_cplt_cb(hspi);
#else
  HAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#endif /* USE_HAL_SPI_DMA */

  return status;
}

/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group7 IRQ Handler/Callbacks/Register Callbacks functions
  * @{
  This subsection provides a set of functions allowing to register the SPI process and error callbacks:

    - The function HAL_SPI_IRQHandler() to handle all SPI interrupts

  There are two ways to use callbacks:
  Override weak callbacks functions:
    - Call the function HAL_SPI_TxCpltCallback() to indicate Tx Transfer is completed.
    - Call the function HAL_SPI_RxCpltCallback() to indicate Rx Transfer is completed.
    - Call the function HAL_SPI_TxRxCpltCallback() to indicate Tx/Rx Transfer is completed.
    - Call the function HAL_SPI_TxHalfCpltCallback() to indicate Tx Half Transfer is completed.
    - Call the function HAL_SPI_RxHalfCpltCallback() to indicate Rx Half Transfer is completed.
    - Call the function HAL_SPI_TxRxHalfCpltCallback() to indicate Tx/Rx Half Transfer is completed.
    - Call the function HAL_SPI_ErrorCallback() to indicate invalidate operation is completed.
    - Call the function HAL_SPI_AbortCpltCallback() to indicate Abort operation is completed.
    - Call the function HAL_SPI_SuspendCallback() to indicate when an operation is suspended.

  Or register callbacks user:
    - Call the function HAL_SPI_RegisterTxCpltCallback() to register the Tx transfer complete Callback.
    - Call the function HAL_SPI_RegisterRxCpltCallback() to register the Rx transfer complete Callback.
    - Call the function HAL_SPI_RegisterTxRxCpltCallback() to register the Tx/Rx transfer complete Callback.
    - Call the function HAL_SPI_RegisterTxHalfCpltCallback() to register the Tx Half transfer complete Callback.
    - Call the function HAL_SPI_RegisterRxHalfCpltCallback() to register the Rx Half transfer complete Callback.
    - Call the function HAL_SPI_RegisterTxRxHalfCpltCallback() to register the Tx/Rx Half transfer complete Callback.
    - Call the function HAL_SPI_RegisterErrorCallback() to register the Error Callback.
    - Call the function HAL_SPI_RegisterAbortCpltCallback() to register the Abort operation Callback.
    - Call the function HAL_SPI_RegisterSuspendCallback() to register the SPI Suspend Callback.

  HAL_SPI_IRQHandler() is designed to process the different interruptions :
    - Error interruptions during transfer (OVR, UDR, MODF, TIFRE)
    - Transfer interruptions (DXP, RXP, TXP, EOT)

  Depending on the process function one's use, different callback might be triggered:

| Process API \n \ \n Callbacks | HAL_SPI_Transmit_IT  | HAL_SPI_Receive_IT | HAL_SPI_TransmitReceive_IT |
|-------------------------------|:--------------------:|:------------------:|:--------------------------:|
| HAL_SPI_TxCpltCallback        |           x          |                    |                            |
| HAL_SPI_RxCpltCallback        |                      |          x         |                            |
| HAL_SPI_TxRxCpltCallback      |                      |                    |              x             |
| HAL_SPI_SuspendCallback       |           x          |          x         |              x             |
| HAL_SPI_ErrorCallback         |           x          |          x         |              x             |

| Process API \n \ \n Callbacks  | HAL_SPI_Transmit_DMA  | HAL_SPI_Receive_DMA  | HAL_SPI_TransmitReceive_DMA |
|--------------------------------|:---------------------:|:--------------------:|:---------------------------:|
| HAL_SPI_TxHalfCpltCallback*    |           x           |                      |                             |
| HAL_SPI_TxCpltCallback         |           x           |                      |                             |
| HAL_SPI_RxHalfCpltCallback*    |                       |           x          |                             |
| HAL_SPI_RxCpltCallback         |                       |           x          |                             |
| HAL_SPI_TxRxHalfCpltCallback*  |                       |                      |              x              |
| HAL_SPI_TxRxCpltCallback       |                       |                      |              x              |
| HAL_SPI_ErrorCallback**        |           x           |           x          |              x              |
@note * these callbacks might be called following DMA IRQ management, not SPIx IRQ management.
@note ** these callbacks might be called following DMA IRQ management, or SPIx IRQ management.

| Process API \n \ \n Callbacks      | HAL_SPI_Abort_IT  |
|------------------------------------|:-----------------:|
| HAL_SPI_AbortCpltCallback          |         x         |
| HAL_SPI_ErrorCallback              |         x         |

  */

/**
  * @brief Handle SPI interrupt request.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
void HAL_SPI_IRQHandler(hal_spi_handle_t *hspi)
{
  uint32_t it_source = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), IER);
  uint32_t it_flag   = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), SR);
  uint32_t trigger  = it_source & it_flag;
#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  uint32_t cfg1_reg_value = LL_SPI_READ_REG(((SPI_TypeDef *)((uint32_t)hspi->instance)), CFG1);
#endif /* USE_HAL_SPI_DMA */
  uint32_t handled  = 0UL;

  hal_spi_state_t tmp_global_state = hspi->global_state;

  /* SPI in SUSPEND mode  ----------------------------------------------------*/
  if (STM32_IS_BIT_SET(it_flag, SPI_SR_SUSP) && STM32_IS_BIT_SET(it_source, SPI_SR_EOT))
  {
    /* Clear the Suspend flag */
    LL_SPI_ClearFlag_SUSP((SPI_TypeDef *)((uint32_t)hspi->instance));

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
    hspi->p_suspend_cb(hspi);
#else
    HAL_SPI_SuspendCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    return;
  }

  /* SPI in mode Transmitter and Receiver ------------------------------------*/
  if ((STM32_IS_BIT_CLR(trigger, SPI_SR_OVR) && STM32_IS_BIT_CLR(trigger, SPI_SR_UDR))
      && (STM32_IS_BIT_SET(trigger, SPI_SR_DXP)))
  {
    hspi->p_tx_isr(hspi);
    hspi->p_rx_isr(hspi);
    handled = 1UL;
  }

  /* SPI in mode Receiver ----------------------------------------------------*/
  if ((STM32_IS_BIT_CLR(trigger, SPI_SR_OVR) && STM32_IS_BIT_SET(trigger, SPI_SR_RXP))
      && (STM32_IS_BIT_CLR(trigger, SPI_SR_DXP)))
  {
    hspi->p_rx_isr(hspi);
    handled = 1UL;
  }

  /* SPI in mode Transmitter -------------------------------------------------*/
  if ((STM32_IS_BIT_CLR(trigger, SPI_SR_UDR) && STM32_IS_BIT_SET(trigger, SPI_SR_TXP))
      && (STM32_IS_BIT_CLR(trigger, SPI_SR_DXP)))
  {
    hspi->p_tx_isr(hspi);
    handled = 1UL;
  }


  if (handled != 0UL)
  {
    return;
  }

  /* SPI End Of Transfer: DMA or IT based transfer */
  if (STM32_IS_BIT_SET(trigger, SPI_SR_EOT))
  {
    /* Clear EOT/TXTF/SUSP flag */
    LL_SPI_ClearFlag((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_FLAG_EOT | LL_SPI_FLAG_TXTF | LL_SPI_FLAG_SUSP);

    LL_SPI_DisableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));

    /* For the IT based receive extra polling maybe required for last packet */
    if (STM32_IS_BIT_CLR(((SPI_TypeDef *)((uint32_t)hspi->instance))->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN))
    {
      /* Polling remaining data */
      while (hspi->rx_xfer_count != 0UL)
      {
        /* Receive data in 32 Bit mode */
        if (LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance)) > LL_SPI_DATA_WIDTH_16_BIT)
        {
          *((uint32_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData32((SPI_TypeDef *)((uint32_t)hspi->instance));
          hspi->p_rx_buff += sizeof(uint32_t);
        }
        /* Receive data in 16 Bit mode */
        else if (LL_SPI_GetDataWidth((SPI_TypeDef *)((uint32_t)hspi->instance)) > LL_SPI_DATA_WIDTH_8_BIT)
        {
          *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
          hspi->p_rx_buff += sizeof(uint16_t);
        }
        /* Receive data in 8 Bit mode */
        else
        {
          *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
          hspi->p_rx_buff += sizeof(uint8_t);
        }

        hspi->rx_xfer_count--;
      }
    }

    if (SPI_CloseTransfer(hspi) != HAL_OK)
    {
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
      hspi->p_error_cb(hspi);
#else
      HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
    /* Call appropriate user callback */
    if (tmp_global_state == HAL_SPI_STATE_TX_RX_ACTIVE)
    {
      hspi->p_tx_rx_cplt_cb(hspi);
    }
    else if (tmp_global_state == HAL_SPI_STATE_RX_ACTIVE)
    {
      hspi->p_rx_cplt_cb(hspi);
    }
    else if (tmp_global_state == HAL_SPI_STATE_TX_ACTIVE)
    {
      hspi->p_tx_cplt_cb(hspi);
    }
#else
    /* Call appropriate user callback */
    if (tmp_global_state == HAL_SPI_STATE_TX_RX_ACTIVE)
    {
      HAL_SPI_TxRxCpltCallback(hspi);
    }
    else if (tmp_global_state == HAL_SPI_STATE_RX_ACTIVE)
    {
      HAL_SPI_RxCpltCallback(hspi);
    }
    else if (tmp_global_state == HAL_SPI_STATE_TX_ACTIVE)
    {
      HAL_SPI_TxCpltCallback(hspi);
    }
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    else
    {
      /* End of the appropriate call */
    }

    return;
  }

  /* SPI in Error Treatment --------------------------------------------------*/
  if ((trigger & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_TIFRE | SPI_SR_UDR)) != 0UL)
  {
    /* SPI Overrun error interrupt occurred ----------------------------------*/
    if ((trigger & SPI_SR_OVR) != 0UL)
    {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_OVR);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_OVR((SPI_TypeDef *)((uint32_t)hspi->instance));
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if ((trigger & SPI_SR_MODF) != 0UL)
    {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_MODF);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_MODF((SPI_TypeDef *)((uint32_t)hspi->instance));

      /* Enter in mode fault state */
      hspi->global_state = HAL_SPI_STATE_FAULT;
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if ((trigger & SPI_SR_TIFRE) != 0UL)
    {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_FRE);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_FRE((SPI_TypeDef *)((uint32_t)hspi->instance));
    }

    /* SPI Underrun error interrupt occurred ------------------------------------*/
    if ((trigger & SPI_SR_UDR) != 0UL)
    {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_UDR);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_UDR((SPI_TypeDef *)((uint32_t)hspi->instance));
    }

    /* Disable SPI peripheral */
    LL_SPI_Disable((SPI_TypeDef *)((uint32_t)hspi->instance));

    /* Disable all interrupts */
    LL_SPI_DisableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                     LL_SPI_IT_EOT | LL_SPI_IT_TXP | LL_SPI_IT_RXP | LL_SPI_IT_DXP |
                     LL_SPI_IT_UDR | LL_SPI_IT_OVR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
    /* Disable the SPI DMA requests if enabled */
    if ((STM32_IS_BIT_SET(cfg1_reg_value, SPI_CFG1_RXDMAEN)) || (STM32_IS_BIT_SET(cfg1_reg_value, SPI_CFG1_TXDMAEN)))
    {
      if (STM32_IS_BIT_SET(cfg1_reg_value, SPI_CFG1_RXDMAEN))
      {
        /* Disable the SPI DMA requests */
        LL_SPI_DisableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));

        /* Abort the SPI DMA Rx channel */
        if (hspi->hdma_rx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdma_rx->p_xfer_abort_cb = SPI_DMAAbortOnError;
          if (HAL_OK != HAL_DMA_Abort_IT(hspi->hdma_rx))
          {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
            STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_ABORT);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
          }
        }
      }
      if (STM32_IS_BIT_SET(cfg1_reg_value, SPI_CFG1_TXDMAEN))
      {
        /* Disable the SPI DMA requests */
        LL_SPI_DisableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));

        /* Abort the SPI DMA Tx channel */
        if (hspi->hdma_tx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdma_tx->p_xfer_abort_cb = SPI_DMAAbortOnError;
          if (HAL_OK != HAL_DMA_Abort_IT(hspi->hdma_tx))
          {
#if defined (USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
            STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_ABORT);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
          }
        }
      }
    }
    else
    {
#endif /* USE_HAL_SPI_DMA */
      if (hspi->global_state != HAL_SPI_STATE_FAULT)
      {
        hspi->global_state = HAL_SPI_STATE_IDLE;
      }

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
      hspi->p_error_cb(hspi);
#else
      HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
    }
#endif /* USE_HAL_SPI_DMA */
  }
}

/**
  * @brief Tx Transfer completed callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_TxCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief Rx Transfer completed callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_RxCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief Tx and Rx Transfer completed callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_TxRxCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief Tx Half Transfer completed callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_TxHalfCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief Rx Half Transfer completed callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_RxHalfCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief Tx and Rx Half Transfer callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_TxRxHalfCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief SPI error callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_ErrorCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief SPI Abort Complete callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_AbortCpltCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

/**
  * @brief SPI Suspend callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @warning This weak function must not be modified. When the callback
  *          is needed, it is overridden in the user file.
  */
__WEAK void HAL_SPI_SuspendCallback(hal_spi_handle_t *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hspi);
}

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)

/**
  * @brief Register the SPI Tx Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Tx Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterTxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Rx Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Rx Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterRxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Tx/Rx Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Tx/Rx Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterTxRxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_tx_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Tx half Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Tx half Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterTxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_tx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Rx half Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Rx half Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterRxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_rx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI TxRx half Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Rx half Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterTxRxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_tx_rx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Error Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Error Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterErrorCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Abort Cplt Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Abort Cplt Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterAbortCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register the SPI Suspend Callback.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param p_callback Pointer to the Error Callback function.
  * @retval HAL_INVALID_PARAM invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_SPI_RegisterSuspendCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hspi != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hspi->global_state, (uint32_t)HAL_SPI_STATE_INIT | (uint32_t)HAL_SPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hspi->p_suspend_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group8 Peripheral current frequency, state and errors functions
  * @{
  * This subsection provides 3 functions allowing to read peripheral current frequency, state and last occurred errors.
  *  - HAL_SPI_GetClockFreq() API to retrieve the current clock frequency of the SPI peripheral.
  *  - HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral.
  *  - HAL_SPI_GetLastErrorCodes() API to retrieve the error codes in case of HAL_ERROR return
  *    available under the compilation switch USE_HAL_SPI_GET_LAST_ERRORS.
  */

/** @brief Return the peripheral clock frequency for SPI.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval uint32_t Frequency in Hz.
  * @retval 0 source clock of the hspi not configured or not ready.
  */
uint32_t HAL_SPI_GetClockFreq(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM((hspi != NULL));

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_ABORT);

  return HAL_RCC_SPI_GetKernelClkFreq((SPI_TypeDef *)((uint32_t)hspi->instance));
}

/**
  * @brief Retrieve the SPI handle state.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval hal_spi_state_t SPI state.
  */
hal_spi_state_t HAL_SPI_GetState(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM((hspi != NULL));

  return hspi->global_state;
}

#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
/**
  * @brief Retrieve the SPI errors codes.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval uint32_t Returned value can be a combination of the following values:
  *         @arg HAL_SPI_ERROR_NONE
  *         @arg HAL_SPI_ERROR_MODF
  *         @arg HAL_SPI_ERROR_CRC
  *         @arg HAL_SPI_ERROR_OVR
  *         @arg HAL_SPI_ERROR_FRE
  *         @arg HAL_SPI_ERROR_DMA
  *         @arg HAL_SPI_ERROR_ABORT
  *         @arg HAL_SPI_ERROR_UDR

  */
uint32_t HAL_SPI_GetLastErrorCodes(const hal_spi_handle_t *hspi)
{
  ASSERT_DBG_PARAM(hspi != (void *)NULL);

  ASSERT_DBG_STATE(hspi->global_state, HAL_SPI_STATE_IDLE | HAL_SPI_STATE_TX_ACTIVE | HAL_SPI_STATE_RX_ACTIVE
                   | HAL_SPI_STATE_TX_RX_ACTIVE | HAL_SPI_STATE_FAULT | HAL_SPI_STATE_ABORT
                   | HAL_SPI_STATE_INIT);

  return hspi->last_error_codes;
}
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)

/** @addtogroup SPI_Exported_Functions_Group9 Acquire/release Bus functions
  * @{
  * This subsection provides a set of functions allowing to Acquire/Release the bus based on the HAL OS
  * abstraction layer (stm32_hal_os.c/.h osal):

  * - The HAL_SPI_AcquireBus() must be called from thread mode only (not from handler mode i.e from ISR).
  * - The HAL_SPI_ReleaseBus() can be called from thread mode or from handler mode i.e from ISR.
 */

/**
  * @brief Acquire the SPI bus thanks to the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param timeout_ms Time to wait before the bus is occupied by the handle.
  * @note The HAL_SPI_AcquireBus function must be called from thread mode only
  *       (not from handler mode i.e from ISR).
  * @retval HAL_OK    Operation started successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_AcquireBus(hal_spi_handle_t *hspi, uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hspi != NULL));

  if (HAL_OS_SemaphoreTake(&hspi->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief Release the SPI bus thanks to the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @note The HAL_SPI_ReleaseBus function can be called from thread mode or from handler mode i.e from ISR
  * @retval HAL_OK      Operation started successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SPI_ReleaseBus(hal_spi_handle_t *hspi)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hspi != NULL));

  if (HAL_OS_SemaphoreRelease(&hspi->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */

#endif /* USE_HAL_MUTEX  */

/**
  * @}
  */

/** @addtogroup SPI_Private_Functions SPI Private Functions
  * @{
  */

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
/**
  * @brief DMA SPI transmit process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMATransmitCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  if (hspi->global_state != HAL_SPI_STATE_ABORT)
  {
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    if (hspi->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
    {
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
      hspi->p_tx_cplt_cb(hspi);
#else
      HAL_SPI_TxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
    else
    {
      LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
    }
#else
    LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_DMA_LINKEDLIST */
  }
}

/**
  * @brief DMA SPI receive process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  if (hspi->global_state != HAL_SPI_STATE_ABORT)
  {
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    if (hspi->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
    {
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
      hspi->p_rx_cplt_cb(hspi);
#else
      HAL_SPI_RxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
    else
    {
      LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
    }
#else
    LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_DMA_LINKEDLIST */
  }
}

/**
  * @brief DMA SPI transmit receive process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMATransmitReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_xfer_mode_t hdma_tx_xfer_mode = hspi->hdma_tx->xfer_mode;
  hal_dma_xfer_mode_t hdma_rx_xfer_mode = hspi->hdma_rx->xfer_mode;
#endif /* USE_HAL_DMA_LINKEDLIST */

  if (hspi->global_state != HAL_SPI_STATE_ABORT)
  {
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    if ((hdma_tx_xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
        && (hdma_rx_xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR))
    {
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
      hspi->p_tx_rx_cplt_cb(hspi);
#else
      HAL_SPI_TxRxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
    else
    {
      LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
    }
#else
    LL_SPI_EnableIT_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_DMA_LINKEDLIST */
  }
}

/**
  * @brief DMA SPI half transmit process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAHalfTransmitCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_tx_half_cplt_cb(hspi);
#else
  HAL_SPI_TxHalfCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI half receive process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAHalfReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_rx_half_cplt_cb(hspi);
#else
  HAL_SPI_RxHalfCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI half transmit receive process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAHalfTransmitReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_tx_rx_half_cplt_cb(hspi);
#else
  HAL_SPI_TxRxHalfCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI communication error callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAError(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* if DMA error is FIFO error ignore it */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  hspi->last_error_codes  = HAL_SPI_ERROR_DMA;
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

  (void)SPI_CloseTransfer(hspi);

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_error_cb(hspi);
#else
  HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI communication abort callback, when initiated by HAL services on Error.
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param hdma DMA handle.
  */
static void SPI_DMAAbortOnError(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  (void)SPI_CloseTransfer(hspi);

  /* Reset p_xfer_abort_cb */
  hdma->p_xfer_abort_cb = NULL;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_error_cb(hspi);
#else
  HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI Tx communication abort callback, when initiated by user.
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @param hdma DMA handle.
  * @warning When this callback is executed, User Abort complete callback is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  */
static void SPI_DMATxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  hspi->hdma_tx->p_xfer_abort_cb = NULL;

  /* Check if an Abort process is still ongoing */
  if (hspi->hdma_rx != NULL)
  {
    if (hspi->hdma_rx->p_xfer_abort_cb != NULL)
    {
      return;
    }
  }

  /* Call the Abort procedure */
  SPI_AbortTransfer(hspi);

  hspi->global_state = HAL_SPI_STATE_IDLE;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_abort_cplt_cb(hspi);
#else
  HAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI Rx communication abort callback, when initiated by user.
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @param hdma DMA handle.
  * @warning When this callback is executed, User Abort complete callback is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  */
static void SPI_DMARxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_spi_handle_t *hspi = (hal_spi_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  hspi->hdma_rx->p_xfer_abort_cb = NULL;

  /* Check if an Abort process is still ongoing */
  if (hspi->hdma_tx != NULL)
  {
    if (hspi->hdma_tx->p_xfer_abort_cb != NULL)
    {
      return;
    }
  }

  /* Call the Abort procedure */
  SPI_AbortTransfer(hspi);

  hspi->global_state = HAL_SPI_STATE_IDLE;

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  hspi->p_abort_cplt_cb(hspi);
#else
  HAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief DMA SPI empty callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *               the configuration information for the specified DMA module.
  */
static void SPI_DMAEmptyCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /* NOTE : As TC callback is mandatory inside DMA handle, the SPI_DMAEmptyCallback
   *        is used to be called on TC DMA event.
   *        Use when SPI is in Full-Duplex as only one DMA channel is used global TC.
   */
}
#endif /* USE_HAL_SPI_DMA */

/**
  * @brief Manage the receive 8-bit in Interrupt context.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_RxISR_8BIT(hal_spi_handle_t *hspi)
{
  /* Receive data in 8 Bit mode */
  *((uint8_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData8((SPI_TypeDef *)((uint32_t)hspi->instance));
  hspi->p_rx_buff += sizeof(uint8_t);
  hspi->rx_xfer_count--;

  /* Disable IT if no more data expected */
  if (hspi->rx_xfer_count == 0UL)
  {
    LL_SPI_DisableIT_RXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}


/**
  * @brief Manage the 16-bit receive in Interrupt context.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_RxISR_16BIT(hal_spi_handle_t *hspi)
{
  /* Receive data in 16 Bit mode */
  *((uint16_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData16((SPI_TypeDef *)((uint32_t)hspi->instance));
  hspi->p_rx_buff += sizeof(uint16_t);
  hspi->rx_xfer_count--;

  /* Disable IT if no more data expected */
  if (hspi->rx_xfer_count == 0UL)
  {
    LL_SPI_DisableIT_RXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}


/**
  * @brief Manage the 32-bit receive in Interrupt context.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_RxISR_32BIT(hal_spi_handle_t *hspi)
{
  /* Receive data in 32 Bit mode */
  *((uint32_t *)hspi->p_rx_buff) = LL_SPI_ReceiveData32((SPI_TypeDef *)((uint32_t)hspi->instance));
  hspi->p_rx_buff += sizeof(uint32_t);
  hspi->rx_xfer_count--;

  /* Disable IT if no more data expected */
  if (hspi->rx_xfer_count == 0UL)
  {
    LL_SPI_DisableIT_RXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}


/**
  * @brief Handle the data 8-bit transmit in Interrupt mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_TxISR_8BIT(hal_spi_handle_t *hspi)
{
  if (hspi->tx_xfer_count != 0UL)
  {
    LL_SPI_TransmitData8(((SPI_TypeDef *)((uint32_t)hspi->instance)), *hspi->p_tx_buff);
    hspi->p_tx_buff += sizeof(uint8_t);
    hspi->tx_xfer_count--;
  }
  else
  {
    /* Disable IT if no more data expected */
    LL_SPI_DisableIT_TXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}

/**
  * @brief Handle the data 16-bit transmit in Interrupt mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_TxISR_16BIT(hal_spi_handle_t *hspi)
{
  if (hspi->tx_xfer_count != 0UL)
  {
    LL_SPI_TransmitData16(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint16_t *)hspi->p_tx_buff));
    hspi->p_tx_buff += sizeof(uint16_t);
    hspi->tx_xfer_count--;
  }
  else
  {
    /* Disable IT if no more data expected */
    LL_SPI_DisableIT_TXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}

/**
  * @brief Handle the data 32-bit transmit in Interrupt mode.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_TxISR_32BIT(hal_spi_handle_t *hspi)
{
  if (hspi->tx_xfer_count != 0UL)
  {
    LL_SPI_TransmitData32(((SPI_TypeDef *)((uint32_t)hspi->instance)), *((const uint32_t *)hspi->p_tx_buff));
    hspi->p_tx_buff += sizeof(uint32_t);
    hspi->tx_xfer_count--;
  }
  else
  {
    /* Disable IT if no more data expected */
    LL_SPI_DisableIT_TXP((SPI_TypeDef *)((uint32_t)hspi->instance));
  }
}

/**
  * @brief Abort Transfer and clear flags.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  */
static void SPI_AbortTransfer(hal_spi_handle_t *hspi)
{
  LL_SPI_Disable((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Disable ITs */
  LL_SPI_DisableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                   LL_SPI_IT_EOT | LL_SPI_IT_DXP | LL_SPI_IT_TXP | LL_SPI_IT_RXP |
                   LL_SPI_IT_OVR | LL_SPI_IT_UDR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

  /* Clear the Status flags in the SR register */
  LL_SPI_ClearFlag_EOT((SPI_TypeDef *)((uint32_t)hspi->instance));
  LL_SPI_ClearFlag_TXTF((SPI_TypeDef *)((uint32_t)hspi->instance));

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  /* Disable Tx and Rx DMA Request */
  LL_SPI_DisableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));
  LL_SPI_DisableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_SPI_DMA */

  /* Clear the Error flags in the SR register */
  LL_SPI_ClearFlag((SPI_TypeDef *)((uint32_t)hspi->instance),
                   LL_SPI_FLAG_OVR | LL_SPI_FLAG_UDR | LL_SPI_FLAG_TIFRE | LL_SPI_FLAG_MODF | LL_SPI_FLAG_SUSP);

#if (USE_HAL_SPI_CRC != 0U)
  LL_SPI_ClearFlag_CRCERR((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_SPI_CRC */

  hspi->tx_xfer_count = (uint16_t)0UL;
  hspi->rx_xfer_count = (uint16_t)0UL;
}

/**
  * @brief Close Transfer and clear flags.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
static hal_status_t SPI_CloseTransfer(hal_spi_handle_t *hspi)
{
  hal_status_t status = HAL_OK;

  /* Clear the Status flags in the SR register */
  LL_SPI_ClearFlag((SPI_TypeDef *)((uint32_t)hspi->instance), LL_SPI_FLAG_EOT | LL_SPI_FLAG_TXTF);

#if (USE_HAL_SPI_CRC != 0UL)
  /* Check if CRC error occurred */
  if (LL_SPI_IsEnabledCRC((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
    if (LL_SPI_IsActiveFlag_CRCERR((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
    {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_CRC);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_CRCERR((SPI_TypeDef *)((uint32_t)hspi->instance));

      status = HAL_ERROR;
    }
  }
#endif /* USE_HAL_SPI_CRC */

  /* Report Underrun error for non RX Only communication */
  if (hspi->global_state != HAL_SPI_STATE_RX_ACTIVE)
  {
    if (LL_SPI_IsActiveFlag_UDR((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
    {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_UDR);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_UDR((SPI_TypeDef *)((uint32_t)hspi->instance));

      status = HAL_ERROR;
    }
  }

  /* Report Overrun error for non TX Only communication */
  if (hspi->global_state != HAL_SPI_STATE_TX_ACTIVE)
  {
    if (LL_SPI_IsActiveFlag_OVR((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
    {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_OVR);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
      LL_SPI_ClearFlag_OVR((SPI_TypeDef *)((uint32_t)hspi->instance));

      status = HAL_ERROR;
    }
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if (LL_SPI_IsActiveFlag_MODF((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_MODF);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    LL_SPI_ClearFlag_MODF((SPI_TypeDef *)((uint32_t)hspi->instance));

    /* Enter in mode fault state */
    hspi->global_state = HAL_SPI_STATE_FAULT;

    status = HAL_ERROR;
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if (LL_SPI_IsActiveFlag_FRE((SPI_TypeDef *)((uint32_t)hspi->instance)) != 0UL)
  {
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hspi->last_error_codes, HAL_SPI_ERROR_FRE);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
    LL_SPI_ClearFlag_FRE((SPI_TypeDef *)((uint32_t)hspi->instance));

    status = HAL_ERROR;
  }

  LL_SPI_Disable((SPI_TypeDef *)((uint32_t)hspi->instance));

  /* Disable ITs */
  LL_SPI_DisableIT((SPI_TypeDef *)((uint32_t)hspi->instance),
                   LL_SPI_IT_EOT | LL_SPI_IT_TXP | LL_SPI_IT_RXP | LL_SPI_IT_DXP |
                   LL_SPI_IT_UDR | LL_SPI_IT_OVR | LL_SPI_IT_TIFRE | LL_SPI_IT_MODF);

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  LL_SPI_DisableDMAReq_TX((SPI_TypeDef *)((uint32_t)hspi->instance));
  LL_SPI_DisableDMAReq_RX((SPI_TypeDef *)((uint32_t)hspi->instance));
#endif /* USE_HAL_SPI_DMA */

  hspi->tx_xfer_count = (uint16_t)0UL;
  hspi->rx_xfer_count = (uint16_t)0UL;

  if (hspi->global_state != HAL_SPI_STATE_FAULT)
  {
    hspi->global_state = HAL_SPI_STATE_IDLE;
  }
  return status;
}

/**
  * @brief Handle SPI Communication Timeout.
  * @param hspi Pointer to a \ref hal_spi_handle_t structure which contains
  *             the SPI instance.
  * @param timeout_ms Timeout duration.
  * @param tick_start Tick start value.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_TIMEOUT Operation exceeds user timeout.
  */
static hal_status_t SPI_WaitEndOfTransfer(hal_spi_handle_t *hspi,
                                          uint32_t timeout_ms, uint32_t tick_start)
{
  while ((LL_SPI_IsActiveFlag((SPI_TypeDef *)((uint32_t)hspi->instance), SPI_SR_EOT) == 0UL))
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        if ((LL_SPI_IsActiveFlag((SPI_TypeDef *)((uint32_t)hspi->instance), SPI_SR_EOT) == 0UL))
        {
          hspi->global_state = HAL_SPI_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @}
  */

#endif /* USE_HAL_SPI_MODULE */
#endif /* SPI1 || SPI2 || SPI3 */

/**
  * @}
  */

/**
  * @}
  */
