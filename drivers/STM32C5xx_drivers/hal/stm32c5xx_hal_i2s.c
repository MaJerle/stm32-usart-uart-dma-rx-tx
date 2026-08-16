/**
  ******************************************************************************
  * @file    stm32c5xx_hal_i2s.c
  * @brief   I2S HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Integrated Interchip Sound (I2S) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral control functions
  *           + Peripheral state functions.
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

/** @addtogroup I2S I2S
  * @{
  */
/** @defgroup I2S_Introduction I2S Introduction
  * @{

  - The I2S hardware abstraction layer provides a set of APIs to interface with the STM32 I2S (Inter-integrated circuit
    sound) peripheral.

  - It simplifies the configuration, initialization, and management of I2S communication by supporting various modes
    such as polling, interrupt, and DMA for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series.
  * @}
  */

/** @defgroup I2S_How_To_Use I2S How To Use
  * @{
# How to use the I2S HAL module driver

1. Declare a hal_i2s_handle_t handle structure and initialize the I2Sx driver with a SPI HW instance
   by calling HAL_I2S_Init().

2. Enable the I2Sx clock by calling HAL_I2S_Init() when USE_HAL_I2S_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.

3. Configure the low-level hardware (GPIO, CLOCK, NVIC, etc.):
      - Enable the I2Sx interface clock.
      - I2Sx pins configuration:
        - Enable the clock for the I2Sx GPIOs.
        - Configure these I2S pins as alternate function pull-up.
      - NVIC configuration for interrupt mode:
        - Configure the I2Sx interrupt priority.
        - Enable the I2Sx IRQ in NVIC.
      - DMA configuration for DMA mode:
        - Declare a hal_dma_handle_t handle structure for the transmit or receive Stream/Channel.
        - Enable the DMAx clock.
        - Configure the DMA handle parameters.
        - Configure the DMA Tx or Rx Stream/Channel.
        - Associate the initialized hi2s handle with the DMA Tx or Rx handle.
        - For each DMA channel (Tx and Rx), configure the corresponding NVIC line priority and enable the
          Tx or Rx Stream/Channel.

4. Set the generic configuration of the I2S:
In Master mode with HAL_I2S_MASTER_SetConfig():
      - The mode
      - The standard
      - The data format
      - The audio frequency
      - The clock polarity
      - The bit order
In Slave mode with HAL_I2S_SLAVE_SetConfig():
      - The mode
      - The standard
      - The data format
      - The clock polarity
      - The bit order

5. For advanced configuration, use the following functions:
      - HAL_I2S_MASTER_EnableClockOutput() to enable the Master Clock Output feature. When master clock output is
        enabled, a master clock is generated on the MCK pin at a frequency 256 times higher than the frame
        synchronization. This master clock often provides a reference clock to an external audio codec.
      - HAL_I2S_EnableWSInversion() to enable the Word Select Inversion feature. When word select inversion is
        enabled, the default polarity of the WS signal is inverted. In I2S PHILIPS standard, the left channel is
        transferred when WS is high and the right channel when WS is low. In MSB or LSB justified mode, the left
        channel is transferred when WS is low and the right channel when WS is high. In PCM mode, the start of frame
        is indicated by a falling edge.
      - HAL_I2S_MASTER_EnableKeepIOState() to enable the Master Keep IO State feature. When the Master Keep IO State
        feature is enabled, the peripheral always keeps control of all associated GPIOs to prevent any glitches on
        the line.
      - HAL_I2S_EnableIOSwap() to enable the IO Swap feature. When the IO Swap is enabled, SDI and SDO lines are
        swapped. This functionality eases PCB routing or errors.
      - HAL_I2S_SLAVE_EnableLengthDetectionError() to enable the Channel Length Detection error feature. When Channel
        Length Detection Error is enabled, the frame error coverage is extended, then the I2S expects fixed channel
        lengths in slave mode.
      - HAL_I2S_LockIOConfig() to enable the Lock of IO configuration feature. When IO lock is enabled, the SPI_CFG2
        register content cannot be modified. Configuration functions HAL_I2S_MASTER_SetConfig,
        HAL_I2S_SLAVE_SetConfig, HAL_I2S_Enable/DisableKeepIOState, HAL_I2S_Enable/DisableIOSwap, and
        HAL_I2S_SetBitOrder are no longer usable and return HAL_I2S_ERROR_IO_LOCKED.
      - HAL_I2S_SetFifoThreshold() to set the FIFO threshold. FIFO threshold configuration allows an optimization of
        the FIFO locations. The basic element of the FIFO is the byte. When the data size is fixed to 24 bits, each
        audio sample takes 3 basic FIFO elements. To reach the FIFO threshold, call the transfer API with a
        size_sample modulo the FIFO threshold.
      - HAL_I2S_SetData24BitsAlignedRight() to set the data alignment to the right for 24-bit data format.
      - HAL_I2S_SetData24BitsAlignedLeft() to set the data alignment to the left for 24-bit data format.

6. Callback registration:
Use the compilation flag USE_HAL_I2S_REGISTER_CALLBACKS to configure driver callbacks dynamically.

  Callback name               | Default value                       | Callback registration function
  ----------------------------| ----------------------------------- | ---------------------------
  ErrorCallback               | HAL_I2S_ErrorCallback()             | HAL_I2S_RegisterErrorCallback()
  TxCpltCallback              | HAL_I2S_TxCpltCallback()            | HAL_I2S_RegisterTxCpltCallback()
  TxHalfCpltCallback          | HAL_I2S_TxHalfCpltCallback()        | HAL_I2S_RegisterTxHalfCpltCallback()
  RxCpltCallback              | HAL_I2S_RxCpltCallback()            | HAL_I2S_RegisterRxCpltCallback()
  RxHalfCpltCallback          | HAL_I2S_RxHalfCpltCallback()        | HAL_I2S_RegisterRxHalfCpltCallback()
  TxRxCpltCallback            | HAL_I2S_TxRxCpltCallback()          | HAL_I2S_RegisterTxRxCpltCallback()
  TxRxHalfCpltCallback        | HAL_I2S_TxRxHalfCpltCallback()      | HAL_I2S_RegisterTxRxHalfCpltCallback()
  AbortCpltCallback           | HAL_I2S_AbortCpltCallback()         | HAL_I2S_RegisterAbortCpltCallback()

  To unregister a callback, register the default callback via the registration function.

  By default, after HAL_I2S_Init() and when the state is HAL_I2S_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Register callbacks when handle global_state is HAL_I2S_STATE_INIT or HAL_I2S_STATE_IDLE.

  When the compilation definition USE_HAL_I2S_REGISTER_CALLBACKS is set to 0 or is not defined, the callback
  registration feature is not available, and weak callbacks are used as shown in the table above.

    Note: HAL_I2S_RegisterTxHalfCpltCallback(), HAL_I2S_RegisterRxHalfCpltCallback(), and
      HAL_I2S_RegisterTxRxHalfCpltCallback() apply only in DMA mode.

7. Acquire/Release the HAL I2S handle:
      - When the compilation flag USE_HAL_MUTEX is set to 1, a multi-thread application can acquire the I2S HAL handle
        to execute a transmit or receive process or a transmit/receive sequence. Release the I2S HAL handle when the
        process or sequence ends.
      - HAL Acquire/Release operations are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
        - HAL_I2S_AcquireBus() Acquire the HAL I2S handle.
        - HAL_I2S_ReleaseBus() Release the HAL I2S handle.
  */

/**
  * @}
  */

/** @defgroup I2S_Configuration_Table I2S Configuration Table
  * @{
## Configuration inside the I2S driver

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_HAL_I2S_MODULE             |         1         | Enable the HAL I2S driver module
USE_HAL_I2S_REGISTER_CALLBACKS |         0         | Allow an application to define a callback
USE_HAL_I2S_DMA                |         1         | Enable the DMA code inside I2S
USE_HAL_CHECK_PARAM            |         0         | Enable runtime parameter check
USE_HAL_I2S_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_MUTEX                  |         0         | Enable semaphore creation for OS
USE_HAL_I2S_USER_DATA          |         0         | Add user data inside the HAL I2S handle
USE_HAL_I2S_GET_LAST_ERRORS    |         0         | Enable retrieving last process error codes
USE_HAL_I2S_OVR_UDR_ERRORS     |         0         | Enable retrieving overrun and underrun errors

Software configuration defined in preprocessor environment:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM           |    Not defined    | Enable parameter check for HAL and LL
USE_ASSERT_DBG_STATE           |    Not defined    | Enable state check for HAL
 */

/**
  * @}
  */

#if defined(USE_HAL_I2S_MODULE) && (USE_HAL_I2S_MODULE == 1)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup I2S_Private_Constants I2S Private Constants
  * @{
  */

#define I2S_DEFAULT_TIMEOUT_MS  100U  /*!< Timeout default value */
#define I2S_FIFO_SIZE_BYTE      16U   /*!< Standard size 16-Bytes */
#define I2S_NUMBER_FREQUENCY    9U    /*!< Number of audio frequencies */

/* Private macro -------------------------------------------------------------*/
/** @defgroup I2S_Private_Macros I2S Private Macros
  * @{
  */

/** @brief  Ensure I2S Mode is valid.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE(mode) (((mode) == HAL_I2S_MODE_SLAVE_TX)             \
                           || ((mode) == HAL_I2S_MODE_SLAVE_RX)          \
                           || ((mode) == HAL_I2S_MODE_MASTER_TX)         \
                           || ((mode) == HAL_I2S_MODE_MASTER_RX)         \
                           || ((mode) == HAL_I2S_MODE_SLAVE_FULL_DUPLEX) \
                           || ((mode) == HAL_I2S_MODE_MASTER_FULL_DUPLEX))

/** @brief  Ensure I2S Mode is Full Duplex.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE_FULL_DUPLEX(mode) (((mode) == HAL_I2S_MODE_SLAVE_FULL_DUPLEX) \
                                       || ((mode) == HAL_I2S_MODE_MASTER_FULL_DUPLEX))

/** @brief  Ensure I2S Mode is Master.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE_MASTER(mode) (((mode) == HAL_I2S_MODE_MASTER_TX)    \
                                  || ((mode) == HAL_I2S_MODE_MASTER_RX) \
                                  || ((mode) == HAL_I2S_MODE_MASTER_FULL_DUPLEX))

/** @brief  Ensure I2S Mode is Slave.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE_SLAVE(mode) (((mode) == HAL_I2S_MODE_SLAVE_TX)    \
                                 || ((mode) == HAL_I2S_MODE_SLAVE_RX) \
                                 || ((mode) == HAL_I2S_MODE_SLAVE_FULL_DUPLEX))

/** @brief  Ensure I2S Mode is Transmit mode.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE_TX(mode) (((mode) == HAL_I2S_MODE_SLAVE_TX) \
                              || ((mode) == HAL_I2S_MODE_MASTER_TX))

/** @brief  Ensure I2S Mode is Receive mode.
  * @param  mode I2S communication mode (@ref hal_i2s_mode_t).
  * @retval 1 mode is a valid value.
  * @retval 0 mode is an invalid value.
  */
#define IS_I2S_MODE_RX(mode) (((mode) == HAL_I2S_MODE_SLAVE_RX) \
                              || ((mode) == HAL_I2S_MODE_MASTER_RX))

/** @brief  Ensure I2S Standard is valid.
  * @param  standard I2S audio standard (@ref hal_i2s_standard_t).
  * @retval 1 standard is a valid value.
  * @retval 0 standard is an invalid value.
  */
#define IS_I2S_STANDARD(standard) (((standard) == HAL_I2S_STANDARD_PHILIPS)      \
                                   || ((standard) == HAL_I2S_STANDARD_MSB)       \
                                   || ((standard) == HAL_I2S_STANDARD_LSB)       \
                                   || ((standard) == HAL_I2S_STANDARD_PCM_SHORT) \
                                   || ((standard) == HAL_I2S_STANDARD_PCM_LONG))

/** @brief  Ensure I2S Standard is PCM standard.
  * @param  standard I2S audio standard (@ref hal_i2s_standard_t).
  * @retval 1 standard is a valid value.
  * @retval 0 standard is an invalid value.
  */
#define IS_I2S_STANDARD_PCM(standard) (((standard) == HAL_I2S_STANDARD_PCM_SHORT) \
                                       || ((standard) == HAL_I2S_STANDARD_PCM_LONG))

/** @brief  Ensure I2S Data Format is valid.
  * @param  data_format I2S data format (@ref hal_i2s_data_format_t).
  * @retval 1 data_format is a valid value.
  * @retval 0 data_format is an invalid value.
  */
#define IS_I2S_DATA_FORMAT(data_format) (((data_format) == HAL_I2S_DATA_FORMAT_16_BIT)             \
                                         || ((data_format) == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED) \
                                         || ((data_format) == HAL_I2S_DATA_FORMAT_24_BIT)          \
                                         || ((data_format) == HAL_I2S_DATA_FORMAT_32_BIT))

/** @brief  Ensure I2S 32-Bits Data Format is valid.
  * @param  data_format I2S data format (@ref hal_i2s_data_format_t).
  * @retval 1 data_format is a valid value.
  * @retval 0 data_format is an invalid value.
  */
#define IS_I2S_CHANNEL_LENGTH_32_BIT(data_format) (((data_format) == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED) \
                                                   || ((data_format) == HAL_I2S_DATA_FORMAT_24_BIT)       \
                                                   || ((data_format) == HAL_I2S_DATA_FORMAT_32_BIT))

/** @brief  Ensure I2S Audio Frequency is valid.
  * @param  audio_frequency I2S audio frequency (@ref hal_i2s_master_audio_frequency_t).
  * @retval 1 audio_frequency is a valid value.
  * @retval 0 audio_frequency is an invalid value.
  */
#define IS_I2S_MASTER_AUDIO_FREQUENCY(audio_frequency) (((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_192_KHZ)   \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_96_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_48_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_44_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_32_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_22_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_16_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_11_KHZ) \
                                                        || ((audio_frequency) == HAL_I2S_MASTER_AUDIO_FREQ_8_KHZ))

/** @brief  Ensure I2S Clock Polarity is valid.
  * @param  clock_polarity I2S clock polarity (@ref hal_i2s_clock_polarity_t).
  * @retval 1 clock_polarity is a valid value.
  * @retval 0 clock_polarity is an invalid value.
  */
#define IS_I2S_CLOCK_POLARITY(clock_polarity) (((clock_polarity) == HAL_I2S_CLOCK_POLARITY_LOW) \
                                               || ((clock_polarity) == HAL_I2S_CLOCK_POLARITY_HIGH))

/** @brief  Ensure I2S Bit Order is valid.
  * @param  bit_order I2S bit order (@ref hal_i2s_bit_order_t).
  * @retval 1 bit_order is a valid value.
  * @retval 0 bit_order is an invalid value.
  */
#define IS_I2S_BIT_ORDER(bit_order) (((bit_order) == HAL_I2S_MSB_FIRST) \
                                     || ((bit_order) == HAL_I2S_LSB_FIRST))

/** @brief  Ensure I2S FIFO Threshold is valid.
  * @param  fifo_threshold I2S FIFO threshold (@ref hal_i2s_fifo_threshold_t).
  * @retval 1 fifo_threshold is a valid value.
  * @retval 0 fifo_threshold is an invalid value.
  */
#define IS_I2S_FIFO_THRESHOLD(fifo_threshold) (((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_1_DATA)    \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_2_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_3_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_4_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_5_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_6_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_7_DATA) \
                                               || ((fifo_threshold) == HAL_I2S_FIFO_THRESHOLD_8_DATA))

/** @brief  Ensure I2S transfer size is a multiplier of the FIFO Threshold.
  * @param  size_sample I2S transfer size.
  * @param  fifo_threshold I2S FIFO threshold (@ref hal_i2s_fifo_threshold_t),
  *                        must be different from HAL_I2S_FIFO_THRESHOLD_1_DATA.
  * @retval 1 size_sample is a valid value for fifo_threshold.
  * @retval 0 size_sample is an invalid value for fifo_threshold.
  */
#define IS_I2S_TRANSFER_SIZE(size_sample, fifo_threshold) (((size_sample) % (fifo_threshold)) == 0U)

/**
  * @brief Retrieve I2S instance from handle.
  * @param handle specifies the I2S handle.
  */
#define I2S_GET_INSTANCE(handle) ((SPI_TypeDef*)((uint32_t)(handle)->instance))

/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup I2S_Private_Functions I2S Private Functions
  * @{
  */
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
static void I2S_DMATxCplt(hal_dma_handle_t *hdma);
static void I2S_DMATxHalfCplt(hal_dma_handle_t *hdma);
static void I2S_DMARxCplt(hal_dma_handle_t *hdma);
static void I2S_DMARxHalfCplt(hal_dma_handle_t *hdma);
static void I2S_DMATxRxCplt(hal_dma_handle_t *hdma);
static void I2S_DMATxRxHalfCplt(hal_dma_handle_t *hdma);
static void I2S_DMAError(hal_dma_handle_t *hdma);
static void I2S_DMATxAbortCallback(hal_dma_handle_t *hdma);
static void I2S_DMARxAbortCallback(hal_dma_handle_t *hdma);
static void I2S_DMATxRxAbortCallback(hal_dma_handle_t *hdma);
#endif /* USE_HAL_I2S_DMA */
static void I2S_Transmit_16Bit_IT(hal_i2s_handle_t *hi2s);
static void I2S_Transmit_32Bit_IT(hal_i2s_handle_t *hi2s);
static void I2S_Receive_16Bit_IT(hal_i2s_handle_t *hi2s);
static void I2S_Receive_32Bit_IT(hal_i2s_handle_t *hi2s);
static void I2S_AbortTransfer(const hal_i2s_handle_t *hi2s);
static hal_status_t  I2S_CloseTransfer(hal_i2s_handle_t *hi2s);
static hal_status_t  I2S_SetAudioFrequency(const hal_i2s_handle_t *hi2s, uint32_t i2s_clk,
                                           hal_i2s_master_audio_frequency_t audio_frequency_ws_hz);
static hal_i2s_master_audio_frequency_t I2S_GetAudioFrequency(const hal_i2s_handle_t *hi2s, uint32_t i2s_clk);
static void I2S_WaitTxFifoEmpty(hal_i2s_handle_t *hi2s);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup I2S_Exported_Functions HAL I2S Functions
  * @{
  */

/** @addtogroup I2S_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{

  This subsection provides functions to initialize and deinitialize the I2Sx peripheral:
    - Call the function HAL_I2S_Init() to initialize the selected I2S handle and associate an instance.
    - Call the function HAL_I2S_DeInit() to restore the default initialization of the selected I2Sx peripheral.
  */

/**
  * @brief  Initialize the I2S according to the associated handle.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  instance I2S instance.
  * @retval HAL_INVALID_PARAM When the handle is NULL.
  * @retval HAL_ERROR When the MUTEX cannot be created.
  * @retval HAL_OK HAL I2S driver correctly initialized for the given I2S instance.
  */
hal_status_t HAL_I2S_Init(hal_i2s_handle_t *hi2s, hal_i2s_t instance)
{
  /* Check parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_ALL_INSTANCE((SPI_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check the handle struct pointer */
  if (hi2s == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->instance = instance;

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  /* Init the I2S callback settings */
  hi2s->p_error_cb           = HAL_I2S_ErrorCallback;
  hi2s->p_tx_cplt_cb         = HAL_I2S_TxCpltCallback;
  hi2s->p_tx_half_cplt_cb    = HAL_I2S_TxHalfCpltCallback;
  hi2s->p_rx_cplt_cb         = HAL_I2S_RxCpltCallback;
  hi2s->p_rx_half_cplt_cb    = HAL_I2S_RxHalfCpltCallback;
  hi2s->p_tx_rx_cplt_cb      = HAL_I2S_TxRxCpltCallback;
  hi2s->p_tx_rx_half_cplt_cb = HAL_I2S_TxRxHalfCpltCallback;
  hi2s->p_abort_cplt_cb      = HAL_I2S_AbortCpltCallback;
#endif  /* USE_HAL_I2S_REGISTER_CALLBACKS */

  /* Other internal fields */
  hi2s->pause_state    = HAL_I2S_STATE_IDLE;
  hi2s->p_tx_buff      = (uint16_t *) NULL;
  hi2s->tx_xfer_size   = (uint16_t) 0U;
  hi2s->tx_xfer_count  = (uint16_t) 0U;
  hi2s->p_rx_buff      = (uint16_t *) NULL;
  hi2s->rx_xfer_size   = (uint16_t) 0U;
  hi2s->rx_xfer_count  = (uint16_t) 0U;

#if defined (USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  hi2s->hdma_tx        = (hal_dma_handle_t *) NULL;
  hi2s->hdma_rx        = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_I2S_DMA  */

#if defined(USE_HAL_I2S_USER_DATA) && (USE_HAL_I2S_USER_DATA == 1)
  /* Initialize the user data pointer to NULL */
  hi2s->p_user_data    = NULL;
#endif /* USE_HAL_I2S_USER_DATA */

#if defined (USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

#if defined(USE_HAL_I2S_CLK_ENABLE_MODEL) && (USE_HAL_I2S_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  switch (hi2s->instance)
  {
    case HAL_I2S1:
      HAL_RCC_SPI1_EnableClock();
      break;
    case HAL_I2S2:
      HAL_RCC_SPI2_EnableClock();
      break;
#if defined(SPI3)
    case HAL_I2S3:
      HAL_RCC_SPI3_EnableClock();
      break;
#endif /* SPI3 */
    default:
      break;
  }
#endif /* USE_HAL_I2S_CLK_ENABLE_MODEL */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  if (HAL_OS_SemaphoreCreate(&hi2s->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  hi2s->global_state = HAL_I2S_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the I2S driver for the given handle.
  * @param  hi2s pointer to a @ref hal_i2s_handle_t.
  */
void HAL_I2S_DeInit(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;
  hal_i2s_state_t temp_state;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_ALL_INSTANCE(p_i2sx));

  temp_state = hi2s->global_state;

  /* Check if any transfer is ongoing */
  if ((temp_state == HAL_I2S_STATE_TX_ACTIVE) || (temp_state == HAL_I2S_STATE_RX_ACTIVE)
      || (temp_state == HAL_I2S_STATE_TX_RX_ACTIVE))
  {
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
    if (LL_I2S_IsEnabledDMAReq_TX(p_i2sx) != 0U)
    {
      if (hi2s->hdma_tx != NULL)
      {
        (void)HAL_DMA_Abort(hi2s->hdma_tx);
      }
    }

    if (LL_I2S_IsEnabledDMAReq_RX(p_i2sx) != 0U)
    {
      if (hi2s->hdma_rx != NULL)
      {
        (void)HAL_DMA_Abort(hi2s->hdma_rx);
      }
    }
#endif /* USE_HAL_I2S_DMA */

    (void)I2S_AbortTransfer(hi2s);
  }

  /* Disable I2S peripheral */
  LL_I2S_DisableI2SMode(p_i2sx);
  LL_I2S_Disable(p_i2sx);

#if defined (USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  /* Reset the last_error_codes variable storing the last errors */
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&hi2s->semaphore);
#endif /* USE_HAL_MUTEX */

  hi2s->global_state = HAL_I2S_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group2 Configuration functions
  * @{
  This subsection provides functions to configure the I2Sx peripheral:
    - Call the function HAL_I2S_MASTER_SetConfig() to configure the selected device in master mode with the selected
      configuration:
      - Mode
      - Standard
      - Data format
      - Audio frequency selected
      - Clock polarity
      - Bit order
    - Call the function HAL_I2S_MASTER_GetConfig() to retrieve the current global configuration set by the application
      in master mode.
    - Call the function HAL_I2S_SLAVE_SetConfig() to configure the selected device in slave mode with the selected
      configuration:
      - Mode
      - Standard
      - Data format
      - Clock polarity
      - Bit order
    - Call the function HAL_I2S_SLAVE_GetConfig() to retrieve the current global configuration set by the application
      in slave mode.
  */

/**
  * @brief  Set the configuration to the I2S peripheral in Master mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_config Pointer to @ref hal_i2s_master_config_t configuration structure.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  * @retval HAL_ERROR When IO locked.
  * @retval HAL_OK I2S instance has been correctly configured.
  */
hal_status_t HAL_I2S_MASTER_SetConfig(hal_i2s_handle_t *hi2s, const hal_i2s_master_config_t *p_config)
{
  SPI_TypeDef *p_i2sx;

  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER(p_config->mode));
  ASSERT_DBG_PARAM(IS_I2S_STANDARD(p_config->standard));
  ASSERT_DBG_PARAM(IS_I2S_DATA_FORMAT(p_config->data_format));
  ASSERT_DBG_PARAM(IS_I2S_MASTER_AUDIO_FREQUENCY(p_config->audio_frequency));
  ASSERT_DBG_PARAM(IS_I2S_CLOCK_POLARITY(p_config->clock_polarity));
  ASSERT_DBG_PARAM(IS_I2S_BIT_ORDER(p_config->bit_order));

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE | HAL_I2S_STATE_INIT);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check the config struct pointer */
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  if (LL_I2S_IsEnabled(p_i2sx) != 0U)
  {
    LL_I2S_ClearFlag(p_i2sx, LL_I2S_FLAG_SUSP);
    LL_I2S_Disable(p_i2sx);
  }

  /* Set I2S generic configuration */
  LL_I2S_ConfigBus(p_i2sx, ((uint32_t)(p_config->mode) | (uint32_t)(p_config->standard)
                            | (uint32_t)(p_config->data_format) | (uint32_t)(p_config->clock_polarity)),
                   (uint32_t)(p_config->bit_order));

  if (I2S_SetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx), p_config->audio_frequency) != HAL_OK)
  {
    return HAL_ERROR;
  }

  LL_I2S_EnableI2SMode(p_i2sx);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Set the configuration to the I2S peripheral in Slave mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_config Pointer to @ref hal_i2s_slave_config_t configuration structure.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  * @retval HAL_ERROR When IO locked.
  * @retval HAL_OK I2S instance has been correctly configured.
  */
hal_status_t HAL_I2S_SLAVE_SetConfig(hal_i2s_handle_t *hi2s, const hal_i2s_slave_config_t *p_config)
{
  SPI_TypeDef *p_i2sx;

  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I2S_MODE_SLAVE(p_config->mode));
  ASSERT_DBG_PARAM(IS_I2S_STANDARD(p_config->standard));
  ASSERT_DBG_PARAM(IS_I2S_DATA_FORMAT(p_config->data_format));
  ASSERT_DBG_PARAM(IS_I2S_CLOCK_POLARITY(p_config->clock_polarity));
  ASSERT_DBG_PARAM(IS_I2S_BIT_ORDER(p_config->bit_order));

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE | HAL_I2S_STATE_INIT);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check the config struct pointer */
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  if (LL_I2S_IsEnabled(p_i2sx) != 0U)
  {
    LL_I2S_ClearFlag(p_i2sx, LL_I2S_FLAG_SUSP);
    LL_I2S_Disable(p_i2sx);
  }

  /* Set I2S generic configuration */
  LL_I2S_ConfigBus(p_i2sx, ((uint32_t)(p_config->mode) | (uint32_t)(p_config->standard)
                            | (uint32_t)(p_config->data_format) | (uint32_t)(p_config->clock_polarity)),
                   (uint32_t)(p_config->bit_order));

  LL_I2S_EnableI2SMode(p_i2sx);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Retrieve the configuration of the I2S peripheral in Master mode.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param p_config Pointer to @ref hal_i2s_master_config_t configuration structure.
  */
void HAL_I2S_MASTER_GetConfig(const hal_i2s_handle_t *hi2s, hal_i2s_master_config_t *p_config)
{
  SPI_TypeDef *p_i2sx;
  uint32_t i2scfgr_reg_value;

  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  i2scfgr_reg_value = (uint32_t)(LL_I2S_READ_REG((p_i2sx), I2SCFGR));

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_I2SCFG))));
#endif /* USE_ASSERT_DBG_PARAM */

  p_config->mode = (hal_i2s_mode_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_I2SCFG));
  p_config->standard = (hal_i2s_standard_t)((uint32_t)(i2scfgr_reg_value & (SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC)));
  p_config->data_format = (hal_i2s_data_format_t)((uint32_t)(i2scfgr_reg_value
                                                             & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)));
  p_config->clock_polarity = (hal_i2s_clock_polarity_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_CKPOL));
  p_config->bit_order = (hal_i2s_bit_order_t)(LL_I2S_GetTransferBitOrder(p_i2sx));
  p_config->audio_frequency = I2S_GetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx));
}

/**
  * @brief Retrieve the configuration of the I2S peripheral in Slave mode.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param p_config Pointer to @ref hal_i2s_slave_config_t configuration structure.
  */
void HAL_I2S_SLAVE_GetConfig(const hal_i2s_handle_t *hi2s, hal_i2s_slave_config_t *p_config)
{
  SPI_TypeDef *p_i2sx;
  uint32_t i2scfgr_reg_value;

  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  i2scfgr_reg_value = (uint32_t)(LL_I2S_READ_REG((p_i2sx), I2SCFGR));

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_I2S_MODE_SLAVE((hal_i2s_mode_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_I2SCFG))));
#endif /* USE_ASSERT_DBG_PARAM */

  p_config->mode = (hal_i2s_mode_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_I2SCFG));
  p_config->standard = (hal_i2s_standard_t)((uint32_t)(i2scfgr_reg_value & (SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC)));
  p_config->data_format = (hal_i2s_data_format_t)((uint32_t)(i2scfgr_reg_value
                                                             & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)));
  p_config->clock_polarity = (hal_i2s_clock_polarity_t)((uint32_t)(i2scfgr_reg_value & SPI_I2SCFGR_CKPOL));
  p_config->bit_order = (hal_i2s_bit_order_t)(LL_I2S_GetTransferBitOrder(p_i2sx));
}

/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group3 Features functions
  * @{
  This subsection provides functions to configure additional
  features for the selected I2Sx peripheral.

- Master Clock Output configuration:
  - HAL_I2S_MASTER_EnableClockOutput()
  - HAL_I2S_MASTER_DisableClockOutput()
  - HAL_I2S_MASTER_IsEnabledClockOutput()

- Word Select Inversion configuration:
  - HAL_I2S_EnableWSInversion()
  - HAL_I2S_DisableWSInversion()
  - HAL_I2S_IsEnabledWSInversion()

- Master Keep IO State configuration:
  - HAL_I2S_MASTER_EnableKeepIOState()
  - HAL_I2S_MASTER_DisableKeepIOState()
  - HAL_I2S_MASTER_IsEnabledKeepIOState()

- IO Swap configuration:
  - HAL_I2S_EnableIOSwap()
  - HAL_I2S_DisableIOSwap()
  - HAL_I2S_IsEnabledIOSwap()

- Channel Length Detection Error configuration:
  - HAL_I2S_SLAVE_EnableLengthDetectionError()
  - HAL_I2S_SLAVE_DisableLengthDetectionError()
  - HAL_I2S_SLAVE_IsEnabledLengthDetectionError()

- IO lock configuration:
  - HAL_I2S_LockIOConfig()
  - HAL_I2S_IsLockedIOConfig()

- FIFO Threshold configuration:
  - HAL_I2S_SetFifoThreshold()
  - HAL_I2S_GetFifoThreshold()

- Data alignment for 24-bit data format configuration:
  - HAL_I2S_SetData24BitsAlignedRight()
  - HAL_I2S_SetData24BitsAlignedLeft()
  */

/**
  * @brief  Enable I2S Master Clock Output.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_MASTER_EnableClockOutput(hal_i2s_handle_t *hi2s)
{
  hal_i2s_master_audio_frequency_t audio_freq;
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  audio_freq = I2S_GetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx));

  LL_I2S_EnableMasterClock(p_i2sx);

  /* Update the  Word Select audio frequency with master clock output enabled */
  return I2S_SetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx), audio_freq);
}

/**
  * @brief  Disable I2S Master Clock Output.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_MASTER_DisableClockOutput(hal_i2s_handle_t *hi2s)
{
  hal_i2s_master_audio_frequency_t audio_freq;
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  audio_freq = I2S_GetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx));

  LL_I2S_DisableMasterClock(p_i2sx);

  /* Update the Word Select audio frequency with master clock output disabled */
  return I2S_SetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx), audio_freq);
}

/**
  * @brief  Get I2S Master Clock Output status.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Master clock output status.
  */
hal_i2s_master_clk_output_status_t HAL_I2S_MASTER_IsEnabledClockOutput(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_master_clk_output_status_t)LL_I2S_IsEnabledMasterClock(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Enable I2S Word Select Inversion.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_EnableWSInversion(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_EnableWordSelectInversion(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Disable I2S Word Select Inversion.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_DisableWSInversion(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_DisableWordSelectInversion(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Check I2S Word Select Inversion status.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Word select inversion status.
  */
hal_i2s_ws_inversion_status_t HAL_I2S_IsEnabledWSInversion(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_ws_inversion_status_t)LL_I2S_IsEnabledWordSelectInversion(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Enable Master Keep IO State feature. The peripheral always keeps control
  *         of all associated GPIOs to prevent any glitches on the line.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR When IO is locked.
  */
hal_status_t HAL_I2S_MASTER_EnableKeepIOState(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)(LL_I2S_GetTransferMode(p_i2sx))));
#endif /* USE_ASSERT_DBG_PARAM */

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  LL_I2S_Disable(p_i2sx);

  LL_I2S_EnableGPIOControl(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Disable Master Keep IO State feature.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR When IO is locked.
  */
hal_status_t HAL_I2S_MASTER_DisableKeepIOState(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)(LL_I2S_GetTransferMode(p_i2sx))));
#endif /* USE_ASSERT_DBG_PARAM */

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  LL_I2S_Disable(p_i2sx);

  /* Disable the Master Keep IO State */
  LL_I2S_DisableGPIOControl(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Check I2S Master Keep IO State feature status.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Master keep IO state status.
  */
hal_i2s_master_keep_io_state_status_t HAL_I2S_MASTER_IsEnabledKeepIOState(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_master_keep_io_state_status_t)LL_I2S_IsEnabledGPIOControl(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Enable IO Swap.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR When IO is locked.
  */
hal_status_t HAL_I2S_EnableIOSwap(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  LL_I2S_Disable(p_i2sx);

  /* Enable the IO Swap */
  LL_I2S_EnableIOSwap(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Disable IO Swap.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR When IO is locked.
  */
hal_status_t HAL_I2S_DisableIOSwap(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  LL_I2S_Disable(p_i2sx);

  /* Disable the IO Swap */
  LL_I2S_DisableIOSwap(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Check I2S IO swap status.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return IO swap status.
  */
hal_i2s_io_swap_status_t HAL_I2S_IsEnabledIOSwap(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_io_swap_status_t)LL_I2S_IsEnabledIOSwap(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Enable Channel Length detection error only in slave mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SLAVE_EnableLengthDetectionError(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_SLAVE((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  LL_I2S_Disable(p_i2sx);

  /* Enable the channel length detection error */
  LL_I2S_SLAVE_EnableLengthDetectionError(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Disable Channel Length detection error only in slave mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SLAVE_DisableLengthDetectionError(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_SLAVE((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  LL_I2S_Disable(p_i2sx);

  /* Disable the channel length detection error */
  LL_I2S_SLAVE_DisableLengthDetectionError(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Check I2S Channel Length detection error status only in slave mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Slave channel detection error status.
  */
hal_i2s_slave_length_detection_error_status_t HAL_I2S_SLAVE_IsEnabledLengthDetectionError(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_slave_length_detection_error_status_t)
         LL_I2S_SLAVE_IsEnabledLengthDetectionError(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Lock the IO configuration.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @note   The reset of the IOLock bit is done by hardware.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_LockIOConfig(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the I2S channel state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_EnableIOLock(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Retrieve the IO configuration status.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval hal_i2s_io_cfg_status_t.
  */
hal_i2s_io_cfg_status_t HAL_I2S_IsLockedIOConfig(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the I2S state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_io_cfg_status_t)LL_I2S_IsEnabledIOLock(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the FIFO threshold level.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  fifo_threshold This parameter must be a value of @ref hal_i2s_fifo_threshold_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetFifoThreshold(const hal_i2s_handle_t *hi2s, const hal_i2s_fifo_threshold_t fifo_threshold)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_FIFO_THRESHOLD(fifo_threshold));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetFIFOThreshold(p_i2sx, (uint32_t)fifo_threshold);

  return HAL_OK;
}

/**
  * @brief  Retrieve the bit order for data transfers (MSB or LSB bit first).
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S FIFO threshold level configuration.
  */
hal_i2s_fifo_threshold_t HAL_I2S_GetFifoThreshold(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_fifo_threshold_t)LL_I2S_GetFIFOThreshold(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the data alignment to right.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetData24BitsAlignedRight(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(LL_I2S_GetDataFormat(p_i2sx) == LL_I2S_DATA_FORMAT_24_BIT);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetDataAlignmentRight(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Set the data alignment to left.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetData24BitsAlignedLeft(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(LL_I2S_GetDataFormat(p_i2sx) == LL_I2S_DATA_FORMAT_24_BIT);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetDataAlignmentLeft(p_i2sx);

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group4 Items functions
  * @{
  This subsection provides functions to change and retrieve a single configuration in the
  IDLE state.
  - HAL_I2S_SetMode(): Set the mode of the I2S peripheral.
  - HAL_I2S_GetMode(): Retrieve the mode of the I2S peripheral.
  - HAL_I2S_SetStandard(): Set the standard of the I2S peripheral.
  - HAL_I2S_GetStandard(): Retrieve the standard of the I2S peripheral.
  - HAL_I2S_SetDataFormat(): Set the data format of the I2S peripheral.
  - HAL_I2S_GetDataFormat(): Retrieve the data format of the I2S peripheral.
  - HAL_I2S_MASTER_SetAudioFrequency(): Set the master Word Select audio frequency of the I2S peripheral.
  - HAL_I2S_MASTER_GetAudioFrequency(): Retrieve the master Word Select audio frequency of the I2S peripheral.
  - HAL_I2S_SetClockPolarity(): Set the clock polarity of the I2S peripheral.
  - HAL_I2S_GetClockPolarity(): Retrieve the clock polarity of the I2S peripheral.
  - HAL_I2S_SetBitOrder(): Set the bit order for data transfers (MSB or LSB bit first) of the I2S peripheral.
  - HAL_I2S_GetBitOrder(): Retrieve the bit order for data transfers (MSB or LSB bit first) of the I2S peripheral.
  - HAL_I2S_SetTxDMA(): Link the transmit DMA handle to the I2S handle.
  - HAL_I2S_SetRxDMA(): Link the receive DMA handle to the I2S handle.
  */

/**
  * @brief  Set the transfer mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  mode This parameter must be a value of @ref hal_i2s_mode_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetMode(const hal_i2s_handle_t *hi2s, const hal_i2s_mode_t mode)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_MODE(mode));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetTransferMode(p_i2sx, (uint32_t)mode);

  return HAL_OK;
}

/**
  * @brief  Retrieve the transfer mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S mode configuration.
  */
hal_i2s_mode_t HAL_I2S_GetMode(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_mode_t)LL_I2S_GetTransferMode(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the standard.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  standard This parameter must be a value of @ref hal_i2s_standard_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetStandard(const hal_i2s_handle_t *hi2s, const hal_i2s_standard_t standard)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_STANDARD(standard));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetStandard(p_i2sx, (uint32_t)standard);

  return HAL_OK;
}

/**
  * @brief  Retrieve the standard.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S standard configuration.
  */
hal_i2s_standard_t HAL_I2S_GetStandard(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_standard_t)LL_I2S_GetStandard(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the data format.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  format This parameter must be a value of @ref hal_i2s_data_format_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetDataFormat(const hal_i2s_handle_t *hi2s, const hal_i2s_data_format_t format)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_DATA_FORMAT(format));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetDataFormat(p_i2sx, (uint32_t)format);

  return HAL_OK;
}

/**
  * @brief  Retrieve the data format.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S data format configuration.
  */
hal_i2s_data_format_t HAL_I2S_GetDataFormat(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_data_format_t)LL_I2S_GetDataFormat(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the master Word Select audio frequency.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  frequency_ws_hz This parameter must be a value of @ref hal_i2s_master_audio_frequency_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_MASTER_SetAudioFrequency(hal_i2s_handle_t *hi2s,
                                              const hal_i2s_master_audio_frequency_t frequency_ws_hz)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_MASTER_AUDIO_FREQUENCY(frequency_ws_hz));

  p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)(LL_I2S_GetTransferMode(p_i2sx))));
#endif /* USE_ASSERT_DBG_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

  LL_I2S_Disable(p_i2sx);

  return I2S_SetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx), frequency_ws_hz);
}

/**
  * @brief  Retrieve the master Word Select audio frequency.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S closest audio frequency configuration to the real audio frequency.
  */
hal_i2s_master_audio_frequency_t HAL_I2S_MASTER_GetAudioFrequency(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  return I2S_GetAudioFrequency(hi2s, HAL_RCC_SPI_GetKernelClkFreq(p_i2sx));
}

/**
  * @brief  Set the clock polarity.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  clock_polarity This parameter must be a value of @ref hal_i2s_clock_polarity_t.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetClockPolarity(const hal_i2s_handle_t *hi2s, const hal_i2s_clock_polarity_t clock_polarity)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_CLOCK_POLARITY(clock_polarity));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetClockPolarity(p_i2sx, (uint32_t)clock_polarity);

  return HAL_OK;
}

/**
  * @brief  Retrieve the clock polarity.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S clock polarity configuration.
  */
hal_i2s_clock_polarity_t HAL_I2S_GetClockPolarity(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_clock_polarity_t)LL_I2S_GetClockPolarity(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Set the bit order for data transfers (MSB or LSB bit first).
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  bit_order This parameter must be a value of @ref hal_i2s_bit_order_t.
  * @retval HAL_ERROR When IO is locked.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_I2S_SetBitOrder(hal_i2s_handle_t *hi2s, const hal_i2s_bit_order_t bit_order)
{
  SPI_TypeDef *p_i2sx;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(IS_I2S_BIT_ORDER(bit_order));

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  if (LL_I2S_IsEnabledIOLock(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_IO_LOCKED);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  LL_I2S_Disable(p_i2sx);

  LL_I2S_SetTransferBitOrder(p_i2sx, (uint32_t)bit_order);

  return HAL_OK;
}

/**
  * @brief  Retrieve the bit order for data transfers (MSB or LSB bit first).
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return Current I2S bit order configuration.
  */
hal_i2s_bit_order_t HAL_I2S_GetBitOrder(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  return (hal_i2s_bit_order_t)LL_I2S_GetTransferBitOrder(I2S_GET_INSTANCE(hi2s));
}

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
/**
  * @brief  Link the Transmit DMA handle to the I2S handle.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t structure.
  * @param  hdma Pointer to a hal_dma_handle_t structure.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_I2S_SetTxDMA(hal_i2s_handle_t *hi2s, hal_dma_handle_t *hdma)
{
  /* Check the I2S and DMA handle */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(hdma != NULL);

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Link the DMA handle to the I2S handle */
  hi2s->hdma_tx = hdma;
  hdma->p_parent = hi2s;

  return HAL_OK;
}

/**
  * @brief  Link the Receive DMA handle to the I2S handle.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  hdma Pointer to a hal_dma_handle_t structure.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_I2S_SetRxDMA(hal_i2s_handle_t *hi2s, hal_dma_handle_t *hdma)
{
  /* Check the I2S and DMA handle */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(hdma != NULL);

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Link the DMA handle to the I2S handle */
  hi2s->hdma_rx = hdma;
  hdma->p_parent = hi2s;

  return HAL_OK;
}
#endif /* USE_HAL_I2S_DMA */

/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group5 IO operation functions
  * @{
  * This subsection provides a set of functions allowing the management of the I2S data transfers.
  *
  * The I2S supports both master and slave mode:
  * There are two modes of transfer:
  *  - Blocking mode: The communication is performed in polling mode.
  *    The status of all data processing is returned by the same function after finishing the transfer.
  *  - No-Blocking mode: The communication is performed using Interrupts or DMA.
  *    These functions return the status of the transfer startup.
  *    The end of the data processing will be indicated through the dedicated I2S IRQ when using
  *    Interrupt mode or the DMA IRQ when using DMA mode.
  */

/**
  * @brief  Return the real master Word Select audio frequency.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval 0 I2S source clock frequency not active
  * @retval Other value, Real audio frequency in Hz.
  */
uint32_t HAL_I2S_MASTER_GetRealAudioFrequency(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;
  uint32_t frequency_ws_hz = 0U;
  uint32_t i2s_clk;
  uint32_t channel_length = 1U;
  uint32_t i2s_odd = 0U;
  uint32_t i2s_div = 0U;
  uint32_t ispcm = 0U;
  uint32_t prescaler = 1U;
  uint32_t i2scfgr_reg_value;
  uint32_t mckoe;
  hal_i2s_standard_t standard;
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT));

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  i2scfgr_reg_value = (uint32_t)(LL_I2S_READ_REG((p_i2sx), I2SCFGR));

  i2s_clk = HAL_RCC_SPI_GetKernelClkFreq(p_i2sx);

  if (i2s_clk != 0U)
  {
    data_format = (hal_i2s_data_format_t)((uint32_t)(i2scfgr_reg_value & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)));
    i2s_odd = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_ODD) >> SPI_I2SCFGR_ODD_Pos));
    i2s_div = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_I2SDIV) >> SPI_I2SCFGR_I2SDIV_Pos));
    standard = (hal_i2s_standard_t)((uint32_t)(i2scfgr_reg_value & (SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC)));
    mckoe = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_MCKOE) >> SPI_I2SCFGR_MCKOE_Pos));

    if (i2s_div != 0U)
    {
      prescaler = (uint32_t)((2U * i2s_div) + i2s_odd);
    }

    if (IS_I2S_STANDARD_PCM(standard))
    {
      ispcm = 1U;
    }

    if (IS_I2S_CHANNEL_LENGTH_32_BIT(data_format))
    {
      /* Channel length is 32 bits */
      channel_length = 2U;
    }

    /* Reference Manual, SPI/I2S section, Chapter Clock generator:
    Depending on the master clock state (MCKOE = 0 or MCKOE = 1), the frequency of the frame
    synchronization is given by the following formulas */
    if (mckoe != 0U)
    {
      frequency_ws_hz = (uint32_t)(i2s_clk / (((uint32_t)(256UL >> ispcm)) * prescaler));
    }
    else
    {
      frequency_ws_hz = (uint32_t)(i2s_clk / ((((uint32_t)(32UL >> ispcm)) * channel_length) * prescaler));
    }

    return frequency_ws_hz;
  }
  else
  {
    return 0U;
  }
}

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to send in bytes.
  * @param  timeout_ms Timeout duration.
  * @return hal_status_t.
  */
hal_status_t HAL_I2S_Transmit(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample,
                              uint32_t timeout_ms)
{
  SPI_TypeDef *p_i2sx;
  uint32_t tickstart;
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_TX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_ACTIVE);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_data;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->tx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_rx_buff = (uint16_t *)NULL;
  hi2s->rx_xfer_size = (uint16_t) 0U;
  hi2s->rx_xfer_count = (uint16_t) 0U;

  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    while (hi2s->tx_xfer_count > 0U)
    {
      if (LL_I2S_IsActiveFlag_TXP(p_i2sx) != 0U)
      {
        LL_I2S_TransmitData32(p_i2sx, *((const uint32_t *)hi2s->p_tx_buff));
        hi2s->p_tx_buff += 2U;
        hi2s->tx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while (hi2s->tx_xfer_count > 0U)
    {
      if (LL_I2S_IsActiveFlag_TXP(p_i2sx) != 0U)
      {
        LL_I2S_TransmitData16(p_i2sx, *((const uint16_t *)hi2s->p_tx_buff));
        hi2s->p_tx_buff++;
        hi2s->tx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }

  return I2S_CloseTransfer(hi2s);
}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to receive. Transfer size must be a multiplier of the fifo threshold.
  * @param  timeout_ms Timeout duration.
  * @return hal_status_t.
  */
hal_status_t HAL_I2S_Receive(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample, uint32_t timeout_ms)
{
  SPI_TypeDef *p_i2sx;
  uint32_t tickstart;
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_RX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_RX_ACTIVE);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_rx_buff = (uint16_t *)p_data;
  hi2s->rx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_tx_buff = (uint16_t *)NULL;
  hi2s->tx_xfer_size = (uint16_t) 0U;
  hi2s->tx_xfer_count = (uint16_t) 0U;

  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  /* Receive data */
  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    while (hi2s->rx_xfer_count > 0U)
    {
      if (LL_I2S_IsActiveFlag_RXP(p_i2sx) != 0U)
      {
        *((uint32_t *)hi2s->p_rx_buff) = LL_I2S_ReceiveData32(p_i2sx);
        hi2s->p_rx_buff += 2U;
        hi2s->rx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while (hi2s->rx_xfer_count > 0U)
    {
      if (LL_I2S_IsActiveFlag_RXP(p_i2sx) != 0U)
      {
        *((uint16_t *)hi2s->p_rx_buff) = LL_I2S_ReceiveData16(p_i2sx);
        hi2s->p_rx_buff++;
        hi2s->rx_xfer_count--;
      }
      else
      {
        /* Timeout management */
        if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }

  return I2S_CloseTransfer(hi2s);
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_tx_data Pointer to data buffer to transmit.
  * @param  p_rx_data Pointer to data buffer to receive.
  * @param  size_sample Amount of data to send and receive.
  * @param  timeout_ms Timeout duration.
  * @return hal_status_t.
  */
hal_status_t HAL_I2S_TransmitReceive(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                     uint32_t size_sample, uint32_t timeout_ms)
{
  SPI_TypeDef *p_i2sx;
  uint32_t tickstart;
  hal_i2s_data_format_t data_format;
  uint16_t initial_tx_xfer_count;
  uint16_t initial_rx_xfer_count;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_FULL_DUPLEX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_RX_ACTIVE);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_tx_data;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->tx_xfer_count = (uint16_t)size_sample;
  hi2s->p_rx_buff = (uint16_t *)p_rx_data;
  hi2s->rx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;
  initial_tx_xfer_count = (uint16_t)size_sample;
  initial_rx_xfer_count = (uint16_t)size_sample;

  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    while ((initial_tx_xfer_count > 0U) || (initial_rx_xfer_count > 0U))
    {
      if ((LL_I2S_IsActiveFlag_TXP(p_i2sx) != 0U) && (initial_tx_xfer_count != 0U))
      {
        LL_I2S_TransmitData32(p_i2sx, *((const uint32_t *)hi2s->p_tx_buff));
        hi2s->p_tx_buff += 2U;
        hi2s->tx_xfer_count--;
        initial_tx_xfer_count = hi2s->tx_xfer_count;
      }

      if ((LL_I2S_IsActiveFlag_RXP(p_i2sx) != 0U) && (initial_rx_xfer_count != 0U))
      {
        *((uint32_t *)hi2s->p_rx_buff) = LL_I2S_ReceiveData32(p_i2sx);
        hi2s->p_rx_buff += 2U;
        hi2s->rx_xfer_count--;
        initial_rx_xfer_count = hi2s->rx_xfer_count;
      }

      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
      {
        return HAL_TIMEOUT;
      }
    }
  }
  else
  {
    while ((initial_tx_xfer_count > 0U) || (initial_rx_xfer_count > 0U))
    {
      if ((LL_I2S_IsActiveFlag_TXP(p_i2sx) != 0U) && (initial_tx_xfer_count != 0U))
      {
        LL_I2S_TransmitData16(p_i2sx, *((const uint16_t *)hi2s->p_tx_buff));
        hi2s->p_tx_buff++;
        hi2s->tx_xfer_count--;
        initial_tx_xfer_count = hi2s->tx_xfer_count;
      }

      if ((LL_I2S_IsActiveFlag_RXP(p_i2sx) != 0U) && (initial_rx_xfer_count != 0U))
      {
        *((uint16_t *)hi2s->p_rx_buff) = LL_I2S_ReceiveData16(p_i2sx);
        hi2s->p_rx_buff++;
        hi2s->rx_xfer_count--;
        initial_rx_xfer_count = hi2s->rx_xfer_count;
      }

      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >= timeout_ms) && (timeout_ms != HAL_MAX_DELAY)) || (timeout_ms == 0U))
      {
        return HAL_TIMEOUT;
      }
    }
  }

  return I2S_CloseTransfer(hi2s);
}

/**
  * @brief  Transmit an amount of data in Interrupt mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to send.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_Transmit_IT(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = LL_I2S_IT_TXP;
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_TX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_data;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->tx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_rx_buff = (uint16_t *)NULL;
  hi2s->rx_xfer_size = (uint16_t) 0U;
  hi2s->rx_xfer_count = (uint16_t) 0U;

  /* Set the function for IT treatment */
  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    hi2s->p_tx_isr = I2S_Transmit_32Bit_IT;
  }
  else
  {
    hi2s->p_tx_isr = I2S_Transmit_16Bit_IT;
  }

  /* Enable I2S peripheral */
  LL_I2S_Enable(p_i2sx);

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= LL_I2S_IT_UDR;
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* Enable TIFRE interrupt if the mode is Slave  */
  if ((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx) == HAL_I2S_MODE_SLAVE_TX)
  {
    it_mask |= LL_I2S_IT_TIFRE;
  }

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Start the transfer */
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Receive an amount of data in Interrupt mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to receive.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_Receive_IT(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = LL_I2S_IT_RXP;
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_RX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_RX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_rx_buff = (uint16_t *)p_data;
  hi2s->rx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_tx_buff = (uint16_t *)NULL;
  hi2s->tx_xfer_size = (uint16_t)0U;
  hi2s->tx_xfer_count = (uint16_t)0U;

  /* Set the function for IT treatment */
  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    hi2s->p_rx_isr = I2S_Receive_32Bit_IT;
  }
  else
  {
    hi2s->p_rx_isr = I2S_Receive_16Bit_IT;
  }

  /* Enable I2S peripheral */
  LL_I2S_Enable(p_i2sx);

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= LL_I2S_IT_OVR;
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* Enable TIFRE interrupt if the mode is Slave  */
  if ((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx) == HAL_I2S_MODE_SLAVE_RX)
  {
    it_mask |= LL_I2S_IT_TIFRE;
  }

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Start the transfer */
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Transmit and Receive an amount of data in Interrupt mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_tx_data Pointer to data buffer to transmit.
  * @param  p_rx_data Pointer to data buffer to receive.
  * @param  size_sample Amount of data to send and receive.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_TransmitReceive_IT(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                        uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP);
  hal_i2s_data_format_t data_format;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_FULL_DUPLEX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_RX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_tx_data;
  hi2s->p_rx_buff = (uint16_t *)p_rx_data;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->tx_xfer_count = (uint16_t)size_sample;
  hi2s->rx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;

  /* Set the function for IT treatment */
  if ((data_format == HAL_I2S_DATA_FORMAT_24_BIT) || (data_format == HAL_I2S_DATA_FORMAT_32_BIT))
  {
    hi2s->p_tx_isr = I2S_Transmit_32Bit_IT;
    hi2s->p_rx_isr = I2S_Receive_32Bit_IT;
  }
  else
  {
    hi2s->p_tx_isr = I2S_Transmit_16Bit_IT;
    hi2s->p_rx_isr = I2S_Receive_16Bit_IT;
  }

  /* Enable I2S peripheral */
  LL_I2S_Enable(p_i2sx);

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_UDR | LL_I2S_IT_OVR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* Enable TIFRE interrupt if the mode is Slave  */
  if ((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx) == HAL_I2S_MODE_SLAVE_FULL_DUPLEX)
  {
    it_mask |= (LL_I2S_IT_TIFRE);
  }

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Start the transfer */
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
/**
  * @brief  Transmit an amount of data in DMA mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to send.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_Transmit_DMA(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = LL_I2S_IT_TIFRE;
  hal_i2s_data_format_t data_format;
  hal_dma_direct_xfer_config_t p_dma_tx_config;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_tx_node_config;
  hal_dma_node_type_t p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(hi2s->hdma_tx != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_TX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_data;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->tx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_rx_buff = (uint16_t *)NULL;
  hi2s->rx_xfer_size = (uint16_t)0U;
  hi2s->rx_xfer_count = (uint16_t)0U;
  hi2s->p_rx_isr = NULL;
  hi2s->p_tx_isr = NULL;

  hi2s->hdma_tx->p_xfer_halfcplt_cb = I2S_DMATxHalfCplt;
  hi2s->hdma_tx->p_xfer_cplt_cb = I2S_DMATxCplt;
  hi2s->hdma_tx->p_xfer_error_cb = I2S_DMAError;

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hi2s->hdma_tx->p_head_node, &p_dma_tx_node_config, &p_node_type);
    p_dma_tx_config.src_data_width = p_dma_tx_node_config.xfer.src_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hi2s->hdma_tx, &p_dma_tx_config);
  }
#else
  HAL_DMA_GetConfigDirectXfer(hi2s->hdma_tx, &p_dma_tx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  if (((data_format == HAL_I2S_DATA_FORMAT_16_BIT)
       && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_24_BIT)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_32_BIT)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD)))
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

  if ((data_format == HAL_I2S_DATA_FORMAT_16_BIT) || (data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED))
  {
    hi2s->tx_xfer_count = (uint16_t)(size_sample * 2U);
  }
  else
  {
    hi2s->tx_xfer_count = (uint16_t)(size_sample * 4U);
  }

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2s->hdma_tx,
                                     (uint32_t)hi2s->p_tx_buff,
                                     (uint32_t) &((SPI_TypeDef *)((uint32_t)hi2s->instance))->TXDR,
                                     hi2s->tx_xfer_count, HAL_DMA_OPT_IT_DEFAULT) != HAL_OK)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_UDR | LL_I2S_IT_OVR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Enable DMA request for the transmission */
  LL_I2S_EnableDMAReq_TX(p_i2sx);

  /* Enable I2S to start the transfer */
  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Receive an amount of data in DMA mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_data Pointer to data buffer.
  * @param  size_sample Amount of data to receive.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_Receive_DMA(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = LL_I2S_IT_TIFRE;
  hal_i2s_data_format_t data_format;
  hal_dma_direct_xfer_config_t p_dma_rx_config;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_rx_node_config;
  hal_dma_node_type_t  p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Check the I2S allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(hi2s->hdma_rx != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_RX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_RX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_rx_buff = (uint16_t *)p_data;
  hi2s->rx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;

  /* Initialize fields not used in handle to zero */
  hi2s->p_tx_buff = (uint16_t *)NULL;
  hi2s->tx_xfer_size = (uint16_t)0U;
  hi2s->tx_xfer_count = (uint16_t)0U;
  hi2s->p_rx_isr = NULL;
  hi2s->p_tx_isr = NULL;

  hi2s->hdma_rx->p_xfer_halfcplt_cb = I2S_DMARxHalfCplt;
  hi2s->hdma_rx->p_xfer_cplt_cb = I2S_DMARxCplt;
  hi2s->hdma_rx->p_xfer_error_cb = I2S_DMAError;

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hi2s->hdma_rx->p_head_node, &p_dma_rx_node_config, &p_node_type);
    p_dma_rx_config.dest_data_width = p_dma_rx_node_config.xfer.dest_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hi2s->hdma_rx, &p_dma_rx_config);
  }
#else
  HAL_DMA_GetConfigDirectXfer(hi2s->hdma_rx, &p_dma_rx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  if (((data_format == HAL_I2S_DATA_FORMAT_16_BIT)
       && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_24_BIT)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_32_BIT)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD)))
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

  if ((data_format == HAL_I2S_DATA_FORMAT_16_BIT) || (data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED))
  {
    hi2s->rx_xfer_count = (uint16_t)(size_sample * 2U);
  }
  else
  {
    hi2s->rx_xfer_count = (uint16_t)(size_sample * 4U);
  }

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2s->hdma_rx,
                                     (uint32_t) &((SPI_TypeDef *)((uint32_t)hi2s->instance))->RXDR,
                                     (uint32_t)hi2s->p_rx_buff,
                                     hi2s->rx_xfer_count, HAL_DMA_OPT_IT_DEFAULT) != HAL_OK)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_UDR | LL_I2S_IT_OVR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Enable the DMA request for the reception */
  LL_I2S_EnableDMAReq_RX(p_i2sx);

  /* Enable I2S to start the transfer */
  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Transmit and Receive an amount of data in DMA mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_tx_data Pointer to data buffer to transmit.
  * @param  p_rx_data Pointer to data buffer to receive.
  * @param  size_sample Amount of data to send and receive.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  * @retval HAL_ERROR Operation completed with error.
  * @retval HAL_INVALID_PARAM Invalid parameters.
  */
hal_status_t HAL_I2S_TransmitReceive_DMA(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                         uint32_t size_sample)
{
  SPI_TypeDef *p_i2sx;
  uint32_t it_mask = LL_I2S_IT_TIFRE;
  hal_i2s_data_format_t data_format;
  hal_dma_direct_xfer_config_t p_dma_rx_config;
  hal_dma_direct_xfer_config_t p_dma_tx_config;
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hal_dma_node_config_t p_dma_rx_node_config;
  hal_dma_node_config_t p_dma_tx_node_config;
  hal_dma_node_type_t  p_node_type = HAL_DMA_NODE_LINEAR_ADDRESSING;
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(hi2s->hdma_tx != NULL);
  ASSERT_DBG_PARAM(hi2s->hdma_rx != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_sample != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_data == NULL) || (p_rx_data == NULL) || (size_sample == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_IDLE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_FULL_DUPLEX((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));
#if defined(USE_ASSERT_DBG_PARAM)
  if (LL_I2S_GetFIFOThreshold(p_i2sx) != LL_I2S_FIFO_THRESHOLD_1_DATA)
  {
    ASSERT_DBG_PARAM(IS_I2S_TRANSFER_SIZE(((uint32_t)size_sample), ((uint32_t)LL_I2S_GetFIFOThreshold(p_i2sx))));
  }
#endif /* USE_ASSERT_DBG_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2s, global_state, HAL_I2S_STATE_IDLE, HAL_I2S_STATE_TX_RX_ACTIVE);

  data_format = (hal_i2s_data_format_t)LL_I2S_GetDataFormat(p_i2sx);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  hi2s->p_tx_buff = (const uint16_t *)p_tx_data;
  hi2s->p_rx_buff = (uint16_t *)p_rx_data;
  hi2s->tx_xfer_count = (uint16_t)size_sample;
  hi2s->tx_xfer_size = (uint16_t)size_sample;
  hi2s->rx_xfer_count = (uint16_t)size_sample;
  hi2s->rx_xfer_size = (uint16_t)size_sample;

  hi2s->hdma_rx->p_xfer_halfcplt_cb = I2S_DMATxRxHalfCplt;
  hi2s->hdma_rx->p_xfer_cplt_cb = I2S_DMATxRxCplt;
  hi2s->hdma_rx->p_xfer_error_cb = I2S_DMAError;

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hi2s->hdma_tx->p_head_node, &p_dma_tx_node_config, &p_node_type);
    p_dma_tx_config.src_data_width = p_dma_tx_node_config.xfer.src_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hi2s->hdma_tx, &p_dma_tx_config);
  }

  if (hi2s->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    /* Get DMA channel circular transfer configuration */
    HAL_DMA_GetNodeConfig(hi2s->hdma_rx->p_head_node, &p_dma_rx_node_config, &p_node_type);
    p_dma_rx_config.dest_data_width = p_dma_rx_node_config.xfer.dest_data_width;
  }
  else
  {
    /* Get DMA channel direct transfer configuration */
    HAL_DMA_GetConfigDirectXfer(hi2s->hdma_rx, &p_dma_rx_config);
  }

#else
  HAL_DMA_GetConfigDirectXfer(hi2s->hdma_tx, &p_dma_tx_config);
  HAL_DMA_GetConfigDirectXfer(hi2s->hdma_rx, &p_dma_rx_config);
#endif /* USE_HAL_DMA_LINKEDLIST */

  if (((data_format == HAL_I2S_DATA_FORMAT_16_BIT)
       && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_16_BIT)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_HALFWORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_24_BIT)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_24_BIT)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_32_BIT)
          && (p_dma_rx_config.dest_data_width != HAL_DMA_DEST_DATA_WIDTH_WORD))
      || ((data_format == HAL_I2S_DATA_FORMAT_32_BIT)
          && (p_dma_tx_config.src_data_width != HAL_DMA_SRC_DATA_WIDTH_WORD)))
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

  if ((data_format == HAL_I2S_DATA_FORMAT_16_BIT) || (data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED))
  {
    hi2s->tx_xfer_count = (uint16_t)(size_sample * 2U);
  }
  else
  {
    hi2s->tx_xfer_count = (uint16_t)(size_sample * 4U);
  }

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2s->hdma_tx,
                                     (uint32_t)hi2s->p_tx_buff,
                                     (uint32_t) &((SPI_TypeDef *)((uint32_t)hi2s->instance))->TXDR,
                                     hi2s->tx_xfer_count, HAL_DMA_OPT_IT_DEFAULT) != HAL_OK)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

  if ((data_format == HAL_I2S_DATA_FORMAT_16_BIT) || (data_format == HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED))
  {
    hi2s->rx_xfer_count = (uint16_t)(size_sample * 2U);
  }
  else
  {
    hi2s->rx_xfer_count = (uint16_t)(size_sample * 4U);
  }

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2s->hdma_rx,
                                     (uint32_t) &((SPI_TypeDef *)((uint32_t)hi2s->instance))->RXDR,
                                     (uint32_t)hi2s->p_rx_buff,
                                     hi2s->rx_xfer_count, HAL_DMA_OPT_IT_DEFAULT) != HAL_OK)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) &&(USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    hi2s->global_state = HAL_I2S_STATE_IDLE;
    return HAL_ERROR;
  }

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_UDR | LL_I2S_IT_OVR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  LL_I2S_EnableIT(p_i2sx, it_mask);

  /* Enable DMA requests */
  LL_I2S_EnableDMAReq_TX(p_i2sx);
  LL_I2S_EnableDMAReq_RX(p_i2sx);

  /* Enable I2S to start the transfer */
  LL_I2S_Enable(p_i2sx);
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}
#endif /* USE_HAL_I2S_DMA */

/**
  * @brief  Pause the process in master mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @note   During the process of pause and resume, some data will be missed.
  * @note   To prevent any glitches on data line, the function HAL_I2S_MASTER_EnableKeepIOState
  *         must be used.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR No process ongoing.
  */
hal_status_t HAL_I2S_MASTER_Pause(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;
  hal_status_t status = HAL_OK;
  volatile uint32_t count;
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  uint32_t dma_rx_req = 0U;
  uint32_t dma_tx_req = 0U;
#endif /* USE_HAL_I2S_DMA */
  uint32_t it_mask = (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE);

  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_TX_ACTIVE | HAL_I2S_STATE_RX_ACTIVE | HAL_I2S_STATE_TX_RX_ACTIVE);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  dma_rx_req = LL_I2S_IsEnabledDMAReq_RX(p_i2sx);
  dma_tx_req = LL_I2S_IsEnabledDMAReq_TX(p_i2sx);
#endif /* USE_HAL_I2S_DMA */

  /* Compute software timeout based on system core clock */
  count = I2S_DEFAULT_TIMEOUT_MS * (SystemCoreClock / 24U / 1000U);

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_OVR | LL_I2S_IT_UDR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  if ((dma_rx_req == 0U) && (dma_tx_req == 0U))
  {
    LL_I2S_DisableIT(p_i2sx, it_mask);
  }
#else
  LL_I2S_DisableIT(p_i2sx, it_mask);
#endif /* USE_HAL_I2S_DMA */

  hi2s->pause_state = hi2s->global_state;
  hi2s->global_state = HAL_I2S_STATE_PAUSED;

  /* Suspend current transfer */
  LL_I2S_SuspendTransfer(p_i2sx);

  /* Wait for the transfer to stop */
  do
  {
    count--;
    if (count == 0U)
    {
      hi2s->global_state = HAL_I2S_STATE_IDLE;
      status = HAL_TIMEOUT;
      break;
    }
  } while (LL_I2S_IsActiveTransfer(p_i2sx) != 0U);

  return status;
}

/**
  * @brief  Resume the process in master mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @note   During the process of pause and resume, some data will be missed.
  * @note   To prevent any glitches on data line, the function HAL_I2S_MASTER_EnableKeepIOState
  *         must be used.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR No process paused.
  */
hal_status_t HAL_I2S_MASTER_Resume(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  uint32_t dma_rx_req = 0U;
  uint32_t dma_tx_req = 0U;
#endif /* USE_HAL_I2S_DMA */
  uint32_t it_mask = LL_I2S_IT_TIFRE;

  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the global state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_PAUSED);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  ASSERT_DBG_PARAM(IS_I2S_MODE_MASTER((hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx)));

  hi2s->global_state = hi2s->pause_state;
  hi2s->pause_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  dma_rx_req = LL_I2S_IsEnabledDMAReq_RX(p_i2sx);
  dma_tx_req = LL_I2S_IsEnabledDMAReq_TX(p_i2sx);
#endif /* USE_HAL_I2S_DMA */

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  it_mask |= (LL_I2S_IT_OVR |  LL_I2S_IT_UDR);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  if ((dma_rx_req == 0U) && (dma_tx_req == 0U))
#endif /* USE_HAL_I2S_DMA */
  {
    if (hi2s->global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      it_mask |= (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP);
    }
    else if (hi2s->global_state == HAL_I2S_STATE_TX_ACTIVE)
    {
      it_mask |= LL_I2S_IT_TXP;
    }
    else
    {
      it_mask |= LL_I2S_IT_RXP;
    }

    LL_I2S_EnableIT(p_i2sx, it_mask);
  }

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  /* Reset error code to prevent any lingering error caused by the pause resume process */
  hi2s->last_error_codes = HAL_I2S_ERROR_NONE;
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  /* Resume transfer */
  LL_I2S_StartTransfer(p_i2sx);

  return HAL_OK;
}

/**
  * @brief  Abort ongoing transfer in blocking mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_I2S_Abort(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx;
  hal_status_t status = HAL_OK;
  volatile uint32_t count;
  hal_i2s_mode_t mode;

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_TX_ACTIVE | HAL_I2S_STATE_RX_ACTIVE
                   | HAL_I2S_STATE_TX_RX_ACTIVE | HAL_I2S_STATE_PAUSED);

  p_i2sx = I2S_GET_INSTANCE(hi2s);

  /* Disable ITs */
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_OVR
                            | LL_I2S_IT_UDR | LL_I2S_IT_TIFRE));
#else
  LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* Set I2S global state to abort to avoid any interaction */
  hi2s->global_state = HAL_I2S_STATE_ABORT;

  /* Compute software timeout based on system core clock */
  count = I2S_DEFAULT_TIMEOUT_MS * (SystemCoreClock / 24U / 1000U);
  mode = (hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx);

  if (IS_I2S_MODE_MASTER(mode))
  {
    /* Suspend current transfer */
    LL_I2S_SuspendTransfer(p_i2sx);

    /* Wait for transfer to stop */
    do
    {
      count--;
      if (count == 0U)
      {
        hi2s->global_state = HAL_I2S_STATE_IDLE;
        status = HAL_TIMEOUT;
        break;
      }
    } while (LL_I2S_IsActiveTransfer(p_i2sx) != 0U);
  }

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  if (LL_I2S_IsEnabledDMAReq_TX(p_i2sx) != 0U)
  {
    if (hi2s->hdma_tx != NULL)
    {
      if (HAL_DMA_Abort(hi2s->hdma_tx) != HAL_OK)
      {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
        STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
  }

  if (LL_I2S_IsEnabledDMAReq_RX(p_i2sx) != 0U)
  {
    if (hi2s->hdma_rx != NULL)
    {
      if (HAL_DMA_Abort(hi2s->hdma_rx) != HAL_OK)
      {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
        STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
        status = HAL_ERROR;
      }
    }
  }
#endif /* USE_HAL_I2S_DMA */

  I2S_AbortTransfer(hi2s);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

  return status;
}

/**
  * @brief  Abort ongoing transfer in no-blocking mode.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_I2S_Abort_IT(hal_i2s_handle_t *hi2s)
{
  hal_status_t status = HAL_OK;
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  SPI_TypeDef *p_i2sx;
  volatile uint32_t count;
  uint32_t dma_rx_req = 0U;
  uint32_t dma_tx_req = 0U;
  hal_i2s_mode_t mode;
#endif /* USE_HAL_I2S_DMA */

  /* Check the I2S handle allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_TX_ACTIVE | HAL_I2S_STATE_RX_ACTIVE
                   | HAL_I2S_STATE_TX_RX_ACTIVE | HAL_I2S_STATE_PAUSED);

  /* Set I2S global state to abort to avoid any interaction */
  hi2s->global_state = HAL_I2S_STATE_ABORT;

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  p_i2sx = I2S_GET_INSTANCE(hi2s);

  dma_rx_req = LL_I2S_IsEnabledDMAReq_RX(p_i2sx);
  dma_tx_req = LL_I2S_IsEnabledDMAReq_TX(p_i2sx);

  if ((dma_rx_req == 0U) && (dma_tx_req == 0U))
  {
    return HAL_OK;
  }
  else
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_OVR
                              | LL_I2S_IT_UDR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    /* Compute software timeout based on system core clock */
    count = I2S_DEFAULT_TIMEOUT_MS * (SystemCoreClock / 24U / 1000U);

    mode = (hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx);

    /* Only in DMA */
    if (IS_I2S_MODE_MASTER(mode))
    {
      /* Suspend current transfer */
      LL_I2S_SuspendTransfer(p_i2sx);

      /* Wait for communication to stop */
      do
      {
        count--;
        if (count == 0UL)
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          status = HAL_TIMEOUT;
          break;
        }
      } while (LL_I2S_IsActiveTransfer(p_i2sx) != 0U);
    }

    if ((hi2s->hdma_tx != NULL) && (hi2s->hdma_rx != NULL))
    {
      if ((dma_rx_req != 0U) && (dma_tx_req != 0U))
      {
        hi2s->hdma_tx->p_xfer_abort_cb = I2S_DMATxRxAbortCallback;

        if (HAL_DMA_Abort_IT(hi2s->hdma_tx) != HAL_OK)
        {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
          STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
          status = HAL_ERROR;
        }
      }
      else if ((dma_rx_req != 0U) && (dma_tx_req == 0U))
      {
        hi2s->hdma_rx->p_xfer_abort_cb = I2S_DMARxAbortCallback;

        if (HAL_DMA_Abort_IT(hi2s->hdma_rx) != HAL_OK)
        {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
          STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
          status = HAL_ERROR;
        }
      }
      else if ((dma_tx_req != 0U) && (dma_rx_req == 0U))
      {
        hi2s->hdma_tx->p_xfer_abort_cb = I2S_DMATxAbortCallback;

        if (HAL_DMA_Abort_IT(hi2s->hdma_tx) != HAL_OK)
        {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
          STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
          status = HAL_ERROR;
        }
      }
      else
      {
        /* No DMA request */
      }
    }
    else if ((hi2s->hdma_rx != NULL) && (hi2s->hdma_tx == NULL))
    {
      if (dma_rx_req != 0U)
      {
        hi2s->hdma_rx->p_xfer_abort_cb = I2S_DMARxAbortCallback;

        if (HAL_DMA_Abort_IT(hi2s->hdma_rx) != HAL_OK)
        {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
          STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
          status = HAL_ERROR;
        }
      }
    }
    else if ((hi2s->hdma_tx != NULL) && (hi2s->hdma_rx == NULL))
    {
      /* Transmit DMA */
      if (dma_tx_req != 0U)
      {
        hi2s->hdma_tx->p_xfer_abort_cb = I2S_DMATxAbortCallback;

        if (HAL_DMA_Abort_IT(hi2s->hdma_tx) != HAL_OK)
        {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
          STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
          status = HAL_ERROR;
        }
      }
    }
    else
    {
      /* No DMA handle available */
      status = HAL_INVALID_PARAM;
    }
  }

#endif /* USE_HAL_I2S_DMA */

  return status;
}
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group6 IRQ handler/callbacks/register callbacks functions
  * @{
  * This subsection provides functions to process I2S interrupts and register I2S
  * process and error callbacks:
  *  - The function HAL_I2S_IRQHandler() to handle all I2S interrupts
  *
  * There are two ways to use callbacks:
  * Override weak callbacks functions:
  *  - Call the function HAL_I2S_ErrorCallback() to indicate invalid operation is completed
  *  - Call the function HAL_I2S_TxCpltCallback() to indicate Tx transfer is completed
  *  - Call the function HAL_I2S_TxHalfCpltCallback() to indicate Tx half transfer is completed
  *  - Call the function HAL_I2S_RxCpltCallback() to indicate Rx transfer is completed
  *  - Call the function HAL_I2S_RxHalfCpltCallback() to indicate Rx half transfer is completed
  *  - Call the function HAL_I2S_TxRxCpltCallback() to indicate Tx/Rx transfer is completed
  *  - Call the function HAL_I2S_TxRxHalfCpltCallback() to indicate Tx/Rx half transfer is completed
  *  - Call the function HAL_I2S_AbortCpltCallback() to indicate Abort operation is completed
  *
  * Register callbacks:
  *  - Call the function HAL_I2S_RegisterErrorCallback() to register the Error callback
  *  - Call the function HAL_I2S_RegisterTxCpltCallback() to register the Tx transfer complete callback
  *  - Call the function HAL_I2S_RegisterTxHalfCpltCallback() to register the Tx half transfer complete callback
  *  - Call the function HAL_I2S_RegisterRxCpltCallback() to register the Rx transfer complete callback
  *  - Call the function HAL_I2S_RegisterRxHalfCpltCallback() to register the Rx half transfer complete callback
  *  - Call the function HAL_I2S_RegisterTxRxCpltCallback() to register the Tx/Rx transfer complete callback
  *  - Call the function HAL_I2S_RegisterTxRxHalfCpltCallback() to register the Tx/Rx half transfer complete callback
  *  - Call the function HAL_I2S_RegisterAbortCpltCallback() to register the Abort operation complete callback

  Depending on the process function in use, different callbacks can be triggered:

| Process API \n \ \n Callbacks | HAL_I2S_Transmit_IT  | HAL_I2S_Receive_IT | HAL_I2S_TransmitReceive_IT |
|-------------------------------|:--------------------:|:------------------:|:--------------------------:|
| HAL_I2S_TxCpltCallback        |           x          |                    |                            |
| HAL_I2S_RxCpltCallback        |                      |          x         |                            |
| HAL_I2S_TxRxCpltCallback      |                      |                    |              x             |
| HAL_I2S_ErrorCallback         |           x          |          x         |              x             |

| Process API \n \ \n Callbacks  | HAL_I2S_Transmit_DMA  | HAL_I2S_Receive_DMA  | HAL_I2S_TransmitReceive_DMA |
|--------------------------------|:---------------------:|:--------------------:|:---------------------------:|
| HAL_I2S_TxHalfCpltCallback*    |           x           |                      |                             |
| HAL_I2S_TxCpltCallback         |           x           |                      |                             |
| HAL_I2S_RxHalfCpltCallback*    |                       |           x          |                             |
| HAL_I2S_RxCpltCallback         |                       |           x          |                             |
| HAL_I2S_TxRxHalfCpltCallback*  |                       |                      |              x              |
| HAL_I2S_TxRxCpltCallback       |                       |                      |              x              |
| HAL_I2S_ErrorCallback**        |           x           |           x          |              x              |
@note * these callbacks might be called following DMA IRQ management, not I2Sx IRQ management.
@note ** these callbacks might be called following DMA IRQ management, or I2Sx IRQ management.

| Process API \n \ \n Callbacks      | HAL_I2S_Abort_IT  |
|------------------------------------|:-----------------:|
| HAL_I2S_AbortCpltCallback          |         x         |
| HAL_I2S_ErrorCallback              |         x         |

  */

/**
  * @brief Handle I2S interrupt request.
  * @param hi2s Pointer to @ref hal_i2s_handle_t.
  */
void HAL_I2S_IRQHandler(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  uint32_t i2sier = p_i2sx->IER;
  uint32_t i2ssr = p_i2sx->SR;
  uint32_t trigger = i2sier & i2ssr;
  volatile uint32_t count;
  hal_i2s_mode_t mode;

  if (hi2s->global_state == HAL_I2S_STATE_ABORT)
  {
    mode = (hal_i2s_mode_t)LL_I2S_GetTransferMode(p_i2sx);

    /* Compute software timeout based on system core clock */
    count = I2S_DEFAULT_TIMEOUT_MS * (SystemCoreClock / 24U / 1000U);

    if (IS_I2S_MODE_MASTER(mode))
    {
      /* Suspend current transfer */
      LL_I2S_SuspendTransfer(p_i2sx);

      /* Wait for communication to stop */
      do
      {
        count--;
        if (count == 0UL)
        {
          hi2s->global_state = HAL_I2S_STATE_IDLE;
          break;
        }
      } while (LL_I2S_IsActiveTransfer(p_i2sx) != 0U);
    }

    /* Proceed with abort procedure */
    I2S_AbortTransfer(hi2s);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

    /* Call application Abort complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    hi2s->p_abort_cplt_cb(hi2s);
#else
    HAL_I2S_AbortCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }
  else
  {
    if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_DXP))
    {
      hi2s->p_tx_isr(hi2s);
      hi2s->p_rx_isr(hi2s);
    }

    if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_RXP) && STM32_IS_BIT_CLR(trigger, LL_I2S_FLAG_DXP))
    {
      hi2s->p_rx_isr(hi2s);
    }

    if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_TXP) && STM32_IS_BIT_CLR(trigger, LL_I2S_FLAG_DXP))
    {
      hi2s->p_tx_isr(hi2s);
    }
  }

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_UDR))
  {
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));

    LL_I2S_ClearFlag_UDR(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_UDR);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    hi2s->p_error_cb(hi2s);
#else
    HAL_I2S_ErrorCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }

  /* I2S Overrun error interrupt occurred */
  if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_OVR))
  {
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));

    LL_I2S_ClearFlag_OVR(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_OVR);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    hi2s->p_error_cb(hi2s);
#else
    HAL_I2S_ErrorCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* I2S Frame error interrupt occurred */
  if (STM32_IS_BIT_SET(trigger, LL_I2S_FLAG_TIFRE))
  {
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));

    LL_I2S_ClearFlag_FRE(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_FRE);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    hi2s->p_error_cb(hi2s);
#else
    HAL_I2S_ErrorCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }
}

/**
  * @brief   I2S Error callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_ErrorCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_ErrorCallback in the user file when needed.
   */
}

/**
  * @brief   Tx transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_TxCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_TxCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Rx transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_RxCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_RxCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Tx and Rx transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_TxRxCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_TxRxCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Tx half transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_TxHalfCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_TxHalfCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Rx half transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_RxHalfCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_RxHalfCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Tx and Rx half transfer complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_TxRxHalfCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_TxRxHalfCpltCallback in the user file when needed.
   */
}

/**
  * @brief   Abort complete callback.
  * @param   hi2s Pointer to @ref hal_i2s_handle_t.
  * @warning Do not modify this weak function. Implement the callback in the user file when needed.
  */
__weak void HAL_I2S_AbortCpltCallback(hal_i2s_handle_t *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2s);

  /* WARNING: Do not modify this function. Implement the HAL_I2S_AbortCpltCallback in the user file when needed.
   */
}

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register the Error callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Error callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterErrorCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Tx complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Tx complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterTxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Rx complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Rx complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterRxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Tx/Rx complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Tx/Rx complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterTxRxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_tx_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Tx half complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Tx half complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterTxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_tx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Rx half complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Rx half complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterRxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_rx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Tx/Rx half complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Tx/Rx half complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterTxRxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_tx_rx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the Abort complete callback.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  p_callback Pointer to the Abort complete callback function.
  * @retval HAL_INVALID_PARAM Invalid Callback pointer.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_I2S_RegisterAbortCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

  ASSERT_DBG_STATE(hi2s->global_state, HAL_I2S_STATE_INIT | HAL_I2S_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2s->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group7 Peripheral current frequency, state and errors functions
  * @{
  * This subsection provides functions to read peripheral current frequency, current state
  * and last occurred errors.
  *  - HAL_I2S_GetClockFreq() function to retrieve the current clock frequency of the I2S peripheral.
  *  - HAL_I2S_GetState() function to retrieve the current state of the I2S peripheral.
  *  - HAL_I2S_GetLastErrorCodes() function to retrieve the error codes in case of HAL_ERROR return
  *    available under the compilation switch USE_HAL_I2S_GET_LAST_ERRORS.
  */

/**
  * @brief  Return the peripheral clock frequency for I2S.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval uint32_t Frequency in Hz.
  * @retval 0 Source clock of the hi2s not configured or not ready.
  */
uint32_t HAL_I2S_GetClockFreq(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle & config allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  return HAL_RCC_SPI_GetKernelClkFreq(I2S_GET_INSTANCE(hi2s));
}

/**
  * @brief  Retrieve the I2S handle state.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @return hal_i2s_state_t I2S state.
  */
hal_i2s_state_t HAL_I2S_GetState(const hal_i2s_handle_t *hi2s)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  /* Return I2S handle state */
  return hi2s->global_state;
}

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
/**
  * @brief  Retrieve the I2S errors codes.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval uint32_t Returned value can be a combination of the following values:
  *         @arg HAL_I2S_ERROR_NONE
  *         @arg HAL_I2S_ERROR_OVR
  *         @arg HAL_I2S_ERROR_UDR
  *         @arg HAL_I2S_ERROR_DMA
  *         @arg HAL_I2S_ERROR_FRE
  *         @arg HAL_I2S_ERROR_IO_LOCKED
  */
uint32_t HAL_I2S_GetLastErrorCodes(const hal_i2s_handle_t *hi2s)
{
  /* Check the I2S handle */
  ASSERT_DBG_PARAM(hi2s != (void *)NULL);

  /* Check the state */
  ASSERT_DBG_STATE(hi2s->global_state, (HAL_I2S_STATE_IDLE           \
                                        | HAL_I2S_STATE_TX_ACTIVE    \
                                        | HAL_I2S_STATE_RX_ACTIVE    \
                                        | HAL_I2S_STATE_TX_RX_ACTIVE \
                                        | HAL_I2S_STATE_PAUSED       \
                                        | HAL_I2S_STATE_ABORT        \
                                        | HAL_I2S_STATE_INIT));

  return hi2s->last_error_codes;
}
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup I2S_Exported_Functions_Group8 Acquire/release Bus functions
  * @{
  * This subsection provides functions to acquire/release the bus based on the HAL OS
  * abstraction layer (stm32_hal_os.c/.h osal):

  * - Call HAL_I2S_AcquireBus() from thread mode only (not from handler mode, i.e., from ISR).
  * - Call HAL_I2S_ReleaseBus() from thread mode or from handler mode, i.e., from ISR.
 */
/**
  * @brief  Acquire the I2S bus using the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param  timeout_ms Time to wait before the bus is occupied by the handle.
  * @note   Call HAL_I2S_AcquireBus from thread mode only (not from handler mode, i.e., from ISR).
  * @retval HAL_OK Operation started successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_I2S_AcquireBus(hal_i2s_handle_t *hi2s, uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  if (HAL_OS_SemaphoreTake(&hi2s->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the I2S bus thanks to the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @note   The HAL_I2S_ReleaseBus function can be called from thread mode or from handler mode i.e from ISR.
  * @retval HAL_OK Operation started successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_I2S_ReleaseBus(hal_i2s_handle_t *hi2s)
{
  hal_status_t status = HAL_ERROR;

  /* Check the parameters */
  ASSERT_DBG_PARAM(hi2s != NULL);

  if (HAL_OS_SemaphoreRelease(&hi2s->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_I2S_USER_DATA) && (USE_HAL_I2S_USER_DATA == 1)
/** @defgroup I2S_Exported_Functions_Group9 Set/Get user data
  * @{
  * A set of functions to manage a user data pointer stored in the I2S handle:
  *  - HAL_I2S_SetUserData() Set the user data in the handle
  *  - HAL_I2S_GetUserData() Get the user data from the handle
  */

/**
  * @brief Store the user data pointer in the handle.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t.
  * @param p_user_data Pointer to the user data.
  */
void HAL_I2S_SetUserData(hal_i2s_handle_t *hi2s, const void *p_user_data)
{
  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  hi2s->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the user data pointer from the handle.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t.
  * @retval Pointer to the user data.
  */
const void *HAL_I2S_GetUserData(const hal_i2s_handle_t *hi2s)
{
  /* Check parameters allocation */
  ASSERT_DBG_PARAM(hi2s != NULL);

  return (hi2s->p_user_data);
}
/**
  * @}
  */
#endif /* USE_HAL_I2S_USER_DATA */
/**
  * @}
  */

/** @addtogroup I2S_Private_Functions I2S Private Functions
  * @{
  */
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
/**
  * @brief DMA I2S transmit process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t structure that contains
  *             the configuration information for the specified DMA module.
  */
static void I2S_DMATxCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_tx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_UDR
                              | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    /* Disable Tx DMA Request */
    LL_I2S_DisableDMAReq_TX(p_i2sx);
    hi2s->tx_xfer_count = (uint16_t)0U;

    I2S_WaitTxFifoEmpty(hi2s);
    LL_I2S_Disable(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;
  }

  /* Call application Tx complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_tx_cplt_cb(hi2s);
#else
  HAL_I2S_TxCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S transmit process half complete callback.
  * @param  hdma pointer to a hal_dma_handle_t structure that contains
  *         the configuration information for the specified DMA module.
  */
static void I2S_DMATxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* Call application Tx half complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_tx_half_cplt_cb(hi2s);
#else
  HAL_I2S_TxHalfCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S receive process complete callback.
  * @param  hdma pointer to a hal_dma_handle_t structure that contains
  *         the configuration information for the specified DMA module.
  */
static void I2S_DMARxCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_UDR
                              | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    /* Disable Rx DMA Request */
    LL_I2S_DisableDMAReq_RX(p_i2sx);
    hi2s->rx_xfer_count = (uint16_t)0U;

    LL_I2S_Disable(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;
  }

  /* Call application Rx complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_rx_cplt_cb(hi2s);
#else
  HAL_I2S_RxCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S receive process half complete callback.
  * @param  hdma pointer to a hal_dma_handle_t structure that contains
  *         the configuration information for the specified DMA module.
  */
static void I2S_DMARxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* Call application Rx half complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_rx_half_cplt_cb(hi2s);
#else
  HAL_I2S_RxHalfCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S transmit receive process complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void I2S_DMATxRxCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hi2s->hdma_rx->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_UDR
                              | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    /* Disable Tx DMA Request */
    LL_I2S_DisableDMAReq_TX(p_i2sx);
    hi2s->tx_xfer_count = (uint16_t)0U;

    /* Disable Rx DMA Request */
    LL_I2S_DisableDMAReq_RX(p_i2sx);
    hi2s->rx_xfer_count = (uint16_t)0U;

    I2S_WaitTxFifoEmpty(hi2s);
    LL_I2S_Disable(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;
  }

  /* Call application TxRx complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_tx_rx_cplt_cb(hi2s);
#else
  HAL_I2S_TxRxCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S transmit receive process half complete callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void I2S_DMATxRxHalfCplt(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* Call application TxRx half complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_tx_rx_half_cplt_cb(hi2s);
#else
  HAL_I2S_TxRxHalfCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S communication error callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void I2S_DMAError(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

  /* Disable Rx and Tx DMA Request */
  LL_I2S_Disable(p_i2sx);
  LL_I2S_DisableDMAReq_RX(p_i2sx);
  LL_I2S_DisableDMAReq_TX(p_i2sx);
  hi2s->tx_xfer_count = (uint16_t) 0UL;
  hi2s->rx_xfer_count = (uint16_t) 0UL;

  hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  /* Set the error code and execute error callback*/
  STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_DMA);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

  /* Call application error callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_error_cb(hi2s);
#else
  HAL_I2S_ErrorCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief   DMA I2S Tx communication abort callback, when initiated by the application.
  *          (To be called at end of DMA Tx abort procedure following the application abort request).
  * @param   hdma DMA handle.
  * @warning When this callback is executed, the Abort complete callback is called only if no
  *          abort is still ongoing for the Rx DMA handle.
  */
static void I2S_DMATxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* Call the Abort procedure */
  I2S_AbortTransfer(hi2s);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_abort_cplt_cb(hi2s);
#else
  HAL_I2S_AbortCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief   DMA I2S Rx communication abort callback, when initiated by the application.
  *          (To be called at end of DMA Rx abort procedure following the application abort request).
  * @param   hdma DMA handle.
  * @warning When this callback is executed, the Abort complete callback is called only if no
  *          abort is still ongoing for the Tx DMA handle.
  */
static void I2S_DMARxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  /* Call the Abort procedure */
  I2S_AbortTransfer(hi2s);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  hi2s->p_abort_cplt_cb(hi2s);
#else
  HAL_I2S_AbortCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief   DMA I2S Tx Rx communication abort callback, when initiated by the application.
  *          (To be called at end of DMA Tx Rx abort procedure following the application abort request).
  * @param   hdma DMA handle.
  * @warning When this callback is executed, the Abort complete callback is called only if no
  *          abort is still ongoing for the DMA handle.
  */
static void I2S_DMATxRxAbortCallback(hal_dma_handle_t *hdma)
{
  hal_i2s_handle_t *hi2s = (hal_i2s_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;

  hi2s->hdma_rx->p_xfer_abort_cb = I2S_DMARxAbortCallback;

  (void)HAL_DMA_Abort_IT(hi2s->hdma_rx);
}
#endif /* USE_HAL_I2S_DMA */

/**
  * @brief  Manage the transmission 16-bit in Interrupt context.
  * @param  hi2s pointer to a @ref hal_i2s_handle_t.
  */
static void I2S_Transmit_16Bit_IT(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

  if (hi2s->tx_xfer_count != 0UL)
  {
    LL_I2S_TransmitData16(p_i2sx, (*hi2s->p_tx_buff));
    hi2s->p_tx_buff++;
    hi2s->tx_xfer_count--;
  }
  else
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_UDR
                              | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    if (hi2s->global_state == HAL_I2S_STATE_TX_ACTIVE)
    {
      I2S_WaitTxFifoEmpty(hi2s);

      LL_I2S_Disable(p_i2sx);

      hi2s->global_state = HAL_I2S_STATE_IDLE;

      /* Call application Tx complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
      hi2s->p_tx_cplt_cb(hi2s);
#else
      HAL_I2S_TxCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  Manage the transmission 32-bit in Interrupt context.
  * @param  hi2s pointer to a @ref hal_i2s_handle_t.
  */
static void I2S_Transmit_32Bit_IT(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

  if (hi2s->tx_xfer_count != 0UL)
  {
    LL_I2S_TransmitData32(p_i2sx, (*((const uint32_t *)hi2s->p_tx_buff)));
    hi2s->p_tx_buff += 2U;
    hi2s->tx_xfer_count--;
  }
  else
  {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_UDR
                              | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
    LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

    if (hi2s->global_state == HAL_I2S_STATE_TX_ACTIVE)
    {
      I2S_WaitTxFifoEmpty(hi2s);

      LL_I2S_Disable(p_i2sx);

      hi2s->global_state = HAL_I2S_STATE_IDLE;

      /* Call application Tx complete callback */
#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
      hi2s->p_tx_cplt_cb(hi2s);
#else
      HAL_I2S_TxCpltCallback(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  Manage the reception 16-bit in Interrupt context.
  * @param  hi2s pointer to a @ref hal_i2s_handle_t.
  */
static void I2S_Receive_16Bit_IT(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  hal_i2s_state_t tmp_global_state = hi2s->global_state;

  (*((uint16_t *)hi2s->p_rx_buff)) = LL_I2S_ReceiveData16(p_i2sx);
  hi2s->p_rx_buff++;
  hi2s->rx_xfer_count--;

  if (hi2s->rx_xfer_count == 0U)
  {
    if (hi2s->global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TXP | LL_I2S_IT_DXP
                                | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */
    }
    else
    {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */
    }

    if (hi2s->global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      I2S_WaitTxFifoEmpty(hi2s);
    }

    LL_I2S_Disable(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    if (tmp_global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      hi2s->p_tx_rx_cplt_cb(hi2s);
    }
    else
    {
      hi2s->p_rx_cplt_cb(hi2s);
    }
#else
    if (tmp_global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      HAL_I2S_TxRxCpltCallback(hi2s);
    }
    else
    {
      HAL_I2S_RxCpltCallback(hi2s);
    }
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Manage the reception 32-bit in Interrupt context.
  * @param  hi2s pointer to a @ref hal_i2s_handle_t.
  */
static void I2S_Receive_32Bit_IT(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  hal_i2s_state_t tmp_global_state = hi2s->global_state;

  (*((uint32_t *)hi2s->p_rx_buff)) = LL_I2S_ReceiveData32(p_i2sx);
  hi2s->p_rx_buff += 2U;
  hi2s->rx_xfer_count--;

  if (hi2s->rx_xfer_count == 0UL)
  {
    if (hi2s->global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TXP | LL_I2S_IT_DXP
                                | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TXP | LL_I2S_IT_DXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

      I2S_WaitTxFifoEmpty(hi2s);
    }
    else
    {
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_UDR | LL_I2S_IT_OVR | LL_I2S_IT_TIFRE));
#else
      LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_RXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */
    }

    LL_I2S_Disable(p_i2sx);

    hi2s->global_state = HAL_I2S_STATE_IDLE;

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
    if (tmp_global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      hi2s->p_tx_rx_cplt_cb(hi2s);
    }
    else
    {
      hi2s->p_rx_cplt_cb(hi2s);
    }
#else
    if (tmp_global_state == HAL_I2S_STATE_TX_RX_ACTIVE)
    {
      HAL_I2S_TxRxCpltCallback(hi2s);
    }
    else
    {
      HAL_I2S_RxCpltCallback(hi2s);
    }
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
  }
}

/**
  * @brief Abort Transfer and clear flags.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t.
  */
static void I2S_AbortTransfer(const hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);

  /* Disable I2S peripheral */
  LL_I2S_Disable(p_i2sx);

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  /* Disable DMA request */
  LL_I2S_DisableDMAReq_TX(p_i2sx);
  LL_I2S_DisableDMAReq_RX(p_i2sx);
#endif /* USE_HAL_I2S_DMA */

  /* Clear the error flags in the SR register */
#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  LL_I2S_ClearFlag_OVR(p_i2sx);
  LL_I2S_ClearFlag_UDR(p_i2sx);
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */
  LL_I2S_ClearFlag_SUSP(p_i2sx);
  LL_I2S_ClearFlag_FRE(p_i2sx);
}

/**
  * @brief  Close Transfer and clear flags.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t structure which contains
  *              the I2S instance.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
static hal_status_t I2S_CloseTransfer(hal_i2s_handle_t *hi2s)
{
  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  hal_status_t status = HAL_OK;

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_OVR
                            | LL_I2S_IT_UDR | LL_I2S_IT_TIFRE));
#else
  LL_I2S_DisableIT(p_i2sx, (LL_I2S_IT_TXP | LL_I2S_IT_RXP | LL_I2S_IT_TIFRE));
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

#if defined(USE_HAL_I2S_OVR_UDR_ERRORS) && (USE_HAL_I2S_OVR_UDR_ERRORS == 1)
  /* Report UnderRun error for non RX Only communication */
  if (hi2s->global_state != HAL_I2S_STATE_RX_ACTIVE)
  {
    if (LL_I2S_IsActiveFlag_UDR(p_i2sx) != 0U)
    {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_UDR);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
      LL_I2S_ClearFlag_UDR(p_i2sx);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (hi2s->global_state != HAL_I2S_STATE_TX_ACTIVE)
  {
    if (LL_I2S_IsActiveFlag_OVR(p_i2sx) != 0U)
    {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
      STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_OVR);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
      LL_I2S_ClearFlag_OVR(p_i2sx);
    }
  }
#endif /* USE_HAL_I2S_OVR_UDR_ERRORS */

  /* I2S frame error interrupt occurred */
  if (LL_I2S_IsActiveFlag_FRE(p_i2sx) != 0U)
  {
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
    STM32_SET_BIT(hi2s->last_error_codes, HAL_I2S_ERROR_FRE);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
    LL_I2S_ClearFlag_FRE(p_i2sx);
    status = HAL_ERROR;
  }

  /* Wait TX FIFO empty if TX direction is used */
  if (hi2s->global_state != HAL_I2S_STATE_RX_ACTIVE)
  {
    I2S_WaitTxFifoEmpty(hi2s);
  }

  LL_I2S_Disable(p_i2sx);

  hi2s->global_state = HAL_I2S_STATE_IDLE;

  return status;
}

/**
  * @brief  Compute the Word Select audio frequency following register configuration.
  * @param  hi2s Pointer to a @ref hal_i2s_handle_t structure which contains
  *              the I2S instance.
  * @param  i2s_clk_hz Value of the I2S kernel clock.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
static hal_i2s_master_audio_frequency_t I2S_GetAudioFrequency(const hal_i2s_handle_t *hi2s, uint32_t i2s_clk_hz)
{
  uint32_t real_audio_frequency_ws_hz = 0U;
  uint32_t ispcm = 0U;
  uint32_t channel_length = 1U;
  uint32_t index_freq = 0U;
  uint32_t tmp_index_freq = 0U;
  uint32_t min_diff_freq = 0U;
  uint32_t diff_freq = 0U;
  uint32_t prescaler = 1U;

  const uint32_t audio_frequencies[I2S_NUMBER_FREQUENCY] = {(uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_8_KHZ,  \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_11_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_16_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_22_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_32_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_44_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_48_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_96_KHZ, \
                                                            (uint32_t)HAL_I2S_MASTER_AUDIO_FREQ_192_KHZ
                                                           };

  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  uint32_t i2scfgr_reg_value = (uint32_t)(LL_I2S_READ_REG((p_i2sx), I2SCFGR));
  hal_i2s_standard_t standard = (hal_i2s_standard_t)((uint32_t)(i2scfgr_reg_value
                                                                & (SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC)));
  hal_i2s_data_format_t data_format = (hal_i2s_data_format_t)((uint32_t)(i2scfgr_reg_value
                                                                         & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)));
  uint32_t mckoe = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_MCKOE) >> SPI_I2SCFGR_MCKOE_Pos));
  uint32_t i2s_odd = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_ODD) >> SPI_I2SCFGR_ODD_Pos));
  uint32_t i2s_div = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_I2SDIV) >> SPI_I2SCFGR_I2SDIV_Pos));

  if (i2s_div != 0U)
  {
    prescaler = (uint32_t)((2U * i2s_div) + i2s_odd);
  }

  if (IS_I2S_STANDARD_PCM(standard))
  {
    ispcm = 1U;
  }

  if (IS_I2S_CHANNEL_LENGTH_32_BIT(data_format))
  {
    /* Channel length is 32 bits */
    channel_length = 2U;
  }

  /* Reference Manual, SPI/I2S section, Chapter Clock generator:
  Depending on the master clock state (MCKOE = 0 or MCKOE = 1), the frequency of the frame
  synchronization is given by the following formulas */
  if (mckoe != 0U)
  {
    real_audio_frequency_ws_hz = (uint32_t)(i2s_clk_hz / (((uint32_t)(256UL >> ispcm)) * prescaler));
  }
  else
  {
    real_audio_frequency_ws_hz = (uint32_t)(i2s_clk_hz / ((((uint32_t)(32UL >> ispcm)) * channel_length) * prescaler));
  }

  /* Find the difference between the lowest audio frequency and the real frequency */
  if (real_audio_frequency_ws_hz > audio_frequencies[0])
  {
    min_diff_freq = (real_audio_frequency_ws_hz - audio_frequencies[0]);
  }
  else
  {
    min_diff_freq = (audio_frequencies[0] - real_audio_frequency_ws_hz);
  }

  /* Parse all the audio frequencies to find the closest to the real frequency */
  for (tmp_index_freq = 1U; tmp_index_freq < I2S_NUMBER_FREQUENCY; tmp_index_freq++)
  {
    if (real_audio_frequency_ws_hz > audio_frequencies[tmp_index_freq])
    {
      diff_freq = (real_audio_frequency_ws_hz - audio_frequencies[tmp_index_freq]);
    }
    else
    {
      diff_freq = (audio_frequencies[tmp_index_freq] - real_audio_frequency_ws_hz);
    }

    /* Update smallest difference found */
    if (diff_freq < min_diff_freq)
    {
      min_diff_freq = diff_freq;
      index_freq = tmp_index_freq;
    }
  }

  return (hal_i2s_master_audio_frequency_t)audio_frequencies[index_freq];
}

/**
  * @brief Compute prescaler following register configuration and Word Select audio frequency.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t structure which contains
  *             the I2S instance.
  * @param i2s_clk_hz Value of the I2S kernel clock.
  * @param audio_frequency_ws_hz Value of the word select audio frequency in Hz.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
static hal_status_t I2S_SetAudioFrequency(const hal_i2s_handle_t *hi2s, uint32_t i2s_clk_hz,
                                          hal_i2s_master_audio_frequency_t audio_frequency_ws_hz)
{
  uint32_t i2s_prescaler = 0U;
  uint32_t ispcm = 0U;
  uint32_t channel_length = 1U;
  uint32_t i2s_div = 2U;
  uint32_t i2s_odd = 0U;

  SPI_TypeDef *p_i2sx = I2S_GET_INSTANCE(hi2s);
  uint32_t i2scfgr_reg_value = (uint32_t)(LL_I2S_READ_REG((p_i2sx), I2SCFGR));
  hal_i2s_standard_t standard = (hal_i2s_standard_t)((uint32_t)(i2scfgr_reg_value
                                                                & (SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC)));
  hal_i2s_data_format_t data_format = (hal_i2s_data_format_t)((uint32_t)(i2scfgr_reg_value
                                                                         & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)));
  uint32_t mckoe = ((uint32_t)((i2scfgr_reg_value & SPI_I2SCFGR_MCKOE) >> SPI_I2SCFGR_MCKOE_Pos));

  if ((uint32_t)audio_frequency_ws_hz == 0U)
  {
    return HAL_ERROR;
  }

  if (IS_I2S_MASTER_AUDIO_FREQUENCY(audio_frequency_ws_hz))
  {
    if (IS_I2S_STANDARD_PCM(standard))
    {
      ispcm = 1U;
    }

    if (IS_I2S_CHANNEL_LENGTH_32_BIT(data_format))
    {
      /* Channel length is 32 bits */
      channel_length = 2U;
    }

    /* Reference Manual, SPI/I2S section, Chapter Clock generator:
    Depending on the master clock state (MCKOE = 0 or MCKOE = 1), the frequency of the frame
    synchronization is given by the following formulas */
    if (mckoe != 0U)
    {
      i2s_prescaler = (uint32_t)((((i2s_clk_hz / ((uint32_t)(256UL >> ispcm))) * 10U)
                                  / ((uint32_t)audio_frequency_ws_hz)) + 5U);
    }
    else
    {
      i2s_prescaler = (uint32_t)((((i2s_clk_hz / (((uint32_t)(32UL >> ispcm)) * channel_length)) * 10U)
                                  / ((uint32_t)audio_frequency_ws_hz)) + 5U);
    }

    /* Remove the floating point */
    i2s_prescaler = i2s_prescaler / 10U;

    /* Check the parity of the divider */
    i2s_odd = (uint32_t)(i2s_prescaler & (uint32_t)1U);

    /* Compute the i2sdiv prescaler */
    i2s_div = (uint32_t)((i2s_prescaler - i2s_odd) / 2U);

    /* Test if the obtain values are forbidden or out of range */
    if (((i2s_odd == 1U) && (i2s_div == 1U)) || (i2s_div > 0xFFU))
    {
      return HAL_ERROR;
    }

    /* Force i2smod to 1 just to be sure that (2xi2sdiv + i2sodd) is always higher than 0 */
    if (i2s_div == 0U)
    {
      i2s_odd = 1U;
    }
  }

  LL_I2S_SetPrescaler(p_i2sx, i2s_div, i2s_odd);

  return HAL_OK;
}

/**
  * @brief Wait for Tx FIFO to be completely empty.
  * @param hi2s Pointer to a @ref hal_i2s_handle_t structure which contains
  *             the I2S instance.
  */
static void I2S_WaitTxFifoEmpty(hal_i2s_handle_t *hi2s)
{
  volatile uint32_t count;
  uint32_t i2s_kernel_divider;
  uint32_t i2s_kernel_clock = HAL_RCC_SPI_GetKernelClkFreq(I2S_GET_INSTANCE(hi2s));
  uint32_t i2s_prescaler;
  hal_i2s_mode_t mode = (hal_i2s_mode_t)LL_I2S_GetTransferMode(I2S_GET_INSTANCE(hi2s));

  if (i2s_kernel_clock != 0U)
  {
    /* I2S presecaler is equal to (2* i2s_div) + odd */
    if (IS_I2S_MODE_MASTER(mode))
    {
      i2s_prescaler = (2U * LL_I2S_GetPrescalerLinear(I2S_GET_INSTANCE(hi2s)))
                      + LL_I2S_GetPrescalerParity(I2S_GET_INSTANCE(hi2s));
    }
    else
    {
      /* Worst case for slave mode */
      i2s_prescaler = (uint32_t)((((i2s_kernel_clock / (((uint32_t)(16)) * 2U))) / ((uint32_t)8000)));
    }

    /* Calculation of ratio between SystemCoreClock and I2S kernel Clock
    up to the nearest multiple of the divisor */
    i2s_kernel_divider = (((SystemCoreClock) + ((i2s_kernel_clock) - 1U)) / (i2s_kernel_clock));

    /* Number of SystemCoreClock cycles to flush data inside FIFO */
    count = ((I2S_FIFO_SIZE_BYTE * i2s_kernel_divider * i2s_prescaler) / 3U) * 16U;

    /* Wait loop: 3 to 4 SystemCoreClock cycles to execute each loop. Depending on pipeline */
    do
    {
      count--;
    } while (count != 0U);
  }
}

/**
  * @}
  */

#endif /* USE_HAL_I2S_MODULE */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
