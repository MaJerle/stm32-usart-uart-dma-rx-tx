/**
  ******************************************************************************
  * @file    stm32c5xx_hal_i2s.h
  * @brief   Header file for the I2S HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5xx_HAL_I2S_H
#define STM32C5xx_HAL_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_spi.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup I2S I2S
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2S_Exported_Types HAL I2S types
  * @{
  */

/**
  * @brief HAL I2S instances enumeration definition.
  */
typedef enum
{
  HAL_I2S1 = SPI1_BASE,      /*!< I2S1 instance  */
  HAL_I2S2 = SPI2_BASE,      /*!< I2S2 instance  */
#if defined(SPI3)
  HAL_I2S3 = SPI3_BASE,      /*!< I2S3 instance  */
#endif /* SPI3 */
} hal_i2s_t;

/**
  * @brief HAL I2S mode enumeration definition.
  */
typedef enum
{
  HAL_I2S_MODE_SLAVE_TX           = LL_I2S_MODE_SLAVE_TX,            /*!< Slave transmit mode     */
  HAL_I2S_MODE_SLAVE_RX           = LL_I2S_MODE_SLAVE_RX,            /*!< Slave receive mode      */
  HAL_I2S_MODE_MASTER_TX          = LL_I2S_MODE_MASTER_TX,           /*!< Master transmit mode    */
  HAL_I2S_MODE_MASTER_RX          = LL_I2S_MODE_MASTER_RX,           /*!< Master receive mode     */
  HAL_I2S_MODE_SLAVE_FULL_DUPLEX  = LL_I2S_MODE_SLAVE_FULL_DUPLEX,   /*!< Slave full duplex mode  */
  HAL_I2S_MODE_MASTER_FULL_DUPLEX = LL_I2S_MODE_MASTER_FULL_DUPLEX,  /*!< Master full duplex mode */
} hal_i2s_mode_t;

/**
  * @brief HAL I2S standard.
  */
typedef enum
{
  HAL_I2S_STANDARD_PHILIPS   = LL_I2S_STANDARD_PHILIPS,    /*!< PHILIPS standard                              */
  HAL_I2S_STANDARD_MSB       = LL_I2S_STANDARD_MSB,        /*!< MSB-justified standard                        */
  HAL_I2S_STANDARD_LSB       = LL_I2S_STANDARD_LSB,        /*!< LSB-justified standard                        */
  HAL_I2S_STANDARD_PCM_SHORT = LL_I2S_STANDARD_PCM_SHORT,  /*!< PCM standard with short frame synchronization */
  HAL_I2S_STANDARD_PCM_LONG  = LL_I2S_STANDARD_PCM_LONG,   /*!< PCM standard with long frame synchronization  */
} hal_i2s_standard_t;

/**
  * @brief HAL I2S data format.
  */
typedef enum
{
  HAL_I2S_DATA_FORMAT_16_BIT          = LL_I2S_DATA_FORMAT_16_BIT,          /*!< 16-bit on 16-bit channel */
  HAL_I2S_DATA_FORMAT_16_BIT_EXTENDED = LL_I2S_DATA_FORMAT_16_BIT_EXTENDED, /*!< 16-bit on 32-bit channel */
  HAL_I2S_DATA_FORMAT_24_BIT          = LL_I2S_DATA_FORMAT_24_BIT,          /*!< 24-bit on 32-bit channel */
  HAL_I2S_DATA_FORMAT_32_BIT          = LL_I2S_DATA_FORMAT_32_BIT,          /*!< 32-bit on 32-bit channel */
} hal_i2s_data_format_t;

/**
  * @brief HAL I2S audio frequency enumeration definition.
  */
typedef enum
{
  HAL_I2S_MASTER_AUDIO_FREQ_192_KHZ    = LL_I2S_MASTER_AUDIO_FREQ_192_KHZ,   /*!< Frequency 192kHz */
  HAL_I2S_MASTER_AUDIO_FREQ_96_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_96_KHZ,    /*!< Frequency 96kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_48_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_48_KHZ,    /*!< Frequency 48kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_44_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_44_KHZ,    /*!< Frequency 44kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_32_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_32_KHZ,    /*!< Frequency 32kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_22_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_22_KHZ,    /*!< Frequency 22kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_16_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_16_KHZ,    /*!< Frequency 16kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_11_KHZ     = LL_I2S_MASTER_AUDIO_FREQ_11_KHZ,    /*!< Frequency 11kHz  */
  HAL_I2S_MASTER_AUDIO_FREQ_8_KHZ      = LL_I2S_MASTER_AUDIO_FREQ_8_KHZ,     /*!< Frequency 8kHz   */
} hal_i2s_master_audio_frequency_t;

/**
  * @brief HAL I2S clock polarity enumeration definition.
  */
typedef enum
{
  HAL_I2S_CLOCK_POLARITY_LOW  = LL_I2S_CLOCK_POLARITY_LOW,   /*!< SCK signal is at 0 when idle */
  HAL_I2S_CLOCK_POLARITY_HIGH = LL_I2S_CLOCK_POLARITY_HIGH,  /*!< SCK signal is at 1 when idle */
} hal_i2s_clock_polarity_t;

/**
  * @brief HAL I2S MSB/LSB transmission enumeration definition.
  */
typedef enum
{
  HAL_I2S_MSB_FIRST = LL_I2S_MSB_FIRST,  /*!< MSB transmitted first */
  HAL_I2S_LSB_FIRST = LL_I2S_LSB_FIRST,  /*!< LSB transmitted first */
} hal_i2s_bit_order_t;

/**
  * @brief HAL I2S FIFO threshold level enumeration definition.
  */
typedef enum
{
  HAL_I2S_FIFO_THRESHOLD_1_DATA = LL_I2S_FIFO_THRESHOLD_1_DATA,   /*!< Threshold level set to 1 data sample */
  HAL_I2S_FIFO_THRESHOLD_2_DATA = LL_I2S_FIFO_THRESHOLD_2_DATA,   /*!< Threshold level set to 2 data sample */
  HAL_I2S_FIFO_THRESHOLD_3_DATA = LL_I2S_FIFO_THRESHOLD_3_DATA,   /*!< Threshold level set to 3 data sample */
  HAL_I2S_FIFO_THRESHOLD_4_DATA = LL_I2S_FIFO_THRESHOLD_4_DATA,   /*!< Threshold level set to 4 data sample */
  HAL_I2S_FIFO_THRESHOLD_5_DATA = LL_I2S_FIFO_THRESHOLD_5_DATA,   /*!< Threshold level set to 5 data sample */
  HAL_I2S_FIFO_THRESHOLD_6_DATA = LL_I2S_FIFO_THRESHOLD_6_DATA,   /*!< Threshold level set to 6 data sample */
  HAL_I2S_FIFO_THRESHOLD_7_DATA = LL_I2S_FIFO_THRESHOLD_7_DATA,   /*!< Threshold level set to 7 data sample */
  HAL_I2S_FIFO_THRESHOLD_8_DATA = LL_I2S_FIFO_THRESHOLD_8_DATA,   /*!< Threshold level set to 8 data sample */
} hal_i2s_fifo_threshold_t;

/**
  * @brief HAL I2S Master configuration structure definition.
  */
typedef struct
{
  hal_i2s_mode_t                   mode;            /*!< The I2S operating mode.                                     */
  hal_i2s_standard_t               standard;        /*!< The I2S standard.                                           */
  hal_i2s_data_format_t            data_format;     /*!< The I2S data format.                                        */
  hal_i2s_clock_polarity_t         clock_polarity;  /*!< The serial steady state.                                    */
  hal_i2s_bit_order_t              bit_order;       /*!< Specifies whether data transfers start from MSB or LSB bit. */
  hal_i2s_master_audio_frequency_t audio_frequency; /*!< The audio frequency selected, configurable in Master mode.  */
} hal_i2s_master_config_t;

/**
  * @brief HAL I2S Slave configuration structure definition.
  */
typedef struct
{
  hal_i2s_mode_t            mode;             /*!< The I2S operating mode.                                      */
  hal_i2s_standard_t        standard;         /*!< The I2S standard.                                            */
  hal_i2s_data_format_t     data_format;      /*!< The I2S data format.                                         */
  hal_i2s_clock_polarity_t  clock_polarity;   /*!< The serial steady state.                                     */
  hal_i2s_bit_order_t       bit_order;        /*!< Specifies whether data transfers start from MSB or LSB bit.  */
} hal_i2s_slave_config_t;

/**
  * @brief  HAL I2S state structure definition.
  */
typedef enum
{
  HAL_I2S_STATE_RESET         = 0U,           /*!< I2S is not yet initialized or de-initialized        */
  HAL_I2S_STATE_INIT          = (1U << 25U),  /*!< I2S is initialized but not yet configured           */
  HAL_I2S_STATE_IDLE          = (1U << 26U),  /*!< I2S is initialized and global config applied        */
  HAL_I2S_STATE_TX_ACTIVE     = (1U << 27U),  /*!< Data Transmission process is ongoing                */
  HAL_I2S_STATE_RX_ACTIVE     = (1U << 28U),  /*!< Data Reception process is ongoing                   */
  HAL_I2S_STATE_TX_RX_ACTIVE  = (1U << 29U),  /*!< Data Transmission and Reception process is ongoing  */
  HAL_I2S_STATE_PAUSED        = (1U << 30U),  /*!< Data process is paused                              */
  HAL_I2S_STATE_ABORT         = (1U << 31U),  /*!< I2S abort is ongoing                                */
} hal_i2s_state_t;

/*! HAL I2S handler type */
typedef struct hal_i2s_handle_s hal_i2s_handle_t;

/**
  * @brief I2S handle structure definition
  */
struct hal_i2s_handle_s
{
  hal_i2s_t                 instance;                           /*!< I2S instance                       */
  volatile hal_i2s_state_t  global_state;                       /*!< I2S state                          */
  volatile hal_i2s_state_t  pause_state;                        /*!< I2S pause state                    */
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
  volatile uint32_t         last_error_codes;                   /*!< I2S Error code                     */
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t        semaphore;                          /*!< I2S OS semaphore                   */
#endif /* USE_HAL_MUTEX */
  const uint16_t            *p_tx_buff;                         /*!< Pointer to I2S Tx transfer buffer  */
  uint16_t                  tx_xfer_size;                       /*!< I2S Tx transfer size               */
  volatile uint16_t         tx_xfer_count;                      /*!< I2S Tx transfer counter            */
  uint16_t                  *p_rx_buff;                         /*!< Pointer to I2S Rx transfer buffer  */
  uint16_t                  rx_xfer_size;                       /*!< I2S Rx transfer size               */
  volatile uint16_t         rx_xfer_count;                      /*!< I2S Rx transfer counter            */
  void (*p_rx_isr)(struct hal_i2s_handle_s *hi2s);              /*!< Function pointer on Rx ISR         */
  void (*p_tx_isr)(struct hal_i2s_handle_s *hi2s);              /*!< Function pointer on Tx ISR         */
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
  hal_dma_handle_t          *hdma_tx;                           /*!< I2S Tx DMA handle parameters       */
  hal_dma_handle_t          *hdma_rx;                           /*!< I2S Rx DMA handle parameters       */
#endif /* USE_HAL_I2S_DMA */
#if defined (USE_HAL_I2S_USER_DATA) && (USE_HAL_I2S_USER_DATA == 1)
  const void                *p_user_data;                       /*!< User data pointer                  */
#endif /* USE_HAL_I2S_USER_DATA */
#if defined (USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
  void (*p_tx_cplt_cb)(struct hal_i2s_handle_s *hi2s);          /*!< I2S Tx complete callback           */
  void (*p_rx_cplt_cb)(struct hal_i2s_handle_s *hi2s);          /*!< I2S Rx complete callback           */
  void (*p_tx_rx_cplt_cb)(struct hal_i2s_handle_s *hi2s);       /*!< I2S TxRx complete callback         */
  void (*p_tx_half_cplt_cb)(struct hal_i2s_handle_s *hi2s);     /*!< I2S Tx half complete callback      */
  void (*p_rx_half_cplt_cb)(struct hal_i2s_handle_s *hi2s);     /*!< I2S Rx half complete callback      */
  void (*p_tx_rx_half_cplt_cb)(struct hal_i2s_handle_s *hi2s);  /*!< I2S TxRx half complete callback    */
  void (*p_error_cb)(struct hal_i2s_handle_s *hi2s);            /*!< I2S error callback                 */
  void (*p_abort_cplt_cb)(struct hal_i2s_handle_s *hi2s);       /*!< I2S abort callback                 */
#endif  /* USE_HAL_I2S_REGISTER_CALLBACKS */
};

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
/*! HAL I2S Generic I2S callback Type */
typedef void (*hal_i2s_cb_t)(hal_i2s_handle_t *hi2s);  /*!< HAL I2S pointer  */
#endif  /* USE_HAL_I2S_REGISTER_CALLBACKS */

/**
  * @brief  HAL I2S MCLK enumeration definition.
  */
typedef enum
{
  HAL_I2S_MASTER_CLK_OUTPUT_DISABLED = 0U,  /*!< Master clock output disabled */
  HAL_I2S_MASTER_CLK_OUTPUT_ENABLED  = 1U,  /*!< Master clock output enabled  */
} hal_i2s_master_clk_output_status_t;

/**
  * @brief  HAL I2S word select inversion enumeration definition.
  */
typedef enum
{
  HAL_I2S_WS_INVERSION_DISABLED = 0U,  /*!< Word select inversion disabled */
  HAL_I2S_WS_INVERSION_ENABLED  = 1U,  /*!< Word select inversion enabled  */
} hal_i2s_ws_inversion_status_t;

/**
  * @brief  HAL I2S master keep IO state enumeration definition.
  */
typedef enum
{
  HAL_I2S_MASTER_KEEP_IO_STATE_DISABLED = 0U,  /*!< Master keep IO state disabled */
  HAL_I2S_MASTER_KEEP_IO_STATE_ENABLED  = 1U,  /*!< Master keep IO state enabled  */
} hal_i2s_master_keep_io_state_status_t;

/**
  * @brief HAL I2S swap IO state enumeration definition.
  */
typedef enum
{
  HAL_I2S_IO_SWAP_DISABLED = 0U,  /*!< IO swap feature disabled */
  HAL_I2S_IO_SWAP_ENABLED  = 1U,  /*!< IO swap feature enabled  */
} hal_i2s_io_swap_status_t;

/**
  * @brief HAL I2S slave channel length state enumeration definition.
  */
typedef enum
{
  HAL_I2S_SLAVE_LEN_DETECT_ERR_DISABLED = 0U,  /*!< Slave channel length detection error disabled */
  HAL_I2S_SLAVE_LEN_DETECT_ERR_ENABLED  = 1U,  /*!< Slave channel length detection error enabled  */
} hal_i2s_slave_length_detection_error_status_t;

/**
  * @brief HAL I2S IO configuration feature status enumeration definition.
  */
typedef enum
{
  HAL_I2S_IO_CFG_UNLOCKED = 0U,  /*!< IO configuration feature unlocked */
  HAL_I2S_IO_CFG_LOCKED   = 1U,  /*!< IO configuration feature locked   */
} hal_i2s_io_cfg_status_t;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2S_Exported_Constants HAL I2S Constants
  * @{
  */
#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
/** @defgroup I2S_Error I2S Error
  * @{
  */
#define HAL_I2S_ERROR_NONE               (0x00000000UL)  /*!< No error                          */
#define HAL_I2S_ERROR_OVR                (0x00000001UL)  /*!< Overrun error                     */
#define HAL_I2S_ERROR_UDR                (0x00000002UL)  /*!< Underrun error                    */
#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
#define HAL_I2S_ERROR_DMA                (0x00000004UL)  /*!< DMA transfer error                */
#endif /* USE_HAL_I2S_DMA */
#define HAL_I2S_ERROR_FRE                (0x00000008UL)  /*!< Frame format error                */
#define HAL_I2S_ERROR_IO_LOCKED          (0x00000010UL)  /*!< Locked IO error                   */
/**
  * @}
  */
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2S_Exported_Functions HAL I2S Functions
  * @{
  */

/** @defgroup I2S_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_I2S_Init(hal_i2s_handle_t *hi2s, hal_i2s_t instance);
void HAL_I2S_DeInit(hal_i2s_handle_t *hi2s);
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group2 Configuration functions
  * @{
  */
hal_status_t HAL_I2S_MASTER_SetConfig(hal_i2s_handle_t *hi2s, const hal_i2s_master_config_t *p_config);
hal_status_t HAL_I2S_SLAVE_SetConfig(hal_i2s_handle_t *hi2s, const hal_i2s_slave_config_t *p_config);
void HAL_I2S_MASTER_GetConfig(const hal_i2s_handle_t *hi2s, hal_i2s_master_config_t *p_config);
void HAL_I2S_SLAVE_GetConfig(const hal_i2s_handle_t *hi2s, hal_i2s_slave_config_t *p_config);
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group3 Features functions
  * @{
  */
hal_status_t HAL_I2S_MASTER_EnableClockOutput(hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_MASTER_DisableClockOutput(hal_i2s_handle_t *hi2s);
hal_i2s_master_clk_output_status_t HAL_I2S_MASTER_IsEnabledClockOutput(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_EnableWSInversion(const hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_DisableWSInversion(const hal_i2s_handle_t *hi2s);
hal_i2s_ws_inversion_status_t HAL_I2S_IsEnabledWSInversion(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_MASTER_EnableKeepIOState(hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_MASTER_DisableKeepIOState(hal_i2s_handle_t *hi2s);
hal_i2s_master_keep_io_state_status_t HAL_I2S_MASTER_IsEnabledKeepIOState(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_EnableIOSwap(hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_DisableIOSwap(hal_i2s_handle_t *hi2s);
hal_i2s_io_swap_status_t HAL_I2S_IsEnabledIOSwap(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SLAVE_EnableLengthDetectionError(const hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_SLAVE_DisableLengthDetectionError(const hal_i2s_handle_t *hi2s);
hal_i2s_slave_length_detection_error_status_t HAL_I2S_SLAVE_IsEnabledLengthDetectionError(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_LockIOConfig(const hal_i2s_handle_t *hi2s);
hal_i2s_io_cfg_status_t HAL_I2S_IsLockedIOConfig(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetFifoThreshold(const hal_i2s_handle_t *hi2s, const hal_i2s_fifo_threshold_t fifo_threshold);
hal_i2s_fifo_threshold_t HAL_I2S_GetFifoThreshold(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetData24BitsAlignedRight(const hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_SetData24BitsAlignedLeft(const hal_i2s_handle_t *hi2s);
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group4 Items functions
  * @{
  */
hal_status_t HAL_I2S_SetMode(const hal_i2s_handle_t *hi2s, const hal_i2s_mode_t mode);
hal_i2s_mode_t HAL_I2S_GetMode(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetStandard(const hal_i2s_handle_t *hi2s, const hal_i2s_standard_t standard);
hal_i2s_standard_t HAL_I2S_GetStandard(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetDataFormat(const hal_i2s_handle_t *hi2s, const hal_i2s_data_format_t format);
hal_i2s_data_format_t HAL_I2S_GetDataFormat(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_MASTER_SetAudioFrequency(hal_i2s_handle_t *hi2s,
                                              const hal_i2s_master_audio_frequency_t frequency_ws_hz);
hal_i2s_master_audio_frequency_t HAL_I2S_MASTER_GetAudioFrequency(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetClockPolarity(const hal_i2s_handle_t *hi2s, const hal_i2s_clock_polarity_t clock_polarity);
hal_i2s_clock_polarity_t HAL_I2S_GetClockPolarity(const hal_i2s_handle_t *hi2s);

hal_status_t HAL_I2S_SetBitOrder(hal_i2s_handle_t *hi2s, const hal_i2s_bit_order_t bit_order);
hal_i2s_bit_order_t HAL_I2S_GetBitOrder(const hal_i2s_handle_t *hi2s);

#if defined (USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
hal_status_t HAL_I2S_SetTxDMA(hal_i2s_handle_t *hi2s, hal_dma_handle_t *hdma);
hal_status_t HAL_I2S_SetRxDMA(hal_i2s_handle_t *hi2s, hal_dma_handle_t *hdma);
#endif /* USE_HAL_I2S_DMA */
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group5 IO operation functions
  * @{
  */
/* Real audio frequency */
uint32_t HAL_I2S_MASTER_GetRealAudioFrequency(const hal_i2s_handle_t *hi2s);

/* Blocking mode: Polling */
hal_status_t HAL_I2S_Transmit(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample,
                              uint32_t timeout_ms);
hal_status_t HAL_I2S_Receive(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample, uint32_t timeout_ms);
hal_status_t HAL_I2S_TransmitReceive(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                     uint32_t size_sample, uint32_t timeout_ms);

/* Non-Blocking mode: Interrupt */
hal_status_t HAL_I2S_Transmit_IT(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample);
hal_status_t HAL_I2S_Receive_IT(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample);
hal_status_t HAL_I2S_TransmitReceive_IT(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                        uint32_t size_sample);

#if defined(USE_HAL_I2S_DMA) && (USE_HAL_I2S_DMA == 1)
/* Non-Blocking mode: DMA */
hal_status_t HAL_I2S_Transmit_DMA(hal_i2s_handle_t *hi2s, const void *p_data, uint32_t size_sample);
hal_status_t HAL_I2S_Receive_DMA(hal_i2s_handle_t *hi2s, void *p_data, uint32_t size_sample);
hal_status_t HAL_I2S_TransmitReceive_DMA(hal_i2s_handle_t *hi2s, const void *p_tx_data, void *p_rx_data,
                                         uint32_t size_sample);
#endif /* USE_HAL_I2S_DMA */

/* Pause resume functions */
hal_status_t HAL_I2S_MASTER_Pause(hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_MASTER_Resume(hal_i2s_handle_t *hi2s);

/* Abort functions */
hal_status_t HAL_I2S_Abort(hal_i2s_handle_t *hi2s);
hal_status_t HAL_I2S_Abort_IT(hal_i2s_handle_t *hi2s);
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group6 IRQ handler/callbacks/register callbacks functions
  * @{
  */
void HAL_I2S_IRQHandler(hal_i2s_handle_t *hi2s);
void HAL_I2S_ErrorCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_TxCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_RxCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_TxRxCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_TxHalfCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_RxHalfCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_TxRxHalfCpltCallback(hal_i2s_handle_t *hi2s);
void HAL_I2S_AbortCpltCallback(hal_i2s_handle_t *hi2s);

#if defined(USE_HAL_I2S_REGISTER_CALLBACKS) && (USE_HAL_I2S_REGISTER_CALLBACKS == 1)
hal_status_t HAL_I2S_RegisterErrorCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterTxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterRxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterTxRxCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterTxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterRxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterTxRxHalfCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
hal_status_t HAL_I2S_RegisterAbortCpltCallback(hal_i2s_handle_t *hi2s, hal_i2s_cb_t p_callback);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group7 Peripheral current frequency, state and errors functions
  * @{
  */
uint32_t HAL_I2S_GetClockFreq(const hal_i2s_handle_t *hi2s);

hal_i2s_state_t HAL_I2S_GetState(const hal_i2s_handle_t *hi2s);

#if defined(USE_HAL_I2S_GET_LAST_ERRORS) && (USE_HAL_I2S_GET_LAST_ERRORS == 1)
uint32_t HAL_I2S_GetLastErrorCodes(const hal_i2s_handle_t *hi2s);
#endif /* USE_HAL_I2S_GET_LAST_ERRORS */
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup I2S_Exported_Functions_Group8 Acquire/release Bus functions
  * @{
  */
hal_status_t HAL_I2S_AcquireBus(hal_i2s_handle_t *hi2s, uint32_t timeout_ms);
hal_status_t HAL_I2S_ReleaseBus(hal_i2s_handle_t *hi2s);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_I2S_USER_DATA) && (USE_HAL_I2S_USER_DATA == 1)
/** @defgroup I2S_Exported_Functions_Group9 Set/Get user data
  * @{
  */
void HAL_I2S_SetUserData(hal_i2s_handle_t *hi2s, const void *p_user_data);
const void *HAL_I2S_GetUserData(const hal_i2s_handle_t *hi2s);
/**
  * @}
  */
#endif /* USE_HAL_I2S_USER_DATA */
/**
  * @}
  */

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* STM32C5xx_HAL_I2S_H */
