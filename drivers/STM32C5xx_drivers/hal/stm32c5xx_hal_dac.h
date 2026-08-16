/**
  ******************************************************************************
  * @file    stm32c5xx_hal_dac.h
  * @brief   Header file for the DAC HAL module.
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
#ifndef STM32C5XX_HAL_DAC_H
#define STM32C5XX_HAL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_dac.h"

/** @addtogroup DAC
  * @{
  */

#if defined(DAC1)

/* Exported types ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Types HAL DAC Types
  * @{
  */

/** @defgroup DAC_Exported_Types_Group1 Enumerations
  * @{
  */

/**
  * @brief  HAL DAC instance
  */
typedef enum
{
  HAL_DAC1 = DAC1_BASE, /*!< DAC1 */
} hal_dac_t;

/**
  * @brief  HAL DAC channels.
  */
typedef enum
{
  HAL_DAC_CHANNEL_1  = 0U, /*!< DAC channel 1 */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  HAL_DAC_CHANNEL_2  = 1U  /*!< DAC channel 2 */
#endif /* DAC_NB_OF_CHANNEL */
} hal_dac_channel_t;

/**
  * @brief  HAL DAC state definition
  */
typedef enum
{
  HAL_DAC_STATE_RESET                          = 0U,           /*!< DAC not yet initialized or is de-initialized */
  HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED    = (1U << 31U),  /*!< DAC is initialized
                                                                    and a global configuration has been applied,
                                                                    the channels are used separately */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
  HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED        = (1U << 30U),  /*!< DAC is configured in dual channel mode */
  HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE            = (1U << 28U)   /*!< DAC is Active in dual channel mode */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
} hal_dac_state_t;

/**
  * @brief  HAL DAC channel state definition
  */
typedef enum
{
  HAL_DAC_CHANNEL_STATE_RESET         = 0U,          /*!< DAC CHANNEL not yet initialized or disabled  */
  HAL_DAC_CHANNEL_STATE_IDLE          = (1U << 31U), /*!< DAC CHANNEL is initialized and a channel configuration
                                                          has been applied */
  HAL_DAC_CHANNEL_STATE_ACTIVE        = (1U << 30U), /*!< DAC CHANNEL is active, conversion is running */
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
  HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT = (1U << 29U)  /*!< DAC CHANNEL is active, conversion is running,
                                                          using DMA in silent mode */
#endif /* USE_HAL_DAC_DMA  */
} hal_dac_channel_state_t;

/**
  * @brief  HAL DAC channel trigger
  */
typedef enum
{
  HAL_DAC_TRIGGER_NONE                = 0xFFFFFFFFUL,                       /*!< Conversion is automatic once the DAC_DHRxxxx register*/
  HAL_DAC_TRIGGER_SOFTWARE            = LL_DAC_TRIGGER_SOFTWARE,            /*!< conversion started by software trigger for DAC channel */
  HAL_DAC_TRIGGER_TIM1_TRGO           = LL_DAC_TRIGGER_TIM1_TRGO,           /*!< TIM1 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM2_TRGO           = LL_DAC_TRIGGER_TIM2_TRGO,           /*!< TIM2 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM5_TRGO           = LL_DAC_TRIGGER_TIM5_TRGO,           /*!< TIM5 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM6_TRGO           = LL_DAC_TRIGGER_TIM6_TRGO,           /*!< TIM6 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM7_TRGO           = LL_DAC_TRIGGER_TIM7_TRGO,           /*!< TIM7 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM8_TRGO           = LL_DAC_TRIGGER_TIM8_TRGO,           /*!< TIM8 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM12_TRGO          = LL_DAC_TRIGGER_TIM12_TRGO,          /*!< TIM12 TRGO selected as external conversion trigger for DAC channel. */
  HAL_DAC_TRIGGER_TIM15_TRGO          = LL_DAC_TRIGGER_TIM15_TRGO,          /*!< TIM15 TRGO selected as external conversion trigger for DAC channel. */
#if defined(LPTIM1)
  HAL_DAC_TRIGGER_LPTIM1_OC1          = LL_DAC_TRIGGER_LPTIM1_OC1,          /*!< LPTIM1 CH1 selected as external conversion trigger for DAC channel. */
#endif /* LPTIM1 */
  HAL_DAC_TRIGGER_EXTI9               = LL_DAC_TRIGGER_EXTI9,               /*!< EXTI Line9 event selected as external conversion trigger for DAC channel. */
} hal_dac_trigger_t;

/**
  * @brief DAC channel output buffer mode
  */
typedef enum
{
  HAL_DAC_OUTPUT_BUFFER_ENABLED  = LL_DAC_OUTPUT_BUFFER_ENABLE,  /*!< The output is buffered:
                                                                      higher drive current capability,
                                                                      but also higher current consumption */
  HAL_DAC_OUTPUT_BUFFER_DISABLED = LL_DAC_OUTPUT_BUFFER_DISABLE, /*!< The output is not buffered:
                                                                      lower drive current capability,
                                                                      but also lower current consumption */
} hal_dac_output_buffer_status_t;

/**
  * @brief   DAC channel data alignment
  */
typedef enum
{
  HAL_DAC_DATA_ALIGN_12_BITS_RIGHT = 0x00000000U, /*!< Data must be written in 12-bit right alignment */
  HAL_DAC_DATA_ALIGN_12_BITS_LEFT  = 0x00000001U, /*!< Data must be written in 12-bit left  alignment */
  HAL_DAC_DATA_ALIGN_8_BITS_RIGHT  = 0x00000002U  /*!< Data must be written in  8-bit right alignment */
} hal_dac_data_alignment_t;

/**
  * @brief DAC channel output connection.
  * @note  With some configuration of mode and buffer, there are both internal and external connections.
  *        This applies regardless of this hal_dac_output_connection_t value.
  */
typedef enum
{
  HAL_DAC_OUTPUT_CONNECTION_EXTERNAL = LL_DAC_OUTPUT_CONNECT_EXTERNAL,    /*!< DAC channel output is connected to external
                                        pin.
                                        Note: Depending on other parameters (mode normal or sample and hold,
                                              output buffer state), output can also be connected to on-chip
                                              peripherals, refer to ref manual. */
  HAL_DAC_OUTPUT_CONNECTION_INTERNAL = LL_DAC_OUTPUT_CONNECT_INTERNAL /*!< DAC channel output is connected to on-chip
                                        peripherals (via internal paths).
                                        Note: Depending on other parameters (mode normal or sample and hold,
                                              output buffer state), output can also be connected to external pin,
                                              refer to ref manual. */
} hal_dac_output_connection_t ;

/**
  * @brief DAC channel sample and hold mode
  */
typedef enum
{
  HAL_DAC_SAMPLE_AND_HOLD_DISABLED = LL_DAC_OUTPUT_MODE_NORMAL,         /*!< The output is in normal mode */
  HAL_DAC_SAMPLE_AND_HOLD_ENABLED  = LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD /*!< The output is in sample-and-hold mode.
                                                                             Note: the sample-and-hold mode requires
                                                                             an external capacitor */
} hal_dac_sample_and_hold_status_t;

/**
  * @brief  DAC high frequency interface mode
  */
typedef enum
{
  HAL_DAC_HIGH_FREQ_MODE_DISABLED     = LL_DAC_HIGH_FREQ_MODE_DISABLED,    /*!< High frequency interface mode disabled*/
  HAL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ  = LL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ, /*!< High frequency interface
                                                                                mode compatible to AHB>80MHz enabled */
  HAL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ = LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ /*!< High frequency interface
                                                                                mode compatible to AHB>160MHz enabled */
} hal_dac_high_freq_mode_t;

/**
  * @brief  DAC channel signed or unsigned data format
  */
typedef enum
{
  HAL_DAC_SIGN_FORMAT_UNSIGNED = LL_DAC_SIGN_FORMAT_UNSIGNED, /*!< The data format is not signed */
  HAL_DAC_SIGN_FORMAT_SIGNED   = LL_DAC_SIGN_FORMAT_SIGNED    /*!< The data format is signed */
} hal_dac_sign_format_t;

/**
  * @brief  DAC channel DMA double data mode
  */
typedef enum
{
  HAL_DAC_DMA_DOUBLE_DATA_MODE_DISABLED = 0U,  /*!< The DMA data mode is the single data mode */
  HAL_DAC_DMA_DOUBLE_DATA_MODE_ENABLED  = 1U   /*!< The DMA data mode is the double data mode */
} hal_dac_dma_double_data_mode_status_t;

/**
  * @brief  HAL DAC channel triangle wave and pseudo noise amplitude
  */

typedef enum
{
  HAL_DAC_WAVE_AMPLITUDE_1    = LL_DAC_TRIANGLE_AMPLITUDE_1,    /*!< Noise/triangle amplitude equal to 1 */
  HAL_DAC_WAVE_AMPLITUDE_3    = LL_DAC_TRIANGLE_AMPLITUDE_3,    /*!< Noise/triangle amplitude equal to 3 */
  HAL_DAC_WAVE_AMPLITUDE_7    = LL_DAC_TRIANGLE_AMPLITUDE_7,    /*!< Noise/triangle amplitude equal to 7 */
  HAL_DAC_WAVE_AMPLITUDE_15   = LL_DAC_TRIANGLE_AMPLITUDE_15,   /*!< Noise/triangle amplitude equal to 15 */
  HAL_DAC_WAVE_AMPLITUDE_31   = LL_DAC_TRIANGLE_AMPLITUDE_31,   /*!< Noise/triangle amplitude equal to 31 */
  HAL_DAC_WAVE_AMPLITUDE_63   = LL_DAC_TRIANGLE_AMPLITUDE_63,   /*!< Noise/triangle amplitude equal to 63 */
  HAL_DAC_WAVE_AMPLITUDE_127  = LL_DAC_TRIANGLE_AMPLITUDE_127,  /*!< Noise/triangle amplitude equal to 127 */
  HAL_DAC_WAVE_AMPLITUDE_255  = LL_DAC_TRIANGLE_AMPLITUDE_255,  /*!< Noise/triangle amplitude equal to 255 */
  HAL_DAC_WAVE_AMPLITUDE_511  = LL_DAC_TRIANGLE_AMPLITUDE_511,  /*!< Noise/triangle amplitude equal to 511 */
  HAL_DAC_WAVE_AMPLITUDE_1023 = LL_DAC_TRIANGLE_AMPLITUDE_1023, /*!< Noise/triangle amplitude equal to 1023 */
  HAL_DAC_WAVE_AMPLITUDE_2047 = LL_DAC_TRIANGLE_AMPLITUDE_2047, /*!< Noise/triangle amplitude equal to 2047  */
  HAL_DAC_WAVE_AMPLITUDE_4095 = LL_DAC_TRIANGLE_AMPLITUDE_4095, /*!< Noise/triangle amplitude equal to 4095  */
} hal_dac_wave_amplitude_t;


/**
  * @}
  */
/* Exported Constants --------------------------------------------------------*/
/** @defgroup DAC_Exported_Constants HAL DAC Constants
  * @{
  */
/**
  * @brief  HAL DAC error code,
  *         declared and used has bit fields in HAL_DAC_GetLastErrorCodes()
  */
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
#define HAL_DAC_ERROR_NONE              (0UL)      /*!< No error */
#define HAL_DAC_ERROR_DMA_UNDERRUN_CH1  (1UL << 0U) /*!< DMA underrun error on channel 1 */
#define HAL_DAC_ERROR_DMA_CH1           (1UL << 1U) /*!< DMA transfer error on channel 1 */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define HAL_DAC_ERROR_DMA_UNDERRUN_CH2  (HAL_DAC_ERROR_DMA_UNDERRUN_CH1 << 8U) /*!< DMA underrun error on channel 2 */
#define HAL_DAC_ERROR_DMA_CH2           (HAL_DAC_ERROR_DMA_CH1 << 8U)          /*!< DMA transfer error on channel 2 */
#endif /* DAC_NB_OF_CHANNEL */

#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief  HAL DAC optional interrupts
  */
#define HAL_DAC_OPT_DMA_IT_NONE     HAL_DMA_OPT_IT_NONE     /*!< All optional interrupts are disabled */
#define HAL_DAC_OPT_DMA_IT_HT       HAL_DMA_OPT_IT_HT       /*!< Enable optional IT half completed transfer */
#define HAL_DAC_OPT_DMA_IT_DEFAULT  HAL_DMA_OPT_IT_DEFAULT  /*!< Enable all optional IT */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define HAL_DAC_OPT_DMA_IT_SILENT   HAL_DMA_OPT_IT_SILENT   /*!< DMA in silent mode */
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /*  USE_HAL_DAC_DMA */

/**
  * @}
  */

/** @defgroup DAC_Exported_Types_Group2 Handle Structure
  * @{
  */
/**
  * @brief DAC Handle structure Definition
  *
  */
/*!< DAC handle structure definition */
typedef struct hal_dac_handle_s hal_dac_handle_t; /*!< DAC handle type definition */
/**
  * @}
  */

/** @defgroup DAC_Exported_Types_Group_Callback callback prototype
  * @{
  */
/* DAC callback prototype and error callback prototype */
typedef void (* hal_dac_cb_t)(hal_dac_handle_t *hdac, hal_dac_channel_t channel); /*!< Callback prototype
                                                                                       for converter completed */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
typedef void (* hal_dac_dual_channel_cb_t)(hal_dac_handle_t *hdac); /*!< Callback prototype
                                                                         for dual channel converter completed */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
typedef void (* hal_dac_error_cb_t)(hal_dac_handle_t *hdac);        /*!< Callback prototype for error callback */
/**
  * @}
  */

/** @addtogroup DAC_Exported_Types_Group2 Handle Structure
  * @{
  */

/**
  * @brief  DAC handle structure definition,
  *         contains: DAC instance, states, callbacks, DMA handles linked with DAC channels.
  */
struct hal_dac_handle_s
{
  hal_dac_t                         instance;      /*!< Peripheral instance */
  volatile hal_dac_state_t          global_state;  /*!< DAC global state */
  volatile hal_dac_channel_state_t  channel_state[DAC_NB_OF_CHANNEL]; /*!< State for channels subinstances,
                                                                           they can be active in parallel */
  volatile uint32_t                *channel_dhr_address[DAC_NB_OF_CHANNEL]; /*!< DHR (data holding register) address
                                                                                 according to the alignment */

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  volatile uint16_t                 last_error_codes[DAC_NB_OF_CHANNEL]; /*!< DAC channel errors codes
                                                    array of uint16_t to avoid race condition between the channels */
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
  hal_dma_handle_t                 *dma_ch[DAC_NB_OF_CHANNEL];   /*!< Pointer to a DMA handle
                                                                      (used by DAC channels or by dual channels) */
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
  hal_dac_channel_t                 dual_channel_dma_requester;  /*!< Dual mode DMA channel requester */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* USE_HAL_DAC_DMA  */

#if defined (USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hal_dac_cb_t                 p_conv_cplt_cb;      /*!< Converter completed callback */
  hal_dac_cb_t                 p_conv_half_cplt_cb; /*!< Converter half completed callback */
  hal_dac_cb_t                 p_stop_cplt_cb;      /*!< Stop completed callback */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
  hal_dac_dual_channel_cb_t    p_dual_channel_conv_cplt_cb;      /*!< Dual channel converter completed callback */
  hal_dac_dual_channel_cb_t    p_dual_channel_conv_half_cplt_cb; /*!< Dual channel converter half completed callback */
  hal_dac_dual_channel_cb_t    p_dual_channel_stop_cplt_cb;      /*!< Dual channel stop completed callback */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
  hal_dac_error_cb_t           p_error_cb;          /*!< Converter error callback */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

#if defined(USE_HAL_DAC_USER_DATA) && (USE_HAL_DAC_USER_DATA == 1)
  const void                  *p_user_data; /*!< User Data Pointer */
#endif /* USE_HAL_DAC_USER_DATA */
};

/**
  * @}
  */

/** @defgroup DAC_Exported_Types_Group4 Configuration Structure
  * @{
  */

/**
  * @brief   DAC configuration "sample and hold" structure definition
  */
typedef struct
{
  uint32_t sample_time_cycle;/*!< The sample time for the channel, unit is in number of clock period.
                                  This parameter must be a number the range [0, 1023].
                                  The sample_time_cycle is applied when the sample_and_hold mode is
                                  HAL_DAC_SAMPLE_AND_HOLD_ENABLED */

  uint32_t hold_time_cycle;  /*!< The hold time for the channel, unit is in number of clock period.
                                  This parameter must be a number the range [0, 1023].
                                  The hold_time is applied when the sample_and_hold mode is
                                  HAL_DAC_SAMPLE_AND_HOLD_ENABLED */

  uint32_t refresh_time_cycle; /*!< The refresh time for the channel, unit is in number of clock period.
                                    This parameter must be a number the range [0, 255].
                                    The refresh_time is applied when the sample_and_hold mode is
                                    HAL_DAC_SAMPLE_AND_HOLD_ENABLED */
} hal_dac_channel_sample_and_hold_config_t;

/**
  * @brief   DAC configuration structure definition
  */
typedef struct
{
  hal_dac_high_freq_mode_t high_frequency_mode;  /*!< The frequency interface mode
                                                      Note: HAL_DAC_GetOptimumFrequencyMode() API allows
                                                      to select and update the high frequency mode afterwards */
} hal_dac_config_t;

/**
  * @brief   DAC channel configuration structure definition
  */
typedef struct
{
  hal_dac_sign_format_t           data_sign_format;  /*!< The data format: signed data or unsigned data */
  hal_dac_trigger_t               trigger;           /*!< The external trigger for the channel */
  hal_dac_output_buffer_status_t  output_buffer;     /*!< The DAC channel output buffer: enabled or disabled */
  hal_dac_output_connection_t     output_connection; /*!< The DAC channel output connection:
                                                          to external pin or to on chip peripheral */
  hal_dac_data_alignment_t        alignment;         /*!< Default alignment and width, for both channel:
                                                          12bit right or left align, 8bit right align */
} hal_dac_channel_config_t;

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief   DAC dual channel configuration structure definition
  */
typedef struct
{
  struct
  {
    hal_dac_sign_format_t           data_sign_format;  /*!< The data format: signed data or unsigned data */
    hal_dac_trigger_t               trigger;           /*!< The external trigger for the channel */
    hal_dac_output_buffer_status_t  output_buffer;     /*!< The DAC channel output buffer: enabled or disabled */
    hal_dac_output_connection_t     output_connection; /*!< The DAC channel output connection:
                                                          to external pin or to on chip peripheral */
  } channel1_config; /*!< Dual channel, sub config for channel 1, see fields description above */

  struct
  {
    hal_dac_sign_format_t           data_sign_format;  /*!< The data format: signed data or unsigned data */
    hal_dac_trigger_t               trigger;           /*!< The external trigger for the channel */
    hal_dac_output_buffer_status_t  output_buffer;     /*!< The DAC channel output buffer: enabled or disabled */
    hal_dac_output_connection_t     output_connection; /*!< The DAC channel output connection:
                                                          to external pin or to on chip peripheral */
  } channel2_config; /*!< Dual channel, sub config for channel 2, see fields description above */

  hal_dac_data_alignment_t          alignment;        /*!< Alignment and width, for dual channel:
                                                           12bit right or left align, 8bit right align */
} hal_dac_dual_channel_config_t;
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1_1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
hal_status_t HAL_DAC_Init(hal_dac_handle_t *hdac, hal_dac_t instance);
void         HAL_DAC_DeInit(hal_dac_handle_t *hdac);
/* Peripheral configuration */
hal_dac_high_freq_mode_t HAL_DAC_GetOptimumFrequencyMode(const hal_dac_handle_t *hdac);
hal_status_t HAL_DAC_SetConfig(hal_dac_handle_t *hdac, const hal_dac_config_t *p_config);
void         HAL_DAC_GetConfig(const hal_dac_handle_t *hdac, hal_dac_config_t *p_config);
void         HAL_DAC_ResetConfig(hal_dac_handle_t *hdac);
/* Calibration functions */
hal_status_t HAL_DAC_CalibrateChannelBuffer(hal_dac_handle_t *hdac, hal_dac_channel_t channel);
hal_status_t HAL_DAC_SetChannelBufferCalibrationValue(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                      uint32_t value);
uint32_t     HAL_DAC_GetChannelBufferCalibrationValue(const hal_dac_handle_t *hdac, hal_dac_channel_t channel);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group1_2
  * @{
  */
/* Peripheral channel configuration */
hal_status_t HAL_DAC_SetConfigChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                      const hal_dac_channel_config_t *p_config);

void         HAL_DAC_GetConfigChannel(const hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                      hal_dac_channel_config_t *p_config);

hal_status_t HAL_DAC_SetChannelAlignment(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                         hal_dac_data_alignment_t        alignment);

hal_dac_data_alignment_t HAL_DAC_GetChannelAlignment(const hal_dac_handle_t *hdac, hal_dac_channel_t channel);

/**
  * @}
  */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/** @addtogroup DAC_Exported_Functions_Group1_3
  * @{
  */
hal_status_t HAL_DAC_SetConfigDualChannel(hal_dac_handle_t *hdac, const hal_dac_dual_channel_config_t *p_config);
void         HAL_DAC_GetConfigDualChannel(const hal_dac_handle_t *hdac, hal_dac_dual_channel_config_t *p_config);
hal_status_t HAL_DAC_SetDualChannelAlignment(hal_dac_handle_t *hdac,  hal_dac_data_alignment_t alignment);
hal_dac_data_alignment_t HAL_DAC_GetDualChannelAlignment(const hal_dac_handle_t *hdac);
/**
  * @}
  */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
/* Input and output operation functions *****************************************************/
/** @addtogroup DAC_Exported_Functions_Group2_1
  * @{
  */

hal_status_t HAL_DAC_TrigSWConversionChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_StartChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_StopChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_SetChannelData(hal_dac_handle_t *hdac, hal_dac_channel_t channel, uint32_t data);

uint32_t     HAL_DAC_GetChannelData(const hal_dac_handle_t *hdac, hal_dac_channel_t channel);

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/* DMA separate channel functions to link DAC channel with DMA */
hal_status_t HAL_DAC_SetChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma, hal_dac_channel_t channel);

hal_status_t HAL_DAC_StartChannel_DMA(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                      uint32_t size_byte);

hal_status_t HAL_DAC_StartChannel_DMA_Opt(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                          uint32_t size_byte, uint32_t dac_opt_interrupt);

hal_status_t HAL_DAC_StopChannel_DMA(hal_dac_handle_t *hdac, hal_dac_channel_t channel);
#endif /* USE_HAL_DAC_DMA */

/**
  * @}
  */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/** @addtogroup DAC_Exported_Functions_Group2_2
  * @{
  */

#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
hal_status_t HAL_DAC_TrigSWConversionDualChannel(hal_dac_handle_t *hdac);
hal_status_t HAL_DAC_StartDualChannel(hal_dac_handle_t *hdac);
hal_status_t HAL_DAC_StopDualChannel(hal_dac_handle_t *hdac);

hal_status_t HAL_DAC_SetDualChannelData(hal_dac_handle_t *hdac, uint32_t data);
uint32_t     HAL_DAC_GetDualChannelData(const hal_dac_handle_t *hdac);

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/* DMA dual channel functions to link DAC channel with DMA */
hal_status_t HAL_DAC_SetDualChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma,
                                       hal_dac_channel_t dma_requester_channel);

hal_status_t HAL_DAC_StartDualChannel_DMA(hal_dac_handle_t *hdac, const void *p_data, uint32_t size_byte);
hal_status_t HAL_DAC_StartDualChannel_DMA_Opt(hal_dac_handle_t *hdac, const void *p_data, uint32_t size_byte,
                                              uint32_t dac_opt_interrupt);

hal_status_t HAL_DAC_StopDualChannel_DMA(hal_dac_handle_t *hdac);
#endif /* USE_HAL_DAC_DMA */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

/**
  * @}
  */
#endif /* DAC_NB_OF_CHANNEL */

/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */

/* Peripheral control functions */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/* DMA double data mode functions  */
hal_status_t HAL_DAC_EnableChannelDMADoubleDataMode(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_DisableChannelDMADoubleDataMode(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_dac_dma_double_data_mode_status_t HAL_DAC_IsEnabledChannelDMADoubleDataMode(hal_dac_handle_t *hdac,
                                                                                hal_dac_channel_t channel);
#endif /* USE_HAL_DAC_DMA */

/* Peripheral channel configuration and control functions */
hal_status_t HAL_DAC_EnableChannelAddingTriangleWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                     hal_dac_wave_amplitude_t amplitude);

hal_status_t HAL_DAC_DisableChannelAddingTriangleWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_EnableChannelAddingNoiseWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                  hal_dac_wave_amplitude_t amplitude);

hal_status_t HAL_DAC_DisableChannelAddingNoiseWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

/* Sample and hold functions  */
hal_status_t HAL_DAC_SetConfigChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                   const hal_dac_channel_sample_and_hold_config_t *p_config);

void HAL_DAC_GetConfigChannelSampleAndHold(const hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                           hal_dac_channel_sample_and_hold_config_t *p_config);

hal_status_t HAL_DAC_EnableChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_status_t HAL_DAC_DisableChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

hal_dac_sample_and_hold_status_t HAL_DAC_IsEnabledChannelSampleAndHold(hal_dac_handle_t *hdac,
                                                                       hal_dac_channel_t channel);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Callbacks functions *****************************/
/* DAC callback */
void HAL_DAC_ConvCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel);
void HAL_DAC_ConvHalfCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel);
void HAL_DAC_StopCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel);
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
void HAL_DAC_DualChannelConvCpltCallback(hal_dac_handle_t *hdac);
void HAL_DAC_DualChannelConvHalfCpltCallback(hal_dac_handle_t *hdac);
void HAL_DAC_DualChannelStopCpltCallback(hal_dac_handle_t *hdac);
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
void HAL_DAC_ErrorCallback(hal_dac_handle_t *hdac);

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering */
hal_status_t HAL_DAC_RegisterConvCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback);
hal_status_t HAL_DAC_RegisterConvHalfCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback);
hal_status_t HAL_DAC_RegisterStopCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback);
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
hal_status_t HAL_DAC_RegisterDualChannelCpltCallback(hal_dac_handle_t *hdac, hal_dac_dual_channel_cb_t p_callback);
hal_status_t HAL_DAC_RegisterDualChannelHalfCpltCallback(hal_dac_handle_t *hdac,
                                                         hal_dac_dual_channel_cb_t p_callback);
hal_status_t HAL_DAC_RegisterDualChannelStopCpltCallback(hal_dac_handle_t *hdac, hal_dac_dual_channel_cb_t p_callback);
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
hal_status_t HAL_DAC_RegisterErrorCallback(hal_dac_handle_t *hdac, hal_dac_error_cb_t p_callback);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group5
  * @{
  */
/* Peripheral State, kernel clock frequency, and Error functions ***************************************/
hal_dac_state_t         HAL_DAC_GetState(const hal_dac_handle_t *hdac);

hal_dac_channel_state_t HAL_DAC_GetChannelState(const hal_dac_handle_t *hdac, hal_dac_channel_t channel);

uint32_t HAL_DAC_GetClockFreq(const hal_dac_handle_t *hdac);

void HAL_DAC_IRQHandler(hal_dac_handle_t *hdac);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
uint32_t                HAL_DAC_GetLastErrorCodes(const hal_dac_handle_t *hdac);
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group6
  * @{
  */
/* User Data API functions ***************************************/
#if defined(USE_HAL_DAC_USER_DATA) && (USE_HAL_DAC_USER_DATA == 1)
void HAL_DAC_SetUserData(hal_dac_handle_t *hdac, const void *p_user_data);

const void *HAL_DAC_GetUserData(const hal_dac_handle_t *hdac);
#endif /* USE_HAL_DAC_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAC1 || DAC2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_DAC_H */
