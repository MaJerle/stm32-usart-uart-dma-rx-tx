/**
  ******************************************************************************
  * @file    stm32c5xx_hal_opamp.h
  * @brief   Header file of OPAMP HAL module.
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
#ifndef STM32C5XX_HAL_OPAMP_H
#define STM32C5XX_HAL_OPAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_opamp.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (OPAMP1) || defined (OPAMP2) || defined(OPAMP3)

/** @defgroup OPAMP OPAMP
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup OPAMP_Exported_Types HAL OPAMP Types
  * @{
  */

/** @defgroup OPAMP_Exported_Types_Group1 Enumerations
  * @{
  */

/**
  * @brief  HAL OPAMP instance
  */
typedef enum
{
  HAL_OPAMP1 = OPAMP1_BASE,
#if defined (OPAMP2)
  HAL_OPAMP2 = OPAMP2_BASE,
#endif /* OPAMP2 */
#if defined (OPAMP3)
  HAL_OPAMP3 = OPAMP3_BASE,
#endif /* OPAMP3 */
} hal_opamp_t;

/**
  * @brief  HAL OPAMP state definition
  */
typedef enum
{
  HAL_OPAMP_STATE_RESET     = 0U,           /*!< OPAMP not yet initialized or is de-initialized */
  HAL_OPAMP_STATE_IDLE      = (1U << 31U),  /*!< OPAMP is initialized */
  HAL_OPAMP_STATE_CALIB     = (1U << 30U),  /*!< OPAMP is being calibrated */
  HAL_OPAMP_STATE_ACTIVE    = (1U << 29U),  /*!< OPAMP is active */
} hal_opamp_state_t;

/** @brief  OPAMP configuration mode
  */
typedef enum
{
  HAL_OPAMP_MODE_STANDALONE             = LL_OPAMP_MODE_STANDALONE, /*!< Standalone mode */
  HAL_OPAMP_MODE_PGA                    = LL_OPAMP_MODE_PGA,        /*!< PGA mode        */
  HAL_OPAMP_MODE_FOLLOWER               = LL_OPAMP_MODE_FOLLOWER    /*!< Follower mode   */
} hal_opamp_config_mode_t;

/**
  * @brief OPAMP PGA external connection mode
  */
typedef enum
{
  HAL_OPAMP_PGA_EXT_NONE                = LL_OPAMP_PGA_EXT_NONE, /*!< In PGA mode, the inverting input is connected
                                        to the internal feedback resistor. No external connection on inverting input.
                                                                   Positive gain (Non-Inverting mode)
                                                                   - VINPx: Input signal */
  HAL_OPAMP_PGA_EXT_FILT                = LL_OPAMP_PGA_EXT_FILT, /*!< In PGA mode, the inverting input is connected to the
                                        internal feedback resistor and to VINM0 for filtering.
                                                                   Positive gain (Non-Inverting mode)
                                                                   - VINPx: Input signal
                                                                   - VINM0 - VOUT: Additional low-pass filtering */
  HAL_OPAMP_PGA_EXT_BIAS                = LL_OPAMP_PGA_EXT_BIAS, /*!< In PGA mode, the inverting input is connected
                                        to the internal feedback resistor. VINM0 is available for bias voltage in
                                        non-inverting mode or to use theOPAMP in inverting mode.
                                        Negative gain (Inverting mode):
                                        - VINPx: Bias voltage
                                        - VINM0: Input signal
                                        Positive gain (Non-Inverting mode)
                                        - VINPx: Input signal
                                        - VINM0: Bias voltage */
  HAL_OPAMP_PGA_EXT_BIAS_FILT           = LL_OPAMP_PGA_EXT_BIAS_FILT /*!< In PGA mode, the inverting input is connected
                                        to the internal feedback resistor and to VINM1 for filtering.
                                        VINM0 is available for bias voltage in non-inverting mode or to use the OPAMP in
                                        inverting mode.
                                        Negative gain (Inverting mode):
                                        - VINPx: Bias voltage
                                        - VINM0: Input signal
                                        - VINM1 - VOUT: Additional low-pass filtering
                                        Positive gain (Non-Inverting mode)
                                        - VINPx: Input signal
                                        - VINM0: Bias voltage
                                        - VINM1 - VOUT: Additional low-pass filtering */
} hal_opamp_pga_external_t;

/** @brief  OPAMP Non-inverting Input
  */
typedef enum
{
  HAL_OPAMP_NON_INVERTING_INPUT_GPIO_0    = LL_OPAMP_INPUT_NONINVERT_IO0, /*!< OPAMP non-inverting input 0 connected to
                                                 GPIO (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP0")*/
  HAL_OPAMP_NON_INVERTING_INPUT_GPIO_1    = LL_OPAMP_INPUT_NONINVERT_IO1, /*!< OPAMP non-inverting input 0 connected to
                                                 GPIO (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP1")*/
  HAL_OPAMP_NON_INVERTING_INPUT_GPIO_2    = LL_OPAMP_INPUT_NONINVERT_IO2, /*!< OPAMP non-inverting input 0 connected to
                                                 GPIO (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP2")*/
  HAL_OPAMP_NON_INVERTING_INPUT_DAC1_CH2  = LL_OPAMP_INPUT_NONINVERT_DAC1_CH2, /*!< OPAMP non-inverting input connected
                                                 to DAC1 channel 2 */
} hal_opamp_non_inverting_input_t;

/** @brief  OPAMP Inverting Input
  */
typedef enum
{
  HAL_OPAMP_INVERTING_INPUT_GPIO_0      = LL_OPAMP_INPUT_INVERT_IO0, /*!< OPAMP inverting input connected
                                        to GPIO pin.
                                        Note: This OPAMP inverting input is used with OPAMP in mode standalone,
                                              PGA with external bias or filtering, PGA with inverting gain.
                                              In mode PGA, select @ref HAL_OPAMP_INVERTING_INPUT_INT
                                              and configure input connection,
                                              refer to @ref HAL_OPAMP_SetPGAExternalMode(). */
  HAL_OPAMP_INVERTING_INPUT_GPIO_1      = LL_OPAMP_INPUT_INVERT_IO1, /*!< OPAMP inverting input connected
                                              to GPIO pin. */
  HAL_OPAMP_INVERTING_INPUT_INT         = (LL_OPAMP_INPUT_INVERT_INT_FOLLOWER \
                                           | LL_OPAMP_INPUT_INVERT_INT_PGA) /*!< OPAMP inverting input connection
                                        depending on follower or PGA mode:
                                        - For follower or PGA without external bias or filtering, PGA without
                                          inverting gain: inverting input internally connected (not connected
                                          to a GPIO pin).
                                        - For PGA with external bias or filtering, PGA with inverting gain:
                                          inverting input connected to GPIO_0 is used. To configure input
                                          connection, refer to @ref HAL_OPAMP_SetPGAExternalMode(). */
} hal_opamp_inverting_input_t;

/** @brief  OPAMP PGA Gain.
  * @note   Selection of PGA gain sign positive/negative, as well as external bias or filtering,
  *         is performed with function @ref HAL_OPAMP_SetPGAExternalMode() (literals hal_opamp_pga_external_t).
  */
typedef enum
{
  HAL_OPAMP_PGA_GAIN_2                  = LL_OPAMP_PGA_GAIN_2,  /*!< PGA gain = x  2 */
  HAL_OPAMP_PGA_GAIN_4                  = LL_OPAMP_PGA_GAIN_4,  /*!< PGA gain = x  4 */
  HAL_OPAMP_PGA_GAIN_8                  = LL_OPAMP_PGA_GAIN_8,  /*!< PGA gain = x  8 */
  HAL_OPAMP_PGA_GAIN_16                 = LL_OPAMP_PGA_GAIN_16, /*!< PGA gain = x 16 */
  HAL_OPAMP_PGA_GAIN_2_OR_MINUS_1       = LL_OPAMP_PGA_GAIN_2_OR_MINUS_1,   /*!< PGA gain = x  2 or  -1 */
  HAL_OPAMP_PGA_GAIN_4_OR_MINUS_3       = LL_OPAMP_PGA_GAIN_4_OR_MINUS_3,   /*!< PGA gain = x  4 or  -3 */
  HAL_OPAMP_PGA_GAIN_8_OR_MINUS_7       = LL_OPAMP_PGA_GAIN_8_OR_MINUS_7,   /*!< PGA gain = x  8 or  -7 */
  HAL_OPAMP_PGA_GAIN_16_OR_MINUS_15     = LL_OPAMP_PGA_GAIN_16_OR_MINUS_15, /*!< PGA gain = x 16 or -15 */
} hal_opamp_pga_gain_t;

/** @brief  OPAMP power_mode
  */
typedef enum
{
  HAL_OPAMP_POWER_MODE_NORMAL           = LL_OPAMP_POWER_MODE_NORMAL,  /*!< OPAMP in normal power mode */
} hal_opamp_power_mode_t;

/** @brief  OPAMP speed_mode
  */
typedef enum
{
  HAL_OPAMP_SPEED_MODE_NORMAL           = LL_OPAMP_SPEED_MODE_NORMAL,  /*!< OPAMP in normal speed mode */
  HAL_OPAMP_SPEED_MODE_HIGH             = LL_OPAMP_SPEED_MODE_HIGH     /*!< OPAMP in high speed mode */
} hal_opamp_speed_mode_t;


/** @brief  OPAMP trimming mode
  */
typedef enum
{
  HAL_OPAMP_TRIMMING_MODE_FACTORY       = LL_OPAMP_TRIMMING_FACTORY,  /*!< Factory trimming mode */
  HAL_OPAMP_TRIMMING_MODE_USER          = LL_OPAMP_TRIMMING_USER      /*!< User trimming mode */
} hal_opamp_trimming_mode_t;

/** @brief OPAMP Timer-controlled input selection
  */
typedef enum
{
  HAL_OPAMP_MUX_INPUT_CTRL_DISABLE      = LL_OPAMP_MUX_INPUT_CTRL_DISABLE, /*!< Timer-controlled input selection
                                        disabled */
  HAL_OPAMP_MUX_INPUT_CTRL_TIM1_OC6     = LL_OPAMP_MUX_INPUT_CTRL_TIM1_OC6, /*!< Timer-controlled input selection using
                                        TIM1 OC6 */
  HAL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4     = LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4, /*!< Timer-controlled input selection using
                                        TIM2 OC4 */
  HAL_OPAMP_MUX_INPUT_CTRL_TIM12_OC1    = LL_OPAMP_MUX_INPUT_CTRL_TIM12_OC1, /*!< Timer-controlled input selection using
                                        TIM12 OC1 */
  HAL_OPAMP_MUX_INPUT_CTRL_TIM15_OC2    = LL_OPAMP_MUX_INPUT_CTRL_TIM15_OC2, /*!< Timer-controlled input selection using
                                        TIM15 OC2 */
} hal_opamp_mux_inputs_ctrl_t;

/** @brief OPAMP Timer-controlled PGA mode selection
  */
typedef enum
{
  HAL_OPAMP_MUX_PGA_GAIN_CTRL_DISABLE   = LL_OPAMP_MUX_PGA_GAIN_CTRL_DISABLE, /*!< Timer-controlled pga mode selection
                                        disabled */
  HAL_OPAMP_MUX_PGA_GAIN_CTRL_TIM1_OC6  = LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM1_OC6, /*!< Timer-controlled pga mode selection
                                        using TIM1 OC6 */
  HAL_OPAMP_MUX_PGA_GAIN_CTRL_TIM2_OC4  = LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM2_OC4, /*!< Timer-controlled pga mode selection
                                        using TIM2 OC4 */
  HAL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC2 = LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC2, /*!< Timer-controlled pga mode selection
                                        using TIM12 OC2 */
  HAL_OPAMP_MUX_PGA_GAIN_CTRL_TIM15_OC2 = LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM15_OC2, /*!< Timer-controlled pga mode selection
                                        using TIM15 OC2 */
} hal_opamp_mux_pga_ctrl_tim_t;

/**
  * @brief OPAMP output connection
  */
typedef enum
{
  HAL_OPAMP_OUTPUT_CONNECTION_EXTERNAL  = LL_OPAMP_OUTPUT_CONNECT_EXTERNAL, /*!< OPAMP output connected
                                        to OPAMP_VOUT pin. */
  HAL_OPAMP_OUTPUT_CONNECTION_INTERNAL  = LL_OPAMP_OUTPUT_CONNECT_INTERNAL  /*!< OPAMP output connected internally
                                        other peripherals (ADC, COMP, ...). */
} hal_opamp_out_connection_t;

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Types_Group2 Handle Structure
  * @{
  */
typedef struct hal_opamp_handle_s hal_opamp_handle_t; /*!< OPAMP handle type definition */

/**
  * @brief   OPAMP handle,
  *          contains: OPAMP instance, states.
  */
struct hal_opamp_handle_s
{
  hal_opamp_t                 instance;      /*!< Peripheral instance */
  volatile hal_opamp_state_t  global_state;  /*!< OPAMP global_state */
#if defined(USE_HAL_OPAMP_USER_DATA) && (USE_HAL_OPAMP_USER_DATA == 1)
  const void                  *p_user_data;  /*!< User data pointer */
#endif /* USE_HAL_OPAMP_USER_DATA */
};

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Types_Group4 Configuration Structure
  * @{
  */

/**
  * @brief  OPAMP configuration structure definition
  */
typedef struct
{
  hal_opamp_speed_mode_t   speed_mode;           /*!< The speed mode: normal-speed or high-speed */
  hal_opamp_config_mode_t  configuration_mode;   /*!< The OPAMP configuration mode: Standalone, Follower or PGA */
  hal_opamp_out_connection_t opamp_output;          /*!< Configure the OPAMP output connection */
} hal_opamp_config_t;

/**
  * @brief  OPAMP input connection configuration structure definition
  */
typedef struct
{
  hal_opamp_inverting_input_t      inverting_input;      /*!< The inverting input in Standalone & PGA modes.
                                                             In Follower mode this parameter is Not Applicable. */
  hal_opamp_non_inverting_input_t  non_inverting_input;  /*!< The non-inverting input of the opamp */
} hal_opamp_config_input_connection_t;

/**
  * @brief   OPAMP offset trimming pair structure definition
  */
typedef struct
{
  uint32_t  trim_offset_p;           /*!< The offset trimming value (PMOS) in normal-power mode.
                                          This parameter must be a number between Min_Data = 1 and Max_Data = 31,
                                          typical default value is 16 */
  uint32_t  trim_offset_n;           /*!< The offset trimming value (NMOS) in normal-power mode.
                                          This parameter must be a number between Min_Data = 1 and Max_Data = 31,
                                          typical default value is 16 */
} hal_opamp_trimming_offset_pair_t;

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup OPAMP_Exported_Functions HAL OPAMP Functions
  * @{
  */

/** @defgroup OPAMP_Exported_Functions_Group1 Initialization, de-initialization, configuration, calibration functions
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
hal_status_t HAL_OPAMP_Init(hal_opamp_handle_t *hopamp, hal_opamp_t instance);
void HAL_OPAMP_DeInit(hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetConfig(hal_opamp_handle_t *hopamp, const hal_opamp_config_t *p_config);
void HAL_OPAMP_GetConfig(const hal_opamp_handle_t *hopamp, hal_opamp_config_t *p_config);
void HAL_OPAMP_ResetConfig(hal_opamp_handle_t *hopamp);

/* Calibration functions  */
hal_status_t HAL_OPAMP_Calibrate(hal_opamp_handle_t *hopamp, hal_opamp_power_mode_t power_mode);

hal_opamp_trimming_mode_t HAL_OPAMP_GetTrimmingMode(const hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetConfigTrimming(const hal_opamp_handle_t *hopamp,
                                         const hal_opamp_trimming_offset_pair_t *p_config,
                                         hal_opamp_power_mode_t power_mode);

void HAL_OPAMP_GetConfigTrimming(const hal_opamp_handle_t *hopamp,
                                 hal_opamp_trimming_offset_pair_t *p_config,
                                 hal_opamp_power_mode_t power_mode);

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Functions_Group2 Output operation functions
  * @{
  */

/* I/O operation functions  ***************************************************/
hal_status_t HAL_OPAMP_Start(hal_opamp_handle_t *hopamp);
hal_status_t HAL_OPAMP_Stop(hal_opamp_handle_t *hopamp);

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */

/* Peripheral Control functions  **********************************************/
hal_status_t HAL_OPAMP_SetConfigInputConnection(const hal_opamp_handle_t *hopamp,
                                                const hal_opamp_config_input_connection_t *p_config);

void HAL_OPAMP_GetConfigInputConnection(const hal_opamp_handle_t *hopamp,
                                        hal_opamp_config_input_connection_t *p_config);

hal_status_t HAL_OPAMP_SetConfigInputMuxSecondaryConnection(const hal_opamp_handle_t *hopamp,
                                                            const hal_opamp_config_input_connection_t *p_config);

/* gain functions */
hal_status_t HAL_OPAMP_SetGain(const hal_opamp_handle_t *hopamp, hal_opamp_pga_gain_t gain);
hal_opamp_pga_gain_t HAL_OPAMP_GetGain(const hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetConfigModeMuxSecondary(hal_opamp_handle_t *hopamp,
                                                 hal_opamp_config_mode_t configuration_mode);
hal_opamp_config_mode_t HAL_OPAMP_GetConfigModeMuxSecondary(const hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetPGAGainMuxSecondary(const hal_opamp_handle_t *hopamp, hal_opamp_pga_gain_t gain);
hal_opamp_pga_gain_t HAL_OPAMP_GetPGAGainMuxSecondary(const hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetPGAExternalMode(const hal_opamp_handle_t *hopamp, hal_opamp_pga_external_t external_mode);
hal_opamp_pga_external_t HAL_OPAMP_GetPGAExternalMode(const hal_opamp_handle_t *hopamp);

hal_status_t HAL_OPAMP_SetMuxInputCtrl(const hal_opamp_handle_t *hopamp, hal_opamp_mux_inputs_ctrl_t mux_inputs_ctrl);
hal_opamp_mux_inputs_ctrl_t HAL_OPAMP_GetMuxInputCtrl(const hal_opamp_handle_t *hopamp);
hal_status_t HAL_OPAMP_SetMuxPGAGainCtrl(const hal_opamp_handle_t *hopamp, hal_opamp_mux_pga_ctrl_tim_t mux_pga_ctrl);
hal_opamp_mux_pga_ctrl_tim_t HAL_OPAMP_GetMuxPGAGainCtrl(const hal_opamp_handle_t *hopamp);

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Functions_Group4 Peripheral state functions
  * @{
  */

/* Peripheral State functions  ************************************************/
hal_opamp_state_t HAL_OPAMP_GetState(const hal_opamp_handle_t *hopamp);

/**
  * @}
  */

/** @defgroup OPAMP_Exported_Functions_Group6 User Data API functions
  * @{
  */
/* User Data API functions ***************************************/
#if defined(USE_HAL_OPAMP_USER_DATA) && (USE_HAL_OPAMP_USER_DATA == 1)
void HAL_OPAMP_SetUserData(hal_opamp_handle_t *hopamp, const void *p_user_data);
const void *HAL_OPAMP_GetUserData(const hal_opamp_handle_t *hopamp);
#endif /* USE_HAL_OPAMP_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* OPAMP1 || OPAMP2 || OPAMP3 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_OPAMP_H */

