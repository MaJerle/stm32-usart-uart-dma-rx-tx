/**
  ******************************************************************************
  * @file    stm32c5xx_hal_comp.h
  * @brief   Header file of COMP HAL module.
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
#ifndef STM32C5XX_HAL_COMP_H
#define STM32C5XX_HAL_COMP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_comp.h"
#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
#include "stm32c5xx_ll_exti.h"
#endif /* USE_HAL_COMP_EXTI */

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(COMP1) || defined(COMP2)

/** @defgroup COMP COMP
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup  COMP_Exported_Types HAL COMP Types
  * @{
  */

/**
  * @brief HAL COMP instance
  */
typedef enum
{
  HAL_COMP1 = COMP1_BASE,
#if defined(COMP2)
  HAL_COMP2 = COMP2_BASE,
#endif /* COMP2 */
} hal_comp_t;

/**
  * @brief COMP Global state
  */
typedef enum
{
  HAL_COMP_STATE_RESET                  = 0UL,              /*!< HAL Comparator handle not yet initialized */
  HAL_COMP_STATE_INIT                   = (1UL << 31U),     /*!< HAL Comparator handle initialized but comparator
                                        not yet configured */
  HAL_COMP_STATE_IDLE                   = (1UL << 30U),     /*!< Comparator configured */
  HAL_COMP_STATE_ACTIVE                 = (1UL << 29U),     /*!< Comparator operating */
  HAL_COMP_STATE_LINKED                 = (1UL << 28U),     /*!< HAL Comparator handle linked to other
                                        comparator handle */
#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
  HAL_COMP_STATE_WINDOW_IDLE            = (1UL << 27U),     /*!< Comparator configured in window mode (with other
                                        comparator handle) */
  HAL_COMP_STATE_WINDOW_ACTIVE          = (1UL << 26U),     /*!< Comparator operating in window mode (with other
                                        comparator handle) */
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */
} hal_comp_state_t;

/**
  * @brief HAL COMP power mode.
  * @note  For the electrical characteristics of comparator power modes (propagation delay, power consumption),
  *        refer to device datasheet.
  */
typedef enum
{
  HAL_COMP_POWER_MODE_HIGH_SPEED        = LL_COMP_POWER_MODE_HIGH_SPEED,   /*!< Comparator power mode to high speed */
  HAL_COMP_POWER_MODE_MEDIUM_SPEED      = LL_COMP_POWER_MODE_MEDIUM_SPEED, /*!< Comparator power mode to medium speed */
  HAL_COMP_POWER_MODE_ULTRA_LOW_POWER   = LL_COMP_POWER_MODE_ULTRA_LOW_POWER /*!< Comparator power mode to ultra-low
                                        power */
} hal_comp_power_mode_t;

/**
  * @brief HAL COMP input plus.
  */
typedef enum
{
  HAL_COMP_INPUT_PLUS_IO1               = LL_COMP_INPUT_PLUS_IO1, /*!< Comparator input plus connected to IO1
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP1"). */
  HAL_COMP_INPUT_PLUS_IO2               = LL_COMP_INPUT_PLUS_IO2, /*!< Comparator input plus connected to IO2
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP2"). */
  HAL_COMP_INPUT_PLUS_IO3               = LL_COMP_INPUT_PLUS_IO3, /*!< Comparator input plus connected to IO3
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP3"). */
  HAL_COMP_INPUT_PLUS_DAC1_CH1          = LL_COMP_INPUT_PLUS_DAC1_CH1, /*!< Comparator input plus connected to
                                        DAC1 channel 1.
                                        Specific to COMP instance: COMP1. */
#if defined(COMP2)
  HAL_COMP_INPUT_PLUS_DAC1_CH2          = LL_COMP_INPUT_PLUS_DAC1_CH2, /*!< Comparator input plus connected to
                                        DAC1 channel 2.
                                        Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx). */
#endif /* COMP2 */
} hal_comp_input_plus_t;

/**
  * @brief HAL COMP input minus
  */
typedef enum
{
  HAL_COMP_INPUT_MINUS_VREFINT          = LL_COMP_INPUT_MINUS_VREFINT,    /*!< Comparator input minus connected to
                                        VrefInt (for VrefInt voltage value, refer to the datasheet). */
  HAL_COMP_INPUT_MINUS_1_4VREFINT       = LL_COMP_INPUT_MINUS_1_4VREFINT, /*!< Comparator input minus connected to
                                        1/4 VrefInt (for VrefInt voltage value, refer to the datasheet). */
  HAL_COMP_INPUT_MINUS_1_2VREFINT       = LL_COMP_INPUT_MINUS_1_2VREFINT, /*!< Comparator input minus connected to
                                        1/2 VrefInt (for VrefInt voltage value, refer to the datasheet). */
  HAL_COMP_INPUT_MINUS_3_4VREFINT       = LL_COMP_INPUT_MINUS_3_4VREFINT, /*!< Comparator input minus connected to
                                        3/4 VrefInt (for VrefInt voltage value, refer to the datasheet). */
  HAL_COMP_INPUT_MINUS_IO1              = LL_COMP_INPUT_MINUS_IO1, /*!< Comparator input minus connected to IO1
                                        (for GPIO mapping, refer to datasheet parameters "COMPx_INM1"). */
  HAL_COMP_INPUT_MINUS_IO2              = LL_COMP_INPUT_MINUS_IO2, /*!< Comparator input minus connected to IO2
                                        (for GPIO mapping, refer to datasheet parameters "COMPx_INM2"). */
  HAL_COMP_INPUT_MINUS_IO3              = LL_COMP_INPUT_MINUS_IO3, /*!< Comparator input minus connected to IO3
                                        (for GPIO mapping, refer to datasheet parameters "COMPx_INM3"). */
  HAL_COMP_INPUT_MINUS_DAC1_CH1         = LL_COMP_INPUT_MINUS_DAC1_CH1, /*!< Comparator input minus connected
                                        to DAC1 channel 1.
                                        Specific to COMP instances: COMP1. */
#if defined(COMP2)
  HAL_COMP_INPUT_MINUS_DAC1_CH2         = LL_COMP_INPUT_MINUS_DAC1_CH2, /*!< Comparator input minus connected
                                        to DAC1 channel 2.
                                        Specific to COMP instances: COMP2 (available on devices STM32C53xx/54xx). */
  HAL_COMP_INPUT_MINUS_OPAMP1_OUT       = LL_COMP_INPUT_MINUS_OPAMP1_OUT, /*!< Comparator input minus connected
                                        to OPAMP1 output.
                                        Specific to COMP instances: COMP2 (available on devices STM32C53xx/54xx). */
#endif /* COMP2 */
  HAL_COMP_INPUT_MINUS_TEMPSENSOR       = LL_COMP_INPUT_MINUS_TEMPSENSOR, /*!< Comparator input minus connected
                                        to the internal temperature sensor (also accessible through the ADC
                                        peripheral).
                                        Note: Specific configuration with bitfields out of comparator registers
                                        due to the temperature sensor buffer controlled by the ADC clock
                                        domain.
                                        Caution: ADC clock domain reset or disable affects the temperature
                                        sensor. */
} hal_comp_input_minus_t;

/**
  * @brief HAL COMP input hysteresis.
  * @note  Hysteresis applied to input plus, subtracted to input voltage value.
  * @note  For the electrical characteristics of comparator hysteresis (voltage amplitude),
  *        refer to device datasheet.
  */
typedef enum
{
  HAL_COMP_INPUT_HYSTERESIS_NONE        = LL_COMP_HYSTERESIS_NONE,   /*!< Comparator input without hysteresis */
  HAL_COMP_INPUT_HYSTERESIS_LOW         = LL_COMP_HYSTERESIS_LOW,    /*!< Comparator input hysteresis level low */
  HAL_COMP_INPUT_HYSTERESIS_MEDIUM      = LL_COMP_HYSTERESIS_MEDIUM, /*!< Comparator input hysteresis level medium */
  HAL_COMP_INPUT_HYSTERESIS_HIGH        = LL_COMP_HYSTERESIS_HIGH    /*!< Comparator input hysteresis level high */
} hal_comp_input_hysteresis_t;

/**
  * @brief HAL COMP output polarity
  */
typedef enum
{
  HAL_COMP_OUTPUT_POLARITY_NONINVERTED  = LL_COMP_OUTPUTPOL_NONINVERTED, /*!< Comparator output polarity not inverted:
                                        comparator output at high level when input voltages: plus higher than minus */
  HAL_COMP_OUTPUT_POLARITY_INVERTED     = LL_COMP_OUTPUTPOL_INVERTED     /*!< Comparator output polarity not inverted:
                                        comparator output at low level when input voltages: plus higher than minus */
} hal_comp_output_polarity_t;

/**
  * @brief HAL COMP output blanking
  */
typedef enum
{
  HAL_COMP_OUTPUT_BLANK_NONE            = LL_COMP_BLANKINGSRC_NONE,     /*!< Comparator output without blanking. */
  HAL_COMP_OUTPUT_BLANK_TIM1_OC5        = LL_COMP_BLANKINGSRC_TIM1_OC5, /*!< Comparator output blanking source
                                        TIM1 OC5. */
  HAL_COMP_OUTPUT_BLANK_TIM1_OC6        = LL_COMP_BLANKINGSRC_TIM1_OC6, /*!< Comparator output blanking source
                                        TIM1 OC6. */
  HAL_COMP_OUTPUT_BLANK_TIM2_OC3        = LL_COMP_BLANKINGSRC_TIM2_OC3, /*!< Comparator output blanking source
                                        TIM2 OC3. */
  HAL_COMP_OUTPUT_BLANK_TIM5_OC3        = LL_COMP_BLANKINGSRC_TIM5_OC3, /*!< Comparator output blanking source
                                        TIM5 OC3. */
  HAL_COMP_OUTPUT_BLANK_TIM5_OC4        = LL_COMP_BLANKINGSRC_TIM5_OC4, /*!< Comparator output blanking source
                                        TIM3 OC4. */
  HAL_COMP_OUTPUT_BLANK_TIM8_OC5        = LL_COMP_BLANKINGSRC_TIM8_OC5, /*!< Comparator output blanking source
                                        TIM8 OC5. */
  HAL_COMP_OUTPUT_BLANK_TIM15_OC2       = LL_COMP_BLANKINGSRC_TIM15_OC2, /*!< Comparator output blanking source
                                        TIM15 OC2. */
  HAL_COMP_OUTPUT_BLANK_LPTIM1_OC1      = LL_COMP_BLANKINGSRC_LPTIM1_OC1, /*!< Comparator output blanking source
                                        LPTIM1 OC1. */
} hal_comp_output_blank_t;

#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
/**
  * @brief HAL COMP output trigger to system.
  * @note  When output set to generate a trigger, impact depends on programming model used:
  *        - with @ref HAL_COMP_Start():    generates a system wake-up event and a CPU event
  *        - with @ref HAL_COMP_Start_IT(): generates a system wake-up event and a CPU interrupt
  */
typedef enum
{
  HAL_COMP_OUTPUT_TRIG_NONE             = LL_EXTI_TRIGGER_NONE, /*!< Comparator output does not generate a trigger */
  HAL_COMP_OUTPUT_TRIG_RISING           = LL_EXTI_TRIGGER_RISING, /*!< Comparator output generates a trigger
                                        event to system on rising edge */
  HAL_COMP_OUTPUT_TRIG_FALLING          = LL_EXTI_TRIGGER_FALLING, /*!< Comparator output generates a trigger
                                        event to system on falling edge */
  HAL_COMP_OUTPUT_TRIG_RISING_FALLING   = LL_EXTI_TRIGGER_RISING_FALLING  /*!< Comparator output generates a trigger
                                        event to system on both rising and falling edges */
} hal_comp_output_trigger_t;
#endif /* USE_HAL_COMP_EXTI */

#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
/**
  * @brief HAL COMP window output
  */
typedef enum
{
  HAL_COMP_WINDOW_OUTPUT_INDEPT         = LL_COMP_WINDOW_OUTPUT_INDEPT, /*!< Comparators window output default mode:
                                        both comparators output are independent, indicating each their own state.
                                        Note: To know signal state versus window thresholds, read each comparator
                                              output and perform a logical "exclusive or" operation. */
  HAL_COMP_WINDOW_OUTPUT_XOR            = LL_COMP_WINDOW_OUTPUT_XOR_BOTH, /*!< Comparators window output synthesized on
                                        a single comparator output: comparator no more indicating its own state,
                                        but window state (XOR: logical "exclusive or"). Logical high means monitored
                                        signal is within comparators window thresholds.
                                        Comparator instance selected corresponds to handle assigned as upper threshold
                                        in @ref HAL_COMP_WINDOW_SetHandle().
                                        Note: impacts only comparator output signal level (propagated to GPIO,
                                              EXTI lines, timers, ...), does not impact output digital state
                                              (hal_comp_output_level_t) always reflecting each comparator
                                              output state. */
} hal_comp_window_output_mode_t;

/**
  * @brief HAL COMP window output level.
  * @note  Comparator output level depends on inputs voltages and output polarity.
  */
typedef enum
{
  HAL_COMP_WINDOW_OUTPUT_LEVEL_BELOW    = (0x00000000UL), /*!< Comparators window output below window low threshold */
  HAL_COMP_WINDOW_OUTPUT_LEVEL_WITHIN   = (0x00000001UL), /*!< Comparators window output within window thresholds */
  HAL_COMP_WINDOW_OUTPUT_LEVEL_ABOVE    = (0x00000002UL), /*!< Comparators window output above window high threshold */
} hal_comp_window_output_level_t;

/**
  * @brief HAL COMP window instances assignment
  */
typedef enum
{
  HAL_COMP_WINDOW_INST_NONE             = (0x00000000UL), /*!< Comparators window instance not assigned */
  HAL_COMP_WINDOW_INST_UPPER            = (0x00000001UL), /*!< Comparators window instance assignment upper */
  HAL_COMP_WINDOW_INST_LOWER            = (0x00000002UL)  /*!< Comparators window instance assignment lower */
} hal_comp_window_instance_t;

#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */

/**
  * @brief HAL COMP lock state
  */
typedef enum
{
  HAL_COMP_LOCK_DISABLED                = (0x00000000UL), /*!< Comparator not locked */
  HAL_COMP_LOCK_ENABLED                 = (0x00000001UL)  /*!< Comparator locked */
} hal_comp_lock_status_t;

/**
  * @brief HAL COMP output level.
  * @note  Comparator output level depends on inputs voltages and output polarity.
  */
typedef enum
{
  HAL_COMP_OUTPUT_LEVEL_LOW             = LL_COMP_OUTPUT_LEVEL_LOW, /*!< Comparator output logical level low */
  HAL_COMP_OUTPUT_LEVEL_HIGH            = LL_COMP_OUTPUT_LEVEL_HIGH /*!< Comparator output logical level high */
} hal_comp_output_level_t;

/**
  * @brief COMP Global configuration
  */
typedef struct
{
  hal_comp_power_mode_t                 power_mode;         /*!< Comparator power mode */
  hal_comp_input_plus_t                 input_plus;         /*!< Comparator input plus */
  hal_comp_input_minus_t                input_minus;        /*!< Comparator input minus */
  hal_comp_input_hysteresis_t           input_hysteresis;   /*!< Comparator input hysteresis */
  hal_comp_output_polarity_t            output_polarity;    /*!< Comparator output polarity */
#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
  hal_comp_output_trigger_t             output_trigger;     /*!< Comparator output trigger to system (wake up, CPU) */
#endif /* USE_HAL_COMP_EXTI */
} hal_comp_config_t;

#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
/**
  * @brief COMP window mode configuration
  */
typedef struct
{
  hal_comp_input_plus_t                 input;              /*!< Window comparators input.
                                        @note This parameter corresponds to common input plus. Comparators instances
                                              pair have their input plus connected together (common input plus).
                                              The input plus used corresponds to handle used in first argument
                                              (hcomp_upper_threshold) of function @ref HAL_COMP_WINDOW_SetHandle()
                                               (input plus of the other comparator is no more accessible) */
  hal_comp_input_minus_t                upper_threshold;    /*!< Window comparators upper threshold.
                                        @note This parameter corresponds to input minus of handle used in
                                              first argument (hcomp_upper_threshold) of function
                                              @ref HAL_COMP_WINDOW_SetHandle().
                                        @note Term "upper" does not imply voltage value must
                                              be higher than the other threshold.
                                              This is an arbitrary selection to determine window
                                              output level (@ref hal_comp_window_output_level_t).*/
  hal_comp_input_minus_t                lower_threshold;    /*!< Window comparators lower threshold.
                                        @note This parameter corresponds to input minus of handle used in
                                              second argument (hcomp_lower_threshold) of function
                                              @ref HAL_COMP_WINDOW_SetHandle().
                                        @note Term "lower" does not imply voltage value must be higher than the other
                                              threshold. This is an arbitrary selection to determine window
                                              output level (@ref hal_comp_window_output_level_t).*/
  hal_comp_power_mode_t                 power_mode;         /*!< Comparator power mode */
  hal_comp_input_hysteresis_t           input_hysteresis;   /*!< Comparator input hysteresis */
  hal_comp_output_polarity_t            output_polarity;    /*!< Comparator output polarity */
#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
  hal_comp_output_trigger_t             output_trigger;     /*!< Comparator output trigger to system (wake up, CPU) */
#endif /* USE_HAL_COMP_EXTI */
  hal_comp_window_output_mode_t         window_output_mode; /*!< Comparator window output */
} hal_comp_window_config_t;
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */

typedef struct hal_comp_handle_s hal_comp_handle_t; /*!< COMP handle type definition */

#if defined (USE_HAL_COMP_REGISTER_CALLBACKS) && (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
typedef  void (*hal_comp_cb_t)(hal_comp_handle_t *hcomp);  /*!< pointer to COMP callback functions */
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

/**
  * @brief COMP Handle Structure Definition
  */
struct hal_comp_handle_s
{
  hal_comp_t instance;                    /*!< Peripheral instance */

#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
  hal_comp_handle_t                     *p_link_next_handle;  /*!< Pointer to another HAL COMP handle of instance
                                        belonging to the same COMP common instance (therefore, sharing common features).
                                        Used to access multiple HAL COMP handles (daisy chain: from one to another
                                        and circular).
                                        Set using function @ref HAL_COMP_WINDOW_SetHandle(). */
  hal_comp_window_instance_t            window_instance; /*!< Comparators window instance assignment */
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */

  volatile hal_comp_state_t             global_state; /*!< Global state */

#if defined (USE_HAL_COMP_USER_DATA) && (USE_HAL_COMP_USER_DATA == 1)
  const void                            *p_user_data; /*!< User data pointer */
#endif /* USE_HAL_COMP_USER_DATA */

#if defined (USE_HAL_COMP_REGISTER_CALLBACKS) && (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
  hal_comp_cb_t                         p_output_trigger_cb; /*!< COMP output trigger callback */
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
  uint32_t                              exti_line; /*!< EXTI line (needed for event and IT operation)
                                        on LL driver format */
  hal_comp_output_trigger_t             output_trigger; /*!< Comparator output trigger configured */
#endif /* USE_HAL_COMP_EXTI */
};

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup COMP_Exported_Functions HAL COMP Functions
  * @{
  */

/** @defgroup COMP_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_COMP_Init(hal_comp_handle_t *hcomp, hal_comp_t instance);
void HAL_COMP_DeInit(hal_comp_handle_t *hcomp);
#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
hal_status_t HAL_COMP_WINDOW_SetHandle(hal_comp_handle_t *hcomp_upper,
                                       hal_comp_handle_t *hcomp_lower);
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */
/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group2 Configuration functions
  * @{
  */
hal_status_t HAL_COMP_SetConfig(hal_comp_handle_t *hcomp, const hal_comp_config_t *p_config);
void HAL_COMP_GetConfig(const hal_comp_handle_t *hcomp, hal_comp_config_t *p_config);

hal_status_t HAL_COMP_SetInputPlus(hal_comp_handle_t *hcomp, hal_comp_input_plus_t input_plus);
hal_comp_input_plus_t HAL_COMP_GetInputPlus(const hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_SetInputMinus(hal_comp_handle_t *hcomp, hal_comp_input_minus_t input_minus);
hal_comp_input_minus_t HAL_COMP_GetInputMinus(const hal_comp_handle_t *hcomp);

hal_status_t HAL_COMP_SetOutputBlanking(hal_comp_handle_t *hcomp, hal_comp_output_blank_t output_blank);
hal_comp_output_blank_t HAL_COMP_GetOutputBlanking(hal_comp_handle_t *hcomp);

#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
hal_status_t HAL_COMP_WINDOW_SetConfig(hal_comp_handle_t *hcomp, const hal_comp_window_config_t *p_config);
void HAL_COMP_WINDOW_GetConfig(const hal_comp_handle_t *hcomp, hal_comp_window_config_t *p_config);

hal_status_t HAL_COMP_WINDOW_SetInput(hal_comp_handle_t *hcomp, hal_comp_input_plus_t input);
hal_comp_input_plus_t HAL_COMP_WINDOW_GetInput(const hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_WINDOW_SetUpperThreshold(hal_comp_handle_t *hcomp, hal_comp_input_minus_t upper_threshold);
hal_comp_input_minus_t HAL_COMP_WINDOW_GetUpperThreshold(const hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_WINDOW_SetLowerThreshold(hal_comp_handle_t *hcomp, hal_comp_input_minus_t lower_threshold);
hal_comp_input_minus_t HAL_COMP_WINDOW_GetLowerThreshold(const hal_comp_handle_t *hcomp);

hal_status_t HAL_COMP_WINDOW_SetOutputBlanking(hal_comp_handle_t *hcomp, hal_comp_output_blank_t output_blank);
hal_comp_output_blank_t HAL_COMP_WINDOW_GetOutputBlanking(const hal_comp_handle_t *hcomp);
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */
/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group3 IRQHandler and Callbacks functions
  * @{
  */
void HAL_COMP_IRQHandler(hal_comp_handle_t *hcomp);
void HAL_COMP_OutputTriggerCallback(hal_comp_handle_t *hcomp);
#if defined (USE_HAL_COMP_REGISTER_CALLBACKS) && (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
hal_status_t HAL_COMP_RegisterOutputTriggerCallback(hal_comp_handle_t *hcomp, hal_comp_cb_t p_callback);
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group4 Peripheral State, Error functions
  * @{
  */
hal_comp_state_t HAL_COMP_GetState(const hal_comp_handle_t *hcomp);
/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group5 Process functions
  * @{
  */
hal_status_t HAL_COMP_Start(hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_Stop(hal_comp_handle_t *hcomp);
#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
hal_status_t HAL_COMP_Start_IT(hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_Stop_IT(hal_comp_handle_t *hcomp);
#endif /* USE_HAL_COMP_EXTI */
hal_status_t HAL_COMP_Lock(hal_comp_handle_t *hcomp);
hal_comp_lock_status_t HAL_COMP_IsLocked(hal_comp_handle_t *hcomp);
hal_comp_output_level_t HAL_COMP_GetOutputLevel(hal_comp_handle_t *hcomp);
#if defined(COMP_WINDOW_MODE_SUPPORT)
#if defined(USE_HAL_COMP_WINDOW_MODE) && (USE_HAL_COMP_WINDOW_MODE == 1)
hal_status_t HAL_COMP_WINDOW_Start(hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_WINDOW_Stop(hal_comp_handle_t *hcomp);
#if defined(USE_HAL_COMP_EXTI) && (USE_HAL_COMP_EXTI == 1)
hal_status_t HAL_COMP_WINDOW_Start_IT(hal_comp_handle_t *hcomp);
hal_status_t HAL_COMP_WINDOW_Stop_IT(hal_comp_handle_t *hcomp);
#endif /* USE_HAL_COMP_EXTI */
hal_status_t HAL_COMP_WINDOW_Lock(hal_comp_handle_t *hcomp);
hal_comp_lock_status_t HAL_COMP_WINDOW_IsLocked(hal_comp_handle_t *hcomp);
hal_comp_window_output_level_t HAL_COMP_WINDOW_GetOutputLevel(hal_comp_handle_t *hcomp);
#endif /* USE_HAL_COMP_WINDOW_MODE */
#endif /* COMP_WINDOW_MODE_SUPPORT */
/**
  * @}
  */

/** @defgroup COMP_Exported_Functions_Group6 User data functions
  * @{
  */
#if defined(USE_HAL_COMP_USER_DATA) && (USE_HAL_COMP_USER_DATA == 1)
void HAL_COMP_SetUserData(hal_comp_handle_t *hcomp, const void *p_user_data);
const void *HAL_COMP_GetUserData(const hal_comp_handle_t *hcomp);
#endif /* USE_HAL_COMP_USER_DATA */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* COMP1 || COMP2 || COMP3 || COMP4 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_COMP_H */
