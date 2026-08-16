/**
  ******************************************************************************
  * @file    stm32c5xx_ll_comp.h
  * @brief   Header file of COMP LL module.
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
#ifndef STM32C5XX_LL_COMP_H
#define STM32C5XX_LL_COMP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(COMP1) || defined(COMP2)

/** @defgroup COMP_LL COMP
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Constants COMP Private Constants
  * @{
  */
#define LL_COMP_REG(__COMP__) ((__COMP__)->CFGR1)

/* Internal masks and bitfields for pair of comparators instances window mode
To select into literals LL_COMP_WINDOW_x the relevant bits for (concatenation of multiple bits used in different
registers):
- Comparator instance selected as master for window mode : register offset
- Window mode enable or disable: bit value */
#define LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK  (0x00000000UL) /* Register of COMP instance odd (COMP1_CSR, ...)
                                        defined as reference register */
#define LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK (0x00000001UL) /* Register of COMP instance even (COMP2_CSR, ...)
                                        offset vs register of COMP instance odd */
#define LL_COMP_WINDOW_COMP_REGOFFSET_MASK (LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK \
                                            | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK)
#if defined(COMP_CFGR1_WINMODE)
#define LL_COMP_WINDOW_COMP_X_SETTING_MASK (COMP_CFGR1_WINMODE) /* Bitfield to select window mode */
#define LL_COMP_WINDOW_OUT_SETTING_MASK (COMP_CFGR1_WINOUT) /* Bitfield to select window output */
#define LL_COMP_WINDOW_OUT_XOR_BOTH_MASK (COMP_CFGR1_WINOUT << 1UL) /* Differentiator of window output settings */
#else
#define LL_COMP_WINDOW_COMP_X_SETTING_MASK (COMP_CSR_INPSEL)  /* Bitfield to select window mode */
#define LL_COMP_WINDOW_OUT_SETTING_MASK (COMP_CSR_WINOUT) /* Bitfield to select window output */
#define LL_COMP_WINDOW_OUT_XOR_BOTH_MASK (COMP_CSR_WINOUT << 1UL) /* Differentiator of window output settings */
#endif /* COMP_CFGR1_WINMODE */

#define LL_COMP_WINDOW_OUT_XOR_BOTH_POS_VS_WINDOW (1UL) /* Bitfields position differences between
                                        LL_COMP_WINDOW_OUT_XOR_BOTH_MASK and LL_COMP_WINDOW_COMP_X_SETTING_MASK */

/* COMP instances relative position in COMP common instance */
#define LL_COMP_WINDOW_COMP_ODD         (LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK)  /* Comparator instance odd
                                        (COMP1, ...) */
#define LL_COMP_WINDOW_COMP_EVEN        (LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK) /* Comparator instance even
                                        (COMP2, ...) */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Macros COMP Private Macros
  * @{
  */

/**
  * @brief  Driver macro reserved for internal use: set a pointer to
  *         a register from a register basis from which an offset
  *         is applied.
  * @param  __REG__ Register basis from which the offset is applied.
  * @param  __REG_OFFFSET__ Offset to be applied (unit: number of registers).
  * @retval Pointer to register address.
  */
#define LL_COMP_PTR_REG_OFFSET(__REG__, __REG_OFFFSET__) \
  ((__IO uint32_t *)((uint32_t) ((uint32_t)(&(__REG__)) + ((__REG_OFFFSET__) << 2UL))))

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Constants LL COMP Constants
  * @{
  */

#if defined(COMP_WINDOW_MODE_SUPPORT)
/** @defgroup COMP_LL_EC_COMMON_WINDOWMODE Comparator common modes - Window mode
  * @{
  */
#define LL_COMP_WINDOW_DISABLE          (0x00000000UL) /*!< Window mode disable: comparators pair are independent */
#if defined(COMP_CFGR1_WINMODE)
#define LL_COMP_WINDOW_INPUT_PLUS_ODD   (COMP_CFGR1_WINMODE | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK) /*!< Window mode
                                        enable: comparators instances pair have their input plus connected together.
                                        The common input is the one of instance index odd (COMP1, ...).
                                        Input plus of the other comparator is no more accessible. */
#define LL_COMP_WINDOW_INPUT_PLUS_EVEN  (COMP_CFGR1_WINMODE | LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK) /*!< Window mode
                                        enable: comparators instances pair have their input plus connected together.
                                        The common input is the one of instance index even (COMP2, ...).
                                        Input plus of the other comparator is no more accessible. */
#else
#define LL_COMP_WINDOW_INPUT_PLUS_ODD   (COMP_CSR_INPSEL_0 | COMP_CSR_INPSEL_1 | COMP_CSR_INPSEL_2 | \
                                         LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK) /*!< Window mode
                                        enable: comparators instances pair have their input plus connected together.
                                        The common input is the one of instance index odd (COMP1, ...).
                                        Input plus of the other comparator is no more accessible. */
#define LL_COMP_WINDOW_INPUT_PLUS_EVEN  (COMP_CSR_INPSEL_0 | COMP_CSR_INPSEL_1 | COMP_CSR_INPSEL_2 | \
                                         LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK) /*!< Window mode
                                        enable: comparators instances pair have their input plus connected together.
                                        The common input is the one of instance index even (COMP2, ...).
                                        Input plus of the other comparator is no more accessible. */
#endif /* COMP_CFGR1_WINMODE */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_COMMON_WINDOWOUTPUT Comparator common modes - Window output
  * @{
  */
#define LL_COMP_WINDOW_OUTPUT_INDEPT    (0x00000000UL)/*!< Comparators window output default mode:
                                        both comparators output are independent, indicating each their own state.
                                        Note: To know signal state versus window thresholds, read each comparator
                                              output and perform a logical "exclusive or" operation. */
#if defined(COMP_CFGR1_WINMODE)
#define LL_COMP_WINDOW_OUTPUT_XOR_ODD   (COMP_CFGR1_WINOUT | LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK)  /*!< Window output
                                        synthesized on COMP1 output: COMP1 output is no more indicating its own state,
                                        but global window mode state. Logical high means monitored signal is within
                                        comparators window.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#define LL_COMP_WINDOW_OUTPUT_XOR_EVEN  (COMP_CFGR1_WINOUT | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK) /*!< Window output
                                        synthesized on COMP2 output: COMP2 output is no more indicating its own state,
                                        but global window mode state. Logical high means monitored signal is within
                                        comparators window.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#define LL_COMP_WINDOW_OUTPUT_XOR_BOTH  (COMP_CFGR1_WINOUT | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK \
                                         | LL_COMP_WINDOW_OUT_XOR_BOTH_MASK) /*!< Window output synthesized on both
                                        comparators output of pair of comparator selected (COMP1 and COMP2):
                                        both comparators outputs are no more indicating their own state, but global
                                        window mode state (XOR: logical "exclusive or"). Logical high means monitored
                                        signal is within comparators window thresholds).
                                        This is a specific configuration (technically possible but not relevant from
                                        application point of view: 2 comparators output used for the same signal level),
                                        standard configuration for window mode is one of the settings above.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#else
#define LL_COMP_WINDOW_OUTPUT_XOR_ODD   (COMP_CSR_WINOUT | LL_COMP_WINDOW_COMP_ODD_REGOFFSET_MASK)  /*!< Window output
                                        synthesized on COMP1 output: COMP1 output is no more indicating its own state,
                                        but global window mode state. Logical high means monitored signal is within
                                        comparators window.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#define LL_COMP_WINDOW_OUTPUT_XOR_EVEN  (COMP_CSR_WINOUT | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK) /*!< Window output
                                        synthesized on COMP2 output: COMP2 output is no more indicating its own state,
                                        but global window mode state. Logical high means monitored signal is within
                                        comparators window.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#define LL_COMP_WINDOW_OUTPUT_XOR_BOTH  (COMP_CSR_WINOUT | LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK \
                                         | LL_COMP_WINDOW_OUT_XOR_BOTH_MASK) /*!< Window output synthesized on both
                                        comparators output of pair of comparator selected (COMP1 and COMP2):
                                        both comparators outputs are no more indicating their own state, but global
                                        window mode state (XOR: logical "exclusive or"). Logical high means monitored
                                        signal is within comparators window thresholds).
                                        This is a specific configuration (technically possible but not relevant from
                                        application point of view: 2 comparators output used for the same signal level),
                                        standard configuration for window mode is one of the settings above.
                                        Note: impacts only comparator output signal level (propagated to GPIO, EXTI
                                              lines, timers, ...), does not impact output digital state
                                              (@ref LL_COMP_ReadOutputLevel()) always reflecting each comparator
                                              output state.*/
#endif /* COMP_CFGR1_WINMODE */
/**
  * @}
  */

#endif /* COMP_WINDOW_MODE_SUPPORT */
/** @defgroup COMP_LL_EC_POWER_MODE Comparator modes - Power mode
  * @{
  */
#define LL_COMP_POWER_MODE_HIGH_SPEED   (0x00000000UL)         /*!< Comparator power mode to high speed */
#define LL_COMP_POWER_MODE_MEDIUM_SPEED (COMP_CFGR1_PWRMODE_0) /*!< Comparator power mode to medium speed */
#define LL_COMP_POWER_MODE_ULTRA_LOW_POWER (COMP_CFGR1_PWRMODE_1 \
                                            | COMP_CFGR1_PWRMODE_0) /*!< Comparator power mode to ultra-low power */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_POWER_MODE_LEGACY Comparator modes - Power mode legacy definitions
  * @{
  */
#define LL_COMP_POWERMODE_HIGHSPEED     LL_COMP_POWER_MODE_HIGH_SPEED
#define LL_COMP_POWERMODE_MEDIUMSPEED   LL_COMP_POWER_MODE_MEDIUM_SPEED
#define LL_COMP_POWERMODE_ULTRALOWPOWER LL_COMP_POWER_MODE_ULTRA_LOW_POWER
/**
  * @}
  */

/** @defgroup COMP_LL_EC_INPUT_PLUS Comparator inputs - Input plus (input non-inverting) selection
  * @{
  */
#define LL_COMP_INPUT_PLUS_IO1          (0x00000000UL)       /*!< Comparator input plus connected to IO1
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP1"). */
#define LL_COMP_INPUT_PLUS_IO2          (COMP_CFGR1_INPSEL_0) /*!< Comparator input plus connected to IO2
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP2"). */
#define LL_COMP_INPUT_PLUS_IO3          (COMP_CFGR1_INPSEL_1) /*!< Comparator input plus connected to IO3
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INP3"). */
#define LL_COMP_INPUT_PLUS_DAC1_CH1     (COMP_CFGR1_INPSEL_1 | COMP_CFGR1_INPSEL_0) /*!< Comparator input plus
                                        connected to DAC1 channel 1.
                                        Specific to COMP instance: COMP1. */
#if defined(COMP2)
#define LL_COMP_INPUT_PLUS_DAC1_CH2     (COMP_CFGR1_INPSEL_1 | COMP_CFGR1_INPSEL_0) /*!< Comparator input plus
                                        connected to DAC1 channel 2.
                                        Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx). */
#endif /* COMP2 */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_INPUT_MINUS Comparator inputs - Input minus (input inverting) selection
  * @{
  */
#define LL_COMP_INPUT_MINUS_VREFINT     (COMP_CFGR1_INMSEL_1 | COMP_CFGR1_INMSEL_0 \
                                         | COMP_CFGR1_SCALEN) /*!< Comparator input minus connected to VrefInt
                                        (for VrefInt voltage value, refer to the datasheet). */
#define LL_COMP_INPUT_MINUS_1_4VREFINT  (COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN) /*!< Comparator input minus connected
                                        to 1/4 VrefInt (for VrefInt voltage value, refer to the datasheet). */
#define LL_COMP_INPUT_MINUS_1_2VREFINT  (COMP_CFGR1_INMSEL_0 | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN) /*!< Comparator
                                        input minus connected to 1/2 VrefInt (for VrefInt voltage value, refer to
                                        the datasheet). */
#define LL_COMP_INPUT_MINUS_3_4VREFINT  (COMP_CFGR1_INMSEL_1 | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN) /*!< Comparator
                                        input minus connected to 3/4 VrefInt (for VrefInt voltage value, refer to
                                        the datasheet). */
#define LL_COMP_INPUT_MINUS_IO1         (COMP_CFGR1_INMSEL_2 | COMP_CFGR1_INMSEL_0) /*!< Comparator
                                        input minus connected to IO1 (for GPIO mapping, refer to the datasheet
                                        parameters "COMPx_INM1"). */
#define LL_COMP_INPUT_MINUS_IO2         (COMP_CFGR1_INMSEL_2 | COMP_CFGR1_INMSEL_1) /*!< Comparator
                                        input minus connected to IO2 (for GPIO mapping, refer to the datasheet
                                        parameters "COMPx_INM2"). */
#define LL_COMP_INPUT_MINUS_IO3         (COMP_CFGR1_INMSEL_2 | COMP_CFGR1_INMSEL_1 \
                                         | COMP_CFGR1_INMSEL_0) /*!< Comparator input minus connected to IO3
                                        (for GPIO mapping, refer to the datasheet parameters "COMPx_INM3"). */
#define LL_COMP_INPUT_MINUS_DAC1_CH1    (COMP_CFGR1_INMSEL_2) /*!< Comparator input minus connected
                                        to DAC1 channel 1.
                                        Specific to COMP instances: COMP1. */
#if defined(COMP2)
#define LL_COMP_INPUT_MINUS_DAC1_CH2    (COMP_CFGR1_INMSEL_2) /*!< Comparator input minus connected
                                        to DAC1 channel 2.
                                        Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx). */
#define LL_COMP_INPUT_MINUS_OPAMP1_OUT  (COMP_CFGR1_INMSEL_3) /*!< Comparator input minus connected
                                        to OPAMP1 output.
                                        Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx). */
#endif /* COMP2 */
#define LL_COMP_INPUT_MINUS_TEMPSENSOR  (COMP_CFGR1_INMSEL_3) /*!< Comparator input minus connected
                                        to the internal temperature sensor (also accessible through the ADC
                                        peripheral).
                                        Call function @ref LL_COMP_EnableInputTempSensorBuffer(). */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_INPUT_HYSTERESIS Comparator input - Hysteresis
  * @{
  */
#define LL_COMP_HYSTERESIS_NONE         (0x00000000UL)                          /*!< No hysteresis */
#define LL_COMP_HYSTERESIS_LOW          (COMP_CFGR1_HYST_0)                     /*!< Hysteresis level low */
#define LL_COMP_HYSTERESIS_MEDIUM       (COMP_CFGR1_HYST_1)                     /*!< Hysteresis level medium */
#define LL_COMP_HYSTERESIS_HIGH         (COMP_CFGR1_HYST_1 | COMP_CFGR1_HYST_0) /*!< Hysteresis level high */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_OUTPUT_POLARITY Comparator output - Output polarity
  * @{
  */
#define LL_COMP_OUTPUTPOL_NONINVERTED   (0x00000000UL) /*!< Comparator output polarity not inverted:
                                        comparator output at high level when input voltages: plus higher than minus */
#define LL_COMP_OUTPUTPOL_INVERTED      (COMP_CFGR1_POLARITY) /*!< Comparator output polarity not inverted:
                                        comparator output at low level when input voltages: plus higher than minus */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_OUTPUT_BLANKING_SOURCE Comparator output - Blanking source
  * @{
  */
#define LL_COMP_BLANKINGSRC_NONE        (0x00000000UL)        /*!< Comparator output without blanking */
#define LL_COMP_BLANKINGSRC_TIM1_OC5    (COMP_CFGR1_BLANKING_0)   /*!< Comparator output blanking source TIM1 OC5,
                                        specific to instance: COMP1. */
#define LL_COMP_BLANKINGSRC_TIM1_OC6    (COMP_CFGR1_BLANKING_0)   /*!< Comparator output blanking source TIM1 OC6,
                                        specific to instance: COMP2. */
#define LL_COMP_BLANKINGSRC_TIM2_OC3    (COMP_CFGR1_BLANKING_1)   /*!< Comparator output blanking source TIM2 OC3. */
#define LL_COMP_BLANKINGSRC_TIM5_OC3    (COMP_CFGR1_BLANKING_0 \
                                         | COMP_CFGR1_BLANKING_1) /*!< Comparator output blanking source TIM5 OC3. */
#define LL_COMP_BLANKINGSRC_TIM5_OC4    (COMP_CFGR1_BLANKING_2)   /*!< Comparator output blanking source TIM5 OC4. */
#define LL_COMP_BLANKINGSRC_TIM8_OC5    (COMP_CFGR1_BLANKING_2 \
                                         | COMP_CFGR1_BLANKING_0) /*!< Comparator output blanking source TIM8 OC5. */
#define LL_COMP_BLANKINGSRC_TIM15_OC2   (COMP_CFGR1_BLANKING_2 \
                                         | COMP_CFGR1_BLANKING_1) /*!< Comparator output blanking source TIM15 OC2. */
#define LL_COMP_BLANKINGSRC_LPTIM1_OC1  (COMP_CFGR1_BLANKING_2 \
                                         | COMP_CFGR1_BLANKING_1 \
                                         | COMP_CFGR1_BLANKING_0) /*!< Comparator output blanking source LPTIM1 OC1,
                                        specific to instance: COMP2. */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_OUTPUT_LEVEL Comparator output - Output level
  * @{
  */
#define LL_COMP_OUTPUT_LEVEL_LOW        (0x00000000UL) /*!< Comparator output level low (with polarity not inverted) */
#define LL_COMP_OUTPUT_LEVEL_HIGH       (0x00000001UL) /*!< Comparator output level high (with polarity not inverted) */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_HW_DELAYS  Definitions of COMP hardware constraints delays
  * @note   Only COMP peripheral HW delays are defined in COMP LL driver driver, not timeout values.
  * @{
  */

/* Delay for comparator startup time.                                         */
/* Note: Delay required to reach propagation delay specification.             */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART").                                                       */
/* Unit: us                                                                   */
#define LL_COMP_DELAY_STARTUP_US        (80UL) /*!< Delay for comparator startup time.
                                        Delay set to maximum value (refer to device datasheet, parameter "tSTART").
                                        Unit: us.
                                        Note: At comparator enable, delay required to reach propagation delay
                                              specification. */

/* Delay for comparator voltage scaler stabilization time.                    */
/* Note: Voltage scaler is used when selecting comparator input               */
/*       based on VrefInt (VrefInt or subdivision of VrefInt).                */
/* Note: To get scaler bridge configuration,                                  */
/*       refer to @ref LL_COMP_IsInputScalerEnabled().                        */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART_SCALER").                                                */
/* Unit: us                                                                   */
#define LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US (220UL) /*!< Delay for comparator voltage scaler stabilization time
                                        Delay set to maximum value (refer to device datasheet,
                                        parameter "tSTART_SCALER").
                                        Unit: us.
                                        Note: Voltage scaler is used when selecting comparator input
                                              based on VrefInt (VrefInt or subdivision of VrefInt).
                                        Note: To get scaler bridge configuration,
                                              refer to @ref LL_COMP_IsInputScalerEnabled().
                                        */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Macros LL COMP Macros
  * @{
  */
/** @defgroup COMP_LL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
  * @brief  Write a value in COMP register.
  * @param  __INSTANCE__ comparator instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  */
#define LL_COMP_WRITE_REG(__INSTANCE__, __REG__, __VALUE__) STM32_WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in COMP register.
  * @param  __INSTANCE__ comparator instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_COMP_READ_REG(__INSTANCE__, __REG__) STM32_READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/** @defgroup COMP_LL_EM_HELPER_MACRO COMP helper macro
  * @{
  */

/**
  * @brief  Helper macro to select the COMP common instance
  *         to which is belonging the selected COMP instance.
  * @param  instance COMP instance
  * @note   COMP common register instance can be used to
  *         set parameters common to several COMP instances.
  *         Refer to functions having argument "p_comp_common" as parameter.
  * @retval COMP common instance or value "0" if there is no COMP common instance.
  */
#define LL_COMP_COMMON_INSTANCE(instance) (COMP12_COMMON)

/**
  * @brief  Helper macro to define comparator instance position generic identification (odd or even)
  *         from comparator instance.
  * @param  instance Comparator instance.
  * @retval Comparator common instance or value "0" if there is no COMP common instance.
  */
#define LL_COMP_WINDOW_INST_POS_ID(instance)                                   \
  ((((instance) == COMP1))                                                     \
   ? ((LL_COMP_WINDOW_COMP_ODD))                                               \
   :                                                                           \
   ((LL_COMP_WINDOW_COMP_EVEN))                                                \
  )

/**
  * @brief  Helper macro to select literal LL_COMP_WINDOW_INPUT_PLUS_x with suffix odd or even
  *         from comparator instance.
  * @param  instance COMP instance.
  * @note   Helper macro intended to be used with function LL_COMP_SetCommonWindowMode()
  * @retval COMP common instance or value "0" if there is no COMP common instance.
  */
#define LL_COMP_WINDOW_INST_TO_INPUT_PLUS(instance)                           \
  (((LL_COMP_WINDOW_INST_POS_ID(instance) == LL_COMP_WINDOW_COMP_ODD))        \
   ? ((LL_COMP_WINDOW_INPUT_PLUS_ODD))                                        \
   :                                                                          \
   ((LL_COMP_WINDOW_INPUT_PLUS_EVEN))                                         \
  )

/**
  * @brief  Helper macro to select literal LL_COMP_WINDOW_OUTPUT_x with suffix odd or even
  *         from comparator instance.
  * @param  instance COMP instance.
  * @note   Helper macro intended to be used with function LL_COMP_SetCommonWindowOutput()
  * @retval COMP common instance or value "0" if there is no COMP common instance.
  */
#define LL_COMP_WINDOW_INST_TO_OUTPUT(instance)                               \
  (((LL_COMP_WINDOW_INST_POS_ID(instance) == LL_COMP_WINDOW_COMP_ODD))        \
   ? ((LL_COMP_WINDOW_OUTPUT_XOR_ODD))                                        \
   :                                                                          \
   ((LL_COMP_WINDOW_OUTPUT_XOR_EVEN))                                         \
  )
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Functions LL COMP Functions
  * @{
  */

#if defined(COMP_WINDOW_MODE_SUPPORT)
/** @defgroup COMP_LL_EF_Configuration_comparator_common Configuration of COMP hierarchical scope:
  *           common to several COMP instances
  * @{
  */

/**
  * @brief  Set window mode of a pair of comparators instances
  *         (2 consecutive COMP instances COMP<x> and COMP<x+1>).
  * @rmtoll
  *  CFGR1    WINMODE        LL_COMP_SetCommonWindowMode
  * @param  p_comp_common Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref LL_COMP_COMMON_INSTANCE() )
  * @param  window_mode This parameter can be one of the following values:
  *         @arg @ref LL_COMP_WINDOW_DISABLE
  *         @arg @ref LL_COMP_WINDOW_INPUT_PLUS_ODD
  *         @arg @ref LL_COMP_WINDOW_INPUT_PLUS_EVEN
  *         Note: Parameters values with suffix odd or even can also be selected from comparator instance
  *               using helper macro LL_COMP_WINDOW_INST_TO_INPUT_PLUS.
  */
__STATIC_INLINE void LL_COMP_SetCommonWindowMode(COMP_Common_TypeDef *p_comp_common, uint32_t window_mode)
{
  /* Note: On this STM32 series, window mode can be set from any instance     */
  /*       of the pair of comparator instances.                               */

#if defined(COMP_CFGR1_WINMODE)
  __IO uint32_t *preg = LL_COMP_PTR_REG_OFFSET(p_comp_common->CFGR1,
                                               (window_mode & LL_COMP_WINDOW_COMP_REGOFFSET_MASK));

  /* Clear the potential previous setting of window mode */
  __IO uint32_t *preg_clear = LL_COMP_PTR_REG_OFFSET(p_comp_common->CFGR1,
                                                     (~(window_mode & LL_COMP_WINDOW_COMP_REGOFFSET_MASK) & 0x1UL)
                                                    );
  STM32_CLEAR_BIT(*preg_clear, COMP_CFGR1_WINMODE);

  /* Set window mode */
  STM32_MODIFY_REG(*preg, COMP_CFGR1_WINMODE, (window_mode & LL_COMP_WINDOW_COMP_X_SETTING_MASK));
#else
  __IO uint32_t *preg = LL_COMP_PTR_REG_OFFSET(p_comp_common->CSR_ODD,
                                               (window_mode & LL_COMP_WINDOW_COMP_REGOFFSET_MASK));

  /* Set window mode */
  STM32_MODIFY_REG(*preg, COMP_CSR_INPSEL, (window_mode & LL_COMP_WINDOW_COMP_X_SETTING_MASK));
#endif /* COMP_CFGR1_WINMODE */

}

/**
  * @brief  Get window mode of a pair of comparators instances
  *         (2 consecutive COMP instances COMP<x> and COMP<x+1>).
  * @rmtoll
  *  CFGR1    WINMODE        LL_COMP_GetCommonWindowMode
  * @param  p_comp_common Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref LL_COMP_COMMON_INSTANCE() ).
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_WINDOW_DISABLE
  *         @arg @ref LL_COMP_WINDOW_INPUT_PLUS_ODD
  *         @arg @ref LL_COMP_WINDOW_INPUT_PLUS_EVEN
  */
__STATIC_INLINE uint32_t LL_COMP_GetCommonWindowMode(const COMP_Common_TypeDef *p_comp_common)
{
  /* Note: On this STM32 series, window mode can be set from any instance      */
  /*       of the pair of comparator instances.                               */

#if defined(COMP_CFGR1_WINMODE)
  const uint32_t window_mode_comp_odd = (uint32_t)STM32_READ_BIT(p_comp_common->CFGR1, COMP_CFGR1_WINMODE);
  const uint32_t window_mode_comp_even = (uint32_t)STM32_READ_BIT(p_comp_common->CFGR2, COMP_CFGR1_WINMODE);

  return (uint32_t)(window_mode_comp_odd
                    | window_mode_comp_even
                    | (window_mode_comp_even >> COMP_CFGR1_WINMODE_Pos)
                   );
#else
  const uint32_t window_mode_comp_odd = (uint32_t)STM32_READ_BIT(p_comp_common->CSR_ODD, COMP_CSR_INPSEL);
  const uint32_t window_mode_comp_even = (uint32_t)STM32_READ_BIT(p_comp_common->CSR_EVEN, COMP_CSR_INPSEL);

  return (uint32_t)(window_mode_comp_odd
                    | window_mode_comp_even
                    | ((window_mode_comp_even >> COMP_CSR_INPSEL_Pos) & LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK)
                   );
#endif /* COMP_CFGR1_WINMODE */
}

/**
  * @brief  Set window output of a pair of comparators instances
  *         (2 consecutive COMP instances COMP<x> and COMP<x+1>).
  * @rmtoll
  *  CFGR1    WINOUT         LL_COMP_SetCommonWindowOutput
  * @param  p_comp_common Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref LL_COMP_COMMON_INSTANCE() ).
  * @param  window_output This parameter can be one of the following values:
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_INDEPT
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_ODD
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_EVEN
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_BOTH
  *         Note: Parameters values with suffix odd or even can also be selected from comparator instance
  *               using helper LL_COMP_WINDOW_INST_TO_OUTPUT.
  */
__STATIC_INLINE void LL_COMP_SetCommonWindowOutput(COMP_Common_TypeDef *p_comp_common, uint32_t window_output)
{
#if defined(COMP_CFGR1_WINMODE)
  __IO uint32_t *preg = LL_COMP_PTR_REG_OFFSET(p_comp_common->CFGR1,
                                               (window_output & LL_COMP_WINDOW_COMP_REGOFFSET_MASK));

  /* Clear the potential previous setting of window output on the relevant comparator instance */
  /* (clear bit of window output unless specific case of setting of comparator both output selected) */
  __IO uint32_t *preg_clear = LL_COMP_PTR_REG_OFFSET(p_comp_common->CFGR1,
                                                     (~(window_output & LL_COMP_WINDOW_COMP_REGOFFSET_MASK) & 0x1UL)
                                                    );
  STM32_MODIFY_REG(*preg_clear,
                   COMP_CFGR1_WINOUT,
                   ((window_output & LL_COMP_WINDOW_OUT_XOR_BOTH_MASK)
                    >> LL_COMP_WINDOW_OUT_XOR_BOTH_POS_VS_WINDOW)
                  );

  /* Set window output */
  STM32_MODIFY_REG(*preg,
                   COMP_CFGR1_WINOUT,
                   (window_output & LL_COMP_WINDOW_OUT_SETTING_MASK)
                  );
#else
  /* Note: On this STM32 series, window mode can be set from any instance     */
  /*       of the pair of comparator instances.                               */

  __IO uint32_t *preg = LL_COMP_PTR_REG_OFFSET(p_comp_common->CSR_ODD,
                                               (window_output & LL_COMP_WINDOW_COMP_REGOFFSET_MASK));

  /* Clear the potential previous setting of window output on the relevant comparator instance */
  /* (clear bit of window output unless specific case of setting of comparator both output selected) */
  __IO uint32_t *preg_clear = LL_COMP_PTR_REG_OFFSET(p_comp_common->CSR_ODD,
                                                     ((~(window_output & LL_COMP_WINDOW_COMP_REGOFFSET_MASK)
                                                       & 0x1UL))
                                                    );
  STM32_MODIFY_REG(*preg_clear,
                   COMP_CSR_WINOUT,
                   ((window_output & LL_COMP_WINDOW_OUT_XOR_BOTH_MASK)
                    >> LL_COMP_WINDOW_OUT_XOR_BOTH_POS_VS_WINDOW)
                  );

  /* Set window output */
  STM32_MODIFY_REG(*preg,
                   COMP_CSR_WINOUT,
                   (window_output & LL_COMP_WINDOW_OUT_SETTING_MASK)
                  );
#endif /* COMP_CFGR1_WINMODE */
}

/**
  * @brief  Get window output of a pair of comparators instances
  *         (2 consecutive COMP instances COMP<x> and COMP<x+1>).
  * @rmtoll
  *  CFGR1    WINOUT         LL_COMP_GetCommonWindowOutput
  * @param  p_comp_common Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref LL_COMP_COMMON_INSTANCE() ).
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_INDEPT
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_ODD
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_EVEN
  *         @arg @ref LL_COMP_WINDOW_OUTPUT_XOR_BOTH
  */
__STATIC_INLINE uint32_t LL_COMP_GetCommonWindowOutput(const COMP_Common_TypeDef *p_comp_common)
{
#if defined(COMP_CFGR1_WINMODE)
  const uint32_t window_output_comp_odd = (uint32_t)STM32_READ_BIT(p_comp_common->CFGR1, COMP_CFGR1_WINOUT);
  const uint32_t window_output_comp_even = (uint32_t)STM32_READ_BIT(p_comp_common->CFGR2, COMP_CFGR1_WINOUT);

  /* Construct value corresponding to LL_COMP_WINDOWOUTPUT_xxx */
  return (uint32_t)(window_output_comp_odd
                    | window_output_comp_even
                    | ((window_output_comp_even >> COMP_CFGR1_WINOUT_Pos) * LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK)
                    | (window_output_comp_odd + window_output_comp_even));

#else
  const uint32_t window_output_comp_odd = (uint32_t)STM32_READ_BIT(p_comp_common->CSR_ODD, COMP_CSR_WINOUT);
  const uint32_t window_output_comp_even = (uint32_t)STM32_READ_BIT(p_comp_common->CSR_EVEN, COMP_CSR_WINOUT);

  /* Construct value corresponding to LL_COMP_WINDOWOUTPUT_xxx */
  return (uint32_t)(window_output_comp_odd
                    | window_output_comp_even
                    | ((window_output_comp_even >> COMP_CSR_WINOUT_Pos) * LL_COMP_WINDOW_COMP_EVEN_REGOFFSET_MASK)
                    | (window_output_comp_odd + window_output_comp_even));
#endif /* COMP_CFGR1_WINMODE */
}
/**
  * @}
  */

#endif /* COMP_WINDOW_MODE_SUPPORT */

/** @defgroup COMP_LL_EF_Configuration_comparator_modes Configuration of comparator modes
  * @{
  */

/**
  * @brief  Set comparator instance operating mode to adjust power and speed.
  * @rmtoll
  *  CFGR1    PWRMODE        LL_COMP_SetPowerMode
  * @param  p_comp Comparator instance.
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_COMP_POWER_MODE_HIGH_SPEED
  *         @arg @ref LL_COMP_POWER_MODE_MEDIUM_SPEED
  *         @arg @ref LL_COMP_POWER_MODE_ULTRA_LOW_POWER
  */
__STATIC_INLINE void LL_COMP_SetPowerMode(COMP_TypeDef *p_comp, uint32_t power_mode)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_PWRMODE, power_mode);
}

/**
  * @brief  Get comparator instance operating mode to adjust power and speed.
  * @rmtoll
  *  CFGR1    PWRMODE        LL_COMP_GetPowerMode
  * @param  p_comp Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_POWER_MODE_HIGH_SPEED
  *         @arg @ref LL_COMP_POWER_MODE_MEDIUM_SPEED
  *         @arg @ref LL_COMP_POWER_MODE_ULTRA_LOW_POWER
  */
__STATIC_INLINE uint32_t LL_COMP_GetPowerMode(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_PWRMODE));
}

/**
  * @}
  */

/** @defgroup COMP_LL_EF_Configuration_comparator_inputs Configuration of comparator inputs
  * @{
  */

/**
  * @brief  Set comparator inputs minus (inverting) and plus (non-inverting).
  * @rmtoll
  *  CFGR1    INPSEL         LL_COMP_ConfigInputs \n
  *  CFGR1    INMSEL         LL_COMP_ConfigInputs \n
  *  CFGR1    BRGEN          LL_COMP_ConfigInputs \n
  *  CFGR1    SCALEN         LL_COMP_ConfigInputs
  * @param  p_comp Comparator instance
  * @param  input_minus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_1_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_1_2VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_3_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  *         @arg @ref LL_COMP_INPUT_MINUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH1   (1)
  *         @arg @ref LL_COMP_INPUT_MINUS_TEMPSENSOR (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH2   (2)
  *         @arg @ref LL_COMP_INPUT_MINUS_OPAMP1_OUT (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH1 (3)
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH2 (3)(4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  * @param  input_plus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  *         @arg @ref LL_COMP_INPUT_PLUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH1 (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH2 (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP1_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP2_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP3_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH1 (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH2 (4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @note   Voltage scaler is used when selecting comparator input based on VrefInt (VrefInt or subdivision of VrefInt).
  *         In this case, specific delay must be fulfilled for voltage stabilization when enabling comparator,
  *         refer to LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US.
  * @note   On this STM32 series, a voltage scaler is used
  *         when COMP input is based on VrefInt (VrefInt or subdivision
  *         of VrefInt):
  *         Voltage scaler requires a delay for voltage stabilization.
  *         Refer to device datasheet, parameter "tSTART_SCALER".
  */
__STATIC_INLINE void LL_COMP_ConfigInputs(COMP_TypeDef *p_comp, uint32_t input_minus, uint32_t input_plus)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp),
                   COMP_CFGR1_INMSEL | COMP_CFGR1_INPSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN,
                   input_minus | input_plus);
}

/**
  * @brief  Set comparator input plus.
  * @rmtoll
  *  CFGR1    INPSEL         LL_COMP_SetInputPlus
  * @param  p_comp Comparator instance
  * @param  input_plus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  *         @arg @ref LL_COMP_INPUT_PLUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH1 (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH2 (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP1_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP2_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP3_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH1 (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH2 (4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  */
__STATIC_INLINE void LL_COMP_SetInputPlus(COMP_TypeDef *p_comp, uint32_t input_plus)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_INPSEL, input_plus);
}

/**
  * @brief  Get comparator input plus.
  * @rmtoll
  *  CFGR1    INPSEL         LL_COMP_GetInputPlus
  * @param  p_comp Comparator instance
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  *         @arg @ref LL_COMP_INPUT_PLUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH1 (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC1_CH2 (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP1_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP2_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_OPAMP3_OUT (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH1 (3)(4)
  *         @arg @ref LL_COMP_INPUT_PLUS_DAC2_CH2 (4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  */
__STATIC_INLINE uint32_t LL_COMP_GetInputPlus(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_INPSEL));
}

/**
  * @brief  Set comparator input minus.
  * @rmtoll
  *  CFGR1    INMSEL         LL_COMP_SetInputMinus \n
  *  CFGR1    BRGEN          LL_COMP_SetInputMinus \n
  *  CFGR1    SCALEN         LL_COMP_SetInputMinus
  * @param  p_comp Comparator instance
  * @param  input_minus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_1_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_1_2VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_3_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  *         @arg @ref LL_COMP_INPUT_MINUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH1   (1)
  *         @arg @ref LL_COMP_INPUT_MINUS_TEMPSENSOR (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH2   (2)
  *         @arg @ref LL_COMP_INPUT_MINUS_OPAMP1_OUT (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH1 (3)
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH2 (3)(4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @note   Voltage scaler is used when selecting comparator input based on VrefInt (VrefInt or subdivision of VrefInt).
  *         In this case, specific delay must be fulfilled for voltage stabilization when enabling comparator,
  *         refer to LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US.
  */
__STATIC_INLINE void LL_COMP_SetInputMinus(COMP_TypeDef *p_comp, uint32_t input_minus)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_INMSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN, input_minus);
}

/**
  * @brief  Get comparator input minus.
  * @rmtoll
  *  CFGR1    INMSEL         LL_COMP_GetInputMinus \n
  *  CFGR1    BRGEN          LL_COMP_GetInputMinus \n
  *  CFGR1    SCALEN         LL_COMP_GetInputMinus
  * @param  p_comp Comparator instance
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_1_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_1_2VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_3_4VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_VREFINT
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  *         @arg @ref LL_COMP_INPUT_MINUS_IO3
  * @if COMP1
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH1   (1)
  *         @arg @ref LL_COMP_INPUT_MINUS_TEMPSENSOR (1)
  * @endif
  * @if COMP2
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC1_CH2   (2)
  *         @arg @ref LL_COMP_INPUT_MINUS_OPAMP1_OUT (2)
  * @endif
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH1 (3)
  *         @arg @ref LL_COMP_INPUT_MINUS_DAC2_CH2 (3)(4)
  *
  *         (1) Specific to COMP instance: COMP1
  *         (2) Specific to COMP instance: COMP2 (available on devices STM32C53xx/54xx)
  *         (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         (4) Parameters only available on STM32C5P1xx/5Q1xx devices

  */
__STATIC_INLINE uint32_t LL_COMP_GetInputMinus(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_INMSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN));
}

/**
  * @brief  Get comparator input voltage scaler bridge configuration.
  * @rmtoll
  *  CFGR1    BRGEN          LL_COMP_IsInputScalerEnabled \n
  *  CFGR1    SCALEN         LL_COMP_IsInputScalerEnabled
  * @param  p_comp Comparator instance
  * @note   Voltage scaler is used when selecting comparator input based on VrefInt (VrefInt or subdivision of VrefInt).
  *         In this case, specific delay must be fulfilled for voltage stabilization when enabling comparator,
  *         refer to LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US.
  * @retval State of scaler bridge configuration (value "1" for enabled, value "0" for disabled).
  */
__STATIC_INLINE uint32_t LL_COMP_IsInputScalerEnabled(const COMP_TypeDef *p_comp)
{
  return ((STM32_READ_BIT(LL_COMP_REG(p_comp), (COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN)) != 0UL) ? 1UL : 0UL);
}

/**
  * @brief  Enable comparator input temperature sensor buffer.
  * @rmtoll
  *  RCC_AHB2ENR AHB2ENR     LL_COMP_EnableInputTempSensorBuffer \n
  *  ADC_CCR     TSEN        LL_COMP_EnableInputTempSensorBuffer
  * @param  p_comp Comparator instance
  * @note   Specific configuration with bitfields out of comparator registers
  *         due to temperature sensor buffer controlled by ADC clock domain.
  *         Caution: ADC clock domain reset or disable impacts temperature sensor.
  * @note   Temperature sensor stabilization delay must be fulfilled,
  *         refer to LL_ADC_DELAY_TEMPSENSOR_STAB_US (in ADC LL driver) or device datasheet.
  *         Delay encompassed in LL_COMP_DELAY_STARTUP_US and LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US,
  *         therefore no additional delay required for temperature sensor buffer.
  */
__STATIC_INLINE void LL_COMP_EnableInputTempSensorBuffer(COMP_TypeDef *p_comp)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(p_comp);

  STM32_SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_ADC12EN);
  STM32_SET_BIT(ADC12_COMMON->CCR, ADCC_CCR_TSEN);
}

/**
  * @brief  Disable comparator input temperature sensor buffer.
  * @rmtoll
  *  ADC_CCR     TSEN        LL_COMP_DisableInputTempSensorBuffer
  * @param  p_comp Comparator instance
  * @note   Specific configuration with bitfields out of comparator registers
  *         due to temperature sensor buffer controlled by ADC clock domain.
  *         Clock domain of ADC not reset to not interfere with potential ADC operation.
  *         Caution: Temperature sensor buffer disable must not be performed if ADC expected
  *         to convert channel temperature sensor.
  */
__STATIC_INLINE void LL_COMP_DisableInputTempSensorBuffer(COMP_TypeDef *p_comp)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(p_comp);

  STM32_CLEAR_BIT(ADC12_COMMON->CCR, ADCC_CCR_TSEN);
}

/**
  * @brief  Get comparator input temperature sensor buffer configuration.
  * @rmtoll
  *  ADC_CCR     TSEN        LL_COMP_IsInputTempSensorBufferEnabled
  * @param  p_comp Comparator instance
  * @note   Specific configuration with bitfields out of comparator registers
  *         due to temperature sensor buffer controlled by ADC clock domain.
  */
__STATIC_INLINE uint32_t LL_COMP_IsInputTempSensorBufferEnabled(COMP_TypeDef *p_comp)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(p_comp);

  return ((STM32_READ_BIT(ADC12_COMMON->CCR, ADCC_CCR_TSEN) != 0UL) ? 1UL : 0UL);
}

/**
  * @brief  Set comparator input hysteresis.
  * @rmtoll
  *  CFGR1    HYST           LL_COMP_SetInputHysteresis
  * @param  p_comp Comparator instance
  * @param  input_hysteresis This parameter can be one of the following values:
  *         @arg @ref LL_COMP_HYSTERESIS_NONE
  *         @arg @ref LL_COMP_HYSTERESIS_LOW
  *         @arg @ref LL_COMP_HYSTERESIS_MEDIUM
  *         @arg @ref LL_COMP_HYSTERESIS_HIGH
  * @note   Hysteresys applied on comparator input minus.
  */
__STATIC_INLINE void LL_COMP_SetInputHysteresis(COMP_TypeDef *p_comp, uint32_t input_hysteresis)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_HYST, input_hysteresis);
}

/**
  * @brief  Get comparator instance hysteresis mode of the minus (inverting) input.
  * @rmtoll
  *  CFGR1    HYST           LL_COMP_GetInputHysteresis
  * @param  p_comp Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_HYSTERESIS_NONE
  *         @arg @ref LL_COMP_HYSTERESIS_LOW
  *         @arg @ref LL_COMP_HYSTERESIS_MEDIUM
  *         @arg @ref LL_COMP_HYSTERESIS_HIGH
  */
__STATIC_INLINE uint32_t LL_COMP_GetInputHysteresis(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_HYST));
}

/** @defgroup COMP_LL_EF_Configuration_comparator_output Configuration of comparator output
  * @{
  */

/**
  * @brief  Set comparator instance output polarity.
  * @rmtoll
  *  CFGR1    POLARITY       LL_COMP_SetOutputPolarity
  * @param  p_comp Comparator instance
  * @param  output_polarity This parameter can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUTPOL_NONINVERTED
  *         @arg @ref LL_COMP_OUTPUTPOL_INVERTED
  */
__STATIC_INLINE void LL_COMP_SetOutputPolarity(COMP_TypeDef *p_comp, uint32_t output_polarity)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_POLARITY, output_polarity);
}

/**
  * @brief  Get comparator instance output polarity.
  * @rmtoll
  *  CFGR1    POLARITY       LL_COMP_GetOutputPolarity
  * @param  p_comp Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUTPOL_NONINVERTED
  *         @arg @ref LL_COMP_OUTPUTPOL_INVERTED
  */
__STATIC_INLINE uint32_t LL_COMP_GetOutputPolarity(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_POLARITY));
}

/**
  * @brief  Set comparator instance blanking source.
  * @rmtoll
  *  CFGR     BLANKING       LL_COMP_SetOutputBlankingSource
  * @param  p_comp Comparator instance
  * @param  blanking_source This parameter can be one of the following values:
  *         @arg @ref LL_COMP_BLANKINGSRC_NONE
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC5 (1)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC5 (2)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM2_OC3
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM5_OC3
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM5_OC4
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM8_OC5
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM15_OC2
  *         @arg @ref LL_COMP_BLANKINGSRC_LPTIM1_OC1 (2)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM8_OC6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_PLAY_OUT6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM2_OC4 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM3_OC3 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM3_OC4 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_PLAY_OUT7 (3)
  *
  *         (1) On STM32C5, parameter available only on COMP instance: COMP1.
  *         (2) On STM32C5, parameter available only on COMP instance: COMP2.
  *         (3) On STM32C5, parameters available only on COMP instances: COMP3 and COMP4.

  * @note   Availability of parameters of blanking source from peripherals
  *         depends on their availability on the selected device.
  * @note   Blanking source can be specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  */
__STATIC_INLINE void LL_COMP_SetOutputBlankingSource(COMP_TypeDef *p_comp, uint32_t blanking_source)
{
  STM32_MODIFY_REG(LL_COMP_REG(p_comp), COMP_CFGR1_BLANKING, blanking_source);
}

/**
  * @brief  Get comparator instance blanking source.
  * @rmtoll
  *  CFGR     BLANKING       LL_COMP_GetOutputBlankingSource
  * @param  p_comp Comparator instance
  * @note   Availability of parameters of blanking source from peripherals
  *         depends on their availability on the selected device.
  * @note   Blanking source can be specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_BLANKINGSRC_NONE
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC5 (1)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC5 (2)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM2_OC3
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM5_OC3
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM5_OC4
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM8_OC5
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM15_OC2
  *         @arg @ref LL_COMP_BLANKINGSRC_LPTIM1_OC1 (2)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM8_OC6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_PLAY_OUT6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM1_OC6 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM2_OC4 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM3_OC3 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_TIM3_OC4 (3)
  *         @arg @ref LL_COMP_BLANKINGSRC_PLAY_OUT7 (3)
  *
  *         (1) On STM32C5, parameter available only on COMP instance: COMP1.
  *         (2) On STM32C5, parameter available only on COMP instance: COMP2.
  *         (3) On STM32C5, parameters available only on COMP instances: COMP3 and COMP4.

  */
__STATIC_INLINE uint32_t LL_COMP_GetOutputBlankingSource(const COMP_TypeDef *p_comp)
{
  return (uint32_t)(STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_BLANKING));
}

/**
  * @}
  */

/** @defgroup COMP_LL_EF_Operation Operation on comparator instance
  * @{
  */

/**
  * @brief  Enable comparator instance.
  * @rmtoll
  *  CFGR1    EN             LL_COMP_Enable
  * @param  p_comp Comparator instance
  * @note   After enable, comparator requires a delay to reach reach propagation delay specification,
  *         refer to LL_COMP_DELAY_STARTUP_US.
  * @note   Voltage scaler is used when selecting comparator input based on VrefInt (VrefInt or subdivision of VrefInt).
  *         In this case, specific delay must be fulfilled for voltage stabilization when enabling comparator,
  *         refer to LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US.
  *         To get scaler bridge configuration, refer to @ref LL_COMP_IsInputScalerEnabled().
  */
__STATIC_INLINE void LL_COMP_Enable(COMP_TypeDef *p_comp)
{
  STM32_SET_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_EN);
}

/**
  * @brief  Disable comparator instance.
  * @rmtoll
  *  CFGR1    EN             LL_COMP_Disable
  * @param  p_comp Comparator instance
  */
__STATIC_INLINE void LL_COMP_Disable(COMP_TypeDef *p_comp)
{
  STM32_CLEAR_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_EN);
}

/**
  * @brief  Get comparator enable state.
  * @rmtoll
  *  CFGR1    EN             LL_COMP_IsEnabled
  * @param  p_comp Comparator instance
  * @retval Value "0" for comparator disabled, value "1" for comparator enabled.
  */
__STATIC_INLINE uint32_t LL_COMP_IsEnabled(const COMP_TypeDef *p_comp)
{
  return ((STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_EN) == (COMP_CFGR1_EN)) ? 1UL : 0UL);
}

/**
  * @brief  Lock comparator instance.
  * @rmtoll
  *  CFGR1    LOCK           LL_COMP_Lock
  * @param  p_comp Comparator instance
  * @note   Once locked, comparator configuration can be accessed in read-only.
  * @note   The only way to unlock the comparator is a device system reset.
  */
__STATIC_INLINE void LL_COMP_Lock(COMP_TypeDef *p_comp)
{
  STM32_SET_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_LOCK);
}

/**
  * @brief  Get comparator lock state.
  * @rmtoll
  *  CFGR1    LOCK           LL_COMP_IsLocked
  * @param  p_comp Comparator instance
  * @note   Once locked, comparator configuration can be accessed in read-only.
  * @note   The only way to unlock the comparator is a device system reset.
  * @retval Value "0" for comparator unlocked, value "1" for comparator locked.
  */
__STATIC_INLINE uint32_t LL_COMP_IsLocked(const COMP_TypeDef *p_comp)
{
  return ((STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_LOCK) == (COMP_CFGR1_LOCK)) ? 1UL : 0UL);
}

/**
  * @brief  Read comparator instance output level.
  * @rmtoll
  *  SR       C1VAL          LL_COMP_ReadOutputLevel
  * @if COMP2
  *  SR       C2VAL          LL_COMP_ReadOutputLevel
  * @endif
  * @param  p_comp Comparator instance
  * @note   The comparator output level depends on the selected polarity
  *         (Refer to function @ref LL_COMP_SetOutputPolarity()).
  *         If the comparator polarity is not inverted:
  *          - Comparator output is low when the input plus
  *            is at a lower voltage than the input minus
  *          - Comparator output is high when the input plus
  *            is at a higher voltage than the input minus
  *         If the comparator polarity is inverted:
  *          - Comparator output is high when the input plus
  *            is at a lower voltage than the input minus
  *          - Comparator output is low when the input plus
  *            is at a higher voltage than the input minus
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUT_LEVEL_LOW
  *         @arg @ref LL_COMP_OUTPUT_LEVEL_HIGH
  */
__STATIC_INLINE uint32_t LL_COMP_ReadOutputLevel(const COMP_TypeDef *p_comp)
{
#if defined(COMP_CSR_VALUE)
  return (uint32_t)(STM32_READ_BIT(p_comp->CSR, COMP_CSR_VALUE) >> COMP_CSR_VALUE_Pos);
#else
#if defined(COMP2)
  if (p_comp == COMP1)
  {
    return (uint32_t)(STM32_READ_BIT(COMP12_COMMON->SR, COMP_SR_C1VAL));
  }
  else
  {
    return (uint32_t)((STM32_READ_BIT(COMP12_COMMON->SR, COMP_SR_C2VAL)) >> COMP_SR_C2VAL_Pos);
  }
#else
  return (uint32_t)(STM32_READ_BIT(p_comp->SR, COMP_SR_C1VAL));
#endif
#endif /* COMP_CSR_VALUE */
}

/**
  * @}
  */

#if defined(COMP_SR_C1IF)
/** @defgroup COMP_LL_EF_FLAG_Management Comparator flag Management
  * @{
  */

/**
  * @brief  Get comparator output trigger flag (latched).
  * @rmtoll
  *  SR       C1IF           LL_COMP_IsActiveFlag_OutputTrig
  * @if COMP2
  *  SR       C2IF           LL_COMP_IsActiveFlag_OutputTrig
  * @endif
  * @param  p_comp Comparator instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_COMP_IsActiveFlag_OutputTrig(const COMP_TypeDef *p_comp)
{
#if defined(COMP2)
  if (p_comp == COMP1)
  {
    return ((STM32_READ_BIT(COMP12_COMMON->SR, COMP_SR_C1IF) == (COMP_SR_C1IF)) ? 1UL : 0UL);
  }
  else
  {
    return ((STM32_READ_BIT(COMP12_COMMON->SR, COMP_SR_C2IF) == (COMP_SR_C2IF)) ? 1UL : 0UL);
  }
#else
  return ((STM32_READ_BIT(p_comp->SR, COMP_SR_C1IF) == (COMP_SR_C1IF)) ? 1UL : 0UL);
#endif /* COMP2 */
}

/**
  * @brief  Clear comparator comparator output trigger flag (latched).
  * @rmtoll
  *  ICFR     CC1IF          LL_COMP_ClearFlag_OutputTrig
  * @if COMP2
  *  ICFR     CC12F          LL_COMP_ClearFlag_OutputTrig
  * @endif
  * @param  p_comp Comparator instance
  */
__STATIC_INLINE void LL_COMP_ClearFlag_OutputTrig(COMP_TypeDef *p_comp)
{
#if defined(COMP2)
  if (p_comp == COMP1)
  {
    STM32_SET_BIT(COMP12_COMMON->ICFR, COMP_ICFR_CC1IF);
  }
  else
  {
    STM32_SET_BIT(COMP12_COMMON->ICFR, COMP_ICFR_CC2IF);
  }
#else
  STM32_SET_BIT(p_comp->ICFR, COMP_ICFR_CC1IF);
#endif /* COMP2 */
}

/**
  * @}
  */
#endif /* COMP_SR_C1IF */

#if defined(COMP_CFGR1_ITEN)
/** @defgroup COMP_LL_EF_IT_Management Comparator IT management
  * @{
  */

/**
  * @brief  Enable comparator output trigger interrupt.
  * @rmtoll
  *  CFGR1    ITEN           LL_COMP_EnableIT_OutputTrig
  * @param  p_comp Comparator instance
  */
__STATIC_INLINE void LL_COMP_EnableIT_OutputTrig(COMP_TypeDef *p_comp)
{
  STM32_SET_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_ITEN);
}

/**
  * @brief  Disable comparator output trigger interrupt.
  * @rmtoll
  *  CFGR1    ITEN           LL_COMP_DisableIT_OutputTrig
  * @param  p_comp Comparator instance
  */
__STATIC_INLINE void LL_COMP_DisableIT_OutputTrig(COMP_TypeDef *p_comp)
{
  STM32_CLEAR_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_ITEN);
}

/**
  * @brief  Get comparator output trigger interrupt state.
  * @rmtoll
  *  CFGR1    ITEN           LL_COMP_IsEnabledIT_OutputTrig
  * @param  p_comp Comparator instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_COMP_IsEnabledIT_OutputTrig(const COMP_TypeDef *p_comp)
{
  return ((STM32_READ_BIT(LL_COMP_REG(p_comp), COMP_CFGR1_ITEN) == (COMP_CFGR1_ITEN)) ? 1UL : 0UL);
}

/**
  * @}
  */
#endif /* COMP_CFGR1_ITEN */

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

#endif /* STM32C5XX_LL_COMP_H */
