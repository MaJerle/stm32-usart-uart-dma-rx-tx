/**
  ******************************************************************************
  * @file    stm32c5xx_ll_opamp.h
  * @brief   Header file of OPAMP LL module.
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
#ifndef STM32C5XX_LL_OPAMP_H
#define STM32C5XX_LL_OPAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined (OPAMP1) || defined (OPAMP2) || defined(OPAMP3)

/** @defgroup OPAMP_LL OPAMP
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup OPAMP_LL_Private_Constants OPAMP Private Constants
  * @{
  */

/* Internal mask for OPAMP power mode:                                        */
/* To select into literal LL_OPAMP_POWERMODE_x the relevant bits for:         */
/* - OPAMP power mode into control register                                   */
/* - OPAMP trimming register offset                                           */


/* Mask for OPAMP power mode into control register */
#define OPAMP_SPEED_MODE_CSR_BIT_MASK            (OPAMP_CSR_OPAHSM)

/* CSR register reset value */
#define OPAMP_CSR_RESET_VALUE  ((uint32_t)0x00000000) /*!< CSR reset value.*/

#define OPAMP_CSR_RESET_BITS                     (OPAMP_CSR_OPAEN          \
                                                  | OPAMP_CSR_CALOUT       \
                                                  | OPAMP_CSR_TRIMOFFSETN  \
                                                  | OPAMP_CSR_TRIMOFFSETP  \
                                                  | OPAMP_CSR_PGA_GAIN     \
                                                  | OPAMP_CSR_CALSEL       \
                                                  | OPAMP_CSR_CALON        \
                                                  | OPAMP_CSR_OPAINTOEN    \
                                                  | OPAMP_CSR_OPAHSM       \
                                                  | OPAMP_CSR_VM_SEL       \
                                                  | OPAMP_CSR_VP_SEL       \
                                                 )  /*!< CSR reset all bits, except USERTRIM and LOCK */

/* TCMR register reset value */
#define OPAMP_TCMR_RESET_VALUE  ((uint32_t)0x00000000) /*!< TCMR reset value.*/

#define OPAMP_TCMR_RESET_BITS                    (OPAMP_TCMR_VMS_SEL      \
                                                  | OPAMP_TCMR_VPS_SEL    \
                                                  | OPAMP_TCMR_TIMCM_SEL  \
                                                  | OPAMP_TCMR_PGAS_GAIN  \
                                                  | OPAMP_TCMR_TIMPGA_SEL \
                                                 )  /*!< TCMR reset all bits, except LOCK */

/* Internal mask for OPAMP trimming of transistors differential pair NMOS     */
/* or PMOS.                                                                   */
/* To select into literal LL_OPAMP_TRIMMING_x the relevant bits for:          */
/* - OPAMP trimming selection of transistors differential pair                */
/* - OPAMP trimming values of transistors differential pair                   */
#define OPAMP_TRIMMING_SELECT_MASK               (OPAMP_CSR_CALSEL)
#define OPAMP_TRIMMING_VALUE_MASK                (OPAMP_CSR_TRIMOFFSETP | OPAMP_CSR_TRIMOFFSETN)

/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup OPAMP_LL_Exported_Constants LL OPAMP Constants
  * @{
  */


/** @defgroup OPAMP_LL_EC_POWERMODE OPAMP power mode
  * @{
  */

#define LL_OPAMP_POWER_MODE_NORMAL               (0x00000000U)       /*!< OPAMP power mode: normal-power */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_SPEEDMODE OPAMP speed mode
  * @{
  */
#define LL_OPAMP_SPEED_MODE_NORMAL               (0x00000000U)       /*!< OPAMP speed mode: normal-speed */
#define LL_OPAMP_SPEED_MODE_HIGH                 (OPAMP_CSR_OPAHSM)  /*!< OPAMP speed mode: high-speed */

/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_MODE OPAMP mode calibration or functional.
  * @{
  */
#define LL_OPAMP_MODE_FUNCTIONAL                 (0x00000000U)          /*!< OPAMP functional mode  */
#define LL_OPAMP_MODE_CALIBRATION                (OPAMP_CSR_CALON)      /*!< OPAMP calibration mode */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_FUNCTIONAL_MODE OPAMP functional mode
  * @{
  */
#define LL_OPAMP_MODE_STANDALONE                 (0x00000000U)                             /*!< OPAMP functional mode,
                                                 OPAMP operation in standalone */
#define LL_OPAMP_MODE_FOLLOWER                   (OPAMP_CSR_VM_SEL_1 | OPAMP_CSR_VM_SEL_0) /*!< OPAMP functional mode,
                                                 OPAMP operation in follower */
#define LL_OPAMP_MODE_PGA                        (OPAMP_CSR_VM_SEL_1)                      /*!< OPAMP functional mode,
                                                 OPAMP operation in PGA */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_PGA_EXT OPAMP PGA external connection mode
  * @{
  */
#define LL_OPAMP_PGA_EXT_NONE                    (0x00000000U)           /*!< In PGA mode, the inverting input is
                                                 connected to the internal feedback resistor. No external connection
                                                 on inverting input.
                                                 Positive gain (Non-Inverting mode)
                                                 - VINPx: Input signal */
#define LL_OPAMP_PGA_EXT_FILT                    (OPAMP_CSR_PGA_GAIN_4)  /*!< In PGA mode, the inverting input is
                                                 connected to the internal feedback resistor and to VINM0 for filtering.
                                                 Positive gain (Non-Inverting mode)
                                                 - VINPx: Input signal
                                                 - VINM0 - VOUT: Additional low-pass filtering */
#define LL_OPAMP_PGA_EXT_BIAS                    (OPAMP_CSR_PGA_GAIN_3)  /*!< In PGA mode, the inverting input is
                                                 connected to the internal feedback resistor. VINM0 is available
                                                 for bias voltage in non-inverting mode or to use the OPAMP in
                                                 inverting mode.
                                                 Negative gain (Inverting mode):
                                                 - VINPx: Bias voltage
                                                 - VINM0: Input signal
                                                 Positive gain (Non-Inverting mode)
                                                 - VINPx: Input signal
                                                 - VINM0: Bias voltage */
#define LL_OPAMP_PGA_EXT_BIAS_FILT               (LL_OPAMP_PGA_EXT_BIAS | LL_OPAMP_PGA_EXT_FILT) /*!< In PGA mode,
                                                 the inverting input is connected to the internal feedback resistor and
                                                 to VINM1 for filtering. VINM0 is available for bias voltage in
                                                 non-inverting mode or to use the OPAMP in inverting mode.
                                                 Negative gain (Inverting mode):
                                                 - VINPx: Bias voltage
                                                 - VINM0: Input signal
                                                 - VINM1 - VOUT: Additional low-pass
                                                   filtering
                                                 Positive gain (Non-Inverting mode)
                                                 - VINPx: Input signal
                                                 - VINM0: Bias voltage
                                                 - VINM1 - VOUT: Additional low-pass filtering */

/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_MODE_PGA_GAIN OPAMP PGA gain (relevant when OPAMP is in functional mode PGA)
  * @{
  */
#define LL_OPAMP_PGA_GAIN_2                      (0x00000000UL)            /*!< OPAMP PGA gain 2  */
#define LL_OPAMP_PGA_GAIN_4                      (OPAMP_CSR_PGA_GAIN_0)    /*!< OPAMP PGA gain 4  */
#define LL_OPAMP_PGA_GAIN_8                      (OPAMP_CSR_PGA_GAIN_1)    /*!< OPAMP PGA gain 8  */
#define LL_OPAMP_PGA_GAIN_16                     (OPAMP_CSR_PGA_GAIN_1 \
                                                  | OPAMP_CSR_PGA_GAIN_0)  /*!< OPAMP PGA gain 16 */
#define LL_OPAMP_PGA_GAIN_2_OR_MINUS_1           (LL_OPAMP_PGA_GAIN_2)     /*!< OPAMP PGA gain 2  or -1  */
#define LL_OPAMP_PGA_GAIN_4_OR_MINUS_3           (LL_OPAMP_PGA_GAIN_4)     /*!< OPAMP PGA gain 4  or -3  */
#define LL_OPAMP_PGA_GAIN_8_OR_MINUS_7           (LL_OPAMP_PGA_GAIN_8)     /*!< OPAMP PGA gain 8  or -7  */
#define LL_OPAMP_PGA_GAIN_16_OR_MINUS_15         (LL_OPAMP_PGA_GAIN_16)    /*!< OPAMP PGA gain 16 or -15 */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_INPUT_NONINVERTING OPAMP input non-inverting
  * @{
  */
#define LL_OPAMP_INPUT_NONINVERT_IO0             (0x00000000U)        /*!< OPAMP non-inverting input 0 connected to GPIO
                                                 (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP0") */
#define LL_OPAMP_INPUT_NONINVERT_IO1             (OPAMP_CSR_VP_SEL_0) /*!< OPAMP non-inverting input 1 connected to GPIO
                                                 (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP1") */
#define LL_OPAMP_INPUT_NONINVERT_IO2             (OPAMP_CSR_VP_SEL_1) /*!< OPAMP non-inverting input 2 connected to GPIO
                                                 (for GPIO mapping, refer to datasheet parameters "OPAMPx_VINP2") */
#define LL_OPAMP_INPUT_NONINVERT_DAC1_CH2        (OPAMP_CSR_VP_SEL)   /*!< OPAMP non-inverting input connected to
                                                 DAC1 channel 2 */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_INPUT_INVERTING OPAMP input inverting
  * @{
  */
#define LL_OPAMP_INPUT_INVERT_IO0                (0x00000000U)         /*!< OPAMP inverting input connected
                                                 to GPIO pin.
                                                 Note: This OPAMP inverting input is used with OPAMP in mode standalone,
                                                       PGA with external bias or filtering, PGA with inverting gain.
                                                       In mode PGA, select LL_OPAMP_INPUT_INVERT_INT_PGA
                                                       and configure input connection,
                                                       refer to @ref LL_OPAMP_SetPGAExternalMode(). */
#define LL_OPAMP_INPUT_INVERT_IO1                (OPAMP_CSR_VM_SEL_0)  /*!< OPAMP inverting input connected
                                                 to GPIO pin. */
#define LL_OPAMP_INPUT_INVERT_INT_PGA            (OPAMP_CSR_VM_SEL_1) /*!< OPAMP inverting input connection
                                                 depending on PGA mode:
                                                 - For PGA without external bias or filtering, PGA without inverting
                                                   gain: inverting input internally connected (not connected
                                                   to a GPIO pin).
                                                 - For PGA with external bias or filtering, PGA with inverting gain:
                                                   inverting input connected to IO0 is used. To configure input
                                                   connection, refer to @ref LL_OPAMP_SetPGAExternalMode(). */
#define LL_OPAMP_INPUT_INVERT_INT_FOLLOWER       (OPAMP_CSR_VM_SEL_1 | OPAMP_CSR_VM_SEL_0) /*!< OPAMP inverting input
                                                 internally connected, intended for OPAMP in mode follower
                                                 (not connected to a GPIO pin). */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_TRIMMING_MODE OPAMP trimming mode
  * @{
  */
#define LL_OPAMP_TRIMMING_FACTORY                (0x00000000U)      /*!< OPAMP trimming factors set to factory values */
#define LL_OPAMP_TRIMMING_USER                   (OPAMP_CSR_USERTRIM)  /*!< OPAMP trimming factors set to user values */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_TRIMMING_TRANSISTORS_DIFF_PAIR OPAMP trimming of transistors differential pair NMOS or PMOS
  * @{
  */
#define LL_OPAMP_TRIMMING_NMOS                   (OPAMP_CSR_TRIMOFFSETN \
                                                  | OPAMP_CSR_CALSEL)     /*!< OPAMP trim for NMOS differential pairs */
#define LL_OPAMP_TRIMMING_PMOS                   (OPAMP_CSR_TRIMOFFSETP \
                                                  | OPAMP_CSR_CALSEL_0)   /*!< OPAMP trim for PMOS differential pairs */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_HW_DELAYS  Definitions of OPAMP hardware constraints delays
  * @note   Only OPAMP IP HW delays are defined in OPAMP LL driver driver,
  *         not timeout values.
  *         For details on delays values, refer to descriptions in source code
  *         above each literal definition.
  * @{
  */

/* Delay for OPAMP startup time (transition from state disable to enable).    */
/* Note: OPAMP startup time depends on board application environment:         */
/*       impedance connected to OPAMP output.                                 */
/*       The delay below is specified under conditions:                       */
/*        - OPAMP in mode low power                                           */
/*        - OPAMP in functional mode follower                                 */
/*        - load impedance of 4kOhm (min), 50pF (max)                         */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tWAKEUP").                                                      */
/* Unit: us                                                                   */
#define LL_OPAMP_DELAY_STARTUP_US                ((uint32_t) 30U)  /*!< Delay for OPAMP startup time */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_TIM_CTRL_INPUT OPAMP Timer-controlled input selection
  * @note The switch can be controlled either by a single timer or a combination of them,
  *       in this case application has to 'ORed' the values below
  *       ex LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC6 | LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4
  * @{
  */
#define LL_OPAMP_MUX_INPUT_CTRL_DISABLE         (0x00000000UL)  /*!< Timer-controlled input selection disabled */
#define LL_OPAMP_MUX_INPUT_CTRL_TIM1_OC6        (OPAMP_TCMR_TIMCM_SEL_0)   /*!< Timer-controlled input selection using
                                                                                 TIM1 OC6 */
#define LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4        (OPAMP_TCMR_TIMCM_SEL_1)   /*!< Timer-controlled input selection using
                                                                                 TIM2 OC4 */
#define LL_OPAMP_MUX_INPUT_CTRL_TIM12_OC1       (OPAMP_TCMR_TIMCM_SEL_1 \
                                                 | OPAMP_TCMR_TIMCM_SEL_0) /*!< Timer-controlled input selection using
                                                                                 TIM12 OC1 */
#define LL_OPAMP_MUX_INPUT_CTRL_TIM15_OC2       (OPAMP_TCMR_TIMCM_SEL_2)   /*!< Timer-controlled input selection using
                                                                                 TIM15 OC2 */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_TIM_CTRL_PGA_GAIN OPAMP Timer-controlled programmable gain
  * @{
  */
#define LL_OPAMP_MUX_PGA_GAIN_CTRL_DISABLE       (0x00000000UL) /*!< Timer-controlled programmable gain
                                                                    selection disabled */
#define LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM1_OC6      (OPAMP_TCMR_TIMPGA_SEL_0)   /*!< Timer-controlled programmable gain
                                                                                  selection using TIM1 OC6 */
#define LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM2_OC4      (OPAMP_TCMR_TIMPGA_SEL_1)   /*!< Timer-controlled programmable gain
                                                                                  selection using TIM2 OC4 */
#define LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC2     (OPAMP_TCMR_TIMPGA_SEL_1 \
                                                  | OPAMP_TCMR_TIMPGA_SEL_0) /*!< Timer-controlled programmable gain
                                                                                  selection using TIM12 OC2 */
#define LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM15_OC2     (OPAMP_TCMR_TIMPGA_SEL_2)   /*!< Timer-controlled programmable gain
                                                                                  selection using TIM15 OC2 */
/**
  * @}
  */

/** @defgroup OPAMP_LL_EC_OUTPUT_CONNECTION OPAMP channel output connection
  * @{
  */
#define LL_OPAMP_OUTPUT_CONNECT_EXTERNAL         (0x00000000UL) /*!< OPAMP output connected to OPAMP_VOUT pin. */
#define LL_OPAMP_OUTPUT_CONNECT_INTERNAL         (OPAMP_CSR_OPAINTOEN) /*!< OPAMP output connected internally to
                                                                            other peripherals (ADC, COMP, ...). */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup OPAMP_LL_Exported_Macros LL OPAMP Macros
  * @{
  */
/** @defgroup OPAMP_LL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
  * @brief  Write a value in OPAMP register.
  * @param  instance OPAMP Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_OPAMP_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in OPAMP register.
  * @param  instance OPAMP Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_OPAMP_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)

/**
  * @}
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup OPAMP_LL_Exported_Functions LL OPAMP Functions
  * @{
  */


/** @defgroup OPAMP_LL_EF_CONFIGURATION_OPAMP_INSTANCE Configuration of OPAMP hierarchical scope: OPAMP instance
  * @{
  */

/**
  * @brief  Reset OPAMP CSR register, reset all bits except USERTRIM and OPA_RANGE.
  * @rmtoll
  *  CSR  OPAHSM    LL_OPAMP_ResetConfig \n
  *  CSR  CALOUT    LL_OPAMP_ResetConfig \n
  *  CSR  USERTRIM  LL_OPAMP_ResetConfig \n
  *  CSR  CALSEL    LL_OPAMP_ResetConfig \n
  *  CSR  CALON     LL_OPAMP_ResetConfig \n
  *  CSR  VP_SEL    LL_OPAMP_ResetConfig
  * @param  p_opamp OPAMP instance
  * @note   The OPAMP must be disabled to change this configuration.
  */
__STATIC_INLINE void LL_OPAMP_ResetConfig(OPAMP_TypeDef *p_opamp)
{
  /* Set OPAMP_CSR register to reset value */
  /* Mind that CSR RANGE bit of OPAMP1 remains unchanged (applies to both OPAMPs) */
  STM32_MODIFY_REG(p_opamp->CSR, (OPAMP_CSR_RESET_BITS), OPAMP_CSR_RESET_VALUE);
}

/**
  * @brief  Set OPAMP speed mode.
  * @rmtoll
  *  CSR     HSM         LL_OPAMP_SetSpeedMode
  * @param  p_opamp OPAMP instance
  * @param  speed_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_SPEED_MODE_NORMAL
  *         @arg @ref LL_OPAMP_SPEED_MODE_HIGH
  * @note   The OPAMP must be disabled to change this configuration.
  */
__STATIC_INLINE void LL_OPAMP_SetSpeedMode(OPAMP_TypeDef *p_opamp, uint32_t speed_mode)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_SPEED_MODE_CSR_BIT_MASK, speed_mode);
}

/**
  * @brief  Get OPAMP speed mode.
  * @rmtoll
  *  CSR     HSM      LL_OPAMP_GetSpeedMode
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_SPEED_MODE_NORMAL
  *         @arg @ref LL_OPAMP_SPEED_MODE_HIGH
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetSpeedMode(const OPAMP_TypeDef *p_opamp)
{
  uint32_t speed_mode = (STM32_READ_BIT(p_opamp->CSR, OPAMP_SPEED_MODE_CSR_BIT_MASK));

  return (uint32_t)(speed_mode);
}

/**
  * @brief  Set OPAMP mode calibration or functional.
  * @rmtoll
  *  CSR      CALON         LL_OPAMP_SetMode
  * @param  p_opamp OPAMP instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_MODE_FUNCTIONAL
  *         @arg @ref LL_OPAMP_MODE_CALIBRATION
  * @note   OPAMP mode corresponds to functional or calibration mode:
  *          - functional mode: OPAMP operation in standalone, follower, ...
  *            Set functional mode using function
  *            @ref LL_OPAMP_SetConfigurationMode().
  *          - calibration mode: offset calibration of the selected
  *            transistors differential pair NMOS or PMOS.
  * @note   On this STM32 series, during calibration, OPAMP functional
  *         mode must be set to standalone or follower mode
  *         (in order to open internal connections to resistors of PGA mode).
  *         Refer to function @ref LL_OPAMP_SetConfigurationMode().
  */
__STATIC_INLINE void LL_OPAMP_SetMode(OPAMP_TypeDef *p_opamp, uint32_t mode)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_CALON, mode);
}

/**
  * @brief  Get OPAMP mode calibration or functional.
  * @rmtoll
  *  CSR      CALON          LL_OPAMP_GetMode
  * @param  p_opamp OPAMP instance
  * @note   OPAMP mode corresponds to functional or calibration mode:
  *          - functional mode: OPAMP operation in standalone, follower, ...
  *            Set functional mode using function
  *            @ref LL_OPAMP_SetConfigurationMode().
  *          - calibration mode: offset calibration of the selected
  *            transistors differential pair NMOS or PMOS.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_MODE_FUNCTIONAL
  *         @arg @ref LL_OPAMP_MODE_CALIBRATION
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetMode(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_CALON));
}

/**
  * @brief  Set OPAMP configuration mode by setting internal connections.
  * @rmtoll
  *  CSR      VM_SEL         LL_OPAMP_SetConfigurationMode
  * @param  p_opamp OPAMP instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_MODE_STANDALONE
  *         @arg @ref LL_OPAMP_MODE_FOLLOWER
  *         @arg @ref LL_OPAMP_MODE_PGA
  *         OPAMP operation in standalone, follower, ...
  * @note   This function reset bit of calibration mode to ensure
  *         to be in functional mode, in order to have OPAMP parameters
  *         (inputs selection, ...) set with the corresponding OPAMP mode
  *         to be effective.
  */
__STATIC_INLINE void LL_OPAMP_SetConfigurationMode(OPAMP_TypeDef *p_opamp, uint32_t mode)
{
  /* Note: Bit OPAMP_CSR_CALON reset to ensure to be in functional mode */
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_VM_SEL | OPAMP_CSR_CALON, mode);
}

/**
  * @brief  Get OPAMP configuration mode from setting of internal connections.
  *         OPAMP operation in standalone, follower, ...
  * @rmtoll
  *  CSR      VM_SEL         LL_OPAMP_GetConfigurationMode
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_MODE_STANDALONE
  *         @arg @ref LL_OPAMP_MODE_FOLLOWER
  *         @arg @ref LL_OPAMP_MODE_PGA
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetConfigurationMode(const OPAMP_TypeDef *p_opamp)
{
  uint32_t tmp_csr = STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_VM_SEL);
  return (uint32_t)(((tmp_csr & OPAMP_CSR_VM_SEL_1) == 0UL) ? (LL_OPAMP_MODE_STANDALONE) : (tmp_csr));
}

/**
  * @brief  Set OPAMP PGA gain.
  * @rmtoll
  *  CSR      gain         LL_OPAMP_SetPGAGain
  * @param  p_opamp OPAMP instance
  * @param  gain This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_GAIN_2
  *         @arg @ref LL_OPAMP_PGA_GAIN_4
  *         @arg @ref LL_OPAMP_PGA_GAIN_8
  *         @arg @ref LL_OPAMP_PGA_GAIN_16
  *         @arg @ref LL_OPAMP_PGA_GAIN_2_OR_MINUS_1
  *         @arg @ref LL_OPAMP_PGA_GAIN_4_OR_MINUS_3
  *         @arg @ref LL_OPAMP_PGA_GAIN_8_OR_MINUS_7
  *         @arg @ref LL_OPAMP_PGA_GAIN_16_OR_MINUS_15
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  */
__STATIC_INLINE void LL_OPAMP_SetPGAGain(OPAMP_TypeDef *p_opamp, uint32_t gain)
{
  STM32_MODIFY_REG(p_opamp->CSR, (OPAMP_CSR_PGA_GAIN_1 | OPAMP_CSR_PGA_GAIN_0), gain);
}

/**
  * @brief  Get OPAMP PGA gain.
  * @rmtoll
  *  CSR      PGA_GAIN         LL_OPAMP_GetPGAGain
  * @param  p_opamp OPAMP instance
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_GAIN_2
  *         @arg @ref LL_OPAMP_PGA_GAIN_4
  *         @arg @ref LL_OPAMP_PGA_GAIN_8
  *         @arg @ref LL_OPAMP_PGA_GAIN_16
  *         @arg @ref LL_OPAMP_PGA_GAIN_2_OR_MINUS_1
  *         @arg @ref LL_OPAMP_PGA_GAIN_4_OR_MINUS_3
  *         @arg @ref LL_OPAMP_PGA_GAIN_8_OR_MINUS_7
  *         @arg @ref LL_OPAMP_PGA_GAIN_16_OR_MINUS_15
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetPGAGain(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, (OPAMP_CSR_PGA_GAIN_1 | OPAMP_CSR_PGA_GAIN_0)));
}

/**
  * @brief  Set OPAMP PGA external connection configuration.
  * @rmtoll
  *  CSR      PGA_GAIN         LL_OPAMP_SetPGAExternalMode
  * @param  p_opamp OPAMP instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_EXT_NONE
  *         @arg @ref LL_OPAMP_PGA_EXT_FILT
  *         @arg @ref LL_OPAMP_PGA_EXT_BIAS
  *         @arg @ref LL_OPAMP_PGA_EXT_BIAS_FILT
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  */
__STATIC_INLINE void LL_OPAMP_SetPGAExternalMode(OPAMP_TypeDef *p_opamp, uint32_t mode)
{
  STM32_MODIFY_REG(p_opamp->CSR, (OPAMP_CSR_PGA_GAIN_4 | OPAMP_CSR_PGA_GAIN_3), mode);
}

/**
  * @brief  Get OPAMP PGA external connection configuration.
  * @rmtoll
  *  CSR      PGA_GAIN         LL_OPAMP_GetPGAExternalMode
  * @param  p_opamp OPAMP instance
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_EXT_NONE
  *         @arg @ref LL_OPAMP_PGA_EXT_FILT
  *         @arg @ref LL_OPAMP_PGA_EXT_BIAS
  *         @arg @ref LL_OPAMP_PGA_EXT_BIAS_FILT
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetPGAExternalMode(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, (OPAMP_CSR_PGA_GAIN_4 | OPAMP_CSR_PGA_GAIN_3)));
}

/**
  * @brief  Set OPAMP in one time configuration parameters.
  * @rmtoll
  *  CSR      CALON         LL_OPAMP_SetConfig \n
  *  CSR      OPAMODE       LL_OPAMP_SetConfig \n
  *  CSR      OPALPM        LL_OPAMP_SetConfig \n
  *  CSR      HSM           LL_OPAMP_SetConfig \n
  *  CSR      OPAINTOEN     LL_OPAMP_SetConfig \n
  *  CSR      VM_SEL        LL_OPAMP_SetConfig \n
  *  CSR      VP_SEL        LL_OPAMP_SetConfig
  * @param  p_opamp OPAMP instance
  * @param  reg_value This parameter is a concatenation of bits CALON,
  *                   HSM,
  *                   VM_SEL, VP_SEL
  * @note   Preliminarily, OPAMP must be disabled.
  */
__STATIC_INLINE void LL_OPAMP_SetConfig(OPAMP_TypeDef *p_opamp, uint32_t reg_value)
{
  STM32_MODIFY_REG(p_opamp->CSR,
                   (OPAMP_CSR_CALON           /* Functional mode */
                    | OPAMP_CSR_OPAHSM        /* High speed mode */
                    | OPAMP_CSR_VM_SEL        /* Inverting input */
                    | OPAMP_CSR_VP_SEL        /* Non-inverting input */
                    | OPAMP_CSR_OPAINTOEN     /* Output connection */
                   ),
                   reg_value);
}

/**
  * @brief  Get OPAMP configuration parameters bit fields.
  * @rmtoll
  *  CSR      CALON         LL_OPAMP_GetConfig \n
  *  CSR      OPAMODE       LL_OPAMP_GetConfig \n
  *  CSR      OPALPM        LL_OPAMP_GetConfig \n
  *  CSR      HSM           LL_OPAMP_GetConfig \n
  *  CSR      OPAINTOEN     LL_OPAMP_SetConfig \n
  *  CSR      VM_SEL        LL_OPAMP_GetConfig \n
  *  CSR      VP_SEL        LL_OPAMP_GetConfig
  * @param  p_opamp OPAMP instance
  * @retval Returned value is the concatenation of bits CALON,
  *         HSM,
  *         VM_SEL, VP_SEL
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetConfig(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)STM32_READ_BIT(p_opamp->CSR,
                                  (OPAMP_CSR_CALON           /* Functional mode */
                                   | OPAMP_CSR_OPAHSM        /* High speed mode */
                                   | OPAMP_CSR_VM_SEL        /* Inverting input */
                                   | OPAMP_CSR_VP_SEL        /* Non-inverting input */
                                   | OPAMP_CSR_OPAINTOEN     /* Output connection */
                                  ));
}

/**
  * @}
  */

/** @defgroup OPAMP_LL_EF_CONFIGURATION_INPUTS Configuration of OPAMP inputs
  * @{
  */

/**
  * @brief  Set OPAMP non-inverting input connection.
  * @rmtoll
  *  CSR      VPSEL          LL_OPAMP_SetInputNonInverting
  * @param  p_opamp OPAMP instance
  * @param  input_non_inverting This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE void LL_OPAMP_SetInputNonInverting(OPAMP_TypeDef *p_opamp, uint32_t input_non_inverting)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_VP_SEL, input_non_inverting);
}

/**
  * @brief  Get OPAMP non-inverting input connection.
  * @rmtoll
  *  CSR      VPSEL          LL_OPAMP_GetInputNonInverting
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetInputNonInverting(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_VP_SEL));
}

/**
  * @brief  Set OPAMP inverting input connection.
  * @rmtoll
  *  CSR      VMSEL          LL_OPAMP_SetInputInverting
  * @param  p_opamp OPAMP instance
  * @param  input_inverting This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  * @note   OPAMP inverting input is used with OPAMP in mode standalone
  *         or PGA with external capacitors for filtering circuit.
  *         Otherwise (OPAMP in mode follower), OPAMP inverting input
  *         is not used (not connected to GPIO pin).
  */
__STATIC_INLINE void LL_OPAMP_SetInputInverting(OPAMP_TypeDef *p_opamp, uint32_t input_inverting)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_VM_SEL, input_inverting);
}

/**
  * @brief  Get OPAMP inverting input connection.
  * @rmtoll
  *  CSR      VMSEL          LL_OPAMP_GetInputInverting
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetInputInverting(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_VM_SEL));
}

/**
  * @brief  Set OPAMP both inverting input and non-inverting input connections.
  * @rmtoll
  *  CSR      VPSEL          LL_OPAMP_SetInputs \n
  *  CSR      VMSEL          LL_OPAMP_SetInputs
  * @param  p_opamp OPAMP instance
  * @param  input_non_inverting This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  * @param  input_inverting This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  * @note   OPAMP inverting input is used with OPAMP in mode standalone
  *         or PGA with external capacitors for filtering circuit.
  *         Otherwise (OPAMP in mode follower), OPAMP inverting input
  *         is not used (not connected to GPIO pin).
  */
__STATIC_INLINE void LL_OPAMP_SetInputs(OPAMP_TypeDef *p_opamp,
                                        uint32_t input_non_inverting, uint32_t input_inverting)
{
  STM32_MODIFY_REG(p_opamp->CSR, (OPAMP_CSR_VP_SEL | OPAMP_CSR_VM_SEL), (input_non_inverting | input_inverting));
}

/**
  * @brief  Get OPAMP both non-inverting input and inverting input connection.
  * @rmtoll
  *  CSR      VPSEL          LL_OPAMP_GetInputs \n
  *  CSR      VMSEL          LL_OPAMP_GetInputs
  * @param  p_opamp OPAMP instance
  * @retval Returned value is inverting input and non-inverting input both contained in an uint32_t.
  *         This bit field contains the following values inside the [31,0] bit field:
  *         VP_SEL bit [3,2]
  *         VM_SEL bit [6,5]
  *         Those parameters can be one of the following values
  *         for VP_SEL:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  *         for VM_SEL:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetInputs(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, (OPAMP_CSR_VP_SEL | OPAMP_CSR_VM_SEL)));
}
/**
  * @}
  */

/** @defgroup OPAMP_LL_EF_OPAMP_TRIMMING Configuration and operation of OPAMP trimming
  * @{
  */

/**
  * @brief  Set OPAMP trimming mode.
  * @rmtoll
  *  CSR      USERTRIM       LL_OPAMP_SetTrimmingMode
  * @param  p_opamp OPAMP instance
  * @param  trimming_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_FACTORY
  *         @arg @ref LL_OPAMP_TRIMMING_USER
  */
__STATIC_INLINE void LL_OPAMP_SetTrimmingMode(OPAMP_TypeDef *p_opamp, uint32_t trimming_mode)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_USERTRIM, trimming_mode);
}

/**
  * @brief  Get OPAMP trimming mode.
  * @rmtoll
  *  CSR      USERTRIM       LL_OPAMP_GetTrimmingMode
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_FACTORY
  *         @arg @ref LL_OPAMP_TRIMMING_USER
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetTrimmingMode(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_USERTRIM));
}

/**
  * @brief  Set OPAMP offset to calibrate the selected transistors
  *         differential pair NMOS or PMOS.
  * @rmtoll
  *  CSR      CALSEL         LL_OPAMP_SetCalibrationSelection
  * @param  p_opamp OPAMP instance
  * @param  transistors_diff_pair This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  * @note   Preliminarily, OPAMP must be set in mode calibration
  *         using function @ref LL_OPAMP_SetMode().
  */
__STATIC_INLINE void LL_OPAMP_SetCalibrationSelection(OPAMP_TypeDef *p_opamp, uint32_t transistors_diff_pair)
{
  /* Parameter used with mask "OPAMP_TRIMMING_SELECT_MASK" because            */
  /* containing other bits reserved for other purpose.                        */
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_CALSEL, (transistors_diff_pair & OPAMP_TRIMMING_SELECT_MASK));
}

/**
  * @brief  Get OPAMP offset to calibrate the selected transistors
  *         differential pair NMOS or PMOS.
  * @rmtoll
  *  CSR      CALSEL         LL_OPAMP_GetCalibrationSelection
  * @param  p_opamp OPAMP instance
  * @note   Preliminarily, OPAMP must be set in mode calibration
  *         using function @ref LL_OPAMP_SetMode().
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetCalibrationSelection(const OPAMP_TypeDef *p_opamp)
{
  uint32_t calib_sel = (uint32_t)(STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_CALSEL));

  return (calib_sel | (((calib_sel & OPAMP_CSR_CALSEL_1) == 0UL) ? LL_OPAMP_TRIMMING_PMOS : LL_OPAMP_TRIMMING_NMOS));
}

/**
  * @brief  Get OPAMP calibration result of toggling output.
  * @rmtoll
  *  CSR      CALOUT         LL_OPAMP_IsCalibrationOutputSet
  * @param  p_opamp OPAMP instance
  * @retval 1 If the offset is not enough compensated with the current trim offset value.
  * @retval 0 If the offset is enough compensated.
  * @note Returned value is not related to CALOUT flag value: intended to monitor
  *       CALOUT flag transition and state corresponding to calibration completed
  *       (refer to calibration procedure in reference manual).
  */
__STATIC_INLINE uint32_t LL_OPAMP_IsCalibrationOutputSet(const OPAMP_TypeDef *p_opamp)
{
  return ((STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_CALOUT) == OPAMP_CSR_CALOUT) ? 0UL : 1UL);
}

/**
  * @brief  Set OPAMP trimming factor for the selected transistors
  *         differential pair NMOS or PMOS, corresponding to the selected
  *         power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_SetTrimmingValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_SetTrimmingValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @param  transistors_diff_pair This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  * @param  trimming_value 0x01...0x1F
  */
__STATIC_INLINE void LL_OPAMP_SetTrimmingValue(OPAMP_TypeDef *p_opamp, uint32_t power_mode,
                                               uint32_t transistors_diff_pair, uint32_t trimming_value)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  __IO uint32_t *preg = &(p_opamp->CSR);

  /* Set bits with position in register depending on parameter                */
  /* "transistors_diff_pair".                                                 */
  /* Parameter used with mask "OPAMP_TRIMMING_VALUE_MASK" because             */
  /* containing other bits reserved for other purpose.                        */
  STM32_MODIFY_REG(*preg,
                   (transistors_diff_pair & OPAMP_TRIMMING_VALUE_MASK),
                   (trimming_value << ((transistors_diff_pair == LL_OPAMP_TRIMMING_NMOS) ?
                                       OPAMP_CSR_TRIMOFFSETN_Pos : OPAMP_CSR_TRIMOFFSETP_Pos))
                  );
}

/**
  * @brief  Get OPAMP trimming factor for the selected transistors
  *         differential pair NMOS or PMOS, corresponding to the selected
  *         power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_GetTrimmingValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_GetTrimmingValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @param  transistors_diff_pair This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  * @retval 0x1...0x1F
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetTrimmingValue(const OPAMP_TypeDef *p_opamp, uint32_t power_mode,
                                                   uint32_t transistors_diff_pair)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  const __IO uint32_t *preg = &(p_opamp->CSR);

  /* Retrieve bits with position in register depending on parameter           */
  /* "transistors_diff_pair".                                                 */
  /* Parameter used with mask "OPAMP_TRIMMING_VALUE_MASK" because             */
  /* containing other bits reserved for other purpose.                        */
  return (uint32_t)(STM32_READ_BIT(*preg, (transistors_diff_pair & OPAMP_TRIMMING_VALUE_MASK))
                    >> ((transistors_diff_pair == LL_OPAMP_TRIMMING_NMOS) ?
                        OPAMP_CSR_TRIMOFFSETN_Pos : OPAMP_CSR_TRIMOFFSETP_Pos)
                   );
}

/**
  * @brief  Set OPAMP trimming factor for the selected transistors
  *         differential pair NMOS or PMOS, corresponding to the selected
  *         power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_SetOffsetTrimValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_SetOffsetTrimValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @param  transistors_diff_pair This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  * @param  trimming_value 0x01...0x1F
  */
__STATIC_INLINE void LL_OPAMP_SetOffsetTrimValue(OPAMP_TypeDef *p_opamp, uint32_t power_mode,
                                                 uint32_t transistors_diff_pair, uint32_t trimming_value)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  __IO uint32_t *preg = &(p_opamp->CSR);

  /* Set bits with position in register depending on parameter    */
  /* "transistors_diff_pair".                                     */
  /* Parameter used with mask "OPAMP_TRIMMING_VALUE_MASK" because */
  /* containing other bits reserved for other purpose.            */
  STM32_MODIFY_REG(*preg,
                   (transistors_diff_pair & OPAMP_TRIMMING_VALUE_MASK),
                   (trimming_value << ((transistors_diff_pair == LL_OPAMP_TRIMMING_NMOS) ?
                                       OPAMP_CSR_TRIMOFFSETN_Pos : OPAMP_CSR_TRIMOFFSETP_Pos
                                      ))
                  );
}

/**
  * @brief  Get OPAMP trimming factor for the selected transistors
  *         differential pair NMOS or PMOS, corresponding to the selected
  *         power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_GetOffsetTrimValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_GetOffsetTrimValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @param  transistors_diff_pair This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_TRIMMING_NMOS
  *         @arg @ref LL_OPAMP_TRIMMING_PMOS
  * @retval 0x1...0x1F
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetOffsetTrimValue(const OPAMP_TypeDef *p_opamp, uint32_t power_mode,
                                                     uint32_t transistors_diff_pair)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  const __IO uint32_t *preg = &(p_opamp->CSR);

  /* Retrieve bits with position in register depending on parameter */
  /* "transistors_diff_pair".                                       */
  /* Parameter used with mask "OPAMP_TRIMMING_VALUE_MASK" because   */
  /* containing other bits reserved for other purpose.              */
  return (uint32_t)(STM32_READ_BIT(*preg, (transistors_diff_pair & OPAMP_TRIMMING_VALUE_MASK))
                    >> ((transistors_diff_pair == LL_OPAMP_TRIMMING_NMOS) ?
                        OPAMP_CSR_TRIMOFFSETN_Pos : OPAMP_CSR_TRIMOFFSETP_Pos
                       )
                   );
}

/**
  * @brief  Set OPAMP all differential pair trimming (PMOS and NMOS) values for the selected power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_SetOffsetTrimAllValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_SetOffsetTrimAllValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @param  p_trim_value 0x01...0x1F
  * @param  n_trim_value 0x01...0x1F
  */
__STATIC_INLINE void LL_OPAMP_SetOffsetTrimAllValue(OPAMP_TypeDef *p_opamp, uint32_t power_mode,
                                                    uint32_t p_trim_value, uint32_t n_trim_value)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  __IO uint32_t *preg = &(p_opamp->CSR);

  /* Set bits in register OTR or LPOTR depending on parameter power_mode */
  STM32_MODIFY_REG(*preg,
                   OPAMP_TRIMMING_VALUE_MASK,
                   ((((p_trim_value) << OPAMP_CSR_TRIMOFFSETP_Pos) | ((n_trim_value) << OPAMP_CSR_TRIMOFFSETN_Pos))
                    & (OPAMP_TRIMMING_VALUE_MASK)
                   )
                  );
}

/**
  * @brief  Get OPAMP PMOS and NMOS differential pair trimming values for the selected power mode.
  * @rmtoll
  *  CSR      TRIMOFFSETN    LL_OPAMP_GetOffsetTrimAllValue \n
  *  CSR      TRIMOFFSETP    LL_OPAMP_GetOffsetTrimAllValue
  * @param  p_opamp OPAMP instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_POWER_MODE_NORMAL(1)
  *         (1) Parameter unused for this function on this series, kept for compatibility purpose
  * @retval Concatenation of trimming values (value range 32 bits),
  *         each value corresponding to register bitfield PMOS and NMOS differential pair
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetOffsetTrimAllValue(const OPAMP_TypeDef *p_opamp, uint32_t power_mode)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(power_mode);

  const __IO uint32_t *preg = &(p_opamp->CSR);

  /* Retrieve bits with position in register depending on parameter           */
  /* "transistors_diff_pair".                                                 */
  /* Parameter used with mask "OPAMP_TRIMMING_VALUE_MASK" because             */
  /* containing other bits reserved for other purpose.                        */
  return (uint32_t)(STM32_READ_BIT(*preg, (OPAMP_TRIMMING_VALUE_MASK)));
}

/**
  * @}
  */

/** @defgroup OPAMP_LL_EF_OPERATION Operation on OPAMP instance
  * @{
  */
/**
  * @brief  Enable OPAMP instance.
  * @rmtoll
  *  CSR      OPAEN          LL_OPAMP_Enable
  * @param  p_opamp OPAMP instance
  * @note   After enable from off state, OPAMP requires a delay
  *         to fulfill wake up time specification.
  *         Refer to device datasheet, parameter "tWAKEUP".
  */
__STATIC_INLINE void LL_OPAMP_Enable(OPAMP_TypeDef *p_opamp)
{
  STM32_SET_BIT(p_opamp->CSR, OPAMP_CSR_OPAEN);
}

/**
  * @brief  Disable OPAMP instance.
  * @rmtoll
  *  CSR      OPAEN          LL_OPAMP_Disable
  * @param  p_opamp OPAMP instance
  */
__STATIC_INLINE void LL_OPAMP_Disable(OPAMP_TypeDef *p_opamp)
{
  STM32_CLEAR_BIT(p_opamp->CSR, OPAMP_CSR_OPAEN);
}

/**
  * @brief  Get OPAMP instance enable state.
  * @rmtoll
  *  CSR      OPAEN          LL_OPAMP_IsEnabled
  * @param  p_opamp OPAMP instance
  * @retval 1 if OPAMP is enabled.
  * @retval 0 if OPAMP is disabled.
  */
__STATIC_INLINE uint32_t LL_OPAMP_IsEnabled(const OPAMP_TypeDef *p_opamp)
{
  return ((STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_OPAEN) == (OPAMP_CSR_OPAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set OPAMP non-inverting input secondary connection.
  * @rmtoll
  *  TCMR     VPSSEL         LL_OPAMP_SetInputMuxNonInvertingSecondary
  * @param  p_opamp OPAMP instance
  * @param  input_non_inverting_sec This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE void LL_OPAMP_SetInputMuxNonInvertingSecondary(OPAMP_TypeDef *p_opamp, uint32_t input_non_inverting_sec)
{
  STM32_MODIFY_REG(p_opamp->TCMR, OPAMP_TCMR_VPS_SEL, input_non_inverting_sec >> 1UL);
}

/**
  * @brief  Get OPAMP non-inverting input secondary connection.
  * @rmtoll
  *  TCMR     VPSSEL         LL_OPAMP_GetInputMuxNonInvertingSecondary
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetInputMuxNonInvertingSecondary(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->TCMR, OPAMP_TCMR_VPS_SEL) << 1UL);
}

/**
  * @brief  Set OPAMP inverting input secondary connection.
  * @rmtoll
  *  TCMR     VMSSEL         LL_OPAMP_SetInputMuxInvertingSecondary
  * @param  p_opamp OPAMP instance
  * @param  input_inverting_sec This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  * @note   OPAMP inverting input is used with OPAMP in mode standalone
  *         or PGA with external capacitors for filtering circuit.
  *         Otherwise (OPAMP in mode follower), OPAMP inverting input
  *         is not used (not connected to GPIO pin).
  * @note   Timer-controlled secondary mode has constraints from timer-controlled primary mode,
  *         therefore function @ref LL_OPAMP_SetConfigurationMode() must be called prior to this function.
  */
__STATIC_INLINE void LL_OPAMP_SetInputMuxInvertingSecondary(OPAMP_TypeDef *p_opamp, uint32_t input_inverting_sec)
{
  STM32_MODIFY_REG(p_opamp->TCMR, OPAMP_TCMR_VMS_SEL,
                   (((input_inverting_sec == LL_OPAMP_INPUT_INVERT_IO1)
                     || (input_inverting_sec == LL_OPAMP_INPUT_INVERT_INT_FOLLOWER)) ? 1UL : 0UL));
}

/**
  * @brief  Get OPAMP inverting input secondary connection.
  * @rmtoll
  *  TCMR     VMSSEL         LL_OPAMP_GetInputMuxInvertingSecondary
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetInputMuxInvertingSecondary(const OPAMP_TypeDef *p_opamp)
{
  uint32_t csr_value = STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_VM_SEL_1);
  uint32_t tcmr_value = STM32_READ_BIT(p_opamp->TCMR, OPAMP_TCMR_VMS_SEL);
  return (uint32_t)(csr_value | (tcmr_value << OPAMP_CSR_VM_SEL_Pos));
}

/**
  * @brief  Set OPAMP both inverting input and non-inverting input secondary connections.
  * @rmtoll
  *  TCMR      VPSSEL         LL_OPAMP_SetInputsMuxSecondary \n
  *  TCMR      VMSSEL         LL_OPAMP_SetInputsMuxSecondary
  * @param  p_opamp OPAMP instance
  * @param  input_non_inverting_sec This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_NONINVERT_IO1
  *         @arg LL_OPAMP_INPUT_NONINVERT_IO2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC1_CH2 (1)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH1 (2)(3)
  *         @arg LL_OPAMP_INPUT_NONINVERT_DAC2_CH2 (2)
  *
  *         - (1) Parameters only available on STM32C542xx/532xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available on STM32C5J1xx/5K1xx devices
  * @param  input_inverting_sec This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO0
  *         @arg @ref LL_OPAMP_INPUT_INVERT_IO1
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_PGA
  *         @arg @ref LL_OPAMP_INPUT_INVERT_INT_FOLLOWER
  * @note   OPAMP inverting input secondary is used with OPAMP in mode standalone
  *         or PGA with external capacitors for filtering circuit.
  *         Otherwise (OPAMP in mode follower), OPAMP inverting input
  *         is not used (not connected to GPIO pin).
  */
__STATIC_INLINE void LL_OPAMP_SetInputsMuxSecondary(OPAMP_TypeDef *p_opamp,
                                                    uint32_t input_non_inverting_sec, uint32_t input_inverting_sec)
{
  STM32_MODIFY_REG(p_opamp->TCMR, (OPAMP_TCMR_VPS_SEL | OPAMP_TCMR_VMS_SEL),
                   ((input_non_inverting_sec >> 1UL)
                    | (((input_inverting_sec == LL_OPAMP_INPUT_INVERT_IO1)
                        || (input_inverting_sec == LL_OPAMP_INPUT_INVERT_INT_PGA)) ? 1UL : 0UL)));
}

/**
  * @brief  Set OPAMP inputs multiplexer mode.
  * @rmtoll
  *  TCMR     TCMEN          LL_OPAMP_SetMuxInputCtrl
  * @param  p_opamp OPAMP instance
  * @param  mux_inputs_ctrl This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_DISABLE
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM1_OC6
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM12_OC1
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM15_OC2
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM8_OC6 (1)(2)
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM20_OC6 (1)
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM3_OC2 (1)(2)
  *
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (2) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE void LL_OPAMP_SetMuxInputCtrl(OPAMP_TypeDef *p_opamp, uint32_t mux_inputs_ctrl)
{
  STM32_MODIFY_REG(p_opamp->TCMR, OPAMP_TCMR_TIMCM_SEL, mux_inputs_ctrl);
}

/**
  * @brief  Get OPAMP inputs multiplexer mode.
  * @rmtoll
  *  TCMR     TCMEN          LL_OPAMP_GetMuxInputCtrl
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_DISABLE
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM1_OC6
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM2_OC4
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM12_OC1
  *         @arg @ref LL_OPAMP_MUX_INPUT_CTRL_TIM15_OC2
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM8_OC6 (1)(2)
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM20_OC6 (1)
  *         @arg LL_OPAMP_MUX_INPUT_CTRL_TIM3_OC2 (1)(2)
  *
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (2) Parameters only available on STM32C5J1xx/5K1xx devices
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetMuxInputCtrl(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->TCMR, OPAMP_TCMR_TIMCM_SEL));
}

/**
  * @brief  Set OPAMP PGA secondary gain.
  * @rmtoll
  *  TCMR   PGAS_GAIN    LL_OPAMP_SetPGAGainMuxSecondary
  * @param  p_opamp OPAMP instance
  * @param  gain_secondary This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_GAIN_2
  *         @arg @ref LL_OPAMP_PGA_GAIN_4
  *         @arg @ref LL_OPAMP_PGA_GAIN_8
  *         @arg @ref LL_OPAMP_PGA_GAIN_16
  *         @arg @ref LL_OPAMP_PGA_GAIN_2_OR_MINUS_1
  *         @arg @ref LL_OPAMP_PGA_GAIN_4_OR_MINUS_3
  *         @arg @ref LL_OPAMP_PGA_GAIN_8_OR_MINUS_7
  *         @arg @ref LL_OPAMP_PGA_GAIN_16_OR_MINUS_15
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  */
__STATIC_INLINE void LL_OPAMP_SetPGAGainMuxSecondary(OPAMP_TypeDef *p_opamp, uint32_t gain_secondary)
{
  STM32_MODIFY_REG(p_opamp->TCMR, OPAMP_TCMR_PGAS_GAIN, (gain_secondary >> 6UL));
}

/**
  * @brief  Get OPAMP PGA gain.
  * @rmtoll
  *  TCMR   PGAS_GAIN      LL_OPAMP_GetPGAGainMuxSecondary
  * @param  p_opamp OPAMP instance
  * @note   Preliminarily, OPAMP must be set in mode PGA
  *         using function @ref LL_OPAMP_SetConfigurationMode().
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_PGA_GAIN_2
  *         @arg @ref LL_OPAMP_PGA_GAIN_4
  *         @arg @ref LL_OPAMP_PGA_GAIN_8
  *         @arg @ref LL_OPAMP_PGA_GAIN_16
  *         @arg @ref LL_OPAMP_PGA_GAIN_2_OR_MINUS_1
  *         @arg @ref LL_OPAMP_PGA_GAIN_4_OR_MINUS_3
  *         @arg @ref LL_OPAMP_PGA_GAIN_8_OR_MINUS_7
  *         @arg @ref LL_OPAMP_PGA_GAIN_16_OR_MINUS_15
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetPGAGainMuxSecondary(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->TCMR, OPAMP_TCMR_PGAS_GAIN) << 6UL);
}

/**
  * @brief  Set OPAMP multiplexer PGA gain.
  * @rmtoll
  *  TCMR   TIMPGA_SEL  LL_OPAMP_SetMuxPGAGainCtrl
  * @param  p_opamp OPAMP instance
  * @param  mux_pga_ctrl This parameter can be one of the following values:
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_DISABLE
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM1_OC6
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM2_OC4
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC1 (1)(2)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC2 (3)
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM15_OC2
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM8_OC6 (1)(2)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM20_OC6 (1)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM3_OC2 (1)(2)
  *
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (2) Parameters only available on STM32C5J1xx/5K1xx devices
  *         - (3) Parameters only available on STM32C542xx/532xx devices
  */
__STATIC_INLINE void LL_OPAMP_SetMuxPGAGainCtrl(OPAMP_TypeDef *p_opamp, uint32_t mux_pga_ctrl)
{
  STM32_MODIFY_REG(p_opamp->TCMR,  OPAMP_TCMR_TIMPGA_SEL, mux_pga_ctrl);
}

/**
  * @brief  Get OPAMP multiplexer PGA gain.
  * @rmtoll
  *  TCMR   TIMPGA_SEL  LL_OPAMP_GetMuxPGAGainCtrl
  * @param  p_opamp OPAMP instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_DISABLE
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM1_OC6
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM2_OC4
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC1 (1)(2)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM12_OC2 (3)
  *         @arg @ref LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM15_OC2
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM8_OC6 (1)(2)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM20_OC6 (1)
  *         @arg LL_OPAMP_MUX_PGA_GAIN_CTRL_TIM3_OC2 (1)(2)
  *
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (2) Parameters only available on STM32C5J1xx/5K1xx devices
  *         - (3) Parameters only available on STM32C542xx/532xx devices
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetMuxPGAGainCtrl(const OPAMP_TypeDef *p_opamp)
{
  return (uint32_t)(STM32_READ_BIT(p_opamp->TCMR, OPAMP_TCMR_TIMPGA_SEL));
}

/**
  * @brief  Set the output connection for OPAMP instance.
  * @rmtoll
  *  CSR      OPAINTOEN       LL_OPAMP_SetOutputConnection
  * @param  p_opamp OPAMP instance
  * @param  opamp_out This parameter can be one of the following values:
  *         @arg LL_OPAMP_OUTPUT_CONNECT_EXTERNAL
  *         @arg LL_OPAMP_OUTPUT_CONNECT_INTERNAL
  */
__STATIC_INLINE void LL_OPAMP_SetOutputConnection(OPAMP_TypeDef *p_opamp, uint32_t opamp_out)
{
  STM32_MODIFY_REG(p_opamp->CSR, OPAMP_CSR_OPAINTOEN, opamp_out);
}

/**
  * @brief  Get the current output connection state for OPAMP instance.
  * @rmtoll
  *  CSR      OPAINTOEN       LL_OPAMP_GetOutputConnection
  * @param  p_opamp OPAMP instance
  * @retval LL_OPAMP_OUTPUT_CONNECT_EXTERNAL if the output is connected to the OPAMP_VOUT pin.
  * @retval LL_OPAMP_OUTPUT_CONNECT_INTERNAL if the output is connected internally to an ADC/COMP channel.
  */
__STATIC_INLINE uint32_t LL_OPAMP_GetOutputConnection(const OPAMP_TypeDef *p_opamp)
{
  return ((STM32_READ_BIT(p_opamp->CSR, OPAMP_CSR_OPAINTOEN) != 0UL) \
          ? LL_OPAMP_OUTPUT_CONNECT_INTERNAL : LL_OPAMP_OUTPUT_CONNECT_EXTERNAL);
}

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

#endif /* STM32C5xx_LL_OPAMP_H */
