/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_dbgmcu.h
  * @brief   Header file of LL DBGMCU module.
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_LL_DBGMCU_H
#define STM32C5XX_LL_DBGMCU_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (DBGMCU)
/** @defgroup LL_DBGMCU LL DBGMCU
  * @{
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/* Exported constants ------------------------------------------------------------------------------------------------*/

/** @defgroup DBGMCU_LL_Exported_Constants LL DBGMCU Constants
  * @{
  */

/** @defgroup DBGMCU_DEV_ID DBGMCU Device ID definition
  * @{
  */
#define LL_DBGMCU_DEV_ID_C53X_C542 0x044FU /*!< STM32C5 series device STM32C53x/542 */
#define LL_DBGMCU_DEV_ID_C55X_C562 0x044EU /*!< STM32C5 series device STM32C55x/562 */
#define LL_DBGMCU_DEV_ID_C59X_C5A3 0x045AU /*!< STM32C5 series device STM32C59x/5A3 */
/**
  * @}
  */

/** @defgroup DBGMCU_REV_ID DBGMCU Device revision ID definition
  * @{
  */
#define LL_DBGMCU_REV_ID_A 0x1000U /*!< STM32C5 device revision A */
#define LL_DBGMCU_REV_ID_Z 0x1001U /*!< STM32C5 device revision Z */
#define LL_DBGMCU_REV_ID_Y 0x1003U /*!< STM32C5 device revision Y */
/**
  * @}
  */

/** @defgroup DBGMCU_DEBUG_LOW_POWER_MODE DBGMCU Debug during low power mode
  * @{
  */
#define LL_DBGMCU_SLEEP_MODE_DEBUG   DBGMCU_CR_DBG_SLEEP    /*!< Debug during Sleep mode   */
#define LL_DBGMCU_STOP_MODE_DEBUG    DBGMCU_CR_DBG_STOP     /*!< Debug during Stop mode    */
#define LL_DBGMCU_STANDBY_MODE_DEBUG DBGMCU_CR_DBG_STANDBY  /*!< Debug during Standby mode */
#define LL_DBGMCU_LP_MODE_DEBUG_ALL  (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP  | \
                                      DBGMCU_CR_DBG_STANDBY)
/*!< Debug during Low power mode
 (Sleep, Stop and Standby modes) */
/**
  * @}
  */

/** @defgroup DBGMCU_LL_EC_APB1_GRP1_STOP_IP DBGMCU APB1_GRP1 STOP IP
  * @{
  */
#define LL_DBGMCU_TIM2_STOP      DBGMCU_APB1LFZR_DBG_TIM2_STOP  /*!< The counter clock of TIM2 is stopped when the core is in debug mode              */
#if defined(TIM3)
#define LL_DBGMCU_TIM3_STOP      DBGMCU_APB1LFZR_DBG_TIM3_STOP  /*!< The counter clock of TIM3 is stopped when the core is in debug mode              */
#endif /* TIM3 */
#if defined(TIM4)
#define LL_DBGMCU_TIM4_STOP      DBGMCU_APB1LFZR_DBG_TIM4_STOP  /*!< The counter clock of TIM4 is stopped when the core is in debug mode              */
#endif /* TIM4 */
#if defined(TIM5)
#define LL_DBGMCU_TIM5_STOP      DBGMCU_APB1LFZR_DBG_TIM5_STOP  /*!< The counter clock of TIM5 is stopped when the core is in debug mode              */
#endif /* TIM5 */
#define LL_DBGMCU_TIM6_STOP      DBGMCU_APB1LFZR_DBG_TIM6_STOP  /*!< The counter clock of TIM6 is stopped when the core is in debug mode              */
#define LL_DBGMCU_TIM7_STOP      DBGMCU_APB1LFZR_DBG_TIM7_STOP  /*!< The counter clock of TIM7 is stopped when the core is in debug mode              */
#define LL_DBGMCU_TIM12_STOP     DBGMCU_APB1LFZR_DBG_TIM12_STOP /*!< The counter clock of TIM12 is stopped when the core is in debug mode             */
#define LL_DBGMCU_WWDG_STOP      DBGMCU_APB1LFZR_DBG_WWDG_STOP  /*!< The window watchdog counter clock is stopped when the core is in debug mode      */
#define LL_DBGMCU_IWDG_STOP      DBGMCU_APB1LFZR_DBG_IWDG_STOP  /*!< The independent watchdog counter clock is stopped when the core is in debug mode */
#define LL_DBGMCU_I2C1_STOP      DBGMCU_APB1LFZR_DBG_I2C1_STOP  /*!< The I2C1 SMBus timeout is frozen while the core is in debug mode                 */
#if defined(I2C2)
#define LL_DBGMCU_I2C2_STOP      DBGMCU_APB1LFZR_DBG_I2C2_STOP  /*!< The I2C2 SMBus timeout is frozen  while the core is in debug mode                */
#endif /* I2C2 */
#define LL_DBGMCU_I3C1_STOP      DBGMCU_APB1LFZR_DBG_I3C1_STOP  /*!< The I3C1 timeout is frozen while the core is in debug mode                       */
/**
  * @}
  */

/** @defgroup DBGMCU_LL_EC_APB2_GRP1_STOP_IP DBGMCU APB2_GRP1 STOP IP
  * @{
  */
#define LL_DBGMCU_TIM1_STOP      DBGMCU_APB2FZR_DBG_TIM1_STOP    /*!< The counter clock of TIM1 is stopped when the core is in debug mode   */
#define LL_DBGMCU_TIM8_STOP      DBGMCU_APB2FZR_DBG_TIM8_STOP    /*!< The counter clock of TIM8 is stopped when the core is in debug mode   */
#define LL_DBGMCU_TIM15_STOP     DBGMCU_APB2FZR_DBG_TIM15_STOP   /*!< The counter clock of TIM15 is stopped when the core is in debug mode  */
#if defined(TIM16)
#define LL_DBGMCU_TIM16_STOP     DBGMCU_APB2FZR_DBG_TIM16_STOP   /*!< The counter clock of TIM16 is stopped when the core is in debug mode  */
#endif /* TIM16 */
#if defined(TIM17)
#define LL_DBGMCU_TIM17_STOP     DBGMCU_APB2FZR_DBG_TIM17_STOP   /*!< The counter clock of TIM17 is stopped when the core is in debug mode  */
#endif /* TIM17 */
/**
  * @}
  */

/** @defgroup DBGMCU_LL_EC_APB3_GRP1_STOP_IP DBGMCU APB3_GRP1 STOP IP
  * @{
  */
#if defined(LPTIM1)
#define LL_DBGMCU_LPTIM1_STOP    DBGMCU_APB3FZR_DBG_LPTIM1_STOP  /*!< The counter clock of LPTIM1 is stopped when the core is in debug mode */
#endif /* LPTIM1 */
#define LL_DBGMCU_RTC_STOP       DBGMCU_APB3FZR_DBG_RTC_STOP     /*!< The counter clock of RTC is stopped when the core is in debug mode    */
/**
  * @}
  */

/** @defgroup DBGMCU_LL_EC_AHB1_GRP1_STOP_IP DBGMCU AHB1_GRP1 STOP IP
  * @{
  */
#define LL_DBGMCU_LPDMA1_CH0_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_0_STOP  /*!< The counter clock of LPDMA1 channel 0 is stopped when the core is in debug mode */
#define LL_DBGMCU_LPDMA1_CH1_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_1_STOP  /*!< The counter clock of LPDMA1 channel 1 is stopped when the core is in debug mode */
#define LL_DBGMCU_LPDMA1_CH2_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_2_STOP  /*!< The counter clock of LPDMA1 channel 2 is stopped when the core is in debug mode */
#define LL_DBGMCU_LPDMA1_CH3_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_3_STOP  /*!< The counter clock of LPDMA1 channel 3 is stopped when the core is in debug mode */
#if defined(LPDMA1_CH4)
#define LL_DBGMCU_LPDMA1_CH4_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_4_STOP  /*!< The counter clock of LPDMA1 channel 4 is stopped when the core is in debug mode */
#endif /* LPDMA1_CH4 */
#if defined(LPDMA1_CH5)
#define LL_DBGMCU_LPDMA1_CH5_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_5_STOP  /*!< The counter clock of LPDMA1 channel 5 is stopped when the core is in debug mode */
#endif /* LPDMA1_CH5 */
#if defined(LPDMA1_CH6)
#define LL_DBGMCU_LPDMA1_CH6_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_6_STOP  /*!< The counter clock of LPDMA1 channel 6 is stopped when the core is in debug mode */
#endif /* LPDMA1_CH6 */
#if defined(LPDMA1_CH7)
#define LL_DBGMCU_LPDMA1_CH7_STOP    DBGMCU_AHB1FZR_DBG_LPDMA1_7_STOP  /*!< The counter clock of LPDMA1 channel 7 is stopped when the core is in debug mode */
#endif /* LPDMA1_CH7 */
#if defined(LPDMA2_CH0)
#define LL_DBGMCU_LPDMA2_CH0_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_0_STOP  /*!< The counter clock of LPDMA2 channel 0 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH0 */
#if defined(LPDMA2_CH1)
#define LL_DBGMCU_LPDMA2_CH1_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_1_STOP  /*!< The counter clock of LPDMA2 channel 1 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH1 */
#if defined(LPDMA2_CH2)
#define LL_DBGMCU_LPDMA2_CH2_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_2_STOP  /*!< The counter clock of LPDMA2 channel 2 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH2 */
#if defined(LPDMA2_CH3)
#define LL_DBGMCU_LPDMA2_CH3_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_3_STOP  /*!< The counter clock of LPDMA2 channel 3 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH3 */
#if defined(LPDMA2_CH4)
#define LL_DBGMCU_LPDMA2_CH4_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_4_STOP  /*!< The counter clock of LPDMA2 channel 4 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH4 */
#if defined(LPDMA2_CH5)
#define LL_DBGMCU_LPDMA2_CH5_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_5_STOP  /*!< The counter clock of LPDMA2 channel 5 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH5 */
#if defined(LPDMA2_CH6)
#define LL_DBGMCU_LPDMA2_CH6_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_6_STOP  /*!< The counter clock of LPDMA2 channel 6 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH6 */
#if defined(LPDMA2_CH7)
#define LL_DBGMCU_LPDMA2_CH7_STOP    DBGMCU_AHB1FZR_DBG_LPDMA2_7_STOP  /*!< The counter clock of LPDMA2 channel 7 is stopped when the core is in debug mode */
#endif /* LPDMA2_CH7 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup DBGMCU_LL_Exported_Macros LL DBGMCU Macros
  * @{
  */

/** @defgroup DBGMCU_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value to the DBGMCU register.
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_DBGMCU_WRITE_REG(reg, value) STM32_WRITE_REG((DBGMCU->reg), (value))

/**
  * @brief  Read a value from the DBGMCU register.
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_DBGMCU_READ_REG(reg) STM32_READ_REG(DBGMCU->reg)

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup DBGMCU_LL_Exported_Functions LL DBGMCU Functions
  * @{
  */

/** @defgroup DBGMCU_LL_EF_ID LL DBGMCU Identifications
  * @{
  */

/**
  * @brief  Return the device identifier.
  * @rmtoll
  *  IDCODE    DEV_ID    LL_DBGMCU_GetDeviceID
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DBGMCU_DEV_ID_C53X_C542 (*)
  *         @arg @ref LL_DBGMCU_DEV_ID_C55X_C562 (*)
  *         @arg @ref LL_DBGMCU_DEV_ID_C59X_C5A3 (*)
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetDeviceID(void)
{
  return (uint32_t)(STM32_READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_DEV_ID));
}

/**
  * @brief  Return the device revision identifier.
  * @rmtoll
  *  IDCODE    REV_ID    LL_DBGMCU_GetRevisionID
  * @note This field indicates the revision of the device.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DBGMCU_REV_ID_A
  *         @arg @ref LL_DBGMCU_REV_ID_Z
  *         @arg @ref LL_DBGMCU_REV_ID_Y
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetRevisionID(void)
{
  return (uint32_t)(STM32_READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_REV_ID) >> DBGMCU_IDCODE_REV_ID_Pos);
}

/**
  * @}
  */

/** @defgroup DBGMCU_LL_EF_LOW_POWER_MODE LL DBGMCU Debug Low power mode
  * @{
  */

/**
  * @brief  Enable the debug module during low power mode.
  * @rmtoll
  *  CR    DBG_SLEEP       LL_DBGMCU_EnableDebugLowPowerMode \n
  *  CR    DBG_STOP        LL_DBGMCU_EnableDebugLowPowerMode \n
  *  CR    DBG_STANDBY     LL_DBGMCU_EnableDebugLowPowerMode
  * @param mode This parameter can be one or a combination of the following values:
  *        @arg @ref LL_DBGMCU_SLEEP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STOP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STANDBY_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_LP_MODE_DEBUG_ALL
  */
__STATIC_INLINE void LL_DBGMCU_EnableDebugLowPowerMode(uint32_t mode)
{
  STM32_SET_BIT(DBGMCU->CR, mode);
}

/**
  * @brief  Disable the debug module during low power mode.
  * @rmtoll
  *  CR    DBG_SLEEP       LL_DBGMCU_DisableDebugLowPowerMode \n
  *  CR    DBG_STOP        LL_DBGMCU_DisableDebugLowPowerMode \n
  *  CR    DBG_STANDBY     LL_DBGMCU_DisableDebugLowPowerMode
  * @param mode This parameter can be one or a combination of the following values:
  *        @arg @ref LL_DBGMCU_SLEEP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STOP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STANDBY_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_LP_MODE_DEBUG_ALL
  */
__STATIC_INLINE void LL_DBGMCU_DisableDebugLowPowerMode(uint32_t mode)
{
  STM32_CLEAR_BIT(DBGMCU->CR, mode);
}

/**
  * @brief  Check whether the debug module during low power mode is enabled.
  * @rmtoll
  *  CR    DBG_SLEEP       LL_DBGMCU_IsEnabledDebugLowPowerMode \n
  *  CR    DBG_STOP        LL_DBGMCU_IsEnabledDebugLowPowerMode \n
  *  CR    DBG_STANDBY     LL_DBGMCU_IsEnabledDebugLowPowerMode
  * @param mode This parameter can be one of the following values:
  *        @arg @ref LL_DBGMCU_SLEEP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STOP_MODE_DEBUG
  *        @arg @ref LL_DBGMCU_STANDBY_MODE_DEBUG
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDebugLowPowerMode(uint32_t mode)
{
  return ((STM32_READ_BIT(DBGMCU->CR, mode) == (mode)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the debug module during Sleep mode.
  * @rmtoll
  *  CR    DBG_SLEEP    LL_DBGMCU_EnableDBGSleepMode
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGSleepMode(void)
{
  STM32_SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Disable the debug module during Sleep mode.
  * @rmtoll
  *  CR    DBG_SLEEP    LL_DBGMCU_DisableDBGSleepMode
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGSleepMode(void)
{
  STM32_CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Check whether the debug module during Sleep mode is enabled.
  * @rmtoll
  *  CR    DBG_SLEEP    LL_DBGMCU_IsEnabledDBGSleepMode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDBGSleepMode(void)
{
  return ((STM32_READ_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP) == (DBGMCU_CR_DBG_SLEEP)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the debug module during Stop mode.
  * @rmtoll
  *  CR    DBG_STOP    LL_DBGMCU_EnableDBGStopMode
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGStopMode(void)
{
  STM32_SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Disable the debug module during Stop mode.
  * @rmtoll
  *  CR    DBG_STOP    LL_DBGMCU_DisableDBGStopMode
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGStopMode(void)
{
  STM32_CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Check whether the debug module during Stop mode is enabled.
  * @rmtoll
  *  CR    DBG_STOP    LL_DBGMCU_IsEnabledDBGStopMode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDBGStopMode(void)
{
  return ((STM32_READ_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP) == (DBGMCU_CR_DBG_STOP)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the debug module during Standby mode.
  * @rmtoll
  *  CR    DBG_STANDBY    LL_DBGMCU_EnableDBGStandbyMode
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGStandbyMode(void)
{
  STM32_SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

/**
  * @brief  Disable the debug module during Standby mode.
  * @rmtoll
  *  CR    DBG_STANDBY    LL_DBGMCU_DisableDBGStandbyMode
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGStandbyMode(void)
{
  STM32_CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

/**
  * @brief  Check whether the debug module during Standby mode is enabled.
  * @rmtoll
  *  CR    DBG_STANDBY    LL_DBGMCU_IsEnabledDBGStandbyMode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDBGStandbyMode(void)
{
  return ((STM32_READ_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY) == (DBGMCU_CR_DBG_STANDBY)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup DBGMCU_LL_EF_FREEZE_UNFREEZE LL DBGMCU Freeze/Unfreeze
  * @{
  */

/**
  * @brief  Freeze APB1_GRP1 peripherals.
  * @rmtoll
  *  APB1LFZR    DBG_xxxx_STOP    LL_DBGMCU_APB1_GRP1_FreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_TIM2_STOP
  * @if (TIM3)
  *         @arg @ref LL_DBGMCU_TIM3_STOP (*)
  * @endif
  * @if (TIM4)
  *         @arg @ref LL_DBGMCU_TIM4_STOP (*)
  * @endif
  * @if (TIM5)
  *         @arg @ref LL_DBGMCU_TIM5_STOP (*)
  * @endif
  *         @arg @ref LL_DBGMCU_TIM6_STOP
  *         @arg @ref LL_DBGMCU_TIM7_STOP
  *         @arg @ref LL_DBGMCU_TIM12_STOP
  *         @arg @ref LL_DBGMCU_WWDG_STOP
  *         @arg @ref LL_DBGMCU_IWDG_STOP
  *         @arg @ref LL_DBGMCU_I2C1_STOP
  * @if (I2C2)
  *         @arg @ref LL_DBGMCU_I2C2_STOP (*)
  * @endif
  *         @arg @ref LL_DBGMCU_I3C1_STOP
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_SET_BIT_32(DBGMCU->APB1LFZR, periphs);
}

/**
  * @brief  Unfreeze APB1_GRP1 peripherals.
  * @rmtoll
  *  APB1LFZR    DBG_xxxx_STOP    LL_DBGMCU_APB1_GRP1_UnFreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_TIM2_STOP
  * @if (TIM3)
  *         @arg @ref LL_DBGMCU_TIM3_STOP (*)
  * @endif
  * @if (TIM4)
  *         @arg @ref LL_DBGMCU_TIM4_STOP (*)
  * @endif
  * @if (TIM5)
  *         @arg @ref LL_DBGMCU_TIM5_STOP (*)
  * @endif
  *         @arg @ref LL_DBGMCU_TIM6_STOP
  *         @arg @ref LL_DBGMCU_TIM7_STOP
  *         @arg @ref LL_DBGMCU_TIM12_STOP
  *         @arg @ref LL_DBGMCU_WWDG_STOP
  *         @arg @ref LL_DBGMCU_IWDG_STOP
  *         @arg @ref LL_DBGMCU_I2C1_STOP
  * @if (I2C2)
  *         @arg @ref LL_DBGMCU_I2C2_STOP (*)
  * @endif
  *         @arg @ref LL_DBGMCU_I3C1_STOP
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(DBGMCU->APB1LFZR, periphs);
}

/**
  * @brief  Freeze APB2_GRP1 peripherals.
  * @rmtoll
  *  APB2FZR    DBG_XXXX_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_TIM1_STOP
  *         @arg @ref LL_DBGMCU_TIM8_STOP
  *         @arg @ref LL_DBGMCU_TIM15_STOP
  * @if (TIM16)
  *         @arg @ref LL_DBGMCU_TIM16_STOP (*)
  * @endif
  * @if (TIM17)
  *         @arg @ref LL_DBGMCU_TIM17_STOP (*)
  * @endif
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_APB2_GRP1_FreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_SET_BIT_32(DBGMCU->APB2FZR, periphs);
}

/**
  * @brief  Unfreeze APB2_GRP1 peripherals.
  * @rmtoll
  *  APB2FZR    DBG_XXXX_STOP    LL_DBGMCU_APB2_GRP1_UnFreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_TIM1_STOP
  *         @arg @ref LL_DBGMCU_TIM8_STOP
  *         @arg @ref LL_DBGMCU_TIM15_STOP
  * @if (TIM16)
  *         @arg @ref LL_DBGMCU_TIM16_STOP (*)
  * @endif
  * @if (TIM17)
  *         @arg @ref LL_DBGMCU_TIM17_STOP (*)
  * @endif
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_APB2_GRP1_UnFreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(DBGMCU->APB2FZR, periphs);
}

/**
  * @brief  Freeze APB3_GRP1 peripherals.
  * @rmtoll
  *  APB3FZR    DBG_XXXX_STOP    LL_DBGMCU_APB3_GRP1_FreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
#if defined(LPTIM1)
  *         @arg @ref LL_DBGMCU_LPTIM1_STOP
#endif
  *         @arg @ref LL_DBGMCU_RTC_STOP
  */
__STATIC_INLINE void LL_DBGMCU_APB3_GRP1_FreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_SET_BIT_32(DBGMCU->APB3FZR, periphs);
}

/**
  * @brief  Unfreeze APB3_GRP1 peripherals.
  * @rmtoll
  *  APB3FZR    DBG_XXXX_STOP    LL_DBGMCU_APB3_GRP1_UnFreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
#if defined(LPTIM1)
  *         @arg @ref LL_DBGMCU_LPTIM1_STOP
#endif
  *         @arg @ref LL_DBGMCU_RTC_STOP
  */
__STATIC_INLINE void LL_DBGMCU_APB3_GRP1_UnFreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(DBGMCU->APB3FZR, periphs);
}

/**
  * @brief  Freeze AHB1_GRP1 peripherals.
  * @rmtoll
  *  AHB1FZR    DBG_XXXX_STOP    LL_DBGMCU_AHB1_GRP1_FreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_LPDMA1_CH0_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH1_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH2_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH3_STOP
  * @if (LPDMA1_CH4)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH4_STOP (*)
  * @endif
  * @if (LPDMA1_CH5)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH5_STOP (*)
  * @endif
  * @if (LPDMA1_CH6)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH6_STOP (*)
  * @endif
  * @if (LPDMA1_CH7)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH7_STOP (*)
  * @endif
  * @if (LPDMA2_CH0)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH0_STOP
  * @endif
  * @if (LPDMA2_CH1)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH1_STOP
  * @endif
  * @if (LPDMA2_CH2)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH2_STOP
  * @endif
  * @if (LPDMA2_CH3)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH3_STOP
  * @endif
  * @if (LPDMA2_CH4)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH4_STOP (*)
  * @endif
  * @if (LPDMA2_CH5)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH5_STOP (*)
  * @endif
  * @if (LPDMA2_CH6)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH6_STOP (*)
  * @endif
  * @if (LPDMA2_CH7)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH7_STOP (*)
  * @endif
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_AHB1_GRP1_FreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_SET_BIT_32(DBGMCU->AHB1FZR, periphs);
}

/**
  * @brief  Unfreeze  peripherals.
  * @rmtoll
  *  AHB1FZR    DBG_XXXX_STOP    LL_DBGMCU_AHB1_GRP1_UnFreezePeriph
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_LPDMA1_CH0_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH1_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH2_STOP
  *         @arg @ref LL_DBGMCU_LPDMA1_CH3_STOP
  * @if (LPDMA1_CH4)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH4_STOP (*)
  * @endif
  * @if (LPDMA1_CH5)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH5_STOP (*)
  * @endif
  * @if (LPDMA1_CH6)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH6_STOP (*)
  * @endif
  * @if (LPDMA1_CH7)
  *         @arg @ref LL_DBGMCU_LPDMA1_CH7_STOP (*)
  * @endif
  * @if (LPDMA2_CH0)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH0_STOP
  * @endif
  * @if (LPDMA2_CH1)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH1_STOP
  * @endif
  * @if (LPDMA2_CH2)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH2_STOP
  * @endif
  * @if (LPDMA2_CH3)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH3_STOP
  * @endif
  * @if (LPDMA2_CH4)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH4_STOP (*)
  * @endif
  * @if (LPDMA2_CH5)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH5_STOP (*)
  * @endif
  * @if (LPDMA2_CH6)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH6_STOP (*)
  * @endif
  * @if (LPDMA2_CH7)
  *         @arg @ref LL_DBGMCU_LPDMA2_CH7_STOP (*)
  * @endif
  *
  * @note   (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(DBGMCU->AHB1FZR, periphs);
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
#endif /* DBGMCU */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_LL_DBGMCU_H */
