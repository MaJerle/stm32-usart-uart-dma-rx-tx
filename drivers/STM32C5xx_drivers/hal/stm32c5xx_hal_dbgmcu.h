/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_dbgmcu.h
  * @brief   Header file of DBGMCU HAL module.
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
#ifndef STM32C5XX_HAL_DBGMCU_H
#define STM32C5XX_HAL_DBGMCU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_dbgmcu.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (DBGMCU)
/** @defgroup DBGMCU DBGMCU
  * @{
  */

/* Exported variables ------------------------------------------------------------------------------------------------*/
/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup DBGMCU_Exported_Types HAL DBGMCU Types
  * @{
  */

/*! HAL DBGMCU DBGMCU Device ID enumeration definition */
typedef enum
{
  HAL_DBGMCU_DEV_ID_C53X_C542 = LL_DBGMCU_DEV_ID_C53X_C542, /*!< STM32C5 device ID for STM32C53x/542 */
  HAL_DBGMCU_DEV_ID_C55X_C562 = LL_DBGMCU_DEV_ID_C55X_C562, /*!< STM32C5 device ID for STM32C55x/562 */
  HAL_DBGMCU_DEV_ID_C59X_C5A3 = LL_DBGMCU_DEV_ID_C59X_C5A3  /*!< STM32C5 device ID for STM32C59x/5A3 */
} hal_dbgmcu_device_id_t;

/*! HAL DBGMCU debug in low power mode state enumeration definition */
typedef enum
{
  HAL_DBGMCU_DBG_LOW_POWER_MODE_DISABLED = 0U, /*!< Debug in low power mode
                                                   (Sleep, Stop and Standby modes) is disabled */
  HAL_DBGMCU_DBG_LOW_POWER_MODE_ENABLED  = 1U  /*!< Debug in low power mode
                                                   (Sleep, Stop and Standby modes) is enabled  */
} hal_dbgmcu_dbg_low_power_mode_status_t;
/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup DBGMCU_Exported_Constants HAL DBGMCU Constants
  * @{
  */

/** @defgroup DBGMCU_Low_Power_Mode_Debug DBGMCU Low power mode debug definition
  * @{
  */
#define HAL_DBGMCU_SLEEP_MODE_DEBUG   LL_DBGMCU_SLEEP_MODE_DEBUG   /*!< Debug during Sleep mode          */
#define HAL_DBGMCU_STOP_MODE_DEBUG    LL_DBGMCU_STOP_MODE_DEBUG    /*!< Debug during Stop mode           */
#define HAL_DBGMCU_STANDBY_MODE_DEBUG LL_DBGMCU_STANDBY_MODE_DEBUG /*!< Debug during Standby mode        */
#define HAL_DBGMCU_LP_MODE_DEBUG_ALL  LL_DBGMCU_LP_MODE_DEBUG_ALL  /*!< Debug during all Low power modes */
/**
  * @}
  */

/**
  * @}
  */
/* Exported macros ---------------------------------------------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup DBGMCU_Exported_Functions  HAL DBGMCU Functions
  * @{
  */

/** @defgroup DBGMCU_Exported_Functions_Group1 DBGMCU Device identification functions
  * @{
  */
uint32_t HAL_DBGMCU_GetRevisionID(void);
hal_dbgmcu_device_id_t HAL_DBGMCU_GetDeviceID(void);
/**
  * @}
  */

/** @defgroup DBGMCU_Exported_Functions_Group2 DBGMCU Low power mode debug activation functions
  * @{
  */
void HAL_DBGMCU_EnableDebugLowPowerMode(uint32_t mode);
void HAL_DBGMCU_DisableDebugLowPowerMode(uint32_t mode);
hal_dbgmcu_dbg_low_power_mode_status_t HAL_DBGMCU_IsEnabledDebugLowPowerMode(uint32_t mode);
/**
  * @}
  */

/** @defgroup DBGMCU_Exported_Functions_Group3 DBGMCU Peripheral clock freeze and unfreeze functions
  * @{
  Use these functions to freeze and unfreeze the peripheral clock when the CPU is halted:
  - Call HAL_DBGMCU_PPPi_Freeze() to freeze the peripheral clock of PPPi when the CPU is halted.
  - Call HAL_DBGMCU_PPPi_UnFreeze() to unfreeze the peripheral clock of PPPi when the CPU is halted.
  */
/**
  * @details Freeze the clock of TIM1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM1_Freeze(void)
{
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_TIM1_STOP);
}

/**
  * @details Unfreeze the clock of TIM1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM1_UnFreeze(void)
{
  LL_DBGMCU_APB2_GRP1_UnFreezePeriph(LL_DBGMCU_TIM1_STOP);
}

/**
  * @details Freeze the clock of TIM2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM2_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM2_STOP);
}

/**
  * @details Unfreeze the clock of TIM2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM2_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM2_STOP);
}

#if defined(TIM3)
/**
  * @details Freeze the clock of TIM3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM3_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM3_STOP);
}

/**
  * @details Unfreeze the clock of TIM3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM3_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM3_STOP);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details Freeze the clock of TIM4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM4_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM4_STOP);
}

/**
  * @details Unfreeze the clock of TIM4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM4_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM4_STOP);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details Freeze the clock of TIM5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM5_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM5_STOP);
}

/**
  * @details Unfreeze the clock of TIM5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM5_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM5_STOP);
}

#endif /* TIM5 */
/**
  * @details Freeze the clock of TIM6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM6_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM6_STOP);
}

/**
  * @details Unfreeze the clock of TIM6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM6_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM6_STOP);
}

/**
  * @details Freeze the clock of TIM7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM7_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM7_STOP);
}

/**
  * @details Unfreeze the clock of TIM7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM7_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM7_STOP);
}

/**
  * @details Freeze the clock of TIM8 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM8_Freeze(void)
{
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_TIM8_STOP);
}

/**
  * @details Unfreeze the clock of TIM8 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM8_UnFreeze(void)
{
  LL_DBGMCU_APB2_GRP1_UnFreezePeriph(LL_DBGMCU_TIM8_STOP);
}

/**
  * @details Freeze the clock of TIM12 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM12_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_TIM12_STOP);
}

/**
  * @details Unfreeze the clock of TIM12 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM12_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_TIM12_STOP);
}

/**
  * @details Freeze the clock of TIM15 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM15_Freeze(void)
{
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_TIM15_STOP);
}

/**
  * @details Unfreeze the clock of TIM15 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM15_UnFreeze(void)
{
  LL_DBGMCU_APB2_GRP1_UnFreezePeriph(LL_DBGMCU_TIM15_STOP);
}

#if defined(TIM16)
/**
  * @details Freeze the clock of TIM16 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM16_Freeze(void)
{
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_TIM16_STOP);
}

/**
  * @details Unfreeze the clock of TIM16 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM16_UnFreeze(void)
{
  LL_DBGMCU_APB2_GRP1_UnFreezePeriph(LL_DBGMCU_TIM16_STOP);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details Freeze the clock of TIM17 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM17_Freeze(void)
{
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_TIM17_STOP);
}

/**
  * @details Unfreeze the clock of TIM17 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_TIM17_UnFreeze(void)
{
  LL_DBGMCU_APB2_GRP1_UnFreezePeriph(LL_DBGMCU_TIM17_STOP);
}

#endif /* TIM17 */
/**
  * @details Freeze the clock of I2C1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I2C1_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_I2C1_STOP);
}

/**
  * @details Unfreeze the clock of I2C1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I2C1_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_I2C1_STOP);
}

#if defined(I2C2)
/**
  * @details Freeze the clock of I2C2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I2C2_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_I2C2_STOP);
}

/**
  * @details Unfreeze the clock of I2C2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I2C2_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_I2C2_STOP);
}

#endif /* I2C2 */
/**
  * @details Freeze the clock of I3C1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I3C1_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_I3C1_STOP);
}

/**
  * @details Unfreeze the clock of I3C1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_I3C1_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_I3C1_STOP);
}

/**
  * @details Freeze the clock of WWDG when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_WWDG_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_WWDG_STOP);
}

/**
  * @details Unfreeze the clock of WWDG when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_WWDG_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_WWDG_STOP);
}

/**
  * @details Freeze the clock of IWDG when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_IWDG_Freeze(void)
{
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_IWDG_STOP);
}

/**
  * @details Unfreeze the clock of IWDG when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_IWDG_UnFreeze(void)
{
  LL_DBGMCU_APB1_GRP1_UnFreezePeriph(LL_DBGMCU_IWDG_STOP);
}

/**
  * @details Freeze the clock of RTC when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_RTC_Freeze(void)
{
  LL_DBGMCU_APB3_GRP1_FreezePeriph(LL_DBGMCU_RTC_STOP);
}

/**
  * @details Unfreeze the clock of RTC when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_RTC_UnFreeze(void)
{
  LL_DBGMCU_APB3_GRP1_UnFreezePeriph(LL_DBGMCU_RTC_STOP);
}

#if defined(LPTIM1)
/**
  * @details Freeze the clock of LPTIM1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPTIM1_Freeze(void)
{
  LL_DBGMCU_APB3_GRP1_FreezePeriph(LL_DBGMCU_LPTIM1_STOP);
}

/**
  * @details Unfreeze the clock of LPTIM1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPTIM1_UnFreeze(void)
{
  LL_DBGMCU_APB3_GRP1_UnFreezePeriph(LL_DBGMCU_LPTIM1_STOP);
}

#endif /* LPTIM1 */
/**
  * @details Freeze the clock of LPDMA1 channel 0 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch0_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH0_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 0 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch0_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH0_STOP);
}

/**
  * @details Freeze the clock of LPDMA1 channel 1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch1_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH1_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch1_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH1_STOP);
}

/**
  * @details Freeze the clock of LPDMA1 channel 2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch2_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH2_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch2_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH2_STOP);
}

/**
  * @details Freeze the clock of LPDMA1 channel 3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch3_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH3_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch3_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH3_STOP);
}

#if defined(LPDMA1_CH4)
/**
  * @details Freeze the clock of LPDMA1 channel 4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch4_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH4_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch4_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH4_STOP);
}

#endif /* LPDMA1_CH4 */
#if defined(LPDMA1_CH5)
/**
  * @details Freeze the clock of LPDMA1 channel 5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch5_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH5_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch5_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH5_STOP);
}

#endif /* LPDMA1_CH5 */
#if defined(LPDMA1_CH6)
/**
  * @details Freeze the clock of LPDMA1 channel 6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch6_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH6_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch6_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH6_STOP);
}

#endif /* LPDMA1_CH6 */
#if defined(LPDMA1_CH7)
/**
  * @details Freeze the clock of LPDMA1 channel 7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch7_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA1_CH7_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA1 channel 7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA1_Ch7_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA1_CH7_STOP);
}

#endif /* LPDMA1_CH7 */
#if defined(LPDMA2)
/**
  * @details Freeze the clock of LPDMA2 channel 0 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch0_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH0_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 0 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch0_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH0_STOP);
}

/**
  * @details Freeze the clock of LPDMA2 channel 1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch1_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH1_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 1 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch1_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH1_STOP);
}

/**
  * @details Freeze the clock of LPDMA2 channel 2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch2_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH2_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 2 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch2_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH2_STOP);
}

/**
  * @details Freeze the clock of LPDMA2 channel 3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch3_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH3_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 3 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch3_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH3_STOP);
}

#if defined(LPDMA2_CH4)
/**
  * @details Freeze the clock of LPDMA2 channel 4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch4_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH4_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 4 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch4_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH4_STOP);
}

#endif /* LPDMA2_CH4 */
#if defined(LPDMA2_CH5)
/**
  * @details Freeze the clock of LPDMA2 channel 5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch5_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH5_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 5 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch5_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH5_STOP);
}

#endif /* LPDMA2_CH5 */
#if defined(LPDMA2_CH6)
/**
  * @details Freeze the clock of LPDMA2 channel 6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch6_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH6_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 6 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch6_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH6_STOP);
}

#endif /* LPDMA2_CH6 */
#if defined(LPDMA2_CH7)
/**
  * @details Freeze the clock of LPDMA2 channel 7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch7_Freeze(void)
{
  LL_DBGMCU_AHB1_GRP1_FreezePeriph(LL_DBGMCU_LPDMA2_CH7_STOP);
}

/**
  * @details Unfreeze the clock of LPDMA2 channel 7 when the CPU is halted.
  */
__STATIC_INLINE void HAL_DBGMCU_LPDMA2_Ch7_UnFreeze(void)
{
  LL_DBGMCU_AHB1_GRP1_UnFreezePeriph(LL_DBGMCU_LPDMA2_CH7_STOP);
}

#endif /* LPDMA2_CH7 */
#endif /* LPDMA2 */
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
#endif

#endif /* STM32C5XX_HAL_DBGMCU_H */
