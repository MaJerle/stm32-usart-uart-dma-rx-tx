/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_wwdg.h
  * @brief   Header file of WWDG LL module.
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
#ifndef STM32C5XX_LL_WWDG_H
#define STM32C5XX_LL_WWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined (WWDG)

/** @defgroup WWDG_LL WWDG
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup WWDG_LL_Exported_Constants LL WWDG Constants
  * @{
  */

/** @defgroup WWDG_LL_EC_IT IT Defines
  * @brief    IT defines that can be used with LL_WWDG_ReadReg and LL_WWDG_WriteReg functions.
  * @{
  */
#define LL_WWDG_CFR_EWI         WWDG_CFR_EWI
/**
  * @}
  */

/** @defgroup WWDG_LL_EC_PRESCALER  PRESCALER
  * @{
  */
#define LL_WWDG_PRESCALER_1   0x00000000U                             /*!< WWDG counter clock = (PCLK/4096)/1   */
#define LL_WWDG_PRESCALER_2   WWDG_CFR_WDGTB_0                        /*!< WWDG counter clock = (PCLK/4096)/2   */
#define LL_WWDG_PRESCALER_4   WWDG_CFR_WDGTB_1                        /*!< WWDG counter clock = (PCLK/4096)/4   */
#define LL_WWDG_PRESCALER_8   (WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1)   /*!< WWDG counter clock = (PCLK/4096)/8   */
#define LL_WWDG_PRESCALER_16  WWDG_CFR_WDGTB_2                        /*!< WWDG counter clock = (PCLK/4096)/16  */
#define LL_WWDG_PRESCALER_32  (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_0)   /*!< WWDG counter clock = (PCLK/4096)/32  */
#define LL_WWDG_PRESCALER_64  (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1)   /*!< WWDG counter clock = (PCLK/4096)/64  */
#define LL_WWDG_PRESCALER_128 (WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1 \
                               | WWDG_CFR_WDGTB_0)                    /*!< WWDG counter clock = (PCLK/4096)/128 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup WWDG_LL_Exported_Macros LL WWDG Macros
  * @{
  */

/** @defgroup WWDG_LL_EM_WRITE_READ Common write and read register macros
  * @{
  */

/**
  * @brief Write a value in the WWDG register.
  * @param instance WWDG Instance.
  * @param reg Register to be written.
  * @param value Value to be written in the register.
  */
#define LL_WWDG_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief Read a value from the WWDG register.
  * @param instance WWDG Instance.
  * @param reg Register to be read.
  * @retval Register value.
  */
#define LL_WWDG_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup WWDG_LL_Exported_Functions LL WWDG Functions
  * @{
  */

/** @defgroup WWDG_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief Enable Window Watchdog. The watchdog is always disabled after a reset.
  * @rmtoll
  *  CR           WDGA          LL_WWDG_Enable
  * @param wwdgx WWDG Instance.
  * @note   It is enabled by setting the WDGA bit in the WWDG_CR register,
  *         and it cannot be disabled again except by a reset.
  *         This bit is set by software and only cleared by hardware after a reset.
  *         When WDGA = 1, the watchdog can generate a reset.
  */
__STATIC_INLINE void LL_WWDG_Enable(WWDG_TypeDef *wwdgx)
{
  STM32_SET_BIT(wwdgx->CR, WWDG_CR_WDGA);
}

/**
  * @brief Check whether Window Watchdog is enabled.
  * @rmtoll
  *  CR           WDGA          LL_WWDG_IsEnabled
  * @param wwdgx WWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_WWDG_IsEnabled(const WWDG_TypeDef *wwdgx)
{
  return ((STM32_READ_BIT(wwdgx->CR, WWDG_CR_WDGA) == (WWDG_CR_WDGA)) ? 1UL : 0UL);
}

/**
  * @brief Set the Watchdog counter value to the provided value (7-bit T[6:0]).
  * @rmtoll
  *  CR           T             LL_WWDG_SetCounter
  * @param wwdgx   WWDG Instance
  * @param counter 0..0x7F (7-bit counter value)
  * @warning When writing to the WWDG_CR register, always write 1 to the MSB b6 to avoid generating an immediate reset.
  *          This counter is decremented every (4096 x 2expWDGTB) PCLK cycles.
  *          A reset is produced when it rolls over from 0x40 to 0x3F (bit T6 becomes cleared).
  *          Setting the counter lower than 0x40 causes an immediate reset (if WWDG is enabled).
  */
__STATIC_INLINE void LL_WWDG_SetCounter(WWDG_TypeDef *wwdgx, uint32_t counter)
{
  STM32_MODIFY_REG(wwdgx->CR, WWDG_CR_T, counter);
}

/**
  * @brief  Return the current Watchdog counter value (7-bit counter value).
  * @rmtoll
  *  CR           T             LL_WWDG_GetCounter
  * @param  wwdgx WWDG Instance
  * @retval 7-bit Watchdog counter value.
  */
__STATIC_INLINE uint32_t LL_WWDG_GetCounter(const WWDG_TypeDef *wwdgx)
{
  return (STM32_READ_BIT(wwdgx->CR, WWDG_CR_T));
}

/**
  * @brief Set the time base of the prescaler (WDGTB).
  * @rmtoll
  *  CFR          WDGTB         LL_WWDG_SetPrescaler
  * @param wwdgx     WWDG Instance
  * @param prescaler This parameter can be one of the following values:
  *        @arg @ref LL_WWDG_PRESCALER_1
  *        @arg @ref LL_WWDG_PRESCALER_2
  *        @arg @ref LL_WWDG_PRESCALER_4
  *        @arg @ref LL_WWDG_PRESCALER_8
  *        @arg @ref LL_WWDG_PRESCALER_16
  *        @arg @ref LL_WWDG_PRESCALER_32
  *        @arg @ref LL_WWDG_PRESCALER_64
  *        @arg @ref LL_WWDG_PRESCALER_128
  * @note Prescaler is used to apply a ratio to the PCLK clock, so that the Watchdog counter
  *       is decremented every (4096 x 2expWDGTB) PCLK cycles.
  */
__STATIC_INLINE void LL_WWDG_SetPrescaler(WWDG_TypeDef *wwdgx, uint32_t prescaler)
{
  STM32_MODIFY_REG(wwdgx->CFR, WWDG_CFR_WDGTB, prescaler);
}

/**
  * @brief Return the current Watchdog prescaler value.
  * @rmtoll
  *  CFR          WDGTB         LL_WWDG_GetPrescaler
  * @param wwdgx WWDG Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_WWDG_PRESCALER_1
  *         @arg @ref LL_WWDG_PRESCALER_2
  *         @arg @ref LL_WWDG_PRESCALER_4
  *         @arg @ref LL_WWDG_PRESCALER_8
  *         @arg @ref LL_WWDG_PRESCALER_16
  *         @arg @ref LL_WWDG_PRESCALER_32
  *         @arg @ref LL_WWDG_PRESCALER_64
  *         @arg @ref LL_WWDG_PRESCALER_128
  */
__STATIC_INLINE uint32_t LL_WWDG_GetPrescaler(const WWDG_TypeDef *wwdgx)
{
  return (STM32_READ_BIT(wwdgx->CFR, WWDG_CFR_WDGTB));
}

/**
  * @brief Set the Watchdog window value to be compared to the downcounter (7-bit W[6:0]).
  * @rmtoll
  *  CFR          W             LL_WWDG_SetWindow
  * @param wwdgx  WWDG Instance.
  * @param window 0x00..0x7F (7-bit Window value)
  * @note This window value defines when a write in the WWDG_CR register
  *       to program the Watchdog counter is allowed.
  *       Update the Watchdog counter value only when the counter value is lower
  *       than the Watchdog window register value.
  *       Otherwise, an MCU reset is generated if the 7-bit Watchdog counter value
  *       (in the control register) is refreshed before the downcounter has reached
  *       the watchdog window register value.
  *       It is possible to set the Window lower than 0x40, but it is not recommended.
  *       To generate an immediate reset, it is possible to set the Counter lower than 0x40.
  */
__STATIC_INLINE void LL_WWDG_SetWindow(WWDG_TypeDef *wwdgx, uint32_t window)
{
  STM32_MODIFY_REG(wwdgx->CFR, WWDG_CFR_W, window);
}

/**
  * @brief Return the current Watchdog window value (7-bit value).
  * @rmtoll
  *  CFR          W             LL_WWDG_GetWindow
  * @param wwdgx WWDG Instance.
  * @retval 7-bit Watchdog window value.
  */
__STATIC_INLINE uint32_t LL_WWDG_GetWindow(const WWDG_TypeDef *wwdgx)
{
  return (STM32_READ_BIT(wwdgx->CFR, WWDG_CFR_W));
}
/**
  * @}
  */

/** @defgroup WWDG_LL_EF_FLAG_Management FLAG_Management
  * @{
  */
/**
  * @brief Indicate whether the WWDG Early Wakeup Interrupt Flag is set.
  * @rmtoll
  *  SR           EWIF          LL_WWDG_IsActiveFlag_EWKUP
  * @param wwdgx WWDG Instance
  * @note This bit is set by hardware when the counter has reached the value 0x40.
  *       Clear it by software by writing 0.
  *       A write of 1 has no effect. This bit is also set if the interrupt is not enabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_WWDG_IsActiveFlag_EWKUP(const WWDG_TypeDef *wwdgx)
{
  return ((STM32_READ_BIT(wwdgx->SR, WWDG_SR_EWIF) == (WWDG_SR_EWIF)) ? 1UL : 0UL);
}

/**
  * @brief Clear WWDG Early Wakeup Interrupt Flag (EWIF).
  * @rmtoll
  *  SR           EWIF          LL_WWDG_ClearFlag_EWKUP
  * @param wwdgx WWDG Instance
  */
__STATIC_INLINE void LL_WWDG_ClearFlag_EWKUP(WWDG_TypeDef *wwdgx)
{
  STM32_WRITE_REG(wwdgx->SR, ~WWDG_SR_EWIF);
}
/**
  * @}
  */

/** @defgroup WWDG_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief Enable the Early Wakeup Interrupt.
  * @rmtoll
  *  CFR          EWI           LL_WWDG_EnableIT_EWKUP
  * @param wwdgx WWDG Instance
  * @note When set, an interrupt occurs whenever the counter reaches value 0x40.
  *       This interrupt is only cleared by hardware after a reset.
  */
__STATIC_INLINE void LL_WWDG_EnableIT_EWKUP(WWDG_TypeDef *wwdgx)
{
  STM32_SET_BIT(wwdgx->CFR, WWDG_CFR_EWI);
}

/**
  * @brief Check if Early Wakeup Interrupt is enabled.
  * @rmtoll
  *  CFR          EWI           LL_WWDG_IsEnabledIT_EWKUP
  * @param wwdgx WWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_WWDG_IsEnabledIT_EWKUP(const WWDG_TypeDef *wwdgx)
{
  return ((STM32_READ_BIT(wwdgx->CFR, WWDG_CFR_EWI) == (WWDG_CFR_EWI)) ? 1UL : 0UL);
}

/**
  * @brief Enable Window Watchdog and set the counter value to the provided value (7-bit T[6:0]).
  * @rmtoll
  *  CR         WDGA        LL_WWDG_SetControl \n
  *  CR         T           LL_WWDG_SetControl
  * @param wwdgx WWDG Instance
  * @param counter 0..0x7F (7-bit counter value)
  */
__STATIC_INLINE void LL_WWDG_SetControl(WWDG_TypeDef *wwdgx, uint32_t counter)
{
  STM32_WRITE_REG(wwdgx->CR, WWDG_CR_WDGA | counter);
}

/**
  * @brief Set the time base of the prescaler (WDGTB) and the Watchdog window value to be compared to the downcounter
  *         (7-bit W[6:0]).
  * @rmtoll
  *  CFR        WDGTB        LL_WWDG_SetConfig \n
  *  CFR        W            LL_WWDG_SetConfig
  * @param wwdgx     WWDG Instance
  * @param prescaler This parameter can be one of the following values:
  *        @arg @ref LL_WWDG_PRESCALER_1
  *        @arg @ref LL_WWDG_PRESCALER_2
  *        @arg @ref LL_WWDG_PRESCALER_4
  *        @arg @ref LL_WWDG_PRESCALER_8
  *        @arg @ref LL_WWDG_PRESCALER_16
  *        @arg @ref LL_WWDG_PRESCALER_32
  *        @arg @ref LL_WWDG_PRESCALER_64
  *        @arg @ref LL_WWDG_PRESCALER_128
  * @param window 0x00..0x7F (7-bit Window value)
  */
__STATIC_INLINE void LL_WWDG_SetConfig(WWDG_TypeDef *wwdgx, uint32_t prescaler, uint32_t window)
{
  STM32_WRITE_REG(wwdgx->CFR, window | prescaler);
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

#endif /* WWDG */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_WWDG_H */
