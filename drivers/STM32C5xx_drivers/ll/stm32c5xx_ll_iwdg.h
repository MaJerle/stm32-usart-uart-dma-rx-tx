/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_iwdg.h
  * @brief   Header file for the IWDG LL module.
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
#ifndef STM32C5xx_LL_IWDG_H
#define STM32C5xx_LL_IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined (IWDG)

/** @defgroup IWDG_LL IWDG
  * @{
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_LL_Private_Constants IWDG Private Constants
  * @{
  */
#define LL_IWDG_KEY_RELOAD             0x0000AAAAU  /*!< IWDG Reload Counter Enable   */
#define LL_IWDG_KEY_ENABLE             0x0000CCCCU  /*!< IWDG Peripheral Enable       */
#define LL_IWDG_KEY_WR_ACCESS_ENABLE   0x00005555U  /*!< IWDG KR Write Access Enable  */
#define LL_IWDG_KEY_WR_ACCESS_DISABLE  0x00000000U  /*!< IWDG KR Write Access Disable */
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/* Exported types ----------------------------------------------------------------------------------------------------*/
/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_LL_Exported_Constants LL IWDG Constants
  * @{
  */

/** @defgroup IWDG_LL_EC_PRESCALER  Prescaler Divider
  * @{
  */
#define LL_IWDG_PRESCALER_4     0U                                            /*!< Divider by 4    */
#define LL_IWDG_PRESCALER_8     (IWDG_PR_PR_0)                                /*!< Divider by 8    */
#define LL_IWDG_PRESCALER_16    (IWDG_PR_PR_1)                                /*!< Divider by 16   */
#define LL_IWDG_PRESCALER_32    (IWDG_PR_PR_1 | IWDG_PR_PR_0)                 /*!< Divider by 32   */
#define LL_IWDG_PRESCALER_64    (IWDG_PR_PR_2)                                /*!< Divider by 64   */
#define LL_IWDG_PRESCALER_128   (IWDG_PR_PR_2 | IWDG_PR_PR_0)                 /*!< Divider by 128  */
#define LL_IWDG_PRESCALER_256   (IWDG_PR_PR_2 | IWDG_PR_PR_1)                 /*!< Divider by 256  */
#define LL_IWDG_PRESCALER_512   (IWDG_PR_PR_2 | IWDG_PR_PR_1 | IWDG_PR_PR_0)  /*!< Divider by 512  */
#define LL_IWDG_PRESCALER_1024  (IWDG_PR_PR_3)                                /*!< Divider by 1024 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_LL_Exported_Macros LL IWDG Macros
  * @{
  */

/** @defgroup IWDG_LL_EM_WRITE_READ Common write and read register macros
  * @{
  */

/**
  * @brief  Write a value to an IWDG register.
  * @param  instance IWDG Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_IWDG_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value from an IWDG register.
  * @param  instance IWDG Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_IWDG_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_LL_Exported_Functions LL IWDG Functions
  * @{
  */

/** @defgroup IWDG_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Start the Independent Watchdog.
  * @param  iwdgx IWDG Instance
  * @note   Except when the hardware watchdog option is selected
  * @rmtoll
  *  KR           KEY           LL_IWDG_Enable
  */
__STATIC_INLINE void LL_IWDG_Enable(IWDG_TypeDef *iwdgx)
{
  STM32_WRITE_REG(iwdgx->KR, LL_IWDG_KEY_ENABLE);
}

/**
  * @brief  Reload IWDG counter with value defined in the reload register.
  * @rmtoll
  *  KR           KEY           LL_IWDG_ReloadCounter
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_ReloadCounter(IWDG_TypeDef *iwdgx)
{
  STM32_WRITE_REG(iwdgx->KR, LL_IWDG_KEY_RELOAD);
}

/**
  * @brief  Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers.
  * @rmtoll
  *  KR           KEY           LL_IWDG_EnableWriteAccess
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_EnableWriteAccess(IWDG_TypeDef *iwdgx)
{
  STM32_WRITE_REG(iwdgx->KR, LL_IWDG_KEY_WR_ACCESS_ENABLE);
}

/**
  * @brief  Disable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers.
  * @rmtoll
  *  KR           KEY           LL_IWDG_DisableWriteAccess
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_DisableWriteAccess(IWDG_TypeDef *iwdgx)
{
  STM32_WRITE_REG(iwdgx->KR, LL_IWDG_KEY_WR_ACCESS_DISABLE);
}

/**
  * @brief  Select the prescaler of the IWDG.
  * @rmtoll
  *  PR           PR            LL_IWDG_SetPrescaler
  * @param  iwdgx IWDG Instance
  * @param  prescaler This parameter can be one of the following values:
  *         @arg @ref LL_IWDG_PRESCALER_4
  *         @arg @ref LL_IWDG_PRESCALER_8
  *         @arg @ref LL_IWDG_PRESCALER_16
  *         @arg @ref LL_IWDG_PRESCALER_32
  *         @arg @ref LL_IWDG_PRESCALER_64
  *         @arg @ref LL_IWDG_PRESCALER_128
  *         @arg @ref LL_IWDG_PRESCALER_256
  *         @arg @ref LL_IWDG_PRESCALER_512
  *         @arg @ref LL_IWDG_PRESCALER_1024
  */
__STATIC_INLINE void LL_IWDG_SetPrescaler(IWDG_TypeDef *iwdgx, uint32_t prescaler)
{
  STM32_WRITE_REG(iwdgx->PR, IWDG_PR_PR & prescaler);
}

/**
  * @brief  Get the selected prescaler of the IWDG.
  * @rmtoll
  *  PR           PR            LL_IWDG_GetPrescaler
  * @param  iwdgx IWDG Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_IWDG_PRESCALER_4
  *         @arg @ref LL_IWDG_PRESCALER_8
  *         @arg @ref LL_IWDG_PRESCALER_16
  *         @arg @ref LL_IWDG_PRESCALER_32
  *         @arg @ref LL_IWDG_PRESCALER_64
  *         @arg @ref LL_IWDG_PRESCALER_128
  *         @arg @ref LL_IWDG_PRESCALER_256
  *         @arg @ref LL_IWDG_PRESCALER_512
  *         @arg @ref LL_IWDG_PRESCALER_1024
  */
__STATIC_INLINE uint32_t LL_IWDG_GetPrescaler(const IWDG_TypeDef *iwdgx)
{
  return (STM32_READ_REG(iwdgx->PR));
}

/**
  * @brief  Specify the IWDG down-counter reload value.
  * @rmtoll
  *  RLR          RL            LL_IWDG_SetReloadCounter
  * @param  iwdgx IWDG Instance
  * @param  counter Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE void LL_IWDG_SetReloadCounter(IWDG_TypeDef *iwdgx, uint32_t counter)
{
  STM32_WRITE_REG(iwdgx->RLR, IWDG_RLR_RL & counter);
}

/**
  * @brief  Get the specified IWDG down-counter reload value.
  * @rmtoll
  *  RLR          RL            LL_IWDG_GetReloadCounter
  * @param  iwdgx IWDG Instance
  * @retval Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t LL_IWDG_GetReloadCounter(const IWDG_TypeDef *iwdgx)
{
  return (STM32_READ_REG(iwdgx->RLR));
}

/**
  * @brief  Specify high limit of the window value to be compared to the down-counter.
  * @rmtoll
  *  WINR         WIN           LL_IWDG_SetWindow
  * @param  iwdgx IWDG Instance
  * @param  window Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE void LL_IWDG_SetWindow(IWDG_TypeDef *iwdgx, uint32_t window)
{
  STM32_WRITE_REG(iwdgx->WINR, IWDG_WINR_WIN & window);
}

/**
  * @brief  Get the high limit of the window value specified.
  * @rmtoll
  *  WINR         WIN           LL_IWDG_GetWindow
  * @param  iwdgx IWDG Instance
  * @retval Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t LL_IWDG_GetWindow(const IWDG_TypeDef *iwdgx)
{
  return (STM32_READ_REG(iwdgx->WINR));
}
/**
  * @}
  */

/** @defgroup IWDG_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Specify comparator value that will be used to trig Early Wakeup interrupt.
  * @rmtoll
  *  EWCR         EWIT          LL_IWDG_SetEwiTime
  * @param  iwdgx IWDG Instance
  * @param  time Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE void LL_IWDG_SetEwiTime(IWDG_TypeDef *iwdgx, uint32_t time)
{
  STM32_MODIFY_REG(iwdgx->EWCR, IWDG_EWCR_EWIT, time);
}

/**
  * @brief  Get the Early Wakeup interrupt comparator value.
  * @rmtoll
  *  EWCR         EWIT          LL_IWDG_GetEwiTime
  * @param  iwdgx IWDG Instance
  * @retval Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t LL_IWDG_GetEwiTime(const IWDG_TypeDef *iwdgx)
{
  return (STM32_READ_BIT(iwdgx->EWCR, IWDG_EWCR_EWIT));
}

/**
  * @brief  Enable Early wakeup interrupt.
  * @rmtoll
  *  EWCR         EWIE          LL_IWDG_EnableIT_EWI
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_EnableIT_EWI(IWDG_TypeDef *iwdgx)
{
  STM32_SET_BIT(iwdgx->EWCR, IWDG_EWCR_EWIE);
}

/**
  * @brief  Disable Early wakeup interrupt.
  * @rmtoll
  *  EWCR         EWIE          LL_IWDG_DisableIT_EWI
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_DisableIT_EWI(IWDG_TypeDef *iwdgx)
{
  STM32_CLEAR_BIT(iwdgx->EWCR, IWDG_EWCR_EWIE);
}

/**
  * @brief  Indicates whether Early wakeup interrupt is enabled.
  * @rmtoll
  *  EWCR         EWIE          LL_IWDG_IsEnabledIT_EWI
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE uint32_t LL_IWDG_IsEnabledIT_EWI(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->EWCR, IWDG_EWCR_EWIE) == IWDG_EWCR_EWIE) ? 1UL : 0UL);
}
/**
  * @}
  */

/** @defgroup IWDG_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if flag Prescaler Value Update is set or not.
  * @rmtoll
  *  SR           PVU           LL_IWDG_IsActiveFlag_PVU
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_PVU(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_PVU) == IWDG_SR_PVU) ? 1UL : 0UL);
}

/**
  * @brief  Check if flag Reload Value Update is set or not.
  * @rmtoll
  *  SR           RVU           LL_IWDG_IsActiveFlag_RVU
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_RVU(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_RVU) == IWDG_SR_RVU) ? 1UL : 0UL);
}

/**
  * @brief  Check if flag Window Value Update is set or not.
  * @rmtoll
  *  SR           WVU           LL_IWDG_IsActiveFlag_WVU
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_WVU(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_WVU) == IWDG_SR_WVU) ? 1UL : 0UL);
}

/**
  * @brief  Check if flag EWI Value Update is set or not.
  * @rmtoll
  *  SR           EVU           LL_IWDG_IsActiveFlag_EWU
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_EWU(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_EWU) == IWDG_SR_EWU) ? 1UL : 0UL);
}

/**
  * @brief  Check if Prescaler, Reload, Window & Early Interrupt Value updates are completed or not.
  * @rmtoll
  *  SR           PVU           LL_IWDG_IsReady \n
  *  SR           RVU           LL_IWDG_IsReady \n
  *  SR           WVU           LL_IWDG_IsReady \n
  *  SR           EWU           LL_IWDG_IsReady
  * @param  iwdgx IWDG Instance
  * @retval State of bits (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsReady(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU | IWDG_SR_EWU) == 0U) ? 1UL : 0UL);
}


/**
  * @brief  Check if IWDG has been started or not.
  * @rmtoll
  *  SR           ONF           LL_IWDG_IsActiveFlag_ONF
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_ONF(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_ONF) == (IWDG_SR_ONF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if Early Wakeup interrupt flag is set or not.
  * @rmtoll
  *  SR           EWIF          LL_IWDG_IsActiveFlag_EWIF
  * @param  iwdgx IWDG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IWDG_IsActiveFlag_EWIF(const IWDG_TypeDef *iwdgx)
{
  return ((STM32_READ_BIT(iwdgx->SR, IWDG_SR_EWIF) == IWDG_SR_EWIF) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Early Wakeup interrupt flag.
  * @rmtoll
  *  ICR         EWIC          LL_IWDG_ClearFlag_EWIF
  * @param  iwdgx IWDG Instance
  */
__STATIC_INLINE void LL_IWDG_ClearFlag_EWIF(IWDG_TypeDef *iwdgx)
{
  STM32_WRITE_REG(iwdgx->ICR, IWDG_ICR_EWIC);
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

#endif /* IWDG */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5xx_LL_IWDG_H */
