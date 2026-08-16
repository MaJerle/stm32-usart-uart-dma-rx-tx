/**
  ******************************************************************************
  * @file    stm32c5xx_dlyb_core.h
  * @brief   Header file of DelayBlock module.
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

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_DLYB_CORE_H
#define STM32C5XX_DLYB_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
/** @defgroup DLYB_CORE DLYB Core
  * @{
  */
#if defined(DLYB_XSPI1)
/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup DLYB_Exported_Types DLYB Exported Types
  * @{
  */
/**
  * @brief DLYB CORE Status definition
  */
typedef enum
{
  DLYB_CORE_OK    = 0x00000000U, /* DLYB CORE operation completed successfully */
  DLYB_CORE_ERROR = 0xFFFFFFFFU  /* DLYB CORE operation completed with error   */
} dlyb_core_status_t;
/**
  * @}
  */
/*! DLYB state */

typedef enum
{
  DLYB_DISABLED = 0U, /*!< DLYB disabled */
  DLYB_ENABLED,       /*!< DLYB enabled  */
} dlyb_state_t;
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @defgroup DLYB_CORE_Exported_Functions DLYB Exported Functions
  * @{
  */

/** @defgroup DLYB_CORE_Group1 Output clock phase tuning functions
  * @{
  */
dlyb_core_status_t DLYB_ConfigureUnitDelay(DLYB_TypeDef *dlybx);
uint32_t DLYB_CalculateMaxOutputClockPhase(DLYB_TypeDef *dlybx);
/**
  * @}
  */

/** @defgroup DLYB_CORE_Group2 Set and Get Output Clock Phase Value Functions
  * @{
  */
void DLYB_SetOutputClockPhase(DLYB_TypeDef *dlybx, uint32_t clock_phase_value);
uint32_t DLYB_GetOutputClockPhase(const DLYB_TypeDef *dlybx);
/**
  * @}
  */

/** @defgroup DLYB_CORE_Group3 Set and Get DLYB CFGR register context
  * @{
  */
void DLYB_SetConfig(DLYB_TypeDef *dlybx, uint32_t unit, uint32_t sel);
void DLYB_GetConfig(const DLYB_TypeDef *dlybx, uint32_t *p_unit, uint32_t *p_sel);
/**
  * @}
  */

/** @defgroup DLYB_CORE_Group4 Enable and Disable Delay Block functions
  * @{
  */
/**
  * @brief Enable the delay block peripheral.
  * @param dlybx DLYB Instance.
  */
__STATIC_INLINE void DLYB_Enable(DLYB_TypeDef *dlybx)
{
  STM32_SET_BIT(dlybx->CR, DLYB_CR_DEN);
}

/**
  * @brief Disable the delay block peripheral.
  * @param dlybx DLYB Instance.
  */
__STATIC_INLINE void DLYB_Disable(DLYB_TypeDef *dlybx)
{
  STM32_CLEAR_BIT(dlybx->CR, DLYB_CR_DEN);
}

/**
  * @brief Check whether the delay block peripheral is enabled or not.
  * @param dlybx DLYB Instance.
  * @retval DLYB_ENABLED  if the delay block peripheral is enabled.
  * @retval DLYB_DISABLED if the delay block peripheral is disabled.
  */
__STATIC_INLINE dlyb_state_t DLYB_IsEnabled(const DLYB_TypeDef *dlybx)
{
  return ((STM32_READ_BIT(dlybx->CR, DLYB_CR_DEN) == DLYB_CR_DEN) ? DLYB_ENABLED : DLYB_DISABLED);
}
/**
  * @}
  */

/**
  * @}
  */
#endif /* DLYB_XSPI1 */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus

}

#endif /* __cplusplus */
#endif /* STM32C5XX_DLYB_CORE_H */
