/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_wwdg.h
  * @brief   Header file of WWDG HAL module.
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
#ifndef STM32C5XX_HAL_WWDG_H
#define STM32C5XX_HAL_WWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_wwdg.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (WWDG)

/** @defgroup WWDG WWDG
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup WWDG_Exported_Constants HAL WWDG Constants
  * @{
  */

/** @defgroup WWDG_Time_Unit_Definition WWDG Time Unit Definition
  * @{
  */
#define HAL_WWDG_TIME_UNIT_US   0U   /*!< WWDG driver time unit in microseconds */
#define HAL_WWDG_TIME_UNIT_MS   1U   /*!< WWDG driver time unit in milliseconds */
#define HAL_WWDG_TIME_UNIT_S    2U   /*!< WWDG driver time unit in seconds      */

#ifndef USE_HAL_WWDG_TIME_UNIT
#define USE_HAL_WWDG_TIME_UNIT  HAL_WWDG_TIME_UNIT_MS /*!< Default time unit is milliseconds if not set */
#endif /* USE_HAL_WWDG_TIME_UNIT */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/

/** @defgroup WWDG_Exported_Types HAL WWDG Types
  * @{
  */

/**
  * @brief HAL WWDG instance definition.
  */
typedef enum
{
  HAL_WWDG1 = WWDG_BASE   /*!< WWDG instance */
} hal_wwdg_t;
/**
  * @brief HAL WWDG state definition.
  */
typedef enum
{
  HAL_WWDG_STATE_RESET = 0U, /*!< WWDG driver not initialized and not started  */
#if !defined(USE_HAL_WWDG_HARDWARE_START) || (USE_HAL_WWDG_HARDWARE_START != 1UL)
  HAL_WWDG_STATE_IDLE = (1U << 30U), /*!< WWDG driver initialized and not started */
#endif /* !USE_HAL_WWDG_HARDWARE_START */
  HAL_WWDG_STATE_ACTIVE = (1U << 31U) /*!< WWDG driver initialized and started */
} hal_wwdg_state_t;

typedef struct hal_wwdg_handle_s hal_wwdg_handle_t; /*!< WWDG Handle Type Definition */

#if defined(USE_HAL_WWDG_REGISTER_CALLBACKS) && (USE_HAL_WWDG_REGISTER_CALLBACKS == 1)
typedef void (*hal_wwdg_cb_t)(hal_wwdg_handle_t *hwwdg); /*!< Pointer to a WWDG callback function. */
#endif /* USE_HAL_WWDG_REGISTER_CALLBACKS */
/**
  * @brief  WWDG Handle structure definition.
  */
struct hal_wwdg_handle_s
{
  hal_wwdg_t instance; /*!< WWDG peripheral instance */
  uint32_t reload; /*!< WWDG value */
  volatile hal_wwdg_state_t global_state; /*!< WWDG state */
  uint32_t pclk_frequency_hz; /*!< PCLK frequency */
#if defined(USE_HAL_WWDG_REGISTER_CALLBACKS) && (USE_HAL_WWDG_REGISTER_CALLBACKS == 1)
  hal_wwdg_cb_t p_early_wakeup_cb; /*!< WWDG Early WakeUp Interrupt */
#endif /* USE_HAL_WWDG_REGISTER_CALLBACKS */
#if defined (USE_HAL_WWDG_USER_DATA) && (USE_HAL_WWDG_USER_DATA == 1)
  const void *p_user_data; /*!< User data pointer. */
#endif /* USE_HAL_WWDG_USER_DATA */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup WWDG_Exported_Functions  HAL WWDG Functions
  * @{
  */

/** @defgroup WWDG_Exported_Functions_Group1 Initialization and Start functions
  * @{
  */
hal_status_t HAL_WWDG_Init(hal_wwdg_handle_t *hwwdg, hal_wwdg_t instance);
hal_status_t HAL_WWDG_Start(hal_wwdg_handle_t *hwwdg, uint32_t min_time, uint32_t max_time, uint32_t ewi_status);
/**
  * @}
  */

/** @defgroup WWDG_Exported_Functions_Group2 I/O operation function
  * @{
  */
hal_status_t HAL_WWDG_Refresh(hal_wwdg_handle_t *hwwdg);
/**
  * @}
  */

/** @defgroup WWDG_Exported_Functions_Group3 State function
  * @{
  */
hal_wwdg_state_t HAL_WWDG_GetState(const hal_wwdg_handle_t *hwwdg);
/**
  * @}
  */

/** @defgroup WWDG_Exported_Functions_Group4 Set/Get item functions
  * @{
  */
uint32_t HAL_WWDG_GetMaxTime(const hal_wwdg_handle_t *hwwdg);
uint32_t HAL_WWDG_GetStep_us(const hal_wwdg_handle_t *hwwdg);
hal_status_t HAL_WWDG_SetMinTime(hal_wwdg_handle_t *hwwdg, uint32_t min_time);
uint32_t HAL_WWDG_GetMinTime(const hal_wwdg_handle_t *hwwdg);
/**
  * @}
  */

/** @defgroup WWDG_Exported_Functions_Group5 IRQ Handler/Callbacks/Register Callbacks functions
  * @{
  */
void HAL_WWDG_IRQHandler(hal_wwdg_handle_t *hwwdg);
void HAL_WWDG_EarlyWakeupCallback(hal_wwdg_handle_t *hwwdg);
#if defined(USE_HAL_WWDG_REGISTER_CALLBACKS) && (USE_HAL_WWDG_REGISTER_CALLBACKS == 1)
hal_status_t HAL_WWDG_RegisterEarlyWakeupCallback(hal_wwdg_handle_t *hwwdg, hal_wwdg_cb_t p_callback);
#endif /* USE_HAL_WWDG_REGISTER_CALLBACKS */
/**
  * @}
  */

#if defined (USE_HAL_WWDG_USER_DATA) && (USE_HAL_WWDG_USER_DATA == 1)
/** @defgroup WWDG_Exported_Functions_Group6 User Data Function
  * @{
  */
void HAL_WWDG_SetUserData(hal_wwdg_handle_t *hwwdg, const void *p_user_data);
const void *HAL_WWDG_GetUserData(const hal_wwdg_handle_t *hwwdg);
/**
  * @}
  */

#endif /* USE_HAL_WWDG_USER_DATA */

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

#endif /* STM32C5XX_HAL_WWDG_H */

