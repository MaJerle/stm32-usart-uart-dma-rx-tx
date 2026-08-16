/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_iwdg.h
  * @brief   Header file for the IWDG HAL module.
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
#ifndef STM32C5xx_HAL_IWDG_H
#define STM32C5xx_HAL_IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_iwdg.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (IWDG)

/** @defgroup IWDG IWDG
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_Exported_Constants HAL IWDG Constants
  * @{
  */

/** @defgroup IWDG_Time_Unit_Definition IWDG Time Unit Definition
  * @{
  */
#define HAL_IWDG_TIME_UNIT_US  0UL  /*!< IWDG driver time unit in microseconds */
#define HAL_IWDG_TIME_UNIT_MS  1UL  /*!< IWDG driver time unit in milliseconds */
#define HAL_IWDG_TIME_UNIT_S   2UL  /*!< IWDG driver time unit in seconds      */

#ifndef USE_HAL_IWDG_TIME_UNIT
#define USE_HAL_IWDG_TIME_UNIT  HAL_IWDG_TIME_UNIT_MS  /*!< Default time unit is milliseconds if not set by the user */
#endif /* USE_HAL_IWDG_TIME_UNIT */
/**
  * @}
  */

/** @defgroup IWDG_LSI_Frequency_Definition IWDG LSI frequency definition
  * @{
  */
#define LSI_VALUE_DYNAMIC  0UL  /*!< LSI value is set by the user */

#ifndef USE_HAL_IWDG_LSI_FREQ
#define USE_HAL_IWDG_LSI_FREQ  LSI_VALUE  /*!< Default LSI value is LSI_VALUE */
#endif /* USE_HAL_IWDG_LSI_FREQ */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_Exported_Types HAL IWDG Types
  * @{
  */

/*! HAL IWDG instances enumeration definition */
typedef enum
{
  HAL_IWDG1 = IWDG_BASE  /*!< IWDG1 instance */
} hal_iwdg_t;

/*! HAL IWDG state enumeration definition */
typedef enum
{
  HAL_IWDG_STATE_RESET  = 0UL,           /*!< IWDG driver not initialized and not started */
#if !defined(USE_HAL_IWDG_HARDWARE_START) || (USE_HAL_IWDG_HARDWARE_START != 1UL)
  HAL_IWDG_STATE_IDLE   = (1UL << 30UL),  /*!< IWDG driver initialized and not started     */
#endif /* !USE_HAL_IWDG_HARDWARE_START */
  HAL_IWDG_STATE_ACTIVE = (1UL << 31UL)   /*!< IWDG driver initialized and started         */
} hal_iwdg_state_t;

typedef struct hal_iwdg_handle_s hal_iwdg_handle_t;  /*!<  IWDG handle type definition  */

#if defined(USE_HAL_IWDG_REGISTER_CALLBACKS) && (USE_HAL_IWDG_REGISTER_CALLBACKS == 1UL)
typedef void (*hal_iwdg_cb_t)(hal_iwdg_handle_t *hiwdg);  /*!< Pointer to an IWDG common callback function */
#endif /* USE_HAL_IWDG_REGISTER_CALLBACKS */

/*! HAL IWDG handle structure definition */
struct hal_iwdg_handle_s
{
  hal_iwdg_t                instance;          /*!< IWDG peripheral instance             */
  uint32_t                  reload;            /*!< IWDG reload value                    */
  volatile hal_iwdg_state_t global_state;      /*!< IWDG state                           */
#if (USE_HAL_IWDG_LSI_FREQ == LSI_VALUE_DYNAMIC)
  uint32_t                  lsi_frequency_hz;  /*!< IWDG LSI frequency                   */
#endif /* USE_HAL_IWDG_LSI_FREQ == LSI_VALUE_DYNAMIC */
#if defined(USE_HAL_IWDG_REGISTER_CALLBACKS) && (USE_HAL_IWDG_REGISTER_CALLBACKS == 1UL)
  hal_iwdg_cb_t p_early_wakeup_cb;             /*!< IWDG Early WakeUp Interrupt callback */
#endif /* USE_HAL_IWDG_REGISTER_CALLBACKS */
#if defined (USE_HAL_IWDG_USER_DATA) && (USE_HAL_IWDG_USER_DATA == 1UL)
  const void *p_user_data;                     /*!< IWDG user data                       */
#endif /* USE_HAL_IWDG_USER_DATA */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup IWDG_Exported_Functions HAL IWDG Functions
  * @{
  */

/** @defgroup IWDG_Exported_Functions_Group1 Initialization and start functions
  * @{
  */
hal_status_t HAL_IWDG_Init(hal_iwdg_handle_t *hiwdg, hal_iwdg_t instance);
hal_status_t HAL_IWDG_Start(hal_iwdg_handle_t *hiwdg, uint32_t min_time, uint32_t max_time, uint32_t early_wakeup_time);
/**
  * @}
  */

/** @defgroup IWDG_Exported_Functions_Group2 I/O operation functions
  * @{
  */
hal_status_t HAL_IWDG_Refresh(hal_iwdg_handle_t *hiwdg);
/**
  * @}
  */

/** @defgroup IWDG_Exported_Functions_Group3 State functions
  * @{
  */
hal_iwdg_state_t HAL_IWDG_GetState(const hal_iwdg_handle_t *hiwdg);
/**
  * @}
  */

/** @defgroup IWDG_Exported_Functions_Group4 Set and Get item functions
  * @{
  */
uint32_t HAL_IWDG_GetMaxTime(const hal_iwdg_handle_t *hiwdg);
uint32_t HAL_IWDG_GetStep_us(const hal_iwdg_handle_t *hiwdg);

hal_status_t HAL_IWDG_SetMinTime(hal_iwdg_handle_t *hiwdg, uint32_t time);
uint32_t HAL_IWDG_GetMinTime(const hal_iwdg_handle_t *hiwdg);

hal_status_t HAL_IWDG_SetEarlyWakeupInterruptTime(hal_iwdg_handle_t *hiwdg, uint32_t time);
uint32_t HAL_IWDG_GetEarlyWakeupInterruptTime(const hal_iwdg_handle_t *hiwdg);
#if (USE_HAL_IWDG_LSI_FREQ == LSI_VALUE_DYNAMIC)
hal_status_t HAL_IWDG_SetLSIFrequency(hal_iwdg_handle_t *hiwdg, uint32_t lsi_frequency_hz);
uint32_t HAL_IWDG_GetLSIFrequency(const hal_iwdg_handle_t *hiwdg);
#endif /* USE_HAL_IWDG_LSI_FREQ == LSI_VALUE_DYNAMIC */
/**
  * @}
  */

/** @defgroup IWDG_Exported_Functions_Group5 IRQ Handler/Callbacks/Register Callbacks functions
  * @{
  */
void HAL_IWDG_IRQHandler(hal_iwdg_handle_t *hiwdg);
void HAL_IWDG_EarlyWakeupCallback(hal_iwdg_handle_t *hiwdg);
#if defined(USE_HAL_IWDG_REGISTER_CALLBACKS) && (USE_HAL_IWDG_REGISTER_CALLBACKS == 1UL)
hal_status_t HAL_IWDG_RegisterEarlyWakeupCallback(hal_iwdg_handle_t *hiwdg, hal_iwdg_cb_t p_callback);
#endif /* USE_HAL_IWDG_REGISTER_CALLBACKS */
/**
  * @}
  */

#if defined (USE_HAL_IWDG_USER_DATA) && (USE_HAL_IWDG_USER_DATA == 1UL)
/** @defgroup IWDG_Exported_Functions_Group6 Set and Get User Data functions
  * @{
  */
void HAL_IWDG_SetUserData(hal_iwdg_handle_t *hiwdg, const void *p_user_data);
const void *HAL_IWDG_GetUserData(const hal_iwdg_handle_t *hiwdg);
/**
  * @}
  */
#endif /* USE_HAL_IWDG_USER_DATA */
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

#endif /* STM32C5xx_HAL_IWDG_H */
