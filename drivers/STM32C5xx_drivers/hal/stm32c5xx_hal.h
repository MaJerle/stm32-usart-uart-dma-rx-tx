/**
  ******************************************************************************
  * @file    stm32c5xx_hal.h
  * @brief   Header file of the HAL module.
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
#ifndef STM32C5XX_HAL_H
#define STM32C5XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_system.h"
#include "stm32c5xx_ll_utils.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup HAL HAL
  * @{
  */

/* Exported constants ---------------------------------------------------------*/
/** @defgroup HAL_Exported_Constants HAL Constants
  * @{
  */

/** @defgroup  STM32C5xx HAL driver version
  * @{
  */
#define HAL_VERSION_MAIN   (2UL) /*!< HAL driver version bitfield [31:24]: main version */
#define HAL_VERSION_SUB1   (1UL) /*!< HAL driver version bitfield [23:16]: sub1 version */
#define HAL_VERSION_SUB2   (0UL) /*!< HAL driver version bitfield [15:8]:  sub2 version */
#define HAL_VERSION_RC     (0UL) /*!< HAL driver version bitfield [7:0]:   release candidate */
#define HAL_VERSION         ((HAL_VERSION_MAIN << 24U)\
                             |(HAL_VERSION_SUB1 << 16U)\
                             |(HAL_VERSION_SUB2 << 8U )\
                             |(HAL_VERSION_RC)) /*!< HAL driver version. */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HAL_Exported_Types HAL Types
  * @{
  */

/**
  * @brief HAL tick frequency.
  * @note  Literal names mention the unit "Hz", but their values are in milliseconds because they are used
  *        with incrementation for delay computation.
  */
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,              /*!< HAL tick frequency 10 Hz */
  HAL_TICK_FREQ_100HZ        =  10U,              /*!< HAL tick frequency 100 Hz */
  HAL_TICK_FREQ_1KHZ         =   1U,              /*!< HAL tick frequency 1 kHz */
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ /*!< HAL tick default frequency: 1 kHz */
} hal_tick_freq_t;

/**
  * @brief Device unique identification data.
  */
typedef struct
{
  uint32_t              uid_0;             /*!< Device unique ID word 0: wafer {x; y} coordinate value */
  uint32_t              uid_1;             /*!< Device unique ID word 1: wafer number and lot number (part 2) */
  uint32_t              uid_2;             /*!< Device unique ID word 2: lot number (part 1) */
} hal_device_uid_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/** @defgroup HAL_Exported_Variables HAL Exported Variables
  * @{
  */
extern volatile uint32_t        uwTick;     /*!< HAL tick counter current value (unit: ms). */
extern uint32_t                 uwTickPrio; /*!< HAL tick interrupt priority. */
extern hal_tick_freq_t          uwTickFreq; /*!< HAL tick frequency. */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup HAL_Exported_Functions HAL Functions
  * @{
  */

/** @defgroup HAL_Exported_Functions_Group1 HAL initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_Init(void);
hal_status_t HAL_DeInit(void);
hal_status_t HAL_InitTick(hal_tick_freq_t tick_freq, uint32_t tick_priority);
hal_status_t HAL_UpdateCoreClock(void);
/**
  * @}
  */

/** @defgroup HAL_Exported_Functions_Group2 HAL time base control functions
  * @{
  */
void HAL_IncTick(void);
void HAL_Delay(uint32_t delay_ms);
void HAL_Delay_NoISR(uint32_t delay_ms);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
hal_tick_freq_t HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
/**
  * @}
  */

/** @defgroup HAL_Exported_Functions_Group3 HAL driver version
  * @{
  */
uint32_t HAL_GetVersion(void);
/**
  * @}
  */

/** @defgroup HAL_Exported_Functions_Group4 HAL device identification
  * @{
  */
hal_status_t HAL_GetDeviceUniqueID(hal_device_uid_t *p_uid);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_H */
