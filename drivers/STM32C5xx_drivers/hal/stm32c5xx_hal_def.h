/**
  ******************************************************************************
  * @file    stm32c5xx_hal_def.h
  * @brief   This file contains HAL common defines, enumerations, macros, and
  *          structure definitions.
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
#ifndef STM32C5XX_HAL_DEF_H
#define STM32C5XX_HAL_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
#include <arm_cmse.h>
#endif /* __ARM_FEATURE_CMSE */

#include "stm32c5xx_hal_conf.h"
#include "stm32c5xx.h"
#include <stddef.h>
#include <math.h>

/* Private defines ------------------------------------------------------------*/
#define DEF_ARMCC_VERSION 6010050

/* Exported types  ------------------------------------------------------------*/
/** @defgroup HAL_Def_Exported_Types HAL Def Types
  * @{
  */

/**
  * @brief  HAL status structures definition.
  */
typedef enum
{
  HAL_OK            = 0xEAEAEAEAU, /* HAL operation completed successfully */
  HAL_ERROR         = 0xF5F5F5F5U, /* HAL operation completed with error   */
  HAL_BUSY          = 0x55555555U, /* HAL concurrent process ongoing       */
  HAL_INVALID_PARAM = 0xAAAAAAAAU, /* HAL invalid parameter                */
  HAL_TIMEOUT       = 0x5A5A5A5AU  /* HAL operation exceeds user timeout   */

} hal_status_t;
/**
  * @}
  */

/** @defgroup HAL_Def_Exported_Constants HAL Def Constants
  * @{
  */

/**
  * @brief   HAL PPP clock model activation definition
  * @details These defines are used inside each HAL_PPP_Init function to indicate
  *          the clock model to be used.
  *          This model is declared inside stm32c5xx_hal_conf.h like this:
  *          @code
  *           #define USE_HAL_PPP_CLK_ENABLE_MODEL  HAL_CLK_ENABLE_XXXX
  *          @endcode
  */
#define HAL_CLK_ENABLE_NO                  0U /*<! No clock activation in PPP                           */
#define HAL_CLK_ENABLE_PERIPH_ONLY         1U /*<! PERIPH clock activation only                         */
#define HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM   2U /*<! PERIPH clock activation including PWR and/or system  */
#define HAL_MAX_DELAY      0xFFFFFFFFU

#define HAL_TICK_INT_LOWEST_PRIORITY   ((1UL << __NVIC_PRIO_BITS) - 1UL)

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup HAL_Def_Exported_Macros HAL Def Macros
  * @{
  */

/** @brief Check the current peripheral handle state and move it to a new state atomically.
  * @param handle specifies the Peripheral Handle.
  * @param state_field specifies the state field within the Handle.
  * @param ppp_conditional_state state to be checked to authorize moving to the new state.
  * @param ppp_new_state new state to set.
  * @note  This macro can be used for the following purpose:
  *        - When the define USE_HAL_CHECK_PROCESS_STATE is set to "1", this macro allows checking the current
  *          handle state against a conditional state and, if true, set it to the desired new state.
  *          The check and update of the state are done using exclusive load/store instructions, making
  *          the operation atomic.
  *        - When the define USE_HAL_CHECK_PROCESS_STATE is not set to "1", this macro simply assigns the new
  *          handle state field to the new desired state without any check.
  * @retval HAL_BUSY if the define USE_HAL_CHECK_PROCESS_STATE is set to "1" and the current state does not match
  *         ppp_conditional_state.
  */
/* Workaround linked to CMSIS IAR issue EWARM-11901; a fix is expected from CMSIS Core. */
#if defined(__ICCARM__)
#define STM32_CLREX_TO_DEPRECATE() __ASM volatile("CLREX" ::: "memory")
#else
#define STM32_CLREX_TO_DEPRECATE() __CLREX()
#endif /* __ICCARM__ */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
#define HAL_CHECK_UPDATE_STATE(handle, state_field, ppp_conditional_state, ppp_new_state)                           \
  do {                                                                                                              \
    __DMB();                                                                                                        \
    do {                                                                                                            \
      /* Return HAL_BUSY if the status is not ready. */                                                             \
      if (__LDREXW((volatile uint32_t *)((uint32_t)&(handle)->state_field)) != (uint32_t)(ppp_conditional_state))   \
      {                                                                                                             \
        STM32_CLREX_TO_DEPRECATE(); /* Workaround linked to CMSIS IAR issue EWARM-11901 fix. */                     \
        return HAL_BUSY;                                                                                            \
      }                                                                                                             \
      /* If the state is ready, attempt to change the state to the new one. */                                      \
    } while (__STREXW((uint32_t)(ppp_new_state), (volatile uint32_t *)((uint32_t)&((handle)->state_field))) != 0U); \
    /* Do not start any other memory access until the memory barrier is complete. */                                \
    __DMB();                                                                                                        \
  } while (0)
#else
#define HAL_CHECK_UPDATE_STATE(handle, state_field, ppp_conditional_state, ppp_new_state) \
  (handle)->state_field = (ppp_new_state)
#endif /* USE_HAL_CHECK_PROCESS_STATE == 1 */
/**
  * @}
  */

/* Redirect to CMSIS macro. */
#ifndef __weak
#define __weak          __WEAK
#endif /* __weak */
#ifndef __ICCARM__
#ifndef __packed
#define __packed        __PACKED
#endif /* __packed */
#endif /* __ICCARM__ */
#ifndef __ALIGN_END
#define __ALIGN_END     __ALIGNED(4)
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */

/**
  * @brief  __RAM_FUNC definition.
  */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= DEF_ARMCC_VERSION)

/* ARM Compiler
   ------------
   RAM functions are defined using the toolchain options.
   Functions that are executed in RAM must reside in a separate source module.
   Use the 'Options for File' dialog to change the 'Code / Const'
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog.
*/
#define __RAM_FUNC hal_status_t

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
   RAM functions are defined using a specific toolchain keyword "__ramfunc".
*/
#define __RAM_FUNC __ramfunc hal_status_t

#elif defined   (  __GNUC__  )
/* GNU Compiler
*/
#define __RAM_FUNC hal_status_t  __attribute__((section(".RamFunc")))

#endif /* __ARMCC_VERSION */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_DEF_H */
