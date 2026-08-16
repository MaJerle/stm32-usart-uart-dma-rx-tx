/**
  ******************************************************************************
  * @file    stm32c5xx.h
  * @brief   CMSIS STM32C5xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c.
  *          This file allows to select the STM32C5xx device used in the target
  *          application.
  *
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32c5xx
  * @{
  */

#ifndef STM32C5XX_H
#define STM32C5XX_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(USE_EXTERNAL_ENV)
#include "stm32_external_env.h"
#endif /* USE_EXTERNAL_ENV */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32C5)
#define STM32C5
#endif /* STM32C5 */

/**
  * @brief STM32C5xx CMSIS Device version number 2.1.0
  */
#define STM32C5_CMSIS_VERSION_MAIN   (2) /*!< [31:24] main version */
#define STM32C5_CMSIS_VERSION_SUB1   (1) /*!< [23:16] sub1 version */
#define STM32C5_CMSIS_VERSION_SUB2   (0) /*!< [15:8]  sub2 version */
#define STM32C5_CMSIS_VERSION_RC     (0) /*!< [7:0]  release candidate */
#define STM32C5_CMSIS_VERSION        ((STM32C5_CMSIS_VERSION_MAIN << 24U)   \
                                      | (STM32C5_CMSIS_VERSION_SUB1 << 16U) \
                                      | (STM32C5_CMSIS_VERSION_SUB2 << 8U ) \
                                      | (STM32C5_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Includes
  * @{
  */

#if defined(STM32C531xx)
#include "stm32c531xx.h"
#elif defined(STM32C532xx)
#include "stm32c532xx.h"
#elif defined(STM32C542xx)
#include "stm32c542xx.h"
#elif defined(STM32C551xx)
#include "stm32c551xx.h"
#elif defined(STM32C552xx)
#include "stm32c552xx.h"
#elif defined(STM32C562xx)
#include "stm32c562xx.h"
#elif defined(STM32C591xx)
#include "stm32c591xx.h"
#elif defined(STM32C593xx)
#include "stm32c593xx.h"
#elif defined(STM32C5A3xx)
#include "stm32c5a3xx.h"
#else
#error "Please select first the target STM32C5xx device used in your application"
#endif

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/* Privileged CPU state check */
#define STM32_IS_PRIVILEGED_EXECUTION() ((__get_CONTROL() & CONTROL_nPRIV_Msk) == 0U)

#define STM32_POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define STM32_UNUSED(x)             ((void)(x))

#define STM32_SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define STM32_CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define STM32_READ_BIT(REG, BIT)    ((REG) & (BIT))

#define STM32_CLEAR_REG(REG)        ((REG) = 0U)

#define STM32_WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define STM32_READ_REG(REG)         ((REG))

#define STM32_MODIFY_REG(REG, CLEARMASK, SETMASK)                              \
  STM32_WRITE_REG((REG), (((STM32_READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define STM32_IS_BIT_SET(REG, BIT)  (((REG) & (BIT)) == (BIT))

#define STM32_IS_BIT_CLR(REG, BIT)  (((REG) & (BIT)) == 0U)

/* Use of CMSIS compiler intrinsics for register exclusive access */

/* Atomic 32-bit register access macro to set one or several bits */
#define STM32_ATOMIC_SET_BIT_32(REG, BIT)                                      \
  do {                                                                         \
    uint32_t val;                                                              \
    do {                                                                       \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);                         \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define STM32_ATOMIC_CLEAR_BIT_32(REG, BIT)                                    \
  do {                                                                         \
    uint32_t val;                                                              \
    do {                                                                       \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);                        \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define STM32_ATOMIC_MODIFY_REG_32(REG, CLEARMSK, SETMASK)                     \
  do {                                                                         \
    uint32_t val;                                                              \
    do {                                                                       \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK);     \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define STM32_ATOMIC_SET_BIT_16(REG, BIT)                                      \
  do {                                                                         \
    uint16_t val;                                                              \
    do {                                                                       \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);                         \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 16-bit register access macro to clear one or several bits */
#define STM32_ATOMIC_CLEAR_BIT_16(REG, BIT)                                    \
  do {                                                                         \
    uint16_t val;                                                              \
    do {                                                                       \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);                        \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define STM32_ATOMIC_MODIFY_REG_16(REG, CLEARMSK, SETMASK)                     \
  do {                                                                         \
    uint16_t val;                                                              \
    do {                                                                       \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK);     \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);                   \
  } while(0)

/* Atomic 8-bit register access macro to set one or several bits */
#define STM32_ATOMIC_SET_BIT_8(REG, BIT)                                       \
  do {                                                                         \
    uint8_t val;                                                               \
    do {                                                                       \
      val = __LDREXB((__IO uint8_t *)&(REG)) | (BIT);                          \
    } while ((__STREXB(val,(__IO uint8_t *)&(REG))) != 0U);                    \
  } while(0)

/* Atomic 8-bit register access macro to clear one or several bits */
#define STM32_ATOMIC_CLEAR_BIT_8(REG, BIT)                                     \
  do {                                                                         \
    uint8_t val;                                                               \
    do {                                                                       \
      val = __LDREXB((__IO uint8_t *)&(REG)) & ~(BIT);                         \
    } while ((__STREXB(val,(__IO uint8_t *)&(REG))) != 0U);                    \
  } while(0)

/* Atomic 8-bit register access macro to clear and set one or several bits */
#define STM32_ATOMIC_MODIFY_REG_8(REG, CLEARMSK, SETMASK)                      \
  do {                                                                         \
    uint8_t val;                                                               \
    do {                                                                       \
      val = (__LDREXB((__IO uint8_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK);      \
    } while ((__STREXB(val,(__IO uint8_t *)&(REG))) != 0U);                    \
  } while(0)

/**
  * @}
  */

/** @addtogroup Exported_functions
  * @{
  *    These functions complement the CMSIS-Core API for Cortex System Control Block (SCB)
  *    and System Tick timer (SysTick) features.
  */

/**
  * @brief  Set the vector table offset register (VTOR).
  * @param  Address Address of the vector table
  * @retval None
  */
__STATIC_INLINE void SCB_SetVTOR(uint32_t Address)
{
  STM32_WRITE_REG(SCB->VTOR, Address);
}

/**
  * @brief  Get CPUID information.
  * @retval 32-bit value containing Revision [3:0], PartNo [15:4], Architecture[19:16], Variant [23:20]
  *         and Implementer [31-24] as defined in the product ARM core Architecture Reference Manual.
  */
__STATIC_INLINE uint32_t SCB_GetCPUID(void)
{
  return STM32_READ_REG(SCB->CPUID);
}

/**
  * @brief  Processor uses deep sleep as its low power mode.
  * @retval None
  */
__STATIC_INLINE void SCB_EnableDeepSleep(void)
{
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  STM32_SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
}

/**
  * @brief  Processor uses sleep as its low power mode.
  * @retval None
  */
__STATIC_INLINE void SCB_DisableDeepSleep(void)
{
  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  STM32_CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
}

/**
  * @brief  This function checks if the deep sleep low power mode is active or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SCB_IsEnabledDeepSleep(void)
{
  return ((STM32_READ_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk) == (SCB_SCR_SLEEPDEEP_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Configure sleep-on-exit when returning from Handler mode to Thread mode.
  * @note   Setting this bit to 1 enables an interrupt-driven application to avoid returning to an
  *         empty main application.
  * @retval None
  */
__STATIC_INLINE void SCB_EnableSleepOnExit(void)
{
  /* Set SLEEPONEXIT bit of Cortex System Control Register */
  STM32_SET_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
}

/**
  * @brief  Do not sleep when returning to Thread mode.
  * @retval None
  */
__STATIC_INLINE void SCB_DisableSleepOnExit(void)
{
  /* Clear SLEEPONEXIT bit of Cortex System Control Register */
  STM32_CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
}

/**
  * @brief  This function checks if the sleep-on-exit feature is active or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SCB_IsEnabledSleepOnExit(void)
{
  return ((STM32_READ_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk) == (SCB_SCR_SLEEPONEXIT_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Enabled events and all interrupts, including disabled interrupts, can wakeup the
  *         processor.
  * @retval None
  */
__STATIC_INLINE void SCB_EnableEventOnPend(void)
{
  /* Set SEVEONPEND bit of Cortex System Control Register */
  STM32_SET_BIT(SCB->SCR, SCB_SCR_SEVONPEND_Msk);
}

/**
  * @brief  Only enabled interrupts or events can wakeup the processor, disabled interrupts are
  *         excluded.
  * @retval None
  */
__STATIC_INLINE void SCB_DisableEventOnPend(void)
{
  /* Clear SEVEONPEND bit of Cortex System Control Register */
  STM32_CLEAR_BIT(SCB->SCR, SCB_SCR_SEVONPEND_Msk);
}

/**
  * @brief  This function checks if enabled events and all interrupts, including disabled interrupts
  *         can wakeup the processor or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SCB_IsEnabledEventOnPend(void)
{
  return ((STM32_READ_BIT(SCB->SCR, SCB_SCR_SEVONPEND_Msk) == (SCB_SCR_SEVONPEND_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Enable a fault in System handler control register (SHCSR).
  * @param  Fault This parameter can be a combination of the following values:
  *         @arg @ref SCB_SHCSR_MEMFAULTENA_Msk
  *         @arg @ref SCB_SHCSR_BUSFAULTENA_Msk
  *         @arg @ref SCB_SHCSR_USGFAULTENA_Msk
  * @retval None
  */
__STATIC_INLINE void SCB_EnableFault(uint32_t Fault)
{
  /* Enable the system handler fault (ie. escalation to HardFault disabled) */
  STM32_SET_BIT(SCB->SHCSR, Fault);
}

/**
  * @brief  Disable a fault in System handler control register (SHCSR).
  * @param  Fault This parameter can be a combination of the following values:
  *         @arg @ref SCB_SHCSR_MEMFAULTENA_Msk
  *         @arg @ref SCB_SHCSR_BUSFAULTENA_Msk
  *         @arg @ref SCB_SHCSR_USGFAULTENA_Msk
  * @retval None
  */
__STATIC_INLINE void SCB_DisableFault(uint32_t Fault)
{
  /* Disable the system handler fault (ie. escalation to HardFault enabled) */
  STM32_CLEAR_BIT(SCB->SHCSR, Fault);
}

/**
  * @brief  Enable the Floating-Point Unit (FPU).
  * @retval None
  */
__STATIC_INLINE void SCB_EnableFPU(void)
{
  /* Set CP10 and CP11 in Coprocessor Access Control Register */
  STM32_SET_BIT(SCB->CPACR, ((3UL << 20U) | (3UL << 22U)));
}

/**
  * @brief  Disable the Floating-Point Unit (FPU).
  * @retval None
  */
__STATIC_INLINE void SCB_DisableFPU(void)
{
  /* Clear CP10 and CP11 in Coprocessor Access Control Register */
  STM32_CLEAR_BIT(SCB->CPACR, ((3UL << 20U) | (3UL << 22U)));
}

/**
  * @brief  Configure the SysTick clock source.
  * @param  Source This parameter can be one of the following values:
  *                0 (external clock source) or SysTick_CTRL_CLKSOURCE_Msk (internal clock source)
  * @retval None
  */
__STATIC_INLINE void SysTick_SetClkSource(uint32_t Source)
{
  if (Source == SysTick_CTRL_CLKSOURCE_Msk)
  {
    STM32_SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);
  }
  else
  {
    STM32_CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);
  }
}

/**
  * @brief  Get the SysTick clock source.
  * @retval Returned value can be one of the following values:
  *                0 (external clock source) or SysTick_CTRL_CLKSOURCE_Msk (internal clock source)
  */
__STATIC_INLINE uint32_t SysTick_GetClkSource(void)
{
  return STM32_READ_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);
}

/**
  * @brief  This function checks if the Systick counter flag is active or not.
  * @note   It can be used in timeout function on application side.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SysTick_IsActiveCounterFlag(void)
{
  return ((STM32_READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SysTick exception request.
  * @retval None
  */
__STATIC_INLINE void SysTick_EnableIT(void)
{
  STM32_SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Disable SysTick exception request.
  * @retval None
  */
__STATIC_INLINE void SysTick_DisableIT(void)
{
  STM32_CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Check if the SysTick interrupt is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SysTick_IsEnabledIT(void)
{
  return ((STM32_READ_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk) == (SysTick_CTRL_TICKINT_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SysTick.
  * @retval None
  */
__STATIC_INLINE void SysTick_Enable(void)
{
  STM32_SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

/**
  * @brief  Disable SysTick.
  * @retval None
  */
__STATIC_INLINE void SysTick_Disable(void)
{
  STM32_CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

/**
  * @brief  Check if the SysTick is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t SysTick_IsEnabled(void)
{
  return ((STM32_READ_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk) == (SysTick_CTRL_ENABLE_Msk)) ? 1UL : 0UL);
}

/**
  * @brief  Set SysTick reload value.
  * @param  Ticks Number of ticks between two interrupts.
  *               This parameter can be one value between 0x1 and 0xFFFFFF.
  * @retval 0 if success, 1 if error
  */
__STATIC_INLINE uint32_t SysTick_SetReload(uint32_t Ticks)
{
  if ((Ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  /* Set reload register */
  STM32_WRITE_REG(SysTick->LOAD, (uint32_t)(Ticks - 1UL));
  return (0UL);                                                     /* Function successful */
}

/**
  * @brief  Set the SysTick counter value.
  * @param  Value Any value clears the SysTick counter.
  * @retval None
  */
__STATIC_INLINE void SysTick_SetCounter(uint32_t Value)
{
  STM32_WRITE_REG(SysTick->VAL, Value);
}

/**
  * @brief  Get the SysTick counter value.
  * @retval Counter value
  */
__STATIC_INLINE uint32_t SysTick_GetCounter(void)
{
  return STM32_READ_REG(SysTick->VAL);
}

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_H */
/**
  * @}
  */

/**
  * @}
  */
