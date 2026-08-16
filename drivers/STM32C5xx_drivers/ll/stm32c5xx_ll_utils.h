/**
  ******************************************************************************
  * @file    stm32c5xx_ll_utils.h
  * @brief   Header file of LL utils module.
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
#ifndef STM32C5XX_LL_UTILS_H
#define STM32C5XX_LL_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

/** @defgroup LL_Utils LL utilities services
  * @{
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup LL_System_Private_Constants LL system private constants
  * @{
  */
#define LL_MAX_DELAY          0xFFFFFFFFU /* Max delay can be used in LL_mDelay. */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LL_Utils_Exported_Functions LL UTILS Functions
  * @{
  */

/** @defgroup LL_Utils_Delay Delay management
  * @{
  */

/**
  * @brief   Configure the Cortex-M SysTick source of the time base.
  * @rmtoll
  *  SysTick_LOAD     RELOAD            LL_InitTick \n
  *  SysTick_VAL      CURRENT           LL_InitTick \n
  *  SysTick_CTRL     CLKSOURCE         LL_InitTick \n
  *  SysTick_CTRL     ENABLE            LL_InitTick
  * @param   cpuclk_frequency CPU clock frequency in Hz
  * @param   ticks ticks frequency (unit: Hz), must be > 0
  * @warning When a RTOS is used, it is recommended to avoid changing the SysTick
  *          configuration by calling this function, for a delay use rather osDelay RTOS service.
  */
__STATIC_INLINE void LL_InitTick(uint32_t cpuclk_frequency, uint32_t ticks)
{
  /* Configure the SysTick to have an interrupt in a 1 ms time base. */
  SysTick->LOAD  = (uint32_t)((cpuclk_frequency / ticks) - 1U); /* Set reload register. */
  SysTick->VAL   = 0U;                                        /* Load the SysTick counter value. */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable the SysTick timer. */
}

/**
  * @brief  Configure the Cortex-M SysTick source to have a 1 ms time base.
  * @rmtoll
  *  SysTick_LOAD     RELOAD            LL_Init1msTick \n
  *  SysTick_VAL      CURRENT           LL_Init1msTick \n
  *  SysTick_CTRL     CLKSOURCE         LL_Init1msTick \n
  *  SysTick_CTRL     ENABLE            LL_Init1msTick
  * @param  cpuclk_frequency CPU clock frequency (unit: Hz)
  * @note   When a RTOS is used, it is recommended to avoid changing the Systick
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  */
__STATIC_INLINE void LL_Init1msTick(uint32_t cpuclk_frequency)
{
  /* Use the frequency provided in argument. */
  LL_InitTick(cpuclk_frequency, 1000U);
}

/**
  * @brief   Provide a delay (in milliseconds) based on the SysTick counter flag.
  * @rmtoll
  *  SysTick_CTRL     COUNTFLAG         LL_Delay_NoISR
  * @param   delay_ms Delay duration (unit: milliseconds)
  * @note    Delay accuracy on the requested value is [0; +1 ms] due to uncertainty of the initial SysTick counter value
  *          compared to the reload value.
  * @note    To respect a 1 ms time base, call the @ref LL_Init1msTick function which
  *          applies the appropriate SysTick configuration.
  * @warning When using an RTOS, it is recommended to avoid changing the Systick configuration by calling this function;
  *          rather, use the osDelay RTOS service for delays.
  */
__STATIC_INLINE void LL_Delay_NoISR(uint32_t delay_ms)
{
  volatile uint32_t  tmp = SysTick->CTRL;  /* Clear the SysTick counter reload flag. */
  uint32_t tmp_delay = delay_ms;

  /* Add this code to indicate that the local variable is not used. */
  ((void)tmp);

  /* Add a period to guarantee a minimum wait (uncertainty of the initial SysTick counter value). */
  if (tmp_delay < LL_MAX_DELAY)
  {
    tmp_delay++;
  }

  while (tmp_delay != 0U)
  {
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      tmp_delay--;
    }
  }
}

/**
  * @}
  */

/** @defgroup LL_Utils_SystemClock Clock management
  * @{
  */

/**
  * @brief  This function sets directly SystemCoreClock CMSIS variable.
  * @param  cpuclk_frequency CPU clock frequency (unit: Hz)
  * @note   The variable can also be calculated through the "SystemCoreClockUpdate()" function.
  */
__STATIC_INLINE void LL_SetSystemCoreClock(uint32_t cpuclk_frequency)
{
  /* CPU clock frequency. */
  SystemCoreClock = cpuclk_frequency;
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_LL_UTILS_H */

