/**
  ******************************************************************************
  * @file    stm32c5xx_ll_icache.h
  * @brief   Header file of ICACHE LL module.
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
#ifndef STM32C5XX_LL_ICACHE_H
#define STM32C5XX_LL_ICACHE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(ICACHE)

/** @defgroup ICACHE_LL ICACHE
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup ICACHE_LL_Private_Constants LL ICACHE Constants
  * @{
  */

/** @defgroup ICACHE_LL_PC_Trafic_Route_Mask Remapped Traffic route mask
  * @{
  */
#if defined(ICACHE_CRRx_MSTSEL)
#define LL_ICACHE_MASTERPORT_MASK         ICACHE_CRRx_MSTSEL
#else
#define LL_ICACHE_MASTERPORT_MASK         0U
#endif /* ICACHE_CRRx_MSTSEL */
/**
  * @}
  */

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup ICACHE_LL_Exported_Constants LL ICACHE Constants
  * @{
  */

/** @defgroup ICACHE_LL_EC_WaysSelection Ways selection
  * @{
  */
#define LL_ICACHE_1WAY        0U                /*!< 1-way cache (direct mapped cache) */
#define LL_ICACHE_2WAYS       ICACHE_CR_WAYSEL  /*!< 2-ways set associative cache (default) */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Monitor_Type Monitor type
  * @{
  */
#define LL_ICACHE_MONITOR_HIT          ICACHE_CR_HITMEN                       /*!< Hit monitor counter */
#define LL_ICACHE_MONITOR_MISS         ICACHE_CR_MISSMEN                      /*!< Miss monitor counter */
#define LL_ICACHE_MONITOR_ALL          (ICACHE_CR_HITMEN | ICACHE_CR_MISSMEN) /*!< All monitors counters */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_ICACHE_READ_REG function
  * @{
  */
#define LL_ICACHE_SR_BUSYF             ICACHE_SR_BUSYF     /*!< Busy flag */
#define LL_ICACHE_SR_BSYENDF           ICACHE_SR_BSYENDF   /*!< Busy end flag */
#define LL_ICACHE_SR_ERRF              ICACHE_SR_ERRF      /*!< Cache error flag */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_ICACHE_WRITE_REG function
  * @{
  */
#define LL_ICACHE_FCR_CBSYENDF         ICACHE_FCR_CBSYENDF /*!< Busy end clear flag */
#define LL_ICACHE_FCR_CERRF            ICACHE_FCR_CERRF    /*!< Cache error clear flag */
#define LL_ICACHE_FCR_ALL              (ICACHE_FCR_CBSYENDF | ICACHE_FCR_CERRF)
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_ICACHE_READ_REG and  LL_ICACHE_WRITE_REG functions
  * @{
  */
#define LL_ICACHE_IER_BSYENDIE         ICACHE_IER_BSYENDIE /*!< Busy end interrupt */
#define LL_ICACHE_IER_ERRIE            ICACHE_IER_ERRIE    /*!< Cache error interrupt */
#define LL_ICACHE_IER_ALL              (ICACHE_IER_BSYENDIE | ICACHE_IER_ERRIE)
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Region Remapped region number
  * @{
  */
#define LL_ICACHE_REGION_0             0U  /*!< Region 0 */
#define LL_ICACHE_REGION_1             1U  /*!< Region 1 */
#define LL_ICACHE_REGION_2             2U  /*!< Region 2 */
#define LL_ICACHE_REGION_3             3U  /*!< Region 3 */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Region_Size Remapped Region size
  * @{
  */
#define LL_ICACHE_REGIONSIZE_2MB       1U  /*!< Region size 2MB */
#define LL_ICACHE_REGIONSIZE_4MB       2U  /*!< Region size 4MB */
#define LL_ICACHE_REGIONSIZE_8MB       3U  /*!< Region size 8MB */
#define LL_ICACHE_REGIONSIZE_16MB      4U  /*!< Region size 16MB */
#define LL_ICACHE_REGIONSIZE_32MB      5U  /*!< Region size 32MB */
#define LL_ICACHE_REGIONSIZE_64MB      6U  /*!< Region size 64MB */
#define LL_ICACHE_REGIONSIZE_128MB     7U  /*!< Region size 128MB */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Traffic_Route Remapped Traffic route
  * @{
  */
#define LL_ICACHE_MASTER1_PORT         0U                  /*!< Master1 port */
#if defined(ICACHE_CRRx_MSTSEL)
#define LL_ICACHE_MASTER2_PORT         ICACHE_CRRx_MSTSEL  /*!< Master2 port */
#endif /* ICACHE_CRRx_MSTSEL */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Output_Burst_Type Remapped Output burst type
  * @{
  */
#define LL_ICACHE_OUTPUT_BURST_WRAP    0U                  /*!< WRAP */
#define LL_ICACHE_OUTPUT_BURST_INCR    ICACHE_CRRx_HBURST  /*!< INCR */
/**
  * @}
  */

/** @defgroup ICACHE_LL_EC_Address_Shift Address shift
  * @{
  */
#define LL_ICACHE_ADDRESS_SHIFT        21U                /*!< Address shift */
/**
  * @}
  */

/**
  * @}
  */


/* Exported macros -----------------------------------------------------------*/
/** @defgroup ICACHE_LL_Exported_Macros LL ICACHE Macros
  * @{
  */

/** @defgroup ICACHE_LL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in the ICACHE register.
  * @param  instance ICACHE instance.
  * @param  reg Register to be written.
  * @param  value Value to be written in the register.
  */
#define LL_ICACHE_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in the ICACHE register.
  * @param  instance ICACHE instance.
  * @param  reg Register to be read.
  * @retval Register value
  */
#define LL_ICACHE_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup ICACHE_LL_Exported_Functions LL ICACHE Functions
  * @{
  */

/** @defgroup ICACHE_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable the ICACHE.
  * @rmtoll
  *  CR           EN            LL_ICACHE_Enable
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_Enable(ICACHE_TypeDef *icachex)
{
  STM32_SET_BIT(icachex->CR, ICACHE_CR_EN);
}

/**
  * @brief  Disable the ICACHE.
  * @rmtoll
  *  CR           EN            LL_ICACHE_Disable
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_Disable(ICACHE_TypeDef *icachex)
{
  STM32_CLEAR_BIT(icachex->CR, ICACHE_CR_EN);
}

/**
  * @brief  Return whether ICACHE is enabled.
  * @rmtoll
  *  CR           EN            LL_ICACHE_IsEnabled
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabled(const ICACHE_TypeDef *icachex)
{
  return (STM32_READ_BIT(icachex->CR, ICACHE_CR_EN));
}

/**
  * @brief  Select the ICACHE operating mode.
  * @rmtoll
  *  CR           WAYSEL        LL_ICACHE_SetMode
  * @param  icachex ICACHE instance.
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_1WAY
  *         @arg @ref LL_ICACHE_2WAYS
  */
__STATIC_INLINE void LL_ICACHE_SetMode(ICACHE_TypeDef *icachex, uint32_t mode)
{
  STM32_MODIFY_REG(icachex->CR, ICACHE_CR_WAYSEL, mode);
}

/**
  * @brief  Get the selected ICACHE operating mode.
  * @rmtoll
  *  CR           WAYSEL        LL_ICACHE_GetMode
  * @param  icachex ICACHE instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ICACHE_1WAY
  *         @arg @ref LL_ICACHE_2WAYS
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetMode(const ICACHE_TypeDef *icachex)
{
  return (STM32_READ_BIT(icachex->CR, ICACHE_CR_WAYSEL));
}

/**
  * @brief  Invalidate the ICACHE.
  * @rmtoll
  *  CR           CACHEINV      LL_ICACHE_Invalidate
  * @param  icachex ICACHE instance.
  * @note   Until the BSYEND flag is set, the cache is bypassed.
  */
__STATIC_INLINE void LL_ICACHE_Invalidate(ICACHE_TypeDef *icachex)
{
  STM32_SET_BIT(icachex->CR, ICACHE_CR_CACHEINV);
}

/**
  * @}
  */

/** @defgroup ICACHE_LL_EF_Monitors Monitors
  * @{
  */

/**
  * @brief  Enable the hit/miss monitor(s).
  * @rmtoll
  *  CR           HITMEN        LL_ICACHE_EnableMonitors \n
  *  CR           MISSMEN       LL_ICACHE_EnableMonitors
  * @param  icachex ICACHE instance.
  * @param  monitors This parameter can be one or a combination of the following values:
  *         @arg @ref LL_ICACHE_MONITOR_HIT
  *         @arg @ref LL_ICACHE_MONITOR_MISS
  *         @arg @ref LL_ICACHE_MONITOR_ALL
  */
__STATIC_INLINE void LL_ICACHE_EnableMonitors(ICACHE_TypeDef *icachex, uint32_t monitors)
{
  STM32_SET_BIT(icachex->CR, monitors);
}

/**
  * @brief  Disable the hit/miss monitor(s).
  * @rmtoll
  *  CR           HITMEN        LL_ICACHE_DisableMonitors \n
  *  CR           MISSMEN       LL_ICACHE_DisableMonitors
  * @param  icachex ICACHE instance.
  * @param  monitors This parameter can be one or a combination of the following values:
  *         @arg @ref LL_ICACHE_MONITOR_HIT
  *         @arg @ref LL_ICACHE_MONITOR_MISS
  *         @arg @ref LL_ICACHE_MONITOR_ALL
  */
__STATIC_INLINE void LL_ICACHE_DisableMonitors(ICACHE_TypeDef *icachex, uint32_t monitors)
{
  STM32_CLEAR_BIT(icachex->CR, monitors);
}

/**
  * @brief  Check whether the monitor(s) are enabled or disabled.
  * @rmtoll
  *  CR           HITMEN        LL_ICACHE_IsEnabledMonitors \n
  *  CR           MISSMEN       LL_ICACHE_IsEnabledMonitors
  * @param  icachex ICACHE instance.
  * @param  monitors This parameter can be one or a combination of the following values:
  *         @arg @ref LL_ICACHE_MONITOR_HIT
  *         @arg @ref LL_ICACHE_MONITOR_MISS
  *         @arg @ref LL_ICACHE_MONITOR_ALL
  * @retval State of parameter value (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabledMonitors(const ICACHE_TypeDef *icachex, uint32_t monitors)
{
  return ((STM32_READ_BIT(icachex->CR, monitors) == (monitors)) ? 1UL : 0UL);
}

/**
  * @brief  Reset the performance monitoring.
  * @rmtoll
  *  CR           HITMRST       LL_ICACHE_ResetMonitors \n
  *  CR           MISSMRST      LL_ICACHE_ResetMonitors
  * @param  icachex ICACHE instance.
  * @param  monitors This parameter can be one or a combination of the following values:
  *         @arg @ref LL_ICACHE_MONITOR_HIT
  *         @arg @ref LL_ICACHE_MONITOR_MISS
  *         @arg @ref LL_ICACHE_MONITOR_ALL
  */
__STATIC_INLINE void LL_ICACHE_ResetMonitors(ICACHE_TypeDef *icachex, uint32_t monitors)
{
  /* Reset */
  STM32_SET_BIT(icachex->CR, (monitors << 2U));
  /* Release reset */
  STM32_CLEAR_BIT(icachex->CR, (monitors << 2U));
}

/**
  * @brief  Get the Hit monitor.
  * @rmtoll
  *  HMONR        HITMON        LL_ICACHE_GetHitMonitor
  * @param  icachex ICACHE instance.
  * @note   Upon reaching the 32-bit maximum value, hit monitor does not wrap.
  * @retval Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetHitMonitor(const ICACHE_TypeDef *icachex)
{
  return (icachex->HMONR);
}

/**
  * @brief  Get the Miss monitor.
  * @rmtoll
  *  MMONR        MISSMON       LL_ICACHE_GetMissMonitor
  * @param  icachex ICACHE instance.
  * @note   Upon reaching the 16-bit maximum value, miss monitor does not wrap.
  * @retval Value between Min_Data=0 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetMissMonitor(const ICACHE_TypeDef *icachex)
{
  return (icachex->MMONR);
}

/**
  * @}
  */

/** @defgroup ICACHE_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable BSYEND interrupt.
  * @rmtoll
  *  IER          BSYENDIE      LL_ICACHE_EnableIT_BSYEND
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_EnableIT_BSYEND(ICACHE_TypeDef *icachex)
{
  STM32_SET_BIT(icachex->IER, ICACHE_IER_BSYENDIE);
}

/**
  * @brief  Disable BSYEND interrupt.
  * @rmtoll
  *  IER          BSYENDIE      LL_ICACHE_DisableIT_BSYEND
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_DisableIT_BSYEND(ICACHE_TypeDef *icachex)
{
  STM32_CLEAR_BIT(icachex->IER, ICACHE_IER_BSYENDIE);
}

/**
  * @brief  Check whether the BSYEND interrupt is enabled or disabled.
  * @rmtoll
  *  IER          BSYENDIE      LL_ICACHE_IsEnabledIT_BSYEND
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabledIT_BSYEND(const ICACHE_TypeDef *icachex)
{
  return ((STM32_READ_BIT(icachex->IER, ICACHE_IER_BSYENDIE) == (ICACHE_IER_BSYENDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable ERR interrupt.
  * @rmtoll
  *  IER          ERRIE         LL_ICACHE_EnableIT_ERR
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_EnableIT_ERR(ICACHE_TypeDef *icachex)
{
  STM32_SET_BIT(icachex->IER, ICACHE_IER_ERRIE);
}

/**
  * @brief  Disable ERR interrupt.
  * @rmtoll
  *  IER          ERRIE        LL_ICACHE_DisableIT_ERR
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_DisableIT_ERR(ICACHE_TypeDef *icachex)
{
  STM32_CLEAR_BIT(icachex->IER, ICACHE_IER_ERRIE);
}

/**
  * @brief  Check whether the ERR interrupt is enabled or disabled.
  * @rmtoll
  *  IER          ERRIE         LL_ICACHE_IsEnabledIT_ERR
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabledIT_ERR(const ICACHE_TypeDef *icachex)
{
  return ((STM32_READ_BIT(icachex->IER, ICACHE_IER_ERRIE) == (ICACHE_IER_ERRIE)) ? 1UL : 0UL);
}

/** @brief  Enable ICACHE interrupt(s).
  * @rmtoll
  *  IER         BSYENDIE         LL_ICACHE_EnableIT \n
  *  IER         ERRIE            LL_ICACHE_EnableIT
  * @param  icachex ICACHE instance.
  * @param  interrupts This parameter can be any combination of the following values:
  *            @arg @ref LL_ICACHE_IER_BSYENDIE  Busy end interrupt
  *            @arg @ref LL_ICACHE_IER_ERRIE     Cache error interrupt
  *            @arg @ref LL_ICACHE_IER_ALL       All interrupts
  */
__STATIC_INLINE void LL_ICACHE_EnableIT(ICACHE_TypeDef *icachex, uint32_t interrupts)
{
  STM32_SET_BIT(icachex->IER, interrupts);
}

/**
  * @brief  Disable ICACHE interrupt(s).
  * @rmtoll
  *  IER          BSYENDIE         LL_ICACHE_DisableIT \n
  *  IER          ERRIE            LL_ICACHE_DisableIT
  * @param  icachex ICACHE instance.
  * @param  interrupt This parameter can be any combination of the following values:
  *            @arg @ref LL_ICACHE_IER_BSYENDIE  Busy end interrupt
  *            @arg @ref LL_ICACHE_IER_ERRIE     Cache error interrupt
  */
__STATIC_INLINE void LL_ICACHE_DisableIT(ICACHE_TypeDef *icachex, uint32_t interrupt)
{
  STM32_CLEAR_BIT(icachex->IER, interrupt);
}

/** @brief  Indicate whether the interrupt(s) is(are) enabled.
  * @rmtoll
  *  IER          BSYENDIE         LL_ICACHE_IsEnabledIT \n
  *  IER          ERRIE            LL_ICACHE_IsEnabledIT
  * @param  icachex ICACHE instance.
  * @param  interrupts Specifies the ICACHE interrupt source to check.
  *         This parameter can be any combination of the following values:
  *            @arg @ref LL_ICACHE_IER_BSYENDIE  Busy end interrupt
  *            @arg @ref LL_ICACHE_IER_ERRIE  Cache error interrupt
  * @retval uint32_t The state of the bit(s) checked :
  *            @arg 0UL if the corresponding test is failed.
  *            @arg 1UL if the corresponding test is succeeded.
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabledIT(const ICACHE_TypeDef *icachex, uint32_t interrupts)
{
  return ((STM32_READ_BIT(icachex->IER, interrupts) == (interrupts)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup ICACHE_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate the status of an ongoing operation flag.
  * @rmtoll
  *  SR           BUSYF         LL_ICACHE_IsActiveFlag_BUSY
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsActiveFlag_BUSY(const ICACHE_TypeDef *icachex)
{
  return ((STM32_READ_BIT(icachex->SR, ICACHE_SR_BUSYF) == (ICACHE_SR_BUSYF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of an operation end flag.
  * @rmtoll
  *  SR           BSYENDF      LL_ICACHE_IsActiveFlag_BSYEND
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsActiveFlag_BSYEND(const ICACHE_TypeDef *icachex)
{
  return ((STM32_READ_BIT(icachex->SR, ICACHE_SR_BSYENDF) == (ICACHE_SR_BSYENDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of an error flag.
  * @rmtoll
  *  SR           ERRF          LL_ICACHE_IsActiveFlag_ERR
  * @param  icachex ICACHE instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsActiveFlag_ERR(const ICACHE_TypeDef *icachex)
{
  return ((STM32_READ_BIT(icachex->SR, ICACHE_SR_ERRF) == (ICACHE_SR_ERRF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear busy end of operation flag.
  * @rmtoll
  *  FCR          CBSYENDF      LL_ICACHE_ClearFlag_BSYEND
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_ClearFlag_BSYEND(ICACHE_TypeDef *icachex)
{
  STM32_WRITE_REG(icachex->FCR, ICACHE_FCR_CBSYENDF);
}

/**
  * @brief  Clear error flag.
  * @rmtoll
  *  FCR          ERRF          LL_ICACHE_ClearFlag_ERR
  * @param  icachex ICACHE instance.
  */
__STATIC_INLINE void LL_ICACHE_ClearFlag_ERR(ICACHE_TypeDef *icachex)
{
  STM32_WRITE_REG(icachex->FCR, ICACHE_FCR_CERRF);
}

/** @brief  Clear the ICACHE flag(s).
  * @rmtoll
  *  FCR          CBSYENDF      LL_ICACHE_ClearFlag \n
  *  FCR          ERRF          LL_ICACHE_ClearFlag
  * @param  icachex ICACHE instance.
  * @param  mask This parameter is a combination of the following values:
  *            @arg @ref LL_ICACHE_FCR_CBSYENDF  Busy end clear flag
  *            @arg @ref LL_ICACHE_FCR_CERRF     Cache error clear flag
  *            @arg @ref LL_ICACHE_FCR_ALL       Clear all flags
  */
__STATIC_INLINE void LL_ICACHE_ClearFlag(ICACHE_TypeDef *icachex, uint32_t mask)
{
  STM32_WRITE_REG(icachex->FCR, mask);
}

/** @brief  Check whether the selected ICACHE flag(s) is(are) set or not.
  * @rmtoll
  *  SR           BUSYF         LL_ICACHE_IsActiveFlag \n
  *  SR           BSYENDF       LL_ICACHE_IsActiveFlag \n
  *  SR           ERRF          LL_ICACHE_IsActiveFlag
  * @param  icachex ICACHE instance.
  * @param  interrupt Specifies the flag to check.
  *         This parameter can be any combination of the following values:
  *            @arg @ref LL_ICACHE_SR_BUSYF    Busy flag
  *            @arg @ref LL_ICACHE_SR_BSYENDF  Busy end flag
  *            @arg @ref LL_ICACHE_SR_ERRF     Cache error flag
  * @retval Return 1 if at least one flag is set; otherwise 0.
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsActiveFlag(const ICACHE_TypeDef *icachex, uint32_t interrupt)
{
  return ((STM32_READ_BIT(icachex->SR, interrupt) == (interrupt)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup ICACHE_LL_EF_REGION_Management REGION_Management
  * @{
  */

/**
  * @brief  Enable the remapped memory region.
  * @rmtoll
  *  CRRx         REN           LL_ICACHE_EnableRegion
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @note   The region must have been already configured.
  */
__STATIC_INLINE void LL_ICACHE_EnableRegion(ICACHE_TypeDef *icachex, uint32_t region)
{
  STM32_SET_BIT(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), ICACHE_CRRx_REN);
}

/**
  * @brief  Disable the remapped memory region.
  * @rmtoll
  *  CRRx         REN           LL_ICACHE_DisableRegion
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  */
__STATIC_INLINE void LL_ICACHE_DisableRegion(ICACHE_TypeDef *icachex, uint32_t region)
{
  STM32_CLEAR_BIT(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), (ICACHE_CRRx_REN));
}

/**
  * @brief  Return whether the remapped memory region is enabled.
  * @rmtoll
  *  CRRx         REN           LL_ICACHE_IsEnabledRegion
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ICACHE_IsEnabledRegion(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return ((STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                          (ICACHE_CRRx_REN)) != 0U) ? 1UL : 0UL);
}

/**
  * @brief  Select the memory remapped region base address.
  * @rmtoll
  *  CRRx         BASEADDR      LL_ICACHE_SetRegionBaseAddress
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  base_address Alias address in the Code region
  * @note   The useful bits depend on RSIZE as described in the Reference Manual.
  */
__STATIC_INLINE void LL_ICACHE_SetRegionBaseAddress(ICACHE_TypeDef *icachex, uint32_t region, uint32_t base_address)
{
  STM32_MODIFY_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                   ICACHE_CRRx_BASEADDR, ((base_address & 0x1FFFFFFFU) >> LL_ICACHE_ADDRESS_SHIFT));
}

/**
  * @brief  Get the memory remapped region base address.
  * @rmtoll
  *  CRRx         BASEADDR      LL_ICACHE_GetRegionBaseAddress
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @note   The useful bits depend on RSIZE as described in the Reference Manual.
  * @retval base_address Alias address in the Code region
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetRegionBaseAddress(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return (STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                         ICACHE_CRRx_BASEADDR) << LL_ICACHE_ADDRESS_SHIFT);
}

/**
  * @brief  Select the memory remapped region remap address.
  * @rmtoll
  *  CRRx         REMAPADDR      LL_ICACHE_SetRegionRemapAddress
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  remap_address Memory address to remap
  * @note   The useful bits depend on RSIZE as described in the Reference Manual.
  */
__STATIC_INLINE void LL_ICACHE_SetRegionRemapAddress(ICACHE_TypeDef *icachex, uint32_t region, uint32_t remap_address)
{
  STM32_MODIFY_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), ICACHE_CRRx_REMAPADDR, \
                   ((remap_address >> LL_ICACHE_ADDRESS_SHIFT) << ICACHE_CRRx_REMAPADDR_Pos));
}

/**
  * @brief  Get the memory remapped region base address.
  * @rmtoll
  *  CRRx         REMAPADDR      LL_ICACHE_GetRegionRemapAddress
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @note   The useful bits depend on RSIZE as described in the Reference Manual.
  * @retval remap_address Remapped memory address
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetRegionRemapAddress(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return ((STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                          ICACHE_CRRx_REMAPADDR) >> ICACHE_CRRx_REMAPADDR_Pos) << LL_ICACHE_ADDRESS_SHIFT);
}

/**
  * @brief  Select the memory remapped region size.
  * @rmtoll
  *  CRRx         RSIZE      LL_ICACHE_SetRegionSize
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGIONSIZE_2MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_4MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_8MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_16MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_32MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_64MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_128MB
  */
__STATIC_INLINE void LL_ICACHE_SetRegionSize(ICACHE_TypeDef *icachex, uint32_t region, uint32_t size)
{
  STM32_MODIFY_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                   ICACHE_CRRx_RSIZE, (size << ICACHE_CRRx_RSIZE_Pos));
}

/**
  * @brief  Get the selected memory remapped region size.
  * @rmtoll
  *  CRRx         RSIZE      LL_ICACHE_GetRegionSize
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ICACHE_REGIONSIZE_2MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_4MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_8MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_16MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_32MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_64MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_128MB
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetRegionSize(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return (STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                         ICACHE_CRRx_RSIZE) >> ICACHE_CRRx_RSIZE_Pos);
}

/**
  * @brief  Select the memory remapped region output burst type.
  * @rmtoll
  *  CRRx         HBURST      LL_ICACHE_SetRegionOutputBurstType
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  burst This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_WRAP
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_INCR
  */
__STATIC_INLINE void LL_ICACHE_SetRegionOutputBurstType(ICACHE_TypeDef *icachex, uint32_t region, uint32_t burst)
{
  STM32_MODIFY_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                   ICACHE_CRRx_HBURST, burst);
}

/**
  * @brief  Get the selected memory remapped region output burst type.
  * @rmtoll
  *  CRRx         HBURST      LL_ICACHE_GetRegionOutputBurstType
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_WRAP
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_INCR
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetRegionOutputBurstType(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return (STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                         ICACHE_CRRx_HBURST));
}

/**
  * @brief  Select the memory remapped region cache master port.
  * @rmtoll
  *  CRRx         MSTSEL      LL_ICACHE_SetRegionMasterPort
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  port Master_port field which can have one of the following values:
  *         @arg @ref LL_ICACHE_MASTER1_PORT
  * @if ICACHE_CRRx_MSTSEL
  *         @arg @ref LL_ICACHE_MASTER2_PORT
  * @endif
  */
__STATIC_INLINE void LL_ICACHE_SetRegionMasterPort(ICACHE_TypeDef *icachex, uint32_t region, uint32_t port)
{
  STM32_MODIFY_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                   LL_ICACHE_MASTERPORT_MASK, port);
}

/**
  * @brief  Get the selected memory remapped region cache master port.
  * @rmtoll
  *  CRRx         MSTSEL      LL_ICACHE_GetRegionMasterPort
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @retval uint32_t Returned value can have one of the following values:
  *         @arg @ref LL_ICACHE_MASTER1_PORT
  * @if ICACHE_CRRx_MSTSEL
  *         @arg @ref LL_ICACHE_MASTER2_PORT
  * @endif
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetRegionMasterPort(const ICACHE_TypeDef *icachex, uint32_t region)
{
#if defined(LL_ICACHE_MASTER2_PORT)
  return (STM32_READ_BIT(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region))), \
                         ICACHE_CRRx_MSTSEL));
#else
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(icachex);
  STM32_UNUSED(region);

  return LL_ICACHE_MASTER1_PORT;
#endif /* LL_ICACHE_MASTER2_PORT */
}

/**
  * @brief  Set the remap region configuration.
  * @rmtoll
  *  CRRx         BASEADDR    LL_ICACHE_SetConfigRemapRegion \n
  *  CRRx         REMAPADDR   LL_ICACHE_SetConfigRemapRegion \n
  *  CRRx         RSIZE       LL_ICACHE_SetConfigRemapRegion \n
  *  CRRx         HBURST      LL_ICACHE_SetConfigRemapRegion \n
  *  CRRx         MSTSEL      LL_ICACHE_SetConfigRemapRegion
  * @param  icachex ICACHE instance.
  * @param  region This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGION_0
  *         @arg @ref LL_ICACHE_REGION_1
  *         @arg @ref LL_ICACHE_REGION_2
  *         @arg @ref LL_ICACHE_REGION_3
  * @param  base_address  Alias address in the Code region.
  * @param  remap_address External memory address.
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_REGIONSIZE_2MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_4MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_8MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_16MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_32MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_64MB
  *         @arg @ref LL_ICACHE_REGIONSIZE_128MB
  * @param  master_port This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_MASTER1_PORT
  * @if ICACHE_CRRx_MSTSEL
  *         @arg @ref LL_ICACHE_MASTER2_PORT
  * @endif
  * @param  output_burst This parameter can be one of the following values:
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_WRAP
  *         @arg @ref LL_ICACHE_OUTPUT_BURST_INCR
  */
__STATIC_INLINE void LL_ICACHE_SetConfigRemapRegion(ICACHE_TypeDef *icachex, uint32_t region, uint32_t base_address,
                                                    uint32_t remap_address, uint32_t size, uint32_t master_port,
                                                    uint32_t output_burst)
{
  STM32_WRITE_REG(*((__IO uint32_t *)(&(icachex->CRR0) + (1U * region))),
                  (((base_address & 0x1FFFFFFFU) >> LL_ICACHE_ADDRESS_SHIFT)) |
                  ((remap_address >> LL_ICACHE_ADDRESS_SHIFT) << ICACHE_CRRx_REMAPADDR_Pos) |
                  (size << ICACHE_CRRx_RSIZE_Pos) | master_port | output_burst);
}

/**
  * @brief  Get the remap region configuration.
  * @rmtoll
  *  CRRx         BASEADDR    LL_ICACHE_GetConfigRemapRegion \n
  *  CRRx         REMAPADDR   LL_ICACHE_GetConfigRemapRegion \n
  *  CRRx         RSIZE       LL_ICACHE_GetConfigRemapRegion \n
  *  CRRx         HBURST      LL_ICACHE_GetConfigRemapRegion \n
  *  CRRx         MSTSEL      LL_ICACHE_GetConfigRemapRegion
  * @param  icachex ICACHE instance.
  * @param  region Region number.
  * @retval uint32_t Return the value of the CRRx register.
  */
__STATIC_INLINE uint32_t LL_ICACHE_GetConfigRemapRegion(const ICACHE_TypeDef *icachex, uint32_t region)
{
  return (STM32_READ_REG(*((__IO const uint32_t *)(&(icachex->CRR0) + (1U * region)))));
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

#endif /* ICACHE */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_ICACHE_H */
