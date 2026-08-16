/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_pwr.h
  * @brief   Header file for PWR LL module.
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in the root directory of this software
  * component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_LL_PWR_H
#define STM32C5XX_LL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (PWR)

/** @defgroup PWR_LL PWR
  * @{
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_LL_Private_Constants PWR Private Constants
  * @{
  */

/** @defgroup PWR_LL_WAKEUP_PIN_OFFSET Wake-Up Pins register offset defines
  * @brief    Flag definitions that can be used with LL_PWR_WriteReg function.
  * @{
  */
/* Wake-Up Pins PWR register offsets */
#define LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET    2UL
#define LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK       0x7FU
#define LL_PWR_WAKEUP_PIN_REF                   PWR_WUCR_WUPP1
#define LL_PWR_WAKEUP_PIN_REF_POS               PWR_WUCR_WUPP1_Pos
#define LL_PWR_WAKEUP_PIN_PULL_REF              PWR_WUCR_WUPPUPD1
#define LL_PWR_WAKEUP_PIN_PULL_REF_POS          PWR_WUCR_WUPPUPD1_Pos
/**
  * @}
  */

/** @defgroup PWR_LL_SRAM2_STOP_RETENTION_POSITION SRAM2 stop retention bit positions
  * @brief    Position definitions that can be used in HAL_PWR_LP_EnableMemoryPageRetention /
  *           HAL_PWR_LP_EnableMemoryPageRetention functions.
  * @{
  */
#if defined(PWR_PMCR_SRAM2_1_SO_Pos)
#define LL_PWR_SRAM2_STOP_RETENTION_POS     PWR_PMCR_SRAM2_1_SO_Pos /*!< SRAM2 Page 1 retention in stop mode position */
#else
#define LL_PWR_SRAM2_STOP_RETENTION_POS     PWR_PMCR_SRAM2SO_Pos    /*!< SRAM2 retention in stop mode position */

#endif /* PWR_PMCR_SRAM2_1_SO_Pos */
/**
  * @}
  */

/**
  * @}
  */
/* Exported constants ------------------------------------------------------------------------------------------------*/

/** @defgroup PWR_LL_Exported_Constants LL PWR Constants
  * @{
  */

/** @defgroup PWR_LL_EC_LOW_POWER_MODE_SELECTION  Low Power Mode Selection
  * @{
  */
#define LL_PWR_STOP0_MODE     0U                  /*!< STOP0 mode    */
#define LL_PWR_STOP1_MODE     PWR_PMCR_LPMS_0     /*!< STOP1 mode    */
#define LL_PWR_STANDBY_MODE   PWR_PMCR_LPMS_1     /*!< STANDBY mode  */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_SRAM1_STOP_CONTENTS_RETENTION PWR SRAM1 Content retention in stop mode
  * @{
  */
#define LL_PWR_SRAM1_STOP_RETENTION    PWR_PMCR_SRAM1SO  /*!< SRAM1 retention in stop mode  */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_SRAM2_STOP_CONTENTS_RETENTION PWR SRAM2 Content retention in stop mode
  * @{
  */
#if defined(PWR_PMCR_SRAM2_1_SO)
#define LL_PWR_SRAM2_PAGE1_STOP_RETENTION   PWR_PMCR_SRAM2_1_SO     /*!< SRAM2 Page 1 retention in stop mode          */
#define LL_PWR_SRAM2_PAGE2_STOP_RETENTION   PWR_PMCR_SRAM2_2_SO     /*!< SRAM2 Page 2 retention in stop mode          */
#if defined(PWR_PMCR_SRAM2_3_SO)
#define LL_PWR_SRAM2_PAGE3_STOP_RETENTION   PWR_PMCR_SRAM2_3_SO     /*!< SRAM2 Page 3 retention in stop mode          */
#define LL_PWR_SRAM2_STOP_RETENTION         (PWR_PMCR_SRAM2_3_SO | PWR_PMCR_SRAM2_2_SO | PWR_PMCR_SRAM2_1_SO)
/*!< SRAM2 retention in stop mode */
#else
#define LL_PWR_SRAM2_STOP_RETENTION         (PWR_PMCR_SRAM2_2_SO | PWR_PMCR_SRAM2_1_SO)  /*!< SRAM2 retention
                                                                                              in stop mode            */
#endif /* PWR_PMCR_SRAM2_3_SO */
#else
#define LL_PWR_SRAM2_STOP_RETENTION         PWR_PMCR_SRAM2SO /*!< SRAM2 retention in stop mode */
#endif /* PWR_PMCR_SRAM2_1_SO */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_PIN PWR Wake Up Pin
  * @{
  */
#define LL_PWR_WAKEUP_PIN_1   PWR_WUCR_WUPEN1  /*!< Wakeup pin 1 enable */
#define LL_PWR_WAKEUP_PIN_2   PWR_WUCR_WUPEN2  /*!< Wakeup pin 2 enable */
#if defined(PWR_WUCR_WUPEN3)
#define LL_PWR_WAKEUP_PIN_3   PWR_WUCR_WUPEN3  /*!< Wakeup pin 3 enable */
#endif /* PWR_WUCR_WUPEN3 */
#define LL_PWR_WAKEUP_PIN_4   PWR_WUCR_WUPEN4  /*!< Wakeup pin 4 enable */
#define LL_PWR_WAKEUP_PIN_5   PWR_WUCR_WUPEN5  /*!< Wakeup pin 5 enable */
#if defined(PWR_WUCR_WUPEN6)
#define LL_PWR_WAKEUP_PIN_6   PWR_WUCR_WUPEN6  /*!< Wakeup pin 6 enable */
#endif /* PWR_WUCR_WUPEN6 */
#if defined(PWR_WUCR_WUPEN7)
#define LL_PWR_WAKEUP_PIN_7   PWR_WUCR_WUPEN7  /*!< Wakeup pin 7 enable */
#endif /* PWR_WUCR_WUPEN7 */
#if defined(PWR_WUCR_WUPEN3) && defined(PWR_WUCR_WUPEN6) && defined(PWR_WUCR_WUPEN7)
#define LL_PWR_WAKEUP_PIN_ALL (0x7FU)          /*!< Wakeup all pin enable */
#else
#define LL_PWR_WAKEUP_PIN_ALL (0x1BU)          /*!< Wakeup all pin enable */
#endif /* PWR_WUCR_WUPEN3 && PWR_WUCR_WUPEN6 && PWR_WUCR_WUPEN7 */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_PIN_PULL  Wakeup Pins pull configuration
  * @{
  */
#define LL_PWR_WAKEUP_PIN_PULL_NO     0x00000000UL   /*!< Configure Wake-Up pin in no pull   */
#define LL_PWR_WAKEUP_PIN_PULL_UP     0x00000001UL   /*!< Configure Wake-Up pin in pull Up   */
#define LL_PWR_WAKEUP_PIN_PULL_DOWN   0x00000002UL   /*!< Configure Wake-Up pin in pull Down */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_PIN_POLARITY  Wakeup Pins polarity
  * @{
  */
#define LL_PWR_WAKEUP_PIN_POLARITY_HIGH     0x00000000UL   /*!< Wakeup pin polarity high  */
#define LL_PWR_WAKEUP_PIN_POLARITY_LOW      0x00000001UL   /*!< Wakeup pin polarity low   */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_IO_RETENTION  Privilege items
  * @{
  */
#define LL_PWR_IO_RETENTION_GPIO    PWR_IORETR_IORETEN      /*!< GPIO retention in Standby Mode   */
#define LL_PWR_IO_RETENTION_JTAGIO  PWR_IORETR_JTAGIORETEN  /*!< JTAGIO retention in Standby Mode */
#define LL_PWR_IO_RETENTION_ALL     (PWR_IORETR_IORETEN | PWR_IORETR_JTAGIORETEN) /*!< All IO retention in Standby
                                                                                       Mode  */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_PRIVILEGE_ITEM  Privilege items
  * @{
  */
#define LL_PWR_PRIV_ITEM_ALL     PWR_PRIVCFGR_PRIV   /*!< All privilege items  */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_PRIVILEGE_ATTRIBUTE  Privilege attribute
  * @{
  */
#define LL_PWR_ATTR_NPRIV     0U   /*!< Non-privileged attribute  */
#define LL_PWR_ATTR_PRIV      1U   /*!< Privileged attribute      */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ----------------------------------------------------------------------------------------------------*/

/** @defgroup PWR_LL_Exported_Macros LL PWR Macros
  * @{
  */

/** @defgroup PWR_LL_EM_WRITE_READ Common Write and Read Registers Macros
  * @{
  */

/**
  * @brief  Write a value in PWR register.
  * @param  reg      Register to be written.
  * @param  value    Value to be written in the register.
  *
  */
#define LL_PWR_WRITE_REG(reg, value) STM32_WRITE_REG(PWR->reg, (value))

/**
  * @brief  Read a value in PWR register.
  * @param  reg      Register to be read.
  * @retval Register value
  */
#define LL_PWR_READ_REG(reg) STM32_READ_REG(PWR->reg)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @defgroup PWR_LL_Exported_Functions LL PWR Functions
  * @{
  */

/** @defgroup PWR_LL_EF_CONFIGURATION PWR Configuration
  * @{
  */

/**
  * @brief  Set system power mode.
  * @rmtoll
  *  PMCR              LPMS          LL_PWR_SetPowerMode
  * @param  mode : This parameter can be one of the following values:
  *         @arg @ref LL_PWR_STOP0_MODE
  *         @arg @ref LL_PWR_STOP1_MODE
  *         @arg @ref LL_PWR_STANDBY_MODE
  * @note   When Stop mode is selected, WFI or WFE instruction is required to enter Stop mode.
  * @note   When Standby mode is selected, only WFI instruction is required to enter Standby mode.
  * @note   Call DSB instruction to ensure there are no outstanding memory transactions prior to executing WFI or WFE.
  */
__STATIC_INLINE void LL_PWR_SetPowerMode(uint32_t mode)
{
  STM32_MODIFY_REG(PWR->PMCR, PWR_PMCR_LPMS, mode);
}

/**
  * @brief  Get system power mode.
  * @rmtoll
  *  PMCR             LPMS          LL_PWR_GetPowerMode
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_STOP0_MODE
  *         @arg @ref LL_PWR_STOP1_MODE
  *         @arg @ref LL_PWR_STANDBY_MODE
  */
__STATIC_INLINE uint32_t LL_PWR_GetPowerMode(void)
{
  return (STM32_READ_BIT(PWR->PMCR, PWR_PMCR_LPMS));
}

/**
  * @brief  Enable the flash power down in stop mode.
  * @rmtoll
  *  PMCR    FLPS       LL_PWR_EnableFlashLowPWRMode
  */
__STATIC_INLINE void LL_PWR_EnableFlashLowPWRMode(void)
{
  STM32_SET_BIT(PWR->PMCR, PWR_PMCR_FLPS);
}

/**
  * @brief  Disable the flash power down in stop mode.
  * @rmtoll
  *  PMCR    FLPS       LL_PWR_DisableFlashLowPWRMode
  */
__STATIC_INLINE void LL_PWR_DisableFlashLowPWRMode(void)
{
  STM32_CLEAR_BIT(PWR->PMCR, PWR_PMCR_FLPS);
}

/**
  * @brief  Check whether the flash power down in stop mode is enabled.
  * @rmtoll
  *  PMCR    FLPS       LL_PWR_IsEnabledFlashLowPWRMode
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledFlashLowPWRMode(void)
{
  return ((STM32_READ_BIT(PWR->PMCR, PWR_PMCR_FLPS) == (PWR_PMCR_FLPS)) ? 1UL : 0UL);
}

/**
  * @brief  Enable memory content stop retention.
  * @rmtoll
  *  PMCR              SRAM1SO       LL_PWR_EnableMemoryStopRetention \n
#if PWR_PMCR_SRAM2_1_SO
  *  PMCR              SRAM2_1_SO    LL_PWR_EnableMemoryStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO    LL_PWR_EnableMemoryStopRetention \n
  *  PMCR              SRAM2_3_SO    LL_PWR_EnableMemoryStopRetention
#else
  *  PMCR              SRAM2SO       LL_PWR_EnableMemoryStopRetention
#endif
#endif
  * @param  memory This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM1_STOP_RETENTION
  *         @arg @ref LL_PWR_SRAM2_STOP_RETENTION
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE void LL_PWR_EnableMemoryStopRetention(uint32_t memory)
{
  STM32_CLEAR_BIT(PWR->PMCR, memory);
}

/**
  * @brief  Disable memory content stop retention.
  * @rmtoll
  *  PMCR              SRAM1SO       LL_PWR_DisableMemoryStopRetention \n
#if PWR_PMCR_SRAM2_1_SO
  *  PMCR              SRAM2_1_SO    LL_PWR_DisableMemoryStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO    LL_PWR_DisableMemoryStopRetention \n
  *  PMCR              SRAM2_3_SO    LL_PWR_DisableMemoryStopRetention
#else
  *  PMCR              SRAM2SO       LL_PWR_DisableMemoryStopRetention
#endif
#endif
  * @param  memory This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM1_STOP_RETENTION
  *         @arg @ref LL_PWR_SRAM2_STOP_RETENTION
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE void LL_PWR_DisableMemoryStopRetention(uint32_t memory)
{
  STM32_SET_BIT(PWR->PMCR, memory);
}

/**
  * @brief  Get memory content stop retention.
  * @rmtoll
  *  PMCR              SRAM1SO       LL_PWR_IsEnabledMemoryStopRetention \n
#if PWR_PMCR_SRAM2_1_SO
  *  PMCR              SRAM2_1_SO    LL_PWR_IsEnabledMemoryStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO    LL_PWR_IsEnabledMemoryStopRetention \n
  *  PMCR              SRAM2_3_SO    LL_PWR_IsEnabledMemoryStopRetention
#else
  *  PMCR              SRAM2SO    LL_PWR_IsEnabledMemoryStopRetention
#endif
#endif
  * @param  memory This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM1_STOP_RETENTION
  *         @arg @ref LL_PWR_SRAM2_STOP_RETENTION
  * @retval 0 if disabled, 1 if enabled
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledMemoryStopRetention(uint32_t memory)
{
  return ((STM32_READ_BIT(PWR->PMCR, memory) == (memory)) ? 0UL : 1UL);
}

#if defined(PWR_PMCR_SRAM2_1_SO)
/**
  * @brief  Enable SRAM2 page 1 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_EnableSRAM2Page1StopRetention
  */
__STATIC_INLINE void LL_PWR_EnableSRAM2Page1StopRetention(void)
{
  STM32_CLEAR_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE1_STOP_RETENTION);
}

/**
  * @brief  Enable SRAM2 page 2 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_2_SO   LL_PWR_EnableSRAM2Page2StopRetention
  */
__STATIC_INLINE void LL_PWR_EnableSRAM2Page2StopRetention(void)
{
  STM32_CLEAR_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE2_STOP_RETENTION);
}

#if defined(PWR_PMCR_SRAM2_3_SO)
/**
  * @brief  Enable SRAM2 page 3 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_3_SO   LL_PWR_EnableSRAM2Page3StopRetention
  */
__STATIC_INLINE void LL_PWR_EnableSRAM2Page3StopRetention(void)
{
  STM32_CLEAR_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE3_STOP_RETENTION);
}
#endif /* PWR_PMCR_SRAM2_3_SO */

/**
  * @brief  Enable SRAM2 page(s) content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_EnableSRAM2PagesStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO   LL_PWR_EnableSRAM2PagesStopRetention \n
  *  PMCR              SRAM2_3_SO   LL_PWR_EnableSRAM2PagesStopRetention
#else
  *  PMCR              SRAM2_2_SO   LL_PWR_EnableSRAM2PagesStopRetention
#endif
  * @param  pages This parameter is a bitfield representing each page of the SRAM2
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE void LL_PWR_EnableSRAM2PagesStopRetention(uint32_t pages)
{
  STM32_CLEAR_BIT(PWR->PMCR, (pages & LL_PWR_SRAM2_STOP_RETENTION));
}

/**
  * @brief  Disable SRAM2 page 1 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_DisableSRAM2Page1StopRetention
  */
__STATIC_INLINE void LL_PWR_DisableSRAM2Page1StopRetention(void)
{
  STM32_SET_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE1_STOP_RETENTION);
}

/**
  * @brief  Disable SRAM2 page 2 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_2_SO   LL_PWR_DisableSRAM2Page2StopRetention
  */
__STATIC_INLINE void LL_PWR_DisableSRAM2Page2StopRetention(void)
{
  STM32_SET_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE2_STOP_RETENTION);
}

#if defined(PWR_PMCR_SRAM2_3_SO)
/**
  * @brief  Disable SRAM2 page 3 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_3_SO   LL_PWR_DisableSRAM2Page3StopRetention
  */
__STATIC_INLINE void LL_PWR_DisableSRAM2Page3StopRetention(void)
{
  STM32_SET_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE3_STOP_RETENTION);
}
#endif /* PWR_PMCR_SRAM2_3_SO */

/**
  * @brief  Disable SRAM2 page(s) content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_DisableSRAM2PagesStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO   LL_PWR_DisableSRAM2PagesStopRetention \n
  *  PMCR              SRAM2_3_SO   LL_PWR_DisableSRAM2PagesStopRetention
#else
  *  PMCR              SRAM2_2_SO   LL_PWR_DisableSRAM2PagesStopRetention
#endif
  * @param  pages This parameter is a bitfield representing each page of the SRAM2
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE void LL_PWR_DisableSRAM2PagesStopRetention(uint32_t pages)
{
  STM32_SET_BIT(PWR->PMCR, (pages & LL_PWR_SRAM2_STOP_RETENTION));
}

/**
  * @brief  Check SRAM2 page 1 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_IsEnabledSRAM2Page1StopRetention
  * @retval 1 if the memory page is retained, 0 otherwise.
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledSRAM2Page1StopRetention(void)
{
  return ((STM32_READ_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE1_STOP_RETENTION) == (LL_PWR_SRAM2_PAGE1_STOP_RETENTION))
          ? 0UL : 1UL);
}

/**
  * @brief  Check SRAM2 page 2 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_2_SO   LL_PWR_IsEnabledSRAM2Page2StopRetention
  * @retval 1 if the memory page is retained, 0 otherwise.
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledSRAM2Page2StopRetention(void)
{
  return ((STM32_READ_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE2_STOP_RETENTION) == (LL_PWR_SRAM2_PAGE2_STOP_RETENTION))
          ? 0UL : 1UL);
}

#if defined(PWR_PMCR_SRAM2_3_SO)
/**
  * @brief  Check SRAM2 page 3 content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_3_SO   LL_PWR_IsEnabledSRAM2Page3StopRetention
  * @retval 1 if the memory page is retained, 0 otherwise.
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledSRAM2Page3StopRetention(void)
{
  return ((STM32_READ_BIT(PWR->PMCR, LL_PWR_SRAM2_PAGE3_STOP_RETENTION) == (LL_PWR_SRAM2_PAGE3_STOP_RETENTION))
          ? 0UL : 1UL);
}
#endif /* PWR_PMCR_SRAM2_3_SO */

/**
  * @brief  Check SRAM2 page content stop retention.
  * @rmtoll
  *  PMCR              SRAM2_1_SO   LL_PWR_IsEnabledSRAM2PagesStopRetention \n
#if PWR_PMCR_SRAM2_3_SO
  *  PMCR              SRAM2_2_SO   LL_PWR_IsEnabledSRAM2PagesStopRetention \n
  *  PMCR              SRAM2_3_SO   LL_PWR_IsEnabledSRAM2PagesStopRetention
#else
  *  PMCR              SRAM2_2_SO   LL_PWR_IsEnabledSRAM2PagesStopRetention
#endif
  * @param  page This parameter is a bitfield representing each page of the SRAM2
  * @retval 1 if the memory page is retained, 0 otherwise.
  * @note   (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledSRAM2PagesStopRetention(uint32_t page)
{
  return ((STM32_READ_BIT(PWR->PMCR, page) == (page)) ? 0UL : 1UL);
}
#endif /* PWR_PMCR_SRAM2_1_SO */

/**
  * @brief  Enable RTC domain write protection.
  * @rmtoll
  *  RTCCR    DRTCP       LL_PWR_EnableRTCDomainWriteProtection
  */
__STATIC_INLINE void LL_PWR_EnableRTCDomainWriteProtection(void)
{
  STM32_CLEAR_BIT(PWR->RTCCR, PWR_RTCCR_DRTCP);
}

/**
  * @brief  Disable RTC domain write protection.
  * @rmtoll
  *  RTCCR    DRTCP       LL_PWR_DisableRTCDomainWriteProtection
  */
__STATIC_INLINE void LL_PWR_DisableRTCDomainWriteProtection(void)
{
  STM32_SET_BIT(PWR->RTCCR, PWR_RTCCR_DRTCP);
}

/**
  * @brief  Check whether the RTC domain write protection is enabled.
  * @rmtoll
  *  RTCCR    DRTCP       LL_PWR_IsEnabledRTCDomainWriteProtection
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledRTCDomainWriteProtection(void)
{
  return ((STM32_READ_BIT(PWR->RTCCR, PWR_RTCCR_DRTCP) == (PWR_RTCCR_DRTCP)) ? 0UL : 1UL);
}

/**
  * @brief  Enable Power Voltage Detector.
  * @rmtoll
  *  VMCR    PVDEN       LL_PWR_EnablePVD
  */
__STATIC_INLINE void LL_PWR_EnablePVD(void)
{
  STM32_SET_BIT(PWR->VMCR, PWR_VMCR_PVDE);
}

/**
  * @brief  Disable Power Voltage Detector.
  * @rmtoll
  *  VMCR    PVDEN       LL_PWR_DisablePVD
  */
__STATIC_INLINE void LL_PWR_DisablePVD(void)
{
  STM32_CLEAR_BIT(PWR->VMCR, PWR_VMCR_PVDE);
}

/**
  * @brief  Check whether Power Voltage Detector is enabled.
  * @rmtoll
  *  VMCR    PVDEN       LL_PWR_IsEnabledPVD
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledPVD(void)
{
  return ((STM32_READ_BIT(PWR->VMCR, PWR_VMCR_PVDE) == (PWR_VMCR_PVDE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the wake up pin 1.
  * @rmtoll
  *  WUCR    WUPEN1       LL_PWR_EnableWakeUpPin1
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin1(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_1);
}

/**
  * @brief  Enable the wake up pin 2.
  * @rmtoll
  *  WUCR    WUPEN2       LL_PWR_EnableWakeUpPin2
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin2(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_2);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Enable the wake up pin 3.
  * @rmtoll
  *  WUCR    WUPEN3       LL_PWR_EnableWakeUpPin3
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin3(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_3);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Enable the wake up pin 4.
  * @rmtoll
  *  WUCR    WUPEN4       LL_PWR_EnableWakeUpPin4
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin4(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_4);
}

/**
  * @brief  Enable the wake up pin 5.
  * @rmtoll
  *  WUCR    WUPEN5       LL_PWR_EnableWakeUpPin5
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin5(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_5);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Enable the wake up pin 6.
  * @rmtoll
  *  WUCR    WUPEN6       LL_PWR_EnableWakeUpPin6
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin6(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_6);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Enable the wake up pin 7.
  * @rmtoll
  *  WUCR    WUPEN7       LL_PWR_EnableWakeUpPin7
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin7(void)
{
  STM32_SET_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_7);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Enable the wake up pin_x.
  * @rmtoll
  *  WUCR    WUPENx       LL_PWR_EnableWakeUpPin
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_ALL
  */
__STATIC_INLINE void LL_PWR_EnableWakeUpPin(uint32_t wakeup_pin)
{
  STM32_SET_BIT(PWR->WUCR, wakeup_pin);
}

/**
  * @brief  Disable the wake up pin 1.
  * @rmtoll
  *  WUCR    WUPEN1       LL_PWR_DisableWakeUpPin1
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin1(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_1);
}

/**
  * @brief  Disable the wake up pin 2.
  * @rmtoll
  *  WUCR    WUPEN2       LL_PWR_DisableWakeUpPin2
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin2(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_2);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Disable the wake up pin 3.
  * @rmtoll
  *  WUCR    WUPEN3       LL_PWR_DisableWakeUpPin3
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin3(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_3);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Disable the wake up pin 4.
  * @rmtoll
  *  WUCR    WUPEN4       LL_PWR_DisableWakeUpPin4
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin4(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_4);
}

/**
  * @brief  Disable the wake up pin 5.
  * @rmtoll
  *  WUCR    WUPEN5       LL_PWR_DisableWakeUpPin5
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin5(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_5);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Disable the wake up pin 6.
  * @rmtoll
  *  WUCR    WUPEN6       LL_PWR_DisableWakeUpPin6
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin6(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_6);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Disable the wake up pin 7.
  * @rmtoll
  *  WUCR    WUPEN7       LL_PWR_DisableWakeUpPin7
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin7(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_7);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Disable the wake up pin_x.
  * @rmtoll
  *  WUCR    WUPENx       LL_PWR_DisableWakeUpPin
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  */
__STATIC_INLINE void LL_PWR_DisableWakeUpPin(uint32_t wakeup_pin)
{
  STM32_CLEAR_BIT(PWR->WUCR, wakeup_pin);
}

/**
  * @brief  Check if the wake up pin 1 is enabled.
  * @rmtoll
  *  WUCR    WUPEN1       LL_PWR_IsEnabledWakeUpPin1
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin1(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_1) == (LL_PWR_WAKEUP_PIN_1)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the wake up pin 2 is enabled.
  * @rmtoll
  *  WUCR    WUPEN2       LL_PWR_IsEnabledWakeUpPin2
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin2(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_2) == (LL_PWR_WAKEUP_PIN_2)) ? 1UL : 0UL);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Check if the wake up pin 3 is enabled.
  * @rmtoll
  *  WUCR    WUPEN3       LL_PWR_IsEnabledWakeUpPin3
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin3(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_3) == (LL_PWR_WAKEUP_PIN_3)) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Check if the wake up pin 4 is enabled.
  * @rmtoll
  *  WUCR    WUPEN4       LL_PWR_IsEnabledWakeUpPin4
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin4(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_4) == (LL_PWR_WAKEUP_PIN_4)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the wake up pin 5 is enabled.
  * @rmtoll
  *  WUCR    WUPEN5       LL_PWR_IsEnabledWakeUpPin5
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin5(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_5) == (LL_PWR_WAKEUP_PIN_5)) ? 1UL : 0UL);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Check if the wake up pin 6 is enabled.
  * @rmtoll
  *  WUCR    WUPEN6       LL_PWR_IsEnabledWakeUpPin6
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin6(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_6) == (LL_PWR_WAKEUP_PIN_6)) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Check if the wake up pin 7 is enabled.
  * @rmtoll
  *  WUCR    WUPEN7       LL_PWR_IsEnabledWakeUpPin7
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin7(void)
{
  return ((STM32_READ_BIT(PWR->WUCR, LL_PWR_WAKEUP_PIN_7) == (LL_PWR_WAKEUP_PIN_7)) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Check if the wake up pin_x is enabled.
  * @rmtoll
  *  WUCR    WUPENx       LL_PWR_IsEnabledWakeUpPin
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledWakeUpPin(uint32_t wakeup_pin)
{
  return ((STM32_READ_BIT(PWR->WUCR, wakeup_pin) == (wakeup_pin)) ? 1UL : 0UL);
}

/**
  * @brief  Set the wake up pin 1 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP1       LL_PWR_SetWakeUpPin1PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_1 << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 2 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP2       LL_PWR_SetWakeUpPin2PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_2 << PWR_WUCR_WUPP1_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the wake up pin 3 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP3       LL_PWR_SetWakeUpPin3PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_3 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the wake up pin 4 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP4       LL_PWR_SetWakeUpPin4PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_4 << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 5 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP5       LL_PWR_SetWakeUpPin5PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_5 << PWR_WUCR_WUPP1_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the wake up pin 6 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP6       LL_PWR_SetWakeUpPin6PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_6 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the wake up pin 7 polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPP7       LL_PWR_SetWakeUpPin7PolarityLow
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7PolarityLow(void)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_7 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the wake up pin polarity low for the event detection.
  * @rmtoll
  *  WUCR          WUPPx       LL_PWR_SetWakeUpPinPolarityLow
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_ALL
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPolarityLow(uint32_t wakeup_pin)
{
  STM32_SET_BIT(PWR->WUCR, (uint32_t)(wakeup_pin << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 1 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP1       LL_PWR_SetWakeUpPin1PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_1 << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 2 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP2       LL_PWR_SetWakeUpPin2PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_2 << PWR_WUCR_WUPP1_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the wake up pin 3 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP3       LL_PWR_SetWakeUpPin3PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_3 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the wake up pin 4 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP4       LL_PWR_SetWakeUpPin4PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_4 << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 5 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP5       LL_PWR_SetWakeUpPin5PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_5 << PWR_WUCR_WUPP1_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the wake up pin 6 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP6       LL_PWR_SetWakeUpPin6PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_6 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the wake up pin 7 polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPP7       LL_PWR_SetWakeUpPin7PolarityHigh
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7PolarityHigh(void)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(LL_PWR_WAKEUP_PIN_7 << PWR_WUCR_WUPP1_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the wake up pin polarity high for the event detection.
  * @rmtoll
  *  WUCR          WUPPx       LL_PWR_SetWakeUpPinPolarityHigh
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_ALL
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPolarityHigh(uint32_t wakeup_pin)
{
  STM32_CLEAR_BIT(PWR->WUCR, (uint32_t)(wakeup_pin << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 1 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP1       LL_PWR_SetWakeUpPin1Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP1, (uint32_t)(polarity << PWR_WUCR_WUPP1_Pos));
}

/**
  * @brief  Set the wake up pin 2 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP2       LL_PWR_SetWakeUpPin2Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP2, (uint32_t)(polarity << PWR_WUCR_WUPP2_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the wake up pin 3 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP3       LL_PWR_SetWakeUpPin3Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP3, (uint32_t)(polarity << PWR_WUCR_WUPP3_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the wake up pin 4 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP4       LL_PWR_SetWakeUpPin4Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP4, (uint32_t)(polarity << PWR_WUCR_WUPP4_Pos));
}

/**
  * @brief  Set the wake up pin 5 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP5       LL_PWR_SetWakeUpPin5Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP5, (uint32_t)(polarity << PWR_WUCR_WUPP5_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the wake up pin 6 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP6       LL_PWR_SetWakeUpPin6Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP6, (uint32_t)(polarity << PWR_WUCR_WUPP6_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the wake up pin 7 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP7       LL_PWR_SetWakeUpPin7Polarity
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7Polarity(uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPP7, (uint32_t)(polarity << PWR_WUCR_WUPP7_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the wake up pin polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPPx       LL_PWR_SetWakeUpPinPolarity
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_HIGH
  *         @arg @ref LL_PWR_WAKEUP_PIN_POLARITY_LOW
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPolarity(uint32_t wakeup_pin, uint32_t polarity)
{
  STM32_MODIFY_REG(PWR->WUCR, (uint32_t)(PWR_WUCR_WUPP1 << STM32_POSITION_VAL(wakeup_pin)), \
                   (uint32_t)(polarity << (PWR_WUCR_WUPP1_Pos + STM32_POSITION_VAL(wakeup_pin))));
}

/**
  * @brief  Get the wake up pin 1 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP1       LL_PWR_GetWakeUpPin1Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin1Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP1)) != 0U) ? 1UL : 0UL);
}

/**
  * @brief  Get the wake up pin 2 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP2       LL_PWR_GetWakeUpPin2Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin2Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP2)) != 0U) ? 1UL : 0UL);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Get the wake up pin 3 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP3       LL_PWR_GetWakeUpPin3Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin3Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP3)) != 0U) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Get the wake up pin 4 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP4       LL_PWR_GetWakeUpPin4Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin4Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP4)) != 0U) ? 1UL : 0UL);
}

/**
  * @brief  Get the wake up pin 5 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP5       LL_PWR_GetWakeUpPin5Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin5Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP5)) != 0U) ? 1UL : 0UL);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Get the wake up pin 6 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP6       LL_PWR_GetWakeUpPin6Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin6Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP6)) != 0U) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Get the wake up pin 7 polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPP7       LL_PWR_GetWakeUpPin7Polarity
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin7Polarity(void)
{
  return (((STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPP7)) != 0U) ? 1UL : 0UL);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Get the wake up pin polarity for the event detection.
  * @rmtoll
  *  WUCR          WUPPx       LL_PWR_GetWakeUpPinPolarity
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @return State of bit (1 : polarity low or 0 : polarity high).
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPinPolarity(uint32_t wakeup_pin)
{
  return (((STM32_READ_BIT(PWR->WUCR, (uint32_t)(PWR_WUCR_WUPP1 << STM32_POSITION_VAL(wakeup_pin)))) != 0U) \
          ? 1UL : 0UL);
}

/**
  * @brief  Set the Wake-Up pin 1 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD1       LL_PWR_SetWakeUpPin1PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD1, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD1_Pos));
}

/**
  * @brief  Set the Wake-Up pin 2 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD2       LL_PWR_SetWakeUpPin2PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD2, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD2_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the Wake-Up pin 3 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD3       LL_PWR_SetWakeUpPin3PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD3, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD3_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the Wake-Up pin 4 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD4       LL_PWR_SetWakeUpPin4PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD4, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD4_Pos));
}

/**
  * @brief  Set the Wake-Up pin 5 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD5       LL_PWR_SetWakeUpPin5PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD5, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD5_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the Wake-Up pin 6 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD6       LL_PWR_SetWakeUpPin6PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD6, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD6_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the Wake-Up pin 7 Pull None.
  * @rmtoll
  *  WUCR   WUPPUPD7       LL_PWR_SetWakeUpPin7PullNone
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7PullNone(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD7, (LL_PWR_WAKEUP_PIN_PULL_NO << PWR_WUCR_WUPPUPD7_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the Wake-Up pin Pull None.
  * @rmtoll
  *  WUCR   WUPPUPDx       LL_PWR_SetWakeUpPinPullNone
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPullNone(uint32_t wakeup_pin)
{
  STM32_MODIFY_REG(PWR->WUCR,
                   (PWR_WUCR_WUPPUPD1 << ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * (STM32_POSITION_VAL(wakeup_pin) & \
                                           0xFU)) & \
                                          LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)),
                   (LL_PWR_WAKEUP_PIN_PULL_NO << ((PWR_WUCR_WUPPUPD1_Pos + ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                                             STM32_POSITION_VAL(wakeup_pin)) & 0xFU)) &\
                                                  LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)));
}

/**
  * @brief  Set the Wake-Up pin 1 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD1       LL_PWR_SetWakeUpPin1PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD1, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD1_Pos));
}

/**
  * @brief  Set the Wake-Up pin 2 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD2       LL_PWR_SetWakeUpPin2PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD2, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD2_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the Wake-Up pin 3 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD3       LL_PWR_SetWakeUpPin3PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD3, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD3_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the Wake-Up pin 4 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD4       LL_PWR_SetWakeUpPin4PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD4, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD4_Pos));
}

/**
  * @brief  Set the Wake-Up pin 5 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD5       LL_PWR_SetWakeUpPin5PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD5, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD5_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the Wake-Up pin 6 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD6       LL_PWR_SetWakeUpPin6PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD6, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD6_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the Wake-Up pin 7 Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPD7       LL_PWR_SetWakeUpPin7PullUp
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7PullUp(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD7, (LL_PWR_WAKEUP_PIN_PULL_UP << PWR_WUCR_WUPPUPD7_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the Wake-Up pin Pull Up.
  * @rmtoll
  *  WUCR   WUPPUPDx       LL_PWR_SetWakeUpPinPullUp
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPullUp(uint32_t wakeup_pin)
{
  STM32_MODIFY_REG(PWR->WUCR,
                   (PWR_WUCR_WUPPUPD1 << ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * (STM32_POSITION_VAL(wakeup_pin) & \
                                           0xFU)) & \
                                          LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)),
                   (LL_PWR_WAKEUP_PIN_PULL_UP << ((PWR_WUCR_WUPPUPD1_Pos + ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                                             STM32_POSITION_VAL(wakeup_pin)) & 0xFU)) &\
                                                  LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)));
}

/**
  * @brief  Set the Wake-Up pin 1 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD1       LL_PWR_SetWakeUpPin1PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD1, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD1_Pos));
}

/**
  * @brief  Set the Wake-Up pin 2 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD2       LL_PWR_SetWakeUpPin2PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD2, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD2_Pos));
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the Wake-Up pin 3 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD3       LL_PWR_SetWakeUpPin3PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD3, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD3_Pos));
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the Wake-Up pin 4 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD4       LL_PWR_SetWakeUpPin4PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD4, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD4_Pos));
}

/**
  * @brief  Set the Wake-Up pin 5 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD5       LL_PWR_SetWakeUpPin5PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD5, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD5_Pos));
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the Wake-Up pin 6 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD6       LL_PWR_SetWakeUpPin6PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD6, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD6_Pos));
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the Wake-Up pin 7 Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPD7       LL_PWR_SetWakeUpPin7PullDown
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7PullDown(void)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD7, (LL_PWR_WAKEUP_PIN_PULL_DOWN << PWR_WUCR_WUPPUPD7_Pos));
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the Wake-Up pin Pull Down.
  * @rmtoll
  *  WUCR   WUPPUPDx       LL_PWR_SetWakeUpPinPullDown
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPullDown(uint32_t wakeup_pin)
{
  STM32_MODIFY_REG(PWR->WUCR,
                   (PWR_WUCR_WUPPUPD1 << ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * (STM32_POSITION_VAL(wakeup_pin) & \
                                           0xFU)) & \
                                          LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)),
                   (LL_PWR_WAKEUP_PIN_PULL_DOWN << ((PWR_WUCR_WUPPUPD1_Pos + ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                                               STM32_POSITION_VAL(wakeup_pin)) \
                                                                              & 0xFU)) & \
                                                    LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)));
}

/**
  * @brief  Set the Wake-Up pin 1 pull.
  * @rmtoll
  *  WUCR   WUPPUPD1       LL_PWR_SetWakeUpPin1Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin1Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD1, pull << PWR_WUCR_WUPPUPD1_Pos);
}

/**
  * @brief  Set the Wake-Up pin 2 pull.
  * @rmtoll
  *  WUCR   WUPPUPD2       LL_PWR_SetWakeUpPin2Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin2Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD2, pull << PWR_WUCR_WUPPUPD2_Pos);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Set the Wake-Up pin 3 pull.
  * @rmtoll
  *  WUCR   WUPPUPD3       LL_PWR_SetWakeUpPin3Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin3Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD3, pull << PWR_WUCR_WUPPUPD3_Pos);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Set the Wake-Up pin 4 pull.
  * @rmtoll
  *  WUCR   WUPPUPD4       LL_PWR_SetWakeUpPin4Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin4Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD4, pull << PWR_WUCR_WUPPUPD4_Pos);
}

/**
  * @brief  Set the Wake-Up pin 5 pull.
  * @rmtoll
  *  WUCR   WUPPUPD5       LL_PWR_SetWakeUpPin5Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin5Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD5, pull << PWR_WUCR_WUPPUPD5_Pos);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Set the Wake-Up pin 6 pull.
  * @rmtoll
  *  WUCR   WUPPUPD6       LL_PWR_SetWakeUpPin6Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin6Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD6, pull << PWR_WUCR_WUPPUPD6_Pos);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Set the Wake-Up pin 7 pull.
  * @rmtoll
  *  WUCR   WUPPUPD7       LL_PWR_SetWakeUpPin7Pull
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPin7Pull(uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, PWR_WUCR_WUPPUPD7, pull << PWR_WUCR_WUPPUPD7_Pos);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Set the Wake-Up pin pull.
  * @rmtoll
  *  WUCR   WUPPUPDx       LL_PWR_SetWakeUpPinPull
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE void LL_PWR_SetWakeUpPinPull(uint32_t wakeup_pin, uint32_t pull)
{
  STM32_MODIFY_REG(PWR->WUCR, (PWR_WUCR_WUPPUPD1 << (LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                     (STM32_POSITION_VAL(wakeup_pin)))), \
                   pull << (PWR_WUCR_WUPPUPD1_Pos + \
                            (LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * (STM32_POSITION_VAL(wakeup_pin)))));
}

/**
  * @brief  Get the Wake-Up pin 1 pull.
  * @rmtoll
  *  WUCR   WUPPUPD1       LL_PWR_GetWakeUpPin1Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin1Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD1);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD1_Pos);
}

/**
  * @brief  Get the Wake-Up pin 2 pull.
  * @rmtoll
  *  WUCR   WUPPUPD2       LL_PWR_GetWakeUpPin2Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin2Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD2);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD2_Pos);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Get the Wake-Up pin 3 pull.
  * @rmtoll
  *  WUCR   WUPPUPD3       LL_PWR_GetWakeUpPin3Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin3Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD3);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD3_Pos);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Get the Wake-Up pin 4 pull.
  * @rmtoll
  *  WUCR   WUPPUPD4       LL_PWR_GetWakeUpPin4Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin4Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD4);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD4_Pos);
}

/**
  * @brief  Get the Wake-Up pin 5 pull.
  * @rmtoll
  *  WUCR   WUPPUPD5       LL_PWR_GetWakeUpPin5Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin5Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD5);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD5_Pos);
}

#if defined(PWR_WUCR_WUPEN6)
/**
  * @brief  Get the Wake-Up pin 6 pull.
  * @rmtoll
  *  WUCR   WUPPUPD6       LL_PWR_GetWakeUpPin6Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin6Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD6);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD6_Pos);
}
#endif /* PWR_WUCR_WUPEN6 */

#if defined(PWR_WUCR_WUPEN7)
/**
  * @brief  Get the Wake-Up pin 7 pull.
  * @rmtoll
  *  WUCR   WUPPUPD7       LL_PWR_GetWakeUpPin7Pull
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPin7Pull(void)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, PWR_WUCR_WUPPUPD7);

  return (uint32_t)(regValue >> PWR_WUCR_WUPPUPD7_Pos);
}
#endif /* PWR_WUCR_WUPEN7 */

/**
  * @brief  Get the Wake-Up pin pull.
  * @rmtoll
  *  WUCR   WUPPUPDx       LL_PWR_GetWakeUpPinPull
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_NO
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_UP
  *         @arg @ref LL_PWR_WAKEUP_PIN_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpPinPull(uint32_t wakeup_pin)
{
  uint32_t regValue = STM32_READ_BIT(PWR->WUCR, (PWR_WUCR_WUPPUPD1 << ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                                        (STM32_POSITION_VAL(wakeup_pin) & 0xFU)) & \
                                                                       LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)));

  return (uint32_t)(regValue >> ((PWR_WUCR_WUPPUPD1_Pos + ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                            STM32_POSITION_VAL(wakeup_pin)) & 0xFU)) & \
                                 LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK));
}

/**
  * @brief  Enable IO retention.
  * @rmtoll
  *  IORETR        IORETEN           LL_PWR_EnableIORetentionStandbyMode
  */
__STATIC_INLINE void LL_PWR_EnableIORetentionStandbyMode(void)
{
  STM32_SET_BIT(PWR->IORETR, PWR_IORETR_IORETEN);
}

/**
  * @brief  Disable IO retention.
  * @rmtoll
  *  IORETR        IORETEN           LL_PWR_DisableIORetentionStandbyMode
  */
__STATIC_INLINE void LL_PWR_DisableIORetentionStandbyMode(void)
{
  STM32_CLEAR_BIT(PWR->IORETR, PWR_IORETR_IORETEN);
}

/**
  * @brief  Check whether I/O retention is enabled.
  * @rmtoll
  *  IORETR        IORETEN           LL_PWR_IsEnabledIORetentionStandbyMode
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledIORetentionStandbyMode(void)
{
  return ((STM32_READ_BIT(PWR->IORETR, PWR_IORETR_IORETEN) == (PWR_IORETR_IORETEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable JTAGIO retention.
  * @rmtoll
  *  IORETR    JTAGIORETEN       LL_PWR_EnableJTAGIORetentionStandbyMode
  */
__STATIC_INLINE void LL_PWR_EnableJTAGIORetentionStandbyMode(void)
{
  STM32_SET_BIT(PWR->IORETR, PWR_IORETR_JTAGIORETEN);
}

/**
  * @brief  Disable JTAGIO retention.
  * @rmtoll
  *  IORETR     JTAGIORETEN      LL_PWR_DisableJTAGIORetentionStandbyMode
  */
__STATIC_INLINE void LL_PWR_DisableJTAGIORetentionStandbyMode(void)
{
  STM32_CLEAR_BIT(PWR->IORETR, PWR_IORETR_JTAGIORETEN);
}

/**
  * @brief  Check whether JTAGIO retention is enabled.
  * @rmtoll
  *  IORETR     JTAGIORETEN          LL_PWR_IsEnabledJTAGIORetentionStandbyMode
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledJTAGIORetentionStandbyMode(void)
{
  return ((STM32_READ_BIT(PWR->IORETR, PWR_IORETR_JTAGIORETEN) == (PWR_IORETR_JTAGIORETEN)) ? 1UL : 0UL);
}
/**
  * @}
  */

/** @defgroup PWR_LL_EF_FLAG_MANAGEMENT PWR FLAG Management
  * @{
  */

/**
  * @brief  Indicate whether the system was in standby mode or not.
  * @rmtoll
  *  PMSR          SBF          LL_PWR_IsActiveFlag_SB
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_SB(void)
{
  return ((STM32_READ_BIT(PWR->PMSR, PWR_PMSR_SBF) == (PWR_PMSR_SBF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether the system was in stop mode or not.
  * @rmtoll
  *  PMSR          STOPF          LL_PWR_IsActiveFlag_STOP
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_STOP(void)
{
  return ((STM32_READ_BIT(PWR->PMSR, PWR_PMSR_STOPF) == (PWR_PMSR_STOPF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether the VDD voltage is below the threshold or not.
  * @rmtoll
  *  VMSR          PVDO          LL_PWR_IsActiveFlag_PVDO
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_PVDO(void)
{
  return ((STM32_READ_BIT(PWR->VMSR, PWR_VMSR_PVDO) == (PWR_VMSR_PVDO)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 1.
  * @rmtoll
  *  WUSR          WUF1          LL_PWR_IsActiveFlag_WU1
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU1(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF1) == (PWR_WUSR_WUF1)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 2.
  * @rmtoll
  *  WUSR          WUF2          LL_PWR_IsActiveFlag_WU2
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU2(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF2) == (PWR_WUSR_WUF2)) ? 1UL : 0UL);
}

#if defined (PWR_WUSR_WUF3)
/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 3.
  * @rmtoll
  *  WUSR          WUF3          LL_PWR_IsActiveFlag_WU3
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU3(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF3) == (PWR_WUSR_WUF3)) ? 1UL : 0UL);
}
#endif /* PWR_WUSR_WUF3 */

/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 4.
  * @rmtoll
  *  WUSR          WUF4          LL_PWR_IsActiveFlag_WU4
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU4(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF4) == (PWR_WUSR_WUF4)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 5.
  * @rmtoll
  *  WUSR          WUF5          LL_PWR_IsActiveFlag_WU5
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU5(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF5) == (PWR_WUSR_WUF5)) ? 1UL : 0UL);
}

#if defined (PWR_WUSR_WUF6)
/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 6.
  * @rmtoll
  *  WUSR          WUF6          LL_PWR_IsActiveFlag_WU6
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU6(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF6) == (PWR_WUSR_WUF6)) ? 1UL : 0UL);
}
#endif /* PWR_WUSR_WUF6 */

#if defined (PWR_WUSR_WUF7)
/**
  * @brief  Indicate whether a wakeup event is detected on wake up pin 7.
  * @rmtoll
  *  WUSR          WUF7          LL_PWR_IsActiveFlag_WU7
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU7(void)
{
  return ((STM32_READ_BIT(PWR->WUSR, PWR_WUSR_WUF7) == (PWR_WUSR_WUF7)) ? 1UL : 0UL);
}
#endif /* PWR_WUSR_WUF7 */

/**
  * @brief  Indicate whether a wakeup event has been detected on the selected wake up pin.
  * @rmtoll
  *  WUSR          WUF1          LL_PWR_IsActiveFlag_WU \n
  *  WUSR          WUF2          LL_PWR_IsActiveFlag_WU \n
#if PWR_WUSR_WUF3
  *  WUSR          WUF3          LL_PWR_IsActiveFlag_WU \n
#endif
  *  WUSR          WUF4          LL_PWR_IsActiveFlag_WU \n
  *  WUSR          WUF5          LL_PWR_IsActiveFlag_WU \n
#if PWR_WUSR_WUF6
  *  WUSR          WUF6          LL_PWR_IsActiveFlag_WU
#endif
#if PWR_WUSR_WUF7
  *  WUSR          WUF7          LL_PWR_IsActiveFlag_WU
#endif
  * @param  wakeup_pin
  *         This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  * @return State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_WU(uint32_t wakeup_pin)
{
  return ((STM32_READ_BIT(PWR->WUSR, wakeup_pin) == (wakeup_pin)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Stop flag.
  * @rmtoll
  *  PMCR          CSSF          LL_PWR_ClearFlag_STOP
  * @warning STOPF and SBF flags are cleared together using this API.
  */
__STATIC_INLINE void LL_PWR_ClearFlag_STOP(void)
{
  STM32_SET_BIT(PWR->PMCR, PWR_PMCR_CSSF);
}

/**
  * @brief  Clear Standby flag.
  * @rmtoll
  *  PMCR          CSSF          LL_PWR_ClearFlag_SB
  * @warning STOPF and SBF flags are cleared together using this API.
  */
__STATIC_INLINE void LL_PWR_ClearFlag_SB(void)
{
  STM32_SET_BIT(PWR->PMCR, PWR_PMCR_CSSF);
}

/**
  * @brief  Clear wake up flag 1.
  * @rmtoll
  *  WUSCR          CWUF1         LL_PWR_ClearFlag_WU1
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU1(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF1);
}

/**
  * @brief  Clear wake up flag 2.
  * @rmtoll
  *  WUSCR          CWUF2         LL_PWR_ClearFlag_WU2
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU2(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF2);
}

#if defined(PWR_WUCR_WUPEN3)
/**
  * @brief  Clear wake up flag 3.
  * @rmtoll
  *  WUSCR          CWUF3         LL_PWR_ClearFlag_WU3
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU3(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF3);
}
#endif /* PWR_WUCR_WUPEN3 */

/**
  * @brief  Clear wake up flag 4.
  * @rmtoll
  *  WUSCR          CWUF4         LL_PWR_ClearFlag_WU4
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU4(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF4);
}

/**
  * @brief  Clear wake up flag 5.
  * @rmtoll
  *  WUSCR          CWUF5         LL_PWR_ClearFlag_WU5
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU5(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF5);
}

#if defined (PWR_WUSCR_CWUF6)
/**
  * @brief  Clear wake up flag 6.
  * @rmtoll
  *  WUSCR          CWUF6         LL_PWR_ClearFlag_WU6
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU6(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF6);
}
#endif /* PWR_WUSCR_CWUF6 */

#if defined (PWR_WUSCR_CWUF7)
/**
  * @brief  Clear wake up flag 7.
  * @rmtoll
  *  WUSCR          CWUF7         LL_PWR_ClearFlag_WU7
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU7(void)
{
  STM32_WRITE_REG(PWR->WUSCR, PWR_WUSCR_CWUF7);
}
#endif /* PWR_WUSCR_CWUF7 */

/**
  * @brief  Clear wakeup pin flags.
  * @rmtoll
  *  WUSCR           WKUPC1           LL_PWR_ClearFlag_WU \n
  *  WUSCR           WKUPC2           LL_PWR_ClearFlag_WU \n
#if PWR_WUCR_WUPEN3
  *  WUSCR           WKUPC3           LL_PWR_ClearFlag_WU \n
#endif
  *  WUSCR           WKUPC4           LL_PWR_ClearFlag_WU \n
  *  WUSCR           WKUPC5           LL_PWR_ClearFlag_WU \n
#if PWR_WUCR_WUPEN6
  *  WUSCR           WKUPC6           LL_PWR_ClearFlag_WU \n
#endif
#if PWR_WUCR_WUPEN7
  *  WUSCR           WKUPC7           LL_PWR_ClearFlag_WU
#endif
  * @param  wakeup_pin
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_PWR_WAKEUP_PIN_1
  *         @arg @ref LL_PWR_WAKEUP_PIN_2
#if PWR_WUCR_WUPEN3
  *         @arg @ref LL_PWR_WAKEUP_PIN_3
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_4
  *         @arg @ref LL_PWR_WAKEUP_PIN_5
#if PWR_WUCR_WUPEN6
  *         @arg @ref LL_PWR_WAKEUP_PIN_6
#endif
#if PWR_WUCR_WUPEN7
  *         @arg @ref LL_PWR_WAKEUP_PIN_7
#endif
  *         @arg @ref LL_PWR_WAKEUP_PIN_ALL
  */
__STATIC_INLINE void LL_PWR_ClearFlag_WU(uint32_t wakeup_pin)
{
  STM32_WRITE_REG(PWR->WUSCR, wakeup_pin);
}
/**
  * @}
  */

/** @defgroup PWR_LL_EF_ATTRIBUTE_MANAGEMENT PWR Attribute Management
  * @{
  */
/**
  * @brief  Set the privilege attribute.
  * @rmtoll
  *  PRIVCFGR     PRIV        LL_PWR_SetPrivAttr
  * @param  item The item attribute to be configured.
  * @param  priv_attr This parameter can be one of the following values:
  *         @arg @ref LL_PWR_ATTR_NPRIV
  *         @arg @ref LL_PWR_ATTR_PRIV
  * @note  This register can be written only when the access is privileged.
  */
__STATIC_INLINE void LL_PWR_SetPrivAttr(uint32_t item, uint32_t priv_attr)
{
  STM32_MODIFY_REG(PWR->PRIVCFGR, item, (item * priv_attr));
}

/**
  * @brief  Get the privilege attribute.
  * @rmtoll
  *  PRIVCFGR     PRIV        LL_PWR_GetPrivAttr
  *
  * @param  item The item attribute to be queried.
  * @return Returned value can be one of the following values:
  *         @arg @ref LL_PWR_ATTR_NPRIV
  *         @arg @ref LL_PWR_ATTR_PRIV
  */
__STATIC_INLINE uint32_t LL_PWR_GetPrivAttr(uint32_t item)
{
  return ((STM32_READ_BIT(PWR->PRIVCFGR, item) == item) ? 1UL : 0UL);
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

#endif /* defined (PWR) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_LL_PWR_H */
