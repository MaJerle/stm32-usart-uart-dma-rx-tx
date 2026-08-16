/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_pwr.h
  * @brief   Header file for PWR HAL module.
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
#ifndef STM32C5XX_HAL_PWR_H
#define STM32C5XX_HAL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_pwr.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (PWR)
/** @defgroup PWR PWR
  * @{
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Exported_Types HAL PWR Types
  * @{
  */

/*! HAL PWR wakeup pin polarity enumeration definition */
typedef enum
{
  HAL_PWR_WAKEUP_PIN_POLARITY_LOW  = LL_PWR_WAKEUP_PIN_POLARITY_LOW,  /*!< Wakeup pin low polarity  */
  HAL_PWR_WAKEUP_PIN_POLARITY_HIGH = LL_PWR_WAKEUP_PIN_POLARITY_HIGH  /*!< Wakeup pin high polarity */
} hal_pwr_wakeup_pin_polarity_t;

/*! HAL PWR wakeup pin pull configuration enumeration definition */
typedef enum
{
  HAL_PWR_WAKEUP_PIN_PULL_NO   = LL_PWR_WAKEUP_PIN_PULL_NO,  /*!< Wakeup pin no pull   */
  HAL_PWR_WAKEUP_PIN_PULL_UP   = LL_PWR_WAKEUP_PIN_PULL_UP,  /*!< Wakeup pin pull up   */
  HAL_PWR_WAKEUP_PIN_PULL_DOWN = LL_PWR_WAKEUP_PIN_PULL_DOWN /*!< Wakeup pin pull down */
} hal_pwr_wakeup_pin_pull_t;

/*! HAL PWR wakeup pin status enumeration definition */
typedef enum
{
  HAL_PWR_WAKEUP_PIN_DISABLED = 0U, /*!< Wakeup pin disabled */
  HAL_PWR_WAKEUP_PIN_ENABLED  = 1U  /*!< Wakeup pin enabled  */
} hal_pwr_wakeup_pin_status_t;

/*! HAL PWR wakeup pin configuration structure definition */
typedef struct
{
  hal_pwr_wakeup_pin_polarity_t polarity; /*!< Wakeup pin polarity */
  hal_pwr_wakeup_pin_pull_t     pull;     /*!< Wakeup pin pull     */
} hal_pwr_wakeup_pin_config_t;

/*! HAL PWR RTC domain write protection status enumeration definition */
typedef enum
{
  HAL_PWR_RTC_DOMAIN_WRP_DISABLED = 0U, /*!< RTC domain write protection disabled */
  HAL_PWR_RTC_DOMAIN_WRP_ENABLED  = 1U  /*!< RTC domain write protection enabled  */
} hal_pwr_rtc_domain_wrp_status_t;

/*! HAL PWR low power mode entry request enumeration definition */
typedef enum
{
  HAL_PWR_LOW_PWR_MODE_WFE = 0U, /*!< Low power mode wait for event request     */
  HAL_PWR_LOW_PWR_MODE_WFI = 1U  /*!< Low power mode wait for interrupt request */
} hal_pwr_low_pwr_mode_entry_t;

/*! HAL PWR stop mode enumeration definition */
typedef enum
{
  HAL_PWR_STOP0_MODE = LL_PWR_STOP0_MODE, /*!< Stop 0 mode */
  HAL_PWR_STOP1_MODE = LL_PWR_STOP1_MODE  /*!< Stop 1 mode */
} hal_pwr_stop_mode_t;

/*! HAL PWR system power mode enumeration definition */
typedef enum
{
  HAL_PWR_SYSTEM_RUN_MODE      = 0xFFU,              /*!< System was in Run mode     */
  HAL_PWR_SYSTEM_STOP0_MODE    = LL_PWR_STOP0_MODE,  /*!< System was in Stop 0 mode  */
  HAL_PWR_SYSTEM_STOP1_MODE    = LL_PWR_STOP1_MODE,  /*!< System was in Stop 1 mode  */
  HAL_PWR_SYSTEM_STANDBY_MODE  = LL_PWR_STANDBY_MODE /*!< System was in Standby mode */
} hal_pwr_system_mode_t;

/*! HAL PWR core sleep enumeration definition */
typedef enum
{
  HAL_PWR_CORE_SLEEP      = 0U, /*!< Core sleep mode      */
  HAL_PWR_CORE_DEEP_SLEEP = 1U  /*!< Core deep sleep mode */
} hal_pwr_core_sleep_mode_t;

/*! HAL PWR core sleep enumeration definition */
typedef enum
{
  HAL_PWR_CORE_SLEEP_ON_EXIT_DISABLED = 0U, /*!< PWR Core sleep on exit disabled */
  HAL_PWR_CORE_SLEEP_ON_EXIT_ENABLED  = 1U  /*!< PWR Core sleep on exit enabled  */
} hal_pwr_core_sleep_on_exit_status_t;

/*! HAL PWR core send event on pending status enumeration definition */
typedef enum
{
  HAL_PWR_CORE_SEV_ON_PENDING_DISABLED = 0U, /*!< Core send event on pending disabled */
  HAL_PWR_CORE_SEV_ON_PENDING_ENABLED  = 1U  /*!< Core send event on pending enabled  */
} hal_pwr_core_sev_on_pending_status_t;

/*! HAL PWR programmable voltage detector status enumeration definition */
typedef enum
{
  HAL_PWR_PVD_DISABLED = 0U, /*!< PWR PVD disabled */
  HAL_PWR_PVD_ENABLED  = 1U  /*!< PWR PVD enabled  */
} hal_pwr_pvd_status_t;

/*! HAL PWR programmable voltage detector output enumeration definition */
typedef enum
{
  HAL_PWR_PVD_OUT_EQ_HIGH_THR = 0U, /*!< VDD equal to or higher than programmable voltage detector threshold  */
  HAL_PWR_PVD_OUT_LOW_THR     = 1U  /*!< VDD lower than programmable voltage detector threshold               */
} hal_pwr_pvd_out_t;

/*! HAL PWR memory retention selection enumeration definition */
typedef enum
{
  HAL_PWR_MEMORY_RETENTION_SRAM1_STOP = 0U,  /*!< SRAM1 memory retention in stop mode */
  HAL_PWR_MEMORY_RETENTION_SRAM2_STOP = 1U,  /*!< SRAM2 memory retention in stop mode */
} hal_pwr_memory_retention_t;

/*! HAL PWR memory retention status enumeration definition */
typedef enum
{
  HAL_PWR_MEMORY_RETENTION_DISABLED = 0U, /*!< Memory retention in stop mode disabled */
  HAL_PWR_MEMORY_RETENTION_ENABLED  = 1U  /*!< Memory retention in stop mode enabled  */
} hal_pwr_memory_retention_status_t;

#if defined(PWR_PMCR_SRAM2_1_SO)
/*! HAL PWR memory page retention status enumeration definition */
typedef enum
{
  HAL_PWR_MEMORY_PAGE_RETENTION_DISABLED = 0U, /*!< Memory page retention in stop mode disabled */
  HAL_PWR_MEMORY_PAGE_RETENTION_ENABLED  = 1U  /*!< Memory page retention in stop mode enabled  */
} hal_pwr_memory_page_retention_status_t;
#endif /* PWR_PMCR_SRAM2_1_SO */

/*! HAL PWR FLASH memory low power mode status enumeration definition */
typedef enum
{
  HAL_PWR_FLASH_LOW_PWR_MODE_DISABLED = 0U, /*!< PWR flash memory low power mode disabled */
  HAL_PWR_FLASH_LOW_PWR_MODE_ENABLED  = 1U  /*!< PWR flash memory low power mode enabled  */
} hal_pwr_flash_low_pwr_mode_status_t;

/*! HAL PWR IO retention selection enumeration definition */
typedef enum
{
  HAL_PWR_IO_RETENTION_JTAGIO = LL_PWR_IO_RETENTION_JTAGIO, /*!< PWR JTAGIO retention enabled               */
  HAL_PWR_IO_RETENTION_GPIO   = LL_PWR_IO_RETENTION_GPIO,   /*!< PWR GPIO retention enabled                 */
  HAL_PWR_IO_RETENTION_ALL    = LL_PWR_IO_RETENTION_ALL     /*!< PWR both JTAGIO and GPIO retention enabled */
} hal_pwr_io_retention_t;

/*! HAL PWR IO retention configuration status enumeration definition  */
typedef enum
{
  HAL_PWR_IO_RETENTION_DISABLED = 0U, /*!< IO retention mode disabled */
  HAL_PWR_IO_RETENTION_ENABLED  = 1U  /*!< IO retention mode enabled  */
} hal_pwr_io_retention_status_t;

/*! HAL PWR privilege attribute enumeration definition  */
typedef enum
{
  HAL_PWR_NPRIV = LL_PWR_ATTR_NPRIV,  /*!< Non-privileged attribute */
  HAL_PWR_PRIV  = LL_PWR_ATTR_PRIV    /*!< Privileged attribute     */
} hal_pwr_priv_attr_t;
/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Exported_Constants HAL PWR Constants
  * @{
  */

/** @defgroup PWR_wakeup_pin PWR wakeup pin definitions
  * @{
  */
#define HAL_PWR_WAKEUP_PIN_1   LL_PWR_WAKEUP_PIN_1   /*!< Wakeup pin 1   */
#define HAL_PWR_WAKEUP_PIN_2   LL_PWR_WAKEUP_PIN_2   /*!< Wakeup pin 2   */
#if defined(PWR_WUCR_WUPEN3)
#define HAL_PWR_WAKEUP_PIN_3   LL_PWR_WAKEUP_PIN_3   /*!< Wakeup pin 3   */
#endif /* PWR_WUCR_WUPEN3 */
#define HAL_PWR_WAKEUP_PIN_4   LL_PWR_WAKEUP_PIN_4   /*!< Wakeup pin 4   */
#define HAL_PWR_WAKEUP_PIN_5   LL_PWR_WAKEUP_PIN_5   /*!< Wakeup pin 5   */
#if defined(PWR_WUCR_WUPEN6)
#define HAL_PWR_WAKEUP_PIN_6   LL_PWR_WAKEUP_PIN_6   /*!< Wakeup pin 6   */
#endif /* PWR_WUCR_WUPEN6 */
#if defined(PWR_WUCR_WUPEN7)
#define HAL_PWR_WAKEUP_PIN_7   LL_PWR_WAKEUP_PIN_7   /*!< Wakeup pin 7   */
#endif /* PWR_WUCR_WUPEN7 */
#define HAL_PWR_WAKEUP_PIN_ALL LL_PWR_WAKEUP_PIN_ALL /*!< Wakeup pin all */
/**
  * @}
  */

/** @defgroup PWR_wakeup_source PWR wakeup source definitions
  * @{
  */
#define HAL_PWR_WAKEUP_SOURCE_1   LL_PWR_WAKEUP_PIN_1   /*!< Wakeup source 1   */
#define HAL_PWR_WAKEUP_SOURCE_2   LL_PWR_WAKEUP_PIN_2   /*!< Wakeup source 2   */
#if defined(PWR_WUCR_WUPEN3)
#define HAL_PWR_WAKEUP_SOURCE_3   LL_PWR_WAKEUP_PIN_3   /*!< Wakeup source 3   */
#endif /* PWR_WUCR_WUPEN3 */
#define HAL_PWR_WAKEUP_SOURCE_4   LL_PWR_WAKEUP_PIN_4   /*!< Wakeup source 4   */
#define HAL_PWR_WAKEUP_SOURCE_5   LL_PWR_WAKEUP_PIN_5   /*!< Wakeup source 5   */
#if defined(PWR_WUCR_WUPEN6)
#define HAL_PWR_WAKEUP_SOURCE_6   LL_PWR_WAKEUP_PIN_6   /*!< Wakeup source 6   */
#endif /* PWR_WUCR_WUPEN6 */
#if defined(PWR_WUCR_WUPEN7)
#define HAL_PWR_WAKEUP_SOURCE_7   LL_PWR_WAKEUP_PIN_7   /*!< Wakeup source 7   */
#endif /* PWR_WUCR_WUPEN7 */
#define HAL_PWR_WAKEUP_SOURCE_ALL LL_PWR_WAKEUP_PIN_ALL /*!< Wakeup source all */
/**
  * @}
  */

/** @defgroup PWR_privilege_items PWR privilege items definition
  * @{
  */
#define HAL_PWR_PRIV_ITEM_ALL LL_PWR_PRIV_ITEM_ALL /*!< All privilege items  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Exported_Functions HAL PWR Functions
  * @{
  */

/** @defgroup PWR_Exported_Functions_Group1 Wakeup pins management functions
  * @{
  */
hal_status_t HAL_PWR_LP_SetConfigWakeupPin(uint32_t wakeup_pin, const hal_pwr_wakeup_pin_config_t *p_config);
void HAL_PWR_LP_GetConfigWakeupPin(uint32_t wakeup_pin, hal_pwr_wakeup_pin_config_t *p_config);
void HAL_PWR_LP_EnableWakeupPin(uint32_t wakeup_pin);
void HAL_PWR_LP_DisableWakeupPin(uint32_t wakeup_pin);
hal_pwr_wakeup_pin_status_t HAL_PWR_LP_IsEnabledWakeupPin(uint32_t wakeup_pin);
uint32_t HAL_PWR_LP_GetWakeupSource(void);
void HAL_PWR_LP_CleanWakeupSource(uint32_t wakeup_source);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group2 RTC domain write protection management functions
  * @{
  */
void HAL_PWR_EnableRTCDomainWriteProtection(void);
void HAL_PWR_DisableRTCDomainWriteProtection(void);
hal_pwr_rtc_domain_wrp_status_t HAL_PWR_IsEnabledRTCDomainWriteProtection(void);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group3 Low power mode management functions
  * @{
  */
void HAL_PWR_ClearCorePendingEvent(void);
void HAL_PWR_EnterSleepMode(hal_pwr_low_pwr_mode_entry_t sleep_entry);
void HAL_PWR_EnterStopMode(hal_pwr_low_pwr_mode_entry_t stop_entry, hal_pwr_stop_mode_t stop_mode);
void HAL_PWR_EnterStandbyMode(void);
void HAL_PWR_SetCoreSleepMode(hal_pwr_core_sleep_mode_t sleep_mode);
hal_pwr_core_sleep_mode_t HAL_PWR_GetCoreSleepMode(void);

void HAL_PWR_EnableCoreSleepOnExit(void);
void HAL_PWR_DisableCoreSleepOnExit(void);
hal_pwr_core_sleep_on_exit_status_t HAL_PWR_IsEnabledCoreSleepOnExit(void);
void HAL_PWR_EnableCoreSendEventOnPending(void);
void HAL_PWR_DisableCoreSendEventOnPending(void);
hal_pwr_core_sev_on_pending_status_t HAL_PWR_IsEnabledCoreSendEventOnPending(void);

hal_pwr_system_mode_t HAL_PWR_GetPreviousSystemPowerMode(void);
void HAL_PWR_CleanPreviousSystemPowerMode(void);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group4 Voltage monitoring management functions
  * @{
  */
void HAL_PWR_EnableProgrammableVoltageDetector(void);
void HAL_PWR_DisableProgrammableVoltageDetector(void);
hal_pwr_pvd_status_t HAL_PWR_IsEnabledProgrammableVoltageDetector(void);
hal_pwr_pvd_out_t HAL_PWR_GetProgrammableVoltageDetectorOutput(void);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group5 Memory retention management functions
  * @{
  */
hal_status_t HAL_PWR_LP_EnableMemoryRetention(hal_pwr_memory_retention_t memory);
hal_status_t HAL_PWR_LP_DisableMemoryRetention(hal_pwr_memory_retention_t memory);
hal_pwr_memory_retention_status_t HAL_PWR_LP_IsEnabledMemoryRetention(hal_pwr_memory_retention_t memory);
#if defined(PWR_PMCR_SRAM2_1_SO)
hal_status_t HAL_PWR_LP_EnableMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                  uint32_t page_idx,
                                                  uint32_t page_nbr);
hal_status_t HAL_PWR_LP_DisableMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                   uint32_t page_idx,
                                                   uint32_t page_nbr);
hal_pwr_memory_page_retention_status_t HAL_PWR_LP_IsEnabledMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                                               uint32_t page_idx);
#endif /* PWR_PMCR_SRAM2_1_SO */
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group6 Memory management functions
  * @{
  */
void HAL_PWR_LP_EnableFlashLowPWRMode(void);
void HAL_PWR_LP_DisableFlashLowPWRMode(void);
hal_pwr_flash_low_pwr_mode_status_t HAL_PWR_LP_IsEnabledFlashLowPWRMode(void);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group7 I/O retention management functions
  * @{
  */
void HAL_PWR_LP_EnableIORetention(hal_pwr_io_retention_t io);
void HAL_PWR_LP_DisableIORetention(hal_pwr_io_retention_t io);
hal_pwr_io_retention_status_t HAL_PWR_LP_IsEnabledIORetention(hal_pwr_io_retention_t io);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group8 Privilege management functions
  * @{
  */
hal_status_t HAL_PWR_SetPrivAttr(uint32_t item, hal_pwr_priv_attr_t priv_attr);
hal_pwr_priv_attr_t HAL_PWR_GetPrivAttr(uint32_t item);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* PWR */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_PWR_H */
