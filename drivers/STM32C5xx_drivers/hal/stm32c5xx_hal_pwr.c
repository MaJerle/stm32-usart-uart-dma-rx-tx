/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_pwr.c
  * @brief   PWR HAL module driver.
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

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @addtogroup PWR
  * @{
  */
/** @defgroup PWR_Introduction PWR Introduction
  * @{

  - The PWR peripheral in STM32 manages power modes (sleep, stop, standby) and RTC domain access.
  - It reduces power consumption, supports multiple wakeup sources, and is essential for battery-powered applications.
  - Control is achieved through PWR registers. Use HAL functions for simplified configuration.

  */
/**
  * @}
  */

/** @defgroup PWR_How_To_Use PWR How To Use
  * @{

# PWR peripheral overview

This section provides an overview of the supply architecture for the different power domains and of the supply
configuration controller.

This file provides firmware functions to manage the following features:
   - Wakeup pin management functions.
   - RTC domain write protection management functions.
   - Low power mode management functions.
   - Voltage monitoring management functions.
   - Item retention management functions.
   - I/O pull management functions.

# How to use the PWR HAL module driver

This module provides different sets of APIs that allow:

1. Manage the wakeup pin:
   Use a wakeup pin to wake up the system from low power modes.
   - Configure the wakeup pin polarity and pull without enabling it:
     - Call HAL_PWR_LP_SetConfigWakeupPin() to configure wakeup polarity and pull.
       Use the p_config pointer to provide the configuration.
       - When p_config is null, this function returns HAL_INVALID_PARAM.
   - Get the wakeup pin polarity and pull configuration:
     - Call HAL_PWR_LP_GetConfigWakeupPin() to get the wakeup pin polarity and pull configuration.
   - Enable, disable, and check the wakeup pin:
     - Call HAL_PWR_LP_EnableWakeupPin(), HAL_PWR_LP_DisableWakeupPin(), and
       HAL_PWR_LP_IsEnabledWakeupPin() to enable, disable, and check the wakeup pin.
   - Get and clean wakeup sources:
     - Call HAL_PWR_LP_GetWakeupSource() and HAL_PWR_LP_CleanWakeupSource() to read
       and clear wakeup sources.

2. Manage RTC domain write protection:
   After a system reset, the RTC domain is protected against possible unwanted write accesses.
   - Enable, disable, and check write protection for the RTC domain:
     - Call HAL_PWR_EnableRTCDomainWriteProtection(), HAL_PWR_DisableRTCDomainWriteProtection(), and
       HAL_PWR_IsEnabledRTCDomainWriteProtection() to enable, disable, and check write protection.

3. Manage low power mode:
  Use available power modes to reduce power consumption.
   - Clear the core pending event to clear the internal Cortex event before entering sleep or stop x mode:
     - Call HAL_PWR_ClearCorePendingEvent() to clear the core pending event.
   - Enter the MCU into low power modes:
     - Enter the MCU into sleep mode through a WFE or WFI request:
       - Call HAL_PWR_EnterSleepMode() to enter sleep mode.
     - Enter the MCU into stop x mode through a WFE or WFI request:
       - Call HAL_PWR_EnterStopMode() to enter stop x mode.
     - Enter the MCU into standby mode:
       - Call HAL_PWR_EnterStandbyMode() to enter standby mode.
   - Configure the core deep sleep mode:
     - Call HAL_PWR_SetCoreSleepMode() to configure the core deep sleep mode.
   - Get the core deep sleep mode configuration:
     - Call HAL_PWR_GetCoreSleepMode() to get the core deep sleep mode configuration.
   - Enable, disable, and check core sleep on exit. This feature allows the core to enter sleep mode immediately
     after interrupt handling completes without returning to the main program.
     - Call HAL_PWR_EnableCoreSleepOnExit(), HAL_PWR_DisableCoreSleepOnExit(), and
       HAL_PWR_IsEnabledCoreSleepOnExit() to enable, disable, and check core sleep on exit.
   - Enable, disable, and check core send event on pending. This feature allows the Cortex to generate an event
     signal whenever there is a pending interrupt or exception. Use this event signal to wake up the processor from
     a low-power state and ensure that the system responds promptly to interrupts.
     - Call HAL_PWR_EnableCoreSendEventOnPending(), HAL_PWR_DisableCoreSendEventOnPending(), and
       HAL_PWR_IsEnabledCoreSendEventOnPending() to enable, disable, and check core send event on pending.
   - Get and clean the previous system power mode:
     - Call HAL_PWR_GetPreviousSystemPowerMode() to get the previous system power mode.
     - Call HAL_PWR_CleanPreviousSystemPowerMode() to clean the previous system power mode flag(s)
       before entering a low power mode.

4. Manage monitoring:
  Use the monitor to manage the power supplies and supply domains.
   - Enable, disable, and check the programmable voltage detector:
     - Call HAL_PWR_EnableProgrammableVoltageDetector(), HAL_PWR_DisableProgrammableVoltageDetector(), and
       HAL_PWR_IsEnabledProgrammableVoltageDetector() to enable, disable, and check the programmable voltage
       detector.
   - Get the programmable voltage detector output:
     - Call HAL_PWR_GetProgrammableVoltageDetectorOutput() to read the PVD output level.

5. Manage memory retention:
   After entering low power mode, the volatile memory (SRAM) content can be retained or not according to application
   needs.
   - Enable, disable, and check memory retention:
     - Call HAL_PWR_LP_EnableMemoryRetention(), HAL_PWR_LP_DisableMemoryRetention(), and
       HAL_PWR_LP_IsEnabledMemoryRetention() to enable, disable, and check memory retention.
   - Enable, disable, and check memory retention per page (not available on all devices):
     - Call HAL_PWR_LP_EnableMemoryPageRetention(), HAL_PWR_LP_DisableMemoryPageRetention(), and
       HAL_PWR_LP_IsEnabledMemoryPageRetention() to enable, disable, and check memory retention per page.

6. Manage memory power modes:
   The flash memory can be configured to enter low power mode when the MCU enters a low power mode.
   - Enable, disable, and check the flash low power mode in stop modes:
     - Call HAL_PWR_LP_EnableFlashLowPWRMode(), HAL_PWR_LP_DisableFlashLowPWRMode(), and
       HAL_PWR_LP_IsEnabledFlashLowPWRMode() to enable, disable, and check the flash low power mode.

7. Manage the I/O retention:
   The I/O retention feature allows maintaining the state of I/Os during low-power modes. Several APIs are available
   to retain or release the output of I/Os.
   - Enable, disable, and check I/O retention:
     - Call HAL_PWR_LP_EnableIORetention(), HAL_PWR_LP_DisableIORetention(), and
       HAL_PWR_LP_IsEnabledIORetention() to enable, disable, and check I/O retention.

8. Manage privilege attribute:
   Use the privilege attribute to set the PWR register access mode (privileged or not).
   - Set and get the privilege attribute:
     - Call HAL_PWR_SetPrivAttr() and HAL_PWR_GetPrivAttr() to set and get the privilege attribute.
  */
/**
  * @}
  */

/** @defgroup PWR_Configuration_Table PWR Configuration Table
  * @{
## Configuration inside the PWR driver

Config defines       | Description        | Default value | Note
---------------------| ---------------    | ------------- | ------------------------------------------------------------
PRODUCT              | from IDE           | NA            | The selected device
USE_HAL_PWR_MODULE   | from hal_conf.h    | 1U            | When set, HAL PWR module is enabled.
USE_ASSERT_DBG_PARAM | from IDE           | None          | When defined, enable parameter assertions.
USE_HAL_CHECK_PARAM  | from hal_conf.h    | 0U            | When set, parameters are checked at runtime.
  */
/**
  * @}
  */
#if defined(PWR)
#if defined(USE_HAL_PWR_MODULE) && (USE_HAL_PWR_MODULE == 1)

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/* Private defines ---------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Private_Constants PWR Private Constants
  * @{
  */
/* Number of memory retention pages */
#define PWR_SRAM1_RETENTION_PAGES_MAX 0x01UL /*!< SRAM1 maximum page count     */
#if defined(PWR_PMCR_SRAM2_1_SO)
#if defined(PWR_PMCR_SRAM2_3_SO)
#define PWR_SRAM2_RETENTION_PAGES_MAX 0x03UL /*!< SRAM2 maximum page count     */
#else
#define PWR_SRAM2_RETENTION_PAGES_MAX 0x02UL /*!< SRAM2 maximum page count     */
#endif /* defined(PWR_PMCR_SRAM2_3_SO) */
#else
#define PWR_SRAM2_RETENTION_PAGES_MAX 0x01UL /*!< SRAM2 maximum page count     */
#endif /* PWR_PMCR_SRAM2_1_SO */

/*! Number of memory banks */
#define PWR_MEMORY_BANKS_NUMBER (uint32_t)(sizeof(PWR_MemoryFullRetentionMap) / sizeof(PWR_MemoryFullRetentionMap[0]))
/**
  * @}
  */

/* Private variables -------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Private_Variables PWR Private Variables
  * @{
  */
/*! Memory retention mapping table */
static const uint32_t PWR_MemoryFullRetentionMap[] =
{
  LL_PWR_SRAM1_STOP_RETENTION,
  LL_PWR_SRAM2_STOP_RETENTION
};

/*! Memory maximum page retention mapping table */
static const uint32_t PWR_MemoryMaxPagesRetentionMap[PWR_MEMORY_BANKS_NUMBER] =
{
  PWR_SRAM1_RETENTION_PAGES_MAX,
  PWR_SRAM2_RETENTION_PAGES_MAX
};
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup PWR_Private_Macros PWR Private Macros
  * @{
  */

/*! Set wakeup pins check macro */
#define IS_PWR_SET_WAKEUP_PIN(pin) \
  ((((pin) & (HAL_PWR_WAKEUP_PIN_ALL)) != 0U) && (((pin) & (~HAL_PWR_WAKEUP_PIN_ALL)) == 0U))

/*! Set wakeup sources check macro */
#define IS_PWR_SET_WAKEUP_SOURCE(source) \
  ((((source) & (HAL_PWR_WAKEUP_SOURCE_ALL)) != 0U) && (((source) & (~HAL_PWR_WAKEUP_SOURCE_ALL)) == 0U))

/*! Wakeup pin polarity check macro */
#define IS_PWR_WAKEUP_PIN_POLARITY(polarity) \
  (((polarity) == HAL_PWR_WAKEUP_PIN_POLARITY_HIGH) || ((polarity) == HAL_PWR_WAKEUP_PIN_POLARITY_LOW))

/*! Wakeup pin pull check macro */
#define IS_PWR_WAKEUP_PIN_PULL(pull)          \
  (((pull) == HAL_PWR_WAKEUP_PIN_PULL_NO)     \
   || ((pull) == HAL_PWR_WAKEUP_PIN_PULL_UP)  \
   || ((pull) == HAL_PWR_WAKEUP_PIN_PULL_DOWN))

/*! Get wakeup pins check macro */
#if defined(PWR_WUCR_WUPEN3) && defined(PWR_WUCR_WUPEN6) && defined(PWR_WUCR_WUPEN7)
#define IS_PWR_GET_WAKEUP_PIN(pin)    \
  (((pin) == HAL_PWR_WAKEUP_PIN_1)    \
   || ((pin) == HAL_PWR_WAKEUP_PIN_2) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_3) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_4) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_5) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_6) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_7))
#else
#define IS_PWR_GET_WAKEUP_PIN(pin)    \
  (((pin) == HAL_PWR_WAKEUP_PIN_1)    \
   || ((pin) == HAL_PWR_WAKEUP_PIN_2) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_4) \
   || ((pin) == HAL_PWR_WAKEUP_PIN_5))
#endif /* PWR_WUCR_WUPEN3 && PWR_WUCR_WUPEN6 && PWR_WUCR_WUPEN7 */

/*! Low power mode entry check macro */
#define IS_PWR_LP_MODE_ENTRY(entry) \
  (((entry) == (uint32_t)HAL_PWR_LOW_PWR_MODE_WFE) || ((entry) == (uint32_t)HAL_PWR_LOW_PWR_MODE_WFI))

/*! Stop mode check macro */
#define IS_PWR_STOP_MODE(mode)                 \
  (((mode) == (uint32_t)HAL_PWR_STOP0_MODE)    \
   || (((mode) == (uint32_t)HAL_PWR_STOP1_MODE)))

/*! Core sleep mode check macro */
#define IS_PWR_CORE_SLEEP_MODE(mode) \
  (((mode) == ((uint32_t)HAL_PWR_CORE_SLEEP)) || ((mode) == ((uint32_t)HAL_PWR_CORE_DEEP_SLEEP)))

/*! Memory retention check macro */
#define IS_PWR_SINGLE_PAGE_MEMORY_RETENTION(memory) \
  ((((uint32_t)(memory)) < PWR_MEMORY_BANKS_NUMBER) && ((PWR_MemoryMaxPagesRetentionMap[memory]) == 1U))

#if defined(PWR_PMCR_SRAM2_1_SO)
/*! Memory page retention check macro */
#define IS_PWR_MEMORY_PAGES_RETENTION(memory, page_idx, page_nbr)                     \
  ((((uint32_t)(memory)) < PWR_MEMORY_BANKS_NUMBER)                                   \
   && ((((page_idx) - 1U) + (page_nbr)) <= (PWR_MemoryMaxPagesRetentionMap[memory]))  \
   && ((((PWR_MemoryMaxPagesRetentionMap[memory]) > 1U)))                             \
   && ((((page_idx) - 1U) + (page_nbr)) > 0U))
#endif /* PWR_PMCR_SRAM2_1_SO */

/*! I/O selection retention check macro */
#define IS_PWR_SET_IO_RETENTION(io)                 \
  (((io) == (uint32_t)HAL_PWR_IO_RETENTION_JTAGIO)  \
   || ((io) == (uint32_t)HAL_PWR_IO_RETENTION_GPIO) \
   || ((io) == (uint32_t)HAL_PWR_IO_RETENTION_ALL))

/*! I/O selection retention enabled check macro */
#define IS_PWR_GET_IO_RETENTION(io)                 \
  (((io) == (uint32_t)HAL_PWR_IO_RETENTION_JTAGIO)  \
   || ((io) == (uint32_t)HAL_PWR_IO_RETENTION_GPIO))

/*! PWR set privilege item check macro */
#define IS_PWR_SET_PRIV_ITEM(item)  \
  ((((item) & HAL_PWR_PRIV_ITEM_ALL) != 0U) && (((item) & (~HAL_PWR_PRIV_ITEM_ALL)) == 0U))

/*! PWR get privilege item check macro */
#define IS_PWR_GET_PRIV_ITEM(item)  \
  ((item) == HAL_PWR_PRIV_ITEM_ALL)
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup PWR_Exported_Functions HAL PWR Functions
  * @{
  */

/** @addtogroup PWR_Exported_Functions_Group1 Wakeup pins management functions
  * @{
  This section provides functions to manage the wakeup pins.
  - Call HAL_PWR_LP_SetConfigWakeupPin() to configure the wakeup pin.
  - Call HAL_PWR_LP_GetConfigWakeupPin() to get the wakeup pin configuration.
  - Call HAL_PWR_LP_EnableWakeupPin() to enable the wakeup pin.
  - Call HAL_PWR_LP_DisableWakeupPin() to disable the wakeup pin.
  - Call HAL_PWR_LP_IsEnabledWakeupPin() to check whether the wakeup pin is enabled.
  - Call HAL_PWR_LP_GetWakeupSource() to get the wakeup source(s).
  - Call HAL_PWR_LP_CleanWakeupSource() to clear wakeup source(s).
  */

/**
  * @brief  Set wakeup pin configuration.
  * @param  wakeup_pin        This parameter can be a combination of @ref PWR_wakeup_pin.
  *         @arg @ref HAL_PWR_WAKEUP_PIN_1
  *         @arg @ref HAL_PWR_WAKEUP_PIN_2
  *         @arg @ref HAL_PWR_WAKEUP_PIN_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_4
  *         @arg @ref HAL_PWR_WAKEUP_PIN_5
  *         @arg @ref HAL_PWR_WAKEUP_PIN_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_ALL
  * @param  p_config          Pointer to a @ref hal_pwr_wakeup_pin_config_t structure.
  * @retval HAL_INVALID_PARAM p_config pointer is NULL.
  * @retval HAL_OK            Wakeup pin is configured correctly.
  * @note   (*) : Not available on all devices.
  */
hal_status_t HAL_PWR_LP_SetConfigWakeupPin(uint32_t wakeup_pin, const hal_pwr_wakeup_pin_config_t *p_config)
{
  uint32_t temp_pin = wakeup_pin;
  __IO uint32_t register_value = LL_PWR_READ_REG(WUCR);
  uint32_t position;
  uint32_t current_pin;

  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_PWR_SET_WAKEUP_PIN(wakeup_pin));
  ASSERT_DBG_PARAM(IS_PWR_WAKEUP_PIN_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_PWR_WAKEUP_PIN_PULL(p_config->pull));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Get wakeup pin information */
  position    = STM32_POSITION_VAL(temp_pin);
  current_pin = 1UL << position;

  while (temp_pin != 0U)
  {
    /* Mask values which will be modified */
    register_value &= ~(uint32_t)((LL_PWR_WAKEUP_PIN_REF << position) \
                                  + (LL_PWR_WAKEUP_PIN_PULL_REF << (position * LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET)));

    /* Compute new value */
    register_value |= (uint32_t)(((uint32_t)(p_config->polarity) << (LL_PWR_WAKEUP_PIN_REF_POS + position))
                                 + ((uint32_t)(p_config->pull) << (LL_PWR_WAKEUP_PIN_PULL_REF_POS \
                                                                   + (position \
                                                                      * LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET))));

    /* Update wakeup pin information */
    temp_pin    &= (~current_pin);
    position    = STM32_POSITION_VAL(temp_pin);
    current_pin = 1UL << position;
  }

  /* Set new value in one register access */
  LL_PWR_WRITE_REG(WUCR, register_value);

  return HAL_OK;
}

/**
  * @brief  Get wakeup pin configuration.
  * @param  wakeup_pin This parameter can be one of @ref PWR_wakeup_pin.
  *         @arg @ref HAL_PWR_WAKEUP_PIN_1
  *         @arg @ref HAL_PWR_WAKEUP_PIN_2
  *         @arg @ref HAL_PWR_WAKEUP_PIN_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_4
  *         @arg @ref HAL_PWR_WAKEUP_PIN_5
  *         @arg @ref HAL_PWR_WAKEUP_PIN_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_ALL
  * @param  p_config   Pointer to a @ref hal_pwr_wakeup_pin_config_t structure.
  * @note   (*) : Not available on all devices.
  */
void HAL_PWR_LP_GetConfigWakeupPin(uint32_t wakeup_pin, hal_pwr_wakeup_pin_config_t *p_config)
{
  uint32_t register_value = LL_PWR_READ_REG(WUCR);
  uint32_t temp = 0U;

  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_PWR_GET_WAKEUP_PIN(wakeup_pin));

  /* Get the configuration from register value */
  p_config->polarity = (hal_pwr_wakeup_pin_polarity_t)((register_value & (LL_PWR_WAKEUP_PIN_REF <<
                                                                          STM32_POSITION_VAL(wakeup_pin))) >>
                                                       (LL_PWR_WAKEUP_PIN_REF_POS + STM32_POSITION_VAL(wakeup_pin)));

  temp = (register_value & (LL_PWR_WAKEUP_PIN_PULL_REF << ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                            (STM32_POSITION_VAL(wakeup_pin) & 0xFU)) & \
                                                           LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK)));

  p_config->pull     = (hal_pwr_wakeup_pin_pull_t)(temp >> ((LL_PWR_WAKEUP_PIN_PULL_REF_POS +
                                                             ((LL_PWR_WAKEUP_PINS_PULL_SHIFT_OFFSET * \
                                                               STM32_POSITION_VAL(wakeup_pin)) & 0xFU)) & \
                                                            LL_PWR_WAKEUP_PINS_MAX_SHIFT_MASK));
}

/**
  * @brief  Enable the wakeup pin configuration.
  * @param  wakeup_pin This parameter can be one or a combination of @ref PWR_wakeup_pin.
  *         @arg @ref HAL_PWR_WAKEUP_PIN_1
  *         @arg @ref HAL_PWR_WAKEUP_PIN_2
  *         @arg @ref HAL_PWR_WAKEUP_PIN_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_4
  *         @arg @ref HAL_PWR_WAKEUP_PIN_5
  *         @arg @ref HAL_PWR_WAKEUP_PIN_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_ALL
  * @note   (*) : Not available on all devices.
  * @note   Wakeup pins are used to wake up the system from Standby mode.
  */
void HAL_PWR_LP_EnableWakeupPin(uint32_t wakeup_pin)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_WAKEUP_PIN(wakeup_pin));

  LL_PWR_EnableWakeUpPin(wakeup_pin);
}

/**
  * @brief  Disable the wakeup pin configuration.
  * @param  wakeup_pin This parameter can be one or a combination of @ref PWR_wakeup_pin.
  *         @arg @ref HAL_PWR_WAKEUP_PIN_1
  *         @arg @ref HAL_PWR_WAKEUP_PIN_2
  *         @arg @ref HAL_PWR_WAKEUP_PIN_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_4
  *         @arg @ref HAL_PWR_WAKEUP_PIN_5
  *         @arg @ref HAL_PWR_WAKEUP_PIN_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_ALL
  * @note   (*) : Not available on all devices.
  * @note   Wakeup pins are used to wake up the system from Standby mode.
  */
void HAL_PWR_LP_DisableWakeupPin(uint32_t wakeup_pin)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_WAKEUP_PIN(wakeup_pin));

  LL_PWR_DisableWakeUpPin(wakeup_pin);
}

/**
  * @brief  Check whether the wakeup pin is enabled.
  * @param  wakeup_pin                  This parameter can be one of @ref PWR_wakeup_pin.
  *         @arg @ref HAL_PWR_WAKEUP_PIN_1
  *         @arg @ref HAL_PWR_WAKEUP_PIN_2
  *         @arg @ref HAL_PWR_WAKEUP_PIN_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_4
  *         @arg @ref HAL_PWR_WAKEUP_PIN_5
  *         @arg @ref HAL_PWR_WAKEUP_PIN_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_PIN_7 (*)
  * @retval Wakeup pin status
  *         @arg @ref HAL_PWR_WAKEUP_PIN_DISABLED Wakeup pin disabled
  *         @arg @ref HAL_PWR_WAKEUP_PIN_ENABLED  Wakeup pin enabled
  * @note   (*) : Not available on all devices.
  */
hal_pwr_wakeup_pin_status_t HAL_PWR_LP_IsEnabledWakeupPin(uint32_t wakeup_pin)
{
  ASSERT_DBG_PARAM(IS_PWR_GET_WAKEUP_PIN(wakeup_pin));

  return ((hal_pwr_wakeup_pin_status_t)LL_PWR_IsEnabledWakeUpPin(wakeup_pin));
}

/**
  * @brief  Return the wakeup sources.
  * @retval Can be one of or a combination of @ref PWR_wakeup_source.
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_1
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_2
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_4
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_5
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_ALL
  * @note   (*) : Not available on all devices.
  * @warning This API returns the wakeup source flags which can be a combination of multiple sources.
  */
uint32_t HAL_PWR_LP_GetWakeupSource(void)
{
  return (LL_PWR_READ_REG(WUSR));
}

/**
  * @brief  Clean the selected wakeup source(s) flag(s).
  * @param  wakeup_source  This parameter can be one or a combination of @ref PWR_wakeup_source.
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_1
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_2
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_3 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_4
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_5
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_6 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_7 (*)
  *         @arg @ref HAL_PWR_WAKEUP_SOURCE_ALL
  * @note   (*) : Not available on all devices.
  */
void HAL_PWR_LP_CleanWakeupSource(uint32_t wakeup_source)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_WAKEUP_SOURCE(wakeup_source));

  LL_PWR_ClearFlag_WU(wakeup_source);
}
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group2 RTC domain write protection management functions
  * @{
  This section provides functions to manage RTC domain write protection:
  - Call HAL_PWR_EnableRTCDomainWriteProtection() to enable RTC domain write protection.
  - Call HAL_PWR_DisableRTCDomainWriteProtection() to disable RTC domain write protection.
  - Call HAL_PWR_IsEnabledRTCDomainWriteProtection() to check whether RTC domain write protection is enabled.
  */

/**
  * @brief  Enable the RTC domain write protection (RCC RTC domain control register RCC_BDCR, RTC registers,
  *         TAMP registers, backup registers and backup SRAM).
  * @note   After a system reset, the RTC domain is protected against possible unwanted write accesses.
  */
void HAL_PWR_EnableRTCDomainWriteProtection(void)
{
  LL_PWR_EnableRTCDomainWriteProtection();
}

/**
  * @brief  Disable the RTC domain write protection (RCC RTC domain control register RCC_BDCR, RTC registers,
  *         TAMP registers, backup registers and backup SRAM).
  */
void HAL_PWR_DisableRTCDomainWriteProtection(void)
{
  LL_PWR_DisableRTCDomainWriteProtection();
}

/**
  * @brief  Check that RTC domain write protection is enabled.
  * @retval RTC Domain write protection status
  *         @arg @ref HAL_PWR_RTC_DOMAIN_WRP_DISABLED RTC domain write protection disabled.
  *         @arg @ref HAL_PWR_RTC_DOMAIN_WRP_ENABLED  RTC domain write protection enabled.
  */
hal_pwr_rtc_domain_wrp_status_t HAL_PWR_IsEnabledRTCDomainWriteProtection(void)
{
  return ((hal_pwr_rtc_domain_wrp_status_t)LL_PWR_IsEnabledRTCDomainWriteProtection());
}
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group3 Low power mode management functions
  * @{
  This section provides functions to manage low power modes:
  - Call HAL_PWR_ClearCorePendingEvent() to clear the internal Cortex event before entering low power mode using WFE.
  - Call HAL_PWR_EnterSleepMode() to enter the core into sleep mode.
  - Call HAL_PWR_EnterStopMode() to enter the MCU into stop x mode.
  - Call HAL_PWR_EnterStandbyMode() to enter the MCU into standby mode.
  - Call HAL_PWR_SetCoreSleepMode() to configure the core sleep or deep sleep mode.
  - Call HAL_PWR_GetCoreSleepMode() to get the core sleep mode configuration.
  - Call HAL_PWR_EnableCoreSleepOnExit() to enable the core to re-enter sleep mode after interrupt handling completes.
  - Call HAL_PWR_DisableCoreSleepOnExit() to disable the core from re-entering sleep mode after interrupt handling
    completes.
  - Call HAL_PWR_IsEnabledCoreSleepOnExit() to check whether core sleep-on-exit is enabled.
  - Call HAL_PWR_EnableCoreSendEventOnPending() to enable the core to wake up after any pending event/interrupt.
  - Call HAL_PWR_DisableCoreSendEventOnPending() to disable the core from waking up after any pending event/interrupt.
  - Call HAL_PWR_IsEnabledCoreSendEventOnPending() to check whether core send event on pending is enabled.
  - Call HAL_PWR_GetPreviousSystemPowerMode() to get previous system power mode.
  - Call HAL_PWR_CleanPreviousSystemPowerMode() to clean previous system power mode flag.
  */

/**
  * @brief   Clear core pending event.
  * @note    Clear the pending event to enter a given core into Sleep or stop mode with WFE entry.
  * @warning Call this function just before APIs that enter Sleep and stop mode using a Wait For Event request.
  */
void HAL_PWR_ClearCorePendingEvent(void)
{
  __SEV();
  __WFE();
}

/**
  * @brief  Enter the core into sleep mode.
  * @param  sleep_entry Parameter of the @ref hal_pwr_low_pwr_mode_entry_t enumeration.
  */
void HAL_PWR_EnterSleepMode(hal_pwr_low_pwr_mode_entry_t sleep_entry)
{
  ASSERT_DBG_PARAM(IS_PWR_LP_MODE_ENTRY((uint32_t)sleep_entry));

  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  SCB_DisableDeepSleep();

  /* DSB to ensure there are no outstanding memory transactions prior to executing WFI/WFE and going to sleep. */
  __DSB();

  if (sleep_entry == HAL_PWR_LOW_PWR_MODE_WFE)
  {
    /* Wait For Event Request */
    __WFE();
  }
  else
  {
    /* Wait For Interrupt Request */
    __WFI();
  }
}

/**
  * @brief  Enter the MCU in stop mode.
  * @param  stop_entry This parameter is an element of @ref hal_pwr_low_pwr_mode_entry_t enumeration.
  * @param  stop_mode  This parameter is an element of @ref hal_pwr_stop_mode_t enumeration.
  */
void HAL_PWR_EnterStopMode(hal_pwr_low_pwr_mode_entry_t stop_entry, hal_pwr_stop_mode_t stop_mode)
{
  ASSERT_DBG_PARAM(IS_PWR_LP_MODE_ENTRY((uint32_t)stop_entry));
  ASSERT_DBG_PARAM(IS_PWR_STOP_MODE((uint32_t)stop_mode));

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB_EnableDeepSleep();

  LL_PWR_SetPowerMode((uint32_t)stop_mode);

  /* DSB to ensure there are no outstanding memory transactions prior to executing WFI/WFE and going to stop. */
  __DSB();

  if (stop_entry == HAL_PWR_LOW_PWR_MODE_WFE)
  {
    /* Wait For Event Request */
    __WFE();
  }
  else
  {
    /* Wait For Interrupt Request */
    __WFI();
  }

  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  SCB_DisableDeepSleep();
}

/**
  * @brief  Enter the MCU in Standby mode.
  */
void HAL_PWR_EnterStandbyMode(void)
{
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB_EnableDeepSleep();

  LL_PWR_SetPowerMode(LL_PWR_STANDBY_MODE);

  /* DSB to ensure there are no outstanding memory transactions prior to executing WFI and going to standby. */
  __DSB();

  /* Wait For Interrupt Request */
  __WFI();
}

/**
  * @brief  Set the core sleep mode configuration.
  * @param  sleep_mode This parameter is an element of @ref hal_pwr_core_sleep_mode_t enumeration.
  */
void HAL_PWR_SetCoreSleepMode(hal_pwr_core_sleep_mode_t sleep_mode)
{
  ASSERT_DBG_PARAM(IS_PWR_CORE_SLEEP_MODE((uint32_t)sleep_mode));

  if (sleep_mode == HAL_PWR_CORE_DEEP_SLEEP)
  {
    SCB_EnableDeepSleep();
  }
  else
  {
    SCB_DisableDeepSleep();
  }
}

/**
  * @brief  Get the core sleep mode configuration.
  * @retval HAL_PWR_CORE_SLEEP      Core sleep mode.
  * @retval HAL_PWR_CORE_DEEP_SLEEP Core deep sleep mode.
  */
hal_pwr_core_sleep_mode_t HAL_PWR_GetCoreSleepMode(void)
{
  return ((hal_pwr_core_sleep_mode_t)(SCB_IsEnabledDeepSleep()));
}

/**
  * @brief  Enable SLEEP-ON-EXIT feature when returning from handler mode to thread mode.
  */
void HAL_PWR_EnableCoreSleepOnExit(void)
{
  SCB_EnableSleepOnExit();
}

/**
  * @brief  Disable SLEEP-ON-EXIT feature when returning from handler mode to thread mode.
  */
void HAL_PWR_DisableCoreSleepOnExit(void)
{
  SCB_DisableSleepOnExit();
}

/**
  * @brief  Check whether core SLEEP-ON-EXIT is enabled.
  * @retval hal_pwr_core_sleep_on_exit_status_t Core sleep-on-exit status.
  */
hal_pwr_core_sleep_on_exit_status_t HAL_PWR_IsEnabledCoreSleepOnExit(void)
{
  return ((hal_pwr_core_sleep_on_exit_status_t)(SCB_IsEnabledSleepOnExit()));
}

/**
  * @brief  Enable core Send Event On Pending feature.
  */
void HAL_PWR_EnableCoreSendEventOnPending(void)
{
  SCB_EnableEventOnPend();
}

/**
  * @brief  Disable core Send Event On Pending.
  */
void HAL_PWR_DisableCoreSendEventOnPending(void)
{
  SCB_DisableEventOnPend();
}

/**
  * @brief  Get core Send Event On Pending status.
  * @retval HAL_PWR_CORE_SEV_ON_PENDING_DISABLED Core send event on pending disabled.
  * @retval HAL_PWR_CORE_SEV_ON_PENDING_ENABLED  Core send event on pending enabled.
  */
hal_pwr_core_sev_on_pending_status_t HAL_PWR_IsEnabledCoreSendEventOnPending(void)
{
  return ((hal_pwr_core_sev_on_pending_status_t)(SCB_IsEnabledEventOnPend()));
}

/**
  * @brief  Get the previous system power mode.
  * @retval hal_pwr_system_mode_t Previous power mode.
  * @note   Sleep mode is an ARM core mode and not a system power mode as it does not affect system clock on return
  *          from sleep, so it is not reported by this function.
  * @warning This function is expected to be called on return from low power mode so to take the necessary actions
  *           such as re-enabling clocks automatically disabled during low power mode.
  */
hal_pwr_system_mode_t HAL_PWR_GetPreviousSystemPowerMode(void)
{
  hal_pwr_system_mode_t previous_mode = HAL_PWR_SYSTEM_RUN_MODE;

  /* Check Standby flag */
  if (LL_PWR_IsActiveFlag_SB() != 0U)
  {
    previous_mode = HAL_PWR_SYSTEM_STANDBY_MODE;
  }
  else
  {
    /* Check Stop flag */
    if (LL_PWR_IsActiveFlag_STOP() != 0U)
    {
      /* Check which Stop Mode has been configured */
      if (LL_PWR_GetPowerMode() == LL_PWR_STOP0_MODE)
      {
        previous_mode = HAL_PWR_SYSTEM_STOP0_MODE;
      }
      else
      {
        previous_mode = HAL_PWR_SYSTEM_STOP1_MODE;
      }
    }
  }

  return previous_mode;
}

/**
  * @brief   Clean the previous system power mode flags.
  * @note    This function must be called before entering in low power mode to ensure that
  *           HAL_PWR_GetPreviousSystemPowerMode() will return the right information on return from low power mode.
  * @warning This function clears both standby and stop flags.
  */
void HAL_PWR_CleanPreviousSystemPowerMode(void)
{
  LL_PWR_ClearFlag_SB();
}
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group4 Voltage monitoring management functions
  * @{
  This section provides functions to manage voltage monitoring.
  - Call HAL_PWR_EnableProgrammableVoltageDetector() to enable the programmable voltage detector.
  - Call HAL_PWR_DisableProgrammableVoltageDetector() to disable the programmable voltage detector.
  - Call HAL_PWR_IsEnabledProgrammableVoltageDetector() to check whether the programmable voltage detector is
    enabled.
  - Call HAL_PWR_GetProgrammableVoltageDetectorOutput() to get the programmable voltage detector output value.
  */

/**
  * @brief  Enable the voltage threshold detection by the programmable voltage detector (PVD).
  */
void HAL_PWR_EnableProgrammableVoltageDetector(void)
{
  LL_PWR_EnablePVD();
}

/**
  * @brief  Disable the voltage threshold detection by the programmable voltage detector (PVD).
  */
void HAL_PWR_DisableProgrammableVoltageDetector(void)
{
  LL_PWR_DisablePVD();
}

/**
  * @brief  Check whether the programmable voltage detection is enabled.
  * @retval HAL_PWR_PVD_DISABLED Programmable voltage detection disabled.
  * @retval HAL_PWR_PVD_ENABLED  Programmable voltage detection enabled.
  */
hal_pwr_pvd_status_t HAL_PWR_IsEnabledProgrammableVoltageDetector(void)
{
  return ((hal_pwr_pvd_status_t)LL_PWR_IsEnabledPVD());
}

/**
  * @brief  Provide the current PVD output.
  * @retval HAL_PWR_PVD_OUT_EQ_HIGH_THR VDD is equal or higher than programmable voltage detector threshold.
  * @retval HAL_PWR_PVD_OUT_LOW_THR  VDD is lower than programmable voltage detector threshold.
  */
hal_pwr_pvd_out_t HAL_PWR_GetProgrammableVoltageDetectorOutput(void)
{
  return (hal_pwr_pvd_out_t)LL_PWR_IsActiveFlag_PVDO();
}

/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group5 Memory retention management functions
  * @{
 This section provides functions to manage memory content retention.
 - Call HAL_PWR_LP_EnableMemoryRetention() to enable memory retention.
 - Call HAL_PWR_LP_DisableMemoryRetention() to disable memory retention.
 - Call HAL_PWR_LP_IsEnabledMemoryRetention() to check whether selected memory retention is enabled.
 - Call HAL_PWR_LP_EnableMemoryPageRetention() to enable memory retention for selected pages. (*)
 - Call HAL_PWR_LP_DisableMemoryPageRetention() to disable memory retention for selected pages. (*)
 - Call HAL_PWR_LP_IsEnabledMemoryPageRetention() to check whether selected memory page retention is enabled. (*)
  * @note   (*) : Not available on all devices.
 */

/**
  * @brief  Enable memory retention in stop mode.
  * @param  memory This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @retval HAL_OK Memory retention has been correctly enabled.
  * @retval HAL_INVALID_PARAM The selected memory does not exist while USE_HAL_CHECK_PARAM is used.
  * @retval HAL_ERROR Selected memory is paginated, use HAL_PWR_LP_EnableMemoryPageRetention().
  */
hal_status_t HAL_PWR_LP_EnableMemoryRetention(hal_pwr_memory_retention_t memory)
{
  ASSERT_DBG_PARAM(IS_PWR_SINGLE_PAGE_MEMORY_RETENTION((uint32_t)memory));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check if the selected memory exists */
  if ((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
  {
    /* Selected memory does not exist */
    return HAL_INVALID_PARAM;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    /* Check if the selected memory is paginated or not */
    if (PWR_MemoryMaxPagesRetentionMap[memory] == 1U)
    {
      /* Enable Memory Retention */
      LL_PWR_EnableMemoryStopRetention(PWR_MemoryFullRetentionMap[memory]);
      return HAL_OK;
    }
    else
    {
      /* Paginated SRAM are managed by HAL_PWR_LP_EnableMemoryPageRetention() */
      return HAL_ERROR;
    }
  }
}

/**
  * @brief  Disable memory retention in stop mode.
  * @param  memory  This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @retval HAL_OK  Memory retention has been correctly disabled.
  * @retval HAL_INVALID_PARAM The selected memory does not exist while USE_HAL_CHECK_PARAM is used.
  * @retval HAL_ERROR Selected memory is paginated, use HAL_PWR_LP_DisableMemoryPageRetention().
  */
hal_status_t HAL_PWR_LP_DisableMemoryRetention(hal_pwr_memory_retention_t memory)
{
  ASSERT_DBG_PARAM(IS_PWR_SINGLE_PAGE_MEMORY_RETENTION((uint32_t)memory));

#if defined(USE_HAL_CHECK_PARAM)
  /* Check if the selected memory exists */
  if ((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
  {
    /* Selected memory does not exist */
    return HAL_INVALID_PARAM;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    /* Check if the selected memory is paginated or not */
    if (PWR_MemoryMaxPagesRetentionMap[memory] == 1U)
    {
      /* Disable Memory Retention */
      LL_PWR_DisableMemoryStopRetention(PWR_MemoryFullRetentionMap[memory]);
      return HAL_OK;
    }
    else
    {
      /* Paginated SRAM are managed by HAL_PWR_LP_DisableMemoryPageRetention() */
      return HAL_ERROR;
    }
  }
}

/**
  * @brief  Get memory retention status.
  * @param  memory  This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @retval HAL_PWR_MEMORY_RETENTION_DISABLED if the selected memory is not retained in stop mode or
  *         if the selected memory does not exist while USE_HAL_CHECK_PARAM is used.
  * @retval HAL_PWR_MEMORY_RETENTION_ENABLED  if the selected memory is retained in stop mode.
  * @note   If the ASSERT are disabled, HAL_PWR_MEMORY_RETENTION_DISABLED is returned for paginated memories.
  */
hal_pwr_memory_retention_status_t HAL_PWR_LP_IsEnabledMemoryRetention(hal_pwr_memory_retention_t memory)
{
  ASSERT_DBG_PARAM(IS_PWR_SINGLE_PAGE_MEMORY_RETENTION((uint32_t)memory));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check if the selected memory exists */
  if ((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
  {
    /* Selected memory does not exist */
    return HAL_PWR_MEMORY_RETENTION_DISABLED;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    if (PWR_MemoryMaxPagesRetentionMap[memory] == 1U)
    {
      /* Return the selected memory retention status */
      return ((hal_pwr_memory_retention_status_t)
              LL_PWR_IsEnabledMemoryStopRetention(PWR_MemoryFullRetentionMap[memory]));
    }
    else
    {
      /* Paginated SRAM are managed by HAL_PWR_LP_IsEnabledMemoryPageRetention() */
      return HAL_PWR_MEMORY_RETENTION_DISABLED;
    }
  }
}

#if defined(PWR_PMCR_SRAM2_1_SO)
/**
  * @brief  Enable memory page retention in stop mode.
  * @param  memory    This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @param  page_idx  the index of memory page (starting from 1).
  * @param  page_nbr  The memory pages number.
  * @retval HAL_OK    Memory page retention has been correctly enabled.
  * @retval HAL_INVALID_PARAM The selected memory does not exist, the page index is out of range or the selected number
  *         of pages does not fit the memory.
  * @retval HAL_ERROR Memory page retention could not be enabled, the selected memory is not paginated.
  */
hal_status_t HAL_PWR_LP_EnableMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                  uint32_t page_idx,
                                                  uint32_t page_nbr)
{
  uint32_t pages = 0;
  uint32_t logical_idx = 0;
  hal_status_t status = HAL_OK;

  ASSERT_DBG_PARAM(IS_PWR_MEMORY_PAGES_RETENTION((uint32_t)memory, page_idx, page_nbr));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check if the selected memory exists, the page index is valid and if the selected number of pages fits the memory */
  if (((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
      || (page_idx > PWR_MemoryMaxPagesRetentionMap[memory])
      || (((page_idx - 1U) + page_nbr) > PWR_MemoryMaxPagesRetentionMap[memory]))
  {
    status = HAL_INVALID_PARAM;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    if (PWR_MemoryMaxPagesRetentionMap[memory] != 1U)
    {
      for (uint32_t i = 0; i < page_nbr; ++i)
      {
        logical_idx = page_idx + i;
        switch (logical_idx)
        {
          /* SRAM2 Page 1 */
          case 1:
          {
            pages |= LL_PWR_SRAM2_PAGE1_STOP_RETENTION;
            break;
          }

          /* SRAM2 Page 2 */
          case 2:
          {
            pages |= LL_PWR_SRAM2_PAGE2_STOP_RETENTION;
            break;
          }

#if defined(PWR_PMCR_SRAM2_3_SO)
          /* SRAM2 Page 3 */
          case 3:
          {
            pages |= LL_PWR_SRAM2_PAGE3_STOP_RETENTION;
            break;
          }
#endif /* PWR_PMCR_SRAM2_3_SO */

          default:
          {
            /* Out-of-range cases */
            status = HAL_ERROR;
            break;
          }
        }
      }

      /* If no error occurred, enable the pages stop retention */
      if (status == HAL_OK)
      {
        /* Pages stop retention enabling */
        LL_PWR_EnableSRAM2PagesStopRetention(pages);
      }
    }
    else
    {
      /* Non paginated SRAM are managed by HAL_PWR_LP_EnableMemoryRetention() */
      status = HAL_ERROR;
    }
  }

  return status;
}

/**
  * @brief  Disable memory page retention in stop mode.
  * @param  memory    This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @param  page_idx  the index of memory page (starting from 1).
  * @param  page_nbr  The memory pages number.
  * @retval HAL_OK    Memory page retention has been correctly disabled.
  * @retval HAL_INVALID_PARAM The selected memory does not exist, the page index is out of range or the selected number
  *         of pages does not fit the memory.
  * @retval HAL_ERROR Memory page retention could not be disabled, the selected memory is not paginated.
  */
hal_status_t HAL_PWR_LP_DisableMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                   uint32_t page_idx,
                                                   uint32_t page_nbr)
{
  uint32_t pages = 0;
  uint32_t logical_idx = 0;
  hal_status_t status = HAL_OK;

  ASSERT_DBG_PARAM(IS_PWR_MEMORY_PAGES_RETENTION((uint32_t)memory, page_idx, page_nbr));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check if the selected memory exists, the page index is valid and if the selected number of pages fits the memory */
  if (((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
      || (page_idx > PWR_MemoryMaxPagesRetentionMap[memory])
      || (((page_idx - 1U) + page_nbr) > PWR_MemoryMaxPagesRetentionMap[memory]))
  {
    status = HAL_INVALID_PARAM;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    if (PWR_MemoryMaxPagesRetentionMap[memory] != 1U)
    {
      for (uint32_t i = 0; i < page_nbr; ++i)
      {
        logical_idx = page_idx + i;
        switch (logical_idx)
        {
          /* SRAM2 Page 1 */
          case 1:
          {
            pages |= LL_PWR_SRAM2_PAGE1_STOP_RETENTION;
            break;
          }

          /* SRAM2 Page 2 */
          case 2:
          {
            pages |= LL_PWR_SRAM2_PAGE2_STOP_RETENTION;
            break;
          }

#if defined(PWR_PMCR_SRAM2_3_SO)
          /* SRAM2 Page 3 */
          case 3:
          {
            pages |= LL_PWR_SRAM2_PAGE3_STOP_RETENTION;
            break;
          }
#endif /* PWR_PMCR_SRAM2_3_SO */

          default:
          {
            /* Out-of-range cases */
            status = HAL_ERROR;
            break;
          }
        }
      }

      /* If no error occurred, disable the pages stop retention */
      if (status == HAL_OK)
      {
        /* Pages stop retention disabling */
        LL_PWR_DisableSRAM2PagesStopRetention(pages);
      }
    }
    else
    {
      /* Non paginated SRAM are managed by HAL_PWR_LP_DisableMemoryRetention() */
      status = HAL_ERROR;
    }
  }

  return status;
}

/**
  * @brief  Check the selected memory page retention in stop mode status.
  * @param  memory    This parameter is an element of @ref hal_pwr_memory_retention_t enumeration.
  * @param  page_idx  the index of memory page (starting from 1).
  * @retval HAL_PWR_MEMORY_PAGE_RETENTION_DISABLED if the selected memory page is not retained in stop mode or
  *         if the selected memory does not exist while USE_HAL_CHECK_PARAM is used.
  * @retval HAL_PWR_MEMORY_PAGE_RETENTION_ENABLED  if the selected memory page is retained in stop mode.
  * @note   If the ASSERT are disabled, HAL_PWR_MEMORY_PAGE_RETENTION_DISABLED is returned for out-of-range pages or
  *         non-paginated memories.
  */
hal_pwr_memory_page_retention_status_t HAL_PWR_LP_IsEnabledMemoryPageRetention(hal_pwr_memory_retention_t memory,
                                                                               uint32_t page_idx)
{
  hal_pwr_memory_page_retention_status_t status = HAL_PWR_MEMORY_PAGE_RETENTION_DISABLED;
  uint32_t page = 0;

  ASSERT_DBG_PARAM(IS_PWR_MEMORY_PAGES_RETENTION((uint32_t)memory, page_idx, 1U));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check if the selected memory exists, the page index is valid and if the selected number of pages fits the memory */
  if (((uint32_t)memory >= PWR_MEMORY_BANKS_NUMBER)
      || (page_idx > PWR_MemoryMaxPagesRetentionMap[memory]))
  {
    /* Selected SRAM bank is not valid */
    status = HAL_PWR_MEMORY_PAGE_RETENTION_DISABLED;
  }
  else
#endif /* USE_HAL_CHECK_PARAM */
  {
    if (PWR_MemoryMaxPagesRetentionMap[memory] != 1U)
    {
      switch (page_idx)
      {
        /* SRAM2 Page 1 */
        case 1:
        {
          page = LL_PWR_SRAM2_PAGE1_STOP_RETENTION;
          break;
        }

        /* SRAM2 Page 2 */
        case 2:
        {
          page = LL_PWR_SRAM2_PAGE2_STOP_RETENTION;
          break;
        }

#if defined(PWR_PMCR_SRAM2_3_SO)
        /* SRAM2 Page 3 */
        case 3:
        {
          page = LL_PWR_SRAM2_PAGE3_STOP_RETENTION;
          break;
        }
#endif /* PWR_PMCR_SRAM2_3_SO */

        default:
        {
          /* Ignore out-of-range cases */
          break;
        }
      }

      /* Get memory page retention status */
      if ((LL_PWR_IsEnabledSRAM2PagesStopRetention(page) != 0UL) && (page != 0UL))
      {
        status = HAL_PWR_MEMORY_PAGE_RETENTION_ENABLED;
      }
    }
  }

  return status;
}
#endif /* PWR_PMCR_SRAM2_1_SO */
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group6 Memory management functions
  * @{
 This section provides functions to manage flash memory low power modes.
 - Call HAL_PWR_LP_EnableFlashLowPWRMode() to enable flash memory power down in stop 0/1 mode.
 - Call HAL_PWR_LP_DisableFlashLowPWRMode() to disable flash memory power down in stop 0/1 mode.
 - Call HAL_PWR_LP_IsEnabledFlashLowPWRMode() to check the flash memory power down status in stop 0/1 mode.
  */
/**
  * @brief  Enable the flash low power mode.
  */
void HAL_PWR_LP_EnableFlashLowPWRMode(void)
{
  LL_PWR_EnableFlashLowPWRMode();
}

/**
  * @brief  Disable the flash low power mode.
  */
void HAL_PWR_LP_DisableFlashLowPWRMode(void)
{
  LL_PWR_DisableFlashLowPWRMode();
}

/**
  * @brief  Check whether the flash low power mode is enabled.
  * @retval hal_pwr_Flash_low_pwr_mode_status_t Flash low power mode status.
  */
hal_pwr_flash_low_pwr_mode_status_t HAL_PWR_LP_IsEnabledFlashLowPWRMode(void)
{
  return (hal_pwr_flash_low_pwr_mode_status_t)LL_PWR_IsEnabledFlashLowPWRMode();
}
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group7 I/O retention management functions
  * @{
  This section provides functions to manage I/O retention in low power mode.
  - Call HAL_PWR_LP_EnableIORetention() to enable I/O (GPIO and/or JTAGIO) retention in Standby mode.
  - Call HAL_PWR_LP_DisableIORetention() to disable I/O (GPIO and/or JTAGIO) retention in Standby mode.
  - Call HAL_PWR_LP_IsEnabledIORetention() to check whether I/O (GPIO or JTAGIO) retention in Standby mode is enabled.
  */

/**
  * @brief  Enable the IO retention in Standby mode.
  * @param  io This parameter is an element or combination of elements of @ref hal_pwr_io_retention_t enumeration.
  *            @arg @ref HAL_PWR_IO_RETENTION_JTAGIO IO retention mode is enabled for JTAG I/Os (PA13, PA14, PA15, and
  *                       PB4).
  *            @arg @ref HAL_PWR_IO_RETENTION_GPIO   IO retention mode is enabled for all I/Os except the I/Os
  *                       supporting the Standby functionality and JTAG I/Os.
  *            @arg @ref HAL_PWR_IO_RETENTION_ALL    IO retention mode is enabled for all I/Os.
  * @note   The output is sampled and applied to the output I/O during Standby mode.
  */
void HAL_PWR_LP_EnableIORetention(hal_pwr_io_retention_t io)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_IO_RETENTION(((uint32_t)io)));

  LL_PWR_WRITE_REG(IORETR, (LL_PWR_READ_REG(IORETR) | (uint32_t)io));
}

/**
  * @brief  Disable the IO retention in Standby mode.
  * @param  io This parameter is an element of @ref hal_pwr_io_retention_t enumeration.
  *            @arg @ref HAL_PWR_IO_RETENTION_JTAGIO IO retention mode is disabled for JTAG I/Os (PA13, PA14, PA15, and
  *                       PB4).
  *            @arg @ref HAL_PWR_IO_RETENTION_GPIO   IO retention mode is disabled for all I/Os except the I/Os
  *                       supporting the Standby functionality and JTAG I/Os.
  *            @arg @ref HAL_PWR_IO_RETENTION_ALL    IO retention mode is disabled for all I/Os.
  */
void HAL_PWR_LP_DisableIORetention(hal_pwr_io_retention_t io)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_IO_RETENTION(((uint32_t)io)));

  LL_PWR_WRITE_REG(IORETR, (LL_PWR_READ_REG(IORETR) & ~(uint32_t)io));
}

/**
  * @brief  Check whether I/O retention in Standby mode is enabled.
  * @param  io This parameter is an element of @ref hal_pwr_io_retention_t enumeration.
  *            @arg @ref HAL_PWR_IO_RETENTION_JTAGIO IO retention mode is enabled for JTAG I/Os (PA13, PA14, PA15, and
  *                                                  PB4).
  *            @arg @ref HAL_PWR_IO_RETENTION_GPIO   IO retention mode is enabled for all I/Os except the I/Os
  *                                                  supporting the Standby functionality and JTAG I/Os.
  * @retval hal_pwr_io_retention_status_t The I/O retention status.
  */
hal_pwr_io_retention_status_t HAL_PWR_LP_IsEnabledIORetention(hal_pwr_io_retention_t io)
{
  hal_pwr_io_retention_status_t status = HAL_PWR_IO_RETENTION_DISABLED;

  ASSERT_DBG_PARAM(IS_PWR_GET_IO_RETENTION((uint32_t)io));

  if (io == HAL_PWR_IO_RETENTION_JTAGIO)
  {
    status = (hal_pwr_io_retention_status_t)LL_PWR_IsEnabledJTAGIORetentionStandbyMode();
  }

  if (io == HAL_PWR_IO_RETENTION_GPIO)
  {
    status = (hal_pwr_io_retention_status_t)LL_PWR_IsEnabledIORetentionStandbyMode();
  }

  return status;
}
/**
  * @}
  */

/** @addtogroup PWR_Exported_Functions_Group8 Privilege management functions
  * @{
  This section provides functions to manage privilege attributes.
  - Call HAL_PWR_SetPrivAttr() to set the privilege attribute.
  - Call HAL_PWR_GetPrivAttr() to get the privilege attribute.
  */
/**
  * @brief  Set privilege attribute.
  * @param  item The item attribute to be configured.
  * @param  priv_attr This parameter is an element of @ref hal_pwr_priv_attr_t enumeration:
  *         @arg @ref HAL_PWR_PRIV   Privileged attribute.
  *         @arg @ref HAL_PWR_NPRIV  Unprivileged attribute.
  * @retval HAL_ERROR  The function is called in unprivileged mode.
  * @retval HAL_OK     Privilege attribute has been correctly set.
  */
hal_status_t HAL_PWR_SetPrivAttr(uint32_t item, hal_pwr_priv_attr_t priv_attr)
{
  ASSERT_DBG_PARAM(IS_PWR_SET_PRIV_ITEM(item));

  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  LL_PWR_SetPrivAttr(item, (uint32_t)priv_attr);

  return HAL_OK;
}

/**
  * @brief  Get privilege attribute.
  * @param  item The item attribute to be queried.
  * @retval hal_pwr_priv_attr_t This parameter is an element of @ref hal_pwr_priv_attr_t enumeration.
  */
hal_pwr_priv_attr_t HAL_PWR_GetPrivAttr(uint32_t item)
{
  ASSERT_DBG_PARAM(IS_PWR_GET_PRIV_ITEM(item));

  return ((hal_pwr_priv_attr_t)LL_PWR_GetPrivAttr(item));
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_PWR_MODULE */
#endif /* PWR */
/**
  * @}
  */

/**
  * @}
  */
