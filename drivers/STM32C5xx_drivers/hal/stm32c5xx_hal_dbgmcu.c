/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_dbgmcu.c
  * @brief   DBGMCU HAL module driver.
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

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (DBGMCU)
#if defined(USE_HAL_DBGMCU_MODULE) && (USE_HAL_DBGMCU_MODULE == 1U)
/** @addtogroup DBGMCU
  * @{
  */

/** @defgroup DBGMCU_Introduction DBGMCU Introduction
  * @{

   The DBGMCU hardware abstraction layer provides a set of APIs to configure and control the DBGMCU (Debug MCU)
   peripheral on STM32 microcontrollers.

   This driver facilitates access to debugging features by enabling easy management of peripheral and clock behavior
   during debug sessions.

  The DBGMCU includes the following features:

    - Retrieve the revision identification and the device identification.

     - Maintain the clock and power for the system debug component during low power modes
       (such as Sleep, Stop and Standby modes).

    - Freeze and unfreeze the clock for specific peripherals when the CPU is halted in debug mode.

  */
/**
  * @}
  */

/** @defgroup DBGMCU_How_To_Use DBGMCU How To Use
  * @{

# How to use the DBGMCU HAL module driver

## The DBGMCU HAL driver can be used as follows:

Use these three API sets to:

1. Identify the device:
  Use this feature to get information about the device.
   - Get the device revision:
    - Get the device revision with HAL_DBGMCU_GetRevisionID().
   - Get the device identifier:
    - Get the device identifier with HAL_DBGMCU_GetDeviceID().

2. Debug during low power mode:
   Use the following APIs to debug peripherals during low power modes.
   - Enable or disable the debug module during Sleep, Stop and Standby modes:
     - Use HAL_DBGMCU_EnableDebugLowPowerMode() to enable the debug module and
       HAL_DBGMCU_DisableDebugLowPowerMode() to disable the debug module.
   - Check whether the debug module is enabled during Sleep, Stop and Standby modes:
     - Use HAL_DBGMCU_IsEnabledDebugLowPowerMode() to check the debug module status.

3. Freeze and unfreeze clock peripherals:
   The DBGMCU peripheral allows certain peripherals to be suspended in debug mode when the CPU is halted.
   - Freeze and unfreeze PPPi peripherals:
     - Use HAL_DBGMCU_PPPi_Freeze() to freeze PPPi peripherals and HAL_DBGMCU_PPPi_UnFreeze() to unfreeze them.
  */
/**
  * @}
  */

/** @defgroup DBGMCU_Configuration_Table DBGMCU Configuration Table
  * @{
# Configuration inside the DBGMCU driver

Config defines        | Description        | Default value | Note
----------------------| ---------------    | ------------- | -----------------------------------------------------------
USE_HAL_DBGMCU_MODULE | from hal_conf.h    | 1U            | When set, HAL DBGMCU module is enabled.
USE_ASSERT_DBG_PARAM  | from IDE           | None          | When defined, enable the params assert.

  */
/**
  * @}
  */

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/* Private define ----------------------------------------------------------------------------------------------------*/
/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup DBGMCU_Private_Macros DBGMCU Private Macros
  * @{
  */

/*! Set Low power mode (Sleep, Stop and Standby modes) check macro */
#define IS_DBGMCU_DEBUG_LP_MODE(mode)                    \
  ((((mode) & HAL_DBGMCU_LP_MODE_DEBUG_ALL)  != 0U)      \
   && (((mode) & (~HAL_DBGMCU_LP_MODE_DEBUG_ALL)) == 0U))

/*! Get low power mode (Sleep, Stop and Standby modes) check macro */
#define IS_DBGMCU_GET_DEBUG_LP_MODE(mode)        \
  (((mode) == HAL_DBGMCU_STOP_MODE_DEBUG)        \
   || ((mode) == HAL_DBGMCU_STANDBY_MODE_DEBUG)  \
   || ((mode) == HAL_DBGMCU_SLEEP_MODE_DEBUG))
/**
  * @}
  */

/* Private variables -------------------------------------------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @addtogroup DBGMCU_Exported_Functions
  * @{
  */

/** @addtogroup DBGMCU_Exported_Functions_Group1
  * @{
  This section provides functions to get the device identity:
  - Call HAL_DBGMCU_GetRevisionID() function to get the device revision identifier.
  - Call HAL_DBGMCU_GetDeviceID() function to get the device identifier.
  */

/**
  * @brief  Returns the device revision identifier.
  * @note   This field indicates the revision ID of the device.
  *         - For STM32C53x/54x:
  *             - 0x1000: revision A
  *             - 0x1001: revision Z
  *         - For STM32C55x/56x:
  *             - 0x1001: revision Z
  *             - 0x1003: revision Y
  *         - For STM32C59x/5Ax:
  *             - 0x1000: revision A
  *             - 0x1001: revision Z
  * @retval uint32_t Value of device revision ID.
  */
uint32_t HAL_DBGMCU_GetRevisionID(void)
{
  return LL_DBGMCU_GetRevisionID();
}

/**
  * @brief  Returns the device identifier.
  * @retval HAL_DBGMCU_DEV_ID_C53X_C542 device ID for STM32C53x/542.
  * @retval HAL_DBGMCU_DEV_ID_C55X_C562 device ID for STM32C55x/562.
  * @retval HAL_DBGMCU_DEV_ID_C59X_C5A3 device ID for STM32C59x/5A3.
  */
hal_dbgmcu_device_id_t HAL_DBGMCU_GetDeviceID(void)
{
  return (hal_dbgmcu_device_id_t)LL_DBGMCU_GetDeviceID();
}
/**
  * @}
  */

/** @addtogroup DBGMCU_Exported_Functions_Group2
  * @{
  This section provides functions allowing to debug peripherals during low power mode
  (Sleep, Stop and Standby modes):
  - Call HAL_DBGMCU_EnableDebugLowPowerMode() function to enable the debug module during
    Sleep, Stop and Standby modes.
  - Call HAL_DBGMCU_DisableDebugLowPowerMode() function to disable the debug module during
    Sleep, Stop and Standby modes.
  - Call HAL_DBGMCU_IsEnabledDebugLowPowerMode() function to check the debug module during
    Sleep, Stop and Standby modes activation.
  */

/**
  * @brief Enable the Debug Module during low power mode (Sleep, Stop and Standby modes).
  * @param mode This parameter can be one or a combination of the following values:
  *             @arg HAL_DBGMCU_SLEEP_MODE_DEBUG   : Debug during Sleep mode.
  *             @arg HAL_DBGMCU_STOP_MODE_DEBUG    : Debug during Stop modes.
  *             @arg HAL_DBGMCU_STANDBY_MODE_DEBUG : Debug during Standby mode.
  *             @arg HAL_DBGMCU_LP_MODE_DEBUG_ALL  : Debug during all Low power modes.
  */
void HAL_DBGMCU_EnableDebugLowPowerMode(uint32_t mode)
{
  ASSERT_DBG_PARAM(IS_DBGMCU_DEBUG_LP_MODE(mode));

  LL_DBGMCU_EnableDebugLowPowerMode(mode);
}

/**
  * @brief Disable the Debug Module during low power mode (Sleep, Stop and Standby modes).
  * @param mode This parameter can be one or a combination of the following values:
  *             @arg HAL_DBGMCU_SLEEP_MODE_DEBUG   : Debug during Sleep mode.
  *             @arg HAL_DBGMCU_STOP_MODE_DEBUG    : Debug during Stop modes.
  *             @arg HAL_DBGMCU_STANDBY_MODE_DEBUG : Debug during Standby mode.
  *             @arg HAL_DBGMCU_LP_MODE_DEBUG_ALL  : Debug during all Low power modes.
  */
void HAL_DBGMCU_DisableDebugLowPowerMode(uint32_t mode)
{
  ASSERT_DBG_PARAM(IS_DBGMCU_DEBUG_LP_MODE(mode));

  LL_DBGMCU_DisableDebugLowPowerMode(mode);
}

/**
  * @brief  Check that the Debug Module during low power mode (Sleep, Stop and Standby modes)
  * is enabled.
  * @param  mode This parameter can be one of the following values:
  *             @arg HAL_DBGMCU_SLEEP_MODE_DEBUG   : Debug during Sleep mode.
  *             @arg HAL_DBGMCU_STOP_MODE_DEBUG    : Debug during Stop modes.
  *             @arg HAL_DBGMCU_STANDBY_MODE_DEBUG : Debug during Standby mode.
  * @retval hal_dbgmcu_dbg_low_power_mode_status_t Debug in low power mode activation.
  */
hal_dbgmcu_dbg_low_power_mode_status_t HAL_DBGMCU_IsEnabledDebugLowPowerMode(uint32_t mode)
{
  ASSERT_DBG_PARAM(IS_DBGMCU_GET_DEBUG_LP_MODE(mode));

  return ((hal_dbgmcu_dbg_low_power_mode_status_t)LL_DBGMCU_IsEnabledDebugLowPowerMode(mode));
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
#endif /* USE_HAL_DBGMCU_MODULE */
#endif /* DBGMCU */

/**
  * @}
  */
