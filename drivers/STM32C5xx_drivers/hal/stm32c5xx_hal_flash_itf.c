/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_flash_itf.c
  * @brief   This file provides FLASH ITF peripheral services.
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

#if defined (FLASH)
#if defined (USE_HAL_FLASH_MODULE) && (USE_HAL_FLASH_MODULE == 1)

/** @addtogroup FLASH_ITF
  * @{
  */

/** @defgroup FLASH_ITF_Introduction FLASH_ITF Introduction
  * @{

  The FLASH ITF Hardware Abstraction Layer (HAL) provides high-level, user-friendly interface APIs for managing
  non-volatile memory (FLASH) configuration and control on STM32 microcontrollers.

  This layer offers firmware functions to support key FLASH interface functionalities, including:

   - Lock and unlock management.
   - Control operations.
   - Privilege attributes.
   - Option byte configuration.
   - Option byte process interrupt handling.

  The abstraction provided by this layer ensures portability and simplifies application development
  across various STM32 microcontroller series.

  */
/**
  * @}
  */

/** @defgroup FLASH_ITF_How_To_Use FLASH ITF How To Use
  * @{
This file provides firmware functions to manage the following functionalities of the FLASH option bytes peripheral:

- Lock and unlock functions
- Configuration functions
- Launch functions

# FLASH option bytes main features

- The FLASH option bytes, often referred to as "Option Bytes" or "OB" in the context of microcontrollers, are a set of
  configuration settings that can be programmed to customize the behavior of the flash memory and other features of a
  microcontroller. These Option Bytes are typically stored in a special area of the flash memory and are used to control
  various aspects of the device's operation.

## Lock mechanism:

  - After reset, the FLASH option bytes are write-protected.
    Use the unlock sequence to run any operation on the FLASH option bytes.

## Option bytes:

  - Write protection area: The user area in flash memory can be protected against unwanted write operations.
                           Each block of up to 2 pages can be individually write-protected.

  - Read-out protection: The read-out protection protects the flash main memory, the option bytes, the backup
                         registers, the backup RAM and the SRAM to reach the highest level of security.
                         - 3 levels (L0, L2_wBS and L2) of protection available.
                         - It is mandatory to set a password OEM key for regression to L0 from L2_wBS and L2 levels.

  - Hide protection area: A part of the main flash memory area.

  - Boot lock: Several option bytes cannot be modified when the boot lock mechanism is enabled.

  - Boot address: The boot address option bytes are used to program a boot address in USER FLASH.

  - User option bytes: Option bytes include additional options to customize the behavior.
                       - Reset generation in low power mode, erase memories upon system reset, watchdog selection,
                         Independent watchdog counter freeze, Bank swapping, Single/Dual bank, SRAM ECC, Boot0,
                         Boot selection, EDATA area, Bootloader Interface.

# How to use the FLASH_ITF HAL module driver

## Lock and unlock functions:

- Use the HAL_FLASH_ITF_Lock() and HAL_FLASH_ITF_Unlock() functions to lock and unlock the access to the FLASH control
  register.
- Use the HAL_FLASH_ITF_IsLocked() function to check the lock access state to the FLASH control register.

## Configuration functions:

- Use the HAL_FLASH_ITF_SetLatency() function to set the FLASH latency.
- Use the HAL_FLASH_ITF_GetLatency() function to get the FLASH latency.
- Use the HAL_FLASH_ITF_SetProgrammingDelay() function to set the FLASH programming delay.
- Use the HAL_FLASH_ITF_GetProgrammingDelay() function to get the FLASH programming delay.
- Use the HAL_FLASH_ITF_EnablePrefetch() function to enable the FLASH prefetch.
- Use the HAL_FLASH_ITF_DisablePrefetch() function to disable the FLASH prefetch.
- Use the HAL_FLASH_ITF_IsEnabledPrefetch() function to check if the FLASH prefetch is enabled or disabled.
- Use the HAL_FLASH_ITF_SetEmptyBootLocation() function to set the FLASH empty boot location information.
- Use the HAL_FLASH_ITF_GetEmptyBootLocation() function to get the FLASH empty boot location information.
- Use the HAL_FLASH_ITF_SetHDPExtArea() function to set the HDP extended area.
- Use the HAL_FLASH_ITF_GetHDPExtArea() function to get the HDP extended area.
- Use the HAL_FLASH_ITF_IsLockedRDPOEMKey() function to check if the RDP OEM key is locked or unlocked.
- Use the HAL_FLASH_ITF_IsLockedRDPBSKey() function to check if the RDP BS key is locked or unlocked.
- Use the HAL_FLASH_ITF_ECC_EnableIT() function to enable the FLASH ECC interrupt.
- Use the HAL_FLASH_ITF_ECC_DisableIT() function to disable the FLASH ECC interrupt.
- Use the HAL_FLASH_ITF_ECC_IsEnabledIT() function to check if the FLASH ECC interrupt is enabled or disabled.

## Option bytes lock and unlock functions:

- Use the HAL_FLASH_ITF_OB_Lock() function to lock the access to the FLASH option bytes registers.
- Use the HAL_FLASH_ITF_OB_Unlock() function to unlock the access to the FLASH option bytes registers.
- Use the HAL_FLASH_ITF_OB_IsLocked() function to check the lock state of the access to the FLASH option bytes
  registers.

## Option bytes OTP lock and unlock functions:

- Use the HAL_FLASH_ITF_OB_LockOTPBlock() function to lock the selected OTP blocks.
- Use the HAL_FLASH_ITF_OB_IsLockedOTPBlock() function to check whether a selected OTP block is locked.

## EDATA Area configuration:
- Use the HAL_FLASH_ITF_OB_EnableEDATAArea() function to enable the EDATA area.
- Use the HAL_FLASH_ITF_OB_DisableEDATAArea() function to disable the EDATA area.
- Use the HAL_FLASH_ITF_OB_IsEnabledEDATAArea() function to check if the EDATA area is enabled or disabled.

## Write protection area configuration:

- Use the HAL_FLASH_ITF_OB_EnablePageWRP() function to enable the FLASH ITF OB pagewise write protection area
  configuration.
- Use the HAL_FLASH_ITF_OB_DisablePageWRP() function to disable the FLASH ITF OB pagewise write protection area
  configuration.
- Use the HAL_FLASH_ITF_OB_IsEnabledPageWRP() function to check if the FLASH ITF OB pagewise write protection area
  configuration is enabled or disabled.

## Hide protection configuration:

- Use the HAL_FLASH_ITF_OB_SetHDPArea() function to set the FLASH ITF OB hide protection area configuration.
- Use the HAL_FLASH_ITF_OB_GetHDPArea() function to get the FLASH ITF OB hide protection area configuration.

## Read-out protection configuration:

- Use the HAL_FLASH_ITF_OB_SetRDPLevel() function to set the FLASH ITF OB read-out protection level.
- Use the HAL_FLASH_ITF_OB_GetRDPLevel() function to get the FLASH ITF OB read-out protection level.
- Use the HAL_FLASH_ITF_OB_SetRDPOEMKey() function to set the FLASH ITF OB read-out protection OEM key.
- Use the HAL_FLASH_ITF_OB_SetRDPBSKey() function to set the FLASH ITF OB read-out protection BS key.

## Enter low power modes by reset generation configuration:

- Use the HAL_FLASH_ITF_OB_SetEnterLowPWRModeRstGeneration() function to set the FLASH ITF OB enter stop mode,
  or standby mode reset generation configuration.
- Use the HAL_FLASH_ITF_OB_GetEnterLowPWRModeRstGeneration() function to get the FLASH ITF OB enter stop mode,
  or standby mode reset generation configuration.

## SRAM System reset erase configuration:

- Use the HAL_FLASH_ITF_OB_SetSystemRstSRAMErase() function to set the FLASH ITF OB system reset SRAM1 and SRAM2 erase
  configuration.
- Use the HAL_FLASH_ITF_OB_GetSystemRstSRAMErase() function to get the FLASH ITF OB system reset SRAM1 and SRAM2 erase
  configuration.

## WDG Mode configuration:

- Use the HAL_FLASH_ITF_OB_SetIWDGMode() function to set the FLASH ITF OB IWDG mode configuration.
- Use the HAL_FLASH_ITF_OB_GetIWDGMode() function to get the FLASH ITF OB IWDG mode configuration.
- Use the HAL_FLASH_ITF_OB_SetWWDGMode() function to set the FLASH ITF OB WWDG mode configuration.
- Use the HAL_FLASH_ITF_OB_GetWWDGMode() function to get the FLASH ITF OB WWDG mode configuration.

## WDG counter freeze configuration:

- Use the HAL_FLASH_ITF_OB_FreezeIWDGCounterLowPWRMode() function to freeze the FLASH ITF OB IWDG for stop and standby
  counter configuration.
- Use the HAL_FLASH_ITF_OB_UnfreezeIWDGCounterLowPWRMode() function to unfreeze the FLASH ITF OB IWDG stop and standby
  counter configuration.
- Use the HAL_FLASH_ITF_OB_IsFrozenIWDGCounterLowPWRMode() function to check the FLASH ITF OB IWDG stop and standby
  counter configuration is enabled.

## Bank swapping configuration:

- Use the HAL_FLASH_ITF_OB_SetBankSwap() function to set the FLASH ITF OB swap bank configuration.
- Use the HAL_FLASH_ITF_OB_GetBankSwap() function to get the FLASH ITF OB swap bank configuration.
- Use the HAL_FLASH_ITF_OB_IsBankSwapped() function to check the FLASH ITF OB swap bank.

## Bank configuration:

- Use the HAL_FLASH_ITF_OB_SetBankTopology() function to set the FLASH ITF OB single dual bank configuration.
- Use the HAL_FLASH_ITF_OB_GetBankTopology() function to get the FLASH ITF OB single dual bank configuration.

## SRAM ECC configuration:

- Use the HAL_FLASH_ITF_OB_EnableSRAMECC() function to enable the FLASH ITF OB for SRAM2 ECC configuration.
- Use the HAL_FLASH_ITF_OB_DisableSRAMECC() function to disable the FLASH ITF OB for SRAM2 ECC configuration.
- Use the HAL_FLASH_ITF_OB_IsEnabledSRAMECC() function to check the FLASH ITF OB for SRAM2 ECC configuration is enabled.

## Boot configuration:

- Use the HAL_FLASH_ITF_OB_SetBootSelection() function to set the FLASH ITF OB boot source selection.
- Use the HAL_FLASH_ITF_OB_GetBootSelection() function to get the FLASH ITF OB boot source selection.

- Use the HAL_FLASH_ITF_OB_SetBoot0() function to set the FLASH ITF OB boot0 configuration.
- Use the HAL_FLASH_ITF_OB_GetBoot0() function to get the FLASH ITF OB boot0 configuration.

- Use the HAL_FLASH_ITF_OB_SetBootAddr() function to set the FLASH OB boot address configuration.
- Use the HAL_FLASH_ITF_OB_GetBootAddr() function to get the FLASH OB boot address configuration.

- Use the HAL_FLASH_ITF_OB_LockBootConfig() function to lock the FLASH option bytes boot configuration.
- Use the HAL_FLASH_ITF_OB_UnlockBootConfig() function to unlock the FLASH option bytes boot configuration.
- Use the HAL_FLASH_ITF_OB_IsLockedBootConfig() function to check if the FLASH ITF OB boot configuration
  is locked or unlocked.

## Bootloader interface configuration:

- Use the HAL_FLASH_ITF_OB_SetBootloaderInterfaceConfig() function to set the bootloader interface configuration.
- Use the HAL_FLASH_ITF_OB_GetBootloaderInterfaceConfig() function to get the bootloader interface configuration.

## Option bytes programming function:

- Use the HAL_FLASH_ITF_OB_Program() function to program the option bytes in polling mode.
- Use the HAL_FLASH_ITF_OB_Program_IT() function to program the option bytes in interrupt mode.

## IRQHandler and Callback functions:

- Use the HAL_FLASH_ITF_IRQHandler() function to handle OB write/error operations.
- Use the HAL_FLASH_ITF_OB_ProgramCpltCallback() function to be redefined within application for the OB complete write
  operations callback.
- Use the HAL_FLASH_ITF_OB_ErrorCallback() function to be redefined within application for the OB write operation error
  callback.

## Privilege attributes management functions:

- Use the HAL_FLASH_ITF_SetPrivAttr() function to set the privilege attribute.
- Use the HAL_FLASH_ITF_GetPrivAttr() function to get the privilege attribute.
  */
/**
  * @}
  */

/** @defgroup FLASH_ITF_Configuration_Table FLASH ITF Configuration Table
  * @{
# Configuration inside the FLASH ITF driver

Configuration defines        | Description     | Default value   | Note                                              |
-----------------------------|-----------------|-----------------|---------------------------------------------------|
PRODUCT                      | from IDE        |        NA       | The selected device (ex STM32C5xx).               |
USE_HAL_FLASH_MODULE         | from hal_conf.h |        1        | Enables the HAL FLASH module.                     |
USE_ASSERT_DBG_PARAM         | from IDE        |      None       | Enables assert check parameters.                  |
  */
/**
  * @}
  */

/* Private Constants -------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_ITF_Private_Constants FLASH ITF Private Constants
  * @{
  */
#define FLASH_ITF_OB_TIMEOUT        1000U                                              /*!< FLASH default OB timeout */
#define FLASH_ITF_OB_LOW_PWR_MSK    (HAL_FLASH_ITF_OB_STOP_MODE      \
                                     | HAL_FLASH_ITF_OB_STANDBY_MODE)                  /*!< FLASH low power mask     */
#define FLASH_ITF_OB_SRAM_ERASE_MSK (HAL_FLASH_ITF_OB_SRAM2 | HAL_FLASH_ITF_OB_SRAM1)  /*!< FLASH SRAM erase mask    */
#define FLASH_ITF_OB_SRAM_ECC_MSK   (HAL_FLASH_ITF_OB_SRAM2)                           /*!< FLASH SRAM ecc mask      */
#define FLASH_ITF_OB_OTP_BLOCK_NBR  24U                                                /*!< FLASH OTP block number   */
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_ITF_Private_Macros FLASH ITF Private Macros
  * @{
  */

/*! Macro to get the FLASH physical instance from the HAL instance */
#define FLASH_GET_ITF_INSTANCE(instance) ((FLASH_TypeDef *)((uint32_t)(instance)))

/*! Macro to check FLASH memory bank */
#define IS_FLASH_BANK(value) (((value) == HAL_FLASH_BANK_1) || ((value) == HAL_FLASH_BANK_2))

/*! Macro to get FLASH low-layer bank */
#define FLASH_GET_ITF_BANK(value) (((value) == HAL_FLASH_BANK_1) ? LL_FLASH_BANK_1 : LL_FLASH_BANK_2)

/*! Macro to check FLASH memory latency */
#define IS_FLASH_ITF_LATENCY(value) (((value) == HAL_FLASH_ITF_LATENCY_0)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_1)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_2)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_3)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_4)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_5)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_6)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_7)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_8)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_9)  \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_10) \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_11) \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_12) \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_13) \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_14) \
                                     || ((value) == HAL_FLASH_ITF_LATENCY_15))

/*! Macro to check FLASH memory programming delay */
#define IS_FLASH_ITF_PROGRAMMING_DELAY(value) (((value) == HAL_FLASH_ITF_PROGRAM_DELAY_0)  \
                                               || ((value) == HAL_FLASH_ITF_PROGRAM_DELAY_1)  \
                                               || ((value) == HAL_FLASH_ITF_PROGRAM_DELAY_2)  \
                                               || ((value) == HAL_FLASH_ITF_PROGRAM_DELAY_3))

/*! Macro to check FLASH empty boot location status */
#define IS_FLASH_ITF_EMPTY_BOOT_LOCATION(value) (((value) == HAL_FLASH_ITF_BOOT_LOCATION_PROGRAMMED) \
                                                 || ((value) == HAL_FLASH_ITF_BOOT_LOCATION_EMPTY))


#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/*! Macro to check FLASH ECC interrupt */
#define IS_FLASH_ITF_ECC_INTERRUPT(value) ((value) == HAL_FLASH_ITF_IT_ECC_SINGLE)
#endif /* USE_HAL_FLASH_ECC */

/*! Macro to get the FLASH HDPExt bank */
#define FLASH_GET_HDPEXT_BANK(bank) (((bank) == HAL_FLASH_BANK_1) ? LL_FLASH_HDPEXT_BANK_1 : LL_FLASH_HDPEXT_BANK_2)

/*! Macro to check FLASH ITF OB Readout protection levels */
#define IS_FLASH_ITF_OB_RDP_LEVEL(value) (((value) == HAL_FLASH_ITF_OB_RDP_LEVEL_0)      \
                                          || ((value) == HAL_FLASH_ITF_OB_RDP_LEVEL_2_WBS) \
                                          || ((value) == HAL_FLASH_ITF_OB_RDP_LEVEL_2))

/*! Macro to check FLASH ITF OB low power mode */
#define IS_FLASH_ITF_OB_LOW_PWR_MODE(value) (((value) == HAL_FLASH_ITF_OB_STOP_MODE)    \
                                             || ((value) == HAL_FLASH_ITF_OB_STANDBY_MODE))

/*! Macro to check FLASH ITF OB low power reset generation */
#define IS_FLASH_ITF_OB_RST_GENERATION(value)(((value) == HAL_FLASH_ITF_OB_RST_GENERATION) \
                                              || ((value) == HAL_FLASH_ITF_OB_NO_RST_GENERATION))

/*! Macro to check FLASH ITF OB SRAM erase*/
#define IS_FLASH_ITF_OB_SRAM_ERASE(value) (((value) == HAL_FLASH_ITF_OB_SRAM2) \
                                           || ((value) == HAL_FLASH_ITF_OB_SRAM1))

/*! Macro to check FLASH ITF OB SRAM ecc*/
#define IS_FLASH_ITF_OB_SRAM_ECC(value) (((value) == HAL_FLASH_ITF_OB_SRAM2))

/*! Macro to check FLASH ITF OB system reset SRAM erase */
#define IS_FLASH_ITF_OB_SYSTEM_RST_SRAM_ERASE(value) (((value) == HAL_FLASH_ITF_OB_SYS_RST_SRAM_ERASE) \
                                                      || ((value) == HAL_FLASH_ITF_OB_SYS_RST_SRAM_NO_ERASE))

/*! Macro to check FLASH ITF OB wdg hardware/software mode */
#define IS_FLASH_ITF_OB_WDG_HW_SW_MODE(value) (((value) == HAL_FLASH_ITF_OB_WDG_HARDWARE) \
                                               || ((value) == HAL_FLASH_ITF_OB_WDG_SOFTWARE))

/*! Macro to check FLASH ITF OB single dual bank */
#define IS_FLASH_ITF_OB_SINGLE_DUAL_BANK(value) (((value) == HAL_FLASH_ITF_OB_SINGLE_BANK) \
                                                 || ((value) == HAL_FLASH_ITF_OB_DUAL_BANK))

/*! Macro to check FLASH ITF OB swap bank */
#define IS_FLASH_ITF_OB_SWAP_BANK(value) (((value) == HAL_FLASH_ITF_OB_BANK_NO_SWAP) \
                                          || ((value) == HAL_FLASH_ITF_OB_BANK_SWAP))

/*! Macro to check FLASH ITF OB boot0 source selection value */
#define IS_FLASH_ITF_OB_BOOT_SELECT(value) (((value) == HAL_FLASH_ITF_OB_BOOT_OPTION_BIT) \
                                            || ((value) == HAL_FLASH_ITF_OB_BOOT_PIN))

/*! Macro to check FLASH ITF OB boot0 state */
#define IS_FLASH_ITF_OB_BOOT_STATE(value) (((value) == HAL_FLASH_ITF_OB_BOOT_LOW) \
                                           || ((value) == HAL_FLASH_ITF_OB_BOOT_HIGH))

/*! Macro to check FLASH ITF privilege item */
#define IS_FLASH_ITF_PRIV_ITEM(value) ((value) == HAL_FLASH_ITF_PRIV_ITEM_ALL)

/*! Macro to check FLASH ITF privilege attribute */
#define IS_FLASH_ITF_PRIV_ATTR(value) (((value) == HAL_FLASH_ITF_PRIV) || ((value) == HAL_FLASH_ITF_NPRIV))

/**
  * @}
  */

/* Private Functions Prototypes --------------------------------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup FLASH_ITF_Exported_Functions
  * @{
  */

/** @addtogroup FLASH_ITF_Exported_Functions_Group1
  * @{
This subsection provides a set of functions allowing lock/unlock and configuration of the FLASH ITF peripheral:

- Call the HAL_FLASH_ITF_Lock() functions to lock the access to the FLASH control register.

- Call the HAL_FLASH_ITF_Unlock() functions to unlock the access to the FLASH control register.

- Call the HAL_FLASH_ITF_IsLocked() function to check the lock access state to the FLASH control register.

- Call the HAL_FLASH_ITF_SetLatency() function to set the FLASH latency.

- Call the HAL_FLASH_ITF_GetLatency() function to get the FLASH latency.

- Call the HAL_FLASH_ITF_SetProgrammingDelay() function to set the FLASH programming delay.

- Call the HAL_FLASH_ITF_GetProgrammingDelay() function to get the FLASH programming delay.

- Call the HAL_FLASH_ITF_EnablePrefetch() function to enable the FLASH prefetch.

- Call the HAL_FLASH_ITF_DisablePrefetch() function to disable the FLASH prefetch.

- Call the HAL_FLASH_ITF_IsEnabledPrefetch() function to check if the FLASH prefetch is enabled or disabled.

- Call the HAL_FLASH_ITF_SetEmptyBootLocation() function to set the FLASH empty boot location information.

- Call the HAL_FLASH_ITF_GetEmptyBootLocation() function to get the FLASH empty boot location information.

- Call the HAL_FLASH_ITF_SetHDPExtArea() function to set the HDP extended area.

- Call the HAL_FLASH_ITF_GetHDPExtArea() function to get the HDP extended area.

- Call the HAL_FLASH_ITF_IsLockedRDPOEMKey() function to check if the RDP OEM key is locked or unlocked.

- Call the HAL_FLASH_ITF_IsLockedRDPBSKey() function to check if the RDP BS key is locked or unlocked.

- Call the HAL_FLASH_ITF_ECC_EnableIT() function to enable the FLASH ECC interruption.

- Call the HAL_FLASH_ITF_ECC_DisableIT() function to disable the FLASH ECC interruption.

- Call the HAL_FLASH_ITF_ECC_IsEnabledIT() function to check if the FLASH ECC interruption is enabled or disabled.
 */

/**
  * @brief  Lock the FLASH control register access.
  * @param  instance  The FLASH instance.
  * @retval HAL_OK    FLASH control register access locked.
  * @retval HAL_ERROR Failed to lock FLASH control register access.
  */
hal_status_t HAL_FLASH_ITF_Lock(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  LL_FLASH_Lock(FLASH_GET_ITF_INSTANCE(instance));

  /* Verify that the control register is locked */
  if (LL_FLASH_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Unlock the FLASH control register access.
  * @param  instance  The FLASH instance.
  * @retval HAL_OK    FLASH control register access unlocked.
  * @retval HAL_ERROR Failed to unlock FLASH control register access.
  */
hal_status_t HAL_FLASH_ITF_Unlock(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
  {
    LL_FLASH_SetUnlockKey(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_KEY1);
    LL_FLASH_SetUnlockKey(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_KEY2);

    /* Verify that the control register is unlocked */
    if (LL_FLASH_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Check if the FLASH control register access is locked or unlocked.
  * @param  instance The FLASH instance.
  * @retval Returned value can be one element of @ref hal_flash_itf_lock_status_t enumeration.
  */
hal_flash_itf_lock_status_t HAL_FLASH_ITF_IsLocked(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_lock_status_t)LL_FLASH_IsLocked(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Set the FLASH latency configuration.
  * @param  instance The FLASH instance.
  * @param  latency  This parameter is one element of @ref hal_flash_itf_latency_t enumeration.
  * @retval HAL_OK   Latency is successfully configured.
  */
hal_status_t HAL_FLASH_ITF_SetLatency(hal_flash_t instance, hal_flash_itf_latency_t latency)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_LATENCY(latency));

  LL_FLASH_SetLatency(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)latency);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH latency configuration.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_latency_t Latency value.
  */
hal_flash_itf_latency_t HAL_FLASH_ITF_GetLatency(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_latency_t)LL_FLASH_GetLatency(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Set the FLASH programming delay configuration.
  * @param  instance   The FLASH instance.
  * @param  prog_delay This parameter is one element of @ref hal_flash_itf_program_delay_t enumeration.
  * @retval HAL_OK     Programming Delay is successfully configured.
  */
hal_status_t HAL_FLASH_ITF_SetProgrammingDelay(hal_flash_t instance, hal_flash_itf_program_delay_t prog_delay)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_PROGRAMMING_DELAY(prog_delay));

  LL_FLASH_SetProgrammingDelay(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)prog_delay);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH programming delay configuration.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_program_delay_t programming delay value.
  */
hal_flash_itf_program_delay_t HAL_FLASH_ITF_GetProgrammingDelay(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_program_delay_t)LL_FLASH_GetProgrammingDelay(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Enable the FLASH prefetch.
  * @param  instance The FLASH instance.
  * @retval HAL_OK   Prefetch enabled.
  */
hal_status_t HAL_FLASH_ITF_EnablePrefetch(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  LL_FLASH_EnablePrefetch(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Disable the FLASH prefetch.
  * @param  instance The FLASH instance.
  * @retval HAL_OK   Prefetch disabled.
  */
hal_status_t HAL_FLASH_ITF_DisablePrefetch(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  LL_FLASH_DisablePrefetch(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Check if the FLASH prefetch is enabled or disabled.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_prefetch_status_t Prefetch status.
  */
hal_flash_itf_prefetch_status_t HAL_FLASH_ITF_IsEnabledPrefetch(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_prefetch_status_t)LL_FLASH_IsEnabledPrefetch(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Set the FLASH empty boot location status.
  * @param  instance   The FLASH instance.
  * @param  empty_boot This parameter is one element of @ref hal_flash_itf_empty_boot_location_t enumeration.
  * @retval HAL_OK  Empty boot location status is successfully configured.
  */
hal_status_t HAL_FLASH_ITF_SetEmptyBootLocation(hal_flash_t instance, hal_flash_itf_empty_boot_location_t empty_boot)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_EMPTY_BOOT_LOCATION(empty_boot));

  LL_FLASH_SetEmptyBootLocation(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)empty_boot);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH empty boot location status.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_empty_boot_location_t Empty boot location status.
  */
hal_flash_itf_empty_boot_location_t HAL_FLASH_ITF_GetEmptyBootLocation(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_empty_boot_location_t)LL_FLASH_GetEmptyBootLocation(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Set the FLASH HDP Extended area.
  * @param  instance The FLASH instance.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @param  page_nbr This parameter can be any value between 0 and the maximum number of pages per bank.
  * @retval HAL_OK   FLASH HDP Extended area number of pages set.
  */
hal_status_t HAL_FLASH_ITF_SetHDPExtArea(hal_flash_t instance, hal_flash_bank_t bank, uint32_t page_nbr)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page_nbr < FLASH_PAGE_NB);

  LL_FLASH_SetHDPExtArea(FLASH_GET_ITF_INSTANCE(instance), FLASH_GET_HDPEXT_BANK(bank), page_nbr);

  return HAL_OK;
}

/**
  * @brief Get the FLASH HDP Extended area number of pages.
  * @param  instance The FLASH instance.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @retval uint32_t FLASH HDP Extended area number of pages value, between 0 and the maximum number of pages per bank.
  */
uint32_t HAL_FLASH_ITF_GetHDPExtArea(hal_flash_t instance, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));

  return LL_FLASH_GetHDPExtArea(FLASH_GET_ITF_INSTANCE(instance), FLASH_GET_HDPEXT_BANK(bank));
}

/**
  * @brief  Check if the FLASH Readout Protection OEM key is locked or unlocked.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_rdp_key_lock_status_t Readout Protection OEM key lock status.
  */
hal_flash_itf_rdp_key_lock_status_t HAL_FLASH_ITF_IsLockedRDPOEMKey(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_rdp_key_lock_status_t)LL_FLASH_IsActiveFlag_OEMLOCK(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Check if the FLASH Readout Protection BS key is locked or unlocked.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_rdp_key_lock_status_t Readout Protection BS key lock status.
  */
hal_flash_itf_rdp_key_lock_status_t HAL_FLASH_ITF_IsLockedRDPBSKey(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_rdp_key_lock_status_t)LL_FLASH_IsActiveFlag_BSLOCK(FLASH_GET_ITF_INSTANCE(instance)));
}


#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/**
  * @brief  Enable the given FLASH ECC interrupt.
  * @param  instance The FLASH instance.
  * @param  interrupt The ECC interrupt to enable.
  * @retval HAL_OK ECC interruption enabled.
  */
hal_status_t HAL_FLASH_ITF_ECC_EnableIT(hal_flash_t instance, uint32_t interrupt)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_ECC_INTERRUPT(interrupt));

  STM32_UNUSED(interrupt);

  LL_FLASH_EnableIT_ECCC(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Disable the given FLASH ECC interrupt.
  * @param  instance The FLASH instance.
  * @param  interrupt The ECC interrupt to disable.
  * @retval HAL_OK ECC interruption disabled.
  */
hal_status_t HAL_FLASH_ITF_ECC_DisableIT(hal_flash_t instance, uint32_t interrupt)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_ECC_INTERRUPT(interrupt));

  STM32_UNUSED(interrupt);

  LL_FLASH_DisableIT_ECCC(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Check if the given FLASH ECC interrupt is enabled or disabled.
  * @param  instance The FLASH instance.
  * @param  interrupt The ECC interrupt to check.
  * @retval Return value can be one element of @ref hal_flash_itf_ecc_it_status_t enumeration.
  */
hal_flash_itf_ecc_it_status_t HAL_FLASH_ITF_ECC_IsEnabledIT(hal_flash_t instance, uint32_t interrupt)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_ECC_INTERRUPT(interrupt));

  STM32_UNUSED(interrupt);

  return (hal_flash_itf_ecc_it_status_t)LL_FLASH_IsEnabledIT_ECCC(FLASH_GET_ITF_INSTANCE(instance));
}
#endif /* USE_HAL_FLASH_ECC */
/**
  * @}
  */

/** @addtogroup FLASH_ITF_Exported_Functions_Group2
  * @{
This subsection provides a set of functions allowing configuration of the FLASH peripheral option bytes:


- Call the HAL_FLASH_ITF_OB_Lock() function to lock the access to the FLASH option bytes registers.

- Call the HAL_FLASH_ITF_OB_Unlock() function to unlock the access to the FLASH option bytes registers.

- Call the HAL_FLASH_ITF_OB_IsLocked() function to check the lock state of the access to the FLASH option bytes

- Call the HAL_FLASH_ITF_OB_LockOTPBlock() function to lock the selected OTP blocks.

- Call the HAL_FLASH_ITF_OB_IsLockedOTPBlock() function to check if the selected OTP block is locked.

- Call the HAL_FLASH_ITF_OB_EnableEDATAArea() function to enable the EDATA area.

- Call the HAL_FLASH_ITF_OB_DisableEDATAArea() function to disable the EDATA area.

- Call the HAL_FLASH_ITF_OB_IsEnabledEDATAArea() function to check if the EDATA area is enabled or disabled.

- Call the HAL_FLASH_ITF_OB_EnablePageWRP() function to enable the FLASH ITF OB pagewise write protection area
  configuration.

- Call the HAL_FLASH_ITF_OB_DisablePageWRP() function to disable the FLASH ITF OB pagewise write protection area
  configuration.

- Call the HAL_FLASH_ITF_OB_IsEnabledPageWRP() function to check if the FLASH ITF OB pagewise write protection area
  configuration is enabled or disabled.

- Call the HAL_FLASH_ITF_OB_SetRDPLevel() function to set the FLASH ITF OB read-out protection level.

- Call the HAL_FLASH_ITF_OB_GetRDPLevel() function to get the FLASH ITF OB read-out protection level.

- Call the HAL_FLASH_ITF_OB_SetRDPOEMKey() function to set the FLASH ITF OB read-out protection OEM keys.
- Call the HAL_FLASH_ITF_OB_SetRDPBSKey() function to set the FLASH ITF OB read-out protection BS key.

- Call the HAL_FLASH_ITF_OB_SetEnterLowPWRModeRstGeneration() function to set the FLASH ITF OB enter stop mode,
  or standby mode reset generation configuration.

- Call the HAL_FLASH_ITF_OB_GetEnterLowPWRModeRstGeneration() function to get the FLASH ITF OB enter stop mode,
  or standby mode reset generation configuration.

- Call the HAL_FLASH_ITF_OB_SetSystemRstSRAMErase() function to set the FLASH ITF OB system reset SRAM1 and SRAM2 erase
  configuration.

- Call the HAL_FLASH_ITF_OB_GetSystemRstSRAMErase() function to get the FLASH ITF OB system reset SRAM1 and SRAM2 erase
  configuration.

- Call the HAL_FLASH_ITF_OB_SetIWDGMode() function to set the FLASH ITF OB IWDG mode configuration.

- Call the HAL_FLASH_ITF_OB_GetIWDGMode() function to get the FLASH ITF OB IWDG mode configuration.

- Call the HAL_FLASH_ITF_OB_SetWWDGMode() function to set the FLASH ITF OB WWDG mode configuration.

- Call the HAL_FLASH_ITF_OB_GetWWDGMode() function to get the FLASH ITF OB WWDG mode configuration.

- Call the HAL_FLASH_ITF_OB_FreezeIWDGCounterLowPWRMode() function to freeze the FLASH ITF OB IWDG for stop and standby
  counter configuration.

- Call the HAL_FLASH_ITF_OB_UnfreezeIWDGCounterLowPWRMode() function to unfreeze the FLASH ITF OB IWDG stop and standby
  counter configuration.

- Call the HAL_FLASH_ITF_OB_IsFrozenIWDGCounterLowPWRMode() function to check the FLASH ITF OB IWDG stop and standby
  counter configuration is enabled.

- Call the HAL_FLASH_ITF_OB_SetBankSwap() function to set the FLASH ITF OB swap bank configuration.

- Call the HAL_FLASH_ITF_OB_GetBankSwap() function to get the FLASH ITF OB swap bank configuration.

- Call the HAL_FLASH_ITF_OB_IsBankSwapped() function to check the FLASH ITF OB swap bank.

- Call the HAL_FLASH_ITF_OB_SetBankTopology() function to set the FLASH ITF OB single dual bank configuration.

- Call the HAL_FLASH_ITF_OB_GetBankTopology() function to get the FLASH ITF OB single dual bank configuration.

- Call the HAL_FLASH_ITF_OB_EnableSRAMECC() function to enable the FLASH ITF OB for SRAM2 ECC configuration.

- Call the HAL_FLASH_ITF_OB_DisableSRAMECC() function to disable the FLASH ITF OB for SRAM2 ECC configuration.

- Call the HAL_FLASH_ITF_OB_IsEnabledSRAMECC() function to check the FLASH ITF OB for SRAM2 ECC configuration
  is enabled.

- Call the HAL_FLASH_ITF_OB_SetBootSelection() function to set the FLASH ITF OB boot source selection.

- Call the HAL_FLASH_ITF_OB_GetBootSelection() function to get the FLASH ITF OB boot source selection.

- Call the HAL_FLASH_ITF_OB_SetBoot0() function to set the FLASH ITF OB boot0 configuration.

- Call the HAL_FLASH_ITF_OB_GetBoot0() function to get the FLASH ITF OB boot0 configuration.

- Call the HAL_FLASH_ITF_OB_SetHDPArea() function to set the FLASH ITF OB hide protection area configuration.

- Call the HAL_FLASH_ITF_OB_GetHDPArea() function to get the FLASH ITF OB hide protection area configuration.

- Call the HAL_FLASH_ITF_OB_SetBootAddr() function to set the FLASH OB boot address configuration.

- Call the HAL_FLASH_ITF_OB_GetBootAddr() function to get the FLASH OB boot address configuration.

- Call the HAL_FLASH_ITF_OB_LockBootConfig() function to lock the FLASH option bytes boot configuration.

- Call the HAL_FLASH_ITF_OB_UnlockBootConfig() function to unlock the FLASH option bytes boot configuration.

- Call the HAL_FLASH_ITF_OB_IsLockedBootConfig() function to Check if the FLASH ITF OB boot configuration
  is locked or unlocked.

- Call the HAL_FLASH_ITF_OB_SetBootloaderInterfaceConfig() function to set the bootloader interface configuration.

- Call the HAL_FLASH_ITF_OB_GetBootloaderInterfaceConfig() function to get the bootloader interface configuration.
  */

/**
  * @brief  Lock the  FLASH OB control register access.
  * @param  instance  The FLASH instance.
  * @retval HAL_OK    FLASH OB control register access is successfully locked.
  * @retval HAL_ERROR Failed to lock FLASH OB control register access.
  */
hal_status_t HAL_FLASH_ITF_OB_Lock(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  LL_FLASH_OB_Lock(FLASH_GET_ITF_INSTANCE(instance));

  /* Verify that the Option bytes are locked */
  if (LL_FLASH_OB_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Unlock the FLASH OB control register access.
  * @param  instance  The FLASH instance.
  * @retval HAL_OK    FLASH OB control register access is successfully unlocked.
  * @retval HAL_ERROR Failed to unlock FLASH OB control register access
  */
hal_status_t HAL_FLASH_ITF_OB_Unlock(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_OB_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
  {
    LL_FLASH_OB_SetUnlockKey(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_OB_OPTKEY1);
    LL_FLASH_OB_SetUnlockKey(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_OB_OPTKEY2);

    /* Verify that the Option bytes are unlocked */
    if (LL_FLASH_OB_IsLocked(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Check if the FLASH OB control register access is locked or unlocked.
  * @param  instance  The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_lock_status_t enumeration.
  */
hal_flash_itf_ob_lock_status_t HAL_FLASH_ITF_OB_IsLocked(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_ob_lock_status_t)LL_FLASH_OB_IsLocked(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Lock the selected blocks of the OTP flash area.
  * @param  instance  The FLASH instance.
  * @param  start_otp_block  Start OTP block.
  *                          This parameter can take any value from 0 to 23.
  * @param  otp_block_nbr    OTP block number.
  *                          This parameter can take any value from 1 to 24.
  * @note   To configure the locked OTP blocks, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB OTP blocks are successfully locked.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_LockOTPBlock(hal_flash_t instance, uint32_t start_otp_block, uint32_t otp_block_nbr)
{
  uint32_t otp_blk_mask = 0U;

  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(start_otp_block < FLASH_ITF_OB_OTP_BLOCK_NBR);
  ASSERT_DBG_PARAM((otp_block_nbr != 0U) && (otp_block_nbr <= FLASH_ITF_OB_OTP_BLOCK_NBR));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  /* Check that the OTP blocks range is valid */
  if ((start_otp_block + otp_block_nbr) > FLASH_ITF_OB_OTP_BLOCK_NBR)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  /* Repeat for page number to be protected */
  for (uint32_t count = 0U; count < otp_block_nbr; count++)
  {
    otp_blk_mask |= (1UL << (start_otp_block + count));
  }

  LL_FLASH_OB_LockOTPBlock(FLASH_GET_ITF_INSTANCE(instance), otp_blk_mask);

  return HAL_OK;
}

/**
  * @brief  Check if the selected OTP blocks are locked.
  * @param  instance The FLASH instance.
  * @param  otp_block OTP block to check.
  *                   This parameter can take any value from 0 to 23.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_otp_blk_lock_status_t enumeration.
  */
hal_flash_itf_ob_otp_blk_lock_status_t HAL_FLASH_ITF_OB_IsLockedOTPBlock(hal_flash_t instance, uint32_t otp_block)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_otp_blk_lock_status_t)LL_FLASH_OB_IsLockedOTPBlock(FLASH_GET_ITF_INSTANCE(instance), \
                                                                              (1UL << otp_block));
}


/**
  * @brief  Enable the write protection feature on the selected pages of the given bank.
  * @param  instance   The FLASH instance.
  * @param  bank       This parameter can be one of the following values:
  *                        @arg @ref HAL_FLASH_BANK_1
  *                        @arg @ref HAL_FLASH_BANK_2
  * @param  start_page Starting page number for enabling write protection.
  *                    This parameter can be any value between 0 and (FLASH_PAGE_NB - 1).
  * @param  page_nbr   Number of pages to be write protected.
  *                    This parameter can be any value between 1 and FLASH_PAGE_NB.
  * @note   To configure the write protected pages, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB write protection area is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_EnablePageWRP(hal_flash_t instance,
                                            hal_flash_bank_t bank,
                                            uint32_t start_page,
                                            uint32_t page_nbr)
{
  uint32_t page_mask = 0U;

  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(start_page < FLASH_PAGE_NB);
  ASSERT_DBG_PARAM((page_nbr != 0U) && (page_nbr <= FLASH_PAGE_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((start_page + page_nbr) > FLASH_PAGE_NB)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  /* Repeat for page number to be protected */
  for (uint32_t count = 0U; count < page_nbr; count++)
  {
    page_mask |= (1UL << ((start_page + count) / FLASH_WRP_GROUP_WIDTH));
  }

  LL_FLASH_OB_EnablePageWRP(FLASH_GET_ITF_INSTANCE(instance), (uint32_t) bank, page_mask);

  return HAL_OK;
}

/**
  * @brief  Disable the write protection feature on the selected pages of the given bank.
  * @param  instance   The FLASH instance.
  * @param  bank       This parameter can be one of the following values:
  *                        @arg @ref HAL_FLASH_BANK_1
  *                        @arg @ref HAL_FLASH_BANK_2
  * @param  start_page Starting page number for disabling write protection.
  *                    This parameter can be any value between 0 and (FLASH_PAGE_NB - 1).
  * @param  page_nbr   Number of pages for disabling write protection.
  *                    This parameter can be any value between 1 and FLASH_PAGE_NB.
  * @note   To configure the write protected pages, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB write protection area is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_DisablePageWRP(hal_flash_t instance,
                                             hal_flash_bank_t bank,
                                             uint32_t start_page,
                                             uint32_t page_nbr)
{
  uint32_t page_mask = 0U;

  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(start_page < FLASH_PAGE_NB);
  ASSERT_DBG_PARAM((page_nbr != 0U) && (page_nbr <= FLASH_PAGE_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((start_page + page_nbr) > FLASH_PAGE_NB)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  /* Repeat for page number to be protected */
  for (uint32_t count = 0U; count < page_nbr; count++)
  {
    page_mask |= (1UL << ((start_page + count) / FLASH_WRP_GROUP_WIDTH));
  }

  LL_FLASH_OB_DisablePageWRP(FLASH_GET_ITF_INSTANCE(instance), (uint32_t) bank, page_mask);

  return HAL_OK;
}

/**
  * @brief  Get the state  of page write protection configuration for the selected pages of the given bank.
  * @param  instance The FLASH instance.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @param  page     This parameter can be any value between 0 and FLASH_PAGE_NB.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_wrp_page_status_t enumeration.
  */
hal_flash_itf_ob_wrp_page_status_t HAL_FLASH_ITF_OB_IsEnabledPageWRP(hal_flash_t instance,
                                                                     hal_flash_bank_t bank,
                                                                     uint32_t page)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));

  return (hal_flash_itf_ob_wrp_page_status_t)LL_FLASH_OB_IsEnabledPageWRP(FLASH_GET_ITF_INSTANCE(instance), \
                                                                          (uint32_t)FLASH_GET_ITF_BANK(bank), \
                                                                          (1UL << (page / FLASH_WRP_GROUP_WIDTH)));
}

/**
  * @brief  Enable the data flash area.
  * @param  instance  The FLASH instance.
  * @note   Before enabling the data flash area, the option lock bit OPTLOCK must
  *         be previously cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB data FLASH is successfully enabled.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_EnableEDATAArea(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_EnableEDATAArea(FLASH_GET_ITF_INSTANCE(instance));
  return HAL_OK;
}

/**
  * @brief  Disable the data flash area.
  * @param  instance  The FLASH instance.
  * @note   To disable the data flash area, the option lock bit OPTLOCK mustbe cleared
  *         with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB data FLASH is successfully disabled.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_DisableEDATAArea(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_DisableEDATAArea(FLASH_GET_ITF_INSTANCE(instance));
  return HAL_OK;
}

/**
  * @brief  Check if the data flash area of the selected bank is enabled or not.
  * @param  instance The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_edata_area_status_t enumeration.
  */
hal_flash_itf_ob_edata_area_status_t HAL_FLASH_ITF_OB_IsEnabledEDATAArea(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_edata_area_status_t)LL_FLASH_OB_IsEnabledEDATAArea(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the HDP area.
  * @param  instance   The FLASH instance.
  * @param  bank       This parameter can be one of the following values:
  *                        @arg @ref HAL_FLASH_BANK_1
  *                        @arg @ref HAL_FLASH_BANK_2
  * @param  start_page This parameter can take any value between 0 and the maximum number of pages.
  * @param  page_nbr   Number of page of the hide protection area.
  * @note   To configure the HDP area start page, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK     FLASH OB HDP area start page is successfully configured.
  * @retval HAL_ERROR  FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetHDPArea(hal_flash_t instance,
                                         hal_flash_bank_t bank,
                                         uint32_t start_page,
                                         uint32_t page_nbr)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(start_page < FLASH_PAGE_NB);

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_ConfigHDPArea(FLASH_GET_ITF_INSTANCE(instance), \
                            (uint32_t)FLASH_GET_ITF_BANK(bank), start_page, start_page + page_nbr - 1U);
  return HAL_OK;
}

/**
  * @brief  Get the hide protection area configuration.
  * @param  instance     The FLASH instance.
  * @param  bank         This parameter can be one of the following values:
  *                          @arg @ref HAL_FLASH_BANK_1
  *                          @arg @ref HAL_FLASH_BANK_2
  * @param  p_start_page Pointer to start page of the hide protection area.
  * @param  p_page_nbr   Pointer to number of page of the hide protection area.
  */
void HAL_FLASH_ITF_OB_GetHDPArea(hal_flash_t instance, hal_flash_bank_t bank,
                                 uint32_t *p_start_page, uint32_t *p_page_nbr)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(p_start_page != NULL);
  ASSERT_DBG_PARAM(p_page_nbr != NULL);

  *p_start_page = LL_FLASH_OB_GetHDPAreaStartPage(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)FLASH_GET_ITF_BANK(bank));
  *p_page_nbr   = (LL_FLASH_OB_GetHDPAreaEndPage(FLASH_GET_ITF_INSTANCE(instance), \
                                                 (uint32_t)FLASH_GET_ITF_BANK(bank)) - *p_start_page + 1U);
}

/**
  * @brief  Set the RDP level.
  * @param  instance  The FLASH instance.
  * @param  rdp_level Element from the @ref hal_flash_itf_ob_rdp_level_t enumeration.
  * @note   To configure the RDP level, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB RDP level is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetRDPLevel(hal_flash_t instance, hal_flash_itf_ob_rdp_level_t rdp_level)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_RDP_LEVEL(rdp_level));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetRDPLevel(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)rdp_level);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB RDP level configuration.
  * @param  instance The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_rdp_level_t enumeration.
  */
hal_flash_itf_ob_rdp_level_t HAL_FLASH_ITF_OB_GetRDPLevel(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_rdp_level_t)LL_FLASH_OB_GetRDPLevel(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the FLASH OB OEM key configuration.
  * @param  instance  The FLASH instance.
  * @param  p_key     Pointer to the key value.
  * @note   To configure the OEM key, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB OEM key is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetRDPOEMKey(hal_flash_t instance, const hal_flash_itf_ob_rdp_oem_key_t *p_key)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetOEMKeyWord1(FLASH_GET_ITF_INSTANCE(instance), p_key->key_w1);
  LL_FLASH_OB_SetOEMKeyWord2(FLASH_GET_ITF_INSTANCE(instance), p_key->key_w2);
  LL_FLASH_OB_SetOEMKeyWord3(FLASH_GET_ITF_INSTANCE(instance), p_key->key_w3);
  LL_FLASH_OB_SetOEMKeyWord4(FLASH_GET_ITF_INSTANCE(instance), p_key->key_w4);

  return HAL_OK;
}

/**
  * @brief  Set the FLASH OB BS key configuration.
  * @param  instance  The FLASH instance.
  * @param  p_key     Pointer to the key value.
  * @note   To configure the BS key, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB BS key is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetRDPBSKey(hal_flash_t instance, const hal_flash_itf_ob_rdp_bs_key_t *p_key)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBSKey(FLASH_GET_ITF_INSTANCE(instance), p_key->key_w1);

  return HAL_OK;
}


/**
  * @brief  Set the FLASH OB enter stop mode reset generation configuration.
  * @param  instance  The FLASH instance.
  * @param  low_pwr_mode
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_STOP_MODE
  *         @arg @ref HAL_FLASH_ITF_OB_STANDBY_MODE
  * @param  rst_gen   Element from the @ref hal_flash_itf_ob_rst_generation_status_t enumeration.
  * @note   To configure the reset generation upon entering stop mode, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB enter low-power mode reset generation is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetEnterLowPWRModeRstGeneration(hal_flash_t instance, uint32_t low_pwr_mode,
                                                              hal_flash_itf_ob_rst_generation_status_t rst_gen)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((low_pwr_mode & FLASH_ITF_OB_LOW_PWR_MSK) == low_pwr_mode);
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_RST_GENERATION(rst_gen));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STOP_MODE) == HAL_FLASH_ITF_OB_STOP_MODE)
  {
    LL_FLASH_OB_SetNRSTStopMode(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)rst_gen * LL_FLASH_OB_NO_RST_STOP_MODE);
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STANDBY_MODE) == HAL_FLASH_ITF_OB_STANDBY_MODE)
  {
    LL_FLASH_OB_SetNRSTStandbyMode(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)rst_gen * LL_FLASH_OB_NO_RST_STDBY_MODE);
  }

  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB enter stop mode reset generation configuration.
  * @param  instance  The FLASH instance.
  * @param  low_pwr_mode
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_STOP_MODE
  *         @arg @ref HAL_FLASH_ITF_OB_STANDBY_MODE
  * @retval Return value can be one element of @ref hal_flash_itf_ob_rst_generation_status_t enumeration.
  */
hal_flash_itf_ob_rst_generation_status_t HAL_FLASH_ITF_OB_GetEnterLowPWRModeRstGeneration(hal_flash_t instance,
    uint32_t low_pwr_mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_LOW_PWR_MODE(low_pwr_mode));

  if (low_pwr_mode == HAL_FLASH_ITF_OB_STOP_MODE)
  {
    return (hal_flash_itf_ob_rst_generation_status_t)
           ((uint32_t)(LL_FLASH_OB_GetNRSTStopMode(FLASH_GET_ITF_INSTANCE(instance)) >> FLASH_OPTSR_PRG_NRST_STOP_Pos));
  }
  else
  {
    return (hal_flash_itf_ob_rst_generation_status_t)
           ((uint32_t)(LL_FLASH_OB_GetNRSTStandbyMode(FLASH_GET_ITF_INSTANCE(instance)) \
                       >> FLASH_OPTSR_PRG_NRST_STDBY_Pos));
  }
}

/**
  * @brief  Set the FLASH OB system reset SRAM erase configuration.
  * @param  instance   The FLASH instance.
  * @param  sram
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM2
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM1
  * @param  sram_erase Element from the @ref hal_flash_itf_ob_sys_rst_sram_erase_t enumeration.
  * @note   To configure the SRAM erase upon system reset, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK     FLASH OB erase SRAM on system reset is successfully configured.
  * @retval HAL_ERROR  FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetSystemRstSRAMErase(hal_flash_t instance,
                                                    uint32_t sram,
                                                    hal_flash_itf_ob_sys_rst_sram_erase_t sram_erase)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((sram & FLASH_ITF_OB_SRAM_ERASE_MSK) == sram);
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_SYSTEM_RST_SRAM_ERASE(sram_erase));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((sram & HAL_FLASH_ITF_OB_SRAM2) == HAL_FLASH_ITF_OB_SRAM2)
  {
    LL_FLASH_OB_SetSystemResetSRAM2Erase(FLASH_GET_ITF_INSTANCE(instance), \
                                         (uint32_t)sram_erase * LL_FLASH_OB_NOT_ERASED_SRAM2_SYS_RST);
  }

  if ((sram & HAL_FLASH_ITF_OB_SRAM1) == HAL_FLASH_ITF_OB_SRAM1)
  {
    LL_FLASH_OB_SetSystemResetSRAM1Erase(FLASH_GET_ITF_INSTANCE(instance), \
                                         (uint32_t)sram_erase * LL_FLASH_OB_NOT_ERASED_SRAM1_SYS_RST);
  }

  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB system reset SRAM erase configuration.
  * @param  instance The FLASH instance.
  * @param  sram
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM2
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM1
  * @retval Return value can be one element of @ref hal_flash_itf_ob_sys_rst_sram_erase_t enumeration.
  */
hal_flash_itf_ob_sys_rst_sram_erase_t HAL_FLASH_ITF_OB_GetSystemRstSRAMErase(hal_flash_t instance, uint32_t sram)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_SRAM_ERASE(sram));

  if (sram == HAL_FLASH_ITF_OB_SRAM2)
  {
    return (hal_flash_itf_ob_sys_rst_sram_erase_t)
           ((uint32_t)(LL_FLASH_OB_GetSystemResetSRAM2Erase(FLASH_GET_ITF_INSTANCE(instance)) \
                       >> FLASH_OPTSR2_PRG_SRAM2_RST_Pos));
  }
  else
  {
    return (hal_flash_itf_ob_sys_rst_sram_erase_t)
           ((uint32_t)(LL_FLASH_OB_GetSystemResetSRAM1Erase(FLASH_GET_ITF_INSTANCE(instance)) \
                       >> FLASH_OPTSR2_PRG_SRAM1_RST_Pos));
  }
}

/**
  * @brief  Set the FLASH OB IWDG mode configuration.
  * @param  instance  The FLASH instance.
  * @param  mode      Element from the @ref hal_flash_itf_ob_wdg_mode_t enumeration.
  * @note   To configure the IWDG control mode, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB IWDG mode is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetIWDGMode(hal_flash_t instance, hal_flash_itf_ob_wdg_mode_t mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_WDG_HW_SW_MODE(mode));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetIWDGSelection(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)mode * LL_FLASH_OB_IWDG_SW);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB IWDG mode configuration.
  * @param  instance The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_wdg_mode_t enumeration.
  */
hal_flash_itf_ob_wdg_mode_t HAL_FLASH_ITF_OB_GetIWDGMode(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_wdg_mode_t)((uint32_t)(LL_FLASH_OB_GetIWDGSelection(FLASH_GET_ITF_INSTANCE(instance)) \
                                                  >> FLASH_OPTSR_PRG_IWDG_SW_Pos));
}

/**
  * @brief  Set the FLASH OB WWDG mode configuration.
  * @param  instance  The FLASH instance.
  * @param  mode      Element from the @ref hal_flash_itf_ob_wdg_mode_t enumeration.
  * @note   To configure the WWDG control mode, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB WWDG mode is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetWWDGMode(hal_flash_t instance, hal_flash_itf_ob_wdg_mode_t mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_WDG_HW_SW_MODE(mode));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetWWDGSelection(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)mode * LL_FLASH_OB_WWDG_SW);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB WWDG mode configuration.
  * @param  instance The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_wdg_mode_t enumeration.
  */
hal_flash_itf_ob_wdg_mode_t HAL_FLASH_ITF_OB_GetWWDGMode(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_wdg_mode_t)((uint32_t)(LL_FLASH_OB_GetWWDGSelection(FLASH_GET_ITF_INSTANCE(instance)) \
                                                  >> FLASH_OPTSR_PRG_WWDG_SW_Pos));
}

/**
  * @brief  Freeze the FLASH OB IWDG counter in low power mode configuration.
  * @param  instance  The FLASH instance.
  * @param  low_pwr_mode
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_STOP_MODE
  *         @arg @ref HAL_FLASH_ITF_OB_STANDBY_MODE
  * @note   To configure the IWDG freeze in low-power mode, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB IWDG counter is frozen in specified low-power mode.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_FreezeIWDGCounterLowPWRMode(hal_flash_t instance, uint32_t low_pwr_mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((low_pwr_mode & FLASH_ITF_OB_LOW_PWR_MSK) == low_pwr_mode);

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STOP_MODE) == HAL_FLASH_ITF_OB_STOP_MODE)
  {
    LL_FLASH_OB_FreezeIWDGStopMode(FLASH_GET_ITF_INSTANCE(instance));
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STANDBY_MODE) == HAL_FLASH_ITF_OB_STANDBY_MODE)
  {
    LL_FLASH_OB_FreezeIWDGStandbyMode(FLASH_GET_ITF_INSTANCE(instance));
  }

  return HAL_OK;
}

/**
  * @brief  Unfreeze the FLASH OB IWDG counter in low-power mode configuration.
  * @param  instance  The FLASH instance.
  * @param  low_pwr_mode
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_STOP_MODE
  *         @arg @ref HAL_FLASH_ITF_OB_STANDBY_MODE
  * @note   To configure the IWDG freeze in low-power mode, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB IWDG counter is not frozen in specified low-power mode.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_UnfreezeIWDGCounterLowPWRMode(hal_flash_t instance, uint32_t low_pwr_mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((low_pwr_mode & FLASH_ITF_OB_LOW_PWR_MSK) == low_pwr_mode);

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STOP_MODE) == HAL_FLASH_ITF_OB_STOP_MODE)
  {
    LL_FLASH_OB_UnfreezeIWDGStopMode(FLASH_GET_ITF_INSTANCE(instance));
  }

  if ((low_pwr_mode & HAL_FLASH_ITF_OB_STANDBY_MODE) == HAL_FLASH_ITF_OB_STANDBY_MODE)
  {
    LL_FLASH_OB_UnfreezeIWDGStandbyMode(FLASH_GET_ITF_INSTANCE(instance));
  }

  return HAL_OK;
}

/**
  * @brief  Check the FLASH OB freeze IWDG counter in low-power mode configuration.
  * @param  instance The FLASH instance.
  * @param  low_pwr_mode
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_STOP_MODE
  *         @arg @ref HAL_FLASH_ITF_OB_STANDBY_MODE
  * @retval Return value can be one element of @ref hal_flash_itf_ob_wdg_freeze_status_t enumeration.
  */
hal_flash_itf_ob_wdg_freeze_status_t HAL_FLASH_ITF_OB_IsFrozenIWDGCounterLowPWRMode(hal_flash_t instance,
    uint32_t low_pwr_mode)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_LOW_PWR_MODE(low_pwr_mode));

  if (low_pwr_mode == HAL_FLASH_ITF_OB_STOP_MODE)
  {
    return (hal_flash_itf_ob_wdg_freeze_status_t)LL_FLASH_OB_IsFrozenIWDGStopMode(FLASH_GET_ITF_INSTANCE(instance));
  }
  else
  {
    return (hal_flash_itf_ob_wdg_freeze_status_t)LL_FLASH_OB_IsFrozenIWDGStandbyMode(FLASH_GET_ITF_INSTANCE(instance));
  }
}

/**
  * @brief  Set the FLASH ITF OB boot0 configuration.
  * @param  instance    The FLASH instance.
  * @param  boot_select Element from @ref hal_flash_itf_ob_boot_selection_t enumeration.
  * @retval HAL_OK      FLASH ITF OB boot0 is successfully configured.
  * @retval HAL_ERROR   FLASH ITF OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBootSelection(hal_flash_t instance, hal_flash_itf_ob_boot_selection_t boot_select)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_BOOT_SELECT(boot_select));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBoot0SourceSelection(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)boot_select);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH ITF OB boot0 source selection configuration.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_ob_boot_selection_t FLASH ITF OB boot0 source selected.
  */
hal_flash_itf_ob_boot_selection_t HAL_FLASH_ITF_OB_GetBootSelection(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_boot_selection_t)LL_FLASH_OB_GetBoot0SourceSelection(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the FLASH ITF OB boot0 option bit state configuration.
  * @param  instance  The FLASH instance.
  * @param  state     Element from @ref hal_flash_itf_ob_boot_state_t enumeration.
  * @retval HAL_OK    FLASH ITF OB boot0 is successfully configured.
  * @retval HAL_ERROR FLASH ITF OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBoot0(hal_flash_t instance, hal_flash_itf_ob_boot_state_t state)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_BOOT_STATE(state));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBoot0(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)state);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH ITF OB boot0 option bit state configuration.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_ob_boot_selection_t FLASH ITF OB boot0 option bit state configuration.
  */
hal_flash_itf_ob_boot_state_t HAL_FLASH_ITF_OB_GetBoot0(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_boot_state_t)LL_FLASH_OB_GetBoot0(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the FLASH ITF OB single/dual bank configuration for products with less user memory.
  * @param  instance      The FLASH instance.
  * @param  bank_topology This parameter is one element of @ref hal_flash_itf_ob_topology_t  enumeration.
  * @note   Before setting the bank topology, the option lock bit OPTLOCK must
  *         be previously cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB single-bank option bit value is successfully enabled.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBankTopology(hal_flash_t instance, hal_flash_itf_ob_topology_t bank_topology)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_SINGLE_DUAL_BANK(bank_topology));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBank(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)bank_topology);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH ITF OB single/dual bank configuration.
  * @param  instance The FLASH instance.
  * @retval hal_flash_itf_ob_topology_t FLASH ITF OB single/dual bank configuration.
  */
hal_flash_itf_ob_topology_t HAL_FLASH_ITF_OB_GetBankTopology(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_topology_t)LL_FLASH_OB_GetBank(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the FLASH OB bank swapping configuration.
  * @param  instance  The FLASH instance.
  * @param  bank_swap This parameter is one element of @ref hal_flash_itf_ob_bank_swap_t enumeration.
  * @note   To configure the bank swapping, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB bank swapping is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBankSwap(hal_flash_t instance, hal_flash_itf_ob_bank_swap_t bank_swap)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_SWAP_BANK(bank_swap));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetSwapBank(FLASH_GET_ITF_INSTANCE(instance), (uint32_t)bank_swap);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB bank swapping configuration.
  * @param  instance The FLASH instance.
  * @note   This function returns the programmed option byte swap bank setting.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_bank_swap_t enumeration.
  */
hal_flash_itf_ob_bank_swap_t HAL_FLASH_ITF_OB_GetBankSwap(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_ob_bank_swap_t)LL_FLASH_OB_GetSwapBank(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Check the FLASH OB bank swapping state.
  * @param  instance The FLASH instance.
  * @note   This function returns the current effective bank swapping status.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_bank_swap_status_t enumeration.
  */
hal_flash_itf_ob_bank_swap_status_t HAL_FLASH_ITF_OB_IsBankSwapped(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return ((hal_flash_itf_ob_bank_swap_status_t)LL_FLASH_OB_IsBankSwapped(FLASH_GET_ITF_INSTANCE(instance)));
}

/**
  * @brief  Enable the FLASH OB SRAM ECC configuration.
  * @param  instance  The FLASH instance.
  * @param  sram
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM2
  * @note   To configure the SRAM ECC feature, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB SRAM ECC is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_EnableSRAMECC(hal_flash_t instance, uint32_t sram)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((sram & FLASH_ITF_OB_SRAM_ECC_MSK) == sram);

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((sram & HAL_FLASH_ITF_OB_SRAM2) == HAL_FLASH_ITF_OB_SRAM2)
  {
    LL_FLASH_OB_EnableECCSRAM2(FLASH_GET_ITF_INSTANCE(instance));
  }

  return HAL_OK;
}

/**
  * @brief  Disable the FLASH OB SRAM ECC configuration.
  * @param  instance  The FLASH instance.
  * @param  sram
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM2
  * @note   To configure the SRAM ECC feature, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB SRAM ECC is successfully configured.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_DisableSRAMECC(hal_flash_t instance, uint32_t sram)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM((sram & FLASH_ITF_OB_SRAM_ECC_MSK) == sram);

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  if ((sram & HAL_FLASH_ITF_OB_SRAM2) == HAL_FLASH_ITF_OB_SRAM2)
  {
    LL_FLASH_OB_DisableECCSRAM2(FLASH_GET_ITF_INSTANCE(instance));
  }

  return HAL_OK;
}

/**
  * @brief  Check the FLASH OB SRAM2 ECC configuration is enabled or disabled.
  * @param  instance The FLASH instance.
  * @param  sram
  *         This parameter can take one of the following values:
  *         @arg @ref HAL_FLASH_ITF_OB_SRAM2
  * @retval Return value can be one element of @ref hal_flash_itf_ob_sram_ecc_status_t enumeration.
  */
hal_flash_itf_ob_sram_ecc_status_t HAL_FLASH_ITF_OB_IsEnabledSRAMECC(hal_flash_t instance, uint32_t sram)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_OB_SRAM_ECC(sram));
  STM32_UNUSED(sram);
  return (hal_flash_itf_ob_sram_ecc_status_t)LL_FLASH_OB_IsEnabledECCSRAM2(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the FLASH OB unique boot address configuration.
  * @param  instance  The FLASH instance.
  * @param  boot_addr Boot address to be configured.
  * @note   To configure the boot address, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK            FLASH OB boot address is successfully configured.
  * @retval HAL_ERROR         FLASH OB write error generated.
  * @retval HAL_INVALID_PARAM FLASH OB boot address out of bounds.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBootAddr(hal_flash_t instance, uint32_t boot_addr)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if ((boot_addr < FLASH_BASE) || (boot_addr > (FLASH_BASE + FLASH_SIZE)))
  {
    return HAL_INVALID_PARAM;
  }

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBootAddr(FLASH_GET_ITF_INSTANCE(instance), boot_addr);
  return HAL_OK;
}

/**
  * @brief  Get the FLASH OB unique boot address 0 configuration.
  * @param  instance The FLASH instance.
  * @retval Return the configured boot address.
  */
uint32_t HAL_FLASH_ITF_OB_GetBootAddr(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return LL_FLASH_OB_GetBootAddr(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Lock FLASH OB unique boot address.
  * @param  instance  The FLASH instance.
  * @note   To configure the boot address lock, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB boot address is successfully locked.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_LockBootConfig(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_LockBootConfig(FLASH_GET_ITF_INSTANCE(instance));
  return HAL_OK;
}

/**
  * @brief  Unlock FLASH OB unique boot address.
  * @param  instance  The FLASH instance.
  * @note   To configure the boot address lock, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK    FLASH OB boot address is successfully unlocked.
  * @retval HAL_ERROR FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_UnlockBootConfig(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_UnlockBootConfig(FLASH_GET_ITF_INSTANCE(instance));
  return HAL_OK;
}

/**
  * @brief  Check if the FLASH OB unique boot address is locked or not.
  * @param  instance The FLASH instance.
  * @retval Return value can be one element of @ref hal_flash_itf_ob_boot_lock_status_t enumeration.
  */
hal_flash_itf_ob_boot_lock_status_t HAL_FLASH_ITF_OB_IsLockedBootConfig(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return (hal_flash_itf_ob_boot_lock_status_t)LL_FLASH_OB_IsLockedBootConfig(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @brief  Set the bootloader interface configuration to the specified value.
  * @param  instance          The FLASH instance.
  * @param  bootloader_config This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
  * @note   To configure the bootloader interface, the option lock bit OPTLOCK must be
  *         cleared with the call of the HAL_FLASH_ITF_OB_Unlock() function.
  * @retval HAL_OK            FLASH OB bootloader configuration is successfully configured.
  * @retval HAL_ERROR         FLASH OB write error generated.
  */
hal_status_t HAL_FLASH_ITF_OB_SetBootloaderInterfaceConfig(hal_flash_t instance, uint32_t bootloader_config)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  if (LL_FLASH_IsActiveFlag(FLASH_GET_ITF_INSTANCE(instance), \
                            (LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL)) != 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_OB_SetBootloaderInterfaceConfig(FLASH_GET_ITF_INSTANCE(instance), bootloader_config);
  return HAL_OK;
}

/**
  * @brief  Get the bootloader interface configuration.
  * @param  instance The FLASH instance.
  * @retval Returned value can be any value between 0x00000000 and 0xFFFFFFFF.
  */
uint32_t HAL_FLASH_ITF_OB_GetBootloaderInterfaceConfig(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  return  LL_FLASH_OB_GetBootloaderInterfaceConfig(FLASH_GET_ITF_INSTANCE(instance));
}

/**
  * @}
  */

/** @addtogroup FLASH_ITF_Exported_Functions_Group3
  * @{
This subsection provides a set of functions allowing handling of the FLASH ITF interrupt subroutines:

- Call the function HAL_FLASH_ITF_IRQHandler() handle any enabled OB write interrupt and call its corresponding
  callback.

- The function HAL_FLASH_ITF_OB_ProgramCpltCallback() to be redefined within application for the OB complete write
  operation callback.

- The function HAL_FLASH_ITF_OB_ErrorCallback() to be redefined within application the OB write error operation
  callback.
  */

/**
  * @brief  Handle the FLASH ITF interrupt request.
  * @param  instance The FLASH instance.
  */
void HAL_FLASH_ITF_IRQHandler(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  uint32_t flags = LL_FLASH_READ_REG(FLASH_GET_ITF_INSTANCE(instance), SR);

  if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
  {
    LL_FLASH_ClearFlag(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_FLAG_EOP);
    LL_FLASH_DisableIT(FLASH_GET_ITF_INSTANCE(instance), (LL_FLASH_IT_EOP | LL_FLASH_IT_OPTCHANGEERR));
    HAL_FLASH_ITF_OB_ProgramCpltCallback(instance);
  }

  if (STM32_READ_BIT(flags, LL_FLASH_FLAG_OPTCHANGEERR) != 0U)
  {
    LL_FLASH_ClearFlag(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_FLAG_OPTCHANGEERR);
    LL_FLASH_DisableIT(FLASH_GET_ITF_INSTANCE(instance), (LL_FLASH_IT_EOP | LL_FLASH_IT_OPTCHANGEERR));
    HAL_FLASH_ITF_OB_ErrorCallback(instance);
  }
}

/**
  * @brief  FLASH ITF option bytes write complete callback.
  * @param  instance The FLASH instance.
  */
__WEAK void HAL_FLASH_ITF_OB_ProgramCpltCallback(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(instance);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ITF_OB_ProgramCpltCallback() could be implemented in the user file */
}

/**
  * @brief  FLASH ITF option bytes write error callback.
  * @param  instance The FLASH instance.
  */
__WEAK void HAL_FLASH_ITF_OB_ErrorCallback(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(instance);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ITF_OB_ErrorCallback() could be implemented in the user file */
}

/**
  * @}
  */

/** @addtogroup FLASH_ITF_Exported_Functions_Group4
  * @{
This subsection provides a set of functions allowing programming of the option bytes functions configuration:

- Call the function HAL_FLASH_ITF_OB_Program() to program the FLASH option bytes in polling mode.
- Call the function HAL_FLASH_ITF_OB_Program_IT() to program the FLASH option bytes in interrupt mode.
  */

/**
  * @brief  Program the FLASH ITF option bytes.
  * @param  instance  The FLASH instance.
  * @retval HAL_ERROR   Generated when an error occurred.
  * @retval HAL_TIMEOUT User timeout.
  * @retval HAL_OK      No error occurred.
  */
hal_status_t HAL_FLASH_ITF_OB_Program(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  uint32_t tick_start;

  LL_FLASH_OB_StartModification(FLASH_GET_ITF_INSTANCE(instance));

  tick_start = HAL_GetTick();

  /* Wait for BSY flag to be cleared */
  while (LL_FLASH_IsActiveFlag_BSY(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
  {
    if ((HAL_GetTick() - tick_start) > FLASH_ITF_OB_TIMEOUT)
    {
      if (LL_FLASH_IsActiveFlag_BSY(FLASH_GET_ITF_INSTANCE(instance)) != 0U)
      {
        /* Timeout occurred while waiting for end of operation */
        return HAL_TIMEOUT;
      }
    }
  }

  if (LL_FLASH_ReadFlag(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_FLAG_ERRORS_ALL) != 0U)
  {
    LL_FLASH_ClearFlag(FLASH_GET_ITF_INSTANCE(instance), LL_FLASH_FLAG_ERRORS_ALL);

    return HAL_ERROR;
  }

  LL_FLASH_ClearFlag_EOP(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Program the FLASH Option Bytes interface settings in interrupt mode.
  * @param  instance  Specifies the FLASH instance based on @ref hal_flash_t enumeration.
  * @retval HAL_OK    FLASH Option Bytes settings successfully programmed.
  */
hal_status_t HAL_FLASH_ITF_OB_Program_IT(hal_flash_t instance)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));

  LL_FLASH_ClearFlag_EOP(FLASH_GET_ITF_INSTANCE(instance));

  LL_FLASH_EnableIT(FLASH_GET_ITF_INSTANCE(instance), (LL_FLASH_IT_EOP | LL_FLASH_IT_OPTCHANGEERR));

  LL_FLASH_OB_StartModification(FLASH_GET_ITF_INSTANCE(instance));

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup FLASH_ITF_Exported_Functions_Group5
  * @{
This subsection provides a set of functions allowing management of privileged access level attributes:

- Call the HAL_FLASH_ITF_SetPrivAttr() function to set privilege attribute.

- Call the HAL_FLASH_ITF_GetPrivAttr() function to get privilege attribute.
  */

/**
  * @brief  Set privilege attribute.
  * @param  instance  The FLASH instance.
  * @param  item      The item attribute to be configured.
  *                   This parameter can be the following value:
  *                   @arg @ref HAL_FLASH_ITF_PRIV_ITEM_ALL
  * @param  priv_attr This parameter is an element of @ref hal_flash_itf_priv_attr_t enumeration:
  *                   @arg @ref HAL_FLASH_ITF_PRIV
  *                   @arg @ref HAL_FLASH_ITF_NPRIV
  * @retval HAL_ERROR  The function is called in unprivileged mode.
  * @retval HAL_OK     Privilege attribute has been correctly set.
  */
hal_status_t HAL_FLASH_ITF_SetPrivAttr(hal_flash_t instance, uint32_t item, hal_flash_itf_priv_attr_t priv_attr)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_PRIV_ITEM(item));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_PRIV_ATTR(priv_attr));


  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  LL_FLASH_SetPrivAttr(FLASH_GET_ITF_INSTANCE(instance), item, (uint32_t)priv_attr);

  return HAL_OK;
}

/**
  * @brief  Get privilege attribute.
  * @param  instance The FLASH instance.
  * @param  item     The item attribute to be queried.
  *                  This parameter can be the following value:
  *                  @arg @ref HAL_FLASH_ITF_PRIV_ITEM_ALL
  * @retval hal_flash_itf_priv_attr_t Returned value is an element of @ref hal_flash_itf_priv_attr_t enumeration:
  *                                   @arg @ref HAL_FLASH_ITF_PRIV
  *                                   @arg @ref HAL_FLASH_ITF_NPRIV
  */
hal_flash_itf_priv_attr_t HAL_FLASH_ITF_GetPrivAttr(hal_flash_t instance, uint32_t item)
{
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_ITF_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_FLASH_ITF_PRIV_ITEM(item));

  return ((hal_flash_itf_priv_attr_t)LL_FLASH_GetPrivAttr(FLASH_GET_ITF_INSTANCE(instance), item));
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

#endif /* USE_HAL_FLASH_MODULE */
#endif /* FLASH */

/**
  * @}
  */
