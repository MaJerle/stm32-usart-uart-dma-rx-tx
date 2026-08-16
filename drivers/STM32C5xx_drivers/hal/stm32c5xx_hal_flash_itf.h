/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_flash_itf.h
  * @brief   Header file of FLASH ITF HAL module.
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

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_FLASH_ITF_H
#define STM32C5XX_HAL_FLASH_ITF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_flash.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (FLASH)

/** @defgroup FLASH_ITF FLASH Interface
  * @{
  */

/* Exported Constants ------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_ITF_Exported_Constants HAL FLASH ITF Constants
  * @{
  */

/** @defgroup FLASH_ITF_OB_PWR_MODE FLASH ITF OB PWR mode
  * @{
  */
#define HAL_FLASH_ITF_OB_STOP_MODE     1U /*!< FLASH ITF option bytes stop mode     */
#define HAL_FLASH_ITF_OB_STANDBY_MODE  2U /*!< FLASH ITF option bytes standby mode  */
/**
  * @}
  */

/** @defgroup FLASH_ITF_OB_SRAM FLASH ITF OB SRAM
  * @{
  */
#define HAL_FLASH_ITF_OB_SRAM2         2U /*!< FLASH ITF option bytes SRAM2         */
#define HAL_FLASH_ITF_OB_SRAM1         8U /*!< FLASH ITF option bytes SRAM1         */
/**
  * @}
  */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/** @defgroup FLASH_ITF_ECC_Interrupts FLASH ITF ECC Interrupts
  * @{
  */
#define HAL_FLASH_ITF_IT_ECC_SINGLE    1U /*!< FLASH ITF ECC single error interrupt */
/**
  * @}
  */
#endif /* USE_HAL_FLASH_ECC */

/** @defgroup FLASH_ITF_privilege_items FLASH ITF privilege items definition
  * @{
  */
#define HAL_FLASH_ITF_PRIV_ITEM_ALL LL_FLASH_PRIV_ITEM_ALL /*!< All privilege items  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_ITF_Exported_Types HAL FLASH ITF Types
  * @{
  */

/*! HAL FLASH instances enumeration definition */
typedef enum
{
  HAL_FLASH = FLASH_R_BASE, /*!< FLASH Instance */
} hal_flash_t;

/*! HAL FLASH bank enumeration definition */
typedef enum
{
  HAL_FLASH_BANK_1   = 0U, /*!< FLASH bank 1   */
  HAL_FLASH_BANK_2   = 1U, /*!< FLASH bank 2   */
  HAL_FLASH_BANK_ALL = 2U  /*!< FLASH all banks. This value is only used internally for mass erase feature.
                                It must not be used for other purposes. */
} hal_flash_bank_t;

/*! HAL FLASH ITF lock status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_UNLOCKED = 0U, /*!< FLASH ITF unlocked */
  HAL_FLASH_ITF_LOCKED   = 1U  /*!< FLASH ITF locked   */
} hal_flash_itf_lock_status_t;

/*! HAL FLASH ITF latency enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_LATENCY_0  = LL_FLASH_LATENCY_0WS,  /*!< FLASH ITF zero wait state      */
  HAL_FLASH_ITF_LATENCY_1  = LL_FLASH_LATENCY_1WS,  /*!< FLASH ITF one wait state       */
  HAL_FLASH_ITF_LATENCY_2  = LL_FLASH_LATENCY_2WS,  /*!< FLASH ITF two wait states      */
  HAL_FLASH_ITF_LATENCY_3  = LL_FLASH_LATENCY_3WS,  /*!< FLASH ITF three wait states    */
  HAL_FLASH_ITF_LATENCY_4  = LL_FLASH_LATENCY_4WS,  /*!< FLASH ITF four wait states     */
  HAL_FLASH_ITF_LATENCY_5  = LL_FLASH_LATENCY_5WS,  /*!< FLASH ITF five wait states     */
  HAL_FLASH_ITF_LATENCY_6  = LL_FLASH_LATENCY_6WS,  /*!< FLASH ITF six wait states      */
  HAL_FLASH_ITF_LATENCY_7  = LL_FLASH_LATENCY_7WS,  /*!< FLASH ITF seven wait states    */
  HAL_FLASH_ITF_LATENCY_8  = LL_FLASH_LATENCY_8WS,  /*!< FLASH ITF eight wait states    */
  HAL_FLASH_ITF_LATENCY_9  = LL_FLASH_LATENCY_9WS,  /*!< FLASH ITF nine wait states     */
  HAL_FLASH_ITF_LATENCY_10 = LL_FLASH_LATENCY_10WS, /*!< FLASH ITF ten wait states      */
  HAL_FLASH_ITF_LATENCY_11 = LL_FLASH_LATENCY_11WS, /*!< FLASH ITF eleven wait states   */
  HAL_FLASH_ITF_LATENCY_12 = LL_FLASH_LATENCY_12WS, /*!< FLASH ITF twelve wait states   */
  HAL_FLASH_ITF_LATENCY_13 = LL_FLASH_LATENCY_13WS, /*!< FLASH ITF thirteen wait states */
  HAL_FLASH_ITF_LATENCY_14 = LL_FLASH_LATENCY_14WS, /*!< FLASH ITF fourteen wait states */
  HAL_FLASH_ITF_LATENCY_15 = LL_FLASH_LATENCY_15WS  /*!< FLASH ITF fifteen wait states  */
} hal_flash_itf_latency_t;

/*! HAL FLASH ITF programming delay enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_PROGRAM_DELAY_0  = LL_FLASH_PROGRAM_DELAY_0,  /*!< FLASH ITF 0 programming delay */
  HAL_FLASH_ITF_PROGRAM_DELAY_1  = LL_FLASH_PROGRAM_DELAY_1,  /*!< FLASH ITF 1 programming delay */
  HAL_FLASH_ITF_PROGRAM_DELAY_2  = LL_FLASH_PROGRAM_DELAY_2,  /*!< FLASH ITF 2 programming delay */
  HAL_FLASH_ITF_PROGRAM_DELAY_3  = LL_FLASH_PROGRAM_DELAY_3,  /*!< FLASH ITF 3 programming delay */
} hal_flash_itf_program_delay_t;

/*! HAL FLASH ITF Prefetch enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_PREFETCH_DISABLED  = 0U, /*!< FLASH ITF prefetch disabled */
  HAL_FLASH_ITF_PREFETCH_ENABLED   = 1U  /*!< FLASH ITF prefetch enabled  */
} hal_flash_itf_prefetch_status_t;

/*! HAL FLASH ITF Empty boot location enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_BOOT_LOCATION_PROGRAMMED = LL_FLASH_BOOT_LOCATION_PROGRAMMED, /*!< FLASH ITF boot location programmed */
  HAL_FLASH_ITF_BOOT_LOCATION_EMPTY      = LL_FLASH_BOOT_LOCATION_EMPTY       /*!< FLASH ITF boot location empty      */
} hal_flash_itf_empty_boot_location_t;

/*! HAL FLASH ITF privilege attribute enumeration definition  */
typedef enum
{
  HAL_FLASH_ITF_NPRIV = LL_FLASH_ATTR_NPRIV,  /*!< FLASH ITF Non-privileged attribute */
  HAL_FLASH_ITF_PRIV  = LL_FLASH_ATTR_PRIV    /*!< FLASH ITF Privileged attribute     */
} hal_flash_itf_priv_attr_t;

/*! HAL FLASH ITF RDP key lock status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_RDP_KEY_UNLOCKED = 0U, /*!< FLASH ITF RDP key unlocked */
  HAL_FLASH_ITF_RDP_KEY_LOCKED   = 1U  /*!< FLASH ITF RDP key locked   */
} hal_flash_itf_rdp_key_lock_status_t;

/*! HAL FLASH ITF option bytes lock status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_UNLOCKED = 0U, /*!< FLASH ITF option bytes unlocked */
  HAL_FLASH_ITF_OB_LOCKED   = 1U  /*!< FLASH ITF option bytes locked   */
} hal_flash_itf_ob_lock_status_t;

/*! HAL FLASH ITF option bytes OTP block lock status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_OTP_BLK_UNLOCKED = 0U, /*!< FLASH ITF option bytes OTP block unlocked */
  HAL_FLASH_ITF_OB_OTP_BLK_LOCKED   = 1U  /*!< FLASH ITF option bytes OTP block locked   */
} hal_flash_itf_ob_otp_blk_lock_status_t;

/*! HAL FLASH ITF option bytes WRP page status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_WRP_PAGE_NOT_PROTECTED = 0U, /*!< FLASH ITF option bytes write protection page not protected */
  HAL_FLASH_ITF_OB_WRP_PAGE_PROTECTED     = 1U  /*!< FLASH ITF option bytes write protection page protected     */
} hal_flash_itf_ob_wrp_page_status_t;

/*! HAL FLASH ITF option bytes EDATA area status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_EDATA_AREA_DISABLED = 0U, /*!< FLASH ITF option bytes EDATA area disabled */
  HAL_FLASH_ITF_OB_EDATA_AREA_ENABLED  = 1U  /*!< FLASH ITF option bytes EDATA area enabled  */
} hal_flash_itf_ob_edata_area_status_t;

/*! HAL FLASH ITF option bytes Read-out Protection level enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_RDP_LEVEL_0     = LL_FLASH_OB_RDP_LEVEL_0,     /*!< FLASH ITF OB RDP Level 0                    */
  HAL_FLASH_ITF_OB_RDP_LEVEL_2_WBS = LL_FLASH_OB_RDP_LEVEL_2_WBS, /*!< FLASH ITF OB RDP Level 2 with Boundary Scan */
  HAL_FLASH_ITF_OB_RDP_LEVEL_2     = LL_FLASH_OB_RDP_LEVEL_2,     /*!< FLASH ITF OB RDP Level 2                    */
} hal_flash_itf_ob_rdp_level_t;

/*! HAL FLASH ITF option bytes reset generation when enter in low power mode enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_RST_GENERATION    = 0U, /*!< FLASH ITF option bytes reset generation when entering low power mode */
  HAL_FLASH_ITF_OB_NO_RST_GENERATION = 1U  /*!< FLASH ITF option bytes no reset generation when entering
                                                low power mode                                                       */
} hal_flash_itf_ob_rst_generation_status_t ;

/*! HAL FLASH ITF option bytes Erased SRAM when system reset enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_SYS_RST_SRAM_ERASE    = 0U, /*!< FLASH ITF option bytes erased SRAM when system reset occurs    */
  HAL_FLASH_ITF_OB_SYS_RST_SRAM_NO_ERASE = 1U  /*!< FLASH ITF option bytes no erased SRAM when system reset occurs */
} hal_flash_itf_ob_sys_rst_sram_erase_t;

/*! HAL FLASH ITF option bytes xWDG hardware/software mode enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_WDG_HARDWARE = 0U,  /*!< FLASH ITF option bytes WDG hardware select */
  HAL_FLASH_ITF_OB_WDG_SOFTWARE = 1U   /*!< FLASH ITF option bytes WDG software select */
} hal_flash_itf_ob_wdg_mode_t;

/*! HAL FLASH ITF option bytes IWDG counter low-power mode freeze status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_WDG_FROZEN   = 1U, /*!< FLASH ITF option bytes IWDG counter low-power mode frozen  */
  HAL_FLASH_ITF_OB_WDG_UNFROZEN = 0U  /*!< FLASH ITF option bytes IWDG counter low-power mode running */
} hal_flash_itf_ob_wdg_freeze_status_t;

/*! HAL FLASH ITF option bytes  BOOT0 source selection enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_BOOT_OPTION_BIT = LL_FLASH_OB_BOOT0_BOOTBIT, /*!< FLASH ITF option bytes BOOT taken from
                                                                     option bit                                      */
  HAL_FLASH_ITF_OB_BOOT_PIN        = LL_FLASH_OB_BOOT0_BOOTPIN  /*!< FLASH ITF option bytes BOOT taken from boot pin */
} hal_flash_itf_ob_boot_selection_t;

/*! HAL FLASH ITF option bytes software BOOT0 enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_BOOT_LOW  = LL_FLASH_OB_BOOT0_DISABLED, /*!< FLASH ITF option bytes BOOT option bit low state  */
  HAL_FLASH_ITF_OB_BOOT_HIGH = LL_FLASH_OB_BOOT0_ENABLED,  /*!< FLASH ITF option bytes BOOT option bit high state */
} hal_flash_itf_ob_boot_state_t;


/*! HAL FLASH ITF option bytes Single/Dual bank enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_SINGLE_BANK = LL_FLASH_OB_SINGLE_BANK, /*!< FLASH ITF option bytes single bank */
  HAL_FLASH_ITF_OB_DUAL_BANK   = LL_FLASH_OB_DUAL_BANK    /*!< FLASH ITF option bytes dual bank   */
} hal_flash_itf_ob_topology_t;

/*! HAL FLASH ITF option bytes Swapping bank enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_BANK_NO_SWAP = LL_FLASH_OB_BANK_NOT_SWAPPED, /*!< FLASH ITF option bytes bank no swap */
  HAL_FLASH_ITF_OB_BANK_SWAP    = LL_FLASH_OB_BANK_SWAPPED      /*!< FLASH ITF option bytes bank swap    */
} hal_flash_itf_ob_bank_swap_t;

/*! HAL FLASH ITF option bytes Swapping bank status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_BANK_NOT_SWAPPED = 0U, /*!< FLASH ITF option bytes bank not swapped */
  HAL_FLASH_ITF_OB_BANK_SWAPPED     = 1U  /*!< FLASH ITF option bytes bank swapped     */
} hal_flash_itf_ob_bank_swap_status_t;

/*! HAL FLASH ITF option bytes SRAM ECC enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_SRAM_ECC_DISABLED = 0U, /*!< FLASH ITF option bytes SRAM ECC disable */
  HAL_FLASH_ITF_OB_SRAM_ECC_ENABLED  = 1U  /*!< FLASH ITF option bytes SRAM ECC enable  */
} hal_flash_itf_ob_sram_ecc_status_t ;

/*! HAL FLASH ITF option bytes boot lock status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_OB_BOOT_UNLOCKED = 0U, /*!< FLASH ITF option bytes boot unlocked */
  HAL_FLASH_ITF_OB_BOOT_LOCKED   = 1U  /*!< FLASH ITF option bytes boot locked   */
} hal_flash_itf_ob_boot_lock_status_t;

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/*! HAL FLASH ITF ECC interrupt status enumeration definition */
typedef enum
{
  HAL_FLASH_ITF_ECC_IT_DISABLED = 0U, /*!< FLASH ITF ECC interrupt disabled */
  HAL_FLASH_ITF_ECC_IT_ENABLED  = 1U  /*!< FLASH ITF ECC interrupt enabled  */
} hal_flash_itf_ecc_it_status_t;
#endif /* USE_HAL_FLASH_ECC */

/*! HAL FLASH ITF option bytes read-out protection OEM key structure definition */
typedef struct
{
  uint32_t key_w1; /*!< FLASH ITF RDP OEM key word 1 */
  uint32_t key_w2; /*!< FLASH ITF RDP OEM key word 2 */
  uint32_t key_w3; /*!< FLASH ITF RDP OEM key word 3 */
  uint32_t key_w4; /*!< FLASH ITF RDP OEM key word 4 */
} hal_flash_itf_ob_rdp_oem_key_t;

/*! HAL FLASH ITF option bytes read-out protection BS key structure definition */
typedef struct
{
  uint32_t key_w1; /*!< FLASH ITF RDP BS key word 1 */
} hal_flash_itf_ob_rdp_bs_key_t;

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_ITF_Exported_Functions HAL FLASH ITF Functions
  * @{
  */

/** @defgroup FLASH_ITF_Exported_Functions_Group1 FLASH ITF Control and Lock/Unlock functions
  * @{
  */

hal_status_t HAL_FLASH_ITF_Lock(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_Unlock(hal_flash_t instance);
hal_flash_itf_lock_status_t HAL_FLASH_ITF_IsLocked(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_SetLatency(hal_flash_t instance, hal_flash_itf_latency_t latency);
hal_flash_itf_latency_t HAL_FLASH_ITF_GetLatency(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_SetProgrammingDelay(hal_flash_t instance, hal_flash_itf_program_delay_t prog_delay);
hal_flash_itf_program_delay_t HAL_FLASH_ITF_GetProgrammingDelay(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_EnablePrefetch(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_DisablePrefetch(hal_flash_t instance);
hal_flash_itf_prefetch_status_t HAL_FLASH_ITF_IsEnabledPrefetch(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_SetEmptyBootLocation(hal_flash_t instance, hal_flash_itf_empty_boot_location_t empty_boot);
hal_flash_itf_empty_boot_location_t HAL_FLASH_ITF_GetEmptyBootLocation(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_SetHDPExtArea(hal_flash_t instance, hal_flash_bank_t bank, uint32_t page_nbr);
uint32_t HAL_FLASH_ITF_GetHDPExtArea(hal_flash_t instance, hal_flash_bank_t bank);

hal_flash_itf_rdp_key_lock_status_t HAL_FLASH_ITF_IsLockedRDPOEMKey(hal_flash_t instance);
hal_flash_itf_rdp_key_lock_status_t HAL_FLASH_ITF_IsLockedRDPBSKey(hal_flash_t instance);

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
hal_status_t HAL_FLASH_ITF_ECC_EnableIT(hal_flash_t instance, uint32_t interrupt);
hal_status_t HAL_FLASH_ITF_ECC_DisableIT(hal_flash_t instance, uint32_t interrupt);
hal_flash_itf_ecc_it_status_t HAL_FLASH_ITF_ECC_IsEnabledIT(hal_flash_t instance, uint32_t interrupt);
#endif /* USE_HAL_FLASH_ECC */

/**
  * @}
  */

/** @defgroup FLASH_ITF_Exported_Functions_Group2 FLASH ITF Option bytes configuration functions
  * @{
  */
hal_status_t HAL_FLASH_ITF_OB_Lock(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_OB_Unlock(hal_flash_t instance);
hal_flash_itf_ob_lock_status_t HAL_FLASH_ITF_OB_IsLocked(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_LockOTPBlock(hal_flash_t instance, uint32_t start_otp_block, uint32_t otp_block_nbr);
hal_flash_itf_ob_otp_blk_lock_status_t HAL_FLASH_ITF_OB_IsLockedOTPBlock(hal_flash_t instance,
                                                                         uint32_t otp_block);

hal_status_t HAL_FLASH_ITF_OB_EnablePageWRP(hal_flash_t instance,
                                            hal_flash_bank_t bank,
                                            uint32_t start_page,
                                            uint32_t page_nbr);
hal_status_t HAL_FLASH_ITF_OB_DisablePageWRP(hal_flash_t instance,
                                             hal_flash_bank_t bank,
                                             uint32_t start_page,
                                             uint32_t page_nbr);
hal_flash_itf_ob_wrp_page_status_t HAL_FLASH_ITF_OB_IsEnabledPageWRP(hal_flash_t instance,
                                                                     hal_flash_bank_t bank,
                                                                     uint32_t page);

hal_status_t HAL_FLASH_ITF_OB_SetHDPArea(hal_flash_t instance,
                                         hal_flash_bank_t bank,
                                         uint32_t start_page,
                                         uint32_t page_nbr);
void HAL_FLASH_ITF_OB_GetHDPArea(hal_flash_t instance,
                                 hal_flash_bank_t bank,
                                 uint32_t *p_start_page,
                                 uint32_t *p_page_nbr);

hal_status_t HAL_FLASH_ITF_OB_EnableEDATAArea(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_OB_DisableEDATAArea(hal_flash_t instance);
hal_flash_itf_ob_edata_area_status_t HAL_FLASH_ITF_OB_IsEnabledEDATAArea(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetRDPLevel(hal_flash_t instance, hal_flash_itf_ob_rdp_level_t rdp_level);
hal_flash_itf_ob_rdp_level_t HAL_FLASH_ITF_OB_GetRDPLevel(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetRDPOEMKey(hal_flash_t instance, const hal_flash_itf_ob_rdp_oem_key_t *p_key);
hal_status_t HAL_FLASH_ITF_OB_SetRDPBSKey(hal_flash_t instance, const hal_flash_itf_ob_rdp_bs_key_t *p_key);

hal_status_t HAL_FLASH_ITF_OB_SetEnterLowPWRModeRstGeneration(hal_flash_t instance, uint32_t low_pwr_mode,
                                                              hal_flash_itf_ob_rst_generation_status_t rst_gen);
hal_flash_itf_ob_rst_generation_status_t HAL_FLASH_ITF_OB_GetEnterLowPWRModeRstGeneration(hal_flash_t instance,
    uint32_t low_pwr_mode);

hal_status_t HAL_FLASH_ITF_OB_SetSystemRstSRAMErase(hal_flash_t instance,
                                                    uint32_t sram,
                                                    hal_flash_itf_ob_sys_rst_sram_erase_t sram_erase);
hal_flash_itf_ob_sys_rst_sram_erase_t HAL_FLASH_ITF_OB_GetSystemRstSRAMErase(hal_flash_t instance,
                                                                             uint32_t sram);

hal_status_t HAL_FLASH_ITF_OB_SetIWDGMode(hal_flash_t instance, hal_flash_itf_ob_wdg_mode_t mode);
hal_flash_itf_ob_wdg_mode_t HAL_FLASH_ITF_OB_GetIWDGMode(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetWWDGMode(hal_flash_t instance, hal_flash_itf_ob_wdg_mode_t mode);
hal_flash_itf_ob_wdg_mode_t HAL_FLASH_ITF_OB_GetWWDGMode(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_FreezeIWDGCounterLowPWRMode(hal_flash_t instance, uint32_t low_pwr_mode);
hal_status_t HAL_FLASH_ITF_OB_UnfreezeIWDGCounterLowPWRMode(hal_flash_t instance, uint32_t low_pwr_mode);
hal_flash_itf_ob_wdg_freeze_status_t HAL_FLASH_ITF_OB_IsFrozenIWDGCounterLowPWRMode(hal_flash_t instance,
    uint32_t low_pwr_mode);

hal_status_t HAL_FLASH_ITF_OB_SetBootSelection(hal_flash_t instance, hal_flash_itf_ob_boot_selection_t boot_select);
hal_flash_itf_ob_boot_selection_t HAL_FLASH_ITF_OB_GetBootSelection(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetBoot0(hal_flash_t instance, hal_flash_itf_ob_boot_state_t state);
hal_flash_itf_ob_boot_state_t HAL_FLASH_ITF_OB_GetBoot0(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetBankTopology(hal_flash_t instance, hal_flash_itf_ob_topology_t bank_topology);
hal_flash_itf_ob_topology_t HAL_FLASH_ITF_OB_GetBankTopology(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetBankSwap(hal_flash_t instance, hal_flash_itf_ob_bank_swap_t bank_swap);
hal_flash_itf_ob_bank_swap_t HAL_FLASH_ITF_OB_GetBankSwap(hal_flash_t instance);
hal_flash_itf_ob_bank_swap_status_t HAL_FLASH_ITF_OB_IsBankSwapped(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_EnableSRAMECC(hal_flash_t instance, uint32_t sram);
hal_status_t HAL_FLASH_ITF_OB_DisableSRAMECC(hal_flash_t instance, uint32_t sram);
hal_flash_itf_ob_sram_ecc_status_t HAL_FLASH_ITF_OB_IsEnabledSRAMECC(hal_flash_t instance, uint32_t sram);

hal_status_t HAL_FLASH_ITF_OB_SetBootAddr(hal_flash_t instance, uint32_t boot_addr);
uint32_t HAL_FLASH_ITF_OB_GetBootAddr(hal_flash_t instance);


hal_status_t HAL_FLASH_ITF_OB_LockBootConfig(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_OB_UnlockBootConfig(hal_flash_t instance);
hal_flash_itf_ob_boot_lock_status_t HAL_FLASH_ITF_OB_IsLockedBootConfig(hal_flash_t instance);

hal_status_t HAL_FLASH_ITF_OB_SetBootloaderInterfaceConfig(hal_flash_t instance, uint32_t bootloader_config);
uint32_t HAL_FLASH_ITF_OB_GetBootloaderInterfaceConfig(hal_flash_t instance);
/**
  * @}
  */

/** @defgroup FLASH_ITF_Exported_Functions_Group3 FLASH ITF IRQHandler and callback functions
  * @{
  */
void HAL_FLASH_ITF_IRQHandler(hal_flash_t instance);


void HAL_FLASH_ITF_OB_ProgramCpltCallback(hal_flash_t instance);
void HAL_FLASH_ITF_OB_ErrorCallback(hal_flash_t instance);
/**
  * @}
  */

/** @defgroup FLASH_ITF_Exported_Functions_Group4 FLASH ITF program option bytes functions configuration
  * @{
  */
hal_status_t HAL_FLASH_ITF_OB_Program(hal_flash_t instance);
hal_status_t HAL_FLASH_ITF_OB_Program_IT(hal_flash_t instance);
/**
  * @}
  */

/** @defgroup FLASH_ITF_Exported_Functions_Group5 FLASH ITF privileged access levels attributes management functions
  * @{
  */
hal_status_t HAL_FLASH_ITF_SetPrivAttr(hal_flash_t instance, uint32_t item, hal_flash_itf_priv_attr_t priv_attr);
hal_flash_itf_priv_attr_t HAL_FLASH_ITF_GetPrivAttr(hal_flash_t instance, uint32_t item);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* FLASH */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_FLASH_ITF_H */
