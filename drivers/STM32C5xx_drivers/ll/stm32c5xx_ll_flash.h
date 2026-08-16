/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_flash.h
  * @brief   Header file of FLASH LL module.
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
#ifndef STM32C5XX_LL_FLASH_H
#define STM32C5XX_LL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (FLASH)

/** @defgroup FLASH_LL FLASH
  * @{
  */

/** @defgroup FLASH_LL_Exported_Constants LL FLASH Constants
  * @{
  */

/** @defgroup FLASH_Interrupt_Flags FLASH Interrupt flags
  * @{
  */
#define LL_FLASH_IT_EOP             FLASH_CR_EOPIE                     /*!< End of operation interrupt flag           */
#define LL_FLASH_IT_WRPERR          FLASH_CR_WRPERRIE                  /*!< Write protection error interrupt flag     */
#define LL_FLASH_IT_PGSERR          FLASH_CR_PGSERRIE                  /*!< Programming sequence error interrupt flag */
#define LL_FLASH_IT_STRBERR         FLASH_CR_STRBERRIE                 /*!< Strobe error interrupt flag               */
#define LL_FLASH_IT_INCERR          FLASH_CR_INCERRIE                  /*!< Inconsistency error interrupt flag        */
#define LL_FLASH_IT_OPTCHANGEERR    FLASH_CR_OPTCHANGEERRIE            /*!< Option-byte change error interrupt flag   */
#define LL_FLASH_IT_ERRORS_ALL      (FLASH_CR_WRPERRIE | FLASH_CR_PGSERRIE | FLASH_CR_STRBERRIE \
                                     | FLASH_CR_INCERRIE | FLASH_CR_OPTCHANGEERRIE) /* All error interrupts           */
#define LL_FLASH_IT_ALL             (FLASH_CR_EOPIE | LL_FLASH_IT_ERRORS_ALL)       /* All interrupt flags            */
/**
  * @}
  */

/** @defgroup FLASH_Interrupted_Operation_Area_Flags FLASH Interrupted Operation Area flags
  * @{
  */
#define LL_FLASH_FLAG_DATA_OP        FLASH_OPSR_DATA_OP             /* Interrupted operation in data flash flag */
#define LL_FLASH_FLAG_BK_OP          FLASH_OPSR_BK_OP               /* Interrupted operation bank flag          */
#define LL_FLASH_FLAG_OTP_OP         FLASH_OPSR_OTP_OP              /* Interrupted operation in OTP flash flag  */
#define LL_FLASH_FLAG_OP_AREA_ALL    (FLASH_OPSR_DATA_OP \
                                      | FLASH_OPSR_BK_OP \
                                      | FLASH_OPSR_OTP_OP)          /* All interrupted operation area flags     */
/**
  * @}
  */

/** @defgroup FLASH_Status_Flags FLASH Status flags
  * @{
  */
#define LL_FLASH_FLAG_BSY             FLASH_SR_BSY                        /*!< Busy flag                    */
#define LL_FLASH_FLAG_WBNE            FLASH_SR_WBNE                       /*!< Write buffer not empty flag  */
#define LL_FLASH_FLAG_DBNE            FLASH_SR_DBNE                       /*!< Data buffer not empty flag   */
#define LL_FLASH_FLAG_STATUS_ALL      (FLASH_SR_BSY | FLASH_SR_WBNE | FLASH_SR_DBNE) /*!< All Status flags  */
/**
  * @}
  */

/** @defgroup FLASH_Error_Flags FLASH Error flags
  * @{
  */
#define LL_FLASH_FLAG_WRPERR          FLASH_SR_WRPERR                 /*!< Write protection error flag     */
#define LL_FLASH_FLAG_PGSERR          FLASH_SR_PGSERR                 /*!< Programming sequence error flag */
#define LL_FLASH_FLAG_STRBERR         FLASH_SR_STRBERR                /*!< Strobe error flag               */
#define LL_FLASH_FLAG_INCERR          FLASH_SR_INCERR                 /*!< Inconsistency error flag        */
#define LL_FLASH_FLAG_OPTCHANGEERR    FLASH_SR_OPTCHANGEERR           /*!< Option-byte change error flag   */
#define LL_FLASH_FLAG_ERRORS_ALL      (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR \
                                       | FLASH_SR_INCERR | FLASH_SR_OPTCHANGEERR)     /*!< All error flags */
#define LL_FLASH_FLAG_EOP             FLASH_SR_EOP                    /*!< End of Operation flag           */
/**
  * @}
  */

/** @defgroup FLASH_ECC_Error_Flags FLASH ECC Error flags
  * @{
  */
#define LL_FLASH_FLAG_ECCC            FLASH_ECCCORR_ECCC                 /*!< ECC single error flag */
#define LL_FLASH_FLAG_ECCD            FLASH_ECCDETR_ECCD                 /*!< ECC double error flag */
/**
  * @}
  */

/** @defgroup FLASH_ECC_Area_Flags FLASH ECC Area flags
  * @{
  */
#define LL_FLASH_FLAG_EDATA_ECC       FLASH_ECCCORR_EDATA_ECC            /* ECC error in data flash flag   */
#define LL_FLASH_FLAG_BK_ECC          FLASH_ECCCORR_BK_ECC               /* ECC error bank flag            */
#define LL_FLASH_FLAG_SYSF_ECC        FLASH_ECCCORR_SYSF_ECC             /* ECC error in system flash flag */
#define LL_FLASH_FLAG_OTP_ECC         FLASH_ECCCORR_OTP_ECC              /* ECC error in OTP flash flag    */
#define LL_FLASH_FLAG_ECC_AREA_ALL    (FLASH_ECCCORR_EDATA_ECC | FLASH_ECCCORR_BK_ECC \
                                       | FLASH_ECCCORR_SYSF_ECC | FLASH_ECCCORR_OTP_ECC) /* All ECC error area flags */
/**
  * @}
  */

/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define LL_FLASH_LATENCY_0WS  0x00000000U          /*!< Zero wait state      */
#define LL_FLASH_LATENCY_1WS  FLASH_ACR_LATENCY_0  /*!< One wait state       */
#define LL_FLASH_LATENCY_2WS  FLASH_ACR_LATENCY_1  /*!< Two wait states      */
#define LL_FLASH_LATENCY_3WS  FLASH_ACR_LATENCY_2  /*!< Three wait states    */
#define LL_FLASH_LATENCY_4WS  FLASH_ACR_LATENCY_3  /*!< Four wait states     */
#define LL_FLASH_LATENCY_5WS  FLASH_ACR_LATENCY_4  /*!< Five wait states     */
#define LL_FLASH_LATENCY_6WS  FLASH_ACR_LATENCY_5  /*!< Six wait states      */
#define LL_FLASH_LATENCY_7WS  FLASH_ACR_LATENCY_6  /*!< Seven wait states    */
#define LL_FLASH_LATENCY_8WS  FLASH_ACR_LATENCY_7  /*!< Eight wait states    */
#define LL_FLASH_LATENCY_9WS  FLASH_ACR_LATENCY_8  /*!< Nine wait states     */
#define LL_FLASH_LATENCY_10WS FLASH_ACR_LATENCY_9  /*!< Ten wait states      */
#define LL_FLASH_LATENCY_11WS FLASH_ACR_LATENCY_10 /*!< Eleven wait states   */
#define LL_FLASH_LATENCY_12WS FLASH_ACR_LATENCY_11 /*!< Twelve wait states   */
#define LL_FLASH_LATENCY_13WS FLASH_ACR_LATENCY_12 /*!< Thirteen wait states */
#define LL_FLASH_LATENCY_14WS FLASH_ACR_LATENCY_13 /*!< Fourteen wait states */
#define LL_FLASH_LATENCY_15WS FLASH_ACR_LATENCY_14 /*!< Fifteen wait states  */
/**
  * @}
  */

/** @defgroup FLASH_ProgrammingDelay FLASH Programming Delay
  * @{
  */
#define LL_FLASH_PROGRAM_DELAY_0 0x00000000U                /*!< Programming delay for Flash frequency at 68 MHz or
                                                                 less */
#define LL_FLASH_PROGRAM_DELAY_1 FLASH_ACR_WRHIGHFREQ_0     /*!< Programming delay for Flash frequency between 68 MHz
                                                                 and 136 MHz */
#define LL_FLASH_PROGRAM_DELAY_2 FLASH_ACR_WRHIGHFREQ_1     /*!< Programming delay for Flash frequency between 136 MHz
                                                                 and 170 MHz */
#define LL_FLASH_PROGRAM_DELAY_3 (FLASH_ACR_WRHIGHFREQ_1 \
                                  | FLASH_ACR_WRHIGHFREQ_0) /*!< Programming delay for Flash frequency at 170 MHz or
                                                                 above */
/**
  * @}
  */

/** @defgroup FLASH_EmptyBootLocation FLASH Empty Boot Location
  * @{
  */
#define LL_FLASH_BOOT_LOCATION_PROGRAMMED 0x00000000U   /*!< Boot location is programmed */
#define LL_FLASH_BOOT_LOCATION_EMPTY FLASH_ACR_EMPTY    /*!< Boot location is empty */
/**
  * @}
  */

/** @defgroup FLASH_privilege_items  FLASH Privilege items
  * @{
  */
#define LL_FLASH_PRIV_ITEM_ALL     FLASH_PRIVCFGR_PRIV   /*!< All privilege items  */
/**
  * @}
  */

/** @defgroup FLASH_privilege_attribute  Privilege attribute
  * @{
  */
#define LL_FLASH_ATTR_NPRIV     0U   /*!< Non-privileged attribute  */
#define LL_FLASH_ATTR_PRIV      1U   /*!< Privileged attribute      */
/**
  * @}
  */

/** @defgroup FLASH_Unlock_Keys FLASH Unlock Keys
  * @{
  */
#define LL_FLASH_KEY1 0x45670123U /*!< Lock key1 */
#define LL_FLASH_KEY2 0xCDEF89ABU /*!< Lock key2 */
/**
  * @}
  */

/** @defgroup FLASH_OB_Unlock_Keys FLASH OB Unlock Keys
  * @{
  */
#define LL_FLASH_OB_OPTKEY1 0x08192A3BU /*!< Option-byte lock key1 */
#define LL_FLASH_OB_OPTKEY2 0x4C5D6E7FU /*!< Option-byte lock key2 */
/**
  * @}
  */

/** @defgroup FLASH_Bank_Selection FLASH Bank Selection
  * @{
  */
#define LL_FLASH_BANK_1 0x0U    /*!< Bank 1 selection */
#define LL_FLASH_BANK_2 0x1U    /*!< Bank 2 selection */
/**
  * @}
  */

/** @defgroup FLASH_Erase_Bank_Selection FLASH Erase Bank Selection
  * @{
  */
#define LL_FLASH_ERASE_BANK_1 0x00000000U       /*!< Page erase bank 1 */
#define LL_FLASH_ERASE_BANK_2 FLASH_CR_BKSEL    /*!< Page erase bank 2 */
/**
  * @}
  */
/** @defgroup FLASH_Erase_Area_Selection FLASH Erase Area Selection
  * @{
  */
#define LL_FLASH_ERASE_USER_AREA  0x00000000U       /*!< Page erase USER area  */
#define LL_FLASH_ERASE_EDATA_AREA FLASH_CR_EDATASEL /*!< Page erase EDATA area */
/**
  * @}
  */


/** @defgroup FLASH_OB_Reset_Generation_Stop_Mode FLASH OB Reset Generation Stop Mode
  * @{
  */
#define LL_FLASH_OB_RST_STOP_MODE    0x00000000U               /*!< Reset in stop mode OB    */
#define LL_FLASH_OB_NO_RST_STOP_MODE FLASH_OPTSR_PRG_NRST_STOP /*!< No reset in stop mode OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Reset_Generation_Standby_Mode FLASH OB Reset Generation standby Mode
  * @{
  */
#define LL_FLASH_OB_RST_STDBY_MODE    0x00000000U                /*!< Reset in standby mode OB    */
#define LL_FLASH_OB_NO_RST_STDBY_MODE FLASH_OPTSR_PRG_NRST_STDBY /*!< No reset in standby mode OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Erase_Sram1_System_Reset FLASH OB Erase Sram1 System Reset
  * @{
  */
#define LL_FLASH_OB_ERASED_SRAM1_SYS_RST     0x00000000U                /*!< Erased sram1 after system reset OB */
#define LL_FLASH_OB_NOT_ERASED_SRAM1_SYS_RST FLASH_OPTSR2_PRG_SRAM1_RST /*!< Not erased sram1 after system reset OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Erase_Sram2_System_Reset FLASH OB Erase Sram2 System Reset
  * @{
  */
#define LL_FLASH_OB_ERASED_SRAM2_SYS_RST     0x00000000U                /*!< Erased sram2 after system reset OB     */
#define LL_FLASH_OB_NOT_ERASED_SRAM2_SYS_RST FLASH_OPTSR2_PRG_SRAM2_RST /*!< Not erased sram2 after system reset OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_IWDG_HW_SW_Selection FLASH OB IWDG HW SW Selection
  * @{
  */
#define LL_FLASH_OB_IWDG_HW 0x00000000U             /*!< IWDG Hardware selection OB */
#define LL_FLASH_OB_IWDG_SW FLASH_OPTSR_PRG_IWDG_SW /*!< IWDG Software selection OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_WWDG_HW_SW_Selection FLASH OB WWDG HW SW Selection
  * @{
  */
#define LL_FLASH_OB_WWDG_HW 0x00000000U             /*!< WWDG Hardware selection OB */
#define LL_FLASH_OB_WWDG_SW FLASH_OPTSR_PRG_WWDG_SW /*!< WWDG Software selection OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Single_Dual_Bank FLASH OB Dual Bank 256K
  * @{
  */
#define LL_FLASH_OB_DUAL_BANK     0x00000000U                 /*!< Dual bank OB   */
#define LL_FLASH_OB_SINGLE_BANK   FLASH_OPTSR_PRG_SINGLE_BANK /*!< Single bank OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Swap_Bank FLASH OB Swap Bank
  * @{
  */
#define LL_FLASH_OB_BANK_NOT_SWAPPED 0x00000000U               /*!< Bank not swapped OB */
#define LL_FLASH_OB_BANK_SWAPPED     FLASH_OPTSR_PRG_SWAP_BANK /*!< Bank swapped OB     */
/**
  * @}
  */

/** @defgroup FLASH_HDPEXT_Bank_Selection FLASH HDPEXT Bank Selection
  * @{
  */
#define LL_FLASH_HDPEXT_BANK_1 FLASH_HDPEXTR_HDP1_EXT_Pos  /*!< HDPEXT bank 1 */
#define LL_FLASH_HDPEXT_BANK_2 FLASH_HDPEXTR_HDP2_EXT_Pos  /*!< HDPEXT bank 2 */
/**
  * @}
  */

/** @defgroup FLASH_OB_BootAddr_Lock FLASH OB Boot Address Lock
  * @{
  */
#define LL_FLASH_OB_BOOT_NOT_LOCKED 0xC3U /*!< Boot OBs not locked  */
#define LL_FLASH_OB_BOOT_LOCKED     0xB4U /*!< Boot OBs locked      */
/**
  * @}
  */

/** @defgroup FLASH_OB_Boot0_Selection FLASH OB Boot0 Selection
  * @{
  */
#define LL_FLASH_OB_BOOT0_BOOTBIT 0x00000000U              /*!< Boot0 selection OB    */
#define LL_FLASH_OB_BOOT0_BOOTPIN FLASH_OPTSR_PRG_BOOT_SEL /*!< Boot pin selection OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_BOOT0_Option FLASH OB BOOT0 Option bit
  * @{
  */
#define LL_FLASH_OB_BOOT0_DISABLED 0x00000000U          /*!< BOOT0 value 0 OB */
#define LL_FLASH_OB_BOOT0_ENABLED FLASH_OPTSR_PRG_BOOT0 /*!< BOOT0 value 1 OB */
/**
  * @}
  */

/** @defgroup FLASH_OB_Read_Protection_Level FLASH OB Read Protection Level
  * @{
  */
#define LL_FLASH_OB_RDP_LEVEL_0     0xEDU /*!< RDP Level 0 OB                    */
#define LL_FLASH_OB_RDP_LEVEL_2_WBS 0xD1U /*!< RDP Level 2 with Boundary Scan OB */
#define LL_FLASH_OB_RDP_LEVEL_2     0x72U /*!< RDP Level 2 OB                    */
/**
  * @}
  */

/** @defgroup LL_FLASH_OB_Write_Protection_Pages FLASH OB Write Protection Pages
  * @{
  */
#if defined(FLASH_WRP_GROUP_WIDTH) && (FLASH_WRP_GROUP_WIDTH == 2U)
#define LL_FLASH_OB_WRP_PAGE_0_1    0x00000001UL               /*!< Write protection of Page0 & 1   */
#define LL_FLASH_OB_WRP_PAGE_2_3    0x00000002UL               /*!< Write protection of Page2 & 3   */
#define LL_FLASH_OB_WRP_PAGE_4_5    0x00000004UL               /*!< Write protection of Page4 & 5   */
#define LL_FLASH_OB_WRP_PAGE_6_7    0x00000008UL               /*!< Write protection of Page6 & 7   */
#define LL_FLASH_OB_WRP_PAGE_8_9    0x00000010UL               /*!< Write protection of Page8 & 9   */
#define LL_FLASH_OB_WRP_PAGE_10_11  0x00000020UL               /*!< Write protection of Page10 & 11 */
#define LL_FLASH_OB_WRP_PAGE_12_13  0x00000040UL               /*!< Write protection of Page12 & 13 */
#define LL_FLASH_OB_WRP_PAGE_14_15  0x00000080UL               /*!< Write protection of Page14 & 15 */
#define LL_FLASH_OB_WRP_PAGE_16_17  0x00000100UL               /*!< Write protection of Page16 & 17 */
#define LL_FLASH_OB_WRP_PAGE_18_19  0x00000200UL               /*!< Write protection of Page18 & 19 */
#define LL_FLASH_OB_WRP_PAGE_20_21  0x00000400UL               /*!< Write protection of Page20 & 21 */
#define LL_FLASH_OB_WRP_PAGE_22_23  0x00000800UL               /*!< Write protection of Page22 & 23 */
#define LL_FLASH_OB_WRP_PAGE_24_25  0x00001000UL               /*!< Write protection of Page24 & 25 */
#define LL_FLASH_OB_WRP_PAGE_26_27  0x00002000UL               /*!< Write protection of Page26 & 27 */
#define LL_FLASH_OB_WRP_PAGE_28_29  0x00004000UL               /*!< Write protection of Page28 & 29 */
#define LL_FLASH_OB_WRP_PAGE_30_31  0x00008000UL               /*!< Write protection of Page30 & 31 */
#define LL_FLASH_OB_WRP_PAGE_32_33  0x00010000UL               /*!< Write protection of Page32 & 33 */
#define LL_FLASH_OB_WRP_PAGE_34_35  0x00020000UL               /*!< Write protection of Page34 & 35 */
#define LL_FLASH_OB_WRP_PAGE_36_37  0x00040000UL               /*!< Write protection of Page36 & 37 */
#define LL_FLASH_OB_WRP_PAGE_38_39  0x00080000UL               /*!< Write protection of Page38 & 39 */
#define LL_FLASH_OB_WRP_PAGE_40_41  0x00100000UL               /*!< Write protection of Page40 & 41 */
#define LL_FLASH_OB_WRP_PAGE_42_43  0x00200000UL               /*!< Write protection of Page42 & 43 */
#define LL_FLASH_OB_WRP_PAGE_44_45  0x00400000UL               /*!< Write protection of Page44 & 45 */
#define LL_FLASH_OB_WRP_PAGE_46_47  0x00800000UL               /*!< Write protection of Page46 & 47 */
#define LL_FLASH_OB_WRP_PAGE_48_49  0x01000000UL               /*!< Write protection of Page48 & 49 */
#define LL_FLASH_OB_WRP_PAGE_50_51  0x02000000UL               /*!< Write protection of Page50 & 51 */
#define LL_FLASH_OB_WRP_PAGE_52_53  0x04000000UL               /*!< Write protection of Page52 & 53 */
#define LL_FLASH_OB_WRP_PAGE_54_55  0x08000000UL               /*!< Write protection of Page54 & 55 */
#define LL_FLASH_OB_WRP_PAGE_56_57  0x10000000UL               /*!< Write protection of Page56 & 57 */
#define LL_FLASH_OB_WRP_PAGE_58_59  0x20000000UL               /*!< Write protection of Page58 & 59 */
#define LL_FLASH_OB_WRP_PAGE_60_61  0x40000000UL               /*!< Write protection of Page60 & 61 */
#define LL_FLASH_OB_WRP_PAGE_62_63  0x80000000UL               /*!< Write protection of Page62 & 63 */
#else
#define LL_FLASH_OB_WRP_PAGE_0      0x00000001UL               /*!< Write protection of Page0  */
#define LL_FLASH_OB_WRP_PAGE_1      0x00000002UL               /*!< Write protection of Page1  */
#define LL_FLASH_OB_WRP_PAGE_2      0x00000004UL               /*!< Write protection of Page2  */
#define LL_FLASH_OB_WRP_PAGE_3      0x00000008UL               /*!< Write protection of Page3  */
#define LL_FLASH_OB_WRP_PAGE_4      0x00000010UL               /*!< Write protection of Page4  */
#define LL_FLASH_OB_WRP_PAGE_5      0x00000020UL               /*!< Write protection of Page5  */
#define LL_FLASH_OB_WRP_PAGE_6      0x00000040UL               /*!< Write protection of Page6  */
#define LL_FLASH_OB_WRP_PAGE_7      0x00000080UL               /*!< Write protection of Page7  */
#define LL_FLASH_OB_WRP_PAGE_8      0x00000100UL               /*!< Write protection of Page8  */
#define LL_FLASH_OB_WRP_PAGE_9      0x00000200UL               /*!< Write protection of Page9  */
#define LL_FLASH_OB_WRP_PAGE_10     0x00000400UL               /*!< Write protection of Page10 */
#define LL_FLASH_OB_WRP_PAGE_11     0x00000800UL               /*!< Write protection of Page11 */
#define LL_FLASH_OB_WRP_PAGE_12     0x00001000UL               /*!< Write protection of Page12 */
#define LL_FLASH_OB_WRP_PAGE_13     0x00002000UL               /*!< Write protection of Page13 */
#define LL_FLASH_OB_WRP_PAGE_14     0x00004000UL               /*!< Write protection of Page14 */
#define LL_FLASH_OB_WRP_PAGE_15     0x00008000UL               /*!< Write protection of Page15 */
#define LL_FLASH_OB_WRP_PAGE_16     0x00010000UL               /*!< Write protection of Page16 */
#define LL_FLASH_OB_WRP_PAGE_17     0x00020000UL               /*!< Write protection of Page17 */
#define LL_FLASH_OB_WRP_PAGE_18     0x00040000UL               /*!< Write protection of Page18 */
#define LL_FLASH_OB_WRP_PAGE_19     0x00080000UL               /*!< Write protection of Page19 */
#define LL_FLASH_OB_WRP_PAGE_20     0x00100000UL               /*!< Write protection of Page20 */
#define LL_FLASH_OB_WRP_PAGE_21     0x00200000UL               /*!< Write protection of Page21 */
#define LL_FLASH_OB_WRP_PAGE_22     0x00400000UL               /*!< Write protection of Page22 */
#define LL_FLASH_OB_WRP_PAGE_23     0x00800000UL               /*!< Write protection of Page23 */
#define LL_FLASH_OB_WRP_PAGE_24     0x01000000UL               /*!< Write protection of Page24 */
#define LL_FLASH_OB_WRP_PAGE_25     0x02000000UL               /*!< Write protection of Page25 */
#define LL_FLASH_OB_WRP_PAGE_26     0x04000000UL               /*!< Write protection of Page26 */
#define LL_FLASH_OB_WRP_PAGE_27     0x08000000UL               /*!< Write protection of Page27 */
#define LL_FLASH_OB_WRP_PAGE_28     0x10000000UL               /*!< Write protection of Page28 */
#define LL_FLASH_OB_WRP_PAGE_29     0x20000000UL               /*!< Write protection of Page29 */
#define LL_FLASH_OB_WRP_PAGE_30     0x40000000UL               /*!< Write protection of Page30 */
#define LL_FLASH_OB_WRP_PAGE_31     0x80000000UL               /*!< Write protection of Page31 */
#endif /* (FLASH_WRP_GROUP_WIDTH == 2U) */
#define LL_FLASH_OB_WRP_PAGE_ALL    FLASH_WRP1R_PRG_WRPSG1_Msk /*!< Write protection of all Pages */
/**
  * @}
  */

/** @defgroup LL_FLASH_OB_OTP_Lock_Blocks FLASH OTP Lock Blocks
  * @{
  */
#define LL_FLASH_OB_OTP_BLK_0       0x00000001UL                /*!< OTP Lock Block0  */
#define LL_FLASH_OB_OTP_BLK_1       0x00000002UL                /*!< OTP Lock Block1  */
#define LL_FLASH_OB_OTP_BLK_2       0x00000004UL                /*!< OTP Lock Block2  */
#define LL_FLASH_OB_OTP_BLK_3       0x00000008UL                /*!< OTP Lock Block3  */
#define LL_FLASH_OB_OTP_BLK_4       0x00000010UL                /*!< OTP Lock Block4  */
#define LL_FLASH_OB_OTP_BLK_5       0x00000020UL                /*!< OTP Lock Block5  */
#define LL_FLASH_OB_OTP_BLK_6       0x00000040UL                /*!< OTP Lock Block6  */
#define LL_FLASH_OB_OTP_BLK_7       0x00000080UL                /*!< OTP Lock Block7  */
#define LL_FLASH_OB_OTP_BLK_8       0x00000100UL                /*!< OTP Lock Block8  */
#define LL_FLASH_OB_OTP_BLK_9       0x00000200UL                /*!< OTP Lock Block9  */
#define LL_FLASH_OB_OTP_BLK_10      0x00000400UL                /*!< OTP Lock Block10 */
#define LL_FLASH_OB_OTP_BLK_11      0x00000800UL                /*!< OTP Lock Block11 */
#define LL_FLASH_OB_OTP_BLK_12      0x00001000UL                /*!< OTP Lock Block12 */
#define LL_FLASH_OB_OTP_BLK_13      0x00002000UL                /*!< OTP Lock Block13 */
#define LL_FLASH_OB_OTP_BLK_14      0x00004000UL                /*!< OTP Lock Block14 */
#define LL_FLASH_OB_OTP_BLK_15      0x00008000UL                /*!< OTP Lock Block15 */
#define LL_FLASH_OB_OTP_BLK_16      0x00010000UL                /*!< OTP Lock Block16 */
#define LL_FLASH_OB_OTP_BLK_17      0x00020000UL                /*!< OTP Lock Block17 */
#define LL_FLASH_OB_OTP_BLK_18      0x00040000UL                /*!< OTP Lock Block18 */
#define LL_FLASH_OB_OTP_BLK_19      0x00080000UL                /*!< OTP Lock Block19 */
#define LL_FLASH_OB_OTP_BLK_20      0x00100000UL                /*!< OTP Lock Block20 */
#define LL_FLASH_OB_OTP_BLK_21      0x00200000UL                /*!< OTP Lock Block21 */
#define LL_FLASH_OB_OTP_BLK_22      0x00400000UL                /*!< OTP Lock Block22 */
#define LL_FLASH_OB_OTP_BLK_23      0x00800000UL                /*!< OTP Lock Block23 */
#define LL_FLASH_OB_OTP_BLK_ALL     FLASH_OTPBLR_PRG_LOCKBL_Msk /*!< OTP Lock All Blocks */
/**
  * @}
  */

/** @defgroup FLASH_Interrupted_Operation_Code FLASH Interrupted Operation Code
  * @{
  */
#define LL_FLASH_INTERRUPTED_NO_OPERATION 0x00000000U                                   /*!< No operation interrupted */
#define LL_FLASH_INTERRUPTED_SINGLE_WRITE FLASH_OPSR_CODE_OP_0                          /*!< Single write interrupted */
#define LL_FLASH_INTERRUPTED_PAGE_ERASE   (FLASH_OPSR_CODE_OP_1 | FLASH_OPSR_CODE_OP_0) /*!< Page erase interrupted   */
#define LL_FLASH_INTERRUPTED_BANK_ERASE   FLASH_OPSR_CODE_OP_2                          /*!< Bank erase interrupted   */
#define LL_FLASH_INTERRUPTED_MASS_ERASE   (FLASH_OPSR_CODE_OP_2 | FLASH_OPSR_CODE_OP_0) /*!< Mass erase interrupted   */
#define LL_FLASH_INTERRUPTED_OB_CHANGE    (FLASH_OPSR_CODE_OP_2 | FLASH_OPSR_CODE_OP_1) /*!< OB change interrupted    */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ----------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_LL_Exported_Macros LL FLASH Macros
  * @{
  */

/**
  * @brief  Write a value in FLASH register.
  * @param  instance FLASH instance.
  * @param  reg      Register to be written.
  * @param  value    Value to be written in the register.
  */
#define LL_FLASH_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in FLASH register.
  * @param  instance FLASH instance.
  * @param  reg      Register to be read
  * @retval Register value
  */
#define LL_FLASH_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @defgroup FLASH_LL_Exported_Functions LL FLASH Functions
  * @{
  */

/** @defgroup FLASH_LL_EF_Configuration LL FLASH Configuration
  * @{
  */

/**
  * @brief  Set Flash latency.
  * @rmtoll
  *  ACR           LATENCY           LL_FLASH_SetLatency
  * @param  flashx FLASH instance.
  * @param  latency
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0WS
  *         @arg @ref LL_FLASH_LATENCY_1WS
  *         @arg @ref LL_FLASH_LATENCY_2WS
  *         @arg @ref LL_FLASH_LATENCY_3WS
  *         @arg @ref LL_FLASH_LATENCY_4WS
  *         @arg @ref LL_FLASH_LATENCY_5WS
  *         @arg @ref LL_FLASH_LATENCY_6WS
  *         @arg @ref LL_FLASH_LATENCY_7WS
  *         @arg @ref LL_FLASH_LATENCY_8WS
  *         @arg @ref LL_FLASH_LATENCY_9WS
  *         @arg @ref LL_FLASH_LATENCY_10WS
  *         @arg @ref LL_FLASH_LATENCY_11WS
  *         @arg @ref LL_FLASH_LATENCY_12WS
  *         @arg @ref LL_FLASH_LATENCY_13WS
  *         @arg @ref LL_FLASH_LATENCY_14WS
  *         @arg @ref LL_FLASH_LATENCY_15WS
  */
__STATIC_INLINE void LL_FLASH_SetLatency(FLASH_TypeDef *flashx, uint32_t latency)
{
  STM32_MODIFY_REG(flashx->ACR, FLASH_ACR_LATENCY, latency);
}

/**
  * @brief  Get Flash latency.
  * @rmtoll
  *  ACR           LATENCY           LL_FLASH_GetLatency
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0WS
  *         @arg @ref LL_FLASH_LATENCY_1WS
  *         @arg @ref LL_FLASH_LATENCY_2WS
  *         @arg @ref LL_FLASH_LATENCY_3WS
  *         @arg @ref LL_FLASH_LATENCY_4WS
  *         @arg @ref LL_FLASH_LATENCY_5WS
  *         @arg @ref LL_FLASH_LATENCY_6WS
  *         @arg @ref LL_FLASH_LATENCY_7WS
  *         @arg @ref LL_FLASH_LATENCY_8WS
  *         @arg @ref LL_FLASH_LATENCY_9WS
  *         @arg @ref LL_FLASH_LATENCY_10WS
  *         @arg @ref LL_FLASH_LATENCY_11WS
  *         @arg @ref LL_FLASH_LATENCY_12WS
  *         @arg @ref LL_FLASH_LATENCY_13WS
  *         @arg @ref LL_FLASH_LATENCY_14WS
  *         @arg @ref LL_FLASH_LATENCY_15WS
  */
__STATIC_INLINE uint32_t LL_FLASH_GetLatency(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ACR, FLASH_ACR_LATENCY));
}

/**
  * @brief  Set Flash programming delay.
  * @rmtoll
  *  ACR           WRHIGHFREQ           LL_FLASH_SetProgrammingDelay
  * @param  flashx FLASH instance.
  * @param  delay
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_0
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_1
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_2
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_3
  */
__STATIC_INLINE void LL_FLASH_SetProgrammingDelay(FLASH_TypeDef *flashx, uint32_t delay)
{
  STM32_MODIFY_REG(flashx->ACR, FLASH_ACR_WRHIGHFREQ, delay);
}

/**
  * @brief  Get Flash programming delay.
  * @rmtoll
  *  ACR           WRHIGHFREQ           LL_FLASH_GetProgrammingDelay
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_0
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_1
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_2
  *         @arg @ref LL_FLASH_PROGRAM_DELAY_3
  */
__STATIC_INLINE uint32_t LL_FLASH_GetProgrammingDelay(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ACR, FLASH_ACR_WRHIGHFREQ));
}

/**
  * @brief  Enable flash prefetch.
  * @rmtoll
  *  ACR           PRFTEN           LL_FLASH_EnablePrefetch
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnablePrefetch(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->ACR, FLASH_ACR_PRFTEN);
}

/**
  * @brief  Disable flash prefetch.
  * @rmtoll
  *  ACR           PRFTEN           LL_FLASH_DisablePrefetch
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisablePrefetch(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->ACR, FLASH_ACR_PRFTEN);
}

/**
  * @brief  Check if the flash prefetch is enabled or disabled.
  * @rmtoll
  *  ACR           PRFTEN           LL_FLASH_IsEnabledPrefetch
  * @param  flashx FLASH instance.
  * @retval State of flash prefetch (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledPrefetch(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ACR, FLASH_ACR_PRFTEN) == (FLASH_ACR_PRFTEN)) ? 1UL : 0UL);
}


/**
  * @brief  Set Flash boot location empty status.
  * @rmtoll
  *  ACR           EMPTY           LL_FLASH_SetEmptyBootLocation
  * @param  flashx FLASH instance.
  * @param  empty_status
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BOOT_LOCATION_PROGRAMMED
  *         @arg @ref LL_FLASH_BOOT_LOCATION_EMPTY
  */
__STATIC_INLINE void LL_FLASH_SetEmptyBootLocation(FLASH_TypeDef *flashx, uint32_t empty_status)
{
  STM32_MODIFY_REG(flashx->ACR, FLASH_ACR_EMPTY, empty_status);
}

/**
  * @brief  Get Flash boot location empty status.
  * @rmtoll
  *  ACR           EMPTY           LL_FLASH_GetEmptyBootLocation
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_BOOT_LOCATION_PROGRAMMED
  *         @arg @ref LL_FLASH_BOOT_LOCATION_EMPTY
  */
__STATIC_INLINE uint32_t LL_FLASH_GetEmptyBootLocation(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ACR, FLASH_ACR_EMPTY));
}

/**
  * @brief  Set the flash unlock key.
  * @rmtoll
  *  KEYR         KEY/SECKEY           LL_FLASH_SetUnlockKey
  * @param  flashx FLASH instance.
  * @param  key
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_KEY1
  *         @arg @ref LL_FLASH_KEY2
  */
__STATIC_INLINE void LL_FLASH_SetUnlockKey(FLASH_TypeDef *flashx, uint32_t key)
{
  STM32_WRITE_REG(flashx->KEYR, key);
}

/**
  * @brief  Set the flash option bytes unlock key.
  * @rmtoll
  *  OPTKEYR           OPTKEY           LL_FLASH_OB_SetUnlockKey
  * @param  flashx FLASH instance.
  * @param  key
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_OPTKEY1
  *         @arg @ref LL_FLASH_OB_OPTKEY2
  */
__STATIC_INLINE void LL_FLASH_OB_SetUnlockKey(FLASH_TypeDef *flashx, uint32_t key)
{
  STM32_WRITE_REG(flashx->OPTKEYR, key);
}

/**
  * @brief  Read the state of selected operation status flags.
  * @rmtoll
  *  OPSR     DATA_OP|BK_OP|OTP_OP     LL_FLASH_ReadFlag_OP
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_DATA_OP
  *         @arg @ref LL_FLASH_FLAG_BK_OP
  *         @arg @ref LL_FLASH_FLAG_OTP_OP
  *         @arg @ref LL_FLASH_FLAG_OP_AREA_ALL
  * @retval Returned value : state of selected operation status flags.
  */
__STATIC_INLINE uint32_t LL_FLASH_ReadFlag_OP(const FLASH_TypeDef *flashx, const uint32_t flags)
{
  return (STM32_READ_BIT(flashx->OPSR, flags));
}

/**
  * @brief  Get the flash interrupted operation data flash flag.
  * @rmtoll
  *  OPSR           DATA_OP           LL_FLASH_IsActiveFlag_DATA_OP
  * @param  flashx FLASH instance.
  * @retval State of interrupted operation data flash flag (1: interrupt from data flash / 0: not from data flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_DATA_OP(const FLASH_TypeDef *flashx)
{
  return (((STM32_READ_BIT(flashx->OPSR, FLASH_OPSR_DATA_OP)) == (FLASH_OPSR_DATA_OP)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash interrupted operation bank.
  * @rmtoll
  *  OPSR           BK_OP           LL_FLASH_IsActiveFlag_BK_OP
  * @param  flashx FLASH instance.
  * @retval Returned value : the flash interrupted operation bank (0: bank1 / 1: bank2).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BK_OP(const FLASH_TypeDef *flashx)
{
  return (((STM32_READ_BIT(flashx->OPSR, FLASH_OPSR_BK_OP)) == (FLASH_OPSR_BK_OP)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash interrupted operation OTP flash flag.
  * @rmtoll
  *  OPSR           OTP_OP           LL_FLASH_IsActiveFlag_OTP_OP
  * @param  flashx FLASH instance.
  * @retval State of interrupted operation OTP flash flag (1: interrupt from OTP flash / 0: not from OTP flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_OTP_OP(const FLASH_TypeDef *flashx)
{
  return (((STM32_READ_BIT(flashx->OPSR, FLASH_OPSR_OTP_OP)) == (FLASH_OPSR_OTP_OP)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash interrupted operation address offset.
  * @rmtoll
  *  OPSR           ADDR_OP           LL_FLASH_GetOperInterruptedAddressOffset
  * @param  flashx FLASH instance.
  * @retval Returned value : the flash interrupted operation address offset.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetOperInterruptedAddressOffset(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPSR, FLASH_OPSR_ADDR_OP) >> FLASH_OPSR_ADDR_OP_Pos);
}

/**
  * @brief  Get the flash interrupted operation code.
  * @rmtoll
  *  OPSR           CODE_OP           LL_FLASH_GetOperInterruptedCode
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_INTERRUPTED_NO_OPERATION
  *         @arg @ref LL_FLASH_INTERRUPTED_SINGLE_WRITE
  *         @arg @ref LL_FLASH_INTERRUPTED_PAGE_ERASE
  *         @arg @ref LL_FLASH_INTERRUPTED_BANK_ERASE
  *         @arg @ref LL_FLASH_INTERRUPTED_MASS_ERASE
  *         @arg @ref LL_FLASH_INTERRUPTED_OB_CHANGE
  */
__STATIC_INLINE uint32_t LL_FLASH_GetOperInterruptedCode(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPSR, FLASH_OPSR_CODE_OP));
}

/**
  * @brief  Lock the flash option bytes control access registers.
  * @rmtoll
  *  OPTCR           OPTLOCK           LL_FLASH_OB_Lock
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_Lock(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTCR, FLASH_OPTCR_OPTLOCK);
}

/**
  * @brief  Check the flash option bytes control access registers lock state.
  * @rmtoll
  *  OPTCR           OPTLOCK           LL_FLASH_OB_IsLocked
  * @param  flashx FLASH instance.
  * @retval State of flash option bytes lock (1 locked / 0 unlocked).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsLocked(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTCR, FLASH_OPTCR_OPTLOCK) == (FLASH_OPTCR_OPTLOCK)) ? 1UL : 0UL);
}

/**
  * @brief  Start the flash option bytes modification.
  * @rmtoll
  *  OPTCR           OPTSTRT           LL_FLASH_OB_StartModification
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_StartModification(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTCR, FLASH_OPTCR_OPTSTRT);
}

/**
  * @brief  Check the flash option bytes control access registers bank swap state.
  * @rmtoll
  *  OPTCR           SWAP_BANK           LL_FLASH_OB_IsBankSwapped
  * @param  flashx FLASH instance.
  * @retval State of flash option bytes bank swap (1 swapped / 0 not swapped).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsBankSwapped(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTCR, FLASH_OPTCR_SWAP_BANK) == (FLASH_OPTCR_SWAP_BANK)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup FLASH_LL_EF_FLAG_Management LL FLASH Flag Management
  * @{
  */

/**
  * @brief  Check if any of the selected flag is active.
  * @rmtoll
  *  SR       BSY|WBNE|DBNE     LL_FLASH_IsActiveFlag
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_BSY
  *         @arg @ref LL_FLASH_FLAG_WBNE
  *         @arg @ref LL_FLASH_FLAG_DBNE
  *         @arg @ref LL_FLASH_FLAG_STATUS_ALL
  *         @arg @ref LL_FLASH_FLAG_EOP
  *         @arg @ref LL_FLASH_FLAG_WRPERR
  *         @arg @ref LL_FLASH_FLAG_PGSERR
  *         @arg @ref LL_FLASH_FLAG_STRBERR
  *         @arg @ref LL_FLASH_FLAG_INCERR
  *         @arg @ref LL_FLASH_FLAG_OPTCHANGEERR
  *         @arg @ref LL_FLASH_FLAG_ERRORS_ALL
  * @retval State of selected flag (1 if at least one is active / 0 if none of the selected flags are active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag(const FLASH_TypeDef *flashx, uint32_t flags)
{
  return ((STM32_READ_BIT(flashx->SR, flags) != 0U) ? 1UL : 0UL);
}

/**
  * @brief  Read the state of selected flash operation error flag.
  * @rmtoll
  *  SR      EOP/WRPERR/PGSERR/STRBERR/INCERR/OPTCHANGEERR      LL_FLASH_ReadFlag
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_BSY
  *         @arg @ref LL_FLASH_FLAG_WBNE
  *         @arg @ref LL_FLASH_FLAG_DBNE
  *         @arg @ref LL_FLASH_FLAG_STATUS_ALL
  *         @arg @ref LL_FLASH_FLAG_EOP
  *         @arg @ref LL_FLASH_FLAG_WRPERR
  *         @arg @ref LL_FLASH_FLAG_PGSERR
  *         @arg @ref LL_FLASH_FLAG_STRBERR
  *         @arg @ref LL_FLASH_FLAG_INCERR
  *         @arg @ref LL_FLASH_FLAG_OPTCHANGEERR
  *         @arg @ref LL_FLASH_FLAG_ERRORS_ALL
  * @retval State of selected flags (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_ReadFlag(const FLASH_TypeDef *flashx, uint32_t flags)
{
  return (STM32_READ_BIT(flashx->SR, flags));
}

/**
  * @brief  Clear the flash operation error flag.
  * @rmtoll
  *  SR      EOP/WRPERR/PGSERR/STRBERR/INCERR/OPTCHANGEERR      LL_FLASH_ClearFlag
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_EOP
  *         @arg @ref LL_FLASH_FLAG_WRPERR
  *         @arg @ref LL_FLASH_FLAG_PGSERR
  *         @arg @ref LL_FLASH_FLAG_STRBERR
  *         @arg @ref LL_FLASH_FLAG_INCERR
  *         @arg @ref LL_FLASH_FLAG_OPTCHANGEERR
  *         @arg @ref LL_FLASH_FLAG_ERRORS_ALL
  */
__STATIC_INLINE void LL_FLASH_ClearFlag(FLASH_TypeDef *flashx, uint32_t flags)
{
  STM32_SET_BIT(flashx->CCR, flags);
}

/**
  * @brief  Check if the flash busy flag is active.
  * @rmtoll
  *  SR           BSY           LL_FLASH_IsActiveFlag_BSY
  * @param  flashx FLASH instance.
  * @retval State of BSY flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BSY(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_BSY) == (FLASH_SR_BSY)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the write buffer not empty flag is active.
  * @rmtoll
  *  SR           WBNE           LL_FLASH_IsActiveFlag_WBNE
  * @param  flashx FLASH instance.
  * @retval State of WBNE flag (1 active (waiting for data to complete) / 0 not active (buffer empty or full)).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_WBNE(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_WBNE) == (FLASH_SR_WBNE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the data buffer not empty flag is active.
  * @rmtoll
  *  SR           DBNE           LL_FLASH_IsActiveFlag_DBNE
  * @param  flashx FLASH instance.
  * @retval State of DBNE flag (1 active (data buffer used, waiting) / 0 not active (buffer not used)).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_DBNE(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_DBNE) == (FLASH_SR_DBNE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the OEM key lock mechanism is active.
  * @rmtoll
  *  SR           OEMLOCK           LL_FLASH_IsActiveFlag_OEMLOCK
  * @param  flashx FLASH instance.
  * @retval State of OEMLOCK flag (1: OEM key locked / 0: OEM key is virgin).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_OEMLOCK(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_OEMLOCK) == (FLASH_SR_OEMLOCK)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the BS key lock mechanism is active.
  * @rmtoll
  *  SR           BSLOCK           LL_FLASH_IsActiveFlag_BSLOCK
  * @param  flashx FLASH instance.
  * @retval State of BSLOCK flag (1: BS key locked / 0: BS key is default).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BSLOCK(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_BSLOCK) == (FLASH_SR_BSLOCK)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the flash end of operation flag is active.
  * @rmtoll
  *  SR           EOP           LL_FLASH_IsActiveFlag_EOP
  * @param  flashx FLASH instance.
  * @retval State of EOP flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_EOP(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_EOP) == (FLASH_SR_EOP)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash end of operation error flag.
  * @rmtoll
  *  CCR        CLR_EOP           LL_FLASH_ClearFlag_EOP
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_EOP(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_EOP);
}

/**
  * @brief  Check if the flash write protection error flag is active.
  * @rmtoll
  *  SR           WRPERR           LL_FLASH_IsActiveFlag_WRPERR
  * @param  flashx FLASH instance.
  * @retval State of WRPERR flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_WRPERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_WRPERR) == (FLASH_SR_WRPERR)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash write protection error flag.
  * @rmtoll
  *  CCR        CLR_WRPERR           LL_FLASH_ClearFlag_WRPERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_WRPERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_WRPERR);
}

/**
  * @brief  Check if the flash programming sequence error flag is active.
  * @rmtoll
  *  SR           PGSERR           LL_FLASH_IsActiveFlag_PGSERR
  * @param  flashx FLASH instance.
  * @retval State of PGSERR flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_PGSERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_PGSERR) == (FLASH_SR_PGSERR)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash programming sequence error flag.
  * @rmtoll
  *  CCR        CLR_PGSERR           LL_FLASH_ClearFlag_PGSERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_PGSERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_PGSERR);
}

/**
  * @brief  Check if the flash strobe error flag is active.
  * @rmtoll
  *  SR           STRBERR           LL_FLASH_IsActiveFlag_STRBERR
  * @param  flashx FLASH instance.
  * @retval State of STRBERR flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_STRBERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_STRBERR) == (FLASH_SR_STRBERR)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash strobe error flag.
  * @rmtoll
  *  CCR        CLR_STRBERR           LL_FLASH_ClearFlag_STRBERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_STRBERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_STRBERR);
}

/**
  * @brief  Check if the flash inconsistency error flag is active.
  * @rmtoll
  *  SR           INCERR           LL_FLASH_IsActiveFlag_INCERR
  * @param  flashx FLASH instance.
  * @retval State of INCERR flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_INCERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_INCERR) == (FLASH_SR_INCERR)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash inconsistency error flag.
  * @rmtoll
  *  CCR        CLR_INCERR           LL_FLASH_ClearFlag_INCERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_INCERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_INCERR);
}
/**
  * @brief  Check if the flash option-byte change error flag is active.
  * @rmtoll
  *  SR           OPTCHANGEERR           LL_FLASH_IsActiveFlag_OPTCHANGEERR
  * @param  flashx FLASH instance.
  * @retval State of OPTCHANGEERR flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_OPTCHANGEERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->SR, FLASH_SR_OPTCHANGEERR) == (FLASH_SR_OPTCHANGEERR)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash option-byte change error flag.
  * @rmtoll
  *  CCR        CLR_OPTCHANGEERR           LL_FLASH_ClearFlag_OPTCHANGEERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_OPTCHANGEERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CCR, FLASH_CCR_CLR_OPTCHANGEERR);
}

/**
  * @}
  */

/** @defgroup FLASH_LL_EF_Control LL FLASH Operation and Control Functions
  * @{
  */

/**
  * @brief  Lock the flash control assess registers.
  * @rmtoll
  *  CR           LOCK           LL_FLASH_Lock
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_Lock(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_LOCK);
}

/**
  * @brief  Check the flash control access registers lock state.
  * @rmtoll
  *  CR           LOCK           LL_FLASH_IsLocked
  * @param  flashx FLASH instance.
  * @retval State of flash lock (1 locked / 0 unlocked).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsLocked(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_LOCK) == (FLASH_CR_LOCK)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash programming.
  * @rmtoll
  *  CR           PG           LL_FLASH_EnableProgramming
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableProgramming(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_PG);
}

/**
  * @brief  Disable the flash programming.
  * @rmtoll
  *  CR           PG           LL_FLASH_DisableProgramming
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableProgramming(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_PG);
}

/**
  * @brief  Check if the flash programming is enabled or disabled.
  * @rmtoll
  *  CR           PG           LL_FLASH_IsEnabledProgramming
  * @param  flashx FLASH instance.
  * @retval State of flash programming (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledProgramming(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_PG) == (FLASH_CR_PG)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash page erase.
  * @rmtoll
  *  CR           PER           LL_FLASH_EnablePageErase
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnablePageErase(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_PER);
}

/**
  * @brief  Disable the flash page erase.
  * @rmtoll
  *  CR           PER           LL_FLASH_DisablePageErase
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisablePageErase(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_PER);
}

/**
  * @brief  Check if the flash page erase is enabled or disabled.
  * @rmtoll
  *  CR           PER           LL_FLASH_IsEnabledPageErase
  * @param  flashx FLASH instance.
  * @retval State of flash page erase (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledPageErase(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_PER) == (FLASH_CR_PER)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash bank erase.
  * @rmtoll
  *  CR        BKSEL/BER           LL_FLASH_EnableBankErase
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  */
__STATIC_INLINE void LL_FLASH_EnableBankErase(FLASH_TypeDef *flashx, uint32_t bank)
{
  STM32_MODIFY_REG(flashx->CR, (FLASH_CR_BKSEL | FLASH_CR_BER), (bank | FLASH_CR_BER));
}

/**
  * @brief  Disable the flash bank erase.
  * @rmtoll
  *  CR           BKSEL/BER           LL_FLASH_DisableBankErase
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  */
__STATIC_INLINE void LL_FLASH_DisableBankErase(FLASH_TypeDef *flashx, uint32_t bank)
{
  STM32_CLEAR_BIT(flashx->CR, (bank | FLASH_CR_BER));
}

/**
  * @brief  Check if the flash bank erase is enabled or disabled.
  * @rmtoll
  *  CR           BKSEL/BER           LL_FLASH_IsEnabledBankErase
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  * @retval State of flash mass erase (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledBankErase(const FLASH_TypeDef *flashx, const uint32_t bank)
{
  return ((STM32_READ_BIT(flashx->CR, (FLASH_CR_BKSEL | FLASH_CR_BER)) == (bank | FLASH_CR_BER)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash mass erase.
  * @rmtoll
  *  CR        MER           LL_FLASH_EnableMassErase
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableMassErase(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_MER);
}

/**
  * @brief  Disable the flash mass erase.
  * @rmtoll
  *  CR        MER           LL_FLASH_DisableMassErase
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableMassErase(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_MER);
}

/**
  * @brief  Check if the flash mass erase is enabled or disabled.
  * @rmtoll
  *  CR           MER           LL_FLASH_IsEnabledMassErase
  * @param  flashx FLASH instance.
  * @retval State of flash mass erase (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledMassErase(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_MER) == (FLASH_CR_MER)) ? 1UL : 0UL);
}

/**
  * @brief  Disable all flash operation.
  * @rmtoll
  *  NSCR/SECCR           PG/PER/MER1/MER2           LL_FLASH_DisableAllOperation
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableAllOperation(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_PER | FLASH_CR_BER | FLASH_CR_MER | FLASH_CR_PG);
}


/**
  * @brief  Enable the flash force-write operation.
  * @rmtoll
  *  CR           FW           LL_FLASH_EnableForceWrite
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableForceWrite(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_FW);
}

/**
  * @brief  Start the flash erase operation.
  * @rmtoll
  *  CR           STRT           LL_FLASH_StartEraseOperation
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_StartEraseOperation(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_STRT);
}


/**
  * @brief  Set the flash page erase index.
  * @rmtoll
  *  CR           BKSEL/EDATASEL/PNB           LL_FLASH_SetPageEraseIndex
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  * @param  area
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_USER_AREA
  *         @arg @ref LL_FLASH_ERASE_EDATA_AREA
  * @param  page This parameter can take any value in [0:31]
  */
__STATIC_INLINE void LL_FLASH_SetPageEraseIndex(FLASH_TypeDef *flashx, uint32_t bank, uint32_t area, uint32_t page)
{
  STM32_MODIFY_REG(flashx->CR, (FLASH_CR_BKSEL | FLASH_CR_EDATASEL | FLASH_CR_PNB), \
                   (bank | area | (page << FLASH_CR_PNB_Pos)));
}

/**
  * @brief  Start the flash erase page.
  * @rmtoll
  *  CR           BKER/EDATASEL/PNB/PER/STRT           LL_FLASH_StartErasePage
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  * @param  area
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_USER_AREA
  *         @arg @ref LL_FLASH_ERASE_EDATA_AREA
  * @param  page This parameter can take any value in [0:31]
  */
__STATIC_INLINE void LL_FLASH_StartErasePage(FLASH_TypeDef *flashx, uint32_t bank, uint32_t area, uint32_t page)
{
  STM32_MODIFY_REG(flashx->CR, (FLASH_CR_BKSEL | FLASH_CR_EDATASEL | FLASH_CR_PNB | FLASH_CR_STRT | FLASH_CR_PER), \
                   (bank | area | (page << FLASH_CR_PNB_Pos) | FLASH_CR_STRT | FLASH_CR_PER));
}

/**
  * @brief  Start the flash erase bank.
  * @rmtoll
  *  CR           BKSEL/STRT/BER           LL_FLASH_StartEraseBank
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values :
  *         @arg @ref LL_FLASH_ERASE_BANK_1
  *         @arg @ref LL_FLASH_ERASE_BANK_2
  */
__STATIC_INLINE void LL_FLASH_StartEraseBank(FLASH_TypeDef *flashx, uint32_t bank)
{
  STM32_MODIFY_REG(flashx->CR, (FLASH_CR_BKSEL | FLASH_CR_BER | FLASH_CR_STRT), (bank | FLASH_CR_BER | FLASH_CR_STRT));
}

/**
  * @brief  Start the flash erase bank.
  * @rmtoll
  *  CR           BER/STRT           LL_FLASH_StartMassErase
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_StartMassErase(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, (FLASH_CR_MER | FLASH_CR_STRT));
}

/**
  * @brief  Enable the flash interrupt.
  * @rmtoll
  *  CR          EOPIE/WRPERRIE/PGSERRIE/STRBERRIE/INCERRIE/OPTCHANGEERRIE     LL_FLASH_EnableIT
  * @param  flashx FLASH instance.
  * @param  interrupt
  *         This parameter can be a combination of the following values:
  *         @arg @ref LL_FLASH_IT_EOP
  *         @arg @ref LL_FLASH_IT_WRPERR
  *         @arg @ref LL_FLASH_IT_PGSERR
  *         @arg @ref LL_FLASH_IT_STRBERR
  *         @arg @ref LL_FLASH_IT_INCERR
  *         @arg @ref LL_FLASH_IT_OPTCHANGEERR
  *         @arg @ref LL_FLASH_IT_ERRORS_ALL
  *         @arg @ref LL_FLASH_IT_ALL
  */
__STATIC_INLINE void LL_FLASH_EnableIT(FLASH_TypeDef *flashx, uint32_t interrupt)
{
  STM32_SET_BIT(flashx->CR, interrupt);
}

/**
  * @brief  Disable the flash interrupt.
  * @rmtoll
  *  CR          EOPIE/WRPERRIE/PGSERRIE/STRBERRIE/INCERRIE/OPTCHANGEERRIE    LL_FLASH_DisableIT
  * @param  flashx FLASH instance.
  * @param  interrupt
  *         This parameter can be a combination of the following values:
  *         @arg @ref LL_FLASH_IT_EOP
  *         @arg @ref LL_FLASH_IT_WRPERR
  *         @arg @ref LL_FLASH_IT_PGSERR
  *         @arg @ref LL_FLASH_IT_STRBERR
  *         @arg @ref LL_FLASH_IT_INCERR
  *         @arg @ref LL_FLASH_IT_OPTCHANGEERR
  *         @arg @ref LL_FLASH_IT_ERRORS_ALL
  *         @arg @ref LL_FLASH_IT_ALL
  */
__STATIC_INLINE void LL_FLASH_DisableIT(FLASH_TypeDef *flashx, uint32_t interrupt)
{
  STM32_CLEAR_BIT(flashx->CR, interrupt);
}

/**
  * @brief  Enable the flash end of operation interrupt.
  * @rmtoll
  *  CR          EOPIE        LL_FLASH_EnableIT_EOP
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_EOP(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_EOPIE);
}

/**
  * @brief  Disable the flash end of operation interrupt.
  * @rmtoll
  *  CR          EOPIE        LL_FLASH_DisableIT_EOP
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_EOP(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_EOPIE);
}

/**
  * @brief  Check if the flash end of operation interrupt is enabled or disabled.
  * @rmtoll
  *  CR          EOPIE        LL_FLASH_IsEnabledIT_EOP
  * @param  flashx FLASH instance.
  * @retval State of flash EOP interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_EOP(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_EOPIE) == (FLASH_CR_EOPIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash write protection error interrupt.
  * @rmtoll
  *  CR          WRPERRIE        LL_FLASH_EnableIT_WRPERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_WRPERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_WRPERRIE);
}

/**
  * @brief  Disable the flash write protection error interrupt.
  * @rmtoll
  *  CR          WRPERRIE        LL_FLASH_DisableIT_WRPERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_WRPERR(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_WRPERRIE);
}

/**
  * @brief  Check if the flash write protection error interrupt is enabled or disabled.
  * @rmtoll
  *  CR          WRPERRIE        LL_FLASH_IsEnabledIT_WRPERR
  * @param  flashx FLASH instance.
  * @retval State of flash WRPERR interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_WRPERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_WRPERRIE) == (FLASH_CR_WRPERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash programming sequence error interrupt.
  * @rmtoll
  *  CR          PGSERRIE        LL_FLASH_EnableIT_PGSERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_PGSERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_PGSERRIE);
}

/**
  * @brief  Disable the flash programming sequence error interrupt.
  * @rmtoll
  *  CR          PGSERRIE        LL_FLASH_DisableIT_PGSERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_PGSERR(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_PGSERRIE);
}

/**
  * @brief  Check if the flash programming sequence error interrupt is enabled or disabled.
  * @rmtoll
  *  CR          PGSERRIE        LL_FLASH_IsEnabledIT_PGSERR
  * @param  flashx FLASH instance.
  * @retval State of flash PGSERR interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_PGSERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_PGSERRIE) == (FLASH_CR_PGSERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash strobe error interrupt.
  * @rmtoll
  *  CR          STRBERRIE        LL_FLASH_EnableIT_STRBERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_STRBERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_STRBERRIE);
}

/**
  * @brief  Disable the flash strobe error interrupt.
  * @rmtoll
  *  CR          STRBERRIE        LL_FLASH_DisableIT_STRBERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_STRBERR(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_STRBERRIE);
}

/**
  * @brief  Check if the flash strobe error interrupt is enabled or disabled.
  * @rmtoll
  *  CR          STRBERRIE        LL_FLASH_IsEnabledIT_STRBERR
  * @param  flashx FLASH instance.
  * @retval State of flash STRBERR interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_STRBERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_STRBERRIE) == (FLASH_CR_STRBERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash inconsistency error interrupt.
  * @rmtoll
  *  CR          INCERRIE        LL_FLASH_EnableIT_INCERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_INCERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_INCERRIE);
}

/**
  * @brief  Disable the flash inconsistency error interrupt.
  * @rmtoll
  *  CR          INCERRIE        LL_FLASH_DisableIT_INCERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_INCERR(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_INCERRIE);
}

/**
  * @brief  Check if the flash inconsistency error interrupt is enabled or disabled.
  * @rmtoll
  *  CR          INCERRIE        LL_FLASH_IsEnabledIT_INCERR
  * @param  flashx FLASH instance.
  * @retval State of flash INCERR interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_INCERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_INCERRIE) == (FLASH_CR_INCERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the flash option-byte change error interrupt.
  * @rmtoll
  *  CR          OPTCHANGEERRIE        LL_FLASH_EnableIT_OPTCHANGEERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_OPTCHANGEERR(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->CR, FLASH_CR_OPTCHANGEERRIE);
}

/**
  * @brief  Disable the flash option-byte change error interrupt.
  * @rmtoll
  *  CR          OPTCHANGEERRIE        LL_FLASH_DisableIT_OPTCHANGEERR
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_OPTCHANGEERR(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->CR, FLASH_CR_OPTCHANGEERRIE);
}

/**
  * @brief  Check if the flash option-byte change error interrupt is enabled or disabled.
  * @rmtoll
  *  CR          OPTCHANGEERRIE        LL_FLASH_IsEnabledIT_OPTCHANGEERR
  * @param  flashx FLASH instance.
  * @retval State of flash OPTCHANGEERR interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_OPTCHANGEERR(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->CR, FLASH_CR_OPTCHANGEERRIE) == (FLASH_CR_OPTCHANGEERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Set the privilege attribute.
  * @rmtoll
  *  PRIVCFGR     PRIV        LL_FLASH_SetPrivAttr
  * @param  flashx    FLASH instance.
  * @param  item      The item attribute to be configured.
  *                   This parameter can be one of the following values:
  *                   @arg @ref LL_FLASH_PRIV_ITEM_ALL
  * @param  priv_attr This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_ATTR_NPRIV
  *         @arg @ref LL_FLASH_ATTR_PRIV
  * @note  This register can be written only when the access is privileged.
  */
__STATIC_INLINE void LL_FLASH_SetPrivAttr(FLASH_TypeDef *flashx, uint32_t item, uint32_t priv_attr)
{
  STM32_MODIFY_REG(flashx->PRIVCFGR, item, (item * priv_attr));
}

/**
  * @brief  Get the privilege attribute.
  * @rmtoll
  *  PRIVCFGR     PRIV        LL_FLASH_GetPrivAttr
  * @param  flashx FLASH instance.
  * @param  item   The item attribute to be queried.
  *                This parameter can be one of the following values:
  *                @arg @ref LL_FLASH_PRIV_ITEM_ALL
  * @return Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_ATTR_NPRIV
  *         @arg @ref LL_FLASH_ATTR_PRIV
  */
__STATIC_INLINE uint32_t LL_FLASH_GetPrivAttr(const FLASH_TypeDef *flashx, uint32_t item)
{
  return ((STM32_READ_BIT(flashx->PRIVCFGR, item) == item) ? LL_FLASH_ATTR_PRIV : LL_FLASH_ATTR_NPRIV);
}

/**
  * @brief  Set the flash page configuration for extended hide protection area by bank.
  * @rmtoll
  *  HDPEXTR       HDP1_EXT/HDP2_EXT          LL_FLASH_SetHDPExtArea
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_HDPEXT_BANK_1
  *         @arg @ref LL_FLASH_HDPEXT_BANK_2
  * @param  page_nbr
  *         This parameter can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE void LL_FLASH_SetHDPExtArea(FLASH_TypeDef *flashx, uint32_t bank, uint32_t page_nbr)
{
  STM32_MODIFY_REG(flashx->HDPEXTR, FLASH_HDPEXTR_HDP1_EXT_Msk << bank, page_nbr << bank);
}

/**
  * @brief  Get the flash page configuration for extended hide protection area by bank.
  * @rmtoll
  *  HDPEXTR       HDP1_EXT/HDP2_EXT          LL_FLASH_GetHDPExtArea
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_HDPEXT_BANK_1
  *         @arg @ref LL_FLASH_HDPEXT_BANK_2
  * @retval Returned value can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetHDPExtArea(const FLASH_TypeDef *flashx, const uint32_t bank)
{
  return (STM32_READ_BIT(flashx->HDPEXTR, FLASH_HDPEXTR_HDP1_EXT_Msk << bank) >> bank);
}

/**
  * @}
  */

/** @defgroup FLASH_LL_EF_OB_Management LL FLASH Option Bytes Management
  * @{
  */

/**
  * @brief  Set the flash independent watchdog selection.
  * @rmtoll
  *  OPTSR           IWDG_SW           LL_FLASH_OB_SetIWDGSelection
  * @param  flashx FLASH instance.
  * @param  hw_sw_selection
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_IWDG_HW
  *         @arg @ref LL_FLASH_OB_IWDG_SW
  */
__STATIC_INLINE void LL_FLASH_OB_SetIWDGSelection(FLASH_TypeDef *flashx, uint32_t hw_sw_selection)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_IWDG_SW, hw_sw_selection);
}

/**
  * @brief  Get the flash independent watchdog selection.
  * @rmtoll
  *  OPTSR           IWDG_SW           LL_FLASH_OB_GetIWDGSelection
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_IWDG_HW
  *         @arg @ref LL_FLASH_OB_IWDG_SW
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetIWDGSelection(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_IWDG_SW));
}

/**
  * @brief  Set the flash window watchdog selection.
  * @rmtoll
  *  OPTSR_PRG           WWDG_SW           LL_FLASH_OB_SetWWDGSelection
  * @param  flashx FLASH instance.
  * @param  hw_sw_selection
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_WWDG_HW
  *         @arg @ref LL_FLASH_OB_WWDG_SW
  */
__STATIC_INLINE void LL_FLASH_OB_SetWWDGSelection(FLASH_TypeDef *flashx, uint32_t hw_sw_selection)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_WWDG_SW, hw_sw_selection);
}

/**
  * @brief  Get the flash window watchdog selection.
  * @rmtoll
  *  OPTSR_CUR           WWDG_SW           LL_FLASH_OB_GetWWDGSelection
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_WWDG_HW
  *         @arg @ref LL_FLASH_OB_WWDG_SW
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetWWDGSelection(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_WWDG_SW));
}

/**
  * @brief  Set the flash reset generation in stop mode.
  * @rmtoll
  *  OPTSR_PRG           NRST_STOP           LL_FLASH_OB_SetNRSTStopMode
  * @param  flashx FLASH instance.
  * @param  rst_generation
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RST_STOP_MODE
  *         @arg @ref LL_FLASH_OB_NO_RST_STOP_MODE
  */
__STATIC_INLINE void LL_FLASH_OB_SetNRSTStopMode(FLASH_TypeDef *flashx, uint32_t rst_generation)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_NRST_STOP, rst_generation);
}

/**
  * @brief  Get the flash reset generation in stop mode.
  * @rmtoll
  *  OPTSR_CUR           NRST_STOP           LL_FLASH_OB_GetNRSTStopMode
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RST_STOP_MODE
  *         @arg @ref LL_FLASH_OB_NO_RST_STOP_MODE
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetNRSTStopMode(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_NRST_STOP));
}

/**
  * @brief  Set the flash reset generation in standby mode.
  * @rmtoll
  *  OPTSR_PRG           NRST_STDBY           LL_FLASH_OB_SetNRSTStandbyMode
  * @param  flashx FLASH instance.
  * @param  rst_generation
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RST_STDBY_MODE
  *         @arg @ref LL_FLASH_OB_NO_RST_STDBY_MODE
  */
__STATIC_INLINE void LL_FLASH_OB_SetNRSTStandbyMode(FLASH_TypeDef *flashx, uint32_t rst_generation)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_NRST_STDBY, rst_generation);
}

/**
  * @brief  Get the flash reset generation in standby mode.
  * @rmtoll
  *  OPTSR_CUR           NRST_STDBY           LL_FLASH_OB_GetNRSTStandbyMode
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RST_STDBY_MODE
  *         @arg @ref LL_FLASH_OB_NO_RST_STDBY_MODE
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetNRSTStandbyMode(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_NRST_STDBY));
}

/**
  * @brief Set the flash RDP level.
  * @rmtoll
  *  OPSR_PRG           RDP_LEVEL           LL_FLASH_OB_SetRDPLevel
  * @param  flashx FLASH instance.
  * @param  rdp_level
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_0
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_2_WBS
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_2
  */
__STATIC_INLINE void LL_FLASH_OB_SetRDPLevel(FLASH_TypeDef *flashx, uint32_t rdp_level)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_RDP_LEVEL, rdp_level << FLASH_OPTSR_PRG_RDP_LEVEL_Pos);
}

/**
  * @brief  Get the flash RDP level.
  * @rmtoll
  *  OPTSR_PGR           RDP_LEVEL           LL_FLASH_OB_GetRDPLevel
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_0
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_2_WBS
  *         @arg @ref LL_FLASH_OB_RDP_LEVEL_2
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetRDPLevel(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_RDP_LEVEL) >> FLASH_OPTSR_PRG_RDP_LEVEL_Pos);
}

/**
  * @brief  Freeze the flash independent watchdog counter in stop mode.
  * @rmtoll
  *  OPTSR_PRG           IWDG_STOP           LL_FLASH_OB_FreezeIWDGStopMode
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_FreezeIWDGStopMode(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_IWDG_STOP);
}

/**
  * @brief  Unfreeze the flash independent watchdog counter in stop mode.
  * @rmtoll
  *  OPTSR_PRG           IWDG_STOP           LL_FLASH_OB_UnfreezeIWDGStopMode
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_UnfreezeIWDGStopMode(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_IWDG_STOP);
}

/**
  * @brief  Check the flash independent watchdog counter in stop mode is frozen.
  * @rmtoll
  *  OPTSR_CUR           IWDG_STOP           LL_FLASH_OB_IsFrozenIWDGStopMode
  * @param  flashx FLASH instance.
  * @retval State of flash option bytes IWDG counter in stop mode (1 frozen / 0 not frozen).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsFrozenIWDGStopMode(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_IWDG_STOP) == (FLASH_OPTSR_PRG_IWDG_STOP)) ? 0UL : 1UL);
}

/**
  * @brief  Freeze the flash independent watchdog counter in standby mode.
  * @rmtoll
  *  OPTSR_PRG           IWDG_STDBY           LL_FLASH_OB_FreezeIWDGStandbyMode
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_FreezeIWDGStandbyMode(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_IWDG_STDBY);
}

/**
  * @brief  Unfreeze the flash independent watchdog counter in standby mode.
  * @rmtoll
  *  OPTSR_PRG           IWDG_STDBY           LL_FLASH_OB_UnfreezeIWDGStandbyMode
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_UnfreezeIWDGStandbyMode(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_IWDG_STDBY);
}

/**
  * @brief  Check the flash independent watchdog counter in standby mode is frozen.
  * @rmtoll
  *  OPTSR_CUR           IWDG_STDBY           LL_FLASH_OB_IsFrozenIWDGStandbyMode
  * @param  flashx FLASH instance.
  * @retval State of flash option bytes IWDG counter in standby mode (1 frozen / 0 not frozen).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsFrozenIWDGStandbyMode(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_IWDG_STDBY) == (FLASH_OPTSR_PRG_IWDG_STDBY)) ? 0UL : 1UL);
}

/**
  * @brief  Set the flash software boot0.
  * @rmtoll
  *  OPTSR_PRG           BOOT_SEL           LL_FLASH_OB_SetBoot0SourceSelection
  * @param  flashx FLASH instance.
  * @param  boot_sel
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOT0
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOTPIN
  */
__STATIC_INLINE void LL_FLASH_OB_SetBoot0SourceSelection(FLASH_TypeDef *flashx, uint32_t boot_sel)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_BOOT_SEL, boot_sel);
}

/**
  * @brief  Get the flash software boot0.
  * @rmtoll
  *  OPTSR_CUR           BOOT_SEL           LL_FLASH_OB_GetBoot0SourceSelection
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOT0
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOTPIN
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBoot0SourceSelection(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_BOOT_SEL));
}

/**
  * @brief  Set the flash BOOT0 option bit.
  * @rmtoll
  *  OPTSR_PRG           BOOT0           LL_FLASH_OB_SetBoot0
  * @param  flashx FLASH instance.
  * @param  boot0
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_DISABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_ENABLED
  */
__STATIC_INLINE void LL_FLASH_OB_SetBoot0(FLASH_TypeDef *flashx, uint32_t boot0)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_BOOT0, boot0);
}

/**
  * @brief  Get the flash BOOT0 option bit.
  * @rmtoll
  *  OPTSR_CUR           BOOT0           LL_FLASH_OB_GetBoot0
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_DISABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_ENABLED
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBoot0(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_BOOT0));
}

/**
  * @brief  Set the flash NBOOT0 option bit.
  * @rmtoll
  *  OPTSR_PRG           BOOT_SEL|BOOT0           LL_FLASH_OB_SetBoot0Config
  * @param  flashx FLASH instance.
  * @param  boot0
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_DISABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_ENABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOTPIN
  */
__STATIC_INLINE void LL_FLASH_OB_SetBoot0Config(FLASH_TypeDef *flashx, uint32_t boot0)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, (FLASH_OPTSR_PRG_BOOT_SEL | FLASH_OPTSR_PRG_BOOT0), boot0);
}

/**
  * @brief  Get the flash NBOOT0 option bit.
  * @rmtoll
  *  OPTSR_CUR           BOOT_SEL|BOOT0           LL_FLASH_OB_GetBoot0Config
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BOOT0_DISABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_ENABLED
  *         @arg @ref LL_FLASH_OB_BOOT0_BOOTPIN
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBoot0Config(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, (FLASH_OPTSR_PRG_BOOT_SEL | FLASH_OPTSR_PRG_BOOT0)));
}

/**
  * @brief  Enable the data flash area.
  * @rmtoll
  *  OPTSR_PRG    EDATA_EN   LL_FLASH_OB_EnableEDATAArea
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_EnableEDATAArea(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_EDATA_EN);
}

/**
  * @brief  Disable the data flash area.
  * @rmtoll
  *  OPTSR_PRG   EDATA_EN   LL_FLASH_OB_DisableEDATAArea
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_DisableEDATAArea(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_EDATA_EN);
}

/**
  * @brief  Check if the data flash area is enabled or disabled.
  * @rmtoll
  *  OPTSR_CUR   EDATA_EN   LL_FLASH_OB_IsEnabledEDATAArea
  * @param  flashx FLASH instance.
  * @retval State of flash option bit EDATA_EN (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsEnabledEDATAArea(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_EDATA_EN) == (FLASH_OPTSR_PRG_EDATA_EN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the single/dual bank configuration for products with less user memory.
  * @rmtoll
  *  OPTSR_PRG    SINGLE_BANK   LL_FLASH_OB_SetBank
  * @param  flashx FLASH instance.
  * @param  single_dual_bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_SINGLE_BANK
  *         @arg @ref LL_FLASH_OB_DUAL_BANK
  */
__STATIC_INLINE void LL_FLASH_OB_SetBank(FLASH_TypeDef *flashx, uint32_t single_dual_bank)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_SINGLE_BANK, single_dual_bank);
}

/**
  * @brief Get the single/dual bank configuration for products with less of user memory.
  * @rmtoll
  *  OPTSR_CUR   SINGLE_BANK   LL_FLASH_OB_GetBank
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_SINGLE_BANK
  *         @arg @ref LL_FLASH_OB_DUAL_BANK
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBank(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_SINGLE_BANK));
}

/**
  * @brief  Set the flash swap banks.
  * @rmtoll
  *  OPTSR_PRG           SWAP_BANK           LL_FLASH_OB_SetSwapBank
  * @param  flashx FLASH instance.
  * @param  swap_bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BANK_NOT_SWAPPED
  *         @arg @ref LL_FLASH_OB_BANK_SWAPPED
  */
__STATIC_INLINE void LL_FLASH_OB_SetSwapBank(FLASH_TypeDef *flashx, uint32_t swap_bank)
{
  STM32_MODIFY_REG(flashx->OPTSR_PRG, FLASH_OPTSR_PRG_SWAP_BANK, swap_bank);
}

/**
  * @brief  Get the flash swap banks.
  * @rmtoll
  *  OPTSR_CUR          SWAP_BANK        LL_FLASH_OB_GetSwapBank
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_BANK_NOT_SWAPPED
  *         @arg @ref LL_FLASH_OB_BANK_SWAPPED
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetSwapBank(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR_CUR, FLASH_OPTSR_PRG_SWAP_BANK));
}


/**
  * @brief  Set the flash erase SRAM1 upon system reset.
  * @rmtoll
  *  OPTSR2_PRG           SRAM1_RST           LL_FLASH_OB_SetSystemResetSRAM1Erase
  * @param  flashx FLASH instance.
  * @param  erase_sram
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_ERASED_SRAM1_SYS_RST
  *         @arg @ref LL_FLASH_OB_NOT_ERASED_SRAM1_SYS_RST
  */
__STATIC_INLINE void LL_FLASH_OB_SetSystemResetSRAM1Erase(FLASH_TypeDef *flashx, uint32_t erase_sram)
{
  STM32_MODIFY_REG(flashx->OPTSR2_PRG, FLASH_OPTSR2_PRG_SRAM1_RST, erase_sram);
}

/**
  * @brief  Get the flash erase SRAM1 upon system reset.
  * @rmtoll
  *  OPTSR2_CUR           SRAM1_RST           LL_FLASH_OB_GetSystemResetSRAM1Erase
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_ERASED_SRAM1_SYS_RST
  *         @arg @ref LL_FLASH_OB_NOT_ERASED_SRAM1_SYS_RST
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetSystemResetSRAM1Erase(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR2_CUR, FLASH_OPTSR2_PRG_SRAM1_RST));
}

/**
  * @brief  Enable the flash SRAM2 ECC.
  * @rmtoll
  *  OPTSR2_PRG           SRAM2_ECC           LL_FLASH_OB_EnableECCSRAM2
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_EnableECCSRAM2(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->OPTSR2_PRG, FLASH_OPTSR2_PRG_SRAM2_ECC);
}

/**
  * @brief  Disable the flash SRAM2 ECC.
  * @rmtoll
  *  OPTSR2_PRG           SRAM2_ECC           LL_FLASH_OB_DisableECCSRAM2
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_DisableECCSRAM2(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->OPTSR2_PRG, FLASH_OPTSR2_PRG_SRAM2_ECC);
}

/**
  * @brief  Check if the flash SRAM2 ECC is enabled or disabled.
  * @rmtoll
  *  OPTSR2_CUR           SRAM2_ECC           LL_FLASH_OB_IsEnabledECCSRAM2
  * @param  flashx FLASH instance.
  * @retval State of flash option bytes ECCSRAM2 (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsEnabledECCSRAM2(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->OPTSR2_CUR, FLASH_OPTSR2_PRG_SRAM2_ECC) == (FLASH_OPTSR2_PRG_SRAM2_ECC)) ? 0UL : 1UL);
}

/**
  * @brief  Set the flash SRAM2 erase upon system reset.
  * @rmtoll
  *  OPTSR2_PRG           SRAM2_RST           LL_FLASH_OB_SetSystemResetSRAM2Erase
  * @param  flashx FLASH instance.
  * @param  erase_sram2
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_OB_ERASED_SRAM2_SYS_RST
  *         @arg @ref LL_FLASH_OB_NOT_ERASED_SRAM2_SYS_RST
  */
__STATIC_INLINE void LL_FLASH_OB_SetSystemResetSRAM2Erase(FLASH_TypeDef *flashx, uint32_t erase_sram2)
{
  STM32_MODIFY_REG(flashx->OPTSR2_PRG, FLASH_OPTSR2_PRG_SRAM2_RST, erase_sram2);
}

/**
  * @brief  Get the flash SRAM2 erase upon system reset.
  * @rmtoll
  *  OPTSR2_CUR           SRAM2_RST           LL_FLASH_OB_GetSystemResetSRAM2Erase
  * @param  flashx FLASH instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_OB_ERASED_SRAM2_SYS_RST
  *         @arg @ref LL_FLASH_OB_NOT_ERASED_SRAM2_SYS_RST
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetSystemResetSRAM2Erase(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->OPTSR2_CUR, FLASH_OPTSR2_PRG_SRAM2_RST));
}

/**
  * @brief  Lock the flash boot address.
  * @rmtoll
  *  SECBOOTR          BOOT_LOCK          LL_FLASH_OB_LockBootConfig
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_LockBootConfig(FLASH_TypeDef *flashx)
{
  STM32_MODIFY_REG(flashx->BOOTR_PRG, FLASH_BOOTR_PRG_BOOT_LOCK, LL_FLASH_OB_BOOT_LOCKED);
}

/**
  * @brief  Unlock the flash boot address.
  * @rmtoll
  *  BOOTR          BOOT_LOCK          LL_FLASH_OB_UnlockBootConfig
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_OB_UnlockBootConfig(FLASH_TypeDef *flashx)
{
  STM32_MODIFY_REG(flashx->BOOTR_PRG, FLASH_BOOTR_PRG_BOOT_LOCK, LL_FLASH_OB_BOOT_NOT_LOCKED);
}

/**
  * @brief  Check the flash boot address lock state.
  * @rmtoll
  *  BOOTR          BOOT_LOCK          LL_FLASH_OB_IsLockedBootConfig
  * @param  flashx FLASH instance.
  * @retval State of boot address lock ( 1: locked, 0 unlocked)
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsLockedBootConfig(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->BOOTR_CUR, FLASH_BOOTR_PRG_BOOT_LOCK) == LL_FLASH_OB_BOOT_LOCKED) ? 1UL : 0UL);
}
/**
  * @brief  Set the flash boot base address.
  * @rmtoll
  *  BOOTR_PRG          BOOTADD          LL_FLASH_OB_SetBootAddr
  * @param  flashx FLASH instance.
  * @param  boot_addr
  *         This parameter can be a value between 0x00000000U and 0xFFFFFFFFU.
  */
__STATIC_INLINE void LL_FLASH_OB_SetBootAddr(FLASH_TypeDef *flashx, uint32_t boot_addr)
{
  STM32_MODIFY_REG(flashx->BOOTR_PRG, FLASH_BOOTR_PRG_BOOTADD, boot_addr);
}

/**
  * @brief  Get the flash boot base address.
  * @rmtoll
  *  BOOTR_CUR          BOOTADD          LL_FLASH_OB_GetBootAddr
  * @param  flashx FLASH instance.
  * @retval Returned value can be a value between 0x00000000U and 0xFFFFFFFFU.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBootAddr(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->BOOTR_CUR, FLASH_BOOTR_PRG_BOOTADD));
}

/**
  * @brief  Enable the flash OTP locking protection for the selected blocks.
  * @rmtoll
  *  OTPBLR_PRG          LOCKBL          LL_FLASH_OB_LockOTPBlock
  * @param  flashx FLASH instance.
  * @param  block specifies the OTP blocks to be locked.
  *         This parameter can be any combination of @ref LL_FLASH_OB_OTP_Lock_Blocks possible values.
  */
__STATIC_INLINE void LL_FLASH_OB_LockOTPBlock(FLASH_TypeDef *flashx, uint32_t block)
{
  STM32_SET_BIT(flashx->OTPBLR_PRG, (FLASH_OTPBLR_PRG_LOCKBL & block));
}

/**
  * @brief  Get the flash OTP locking protection by block.
  * @rmtoll
  *  OTPBLR_CUR          LOCKBL          LL_FLASH_OB_IsLockedOTPBlock
  * @param  flashx FLASH instance.
  * @param  block specifies the OTP blocks to check.
  *         This parameter can be any combination of @ref LL_FLASH_OB_OTP_Lock_Blocks possible values.
  * @retval 1 if all blocks specified by blocks are locked, 0 otherwise.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsLockedOTPBlock(const FLASH_TypeDef *flashx, const uint32_t block)
{
  const uint32_t masked_block = block & FLASH_OTPBLR_PRG_LOCKBL;
  return (((STM32_READ_BIT(flashx->OTPBLR_CUR, FLASH_OTPBLR_PRG_LOCKBL) & masked_block) == masked_block) ? 1UL : 0UL);
}

/**
  * @brief  Set the bootloader interface configuration.
  * @rmtoll
  *  BL_COM_CFGR         BL_COM_CFG          LL_FLASH_OB_SetBootloaderInterfaceConfig
  * @param  flashx FLASH instance.
  * @param  bootloader_config specifies the bootloader interface configuration to set.
  *         This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
  */
__STATIC_INLINE void LL_FLASH_OB_SetBootloaderInterfaceConfig(FLASH_TypeDef *flashx, uint32_t bootloader_config)
{
  STM32_WRITE_REG(flashx->BL_COM_CFG_PRG, bootloader_config);
}

/**
  * @brief  Get the bootloader interface configuration.
  * @rmtoll
  *  BL_COM_CFGR         BL_COM_CFG          LL_FLASH_OB_GetBootloaderInterfaceConfig
  * @param  flashx FLASH instance.
  * @retval Returned value can be a value between 0x00000000 and 0xFFFFFFFF.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetBootloaderInterfaceConfig(const FLASH_TypeDef *flashx)
{
  return STM32_READ_BIT(flashx->BL_COM_CFG_CUR, FLASH_BL_COM_CFG_PRG_BL_COM_CFG);
}

/**
  * @brief  Set the flash OEM key first word.
  * @rmtoll
  *  OEMKEYR1    OEMKEY   LL_FLASH_OB_SetOEMKeyWord1
  * @param  flashx FLASH instance.
  * @param  oem_key_word First 32-bit word of the 128-bit OEM key value to set
  */
__STATIC_INLINE void LL_FLASH_OB_SetOEMKeyWord1(FLASH_TypeDef *flashx, uint32_t oem_key_word)
{
  STM32_WRITE_REG(flashx->OEMKEYR1_PRG, oem_key_word);
}

/**
  * @brief  Set the flash OEM key second word.
  * @rmtoll
  *  OEMKEYR2    OEMKEY   LL_FLASH_OB_SetOEMKeyWord2
  * @param  flashx FLASH instance.
  * @param  oem_key_word Second 32-bit word of the 128-bit OEM key value to set
  */
__STATIC_INLINE void LL_FLASH_OB_SetOEMKeyWord2(FLASH_TypeDef *flashx, uint32_t oem_key_word)
{
  STM32_WRITE_REG(flashx->OEMKEYR2_PRG, oem_key_word);
}

/**
  * @brief  Set the flash OEM key third word.
  * @rmtoll
  *  OEMKEYR3    OEMKEY   LL_FLASH_OB_SetOEMKeyWord3
  * @param  flashx FLASH instance.
  * @param  oem_key_word Third 32-bit word of the 128-bit OEM key value to set
  */
__STATIC_INLINE void LL_FLASH_OB_SetOEMKeyWord3(FLASH_TypeDef *flashx, uint32_t oem_key_word)
{
  STM32_WRITE_REG(flashx->OEMKEYR3_PRG, oem_key_word);
}

/**
  * @brief  Set the flash OEM key second word.
  * @rmtoll
  *  OEMKEYR4    OEMKEY   LL_FLASH_OB_SetOEMKeyWord4
  * @param  flashx FLASH instance.
  * @param  oem_key_word Second 32-bit word of the 128-bit OEM key value to set
  */
__STATIC_INLINE void LL_FLASH_OB_SetOEMKeyWord4(FLASH_TypeDef *flashx, uint32_t oem_key_word)
{
  STM32_WRITE_REG(flashx->OEMKEYR4_PRG, oem_key_word);
}

/**
  * @brief  Set the flash BS key.
  * @rmtoll
  *  BSKEYR    BSKEY   LL_FLASH_OB_SetBSKey
  * @param  flashx FLASH instance.
  * @param  bs_key BS key value to set
  */
__STATIC_INLINE void LL_FLASH_OB_SetBSKey(FLASH_TypeDef *flashx, uint32_t bs_key)
{
  STM32_WRITE_REG(flashx->BSKEYR_PRG, bs_key);

}

/**
  * @brief  Enable the flash write protection on the selected pages of the selected bank.
  * @rmtoll
  *  WRP1R/WRP2R          WRP1R_WRPSG/WRP2R_WRPSG          LL_FLASH_OB_EnablePageWRP
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  page specifies the pages to be write protected.
  *         This parameter can be any combination of @ref LL_FLASH_OB_Write_Protection_Pages possible values.
  */
__STATIC_INLINE void LL_FLASH_OB_EnablePageWRP(FLASH_TypeDef *flashx, uint32_t bank, uint32_t page)
{
  __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->WRP1R_PRG : &flashx->WRP2R_PRG;
  STM32_CLEAR_BIT(*reg_addr, (FLASH_WRP1R_PRG_WRPSG1 & page));
}

/**
  * @brief  Disable the flash write protection on the selected pages of the selected bank.
  * @rmtoll
  *  WRP1R/WRP2R          WRP1R_WRPSG/WRP2R_WRPSG          LL_FLASH_OB_DisablePageWRP
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  page specifies the pages to be write protected.
  *         This parameter can be any combination of @ref LL_FLASH_OB_Write_Protection_Pages possible values.
  */
__STATIC_INLINE void LL_FLASH_OB_DisablePageWRP(FLASH_TypeDef *flashx, uint32_t bank, uint32_t page)
{
  __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->WRP1R_PRG : &flashx->WRP2R_PRG;
  STM32_SET_BIT(*reg_addr, (FLASH_WRP1R_PRG_WRPSG1 & page));
}

/**
  * @brief  Get the flash page write protection flags by bank.
  * @rmtoll
  *  WRP1R_PRG/WRP2R_PRG          WRP1R_WRPSG/WRP2R_WRPSG          LL_FLASH_OB_IsEnabledPageWRP
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  page specifies the pages to be write protected.
  *         This parameter can be any combination of @ref LL_FLASH_OB_Write_Protection_Pages possible values.
  * @retval 1 if all pages specified by the page parameter are write protected, 0 otherwise.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_IsEnabledPageWRP(const FLASH_TypeDef *flashx,
                                                      const uint32_t bank,
                                                      uint32_t page)
{
  const __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->WRP1R_CUR : &flashx->WRP2R_CUR;
  return ((((~(STM32_READ_BIT(*reg_addr, FLASH_WRP1R_PRG_WRPSG1))) & page) == page) ? 1UL : 0UL);
}

/**
  * @brief  Set the flash start page for hide protection area by bank.
  * @rmtoll
  *  HDP1R_PRG/HDP2R_PRG          HDP1R_STRT/HDP2R_STRT          LL_FLASH_OB_SetHDPAreaStartPage
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  page
  *         This parameter can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE void LL_FLASH_OB_SetHDPAreaStartPage(FLASH_TypeDef *flashx, uint32_t bank, uint32_t page)
{
  __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->HDP1R_PRG : &flashx->HDP2R_PRG;
  STM32_MODIFY_REG(*reg_addr, FLASH_HDP1R_PRG_HDP1_STRT, page << FLASH_HDP1R_PRG_HDP1_STRT_Pos);
}

/**
  * @brief  Get the flash start page for hide protection area by bank.
  * @rmtoll
  *  HDP1R_CUR/HDP2R_CUR          HDP1R_STRT/HDP2R_STRT          LL_FLASH_OB_GetHDPAreaStartPage
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @retval Returned value can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetHDPAreaStartPage(const FLASH_TypeDef *flashx, const uint32_t bank)
{
  const __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->HDP1R_CUR : &flashx->HDP2R_CUR;
  return (STM32_READ_BIT(*reg_addr, FLASH_HDP1R_PRG_HDP1_STRT) >> FLASH_HDP1R_PRG_HDP1_STRT_Pos);
}

/**
  * @brief  Set the flash end page for hide protection area by bank.
  * @rmtoll
  *  HDP1R_PRG/HDP2R_PRG          HDP1R_END/HDP2R_END          LL_FLASH_OB_SetHDPAreaEndPage
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  page
  *         This parameter can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE void LL_FLASH_OB_SetHDPAreaEndPage(FLASH_TypeDef *flashx, uint32_t bank, uint32_t page)
{
  __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->HDP1R_PRG : &flashx->HDP2R_PRG;
  STM32_MODIFY_REG(*reg_addr, FLASH_HDP1R_PRG_HDP1_END, page << FLASH_HDP1R_PRG_HDP1_END_Pos);
}

/**
  * @brief  Get the flash end page for hide protection area by bank.
  * @rmtoll
  *  HDP1R_CUR/HDP2R_CUR          HDP1R_END/HDP2R_END          LL_FLASH_OB_GetHDPAreaEndPage
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @retval Returned value can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE uint32_t LL_FLASH_OB_GetHDPAreaEndPage(const FLASH_TypeDef *flashx, const uint32_t bank)
{
  const __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->HDP1R_CUR : &flashx->HDP2R_CUR;
  return (STM32_READ_BIT(*reg_addr, FLASH_HDP1R_PRG_HDP1_END) >> FLASH_HDP1R_PRG_HDP1_END_Pos);
}

/**
  * @brief  Configure the flash hide protection area by bank.
  * @rmtoll
  *  HDP1R_PRG/HDP2R_PRG        HDP1R_END|HDP1R_STRT/HDP2R_END|HDP2R_STRT      LL_FLASH_OB_ConfigHDPArea
  * @param  flashx FLASH instance.
  * @param  bank
  *         This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_BANK_1
  *         @arg @ref LL_FLASH_BANK_2
  * @param  start_page
  *         This parameter can be a value between 0x00U and the maximum number of pages by bank.
  * @param  end_page
  *         This parameter can be a value between 0x00U and the maximum number of pages by bank.
  */
__STATIC_INLINE void LL_FLASH_OB_ConfigHDPArea(FLASH_TypeDef *flashx, uint32_t bank,
                                               uint32_t start_page,
                                               uint32_t end_page)
{
  __IO uint32_t *reg_addr = (bank == LL_FLASH_BANK_1) ? &flashx->HDP1R_PRG : &flashx->HDP2R_PRG;
  STM32_MODIFY_REG(*reg_addr, FLASH_HDP1R_PRG_HDP1_END | FLASH_HDP1R_PRG_HDP1_STRT, \
                   (end_page << FLASH_HDP1R_PRG_HDP1_END_Pos) + (start_page << FLASH_HDP1R_PRG_HDP1_STRT_Pos));
}

/**
  * @}
  */

/** @defgroup FLASH_LL_EF_ECC_Management LL FLASH ECC Management
  * @{
  */

/**
  * @brief  Get the flash ECC single-correction error address offset.
  * @rmtoll
  *  ECCCORR           ADDR_ECC           LL_FLASH_GetECCCAddressOffset
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC single-correction error address offset.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetECCCAddressOffset(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_ADDR_ECC) >> FLASH_ECCCORR_ADDR_ECC_Pos);
}

/**
  * @brief  Get the flash ECC single-correction error data flash flag.
  * @rmtoll
  *  ECCCORR           EDATA_ECC           LL_FLASH_IsActiveFlag_EDATA_ECCC
  * @param  flashx FLASH instance.
  * @retval State of ECC single-correction error data flash flag (1: ECC error in data flash / 0: not in data flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_EDATA_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_EDATA_ECC) == (FLASH_ECCCORR_EDATA_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash ECC single-correction error bank flag.
  * @rmtoll
  *  ECCCORR           BK_ECC           LL_FLASH_IsActiveFlag_BK_ECCC
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC error bank (1: ECC error in Bank 2 / 0: ECC error in Bank 1).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BK_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_BK_ECC) == (FLASH_ECCCORR_BK_ECC)) ? 1UL : 0UL);
}


/**
  * @brief  Get the flash ECC single-correction error system flash flag.
  * @rmtoll
  *  ECCCORR           SYSF_ECC           LL_FLASH_IsActiveFlag_SYSF_ECCC
  * @param  flashx FLASH instance.
  * @retval State of ECC single-correction error system flash flag
  *         (1: ECC error in system flash / 0: not in system flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_SYSF_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_SYSF_ECC) == (FLASH_ECCCORR_SYSF_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash ECC single-correction error OTP flash flag.
  * @rmtoll
  *  ECCCORR           OTP_ECC           LL_FLASH_IsActiveFlag_OTP_ECCC
  * @param  flashx FLASH instance.
  * @retval State of ECC single-correction error OTP flash flag (1: ECC error in OTP flash / 0: not in OTP flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_OTP_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_OTP_ECC) == (FLASH_ECCCORR_OTP_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Read the state of selected ECCC flags.
  * @rmtoll
  *  ECCCORR     EDATA_ECC|BK_ECC/SYSF_ECC|OTP_ECC      LL_FLASH_ReadFlag_ECCC
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_EDATA_ECC
  *         @arg @ref LL_FLASH_FLAG_BK_ECC
  *         @arg @ref LL_FLASH_FLAG_SYSF_ECC
  *         @arg @ref LL_FLASH_FLAG_OTP_ECC
  *         @arg @ref LL_FLASH_FLAG_ECC_AREA_ALL
  * @retval Returned value : state of ECCC selected flags.
  */
__STATIC_INLINE uint32_t LL_FLASH_ReadFlag_ECCC(const FLASH_TypeDef *flashx, const uint32_t flags)
{
  return (STM32_READ_BIT(flashx->ECCCORR, flags));
}

/**
  * @brief  Enable the flash ECC single-correction error interrupt.
  * @rmtoll
  *  ECCCORR          ECCCIE        LL_FLASH_EnableIT_ECCC
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_EnableIT_ECCC(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->ECCCORR, FLASH_ECCCORR_ECCCIE);
}

/**
  * @brief  Disable the flash ECC single-correction error interrupt.
  * @rmtoll
  *  ECCCORR          ECCCIE        LL_FLASH_DisableIT_ECCC
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_DisableIT_ECCC(FLASH_TypeDef *flashx)
{
  STM32_CLEAR_BIT(flashx->ECCCORR, FLASH_ECCCORR_ECCCIE);
}

/**
  * @brief  Check if the flash ECC single-correction error interrupt is enabled or disabled.
  * @rmtoll
  *  ECCCORR          ECCCIE        LL_FLASH_IsEnabledIT_ECCC
  * @param  flashx FLASH instance.
  * @retval State of flash EOP interruption (1 enabled / 0 disabled).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsEnabledIT_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_ECCCIE) == (FLASH_ECCCORR_ECCCIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the flash ECC single-correction error flag is active.
  * @rmtoll
  *  ECCCORR           ECCC           LL_FLASH_IsActiveFlag_ECCC
  * @param  flashx FLASH instance.
  * @retval State of flash ECCC flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_ECCC(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCCORR, FLASH_ECCCORR_ECCC) == (FLASH_ECCCORR_ECCC)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash ECC single-correction error flag.
  * @rmtoll
  *  ECCCORR           ECCC           LL_FLASH_ClearFlag_ECCC
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_ECCC(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->ECCCORR, FLASH_ECCCORR_ECCC);
}

/**
  * @brief  Get the flash ECC double error address offset.
  * @rmtoll
  *  ECCDETR           ADDR_ECC           LL_FLASH_GetECCDAddressOffset
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC double error address offset.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetECCDAddressOffset(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_ADDR_ECC) >> FLASH_ECCDETR_ADDR_ECC_Pos);
}

/**
  * @brief  Get the flash ECC double error data flash flag.
  * @rmtoll
  *  ECCDETR           EDATA_ECC           LL_FLASH_IsActiveFlag_EDATA_ECCD
  * @param  flashx FLASH instance.
  * @retval State of ECC double error data flash flag (1: ECC error in data flash / 0: not in data flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_EDATA_ECCD(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_EDATA_ECC) == (FLASH_ECCDETR_EDATA_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash ECC double error bank flag.
  * @rmtoll
  *  ECCDETR           BK_ECC           LL_FLASH_IsActiveFlag_BK_ECCD
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC error bank (1: ECC error in Bank 2 / 0: ECC error in Bank 1).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BK_ECCD(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_BK_ECC) == (FLASH_ECCDETR_BK_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash ECC double error system flash flag.
  * @rmtoll
  *  ECCDETR           SYSF_ECC           LL_FLASH_IsActiveFlag_SYSF_ECCD
  * @param  flashx FLASH instance.
  * @retval State of ECC double error system flash flag (1: ECC error in system flash / 0: not in system flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_SYSF_ECCD(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_SYSF_ECC) == (FLASH_ECCDETR_SYSF_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Get the flash ECC double error OTP flash flag.
  * @rmtoll
  *  ECCDETR           OTP_ECC           LL_FLASH_IsActiveFlag_OTP_ECCD
  * @param  flashx FLASH instance.
  * @retval State of ECC double error OTP flash flag (1: ECC error in OTP flash / 0: not in OTP flash).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_OTP_ECCD(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_OTP_ECC) == (FLASH_ECCDETR_OTP_ECC)) ? 1UL : 0UL);
}

/**
  * @brief  Read the state of selected ECCD flags.
  * @rmtoll
  *  ECCDETR     EDATA_ECC|BK_ECC/SYSF_ECC|OTP_ECC     LL_FLASH_ReadFlag_ECCD
  * @param  flashx FLASH instance.
  * @param  flags
  *         This parameter can one or a combination of the following values:
  *         @arg @ref LL_FLASH_FLAG_EDATA_ECC
  *         @arg @ref LL_FLASH_FLAG_BK_ECC
  *         @arg @ref LL_FLASH_FLAG_SYSF_ECC
  *         @arg @ref LL_FLASH_FLAG_OTP_ECC
  *         @arg @ref LL_FLASH_FLAG_ECC_AREA_ALL
  * @retval Returned value : state of ECCD selected flags.
  */
__STATIC_INLINE uint32_t LL_FLASH_ReadFlag_ECCD(const FLASH_TypeDef *flashx, const uint32_t flags)
{
  return (STM32_READ_BIT(flashx->ECCDETR, flags));
}


/**
  * @brief  Check if the flash ECC double error flag is active.
  * @rmtoll
  *  ECCDETR           ECCD           LL_FLASH_IsActiveFlag_ECCD
  * @param  flashx FLASH instance.
  * @retval State of flash ECCD flag (1 active / 0 not active).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_ECCD(const FLASH_TypeDef *flashx)
{
  return ((STM32_READ_BIT(flashx->ECCDETR, FLASH_ECCDETR_ECCD) == (FLASH_ECCDETR_ECCD)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flash ECC double error flag.
  * @rmtoll
  *  ECCDETR           ECCD           LL_FLASH_ClearFlag_ECCD
  * @param  flashx FLASH instance.
  */
__STATIC_INLINE void LL_FLASH_ClearFlag_ECCD(FLASH_TypeDef *flashx)
{
  STM32_SET_BIT(flashx->ECCDETR, FLASH_ECCDETR_ECCD);
}

/**
  * @brief  Get the flash ECC double error data.
  * @rmtoll
  *  ECCDR           DATA_ECC           LL_FLASH_GetECCDData
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC error data.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetECCDData(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ECCDR, FLASH_ECCDR_DATA_ECC) >> FLASH_ECCDR_DATA_ECC_Pos);
}

/**
  * @brief  Get the flash ECC double error 16-bit word number.
  * @rmtoll
  *  ECCDR         DATA_ADDR_ECC         LL_FLASH_GetECCDWordNumber
  * @param  flashx FLASH instance.
  * @retval Returned value : ECC error 16-bit word number.
  */
__STATIC_INLINE uint32_t LL_FLASH_GetECCDWordNumber(const FLASH_TypeDef *flashx)
{
  return (STM32_READ_BIT(flashx->ECCDR, FLASH_ECCDR_DATA_ADDR_ECC) >> FLASH_ECCDR_DATA_ADDR_ECC_Pos);
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

#endif /* FLASH */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_LL_FLASH_H */
