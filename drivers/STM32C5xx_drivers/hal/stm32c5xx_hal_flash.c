/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_flash.c
  * @brief   This file provides FLASH memory services.
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

/** @addtogroup FLASH
  * @{
  */
/** @defgroup FLASH_Introduction FLASH Introduction
  * @{

  The FLASH IO Hardware Abstraction Layer (HAL) provides high-level, user-friendly APIs for managing
  non-volatile memory (FLASH) operations on STM32 microcontrollers.

  This layer offers firmware functions to handle key FLASH functionalities, including:

    - Initialization and de-initialization of FLASH handling.
    - Programming operations across different FLASH memory areas.
    - Management of operation completion and error interrupt requests.
    - Retrieval of FLASH memory topology and configuration information.

  The abstraction provided by this layer ensures portability and simplifies application development
  across various STM32 microcontroller series.

  */
/**
  * @}
  */

/** @defgroup FLASH_How_To_Use FLASH How To Use
  * @{

The Flash memory interface manages CPU AHB C-Bus accesses to the Flash memory. It implements the erase and program Flash
memory operations and the read and write protection mechanisms.

# FLASH main features

The FLASH memory organization is based on a main area, an EDATA area and an information block.
  - The main flash memory block organized as two banks of up to 512 Kbytes each containing up to 64 pages of 8 Kbytes.
  - The EDATA memory block can be configured
    - to extend the user area, with two banks of 32 Kbytes each containing 16 pages of 2 Kbytes.
    - or to offer data flash area, with two banks of 24 Kbytes each containing 16 pages of 1.5 Kbytes.
  - The information block contains:
    - 32 Kbytes for system memory. It contains the bootloader that is used to reprogram the flash memory.
    - 32 Kbytes immutable secure area containing the root security services (RSS and RSS library).
    - 512 bytes OTP (one-time programmable). The OTP data cannot be erased and can be written only once.
    - Option bytes for user configuration. The FLASH_ITF module provides option bytes configuration functions.

## Programming operation
  - The flash memory program operation can be performed 128, 64, 32, 16 or 8 bits at a time within the flash user memory
    areas.
  - The flash memory program operation can be performed 32 or 16 bits at a time within the flash data and OTP memory
    areas.
  - Programming in a previously programmed address is not allowed except if the data to write is full zero.

## Erasing operation
  - The flash memory erase operation can be performed at page level.
  - The flash memory erase operation can be performed at bank level.
  - The flash memory erase operation can be performed on the whole flash memory.

# How to use the FLASH HAL module driver

## Initialization and De-initialization functions

- For a given instance, use the HAL_FLASH_Init() function to initialize the FLASH handle and associate the physical
  instance.
- Use the HAL_FLASH_DeInit() function to de-initialize FLASH. When called, the FLASH must be in reset.

## Configuration functions

- Use the HAL_FLASH_SetProgrammingMode() function to set the programming mode.
- Use the HAL_FLASH_GetProgrammingMode() function to get the programming mode.

## Process operation functions

### Polling mode operations
- Use the HAL_FLASH_ProgramByAddr() function to program by address any area of FLASH USER memory in polling mode.
- Use the HAL_FLASH_ProgramByAddrAdapt() function to program by address using adaptive width any area of FLASH USER
  memory in polling mode.
- Use the HAL_FLASH_OTP_ProgramByAddr() function to program by address any area of OTP area in polling mode.
- Use the HAL_FLASH_OTP_ProgramByAddrAdapt() function to program by address using adaptive width any area of OTP area in
  polling mode.
- Use the HAL_FLASH_EDATA_ProgramByAddr() function to program by address any area of FLASH EDATA memory in polling mode.
- Use the HAL_FLASH_EDATA_ProgramByAddrAdapt() function to program by address using adaptive width any area of FLASH
  EDATA memory in polling mode.
- Use the HAL_FLASH_EraseByAddr() function to erase by address any area of FLASH USER memory in polling mode.
- Use the HAL_FLASH_EDATA_EraseByAddr() function to erase by address any area of FLASH EDATA memory in polling mode.
- Use the HAL_FLASH_ErasePage() function to erase a specific set of pages of FLASH USER memory in polling mode.
- Use the HAL_FLASH_EDATA_ErasePage() function to erase a specific set of pages of FLASH EDATA memory in polling mode.
- Use the HAL_FLASH_EraseBank() function to erase a specific bank of FLASH memory (USER and EDATA) in polling mode.
- Use the HAL_FLASH_MassErase() function to erase all the FLASH memory (USER and EDATA) in polling mode.

### Interrupt mode operations
- Configure the FLASH interrupt priority using HAL_CORTEX_NVIC_SetPriority() function
- Enable the FLASH IRQ handler using HAL_CORTEX_NVIC_EnableIRQ() function

- Use the HAL_FLASH_ProgramByAddr_IT() function to program by address any area of FLASH USER memory in interrupt mode.
- Use the HAL_FLASH_ProgramByAddrAdapt_IT() function to program by address using adaptive width any area of FLASH USER
  memory in interrupt mode.
- Use the HAL_FLASH_OTP_ProgramByAddr_IT() function to program by address any area of OTP area in interrupt mode.
- Use the HAL_FLASH_OTP_ProgramByAddrAdapt_IT() function to program by address using adaptive width any area of OTP area
  in interrupt mode.
- Use the HAL_FLASH_EDATA_ProgramByAddr_IT() function to program by address any area of FLASH EDATA memory in interrupt
  mode.
- Use the HAL_FLASH_EDATA_ProgramByAddrAdapt_IT() function to program by address using adaptive width any area of FLASH
  EDATA memory in interrupt mode.
- Use the HAL_FLASH_EraseByAddr_IT() function to erase by address the FLASH USER memory in interrupt mode.
- Use the HAL_FLASH_EDATA_EraseByAddr_IT() function to erase by address the FLASH EDATA memory in interrupt mode.
- Use the HAL_FLASH_ErasePage_IT() function to erase by page the FLASH USER memory in interrupt mode.
- Use the HAL_FLASH_EDATA_ErasePage_IT() function to erase by page the FLASH EDATA memory in interrupt mode.
- Use the HAL_FLASH_EraseBank_IT() function to erase a bank of FLASH memory (USER and EDATA) in interrupt mode.
- Use the HAL_FLASH_MassErase_IT() function to erase all the FLASH memory (USER and EDATA) in interrupt mode.
- Use the HAL_FLASH_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle any FLASH
  interrupt.
- Use the HAL_FLASH_ProgramByAddr_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any FLASH program interrupt.
- Use the HAL_FLASH_EraseByAddr_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any FLASH erase by address interrupt.
- Use the HAL_FLASH_ErasePage_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any FLASH erase page interrupt.
- Use the HAL_FLASH_EraseBank_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any FLASH erase bank interrupt.
- Use the HAL_FLASH_MassErase_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any FLASH mass erase interrupt.
- Use the HAL_FLASH_ECC_IRQHandler() function called under FLASH_IRQHandler interrupt subroutine to handle
  any ECC single error interrupt.
- Use the HAL_FLASH_NMI_IRQHandler() function called under NMI_Handler interrupt subroutine to handle any NMI FLASH
  interrupt.

## Callback registration

When the compilation flag USE_HAL_FLASH_REGISTER_CALLBACKS is set to 1, the following functions allow to register
the different FLASH callbacks:
- Use the HAL_FLASH_RegisterProgramCpltCallback() function to register the FLASH program complete callback.
- Use the HAL_FLASH_RegisterEraseByAddrCpltCallback() function to register the FLASH erase by address complete callback.
- Use the HAL_FLASH_RegisterErasePageCpltCallback() function to register the FLASH erase by page complete callback.
- Use the HAL_FLASH_RegisterEraseBankCpltCallback() function to register the FLASH bank erase complete callback.
- Use the HAL_FLASH_RegisterMassEraseCpltCallback() function to register the FLASH mass erase complete callback.
- Use the HAL_FLASH_RegisterErrorCallback() function to register the FLASH error callback.
- Use the HAL_FLASH_RegisterECCErrorCallback() function to register the FLASH ECC error callback.

When the compilation flag USE_HAL_FLASH_REGISTER_CALLBACKS is undefined or set to 0, the callback registration feature
is not available and all callbacks are set to the following weak functions:
- The HAL_FLASH_ProgramCpltCallback() function for the program complete callback.
- The HAL_FLASH_EraseByAddrCpltCallback() function for the erase by address complete callback.
- The HAL_FLASH_ErasePageCpltCallback() function for the erase by page complete callback.
- The HAL_FLASH_EraseBankCpltCallback() function for the erase bank complete callback.
- The HAL_FLASH_MassEraseCpltCallback() function for the mass erase complete callback.
- The HAL_FLASH_ErrorCallback() function for the error callback.
- The HAL_FLASH_ECC_ErrorCallback() function for the ECC error callback.

These weak functions can be redefined within user application if the callbacks are needed.

## Status functions

- Use the HAL_FLASH_GetCurrentOperation() function to get the current flash operation.
- Use the HAL_FLASH_GetCurrentProgrammedAddr() function to get the currently being programmed flash address.
- Use the HAL_FLASH_GetCurrentErasedAddr() function to get the currently being erased flash address.
- Use the HAL_FLASH_GetCurrentErasedPage() function to get the currently being erased flash page.
- Use the HAL_FLASH_GetInterruptedByResetOperationInfo() function to get the interrupted flash operation information.
- Use the HAL_FLASH_GetInfo() function to get the FLASH information.
- Use the HAL_FLASH_GetSizeByte() function to get the FLASH total size.
- Use the HAL_FLASH_GetBankNbr() function to get the FLASH number of banks.
- Use the HAL_FLASH_GetBankSizeByte() function to get the FLASH bank size.
- Use the HAL_FLASH_GetUserFlashSizeByte() function to get the FLASH USER area size within a bank.
- Use the HAL_FLASH_EDATA_GetSizeByte() function to get the FLASH EDATA area size within a bank.
- Use the HAL_FLASH_GetExtUserFlashSizeByte() function to get the FLASH Extended USER area size within a bank.
- Use the HAL_FLASH_GetUserFlashPageNbr() function to get the number of pages in FLASH USER area.
- Use the HAL_FLASH_EDATA_GetPageNbr() function to get the number of pages in FLASH EDATA area.
- Use the HAL_FLASH_GetExtUserFlashPageNbr() function to get the number of pages in FLASH extended USER area.
- Use the HAL_FLASH_GetUserFlashPageSizeByte() function to get the size of a given page in FLASH USER area.
- Use the HAL_FLASH_EDATA_GetPageSizeByte() function to get the size of a given page in FLASH EDATA area.
- Use the HAL_FLASH_GetExtUserFlashPageSizeByte() function to get the size of a given page in FLASH extended USER area.
- Use the HAL_FLASH_GetUserFlashAddrOffset() function to get the address offset of a specific page in FLASH USER area.
- Use the HAL_FLASH_EDATA_GetAddrOffset() function to get the address offset of a specific page in FLASH EDATA area.
- Use the HAL_FLASH_GetExtUserFlashAddrOffset() function to get the address offset of a specific page in FLASH extended
  USER area. The offset is computed from the beginning of the FLASH extended USER area.
- Use the HAL_FLASH_ECC_GetInfo() function to get the FLASH ECC information.
- Use the HAL_FLASH_GetInstance() function to get the HAL FLASH instance.
- Use the HAL_FLASH_GetLLInstance() function to get the hardware FLASH instance.
- Use the HAL_FLASH_GetState() function to get the flash global state.
- Use the HAL_FLASH_GetLastErrorCodes() function to get the last error codes.
- Use the HAL_FLASH_SetUserData() function to set user data within the FLASH handle.
- Use the HAL_FLASH_GetUserData() function to get the user data from the FLASH handle.
  */
/**
  * @}
  */

/** @defgroup FLASH_Configuration_Table FLASH Configuration Table
  * @{
## Configuration inside the FLASH driver

Configuration defines           | Description   | Default value   | Note                                               |
--------------------------------| ------------- | --------------- | -------------------------------------------------- |
PRODUCT                         |from IDE       |        NA       | The selected device (ex STM32C5xx).                |
USE_HAL_FLASH_MODULE            |from hal_conf.h|        1        | Enables the HAL FLASH module.                      |
USE_ASSERT_DBG_PARAM            |from IDE       |      None       | Enables assert check parameters.                   |
USE_ASSERT_DBG_STATE            |from IDE       |      None       | Enables assert check states.                       |
USE_HAL_CHECK_PARAM             |from hal_conf.h|        0        | Enables run-time parameter checks.                 |
USE_HAL_CHECK_PROCESS_STATE     |from hal_conf.h|        0        | Enables load and store exclusive checks.           |
USE_HAL_FLASH_REGISTER_CALLBACKS|from hal_conf.h|        0        | Enables callback registration for FLASH.           |
USE_HAL_FLASH_CLK_ENABLE_MODEL  |from hal_conf.h|HAL_CLK_ENABLE_NO| Enables the clock model for the FLASH.             |
USE_HAL_FLASH_GET_LAST_ERRORS   |from hal_conf.h|        0        | Enables the error code mechanism.                  |
USE_HAL_FLASH_USER_DATA         |from hal_conf.h|        0        | Enables user data.                                 |
USE_HAL_FLASH_PROGRAM_BY_ADDR   |from hal_conf.h|        1        | Enables program by address feature APIs.           |
USE_HAL_FLASH_ERASE_BY_ADDR     |from hal_conf.h|        1        | Enables erase by address feature APIs.             |
USE_HAL_FLASH_ERASE_PAGE        |from hal_conf.h|        1        | Enables erase by page feature APIs.                |
USE_HAL_FLASH_ERASE_BANK        |from hal_conf.h|        1        | Enables erase by bank feature APIs.                |
USE_HAL_FLASH_MASS_ERASE        |from hal_conf.h|        1        | Enables mass erase feature APIs.                   |
USE_HAL_FLASH_ECC               |from hal_conf.h|        1        | Enables ECC error handling APIs.                   |
USE_HAL_FLASH_OB_EDATA          |from hal_conf.h|        0    | Enables EDATA APIs. Value must be equal to EDATA_EN OB.|
  */
/**
  * @}
  */

/* Private Constants -------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Private_Constants FLASH Private Constants
  * @{
  */

#define FLASH_OPERATION_MAX_TIMEOUT              1000U /*!< FLASH Operation maximum timeout in ms           */
#define FLASH_PROGRAM_OPERATION_QUADWORD_TIMEOUT 1U    /*!< FLASH Program operation quad-word timeout in ms */
#define FLASH_ERASE_PAGE_OPERATION_TIMEOUT       10U   /*!< FLASH Erase page operation timeout in ms        */

#define FLASH_QUADWORD_SIZE_BYTE   16U /*!< FLASH quad-word size in bytes   */
#define FLASH_DOUBLEWORD_SIZE_BYTE 8U  /*!< FLASH double-word size in bytes */
#define FLASH_WORD_SIZE_BYTE       4U  /*!< FLASH word size in bytes        */
#define FLASH_HALFWORD_SIZE_BYTE   2U  /*!< FLASH half-word size in bytes   */

#define FLASH_OTP_16BIT_OFFSET_BASE 0x200UL /*!< Base 16-bit offset of OTP area in the 16-bit interface address space */
#define FLASH_16BIT_ROW_SIZE        0x80UL  /*!< Size of a row in 16-bit interface units used for address mapping     */
#define FLASH_OTP_PAGE_SIZE         0x600UL /*!< FLASH OTP page size                                                  */

/**
  * @}
  */

/* Private Macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Private_Macros FLASH Private Macros
  * @{
  */

/*! Macro to get the FLASH physical instance from the handle */
#define FLASH_GET_INSTANCE(hflash) ((FLASH_TypeDef *)((uint32_t)((hflash)->instance)))

/*! Macro to check the FLASH memory programming mode type */
#define IS_FLASH_PROGRAM_MODE_TYPE(mode) (((mode) == HAL_FLASH_PROGRAM_QUADWORD) \
                                          || ((mode) == HAL_FLASH_PROGRAM_DOUBLEWORD) \
                                          || ((mode) == HAL_FLASH_PROGRAM_WORD) \
                                          || ((mode) == HAL_FLASH_PROGRAM_HALFWORD) \
                                          || ((mode) == HAL_FLASH_PROGRAM_BYTE))


/*! Macro to check FLASH memory bank */
#define IS_FLASH_BANK(value) (((value) == HAL_FLASH_BANK_1) \
                              || ((value) == HAL_FLASH_BANK_2))

/*! Macro to check the FLASH user bank boundary cross */
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
#define IS_FLASH_NO_USER_CROSS_BANK(addr, size) (((addr) < (FLASH_BASE + FLASH_BANK_SIZE)) \
                                                 ? (((addr) + (size)) \
                                                    <= (FLASH_BASE + FLASH_BANK_SIZE)) \
                                                 : (((addr) + (size)) \
                                                    <= (FLASH_BASE + FLASH_SIZE)))

#else
#define IS_FLASH_NO_USER_CROSS_BANK(addr, size) (((addr) < (FLASH_BASE + FLASH_SIZE)) \
                                                 ? (((addr) < (FLASH_BASE + FLASH_BANK_SIZE)) \
                                                    ? (((addr) + (size)) \
                                                       <= (FLASH_BASE + FLASH_BANK_SIZE)) \
                                                    : (((addr) + (size)) \
                                                       <= (FLASH_BASE + FLASH_SIZE))) \
                                                 : (((addr) < (FLASH_EXT_USER_BASE + FLASH_EXT_USER_BANK_SIZE)) \
                                                    ? (((addr) + (size)) \
                                                       <= (FLASH_EXT_USER_BASE + FLASH_EXT_USER_BANK_SIZE)) \
                                                    : (((addr) + (size)) \
                                                       <= (FLASH_EXT_USER_BASE + FLASH_EXT_USER_SIZE))))
#endif /* USE_HAL_FLASH_OB_EDATA */

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/*! Macro to check the FLASH EDATA bank boundary cross */
#define IS_FLASH_NO_EDATA_CROSS_BANK(addr, size) (((addr) < (FLASH_EDATA_BASE + FLASH_EDATA_BANK_SIZE)) \
                                                  ? (((addr) + (size)) \
                                                     <= (FLASH_EDATA_BASE + FLASH_EDATA_BANK_SIZE)) \
                                                  : (((addr) + (size)) \
                                                     <= (FLASH_EDATA_BASE + FLASH_EDATA_SIZE)))

#endif /* USE_HAL_FLASH_OB_EDATA */
/*! Macro to check FLASH OTP memory address granularity */
#define IS_FLASH_OTP_ADDRESS(value) (((value) >= FLASH_OTP_BASE) && ((value) < (FLASH_OTP_BASE + FLASH_OTP_SIZE)) \
                                     && (((value) % FLASH_HALFWORD_SIZE_BYTE) == 0U))

/*! Macro to check FLASH OTP memory size_byte in range */
#define IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte) (((otp_addr) + (size_byte)) \
                                                         <= (FLASH_OTP_BASE + FLASH_OTP_SIZE))

/*! Macro to check FLASH memory start page and page number in range */
#define IS_FLASH_PAGE_IN_RANGE(page, page_nbr) (((page) + (page_nbr)) \
                                                <= (FLASH_PAGE_NB + FLASH_EDATA_PAGE_NB))

/*! Macro to get FLASH programming size */
#define FLASH_GET_PROGRAM_SIZE(mode) (((mode) == HAL_FLASH_PROGRAM_QUADWORD) ? FLASH_QUADWORD_SIZE_BYTE : \
                                      (((mode) == HAL_FLASH_PROGRAM_DOUBLEWORD) ? FLASH_DOUBLEWORD_SIZE_BYTE : \
                                       (((mode) == HAL_FLASH_PROGRAM_WORD) ? FLASH_WORD_SIZE_BYTE : \
                                        (((mode) == HAL_FLASH_PROGRAM_HALFWORD) ? FLASH_HALFWORD_SIZE_BYTE : 1U))))

/*! Macro to get the FLASH erase bank */
#define FLASH_GET_ERASE_BANK(bank) (((bank) == HAL_FLASH_BANK_1) ? LL_FLASH_ERASE_BANK_1 : LL_FLASH_ERASE_BANK_2)

/*! Macro to get FLASH memory erase area */
#define FLASH_GET_ERASE_AREA(page) (((page) < FLASH_PAGE_NB) \
                                    ? LL_FLASH_ERASE_USER_AREA : LL_FLASH_ERASE_EDATA_AREA)

/*! Macro to get FLASH memory erase area address */
#define FLASH_GET_ERASE_AREA_ADDR(addr) (((addr) < FLASH_EXT_USER_BASE) \
                                         ? LL_FLASH_ERASE_USER_AREA : LL_FLASH_ERASE_EDATA_AREA)

/*! Macro to get FLASH memory page index */
#define FLASH_GET_HW_PAGE_INDEX(page_index, max_page_index) \
  (((page_index) >= (max_page_index)) ? ((page_index) - (max_page_index)) : (page_index))


/**
  * @}
  */

/* Private Functions Prototypes --------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Private_Functions FLASH Private Functions
  * @{
  */
#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
static hal_status_t FLASH_IsReadyForOperation(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */

static hal_status_t FLASH_WaitForEndOfOperation(hal_flash_handle_t *hflash, uint32_t timeout_ms);

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
static hal_flash_program_unit_func_t FLASH_GetProgramUnitFunction(hal_flash_program_mode_t programming_mode);
static void         FLASH_UpdateAdaptiveProgMode(hal_flash_handle_t *hflash);
static hal_status_t FLASH_Program(hal_flash_handle_t *hflash, uint32_t flash_addr, const uint32_t *p_data,
                                  uint32_t size_byte, uint32_t timeout_ms);
static hal_status_t FLASH_Program_IT(hal_flash_handle_t *hflash, uint32_t flash_addr, const uint32_t *p_data,
                                     uint32_t size_byte);
static void         FLASH_ProgramByWord(hal_flash_handle_t *hflash);
static void         FLASH_ProgramByHalfWord(hal_flash_handle_t *hflash);
static void         FLASH_ProgramByByte(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
static uint32_t FLASH_GetPageSizeByAddr(uint32_t page_addr);
static uint32_t FLASH_GetPageIndexByAddr(uint32_t flash_addr);
static hal_status_t FLASH_EraseAddr(hal_flash_handle_t *hflash,
                                    uint32_t flash_addr,
                                    uint32_t size_byte,
                                    uint32_t page_size,
                                    uint32_t timeout_ms);
static hal_status_t FLASH_EraseAddr_IT(hal_flash_handle_t *hflash,
                                       uint32_t flash_addr,
                                       uint32_t size_byte);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
static hal_status_t FLASH_ErasePage(hal_flash_handle_t *hflash,
                                    hal_flash_bank_t bank,
                                    uint32_t start_page,
                                    uint32_t page_nbr,
                                    uint32_t erase_area,
                                    uint32_t timeout_ms);
static hal_status_t FLASH_ErasePage_IT(hal_flash_handle_t *hflash,
                                       hal_flash_bank_t bank,
                                       uint32_t start_page,
                                       uint32_t page_nbr,
                                       uint32_t erase_area);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
static void FLASH_FillErrorCode(hal_flash_handle_t *hflash, uint32_t flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1))
static hal_flash_bank_t FLASH_GetBank(const hal_flash_handle_t *hflash, uint32_t flash_addr);
static uint32_t FLASH_IsFlashUSERAddr(uint32_t flash_addr, uint8_t check_program_alignment);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1) \
    && (defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)))
static uint32_t FLASH_IsFlashEDATAAddr(uint32_t flash_addr, uint8_t check_program_alignment);
#endif /* USE_HAL_FLASH_OB_EDATA && (USE_ASSERT_DBG_PARAM || (USE_HAL_CHECK_PARAM)) */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR */

#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
static void FLASH_HandleErrorIT(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */

/**
  * @}
  */

/* Exported Functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup FLASH_Exported_Functions
  * @{
  */

/** @addtogroup FLASH_Exported_Functions_Group1
  * @{
This subsection provides a set of functions allowing initialization and de-initialization of the FLASH peripheral:

- Call the function HAL_FLASH_Init() to initialize the FLASH handle and associate a physical FLASH instance.
  (As optional, FLASH clock is enabled inside the function)

- Call the function HAL_FLASH_DeInit() to de-initialize the FLASH handle.
  */


/**
  * @brief  Initialize the FLASH handle and associate a physical FLASH instance.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  instance          Specifies the FLASH instance based on @ref hal_flash_t enumeration.
  * @retval HAL_INVALID_PARAM Invalid parameter when hflash pointer is NULL.
  * @retval HAL_OK            FLASH is successfully initialized.
  */
hal_status_t HAL_FLASH_Init(hal_flash_handle_t *hflash, hal_flash_t instance)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE((FLASH_TypeDef *)(uint32_t)instance));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hflash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->instance = instance;

#if defined(USE_HAL_FLASH_CLK_ENABLE_MODEL) && (USE_HAL_FLASH_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_FLASH_EnableClock();
#endif /* USE_HAL_FLASH_CLK_ENABLE_MODEL */

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  hflash->p_program_cplt_cb       = HAL_FLASH_ProgramCpltCallback;
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
  hflash->p_erase_by_addr_cplt_cb = HAL_FLASH_EraseByAddrCpltCallback;
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
  hflash->p_erase_page_cplt_cb    = HAL_FLASH_ErasePageCpltCallback;
#endif /* USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
  hflash->p_erase_bank_cplt_cb    = HAL_FLASH_EraseBankCpltCallback;
#endif /* USE_HAL_FLASH_ERASE_BANK */
#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
  hflash->p_mass_erase_cplt_cb    = HAL_FLASH_MassEraseCpltCallback;
#endif /* USE_HAL_FLASH_MASS_ERASE */
  hflash->p_error_cb              = HAL_FLASH_ErrorCallback;
#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
  hflash->p_ecc_error_cb          = HAL_FLASH_ECC_ErrorCallback;
#endif /* USE_HAL_FLASH_ECC */
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */

#if defined(USE_HAL_FLASH_USER_DATA) && (USE_HAL_FLASH_USER_DATA == 1)
  hflash->p_user_data = NULL;
#endif /* USE_HAL_FLASH_USER_DATA */

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
  hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

  hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  hflash->programming_mode = HAL_FLASH_PROGRAM_QUADWORD;
  hflash->prog_size_byte = FLASH_QUADWORD_SIZE_BYTE;
  hflash->p_prog_unit_func = FLASH_ProgramByWord;
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

  hflash->global_state = HAL_FLASH_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  De-initialize the FLASH handle.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_DeInit(hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE((FLASH_TypeDef *)(uint32_t)hflash->instance));

  (void)FLASH_WaitForEndOfOperation(hflash, FLASH_OPERATION_MAX_TIMEOUT);

  hflash->global_state = HAL_FLASH_STATE_RESET;
}
/**
  * @}
  */

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/** @addtogroup FLASH_Exported_Functions_Group2
  * @{
This subsection provides a set of functions allowing configuration of the FLASH peripheral:

- Call the function HAL_FLASH_SetProgrammingMode() to set the programming mode according to selected parameter within
  @ref hal_flash_program_mode_t enumeration.

- Call the function HAL_FLASH_GetProgrammingMode() to get the current configured programming mode.
  */

/**
  * @brief  Set the FLASH programming mode configuration.
  * @param  hflash           Pointer to a @ref hal_flash_handle_t structure.
  * @param  programming_mode This parameter is one element of @ref hal_flash_program_mode_t enumeration.
  * @retval HAL_OK           Programming mode is successfully configured.
  */
hal_status_t HAL_FLASH_SetProgrammingMode(hal_flash_handle_t *hflash, hal_flash_program_mode_t programming_mode)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_PROGRAM_MODE_TYPE(programming_mode));

  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  hflash->programming_mode = programming_mode;
  hflash->prog_size_byte = FLASH_GET_PROGRAM_SIZE(programming_mode);
  hflash->p_prog_unit_func = FLASH_GetProgramUnitFunction(programming_mode);

  return HAL_OK;
}

/**
  * @brief  Get the FLASH programming mode configuration.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval hal_flash_program_mode_t Programming mode value.
  */
hal_flash_program_mode_t HAL_FLASH_GetProgrammingMode(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  return (hflash->programming_mode);
}

/**
  * @}
  */

#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

/** @addtogroup FLASH_Exported_Functions_Group3
  * @{
This subsection provides a set of functions allowing programming and erasing of the FLASH memory:

- Call the function HAL_FLASH_ProgramByAddr() to program the FLASH USER memory by address in polling mode.

- Call the function HAL_FLASH_ProgramByAddr_IT() to program the FLASH USER memory by address in interrupt mode.

- Call the function HAL_FLASH_ProgramByAddrAdapt() to program the FLASH USER memory by address using adaptive width in
  polling mode.

- Call the function HAL_FLASH_ProgramByAddrAdapt_IT() to program the FLASH USER memory by address using adaptive width
  in interrupt mode.

- Call the function HAL_FLASH_OTP_ProgramByAddr() to program the OTP area by address in polling mode.

- Call the function HAL_FLASH_OTP_ProgramByAddr_IT() to program the OTP area by address in interrupt mode.

- Call the function HAL_FLASH_OTP_ProgramByAddrAdapt() to program the OTP Area by address using adaptive width in
  polling mode.

- Call the function HAL_FLASH_OTP_ProgramByAddrAdapt_IT() to program the OTP Area by address using adaptive width in
  interrupt mode.

- Call the function HAL_FLASH_EDATA_ProgramByAddr() to program the FLASH EDATA memory by address in polling mode.

- Call the function HAL_FLASH_EDATA_ProgramByAddr_IT() to program the FLASH EDATA memory by address in interrupt mode.

- Call the function HAL_FLASH_EDATA_ProgramByAddrAdapt() to program the FLASH EDATA memory by address using adaptive
  width in polling mode.

- Call the function HAL_FLASH_EDATA_ProgramByAddrAdapt_IT() to program the FLASH EDATA memory by address using adaptive
  width in interrupt mode.

- Call the function HAL_FLASH_EraseByAddr() to erase the FLASH USER memory by address in polling mode.

- Call the function HAL_FLASH_EraseByAddr_IT() to erase the FLASH USER memory by address in interrupt mode.

- Call the function HAL_FLASH_EDATA_EraseByAddr() to erase the FLASH EDATA memory by address in polling mode.

- Call the function HAL_FLASH_EDATA_EraseByAddr_IT() to erase the FLASH EDATA memory by address in interrupt mode.

- Call the function HAL_FLASH_ErasePage() to erase the FLASH USER memory by page in polling mode.

- Call the function HAL_FLASH_ErasePage_IT() to erase the FLASH USER memory by page in interrupt mode.

- Call the function HAL_FLASH_EDATA_ErasePage() to erase the FLASH EDATA memory by page in polling mode.

- Call the function HAL_FLASH_EDATA_ErasePage_IT() to erase the FLASH EDATA memory by page in interrupt mode.

- Call the function HAL_FLASH_EraseBank() to erase the FLASH memory by bank in polling mode.

- Call the function HAL_FLASH_EraseBank_IT() to erase the FLASH memory by bank in interrupt mode.

- Call the function HAL_FLASH_MassErase() to mass erase the complete FLASH memoryin polling mode.

- Call the function HAL_FLASH_MassErase_IT() to mass erase the complete FLASH memory in interrupt mode.

- Call the function HAL_FLASH_IRQHandler() to handle any enabled interrupt and call its corresponding callback.

- Call the function HAL_FLASH_ProgramByAddr_IRQHandler() to handle any enabled FLASH program interrupt and call
  its corresponding callback.

- Call the function HAL_FLASH_EraseByAddr_IRQHandler() to handle any enabled FLASH erase by address interrupt and call
  its corresponding callback.

- Call the function HAL_FLASH_ErasePage_IRQHandler() to handle any enabled FLASH erase page interrupt and call
  its corresponding callback.

- Call the function HAL_FLASH_EraseBank_IRQHandler() to handle any enabled FLASH erase bank interrupt and call
  its corresponding callback.

- Call the function HAL_FLASH_MassErase_IRQHandler() to handle any enabled FLASH mass erase interrupt and call
  its corresponding callback.

- Call the function HAL_FLASH_ECC_IRQHandler() to handle any enabled interrupt and call its corresponding
  callback.

- Call the function HAL_FLASH_NMI_IRQHandler() to handle any non-maskable interrupt and call its corresponding
  callback.
 */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/**
  * @brief  Program the FLASH memory at specified user address in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash to be programmed.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_ProgramByAddr(hal_flash_handle_t *hflash,
                                     uint32_t flash_addr,
                                     const uint32_t *p_data,
                                     uint32_t size_byte,
                                     uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_PROGRAM_MODE_TYPE(hflash->programming_mode));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program(hflash, flash_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the FLASH memory at specified address in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash to be programmed.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                        uint32_t flash_addr,
                                        const uint32_t *p_data,
                                        uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_PROGRAM_MODE_TYPE(hflash->programming_mode));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program_IT(hflash, flash_addr, p_data, size_byte);
}

/**
  * @brief  Program the FLASH memory at specified user address using adaptive width in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash to be programmed.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                          uint32_t flash_addr,
                                          const uint32_t *p_data,
                                          uint32_t size_byte,
                                          uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program(hflash, flash_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the FLASH memory at specified address using adaptive width in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash to be programmed.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                             uint32_t flash_addr,
                                             const uint32_t *p_data,
                                             uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program_IT(hflash, flash_addr, p_data, size_byte);
}

/**
  * @brief  Program the OTP area at specified address in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  otp_addr          OTP address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked or when the TrustZone enabled.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_OTP_ProgramByAddr(hal_flash_handle_t *hflash,
                                         uint32_t otp_addr,
                                         const uint32_t *p_data,
                                         uint32_t size_byte,
                                         uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_OTP_ADDRESS(otp_addr));
  ASSERT_DBG_PARAM(IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte));
  ASSERT_DBG_PARAM((hflash->programming_mode == HAL_FLASH_PROGRAM_HALFWORD) \
                   || (hflash->programming_mode == HAL_FLASH_PROGRAM_WORD));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((IS_FLASH_OTP_ADDRESS(otp_addr) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program(hflash, otp_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the OTP area at specified address in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  otp_addr          OTP address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked or when the TrustZone enabled.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_OTP_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                            uint32_t otp_addr,
                                            const uint32_t *p_data,
                                            uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_OTP_ADDRESS(otp_addr));
  ASSERT_DBG_PARAM(IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte));
  ASSERT_DBG_PARAM((hflash->programming_mode == HAL_FLASH_PROGRAM_HALFWORD) \
                   || (hflash->programming_mode == HAL_FLASH_PROGRAM_WORD));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((IS_FLASH_OTP_ADDRESS(otp_addr) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program_IT(hflash, otp_addr, p_data, size_byte);
}

/**
  * @brief  Program the OTP area at specified address using adaptive width in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  otp_addr          OTP address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked or when the TrustZone enabled.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_OTP_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                              uint32_t otp_addr,
                                              const uint32_t *p_data,
                                              uint32_t size_byte,
                                              uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_OTP_ADDRESS(otp_addr));
  ASSERT_DBG_PARAM(IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_HALFWORD_SIZE_BYTE) == 0U));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((IS_FLASH_OTP_ADDRESS(otp_addr) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program(hflash, otp_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the OTP area at specified address using adaptive width in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  otp_addr          OTP address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked or when the TrustZone enabled.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_OTP_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                                 uint32_t otp_addr,
                                                 const uint32_t *p_data,
                                                 uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_OTP_ADDRESS(otp_addr));
  ASSERT_DBG_PARAM(IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_HALFWORD_SIZE_BYTE) == 0U));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((IS_FLASH_OTP_ADDRESS(otp_addr) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((IS_FLASH_OTP_SIZE_IN_RANGE(otp_addr, size_byte) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program_IT(hflash, otp_addr, p_data, size_byte);
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Program the EDATA area at specified address in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        FLASH EDATA address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ProgramByAddr(hal_flash_handle_t *hflash,
                                           uint32_t flash_addr,
                                           const uint32_t *p_data,
                                           uint32_t size_byte,
                                           uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM((hflash->programming_mode == HAL_FLASH_PROGRAM_HALFWORD) \
                   || (hflash->programming_mode == HAL_FLASH_PROGRAM_WORD));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program(hflash, flash_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the EDATA area at specified address in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        FLASH EDATA address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                              uint32_t flash_addr,
                                              const uint32_t *p_data,
                                              uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM((hflash->programming_mode == HAL_FLASH_PROGRAM_HALFWORD) \
                   || (hflash->programming_mode == HAL_FLASH_PROGRAM_WORD));
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_GET_PROGRAM_SIZE(hflash->programming_mode)) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 0U;

  return FLASH_Program_IT(hflash, flash_addr, p_data, size_byte);
}

/**
  * @brief  Program the EDATA area at specified address using adaptive width in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        FLASH EDATA address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @param  timeout_ms        Timeout value for programming.
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                                uint32_t flash_addr,
                                                const uint32_t *p_data,
                                                uint32_t size_byte,
                                                uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_HALFWORD_SIZE_BYTE) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program(hflash, flash_addr, p_data, size_byte, timeout_ms);
}

/**
  * @brief  Program the EDATA area at specified address using adaptive width in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        FLASH EDATA address.
  * @param  p_data            Pointer to the data to be programmed.
  * @param  size_byte         Size of the data to be programmed (in bytes).
  * @note   The adaptive mode optimizes FLASH programming access based on the remaining data size,
  *         ignoring the programming mode selected by the HAL_FLASH_SetProgrammingMode() API.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new programming operation.
  * @retval HAL_OK            Returned when the programming operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                                   uint32_t flash_addr,
                                                   const uint32_t *p_data,
                                                   uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM((size_byte != 0U) && ((size_byte % FLASH_HALFWORD_SIZE_BYTE) == 0U));
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hflash->is_adaptive_prog = 1U;

  return FLASH_Program_IT(hflash, flash_addr, p_data, size_byte);
}
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief  Erase the FLASH memory at specified address in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash page to be erased.
  *                           This parameter must be aligned with the start of a page.
  * @param  size_byte         Size of the area to be erased (in bytes).
  *                           This parameter must be a multiple of the page size.
  * @param  timeout_ms        Timeout value for erasing.
  *
  * @warning Since the smallest erase granularity is at page level, this function will erase a greater area than the one
  *          defined by flash_addr and size_byte if these parameters do not represent an area aligned with the start
  *          and the end of flash pages.
  *
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK            Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EraseByAddr(hal_flash_handle_t *hflash,
                                   uint32_t flash_addr,
                                   uint32_t size_byte,
                                   uint32_t timeout_ms)
{
  uint32_t page_size;

  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr  + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (!IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte)))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  page_size = FLASH_PAGE_SIZE;
#else
  page_size = ((flash_addr < FLASH_EXT_USER_BASE) ? FLASH_PAGE_SIZE : FLASH_EXT_USER_PAGE_SIZE);
#endif /* !USE_HAL_FLASH_OB_EDATA */

  return FLASH_EraseAddr(hflash, flash_addr, size_byte, page_size, timeout_ms);
}

/**
  * @brief  Erase the FLASH memory at specified address in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash page to be erased.
  *                           This parameter must be aligned with the start of a page.
  * @param  size_byte         Size of the data to be erased (in bytes).
  *                           This parameter must be a multiple of the page size.
  *
  * @warning Since the smallest erase granularity is at page level, this function will erase a greater area than the one
  *          defined by flash_addr and size_byte if these parameters do not represent an area aligned with the start
  *          and the end of flash pages.
  *
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK            Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EraseByAddr_IT(hal_flash_handle_t *hflash,
                                      uint32_t flash_addr,
                                      uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashUSERAddr(flash_addr  + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashUSERAddr(flash_addr, 1U) == 0U) || (!IS_FLASH_NO_USER_CROSS_BANK(flash_addr, size_byte)))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashUSERAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  return FLASH_EraseAddr_IT(hflash, flash_addr, size_byte);
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Erase the FLASH EDATA memory at specified address in polling mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash page to be erased.
  *                           This parameter must be aligned with the start of a page.
  * @param  size_byte         Size of the area to be erased (in bytes).
  *                           This parameter must be a multiple of the page size.
  * @param  timeout_ms        Timeout value for erasing.
  *
  * @warning Since the smallest erase granularity is at page level, this function will erase a greater area than the one
  *          defined by flash_addr and size_byte if these parameters do not represent an area aligned with the start
  *          and the end of flash pages.
  *
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT       Internal processing exceeded the timeout.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK            Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_EraseByAddr(hal_flash_handle_t *hflash,
                                         uint32_t flash_addr,
                                         uint32_t size_byte,
                                         uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr  + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (!IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte)))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  return FLASH_EraseAddr(hflash, flash_addr, size_byte, FLASH_EDATA_PAGE_SIZE, timeout_ms);
}

/**
  * @brief  Erase the FLASH EDATA memory at specified address in interrupt mode.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr        Start address of flash page to be erased.
  *                           This parameter must be aligned with the start of a page.
  * @param  size_byte         Size of the data to be erased (in bytes).
  *                           This parameter must be a multiple of the page size.
  *
  * @warning Since the smallest erase granularity is at page level, this function will erase a greater area than the one
  *          defined by flash_addr and size_byte if these parameters do not represent an area aligned with the start
  *          and the end of flash pages.
  *
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval HAL_BUSY          Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK            Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_EraseByAddr_IT(hal_flash_handle_t *hflash,
                                            uint32_t flash_addr,
                                            uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr, 1U) != 0U);
  ASSERT_DBG_PARAM(FLASH_IsFlashEDATAAddr(flash_addr  + size_byte - 1U, 0U) != 0U);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_PARAM(IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((FLASH_IsFlashEDATAAddr(flash_addr, 1U) == 0U) || (!IS_FLASH_NO_EDATA_CROSS_BANK(flash_addr, size_byte)))
  {
    return HAL_INVALID_PARAM;
  }

  if ((FLASH_IsFlashEDATAAddr(flash_addr + size_byte - 1U, 0U) == 0U) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  return FLASH_EraseAddr_IT(hflash, flash_addr, size_byte);
}
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  Erase the FLASH memory by page in polling mode.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank        This parameter can be one of the following values:
  *                         @arg @ref HAL_FLASH_BANK_1
  *                         @arg @ref HAL_FLASH_BANK_2
  * @param  page        First page to be erased.
  * @param  page_nbr    Number of pages to be erased.
  * @param  timeout_ms  Timeout value for page erasing.
  * @retval HAL_ERROR   Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT Internal processing exceeded the timeout.
  * @retval HAL_BUSY    Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK      Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_ErasePage(hal_flash_handle_t *hflash,
                                 hal_flash_bank_t bank,
                                 uint32_t page,
                                 uint32_t page_nbr,
                                 uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  ASSERT_DBG_PARAM((page + page_nbr) <= FLASH_PAGE_NB);
#else
  ASSERT_DBG_PARAM(IS_FLASH_PAGE_IN_RANGE(page, page_nbr));
#endif /* USE_HAL_FLASH_OB_EDATA */
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  return FLASH_ErasePage(hflash, bank, page, page_nbr, FLASH_GET_ERASE_AREA(page), timeout_ms);
}

/**
  * @brief  Erase the FLASH memory by page in interrupt mode.
  * @param  hflash    Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank      This parameter can be one of the following values:
  *                       @arg @ref HAL_FLASH_BANK_1
  *                       @arg @ref HAL_FLASH_BANK_2
  * @param  page      First page to be erased.
  * @param  page_nbr  Number of pages to be erased.
  * @retval HAL_ERROR Returned when the FLASH access registers locked.
  * @retval HAL_BUSY  Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK    Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_ErasePage_IT(hal_flash_handle_t *hflash,
                                    hal_flash_bank_t bank,
                                    uint32_t page,
                                    uint32_t page_nbr)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  ASSERT_DBG_PARAM((page + page_nbr) <= FLASH_PAGE_NB);
#else
  ASSERT_DBG_PARAM(IS_FLASH_PAGE_IN_RANGE(page, page_nbr));
#endif /* USE_HAL_FLASH_OB_EDATA */
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  return FLASH_ErasePage_IT(hflash, bank, page, page_nbr, FLASH_GET_ERASE_AREA(page));
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Erase the FLASH EDATA memory by page in polling mode.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank        This parameter can be one of the following values:
  *                         @arg @ref HAL_FLASH_BANK_1
  *                         @arg @ref HAL_FLASH_BANK_2
  * @param  page        First page to be erased.
  * @param  page_nbr    Number of pages to be erased.
  * @param  timeout_ms  Timeout value for page erasing.
  * @retval HAL_ERROR   Returned when the FLASH access registers locked.
  * @retval HAL_TIMEOUT Internal processing exceeded the timeout.
  * @retval HAL_BUSY    Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK      Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ErasePage(hal_flash_handle_t *hflash,
                                       hal_flash_bank_t bank,
                                       uint32_t page,
                                       uint32_t page_nbr,
                                       uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM((page + page_nbr) <= FLASH_EDATA_PAGE_NB);

  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  return FLASH_ErasePage(hflash, bank, page + FLASH_PAGE_NB, page_nbr, LL_FLASH_ERASE_EDATA_AREA, timeout_ms);
}

/**
  * @brief  Erase the FLASH EDATA memory by page in interrupt mode.
  * @param  hflash    Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank      This parameter can be one of the following values:
  *                       @arg @ref HAL_FLASH_BANK_1
  *                       @arg @ref HAL_FLASH_BANK_2
  * @param  page      First page to be erased.
  * @param  page_nbr  Number of pages to be erased.
  * @retval HAL_ERROR Returned when the FLASH access registers locked.
  * @retval HAL_BUSY  Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK    Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EDATA_ErasePage_IT(hal_flash_handle_t *hflash,
                                          hal_flash_bank_t bank,
                                          uint32_t page,
                                          uint32_t page_nbr)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM((page + page_nbr) <= FLASH_EDATA_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  return FLASH_ErasePage_IT(hflash, bank, page + FLASH_PAGE_NB, page_nbr, LL_FLASH_ERASE_EDATA_AREA);
}
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
/**
  * @brief   Erase the FLASH memory by bank in polling mode.
  * @param   hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param   bank        Bank to be erased.
  *                      This parameter can be one of the following values:
  *                         @arg @ref HAL_FLASH_BANK_1
  *                         @arg @ref HAL_FLASH_BANK_2
  * @param   timeout_ms  Timeout value for bank erasing.
  * @warning This function will erase a specific Flash bank, thus it must not be mapped in this bank.
  * @retval  HAL_ERROR   Returned when the FLASH access registers locked.
  * @retval  HAL_TIMEOUT Internal processing exceeded the timeout.
  * @retval  HAL_BUSY    Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval  HAL_OK      Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EraseBank(hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_BANK_ERASE;
    hflash->erase_bank = bank;

    LL_FLASH_StartEraseBank(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank));

    status = FLASH_WaitForEndOfOperation(hflash, timeout_ms);

    LL_FLASH_DisableBankErase(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(bank));

    hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
    hflash->global_state = HAL_FLASH_STATE_IDLE;
  }

  return status;
}

/**
  * @brief   Erase the FLASH memory by bank in interrupt mode.
  * @param   hflash    Pointer to a @ref hal_flash_handle_t structure.
  * @param   bank      Bank to be erased.
  *                    This parameter can be one of the following values:
        *                  @arg @ref HAL_FLASH_BANK_1
        *                  @arg @ref HAL_FLASH_BANK_2
  * @warning This function will erase a specific Flash bank, thus it must not be mapped in this bank.
  * @retval  HAL_ERROR Returned when the FLASH access registers locked.
  * @retval  HAL_BUSY  Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval  HAL_OK    Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_EraseBank_IT(hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(bank != HAL_FLASH_BANK_ALL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation  = HAL_FLASH_BANK_ERASE;
    hflash->erase_bank = bank;

    LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
    LL_FLASH_EnableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
    LL_FLASH_StartEraseBank(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank));
  }

  return status;
}
#endif /* USE_HAL_FLASH_ERASE_BANK */


#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
/**
  * @brief   Mass erase the FLASH memory in polling mode.
  * @param   hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param   timeout_ms  Timeout value for mass erasing.
  * @warning This function will erase the whole Flash, thus it must not be mapped in the Flash area.
  * @retval  HAL_ERROR   Returned when the FLASH access registers locked.
  * @retval  HAL_TIMEOUT Internal processing exceeded the timeout.
  * @retval  HAL_BUSY    Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval  HAL_OK      Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_MassErase(hal_flash_handle_t *hflash, uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation  = HAL_FLASH_MASS_ERASE;
    hflash->erase_bank = HAL_FLASH_BANK_ALL;

    LL_FLASH_StartMassErase(FLASH_GET_INSTANCE(hflash));

    status = FLASH_WaitForEndOfOperation(hflash, timeout_ms);

    LL_FLASH_DisableMassErase(FLASH_GET_INSTANCE(hflash));

    hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
    hflash->global_state = HAL_FLASH_STATE_IDLE;
  }

  return status;
}

/**
  * @brief   Mass erase the FLASH memory in interrupt mode.
  * @param   hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @warning This function will erase the whole Flash, thus it must not be mapped in the Flash area.
  * @retval  HAL_ERROR         Returned when the FLASH access registers locked.
  * @retval  HAL_BUSY          Returned when the FLASH is busy and cannot start a new erase operation.
  * @retval  HAL_OK            Returned when the erasing operation completes successfully.
  */
hal_status_t HAL_FLASH_MassErase_IT(hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation  = HAL_FLASH_MASS_ERASE;
    hflash->erase_bank = HAL_FLASH_BANK_ALL;

    LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
    LL_FLASH_EnableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
    LL_FLASH_StartMassErase(FLASH_GET_INSTANCE(hflash));
  }

  return status;
}
#endif /* USE_HAL_FLASH_MASS_ERASE */


/**
  * @brief  Handle the FLASH interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */

void HAL_FLASH_IRQHandler(hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  HAL_FLASH_ProgramByAddr_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
  HAL_FLASH_EraseByAddr_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
  HAL_FLASH_ErasePage_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
  HAL_FLASH_EraseBank_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
  HAL_FLASH_MassErase_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_MASS_ERASE */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
  HAL_FLASH_ECC_IRQHandler(hflash);
#endif /* USE_HAL_FLASH_ECC */

#if (!defined(USE_HAL_FLASH_PROGRAM_BY_ADDR) || (USE_HAL_FLASH_PROGRAM_BY_ADDR != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_BY_ADDR) || (USE_HAL_FLASH_ERASE_BY_ADDR != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_PAGE) || (USE_HAL_FLASH_ERASE_PAGE != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_BANK) || (USE_HAL_FLASH_ERASE_BANK != 1)) \
    && (!defined(USE_HAL_FLASH_MASS_ERASE) || (USE_HAL_FLASH_MASS_ERASE != 1)) \
    && (!defined(USE_HAL_FLASH_ECC) || (USE_HAL_FLASH_ECC != 1))

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);

#endif /* (!defined(USE_HAL_FLASH_PROGRAM_BY_ADDR) || (USE_HAL_FLASH_PROGRAM_BY_ADDR != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_BY_ADDR) || (USE_HAL_FLASH_ERASE_BY_ADDR != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_PAGE) || (USE_HAL_FLASH_ERASE_PAGE != 1)) \
    && (!defined(USE_HAL_FLASH_ERASE_BANK) || (USE_HAL_FLASH_ERASE_BANK != 1)) \
    && (!defined(USE_HAL_FLASH_MASS_ERASE) || (USE_HAL_FLASH_MASS_ERASE != 1)) \
    && (!defined(USE_HAL_FLASH_ECC) || (USE_HAL_FLASH_ECC != 1)) */
}


#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/**
  * @brief  Handle the FLASH program by address interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_ProgramByAddr_IRQHandler(hal_flash_handle_t *hflash)
{
  uint32_t flags;
  uint32_t prog_size;

  ASSERT_DBG_PARAM(hflash != NULL);

  if (hflash->ongoing_operation == HAL_FLASH_PROGRAM)
  {
    flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL | LL_FLASH_FLAG_EOP);

    if (STM32_READ_BIT(flags, LL_FLASH_FLAG_ERRORS_ALL) != 0U)
    {
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
      FLASH_FillErrorCode(hflash, flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

      FLASH_HandleErrorIT(hflash, FLASH_GetBank(hflash, hflash->prog_flash_addr));
    }
    else
    {
      if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
      {
        LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
        prog_size = hflash->prog_size_byte;

        /* Check if there are still data to program */
        if (hflash->count > prog_size)
        {
          hflash->count           -= prog_size;
          hflash->prog_data_addr  += prog_size;
          hflash->prog_flash_addr += prog_size;

          if (hflash->is_adaptive_prog != 0U)
          {
            FLASH_UpdateAdaptiveProgMode(hflash);
          }

          hflash->p_prog_unit_func(hflash);
        }
        else
        {
          LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
          LL_FLASH_DisableProgramming(FLASH_GET_INSTANCE(hflash));

          hflash->count            = 0;
          hflash->prog_data_addr  += (prog_size - 1U);
          hflash->prog_flash_addr += (prog_size - 1U);

          hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
          hflash->global_state = HAL_FLASH_STATE_IDLE;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
          hflash->p_program_cplt_cb(hflash, (hflash->prog_flash_addr - hflash->size + 1U), hflash->size);
#else
          HAL_FLASH_ProgramCpltCallback(hflash, (hflash->prog_flash_addr - hflash->size + 1U), hflash->size);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
        }
      }
    }
  }
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief  Handle the FLASH erase by address interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_EraseByAddr_IRQHandler(hal_flash_handle_t *hflash)
{
  uint32_t flags;
  uint32_t page_size;
  uint32_t erased_size;
  uint32_t erase_area;

  ASSERT_DBG_PARAM(hflash != NULL);

  if (hflash->ongoing_operation == HAL_FLASH_ADDR_ERASE)
  {
    flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL | LL_FLASH_FLAG_EOP);

    if (STM32_READ_BIT(flags, LL_FLASH_FLAG_ERRORS_ALL) != 0U)
    {
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
      FLASH_FillErrorCode(hflash, flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

      FLASH_HandleErrorIT(hflash, hflash->erase_bank);
    }
    else
    {
      if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
      {
        LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));

        /* Update erase count */
        page_size = FLASH_GetPageSizeByAddr(hflash->erase_page);
        erased_size = page_size - (hflash->erase_page % page_size);

        /* Check if there are still pages to erase */
        if (hflash->count > erased_size)
        {
          /* Prepare and call next erase operation */
          hflash->count -= erased_size;
          hflash->erase_page += erased_size;
          erase_area = FLASH_GET_ERASE_AREA_ADDR(hflash->erase_page);
          LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), \
                                  erase_area, FLASH_GetPageIndexByAddr(hflash->erase_page));
        }
        else
        {
          LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
          LL_FLASH_DisablePageErase(FLASH_GET_INSTANCE(hflash));
          erased_size = hflash->count;
          hflash->erase_page += erased_size - 1U;
          hflash->count = 0U;
          hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
          hflash->global_state = HAL_FLASH_STATE_IDLE;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
          hflash->p_erase_by_addr_cplt_cb(hflash, (hflash->erase_page - hflash->size + 1U), hflash->size);
#else
          HAL_FLASH_EraseByAddrCpltCallback(hflash, (hflash->erase_page - hflash->size + 1U), hflash->size);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
        }
      }
    }
  }
}
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  Handle the FLASH erase page interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_ErasePage_IRQHandler(hal_flash_handle_t *hflash)
{
  uint32_t flags;
  uint32_t erase_area;

  ASSERT_DBG_PARAM(hflash != NULL);

  if (hflash->ongoing_operation == HAL_FLASH_PAGE_ERASE)
  {
    flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL | LL_FLASH_FLAG_EOP);

    if (STM32_READ_BIT(flags, LL_FLASH_FLAG_ERRORS_ALL) != 0U)
    {
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
      FLASH_FillErrorCode(hflash, flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

      FLASH_HandleErrorIT(hflash, hflash->erase_bank);
    }
    else
    {
      if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
      {
        LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));

        hflash->count--;

        /* Check if there are still pages to erase */
        if (hflash->count > 0U)
        {
          /* Prepare and call next erase operation */
          hflash->erase_page++;
          erase_area = FLASH_GET_ERASE_AREA(hflash->erase_page);
          LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), \
                                  erase_area, FLASH_GET_HW_PAGE_INDEX(hflash->erase_page, FLASH_PAGE_NB));
        }
        else /* No more pages to erase */
        {
          LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
          LL_FLASH_DisablePageErase(FLASH_GET_INSTANCE(hflash));

          hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
          hflash->global_state = HAL_FLASH_STATE_IDLE;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
          hflash->p_erase_page_cplt_cb(hflash, hflash->erase_bank, \
                                       FLASH_GET_HW_PAGE_INDEX((hflash->erase_page - hflash->size + 1U), \
                                                               FLASH_PAGE_NB), \
                                       hflash->size);
#else
          HAL_FLASH_ErasePageCpltCallback(hflash, hflash->erase_bank, \
                                          FLASH_GET_HW_PAGE_INDEX((hflash->erase_page - hflash->size + 1U), \
                                                                  FLASH_PAGE_NB), \
                                          hflash->size);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
        }
      }
    }
  }
}
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
/**
  * @brief  Handle the FLASH erase bank interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_EraseBank_IRQHandler(hal_flash_handle_t *hflash)
{
  uint32_t flags;

  ASSERT_DBG_PARAM(hflash != NULL);

  if (hflash->ongoing_operation == HAL_FLASH_BANK_ERASE)
  {
    flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL | LL_FLASH_FLAG_EOP);

    if (STM32_READ_BIT(flags, LL_FLASH_FLAG_ERRORS_ALL) != 0U)
    {
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
      FLASH_FillErrorCode(hflash, flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

      FLASH_HandleErrorIT(hflash, hflash->erase_bank);
    }
    else
    {
      if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
      {
        LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
        LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
        LL_FLASH_DisableBankErase(FLASH_GET_INSTANCE(hflash), (uint32_t)hflash->erase_bank);

        hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
        hflash->global_state = HAL_FLASH_STATE_IDLE;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
        hflash->p_erase_bank_cplt_cb(hflash, hflash->erase_bank);
#else
        HAL_FLASH_EraseBankCpltCallback(hflash, hflash->erase_bank);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
      }
    }
  }
}
#endif /* USE_HAL_FLASH_ERASE_BANK */


#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
/**
  * @brief  Handle the FLASH mass erase interrupt request.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_MassErase_IRQHandler(hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  uint32_t flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL);

  if (hflash->ongoing_operation == HAL_FLASH_MASS_ERASE)
  {
    flags |= LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_EOP);

    if (STM32_READ_BIT(flags, LL_FLASH_FLAG_ERRORS_ALL) != 0U)
    {
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
      FLASH_FillErrorCode(hflash, flags);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

      FLASH_HandleErrorIT(hflash, HAL_FLASH_BANK_ALL);
    }
    else
    {
      if (STM32_READ_BIT(flags, LL_FLASH_FLAG_EOP) != 0U)
      {
        LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
        LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
        LL_FLASH_DisableMassErase(FLASH_GET_INSTANCE(hflash));

        hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
        hflash->global_state = HAL_FLASH_STATE_IDLE;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
        hflash->p_mass_erase_cplt_cb(hflash);
#else
        HAL_FLASH_MassEraseCpltCallback(hflash);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
      }
    }
  }
}
#endif /* USE_HAL_FLASH_MASS_ERASE */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/**
  * @brief  Handle the FLASH ITF NMI interrupt request.
  * @param hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval HAL_ERROR Returned when the NMI has not been handled by this function.
  * @retval HAL_OK    Returned when the NMI has been handled by this function.
  */
hal_status_t HAL_FLASH_NMI_IRQHandler(hal_flash_handle_t *hflash)
{
  hal_status_t cb_status = HAL_ERROR;
  ASSERT_DBG_PARAM(hflash != NULL);

  if (LL_FLASH_IsActiveFlag_ECCD(FLASH_GET_INSTANCE(hflash)) != 0U)
  {
    /* Double ECC flag is only cleared if the callback returns HAL_OK,
       i.e. if the NMI is specifically handled in the callback. */
#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
    if (hflash->p_ecc_error_cb(hflash, ((LL_FLASH_IsActiveFlag_BK_ECCD(FLASH_GET_INSTANCE(hflash)) == 0U) \
                                        ? HAL_FLASH_BANK_1 : HAL_FLASH_BANK_2)) == HAL_OK)
#else
    if (HAL_FLASH_ECC_ErrorCallback(hflash, ((LL_FLASH_IsActiveFlag_BK_ECCD(FLASH_GET_INSTANCE(hflash)) == 0U) \
                                             ? HAL_FLASH_BANK_1 : HAL_FLASH_BANK_2)) == HAL_OK)
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
    {
      LL_FLASH_ClearFlag_ECCD(FLASH_GET_INSTANCE(hflash));

      cb_status = HAL_OK;
    }
  }

  return cb_status;
}

/**
  * @brief  Handle the FLASH ITF ECC correction interrupt request.
  * @param hflash Pointer to a @ref hal_flash_handle_t structure.
  */
void HAL_FLASH_ECC_IRQHandler(hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  uint32_t flags = LL_FLASH_IsEnabledIT_ECCC(FLASH_GET_INSTANCE(hflash));
  uint32_t its   = LL_FLASH_IsActiveFlag_ECCC(FLASH_GET_INSTANCE(hflash));
  if ((flags != 0U) && (its != 0U))
  {
#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
    hflash->p_ecc_error_cb(hflash, ((LL_FLASH_IsActiveFlag_BK_ECCC(FLASH_GET_INSTANCE(hflash)) == 0U) \
                                    ? HAL_FLASH_BANK_1 : HAL_FLASH_BANK_2));
#else
    HAL_FLASH_ECC_ErrorCallback(hflash, ((LL_FLASH_IsActiveFlag_BK_ECCC(FLASH_GET_INSTANCE(hflash)) == 0U) \
                                         ? HAL_FLASH_BANK_1 : HAL_FLASH_BANK_2));
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */

    LL_FLASH_ClearFlag_ECCC(FLASH_GET_INSTANCE(hflash));
  }
}
#endif /* USE_HAL_FLASH_ECC */
/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group4
  * @{

This subsection provides a set of functions allowing management of the FLASH callbacks:

- The weak function HAL_FLASH_ProgramCpltCallback() can be redefined within user application for the program complete
  callback.

- The weak function HAL_FLASH_EraseByAddrCpltCallback() can be redefined within user application for the erase by
  address complete callback.

- The weak function HAL_FLASH_ErasePageCpltCallback() can be redefined within user application for the erase by page
  complete callback.

- The weak function HAL_FLASH_EraseBankCpltCallback() can be redefined within user application for the bank erase
  complete callback.

- The weak function HAL_FLASH_MassEraseCpltCallback() can be redefined within user application for the mass erase
  complete callback.

- The weak function HAL_FLASH_ErrorCallback() can be redefined within user application for the error callback.

- The weak function HAL_FLASH_ECC_ErrorCallback() can be redefined within user application for the ECC error callback.

- Call the function HAL_FLASH_RegisterProgramCpltCallback() to register the FLASH program complete callback.

- Call the function HAL_FLASH_RegisterEraseByAddrCpltCallback() to register the FLASH erase by address complete
  callback.

- Call the function HAL_FLASH_RegisterErasePageCpltCallback() to register the FLASH erase by page complete callback.

- Call the function HAL_FLASH_RegisterEraseBankCpltCallback() to register the FLASH bank erase complete callback.

- Call the function HAL_FLASH_RegisterMassEraseCpltCallback() to register the FLASH as erase complete callback.

- Call the function HAL_FLASH_RegisterErrorCallback() to register the FLASH error callback.

- Call the function HAL_FLASH_RegisterECCErrorCallback() to register the FLASH ECC error callback.
  */

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/**
  * @brief  FLASH program complete callback.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr Programmed address.
  * @param  size_byte  Programmed size in byte.
  */
__WEAK void HAL_FLASH_ProgramCpltCallback(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(flash_addr);
  STM32_UNUSED(size_byte);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ProgramCpltCallback() can be implemented in the user file */
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief  FLASH erase by address complete callback.
  * @param  hflash     Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr Erased address.
  * @param  size_byte  Erased size in byte.
  */
__WEAK void HAL_FLASH_EraseByAddrCpltCallback(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(flash_addr);
  STM32_UNUSED(size_byte);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_EraseByAddrCpltCallback() can be implemented in the user file */
}
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  FLASH erase by page complete callback.
  * @param  hflash   Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @param  page     Index of erased page.
  * @param  page_nbr Number of erased pages.
  */
__WEAK void HAL_FLASH_ErasePageCpltCallback(hal_flash_handle_t *hflash,
                                            hal_flash_bank_t bank,
                                            uint32_t page,
                                            uint32_t page_nbr)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);
  STM32_UNUSED(page);
  STM32_UNUSED(page_nbr);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ErasePageCpltCallback() can be implemented in the user file */
}
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
/**
  * @brief  FLASH bank erase complete callback.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  */
__WEAK void HAL_FLASH_EraseBankCpltCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_EraseBankCpltCallback() can be implemented in the user file */
}
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
/**
  * @brief  FLASH mass erase complete callback.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
__WEAK void HAL_FLASH_MassEraseCpltCallback(hal_flash_handle_t *hflash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_MassEraseCpltCallback() can be implemented in the user file */
}
#endif /* USE_HAL_FLASH_MASS_ERASE */

/**
  * @brief  FLASH error callback.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one element of the @ref hal_flash_bank_t enumeration.
  */
__WEAK void HAL_FLASH_ErrorCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ErrorCallback() can be implemented in the user file */
}

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/**
  * @brief  FLASH error code correction callback.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval HAL_ERROR Returned when this callback did not manage the ECC interrupt.
  * @retval HAL_OK    Returned when this callback managed the ECC interrupt.
  */
__WEAK hal_status_t HAL_FLASH_ECC_ErrorCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  /*! WARNING: This function must not be modified, when the callback is needed, the
               HAL_FLASH_ECC_ErrorCallback() can be implemented in the user file */

  return HAL_ERROR;
}
#endif /* USE_HAL_FLASH_ECC */

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/**
  * @brief  Register the FLASH program complete callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the program complete callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterProgramCpltCallback(hal_flash_handle_t *hflash, hal_flash_program_cplt_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH program complete callback */
  hflash->p_program_cplt_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief  Register the FLASH erase by address complete callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the erase by address complete callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterEraseByAddrCpltCallback(hal_flash_handle_t *hflash,
                                                       hal_flash_erase_by_addr_cplt_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH erase by address complete callback */
  hflash->p_erase_by_addr_cplt_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  Register the FLASH erase by page complete callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the erase by page complete callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterErasePageCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_erase_page_cplt_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH erase by page complete callback */
  hflash->p_erase_page_cplt_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
/**
  * @brief  Register the FLASH bank erase complete callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the bank erase complete callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterEraseBankCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_erase_bank_cplt_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH bank erase complete callback */
  hflash->p_erase_bank_cplt_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
/**
  * @brief  Register the FLASH mass erase complete callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the mass erase complete callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterMassEraseCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_mass_erase_cplt_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH mass erase complete callback */
  hflash->p_mass_erase_cplt_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_MASS_ERASE */

/**
  * @brief  Register the FLASH error callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the error callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterErrorCallback(hal_flash_handle_t *hflash, hal_flash_error_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH error callback  */
  hflash->p_error_cb = callback;

  return HAL_OK;
}

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/**
  * @brief  Register the FLASH ECC error callback.
  * @param  hflash            Pointer to a @ref hal_flash_handle_t structure.
  * @param  callback          Specifies the error callback.
  * @retval HAL_INVALID_PARAM Returned when an invalid input parameter selected.
  * @retval HAL_OK            Returned when the register of the callback completes successfully.
  */
hal_status_t HAL_FLASH_RegisterECCErrorCallback(hal_flash_handle_t *hflash, hal_flash_ecc_error_cb_t callback)
{
  ASSERT_DBG_PARAM(hflash   != NULL);
  ASSERT_DBG_PARAM(callback != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the FLASH error callback  */
  hflash->p_ecc_error_cb = callback;

  return HAL_OK;
}
#endif /* USE_HAL_FLASH_ECC */

#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group5
  * @{

This subsection provides a set of functions allowing access to FLASH data information and status:

- Call the function HAL_FLASH_GetCurrentOperation() to get the current flash operation.

- Call the function HAL_FLASH_GetCurrentProgrammedAddr() to get the currently being programmed flash address.

- Call the function HAL_FLASH_GetCurrentErasedAddr() to get the currently being erased flash address.

- Call the function HAL_FLASH_GetCurrentErasedPage() to get the currently being erased flash page.

- Call the function HAL_FLASH_GetInterruptedByResetOperationInfo() to get the FLASH interrupted operation information.

- Call the function HAL_FLASH_GetInfo() to get the FLASH information.

- Call the function HAL_FLASH_GetSizeByte() to get the FLASH total size.

- Call the function HAL_FLASH_GetBankNbr() to get the FLASH number of banks.

- Call the function HAL_FLASH_GetBankSizeByte() to get the FLASH bank size.

- Call the function HAL_FLASH_GetUserFlashSizeByte() to get the FLASH USER area size within a bank.

- Call the function HAL_FLASH_EDATA_GetSizeByte() to get the FLASH EDATA area size within a bank

- Call the function HAL_FLASH_GetExtUserFlashSizeByte() to get the FLASH Extended USER area size within a bank.

- Call the function HAL_FLASH_GetUserFlashPageNbr() to get the number of pages in FLASH USER area.

- Call the function HAL_FLASH_EDATA_GetPageNbr() to get the number of pages in FLASH EDATA area.

- Call the function HAL_FLASH_GetExtUserFlashPageNbr() to get the number of pages in FLASH extended USER area.

- Call the function HAL_FLASH_GetUserFlashPageSizeByte() to get the FLASH size of a given page in FLASH USER area.

- Call the function HAL_FLASH_EDATA_GetPageSizeByte() to get the FLASH size of a given page in FLASH EDATA area.

- Call the function HAL_FLASH_GetExtUserFlashPageSizeByte() to get the FLASH size of a given page in FLASH extended
  USER area.

- Call the function HAL_FLASH_GetUserFlashAddrOffset() to get the address offset of a specific page in FLASH USER area.

- Call the function HAL_FLASH_EDATA_GetAddrOffset() to get the address offset of a specific page in FLASH EDATA area.

- Call the function HAL_FLASH_GetExtUserFlashAddrOffset() to get the address offset of a specific page in FLASH extended
  USER area. The offset is computed from the beginning of the FLASH extended USER area.

- Call the function HAL_FLASH_ECC_GetInfo() to get the FLASH ECC information.

- Call the function HAL_FLASH_GetInstance() to get the HAL FLASH instance.

- Call the function HAL_FLASH_GetLLInstance() to get the hardware FLASH instance.

- Call the function HAL_FLASH_GetState() to get the FLASH current state.

- Call the function HAL_FLASH_GetLastErrorCodes() to get the FLASH last errors codes.

- Call the function HAL_FLASH_SetUserData() to set a user data in the FLASH handle.

- Call the function HAL_FLASH_GetUserData() to get a user data from the FLASH handle.
  */


/**
  * @brief  Get the current operation on the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval hal_flash_operation_t Operation ongoing.
  */
hal_flash_operation_t HAL_FLASH_GetCurrentOperation(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(bank);

  return (hflash->ongoing_operation);
}

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/**
  * @brief  Get the last programming operation information.
  *
  * @param  hflash   Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @note   This function must be called during or directly after a ProgramByAddr operation, otherwise the returned
  *         information could not be true anymore.
  * @retval uint32_t FLASH address currently being programmed.
  *                  0xFFFFFFFF if no program by address operation is ongoing on the given bank.
  */
uint32_t HAL_FLASH_GetCurrentProgrammedAddr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  uint32_t prog_flash_addr;
  hal_flash_bank_t current_bank;

  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  prog_flash_addr = hflash->prog_flash_addr;
  current_bank = FLASH_GetBank(hflash, prog_flash_addr);

  if ((hflash->ongoing_operation != HAL_FLASH_PROGRAM) || (current_bank != bank))
  {
    return 0xFFFFFFFFU;
  }

  return prog_flash_addr;
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief  Get the last address erase operation information.
  *
  * @param  hflash   Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  * @note   This function must be called during or directly after a EraseByAddr operation, otherwise the returned
  *         information could not be true anymore.
  * @retval uint32_t Address of the FLASH page currently being erased.
  *                  0xFFFFFFFF if no erase page by address operation is ongoing on the given bank.
  */
uint32_t HAL_FLASH_GetCurrentErasedAddr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  if ((hflash->ongoing_operation != HAL_FLASH_ADDR_ERASE) || (hflash->erase_bank != bank))
  {
    return 0xFFFFFFFFU;
  }

  return hflash->erase_page;
}
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  Get the last page erase operation information.
  * @param  hflash   Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank     This parameter can be one of the following values:
  *                      @arg @ref HAL_FLASH_BANK_1
  *                      @arg @ref HAL_FLASH_BANK_2
  *
  * @note   This function must be called during or directly after an ErasePage operation, otherwise the returned
  *         information could not be true anymore.
  *
  * @retval uint32_t FLASH page currently being erased.
  *                  0xFFFFFFFF if no erase page operation is ongoing on the given bank.
  */
uint32_t HAL_FLASH_GetCurrentErasedPage(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  if ((hflash->ongoing_operation != HAL_FLASH_PAGE_ERASE) || (hflash->erase_bank != bank))
  {
    return 0xFFFFFFFFU;
  }

  return FLASH_GET_HW_PAGE_INDEX(hflash->erase_page, FLASH_PAGE_NB);
}
#endif /* USE_HAL_FLASH_ERASE_PAGE */

/**
  * @brief   Get the interrupted operation information.
  * @param   hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param   p_info Pointer to a @ref hal_flash_interrupted_by_reset_operation_info_t structure.
  * @warning Be aware that if the interrupted operation is either in the EDATA (with EDATA_EN set to 1) or OTP areas,
  *          the reported address is not always accurate due to a hardware limitation.
  *          Indeed, in this case, the correct address can be one of the following:
  *            - Reported address + 0x400
  *            - Reported address + 0x200
  *            - Reported address
  */
void HAL_FLASH_GetInterruptedByResetOperationInfo(const hal_flash_handle_t *hflash,
                                                  hal_flash_interrupted_by_reset_operation_info_t *p_info)
{
  uint32_t area_flag;
  uint32_t offset;
  uint32_t bank;

  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_info != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);

  p_info->operation = (hal_flash_interrupted_operation_t)LL_FLASH_GetOperInterruptedCode(FLASH_GET_INSTANCE(hflash));
  if (p_info->operation != HAL_FLASH_INTERRUPTED_NO_OPERATION)
  {
    offset = LL_FLASH_GetOperInterruptedAddressOffset(FLASH_GET_INSTANCE(hflash));
    area_flag = LL_FLASH_ReadFlag_OP(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_OP_AREA_ALL);
    if ((area_flag & LL_FLASH_FLAG_OTP_OP) != 0U)
    {
      p_info->addr  = FLASH_OTP_BASE;
      /* Convert 16-bit interface address offset to usual 128-bit interface address */
      p_info->addr += (((offset - FLASH_OTP_16BIT_OFFSET_BASE) % FLASH_16BIT_ROW_SIZE) << 2U) \
                      + (((offset - FLASH_OTP_16BIT_OFFSET_BASE) / FLASH_16BIT_ROW_SIZE) * FLASH_OTP_PAGE_SIZE);
    }
    else
    {
      /* Get the bank if not in OTP area */
      bank = (area_flag & LL_FLASH_FLAG_BK_OP);
      if ((area_flag & LL_FLASH_FLAG_DATA_OP) != 0U)
      {
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
        p_info->addr = ((bank == 0U) ? FLASH_EDATA_BASE : (FLASH_EDATA_BASE + FLASH_EDATA_BANK_SIZE));
        /* Convert 16-bit interface address offset to usual 128-bit interface address */
        p_info->addr += ((offset % FLASH_16BIT_ROW_SIZE) << 2U) \
                        + ((offset / FLASH_16BIT_ROW_SIZE) * FLASH_EDATA_PAGE_SIZE);
#else
        p_info->addr = ((bank == 0U) ? FLASH_EXT_USER_BASE : (FLASH_EXT_USER_BASE + FLASH_EXT_USER_BANK_SIZE));
        p_info->addr += (offset << 4U);
#endif /* USE_HAL_FLASH_OB_EDATA */
      }
      else /* If no flag is set, select flash user area */
      {
        p_info->addr = ((bank == 0U) ? FLASH_BASE : (FLASH_BASE + FLASH_BANK_SIZE));
        p_info->addr += (offset << 4U);
      }
    }
  }
}

/**
  * @brief  Get the user FLASH info (size, number and size of banks, number and size of pages).
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  p_info Pointer to a @ref hal_flash_info_t structure.
  */
void HAL_FLASH_GetInfo(const hal_flash_handle_t *hflash, hal_flash_info_t *p_info)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_info != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  p_info->flash_size_byte = FLASH_SIZE + FLASH_EDATA_SIZE;
#else
  p_info->flash_size_byte = FLASH_SIZE + FLASH_EXT_USER_SIZE;
#endif /* USE_HAL_FLASH_OB_EDATA */
  p_info->bank_nbr = FLASH_BANK_NB;

  for (uint32_t bank = 0; bank < FLASH_BANK_NB; bank++)
  {
    p_info->bank[bank].area_nbr = 2;
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
    p_info->bank[bank].bank_size_byte = FLASH_BANK_SIZE + FLASH_EDATA_BANK_SIZE;
#else
    p_info->bank[bank].bank_size_byte = FLASH_BANK_SIZE + FLASH_EXT_USER_BANK_SIZE;
#endif /* USE_HAL_FLASH_OB_EDATA */
    /* USER flash area */
    p_info->bank[bank].user_flash.base_addr      = FLASH_BASE + (bank * FLASH_BANK_SIZE);
    p_info->bank[bank].user_flash.area_size_byte = FLASH_BANK_SIZE;
    p_info->bank[bank].user_flash.page_nbr       = (uint16_t)FLASH_PAGE_NB;

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
    /* EDATA flash area */
    p_info->bank[bank].edata_flash.base_addr      = FLASH_EDATA_BASE + (bank * FLASH_EDATA_BANK_SIZE);
    p_info->bank[bank].edata_flash.area_size_byte = FLASH_EDATA_BANK_SIZE;
    p_info->bank[bank].edata_flash.page_nbr       = (uint16_t)FLASH_EDATA_PAGE_NB;
#else
    /* Extended USER flash area */
    p_info->bank[bank].ext_user_flash.base_addr      = FLASH_EXT_USER_BASE + (bank * FLASH_EXT_USER_BANK_SIZE);
    p_info->bank[bank].ext_user_flash.area_size_byte = FLASH_EXT_USER_BANK_SIZE;
    p_info->bank[bank].ext_user_flash.page_nbr       = (uint16_t)FLASH_EXT_USER_PAGE_NB;
#endif /* USE_HAL_FLASH_OB_EDATA */
  }

  STM32_UNUSED(hflash);
}

/**
  * @brief  Get the total FLASH size.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval The size in bytes of total FLASH.
  */
uint32_t HAL_FLASH_GetSizeByte(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  return (FLASH_SIZE + FLASH_EDATA_SIZE);
#else
  return (FLASH_SIZE + FLASH_EXT_USER_SIZE);
#endif /* USE_HAL_FLASH_OB_EDATA */
}

/**
  * @brief  Get the number of FLASH banks.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval The number of bank.
  */
uint8_t HAL_FLASH_GetBankNbr(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  return (uint8_t) FLASH_BANK_NB;
}

/**
  * @brief  Get the total size in bytes of the given FLASH bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The total size in bytes of the given bank.
  */
uint32_t HAL_FLASH_GetBankSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  return (FLASH_BANK_SIZE + FLASH_EDATA_BANK_SIZE);
#else
  return (FLASH_BANK_SIZE + FLASH_EXT_USER_BANK_SIZE);
#endif /* USE_HAL_FLASH_OB_EDATA */
}


/**
  * @brief  Get the size in bytes of the user flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The size in bytes of the user flash area of the given bank.
  */
uint32_t HAL_FLASH_GetUserFlashSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return FLASH_BANK_SIZE;
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Get the size in bytes of the EDATA flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The size in bytes of the EDATA flash area of the given bank.
  */
uint32_t HAL_FLASH_EDATA_GetSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return FLASH_EDATA_BANK_SIZE;
}
#else
/**
  * @brief  Get the size in bytes of the extended user flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The size in bytes of the extended user flash area of the given bank.
  */
uint32_t HAL_FLASH_GetExtUserFlashSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return FLASH_EXT_USER_BANK_SIZE;
}
#endif /* USE_HAL_FLASH_OB_EDATA */

/**
  * @brief  Get the number of pages of the user flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The number of pages of the user flash area of the given bank.
  */
uint16_t HAL_FLASH_GetUserFlashPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return (uint16_t) FLASH_PAGE_NB;
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Get the number of pages of the EDATA flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The number of pages of the EDATA flash area of the given bank.
  */
uint16_t HAL_FLASH_EDATA_GetPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return (uint16_t) FLASH_EDATA_PAGE_NB;
}
#else
/**
  * @brief  Get the number of pages of the extended user flash area of the given bank.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @retval The number of pages of the extended user flash area of the given bank.
  */
uint16_t HAL_FLASH_GetExtUserFlashPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  return (uint16_t) FLASH_EXT_USER_PAGE_NB;
}
#endif /* USE_HAL_FLASH_OB_EDATA */

/**
  * @brief  Get the size in bytes of the given user area page.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @param  page   Selected page.
  * @retval The size in bytes of the given user area page.
  */
uint32_t HAL_FLASH_GetUserFlashPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);
  STM32_UNUSED(page);

  return FLASH_PAGE_SIZE;
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Get the size in bytes of the given EDATA area page.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @param  page   Selected page.
  * @retval The size in bytes of the given EDATA area page.
  */
uint32_t HAL_FLASH_EDATA_GetPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_EDATA_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);
  STM32_UNUSED(page);

  return FLASH_EDATA_PAGE_SIZE;
}
#else
/**
  * @brief  Get the size in bytes of the given extended user area page.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @param  page   Selected page
  * @retval The size in bytes of the given extended user area page.
  */
uint32_t HAL_FLASH_GetExtUserFlashPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_EXT_USER_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);
  STM32_UNUSED(page);

  return FLASH_EXT_USER_PAGE_SIZE;
}
#endif /* USE_HAL_FLASH_OB_EDATA */

/**
  * @brief  Get the offset address of the given user area page.
  * @param  hflash  Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank    This parameter can be one of the following values:
  *                     @arg @ref HAL_FLASH_BANK_1
  *                     @arg @ref HAL_FLASH_BANK_2
  * @param  page    Page within selected bank.
  * @retval Address The offset address of the given user area page.
  */
uint32_t HAL_FLASH_GetUserFlashAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);

  return ((uint32_t)bank * FLASH_BANK_SIZE) + (page * FLASH_PAGE_SIZE);
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
/**
  * @brief  Get the offset address of the given EDATA area page.
  * @param  hflash  Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank    This parameter can be one of the following values:
  *                     @arg @ref HAL_FLASH_BANK_1
  *                     @arg @ref HAL_FLASH_BANK_2
  * @param  page    Page within selected bank.
  * @retval Address The offset address of the given EDATA area page.
  */
uint32_t HAL_FLASH_EDATA_GetAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_EDATA_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);

  return ((uint32_t)bank * FLASH_EDATA_BANK_SIZE) + (page * FLASH_EDATA_PAGE_SIZE);
}
#else
/**
  * @brief  Get the offset address of the given extended user area page.
  * @param  hflash  Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank    This parameter can be one of the following values:
  *                     @arg @ref HAL_FLASH_BANK_1
  *                     @arg @ref HAL_FLASH_BANK_2
  * @param  page    Page within selected bank.
  * @retval Address The offset address of the given extended user area page.
  */
uint32_t HAL_FLASH_GetExtUserFlashAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_PARAM(page < FLASH_EXT_USER_PAGE_NB);
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);

  return ((uint32_t)bank * FLASH_EXT_USER_BANK_SIZE) + (page * FLASH_EXT_USER_PAGE_SIZE);
}
#endif /* USE_HAL_FLASH_OB_EDATA */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/**
  * @brief   Get the ECC information.
  * @param   hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param   bank   This parameter can be one of the following values:
  *                    @arg @ref HAL_FLASH_BANK_1
  *                    @arg @ref HAL_FLASH_BANK_2
  * @param   p_info Pointer to a @ref hal_flash_ecc_info_t structure.
  * @warning Be aware that if the error is a ECC single error located either in the EDATA (with EDATA_EN set to 1)
  *          or OTP areas, the reported address is not always accurate due to a hardware limitation.
  *          Indeed, in this case, the correct address can be one of the following:
  *            - Reported address + 0x400
  *            - Reported address + 0x200
  *            - Reported address
  */
void HAL_FLASH_ECC_GetInfo(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, hal_flash_ecc_info_t *p_info)
{
  uint32_t offset;
  uint32_t ecc_bank;
  uint32_t area_flag = 0U;
  uint32_t word_number = 0U;
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(p_info != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_BANK(bank));
  ASSERT_DBG_STATE(hflash->global_state, (uint32_t)HAL_FLASH_STATE_IDLE | (uint32_t)HAL_FLASH_STATE_ACTIVE);

  STM32_UNUSED(hflash);
  STM32_UNUSED(bank);

  if (LL_FLASH_IsActiveFlag_ECCD(FLASH_GET_INSTANCE(hflash)) != 0U)
  {
    p_info->type = HAL_FLASH_ECC_DOUBLE;
    p_info->status = HAL_FLASH_ECC_NOT_CORRECTED;

    offset = LL_FLASH_GetECCDAddressOffset(FLASH_GET_INSTANCE(hflash));
    /* get ECC error data information in case of double error detection*/
    word_number = LL_FLASH_GetECCDWordNumber(FLASH_GET_INSTANCE(hflash));
    p_info->data = LL_FLASH_GetECCDData(FLASH_GET_INSTANCE(hflash));

    area_flag = LL_FLASH_ReadFlag_ECCD(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ECC_AREA_ALL);
  }
  else if (LL_FLASH_IsActiveFlag_ECCC(FLASH_GET_INSTANCE(hflash)) != 0U)
  {
    p_info->type = HAL_FLASH_ECC_SINGLE;
    p_info->status = HAL_FLASH_ECC_CORRECTED;
    offset = LL_FLASH_GetECCCAddressOffset(FLASH_GET_INSTANCE(hflash));
    /* No ECC error data information for single correction */
    p_info->data = 0U;

    area_flag = LL_FLASH_ReadFlag_ECCC(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ECC_AREA_ALL);
  }
  else /* No ECC */
  {
    p_info->type = HAL_FLASH_ECC_NONE;
    return;
  }

  if ((area_flag & LL_FLASH_FLAG_OTP_ECC) != 0U)
  {
    p_info->addr  = FLASH_OTP_BASE;
    /* Convert 16-bit interface address offset to usual 128-bit interface address */
    p_info->addr += (((offset - FLASH_OTP_16BIT_OFFSET_BASE) % FLASH_16BIT_ROW_SIZE) << 2U) \
                    + (((offset - FLASH_OTP_16BIT_OFFSET_BASE) / FLASH_16BIT_ROW_SIZE) * FLASH_OTP_PAGE_SIZE) \
                    + ((word_number / 2U) * FLASH_OTP_16BIT_OFFSET_BASE) + ((word_number % 2UL) * 0x2UL);
  }
  else
  {
    /* Get the ECC error bank if not in OTP area */
    ecc_bank = (area_flag & LL_FLASH_FLAG_BK_ECC);
    if ((area_flag & LL_FLASH_FLAG_SYSF_ECC) != 0U) /* System FLASH */
    {
      p_info->addr = ((ecc_bank == 0U) ? FLASH_SYSTEM_BASE : (FLASH_SYSTEM_BASE + (FLASH_SYSTEM_SIZE >> 1U)));
      p_info->addr += (offset << 4U);
    }
    else if ((area_flag & LL_FLASH_FLAG_EDATA_ECC) != 0U)
    {
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
      p_info->addr = ((ecc_bank == 0U) ? FLASH_EDATA_BASE : (FLASH_EDATA_BASE + FLASH_EDATA_BANK_SIZE));
      /* Convert 16-bit interface address offset to usual 128-bit interface address */
      p_info->addr += ((offset % FLASH_16BIT_ROW_SIZE) << 2U) \
                      + ((offset / FLASH_16BIT_ROW_SIZE) * FLASH_EDATA_PAGE_SIZE) \
                      + ((word_number / 2U) * FLASH_OTP_16BIT_OFFSET_BASE) + ((word_number % 2UL) * 0x2UL);
#else
      p_info->addr = ((ecc_bank == 0U) ? FLASH_EXT_USER_BASE : (FLASH_EXT_USER_BASE + FLASH_EXT_USER_BANK_SIZE));
      p_info->addr += (offset << 4U);
#endif /* USE_HAL_FLASH_OB_EDATA */
    }
    else /* User FLASH */
    {
      p_info->addr = ((ecc_bank == 0U) ? FLASH_BASE : (FLASH_BASE + FLASH_BANK_SIZE));
      p_info->addr += (offset << 4U);
    }
  }
}
#endif /* USE_HAL_FLASH_ECC */

/**
  * @brief  Get the HAL FLASH instance.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval The HAL FLASH instance.
  */
hal_flash_t HAL_FLASH_GetInstance(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_INSTANCE(hflash)));

  return (hflash->instance);
}

/**
  * @brief  Get the hardware FLASH instance.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval The hardware FLASH instance.
  */
FLASH_TypeDef *HAL_FLASH_GetLLInstance(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);
  ASSERT_DBG_PARAM(IS_FLASH_INSTANCE(FLASH_GET_INSTANCE(hflash)));

  return ((FLASH_TypeDef *)((uint32_t)((hflash)->instance)));
}

/**
  * @brief  Get the FLASH current state.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @retval State  The FLASH global state.
  */
hal_flash_state_t HAL_FLASH_GetState(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  return (hflash->global_state);
}

#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
/**
  * @brief  Get the FLASH last error codes.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @retval Error codes The FLASH last error codes.
  */
uint32_t HAL_FLASH_GetLastErrorCodes(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  return (hflash->last_error_codes);
}
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

#if defined (USE_HAL_FLASH_USER_DATA) && (USE_HAL_FLASH_USER_DATA == 1)
/**
  * @brief  Store User Data pointer within the FLASH handle.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param  p_user_data Pointer to the user data.
  */
void HAL_FLASH_SetUserData(hal_flash_handle_t *hflash, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  hflash->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the user data pointer from the FLASH handle.
  * @param  hflash  Pointer to a @ref hal_flash_handle_t structure.
  * @retval Pointer to the user data.
  */
const void *HAL_FLASH_GetUserData(const hal_flash_handle_t *hflash)
{
  ASSERT_DBG_PARAM(hflash != NULL);

  return (hflash->p_user_data);
}
#endif /* USE_HAL_FLASH_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup FLASH_Private_Functions
  * @{
  */

#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
/**
  * @brief  Check if the FLASH operation for a new operation.
  * @param  hflash       Pointer to a @ref hal_flash_handle_t structure.
  * @retval HAL_BUSY     FLASH is busy.
  * @retval HAL_ERROR    FLASH is locked or an error is pending.
  * @retval HAL_OK       FLASH is ready.
  */
static hal_status_t FLASH_IsReadyForOperation(hal_flash_handle_t *hflash)
{
  uint32_t flags = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_STATUS_ALL | LL_FLASH_FLAG_ERRORS_ALL);

  if ((LL_FLASH_IsLocked(FLASH_GET_INSTANCE(hflash)) != 0U) || ((flags & LL_FLASH_FLAG_ERRORS_ALL) != 0U))
  {
    return HAL_ERROR;
  }
  else if ((flags & LL_FLASH_FLAG_STATUS_ALL) != 0U)
  {
    return HAL_BUSY;
  }
  else
  {
    return HAL_OK;
  }
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */

/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  hflash       Pointer to a @ref hal_flash_handle_t structure.
  * @param  timeout_ms   Maximum flash operation timeout..
  * @retval HAL_TIMEOUT  User timeout.
  * @retval HAL_ERROR    Error during operation.
  * @retval HAL_OK       No operation to be waiting.
  */
static hal_status_t FLASH_WaitForEndOfOperation(hal_flash_handle_t *hflash, uint32_t timeout_ms)
{
  uint32_t tickstart;
  uint32_t error_code;

  tickstart = HAL_GetTick();
  while (LL_FLASH_IsActiveFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_STATUS_ALL) != 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        if (LL_FLASH_IsActiveFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_STATUS_ALL) != 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }

  error_code = LL_FLASH_ReadFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL);

  if (error_code != 0U)
  {
#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    /* Fill the FLASH handle with the detected error codes */
    FLASH_FillErrorCode(hflash, error_code);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Check error flags */
    LL_FLASH_ClearFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL);

    return HAL_ERROR;
  }

  /* Clear FLASH End of Operation pending bit */
  LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));

  /* If no error flag is set */
  return HAL_OK;
}

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)

/**
  * @brief  Get the program unit function according to given programming mode.
  * @param  programming_mode This parameter can be one element of the @ref hal_flash_program_mode_t enumeration.
  * @retval Pointer to the program unit function.
  */
static hal_flash_program_unit_func_t FLASH_GetProgramUnitFunction(hal_flash_program_mode_t programming_mode)
{
  hal_flash_program_unit_func_t p_prog_unit_func;
  if (FLASH_GET_PROGRAM_SIZE(programming_mode) >= FLASH_WORD_SIZE_BYTE)
  {
    /* Program by successive 32-bit accesses */
    p_prog_unit_func = FLASH_ProgramByWord;
  }
  else if (programming_mode == HAL_FLASH_PROGRAM_HALFWORD)
  {
    /* Program by successive 16-bit accesses */
    p_prog_unit_func = FLASH_ProgramByHalfWord;
  }
  else
  {
    /* Program by successive 8-bit accesses */
    p_prog_unit_func = FLASH_ProgramByByte;
  }

  return p_prog_unit_func;
}

/**
  * @brief  Update the adaptive programming mode and compute the number of elementary writes to perform.
  * @param  hflash   Pointer to a @ref hal_flash_handle_t structure.
  */
static void FLASH_UpdateAdaptiveProgMode(hal_flash_handle_t *hflash)
{
  hal_flash_program_mode_t programming_mode;
  uint32_t count = hflash->count;
  hal_flash_program_unit_func_t p_prog_unit_func;

  if (FLASH_IsFlashUSERAddr(hflash->prog_flash_addr, 1U) != 0U)
  {
    if (count >= FLASH_QUADWORD_SIZE_BYTE)
    {
      programming_mode = HAL_FLASH_PROGRAM_QUADWORD;
      count = FLASH_QUADWORD_SIZE_BYTE;
      p_prog_unit_func = FLASH_ProgramByWord;
    }
    else
    {
      programming_mode = (((count % FLASH_DOUBLEWORD_SIZE_BYTE) == 0U) ? HAL_FLASH_PROGRAM_DOUBLEWORD :
                          (((count % FLASH_WORD_SIZE_BYTE) == 0U) ? HAL_FLASH_PROGRAM_WORD :
                           (((count % FLASH_HALFWORD_SIZE_BYTE) == 0U) ? HAL_FLASH_PROGRAM_HALFWORD :
                            HAL_FLASH_PROGRAM_BYTE)));
      p_prog_unit_func = FLASH_GetProgramUnitFunction(programming_mode);
    }
  }
  else
  {
    if ((count % FLASH_WORD_SIZE_BYTE) == 0U)
    {
      programming_mode = HAL_FLASH_PROGRAM_WORD;
      count = FLASH_WORD_SIZE_BYTE;
      p_prog_unit_func = FLASH_ProgramByWord;
    }
    else
    {
      programming_mode = HAL_FLASH_PROGRAM_HALFWORD;
      count = FLASH_HALFWORD_SIZE_BYTE;
      p_prog_unit_func = FLASH_ProgramByHalfWord;
    }
  }

  hflash->programming_mode = programming_mode;
  hflash->prog_size_byte = count;
  hflash->p_prog_unit_func = p_prog_unit_func;
}

/**
  * @brief  Program up to a quad-word (128-bit) at a specified address by successive write operations in polling mode.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr  FLASH address to program.
  * @param  p_data      Pointer to the data to be programmed.
  * @param  size_byte   Size of the data to be programmed.
  * @param  timeout_ms  Maximum user timeout value for program operation in milliseconds.
  * @retval HAL_ERROR   Error during programming.
  * @retval HAL_TIMEOUT User timeout.
  * @retval HAL_BUSY    FLASH is busy and cannot start a new program operation.
  * @retval HAL_OK      No operation to be waiting.
  */
static hal_status_t FLASH_Program(hal_flash_handle_t *hflash,
                                  uint32_t flash_addr,
                                  const uint32_t *p_data,
                                  uint32_t size_byte,
                                  uint32_t timeout_ms)
{
  uint32_t prog_size;
  uint32_t tickstart = HAL_GetTick();

  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information in the handle */
    hflash->ongoing_operation = HAL_FLASH_PROGRAM;
    hflash->prog_data_addr    = (uint32_t)p_data;
    hflash->prog_flash_addr   = flash_addr;
    hflash->count             = size_byte;
    hflash->size              = size_byte;

    LL_FLASH_EnableProgramming(FLASH_GET_INSTANCE(hflash));

    while ((hflash->count > 0U) && (status == HAL_OK))
    {
      if (hflash->is_adaptive_prog != 0U)
      {
        FLASH_UpdateAdaptiveProgMode(hflash);
      }

      hflash->p_prog_unit_func(hflash);

      status = FLASH_WaitForEndOfOperation(hflash, FLASH_PROGRAM_OPERATION_QUADWORD_TIMEOUT);

      /* Update handle programming elements and reset programming flags if no error occurred */
      if (status == HAL_OK)
      {
        prog_size = hflash->prog_size_byte;

        if (hflash->count > prog_size)
        {
          hflash->count           -= prog_size;
          hflash->prog_data_addr  += prog_size;
          hflash->prog_flash_addr += prog_size;
        }
        else
        {
          hflash->count = 0U;
          hflash->prog_data_addr  += (prog_size - 1U);
          hflash->prog_flash_addr += (prog_size - 1U);
        }

        if ((hflash->count > 0U) && (timeout_ms != HAL_MAX_DELAY))
        {
          if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
          {
            status = HAL_TIMEOUT;
          }
        }
      }
    }

    LL_FLASH_DisableProgramming(FLASH_GET_INSTANCE(hflash));

    hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
    hflash->global_state = HAL_FLASH_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Program up to a quad-word (128-bit) at a specified address by successive write operations.
  * @param  hflash          Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr      FLASH address to program.
  * @param  p_data          Pointer to the data to be programmed.
  * @param  size_byte       Size of the data to be programmed.
  * @retval HAL_ERROR       Error before starting program operation.
  * @retval HAL_BUSY        FLASH is busy and cannot start a new program operation.
  * @retval HAL_OK          Program operation started.
  */
static hal_status_t FLASH_Program_IT(hal_flash_handle_t *hflash,
                                     uint32_t flash_addr,
                                     const uint32_t *p_data,
                                     uint32_t size_byte)
{
  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_PROGRAM;
    hflash->prog_data_addr    = (uint32_t)p_data;
    hflash->prog_flash_addr   = flash_addr;
    hflash->count             = size_byte;
    hflash->size              = size_byte;

    LL_FLASH_EnableProgramming(FLASH_GET_INSTANCE(hflash));

    if (hflash->is_adaptive_prog != 0U)
    {
      FLASH_UpdateAdaptiveProgMode(hflash);
    }

    LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
    LL_FLASH_EnableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);

    hflash->p_prog_unit_func(hflash);
  }

  return status;
}

/**
  * @brief  Program up to a quad-word (128-bit) at a specified address by successive word write operations.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
static void FLASH_ProgramByWord(hal_flash_handle_t *hflash)
{
  uint32_t increment_index = hflash->prog_size_byte / FLASH_WORD_SIZE_BYTE;
  uint32_t *dest_addr      = (uint32_t *)(hflash->prog_flash_addr);
  uint32_t *src_addr       = (uint32_t *)(hflash->prog_data_addr);
  uint32_t primask_bit;

  /* Enter critical section: Disable interrupts to avoid any interruption during the loop */
  primask_bit = __get_PRIMASK();
  __disable_irq();

  /* Program */
  do
  {
    *dest_addr = *src_addr;
    dest_addr++;
    src_addr++;
    increment_index--;
  } while (increment_index != 0U);

  /* Set FW bit when less than a quad-word is written */
  if (hflash->programming_mode != HAL_FLASH_PROGRAM_QUADWORD)
  {
    LL_FLASH_EnableForceWrite(FLASH_GET_INSTANCE(hflash));
  }
  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);
}

/**
  * @brief  Program up to a quad-word (128-bit) at a specified address by successive word write operations.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
static void FLASH_ProgramByHalfWord(hal_flash_handle_t *hflash)
{
  uint32_t increment_index = hflash->prog_size_byte / FLASH_HALFWORD_SIZE_BYTE;
  uint32_t primask_bit;
  uint16_t *dest_addr = (uint16_t *)(hflash->prog_flash_addr);
  uint16_t *src_addr  = (uint16_t *)(hflash->prog_data_addr);

  /* Enter critical section: Disable interrupts to avoid any interruption during the loop */
  primask_bit = __get_PRIMASK();
  __disable_irq();

  /* Program */
  do
  {
    *dest_addr = *src_addr;
    dest_addr++;
    src_addr++;
    increment_index--;
  } while (increment_index != 0U);

  /* Set FW bit when less than a quad-word is written */
  if ((FLASH_IsFlashUSERAddr(hflash->prog_flash_addr, 1U) != 0U))
  {
    LL_FLASH_EnableForceWrite(FLASH_GET_INSTANCE(hflash));
  }
  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);
}

/**
  * @brief  Program up to a quad-word (128-bit) at a specified address by successive byte write operations.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  */
static void FLASH_ProgramByByte(hal_flash_handle_t *hflash)
{
  uint32_t increment_index = hflash->prog_size_byte;
  uint32_t primask_bit;
  uint8_t *dest_addr = (uint8_t *)(hflash->prog_flash_addr);
  uint8_t *src_addr  = (uint8_t *)(hflash->prog_data_addr);

  /* Enter critical section: Disable interrupts to avoid any interruption during the loop */
  primask_bit = __get_PRIMASK();
  __disable_irq();

  /* Program */
  do
  {
    *dest_addr = *src_addr;
    dest_addr++;
    src_addr++;
    increment_index--;
  } while (increment_index != 0U);

  /* Set FW bit when less than a quad-word is written */
  if (FLASH_IsFlashUSERAddr(hflash->prog_flash_addr, 1U) != 0U)
  {
    LL_FLASH_EnableForceWrite(FLASH_GET_INSTANCE(hflash));
  }
  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/**
  * @brief Get the size in bytes of the page which contains the given flash address.
  * @param flash_addr Address of the page
  * @retval The size of the page containing the given address.
  */
static uint32_t FLASH_GetPageSizeByAddr(uint32_t flash_addr)
{
  uint32_t page_size;

  if ((flash_addr >= FLASH_BASE) && (flash_addr < (FLASH_BASE + FLASH_SIZE))) /* Main User Flash*/
  {
    page_size = FLASH_PAGE_SIZE;
  }
  else
  {
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
    page_size = FLASH_EDATA_PAGE_SIZE;
#else
    page_size = FLASH_EXT_USER_PAGE_SIZE;
#endif /* USE_HAL_FLASH_OB_EDATA */
  }

  return page_size;
}

/**
  * @brief Get the page index associated to the given flash address.
  * @param flash_addr Flash Address.
  * @retval uint32_t The index of the page containing the given address.
  */
static uint32_t FLASH_GetPageIndexByAddr(uint32_t flash_addr)
{
  uint32_t max_page_index;
  uint32_t page_index;

  if ((flash_addr >= FLASH_BASE) && (flash_addr < (FLASH_BASE + FLASH_SIZE))) /* Main User Flash*/
  {
    page_index = (((flash_addr - FLASH_BASE) / FLASH_PAGE_SIZE));
    max_page_index = FLASH_PAGE_NB;
  }
  else
  {
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
    /* EDATA area configured as data flash */
    page_index = ((flash_addr - FLASH_EDATA_BASE) / FLASH_EDATA_PAGE_SIZE);
    max_page_index = FLASH_EDATA_PAGE_NB;
#else
    /* EDATA area configured as user flash */
    page_index = ((flash_addr - FLASH_EXT_USER_BASE) / FLASH_EXT_USER_PAGE_SIZE);
    max_page_index = FLASH_EXT_USER_PAGE_NB;
#endif /* USE_HAL_FLASH_OB_EDATA */
  }

  return FLASH_GET_HW_PAGE_INDEX(page_index, max_page_index);
}

/**
  * @brief  Erase pages of FLASH memory by address in polling mode.
  * @param  hflash      Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr  FLASH address to erase.
  * @param  size_byte   Size of the data to be erased.
  * @param  page_size   Page size of the erased area.
  * @param  timeout_ms  Maximum user timeout value for erase operation in milliseconds.
  * @retval HAL_ERROR   Error during erasing.
  * @retval HAL_TIMEOUT User timeout.
  * @retval HAL_BUSY    FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK      No operation to be waiting.
  */
static hal_status_t FLASH_EraseAddr(hal_flash_handle_t *hflash,
                                    uint32_t flash_addr,
                                    uint32_t size_byte,
                                    uint32_t page_size,
                                    uint32_t timeout_ms)
{
  uint32_t erased_size;
  uint32_t tickstart;
  uint32_t erase_area;
  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_ADDR_ERASE;
    hflash->erase_bank = FLASH_GetBank(hflash, flash_addr);
    hflash->erase_page = flash_addr;
    hflash->count      = size_byte;
    hflash->size       = size_byte;
    erased_size = page_size - (hflash->erase_page % page_size);
    erase_area  = FLASH_GET_ERASE_AREA_ADDR(hflash->erase_page);

    while ((hflash->count > 0U) && (status == HAL_OK))
    {
      tickstart = HAL_GetTick();

      LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), \
                              erase_area, FLASH_GetPageIndexByAddr(hflash->erase_page));

      status = FLASH_WaitForEndOfOperation(hflash, FLASH_ERASE_PAGE_OPERATION_TIMEOUT);

      if (status == HAL_OK)
      {
        if (hflash->count > erased_size)
        {
          hflash->count -= erased_size;
          hflash->erase_page += erased_size;
          erased_size = page_size;
        }
        else
        {
          erased_size = hflash->count;
          hflash->erase_page += erased_size - 1U;
          hflash->count = 0U;
        }

        if ((hflash->count > 0U) && (timeout_ms != HAL_MAX_DELAY))
        {
          if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
          {
            status = HAL_TIMEOUT;
          }
        }
      }
    }

    LL_FLASH_DisablePageErase(FLASH_GET_INSTANCE(hflash));

    hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
    hflash->global_state = HAL_FLASH_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Erase pages of FLASH memory by address in interrupt mode.
  * @param  hflash         Pointer to a @ref hal_flash_handle_t structure.
  * @param  flash_addr     FLASH address to erase.
  * @param  size_byte      Size of the data to be erased.
  * @retval HAL_ERROR      Error during erasing.
  * @retval HAL_BUSY       FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK         No operation to be waiting.
  */
static hal_status_t FLASH_EraseAddr_IT(hal_flash_handle_t *hflash,
                                       uint32_t flash_addr,
                                       uint32_t size_byte)
{
  hal_status_t status = FLASH_IsReadyForOperation(hflash);
  uint32_t erase_area = FLASH_GET_ERASE_AREA_ADDR(flash_addr);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);
#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_ADDR_ERASE;
    hflash->erase_bank = FLASH_GetBank(hflash, flash_addr);
    hflash->erase_page = flash_addr;
    hflash->count      = size_byte;
    hflash->size       = size_byte;

    LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
    LL_FLASH_EnableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
    LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), \
                            erase_area, FLASH_GetPageIndexByAddr(hflash->erase_page));
  }

  return status;
}
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/**
  * @brief  Erase pages of FLASH memory by address in polling mode.
  * @param  hflash       Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank         Bank to erase.
  *                      This parameter can be one of the following values:
  *                          @arg @ref HAL_FLASH_BANK_1
  *                          @arg @ref HAL_FLASH_BANK_2
  * @param  start_page   Index of the first page to erase.
  * @param  page_nbr     The number of pages to erase.
  * @param  erase_area   Area to erase.
  * @param  timeout_ms   Maximum user timeout value for erase operation in milliseconds.
  * @retval HAL_ERROR    Error during erasing.
  * @retval HAL_TIMEOUT  User timeout.
  * @retval HAL_BUSY     FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK       No operation to be waiting.
  */
static hal_status_t FLASH_ErasePage(hal_flash_handle_t *hflash,
                                    hal_flash_bank_t bank,
                                    uint32_t start_page,
                                    uint32_t page_nbr,
                                    uint32_t erase_area,
                                    uint32_t timeout_ms)
{
  uint32_t tickstart;
  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_PAGE_ERASE;
    hflash->erase_bank = bank;
    hflash->erase_page = start_page;
    hflash->count      = page_nbr;
    hflash->size       = page_nbr;

    while ((hflash->count > 0U) && (status == HAL_OK))
    {
      tickstart = HAL_GetTick();
      LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), erase_area, \
                              FLASH_GET_HW_PAGE_INDEX(hflash->erase_page, FLASH_PAGE_NB));

      status = FLASH_WaitForEndOfOperation(hflash, FLASH_ERASE_PAGE_OPERATION_TIMEOUT);

      if (status == HAL_OK)
      {
        hflash->count--;
        if (hflash->count > 0U)
        {
          hflash->erase_page++;

          if (timeout_ms != HAL_MAX_DELAY)
          {
            if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
            {
              status = HAL_TIMEOUT;
            }
          }
        }
      }
    }

    LL_FLASH_DisablePageErase(FLASH_GET_INSTANCE(hflash));

    hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;
    hflash->global_state = HAL_FLASH_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Erase pages of FLASH memory by address in interrupt mode.
  * @param  hflash       Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank         Bank to erase.
  *                      This parameter can be one of the following values:
  *                          @arg @ref HAL_FLASH_BANK_1
  *                          @arg @ref HAL_FLASH_BANK_2
  * @param  start_page   Index of the first page to erase.
  * @param  page_nbr     Number of pages to erase.
  * @param  erase_area   Area to erase.
  * @retval HAL_ERROR    Error during erasing.
  * @retval HAL_BUSY     FLASH is busy and cannot start a new erase operation.
  * @retval HAL_OK       No operation to be waiting.
  */
static hal_status_t FLASH_ErasePage_IT(hal_flash_handle_t *hflash,
                                       hal_flash_bank_t bank,
                                       uint32_t start_page,
                                       uint32_t page_nbr,
                                       uint32_t erase_area)
{
  hal_status_t status = FLASH_IsReadyForOperation(hflash);

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hflash, global_state, HAL_FLASH_STATE_IDLE, HAL_FLASH_STATE_ACTIVE);

#if defined (USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
    hflash->last_error_codes = HAL_FLASH_ERROR_NONE;
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

    /* Fill the operation information structure */
    hflash->ongoing_operation = HAL_FLASH_PAGE_ERASE;
    hflash->erase_bank = bank;
    hflash->erase_page = start_page;
    hflash->count      = page_nbr;
    hflash->size       = page_nbr;

    LL_FLASH_ClearFlag_EOP(FLASH_GET_INSTANCE(hflash));
    LL_FLASH_EnableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);
    LL_FLASH_StartErasePage(FLASH_GET_INSTANCE(hflash), FLASH_GET_ERASE_BANK(hflash->erase_bank), erase_area, \
                            FLASH_GET_HW_PAGE_INDEX(hflash->erase_page, FLASH_PAGE_NB));
  }

  return status;
}
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
/**
  * @brief  Update last error element of FLASH handle.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  flags  Bitfield containing the error flags.
  */
static void FLASH_FillErrorCode(hal_flash_handle_t *hflash, uint32_t flags)
{
  uint32_t error_codes = 0U;

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  if ((flags & LL_FLASH_FLAG_STRBERR) != 0U)
  {
    error_codes |= HAL_FLASH_ERROR_STRB;
  }

  if ((flags & LL_FLASH_FLAG_INCERR) != 0U)
  {
    error_codes |= HAL_FLASH_ERROR_INC;
  }
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
  if ((flags & LL_FLASH_FLAG_WRPERR) != 0U)
  {
    error_codes |= HAL_FLASH_ERROR_WRP;
  }
  if ((flags & LL_FLASH_FLAG_PGSERR) != 0U)
  {
    error_codes |= HAL_FLASH_ERROR_PGS;
  }
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */

  hflash->last_error_codes |= error_codes;
}
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1))
/**
  * @brief Get the bank which contains the given flash address.
  * @param hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param flash_addr Flash Address
  * @retval hal_flash_bank_t the bank containing the given address.
  *                    This parameter is one element of @ref hal_flash_bank_t enumeration.
  */
static hal_flash_bank_t FLASH_GetBank(const hal_flash_handle_t *hflash, uint32_t flash_addr)
{
  hal_flash_bank_t low_addr_range_bank, high_addr_range_bank;

  if (LL_FLASH_OB_IsBankSwapped(FLASH_GET_INSTANCE(hflash)) == 0U)
  {
    low_addr_range_bank = HAL_FLASH_BANK_1;
    high_addr_range_bank = HAL_FLASH_BANK_2;
  }
  else
  {
    low_addr_range_bank = HAL_FLASH_BANK_2;
    high_addr_range_bank = HAL_FLASH_BANK_1;
  }

  if (flash_addr < (FLASH_BASE + FLASH_SIZE)) /* Main User Flash*/
  {
    return (((flash_addr - FLASH_BASE) < FLASH_BANK_SIZE) ? low_addr_range_bank : high_addr_range_bank);
  }
  else /* EDATA Flash */
  {
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
    return (((flash_addr - FLASH_EDATA_BASE) < FLASH_EDATA_BANK_SIZE) ? low_addr_range_bank : high_addr_range_bank);
#else
    return (((flash_addr - FLASH_EXT_USER_BASE) < FLASH_EXT_USER_BANK_SIZE) \
            ? low_addr_range_bank : high_addr_range_bank);
#endif /* USE_HAL_FLASH_OB_EDATA */
  }
}

/**
  * @brief  Check if a FLASH address is part of the user FLASH area.
  * @param  flash_addr The FLASH address to check.
  * @param  check_program_alignment The flag indicating if the address must be aligned for programming.
  * @retval The status of the FLASH address test (1: address is in user FLASH, 0: otherwise)
  */
static uint32_t FLASH_IsFlashUSERAddr(uint32_t flash_addr, uint8_t check_program_alignment)
{
  uint32_t is_user_addr = ((flash_addr >= FLASH_BASE) && (flash_addr < (FLASH_BASE + FLASH_SIZE))) ? 1U : 0U;

#if !defined(USE_HAL_FLASH_OB_EDATA) || (USE_HAL_FLASH_OB_EDATA == 0)
  is_user_addr = ((is_user_addr != 0U) \
                  || ((flash_addr >= FLASH_EXT_USER_BASE) \
                      && (flash_addr < (FLASH_EXT_USER_BASE + FLASH_EXT_USER_SIZE)))) ? 1U : 0U;
#endif /* !USE_HAL_FLASH_OB_EDATA */
  return (((is_user_addr != 0U) && ((check_program_alignment == 0U) ? (1U == 1U) : (((flash_addr) % 16U) == 0U))) \
          ? 1U : 0U);
}

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1) \
    && (defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)))
/**
  * @brief  Check if a FLASH address is part of the EDATA FLASH area.
  * @param  flash_addr The FLASH address to check.
  * @param  check_program_alignment The flag indicating if the address must be aligned for programming.
  * @retval The status of the FLASH address test (1: address is in EDATA FLASH, 0: otherwise)
  */
static uint32_t FLASH_IsFlashEDATAAddr(uint32_t flash_addr, uint8_t check_program_alignment)
{
  return (((flash_addr >= FLASH_EDATA_BASE) && (flash_addr < (FLASH_EDATA_BASE + FLASH_EDATA_SIZE)) \
           && (check_program_alignment == 0U)) || ((flash_addr & 1U) == 0U)) ? 1U : 0U;
}
#endif /* USE_HAL_FLASH_OB_EDATA && (USE_ASSERT_DBG_PARAM || (USE_HAL_CHECK_PARAM)) */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR */

#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
/**
  * @brief  Handle the FLASH error interrupt.
  * @param  hflash Pointer to a @ref hal_flash_handle_t structure.
  * @param  bank   Bank where the error occurred.
  *                This parameter can be one element of the @ref hal_flash_bank_t enumeration.
  */
static void FLASH_HandleErrorIT(hal_flash_handle_t *hflash, hal_flash_bank_t bank)
{
  LL_FLASH_DisableAllOperation(FLASH_GET_INSTANCE(hflash));

  LL_FLASH_DisableIT(FLASH_GET_INSTANCE(hflash), LL_FLASH_IT_ALL);

  LL_FLASH_ClearFlag(FLASH_GET_INSTANCE(hflash), LL_FLASH_FLAG_ERRORS_ALL);

  hflash->global_state = HAL_FLASH_STATE_IDLE;

  hflash->ongoing_operation = HAL_FLASH_NO_OPERATION;

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
  hflash->p_error_cb(hflash, bank);
#else
  HAL_FLASH_ErrorCallback(hflash, bank);
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
}
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */

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
