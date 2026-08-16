/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_flash.h
  * @brief   Header file of FLASH HAL module.
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
#ifndef STM32C5XX_HAL_FLASH_H
#define STM32C5XX_HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_flash_itf.h"


/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (FLASH)

/** @defgroup FLASH FLASH
  * @{
  */

/* Exported Constants ------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Constants HAL FLASH Constants
  * @{
  */

#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
/** @defgroup FLASH_Error_Code FLASH Error Code
  * @{
  */
#define HAL_FLASH_ERROR_NONE    0UL             /*!< No error during erase or operation          */
#define HAL_FLASH_ERROR_WRP     (0x01UL << 1U)  /*!< Write protection error during operation     */
#define HAL_FLASH_ERROR_PGS     (0x01UL << 3U)  /*!< Programming sequence error during operation */
#define HAL_FLASH_ERROR_STRB    (0x01UL << 5U)  /*!< Strobe error during operation               */
#define HAL_FLASH_ERROR_INC     (0x01UL << 7U)  /*!< Inconsistency error during operation        */


/**
  * @}
  */
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

/**
  * @}
  */

/* Exported Types ----------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Types HAL FLASH Types
  * @{
  */

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/*! HAL FLASH program mode enumeration definition */
typedef enum
{
  HAL_FLASH_PROGRAM_QUADWORD   = 0x5A5A5A5AU, /*!< FLASH Program Quad-Word mode   */
  HAL_FLASH_PROGRAM_DOUBLEWORD = 0xAAAAAAAAU, /*!< FLASH Program Double-Word mode */
  HAL_FLASH_PROGRAM_WORD       = 0x55555555U, /*!< FLASH Program Word mode        */
  HAL_FLASH_PROGRAM_HALFWORD   = 0xF5F5F5F5U, /*!< FLASH Program Half-Word mode   */
  HAL_FLASH_PROGRAM_BYTE       = 0xEEEEEEEEU, /*!< FLASH Program Byte mode        */
} hal_flash_program_mode_t;
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

/*! HAL FLASH state enumeration definition */
typedef enum
{
  HAL_FLASH_STATE_RESET       = 0UL,          /*!< FLASH not initialized                       */
  HAL_FLASH_STATE_IDLE        = (1UL << 30U), /*!< FLASH initialized and configured            */
  HAL_FLASH_STATE_ACTIVE      = (1UL << 31U)  /*!< FLASH process operation state               */
} hal_flash_state_t;

/*! HAL FLASH area info structure definition */
typedef struct
{
  uint32_t base_addr;      /*!< FLASH area base address  */
  uint32_t area_size_byte; /*!< FLASH area size in bytes */
  uint16_t page_nbr;       /*!< FLASH number of pages    */
} hal_flash_area_info_t;

/*! HAL FLASH bank info structure definition */
typedef struct
{
  uint8_t area_nbr;                     /*!< FLASH number of areas                          */
  uint32_t bank_size_byte;              /*!< FLASH bank size in bytes                       */
  hal_flash_area_info_t user_flash;     /*!< FLASH user area information structure          */
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
  hal_flash_area_info_t edata_flash;    /*!< FLASH edata area information structure         */
#else
  hal_flash_area_info_t ext_user_flash; /*!< FLASH extended user area information structure */
#endif /* USE_HAL_FLASH_OB_EDATA */
} hal_flash_bank_info_t;


/*! HAL FLASH info structure definition */
typedef struct
{
  uint32_t flash_size_byte;                  /*!< FLASH total size in bytes                  */
  uint8_t bank_nbr;                          /*!< FLASH number of banks                      */
  hal_flash_bank_info_t bank[FLASH_BANK_NB]; /*!< FLASH array of bank information structures */
} hal_flash_info_t;

/*! HAL FLASH interrupted operation enumeration definition */
typedef enum
{
  HAL_FLASH_INTERRUPTED_NO_OPERATION = LL_FLASH_INTERRUPTED_NO_OPERATION, /*!< FLASH interrupted no operation        */
  HAL_FLASH_INTERRUPTED_SINGLE_WRITE = LL_FLASH_INTERRUPTED_SINGLE_WRITE, /*!< FLASH interrupted single write        */
  HAL_FLASH_INTERRUPTED_PAGE_ERASE   = LL_FLASH_INTERRUPTED_PAGE_ERASE,   /*!< FLASH interrupted page erase          */
  HAL_FLASH_INTERRUPTED_BANK_ERASE   = LL_FLASH_INTERRUPTED_BANK_ERASE,   /*!< FLASH interrupted bank erase          */
  HAL_FLASH_INTERRUPTED_MASS_ERASE   = LL_FLASH_INTERRUPTED_MASS_ERASE,   /*!< FLASH interrupted mass erase          */
  HAL_FLASH_INTERRUPTED_OB_CHANGE    = LL_FLASH_INTERRUPTED_OB_CHANGE     /*!< FLASH interrupted option bytes change */
} hal_flash_interrupted_operation_t;

/*! HAL FLASH interrupted operation structure definition */
typedef struct
{
  uint32_t                          addr;        /*!< FLASH interrupted operation address */
  hal_flash_interrupted_operation_t operation;   /*!< FLASH interrupted operation code    */
} hal_flash_interrupted_by_reset_operation_info_t;

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/*! HAL FLASH ECC type enumeration definition */
typedef enum
{
  HAL_FLASH_ECC_NONE   = 0U,                     /*!< FLASH ECC no error               */
  HAL_FLASH_ECC_SINGLE = LL_FLASH_FLAG_ECCC,     /*!< FLASH ECC single error detection */
  HAL_FLASH_ECC_DOUBLE = LL_FLASH_FLAG_ECCD      /*!< FLASH ECC double error detection */
} hal_flash_ecc_type_t;

/*! HAL FLASH ECC status enumeration definition */
typedef enum
{
  HAL_FLASH_ECC_NOT_CORRECTED = 0U,     /*!< FLASH ECC not corrected */
  HAL_FLASH_ECC_CORRECTED     = 1U      /*!< FLASH ECC corrected     */
} hal_flash_ecc_status_t;

/*! HAL FLASH ECC information enumeration definition */
typedef struct
{
  hal_flash_ecc_type_t   type;     /*!< FLASH ECC type    */
  hal_flash_ecc_status_t status;   /*!< FLASH ECC status  */
  uint32_t               addr;     /*!< FLASH ECC address */
  uint32_t               data;     /*!< FLASH ECC data    */
} hal_flash_ecc_info_t;
#endif /* USE_HAL_FLASH_ECC */

/*! HAL FLASH operation enumeration definition */
typedef enum
{
  HAL_FLASH_NO_OPERATION = 0U, /*!< FLASH no operation            */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  HAL_FLASH_PROGRAM      = 1U, /*!< FLASH operation program       */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
  HAL_FLASH_ADDR_ERASE   = 2U, /*!< FLASH operation address erase */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
  HAL_FLASH_PAGE_ERASE   = 3U, /*!< FLASH operation page erase  */
#endif /* USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
  HAL_FLASH_BANK_ERASE   = 4U, /*!< FLASH operation bank erase    */
#endif /* USE_HAL_FLASH_ERASE_BANK */
#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
  HAL_FLASH_MASS_ERASE   = 5U  /*!< FLASH operation mass erase    */
#endif /* USE_HAL_FLASH_MASS_ERASE */
} hal_flash_operation_t;

typedef struct hal_flash_handle_s hal_flash_handle_t; /*!< HAL FLASH handle structure type */

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/*! HAL FLASH program unit function pointer definition */
typedef void (*hal_flash_program_unit_func_t)(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
/*! HAL FLASH program complete callback pointer definition */
typedef void (*hal_flash_program_cplt_cb_t)(hal_flash_handle_t *hflash, uint32_t addr, uint32_t size_byte);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
/*! HAL FLASH erase by address complete callback pointer definition */
typedef void (*hal_flash_erase_by_addr_cplt_cb_t)(hal_flash_handle_t *hflash, uint32_t addr, uint32_t size_byte);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
/*! HAL FLASH erase by page complete callback pointer definition */
typedef void (*hal_flash_erase_page_cplt_cb_t)(hal_flash_handle_t *hflash,
                                               hal_flash_bank_t bank,
                                               uint32_t page,
                                               uint32_t page_nbr);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
/*! HAL FLASH bank erase complete callback pointer definition */
typedef void (*hal_flash_erase_bank_cplt_cb_t)(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
/*! HAL FLASH mass erase complete callback pointer definition */
typedef void (*hal_flash_mass_erase_cplt_cb_t)(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_MASS_ERASE */

/*! HAL FLASH error callback pointer definition */
typedef void (*hal_flash_error_cb_t)(hal_flash_handle_t *hflash, hal_flash_bank_t bank);

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
/*! HAL FLASH ECC error callback pointer definition */
typedef hal_status_t (*hal_flash_ecc_error_cb_t)(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ECC */
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */

/*! HAL FLASH handle structure definition */
struct hal_flash_handle_s
{
  hal_flash_t                       instance;                /*!< FLASH instance                                  */
  volatile hal_flash_state_t        global_state;            /*!< FLASH global state                              */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  hal_flash_program_mode_t          programming_mode;        /*!< FLASH programming mode                          */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
  volatile hal_flash_operation_t    ongoing_operation;       /*!< FLASH ongoing operation type                    */
#if (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)) \
    || (defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1))
  hal_flash_bank_t                 erase_bank;               /*!< FLASH erase bank                                */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE
          || USE_HAL_FLASH_ERASE_BANK || USE_HAL_FLASH_MASS_ERASE */
#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
  volatile uint32_t                 last_error_codes;        /*!< FLASH last error codes                          */
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */
#if (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1))
  volatile uint32_t                 erase_page;              /*!< FLASH erase page                                */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  volatile uint32_t                 prog_data_addr;          /*!< FLASH program user data buffer                  */
  volatile uint32_t                 prog_flash_addr;         /*!< FLASH program flash address                     */
  volatile uint32_t                 prog_size_byte;          /*!< FLASH programmed size in bytes                  */
  uint32_t                          is_adaptive_prog;        /*!< FLASH is adaptive programming status            */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if (defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)) \
    || (defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1))
  uint32_t                          size;                    /*!< FLASH program or erase size byte or page number */
  volatile uint32_t                 count;                   /*!< FLASH program and erase count                   */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR || USE_HAL_FLASH_ERASE_BY_ADDR || USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  hal_flash_program_unit_func_t     p_prog_unit_func;        /*!< FLASH program unit function                     */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
  hal_flash_program_cplt_cb_t       p_program_cplt_cb;       /*!< FLASH program complete callback                 */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
  hal_flash_erase_by_addr_cplt_cb_t p_erase_by_addr_cplt_cb; /*!< FLASH erase by address complete callback        */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
  hal_flash_erase_page_cplt_cb_t    p_erase_page_cplt_cb;    /*!< FLASH erase page complete callback              */
#endif /* USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
  hal_flash_erase_bank_cplt_cb_t    p_erase_bank_cplt_cb;    /*!< FLASH erase bank complete callback              */
#endif /* USE_HAL_FLASH_ERASE_BANK */
#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
  hal_flash_mass_erase_cplt_cb_t    p_mass_erase_cplt_cb;    /*!< FLASH mass erase complete callback              */
#endif /* USE_HAL_FLASH_MASS_ERASE */
  hal_flash_error_cb_t              p_error_cb;              /*!< FLASH error callback                            */
#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
  hal_flash_ecc_error_cb_t          p_ecc_error_cb;          /*!< FLASH ECC error callback                        */
#endif /* USE_HAL_FLASH_ECC */
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */
#if defined(USE_HAL_FLASH_USER_DATA) && (USE_HAL_FLASH_USER_DATA == 1)
  const void                        *p_user_data;            /*!< FLASH user data                                 */
#endif /* USE_HAL_FLASH_USER_DATA */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Functions HAL FLASH Functions
  * @{
  */

/** @defgroup FLASH_Exported_Functions_Group1 HAL FLASH Initialization and De-initialization functions
  * @{
  */
hal_status_t HAL_FLASH_Init(hal_flash_handle_t *hflash, hal_flash_t instance);
void HAL_FLASH_DeInit(hal_flash_handle_t *hflash);
/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group2 HAL FLASH Configuration functions
  * @{
  */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
hal_status_t HAL_FLASH_SetProgrammingMode(hal_flash_handle_t *hflash, hal_flash_program_mode_t programming_mode);
hal_flash_program_mode_t HAL_FLASH_GetProgrammingMode(const hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group3 HAL FLASH Process Operations functions
  * @{
  */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
hal_status_t HAL_FLASH_ProgramByAddr(hal_flash_handle_t *hflash,
                                     uint32_t flash_addr,
                                     const uint32_t *p_data,
                                     uint32_t size_byte,
                                     uint32_t timeout_ms);

hal_status_t HAL_FLASH_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                        uint32_t flash_addr,
                                        const uint32_t *p_data,
                                        uint32_t size_byte);

hal_status_t HAL_FLASH_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                          uint32_t flash_addr,
                                          const uint32_t *p_data,
                                          uint32_t size_byte,
                                          uint32_t timeout_ms);

hal_status_t HAL_FLASH_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                             uint32_t flash_addr,
                                             const uint32_t *p_data,
                                             uint32_t size_byte);

hal_status_t HAL_FLASH_OTP_ProgramByAddr(hal_flash_handle_t *hflash,
                                         uint32_t otp_addr,
                                         const uint32_t *p_data,
                                         uint32_t size_byte,
                                         uint32_t timeout_ms);

hal_status_t HAL_FLASH_OTP_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                            uint32_t otp_addr,
                                            const uint32_t *p_data,
                                            uint32_t size_byte);

hal_status_t HAL_FLASH_OTP_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                              uint32_t otp_addr,
                                              const uint32_t *p_data,
                                              uint32_t size_byte,
                                              uint32_t timeout_ms);

hal_status_t HAL_FLASH_OTP_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                                 uint32_t otp_addr,
                                                 const uint32_t *p_data,
                                                 uint32_t size_byte);

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
hal_status_t HAL_FLASH_EDATA_ProgramByAddr(hal_flash_handle_t *hflash,
                                           uint32_t flash_addr,
                                           const uint32_t *p_data,
                                           uint32_t size_byte,
                                           uint32_t timeout_ms);

hal_status_t HAL_FLASH_EDATA_ProgramByAddr_IT(hal_flash_handle_t *hflash,
                                              uint32_t flash_addr,
                                              const uint32_t *p_data,
                                              uint32_t size_byte);

hal_status_t HAL_FLASH_EDATA_ProgramByAddrAdapt(hal_flash_handle_t *hflash,
                                                uint32_t flash_addr,
                                                const uint32_t *p_data,
                                                uint32_t size_byte,
                                                uint32_t timeout_ms);

hal_status_t HAL_FLASH_EDATA_ProgramByAddrAdapt_IT(hal_flash_handle_t *hflash,
                                                   uint32_t flash_addr,
                                                   const uint32_t *p_data,
                                                   uint32_t size_byte);
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */


#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
hal_status_t HAL_FLASH_EraseByAddr(hal_flash_handle_t *hflash,
                                   uint32_t flash_addr,
                                   uint32_t size_byte,
                                   uint32_t timeout_ms);


hal_status_t HAL_FLASH_EraseByAddr_IT(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte);

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
hal_status_t HAL_FLASH_EDATA_EraseByAddr(hal_flash_handle_t *hflash,
                                         uint32_t flash_addr,
                                         uint32_t size_byte,
                                         uint32_t timeout_ms);

hal_status_t HAL_FLASH_EDATA_EraseByAddr_IT(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte);
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
hal_status_t HAL_FLASH_ErasePage(hal_flash_handle_t *hflash,
                                 hal_flash_bank_t bank,
                                 uint32_t page,
                                 uint32_t page_nbr,
                                 uint32_t timeout_ms);

hal_status_t HAL_FLASH_ErasePage_IT(hal_flash_handle_t *hflash,
                                    hal_flash_bank_t bank,
                                    uint32_t page,
                                    uint32_t page_nbr);

#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
hal_status_t HAL_FLASH_EDATA_ErasePage(hal_flash_handle_t *hflash,
                                       hal_flash_bank_t bank,
                                       uint32_t page,
                                       uint32_t page_nbr,
                                       uint32_t timeout_ms);

hal_status_t HAL_FLASH_EDATA_ErasePage_IT(hal_flash_handle_t *hflash,
                                          hal_flash_bank_t bank,
                                          uint32_t page,
                                          uint32_t page_nbr);
#endif /* USE_HAL_FLASH_OB_EDATA */
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
hal_status_t HAL_FLASH_EraseBank(hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t timeout_ms);
hal_status_t HAL_FLASH_EraseBank_IT(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
hal_status_t HAL_FLASH_MassErase(hal_flash_handle_t *hflash, uint32_t timeout_ms);
hal_status_t HAL_FLASH_MassErase_IT(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_MASS_ERASE */

void HAL_FLASH_IRQHandler(hal_flash_handle_t *hflash);

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
void HAL_FLASH_ProgramByAddr_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
void HAL_FLASH_EraseByAddr_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
void HAL_FLASH_ErasePage_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
void HAL_FLASH_EraseBank_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
void HAL_FLASH_MassErase_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_MASS_ERASE */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
void HAL_FLASH_ECC_IRQHandler(hal_flash_handle_t *hflash);
hal_status_t HAL_FLASH_NMI_IRQHandler(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_ECC */


/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group4 HAL FLASH Callback functions
  * @{
  */
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
void HAL_FLASH_ProgramCpltCallback(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
void HAL_FLASH_EraseByAddrCpltCallback(hal_flash_handle_t *hflash, uint32_t flash_addr, uint32_t size_byte);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
void HAL_FLASH_ErasePageCpltCallback(hal_flash_handle_t *hflash,
                                     hal_flash_bank_t bank,
                                     uint32_t page,
                                     uint32_t page_nbr);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
void HAL_FLASH_EraseBankCpltCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ERASE_BANK */

#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
void HAL_FLASH_MassEraseCpltCallback(hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_MASS_ERASE */

void HAL_FLASH_ErrorCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank);

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
hal_status_t HAL_FLASH_ECC_ErrorCallback(hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ECC */

#if defined (USE_HAL_FLASH_REGISTER_CALLBACKS) && (USE_HAL_FLASH_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
hal_status_t HAL_FLASH_RegisterProgramCpltCallback(hal_flash_handle_t *hflash, hal_flash_program_cplt_cb_t callback);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
hal_status_t HAL_FLASH_RegisterEraseByAddrCpltCallback(hal_flash_handle_t *hflash,
                                                       hal_flash_erase_by_addr_cplt_cb_t callback);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */
#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
hal_status_t HAL_FLASH_RegisterErasePageCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_erase_page_cplt_cb_t callback);
#endif /* USE_HAL_FLASH_ERASE_PAGE */
#if defined (USE_HAL_FLASH_ERASE_BANK) && (USE_HAL_FLASH_ERASE_BANK == 1)
hal_status_t HAL_FLASH_RegisterEraseBankCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_erase_bank_cplt_cb_t callback);
#endif /* USE_HAL_FLASH_ERASE_BANK */
#if defined (USE_HAL_FLASH_MASS_ERASE) && (USE_HAL_FLASH_MASS_ERASE == 1)
hal_status_t HAL_FLASH_RegisterMassEraseCpltCallback(hal_flash_handle_t *hflash,
                                                     hal_flash_mass_erase_cplt_cb_t callback);
#endif /* USE_HAL_FLASH_MASS_ERASE */
hal_status_t HAL_FLASH_RegisterErrorCallback(hal_flash_handle_t *hflash, hal_flash_error_cb_t callback);

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
hal_status_t HAL_FLASH_RegisterECCErrorCallback(hal_flash_handle_t *hflash, hal_flash_ecc_error_cb_t callback);
#endif /* USE_HAL_FLASH_ECC */
#endif /* USE_HAL_FLASH_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group5 HAL FLASH Status functions
  * @{
  */
hal_flash_operation_t HAL_FLASH_GetCurrentOperation(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);

#if defined (USE_HAL_FLASH_PROGRAM_BY_ADDR) && (USE_HAL_FLASH_PROGRAM_BY_ADDR == 1)
uint32_t HAL_FLASH_GetCurrentProgrammedAddr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_PROGRAM_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_BY_ADDR) && (USE_HAL_FLASH_ERASE_BY_ADDR == 1)
uint32_t HAL_FLASH_GetCurrentErasedAddr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ERASE_BY_ADDR */

#if defined (USE_HAL_FLASH_ERASE_PAGE) && (USE_HAL_FLASH_ERASE_PAGE == 1)
uint32_t HAL_FLASH_GetCurrentErasedPage(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_ERASE_PAGE */

void HAL_FLASH_GetInterruptedByResetOperationInfo(const hal_flash_handle_t *hflash,
                                                  hal_flash_interrupted_by_reset_operation_info_t *p_info);

void HAL_FLASH_GetInfo(const hal_flash_handle_t *hflash, hal_flash_info_t *p_info);

uint32_t HAL_FLASH_GetSizeByte(const hal_flash_handle_t *hflash);
uint8_t HAL_FLASH_GetBankNbr(const hal_flash_handle_t *hflash);
uint32_t HAL_FLASH_GetBankSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);

uint32_t HAL_FLASH_GetUserFlashSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
uint32_t HAL_FLASH_EDATA_GetSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#else
uint32_t HAL_FLASH_GetExtUserFlashSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_OB_EDATA */

uint16_t HAL_FLASH_GetUserFlashPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
uint16_t HAL_FLASH_EDATA_GetPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#else
uint16_t HAL_FLASH_GetExtUserFlashPageNbr(const hal_flash_handle_t *hflash, hal_flash_bank_t bank);
#endif /* USE_HAL_FLASH_OB_EDATA */

uint32_t HAL_FLASH_GetUserFlashPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
uint32_t HAL_FLASH_EDATA_GetPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#else
uint32_t HAL_FLASH_GetExtUserFlashPageSizeByte(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#endif /* USE_HAL_FLASH_OB_EDATA */

uint32_t HAL_FLASH_GetUserFlashAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#if defined(USE_HAL_FLASH_OB_EDATA) && (USE_HAL_FLASH_OB_EDATA == 1)
uint32_t HAL_FLASH_EDATA_GetAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#else
uint32_t HAL_FLASH_GetExtUserFlashAddrOffset(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, uint32_t page);
#endif /* USE_HAL_FLASH_OB_EDATA */

#if defined (USE_HAL_FLASH_ECC) && (USE_HAL_FLASH_ECC == 1)
void HAL_FLASH_ECC_GetInfo(const hal_flash_handle_t *hflash, hal_flash_bank_t bank, hal_flash_ecc_info_t *p_info);
#endif /* USE_HAL_FLASH_ECC */

hal_flash_t HAL_FLASH_GetInstance(const hal_flash_handle_t *hflash);
FLASH_TypeDef *HAL_FLASH_GetLLInstance(const hal_flash_handle_t *hflash);

hal_flash_state_t HAL_FLASH_GetState(const hal_flash_handle_t *hflash);

#if defined(USE_HAL_FLASH_GET_LAST_ERRORS) && (USE_HAL_FLASH_GET_LAST_ERRORS == 1)
uint32_t HAL_FLASH_GetLastErrorCodes(const hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_GET_LAST_ERRORS */

#if defined (USE_HAL_FLASH_USER_DATA) && (USE_HAL_FLASH_USER_DATA == 1)
void HAL_FLASH_SetUserData(hal_flash_handle_t *hflash, const void *p_user_data);

const void *HAL_FLASH_GetUserData(const hal_flash_handle_t *hflash);
#endif /* USE_HAL_FLASH_USER_DATA */

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

#endif /* STM32C5XX_HAL_FLASH_H */
