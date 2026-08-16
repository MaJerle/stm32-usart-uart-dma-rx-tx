/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_ramcfg.h
  * @brief   Header file for the RAMCFG HAL module.
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

/* Define to prevent recursive inclusion. ----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_RAMCFG_H
#define STM32C5XX_HAL_RAMCFG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_ramcfg.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (RAMCFG_SRAM1) || defined (RAMCFG_SRAM2)

/** @addtogroup RAMCFG
  * @{
  */

/* Exported Constants ------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_Exported_Constants HAL RAMCFG Constants
  * @{
  */

/** @defgroup RAMCFG_Interrupts RAMCFG Interrupts
  * @{
  */
#define HAL_RAMCFG_IT_ECC_SINGLE       LL_RAMCFG_IT_SE    /*!< RAMCFG ECC single error interrupt        */
#define HAL_RAMCFG_IT_ECC_DOUBLE       LL_RAMCFG_IT_DE    /*!< RAMCFG ECC double error interrupt        */
#define HAL_RAMCFG_IT_ECC_DOUBLE_NMI   LL_RAMCFG_IT_NMI   /*!< NMI on RAMCFG ECC double error interrupt */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_Exported_Types HAL RAMCFG Types
  * @{
  */

/*! RAMCFG instance enumeration definition */
typedef enum
{
  HAL_RAMCFG_SRAM1 = RAMCFG_SRAM1_BASE,  /*!< SRAM1 instance  */
  HAL_RAMCFG_SRAM2 = RAMCFG_SRAM2_BASE,  /*!< SRAM2 instance  */
} hal_ramcfg_t;

/*! HAL RAMCFG write protection status enumeration definition */
typedef enum
{
  HAL_RAMCFG_WRP_PAGE_NOT_PROTECTED = 0U, /*!< RAMCFG not write protected page */
  HAL_RAMCFG_WRP_PAGE_PROTECTED     = 1U  /*!< RAMCFG write protected page     */
} hal_ramcfg_wrp_page_status_t;

/*! HAL RAMCFG ECC type enumeration definition */
typedef enum
{
  HAL_RAMCFG_ECC_NONE   = 0U,                  /*!< RAMCFG ECC none                   */
  HAL_RAMCFG_ECC_SINGLE = LL_RAMCFG_FLAG_SEDC, /*!< RAMCFG ECC single error detection */
  HAL_RAMCFG_ECC_DOUBLE = LL_RAMCFG_FLAG_DED   /*!< RAMCFG ECC double error detection */
} hal_ramcfg_ecc_type_t;

/*! HAL RAMCFG ECC type enumeration definition */
typedef enum
{
  HAL_RAMCFG_ECC_NOT_CORRECTED = 0U,  /*!< RAMCFG ECC not corrected */
  HAL_RAMCFG_ECC_CORRECTED     = 1U   /*!< RAMCFG ECC corrected     */
} hal_ramcfg_ecc_status_t;

/*! HAL RAMCFG ECC structure definition */
typedef struct
{
  hal_ramcfg_ecc_type_t   type;    /*!< RAMCFG ECC type    */
  hal_ramcfg_ecc_status_t status;  /*!< RAMCFG ECC status  */
  uint32_t                address; /*!< RAMCFG ECC address */
} hal_ramcfg_ecc_info_t;

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_Exported_Functions HAL RAMCFG Functions
  * @{
  */

/** @defgroup RAMCFG_Exported_Functions_Group1 ECC operation functions
  * @{
  */
/* Enable and Disable ECC process APIs */
hal_status_t HAL_RAMCFG_ECC_Enable(hal_ramcfg_t instance);
hal_status_t HAL_RAMCFG_ECC_Enable_IT(hal_ramcfg_t instance, uint32_t interrupt);
hal_status_t HAL_RAMCFG_ECC_Disable(hal_ramcfg_t instance);
void         HAL_RAMCFG_ECC_GetInfo(hal_ramcfg_t instance, hal_ramcfg_ecc_info_t *p_info);
/**
  * @}
  */

/** @defgroup RAMCFG_Exported_Functions_Group3 Write protection functions
  * @{
  */
/* Write protection APIs */
hal_status_t HAL_RAMCFG_EnablePageWRP(hal_ramcfg_t instance, uint32_t start_page, uint32_t page_nbr);
hal_status_t HAL_RAMCFG_EnableWRPByAddr(hal_ramcfg_t instance, uint32_t sram_addr, uint32_t size_byte);
hal_ramcfg_wrp_page_status_t HAL_RAMCFG_IsEnabledPageWRP(hal_ramcfg_t instance, uint32_t page);
hal_ramcfg_wrp_page_status_t HAL_RAMCFG_IsEnabledWRPByAddr(hal_ramcfg_t instance, uint32_t sram_addr);
/**
  * @}
  */

/** @defgroup RAMCFG_Exported_Functions_Group4 Erase operation functions
  * @{
  */
/* Erase APIs */
hal_status_t HAL_RAMCFG_MassErase(hal_ramcfg_t instance, uint32_t timeout);
/**
  * @}
  */

/** @defgroup RAMCFG_Exported_Functions_Group5 Handle interrupt and callbacks functions
  * @{
  */
/* IRQHandler APIs */
void HAL_RAMCFG_IRQHandler(hal_ramcfg_t instance);
hal_status_t HAL_RAMCFG_NMI_IRQHandler(hal_ramcfg_t instance);

/* Callback APIs */
hal_status_t HAL_RAMCFG_ECC_ErrorCallback(hal_ramcfg_t instance);
/**
  * @}
  */

/** @defgroup RAMCFG_Exported_Functions_Group6 SRAM info getter functions
  * @{
  */
RAMCFG_TypeDef *HAL_RAMCFG_GetLLInstance(hal_ramcfg_t instance);
uint32_t HAL_RAMCFG_GetSRAMBaseAddress(hal_ramcfg_t instance);
uint32_t HAL_RAMCFG_GetSRAMSize(hal_ramcfg_t instance);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* RAMCFG_SRAM1 || RAMCFG_SRAM2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
#endif /* STM32C5XX_HAL_RAMCFG_H */
