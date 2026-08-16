/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_ramcfg.c
  * @brief   This file provides firmware functions to manage the functionality
  *          of the RAM configuration controller peripheral.
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

#if defined (RAMCFG_SRAM1) || defined (RAMCFG_SRAM2)
#if defined (USE_HAL_RAMCFG_MODULE) && (USE_HAL_RAMCFG_MODULE == 1)

/** @addtogroup RAMCFG
  * @brief RAMCFG HAL module driver.
  * @{
  */

/** @defgroup RAMCFG_Introduction RAMCFG Introduction
  * @{

  The RAMCFG hardware abstraction layer provides a set of APIs to interface with the RAMCFG peripheral controlling the
  internal SRAMs configuration on STM32 microcontrollers.

  The RAMCFG module provides firmware functions to manage the following functionalities :
  - ECC management functions.
  - SRAM write protection management functions.
  - SRAM erase management functions.
  - SRAM information getter functions.

  This abstraction layer guarantees portability and ease of use across different STM32 series.
  */

/**
  * @}
  */

/** @defgroup RAMCFG_How_To_Use RAMCFG How To Use
  * @{

# How to use the RAMCFG HAL module driver

This module provides a set of APIs to manage each feature supported by the internal SRAMs:

1. ECC feature

  - This feature is supported by SRAM2. ECC monitoring supports the single error detection and correction and
    double error detection.

  - Use the HAL_RAMCFG_ECC_Enable() function to enable the RAMCFG ECC monitoring process in silent mode, i.e. no
    interrupt will be triggered, nor will a callback be sent upon an ECC error. User must rely on
    HAL_RAMCFG_ECC_GetInfo() function to monitor or/and check for ECC errors if any.

  - Use the HAL_RAMCFG_ECC_Enable_IT() function to start ECC error monitoring process in interrupt mode and to activate
    the latching error address. The following interrupts will be enabled and user will receive a dedicated callback
    upon an ECC error :
    - Single error interrupt,
    - Double error interrupt,
    - Double error interrupt redirected to Non-Maskable Interrupt(NMI).

    If an interrupt occurs on the RAMCFG line, user can call the HAL_RAMCFG_IRQHandler() function to manage the
    interrupt for a given instance.

    If the double error interrupt is redirected to NMI, user can call the HAL_RAMCFG_NMI_IRQHandler() function to manage
    the RAMCFG NMI interrupt for a given instance.

  - Use the HAL_RAMCFG_ECC_GetInfo() function to get the type, status and address of the last ECC error detected.
    The type can be :
    - No ECC error detected,
    - Single error,
    - Double error.
    The status can be :
    - ECC error not corrected,
    - ECC error corrected.

  - Use the HAL_RAMCFG_ECC_Disable() function to disable the RAMCFG ECC monitoring.

2. Write protection feature
  - This feature is supported by SRAM2.
    The SRAM1 is divided to 64 pages and SRAM2 to 64 pages with 1 KB granularity.
    Each page can be write protected independently.

  - Use the HAL_RAMCFG_EnablePageWRP() function to set the write protection for the given SRAM page(s).

  - Use the HAL_RAMCFG_EnableWRPByAddr() function to set the write protection for the given SRAM with start address
    and number of bytes.

  - Use the HAL_RAMCFG_IsEnabledPageWRP() function to check the write protection status of a page of the SRAM.

  - Use the HAL_RAMCFG_IsEnabledWRPByAddr() function to check the write protection status of a SRAM address.

  - There is no API to disable write protection as this feature can only be disabled by a global peripheral reset
    or system reset.

  - Any write access to a write protected area of the given SRAM causes a HardFault interrupt.

3. Erase feature
  - Each SRAM can be erased independently through its RAMCFG instance.

  - After a complete hardware erase, the given SRAM is set to 0 value.

  - Use the HAL_RAMCFG_MassErase() function to launch a hardware erase for the given SRAM.

4. SRAM information getter
  - Use the HAL_RAMCFG_GetLLInstance() function to get the selected RAMCFG hardware instance.

  - Use the HAL_RAMCFG_GetSRAMBaseAddress() function to get the selected RAMCFG SRAM base address.

  - Use the HAL_RAMCFG_GetSRAMSize() function to get the selected RAMCFG SRAM size.
  */
/**
  * @}
  */

/** @defgroup RAMCFG_Configuration_Table RAMCFG Configuration Table
  * @{
# Configuration inside the RAMCFG driver

Config defines                    | Description      | Default value     | Note
--------------------------------- | ---------------- | ----------------- | --------------------------------------
PRODUCT                           | from IDE         | NA                | Ex:STM32C562XX.
USE_ASSERT_DBG_PARAM              | from IDE         | None              | Enable the parameters asserts.
USE_HAL_CHECK_PARAM               | from hal_conf.h  | 0                 | Enable the parameters runtime checks.
USE_HAL_RAMCFG_MODULE             | from hal_conf.h  | 1                 | Enable the HAL RAMCFG module.
  */
/**
  * @}
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/

/** @defgroup RAMCFG_Private_Constants RAMCFG Private Constants
  * @{
  */

/*! RAMCFG page size value */
#define RAMCFG_PAGE_SIZE     0x400U
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup RAMCFG_Private_Macros RAMCFG Private Macros
  * @{
  */

/*! Get RAMCFG instance */
#define RAMCFG_GET_INSTANCE(instance) ((RAMCFG_TypeDef *)((uint32_t)(instance)))

/*! Macro to get the base address of the given SRAM */
#define RAMCFG_GET_SRAM_BASE_ADDR(instance) (((instance) == HAL_RAMCFG_SRAM1) ? SRAM1_BASE : SRAM2_BASE)

/*! Macro to get size of the given SRAM */
#define RAMCFG_GET_SRAM_SIZE_BYTE(instance) (((instance) == HAL_RAMCFG_SRAM1) ? SRAM1_SIZE : SRAM2_SIZE)

/*! Macro to check all interrupts */
#define IS_RAMCFG_ECC_INTERRUPT(interrupt) \
  ((((interrupt) & (HAL_RAMCFG_IT_ECC_SINGLE | HAL_RAMCFG_IT_ECC_DOUBLE | HAL_RAMCFG_IT_ECC_DOUBLE_NMI)) != 0U) \
   && (((interrupt) & ~(HAL_RAMCFG_IT_ECC_SINGLE | HAL_RAMCFG_IT_ECC_DOUBLE | HAL_RAMCFG_IT_ECC_DOUBLE_NMI)) == 0U))

/*! Macro to check parameters in range of the given SRAM */
#define IS_RAMCFG_WP_IN_RANGE(offset, size, sram_size) (((offset) + (size)) <= sram_size)

/*! Macro to check write protection granularity */
#define IS_RAMCFG_WP_GRANULARITY(addr, size, sram_size) (((((addr) - sram_size) % RAMCFG_PAGE_SIZE) == 0U) \
                                                         && (((size) % RAMCFG_PAGE_SIZE) == 0U))

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @addtogroup RAMCFG_Exported_Functions
  * @{
  */

/** @addtogroup RAMCFG_Exported_Functions_Group1
  * @{
This section provides functions to manage the ECC feature provided by the RAMCFG peripheral.

- Call the function HAL_RAMCFG_ECC_Enable() to enable the ECC error monitoring process and latch the error address
 in silent mode.

- Call the function HAL_RAMCFG_ECC_Enable_IT() to enable the ECC error monitoring process and latch the error address
 in interrupt mode (only interrupts specified in the dedicated function parameter are enabled).

- Call the function HAL_RAMCFG_ECC_Disable() to disable and stop ECC monitoring for the selected RAMCFG instance.

- Call the function HAL_RAMCFG_ECC_GetInfo() to get the RAMCFG ECC information.
  */

/**
  * @brief  Enable ECC monitoring with error address latching for the given RAMCFG instance.
  * @param  instance RAMCFG instance.
  * @retval HAL_OK ECC monitoring enabled successfully.
  */
hal_status_t HAL_RAMCFG_ECC_Enable(hal_ramcfg_t instance)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  LL_RAMCFG_EnableECC(RAMCFG_GET_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Enable ECC monitoring with error address latching and given interrupt(s) for the given RAMCFG instance.
  * @param  instance RAMCFG instance.
  * @param  interrupt Interrupt(s) to be enabled.
  *         This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_RAMCFG_IT_ECC_SINGLE
  *         @arg @ref HAL_RAMCFG_IT_ECC_DOUBLE
  *         @arg @ref HAL_RAMCFG_IT_ECC_DOUBLE_NMI
  * @retval HAL_OK ECC monitoring enabled successfully.
  */
hal_status_t HAL_RAMCFG_ECC_Enable_IT(hal_ramcfg_t instance, uint32_t interrupt)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INTERRUPT(interrupt));

  LL_RAMCFG_ClearFlag(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_FLAG_ECC_ALL);
  LL_RAMCFG_EnableIT(RAMCFG_GET_INSTANCE(instance), interrupt);
  LL_RAMCFG_EnableECC(RAMCFG_GET_INSTANCE(instance));

  return HAL_OK;
}

/**
  * @brief  Disable ECC monitoring for the given RAMCFG instance.
  * @param  instance RAMCFG instance.
  * @retval HAL_OK ECC monitoring disabled successfully.
  */
hal_status_t HAL_RAMCFG_ECC_Disable(hal_ramcfg_t instance)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  LL_RAMCFG_SetECCKey(RAMCFG_GET_INSTANCE(instance), (uint32_t)LL_RAMCFG_ECC_KEY_1);
  LL_RAMCFG_SetECCKey(RAMCFG_GET_INSTANCE(instance), (uint32_t)LL_RAMCFG_ECC_KEY_2);
  LL_RAMCFG_DisableECC(RAMCFG_GET_INSTANCE(instance));
  LL_RAMCFG_DisableIT(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_IT_ALL);

  return HAL_OK;
}

/**
  * @brief   Get the ECC information.
  * @param   instance RAMCFG instance.
  * @param   p_info Pointer to a \ref hal_ramcfg_ecc_info_t structure.
  */
void HAL_RAMCFG_ECC_GetInfo(hal_ramcfg_t instance, hal_ramcfg_ecc_info_t *p_info)
{
  ASSERT_DBG_PARAM(p_info != NULL);
  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  if (LL_RAMCFG_IsActiveFlag_DED(RAMCFG_GET_INSTANCE(instance)) != 0U)
  {
    p_info->type    = HAL_RAMCFG_ECC_DOUBLE;
    p_info->status  = HAL_RAMCFG_ECC_NOT_CORRECTED;
    p_info->address = LL_RAMCFG_GetECCDoubleErrorAddress(RAMCFG_GET_INSTANCE(instance));
  }
  else if (LL_RAMCFG_IsActiveFlag_SEDC(RAMCFG_GET_INSTANCE(instance)) != 0U)
  {
    p_info->type    = HAL_RAMCFG_ECC_SINGLE;
    p_info->status  = HAL_RAMCFG_ECC_CORRECTED;
    p_info->address = LL_RAMCFG_GetECCSingleErrorAddress(RAMCFG_GET_INSTANCE(instance));
  }
  else
  {
    p_info->type = HAL_RAMCFG_ECC_NONE;
  }
}
/**
  * @}
  */

/** @addtogroup RAMCFG_Exported_Functions_Group3
  * @{
This section provides functions to enable the write protection feature for the page(s) of the given SRAM.
SRAM page protection can be disabled only by a global peripheral reset or a system reset.

- Call the function HAL_RAMCFG_EnablePageWRP() to enable the write protection for the given page(s) of the SRAM.
- Call the function HAL_RAMCFG_EnableWRPByAddr() to enable the write protection for the given SRAM address range.
- Call the function HAL_RAMCFG_IsEnabledPageWRP() to check the write protection status of a page of the SRAM.
- Call the function HAL_RAMCFG_IsEnabledWRPByAddr() to check the write protection status of a SRAM address.

  */

/**
  * @brief  Enable write protection for the given page(s).
  * @param  instance   RAMCFG instance.
  * @param  start_page Select the start page number.
  * @param  page_nbr   Number of pages to be protected.
  * @retval HAL_INVALID_PARAM Invalid parameter when the number of pages to protect is higher than the number of pages
  *                           available for the given SRAM.
  * @retval HAL_OK            RAMCFG pages are successfully protected.
  */
hal_status_t HAL_RAMCFG_EnablePageWRP(hal_ramcfg_t instance, uint32_t start_page, uint32_t page_nbr)
{
  uint32_t page_mask_0 = 0U;
#if defined (LL_RAMCFG_WRP_PAGE_32)
  uint32_t page_mask_1 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_32 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  uint32_t page_mask_2 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_64 */
#if defined (LL_RAMCFG_WRP_PAGE_96)
  uint32_t page_mask_3 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_96 */

  ASSERT_DBG_PARAM(IS_RAMCFG_WP_INSTANCE(RAMCFG_GET_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_IN_RANGE((start_page * RAMCFG_PAGE_SIZE), (page_nbr * RAMCFG_PAGE_SIZE),
                                         RAMCFG_GET_SRAM_SIZE_BYTE(instance)));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((start_page + page_nbr) > (RAMCFG_GET_SRAM_SIZE_BYTE(instance) / RAMCFG_PAGE_SIZE))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Repeat for the page number to be protected. */
  for (uint32_t count = 0U; count < page_nbr; count++)
  {
#if defined (LL_RAMCFG_WRP_PAGE_32)
    if ((start_page + count) < 32U)
    {
#endif /* LL_RAMCFG_WRP_PAGE_32 */
      page_mask_0 |= (1UL << (start_page + count));
#if defined (LL_RAMCFG_WRP_PAGE_32)
    }
#if defined (LL_RAMCFG_WRP_PAGE_96)
    else if ((start_page + count) >= 96U)
    {
      page_mask_3 |= (1UL << ((start_page + count) - 96U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_96 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
    else if ((start_page + count) >= 64U)
    {
      page_mask_2 |= (1UL << ((start_page + count) - 64U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_64 */
    else
    {
      page_mask_1 |= (1UL << ((start_page + count) - 32U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_32 */
  }

  LL_RAMCFG_EnablePageWRP_0_31(RAMCFG_GET_INSTANCE(instance), page_mask_0);
#if defined (LL_RAMCFG_WRP_PAGE_32)
  LL_RAMCFG_EnablePageWRP_32_63(RAMCFG_GET_INSTANCE(instance), page_mask_1);
#endif /* LL_RAMCFG_WRP_PAGE_32 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  LL_RAMCFG_EnablePageWRP_64_95(RAMCFG_GET_INSTANCE(instance), page_mask_2);
#endif /* LL_RAMCFG_WRP_PAGE_64 */
#if defined (LL_RAMCFG_WRP_PAGE_96)
  LL_RAMCFG_EnablePageWRP_96_127(RAMCFG_GET_INSTANCE(instance), page_mask_3);
#endif /* LL_RAMCFG_WRP_PAGE_96 */

  return HAL_OK;
}

/**
  * @brief   Enable write protection for the given SRAM address range.
  * @param   instance  RAMCFG instance.
  * @param   sram_addr Start of the address range to be protected.
  * @param   size_byte Size of address range to be protected (in bytes).
  * @warning Physically, the SRAM protection granularity is a page. When sram_addr does not correspond to the start of
  *          a page and size_byte does not cover an integer number of pages, the driver rounds to the first and last
  *          pages to cover the area defined by sram_addr and size_byte.
  * @retval  HAL_INVALID_PARAM Invalid parameter when sram_addr is not in the range of the given SRAM and the total
  *                            size to be protected is larger than the given SRAM size.
  * @retval  HAL_OK            RAMCFG pages are successfully protected by address.
  */
hal_status_t HAL_RAMCFG_EnableWRPByAddr(hal_ramcfg_t instance,
                                        uint32_t sram_addr,
                                        uint32_t size_byte)
{
  uint32_t page_mask_0 = 0U;
#if defined (LL_RAMCFG_WRP_PAGE_32)
  uint32_t page_mask_1 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_32 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  uint32_t page_mask_2 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_64 */
#if defined (LL_RAMCFG_WRP_PAGE_96)
  uint32_t page_mask_3 = 0U;
#endif /* LL_RAMCFG_WRP_PAGE_96 */
  uint32_t start_page;
  uint32_t page_nbr;

  ASSERT_DBG_PARAM(IS_RAMCFG_WP_INSTANCE(RAMCFG_GET_INSTANCE(instance)));
  ASSERT_DBG_PARAM(RAMCFG_GET_SRAM_BASE_ADDR(instance) <= sram_addr);
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_GRANULARITY(sram_addr, size_byte, RAMCFG_GET_SRAM_SIZE_BYTE(instance)));
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_IN_RANGE((sram_addr - RAMCFG_GET_SRAM_BASE_ADDR(instance)), size_byte,
                                         RAMCFG_GET_SRAM_SIZE_BYTE(instance)));


#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((sram_addr < RAMCFG_GET_SRAM_BASE_ADDR(instance)) \
      || (((sram_addr - RAMCFG_GET_SRAM_BASE_ADDR(instance)) + size_byte) > \
          RAMCFG_GET_SRAM_SIZE_BYTE(instance)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  start_page = ((sram_addr - RAMCFG_GET_SRAM_BASE_ADDR(instance)) / RAMCFG_PAGE_SIZE);
  page_nbr = (size_byte / RAMCFG_PAGE_SIZE);

  /* Repeat for the page number to be protected. */
  for (uint32_t count = 0U; count < page_nbr; count++)
  {
#if defined (LL_RAMCFG_WRP_PAGE_32)
    if ((start_page + count) < 32U)
    {
#endif /* LL_RAMCFG_WRP_PAGE_32 */
      page_mask_0 |= (1UL << (start_page + count));
#if defined (LL_RAMCFG_WRP_PAGE_32)
    }
#if defined (LL_RAMCFG_WRP_PAGE_96)
    else if ((start_page + count) >= 96U)
    {
      page_mask_3 |= (1UL << ((start_page + count) - 96U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_96 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
    else if ((start_page + count) >= 64U)
    {
      page_mask_2 |= (1UL << ((start_page + count) - 64U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_64 */
    else
    {
      page_mask_1 |= (1UL << ((start_page + count) - 32U));
    }
#endif /* LL_RAMCFG_WRP_PAGE_32 */
  }

  LL_RAMCFG_EnablePageWRP_0_31(RAMCFG_GET_INSTANCE(instance), page_mask_0);
#if defined (LL_RAMCFG_WRP_PAGE_32)
  LL_RAMCFG_EnablePageWRP_32_63(RAMCFG_GET_INSTANCE(instance), page_mask_1);
#endif /* LL_RAMCFG_WRP_PAGE_32 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  LL_RAMCFG_EnablePageWRP_64_95(RAMCFG_GET_INSTANCE(instance), page_mask_2);
#endif /* LL_RAMCFG_WRP_PAGE_64 */
#if defined (LL_RAMCFG_WRP_PAGE_96)
  LL_RAMCFG_EnablePageWRP_96_127(RAMCFG_GET_INSTANCE(instance), page_mask_3);
#endif /* LL_RAMCFG_WRP_PAGE_96 */
  return HAL_OK;
}

/**
  * @brief  Check the write protection status for the given page.
  * @param  instance RAMCFG instance.
  * @param  page     Select the page index.
  * @retval Return value can be one element of \ref hal_ramcfg_wrp_page_status_t enumeration.
  */
hal_ramcfg_wrp_page_status_t HAL_RAMCFG_IsEnabledPageWRP(hal_ramcfg_t instance, uint32_t page)
{
  uint32_t wrp_status = 0U;

  ASSERT_DBG_PARAM(IS_RAMCFG_WP_INSTANCE(RAMCFG_GET_INSTANCE(instance)));
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_IN_RANGE((page * RAMCFG_PAGE_SIZE), RAMCFG_PAGE_SIZE,
                                         RAMCFG_GET_SRAM_SIZE_BYTE(instance)));

  /* Get the protection status of the given page */
#if defined (LL_RAMCFG_WRP_PAGE_32)
  if (page < 32U)
  {
#endif /* LL_RAMCFG_WRP_PAGE_32 */
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_0_31(RAMCFG_GET_INSTANCE(instance), (1UL << page));
#if defined (LL_RAMCFG_WRP_PAGE_32)
  }
#if defined (LL_RAMCFG_WRP_PAGE_96)
  else if (page >= 96U)
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_96_127(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 96U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_96 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  else if (page >= 64U)
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_64_95(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 64U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_64 */
  else
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_32_63(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 32U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_32 */

  return ((wrp_status == 0U) ? HAL_RAMCFG_WRP_PAGE_NOT_PROTECTED : HAL_RAMCFG_WRP_PAGE_PROTECTED);
}

/**
  * @brief   Check the write protection status for the given SRAM by address.
  * @param   instance  RAMCFG instance.
  * @param   sram_addr Address to check for protection.
  * @warning Physically, the SRAM protection granularity is a page. When sram_addr does not correspond to the start of
  *          a page, the driver checks the status for the page containing the given sram_addr.
  * @retval  Return value can be one element of \ref hal_ramcfg_wrp_page_status_t enumeration.
  */
hal_ramcfg_wrp_page_status_t HAL_RAMCFG_IsEnabledWRPByAddr(hal_ramcfg_t instance, uint32_t sram_addr)
{
  uint32_t page;
  uint32_t wrp_status;

  ASSERT_DBG_PARAM(IS_RAMCFG_WP_INSTANCE(RAMCFG_GET_INSTANCE(instance)));
  ASSERT_DBG_PARAM(RAMCFG_GET_SRAM_BASE_ADDR(instance) <= sram_addr);
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_GRANULARITY(sram_addr, 0U, RAMCFG_GET_SRAM_SIZE_BYTE(instance)));
  ASSERT_DBG_PARAM(IS_RAMCFG_WP_IN_RANGE((sram_addr - RAMCFG_GET_SRAM_BASE_ADDR(instance)), 0U,
                                         RAMCFG_GET_SRAM_SIZE_BYTE(instance)));

  page = ((sram_addr - RAMCFG_GET_SRAM_BASE_ADDR(instance)) / RAMCFG_PAGE_SIZE);

  /* Get the protection status of the given page */
#if defined (LL_RAMCFG_WRP_PAGE_32)
  if (page < 32U)
  {
#endif /* LL_RAMCFG_WRP_PAGE_32 */
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_0_31(RAMCFG_GET_INSTANCE(instance), (1UL << page));
#if defined (LL_RAMCFG_WRP_PAGE_32)
  }
#if defined (LL_RAMCFG_WRP_PAGE_96)
  else if (page >= 96U)
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_96_127(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 96U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_96 */
#if defined (LL_RAMCFG_WRP_PAGE_64)
  else if (page >= 64U)
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_64_95(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 64U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_64 */
  else
  {
    wrp_status = LL_RAMCFG_IsEnabledPageWRP_32_63(RAMCFG_GET_INSTANCE(instance), (1UL << (page - 32U)));
  }
#endif /* LL_RAMCFG_WRP_PAGE_32 */

  return ((wrp_status == 0U) ? HAL_RAMCFG_WRP_PAGE_NOT_PROTECTED : HAL_RAMCFG_WRP_PAGE_PROTECTED);
}
/**
  * @}
  */

/** @addtogroup RAMCFG_Exported_Functions_Group4
  * @{
This section provides a function that performs a hardware erase of a given SRAM.

- Call the function HAL_RAMCFG_MassErase() to perform a hardware mass erase of the given SRAM. Once erased,
  the SRAM content is zero.
  */

/**
  * @brief  Launch a mass erase for the given SRAM.
  * @param  instance RAMCFG instance.
  * @param  timeout  User timeout to finish the mass erase.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_OK      SRAM is successfully erased.
  */

hal_status_t HAL_RAMCFG_MassErase(hal_ramcfg_t instance, uint32_t timeout)
{
  uint32_t tickstart = HAL_GetTick();

  ASSERT_DBG_PARAM(IS_RAMCFG_MASS_ERASE_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  LL_RAMCFG_SetEraseKey(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_ERASE_KEY_1);
  LL_RAMCFG_SetEraseKey(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_ERASE_KEY_2);
  LL_RAMCFG_EnableSRAMErase(RAMCFG_GET_INSTANCE(instance));
  /* Wait for the SRAM hardware erase operation to complete by polling the SRAMBUSY flag until it resets. */
  while (LL_RAMCFG_IsActiveFlag_SRAMBUSY(RAMCFG_GET_INSTANCE(instance)) != 0U)
  {
    if ((HAL_GetTick() - tickstart) > timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup RAMCFG_Exported_Functions_Group5
  * @{

This section provides functions to handle RAMCFG interrupts and register callbacks.
- Call the function HAL_RAMCFG_IRQHandler() within the RAMCFG vector to handle any RAMCFG interrupt. Execute this API
 in handler mode.
- Call the function HAL_RAMCFG_NMI_IRQHandler() within the NMI vector to handle the NMI interrupt. Execute this API
 in handler mode.
- Call the function HAL_RAMCFG_ECC_ErrorCallback() to register the RAMCFG single or double error detect callback.
  */

/**
  * @brief  Handle RAMCFG interrupt request.
  * @param  instance RAMCFG instance.
  */
void HAL_RAMCFG_IRQHandler(hal_ramcfg_t instance)
{
  uint32_t flags;

  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  flags = LL_RAMCFG_ReadFlag(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_FLAG_ECC_ALL);

  /* Double Error Interrupt Management */
  if ((LL_RAMCFG_IsEnabledIT_DE(RAMCFG_GET_INSTANCE(instance)) != 0U) && ((flags & LL_RAMCFG_IT_DE) != 0U))
  {
    (void) HAL_RAMCFG_ECC_ErrorCallback(instance);
    LL_RAMCFG_ClearFlag_DED(RAMCFG_GET_INSTANCE(instance));
  }

  /* Single Error Interrupt Management */
  if ((LL_RAMCFG_IsEnabledIT_SE(RAMCFG_GET_INSTANCE(instance)) != 0U) && ((flags & LL_RAMCFG_IT_SE) != 0U))
  {
    (void) HAL_RAMCFG_ECC_ErrorCallback(instance);
    LL_RAMCFG_ClearFlag_SEDC(RAMCFG_GET_INSTANCE(instance));
  }
}

/**
  * @brief  Handle RAMCFG NMI interrupt request.
  * @param  instance  RAMCFG instance.
  * @retval HAL_OK    when the NMI has been specifically treated by this IRQHandler.
  * @retval HAL_ERROR otherwise.
  */
hal_status_t HAL_RAMCFG_NMI_IRQHandler(hal_ramcfg_t instance)
{
  hal_status_t cb_status = HAL_ERROR;
  uint32_t flags;

  ASSERT_DBG_PARAM(IS_RAMCFG_ECC_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  flags = LL_RAMCFG_ReadFlag(RAMCFG_GET_INSTANCE(instance), LL_RAMCFG_FLAG_DED);

  /* Double error redirected to NMI interrupt Management */
  if ((LL_RAMCFG_IsEnabledIT_NMI(RAMCFG_GET_INSTANCE(instance)) != 0U) && (flags != 0U))
  {
    /* ECC flag is only cleared if the callback returns HAL_OK,
       that is, if the NMI is specifically handled in the callback. */
    if (HAL_RAMCFG_ECC_ErrorCallback(instance) == HAL_OK)
    {
      LL_RAMCFG_ClearFlag_DED(RAMCFG_GET_INSTANCE(instance));
      cb_status = HAL_OK;
    }
  }

  return cb_status;
}

/**
  * @brief  RAMCFG single or double error detection callback.
  * @param  instance  RAMCFG instance.
  * @retval HAL_ERROR when the NMI has not been treated by the callback.
  */
__WEAK hal_status_t HAL_RAMCFG_ECC_ErrorCallback(hal_ramcfg_t instance)
{
  /* Prevent unused argument compilation warnings. */
  STM32_UNUSED(instance);

  /* Note: Do not modify this function. When the callback is needed,
           implement HAL_RAMCFG_ECC_ErrorCallback in the user file.  */

  return HAL_ERROR;
}

/**
  * @}
  */

/** @addtogroup RAMCFG_Exported_Functions_Group6
  * @{
This section provides functions to get RAMCFG instance information.
- Call the function HAL_RAMCFG_GetLLInstance() to get the selected RAMCFG hardware instance.
- Call the function HAL_RAMCFG_GetSRAMBaseAddress() to get the selected RAMCFG SRAM base address.
- Call the function HAL_RAMCFG_GetSRAMSize() to get the selected RAMCFG SRAM size.
  */
/**
  * @brief  Get the selected hardware RAMCFG instance.
  * @param  instance RAMCFG instance.
  * @retval The selected hardware RAMCFG instance.
  */
RAMCFG_TypeDef *HAL_RAMCFG_GetLLInstance(hal_ramcfg_t instance)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ALL_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  return ((RAMCFG_TypeDef *)((uint32_t)(instance)));
}

/**
  * @brief  Get the RAMCFG SRAM base address.
  * @param  instance RAMCFG instance.
  * @retval uint32_t The base address of the given SRAM.
  */
uint32_t HAL_RAMCFG_GetSRAMBaseAddress(hal_ramcfg_t instance)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ALL_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  return RAMCFG_GET_SRAM_BASE_ADDR(instance);
}

/**
  * @brief  Get the RAMCFG SRAM size in bytes.
  * @param  instance RAMCFG instance.
  * @retval uint32_t The size of the given SRAM in bytes.
  */
uint32_t HAL_RAMCFG_GetSRAMSize(hal_ramcfg_t instance)
{
  ASSERT_DBG_PARAM(IS_RAMCFG_ALL_INSTANCE(RAMCFG_GET_INSTANCE(instance)));

  return RAMCFG_GET_SRAM_SIZE_BYTE(instance);
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

#endif /* USE_HAL_RAMCFG_MODULE */
#endif /* RAMCFG_SRAM1 || RAMCFG_SRAM2 */

/**
  * @}
  */
