/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_sbs.c
  * @brief   SBS HAL module driver.
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

#if defined (SBS)
#if defined(USE_HAL_SBS_MODULE) && (USE_HAL_SBS_MODULE == 1)
/** @addtogroup SBS
  * @{
  */

/** @defgroup SBS_Introduction SBS Introduction
  * @{

   The System configuration, Boot and Security (SBS) hardware abstraction layer provides a set of APIs to interface with
   the STM32 SBS.

   The SBS includes the following features:

     - Manage the I/O compensation cell.
     - Activate and deactivate the connection to TIM1.
     - Activate and deactivate the FPU interrupts.
     - Manage the compensation cells.
     - Activate and deactivate the NMI generation when a double ECC error occurs on FLASH.
     - Lock the core registers.
     - Get and clear the memory erase flags.
     - Set and get the ethernet external phy interrupt polarity and the ethernet phy interface.
     - Activate and deactivate the ADC channel pin remap.
  */
/**
  * @}
  */

/** @defgroup SBS_How_To_Use SBS How To Use
  * @{

# How to use the SBS HAL module driver

## The SBS HAL driver can be used as follows:

After startup, SBS peripheral is not active by default. Use HAL_RCC_SBS_EnableClock() function to enable
SBS APB3 clock.

This module provides different sets of APIs that allow to :


1. Manage floating point unit interrupts :
   Several APIs are available to manage the floating point unit interrupts
   - Enable and disable the floating point unit interrupts:
     - These functionalities are ensured respectively by HAL_SBS_EnableFPUIT() and HAL_SBS_DisableFPUIT().
   - Get the floating point unit interrupts state:
     - This functionality is ensured by HAL_SBS_IsEnabledFPUIT() function.

2. Manage TIM break inputs :
   This feature is used to control the system break interconnect to TIM1 break inputs
   - Enable and disable the TIM break inputs:
    - This functionality is ensured respectively by HAL_SBS_EnableTIMBreakInputs() function and
      HAL_SBS_DisableTIMBreakInputs() function.
   - Check the TIM break inputs activation:
     - This functionality is ensured by HAL_SBS_IsEnabledTIMBreakInputs() function.
   - \b Note: Some TIM break inputs disabling can only be done by a hardware reset.

3. Control the compensation cell :
   The SBS can be configured to manage the compensation cell
   - Set the compensation cell code source :
     - This functionality is ensured by HAL_SBS_SetCompensationCellCodeSrc() function.
   - Get the compensation cell code source :
     - This functionality is ensured by HAL_SBS_GetCompensationCellCodeSrc() function.
   - Enable and disable the compensation cell activation :
     - These functionalities are ensured respectively by HAL_SBS_EnableCompensationCell() and
       HAL_SBS_DisableCompensationCell().
   - Get the compensation cell state:
     - This functionality is ensured by HAL_SBS_IsEnabledCompensationCell() function.

4. Manage the compensation code :
   Several APIs are available to manage the compensation code
   - Get the compensation value of PMOS transistor :
     - This functionality is ensured by HAL_SBS_GetPMOSCompensationCellValue() function.
   - Get the compensation value of NMOS transistor :
     - This functionality is ensured by HAL_SBS_GetNMOSCompensationCellValue() function.
   - Set the compensation cell code:
     - This functionality is ensured by HAL_SBS_SetConfigxMOSCompensationCellCode() function.
   - Get the compensation cell code :
     - This functionality is ensured by HAL_SBS_GetConfigxMOSCompensationCellCode() function.

5. NMI double ECC error in FLASH Interface functions:
   Several APIs are available to control NMI double ECC error in FLASH Interface.
   - Enable and disable the NMI in case of double ECC error in FLASH Interface:
     - These functionalities are ensured respectively by HAL_SBS_FLASH_EnableECCNMI() and
       HAL_SBS_FLASH_DisableECCNMI() function.
   - Get if the NMI is Enabled in case of double ECC error in FLASH Interface:
     - This functionality is ensured by HAL_SBS_FLASH_IsEnabledECCNMI() function.

6. HDP levels management functions :
   Several APIs are available to manage HDPL
   - Set the HDPL value :
     - This functionality is ensured by HAL_SBS_SetHDPLevelValue() function.
   - Get the HDPL value :
     - This functionality is ensured by HAL_SBS_GetHDPLevelValue() function.
   - Set OBK HDPL value :
     - This functionality is ensured by HAL_SBS_SetHDPOBKLevelValue() function.
   - Get OBK HDPL value :
     - This functionality is ensured by HAL_SBS_GetHDPOBKLevelValue() function.

7. Lock functions :
   Several APIs are available to manage the lock mechanism
   - Lock the configuration of the Core registers :
     - This functionality is ensured by HAL_SBS_LockCoreRegisters() function.
   - Check the Core registers lock status :
     - This functionality is ensured by HAL_SBS_IsLockedCoreRegisters() function.

8. Flag management functions :
   Several APIs are available on the header file as "static_inline" functions to manage the memories erase status
   - Check if the memories erase status flags is active or not :
     - This functionality is ensured by HAL_SBS_IsActiveFlag() function.
   - Clear the memories erase status pending flags :
     - This functionality is ensured by HAL_SBS_ClearFlag() function.

9. Ethernet functions :
   Several APIs are available to manage the Ethernet features
   - Set the Ethernet external PHY interrupt polarity :
     - This functionality is ensured by HAL_SBS_SetETHExternalPHYInterruptPolarity() function.
   - Get the Ethernet external PHY interrupt polarity :
     - This functionality is ensured by HAL_SBS_GetETHExternalPHYInterruptPolarity() function.
   - Set the Ethernet PHY interface :
     - This functionality is ensured by HAL_SBS_SetETHPHYInterface() function.
   - Get the Ethernet PHY interface :
     - This functionality is ensured by HAL_SBS_GetETHPHYInterface() function.
   - Get the Ethernet TXLPI mode status :
     - This functionality is ensured by HAL_SBS_GetETHMACTXLPIStatus()function.
   - Get the Ethernet power-down acknowledge :
     - This functionality is ensured by HAL_SBS_GetETHPowerDownAck()function.

10. Pin Remap functions :
   Several APIs are available to manage the ADC channel pin remapping features
   - Enable and disable the ADC channel pin remap:
    - This functionality is ensured respectively by HAL_SBS_EnableADCChannelPinRemap() function and
      HAL_SBS_DisableADCChannelPinRemap() function.
   - Check the ADC channel pin remapping status:
     - This functionality is ensured by HAL_SBS_IsEnabledADCChannelPinRemap() function.
  */
/**
  * @}
  */

/** @defgroup SBS_Configuration_Table SBS Configuration Table
  * @{
# Configuration inside the SBS driver

Config defines        | Description        | Default value | Note
----------------------| -------------------| ------------- | -----------------------------------------------------------
USE_HAL_SBS_MODULE    | from hal_conf.h    | 1U            | When set to 1, the HAL SBS module is enabled.
USE_ASSERT_DBG_PARAM  | from IDE           | None          | When defined, enable the params assert.
  */
/**
  * @}
  */

/* Private define ----------------------------------------------------------------------------------------------------*/
/** @defgroup SBS_Private_Constants SBS Private Constants
  * @{
  */
#define SBS_CCELL_MAX_DELAY_MS  50U   /*!< Max compensation cell timeout value (unit: milliseconds) */
#define HAL_SBS_CCELL_SIZE      0x0FU /*!< Max size of compensation cell code                       */
/**
  * @}
  */

/* Private macro -----------------------------------------------------------------------------------------------------*/
/** @defgroup SBS_Private_Macros SBS Private Macros
  * @{
  */

/*! Set floating point unit interrupts check macro */
#define IS_SBS_SET_FLOATING_POINT_IT(floating_point)     \
  ((((floating_point) & (HAL_SBS_IT_FPU_ALL)) != 0U)     \
   && (((floating_point) & (~HAL_SBS_IT_FPU_ALL)) == 0U))

/*! Get floating point unit interrupts check macro */
#define IS_SBS_GET_FLOATING_POINT_IT(floating_point) \
  (((floating_point) == (HAL_SBS_IT_FPU_IOC))        \
   || ((floating_point) == (HAL_SBS_IT_FPU_DZC))     \
   || ((floating_point) == (HAL_SBS_IT_FPU_UFC))     \
   || ((floating_point) == (HAL_SBS_IT_FPU_OFC))     \
   || ((floating_point) == (HAL_SBS_IT_FPU_IDC))     \
   || ((floating_point) == (HAL_SBS_IT_FPU_IXC)))

/*! TIM break inputs check macro */
#define IS_SBS_SET_TIM_BREAK_INPUTS(break_input)               \
  ((((break_input) & HAL_SBS_TIM_BREAK_INPUTS_ALL) != 0U)      \
   && (((break_input) & (~HAL_SBS_TIM_BREAK_INPUTS_ALL)) == 0U))

/*! TIM break inputs disable check macro */
#define IS_SBS_DISABLE_TIM_BREAK_INPUTS(break_input)     \
  ((break_input) == (HAL_SBS_FLASH_ECC_DOUBLE_ERROR))

/*! Get TIM break inputs check macro */
#define IS_SBS_GET_TIM_BREAK_INPUTS(break_input)       \
  (((break_input) == HAL_SBS_FLASH_ECC_DOUBLE_ERROR)   \
   || ((break_input) == HAL_SBS_PVD)                   \
   || ((break_input) == HAL_SBS_SRAM_ECC_DOUBLE_ERROR) \
   || ((break_input) == HAL_SBS_LOCKUP_OUT))

/*! Compensation cell check macro */
#define IS_SBS_CCELL(comp_cell) ((comp_cell) == HAL_SBS_CCELL_VDDIO)

/*! Compensation code check macro */
#define IS_SBS_CCELL_CODE(code_select)             \
  (((code_select) == HAL_SBS_CCELL_CODE_DEFAULT)   \
   || ((code_select) == HAL_SBS_CCELL_CODE_CUSTOM))

/*! Get compensation cell check macro */
#define IS_SBS_GET_CCELL(comp_cell)             \
  (((comp_cell) == HAL_SBS_CCELL_VDDIO))

/*! XMOS compensation cell check macro */
#define IS_SBS_XMOS_CCELL_CODE(pmos_code,nmos_code) \
  (((pmos_code) <= HAL_SBS_CCELL_SIZE)              \
   && ((nmos_code) <= HAL_SBS_CCELL_SIZE))

/*! Hide protection level check macro */
#define IS_SBS_HDP_LEVEL(level)        \
  (((level) == HAL_SBS_HDP_LEVEL_1)    \
   || ((level) == HAL_SBS_HDP_LEVEL_2) \
   || ((level) == HAL_SBS_HDP_LEVEL_3))
#if defined(SBS_NEXTHDPLCR_NEXTHDPL)

/*! Next Hide Protection Level Selection check macro */
#define IS_SBS_HDPOBK_SELECTION(hdp_obk_select)      \
  (((hdp_obk_select)   == HAL_SBS_HDP_OBK_LEVEL_0)   \
   || ((hdp_obk_select) == HAL_SBS_HDP_OBK_LEVEL_1)  \
   || ((hdp_obk_select) == HAL_SBS_HDP_OBK_LEVEL_2)  \
   || ((hdp_obk_select) == HAL_SBS_HDP_OBK_LEVEL_3))
#endif /* SBS_NEXTHDPLCR_NEXTHDPL */

/*! Macro to check the Core lock registers. */
#define IS_SBS_LOCK_CORE_REGS(core_regs) ((((core_regs) & (HAL_SBS_CORE_ALL_REGS)) != 0U) \
                                          && (((core_regs) & (~HAL_SBS_CORE_ALL_REGS)) == 0U))

/*! Macro to check the Core locked registers. */
#define IS_SBS_LOCKED_CORE_REGS(core_regs) (((core_regs) == HAL_SBS_CORE_VTOR_REG) \
                                            || ((core_regs) == HAL_SBS_CORE_MPU_REG))

#if defined(ETH1_BASE)
/*! ETH Interface selection check macro */
#define IS_SBS_ETHPHY_ITF(interface)             \
  (((interface) == HAL_SBS_ETHPHY_ITF_GMII_MII)   \
   || ((interface) == HAL_SBS_ETHPHY_ITF_RMII))

/*! ETH Polarity configuration check macro */
#define IS_SBS_ETHPHY_IT_POL(polarity)             \
  (((polarity) == HAL_SBS_ETHPHY_IT_POL_ACTIVE_HIGH)   \
   || ((polarity) == HAL_SBS_ETHPHY_IT_POL_ACTIVE_LOW))
#endif /* ETH1_BASE */

#if defined(SBS_PMCR_ADC1_IN2_REMAP)
/*! Set ADC channel pin remap check macro */
#define IS_SBS_SET_ADC_CHANNEL_PIN_REMAP(channel_pin_remap)   \
  ((((channel_pin_remap) & (HAL_SBS_REMAP_ADC_IN_ALL)) != 0U) \
   && (((channel_pin_remap) & (~HAL_SBS_REMAP_ADC_IN_ALL)) == 0U))

/*! Get ADC channel pin remap check macro */
#define IS_SBS_GET_ADC_CHANNEL_PIN_REMAP(channel_pin_remap) \
  (((channel_pin_remap) == HAL_SBS_REMAP_ADC_IN7_TO_PB1)    \
   || ((channel_pin_remap) == HAL_SBS_REMAP_ADC_IN6_TO_PB0) \
   || ((channel_pin_remap) == HAL_SBS_REMAP_ADC_IN5_TO_PC5) \
   || ((channel_pin_remap) == HAL_SBS_REMAP_ADC_IN2_TO_PC4))
#endif /* SBS_PMCR_ADC1_IN2_REMAP */
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup SBS_Exported_Functions
  * @{
  */

/** @addtogroup SBS_Exported_Functions_Group1
  * @{
  This section provides functions to manage the floating point interrupts:
  - Call HAL_SBS_EnableFPUIT() to enable the floating point unit interrupt(s).
  - Call HAL_SBS_DisableFPUIT() to disable the floating point unit interrupt(s).
  - Call HAL_SBS_IsEnabledFPUIT() to check if the floating point unit interrupt is enabled.
  */

/**
  * @brief  Enable the floating point unit interrupt(s).
  * @param  floating_point This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_SBS_IT_FPU_IOC
  *         @arg @ref HAL_SBS_IT_FPU_DZC
  *         @arg @ref HAL_SBS_IT_FPU_UFC
  *         @arg @ref HAL_SBS_IT_FPU_OFC
  *         @arg @ref HAL_SBS_IT_FPU_IDC
  *         @arg @ref HAL_SBS_IT_FPU_IXC
  *         @arg @ref HAL_SBS_IT_FPU_ALL
  */
void HAL_SBS_EnableFPUIT(uint32_t floating_point)
{
  ASSERT_DBG_PARAM(IS_SBS_SET_FLOATING_POINT_IT(floating_point));

  LL_SBS_EnableFPUIT(floating_point);
}

/**
  * @brief  Disable the floating point unit interrupt(s).
  * @param  floating_point This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_SBS_IT_FPU_IOC
  *         @arg @ref HAL_SBS_IT_FPU_DZC
  *         @arg @ref HAL_SBS_IT_FPU_UFC
  *         @arg @ref HAL_SBS_IT_FPU_OFC
  *         @arg @ref HAL_SBS_IT_FPU_IDC
  *         @arg @ref HAL_SBS_IT_FPU_IXC
  *         @arg @ref HAL_SBS_IT_FPU_ALL
  */
void HAL_SBS_DisableFPUIT(uint32_t floating_point)
{
  ASSERT_DBG_PARAM(IS_SBS_SET_FLOATING_POINT_IT(floating_point));

  LL_SBS_DisableFPUIT(floating_point);
}

/**
  * @brief  Check if the floating point unit interrupt is enabled.
  * @param  floating_point This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_IT_FPU_IOC
  *         @arg @ref HAL_SBS_IT_FPU_DZC
  *         @arg @ref HAL_SBS_IT_FPU_UFC
  *         @arg @ref HAL_SBS_IT_FPU_OFC
  *         @arg @ref HAL_SBS_IT_FPU_IDC
  *         @arg @ref HAL_SBS_IT_FPU_IXC
  * @retval hal_sbs_it_fpu_status_t Floating point interrupt status.
  */
hal_sbs_it_fpu_status_t HAL_SBS_IsEnabledFPUIT(uint32_t floating_point)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_FLOATING_POINT_IT(floating_point));

  return (hal_sbs_it_fpu_status_t)LL_SBS_IsEnabledFPUIT(floating_point);
}
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group2
  * @{
  This section provides functions to manage the TIM break inputs:
  - Call HAL_SBS_EnableTIMBreakInputs() to enable the TIM break inputs.
  - Call HAL_SBS_DisableTIMBreakInputs() to disable the TIM break inputs.
  - Call HAL_SBS_IsEnabledTIMBreakInputs() to check if the TIM break inputs is enabled.
  */

/**
  * @brief  Enable the TIM break inputs.
  * @param  break_input This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_SBS_FLASH_ECC_DOUBLE_ERROR
  *         @arg @ref HAL_SBS_PVD (*)
  *         @arg @ref HAL_SBS_SRAM_ECC_DOUBLE_ERROR (*)
  *         @arg @ref HAL_SBS_LOCKUP_OUT (*)
  * @note   (*) TIM break inputs disabling can only be done by a hardware reset.
  */
void HAL_SBS_EnableTIMBreakInputs(uint32_t break_input)
{
  ASSERT_DBG_PARAM(IS_SBS_SET_TIM_BREAK_INPUTS(break_input));

  LL_SBS_EnableTIMBreakInputs(break_input);
}

/**
  * @brief  Disable the TIM break inputs.
  * @param  break_input This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_SBS_FLASH_ECC_DOUBLE_ERROR
  */
void HAL_SBS_DisableTIMBreakInputs(uint32_t break_input)
{
  ASSERT_DBG_PARAM(IS_SBS_DISABLE_TIM_BREAK_INPUTS(break_input));

  LL_SBS_DisableTIMBreakInputs(break_input);
}

/**
  * @brief  Check if the TIM break inputs is enabled.
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_FLASH_ECC_DOUBLE_ERROR
  *         @arg @ref HAL_SBS_PVD (*)
  *         @arg @ref HAL_SBS_SRAM_ECC_DOUBLE_ERROR (*)
  *         @arg @ref HAL_SBS_LOCKUP_OUT (*)
  * @retval hal_sbs_tim_break_input_status_t TIM break inputs status.
  */
hal_sbs_tim_break_input_status_t HAL_SBS_IsEnabledTIMBreakInputs(uint32_t break_input)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_TIM_BREAK_INPUTS(break_input));

  return (hal_sbs_tim_break_input_status_t)LL_SBS_IsEnabledTIMBreakInputs(break_input);
}
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group3
  * @{
  This section provides functions to control the compensation cell:
  - Call HAL_SBS_SetCompensationCellCodeSrc() to set the compensation cell code source.
  - Call HAL_SBS_GetCompensationCellCodeSrc() to get the compensation cell code source.
  - Call HAL_SBS_EnableCompensationCell() to enable the I/O compensation cell.
  - Call HAL_SBS_DisableCompensationCell() to disable the I/O compensation cell.
  - Call HAL_SBS_IsEnabledCompensationCell() to check if the I/O compensation cell is enabled.
  */

/**
  * @brief  Set the compensation cell code source.
  * @param  comp_cell This parameter has the following value:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @param  code_select This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_CODE_DEFAULT
  *         @arg @ref HAL_SBS_CCELL_CODE_CUSTOM
  */
void HAL_SBS_SetCompensationCellCodeSrc(uint32_t comp_cell, hal_sbs_ccell_code_src_t code_select)
{
  ASSERT_DBG_PARAM(IS_SBS_CCELL_CODE(code_select));
  ASSERT_DBG_PARAM(IS_SBS_CCELL(comp_cell));

  LL_SBS_SetCompensationCellCodeSrc(comp_cell, (uint32_t)code_select);
}

/**
  * @brief  Get the compensation cell code source.
  * @param  comp_cell This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @retval hal_sbs_ccell_code_src_t Compensation cell code source.
  */
hal_sbs_ccell_code_src_t HAL_SBS_GetCompensationCellCodeSrc(uint32_t comp_cell)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_CCELL(comp_cell));

  return (hal_sbs_ccell_code_src_t)LL_SBS_GetCompensationCellCodeSrc(comp_cell);
}

/**
  * @brief  Enable the I/O compensation cell.
  * @param  comp_cell This parameter has the following value:
  *         @arg @ref HAL_SBS_CCELL_VDDIO

  * @retval HAL_ERROR The compensation cell was not enabled within the allowed timeout period.
  * @retval HAL_OK    Compensation cell is enabled.
  */
hal_status_t HAL_SBS_EnableCompensationCell(uint32_t comp_cell)
{
  uint32_t timeout = (SBS_CCELL_MAX_DELAY_MS * (SystemCoreClock / 1000U)) + 1U;
  uint32_t comp_rdy;

  ASSERT_DBG_PARAM(IS_SBS_CCELL(comp_cell));

  LL_SBS_EnableCompensationCell(comp_cell);

  while (timeout > 0U)
  {
    comp_rdy = LL_SBS_IsActiveFlag_RDY1();

    if ((comp_rdy & comp_cell) == comp_cell)
    {
      return HAL_OK;
    }

    timeout--;
  }

  return HAL_ERROR;
}

/**
  * @brief  Disable the I/O compensation cell.
  * @param  comp_cell This parameter has the following value:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  */
void HAL_SBS_DisableCompensationCell(uint32_t comp_cell)
{
  ASSERT_DBG_PARAM(IS_SBS_CCELL(comp_cell));

  LL_SBS_DisableCompensationCell(comp_cell);
}

/**
  * @brief  Check if the I/O compensation cell is enabled.
  * @param  comp_cell This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @retval hal_sbs_ccell_status_t I/O Compensation cell status.
  */
hal_sbs_ccell_status_t HAL_SBS_IsEnabledCompensationCell(uint32_t comp_cell)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_CCELL(comp_cell));

  return ((hal_sbs_ccell_status_t)LL_SBS_IsEnabledCompensationCell(comp_cell));
}
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group4
  * @{
  This section provides functions to manage the compensation cell:
  - Call HAL_SBS_GetPMOSCompensationCellValue() to get the PMOS compensation value of selected compensation cell.
  - Call HAL_SBS_GetNMOSCompensationCellValue() to get the NMOS compensation value of selected compensation cell.
  - Call HAL_SBS_SetConfigxMOSCompensationCellCode() to set the compensation cell code.
  - Call HAL_SBS_GetConfigxMOSCompensationCellCode() to get the compensation cell code.
  */

/**
  * @brief  Get the PMOS compensation value of the selected compensation cell.
  * @param  comp_cell This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @retval uint32_t Value of the PMOS compensation cell.
  */
uint32_t HAL_SBS_GetPMOSCompensationCellValue(uint32_t comp_cell)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_CCELL(comp_cell));

  return (LL_SBS_GetPMOSCompensationCellValue(comp_cell));
}

/**
  * @brief  Get the NMOS compensation value of the selected compensation cell.
  * @param  comp_cell This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @retval uint32_t Value of the NMOS compensation cell.
  */
uint32_t HAL_SBS_GetNMOSCompensationCellValue(uint32_t comp_cell)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_CCELL(comp_cell));

  return (LL_SBS_GetNMOSCompensationCellValue(comp_cell));
}

/**
  * @brief  Set the compensation cell code.
  * @param  comp_cell This parameter has the following value:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @param  pmos_code PMOS value to be applied to the compensation cell
  * @param  nmos_code NMOS value to be applied to the compensation cell
  */
void HAL_SBS_SetConfigxMOSCompensationCellCode(uint32_t comp_cell, uint32_t pmos_code, uint32_t nmos_code)
{
  ASSERT_DBG_PARAM(IS_SBS_CCELL(comp_cell));
  ASSERT_DBG_PARAM(IS_SBS_XMOS_CCELL_CODE(pmos_code, nmos_code));

  if ((comp_cell & HAL_SBS_CCELL_VDDIO) != 0U)
  {
    LL_SBS_SetxMOSVddIOCompensationCellCode(pmos_code, nmos_code);
  }
}

/**
  * @brief  Get the compensation cell code.
  * @param  comp_cell This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CCELL_VDDIO
  * @param  p_pmos_code Pointer to PMOS register of the selected compensation cell
  * @param  p_nmos_code Pointer to NMOS register of the selected compensation cell
  */
void HAL_SBS_GetConfigxMOSCompensationCellCode(uint32_t comp_cell, uint32_t *p_pmos_code, uint32_t *p_nmos_code)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_CCELL(comp_cell));
  ASSERT_DBG_PARAM(p_pmos_code != NULL);
  ASSERT_DBG_PARAM(p_nmos_code != NULL);

  *p_pmos_code = (LL_SBS_GetPMOSCompensationCellCode(comp_cell) >> (STM32_POSITION_VAL(comp_cell << 1U) * 4U));
  *p_nmos_code = (LL_SBS_GetNMOSCompensationCellCode(comp_cell) >> (STM32_POSITION_VAL(comp_cell) * 4U));
}
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group5
  * @{
  This section provides functions to enable/disable the NMI in case of double ECC error in FLASH Interface:
  - Call HAL_SBS_FLASH_EnableECCNMI() to enable the NMI in case of double ECC error in FLASH Interface.
  - Call HAL_SBS_FLASH_DisableECCNMI() to disable the NMI in case of double ECC error in FLASH Interface.
  - Call HAL_SBS_FLASH_IsEnabledECCNMI() to Check if the NMI is Enabled in case of double ECC error in
    FLASH Interface.
  */

/**
  * @brief  Enable the NMI in case of double ECC error in FLASH Interface.
  */
void HAL_SBS_FLASH_EnableECCNMI(void)
{
  LL_SBS_FLASH_EnableECCNMI();
}

/**
  * @brief  Disable the NMI in case of double ECC error in FLASH Interface.
  */
void HAL_SBS_FLASH_DisableECCNMI(void)
{
  LL_SBS_FLASH_DisableECCNMI();
}

/**
  * @brief  Check if the NMI is Enabled in case of double ECC error in FLASH Interface.
  * @retval State of bit (1 or 0).
  */
uint32_t HAL_SBS_FLASH_IsEnabledECCNMI(void)
{
  return (LL_SBS_FLASH_IsEnabledECCNMI());
}
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group6
  * @{
  This section provides functions to set and get the HDP level value :
  - Call HAL_SBS_SetHDPLevelValue() to set the HDP level value.
  - Call HAL_SBS_GetHDPLevelValue() to get the HDP level value.
  - Call HAL_SBS_SetHDPOBKLevelValue() to set the OBK-HDP level value.
  - Call HAL_SBS_GetHDPOBKLevelValue() to get the OBK-HDP level value.
  */

/**
  * @brief  Set the HDP level value.
  * @param  value Value of the HDP level.
  *         This parameter can be one of the following values:
  *         @arg @ref  HAL_SBS_HDP_LEVEL_1
  *         @arg @ref  HAL_SBS_HDP_LEVEL_2
  *         @arg @ref  HAL_SBS_HDP_LEVEL_3
  * @retval HAL_ERROR Invalid HDP level value not set.
  * @retval HAL_OK    Valid HDP level value set correctly.
  */
hal_status_t HAL_SBS_SetHDPLevelValue(hal_sbs_hdp_level_value_t value)
{
  hal_sbs_hdp_level_value_t prev_hdp_level_value;

  ASSERT_DBG_PARAM(IS_SBS_HDP_LEVEL(value));

  prev_hdp_level_value = (hal_sbs_hdp_level_value_t)LL_SBS_GetHDPLevel();

  if (((value == HAL_SBS_HDP_LEVEL_2) && (prev_hdp_level_value == HAL_SBS_HDP_LEVEL_1)) \
      || ((value == HAL_SBS_HDP_LEVEL_3) && (prev_hdp_level_value == HAL_SBS_HDP_LEVEL_2)))
  {
    LL_SBS_IncrementHDPLevel();

    return HAL_OK;
  }

  return HAL_ERROR;
}

/**
  * @brief  Get the HDP level value.
  * @retval hal_sbs_hdp_level_value_t HDPL value.
  */
hal_sbs_hdp_level_value_t HAL_SBS_GetHDPLevelValue(void)
{
  return ((hal_sbs_hdp_level_value_t)LL_SBS_GetHDPLevel());
}
#if defined(SBS_NEXTHDPLCR_NEXTHDPL)

/**
  * @brief  Set the OBK-HDP level value.
  * @param  value Value of the increment to be added to HDPL value to generate the OBK-HDPL.
  *         This parameter can be one of the following values:
  *         @arg @ref  HAL_SBS_HDP_OBK_LEVEL_0 : HDPL (default value)
  *         @arg @ref  HAL_SBS_HDP_OBK_LEVEL_1 : HDPL + 1
  *         @arg @ref  HAL_SBS_HDP_OBK_LEVEL_2 : HDPL + 2
  *         @arg @ref  HAL_SBS_HDP_OBK_LEVEL_3 : HDPL + 3
  */
void HAL_SBS_SetHDPOBKLevelValue(hal_sbs_hdp_obk_level_value_t value)
{
  ASSERT_DBG_PARAM(IS_SBS_HDPOBK_SELECTION(value));

  LL_SBS_SetOBKHDPLevel((uint32_t)value);
}

/**
  * @brief  Get the OBK-HDP level value.
  * @retval hal_sbs_hdp_obk_level_value_t  OBK-HDP level value.
  */
hal_sbs_hdp_obk_level_value_t HAL_SBS_GetHDPOBKLevelValue(void)
{
  return ((hal_sbs_hdp_obk_level_value_t)LL_SBS_GetOBKHDPLevel());
}
#endif /* SBS_NEXTHDPLCR_NEXTHDPL */
/**
  * @}
  */

/** @addtogroup SBS_Exported_Functions_Group7
  * @{
  This section provides functions to manage the lock feature :
  - Call HAL_SBS_LockCoreRegisters() to lock the Core registers.
  - Call HAL_SBS_IsLockedCoreRegisters() to check if the Core registers is locked.
  */

/**
  * @brief Lock the Core registers.
  * @param core_regs This parameter can be one or a combination of the following values:
  *        @arg @ref HAL_SBS_CORE_VTOR_REG
  *        @arg @ref HAL_SBS_CORE_MPU_REG
  *        @arg @ref HAL_SBS_CORE_ALL_REGS
  * @note The unlock can only be done with a system reset
  */
void HAL_SBS_LockCoreRegisters(uint32_t core_regs)
{
  ASSERT_DBG_PARAM(IS_SBS_LOCK_CORE_REGS(core_regs));

  LL_SBS_CPU_LockRegisters(core_regs);
}

/**
  * @brief  Check if the Core registers are locked.
  * @param  core_regs This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_CORE_VTOR_REG
  *         @arg @ref HAL_SBS_CORE_MPU_REG
  * @retval hal_sbs_core_reg_lock_status_t SBS Core registers lock status.
  */
hal_sbs_core_reg_lock_status_t HAL_SBS_IsLockedCoreRegisters(uint32_t core_regs)
{
  ASSERT_DBG_PARAM(IS_SBS_LOCKED_CORE_REGS(core_regs));

  return (hal_sbs_core_reg_lock_status_t)LL_SBS_CPU_IsLockedRegisters(core_regs);
}

/**
  * @}
  */

#if defined(ETH1_BASE)
/** @addtogroup SBS_Exported_Functions_Group9
  * @{
  This section provides functions to management the Ethernet features :
  - Call HAL_SBS_SetETHExternalPHYInterruptPolarity() to set the ethernet external PHY interrupt polarity.
  - Call HAL_SBS_GetETHExternalPHYInterruptPolarity() to get the ethernet external PHY interrupt polarity.
  - Call HAL_SBS_SetETHPHYInterface() to set the ethernet PHY interface.
  - Call HAL_SBS_GetETHPHYInterface() to get the ethernet PHY interface.
  - Call HAL_SBS_GetETHMACTXLPIStatus() to get the Ethernet TXLPI mode status.
  - Call HAL_SBS_GetETHPowerDownAck() to get the Ethernet power-down sequence acknowledge.
  */

/**
  * @brief  Set the ethernet external PHY interrupt polarity.
  * @param  ethx ETH1 instance.
  * @param  it_pol interrupt polarity to set based on @ref hal_sbs_ethphy_it_pol_active_level_t
  */
void HAL_SBS_SetETHExternalPHYInterruptPolarity(const ETH_TypeDef *ethx,
                                                hal_sbs_ethphy_it_pol_active_level_t it_pol)
{
  ASSERT_DBG_PARAM(IS_SBS_ETHPHY_IT_POL(it_pol));

  STM32_UNUSED(ethx);
  LL_SBS_SetETHExternalPHYInterruptPolarity(LL_SBS_PERIPH_ETH1, (uint32_t)it_pol);
}

/**
  * @brief  Get the ethernet external PHY interrupt polarity.
  * @param  ethx ETH1 instance.
  * @retval Ethernet interrupt polarity based on @ref hal_sbs_ethphy_it_pol_active_level_t
  */
hal_sbs_ethphy_it_pol_active_level_t HAL_SBS_GetETHExternalPHYInterruptPolarity(ETH_TypeDef *ethx)
{
  STM32_UNUSED(ethx);
  return (hal_sbs_ethphy_it_pol_active_level_t)LL_SBS_GetETHExternalPHYInterruptPolarity(LL_SBS_PERIPH_ETH1);
}

/**
  * @brief  Set the ethernet PHY interface.
  * @param  ethx ETH1 instance.
  * @param  phy_int phy interface to set based on @ref hal_sbs_ethphy_itf_t
  */
void HAL_SBS_SetETHPHYInterface(const ETH_TypeDef *ethx, hal_sbs_ethphy_itf_t phy_int)
{
  ASSERT_DBG_PARAM(IS_SBS_ETHPHY_ITF(phy_int));

  STM32_UNUSED(ethx);
  LL_SBS_SetETHPHYInterface(LL_SBS_PERIPH_ETH1, (uint32_t)phy_int);
}

/**
  * @brief  Get the ethernet PHY interface.
  * @param  ethx ETH1 instance.
  * @retval Ethernet phy interface based on @ref hal_sbs_ethphy_itf_t
  */
hal_sbs_ethphy_itf_t HAL_SBS_GetETHPHYInterface(ETH_TypeDef *ethx)
{
  STM32_UNUSED(ethx);
  return (hal_sbs_ethphy_itf_t)LL_SBS_GetETHPHYInterface(LL_SBS_PERIPH_ETH1);
}

/**
  * @brief  Get the Ethernet TXLPI mode status.
  * @param  ethx ETH1 instance.
  * @retval Ethernet TXLPI mode status based on @ref hal_sbs_ethmac_txlpi_status_t
  */
hal_sbs_ethmac_txlpi_status_t HAL_SBS_GetETHMACTXLPIStatus(ETH_TypeDef *ethx)
{
  STM32_UNUSED(ethx);
  return (hal_sbs_ethmac_txlpi_status_t)LL_SBS_GetETHMACTXLPIStatus(LL_SBS_PERIPH_ETH1);
}

/**
  * @brief  Get the Ethernet power-down sequence acknowledge.
  * @param  ethx ETH1 instance.
  * @retval Ethernet power-down sequence acknowledge based on @ref hal_sbs_eth_power_down_seq_ack_t
  */
hal_sbs_eth_power_down_seq_ack_t HAL_SBS_GetETHPowerDownAck(ETH_TypeDef *ethx)
{
  STM32_UNUSED(ethx);
  return (hal_sbs_eth_power_down_seq_ack_t)LL_SBS_GetETHPowerDownAck(LL_SBS_PERIPH_ETH1);
}
/**
  * @}
  */
#endif /* ETH1_BASE */

#if defined(SBS_PMCR_ADC1_IN2_REMAP)
/** @addtogroup SBS_Exported_Functions_Group10
  * @{
  This section provides functions to management the ADC channel pin remap feature :
  - Call HAL_SBS_EnableADCChannelPinRemap() to enable the ADC channel pin remap.
  - Call HAL_SBS_DisableADCChannelPinRemap() to disable the ADC channel pin remap.
  - Call HAL_SBS_IsEnabledADCChannelPinRemap() to check the ADC channel pin remapping status.
  */

/**
  * @brief Enable the ADC channel pin remap.
  * @param adc_channel_pin_remap This parameter can be one or a combination of the following values:
  *        @arg @ref HAL_SBS_REMAP_ADC_IN7_TO_PB1
  *        @arg @ref HAL_SBS_REMAP_ADC_IN6_TO_PB0
  *        @arg @ref HAL_SBS_REMAP_ADC_IN5_TO_PC5
  *        @arg @ref HAL_SBS_REMAP_ADC_IN2_TO_PC4
  *        @arg @ref HAL_SBS_REMAP_ADC_IN_ALL
  */
void HAL_SBS_EnableADCChannelPinRemap(uint32_t adc_channel_pin_remap)
{
  ASSERT_DBG_PARAM(IS_SBS_SET_ADC_CHANNEL_PIN_REMAP(adc_channel_pin_remap));

  LL_SBS_EnableADCChannelPinRemap(adc_channel_pin_remap);
}

/**
  * @brief Disable the ADC channel pin remap.
  * @param adc_channel_pin_remap This parameter can be one or a combination of the following values:
  *        @arg @ref HAL_SBS_REMAP_ADC_IN7_TO_PB1
  *        @arg @ref HAL_SBS_REMAP_ADC_IN6_TO_PB0
  *        @arg @ref HAL_SBS_REMAP_ADC_IN5_TO_PC5
  *        @arg @ref HAL_SBS_REMAP_ADC_IN2_TO_PC4
  *        @arg @ref HAL_SBS_REMAP_ADC_IN_ALL
  */
void HAL_SBS_DisableADCChannelPinRemap(uint32_t adc_channel_pin_remap)
{
  ASSERT_DBG_PARAM(IS_SBS_SET_ADC_CHANNEL_PIN_REMAP(adc_channel_pin_remap));

  LL_SBS_DisableADCChannelPinRemap(adc_channel_pin_remap);
}

/**
  * @brief  Check if the ADC channel pin remap is enabled or disabled.
  * @param  adc_channel_pin_remap This parameter can be one of the following values:
  *         @arg @ref HAL_SBS_REMAP_ADC_IN7_TO_PB1
  *         @arg @ref HAL_SBS_REMAP_ADC_IN6_TO_PB0
  *         @arg @ref HAL_SBS_REMAP_ADC_IN5_TO_PC5
  *         @arg @ref HAL_SBS_REMAP_ADC_IN2_TO_PC4
  * @retval hal_sbs_adc_channel_pin_remap_status_t ADC channel pin remapping status.
  */
hal_sbs_adc_channel_pin_remap_status_t HAL_SBS_IsEnabledADCChannelPinRemap(uint32_t adc_channel_pin_remap)
{
  ASSERT_DBG_PARAM(IS_SBS_GET_ADC_CHANNEL_PIN_REMAP(adc_channel_pin_remap));

  return (hal_sbs_adc_channel_pin_remap_status_t)LL_SBS_IsEnabledADCChannelPinRemap(adc_channel_pin_remap);
}
/**
  * @}
  */
#endif /* SBS_PMCR_ADC1_IN2_REMAP */

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_HAL_SBS_MODULE */
#endif /* SBS */

/**
  * @}
  */
