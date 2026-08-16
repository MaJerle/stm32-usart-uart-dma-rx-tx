/**
  ******************************************************************************
  * @file    stm32c5xx_hal_crc.c
  * @brief   CRC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Cyclic Redundancy Check (CRC) peripheral:
  *           + Initialization and deinitialization functions
  *           + Configuration functions
  *           + I/O operation functions
  *           + Peripheral state and error functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */
/** @defgroup CRC_Introduction CRC Introduction
  * @{

  The CRC hardware abstraction layer provides a set of APIs to configure and control the CRC peripheral on
  STM32 microcontrollers.

  The CRC (Cyclic Redundancy Check) calculation unit is used to compute a CRC code from 8-, 16-, or 32-bit data
  words and a generator polynomial.

  Among other applications, CRC-based techniques are used to verify data transmission or storage integrity.
  Within the scope of functional safety standards, they offer a means of verifying flash memory integrity.

  The CRC calculation unit helps compute a software signature at runtime, to be compared with a
  reference signature generated at link time and stored at a given memory location.


  */
/**
  * @}
  */

/** @defgroup CRC_How_To_Use CRC How To Use
  * @{
# How to use the CRC HAL module driver

## Use the CRC HAL driver as follows:

- Declare a @ref hal_crc_handle_t handle structure, for example:
   @ref hal_crc_handle_t  hcrc;

- Initialize the CRC handle by calling the HAL_CRC_Init() API. This API performs these operations:
 - Associate the instance with the handle
 - Initialize the handle state to HAL_CRC_STATE_IDLE

- Enable the CRC peripheral clock:
  - Either at application level by calling the **HAL_RCC_CRC_EnableClock()** API
  - Or set the USE_HAL_CRC_CLK_ENABLE_MODEL define to HAL_CLK_ENABLE_PERIPH_ONLY within
    the stm32c5xx_hal_conf.h file. In this case, the CRC clock is enabled within the HAL_CRC_Init()
    API

- Keep the default configuration (default register values) or configure the CRC module with user values:
  - Declare a @ref hal_crc_config_t structure
  - Fill all parameters of the declared configuration structure
  - Call the HAL_CRC_SetConfig() function. This function: \n
    Update the CRC registers according to the user configuration provided by the input config structure

- To restore the CRC default configuration, use the HAL_CRC_ResetConfig() API: \n
   This function resets the CRC configuration to the default one by setting the following fields
    to their default values:
    - The polynomial coefficient is set to 0x04C11DB7U
    - The polynomial size is set to 32-bits
    - The CRC init value is set to 0xFFFFFFFFU
    - The input reversibility mode is set to none
    - The output reversibility mode is set to none

- For CRC I/O operations, one operation mode is available within this driver: polling mode I/O operation
  - Compute the CRC value of the input data buffer starting with the configured CRC initialization value
    using the HAL_CRC_Calculate() function
  - Compute the CRC value of the input data buffer starting with the previously computed CRC
    using the HAL_CRC_Accumulate() function

- Deinitialize the CRC peripheral by calling the HAL_CRC_DeInit() API. This API performs these operations:
 - Reset the CRC configuration to the default one by setting the following fields to their default values:
    - The polynomial coefficient is set to 0x04C11DB7U
    - The polynomial size is set to 32-bits
    - The CRC init value is set to 0xFFFFFFFFU
    - The input reversibility mode is set to none
    - The output reversibility mode is set to none
 - Reset the independent data value to the default one (0xFFFFFFFFU)
 - Reset the handle state to the HAL_CRC_STATE_RESET
  */
/**
  * @}
  */

/** @defgroup CRC_Configuration_Table CRC Configuration Table
  * @{
## Configuration inside the CRC driver

Config defines               | Description               | Default value              | Note
---------------------------- | ------------------------- | -------------------------- | -----------------------------
PRODUCT                      | from IDE                  | NA                         | Example: STM32C5XX
USE_ASSERT_DBG_PARAM         | from IDE                  | NA                         | Enable parameter asserts
USE_ASSERT_DBG_STATE         | from IDE                  | NA                         | Enable state asserts
USE_HAL_CHECK_PARAM          | from stm32c5xx_hal_conf.h | 0                          | Runtime parameter check
USE_HAL_CRC_MODULE           | from stm32c5xx_hal_conf.h | 1                          | Enable the HAL CRC module
USE_HAL_CRC_CLK_ENABLE_MODEL | from stm32c5xx_hal_conf.h | HAL_CLK_ENABLE_PERIPH_ONLY | Enable HAL_CRC_CLK
USE_HAL_CRC_USER_DATA        | from stm32c5xx_hal_conf.h | 0                          | Allow user data usage
  */
/**
  * @}
  */

#if defined(USE_HAL_CRC_MODULE) && (USE_HAL_CRC_MODULE == 1U)

/* Private constants ---------------------------------------------------------*/
/** @defgroup CRC_Private_Constants CRC Private Constants
  * @{
  */
#define CRC_POLYSIZE_16B (16U) /*!< 16-bit polynomial */
#define CRC_POLYSIZE_8B  (8U)  /*!< 8-bit polynomial  */
#define CRC_POLYSIZE_7B  (7U)  /*!< 7-bit polynomial  */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup CRC_Private_Macros CRC Private Macros
  * @{
  */

/*! Macro to check the polynomial size */
#define IS_CRC_POL_SIZE(size)     (((size) == HAL_CRC_POLY_SIZE_32B) \
                                   || ((size) == HAL_CRC_POLY_SIZE_16B) \
                                   || ((size) == HAL_CRC_POLY_SIZE_8B) \
                                   || ((size) == HAL_CRC_POLY_SIZE_7B))

/*! Macro to check the input reverse mode */
#define IS_CRC_INPUTDATA_REVERSE_MODE(mode)     (((mode) == HAL_CRC_INDATA_REVERSE_NONE) \
                                                 || ((mode) == HAL_CRC_INDATA_REVERSE_BYTE) \
                                                 || ((mode) == HAL_CRC_INDATA_REVERSE_HALFWORD) \
                                                 || ((mode) == HAL_CRC_INDATA_REVERSE_WORD) \
                                                 || ((mode) == HAL_CRC_INDATA_REVERSE_HALFWORD_BYWORD) \
                                                 || ((mode) == HAL_CRC_INDATA_REVERSE_BYTE_BYWORD))

/*! Macro to check the output reverse mode */
#define IS_CRC_OUTPUTDATA_REVERSE_MODE(mode)    (((mode) == HAL_CRC_OUTDATA_REVERSE_NONE) \
                                                 || ((mode) == HAL_CRC_OUTDATA_REVERSE_BIT) \
                                                 || ((mode) == HAL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD) \
                                                 || ((mode) == HAL_CRC_OUTDATA_REVERSE_BYTE_BYWORD))

/*! Macro to get the handle instance */
#define CRC_GET_INSTANCE(handle) ((CRC_TypeDef *)((uint32_t)(handle)->instance))

/*! Macro to check the coherence between the input reverse mode and the user data size */
#define IS_CRC_DATA_SIZE_VALID(handle, size) ((size != 0U) && ((((CRC_GET_INSTANCE(handle)->CR) & CRC_CR_REV_IN) \
                                                                == CRC_CR_REV_IN_1) ? \
                                                               ((size % 2U) == 0U) : \
                                                               ((((CRC_GET_INSTANCE(handle)->CR) & CRC_CR_REV_IN) \
                                                                 == (CRC_CR_REV_IN_1 | CRC_CR_REV_IN_0)) \
                                                                || (((CRC_GET_INSTANCE(handle)->CR) & CRC_CR_RTYPE_IN) \
                                                                    == CRC_CR_RTYPE_IN)) ? ((size % 4U) == 0U) : 1))

/*! Macro to check the coherence between the output reverse mode and the polynomial size */
#define IS_CRC_OUTPUT_REVERSE_MODE_VALID(poly_size, mode) ((poly_size != 0U) ? \
                                                           ((mode == HAL_CRC_OUTDATA_REVERSE_NONE) \
                                                            || (mode == HAL_CRC_OUTDATA_REVERSE_BIT)) : 1)
/**
  * @}
  */

/** @defgroup CRC_Private_Functions CRC Private Functions
  * @{
  */
static uint32_t CRC_FeedData(hal_crc_handle_t *hcrc, const uint8_t *p_data, uint32_t size_byte);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
static hal_status_t CRC_CheckPolynomial(uint32_t poly_coefficient, hal_crc_polynomial_size_t poly_size);
#endif /* USE_HAL_CHECK_PARAM */

static void CRC_ResetConfig(hal_crc_handle_t *hcrc);
/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRC_Exported_Functions
  * @{
  */

/** @addtogroup CRC_Exported_Functions_Group1
  * @{
This subsection provides functions to initialize and deinitialize the CRC peripheral: \n

- The HAL_CRC_Init() API: Allows initializing the HAL CRC driver so it can be configured and used to calculate the CRC
  of a given user data buffer.
  This API is the first API to call when using the HAL CRC. It takes two parameters as input:
 - The HAL CRC handle         : A pointer to a @ref hal_crc_handle_t structure
 - The CRC peripheral instance: A value from the enumeration @ref hal_crc_t  \n \n

- The HAL_CRC_DeInit() API: Allows deinitializing the HAL CRC driver by resetting:
   - The global CRC configuration to the default one (default register values)
   - The independent data register to the default value
   - The handle global state to the **HAL_CRC_STATE_RESET** \n \n
  */

/**
  * @brief  Initialize the CRC according to the associated instance.
  * @param  hcrc              Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                           CRC HAL context
  * @param  instance          @ref hal_crc_t enumerated type variable to be set according to the physical instance \n
  * @note   The user can choose to activate the CRC clock within the HAL_CRC_Init() function by setting
  *         the <b> USE_HAL_CRC_CLK_ENABLE_MODEL </b> flag to **HAL_CLK_ENABLE_PERIPH_ONLY**
  *         in the configuration file **stm32c5xx_hal_conf.h**
  *         \n
  * @retval HAL_INVALID_PARAM Invalid parameter return when the CRC handle is NULL
  * @retval HAL_OK            The HAL CRC driver is initialized according to the given handle and instance \n
  */
hal_status_t HAL_CRC_Init(hal_crc_handle_t *hcrc, hal_crc_t instance)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(IS_CRC_ALL_INSTANCE((CRC_TypeDef *)(uint32_t)instance));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hcrc == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcrc->instance = instance;

#if defined(USE_HAL_CRC_USER_DATA) && (USE_HAL_CRC_USER_DATA == 1)
  hcrc->p_user_data = NULL;
#endif /* (USE_HAL_CRC_USER_DATA) */

#if defined(USE_HAL_CRC_CLK_ENABLE_MODEL) && (USE_HAL_CRC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_CRC_EnableClock();
#endif /* USE_HAL_CRC_CLK_ENABLE_MODEL */

  hcrc->global_state = HAL_CRC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the CRC peripheral.
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified CRC HAL
  *              context
  */
void HAL_CRC_DeInit(hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(IS_CRC_ALL_INSTANCE(CRC_GET_INSTANCE(hcrc)));

  CRC_ResetConfig(hcrc);

  LL_CRC_WriteIDR(CRC_GET_INSTANCE(hcrc), 0x00000000U);

  hcrc->global_state = HAL_CRC_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup CRC_Exported_Functions_Group2
  * @{
This subsection provides a set of functions to configure the CRC peripheral:

There are two categories of HAL configuration APIs.

- Global configuration APIs:
  - HAL_CRC_SetConfig()          : Used to apply the user configuration @ref hal_crc_config_t structure to the CRC
                                   peripheral
  - HAL_CRC_GetConfig()          : Used to retrieve the CRC configuration and to fill it into @ref hal_crc_config_t
                                   structure
  - HAL_CRC_ResetConfig()        : Used to restore the default CRC configuration(default registers values)
  - HAL_CRC_SetConfigPolynomial(): Used to configure the polynomial coefficient, size and the CRC Init value \n \n

- Unitary configuration APIs: \n
  These APIs are intended to dynamically modify or retrieve a unitary item, meaning that a global configuration has
  already been applied \n
  Do not handle items that can alter other configuration parameters within unitary APIs

  - HAL_CRC_SetInputReverseMode() : Used to set the @ref hal_crc_input_data_reverse_mode_t CRC input reverse mode
  - HAL_CRC_GetInputReverseMode() : Used to retrieve the @ref hal_crc_input_data_reverse_mode_t CRC input reverse mode
  - HAL_CRC_SetOutputReverseMode(): Used to set the @ref hal_crc_output_data_reverse_mode_t CRC output reverse mode
  - HAL_CRC_GetOutputReverseMode(): Used to retrieve the @ref hal_crc_output_data_reverse_mode_t CRC output reverse mode
  - HAL_CRC_SetIndependentData()  : Used to store user data in the CRC independent register
  - HAL_CRC_GetIndependentData()  : Used to retrieve the stored user data from the CRC independent register


Each configuration API must first check the IDLE state (meaning HAL_CRC_Init() has been performed)
    \n
  */

/**
  * @brief  Configure the CRC according to the user parameters.
  * @param  hcrc               Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                            CRC HAL context
  * @param  p_config           Pointer to a @ref hal_crc_config_t structure that contains the CRC configuration
  * @warning The Polynomial size must be aligned to the configured output reverse mode (ex when output reverse mode is
  *          set to **HAL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD**, the provided Polynomial size must be a word)
  * @retval HAL_INVALID_PARAM  Invalid parameter return when:
  *                            - The configuration structure pointer is NULL
  *                            - Or the provided polynomial is invalid (Even polynomial or polynomial size and
                                 coefficient are incoherent)
  * @retval HAL_OK             CRC peripheral is correctly configured
  */
hal_status_t HAL_CRC_SetConfig(hal_crc_handle_t *hcrc, const hal_crc_config_t *p_config)
{
  CRC_TypeDef *p_crcx;

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_PARAM(IS_CRC_POL_SIZE(p_config->polynomial_size));

  ASSERT_DBG_PARAM(IS_CRC_INPUTDATA_REVERSE_MODE(p_config->input_data_reverse_mode));

  ASSERT_DBG_PARAM(IS_CRC_OUTPUTDATA_REVERSE_MODE(p_config->output_data_reverse_mode));

  ASSERT_DBG_PARAM(IS_CRC_OUTPUT_REVERSE_MODE_VALID(p_config->polynomial_size, p_config->output_data_reverse_mode));

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_config == NULL)
      || (CRC_CheckPolynomial(p_config->polynomial_coefficient, p_config->polynomial_size) != HAL_OK))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_crcx = CRC_GET_INSTANCE(hcrc);

  LL_CRC_SetPolynomialCoef(p_crcx, p_config->polynomial_coefficient);

  LL_CRC_SetPolynomialSize(p_crcx, (uint32_t)p_config->polynomial_size);

  LL_CRC_SetInitialData(p_crcx, p_config->crc_init_value);

  LL_CRC_SetDataReverseMode(p_crcx, (uint32_t) p_config->input_data_reverse_mode,
                            (uint32_t) p_config->output_data_reverse_mode);
  return HAL_OK;
}

/**
  * @brief  Retrieve the CRC configuration.
  * @param  hcrc     Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified CRC HAL
  *                  context
  * @param  p_config Pointer to a @ref hal_crc_config_t retrieved structure configuration
  */
void HAL_CRC_GetConfig(const hal_crc_handle_t *hcrc, hal_crc_config_t *p_config)
{
  CRC_TypeDef *p_crcx;

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  p_crcx = CRC_GET_INSTANCE(hcrc);

  p_config->polynomial_size = (hal_crc_polynomial_size_t)LL_CRC_GetPolynomialSize(p_crcx);

  p_config->polynomial_coefficient = LL_CRC_GetPolynomialCoef(p_crcx);

  p_config->crc_init_value = LL_CRC_GetInitialData(p_crcx);

  p_config->input_data_reverse_mode = (hal_crc_input_data_reverse_mode_t)LL_CRC_GetInputDataReverseMode(p_crcx);

  p_config->output_data_reverse_mode = (hal_crc_output_data_reverse_mode_t)LL_CRC_GetOutputDataReverseMode(p_crcx);
}

/**
  * @brief  This function allows resetting a set of fields to their default values.
  *         - The polynomial coefficient is set to 0x04C11DB7U
  *         - The polynomial size is set to 32-bits
  *         - The CRC init value is set to 0xFFFFFFFFU
  *         - The input reversibility mode is set to none
  *         - The output reversibility mode is set to none
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure  that is the object maintaining the specified CRC HAL
  *              context
  */
void HAL_CRC_ResetConfig(hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  CRC_ResetConfig(hcrc);
}

/**
  * @brief  Configure the CRC polynomial (polynomial size, polynomial coefficient, CRC init value).
  * @param  hcrc              Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                           CRC HAL context
  * @param  poly_coefficient  A **uint32_t** CRC Polynomial coefficient that must be odd and coherent with the
  *                           **poly_size**
  * @param  poly_size         CRC Polynomial size with a **hal_crc_polynomial_size_t** type and a value within:
  *                           - HAL_CRC_POLY_SIZE_32B
  *                           - HAL_CRC_POLY_SIZE_16B
  *                           - HAL_CRC_POLY_SIZE_8B
  *                           - HAL_CRC_POLY_SIZE_7B
  * @param  crc_init_value    A **uint32_t** initial CRC value
  * @retval HAL_INVALID_PARAM Invalid parameter return when the provided polynomial is invalid
  *                           (Even polynomial or polynomial size and coefficient are incoherent
                               or polynomial size and output reverse mode are incoherent)
  * @retval HAL_OK            CRC polynomial is correctly configured
  */
hal_status_t HAL_CRC_SetConfigPolynomial(hal_crc_handle_t *hcrc, uint32_t poly_coefficient,
                                         hal_crc_polynomial_size_t poly_size, uint32_t crc_init_value)
{
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  uint32_t out_rev_mode;
#endif /* USE_HAL_CHECK_PARAM */

  CRC_TypeDef *p_crcx;

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(IS_CRC_POL_SIZE(poly_size));

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  out_rev_mode = LL_CRC_GetOutputDataReverseMode(CRC_GET_INSTANCE(hcrc));

  if ((poly_size != HAL_CRC_POLY_SIZE_32B)
      && ((out_rev_mode == LL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD)
          || (out_rev_mode == LL_CRC_OUTDATA_REVERSE_BYTE_BYWORD)))
  {
    return HAL_INVALID_PARAM;
  }

  if (CRC_CheckPolynomial(poly_coefficient, poly_size) != HAL_OK)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_crcx = CRC_GET_INSTANCE(hcrc);

  LL_CRC_SetPolynomialCoef(p_crcx, poly_coefficient);

  LL_CRC_SetPolynomialSize(p_crcx, (uint32_t)poly_size);

  LL_CRC_SetInitialData(p_crcx, crc_init_value);

  return HAL_OK;
}

/**
  * @brief  Configure the CRC Input reversibility mode.
  * @param  hcrc               Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                            CRC HAL context
  * @param  input_reverse_mode CRC input reversibility mode with a **hal_crc_input_data_reverse_mode_t** type and a
                               value within: \n
  *                            - HAL_CRC_INDATA_REVERSE_NONE
  *                            - HAL_CRC_INDATA_REVERSE_BYTE
  *                            - HAL_CRC_INDATA_REVERSE_HALFWORD
  *                            - HAL_CRC_INDATA_REVERSE_WORD
  *                            - HAL_CRC_INDATA_REVERSE_HALFWORD_BYWORD
  *                            - HAL_CRC_INDATA_REVERSE_BYTE_BYWORD
  * @retval HAL_OK             CRC input reverse mode is correctly configured
  */
hal_status_t HAL_CRC_SetInputReverseMode(hal_crc_handle_t *hcrc, hal_crc_input_data_reverse_mode_t input_reverse_mode)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(IS_CRC_INPUTDATA_REVERSE_MODE(input_reverse_mode));

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  LL_CRC_SetInputDataReverseMode(CRC_GET_INSTANCE(hcrc), (uint32_t)input_reverse_mode);

  return HAL_OK;
}

/**
  * @brief  Retrieve the CRC configured input reversibility mode.
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified CRC HAL
  *              context
  * @retval HAL_CRC_INDATA_REVERSE_NONE             The bit order of the input data is not affected
  * @retval HAL_CRC_INDATA_REVERSE_BYTE             The bit reversal is done by byte. Example: 0x1A2B3C4D becomes
  *                                                 0x58D43CB2
  * @retval HAL_CRC_INDATA_REVERSE_HALFWORD         The bit reversal is done by half word
  *                                                 Ex: 0x1A2B3C4D becomes 0xD458B23C
  * @retval HAL_CRC_INDATA_REVERSE_WORD             The bit-reversal is done on the full word
  *                                                 Ex: 0x1A2B3C4D becomes 0xB23CD458
  * @retval HAL_CRC_INDATA_REVERSE_HALFWORD_BYWORD  The bit reversal is done by half word by word
  *                                                 Ex: 0x1A2B3C4D becomes 0x3C4D1A2B
  * @retval HAL_CRC_INDATA_REVERSE_BYTE_BYWORD      The bit reversal is done by byte by word
  *                                                 Ex: 0x1A2B3C4D becomes 0x4D3C2B1A
  */
hal_crc_input_data_reverse_mode_t HAL_CRC_GetInputReverseMode(const hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  return ((hal_crc_input_data_reverse_mode_t)LL_CRC_GetInputDataReverseMode(CRC_GET_INSTANCE(hcrc)));
}

/**
  * @brief  Configure the CRC Output reversibility mode.
  * @param  hcrc                Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the
  *                             specified CRC HAL context
  * @param  output_reverse_mode CRC output reversibility mode with a **hal_crc_output_data_reverse_mode_t** type and a
                                value within: \n
  *                             - HAL_CRC_OUTDATA_REVERSE_NONE
  *                             - HAL_CRC_OUTDATA_REVERSE_BIT
  *                             - HAL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD
  *                             - HAL_CRC_OUTDATA_REVERSE_BYTE_BYWORD
  * @retval HAL_INVALID_PARAM Invalid parameter return when the provided output reverse mode is invalid
  *                           (output reverse mode and polynomial size are incoherent)
  * @retval HAL_OK              CRC output reverse mode is correctly configured
  */
hal_status_t HAL_CRC_SetOutputReverseMode(hal_crc_handle_t *hcrc,
                                          hal_crc_output_data_reverse_mode_t output_reverse_mode)
{
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  uint32_t poly_size;
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(IS_CRC_OUTPUTDATA_REVERSE_MODE(output_reverse_mode));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  poly_size = LL_CRC_GetPolynomialSize(CRC_GET_INSTANCE(hcrc));

  if (((output_reverse_mode == HAL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD)
       || (output_reverse_mode == HAL_CRC_OUTDATA_REVERSE_BYTE_BYWORD))
      && (poly_size != LL_CRC_POLY_SIZE_32B))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  LL_CRC_SetOutputDataReverseMode(CRC_GET_INSTANCE(hcrc), (uint32_t)output_reverse_mode);

  return HAL_OK;
}

/**
  * @brief  Retrieve the CRC configured output reversibility mode.
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified CRC HAL
  *              context
  * @retval HAL_CRC_OUTDATA_REVERSE_NONE           The bit order of the output data is not affected
  * @retval HAL_CRC_OUTDATA_REVERSE_BIT            The bit-reversal of the output data is done by byte
  * @retval HAL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD The reversibility is done half word by word
  * @retval HAL_CRC_OUTDATA_REVERSE_BYTE_BYWORD     The reversibility is done byte by word
  */
hal_crc_output_data_reverse_mode_t HAL_CRC_GetOutputReverseMode(const hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  return ((hal_crc_output_data_reverse_mode_t)LL_CRC_GetOutputDataReverseMode(CRC_GET_INSTANCE(hcrc)));
}

/**
  * @brief  Store user data in the Independent Data register.
  * @param  hcrc             Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                          CRC HAL context
  * @param  independent_data A **uint32_t** user data to be stored in the Independent data register
  * @retval HAL_OK           User independent data is correctly configured
  */
hal_status_t HAL_CRC_SetIndependentData(hal_crc_handle_t *hcrc, uint32_t independent_data)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  LL_CRC_WriteIDR(CRC_GET_INSTANCE(hcrc), independent_data);

  return HAL_OK;
}

/**
  * @brief  Return the data stored in the Independent Data register.
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified CRC HAL
  *              context
  * @retval A **uint32_t** retrieved user data from the Independent data register
  */
uint32_t HAL_CRC_GetIndependentData(const hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

  return (LL_CRC_ReadIDR(CRC_GET_INSTANCE(hcrc)));
}
/**
  * @}
  */

/** @addtogroup CRC_Exported_Functions_Group3
  * @{
This subsection provides two CRC calculation functions:

- HAL_CRC_Calculate() API:
This function allows the user to calculate the CRC of an input data buffer starting with the configured CRC init value

- HAL_CRC_Accumulate() API:
This function allows the user to calculate the CRC of an input data buffer starting with the previously computed CRC as
the initialization value \n \n
  */

/**
  * @brief  Compute the 7, 8, 16, or 32-bit CRC value of a user data buffer starting with hcrc->Instance->INIT as
  *         initialization value.
  * @param   hcrc             Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                           CRC HAL context
  * @param   p_data           Pointer to **const void** data buffer provided by the user (buffer of bytes, halfwords or
                              words)
  * @param   size_byte        A **uint32_t** input data buffer length (number of bytes)
  * @param   p_crc_result     A **uint32_t** Calculated CRC with a size aligned with the used polynomial one
  * @warning The data size must be aligned to the configured input reverse mode (ex when input reverse mode is set to
  *          **HAL_CRC_INDATA_REVERSE_WORD**, the provided data size must be a multiple of words)
  * @retval  HAL_INVALID_PARAM Invalid param return when the provided data buffer pointer is null or when this buffer is
  *                            empty
  * @retval  HAL_BUSY          Another calculation process is ongoing
  * @retval  HAL_OK            The CRC is successfully calculated
  */
hal_status_t HAL_CRC_Calculate(hal_crc_handle_t *hcrc, const void *p_data, uint32_t size_byte, uint32_t *p_crc_result)
{
  const uint8_t *p_tmp_data = (const uint8_t *)p_data;

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_PARAM(IS_CRC_DATA_SIZE_VALID(hcrc, size_byte));

  ASSERT_DBG_PARAM(p_crc_result != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (p_crc_result == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcrc, global_state, HAL_CRC_STATE_IDLE, HAL_CRC_STATE_ACTIVE);

  LL_CRC_ResetCRCCalculationUnit(CRC_GET_INSTANCE(hcrc));

  /* Feed the CRC peripheral with the user data and get the CRC result */
  *p_crc_result = CRC_FeedData(hcrc, p_tmp_data, size_byte);

  hcrc->global_state = HAL_CRC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Compute the 7, 8, 16, or 32-bit CRC value of a user data buffer starting with the previously computed CRC as
  *         the initialization value.
  * @param   hcrc             Pointer to a @ref hal_crc_handle_t structure that is the object maintaining the specified
  *                           CRC HAL context
  * @param   p_data           Pointer to **const void** data buffer provided by the user (buffer of bytes, halfwords or
                              words)
  * @param   size_byte        A **uint32_t** input data buffer length (number of bytes)
  * @param   p_crc_result     A **uint32_t** Calculated CRC with a size aligned with the used polynomial one
  * @warning The data size must be aligned to the configured input reverse mode (ex when input reverse mode is set to
  *          **HAL_CRC_INDATA_REVERSE_WORD**, the provided data size must be a multiple of words)
  * @retval  HAL_INVALID_PARAM Invalid param return when the provided data buffer pointer is null or when this buffer is
  *                            empty
  * @retval  HAL_BUSY          Another calculation process is ongoing
  * @retval  HAL_OK            The CRC is successfully calculated
  * @note    The HAL_CRC_Accumulate() function must not be applied when the input reverse mode's granularity is higher
  *          than the number of bytes already calculated
  *          \n
  */
hal_status_t HAL_CRC_Accumulate(hal_crc_handle_t *hcrc, const void *p_data, uint32_t size_byte, uint32_t *p_crc_result)
{
  const uint8_t *p_tmp_data = (const uint8_t *)p_data;

  ASSERT_DBG_PARAM(hcrc != NULL);

  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_PARAM(IS_CRC_DATA_SIZE_VALID(hcrc, size_byte));

  ASSERT_DBG_PARAM(p_crc_result != NULL);

  ASSERT_DBG_STATE(hcrc->global_state, HAL_CRC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U) || (p_crc_result == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcrc, global_state, HAL_CRC_STATE_IDLE, HAL_CRC_STATE_ACTIVE);

  /* Feed the CRC peripheral with the user data and get the CRC result */
  *p_crc_result = CRC_FeedData(hcrc, p_tmp_data, size_byte);

  hcrc->global_state = HAL_CRC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @}
  */


/** @addtogroup CRC_Exported_Functions_Group4
  * @{
This subsection provides a HAL_CRC_GetState() function to retrieve the CRC peripheral global state
  */


/**
  * @brief  Retrieve the HAL CRC Global State.
  * @param  hcrc                 Pointer to a @ref hal_crc_handle_t structure  that is the object maintaining the
  *                              specified CRC HAL context
  * @retval HAL_CRC_STATE_RESET  CRC peripheral is de-initialized
  * @retval HAL_CRC_STATE_IDLE   CRC peripheral is initialized and configured
  * @retval HAL_CRC_STATE_ACTIVE CRC calculation is ongoing
  */
hal_crc_state_t HAL_CRC_GetState(const hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  return hcrc->global_state;
}
/**
  * @}
  */

#if defined (USE_HAL_CRC_USER_DATA) && (USE_HAL_CRC_USER_DATA == 1)
/** @addtogroup CRC_Exported_Functions_Group5
  * @{
This subsection provides a set of functions to get and set user data:
  - HAL_CRC_SetUserData() : Used to store the application user data pointer into the handle
  - HAL_CRC_GetUserData() : Used to retrieve the application user data pointer from the handle
  */
/**
  * @brief  Store application user data pointer into the handle.
  * @param  hcrc        Pointer to a @ref hal_crc_handle_t structure
  * @param  p_user_data Pointer to the user data
  */
void HAL_CRC_SetUserData(hal_crc_handle_t *hcrc, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  hcrc->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the application user data pointer from the handle.
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure
  * @retval Pointer to the user data
  */
const void *HAL_CRC_GetUserData(const hal_crc_handle_t *hcrc)
{
  ASSERT_DBG_PARAM(hcrc != NULL);

  return (hcrc->p_user_data);
}
/**
  * @}
  */
#endif /* USE_HAL_CRC_USER_DATA == 1 */

/**
  * @}
  */

/** @addtogroup CRC_Private_Functions
  * @{
  */
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)

/**
  * @brief  Check the validity of the CRC polynomial.
  *         - The polynomial coefficient must be odd (1+X+X^2+..+X^n)
  *         - The polynomial size and coefficient must be coherent
  * @param  poly_coefficient  A **uint32_t** CRC Polynomial coefficient that must be odd and coherent with the
  *                           **poly_size**
  * @param  poly_size         CRC Polynomial size with a **hal_crc_polynomial_size_t** type and a value within:
  *                           - HAL_CRC_POLY_SIZE_32B
  *                           - HAL_CRC_POLY_SIZE_16B
  *                           - HAL_CRC_POLY_SIZE_8B
  *                           - HAL_CRC_POLY_SIZE_7B
  * @retval HAL_OK            CRC Polynomial has been correctly configured
  * @retval HAL_INVALID_PARAM Invalid parameter return when the polynomial is even or when its size and coefficient
  *                           are incoherent \n
  * @note   The even polynomials (X+X^2+..+X^n) are not supported by the CRC peripheral
  *         \n
  */
static hal_status_t CRC_CheckPolynomial(uint32_t poly_coefficient, hal_crc_polynomial_size_t poly_size)
{
  hal_status_t status = HAL_OK;
  uint32_t msb = 31U; /* Polynomial degree is 32 at most, so msb is initialized to max value */

  /* Ensure that the generating polynomial is odd */
  if ((poly_coefficient % 2U) == 0U)
  {
    status =   HAL_INVALID_PARAM;
  }
  else
  {
    /** Check polynomial degree vs polynomial size:
      * Polynomial size must be aligned with polynomial degree.
      * HAL_INVALID_PARAM is reported if polynomial degree is larger than that indicated by polynomial size.
      * Look for the MSB position: msb contains the degree of the second-largest polynomial member.
      * E.g., for X^7 + X^6 + X^5 + X^2 + 1, msb = 6
      */
    while ((poly_coefficient & ((uint32_t)(0x1U) << (msb & 0x1FU))) == 0U)
    {
      msb--;
    }

    switch (poly_size)
    {
      case HAL_CRC_POLY_SIZE_7B:
        if (msb >= CRC_POLYSIZE_7B)
        {
          status =   HAL_INVALID_PARAM;
        }
        break;
      case HAL_CRC_POLY_SIZE_8B:
        if (msb >= CRC_POLYSIZE_8B)
        {
          status =   HAL_INVALID_PARAM;
        }
        break;
      case HAL_CRC_POLY_SIZE_16B:
        if (msb >= CRC_POLYSIZE_16B)
        {
          status =   HAL_INVALID_PARAM;
        }
        break;

      case HAL_CRC_POLY_SIZE_32B:
        /* No possible coherency issue between the polynomial coefficient and size */
        break;
      default:
        status =  HAL_INVALID_PARAM;
        break;
    }
  }

  return status;
}
#endif /* USE_HAL_CHECK_PARAM */

/**
  * @brief  Feed the CRC peripheral with the user buffer and return the calculated CRC value.
  *         Optimize the CRC data access according to the input data alignment and size.
  * @param  hcrc      Pointer to a @ref hal_crc_handle_t structure  that is the object maintaining the specified CRC
  *                   HAL context
  * @param  p_data    Pointer to the 8-bit input data buffer
  * @param  size_byte Input data buffer length (number of bytes)
  * @retval uint32_t  Calculated CRC with a size aligned with the used polynomial one
  */
static uint32_t CRC_FeedData(hal_crc_handle_t *hcrc, const uint8_t *p_data, uint32_t size_byte)
{
  const uint8_t *p_tmp_data = (const uint8_t *)p_data;
  uint32_t non_aligned_bytes = 0;
  uint32_t i;
  uint32_t last_bytes;
  CRC_TypeDef *p_crcx;

  p_crcx = CRC_GET_INSTANCE(hcrc);

  /* Alignment approach is not used to feed data register when the input reverse mode is other than none */
  if (((hal_crc_input_data_reverse_mode_t)LL_CRC_GetInputDataReverseMode(p_crcx)
       != HAL_CRC_INDATA_REVERSE_NONE) && (((uint32_t)p_tmp_data & 3U) != 0U))
  {
    for (i = 0U; i < (size_byte / 4U); i++)
    {
      LL_CRC_FeedData32(p_crcx, ((uint32_t)p_tmp_data[4U * i] << 24U) |
                        ((uint32_t)p_tmp_data[(4U * i) + 1U] << 16U) |
                        ((uint32_t)p_tmp_data[(4U * i) + 2U] << 8U)  |
                        (uint32_t)p_tmp_data[(4U * i) + 3U]);
    }
    p_tmp_data += i * 4U;
  }
  else
  {
    /* Read non aligned 32 bits address: address can be multiple of 1 or 2 or 3 */
    /* First address alignment if needed */
    if (size_byte >= 4U)
    {
      if (((uint32_t)p_data & 1U) != 0U)
      {
        LL_CRC_FeedData8(p_crcx, *p_tmp_data);
        non_aligned_bytes = 1U;
        p_tmp_data++;
      }

      /* Second address alignment if needed */
      if (((uint32_t)p_tmp_data & 3U) != 0U)
      {
        LL_CRC_FeedData16(p_crcx, (uint16_t)__REV16(*(const uint16_t *)p_tmp_data));
        non_aligned_bytes += 2U;
        p_tmp_data += 2U;
      }

      /* Enter 32-bit input data to the CRC calculator */
      for (i = 0; i < ((size_byte - non_aligned_bytes) / 4U); i++)
      {
        LL_CRC_FeedData32(p_crcx, __REV(*(const uint32_t *)p_tmp_data));
        p_tmp_data += 4U;
      }
    }
  }

  /* Last bytes specific handling */
  last_bytes = (size_byte - non_aligned_bytes) & 3U;
  if (last_bytes != 0U)
  {
    if (last_bytes == 1U)
    {
      LL_CRC_FeedData8(p_crcx, *p_tmp_data);
    }
    else if (last_bytes == 2U)
    {
      LL_CRC_FeedData16(p_crcx, ((uint16_t)(*p_tmp_data) << 8U) | (uint16_t) * (p_tmp_data + 1));
    }
    else
    {
      LL_CRC_FeedData16(p_crcx, ((uint16_t)(*p_tmp_data) << 8U) | (uint16_t) * (p_tmp_data + 1));
      LL_CRC_FeedData8(p_crcx, *(p_tmp_data + 2));
    }
  }

  /* Return the CRC computed value */
  return (p_crcx->DR);
}

/**
  * @brief  This function allows resetting the CRC configuration fields to their default values.
  *         - The polynomial coefficient is set to 0x04C11DB7U
  *         - The polynomial size is set to 32-bits
  *         - The CRC init value is set to 0xFFFFFFFFU
  *         - The input reversibility mode is set to none
  *         - The output reversibility mode is set to none
  * @param  hcrc Pointer to a @ref hal_crc_handle_t structure  that is the object maintaining the specified CRC HAL
  *              context
  */
static void CRC_ResetConfig(hal_crc_handle_t *hcrc)
{
  CRC_TypeDef *p_crcx;

  p_crcx = CRC_GET_INSTANCE(hcrc);

  LL_CRC_SetPolynomialCoef(p_crcx, LL_CRC_DEFAULT_CRC32_POLY);

  LL_CRC_SetPolynomialSize(p_crcx, LL_CRC_POLY_SIZE_32B);

  LL_CRC_SetInitialData(p_crcx, LL_CRC_DEFAULT_CRC_INITVALUE);

  LL_CRC_SetDataReverseMode(p_crcx, LL_CRC_INDATA_REVERSE_NONE, LL_CRC_OUTDATA_REVERSE_NONE);
}

/**
  * @}
  */

#endif /* USE_HAL_CRC_MODULE */
/**
  * @}
  */

/**
  * @}
  */
