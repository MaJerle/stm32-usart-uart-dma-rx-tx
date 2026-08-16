/**
  *********************************************************************************************************************
  * @file    stm32c5xx_hal_exti.c
  * @brief   EXTI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionality of the General Purpose Input/Output (EXTI) peripheral:
  *           + Initialization and de-initialization functions
  *           + I/O operation functions
  *
  *********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *********************************************************************************************************************
  */

/* Includes ---------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (EXTI)
#if defined (USE_HAL_EXTI_MODULE) && (USE_HAL_EXTI_MODULE == 1)

/** @addtogroup EXTI
  * @{
  */

/** @defgroup EXTI_Introduction EXTI Introduction
  * @{

   The EXTI hardware abstraction layer provides a set of APIs to interface with the EXTI peripheral
   on STM32 microcontrollers.

   The EXTI peripheral manages each line (configurable and direct) using a state machine per line.
   The EXTI includes the following features:

   - Initialize a line.
   - Configure a line.
   - Enable a line in a specific mode (interrupt or event).
   - Force a software interrupt on a specific line.
   - Manage interrupts for a specific line.

  This abstraction layer guarantees portability and ease of use across different STM32 series.
  */

/**
  * @}
  */
/** @defgroup EXTI_How_To_Use EXTI How To Use
  * @{

# How to use the EXTI HAL module driver

## The EXTI HAL driver can be used as follows:

## When a configurable EXTI line is used as an event input:
  - Instantiate an EXTI handle and associate it to a configurable EXTI line using HAL_EXTI_Init().
  - Configure the configurable EXTI line identified by the given handle using HAL_EXTI_SetConfig().
  - Register a user-defined callback for the configurable EXTI line identified by the handle
  using HAL_EXTI_RegisterTriggerCallback(), or use the default callback function HAL_EXTI_TriggerCallback().
  - Enable the configurable EXTI line for interrupt, event, or both modes using HAL_EXTI_Enable().
  - Retrieve pending events for the configurable EXTI line using HAL_EXTI_GetPending() and clear them
  using HAL_EXTI_ClearPending().
  - Disable EXTI modes using HAL_EXTI_Disable().

## When a direct EXTI line is used as an event input:
  - Instantiate an EXTI handle and associate it to a direct EXTI line using HAL_EXTI_Init().
  - Enable the direct EXTI line for interrupt, event, or both modes using HAL_EXTI_Enable().
  - Disable EXTI modes using HAL_EXTI_Disable().
## When generating a software interrupt on an EXTI line:
  - Instantiate an EXTI handle and associate it to an EXTI line using HAL_EXTI_Init().
  - Register a user-defined callback for an EXTI line identified by the handle
  using HAL_EXTI_RegisterTriggerCallback(), or use the default callback function HAL_EXTI_TriggerCallback()
  - Generate the software interrupt using HAL_EXTI_GenerateSWI().
  - Retrieve pending software interrupt events using HAL_EXTI_GetPending() and clear them
  using HAL_EXTI_ClearPending().

## The HAL EXTI driver allows management of EXTI security attributes:
  - Set the privilege access level attribute for an EXTI line using HAL_EXTI_SetPrivAttr().
  - Get the privilege access level attribute for an EXTI line using HAL_EXTI_GetPrivAttr().

## The HAL EXTI driver allows to get the state and instance of lines:
  - Get the current general state of the EXTI line using HAL_EXTI_GetState().
  - Get the HAL EXTI line using HAL_EXTI_GetInstance().
  - Get the LL EXTI line using HAL_EXTI_GetLLInstance().
  */

/**
  * @}
  */

/** @defgroup EXTI_Configuration_Table EXTI Configuration Table
  * @{
# Configuration inside the EXTI driver

Config defines                  | Description     | Default value | Note
--------------------------------|-----------------|---------------|------
USE_ASSERT_DBG_PARAM            | from IDE        | None          | Enables parameter assertions when defined
USE_ASSERT_DBG_STATE            | from IDE        | None          | Enables state assertions when defined
USE_HAL_CHECK_PARAM             | from hal_conf.h | 0             | Parameters such as pointers are checked at runtime
USE_HAL_EXTI_MODULE             | from hal_conf.h | 1             | If defined, stm32c5xx_hal_exti.h is included
USE_HAL_EXTI_REGISTER_CALLBACKS | from hal_conf.h | 0             | If defined, EXTI register callbacks are enabled
USE_HAL_EXTI_USER_DATA          | from hal_conf.h | 0             | If defined, allows the use of user data
CMSE_SECURE_EXECUTION_ENVIRONMENT| from stm32tnxx.h |        NA       | Defined in secure context
  */
/**
  * @}
  */

/* Private constants ------------------------------------------------------------------------------------------------*/
/** @defgroup EXTI_Private_Constants EXTI Private Constants
  * @{
  */

/**
  * @brief  EXTI Mask for GPIO PIN.
  */
#define EXTI_PIN_MASK (LL_EXTI_PIN_MASK | LL_EXTI_CR4)
/**
  * @}
  */

/* Private macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup EXTI_Private_Macros EXTI Private Macros
  * @{
  */
/*! Check if GPIO line or configurable line or direct line and check line number is within range */
#define IS_EXTI_LINE(line) \
  ((((line) & ~(LL_EXTI_PROPERTY_MASK | LL_EXTI_PIN_MASK | LL_EXTI_REG_MASK | LL_EXTI_CR4)) == 0x00U)  \
   && ((((line) & LL_EXTI_PROPERTY_MASK) == LL_EXTI_CONFIG) \
       || (((line) & LL_EXTI_PROPERTY_MASK) == LL_EXTI_GPIO) \
       || (((line) & LL_EXTI_PROPERTY_MASK) == LL_EXTI_DIRECT)) \
   && (((line) & LL_EXTI_PIN_MASK) < LL_EXTI_LINE_NB))

/*! Macro to check EXTI mode */
#define IS_EXTI_MODE(mode) (((mode) == HAL_EXTI_MODE_INTERRUPT) \
                            || ((mode) == HAL_EXTI_MODE_EVENT) \
                            || ((mode) == HAL_EXTI_MODE_INTERRUPT_EVENT))

/*! Macro to check EXTI trigger */
#define IS_EXTI_TRIGGER(trigger) (((trigger) == HAL_EXTI_TRIGGER_NONE) \
                                  || ((trigger) == HAL_EXTI_TRIGGER_RISING) \
                                  || ((trigger) == HAL_EXTI_TRIGGER_FALLING) \
                                  || ((trigger) == HAL_EXTI_TRIGGER_RISING_FALLING))

/*! Macro to check EXTI pending edge */
#define IS_EXTI_PENDING_EDGE(pending_edge)   (((pending_edge) == HAL_EXTI_TRIGGER_RISING) \
                                              || ((pending_edge) == HAL_EXTI_TRIGGER_FALLING) \
                                              || ((pending_edge) == HAL_EXTI_TRIGGER_RISING_FALLING))

/*! Macro to check EXTI GPIO port */
#if defined(GPIOF)
#define IS_EXTI_GPIO_PORT(port)     (((port) == HAL_EXTI_GPIOA) \
                                     || ((port) == HAL_EXTI_GPIOB) \
                                     || ((port) == HAL_EXTI_GPIOC) \
                                     || ((port) == HAL_EXTI_GPIOD) \
                                     || ((port) == HAL_EXTI_GPIOE) \
                                     || ((port) == HAL_EXTI_GPIOF) \
                                     || ((port) == HAL_EXTI_GPIOG) \
                                     || ((port) == HAL_EXTI_GPIOH))
#else
#define IS_EXTI_GPIO_PORT(port)     (((port) == HAL_EXTI_GPIOA) \
                                     || ((port) == HAL_EXTI_GPIOB) \
                                     || ((port) == HAL_EXTI_GPIOC) \
                                     || ((port) == HAL_EXTI_GPIOD) \
                                     || ((port) == HAL_EXTI_GPIOE) \
                                     || ((port) == HAL_EXTI_GPIOH))
#endif /* GPIOI */

/*! Macro to check EXTI configurable line */
#define IS_EXTI_CONFIG_LINE(line)    (((line) & LL_EXTI_CONFIG) == LL_EXTI_CONFIG)

/*! Macro to check the EXTI privilege attributes */
#define IS_EXTI_LINE_PRIV_ATTR(attribute) ((attribute == HAL_EXTI_PRIV) || (attribute == HAL_EXTI_NPRIV))
/**
  * @}
  */

/* Exported functions -----------------------------------------------------------------------------------------------*/
/** @addtogroup EXTI_Exported_Functions
  * @{
  */

/** @addtogroup EXTI_Exported_Functions_Group1
  * @{
  * ## This subsection provides a set of functions to initialize, de-initialize, and configure the EXTI:

### Initialize the EXTI handle using HAL_EXTI_Init():
  - Provide the EXTI handle as a parameter.
  - Provide the EXTI line number as a second parameter from \ref hal_exti_line_t enumeration.

### De-initialize the EXTI and reset the EXTI global state to HAL_EXTI_STATE_RESET by calling HAL_EXTI_DeInit():
  - Provide the EXTI handle as a parameter.

### Configure the EXTI line using HAL_EXTI_SetConfig():
  - Provide the EXTI handle as a parameter.
  - Provide a configuration containing the trigger edge from \ref hal_exti_trigger_t and the gpio_port from
  \ref hal_exti_gpio_port_t.

### Retrieve the current EXTI configuration of a dedicated line using HAL_EXTI_GetConfig():
  - Provide the EXTI handle as the first parameter.
  - Provide a configuration structure used to retrieve the current EXTI line configuration. \n
<br>
  */

/**
  * @brief  Store the EXTI line into the handle.
  * @param hexti Pointer to EXTI handle.
  * @param line EXTI line. This parameter can be one of the values of @ref hal_exti_line_t.
  * @retval HAL_INVALID_PARAM In case of an invalid parameter.
  * @retval HAL_OK In case of a successful initialization.
  */
hal_status_t HAL_EXTI_Init(hal_exti_handle_t *hexti, hal_exti_line_t line)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)line));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hexti == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store EXTI line into handle */
  hexti->line = line;

  /* Compute the corresponding LL EXTI line needed for LL functions */
  hexti->ll_line = (1UL << ((uint32_t)hexti->line & LL_EXTI_PIN_MASK));

#if defined (USE_HAL_EXTI_REGISTER_CALLBACKS) && (USE_HAL_EXTI_REGISTER_CALLBACKS == 1)
  /* Store the predeclared callback functions */
  hexti->p_trigger_cb = HAL_EXTI_TriggerCallback;
#endif /* USE_HAL_EXTI_REGISTER_CALLBACKS */

#if defined(USE_HAL_EXTI_USER_DATA) && (USE_HAL_EXTI_USER_DATA == 1)
  hexti->p_user_data = NULL;
#endif /* USE_HAL_EXTI_USER_DATA */

  /* Check if the selected EXTI line is configurable */
  if ((IS_EXTI_CONFIG_LINE((uint32_t)hexti->line)) == 0U)
  {
    hexti->global_state = HAL_EXTI_STATE_ACTIVE;
  }
  else
  {
    hexti->global_state = HAL_EXTI_STATE_INIT;
  }

  return HAL_OK;
}

/**
  * @brief  De-initialize the EXTI line.
  * @param hexti Pointer to EXTI handle.
  */
void HAL_EXTI_DeInit(hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)hexti->line));

  if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    LL_EXTI_DisableIT_0_31(hexti->ll_line);
    LL_EXTI_DisableEvent_0_31(hexti->ll_line);

    /* Check if the selected EXTI line is a configurable line */
    if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) != 0U)
    {
      LL_EXTI_DisableRisingTrig_0_31(hexti->ll_line);
      LL_EXTI_DisableFallingTrig_0_31(hexti->ll_line);
      LL_EXTI_ClearRisingFlag_0_31(hexti->ll_line);
      LL_EXTI_ClearFallingFlag_0_31(hexti->ll_line);
    }
  }

  else
  {
    LL_EXTI_DisableIT_32_63(hexti->ll_line);
    LL_EXTI_DisableEvent_32_63(hexti->ll_line);

    /* Check if the selected EXTI line is a configurable line */
    if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) != 0U)
    {
      LL_EXTI_DisableRisingTrig_32_63(hexti->ll_line);
      LL_EXTI_DisableFallingTrig_32_63(hexti->ll_line);
      LL_EXTI_ClearRisingFlag_32_63(hexti->ll_line);
      LL_EXTI_ClearFallingFlag_32_63(hexti->ll_line);
    }
  }

  /* Verify if the selected line is a GPIO line */
  if (((uint32_t)hexti->line & LL_EXTI_GPIO) == LL_EXTI_GPIO)
  {
    /* Reset the EXTI source */
    LL_EXTI_SetEXTISource((uint32_t)HAL_EXTI_GPIOA, ((uint32_t)hexti->line & EXTI_PIN_MASK));
  }

  hexti->global_state = HAL_EXTI_STATE_RESET;
}

/**
  * @brief  Set configuration for the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @param p_exti_config Pointer to EXTI configuration structure.
  * @retval HAL_INVALID_PARAM In case of an invalid parameter.
  * @retval HAL_OK In case of a successful configuration.
  * @retval HAL_ERROR In case of selected EXTI line is direct.
  */
hal_status_t HAL_EXTI_SetConfig(hal_exti_handle_t *hexti, const hal_exti_config_t *p_exti_config)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(p_exti_config != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_CONFIG_LINE((uint32_t)hexti->line));
  ASSERT_DBG_PARAM(IS_EXTI_TRIGGER(p_exti_config->trigger));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_INIT | (uint32_t)HAL_EXTI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_exti_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check if selected EXTI line is configurable */
  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) == 0U)
  {
    return HAL_ERROR;
  }

  /* Check whether selected trigger is on rising edge */
  if (((uint32_t)p_exti_config->trigger & (uint32_t)HAL_EXTI_TRIGGER_RISING) != 0U)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_EnableRisingTrig_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_EnableRisingTrig_32_63(hexti->ll_line);
    }
  }
  else
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_DisableRisingTrig_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_DisableRisingTrig_32_63(hexti->ll_line);
    }
  }

  /* Check whether selected trigger is on falling edge */
  if (((uint32_t)p_exti_config->trigger & (uint32_t)HAL_EXTI_TRIGGER_FALLING) != 0U)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_EnableFallingTrig_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_EnableFallingTrig_32_63(hexti->ll_line);
    }
  }
  else
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_DisableFallingTrig_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_DisableFallingTrig_32_63(hexti->ll_line);
    }
  }

  /* Verify if the selected line is a GPIO line */
  if (((uint32_t)hexti->line & LL_EXTI_GPIO) == LL_EXTI_GPIO)
  {
    ASSERT_DBG_PARAM(IS_EXTI_GPIO_PORT(p_exti_config->gpio_port));

    /* Compute the EXTI source register and configure the corresponding GPIO port */
    LL_EXTI_SetEXTISource((uint32_t)p_exti_config->gpio_port, ((uint32_t)hexti->line & EXTI_PIN_MASK));
  }

  hexti->global_state = HAL_EXTI_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the configuration of the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @param p_exti_config Pointer to structure to store EXTI configuration.
  */
void HAL_EXTI_GetConfig(const hal_exti_handle_t *hexti, hal_exti_config_t *p_exti_config)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(p_exti_config != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_CONFIG_LINE((uint32_t)hexti->line));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_IDLE | (uint32_t)HAL_EXTI_STATE_ACTIVE);

  p_exti_config->trigger = HAL_EXTI_TRIGGER_NONE;

  /* Check if the selected EXTI line is configurable */
  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) != 0U)
  {
    /* Check if rising edge trigger is enabled on the selected line */
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      if (LL_EXTI_IsEnabledRisingTrig_0_31(hexti->ll_line) != 0U)
      {
        p_exti_config->trigger = HAL_EXTI_TRIGGER_RISING;
      }
      if (LL_EXTI_IsEnabledFallingTrig_0_31(hexti->ll_line) != 0U)
      {
        if (p_exti_config->trigger == HAL_EXTI_TRIGGER_RISING)
        {
          p_exti_config->trigger = HAL_EXTI_TRIGGER_RISING_FALLING;
        }
        else
        {
          p_exti_config->trigger = HAL_EXTI_TRIGGER_FALLING;
        }
      }
    }
    else
    {
      if (LL_EXTI_IsEnabledRisingTrig_32_63(hexti->ll_line) != 0U)
      {
        p_exti_config->trigger = HAL_EXTI_TRIGGER_RISING;
      }
      if (LL_EXTI_IsEnabledFallingTrig_32_63(hexti->ll_line) != 0U)
      {
        if (p_exti_config->trigger == HAL_EXTI_TRIGGER_RISING)
        {
          p_exti_config->trigger = HAL_EXTI_TRIGGER_RISING_FALLING;
        }
        else
        {
          p_exti_config->trigger = HAL_EXTI_TRIGGER_FALLING;
        }
      }
    }

    /* Check if the selected line is a GPIO line */
    if (((uint32_t)hexti->line & LL_EXTI_GPIO) == LL_EXTI_GPIO)
    {
      /* Compute the EXTI source register and retrieve the actual GPIO port */
      p_exti_config->gpio_port = (hal_exti_gpio_port_t)LL_EXTI_GetEXTISource((uint32_t)hexti->line & EXTI_PIN_MASK);
    }
  }
}

/**
  * @}
  */

/** @addtogroup EXTI_Exported_Functions_Group2
  * @{
  * ## This subsection describes the APIs that manage I/O operations for EXTI lines:

### Enable the EXTI line for the selected mode: Interrupt, Event or both using HAL_EXTI_Enable():
  - Provide the EXTI handle as a first parameter.
  - Provide an EXTI mode from \ref hal_exti_mode_t as a second parameter.

### Disable EXTI line using HAL_EXTI_Disable():
  - Provide the EXTI handle as a parameter.

### Generate a software interrupt using HAL_EXTI_GenerateSWI():
  - Provide the EXTI handle as a parameter.

### Get interrupt pending edge using HAL_EXTI_GetPending():
  - Provide the EXTI handle as a parameter.

### Clear interrupt pending bit for the given edge using HAL_EXTI_ClearPending():
  - Provide the EXTI handle as the first parameter.
  - Provide the pending edge as the second parameter. \n
<br>
  */

/**
  * @brief  Enable the EXTI mode for the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @param mode EXTI mode. This parameter can be one of the values of @ref hal_exti_mode_t.
  * @retval HAL_OK In case of a successful enable.
  */
hal_status_t HAL_EXTI_Enable(hal_exti_handle_t *hexti, hal_exti_mode_t mode)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_MODE(mode));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_IDLE | (uint32_t)HAL_EXTI_STATE_ACTIVE);

  hexti->global_state = HAL_EXTI_STATE_ACTIVE;

  /* Update the previous state to ACTIVE to save actual state throughout ISR */
  hexti->prev_state = HAL_EXTI_STATE_ACTIVE;

  if (((uint32_t)mode & (uint32_t)HAL_EXTI_MODE_INTERRUPT) != 0U)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_EnableIT_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_EnableIT_32_63(hexti->ll_line);
    }
  }

  if (((uint32_t)mode & (uint32_t)HAL_EXTI_MODE_EVENT) != 0U)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_EnableEvent_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_EnableEvent_32_63(hexti->ll_line);
    }
  }

  return HAL_OK;
}

/**
  * @brief  Disable the EXTI mode for the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @retval HAL_OK In case of a successful disable.
  */
hal_status_t HAL_EXTI_Disable(hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);

  ASSERT_DBG_STATE(hexti->global_state, HAL_EXTI_STATE_ACTIVE);

  if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    LL_EXTI_DisableIT_0_31(hexti->ll_line);
    LL_EXTI_DisableEvent_0_31(hexti->ll_line);
  }
  else
  {
    LL_EXTI_DisableIT_32_63(hexti->ll_line);
    LL_EXTI_DisableEvent_32_63(hexti->ll_line);
  }

  hexti->global_state = HAL_EXTI_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Generate a software interrupt for the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @retval HAL_OK When software interrupt is successfully generated.
  * @retval HAL_ERROR In case of selected EXTI line is direct.
  */
hal_status_t HAL_EXTI_GenerateSWI(hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM((IS_EXTI_CONFIG_LINE((uint32_t)hexti->line)));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_INIT | (uint32_t)HAL_EXTI_STATE_IDLE);

  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) == 0U) /* Check if selected EXTI line is configurable */
  {
    return HAL_ERROR;
  }

  hexti->prev_state = hexti->global_state;

  hexti->global_state = HAL_EXTI_STATE_ACTIVE;

  if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    LL_EXTI_EnableIT_0_31(hexti->ll_line);

    LL_EXTI_GenerateSWI_0_31(hexti->ll_line);
  }
  else
  {
    LL_EXTI_EnableIT_32_63(hexti->ll_line);

    LL_EXTI_GenerateSWI_32_63(hexti->ll_line);
  }

  return HAL_OK;
}

/**
  * @brief  Get interrupt pending bit of the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @retval pending_edge of type @ref hal_exti_trigger_t Trigger value.
  */
hal_exti_trigger_t HAL_EXTI_GetPending(const hal_exti_handle_t *hexti)
{
  hal_exti_trigger_t pending_edge = HAL_EXTI_TRIGGER_NONE;

  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM((IS_EXTI_CONFIG_LINE((uint32_t)hexti->line)));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_IDLE | (uint32_t)HAL_EXTI_STATE_ACTIVE);

  /* Check if the selected EXTI line is configurable */
  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) != 0U)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      /* Check if rising edge trigger is pending */
      if (LL_EXTI_IsActiveRisingFlag_0_31(hexti->ll_line) != 0UL)
      {
        /* Rising edge trigger is pending */
        pending_edge = HAL_EXTI_TRIGGER_RISING;
      }
      /* Check if falling edge trigger is pending */
      if (LL_EXTI_IsActiveFallingFlag_0_31(hexti->ll_line) != 0UL)
      {
        if (pending_edge == HAL_EXTI_TRIGGER_RISING)
        {
          pending_edge = HAL_EXTI_TRIGGER_RISING_FALLING;
        }
        else
        {
          pending_edge = HAL_EXTI_TRIGGER_FALLING;
        }
      }
    }
    else
    {
      /* Check if rising edge trigger is pending */
      if (LL_EXTI_IsActiveRisingFlag_32_63(hexti->ll_line) != 0UL)
      {
        /* Rising edge trigger is pending */
        pending_edge = HAL_EXTI_TRIGGER_RISING;
      }
      /* Check if falling edge trigger is pending */
      if (LL_EXTI_IsActiveFallingFlag_32_63(hexti->ll_line) != 0UL)
      {
        if (pending_edge == HAL_EXTI_TRIGGER_RISING)
        {
          pending_edge = HAL_EXTI_TRIGGER_RISING_FALLING;
        }
        else
        {
          pending_edge = HAL_EXTI_TRIGGER_FALLING;
        }
      }
    }
  }

  return (pending_edge);
}


/**
  * @brief  Clear interrupt pending bit of the selected EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @param edge Pending edge to be cleared. This parameter can be one of the values of
  *             @ref hal_exti_trigger_t Trigger value.
  */
void HAL_EXTI_ClearPending(const hal_exti_handle_t *hexti, hal_exti_trigger_t edge)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM((IS_EXTI_CONFIG_LINE((uint32_t)hexti->line)));
  ASSERT_DBG_PARAM(IS_EXTI_PENDING_EDGE(edge));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_IDLE | (uint32_t)HAL_EXTI_STATE_ACTIVE);

  /* Check if the selected EXTI line is configurable */
  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) != 0U)
  {
    if (((uint32_t)edge & (uint32_t)HAL_EXTI_TRIGGER_RISING) != 0UL)
    {
      /* Clear rising edge trigger pending bit */
      if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
      {
        LL_EXTI_ClearRisingFlag_0_31(hexti->ll_line);
      }
      else
      {
        LL_EXTI_ClearRisingFlag_32_63(hexti->ll_line);
      }
    }

    if (((uint32_t)edge & (uint32_t)HAL_EXTI_TRIGGER_FALLING) != 0UL)
    {
      /* Clear falling edge trigger pending bit */
      if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
      {
        LL_EXTI_ClearFallingFlag_0_31(hexti->ll_line);
      }
      else
      {
        LL_EXTI_ClearFallingFlag_32_63(hexti->ll_line);
      }
    }
  }
}
/**
  * @}
  */

/** @addtogroup EXTI_Exported_Functions_Group3
  * @{
  * ## This subsection contains the EXTI IRQHandler and callback registration functions:

### Handle EXTI interrupt requests using HAL_EXTI_IRQHandler():
  - Provide the EXTI handle as a parameter.

### Register callback function for interrupts on trigger edge using HAL_EXTI_RegisterTriggerCallback():
  - Provide the EXTI handle as the first parameter.
  - Provide a pointer to the callback function as the second parameter.

### Default callback function for interrupts on trigger edge using HAL_EXTI_TriggerCallback():
  - Provide the EXTI handle as a parameter.
  - Provide the trigger as the second parameter.

### Store user data pointer into the handle using HAL_EXTI_SetUserData():
  - Provide the EXTI handle as the first parameter.
  - Provide the pointer to the user data as the second parameter.

### Retrieve user data pointer from the handle using HAL_EXTI_GetUserData():
  - Provide the EXTI handle as a parameter. \n
<br>
  */

/**
  * @brief  Handle EXTI interrupt requests.
  * @param hexti Pointer to EXTI handle.
  */
void HAL_EXTI_IRQHandler(hal_exti_handle_t *hexti)
{
  hal_exti_trigger_t trigger = HAL_EXTI_TRIGGER_NONE;

  /* Check if previous state is not ACTIVE; the interrupt follows a HAL_EXTI_GenerateSWI call.
  Note that when HAL_EXTI_Enable is called, both global state and previous state are set to ACTIVE. */
  if (hexti->prev_state != HAL_EXTI_STATE_ACTIVE)
  {
    if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
    {
      LL_EXTI_DisableIT_0_31(hexti->ll_line);
    }
    else
    {
      LL_EXTI_DisableIT_32_63(hexti->ll_line);
    }

    /* Restore the previous state */
    hexti->global_state = hexti->prev_state;
  }

  if (((uint32_t)hexti->line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    if (LL_EXTI_IsActiveRisingFlag_0_31(hexti->ll_line) != 0UL)
    {
      LL_EXTI_ClearRisingFlag_0_31(hexti->ll_line);

      trigger = HAL_EXTI_TRIGGER_RISING;
    }

    if (LL_EXTI_IsActiveFallingFlag_0_31(hexti->ll_line) != 0UL)
    {
      LL_EXTI_ClearFallingFlag_0_31(hexti->ll_line);

      trigger = ((trigger == HAL_EXTI_TRIGGER_RISING) ? HAL_EXTI_TRIGGER_RISING_FALLING : HAL_EXTI_TRIGGER_FALLING);
    }
  }
  else
  {
    if (LL_EXTI_IsActiveRisingFlag_32_63(hexti->ll_line) != 0UL)
    {
      LL_EXTI_ClearRisingFlag_32_63(hexti->ll_line);

      trigger = HAL_EXTI_TRIGGER_RISING;
    }

    if (LL_EXTI_IsActiveFallingFlag_32_63(hexti->ll_line) != 0UL)
    {
      LL_EXTI_ClearFallingFlag_32_63(hexti->ll_line);

      trigger = ((trigger == HAL_EXTI_TRIGGER_RISING) ? HAL_EXTI_TRIGGER_RISING_FALLING : HAL_EXTI_TRIGGER_FALLING);
    }
  }

  if (trigger != HAL_EXTI_TRIGGER_NONE)
  {
#if defined (USE_HAL_EXTI_REGISTER_CALLBACKS) && (USE_HAL_EXTI_REGISTER_CALLBACKS == 1)
    hexti->p_trigger_cb(hexti, trigger);
#else
    HAL_EXTI_TriggerCallback(hexti, trigger);
#endif /* USE_HAL_EXTI_REGISTER_CALLBACKS */
  }
}

#if defined (USE_HAL_EXTI_REGISTER_CALLBACKS) && (USE_HAL_EXTI_REGISTER_CALLBACKS == 1)

/**
  * @brief  Register callback function for the selected EXTI line on trigger edge.
  * @param hexti Pointer to EXTI handle.
  * @param p_exti_cb Pointer to the callback function to be registered.
  * @retval HAL_INVALID_PARAM In case of an invalid parameter.
  * @retval HAL_OK In case of a successful callback registration.
  */
hal_status_t HAL_EXTI_RegisterTriggerCallback(hal_exti_handle_t *hexti, hal_exti_cb_t p_exti_cb)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(p_exti_cb != NULL);
  ASSERT_DBG_PARAM((IS_EXTI_CONFIG_LINE((uint32_t)hexti->line)));

  ASSERT_DBG_STATE(hexti->global_state, (uint32_t)HAL_EXTI_STATE_INIT | (uint32_t)HAL_EXTI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_exti_cb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (IS_EXTI_CONFIG_LINE((uint32_t)hexti->line) == 0U) /* Check if selected EXTI line is configurable */
  {
    return HAL_ERROR;
  }

  hexti->p_trigger_cb = p_exti_cb;

  return HAL_OK;
}
#endif /* USE_HAL_EXTI_REGISTER_CALLBACKS */

/**
  * @brief  EXTI line trigger edge default callback.
  * @param hexti Pointer to EXTI handle.
  * @param trigger This parameter can be one of the values of @ref hal_exti_trigger_t Trigger value.
  */
__WEAK void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hexti);
  STM32_UNUSED(trigger);

  /*! <b>NOTE:</b> This function must not be modified. When the callback is needed,
              the HAL_EXTI_TriggerCallback can be implemented in the user file.
   */
}

#if defined (USE_HAL_EXTI_USER_DATA) && (USE_HAL_EXTI_USER_DATA == 1)

/**
  * @brief Store the user data pointer in the handle.
  * @param hexti Pointer to EXTI handle.
  * @param p_user_data Pointer to the user data.
  */
void HAL_EXTI_SetUserData(hal_exti_handle_t *hexti, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hexti != NULL);

  hexti->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the user data pointer from the handle.
  * @param hexti Pointer to EXTI handle.
  * @retval Pointer to the user data.
  */
const void *HAL_EXTI_GetUserData(const hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);

  return (hexti->p_user_data);
}

#endif /* USE_HAL_EXTI_USER_DATA */
/**
  * @}
  */

/** @addtogroup EXTI_Exported_Functions_Group4
  * @{
  * ## This subsection contains the EXTI state and info functions:

### Retrieve the global state of the current EXTI line using HAL_EXTI_GetState():
  - Provide the EXTI handle as a parameter.

### Retrieve the HAL EXTI line using HAL_EXTI_GetInstance():
  - Provide the EXTI handle as a parameter.

### Retrieve the LL EXTI line using HAL_EXTI_GetLLInstance():
  - Provide the EXTI handle as a parameter.
  */

/**
  * @brief  Get the current general state of the EXTI line.
  * @param hexti Pointer to EXTI handle.
  * @retval HAL_EXTI_STATE_RESET when EXTI is de-initialized.
  * @retval HAL_EXTI_STATE_INIT when EXTI is initialized but not yet configured.
  * @retval HAL_EXTI_STATE_IDLE when EXTI is initialized and configured.
  * @retval HAL_EXTI_STATE_ACTIVE when EXTI is initialized, configured, and enabled.
  */
hal_exti_state_t HAL_EXTI_GetState(const hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);

  return hexti->global_state;
}

/**
  * @brief  Get the HAL EXTI line.
  * @param  hexti Pointer to a \ref hal_exti_handle_t structure.
  * @retval The HAL EXTI line.
  */
hal_exti_line_t HAL_EXTI_GetInstance(const hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)hexti->line));

  return (hexti->line);
}

/**
  * @brief  Get the LL instance from the handle.
  * @param  hexti Pointer to a \ref hal_exti_handle_t structure.
  * @retval The LL EXTI Line.
  */
uint32_t HAL_EXTI_GetLLInstance(const hal_exti_handle_t *hexti)
{
  ASSERT_DBG_PARAM(hexti != NULL);
  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)hexti->line));

  return (hexti->ll_line);
}
/**
  * @}
  */

/** @addtogroup EXTI_Exported_Functions_Group5
  * @{
  * ## This subsection contains EXTI security attributes management functions:

### Set the EXTI line privilege attribute using HAL_EXTI_SetPrivAttr():
  - Provide the EXTI line as first parameter.
  - Provide the EXTI privilege attribute as second parameter.

### Get the EXTI line privilege attribute using HAL_EXTI_GetPrivAttr():
  - Provide the EXTI line as first parameter.
  */

/**
  * @brief  Set the privileged access level attribute for an EXTI line.
  * @param exti_line This parameter can be a value of @ref hal_exti_line_t.
  * @param priv_attr This parameter can be one of the following values:
  *         @arg @ref HAL_EXTI_PRIV
  *         @arg @ref HAL_EXTI_NPRIV
  * @retval HAL_OK Privilege attribute has been set successfully
  * @retval HAL_ERROR Non-privileged write to a privileged-only register.
  */
hal_status_t HAL_EXTI_SetPrivAttr(hal_exti_line_t exti_line, hal_exti_priv_attr_t priv_attr)
{
  uint32_t ll_exti_line;

  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)exti_line));
  ASSERT_DBG_PARAM(IS_EXTI_LINE_PRIV_ATTR(priv_attr));

  ll_exti_line = 1UL << ((uint32_t)exti_line & LL_EXTI_PIN_MASK);

  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  if (((uint32_t)exti_line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    LL_EXTI_SetPrivAttr_0_31(ll_exti_line, (uint32_t)priv_attr);
  }
  else
  {
    LL_EXTI_SetPrivAttr_32_63(ll_exti_line, (uint32_t)priv_attr);
  }

  return HAL_OK;
}

/**
  * @brief  Get the privileged access level attribute for an EXTI line.
  * @param exti_line This parameter can be one of the values of @ref hal_exti_line_t.
  * @retval The returned value can be one of the following values:
  *         @arg @ref HAL_EXTI_PRIV
  *         @arg @ref HAL_EXTI_NPRIV
  */
hal_exti_priv_attr_t HAL_EXTI_GetPrivAttr(hal_exti_line_t exti_line)
{
  uint32_t ll_exti_line;

  ASSERT_DBG_PARAM(IS_EXTI_LINE((uint32_t)exti_line));

  ll_exti_line = 1UL << ((uint32_t)exti_line & LL_EXTI_PIN_MASK);

  if (((uint32_t)exti_line & LL_EXTI_REG1) == LL_EXTI_REG1)
  {
    return ((hal_exti_priv_attr_t)LL_EXTI_GetPrivAttr_0_31(ll_exti_line));
  }
  else
  {
    return ((hal_exti_priv_attr_t)LL_EXTI_GetPrivAttr_32_63(ll_exti_line));
  }
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

#endif /* USE_HAL_EXTI_MODULE */
#endif /* EXTI */
/**
  * @}
  */
