/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_gpio.c
  * @brief   GPIO HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the General Purpose Input/Output (GPIO) peripheral:
  *           + Initialization and de-initialization functions
  *           + I/O operation functions
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
#if defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) \
    || defined (GPIOF) ||  defined (GPIOG) || defined (GPIOH)
#if defined(USE_HAL_GPIO_MODULE) && (USE_HAL_GPIO_MODULE == 1U)

/** @addtogroup GPIO
  * @{
  */

/** @defgroup GPIO_Introduction GPIO Introduction
  * @{

  The GPIO hardware abstraction layer (HAL) provides a set of APIs to interface with the general-purpose
  Input/Output (GPIO) peripheral on STM32 microcontrollers.

  This layer simplifies the configuration, initialization, and management of GPIO pins, allowing multiple
  pins to be managed simultaneously when they belong to the same port.
  The GPIO includes the following features:

   - Initialize one or more GPIO pins.
   - Read the value of a pin.
   - Write values to one or more pins.
   - Toggle the state of pins.
   - Lock pins configuration.
   - Lock a set of pins.

  This abstraction layer ensures portability and ease of use across different STM32 series microcontrollers.
  * @}
  */

/** @defgroup GPIO_How_To_Use GPIO How To Use
  * @{

# GPIO features

- Configure each port bit of the general-purpose I/O (GPIO) ports in one of the following modes:
  - Input mode
  - Output mode
  - Alternate function mode
  - Analog mode

- After startup:
  - The alternate functions are inactive.
  - The I/O ports are configured in analog mode, with the exception of some pre-configured pins
  (debug pins, for instance).

- All GPIO pins have weak internal pull-up and pull-down resistors that can be activated or deactivated.

  - In output or alternate mode:
    - Configure each I/O for open-drain or push-pull type.
    - Select the I/O speed based on the VDD value.

- The microcontroller I/O pins are connected to onboard peripherals/modules through a multiplexer:
  - It allows only one peripheral (alternate function) to be connected to an I/O pin at a time. \n
  &rarr; No conflict occurs between peripherals sharing the same I/O pin.

- Use the LSE oscillator pins OSC32_IN and OSC32_OUT as general-purpose I/O when LSE is off.
\note The LSE has priority over the GPIO function.

<br>

6. Use the HSE oscillator pins OSC_IN/OSC_OUT as general-purpose I/O when HSE is off.
\note The HSE has priority over the GPIO function.

<br>

# How to use the GPIO HAL module driver

## Use the GPIO HAL driver as follows:

- Enable the GPIO peripheral clock:
  - Either at application level by calling the **HAL_RCC_GPIOx_EnableClock()** API
  - Or by setting the USE_HAL_GPIO_CLK_ENABLE_MODEL define in the HAL configuration file to:
      - HAL_CLK_ENABLE_PERIPH_ONLY or HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM: in this case the GPIOx clock will be enabled
        within the HAL_GPIO_Init()

- Configure the GPIO pin(s) using HAL_GPIO_Init().
  - Set the I/O mode to the "mode" member from hal_gpio_config_t structure.
  - Select the pull-up or pull-down resistor using the "pull" member from hal_gpio_config_t structure.
  - For output or alternate function mode selection:
    - Configure the speed through the "speed" member from hal_gpio_config_t structure.
  - For alternate mode:
    - Configure the alternate function connected to the I/O through the "alternate"
    member from hal_gpio_config_t structure.
  - For output mode:
    - Configure the initial pin state through the "init_state" member from hal_gpio_config_t structure.
  - Select analog mode when a pin is used as an ADC channel input or as a DAC output.
  - For a GPIO pin with an external interrupt/event, use the HAL EXTI driver to configure the corresponding
    EXTI line.

- To reset the configuration of GPIO pin(s), use HAL_GPIO_DeInit().

- To get the level of a pin configured in input mode, use HAL_GPIO_ReadPin().

- To set/reset the level of pin(s) configured in output mode, use HAL_GPIO_WritePin() / HAL_GPIO_TogglePin().

- To set the level of several pins and reset the level of other pins in the same cycle,
 use HAL_GPIO_WriteMultipleStatePin().

- To lock a GPIO pin configuration until the next reset, use HAL_GPIO_LockPin().

- To get the selected GPIO hardware instance, use HAL_GPIO_GetLLInstance() function.

  */

/**
  * @}
  */

/** @defgroup GPIO_Configuration_Table GPIO Configuration Table
  * @{

# Configuration inside the GPIO driver

Config defines                | Description      | Default value         | Note
------------------------------| -----------------| --------------------- | --------------------------------------------
USE_HAL_GPIO_MODULE           | hal_conf.h       | 1                     | When set, HAL GPIO module is enabled
USE_HAL_CHECK_PARAM           | hal_conf.h       | 0                     | When set, parameters are checked at runtime
USE_ASSERT_DBG_PARAM          | PreProcessor env | NA                    | When defined, enable the params assert
USE_HAL_GPIO_CLK_ENABLE_MODEL | hal_conf.h       | HAL_CLK_ENABLE_NO     | Enable GPIO port clock in HAL_GPIO_Init
  */

/**
  * @}
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Private_Types GPIO Private Types
  * @{
  */

/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Private_Constants GPIO Private Constants
  * @{
  */

#define GPIO_NUMBER                 (16U)   /*!< GPIO port pins number.               */

/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */
/*! Macro to compute the clock enable mask from GPIO port */
#define GET_GPIO_CLK_ENABLE_BIT(port) ((1UL << (((port) & 0x03F00U) >> 10U)) << RCC_AHB2ENR_GPIOAEN_Pos)

/*! Macro to get the instance */
#define GPIO_GET_INSTANCE(instance)  ((GPIO_TypeDef *)((uint32_t)(instance)))

/*! Macro to check GPIO port validity */
#if defined(GPIOF) && defined(GPIOG)
#define IS_GPIO_PORT(port)  (((port) == HAL_GPIOA) \
                             || ((port) == HAL_GPIOB) \
                             || ((port) == HAL_GPIOC) \
                             || ((port) == HAL_GPIOD) \
                             || ((port) == HAL_GPIOE) \
                             || ((port) == HAL_GPIOF) \
                             || ((port) == HAL_GPIOG) \
                             || ((port) == HAL_GPIOH))
#else
#define IS_GPIO_PORT(port)  (((port) == HAL_GPIOA) \
                             || ((port) == HAL_GPIOB) \
                             || ((port) == HAL_GPIOC) \
                             || ((port) == HAL_GPIOD) \
                             || ((port) == HAL_GPIOE) \
                             || ((port) == HAL_GPIOH))
#endif /* GPIOF && GPIOG */

/*! Macro to check pin(s) validity */
#define IS_GPIO_PIN(pin)        ((((uint32_t)(pin) & HAL_GPIO_PIN_ALL) != 0x00U) \
                                 && (((uint32_t)(pin) & ~HAL_GPIO_PIN_ALL) == 0x00U))

/*! Macro to check one single pin validity */
#define IS_GPIO_SINGLE_PIN(pin)  (((pin) == HAL_GPIO_PIN_0) \
                                  || ((pin) == HAL_GPIO_PIN_1) \
                                  || ((pin) == HAL_GPIO_PIN_2) \
                                  || ((pin) == HAL_GPIO_PIN_3) \
                                  || ((pin) == HAL_GPIO_PIN_4) \
                                  || ((pin) == HAL_GPIO_PIN_5) \
                                  || ((pin) == HAL_GPIO_PIN_6) \
                                  || ((pin) == HAL_GPIO_PIN_7) \
                                  || ((pin) == HAL_GPIO_PIN_8) \
                                  || ((pin) == HAL_GPIO_PIN_9) \
                                  || ((pin) == HAL_GPIO_PIN_10) \
                                  || ((pin) == HAL_GPIO_PIN_11) \
                                  || ((pin) == HAL_GPIO_PIN_12) \
                                  || ((pin) == HAL_GPIO_PIN_13) \
                                  || ((pin) == HAL_GPIO_PIN_14) \
                                  || ((pin) == HAL_GPIO_PIN_15))

/*! Macro to check mode validity */
#define IS_GPIO_MODE(mode)  (((mode) == HAL_GPIO_MODE_INPUT) \
                             || ((mode) == HAL_GPIO_MODE_OUTPUT) \
                             || ((mode) == HAL_GPIO_MODE_ALTERNATE) \
                             || ((mode) == HAL_GPIO_MODE_ANALOG))

/*! Macro to check pull validity */
#define IS_GPIO_PULL(pull)  (((pull) == HAL_GPIO_PULL_NO) \
                             || ((pull) == HAL_GPIO_PULL_UP) \
                             || ((pull) == HAL_GPIO_PULL_DOWN))


/*! Macro to check speed validity */
#define IS_GPIO_SPEED(speed) (((speed) == HAL_GPIO_SPEED_FREQ_LOW) \
                              || ((speed) == HAL_GPIO_SPEED_FREQ_MEDIUM) \
                              || ((speed) == HAL_GPIO_SPEED_FREQ_HIGH) \
                              || ((speed) == HAL_GPIO_SPEED_FREQ_VERY_HIGH))

/*! Macro to check output type validity */
#define IS_GPIO_OUTPUT_TYPE(output_type)  (((output_type) == HAL_GPIO_OUTPUT_PUSHPULL) \
                                           || ((output_type) == HAL_GPIO_OUTPUT_OPENDRAIN))

/*! Macro to check alternate validity */
#define IS_GPIO_ALTERNATE(alternate)  (((alternate) == HAL_GPIO_AF_0) \
                                       || ((alternate) == HAL_GPIO_AF_1) \
                                       || ((alternate) == HAL_GPIO_AF_2) \
                                       || ((alternate) == HAL_GPIO_AF_3) \
                                       || ((alternate) == HAL_GPIO_AF_4) \
                                       || ((alternate) == HAL_GPIO_AF_5) \
                                       || ((alternate) == HAL_GPIO_AF_6) \
                                       || ((alternate) == HAL_GPIO_AF_7) \
                                       || ((alternate) == HAL_GPIO_AF_8) \
                                       || ((alternate) == HAL_GPIO_AF_9) \
                                       || ((alternate) == HAL_GPIO_AF_10) \
                                       || ((alternate) == HAL_GPIO_AF_11) \
                                       || ((alternate) == HAL_GPIO_AF_12) \
                                       || ((alternate) == HAL_GPIO_AF_13) \
                                       || ((alternate) == HAL_GPIO_AF_14) \
                                       || ((alternate) == HAL_GPIO_AF_15))

/*! Macro to check state validity */
#define IS_GPIO_PIN_STATE(state)   (((state)  ==  HAL_GPIO_PIN_SET) || ((state)  ==  HAL_GPIO_PIN_RESET))

/*! Macro to check pins overlap between those to set and those to reset */
#define IS_GPIO_COMMON_PIN(resetmask, setmask)  (((uint32_t)(resetmask) & (uint32_t)(setmask)) == 0x00u)

/**
  * @}
  */

/* Private variables -------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Private_Variables GPIO Private Variables
  * @{
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @addtogroup GPIO_Exported_Functions
  * @{
  */

/** @addtogroup GPIO_Exported_Functions_Group1
  * @{
  * ## This subsection provides a set of functions allowing to initialize/de-initialize and configure GPIO ports.

### Initialize GPIO pins of a dedicated GPIO port using HAL_GPIO_Init():
  - Provide the GPIO port as a parameter.
  - Specify the GPIO pins to be configured, as a second parameter.
  - Provide the configuration structure containing the configuration to be applied as the third parameter.
### De-initialize GPIO pins of a dedicated GPIO port using HAL_GPIO_DeInit():
  - Provide the GPIO port as a first parameter.
  - Specify the GPIO pins to be de-initialized as a second parameter.
<br>
  */

/**
  * @brief  Initialize a pin or a set of pins for a GPIO port according to the specified parameters in the p_config.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pins specifies the port pins to be written.
  *         This parameter can be a combination of HAL_GPIO_PIN_x where x can be (0..15).
  * @param  p_config pointer to a @ref hal_gpio_config_t structure that contains
  *         the configuration information for the specified GPIO pins.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  * @retval HAL_OK GPIO pins has been correctly configured.
  */
hal_status_t  HAL_GPIO_Init(hal_gpio_t gpiox, uint32_t pins, const hal_gpio_config_t *p_config)
{
  GPIO_TypeDef  *p_gpio;

  uint32_t tmp_pins_msk;
  uint32_t mode;
  uint32_t alternate;
  uint32_t position;
  uint32_t iocurrent;
  uint32_t io_msk;

  ASSERT_DBG_PARAM(IS_GPIO_PORT(gpiox));
  ASSERT_DBG_PARAM(IS_GPIO_PIN(pins));
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_GPIO_MODE(p_config->mode));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)

  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  tmp_pins_msk = pins;
  p_gpio = GPIO_GET_INSTANCE(gpiox);
  mode = (uint32_t)p_config->mode;
  alternate = (uint32_t)p_config->alternate;

  position = STM32_POSITION_VAL(tmp_pins_msk);     /* Get the first pin position from the pins mask */
  iocurrent = 1UL << position;                     /* Get the first pin from the pins mask */

#if defined (USE_HAL_GPIO_CLK_ENABLE_MODEL) && (USE_HAL_GPIO_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  LL_AHB2_GRP1_EnableClock(GET_GPIO_CLK_ENABLE_BIT((uint32_t)gpiox));
#endif /* USE_HAL_GPIO_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

  if (mode == LL_GPIO_MODE_OUTPUT)
  {
    ASSERT_DBG_PARAM(IS_GPIO_PIN_STATE(p_config->init_state));

    LL_GPIO_WriteOutputPin(p_gpio, pins, (uint32_t)p_config->init_state);
  }

  /* Configure the port pins */
  while (tmp_pins_msk != 0U)
  {
    /* The io_mask is identical to the current one */
    io_msk = iocurrent;

    if ((mode == LL_GPIO_MODE_OUTPUT) || (mode == LL_GPIO_MODE_ALTERNATE))
    {
      ASSERT_DBG_PARAM(IS_GPIO_SPEED(p_config->speed));
      ASSERT_DBG_PARAM(IS_GPIO_OUTPUT_TYPE(p_config->output_type));

      LL_GPIO_SetPinSpeed(p_gpio, io_msk, (uint32_t)p_config->speed);

      LL_GPIO_SetPinOutputType(p_gpio, io_msk, (uint32_t)p_config->output_type);
    }

    if ((mode != LL_GPIO_MODE_ANALOG) || ((mode == LL_GPIO_MODE_ANALOG) && (p_config->pull != HAL_GPIO_PULL_UP)))
    {
      ASSERT_DBG_PARAM(IS_GPIO_PULL(p_config->pull));

      LL_GPIO_SetPinPull(p_gpio, io_msk, (uint32_t)p_config->pull);
    }

    if (mode == LL_GPIO_MODE_ALTERNATE)
    {
      ASSERT_DBG_PARAM(IS_GPIO_ALTERNATE(p_config->alternate));

      if (io_msk < LL_GPIO_PIN_8)
      {
        LL_GPIO_SetAFPin_0_7(p_gpio, io_msk, alternate);
      }
      else
      {
        LL_GPIO_SetAFPin_8_15(p_gpio, io_msk, alternate);
      }
    }

    LL_GPIO_SetPinMode(p_gpio, io_msk, mode);

    tmp_pins_msk &= (~iocurrent);                   /* Clear the current pin from the pins mask */
    position = STM32_POSITION_VAL(tmp_pins_msk);    /* Get the next pin position from the pins mask */
    iocurrent = 1UL << position;                    /* Get the next pin from the pins mask */
  }
  return HAL_OK;
}

/**
  * @brief  Reset the configuration of a pin or a set of pins for a GPIO port to the default one.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pins specifies the port pins to be written.
  *         This parameter can be a combination of HAL_GPIO_PIN_x where x can be (0..15).
  */
void  HAL_GPIO_DeInit(hal_gpio_t gpiox, uint32_t pins)
{
  uint32_t tmp_pins_msk = pins;
  uint32_t position;
  uint32_t iocurrent;
  uint32_t io_msk;

  ASSERT_DBG_PARAM(IS_GPIO_PORT((gpiox)));
  ASSERT_DBG_PARAM(IS_GPIO_PIN(pins));

  GPIO_TypeDef  *p_gpio = GPIO_GET_INSTANCE(gpiox);

  position = STM32_POSITION_VAL(tmp_pins_msk);     /* Get the first pin position from the pins mask */
  iocurrent = 1UL << position;                     /* Get the first pin from the pins mask */

  LL_GPIO_WriteOutputPin(p_gpio, pins, LL_GPIO_PIN_RESET);

  while (tmp_pins_msk != 0U)
  {
    /* The io_mask is identical to the current one */
    io_msk = iocurrent;

    LL_GPIO_SetPinMode(p_gpio, io_msk, LL_GPIO_MODE_ANALOG);

    if (io_msk < LL_GPIO_PIN_8)
    {
      LL_GPIO_SetAFPin_0_7(p_gpio, io_msk, LL_GPIO_AF_0);
    }
    else
    {
      LL_GPIO_SetAFPin_8_15(p_gpio, io_msk, LL_GPIO_AF_0);
    }

    LL_GPIO_SetPinSpeed(p_gpio, io_msk, LL_GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetPinOutputType(p_gpio, io_msk, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinPull(p_gpio, io_msk, LL_GPIO_PULL_NO);

    tmp_pins_msk &= (~iocurrent);                   /* Clear the current pin from the pins mask */
    position = STM32_POSITION_VAL(tmp_pins_msk);    /* Get the next pin position from the pins mask */
    iocurrent = 1UL << position;                    /* Get the next pin from the pins mask */
  }
}

/**
  * @}
  */

/** @addtogroup GPIO_Exported_Functions_Group2
  * @{
  * ## This subsection contains the APIs managing possible I/O operations on the GPIO pins:

### Read the input level of a dedicated input port pin using HAL_GPIO_ReadPin():
  - Provide the GPIO port as a first parameter.
  - Provide the GPIO pin to read as a second parameter.

### Set or clear a set of pins of a GPIO port using HAL_GPIO_WritePin():
  - Provide the GPIO port as a first parameter.
  - Specify the pins to be updated as a second parameter.
  - Enter the pin state to be written to the specified pin as the third parameter.

### Set and clear several pins of the same GPIO port in a single cycle using HAL_GPIO_WriteMultipleStatePin():
  - Provide the GPIO port as the first parameter.
  - Specify the set of pins to be reset as a second parameter.
  - Specify the set of pins to be set as a third parameter.

### Toggle a set of pins of a dedicated GPIO port using HAL_GPIO_TogglePin():
  - Provide the GPIO port as a first parameter.
  - Specify the set of pins to be toggled as a second parameter.

### Lock the configuration of a set of pins of a dedicated GPIO port using HAL_GPIO_LockPin():
  - Provide the GPIO port as a first parameter.
  - Specify the set of pins of which to lock the configuration as a second parameter.

### To get the selected GPIO hardware instance, use HAL_GPIO_GetLLInstance() function.

<br>
  */

/**
  * @brief  Read the specific input pin for a GPIO port.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pin specifies the port pin to read.
  *         This parameter can be one of HAL_GPIO_PIN_x where x can be (0..15).
  * @retval HAL_GPIO_PIN_RESET when the input port pin is low.
  * @retval HAL_GPIO_PIN_SET when the input port pin is high.
  */
hal_gpio_pin_state_t HAL_GPIO_ReadPin(hal_gpio_t gpiox, uint32_t pin)
{
  ASSERT_DBG_PARAM(IS_GPIO_PORT(gpiox));
  ASSERT_DBG_PARAM(IS_GPIO_SINGLE_PIN(pin));

  return (hal_gpio_pin_state_t)LL_GPIO_IsInputPinSet(GPIO_GET_INSTANCE(gpiox), pin);
}

/**
  * @brief  Set or clear a pin or a set of pins of a GPIO port.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pins specifies the port pins to be written.
  * @param  pin_state specifies the value to be written to the selected bit.
  *         This parameter can be one of the @ref hal_gpio_pin_state_t enum values:
  *            @arg HAL_GPIO_PIN_RESET: to clear the port pin.
  *            @arg HAL_GPIO_PIN_SET: to set the port pin.
  * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  */
void HAL_GPIO_WritePin(hal_gpio_t gpiox, uint32_t pins, hal_gpio_pin_state_t pin_state)
{
  ASSERT_DBG_PARAM(IS_GPIO_PORT(gpiox));
  ASSERT_DBG_PARAM(IS_GPIO_PIN(pins));
  ASSERT_DBG_PARAM(IS_GPIO_PIN_STATE(pin_state));

  LL_GPIO_WriteOutputPin(GPIO_GET_INSTANCE(gpiox), pins, (uint32_t)pin_state);
}

/**
  * @brief    Set and clear a pin or a set of pins of a GPIO port in same cycle.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param    pins_reset specifies the port bits to be reset.
  *           This parameter can be any combination of HAL_GPIO_PIN_x where x can be (0..15) or zero.
  * @param    pins_set specifies the port bits to be set.
  *           This parameter can be any combination of HAL_GPIO_PIN_x where x can be (0..15) or zero.
  * @note     This function uses GPIOx_BSRR register to set the level of several pins
  *           and reset level of several other pins in the same cycle.
  * @note     Ensure at least one of the two parameters used to set or reset is different from zero.
  * @warning  Do not set any common bit in pins_reset and pins_set combinations, otherwise the
  *           assert is triggered.
  */
void HAL_GPIO_WriteMultipleStatePin(hal_gpio_t gpiox, uint32_t pins_reset, uint32_t pins_set)
{
  /* Make sure at least one parameter is different from zero no common pin between set and reset mask */
  ASSERT_DBG_PARAM(IS_GPIO_PORT((gpiox)));
  ASSERT_DBG_PARAM(IS_GPIO_PIN((uint32_t)pins_reset | (uint32_t)pins_set));
  ASSERT_DBG_PARAM(IS_GPIO_COMMON_PIN(pins_reset, pins_set));

  LL_GPIO_WriteMultipleStatePin(GPIO_GET_INSTANCE(gpiox), pins_reset, pins_set);
}

/**
  * @brief  Toggle a specific pin of a GPIO port.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pins specifies the port pins to be toggled.
  *         This parameter can be a combination of HAL_GPIO_PIN_x where x can be (0..15).
  */
void  HAL_GPIO_TogglePin(hal_gpio_t gpiox, uint32_t pins)
{
  ASSERT_DBG_PARAM(IS_GPIO_PORT((gpiox)));
  ASSERT_DBG_PARAM(IS_GPIO_PIN(pins));

  LL_GPIO_TogglePin(GPIO_GET_INSTANCE(gpiox), pins);
}

/**
  * @brief  Lock the configuration of a pin or a set of pins of a GPIO port.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @param  pins specifies the port pins to be locked.
  *         This parameter can be any combination of HAL_GPIO_PIN_x where x can be (0..15).
  * @retval HAL_OK  pins locked successfully.
  * @retval HAL_ERROR  error occurred during pins lock operation.
  * @note   The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  */
hal_status_t  HAL_GPIO_LockPin(hal_gpio_t gpiox, uint32_t pins)
{
  hal_status_t status = HAL_OK;

  ASSERT_DBG_PARAM(IS_GPIO_PORT(gpiox));
  ASSERT_DBG_PARAM(IS_GPIO_PIN(pins));

  GPIO_TypeDef  *p_gpio = GPIO_GET_INSTANCE(gpiox);

  /* In case of GPIO port, can lock all selected pins in one shot */
  LL_GPIO_LockPin(p_gpio, pins);

  if (LL_GPIO_IsAnyPinLocked(p_gpio) == 0UL)
  {
    status = HAL_ERROR;
  }

  return status;
}
/**
  * @}
  */

/**
  * @brief  Get the selected hardware GPIO instance.
  * @param  gpiox HAL_GPIOx: selected GPIO port based on @ref hal_gpio_t.
  * @retval The selected hardware GPIO instance.
  */
GPIO_TypeDef *HAL_GPIO_GetLLInstance(hal_gpio_t gpiox)
{
  ASSERT_DBG_PARAM(IS_GPIO_PORT(gpiox));

  return (GPIO_GET_INSTANCE(gpiox));
}


/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_HAL_GPIO_MODULE */
#endif /* GPIOA || GPIOB || GPIOC || GPIOD || GPIOE || GPIOF || GPIOG || GPIOH */
/**
  * @}
  */
