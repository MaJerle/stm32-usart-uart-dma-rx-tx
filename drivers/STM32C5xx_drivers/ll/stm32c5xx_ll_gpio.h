/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_gpio.h
  * @brief   Header file of GPIO LL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_LL_GPIO_H
#define STM32C5XX_LL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) \
    || defined (GPIOF) ||  defined (GPIOG) || defined (GPIOH)

/** @defgroup GPIO_LL GPIO
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_LL_Exported_Constants LL GPIO Constants
  * @{
  */

/** @defgroup GPIO_LL_EC_PIN PIN
  * @{
  */
#define LL_GPIO_PIN_0          GPIO_BSRR_BS0     /*!< Select pin 0. */
#define LL_GPIO_PIN_1          GPIO_BSRR_BS1     /*!< Select pin 1. */
#define LL_GPIO_PIN_2          GPIO_BSRR_BS2     /*!< Select pin 2. */
#define LL_GPIO_PIN_3          GPIO_BSRR_BS3     /*!< Select pin 3. */
#define LL_GPIO_PIN_4          GPIO_BSRR_BS4     /*!< Select pin 4. */
#define LL_GPIO_PIN_5          GPIO_BSRR_BS5     /*!< Select pin 5. */
#define LL_GPIO_PIN_6          GPIO_BSRR_BS6     /*!< Select pin 6. */
#define LL_GPIO_PIN_7          GPIO_BSRR_BS7     /*!< Select pin 7. */
#define LL_GPIO_PIN_8          GPIO_BSRR_BS8     /*!< Select pin 8. */
#define LL_GPIO_PIN_9          GPIO_BSRR_BS9     /*!< Select pin 9. */
#define LL_GPIO_PIN_10         GPIO_BSRR_BS10    /*!< Select pin 10. */
#define LL_GPIO_PIN_11         GPIO_BSRR_BS11    /*!< Select pin 11. */
#define LL_GPIO_PIN_12         GPIO_BSRR_BS12    /*!< Select pin 12. */
#define LL_GPIO_PIN_13         GPIO_BSRR_BS13    /*!< Select pin 13. */
#define LL_GPIO_PIN_14         GPIO_BSRR_BS14    /*!< Select pin 14. */
#define LL_GPIO_PIN_15         GPIO_BSRR_BS15    /*!< Select pin 15. */
#define LL_GPIO_PIN_ALL       (GPIO_BSRR_BS0 | GPIO_BSRR_BS1  | GPIO_BSRR_BS2   \
                               | GPIO_BSRR_BS3  | GPIO_BSRR_BS4  | GPIO_BSRR_BS5   \
                               | GPIO_BSRR_BS6  | GPIO_BSRR_BS7  | GPIO_BSRR_BS8   \
                               | GPIO_BSRR_BS9  | GPIO_BSRR_BS10 | GPIO_BSRR_BS11  \
                               | GPIO_BSRR_BS12 | GPIO_BSRR_BS13 | GPIO_BSRR_BS14  \
                               | GPIO_BSRR_BS15) /*!< Select all pins. */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_PIN_STATE Pin State
  * @{
  */
#define LL_GPIO_PIN_RESET                 (0U)   /*!< Pin state is reset/low. */
#define LL_GPIO_PIN_SET                   (1UL)  /*!< Pin state is set/high.  */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_MODE Mode
  * @{
  */
#define LL_GPIO_MODE_INPUT                 0x00000000U         /*!< Select input mode. */
#define LL_GPIO_MODE_OUTPUT                GPIO_MODER_MODE0_0  /*!< Select output mode. */
#define LL_GPIO_MODE_ALTERNATE             GPIO_MODER_MODE0_1  /*!< Select alternate function mode. */
#define LL_GPIO_MODE_ANALOG                GPIO_MODER_MODE0    /*!< Select analog mode. */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_OUTPUT Output Type
  * @{
  */
#define LL_GPIO_OUTPUT_PUSHPULL            0x00000000U     /*!< Select push-pull as output type. */
#define LL_GPIO_OUTPUT_OPENDRAIN           GPIO_OTYPER_OT0 /*!< Select open-drain as output type. */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_SPEED Output Speed
  * @{
  */
#define LL_GPIO_SPEED_FREQ_LOW             0x00000000U            /*!< Select I/O low output speed.    */
#define LL_GPIO_SPEED_FREQ_MEDIUM          GPIO_OSPEEDR_OSPEED0_0 /*!< Select I/O medium output speed. */
#define LL_GPIO_SPEED_FREQ_HIGH            GPIO_OSPEEDR_OSPEED0_1 /*!< Select I/O fast output speed.   */
#define LL_GPIO_SPEED_FREQ_VERY_HIGH       GPIO_OSPEEDR_OSPEED0   /*!< Select I/O high output speed.   */
/**
  * @}
  */
#define LL_GPIO_SPEED_LOW                  LL_GPIO_SPEED_FREQ_LOW
#define LL_GPIO_SPEED_MEDIUM               LL_GPIO_SPEED_FREQ_MEDIUM
#define LL_GPIO_SPEED_FAST                 LL_GPIO_SPEED_FREQ_HIGH
#define LL_GPIO_SPEED_HIGH                 LL_GPIO_SPEED_FREQ_VERY_HIGH

/** @defgroup GPIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
#define LL_GPIO_PULL_NO                    0x00000000U        /*!< Select I/O no pull. */
#define LL_GPIO_PULL_UP                    GPIO_PUPDR_PUPD0_0 /*!< Select I/O pull up. */
#define LL_GPIO_PULL_DOWN                  GPIO_PUPDR_PUPD0_1 /*!< Select I/O pull down. */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_AF Alternate Function
  * @{
  */
#define LL_GPIO_AF_0                       0x0000000U /*!< Select alternate function 0. */
#define LL_GPIO_AF_1                       0x0000001U /*!< Select alternate function 1. */
#define LL_GPIO_AF_2                       0x0000002U /*!< Select alternate function 2. */
#define LL_GPIO_AF_3                       0x0000003U /*!< Select alternate function 3. */
#define LL_GPIO_AF_4                       0x0000004U /*!< Select alternate function 4. */
#define LL_GPIO_AF_5                       0x0000005U /*!< Select alternate function 5. */
#define LL_GPIO_AF_6                       0x0000006U /*!< Select alternate function 6. */
#define LL_GPIO_AF_7                       0x0000007U /*!< Select alternate function 7. */
#define LL_GPIO_AF_8                       0x0000008U /*!< Select alternate function 8. */
#define LL_GPIO_AF_9                       0x0000009U /*!< Select alternate function 9. */
#define LL_GPIO_AF_10                      0x000000AU /*!< Select alternate function 10. */
#define LL_GPIO_AF_11                      0x000000BU /*!< Select alternate function 11. */
#define LL_GPIO_AF_12                      0x000000CU /*!< Select alternate function 12. */
#define LL_GPIO_AF_13                      0x000000DU /*!< Select alternate function 13. */
#define LL_GPIO_AF_14                      0x000000EU /*!< Select alternate function 14. */
#define LL_GPIO_AF_15                      0x000000FU /*!< Select alternate function 15. */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup GPIO_LL_Exported_Macros LL GPIO Macros
  * @{
  */

/** @defgroup GPIO_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in GPIO register.
  * @param  instance GPIO Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_GPIO_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in GPIO register.
  * @param  instance GPIO Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_GPIO_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GPIO_LL_Exported_Functions LL GPIO Functions
  * @{
  */

/** @defgroup GPIO_LL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Configure the GPIO mode for a dedicated pin on a dedicated port.
  * @rmtoll
  *  MODER        MODEy         LL_GPIO_SetPinMode
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  *         @arg @ref LL_GPIO_MODE_ALTERNATE
  *         @arg @ref LL_GPIO_MODE_ANALOG
  * @note   I/O mode can be input mode, general-purpose output mode, alternate function mode, or analog mode.
  */
__STATIC_INLINE void LL_GPIO_SetPinMode(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t mode)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->MODER,
                             (GPIO_MODER_MODE0 << (STM32_POSITION_VAL(pin) * 2U)),
                             (mode << (STM32_POSITION_VAL(pin) * 2U)));
}

/**
  * @brief  Return the GPIO mode for a dedicated pin on a dedicated port.
  * @rmtoll
  *  MODER        MODEy         LL_GPIO_GetPinMode
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_MODE_INPUT
  *         @arg @ref LL_GPIO_MODE_OUTPUT
  *         @arg @ref LL_GPIO_MODE_ALTERNATE
  *         @arg @ref LL_GPIO_MODE_ANALOG
  * @note   I/O mode can be input mode, general-purpose output mode, alternate function mode, or analog mode.
  */
__STATIC_INLINE uint32_t LL_GPIO_GetPinMode(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->MODER,
                                   (GPIO_MODER_MODE0 << (STM32_POSITION_VAL(pin) * 2U))) \
                    >> (STM32_POSITION_VAL(pin) * 2U));
}

/**
  * @brief  Configure the GPIO output type for several pins on a dedicated port.
  * @rmtoll
  *  OTYPER       OTy           LL_GPIO_SetPinOutputType
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @param  output_type This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref LL_GPIO_OUTPUT_OPENDRAIN
  * @note   Set the output type when the GPIO pin is in output or
  *         alternate mode. Possible types are push-pull or open-drain.
  */
__STATIC_INLINE void LL_GPIO_SetPinOutputType(GPIO_TypeDef *gpiox, uint32_t pin_mask, uint32_t output_type)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->OTYPER, pin_mask, (pin_mask * output_type));
}

/**
  * @brief  Return the GPIO output type for several pins on a dedicated port.
  * @rmtoll
  *  OTYPER       OTy           LL_GPIO_GetPinOutputType
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref LL_GPIO_OUTPUT_OPENDRAIN
  * @note   Set the output type when the GPIO pin is in output or
  *         alternate mode. Possible types are push-pull or open-drain.
  */
__STATIC_INLINE uint32_t LL_GPIO_GetPinOutputType(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->OTYPER, pin) >> STM32_POSITION_VAL(pin));
}

/**
  * @brief  Configure the GPIO speed for a dedicated pin on a dedicated port.
  * @rmtoll
  *  OSPEEDR      OSPEEDy       LL_GPIO_SetPinSpeed
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  speed This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref LL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref LL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref LL_GPIO_SPEED_FREQ_VERY_HIGH
  * @note   I/O speed can be low, medium, fast, or high speed.
  * @note   Refer to the datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  */
__STATIC_INLINE void LL_GPIO_SetPinSpeed(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t  speed)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (STM32_POSITION_VAL(pin) * 2U)),
                             (speed << (STM32_POSITION_VAL(pin) * 2U)));
}

/**
  * @brief  Return the GPIO speed for a dedicated pin on a dedicated port.
  * @rmtoll
  *  OSPEEDR      OSPEEDy       LL_GPIO_GetPinSpeed
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref LL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref LL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref LL_GPIO_SPEED_FREQ_VERY_HIGH
  * @note   I/O speed can be low, medium, fast, or high speed.
  * @note   Refer to the datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  */
__STATIC_INLINE uint32_t LL_GPIO_GetPinSpeed(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->OSPEEDR,
                                   (GPIO_OSPEEDR_OSPEED0 << (STM32_POSITION_VAL(pin) * 2U))) \
                    >> (STM32_POSITION_VAL(pin) * 2U));
}

/**
  * @brief  Configure GPIO pull-up or pull-down for a dedicated pin on a dedicated port.
  * @rmtoll
  *  PUPDR        PUPDy         LL_GPIO_SetPinPull
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  pull This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  */
__STATIC_INLINE void LL_GPIO_SetPinPull(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t pull)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->PUPDR,
                             (GPIO_PUPDR_PUPD0 << (STM32_POSITION_VAL(pin) * 2U)),
                             (pull << (STM32_POSITION_VAL(pin) * 2U)));
}

/**
  * @brief  Return the GPIO pull-up or pull-down for a dedicated pin on a dedicated port.
  * @rmtoll
  *  PUPDR        PUPDy         LL_GPIO_GetPinPull
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_PULL_NO
  *         @arg @ref LL_GPIO_PULL_UP
  *         @arg @ref LL_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t LL_GPIO_GetPinPull(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->PUPDR,
                                   (GPIO_PUPDR_PUPD0 << (STM32_POSITION_VAL(pin) * 2U))) \
                    >> (STM32_POSITION_VAL(pin) * 2U));
}

/**
  * @brief  Configure GPIO alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @rmtoll
  *  AFRL         AFSELy        LL_GPIO_SetAFPin_0_7
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  * @param  alternate This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  * @note   Possible values are from AF0 to AF15 depending on target.
  */
__STATIC_INLINE void LL_GPIO_SetAFPin_0_7(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t alternate)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->AFR[0], (GPIO_AFRL_AFSEL0 << (STM32_POSITION_VAL(pin) * 4U)),
                             (alternate << (STM32_POSITION_VAL(pin) * 4U)));
}

/**
  * @brief  Return GPIO alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @rmtoll
  *  AFRL         AFSELy        LL_GPIO_GetAFPin_0_7
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  */
__STATIC_INLINE uint32_t LL_GPIO_GetAFPin_0_7(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->AFR[0],
                                   (GPIO_AFRL_AFSEL0 << (STM32_POSITION_VAL(pin) * 4U))) \
                    >> (STM32_POSITION_VAL(pin) * 4U));
}

/**
  * @brief  Configure GPIO alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @rmtoll
  *  AFRH         AFSELy        LL_GPIO_SetAFPin_8_15
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @param  alternate This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  * @note   Possible values are from AF0 to AF15 depending on target.
  */
__STATIC_INLINE void LL_GPIO_SetAFPin_8_15(GPIO_TypeDef *gpiox, uint32_t pin, uint32_t alternate)
{
  STM32_ATOMIC_MODIFY_REG_32(gpiox->AFR[1], (GPIO_AFRH_AFSEL8 << (STM32_POSITION_VAL(pin >> 8U) * 4U)),
                             (alternate << (STM32_POSITION_VAL(pin >> 8U) * 4U)));
}

/**
  * @brief  Return GPIO alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @rmtoll
  *  AFRH         AFSELy        LL_GPIO_GetAFPin_8_15
  * @param  gpiox GPIO Port
  * @param  pin This parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_GPIO_AF_0
  *         @arg @ref LL_GPIO_AF_1
  *         @arg @ref LL_GPIO_AF_2
  *         @arg @ref LL_GPIO_AF_3
  *         @arg @ref LL_GPIO_AF_4
  *         @arg @ref LL_GPIO_AF_5
  *         @arg @ref LL_GPIO_AF_6
  *         @arg @ref LL_GPIO_AF_7
  *         @arg @ref LL_GPIO_AF_8
  *         @arg @ref LL_GPIO_AF_9
  *         @arg @ref LL_GPIO_AF_10
  *         @arg @ref LL_GPIO_AF_11
  *         @arg @ref LL_GPIO_AF_12
  *         @arg @ref LL_GPIO_AF_13
  *         @arg @ref LL_GPIO_AF_14
  *         @arg @ref LL_GPIO_AF_15
  * @note   Possible values are from AF0 to AF15 depending on target.
  */
__STATIC_INLINE uint32_t LL_GPIO_GetAFPin_8_15(const GPIO_TypeDef *gpiox, uint32_t pin)
{
  return (uint32_t)(STM32_READ_BIT(gpiox->AFR[1],
                                   (GPIO_AFRH_AFSEL8 << (STM32_POSITION_VAL(pin >> 8U) * 4U))) \
                    >> (STM32_POSITION_VAL(pin >> 8U) * 4U));
}

/**
  * @brief  Lock configuration of several pins for a dedicated port.
  * @rmtoll
  *  LCKR         LCKK          LL_GPIO_LockPin
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @note   When the lock sequence has been applied on a port bit, the
  *         value of this port bit can no longer be modified until the
  *         next reset.
  * @note   Each lock bit freezes a specific configuration register
  *         (control and alternate function registers).
  */
__STATIC_INLINE void LL_GPIO_LockPin(GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  __IO uint32_t temp;
  STM32_WRITE_REG(gpiox->LCKR, GPIO_LCKR_LCKK | pin_mask);
  STM32_WRITE_REG(gpiox->LCKR, pin_mask);
  STM32_WRITE_REG(gpiox->LCKR, GPIO_LCKR_LCKK | pin_mask);
  /* Read LCKK register. This read is mandatory to complete key lock sequence */
  temp = STM32_READ_REG(gpiox->LCKR);
  (void) temp;
}

/**
  * @brief  Return 1 if all pins passed as parameters of a dedicated port are locked; otherwise return 0.
  * @rmtoll
  *  LCKR         LCKy          LL_GPIO_IsPinLocked
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_GPIO_IsPinLocked(const GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  return ((STM32_READ_BIT(gpiox->LCKR, pin_mask) == (pin_mask)) ? 1UL : 0UL);
}

/**
  * @brief  Return 1 if one of the pins of a dedicated port is locked; otherwise return 0.
  * @rmtoll
  *  LCKR         LCKK          LL_GPIO_IsAnyPinLocked
  * @param  gpiox GPIO Port
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_GPIO_IsAnyPinLocked(const GPIO_TypeDef *gpiox)
{
  return ((STM32_READ_BIT(gpiox->LCKR, GPIO_LCKR_LCKK) == (GPIO_LCKR_LCKK)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup GPIO_LL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value for a dedicated port.
  * @rmtoll
  *  IDR          IDy           LL_GPIO_ReadInputPort
  * @param  gpiox GPIO Port
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t LL_GPIO_ReadInputPort(const GPIO_TypeDef *gpiox)
{
  return (uint32_t)(STM32_READ_REG(gpiox->IDR));
}

/**
  * @brief  Return whether the input data level for several pins of a dedicated port is high or low.
  * @rmtoll
  *  IDR          IDy           LL_GPIO_IsInputPinSet
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_GPIO_IsInputPinSet(const GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  return ((STM32_READ_BIT(gpiox->IDR, pin_mask) == (pin_mask)) ? 1UL : 0UL);
}

/**
  * @brief  Write output data register for the port.
  * @rmtoll
  *  ODR          ODy           LL_GPIO_WriteOutputPort
  * @param  gpiox GPIO Port
  * @param  port_value Level value for each pin of the port
  */
__STATIC_INLINE void LL_GPIO_WriteOutputPort(GPIO_TypeDef *gpiox, uint32_t port_value)
{
  STM32_WRITE_REG(gpiox->ODR, port_value);
}

/**
  * @brief  Return full output data register value for a dedicated port.
  * @rmtoll
  *  ODR          ODy           LL_GPIO_ReadOutputPort
  * @param  gpiox GPIO Port
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t LL_GPIO_ReadOutputPort(const GPIO_TypeDef *gpiox)
{
  return (uint32_t)(STM32_READ_REG(gpiox->ODR));
}

/**
  * @brief  Return whether the output data level for several pins of a dedicated port is high or low.
  * @rmtoll
  *  ODR          ODy           LL_GPIO_IsOutputPinSet
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_GPIO_IsOutputPinSet(const GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  return ((STM32_READ_BIT(gpiox->ODR, pin_mask) == (pin_mask)) ? 1UL : 0UL);
}

/**
  * @brief  Set several pins to high level on dedicated gpio port.
  * @rmtoll
  *  BSRR         BSy           LL_GPIO_SetOutputPin
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  */
__STATIC_INLINE void LL_GPIO_SetOutputPin(GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  STM32_WRITE_REG(gpiox->BSRR, pin_mask);
}

/**
  * @brief  Set several pins to low level on dedicated gpio port.
  * @rmtoll
  *  BRR          BRy           LL_GPIO_ResetOutputPin
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  */
__STATIC_INLINE void LL_GPIO_ResetOutputPin(GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  STM32_WRITE_REG(gpiox->BRR, pin_mask);
}

/**
  * @brief  Set or reset selected pins of a GPIO port.
  * @rmtoll
  *  BSRR         BSy           LL_GPIO_WriteOutputPin \n
  *  BSRR         BRy           LL_GPIO_WriteOutputPin
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @param  state State of bits, this parameter can be one of the following values:
  *         @arg @ref LL_GPIO_PIN_RESET
  *         @arg @ref LL_GPIO_PIN_SET
  */
__STATIC_INLINE void LL_GPIO_WriteOutputPin(GPIO_TypeDef *gpiox, uint32_t pin_mask, uint32_t state)
{
  STM32_WRITE_REG(gpiox->BSRR, (pin_mask << ((1U - (state & 0x1UL)) << 4U)));
}

/**
  * @brief  Set and reset selected pins of a GPIO port atomically.
  * @rmtoll
  *  BSRR         BSy           LL_GPIO_WriteMultipleStatePin \n
  *  BSRR         BRy           LL_GPIO_WriteMultipleStatePin
  * @param  gpiox GPIO Port
  * @param  pins_reset This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @param  pins_set This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  */
__STATIC_INLINE void LL_GPIO_WriteMultipleStatePin(GPIO_TypeDef *gpiox, uint32_t pins_reset, uint32_t pins_set)
{
  STM32_WRITE_REG(gpiox->BSRR, (pins_set & 0xFFFFU) | ((pins_reset & 0xFFFFU) << 16));
}

/**
  * @brief  Toggle data value for several pin of dedicated port.
  * @rmtoll
  *  ODR          ODy           LL_GPIO_TogglePin
  * @param  gpiox GPIO Port
  * @param  pin_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  */
__STATIC_INLINE void LL_GPIO_TogglePin(GPIO_TypeDef *gpiox, uint32_t pin_mask)
{
  uint32_t odr = STM32_READ_REG(gpiox->ODR);
  STM32_WRITE_REG(gpiox->BSRR, ((odr & pin_mask) << 16U) | (~odr & pin_mask));
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

#endif /* GPIOA || GPIOB || GPIOC || GPIOD || GPIOE || GPIOF || GPIOG || GPIOH */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_GPIO_H */
