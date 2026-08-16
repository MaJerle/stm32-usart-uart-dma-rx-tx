/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_gpio.h
  * @brief   Header file of GPIO HAL module.
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
#ifndef STM32C5XX_HAL_GPIO_H
#define STM32C5XX_HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_gpio.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) \
    || defined (GPIOF) ||  defined (GPIOG) || defined (GPIOH)

/** @defgroup GPIO GPIO
  * @{
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/

/** @defgroup GPIO_Exported_Types HAL GPIO Types
  * @{
  */

/**
  * @brief HAL GPIO instance
  */
typedef enum
{
  HAL_GPIOA = GPIOA_BASE,     /*!< GPIO port A. */
  HAL_GPIOB = GPIOB_BASE,     /*!< GPIO port B. */
  HAL_GPIOC = GPIOC_BASE,     /*!< GPIO port C. */
  HAL_GPIOD = GPIOD_BASE,     /*!< GPIO port D. */
  HAL_GPIOE = GPIOE_BASE,     /*!< GPIO port E. */
#if defined(GPIOF) && defined(GPIOG)
  HAL_GPIOF = GPIOF_BASE,     /*!< GPIO port F. */
  HAL_GPIOG = GPIOG_BASE,     /*!< GPIO port G. */
#endif /* GPIOF && GPIOG */
  HAL_GPIOH = GPIOH_BASE,     /*!< GPIO port H. */
} hal_gpio_t;

/**
  * @brief HAL GPIO mode
  */
typedef enum
{
  HAL_GPIO_MODE_INPUT     =  LL_GPIO_MODE_INPUT,      /*!< Input Floating mode. */
  HAL_GPIO_MODE_OUTPUT    =  LL_GPIO_MODE_OUTPUT,     /*!< Output mode.         */
  HAL_GPIO_MODE_ALTERNATE =  LL_GPIO_MODE_ALTERNATE,  /*!< Alternate mode.      */
  HAL_GPIO_MODE_ANALOG    =  LL_GPIO_MODE_ANALOG      /*!< Analog mode.         */
} hal_gpio_mode_t;

/**
  * @brief HAL GPIO Output Type.
  */
typedef enum
{
  HAL_GPIO_OUTPUT_PUSHPULL  = LL_GPIO_OUTPUT_PUSHPULL,   /*!< Select push-pull as output type.  */
  HAL_GPIO_OUTPUT_OPENDRAIN = LL_GPIO_OUTPUT_OPENDRAIN   /*!< Select open-drain as output type. */
} hal_gpio_output_t;

/**
  * @brief HAL GPIO speed.
  * @note  Refer to the device datasheet for the frequency specifications, and the power supply and load conditions
  *        for each speed.
  */
typedef enum
{
  HAL_GPIO_SPEED_FREQ_LOW        = LL_GPIO_SPEED_FREQ_LOW,        /*!< Low speed.       */
  HAL_GPIO_SPEED_FREQ_MEDIUM     = LL_GPIO_SPEED_FREQ_MEDIUM,     /*!< Medium speed.    */
  HAL_GPIO_SPEED_FREQ_HIGH       = LL_GPIO_SPEED_FREQ_HIGH,       /*!< High speed.      */
  HAL_GPIO_SPEED_FREQ_VERY_HIGH  = LL_GPIO_SPEED_FREQ_VERY_HIGH   /*!< Very-high speed. */
} hal_gpio_speed_freq_t;

/**
  * @brief HAL GPIO pull
  */
typedef enum
{
  HAL_GPIO_PULL_NO   = LL_GPIO_PULL_NO,   /*!< No Pull-up or Pull-down activation.  */
  HAL_GPIO_PULL_UP   = LL_GPIO_PULL_UP,   /*!< Pull-up activation.                  */
  HAL_GPIO_PULL_DOWN = LL_GPIO_PULL_DOWN  /*!< Pull-down activation.                */
} hal_gpio_pull_t;

/**
  * @brief HAL GPIO Alternate function.
  */
typedef enum
{
  HAL_GPIO_AF_0  = LL_GPIO_AF_0,  /*!< Select alternate function 0.  */
  HAL_GPIO_AF_1  = LL_GPIO_AF_1,  /*!< Select alternate function 1.  */
  HAL_GPIO_AF_2  = LL_GPIO_AF_2,  /*!< Select alternate function 2.  */
  HAL_GPIO_AF_3  = LL_GPIO_AF_3,  /*!< Select alternate function 3.  */
  HAL_GPIO_AF_4  = LL_GPIO_AF_4,  /*!< Select alternate function 4.  */
  HAL_GPIO_AF_5  = LL_GPIO_AF_5,  /*!< Select alternate function 5.  */
  HAL_GPIO_AF_6  = LL_GPIO_AF_6,  /*!< Select alternate function 6.  */
  HAL_GPIO_AF_7  = LL_GPIO_AF_7,  /*!< Select alternate function 7.  */
  HAL_GPIO_AF_8  = LL_GPIO_AF_8,  /*!< Select alternate function 8.  */
  HAL_GPIO_AF_9  = LL_GPIO_AF_9,  /*!< Select alternate function 9.  */
  HAL_GPIO_AF_10 = LL_GPIO_AF_10, /*!< Select alternate function 10. */
  HAL_GPIO_AF_11 = LL_GPIO_AF_11, /*!< Select alternate function 11. */
  HAL_GPIO_AF_12 = LL_GPIO_AF_12, /*!< Select alternate function 12. */
  HAL_GPIO_AF_13 = LL_GPIO_AF_13, /*!< Select alternate function 13. */
  HAL_GPIO_AF_14 = LL_GPIO_AF_14, /*!< Select alternate function 14. */
  HAL_GPIO_AF_15 = LL_GPIO_AF_15  /*!< Select alternate function 15. */
} hal_gpio_af_t;

/**
  * @brief  GPIO Bit SET and Bit RESET enumeration.
  */
typedef enum
{
  HAL_GPIO_PIN_RESET = LL_GPIO_PIN_RESET,  /*!< Pin state is reset/low. */
  HAL_GPIO_PIN_SET   = LL_GPIO_PIN_SET     /*!< Pin state is set/high.  */
} hal_gpio_pin_state_t;

/**
  * @brief   GPIO Init structure definition.
  */
typedef struct
{
  hal_gpio_mode_t       mode;            /*!< Specifies the operating mode for the selected pins. */
  hal_gpio_pull_t       pull;            /*!< Specifies the Pull-up or Pull-Down activation for the selected pins. */
  hal_gpio_speed_freq_t speed;           /*!< Specifies the speed for the selected pins. */
  hal_gpio_output_t     output_type;     /*!< Specifies the operating output type for the selected pins. */
  hal_gpio_af_t         alternate;       /*!< Specifies the alternate function for the selected pins. */
  hal_gpio_pin_state_t  init_state;      /*!< Specifies the initial state Set or Reset for the selected pins. */
} hal_gpio_config_t;

/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Constants HAL GPIO Constants
  * @{
  */

/** @defgroup GPIO_pins GPIO pins
  * @{
  */

#define HAL_GPIO_PIN_0                 LL_GPIO_PIN_0    /*!< GPIO pin 0.  */
#define HAL_GPIO_PIN_1                 LL_GPIO_PIN_1    /*!< GPIO pin 1.  */
#define HAL_GPIO_PIN_2                 LL_GPIO_PIN_2    /*!< GPIO pin 2.  */
#define HAL_GPIO_PIN_3                 LL_GPIO_PIN_3    /*!< GPIO pin 3.  */
#define HAL_GPIO_PIN_4                 LL_GPIO_PIN_4    /*!< GPIO pin 4.  */
#define HAL_GPIO_PIN_5                 LL_GPIO_PIN_5    /*!< GPIO pin 5.  */
#define HAL_GPIO_PIN_6                 LL_GPIO_PIN_6    /*!< GPIO pin 6.  */
#define HAL_GPIO_PIN_7                 LL_GPIO_PIN_7    /*!< GPIO pin 7.  */
#define HAL_GPIO_PIN_8                 LL_GPIO_PIN_8    /*!< GPIO pin 8.  */
#define HAL_GPIO_PIN_9                 LL_GPIO_PIN_9    /*!< GPIO pin 9.  */
#define HAL_GPIO_PIN_10                LL_GPIO_PIN_10   /*!< GPIO pin 10. */
#define HAL_GPIO_PIN_11                LL_GPIO_PIN_11   /*!< GPIO pin 11. */
#define HAL_GPIO_PIN_12                LL_GPIO_PIN_12   /*!< GPIO pin 12. */
#define HAL_GPIO_PIN_13                LL_GPIO_PIN_13   /*!< GPIO pin 13. */
#define HAL_GPIO_PIN_14                LL_GPIO_PIN_14   /*!< GPIO pin 14. */
#define HAL_GPIO_PIN_15                LL_GPIO_PIN_15   /*!< GPIO pin 15. */
#define HAL_GPIO_PIN_ALL               LL_GPIO_PIN_ALL  /*!< All GPIO pins. */
/**
  * @}
  */

/** @defgroup GPIO_Alternates GPIO Alternates
  * @{
  */
#if (defined (STM32C591xx) || defined (STM32C593xx) || defined (STM32C5A3xx))
/**
  * @brief   AF 0 selection
  */
#define HAL_GPIO_AF0_SWJ                 HAL_GPIO_AF_0 /*!< SWJ (SWD and JTAG)                     */
#define HAL_GPIO_AF0_JTCK                HAL_GPIO_AF_0 /*!< JTAG Test Clock                        */
#define HAL_GPIO_AF0_JTDI                HAL_GPIO_AF_0 /*!< JTAG Test Data In                      */
#define HAL_GPIO_AF0_JTDO                HAL_GPIO_AF_0 /*!< JTAG Test Data Out                     */
#define HAL_GPIO_AF0_JTMS                HAL_GPIO_AF_0 /*!< JTAG Test Mode Select                  */
#define HAL_GPIO_AF0_MCO                 HAL_GPIO_AF_0 /*!< MCO Alternate Function mapping         */
#define HAL_GPIO_AF0_MCO1                HAL_GPIO_AF_0 /*!< MCO1                                   */
#define HAL_GPIO_AF0_MCO2                HAL_GPIO_AF_0 /*!< MCO2                                   */
#define HAL_GPIO_AF0_NJTRST              HAL_GPIO_AF_0 /*!< NJTRST                                 */
#define HAL_GPIO_AF0_PWR                 HAL_GPIO_AF_0 /*!< PWR Alternate Function mapping         */
#define HAL_GPIO_AF0_PWR_CSLEEP          HAL_GPIO_AF_0 /*!< PWR CSLEEP mode                        */
#define HAL_GPIO_AF0_CSLEEP              HAL_GPIO_AF0_PWR_CSLEEP /*!< Alias to PWR_CSLEEP          */
#define HAL_GPIO_AF0_PWR_CSTOP           HAL_GPIO_AF_0 /*!< PWR CSTOP mode                         */
#define HAL_GPIO_AF0_CSTOP               HAL_GPIO_AF0_PWR_CSTOP /*!< Alias to PWR_CSTOP            */
#define HAL_GPIO_AF0_RTC                 HAL_GPIO_AF_0 /*!< RTC Alternate Function mapping         */
#define HAL_GPIO_AF0_RTC_OUT2            HAL_GPIO_AF_0 /*!< RTC Output 2                           */
#define HAL_GPIO_AF0_RTC_REFIN           HAL_GPIO_AF_0 /*!< RTC Reference input                    */
#define HAL_GPIO_AF0_SWCLK               HAL_GPIO_AF_0 /*!< Serial Wire Clock                      */
#define HAL_GPIO_AF0_SWDIO               HAL_GPIO_AF_0 /*!< Serial Wire Debug Input/Output         */
#define HAL_GPIO_AF0_TRACECLK            HAL_GPIO_AF_0 /*!< TRACE Clock                            */
#define HAL_GPIO_AF0_TRACED0             HAL_GPIO_AF_0 /*!< TRACE Data Output 0                    */
#define HAL_GPIO_AF0_TRACED1             HAL_GPIO_AF_0 /*!< TRACE Data Output 1                    */
#define HAL_GPIO_AF0_TRACED2             HAL_GPIO_AF_0 /*!< TRACE Data Output 2                    */
#define HAL_GPIO_AF0_TRACED3             HAL_GPIO_AF_0 /*!< TRACE Data Output 3                    */
#define HAL_GPIO_AF0_TRACESWO            HAL_GPIO_AF_0 /*!< TRACE Serial Wire Output               */

/**
  * @brief   AF 1 selection
  */
#define HAL_GPIO_AF1_LPTIM1              HAL_GPIO_AF_1 /*!< LPTIM1 Alternate Function mapping      */
#define HAL_GPIO_AF1_LPTIM1_CH1          HAL_GPIO_AF_1 /*!< LPTIM1 Multi-purpose Channel 1         */
#define HAL_GPIO_AF1_LPTIM1_CH2          HAL_GPIO_AF_1 /*!< LPTIM1 Multi-purpose Channel 2         */
#define HAL_GPIO_AF1_LPTIM1_ETR          HAL_GPIO_AF_1 /*!< LPTIM1 External trigger input          */
#define HAL_GPIO_AF1_LPTIM1_IN1          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 1 Input                 */
#define HAL_GPIO_AF1_LPTIM1_IN2          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 2 Input                 */
#define HAL_GPIO_AF1_TIM1                HAL_GPIO_AF_1 /*!< TIM1 Alternate Function mapping        */
#define HAL_GPIO_AF1_TIM1_BKIN           HAL_GPIO_AF_1 /*!< TIM1 Break input                       */
#define HAL_GPIO_AF1_TIM1_BKIN2          HAL_GPIO_AF_1 /*!< TIM1 Break input 2                     */
#define HAL_GPIO_AF1_TIM1_CH1            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 1           */
#define HAL_GPIO_AF1_TIM1_CH1N           HAL_GPIO_AF_1 /*!< TIM1 CH1 complementary output          */
#define HAL_GPIO_AF1_TIM1_CH2            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 2           */
#define HAL_GPIO_AF1_TIM1_CH2N           HAL_GPIO_AF_1 /*!< TIM1 CH2 complementary output          */
#define HAL_GPIO_AF1_TIM1_CH3            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 3           */
#define HAL_GPIO_AF1_TIM1_CH3N           HAL_GPIO_AF_1 /*!< TIM1 CH3 complementary output          */
#define HAL_GPIO_AF1_TIM1_CH4            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 4           */
#define HAL_GPIO_AF1_TIM1_CH4N           HAL_GPIO_AF_1 /*!< TIM1 CH4 complementary output          */
#define HAL_GPIO_AF1_TIM1_ETR            HAL_GPIO_AF_1 /*!< TIM1 External trigger input            */
#define HAL_GPIO_AF1_TIM2                HAL_GPIO_AF_1 /*!< TIM2 Alternate Function mapping        */
#define HAL_GPIO_AF1_TIM2_CH1            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 1           */
#define HAL_GPIO_AF1_TIM2_CH2            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 2           */
#define HAL_GPIO_AF1_TIM2_CH3            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 3           */
#define HAL_GPIO_AF1_TIM2_CH4            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 4           */
#define HAL_GPIO_AF1_TIM17               HAL_GPIO_AF_1 /*!< TIM17 Alternate Function mapping       */
#define HAL_GPIO_AF1_TIM17_BKIN          HAL_GPIO_AF_1 /*!< TIM17 Break input                      */
#define HAL_GPIO_AF1_TIM17_CH1           HAL_GPIO_AF_1 /*!< TIM17 Multi-purpose Channel 1          */
#define HAL_GPIO_AF1_TIM17_CH1N          HAL_GPIO_AF_1 /*!< TIM17 CH1 complementary output         */

/**
  * @brief   AF 2 selection
  */
#define HAL_GPIO_AF2_I3C1                HAL_GPIO_AF_2 /*!< I3C1 Alternate Function mapping        */
#define HAL_GPIO_AF2_I3C1_SCL            HAL_GPIO_AF_2 /*!< I3C1 Clock                             */
#define HAL_GPIO_AF2_I3C1_SDA            HAL_GPIO_AF_2 /*!< I3C1 Data                              */
#define HAL_GPIO_AF2_TIM1                HAL_GPIO_AF_2 /*!< TIM1 Alternate Function mapping        */
#define HAL_GPIO_AF2_TIM1_CH2N           HAL_GPIO_AF_2 /*!< TIM1 CH2 complementary output          */
#define HAL_GPIO_AF2_TIM1_CH3            HAL_GPIO_AF_2 /*!< TIM1 Multi-purpose Channel 3           */
#define HAL_GPIO_AF2_TIM3                HAL_GPIO_AF_2 /*!< TIM3 Alternate Function mapping        */
#define HAL_GPIO_AF2_TIM3_CH1            HAL_GPIO_AF_2 /*!< TIM3 Multi-purpose Channel 1           */
#define HAL_GPIO_AF2_TIM3_CH2            HAL_GPIO_AF_2 /*!< TIM3 Multi-purpose Channel 2           */
#define HAL_GPIO_AF2_TIM3_CH3            HAL_GPIO_AF_2 /*!< TIM3 Multi-purpose Channel 3           */
#define HAL_GPIO_AF2_TIM3_CH4            HAL_GPIO_AF_2 /*!< TIM3 Multi-purpose Channel 4           */
#define HAL_GPIO_AF2_TIM3_ETR            HAL_GPIO_AF_2 /*!< TIM3 External trigger input            */
#define HAL_GPIO_AF2_TIM4                HAL_GPIO_AF_2 /*!< TIM4 Alternate Function mapping        */
#define HAL_GPIO_AF2_TIM4_CH1            HAL_GPIO_AF_2 /*!< TIM4 Multi-purpose Channel 1           */
#define HAL_GPIO_AF2_TIM4_CH2            HAL_GPIO_AF_2 /*!< TIM4 Multi-purpose Channel 2           */
#define HAL_GPIO_AF2_TIM4_CH3            HAL_GPIO_AF_2 /*!< TIM4 Multi-purpose Channel 3           */
#define HAL_GPIO_AF2_TIM4_CH4            HAL_GPIO_AF_2 /*!< TIM4 Multi-purpose Channel 4           */
#define HAL_GPIO_AF2_TIM4_ETR            HAL_GPIO_AF_2 /*!< TIM4 External trigger input            */
#define HAL_GPIO_AF2_TIM5                HAL_GPIO_AF_2 /*!< TIM5 Alternate Function mapping        */
#define HAL_GPIO_AF2_TIM5_CH1            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 1           */
#define HAL_GPIO_AF2_TIM5_CH2            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 2           */
#define HAL_GPIO_AF2_TIM5_CH3            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 3           */
#define HAL_GPIO_AF2_TIM5_CH4            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 4           */
#define HAL_GPIO_AF2_TIM5_ETR            HAL_GPIO_AF_2 /*!< TIM5 External trigger input            */
#define HAL_GPIO_AF2_TIM8                HAL_GPIO_AF_2 /*!< TIM8 Alternate Function mapping        */
#define HAL_GPIO_AF2_TIM8_CH1            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 1           */
#define HAL_GPIO_AF2_TIM8_CH2            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 2           */
#define HAL_GPIO_AF2_TIM8_CH3            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 3           */
#define HAL_GPIO_AF2_TIM12               HAL_GPIO_AF_2 /*!< TIM12 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM12_CH1           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 1          */
#define HAL_GPIO_AF2_TIM12_CH2           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 2          */
#define HAL_GPIO_AF2_TIM15               HAL_GPIO_AF_2 /*!< TIM15 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM15_CH1           HAL_GPIO_AF_2 /*!< TIM15 Multi-purpose Channel 1          */

/**
  * @brief   AF 3 selection
  */
#define HAL_GPIO_AF3_I3C1                HAL_GPIO_AF_3 /*!< I3C1 Alternate Function mapping        */
#define HAL_GPIO_AF3_I3C1_SCL            HAL_GPIO_AF_3 /*!< I3C1 Clock                             */
#define HAL_GPIO_AF3_I3C1_SDA            HAL_GPIO_AF_3 /*!< I3C1 Data                              */
#define HAL_GPIO_AF3_LPTIM1              HAL_GPIO_AF_3 /*!< LPTIM1 Alternate Function mapping      */
#define HAL_GPIO_AF3_LPTIM1_CH1          HAL_GPIO_AF_3 /*!< LPTIM1 Multi-purpose Channel 1         */
#define HAL_GPIO_AF3_LPTIM1_ETR          HAL_GPIO_AF_3 /*!< LPTIM1 External trigger input          */
#define HAL_GPIO_AF3_LPTIM1_IN1          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 1 Input                 */
#define HAL_GPIO_AF3_LPTIM1_IN2          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 2 Input                 */
#define HAL_GPIO_AF3_LPUART1             HAL_GPIO_AF_3 /*!< LPUART1 Alternate Function mapping     */
#define HAL_GPIO_AF3_LPUART1_CTS         HAL_GPIO_AF_3 /*!< LPUART1 Clear to send                  */
#define HAL_GPIO_AF3_LPUART1_RTS         HAL_GPIO_AF_3 /*!< LPUART1 Request to send                */
#define HAL_GPIO_AF3_LPUART1_RTS_DE      HAL_GPIO_AF3_LPUART1_RTS /*!< Alias to LPUART1_RTS        */
#define HAL_GPIO_AF3_LPUART1_RX          HAL_GPIO_AF_3 /*!< LPUART1 Serial Data Receive Input      */
#define HAL_GPIO_AF3_LPUART1_TX          HAL_GPIO_AF_3 /*!< LPUART1 Transmit Data Output           */
#define HAL_GPIO_AF3_TIM1                HAL_GPIO_AF_3 /*!< TIM1 Alternate Function mapping        */
#define HAL_GPIO_AF3_TIM1_CH4N           HAL_GPIO_AF_3 /*!< TIM1 CH4 complementary output          */
#define HAL_GPIO_AF3_TIM5                HAL_GPIO_AF_3 /*!< TIM5 Alternate Function mapping        */
#define HAL_GPIO_AF3_TIM5_CH3            HAL_GPIO_AF_3 /*!< TIM5 Multi-purpose Channel 3           */
#define HAL_GPIO_AF3_TIM8                HAL_GPIO_AF_3 /*!< TIM8 Alternate Function mapping        */
#define HAL_GPIO_AF3_TIM8_BKIN           HAL_GPIO_AF_3 /*!< TIM8 Break input                       */
#define HAL_GPIO_AF3_TIM8_BKIN2          HAL_GPIO_AF_3 /*!< TIM8 Break input 2                     */
#define HAL_GPIO_AF3_TIM8_CH1            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 1           */
#define HAL_GPIO_AF3_TIM8_CH1N           HAL_GPIO_AF_3 /*!< TIM8 CH1 complementary output          */
#define HAL_GPIO_AF3_TIM8_CH2            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 2           */
#define HAL_GPIO_AF3_TIM8_CH2N           HAL_GPIO_AF_3 /*!< TIM8 CH2 complementary output          */
#define HAL_GPIO_AF3_TIM8_CH3            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 3           */
#define HAL_GPIO_AF3_TIM8_CH3N           HAL_GPIO_AF_3 /*!< TIM8 CH3 complementary output          */
#define HAL_GPIO_AF3_TIM8_CH4            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 4           */
#define HAL_GPIO_AF3_TIM8_CH4N           HAL_GPIO_AF_3 /*!< TIM8 CH4 complementary output          */
#define HAL_GPIO_AF3_TIM8_ETR            HAL_GPIO_AF_3 /*!< TIM8 External trigger input            */

/**
  * @brief   AF 4 selection
  */
#define HAL_GPIO_AF4_I2C1                HAL_GPIO_AF_4 /*!< I2C1 Alternate Function mapping        */
#define HAL_GPIO_AF4_I2C1_SCL            HAL_GPIO_AF_4 /*!< I2C1 Clock                             */
#define HAL_GPIO_AF4_I2C1_SDA            HAL_GPIO_AF_4 /*!< I2C1 Data                              */
#define HAL_GPIO_AF4_I2C1_SMBA           HAL_GPIO_AF_4 /*!< I2C1 SMBus Alert                       */
#define HAL_GPIO_AF4_I2C2                HAL_GPIO_AF_4 /*!< I2C2 Alternate Function mapping        */
#define HAL_GPIO_AF4_I2C2_SCL            HAL_GPIO_AF_4 /*!< I2C2 Clock                             */
#define HAL_GPIO_AF4_I2C2_SDA            HAL_GPIO_AF_4 /*!< I2C2 Data                              */
#define HAL_GPIO_AF4_I2C2_SMBA           HAL_GPIO_AF_4 /*!< I2C2 SMBus Alert                       */
#define HAL_GPIO_AF4_I2S3                HAL_GPIO_AF_4 /*!< I2S3 Alternate Function mapping        */
#define HAL_GPIO_AF4_I2S3_SCK            HAL_GPIO_AF_4 /*!< I2S3 Clock                             */
#define HAL_GPIO_AF4_I2S3_SDO            HAL_GPIO_AF_4 /*!< I2S3 Serial Data Output                */
#define HAL_GPIO_AF4_I3C1                HAL_GPIO_AF_4 /*!< I3C1 Alternate Function mapping        */
#define HAL_GPIO_AF4_I3C1_SCL            HAL_GPIO_AF_4 /*!< I3C1 Serial Clock                      */
#define HAL_GPIO_AF4_I3C1_SDA            HAL_GPIO_AF_4 /*!< I3C1 Serial Data                       */
#define HAL_GPIO_AF4_LPTIM1              HAL_GPIO_AF_4 /*!< LPTIM1 Alternate Function mapping      */
#define HAL_GPIO_AF4_LPTIM1_CH2          HAL_GPIO_AF_4 /*!< LPTIM1 Multi-purpose Channel 2         */
#define HAL_GPIO_AF4_SPI1                HAL_GPIO_AF_4 /*!< SPI1 Alternate Function mapping        */
#define HAL_GPIO_AF4_SPI1_RDY            HAL_GPIO_AF_4 /*!< SPI1 Master-In/Slave-Out FIFOs status  */
#define HAL_GPIO_AF4_SPI3                HAL_GPIO_AF_4 /*!< SPI3 Alternate Function mapping        */
#define HAL_GPIO_AF4_SPI3_MOSI           HAL_GPIO_AF_4 /*!< SPI3 Master-Out/Slave-In               */
#define HAL_GPIO_AF4_SPI3_SCK            HAL_GPIO_AF_4 /*!< SPI3 Master-Out/Slave-In Clock         */
#define HAL_GPIO_AF4_TIM15               HAL_GPIO_AF_4 /*!< TIM15 Alternate Function mapping       */
#define HAL_GPIO_AF4_TIM15_BKIN          HAL_GPIO_AF_4 /*!< TIM15 Break input                      */
#define HAL_GPIO_AF4_TIM15_CH1           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 1          */
#define HAL_GPIO_AF4_TIM15_CH1N          HAL_GPIO_AF_4 /*!< TIM15 CH1 complementary output         */
#define HAL_GPIO_AF4_TIM15_CH2           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 2          */
#define HAL_GPIO_AF4_USART1              HAL_GPIO_AF_4 /*!< USART1 Alternate Function mapping      */
#define HAL_GPIO_AF4_USART1_RX           HAL_GPIO_AF_4 /*!< USART1 Serial Data Receive Input       */
#define HAL_GPIO_AF4_USART1_TX           HAL_GPIO_AF_4 /*!< USART1 Transmit Data Output            */
#define HAL_GPIO_AF4_USART2              HAL_GPIO_AF_4 /*!< USART2 Alternate Function mapping      */
#define HAL_GPIO_AF4_USART2_RX           HAL_GPIO_AF_4 /*!< USART2 Serial Data Receive Input       */
#define HAL_GPIO_AF4_USART2_TX           HAL_GPIO_AF_4 /*!< USART2 Transmit Data Output            */

/**
  * @brief   AF 5 selection
  */
#define HAL_GPIO_AF5_AUDIOCLK            HAL_GPIO_AF_5 /*!< Audio Clock                            */
#define HAL_GPIO_AF5_I2S1                HAL_GPIO_AF_5 /*!< I2S1 Alternate Function mapping        */
#define HAL_GPIO_AF5_I2S1_CK             HAL_GPIO_AF_5 /*!< I2S1 Serial Clock                      */
#define HAL_GPIO_AF5_I2S1_MCK            HAL_GPIO_AF_5 /*!< I2S1 Master Clock                      */
#define HAL_GPIO_AF5_I2S1_SDI            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Input                 */
#define HAL_GPIO_AF5_I2S1_SDO            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Output                */
#define HAL_GPIO_AF5_I2S1_WS             HAL_GPIO_AF_5 /*!< I2S1 Word Select                       */
#define HAL_GPIO_AF5_I2S2                HAL_GPIO_AF_5 /*!< I2S2 Alternate Function mapping        */
#define HAL_GPIO_AF5_I2S2_CK             HAL_GPIO_AF_5 /*!< I2S2 Serial Clock                      */
#define HAL_GPIO_AF5_I2S2_MCK            HAL_GPIO_AF_5 /*!< I2S2 Master Clock                      */
#define HAL_GPIO_AF5_I2S2_SDI            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Input                 */
#define HAL_GPIO_AF5_I2S2_SDO            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Output                */
#define HAL_GPIO_AF5_I2S2_WS             HAL_GPIO_AF_5 /*!< I2S2 Word Select                       */
#define HAL_GPIO_AF5_I2S3                HAL_GPIO_AF_5 /*!< I2S3 Alternate Function mapping        */
#define HAL_GPIO_AF5_I2S3_CK             HAL_GPIO_AF_5 /*!< I2S3 Serial Clock                      */
#define HAL_GPIO_AF5_I2S3_SDI            HAL_GPIO_AF_5 /*!< I2S3 Serial Data Input                 */
#define HAL_GPIO_AF5_I2S3_SDO            HAL_GPIO_AF_5 /*!< I2S3 Serial Data Output                */
#define HAL_GPIO_AF5_I2S3_WS             HAL_GPIO_AF_5 /*!< I2S3 Word Select                       */
#define HAL_GPIO_AF5_I3C1                HAL_GPIO_AF_5 /*!< I3C1 Alternate Function mapping        */
#define HAL_GPIO_AF5_I3C1_SCL            HAL_GPIO_AF_5 /*!< I3C1 Clock                             */
#define HAL_GPIO_AF5_I3C1_SDA            HAL_GPIO_AF_5 /*!< I3C1 Data                              */
#define HAL_GPIO_AF5_LPTIM1              HAL_GPIO_AF_5 /*!< LPTIM1 Alternate Function mapping      */
#define HAL_GPIO_AF5_LPTIM1_CH1          HAL_GPIO_AF_5 /*!< LPTIM1 Multi-purpose Channel 1         */
#define HAL_GPIO_AF5_LPTIM1_IN1          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 1 Input                 */
#define HAL_GPIO_AF5_LPTIM1_IN2          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 2 Input                 */
#define HAL_GPIO_AF5_SPI1                HAL_GPIO_AF_5 /*!< SPI1 Alternate Function mapping        */
#define HAL_GPIO_AF5_SPI1_MISO           HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out               */
#define HAL_GPIO_AF5_SPI1_MOSI           HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In               */
#define HAL_GPIO_AF5_SPI1_NSS            HAL_GPIO_AF_5 /*!< SPI1 Slave Selection                   */
#define HAL_GPIO_AF5_SPI1_RDY            HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out FIFOs status  */
#define HAL_GPIO_AF5_SPI1_SCK            HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In Clock         */
#define HAL_GPIO_AF5_SPI2                HAL_GPIO_AF_5 /*!< SPI2 Alternate Function mapping        */
#define HAL_GPIO_AF5_SPI2_MISO           HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out               */
#define HAL_GPIO_AF5_SPI2_MOSI           HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In               */
#define HAL_GPIO_AF5_SPI2_NSS            HAL_GPIO_AF_5 /*!< SPI2 Slave Selection                   */
#define HAL_GPIO_AF5_SPI2_RDY            HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out FIFOs status  */
#define HAL_GPIO_AF5_SPI2_SCK            HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In Clock         */
#define HAL_GPIO_AF5_SPI3                HAL_GPIO_AF_5 /*!< SPI3 Alternate Function mapping        */
#define HAL_GPIO_AF5_SPI3_MISO           HAL_GPIO_AF_5 /*!< SPI3 Master-In/Slave-Out               */
#define HAL_GPIO_AF5_SPI3_MOSI           HAL_GPIO_AF_5 /*!< SPI3 Master-Out/Slave-In               */
#define HAL_GPIO_AF5_SPI3_NSS            HAL_GPIO_AF_5 /*!< SPI3 Slave Selection                   */
#define HAL_GPIO_AF5_SPI3_SCK            HAL_GPIO_AF_5 /*!< SPI3 Master-Out/Slave-In Clock         */

/**
  * @brief   AF 6 selection
  */
#define HAL_GPIO_AF6_I2S1                HAL_GPIO_AF_6 /*!< I2S1 Alternate Function mapping        */
#define HAL_GPIO_AF6_I2S1_SDO            HAL_GPIO_AF_6 /*!< I2S1 Serial Data Output                */
#define HAL_GPIO_AF6_I2S2                HAL_GPIO_AF_6 /*!< I2S2 Alternate Function mapping        */
#define HAL_GPIO_AF6_I2S2_CK             HAL_GPIO_AF_6 /*!< I2S2 Serial Clock                      */
#define HAL_GPIO_AF6_I2S2_SDI            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Input                 */
#define HAL_GPIO_AF6_I2S2_SDO            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Output                */
#define HAL_GPIO_AF6_I2S3                HAL_GPIO_AF_6 /*!< I2S3 Alternate Function mapping        */
#define HAL_GPIO_AF6_I2S3_CK             HAL_GPIO_AF_6 /*!< I2S3 Serial Clock                      */
#define HAL_GPIO_AF6_I2S3_MCK            HAL_GPIO_AF_6 /*!< I2S3 Master Clock                      */
#define HAL_GPIO_AF6_I2S3_SDI            HAL_GPIO_AF_6 /*!< I2S3 Serial Data Input                 */
#define HAL_GPIO_AF6_I2S3_SDO            HAL_GPIO_AF_6 /*!< I2S3 Serial Data Output                */
#define HAL_GPIO_AF6_I2S3_WS             HAL_GPIO_AF_6 /*!< I2S3 Word Select                       */
#define HAL_GPIO_AF6_SPI1                HAL_GPIO_AF_6 /*!< SPI1 Alternate Function mapping        */
#define HAL_GPIO_AF6_SPI1_MOSI           HAL_GPIO_AF_6 /*!< SPI1 Master-Out/Slave-In               */
#define HAL_GPIO_AF6_SPI2                HAL_GPIO_AF_6 /*!< SPI2 Alternate Function mapping        */
#define HAL_GPIO_AF6_SPI2_MISO           HAL_GPIO_AF_6 /*!< SPI2 Master-In/Slave-Out               */
#define HAL_GPIO_AF6_SPI2_MOSI           HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In               */
#define HAL_GPIO_AF6_SPI2_SCK            HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In Clock         */
#define HAL_GPIO_AF6_SPI3                HAL_GPIO_AF_6 /*!< SPI3 Alternate Function mapping        */
#define HAL_GPIO_AF6_SPI3_MISO           HAL_GPIO_AF_6 /*!< SPI3 Master-In/Slave-Out               */
#define HAL_GPIO_AF6_SPI3_MOSI           HAL_GPIO_AF_6 /*!< SPI3 Master-Out/Slave-In               */
#define HAL_GPIO_AF6_SPI3_NSS            HAL_GPIO_AF_6 /*!< SPI3 Slave Selection                   */
#define HAL_GPIO_AF6_SPI3_RDY            HAL_GPIO_AF_6 /*!< SPI3 Master-In/Slave-Out FIFOs status  */
#define HAL_GPIO_AF6_SPI3_SCK            HAL_GPIO_AF_6 /*!< SPI3 Master-Out/Slave-In Clock         */
#define HAL_GPIO_AF6_UART4               HAL_GPIO_AF_6 /*!< UART4 Alternate Function mapping       */
#define HAL_GPIO_AF6_UART4_RX            HAL_GPIO_AF_6 /*!< UART4 Serial Data Receive Input        */
#define HAL_GPIO_AF6_UART4_TX            HAL_GPIO_AF_6 /*!< UART4 Transmit Data Output             */

/**
  * @brief   AF 7 selection
  */
#define HAL_GPIO_AF7_I2S2                HAL_GPIO_AF_7 /*!< I2S2 Alternate Function mapping        */
#define HAL_GPIO_AF7_I2S2_SDO            HAL_GPIO_AF_7 /*!< I2S2 Serial Data Output                */
#define HAL_GPIO_AF7_I2S2_WS             HAL_GPIO_AF_7 /*!< I2S2 Word Select                       */
#define HAL_GPIO_AF7_I2S3                HAL_GPIO_AF_7 /*!< I2S3 Alternate Function mapping        */
#define HAL_GPIO_AF7_I2S3_SDO            HAL_GPIO_AF_7 /*!< I2S3 Serial Data Output                */
#define HAL_GPIO_AF7_SPI2                HAL_GPIO_AF_7 /*!< SPI2 Alternate Function mapping        */
#define HAL_GPIO_AF7_SPI2_MOSI           HAL_GPIO_AF_7 /*!< SPI2 Master-Out/Slave-In               */
#define HAL_GPIO_AF7_SPI2_NSS            HAL_GPIO_AF_7 /*!< SPI2 Slave Selection                   */
#define HAL_GPIO_AF7_SPI2_RDY            HAL_GPIO_AF_7 /*!< SPI2 Master-In/Slave-Out FIFOs status  */
#define HAL_GPIO_AF7_SPI3                HAL_GPIO_AF_7 /*!< SPI3 Alternate Function mapping        */
#define HAL_GPIO_AF7_SPI3_MOSI           HAL_GPIO_AF_7 /*!< SPI3 Master-Out/Slave-In               */
#define HAL_GPIO_AF7_UART7               HAL_GPIO_AF_7 /*!< UART7 Alternate Function mapping       */
#define HAL_GPIO_AF7_UART7_CTS           HAL_GPIO_AF_7 /*!< UART7 Clear to send                    */
#define HAL_GPIO_AF7_UART7_RTS           HAL_GPIO_AF_7 /*!< UART7 Request to send/Driver enable    */
#define HAL_GPIO_AF7_UART7_RX            HAL_GPIO_AF_7 /*!< UART7 Serial Data Receive Input        */
#define HAL_GPIO_AF7_UART7_TX            HAL_GPIO_AF_7 /*!< UART7 Transmit Data Output             */
#define HAL_GPIO_AF7_USART1              HAL_GPIO_AF_7 /*!< USART1 Alternate Function mapping      */
#define HAL_GPIO_AF7_USART1_CK           HAL_GPIO_AF_7 /*!< USART1 Clock                           */
#define HAL_GPIO_AF7_USART1_CTS          HAL_GPIO_AF_7 /*!< USART1 Clear to send                   */
#define HAL_GPIO_AF7_USART1_NSS          HAL_GPIO_AF_7 /*!< USART1 Slave Selection                 */
#define HAL_GPIO_AF7_USART1_RTS          HAL_GPIO_AF_7 /*!< USART1 Request to send                 */
#define HAL_GPIO_AF7_USART1_RTS_DE       HAL_GPIO_AF7_USART1_RTS /*!< Alias to USART1_RTS          */
#define HAL_GPIO_AF7_USART1_RX           HAL_GPIO_AF_7 /*!< USART1 Serial Data Receive Input       */
#define HAL_GPIO_AF7_USART1_TX           HAL_GPIO_AF_7 /*!< USART1 Transmit Data Output            */
#define HAL_GPIO_AF7_USART2              HAL_GPIO_AF_7 /*!< USART2 Alternate Function mapping      */
#define HAL_GPIO_AF7_USART2_CK           HAL_GPIO_AF_7 /*!< USART2 Clock                           */
#define HAL_GPIO_AF7_USART2_CTS          HAL_GPIO_AF_7 /*!< USART2 Clear to send                   */
#define HAL_GPIO_AF7_USART2_NSS          HAL_GPIO_AF_7 /*!< USART2 Slave Selection                 */
#define HAL_GPIO_AF7_USART2_RTS          HAL_GPIO_AF_7 /*!< USART2 Request to send                 */
#define HAL_GPIO_AF7_USART2_RTS_DE       HAL_GPIO_AF7_USART2_RTS /*!< Alias to USART2_RTS          */
#define HAL_GPIO_AF7_USART2_RX           HAL_GPIO_AF_7 /*!< USART2 Serial Data Receive Input       */
#define HAL_GPIO_AF7_USART2_TX           HAL_GPIO_AF_7 /*!< USART2 Transmit Data Output            */
#define HAL_GPIO_AF7_USART3              HAL_GPIO_AF_7 /*!< USART3 Alternate Function mapping      */
#define HAL_GPIO_AF7_USART3_CK           HAL_GPIO_AF_7 /*!< USART3 Clock                           */
#define HAL_GPIO_AF7_USART3_CTS          HAL_GPIO_AF_7 /*!< USART3 Clear to send                   */
#define HAL_GPIO_AF7_USART3_NSS          HAL_GPIO_AF_7 /*!< USART3 Slave Selection                 */
#define HAL_GPIO_AF7_USART3_RTS          HAL_GPIO_AF_7 /*!< USART3 Request to send                 */
#define HAL_GPIO_AF7_USART3_RTS_DE       HAL_GPIO_AF7_USART3_RTS /*!< Alias to USART3_RTS          */
#define HAL_GPIO_AF7_USART3_RX           HAL_GPIO_AF_7 /*!< USART3 Serial Data Receive Input       */
#define HAL_GPIO_AF7_USART3_TX           HAL_GPIO_AF_7 /*!< USART3 Transmit Data Output            */
#define HAL_GPIO_AF7_USART6              HAL_GPIO_AF_7 /*!< USART6 Alternate Function mapping      */
#define HAL_GPIO_AF7_USART6_CK           HAL_GPIO_AF_7 /*!< USART6 Clock                           */
#define HAL_GPIO_AF7_USART6_CTS          HAL_GPIO_AF_7 /*!< USART6 Clear to send                   */
#define HAL_GPIO_AF7_USART6_NSS          HAL_GPIO_AF_7 /*!< USART6 Slave Selection                 */
#define HAL_GPIO_AF7_USART6_RTS          HAL_GPIO_AF_7 /*!< USART6 Request to send                 */
#define HAL_GPIO_AF7_USART6_RX           HAL_GPIO_AF_7 /*!< USART6 Serial Data Receive Input       */
#define HAL_GPIO_AF7_USART6_TX           HAL_GPIO_AF_7 /*!< USART6 Transmit Data Output            */
#define HAL_GPIO_AF7_XSPI1               HAL_GPIO_AF_7 /*!< XSPI1 Alternate Function mapping       */
#define HAL_GPIO_AF7_XSPI1_IO0           HAL_GPIO_AF_7 /*!< XSPI1 Data pin 0                       */
#define HAL_GPIO_AF7_XSPI1_IO6           HAL_GPIO_AF_7 /*!< XSPI1 Data pin 6                       */

/**
  * @brief   AF 8 selection
  */
#define HAL_GPIO_AF8_I2C2                HAL_GPIO_AF_8 /*!< I2C2 Alternate Function mapping        */
#define HAL_GPIO_AF8_I2C2_SCL            HAL_GPIO_AF_8 /*!< I2C2 Clock                             */
#define HAL_GPIO_AF8_I2C2_SDA            HAL_GPIO_AF_8 /*!< I2C2 Data                              */
#define HAL_GPIO_AF8_LPUART1             HAL_GPIO_AF_8 /*!< LPUART1 Alternate Function mapping     */
#define HAL_GPIO_AF8_LPUART1_CTS         HAL_GPIO_AF_8 /*!< LPUART1 Clear to send                  */
#define HAL_GPIO_AF8_LPUART1_RTS         HAL_GPIO_AF_8 /*!< LPUART1 Request to send                */
#define HAL_GPIO_AF8_LPUART1_RTS_DE      HAL_GPIO_AF8_LPUART1_RTS /*!< Alias to LPUART1_RTS        */
#define HAL_GPIO_AF8_LPUART1_RX          HAL_GPIO_AF_8 /*!< LPUART1 Serial Data Receive Input      */
#define HAL_GPIO_AF8_LPUART1_TX          HAL_GPIO_AF_8 /*!< LPUART1 Transmit Data Output           */
#define HAL_GPIO_AF8_TIM5                HAL_GPIO_AF_8 /*!< TIM5 Alternate Function mapping        */
#define HAL_GPIO_AF8_TIM5_CH4            HAL_GPIO_AF_8 /*!< TIM5 Multi-purpose Channel 4           */
#define HAL_GPIO_AF8_TIM5_ETR            HAL_GPIO_AF_8 /*!< TIM5 External trigger input            */
#define HAL_GPIO_AF8_UART4               HAL_GPIO_AF_8 /*!< UART4 Alternate Function mapping       */
#define HAL_GPIO_AF8_UART4_CTS           HAL_GPIO_AF_8 /*!< UART4 Clear to send                    */
#define HAL_GPIO_AF8_UART4_RTS           HAL_GPIO_AF_8 /*!< UART4 Request to send                  */
#define HAL_GPIO_AF8_UART4_RTS_DE        HAL_GPIO_AF8_UART4_RTS /*!< Alias to UART4_RTS            */
#define HAL_GPIO_AF8_UART4_RX            HAL_GPIO_AF_8 /*!< UART4 Serial Data Receive Input        */
#define HAL_GPIO_AF8_UART4_TX            HAL_GPIO_AF_8 /*!< UART4 Transmit Data Output             */
#define HAL_GPIO_AF8_UART5               HAL_GPIO_AF_8 /*!< UART5 Alternate Function mapping       */
#define HAL_GPIO_AF8_UART5_CTS           HAL_GPIO_AF_8 /*!< UART5 Clear to send                    */
#define HAL_GPIO_AF8_UART5_RTS           HAL_GPIO_AF_8 /*!< UART5 Request to send                  */
#define HAL_GPIO_AF8_UART5_RTS_DE        HAL_GPIO_AF8_UART5_RTS /*!< Alias to UART5_RTS            */
#define HAL_GPIO_AF8_UART5_RX            HAL_GPIO_AF_8 /*!< UART5 Serial Data Receive Input        */
#define HAL_GPIO_AF8_UART5_TX            HAL_GPIO_AF_8 /*!< UART5 Transmit Data Output             */

/**
  * @brief   AF 9 selection
  */
#define HAL_GPIO_AF9_FDCAN1              HAL_GPIO_AF_9 /*!< FDCAN1 Alternate Function mapping      */
#define HAL_GPIO_AF9_FDCAN1_RX           HAL_GPIO_AF_9 /*!< FDCAN1 Receive pin                     */
#define HAL_GPIO_AF9_FDCAN1_TX           HAL_GPIO_AF_9 /*!< FDCAN1 Transmit pin                    */
#define HAL_GPIO_AF9_FDCAN2              HAL_GPIO_AF_9 /*!< FDCAN2 Alternate Function mapping      */
#define HAL_GPIO_AF9_FDCAN2_RX           HAL_GPIO_AF_9 /*!< FDCAN2 Receive pin                     */
#define HAL_GPIO_AF9_FDCAN2_TX           HAL_GPIO_AF_9 /*!< FDCAN2 Transmit pin                    */
#define HAL_GPIO_AF9_I2C1                HAL_GPIO_AF_9 /*!< I2C1 Alternate Function mapping        */
#define HAL_GPIO_AF9_I2C1_SCL            HAL_GPIO_AF_9 /*!< I2C1 Clock                             */
#define HAL_GPIO_AF9_I2C1_SDA            HAL_GPIO_AF_9 /*!< I2C1 Data                              */
#define HAL_GPIO_AF9_I2C1_SMBA           HAL_GPIO_AF_9 /*!< I2C1 SMBus Alert                       */
#define HAL_GPIO_AF9_I2C2                HAL_GPIO_AF_9 /*!< I2C2 Alternate Function mapping        */
#define HAL_GPIO_AF9_I2C2_SCL            HAL_GPIO_AF_9 /*!< I2C2 Clock                             */
#define HAL_GPIO_AF9_I2C2_SDA            HAL_GPIO_AF_9 /*!< I2C2 Data                              */
#define HAL_GPIO_AF9_I2S2                HAL_GPIO_AF_9 /*!< I2S2 Alternate Function mapping        */
#define HAL_GPIO_AF9_I2S2_WS             HAL_GPIO_AF_9 /*!< I2S2 Word Select                       */
#define HAL_GPIO_AF9_SPI2                HAL_GPIO_AF_9 /*!< SPI2 Alternate Function mapping        */
#define HAL_GPIO_AF9_SPI2_NSS            HAL_GPIO_AF_9 /*!< SPI2 Slave Selection                   */
#define HAL_GPIO_AF9_XSPI1               HAL_GPIO_AF_9 /*!< XSPI1 Alternate Function mapping       */
#define HAL_GPIO_AF9_XSPI1_CLK           HAL_GPIO_AF_9 /*!< XSPI1 Clock                            */
#define HAL_GPIO_AF9_XSPI1_IO0           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 0                       */
#define HAL_GPIO_AF9_XSPI1_IO1           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 1                       */
#define HAL_GPIO_AF9_XSPI1_IO2           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 2                       */
#define HAL_GPIO_AF9_XSPI1_IO3           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 3                       */
#define HAL_GPIO_AF9_XSPI1_IO4           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 4                       */
#define HAL_GPIO_AF9_XSPI1_IO5           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 5                       */
#define HAL_GPIO_AF9_XSPI1_IO6           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 6                       */
#define HAL_GPIO_AF9_XSPI1_IO7           HAL_GPIO_AF_9 /*!< XSPI1 Data pin 7                       */
#define HAL_GPIO_AF9_XSPI1_NCLK          HAL_GPIO_AF_9 /*!< XSPI1 Inverted Clock                   */
#define HAL_GPIO_AF9_XSPI1_NCS1          HAL_GPIO_AF_9 /*!< XSPI1 Memory Chip Select 1             */
#define HAL_GPIO_AF9_XSPI1_NCS2          HAL_GPIO_AF_9 /*!< XSPI1 Memory Chip Select 2             */

/**
  * @brief   AF 10 selection
  */
#define HAL_GPIO_AF10_CRS                HAL_GPIO_AF_10 /*!< CRS Alternate Function mapping        */
#define HAL_GPIO_AF10_CRS_SYNC           HAL_GPIO_AF_10 /*!< CRS synchronization                   */
#define HAL_GPIO_AF10_ETH1               HAL_GPIO_AF_10 /*!< ETH1 Alternate Function mapping       */
#define HAL_GPIO_AF10_ETH1_CLK           HAL_GPIO_AF_10 /*!< ETH1 Clock signal for synchronization */
#define HAL_GPIO_AF10_ETH1_MDC           HAL_GPIO_AF_10 /*!< ETH1 Management Data Clock            */
#define HAL_GPIO_AF10_ETH1_MDIO          HAL_GPIO_AF_10 /*!< ETH1 Management Data Input/Output     */
#define HAL_GPIO_AF10_ETH1_MII_COL       HAL_GPIO_AF_10 /*!< ETH1 Collision detect signal in MII   */
#define HAL_GPIO_AF10_ETH1_MII_CRS       HAL_GPIO_AF_10 /*!< ETH1 Carrier Sense signal in MII      */
#define HAL_GPIO_AF10_ETH1_MII_RX_CLK    HAL_GPIO_AF_10 /*!< ETH1 Receive clock signal in MII      */
#define HAL_GPIO_AF10_ETH1_MII_RX_DV     HAL_GPIO_AF_10 /*!< ETH1 Receive Data Valid signal in MII */
#define HAL_GPIO_AF10_ETH1_MII_RX_ER     HAL_GPIO_AF_10 /*!< ETH1 Receive Error signal in MII      */
#define HAL_GPIO_AF10_ETH1_MII_RXD0      HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 0 in MII        */
#define HAL_GPIO_AF10_ETH1_MII_RXD1      HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 1 in MII        */
#define HAL_GPIO_AF10_ETH1_MII_RXD2      HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 2 in MII        */
#define HAL_GPIO_AF10_ETH1_MII_RXD3      HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 3 in MII        */
#define HAL_GPIO_AF10_ETH1_MII_TX_CLK    HAL_GPIO_AF_10 /*!< ETH1 Transmit clock signal in MII     */
#define HAL_GPIO_AF10_ETH1_MII_TX_EN     HAL_GPIO_AF_10 /*!< ETH1 Transmit Enable signal in MII    */
#define HAL_GPIO_AF10_ETH1_MII_TX_ER     HAL_GPIO_AF_10 /*!< ETH1 Transmit Error signal in MII     */
#define HAL_GPIO_AF10_ETH1_MII_TXD0      HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 0 in MII       */
#define HAL_GPIO_AF10_ETH1_MII_TXD1      HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 1 in MII       */
#define HAL_GPIO_AF10_ETH1_MII_TXD2      HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 2 in MII       */
#define HAL_GPIO_AF10_ETH1_MII_TXD3      HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 3 in MII       */
#define HAL_GPIO_AF10_ETH1_PHY_INTN      HAL_GPIO_AF_10 /*!< ETH1 Interrupt signal from PHY        */
#define HAL_GPIO_AF10_ETH1_PPS_OUT       HAL_GPIO_AF_10 /*!< ETH1 Pulse Per Second output          */
#define HAL_GPIO_AF10_ETH1_PTP_AUX_TS    HAL_GPIO_AF_10 /*!< ETH1 Auxiliary timestamp              */
#define HAL_GPIO_AF10_ETH1_RMII_CRS_DV   HAL_GPIO_AF_10 /*!< ETH1 Carrier Sense and Data Valid     */
#define HAL_GPIO_AF10_ETH1_RMII_REF_CLK  HAL_GPIO_AF_10 /*!< ETH1 Reference clock signal in RMII   */
#define HAL_GPIO_AF10_ETH1_RMII_RXD0     HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 0 in RMII       */
#define HAL_GPIO_AF10_ETH1_RMII_RXD1     HAL_GPIO_AF_10 /*!< ETH1 Receive Data bit 1 in RMII       */
#define HAL_GPIO_AF10_ETH1_RMII_TX_EN    HAL_GPIO_AF_10 /*!< ETH1 Transmit Enable signal in RMII   */
#define HAL_GPIO_AF10_ETH1_RMII_TXD0     HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 0 in RMII      */
#define HAL_GPIO_AF10_ETH1_RMII_TXD1     HAL_GPIO_AF_10 /*!< ETH1 Transmit Data bit 1 in RMII      */
#define HAL_GPIO_AF10_TIM16              HAL_GPIO_AF_10 /*!< TIM16 Alternate Function mapping      */
#define HAL_GPIO_AF10_TIM16_BKIN         HAL_GPIO_AF_10 /*!< TIM16 Break input                     */
#define HAL_GPIO_AF10_TIM16_CH1          HAL_GPIO_AF_10 /*!< TIM16 Multi-purpose Channel 1         */
#define HAL_GPIO_AF10_TIM16_CH1N         HAL_GPIO_AF_10 /*!< TIM16 CH1 complementary output        */
#define HAL_GPIO_AF10_UART7              HAL_GPIO_AF_10 /*!< UART7 Alternate Function mapping      */
#define HAL_GPIO_AF10_UART7_RX           HAL_GPIO_AF_10 /*!< UART7 Serial Data Receive Input       */
#define HAL_GPIO_AF10_UART7_TX           HAL_GPIO_AF_10 /*!< UART7 Transmit Data Output            */

/**
  * @brief   AF 11 selection
  */
#define HAL_GPIO_AF11_USART1             HAL_GPIO_AF_11 /*!< USART1 Alternate Function mapping     */
#define HAL_GPIO_AF11_USART1_TX          HAL_GPIO_AF_11 /*!< USART1 Transmit Data Output           */
#define HAL_GPIO_AF11_USART3             HAL_GPIO_AF_11 /*!< USART3 Alternate Function mapping     */
#define HAL_GPIO_AF11_USART3_CK          HAL_GPIO_AF_11 /*!< USART3 Clock                          */
#define HAL_GPIO_AF11_USART3_CTS         HAL_GPIO_AF_11 /*!< USART3 Clear to send                  */
#define HAL_GPIO_AF11_USART3_NSS         HAL_GPIO_AF_11 /*!< USART3 Slave Selection                */
#define HAL_GPIO_AF11_USART3_RTS         HAL_GPIO_AF_11 /*!< USART3 Request to send                */
#define HAL_GPIO_AF11_USART3_RTS_DE      HAL_GPIO_AF11_USART3_RTS /*!< Alias to USART3_RTS         */
#define HAL_GPIO_AF11_USART3_RX          HAL_GPIO_AF_11 /*!< USART3 Serial Data Receive Input      */
#define HAL_GPIO_AF11_USART3_TX          HAL_GPIO_AF_11 /*!< USART3 Transmit Data Output           */
#define HAL_GPIO_AF11_XSPI1              HAL_GPIO_AF_11 /*!< XSPI1 Alternate Function mapping      */
#define HAL_GPIO_AF11_XSPI1_CLK          HAL_GPIO_AF_11 /*!< XSPI1 Clock                           */
#define HAL_GPIO_AF11_XSPI1_DQS          HAL_GPIO_AF_11 /*!< XSPI1 Memory Data Strobe I/O          */
#define HAL_GPIO_AF11_XSPI1_IO0          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 0                      */
#define HAL_GPIO_AF11_XSPI1_IO1          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 1                      */
#define HAL_GPIO_AF11_XSPI1_IO2          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 2                      */
#define HAL_GPIO_AF11_XSPI1_IO3          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 3                      */
#define HAL_GPIO_AF11_XSPI1_IO4          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 4                      */
#define HAL_GPIO_AF11_XSPI1_IO5          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 5                      */
#define HAL_GPIO_AF11_XSPI1_IO6          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 6                      */
#define HAL_GPIO_AF11_XSPI1_IO7          HAL_GPIO_AF_11 /*!< XSPI1 Data pin 7                      */
#define HAL_GPIO_AF11_XSPI1_NCLK         HAL_GPIO_AF_11 /*!< XSPI1 Inverted Clock                  */
#define HAL_GPIO_AF11_XSPI1_NCS1         HAL_GPIO_AF_11 /*!< XSPI1 Memory Chip Select 1            */
#define HAL_GPIO_AF11_XSPI1_NCS2         HAL_GPIO_AF_11 /*!< XSPI1 Memory Chip Select 2            */

/**
  * @brief   AF 12 selection
  */
#define HAL_GPIO_AF12_ETH1               HAL_GPIO_AF_12 /*!< ETH1 Alternate Function mapping       */
#define HAL_GPIO_AF12_ETH1_MDC           HAL_GPIO_AF_12 /*!< ETH1 Management Data Clock            */
#define HAL_GPIO_AF12_ETH1_MDIO          HAL_GPIO_AF_12 /*!< ETH1 Management Data Input/Output     */
#define HAL_GPIO_AF12_ETH1_MII_RXD0      HAL_GPIO_AF_12 /*!< ETH1 Receive Data bit 0 in MII        */
#define HAL_GPIO_AF12_ETH1_MII_TX_ER     HAL_GPIO_AF_12 /*!< ETH1 Transmit Error signal in MII     */
#define HAL_GPIO_AF12_ETH1_RMII_RXD0     HAL_GPIO_AF_12 /*!< ETH1 Receive Data bit 0 in RMII       */

/**
  * @brief   AF 13 selection
  */
#define HAL_GPIO_AF13_ETH1               HAL_GPIO_AF_13 /*!< ETH1 Alternate Function mapping       */
#define HAL_GPIO_AF13_ETH1_CLK           HAL_GPIO_AF_13 /*!< ETH1 Clock signal for synchronization */
#define HAL_GPIO_AF13_ETH1_MII_RX_ER     HAL_GPIO_AF_13 /*!< ETH1 Receive Error signal in MII      */
#define HAL_GPIO_AF13_ETH1_MII_RXD0      HAL_GPIO_AF_13 /*!< ETH1 Receive Data bit 0 in MII        */
#define HAL_GPIO_AF13_ETH1_MII_RXD1      HAL_GPIO_AF_13 /*!< ETH1 Receive Data bit 1 in MII        */
#define HAL_GPIO_AF13_ETH1_MII_TX_ER     HAL_GPIO_AF_13 /*!< ETH1 Transmit Error signal in MII     */
#define HAL_GPIO_AF13_ETH1_RMII_RXD0     HAL_GPIO_AF_13 /*!< ETH1 Receive Data bit 0 in RMII       */
#define HAL_GPIO_AF13_ETH1_RMII_RXD1     HAL_GPIO_AF_13 /*!< ETH1 Receive Data bit 1 in RMII       */
#define HAL_GPIO_AF13_TIM8               HAL_GPIO_AF_13 /*!< TIM8 Alternate Function mapping       */
#define HAL_GPIO_AF13_TIM8_CH1           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 1          */
#define HAL_GPIO_AF13_TIM8_CH2           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 2          */
#define HAL_GPIO_AF13_TIM8_CH2N          HAL_GPIO_AF_13 /*!< TIM8 CH2 complementary output         */
#define HAL_GPIO_AF13_TIM8_CH3           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 3          */
#define HAL_GPIO_AF13_TIM8_CH4           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 4          */
#define HAL_GPIO_AF13_TIM8_CH4N          HAL_GPIO_AF_13 /*!< TIM8 CH4 complementary output         */
#define HAL_GPIO_AF13_USB                HAL_GPIO_AF_13 /*!< USB Alternate Function mapping        */
#define HAL_GPIO_AF13_USB_SOF            HAL_GPIO_AF_13 /*!< USB Start of Frame                    */

/**
  * @brief   AF 14 selection
  */
#define HAL_GPIO_AF14_COMP1              HAL_GPIO_AF_14 /*!< COMP1 Alternate Function mapping     */
#define HAL_GPIO_AF14_COMP1_OUT          HAL_GPIO_AF_14 /*!< COMP1 Output channel                 */
#define HAL_GPIO_AF14_TIM2               HAL_GPIO_AF_14 /*!< TIM2 Alternate Function mapping      */
#define HAL_GPIO_AF14_TIM2_CH4           HAL_GPIO_AF_14 /*!< TIM2 Multi-purpose Channel 4         */
#define HAL_GPIO_AF14_TIM2_ETR           HAL_GPIO_AF_14 /*!< TIM2 External trigger input          */
#define HAL_GPIO_AF14_UART5              HAL_GPIO_AF_14 /*!< UART5 Alternate Function mapping     */
#define HAL_GPIO_AF14_UART5_CTS          HAL_GPIO_AF_14 /*!< UART5 Clear to send                  */
#define HAL_GPIO_AF14_UART5_RTS          HAL_GPIO_AF_14 /*!< UART5 Request to send                */
#define HAL_GPIO_AF14_UART5_RTS_DE       HAL_GPIO_AF14_UART5_RTS /*!< Alias to UART5_RTS          */
#define HAL_GPIO_AF14_UART5_RX           HAL_GPIO_AF_14 /*!< UART5 Serial Data Receive Input      */
#define HAL_GPIO_AF14_UART5_TX           HAL_GPIO_AF_14 /*!< UART5 Transmit Data Output           */

/**
  * @brief   AF 15 selection
  */
#define HAL_GPIO_AF15_EVENTOUT           HAL_GPIO_AF_15 /*!< EVENTOUT Alternate Function mapping  */
#endif /* STM32C591xx || STM32C593xx || STM32C5A3xx */
#if (defined (STM32C551xx) || defined (STM32C552xx) || defined (STM32C562xx))
/**
  * @brief   AF 0 selection
  */
#define HAL_GPIO_AF0_SWJ                 HAL_GPIO_AF_0 /*!< SWJ (SWD and JTAG)                     */
#define HAL_GPIO_AF0_JTCK                HAL_GPIO_AF_0 /*!< JTAG Test Clock                        */
#define HAL_GPIO_AF0_JTDI                HAL_GPIO_AF_0 /*!< JTAG Test Data In                      */
#define HAL_GPIO_AF0_JTDO                HAL_GPIO_AF_0 /*!< JTAG Test Data Out                     */
#define HAL_GPIO_AF0_JTMS                HAL_GPIO_AF_0 /*!< JTAG Test Mode Select                  */
#define HAL_GPIO_AF0_MCO                 HAL_GPIO_AF_0 /*!< MCO Alternate Function mapping         */
#define HAL_GPIO_AF0_MCO1                HAL_GPIO_AF_0 /*!< MCO1                                   */
#define HAL_GPIO_AF0_MCO2                HAL_GPIO_AF_0 /*!< MCO2                                   */
#define HAL_GPIO_AF0_NJTRST              HAL_GPIO_AF_0 /*!< NJTRST                                 */
#define HAL_GPIO_AF0_PWR                 HAL_GPIO_AF_0 /*!< PWR Alternate Function mapping         */
#define HAL_GPIO_AF0_PWR_CSLEEP          HAL_GPIO_AF_0 /*!< PWR CSLEEP mode                        */
#define HAL_GPIO_AF0_CSLEEP              HAL_GPIO_AF0_PWR_CSLEEP /*!< Alias to PWR_CSLEEP          */
#define HAL_GPIO_AF0_PWR_CSTOP           HAL_GPIO_AF_0 /*!< PWR CSTOP mode                         */
#define HAL_GPIO_AF0_CSTOP               HAL_GPIO_AF0_PWR_CSTOP /*!< Alias to PWR_CSTOP            */
#define HAL_GPIO_AF0_RTC                 HAL_GPIO_AF_0 /*!< RTC Alternate Function mapping         */
#define HAL_GPIO_AF0_RTC_OUT2            HAL_GPIO_AF_0 /*!< RTC Output 2                           */
#define HAL_GPIO_AF0_RTC_REFIN           HAL_GPIO_AF_0 /*!< RTC Reference input                    */
#define HAL_GPIO_AF0_SWCLK               HAL_GPIO_AF_0 /*!< Serial Wire Clock                      */
#define HAL_GPIO_AF0_SWDIO               HAL_GPIO_AF_0 /*!< Serial Wire Debug Input/Output         */
#define HAL_GPIO_AF0_TRACECLK            HAL_GPIO_AF_0 /*!< TRACE Clock                            */
#define HAL_GPIO_AF0_TRACED0             HAL_GPIO_AF_0 /*!< TRACE Data Output 0                    */
#define HAL_GPIO_AF0_TRACED1             HAL_GPIO_AF_0 /*!< TRACE Data Output 1                    */
#define HAL_GPIO_AF0_TRACED2             HAL_GPIO_AF_0 /*!< TRACE Data Output 2                    */
#define HAL_GPIO_AF0_TRACED3             HAL_GPIO_AF_0 /*!< TRACE Data Output 3                    */
#define HAL_GPIO_AF0_TRACESWO            HAL_GPIO_AF_0 /*!< TRACE Serial Wire Output               */

/**
  * @brief   AF 1 selection
  */
#define HAL_GPIO_AF1_LPTIM1              HAL_GPIO_AF_1 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF1_LPTIM1_CH1          HAL_GPIO_AF_1 /*!< LPTIM1 Multi-purpose Channel 1        */
#define HAL_GPIO_AF1_LPTIM1_CH2          HAL_GPIO_AF_1 /*!< LPTIM1 Multi-purpose Channel 2        */
#define HAL_GPIO_AF1_LPTIM1_ETR          HAL_GPIO_AF_1 /*!< LPTIM1 External trigger input         */
#define HAL_GPIO_AF1_LPTIM1_IN1          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF1_LPTIM1_IN2          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF1_TIM1                HAL_GPIO_AF_1 /*!< TIM1 Alternate Function mapping       */
#define HAL_GPIO_AF1_TIM1_BKIN           HAL_GPIO_AF_1 /*!< TIM1 Break input                      */
#define HAL_GPIO_AF1_TIM1_BKIN2          HAL_GPIO_AF_1 /*!< TIM1 Break input 2                    */
#define HAL_GPIO_AF1_TIM1_CH1            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 1          */
#define HAL_GPIO_AF1_TIM1_CH1N           HAL_GPIO_AF_1 /*!< TIM1 CH1 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH2            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 2          */
#define HAL_GPIO_AF1_TIM1_CH2N           HAL_GPIO_AF_1 /*!< TIM1 CH2 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH3            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 3          */
#define HAL_GPIO_AF1_TIM1_CH3N           HAL_GPIO_AF_1 /*!< TIM1 CH3 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH4            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 4          */
#define HAL_GPIO_AF1_TIM1_CH4N           HAL_GPIO_AF_1 /*!< TIM1 CH4 complementary output         */
#define HAL_GPIO_AF1_TIM1_ETR            HAL_GPIO_AF_1 /*!< TIM1 External trigger input           */
#define HAL_GPIO_AF1_TIM2                HAL_GPIO_AF_1 /*!< TIM2 Alternate Function mapping       */
#define HAL_GPIO_AF1_TIM2_CH1            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 1          */
#define HAL_GPIO_AF1_TIM2_CH2            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 2          */
#define HAL_GPIO_AF1_TIM2_CH3            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 3          */
#define HAL_GPIO_AF1_TIM2_CH4            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 4          */
#define HAL_GPIO_AF1_TIM17               HAL_GPIO_AF_1 /*!< TIM17 Alternate Function mapping      */
#define HAL_GPIO_AF1_TIM17_BKIN          HAL_GPIO_AF_1 /*!< TIM17 Break input                     */
#define HAL_GPIO_AF1_TIM17_CH1           HAL_GPIO_AF_1 /*!< TIM17 Multi-purpose Channel 1         */
#define HAL_GPIO_AF1_TIM17_CH1N          HAL_GPIO_AF_1 /*!< TIM17 CH1 complementary output        */

/**
  * @brief   AF 2 selection
  */
#define HAL_GPIO_AF2_I3C1                HAL_GPIO_AF_2 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF2_I3C1_SCL            HAL_GPIO_AF_2 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF2_I3C1_SDA            HAL_GPIO_AF_2 /*!< I3C1 Data                             */
#define HAL_GPIO_AF2_TIM1                HAL_GPIO_AF_2 /*!< TIM1 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM1_CH2N           HAL_GPIO_AF_2 /*!< TIM1 CH2 complementary output         */
#define HAL_GPIO_AF2_TIM1_CH3            HAL_GPIO_AF_2 /*!< TIM1 Multi-purpose Channel 3          */
#define HAL_GPIO_AF2_TIM5                HAL_GPIO_AF_2 /*!< TIM5 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM5_CH1            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 1          */
#define HAL_GPIO_AF2_TIM5_CH2            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 2          */
#define HAL_GPIO_AF2_TIM5_CH3            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 3          */
#define HAL_GPIO_AF2_TIM5_CH4            HAL_GPIO_AF_2 /*!< TIM5 Multi-purpose Channel 4          */
#define HAL_GPIO_AF2_TIM5_ETR            HAL_GPIO_AF_2 /*!< TIM5 External trigger input           */
#define HAL_GPIO_AF2_TIM8                HAL_GPIO_AF_2 /*!< TIM8 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM8_CH1            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 1          */
#define HAL_GPIO_AF2_TIM8_CH2            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 2          */
#define HAL_GPIO_AF2_TIM8_CH3            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 3          */
#define HAL_GPIO_AF2_TIM12               HAL_GPIO_AF_2 /*!< TIM12 Alternate Function mapping      */
#define HAL_GPIO_AF2_TIM12_CH1           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 1         */
#define HAL_GPIO_AF2_TIM12_CH2           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 2         */
#define HAL_GPIO_AF2_TIM15               HAL_GPIO_AF_2 /*!< TIM15 Alternate Function mapping      */
#define HAL_GPIO_AF2_TIM15_CH1           HAL_GPIO_AF_2 /*!< TIM15 Multi-purpose Channel 1         */

/**
  * @brief   AF 3 selection
  */
#define HAL_GPIO_AF3_I3C1                HAL_GPIO_AF_3 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF3_I3C1_SCL            HAL_GPIO_AF_3 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF3_I3C1_SDA            HAL_GPIO_AF_3 /*!< I3C1 Data                             */
#define HAL_GPIO_AF3_LPTIM1              HAL_GPIO_AF_3 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF3_LPTIM1_CH1          HAL_GPIO_AF_3 /*!< LPTIM1 Multi-purpose Channel 1        */
#define HAL_GPIO_AF3_LPTIM1_ETR          HAL_GPIO_AF_3 /*!< LPTIM1 External trigger input         */
#define HAL_GPIO_AF3_LPTIM1_IN1          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF3_LPTIM1_IN2          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF3_LPUART1             HAL_GPIO_AF_3 /*!< LPUART1 Alternate Function mapping    */
#define HAL_GPIO_AF3_LPUART1_CTS         HAL_GPIO_AF_3 /*!< LPUART1 Clear to send                 */
#define HAL_GPIO_AF3_LPUART1_RTS         HAL_GPIO_AF_3 /*!< LPUART1 Request to send               */
#define HAL_GPIO_AF3_LPUART1_RTS_DE      HAL_GPIO_AF3_LPUART1_RTS /*!< Alias to LPUART1_RTS        */
#define HAL_GPIO_AF3_LPUART1_RX          HAL_GPIO_AF_3 /*!< LPUART1 Serial Data Receive Input     */
#define HAL_GPIO_AF3_LPUART1_TX          HAL_GPIO_AF_3 /*!< LPUART1 Transmit Data Output          */
#define HAL_GPIO_AF3_TIM1                HAL_GPIO_AF_3 /*!< TIM1 Alternate Function mapping       */
#define HAL_GPIO_AF3_TIM1_CH4N           HAL_GPIO_AF_3 /*!< TIM1 CH4 complementary output         */
#define HAL_GPIO_AF3_TIM5                HAL_GPIO_AF_3 /*!< TIM5 Alternate Function mapping       */
#define HAL_GPIO_AF3_TIM5_CH3            HAL_GPIO_AF_3 /*!< TIM5 Multi-purpose Channel 3          */
#define HAL_GPIO_AF3_TIM8                HAL_GPIO_AF_3 /*!< TIM8 Alternate Function mapping       */
#define HAL_GPIO_AF3_TIM8_BKIN           HAL_GPIO_AF_3 /*!< TIM8 Break input                      */
#define HAL_GPIO_AF3_TIM8_BKIN2          HAL_GPIO_AF_3 /*!< TIM8 Break input 2                    */
#define HAL_GPIO_AF3_TIM8_CH1            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 1          */
#define HAL_GPIO_AF3_TIM8_CH1N           HAL_GPIO_AF_3 /*!< TIM8 CH1 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH2            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 2          */
#define HAL_GPIO_AF3_TIM8_CH2N           HAL_GPIO_AF_3 /*!< TIM8 CH2 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH3            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 3          */
#define HAL_GPIO_AF3_TIM8_CH3N           HAL_GPIO_AF_3 /*!< TIM8 CH3 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH4            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 4          */
#define HAL_GPIO_AF3_TIM8_CH4N           HAL_GPIO_AF_3 /*!< TIM8 CH4 complementary output         */
#define HAL_GPIO_AF3_TIM8_ETR            HAL_GPIO_AF_3 /*!< TIM8 External trigger input           */

/**
  * @brief   AF 4 selection
  */
#define HAL_GPIO_AF4_I2C1                HAL_GPIO_AF_4 /*!< I2C1 Alternate Function mapping       */
#define HAL_GPIO_AF4_I2C1_SCL            HAL_GPIO_AF_4 /*!< I2C1 Clock                            */
#define HAL_GPIO_AF4_I2C1_SDA            HAL_GPIO_AF_4 /*!< I2C1 Data                             */
#define HAL_GPIO_AF4_I2C1_SMBA           HAL_GPIO_AF_4 /*!< I2C1 SMBus Alert                      */
#define HAL_GPIO_AF4_I2C2                HAL_GPIO_AF_4 /*!< I2C2 Alternate Function mapping       */
#define HAL_GPIO_AF4_I2C2_SCL            HAL_GPIO_AF_4 /*!< I2C2 Clock                            */
#define HAL_GPIO_AF4_I2C2_SDA            HAL_GPIO_AF_4 /*!< I2C2 Data                             */
#define HAL_GPIO_AF4_I2C2_SMBA           HAL_GPIO_AF_4 /*!< I2C2 SMBus Alert                      */
#define HAL_GPIO_AF4_I2S3                HAL_GPIO_AF_4 /*!< I2S3 Alternate Function mapping       */
#define HAL_GPIO_AF4_I2S3_SCK            HAL_GPIO_AF_4 /*!< I2S3 Clock                            */
#define HAL_GPIO_AF4_I2S3_SDO            HAL_GPIO_AF_4 /*!< I2S3 Serial Data Output               */
#define HAL_GPIO_AF4_I3C1                HAL_GPIO_AF_4 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF4_I3C1_SCL            HAL_GPIO_AF_4 /*!< I3C1 Serial Clock                     */
#define HAL_GPIO_AF4_I3C1_SDA            HAL_GPIO_AF_4 /*!< I3C1 Serial Data                      */
#define HAL_GPIO_AF4_LPTIM1              HAL_GPIO_AF_4 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF4_LPTIM1_CH2          HAL_GPIO_AF_4 /*!< LPTIM1 Multi-purpose Channel 2        */
#define HAL_GPIO_AF4_SPI1                HAL_GPIO_AF_4 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF4_SPI1_RDY            HAL_GPIO_AF_4 /*!< SPI1 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF4_SPI3                HAL_GPIO_AF_4 /*!< SPI3 Alternate Function mapping       */
#define HAL_GPIO_AF4_SPI3_MOSI           HAL_GPIO_AF_4 /*!< SPI3 Master-Out/Slave-In              */
#define HAL_GPIO_AF4_SPI3_SCK            HAL_GPIO_AF_4 /*!< SPI3 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF4_TIM15               HAL_GPIO_AF_4 /*!< TIM15 Alternate Function mapping      */
#define HAL_GPIO_AF4_TIM15_BKIN          HAL_GPIO_AF_4 /*!< TIM15 Break input                     */
#define HAL_GPIO_AF4_TIM15_CH1           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 1         */
#define HAL_GPIO_AF4_TIM15_CH1N          HAL_GPIO_AF_4 /*!< TIM15 CH1 complementary output        */
#define HAL_GPIO_AF4_TIM15_CH2           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 2         */
#define HAL_GPIO_AF4_USART1              HAL_GPIO_AF_4 /*!< USART1 Alternate Function mapping     */
#define HAL_GPIO_AF4_USART1_RX           HAL_GPIO_AF_4 /*!< USART1 Serial Data Receive Input      */
#define HAL_GPIO_AF4_USART1_TX           HAL_GPIO_AF_4 /*!< USART1 Transmit Data Output           */
#define HAL_GPIO_AF4_USART2              HAL_GPIO_AF_4 /*!< USART2 Alternate Function mapping     */
#define HAL_GPIO_AF4_USART2_RX           HAL_GPIO_AF_4 /*!< USART2 Serial Data Receive Input      */
#define HAL_GPIO_AF4_USART2_TX           HAL_GPIO_AF_4 /*!< USART2 Transmit Data Output           */

/**
  * @brief   AF 5 selection
  */
#define HAL_GPIO_AF5_AUDIOCLK            HAL_GPIO_AF_5 /*!< Audio Clock                           */
#define HAL_GPIO_AF5_I2S1                HAL_GPIO_AF_5 /*!< I2S1 Alternate Function mapping       */
#define HAL_GPIO_AF5_I2S1_CK             HAL_GPIO_AF_5 /*!< I2S1 Serial Clock                     */
#define HAL_GPIO_AF5_I2S1_MCK            HAL_GPIO_AF_5 /*!< I2S1 Master Clock                     */
#define HAL_GPIO_AF5_I2S1_SDI            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Input                */
#define HAL_GPIO_AF5_I2S1_SDO            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Output               */
#define HAL_GPIO_AF5_I2S1_WS             HAL_GPIO_AF_5 /*!< I2S1 Word Select                      */
#define HAL_GPIO_AF5_I2S2                HAL_GPIO_AF_5 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF5_I2S2_CK             HAL_GPIO_AF_5 /*!< I2S2 Serial Clock                     */
#define HAL_GPIO_AF5_I2S2_MCK            HAL_GPIO_AF_5 /*!< I2S2 Master Clock                     */
#define HAL_GPIO_AF5_I2S2_SDI            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Input                */
#define HAL_GPIO_AF5_I2S2_SDO            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Output               */
#define HAL_GPIO_AF5_I2S2_WS             HAL_GPIO_AF_5 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF5_I2S3                HAL_GPIO_AF_5 /*!< I2S3 Alternate Function mapping       */
#define HAL_GPIO_AF5_I2S3_CK             HAL_GPIO_AF_5 /*!< I2S3 Serial Clock                     */
#define HAL_GPIO_AF5_I2S3_SDI            HAL_GPIO_AF_5 /*!< I2S3 Serial Data Input                */
#define HAL_GPIO_AF5_I2S3_SDO            HAL_GPIO_AF_5 /*!< I2S3 Serial Data Output               */
#define HAL_GPIO_AF5_I2S3_WS             HAL_GPIO_AF_5 /*!< I2S3 Word Select                      */
#define HAL_GPIO_AF5_I3C1                HAL_GPIO_AF_5 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF5_I3C1_SCL            HAL_GPIO_AF_5 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF5_I3C1_SDA            HAL_GPIO_AF_5 /*!< I3C1 Data                             */
#define HAL_GPIO_AF5_LPTIM1              HAL_GPIO_AF_5 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF5_LPTIM1_CH1          HAL_GPIO_AF_5 /*!< LPTIM1 Multi-purpose Channel 1        */
#define HAL_GPIO_AF5_LPTIM1_IN1          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF5_LPTIM1_IN2          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF5_SPI1                HAL_GPIO_AF_5 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF5_SPI1_MISO           HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out              */
#define HAL_GPIO_AF5_SPI1_MOSI           HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In              */
#define HAL_GPIO_AF5_SPI1_NSS            HAL_GPIO_AF_5 /*!< SPI1 Slave Selection                  */
#define HAL_GPIO_AF5_SPI1_RDY            HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF5_SPI1_SCK            HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF5_SPI2                HAL_GPIO_AF_5 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF5_SPI2_MISO           HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out              */
#define HAL_GPIO_AF5_SPI2_MOSI           HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In              */
#define HAL_GPIO_AF5_SPI2_NSS            HAL_GPIO_AF_5 /*!< SPI2 Slave Selection                  */
#define HAL_GPIO_AF5_SPI2_RDY            HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF5_SPI2_SCK            HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF5_SPI3                HAL_GPIO_AF_5 /*!< SPI3 Alternate Function mapping       */
#define HAL_GPIO_AF5_SPI3_MISO           HAL_GPIO_AF_5 /*!< SPI3 Master-In/Slave-Out              */
#define HAL_GPIO_AF5_SPI3_MOSI           HAL_GPIO_AF_5 /*!< SPI3 Master-Out/Slave-In              */
#define HAL_GPIO_AF5_SPI3_NSS            HAL_GPIO_AF_5 /*!< SPI3 Slave Selection                  */
#define HAL_GPIO_AF5_SPI3_SCK            HAL_GPIO_AF_5 /*!< SPI3 Master-Out/Slave-In Clock        */

/**
  * @brief   AF 6 selection
  */
#define HAL_GPIO_AF6_I2S1                HAL_GPIO_AF_6 /*!< I2S1 Alternate Function mapping       */
#define HAL_GPIO_AF6_I2S1_SDO            HAL_GPIO_AF_6 /*!< I2S1 Serial Data Output               */
#define HAL_GPIO_AF6_I2S2                HAL_GPIO_AF_6 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF6_I2S2_CK             HAL_GPIO_AF_6 /*!< I2S2 Serial Clock                     */
#define HAL_GPIO_AF6_I2S2_SDI            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Input                */
#define HAL_GPIO_AF6_I2S2_SDO            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Output               */
#define HAL_GPIO_AF6_I2S3                HAL_GPIO_AF_6 /*!< I2S3 Alternate Function mapping       */
#define HAL_GPIO_AF6_I2S3_CK             HAL_GPIO_AF_6 /*!< I2S3 Serial Clock                     */
#define HAL_GPIO_AF6_I2S3_MCK            HAL_GPIO_AF_6 /*!< I2S3 Master Clock                     */
#define HAL_GPIO_AF6_I2S3_SDI            HAL_GPIO_AF_6 /*!< I2S3 Serial Data Input                */
#define HAL_GPIO_AF6_I2S3_SDO            HAL_GPIO_AF_6 /*!< I2S3 Serial Data Output               */
#define HAL_GPIO_AF6_I2S3_WS             HAL_GPIO_AF_6 /*!< I2S3 Word Select                      */
#define HAL_GPIO_AF6_SPI1                HAL_GPIO_AF_6 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF6_SPI1_MOSI           HAL_GPIO_AF_6 /*!< SPI1 Master-Out/Slave-In              */
#define HAL_GPIO_AF6_SPI2                HAL_GPIO_AF_6 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF6_SPI2_MISO           HAL_GPIO_AF_6 /*!< SPI2 Master-In/Slave-Out              */
#define HAL_GPIO_AF6_SPI2_MOSI           HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In              */
#define HAL_GPIO_AF6_SPI2_SCK            HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF6_SPI3                HAL_GPIO_AF_6 /*!< SPI3 Alternate Function mapping       */
#define HAL_GPIO_AF6_SPI3_MISO           HAL_GPIO_AF_6 /*!< SPI3 Master-In/Slave-Out              */
#define HAL_GPIO_AF6_SPI3_MOSI           HAL_GPIO_AF_6 /*!< SPI3 Master-Out/Slave-In              */
#define HAL_GPIO_AF6_SPI3_NSS            HAL_GPIO_AF_6 /*!< SPI3 Slave Selection                  */
#define HAL_GPIO_AF6_SPI3_RDY            HAL_GPIO_AF_6 /*!< SPI3 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF6_SPI3_SCK            HAL_GPIO_AF_6 /*!< SPI3 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF6_UART4               HAL_GPIO_AF_6 /*!< UART4 Alternate Function mapping      */
#define HAL_GPIO_AF6_UART4_RX            HAL_GPIO_AF_6 /*!< UART4 Serial Data Receive Input       */
#define HAL_GPIO_AF6_UART4_TX            HAL_GPIO_AF_6 /*!< UART4 Transmit Data Output            */

/**
  * @brief   AF 7 selection
  */
#define HAL_GPIO_AF7_I2S2                HAL_GPIO_AF_7 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF7_I2S2_WS             HAL_GPIO_AF_7 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF7_I2S3                HAL_GPIO_AF_7 /*!< I2S3 Alternate Function mapping       */
#define HAL_GPIO_AF7_I2S3_SDO            HAL_GPIO_AF_7 /*!< I2S3 Serial Data Output               */
#define HAL_GPIO_AF7_SPI2                HAL_GPIO_AF_7 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF7_SPI2_NSS            HAL_GPIO_AF_7 /*!< SPI2 Slave Selection                  */
#define HAL_GPIO_AF7_SPI2_RDY            HAL_GPIO_AF_7 /*!< SPI2 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF7_SPI3                HAL_GPIO_AF_7 /*!< SPI3 Alternate Function mapping       */
#define HAL_GPIO_AF7_SPI3_MOSI           HAL_GPIO_AF_7 /*!< SPI3 Master-Out/Slave-In              */
#define HAL_GPIO_AF7_USART1              HAL_GPIO_AF_7 /*!< USART1 Alternate Function mapping     */
#define HAL_GPIO_AF7_USART1_CK           HAL_GPIO_AF_7 /*!< USART1 Clock                          */
#define HAL_GPIO_AF7_USART1_CTS          HAL_GPIO_AF_7 /*!< USART1 Clear to send                  */
#define HAL_GPIO_AF7_USART1_NSS          HAL_GPIO_AF_7 /*!< USART1 Slave Selection                */
#define HAL_GPIO_AF7_USART1_RTS          HAL_GPIO_AF_7 /*!< USART1 Request to send                */
#define HAL_GPIO_AF7_USART1_RTS_DE       HAL_GPIO_AF7_USART1_RTS /*!< Alias to USART1_RTS         */
#define HAL_GPIO_AF7_USART1_RX           HAL_GPIO_AF_7 /*!< USART1 Serial Data Receive Input      */
#define HAL_GPIO_AF7_USART1_TX           HAL_GPIO_AF_7 /*!< USART1 Transmit Data Output           */
#define HAL_GPIO_AF7_USART2              HAL_GPIO_AF_7 /*!< USART2 Alternate Function mapping     */
#define HAL_GPIO_AF7_USART2_CK           HAL_GPIO_AF_7 /*!< USART2 Clock                          */
#define HAL_GPIO_AF7_USART2_CTS          HAL_GPIO_AF_7 /*!< USART2 Clear to send                  */
#define HAL_GPIO_AF7_USART2_NSS          HAL_GPIO_AF_7 /*!< USART2 Slave Selection                */
#define HAL_GPIO_AF7_USART2_RTS          HAL_GPIO_AF_7 /*!< USART2 Request to send                */
#define HAL_GPIO_AF7_USART2_RTS_DE       HAL_GPIO_AF7_USART2_RTS /*!< Alias to USART2_RTS         */
#define HAL_GPIO_AF7_USART2_RX           HAL_GPIO_AF_7 /*!< USART2 Serial Data Receive Input      */
#define HAL_GPIO_AF7_USART2_TX           HAL_GPIO_AF_7 /*!< USART2 Transmit Data Output           */
#define HAL_GPIO_AF7_USART3              HAL_GPIO_AF_7 /*!< USART3 Alternate Function mapping     */
#define HAL_GPIO_AF7_USART3_CK           HAL_GPIO_AF_7 /*!< USART3 Clock                          */
#define HAL_GPIO_AF7_USART3_CTS          HAL_GPIO_AF_7 /*!< USART3 Clear to send                  */
#define HAL_GPIO_AF7_USART3_NSS          HAL_GPIO_AF_7 /*!< USART3 Slave Selection                */
#define HAL_GPIO_AF7_USART3_RTS          HAL_GPIO_AF_7 /*!< USART3 Request to send                */
#define HAL_GPIO_AF7_USART3_RTS_DE       HAL_GPIO_AF7_USART3_RTS /*!< Alias to USART3_RTS         */
#define HAL_GPIO_AF7_USART3_RX           HAL_GPIO_AF_7 /*!< USART3 Serial Data Receive Input      */
#define HAL_GPIO_AF7_USART3_TX           HAL_GPIO_AF_7 /*!< USART3 Transmit Data Output           */

/**
  * @brief   AF 8 selection
  */
#define HAL_GPIO_AF8_I2C2                HAL_GPIO_AF_8 /*!< I2C2 Alternate Function mapping       */
#define HAL_GPIO_AF8_I2C2_SCL            HAL_GPIO_AF_8 /*!< I2C2 Clock                            */
#define HAL_GPIO_AF8_I2C2_SDA            HAL_GPIO_AF_8 /*!< I2C2 Data                             */
#define HAL_GPIO_AF8_LPUART1             HAL_GPIO_AF_8 /*!< LPUART1 Alternate Function mapping    */
#define HAL_GPIO_AF8_LPUART1_CTS         HAL_GPIO_AF_8 /*!< LPUART1 Clear to send                 */
#define HAL_GPIO_AF8_LPUART1_RTS         HAL_GPIO_AF_8 /*!< LPUART1 Request to send               */
#define HAL_GPIO_AF8_LPUART1_RTS_DE      HAL_GPIO_AF8_LPUART1_RTS /*!< Alias to LPUART1_RTS        */
#define HAL_GPIO_AF8_LPUART1_RX          HAL_GPIO_AF_8 /*!< LPUART1 Serial Data Receive Input     */
#define HAL_GPIO_AF8_LPUART1_TX          HAL_GPIO_AF_8 /*!< LPUART1 Transmit Data Output          */
#define HAL_GPIO_AF8_TIM5                HAL_GPIO_AF_8 /*!< TIM5 Alternate Function mapping       */
#define HAL_GPIO_AF8_TIM5_CH4            HAL_GPIO_AF_8 /*!< TIM5 Multi-purpose Channel 4          */
#define HAL_GPIO_AF8_TIM5_ETR            HAL_GPIO_AF_8 /*!< TIM5 External trigger input           */
#define HAL_GPIO_AF8_UART4               HAL_GPIO_AF_8 /*!< UART4 Alternate Function mapping      */
#define HAL_GPIO_AF8_UART4_CTS           HAL_GPIO_AF_8 /*!< UART4 Clear to send                   */
#define HAL_GPIO_AF8_UART4_RTS           HAL_GPIO_AF_8 /*!< UART4 Request to send                 */
#define HAL_GPIO_AF8_UART4_RTS_DE        HAL_GPIO_AF8_UART4_RTS /*!< Alias to UART4_RTS           */
#define HAL_GPIO_AF8_UART4_RX            HAL_GPIO_AF_8 /*!< UART4 Serial Data Receive Input       */
#define HAL_GPIO_AF8_UART4_TX            HAL_GPIO_AF_8 /*!< UART4 Transmit Data Output            */
#define HAL_GPIO_AF8_UART5               HAL_GPIO_AF_8 /*!< UART5 Alternate Function mapping      */
#define HAL_GPIO_AF8_UART5_CTS           HAL_GPIO_AF_8 /*!< UART5 Clear to send                   */
#define HAL_GPIO_AF8_UART5_RTS           HAL_GPIO_AF_8 /*!< UART5 Request to send                 */
#define HAL_GPIO_AF8_UART5_RTS_DE        HAL_GPIO_AF8_UART5_RTS /*!< Alias to UART5_RTS           */
#define HAL_GPIO_AF8_UART5_RX            HAL_GPIO_AF_8 /*!< UART5 Serial Data Receive Input       */
#define HAL_GPIO_AF8_UART5_TX            HAL_GPIO_AF_8 /*!< UART5 Transmit Data Output            */

/**
  * @brief   AF 9 selection
  */
#define HAL_GPIO_AF9_FDCAN1              HAL_GPIO_AF_9 /*!< FDCAN1 Alternate Function mapping     */
#define HAL_GPIO_AF9_FDCAN1_RX           HAL_GPIO_AF_9 /*!< FDCAN1 Receive pin                    */
#define HAL_GPIO_AF9_FDCAN1_TX           HAL_GPIO_AF_9 /*!< FDCAN1 Transmit pin                   */
#define HAL_GPIO_AF9_I2C1                HAL_GPIO_AF_9 /*!< I2C1 Alternate Function mapping       */
#define HAL_GPIO_AF9_I2C1_SCL            HAL_GPIO_AF_9 /*!< I2C1 Clock                            */
#define HAL_GPIO_AF9_I2C1_SDA            HAL_GPIO_AF_9 /*!< I2C1 Data                             */
#define HAL_GPIO_AF9_I2C1_SMBA           HAL_GPIO_AF_9 /*!< I2C1 SMBus Alert                      */
#define HAL_GPIO_AF9_I2C2                HAL_GPIO_AF_9 /*!< I2C2 Alternate Function mapping       */
#define HAL_GPIO_AF9_I2C2_SCL            HAL_GPIO_AF_9 /*!< I2C2 Clock                            */
#define HAL_GPIO_AF9_I2C2_SDA            HAL_GPIO_AF_9 /*!< I2C2 Data                             */
#define HAL_GPIO_AF9_I2S2                HAL_GPIO_AF_9 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF9_I2S2_WS             HAL_GPIO_AF_9 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF9_SPI2                HAL_GPIO_AF_9 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF9_SPI2_NSS            HAL_GPIO_AF_9 /*!< SPI2 Slave Selection                  */

/**
  * @brief   AF 10 selection
  */
#define HAL_GPIO_AF10_CRS                HAL_GPIO_AF_10 /*!< CRS Alternate Function mapping       */
#define HAL_GPIO_AF10_CRS_SYNC           HAL_GPIO_AF_10 /*!< CRS synchronization                  */
#define HAL_GPIO_AF10_TIM16              HAL_GPIO_AF_10 /*!< TIM16 Alternate Function mapping     */
#define HAL_GPIO_AF10_TIM16_BKIN         HAL_GPIO_AF_10 /*!< TIM16 Break input                    */
#define HAL_GPIO_AF10_TIM16_CH1          HAL_GPIO_AF_10 /*!< TIM16 Multi-purpose Channel 1        */
#define HAL_GPIO_AF10_TIM16_CH1N         HAL_GPIO_AF_10 /*!< TIM16 CH1 complementary output       */

/**
  * @brief   AF 11 selection
  */
#define HAL_GPIO_AF11_USART1             HAL_GPIO_AF_11 /*!< USART1 Alternate Function mapping    */
#define HAL_GPIO_AF11_USART1_TX          HAL_GPIO_AF_11 /*!< USART1 Transmit Data Output          */
#define HAL_GPIO_AF11_USART3             HAL_GPIO_AF_11 /*!< USART3 Alternate Function mapping    */
#define HAL_GPIO_AF11_USART3_CK          HAL_GPIO_AF_11 /*!< USART3 Clock                         */
#define HAL_GPIO_AF11_USART3_CTS         HAL_GPIO_AF_11 /*!< USART3 Clear to send                 */
#define HAL_GPIO_AF11_USART3_NSS         HAL_GPIO_AF_11 /*!< USART3 Slave Selection               */
#define HAL_GPIO_AF11_USART3_RTS         HAL_GPIO_AF_11 /*!< USART3 Request to send               */
#define HAL_GPIO_AF11_USART3_RTS_DE      HAL_GPIO_AF11_USART3_RTS /*!< Alias to USART3_RTS        */
#define HAL_GPIO_AF11_USART3_RX          HAL_GPIO_AF_11 /*!< USART3 Serial Data Receive Input     */
#define HAL_GPIO_AF11_USART3_TX          HAL_GPIO_AF_11 /*!< USART3 Transmit Data Output          */

/**
  * @brief   AF 13 selection
  */
#define HAL_GPIO_AF13_TIM8               HAL_GPIO_AF_13 /*!< TIM8 Alternate Function mapping      */
#define HAL_GPIO_AF13_TIM8_CH1           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 1         */
#define HAL_GPIO_AF13_TIM8_CH2           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 2         */
#define HAL_GPIO_AF13_TIM8_CH2N          HAL_GPIO_AF_13 /*!< TIM8 CH2 complementary output        */
#define HAL_GPIO_AF13_TIM8_CH3           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 3         */
#define HAL_GPIO_AF13_TIM8_CH4           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 4         */
#define HAL_GPIO_AF13_TIM8_CH4N          HAL_GPIO_AF_13 /*!< TIM8 CH4 complementary output        */

/**
  * @brief   AF 14 selection
  */
#define HAL_GPIO_AF14_COMP1              HAL_GPIO_AF_14 /*!< COMP1 Alternate Function mapping     */
#define HAL_GPIO_AF14_COMP1_OUT          HAL_GPIO_AF_14 /*!< COMP1 Output channel                 */
#define HAL_GPIO_AF14_TIM2               HAL_GPIO_AF_14 /*!< TIM2 Alternate Function mapping      */
#define HAL_GPIO_AF14_TIM2_CH4           HAL_GPIO_AF_14 /*!< TIM2 Multi-purpose Channel 4         */
#define HAL_GPIO_AF14_TIM2_ETR           HAL_GPIO_AF_14 /*!< TIM2 External trigger input          */
#define HAL_GPIO_AF14_UART5              HAL_GPIO_AF_14 /*!< UART5 Alternate Function mapping     */
#define HAL_GPIO_AF14_UART5_CTS          HAL_GPIO_AF_14 /*!< UART5 Clear to send                  */
#define HAL_GPIO_AF14_UART5_RTS          HAL_GPIO_AF_14 /*!< UART5 Request to send                */
#define HAL_GPIO_AF14_UART5_RTS_DE       HAL_GPIO_AF14_UART5_RTS /*!< Alias to UART5_RTS          */
#define HAL_GPIO_AF14_UART5_RX           HAL_GPIO_AF_14 /*!< UART5 Serial Data Receive Input      */
#define HAL_GPIO_AF14_UART5_TX           HAL_GPIO_AF_14 /*!< UART5 Transmit Data Output           */

/**
  * @brief   AF 15 selection
  */
#define HAL_GPIO_AF15_EVENTOUT           HAL_GPIO_AF_15 /*!< EVENTOUT Alternate Function mapping  */
#endif /* STM32C551xx || STM32C552xx || STM32C562xx */
#if (defined (STM32C531xx) || defined (STM32C532xx) || defined (STM32C542xx))
/**
  * @brief   AF 0 selection
  */
#define HAL_GPIO_AF0_SWJ                 HAL_GPIO_AF_0 /*!< SWJ (SWD and JTAG)                     */
#define HAL_GPIO_AF0_JTCK                HAL_GPIO_AF_0 /*!< JTAG Test Clock                        */
#define HAL_GPIO_AF0_JTDI                HAL_GPIO_AF_0 /*!< JTAG Test Data In                      */
#define HAL_GPIO_AF0_JTDO                HAL_GPIO_AF_0 /*!< JTAG Test Data Out                     */
#define HAL_GPIO_AF0_JTMS                HAL_GPIO_AF_0 /*!< JTAG Test Mode Select                  */
#define HAL_GPIO_AF0_MCO                 HAL_GPIO_AF_0 /*!< MCO Alternate Function mapping         */
#define HAL_GPIO_AF0_MCO1                HAL_GPIO_AF_0 /*!< MCO1                                   */
#define HAL_GPIO_AF0_MCO2                HAL_GPIO_AF_0 /*!< MCO2                                   */
#define HAL_GPIO_AF0_NJTRST              HAL_GPIO_AF_0 /*!< NJTRST                                 */
#define HAL_GPIO_AF0_PWR                 HAL_GPIO_AF_0 /*!< PWR Alternate Function mapping         */
#define HAL_GPIO_AF0_PWR_CSLEEP          HAL_GPIO_AF_0 /*!< PWR CSLEEP mode                        */
#define HAL_GPIO_AF0_CSLEEP              HAL_GPIO_AF0_PWR_CSLEEP /*!< Alias to PWR_CSLEEP          */
#define HAL_GPIO_AF0_PWR_CSTOP           HAL_GPIO_AF_0 /*!< PWR CSTOP mode                         */
#define HAL_GPIO_AF0_CSTOP               HAL_GPIO_AF0_PWR_CSTOP /*!< Alias to PWR_CSTOP            */
#define HAL_GPIO_AF0_RTC                 HAL_GPIO_AF_0 /*!< RTC Alternate Function mapping         */
#define HAL_GPIO_AF0_RTC_OUT2            HAL_GPIO_AF_0 /*!< RTC Output 2                           */
#define HAL_GPIO_AF0_RTC_REFIN           HAL_GPIO_AF_0 /*!< RTC Reference input                    */
#define HAL_GPIO_AF0_SWCLK               HAL_GPIO_AF_0 /*!< Serial Wire Clock                      */
#define HAL_GPIO_AF0_SWDIO               HAL_GPIO_AF_0 /*!< Serial Wire Debug Input/Output         */
#define HAL_GPIO_AF0_TRACECLK            HAL_GPIO_AF_0 /*!< TRACE Clock                            */
#define HAL_GPIO_AF0_TRACED0             HAL_GPIO_AF_0 /*!< TRACE Data Output 0                    */
#define HAL_GPIO_AF0_TRACED1             HAL_GPIO_AF_0 /*!< TRACE Data Output 1                    */
#define HAL_GPIO_AF0_TRACED2             HAL_GPIO_AF_0 /*!< TRACE Data Output 2                    */
#define HAL_GPIO_AF0_TRACED3             HAL_GPIO_AF_0 /*!< TRACE Data Output 3                    */
#define HAL_GPIO_AF0_TRACESWO            HAL_GPIO_AF_0 /*!< TRACE Serial Wire Output               */

/**
  * @brief   AF 1 selection
  */
#define HAL_GPIO_AF1_LPTIM1              HAL_GPIO_AF_1 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF1_LPTIM1_ETR          HAL_GPIO_AF_1 /*!< LPTIM1 External trigger input         */
#define HAL_GPIO_AF1_LPTIM1_IN1          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF1_LPTIM1_IN2          HAL_GPIO_AF_1 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF1_TIM1                HAL_GPIO_AF_1 /*!< TIM1 Alternate Function mapping       */
#define HAL_GPIO_AF1_TIM1_BKIN           HAL_GPIO_AF_1 /*!< TIM1 Break input                      */
#define HAL_GPIO_AF1_TIM1_BKIN2          HAL_GPIO_AF_1 /*!< TIM1 Break input 2                    */
#define HAL_GPIO_AF1_TIM1_CH1            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 1          */
#define HAL_GPIO_AF1_TIM1_CH1N           HAL_GPIO_AF_1 /*!< TIM1 CH1 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH2            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 2          */
#define HAL_GPIO_AF1_TIM1_CH2N           HAL_GPIO_AF_1 /*!< TIM1 CH2 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH3            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 3          */
#define HAL_GPIO_AF1_TIM1_CH3N           HAL_GPIO_AF_1 /*!< TIM1 CH3 complementary output         */
#define HAL_GPIO_AF1_TIM1_CH4            HAL_GPIO_AF_1 /*!< TIM1 Multi-purpose Channel 4          */
#define HAL_GPIO_AF1_TIM1_CH4N           HAL_GPIO_AF_1 /*!< TIM1 CH4 complementary output         */
#define HAL_GPIO_AF1_TIM1_ETR            HAL_GPIO_AF_1 /*!< TIM1 External trigger input           */
#define HAL_GPIO_AF1_TIM2                HAL_GPIO_AF_1 /*!< TIM2 Alternate Function mapping       */
#define HAL_GPIO_AF1_TIM2_CH1            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 1          */
#define HAL_GPIO_AF1_TIM2_CH2            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 2          */
#define HAL_GPIO_AF1_TIM2_CH3            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 3          */
#define HAL_GPIO_AF1_TIM2_CH4            HAL_GPIO_AF_1 /*!< TIM2 Multi-purpose Channel 4          */

/**
  * @brief   AF 2 selection
  */
#define HAL_GPIO_AF2_I3C1                HAL_GPIO_AF_2 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF2_I3C1_SCL            HAL_GPIO_AF_2 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF2_I3C1_SDA            HAL_GPIO_AF_2 /*!< I3C1 Data                             */
#define HAL_GPIO_AF2_TIM1                HAL_GPIO_AF_2 /*!< TIM1 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM1_CH2N           HAL_GPIO_AF_2 /*!< TIM1 CH2 complementary output         */
#define HAL_GPIO_AF2_TIM1_CH3            HAL_GPIO_AF_2 /*!< TIM1 Multi-purpose Channel 3          */
#define HAL_GPIO_AF2_TIM8                HAL_GPIO_AF_2 /*!< TIM8 Alternate Function mapping       */
#define HAL_GPIO_AF2_TIM8_CH1            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 1          */
#define HAL_GPIO_AF2_TIM8_CH2            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 2          */
#define HAL_GPIO_AF2_TIM8_CH3            HAL_GPIO_AF_2 /*!< TIM8 Multi-purpose Channel 3          */
#define HAL_GPIO_AF2_TIM12               HAL_GPIO_AF_2 /*!< TIM12 Alternate Function mapping      */
#define HAL_GPIO_AF2_TIM12_CH1           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 1         */
#define HAL_GPIO_AF2_TIM12_CH2           HAL_GPIO_AF_2 /*!< TIM12 Multi-purpose Channel 2         */
#define HAL_GPIO_AF2_TIM15               HAL_GPIO_AF_2 /*!< TIM15 Alternate Function mapping      */
#define HAL_GPIO_AF2_TIM15_CH1           HAL_GPIO_AF_2 /*!< TIM15 Multi-purpose Channel 1         */

/**
  * @brief   AF 3 selection
  */
#define HAL_GPIO_AF3_I3C1                HAL_GPIO_AF_3 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF3_I3C1_SCL            HAL_GPIO_AF_3 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF3_I3C1_SDA            HAL_GPIO_AF_3 /*!< I3C1 Data                             */
#define HAL_GPIO_AF3_LPTIM1              HAL_GPIO_AF_3 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF3_LPTIM1_CH1          HAL_GPIO_AF_3 /*!< LPTIM1 Multi-purpose Channel 1        */
#define HAL_GPIO_AF3_LPTIM1_ETR          HAL_GPIO_AF_3 /*!< LPTIM1 External trigger input         */
#define HAL_GPIO_AF3_LPTIM1_IN1          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF3_LPTIM1_IN2          HAL_GPIO_AF_3 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF3_LPUART1             HAL_GPIO_AF_3 /*!< LPUART1 Alternate Function mapping    */
#define HAL_GPIO_AF3_LPUART1_CTS         HAL_GPIO_AF_3 /*!< LPUART1 Clear to send                 */
#define HAL_GPIO_AF3_LPUART1_RTS         HAL_GPIO_AF_3 /*!< LPUART1 Request to send               */
#define HAL_GPIO_AF3_LPUART1_RTS_DE      HAL_GPIO_AF3_LPUART1_RTS /*!< Alias to LPUART1_RTS       */
#define HAL_GPIO_AF3_LPUART1_RX          HAL_GPIO_AF_3 /*!< LPUART1 Serial Data Receive Input     */
#define HAL_GPIO_AF3_LPUART1_TX          HAL_GPIO_AF_3 /*!< LPUART1 Transmit Data Output          */
#define HAL_GPIO_AF3_TIM8                HAL_GPIO_AF_3 /*!< TIM8 Alternate Function mapping       */
#define HAL_GPIO_AF3_TIM8_BKIN           HAL_GPIO_AF_3 /*!< TIM8 Break input                      */
#define HAL_GPIO_AF3_TIM8_BKIN2          HAL_GPIO_AF_3 /*!< TIM8 Break input 2                    */
#define HAL_GPIO_AF3_TIM8_CH1            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 1          */
#define HAL_GPIO_AF3_TIM8_CH1N           HAL_GPIO_AF_3 /*!< TIM8 CH1 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH2            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 2          */
#define HAL_GPIO_AF3_TIM8_CH2N           HAL_GPIO_AF_3 /*!< TIM8 CH2 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH3            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 3          */
#define HAL_GPIO_AF3_TIM8_CH3N           HAL_GPIO_AF_3 /*!< TIM8 CH3 complementary output         */
#define HAL_GPIO_AF3_TIM8_CH4            HAL_GPIO_AF_3 /*!< TIM8 Multi-purpose Channel 4          */
#define HAL_GPIO_AF3_TIM8_CH4N           HAL_GPIO_AF_3 /*!< TIM8 CH4 complementary output         */
#define HAL_GPIO_AF3_TIM8_ETR            HAL_GPIO_AF_3 /*!< TIM8 External trigger input           */

/**
  * @brief   AF 4 selection
  */
#define HAL_GPIO_AF4_I2C1                HAL_GPIO_AF_4 /*!< I2C1 Alternate Function mapping       */
#define HAL_GPIO_AF4_I2C1_SCL            HAL_GPIO_AF_4 /*!< I2C1 Clock                            */
#define HAL_GPIO_AF4_I2C1_SDA            HAL_GPIO_AF_4 /*!< I2C1 Data                             */
#define HAL_GPIO_AF4_I2C1_SMBA           HAL_GPIO_AF_4 /*!< I2C1 SMBus Alert                      */
#define HAL_GPIO_AF4_I2S1                HAL_GPIO_AF_4 /*!< I2S1 Alternate Function mapping       */
#define HAL_GPIO_AF4_I2S1_SDO            HAL_GPIO_AF_4 /*!< I2S1 Serial Data Output               */
#define HAL_GPIO_AF4_I3C1                HAL_GPIO_AF_4 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF4_I3C1_SCL            HAL_GPIO_AF_4 /*!< I3C1 Serial Clock                     */
#define HAL_GPIO_AF4_I3C1_SDA            HAL_GPIO_AF_4 /*!< I3C1 Serial Data                      */
#define HAL_GPIO_AF4_LPTIM1              HAL_GPIO_AF_4 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF4_LPTIM1_CH2          HAL_GPIO_AF_4 /*!< LPTIM1 Multi-purpose Channel 2        */
#define HAL_GPIO_AF4_SPI1                HAL_GPIO_AF_4 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF4_SPI1_MOSI           HAL_GPIO_AF_4 /*!< SPI1 Master-Out/Slave-In              */
#define HAL_GPIO_AF4_SPI1_RDY            HAL_GPIO_AF_4 /*!< SPI1 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF4_TIM15               HAL_GPIO_AF_4 /*!< TIM15 Alternate Function mapping      */
#define HAL_GPIO_AF4_TIM15_BKIN          HAL_GPIO_AF_4 /*!< TIM15 Break input                     */
#define HAL_GPIO_AF4_TIM15_CH1           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 1         */
#define HAL_GPIO_AF4_TIM15_CH1N          HAL_GPIO_AF_4 /*!< TIM15 CH1 complementary output        */
#define HAL_GPIO_AF4_TIM15_CH2           HAL_GPIO_AF_4 /*!< TIM15 Multi-purpose Channel 2         */
#define HAL_GPIO_AF4_USART1              HAL_GPIO_AF_4 /*!< USART1 Alternate Function mapping     */
#define HAL_GPIO_AF4_USART1_RX           HAL_GPIO_AF_4 /*!< USART1 Serial Data Receive Input      */
#define HAL_GPIO_AF4_USART1_TX           HAL_GPIO_AF_4 /*!< USART1 Transmit Data Output           */
#define HAL_GPIO_AF4_USART2              HAL_GPIO_AF_4 /*!< USART2 Alternate Function mapping     */
#define HAL_GPIO_AF4_USART2_RX           HAL_GPIO_AF_4 /*!< USART2 Serial Data Receive Input      */
#define HAL_GPIO_AF4_USART2_TX           HAL_GPIO_AF_4 /*!< USART2 Transmit Data Output           */

/**
  * @brief   AF 5 selection
  */
#define HAL_GPIO_AF5_AUDIOCLK            HAL_GPIO_AF_5 /*!< Audio Clock                           */
#define HAL_GPIO_AF5_I2S1                HAL_GPIO_AF_5 /*!< I2S1 Alternate Function mapping       */
#define HAL_GPIO_AF5_I2S1_CK             HAL_GPIO_AF_5 /*!< I2S1 Serial Clock                     */
#define HAL_GPIO_AF5_I2S1_MCK            HAL_GPIO_AF_5 /*!< I2S1 Master Clock                     */
#define HAL_GPIO_AF5_I2S1_SDI            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Input                */
#define HAL_GPIO_AF5_I2S1_SDO            HAL_GPIO_AF_5 /*!< I2S1 Serial Data Output               */
#define HAL_GPIO_AF5_I2S1_WS             HAL_GPIO_AF_5 /*!< I2S1 Word Select                      */
#define HAL_GPIO_AF5_I2S2                HAL_GPIO_AF_5 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF5_I2S2_CK             HAL_GPIO_AF_5 /*!< I2S2 Serial Clock                     */
#define HAL_GPIO_AF5_I2S2_MCK            HAL_GPIO_AF_5 /*!< I2S2 Master Clock                     */
#define HAL_GPIO_AF5_I2S2_SDI            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Input                */
#define HAL_GPIO_AF5_I2S2_SDO            HAL_GPIO_AF_5 /*!< I2S2 Serial Data Output               */
#define HAL_GPIO_AF5_I2S2_WS             HAL_GPIO_AF_5 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF5_I3C1                HAL_GPIO_AF_5 /*!< I3C1 Alternate Function mapping       */
#define HAL_GPIO_AF5_I3C1_SCL            HAL_GPIO_AF_5 /*!< I3C1 Clock                            */
#define HAL_GPIO_AF5_LPTIM1              HAL_GPIO_AF_5 /*!< LPTIM1 Alternate Function mapping     */
#define HAL_GPIO_AF5_LPTIM1_CH1          HAL_GPIO_AF_5 /*!< LPTIM1 Multi-purpose Channel 1        */
#define HAL_GPIO_AF5_LPTIM1_IN1          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 1 Input                */
#define HAL_GPIO_AF5_LPTIM1_IN2          HAL_GPIO_AF_5 /*!< LPTIM1 Channel 2 Input                */
#define HAL_GPIO_AF5_SPI1                HAL_GPIO_AF_5 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF5_SPI1_MISO           HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out              */
#define HAL_GPIO_AF5_SPI1_MOSI           HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In              */
#define HAL_GPIO_AF5_SPI1_NSS            HAL_GPIO_AF_5 /*!< SPI1 Slave Selection                  */
#define HAL_GPIO_AF5_SPI1_RDY            HAL_GPIO_AF_5 /*!< SPI1 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF5_SPI1_SCK            HAL_GPIO_AF_5 /*!< SPI1 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF5_SPI2                HAL_GPIO_AF_5 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF5_SPI2_MISO           HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out              */
#define HAL_GPIO_AF5_SPI2_MOSI           HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In              */
#define HAL_GPIO_AF5_SPI2_NSS            HAL_GPIO_AF_5 /*!< SPI2 Slave Selection                  */
#define HAL_GPIO_AF5_SPI2_RDY            HAL_GPIO_AF_5 /*!< SPI2 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF5_SPI2_SCK            HAL_GPIO_AF_5 /*!< SPI2 Master-Out/Slave-In Clock        */

/**
  * @brief   AF 6 selection
  */
#define HAL_GPIO_AF6_FDCAN2              HAL_GPIO_AF_6 /*!< FDCAN2 Alternate Function mapping     */
#define HAL_GPIO_AF6_FDCAN2_RX           HAL_GPIO_AF_6 /*!< FDCAN2 Receive pin                    */
#define HAL_GPIO_AF6_FDCAN2_TX           HAL_GPIO_AF_6 /*!< FDCAN2 Transmit pin                   */
#define HAL_GPIO_AF6_I2S1                HAL_GPIO_AF_6 /*!< I2S1 Alternate Function mapping       */
#define HAL_GPIO_AF6_I2S1_CK             HAL_GPIO_AF_6 /*!< I2S1 Serial Clock                     */
#define HAL_GPIO_AF6_I2S1_MCK            HAL_GPIO_AF_6 /*!< I2S1 Master Clock                     */
#define HAL_GPIO_AF6_I2S1_SDI            HAL_GPIO_AF_6 /*!< I2S1 Serial Data Input                */
#define HAL_GPIO_AF6_I2S1_SDO            HAL_GPIO_AF_6 /*!< I2S1 Serial Data Output               */
#define HAL_GPIO_AF6_I2S1_WS             HAL_GPIO_AF_6 /*!< I2S1 Word Select                      */
#define HAL_GPIO_AF6_I2S2                HAL_GPIO_AF_6 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF6_I2S2_CK             HAL_GPIO_AF_6 /*!< I2S2 Serial Clock                     */
#define HAL_GPIO_AF6_I2S2_SDI            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Input                */
#define HAL_GPIO_AF6_I2S2_SDO            HAL_GPIO_AF_6 /*!< I2S2 Serial Data Output               */
#define HAL_GPIO_AF6_I2S2_WS             HAL_GPIO_AF_6 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF6_SPI1                HAL_GPIO_AF_6 /*!< SPI1 Alternate Function mapping       */
#define HAL_GPIO_AF6_SPI1_MISO           HAL_GPIO_AF_6 /*!< SPI1 Master-In/Slave-Out              */
#define HAL_GPIO_AF6_SPI1_MOSI           HAL_GPIO_AF_6 /*!< SPI1 Master-Out/Slave-In              */
#define HAL_GPIO_AF6_SPI1_NSS            HAL_GPIO_AF_6 /*!< SPI1 Slave Selection                  */
#define HAL_GPIO_AF6_SPI1_RDY            HAL_GPIO_AF_6 /*!< SPI1 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF6_SPI1_SCK            HAL_GPIO_AF_6 /*!< SPI1 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF6_SPI2                HAL_GPIO_AF_6 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF6_SPI2_MISO           HAL_GPIO_AF_6 /*!< SPI2 Master-In/Slave-Out              */
#define HAL_GPIO_AF6_SPI2_MOSI           HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In              */
#define HAL_GPIO_AF6_SPI2_NSS            HAL_GPIO_AF_6 /*!< SPI2 Slave Selection                  */
#define HAL_GPIO_AF6_SPI2_SCK            HAL_GPIO_AF_6 /*!< SPI2 Master-Out/Slave-In Clock        */
#define HAL_GPIO_AF6_UART4               HAL_GPIO_AF_6 /*!< UART4 Alternate Function mapping      */
#define HAL_GPIO_AF6_UART4_RX            HAL_GPIO_AF_6 /*!< UART4 Serial Data Receive Input       */
#define HAL_GPIO_AF6_UART4_TX            HAL_GPIO_AF_6 /*!< UART4 Transmit Data Output            */

/**
  * @brief   AF 7 selection
  */
#define HAL_GPIO_AF7_I2S2                HAL_GPIO_AF_7 /*!< I2S2 Alternate Function mapping       */
#define HAL_GPIO_AF7_I2S2_SDO            HAL_GPIO_AF_7 /*!< I2S2 Serial Data Output               */
#define HAL_GPIO_AF7_I2S2_WS             HAL_GPIO_AF_7 /*!< I2S2 Word Select                      */
#define HAL_GPIO_AF7_SPI2                HAL_GPIO_AF_7 /*!< SPI2 Alternate Function mapping       */
#define HAL_GPIO_AF7_SPI2_MOSI           HAL_GPIO_AF_7 /*!< SPI2 Master-Out/Slave-In              */
#define HAL_GPIO_AF7_SPI2_NSS            HAL_GPIO_AF_7 /*!< SPI2 Slave Selection                  */
#define HAL_GPIO_AF7_SPI2_RDY            HAL_GPIO_AF_7 /*!< SPI2 Master-In/Slave-Out FIFOs status */
#define HAL_GPIO_AF7_USART1              HAL_GPIO_AF_7 /*!< USART1 Alternate Function mapping     */
#define HAL_GPIO_AF7_USART1_CK           HAL_GPIO_AF_7 /*!< USART1 Clock                          */
#define HAL_GPIO_AF7_USART1_CTS          HAL_GPIO_AF_7 /*!< USART1 Clear to send                  */
#define HAL_GPIO_AF7_USART1_NSS          HAL_GPIO_AF_7 /*!< USART1 Slave Selection                */
#define HAL_GPIO_AF7_USART1_RTS          HAL_GPIO_AF_7 /*!< USART1 Request to send                */
#define HAL_GPIO_AF7_USART1_RTS_DE       HAL_GPIO_AF7_USART1_RTS /*!< Alias to USART1_RTS          */
#define HAL_GPIO_AF7_USART1_RX           HAL_GPIO_AF_7 /*!< USART1 Serial Data Receive Input      */
#define HAL_GPIO_AF7_USART1_TX           HAL_GPIO_AF_7 /*!< USART1 Transmit Data Output           */
#define HAL_GPIO_AF7_USART2              HAL_GPIO_AF_7 /*!< USART2 Alternate Function mapping     */
#define HAL_GPIO_AF7_USART2_CK           HAL_GPIO_AF_7 /*!< USART2 Clock                          */
#define HAL_GPIO_AF7_USART2_CTS          HAL_GPIO_AF_7 /*!< USART2 Clear to send                  */
#define HAL_GPIO_AF7_USART2_NSS          HAL_GPIO_AF_7 /*!< USART2 Slave Selection                */
#define HAL_GPIO_AF7_USART2_RTS          HAL_GPIO_AF_7 /*!< USART2 Request to send                */
#define HAL_GPIO_AF7_USART2_RTS_DE       HAL_GPIO_AF7_USART2_RTS /*!< Alias to USART2_RTS         */
#define HAL_GPIO_AF7_USART2_RX           HAL_GPIO_AF_7 /*!< USART2 Serial Data Receive Input      */
#define HAL_GPIO_AF7_USART2_TX           HAL_GPIO_AF_7 /*!< USART2 Transmit Data Output           */

/**
  * @brief   AF 8 selection
  */
#define HAL_GPIO_AF8_I2C1                HAL_GPIO_AF_8 /*!< I2C1 Alternate Function mapping      */
#define HAL_GPIO_AF8_I2C1_SCL            HAL_GPIO_AF_8 /*!< I2C1 Clock                           */
#define HAL_GPIO_AF8_I2C1_SDA            HAL_GPIO_AF_8 /*!< I2C1 Data                            */
#define HAL_GPIO_AF8_LPUART1             HAL_GPIO_AF_8 /*!< LPUART1 Alternate Function mapping   */
#define HAL_GPIO_AF8_LPUART1_CTS         HAL_GPIO_AF_8 /*!< LPUART1 Clear to send                */
#define HAL_GPIO_AF8_LPUART1_RTS         HAL_GPIO_AF_8 /*!< LPUART1 Request to send              */
#define HAL_GPIO_AF8_LPUART1_RTS_DE      HAL_GPIO_AF8_LPUART1_RTS /*!< Alias to LPUART1_RTS      */
#define HAL_GPIO_AF8_LPUART1_RX          HAL_GPIO_AF_8 /*!< LPUART1 Serial Data Receive Input    */
#define HAL_GPIO_AF8_LPUART1_TX          HAL_GPIO_AF_8 /*!< LPUART1 Transmit Data Output         */
#define HAL_GPIO_AF8_UART4               HAL_GPIO_AF_8 /*!< UART4 Alternate Function mapping     */
#define HAL_GPIO_AF8_UART4_CTS           HAL_GPIO_AF_8 /*!< UART4 Clear to send                  */
#define HAL_GPIO_AF8_UART4_RTS           HAL_GPIO_AF_8 /*!< UART4 Request to send                */
#define HAL_GPIO_AF8_UART4_RTS_DE        HAL_GPIO_AF8_UART4_RTS /*!< Alias to UART4_RTS          */
#define HAL_GPIO_AF8_UART4_RX            HAL_GPIO_AF_8 /*!< UART4 Serial Data Receive Input      */
#define HAL_GPIO_AF8_UART4_TX            HAL_GPIO_AF_8 /*!< UART4 Transmit Data Output           */
#define HAL_GPIO_AF8_UART5               HAL_GPIO_AF_8 /*!< UART5 Alternate Function mapping     */
#define HAL_GPIO_AF8_UART5_CTS           HAL_GPIO_AF_8 /*!< UART5 Clear to send                  */
#define HAL_GPIO_AF8_UART5_RTS           HAL_GPIO_AF_8 /*!< UART5 Request to send                */
#define HAL_GPIO_AF8_UART5_RTS_DE        HAL_GPIO_AF8_UART5_RTS /*!< Alias to UART5_RTS          */
#define HAL_GPIO_AF8_UART5_RX            HAL_GPIO_AF_8 /*!< UART5 Serial Data Receive Input      */
#define HAL_GPIO_AF8_UART5_TX            HAL_GPIO_AF_8 /*!< UART5 Transmit Data Output           */

/**
  * @brief   AF 9 selection
  */
#define HAL_GPIO_AF9_FDCAN1              HAL_GPIO_AF_9 /*!< FDCAN1 Alternate Function mapping    */
#define HAL_GPIO_AF9_FDCAN1_RX           HAL_GPIO_AF_9 /*!< FDCAN1 Receive pin                   */
#define HAL_GPIO_AF9_FDCAN1_TX           HAL_GPIO_AF_9 /*!< FDCAN1 Transmit pin                  */
#define HAL_GPIO_AF9_FDCAN2              HAL_GPIO_AF_9 /*!< FDCAN2 Alternate Function mapping    */
#define HAL_GPIO_AF9_FDCAN2_RX           HAL_GPIO_AF_9 /*!< FDCAN2 Receive pin                   */
#define HAL_GPIO_AF9_FDCAN2_TX           HAL_GPIO_AF_9 /*!< FDCAN2 Transmit pin                  */
#define HAL_GPIO_AF9_I2C1                HAL_GPIO_AF_9 /*!< I2C1 Alternate Function mapping      */
#define HAL_GPIO_AF9_I2C1_SCL            HAL_GPIO_AF_9 /*!< I2C1 Clock                           */
#define HAL_GPIO_AF9_I2C1_SDA            HAL_GPIO_AF_9 /*!< I2C1 Data                            */
#define HAL_GPIO_AF9_I2C1_SMBA           HAL_GPIO_AF_9 /*!< I2C1 SMBus Alert                     */
#define HAL_GPIO_AF9_I2S2                HAL_GPIO_AF_9 /*!< I2S2 Alternate Function mapping      */
#define HAL_GPIO_AF9_I2S2_WS             HAL_GPIO_AF_9 /*!< I2S2 Word Select                     */
#define HAL_GPIO_AF9_SPI2                HAL_GPIO_AF_9 /*!< SPI2 Alternate Function mapping      */
#define HAL_GPIO_AF9_SPI2_NSS            HAL_GPIO_AF_9 /*!< SPI2 Slave Selection                 */

/**
  * @brief   AF 10 selection
  */
#define HAL_GPIO_AF10_CRS                HAL_GPIO_AF_10 /*!< CRS Alternate Function mapping      */
#define HAL_GPIO_AF10_CRS_SYNC           HAL_GPIO_AF_10 /*!< CRS synchronization                 */

/**
  * @brief   AF 11 selection
  */
#define HAL_GPIO_AF11_USART1             HAL_GPIO_AF_11 /*!< USART1 Alternate Function mapping   */
#define HAL_GPIO_AF11_USART1_CK          HAL_GPIO_AF_11 /*!< USART1 Clock                        */
#define HAL_GPIO_AF11_USART1_CTS         HAL_GPIO_AF_11 /*!< USART1 Clear to send                */
#define HAL_GPIO_AF11_USART1_NSS         HAL_GPIO_AF_11 /*!< USART1 Slave Selection              */
#define HAL_GPIO_AF11_USART1_RTS         HAL_GPIO_AF_11 /*!< USART1 Request to send              */
#define HAL_GPIO_AF11_USART1_RX          HAL_GPIO_AF_11 /*!< USART1 Serial Data Receive Input    */
#define HAL_GPIO_AF11_USART1_TX          HAL_GPIO_AF_11 /*!< USART1 Transmit Data Output         */

/**
  * @brief   AF 13 selection
  */
#define HAL_GPIO_AF13_TIM8               HAL_GPIO_AF_13 /*!< TIM8 Alternate Function mapping     */
#define HAL_GPIO_AF13_TIM8_CH1           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 1        */
#define HAL_GPIO_AF13_TIM8_CH2           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 2        */
#define HAL_GPIO_AF13_TIM8_CH2N          HAL_GPIO_AF_13 /*!< TIM8 CH2 complementary output       */
#define HAL_GPIO_AF13_TIM8_CH3           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 3        */
#define HAL_GPIO_AF13_TIM8_CH4           HAL_GPIO_AF_13 /*!< TIM8 Multi-purpose Channel 4        */
#define HAL_GPIO_AF13_TIM8_CH4N          HAL_GPIO_AF_13 /*!< TIM8 CH4 complementary output       */
#define HAL_GPIO_AF13_USB                HAL_GPIO_AF_13 /*!< USB Alternate Function mapping      */
#define HAL_GPIO_AF13_USB_SOF            HAL_GPIO_AF_13 /*!< USB Start of Frame                  */

/**
  * @brief   AF 14 selection
  */
#define HAL_GPIO_AF14_COMP1              HAL_GPIO_AF_14 /*!< COMP1 Alternate Function mapping    */
#define HAL_GPIO_AF14_COMP1_OUT          HAL_GPIO_AF_14 /*!< COMP1 Output channel                */
#define HAL_GPIO_AF14_COMP2              HAL_GPIO_AF_14 /*!< COMP2 Alternate Function mapping    */
#define HAL_GPIO_AF14_COMP2_OUT          HAL_GPIO_AF_14 /*!< COMP2 Output channel                */
#define HAL_GPIO_AF14_TIM2               HAL_GPIO_AF_14 /*!< TIM2 Alternate Function mapping     */
#define HAL_GPIO_AF14_TIM2_CH4           HAL_GPIO_AF_14 /*!< TIM2 Multi-purpose Channel 4        */
#define HAL_GPIO_AF14_TIM2_ETR           HAL_GPIO_AF_14 /*!< TIM2 External trigger input         */
#define HAL_GPIO_AF14_UART5              HAL_GPIO_AF_14 /*!< UART5 Alternate Function mapping    */
#define HAL_GPIO_AF14_UART5_CTS          HAL_GPIO_AF_14 /*!< UART5 Clear to send                 */
#define HAL_GPIO_AF14_UART5_RTS          HAL_GPIO_AF_14 /*!< UART5 Request to send               */
#define HAL_GPIO_AF14_UART5_RTS_DE       HAL_GPIO_AF14_UART5_RTS /*!< Alias to UART5_RTS         */
#define HAL_GPIO_AF14_UART5_RX           HAL_GPIO_AF_14 /*!< UART5 Serial Data Receive Input     */
#define HAL_GPIO_AF14_UART5_TX           HAL_GPIO_AF_14 /*!< UART5 Transmit Data Output          */

/**
  * @brief   AF 15 selection
  */
#define HAL_GPIO_AF15_EVENTOUT           HAL_GPIO_AF_15 /*!< EVENTOUT Alternate Function mapping */
#endif /* STM32C531xx || STM32C532xx || STM32C542xx */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Functions HAL GPIO Functions
  * @{
  */

/** @defgroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
  * @{
  */

hal_status_t  HAL_GPIO_Init(hal_gpio_t gpiox, uint32_t pins, const hal_gpio_config_t *p_config);
void          HAL_GPIO_DeInit(hal_gpio_t  gpiox, uint32_t pins);

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions_Group2 IO operation functions
  * @{
  */

hal_gpio_pin_state_t HAL_GPIO_ReadPin(hal_gpio_t gpiox, uint32_t pin);
void HAL_GPIO_WritePin(hal_gpio_t gpiox, uint32_t pins, hal_gpio_pin_state_t pin_state);
void HAL_GPIO_WriteMultipleStatePin(hal_gpio_t gpiox, uint32_t pins_reset, uint32_t pins_set);
void HAL_GPIO_TogglePin(hal_gpio_t gpiox, uint32_t pins);
hal_status_t HAL_GPIO_LockPin(hal_gpio_t gpiox, uint32_t pins);
GPIO_TypeDef *HAL_GPIO_GetLLInstance(hal_gpio_t gpiox);
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

#endif /* STM32C5XX_HAL_GPIO_H */
