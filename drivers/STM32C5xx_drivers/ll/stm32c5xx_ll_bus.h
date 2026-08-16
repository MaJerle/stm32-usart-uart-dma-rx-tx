/**
  ******************************************************************************
  * @file    stm32c5xx_ll_bus.h
  * @brief   Header file of LL BUS module.

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
  @verbatim
                      ##### RCC Limitations #####
  ==============================================================================
    [..]
      A delay between an RCC peripheral clock enable and the effective peripheral
      enabling must be taken into account in order to manage the peripheral read/write
      from/to registers.
      (+) This delay depends on the peripheral mapping.
        (++) AHB, APB peripherals, one dummy read is necessary

    [..]
      Workarounds:
      (#) For AHB, APB peripherals, a dummy read of the peripheral register has been
          inserted in each LL_{BUS}_GRP{x}_EnableClock() function.

  @endverbatim
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_LL_BUS_H
#define STM32C5XX_LL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5XX_LL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup BUS_LL BUS
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Constants LL BUS Constants
  * @{
  */

/** @defgroup BUS_LL_EC_AHB1_GRP1_PERIPH  AHB1 GRP1 PERIPH
  * @{
  */
#define LL_AHB1_GRP1_PERIPH_ALL           0xFFFFFFFFU
#if defined(ETH1)
#define LL_AHB1_GRP1_PERIPH_ALL_EN_MASK   (RCC_AHB1ENR_LPDMA1EN | RCC_AHB1ENR_LPDMA2EN | RCC_AHB1ENR_FLASHEN  | \
                                           RCC_AHB1ENR_CRCEN    | RCC_AHB1ENR_CORDICEN | RCC_AHB1ENR_RAMCFGEN | \
                                           RCC_AHB1ENR_ETH1CKEN | RCC_AHB1ENR_ETH1EN   | RCC_AHB1ENR_ETH1TXEN | \
                                           RCC_AHB1ENR_ETH1RXEN | RCC_AHB1ENR_SRAM2EN  | RCC_AHB1ENR_SRAM1EN)
#define LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK (RCC_AHB1LPENR_LPDMA1LPEN | RCC_AHB1LPENR_LPDMA2LPEN  | \
                                           RCC_AHB1LPENR_FLASHLPEN  | RCC_AHB1LPENR_CRCLPEN     | \
                                           RCC_AHB1LPENR_CORDICLPEN | RCC_AHB1LPENR_RAMCFGLPEN  | \
                                           RCC_AHB1LPENR_ICACHELPEN | RCC_AHB1LPENR_ETH1CLKLPEN | \
                                           RCC_AHB1LPENR_ETH1LPEN   | RCC_AHB1LPENR_ETH1TXLPEN  | \
                                           RCC_AHB1LPENR_ETH1RXLPEN |RCC_AHB1LPENR_SRAM2LPEN    | \
                                           RCC_AHB1LPENR_SRAM1LPEN)
#define LL_AHB1_GRP1_PERIPH_ALL_RST_MASK  (RCC_AHB1RSTR_LPDMA1RST | RCC_AHB1RSTR_LPDMA2RST | RCC_AHB1RSTR_CRCRST | \
                                           RCC_AHB1RSTR_CORDICRST | RCC_AHB1RSTR_RAMCFGRST | RCC_AHB1RSTR_ETH1RST )
#else
#define LL_AHB1_GRP1_PERIPH_ALL_EN_MASK   (RCC_AHB1ENR_LPDMA1EN | RCC_AHB1ENR_LPDMA2EN | RCC_AHB1ENR_FLASHEN  | \
                                           RCC_AHB1ENR_CRCEN    | RCC_AHB1ENR_CORDICEN | RCC_AHB1ENR_RAMCFGEN | \
                                           RCC_AHB1ENR_SRAM2EN  | RCC_AHB1ENR_SRAM1EN)
#define LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK (RCC_AHB1LPENR_LPDMA1LPEN | RCC_AHB1LPENR_LPDMA2LPEN | \
                                           RCC_AHB1LPENR_FLASHLPEN  | RCC_AHB1LPENR_CRCLPEN    | \
                                           RCC_AHB1LPENR_CORDICLPEN | RCC_AHB1LPENR_RAMCFGLPEN | \
                                           RCC_AHB1LPENR_ICACHELPEN | RCC_AHB1LPENR_SRAM2LPEN  | \
                                           RCC_AHB1LPENR_SRAM1LPEN)
#define LL_AHB1_GRP1_PERIPH_ALL_RST_MASK  (RCC_AHB1RSTR_LPDMA1RST | RCC_AHB1RSTR_LPDMA2RST | RCC_AHB1RSTR_CRCRST | \
                                           RCC_AHB1RSTR_CORDICRST | RCC_AHB1RSTR_RAMCFGRST)
#endif /* ETH1 */
#define LL_AHB1_GRP1_PERIPH_LPDMA1        RCC_AHB1ENR_LPDMA1EN
#define LL_AHB1_GRP1_PERIPH_LPDMA2        RCC_AHB1ENR_LPDMA2EN
#define LL_AHB1_GRP1_PERIPH_FLASH         RCC_AHB1ENR_FLASHEN
#define LL_AHB1_GRP1_PERIPH_CRC           RCC_AHB1ENR_CRCEN
#define LL_AHB1_GRP1_PERIPH_CORDIC        RCC_AHB1ENR_CORDICEN
#define LL_AHB1_GRP1_PERIPH_RAMCFG        RCC_AHB1ENR_RAMCFGEN
#define LL_AHB1_GRP1_PERIPH_ICACHE1       RCC_AHB1LPENR_ICACHELPEN
#if defined(ETH1)
#define LL_AHB1_GRP1_PERIPH_ETH1CK        RCC_AHB1ENR_ETH1CKEN
#define LL_AHB1_GRP1_PERIPH_ETH1          RCC_AHB1ENR_ETH1EN
#define LL_AHB1_GRP1_PERIPH_ETH1TX        RCC_AHB1ENR_ETH1TXEN
#define LL_AHB1_GRP1_PERIPH_ETH1RX        RCC_AHB1ENR_ETH1RXEN
#endif /* ETH1 */
#define LL_AHB1_GRP1_PERIPH_SRAM2         RCC_AHB1ENR_SRAM2EN
#define LL_AHB1_GRP1_PERIPH_SRAM1         RCC_AHB1ENR_SRAM1EN
/**
  * @}
  */ /* End of BUS_LL_EC_AHB1_GRP1_PERIPH */

/** @defgroup BUS_LL_EC_AHB2_GRP1_PERIPH  AHB2 GRP1 PERIPH
  * @{
  */
#define LL_AHB2_GRP1_PERIPH_ALL           0xFFFFFFFFU
#if defined(GPIOF)
#if defined(AES)
#define LL_AHB2_GRP1_PERIPH_ALL_EN_MASK    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | \
                                            RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOFEN | \
                                            RCC_AHB2ENR_GPIOGEN | RCC_AHB2ENR_GPIOHEN | RCC_AHB2ENR_ADC12EN | \
                                            RCC_AHB2ENR_DAC1EN  | RCC_AHB2ENR_AESEN   | RCC_AHB2ENR_HASHEN  | \
                                            RCC_AHB2ENR_RNGEN   | RCC_AHB2ENR_PKAEN   | RCC_AHB2ENR_SAESEN  | \
                                            RCC_AHB2ENR_CCBEN   | RCC_AHB2ENR_ADC3EN)
#define LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK  (RCC_AHB2LPENR_GPIOALPEN | RCC_AHB2LPENR_GPIOBLPEN | \
                                            RCC_AHB2LPENR_GPIOCLPEN | RCC_AHB2LPENR_GPIODLPEN | \
                                            RCC_AHB2LPENR_GPIOELPEN | RCC_AHB2LPENR_GPIOFLPEN | \
                                            RCC_AHB2LPENR_GPIOGLPEN | RCC_AHB2LPENR_GPIOHLPEN | \
                                            RCC_AHB2LPENR_ADC12LPEN | RCC_AHB2LPENR_DAC1LPEN  | \
                                            RCC_AHB2LPENR_AESLPEN   | RCC_AHB2LPENR_HASHLPEN  | \
                                            RCC_AHB2LPENR_RNGLPEN   | RCC_AHB2LPENR_PKALPEN   | \
                                            RCC_AHB2LPENR_SAESLPEN  | RCC_AHB2LPENR_CCBLPEN   | \
                                            RCC_AHB2LPENR_ADC3LPEN)
#define LL_AHB2_GRP1_PERIPH_ALL_RST_MASK   (RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST | RCC_AHB2RSTR_GPIOCRST | \
                                            RCC_AHB2RSTR_GPIODRST | RCC_AHB2RSTR_GPIOERST | RCC_AHB2RSTR_GPIOFRST | \
                                            RCC_AHB2RSTR_GPIOGRST | RCC_AHB2RSTR_GPIOHRST | RCC_AHB2RSTR_ADC12RST | \
                                            RCC_AHB2RSTR_DAC1RST  | RCC_AHB2RSTR_AESRST   | RCC_AHB2RSTR_HASHRST  | \
                                            RCC_AHB2RSTR_RNGRST   | RCC_AHB2RSTR_PKARST   | RCC_AHB2RSTR_SAESRST  | \
                                            RCC_AHB2RSTR_CCBRST   | RCC_AHB2RSTR_ADC3RST)
#else
#define LL_AHB2_GRP1_PERIPH_ALL_EN_MASK    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | \
                                            RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOFEN | \
                                            RCC_AHB2ENR_GPIOGEN | RCC_AHB2ENR_GPIOHEN | RCC_AHB2ENR_ADC12EN | \
                                            RCC_AHB2ENR_DAC1EN  | RCC_AHB2ENR_HASHEN  | RCC_AHB2ENR_RNGEN   | \
                                            RCC_AHB2ENR_PKAEN   | RCC_AHB2ENR_ADC3EN)
#define LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK  (RCC_AHB2LPENR_GPIOALPEN | RCC_AHB2LPENR_GPIOBLPEN | \
                                            RCC_AHB2LPENR_GPIOCLPEN | RCC_AHB2LPENR_GPIODLPEN | \
                                            RCC_AHB2LPENR_GPIOELPEN | RCC_AHB2LPENR_GPIOFLPEN | \
                                            RCC_AHB2LPENR_GPIOGLPEN | RCC_AHB2LPENR_GPIOHLPEN | \
                                            RCC_AHB2LPENR_ADC12LPEN | RCC_AHB2LPENR_DAC1LPEN  | \
                                            RCC_AHB2LPENR_HASHLPEN  | RCC_AHB2LPENR_RNGLPEN   | \
                                            RCC_AHB2LPENR_PKALPEN   | RCC_AHB2LPENR_ADC3LPEN)
#define LL_AHB2_GRP1_PERIPH_ALL_RST_MASK   (RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST | RCC_AHB2RSTR_GPIOCRST | \
                                            RCC_AHB2RSTR_GPIODRST | RCC_AHB2RSTR_GPIOERST | RCC_AHB2RSTR_GPIOFRST | \
                                            RCC_AHB2RSTR_GPIOGRST | RCC_AHB2RSTR_GPIOHRST | RCC_AHB2RSTR_ADC12RST | \
                                            RCC_AHB2RSTR_DAC1RST  | RCC_AHB2RSTR_HASHRST  | RCC_AHB2RSTR_RNGRST   | \
                                            RCC_AHB2RSTR_PKARST  | RCC_AHB2RSTR_ADC3RST)
#endif /* AES */
#else
#if defined(AES)
#define LL_AHB2_GRP1_PERIPH_ALL_EN_MASK    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | \
                                            RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOHEN | \
                                            RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_DAC1EN  | RCC_AHB2ENR_AESEN | \
                                            RCC_AHB2ENR_HASHEN  | RCC_AHB2ENR_RNGEN)
#define LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK  (RCC_AHB2LPENR_GPIOALPEN | RCC_AHB2LPENR_GPIOBLPEN | \
                                            RCC_AHB2LPENR_GPIOCLPEN | RCC_AHB2LPENR_GPIODLPEN | \
                                            RCC_AHB2LPENR_GPIOELPEN | RCC_AHB2LPENR_GPIOHLPEN | \
                                            RCC_AHB2LPENR_ADC12LPEN | RCC_AHB2LPENR_DAC1LPEN  | \
                                            RCC_AHB2LPENR_AESLPEN   | RCC_AHB2LPENR_HASHLPEN  | \
                                            RCC_AHB2LPENR_RNGLPEN)
#define LL_AHB2_GRP1_PERIPH_ALL_RST_MASK   (RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST | RCC_AHB2RSTR_GPIOCRST | \
                                            RCC_AHB2RSTR_GPIODRST | RCC_AHB2RSTR_GPIOERST | RCC_AHB2RSTR_GPIOHRST | \
                                            RCC_AHB2RSTR_ADC12RST | RCC_AHB2RSTR_DAC1RST  | RCC_AHB2RSTR_AESRST | \
                                            RCC_AHB2RSTR_HASHRST  | RCC_AHB2RSTR_RNGRST)
#else
#define LL_AHB2_GRP1_PERIPH_ALL_EN_MASK    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | \
                                            RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOHEN | \
                                            RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_DAC1EN  | \
                                            RCC_AHB2ENR_HASHEN  | RCC_AHB2ENR_RNGEN)
#define LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK  (RCC_AHB2LPENR_GPIOALPEN | RCC_AHB2LPENR_GPIOBLPEN | \
                                            RCC_AHB2LPENR_GPIOCLPEN | RCC_AHB2LPENR_GPIODLPEN | \
                                            RCC_AHB2LPENR_GPIOELPEN | RCC_AHB2LPENR_GPIOHLPEN | \
                                            RCC_AHB2LPENR_ADC12LPEN | RCC_AHB2LPENR_DAC1LPEN  | \
                                            RCC_AHB2LPENR_HASHLPEN  | RCC_AHB2LPENR_RNGLPEN)
#define LL_AHB2_GRP1_PERIPH_ALL_RST_MASK   (RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST | RCC_AHB2RSTR_GPIOCRST | \
                                            RCC_AHB2RSTR_GPIODRST | RCC_AHB2RSTR_GPIOERST | RCC_AHB2RSTR_GPIOHRST | \
                                            RCC_AHB2RSTR_ADC12RST | RCC_AHB2RSTR_DAC1RST  | \
                                            RCC_AHB2RSTR_HASHRST  | RCC_AHB2RSTR_RNGRST)
#endif /* AES */
#endif /* GPIOF */
#define LL_AHB2_GRP1_PERIPH_GPIOA         RCC_AHB2ENR_GPIOAEN
#define LL_AHB2_GRP1_PERIPH_GPIOB         RCC_AHB2ENR_GPIOBEN
#define LL_AHB2_GRP1_PERIPH_GPIOC         RCC_AHB2ENR_GPIOCEN
#define LL_AHB2_GRP1_PERIPH_GPIOD         RCC_AHB2ENR_GPIODEN
#define LL_AHB2_GRP1_PERIPH_GPIOE         RCC_AHB2ENR_GPIOEEN
#if defined(GPIOF)
#define LL_AHB2_GRP1_PERIPH_GPIOF         RCC_AHB2ENR_GPIOFEN
#endif /* GPIOF */
#if defined(GPIOG)
#define LL_AHB2_GRP1_PERIPH_GPIOG         RCC_AHB2ENR_GPIOGEN
#endif /* GPIOG */
#define LL_AHB2_GRP1_PERIPH_GPIOH         RCC_AHB2ENR_GPIOHEN
#define LL_AHB2_GRP1_PERIPH_ADC12         RCC_AHB2ENR_ADC12EN
#define LL_AHB2_GRP1_PERIPH_DAC1          RCC_AHB2ENR_DAC1EN
#if defined(AES)
#define LL_AHB2_GRP1_PERIPH_AES           RCC_AHB2ENR_AESEN
#endif /* AES */
#define LL_AHB2_GRP1_PERIPH_HASH          RCC_AHB2ENR_HASHEN
#define LL_AHB2_GRP1_PERIPH_RNG           RCC_AHB2ENR_RNGEN
#if defined(PKA)
#define LL_AHB2_GRP1_PERIPH_PKA           RCC_AHB2ENR_PKAEN
#else
#define LL_AHB2_GRP1_PERIPH_PKA
#endif /* PKA */
#if defined(SAES)
#define LL_AHB2_GRP1_PERIPH_SAES          RCC_AHB2ENR_SAESEN
#else
#define LL_AHB2_GRP1_PERIPH_SAES
#endif /* SAES */
#if defined(CCB)
#define LL_AHB2_GRP1_PERIPH_CCB           RCC_AHB2ENR_CCBEN
#else
#define LL_AHB2_GRP1_PERIPH_CCB
#endif /* CCB */
#if defined(ADC3)
#define LL_AHB2_GRP1_PERIPH_ADC3          RCC_AHB2ENR_ADC3EN
#endif /* ADC3 */
/**
  * @}
  */ /* End of BUS_LL_EC_AHB2_GRP1_PERIPH */

#if defined(AHB4PERIPH_BASE)
/** @defgroup BUS_LL_EC_AHB4_GRP1_PERIPH  AHB4 GRP1 PERIPH
  * @{
  */
#define LL_AHB4_GRP1_PERIPH_ALL           0xFFFFFFFFU
#define LL_AHB4_GRP1_PERIPH_ALL_EN_MASK   RCC_AHB4ENR_XSPI1EN
#define LL_AHB4_GRP1_PERIPH_ALL_LPEN_MASK RCC_AHB4LPENR_XSPI1LPEN
#define LL_AHB4_GRP1_PERIPH_ALL_RST_MASK  RCC_AHB4RSTR_XSPI1RST
#define LL_AHB4_GRP1_PERIPH_XSPI1         RCC_AHB4ENR_XSPI1EN
/**
  * @}
  */ /* End of BUS_LL_EC_AHB4_GRP1_PERIPH */
#endif /* AHB4PERIPH_BASE */

/** @defgroup BUS_LL_EC_APB1_GRP1_PERIPH  APB1 GRP1 PERIPH
  * @{
  */
#define LL_APB1_GRP1_PERIPH_ALL           0xFFFFFFFFU
#if defined(TIM3)
#define LL_APB1_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB1LENR_TIM2EN   | RCC_APB1LENR_TIM3EN   | RCC_APB1LENR_TIM4EN   | \
                                           RCC_APB1LENR_TIM5EN   | RCC_APB1LENR_TIM6EN   | RCC_APB1LENR_TIM7EN   | \
                                           RCC_APB1LENR_TIM12EN  | RCC_APB1LENR_WWDGEN   |  RCC_APB1LENR_SPI2EN   | \
                                           RCC_APB1LENR_SPI3EN   | RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN | \
                                           RCC_APB1LENR_UART4EN  | RCC_APB1LENR_UART5EN  | RCC_APB1LENR_I2C1EN   | \
                                           RCC_APB1LENR_I2C2EN   | RCC_APB1LENR_I3C1EN   | RCC_APB1LENR_CRSEN    | \
                                           RCC_APB1LENR_USART6EN | RCC_APB1LENR_UART7EN)
#define LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB1LLPENR_TIM2LPEN   | RCC_APB1LLPENR_TIM3LPEN   | \
                                           RCC_APB1LLPENR_TIM4LPEN   | RCC_APB1LLPENR_TIM5LPEN   | \
                                           RCC_APB1LLPENR_TIM6LPEN   | RCC_APB1LLPENR_TIM7LPEN   | \
                                           RCC_APB1LLPENR_TIM12LPEN  | RCC_APB1LLPENR_WWDGLPEN   | \
                                           RCC_APB1LLPENR_SPI2LPEN   | RCC_APB1LLPENR_SPI3LPEN   | \
                                           RCC_APB1LLPENR_USART2LPEN | RCC_APB1LLPENR_USART3LPEN | \
                                           RCC_APB1LLPENR_UART4LPEN  | RCC_APB1LLPENR_UART5LPEN  | \
                                           RCC_APB1LLPENR_I2C1LPEN   | RCC_APB1LLPENR_I2C2LPEN   | \
                                           RCC_APB1LLPENR_I3C1LPEN   | RCC_APB1LLPENR_CRSLPEN    | \
                                           RCC_APB1LLPENR_USART6LPEN | RCC_APB1LLPENR_UART7LPEN)
#define LL_APB1_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB1LRSTR_TIM2RST   | RCC_APB1LRSTR_TIM3RST   | \
                                           RCC_APB1LRSTR_TIM4RST   | RCC_APB1LRSTR_TIM5RST   | \
                                           RCC_APB1LRSTR_TIM6RST   | RCC_APB1LRSTR_TIM7RST   | \
                                           RCC_APB1LRSTR_TIM12RST  | RCC_APB1LRSTR_SPI2RST   | \
                                           RCC_APB1LRSTR_SPI3RST   | RCC_APB1LRSTR_USART2RST | \
                                           RCC_APB1LRSTR_USART3RST | RCC_APB1LRSTR_UART4RST  | \
                                           RCC_APB1LRSTR_UART5RST  | RCC_APB1LRSTR_I2C1RST   | \
                                           RCC_APB1LRSTR_I2C2RST   | RCC_APB1LRSTR_I3C1RST   | \
                                           RCC_APB1LRSTR_CRSRST    | RCC_APB1LRSTR_USART6RST | \
                                           RCC_APB1LRSTR_UART7RST)
#elif defined(TIM5)
#define LL_APB1_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB1LENR_TIM2EN | RCC_APB1LENR_TIM5EN | RCC_APB1LENR_TIM6EN | \
                                           RCC_APB1LENR_TIM7EN | RCC_APB1LENR_TIM12EN | RCC_APB1LENR_WWDGEN | \
                                           RCC_APB1LENR_SPI2EN | RCC_APB1LENR_SPI3EN | RCC_APB1LENR_USART2EN | \
                                           RCC_APB1LENR_USART3EN | RCC_APB1LENR_UART4EN | RCC_APB1LENR_UART5EN | \
                                           RCC_APB1LENR_I2C1EN | RCC_APB1LENR_I2C2EN | RCC_APB1LENR_I3C1EN | \
                                           RCC_APB1LENR_CRSEN)
#define LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB1LLPENR_TIM2LPEN | RCC_APB1LLPENR_TIM5LPEN | \
                                           RCC_APB1LLPENR_TIM6LPEN | RCC_APB1LLPENR_TIM7LPEN | \
                                           RCC_APB1LLPENR_TIM12LPEN | RCC_APB1LLPENR_WWDGLPEN | \
                                           RCC_APB1LLPENR_SPI2LPEN | RCC_APB1LLPENR_SPI3LPEN | \
                                           RCC_APB1LLPENR_USART2LPEN | RCC_APB1LLPENR_USART3LPEN | \
                                           RCC_APB1LLPENR_UART4LPEN | RCC_APB1LLPENR_UART5LPEN | \
                                           RCC_APB1LLPENR_I2C1LPEN | RCC_APB1LLPENR_I2C2LPEN | \
                                           RCC_APB1LLPENR_I3C1LPEN | RCC_APB1LLPENR_CRSLPEN)
#define LL_APB1_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB1LRSTR_TIM2RST | RCC_APB1LRSTR_TIM5RST | RCC_APB1LRSTR_TIM6RST | \
                                           RCC_APB1LRSTR_TIM7RST | RCC_APB1LRSTR_TIM12RST | RCC_APB1LRSTR_SPI2RST | \
                                           RCC_APB1LRSTR_SPI3RST | RCC_APB1LRSTR_USART2RST | RCC_APB1LRSTR_USART3RST |\
                                           RCC_APB1LRSTR_UART4RST | RCC_APB1LRSTR_UART5RST | RCC_APB1LRSTR_I2C1RST | \
                                           RCC_APB1LRSTR_I2C2RST | RCC_APB1LRSTR_I3C1RST | RCC_APB1LRSTR_CRSRST)
#else
#define LL_APB1_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB1LENR_TIM2EN | RCC_APB1LENR_TIM6EN | RCC_APB1LENR_TIM7EN | \
                                           RCC_APB1LENR_TIM12EN | RCC_APB1LENR_WWDGEN | RCC_APB1LENR_OPAMP1EN | \
                                           RCC_APB1LENR_SPI2EN | RCC_APB1LENR_USART2EN | RCC_APB1LENR_UART4EN | \
                                           RCC_APB1LENR_UART5EN | RCC_APB1LENR_I2C1EN | RCC_APB1LENR_I3C1EN | \
                                           RCC_APB1LENR_CRSEN)
#define LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB1LLPENR_TIM2LPEN | RCC_APB1LLPENR_TIM6LPEN | \
                                           RCC_APB1LLPENR_TIM7LPEN | RCC_APB1LLPENR_TIM12LPEN | \
                                           RCC_APB1LLPENR_WWDGLPEN | RCC_APB1LLPENR_OPAMP1LPEN | \
                                           RCC_APB1LLPENR_SPI2LPEN | RCC_APB1LLPENR_USART2LPEN | \
                                           RCC_APB1LLPENR_UART4LPEN | RCC_APB1LLPENR_UART5LPEN | \
                                           RCC_APB1LLPENR_I2C1LPEN | RCC_APB1LLPENR_I3C1LPEN | \
                                           RCC_APB1LLPENR_CRSLPEN)
#define LL_APB1_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB1LRSTR_TIM2RST | RCC_APB1LRSTR_TIM6RST | RCC_APB1LRSTR_TIM7RST | \
                                           RCC_APB1LRSTR_TIM12RST | RCC_APB1LRSTR_OPAMP1RST | RCC_APB1LRSTR_SPI2RST | \
                                           RCC_APB1LRSTR_USART2RST | RCC_APB1LRSTR_UART4RST | RCC_APB1LRSTR_UART5RST |\
                                           RCC_APB1LRSTR_I2C1RST | RCC_APB1LRSTR_I3C1RST | RCC_APB1LRSTR_CRSRST)
#endif /* TIM3 */
#define LL_APB1_GRP1_PERIPH_TIM2          RCC_APB1LENR_TIM2EN
#if defined(TIM3)
#define LL_APB1_GRP1_PERIPH_TIM3          RCC_APB1LENR_TIM3EN
#endif /* TIM3 */
#if defined(TIM4)
#define LL_APB1_GRP1_PERIPH_TIM4          RCC_APB1LENR_TIM4EN
#endif /* TIM4 */
#if defined(TIM5)
#define LL_APB1_GRP1_PERIPH_TIM5          RCC_APB1LENR_TIM5EN
#endif /* TIM5 */
#define LL_APB1_GRP1_PERIPH_TIM6          RCC_APB1LENR_TIM6EN
#define LL_APB1_GRP1_PERIPH_TIM7          RCC_APB1LENR_TIM7EN
#define LL_APB1_GRP1_PERIPH_TIM12         RCC_APB1LENR_TIM12EN
#define LL_APB1_GRP1_PERIPH_WWDG          RCC_APB1LENR_WWDGEN
#if defined(OPAMP1)
#define LL_APB1_GRP1_PERIPH_OPAMP1        RCC_APB1LENR_OPAMP1EN
#endif /* OPAMP1 */
#define LL_APB1_GRP1_PERIPH_SPI2          RCC_APB1LENR_SPI2EN
#if defined(SPI3)
#define LL_APB1_GRP1_PERIPH_SPI3          RCC_APB1LENR_SPI3EN
#endif /* SPI3 */
#define LL_APB1_GRP1_PERIPH_USART2        RCC_APB1LENR_USART2EN
#if defined(USART3)
#define LL_APB1_GRP1_PERIPH_USART3        RCC_APB1LENR_USART3EN
#endif /* USART3 */
#define LL_APB1_GRP1_PERIPH_UART4         RCC_APB1LENR_UART4EN
#define LL_APB1_GRP1_PERIPH_UART5         RCC_APB1LENR_UART5EN
#define LL_APB1_GRP1_PERIPH_I2C1          RCC_APB1LENR_I2C1EN
#if defined(I2C2)
#define LL_APB1_GRP1_PERIPH_I2C2          RCC_APB1LENR_I2C2EN
#endif /* I2C2 */
#define LL_APB1_GRP1_PERIPH_I3C1          RCC_APB1LENR_I3C1EN
#define LL_APB1_GRP1_PERIPH_CRS           RCC_APB1LENR_CRSEN
#if defined(USART6)
#define LL_APB1_GRP1_PERIPH_USART6        RCC_APB1LENR_USART6EN
#endif /* USART6 */
#if defined(UART7)
#define LL_APB1_GRP1_PERIPH_UART7         RCC_APB1LENR_UART7EN
#endif /* UART7 */
/**
  * @}
  */ /* End of BUS_LL_EC_APB1_GRP1_PERIPH */

/** @defgroup BUS_LL_EC_APB1_GRP2_PERIPH  APB1 GRP2 PERIPH
  * @{
  */
#define LL_APB1_GRP2_PERIPH_ALL           0xFFFFFFFFU
#if defined(FDCAN1)
#define LL_APB1_GRP2_PERIPH_ALL_EN_MASK   (RCC_APB1HENR_COMP12EN | RCC_APB1HENR_FDCANEN)
#define LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK (RCC_APB1HLPENR_COMP12LPEN | RCC_APB1HLPENR_FDCANLPEN)
#define LL_APB1_GRP2_PERIPH_ALL_RST_MASK  (RCC_APB1HRSTR_COMP12RST | RCC_APB1HRSTR_FDCANRST)
#else
#define LL_APB1_GRP2_PERIPH_ALL_EN_MASK   (RCC_APB1HENR_COMP12EN)
#define LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK (RCC_APB1HLPENR_COMP12LPEN)
#define LL_APB1_GRP2_PERIPH_ALL_RST_MASK  (RCC_APB1HRSTR_COMP12RST)
#endif /* FDCAN1 */

#define LL_APB1_GRP2_PERIPH_COMP12         RCC_APB1HENR_COMP12EN
#if defined(FDCAN1)
#define LL_APB1_GRP2_PERIPH_FDCAN         RCC_APB1HENR_FDCANEN
#endif /* FDCAN1 */
/**
  * @}
  */ /* End of BUS_LL_EC_APB1_GRP2_PERIPH */

/** @defgroup BUS_LL_EC_APB2_GRP1_PERIPH  APB2 GRP1 PERIPH
  * @{
  */
#define LL_APB2_GRP1_PERIPH_ALL           0xFFFFFFFFU
#if defined(TIM16)
#define LL_APB2_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB2ENR_TIM1EN   | RCC_APB2ENR_SPI1EN  | RCC_APB2ENR_TIM8EN  | \
                                           RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN | \
                                           RCC_APB2ENR_TIM17EN  | RCC_APB2ENR_USBEN)
#define LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB2LPENR_TIM1LPEN   | RCC_APB2LPENR_SPI1LPEN   | \
                                           RCC_APB2LPENR_TIM8LPEN   | RCC_APB2LPENR_USART1LPEN | \
                                           RCC_APB2LPENR_TIM15LPEN  | RCC_APB2LPENR_TIM16LPEN  | \
                                           RCC_APB2LPENR_TIM17LPEN | RCC_APB2LPENR_USBLPEN)
#define LL_APB2_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB2RSTR_TIM1RST   | RCC_APB2RSTR_SPI1RST  | RCC_APB2RSTR_TIM8RST | \
                                           RCC_APB2RSTR_USART1RST | RCC_APB2RSTR_TIM15RST | RCC_APB2RSTR_TIM16RST | \
                                           RCC_APB2RSTR_TIM17RST  | RCC_APB2RSTR_USBRST)
#else
#define LL_APB2_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB2ENR_TIM1EN   | RCC_APB2ENR_SPI1EN  | RCC_APB2ENR_TIM8EN | \
                                           RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN | RCC_APB2ENR_USBEN)
#define LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB2LPENR_TIM1LPEN   | RCC_APB2LPENR_SPI1LPEN    | \
                                           RCC_APB2LPENR_TIM8LPEN   |  RCC_APB2LPENR_USART1LPEN | \
                                           RCC_APB2LPENR_TIM15LPEN | RCC_APB2LPENR_USBLPEN)
#define LL_APB2_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB2RSTR_TIM1RST   | RCC_APB2RSTR_SPI1RST  | RCC_APB2RSTR_TIM8RST | \
                                           RCC_APB2RSTR_USART1RST | RCC_APB2RSTR_TIM15RST | RCC_APB2RSTR_USBRST)
#endif /* TIM16 */
#define LL_APB2_GRP1_PERIPH_TIM1          RCC_APB2ENR_TIM1EN
#define LL_APB2_GRP1_PERIPH_SPI1          RCC_APB2ENR_SPI1EN
#define LL_APB2_GRP1_PERIPH_TIM8          RCC_APB2ENR_TIM8EN
#define LL_APB2_GRP1_PERIPH_USART1        RCC_APB2ENR_USART1EN
#define LL_APB2_GRP1_PERIPH_TIM15         RCC_APB2ENR_TIM15EN
#if defined(TIM16)
#define LL_APB2_GRP1_PERIPH_TIM16         RCC_APB2ENR_TIM16EN
#endif /* TIM16 */
#if defined(TIM17)
#define LL_APB2_GRP1_PERIPH_TIM17         RCC_APB2ENR_TIM17EN
#endif /* TIM17 */
#define LL_APB2_GRP1_PERIPH_USB           RCC_APB2ENR_USBEN
/**
  * @}
  */ /* End of BUS_LL_EC_APB2_GRP1_PERIPH */

/** @defgroup BUS_LL_EC_APB3_GRP1_PERIPH  APB3 GRP1 PERIPH
  * @{
  */
#define LL_APB3_GRP1_PERIPH_ALL           0xFFFFFFFFU
#define LL_APB3_GRP1_PERIPH_ALL_EN_MASK   (RCC_APB3ENR_SBSEN | RCC_APB3ENR_LPUART1EN | RCC_APB3ENR_LPTIM1EN | \
                                           RCC_APB3ENR_RTCAPBEN)
#define LL_APB3_GRP1_PERIPH_ALL_LPEN_MASK (RCC_APB3LPENR_SBSLPEN | RCC_APB3LPENR_LPUART1LPEN | \
                                           RCC_APB3LPENR_LPTIM1LPEN | RCC_APB3LPENR_RTCAPBLPEN)
#define LL_APB3_GRP1_PERIPH_ALL_RST_MASK  (RCC_APB3RSTR_SBSRST | RCC_APB3RSTR_LPUART1RST | RCC_APB3RSTR_LPTIM1RST)
#define LL_APB3_GRP1_PERIPH_SBS           RCC_APB3ENR_SBSEN
#define LL_APB3_GRP1_PERIPH_LPUART1       RCC_APB3ENR_LPUART1EN
#define LL_APB3_GRP1_PERIPH_LPTIM1        RCC_APB3ENR_LPTIM1EN
#define LL_APB3_GRP1_PERIPH_RTCAPB        RCC_APB3ENR_RTCAPBEN

/**
  * @}
  */ /* End of BUS_LL_EC_APB3_GRP1_PERIPH */

/**
  * @}
  */ /* End of BUS_LL_Exported_Constants */

/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Functions LL BUS Functions
  * @{
  */

/** @defgroup BUS_LL_EF_AHB1 AHB1
  * @{
  */
/**
  * @brief  Enable AHB1 bus clock.
  * @rmtoll
  *  CFGR2         AHB1DIS         LL_AHB1_EnableBusClock
  */
__STATIC_INLINE void LL_AHB1_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB1DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB1DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if AHB1 bus clock is enabled.
  * @rmtoll
  *  CFGR2         AHB1DIS         LL_AHB1_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB1_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB1DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable AHB1 peripheral clocks.
  * @rmtoll
  *  AHB1ENR         LPDMA1EN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         LPDMA2EN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         FLASHEN          LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         CRCEN            LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         CORDICEN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         RAMCFGEN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         ETH1CKEN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         ETH1EN           LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         ETH1TXEN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         ETH1RXEN         LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         SRAM2EN          LL_AHB1_GRP1_EnableClock \n
  *  AHB1ENR         SRAM1EN          LL_AHB1_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB1ENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB1ENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB1 peripheral clock is enabled.
  * @rmtoll
  *  AHB1ENR         LPDMA1EN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         LPDMA2EN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         FLASHEN          LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         CRCEN            LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         CORDICEN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         RAMCFGEN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         ETH1CKEN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         ETH1EN           LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         ETH1TXEN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         ETH1RXEN         LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         SRAM2EN          LL_AHB1_GRP1_IsEnabledClock \n
  *  AHB1ENR         SRAM1EN          LL_AHB1_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB1_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB1ENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_AHB1_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}


/**
  * @brief  Disable AHB1 bus clock.
  * @note   except for FLASH, SRAM1 and SRAM2.
  * @rmtoll
  *  CFGR2         AHB1DIS         LL_AHB1_DisableBusClock
  */
__STATIC_INLINE void LL_AHB1_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB1DIS);
}

/**
  * @brief  Disable AHB1 peripheral clocks.
  * @rmtoll
  *  AHB1ENR         LPDMA1EN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         LPDMA2EN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         FLASHEN          LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         CRCEN            LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         CORDICEN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         RAMCFGEN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         ETH1CKEN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         ETH1EN           LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         ETH1TXEN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         ETH1RXEN         LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         SRAM2EN          LL_AHB1_GRP1_DisableClock \n
  *  AHB1ENR         SRAM1EN          LL_AHB1_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB1ENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force AHB1 peripherals reset.
  * @rmtoll
  *  AHB1RSTR         LPDMA1RST         LL_AHB1_GRP1_ForceReset \n
  *  AHB1RSTR         LPDMA2RST         LL_AHB1_GRP1_ForceReset \n
  *  AHB1RSTR         CRCRST            LL_AHB1_GRP1_ForceReset \n
  *  AHB1RSTR         CORDICRST         LL_AHB1_GRP1_ForceReset \n
  *  AHB1RSTR         RAMCFGRST         LL_AHB1_GRP1_ForceReset \n
  *  AHB1RSTR         ETH1RST           LL_AHB1_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->AHB1RSTR, periphs & LL_AHB1_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release AHB1 peripherals reset.
  * @rmtoll
  *  AHB1RSTR         LPDMA1RST         LL_AHB1_GRP1_ReleaseReset \n
  *  AHB1RSTR         LPDMA2RST         LL_AHB1_GRP1_ReleaseReset \n
  *  AHB1RSTR         CRCRST            LL_AHB1_GRP1_ReleaseReset \n
  *  AHB1RSTR         CORDICRST         LL_AHB1_GRP1_ReleaseReset \n
  *  AHB1RSTR         RAMCFGRST         LL_AHB1_GRP1_ReleaseReset \n
  *  AHB1RSTR         ETH1RST           LL_AHB1_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->AHB1RSTR, periphs & LL_AHB1_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable AHB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB1LPENR         LPDMA1LPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         LPDMA2LPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         FLASHLPEN          LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         CRCLPEN            LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         CORDICLPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         RAMCFGLPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         ETH1CKLPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         ETH1LPEN           LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         ETH1TXLPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         ETH1RXLPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         ICACHELPEN         LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         SRAM2LPEN          LL_AHB1_GRP1_EnableClockLowPower \n
  *  AHB1LPENR         SRAM1LPEN          LL_AHB1_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ICACHE1
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB1LPENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB1LPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB1 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  AHB1LPENR         LPDMA1LPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         LPDMA2LPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         FLASHLPEN          LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         CRCLPEN            LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         CORDICLPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         RAMCFGLPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         ETH1CKLPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         ETH1LPEN           LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         ETH1TXLPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         ETH1RXLPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         ICACHELPEN         LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         SRAM2LPEN          LL_AHB1_GRP1_IsEnabledClockLowPower \n
  *  AHB1LPENR         SRAM1LPEN          LL_AHB1_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ICACHE1
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB1_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB1LPENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB1LPENR         LPDMA1LPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         LPDMA2LPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         FLASHLPEN          LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         CRCLPEN            LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         CORDICLPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         RAMCFGLPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         ETH1CKLPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         ETH1LPEN           LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         ETH1TXLPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         ETH1RXLPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         ICACHELPEN         LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         SRAM2LPEN          LL_AHB1_GRP1_DisableClockLowPower \n
  *  AHB1LPENR         SRAM1LPEN          LL_AHB1_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_LPDMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CORDIC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_RAMCFG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ICACHE1
  * @if ETH1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1CK  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1    (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1TX  (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETH1RX  (*)
  * @endif
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM1
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB1_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB1LPENR, periphs & LL_AHB1_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_AHB1 */

/** @defgroup BUS_LL_EF_AHB2_GRP1_PERIPH AHB2 GRP1 PERIPH
  * @{
  */
/**
  * @brief  Enable AHB2 bus clock.
  * @rmtoll
  *  CFGR2         AHB2DIS         LL_AHB2_EnableBusClock
  */
__STATIC_INLINE void LL_AHB2_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB2DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB2DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if AHB2 bus clock is enabled.
  * @rmtoll
  *  CFGR2         AHB2DIS         LL_AHB2_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB2_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB2DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable AHB2 peripheral clocks.
  * @rmtoll
  *  AHB2ENR         GPIOAEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOBEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOCEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIODEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOEEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOFEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOGEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         GPIOHEN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         ADC12EN         LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         DAC1EN          LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         AESEN           LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         HASHEN          LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         RNGEN           LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         PKAEN           LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         SAESEN          LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         CCBEN           LL_AHB2_GRP1_EnableClock \n
  *  AHB2ENR         ADC3EN          LL_AHB2_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB2ENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB2ENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB2 peripheral clock is enabled.
  * @rmtoll
  *  AHB2ENR         GPIOAEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOBEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOCEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIODEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOEEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOFEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOGEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         GPIOHEN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         ADC12EN         LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         DAC1EN          LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         AESEN           LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         HASHEN          LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         RNGEN           LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         PKAEN           LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         SAESEN          LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         CCBEN           LL_AHB2_GRP1_IsEnabledClock \n
  *  AHB2ENR         ADC3EN          LL_AHB2_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB2_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB2ENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_AHB2_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB2 bus clock.
  * @rmtoll
  *  CFGR2         AHB2DIS         LL_AHB2_DisableBusClock
  */
__STATIC_INLINE void LL_AHB2_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB2DIS);
}

/**
  * @brief  Disable AHB2 peripheral clocks.
  * @rmtoll
  *  AHB2ENR         GPIOAEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOBEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOCEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIODEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOEEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOFEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOGEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         GPIOHEN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         ADC12EN         LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         DAC1EN          LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         AESEN           LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         HASHEN          LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         RNGEN           LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         PKAEN           LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         SAESEN          LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         CCBEN           LL_AHB2_GRP1_DisableClock \n
  *  AHB2ENR         ADC3EN          LL_AHB2_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB2ENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force AHB2 peripherals reset.
  * @rmtoll
  *  AHB2RSTR         GPIOARST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOBRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOCRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIODRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOERST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOFRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOGRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         GPIOHRST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         ADC12RST         LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         DAC1RST          LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         AESRST           LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         HASHRST          LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         RNGRST           LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         PKARST           LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         SAESRST          LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         CCBRST           LL_AHB2_GRP1_ForceReset \n
  *  AHB2RSTR         ADC3RST          LL_AHB2_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->AHB2RSTR, periphs & LL_AHB2_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release AHB2 peripherals reset.
  * @rmtoll
  *  AHB2RSTR         GPIOARST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOBRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOCRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIODRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOERST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOFRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOGRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         GPIOHRST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         ADC12RST         LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         DAC1RST          LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         AESRST           LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         HASHRST          LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         RNGRST           LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         PKARST           LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         SAESRST          LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         CCBRST           LL_AHB2_GRP1_ReleaseReset \n
  *  AHB2RSTR         ADC3RST          LL_AHB2_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->AHB2RSTR, periphs & LL_AHB2_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable AHB2 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB2LPENR         GPIOALPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOBLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOCLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIODLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOELPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOFLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOGLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         GPIOHLPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         ADC12LPEN         LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         DAC1LPEN          LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         AESLPEN           LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         HASHLPEN          LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         RNGLPEN           LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         PKALPEN           LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         SAESLPEN          LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         CCBLPEN           LL_AHB2_GRP1_EnableClockLowPower \n
  *  AHB2LPENR         ADC3LPEN          LL_AHB2_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB2LPENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB2LPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB2 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  AHB2LPENR         GPIOALPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOBLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOCLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIODLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOELPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOFLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOGLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         GPIOHLPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         ADC12LPEN         LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         DAC1LPEN          LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         AESLPEN           LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         HASHLPEN          LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         RNGLPEN           LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         PKALPEN           LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         SAESLPEN          LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         CCBLPEN           LL_AHB2_GRP1_IsEnabledClockLowPower \n
  *  AHB2LPENR         ADC3LPEN          LL_AHB2_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB2_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB2LPENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB2 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB2LPENR         GPIOALPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOBLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOCLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIODLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOELPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOFLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOGLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         GPIOHLPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         ADC12LPEN         LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         DAC1LPEN          LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         AESLPEN           LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         HASHLPEN          LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         RNGLPEN           LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         PKALPEN           LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         SAESLPEN          LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         CCBLPEN           LL_AHB2_GRP1_DisableClockLowPower \n
  *  AHB2LPENR         ADC3LPEN          LL_AHB2_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE
  * @if GPIOF
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
  * @endif
  * @if GPIOG
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC12
  *         @arg @ref LL_AHB2_GRP1_PERIPH_DAC1
  * @if AES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_AES   (*)
  * @endif
  *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH
  *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
  * @if PKA
  *         @arg @ref LL_AHB2_GRP1_PERIPH_PKA   (*)
  * @endif
  * @if SAES
  *         @arg @ref LL_AHB2_GRP1_PERIPH_SAES  (*)
  * @endif
  * @if CCB
  *         @arg @ref LL_AHB2_GRP1_PERIPH_CCB   (*)
  * @endif
  * @if ADC3
  *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC3  (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_AHB2_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB2LPENR, periphs & LL_AHB2_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_AHB2 */

#if defined(AHB4PERIPH_BASE)
/** @defgroup BUS_LL_EF_AHB4_GRP1_PERIPH AHB4 GRP1 PERIPH
  * @{
  */
/**
  * @brief  Enable AHB4 bus clock.
  * @rmtoll
  *  CFGR2         AHB4DIS         LL_AHB4_EnableBusClock
  */
__STATIC_INLINE void LL_AHB4_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB4DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB4DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if AHB4 bus clock is enabled.
  * @rmtoll
  *  CFGR2         AHB4DIS         LL_AHB4_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB4_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_AHB4DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable AHB4 peripheral clocks.
  * @rmtoll
  *  AHB4ENR         XSPI1EN         LL_AHB4_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB4ENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB4ENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB4 peripheral clock is enabled.
  * @rmtoll
  *  AHB4ENR         XSPI1EN         LL_AHB4_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB4_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB4ENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_AHB4_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB4 bus clock.
  * @rmtoll
  *  CFGR2         AHB4DIS         LL_AHB4_DisableBusClock
  */
__STATIC_INLINE void LL_AHB4_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_AHB4DIS);
}

/**
  * @brief  Disable AHB4 peripheral clocks.
  * @rmtoll
  *  AHB4ENR         XSPI1EN         LL_AHB4_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB4ENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force AHB4 peripherals reset.
  * @rmtoll
  *  AHB4RSTR         XSPI1RST         LL_AHB4_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->AHB4RSTR, periphs & LL_AHB4_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release AHB4 peripherals reset.
  * @rmtoll
  *  AHB4RSTR         XSPI1RST         LL_AHB4_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->AHB4RSTR, periphs & LL_AHB4_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable AHB4 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB4LPENR         XSPI1LPEN         LL_AHB4_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->AHB4LPENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->AHB4LPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB4 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  AHB4LPENR         XSPI1LPEN         LL_AHB4_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB4_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->AHB4LPENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_AHB4_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB4 peripheral clocks in low-power mode.
  * @rmtoll
  *  AHB4LPENR         XSPI1LPEN         LL_AHB4_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB4_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB4_GRP1_PERIPH_XSPI1
  */
__STATIC_INLINE void LL_AHB4_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->AHB4LPENR, periphs & LL_AHB4_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_AHB4 */
#endif /* AHB4PERIPH_BASE */

/** @defgroup BUS_LL_EF_APB1 APB1
  * @{
  */
/**
  * @brief  Enable APB1 bus clock.
  * @rmtoll
  *  CFGR2         APB1DIS         LL_APB1_EnableBusClock
  */
__STATIC_INLINE void LL_APB1_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_APB1DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB1DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if APB1 bus clock is enabled.
  * @rmtoll
  *  CFGR2         APB1DIS         LL_APB1_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB1DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable APB1 peripheral clocks.
  * @rmtoll
  *  APB1LENR         TIM2EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM3EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM4EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM5EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM6EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM7EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         TIM12EN          LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         WWDGEN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         OPAMP1EN         LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         SPI2EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         SPI3EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         USART2EN         LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         USART3EN         LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         UART4EN          LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         UART5EN          LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         I2C1EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         I2C2EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         I3C1EN           LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         CRSEN            LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         USART6EN         LL_APB1_GRP1_EnableClock \n
  *  APB1LENR         UART7EN          LL_APB1_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB1LENR, periphs & LL_APB1_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB1LENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clock is enabled.
  * @rmtoll
  *  APB1LENR         TIM2EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM3EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM4EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM5EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM6EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM7EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         TIM12EN          LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         WWDGEN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         OPAMP1EN         LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         SPI2EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         SPI3EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         USART2EN         LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         USART3EN         LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         UART4EN          LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         UART5EN          LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         I2C1EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         I2C2EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         I3C1EN           LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         CRSEN            LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         USART6EN         LL_APB1_GRP1_IsEnabledClock \n
  *  APB1LENR         UART7EN          LL_APB1_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB1LENR, periphs & LL_APB1_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_APB1_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 bus clock.
  * @rmtoll
  * CFGR2         APB1DIS         LL_APB1_DisableBusClock
  */
__STATIC_INLINE void LL_APB1_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_APB1DIS);
}

/**
  * @brief  Disable APB1 peripheral clocks.
  * @rmtoll
  *  APB1LENR         TIM2EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM3EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM4EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM5EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM6EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM7EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         TIM12EN          LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         WWDGEN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         OPAMP1EN         LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         SPI2EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         SPI3EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         USART2EN         LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         USART3EN         LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         UART4EN          LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         UART5EN          LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         I2C1EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         I2C2EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         I3C1EN           LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         CRSEN            LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         USART6EN         LL_APB1_GRP1_DisableClock \n
  *  APB1LENR         UART7EN          LL_APB1_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB1LENR, periphs & LL_APB1_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force APB1 peripherals reset.
  * @rmtoll
  *  APB1LRSTR         TIM2RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM3RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM4RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM5RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM6RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM7RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         TIM12RST          LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         OPAMP1RST         LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         SPI2RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         SPI3RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         USART2RST         LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         USART3RST         LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         UART4RST          LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         UART5RST          LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         I2C1RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         I2C2RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         I3C1RST           LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         CRSRST            LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         USART6RST         LL_APB1_GRP1_ForceReset \n
  *  APB1LRSTR         UART7RST          LL_APB1_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->APB1LRSTR, periphs & LL_APB1_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release APB1 peripherals reset.
  * @rmtoll
  *  APB1LRSTR         TIM2RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM3RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM4RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM5RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM6RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM7RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         TIM12RST          LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         OPAMP1RST         LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         SPI2RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         SPI3RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         USART2RST         LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         USART3RST         LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         UART4RST          LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         UART5RST          LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         I2C1RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         I2C2RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         I3C1RST           LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         CRSRST            LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         USART6RST         LL_APB1_GRP1_ReleaseReset \n
  *  APB1LRSTR         UART7RST          LL_APB1_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->APB1LRSTR, periphs & LL_APB1_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable APB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB1LLPENR         TIM2LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM3LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM4LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM5LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM6LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM7LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         TIM12LPEN          LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         WWDGLPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         OPAMP1LPEN         LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         SPI2LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         SPI3LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         USART2LPEN         LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         USART3LPEN         LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         UART4LPEN          LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         UART5LPEN          LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         I2C1LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         I2C2LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         I3C1LPEN           LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         CRSLPEN            LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         USART6LPEN         LL_APB1_GRP1_EnableClockLowPower \n
  *  APB1LLPENR         UART7LPEN          LL_APB1_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB1LLPENR, periphs & LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB1LLPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  APB1LLPENR         TIM2LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM3LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM4LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM5LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM6LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM7LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         TIM12LPEN          LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         WWDGLPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         OPAMP1LPEN         LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         SPI2LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         SPI3LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         USART2LPEN         LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         USART3LPEN         LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         UART4LPEN          LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         UART5LPEN          LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         I2C1LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         I2C2LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         I3C1LPEN           LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         CRSLPEN            LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         USART6LPEN         LL_APB1_GRP1_IsEnabledClockLowPower \n
  *  APB1LLPENR         UART7LPEN          LL_APB1_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB1LLPENR, periphs & LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB1LLPENR         TIM2LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM3LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM4LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM5LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM6LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM7LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         TIM12LPEN          LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         WWDGLPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         OPAMP1LPEN         LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         SPI2LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         SPI3LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         USART2LPEN         LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         USART3LPEN         LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         UART4LPEN          LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         UART5LPEN          LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         I2C1LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         I2C2LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         I3C1LPEN           LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         CRSLPEN            LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         USART6LPEN         LL_APB1_GRP1_DisableClockLowPower \n
  *  APB1LLPENR         UART7LPEN          LL_APB1_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  * @if TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3    (*)
  * @endif
  * @if TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4    (*)
  * @endif
  * @if TIM5
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  * @if OPAMP1
  *         @arg @ref LL_APB1_GRP1_PERIPH_OPAMP1  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2
  * @if SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  * @if USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3  (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  * @if I2C
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2    (*)
  * @endif
  *         @arg @ref LL_APB1_GRP1_PERIPH_I3C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CRS
  * @if USART6
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART6  (*)
  * @endif
  * @if UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7   (*)
  * @endif
  */
__STATIC_INLINE void LL_APB1_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB1LLPENR, periphs & LL_APB1_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @brief  Enable APB1 peripheral clocks.
  * @rmtoll
  *  APB1HENR         COMP12EN        LL_APB1_GRP2_EnableClock \n
  *  APB1HENR         FDCANEN         LL_APB1_GRP2_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB1HENR, periphs & LL_APB1_GRP2_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB1HENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clock is enabled.
  * @rmtoll
  *  APB1HENR         COMP12EN        LL_APB1_GRP2_IsEnabledClock \n
  *  APB1HENR         FDCANEN         LL_APB1_GRP2_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP2_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB1HENR, periphs & LL_APB1_GRP2_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_APB1_GRP2_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 peripheral clocks.
  * @rmtoll
  *  APB1HENR         COMP12EN        LL_APB1_GRP2_DisableClock \n
  *  APB1HENR         FDCANEN         LL_APB1_GRP2_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB1HENR, periphs & LL_APB1_GRP2_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force APB1 peripherals reset.
  * @rmtoll
  *  APB1HRSTR         COMP12RST        LL_APB1_GRP2_ForceReset \n
  *  APB1HRSTR         FDCANRST         LL_APB1_GRP2_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->APB1HRSTR, periphs & LL_APB1_GRP2_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release APB1 peripherals reset.
  * @rmtoll
  *  APB1HRSTR         COMP12RST        LL_APB1_GRP2_ReleaseReset \n
  *  APB1HRSTR         FDCANRST         LL_APB1_GRP2_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->APB1HRSTR, periphs & LL_APB1_GRP2_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable APB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB1HLPENR         COMP12LPEN        LL_APB1_GRP2_EnableClockLowPower \n
  *  APB1HLPENR         FDCANLPEN         LL_APB1_GRP2_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB1HLPENR, periphs & LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB1HLPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  APB1HLPENR         COMP12LPEN        LL_APB1_GRP2_IsEnabledClockLowPower \n
  *  APB1HLPENR         FDCANLPEN         LL_APB1_GRP2_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP2_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB1HLPENR, periphs & LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB1HLPENR         COMP12LPEN        LL_APB1_GRP2_DisableClockLowPower \n
  *  APB1HLPENR         FDCANLPEN         LL_APB1_GRP2_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP12
  * @if FDCAN
  *         @arg @ref LL_APB1_GRP2_PERIPH_FDCAN (*)
  * @endif
  *
  *        (*) value not defined in all devices.
  */
__STATIC_INLINE void LL_APB1_GRP2_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB1HLPENR, periphs & LL_APB1_GRP2_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_APB1 */

/** @defgroup BUS_LL_EF_APB2 APB2
  * @{
  */
/**
  * @brief  Enable APB2 bus clock.
  * @rmtoll CFGR2         APB2DIS         LL_APB2_EnableBusClock
  */
__STATIC_INLINE void LL_APB2_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_APB2DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB2DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if APB2 bus clock is enabled.
  * @rmtoll CFGR2         APB2DIS         LL_APB2_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB2_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB2DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable APB2 peripheral clocks.
  * @rmtoll
  *  APB2ENR         TIM1EN           LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         SPI1EN           LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         TIM8EN           LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         USART1EN         LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         TIM15EN          LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         TIM16EN          LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         TIM17EN          LL_APB2_GRP1_EnableClock \n
  *  APB2ENR         USBEN            LL_APB2_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1APB (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB2ENR, periphs & LL_APB2_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB2ENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB2 peripheral clock is enabled.
  * @rmtoll
  *  APB2ENR         TIM1EN           LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         SPI1EN           LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         TIM8EN           LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         USART1EN         LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         TIM15EN          LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         TIM16EN          LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         TIM17EN          LL_APB2_GRP1_IsEnabledClock \n
  *  APB2ENR         USBEN            LL_APB2_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1APB (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB2_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB2ENR, periphs & LL_APB2_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_APB2_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB2 bus clock.
  * @rmtoll CFGR2         APB2DIS         LL_APB2_DisableBusClock
  */
__STATIC_INLINE void LL_APB2_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_APB2DIS);
}

/**
  * @brief  Disable APB2 peripheral clocks.
  * @rmtoll
  *  APB2ENR         TIM1EN           LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         SPI1EN           LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         TIM8EN           LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         USART1EN         LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         TIM15EN          LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         TIM16EN          LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         TIM17EN          LL_APB2_GRP1_DisableClock \n
  *  APB2ENR         USBEN            LL_APB2_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1APB (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB2ENR, periphs & LL_APB2_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force APB2 peripherals reset.
  * @rmtoll
  *  APB2RSTR         TIM1RST           LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         SPI1RST           LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         TIM8RST           LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         USART1RST         LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         TIM15RST          LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         TIM16RST          LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         TIM17RST          LL_APB2_GRP1_ForceReset \n
  *  APB2RSTR         USBRST            LL_APB2_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1GLOBAL (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1APP (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->APB2RSTR, periphs & LL_APB2_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release APB2 peripherals reset.
  * @rmtoll
  *  APB2RSTR         TIM1RST           LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         SPI1RST           LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         TIM8RST           LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         USART1RST         LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         TIM15RST          LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         TIM16RST          LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         TIM17RST          LL_APB2_GRP1_ReleaseReset \n
  *  APB2RSTR         USBRST            LL_APB2_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1GLOBAL (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1APP (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->APB2RSTR, periphs & LL_APB2_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable APB2 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB2LPENR         TIM1LPEN           LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         SPI1LPEN           LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         TIM8LPEN           LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         USART1LPEN         LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         TIM15LPEN          LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         TIM16LPEN          LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         TIM17LPEN          LL_APB2_GRP1_EnableClockLowPower \n
  *  APB2LPENR         USBLPEN            LL_APB2_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB2LPENR, periphs & LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB2LPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB2 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  APB2LPENR         TIM1LPEN           LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         SPI1LPEN           LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         TIM8LPEN           LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         USART1LPEN         LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         TIM15LPEN          LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         TIM16LPEN          LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         TIM17LPEN          LL_APB2_GRP1_IsEnabledClockLowPower \n
  *  APB2LPENR         USBLPEN            LL_APB2_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB2_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB2LPENR, periphs & LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB2 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB2LPENR         TIM1LPEN           LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         SPI1LPEN           LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         TIM8LPEN           LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         USART1LPEN         LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         TIM15LPEN          LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         TIM16LPEN          LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         TIM17LPEN          LL_APB2_GRP1_DisableClockLowPower \n
  *  APB2LPENR         USBLPEN            LL_APB2_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  * @if PLAY1
  *         @arg @ref LL_APB2_GRP1_PERIPH_PLAY1 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15
  * @if TIM16
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  * @endif
  * @if TIM17
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  * @endif
  *         @arg @ref LL_APB2_GRP1_PERIPH_USB
  */
__STATIC_INLINE void LL_APB2_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB2LPENR, periphs & LL_APB2_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_APB2 */


/** @defgroup BUS_LL_EF_APB3 APB3
  * @{
  */
/**
  * @brief  Enable APB3 bus clock.
  * @rmtoll CFGR2         APB3DIS         LL_APB3_EnableBusClock
  */
__STATIC_INLINE void LL_APB3_EnableBusClock(void)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_CLEAR_BIT_32(RCC->CFGR2, RCC_CFGR2_APB3DIS);
  tmpreg = STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB3DIS);
  (void)(tmpreg);
}
/**
  * @brief  Check if APB3 bus clock is enabled.
  * @rmtoll
  *  CFGR2         APB3DIS         LL_APB3_IsEnabledBusClock
  * @retval State (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB3_IsEnabledBusClock(void)
{
  return ((STM32_READ_BIT(RCC->CFGR2, RCC_CFGR2_APB3DIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Enable APB3 peripheral clocks.
  * @rmtoll
  *  APB3ENR         SBSEN             LL_APB3_GRP1_EnableClock \n
  *  APB3ENR         LPUART1EN         LL_APB3_GRP1_EnableClock \n
  *  APB3ENR         LPTIM1EN          LL_APB3_GRP1_EnableClock \n
  *  APB3ENR         RTCAPBEN          LL_APB3_GRP1_EnableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  */
__STATIC_INLINE void LL_APB3_GRP1_EnableClock(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB3ENR, periphs & LL_APB3_GRP1_PERIPH_ALL_EN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB3ENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB3 peripheral clock is enabled.
  * @rmtoll
  *  APB3ENR         SBSEN             LL_APB3_GRP1_IsEnabledClock \n
  *  APB3ENR         LPUART1EN         LL_APB3_GRP1_IsEnabledClock \n
  *  APB3ENR         LPTIM1EN          LL_APB3_GRP1_IsEnabledClock \n
  *  APB3ENR         RTCAPBEN          LL_APB3_GRP1_IsEnabledClock
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB3_GRP1_IsEnabledClock(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB3ENR, periphs & LL_APB3_GRP1_PERIPH_ALL_EN_MASK) == \
           (periphs & LL_APB3_GRP1_PERIPH_ALL_EN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB3 bus clock.
  * @rmtoll
  *  CFGR2         APB3DIS         LL_APB3_DisableBusClock
  */
__STATIC_INLINE void LL_APB3_DisableBusClock(void)
{
  STM32_ATOMIC_SET_BIT_32(RCC->CFGR2, RCC_CFGR2_APB3DIS);
}

/**
  * @brief  Disable APB3 peripheral clocks.
  * @rmtoll
  *  APB3ENR         SBSEN             LL_APB3_GRP1_DisableClock  \n
  *  APB3ENR         LPUART1EN         LL_APB3_GRP1_DisableClock  \n
  *  APB3ENR         LPTIM1EN          LL_APB3_GRP1_DisableClock  \n
  *  APB3ENR         RTCAPBEN          LL_APB3_GRP1_DisableClock
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  */
__STATIC_INLINE void LL_APB3_GRP1_DisableClock(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB3ENR, periphs & LL_APB3_GRP1_PERIPH_ALL_EN_MASK);
}

/**
  * @brief  Force APB3 peripherals reset.
  * @rmtoll
  *  APB3RSTR         SBSRST             LL_APB3_GRP1_ForceReset  \n
  *  APB3RSTR         LPUART1RST         LL_APB3_GRP1_ForceReset  \n
  *  APB3RSTR         LPTIM1RST          LL_APB3_GRP1_ForceReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  */
__STATIC_INLINE void LL_APB3_GRP1_ForceReset(uint32_t periphs)
{
  STM32_SET_BIT(RCC->APB3RSTR, periphs & LL_APB3_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Release APB3 peripherals reset.
  * @rmtoll
  *  APB3RSTR         SBSRST             LL_APB3_GRP1_ReleaseReset \n
  *  APB3RSTR         LPUART1RST         LL_APB3_GRP1_ReleaseReset \n
  *  APB3RSTR         LPTIM1RST          LL_APB3_GRP1_ReleaseReset
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  */
__STATIC_INLINE void LL_APB3_GRP1_ReleaseReset(uint32_t periphs)
{
  STM32_CLEAR_BIT(RCC->APB3RSTR, periphs & LL_APB3_GRP1_PERIPH_ALL_RST_MASK);
}

/**
  * @brief  Enable APB3 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB3LPENR         SBSLPEN             LL_APB3_GRP1_EnableClockLowPower \n
  *  APB3LPENR         LPUART1LPEN         LL_APB3_GRP1_EnableClockLowPower \n
  *  APB3LPENR         LPTIM1LPEN          LL_APB3_GRP1_EnableClockLowPower \n
  *  APB3LPENR         RTCAPBLPEN          LL_APB3_GRP1_EnableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  */
__STATIC_INLINE void LL_APB3_GRP1_EnableClockLowPower(uint32_t periphs)
{
  __IO uint32_t tmpreg;
  STM32_ATOMIC_SET_BIT_32(RCC->APB3LPENR, periphs & LL_APB3_GRP1_PERIPH_ALL_LPEN_MASK);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = STM32_READ_BIT(RCC->APB3LPENR, periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB3 peripheral clocks in low-power mode are enabled.
  * @rmtoll
  *  APB3LPENR         SBSLPEN             LL_APB3_GRP1_IsEnabledClockLowPower \n
  *  APB3LPENR         LPUART1LPEN         LL_APB3_GRP1_IsEnabledClockLowPower \n
  *  APB3LPENR         LPTIM1LPEN          LL_APB3_GRP1_IsEnabledClockLowPower \n
  *  APB3LPENR         RTCAPBLPEN          LL_APB3_GRP1_IsEnabledClockLowPower
  * @param  periphs This parameter can be a one of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  *
  * @retval State of periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB3_GRP1_IsEnabledClockLowPower(uint32_t periphs)
{
  return ((STM32_READ_BIT(RCC->APB3LPENR, periphs & LL_APB3_GRP1_PERIPH_ALL_LPEN_MASK) == \
           (periphs & LL_APB3_GRP1_PERIPH_ALL_LPEN_MASK)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB3 peripheral clocks in low-power mode.
  * @rmtoll
  *  APB3LPENR         SBSLPEN             LL_APB3_GRP1_DisableClockLowPower \n
  *  APB3LPENR         LPUART1LPEN         LL_APB3_GRP1_DisableClockLowPower \n
  *  APB3LPENR         LPTIM1LPEN          LL_APB3_GRP1_DisableClockLowPower \n
  *  APB3LPENR         RTCAPBLPEN          LL_APB3_GRP1_DisableClockLowPower
  * @param  periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB3_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB3_GRP1_PERIPH_SBS
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPUART1
  *         @arg @ref LL_APB3_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB3_GRP1_PERIPH_RTCAPB
  */
__STATIC_INLINE void LL_APB3_GRP1_DisableClockLowPower(uint32_t periphs)
{
  STM32_ATOMIC_CLEAR_BIT_32(RCC->APB3LPENR, periphs & LL_APB3_GRP1_PERIPH_ALL_LPEN_MASK);
}

/**
  * @}
  */ /* End of BUS_LL_EF_APB3 */

/**
  * @}
  */ /* End of BUS_LL_Exported_Functions */

/**
  * @}
  */ /* End of BUS_LL */

#endif /* RCC */

/**
  * @}
  */ /* End of STM32C5XX_LL_Driver */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_BUS_H */
