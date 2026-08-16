/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_dma.h
  * @brief   Header file of DMA HAL module.
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
#ifndef STM32C5XX_HAL_DMA_H
#define STM32C5XX_HAL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_dma.h"
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/* Include Q module for linked list management with associated features */
#define USE_HAL_Q_DIRECT_ADDR_MODE (1U) /*!< Use direct addressing mode for DMA queue */
#define USE_HAL_Q_CIRCULAR_LINK    (1U) /*!< Use circular linking for DMA queue       */
#include "stm32c5xx_hal_q.h"
#endif /* USE_HAL_DMA_LINKEDLIST */

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if (defined (LPDMA1) || defined (LPDMA2))

/** @defgroup DMA DMA
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Exported_Constants HAL DMA Constants
  * @{
  */
#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
/** @defgroup DMA_Error_Code Error code definition reflecting asynchronous process errors
  * @{
  */
#define HAL_DMA_ERROR_NONE    (0U)           /*!< DMA channel no error                              */
#define HAL_DMA_ERROR_DTE     (0x01UL << 0U) /*!< DMA channel data transfer error                   */
#define HAL_DMA_ERROR_USE     (0x01UL << 1U) /*!< DMA channel user setting error                    */
#define HAL_DMA_ERROR_TO      (0x01UL << 2U) /*!< DMA channel trigger overrun error                 */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define HAL_DMA_ERROR_ULE     (0x01UL << 3U) /*!< DMA channel fetch node error                      */
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

/** @defgroup DMA_Optional_Interrupt Optional interrupts
  * @{
  */
#define HAL_DMA_OPT_IT_NONE    (0U)                                    /*!< DMA channel optional interrupts Half
                                                                            Transfer(HT) and Trigger Overrun (TO)
                                                                            are disabled */
#define HAL_DMA_OPT_IT_HT      (0x01UL << 9U)                          /*!< DMA channel Half Transfer interrupt
                                                                            enabled   */
#define HAL_DMA_OPT_IT_TO      (0x01UL << 14U)                         /*!< DMA channel Trigger Overrun interrupt
                                                                            enabled */
#define HAL_DMA_OPT_IT_DEFAULT (HAL_DMA_OPT_IT_HT | HAL_DMA_OPT_IT_TO) /*!< DMA channel all optional interrupts
                                                                            i.e Half Transfer(HT) and
                                                                            Trigger Overrun (TO) are enabled */
#define HAL_DMA_OPT_IT_SILENT  ((uint32_t)0xFFFFFFFFU)                 /*!< DMA channel all interrupts are disabled,
                                                                            the DMA transfer will not issue any
                                                                            interrupt */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Exported_Types HAL DMA Types
  * @{
  */

/*! DMA channel instances enumeration definition */
typedef enum
{
  /* LPDMA1 channel instances */
  HAL_LPDMA1_CH0 = LPDMA1_CH0_BASE,  /*!< LPDMA1 Channel 0  */
  HAL_LPDMA1_CH1 = LPDMA1_CH1_BASE,  /*!< LPDMA1 Channel 1  */
  HAL_LPDMA1_CH2 = LPDMA1_CH2_BASE,  /*!< LPDMA1 Channel 2  */
  HAL_LPDMA1_CH3 = LPDMA1_CH3_BASE,  /*!< LPDMA1 Channel 3  */
#if defined(LPDMA1_CH4)
  HAL_LPDMA1_CH4 = LPDMA1_CH4_BASE,  /*!< LPDMA1 Channel 4  */
#endif /* LPDMA1_CH4 */
#if defined(LPDMA1_CH5)
  HAL_LPDMA1_CH5 = LPDMA1_CH5_BASE,  /*!< LPDMA1 Channel 5  */
#endif /* LPDMA1_CH5 */
#if defined(LPDMA1_CH6)
  HAL_LPDMA1_CH6 = LPDMA1_CH6_BASE,  /*!< LPDMA1 Channel 6  */
#endif /* LPDMA1_CH6 */
#if defined(LPDMA1_CH7)
  HAL_LPDMA1_CH7 = LPDMA1_CH7_BASE,  /*!< LPDMA1 Channel 7  */
#endif /* LPDMA1_CH7 */

#if defined(LPDMA2)
  /* LPDMA2 channel instances */
  HAL_LPDMA2_CH0 = LPDMA2_CH0_BASE,  /*!< LPDMA2 Channel 0 */
  HAL_LPDMA2_CH1 = LPDMA2_CH1_BASE,  /*!< LPDMA2 Channel 1 */
  HAL_LPDMA2_CH2 = LPDMA2_CH2_BASE,  /*!< LPDMA2 Channel 2 */
  HAL_LPDMA2_CH3 = LPDMA2_CH3_BASE,  /*!< LPDMA2 Channel 3 */
#if defined(LPDMA2_CH4)
  HAL_LPDMA2_CH4 = LPDMA2_CH4_BASE,  /*!< LPDMA2 Channel 4  */
#endif /* LPDMA2_CH4 */
#if defined(LPDMA2_CH5)
  HAL_LPDMA2_CH5 = LPDMA2_CH5_BASE,  /*!< LPDMA2 Channel 5  */
#endif /* LPDMA2_CH5 */
#if defined(LPDMA2_CH6)
  HAL_LPDMA2_CH6 = LPDMA2_CH6_BASE,  /*!< LPDMA2 Channel 6  */
#endif /* LPDMA2_CH6 */
#if defined(LPDMA2_CH7)
  HAL_LPDMA2_CH7 = LPDMA2_CH7_BASE   /*!< LPDMA2 Channel 7  */
#endif /* LPDMA2_CH7 */
#endif /* LPDMA2 */
} hal_dma_channel_t;

/*! DMA channel state enumeration definition */
typedef enum
{
  HAL_DMA_STATE_RESET   = 0U,        /*!< DMA channel not initialized                    */
  HAL_DMA_STATE_INIT    = 1U << 31U, /*!< DMA channel initialized but not yet configured */
  HAL_DMA_STATE_IDLE    = 1U << 30U, /*!< DMA channel initialized and configured         */
  HAL_DMA_STATE_ACTIVE  = 1U << 29U, /*!< DMA channel transfer is ongoing                */
  HAL_DMA_STATE_SUSPEND = 1U << 28U, /*!< DMA channel transfer suspended                 */
  HAL_DMA_STATE_ABORT   = 1U << 27U  /*!< DMA channel transfer aborted                   */
} hal_dma_state_t;

/*! HAL DMA channel transfer level completion enumeration definition */
typedef enum
{
  HAL_DMA_XFER_FULL_COMPLETE = LL_DMA_FLAG_IDLE,                 /*!< Full channel transfer */
  HAL_DMA_XFER_HALF_COMPLETE = LL_DMA_FLAG_HT | LL_DMA_FLAG_IDLE /*!< Half channel transfer */
} hal_dma_xfer_level_t;

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! DMA xfer mode enumeration definition */
typedef enum
{
  HAL_DMA_XFER_MODE_DIRECT              = 0x00U, /*!< DMA direct mode               */
  HAL_DMA_XFER_MODE_LINKEDLIST_LINEAR   = 0x01U, /*!< DMA linked list linear mode   */
  HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR = 0x02U  /*!< DMA linked list circular mode */
} hal_dma_xfer_mode_t;
#endif /* USE_HAL_DMA_LINKEDLIST */

/*! HAL DMA channel requests enumeration definition */
typedef enum
{
  /* LPDMA1 requests */
  HAL_LPDMA1_REQUEST_ADC1          = LL_LPDMA1_REQUEST_ADC1,         /*!< LPDMA1 HW request is ADC1         */
#if defined(ADC2)
  HAL_LPDMA1_REQUEST_ADC2          = LL_LPDMA1_REQUEST_ADC2,         /*!< LPDMA1 HW request is ADC2         */
#endif /* ADC2 */
  HAL_LPDMA1_REQUEST_TIM6_UPD      = LL_LPDMA1_REQUEST_TIM6_UPD,     /*!< LPDMA1 HW request is TIM6_UPD     */
  HAL_LPDMA1_REQUEST_TIM7_UPD      = LL_LPDMA1_REQUEST_TIM7_UPD,     /*!< LPDMA1 HW request is TIM7_UPD     */
  HAL_LPDMA1_REQUEST_SPI1_RX       = LL_LPDMA1_REQUEST_SPI1_RX,      /*!< LPDMA1 HW request is SPI1_RX      */
  HAL_LPDMA1_REQUEST_SPI1_TX       = LL_LPDMA1_REQUEST_SPI1_TX,      /*!< LPDMA1 HW request is SPI1_TX      */
  HAL_LPDMA1_REQUEST_SPI2_RX       = LL_LPDMA1_REQUEST_SPI2_RX,      /*!< LPDMA1 HW request is SPI2_RX      */
  HAL_LPDMA1_REQUEST_SPI2_TX       = LL_LPDMA1_REQUEST_SPI2_TX,      /*!< LPDMA1 HW request is SPI2_TX      */
#if defined(SPI3)
  HAL_LPDMA1_REQUEST_SPI3_RX       = LL_LPDMA1_REQUEST_SPI3_RX,      /*!< LPDMA1 HW request is SPI3_RX      */
  HAL_LPDMA1_REQUEST_SPI3_TX       = LL_LPDMA1_REQUEST_SPI3_TX,      /*!< LPDMA1 HW request is SPI3_TX      */
#endif /* SPI3 */
  HAL_LPDMA1_REQUEST_I2C1_RX       = LL_LPDMA1_REQUEST_I2C1_RX,      /*!< LPDMA1 HW request is I2C1_RX      */
  HAL_LPDMA1_REQUEST_I2C1_TX       = LL_LPDMA1_REQUEST_I2C1_TX,      /*!< LPDMA1 HW request is I2C1_TX      */
  HAL_LPDMA1_REQUEST_USART1_RX     = LL_LPDMA1_REQUEST_USART1_RX,    /*!< LPDMA1 HW request is USART1_RX    */
  HAL_LPDMA1_REQUEST_USART1_TX     = LL_LPDMA1_REQUEST_USART1_TX,    /*!< LPDMA1 HW request is USART1_TX    */
  HAL_LPDMA1_REQUEST_USART2_RX     = LL_LPDMA1_REQUEST_USART2_RX,    /*!< LPDMA1 HW request is USART2_RX    */
  HAL_LPDMA1_REQUEST_USART2_TX     = LL_LPDMA1_REQUEST_USART2_TX,    /*!< LPDMA1 HW request is USART2_TX    */
#if defined(USART3)
  HAL_LPDMA1_REQUEST_USART3_RX     = LL_LPDMA1_REQUEST_USART3_RX,    /*!< LPDMA1 HW request is USART3_RX    */
  HAL_LPDMA1_REQUEST_USART3_TX     = LL_LPDMA1_REQUEST_USART3_TX,    /*!< LPDMA1 HW request is USART3_TX    */
#endif /* USART3 */
  HAL_LPDMA1_REQUEST_UART4_RX      = LL_LPDMA1_REQUEST_UART4_RX,     /*!< LPDMA1 HW request is UART4_RX     */
  HAL_LPDMA1_REQUEST_UART4_TX      = LL_LPDMA1_REQUEST_UART4_TX,     /*!< LPDMA1 HW request is UART4_TX     */
  HAL_LPDMA1_REQUEST_UART5_RX      = LL_LPDMA1_REQUEST_UART5_RX,     /*!< LPDMA1 HW request is UART5_RX     */
  HAL_LPDMA1_REQUEST_UART5_TX      = LL_LPDMA1_REQUEST_UART5_TX,     /*!< LPDMA1 HW request is UART5_TX     */
  HAL_LPDMA1_REQUEST_LPUART1_RX    = LL_LPDMA1_REQUEST_LPUART1_RX,   /*!< LPDMA1 HW request is LPUART1_RX   */
  HAL_LPDMA1_REQUEST_LPUART1_TX    = LL_LPDMA1_REQUEST_LPUART1_TX,   /*!< LPDMA1 HW request is LPUART1_TX   */
  HAL_LPDMA1_REQUEST_TIM1_CC1      = LL_LPDMA1_REQUEST_TIM1_CC1,     /*!< LPDMA1 HW request is TIM1_CC1     */
  HAL_LPDMA1_REQUEST_TIM1_CC2      = LL_LPDMA1_REQUEST_TIM1_CC2,     /*!< LPDMA1 HW request is TIM1_CC2     */
  HAL_LPDMA1_REQUEST_TIM1_CC3      = LL_LPDMA1_REQUEST_TIM1_CC3,     /*!< LPDMA1 HW request is TIM1_CC3     */
  HAL_LPDMA1_REQUEST_TIM1_CC4      = LL_LPDMA1_REQUEST_TIM1_CC4,     /*!< LPDMA1 HW request is TIM1_CC4     */
  HAL_LPDMA1_REQUEST_TIM1_UPD      = LL_LPDMA1_REQUEST_TIM1_UPD,     /*!< LPDMA1 HW request is TIM1_UPD     */
  HAL_LPDMA1_REQUEST_TIM1_TRGI     = LL_LPDMA1_REQUEST_TIM1_TRGI,    /*!< LPDMA1 HW request is TIM1_TRGI    */
  HAL_LPDMA1_REQUEST_TIM1_COM      = LL_LPDMA1_REQUEST_TIM1_COM,     /*!< LPDMA1 HW request is TIM1_COM     */
  HAL_LPDMA1_REQUEST_TIM2_CC1      = LL_LPDMA1_REQUEST_TIM2_CC1,     /*!< LPDMA1 HW request is TIM2_CC1     */
  HAL_LPDMA1_REQUEST_TIM2_CC2      = LL_LPDMA1_REQUEST_TIM2_CC2,     /*!< LPDMA1 HW request is TIM2_CC2     */
  HAL_LPDMA1_REQUEST_TIM2_CC3      = LL_LPDMA1_REQUEST_TIM2_CC3,     /*!< LPDMA1 HW request is TIM2_CC3     */
  HAL_LPDMA1_REQUEST_TIM2_CC4      = LL_LPDMA1_REQUEST_TIM2_CC4,     /*!< LPDMA1 HW request is TIM2_CC4     */
  HAL_LPDMA1_REQUEST_TIM2_UPD      = LL_LPDMA1_REQUEST_TIM2_UPD,     /*!< LPDMA1 HW request is TIM2_UPD     */
  HAL_LPDMA1_REQUEST_TIM2_TRGI     = LL_LPDMA1_REQUEST_TIM2_TRGI,    /*!< LPDMA1 HW request is TIM2_TRGI    */
#if defined(TIM5)
  HAL_LPDMA1_REQUEST_TIM5_CC1      = LL_LPDMA1_REQUEST_TIM5_CC1,     /*!< LPDMA1 HW request is TIM5_CC1     */
  HAL_LPDMA1_REQUEST_TIM5_CC2      = LL_LPDMA1_REQUEST_TIM5_CC2,     /*!< LPDMA1 HW request is TIM5_CC2     */
  HAL_LPDMA1_REQUEST_TIM5_CC3      = LL_LPDMA1_REQUEST_TIM5_CC3,     /*!< LPDMA1 HW request is TIM5_CC3     */
  HAL_LPDMA1_REQUEST_TIM5_CC4      = LL_LPDMA1_REQUEST_TIM5_CC4,     /*!< LPDMA1 HW request is TIM5_CC4     */
  HAL_LPDMA1_REQUEST_TIM5_UPD      = LL_LPDMA1_REQUEST_TIM5_UPD,     /*!< LPDMA1 HW request is TIM5_UPD     */
  HAL_LPDMA1_REQUEST_TIM5_TRGI     = LL_LPDMA1_REQUEST_TIM5_TRGI,    /*!< LPDMA1 HW request is TIM5_TRGI    */
#endif /* TIM5 */
  HAL_LPDMA1_REQUEST_TIM15_CC1     = LL_LPDMA1_REQUEST_TIM15_CC1,    /*!< LPDMA1 HW request is TIM15_CC1    */
  HAL_LPDMA1_REQUEST_TIM15_CC2     = LL_LPDMA1_REQUEST_TIM15_CC2,    /*!< LPDMA1 HW request is TIM15_CC2    */
  HAL_LPDMA1_REQUEST_TIM15_UPD     = LL_LPDMA1_REQUEST_TIM15_UPD,    /*!< LPDMA1 HW request is TIM15_UPD    */
  HAL_LPDMA1_REQUEST_TIM15_TRGI    = LL_LPDMA1_REQUEST_TIM15_TRGI,   /*!< LPDMA1 HW request is TIM15_TRGI   */
  HAL_LPDMA1_REQUEST_TIM15_COM     = LL_LPDMA1_REQUEST_TIM15_COM,    /*!< LPDMA1 HW request is TIM15_COM    */
#if defined(TIM16)
  HAL_LPDMA1_REQUEST_TIM16_CC1     = LL_LPDMA1_REQUEST_TIM16_CC1,    /*!< LPDMA1 HW request is TIM16_CC1    */
  HAL_LPDMA1_REQUEST_TIM16_UPD     = LL_LPDMA1_REQUEST_TIM16_UPD,    /*!< LPDMA1 HW request is TIM16_UPD    */
#endif /* TIM16 */
#if defined(TIM17)
  HAL_LPDMA1_REQUEST_TIM17_CC1     = LL_LPDMA1_REQUEST_TIM17_CC1,    /*!< LPDMA1 HW request is TIM17_CC1    */
  HAL_LPDMA1_REQUEST_TIM17_UPD     = LL_LPDMA1_REQUEST_TIM17_UPD,    /*!< LPDMA1 HW request is TIM17_UPD    */
#endif /* TIM17 */
#if defined(LPTIM1)
  HAL_LPDMA1_REQUEST_LPTIM1_IC1    = LL_LPDMA1_REQUEST_LPTIM1_IC1,   /*!< LPDMA1 HW request is LPTIM1_IC1   */
  HAL_LPDMA1_REQUEST_LPTIM1_IC2    = LL_LPDMA1_REQUEST_LPTIM1_IC2,   /*!< LPDMA1 HW request is LPTIM1_IC2   */
  HAL_LPDMA1_REQUEST_LPTIM1_UE     = LL_LPDMA1_REQUEST_LPTIM1_UE,    /*!< LPDMA1 HW request is LPTIM1_UE    */
#endif /* LPTIM1 */
  HAL_LPDMA1_REQUEST_CORDIC_RD     = LL_LPDMA1_REQUEST_CORDIC_RD,    /*!< LPDMA1 HW request is CORDIC_RD    */
  HAL_LPDMA1_REQUEST_CORDIC_WR     = LL_LPDMA1_REQUEST_CORDIC_WR,    /*!< LPDMA1 HW request is CORDIC_WR    */
  HAL_LPDMA1_REQUEST_I3C1_RX       = LL_LPDMA1_REQUEST_I3C1_RX,      /*!< LPDMA1 HW request is I3C1_RX      */
  HAL_LPDMA1_REQUEST_I3C1_TX       = LL_LPDMA1_REQUEST_I3C1_TX,      /*!< LPDMA1 HW request is I3C1_TX      */
  HAL_LPDMA1_REQUEST_I3C1_TC       = LL_LPDMA1_REQUEST_I3C1_TC,      /*!< LPDMA1 HW request is I3C1_TC      */
  HAL_LPDMA1_REQUEST_I3C1_RS       = LL_LPDMA1_REQUEST_I3C1_RS,      /*!< LPDMA1 HW request is I3C1_RS      */
  HAL_LPDMA1_REQUEST_AES_OUT       = LL_LPDMA1_REQUEST_AES_OUT,      /*!< LPDMA1 HW request is AES_OUT      */
  HAL_LPDMA1_REQUEST_AES_IN        = LL_LPDMA1_REQUEST_AES_IN,       /*!< LPDMA1 HW request is AES_IN       */
  HAL_LPDMA1_REQUEST_HASH_IN       = LL_LPDMA1_REQUEST_HASH_IN,      /*!< LPDMA1 HW request is HASH_IN      */
#if defined(I2C2)
  HAL_LPDMA1_REQUEST_I2C2_RX       = LL_LPDMA1_REQUEST_I2C2_RX,      /*!< LPDMA1 HW request is I2C2_RX      */
  HAL_LPDMA1_REQUEST_I2C2_TX       = LL_LPDMA1_REQUEST_I2C2_TX,      /*!< LPDMA1 HW request is I2C2_TX      */
#endif /* I2C2 */
  HAL_LPDMA1_REQUEST_TIM8_CC1      = LL_LPDMA1_REQUEST_TIM8_CC1,     /*!< LPDMA1 HW request is TIM8_CC1     */
  HAL_LPDMA1_REQUEST_TIM8_CC2      = LL_LPDMA1_REQUEST_TIM8_CC2,     /*!< LPDMA1 HW request is TIM8_CC2     */
  HAL_LPDMA1_REQUEST_TIM8_CC3      = LL_LPDMA1_REQUEST_TIM8_CC3,     /*!< LPDMA1 HW request is TIM8_CC3     */
  HAL_LPDMA1_REQUEST_TIM8_CC4      = LL_LPDMA1_REQUEST_TIM8_CC4,     /*!< LPDMA1 HW request is TIM8_CC4     */
  HAL_LPDMA1_REQUEST_TIM8_UPD      = LL_LPDMA1_REQUEST_TIM8_UPD,     /*!< LPDMA1 HW request is TIM8_UPD     */
  HAL_LPDMA1_REQUEST_TIM8_TRGI     = LL_LPDMA1_REQUEST_TIM8_TRGI,    /*!< LPDMA1 HW request is TIM8_TRGI    */
  HAL_LPDMA1_REQUEST_TIM8_COM      = LL_LPDMA1_REQUEST_TIM8_COM,     /*!< LPDMA1 HW request is TIM8_COM     */
  HAL_LPDMA1_REQUEST_DAC1_CH1      = LL_LPDMA1_REQUEST_DAC1_CH1,     /*!< LPDMA1 HW request is DAC1_CH1     */
#if defined(DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2UL)
  HAL_LPDMA1_REQUEST_DAC1_CH2      = LL_LPDMA1_REQUEST_DAC1_CH2,     /*!< LPDMA1 HW request is DAC1_CH2     */
#endif /* DAC_NB_OF_CHANNEL == 2UL */
#if defined(USART6)
  HAL_LPDMA1_REQUEST_USART6_RX     = LL_LPDMA1_REQUEST_USART6_RX,    /*!< LPDMA1 HW request is USART6_RX    */
  HAL_LPDMA1_REQUEST_USART6_TX     = LL_LPDMA1_REQUEST_USART6_TX,    /*!< LPDMA1 HW request is USART6_TX    */
#endif /* USART6 */
#if defined(UART7)
  HAL_LPDMA1_REQUEST_UART7_TX      = LL_LPDMA1_REQUEST_UART7_TX,     /*!< LPDMA1 HW request is UART7_TX     */
  HAL_LPDMA1_REQUEST_UART7_RX      = LL_LPDMA1_REQUEST_UART7_RX,     /*!< LPDMA1 HW request is UART7_RX     */
#endif /* UART7 */
#if defined(ADC3)
  HAL_LPDMA1_REQUEST_ADC3          = LL_LPDMA1_REQUEST_ADC3,         /*!< LPDMA1 HW request is ADC3         */
#endif /* ADC3 */
#if defined(TIM3)
  HAL_LPDMA1_REQUEST_TIM3_CC1      = LL_LPDMA1_REQUEST_TIM3_CC1,     /*!< LPDMA1 HW request is TIM3_CC1     */
  HAL_LPDMA1_REQUEST_TIM3_CC2      = LL_LPDMA1_REQUEST_TIM3_CC2,     /*!< LPDMA1 HW request is TIM3_CC2     */
  HAL_LPDMA1_REQUEST_TIM3_CC3      = LL_LPDMA1_REQUEST_TIM3_CC3,     /*!< LPDMA1 HW request is TIM3_CC3     */
  HAL_LPDMA1_REQUEST_TIM3_CC4      = LL_LPDMA1_REQUEST_TIM3_CC4,     /*!< LPDMA1 HW request is TIM3_CC4     */
  HAL_LPDMA1_REQUEST_TIM3_UPD      = LL_LPDMA1_REQUEST_TIM3_UPD,     /*!< LPDMA1 HW request is TIM3_UPD     */
  HAL_LPDMA1_REQUEST_TIM3_TRGI     = LL_LPDMA1_REQUEST_TIM3_TRGI,    /*!< LPDMA1 HW request is TIM3_TRGI    */
#endif /* TIM3 */
#if defined(TIM4)
  HAL_LPDMA1_REQUEST_TIM4_CC1      = LL_LPDMA1_REQUEST_TIM4_CC1,     /*!< LPDMA1 HW request is TIM4_CC1     */
  HAL_LPDMA1_REQUEST_TIM4_CC2      = LL_LPDMA1_REQUEST_TIM4_CC2,     /*!< LPDMA1 HW request is TIM4_CC2     */
  HAL_LPDMA1_REQUEST_TIM4_CC3      = LL_LPDMA1_REQUEST_TIM4_CC3,     /*!< LPDMA1 HW request is TIM4_CC3     */
  HAL_LPDMA1_REQUEST_TIM4_CC4      = LL_LPDMA1_REQUEST_TIM4_CC4,     /*!< LPDMA1 HW request is TIM4_CC4     */
  HAL_LPDMA1_REQUEST_TIM4_UPD      = LL_LPDMA1_REQUEST_TIM4_UPD,     /*!< LPDMA1 HW request is TIM4_UPD     */
  HAL_LPDMA1_REQUEST_TIM4_TRGI     = LL_LPDMA1_REQUEST_TIM4_TRGI,    /*!< LPDMA1 HW request is TIM4_TRGI    */
#endif /* TIM4 */
#if defined(SAES)
  HAL_LPDMA1_REQUEST_SAES_OUT      = LL_LPDMA1_REQUEST_SAES_OUT,     /*!< LPDMA1 HW request is SAES_OUT     */
  HAL_LPDMA1_REQUEST_SAES_IN       = LL_LPDMA1_REQUEST_SAES_IN,      /*!< LPDMA1 HW request is SAES_IN      */
#endif /* SAES */
#if defined(XSPI1)
  HAL_LPDMA1_REQUEST_XSPI1         = LL_LPDMA1_REQUEST_XSPI1,        /*!< LPDMA1 HW request is XSPI1        */
#endif /* XSPI1 */

#if defined(LPDMA2)
  /* LPDMA2 requests */
  HAL_LPDMA2_REQUEST_ADC1          = LL_LPDMA2_REQUEST_ADC1,         /*!< LPDMA2 HW request is ADC1         */
#if defined(ADC2)
  HAL_LPDMA2_REQUEST_ADC2          = LL_LPDMA2_REQUEST_ADC2,         /*!< LPDMA2 HW request is ADC2         */
#endif /* ADC2 */
  HAL_LPDMA2_REQUEST_TIM6_UPD      = LL_LPDMA2_REQUEST_TIM6_UPD,     /*!< LPDMA2 HW request is TIM6_UPD     */
  HAL_LPDMA2_REQUEST_TIM7_UPD      = LL_LPDMA2_REQUEST_TIM7_UPD,     /*!< LPDMA2 HW request is TIM7_UPD     */
  HAL_LPDMA2_REQUEST_SPI1_RX       = LL_LPDMA2_REQUEST_SPI1_RX,      /*!< LPDMA2 HW request is SPI1_RX      */
  HAL_LPDMA2_REQUEST_SPI1_TX       = LL_LPDMA2_REQUEST_SPI1_TX,      /*!< LPDMA2 HW request is SPI1_TX      */
  HAL_LPDMA2_REQUEST_SPI2_RX       = LL_LPDMA2_REQUEST_SPI2_RX,      /*!< LPDMA2 HW request is SPI2_RX      */
  HAL_LPDMA2_REQUEST_SPI2_TX       = LL_LPDMA2_REQUEST_SPI2_TX,      /*!< LPDMA2 HW request is SPI2_TX      */
#if defined(SPI3)
  HAL_LPDMA2_REQUEST_SPI3_RX       = LL_LPDMA2_REQUEST_SPI3_RX,      /*!< LPDMA2 HW request is SPI3_RX      */
  HAL_LPDMA2_REQUEST_SPI3_TX       = LL_LPDMA2_REQUEST_SPI3_TX,      /*!< LPDMA2 HW request is SPI3_TX      */
#endif /* SPI3 */
  HAL_LPDMA2_REQUEST_I2C1_RX       = LL_LPDMA2_REQUEST_I2C1_RX,      /*!< LPDMA2 HW request is I2C1_RX      */
  HAL_LPDMA2_REQUEST_I2C1_TX       = LL_LPDMA2_REQUEST_I2C1_TX,      /*!< LPDMA2 HW request is I2C1_TX      */
  HAL_LPDMA2_REQUEST_USART1_RX     = LL_LPDMA2_REQUEST_USART1_RX,    /*!< LPDMA2 HW request is USART1_RX    */
  HAL_LPDMA2_REQUEST_USART1_TX     = LL_LPDMA2_REQUEST_USART1_TX,    /*!< LPDMA2 HW request is USART1_TX    */
  HAL_LPDMA2_REQUEST_USART2_RX     = LL_LPDMA2_REQUEST_USART2_RX,    /*!< LPDMA2 HW request is USART2_RX    */
  HAL_LPDMA2_REQUEST_USART2_TX     = LL_LPDMA2_REQUEST_USART2_TX,    /*!< LPDMA2 HW request is USART2_TX    */
#if defined(USART3)
  HAL_LPDMA2_REQUEST_USART3_RX     = LL_LPDMA2_REQUEST_USART3_RX,    /*!< LPDMA2 HW request is USART3_RX    */
  HAL_LPDMA2_REQUEST_USART3_TX     = LL_LPDMA2_REQUEST_USART3_TX,    /*!< LPDMA2 HW request is USART3_TX    */
#endif /* USART3 */
  HAL_LPDMA2_REQUEST_UART4_RX      = LL_LPDMA2_REQUEST_UART4_RX,     /*!< LPDMA2 HW request is UART4_RX     */
  HAL_LPDMA2_REQUEST_UART4_TX      = LL_LPDMA2_REQUEST_UART4_TX,     /*!< LPDMA2 HW request is UART4_TX     */
  HAL_LPDMA2_REQUEST_UART5_RX      = LL_LPDMA2_REQUEST_UART5_RX,     /*!< LPDMA2 HW request is UART5_RX     */
  HAL_LPDMA2_REQUEST_UART5_TX      = LL_LPDMA2_REQUEST_UART5_TX,     /*!< LPDMA2 HW request is UART5_TX     */
  HAL_LPDMA2_REQUEST_LPUART1_RX    = LL_LPDMA2_REQUEST_LPUART1_RX,   /*!< LPDMA2 HW request is LPUART1_RX   */
  HAL_LPDMA2_REQUEST_LPUART1_TX    = LL_LPDMA2_REQUEST_LPUART1_TX,   /*!< LPDMA2 HW request is LPUART1_TX   */
  HAL_LPDMA2_REQUEST_TIM1_CC1      = LL_LPDMA2_REQUEST_TIM1_CC1,     /*!< LPDMA2 HW request is TIM1_CC1     */
  HAL_LPDMA2_REQUEST_TIM1_CC2      = LL_LPDMA2_REQUEST_TIM1_CC2,     /*!< LPDMA2 HW request is TIM1_CC2     */
  HAL_LPDMA2_REQUEST_TIM1_CC3      = LL_LPDMA2_REQUEST_TIM1_CC3,     /*!< LPDMA2 HW request is TIM1_CC3     */
  HAL_LPDMA2_REQUEST_TIM1_CC4      = LL_LPDMA2_REQUEST_TIM1_CC4,     /*!< LPDMA2 HW request is TIM1_CC4     */
  HAL_LPDMA2_REQUEST_TIM1_UPD      = LL_LPDMA2_REQUEST_TIM1_UPD,     /*!< LPDMA2 HW request is TIM1_UPD     */
  HAL_LPDMA2_REQUEST_TIM1_TRGI     = LL_LPDMA2_REQUEST_TIM1_TRGI,    /*!< LPDMA2 HW request is TIM1_TRGI    */
  HAL_LPDMA2_REQUEST_TIM1_COM      = LL_LPDMA2_REQUEST_TIM1_COM,     /*!< LPDMA2 HW request is TIM1_COM     */
  HAL_LPDMA2_REQUEST_TIM2_CC1      = LL_LPDMA2_REQUEST_TIM2_CC1,     /*!< LPDMA2 HW request is TIM2_CC1     */
  HAL_LPDMA2_REQUEST_TIM2_CC2      = LL_LPDMA2_REQUEST_TIM2_CC2,     /*!< LPDMA2 HW request is TIM2_CC2     */
  HAL_LPDMA2_REQUEST_TIM2_CC3      = LL_LPDMA2_REQUEST_TIM2_CC3,     /*!< LPDMA2 HW request is TIM2_CC3     */
  HAL_LPDMA2_REQUEST_TIM2_CC4      = LL_LPDMA2_REQUEST_TIM2_CC4,     /*!< LPDMA2 HW request is TIM2_CC4     */
  HAL_LPDMA2_REQUEST_TIM2_UPD      = LL_LPDMA2_REQUEST_TIM2_UPD,     /*!< LPDMA2 HW request is TIM2_UPD     */
  HAL_LPDMA2_REQUEST_TIM2_TRGI     = LL_LPDMA2_REQUEST_TIM2_TRGI,    /*!< LPDMA2 HW request is TIM2_TRGI    */
#if defined(TIM5)
  HAL_LPDMA2_REQUEST_TIM5_CC1      = LL_LPDMA2_REQUEST_TIM5_CC1,     /*!< LPDMA2 HW request is TIM5_CC1     */
  HAL_LPDMA2_REQUEST_TIM5_CC2      = LL_LPDMA2_REQUEST_TIM5_CC2,     /*!< LPDMA2 HW request is TIM5_CC2     */
  HAL_LPDMA2_REQUEST_TIM5_CC3      = LL_LPDMA2_REQUEST_TIM5_CC3,     /*!< LPDMA2 HW request is TIM5_CC3     */
  HAL_LPDMA2_REQUEST_TIM5_CC4      = LL_LPDMA2_REQUEST_TIM5_CC4,     /*!< LPDMA2 HW request is TIM5_CC4     */
  HAL_LPDMA2_REQUEST_TIM5_UPD      = LL_LPDMA2_REQUEST_TIM5_UPD,     /*!< LPDMA2 HW request is TIM5_UPD     */
  HAL_LPDMA2_REQUEST_TIM5_TRGI     = LL_LPDMA2_REQUEST_TIM5_TRGI,    /*!< LPDMA2 HW request is TIM5_TRGI    */
#endif /* TIM5 */
  HAL_LPDMA2_REQUEST_TIM15_CC1     = LL_LPDMA2_REQUEST_TIM15_CC1,    /*!< LPDMA2 HW request is TIM15_CC1    */
  HAL_LPDMA2_REQUEST_TIM15_CC2     = LL_LPDMA2_REQUEST_TIM15_CC2,    /*!< LPDMA2 HW request is TIM15_CC2    */
  HAL_LPDMA2_REQUEST_TIM15_UPD     = LL_LPDMA2_REQUEST_TIM15_UPD,    /*!< LPDMA2 HW request is TIM15_UPD    */
  HAL_LPDMA2_REQUEST_TIM15_TRGI    = LL_LPDMA2_REQUEST_TIM15_TRGI,   /*!< LPDMA2 HW request is TIM15_TRGI   */
  HAL_LPDMA2_REQUEST_TIM15_COM     = LL_LPDMA2_REQUEST_TIM15_COM,    /*!< LPDMA2 HW request is TIM15_COM    */
#if defined(TIM16)
  HAL_LPDMA2_REQUEST_TIM16_CC1     = LL_LPDMA2_REQUEST_TIM16_CC1,    /*!< LPDMA2 HW request is TIM16_CC1    */
  HAL_LPDMA2_REQUEST_TIM16_UPD     = LL_LPDMA2_REQUEST_TIM16_UPD,    /*!< LPDMA2 HW request is TIM16_UPD    */
#endif /* TIM16 */
#if defined(TIM17)
  HAL_LPDMA2_REQUEST_TIM17_CC1     = LL_LPDMA2_REQUEST_TIM17_CC1,    /*!< LPDMA2 HW request is TIM17_CC1    */
  HAL_LPDMA2_REQUEST_TIM17_UPD     = LL_LPDMA2_REQUEST_TIM17_UPD,    /*!< LPDMA2 HW request is TIM17_UPD    */
#endif /* TIM17 */
#if defined(LPTIM1)
  HAL_LPDMA2_REQUEST_LPTIM1_IC1    = LL_LPDMA2_REQUEST_LPTIM1_IC1,   /*!< LPDMA2 HW request is LPTIM1_IC1   */
  HAL_LPDMA2_REQUEST_LPTIM1_IC2    = LL_LPDMA2_REQUEST_LPTIM1_IC2,   /*!< LPDMA2 HW request is LPTIM1_IC2   */
  HAL_LPDMA2_REQUEST_LPTIM1_UE     = LL_LPDMA2_REQUEST_LPTIM1_UE,    /*!< LPDMA2 HW request is LPTIM1_UE    */
#endif /* LPTIM1 */
  HAL_LPDMA2_REQUEST_CORDIC_RD     = LL_LPDMA2_REQUEST_CORDIC_RD,    /*!< LPDMA2 HW request is CORDIC_RD    */
  HAL_LPDMA2_REQUEST_CORDIC_WR     = LL_LPDMA2_REQUEST_CORDIC_WR,    /*!< LPDMA2 HW request is CORDIC_WR    */
  HAL_LPDMA2_REQUEST_I3C1_RX       = LL_LPDMA2_REQUEST_I3C1_RX,      /*!< LPDMA2 HW request is I3C1_RX      */
  HAL_LPDMA2_REQUEST_I3C1_TX       = LL_LPDMA2_REQUEST_I3C1_TX,      /*!< LPDMA2 HW request is I3C1_TX      */
  HAL_LPDMA2_REQUEST_I3C1_TC       = LL_LPDMA2_REQUEST_I3C1_TC,      /*!< LPDMA2 HW request is I3C1_TC      */
  HAL_LPDMA2_REQUEST_I3C1_RS       = LL_LPDMA2_REQUEST_I3C1_RS,      /*!< LPDMA2 HW request is I3C1_RS      */
  HAL_LPDMA2_REQUEST_AES_OUT       = LL_LPDMA2_REQUEST_AES_OUT,      /*!< LPDMA2 HW request is AES_OUT      */
  HAL_LPDMA2_REQUEST_AES_IN        = LL_LPDMA2_REQUEST_AES_IN,       /*!< LPDMA2 HW request is AES_IN       */
  HAL_LPDMA2_REQUEST_HASH_IN       = LL_LPDMA2_REQUEST_HASH_IN,      /*!< LPDMA2 HW request is HASH_IN      */
#if defined(I2C2)
  HAL_LPDMA2_REQUEST_I2C2_RX       = LL_LPDMA2_REQUEST_I2C2_RX,      /*!< LPDMA2 HW request is I2C2_RX      */
  HAL_LPDMA2_REQUEST_I2C2_TX       = LL_LPDMA2_REQUEST_I2C2_TX,      /*!< LPDMA2 HW request is I2C2_TX      */
#endif /* I2C2 */
  HAL_LPDMA2_REQUEST_TIM8_CC1      = LL_LPDMA2_REQUEST_TIM8_CC1,     /*!< LPDMA2 HW request is TIM8_CC1     */
  HAL_LPDMA2_REQUEST_TIM8_CC2      = LL_LPDMA2_REQUEST_TIM8_CC2,     /*!< LPDMA2 HW request is TIM8_CC2     */
  HAL_LPDMA2_REQUEST_TIM8_CC3      = LL_LPDMA2_REQUEST_TIM8_CC3,     /*!< LPDMA2 HW request is TIM8_CC3     */
  HAL_LPDMA2_REQUEST_TIM8_CC4      = LL_LPDMA2_REQUEST_TIM8_CC4,     /*!< LPDMA2 HW request is TIM8_CC4     */
  HAL_LPDMA2_REQUEST_TIM8_UPD      = LL_LPDMA2_REQUEST_TIM8_UPD,     /*!< LPDMA2 HW request is TIM8_UPD     */
  HAL_LPDMA2_REQUEST_TIM8_TRGI     = LL_LPDMA2_REQUEST_TIM8_TRGI,    /*!< LPDMA2 HW request is TIM8_TRGI    */
  HAL_LPDMA2_REQUEST_TIM8_COM      = LL_LPDMA2_REQUEST_TIM8_COM,     /*!< LPDMA2 HW request is TIM8_COM     */
  HAL_LPDMA2_REQUEST_DAC1_CH1      = LL_LPDMA2_REQUEST_DAC1_CH1,     /*!< LPDMA2 HW request is DAC1_CH1     */
#if defined(DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2UL)
  HAL_LPDMA2_REQUEST_DAC1_CH2      = LL_LPDMA2_REQUEST_DAC1_CH2,     /*!< LPDMA2 HW request is DAC1_CH2     */
#endif /* DAC_NB_OF_CHANNEL == 2UL */
#if defined(USART6)
  HAL_LPDMA2_REQUEST_USART6_RX     = LL_LPDMA2_REQUEST_USART6_RX,    /*!< LPDMA2 HW request is USART6_RX    */
  HAL_LPDMA2_REQUEST_USART6_TX     = LL_LPDMA2_REQUEST_USART6_TX,    /*!< LPDMA2 HW request is USART6_TX    */
#endif /* USART6 */
#if defined(UART7)
  HAL_LPDMA2_REQUEST_UART7_TX      = LL_LPDMA2_REQUEST_UART7_TX,     /*!< LPDMA2 HW request is UART7_TX     */
  HAL_LPDMA2_REQUEST_UART7_RX      = LL_LPDMA2_REQUEST_UART7_RX,     /*!< LPDMA2 HW request is UART7_RX     */
#endif /* UART7 */
#if defined(ADC3)
  HAL_LPDMA2_REQUEST_ADC3          = LL_LPDMA2_REQUEST_ADC3,         /*!< LPDMA2 HW request is ADC3         */
#endif /* ADC3 */
#if defined(TIM3)
  HAL_LPDMA2_REQUEST_TIM3_CC1      = LL_LPDMA2_REQUEST_TIM3_CC1,     /*!< LPDMA2 HW request is TIM3_CC1     */
  HAL_LPDMA2_REQUEST_TIM3_CC2      = LL_LPDMA2_REQUEST_TIM3_CC2,     /*!< LPDMA2 HW request is TIM3_CC2     */
  HAL_LPDMA2_REQUEST_TIM3_CC3      = LL_LPDMA2_REQUEST_TIM3_CC3,     /*!< LPDMA2 HW request is TIM3_CC3     */
  HAL_LPDMA2_REQUEST_TIM3_CC4      = LL_LPDMA2_REQUEST_TIM3_CC4,     /*!< LPDMA2 HW request is TIM3_CC4     */
  HAL_LPDMA2_REQUEST_TIM3_UPD      = LL_LPDMA2_REQUEST_TIM3_UPD,     /*!< LPDMA2 HW request is TIM3_UPD     */
  HAL_LPDMA2_REQUEST_TIM3_TRGI     = LL_LPDMA2_REQUEST_TIM3_TRGI,    /*!< LPDMA2 HW request is TIM3_TRGI    */
#endif /* TIM3 */
#if defined(TIM4)
  HAL_LPDMA2_REQUEST_TIM4_CC1      = LL_LPDMA2_REQUEST_TIM4_CC1,     /*!< LPDMA2 HW request is TIM4_CC1     */
  HAL_LPDMA2_REQUEST_TIM4_CC2      = LL_LPDMA2_REQUEST_TIM4_CC2,     /*!< LPDMA2 HW request is TIM4_CC2     */
  HAL_LPDMA2_REQUEST_TIM4_CC3      = LL_LPDMA2_REQUEST_TIM4_CC3,     /*!< LPDMA2 HW request is TIM4_CC3     */
  HAL_LPDMA2_REQUEST_TIM4_CC4      = LL_LPDMA2_REQUEST_TIM4_CC4,     /*!< LPDMA2 HW request is TIM4_CC4     */
  HAL_LPDMA2_REQUEST_TIM4_UPD      = LL_LPDMA2_REQUEST_TIM4_UPD,     /*!< LPDMA2 HW request is TIM4_UPD     */
  HAL_LPDMA2_REQUEST_TIM4_TRGI     = LL_LPDMA2_REQUEST_TIM4_TRGI,    /*!< LPDMA2 HW request is TIM4_TRGI    */
#endif /* TIM4 */
#if defined(SAES)
  HAL_LPDMA2_REQUEST_SAES_OUT      = LL_LPDMA2_REQUEST_SAES_OUT,     /*!< LPDMA2 HW request is SAES_OUT     */
  HAL_LPDMA2_REQUEST_SAES_IN       = LL_LPDMA2_REQUEST_SAES_IN,      /*!< LPDMA2 HW request is SAES_IN      */
#endif /* SAES */
#if defined(XSPI1)
  HAL_LPDMA2_REQUEST_XSPI1         = LL_LPDMA2_REQUEST_XSPI1,        /*!< LPDMA2 HW request is XSPI1        */
#endif /* XSPI1 */
#endif /* LPDMA2 */

  /* Software request */
  HAL_DMA_REQUEST_SW               = DMA_CTR2_SWREQ                  /*!< DMA SW request                    */
} hal_dma_request_source_t;

/*! DMA channel mode enumeration definition */
typedef enum
{
  HAL_DMA_HARDWARE_REQUEST_BURST = LL_DMA_HARDWARE_REQUEST_BURST, /*!< DMA burst request transfer */
  HAL_DMA_HARDWARE_REQUEST_BLOCK = LL_DMA_HARDWARE_REQUEST_BLOCK  /*!< DMA block request transfer */
} hal_dma_hardware_request_mode_t;

/*! DMA channel direction enumeration definition */
typedef enum
{
  HAL_DMA_DIRECTION_MEMORY_TO_MEMORY = LL_DMA_DIRECTION_MEMORY_TO_MEMORY, /*!< Memory to memory direction     */
  HAL_DMA_DIRECTION_PERIPH_TO_MEMORY = LL_DMA_DIRECTION_PERIPH_TO_MEMORY, /*!< Peripheral to memory direction */
  HAL_DMA_DIRECTION_MEMORY_TO_PERIPH = LL_DMA_DIRECTION_MEMORY_TO_PERIPH  /*!< Memory to peripheral direction */
} hal_dma_direction_t;

/*! DMA channel source increment enumeration definition */
typedef enum
{
  HAL_DMA_SRC_ADDR_FIXED       = LL_DMA_SRC_ADDR_FIXED,       /*!< Source fixed single / burst       */
  HAL_DMA_SRC_ADDR_INCREMENTED = LL_DMA_SRC_ADDR_INCREMENTED  /*!< Source incremented single / burst */
} hal_dma_src_addr_increment_t;

/*! DMA channel destination increment enumeration definition */
typedef enum
{
  HAL_DMA_DEST_ADDR_FIXED       = LL_DMA_DEST_ADDR_FIXED,       /*!< Destination fixed single / burst       */
  HAL_DMA_DEST_ADDR_INCREMENTED = LL_DMA_DEST_ADDR_INCREMENTED  /*!< Destination incremented single / burst */
} hal_dma_dest_addr_increment_t;

/*! DMA channel source data width enumeration definition */
typedef enum
{
  HAL_DMA_SRC_DATA_WIDTH_BYTE       = LL_DMA_SRC_DATA_WIDTH_BYTE,      /*!< Source data width: byte        */
  HAL_DMA_SRC_DATA_WIDTH_HALFWORD   = LL_DMA_SRC_DATA_WIDTH_HALFWORD,  /*!< Source data width: halfword    */
  HAL_DMA_SRC_DATA_WIDTH_WORD       = LL_DMA_SRC_DATA_WIDTH_WORD,      /*!< Source data width: word        */
} hal_dma_src_data_width_t;

/*! DMA channel destination data width enumeration definition */
typedef enum
{
  HAL_DMA_DEST_DATA_WIDTH_BYTE       = LL_DMA_DEST_DATA_WIDTH_BYTE,      /*!< Destination data width: byte        */
  HAL_DMA_DEST_DATA_WIDTH_HALFWORD   = LL_DMA_DEST_DATA_WIDTH_HALFWORD,  /*!< Destination data width: halfword    */
  HAL_DMA_DEST_DATA_WIDTH_WORD       = LL_DMA_DEST_DATA_WIDTH_WORD,      /*!< Destination data width: word        */
} hal_dma_dest_data_width_t;

/*! DMA channel priority enumeration definition */
typedef enum
{
  HAL_DMA_PRIORITY_LOW_WEIGHT_LOW  = LL_DMA_PRIORITY_LOW_WEIGHT_LOW,  /*!< Priority level: low priority, low weight   */
  HAL_DMA_PRIORITY_LOW_WEIGHT_MID  = LL_DMA_PRIORITY_LOW_WEIGHT_MID,  /*!< Priority level: low priority, mid weight   */
  HAL_DMA_PRIORITY_LOW_WEIGHT_HIGH = LL_DMA_PRIORITY_LOW_WEIGHT_HIGH, /*!< Priority level: low priority, high weight  */
  HAL_DMA_PRIORITY_HIGH            = LL_DMA_PRIORITY_HIGH             /*!< Priority level: high priority              */
} hal_dma_priority_t;

/*! DMA channel transfer configuration structure definition */
typedef struct
{
  hal_dma_request_source_t      request;         /*!< DMA channel transfer request           */
  hal_dma_direction_t           direction;       /*!< DMA channel transfer direction         */
  hal_dma_src_addr_increment_t  src_inc;         /*!< DMA channel source increment mode      */
  hal_dma_dest_addr_increment_t dest_inc;        /*!< DMA channel destination increment mode */
  hal_dma_src_data_width_t      src_data_width;  /*!< DMA channel source data width          */
  hal_dma_dest_data_width_t     dest_data_width; /*!< DMA channel destination data width     */
  hal_dma_priority_t            priority;        /*!< DMA channel priority level             */
} hal_dma_direct_xfer_config_t;

/*! DMA channel flow control mode enumeration definition */
typedef enum
{
  HAL_DMA_FLOW_CONTROL_DMA    = LL_DMA_FLOW_CONTROL_DMA,    /*!< DMA request DMA channel flow control */
  HAL_DMA_FLOW_CONTROL_PERIPH = LL_DMA_FLOW_CONTROL_PERIPH  /*!< DMA request peripheral flow control  */
} hal_dma_flow_control_mode_t;

/*! HAL DMA channel trigger hardware signal enumeration definition */
typedef enum
{
  /* LPDMA1 triggers */
  HAL_LPDMA1_TRIGGER_EXTI0         = LL_LPDMA1_TRIGGER_EXTI0,          /*!< LPDMA1 HW Trigger is EXTI0         */
  HAL_LPDMA1_TRIGGER_EXTI1         = LL_LPDMA1_TRIGGER_EXTI1,          /*!< LPDMA1 HW Trigger is EXTI1         */
  HAL_LPDMA1_TRIGGER_EXTI2         = LL_LPDMA1_TRIGGER_EXTI2,          /*!< LPDMA1 HW Trigger is EXTI2         */
  HAL_LPDMA1_TRIGGER_EXTI3         = LL_LPDMA1_TRIGGER_EXTI3,          /*!< LPDMA1 HW Trigger is EXTI3         */
  HAL_LPDMA1_TRIGGER_EXTI4         = LL_LPDMA1_TRIGGER_EXTI4,          /*!< LPDMA1 HW Trigger is EXTI4         */
  HAL_LPDMA1_TRIGGER_EXTI5         = LL_LPDMA1_TRIGGER_EXTI5,          /*!< LPDMA1 HW Trigger is EXTI5         */
  HAL_LPDMA1_TRIGGER_EXTI6         = LL_LPDMA1_TRIGGER_EXTI6,          /*!< LPDMA1 HW Trigger is EXTI6         */
  HAL_LPDMA1_TRIGGER_EXTI7         = LL_LPDMA1_TRIGGER_EXTI7,          /*!< LPDMA1 HW Trigger is EXTI7         */
  HAL_LPDMA1_TRIGGER_TAMP_TRG1     = LL_LPDMA1_TRIGGER_TAMP_TRG1,      /*!< LPDMA1 HW Trigger is TAMP_TRG1     */
  HAL_LPDMA1_TRIGGER_TAMP_TRG2     = LL_LPDMA1_TRIGGER_TAMP_TRG2,      /*!< LPDMA1 HW Trigger is TAMP_TRG2     */
  HAL_LPDMA1_TRIGGER_TAMP_TRG3     = LL_LPDMA1_TRIGGER_TAMP_TRG3,      /*!< LPDMA1 HW Trigger is TAMP_TRG3     */
#if defined(LPTIM1)
  HAL_LPDMA1_TRIGGER_LPTIM1_CH1    = LL_LPDMA1_TRIGGER_LPTIM1_CH1,     /*!< LPDMA1 HW Trigger is LPTIM1_CH1    */
  HAL_LPDMA1_TRIGGER_LPTIM1_CH2    = LL_LPDMA1_TRIGGER_LPTIM1_CH2,     /*!< LPDMA1 HW Trigger is LPTIM1_CH2    */
#endif /* LPTIM1 */
  HAL_LPDMA1_TRIGGER_RTC_ALRA_TRG  = LL_LPDMA1_TRIGGER_RTC_ALRA_TRG,   /*!< LPDMA1 HW Trigger is RTC_ALRA_TRG  */
  HAL_LPDMA1_TRIGGER_RTC_ALRB_TRG  = LL_LPDMA1_TRIGGER_RTC_ALRB_TRG,   /*!< LPDMA1 HW Trigger is RTC_ALRB_TRG  */
  HAL_LPDMA1_TRIGGER_RTC_WUT_TRG   = LL_LPDMA1_TRIGGER_RTC_WUT_TRG,    /*!< LPDMA1 HW Trigger is RTC_WUT_TRG   */
  HAL_LPDMA1_TRIGGER_TIM2_TRGO     = LL_LPDMA1_TRIGGER_TIM2_TRGO,      /*!< LPDMA1 HW Trigger is TIM2_TRGO     */
  HAL_LPDMA1_TRIGGER_TIM15_TRGO    = LL_LPDMA1_TRIGGER_TIM15_TRGO,     /*!< LPDMA1 HW Trigger is TIM15_TRGO    */
  HAL_LPDMA1_TRIGGER_COMP1_OUT     = LL_LPDMA1_TRIGGER_COMP1_OUT,      /*!< LPDMA1 HW Trigger is COMP1_OUT     */
  HAL_LPDMA1_TRIGGER_EVENTOUT      = LL_LPDMA1_TRIGGER_EVENTOUT,       /*!< LPDMA1 HW Trigger is EVENTOUT      */
  HAL_LPDMA1_TRIGGER_LPDMA1_CH0_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH0_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH0_TC */
  HAL_LPDMA1_TRIGGER_LPDMA1_CH1_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH1_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH1_TC */
  HAL_LPDMA1_TRIGGER_LPDMA1_CH2_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH2_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH2_TC */
  HAL_LPDMA1_TRIGGER_LPDMA1_CH3_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH3_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH3_TC */
#if defined (LPDMA1_CH4)
  HAL_LPDMA1_TRIGGER_LPDMA1_CH4_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH4_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH4_TC */
#endif /* LPDMA1_CH4 */
#if defined (LPDMA1_CH5)
  HAL_LPDMA1_TRIGGER_LPDMA1_CH5_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH5_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH5_TC */
#endif /* LPDMA1_CH5 */
#if defined (LPDMA1_CH6)
  HAL_LPDMA1_TRIGGER_LPDMA1_CH6_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH6_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH6_TC */
#endif /* LPDMA1_CH6 */
#if defined (LPDMA1_CH7)
  HAL_LPDMA1_TRIGGER_LPDMA1_CH7_TC = LL_LPDMA1_TRIGGER_LPDMA1_CH7_TC,  /*!< LPDMA1 HW Trigger is LPDMA1_CH7_TC */
#endif /* LPDMA1_CH7 */
#if defined(LPDMA2)
  HAL_LPDMA1_TRIGGER_LPDMA2_CH0_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH0_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH0_TC */
  HAL_LPDMA1_TRIGGER_LPDMA2_CH1_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH1_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH1_TC */
  HAL_LPDMA1_TRIGGER_LPDMA2_CH2_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH2_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH2_TC */
  HAL_LPDMA1_TRIGGER_LPDMA2_CH3_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH3_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH3_TC */
#if defined (LPDMA2_CH4)
  HAL_LPDMA1_TRIGGER_LPDMA2_CH4_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH4_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH4_TC */
#endif /* LPDMA2_CH4 */
#if defined (LPDMA2_CH5)
  HAL_LPDMA1_TRIGGER_LPDMA2_CH5_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH5_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH5_TC */
#endif /* LPDMA2_CH5 */
#if defined (LPDMA2_CH6)
  HAL_LPDMA1_TRIGGER_LPDMA2_CH6_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH6_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH6_TC */
#endif /* LPDMA2_CH6 */
#if defined (LPDMA2_CH7)
  HAL_LPDMA1_TRIGGER_LPDMA2_CH7_TC = LL_LPDMA1_TRIGGER_LPDMA2_CH7_TC,  /*!< LPDMA1 HW Trigger is LPDMA2_CH7_TC */
#endif /* LPDMA2_CH7 */
#endif /* LPDMA2 */
#if defined(COMP2)
  HAL_LPDMA1_TRIGGER_COMP2_OUT     = LL_LPDMA1_TRIGGER_COMP2_OUT,      /*!< LPDMA1 HW Trigger is COMP2_OUT     */
#endif /* COMP2 */

#if defined(LPDMA2)
  /* LPDMA2 triggers */
  HAL_LPDMA2_TRIGGER_EXTI0         = LL_LPDMA2_TRIGGER_EXTI0,          /*!< LPDMA2 HW Trigger is EXTI0         */
  HAL_LPDMA2_TRIGGER_EXTI1         = LL_LPDMA2_TRIGGER_EXTI1,          /*!< LPDMA2 HW Trigger is EXTI1         */
  HAL_LPDMA2_TRIGGER_EXTI2         = LL_LPDMA2_TRIGGER_EXTI2,          /*!< LPDMA2 HW Trigger is EXTI2         */
  HAL_LPDMA2_TRIGGER_EXTI3         = LL_LPDMA2_TRIGGER_EXTI3,          /*!< LPDMA2 HW Trigger is EXTI3         */
  HAL_LPDMA2_TRIGGER_EXTI4         = LL_LPDMA2_TRIGGER_EXTI4,          /*!< LPDMA2 HW Trigger is EXTI4         */
  HAL_LPDMA2_TRIGGER_EXTI5         = LL_LPDMA2_TRIGGER_EXTI5,          /*!< LPDMA2 HW Trigger is EXTI5         */
  HAL_LPDMA2_TRIGGER_EXTI6         = LL_LPDMA2_TRIGGER_EXTI6,          /*!< LPDMA2 HW Trigger is EXTI6         */
  HAL_LPDMA2_TRIGGER_EXTI7         = LL_LPDMA2_TRIGGER_EXTI7,          /*!< LPDMA2 HW Trigger is EXTI7         */
  HAL_LPDMA2_TRIGGER_TAMP_TRG1     = LL_LPDMA2_TRIGGER_TAMP_TRG1,      /*!< LPDMA2 HW Trigger is TAMP_TRG1     */
  HAL_LPDMA2_TRIGGER_TAMP_TRG2     = LL_LPDMA2_TRIGGER_TAMP_TRG2,      /*!< LPDMA2 HW Trigger is TAMP_TRG2     */
  HAL_LPDMA2_TRIGGER_TAMP_TRG3     = LL_LPDMA2_TRIGGER_TAMP_TRG3,      /*!< LPDMA2 HW Trigger is TAMP_TRG3     */
#if defined(LPTIM1)
  HAL_LPDMA2_TRIGGER_LPTIM1_CH1    = LL_LPDMA2_TRIGGER_LPTIM1_CH1,     /*!< LPDMA2 HW Trigger is LPTIM1_CH1    */
  HAL_LPDMA2_TRIGGER_LPTIM1_CH2    = LL_LPDMA2_TRIGGER_LPTIM1_CH2,     /*!< LPDMA2 HW Trigger is LPTIM1_CH2    */
#endif /* LPTIM1 */
  HAL_LPDMA2_TRIGGER_RTC_ALRA_TRG  = LL_LPDMA2_TRIGGER_RTC_ALRA_TRG,   /*!< LPDMA2 HW Trigger is RTC_ALRA_TRG  */
  HAL_LPDMA2_TRIGGER_RTC_ALRB_TRG  = LL_LPDMA2_TRIGGER_RTC_ALRB_TRG,   /*!< LPDMA2 HW Trigger is RTC_ALRB_TRG  */
  HAL_LPDMA2_TRIGGER_RTC_WUT_TRG   = LL_LPDMA2_TRIGGER_RTC_WUT_TRG,    /*!< LPDMA2 HW Trigger is RTC_WUT_TRG   */
  HAL_LPDMA2_TRIGGER_TIM2_TRGO     = LL_LPDMA2_TRIGGER_TIM2_TRGO,      /*!< LPDMA2 HW Trigger is TIM2_TRGO     */
  HAL_LPDMA2_TRIGGER_TIM15_TRGO    = LL_LPDMA2_TRIGGER_TIM15_TRGO,     /*!< LPDMA2 HW Trigger is TIM15_TRGO    */
  HAL_LPDMA2_TRIGGER_COMP1_OUT     = LL_LPDMA2_TRIGGER_COMP1_OUT,      /*!< LPDMA2 HW Trigger is COMP1_OUT     */
  HAL_LPDMA2_TRIGGER_EVENTOUT      = LL_LPDMA2_TRIGGER_EVENTOUT,       /*!< LPDMA2 HW Trigger is EVENTOUT      */
  HAL_LPDMA2_TRIGGER_LPDMA1_CH0_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH0_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH0_TC */
  HAL_LPDMA2_TRIGGER_LPDMA1_CH1_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH1_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH1_TC */
  HAL_LPDMA2_TRIGGER_LPDMA1_CH2_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH2_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH2_TC */
  HAL_LPDMA2_TRIGGER_LPDMA1_CH3_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH3_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH3_TC */
#if defined (LPDMA1_CH4)
  HAL_LPDMA2_TRIGGER_LPDMA1_CH4_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH4_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH4_TC */
#endif /* LPDMA1_CH4 */
#if defined (LPDMA1_CH5)
  HAL_LPDMA2_TRIGGER_LPDMA1_CH5_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH5_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH5_TC */
#endif /* LPDMA1_CH5 */
#if defined (LPDMA1_CH6)
  HAL_LPDMA2_TRIGGER_LPDMA1_CH6_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH6_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH6_TC */
#endif /* LPDMA1_CH6 */
#if defined (LPDMA1_CH7)
  HAL_LPDMA2_TRIGGER_LPDMA1_CH7_TC = LL_LPDMA2_TRIGGER_LPDMA1_CH7_TC,  /*!< LPDMA2 HW Trigger is LPDMA1_CH7_TC */
#endif /* LPDMA1_CH7 */
  HAL_LPDMA2_TRIGGER_LPDMA2_CH0_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH0_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH0_TC */
  HAL_LPDMA2_TRIGGER_LPDMA2_CH1_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH1_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH1_TC */
  HAL_LPDMA2_TRIGGER_LPDMA2_CH2_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH2_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH2_TC */
  HAL_LPDMA2_TRIGGER_LPDMA2_CH3_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH3_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH3_TC */
#if defined (LPDMA2_CH4)
  HAL_LPDMA2_TRIGGER_LPDMA2_CH4_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH4_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH4_TC */
#endif /* LPDMA2_CH4 */
#if defined (LPDMA2_CH5)
  HAL_LPDMA2_TRIGGER_LPDMA2_CH5_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH5_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH5_TC */
#endif /* LPDMA2_CH5 */
#if defined (LPDMA2_CH6)
  HAL_LPDMA2_TRIGGER_LPDMA2_CH6_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH6_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH6_TC */
#endif /* LPDMA2_CH6 */
#if defined (LPDMA2_CH7)
  HAL_LPDMA2_TRIGGER_LPDMA2_CH7_TC = LL_LPDMA2_TRIGGER_LPDMA2_CH7_TC,  /*!< LPDMA2 HW Trigger is LPDMA2_CH7_TC */
#endif /* LPDMA2_CH7 */
#if defined(COMP2)
  HAL_LPDMA2_TRIGGER_COMP2_OUT     = LL_LPDMA2_TRIGGER_COMP2_OUT,      /*!< LPDMA2 HW Trigger is COMP2_OUT     */
#endif /* COMP2 */
#endif /* LPDMA2 */
} hal_dma_trigger_source_t;

/*! DMA channel trigger polarity enumeration definition */
typedef enum
{
  HAL_DMA_TRIGGER_POLARITY_MASKED  = LL_DMA_TRIGGER_POLARITY_MASKED,  /*!< No trigger of the selected DMA request.
                                                                           Masked trigger event                      */
  HAL_DMA_TRIGGER_POLARITY_RISING  = LL_DMA_TRIGGER_POLARITY_RISING,  /*!< Trigger of the selected DMA request on the
                                                                           rising edge of the selected trigger event
                                                                           input                                     */
  HAL_DMA_TRIGGER_POLARITY_FALLING = LL_DMA_TRIGGER_POLARITY_FALLING  /*!< Trigger of the selected DMA request on the
                                                                           falling edge of the selected trigger event
                                                                           input                                     */
} hal_dma_trigger_polarity_t;

/*! DMA channel trigger mode enumeration definition */
typedef enum
{
  HAL_DMA_TRIGGER_BLOCK_TRANSFER          = LL_DMA_TRIGGER_BLOCK_TRANSFER,         /*!< A block transfer is conditioned
                                                                                        by (at least) one hit trigger */
  HAL_DMA_TRIGGER_NODE_TRANSFER           = LL_DMA_TRIGGER_NODE_TRANSFER,    /*!< An LLI link transfer is conditioned by
                                                                                   (at least) one hit trigger         */
  HAL_DMA_TRIGGER_SINGLE_BURST_TRANSFER   = LL_DMA_TRIGGER_SINGLE_BURST_TRANSFER  /*!< A single/burst transfer is
                                                                                       conditioned by (at least) one hit
                                                                                       trigger                        */
} hal_dma_trigger_mode_t;

/*! DMA channel trigger configuration structure definition */
typedef struct
{
  hal_dma_trigger_source_t   source;   /*!< DMA channel trigger event source selection */
  hal_dma_trigger_polarity_t polarity; /*!< DMA channel trigger event polarity         */
  hal_dma_trigger_mode_t     mode;     /*!< DMA channel trigger mode                   */
} hal_dma_trigger_config_t;

/*! DMA channel destination data truncation and padding enumeration definition */
typedef enum
{
  HAL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO  = LL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO,  /*!< Destination data left
                                                                                         truncation with zero padding */
  HAL_DMA_DEST_DATA_TRUNC_RIGHT_PADD_SIGN = LL_DMA_DEST_DATA_TRUNC_RIGHT_PADD_SIGN  /*!< Destination data right
                                                                                         truncation with sign padding */
} hal_dma_dest_data_trunc_padd_t;

/*! DMA channel data handling configuration structure definition */
typedef struct
{
  hal_dma_dest_data_trunc_padd_t   trunc_padd;             /*!< DMA channel data truncation or padding mode     */
} hal_dma_data_handling_config_t;

/*! Privileged access level attribute */
typedef enum
{
  HAL_DMA_NPRIV = LL_DMA_ATTR_NPRIV, /*!< DMA non-privileged attribute */
  HAL_DMA_PRIV  = LL_DMA_ATTR_PRIV   /*!< DMA privileged attribute     */
} hal_dma_priv_attr_t;

/*! Attributes lock status */
typedef enum
{
  HAL_DMA_UNLOCKED = 0U, /*!< DMA unlocked attribute */
  HAL_DMA_LOCKED   = 1U    /*!< DMA locked attribute   */
} hal_dma_attr_lock_status_t;

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! DMA channel half transfer and transfer complete event generation enumeration definition */
typedef enum
{
  HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK          = LL_DMA_LINKEDLIST_XFER_EVENT_BLOCK,   /*!< The TC event is generated at
                                                                                        the end of each block and the HT
                                                                                        event is generated at the half
                                                                                        of each block                 */
  HAL_DMA_LINKEDLIST_XFER_EVENT_NODE           = LL_DMA_LINKEDLIST_XFER_EVENT_NODE,/*!< The TC event is generated at the
                                                                                        end of each linked list item and
                                                                                        the HT event is generated at the
                                                                                        half of each linked list item */
  HAL_DMA_LINKEDLIST_XFER_EVENT_Q              = LL_DMA_LINKEDLIST_XFER_EVENT_Q    /*!< The TC event is generated at the
                                                                                        end of the last linked list item
                                                                                        and the HT event is generated at
                                                                                        the half of the last linked list
                                                                                        item                          */
} hal_dma_linkedlist_xfer_event_mode_t;

/*! DMA channel linked list configuration structure definition */
typedef struct
{
  hal_dma_priority_t                   priority;   /*!< DMA channel priority level             */
  hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode; /*!< DMA channel transfer event mode        */
} hal_dma_linkedlist_xfer_config_t;

/*! DMA channel linked list execution enumeration definition */
typedef enum
{
  HAL_DMA_LINKEDLIST_EXECUTION_Q    = LL_DMA_LINKEDLIST_EXECUTION_Q,   /*!< Channel executed for the full linked list */
  HAL_DMA_LINKEDLIST_EXECUTION_NODE = LL_DMA_LINKEDLIST_EXECUTION_NODE /*!< Channel executed once for the current LLI */
} hal_dma_linkedlist_execution_mode_t;

/*! DMA channel node configuration enumeration definition */
typedef struct
{
  hal_dma_direct_xfer_config_t         xfer;                    /*!< DMA Channel direct transfer configuration    */
  hal_dma_hardware_request_mode_t      hw_request_mode;         /*!< DMA channel hardware request mode            */
  hal_dma_flow_control_mode_t          flow_ctrl_mode;          /*!< DMA channel flow control mode                */
  hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode;         /*!< DMA Channel transfer event mode              */
  hal_dma_trigger_config_t             trigger;                 /*!< DMA Channel trigger configuration            */
  hal_dma_data_handling_config_t       data_handling;           /*!< DMA Channel data handling configuration      */
  uint32_t                             src_addr;                /*!< DMA Channel source address                   */
  uint32_t                             dest_addr;               /*!< DMA Channel destination address              */
  uint32_t                             size_byte;               /*!< DMA Channel size in byte                     */
} hal_dma_node_config_t;

/*! DMA channel node type enumeration definition */
typedef enum
{
  HAL_DMA_NODE_LINEAR_ADDRESSING = 0x05U, /*!< Linear addressing DMA node */
} hal_dma_node_type_t;

/*! DMA linked list node structure definition */
typedef struct
{
  uint32_t regs[LL_DMA_NODE_REGISTER_NUM]; /*!< Specifies the physical DMA channel node registers description */
  uint32_t info;                           /*!< Specified the physical DMA channel node information           */
} hal_dma_node_t;
#endif /* USE_HAL_DMA_LINKEDLIST */

typedef struct hal_dma_handle_s hal_dma_handle_t; /*!< HAL DMA channel handle structure type */

/*! DMA channel process callback type definition */
typedef void (*hal_dma_cb_t)(hal_dma_handle_t *hdma);

/*! HAL DMA channel handle Structure definition */
struct hal_dma_handle_s
{
  hal_dma_channel_t                instance;          /*!< DMA channel instance                        */
  void                             *p_parent;         /*!< DMA channel parent                          */
  volatile hal_dma_state_t         global_state;      /*!< DMA channel transfer state                  */
#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  volatile uint32_t                last_error_codes; /*!< DMA channel transfer error codes             */
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  volatile hal_dma_xfer_mode_t     xfer_mode;         /*!< DMA channel transfer mode                   */
  hal_dma_node_t                   *p_head_node;      /*!< DMA channel q                               */
#endif /* USE_HAL_DMA_LINKEDLIST */
  hal_dma_cb_t p_xfer_halfcplt_cb;                    /*!< DMA channel half transfer complete callback */
  hal_dma_cb_t p_xfer_cplt_cb;                        /*!< DMA channel transfer complete callback      */
  hal_dma_cb_t p_xfer_abort_cb;                       /*!< DMA channel transfer Abort callback         */
  hal_dma_cb_t p_xfer_suspend_cb;                     /*!< DMA channel transfer Suspend callback       */
  hal_dma_cb_t p_xfer_error_cb;                       /*!< DMA channel transfer error callback         */
#if defined(USE_HAL_DMA_USER_DATA) && (USE_HAL_DMA_USER_DATA == 1)
  const void                        *p_user_data;     /*!< DMA channel user data                       */
#endif /* USE_HAL_DMA_USER_DATA */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Exported_Functions HAL DMA Functions
  * @{
  */

/** @defgroup DMA_Exported_Functions_Group1 Initialization and De-initialization functions
  * @{
  */
/* Initialization and Deinitialization APIs */
hal_status_t HAL_DMA_Init(hal_dma_handle_t *hdma, hal_dma_channel_t instance);
void HAL_DMA_DeInit(hal_dma_handle_t *hdma);
hal_dma_channel_t HAL_DMA_GetInstance(const hal_dma_handle_t *hdma);
DMA_Channel_TypeDef *HAL_DMA_GetLLInstance(const hal_dma_handle_t *hdma);
/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group2 Configuration functions
  * @{
  */
/* Direct xfer config APIs */
hal_status_t HAL_DMA_SetConfigDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config);
void HAL_DMA_GetConfigDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config);

/* Hardware request mode config APIs */
hal_status_t HAL_DMA_SetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma,
                                                            hal_dma_hardware_request_mode_t hw_request_mode);
hal_status_t HAL_DMA_ResetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma);
hal_dma_hardware_request_mode_t HAL_DMA_GetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma);

/* Transfer peripheral flow control mode config APIs */
hal_status_t HAL_DMA_SetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma,
                                                        hal_dma_flow_control_mode_t flow_control_mode);
hal_status_t HAL_DMA_ResetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma);
hal_dma_flow_control_mode_t HAL_DMA_GetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma);

/* Trigger config APIs */
hal_status_t HAL_DMA_SetConfigDirectXferTrigger(hal_dma_handle_t *hdma, const hal_dma_trigger_config_t *p_config);
hal_status_t HAL_DMA_ResetConfigDirectXferTrigger(hal_dma_handle_t *hdma);
void HAL_DMA_GetConfigDirectXferTrigger(hal_dma_handle_t *hdma, hal_dma_trigger_config_t *p_config);

/* Data handling config APIs */
hal_status_t HAL_DMA_SetConfigDirectXferDataHandling(hal_dma_handle_t *hdma,
                                                     const hal_dma_data_handling_config_t *p_config);
hal_status_t HAL_DMA_ResetConfigDirectXferDataHandling(hal_dma_handle_t *hdma);
void HAL_DMA_GetConfigDirectXferDataHandling(hal_dma_handle_t *hdma, hal_dma_data_handling_config_t *p_config);

/* Channel privilege attribute config APIs */
hal_status_t HAL_DMA_SetPrivAttr(hal_dma_channel_t instance, hal_dma_priv_attr_t priv_attr);
hal_dma_priv_attr_t HAL_DMA_GetPrivAttr(hal_dma_channel_t instance);

/* Channel lock attribute config APIs */

hal_status_t HAL_DMA_LockAttr(hal_dma_channel_t instance);

hal_dma_attr_lock_status_t HAL_DMA_IsLockedAttr(hal_dma_channel_t instance);

/* Direct periph xfer config APIs */
hal_status_t HAL_DMA_SetConfigPeriphDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config);
void HAL_DMA_GetConfigPeriphDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/* Linked list xfer config APIs */
hal_status_t HAL_DMA_SetConfigLinkedListXfer(hal_dma_handle_t *hdma, const hal_dma_linkedlist_xfer_config_t *p_config);
void HAL_DMA_GetConfigLinkedListXfer(hal_dma_handle_t *hdma, hal_dma_linkedlist_xfer_config_t *p_config);

/* Linked list xfer event mode config APIs */
hal_status_t HAL_DMA_SetLinkedListXferEventMode(hal_dma_handle_t *hdma,
                                                hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode);
hal_status_t HAL_DMA_ResetLinkedListXferEventMode(hal_dma_handle_t *hdma);
hal_dma_linkedlist_xfer_event_mode_t HAL_DMA_GetLinkedListXferEventMode(hal_dma_handle_t *hdma);


/* Linked list xfer channel priority APIs */
hal_status_t HAL_DMA_SetLinkedListXferPriority(hal_dma_handle_t *hdma, hal_dma_priority_t priority);
hal_status_t HAL_DMA_ResetLinkedListXferPriority(hal_dma_handle_t *hdma);
hal_dma_priority_t HAL_DMA_GetLinkedListXferPriority(hal_dma_handle_t *hdma);

/* Linked list xfer execution mode APIs */
hal_status_t HAL_DMA_SetLinkedListXferExecutionMode(hal_dma_handle_t *hdma,
                                                    hal_dma_linkedlist_execution_mode_t exec_mode);
hal_status_t HAL_DMA_ResetLinkedListXferExecutionMode(hal_dma_handle_t *hdma);
hal_dma_linkedlist_execution_mode_t HAL_DMA_GetLinkedListXferExecutionMode(hal_dma_handle_t *hdma);

/* Linked list circular periph xfer config APIs */
hal_status_t HAL_DMA_SetConfigPeriphLinkedListCircularXfer(hal_dma_handle_t *hdma, hal_dma_node_t *p_node,
                                                           const hal_dma_direct_xfer_config_t *p_node_config);
void HAL_DMA_GetConfigPeriphLinkedListCircularXfer(hal_dma_handle_t *hdma, hal_dma_node_t *p_node,
                                                   hal_dma_direct_xfer_config_t *p_node_config);
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/** @defgroup DMA_Exported_Functions_Group3 Linked list node management functions
  * @{
  */
/* Node building APIs */
hal_status_t HAL_DMA_FillNodeConfig(hal_dma_node_t *p_node, const hal_dma_node_config_t *p_conf,
                                    hal_dma_node_type_t node_type);
void HAL_DMA_GetNodeConfig(const hal_dma_node_t *p_node, hal_dma_node_config_t *p_conf,
                           hal_dma_node_type_t *p_node_type);

hal_status_t HAL_DMA_FillNodeDirectXfer(hal_dma_node_t *p_node, const hal_dma_direct_xfer_config_t *p_config,
                                        hal_dma_node_type_t node_type);
void HAL_DMA_GetNodeDirectXfer(const hal_dma_node_t *p_node, hal_dma_direct_xfer_config_t *p_config,
                               hal_dma_node_type_t *p_node_type);

hal_status_t HAL_DMA_FillNodeHardwareRequestMode(hal_dma_node_t *p_node,
                                                 hal_dma_hardware_request_mode_t hw_request_mode);
hal_dma_hardware_request_mode_t HAL_DMA_GetNodeHardwareRequestMode(const hal_dma_node_t *p_node);

hal_status_t HAL_DMA_FillNodeFlowControlMode(hal_dma_node_t *p_node, hal_dma_flow_control_mode_t flow_control_mode);
hal_dma_flow_control_mode_t HAL_DMA_GetNodeFlowControlMode(const hal_dma_node_t *p_node);

hal_status_t HAL_DMA_FillNodeXferEventMode(hal_dma_node_t *p_node,
                                           hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode);
hal_dma_linkedlist_xfer_event_mode_t HAL_DMA_GetNodeXferEventMode(const hal_dma_node_t *p_node);

hal_status_t HAL_DMA_FillNodeTrigger(hal_dma_node_t *p_node, const hal_dma_trigger_config_t *p_config);
void HAL_DMA_GetNodeTrigger(const hal_dma_node_t *p_node, hal_dma_trigger_config_t *p_config);

hal_status_t HAL_DMA_FillNodeDataHandling(hal_dma_node_t *p_node, const hal_dma_data_handling_config_t *p_config);
void HAL_DMA_GetNodeDataHandling(const hal_dma_node_t *p_node, hal_dma_data_handling_config_t *p_config);

hal_status_t HAL_DMA_FillNodeData(hal_dma_node_t *p_node, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte);
void HAL_DMA_GetNodeData(const hal_dma_node_t *p_node, uint32_t *p_src_addr, uint32_t *p_dest_addr,
                         uint32_t *p_size_byte);

/* Conversion queue APIs */
hal_status_t HAL_DMA_ConvertQNodesToDynamic(hal_q_t *p_q);
hal_status_t HAL_DMA_ConvertQNodesToStatic(hal_q_t *p_q);
/**
  * @}
  */
#endif /* USE_HAL_DMA_LINKEDLIST */

/** @defgroup DMA_Exported_Functions_Group4 Process management functions
  * @{
  */
/* Start APIs */
hal_status_t HAL_DMA_StartDirectXfer(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte);
hal_status_t HAL_DMA_StartDirectXfer_IT(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                        uint32_t size_byte);
hal_status_t HAL_DMA_StartDirectXfer_IT_Opt(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                            uint32_t size_byte, uint32_t interrupts);
hal_status_t HAL_DMA_StartPeriphXfer_IT_Opt(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                            uint32_t size_byte, uint32_t interrupts);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
hal_status_t HAL_DMA_StartLinkedListXfer(hal_dma_handle_t *hdma, const hal_q_t *p_q);
hal_status_t HAL_DMA_StartLinkedListXfer_IT(hal_dma_handle_t *hdma, const hal_q_t *p_q);
hal_status_t HAL_DMA_StartLinkedListXfer_IT_Opt(hal_dma_handle_t *hdma, const hal_q_t *p_q, uint32_t interrupts);
#endif /* USE_HAL_DMA_LINKEDLIST */

/* Abort APIs */
hal_status_t HAL_DMA_Abort(hal_dma_handle_t *hdma);
hal_status_t HAL_DMA_Abort_IT(hal_dma_handle_t *hdma);

/* Suspend and resume APIs */
hal_status_t HAL_DMA_Suspend(hal_dma_handle_t *hdma);
hal_status_t HAL_DMA_Suspend_IT(hal_dma_handle_t *hdma);
hal_status_t HAL_DMA_Resume(hal_dma_handle_t *hdma);

/* Polling for transfer API */
hal_status_t HAL_DMA_PollForXfer(hal_dma_handle_t *hdma, hal_dma_xfer_level_t xfer_level, uint32_t timeout_ms);

/* IRQHandler API */
void HAL_DMA_IRQHandler(hal_dma_handle_t *hdma);
/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group5 Callbacks functions
  * @{
  */
/* Register callback APIs */
hal_status_t HAL_DMA_RegisterXferHalfCpltCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback);
hal_status_t HAL_DMA_RegisterXferCpltCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback);
hal_status_t HAL_DMA_RegisterXferAbortCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback);
hal_status_t HAL_DMA_RegisterXferSuspendCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback);
hal_status_t HAL_DMA_RegisterXferErrorCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback);

/* Callbacks APIs */
void HAL_DMA_XferHalfCpltCallback(hal_dma_handle_t *hdma);
void HAL_DMA_XferCpltCallback(hal_dma_handle_t *hdma);
void HAL_DMA_XferAbortCallback(hal_dma_handle_t *hdma);
void HAL_DMA_XferSuspendCallback(hal_dma_handle_t *hdma);
void HAL_DMA_XferErrorCallback(hal_dma_handle_t *hdma);

#if defined(USE_HAL_DMA_USER_DATA) && (USE_HAL_DMA_USER_DATA == 1)
void HAL_DMA_SetUserData(hal_dma_handle_t *hdma, const void *p_user_data);
const void *HAL_DMA_GetUserData(const hal_dma_handle_t *hdma);
#endif /* USE_HAL_DMA_USER_DATA */
/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group6 Status functions
  * @{
  */
/* Get transfer data APIs */
uint32_t HAL_DMA_GetDirectXferRemainingDataByte(const hal_dma_handle_t *hdma);

/* State APIs */
hal_dma_state_t HAL_DMA_GetState(const hal_dma_handle_t *hdma);

#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
uint32_t HAL_DMA_GetLastErrorCodes(const hal_dma_handle_t *hdma);
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */
/**
  * @}
  */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Private_Functions DMA Private Functions
  * @{
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)

/**
  * @brief Get Node information for a DMA channel linear addressing.
  * @param p_next_offset_addr Pointer to next node address offset
  * @param p_addressing_mode  Pointer to node addressing mode
  */
__STATIC_INLINE void HAL_DMA_LinearAddr_GetNodeInfo(uint32_t *p_next_offset_addr,
                                                    hal_q_addressing_mode_t *p_addressing_mode)
{
  *p_next_offset_addr = (uint32_t)LL_DMA_NODE_LINEAR_ADDRESSING_OFFSET;
  *p_addressing_mode  = HAL_Q_ADDRESSING_DIRECT;
}

/**
  * @brief Set DMA node address.
  * @param head_node_addr   Head node address
  * @param prev_node_addr   Previous node address
  * @param next_node_addr   Next node address
  * @param node_addr_offset Node address offset
  */
__STATIC_INLINE void HAL_DMA_SetNodeAddress(uint32_t head_node_addr, uint32_t prev_node_addr, uint32_t next_node_addr,
                                            uint32_t node_addr_offset)
{
  STM32_UNUSED(head_node_addr);

  STM32_MODIFY_REG((*(uint32_t *)(prev_node_addr + node_addr_offset)), (LL_DMA_UPDATE_ALL | DMA_CLLR_LA),
                   (next_node_addr & DMA_CLLR_LA) | LL_DMA_UPDATE_ALL);
}

/**
  * @brief  Get DMA node address.
  * @param  head_node_addr    Head node address
  * @param  current_node_addr Current node address
  * @param  node_addr_offset  Node address offset
  * @retval uint32_t          DMA node address
  */
__STATIC_INLINE uint32_t HAL_DMA_GetNodeAddress(uint32_t head_node_addr, uint32_t current_node_addr,
                                                uint32_t node_addr_offset)
{
  STM32_UNUSED(head_node_addr);

  return (((uint32_t)current_node_addr & DMA_CLBAR_LBA) \
          + (*(uint32_t *)(((uint32_t)current_node_addr) + node_addr_offset) & DMA_CLLR_LA));
}
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */

/* Exported variables ------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Exported_Variables HAL DMA Variables
  * @{
  */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)

/*!  HAL DMA linear addressing operation descriptor variables definition */
static const hal_q_desc_ops_t HAL_DMA_LinearAddressing_DescOps =
{
  HAL_DMA_LinearAddr_GetNodeInfo,
  HAL_DMA_SetNodeAddress,
  HAL_DMA_GetNodeAddress
};
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */

/**
  * @}
  */

#endif /* (defined (LPDMA1) || defined (LPDMA2)) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_DMA_H */
