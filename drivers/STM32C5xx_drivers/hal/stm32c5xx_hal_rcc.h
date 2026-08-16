/**
  ******************************************************************************
  * @file    stm32c5xx_hal_rcc.h
  * @brief   Header file of RCC HAL  module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_HAL_RCC_H
#define STM32C5XX_HAL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_rcc.h"
#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_pwr.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (RCC)

/** @addtogroup RCC
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_Exported_Constants HAL RCC Constants
  * @{
  */

/** @defgroup RCC_Reset_Flag Reset Flag
  * @{
  */
#define HAL_RCC_RESET_FLAG_PIN   RCC_RSR_PINRSTF    /*!< PIN reset flag */
#define HAL_RCC_RESET_FLAG_PWR   RCC_RSR_BORRSTF    /*!< BOR or POR/PDR reset flag */
#define HAL_RCC_RESET_FLAG_SW    RCC_RSR_SFTRSTF    /*!< Software Reset flag */
#define HAL_RCC_RESET_FLAG_IWDG  RCC_RSR_IWDGRSTF   /*!< Independent Watchdog reset flag */
#define HAL_RCC_RESET_FLAG_WWDG  RCC_RSR_WWDGRSTF   /*!< Window watchdog reset flag */
#define HAL_RCC_RESET_FLAG_LPWR  RCC_RSR_LPWRRSTF   /*!< Low power reset flag */
#define HAL_RCC_RESET_FLAG_ALL   (HAL_RCC_RESET_FLAG_PIN | HAL_RCC_RESET_FLAG_PWR |  \
                                  HAL_RCC_RESET_FLAG_SW | HAL_RCC_RESET_FLAG_IWDG | \
                                  HAL_RCC_RESET_FLAG_WWDG | HAL_RCC_RESET_FLAG_LPWR) /*!< All RCC reset flags */
/**
  * @}
  */
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RCC_Exported_Types HAL RCC Types
  * @{
  */

/**
  * @brief Peripheral clock activation status.
  */
typedef enum
{
  HAL_RCC_OSC_DISABLED = 0U,                     /*!< Oscillator is disabled */
  HAL_RCC_OSC_ENABLED  = ! HAL_RCC_OSC_DISABLED, /*!< Oscillator is enabled  */
} hal_rcc_osc_enable_status_t;

/**
  * @brief Oscillator ready status.
  */
typedef enum
{
  HAL_RCC_OSC_NOT_READY = 0U,                      /*!< Oscillator is not ready */
  HAL_RCC_OSC_READY     = ! HAL_RCC_OSC_NOT_READY, /*!< Oscillator is ready     */
} hal_rcc_osc_ready_status_t;

/**
  * @brief Oscillator Stop mode status.
  */
typedef enum
{
  HAL_RCC_OSC_DISABLED_IN_STOP_MODE = 0U,                                  /*!< Oscillator is enabled in Run mode only      */
  HAL_RCC_OSC_ENABLED_IN_STOP_MODE  = ! HAL_RCC_OSC_DISABLED_IN_STOP_MODE, /*!< Oscillator is enabled in Run and stop modes */
} hal_rcc_osc_stop_mode_status_t;

/**
  * @brief Clock status
  */
typedef enum
{
  HAL_RCC_CLK_DISABLED = 0U,                      /*!< Clock is disabled */
  HAL_RCC_CLK_ENABLED = ! HAL_RCC_CLK_DISABLED    /*!< Clock is enabled */
} hal_rcc_clk_status_t;

#if defined(HSE_VALUE)
/**
  * @brief HSE state.
  */
typedef enum
{
  HAL_RCC_HSE_ON             = RCC_CR1_HSEON,                                     /*!< HSE clock activation   */
  HAL_RCC_HSE_BYPASS         = (RCC_CR1_HSEBYP | RCC_CR1_HSEON),                  /*!< External Analog clock source for HSE clock */
  HAL_RCC_HSE_BYPASS_DIGITAL = (RCC_CR1_HSEEXT | RCC_CR1_HSEBYP | RCC_CR1_HSEON), /*!< External Digital clock source
                                                                                       for HSE clock  */
} hal_rcc_hse_t;

#endif /* HSE_VALUE */
#if defined(LSE_VALUE)
/**
  * @brief LSE State.
  */
typedef enum
{
  HAL_RCC_LSE_ON              = RCC_RTCCR_LSEON,                                         /*!< LSE clock activation                        */
  HAL_RCC_LSE_BYPASS          = (RCC_RTCCR_LSEBYP | RCC_RTCCR_LSEON),                    /*!< External clock source for LSE clock         */
  HAL_RCC_LSE_BYPASS_DIGITAL  = (RCC_RTCCR_LSEBYP | RCC_RTCCR_LSEEXT | RCC_RTCCR_LSEON), /*!< External digital clock source for LSE clock */
} hal_rcc_lse_t;

#endif /* LSE_VALUE */

/**
  * @brief HSIK Clock Divider.
  */
typedef enum
{
  HAL_RCC_HSIK_DIV1      = LL_RCC_HSIK_DIV_1,          /*!< HSIK = HSI divided by 1    */
  HAL_RCC_HSIK_DIV1_5    = LL_RCC_HSIK_DIV_1_5,        /*!< HSIK = HSI divided by 1.5  */
  HAL_RCC_HSIK_DIV2      = LL_RCC_HSIK_DIV_2,          /*!< HSIK = HSI divided by 2    */
  HAL_RCC_HSIK_DIV2_5    = LL_RCC_HSIK_DIV_2_5,        /*!< HSIK = HSI divided by 2.5  */
  HAL_RCC_HSIK_DIV3      = LL_RCC_HSIK_DIV_3,          /*!< HSIK = HSI divided by 3    */
  HAL_RCC_HSIK_DIV3_5    = LL_RCC_HSIK_DIV_3_5,        /*!< HSIK = HSI divided by 3.5  */
  HAL_RCC_HSIK_DIV4      = LL_RCC_HSIK_DIV_4,          /*!< HSIK = HSI divided by 4    */
  HAL_RCC_HSIK_DIV4_5    = LL_RCC_HSIK_DIV_4_5,        /*!< HSIK = HSI divided by 4.5  */
  HAL_RCC_HSIK_DIV5      = LL_RCC_HSIK_DIV_5,          /*!< HSIK = HSI divided by 5    */
  HAL_RCC_HSIK_DIV5_5    = LL_RCC_HSIK_DIV_5_5,        /*!< HSIK = HSI divided by 5.5  */
  HAL_RCC_HSIK_DIV6      = LL_RCC_HSIK_DIV_6,          /*!< HSIK = HSI divided by 6    */
  HAL_RCC_HSIK_DIV6_5    = LL_RCC_HSIK_DIV_6_5,        /*!< HSIK = HSI divided by 6.5  */
  HAL_RCC_HSIK_DIV7      = LL_RCC_HSIK_DIV_7,          /*!< HSIK = HSI divided by 7    */
  HAL_RCC_HSIK_DIV7_5    = LL_RCC_HSIK_DIV_7_5,        /*!< HSIK = HSI divided by 7.5  */
  HAL_RCC_HSIK_DIV8      = LL_RCC_HSIK_DIV_8,          /*!< HSIK = HSI divided by 8    */
} hal_rcc_hsik_div_t;

/**
  * @brief PSIK Clock Divider.
  */
typedef enum
{
  HAL_RCC_PSIK_DIV1      = LL_RCC_PSIK_DIV_1,          /*!< PSIK = PSI divided by 1    */
  HAL_RCC_PSIK_DIV1_5    = LL_RCC_PSIK_DIV_1_5,        /*!< PSIK = PSI divided by 1.5  */
  HAL_RCC_PSIK_DIV2      = LL_RCC_PSIK_DIV_2,          /*!< PSIK = PSI divided by 2    */
  HAL_RCC_PSIK_DIV2_5    = LL_RCC_PSIK_DIV_2_5,        /*!< PSIK = PSI divided by 2.5  */
  HAL_RCC_PSIK_DIV3      = LL_RCC_PSIK_DIV_3,          /*!< PSIK = PSI divided by 3    */
  HAL_RCC_PSIK_DIV3_5    = LL_RCC_PSIK_DIV_3_5,        /*!< PSIK = PSI divided by 3.5  */
  HAL_RCC_PSIK_DIV4      = LL_RCC_PSIK_DIV_4,          /*!< PSIK = PSI divided by 4    */
  HAL_RCC_PSIK_DIV4_5    = LL_RCC_PSIK_DIV_4_5,        /*!< PSIK = PSI divided by 4.5  */
  HAL_RCC_PSIK_DIV5      = LL_RCC_PSIK_DIV_5,          /*!< PSIK = PSI divided by 5    */
  HAL_RCC_PSIK_DIV5_5    = LL_RCC_PSIK_DIV_5_5,        /*!< PSIK = PSI divided by 5.5  */
  HAL_RCC_PSIK_DIV6      = LL_RCC_PSIK_DIV_6,          /*!< PSIK = PSI divided by 6    */
  HAL_RCC_PSIK_DIV6_5    = LL_RCC_PSIK_DIV_6_5,        /*!< PSIK = PSI divided by 6.5  */
  HAL_RCC_PSIK_DIV7      = LL_RCC_PSIK_DIV_7,          /*!< PSIK = PSI divided by 7    */
  HAL_RCC_PSIK_DIV7_5    = LL_RCC_PSIK_DIV_7_5,        /*!< PSIK = PSI divided by 7.5  */
  HAL_RCC_PSIK_DIV8      = LL_RCC_PSIK_DIV_8,          /*!< PSIK = PSI divided by 8    */
} hal_rcc_psik_div_t;

/**
  * @brief  RCC PSI Reference Clock Source.
  */
typedef enum
{
  HAL_RCC_PSI_SRC_HSE       = LL_RCC_PSISOURCE_HSE,       /*!< HSE clock selected as PSI oscillator entry clock source */
  HAL_RCC_PSI_SRC_LSE       = LL_RCC_PSISOURCE_LSE,       /*!< LSE clock selected as PSI oscillator entry clock source */
  HAL_RCC_PSI_SRC_HSI_8MHz  = LL_RCC_PSISOURCE_HSIDIV18,  /*!< HSI clock divided by 18 (144/18 = 8MHz) selected
                                                                 as PSI oscillator entry clock source */
} hal_rcc_psi_src_t;

/**
  * @brief PSI reference clock frequency selection.
  */
typedef enum
{
  HAL_RCC_PSI_REF_32768HZ      = LL_RCC_PSIREF_32768HZ,/*!< PSI reference clock input is 32768 Hz */
  HAL_RCC_PSI_REF_8MHZ         = LL_RCC_PSIREF_8MHZ,   /*!< PSI reference clock input is 8 MHz    */
  HAL_RCC_PSI_REF_16MHZ        = LL_RCC_PSIREF_16MHZ,  /*!< PSI reference clock input is 16 MHz   */
  HAL_RCC_PSI_REF_24MHZ        = LL_RCC_PSIREF_24MHZ,  /*!< PSI reference clock input is 24 MHz   */
  HAL_RCC_PSI_REF_25MHZ        = LL_RCC_PSIREF_25MHZ,  /*!< PSI reference clock input is 25 MHz   */
  HAL_RCC_PSI_REF_32MHZ        = LL_RCC_PSIREF_32MHZ,  /*!< PSI reference clock input is 32 MHz   */
  HAL_RCC_PSI_REF_48MHZ        = LL_RCC_PSIREF_48MHZ,  /*!< PSI reference clock input is 48 MHz   */
  HAL_RCC_PSI_REF_50MHZ        = LL_RCC_PSIREF_50MHZ,  /*!< PSI reference clock input is 50 MHz   */
} hal_rcc_psi_ref_t;

/**
  * @brief PSI Clock frequency selection.
  */
typedef enum
{
  HAL_RCC_PSI_OUT_100MHZ      = LL_RCC_PSIFREQ_100MHZ,   /*!< PSI output frequency is 100 MHz */
  HAL_RCC_PSI_OUT_144MHZ      = LL_RCC_PSIFREQ_144MHZ,   /*!< PSI output frequency is 144 MHz */
  HAL_RCC_PSI_OUT_160MHZ      = LL_RCC_PSIFREQ_160MHZ,   /*!< PSI output frequency is 160 MHz */
} hal_rcc_psi_out_t;

/**
  * @brief System Clock Source.
  */
typedef enum
{
  HAL_RCC_SYSCLK_SRC_HSIS           = LL_RCC_SYS_CLKSOURCE_HSIS,    /*!< HSIS selected as system clock      */
  HAL_RCC_SYSCLK_SRC_HSIDIV3        = LL_RCC_SYS_CLKSOURCE_HSIDIV3, /*!< HSIDIV3 selection as system clock  */
  HAL_RCC_SYSCLK_SRC_HSE            = LL_RCC_SYS_CLKSOURCE_HSE,     /*!< HSE selection as system clock      */
  HAL_RCC_SYSCLK_SRC_PSIS           = LL_RCC_SYS_CLKSOURCE_PSIS,    /*!< PSIS selection as system clock     */
} hal_rcc_sysclk_src_t;

/**
  * @brief AHB Clock Source.
  */
typedef enum
{
  HAL_RCC_HCLK_PRESCALER1               = LL_RCC_HCLK_PRESCALER_1,   /*!< SYSCLK not divided    */
  HAL_RCC_HCLK_PRESCALER2               = LL_RCC_HCLK_PRESCALER_2,   /*!< SYSCLK divided by 2   */
  HAL_RCC_HCLK_PRESCALER4               = LL_RCC_HCLK_PRESCALER_4,   /*!< SYSCLK divided by 4   */
  HAL_RCC_HCLK_PRESCALER8               = LL_RCC_HCLK_PRESCALER_8,   /*!< SYSCLK divided by 8   */
  HAL_RCC_HCLK_PRESCALER16              = LL_RCC_HCLK_PRESCALER_16,  /*!< SYSCLK divided by 16  */
  HAL_RCC_HCLK_PRESCALER64              = LL_RCC_HCLK_PRESCALER_64,  /*!< SYSCLK divided by 64  */
  HAL_RCC_HCLK_PRESCALER128             = LL_RCC_HCLK_PRESCALER_128, /*!< SYSCLK divided by 128 */
  HAL_RCC_HCLK_PRESCALER256             = LL_RCC_HCLK_PRESCALER_256, /*!< SYSCLK divided by 256 */
  HAL_RCC_HCLK_PRESCALER512             = LL_RCC_HCLK_PRESCALER_512, /*!< SYSCLK divided by 512 */
} hal_rcc_hclk_prescaler_t;

/**
  * @brief APB1 APB2 APB3 Clock Source.
  */
typedef enum
{
  HAL_RCC_PCLK_PRESCALER1                 = LL_RCC_APB1_PRESCALER_1,  /*!< HCLK not divided   */
  HAL_RCC_PCLK_PRESCALER2                 = LL_RCC_APB1_PRESCALER_2,  /*!< HCLK divided by 2  */
  HAL_RCC_PCLK_PRESCALER4                 = LL_RCC_APB1_PRESCALER_4,  /*!< HCLK divided by 4  */
  HAL_RCC_PCLK_PRESCALER8                 = LL_RCC_APB1_PRESCALER_8,  /*!< HCLK divided by 8  */
  HAL_RCC_PCLK_PRESCALER16                = LL_RCC_APB1_PRESCALER_16, /*!< HCLK divided by 16 */
} hal_rcc_pclk_prescaler_t;

/**
  * @brief MCO Clock Source.
  */
typedef enum
{
  HAL_RCC_MCO1_SRC_SYSCLK  = LL_RCC_MCO1SOURCE_SYSCLK,  /*!< SYSCLK selection as MCO1 source  */
  HAL_RCC_MCO1_SRC_HSE     = LL_RCC_MCO1SOURCE_HSE,     /*!< HSE selection as MCO1 source     */
  HAL_RCC_MCO1_SRC_LSE     = LL_RCC_MCO1SOURCE_LSE,     /*!< LSE selection as MCO1 source     */
  HAL_RCC_MCO1_SRC_LSI     = LL_RCC_MCO1SOURCE_LSI,     /*!< LSI selection as MCO1 source     */
  HAL_RCC_MCO1_SRC_PSIK    = LL_RCC_MCO1SOURCE_PSIK,    /*!< PSIK selection as MCO1 source    */
  HAL_RCC_MCO1_SRC_HSIK    = LL_RCC_MCO1SOURCE_HSIK,    /*!< HSIK selection as MCO1 source    */
  HAL_RCC_MCO1_SRC_PSIS    = LL_RCC_MCO1SOURCE_PSIS,    /*!< PSIS selection as MCO1 source    */
  HAL_RCC_MCO1_SRC_HSIS    = LL_RCC_MCO1SOURCE_HSIS,    /*!< HSIS selection as MCO1 source    */
  HAL_RCC_MCO2_SRC_SYSCLK  = LL_RCC_MCO2SOURCE_SYSCLK,  /*!< SYSCLK selection as MCO2 source  */
  HAL_RCC_MCO2_SRC_HSE     = LL_RCC_MCO2SOURCE_HSE,     /*!< HSE selection as MCO2 source     */
  HAL_RCC_MCO2_SRC_LSE     = LL_RCC_MCO2SOURCE_LSE,     /*!< LSE selection as MCO2 source     */
  HAL_RCC_MCO2_SRC_LSI     = LL_RCC_MCO2SOURCE_LSI,     /*!< LSI selection as MCO2 source     */
  HAL_RCC_MCO2_SRC_PSIK    = LL_RCC_MCO2SOURCE_PSIK,    /*!< PSIK selection as MCO2 source    */
  HAL_RCC_MCO2_SRC_HSIK    = LL_RCC_MCO2SOURCE_HSIK,    /*!< HSIK  selection as MCO2 source   */
  HAL_RCC_MCO2_SRC_PSIDIV3 = LL_RCC_MCO2SOURCE_PSIDIV3, /*!< PSIDIV3 selection as MCO2 source */
  HAL_RCC_MCO2_SRC_HSIDIV3 = LL_RCC_MCO2SOURCE_HSIDIV3  /*!< HSIDIV3 selection as MCO2 source */
} hal_rcc_mco_src_t;

/**
  * @brief MCO Clock Prescaler.
  */
typedef enum
{
  HAL_RCC_MCO1_NO_CLK       = LL_RCC_MCO1SOURCE_NOCLOCK,   /*!< MCO1 output disabled, no clock on MCO1 */
  HAL_RCC_MCO1_PRESCALER1   = LL_RCC_MCO1_PRESCALER_1,    /*!< MCO1 clock is divided by 1  */
  HAL_RCC_MCO1_PRESCALER2   = LL_RCC_MCO1_PRESCALER_2,    /*!< MCO1 clock is divided by 2  */
  HAL_RCC_MCO1_PRESCALER3   = LL_RCC_MCO1_PRESCALER_3,    /*!< MCO1 clock is divided by 3  */
  HAL_RCC_MCO1_PRESCALER4   = LL_RCC_MCO1_PRESCALER_4,    /*!< MCO1 clock is divided by 4  */
  HAL_RCC_MCO1_PRESCALER5   = LL_RCC_MCO1_PRESCALER_5,    /*!< MCO1 clock is divided by 5  */
  HAL_RCC_MCO1_PRESCALER6   = LL_RCC_MCO1_PRESCALER_6,    /*!< MCO1 clock is divided by 6  */
  HAL_RCC_MCO1_PRESCALER7   = LL_RCC_MCO1_PRESCALER_7,    /*!< MCO1 clock is divided by 7  */
  HAL_RCC_MCO1_PRESCALER8   = LL_RCC_MCO1_PRESCALER_8,    /*!< MCO1 clock is divided by 8  */
  HAL_RCC_MCO1_PRESCALER9   = LL_RCC_MCO1_PRESCALER_9,    /*!< MCO1 clock is divided by 9  */
  HAL_RCC_MCO1_PRESCALER10  = LL_RCC_MCO1_PRESCALER_10,   /*!< MCO1 clock is divided by 10 */
  HAL_RCC_MCO1_PRESCALER11  = LL_RCC_MCO1_PRESCALER_11,   /*!< MCO1 clock is divided by 11 */
  HAL_RCC_MCO1_PRESCALER12  = LL_RCC_MCO1_PRESCALER_12,   /*!< MCO1 clock is divided by 12 */
  HAL_RCC_MCO1_PRESCALER13  = LL_RCC_MCO1_PRESCALER_13,   /*!< MCO1 clock is divided by 13 */
  HAL_RCC_MCO1_PRESCALER14  = LL_RCC_MCO1_PRESCALER_14,   /*!< MCO1 clock is divided by 14 */
  HAL_RCC_MCO1_PRESCALER15  = LL_RCC_MCO1_PRESCALER_15,   /*!< MCO1 clock is divided by 15 */
  HAL_RCC_MCO2_NO_CLK       = LL_RCC_MCO2SOURCE_NOCLOCK,  /*!< MCO2 output disabled, no clock on MCO2 */
  HAL_RCC_MCO2_PRESCALER1   = LL_RCC_MCO2_PRESCALER_1,    /*!< MCO2 clock is divided by 1  */
  HAL_RCC_MCO2_PRESCALER2   = LL_RCC_MCO2_PRESCALER_2,    /*!< MCO2 clock is divided by 2  */
  HAL_RCC_MCO2_PRESCALER3   = LL_RCC_MCO2_PRESCALER_3,    /*!< MCO2 clock is divided by 3  */
  HAL_RCC_MCO2_PRESCALER4   = LL_RCC_MCO2_PRESCALER_4,    /*!< MCO2 clock is divided by 4  */
  HAL_RCC_MCO2_PRESCALER5   = LL_RCC_MCO2_PRESCALER_5,    /*!< MCO2 clock is divided by 5  */
  HAL_RCC_MCO2_PRESCALER6   = LL_RCC_MCO2_PRESCALER_6,    /*!< MCO2 clock is divided by 6  */
  HAL_RCC_MCO2_PRESCALER7   = LL_RCC_MCO2_PRESCALER_7,    /*!< MCO2 clock is divided by 7  */
  HAL_RCC_MCO2_PRESCALER8   = LL_RCC_MCO2_PRESCALER_8,    /*!< MCO2 clock is divided by 8  */
  HAL_RCC_MCO2_PRESCALER9   = LL_RCC_MCO2_PRESCALER_9,    /*!< MCO2 clock is divided by 9  */
  HAL_RCC_MCO2_PRESCALER10  = LL_RCC_MCO2_PRESCALER_10,   /*!< MCO2 clock is divided by 10 */
  HAL_RCC_MCO2_PRESCALER11  = LL_RCC_MCO2_PRESCALER_11,   /*!< MCO2 clock is divided by 11 */
  HAL_RCC_MCO2_PRESCALER12  = LL_RCC_MCO2_PRESCALER_12,   /*!< MCO2 clock is divided by 12 */
  HAL_RCC_MCO2_PRESCALER13  = LL_RCC_MCO2_PRESCALER_13,   /*!< MCO2 clock is divided by 13 */
  HAL_RCC_MCO2_PRESCALER14  = LL_RCC_MCO2_PRESCALER_14,   /*!< MCO2 clock is divided by 14 */
  HAL_RCC_MCO2_PRESCALER15  = LL_RCC_MCO2_PRESCALER_15,   /*!< MCO2 clock is divided by 15 */
} hal_rcc_mco_prescaler_t;

#if defined(LSE_VALUE)
/**
  * @brief LSE Drive Configuration.
  */
typedef enum
{
  HAL_RCC_LSE_DRIVE_LOW                = LL_RCC_LSEDRIVE_LOW,        /*!< LSE low drive capability         */
  HAL_RCC_LSE_DRIVE_MEDIUMLOW          = LL_RCC_LSEDRIVE_MEDIUMLOW,  /*!< LSE medium low drive capability  */
  HAL_RCC_LSE_DRIVE_MEDIUMHIGH         = LL_RCC_LSEDRIVE_MEDIUMHIGH, /*!< LSE medium high drive capability */
  HAL_RCC_LSE_DRIVE_HIGH               = LL_RCC_LSEDRIVE_HIGH,       /*!< LSE high drive capability        */
} hal_rcc_lse_drive_t;

#endif /* LSE_VALUE */

/**
  * @brief Wake-Up from STOP Clock.
  */
typedef enum
{
  HAL_RCC_STOP_WAKEUPCLOCK_HSIDIV3     = LL_RCC_STOP_WAKEUPCLOCK_HSIDIV3, /*!< HSIDIV3 selected after wake-up from STOP */
  HAL_RCC_STOP_WAKEUPCLOCK_HSIS        = LL_RCC_STOP_WAKEUPCLOCK_HSIS,    /*!< HSIS selected after wake-up from STOP    */
} hal_rcc_stop_wakeup_clk_t;


/**
  * @brief Low Speed Clock Source.
  */
typedef enum
{
  HAL_RCC_LSCO_SRC_LSI              = LL_RCC_LSCO_CLKSOURCE_LSI,      /*!< LSI selected for low speed clock output */
  HAL_RCC_LSCO_SRC_LSE              = LL_RCC_LSCO_CLKSOURCE_LSE,      /*!< LSE selected for low speed clock output */
} hal_rcc_lsco_src_t;

/**
  * @brief RTC Clock Source.
  */
typedef enum
{
  HAL_RCC_RTC_CLK_SRC_NONE         = LL_RCC_RTC_CLKSOURCE_NONE,      /*!< No clock used as RTC clock                 */
  HAL_RCC_RTC_CLK_SRC_LSE          = LL_RCC_RTC_CLKSOURCE_LSE,       /*!< LSE oscillator clock used as RTC clock     */
  HAL_RCC_RTC_CLK_SRC_LSI          = LL_RCC_RTC_CLKSOURCE_LSI,       /*!< LSI oscillator clock used as RTC clock     */
  HAL_RCC_RTC_CLK_SRC_HSE_DIV      = LL_RCC_RTC_CLKSOURCE_HSE_DIV,   /*!< HSE oscillator clock divided by RTCPRE[]
                                                                           used as RTC clock                         */
} hal_rcc_rtc_clk_src_t;

/**
  * @brief USART1 Clock Source.
  */
typedef enum
{
  HAL_RCC_USART1_CLK_SRC_PCLK2      = LL_RCC_USART1_CLKSOURCE_PCLK2, /*!< PCLK2 selected as USART1 kernel clock */
  HAL_RCC_USART1_CLK_SRC_PSIK       = LL_RCC_USART1_CLKSOURCE_PSIK,  /*!< PSIK selected as USART1 kernel clock  */
  HAL_RCC_USART1_CLK_SRC_HSIK       = LL_RCC_USART1_CLKSOURCE_HSIK,  /*!< HSIK selected as USART1 kernel clock  */
  HAL_RCC_USART1_CLK_SRC_LSE        = LL_RCC_USART1_CLKSOURCE_LSE,   /*!< LSE selected as USART1 kernel clock   */
} hal_rcc_usart1_clk_src_t;

/**
  * @brief USART2 Clock Source.
  */
typedef enum
{
  HAL_RCC_USART2_CLK_SRC_PCLK1      = LL_RCC_USART2_CLKSOURCE_PCLK1, /*!< PCLK1 selected as USART2 kernel clock */
  HAL_RCC_USART2_CLK_SRC_PSIK       = LL_RCC_USART2_CLKSOURCE_PSIK,  /*!< PSIK selected as USART2 kernel clock  */
  HAL_RCC_USART2_CLK_SRC_HSIK       = LL_RCC_USART2_CLKSOURCE_HSIK,  /*!< HSIK selected as USART2 kernel clock  */
  HAL_RCC_USART2_CLK_SRC_LSE        = LL_RCC_USART2_CLKSOURCE_LSE,   /*!< LSE selected as USART2 kernel clock   */
} hal_rcc_usart2_clk_src_t;

#if defined(USART3)
/**
  * @brief USART3 Clock Source.
  */
typedef enum
{
  HAL_RCC_USART3_CLK_SRC_PCLK1      = LL_RCC_USART3_CLKSOURCE_PCLK1, /*!< PCLK1 selected as USART3 kernel clock */
  HAL_RCC_USART3_CLK_SRC_PSIK       = LL_RCC_USART3_CLKSOURCE_PSIK,  /*!< PSIK selected as USART3 kernel clock  */
  HAL_RCC_USART3_CLK_SRC_HSIK       = LL_RCC_USART3_CLKSOURCE_HSIK,  /*!< HSIK selected as USART3 kernel clock  */
  HAL_RCC_USART3_CLK_SRC_LSE        = LL_RCC_USART3_CLKSOURCE_LSE,   /*!< LSE selected as USART3 kernel clock   */
} hal_rcc_usart3_clk_src_t;

#endif /* USART3 */
/**
  * @brief UART4 Clock Source.
  */
typedef enum
{
  HAL_RCC_UART4_CLK_SRC_PCLK1       = LL_RCC_UART4_CLKSOURCE_PCLK1, /*!< PCLK1 selected as UART4 kernel clock */
  HAL_RCC_UART4_CLK_SRC_PSIK        = LL_RCC_UART4_CLKSOURCE_PSIK,  /*!< PSIK selected as UART4 kernel clock  */
  HAL_RCC_UART4_CLK_SRC_HSIK        = LL_RCC_UART4_CLKSOURCE_HSIK,  /*!< HSIK selected as UART4 kernel clock  */
  HAL_RCC_UART4_CLK_SRC_LSE         = LL_RCC_UART4_CLKSOURCE_LSE,   /*!< LSE selected as UART4 kernel clock   */
} hal_rcc_uart4_clk_src_t;

/**
  * @brief UART5 Clock Source.
  */
typedef enum
{
  HAL_RCC_UART5_CLK_SRC_PCLK1       = LL_RCC_UART5_CLKSOURCE_PCLK1, /*!< PCLK1 selected as UART5 kernel clock */
  HAL_RCC_UART5_CLK_SRC_PSIK        = LL_RCC_UART5_CLKSOURCE_PSIK,  /*!< PSIK selected as UART5 kernel clock  */
  HAL_RCC_UART5_CLK_SRC_HSIK        = LL_RCC_UART5_CLKSOURCE_HSIK,  /*!< HSIK selected as UART5 kernel clock  */
  HAL_RCC_UART5_CLK_SRC_LSE         = LL_RCC_UART5_CLKSOURCE_LSE,   /*!< LSE selected as UART5 kernel clock   */
} hal_rcc_uart5_clk_src_t;

#if defined(USART6)
/**
  * @brief USART6 Clock Source.
  */
typedef enum
{
  HAL_RCC_USART6_CLK_SRC_PCLK1      = LL_RCC_USART6_CLKSOURCE_PCLK1, /*!< PCLK1 selected as USART6 kernel clock */
  HAL_RCC_USART6_CLK_SRC_PSIK       = LL_RCC_USART6_CLKSOURCE_PSIK,  /*!< PSIK selected as USART6 kernel clock  */
  HAL_RCC_USART6_CLK_SRC_HSIK       = LL_RCC_USART6_CLKSOURCE_HSIK,  /*!< HSIK selected as USART6 kernel clock  */
  HAL_RCC_USART6_CLK_SRC_LSE        = LL_RCC_USART6_CLKSOURCE_LSE,   /*!< LSE selected as USART6 kernel clock   */
} hal_rcc_usart6_clk_src_t;

#endif /* USART6 */
#if defined(UART7)
/**
  * @brief UART7 Clock Source.
  */
typedef enum
{
  HAL_RCC_UART7_CLK_SRC_PCLK1       = LL_RCC_UART7_CLKSOURCE_PCLK1, /*!< PCLK1 selected as UART7 kernel clock */
  HAL_RCC_UART7_CLK_SRC_PSIK        = LL_RCC_UART7_CLKSOURCE_PSIK,  /*!< PSIK selected as UART7 kernel clock  */
  HAL_RCC_UART7_CLK_SRC_HSIK        = LL_RCC_UART7_CLKSOURCE_HSIK,  /*!< HSIK selected as UART7 kernel clock  */
  HAL_RCC_UART7_CLK_SRC_LSE         = LL_RCC_UART7_CLKSOURCE_LSE,   /*!< LSE selected as UART7 kernel clock   */
} hal_rcc_uart7_clk_src_t;

#endif /* UART7 */
/**
  * @brief LPUART1 Clock Source.
  */
typedef enum
{
  HAL_RCC_LPUART1_CLK_SRC_PCLK3     = LL_RCC_LPUART1_CLKSOURCE_PCLK3, /*!< PCLK3 selected as LPUART1 kernel clock */
  HAL_RCC_LPUART1_CLK_SRC_HSIK      = LL_RCC_LPUART1_CLKSOURCE_HSIK,  /*!< HSIK selected as LPUART1 kernel clock  */
  HAL_RCC_LPUART1_CLK_SRC_LSE       = LL_RCC_LPUART1_CLKSOURCE_LSE,   /*!< LSE selected as LPUART1 kernel clock   */
  HAL_RCC_LPUART1_CLK_SRC_LSI       = LL_RCC_LPUART1_CLKSOURCE_LSI,   /*!< LSI selected as LPUART1 kernel clock   */
} hal_rcc_lpuart1_clk_src_t;

/**
  * @brief SPI1 Clock Source.
  */
typedef enum
{
  HAL_RCC_SPI1_CLK_SRC_PCLK2        = LL_RCC_SPI1_CLKSOURCE_PCLK2,     /*!< PCLK2 selected as SPI1 kernel clock    */
  HAL_RCC_SPI1_CLK_SRC_PSIK         = LL_RCC_SPI1_CLKSOURCE_PSIK,      /*!< PSIK selected as SPI1 kernel clock     */
  HAL_RCC_SPI1_CLK_SRC_HSIK         = LL_RCC_SPI1_CLKSOURCE_HSIK,      /*!< HSIK selected as SPI1 kernel clock     */
  HAL_RCC_SPI1_CLK_SRC_AUDIOCLK     = LL_RCC_SPI1_CLKSOURCE_AUDIOCLK,  /*!< AUDIOCLK selected as SPI1 kernel clock */
} hal_rcc_spi1_clk_src_t;

/**
  * @brief SPI2 Clock Source.
  */
typedef enum
{
  HAL_RCC_SPI2_CLK_SRC_PCLK1        = LL_RCC_SPI2_CLKSOURCE_PCLK1,     /*!< PCLK1 selected as SPI2 kernel clock    */
  HAL_RCC_SPI2_CLK_SRC_PSIK         = LL_RCC_SPI2_CLKSOURCE_PSIK,      /*!< PSIK selected as SPI2 kernel clock     */
  HAL_RCC_SPI2_CLK_SRC_HSIK         = LL_RCC_SPI2_CLKSOURCE_HSIK,      /*!< HSIK selected as SPI2 kernel clock     */
  HAL_RCC_SPI2_CLK_SRC_AUDIOCLK     = LL_RCC_SPI2_CLKSOURCE_AUDIOCLK,  /*!< AUDIOCLK selected as SPI2 kernel clock */
} hal_rcc_spi2_clk_src_t;

#if defined(SPI3)
/**
  * @brief SPI3 Clock Source.
  */
typedef enum
{
  HAL_RCC_SPI3_CLK_SRC_PCLK1        = LL_RCC_SPI3_CLKSOURCE_PCLK1,     /*!< PCLK1 selected as SPI3 kernel clock    */
  HAL_RCC_SPI3_CLK_SRC_PSIK         = LL_RCC_SPI3_CLKSOURCE_PSIK,      /*!< PSIK selected as SPI3 kernel clock     */
  HAL_RCC_SPI3_CLK_SRC_HSIK         = LL_RCC_SPI3_CLKSOURCE_HSIK,      /*!< HSIK selected as SPI3 kernel clock     */
  HAL_RCC_SPI3_CLK_SRC_AUDIOCLK     = LL_RCC_SPI3_CLKSOURCE_AUDIOCLK,  /*!< AUDIOCLK selected as SPI3 kernel clock */
} hal_rcc_spi3_clk_src_t;

#endif /* SPI3 */
#if defined(FDCAN1)
/**
  * @brief FDCAN Kernel Clock Source.
  */
typedef enum
{
  HAL_RCC_FDCAN_CLK_SRC_PCLK1       = LL_RCC_FDCAN_CLKSOURCE_PCLK1, /*!< PCLK1 selected as FDCAN kernel clock */
  HAL_RCC_FDCAN_CLK_SRC_PSIS        = LL_RCC_FDCAN_CLKSOURCE_PSIS,  /*!< PSIS selected as FDCAN kernel clock  */
  HAL_RCC_FDCAN_CLK_SRC_PSIK        = LL_RCC_FDCAN_CLKSOURCE_PSIK,  /*!< PSIK selected as FDCAN kernel clock  */
  HAL_RCC_FDCAN_CLK_SRC_HSE         = LL_RCC_FDCAN_CLKSOURCE_HSE,   /*!< HSE selected as FDCAN kernel clock   */
} hal_rcc_fdcan_clk_src_t;

#endif /* FDCAN1 */
/**
  * @brief I2C1 Clock Source.
  */
typedef enum
{
  HAL_RCC_I2C1_CLK_SRC_PCLK1        = LL_RCC_I2C1_CLKSOURCE_PCLK1, /*!< PCLK1 selected as I2C1 kernel clock */
  HAL_RCC_I2C1_CLK_SRC_PSIK         = LL_RCC_I2C1_CLKSOURCE_PSIK,  /*!< PSIK selected as I2C1 kernel clock  */
  HAL_RCC_I2C1_CLK_SRC_HSIK         = LL_RCC_I2C1_CLKSOURCE_HSIK,  /*!< HSIK selected as I2C1 kernel clock  */
} hal_rcc_i2c1_clk_src_t;

#if defined(I2C2)
/**
  * @brief I2C2 Clock Source.
  */
typedef enum
{
  HAL_RCC_I2C2_CLK_SRC_PCLK1        = LL_RCC_I2C2_CLKSOURCE_PCLK1, /*!< PCLK1 selected as I2C2 kernel clock */
  HAL_RCC_I2C2_CLK_SRC_PSIK         = LL_RCC_I2C2_CLKSOURCE_PSIK,  /*!< PSIK selected as I2C2 kernel clock  */
  HAL_RCC_I2C2_CLK_SRC_HSIK         = LL_RCC_I2C2_CLKSOURCE_HSIK,  /*!< HSIK selected as I2C2 kernel clock  */
} hal_rcc_i2c2_clk_src_t;

#endif /* I2C2 */
/**
  * @brief I3C1 Clock Source.
  */
typedef enum
{
  HAL_RCC_I3C1_CLK_SRC_PCLK1        = LL_RCC_I3C1_CLKSOURCE_PCLK1, /*!< PCLK1 selected as I3C1 kernel clock */
  HAL_RCC_I3C1_CLK_SRC_PSIK         = LL_RCC_I3C1_CLKSOURCE_PSIK,  /*!< PSIK selected as I3C1 kernel clock  */
  HAL_RCC_I3C1_CLK_SRC_HSIK         = LL_RCC_I3C1_CLKSOURCE_HSIK,  /*!< HSIK selected as I3C1 kernel clock  */
} hal_rcc_i3c1_clk_src_t;

/**
  * @brief ADCDAC Clock Source.
  */
typedef enum
{
  HAL_RCC_ADCDAC_CLK_SRC_HCLK       = LL_RCC_ADCDAC_CLKSOURCE_HCLK,   /*!< HCLK selected as ADCDAC kernel clock */
  HAL_RCC_ADCDAC_CLK_SRC_PSIS       = LL_RCC_ADCDAC_CLKSOURCE_PSIS,   /*!< PSIS selected as ADCDAC kernel clock */
  HAL_RCC_ADCDAC_CLK_SRC_PSIK       = LL_RCC_ADCDAC_CLKSOURCE_PSIK,   /*!< PSIK selected as ADCDAC kernel clock */
  HAL_RCC_ADCDAC_CLK_SRC_HSIK       = LL_RCC_ADCDAC_CLKSOURCE_HSIK,   /*!< HSIK selected as ADCDAC kernel clock */
} hal_rcc_adcdac_clk_src_t;

/**
  * @brief Clock Prescaler for ADCDAC kernel clock.
  */
typedef enum
{
  HAL_RCC_ADCDAC_PRESCALER1   = LL_RCC_ADCDAC_PRESCALER_1,  /*!< Prescaler 1 for ADC and DAC kernel clock   */
  HAL_RCC_ADCDAC_PRESCALER2   = LL_RCC_ADCDAC_PRESCALER_2,  /*!< Prescaler 2 for ADC and DAC kernel clock   */
  HAL_RCC_ADCDAC_PRESCALER4   = LL_RCC_ADCDAC_PRESCALER_4,  /*!< Prescaler 4 for ADC and DAC kernel clock   */
  HAL_RCC_ADCDAC_PRESCALER8   = LL_RCC_ADCDAC_PRESCALER_8,  /*!< Prescaler 8 for ADC and DAC kernel clock   */
  HAL_RCC_ADCDAC_PRESCALER16  = LL_RCC_ADCDAC_PRESCALER_16, /*!< Prescaler 16 for ADC and DAC kernel clock  */
  HAL_RCC_ADCDAC_PRESCALER32  = LL_RCC_ADCDAC_PRESCALER_32, /*!< Prescaler 32 for ADC and DAC kernel clock  */
  HAL_RCC_ADCDAC_PRESCALER64  = LL_RCC_ADCDAC_PRESCALER_64, /*!< Prescaler 64 for ADC and DAC kernel clock  */
  HAL_RCC_ADCDAC_PRESCALER128 = LL_RCC_ADCDAC_PRESCALER_128 /*!< Prescaler 128 for ADC and DAC kernel clock */
} hal_rcc_adcdac_prescaler_t;
/**
  * @brief DAC1 sample-and-hold clock source.
  */
typedef enum
{
  HAL_RCC_DAC1_SH_CLK_SRC_LSE = LL_RCC_DAC1SH_CLKSOURCE_LSE, /*!< LSE selected as DAC1 SH clock source */
  HAL_RCC_DAC1_SH_CLK_SRC_LSI = LL_RCC_DAC1SH_CLKSOURCE_LSI  /*!< LSI selected as DAC1 SH clock source */
} hal_rcc_dac1_sh_clk_src_t;

#if defined(LPTIM1)
/**
  * @brief LPTIM1 Clock Source.
  */
typedef enum
{
  HAL_RCC_LPTIM1_CLK_SRC_PCLK3      = LL_RCC_LPTIM1_CLKSOURCE_PCLK3, /*!< PCLK3 selected as LPTIM1 kernel clock */
  HAL_RCC_LPTIM1_CLK_SRC_HSIK       = LL_RCC_LPTIM1_CLKSOURCE_HSIK,  /*!< HSIK selected as LPTIM1 kernel clock  */
  HAL_RCC_LPTIM1_CLK_SRC_LSE        = LL_RCC_LPTIM1_CLKSOURCE_LSE,   /*!< LSE selected as LPTIM1 kernel clock   */
  HAL_RCC_LPTIM1_CLK_SRC_LSI        = LL_RCC_LPTIM1_CLKSOURCE_LSI,   /*!< LSI selected as LPTIM1 kernel clock   */
} hal_rcc_lptim1_clk_src_t;

#endif /* LPTIM1 */
/**
  * @brief CK48 Clock Source (Used for RNG and USB).
  */
typedef enum
{
  HAL_RCC_CK48_CLK_SRC_PSIDIV3      = LL_RCC_CK48_CLKSOURCE_PSIDIV3, /*!< PSIDIV3 selected as CK48 clock source */
  HAL_RCC_CK48_CLK_SRC_HSIDIV3      = LL_RCC_CK48_CLKSOURCE_HSIDIV3, /*!< HSIDIV3 selected as CK48 clock source */
  HAL_RCC_CK48_CLK_SRC_HSE          = LL_RCC_CK48_CLKSOURCE_HSE,     /*!< HSE selected as CK48 clock source     */
} hal_rcc_ck48_clk_src_t;

#if defined(XSPI1)
/**
  * @brief XSPI1 Clock Source.
  */
typedef enum
{
  HAL_RCC_XSPI1_CLK_SRC_HCLK      = LL_RCC_XSPI1_CLKSOURCE_HCLK, /*!< HCLK selected as XSPI1 kernel clock */
  HAL_RCC_XSPI1_CLK_SRC_PSIK      = LL_RCC_XSPI1_CLKSOURCE_PSIK, /*!< PSIK selected as XSPI1 kernel clock */
  HAL_RCC_XSPI1_CLK_SRC_HSIK      = LL_RCC_XSPI1_CLKSOURCE_HSIK  /*!< HSIK selected as XSPI1 kernel clock */
} hal_rcc_xspi1_clk_src_t;

#endif /* XSPI1 */
#if defined(ETH1)
/**
  * @brief ETH1 REF Clock Source.
  */
typedef enum
{
  HAL_RCC_ETH1REF_CLK_SRC_RMII    = LL_RCC_ETH1REF_CLKSOURCE_RMII, /*!< RMII selected as ETH1REF clock source */
  HAL_RCC_ETH1REF_CLK_SRC_FB      = LL_RCC_ETH1REF_CLKSOURCE_FB    /*!< FB selected as ETH1REF clock source   */
} hal_rcc_eth1ref_clk_src_t;

/**
  * @brief ETH1 PTP Clock Source.
  */
typedef enum
{
  HAL_RCC_ETH1PTP_CLK_SRC_NONE      = LL_RCC_ETH1PTP_CLKSOURCE_NONE, /*!< NONE selected as ETH1PTP clock source */
  HAL_RCC_ETH1PTP_CLK_SRC_HCLK      = LL_RCC_ETH1PTP_CLKSOURCE_HCLK, /*!< HCLK selected as ETH1PTP clock source */
  HAL_RCC_ETH1PTP_CLK_SRC_PSIS      = LL_RCC_ETH1PTP_CLKSOURCE_PSIS, /*!< PSIS selected as ETH1PTP clock source */
  HAL_RCC_ETH1PTP_CLK_SRC_PSIK      = LL_RCC_ETH1PTP_CLKSOURCE_PSIK  /*!< PSIK selected as ETH1PTP clock source */
} hal_rcc_eth1ptp_clk_src_t;

/**
  * @brief ETH1 Clock Source.
  */
typedef enum
{
  HAL_RCC_ETH1_CLK_SRC_NONE  = LL_RCC_ETH1_CLKSOURCE_NONE, /*!< NONE selected as ETH1 clock source */
  HAL_RCC_ETH1_CLK_SRC_PSIS  = LL_RCC_ETH1_CLKSOURCE_PSIS, /*!< PSIS selected as ETH1 clock source */
  HAL_RCC_ETH1_CLK_SRC_PSIK  = LL_RCC_ETH1_CLKSOURCE_PSIK, /*!< PSIK selected as ETH1 clock source */
  HAL_RCC_ETH1_CLK_SRC_HSE   = LL_RCC_ETH1_CLKSOURCE_HSE   /*!< HSE selected as ETH1 clock source  */
} hal_rcc_eth1_clk_src_t;

/**
  * @brief ETH1 Clock Divider.
  */
typedef enum
{
  HAL_RCC_ETH1_PRESCALER1    = LL_RCC_ETH1_PRESCALER_1,     /*!< ETH1 clock is divided by 1 */
  HAL_RCC_ETH1_PRESCALER2    = LL_RCC_ETH1_PRESCALER_2,     /*!< ETH1 clock is divided by 2 */
  HAL_RCC_ETH1_PRESCALER4    = LL_RCC_ETH1_PRESCALER_4,     /*!< ETH1 clock is divided by 4 */
} hal_rcc_eth1_prescaler_t;

#endif /* ETH1 */

/**
  * @brief SYSTICK Clock Source.
  */
typedef enum
{
  HAL_RCC_SYSTICK_CLK_SRC_HCLKDIV8 = LL_RCC_SYSTICK_CLKSOURCE_HCLKDIV8, /*!< HCLK_DIV8 clock used as SYSTICK clock */
  HAL_RCC_SYSTICK_CLK_SRC_LSI = LL_RCC_SYSTICK_CLKSOURCE_LSI,           /*!< LSI clock used as SYSTICK clock       */
  HAL_RCC_SYSTICK_CLK_SRC_LSE = LL_RCC_SYSTICK_CLKSOURCE_LSE            /*!< LSE clock used as SYSTICK clock       */
} hal_rcc_systick_clk_src_t;

/**
  * @brief Privileged access level attribute
  */
typedef enum
{
  HAL_RCC_NPRIV = LL_RCC_ATTR_NPRIV, /*!< Non-privileged access level attribute */
  HAL_RCC_PRIV  = LL_RCC_ATTR_PRIV   /*!< Privileged access level attribute     */
} hal_rcc_priv_attr_t;

/**
  * @brief RCC attributes configuration items
  */
#define HAL_RCC_PRIV_ITEM_ALL              LL_RCC_PRIV_ITEM_ALL           /*!< All RCC resources privilege configuration item  */
/**
  * @}
  */

/**
  * @brief  RCC System, AHB and APB buses clock configuration structure definition.
  */
typedef struct
{
  hal_rcc_hclk_prescaler_t hclk_prescaler; /*!< The HCLK (AHB clock) prescaler. This clock is derived from the system clock
                                           (SYSCLK).                                                */

  hal_rcc_pclk_prescaler_t pclk1_prescaler;  /*!< The PCLK1 (APB1 clock) prescaler.
                                             This clock is derived from the AHB clock (HCLK).       */

  hal_rcc_pclk_prescaler_t pclk2_prescaler;  /*!< The PCLK2 (APB2 clock) prescaler.
                                             This clock is derived from the AHB clock (HCLK).       */

  hal_rcc_pclk_prescaler_t pclk3_prescaler;  /*!< The PCLK3 (APB3 clock) prescaler.
                                            This clock is derived from the AHB clock (HCLK).        */
} hal_rcc_bus_clk_config_t;

/**
  * @brief  RCC PSI configuration structure definition.
  */
typedef struct
{
  hal_rcc_psi_src_t psi_source;        /*!< PSI entry clock source selection  */

  hal_rcc_psi_ref_t psi_ref;           /*!< PSI entry clock source frequency  */

  hal_rcc_psi_out_t psi_out;           /*!< PSI clock frequency output        */

} hal_rcc_psi_config_t;

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup RCC_Exported_Functions HAL RCC Functions
  * @{
  *
  */

/** @defgroup RCC_Exported_Functions_Group1 Oscillators, PSI, bus configurations, RCC and system clock reset functions
  This section provides functions allowing to configure the internal and external oscillators
  (HSE, HSIS, LSE, LSI, HSIDIV3, HSIK, PSIS, PSIDIV3, PSIK, HSIDIV18), CSS, MCO and the System buses clocks
  (SYSCLK, AHB, APB1, APB2 and APB3).

  - Internal/external clock and PSI configuration:

    - HSIS (High-Speed Internal System): 144 MHz factory-trimmed RC used as System clock source.

    - HSIDIV3 (High-Speed Internal Divided by 3): HSI clock divided by 3 (48 MHz) used for USB, RNG
      or as System clock source.

    - HSIK (High-Speed Internal Kernel Clock): Used for peripherals as kernel clock.

    - PSIS (Precise-Speed Internal System): PSIS configurable to several ranges used as System clock source.

    - PSIDIV3 (Precise-Speed Internal Divided by 3): PSI clock divided by 3 can be used for USB or RNG.

    - PSIK (Precise-Speed Internal Kernel Clock): Used for peripherals as kernel clock.

    - LSI (Low-Speed Internal): 32 kHz low consumption RC used as IWDG
    and/or RTC clock source. It can be selected by some peripherals as kernel clock.

    - HSE (High-Speed External): crystal or clock, from 4 to 50 MHz, used directly or
    through the PSI as System clock source. Can be used also optionally as RTC, USB, RNG or FDCAN clock source.

    - LSE (low-speed external): 32.768 kHz oscillator used optionally as RTC clock source. Used
    through the PSI as System clock source. Used for peripherals as kernel clock.

    - HSIDIV18 (High-Speed Internal Divided by 18): HSI clock divided by 18 (8 MHz). Used through the
    PSI as System clock source.


  - System, AHB and APB buses clocks configuration:

    - Several clock sources can be used to drive the System clock (SYSCLK): HSIS, HSI_DIV3
    HSE and PSIS.
    The AHB clock (HCLK) is derived from System clock through configurable
    prescaler and used to clock the CPU, memory and peripherals mapped
    on AHB bus (DMA, GPIO...). APB1 (PCLK1), APB2 (PCLK2) and APB3 (PCLK3) clocks are derived
    from AHB clock through configurable prescalers and used to clock
    the peripherals mapped on these buses.

    - All the peripheral bus clocks are derived from the System clock (SYSCLK) except:

      - RTC: the RTC clock can be derived either from the LSI, LSE or HSE clock
      divided by 2 to 511.
      - USB Host and Device FS and RNG: USB Host and Device FS require a frequency equal to 48 MHz
      to work correctly, while RNG peripherals require a frequency
      equal or lower than to 48 MHz. This clock is derived of the PSIDIV3, HSIDIV3 or HSE
      - IWDG clock which is always the LSI clock.
      - DAC1 sample and hold clock. This clock is derived from LSI or LSE.

    - The maximum frequency of the SYSCLK, HCLK, PCLK1, PCLK2 and PCLK3 is 144 MHz.
    To correctly read data from Flash memory, the number of wait states (latency) must be
    correctly programmed in the FLASH register according to the
    frequency of the CPU clock (HCLK) and the internal voltage range of the device VCORE.

    When changing the CPU frequency, a software sequence must be applied
    in order to tune the number of wait states needed to access the flash memory:
    - FLASH latency must be increased before increasing the HCLK frequency
    - FLASH latency can be decreased only after decreasing the HCLK frequency

    The table below shows the correspondence between wait states and CPU clock frequency.

    Table 1. HCLK clock frequency for devices depending on FLASH latency and voltage range.

    |   Latency       | range 1<br>1.1V-1.2V| range 2<br>1.0V-1.1V| range 3<br>0.9 V-1.0V| range 4<br>0.9V |
    |-----------------|---------------------|---------------------|----------------------|-----------------|
    |0WS(1 CPU cycle) |  0 < HCLK <= 32     |  0 < HCLK <= 30     |    0 < HCLK <= 24    |  0 < HCLK <= 12 |
    |1WS(2 CPU cycles)| 32 < HCLK <= 64     | 30 < HCLK <= 60     |   24 < HCLK <= 48    | 12 < HCLK <= 25 |
    |2WS(3 CPU cycles)| 64 < HCLK <= 96     | 60 < HCLK <= 90     |   48 < HCLK <= 55    |        -        |
    |3WS(4 CPU cycles)| 96 < HCLK <= 128    | 90 < HCLK <= 110    |         -            |        -        |
    |4WS(5 CPU cycles)|128 < HCLK <= 144    |         -           |         -            |        -        |

    The table below shows the correspondence between wait states and CPU clock frequency.

    Table 2. HCLK clock frequency for devices depending on FLASH latency and voltage range.

    |   Latency         | range 1/2/3<br>0.9V-1.2V  | range 4<br>0.9V |
    |-------------------|---------------------------|-----------------|
    |0WS(1 CPU cycle)   |                           |  0 < HCLK <= 8  |
    |1WS(2 CPU cycles)  | WS >= HCLK (MHz) / 10 -1  |  8 < HCLK <= 16 |
    |2WS(3 CPU cycles)  |      Maximum HCLK         | 16 < HCLK <= 25 |
    |3WS(4 CPU cycles)  |      frequency is         |        -        |
    |       ...         |    given by Table 1       |        -        |
    |15WS(16 CPU cycles)|                           |        -        |

  - Reset functions:

    - Reset the RCC clock configuration to the default values (HSI at 48 MHz).
  * @{
  */

/** @defgroup RCC_Exported_Functions_Group1_0 Reset RCC and System clock to default values function
  * @brief    Functions to reset the RCC and system clock to default values.
  * @{
  */

void         HAL_RCC_Reset(void);
hal_status_t HAL_RCC_ResetSystemClock(void);

/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group1_1 RCC oscillators config service
  * @brief    Functions to configure the different oscillators.
  * @note     The following functions configure and activate the different oscillators
  *           Configuration can be done:
  *           - using the atomic functions defined for each oscillators
  *             -# like HAL_RCC_{OSC}_Enable: {OSC} is HSIS/HSIDIV3/HSIK/LSI/LSE/HSE/PSIS/PSIDIV3/PSIK
  *
  *           example: enable HSE oscillator
  *           - using atomic function (footprint optimisation):
  *             @code
  *             HAL_RCC_HSE_Enable(HAL_RCC_HSE_ON);
  *             @endcode
  *
  * @{
  */

/* HSI oscillator configuration and activation    ******************************/
hal_status_t        HAL_RCC_HSI_EnableInStopMode(void);
hal_status_t        HAL_RCC_HSI_DisableInStopMode(void);
hal_rcc_osc_stop_mode_status_t HAL_RCC_HSI_IsEnabledInStopMode(void);
hal_status_t        HAL_RCC_HSIS_Enable(void);
hal_status_t        HAL_RCC_HSIS_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_HSIS_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_HSIS_IsReady(void);
hal_status_t        HAL_RCC_HSIDIV3_Enable(void);
hal_status_t        HAL_RCC_HSIDIV3_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_HSIDIV3_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_HSIDIV3_IsReady(void);
hal_status_t        HAL_RCC_HSIK_Enable(hal_rcc_hsik_div_t divider);
hal_status_t        HAL_RCC_HSIK_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_HSIK_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_HSIK_IsReady(void);
hal_rcc_hsik_div_t HAL_RCC_HSIK_GetDivider(void);

/* LSI oscillator configuration and activation    ******************************/
hal_status_t        HAL_RCC_LSI_Enable(void);
hal_status_t        HAL_RCC_LSI_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_LSI_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_LSI_IsReady(void);

#if defined(HSE_VALUE)
/* HSE oscillator configuration and activation    ******************************/
hal_status_t        HAL_RCC_HSE_Enable(hal_rcc_hse_t mode);
hal_status_t        HAL_RCC_HSE_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_HSE_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_HSE_IsReady(void);

#endif /* HSE_VALUE */
#if defined(LSE_VALUE)
/* LSE oscillator configuration and activation    ******************************/
hal_status_t        HAL_RCC_LSE_Enable(hal_rcc_lse_t mode, hal_rcc_lse_drive_t drive);
hal_status_t        HAL_RCC_LSE_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_LSE_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_LSE_IsReady(void);

#endif /* LSE_VALUE */
/* PSI oscillator configuration and activation    ******************************/
hal_status_t        HAL_RCC_PSI_EnableInStopMode(void);
hal_status_t        HAL_RCC_PSI_DisableInStopMode(void);
hal_rcc_osc_stop_mode_status_t HAL_RCC_PSI_IsEnabledInStopMode(void);
hal_status_t        HAL_RCC_PSIS_Enable(void);
hal_status_t        HAL_RCC_PSIS_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_PSIS_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_PSIS_IsReady(void);
hal_status_t        HAL_RCC_PSIDIV3_Enable(void);
hal_status_t        HAL_RCC_PSIDIV3_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_PSIDIV3_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_PSIDIV3_IsReady(void);
hal_status_t        HAL_RCC_PSIK_Enable(hal_rcc_psik_div_t divider);
hal_status_t        HAL_RCC_PSIK_Disable(void);
hal_rcc_osc_enable_status_t HAL_RCC_PSIK_IsEnabled(void);
hal_rcc_osc_ready_status_t HAL_RCC_PSIK_IsReady(void);
hal_rcc_psik_div_t HAL_RCC_PSIK_GetDivider(void);
hal_status_t        HAL_RCC_PSI_SetConfig(const hal_rcc_psi_config_t *p_config);
void                HAL_RCC_PSI_GetConfig(hal_rcc_psi_config_t *p_config);
/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group1_2 RCC clock config service
  * @brief    Functions to configure the bus prescalers and retrieve bus clock frequencies (SYSCLK, HCLK and PCLKx).
  * @note     Unitary functions can be used to configure independently each bus.
  *
  *           example: all the BUS prescalers are set
  *
  *           - Call the global @ref HAL_RCC_SetBusClockConfig
  *             @code
  *               LL_FLASH_SetLatency(FLASH1, LL_FLASH_LATENCY_4WS);
  *               HAL_RCC_SetSYSCLKSource(HAL_RCC_SYSCLK_SRC_PLLCLK);
  *               hal_rcc_bus_clk_config_t config_bus;
  *               config_bus.hclk_prescaler  = HAL_RCC_HCLK_PRESCALER1;
  *               config_bus.pclk1_prescaler = HAL_RCC_PCLK_PRESCALER1;
  *               config_bus.pclk2_prescaler = HAL_RCC_PCLK_PRESCALER1;
  *               config_bus.pclk3_prescaler = HAL_RCC_PCLK_PRESCALER1;
  *               HAL_RCC_SetBusClockConfig(&config_bus);
  *             @endcode
  * @{
  */

hal_status_t          HAL_RCC_SetSYSCLKSource(hal_rcc_sysclk_src_t source);
hal_rcc_sysclk_src_t  HAL_RCC_GetSYSCLKSource(void);
void                  HAL_RCC_SetHCLKPrescaler(hal_rcc_hclk_prescaler_t prescaler);
void                  HAL_RCC_SetPCLK1Prescaler(hal_rcc_pclk_prescaler_t prescaler);
void                  HAL_RCC_SetPCLK2Prescaler(hal_rcc_pclk_prescaler_t prescaler);
void                  HAL_RCC_SetPCLK3Prescaler(hal_rcc_pclk_prescaler_t prescaler);
hal_rcc_hclk_prescaler_t    HAL_RCC_GetHCLKPrescaler(void);
hal_rcc_pclk_prescaler_t    HAL_RCC_GetPCLK1Prescaler(void);
hal_rcc_pclk_prescaler_t    HAL_RCC_GetPCLK2Prescaler(void);
hal_rcc_pclk_prescaler_t    HAL_RCC_GetPCLK3Prescaler(void);
hal_status_t          HAL_RCC_SetBusClockConfig(const hal_rcc_bus_clk_config_t *p_config);
void                  HAL_RCC_GetBusClockConfig(hal_rcc_bus_clk_config_t *p_config);

uint32_t              HAL_RCC_GetPSIClockFreq(void);
uint32_t              HAL_RCC_GetSYSCLKFreq(void);
uint32_t              HAL_RCC_GetHCLKFreq(void);
uint32_t              HAL_RCC_GetPCLK1Freq(void);
uint32_t              HAL_RCC_GetPCLK2Freq(void);
uint32_t              HAL_RCC_GetPCLK3Freq(void);

/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group1_3 RCC management of Systick external clock source
  * @brief    Functions to Set, Get the Systick external clock source and frequency.
  * @{
  */

void                       HAL_RCC_SetSysTickExternalClkSource(hal_rcc_systick_clk_src_t clk_src);
hal_rcc_systick_clk_src_t  HAL_RCC_GetSysTickExternalClkSource(void);
uint32_t                   HAL_RCC_GetSysTickExternalClkFreq(void);

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group2 Peripheral Clock management on buses
  This subsection provides a set of functions (on AHB1, AHB2, AHB4, APB1, APB2 or APB3 buses) allowing to:

  - Enable or disable the peripherals clock.
  - Reset of peripherals clock.
  - Enable or disable the peripherals clock in low power mode.
  * @{
  */

/** @defgroup RCC_Bus_Clock_Enable_Disable Bus Clock Enable Disable
  * @brief  Enable or disable the Bus peripheral clocks.
  * @{
  */

/**
  * @details This function enables the AHB1 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB1_EnableBusClock(void)
{
  LL_AHB1_EnableBusClock();
}

/**
  * @details This function checks if AHB1 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED AHB1 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED AHB1 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_AHB1_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_IsEnabledBusClock();
}

/**
  * @details This function disables the AHB1 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB1_DisableBusClock(void)
{
  LL_AHB1_DisableBusClock();
}

/**
  * @details This function enables the AHB2 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB2_EnableBusClock(void)
{
  LL_AHB2_EnableBusClock();
}

/**
  * @details This function checks if AHB2 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED AHB2 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED AHB2 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_AHB2_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_IsEnabledBusClock();
}

/**
  * @details This function disables the AHB2 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB2_DisableBusClock(void)
{
  LL_AHB2_DisableBusClock();
}
#if defined(AHB4PERIPH_BASE)

/**
  * @details This function enables the AHB4 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB4_EnableBusClock(void)
{
  LL_AHB4_EnableBusClock();
}

/**
  * @details This function checks if AHB4 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED AHB4 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED AHB4 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_AHB4_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB4_IsEnabledBusClock();
}

/**
  * @details This function disables the AHB4 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_AHB4_DisableBusClock(void)
{
  LL_AHB4_DisableBusClock();
}
#endif /* AHB4PERIPH_BASE */
/**
  * @details This function enables the APB1 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB1_EnableBusClock(void)
{
  LL_APB1_EnableBusClock();
}

/**
  * @details This function checks if APB1 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED APB1 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED APB1 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_APB1_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_IsEnabledBusClock();
}

/**
  * @details This function disables the APB1 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB1_DisableBusClock(void)
{
  LL_APB1_DisableBusClock();
}

/**
  * @details This function enables the APB2 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB2_EnableBusClock(void)
{
  LL_APB2_EnableBusClock();
}

/**
  * @details This function checks if APB2 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED APB2 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED APB2 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_APB2_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_IsEnabledBusClock();
}

/**
  * @details This function disables the APB2 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB2_DisableBusClock(void)
{
  LL_APB2_DisableBusClock();
}

/**
  * @details This function enables the APB3 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB3_EnableBusClock(void)
{
  LL_APB3_EnableBusClock();
}

/**
  * @details This function checks if APB3 bus clock is enabled.
  * @retval  HAL_RCC_CLK_DISABLED APB3 bus clock is disabled
  * @retval  HAL_RCC_CLK_ENABLED APB3 bus clock is enabled
  */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_APB3_IsEnabledBusClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB3_IsEnabledBusClock();
}

/**
  * @details This function disables the APB3 Bus clock.
  */
__STATIC_INLINE void HAL_RCC_APB3_DisableBusClock(void)
{
  LL_APB3_DisableBusClock();
}
/**
  * @}
  */ /* RCC_Bus_Clock_Enable_Disable */

/** @defgroup RCC_AHB1_Peripheral_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it excepted for FLASH and SRAMx.
  * @{
  */
/**
  * @details This function enables the LPDMA1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPDMA1_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function enables the LPDMA2 clock.
  */
__STATIC_INLINE void HAL_RCC_LPDMA2_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function enables the FLASH clock.
  */
__STATIC_INLINE void HAL_RCC_FLASH_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_FLASH);
}

/**
  * @details This function enables the CRC clock.
  */
__STATIC_INLINE void HAL_RCC_CRC_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function enables the CORDIC clock.
  */
__STATIC_INLINE void HAL_RCC_CORDIC_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function enables the RAMCFG clock.
  */
__STATIC_INLINE void HAL_RCC_RAMCFG_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

#if defined(ETH1)
/**
  * @details This function enables the ETH1CK clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1CK_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function enables the ETH1 clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function enables the ETH1TX clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1TX_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function enables the ETH1RX clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1RX_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

#endif /* ETH1 */
/**
  * @details This function enables the SRAM1 clock.
  */
__STATIC_FORCEINLINE void HAL_RCC_SRAM1_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function enables the SRAM1 clock.
  */
__STATIC_FORCEINLINE void HAL_RCC_SRAM2_EnableClock(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @details This function disables the LPDMA1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPDMA1_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function disables the LPDMA2 clock.
  */
__STATIC_INLINE void HAL_RCC_LPDMA2_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function disables the FLASH clock.
  */
__STATIC_INLINE void HAL_RCC_FLASH_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_FLASH);
}

/**
  * @details This function disables the CRC clock.
  */
__STATIC_INLINE void HAL_RCC_CRC_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function disables the CORDIC clock.
  */
__STATIC_INLINE void HAL_RCC_CORDIC_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function disables the RAMCFG clock.
  */
__STATIC_INLINE void HAL_RCC_RAMCFG_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

#if defined(ETH1)
/**
  * @details This function disables the ETH1CK clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1CK_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function disables the ETH1 clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function disables the ETH1TX clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1TX_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function disables the ETH1RX clock.
  */
__STATIC_INLINE void HAL_RCC_ETH1RX_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

#endif /* ETH1 */
/**
  * @details This function disables the SRAM1 clock.
  */
__STATIC_FORCEINLINE void HAL_RCC_SRAM1_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function disables the SRAM2 clock.
  */
__STATIC_FORCEINLINE void HAL_RCC_SRAM2_DisableClock(void)
{
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_SRAM2);
}
/**
  * @}
  */

/** @defgroup RCC_AHB2_Peripheral_Clock_Enable_Disable AHB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function enables the GPIOA clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOA_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function enables the GPIOB clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOB_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function enables the GPIOC clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOC_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function enables the GPIOD clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOD_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function enables the GPIOE clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOE_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function enables the GPIOF clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOF_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function enables the GPIOG clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOG_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function enables the GPIOH clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOH_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function enables the ADC12 clock.
  */
__STATIC_INLINE void HAL_RCC_ADC12_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function enables the DAC1 clock.
  */
__STATIC_INLINE void HAL_RCC_DAC1_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function enables the AES clock.
  */
__STATIC_INLINE void HAL_RCC_AES_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_AES);
}

#endif /* AES */
/**
  * @details This function enables the HASH clock.
  */
__STATIC_INLINE void HAL_RCC_HASH_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function enables the RNG clock.
  */
__STATIC_INLINE void HAL_RCC_RNG_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function enables the PKA clock.
  */
__STATIC_INLINE void HAL_RCC_PKA_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function enables the SAES clock.
  */
__STATIC_INLINE void HAL_RCC_SAES_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function enables the CCB clock.
  */
__STATIC_INLINE void HAL_RCC_CCB_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */
#if defined(ADC3)
/**
  * @details This function enables the ADC3 clock.
  */
__STATIC_INLINE void HAL_RCC_ADC3_EnableClock(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @details This function disables the GPIOA clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOA_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function disables the GPIOB clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOB_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function disables the GPIOC clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOC_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function disables the GPIOD clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOD_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function disables the GPIOE clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOE_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function disables the GPIOF clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOF_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function disables the GPIOG clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOG_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function disables the GPIOH clock.
  */
__STATIC_INLINE void HAL_RCC_GPIOH_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function disables the ADC12 clock.
  */
__STATIC_INLINE void HAL_RCC_ADC12_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function disables the DAC1 clock.
  */
__STATIC_INLINE void HAL_RCC_DAC1_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function disables the AES clock.
  */
__STATIC_INLINE void HAL_RCC_AES_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_AES);
}

#endif /* AES */
/**
  * @details This function disables the HASH clock.
  */
__STATIC_INLINE void HAL_RCC_HASH_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function disables the RNG clock.
  */
__STATIC_INLINE void HAL_RCC_RNG_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function disables the PKA clock.
  */
__STATIC_INLINE void HAL_RCC_PKA_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function disables the SAES clock.
  */
__STATIC_INLINE void HAL_RCC_SAES_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function disables the CCB clock.
  */
__STATIC_INLINE void HAL_RCC_CCB_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */
#if defined(ADC3)
/**
  * @details This function disables the ADC3 clock.
  */
__STATIC_INLINE void HAL_RCC_ADC3_DisableClock(void)
{
  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @}
  */

#if defined(XSPI1)
/** @defgroup RCC_AHB4_Peripheral_Clock_Enable_Disable AHB4 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB4 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#if defined(XSPI1)
/**
  * @details This function enables the XSPI1 clock.
  */
__STATIC_INLINE void HAL_RCC_XSPI1_EnableClock(void)
{
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_XSPI1);
}

#endif /* XSPI1 */
#if defined(XSPI1)
/**
  * @details This function disables the XSPI1 clock.
  */
__STATIC_INLINE void HAL_RCC_XSPI1_DisableClock(void)
{
  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_XSPI1);
}

#endif /* XSPI1 */
/**
  * @}
  */

#endif /* XSPI1 || XSPI2 || FMC_BASE*/
/** @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

/**
  * @details This function enables the TIM2 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM2_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
}

#if defined(TIM3)
/**
  * @details This function enables the TIM3 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM3_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function enables the TIM4 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM4_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function enables the TIM5 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM5_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function enables the TIM6 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM6_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function enables the TIM7 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM7_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function enables the TIM12 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM12_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM12);
}

/**
  * @details This function enables the WWDG clock.
  */
__STATIC_INLINE void HAL_RCC_WWDG_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
}

#if defined(OPAMP1)
/**
  * @details This function enables the OPAMP1 clock.
  */
__STATIC_INLINE void HAL_RCC_OPAMP1_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function enables the SPI2 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI2_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
}

#if defined(SPI3)
/**
  * @details This function enables the SPI3 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI3_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function enables the USART2 clock.
  */
__STATIC_INLINE void HAL_RCC_USART2_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
}

#if defined(USART3)
/**
  * @details This function enables the USART3 clock.
  */
__STATIC_INLINE void HAL_RCC_USART3_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function enables the UART4 clock.
  */
__STATIC_INLINE void HAL_RCC_UART4_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function enables the UART5 clock.
  */
__STATIC_INLINE void HAL_RCC_UART5_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function enables the I2C1 clock.
  */
__STATIC_INLINE void HAL_RCC_I2C1_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function enables the I2C2 clock.
  */
__STATIC_INLINE void HAL_RCC_I2C2_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function enables the I3C1 clock.
  */
__STATIC_INLINE void HAL_RCC_I3C1_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function enables the CRS clock.
  */
__STATIC_INLINE void HAL_RCC_CRS_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);
}

#if defined(USART6)
/**
  * @details This function enables the USART6 clock.
  */
__STATIC_INLINE void HAL_RCC_USART6_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function enables the UART7 clock.
  */
__STATIC_INLINE void HAL_RCC_UART7_EnableClock(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function enables the COMP12 clock.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_COMP12_EnableClock(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)
/**
  * @details This function enables the FDCAN clock.
  * @note    The FDCAN clock is common for all FDCAN instances
  */
__STATIC_INLINE void HAL_RCC_FDCAN_EnableClock(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @details This function disables the TIM2 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM2_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
}

#if defined(TIM3)
/**
  * @details This function disables the TIM3 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM3_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function disables the TIM4 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM4_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function disables the TIM5 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM5_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function disables the TIM6 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM6_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function disables the TIM7 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM7_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function disables the TIM12 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM12_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM12);
}

#if defined(OPAMP1)
/**
  * @details This function disables the OPAMP1 clock.
  */
__STATIC_INLINE void HAL_RCC_OPAMP1_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function disables the SPI2 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI2_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
}

#if defined(SPI3)
/**
  * @details This function disables the SPI3 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI3_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function disables the USART2 clock.
  */
__STATIC_INLINE void HAL_RCC_USART2_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
}

#if defined(USART3)
/**
  * @details This function disables the USART3 clock.
  */
__STATIC_INLINE void HAL_RCC_USART3_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function disables the UART4 clock.
  */
__STATIC_INLINE void HAL_RCC_UART4_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function disables the UART5 clock.
  */
__STATIC_INLINE void HAL_RCC_UART5_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function disables the I2C1 clock.
  */
__STATIC_INLINE void HAL_RCC_I2C1_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function disables the I2C2 clock.
  */
__STATIC_INLINE void HAL_RCC_I2C2_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function disables the I3C1 clock.
  */
__STATIC_INLINE void HAL_RCC_I3C1_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function disables the CRS clock.
  */
__STATIC_INLINE void HAL_RCC_CRS_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_CRS);
}

#if defined(USART6)
/**
  * @details This function disables the USART6 clock.
  */
__STATIC_INLINE void HAL_RCC_USART6_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function disables the UART7 clock.
  */
__STATIC_INLINE void HAL_RCC_UART7_DisableClock(void)
{
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function disables the COMP12 clock.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_COMP12_DisableClock(void)
{
  LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)
/**
  * @details This function disables the FDCAN clock.
  * @note    The FDCAN clock is common for all FDCAN instances
  */
__STATIC_INLINE void HAL_RCC_FDCAN_DisableClock(void)
{
  LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup RCC_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function enables the TIM1 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM1_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function enables the SPI1 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI1_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function enables the TIM8 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM8_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
}


/**
  * @details This function enables the USART1 clock.
  */
__STATIC_INLINE void HAL_RCC_USART1_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function enables the TIM15 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM15_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function enables the TIM16 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM16_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function enables the TIM17 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM17_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function enables the USB clock.
  */
__STATIC_INLINE void HAL_RCC_USB_EnableClock(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @details This function disables the TIM1 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM1_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function disables the SPI1 clock.
  */
__STATIC_INLINE void HAL_RCC_SPI1_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function disables the TIM8 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM8_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM8);
}

/**
  * @details This function disables the USART1 clock.
  */
__STATIC_INLINE void HAL_RCC_USART1_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function disables the TIM15 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM15_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function disables the TIM16 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM16_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function disables the TIM17 clock.
  */
__STATIC_INLINE void HAL_RCC_TIM17_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function disables the USB clock.
  */
__STATIC_INLINE void HAL_RCC_USB_DisableClock(void)
{
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @}
  */

/** @defgroup RCC_APB3_Clock_Enable_Disable APB3 Peripheral Clock Enable Disable
  * @brief  Enable or disable the APB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function enables the SBS clock.
  */
__STATIC_INLINE void HAL_RCC_SBS_EnableClock(void)
{
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function enables the LPUART1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPUART1_EnableClock(void)
{
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LPUART1);
}
#if defined(LPTIM1)
/**
  * @details This function enables the LPTIM1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPTIM1_EnableClock(void)
{
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @details This function enables the RTCAPB clock.
  */
__STATIC_INLINE void HAL_RCC_RTCAPB_EnableClock(void)
{
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @details This function disables the SBS clock.
  */
__STATIC_INLINE void HAL_RCC_SBS_DisableClock(void)
{
  LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function disables the LPUART1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPUART1_DisableClock(void)
{
  LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_LPUART1);
}
#if defined(LPTIM1)
/**
  * @details This function disables the LPTIM1 clock.
  */
__STATIC_INLINE void HAL_RCC_LPTIM1_DisableClock(void)
{
  LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @details This function disables the RTCAPB clock.
  */
__STATIC_INLINE void HAL_RCC_RTCAPB_DisableClock(void)
{
  LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_RTCAPB);
}
/**
  * @}
  */

/** @defgroup RCC_AHB1_Peripheral_Clock_Enable_Status AHB1 Peripheral Clock Enabled Status
  * @brief  Check whether the AHB1 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function checks if the LPDMA1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED LPDMA1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  LPDMA1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_LPDMA1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function checks if the LPDMA2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED LPDMA2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  LPDMA2 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_LPDMA2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function checks if the FLASH clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED FLASH clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  FLASH clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_FLASH_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_FLASH);
}

/**
  * @details This function checks if the CRC clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED CRC clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  CRC clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_CRC_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function checks if the CORDIC clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED CORDIC clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  CORDIC clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_CORDIC_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function checks if the RAMCFG clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED RAMCFG clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  RAMCFG clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_RAMCFG_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

#if defined(ETH1)
/**
  * @details This function checks if the ETH1CK clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ETH1CK clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ETH1CK clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ETH1CK_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function checks if the ETH1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ETH1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ETH1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ETH1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function checks if the ETH1TX clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ETH1TX clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ETH1TX clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ETH1TX_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function checks if the ETH1RX clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ETH1RX clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ETH1RX clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ETH1RX_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

#endif /* ETH1 */
/**
  * @details This function checks if the SRAM1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SRAM1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SRAM1 clock is enabled
 */
__STATIC_FORCEINLINE hal_rcc_clk_status_t HAL_RCC_SRAM1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function checks if the SRAM2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SRAM2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SRAM2 clock is enabled
 */
__STATIC_FORCEINLINE hal_rcc_clk_status_t HAL_RCC_SRAM2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @}
  */

/** @defgroup RCC_AHB2_Peripheral_Clock_Enable_Status AHB2 Peripheral Clock Enabled Status
  * @brief  Check whether the AHB2 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function checks if the GPIOA clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOA clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOA clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOA_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function checks if the GPIOB clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOB clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOB clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOB_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function checks if the GPIOC clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOC clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOC clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOC_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function checks if the GPIOD clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOD clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOD clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOD_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function checks if the GPIOE clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOE clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOE clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOE_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function checks if the GPIOF clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOF clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOF clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOF_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function checks if the GPIOG clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOG clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOG clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOG_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function checks if the GPIOH clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED GPIOH clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  GPIOH clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_GPIOH_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function checks if the ADC12 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ADC12 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ADC12 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ADC12_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function checks if the DAC1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED DAC1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  DAC1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_DAC1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function checks if the AES clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED AES clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  AES clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_AES_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_AES);
}

#endif /* AES */
/**
  * @details This function checks if the HASH clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED HASH clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  HASH clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_HASH_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function checks if the RNG clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED RNG clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  RNG clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_RNG_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function checks if the PKA clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED PKA clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  PKA clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_PKA_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function checks if the SAES clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SAES clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SAES clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_SAES_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function checks if the CCB clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED CCB clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  CCB clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_CCB_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */
#if defined(ADC3)
/**
  * @details This function checks if the ADC3 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED ADC3 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  ADC3 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_ADC3_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @}
  */

#if defined(XSPI1)
/** @defgroup RCC_AHB4_Peripheral_Clock_Enable_Status AHB4 Peripheral Clock Enabled Status
  * @brief  Check whether the AHB4 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#if defined(XSPI1)
/**
  * @details This function checks if the XSPI1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED XSPI1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  XSPI1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_XSPI1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_XSPI1);
}
/**
  * @}
  */

#endif /* XSPI1 */
#endif /* XSPI1 || XSPI2 || FMC_BASE*/
/** @defgroup RCC_APB1_Peripheral_Clock_Enable_Status APB1 Peripheral Clock Enabled Status
  * @brief  Check whether the APB1 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function checks if the TIM2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM2 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM2);
}

#if defined(TIM3)
/**
  * @details This function checks if the TIM3 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM3 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM3 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM3_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function checks if the TIM4 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM4 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM4 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM4_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function checks if the TIM5 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM5 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM5 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM5_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function checks if the TIM6 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM6 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM6 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM6_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function checks if the TIM7 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM7 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM7 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM7_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function checks if the TIM12 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM12 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM12 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM12_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM12);
}

/**
  * @details This function checks if the WWDG clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED WWDG clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  WWDG clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_WWDG_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_WWDG);
}

#if defined(OPAMP1)
/**
  * @details This function checks if the OPAMP1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED OPAMP1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  OPAMP1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_OPAMP1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function checks if the SPI2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SPI2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SPI2 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_SPI2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_SPI2);
}
#if defined(SPI3)
/**
  * @details This function checks if the SPI3 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SPI3 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SPI3 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_SPI3_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function checks if the USART2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED USART2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  USART2 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_USART2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_USART2);
}
#if defined(USART3)
/**
  * @details This function checks if the USART3 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED USART3 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  USART3 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_USART3_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function checks if the UART4 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED UART4 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  UART4 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_UART4_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function checks if the UART5 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED UART5 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  UART5 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_UART5_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function checks if the I2C1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED I2C1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  I2C1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_I2C1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function checks if the I2C2 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED I2C2 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  I2C2 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_I2C2_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function checks if the I3C1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED I3C1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  I3C1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_I3C1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function checks if the CRS clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED CRS clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  CRS clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_CRS_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_CRS);
}

#if defined(USART6)
/**
  * @details This function checks if the USART6 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED USART6 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  USART6 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_USART6_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function checks if the UART7 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED UART7 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  UART7 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_UART7_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function checks if the COMP12 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED COMP12 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  COMP12 clock is enabled
  * @note  COMP2 not defined in all devices.
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_COMP12_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP2_IsEnabledClock(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)

/**
  * @details This function checks if the FDCAN clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED FDCAN clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  FDCAN clock is enabled
  * @note The FDCAN clock is common for all FDCAN instances
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_FDCAN_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB1_GRP2_IsEnabledClock(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup RCC_APB2_Peripheral_Clock_Enable_Status APB2 Peripheral Clock Enabled Status
  * @brief  Check whether the APB2 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */

/**
  * @details This function checks if the TIM1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function checks if the SPI1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SPI1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SPI1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_SPI1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function checks if the TIM8 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM8 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM8 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM8_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM8);
}

/**
  * @details This function checks if the USART1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED USART1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  USART1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_USART1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function checks if the TIM15 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM15 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM15 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM15_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function checks if the TIM16 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM16 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM16 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM16_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function checks if the TIM17 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED TIM17 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  TIM17 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_TIM17_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function checks if the USB clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED USB clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  USB clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_USB_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @}
  */

/** @defgroup RCC_APB3_Peripheral_Clock_Enable_Status APB3 Peripheral Clock Enabled Status
  * @brief  Check whether the APB3 peripheral clock is enabled or not.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/**
  * @details This function checks if the SBS clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED SBS clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  SBS clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_SBS_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB3_GRP1_IsEnabledClock(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function checks if the LPUART1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED LPUART1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  LPUART1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_LPUART1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB3_GRP1_IsEnabledClock(LL_APB3_GRP1_PERIPH_LPUART1);
}
#if defined(LPTIM1)
/**
  * @details This function checks if the LPTIM1 clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED LPTIM1 clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  LPTIM1 clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_LPTIM1_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB3_GRP1_IsEnabledClock(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @details This function checks if the RTCAPB clock is enabled.
  * @retval HAL_RCC_CLK_DISABLED RTCAPB clock is disabled
  * @retval HAL_RCC_CLK_ENABLED  RTCAPB clock is enabled
 */
__STATIC_INLINE hal_rcc_clk_status_t HAL_RCC_RTCAPB_IsEnabledClock(void)
{
  return (hal_rcc_clk_status_t)LL_APB3_GRP1_IsEnabledClock(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @}
  */

/** @defgroup RCC_AHB1_Reset AHB1 PeripheralReset
  * @brief  AHB1 peripheral reset.
  * @{
  */
/**
  * @details This function resets the LPDMA1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_LPDMA1_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_LPDMA1);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function resets the LPDMA2 peripheral.
  */
__STATIC_INLINE void HAL_RCC_LPDMA2_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_LPDMA2);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function resets the CRC peripheral.
  */
__STATIC_INLINE void HAL_RCC_CRC_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_CRC);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function resets the CORDIC peripheral.
  */
__STATIC_INLINE void HAL_RCC_CORDIC_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_CORDIC);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function resets the RAMCFG peripheral.
  */
__STATIC_INLINE void HAL_RCC_RAMCFG_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_RAMCFG);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

#if defined(ETH1)
/**
  * @details This function resets the ETH1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_ETH1_Reset(void)
{
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_ETH1);
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_ETH1);
}

#endif /* ETH1 */
/**
  * @}
  */

/** @defgroup RCC_AHB2_Reset AHB2 Peripheral Reset
  * @brief  AHB2 peripheral reset.
  * @{
  */
/**
  * @details This function resets the GPIOA peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOA_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function resets the GPIOB peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOB_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function resets the GPIOC peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOC_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function resets the GPIOD peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOD_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOD);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function resets the GPIOE peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOE_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOE);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function resets the GPIOF peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOF_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function resets the GPIOG peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOG_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOG);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function resets the GPIOH peripheral.
  */
__STATIC_INLINE void HAL_RCC_GPIOH_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_GPIOH);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function resets the ADC12 peripheral.
  */
__STATIC_INLINE void HAL_RCC_ADC12_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function resets the DAC1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_DAC1_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_DAC1);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function resets the AES peripheral.
  */
__STATIC_INLINE void HAL_RCC_AES_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_AES);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_AES);
}

#endif  /* AES */
/**
  * @details This function resets the HASH peripheral.
  */
__STATIC_INLINE void HAL_RCC_HASH_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_HASH);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function resets the RNG peripheral.
  */
__STATIC_INLINE void HAL_RCC_RNG_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_RNG);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function resets the PKA peripheral.
  */
__STATIC_INLINE void HAL_RCC_PKA_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_PKA);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function resets the SAES peripheral.
  */
__STATIC_INLINE void HAL_RCC_SAES_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_SAES);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function resets the CCB peripheral.
  */
__STATIC_INLINE void HAL_RCC_CCB_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_CCB);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */

#if defined(ADC3)
/**
  * @details This function resets the ADC3 peripheral.
  */
__STATIC_INLINE void HAL_RCC_ADC3_Reset(void)
{
  LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC3);
  LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @}
  */

#if defined(XSPI1)
/** @defgroup RCC_AHB4_Reset AHB4 Peripheral Reset
  * @brief  AHB4 peripheral reset.
  * @{
  */
#if defined(XSPI1)
/**
  * @details This function resets the XSPI1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_XSPI1_Reset(void)
{
  LL_AHB4_GRP1_ForceReset(LL_AHB4_GRP1_PERIPH_XSPI1);
  LL_AHB4_GRP1_ReleaseReset(LL_AHB4_GRP1_PERIPH_XSPI1);
}
/**
  * @}
  */

#endif /* XSPI1 */
#endif /* XSPI1 || XSPI2 || FMC_BASE*/
/** @defgroup RCC_APB1_Reset APB1 Peripheral Reset
  * @brief  APB1 peripheral reset.
  * @{
  */
/**
  * @details This function resets the TIM2 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM2_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
}

#if defined(TIM3)
/**
  * @details This function resets the TIM3 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM3_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function resets the TIM4 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM4_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM4);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function resets the TIM5 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM5_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM5);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function resets the TIM6 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM6_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM6);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function resets the TIM7 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM7_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM7);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function resets the TIM12 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM12_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM12);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM12);
}

#if defined(OPAMP1)
/**
  * @details This function resets the OPAMP1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_OPAMP1_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_OPAMP1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function resets the SPI2 peripheral.
  */
__STATIC_INLINE void HAL_RCC_SPI2_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_SPI2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_SPI2);
}
#if defined(SPI3)
/**
  * @details This function resets the SPI3 peripheral.
  */
__STATIC_INLINE void HAL_RCC_SPI3_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_SPI3);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function resets the USART2 peripheral.
  */
__STATIC_INLINE void HAL_RCC_USART2_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART2);
}
#if defined(USART3)
/**
  * @details This function resets the USART3 peripheral.
  */
__STATIC_INLINE void HAL_RCC_USART3_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART3);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function resets the UART4 peripheral.
  */
__STATIC_INLINE void HAL_RCC_UART4_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_UART4);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function resets the UART5 peripheral.
  */
__STATIC_INLINE void HAL_RCC_UART5_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_UART5);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function resets the I2C1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_I2C1_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function resets the I2C2 peripheral.
  */
__STATIC_INLINE void HAL_RCC_I2C2_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function resets the I3C1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_I3C1_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I3C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function resets the CRS peripheral.
  */
__STATIC_INLINE void HAL_RCC_CRS_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_CRS);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_CRS);
}

#if defined(USART6)
/**
  * @details This function resets the USART6 peripheral.
  */
__STATIC_INLINE void HAL_RCC_USART6_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART6);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function resets the UART7 peripheral.
  */
__STATIC_INLINE void HAL_RCC_UART7_Reset(void)
{
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_UART7);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function resets the COMP12 peripheral.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_COMP12_Reset(void)
{
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_COMP12);
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)
/**
  * @details This function resets the FDCAN peripheral.
  * @note    The FDCAN clock is common for all FDCAN instances
  */
__STATIC_INLINE void HAL_RCC_FDCAN_Reset(void)
{
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_FDCAN);
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup RCC_APB2_Reset APB2 Peripheral Reset
  * @brief  APB2 peripheral reset.
  * @{
  */
/**
  * @details This function resets the TIM1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM1_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM1);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function resets the SPI1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_SPI1_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SPI1);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function resets the TIM8 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM8_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM8);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM8);
}

/**
  * @details This function resets the USART1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_USART1_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_USART1);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function resets the TIM15 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM15_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function resets the TIM16 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM16_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM16);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function resets the TIM17 peripheral.
  */
__STATIC_INLINE void HAL_RCC_TIM17_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM17);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function resets the USB peripheral.
  */
__STATIC_INLINE void HAL_RCC_USB_Reset(void)
{
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_USB);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @}
  */

/** @defgroup RCC_APB3_Reset APB3 Peripheral Reset
  * @brief  APB3 peripheral reset.
  * @{
  */
/**
  * @details This function resets the SBS peripheral.
  */
__STATIC_INLINE void HAL_RCC_SBS_Reset(void)
{
  LL_APB3_GRP1_ForceReset(LL_APB3_GRP1_PERIPH_SBS);
  LL_APB3_GRP1_ReleaseReset(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function resets the LPUART1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_LPUART1_Reset(void)
{
  LL_APB3_GRP1_ForceReset(LL_APB3_GRP1_PERIPH_LPUART1);
  LL_APB3_GRP1_ReleaseReset(LL_APB3_GRP1_PERIPH_LPUART1);
}

#if defined(LPTIM1)
/**
  * @details This function resets the LPTIM1 peripheral.
  */
__STATIC_INLINE void HAL_RCC_LPTIM1_Reset(void)
{
  LL_APB3_GRP1_ForceReset(LL_APB3_GRP1_PERIPH_LPTIM1);
  LL_APB3_GRP1_ReleaseReset(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @}
  */

/** @defgroup RCC_AHB1_Peripheral_Clock_Sleep_Enable_Disable AHB1 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock during Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/**
  * @details This function enables the LPDMA1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPDMA1_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function enables the LPDMA2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPDMA2_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function enables the FLASH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_FLASH_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_FLASH);
}

/**
  * @details This function enables the CRC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CRC_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function enables the CORDIC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CORDIC_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function enables the RAMCFG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RAMCFG_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

#if defined(ETH1)
/**
  * @details This function enables the ETH1CK clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1CK_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function enables the ETH1CK clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1CK_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function enables the ETH1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function enables the ETH1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function enables the ETH1TX clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1TX_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function enables the ETH1TX clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1TX_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function enables the ETH1RX clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1RX_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

/**
  * @details This function enables the ETH1RX clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1RX_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

#endif /* ETH1 */
/**
  * @details This function enables the ICACHE1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ICACHE1_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_ICACHE1);
}

/**
  * @details This function enables the SRAM2 clock in sleep mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM2_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @details This function enables the SRAM2 clock in stop mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM2_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @details This function enables the SRAM1 clock in sleep mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM1_EnableClockInSleepMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function enables the SRAM1 clock in stop mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM1_EnableClockInStopMode(void)
{
  LL_AHB1_GRP1_EnableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function disables the LPDMA1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPDMA1_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_LPDMA1);
}

#if defined(LPDMA2)
/**
  * @details This function disables the LPDMA2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPDMA2_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_LPDMA2);
}

#endif /* LPDMA2 */
/**
  * @details This function disables the FLASH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_FLASH_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_FLASH);
}

/**
  * @details This function disables the CRC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CRC_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
  * @details This function disables the CORDIC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CORDIC_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/**
  * @details This function disables the RAMCFG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RAMCFG_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_RAMCFG);
}

/**
  * @details This function disables the ICACHE1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ICACHE1_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ICACHE1);
}

#if defined(ETH1)
/**
  * @details This function disables the ETH1CK clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1CK_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function disables the ETH1CK clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1CK_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1CK);
}

/**
  * @details This function disables the ETH1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function disables the ETH1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1);
}

/**
  * @details This function disables the ETH1TX clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1TX_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function disables the ETH1TX clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1TX_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1TX);
}

/**
  * @details This function disables the ETH1RX clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1RX_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

/**
  * @details This function disables the ETH1RX clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ETH1RX_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_ETH1RX);
}

#endif /* ETH1 */
/**
  * @details This function disables the SRAM2 clock in sleep mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM2_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @details This function disables the SRAM2 clock in stop mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM2_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM2);
}

/**
  * @details This function disables the SRAM1 clock in sleep mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM1_DisableClockInSleepMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @details This function disables the SRAM1 clock in stop mode.
  */
__STATIC_FORCEINLINE void HAL_RCC_LP_SRAM1_DisableClockInStopMode(void)
{
  LL_AHB1_GRP1_DisableClockLowPower(LL_AHB1_GRP1_PERIPH_SRAM1);
}

/**
  * @}
  */

/** @defgroup RCC_AHB2_Peripheral_Clock_Sleep_Enable_Disable AHB2 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the AHB2 peripheral clock during Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/**
  * @details This function enables the GPIOA clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOA_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function enables the GPIOB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOB_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function enables the GPIOC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOC_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function enables the GPIOD clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOD_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function enables the GPIOE clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOE_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function enables the GPIOF clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOF_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function enables the GPIOG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOG_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function enables the GPIOH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOH_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function enables the ADC12 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ADC12_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function enables the DAC1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_DAC1_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_DAC1);
}

/**
  * @details This function enables the DAC1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_DAC1_EnableClockInStopMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function enables the AES clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_AES_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_AES);
}

#endif /* AES */
/**
  * @details This function enables the HASH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_HASH_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function enables the RNG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RNG_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function enables the PKA clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_PKA_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function enables the SAES clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SAES_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function enables the CCB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CCB_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */

#if defined(ADC3)
/**
  * @details This function enables the ADC3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ADC3_EnableClockInSleepMode(void)
{
  LL_AHB2_GRP1_EnableClockLowPower(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @details This function disables the GPIOA clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOA_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/**
  * @details This function disables the GPIOB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOB_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOB);
}

/**
  * @details This function disables the GPIOC clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOC_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOC);
}

/**
  * @details This function disables the GPIOD clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOD_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOD);
}

/**
  * @details This function disables the GPIOE clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOE_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOE);
}

#if defined(GPIOF)
/**
  * @details This function disables the GPIOF clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOF_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOF);
}

#endif /* GPIOF */
#if defined(GPIOG)
/**
  * @details This function disables the GPIOG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOG_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOG);
}

#endif /* GPIOG */
/**
  * @details This function disables the GPIOH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_GPIOH_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_GPIOH);
}
/**
  * @details This function disables the ADC12 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ADC12_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_ADC12);
}

/**
  * @details This function disables the DAC1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_DAC1_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_DAC1);
}

/**
  * @details This function disables the DAC1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_DAC1_DisableClockInStopMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_DAC1);
}

#if defined(AES)
/**
  * @details This function disables the AES clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_AES_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_AES);
}

#endif /* AES */
/**
  * @details This function disables the HASH clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_HASH_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_HASH);
}

/**
  * @details This function disables the RNG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RNG_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_RNG);
}

#if defined(PKA)
/**
  * @details This function disables the PKA clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_PKA_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_PKA);
}

#endif /* PKA */
#if defined(SAES)
/**
  * @details This function disables the SAES clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SAES_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_SAES);
}

#endif /* SAES */
#if defined(CCB)
/**
  * @details This function disables the CCB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CCB_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_CCB);
}

#endif /* CCB */
#if defined(ADC3)
/**
  * @details This function disables the ADC3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_ADC3_DisableClockInSleepMode(void)
{
  LL_AHB2_GRP1_DisableClockLowPower(LL_AHB2_GRP1_PERIPH_ADC3);
}

#endif /* ADC3 */
/**
  * @}
  */

#if defined(XSPI1)
/** @defgroup RCC_AHB4_Peripheral_Clock_Sleep_Enable_Disable AHB4 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the AHB4 peripheral clock during Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#if defined(XSPI1)
/**
  * @details This function enables the XSPI1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_XSPI1_EnableClockInSleepMode(void)
{
  LL_AHB4_GRP1_EnableClockLowPower(LL_AHB4_GRP1_PERIPH_XSPI1);
}

#endif /* XSPI1 */
#if defined(XSPI1)
/**
  * @details This function disables the XSPI1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_XSPI1_DisableClockInSleepMode(void)
{
  LL_AHB4_GRP1_DisableClockLowPower(LL_AHB4_GRP1_PERIPH_XSPI1);
}
/**
  * @}
  */

#endif /* XSPI1 */
#endif /* XSPI1 || XSPI2 || FMC_BASE */
/** @defgroup RCC_APB1_Peripheral_Clock_Sleep_Enable_Disable APB1 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the APB1 peripheral clock during Low Power Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/**
  * @details This function enables the TIM2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM2_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM2);
}

#if defined(TIM3)
/**
  * @details This function enables the TIM3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM3_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function enables the TIM4 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM4_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function enables the TIM5 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM5_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function enables the TIM6 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM6_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function enables the TIM7 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM7_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function enables the TIM12 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM12_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_TIM12);
}

/**
  * @details This function enables the WWDG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_WWDG_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_WWDG);
}

#if defined(OPAMP1)
/**
  * @details This function enables the OPAMP1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_OPAMP1_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function enables the SPI2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI2_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_SPI2);
}

/**
  * @details This function enables the SPI2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI2_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_SPI2);
}
#if defined(SPI3)
/**
  * @details This function enables the SPI3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI3_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_SPI3);
}

/**
  * @details This function enables the SPI3 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI3_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function enables the USART2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART2_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART2);
}

/**
  * @details This function enables the USART2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART2_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART2);
}

#if defined(USART3)
/**
  * @details This function enables the USART3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART3_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART3);
}

/**
  * @details This function enables the USART3 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART3_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function enables the UART4 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART4_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function enables the UART4 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART4_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function enables the UART5 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART5_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function enables the UART5 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART5_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function enables the I2C1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C1_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I2C1);
}

/**
  * @details This function enables the I2C1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C1_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function enables the I2C2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C2_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I2C2);
}

/**
  * @details This function enables the I2C2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C2_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function enables the I3C1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I3C1_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function enables the I3C1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I3C1_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function enables the CRS clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CRS_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_CRS);
}

#if defined(USART6)
/**
  * @details This function enables the USART6 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART6_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART6);
}

/**
  * @details This function enables the USART6 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART6_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function enables the UART7 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART7_EnableClockInSleepMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART7);
}

/**
  * @details This function enables the UART7 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART7_EnableClockInStopMode(void)
{
  LL_APB1_GRP1_EnableClockLowPower(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function enables the COMP12 clock in sleep mode.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_LP_COMP12_EnableClockInSleepMode(void)
{
  LL_APB1_GRP2_EnableClockLowPower(LL_APB1_GRP2_PERIPH_COMP12);
}

/**
  * @details This function enables the COMP12 clock in stop mode.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_LP_COMP12_EnableClockInStopMode(void)
{
  LL_APB1_GRP2_EnableClockLowPower(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)
/**
  * @details This function enables the FDCAN clock in sleep mode.
  * @note    The FDCAN clock is common for all FDCAN instances
  */
__STATIC_INLINE void HAL_RCC_LP_FDCAN_EnableClockInSleepMode(void)
{
  LL_APB1_GRP2_EnableClockLowPower(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @details This function disables the TIM2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM2_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM2);
}
#if defined(TIM3)
/**
  * @details This function disables the TIM3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM3_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM3);
}

#endif /* TIM3 */
#if defined(TIM4)
/**
  * @details This function disables the TIM3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM4_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM4);
}

#endif /* TIM4 */
#if defined(TIM5)
/**
  * @details This function disables the TIM5 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM5_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM5);
}

#endif /* TIM5 */
/**
  * @details This function disables the TIM6 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM6_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM6);
}

/**
  * @details This function disables the TIM7 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM7_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM7);
}

/**
  * @details This function disables the TIM12 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM12_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_TIM12);
}

/**
  * @details This function disables the WWDG clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_WWDG_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_WWDG);
}

#if defined(OPAMP1)
/**
  * @details This function disables the OPAMP1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_OPAMP1_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_OPAMP1);
}

#endif /* OPAMP1 */
/**
  * @details This function disables the SPI2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI2_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_SPI2);
}

/**
  * @details This function disables the SPI2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI2_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_SPI2);
}
#if defined(SPI3)
/**
  * @details This function disables the SPI3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI3_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_SPI3);
}

/**
  * @details This function disables the SPI3 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI3_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_SPI3);
}

#endif /* SPI3 */
/**
  * @details This function disables the USART2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART2_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART2);
}

/**
  * @details This function disables the USART2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART2_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART2);
}
#if defined(USART3)
/**
  * @details This function disables the USART3 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART3_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART3);
}

/**
  * @details This function disables the USART3 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART3_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART3);
}

#endif /* USART3 */
/**
  * @details This function disables the UART4 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART4_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function disables the UART4 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART4_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART4);
}

/**
  * @details This function disables the UART5 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART5_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART5);
}

/**
  * @details This function disables the UART5 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART5_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART5);
}

#if defined(USART6)
/**
  * @details This function disables the USART6 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART6_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART6);
}

/**
  * @details This function disables the USART6 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART6_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_USART6);
}

#endif /* USART6 */
#if defined(UART7)
/**
  * @details This function disables the UART7 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART7_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART7);
}

/**
  * @details This function disables the UART7 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_UART7_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_UART7);
}

#endif /* UART7 */
/**
  * @details This function disables the I2C1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C1_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I2C1);
}

/**
  * @details This function disables the I2C1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C1_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I2C1);
}

#if defined(I2C2)
/**
  * @details This function disables the I2C2 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C2_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I2C2);
}

/**
  * @details This function disables the I2C2 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I2C2_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I2C2);
}

#endif /* I2C2 */
/**
  * @details This function disables the I3C1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I3C1_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function disables the I3C1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_I3C1_DisableClockInStopMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_I3C1);
}

/**
  * @details This function disables the CRS clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_CRS_DisableClockInSleepMode(void)
{
  LL_APB1_GRP1_DisableClockLowPower(LL_APB1_GRP1_PERIPH_CRS);
}
/**
  * @details This function disables the COMP12 clock in sleep mode.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_LP_COMP12_DisableClockInSleepMode(void)
{
  LL_APB1_GRP2_DisableClockLowPower(LL_APB1_GRP2_PERIPH_COMP12);
}

/**
  * @details This function disables the COMP12 clock in stop mode.
  * @note    COMP2 not defined in all devices.
  */
__STATIC_INLINE void HAL_RCC_LP_COMP12_DisableClockInStopMode(void)
{
  LL_APB1_GRP2_DisableClockLowPower(LL_APB1_GRP2_PERIPH_COMP12);
}

#if defined(FDCAN1)

/**
  * @details This function disables the FDCAN clock in sleep mode.
  * @note    The FDCAN clock is common for all FDCAN instances
  */
__STATIC_INLINE void HAL_RCC_LP_FDCAN_DisableClockInSleepMode(void)
{
  LL_APB1_GRP2_DisableClockLowPower(LL_APB1_GRP2_PERIPH_FDCAN);
}

#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup RCC_APB2_Peripheral_Clock_Sleep_Enable_Disable APB2 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the APB2 peripheral clock during Low Power Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/**
  * @details This function enables the TIM1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM1_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function enables the SPI1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI1_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function enables the SPI1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI1_EnableClockInStopMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function enables the TIM8 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM8_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_TIM8);
}

/**
  * @details This function enables the USART1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART1_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function enables the USART1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART1_EnableClockInStopMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function enables the TIM15 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM15_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function enables the TIM16 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM16_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function enables the TIM17 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM17_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function enables the USB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USB_EnableClockInSleepMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_USB);
}

/**
  * @details This function enables the USB clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USB_EnableClockInStopMode(void)
{
  LL_APB2_GRP1_EnableClockLowPower(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @details This function disables the TIM1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM1_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_TIM1);
}

/**
  * @details This function disables the SPI1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI1_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function disables the SPI1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SPI1_DisableClockInStopMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_SPI1);
}

/**
  * @details This function disables the TIM8 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM8_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_TIM8);
}

/**
  * @details This function disables the USART1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART1_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function disables the USART1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USART1_DisableClockInStopMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_USART1);
}

/**
  * @details This function disables the TIM15 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM15_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_TIM15);
}

#if defined(TIM16)
/**
  * @details This function disables the TIM16 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM16_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_TIM16);
}

#endif /* TIM16 */
#if defined(TIM17)
/**
  * @details This function disables the TIM17 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_TIM17_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_TIM17);
}

#endif /* TIM17 */
#if defined(USB_DRD_FS_BASE)
/**
  * @details This function disables the USB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USB_DisableClockInSleepMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_USB);
}

/**
  * @details This function disables the USB clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_USB_DisableClockInStopMode(void)
{
  LL_APB2_GRP1_DisableClockLowPower(LL_APB2_GRP1_PERIPH_USB);
}

#endif /* USB_DRD_FS_BASE */
/**
  * @}
  */

/** @defgroup RCC_APB3_Peripheral_Clock_Sleep_Enable_Disable APB3 Peripheral Clock Sleep Enable Disable
  * @brief  Enable or disable the APB3 peripheral clock during Low Power Sleep mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/**
  * @details This function enables the SBS clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SBS_EnableClockInSleepMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function enables the LPUART1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPUART1_EnableClockInSleepMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_LPUART1);
}

/**
  * @details This function enables the LPUART1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPUART1_EnableClockInStopMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_LPUART1);
}
#if defined(LPTIM1)
/**
  * @details This function enables the LPTIM1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPTIM1_EnableClockInSleepMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_LPTIM1);
}

/**
  * @details This function enables the LPTIM1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPTIM1_EnableClockInStopMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @details This function enables the RTCAPB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RTCAPB_EnableClockInSleepMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @details This function enables the RTCAPB clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RTCAPB_EnableClockInStopMode(void)
{
  LL_APB3_GRP1_EnableClockLowPower(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @details This function disables the SBS clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_SBS_DisableClockInSleepMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_SBS);
}

/**
  * @details This function disables the LPUART1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPUART1_DisableClockInSleepMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_LPUART1);
}

/**
  * @details This function disables the LPUART1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPUART1_DisableClockInStopMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_LPUART1);
}

#if defined(LPTIM1)
/**
  * @details This function disables the LPTIM1 clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPTIM1_DisableClockInSleepMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_LPTIM1);
}

/**
  * @details This function disables the LPTIM1 clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_LPTIM1_DisableClockInStopMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_LPTIM1);
}

#endif /* LPTIM1 */
/**
  * @details This function disables the RTCAPB clock in sleep mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RTCAPB_DisableClockInSleepMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @details This function disables the RTCAPB clock in stop mode.
  */
__STATIC_INLINE void HAL_RCC_LP_RTCAPB_DisableClockInStopMode(void)
{
  LL_APB3_GRP1_DisableClockLowPower(LL_APB3_GRP1_PERIPH_RTCAPB);
}

/**
  * @}
  */

/** @defgroup RCC_RTC_Reset RCC RTC Domain Reset
  * @brief  Reset the RTC domain.
  * @{
  */

/** @brief    Function to force and release the RTC domain.
  * @details  When a RTC domain reset occurs, the following domains are impacted:
  *           - the RTC is stopped and all the RTC registers are set to their reset values
  *             (including the backup registers)
  *           - all TAMP registers can be read or written.
  *           - LSE crystal 32kHz crystal oscillator
  *           - RCC_RTCCR register
  */
__STATIC_INLINE void HAL_RCC_ResetRTCDomain(void)
{
  LL_RCC_ForceRTCDomainReset();
  LL_RCC_ReleaseRTCDomainReset();
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group3 RCC services functions
  This subsection provides a set of functions allowing to:

  - Configure the MCO.
    - MCO (Microcontroller Clock Output): used to output HSIK, HSIS, HSIDIV3, PSIK, PSIS, PSIDIV3, LSE,
      LSI, HSE, SYSCLK on a pin.

  - Get and clear reset flags

  - Enable the Clock Security System.
    - CSS (Clock Security System): once enabled, if a HSE clock failure occurs
    (HSE used directly or through PSI as System clock source), the System clock
    is automatically switched to HSI.
    If a LSE clock failure occurs (LSE used through PSI as System clock source), the System clock
    is automatically switched to HSI.
    The CSS is linked to the Cortex-M33 NMI (non-maskable interrupt) exception vector.
    The NMI is executed indefinitely unless the CSS interrupt flag (CSSF) pending bit is cleared.
    Therefore, in the NMI ISR, ensure to clear the CSSF by setting the CSSC bit.

  - Enable Clock security system on LSE.

  - Configure and get the oscillator Kernel clock source for wakeup from Stop.

  - Enable output (LSCO) allows one of the low-speed clocks below to be output onto the external LSCO pin:
    - LSI
    - LSE
    - This output remains available in all Stop modes and Standby mode

  - Enable RTC and TAMP clock
  *
  * @{
  */

void                         HAL_RCC_SetConfigMCO(hal_rcc_mco_src_t mcox_src, hal_rcc_mco_prescaler_t mco_prescaler);
uint32_t                     HAL_RCC_GetResetSource(void);
void                         HAL_RCC_ClearResetFlags(void);
#if defined(HSE_VALUE)
void                         HAL_RCC_HSE_EnableCSS(void);
hal_status_t                 HAL_RCC_HSE_CSSCallback(void);
void                         HAL_RCC_RTC_SetHSEPrescaler(uint32_t prescaler);
uint32_t                     HAL_RCC_RTC_GetHSEPrescaler(void);
#endif /* HSE_VALUE */
hal_status_t                 HAL_RCC_NMI_IRQHandler(void);
void                         HAL_RCC_SetClockAfterWakeFromStop(hal_rcc_stop_wakeup_clk_t wakeup_clk);
hal_rcc_stop_wakeup_clk_t    HAL_RCC_GetClockAfterWakeFromStop(void);
#if defined(LSE_VALUE)
void                         HAL_RCC_LSE_EnableCSS(void);
void                         HAL_RCC_LSE_DisableCSS(void);
hal_status_t                 HAL_RCC_LSE_CSSCallback(void);
#endif /* LSE_VALUE */
hal_status_t                 HAL_RCC_EnableLSCO(hal_rcc_lsco_src_t source);
hal_status_t                 HAL_RCC_DisableLSCO(void);
hal_status_t                 HAL_RCC_RTC_EnableKernelClock(void);
hal_status_t                 HAL_RCC_RTC_DisableKernelClock(void);
/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group4 Kernel clock source configuration for peripherals
  This subsection provides a set of functions:
  - to control Kernel clock source configuration for peripherals
  - to control specific Kernel clock prescaler for peripherals if available
  - to get peripheral clock frequency

  Only functions having a kernel clock are handled by these functions.

    - The peripheral clock can be selected through the independent
      - API HAL_RCC_{PERIPHx}_SetKernelClkSource(hal_rcc_{periphx}_clk_src_t clk_src)
      example: @ref HAL_RCC_USART1_SetKernelClkSource(hal_rcc_usart1_clk_src_t clk_src)

      Usage: activate PSIK as clock source of USART1

      - with @ref HAL_RCC_USART1_SetKernelClkSource function:
      @code
        HAL_RCC_USART1_SetKernelClkSource(HAL_RCC_USART1_CLK_SRC_PSIK);
      @endcode

      - to get the peripheral Clocks frequencies:
      API hal_rcc_{periphx}_clk_src_t HAL_RCC_{PERIPHx}_GetKernelClkSource(void)


    - The peripheral clock can be divided by a specific value
      - API HAL_RCC_{PERIPHx}_SetKernelClkPrescaler(hal_rcc_{periphx}_prescaler_t prescaler)
      example: @ref HAL_RCC_ADCDAC_SetKernelClkPrescaler(hal_rcc_adcdac_prescaler_t prescaler)
      - API HAL_RCC_{PERIPHx}_prescaler_t HAL_RCC_{PERIPHx}_GetKernelClkPrescaler(void)
      example: @ref hal_rcc_adcdac_prescaler_t HAL_RCC_ADCDAC_GetKernelClkPrescaler(void)

    - The functionalities of the two previous functions are merged into one common function
      - API HAL_RCC_{PERIPHx}_SetConfigKernelClk(hal_rcc_{periphx}_clk_src_t clk_src,
                                            hal_rcc_{periphx}_prescaler_t prescaler)
      example: @ref HAL_RCC_ADCDAC_SetConfigKernelClk(hal_rcc_adcdac_clk_src_t clk_src,
                                                    hal_rcc_adcdac_prescaler_t prescaler)
      - API HAL_RCC_{PERIPHx}_GetConfigKernelClk(hal_rcc_{periphx}_clk_src_t clk_src,
                                            hal_rcc_{periphx}_prescaler_t prescaler)
      example: @ref HAL_RCC_ADCDAC_GetConfigKernelClk(hal_rcc_adcdac_clk_src_t *p_clk_src,
                                                    hal_rcc_adcdac_prescaler_t *p_prescaler)


    - All peripherals can get their peripheral clock frequency
      - API uint32_t HAL_RCC_{PERIPHx}_GetKernelClkFreq(void)
      example: @ref HAL_RCC_USART1_GetKernelClkFreq(void)

  * @{
  */

hal_status_t             HAL_RCC_USART1_SetKernelClkSource(hal_rcc_usart1_clk_src_t clk_src);
hal_status_t             HAL_RCC_USART2_SetKernelClkSource(hal_rcc_usart2_clk_src_t clk_src);
#if defined(USART3)
hal_status_t             HAL_RCC_USART3_SetKernelClkSource(hal_rcc_usart3_clk_src_t clk_src);
#endif /* USART3 */
hal_status_t             HAL_RCC_UART4_SetKernelClkSource(hal_rcc_uart4_clk_src_t clk_src);
hal_status_t             HAL_RCC_UART5_SetKernelClkSource(hal_rcc_uart5_clk_src_t clk_src);
#if defined(USART6)
hal_status_t             HAL_RCC_USART6_SetKernelClkSource(hal_rcc_usart6_clk_src_t clk_src);
#endif /* USART6 */
#if defined(UART7)
hal_status_t             HAL_RCC_UART7_SetKernelClkSource(hal_rcc_uart7_clk_src_t clk_src);
#endif /* UART7 */
hal_status_t             HAL_RCC_LPUART1_SetKernelClkSource(hal_rcc_lpuart1_clk_src_t clk_src);
hal_status_t             HAL_RCC_SPI1_SetKernelClkSource(hal_rcc_spi1_clk_src_t clk_src);
hal_status_t             HAL_RCC_SPI2_SetKernelClkSource(hal_rcc_spi2_clk_src_t clk_src);
#if defined(SPI3)
hal_status_t             HAL_RCC_SPI3_SetKernelClkSource(hal_rcc_spi3_clk_src_t clk_src);
#endif /* SPI3 */
#if defined(FDCAN1)
hal_status_t             HAL_RCC_FDCAN_SetKernelClkSource(hal_rcc_fdcan_clk_src_t clk_src);
#endif /* FDCAN1 */
hal_status_t             HAL_RCC_I2C1_SetKernelClkSource(hal_rcc_i2c1_clk_src_t clk_src);
#if defined(I2C2)
hal_status_t             HAL_RCC_I2C2_SetKernelClkSource(hal_rcc_i2c2_clk_src_t clk_src);
#endif /* I2C2 */
hal_status_t             HAL_RCC_I3C1_SetKernelClkSource(hal_rcc_i3c1_clk_src_t clk_src);
hal_status_t             HAL_RCC_ADCDAC_SetKernelClkSource(hal_rcc_adcdac_clk_src_t clk_src);
hal_status_t             HAL_RCC_DAC1_SetSampleHoldClkSource(hal_rcc_dac1_sh_clk_src_t clk_src);
#if defined(LPTIM1)
hal_status_t             HAL_RCC_LPTIM1_SetKernelClkSource(hal_rcc_lptim1_clk_src_t clk_src);
#endif /* LPTIM1 */
hal_status_t             HAL_RCC_CK48_SetKernelClkSource(hal_rcc_ck48_clk_src_t clk_src);
#if defined(XSPI1)
hal_status_t             HAL_RCC_XSPI1_SetKernelClkSource(hal_rcc_xspi1_clk_src_t clk_src);
#endif /* XSPI1 */
#if defined(ETH1)
hal_status_t             HAL_RCC_ETH1REF_SetKernelClkSource(hal_rcc_eth1ref_clk_src_t clk_src);
hal_status_t             HAL_RCC_ETH1PTP_SetKernelClkSource(hal_rcc_eth1ptp_clk_src_t clk_src);
hal_status_t             HAL_RCC_ETH1_SetKernelClkSource(hal_rcc_eth1_clk_src_t clk_src);
#endif /* ETH1 */
hal_status_t             HAL_RCC_RTC_SetKernelClkSource(hal_rcc_rtc_clk_src_t clk_src);

hal_rcc_usart1_clk_src_t  HAL_RCC_USART1_GetKernelClkSource(void);
hal_rcc_usart2_clk_src_t  HAL_RCC_USART2_GetKernelClkSource(void);
#if defined(USART3)
hal_rcc_usart3_clk_src_t  HAL_RCC_USART3_GetKernelClkSource(void);
#endif /* USART3 */
hal_rcc_uart4_clk_src_t   HAL_RCC_UART4_GetKernelClkSource(void);
hal_rcc_uart5_clk_src_t   HAL_RCC_UART5_GetKernelClkSource(void);
#if defined(USART6)
hal_rcc_usart6_clk_src_t  HAL_RCC_USART6_GetKernelClkSource(void);
#endif /* USART6 */
#if defined(UART7)
hal_rcc_uart7_clk_src_t   HAL_RCC_UART7_GetKernelClkSource(void);
#endif /* UART7 */
hal_rcc_lpuart1_clk_src_t HAL_RCC_LPUART1_GetKernelClkSource(void);
hal_rcc_spi1_clk_src_t    HAL_RCC_SPI1_GetKernelClkSource(void);
hal_rcc_spi2_clk_src_t    HAL_RCC_SPI2_GetKernelClkSource(void);
#if defined(SPI3)
hal_rcc_spi3_clk_src_t    HAL_RCC_SPI3_GetKernelClkSource(void);
#endif /* SPI3 */
#if defined(FDCAN1)
hal_rcc_fdcan_clk_src_t   HAL_RCC_FDCAN_GetKernelClkSource(void);
#endif /* FDCAN1 */
hal_rcc_i2c1_clk_src_t    HAL_RCC_I2C1_GetKernelClkSource(void);
#if defined(I2C2)
hal_rcc_i2c2_clk_src_t    HAL_RCC_I2C2_GetKernelClkSource(void);
#endif /* I2C2 */
hal_rcc_i3c1_clk_src_t    HAL_RCC_I3C1_GetKernelClkSource(void);
hal_rcc_adcdac_clk_src_t  HAL_RCC_ADCDAC_GetKernelClkSource(void);
hal_rcc_dac1_sh_clk_src_t    HAL_RCC_DAC1_GetSampleHoldClkSource(void);
#if defined(LPTIM1)
hal_rcc_lptim1_clk_src_t  HAL_RCC_LPTIM1_GetKernelClkSource(void);
#endif /* LPTIM1 */
hal_rcc_ck48_clk_src_t HAL_RCC_CK48_GetKernelClkSource(void);
#if defined(XSPI1)
hal_rcc_xspi1_clk_src_t   HAL_RCC_XSPI1_GetKernelClkSource(void);
#endif /* XSPI1 */
#if defined(ETH1)
hal_rcc_eth1ref_clk_src_t  HAL_RCC_ETH1REF_GetKernelClkSource(void);
hal_rcc_eth1ptp_clk_src_t  HAL_RCC_ETH1PTP_GetKernelClkSource(void);
hal_rcc_eth1_clk_src_t     HAL_RCC_ETH1_GetKernelClkSource(void);
#endif /* ETH1 */
hal_rcc_rtc_clk_src_t     HAL_RCC_RTC_GetKernelClkSource(void);

hal_status_t               HAL_RCC_ADCDAC_SetKernelClkPrescaler(hal_rcc_adcdac_prescaler_t prescaler);
hal_rcc_adcdac_prescaler_t HAL_RCC_ADCDAC_GetKernelClkPrescaler(void);
hal_status_t               HAL_RCC_ADCDAC_SetConfigKernelClk(hal_rcc_adcdac_clk_src_t clk_src,
                                                             hal_rcc_adcdac_prescaler_t prescaler);
void                       HAL_RCC_ADCDAC_GetConfigKernelClk(hal_rcc_adcdac_clk_src_t *p_clk_src,
                                                             hal_rcc_adcdac_prescaler_t *p_prescaler);
#if defined(ETH1)
hal_status_t                HAL_RCC_ETH1_SetKernelClkPrescaler(hal_rcc_eth1_prescaler_t eth_prescaler);
hal_rcc_eth1_prescaler_t    HAL_RCC_ETH1_GetKernelClkPrescaler(void);
hal_status_t                HAL_RCC_ETH1_SetConfigKernelClk(hal_rcc_eth1_clk_src_t clk_src,
                                                            hal_rcc_eth1_prescaler_t eth_prescaler);
void                        HAL_RCC_ETH1_GetConfigKernelClk(hal_rcc_eth1_clk_src_t *p_clk_src,
                                                            hal_rcc_eth1_prescaler_t *p_prescaler);
hal_status_t                HAL_RCC_ETH1PTP_SetKernelClkPrescaler(uint32_t ethptp_prescaler);
uint32_t                    HAL_RCC_ETH1PTP_GetKernelClkPrescaler(void);
hal_status_t                HAL_RCC_ETH1PTP_SetConfigKernelClk(hal_rcc_eth1ptp_clk_src_t clk_src,
                                                               uint32_t ethptp_prescaler);
void                        HAL_RCC_ETH1PTP_GetConfigKernelClk(hal_rcc_eth1ptp_clk_src_t *p_clk_src,
                                                               uint32_t *p_prescaler);
#endif /* ETH1 */

uint32_t                  HAL_RCC_USART_GetKernelClkFreq(const USART_TypeDef *usartx);
uint32_t                  HAL_RCC_UART_GetKernelClkFreq(const USART_TypeDef *uartx);
uint32_t                  HAL_RCC_USART1_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_USART2_GetKernelClkFreq(void);
#if defined(USART3)
uint32_t                  HAL_RCC_USART3_GetKernelClkFreq(void);
#endif /* USART3 */
uint32_t                  HAL_RCC_UART4_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_UART5_GetKernelClkFreq(void);
#if defined(USART6)
uint32_t                  HAL_RCC_USART6_GetKernelClkFreq(void);
#endif /* USART6 */
#if defined(UART7)
uint32_t                  HAL_RCC_UART7_GetKernelClkFreq(void);
#endif /* UART7 */
uint32_t                  HAL_RCC_LPUART1_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_SPI_GetKernelClkFreq(const SPI_TypeDef *spix);
uint32_t                  HAL_RCC_SPI1_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_SPI2_GetKernelClkFreq(void);
#if defined(SPI3)
uint32_t                  HAL_RCC_SPI3_GetKernelClkFreq(void);
#endif /* SPI3 */
#if defined(FDCAN1)
uint32_t                  HAL_RCC_FDCAN_GetKernelClkFreq(void);
#endif /* FDCAN1 */
uint32_t                  HAL_RCC_I2C_GetKernelClkFreq(const I2C_TypeDef *i2cx);
uint32_t                  HAL_RCC_I2C1_GetKernelClkFreq(void);
#if defined(I2C2)
uint32_t                  HAL_RCC_I2C2_GetKernelClkFreq(void);
#endif /* I2C2 */
uint32_t                  HAL_RCC_I3C_GetKernelClkFreq(const I3C_TypeDef *i3cx);
uint32_t                  HAL_RCC_I3C1_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_ADC_GetKernelClkFreq(const ADC_TypeDef *adcx);
uint32_t                  HAL_RCC_ADCDAC_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_DAC_GetKernelClkFreq(const DAC_TypeDef *dacx);
#if defined(LPTIM1)
uint32_t                  HAL_RCC_LPTIM_GetKernelClkFreq(const LPTIM_TypeDef *lptimx);
uint32_t                  HAL_RCC_LPTIM1_GetKernelClkFreq(void);
#endif /* LPTIM1 */
uint32_t                  HAL_RCC_TIM_GetKernelClkFreq(const TIM_TypeDef *timx);
#if defined(XSPI1)
uint32_t                  HAL_RCC_XSPI_GetKernelClkFreq(const XSPI_TypeDef *xspix);
uint32_t                  HAL_RCC_XSPI1_GetKernelClkFreq(void);
#endif /* XSPI1 */
#if defined(ETH1)
uint32_t                  HAL_RCC_ETH1PTP_GetKernelClkFreq(void);
uint32_t                  HAL_RCC_ETH1_GetKernelClkFreq(void);
#endif /* ETH1 */

uint32_t                  HAL_RCC_DAC1_GetSampleHoldClkFreq(void);
uint32_t                  HAL_RCC_RTC_GetKernelClkFreq(void);
/**
  * @}
  */

/** @defgroup RCC_Exported_Functions_Group5 RCC privileged access levels attributes management
  * This subsection provides a set of functions:
  * @{
  */
hal_status_t  HAL_RCC_SetPrivAttr(uint32_t item, hal_rcc_priv_attr_t priv_attr);
hal_rcc_priv_attr_t HAL_RCC_GetPrivAttr(uint32_t item);
/**
  * @}
  */ /* RCC_Exported_Functions_Group5 */

/**
  * @}
  */

#endif /* RCC */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_RCC_H */
