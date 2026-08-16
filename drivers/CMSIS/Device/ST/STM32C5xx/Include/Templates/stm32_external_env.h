/**
  ******************************************************************************
  * @file    stm32_external_env.h
  * @brief   External environments values (external oscillators values).
  *          This file should be copied to the application folder.
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
#ifndef STM32_EXTERNAL_ENV_H
#define STM32_EXTERNAL_ENV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## VDD Value #######################################*/
/**
  * @brief VDD Value.
  */
#if !defined  (VDD_VALUE)
#define  VDD_VALUE             3300UL /*!< Value of VDD in mv */
#endif /* VDD_VALUE */

/* ########################## Oscillator Values adaptation ####################*/
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PSI).
  */
#if !defined  (HSE_VALUE)
#if defined (AHB4PERIPH_BASE)
#define HSE_VALUE              48000000UL /*!< Value of the External oscillator in Hz */
#else
#define HSE_VALUE              24000000UL /*!< Value of the External oscillator in Hz */
#endif /* AHB4PERIPH_BASE */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100UL   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief External Low Speed oscillator (LSE) value.
  *        This value is used by the UART, RTC HAL module to compute the system frequency
  */
#if !defined  (LSE_VALUE)
#define LSE_VALUE              32768UL   /*!< Value of the External oscillator in Hz*/
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000UL     /*!< Time out for LSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

#ifdef __cplusplus
}
#endif

#endif /* STM32_EXTERNAL_ENV_H */
