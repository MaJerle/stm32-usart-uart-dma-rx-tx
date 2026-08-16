/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_crs.h
  * @brief   Header file of CRS LL module.
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
#ifndef STM32C5XX_LL_CRS_H
#define STM32C5XX_LL_CRS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(CRS)

/** @defgroup CRS_LL CRS
  * @{
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/* Private macros ----------------------------------------------------------------------------------------------------*/
/* Exported types ----------------------------------------------------------------------------------------------------*/
/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_LL_Exported_Constants LL CRS Constants
  * @{
  */

/** @defgroup CRS_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_CRS_WRITE_REG function.
  * @{
  */
#define LL_CRS_ICR_SYNCOKC                 CRS_ICR_SYNCOKC
#define LL_CRS_ICR_SYNCWARNC               CRS_ICR_SYNCWARNC
#define LL_CRS_ICR_ERRC                    CRS_ICR_ERRC
#define LL_CRS_ICR_ESYNCC                  CRS_ICR_ESYNCC
/**
  * @}
  */

/** @defgroup CRS_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_CRS_READ_REG function.
  * @{
  */
#define LL_CRS_ISR_SYNCOKF                 CRS_ISR_SYNCOKF
#define LL_CRS_ISR_SYNCWARNF               CRS_ISR_SYNCWARNF
#define LL_CRS_ISR_ERRF                    CRS_ISR_ERRF
#define LL_CRS_ISR_ESYNCF                  CRS_ISR_ESYNCF
#define LL_CRS_ISR_SYNCERR                 CRS_ISR_SYNCERR
#define LL_CRS_ISR_SYNCMISS                CRS_ISR_SYNCMISS
#define LL_CRS_ISR_TRIMOVF                 CRS_ISR_TRIMOVF
/**
  * @}
  */

/** @defgroup CRS_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_CRS_READ_REG and  LL_CRS_WRITE_REG functions.
  * @{
  */
#define LL_CRS_CR_SYNCOKIE                 CRS_CR_SYNCOKIE
#define LL_CRS_CR_SYNCWARNIE               CRS_CR_SYNCWARNIE
#define LL_CRS_CR_ERRIE                    CRS_CR_ERRIE
#define LL_CRS_CR_ESYNCIE                  CRS_CR_ESYNCIE
/**
  * @}
  */

/** @defgroup CRS_LL_EC_AUTO_TRIMMING Auto trimming
  * @{
  */
#define LL_CRS_AUTO_TRIMMING_DISABLE       0U                      /*!< Auto trimming disabled (default) */
#define LL_CRS_AUTO_TRIMMING_ENABLE        CRS_CR_AUTOTRIMEN       /*!< Auto trimming enabled */
/**
  * @}
  */

/** @defgroup CRS_LL_EC_SYNC_DIV Synchronization Signal Divider
  * @{
  */
#define LL_CRS_SYNC_DIV_1        0U                                 /*!< Synchronization signal not divided (default) */
#define LL_CRS_SYNC_DIV_2        CRS_CFGR_SYNCDIV_0                        /*!< Synchronization signal divided by 2   */
#define LL_CRS_SYNC_DIV_4        CRS_CFGR_SYNCDIV_1                        /*!< Synchronization signal divided by 4   */
#define LL_CRS_SYNC_DIV_8        (CRS_CFGR_SYNCDIV_1 | CRS_CFGR_SYNCDIV_0) /*!< Synchronization signal divided by 8   */
#define LL_CRS_SYNC_DIV_16       CRS_CFGR_SYNCDIV_2                        /*!< Synchronization signal divided by 16  */
#define LL_CRS_SYNC_DIV_32       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_0) /*!< Synchronization signal divided by 32  */
#define LL_CRS_SYNC_DIV_64       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_1) /*!< Synchronization signal divided by 64  */
#define LL_CRS_SYNC_DIV_128      CRS_CFGR_SYNCDIV                          /*!< Synchronization signal divided by 128 */
/**
  * @}
  */

/** @defgroup CRS_LL_EC_SYNC_SOURCE Synchronization Signal Source
  * @{
  */
#define LL_CRS_SYNC_SOURCE_GPIO       0U                      /*!< Synchronization signal source GPIO */
#define LL_CRS_SYNC_SOURCE_LSE        CRS_CFGR_SYNCSRC_0      /*!< Synchronization signal source LSE */
#define LL_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      /*!< Synchronization signal source USB SOF (default) */
#define LL_CRS_SYNC_SOURCE_HSE_1MHZ   (CRS_CFGR_SYNCSRC_0 | \
                                       CRS_CFGR_SYNCSRC_1)    /*!< Synchronization signal source HSE 1MHz */
/**
  * @}
  */

/** @defgroup CRS_LL_EC_SYNC_POLARITY Synchronization Signal Polarity
  * @{
  */
#define LL_CRS_SYNC_POLARITY_RISING        0U                    /*!< Synchronization active on rising edge (default) */
#define LL_CRS_SYNC_POLARITY_FALLING       CRS_CFGR_SYNCPOL      /*!< Synchronization active on falling edge */
/**
  * @}
  */

/** @defgroup CRS_LL_EC_FREQERRORDIR Frequency Error Direction
  * @{
  */
#define LL_CRS_FREQ_ERROR_DIR_UP           0U                  /*!< Upcounting direction, the actual frequency
                                                                    is above the target */
#define LL_CRS_FREQ_ERROR_DIR_DOWN         CRS_ISR_FEDIR       /*!< Downcounting direction, the actual frequency
                                                                    is below the target */
/**
  * @}
  */

/** @defgroup CRS_LL_EC_DEFAULTVALUES Default Values
  * @{
  */
/**
  * @brief Reset value of the RELOAD field.
  * @note The reset value of the RELOAD field corresponds to a target frequency of 48 MHz
  *       and a synchronization signal frequency of 1 kHz (SOF signal from USB).
  */
#define LL_CRS_RELOADVALUE_DEFAULT         0x0000BB7FU

/**
  * @brief Reset value of frequency error limit.
  */
#define LL_CRS_ERRORLIMIT_DEFAULT          0x00000022U

/**
  * @brief Reset value of the HSI144 Calibration field.
  * @note The default value is 0x30U, which corresponds to the middle of the trimming interval.
  *       The trimming step is specified in the product datasheet.
  *       A higher TRIM value corresponds to a higher output frequency.
  */
#define LL_CRS_HSI144CALIBRATION_DEFAULT    0x30U

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_LL_Exported_Macros LL CRS Macros
  * @{
  */

/** @defgroup CRS_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in the CRS register.
  * @param  instance CRS instance.
  * @param  reg Register to be written.
  * @param  value Value to be written in the register.
  */
#define LL_CRS_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in the CRS register.
  * @param  instance CRS instance.
  * @param  reg Register to be read.
  * @retval Register value
  */
#define LL_CRS_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/** @defgroup CRS_LL_EM_Exported_Macros_Calculate_Reload Exported_Macros_Calculate_Reload
  * @{
  */

/**
  * @brief  Macro to calculate the reload value to be set in the CRS register according to target and sync frequencies.
  * @param  ftarget Target frequency (value in Hz).
  * @param  fsync Synchronization signal frequency (value in Hz).
  * @note   The RELOAD value must be selected according to the ratio between
  *         the target frequency and the frequency of the synchronization source after
  *         prescaling. It is then decreased by one to reach the expected
  *         synchronization on the zero value. The formula is as follows:
  *              RELOAD = (ftarget / fsync) -1
  * @retval Reload value (in Hz).
  */
#define LL_CRS_CALCULATE_RELOAD(ftarget, fsync) (((ftarget) / (fsync)) - 1U)

/**
  * @brief  Macro to read the frequency error direction value in CRS register.
  * @param  value Value returned by LL_CRS_GetFreqErrorInfo().
  * @retval Frequency error direction value
  */
#define LL_CRS_READ_FREQ_ERROR_DIRECTION(value) (STM32_READ_BIT((value), CRS_ISR_FEDIR))

/**
  * @brief  Macro to read the frequency error capture value in CRS register.
  * @param  value Value returned by LL_CRS_GetFreqErrorInfo().
  * @retval Frequency error capture value
  */
#define LL_CRS_READ_FREQ_ERROR_CAPTURE(value) (STM32_READ_BIT((value), CRS_ISR_FECAP) >> CRS_ISR_FECAP_Pos)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_LL_Exported_Functions LL CRS Functions
  * @{
  */

/** @defgroup CRS_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable frequency error counter.
  * @rmtoll
  *  CR           CEN           LL_CRS_EnableFreqErrorCounter
  * @param  crsx CRS instance.
  * @note When this bit is set, the CRS_CFGR register is write-protected and cannot be modified.
  */
__STATIC_INLINE void LL_CRS_EnableFreqErrorCounter(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_CEN);
}

/**
  * @brief  Disable frequency error counter.
  * @rmtoll
  *  CR           CEN           LL_CRS_DisableFreqErrorCounter
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableFreqErrorCounter(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_CEN);
}

/**
  * @brief  Check whether the frequency error counter is enabled.
  * @rmtoll
  *  CR           CEN           LL_CRS_IsEnabledFreqErrorCounter
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledFreqErrorCounter(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_CEN) == (CRS_CR_CEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable automatic trimming counter.
  * @rmtoll
  *  CR           AUTOTRIMEN    LL_CRS_EnableAutoTrimming
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_EnableAutoTrimming(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_AUTOTRIMEN);
}

/**
  * @brief  Disable automatic trimming counter.
  * @rmtoll
  *  CR           AUTOTRIMEN    LL_CRS_DisableAutoTrimming
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableAutoTrimming(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_AUTOTRIMEN);
}

/**
  * @brief  Check whether automatic trimming is enabled.
  * @rmtoll
  *  CR           AUTOTRIMEN    LL_CRS_IsEnabledAutoTrimming
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledAutoTrimming(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_AUTOTRIMEN) == (CRS_CR_AUTOTRIMEN)) ? 1UL : 0UL);
}

/**
  * @brief   Set HSI144 oscillator smooth trimming.
  * @rmtoll
  *  CR           TRIM          LL_CRS_SetHSI144SmoothTrimming
  * @param   crsx CRS instance
  * @param   value A number between Min_Data = 0 and Max_Data = 0x5FU.
  * @note    Default value can be set thanks to @ref LL_CRS_HSI144CALIBRATION_DEFAULT.
  * @warning When the AUTOTRIMEN bit is set, this field is controlled by hardware and is read-only.
  */
__STATIC_INLINE void LL_CRS_SetHSI144SmoothTrimming(CRS_TypeDef *crsx, uint32_t value)
{
  STM32_MODIFY_REG(crsx->CR, CRS_CR_TRIM, value << CRS_CR_TRIM_Pos);
}

/**
  * @brief  Get HSI144 oscillator smooth trimming.
  * @rmtoll
  *  CR           TRIM          LL_CRS_GetHSI144SmoothTrimming
  * @param  crsx CRS instance.
  * @retval A number between Min_Data = 0 and Max_Data = 0x5FU.
  */
__STATIC_INLINE uint32_t LL_CRS_GetHSI144SmoothTrimming(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CR, CRS_CR_TRIM) >> CRS_CR_TRIM_Pos);
}

/**
  * @brief  Set counter reload value.
  * @rmtoll
  *  CFGR         RELOAD        LL_CRS_SetReloadCounter
  * @param  crsx CRS instance.
  * @param  value A number between Min_Data = 0 and Max_Data = 0xFFFF.
  * @note   Default value can be set thanks to @ref LL_CRS_RELOADVALUE_DEFAULT.
  *         Otherwise, it can be calculated using the macro @ref LL_CRS_CALCULATE_RELOAD(_FTARGET_, _FSYNC_).
  */
__STATIC_INLINE void LL_CRS_SetReloadCounter(CRS_TypeDef *crsx, uint32_t value)
{
  STM32_MODIFY_REG(crsx->CFGR, CRS_CFGR_RELOAD, value);
}

/**
  * @brief  Get counter reload value.
  * @rmtoll
  *  CFGR         RELOAD        LL_CRS_GetReloadCounter
  * @param  crsx CRS instance.
  * @retval A number between Min_Data = 0 and Max_Data = 0xFFFF.
  */
__STATIC_INLINE uint32_t LL_CRS_GetReloadCounter(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CFGR, CRS_CFGR_RELOAD));
}

/**
  * @brief  Set frequency error limit.
  * @rmtoll
  *  CFGR         FELIM         LL_CRS_SetFreqErrorLimit
  * @param  crsx CRS instance.
  * @param  value A number between Min_Data = 0 and Max_Data = 255.
  * @note   Default value can be set thanks to @ref LL_CRS_ERRORLIMIT_DEFAULT.
  */
__STATIC_INLINE void LL_CRS_SetFreqErrorLimit(CRS_TypeDef *crsx, uint32_t value)
{
  STM32_MODIFY_REG(crsx->CFGR, CRS_CFGR_FELIM, value << CRS_CFGR_FELIM_Pos);
}

/**
  * @brief  Get frequency error limit.
  * @rmtoll
  *  CFGR         FELIM         LL_CRS_GetFreqErrorLimit
  * @param  crsx CRS instance.
  * @retval A number between Min_Data = 0 and Max_Data = 255.
  */
__STATIC_INLINE uint32_t LL_CRS_GetFreqErrorLimit(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CFGR, CRS_CFGR_FELIM) >> CRS_CFGR_FELIM_Pos);
}

/**
  * @brief  Set division factor for SYNC signal.
  * @rmtoll
  *  CFGR         SYNCDIV       LL_CRS_SetSyncDivider
  * @param  crsx CRS instance.
  * @param  divider This parameter can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_DIV_1
  *         @arg @ref LL_CRS_SYNC_DIV_2
  *         @arg @ref LL_CRS_SYNC_DIV_4
  *         @arg @ref LL_CRS_SYNC_DIV_8
  *         @arg @ref LL_CRS_SYNC_DIV_16
  *         @arg @ref LL_CRS_SYNC_DIV_32
  *         @arg @ref LL_CRS_SYNC_DIV_64
  *         @arg @ref LL_CRS_SYNC_DIV_128
  */
__STATIC_INLINE void LL_CRS_SetSyncDivider(CRS_TypeDef *crsx, uint32_t divider)
{
  STM32_MODIFY_REG(crsx->CFGR, CRS_CFGR_SYNCDIV, divider);
}

/**
  * @brief  Get division factor for SYNC signal.
  * @rmtoll
  *  CFGR         SYNCDIV       LL_CRS_GetSyncDivider
  * @param  crsx CRS instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_DIV_1
  *         @arg @ref LL_CRS_SYNC_DIV_2
  *         @arg @ref LL_CRS_SYNC_DIV_4
  *         @arg @ref LL_CRS_SYNC_DIV_8
  *         @arg @ref LL_CRS_SYNC_DIV_16
  *         @arg @ref LL_CRS_SYNC_DIV_32
  *         @arg @ref LL_CRS_SYNC_DIV_64
  *         @arg @ref LL_CRS_SYNC_DIV_128
  */
__STATIC_INLINE uint32_t LL_CRS_GetSyncDivider(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CFGR, CRS_CFGR_SYNCDIV));
}

/**
  * @brief  Set SYNC signal source.
  * @rmtoll
  *  CFGR         SYNCSRC       LL_CRS_SetSyncSignalSource
  * @param  crsx CRS instance.
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_SOURCE_GPIO
  *         @arg @ref LL_CRS_SYNC_SOURCE_LSE
  *         @arg @ref LL_CRS_SYNC_SOURCE_USB
  *         @arg @ref LL_CRS_SYNC_SOURCE_HSE_1MHZ
  */
__STATIC_INLINE void LL_CRS_SetSyncSignalSource(CRS_TypeDef *crsx, uint32_t source)
{
  STM32_MODIFY_REG(crsx->CFGR, CRS_CFGR_SYNCSRC, source);
}

/**
  * @brief  Get SYNC signal source.
  * @rmtoll
  *  CFGR         SYNCSRC       LL_CRS_GetSyncSignalSource
  * @param  crsx CRS instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_SOURCE_GPIO
  *         @arg @ref LL_CRS_SYNC_SOURCE_LSE
  *         @arg @ref LL_CRS_SYNC_SOURCE_USB
  *         @arg @ref LL_CRS_SYNC_SOURCE_HSE_1MHZ
  */
__STATIC_INLINE uint32_t LL_CRS_GetSyncSignalSource(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CFGR, CRS_CFGR_SYNCSRC));
}

/**
  * @brief  Set input polarity for the SYNC signal source.
  * @rmtoll
  *  CFGR         SYNCPOL       LL_CRS_SetSyncPolarity
  * @param  crsx CRS instance.
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_POLARITY_RISING
  *         @arg @ref LL_CRS_SYNC_POLARITY_FALLING
  */
__STATIC_INLINE void LL_CRS_SetSyncPolarity(CRS_TypeDef *crsx, uint32_t polarity)
{
  STM32_MODIFY_REG(crsx->CFGR, CRS_CFGR_SYNCPOL, polarity);
}

/**
  * @brief  Get input polarity for the SYNC signal source.
  * @rmtoll
  *  CFGR         SYNCPOL       LL_CRS_GetSyncPolarity
  * @param  crsx CRS instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRS_SYNC_POLARITY_RISING
  *         @arg @ref LL_CRS_SYNC_POLARITY_FALLING
  */
__STATIC_INLINE uint32_t LL_CRS_GetSyncPolarity(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->CFGR, CRS_CFGR_SYNCPOL));
}

/**
  * @brief  Configure CRS for trimming.
  * @rmtoll
  *  CR           TRIM          LL_CRS_ConfigTrimming \n
  *  CR           AUTOTRIMEN    LL_CRS_ConfigTrimming
  * @param   crsx CRS instance.
  * @param   trimming A number between Min_Data = 0 and Max_Data = 0x5FU.
  * @param   auto_trimming LL_CRS_AUTO_TRIMMING_DISABLE to disable auto trimming.
                          LL_CRS_AUTO_TRIMMING_ENABLE to enable auto trimming.
  * @warning When the auto trimming is enabled, the trimming is controlled by hardware and is read-only.
  */
__STATIC_INLINE void LL_CRS_ConfigTrimming(CRS_TypeDef *crsx, uint32_t trimming, uint32_t auto_trimming)
{
  STM32_WRITE_REG(crsx->CR, ((trimming << CRS_CR_TRIM_Pos) | auto_trimming));
}

/**
  * @brief  Configure CRS for the synchronization.
  * @rmtoll
  *  CFGR         RELOAD        LL_CRS_ConfigSynchronization \n
  *  CFGR         FELIM         LL_CRS_ConfigSynchronization \n
  *  CFGR         SYNCDIV       LL_CRS_ConfigSynchronization \n
  *  CFGR         SYNCSRC       LL_CRS_ConfigSynchronization \n
  *  CFGR         SYNCPOL       LL_CRS_ConfigSynchronization
  * @param  crsx CRS instance.
  * @param  settings This parameter must be a combination of the following values:
  *         @arg @ref LL_CRS_SYNC_DIV_1 or @ref LL_CRS_SYNC_DIV_2 or @ref LL_CRS_SYNC_DIV_4
  *              or @ref LL_CRS_SYNC_DIV_8 or @ref LL_CRS_SYNC_DIV_16 or @ref LL_CRS_SYNC_DIV_32
  *              or @ref LL_CRS_SYNC_DIV_64 or @ref LL_CRS_SYNC_DIV_128
  *         @arg @ref LL_CRS_SYNC_SOURCE_GPIO or @ref LL_CRS_SYNC_SOURCE_LSE
  *              or @ref LL_CRS_SYNC_SOURCE_USB or @ref LL_CRS_SYNC_SOURCE_HSE_1MHZ
  *         @arg @ref LL_CRS_SYNC_POLARITY_RISING or @ref LL_CRS_SYNC_POLARITY_FALLING
  * @param  reload A number between Min_Data = 0 and Max_Data = 255.
  * @param  frequency_error_limit A number between Min_Data = 0 and Max_Data = 0xFFFF.
  */
__STATIC_INLINE void LL_CRS_ConfigSynchronization(CRS_TypeDef *crsx, uint32_t settings, uint32_t reload,
                                                  uint32_t frequency_error_limit)
{
  STM32_MODIFY_REG(crsx->CFGR,
                   CRS_CFGR_SYNCDIV | CRS_CFGR_SYNCSRC | CRS_CFGR_SYNCPOL | CRS_CFGR_RELOAD | CRS_CFGR_FELIM,
                   settings | reload | (frequency_error_limit << CRS_CFGR_FELIM_Pos));
}

/**
  * @}
  */

/** @defgroup CRS_LL_EF_CRS_Management CRS_Management
  * @{
  */

/**
  * @brief  Generate software SYNC event.
  * @rmtoll
  *  CR           SWSYNC        LL_CRS_GenerateEvent_SWSYNC
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_GenerateEvent_SWSYNC(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_SWSYNC);
}

/**
  * @brief  Get the frequency error direction latched at the time of the last
  *         SYNC event.
  * @rmtoll
  *  ISR          FEDIR         LL_CRS_GetFreqErrorDirection
  * @param  crsx CRS instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRS_FREQ_ERROR_DIR_UP
  *         @arg @ref LL_CRS_FREQ_ERROR_DIR_DOWN
  */
__STATIC_INLINE uint32_t LL_CRS_GetFreqErrorDirection(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->ISR, CRS_ISR_FEDIR));
}

/**
  * @brief  Get the frequency error counter value latched at the time of the last SYNC event.
  * @rmtoll
  *  ISR          FECAP         LL_CRS_GetFreqErrorCapture
  * @param  crsx CRS instance.
  * @retval A number between Min_Data = 0x0000 and Max_Data = 0xFFFF.
  */
__STATIC_INLINE uint32_t LL_CRS_GetFreqErrorCapture(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_BIT(crsx->ISR, CRS_ISR_FECAP) >> CRS_ISR_FECAP_Pos);
}

/**
  * @brief  Get the frequency error counter value and error direction latched at the time of the last SYNC event.
  * @rmtoll
  *  ISR          FEDIR         LL_CRS_GetFreqErrorInfo \n
  *  ISR          FECAP         LL_CRS_GetFreqErrorInfo
  * @param  crsx CRS instance.
  * @retval A number between Min_Data = 0x0000 and Max_Data = 0x1FFFF.
  */
__STATIC_INLINE uint32_t LL_CRS_GetFreqErrorInfo(const CRS_TypeDef *crsx)
{
  return (uint32_t)(STM32_READ_REG(crsx->ISR) & (CRS_ISR_FECAP | CRS_ISR_FEDIR));
}

/**
  * @}
  */

/** @defgroup CRS_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get CRS flag(s).
  * @rmtoll
  *  ISR      SYNCOKF           LL_CRS_IsActiveFlag \n
  *  ISR      SYNCWARNF         LL_CRS_IsActiveFlag \n
  *  ISR      ERRF              LL_CRS_IsActiveFlag \n
  *  ISR      ESYNCF            LL_CRS_IsActiveFlag \n
  *  ISR      SYNCERR           LL_CRS_IsActiveFlag \n
  *  ISR      SYNCMISS          LL_CRS_IsActiveFlag \n
  *  ISR      TRIMOVF           LL_CRS_IsActiveFlag
  * @param  crsx CRS instance.
  * @param  mask This parameter can be a combination of the following values:
  *            @arg LL_CRS_ISR_SYNCOKF
  *            @arg LL_CRS_ISR_SYNCWARNF
  *            @arg LL_CRS_ISR_ERRF
  *            @arg LL_CRS_ISR_ESYNCF
  *            @arg LL_CRS_ISR_SYNCERR
  *            @arg LL_CRS_ISR_SYNCMISS
  *            @arg LL_CRS_ISR_TRIMOVF
  * @retval Return 1 if at least one flag is set; otherwise, 0.
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag(const CRS_TypeDef *crsx, uint32_t mask)
{
  return ((STM32_READ_BIT(crsx->ISR, mask) != 0UL) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the SYNC event OK signal occurred.
  * @rmtoll
  *  ISR          SYNCOKF       LL_CRS_IsActiveFlag_SYNCOK
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_SYNCOK(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_SYNCOKF) == (CRS_ISR_SYNCOKF)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the SYNC warning signal occurred.
  * @rmtoll
  *  ISR          SYNCWARNF     LL_CRS_IsActiveFlag_SYNCWARN
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_SYNCWARN(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_SYNCWARNF) == (CRS_ISR_SYNCWARNF)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the synchronization or trimming error signal occurred.
  * @rmtoll
  *  ISR          ERRF          LL_CRS_IsActiveFlag_ERR
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_ERR(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_ERRF) == (CRS_ISR_ERRF)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the expected SYNC signal occurred.
  * @rmtoll
  *  ISR          ESYNCF        LL_CRS_IsActiveFlag_ESYNC
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_ESYNC(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_ESYNCF) == (CRS_ISR_ESYNCF)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the SYNC error signal occurred.
  * @rmtoll
  *  ISR          SYNCERR       LL_CRS_IsActiveFlag_SYNCERR
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_SYNCERR(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_SYNCERR) == (CRS_ISR_SYNCERR)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the SYNC missed error signal occurred.
  * @rmtoll
  *  ISR          SYNCMISS      LL_CRS_IsActiveFlag_SYNCMISS
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_SYNCMISS(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_SYNCMISS) == (CRS_ISR_SYNCMISS)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether trimming overflow or underflow occurred.
  * @rmtoll
  *  ISR          TRIMOVF       LL_CRS_IsActiveFlag_TRIMOVF
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsActiveFlag_TRIMOVF(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->ISR, CRS_ISR_TRIMOVF) == (CRS_ISR_TRIMOVF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear CRS flag(s).
  * @rmtoll
  *  ICR          ERRC          LL_CRS_ClearFlag \n
  *  ICR          SYNCWARNC     LL_CRS_ClearFlag \n
  *  ICR          SYNCOKC       LL_CRS_ClearFlag \n
  *  ICR          ESYNCC        LL_CRS_ClearFlag
  * @param  crsx CRS instance.
  * @param  mask Specifies the CRS flags to be cleared.
  *         This parameter can be a combination of the following values:
  *            @arg LL_CRS_ICR_SYNCOKC
  *            @arg LL_CRS_ICR_SYNCWARNC
  *            @arg LL_CRS_ICR_ERRC
  *            @arg LL_CRS_ICR_ESYNCC
  */
__STATIC_INLINE void LL_CRS_ClearFlag(CRS_TypeDef *crsx, uint32_t mask)
{
  STM32_WRITE_REG(crsx->ICR, mask);
}

/**
  * @brief  Clear the SYNC event OK flag.
  * @rmtoll
  *  ICR          SYNCOKC       LL_CRS_ClearFlag_SYNCOK
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_ClearFlag_SYNCOK(CRS_TypeDef *crsx)
{
  STM32_WRITE_REG(crsx->ICR, CRS_ICR_SYNCOKC);
}

/**
  * @brief  Clear the SYNC warning flag.
  * @rmtoll
  *  ICR          SYNCWARNC     LL_CRS_ClearFlag_SYNCWARN
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_ClearFlag_SYNCWARN(CRS_TypeDef *crsx)
{
  STM32_WRITE_REG(crsx->ICR, CRS_ICR_SYNCWARNC);
}

/**
  * @brief  Clear TRIMOVF, SYNCMISS and SYNCERR bits and consequently also
  *         the ERR flag.
  * @rmtoll
  *  ICR          ERRC          LL_CRS_ClearFlag_ERR
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_ClearFlag_ERR(CRS_TypeDef *crsx)
{
  STM32_WRITE_REG(crsx->ICR, CRS_ICR_ERRC);
}

/**
  * @brief  Clear expected SYNC flag.
  * @rmtoll
  *  ICR          ESYNCC        LL_CRS_ClearFlag_ESYNC
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_ClearFlag_ESYNC(CRS_TypeDef *crsx)
{
  STM32_WRITE_REG(crsx->ICR, CRS_ICR_ESYNCC);
}

/**
  * @}
  */

/** @defgroup CRS_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable interrupt(s).
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_EnableIT \n
  *  CR           SYNCWARNIE    LL_CRS_EnableIT \n
  *  CR           ERRIE         LL_CRS_EnableIT \n
  *  CR           ESYNCIE       LL_CRS_EnableIT
  * @param  crsx CRS instance.
  * @param  mask This parameter can be a combination of the following values:
  *            @arg LL_CRS_CR_SYNCOKIE
  *            @arg LL_CRS_CR_SYNCWARNIE
  *            @arg LL_CRS_CR_ERRIE
  *            @arg LL_CRS_CR_ESYNCIE
  */
__STATIC_INLINE void LL_CRS_EnableIT(CRS_TypeDef *crsx, uint32_t mask)
{
  STM32_SET_BIT(crsx->CR, mask);
}

/**
  * @brief  Disable interrupt(s).
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_DisableIT \n
  *  CR           SYNCWARNIE    LL_CRS_DisableIT \n
  *  CR           ERRIE         LL_CRS_DisableIT \n
  *  CR           ESYNCIE       LL_CRS_DisableIT
  * @param  crsx CRS instance.
  * @param  mask This parameter can be a combination of the following values:
  *            @arg LL_CRS_CR_SYNCOKIE
  *            @arg LL_CRS_CR_SYNCWARNIE
  *            @arg LL_CRS_CR_ERRIE
  *            @arg LL_CRS_CR_ESYNCIE
  */
__STATIC_INLINE void LL_CRS_DisableIT(CRS_TypeDef *crsx, uint32_t mask)
{
  STM32_CLEAR_BIT(crsx->CR, mask);
}

/**
  * @brief  Indicate whether the interrupt(s) are enabled.
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_IsEnabledIT \n
  *  CR           SYNCWARNIE    LL_CRS_IsEnabledIT \n
  *  CR           ERRIE         LL_CRS_IsEnabledIT \n
  *  CR           ESYNCIE       LL_CRS_IsEnabledIT
  * @param  crsx CRS instance.
  * @param  mask This parameter can be a combination of the following values:
  *            @arg LL_CRS_CR_SYNCOKIE
  *            @arg LL_CRS_CR_SYNCWARNIE
  *            @arg LL_CRS_CR_ERRIE
  *            @arg LL_CRS_CR_ESYNCIE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledIT(const CRS_TypeDef *crsx, uint32_t mask)
{
  return ((STM32_READ_BIT(crsx->CR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SYNC event OK interrupt.
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_EnableIT_SYNCOK
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_EnableIT_SYNCOK(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_SYNCOKIE);
}

/**
  * @brief  Disable SYNC event OK interrupt.
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_DisableIT_SYNCOK
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableIT_SYNCOK(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_SYNCOKIE);
}

/**
  * @brief  Check whether the SYNC event OK interrupt is enabled.
  * @rmtoll
  *  CR           SYNCOKIE      LL_CRS_IsEnabledIT_SYNCOK
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledIT_SYNCOK(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_SYNCOKIE) == (CRS_CR_SYNCOKIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SYNC warning interrupt.
  * @rmtoll
  *  CR           SYNCWARNIE    LL_CRS_EnableIT_SYNCWARN
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_EnableIT_SYNCWARN(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_SYNCWARNIE);
}

/**
  * @brief  Disable SYNC warning interrupt.
  * @rmtoll
  *  CR           SYNCWARNIE    LL_CRS_DisableIT_SYNCWARN
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableIT_SYNCWARN(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_SYNCWARNIE);
}

/**
  * @brief  Check whether the SYNC warning interrupt is enabled.
  * @rmtoll
  *  CR           SYNCWARNIE    LL_CRS_IsEnabledIT_SYNCWARN
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledIT_SYNCWARN(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_SYNCWARNIE) == (CRS_CR_SYNCWARNIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable synchronization or trimming error interrupt.
  * @rmtoll
  *  CR           ERRIE         LL_CRS_EnableIT_ERR
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_EnableIT_ERR(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_ERRIE);
}

/**
  * @brief  Disable synchronization or trimming error interrupt.
  * @rmtoll
  *  CR           ERRIE         LL_CRS_DisableIT_ERR
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableIT_ERR(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_ERRIE);
}

/**
  * @brief  Check whether the synchronization or trimming error interrupt is enabled.
  * @rmtoll
  *  CR           ERRIE         LL_CRS_IsEnabledIT_ERR
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledIT_ERR(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_ERRIE) == (CRS_CR_ERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable expected SYNC interrupt.
  * @rmtoll
  *  CR           ESYNCIE       LL_CRS_EnableIT_ESYNC
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_EnableIT_ESYNC(CRS_TypeDef *crsx)
{
  STM32_SET_BIT(crsx->CR, CRS_CR_ESYNCIE);
}

/**
  * @brief  Disable expected SYNC interrupt.
  * @rmtoll
  *  CR           ESYNCIE       LL_CRS_DisableIT_ESYNC
  * @param  crsx CRS instance.
  */
__STATIC_INLINE void LL_CRS_DisableIT_ESYNC(CRS_TypeDef *crsx)
{
  STM32_CLEAR_BIT(crsx->CR, CRS_CR_ESYNCIE);
}

/**
  * @brief  Check whether the expected SYNC interrupt is enabled.
  * @rmtoll
  *  CR           ESYNCIE       LL_CRS_IsEnabledIT_ESYNC
  * @param  crsx CRS instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CRS_IsEnabledIT_ESYNC(const CRS_TypeDef *crsx)
{
  return ((STM32_READ_BIT(crsx->CR, CRS_CR_ESYNCIE) == (CRS_CR_ESYNCIE)) ? 1UL : 0UL);
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

#endif /* CRS */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_CRS_H */
