/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_crs.h
  * @brief   Header file of CRS HAL module.
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
#ifndef STM32C5XX_HAL_CRS_H
#define STM32C5XX_HAL_CRS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_crs.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(CRS)
/** @defgroup CRS CRS
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_Exported_Constants HAL CRS Constants
  * @{
  */

#if defined(USE_HAL_CRS_GET_LAST_ERRORS) && (USE_HAL_CRS_GET_LAST_ERRORS == 1U)
/** @defgroup CRS_Error_Code Error Code
  * @{
  */
#define HAL_CRS_ERROR_NONE          0U              /*!< No error */
#define HAL_CRS_ERROR_SYNC_ERROR    (1U << 0U)      /*!< Frequency error too large (internal frequency too low) */
#define HAL_CRS_ERROR_SYNC_MISSED   (1U << 1U)      /*!< Synchronization pulse missed or frequency error too large
                                                         (internal frequency too high) */
#define HAL_CRS_ERROR_TRIMMING      (1U << 2U)      /*!< Automatic trimming overflows or underflows the trimming value */
#define HAL_CRS_ERROR_EXPECTED_SYNC (1U << 3U)      /*!< Frequency error counter reached a zero value */
#define HAL_CRS_ERROR_SYNC_WARN     (1U << 4U)      /*!< Synchronization warning */
/**
  * @}
  */
#endif /* USE_HAL_CRS_GET_LAST_ERRORS */

/** @defgroup CRS_Reload_Default_Value Reload Default Value
  * @{
  */
#define HAL_CRS_RELOAD_DEFAULT_VALUE    LL_CRS_RELOADVALUE_DEFAULT /*!< The reset value of the RELOAD field corresponds
                                                                        to a target frequency of 48 MHz and a
                                                                        synchronization signal frequency of 1 kHz
                                                                        (SOF signal from USB) */

/**
  * @}
  */

/** @defgroup CRS_ErrorLimit_Default_Value Error Default Value
  * @{
  */
#define HAL_CRS_ERRORLIMIT_DEFAULT_VALUE      LL_CRS_ERRORLIMIT_DEFAULT        /*!< Default frequency error limit. */
/**
  * @}
  */

/** @defgroup CRS_Trimming_Default_Value Trimming Default Value
  * @{
  */
#define HAL_CRS_TRIMMING_DEFAULT_VALUE  LL_CRS_HSI144CALIBRATION_DEFAULT /*!< The default value is 0x30U, which
                                                                             corresponds to the middle of the trimming
                                                                             interval. The trimming step is around
                                                                             67 kHz between two consecutive TRIM steps.
                                                                             A higher TRIM corresponds to a higher
                                                                             output frequency */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_Exported_Macros HAL CRS Macros
  * @{
  */

/**
  * @brief  Macro to calculate the reload value to be set in the CRS configuration register according to the target
  *         and synchronization frequencies.
  * @param  ftarget Target frequency (value in Hz).
  * @param  fsync Synchronization signal frequency (value in Hz).
  * @note   The RELOAD value must be selected according to the ratio between the target frequency and the frequency
  *         of the synchronization source after prescaling. It is then decreased by one in order to
  *         reach the expected synchronization on the zero value. The formula is as follows:
  *         RELOAD = (ftarget / fsync) -1.
  * @retval uint32_t Reload value
  */
#define HAL_CRS_CALCULATE_RELOAD(ftarget, fsync) LL_CRS_CALCULATE_RELOAD(ftarget, fsync)
/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup  CRS_Exported_Types HAL CRS Types
  * @{
  */

/** @defgroup CRS_Exported_Types_Group1 Enumerations
  * @{
  */

/**
  * @brief HAL CRS auto trimming status definitions.
  */
typedef enum
{
  HAL_CRS_AUTO_TRIMMING_DISABLED = 0U,               /*!< Auto trimming is disabled */
  HAL_CRS_AUTO_TRIMMING_ENABLED  = 1U                /*!< Auto trimming is enabled */
} hal_crs_auto_trimming_status_t;

/**
  * @brief HAL CRS auto trimming state definitions.
  */
typedef enum
{
  HAL_CRS_AUTO_TRIMMING_DISABLE = LL_CRS_AUTO_TRIMMING_DISABLE,  /*!< Auto trimming disabled (default) */
  HAL_CRS_AUTO_TRIMMING_ENABLE  = LL_CRS_AUTO_TRIMMING_ENABLE    /*!< Auto trimming enabled */
} hal_crs_auto_trimming_state_t;

/**
  * @brief HAL CRS synchronization source definitions.
  */
typedef enum
{
  HAL_CRS_SYNC_SOURCE_GPIO     = LL_CRS_SYNC_SOURCE_GPIO,     /*!< Synchronization signal source GPIO */
  HAL_CRS_SYNC_SOURCE_LSE      = LL_CRS_SYNC_SOURCE_LSE,      /*!< Synchronization signal source LSE */
  HAL_CRS_SYNC_SOURCE_USB      = LL_CRS_SYNC_SOURCE_USB,      /*!< Synchronization signal source USB SOF (default)    */
  HAL_CRS_SYNC_SOURCE_HSE_1MHZ = LL_CRS_SYNC_SOURCE_HSE_1MHZ  /*!< Synchronization signal source HSE 1MHz             */
} hal_crs_sync_source_t;

/**
  * @brief HAL CRS synchronization divider definitions.
  */
typedef enum
{
  HAL_CRS_SYNC_DIV1   = LL_CRS_SYNC_DIV_1,   /*!< Synchronization signal not divided (default) */
  HAL_CRS_SYNC_DIV2   = LL_CRS_SYNC_DIV_2,   /*!< Synchronization signal divided by 2 */
  HAL_CRS_SYNC_DIV4   = LL_CRS_SYNC_DIV_4,   /*!< Synchronization signal divided by 4 */
  HAL_CRS_SYNC_DIV8   = LL_CRS_SYNC_DIV_8,   /*!< Synchronization signal divided by 8 */
  HAL_CRS_SYNC_DIV16  = LL_CRS_SYNC_DIV_16,  /*!< Synchronization signal divided by 16 */
  HAL_CRS_SYNC_DIV32  = LL_CRS_SYNC_DIV_32,  /*!< Synchronization signal divided by 32 */
  HAL_CRS_SYNC_DIV64  = LL_CRS_SYNC_DIV_64,  /*!< Synchronization signal divided by 64 */
  HAL_CRS_SYNC_DIV128 = LL_CRS_SYNC_DIV_128  /*!< Synchronization signal divided by 128 */
} hal_crs_sync_div_t;

/**
  * @brief HAL CRS synchronization polarity definitions.
  */
typedef enum
{
  HAL_CRS_SYNC_POLARITY_RISING  = LL_CRS_SYNC_POLARITY_RISING,  /*!< Synchronization active on rising edge (default) */
  HAL_CRS_SYNC_POLARITY_FALLING = LL_CRS_SYNC_POLARITY_FALLING  /*!< Synchronization active on falling edge */
} hal_crs_sync_polarity_t;

/**
  * @brief HAL CRS frequency error direction definitions.
  */
typedef enum
{
  HAL_CRS_FREQUENCY_ERROR_DIR_UP   = LL_CRS_FREQ_ERROR_DIR_UP,   /*!< Upcounting direction, the actual frequency
                                                                      is above the target */
  HAL_CRS_FREQUENCY_ERROR_DIR_DOWN = LL_CRS_FREQ_ERROR_DIR_DOWN  /*!< Downcounting direction, the actual frequency
                                                                      is below the target */
} hal_crs_frequency_error_dir_t;

/**
  * @brief HAL CRS state definition.
  */
typedef enum
{
  HAL_CRS_STATE_RESET        = 0U,           /*!< CRS driver not initialized and not started */
  HAL_CRS_STATE_IDLE         = (1U << 31U),  /*!< CRS driver initialized and not started */
  HAL_CRS_STATE_ACTIVE       = (1U << 30U),  /*!< CRS driver initialized and started */
} hal_crs_state_t;

/**
  * @brief HAL CRS instances definition.
  */
typedef enum
{
  HAL_CRS1 = CRS_BASE        /*!< Instance CRS */
} hal_crs_t;
/**
  * @}
  */

/** @defgroup CRS_Exported_Types_Group2 Handle Structure
  * @{
  */
typedef struct hal_crs_handle_s hal_crs_handle_t; /*!< CRS handle type definition */

#if defined(USE_HAL_CRS_REGISTER_CALLBACKS) && (USE_HAL_CRS_REGISTER_CALLBACKS == 1U)
typedef  void (*hal_crs_cb_t)(hal_crs_handle_t *hcrs); /*!< Pointer to a CRS callback function */
#endif /* USE_HAL_CRS_REGISTER_CALLBACKS */

/**
  * @brief CRS handle structure definition.
  */
struct hal_crs_handle_s
{
  /*! Peripheral instance */
  hal_crs_t instance;

  /*! CRS global state */
  volatile hal_crs_state_t global_state;

#if defined(USE_HAL_CRS_GET_LAST_ERRORS) && (USE_HAL_CRS_GET_LAST_ERRORS == 1U)
  /*! Variable storing the cumulative last errors */
  volatile uint32_t last_error_codes;
#endif /* USE_HAL_CRS_GET_LAST_ERRORS */

#if defined(USE_HAL_CRS_REGISTER_CALLBACKS) && (USE_HAL_CRS_REGISTER_CALLBACKS == 1U)
  /*! Error user callback (disabled if the switch USE_HAL_CRS_REGISTER_CALLBACKS is set to 0U) */
  hal_crs_cb_t p_error_cb;
  /*! Synchronization OK user callback (disabled if the switch USE_HAL_CRS_REGISTER_CALLBACKS is set to 0U) */
  hal_crs_cb_t p_sync_ok_cb;
  /*! Synchronization warning user callback (disabled if the switch USE_HAL_CRS_REGISTER_CALLBACKS is set to 0U) */
  hal_crs_cb_t p_sync_warn_cb;
  /*! Expected synchronization user callback (disabled if the switch USE_HAL_CRS_REGISTER_CALLBACKS is set to 0U) */
  hal_crs_cb_t p_expected_sync_cb;
#endif /* USE_HAL_CRS_REGISTER_CALLBACKS */

#if defined(USE_HAL_CRS_USER_DATA) && (USE_HAL_CRS_USER_DATA == 1U)
  /*! CRS user data */
  const void *p_user_data;
#endif /* USE_HAL_CRS_USER_DATA */
};

/**
  * @}
  */

/** @defgroup CRS_Exported_Types_Group3 Configuration Structure
  * @{
  */

/**
  * @brief HAL CRS configuration structure definition.
  */
typedef struct
{
  hal_crs_sync_div_t divider;                  /*!< Specifies the division factor of the SYNC signal. */

  hal_crs_sync_source_t source;                /*!< Specifies the SYNC signal source. */

  hal_crs_sync_polarity_t polarity;            /*!< Specifies the input polarity for the SYNC signal source. */

  uint32_t reload;                             /*!< Specifies the value to be loaded in the frequency error counter
                                                    with each SYNC event. It can be calculated using the macro
                                                    HAL_CRS_CALCULATE_RELOAD(ftarget, fsync). This parameter must be a
                                                    number between 0 and 0xFFFF or
                                                    @ref HAL_CRS_RELOAD_DEFAULT_VALUE. */

  uint32_t frequency_error_limit;              /*!< Specifies the value to be used to evaluate the captured frequency
                                                    error value. This parameter must be a number between 0 and 0xFF or
                                                    @ref HAL_CRS_ERRORLIMIT_DEFAULT_VALUE. */
  uint32_t trimming;                           /*!< Specifies a user-programmable trimming value to the HSI144
                                                    oscillator. This parameter must be a number between 0 and
                                                    0x5FU or @ref HAL_CRS_TRIMMING_DEFAULT_VALUE. */
  hal_crs_auto_trimming_state_t auto_trimming; /*!< Specifies the auto trimming enable or disable. */
} hal_crs_config_t;
/**
  * @}
  */

/** @defgroup CRS_Exported_Types_Group4 Synchronization Structure
  * @{
  */

/**
  * @brief HAL CRS synchronization structure definition.
  */
typedef struct
{
  uint32_t frequency_error_capture;                   /*!< Specifies the frequency error counter value latched
                                                           at the time of the last SYNC event.
                                                           This parameter must be a number between 0 and 0xFFFF. */

  hal_crs_frequency_error_dir_t frequency_error_dir; /*!< Specifies the counting direction of
                                                          the frequency error counter latched at the time of
                                                          the last SYNC event.
                                                          It shows whether the actual frequency is below or
                                                          above the target. */
} hal_crs_frequency_error_info_t;
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup CRS_Exported_Functions HAL CRS Functions
  * @{
  */

/** @defgroup CRS_Exported_Functions_Group1 Initialization and DeInitialization functions
  * @{
  */
hal_status_t HAL_CRS_Init(hal_crs_handle_t *hcrs, hal_crs_t instance);
void HAL_CRS_DeInit(hal_crs_handle_t *hcrs);
/**
  * @}
  */

/** @defgroup CRS_Exported_Functions_Group2 Configuration functions
  * @{
  */
hal_status_t HAL_CRS_SetConfig(const hal_crs_handle_t *hcrs, const hal_crs_config_t *p_config);
void HAL_CRS_GetConfig(const hal_crs_handle_t *hcrs, hal_crs_config_t *p_config);
void HAL_CRS_ResetConfig(const hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_SetTrimming(const hal_crs_handle_t *hcrs, uint32_t trimming);
uint32_t HAL_CRS_GetTrimming(const hal_crs_handle_t *hcrs);
void HAL_CRS_GetFrequencyErrorInfo(const hal_crs_handle_t *hcrs,
                                   hal_crs_frequency_error_info_t *p_frequency_error_info);
/**
  * @}
  */

/** @defgroup CRS_Exported_Functions_Group3 Control functions
  * @{
  */
hal_status_t HAL_CRS_EnableAutoTrimming(hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_DisableAutoTrimming(hal_crs_handle_t *hcrs);
hal_crs_auto_trimming_status_t HAL_CRS_IsEnabledAutoTrimming(const hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_StartSync(hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_StopSync(hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_StartSync_IT(hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_StopSync_IT(hal_crs_handle_t *hcrs);
/**
  * @}
  */

/** @defgroup CRS_Exported_Functions_Group4 Process functions
  * @{
  */
hal_status_t HAL_CRS_GenerateSoftwareSync(hal_crs_handle_t *hcrs);
hal_status_t HAL_CRS_PollForSync(hal_crs_handle_t *hcrs, uint32_t timeout_ms);
/**
  * @}
  */

/** @defgroup CRS_Exported_Functions_Group5 IRQHandler and Callbacks functions
  * @{
  */
void HAL_CRS_IRQHandler(hal_crs_handle_t *hcrs);
void HAL_CRS_SyncOkCallback(hal_crs_handle_t *hcrs);
void HAL_CRS_SyncWarnCallback(hal_crs_handle_t *hcrs);
void HAL_CRS_ExpectedSyncCallback(hal_crs_handle_t *hcrs);
void HAL_CRS_ErrorCallback(hal_crs_handle_t *hcrs);

#if defined(USE_HAL_CRS_REGISTER_CALLBACKS) && (USE_HAL_CRS_REGISTER_CALLBACKS == 1U)
hal_status_t HAL_CRS_RegisterSyncOkCallback(hal_crs_handle_t *hcrs, hal_crs_cb_t p_callback);
hal_status_t HAL_CRS_RegisterSyncWarnCallback(hal_crs_handle_t *hcrs, hal_crs_cb_t p_callback);
hal_status_t HAL_CRS_RegisterExpectedSyncCallback(hal_crs_handle_t *hcrs, hal_crs_cb_t p_callback);
hal_status_t HAL_CRS_RegisterErrorCallback(hal_crs_handle_t *hcrs, hal_crs_cb_t p_callback);
#endif /* USE_HAL_CRS_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup CRS_Exported_Functions_Group6 State functions
  * @{
  */
hal_crs_state_t HAL_CRS_GetState(const hal_crs_handle_t *hcrs);
/**
  * @}
  */

#if defined(USE_HAL_CRS_GET_LAST_ERRORS) && (USE_HAL_CRS_GET_LAST_ERRORS == 1U)
/** @defgroup CRS_Exported_Functions_Group7 Error functions
  * @{
  */
uint32_t HAL_CRS_GetLastErrorCodes(const hal_crs_handle_t *hcrs);
/**
  * @}
  */
#endif /* USE_HAL_CRS_GET_LAST_ERRORS */

#if defined(USE_HAL_CRS_USER_DATA) && (USE_HAL_CRS_USER_DATA == 1U)
/** @defgroup CRS_Exported_Functions_Group8 Set/Get user data functions
  * @{
  */
void HAL_CRS_SetUserData(hal_crs_handle_t *hcrs, const void *p_user_data);
const void *HAL_CRS_GetUserData(const hal_crs_handle_t *hcrs);
/**
  * @}
  */
#endif /* USE_HAL_CRS_USER_DATA */

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
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_CRS_H */
