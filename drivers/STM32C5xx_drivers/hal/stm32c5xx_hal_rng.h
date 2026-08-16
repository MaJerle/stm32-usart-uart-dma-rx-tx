/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_rng.h
  * @brief   Header file for the RNG HAL module.
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
#ifndef STM32C5XX_HAL_RNG_H
#define STM32C5XX_HAL_RNG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_rng.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup RNG RNG
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_Exported_Constants HAL RNG Constants
  * @{
  */

/** @defgroup RNG_Noise_Source_Port Noise Source Port
  * @{
  */
#define HAL_RNG_OSCILLATOR_SOURCE_1 LL_RNG_NOISE_SRC_1 /*!< RNG noise source oscillator port index 1 */
#define HAL_RNG_OSCILLATOR_SOURCE_2 LL_RNG_NOISE_SRC_2 /*!< RNG noise source oscillator port index 2 */
#define HAL_RNG_OSCILLATOR_SOURCE_3 LL_RNG_NOISE_SRC_3 /*!< RNG noise source oscillator port index 3 */
/**
  * @}
  */

#if defined(USE_HAL_RNG_GET_LAST_ERRORS) && (USE_HAL_RNG_GET_LAST_ERRORS == 1)
/** @defgroup RNG_Error_Code Error Code
  * @{
  */
#define HAL_RNG_ERROR_NONE  0U             /*!< RNG no error    */
#define HAL_RNG_ERROR_SEED  LL_RNG_SR_SEIS /*!< RNG seed error  */
#define HAL_RNG_ERROR_CLOCK LL_RNG_SR_CEIS /*!< RNG clock error */
/**
  * @}
  */
#endif /* USE_HAL_RNG_GET_LAST_ERRORS */
/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_Exported_Types HAL RNG Types
  * @{
  */

/**
  * @brief RNG instance
  */
typedef enum
{
  HAL_RNG = RNG_BASE /*!< RNG instance */
} hal_rng_t;

/**
  * @brief RNG global state.
  */
typedef enum
{
  HAL_RNG_STATE_RESET  = 0UL,          /*!< RNG not yet initialized                 */
  HAL_RNG_STATE_INIT   = (1UL << 31U), /*!< RNG initialized and not yet configured  */
  HAL_RNG_STATE_IDLE   = (1UL << 30U), /*!< RNG initialized and configured          */
  HAL_RNG_STATE_ACTIVE = (1UL << 29U), /*!< RNG random number generation is ongoing */
  HAL_RNG_STATE_ERROR  = (1UL << 28U)  /*!< RNG seed error is detected              */
} hal_rng_state_t;

/**
  * @brief RNG clock error detection state
  */
typedef enum
{
  HAL_RNG_CLOCK_ERROR_DETECTION_ENABLED  = LL_RNG_CED_ENABLE, /*!< RNG clock error detection enabled  */
  HAL_RNG_CLOCK_ERROR_DETECTION_DISABLED = LL_RNG_CED_DISABLE /*!< RNG clock error detection disabled */
} hal_rng_clock_error_detection_status_t;

/**
  * @brief RNG clock divider
  */
typedef enum
{
  HAL_RNG_CLOCK_DIVIDER_BY_1     = LL_RNG_CLKDIV_BY_1,     /*!< RNG 1 clock cycles per internal clock     */
  HAL_RNG_CLOCK_DIVIDER_BY_2     = LL_RNG_CLKDIV_BY_2,     /*!< RNG 2 clock cycles per internal clock     */
  HAL_RNG_CLOCK_DIVIDER_BY_4     = LL_RNG_CLKDIV_BY_4,     /*!< RNG 4 clock cycles per internal clock     */
  HAL_RNG_CLOCK_DIVIDER_BY_8     = LL_RNG_CLKDIV_BY_8,     /*!< RNG 8 clock cycles per internal clock     */
  HAL_RNG_CLOCK_DIVIDER_BY_16    = LL_RNG_CLKDIV_BY_16,    /*!< RNG 16 clock cycles per internal clock    */
  HAL_RNG_CLOCK_DIVIDER_BY_32    = LL_RNG_CLKDIV_BY_32,    /*!< RNG 32 clock cycles per internal clock    */
  HAL_RNG_CLOCK_DIVIDER_BY_64    = LL_RNG_CLKDIV_BY_64,    /*!< RNG 64 clock cycles per internal clock    */
  HAL_RNG_CLOCK_DIVIDER_BY_128   = LL_RNG_CLKDIV_BY_128,   /*!< RNG 128 clock cycles per internal clock   */
  HAL_RNG_CLOCK_DIVIDER_BY_256   = LL_RNG_CLKDIV_BY_256,   /*!< RNG 256 clock cycles per internal clock   */
  HAL_RNG_CLOCK_DIVIDER_BY_512   = LL_RNG_CLKDIV_BY_512,   /*!< RNG 512 clock cycles per internal clock   */
  HAL_RNG_CLOCK_DIVIDER_BY_1024  = LL_RNG_CLKDIV_BY_1024,  /*!< RNG 1024 clock cycles per internal clock  */
  HAL_RNG_CLOCK_DIVIDER_BY_2048  = LL_RNG_CLKDIV_BY_2048,  /*!< RNG 2048 clock cycles per internal clock  */
  HAL_RNG_CLOCK_DIVIDER_BY_4096  = LL_RNG_CLKDIV_BY_4096,  /*!< RNG 4096 clock cycles per internal clock  */
  HAL_RNG_CLOCK_DIVIDER_BY_8192  = LL_RNG_CLKDIV_BY_8192,  /*!< RNG 8192 clock cycles per internal clock  */
  HAL_RNG_CLOCK_DIVIDER_BY_16384 = LL_RNG_CLKDIV_BY_16384, /*!< RNG 16384 clock cycles per internal clock */
  HAL_RNG_CLOCK_DIVIDER_BY_32768 = LL_RNG_CLKDIV_BY_32768  /*!< RNG 32768 clock cycles per internal clock */
} hal_rng_clock_divider_t;

/**
  * @brief RNG NIST compliance
  */
typedef enum
{
  HAL_RNG_CUSTOM = LL_RNG_CUSTOM_NIST,    /*!< RNG custom configuration         */
  HAL_RNG_NIST   = LL_RNG_NIST_COMPLIANT  /*!< RNG NIST compliant configuration */
} hal_rng_standard_t;
/**
  * @brief RNG Additional health test index
  */
typedef enum
{
  HAL_RNG_HTCR1  = LL_RNG_HTCR1,    /*!< RNG Additional health test 1     */
  HAL_RNG_HTCR2  = LL_RNG_HTCR2,    /*!< RNG Additional health test 2     */
  HAL_RNG_HTCR3  = LL_RNG_HTCR3     /*!< RNG Additional health test 3     */
} hal_rng_htcr_idx_t;
/**
  * @brief RNG automatic reset state
  */
typedef enum
{
  HAL_RNG_AUTO_RESET_ENABLED  = 1U, /*!< RNG enable automatic reset after seed error  */
  HAL_RNG_AUTO_RESET_DISABLED = 0U  /*!< RNG disable automatic reset after seed error */
} hal_rng_auto_reset_status_t;

/**
  * @brief RNG lock configuration state
  */
typedef enum
{
  HAL_RNG_LOCK_CONFIG_ENABLED  = 1U, /*!< RNG lock configuration enabled   */
  HAL_RNG_LOCK_CONFIG_DISABLED = 0U  /*!< RNG lock configuration disabled */
} hal_rng_lock_config_status_t;

typedef struct hal_rng_handle_s hal_rng_handle_t; /*!< RNG Handle Type Definition */

#if defined (USE_HAL_RNG_REGISTER_CALLBACKS) && (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
typedef void (*hal_rng_cb_t)(hal_rng_handle_t *hrng); /*!< Pointer to an RNG callback function. */
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

/**
  * @brief RNG handle structure definition.
  */
struct hal_rng_handle_s
{
  hal_rng_t                  instance;             /*!< RNG instance                          */
  volatile hal_rng_state_t   global_state;         /*!< RNG global state                      */

  uint32_t                   *p_data;              /*!< RNG pointer to data buffer            */
  volatile uint32_t          count;                /*!< RNG random number generation counter  */

#if defined(USE_HAL_RNG_GET_LAST_ERRORS) && (USE_HAL_RNG_GET_LAST_ERRORS == 1)
  volatile uint32_t          last_error_codes;     /*!< RNG last error codes                  */
#endif /* USE_HAL_RNG_GET_LAST_ERRORS */
#if defined (USE_HAL_RNG_REGISTER_CALLBACKS) && (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
  hal_rng_cb_t               p_generation_cplt_cb; /*!< RNG random number generation callback */
  hal_rng_cb_t               p_error_cb;           /*!< RNG error callback                    */
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */
#if defined (USE_HAL_RNG_USER_DATA) && (USE_HAL_RNG_USER_DATA == 1)
  const void                 *p_user_data;         /*!< RNG user data                         */
#endif /* USE_HAL_RNG_USER_DATA */
};

/**
  * @brief RNG noise source structure definition.
  */
typedef struct
{
  uint8_t osc_1_src; /*!< Oscillator noise source 1.
                          This parameter can be a combination of value described in @ref RNG_Noise_Source_Port */
  uint8_t osc_2_src; /*!< Oscillator noise source 2.
                          This parameter can be a combination of value described in @ref RNG_Noise_Source_Port */
  uint8_t osc_3_src; /*!< Oscillator noise source 3.
                          This parameter can be a combination of value described in @ref RNG_Noise_Source_Port */
} hal_rng_noise_source_t;

/**
  * @brief RNG configuration structure definition.
  */
typedef struct
{
  uint32_t                               config_1;              /*!< Config1 must be a value between 0 and 0x3F */
  uint32_t                               config_2;              /*!< Config2 must be a value between 0 and 0x7  */
  uint32_t                               config_3;              /*!< Config3 must be a value between 0 and 0xF  */
  uint32_t                               health_test;           /*!< RNG health test                            */
  hal_rng_clock_divider_t                clock_divider;         /*!< Clock divider factor                       */
  hal_rng_standard_t                     standard;              /*!< NIST compliance                            */
  hal_rng_clock_error_detection_status_t clock_error_detection; /*!< clock error detection                      */
  hal_rng_noise_source_t                 noise_src;             /*!< noise source                               */
} hal_rng_config_t;
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_Exported_Functions HAL RNG Functions
  * @{
  */

/** @defgroup RNG_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_RNG_Init(hal_rng_handle_t *hrng, hal_rng_t instance);
void HAL_RNG_DeInit(hal_rng_handle_t *hrng);
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group2 Configuration functions
  * @{
  */
hal_status_t HAL_RNG_SetConfig(hal_rng_handle_t *hrng, const hal_rng_config_t *p_config);
hal_status_t HAL_RNG_SetCandidateNISTConfig(hal_rng_handle_t *hrng);
hal_status_t HAL_RNG_SetCandidateGermanBSIConfig(hal_rng_handle_t *hrng);
void         HAL_RNG_GetConfig(const hal_rng_handle_t *hrng, hal_rng_config_t *p_config);
hal_status_t                           HAL_RNG_EnableClockErrorDetection(hal_rng_handle_t *hrng);
hal_status_t                           HAL_RNG_DisableClockErrorDetection(hal_rng_handle_t *hrng);
hal_rng_clock_error_detection_status_t HAL_RNG_IsEnabledClockErrorDetection(const hal_rng_handle_t *hrng);
hal_status_t                HAL_RNG_EnableAutoReset(hal_rng_handle_t *hrng);
hal_status_t                HAL_RNG_DisableAutoReset(hal_rng_handle_t *hrng);
hal_rng_auto_reset_status_t HAL_RNG_IsEnabledAutoReset(const hal_rng_handle_t *hrng);
hal_status_t            HAL_RNG_SetClockDivider(hal_rng_handle_t *hrng, hal_rng_clock_divider_t clk_divider);
hal_rng_clock_divider_t HAL_RNG_GetClockDivider(const hal_rng_handle_t *hrng);
hal_status_t   HAL_RNG_SetHealthFactorConfig(hal_rng_handle_t *hrng, hal_rng_htcr_idx_t htcr_idx, uint32_t htcr_value);
uint32_t HAL_RNG_GetHealthFactorConfig(const hal_rng_handle_t *hrng, hal_rng_htcr_idx_t htcr_idx);
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral control functions
  * @{
  */
hal_status_t HAL_RNG_GenerateRandomNumber(hal_rng_handle_t *hrng, uint32_t *p_data, uint32_t size_word,
                                          uint32_t timeout_ms);
hal_status_t HAL_RNG_GenerateRandomNumber_IT(hal_rng_handle_t *hrng, uint32_t *p_data, uint32_t size_word);
void         HAL_RNG_IRQHandler(hal_rng_handle_t *hrng);
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group4 Seed error recovery function
  * @{
  */
hal_status_t HAL_RNG_RecoverSeedError(hal_rng_handle_t *hrng);
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group5 Callbacks functions
  * @{
  */
void HAL_RNG_GenerationCpltCallback(hal_rng_handle_t *hrng);
void HAL_RNG_ErrorCallback(hal_rng_handle_t *hrng);
#if defined(USE_HAL_RNG_REGISTER_CALLBACKS) && (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
hal_status_t HAL_RNG_RegisterGenerationCpltCallback(hal_rng_handle_t *hrng, hal_rng_cb_t callback);
hal_status_t HAL_RNG_RegisterErrorCallback(hal_rng_handle_t *hrng, hal_rng_cb_t callback);
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group6 User Data functions
  * @{
  */
#if defined (USE_HAL_RNG_USER_DATA) && (USE_HAL_RNG_USER_DATA == 1)
void HAL_RNG_SetUserData(hal_rng_handle_t *hrng, const void *p_user_data);
const void *HAL_RNG_GetUserData(const hal_rng_handle_t *hrng);
#endif /* USE_HAL_RNG_USER_DATA */
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group7 Status functions
  * @{
  */
hal_rng_state_t HAL_RNG_GetState(const hal_rng_handle_t *hrng);
#if defined(USE_HAL_RNG_GET_LAST_ERRORS) && (USE_HAL_RNG_GET_LAST_ERRORS == 1)
uint32_t HAL_RNG_GetLastErrorCodes(const hal_rng_handle_t *hrng);
#endif /* USE_HAL_RNG_GET_LAST_ERRORS */
/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group8 Lock configuration functions
  * @{
  */
hal_status_t HAL_RNG_LockConfig(hal_rng_handle_t *hrng);
hal_rng_lock_config_status_t HAL_RNG_IsConfigLocked(const hal_rng_handle_t *hrng);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_RNG_H */
