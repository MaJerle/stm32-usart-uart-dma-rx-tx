/**
  *********************************************************************************************************************
  * @file    stm32c5xx_hal_ccb.h
  * @brief   Header file of CCB HAL module.
  *********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *********************************************************************************************************************
  */

/* Define to prevent recursive inclusion ----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_CCB_H
#define STM32C5XX_HAL_CCB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_rng.h"
#include "stm32c5xx_ll_pka.h"
#include "stm32c5xx_ll_tamp.h"
/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(CCB)

/** @defgroup CCB CCB
  * @{
  */

/* Exported constants -----------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Exported_Constants HAL CCB Constants
  * @{
  */

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
/** @defgroup  CCB_Error_Code Error codes defined as bitfields
  * @{
  */
#define HAL_CCB_ERROR_NONE                   (0x0U)     /*!< No error */
#define HAL_CCB_ERROR_CCB_RCC_RESET          (0x1U)     /*!< CCB RCC reset detected when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_SAES_RCC_RESET         (0x2U)     /*!< SAES RCC reset detected when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_PKA_RCC_RESET          (0x3U)     /*!< PKA RCC reset detected when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_RNG_RCC_RESET          (0x4U)     /*!< RNG RCC reset detected when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_CCB_CLOCK_GATED        (0x5U)     /*!< RCC gated the CCB clock when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_SAES_CLOCK_GATED       (0x6U)     /*!< RCC gated the SAES clock when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_PKA_CLOCK_GATED        (0x7U)     /*!< RCC gated the PKA clock when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_RNG_CLOCK_GATED        (0x8U)     /*!< RCC gated the RNG clock when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_SAES_IV1_TRIVIAL       (0x9U)     /*!< SAES_IV1 is trivial (all 0s or all 1s) when OPSTEP = 0x2 */
#define HAL_CCB_ERROR_SAES_IV2_TRIVIAL       (0xAU)     /*!< SAES_IV2 is trivial (all 0s or all 1s) when OPSTEP = 0x2 */
#define HAL_CCB_ERROR_SAES_IV3_TRIVIAL       (0xBU)     /*!< SAES_IV3 is trivial (all 0s or all 1s) when OPSTEP = 0x2 */
#define HAL_CCB_ERROR_SAES_GCM_TAG_FAIL      (0xCU)     /*!< SAES GCM TAG comparison failed or wrong register read
                                                             at OPSTEP = 0x17 */
#define HAL_CCB_ERROR_SAES_KEYSIZE_INVALID   (0xDU)     /*!< KEYSIZE different from 256 bits in SAES_CR when enabled
                                                             and OPSTEP > 0x0 */
#define HAL_CCB_ERROR_SAES_CHMOD_INVALID     (0xEU)     /*!< CHMOD different than GCM or SAES not enabled or KEYVALID
                                                             cleared when GCMPH > 0x0 */
#define HAL_CCB_ERROR_SAES_KMOD_INVALID      (0xFU)     /*!< KMOD different from wrapped key mode in SAES_CR
                                                             at OPSTEP = 0x1 or 0x1A */
#define HAL_CCB_ERROR_SAES_KEYSEL_INVALID    (0x10U)    /*!< KEYSEL different from 0x1 or 0x4 in SAES_CR when enabled,
                                                             KEYVALID set, OPSTEP = 0x1 */
#define HAL_CCB_ERROR_SAES_GCMPH_INVALID     (0x11U)    /*!< GCMPH not correct (not incremented) in SAES_CR
                                                             when OPSTEP > 0x1 */
#define HAL_CCB_ERROR_SAES_MODE_MISMATCH     (0x12U)    /*!< MODE in SAES_CR does not match expected value
                                                             when OPSTEP > 0x1 */
#define HAL_CCB_ERROR_SAES_BUSY              (0x13U)    /*!< BUSY = 1 in SAES_SR when OPSTEP = 0x18 or 0x19 */
#define HAL_CCB_ERROR_PKA_EN_INVALID         (0x14U)    /*!< EN = 0 in PKA_CR when OPSTEP > 0x2 (except 0x12) */
#define HAL_CCB_ERROR_PKA_INITOK_INVALID     (0x15U)    /*!< INITOK = 0 in PKA_SR when OPSTEP > 0x2 (except 0x12) */
#define HAL_CCB_ERROR_AHB                    (0x16U)    /*!< Authorized by local firewall, but unexpected AHB error
                                                             when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_PKA_LMF_SET            (0x17U)    /*!< LMF = 1 in PKA_SR (only ECDSA verification available)
                                                             when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_REFTAG_ZERO            (0x18U)    /*!< CCB_REFTAGRx registers are equal to 0 when OPSTEP > 0x12 */
#define HAL_CCB_ERROR_SECURITY_PROTECTION    (0x1AU)    /*!< Security/privilege protection inconsistent or changed
                                                             when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_PKA_MODE_MISMATCH      (0x1BU)    /*!< MODE in PKA_CR does not match expected value
                                                             when OPSTEP > 0x0 */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_0        (0x1CU)    /*!< OPSTEP end-step event not found at OPSTEP = 0x0  */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_1        (0x1DU)    /*!< OPSTEP end-step event not found at OPSTEP = 0x1  */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_2        (0x1EU)    /*!< OPSTEP end-step event not found at OPSTEP = 0x2  */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_3        (0x1FU)    /*!< OPSTEP end-step event not found at OPSTEP = 0x3  */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_A        (0x20U)    /*!< OPSTEP end-step event not found at OPSTEP = 0xA  */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_12       (0x21U)    /*!< OPSTEP end-step event not found at OPSTEP = 0x12 */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_13       (0x22U)    /*!< OPSTEP end-step event not found at OPSTEP = 0x13 */
#define HAL_CCB_ERROR_EVENT_ENDSTEP_17       (0x23U)    /*!< OPSTEP end-step event not found at OPSTEP = 0x17 */
/**
  * @}
  */

#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

/** @defgroup CCB_Flag_Event Flag event
  * @{
  */
#define HAL_CCB_FLAG_BUSY                CCB_SR_CCB_BUSY        /*!< CCB busy or PKA RAM is being cleared
                                                                     following a peripheral reset          */
#define HAL_CCB_TAMP_EVT_RNG             CCB_SR_TAMP_EVT0       /*!< RNG TAMP Event                        */
#define HAL_CCB_TAMP_EVT_SAES            CCB_SR_TAMP_EVT1       /*!< SAES TAMP Event                       */
#define HAL_CCB_TAMP_EVT_PKA_RAM_REG     CCB_SR_TAMP_EVT2       /*!< PKA RAM TAMP Event                    */
#define HAL_CCB_TAMP_EVT_PKA_OPR         CCB_SR_TAMP_EVT3       /*!< PKA TAMP Event                        */
#define HAL_CCB_TAMP_EVT_CCB             CCB_SR_TAMP_EVT4       /*!< CCB TAMP Event                        */
/**
  * @}
  */
/**
  * @}
  */

/* Exported types ---------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Exported_Types HAL CCB Types
  * @{
  */

/**
  * @brief CCB instance enumeration definition
  */
typedef enum
{
  HAL_CCB = CCB_BASE       /*!< CCB instance  */
} hal_ccb_t;

/**
  * @brief CCB Global state enumeration definition
  */
typedef enum
{
  HAL_CCB_STATE_RESET  = 0UL,                 /*!< CCB not yet initialized or disabled         */
  HAL_CCB_STATE_IDLE   = (1UL << 31U),        /*!< CCB peripheral is initialized               */
  HAL_CCB_STATE_ACTIVE = (1UL << 30U),        /*!< CCB internal processing is ongoing          */
  HAL_CCB_STATE_FAULT  = (1UL << 29U)         /*!< CCB encountered an unrecoverable error and
                                                   a recovery sequence is needed               */
} hal_ccb_state_t;

/**
  * @brief CCB Wrapping Key definition
  */
typedef enum
{
  HAL_CCB_KEY_HW          =  AES_CR_KEYSEL_0,   /*!< Hardware key : derived hardware
                                                        unique key (DHUK 256-bit)       */
  HAL_CCB_KEY_HSW         = AES_CR_KEYSEL_2     /*!< DHUK XOR BHK Hardware unique
                                                        key XOR software key            */
} hal_ccb_wrapping_hw_key_type_t;


/**
  * @brief CCB AES Algorithm Mode definition
  */
typedef enum
{
  HAL_CCB_AES_ECB  =   0U,                /*!< Electronic codebook chaining algorithm   */
  HAL_CCB_AES_CBC  =  AES_CR_CHMOD_0,    /*!< Cipher block chaining algorithm          */

} hal_ccb_aes_chaining_mode_t;

/**
  * @brief Wrapping sw key context structure definition
  */
typedef struct
{
  hal_ccb_aes_chaining_mode_t        aes_algorithm;                     /*!< AES Algorithm.
                                                                             It can be ECB or CBC algorithms      */

  uint32_t                           *p_init_vect;                      /*!< Pointer to the initialization vector
                                                                             counter with CBC Algorithm           */
  uint32_t                            key_size;                         /*!< symmetric key size is always 256     */
} hal_ccb_wrapping_sw_key_context_t;

/**
  * @brief CCB pool buffer structure definition
  */
typedef struct
{
  void *p_buff;              /*!< A pre-allocated memory block provided by the user for internal calculation
                                  of CCB blob creation                                                        */
  uint32_t buff_size_byte;   /*!< Pool buffer Size in bytes. The size can be calculated using the provided
                                  helper macros                                                               */
} hal_ccb_ctx_pool_buff_t;

/**
  * @brief CCB ECDSA curve parameters structure definition
  */
typedef struct
{
  uint32_t prime_order_size_byte;            /*!< Number of element in prime Order array in byte                     */
  uint32_t modulus_size_byte;                /*!< Number of element in modulus array in byte                         */
  uint32_t coef_sign_a;                      /*!< Curve coefficient a sign                                           */
  const uint8_t *p_abs_coef_a;               /*!< Pointer to curve coefficient |a| (Array of modulus size elements)  */
  const uint8_t *p_coef_b;                   /*!< Pointer to B coefficient         (Array of modulus size elements)  */
  const uint8_t *p_modulus;                  /*!< Pointer to curve modulus value p (Array of modulus size elements)  */
  const uint8_t *p_prime_order;              /*!< Pointer to prime order of the curve n  (Array of prime order size
                                                  elements)                                                          */
  const uint8_t *p_point_x;                  /*!< Pointer to curve base point xG   (Array of modulus size elements)  */
  const uint8_t *p_point_y;                  /*!< Pointer to curve base point yG   (Array of modulus size elements)  */
  hal_ccb_ctx_pool_buff_t ecdsa_pool_buffer; /*!< Pool buffer context used for ECDSA internal calculation of
                                                  CCB blob creation. Its size can be calculated using
                                                  HAL_CCB_ECDSA_CALC_BUFFER_SIZE() helper macro                      */
} hal_ccb_ecdsa_curve_param_t;

/**
  * @brief CCB ECC scalar Multiplication curve parameters structure definition
  */
typedef struct
{
  uint32_t prime_order_size_byte;          /*!< Number of element in prime Order array in byte                      */
  uint32_t modulus_size_byte;              /*!< Number of element in modulus array in byte                          */
  uint32_t coef_sign_a;                    /*!< Curve coefficient a sign                                            */
  const uint8_t *p_abs_coef_a;             /*!< Pointer to curve coefficient |a| (Array of modulus size elements)   */
  const uint8_t *p_coef_b;                 /*!< Pointer to B coefficient         (Array of modulus size elements)   */
  const uint8_t *p_modulus;                /*!< Pointer to curve modulus value p (Array of modulus size elements)   */
  const uint8_t *p_prime_order;            /*!< Pointer to prime order of the curve n  (Array of prime order size
                                                elements)                                                           */
  const uint8_t *p_point_x;                /*!< Pointer to curve base point xG   (Array of modulus size elements)   */
  const uint8_t *p_point_y;                /*!< Pointer to curve base point yG   (Array of modulus size elements)   */
  hal_ccb_ctx_pool_buff_t ecc_pool_buffer; /*!< Pool buffer context used for ECC internal calculation of
                                                CCB blob creation. Its size can be calculated using
                                                HAL_CCB_ECC_CALC_BUFFER_SIZE() helper macro                         */
} hal_ccb_ecc_mul_curve_param_t;


/**
  * @brief CCB ECDSA Key Blob structure definition
  */
typedef struct
{
  uint32_t *p_iv;                        /*!< Pointer to the Initial Vector   */
  uint32_t *p_tag;                       /*!< Pointer to the Tag              */
  uint32_t *p_wrapped_key;               /*!< Pointer to the Wrapped Key      */
} hal_ccb_ecdsa_key_blob_t;

/**
  * @brief CCB ECC scalar Multiplication Key Blob structure definition
  */
typedef struct
{
  uint32_t *p_iv;                        /*!< Pointer to the Initial Vector   */
  uint32_t *p_tag;                       /*!< Pointer to the Tag              */
  uint32_t *p_wrapped_key;               /*!< Pointer to the Wrapped Key      */
} hal_ccb_ecc_mul_key_blob_t;

/**
  * @brief CCB ECDSA Signature structure definition
  */
typedef struct
{
  uint8_t *p_r_sign;                       /*!< Pointer to signature part r  (Array of modulus size elements) */
  uint8_t *p_s_sign;                       /*!< Pointer to signature part s  (Array of modulus size elements) */
} hal_ccb_ecdsa_sign_t;

/**
  * @brief CCB ECC scalar multiplication point structure definition
  */
typedef struct
{
  uint8_t *p_point_x;                         /*!< Pointer to point P coordinate xP */
  uint8_t *p_point_y;                         /*!< Pointer to point P coordinate yP */
} hal_ccb_ecc_point_t;

/**
  * @brief CCB RSA clear key structure definition
  */
typedef struct
{
  uint8_t *p_exp;                             /*!< Pointer to Exponent   */
  uint8_t *p_phi;                             /*!< Pointer to Phi value  */
} hal_ccb_rsa_clear_key_t;

/**
  * @brief CCB RSA Modular exponentiation parameters structure definition
  */
typedef struct
{
  uint32_t exp_size_byte;                  /*!< Number of element in p_exp and p_montgomery_param arrays            */
  uint32_t modulus_size_byte;              /*!< Number of element in modulus array                                  */
  const uint8_t *p_mod;                    /*!< Pointer to modulus (Array of operand size elements)                 */
  hal_ccb_ctx_pool_buff_t rsa_pool_buffer; /*!< Pool buffer context used for RSA internal calculation of
                                                CCB blob creation. Its size can be calculated using
                                                HAL_CCB_RSA_CALC_BUFFER_SIZE() helper macro                         */
} hal_ccb_rsa_param_t;

/**
  * @brief CCB RSA Modular exponentiation Key Blob structure definition
  */
typedef struct
{
  uint32_t *p_iv;                        /*!< Pointer to the Initial Vector     */
  uint32_t *p_tag;                       /*!< Pointer to the Tag                */
  uint32_t *p_wrapped_exp;               /*!< Pointer to the Wrapped Exponent   */
  uint32_t *p_wrapped_phi;               /*!< Pointer to the Wrapped Phi        */
} hal_ccb_rsa_key_blob_t;

typedef struct hal_ccb_handle_s hal_ccb_handle_t;     /*!< CCB Handle Structure type */

/**
  * @brief CCB handle Structure definition
  */
struct hal_ccb_handle_s
{
  hal_ccb_t                 instance;           /*!< CCB Register base address, can be a value of @ref hal_ccb_t  */
  volatile hal_ccb_state_t  global_state;       /*!< CCB peripheral state, can be a value of @ref hal_ccb_state_t */
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  volatile uint32_t        last_error_codes;    /*!< CCB  peripheral error code                                   */
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
#if defined(USE_HAL_CCB_USER_DATA) && (USE_HAL_CCB_USER_DATA == 1)
  const void               *p_user_data;        /*!< CCB User Data Pointer                                        */
#endif /* USE_HAL_CCB_USER_DATA */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Exported_Functions HAL CCB Functions
  * @{
  */

/** @defgroup CCB_Exported_Functions_Group1 Initialization and De-initialization functions
  * @{
  */
hal_status_t HAL_CCB_Init(hal_ccb_handle_t *hccb, hal_ccb_t instance);
void HAL_CCB_DeInit(hal_ccb_handle_t *hccb);
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group2 Reset function
  * @{
  */
void HAL_CCB_Reset(hal_ccb_handle_t *hccb);
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group3 Wrapping symmetric key functions
  * @{
  */
hal_status_t HAL_CCB_ECDSA_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                            hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                            uint32_t *p_out_wrapped_user_key);

hal_status_t HAL_CCB_ECC_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                          hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                          uint32_t *p_out_wrapped_user_key);
hal_status_t HAL_CCB_RSA_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                          hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                          uint32_t *p_out_wrapped_user_key);
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group4 Process with software wrapping key functions
  * @{
  */
/* ECDSA operation */
hal_status_t HAL_CCB_ECDSA_SW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                             const uint8_t *p_in_clear_private_key,
                                             const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                             const uint32_t *p_in_wrapped_user_key,
                                             hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECDSA_SW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                     const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                     const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                                     const uint32_t *p_in_wrapped_user_key,
                                                     hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECDSA_SW_Sign(hal_ccb_handle_t *hccb, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                   const uint32_t *p_in_wrapped_user_key,
                                   hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob, const uint8_t *p_in_hash,
                                   uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature);

hal_status_t HAL_CCB_ECDSA_SW_ComputePublicKey(hal_ccb_handle_t *hccb,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                               const uint32_t *p_in_wrapped_user_key,
                                               hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob,
                                               hal_ccb_ecc_point_t *p_out_public_key);

/* ECC operation */
hal_status_t HAL_CCB_ECC_SW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                           const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                           const uint8_t *p_in_clear_private_key,
                                           const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                           const uint32_t *p_in_wrapped_user_key,
                                           hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECC_SW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                   const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                   const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                                   const uint32_t *p_in_wrapped_user_key,
                                                   hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECC_SW_ComputeScalarMul(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                             const uint32_t *p_in_wrapped_user_key,
                                             hal_ccb_ecc_mul_key_blob_t *p_in_wrapped_private_key_blob,
                                             hal_ccb_ecc_point_t *p_in_point,
                                             hal_ccb_ecc_point_t *p_out_point);

/* RSA operation */
hal_status_t HAL_CCB_RSA_SW_WrapPrivateKey(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                           const hal_ccb_rsa_clear_key_t *p_in_rsa_clear_private_key,
                                           const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                           const uint32_t *p_in_wrapped_user_key,
                                           hal_ccb_rsa_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_RSA_SW_ComputeModularExp(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                              const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                              const uint32_t *p_in_wrapped_user_key,
                                              hal_ccb_rsa_key_blob_t *p_in_wrapped_private_key_blob,
                                              const uint8_t *p_in_operand, uint8_t *p_out_modular_exp);
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group5 Process with hardware wrapping key functions
  * @{
  */
/* ECDSA operation */
hal_status_t HAL_CCB_ECDSA_HW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                             const uint8_t *p_in_clear_private_key,
                                             hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                             hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECDSA_HW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                     const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                     hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                                     hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECDSA_HW_Sign(hal_ccb_handle_t *hccb, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                   hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob, const uint8_t *p_in_hash,
                                   uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature);

hal_status_t HAL_CCB_ECDSA_HW_ComputePublicKey(hal_ccb_handle_t *hccb,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                               hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob,
                                               hal_ccb_ecc_point_t *p_out_public_key);

/* ECC operation */
hal_status_t HAL_CCB_ECC_HW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                           const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                           const uint8_t *p_in_clear_private_key,
                                           hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                           hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECC_HW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                   const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                   hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                                   hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob);

hal_status_t HAL_CCB_ECC_HW_ComputeScalarMul(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                             hal_ccb_ecc_mul_key_blob_t *p_in_wrapped_private_key_blob,
                                             hal_ccb_ecc_point_t *p_in_point,
                                             hal_ccb_ecc_point_t *p_out_point);

/* RSA operation */
hal_status_t HAL_CCB_RSA_HW_WrapPrivateKey(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                           const hal_ccb_rsa_clear_key_t *p_in_rsa_clear_private_key,
                                           hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                           hal_ccb_rsa_key_blob_t *p_out_wrapped_private_key_blob);


hal_status_t HAL_CCB_RSA_HW_ComputeModularExp(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                              hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                              hal_ccb_rsa_key_blob_t *p_in_wrapped_private_key_blob,
                                              const uint8_t *p_in_operand, uint8_t *p_out_modular_exp);
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group6 Peripheral State, Error functions
  * @{
  */
hal_ccb_state_t HAL_CCB_GetState(const hal_ccb_handle_t *hccb);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
uint32_t HAL_CCB_GetLastErrorCodes(const hal_ccb_handle_t *hccb);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */


#if defined(USE_HAL_CCB_USER_DATA) && (USE_HAL_CCB_USER_DATA == 1)
void HAL_CCB_SetUserData(hal_ccb_handle_t *hccb, const void *p_user_data);
const void *HAL_CCB_GetUserData(const hal_ccb_handle_t *hccb);
#endif /* (USE_HAL_CCB_USER_DATA) */
/**
  * @}
  */

/** @defgroup CCB_Exported_Functions_Group7 Static Inlines functions
  * @{
This section provides functions to manage CCB flags:
  - HAL_CCB_GetFlag()          :Return the state of a flag
  */
/** @brief  Check whether the specified CCB status flag is set or not.
  * @param  hccb Specifies the ccb handle
  * @param  flag Specifies the flag to check, this parameter can be one of the following values:
  *            @arg @ref HAL_CCB_FLAG_BUSY               CCB busy or PKA RAM is being cleared
                                                         following a peripheral reset
  *            @arg @ref HAL_CCB_TAMP_EVT_RNG            RNG TAMP Event flag
  *            @arg @ref HAL_CCB_TAMP_EVT_SAES           SAES TAMP Event flag
  *            @arg @ref HAL_CCB_TAMP_EVT_PKA_RAM_REG    PKA Register TAMP Event flag
  *            @arg @ref HAL_CCB_TAMP_EVT_PKA_OPR        PKA Operation TAMP Event flag
  *            @arg @ref HAL_CCB_TAMP_EVT_CCB            CCB TAMP Event flag
  * @retval uint32_t The state or value of flag.
  */
__STATIC_INLINE uint32_t HAL_CCB_GetFlag(const hal_ccb_handle_t *hccb, uint32_t flag)
{
  return (((STM32_READ_BIT(((CCB_TypeDef *)((uint32_t)hccb->instance))->SR, flag) == flag) ? 1U : 0U));
}
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Exported_Macros HAL CCB Macros
  * @{
  */

/** @brief  Helper macro to calculate the minimum ECDSA pool buffer required size.
  * @param  modulus_size_byte Specifies the modulus size in bytes.
  * @retval The required Pool buffer size in bytes.
  */
#define HAL_CCB_ECDSA_CALC_BUFFER_SIZE(modulus_size_byte) (48U + ((modulus_size_byte) * 5U))

/** @brief  Helper macro to calculate the minimum ECC pool buffer required size.
  * @param  modulus_size_byte Specifies the modulus size in bytes.
  * @retval The required Pool buffer size in bytes.
  */
#define HAL_CCB_ECC_CALC_BUFFER_SIZE(modulus_size_byte) (52U + ((modulus_size_byte) * 2U))

/** @brief  Helper macro to calculate the minimum RSA pool buffer required size.
  * @param  modulus_size_byte Specifies the modulus size in bytes.
  * @retval The required Pool buffer size in bytes.
  */
#define HAL_CCB_RSA_CALC_BUFFER_SIZE(modulus_size_byte) (64U + ((modulus_size_byte) * 2U))
/**
  * @}
  */

/**
  * @}
  */

#endif /* CCB */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_CCB_H */
