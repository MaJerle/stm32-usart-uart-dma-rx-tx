/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_pka.h
  * @brief   Header file of PKA HAL module.
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
#ifndef STM32C5XX_HAL_PKA_H
#define STM32C5XX_HAL_PKA_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_pka.h"
#include "stm32c5xx_ll_rng.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(PKA)

/** @defgroup PKA PKA
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_Exported_Constants HAL PKA Constants
  * @{
  */

/** @defgroup PKA_Error_Code_definition PKA Error Code definition
  * @{
  */
#define HAL_PKA_ERROR_NONE      0x00U               /*!< PKA error code: none */
#define HAL_PKA_ERROR_RESULT    0x01U               /*!< PKA result error    */
#define HAL_PKA_ERROR_ADDRERR   LL_PKA_FLAG_ADDRERR /*!< PKA address error   */
#define HAL_PKA_ERROR_RAMERR    LL_PKA_FLAG_RAMERR  /*!< PKA RAM error       */
#define HAL_PKA_ERROR_OPERATION LL_PKA_FLAG_OPERR   /*!< PKA operation error */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_Exported_Types HAL PKA Types
  * @{
  */

/**
  * @brief PKA instance enumeration definition
  */
typedef enum
{
  HAL_PKA1 = PKA_BASE  /*!< HAL PKA instance */
} hal_pka_t;

/**
  * @brief PKA state enumeration definition
  */
typedef enum
{
  HAL_PKA_STATE_RESET  = 0U,           /*!< PKA not yet initialized or disabled       */
  HAL_PKA_STATE_INIT   = (1UL << 31U), /*!< PKA is initialized but not yet configured */
  HAL_PKA_STATE_IDLE   = (1UL << 30U), /*!< PKA is initialized and configured         */
  HAL_PKA_STATE_ACTIVE = (1UL << 29U)  /*!< PKA internal processing is ongoing        */
} hal_pka_state_t;

/**
  * @brief PKA RSA signature state
  */
typedef enum
{
  PKA_RSA_SIGNATURE_NOT_VALID = 0U, /*!< The RSA signature is not valid */
  PKA_RSA_SIGNATURE_VALID     = 1U  /*!< The RSA signature is valid     */
} hal_pka_rsa_signature_status_t;

/**
  * @brief PKA ECDSA signature state
  */
typedef enum
{
  PKA_ECDSA_SIGNATURE_NOT_VALID = 0U, /*!< The ECDSA signature is not valid */
  PKA_ECDSA_SIGNATURE_VALID     = 1U  /*!< The ECDSA signature is valid     */
} hal_pka_ecdsa_signature_status_t;

/**
  * @brief PKA ECC point state
  */
typedef enum
{
  PKA_ECC_POINT_NOT_ON_CURVE = 0U, /*!< The ECC point is not on the curve */
  PKA_ECC_POINT_ON_CURVE     = 1U  /*!< The ECC point is on the curve     */
} hal_pka_ecc_point_status_t;


/**
  * @brief PKA ECC scalar multiplication protected configuration structure definition
  */
typedef struct
{
  uint32_t      prime_order_size_byte; /*!< Number of elements in p_prime_order array                                */
  uint32_t      scalar_mul_size_byte;  /*!< Number of elements in p_scalar_mul array                                 */
  uint32_t      modulus_size_byte;     /*!< Number of elements in p_modulus, p_coeff_a, p_pt_x and p_pt_y arrays     */
  uint32_t      coeff_sign;            /*!< Curve coefficient a sign                                                */
  const uint8_t *p_coeff_a;            /*!< Pointer to curve coefficient |a|                                        */
  const uint8_t *p_coeff_b;            /*!< Pointer to curve coefficient b                                          */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of modulus_size_byte elements)  */
  const uint8_t *p_pt_x;               /*!< Pointer to point P coordinate xP (Array of modulus_size_byte elements)  */
  const uint8_t *p_pt_y;               /*!< Pointer to point P coordinate yP (Array of modulus_size_byte elements)  */
  const uint8_t *p_scalar_mul;         /*!< Pointer to scalar multiplier k (Array of scalar_mul_size_byte elements) */
  const uint8_t *p_prime_order;        /*!< Pointer to curve prime order (Array of prime_order_size_byte elements)  */
} hal_pka_ecc_mul_protect_config_t;

/**
  * @brief PKA point-on-elliptic-curve check configuration structure definition
  */
typedef struct
{
  uint32_t      modulus_size_byte;   /*!< Number of elements in p_coeff_a, p_coeff_b, p_modulus, p_pt_x and p_pt_y
                                          arrays                                                                 */
  uint32_t      coeff_sign;          /*!< Curve coefficient a sign                                               */
  const uint8_t *p_montgomery_param; /*!< Pointer to Montgomery parameter (Array of modulus_size_byte)           */
  const uint8_t *p_coeff_a;          /*!< Pointer to curve coefficient |a| (Array of modulus_size_byte elements) */
  const uint8_t *p_coeff_b;          /*!< Pointer to curve coefficient b   (Array of modulus_size_byte elements) */
  const uint8_t *p_modulus;          /*!< Pointer to curve modulus value p (Array of modulus_size_byte elements) */
  const uint8_t *p_pt_x;             /*!< Pointer to point P coordinate xP (Array of modulus_size_byte elements) */
  const uint8_t *p_pt_y;             /*!< Pointer to point P coordinate yP (Array of modulus_size_byte elements) */
} hal_pka_point_check_config_t;

/**
  * @brief PKA RSA CRT exponentiation configuration structure definition
  */
typedef struct
{
  uint32_t      size_byte;       /*!< Number of elements in p_operand_a array                 */
  const uint8_t *p_operand_dp;   /*!< Pointer to operand dP (Array of size_byte/2 elements)   */
  const uint8_t *p_operand_dq;   /*!< Pointer to operand dQ (Array of size_byte/2 elements)   */
  const uint8_t *p_operand_qinv; /*!< Pointer to operand qinv (Array of size_byte/2 elements) */
  const uint8_t *p_prime_p;      /*!< Pointer to prime p (Array of size_byte/2 elements)      */
  const uint8_t *p_prime_q;      /*!< Pointer to prime Q (Array of size_byte/2 elements)      */
  const uint8_t *p_operand_a;    /*!< Pointer to operand A (Array of size_byte elements)      */
} hal_pka_rsa_crt_exp_config_t;

/**
  * @brief PKA RSA signature configuration structure definition
  */
typedef struct
{
  uint32_t      private_key_size_byte; /*!< Number of elements in p_private_key array                            */
  uint32_t      hash_size_byte;        /*!< Number of elements in p_hash array                                   */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of hash_size_byte elements) */
  const uint8_t *p_private_key;        /*!< Pointer to private key d (Array of private_key_size_byte elements)  */
  const uint8_t *p_hash;               /*!< Pointer to hash of the message (Array of hash_size_byte elements)   */
} hal_pka_rsa_signature_config_t;

/**
  * @brief PKA RSA signature fast configuration structure definition
  */
typedef struct
{
  uint32_t      private_key_size_byte; /*!< Number of elements in p_private_key array                             */
  uint32_t      hash_size_byte;        /*!< Number of elements in p_hash array                                    */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of hash_size_byte elements)   */
  const uint8_t *p_private_key;        /*!< Pointer to private key d (Array of private_key_size_byte elements)    */
  const uint8_t *p_hash;               /*!< Pointer to hash of the message (Array of hash_size_byte elements)     */
  const uint8_t *p_montgomery_param;   /*!< Pointer to Montgomery parameter (Array of exponent_size_byte elements)*/
} hal_pka_rsa_signature_fast_config_t;

/**
  * @brief PKA RSA signature protected configuration structure definition
  */
typedef struct
{
  uint32_t      private_key_size_byte; /*!< Number of elements in p_private_key array                           */
  uint32_t      hash_size_byte;        /*!< Number of elements in p_hash array                                  */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of hash_size_byte elements) */
  const uint8_t *p_private_key;        /*!< Pointer to private key d (Array of private_key_size_byte elements)  */
  const uint8_t *p_hash;               /*!< Pointer to hash of the message (Array of hash_size_byte elements)   */
  const uint8_t *p_phi;                /*!< Pointer to Phi value                                                */
} hal_pka_rsa_signature_protect_config_t;

/**
  * @brief PKA RSA verification configuration structure definition
  */
typedef struct
{
  uint32_t      public_key_size_byte; /*!< Number of elements in p_public_key array                            */
  uint32_t      sign_size_byte;       /*!< Number of elements in p_sign array                                  */
  const uint8_t *p_modulus;           /*!< Pointer to curve modulus value p (Array of sign_size_byte elements) */
  const uint8_t *p_public_key;        /*!< Pointer to public key d (Array of public_key_size_byte elements)    */
  const uint8_t *p_sign;              /*!< Pointer to RSA signature (Array of sign_size_byte elements)         */
} hal_pka_rsa_verif_config_t;

/**
  * @brief PKA elliptic curves over prime fields verification configuration structure definition
  */
typedef struct
{
  uint32_t      prime_order_size_byte; /*!< Number of elements in prime order array                                   */
  uint32_t      modulus_size_byte;     /*!< Number of elements in modulus array                                       */
  uint32_t      coeff_sign;            /*!< Curve coefficient a sign                                                  */
  const uint8_t *p_coeff;              /*!< Pointer to curve coefficient |a| (Array of modulus_size_byte elements)    */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of modulus_size_byte elements)    */
  const uint8_t *p_base_pt_x;          /*!< Pointer to curve base point xG (Array of modulus_size_byte elements)      */
  const uint8_t *p_base_pt_y;          /*!< Pointer to curve base point yG (Array of modulus_size_byte elements)      */
  const uint8_t *p_pub_key_curve_pt_x; /*!< Pointer to public key curve point xG (Array of modulus_size_byte elements) */
  const uint8_t *p_pub_key_curve_pt_y; /*!< Pointer to public key curve point yG (Array of modulus_size_byte elements) */
  const uint8_t *p_r_sign;             /*!< Pointer to signature part r (Array of prime_order_size_byte elements)     */
  const uint8_t *p_s_sign;             /*!< Pointer to signature part s (Array of prime_order_size_byte elements)     */
  const uint8_t *p_hash;               /*!< Pointer to hash of message z (Array of prime_order_size_byte elements)    */
  const uint8_t *p_prime_order;        /*!< Pointer to order of the curve n (Array of prime_order_size_byte elements) */
} hal_pka_ecdsa_verif_config_t;


/**
  * @brief PKA elliptic curves over prime fields signature configuration structure definition (protected mode).
  */
typedef struct
{
  uint32_t      prime_order_size_byte; /*!< Number of elements in p_prime_order array                              */
  uint32_t      modulus_size_byte;     /*!< Number of elements in p_modulus array                                  */
  uint32_t      coeff_sign;            /*!< Curve coefficient a sign                                               */
  const uint8_t *p_coeff;              /*!< Pointer to curve coefficient |a| (Array of modulus_size_byte elements) */
  const uint8_t *p_coeff_b;            /*!< Pointer to B coefficient (Array of modulus_size_byte elements)         */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p (Array of modulus_size_byte elements) */
  const uint8_t *p_integer;            /*!< Pointer to random integer k (Array of primeOrderSize elements)         */
  const uint8_t *p_base_pt_x;          /*!< Pointer to curve base point xG (Array of modulus_size_byte elements)   */
  const uint8_t *p_base_pt_y;          /*!< Pointer to curve base point yG (Array of modulus_size_byte elements)   */
  const uint8_t *p_hash;               /*!< Pointer to hash of the message (Array of primeOrderSize elements)      */
  const uint8_t *p_private_key;        /*!< Pointer to private key d (Array of primeOrderSize elements)            */
  const uint8_t *p_prime_order;        /*!< Pointer to order of the curve n (Array of primeOrderSize elements)     */
} hal_pka_ecdsa_signature_protect_config_t;


/**
  * @brief PKA elliptic curves over prime fields output structure definition (protected mode)
  */
typedef struct
{
  uint8_t *p_r_sign; /*!< Pointer to signature part r */
  uint8_t *p_s_sign; /*!< Pointer to signature part s */
} hal_pka_ecdsa_signature_protect_result_t;

/**
  * @brief PKA curve operations output structure definition
  */
typedef struct
{
  uint8_t *p_pt_x; /*!< Pointer to point P coordinate xP */
  uint8_t *p_pt_y; /*!< Pointer to point P coordinate yP */
} hal_pka_ecdsa_signature_result_ext_config_t;

typedef hal_pka_ecdsa_signature_result_ext_config_t hal_pka_ecc_projective_to_affine_result_t;
typedef hal_pka_ecdsa_signature_result_ext_config_t hal_pka_ecc_double_base_ladder_result_t;


typedef hal_pka_ecdsa_signature_result_ext_config_t hal_pka_ecc_mul_protect_result_t;

/**
  * @brief PKA Modular exponentiation configuration structure definition
  */
typedef struct
{
  uint32_t      exponent_size_byte; /*!< Number of elements in p_exponent array                     */
  uint32_t      operand_size_byte;  /*!< Number of elements in p_operand and pMod arrays            */
  const uint8_t *p_exponent;        /*!< Pointer to Exponent (Array of exponent_size_byte elements) */
  const uint8_t *p_operand;         /*!< Pointer to Operand (Array of operand_size_byte elements)   */
  const uint8_t *p_modulus;         /*!< Pointer to modulus (Array of operand_size_byte elements)   */
} hal_pka_mod_exp_config_t;

/**
  * @brief PKA Modular exponentiation protected configuration structure definition
  */
typedef struct
{
  uint32_t      exponent_size_byte; /*!< Size of the exponent in bytes                                    */
  uint32_t      operand_size_byte;  /*!< Size of the operand in bytes                                     */
  const uint8_t *p_operand;         /*!< Pointer to Operand (Array of exponent_size_byte elements)        */
  const uint8_t *p_exponent;        /*!< Pointer to Exponent (Array of operand_size_byte elements)        */
  const uint8_t *p_modulus;         /*!< Pointer to modulus value n (Array of operand_size_byte elements) */
  const uint8_t *p_phi;             /*!< Pointer to Phi value                                             */
} hal_pka_mod_exp_protect_config_t;

/**
  * @brief PKA Modular exponentiation (fast) configuration structure definition
  */
typedef struct
{
  uint32_t      exponent_size_byte;  /*!< Number of elements in p_exponent and p_montgomery_param arrays           */
  uint32_t      operand_size_byte;   /*!< Number of elements in p_operand and p_modulus arrays                     */
  const uint8_t *p_exponent;         /*!< Pointer to Exponent (Array of exponent_size_byte elements)               */
  const uint8_t *p_operand;          /*!< Pointer to Operand (Array of operand_size_byte elements)                 */
  const uint8_t *p_modulus;          /*!< Pointer to modulus (Array of operand_size_byte elements)                 */
  const uint8_t *p_montgomery_param; /*!< Pointer to Montgomery parameter (Array of exponent_size_byte elements)   */
} hal_pka_mod_exp_fast_config_t;

/**
  * @brief PKA Montgomery parameter computation configuration structure definition
  */
typedef struct
{
  uint32_t      size_byte;  /*!< Number of elements in p_operand array            */
  const uint8_t *p_operand; /*!< Pointer to Operand (Array of size_byte elements) */
} hal_pka_montgomery_config_t;

/**
  * @brief PKA Arithmetic configuration structure definition
  */
typedef struct
{
  uint32_t      size_byte;    /*!< Number of elements in p_operand_1 and p_operand_2 arrays */
  const uint8_t *p_operand_1; /*!< Pointer to Operand 1 (Array of size_byte elements)      */
  const uint8_t *p_operand_2; /*!< Pointer to Operand 2 (Array of size_byte elements)      */
} hal_pka_add_config_t, hal_pka_sub_config_t, hal_pka_mul_config_t, hal_pka_cmp_config_t;

/**
  * @brief PKA Modular inversion configuration structure definition
  */
typedef struct
{
  uint32_t      size_byte;  /*!< Number of elements in p_operand and p_modulus arrays     */
  const uint8_t *p_operand; /*!< Pointer to Operand (Array of size_byte elements)         */
  const uint8_t *p_modulus; /*!< Pointer to modulus value n (Array of size_byte elements) */
} hal_pka_mod_inv_config_t;

/**
  * @brief PKA Modular reduction configuration structure definition
  */
typedef struct
{
  uint32_t      operand_size_byte; /*!< Number of elements in p_operand array                            */
  uint32_t      modulus_size_byte; /*!< Number of elements in p_modulus array                            */
  const uint8_t *p_operand;        /*!< Pointer to Operand (Array of operand_size_byte elements)         */
  const uint8_t *p_modulus;        /*!< Pointer to modulus value n (Array of modulus_size_byte elements) */
} hal_pka_mod_red_config_t;

/**
  * @brief PKA Modular arithmetic configuration structure definition
  */
typedef struct
{
  uint32_t      size_byte;    /*!< Number of elements in p_operand_1 and p_operand_2 arrays */
  const uint8_t *p_operand_1; /*!< Pointer to Operand 1 (Array of size_byte elements)      */
  const uint8_t *p_operand_2; /*!< Pointer to Operand 2 (Array of size_byte elements)      */
  const uint8_t *p_operand_3; /*!< Pointer to Operand 3 (Array of size_byte elements)      */
} hal_pka_mod_add_config_t, hal_pka_mod_sub_config_t, hal_pka_montgomery_mul_config_t;

/**
  * @brief PKA ECC double base ladder configuration structure definition
  */
typedef struct
{
  uint32_t      prime_order_size_byte; /*!< Curve prime order n length                           */
  uint32_t      modulus_size_byte;     /*!< Curve modulus p length                               */
  uint32_t      coeff_sign;            /*!< Curve coefficient a sign                             */
  const uint8_t *p_coeff_a;            /*!< Pointer to curve coefficient |a|                     */
  const uint8_t *p_modulus;            /*!< Pointer to curve modulus value p                     */
  const uint8_t *p_integer_k;          /*!< Pointer to cryptographically secure random integer k */
  const uint8_t *p_integer_m;          /*!< Pointer to cryptographically secure random integer m */
  const uint8_t *p_base_pt_x_1;        /*!< Pointer to curve base first point coordinate x       */
  const uint8_t *p_base_pt_y_1;        /*!< Pointer to curve base first point coordinate y       */
  const uint8_t *p_base_pt_z_1;        /*!< Pointer to curve base first point coordinate z       */
  const uint8_t *p_base_pt_x_2;        /*!< Pointer to curve base second point coordinate x      */
  const uint8_t *p_base_pt_y_2;        /*!< Pointer to curve base second point coordinate y      */
  const uint8_t *p_base_pt_z_2;        /*!< Pointer to curve base second point coordinate z      */
} hal_pka_ecc_double_base_ladder_config_t;

/**
  * @brief PKA ECC projective to affine configuration structure definition
  */
typedef struct
{
  uint32_t      modulus_size_byte;   /*!< Curve modulus p length                       */
  const uint8_t *p_modulus;          /*!< Pointer to curve modulus value p             */
  const uint8_t *p_base_pt_x;        /*!< Pointer to curve base point coordinate x     */
  const uint8_t *p_base_pt_y;        /*!< Pointer to curve base point coordinate y     */
  const uint8_t *p_base_pt_z;        /*!< Pointer to curve base point coordinate z     */
  const uint8_t *p_montgomery_param; /*!< Pointer to Montgomery parameter R2 modulus n */
} hal_pka_ecc_projective_to_affine_config_t;

/**
  * @brief PKA ECC complete addition configuration structure definition
  */
typedef struct
{
  uint32_t      modulus_size_byte; /*!< Curve modulus p length                          */
  uint32_t      coeff_sign;        /*!< Curve coefficient a sign                        */
  const uint8_t *p_modulus;        /*!< Pointer to curve modulus value p                */
  const uint8_t *p_coeff_a;        /*!< Pointer to curve coefficient |a|                */
  const uint8_t *p_base_pt_x_1;    /*!< Pointer to curve base first point coordinate x  */
  const uint8_t *p_base_pt_y_1;    /*!< Pointer to curve base first point coordinate y  */
  const uint8_t *p_base_pt_z_1;    /*!< Pointer to curve base first point coordinate z  */
  const uint8_t *p_base_pt_x_2;    /*!< Pointer to curve base second point coordinate x */
  const uint8_t *p_base_pt_y_2;    /*!< Pointer to curve base second point coordinate y */
  const uint8_t *p_base_pt_z_2;    /*!< Pointer to curve base second point coordinate z */
} hal_pka_ecc_complete_add_config_t;

/**
  * @brief PKA output ECC complete addition structure definition
  */
typedef struct
{
  uint8_t *p_pt_x; /*!< Pointer to point P coordinate xP */
  uint8_t *p_pt_y; /*!< Pointer to point P coordinate yP */
  uint8_t *p_pt_z; /*!< Pointer to point P coordinate zP */
} hal_pka_ecc_complete_add_result_t;

typedef struct hal_pka_handle_s hal_pka_handle_t; /*!< PKA Handle Structure type */
#if defined(USE_HAL_PKA_REGISTER_CALLBACKS) && (USE_HAL_PKA_REGISTER_CALLBACKS == 1U)
typedef void (*hal_pka_cb_t)(hal_pka_handle_t *hpka); /*!< PKA Callback pointer definition */
#endif /* USE_HAL_PKA_REGISTER_CALLBACKS */

/**
  * @brief PKA handle Structure definition
  */
struct hal_pka_handle_s
{
  hal_pka_t                instance;            /*!< PKA register base address     */
  volatile hal_pka_state_t global_state;        /*!< PKA state                     */
  uint32_t                 operation;           /*!< PKA operating mode            */
#if defined(USE_HAL_PKA_GET_LAST_ERRORS) && (USE_HAL_PKA_GET_LAST_ERRORS == 1U)
  volatile uint32_t        last_error_codes;    /*!< PKA last error codes          */
#endif /* USE_HAL_PKA_GET_LAST_ERRORS */
#if defined(USE_HAL_PKA_REGISTER_CALLBACKS) && (USE_HAL_PKA_REGISTER_CALLBACKS == 1U)
  hal_pka_cb_t             p_operation_cplt_cb; /*!< PKA End of operation callback */
  hal_pka_cb_t             p_error_cb;          /*!< PKA Last error callback       */
#endif /* USE_HAL_PKA_REGISTER_CALLBACKS */
#if defined(USE_HAL_PKA_USER_DATA) && (USE_HAL_PKA_USER_DATA == 1U)
  const void               *p_user_data;        /*!< PKA user data                 */
#endif /* USE_HAL_PKA_USER_DATA */
};

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_Exported_Functions HAL PKA Functions
  * @{
  */

/** @defgroup PKA_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
/* Initialization and deinitialization functions */
hal_status_t HAL_PKA_Init(hal_pka_handle_t *hpka, hal_pka_t instance);
void         HAL_PKA_DeInit(hal_pka_handle_t *hpka);
/**
  * @}
  */

/** @defgroup PKA_Exported_Functions_Group2 Configuration functions
  * @{
  */
/* PKA arithmetic and modular configuration functions  */
hal_status_t HAL_PKA_SetConfigModExp(hal_pka_handle_t *hpka, const hal_pka_mod_exp_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModExpFast(hal_pka_handle_t *hpka, const hal_pka_mod_exp_fast_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModExpProtect(hal_pka_handle_t *hpka, const hal_pka_mod_exp_protect_config_t *p_config);
hal_status_t HAL_PKA_SetConfigAdd(hal_pka_handle_t *hpka, const hal_pka_add_config_t *p_config);
hal_status_t HAL_PKA_SetConfigSub(hal_pka_handle_t *hpka, const hal_pka_sub_config_t *p_config);
hal_status_t HAL_PKA_SetConfigCmp(hal_pka_handle_t *hpka, const hal_pka_cmp_config_t *p_config);
hal_status_t HAL_PKA_SetConfigMul(hal_pka_handle_t *hpka, const hal_pka_mul_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModAdd(hal_pka_handle_t *hpka, const hal_pka_mod_add_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModSub(hal_pka_handle_t *hpka, const hal_pka_mod_sub_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModInv(hal_pka_handle_t *hpka, const hal_pka_mod_inv_config_t *p_config);
hal_status_t HAL_PKA_SetConfigModRed(hal_pka_handle_t *hpka, const hal_pka_mod_red_config_t *p_config);
hal_status_t HAL_PKA_SetConfigMontgomeryMul(hal_pka_handle_t *hpka, const hal_pka_montgomery_mul_config_t *p_config);
hal_status_t HAL_PKA_SetConfigMontgomery(hal_pka_handle_t *hpka, const hal_pka_montgomery_config_t *p_config);
/* PKA RSA configuration functions */
hal_status_t HAL_PKA_RSA_SetConfigCRTExp(hal_pka_handle_t *hpka, const hal_pka_rsa_crt_exp_config_t *p_config);
hal_status_t HAL_PKA_RSA_SetConfigSignature(hal_pka_handle_t *hpka, const hal_pka_rsa_signature_config_t *p_config);
hal_status_t HAL_PKA_RSA_SetConfigSignatureFast(hal_pka_handle_t *hpka,
                                                const hal_pka_rsa_signature_fast_config_t *p_config);
hal_status_t HAL_PKA_RSA_SetConfigSignatureProtect(hal_pka_handle_t *hpka,
                                                   const hal_pka_rsa_signature_protect_config_t *p_config);
hal_status_t HAL_PKA_RSA_SetConfigVerifSignature(hal_pka_handle_t *hpka, const hal_pka_rsa_verif_config_t *p_config);
/* PKA ECDSA configuration functions */
hal_status_t HAL_PKA_ECDSA_SetConfigSignatureProtect(hal_pka_handle_t *hpka,
                                                     const hal_pka_ecdsa_signature_protect_config_t *p_config);
hal_status_t HAL_PKA_ECDSA_SetConfigVerifSignature(hal_pka_handle_t *hpka,
                                                   const hal_pka_ecdsa_verif_config_t *p_config);
/* PKA ECC configuration functions */
hal_status_t HAL_PKA_ECC_SetConfigPointCheck(hal_pka_handle_t *hpka, const hal_pka_point_check_config_t *p_config);
hal_status_t HAL_PKA_ECC_SetConfigMulProtect(hal_pka_handle_t *hpka, const hal_pka_ecc_mul_protect_config_t *p_config);
hal_status_t HAL_PKA_ECC_SetConfigDoubleBaseLadder(hal_pka_handle_t *hpka,
                                                   const hal_pka_ecc_double_base_ladder_config_t *p_config);
hal_status_t HAL_PKA_ECC_SetConfigProjectiveToAffine(hal_pka_handle_t *hpka,
                                                     const hal_pka_ecc_projective_to_affine_config_t *p_config);
hal_status_t HAL_PKA_ECC_SetConfigCompleteAdd(hal_pka_handle_t *hpka,
                                              const hal_pka_ecc_complete_add_config_t *p_config);
/**
  * @}
  */

/** @defgroup PKA_Exported_Functions_Group3 Process management functions
  * @{
  */
hal_status_t HAL_PKA_Compute(hal_pka_handle_t *hpka, uint32_t timeout_ms);
hal_status_t HAL_PKA_Compute_IT(hal_pka_handle_t *hpka);
void         HAL_PKA_IRQHandler(hal_pka_handle_t *hpka);
hal_status_t HAL_PKA_Abort(hal_pka_handle_t *hpka);
/* PKA arithmetic and modular get result functions */
uint32_t HAL_PKA_GetResultModExp(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModExpFast(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModExpProtect(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultAdd(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultSub(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultCmp(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultMul(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModAdd(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModSub(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModInv(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultModRed(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultMontgomeryMul(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t HAL_PKA_GetResultMontgomery(hal_pka_handle_t *hpka, uint8_t *p_result);
/* PKA RSA get result functions */
uint32_t                      HAL_PKA_RSA_GetResultCRTExp(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t                      HAL_PKA_RSA_GetResultSignature(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t                      HAL_PKA_RSA_GetResultSignatureFast(hal_pka_handle_t *hpka, uint8_t *p_result);
uint32_t                      HAL_PKA_RSA_GetResultSignatureProtect(hal_pka_handle_t *hpka, uint8_t *p_result);
hal_pka_rsa_signature_status_t HAL_PKA_RSA_IsValidVerifSignature(hal_pka_handle_t *hpka, const uint8_t *p_hash);
/* PKA ECDSA get result functions */
uint32_t HAL_PKA_ECDSA_GetResultSignatureProtect(hal_pka_handle_t *hpka,
                                                 hal_pka_ecdsa_signature_protect_result_t *p_result,
                                                 hal_pka_ecdsa_signature_result_ext_config_t *p_result_ext);
hal_pka_ecdsa_signature_status_t HAL_PKA_ECDSA_IsValidVerifSignature(const hal_pka_handle_t *hpka);
/* PKA ECC get result functions */
hal_pka_ecc_point_status_t HAL_PKA_ECC_IsPointCheckOnCurve(const hal_pka_handle_t *hpka);
uint32_t                  HAL_PKA_ECC_GetResultMulProtect(hal_pka_handle_t *hpka,
                                                          hal_pka_ecc_mul_protect_result_t *p_result);
uint32_t                  HAL_PKA_ECC_GetResultDoubleBaseLadder(hal_pka_handle_t *hpka,
                                                                hal_pka_ecc_double_base_ladder_result_t *p_result);
uint32_t                  HAL_PKA_ECC_GetResultProjectiveToAffine(hal_pka_handle_t *hpka,
                                                                  hal_pka_ecc_projective_to_affine_result_t *p_result);
uint32_t                  HAL_PKA_ECC_GetResultCompleteAdd(hal_pka_handle_t *hpka,
                                                           hal_pka_ecc_complete_add_result_t *p_result);
/**
  * @}
  */

/** @defgroup PKA_Exported_Functions_Group4 Callback functions
  * @{
  */
void         HAL_PKA_OperationCpltCallback(hal_pka_handle_t *hpka);
void         HAL_PKA_ErrorCallback(hal_pka_handle_t *hpka);
#if defined(USE_HAL_PKA_REGISTER_CALLBACKS) && (USE_HAL_PKA_REGISTER_CALLBACKS == 1U)
hal_status_t HAL_PKA_RegisterOperationCpltCallback(hal_pka_handle_t *hpka, hal_pka_cb_t p_callback);
hal_status_t HAL_PKA_RegisterErrorCallback(hal_pka_handle_t *hpka, hal_pka_cb_t p_callback);
#endif /* USE_HAL_PKA_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup PKA_Exported_Functions_Group5 State and Error functions
  * @{
  */
/* Peripheral State and Error functions */
hal_pka_state_t HAL_PKA_GetState(const hal_pka_handle_t *hpka);
#if defined(USE_HAL_PKA_GET_LAST_ERRORS) && (USE_HAL_PKA_GET_LAST_ERRORS == 1U)
uint32_t        HAL_PKA_GetLastErrorCodes(const hal_pka_handle_t *hpka);
#endif /* USE_HAL_PKA_GET_LAST_ERRORS */
#if defined(USE_HAL_PKA_USER_DATA) && (USE_HAL_PKA_USER_DATA == 1U)
void            HAL_PKA_SetUserData(hal_pka_handle_t *hpka, const void *p_user_data);
const void      *HAL_PKA_GetUserData(const hal_pka_handle_t *hpka);
#endif /* USE_HAL_PKA_USER_DATA */
/**
  * @}
  */

/** @defgroup PKA_Exported_Functions_Group6 PKA RAM Mass Erase function
  * @{
  */
hal_status_t HAL_PKA_RAMMassErase(hal_pka_handle_t *hpka);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* PKA */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_PKA_H */
