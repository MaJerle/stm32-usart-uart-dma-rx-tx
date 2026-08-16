/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_ccb.c
  * @brief   CCB HAL module driver
  *          This file provides firmware functions to manage the following functionalities of the Coupling and
  *          Chaining Bridge (CCB) peripheral:
  *           + Initialization and de-initialization functions
  *           + I/O operation functions
  *           + Peripheral state and error functions
  *
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

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(CCB)
#if defined(USE_HAL_CCB_MODULE) && (USE_HAL_CCB_MODULE == 1)
/** @addtogroup CCB
  * @{
  */
/** @defgroup CCB_Introduction CCB Introduction
  * @{

  The CCB hardware abstraction layer provides a set of APIs to configure and control the CCB peripheral on
  STM32 microcontrollers.
  The CCB (coupling and chaining bridge) is a cryptographic hardware accelerator designed to protect private keys
  used in PKA-protected operations.

  */
/**
  * @}
  */

/** @defgroup CCB_How_To_Use CCB How To Use
  * @{
## The CCB main features:

The CCB HAL driver is designed to implement specialized coupling and chaining operations
that secure these three PKA operations (modular exponentiation, scalar multiplication, ECDSA signature)
within dedicated sequences. To perform these operations, the CCB driver requires PKA, SAES and RNG peripherals.
- Each PKA protected operation involves two processes:
  - A blob creation which is a one-time sequence to protect private keys used in PKA protected operations.
  - A blob usage, which is a sequence that can be executed many times to run a PKA protected operation.
- The driver can encrypt, with AES-256, any PKA blob encryption key, which makes the PKA encrypted blob
  usable only on this device.

# How to use the HAL CCB driver
- The CCB driver is secure because it ensures isolation and protects the confidentiality and integrity of private keys.
- The CCB driver allows applications to access its services through dedicated APIs, providing controlled access
  to protected operations.
## The HAL CCB driver can be used as follows:

- Initialize the CCB handle by calling the HAL_CCB_Init() API which performs the following operations:
  - Associate the instance to the handle.
  - Enable the CCB clock interface (when USE_HAL_CCB_CLK_ENABLE_MODEL compilation flag is set to
    HAL_CLK_ENABLE_PERIPH_ONLY or HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM in the stm32c5xx_hal_conf.h module)
  - Initialize the handle state to the HAL_CCB_STATE_IDLE.
\note No configuration is required after initializing the CCB driver. Once the CCB handle is initialized
using the HAL_CCB_Init() API, the driver is ready to start a process.

- De-initialize the CCB peripheral by calling the HAL_CCB_DeInit() API, which performs the following operations:
  - Disable of the CCB peripheral.
  - De-initialize the current handle object.
  - Reset the handle state to the HAL_CCB_STATE_RESET.

- Blob creation operation:
 - Wrap a private key, either a clear text (plaintext) user-supplied key or an RNG-generated key,
  using either a hardware key or a wrapped symmetric software key to create the key blob.
- When using a hardware key as a wrapper:
  - Call either:
         - HAL_CCB_ECDSA_HW_WrapPrivateKey() to wrap a clear text key for ECDSA purposes.
         - HAL_CCB_ECDSA_HW_GenerateWrapPrivateKey() to wrap an RNG-generated key for ECDSA purposes.
- For both APIs, specify the hardware key type. Similar APIs are provided to support ECC and RSA operations.
- When using a software key as a wrapper:
        - First wrap this software key using HAL_CCB_ECDSA_WrapSymmetricKey().
        - The result (wrapped software key) is then used as a wrapper to create the blob.
        - Call either:
          - HAL_CCB_ECDSA_SW_WrapPrivateKey() to wrap a clear text key for ECDSA purposes.
          - HAL_CCB_ECDSA_SW_GenerateWrapPrivateKey() to wrap an RNG-generated key for ECDSA purposes.
      Similar APIs are provided to support ECC and RSA operations.

- Blob usage operation:
  - Unwrap the created key blob to execute PKA protected operations:
  - When using a hardware-wrapped key blob:
      - The user can call either:
         - HAL_CCB_ECDSA_HW_Sign() to sign a message using the hardware-wrapped ECDSA private key blob.
         - HAL_CCB_ECDSA_HW_ComputePublicKey() to compute the public key using the hardware-wrapped ECDSA private key
           blob.
         - HAL_CCB_ECC_HW_ComputeScalarMul() to perform ECC scalar multiplication using the hardware-wrapped
           ECC private key blob.
         - HAL_CCB_RSA_HW_ComputeModularExp() to perform RSA modular exponentiation using the hardware-wrapped
           RSA private key blob.
      - For HW APIs, specify the hardware key type.
  - When using a software-wrapped key blob:
      - The user can call either:
         - HAL_CCB_ECDSA_SW_Sign() to sign a message using the software-wrapped ECDSA private key blob.
         - HAL_CCB_ECDSA_SW_ComputePublicKey() to compute the public key from the software-wrapped
           ECDSA private key blob.
         - HAL_CCB_ECC_SW_ComputeScalarMul() to perform ECC scalar multiplication using the software-wrapped
           ECC private key blob.
         - HAL_CCB_RSA_SW_ComputeModularExp() to perform RSA modular exponentiation using the software-wrapped
           RSA private key blob.

- Set the compilation flag USE_HAL_CCB_USER_DATA to 1U in the stm32c5xx_hal_conf.h module to allow storing
  (into the handle) and retrieving the user data respectively through the HAL_CCB_SetUserData()
  and HAL_CCB_GetUserData() APIs.

  */
/**
  * @}
  */

/** @defgroup CCB_Configuration_Table CCB Configuration Table
  * @{

# Configuration inside the CCB driver

Config defines               | Description           | Default value     | Note
-----------------------------| ----------------------| ------------------| ----------------------------------------
PRODUCT                      | from IDE              | NA                | Ex:STM32C5XXxx.
USE_ASSERT_DBG_PARAM         | from IDE              | None              | Enable the parameters asserts.
USE_ASSERT_DBG_STATE         | from IDE              | None              | Enable the state asserts.
USE_HAL_CHECK_PARAM          | from hal_conf.h       | 0U                | Parameters runtime check.
USE_HAL_SECURE_CHECK_PARAM   | from hal_conf.h       | 0U                | Parameters runtime check for sensitive APIs.
USE_HAL_CCB_MODULE           | from hal_conf.h       | 1U                | Enable the HAL CCB module.
USE_HAL_CCB_CLK_ENABLE_MODEL | from hal_conf.h       | HAL_CLK_ENABLE_NO | Enable the HAL_CCB_CLK.
USE_HAL_CCB_GET_LAST_ERRORS  | from hal_conf.h       | 0U                | Allows retrieving the last error codes.
USE_HAL_CCB_USER_DATA        | from hal_conf.h       | 0U                | Allows enabling or disabling user data.
  */
/**
  * @}
  */

/* Private constants--------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Private_Constants CCB Private Constants
  * @{
  */
/*! CCB_PKA_Mode CCB PKA mode */
#define CCB_PKA_MODULAR_EXP_PROTECT_MODE      (PKA_CR_MODE_0 | PKA_CR_MODE_1) /*!< Modular exponentiation protected
                                                                                   mode                               */
#define CCB_PKA_ECC_MUL_PROTECT_MODE          (PKA_CR_MODE_5)                 /*!< ECC scalar multiplication protected
                                                                                   mode                               */
#define CCB_PKA_ECDSA_SIGNATURE_PROTECT_MODE  (PKA_CR_MODE_5 | PKA_CR_MODE_2) /*!< ECDSA signature protected mode     */
#define CCB_PKA_ECDSA_VERIFICATION_MODE       (PKA_CR_MODE_5 | PKA_CR_MODE_2 | PKA_CR_MODE_1) /*!< ECDSA verification */

/*! CCB_PKA_RAM_SIZE CCB PKA RAM size */
#define CCB_PKA_RAM_SIZE                     (0x00000536U)  /*!< CCB PKA RAM size */

/*! CCB_Operations CCB Operations */
#define CCB_ECDSA_SIGN_CPU_BLOB_CREATION     (0x000000C0U)  /*!< ECDSA signature blob creation with private key
                                                                 from the CPU                                   */
#define CCB_ECDSA_SIGN_RNG_BLOB_CREATION     (0x000000C2U)  /*!< ECDSA signature blob creation with private key
                                                                 from the RNG                                   */
#define CCB_ECDSA_SIGN_BLOB_USE              (0x000000C3U)  /*!< ECDSA signature blob usage for ECDSA signature   */
#define CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION (0x00000080U)  /*!< ECC scalar multiplication blob creation with
                                                                 private key from the CPU                       */
#define CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION (0x00000082U)  /*!< ECC scalar multiplication blob creation with
                                                                 private key from the RNG                       */
#define CCB_ECC_SCALAR_MUL_BLOB_USE          (0x00000081U)  /*!< ECC Scalar multiplication blob use, or ECDSA
                                                                 signature blob use for public key computation  */
#define CCB_MODULAR_EXP_CPU_BLOB_CREATION    (0x00000044U)  /*!< Modular exponentiation blob creation with priv
                                                                 key from the CPU                               */
#define CCB_MODULAR_EXP_BLOB_USE             (0x00000045U)  /*!< Modular exponentiation blob usage                */

/*! CCB_PKA_Result CCB PKA result */
#define CCB_PKA_RESULT_OK                    (0x0000D60DU)  /*!< PKA operation result is OK and no PKA Hardware
                                                                 operation error                               */

/*! CCB several values */
#define CCB_FAKE_VALUE               (0x0001UL)        /*!< Fake value used for SAES_IVRs                             */
#define CCB_MAGIC_VALUE              (0x0CCBUL)        /*!< Magic number used when chaining key from SAES to PKA RAM  */
#define CCB_IV0_VALUE                (0x0002UL)        /*!< SAES_IVR0 that must be equal to 0x2                       */
#define CCB_BLOCK_SIZE_WORD          (0x0004UL)        /*!< Block size is 128 bits (4*32 bits)                        */
#define CCB_SYMMETRIC_KEY_SIZE_WORD  (0x0008UL)        /*!< Symmetric key size is always 256 (8*32 bits)              */

/*! CCB Timeout Values */
#define CCB_GENERAL_TIMEOUT_MS      0x1000U          /*!< General CCB operation timeout in milliseconds */

#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
#define CCB_RNG_TIMEOUT_VALUE       0x00000002U      /*!< RNG timeout value in milliseconds */
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* CCB_RNG_TIMEOUT_VALUE */

/*! Number of bytes in one 32-bit word */
#define CCB_BYTES_PER_WORD    4U

/*! Number of 32-bit words in one block */
#define CCB_WORDS_PER_BLOCK   2U

/*! CCB key type */
#define CCB_KEY_TYPE_SOFTWARE 0U  /*!< CCB key type software */
#define CCB_KEY_TYPE_HARDWARE 1U  /*!< CCB key type hardware */
/**
  * @}
  */

/* Private types--------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Private_Types CCB Private Types
  * @{
  */

/*! CCB software unwrap key context struct */
typedef struct
{
  uint32_t ccb_key_type;                                            /*!< CCB key type */
  const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context;  /*!< Pointer to the software key context */
  const uint32_t *p_in_wrapped_user_key;                            /*!< Pointer to the wrapped user key */
} ccb_sw_unwrap_key_context_t;

/*! CCB hardware unwrap key context struct */
typedef struct
{
  uint32_t ccb_key_type;                                   /*!< CCB key type */
  hal_ccb_wrapping_hw_key_type_t wrapping_key_context;     /*!< wrapping key context, can be a value
                                                                of @ref hal_ccb_wrapping_hw_key_type_t */
} ccb_hw_unwrap_key_context_t;
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup CCB_Private_Macros CCB Private Macros
  * @{
  */
/*! Macro to get the handle instance */
#define CCB_GET_INSTANCE(handle) ((CCB_TypeDef *)((uint32_t)(handle)->instance))

/*! Macro to get PKA flag */
#define CCB_GET_PKA_FLAG(pka_instance, flag)              (((((pka_instance)->SR) & (flag)) == (flag)) ? 1U : 0U)

/*! Macro to clear PKA flag */
#define CCB_CLEAR_PKA_FLAG(pka_instance, flag)            STM32_WRITE_REG(((pka_instance)->CLRFR), (flag))

/*! Macro to check the AES chaining mode */
#define IS_CCB_AES_MODE(aes_algorithm) (((aes_algorithm) == HAL_CCB_AES_ECB) \
                                        || ((aes_algorithm) == HAL_CCB_AES_CBC))

/*! Macro to check the hardware key type */
#define IS_CCB_HW_KEY_TYPE(key_type) (((key_type) == AES_CR_KEYSEL_0) \
                                      || ((key_type) == AES_CR_KEYSEL_2))

/*! Macro to access a 32-bit word in the PKA RAM */
#define CCB_PKA_RAM_WORD_ACCESS(pka_instance, idx) (*(volatile uint32_t *)(uint32_t) \
                                                    & ((PKA_TypeDef *)pka_instance)->RAM[idx * sizeof(uint32_t)])

/*! Macro to check the provided CCB ECDSA pool buffer size */
#define IS_CCB_ECDSA_POOL_BUFFER_SIZE(buff_size_byte, modulus_size) (((buff_size_byte) \
                                                                      >= HAL_CCB_ECDSA_CALC_BUFFER_SIZE(modulus_size)))

/*! Macro to check the provided CCB ECC pool buffer size */
#define IS_CCB_ECC_POOL_BUFFER_SIZE(buff_size_byte, modulus_size) (((buff_size_byte) \
                                                                    >= HAL_CCB_ECC_CALC_BUFFER_SIZE(modulus_size)))

/*! Macro to check the provided CCB RSA pool buffer size */
#define IS_CCB_RSA_POOL_BUFFER_SIZE(buff_size_byte, modulus_size) (((buff_size_byte) \
                                                                    >= HAL_CCB_RSA_CALC_BUFFER_SIZE(modulus_size)))
/**
  * @}
  */

/* private function prototypes --------------------------------------------------------------------------------------*/
/** @defgroup CCB_Private_Functions CCB Private Functions
  * @{
  */
/* Initialization private function */
static hal_status_t CCB_PKA_Init(PKA_TypeDef *p_pka_instance);
static hal_status_t CCB_RNG_Init(RNG_TypeDef *p_rng_instance);

/* Software reset private function */
static void CCB_RESET(const hal_ccb_handle_t *hccb);

/* Polling private function */
static hal_status_t CCB_WaitOperStep(CCB_TypeDef const *p_ccb_instance, uint32_t step);
static hal_status_t CCB_WaitFLAG(CCB_TypeDef const *p_ccb_instance, uint32_t flag);
static hal_status_t CCB_PKA_WaitFlag(PKA_TypeDef *p_pka_instance, uint32_t flag);
static hal_status_t CCB_RNG_WaitFlag(RNG_TypeDef *p_rng_instance, uint32_t flag);
static hal_status_t CCB_SAES_WaitFlag(AES_TypeDef *p_saes_instance, uint32_t flag, uint32_t status);

/* Set parameters private function */
static hal_status_t CCB_ECDSASign_SetParam(const hal_ccb_ecdsa_curve_param_t *p_in_curve_param);
static hal_status_t CCB_ECCMul_SetParam(const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param);
static hal_status_t CCB_RSAModExp_SetParam(const hal_ccb_rsa_param_t *p_in_curve_param);
static hal_status_t CCB_SetParam(uint32_t modulus_size_byte, uint32_t dst_address, const uint8_t *p_src);

/* Wrapping key private function */
static hal_status_t CCB_Wrapping_Key_Config(void *p_unwrapkey_context, uint8_t ccb_operation);

static hal_status_t CCB_WrapSymmetricKey(CCB_TypeDef *p_ccb_instance, const uint32_t *p_in_clear_user_key,
                                         uint32_t operation,
                                         const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                         uint32_t *p_out_wrapped_user_key);

/* Data management private function */
static uint32_t CCB_GetOptBitSize_u8(uint32_t byte_number, uint8_t msb);
static void CCB_Memcpy_u32_to_u8(volatile uint8_t *p_dst, const volatile uint32_t *p_src, size_t size);
static void CCB_Memcpy_u8_to_u32(volatile uint32_t *p_dst, const volatile  uint8_t *p_src, size_t size);
static void CCB_Memcpy_u32_to_u32(volatile uint32_t *p_dst, const volatile uint32_t *p_src, size_t size);
static void CCB_Memcpy_u8_to_u64(volatile uint32_t *p_dst, const volatile uint8_t *p_src);
static void CCB_Memcpy_TailBytesToU64(volatile uint32_t *p_dst, const volatile uint8_t *p_src, size_t size);

/* Blob processing private function */
static hal_status_t CCB_BlobCreation_InitialPhase(uint32_t *p_iv, uint32_t randoms);
static hal_status_t CCB_BlobUse_InitialPhase(CCB_TypeDef *p_ccb_instance, const uint32_t *p_iv, uint32_t *p_tag,
                                             uint32_t randoms);
static hal_status_t CCB_BlobCreation_FinalPhase(uint32_t operation, uint32_t *p_tag, uint32_t size_param,
                                                uint32_t randoms);
static hal_status_t CCB_BlobUse_FinalPhase(AES_TypeDef *p_saes_instance, uint32_t operation,
                                           uint32_t size_param);
static hal_status_t CCB_ECDSA_SignBlobCreation(CCB_TypeDef *p_ccb_instance,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               void *p_unwrapkey_context, const uint8_t *p_clear_private_key,
                                               uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_key,
                                               uint32_t randoms, uint8_t *p_hash, hal_ccb_ecdsa_sign_t *p_signature,
                                               uint8_t ccb_operation);
static hal_status_t CCB_ECDSA_ComputePublicKey(CCB_TypeDef *p_ccb_instance,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               void *p_unwrapkey_context, uint32_t *p_iv, uint32_t *p_tag,
                                               uint32_t *p_wrapped_key, uint32_t randoms,
                                               hal_ccb_ecc_point_t *p_output_point);
static hal_status_t CCB_ECDSA_Sign(CCB_TypeDef *p_ccb_instance, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   void *p_unwrapkey_context, hal_ccb_ecdsa_key_blob_t *p_in_ecdsa_key_blob,
                                   const uint8_t *p_in_hash, uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature);
static hal_status_t CCB_ECC_ScalarMulBlobCreation(CCB_TypeDef *p_ccb_instance,
                                                  const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                  void *p_unwrapkey_context, const uint8_t *p_clear_private_key,
                                                  uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_key,
                                                  uint32_t randoms, uint32_t *scalar_mul_x_ref,
                                                  uint32_t *scalar_mul_y_ref, uint8_t ccb_operation);
static hal_status_t CCB_RSA_ExpBlobCreation(CCB_TypeDef *p_ccb_instance, const hal_ccb_rsa_param_t *p_param,
                                            void *p_unwrapkey_context,
                                            const hal_ccb_rsa_clear_key_t *p_rsa_clear_private_key,
                                            uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_exp,
                                            uint32_t *p_wrapped_phi, uint32_t randoms, uint8_t *p_operand,
                                            uint32_t *p_reference_modular_exp);
static hal_status_t CCB_ECC_ComputeScalarMul(CCB_TypeDef *p_ccb_instance,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             void *p_unwrapkey_context, uint32_t *p_iv, uint32_t *p_tag,
                                             uint32_t *p_wrapped_key, uint32_t randoms,
                                             const hal_ccb_ecc_point_t *p_input_point,
                                             hal_ccb_ecc_point_t *p_output_point);
static hal_status_t CCB_RSA_ComputeModularExp(CCB_TypeDef *p_ccb_instance, const hal_ccb_rsa_param_t *p_param,
                                              void *p_unwrapkey_context, uint32_t *p_iv, uint32_t *p_tag,
                                              uint32_t *p_wrapped_exp, uint32_t *p_wrapped_phi, uint32_t randoms,
                                              const uint8_t *p_operand, uint8_t *p_modular_exp);
static hal_status_t CCB_PKA_ECDSASetConfigVerifSignature(PKA_TypeDef *p_pka_instance,
                                                         const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                         hal_ccb_ecc_point_t *p_public_key_out, const uint8_t *p_hash,
                                                         hal_ccb_ecdsa_sign_t *p_signature);
static hal_status_t CCB_PKA_SetOperation(PKA_TypeDef *p_pka_instance, uint32_t operation);
static hal_status_t CCB_SAES_SW_UnwrapKey(AES_TypeDef *p_saes_instance, const void *p_unwrapkey_context);
static hal_status_t CCB_RNG_GenerateRandomNumbers(RNG_TypeDef *p_rng_instance, uint16_t *p_randoms);
static hal_status_t CCB_RNG_GenerateHashMessage(RNG_TypeDef *p_rng_instance, uint8_t *p_hash, uint32_t hash_size);
static inline void CCB_PKA_PadEndRam(volatile uint32_t *p_pka_ram, uint32_t index);
static inline uint32_t CCB_SAES_GetFlag(AES_TypeDef const *p_saes_instance, uint32_t flag);
static inline void CCB_PKA_WriteClearTextData(volatile uint32_t *p_pka_ram, uint16_t dst_address, const uint8_t *p_src,
                                              uint32_t modulus_size_byte, uint32_t operand_size);
static inline hal_status_t CCB_WriteWrappedKey(uint16_t dst_address, const uint32_t *p_wrapped_key, uint32_t size_byte,
                                               uint32_t randoms);
static inline hal_status_t CCB_WriteKeyFromRNG(uint16_t dst_address, uint32_t operand_size);
static inline hal_status_t CCB_ReadCipheredPrivateKey(uint16_t dst_address, uint32_t operand_size,
                                                      uint32_t *p_wrapped_key, uint32_t randoms);

/* Pool buffer erase private function */
static void CCB_ErasePoolBuffer(uint8_t *p_buff, uint32_t buff_size_byte);

#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
hal_status_t CCB_RNG_ResilientRecoverSeedError(void);
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* USE_HAL_CCB_RNG_RECOVERY */
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup CCB_Exported_Functions
  * @{
  */

/** @addtogroup CCB_Exported_Functions_Group1
  * @{
This subsection provides a set of functions to initialize and de-initialize the CCB peripheral:
- The HAL_CCB_Init() API: Allows initializing the HAL CCB driver so it can be used for blob creation or usage.
  This API is the first API to call when using the HAL CCB, it takes 2 parameters as input:
 - The HAL CCB handle: A pointer to a @ref hal_ccb_handle_t structure
 - The CCB instance  : A value from the enumeration  @ref hal_ccb_t
- The HAL_CCB_DeInit() API: Allows de-initializing the HAL CCB driver by:
   - Disabling the CCB peripheral
   - De-initializing the current handle object
   - Resetting the handle global state to the **HAL_CCB_STATE_RESET**
  */

/**
  * @brief  Initialize the HAL CCB handle and associate it to an instance.
  * @param  hccb              Pointer to a @ref hal_ccb_handle_t structure
  * @param  instance          @ref hal_ccb_t enumerated type variable to be set according to the physical instance
  * @retval HAL_INVALID_PARAM Invalid param return when the CCB handle is NULL
  * @retval HAL_OK            The HAL CCB driver is initialized according to the given handle and instance
  */
hal_status_t HAL_CCB_Init(hal_ccb_handle_t *hccb, hal_ccb_t instance)
{
  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM((IS_CCB_ALL_INSTANCE((CCB_TypeDef *)(uint32_t)instance)));

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
       || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  hccb->instance = instance;

#if defined(USE_HAL_CCB_USER_DATA) && (USE_HAL_CCB_USER_DATA == 1)
  hccb->p_user_data = NULL;
#endif /* (USE_HAL_CCB_USER_DATA) */

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

#if defined(USE_HAL_CCB_CLK_ENABLE_MODEL) && (USE_HAL_CCB_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_CCB_EnableClock();
  HAL_RCC_RNG_EnableClock();
  HAL_RCC_PKA_EnableClock();
  HAL_RCC_SAES_EnableClock();
#endif /* USE_HAL_CCB_CLK_ENABLE_MODEL */

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  De-initialize the CCB peripheral.
  * @param  hccb Pointer to a @ref hal_ccb_handle_t structure
  */
void HAL_CCB_DeInit(hal_ccb_handle_t *hccb)
{
  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM((IS_CCB_ALL_INSTANCE((CCB_TypeDef *)(uint32_t)hccb->instance)));

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup CCB_Exported_Functions_Group2
  * @{
This subsection provides a CCB Reset API :
- HAL_CCB_Reset() allows either aborting an ongoing CCB operation or recovering from an unrecoverable error by
  resetting all related peripherals including CCB, SAES, PKA and RNG.
  */

/**
  * @brief Reset the ongoing operation and clear stored SAES and PKA data.
  * @param  hccb              Pointer to a @ref hal_ccb_handle_t.
  */
void HAL_CCB_Reset(hal_ccb_handle_t *hccb)
{
  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_ACTIVE | (uint32_t)HAL_CCB_STATE_FAULT);

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
}
/**
  * @}
  */

/** @addtogroup CCB_Exported_Functions_Group3
  * @{
This subsection provides a set of functions to wrap the clear symmetric key:

- HAL_CCB_ECDSA_WrapSymmetricKey(): Allows wrapping the clear symmetric key for ECDSA operations.
- HAL_CCB_ECC_WrapSymmetricKey(): Allows wrapping the clear symmetric key for ECC operations.
- HAL_CCB_RSA_WrapSymmetricKey(): Allows wrapping the clear symmetric key for RSA operations.
  */

/**
  * @brief  Wrap the clear symmetric key for ECDSA operations.
  * @param  hccb                      Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_user_key             Pointer to the clear AES key
  * @param  p_wrapping_key_context    Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_out_wrapped_user_key    Pointer to the wrapped user key
  * @retval HAL_INVALID_PARAM         The function return an Invalid param in the following cases: \n
  *                                   - The CCB handle is NULL \n
  *                                   - Input user key pointer is NULL \n
  *                                   - Wrapping key context pointer is NULL \n
  *                                   - Output wrapped user key pointer is NULL \n
  * @retval HAL_ERROR                 Error detected
  * @retval HAL_OK                    Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                            hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                            uint32_t *p_out_wrapped_user_key)
{
  CCB_TypeDef *p_ccb_instance = NULL;

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_user_key != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(p_out_wrapped_user_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
|| (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_user_key == NULL) || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_out_wrapped_user_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_WrapSymmetricKey(p_ccb_instance, p_in_user_key, CCB_ECDSA_SIGN_CPU_BLOB_CREATION,
                           p_wrapping_key_context, p_out_wrapped_user_key) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Wrap the clear symmetric key for ECC operations.
  * @param  hccb                    Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_user_key           Pointer to the clear AES key
  * @param  p_wrapping_key_context  Pointer to the wrapping key context
  * @param  p_out_wrapped_user_key  Pointer to the wrapped user key
  * @retval HAL_INVALID_PARAM       The function return an Invalid param in the following cases: \n
                                    - The CCB handle is NULL \n
                                    - Input user key is NULL \n
                                    - Wrapping key context pointer key is NULL \n
                                    - Output wrapped user key pointer is NULL \n
  * @retval HAL_ERROR               Error detected
  * @retval HAL_OK                  Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                          hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                          uint32_t *p_out_wrapped_user_key)
{
  CCB_TypeDef *p_ccb_instance = NULL;

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_user_key != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(p_out_wrapped_user_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
|| (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_user_key == NULL) || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_out_wrapped_user_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_WrapSymmetricKey(p_ccb_instance, p_in_user_key, CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION,
                           p_wrapping_key_context, p_out_wrapped_user_key) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Wrap the clear symmetric key for RSA operations.
  * @param  hccb                      Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_user_key             Pointer to the clear AES key
  * @param  p_wrapping_key_context    Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_out_wrapped_user_key    Pointer to the wrapped user key
  * @retval HAL_INVALID_PARAM         The function return an Invalid param in the following cases: \n
                                      - The CCB handle is NULL \n
                                      - The CCB handle is NULL \n
                                      - Input user key is NULL \n
                                      - Wrapping key context pointer key is NULL \n
                                      - Output wrapped user key pointer is NULL \n
  * @retval HAL_ERROR                 Error detected
  * @retval HAL_OK                    Operation completed successfully
  */
hal_status_t HAL_CCB_RSA_WrapSymmetricKey(hal_ccb_handle_t *hccb, const uint32_t *p_in_user_key,
                                          hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                          uint32_t *p_out_wrapped_user_key)
{
  CCB_TypeDef *p_ccb_instance = NULL;

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_user_key != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(p_out_wrapped_user_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
|| (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_user_key == NULL) || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_out_wrapped_user_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_WrapSymmetricKey(p_ccb_instance, p_in_user_key, CCB_MODULAR_EXP_CPU_BLOB_CREATION,
                           p_wrapping_key_context, p_out_wrapped_user_key) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup CCB_Exported_Functions_Group4
  * @{
This subsection provides a set of functions allowing to create and use a dedicated blob using a
wrapped symmetric software key.

ECDSA operations:
- HAL_CCB_ECDSA_SW_WrapPrivateKey(): Allows ECDSA Blob creation by wrapping user private key
  with a wrapped symmetric software key.
- HAL_CCB_ECDSA_SW_GenerateWrapPrivateKey(): Allows ECDSA Blob creation by wrapping an RNG generated private key
  with a wrapped symmetric software key.
- HAL_CCB_ECDSA_SW_Sign(): Allows ECDSA Blob usage for ECDSA signature with a wrapped symmetric software key.
- HAL_CCB_ECDSA_SW_ComputePublicKey(): Allows ECDSA Blob usage for public key computation
  with a wrapped symmetric software key.

ECC operations:
- HAL_CCB_ECC_SW_WrapPrivateKey(): Allows ECC Blob creation by wrapping user private key
  with a wrapped symmetric software key.
- HAL_CCB_ECC_SW_GenerateWrapPrivateKey(): Allows ECC Blob creation by wrapping an RNG generated private key
  with a wrapped symmetric software key.
- HAL_CCB_ECC_SW_ComputeScalarMul(): Allows ECC Blob usage for ECC compute scalar multiplication
  with a wrapped symmetric software key.

RSA operations:
- HAL_CCB_RSA_SW_WrapPrivateKey(): Allows RSA Blob creation by wrapping user private key
  with a wrapped symmetric software key.
- HAL_CCB_RSA_SW_ComputeModularExp(): Allows RSA Blob usage for RSA modular exponentiation
  with a wrapped symmetric software key.
  */

/**
  * @brief  Blob Creation: ECDSA Wrapping private Key with a wrapped symmetric software key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_in_clear_private_key          Pointer to the clear private key
  * @param  p_wrapping_key_context          Pointer to @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key           Pointer to the wrapped user key
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecdsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Wrapping key context pointer key is NULL \n
                                            - Provided init vector pointer is NULL \n
                                            - Provided key size pointer is NULL \n
                                            - Input wrapped user key pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_SW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                             const uint8_t *p_in_clear_private_key,
                                             const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                             const uint32_t *p_in_wrapped_user_key,
                                             hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob)
{

  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint16_t randoms[3] = {0U};
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  hal_ccb_ecdsa_sign_t signature = {NULL, NULL};
  hal_ccb_ecc_point_t public_key_out = {NULL, NULL};
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecdsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECDSA_POOL_BUFFER_SIZE(p_in_curve_param->ecdsa_pool_buffer.buff_size_byte,
                                                 p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_clear_private_key != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_clear_private_key == NULL) || (p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecdsa_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecdsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECDSA_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_out_wrapped_private_key_blob == NULL) || (p_out_wrapped_private_key_blob->p_iv == NULL)
      || (p_out_wrapped_private_key_blob->p_tag == NULL) || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                  p_in_curve_param->prime_order_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecdsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  signature.p_r_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  signature.p_s_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECDSA_SignBlobCreation(p_ccb_instance, p_in_curve_param, &sw_ctx, p_in_clear_private_key, p_tmp_iv,
                                 p_tmp_tag, p_tmp_wrapped_key, random32,
                                 (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key, &signature,
                                 CCB_ECDSA_SIGN_CPU_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  public_key_out.p_point_x = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  public_key_out.p_point_y = &p_base_pool_buff[offset_pool_buff];

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &sw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                                 random32, &public_key_out) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* PKA ECDSA valid R & S signature */
  if (CCB_PKA_ECDSASetConfigVerifSignature(PKA, p_in_curve_param, &public_key_out,
                                           (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                           &signature) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Check random number */
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECDSA Wrapping an RNG generated Key with a wrapped symmetric software key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_wrapping_key_context          Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key           Pointer to the wrapped user key
  * @param  p_out_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Wrapping key context pointer key is NULL \n
                                            - Provided init vector pointer is NULL \n
                                            - Provided key size pointer is NULL \n
                                            - Input wrapped user key pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_SW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                     const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                     const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                                     const uint32_t *p_in_wrapped_user_key,
                                                     hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob)
{

  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint16_t randoms[3] = {0U};
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  hal_ccb_ecdsa_sign_t signature = {NULL, NULL};
  hal_ccb_ecc_point_t public_key_out = {NULL, NULL};
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecdsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECDSA_POOL_BUFFER_SIZE(p_in_curve_param->ecdsa_pool_buffer.buff_size_byte,
                                                 p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecdsa_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecdsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECDSA_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_out_wrapped_private_key_blob == NULL) || (p_out_wrapped_private_key_blob->p_iv == NULL)
      || (p_out_wrapped_private_key_blob->p_tag == NULL) || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                  p_in_curve_param->prime_order_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecdsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  signature.p_r_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  signature.p_s_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECDSA_SignBlobCreation(p_ccb_instance, p_in_curve_param, &sw_ctx, NULL, p_tmp_iv, p_tmp_tag,
                                 p_tmp_wrapped_key, random32,
                                 (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key, &signature,
                                 CCB_ECDSA_SIGN_RNG_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  public_key_out.p_point_x = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  public_key_out.p_point_y = &p_base_pool_buff[offset_pool_buff];

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &sw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                                 random32, &public_key_out) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* PKA ECDSA valid R & S signature */
  if (CCB_PKA_ECDSASetConfigVerifSignature(PKA, p_in_curve_param, &public_key_out,
                                           (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                           &signature) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Check random number */
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECDSA Signature with a wrapped symmetric software key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_wrapping_key_context         Pointer to @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key          Pointer to the wrapped user key
  * @param  p_in_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @param  p_in_hash                      Pointer to the hash message
  * @param  hash_size                      Specify the size of the hash message
  * @param  p_out_signature                pointer to a @ref hal_ccb_ecdsa_sign_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter modulus pointer is NULL \n
                                           - Curve parameter prime_order pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Wrapping key context pointer key is NULL \n
                                           - Provided init vector pointer is NULL \n
                                           - Provided key size pointer is NULL \n
                                           - Input wrapped user key pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input hash pointer is NULL \n
                                           - Hash size pointer is zero \n
                                           - Output signature pointer is NULL \n
                                           - Output signature r_sign pointer is NULL \n
                                           - Output signature s_sign pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_SW_Sign(hal_ccb_handle_t *hccb, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                   const uint32_t *p_in_wrapped_user_key,
                                   hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob, const uint8_t *p_in_hash,
                                   uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_in_hash != NULL);
  ASSERT_DBG_PARAM(hash_size != 0U);
  ASSERT_DBG_PARAM(p_out_signature != NULL);
  ASSERT_DBG_PARAM(p_out_signature->p_r_sign != NULL);
  ASSERT_DBG_PARAM(p_out_signature->p_s_sign != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_private_key_blob == NULL)
      || (p_in_wrapped_private_key_blob->p_iv == NULL) || (p_in_wrapped_private_key_blob->p_tag == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL) || (p_in_hash == NULL) || (hash_size == 0U)
      || (p_out_signature == NULL) || (p_out_signature->p_r_sign == NULL) || (p_out_signature->p_s_sign == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_ECDSA_Sign(p_ccb_instance, p_in_curve_param, &sw_ctx, p_in_wrapped_private_key_blob, p_in_hash, hash_size,
                     p_out_signature) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECDSA Compute Public Key with a wrapped symmetric software key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_wrapping_key_context         Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key          Pointer to the wrapped user key
  * @param  p_in_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @param  p_out_public_key               Pointer to a @ref hal_ccb_ecc_point_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter modulus pointer is NULL \n
                                           - Curve parameter prime_order pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Wrapping key context pointer key is NULL \n
                                           - Provided init vector pointer is NULL \n
                                           - Provided key size pointer is NULL \n
                                           - Input wrapped user key pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input wrapped private key blob wrapped key pointer is NULL \n
                                           - Input hash pointer is NULL \n
                                           - Output public key point_x pointer is NULL \n
                                           - Output public key point_y pointer is NULL \n
                                           - Output public key pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_SW_ComputePublicKey(hal_ccb_handle_t *hccb,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                               const uint32_t *p_in_wrapped_user_key,
                                               hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob,
                                               hal_ccb_ecc_point_t *p_out_public_key)

{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_out_public_key != NULL);
  ASSERT_DBG_PARAM(p_out_public_key->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_out_public_key->p_point_y != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_in_wrapped_private_key_blob == NULL) || (p_in_wrapped_private_key_blob->p_iv == NULL)
      || (p_in_wrapped_private_key_blob->p_tag == NULL) || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL)
      || (p_out_public_key == NULL) || (p_out_public_key->p_point_x == NULL) || (p_out_public_key->p_point_y == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &sw_ctx, p_in_wrapped_private_key_blob->p_iv,
                                 p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_key,
                                 0U, p_out_public_key) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECC Wrapping private Key with a wrapped symmetric software key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  p_in_clear_private_key          Pointer to the clear private key
  * @param  p_wrapping_key_context          Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key           Pointer to the wrapped user key
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Input clear private key pointer is NULL \n
                                            - Output wrapped private key blob \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_SW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                           const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                           const uint8_t *p_in_clear_private_key,
                                           const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                           const uint32_t *p_in_wrapped_user_key,
                                           hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob)
{

  uint32_t *p_scalar_mul_y = NULL;
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint32_t modulus_words_count = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecc_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_curve_param->ecc_pool_buffer.buff_size_byte,
                                               p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_clear_private_key != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecc_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecc_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECC_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_in_clear_private_key == NULL) || (p_wrapping_key_context == NULL)
      || (p_wrapping_key_context->p_init_vect == NULL) || (p_wrapping_key_context->key_size == 0U)
      || (p_in_wrapped_user_key == NULL) || (p_out_wrapped_private_key_blob == NULL)
      || (p_out_wrapped_private_key_blob->p_iv == NULL) || (p_out_wrapped_private_key_blob->p_tag == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecc_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_scalar_mul_y = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += ((p_in_curve_param->modulus_size_byte + 3U) >> 2U) * CCB_BLOCK_SIZE_WORD;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECC_ScalarMulBlobCreation(p_ccb_instance, p_in_curve_param, &sw_ctx, p_in_clear_private_key, p_tmp_iv,
                                    p_tmp_tag, p_tmp_wrapped_key, random32,
                                    p_out_wrapped_private_key_blob->p_wrapped_key, p_scalar_mul_y,
                                    CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &sw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                               random32, NULL, NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_curve_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECC Wrapping an RNG generated Key with a wrapped symmetric software key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  p_wrapping_key_context          Pointer a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key           Pointer to the wrapped user key
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Wrapping key context pointer key is NULL \n
                                            - Provided init vector pointer is NULL \n
                                            - Provided key size pointer is NULL \n
                                            - Input wrapped user key pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped key pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_SW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                   const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                   const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                                   const uint32_t *p_in_wrapped_user_key,
                                                   hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_scalar_mul_y = NULL;
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint32_t modulus_words_count = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecc_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_curve_param->ecc_pool_buffer.buff_size_byte,
                                               p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecc_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecc_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECC_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_out_wrapped_private_key_blob == NULL) || (p_out_wrapped_private_key_blob->p_iv == NULL)
      || (p_out_wrapped_private_key_blob->p_tag == NULL) || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecc_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_scalar_mul_y = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += ((p_in_curve_param->modulus_size_byte + 3U) >> 2U) * CCB_BLOCK_SIZE_WORD;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECC_ScalarMulBlobCreation(p_ccb_instance, p_in_curve_param, &sw_ctx, NULL, p_tmp_iv, p_tmp_tag,
                                    p_tmp_wrapped_key, random32,
                                    p_out_wrapped_private_key_blob->p_wrapped_key, p_scalar_mul_y,
                                    CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &sw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                               random32, NULL, NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_curve_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECC Compute Scalar Multiplication with a wrapped symmetric software key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  p_wrapping_key_context         Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key          Pointer to the wrapped user key
  * @param  p_in_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @param  p_in_point                     Pointer to @ref hal_ccb_ecc_point_t structure
  * @param  p_out_point                    Pointer to @ref hal_ccb_ecc_point_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter modulus pointer is NULL \n
                                           - Curve parameter prime_order pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Wrapping key context pointer key is NULL \n
                                           - Provided init vector pointer is NULL \n
                                           - Provided key size pointer is NULL \n
                                           - Input wrapped user key pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Output wrapped private key blob IV pointer is NULL \n
                                           - Output wrapped private key blob tag pointer is NULL \n
                                           - Output wrapped private key blob wrapped key pointer is NULL \n
                                           - Input point pointer is NULL \n
                                           - Input point x pointer is NULL \n
                                           - Input point y pointer is NULL \n
                                           - Output point pointer is NULL \n
                                           - Input point x pointer is NULL \n
                                           - Input point y pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_SW_ComputeScalarMul(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                             const uint32_t *p_in_wrapped_user_key,
                                             hal_ccb_ecc_mul_key_blob_t *p_in_wrapped_private_key_blob,
                                             hal_ccb_ecc_point_t *p_in_point, hal_ccb_ecc_point_t *p_out_point)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_in_point != NULL);
  ASSERT_DBG_PARAM(p_in_point->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_point->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_out_point != NULL);
  ASSERT_DBG_PARAM(p_out_point->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_out_point->p_point_y != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_in_wrapped_private_key_blob == NULL) || (p_in_wrapped_private_key_blob->p_iv == NULL)
      || (p_in_wrapped_private_key_blob->p_tag == NULL) || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL)
      || (p_in_point == NULL) || (p_in_point->p_point_x == NULL) || (p_in_point->p_point_y == NULL)
      || (p_out_point == NULL) || (p_out_point->p_point_x == NULL) || (p_out_point->p_point_y == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &sw_ctx, p_in_wrapped_private_key_blob->p_iv,
                               p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_key,
                               0U, p_in_point, p_out_point) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: RSA Wrap the private Key with a wrapped symmetric software key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_param                      Pointer to @ref hal_ccb_rsa_param_t structure
  * @param  p_in_rsa_clear_private_key      Pointer to @ref hal_ccb_rsa_clear_key_t structure
  * @param  p_wrapping_key_context          Pointer to to @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key           Pointer to the wrapped user key
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_rsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input parameters pointer is NULL \n
                                            - Input RSA clear private key pointer key is NULL \n
                                            - Input parameter exp_size_byte is zero \n
                                            - Input parameter modulus_size_byte is zero \n
                                            - Input parameter modulus pointer is NULL \n
                                            - Input RSA clear private key exp pointer is NULL \n
                                            - Input RSA clear private key phi pointer is NULL \n
                                            - Wrapping key context pointer key is NULL \n
                                            - Provided init vector pointer is NULL \n
                                            - Provided key size pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped phi pointer is NULL \n
                                            - Output wrapped private key blob wrapped exp pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_RSA_SW_WrapPrivateKey(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                           const hal_ccb_rsa_clear_key_t *p_in_rsa_clear_private_key,
                                           const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                           const uint32_t *p_in_wrapped_user_key,
                                           hal_ccb_rsa_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_exp = NULL;
  uint32_t *p_tmp_wrapped_phi = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t modulus_words_count = 0U;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_param != NULL);
  ASSERT_DBG_PARAM(p_in_param->exp_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->p_mod != NULL);
  ASSERT_DBG_PARAM(p_in_param->rsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_param->rsa_pool_buffer.buff_size_byte,
                                               p_in_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key != NULL);
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key->p_exp != NULL);
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key->p_phi != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_phi != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_exp != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_param == NULL) || (p_in_param->exp_size_byte == 0U)
      || (p_in_param->modulus_size_byte == 0U) || (p_in_param->p_mod == NULL)
      || (p_in_param->rsa_pool_buffer.p_buff == NULL)
      || ((p_in_param->rsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_RSA_CALC_BUFFER_SIZE(p_in_param->modulus_size_byte)))
      || (p_in_rsa_clear_private_key == NULL) || (p_in_rsa_clear_private_key->p_exp == NULL)
      || (p_in_rsa_clear_private_key->p_phi == NULL) || (p_wrapping_key_context == NULL)
      || (p_wrapping_key_context->p_init_vect == NULL) || (p_wrapping_key_context->key_size == 0U)
      || (p_in_wrapped_user_key == NULL) || (p_out_wrapped_private_key_blob == NULL)
      || (p_out_wrapped_private_key_blob->p_iv == NULL) || (p_out_wrapped_private_key_blob->p_tag == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_phi == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_exp == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp,
                                  p_in_param->modulus_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_param->rsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_exp = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  p_tmp_wrapped_phi = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_RSA_ExpBlobCreation(p_ccb_instance, p_in_param, &sw_ctx, p_in_rsa_clear_private_key, p_tmp_iv, p_tmp_tag,
                              p_tmp_wrapped_exp, p_tmp_wrapped_phi, random32,
                              (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp,
                              p_out_wrapped_private_key_blob->p_wrapped_phi) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_RSA_ComputeModularExp(p_ccb_instance, p_in_param, &sw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_exp,
                                p_tmp_wrapped_phi, random32,
                                (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp, NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_exp[count] = (p_tmp_wrapped_exp[count] ^ random32);
    p_out_wrapped_private_key_blob->p_wrapped_phi[count] = (p_tmp_wrapped_phi[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: RSA Compute Modular exponentiation with a wrapped symmetric software key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_param                     Pointer to a @ref hal_ccb_rsa_param_t structure
  * @param  p_wrapping_key_context         Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_in_wrapped_user_key          Pointer to the wrapped user key
  * @param  p_in_wrapped_private_key_blob  Pointer to @ref hal_ccb_rsa_key_blob_t structure
  * @param  p_in_operand                   Pointer to the operand
  * @param  p_out_modular_exp              Pointer to the output operation
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input parameters pointer is NULL \n
                                           - Input parameter exp_size_byte is zero \n
                                           - Input parameter modulus_size_byte is zero \n
                                           - Input parameter modulus pointer is NULL \n
                                           - Wrapping key context pointer key is NULL \n
                                           - Provided init vector pointer is NULL \n
                                           - Provided key size pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input wrapped private key blob wrapped exp pointer is NULL \n
                                           - Input wrapped private key blob wrapped phi pointer is NULL \n
                                           - Output operand pointer is NULL \n
                                           - Output modular exp pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_RSA_SW_ComputeModularExp(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                              const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                              const uint32_t *p_in_wrapped_user_key,
                                              hal_ccb_rsa_key_blob_t *p_in_wrapped_private_key_blob,
                                              const uint8_t *p_in_operand, uint8_t *p_out_modular_exp)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_sw_unwrap_key_context_t sw_ctx = {CCB_KEY_TYPE_SOFTWARE, p_wrapping_key_context, p_in_wrapped_user_key};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_param != NULL);
  ASSERT_DBG_PARAM(p_in_param->exp_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->p_mod != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->p_init_vect != NULL);
  ASSERT_DBG_PARAM(p_wrapping_key_context->key_size != 0U);
  ASSERT_DBG_PARAM(p_in_wrapped_user_key != NULL);
  ASSERT_DBG_PARAM(IS_CCB_AES_MODE(p_wrapping_key_context->aes_algorithm));
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_exp != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_phi != NULL);
  ASSERT_DBG_PARAM(p_in_operand != NULL);
  ASSERT_DBG_PARAM(p_out_modular_exp != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_param == NULL) || (p_in_param->exp_size_byte == 0U)
      || (p_in_param->modulus_size_byte == 0U) || (p_in_param->p_mod == NULL)
      || (p_wrapping_key_context == NULL) || (p_wrapping_key_context->p_init_vect == NULL)
      || (p_wrapping_key_context->key_size == 0U) || (p_in_wrapped_user_key == NULL)
      || (p_in_wrapped_private_key_blob == NULL) || (p_in_wrapped_private_key_blob->p_iv == NULL)
      || (p_in_wrapped_private_key_blob->p_tag == NULL) || (p_in_wrapped_private_key_blob->p_wrapped_exp == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_phi == NULL) || (p_in_operand == NULL)
      || (p_out_modular_exp == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RSA_ComputeModularExp(p_ccb_instance, p_in_param, &sw_ctx, p_in_wrapped_private_key_blob->p_iv,
                                p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_exp,
                                p_in_wrapped_private_key_blob->p_wrapped_phi, 0U, p_in_operand,
                                p_out_modular_exp) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup CCB_Exported_Functions_Group5
  * @{
This subsection provides a set of functions allowing to create and use a dedicated blob using a hardware key.

ECDSA operations:
- HAL_CCB_ECDSA_HW_WrapPrivateKey(): Allows ECDSA Blob creation by wrapping user private key
  with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_ECDSA_HW_GenerateWrapPrivateKey(): Allows ECDSA Blob creation by wrapping an RNG generated key
  with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_ECDSA_HW_Sign(): Allows ECDSA Blob usage for ECDSA signature with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_ECDSA_HW_ComputePublicKey(): Allows ECDSA Blob usage for public key computation
  with a hardware key (DHUK or DHUK XOR).

ECC operations:
- HAL_CCB_ECC_HW_WrapPrivateKey(): Allows ECC Blob creation by wrapping user private key
  with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_ECC_HW_GenerateWrapPrivateKey(): Allows ECC Blob creation by wrapping an RNG generated key
  with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_ECC_HW_ComputeScalarMul(): Allows ECC Blob usage for ECC compute scalar multiplication
  with a hardware key

RSA operations:
- HAL_CCB_RSA_HW_WrapPrivateKey(): Allows RSA Blob creation by wrapping user private key
  with a hardware key (DHUK or DHUK XOR).
- HAL_CCB_RSA_HW_ComputeModularExp(): Allows RSA Blob usage for RSA modular exponentiation
  with a hardware key (DHUK or DHUK XOR).
  */

/**
  * @brief  Blob Creation: ECDSA Wrapping the private Key with a hardware key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_in_clear_private_key          Pointer to the clear private key
  * @param  wrapping_hw_key_type            Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecdsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Input clear private key pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped key pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */

hal_status_t HAL_CCB_ECDSA_HW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                             const uint8_t *p_in_clear_private_key,
                                             hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                             hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint16_t randoms[3] = {0U};
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  hal_ccb_ecdsa_sign_t signature = {NULL, NULL};
  hal_ccb_ecc_point_t public_key_out = {NULL, NULL};
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecdsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECDSA_POOL_BUFFER_SIZE(p_in_curve_param->ecdsa_pool_buffer.buff_size_byte,
                                                 p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_clear_private_key != NULL);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecdsa_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecdsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECDSA_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_in_clear_private_key == NULL) || (p_out_wrapped_private_key_blob == NULL)
      || (p_out_wrapped_private_key_blob->p_iv == NULL) || (p_out_wrapped_private_key_blob->p_tag == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                  p_in_curve_param->prime_order_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecdsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  signature.p_r_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  signature.p_s_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECDSA_SignBlobCreation(p_ccb_instance, p_in_curve_param, &hw_ctx, p_in_clear_private_key, p_tmp_iv,
                                 p_tmp_tag, p_tmp_wrapped_key, random32,
                                 (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key, &signature,
                                 CCB_ECDSA_SIGN_CPU_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  public_key_out.p_point_x = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  public_key_out.p_point_y = &p_base_pool_buff[offset_pool_buff];

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &hw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                                 random32, &public_key_out) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* PKA ECDSA valid R & S signature */
  if (CCB_PKA_ECDSASetConfigVerifSignature(PKA, p_in_curve_param, &public_key_out,
                                           (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                           &signature) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Check random number */
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECDSA Wrapping an RNG generated Key with a hardware key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  wrapping_hw_key_type            Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_out_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Input clear private key pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped key pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_HW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                     const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                     hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                                     hal_ccb_ecdsa_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint16_t randoms[3] = {0U};
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  hal_ccb_ecdsa_sign_t signature = {NULL, NULL};
  hal_ccb_ecc_point_t public_key_out = {NULL, NULL};
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecdsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECDSA_POOL_BUFFER_SIZE(p_in_curve_param->ecdsa_pool_buffer.buff_size_byte,
                                                 p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecdsa_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecdsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECDSA_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_out_wrapped_private_key_blob == NULL)
      || (p_out_wrapped_private_key_blob->p_iv == NULL) || (p_out_wrapped_private_key_blob->p_tag == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                  p_in_curve_param->prime_order_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecdsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  signature.p_r_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  signature.p_s_sign = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECDSA_SignBlobCreation(p_ccb_instance, p_in_curve_param, &hw_ctx, NULL, p_tmp_iv, p_tmp_tag,
                                 p_tmp_wrapped_key, random32,
                                 (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key, &signature,
                                 CCB_ECDSA_SIGN_RNG_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  public_key_out.p_point_x = &p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += p_in_curve_param->modulus_size_byte;
  public_key_out.p_point_y = &p_base_pool_buff[offset_pool_buff];

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &hw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                                 random32, &public_key_out) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* PKA ECDSA valid R & S signature */
  if (CCB_PKA_ECDSASetConfigVerifSignature(PKA, p_in_curve_param, &public_key_out,
                                           (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_key,
                                           &signature) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Check random number */
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check if it is valid signature and improve robustness against intrusion (intentional) */
  if ((pka_ram_u32[PKA_ECDSA_VERIF_OUT_RESULT] != CCB_PKA_RESULT_OK) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecdsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECDSA Signature with a hardware key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  wrapping_hw_key_type           Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_in_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @param  p_in_hash                      Pointer to the hash message
  * @param  hash_size                      Specify the size of the hash message
  * @param  p_out_signature                Pointer to a @ref hal_ccb_ecdsa_sign_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter modulus pointer is NULL \n
                                           - Curve parameter prime_order pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input hash pointer is NULL \n
                                           - Hash size is zero \n
                                           - Output signature pointer is NULL \n
                                           - Output signature r_sign pointer is NULL \n
                                           - Output signature s_sign pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_HW_Sign(hal_ccb_handle_t *hccb, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                   hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob, const uint8_t *p_in_hash,
                                   uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_in_hash != NULL);
  ASSERT_DBG_PARAM(hash_size != 0U);
  ASSERT_DBG_PARAM(p_out_signature != NULL);
  ASSERT_DBG_PARAM(p_out_signature->p_r_sign != NULL);
  ASSERT_DBG_PARAM(p_out_signature->p_s_sign != NULL);


  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_wrapped_private_key_blob == NULL)
      || (p_in_wrapped_private_key_blob->p_iv == NULL) || (p_in_wrapped_private_key_blob->p_tag == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL) || (p_in_hash == NULL)
      || (hash_size == 0U) || (p_out_signature == NULL)
      || (p_out_signature->p_r_sign == NULL) || (p_out_signature->p_s_sign == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_ECDSA_Sign(p_ccb_instance, p_in_curve_param, &hw_ctx, p_in_wrapped_private_key_blob, p_in_hash, hash_size,
                     p_out_signature) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECDSA Compute Public Key with a hardware key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  wrapping_hw_key_type           Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_in_wrapped_private_key_blob  Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @param  p_out_public_key               Pointer to a @ref hal_ccb_ecc_point_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter modulus pointer is NULL \n
                                           - Curve parameter prime_order pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input wrapped private key blob wrapped key pointer is NULL \n
                                           - Output public key point_x pointer is NULL \n
                                           - Output public key point_y pointer is NULL \n
                                           - Output public key pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECDSA_HW_ComputePublicKey(hal_ccb_handle_t *hccb,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                               hal_ccb_ecdsa_key_blob_t *p_in_wrapped_private_key_blob,
                                               hal_ccb_ecc_point_t *p_out_public_key)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_out_public_key != NULL);
  ASSERT_DBG_PARAM(p_out_public_key->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_out_public_key->p_point_y != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_wrapped_private_key_blob == NULL)
      || (p_in_wrapped_private_key_blob->p_iv == NULL) || (p_in_wrapped_private_key_blob->p_tag == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL) || (p_out_public_key == NULL)
      || (p_out_public_key->p_point_x == NULL) || (p_out_public_key->p_point_y == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_ECDSA_ComputePublicKey(p_ccb_instance, p_in_curve_param, &hw_ctx, p_in_wrapped_private_key_blob->p_iv,
                                 p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_key,
                                 0U, p_out_public_key) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECC Wrapping private Key with a hardware key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  p_in_clear_private_key          Pointer to the clear private key
  * @param  wrapping_hw_key_type            Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Input clear private key blob pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped key pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_HW_WrapPrivateKey(hal_ccb_handle_t *hccb,
                                           const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                           const uint8_t *p_in_clear_private_key,
                                           hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                           hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_scalar_mul_y = NULL;
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint32_t modulus_words_count = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecc_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_curve_param->ecc_pool_buffer.buff_size_byte,
                                               p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_clear_private_key != NULL);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecc_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecc_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECC_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte))) || (p_in_clear_private_key == NULL)
      || (p_out_wrapped_private_key_blob == NULL) || (p_out_wrapped_private_key_blob->p_iv == NULL)
      || (p_out_wrapped_private_key_blob->p_tag == NULL) || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecc_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_scalar_mul_y = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += ((p_in_curve_param->modulus_size_byte + 3U) >> 2U) * CCB_BLOCK_SIZE_WORD;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECC_ScalarMulBlobCreation(p_ccb_instance, p_in_curve_param, &hw_ctx, p_in_clear_private_key, p_tmp_iv,
                                    p_tmp_tag, p_tmp_wrapped_key, random32,
                                    p_out_wrapped_private_key_blob->p_wrapped_key, p_scalar_mul_y,
                                    CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &hw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key,
                               random32, NULL, NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_curve_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: ECC Wrapping an RNG generated Key with a hardware key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param                Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  wrapping_hw_key_type            Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input curve parameters pointer is NULL \n
                                            - Curve parameter abs_coef_a pointer is NULL \n
                                            - Curve parameter coef_b pointer is NULL \n
                                            - Curve parameter modulus pointer is NULL \n
                                            - Curve parameter prime_order pointer is NULL \n
                                            - Curve parameter point_x pointer is NULL \n
                                            - Curve parameter point_y pointer is NULL \n
                                            - Curve parameter prime_order_size_byte is zero \n
                                            - Curve parameter modulus_size_byte is zero \n
                                            - Curve parameter coef_sign_a is zero \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped key pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_HW_GenerateWrapPrivateKey(hal_ccb_handle_t *hccb,
                                                   const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                   hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                                   hal_ccb_ecc_mul_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_scalar_mul_y = NULL;
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_key = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint32_t modulus_words_count = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->ecc_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_curve_param->ecc_pool_buffer.buff_size_byte,
                                               p_in_curve_param->modulus_size_byte));
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_key != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_curve_param->ecc_pool_buffer.p_buff == NULL)
      || ((p_in_curve_param->ecc_pool_buffer.buff_size_byte)
          < (HAL_CCB_ECC_CALC_BUFFER_SIZE(p_in_curve_param->modulus_size_byte)))
      || (p_out_wrapped_private_key_blob == NULL)
      || (p_out_wrapped_private_key_blob->p_iv == NULL) || (p_out_wrapped_private_key_blob->p_tag == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_key == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_curve_param->ecc_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_scalar_mul_y = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += ((p_in_curve_param->modulus_size_byte + 3U) >> 2U) * CCB_BLOCK_SIZE_WORD;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_key = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_ECC_ScalarMulBlobCreation(p_ccb_instance, p_in_curve_param, &hw_ctx,
                                    NULL, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key, random32,
                                    p_out_wrapped_private_key_blob->p_wrapped_key, p_scalar_mul_y,
                                    CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &hw_ctx,
                               p_tmp_iv, p_tmp_tag, p_tmp_wrapped_key, random32, NULL,
                               NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_curve_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Scalar Multiplication and improve robustness against intrusion (intentional) */
  /* P coordinate x */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X + word_offset]
            ^ p_out_wrapped_private_key_blob->p_wrapped_key[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* P coordinate y */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y + word_offset] ^ p_scalar_mul_y[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_key[count] = (p_tmp_wrapped_key[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_curve_param->ecc_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECC Compute Scalar Multiplication with a hardware key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_curve_param               Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  wrapping_hw_key_type           Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_in_wrapped_private_key_blob  Pointer to @ref hal_ccb_ecc_mul_key_blob_t structure
  * @param  p_in_point                     Pointer to @ref hal_ccb_ecc_point_t structure
  * @param  p_out_point                    Pointer to @ref hal_ccb_ecc_point_t structure
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input curve parameters pointer is NULL \n
                                           - Curve parameter abs_coef_a pointer is NULL \n
                                           - Curve parameter coef_b pointer is NULL \n
                                           - Curve parameter point_x pointer is NULL \n
                                           - Curve parameter point_y pointer is NULL \n
                                           - Curve parameter prime_order_size_byte is zero \n
                                           - Curve parameter modulus_size_byte is zero \n
                                           - Curve parameter coef_sign_a is zero \n
                                           - Input clear private key blob pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input wrapped private key blob wrapped key pointer is NULL \n
                                           - Input point pointer is NULL \n
                                           - Input point x pointer is NULL \n
                                           - Input point y pointer is NULL \n
                                           - Output point pointer is NULL \n
                                           - Output point x pointer is NULL \n
                                           - Output point y pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_ECC_HW_ComputeScalarMul(hal_ccb_handle_t *hccb,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                             hal_ccb_ecc_mul_key_blob_t *p_in_wrapped_private_key_blob,
                                             hal_ccb_ecc_point_t *p_in_point, hal_ccb_ecc_point_t *p_out_point)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_abs_coef_a != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_coef_b != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_modulus != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_prime_order != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_in_curve_param->prime_order_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_curve_param->coef_sign_a != 0U);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_key != NULL);
  ASSERT_DBG_PARAM(p_in_point != NULL);
  ASSERT_DBG_PARAM(p_in_point->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_in_point->p_point_y != NULL);
  ASSERT_DBG_PARAM(p_out_point != NULL);
  ASSERT_DBG_PARAM(p_out_point->p_point_x != NULL);
  ASSERT_DBG_PARAM(p_out_point->p_point_y != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_curve_param == NULL) || (p_in_curve_param->p_abs_coef_a == NULL)
      || (p_in_curve_param->p_coef_b == NULL) || (p_in_curve_param->p_modulus == NULL)
      || (p_in_curve_param->p_prime_order == NULL) || (p_in_curve_param->p_point_x == NULL)
      || (p_in_curve_param->p_point_y == NULL) || (p_in_curve_param->prime_order_size_byte == 0U)
      || (p_in_curve_param->modulus_size_byte == 0U) || (p_in_curve_param->coef_sign_a == 0U)
      || (p_in_wrapped_private_key_blob == NULL)
      || (p_in_wrapped_private_key_blob->p_iv == NULL) || (p_in_wrapped_private_key_blob->p_tag == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_key == NULL) || (p_in_point == NULL)
      || (p_in_point->p_point_x == NULL) || (p_in_point->p_point_y == NULL) || (p_out_point == NULL)
      || (p_out_point->p_point_x == NULL) || (p_out_point->p_point_y == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_ECC_ComputeScalarMul(p_ccb_instance, p_in_curve_param, &hw_ctx, p_in_wrapped_private_key_blob->p_iv,
                               p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_key, 0U,
                               p_in_point, p_out_point) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Creation: RSA Wrapping private Key with a hardware key.
  * @param  hccb                            Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_param                      Pointer to @ref hal_ccb_rsa_param_t structure
  * @param  p_in_rsa_clear_private_key      Pointer to @ref hal_ccb_rsa_clear_key_t structure
  * @param  wrapping_hw_key_type            Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_out_wrapped_private_key_blob  Pointer to @ref hal_ccb_rsa_key_blob_t structure
  * @retval HAL_INVALID_PARAM               The function return an Invalid param in the following cases: \n
                                            - The CCB handle is NULL \n
                                            - Input parameters pointer is NULL \n
                                            - Input parameter exp_size_byte is zero \n
                                            - Input parameter modulus_size_byte is zero \n
                                            - Input parameter modulus pointer is NULL \n
                                            - Input RSA clear private key exp pointer is NULL \n
                                            - Input RSA clear private key phi pointer is NULL \n
                                            - Output wrapped private key blob pointer is NULL \n
                                            - Output wrapped private key blob IV pointer is NULL \n
                                            - Output wrapped private key blob tag pointer is NULL \n
                                            - Output wrapped private key blob wrapped exp pointer is NULL \n
                                            - Output wrapped private key blob wrapped phi pointer is NULL \n
  * @retval HAL_ERROR                       Error detected
  * @retval HAL_OK                          Operation completed successfully
  */
hal_status_t HAL_CCB_RSA_HW_WrapPrivateKey(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                           const hal_ccb_rsa_clear_key_t *p_in_rsa_clear_private_key,
                                           hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                           hal_ccb_rsa_key_blob_t *p_out_wrapped_private_key_blob)
{
  uint32_t *p_tmp_iv = NULL;
  uint32_t *p_tmp_tag = NULL;
  uint32_t *p_tmp_wrapped_exp = NULL;
  uint32_t *p_tmp_wrapped_phi = NULL;
  __IOM const uint32_t *pka_ram_u32 = NULL;
  uint32_t modulus_words_count = 0U;
  uint32_t operand_size = 0U;
  uint32_t cipherkey_size = 0U;
  uint32_t offset_pool_buff = 0U;
  uint32_t random32 = 0U;
  uint16_t randoms[3] = {0U};
  uint32_t diff = 0U;
  volatile uint16_t random_count = 0U;
  uint16_t tmp_random_count = 0U;
  uint8_t *p_base_pool_buff = NULL;
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_param != NULL);
  ASSERT_DBG_PARAM(p_in_param->exp_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->p_mod != NULL);
  ASSERT_DBG_PARAM(p_in_param->rsa_pool_buffer.p_buff != NULL);
  ASSERT_DBG_PARAM(IS_CCB_ECC_POOL_BUFFER_SIZE(p_in_param->rsa_pool_buffer.buff_size_byte,
                                               p_in_param->modulus_size_byte));
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key != NULL);
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key->p_exp != NULL);
  ASSERT_DBG_PARAM(p_in_rsa_clear_private_key->p_phi != NULL);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_exp != NULL);
  ASSERT_DBG_PARAM(p_out_wrapped_private_key_blob->p_wrapped_phi != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_param == NULL) || (p_in_param->exp_size_byte == 0U)
      || (p_in_param->modulus_size_byte == 0U) || (p_in_param->p_mod == NULL)
      || ((p_in_param->rsa_pool_buffer.buff_size_byte)
          < (HAL_CCB_RSA_CALC_BUFFER_SIZE(p_in_param->modulus_size_byte)))
      || (p_in_rsa_clear_private_key == NULL) || (p_in_rsa_clear_private_key->p_exp == NULL)
      || (p_in_rsa_clear_private_key->p_phi == NULL)
      || (p_out_wrapped_private_key_blob == NULL) || (p_out_wrapped_private_key_blob->p_iv == NULL)
      || (p_out_wrapped_private_key_blob->p_tag == NULL) || (p_out_wrapped_private_key_blob->p_wrapped_exp == NULL)
      || (p_out_wrapped_private_key_blob->p_wrapped_phi == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RNG_Init(RNG) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateRandomNumbers(RNG, randoms) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  if (CCB_RNG_GenerateHashMessage(RNG, (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp,
                                  p_in_param->modulus_size_byte) != HAL_OK)
  {
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  p_base_pool_buff = (uint8_t *)p_in_param->rsa_pool_buffer.p_buff;

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);

  operand_size = 2UL * (((p_in_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  p_tmp_iv = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_tag = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += 4U * sizeof(uint32_t);
  p_tmp_wrapped_exp = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  offset_pool_buff += cipherkey_size * CCB_BLOCK_SIZE_WORD;
  p_tmp_wrapped_phi = (uint32_t *)(void *)&p_base_pool_buff[offset_pool_buff];
  random32 = (((uint32_t)randoms[1] << 16U) | (uint32_t)randoms[0]);

  if (CCB_RSA_ExpBlobCreation(p_ccb_instance, p_in_param, &hw_ctx, p_in_rsa_clear_private_key, p_tmp_iv, p_tmp_tag,
                              p_tmp_wrapped_exp, p_tmp_wrapped_phi, random32,
                              (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp,
                              p_out_wrapped_private_key_blob->p_wrapped_phi) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  if (CCB_RSA_ComputeModularExp(p_ccb_instance, p_in_param, &hw_ctx, p_tmp_iv, p_tmp_tag, p_tmp_wrapped_exp,
                                p_tmp_wrapped_phi, random32,
                                (uint8_t *)p_out_wrapped_private_key_blob->p_wrapped_exp, NULL) != HAL_OK)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  modulus_words_count = (p_in_param->modulus_size_byte + 3UL) >> 2UL;
  if (randoms[0] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[0]; ++j)
  {
    random_count++;
  }

  pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;
  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[0]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[1] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[1]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[1]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Initialize random_count and Check random number */
  random_count = 0U;
  if (randoms[2] == 0U)
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Random wait: introduce a random-duration loop to complicate timing/fault attacks.
     The loop count is derived from TRNG and then checked to detect potential fault injection. */
  for (uint16_t j = 0U; j < randoms[2]; ++j)
  {
    random_count++;
  }

  tmp_random_count = random_count;

  /* Check Modular Exponentiation and improve robustness against intrusion (intentional) */
  for (uint32_t word_offset = 0UL; word_offset < modulus_words_count; word_offset++)
  {
    diff |= pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset] ^
            p_out_wrapped_private_key_blob->p_wrapped_phi[word_offset];
  }

  if ((diff != 0U) || (tmp_random_count != randoms[2]))
  {
    CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);
    hccb->global_state = HAL_CCB_STATE_IDLE;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  /* Export created Blob */
  for (uint32_t count = 0U; count < cipherkey_size; count++)
  {
    if (count < CCB_BLOCK_SIZE_WORD)
    {
      p_out_wrapped_private_key_blob->p_iv[count] = (p_tmp_iv[count] ^ random32);
      p_out_wrapped_private_key_blob->p_tag[count] = (p_tmp_tag[count] ^ random32);
    }
    p_out_wrapped_private_key_blob->p_wrapped_exp[count] = (p_tmp_wrapped_exp[count] ^ random32);
    p_out_wrapped_private_key_blob->p_wrapped_phi[count] = (p_tmp_wrapped_phi[count] ^ random32);
  }

  CCB_ErasePoolBuffer(p_base_pool_buff, p_in_param->rsa_pool_buffer.buff_size_byte);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}

/**
  * @brief  Blob Usage: RSA Compute Modular exponentiation with a hardware key.
  * @param  hccb                           Pointer to a @ref hal_ccb_handle_t structure
  * @param  p_in_param                     Pointer to a @ref hal_ccb_rsa_param_t structure
  * @param  wrapping_hw_key_type           Wrapping key with a **hal_ccb_wrapping_hw_key_type_t** type
  * @param  p_in_wrapped_private_key_blob  Pointer to @ref hal_ccb_rsa_key_blob_t structure
  * @param  p_in_operand                   Pointer to the operand
  * @param  p_out_modular_exp              Pointer to the output operation
  * @retval HAL_INVALID_PARAM              The function return an Invalid param in the following cases: \n
                                           - The CCB handle is NULL \n
                                           - Input parameters pointer is NULL \n
                                           - Input parameter exp_size_byte is zero \n
                                           - Input parameter modulus_size_byte is zero \n
                                           - Input parameter modulus pointer is NULL \n
                                           - Input wrapped private key blob pointer is NULL \n
                                           - Input wrapped private key blob IV pointer is NULL \n
                                           - Input wrapped private key blob tag pointer is NULL \n
                                           - Input wrapped private key blob wrapped exp pointer is NULL \n
                                           - Input wrapped private key blob wrapped phi pointer is NULL \n
                                           - Output operand pointer is NULL \n
                                           - Output modular exp pointer is NULL \n
  * @retval HAL_ERROR                      Error detected
  * @retval HAL_OK                         Operation completed successfully
  */
hal_status_t HAL_CCB_RSA_HW_ComputeModularExp(hal_ccb_handle_t *hccb, const hal_ccb_rsa_param_t *p_in_param,
                                              hal_ccb_wrapping_hw_key_type_t wrapping_hw_key_type,
                                              hal_ccb_rsa_key_blob_t *p_in_wrapped_private_key_blob,
                                              const uint8_t *p_in_operand, uint8_t *p_out_modular_exp)
{
  CCB_TypeDef *p_ccb_instance = NULL;
  ccb_hw_unwrap_key_context_t hw_ctx = {CCB_KEY_TYPE_HARDWARE, wrapping_hw_key_type};

  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_in_param != NULL);
  ASSERT_DBG_PARAM(p_in_param->exp_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->modulus_size_byte != 0U);
  ASSERT_DBG_PARAM(p_in_param->p_mod != NULL);
  ASSERT_DBG_PARAM(IS_CCB_HW_KEY_TYPE(wrapping_hw_key_type));
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_iv != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_tag != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_exp != NULL);
  ASSERT_DBG_PARAM(p_in_wrapped_private_key_blob->p_wrapped_phi != NULL);
  ASSERT_DBG_PARAM(p_in_operand != NULL);
  ASSERT_DBG_PARAM(p_out_modular_exp != NULL);

  ASSERT_DBG_STATE(hccb->global_state, (uint32_t)HAL_CCB_STATE_IDLE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hccb == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)) \
   || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if ((p_in_param == NULL) || (p_in_param->exp_size_byte == 0U)
      || (p_in_param->modulus_size_byte == 0U) || (p_in_param->p_mod == NULL)
      || (p_in_wrapped_private_key_blob == NULL)
      || (p_in_wrapped_private_key_blob->p_iv == NULL) || (p_in_wrapped_private_key_blob->p_tag == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_exp == NULL)
      || (p_in_wrapped_private_key_blob->p_wrapped_phi == NULL)
      || (p_in_operand == NULL) || (p_out_modular_exp == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM or USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hccb, global_state, HAL_CCB_STATE_IDLE, HAL_CCB_STATE_ACTIVE);

  CCB_RESET(hccb);

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
  hccb->last_error_codes = HAL_CCB_ERROR_NONE;
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */

  p_ccb_instance = CCB_GET_INSTANCE(hccb);

  if (CCB_RSA_ComputeModularExp(p_ccb_instance, p_in_param, &hw_ctx, p_in_wrapped_private_key_blob->p_iv,
                                p_in_wrapped_private_key_blob->p_tag, p_in_wrapped_private_key_blob->p_wrapped_exp,
                                p_in_wrapped_private_key_blob->p_wrapped_phi, 0U, p_in_operand,
                                p_out_modular_exp) != HAL_OK)
  {
#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
    hccb->last_error_codes = STM32_READ_BIT(p_ccb_instance->SR, CCB_SR_OPERR);
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
    hccb->global_state = HAL_CCB_STATE_FAULT;
    return HAL_ERROR;
  }

  CCB_RESET(hccb);

  hccb->global_state = HAL_CCB_STATE_IDLE;
  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup CCB_Exported_Functions_Group6
  * @{
  */
/**
This subsection provides a set of functions to retrieve the state and the error codes
- HAL_CCB_GetState() retrieve the global CCB state.
- HAL_CCB_GetLastErrorCodes() retrieve the last CCB error code.
- HAL_CCB_SetUserData() store user data pointer into the handle.
- HAL_CCB_GetUserData() retrieve user data pointer from the handle.
  */

/**
  * @brief  Return the CCB state.
  * @param  hccb pointer to @ref hal_ccb_handle_t structure.
  * @retval hal_ccb_state_t CCB global state.
  */
hal_ccb_state_t HAL_CCB_GetState(const hal_ccb_handle_t *hccb)
{
  ASSERT_DBG_PARAM(hccb != NULL);

  return (hccb->global_state);
}

#if defined(USE_HAL_CCB_GET_LAST_ERRORS) && (USE_HAL_CCB_GET_LAST_ERRORS == 1)
/**
  * @brief  Return the CCB handle error code.
  * @param  hccb pointer to @ref hal_ccb_handle_t structure.
  * @retval uint32_t CCB last error codes.
  */
uint32_t HAL_CCB_GetLastErrorCodes(const hal_ccb_handle_t *hccb)
{
  ASSERT_DBG_PARAM(hccb != NULL);

  return (hccb->last_error_codes);
}
#endif /* USE_HAL_CCB_GET_LAST_ERRORS */
/**
  * @}
  */

#if defined (USE_HAL_CCB_USER_DATA) && (USE_HAL_CCB_USER_DATA == 1)
/**
  * @brief Store the user data into the CCB handle.
  * @param hccb        pointer to @ref hal_ccb_handle_t structure.
  * @param p_user_data pointer to the user data
  */
void HAL_CCB_SetUserData(hal_ccb_handle_t *hccb, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hccb != NULL);
  ASSERT_DBG_PARAM(p_user_data != NULL);

  hccb->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the user data from the CCB handle.
  * @param  hccb Pointer to CCB handle
  * @retval Pointer to the user data
  */
const void *HAL_CCB_GetUserData(const hal_ccb_handle_t *hccb)
{
  ASSERT_DBG_PARAM(hccb != NULL);

  return (hccb->p_user_data);
}
#endif /* USE_HAL_CCB_USER_DATA */
/**
  * @}
  */


/** @addtogroup CCB_Private_Functions
  * @{
  */
/**
  * @brief  Set PKA Operation Mode.
  * @param  p_pka_instance  PKA instance
  * @param  operation       PKA Operation
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Initialization completes successfully
  */
static hal_status_t CCB_PKA_SetOperation(PKA_TypeDef *p_pka_instance, uint32_t operation)
{
  uint32_t ccb_pka_mode;

  LL_PKA_Enable(p_pka_instance);

  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_INITOK) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Reset any pending flag */
  STM32_SET_BIT(p_pka_instance->CLRFR, PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC
                | PKA_CLRFR_ADDRERRFC | PKA_CLRFR_OPERRFC);

  switch (operation)
  {
    case CCB_ECDSA_SIGN_CPU_BLOB_CREATION:
    case CCB_ECDSA_SIGN_RNG_BLOB_CREATION:
    case CCB_ECDSA_SIGN_BLOB_USE:
    {
      ccb_pka_mode = CCB_PKA_ECDSA_SIGNATURE_PROTECT_MODE;
      break;
    }

    case CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION:
    case CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION:
    case CCB_ECC_SCALAR_MUL_BLOB_USE:
    {
      ccb_pka_mode = CCB_PKA_ECC_MUL_PROTECT_MODE;
      break;
    }

    default:
    {
      ccb_pka_mode = CCB_PKA_MODULAR_EXP_PROTECT_MODE;
      break;
    }
  }

  STM32_MODIFY_REG(p_pka_instance->CR, PKA_CR_MODE | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE
                   | PKA_CR_OPERRIE, ccb_pka_mode);
  return HAL_OK;
}

/**
  * @brief  Unprotected PKA Initialization.
  * @param  p_pka_instance  PKA instance
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Initialization completes successfully
  */
static hal_status_t CCB_PKA_Init(PKA_TypeDef *p_pka_instance)
{
  /* Reset the control register and enable the PKA (wait the end of PKA RAM erase) */
  LL_PKA_Enable(p_pka_instance);

  /* Wait the INITOK flag Setting */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_INITOK) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Reset any pending flag */
  STM32_SET_BIT(p_pka_instance->CLRFR, PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC
                | PKA_CLRFR_ADDRERRFC | PKA_CLRFR_OPERRFC);

  return HAL_OK;
}

/**
  * @brief  Initialize the RNG.
  * @param  p_rng_instance  RNG instance
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Initialization completes successfully
  */
static hal_status_t CCB_RNG_Init(RNG_TypeDef *p_rng_instance)
{
  uint32_t tickstart;

  /* Disable RNG */
  LL_RNG_Disable(p_rng_instance);
  /* Clock Error Detection Configuration when CONDRT bit is 1U to 1 */
  LL_RNG_EnableCondReset(p_rng_instance);
#if defined(RNG_CR_NIST_VALUE)
  /* Recommended value for NIST compliance, refer to application note AN4230 */
  STM32_WRITE_REG(p_rng_instance->CR, RNG_CR_NIST_VALUE | RNG_CR_CONDRST);
#endif /* defined(RNG_CR_NIST_VALUE) */
#if defined(RNG_HTCR_NIST_VALUE)
  /* Recommended value for NIST compliance, refer to application note AN4230 */
  LL_RNG_SetHealthConfig(p_rng_instance, RNG_HTCR_NIST_VALUE);
#endif /* defined(RNG_HTCR_NIST_VALUE) */
#if defined(RNG_NSCR_NIST_VALUE)
  STM32_WRITE_REG(p_rng_instance->NSCR, RNG_NSCR_NIST_VALUE);
#endif /* defined(RNG_NSCR_NIST_VALUE) */

  LL_RNG_DisableCondReset(p_rng_instance);

  tickstart = HAL_GetTick();

  while (LL_RNG_IsEnabledCondReset(p_rng_instance) == 1U)
  {
    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if (LL_RNG_IsEnabledCondReset(p_rng_instance) == 1U)
      {
        return HAL_ERROR;
      }
    }
  }

  LL_RNG_Enable(p_rng_instance);


  /* Check if data register contains valid random data */
  if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Wait CCB Operation step.
  * @param  p_ccb_instance  CCB instance
  * @param  step            Step to wait
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Operation is successfully accomplished
  */
static hal_status_t CCB_WaitOperStep(CCB_TypeDef const *p_ccb_instance, uint32_t step)
{
  uint32_t tickstart;

  tickstart = HAL_GetTick();

  while ((STM32_READ_REG(p_ccb_instance->SR) & step) != step)
  {
    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if ((STM32_READ_REG(p_ccb_instance->SR) & step) != step)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Wait CCB Flag.
  * @param  p_ccb_instance  CCB instance
  * @param  flag            Specifies the flag to check
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Operation is successfully accomplished
  */
static hal_status_t CCB_WaitFLAG(CCB_TypeDef const *p_ccb_instance, uint32_t flag)
{
  uint32_t tickstart = HAL_GetTick();

  while (((p_ccb_instance->SR) & flag) == flag)
  {
    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if (((p_ccb_instance->SR) & flag) == flag)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Wait PKA Flag.
  * @param  p_pka_instance  PKA instance
  * @param  flag            Specifies the flag to check
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Operation is successfully accomplished
  */
static hal_status_t CCB_PKA_WaitFlag(PKA_TypeDef *p_pka_instance, uint32_t flag)
{
  uint32_t tickstart = HAL_GetTick();

  while ((LL_PKA_IsActiveFlag(p_pka_instance, flag)) == 0U)
  {
#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
    if ((flag == PKA_SR_INITOK) && (LL_PKA_IsActiveFlag(p_pka_instance, PKA_SR_INITOK) == 0U))
    {
      /*Check if there is an RNG seed error */
      if (LL_RNG_IsActiveFlag_SECS(RNG) != 0U)
      {
        /* Attempt to recover from the seed error */
        if (CCB_RNG_ResilientRecoverSeedError() != HAL_OK)
        {
          return HAL_ERROR;
        }
      }
    }
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* USE_HAL_CCB_RNG_RECOVERY */

    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if ((LL_PKA_IsActiveFlag(p_pka_instance, flag)) == 0U)
      {
        LL_PKA_Disable(p_pka_instance);
        return HAL_ERROR;
      }
    }
  }

  LL_PKA_ClearFlag(p_pka_instance, flag);

  return HAL_OK;
}

/**
  * @brief  Wait RNG Flag.
  * @param  p_rng_instance  RNG instance
  * @param  flag            Specifies the flag to check
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Operation is successfully accomplished
  */
static hal_status_t CCB_RNG_WaitFlag(RNG_TypeDef *p_rng_instance, uint32_t flag)
{
  uint32_t tickstart = HAL_GetTick();
  while (((STM32_READ_REG(p_rng_instance->SR)) & flag) == 0U)
  {
#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
    /*Check if there is an RNG seed error */
    if ((LL_RNG_IsActiveFlag_SECS(RNG) != 0U) || (LL_RNG_IsActiveFlag_SEIS(RNG) != 0U))
    {
      /* Attempt to recover from the seed error */
      if (CCB_RNG_ResilientRecoverSeedError() != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* USE_HAL_CCB_RNG_RECOVERY */

    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if (((STM32_READ_REG(p_rng_instance->SR)) & flag) == 0U)
      {
        LL_RNG_Disable(p_rng_instance);
        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Wait SAES Flag.
  * @param  p_saes_instance  SAES instance
  * @param  flag             Specifies the flag to check
  * @param  status           Flag status
  * @retval HAL_ERROR        Error detected
  * @retval HAL_OK           Operation is successfully accomplished
  */
static hal_status_t CCB_SAES_WaitFlag(AES_TypeDef *p_saes_instance, uint32_t flag, uint32_t status)
{
  uint32_t tickstart = HAL_GetTick();

  while (CCB_SAES_GetFlag(p_saes_instance, flag) != status)
  {
#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
    if ((flag == AES_SR_BUSY) && (CCB_SAES_GetFlag(p_saes_instance, AES_SR_BUSY) != status))
    {
      /*Check if there is an RNG seed error */
      if (LL_RNG_IsActiveFlag_SECS(RNG) != 0U)
      {
        /* Attempt to recover from the seed error */
        if (CCB_RNG_ResilientRecoverSeedError() != HAL_OK)
        {
          return HAL_ERROR;
        }
      }
    }
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* USE_HAL_CCB_RNG_RECOVERY */

    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if (CCB_SAES_GetFlag(p_saes_instance, flag) != status)
      {
        STM32_CLEAR_BIT(p_saes_instance->CR, AES_CR_EN);
        return HAL_ERROR;
      }
    }
  }

  STM32_SET_BIT(p_saes_instance->ICR, flag);

  return HAL_OK;
}

/**
  * @brief  Set ECDSA signature parameters in related PKA RAM address.
  * @param  p_in_curve_param   Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @retval HAL_ERROR          Error detected
  * @retval HAL_OK             Operation is successfully accomplished
  */
static hal_status_t CCB_ECDSASign_SetParam(const hal_ccb_ecdsa_curve_param_t *p_in_curve_param)
{
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;

  /* Get the prime order n length */
  pka_ram_u32[PKA_ECDSA_SIGN_IN_ORDER_NB_BITS]
    = CCB_GetOptBitSize_u8(p_in_curve_param->prime_order_size_byte, *(p_in_curve_param->p_prime_order));
  pka_ram_u32[PKA_ECDSA_SIGN_IN_ORDER_NB_BITS + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the modulus p length */
  pka_ram_u32[PKA_ECDSA_SIGN_IN_MOD_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->modulus_size_byte,
                                                                    *(p_in_curve_param->p_modulus));
  pka_ram_u32[PKA_ECDSA_SIGN_IN_MOD_NB_BITS + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the coefficient a sign */
  pka_ram_u32[PKA_ECDSA_SIGN_IN_A_COEFF_SIGN] = p_in_curve_param->coef_sign_a;
  pka_ram_u32[PKA_ECDSA_SIGN_IN_A_COEFF_SIGN + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters coefficient |a| to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_A_COEFF,
                   p_in_curve_param->p_abs_coef_a) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters coefficient b to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_B_COEFF,
                   p_in_curve_param->p_coef_b) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters modulus value p to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_MOD_GF,
                   p_in_curve_param->p_modulus) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters prime order n to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_ORDER_N,
                   p_in_curve_param->p_prime_order) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters base point G coordinate x to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_INITIAL_POINT_X,
                   p_in_curve_param->p_point_x) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters base point G coordinate y to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECDSA_SIGN_IN_INITIAL_POINT_Y,
                   p_in_curve_param->p_point_y) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Set ECC scalar multiplication parameters in related PKA RAM address.
  * @param  p_in_curve_param  Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_ECCMul_SetParam(const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param)
{
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;

  /* Get the prime order n length */
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_EXP_NB_BITS]
    = CCB_GetOptBitSize_u8(p_in_curve_param->prime_order_size_byte, *(p_in_curve_param->p_prime_order));
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_EXP_NB_BITS + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the modulus p length */
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_OP_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->modulus_size_byte,
                                                                       *(p_in_curve_param->p_modulus));
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_OP_NB_BITS + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the coefficient a sign */
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_A_COEFF_SIGN] = p_in_curve_param->coef_sign_a;
  pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_A_COEFF_SIGN + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters coefficient |a| to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECC_SCALAR_MUL_IN_A_COEFF,
                   p_in_curve_param->p_abs_coef_a) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters coefficient b to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECC_SCALAR_MUL_IN_B_COEFF,
                   p_in_curve_param->p_coef_b) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters modulus value p to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECC_SCALAR_MUL_IN_MOD_GF,
                   p_in_curve_param->p_modulus) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters prime order n to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_ECC_SCALAR_MUL_IN_N_PRIME_ORDER,
                   p_in_curve_param->p_prime_order) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Set RSA Modular Exponentiation parameters in related PKA RAM address.
  * @param  p_in_curve_param  Pointer to a @ref hal_ccb_rsa_param_t structure
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_RSAModExp_SetParam(const  hal_ccb_rsa_param_t *p_in_curve_param)
{
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;


  /* Get the exp length */
  pka_ram_u32[PKA_MODULAR_EXP_IN_EXP_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->exp_size_byte,
                                                                     *(p_in_curve_param->p_mod));
  pka_ram_u32[PKA_MODULAR_EXP_IN_EXP_NB_BITS + 1U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the modulus n length */
  pka_ram_u32[PKA_MODULAR_EXP_IN_OP_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->modulus_size_byte,
                                                                    *(p_in_curve_param->p_mod));
  pka_ram_u32[PKA_MODULAR_EXP_IN_OP_NB_BITS + 4U] = 0x0U;
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input parameters modulus to PKA RAM */
  if (CCB_SetParam(p_in_curve_param->modulus_size_byte, PKA_MODULAR_EXP_PROTECT_IN_MODULUS,
                   p_in_curve_param->p_mod) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Set CCB parameters.
  * @param  modulus_size_byte Modulus size in bytes
  * @param  dst_address       Destination address
  * @param  p_src             Pointer to the source buffer
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_SetParam(uint32_t modulus_size_byte, uint32_t dst_address, const uint8_t *p_src)
{
  uint32_t modulus_size_words = (modulus_size_byte + 7UL) >> 3UL;
  uint32_t operand_size = 2UL * (modulus_size_words + 1UL);
  const uint32_t remainder_bytes = (modulus_size_byte) & 7UL;
  const uint32_t max_word_offset = (remainder_bytes != 0U) ? ((operand_size) - 4UL) : ((operand_size) - 2UL);
  uint32_t word_offset;
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;

  for (word_offset = 0U; word_offset < max_word_offset; word_offset += CCB_WORDS_PER_BLOCK)
  {
    uint32_t src_index = modulus_size_byte - ((word_offset * CCB_BYTES_PER_WORD) + 1U);
    CCB_Memcpy_u8_to_u64(&pka_ram_u32[dst_address + word_offset], &p_src[src_index]);

    if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (remainder_bytes != 0U)
  {
    uint32_t src_index = modulus_size_byte - ((word_offset * CCB_BYTES_PER_WORD) + 1U);
    CCB_Memcpy_TailBytesToU64(&pka_ram_u32[dst_address + word_offset], &p_src[src_index],
                              remainder_bytes);
    if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }
    word_offset += CCB_WORDS_PER_BLOCK;
  }

  CCB_PKA_PadEndRam(pka_ram_u32, dst_address + word_offset);

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  CCB wrapping key configuration.
  * @param  p_unwrapkey_context  Pointer to the context or parameters required by the unwrapkey
  * @param  ccb_operation      Operation
  * @retval HAL_ERROR          Error detected
  * @retval HAL_OK             Operation is successfully accomplished
  */
static hal_status_t CCB_Wrapping_Key_Config(void *p_unwrapkey_context, uint8_t ccb_operation)
{
  AES_TypeDef *p_saes_instance = SAES;
  ccb_hw_unwrap_key_context_t const *ctx = (ccb_hw_unwrap_key_context_t *)p_unwrapkey_context;
  uint32_t key_selection = (ctx->wrapping_key_context == HAL_CCB_KEY_HSW) ? AES_CR_KEYSEL_2 : AES_CR_KEYSEL_0;
  uint32_t saes_cr_value = 0U;

  /* 1. Software Unwrap Key Path */
  if (ctx->ccb_key_type == CCB_KEY_TYPE_SOFTWARE)
  {
    if (CCB_SAES_SW_UnwrapKey(p_saes_instance, p_unwrapkey_context) != HAL_OK)
    {
      return HAL_ERROR;
    }

    switch (ccb_operation)
    {
      case CCB_ECDSA_SIGN_CPU_BLOB_CREATION:
      case CCB_ECDSA_SIGN_RNG_BLOB_CREATION:
      case CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION:
      case CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION:
      case CCB_MODULAR_EXP_CPU_BLOB_CREATION:
        saes_cr_value = AES_CR_KEYSIZE | AES_CR_CHMOD_0 | AES_CR_CHMOD_1;
        break;

      default:
        saes_cr_value = AES_CR_KEYSIZE | AES_CR_CHMOD_0 | AES_CR_CHMOD_1
                        | AES_CR_MODE_1;
        break;
    }

    if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
    {
      return HAL_ERROR;
    }

    STM32_WRITE_REG(p_saes_instance->CR, saes_cr_value);
    return HAL_OK;
  }
  else
  {
    switch (ccb_operation)
    {
      case CCB_ECDSA_SIGN_CPU_BLOB_CREATION:
      case CCB_ECDSA_SIGN_RNG_BLOB_CREATION:
      case CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION:
      case CCB_ECC_SCALAR_MUL_RNG_BLOB_CREATION:
      case CCB_MODULAR_EXP_CPU_BLOB_CREATION:
        saes_cr_value = key_selection | AES_CR_KEYSIZE | AES_CR_CHMOD_0
                        | AES_CR_CHMOD_1;
        break;

      default:
        saes_cr_value = key_selection | AES_CR_KEYSIZE | AES_CR_CHMOD_0
                        | AES_CR_CHMOD_1 | AES_CR_MODE_1;
        break;
    }

    if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
    {
      return HAL_ERROR;
    }

    STM32_WRITE_REG(p_saes_instance->CR, saes_cr_value);

    /* HSW Key: Transfer SW Key from TAMP to SAES */
    if (ctx->wrapping_key_context == HAL_CCB_KEY_HSW)
    {
      for (uint32_t k = 0UL; k < 8UL; k++)
      {
        uint32_t tmp = LL_TAMP_BKP_GetRegister(k);
        if (tmp != 0UL)
        {
          return HAL_ERROR;
        }
      }
    }

    if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_KEYVALID, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

/**
  * @brief  Wrap the clear symmetric key.
  * @param  p_ccb_instance          CCB instance
  * @param  p_in_clear_user_key     Pointer to the clear AES key
  * @param  operation               CCB Operation
  * @param  p_wrapping_key_context  Pointer to a @ref hal_ccb_wrapping_sw_key_context_t structure
  * @param  p_out_wrapped_user_key  Pointer to the wrapped user key
  * @retval HAL_ERROR               Error detected
  * @retval HAL_OK                  Operation is successfully accomplished
  */
static hal_status_t CCB_WrapSymmetricKey(CCB_TypeDef *p_ccb_instance, const uint32_t *p_in_clear_user_key,
                                         uint32_t operation,
                                         const hal_ccb_wrapping_sw_key_context_t *p_wrapping_key_context,
                                         uint32_t *p_out_wrapped_user_key)
{
  uint32_t saes_in_count = 0UL;
  uint32_t saes_out_count = 0UL;
  const uint32_t *p_saes_in_buff = p_in_clear_user_key;
  uint32_t *p_ccb_out_buff = p_out_wrapped_user_key;
  AES_TypeDef *p_saes_instance = SAES;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, operation);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (p_wrapping_key_context->aes_algorithm != HAL_CCB_AES_ECB)
  {
    STM32_WRITE_REG(p_saes_instance->CR,
                    AES_CR_KEYSEL_0 | AES_CR_KMOD_0 | AES_CR_KEYSIZE
                    | AES_CR_CHMOD_0);
  }
  else
  {
    STM32_WRITE_REG(p_saes_instance->CR, AES_CR_KEYSEL_0
                    | AES_CR_KMOD_0 | AES_CR_KEYSIZE);
  }

  /* Wait for Key valid to be 1U */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_KEYVALID, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Disable the SAES peripheral */
  STM32_CLEAR_BIT(p_saes_instance->CR, AES_CR_EN);

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* set the operating mode and encryption operating mode*/
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_KMOD | AES_CR_MODE,
                   AES_CR_KMOD_0 | 0x0UL);

  if (p_wrapping_key_context->aes_algorithm != HAL_CCB_AES_ECB)
  {
    /* Set initialization vector from wrapping context */
    p_saes_instance->IVR3 = p_wrapping_key_context->p_init_vect[0];
    p_saes_instance->IVR2 = p_wrapping_key_context->p_init_vect[1];
    p_saes_instance->IVR1 = p_wrapping_key_context->p_init_vect[2];
    p_saes_instance->IVR0 = p_wrapping_key_context->p_init_vect[3];
  }

  /* Enable SAES */
  STM32_SET_BIT(p_saes_instance->CR, AES_CR_EN);

  while (saes_in_count < CCB_SYMMETRIC_KEY_SIZE_WORD)
  {
    for (uint32_t i = 0UL; i < 4UL; i++)
    {
      STM32_WRITE_REG(p_saes_instance->DINR, p_saes_in_buff[saes_in_count]);
      saes_in_count++;
    }

    /* Wait for CCF flag to be raised */
    if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear CCF Flag */
    STM32_SET_BIT(p_saes_instance->ICR, AES_ICR_CCF);

    /* Read the output block from the output FIFO */
    for (uint32_t i = 0UL; i < 4UL; i++)
    {
      p_ccb_out_buff[saes_out_count] = STM32_READ_REG(p_saes_instance->DOUTR);
      saes_out_count++;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Get optimal number of bits inside an array of u8.
  * @param  byte_number Number of u8 inside the array
  * @param  msb         Most significant uint8_t of the array
  * @retval uint32_t    returned size
  *
  */
static uint32_t CCB_GetOptBitSize_u8(uint32_t byte_number, uint8_t msb)
{
  uint32_t position;

  position = 32UL - __CLZ(msb);

  return (((byte_number - 1UL) * 8UL) + position);
}

/**
  * @brief  Copy uint32_t array to uint8_t array to fit PKA number representation.
  * @param  p_dst   Pointer to destination
  * @param  p_src   Pointer to source
  * @param  size    Number of uint8_t to copy
  */
static void CCB_Memcpy_u32_to_u8(volatile uint8_t *p_dst, const volatile uint32_t *p_src, size_t size)
{
  size_t num_words = size >> 2U;
  size_t rem_bytes = size & 3U;

  /* Apply __REV equivalent */
  for (size_t i = 0U; i < num_words; i++)
  {
    size_t dst_offset = size - 4U - (i << 2);
    uint32_t word = p_src[i];
    p_dst[dst_offset + 0U] = (uint8_t)((word >> 24U) & 0xFFU);
    p_dst[dst_offset + 1U] = (uint8_t)((word >> 16U) & 0xFFU);
    p_dst[dst_offset + 2U] = (uint8_t)((word >> 8U)  & 0xFFU);
    p_dst[dst_offset + 3U] = (uint8_t)(word & 0xFFU);
  }

  /* Manage the buffers not aligned on uint32_t */
  if (rem_bytes != 0U)
  {
    uint32_t word = p_src[num_words];
    for (size_t j = 0U; j < rem_bytes; j++)
    {
      p_dst[rem_bytes - 1U - j] = (uint8_t)((word >> (j << 3)) & 0xFFU);
    }
  }
}

/**
  * @brief  Copy uint8_t array to uint32_t array to fit PKA number representation.
  * @param  p_dst      Pointer to destination.
  * @param  p_src      Pointer to source.
  * @param  size       Number of uint8_t to copy (must be multiple of 4).
  */
static void CCB_Memcpy_u8_to_u32(volatile  uint32_t *p_dst, const volatile  uint8_t *p_src, size_t size)
{
  size_t num_words = (size + 3U) >> 2U;

  for (size_t i = 0U; i < num_words; ++i)
  {
    uint32_t word = 0U;
    for (size_t j = 0U; j < 4U; ++j)
    {
      size_t byte_index = size - (i << 2) - 1U - j;
      if (((i << 2) + j) < size)
      {
        word |= ((uint32_t)p_src[byte_index]) << (j << 3);
      }
    }
    p_dst[i] = word;
  }

  /* Zero padding for the next two words */
  p_dst[num_words] = 0U;
  p_dst[num_words + 1U] = 0U;
}

/**
  * @brief  Copy uint32_t array to uint32_t array.
  * @param  p_dst    Pointer to destination
  * @param  p_src    Pointer to source
  * @param  size     Number of u32 to be handled
  */
static void CCB_Memcpy_u32_to_u32(volatile uint32_t *p_dst, const volatile uint32_t *p_src, size_t size)
{
  for (uint32_t index = 0UL; index < size; index++)
  {
    p_dst[index] = p_src[index];
  }
}

/**
  * @brief  Copy uint8_t array to uint64_t array to fit number representation.
  * @param  p_dst Pointer to destination
  * @param  p_src Pointer to source
  */
static void CCB_Memcpy_u8_to_u64(volatile uint32_t *p_dst, const volatile uint8_t *p_src)
{
  uint32_t word0 = 0U;
  uint32_t word1 = 0U;
  const volatile uint8_t *p_source = p_src;

  for (uint32_t i = 0U; i < 4U; ++i)
  {
    word0 |= ((uint32_t)(*p_source)) << (8U * i);
    --p_source;
  }

  for (uint32_t i = 0U; i < 4U; ++i)
  {
    word1 |= ((uint32_t)(*p_source)) << (8U * i);
    --p_source;
  }
  p_dst[0] = word0;
  p_dst[1] = word1;
}

/**
  * @brief  Copy up to 7 bytes from a uint8_t array into two uint32_t words.
  * @param  p_dst     Pointer to destination
  * @param  p_src     Pointer to source
  * @param  size    Pointer to number of uint8_t to copy
  */
static void CCB_Memcpy_TailBytesToU64(volatile uint32_t *p_dst, const volatile uint8_t *p_src, size_t size)
{
  uint32_t word0 = 0U;
  uint32_t word1 = 0U;
  const volatile uint8_t *p_source = p_src;

  /* Copy up to 4 bytes into word0 (least significant bytes) */
  for (uint32_t i = 0U; (i < 4U) && (i < size); ++i)
  {
    word0 |= ((uint32_t)(*p_source)) << (8U * i);
    --p_source;
  }

  /* Copy next up to 3 bytes into word1 (if size > 4) */
  for (uint32_t i = 4U; i < size; ++i)
  {
    word1 |= ((uint32_t)(*p_source)) << (8U * (i - 4U));
    --p_source;
  }

  p_dst[0] = word0;
  p_dst[1] = word1;
}

/**
  * @brief  Blob Creation initial phase.
  * @param  p_iv              Pointer to the initial vector
  * @param  randoms           Random Number
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_BlobCreation_InitialPhase(uint32_t *p_iv, uint32_t randoms)
{
  AES_TypeDef *p_saes_instance = SAES;
  RNG_TypeDef *p_rng_instance = RNG;
  PKA_TypeDef const *p_pka_instance = PKA;

  /* Load IVs from RNG to SAES */
  STM32_WRITE_REG(p_saes_instance->IVR0, CCB_IV0_VALUE); /* SAES_IVR0 that must be equal to 0x2 */

  if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* For IV1, IV2 and IV3, random generated values are loaded from RNG to SAES by CCB */
  STM32_WRITE_REG(p_saes_instance->IVR1, CCB_FAKE_VALUE);
  if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  STM32_WRITE_REG(p_saes_instance->IVR2, CCB_FAKE_VALUE);
  if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  STM32_WRITE_REG(p_saes_instance->IVR3, CCB_FAKE_VALUE);
  if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* if an error occurs, RNGERRF flag is 1U in PKA */
  if (CCB_GET_PKA_FLAG(p_pka_instance, PKA_SR_RNGERRF) == 1U)
  {
    return HAL_ERROR;
  }

  /* Read back IVs from SAES */
  p_iv[3] = (STM32_READ_REG(p_saes_instance->IVR3) ^ randoms);
  p_iv[2] = (STM32_READ_REG(p_saes_instance->IVR2) ^ randoms);
  p_iv[1] = (STM32_READ_REG(p_saes_instance->IVR1) ^ randoms);
  p_iv[0] = (STM32_READ_REG(p_saes_instance->IVR0) ^ randoms);
  /* Set EN in SAES_CR*/
  STM32_SET_BIT(p_saes_instance->CR, AES_CR_EN);
  /* Wait until CCF is 1U in SAES_ISR (end of GCM init) */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Header phase and trig OPSTEP transition 0x2 --> 0x3 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_0 | AES_CR_EN);

  return HAL_OK;
}


/**
  * @brief  Blob usage initial phase.
  * @param  p_ccb_instance    CCB instance
  * @param  p_iv              Pointer to the initial vector
  * @param  p_tag             Pointer to the tag
  * @param  randoms           Random Number
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_BlobUse_InitialPhase(CCB_TypeDef *p_ccb_instance, const uint32_t *p_iv, uint32_t *p_tag,
                                             uint32_t randoms)
{
  AES_TypeDef *p_saes_instance = SAES;

  /* Set IVs from created Blob */
  STM32_WRITE_REG(p_saes_instance->IVR0, (p_iv[0] ^ randoms));
  STM32_WRITE_REG(p_saes_instance->IVR1, (p_iv[1] ^ randoms));
  STM32_WRITE_REG(p_saes_instance->IVR2, (p_iv[2] ^ randoms));
  STM32_WRITE_REG(p_saes_instance->IVR3, (p_iv[3] ^ randoms));

  /* Set EN bit in SAES_CR*/
  STM32_SET_BIT(p_saes_instance->CR, AES_CR_EN);

  /* Wait until CCF is 1U in SAES_ISR (end of GCM init) */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write reference tag in CCB */
  for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
  {
    STM32_WRITE_REG(p_ccb_instance->REFTAGR[count], (p_tag[count] ^ randoms));
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Header phase and trig OPSTEP transition 0x12 --> 0x13 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE | AES_CR_EN,
                   AES_CR_CPHASE_0 | AES_CR_EN);

  return HAL_OK;
}

/**
  * @brief  Blob creation initial phase.
  * @param  operation       CCB Operation
  * @param  p_tag           Pointer to the tag
  * @param  size_param      size of parameter
  * @param  randoms         Random Number
  * @retval HAL_ERROR       Error detected
  * @retval HAL_OK          Operation is successfully accomplished
  */
static hal_status_t CCB_BlobCreation_FinalPhase(uint32_t operation, uint32_t *p_tag, uint32_t size_param,
                                                uint32_t randoms)
{
  uint32_t last_block[4U] = {0};
  uint32_t operand_size = 2UL * (((size_param + 7UL) >> 3UL) + 1UL);
  const uint32_t cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  AES_TypeDef *p_saes_instance = SAES;

  CCB_CLEAR_PKA_FLAG(PKA, PKA_CLRFR_CMFC);

  /* Preparing last Block */
  switch (operation)
  {
    case CCB_MODULAR_EXP_CPU_BLOB_CREATION:
      /* bitsize of exp length, modulus n length  and modulus (address and data)*/
      last_block[1] = ((64UL * 2UL) + (operand_size * 32UL)) * 2UL;
      last_block[3] = cipherkey_size * 32UL;
      break;
    case CCB_ECDSA_SIGN_CPU_BLOB_CREATION:
    case CCB_ECDSA_SIGN_RNG_BLOB_CREATION:
      /* bitsize of |a|, b, p, n, Gx and Gy --> size_param*8*6*2 (address and data); n length,
         plength and a sign --> 64*3*2 (address and data) */
      last_block[1] = (((operand_size * 32UL) * 6UL) + (3UL * 64UL)) * 2UL;
      last_block[3] = cipherkey_size * 32UL;
      break;
    default:
      /* bitsize of |a|, b, p, n, --> size_param*8*4*2 (address and data); n length,
         plength and a sign --> 64*3*2 (address and data) */
      last_block[1] = (((operand_size * 32UL) * 4UL) + (3UL * 64UL)) * 2UL;
      /* Set ciphered data with size 256 */
      last_block[3] = cipherkey_size * 32UL;
      break;
  }

  for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
  {
    STM32_WRITE_REG(p_saes_instance->DINR, last_block[count]);
  }

  /* Wait until CCF flag is Set in SAES */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Read authentication tag */
  for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
  {
    p_tag[count] = (STM32_READ_REG(p_saes_instance->DOUTR) ^ randoms);
  }

  return HAL_OK;
}

/**
  * @brief  Blob usage final phase.
  * @param  p_saes_instance   SAES instance
  * @param  operation         CCB Operation
  * @param  size_param        Size of parameter
  * @retval HAL_ERROR         Error detected
  * @retval HAL_OK            Operation is successfully accomplished
  */
static hal_status_t CCB_BlobUse_FinalPhase(AES_TypeDef *p_saes_instance, uint32_t operation,
                                           uint32_t size_param)
{
  uint32_t last_block[4U] = {0};
  uint32_t operand_size = 2UL * (((size_param + 7UL) >> 3UL) + 1UL);
  const uint32_t cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH final phase */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE, AES_CR_CPHASE_0
                   | AES_CR_CPHASE_1);

  /* Preparing last Block */
  switch (operation)
  {
    case CCB_MODULAR_EXP_BLOB_USE:
      /* bitsize of exp length, modulus n length  and modulus (address and data)*/
      last_block[1] = ((64UL * 2UL) + (operand_size * 32UL)) * 2UL;
      /* Ciphered data size */
      last_block[3] = cipherkey_size * 32UL;
      break;
    case CCB_ECDSA_SIGN_BLOB_USE:
      /* bitsize of |a|, b, p, n, Gx and Gy --> size_param*8*6*2 (address and data); n length,
         plength and a sign --> 64*3*2 (address and data) */
      last_block[1] = (((operand_size * 32UL) * 6UL) + (3UL * 64UL)) * 2UL;
      /* One ciphered data block with size 256 */
      last_block[3] = cipherkey_size * 32UL;
      break;
    default:
      /* bitsize of |a|, b, p, n, --> size_param*8*4*2 (address and data); n length,
         plength and a sign --> 64*3*2 (address and data) */
      last_block[1] = (((operand_size * 32UL) * 4UL) + (3UL * 64UL)) * 2UL;
      /* One ciphered data block with size 256 */
      last_block[3] = cipherkey_size * 32UL;
      break;
  }

  for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
  {
    STM32_WRITE_REG(p_saes_instance->DINR, last_block[count]);
  }

  /* Wait until CCF flag is 1U in SAES  */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Read authentication tag and check blob integrity as the event that triggers OPSTEP transition 0x17 --> 0x18 */
  for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
  {
    if (STM32_READ_REG(p_saes_instance->DOUTR) != 0UL)
    {
      return HAL_ERROR;
    }
  }

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Blob creation for ECDSA Signature.
  * @param  p_ccb_instance        CCB instance
  * @param  p_in_curve_param      Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_unwrapkey_context     Pointer to the context or parameters required by the unwrapkey
  * @param  p_clear_private_key   Pointer to the p_clear_private_key
  * @param  p_iv                  Pointer to the initial vector
  * @param  p_tag                 Pointer to the tag
  * @param  p_wrapped_key         Pointer to the wrapped key
  * @param  randoms               Random Number
  * @param  p_hash                Pointer to the hash
  * @param  p_signature           Pointer to the signature
  * @param  ccb_operation         CCB Operation
  * @retval HAL_ERROR             Error detected
  * @retval HAL_OK                Operation is successfully accomplished
  */
static hal_status_t CCB_ECDSA_SignBlobCreation(CCB_TypeDef *p_ccb_instance,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               void *p_unwrapkey_context, const uint8_t *p_clear_private_key,
                                               uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_key,
                                               uint32_t randoms, uint8_t *p_hash, hal_ccb_ecdsa_sign_t *p_signature,
                                               uint8_t ccb_operation)
{
  uint32_t operand_size = 2UL * (((p_in_curve_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, ccb_operation);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, ccb_operation) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Move the input hash value to PKA RAM */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECDSA_SIGN_IN_HASH_E ], p_hash,
                       p_in_curve_param->prime_order_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32, PKA_ECDSA_SIGN_IN_HASH_E  + ((p_in_curve_param->prime_order_size_byte + 3UL) >> 2UL));

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, ccb_operation) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x02U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobCreation_InitialPhase(p_iv, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x03U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_ECDSASign_SetParam(p_in_curve_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x3 --> 0x4 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  switch (ccb_operation)
  {
    case CCB_ECDSA_SIGN_CPU_BLOB_CREATION:

      if (CCB_WaitOperStep(p_ccb_instance, 0x04U) != HAL_OK)
      {
        return HAL_ERROR;
      }

      CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

      /* Copy private key d by CPU (user key) */
      CCB_PKA_WriteClearTextData(pka_ram_u32, (uint16_t)PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D,
                                 p_clear_private_key, p_in_curve_param->modulus_size_byte, operand_size);

      /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x4 --> 0x8 */
      if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
      {
        return HAL_ERROR;
      }
      break;
    default:
      /* Wait until OPSTEP is set to 0x6 */
      if (CCB_WaitOperStep(p_ccb_instance, 0x06U) != HAL_OK)
      {
        return HAL_ERROR;
      }

      CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

      /* Write private d Key from RNG */
      if (CCB_WriteKeyFromRNG((uint16_t)PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D, operand_size) != HAL_OK)
      {
        return HAL_ERROR;
      }
      break;
  }

  /* Wait until OPSTEP is set to 0x08 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x08U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Read ciphered private Key d */
  if (CCB_ReadCipheredPrivateKey((uint16_t)PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D, operand_size, p_wrapped_key,
                                 randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x9 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x09U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write random k */
  if (CCB_WriteKeyFromRNG((uint16_t)PKA_ECDSA_SIGN_IN_K, operand_size) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH final phase */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_0 | AES_CR_CPHASE_1);

  /* Final phase processing */
  if (CCB_BlobCreation_FinalPhase(ccb_operation, p_tag, p_in_curve_param->modulus_size_byte, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x0A --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Check PKA Operation error result */
  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_ECDSA_SIGN_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }

  /* Read r part signature */
  CCB_Memcpy_u32_to_u8(p_signature->p_r_sign, (__IOM uint32_t *) &pka_ram_u32[PKA_ECDSA_SIGN_OUT_SIGNATURE_R],
                       p_in_curve_param->modulus_size_byte);

  /* Read s part signature */
  CCB_Memcpy_u32_to_u8(p_signature->p_s_sign, (__IOM uint32_t *)&pka_ram_u32[PKA_ECDSA_SIGN_OUT_SIGNATURE_S],
                       p_in_curve_param->modulus_size_byte);

  return HAL_OK;
}

/**
  * @brief  Protected ECDSA compute public key.
  * @param  p_ccb_instance         CCB instance
  * @param  p_in_curve_param       Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_unwrapkey_context    Pointer to the context or parameters required by the unwrapkey
  * @param  p_iv                   Pointer to the initial vector
  * @param  p_tag                  Pointer to the tag
  * @param  p_wrapped_key          Pointer to the wrapped key
  * @param  randoms                Random Number
  * @param  p_output_point         Pointer to a @ref hal_ccb_ecc_point_t structure
  * @retval HAL_ERROR              Error detected
  * @retval HAL_OK                 Operation is successfully accomplished
  */
static hal_status_t CCB_ECDSA_ComputePublicKey(CCB_TypeDef *p_ccb_instance,
                                               const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                               void *p_unwrapkey_context, uint32_t *p_iv,
                                               uint32_t *p_tag, uint32_t *p_wrapped_key, uint32_t randoms,
                                               hal_ccb_ecc_point_t *p_output_point)
{
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, CCB_ECC_SCALAR_MUL_BLOB_USE);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, CCB_ECC_SCALAR_MUL_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write Customized point coordinate */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X ],
                       p_in_curve_param->p_point_x, p_in_curve_param->modulus_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X + ((p_in_curve_param->modulus_size_byte + 3UL) >> 2UL));

  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y ],
                       p_in_curve_param->p_point_y, p_in_curve_param->modulus_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y + ((p_in_curve_param->modulus_size_byte + 3UL) >> 2UL));

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, CCB_ECC_SCALAR_MUL_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x012 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x012U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobUse_InitialPhase(p_ccb_instance, p_iv, p_tag, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x13 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x13U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_ECDSASign_SetParam(p_in_curve_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x13 --> 0x14 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x14 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x14U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WriteWrappedKey((uint16_t)PKA_ECDSA_SIGN_IN_K, p_wrapped_key, p_in_curve_param->prime_order_size_byte,
                          randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobUse_FinalPhase(p_saes_instance, CCB_ECDSA_SIGN_BLOB_USE,
                             p_in_curve_param->modulus_size_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x18U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x18 --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Check PKA Operation error result */
  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_ECC_SCALAR_MUL_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }

  /* P coordinate x */
  CCB_Memcpy_u32_to_u8(p_output_point->p_point_x, &pka_ram_u32[PKA_POINT_CHECK_IN_INITIAL_POINT_X],
                       p_in_curve_param->modulus_size_byte);

  /* P coordinate y */
  CCB_Memcpy_u32_to_u8(p_output_point->p_point_y, &pka_ram_u32[PKA_POINT_CHECK_IN_INITIAL_POINT_Y],
                       p_in_curve_param->modulus_size_byte);

  return HAL_OK;
}

/**
  * @brief  Blob Usage: ECDSA Signature.
  * @param  p_ccb_instance      CCB instance
  * @param  p_in_curve_param    Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_unwrapkey_context   Pointer to the context or parameters required by the unwrapkey
  * @param  p_in_ecdsa_key_blob Pointer to a @ref hal_ccb_ecdsa_key_blob_t structure
  * @param  p_in_hash           Pointer to the hash
  * @param  hash_size           Specify the size of the hash message
  * @param  p_out_signature     Pointer to the signature
  * @retval HAL_ERROR           Error detected
  * @retval HAL_OK              Operation is successfully accomplished
  */
static hal_status_t CCB_ECDSA_Sign(CCB_TypeDef *p_ccb_instance, const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                   void *p_unwrapkey_context, hal_ccb_ecdsa_key_blob_t *p_in_ecdsa_key_blob,
                                   const uint8_t *p_in_hash, uint8_t hash_size, hal_ccb_ecdsa_sign_t *p_out_signature)
{
  uint32_t word_offset;
  uint32_t operand_size;
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  RNG_TypeDef *p_rng_instance = RNG;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, CCB_ECDSA_SIGN_BLOB_USE);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, CCB_ECDSA_SIGN_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, CCB_ECDSA_SIGN_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x012 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x012U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set Hash message */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECDSA_SIGN_IN_HASH_E ], p_in_hash, hash_size);

  if (CCB_BlobUse_InitialPhase(p_ccb_instance, p_in_ecdsa_key_blob->p_iv, p_in_ecdsa_key_blob->p_tag, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x13 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x13U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_ECDSASign_SetParam(p_in_curve_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x13 --> 0x14 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x14 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x14U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write encrypted Key*/
  if (CCB_WriteWrappedKey((uint16_t)PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D, p_in_ecdsa_key_blob->p_wrapped_key,
                          p_in_curve_param->prime_order_size_byte, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x14 --> 0x16 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x16 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x16U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set operand size (in word 32bits)*/
  operand_size = 2UL * ((uint32_t)(((p_in_curve_param->modulus_size_byte) + 7UL) >> 3UL) + 1UL);

  /* Write random k */
  for (word_offset = 0UL; word_offset < (operand_size - 2UL); word_offset++)
  {
    /* Wait for RNG Data Ready flag */
    if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
    {
      return HAL_ERROR;
    }
    pka_ram_u32[PKA_ECDSA_SIGN_IN_K + word_offset] = CCB_FAKE_VALUE;

  }
  CCB_PKA_PadEndRam(pka_ram_u32, (PKA_ECDSA_SIGN_IN_K + word_offset));
  /* Wait for PKA RNGOK flag : GCMPH = 0x3 (final phase) as events that trig OPSTEP transition 0x16 --> 0x17 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_RNGOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x17 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x17U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobUse_FinalPhase(p_saes_instance, CCB_ECDSA_SIGN_BLOB_USE,
                             p_in_curve_param->modulus_size_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x18U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x18 --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_ECDSA_SIGN_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }
  /* Read r part signature */
  CCB_Memcpy_u32_to_u8(p_out_signature->p_r_sign, (__IOM uint32_t *) &pka_ram_u32[PKA_ECDSA_SIGN_OUT_SIGNATURE_R],
                       p_in_curve_param->modulus_size_byte);

  /* Read s part signature */
  CCB_Memcpy_u32_to_u8(p_out_signature->p_s_sign, (__IOM uint32_t *)&pka_ram_u32[PKA_ECDSA_SIGN_OUT_SIGNATURE_S],
                       p_in_curve_param->modulus_size_byte);

  return HAL_OK;
}

/**
  * @brief  Blob creation for ECC Scalar Multiplication.
  * @param  p_ccb_instance         CCB instance
  * @param  p_in_curve_param       Pointer to a @ref hal_ccb_ecc_mul_curve_param_t structure
  * @param  p_unwrapkey_context    Pointer to the context or parameters required by the unwrapkey
  * @param  p_clear_private_key    Pointer to the p_clear_private_key
  * @param  p_iv                   Pointer to the initial vector
  * @param  p_tag                  Pointer to the tag
  * @param  p_wrapped_key          Pointer to the wrapped key
  * @param  randoms                Random Number
  * @param  scalar_mul_x_ref       Buffer for scalar multiplication point x reference
  * @param  scalar_mul_y_ref       Buffer for scalar multiplication point y reference
  * @param  ccb_operation          CCB Operation
  * @retval HAL_ERROR              Error detected
  * @retval HAL_OK                 Operation is successfully accomplished
  */
static hal_status_t CCB_ECC_ScalarMulBlobCreation(CCB_TypeDef *p_ccb_instance,
                                                  const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                                  void *p_unwrapkey_context, const uint8_t *p_clear_private_key,
                                                  uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_key,
                                                  uint32_t randoms, uint32_t *scalar_mul_x_ref,
                                                  uint32_t *scalar_mul_y_ref, uint8_t ccb_operation)
{
  uint32_t operand_size = 2UL * (((p_in_curve_param->prime_order_size_byte + 7UL) >> 3UL) + 1UL);
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, ccb_operation);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, ccb_operation) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write  point G coordinate */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X ],
                       p_in_curve_param->p_point_x, p_in_curve_param->prime_order_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X + ((p_in_curve_param->prime_order_size_byte + 3UL) >> 2UL));
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y ],
                       p_in_curve_param->p_point_y, p_in_curve_param->prime_order_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y + ((p_in_curve_param->prime_order_size_byte + 3UL) >> 2UL));

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, ccb_operation) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x02 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x02U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobCreation_InitialPhase(p_iv, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x03 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x03U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_ECCMul_SetParam(p_in_curve_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x3 --> 0x4 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x04 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x04U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  switch (ccb_operation)
  {
    case CCB_ECC_SCALAR_MUL_CPU_BLOB_CREATION:
      CCB_PKA_WriteClearTextData(pka_ram_u32, (uint16_t)PKA_ECC_SCALAR_MUL_IN_K,
                                 p_clear_private_key, p_in_curve_param->modulus_size_byte, operand_size);

      /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x4 --> 0x8 */
      if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
      {
        return HAL_ERROR;
      }
      break;
    default:
      /* Write scalar k when RNG */
      if (CCB_WriteKeyFromRNG((uint16_t)PKA_ECC_SCALAR_MUL_IN_K, operand_size) != HAL_OK)
      {
        return HAL_ERROR;
      }
      break;
  }

  /* Wait until OPSTEP is set to 0x08 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x08U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Read ciphered scalar k */
  if (CCB_ReadCipheredPrivateKey((uint16_t)PKA_ECC_SCALAR_MUL_IN_K, operand_size, p_wrapped_key, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH final phase */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_0 | AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x0A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x0AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobCreation_FinalPhase(ccb_operation, p_tag, p_in_curve_param->modulus_size_byte, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x0A --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Check PKA Operation error result */
  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_ECC_SCALAR_MUL_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }

  /* Move the result from appropriate location */
  CCB_Memcpy_u32_to_u32(scalar_mul_x_ref, &pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X],
                        ((p_in_curve_param->modulus_size_byte + 3U) >> 2U));
  CCB_Memcpy_u32_to_u32(scalar_mul_y_ref, &pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y],
                        ((p_in_curve_param->modulus_size_byte + 3U) >> 2U));

  return HAL_OK;
}

/**
  * @brief  Blob creation for RSA Modular exponentiation.
  * @param  p_ccb_instance          CCB instance
  * @param  p_param                 Pointer to a @ref hal_ccb_rsa_param_t structure
  * @param  p_unwrapkey_context     Pointer to the context or parameters required by the unwrapkey
  * @param  p_rsa_clear_private_key Pointer to a @ref hal_ccb_rsa_clear_key_t
  * @param  p_iv                    Pointer to the initial vector
  * @param  p_tag                   Pointer to the tag
  * @param  p_wrapped_exp           Pointer to the wrapped exp
  * @param  p_wrapped_phi           Pointer to the wrapped phi
  * @param  randoms                 Random Number
  * @param  p_operand               Pointer to to the constant K as operand A
  * @param  p_reference_modular_exp Pointer to the output reference modular exponentiation
  * @retval HAL_ERROR               Error detected
  * @retval HAL_OK                  Operation is successfully accomplished
  */
static hal_status_t CCB_RSA_ExpBlobCreation(CCB_TypeDef *p_ccb_instance, const hal_ccb_rsa_param_t *p_param,
                                            void *p_unwrapkey_context,
                                            const hal_ccb_rsa_clear_key_t *p_rsa_clear_private_key,
                                            uint32_t *p_iv, uint32_t *p_tag, uint32_t *p_wrapped_exp,
                                            uint32_t *p_wrapped_phi, uint32_t randoms, uint8_t *p_operand,
                                            uint32_t *p_reference_modular_exp)
{
  uint32_t operand_size = 2UL * (((p_param->modulus_size_byte + 7UL) >> 3UL) + 1UL);
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, CCB_MODULAR_EXP_CPU_BLOB_CREATION);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, CCB_MODULAR_EXP_CPU_BLOB_CREATION) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write a constant K as operand A (base of exponentiation) in the PKA RAM */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_MODULAR_EXP_PROTECT_IN_EXPONENT_BASE ], p_operand,
                       p_param->modulus_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_MODULAR_EXP_PROTECT_IN_EXPONENT_BASE + ((p_param->modulus_size_byte + 3UL) >> 2UL));

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, CCB_MODULAR_EXP_CPU_BLOB_CREATION) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x02 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x02U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobCreation_InitialPhase(p_iv, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x03 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x03U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_RSAModExp_SetParam(p_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x3 --> 0x4 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x04 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x04U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Clear CMF flag before starting */
  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Write clear-text exponent e */
  CCB_PKA_WriteClearTextData(pka_ram_u32, (uint16_t)PKA_MODULAR_EXP_PROTECT_IN_EXPONENT,
                             p_rsa_clear_private_key->p_exp, p_param->modulus_size_byte, operand_size);

  /* Wait until DATAOKF flag is 1U in PKA and trigger OPSTEP transition 0x4 --> 0x5 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x05 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x05) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Write clear-text phi */
  CCB_PKA_WriteClearTextData(pka_ram_u32, (uint16_t)PKA_MODULAR_EXP_PROTECT_IN_PHI,
                             p_rsa_clear_private_key->p_phi, p_param->modulus_size_byte, operand_size);

  /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x5 --> 0x8:   */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x08 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x08U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Read ciphered exponent e */
  if (CCB_ReadCipheredPrivateKey((uint16_t)PKA_MODULAR_EXP_PROTECT_IN_EXPONENT, operand_size, p_wrapped_exp,
                                 randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x09 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x09U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  /* Read ciphered phi */
  if (CCB_ReadCipheredPrivateKey((uint16_t)PKA_MODULAR_EXP_PROTECT_IN_PHI, operand_size,
                                 p_wrapped_phi, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH final phase */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_0 | AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x0A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x0AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Final phase processing */
  if (CCB_BlobCreation_FinalPhase(CCB_MODULAR_EXP_CPU_BLOB_CREATION, p_tag, p_param->modulus_size_byte,
                                  randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set PKA START operation bit and trig OPSTEP transition 0x0A --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Check PKA Operation error result */
  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_MODULAR_EXP_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }

  uint32_t modulus_words_count = (p_param->modulus_size_byte + 3UL) >> 2UL;

  /* Read result output */
  for (uint32_t word_offset = 0U; word_offset < modulus_words_count; word_offset++)
  {
    p_reference_modular_exp[word_offset] = pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT + word_offset];
  }

  return HAL_OK;
}

/**
  * @brief  Protected ECC compute scalar multiplication.
  * @param  p_ccb_instance         CCB instance
  * @param  p_in_curve_param       Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_unwrapkey_context    Pointer to the context or parameters required by the unwrapkey
  * @param  p_iv                   Pointer to the initial vector
  * @param  p_tag                  Pointer to the tag
  * @param  p_wrapped_key          Pointer to the wrapped key
  * @param  randoms                Random Number
  * @param  p_input_point          Pointer to a @ref hal_ccb_ecc_point_t structure
  * @param  p_output_point         Pointer to a @ref hal_ccb_ecc_point_t structure
  * @retval HAL_ERROR              Error detected
  * @retval HAL_OK                 Operation is successfully accomplished
  */
static hal_status_t CCB_ECC_ComputeScalarMul(CCB_TypeDef *p_ccb_instance,
                                             const hal_ccb_ecc_mul_curve_param_t *p_in_curve_param,
                                             void *p_unwrapkey_context, uint32_t *p_iv, uint32_t *p_tag,
                                             uint32_t *p_wrapped_key, uint32_t randoms,
                                             const hal_ccb_ecc_point_t *p_input_point,
                                             hal_ccb_ecc_point_t *p_output_point)
{
  const uint8_t *p_point_x = (p_input_point == NULL) ? p_in_curve_param->p_point_x : p_input_point->p_point_x;
  const uint8_t *p_point_y = (p_input_point == NULL) ? p_in_curve_param->p_point_y : p_input_point->p_point_y;
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, CCB_ECC_SCALAR_MUL_BLOB_USE);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, CCB_ECC_SCALAR_MUL_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Write Customized point coordinate */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X ],
                       p_point_x, p_in_curve_param->modulus_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X + ((p_in_curve_param->modulus_size_byte + 3UL) >> 2UL));

  CCB_Memcpy_u8_to_u32((&pka_ram_u32[PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y ]),
                       p_point_y, p_in_curve_param->modulus_size_byte);
  CCB_PKA_PadEndRam(pka_ram_u32,
                    PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y + ((p_in_curve_param->modulus_size_byte + 3UL) >> 2UL));

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, CCB_ECC_SCALAR_MUL_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x012 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x012U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobUse_InitialPhase(p_ccb_instance, p_iv, p_tag, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x13 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x13U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_ECCMul_SetParam(p_in_curve_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x13 --> 0x14 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x14 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x14U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WriteWrappedKey((uint16_t)PKA_ECDSA_SIGN_IN_K, p_wrapped_key, p_in_curve_param->prime_order_size_byte,
                          randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Final phase processing */
  if (CCB_BlobUse_FinalPhase(p_saes_instance, CCB_ECC_SCALAR_MUL_BLOB_USE,
                             p_in_curve_param->modulus_size_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x18U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x18 --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Check PKA Operation error result */
  if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_ECC_SCALAR_MUL_OUT_ERROR) != CCB_PKA_RESULT_OK)
  {
    return HAL_ERROR;
  }

  if (p_input_point != NULL)
  {
    /* P coordinate x */
    CCB_Memcpy_u32_to_u8(p_output_point->p_point_x, &pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_X],
                         p_in_curve_param->modulus_size_byte);

    /* P coordinate y */
    CCB_Memcpy_u32_to_u8(p_output_point->p_point_y, &pka_ram_u32[PKA_ECC_SCALAR_MUL_OUT_RESULT_Y],
                         p_in_curve_param->modulus_size_byte);

  }
  return HAL_OK;
}

/**
  * @brief  Blob Usage: RSA Compute Modular exponentiation.
  * @param  p_ccb_instance          CCB instance
  * @param  p_param                 Pointer to a @ref hal_ccb_rsa_param_t structure
  * @param  p_unwrapkey_context     Pointer to the context or parameters required by the unwrapkey
  * @param  p_iv                    Pointer to the initial vector
  * @param  p_tag                   Pointer to the tag
  * @param  p_wrapped_exp           Pointer to the wrapped exp
  * @param  p_wrapped_phi           Pointer to the wrapped phi
  * @param  randoms                 Random Number
  * @param  p_operand               Pointer to the operand
  * @param  p_modular_exp           Pointer to the modular exponent
  * @retval HAL_ERROR               Error detected
  * @retval HAL_OK                  Operation is successfully accomplished
  */
static hal_status_t CCB_RSA_ComputeModularExp(CCB_TypeDef *p_ccb_instance, const hal_ccb_rsa_param_t *p_param,
                                              void *p_unwrapkey_context, uint32_t *p_iv,
                                              uint32_t *p_tag, uint32_t *p_wrapped_exp, uint32_t *p_wrapped_phi,
                                              uint32_t randoms, const uint8_t *p_operand, uint8_t *p_modular_exp)
{
  AES_TypeDef *p_saes_instance = SAES;
  PKA_TypeDef *p_pka_instance = PKA;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Set operation in CCB */
  STM32_MODIFY_REG(p_ccb_instance->CR, CCB_CR_CCOP, CCB_MODULAR_EXP_BLOB_USE);

  /* Wait until OPSTEP is set to 0x01 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_PKA_SetOperation(p_pka_instance, CCB_MODULAR_EXP_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait for Galois Filter End of Computation */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_Wrapping_Key_Config(p_unwrapkey_context, CCB_MODULAR_EXP_BLOB_USE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x012 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x012U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set Operand A - base of exponentiation */
  CCB_Memcpy_u8_to_u32(&pka_ram_u32[PKA_MODULAR_EXP_PROTECT_IN_EXPONENT_BASE], p_operand,
                       p_param->modulus_size_byte);

  if (CCB_BlobUse_InitialPhase(p_ccb_instance, p_iv, p_tag, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x13 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x13U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_RSAModExp_SetParam(p_param) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set SAES GCMPH Payload phase and trig OPSTEP that trig OPSTEP transition 0x13 --> 0x14 */
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_CPHASE,
                   AES_CR_CPHASE_1);

  /* Wait until OPSTEP is set to 0x14 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x14U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WriteWrappedKey((uint16_t)PKA_MODULAR_EXP_PROTECT_IN_EXPONENT, p_wrapped_exp,
                          p_param->modulus_size_byte, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x14 --> 0x15 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x15U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  CCB_CLEAR_PKA_FLAG(p_pka_instance, PKA_CLRFR_CMFC);

  if (CCB_WriteWrappedKey((uint16_t)PKA_MODULAR_EXP_PROTECT_IN_PHI, p_wrapped_phi,
                          p_param->modulus_size_byte, randoms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x15 --> 0x17 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x17 */
  if (CCB_WaitOperStep(p_ccb_instance, 0x17U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_BlobUse_FinalPhase(p_saes_instance, CCB_MODULAR_EXP_BLOB_USE,
                             p_param->modulus_size_byte) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_WaitOperStep(p_ccb_instance, 0x18U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set PKA START operation bit and trig OPSTEP transition 0x18 --> 0x19 */
  LL_PKA_Start(p_pka_instance);

  /* Wait until end of operation flag is 1U in PKA and trig OPSTEP transition 0x19 --> 0x1A */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until OPSTEP is set to 0x1A */
  if (CCB_WaitOperStep(p_ccb_instance, 0x1AU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (p_modular_exp != NULL)
  {
    /* Check PKA Operation error result */
    if ((uint32_t)CCB_PKA_RAM_WORD_ACCESS(p_pka_instance, PKA_MODULAR_EXP_OUT_ERROR) != CCB_PKA_RESULT_OK)
    {
      return HAL_ERROR;
    }

    /* Read result output */
    CCB_Memcpy_u32_to_u8(p_modular_exp, &pka_ram_u32[PKA_MODULAR_EXP_OUT_RESULT],
                         p_param->modulus_size_byte);
  }
  return HAL_OK;
}

/**
  * @brief  Verify the validity of a signature using elliptic curves over prime fields in blocking mode.
  * @param  p_pka_instance              PKA instance
  * @param  p_in_curve_param            Pointer to a @ref hal_ccb_ecdsa_curve_param_t structure
  * @param  p_public_key_out            Pointer to a @ref hal_ccb_ecc_point_t structure
  * @param  p_hash                      Pointer to the hash
  * @param  p_signature                 Pointer to a @ref hal_ccb_ecdsa_sign_t structure
  * @retval HAL_ERROR                   Error detected
  * @retval HAL_OK                      Operation is successfully accomplished
  */
static hal_status_t CCB_PKA_ECDSASetConfigVerifSignature(PKA_TypeDef *p_pka_instance,
                                                         const hal_ccb_ecdsa_curve_param_t *p_in_curve_param,
                                                         hal_ccb_ecc_point_t *p_public_key_out, const uint8_t *p_hash,
                                                         hal_ccb_ecdsa_sign_t *p_signature)
{
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;

  if (CCB_PKA_Init(p_pka_instance) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Get the prime order n length */
  pka_ram_u32[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->prime_order_size_byte,
                                                                       *(p_in_curve_param->p_prime_order));

  /* Get the modulus p length */
  pka_ram_u32[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = CCB_GetOptBitSize_u8(p_in_curve_param->modulus_size_byte,
                                                                     *(p_in_curve_param->p_modulus));

  /* Get the coefficient a sign */
  pka_ram_u32[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = p_in_curve_param->coef_sign_a;

  /* Move the input parameters coefficient |a| to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_A_COEFF]), p_in_curve_param->p_abs_coef_a,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters modulus value p to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_MOD_GF]), p_in_curve_param->p_modulus,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters base point G coordinate x to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_INITIAL_POINT_X]), p_in_curve_param->p_point_x,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters base point G coordinate y to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y]), p_in_curve_param->p_point_y,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters public-key curve point Q coordinate xQ to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X]), p_public_key_out->p_point_x,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters public-key curve point Q coordinate xQ to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y]), p_public_key_out->p_point_y,
                       p_in_curve_param->modulus_size_byte);

  /* Move the input parameters signature part r to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_SIGNATURE_R]), p_signature->p_r_sign,
                       p_in_curve_param->prime_order_size_byte);

  /* Move the input parameters signature part s to PKA RAM */
  CCB_Memcpy_u8_to_u32((__IOM uint32_t *) & (pka_ram_u32[PKA_ECDSA_VERIF_IN_SIGNATURE_S]), p_signature->p_s_sign,
                       p_in_curve_param->prime_order_size_byte);

  /* Move the input parameters hash of message z to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_HASH_E]), p_hash, p_in_curve_param->prime_order_size_byte);

  /* Move the input parameters curve prime order n to PKA RAM */
  CCB_Memcpy_u8_to_u32(&(pka_ram_u32[PKA_ECDSA_VERIF_IN_ORDER_N]), p_in_curve_param->p_prime_order,
                       p_in_curve_param->prime_order_size_byte);

  /* set the mode and deactivate the interrupts */
  STM32_MODIFY_REG(p_pka_instance->CR,
                   PKA_CR_MODE | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE | PKA_CR_ADDRERRIE | PKA_CR_OPERRIE,
                   CCB_PKA_ECDSA_VERIFICATION_MODE);

  /* Start the computation */
  LL_PKA_Start(p_pka_instance);

  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_PROCENDF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  STM32_SET_BIT(p_pka_instance->CLRFR,
                PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | PKA_CLRFR_ADDRERRFC | PKA_CLRFR_OPERRFC);

  return HAL_OK;
}

/**
  * @brief  Unwraps a software-wrapped key.
  * @param  p_saes_instance     SAES instance
  * @param  p_unwrapkey_context   Pointer to a context structure containing key and algorithm information
  * @retval HAL_OK              Operation completed successfully
  * @retval HAL_ERROR           Error detected during the unwrapping process
  */
static hal_status_t CCB_SAES_SW_UnwrapKey(AES_TypeDef *p_saes_instance, const void *p_unwrapkey_context)
{
  uint32_t tickstart = HAL_GetTick();
  const ccb_sw_unwrap_key_context_t *ctx = (const ccb_sw_unwrap_key_context_t *)p_unwrapkey_context;
  const uint32_t *p_iv = (const uint32_t *)ctx->p_wrapping_key_context->p_init_vect;

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ctx->p_wrapping_key_context->aes_algorithm != HAL_CCB_AES_ECB)
  {
    STM32_WRITE_REG(p_saes_instance->CR, AES_CR_KEYSEL_0 | AES_CR_KMOD_0 | AES_CR_KEYSIZE
                    | AES_CR_CHMOD_0);
  }
  else
  {
    STM32_WRITE_REG(p_saes_instance->CR, AES_CR_KEYSEL_0 | AES_CR_KMOD_0 | AES_CR_KEYSIZE);
  }

  /* Wait for Key valid to be 1U */
  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_KEYVALID, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Disable the SAES peripheral */
  STM32_CLEAR_BIT(p_saes_instance->CR, AES_CR_EN);

  /* wait for key valid */
  while ((STM32_READ_REG(p_saes_instance->SR) & AES_SR_KEYVALID) == 0U)
  {
    if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
    {
      if ((STM32_READ_REG(p_saes_instance->SR) & AES_SR_KEYVALID) == 0U)
      {
        STM32_CLEAR_BIT(p_saes_instance->CR, AES_CR_EN);
        return HAL_ERROR;
      }
    }
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* set the operating mode and key preparation for decryption, operating mode*/
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_KMOD | AES_CR_MODE,
                   AES_CR_KMOD_0 | AES_CR_MODE_0);

  STM32_SET_BIT(p_saes_instance->CR, AES_CR_EN);

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Return to decryption operating mode(Mode 3)*/
  STM32_MODIFY_REG(p_saes_instance->CR, AES_CR_MODE, AES_CR_MODE_1);

  if (ctx->p_wrapping_key_context->aes_algorithm != HAL_CCB_AES_ECB)
  {
    STM32_WRITE_REG(p_saes_instance->IVR3, p_iv[0]);
    STM32_WRITE_REG(p_saes_instance->IVR2, p_iv[1]);
    STM32_WRITE_REG(p_saes_instance->IVR1, p_iv[2]);
    STM32_WRITE_REG(p_saes_instance->IVR0, p_iv[3]);
  }

  STM32_SET_BIT(p_saes_instance->CR, AES_CR_EN);

  /*
  * Feed the wrapped software key (256-bit) to SAES.
  * ctx->p_in_wrapped_user_key contains 8 words (2 blocks of 128-bit).
  */
  for (uint32_t in_count = 0U; in_count < 8UL; in_count += 4UL)
  {
    for (uint32_t i = 0U; i < 4U; ++i)
    {
      STM32_WRITE_REG(p_saes_instance->DINR, ctx->p_in_wrapped_user_key[in_count + i]);
    }
    /* wait  CCF in SAES */
    if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  /* Disable the SAES peripheral */
  STM32_CLEAR_BIT(p_saes_instance->CR, AES_CR_EN);
  STM32_SET_BIT(p_saes_instance->ICR, 0x0EUL);

  return HAL_OK;
}

/**
  * @brief  Generates a buffer of random bytes.
  * @param  p_rng_instance  RNG instance
  * @param  p_randoms       Pointer to store randoms.
  * @retval HAL_OK          Operation completed successfully
  * @retval HAL_ERROR       Error detected
  */
static hal_status_t CCB_RNG_GenerateRandomNumbers(RNG_TypeDef *p_rng_instance, uint16_t *p_randoms)
{
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    uint32_t tickstart = HAL_GetTick();
    while (p_randoms[i] == 0U)
    {
      p_randoms[i] = (uint16_t)(STM32_READ_REG(p_rng_instance->DR) & 0x3FFU);
      if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
      {
        if (p_randoms[i] == 0U)
        {
          return HAL_ERROR;
        }
      }
    }
    if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

/**
  * @brief  Generates a buffer of random bytes.
  * @param  p_rng_instance  RNG instance
  * @param  p_hash          Pointer to store the first random value.
  * @param  hash_size       Specify the size of the hash message
  * @retval HAL_OK          Operation completed successfully
  * @retval HAL_ERROR       Error detected
  */
static hal_status_t CCB_RNG_GenerateHashMessage(RNG_TypeDef *p_rng_instance, uint8_t *p_hash, uint32_t hash_size)
{
  for (uint32_t count = 0U; count < hash_size; count++)
  {
    /* Ensure RNG has fresh data ready */
    if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
    {
      return HAL_ERROR;
    }

    uint32_t tickstart = HAL_GetTick();
    uint8_t value = 0U;

    while (value == 0U)
    {
      value = (uint8_t)(STM32_READ_REG(p_rng_instance->DR) & 0xFFU);
      if ((HAL_GetTick() - tickstart) > CCB_GENERAL_TIMEOUT_MS)
      {
        if (value == 0U)
        {
          return HAL_ERROR;
        }
      }
    }
    p_hash[count] = value;
  }
  return HAL_OK;
}

/**
  * @brief  Software reset the CCB peripheral.
  * @param  hccb            Pointer to a @ref hal_ccb_handle_t structure
  */
static void CCB_RESET(const hal_ccb_handle_t *hccb)
{
  CCB_TypeDef *p_ccb_instance = CCB_GET_INSTANCE(hccb);

  STM32_SET_BIT(p_ccb_instance->CR, CCB_CR_IPRST);

  (void) CCB_WaitFLAG(p_ccb_instance, CCB_SR_CCB_BUSY);

  STM32_CLEAR_BIT(p_ccb_instance->CR, CCB_CR_IPRST);
}

/**
  * @brief  Pad the end of a buffer with two zero words.
  * @param  p_pka_ram   Pointer to the PKA RAM memory
  * @param  index       Starting index at which to pad zeros
  */
static inline void CCB_PKA_PadEndRam(volatile uint32_t *p_pka_ram, uint32_t index)
{
  p_pka_ram[index] = 0UL;
  p_pka_ram[index + 1U] = 0UL;
}

/**
  * @brief  Get SAES flag.
  * @param  p_saes_instance  SAES instance
  * @param  flag             Specifies the flag to check
  * @retval 1U               Flag is set
  * @retval 0U               Flag is not set
  */
static inline uint32_t CCB_SAES_GetFlag(AES_TypeDef const *p_saes_instance, uint32_t flag)
{
  uint32_t register_value = (flag == (uint32_t)AES_ISR_CCF) ?
                            p_saes_instance->ISR : p_saes_instance->SR;

  return (STM32_IS_BIT_SET(STM32_READ_REG(register_value), flag)) ? 1U : 0U;
}


/**
  * @brief  Write clear-text data into PKA RAM.
  * @param  p_pka_ram             Pointer to the PKA RAM memory
  * @param  dst_address           Destination address
  * @param  p_src                 Pointer to the source buffer
  * @param  modulus_size_byte     Modulus size in bytes
  * @param  operand_size          Operand size in bytes
  */
static inline void CCB_PKA_WriteClearTextData(volatile uint32_t *p_pka_ram, uint16_t dst_address, const uint8_t *p_src,
                                              uint32_t modulus_size_byte, uint32_t operand_size)
{
  const uint32_t remainder_bytes = (modulus_size_byte) & 7UL;
  const uint32_t max_word_offset = (remainder_bytes != 0U) ? ((operand_size) - 4UL) : ((operand_size) - 2UL);
  uint32_t word_offset;

  for (word_offset = 0U; word_offset < max_word_offset; word_offset += CCB_WORDS_PER_BLOCK)
  {
    const uint32_t src_index = modulus_size_byte - ((word_offset * CCB_BYTES_PER_WORD) + 1U);
    CCB_Memcpy_u8_to_u64(&p_pka_ram[dst_address + word_offset], &p_src[src_index]);
  }

  if (remainder_bytes != 0U)
  {
    const uint32_t src_index = modulus_size_byte - ((word_offset * CCB_BYTES_PER_WORD) + 1U);
    CCB_Memcpy_TailBytesToU64(&p_pka_ram[dst_address + word_offset], &p_src[src_index],
                              remainder_bytes);

    word_offset += CCB_WORDS_PER_BLOCK;
  }

  CCB_PKA_PadEndRam(p_pka_ram, dst_address + word_offset);
}
/**
  * @brief  Write wrapped key into SAES data register.
  * @param  dst_address       Destination address
  * @param  p_wrapped_key     Pointer to wrapped key
  * @param  size_byte         Operand size in bytes
  * @param  randoms           Random Number
  * @retval HAL_OK            Operation is successfully accomplished
  * @retval HAL_ERROR         Error detected during the process
  *
  */
static inline hal_status_t CCB_WriteWrappedKey(uint16_t dst_address, const uint32_t *p_wrapped_key, uint32_t size_byte,
                                               uint32_t randoms)
{
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)PKA->RAM;

  /* Convert the key size in bytes into the number of 32-bit words required by the PKA operand format.*/
  uint32_t operand_size   = 2UL * ((uint32_t)((size_byte + 7UL) >> 3UL) + 1UL);

  /* Derive the number of 32-bit words of actual cipher key to use */
  const uint32_t cipherkey_size = ((operand_size & 3U) != 0U) ? (operand_size - 2U) : operand_size;
  const uint32_t has_remainder  = (operand_size & 3U);
  uint32_t count_block = 0UL;

  for (uint32_t word_offset = 0UL; word_offset < cipherkey_size; word_offset++)
  {
    /* Write encrypted phi in DINR of SAES */
    STM32_WRITE_REG(p_saes_instance->DINR, (p_wrapped_key[cipherkey_size - (word_offset + 1UL)] ^ randoms));

    if ((word_offset & 3UL) == 0x3UL)
    {
      /* Wait until CCF flag is Set in SAES */
      if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Write key in PKA RAM */
      for (uint32_t count_ram = 0U; count_ram < 4U; count_ram++)
      {
        pka_ram_u32[dst_address + count_block + count_ram] = CCB_MAGIC_VALUE;

      }
      count_block += 4UL;
    }
  }

  if (has_remainder != 0UL)
  {
    CCB_PKA_PadEndRam(pka_ram_u32, (dst_address + cipherkey_size));
  }

  return HAL_OK;
}

/**
  * @brief  Write CCB key from RNG.
  * @param  dst_address     Destination address
  * @param  operand_size    Operand size in bytes
  * @retval HAL_OK          Operation completed successfully
  * @retval HAL_ERROR       Error detected
  */
static inline hal_status_t CCB_WriteKeyFromRNG(uint16_t dst_address, uint32_t operand_size)
{
  PKA_TypeDef *p_pka_instance = PKA;
  RNG_TypeDef *p_rng_instance = RNG;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;
  uint32_t word_offset;

  for (word_offset = 0UL; word_offset < (operand_size - 2UL); word_offset++)
  {
    if (CCB_RNG_WaitFlag(p_rng_instance, RNG_SR_DRDY) != HAL_OK)
    {
      return HAL_ERROR;
    }
    pka_ram_u32[(dst_address + word_offset)] = CCB_MAGIC_VALUE;
  }
  /* Check RNG Error Flag in PKA */
  if (CCB_GET_PKA_FLAG(p_pka_instance, PKA_SR_RNGERRF) != 0U)
  {
    return HAL_ERROR;
  }
  CCB_PKA_PadEndRam(pka_ram_u32, (dst_address + word_offset));

  /* Wait until RNGOKF flag is 1U in PKA and trig OPSTEP transition 0x6 --> 0x8 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_RNGOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Read ciphered private key.
  * @param  dst_address      Destination address
  * @param  operand_size     Operand size in bytes
  * @param  p_wrapped_key    Pointer to wrapped key
  * @param  randoms          Random Number
  * @retval HAL_OK           Operation completed successfully
  * @retval HAL_ERROR        Error detected
  */
static inline hal_status_t CCB_ReadCipheredPrivateKey(uint16_t dst_address, uint32_t operand_size,
                                                      uint32_t *p_wrapped_key, uint32_t randoms)
{
  PKA_TypeDef *p_pka_instance = PKA;
  AES_TypeDef *p_saes_instance = SAES;
  __IOM uint32_t *pka_ram_u32 = (__IOM uint32_t *)p_pka_instance->RAM;
  const uint32_t has_remainder  = (operand_size & 3U);
  const uint32_t cipherkey_size = (has_remainder != 0U) ? (operand_size - 2U) : operand_size;
  uint32_t count_block = 0UL;

  for (uint32_t word_offset = 0U; word_offset < cipherkey_size; word_offset++)
  {
    pka_ram_u32[dst_address + word_offset] = CCB_MAGIC_VALUE;
    if ((word_offset & 3UL) == 3UL)
    {
      /* Wait until CCF flag is 1U in SAES*/
      if (CCB_SAES_WaitFlag(p_saes_instance, AES_ISR_CCF, 1U) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read ciphered private Key*/
      for (uint16_t count = 0U; count < CCB_BLOCK_SIZE_WORD ; count++)
      {
        p_wrapped_key[cipherkey_size - (count_block + count + 1UL)]
          = (STM32_READ_REG(p_saes_instance->DOUTR) ^ randoms);
      }
      count_block += 4UL;
    }
  }

  if (CCB_SAES_WaitFlag(p_saes_instance, AES_SR_BUSY, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (has_remainder != 0UL)
  {
    CCB_PKA_PadEndRam(pka_ram_u32, (dst_address + cipherkey_size));
  }

  /* Wait until DATAOKF flag is 1U in PKA and trig OPSTEP transition 0x08 --> 0x09 */
  if (CCB_PKA_WaitFlag(p_pka_instance, PKA_SR_DATAOKF) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * @brief  Erase Pool Buffer.
  * @param  p_buff           Pool buffer
  * @param  buff_size_byte   Pool buffer size in byte
  */
static void CCB_ErasePoolBuffer(uint8_t *p_buff, uint32_t buff_size_byte)
{
  volatile uint32_t *p_buffer_32 = (volatile uint32_t *)p_buff;
  uint32_t words = buff_size_byte / 4U;
  uint32_t tail = buff_size_byte & 0x3U;

  for (uint32_t i = 0U; i < words; i++)
  {
    p_buffer_32[i] = 0U;
  }

  for (uint32_t i = 0U; i < tail; i++)
  {
    p_buff[(words * 4U) + i] = 0U;
  }
}

#if defined (USE_HAL_CCB_RNG_RECOVERY) && (USE_HAL_CCB_RNG_RECOVERY == 1U)
#if defined(RNG_HTSR0_RPERRX) || defined(RNG_HTSR1_ADERRX)
/**
  * @brief  RNG sequence to resilient recover from a seed error.
  * @retval HAL status
  */
hal_status_t CCB_RNG_ResilientRecoverSeedError(void)
{
  RNG_TypeDef *p_rng_instance = RNG;

  uint32_t timeout;
  uint32_t htsr_temp = 0U;
  uint32_t htsr_previous_temp = 0U;
  uint32_t htsr_count = 0U;
  uint32_t nsmr_temp = 0U;
  uint32_t tickstart1 = 0U;
  uint32_t tickstart2 = 0U;
  uint32_t tickstart3 = 0U;
  uint32_t oscillators_count = 0U;
  uint32_t config_b_fewer_than_6_osc_count = 0U;
  uint8_t count = 0U;

  /* timeout here is an emperic value */
  timeout = (1UL + ((1UL << (STM32_READ_BIT(p_rng_instance->CR, RNG_CR_CLKDIV) >> 16UL))
                    * CCB_RNG_TIMEOUT_VALUE / 8UL));
  LL_RNG_Enable(RNG);

  tickstart1 = HAL_GetTick();

  /* Check if seed error current status indicates no error and auto-reset succeeded */
  if (LL_RNG_IsActiveFlag_SECS(RNG) == 0U)
  {
    /* Clear SEIS flag when automatic reset is activated */
    LL_RNG_ClearFlag_SEIS(RNG);
  }

  else  /* Sequence to fully recover from a seed error*/
  {
    if (LL_RNG_IsConfigLocked(RNG) == 0U)
    {
      do
      {
        if (LL_RNG_IsActiveFlag_SECS(RNG) == 0U)
        {
          break;
        }
        /* Read oscillator status registers combined */
        htsr_temp = LL_RNG_GetHealthTestStatus(RNG, 0U);
        htsr_temp |= LL_RNG_GetHealthTestStatus(RNG, 1U);
        if (htsr_temp > 0U)
        {
          /* If any oscillator status bits overlap with previous status, increment counter */
          if ((htsr_temp & htsr_previous_temp) != 0U)
          {
            htsr_count++;
          }

          if (htsr_count > 3U)
          {
            /* if the same repetitive or adaptative error is detected 3 times */
            nsmr_temp = LL_RNG_GetNoiseSourceMask(RNG);

            /* deactivate the same osc in each triple oscillator (Mask oscillators with the seed error by
            clearing bits shifted right by 1) */
            nsmr_temp = nsmr_temp & ~(htsr_temp >> 1U);

            /* Count the number of active oscillators in nsmr */
            oscillators_count = 0U;
            for (count = 0U; count < 9U; count++)
            {
              if (((nsmr_temp >> count) & 0x1U) != 0U)
              {
                /* increment count1 for each 1 in nsmr */
                oscillators_count++;
              }
            }

            if (oscillators_count < 6U)
            {
              /* If fewer than 6 oscillators remain active, unmask all oscillators --> Reset masking */
              nsmr_temp = LL_RNG_GetOscNoiseSrc(RNG, LL_RNG_NOISE_SRC_1 | LL_RNG_NOISE_SRC_2 | LL_RNG_NOISE_SRC_3);
              htsr_previous_temp = 0;
              htsr_count = 0U;
              if ((RNG->CR  & RNG_CR_CLKDIV_Msk) < ((uint32_t)RNG_CAND_NIST_CR_VALUE & RNG_CR_CLKDIV_Msk))
              {
                config_b_fewer_than_6_osc_count++;
              }
            }

            if (config_b_fewer_than_6_osc_count > 2U)
            {
              /* Reset RNG condition */
              STM32_WRITE_REG(p_rng_instance->CR, (RNG_CR_CONDRST_Msk | (uint32_t)RNG_CAND_NIST_CR_VALUE));

              /* Update mask register with new oscillator mask */
              LL_RNG_SetNoiseSourceMask(RNG, nsmr_temp);

              /* Clear condition reset bit to resume operation */
              LL_RNG_DisableCondReset(RNG);
            }

            else
            {
              /* Reset RNG condition */
              STM32_WRITE_REG(p_rng_instance->CR,
                              (p_rng_instance->CR & ~RNG_CR_RNGEN_Msk) | RNG_CR_CONDRST_Msk);

              /* Update mask register with new oscillator mask */
              LL_RNG_SetNoiseSourceMask(RNG, nsmr_temp);

              /* Clear condition reset bit to resume operation */
              LL_RNG_DisableCondReset(RNG);
            }
          }

          else
          {
            /* Briefly toggle conditional reset to recover RNG */
            STM32_WRITE_REG(p_rng_instance->CR, (p_rng_instance->CR & ~RNG_CR_RNGEN_Msk) | RNG_CR_CONDRST_Msk);

            /* unmask all oscillators to find another working condition */
            LL_RNG_SetNoiseSourceMask(RNG, LL_RNG_GetOscNoiseSrc(RNG, LL_RNG_OSC_1 | LL_RNG_OSC_2 | LL_RNG_OSC_3));
            LL_RNG_DisableCondReset(RNG);
          }

          /* Wait until RNG is not busy */
          tickstart2 = HAL_GetTick();
          do
          {
            if ((HAL_GetTick() - tickstart2) > CCB_RNG_TIMEOUT_VALUE)
            {
              /* New check to avoid false timeout detection in case of preemption */
              LL_RNG_Disable(RNG);
              return HAL_ERROR;
            }
          } while (STM32_IS_BIT_SET(p_rng_instance->CR, RNG_SR_BUSY));

          /* No timeout --> Enable RNG */
          LL_RNG_Enable(RNG);
          tickstart3 = HAL_GetTick();
          do
          {
            if (LL_RNG_IsActiveFlag_DRDY(RNG) != 0UL)
            {
              break;
            }
            if ((HAL_GetTick() - tickstart3) > timeout)
            {
              /* New check to avoid false timeout detection in case of preemption */
              if (LL_RNG_IsActiveFlag_DRDY(RNG) == 0UL)
              {
                if (LL_RNG_IsActiveFlag_SECS(RNG) == 0UL)
                {
                  LL_RNG_Disable(RNG);
                  return HAL_ERROR;
                }
              }
            }
          } while (LL_RNG_IsActiveFlag_SECS(RNG) == 0UL);

          /* Accumulate seed error status bits */
          htsr_previous_temp = htsr_previous_temp | htsr_temp;
        }
      } while ((HAL_GetTick() - tickstart1) <= timeout);
    }
  }

  /*Check if seed error current status (SECS)is set */
  if (LL_RNG_IsActiveFlag_SECS(RNG) != 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* RNG_HTSR0_RPERRX || RNG_HTSR1_ADERRX */
#endif /* USE_HAL_CCB_RNG_RECOVERY */
/**
  * @}
  */

#endif /* USE_HAL_CCB_MODULE */
/**
  * @}
  */
#endif /* CCB */
/**
  * @}
  */
