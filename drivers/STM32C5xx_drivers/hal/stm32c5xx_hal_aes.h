/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_aes.h
  * @brief   Header file of AES HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_HAL_AES_H
#define STM32C5XX_HAL_AES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_rng.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(AES) || defined(SAES)
#if (defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)) \
    || (defined(USE_HAL_AES_CTR_ALGO) && (USE_HAL_AES_CTR_ALGO == 1)) \
    || (defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1))\
    || (defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1))
/** @defgroup AES AES
  * @{
  */

/* Exported constants ---------------------------------------------------------*/
/** @defgroup AES_Exported_Constants HAL AES Constants
  * @{
  */

#if defined(USE_HAL_AES_GET_LAST_ERRORS) && (USE_HAL_AES_GET_LAST_ERRORS == 1)

/** @defgroup  AES_Error_Code Error Code definition reflecting the process asynchronous errors
  * @{
  */
#define  HAL_AES_ERROR_NONE   (0UL)             /*!< No error    */
#define  HAL_AES_ERROR_KEY    AES_ISR_KEIF      /*!< Key error   */
#if defined(SAES)
#define  HAL_AES_ERROR_RNG    AES_ISR_RNGEIF    /*!< RNG error   */
#endif /* SAES */
#if defined(USE_HAL_AES_DMA) && (USE_HAL_AES_DMA == 1)
#define  HAL_AES_ERROR_DMA    (0x01UL << 4U)    /*!< DMA error   */
#endif /* USE_HAL_AES_DMA */
#define  HAL_AES_ERROR_READ   AES_SR_RDERRF      /*!< Read error  */
#define  HAL_AES_ERROR_WRITE  AES_SR_WRERRF      /*!< Write error */

/**
  * @}
  */

#endif /* USE_HAL_AES_GET_LAST_ERRORS */

/*! AES flag  definition */
#define HAL_AES_FLAG_BUSY     AES_SR_BUSY                 /*!< Process suspension forbidden
                                                               also set when transferring a shared key
                                                               from SAES peripheral
                                                               */
#define HAL_AES_FLAG_WRERR    (AES_SR_WRERRF | 0x80000000U) /*!< Write Error flag */
#define HAL_AES_FLAG_RDERR    (AES_SR_RDERRF | 0x80000000U) /*!< Read error  flag */
#define HAL_AES_FLAG_KEYVALID AES_SR_KEYVALID              /*!< Key valid flag */
#define HAL_AES_FLAG_CC       AES_ISR_CCF                  /*!< Computation completed flag */
#define HAL_AES_FLAG_KERR     AES_ISR_KEIF                 /*!< Key error interrupt flag */
#define HAL_AES_FLAG_RDWRERR  AES_ISR_RWEIF                /*!< Read or write error interrupt flag */
#if defined(SAES)
#define HAL_AES_FLAG_RNGERR   AES_ISR_RNGEIF               /*!< RNG error interrupt flag */
#endif /* SAES */

/*! AES key interrupts definition */
#define HAL_AES_IT_CC       AES_IER_CCFIE  /*!< Computation Complete interrupt enable */
#define HAL_AES_IT_RDWRERR  AES_IER_RWEIE  /*!< Read or write Error interrupt enable  */
#define HAL_AES_IT_KERR     AES_IER_KEIE   /*!< Key error interrupt enable */
#define HAL_AES_IT_ALL (HAL_AES_IT_CC | HAL_AES_IT_RDWRERR | HAL_AES_IT_KERR) /*!< AES Interrupt Enable */
#if defined(SAES)
#define HAL_AES_IT_RNGERR   AES_IER_RNGEIE /*!< RNG error interrupt enable */
#endif /* SAES */
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup  AES_Exported_Types HAL AES Types
  * @{
  */

/*! AES instance enumeration definition */
typedef enum
{
  HAL_AES = AES_BASE, /*!< AES  instance */
#if defined(SAES)
  HAL_SAES = SAES_BASE /*!< SAES instance */
#endif /* SAES */
} hal_aes_t;

/*! AES Global state enumeration definition */
typedef enum
{
  HAL_AES_STATE_RESET     = 0UL,          /*!< AES peripheral is not yet initialized */
  HAL_AES_STATE_INIT      = (1UL << 31U), /*!< AES peripheral is initialized but not yet configured */
  HAL_AES_STATE_IDLE      = (1UL << 30U), /*!< AES peripheral is initialized and configured */
  HAL_AES_STATE_ACTIVE    = (1UL << 29U), /*!< AES internal processing is ongoing */
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
  HAL_AES_STATE_SUSPENDED = (1UL << 28U)  /*!< AES internal processing is suspended */
#endif /* USE_HAL_AES_SUSPEND_RESUME */
} hal_aes_state_t;

/*! AES key size enumeration definition */
typedef enum
{
  HAL_AES_KEY_SIZE_128BIT = 0x00000000U,   /*!< 128-bit long key */
  HAL_AES_KEY_SIZE_256BIT = AES_CR_KEYSIZE /*!< 256-bit long key */
} hal_aes_key_size_t;

#if defined(SAES)
/*! AES key select enumeration definition */
typedef enum
{
  HAL_AES_KEY_SELECT_DHUK         = AES_CR_KEYSEL_0, /*!< Only for SAES, hardware key: Derived hardware unique key
                                                       (DHUK 256-bit) */
  HAL_AES_KEY_SELECT_BHK          = AES_CR_KEYSEL_1, /*!< Only for SAES, software key:Boot hardware key BHK (256-bit) */
  HAL_AES_KEY_SELECT_DHUK_XOR_BHK = AES_CR_KEYSEL_2  /*!< Only for SAES, hardware unique key XOR software key */
} hal_aes_key_select_t;


/*! AES key mode enumeration definition */
typedef enum
{
  HAL_AES_KEY_MODE_NORMAL  = 0x00000000U,   /*!< Use HW key to do encrypt/decrypt in normal key mode */
  HAL_AES_KEY_MODE_WRAPPED = AES_CR_KMOD_0, /*!< Use HW key to do encrypt/decrypt in wrap key mode   */
  HAL_AES_KEY_MODE_SHARED  = AES_CR_KMOD_1  /*!< Use HW key to do encrypt/decrypt in share key mode  */
} hal_aes_key_mode_t;
#endif /* SAES */

/*! AES data swapping enumeration definition */
typedef enum
{
  HAL_AES_DATA_SWAPPING_NO       = 0x00000000U,       /*!< No swapping        */
  HAL_AES_DATA_SWAPPING_HALFWORD = AES_CR_DATATYPE_0, /*!< Half-word swapping */
  HAL_AES_DATA_SWAPPING_BYTE     = AES_CR_DATATYPE_1, /*!< Byte swapping      */
  HAL_AES_DATA_SWAPPING_BIT      = AES_CR_DATATYPE    /*!< Bit swapping       */
} hal_aes_data_swapping_t;

#if defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)
/*! AES GCM_GMAC configuration structure */
typedef struct
{
  uint32_t *p_init_vect;      /*!< The initialization vector */

  uint32_t *p_header;         /*!< Used only in AES GCM and CCM Algorithm for authentication,
                                   For GCM: The header is also known as Additional Authentication Data */
  uint32_t header_size_byte ; /*!< The size of header buffer in bytes */

} hal_aes_gcm_config_t;
#endif /* USE_HAL_AES_GCM_GMAC_ALGO or USE_HAL_AES_CCM_ALGO */

#if defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1)
/*! AES CCM configuration structure */
typedef struct
{
  uint32_t *p_b0;             /*!< B0 is the first authentication block used only in AES CCM mode, composed of 16 bytes */

  uint32_t *p_header;         /*!< Used only in AES GCM and CCM Algorithm for authentication,
                                   For CCM: Named B1 composed of the associated data length and Associated Data. */

  uint32_t header_size_byte ; /*!< The size of header buffer in bytes */

} hal_aes_ccm_config_t;
#endif /* USE_HAL_AES_GCM_GMAC_ALGO or USE_HAL_AES_CCM_ALGO */

typedef struct hal_aes_handle_s hal_aes_handle_t; /*!< AES Handle type Definition */

#if defined(USE_HAL_AES_REGISTER_CALLBACKS) && (USE_HAL_AES_REGISTER_CALLBACKS == 1)
/*! AES callback type definition */
typedef void (*hal_aes_cb_t)(hal_aes_handle_t *haes);
#endif /* USE_HAL_AES_REGISTER_CALLBACKS */

/*! AES handle Structure definition */
struct hal_aes_handle_s
{
  hal_aes_t                 instance;           /*!< AES Register base address, can be a value of @ref hal_aes_t */

  volatile  hal_aes_state_t global_state;       /*!< AES peripheral state, can be a value of @ref hal_aes_state_t */

  uint32_t                  algorithm;          /*!< AES chaining mode */

  volatile uint32_t         data_size_byte;     /*!< Length of input data in byte */

  volatile uint32_t         data_size_sum_byte; /*!< Sum of successive payloads lengths (in bytes), stored for a single
                                                     signature computation after several messages processing */

  const uint32_t            *p_in_buff;         /*!< Pointer to AES input processing buffer(plaintext or ciphertext) */

  uint32_t                  *p_out_buff;        /*!< Pointer to AES output processing buffer (Allowing to store
                                                     encrypted or decrypted text) */

  volatile uint32_t         block_count;        /*!< Counter of input data blocks, one block is equal to 128 bits */

#if (defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)) \
     || (defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1))
  const uint32_t            *p_header;          /*!< Used only in AES GCM and CCM Algorithm for authentication,
                                                     GCM: Also known as Additional Authentication Data
                                                     CCM: Named B1 composed of the associated data length and
                                                     Associated Data. */

  uint32_t                  header_size_byte ; /*!< The size of header buffer in bytes */
#endif /* USE_HAL_AES_GCM_GMAC_ALGO or USE_HAL_AES_CCM_ALGO */

#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
  volatile uint32_t         suspend_request; /*!< AES peripheral suspension request flag */

  const uint32_t            *p_key;          /*!< Application pointer key to be stored in the handle during
                                                  suspension */
#endif /* USE_HAL_AES_SUSPEND_RESUME */

#if defined(USE_HAL_AES_DMA) && (USE_HAL_AES_DMA == 1)
  hal_dma_handle_t          *hdma_in;  /*!< AES In DMA handle parameters */
  hal_dma_handle_t          *hdma_out; /*!< AES Out DMA handle parameters */
#endif /* USE_HAL_AES_DMA */

#if defined(USE_HAL_AES_USER_DATA) && (USE_HAL_AES_USER_DATA == 1)
  const void                *p_user_data;      /*!< User Data Pointer */
#endif /* (USE_HAL_AES_USER_DATA) */

#if defined(USE_HAL_AES_REGISTER_CALLBACKS) && (USE_HAL_AES_REGISTER_CALLBACKS == 1)
  hal_aes_cb_t               p_in_cplt_cb;      /*!< AES Input FIFO transfer completed callback  */
  hal_aes_cb_t               p_out_cplt_cb;     /*!< AES Output FIFO transfer completed callback */
  hal_aes_cb_t               p_error_cb;        /*!< AES Error callback */
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
  hal_aes_cb_t               p_suspend_cb;      /*!< AES Suspend callback */
#endif /* USE_HAL_AES_SUSPEND_RESUME */
#endif /* (USE_HAL_AES_REGISTER_CALLBACKS) */

#if defined(USE_HAL_AES_GET_LAST_ERRORS) && (USE_HAL_AES_GET_LAST_ERRORS == 1)
  /* in case of single process at a time: one single variable storing the last errors */
  volatile uint32_t          last_error_codes;  /*!< AES peripheral error code */
#endif /* USE_HAL_AES_GET_LAST_ERRORS */
};

/*! AES suspend resume Structure definition */
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
/*! AES suspend resume configuration structure */
typedef struct
{
  uint32_t                  CR;                 /*!< Copy of AES control register when processing is suspended */

  uint32_t                  iv_buff[4];         /*!< Copy of Initialization Vector registers */

#if (defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)) \
     || (defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1))
  uint32_t                  SUSPRx[8];        /*!< Copy of suspension registers, used only in AES GCM and CCM
                                                   algorithms */
#endif /* USE_HAL_AES_GCM_GMAC_ALGO or USE_HAL_AES_CCM_ALGO */

  hal_aes_t                 instance;           /*!< AES Register base address, can be a value of @ref hal_aes_t */

  volatile  hal_aes_state_t previous_state;       /*!< AES peripheral state, can be a value of @ref hal_aes_state_t */

  uint32_t                  algorithm;          /*!< AES chaining mode */

  volatile uint32_t         data_size_byte;     /*!< Length of input data in byte */

  volatile uint32_t         data_size_sum_byte; /*!< Sum of successive payloads lengths (in bytes), stored for a single
                                                     signature computation after several messages processing */

  const uint32_t            *p_in_buff;         /*!< Pointer to AES input processing buffer(plaintext or ciphertext) */

  uint32_t                  *p_out_buff;        /*!< Pointer to AES output processing buffer (Allowing to store
                                                     encrypted or decrypted text) */

  volatile uint32_t         block_count;        /*!< Counter of input data blocks, one block is equal to 128 bits */

#if (defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)) \
     || (defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1))
  const uint32_t            *p_header;          /*!< Used only in AES GCM and CCM Algorithm for authentication,
                                                     GCM: Also known as Additional Authentication Data
                                                     CCM: Named B1 composed of the associated data length and
                                                           Associated Data. */

  uint32_t                  header_size_byte ; /*!< The size of header buffer in bytes */
#endif /* USE_HAL_AES_GCM_GMAC_ALGO or USE_HAL_AES_CCM_ALGO */

#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
  volatile uint32_t         suspend_request;   /*!< AES peripheral suspension request flag */

  const uint32_t            *p_key;           /*!< Application pointer key to be stored in the handle during
                                                   suspension */
#endif /* USE_HAL_AES_SUSPEND_RESUME */

#if defined(USE_HAL_AES_REGISTER_CALLBACKS) && (USE_HAL_AES_REGISTER_CALLBACKS == 1)
  hal_aes_cb_t               p_in_cplt_cb;      /*!< AES Input FIFO transfer completed callback  */
  hal_aes_cb_t               p_out_cplt_cb;     /*!< AES Output FIFO transfer completed callback */
  hal_aes_cb_t               p_error_cb;        /*!< AES Error callback */
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
  hal_aes_cb_t               p_suspend_cb;      /*!< AES Suspend callback */
#endif /* USE_HAL_AES_SUSPEND_RESUME */
#endif /* (USE_HAL_AES_REGISTER_CALLBACKS) */

} hal_aes_save_context_t;
#endif /* USE_HAL_AES_SUSPEND_RESUME */

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup AES_Exported_Functions HAL AES Functions
  * @{
  */

/** @defgroup AES_Exported_Functions_Group1 Initialization and De-initialization functions
  * @{
  */
hal_status_t HAL_AES_Init(hal_aes_handle_t *haes, hal_aes_t instance);
void HAL_AES_DeInit(hal_aes_handle_t *haes);
/**
  * @}
  */

/** @defgroup AES_Exported_Functions_Group2 Configuration functions
  * @{
  */
#if defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)
hal_status_t HAL_AES_ECB_SetConfig(hal_aes_handle_t *haes);
hal_status_t HAL_AES_CBC_SetConfig(hal_aes_handle_t *haes, const uint32_t *p_init_vect);
#endif /* USE_HAL_AES_ECB_CBC_ALGO */

#if defined(USE_HAL_AES_CTR_ALGO) && (USE_HAL_AES_CTR_ALGO == 1)
hal_status_t HAL_AES_CTR_SetConfig(hal_aes_handle_t *haes, const uint32_t *p_init_vect);
#endif /* USE_HAL_AES_CTR_ALGO */

#if defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)
hal_status_t HAL_AES_GCM_GMAC_SetConfig(hal_aes_handle_t *haes, const hal_aes_gcm_config_t *p_config);
#endif /* USE_HAL_AES_GCM_GMAC_ALGO */

#if defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1)
hal_status_t HAL_AES_CCM_SetConfig(hal_aes_handle_t *haes, const hal_aes_ccm_config_t *p_config);
#endif /* USE_HAL_AES_CCM_ALGO */

hal_status_t HAL_AES_SetNormalKey(hal_aes_handle_t *haes, hal_aes_key_size_t key_size, const uint32_t *p_key);

#if defined(SAES)
hal_status_t HAL_AES_SetSharedKey(hal_aes_handle_t *haes, hal_aes_key_size_t key_size);
hal_status_t HAL_AES_SetHWKey(hal_aes_handle_t *haes, hal_aes_key_size_t key_size, hal_aes_key_select_t key_select,
                              hal_aes_key_mode_t key_mode);
#endif /* SAES */

hal_status_t HAL_AES_SetDataSwapping(hal_aes_handle_t *haes, hal_aes_data_swapping_t data_swapping);
hal_aes_data_swapping_t HAL_AES_GetDataSwapping(const hal_aes_handle_t *haes);
/**
  * @}
  */

/** @defgroup AES_Exported_Functions_Group3 Process functions
  * @{
  */
/* encryption/decryption */
hal_status_t HAL_AES_Encrypt(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output,
                             uint32_t timeout_ms);
hal_status_t HAL_AES_Decrypt(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output,
                             uint32_t timeout_ms);
hal_status_t HAL_AES_Encrypt_IT(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output);
hal_status_t HAL_AES_Decrypt_IT(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output);
#if defined(USE_HAL_AES_DMA) && (USE_HAL_AES_DMA == 1)
hal_status_t HAL_AES_Encrypt_DMA(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output);
hal_status_t HAL_AES_Decrypt_DMA(hal_aes_handle_t *haes, const void *p_input, uint16_t size_byte, void *p_output);
#endif /* USE_HAL_AES_DMA */

#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
hal_status_t HAL_AES_RequestSuspend(hal_aes_handle_t *haes);
hal_status_t HAL_AES_SaveContext(hal_aes_handle_t *haes, hal_aes_save_context_t *p_context);
hal_status_t HAL_AES_RestoreContext(hal_aes_handle_t *haes, const hal_aes_save_context_t *p_context);
hal_status_t HAL_AES_Resume(hal_aes_handle_t *haes);
#endif /* defined (USE_HAL_AES_SUSPEND_RESUME) */
/**
  * @}
  */

/** @defgroup AES_Exported_Functions_Group4 IRQHandler,Callbacks and, Link DMA functions
  * @{
  */
void HAL_AES_IRQHandler(hal_aes_handle_t *haes);

void HAL_AES_InCpltCallback(hal_aes_handle_t *haes);
void HAL_AES_OutCpltCallback(hal_aes_handle_t *haes);
void HAL_AES_ErrorCallback(hal_aes_handle_t *haes);
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
void HAL_AES_SuspendCallback(hal_aes_handle_t *haes);
#endif /* defined (USE_HAL_AES_SUSPEND_RESUME) */

#if defined(USE_HAL_AES_REGISTER_CALLBACKS) && (USE_HAL_AES_REGISTER_CALLBACKS == 1)
hal_status_t HAL_AES_RegisterInTransferCpltCallback(hal_aes_handle_t *haes, hal_aes_cb_t p_callback);
hal_status_t HAL_AES_RegisterOutTransferCpltCallback(hal_aes_handle_t *haes, hal_aes_cb_t p_callback);
hal_status_t HAL_AES_RegisterErrorCallback(hal_aes_handle_t *haes, hal_aes_cb_t p_callback);
#if defined(USE_HAL_AES_SUSPEND_RESUME) && (USE_HAL_AES_SUSPEND_RESUME == 1)
hal_status_t HAL_AES_RegisterSuspendCallback(hal_aes_handle_t *haes, hal_aes_cb_t p_callback);
#endif /* defined (USE_HAL_AES_SUSPEND_RESUME) */
#endif /* (USE_HAL_AES_REGISTER_CALLBACKS) */

#if defined(USE_HAL_AES_DMA) && (USE_HAL_AES_DMA == 1)
hal_status_t HAL_AES_SetInDMA(hal_aes_handle_t *haes, hal_dma_handle_t *hdma_in);
hal_status_t HAL_AES_SetOutDMA(hal_aes_handle_t *haes, hal_dma_handle_t *hdma_out);
#endif /* USE_HAL_AES_DMA */
/**
  * @}
  */

/** @defgroup AES_Exported_Functions_Group5 Peripheral State, Error and Get last IV functions
  * @{
  */
hal_aes_state_t HAL_AES_GetState(const hal_aes_handle_t *haes);
#if defined(USE_HAL_AES_GET_LAST_ERRORS) && (USE_HAL_AES_GET_LAST_ERRORS == 1)
uint32_t HAL_AES_GetLastErrorCodes(const hal_aes_handle_t *haes);
#endif /* USE_HAL_AES_GET_LAST_ERRORS */

#if defined(USE_HAL_AES_USER_DATA) && (USE_HAL_AES_USER_DATA == 1)
void HAL_AES_SetUserData(hal_aes_handle_t *haes, const void *p_user_data);
const void *HAL_AES_GetUserData(const hal_aes_handle_t *haes);
#endif /* (USE_HAL_AES_USER_DATA) */
#if defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)
hal_status_t HAL_AES_CBC_GetLastOutputIV(const hal_aes_handle_t *haes, const uint8_t *p_last_iv, uint8_t last_iv_size);
#endif /* USE_HAL_AES_ECB_CBC_ALGO*/
#if (defined(USE_HAL_AES_CTR_ALGO) && (USE_HAL_AES_CTR_ALGO == 1))
hal_status_t HAL_AES_CTR_GetLastOutputIV(const hal_aes_handle_t *haes, const uint8_t *p_last_iv, uint8_t last_iv_size);
#endif /*USE_HAL_AES_CTR_ALGO */
/**
  * @}
  */

/** @defgroup AES_Exported_Functions_Group6 Tag Generating functions
  * @{
  */

#if defined(USE_HAL_AES_GCM_GMAC_ALGO) && (USE_HAL_AES_GCM_GMAC_ALGO == 1)
hal_status_t HAL_AES_GCM_GenerateAuthTAG(hal_aes_handle_t *haes, uint32_t *p_auth_tag, uint32_t timeout_ms);
#endif /* (USE_HAL_AES_GCM_GMAC_ALGO) */

#if defined(USE_HAL_AES_CCM_ALGO) && (USE_HAL_AES_CCM_ALGO == 1)
hal_status_t HAL_AES_CCM_GenerateAuthTAG(hal_aes_handle_t *haes, uint32_t *p_auth_tag, uint32_t timeout_ms);
#endif /* (USE_HAL_AES_CCM_ALGO) */

/**
  * @}
  */

#if defined(SAES)
/** @defgroup AES_Exported_Functions_Group7 AES Processing Key functions
  * @{
  */
#if defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)
/* Wrap key function */
hal_status_t HAL_AES_WrapKey(hal_aes_handle_t *haes, const uint32_t *p_key_in, hal_aes_key_size_t key_size,
                             uint32_t *p_key_output, uint32_t timeout_ms);
#endif /* USE_HAL_AES_ECB_CBC_ALGO */
#if (defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)) \
    || (defined(USE_HAL_AES_CTR_ALGO) && (USE_HAL_AES_CTR_ALGO == 1))
/* Unwrap key function */
hal_status_t HAL_AES_UnwrapKey(hal_aes_handle_t *haes, const uint32_t *p_key_in, hal_aes_key_size_t key_size,
                               uint32_t timeout_ms);
#endif /* USE_HAL_AES_ECB_CBC_ALGO || USE_HAL_AES_CTR_ALGO */
#if defined(USE_HAL_AES_ECB_CBC_ALGO) && (USE_HAL_AES_ECB_CBC_ALGO == 1)
/* Encrypt and Decrypt Shared key functions */
hal_status_t HAL_AES_EncryptSharedKey(hal_aes_handle_t *haes, const uint32_t *p_key_in, hal_aes_key_size_t key_size,
                                      uint32_t *p_key_output, uint32_t target_id, uint32_t timeout_ms);
hal_status_t HAL_AES_DecryptSharedKey(hal_aes_handle_t *haes, const uint32_t *p_key_in, hal_aes_key_size_t key_size,
                                      uint32_t target_id, uint32_t timeout_ms);
#endif /* USE_HAL_AES_ECB_CBC_ALGO */
/**
  * @}
  */
#endif /* SAES */

/** @defgroup AES_Exported_Functions_Group8 Static Inlines functions
  * @{
This section provides functions allowing to manage AES interrupts and flags:
  - HAL_AES_GetFlag():Allowing to return the state of a flag
  - HAL_AES_ClearFlagRDWRERR():Allowing to clear the read/write error flag
  - HAL_AES_ClearFlagCC():Allowing to clear the computation complete flag
  - HAL_AES_ClearFlagKERR():Allowing to clear the invalid key error flag
#if defined(SAES)
  - HAL_AES_ClearFlagRNGERR():Allowing to clear the AES RNG error flag
#endif

  - HAL_AES_GetITSource():Allowing to return the state of an interrupt
  - HAL_AES_EnableIT():Allowing to enable an AES interrupt
  - HAL_AES_DisableIT():Allowing to disable an AES interrupt
  */
/** @brief  Check whether the specified AES status flag is set or not.
  * @param  haes Specifies the AES handle
  * @param  flag Specifies the flag to check, this parameter can be one of the following values:
  *            @arg @ref HAL_AES_FLAG_KEYVALID Key valid flag
  *            @arg @ref HAL_AES_FLAG_BUSY GCM process suspension forbidden or transferring a shared key from SAES IP.
  *            @arg @ref HAL_AES_FLAG_WRERR Write Error flag
  *            @arg @ref HAL_AES_FLAG_RDERR Read Error flag
  *            @arg @ref HAL_AES_FLAG_CC Computation Complete flag
  *            @arg @ref HAL_AES_FLAG_KERR Key error flag
  *            @arg @ref HAL_AES_FLAG_RDWRERR Read/write Error flag
#if defined(SAES)
  *            @arg @ref HAL_AES_FLAG_RNGERR RNG Error flag
#endif

  * @retval uint32_t The state of flag (0 or 1).
  */

__STATIC_INLINE uint32_t HAL_AES_GetFlag(const hal_aes_handle_t *haes, uint32_t flag)
{
  uint32_t status;

  if ((flag == (uint32_t)HAL_AES_FLAG_KEYVALID) || (flag == (uint32_t)HAL_AES_FLAG_BUSY)
      || (flag == (uint32_t)HAL_AES_FLAG_WRERR) || (flag == (uint32_t)HAL_AES_FLAG_RDERR))
  {
    status = ((STM32_READ_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->SR,
                              (flag & 0x7FFFFFFFU)) == (flag & 0x7FFFFFFFU)) ? 1U : 0U);
  }
#if defined(SAES)
  else if ((flag == (uint32_t)HAL_AES_FLAG_CC) || (flag == (uint32_t)HAL_AES_FLAG_KERR)
           || (flag == (uint32_t)HAL_AES_FLAG_RDWRERR) || (flag == (uint32_t)HAL_AES_FLAG_RNGERR))
#else
  else if ((flag == (uint32_t)HAL_AES_FLAG_CC) || (flag == (uint32_t)HAL_AES_FLAG_KERR)
           || (flag == (uint32_t)HAL_AES_FLAG_RDWRERR))
#endif /* SAES */
  {
    status = ((STM32_READ_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->ISR, flag) == flag) ? 1U : 0U);
  }
  else
  {
    status = 0;
  }
  return (status);
}

/** @brief  Clear the AES Read/write error flag.
  * @param  haes Specifies the AES handle
  */
__STATIC_INLINE void HAL_AES_ClearFlagRDWRERR(hal_aes_handle_t *haes)
{
  STM32_SET_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->ICR, AES_ICR_RWEIF);
}

/** @brief  Clear the AES computation complete flag.
  * @param  haes Specifies the AES handle
  */
__STATIC_INLINE void HAL_AES_ClearFlagCC(hal_aes_handle_t *haes)
{
  STM32_SET_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->ICR, AES_ICR_CCF);
}

/** @brief  Clear the AES invalid key error flag.
  * @param  haes Specifies the AES handle
  */
__STATIC_INLINE void HAL_AES_ClearFlagKERR(hal_aes_handle_t *haes)
{
  STM32_SET_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->ICR, AES_ICR_KEIF);
}

#if defined(SAES)
/** @brief  Clear the AES RNG error flag.
  * @param  haes Specifies the AES handle
  */
__STATIC_INLINE void HAL_AES_ClearFlagRNGERR(hal_aes_handle_t *haes)
{
  STM32_SET_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->ICR, AES_ICR_RNGEIF);
}
#endif /* SAES */

/** @brief  Check whether the specified AES interrupt source is enabled or not.
  * @param  haes      Specifies the AES handle
  * @param  interrupt AES interrupt source to check
  *         This parameter can be one of the following values:
  *            @arg @ref HAL_AES_IT_RDWRERR Error interrupt (used for RDERR and WRERR)
  *            @arg @ref HAL_AES_IT_CC Computation Complete interrupt
  *            @arg @ref HAL_AES_IT_KERR Key error interrupt
  *            @arg @ref HAL_AES_IT_RNGERR RNG error interrupt
  * @retval uint32_t State of interruption (0 or 1).
  */
__STATIC_INLINE uint32_t HAL_AES_GetITSource(const hal_aes_handle_t *haes, uint32_t interrupt)
{
  return ((STM32_READ_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->IER, (uint32_t)interrupt)
           == (uint32_t)interrupt) ? 1U : 0U);
}

/**
  * @brief  Enable the AES interrupt.
  * @param  haes      Specifies the AES handle
  * @param  interrupt AES Interrupt
  *         This parameter can be a combination of the following values:
  *            @arg @ref HAL_AES_IT_RDWRERR Error interrupt (used for RDERR and WRERR)
  *            @arg @ref HAL_AES_IT_CC Computation Complete interrupt
  *            @arg @ref HAL_AES_IT_KERR Key error interrupt
  *            @arg @ref HAL_AES_IT_RNGERR RNG error interrupt
  */
__STATIC_INLINE void HAL_AES_EnableIT(hal_aes_handle_t *haes, uint32_t interrupt)
{
  STM32_SET_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->IER, interrupt);
}

/**
  * @brief  Disable the AES interrupt.
  * @param  haes      Specifies the AES handle
  * @param  interrupt AES Interrupt
  *         This parameter can be a combination of the following values:
  *            @arg @ref HAL_AES_IT_RDWRERR Error interrupt (used for RDERR and WRERR)
  *            @arg @ref HAL_AES_IT_CC Computation Complete interrupt
  *            @arg @ref HAL_AES_IT_KERR Key error interrupt
  *            @arg @ref HAL_AES_IT_RNGERR RNG error interrupt
  */
__STATIC_INLINE void HAL_AES_DisableIT(hal_aes_handle_t *haes, uint32_t interrupt)
{
  STM32_CLEAR_BIT(((AES_TypeDef *)((uint32_t)haes->instance))->IER, interrupt);
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
#endif /* USE_HAL_AES_ECB_CBC_ALGO || USE_HAL_AES_CTR_ALGO || USE_HAL_AES_GCM_GMAC_ALGO || USE_HAL_AES_CCM_ALGO*/
#endif /* AES or SAES */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_AES_H */
