/**
  ******************************************************************************
  * @file    stm32c5xx_hal_hash.h
  * @brief   Header file of HASH HAL module.
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

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_HASH_H
#define STM32C5XX_HAL_HASH_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (HASH)

/** @defgroup HASH HASH
  * @brief HASH HAL module driver.
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Exported_Constants HAL HASH Constants
  * @{
  */

/** @defgroup HASH_Error_Definition HASH Error Definition
  * @{
  */
#define HAL_HASH_ERROR_NONE  (0x00000000U) /*!< No error                */
#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1U)
#define HAL_HASH_ERROR_DMA   (0x00000002U) /*!< DMA-based process error */
#endif /* USE_HAL_HASH_DMA */
/**
  * @}
  */

/** @defgroup HASH_flags_definition HASH flags definitions
  * @{
  */
#define HAL_HASH_FLAG_DINI   HASH_SR_DINIS /*!< Input buffer is ready for new data                         */
#define HAL_HASH_FLAG_DCI    HASH_SR_DCIS  /*!< Digest calculation is completed                            */
#define HAL_HASH_FLAG_DMA    HASH_SR_DMAS  /*!< DMA interface is enabled or a transfer is ongoing          */
#define HAL_HASH_FLAG_BUSY   HASH_SR_BUSY  /*!< Hash core is busy, processing a block of data              */
#define HAL_HASH_FLAG_DINNE  HASH_SR_DINNE /*!< Data input register is not empty                           */
/**
  * @}
  */

/** @defgroup HASH_interrupts_definition HASH interrupts definitions
  * @{
  */
#define HAL_HASH_IT_DIN      HASH_IMR_DINIE /*!< Input buffer ready interrupt                              */
#define HAL_HASH_IT_DC       HASH_IMR_DCIE  /*!< Digest calculation complete interrupt                     */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Exported_Macros HASH Core Macros
  * @{
This subsection provides a macro that allows you to check the current handle state and move it to a new state
in an atomic way.
  */
/*! Check the current handle state and move it to a new state in an atomic way */
#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1U)
#define HASH_CHECK_UPDATE_STATE(handle, state_field, hash_conditional_state, hash_new_state)                        \
  do {                                                                                                              \
    do {                                                                                                            \
      /* Return HAL_BUSY if the status is not ready */                                                              \
      if ((__LDREXW((volatile uint32_t *)((uint32_t)&(handle)->state_field))                                        \
           & (uint32_t)(hash_conditional_state)) == 0U)                                                             \
      {                                                                                                             \
        return HAL_BUSY;                                                                                            \
      }                                                                                                             \
      /* If state is ready then attempt to change the state to the new one */                                       \
    } while (__STREXW((uint32_t)(hash_new_state), (volatile uint32_t *)((uint32_t)&((handle)->state_field))) != 0U);\
    /* Do not start any other memory access until memory barrier is complete */                                     \
    __DMB();                                                                                                        \
  } while (0)
#else
#define HASH_CHECK_UPDATE_STATE(handle, state_field, hash_conditional_state, hash_new_state)                        \
  do {                                                                                                              \
    (handle)->state_field = (hash_new_state);                                                                       \
  } while(0)
#endif /* USE_HAL_CHECK_PROCESS_STATE == 1U */
/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Exported_Types HAL HASH Types
  * @{
  */
/*! HASH instance */
typedef enum
{
  HAL_HASH = HASH_BASE, /*!< HAL HASH instance */
} hal_hash_t;

/*! HAL Global State enumeration definition */
typedef enum
{
  HAL_HASH_STATE_RESET               = 0U,           /*!< HASH is not initialized                           */
  HAL_HASH_STATE_INIT                = (1UL << 31U), /*!< HASH is initialized but not yet configured        */
  HAL_HASH_STATE_CONFIGURED          = (1UL << 30U), /*!< HASH initialized and a global config applied      */
  HAL_HASH_HMAC_STATE_CONFIGURED     = (1UL << 29U), /*!< HASH HMAC initialized and a global config applied */
  HAL_HASH_STATE_COMPUTE_ACTIVE      = (1UL << 28U), /*!< HASH compute active process is ongoing            */
  HAL_HASH_STATE_UPDATE_ACTIVE       = (1UL << 27U), /*!< HASH update active process is ongoing             */
  HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE = (1UL << 26U), /*!< HASH HMAC compute active process is ongoing       */
  HAL_HASH_HMAC_STATE_UPDATE_ACTIVE  = (1UL << 25U), /*!< HASH HMAC update active process is ongoing        */
  HAL_HASH_STATE_SUSPENDED           = (1UL << 24U), /*!< HASH is suspended                                 */
  HAL_HASH_STATE_ABORT               = (1UL << 23U)  /*!< HASH is aborted                                   */
} hal_hash_state_t;

/*! HASH algorithm selection */
typedef enum
{
  HAL_HASH_ALGO_SHA1      = 0U,                                         /*!< HASH algorithm is SHA1      */
  HAL_HASH_ALGO_SHA224    = HASH_CR_ALGO_1,                             /*!< HASH algorithm is SHA224    */
  HAL_HASH_ALGO_SHA256    = (HASH_CR_ALGO_1 | HASH_CR_ALGO_0),          /*!< HASH algorithm is SHA256    */
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
  HAL_HASH_ALGO_SHA384    = (HASH_CR_ALGO_3 | HASH_CR_ALGO_2),          /*!< HASH algorithm is SHA384    */
#endif /* HASH_CR_ALGO_3 | HASH_CR_ALGO_2 */
#if ( defined(HASH_CR_ALGO_0) && defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3))
  HAL_HASH_ALGO_SHA512224 = (HASH_CR_ALGO_3 | HASH_CR_ALGO_2 | HASH_CR_ALGO_0), /*!< HASH algorithm is SHA512224 */
#endif /* HASH_CR_ALGO_3 | HASH_CR_ALGO_2 | HASH_CR_ALGO_0 */
#if ( defined(HASH_CR_ALGO_1) && defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3))
  HAL_HASH_ALGO_SHA512256 = (HASH_CR_ALGO_3 | HASH_CR_ALGO_2 | HASH_CR_ALGO_1), /*!< HASH algorithm is SHA512256 */
#endif /* HASH_CR_ALGO_3 | HASH_CR_ALGO_2 | HASH_CR_ALGO_1 */
#if ( defined(HASH_CR_ALGO_0) && defined(HASH_CR_ALGO_1) && defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3))
  HAL_HASH_ALGO_SHA512    = (HASH_CR_ALGO_3 | HASH_CR_ALGO_2 | HASH_CR_ALGO_1 | HASH_CR_ALGO_0), /*!< HASH algorithm is SHA512    */
#endif /* HASH_CR_ALGO_3 |HASH_CR_ALGO_2 | HASH_CR_ALGO_1 | HASH_CR_ALGO_0 */
} hal_hash_algo_t;

/*! HASH data swapping enumeration definition */
typedef enum
{
  HAL_HASH_DATA_SWAP_NO       = 0U,                 /*!< 32-bit data. No swapping                     */
  HAL_HASH_DATA_SWAP_HALFWORD = HASH_CR_DATATYPE_0, /*!< 16-bit data. Each half word is swapped       */
  HAL_HASH_DATA_SWAP_BYTE     = HASH_CR_DATATYPE_1, /*!< 8-bit data. All bytes are swapped            */
  HAL_HASH_DATA_SWAP_BIT      = HASH_CR_DATATYPE    /*!< 1-bit data. In the word all bits are swapped */
} hal_hash_data_swapping_t;

/*! HASH Configuration structure definition */
typedef struct
{
  hal_hash_data_swapping_t data_swapping; /*!< Data swapping mode: No swap (32-bit data), half word swap (16-bit data),
                                               byte swap (8-bit data) or bit swap (1-bit data).
                                               This parameter can be a value of @ref hal_hash_data_swapping_t. */
  hal_hash_algo_t          algorithm;     /*!< HASH algorithm SHA-1, SHA2-224, SHA2-256, and on some supported
                    devices SHA2-384, SHA2-512224, SHA2-512256 and SHA2-512.
                                               This parameter can be a value of @ref hal_hash_algo_t           */
} hal_hash_config_t;

/*! HASH HMAC Configuration structure definition */
typedef struct
{
  hal_hash_data_swapping_t data_swapping;  /*!< No swap (32-bit data), half word swap (16-bit data), byte swap
                                                (8-bit data) or bit swap (1-bit data).
                                                This parameter can be a value of @ref hal_hash_data_swapping_t. */

  hal_hash_algo_t          algorithm;      /*!< HASH algorithm SHA-1, SHA2-224, SHA2-256, and on some supported
                    devices SHA2-384, SHA2-512224, SHA2-512256 and SHA2-512.
                                               This parameter can be a value of @ref hal_hash_algo_t            */

  uint8_t                  *p_hmac_key;    /*!< Pointer to the HMAC key data (read-only)                        */

  uint32_t                 key_size_byte;  /*!< The HMAC key size in bytes                                      */
} hal_hash_hmac_config_t;

typedef struct hal_hash_handle_s hal_hash_handle_t; /*!< HASH handle structure type */

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
typedef void (*hal_hash_cb_t)(hal_hash_handle_t *hhash); /*!< HAL HASH callback pointer definition */
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

/*! HASH suspend/resume context structure */
typedef struct
{
  uint32_t csr_reg[HASH_CSR_REGISTERS_NUMBER];      /*!< Copy of HASH context swap register when processing is
                                                         suspended                                                 */
  uint32_t imr_reg;                                 /*!< Copy of HASH interrupt enable register when processing is
                                                         suspended                                                 */
  uint32_t str_reg;                                 /*!< Copy of HASH start register when processing is suspended  */

  uint32_t cr_reg;                                  /*!< Copy of HASH control register when processing is
                                                         suspended                                                 */

  uint32_t input_data_count_byte;                   /*!< Copy of input data counter                                */

  uint32_t input_size_byte;                         /*!< Copy of buffer input size to be processed in bytes        */

  uint32_t output_size_byte;                        /*!< Copy of buffer output size to be processed in bytes       */

  uint32_t digest_size_byte;                        /*!< Copy of digest size in bytes of selected algorithm        */

  uint32_t key_size_byte;                           /*!< Copy of HASH key size in bytes                            */

  uint32_t phase;                                   /*!< Copy of HASH peripheral phase                             */

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1U)
  hal_dma_handle_t *hdma_in;                        /*!< Copy of HASH input DMA handle parameters                  */
#endif /* USE_HAL_HASH_DMA */

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS ) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hal_hash_cb_t p_digest_cplt_callback;             /*!< Copy of HASH digest computation completion callback        */
  hal_hash_cb_t p_input_cplt_callback;              /*!< Copy of HASH input FIFO transfer completed callback        */
  hal_hash_cb_t p_error_callback;                   /*!< Copy of HASH error callback                                */
  hal_hash_cb_t p_abort_cplt_callback;              /*!< Copy of HASH abort callback                                */
  hal_hash_cb_t p_suspend_cplt_callback;            /*!< Copy of HASH suspend callback                              */
#endif /* (USE_HAL_HASH_REGISTER_CALLBACKS) */

  const uint8_t *p_input_buff;                      /*!< Copy of pointer to input buffer                            */

  uint8_t *p_output_buff;                           /*!< Copy of pointer to output buffer (digest)                  */

  uint8_t *p_hmac_key_buff;                         /*!< Copy of pointer to key buffer (HMAC only)                  */

  uint8_t *p_hmac_key_saved;                        /*!< Copy of pointer to key buffer (HMAC only)                  */

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1U)
  uint8_t  dma_operation_active_flag;               /*!< Copy of DMA flag operation ongoing                         */
#endif /* USE_HAL_HASH_DMA */

  hal_hash_state_t previous_state;                  /*!< Copy of HASH peripheral state                              */

} hal_hash_suspended_context_t;

/*! HASH handle structure definition */
struct hal_hash_handle_s
{
  hal_hash_t                  instance;                 /*!< HASH register base address                     */

  uint32_t                    input_size_byte;          /*!< Buffer input size to be processed in bytes     */

  uint32_t                    output_size_byte;         /*!< Buffer output size to be processed in bytes    */

  uint32_t                    *p_output_hash_size_byte; /*!< Buffer output size processed in bytes          */

  uint32_t                    input_data_count_byte;    /*!< Input data counter                             */

  uint32_t                    digest_size_byte;         /*!< HASH digest size in bytes of selected algorithm */

  uint32_t                    block_size_byte;          /*!< HASH block size in bytes of selected algorithm */

  uint32_t                    key_size_byte;            /*!< HASH key size in bytes                         */

  volatile uint32_t           suspend_request;          /*!< HASH peripheral suspension request flag        */

  uint32_t                    phase;                    /*!< HASH peripheral phase                          */

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
  volatile  uint32_t          last_error_codes;         /*!< HASH last error codes                          */
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

  const uint8_t               *p_input_buff;            /*!< Pointer to input buffer                     */

  uint8_t                     *p_output_buff;           /*!< Pointer to output buffer (digest)           */

  uint8_t                     *p_hmac_key_buff;         /*!< Pointer to key buffer (HMAC only)           */

  uint8_t                     *p_hmac_key_saved;        /*!< Pointer to store the key buffer (HMAC only) */

  uint8_t                     remain_bytes[3];          /*!< HASH remaining bytes                        */

  uint8_t                     remain_bytes_number;      /*!< Number of remaining HASH bytes              */

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  uint8_t                     dma_operation_active_flag; /*!< DMA flag operation ongoing                  */
#endif /* USE_HAL_HASH_DMA */

  volatile hal_hash_state_t   global_state;              /*!< HASH peripheral state                       */
  volatile hal_hash_state_t   previous_state;            /*!< HASH peripheral previous state              */

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1U)
  hal_dma_handle_t           *hdma_in;                   /*!< HASH input DMA handle parameters            */
#endif /* USE_HAL_HASH_DMA */

#if defined (USE_HAL_HASH_USER_DATA) && (USE_HAL_HASH_USER_DATA == 1)
  const void                  *p_user_data;              /*!< HASH user data                              */
#endif /* USE_HAL_HASH_USER_DATA */

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS ) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hal_hash_cb_t               p_input_cplt_callback;   /*!< HASH input completion callback              */
  hal_hash_cb_t               p_digest_cplt_callback;  /*!< HASH digest computation completion callback */
  hal_hash_cb_t               p_error_callback;        /*!< HASH error callback                         */
  hal_hash_cb_t               p_suspend_cplt_callback; /*!< HASH suspend callback                       */
  hal_hash_cb_t               p_abort_cplt_callback;   /*!< HASH abort callback                         */
#endif /* (USE_HAL_HASH_REGISTER_CALLBACKS) */
};
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Exported_Functions HAL HASH Functions
  * @{
  */

/** @defgroup HASH_Exported_Functions_Group1 HASH initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_HASH_Init(hal_hash_handle_t *hhash, hal_hash_t instance);
void HAL_HASH_DeInit(hal_hash_handle_t *hhash);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group2 HASH set and get configuration functions
  * @{
  */
hal_status_t HAL_HASH_SetConfig(hal_hash_handle_t *hhash, const hal_hash_config_t *p_config);
void HAL_HASH_GetConfig(const hal_hash_handle_t *hhash, hal_hash_config_t *p_config);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group3 HASH processing functions
  * @{
  */
hal_status_t HAL_HASH_Compute(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                              void *p_output_buffer, uint32_t output_buffer_size_byte,
                              uint32_t *p_output_hash_size_byte, uint32_t timeout_ms);
hal_status_t HAL_HASH_Compute_IT(hal_hash_handle_t *hhash, const void *p_input_buffer,
                                 uint32_t input_size_byte, void *p_output_buffer,
                                 uint32_t output_buffer_size_byte, uint32_t *p_output_hash_size_byte);
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
hal_status_t HAL_HASH_Compute_DMA(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                  void *p_output_buffer, uint32_t output_buffer_size_byte,
                                  uint32_t *p_output_hash_size_byte);
#endif /* USE_HAL_HASH_DMA */

hal_status_t HAL_HASH_Update(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte,
                             uint32_t timeout_ms);

hal_status_t HAL_HASH_Update_IT(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte);

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
hal_status_t HAL_HASH_Update_DMA(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                 uint32_t input_size_byte);
#endif /* USE_HAL_HASH_DMA */

hal_status_t HAL_HASH_Finish(hal_hash_handle_t *hhash, void *p_output_buffer, uint32_t output_buffer_size_byte,
                             uint32_t *p_output_hash_size_byte, uint32_t timeout_ms);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group4 HASH HMAC Set and Get configurations functions
  * @{
  */
hal_status_t HAL_HASH_HMAC_SetConfig(hal_hash_handle_t *hhash, const hal_hash_hmac_config_t *p_config);
void HAL_HASH_HMAC_GetConfig(const hal_hash_handle_t *hhash, hal_hash_hmac_config_t *p_config);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group5 HMAC processing functions
  * @{
  */
hal_status_t HAL_HASH_HMAC_Compute(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                   void *p_output_buffer, uint32_t output_buffer_size_byte,
                                   uint32_t *p_output_hash_size_byte, uint32_t timeout_ms);
hal_status_t HAL_HASH_HMAC_Compute_IT(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                      void *p_output_buffer, uint32_t output_buffer_size_byte,
                                      uint32_t *p_output_hash_size_byte);
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
hal_status_t HAL_HASH_HMAC_Compute_DMA(hal_hash_handle_t *hhash, const void *p_input_buffer,
                                       uint32_t input_size_byte, void *p_output_buffer,
                                       uint32_t output_buffer_size_byte, uint32_t *p_output_hash_size_byte);
#endif /* USE_HAL_HASH_DMA */

hal_status_t HAL_HASH_HMAC_Update(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte,
                                  uint32_t timeout_ms);

hal_status_t HAL_HASH_HMAC_Update_IT(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                     uint32_t input_size_byte);

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
hal_status_t HAL_HASH_HMAC_Update_DMA(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                      uint32_t input_size_byte);
#endif /* USE_HAL_HASH_DMA */

hal_status_t HAL_HASH_HMAC_Finish(hal_hash_handle_t *hhash, void *p_output_buffer, uint32_t output_buffer_size_byte,
                                  uint32_t *p_output_hash_size_byte, uint32_t timeout_ms);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group6 HASH Abort functions
  * @{
  */
hal_status_t HAL_HASH_Abort(hal_hash_handle_t *hhash, uint32_t timeout_ms);
hal_status_t HAL_HASH_Abort_IT(hal_hash_handle_t *hhash);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group7 HASH IRQ handler, callbacks, and link DMA functions
  * @{
  */
void HAL_HASH_IRQHandler(hal_hash_handle_t *hhash);
void HAL_HASH_InputCpltCallback(hal_hash_handle_t *hhash);
void HAL_HASH_DigestCpltCallback(hal_hash_handle_t *hhash);
void HAL_HASH_ErrorCallback(hal_hash_handle_t *hhash);
void HAL_HASH_SuspendCallback(hal_hash_handle_t *hhash);
void HAL_HASH_AbortCallback(hal_hash_handle_t *hhash);

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS ) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
hal_status_t HAL_HASH_RegisterInputCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback);
hal_status_t HAL_HASH_RegisterDigestComputationCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback);
hal_status_t HAL_HASH_RegisterErrorCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback);
hal_status_t HAL_HASH_RegisterSuspendCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback);
hal_status_t HAL_HASH_RegisterAbortCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
hal_status_t HAL_HASH_SetInDMA(hal_hash_handle_t *hhash, hal_dma_handle_t *hdma_in);
#endif /* USE_HAL_HASH_DMA */
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group8 HASH suspend/resume functions
  * @{
  */
hal_status_t HAL_HASH_RequestSuspendComputation(hal_hash_handle_t *hhash);
hal_status_t HAL_HASH_ResumeComputation(hal_hash_handle_t *hhash);
hal_status_t HAL_HASH_RequestSuspendUpdate(hal_hash_handle_t *hhash);
hal_status_t HAL_HASH_ResumeUpdate(hal_hash_handle_t *hhash);
void HAL_HASH_SaveContext(hal_hash_handle_t *hhash, hal_hash_suspended_context_t *p_context);
void HAL_HASH_RestoreContext(hal_hash_handle_t *hhash, const hal_hash_suspended_context_t *p_context);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group9 HASH peripheral state, error, and user data functions
  * @{
  */
hal_hash_state_t HAL_HASH_GetState(const hal_hash_handle_t *hhash);

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
uint32_t HAL_HASH_GetLastErrorCodes(const hal_hash_handle_t *hhash);
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

#if defined (USE_HAL_HASH_USER_DATA) && (USE_HAL_HASH_USER_DATA == 1)
void HAL_HASH_SetUserData(hal_hash_handle_t *hhash, const void *p_user_data);
const void *HAL_HASH_GetUserData(const hal_hash_handle_t *hhash);
#endif /* USE_HAL_HASH_USER_DATA */
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group10 HASH static inline functions
  * @{
This section provides functions that allow you to manage HASH interrupts and flags:
- Call the function HAL_HASH_IsActiveFlag() to check whether the specified HASH flag is set or not.
- Call the function HAL_HASH_GetITSource() to check whether the specified HASH interrupt source is enabled or not.
- Call the function HAL_HASH_EnableIT() to enable the HASH device interrupt.
- Call the function HAL_HASH_DisableIT() to disable the device interrupt.
- Call the function HAL_HASH_ClearFlag() to clear the specified HASH flag.
  */
/** @brief Check whether the specified HASH flag is set.
  * @param hhash Specifies the HASH handle.
  * @param flag  Specifies the flag to check.
  *        This parameter must be a combination of @ref HASH_flags_definition.
  * @retval uint32_t The state of flag (0 or 1).
  */
__STATIC_INLINE uint32_t HAL_HASH_IsActiveFlag(const hal_hash_handle_t *hhash, uint32_t flag)
{
  return (uint32_t)(((((HASH_TypeDef *)((uint32_t)(hhash->instance)))->SR & (flag)) == (flag)) ? 1U : 0U);
}

/** @brief Check whether the specified HASH interrupt source is enabled.
  * @param hhash     Specifies the HASH handle.
  * @param interrupt Source to check.
  *         This parameter must be a combination of @ref HASH_interrupts_definition.
  * @retval uint32_t State of the interrupt (0 or 1).
  */
__STATIC_INLINE uint32_t HAL_HASH_GetITSource(const hal_hash_handle_t *hhash, uint32_t interrupt)
{
  return ((STM32_READ_BIT(((HASH_TypeDef *)((uint32_t)hhash->instance))->IMR, (uint32_t)interrupt)
           == (uint32_t)interrupt) ? 1U : 0U);
}

/** @brief Enable the specified HASH interrupt.
  * @param hhash     Specifies the HASH handle.
  * @param interrupt Specifies the HASH interrupt source to enable.
  *         This parameter must be a combination of @ref HASH_interrupts_definition.
  */
__STATIC_INLINE void HAL_HASH_EnableIT(hal_hash_handle_t *hhash, uint32_t interrupt)
{
  STM32_SET_BIT(((HASH_TypeDef *)((uint32_t)(hhash->instance)))->IMR, interrupt);
}

/** @brief Disable the specified HASH interrupt.
  * @param hhash     Specifies the HASH handle.
  * @param interrupt Specifies the HASH interrupt source to disable.
  *         This parameter must be a combination of @ref HASH_interrupts_definition.
  */
__STATIC_INLINE void HAL_HASH_DisableIT(hal_hash_handle_t *hhash, uint32_t interrupt)
{
  STM32_CLEAR_BIT(((HASH_TypeDef *)((uint32_t)(hhash->instance)))->IMR, interrupt);
}

/** @brief Clear the specified HASH flag.
  * @param hhash Specifies the HASH handle.
  * @param flag  Specifies the flag to clear.
  *        This parameter must be a combination of @ref HASH_flags_definition.
  */
__STATIC_INLINE void HAL_HASH_ClearFlag(hal_hash_handle_t *hhash, uint32_t flag)
{
  STM32_CLEAR_BIT(((HASH_TypeDef *)((uint32_t)(hhash->instance)))->SR, (flag));
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

#endif /* HASH */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_HASH_H */
