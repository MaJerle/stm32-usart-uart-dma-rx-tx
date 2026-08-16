/**
  ******************************************************************************
  * @file    stm32c5xx_hal_hash.c
  * @brief   HASH HAL module driver.
  *          This file provides firmware functions to manage HASH peripheral
  *
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (HASH)
#if defined (USE_HAL_HASH_MODULE) && (USE_HAL_HASH_MODULE == 1)

/** @addtogroup HASH
  * @{
  */
/** @defgroup HASH_Introduction HASH Introduction
  * @{

   The HASH hardware abstraction layer provides a set of APIs to configure and control the HASH peripheral
   on STM32 microcontrollers.

   HMAC is suitable for applications requiring message authentication.

   The HASH processor computes FIPS (Federal Information Processing Standards) approved digests of length of 160, 224,
   256 bits, for messages of any length less than 264 bits (for SHA-1, SHA-224 and SHA-256) or less than 2128 bits
   (for SHA-384, SHA-512).


  */
/**
  * @}
  */

/** @defgroup HASH_How_To_Use HASH How To Use
  * @{

# How to use this driver
The HASH HAL driver can be used as follows:

1. Initialize and de-initialize the logical HASH object :
  - To initialize the HASH peripheral, use the HAL_HASH_Init() function to initialize the HAL HASH driver for the given
    handle object
    - The HAL_HASH_Init() API allows you to associate physical instance to logical object(handle) and
      initialize the internal parameters of the handle.

    - The HAL_HASH_Init() API allows to enable the peripheral clock when USE_HASH_CLK_ENABLE_MODEL is different from
      HAL_CLK_ENABLE_NO. In case of the USE_HASH_CLK_ENABLE_MODEL compilation is not defined or define to
      HAL_CLK_ENABLE_NO, the application needs to explicitly call the HAL RCC API HAL_RCC_HASH_EnableClock() in order
      to enable the clock for the HASH peripheral.

  - To de-initialize the HASH peripheral, use the HAL_HASH_DeInit() function that stops any ongoing process and
    sets the HASH handle state to reset.

2. Set and Get HASH configuration:
 - To apply the HASH peripheral configuration, use the HAL_HASH_SetConfig().
   HASH Parameters are :
      - Data Swapping : no swap or half word swap or bit swap or byte swap.
      - algorithm : SHA-1, SHA2-224, SHA2-256, and on some supported
                    devices SHA2-384, SHA2-512224, SHA2-512256 and SHA2-512.
 - To get the HASH peripheral configuration, use the HAL_HASH_GetConfig() to retrieve the current HASH configuration.

3. Set and Get HASH HMAC configuration:
 - To apply the HASH HMAC peripheral configuration, use the HAL_HASH_HMAC_SetConfig().
   HASH HMAC Parameters are :
      - Data Swapping : no swap or half word swap or bit swap or byte swap.
      - algorithm : SHA-1, SHA2-224, SHA2-256, and on some supported
                    devices SHA2-384, SHA2-512224, SHA2-512256 and SHA2-512.
      - Key : Identifier of the key to use for the HMAC operation.
      - key size in bytes.
 - To get the HASH HMAC peripheral configuration, use the HAL_HASH_HMAC_GetConfig() to retrieve the current HASH HMAC
   configuration.

4. There are 2 families of API:
  - OneShot API: handling one single/complete buffer and providing the HASH result.
  - Update APIs: allowing the user to update several buffers then provide the HASH result corresponding
    to the data provided by the sum of these updated buffers.

5. Three processing modes are available:
 - OneShot APIs:
   - Polling mode: Processing APIs are blocking functions.
    These APIs process the data and wait until the digest computation is finished.
    Use the function HAL_HASH_Compute() for HASH or HAL_HASH_HMAC_Compute() for HMAC.
   - Interrupt mode: processing APIs are not blocking functions.
     It processes the data in interrupt mode
     Use the function HAL_HASH_Compute_IT() for HASH or HAL_HASH_HMAC_Compute_IT() for HMAC.
   - DMA mode: processing APIs are not blocking functions and the CPU is not used for data transfer.
     The data transfer is ensured by DMA.
     Use the function HAL_HASH_Compute_DMA() for HASH or HAL_HASH_HMAC_Compute_DMA() for HMAC.

 - Update APIs:
   - Polling mode:
     - HASH context mode :
       API HAL_HASH_Update() must be called to start hashing and update several input buffers.
      Call HAL_HASH_Finish() to retrieve the computed digest.
     - HMAC context mode :
       The key and the key size are entered in config API HAL_HASH_HMAC_SetConfig().
       API HAL_HASH_HMAC_Update() must be called to start hashing and update several input buffers.
      Call HAL_HASH_HMAC_Finish() to retrieve the computed digest.

   - Interrupt mode:
     - HASH context mode :
       API HAL_HASH_Update_IT() must be called to start hashing and update several input buffers.
      Call HAL_HASH_Finish() to retrieve the computed digest.

     - HMAC context mode :
       The key and the key size are entered in config API HAL_HASH_HMAC_SetConfig().
       API HAL_HASH_HMAC_Update_IT() must be called to start hashing and update several input buffers.
      Call HAL_HASH_HMAC_Finish() to retrieve the computed digest.

   - DMA mode:
     - HASH context mode :
       API HAL_HASH_Update_DMA() must be called to start hashing and update several input buffers.
      Call HAL_HASH_Finish() to retrieve the computed digest.
     - HMAC context mode :
       The key and the key size are entered in config API HAL_HASH_HMAC_SetConfig().
       API HAL_HASH_HMAC_Update_DMA() must be called to start hashing and update several input buffers.
      Call HAL_HASH_HMAC_Finish() to retrieve the computed digest.

6. Switch context:
 - Two APIs are available to suspend HASH or HMAC processing:
  - For computation process : Call the function HAL_HASH_RequestSuspendComputation()
     when a computation process is ongoing.
  - For update process: Call the function HAL_HASH_RequestSuspendUpdate() when an
     update process is ongoing.
 - Two APIs are available to resume HASH or HMAC processing:
  - For computation process : Call the function HAL_HASH_ResumeComputation() to resume the prior
     computation process and set the HAL HASH handle state to HAL_HASH_STATE_ACTIVE_COMPUTE.
  - For update process: Call the function HAL_HASH_ResumeUpdate() to resume the prior
     update process and set the HAL HASH handle state to HAL_HASH_STATE_ACTIVE_UPDATE.

 - When HASH or HMAC processing is suspended, the user can use the function HAL_HASH_SaveContext() to save the
   peripheral context. This context can be restored afterwards.
 - Before resuming the HASH or HMAC processing user can call HAL_HASH_RestoreContext() to restore the saved context
   needed if the HASH peripheral was used by another applicative process to perform some other hashing tasks.

 - Once the HASH Peripheral context has been restored to the same configuration as that at suspension time, processing
   can be resumed thanks to the APIs HAL_HASH_ResumeComputation()/HAL_HASH_ResumeUpdate() from the proper location
   reached at suspend time and with the same parameters (the required parameters to resume the operation are saved
   into the handle).

7. Remarks on message length:
 - HAL in interrupt mode (interrupt-driven):
    - Due to HASH peripheral hardware design, the peripheral interruption is triggered every 64 bytes.

 - HAL in DMA mode
    - Again, due to hardware design, the DMA transfer to feed the data can only be done on a word-basis.
      The same field described above in HASH_STR is used to specify which bits to discard at the end of the DMA
      transfer to process only the message bits and not extra bits. Due to hardware implementation,
      this is possible only at the end of the complete message. When several DMA transfers are needed to enter the
      message, this is not applicable at the end of the intermediary transfers.

8. Callback registration:
  - By default, after the HAL_HASH_Init, all callbacks are reset to the corresponding legacy weak functions:
     - HAL_HASH_InputCpltCallback()  : A Callback when an input data transfer complete has occurred.
     - HAL_HASH_DigestCpltCallback() : A Callback when a digest computation complete has occurred.
     - HAL_HASH_ErrorCallback()      : A Callback when an error has occurred.
     - HAL_HASH_SuspendCallback()    : A callback when a suspend operation has occurred.
     - HAL_HASH_AbortCallback()      : A callback when an abort operation has occurred.
  - The compilation define USE_HAL_HASH_REGISTER_CALLBACKS when set to 1 allows the user to configure dynamically
    the driver callbacks.
  - Use the function HAL_HASH_RegisterInputCpltCallback() to register a user callback for input completion.
  - Use the function HAL_HASH_RegisterDigestComputationCpltCallback() to register a user callback for input completion.
  - Use the function HAL_HASH_RegisterErrorCpltCallback() to register a user callback for error callback.
  - Use the function HAL_HASH_RegisterSuspendCpltCallback() to register a user callback for suspend callback.
  - Use the function HAL_HASH_RegisterAbortCpltCallback() to register a user callback for abort callback.

  - When the compilation define USE_HAL_HASH_REGISTER_CALLBACKS is set to 0 or not defined, the callback registering
    feature is not available and weak (overridden) callbacks are used.
  */
/**
  * @}
  */

/** @defgroup HASH_Configuration_Table HASH Configuration Table
  * @{
## Configuration inside the HASH driver

Config defines                 | Description     |   Default value | Note
-------------------------------| --------------- |-----------------| ---------------------------------------------
PRODUCT                        | from IDE        |     NONE        | STM32C5XX
USE_ASSERT_DBG_PARAM           | from the IDE    |     NONE        | Allows you to use assert parameter checks.
USE_ASSERT_DBG_STATE           | from the IDE    |     NONE        | Allows you to use assert state checks.
USE_HAL_HASH_MODULE            | from hal_conf.h |      1          | Enable the HAL HASH module
USE_HAL_CHECK_PARAM            | from hal_conf.h |      0          | Allows you to use runtime parameter checks.
USE_HAL_HASH_REGISTER_CALLBACKS| from hal_conf.h |      0          | Allows to provide specific callback functions.
USE_HAL_HASH_GET_LAST_ERRORS   | from hal_conf.h |      0          | Allows you to get last error codes.
USE_HAL_HASH_USER_DATA         | from hal_conf.h |      0          | Allows to enable/disable user data.
USE_HAL_HASH_CLK_ENABLE_MODEL  | from hal_conf.h |HAL_CLK_ENABLE_NO| Allows to enable the clock model for the HASH.
USE_HAL_HASH_DMA               | from hal_conf.h |      1          | Allows to enable the HASH DMA module service.
USE_HAL_SECURE_CHECK_PARAM     | from hal_conf.h |      0          | Allows to use the runtime check for sensitive APIs.
USE_HAL_CHECK_PROCESS_STATE    | from hal_conf.h |      0          | Allows to use the load and store exclusive
  */
/**
  * @}
  */

/* Private function prototypes --------------------------------------------------------------------------------------*/
/** @defgroup HASH_Private_Functions HASH Private Functions
  * @{
  */
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
static void HASH_ComputeDMAXferCplt(hal_dma_handle_t *hdma);
static void HASH_HMAC_ComputeDMAXferCplt(hal_dma_handle_t *hdma);
static void HASH_UpdateDMAXferCplt(hal_dma_handle_t *hdma);
static void HASH_HMAC_UpdateDMAXferCplt(hal_dma_handle_t *hdma);
static void HASH_DMAAbort(hal_dma_handle_t *hdma);
static void HASH_DMAError(hal_dma_handle_t *hdma);
static hal_status_t HASH_SuspendDMA(hal_hash_handle_t *hhash);
static hal_status_t HASH_ResumeDMA(hal_hash_handle_t *hhash);
#endif /* USE_HAL_HASH_DMA */

static void HASH_WriteKey(hal_hash_handle_t *hhash, const uint8_t *p_key, uint32_t key_size_byte);
static void HASH_GetDigestMsg(const hal_hash_handle_t *hhash, uint8_t *p_msg_digest, uint8_t input_size_byte);
static uint32_t HASH_GetDigestLength(hal_hash_algo_t algorithm);
static hal_status_t HASH_WaitUntilTimeout(hal_hash_handle_t *hhash, uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t HASH_WaitOnFlag(hal_hash_handle_t *hhash, uint32_t flag, uint32_t flag_state,
                                    uint32_t timeout_ms);
static void HASH_SetHMACMode(const hal_hash_handle_t *hhash, uint32_t key_size_bytes);
static void HASH_WriteRemainingByte(hal_hash_handle_t *hhash, const uint8_t *p_in_buff, uint32_t input_size_bytes);
static void HASH_WriteBlock(hal_hash_handle_t *hhash);
static hal_status_t HASH_WriteBlock_IT(hal_hash_handle_t *hhash);
static hal_status_t HASH_WriteIncompleteBlock(hal_hash_handle_t *hhash, uint32_t nbr_remaining_words);
static void HASH_SaveRemainingBytes(hal_hash_handle_t *hhash, uint32_t remain_bytes_nbr);
static void HASH_AppendLastIncompleteWord(hal_hash_handle_t *hhash);
static hal_status_t HASH_WriteLastBlock(hal_hash_handle_t *hhash);
static hal_status_t HASH_HMAC_ComputeProcessData_IT(hal_hash_handle_t *hhash);
static hal_status_t HASH_HMAC_UpdateProcessData_IT(hal_hash_handle_t *hhash);
static void HASH_HMAC_SwitchToStep3(hal_hash_handle_t *hhash);
/**
  * @}
  */

/* Private constants ------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Private_Constants HASH Private Constants
  * @{
  */
#define HASH_MAX_BLOCK_PROCESS_CYCLE   (500U) /*!< Max block processing time in clock cycles        */

#define HAL_DIGEST_SIZE_20B            (20U)           /*!< Digest size in bytes of SHA1                      */
#define HAL_DIGEST_SIZE_28B            (28U)           /*!< Digest size in bytes of SHA224                    */
#define HAL_DIGEST_SIZE_32B            (32U)           /*!< Digest size in bytes of SHA256                    */
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
#define HAL_DIGEST_SIZE_48B            (48U)           /*!< Digest size in bytes of SHA384                    */
#endif /* HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
#define HAL_DIGEST_SIZE_64B            (64U)           /*!< Digest size in bytes of SHA512                    */
#endif /* HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */

#define HASH_PHASE_READY                 (0x01U) /*!< HASH peripheral is ready to start                       */
#define HASH_PHASE_PROCESS               (0x02U) /*!< HASH peripheral is in HASH processing phase             */
#define HASH_PHASE_HMAC_MESSAGE_ENTRY    (0x03U) /*!< HASH peripheral is in HMAC step 2 processing phase
                                                      (It consists in entering the message text)              */
#define HASH_PHASE_HMAC_OUTER_KEY_ENTRY  (0x04U) /*!< HASH peripheral is in HMAC step 3 processing phase
                                                       (It consists in entering the outer hash function key)  */

#define HASH_WORD_SIZE_BYTE           (4U)      /*!< HASH peripheral transfers must be word-aligned (4 bytes) */
/**
  * @}
  */

/* Private macros --------------------------------------------------------------------------------------------------*/
/** @defgroup HASH_Private_Macros   HASH Private Macros
  * @{
  */
/*! Macro to get the handle instance */
#define HASH_GET_INSTANCE(handle)    ((HASH_TypeDef *)((uint32_t)(handle)->instance))

/*! Ensure that HASH input data type is valid */
#define IS_HASH_DATA_SWAPPING(data_swapping) (((data_swapping) == HAL_HASH_DATA_SWAP_NO)          \
                                              || ((data_swapping) == HAL_HASH_DATA_SWAP_HALFWORD) \
                                              || ((data_swapping) == HAL_HASH_DATA_SWAP_BYTE)     \
                                              || ((data_swapping) == HAL_HASH_DATA_SWAP_BIT))

/*! Ensure that HASH input algorithm is valid */
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
#define IS_HASH_ALGORITHM(algorithm) (((algorithm) == HAL_HASH_ALGO_SHA1)         \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA224)    \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA256)    \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA384)    \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA512224) \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA512256) \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA512))
#else
#define IS_HASH_ALGORITHM(algorithm) (((algorithm) == HAL_HASH_ALGO_SHA1)      \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA224) \
                                      || ((algorithm) == HAL_HASH_ALGO_SHA256))
#endif /* HASH_CR_ALGO_3 | HASH_CR_ALGO_2 */

/*! Ensure that HASH HMAC input algorithm is valid */
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
#define IS_HASH_HMAC_ALGORITHM(algorithm) (((algorithm) == HAL_HASH_ALGO_SHA1)         \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA224)    \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA256)    \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA384)    \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA512224) \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA512256) \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA512))
#else
#define IS_HASH_HMAC_ALGORITHM(algorithm) (((algorithm) == HAL_HASH_ALGO_SHA1)      \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA224) \
                                           || ((algorithm) == HAL_HASH_ALGO_SHA256))
#endif /* HASH_CR_ALGO_3 | HASH_CR_ALGO_2 */

/*! Checks if the output buffer size is valid for the given HASH/HMAC handle */
#define IS_OUT_BUFFER_SIZE_VALID(handle, size) ((size != 0U) && (size >= handle->digest_size_byte))
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup HASH_Exported_Functions
  * @{
  */
/** @addtogroup HASH_Exported_Functions_Group1
  * @{
This section provides functions allowing to:
- Initialize the HASH handle and associate it to a given HASH peripheral instance.
- DeInitialize the HASH peripheral.
  */
/**
  * @brief Initialize the selected HAL HASH handle and associate an HASH peripheral instance.
  * @param  hhash             Pointer to a hal_hash_handle_t structure.
  * @param  instance          HASH instance.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            HASH handle has been correctly initialized.
  */
hal_status_t HAL_HASH_Init(hal_hash_handle_t *hhash, hal_hash_t instance)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(IS_HASH_ALL_INSTANCE((HASH_TypeDef *)((uint32_t)instance)));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || (defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  /* Associate physical instance to logical object */
  hhash->instance = instance;

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1U)
  hhash->hdma_in = (hal_dma_handle_t *)NULL;
#endif /* USE_HAL_HASH_DMA */

#if defined (USE_HAL_HASH_CLK_ENABLE_MODEL) && (USE_HAL_HASH_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_HASH_EnableClock();
#endif /* USE_HAL_HASH_CLK_ENABLE_MODEL */

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_input_cplt_callback   = HAL_HASH_InputCpltCallback;
  hhash->p_digest_cplt_callback  = HAL_HASH_DigestCpltCallback;
  hhash->p_error_callback        = HAL_HASH_ErrorCallback;
  hhash->p_suspend_cplt_callback = HAL_HASH_SuspendCallback;
  hhash->p_abort_cplt_callback   = HAL_HASH_AbortCallback;
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
  hhash->last_error_codes = HAL_HASH_ERROR_NONE;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

#if defined(USE_HAL_HASH_USER_DATA) && (USE_HAL_HASH_USER_DATA == 1)
  hhash->p_user_data = NULL;
#endif /* USE_HAL_HASH_USER_DATA */

  hhash->global_state = HAL_HASH_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief DeInitialize the HASH peripheral.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  */
void HAL_HASH_DeInit(hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(IS_HASH_ALL_INSTANCE(HASH_GET_INSTANCE(hhash)));

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  if ((hash_instance->CR & HASH_CR_DMAE) != 0U)
  {
    STM32_CLEAR_BIT(hash_instance->CR, HASH_CR_DMAE);

    (void)HAL_DMA_Abort(hhash->hdma_in);
  }
#endif /* USE_HAL_HASH_DMA */

  hhash->global_state = HAL_HASH_STATE_RESET;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group2
  * @{
This subsection provides a set of functions allowing to set and get the HASH configuration.
- Use the function HAL_HASH_SetConfig() to Configure HASH with the specified parameters in the hal_hash_config_t
  Parameters which are :
   - Data Swapping : no swap or half word swap or bit swap or byte swap.
   - algorithm : SHA-1, SHA2-224, SHA2-256, and on some supported
                    devices SHA2-384, SHA2-512224, SHA2-512256 and SHA2-512.
- Use the function HAL_HASH_GetConfig() to retrieve the HAL HASH configuration.
  */
/**
  * @brief Configure the HASH according to the specified parameters in the hal_hash_config_t.
  * @param  hhash             Pointer to a hal_hash_handle_t structure
  * @param  p_config          Pointer to a hal_hash_config_t structure that contains the configuration for HASH module.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            HASH instance has been correctly configured.
  */
hal_status_t HAL_HASH_SetConfig(hal_hash_handle_t *hhash, const hal_hash_config_t *p_config)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_HASH_DATA_SWAPPING(p_config->data_swapping));
  ASSERT_DBG_PARAM(IS_HASH_ALGORITHM(p_config->algorithm));

  ASSERT_DBG_STATE(hhash->global_state, ((uint32_t)HAL_HASH_STATE_INIT) | (uint32_t)HAL_HASH_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

#if (defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
     || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1))
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  STM32_CLEAR_BIT(hash_instance->CR, HASH_CR_LKEY);

  /* Set the data type and algorithm */
  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_DATATYPE | HASH_CR_ALGO | HASH_CR_INIT,
                   ((uint32_t)p_config->data_swapping | (uint32_t)p_config->algorithm | HASH_CR_INIT));

  /* Get the digest size in bytes according to the selected algorithm */
  hhash->digest_size_byte = HASH_GetDigestLength(p_config->algorithm);

  /* Get the block size in bytes which can be deduced from the number of expected words returned by NBWE bits */
  hhash->block_size_byte = (((STM32_READ_REG(hash_instance->SR) >> HASH_SR_NBWE_Pos) - 1U) << 2U);

  hhash->remain_bytes_number = 0U;
  hhash->phase               = HASH_PHASE_READY;
  hhash->suspend_request     = 0U;
  hhash->global_state        = HAL_HASH_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Get HASH Configuration parameters.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param p_config Pointer to a hal_hash_config_t structure.
  */
void HAL_HASH_GetConfig(const hal_hash_handle_t *hhash, hal_hash_config_t *p_config)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE | (uint32_t)HAL_HASH_STATE_SUSPENDED);

  const HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  p_config->data_swapping = (hal_hash_data_swapping_t)(uint32_t)((hash_instance->CR) & (HASH_CR_DATATYPE));
  p_config->algorithm = (hal_hash_algo_t)(uint32_t)((hash_instance->CR) & (HASH_CR_ALGO));
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group3
  * @{
This section provides API allowing to calculate the hash value using one of the HASH algorithms supported by
the peripheral.

For a single buffer to be hashed, user can resort to one of three processing functions available:
- Polling mode   : HAL_HASH_Compute()
- Interrupt mode : HAL_HASH_Compute_IT()
- DMA mode       : HAL_HASH_Compute_DMA()

In case of multi-buffer HASH processing (a single digest is computed while several buffers are fed to the peripheral),
the user can resort to successive calls to :
- Polling mode : Call HAL_HASH_Update() to continue HASH update process.
- Interrupt mode : Call HAL_HASH_Update_IT() to continue HASH update process.
- DMA mode: Call HAL_HASH_Update_DMA() to continue HASH update process.

To retrieve the digest computation, call HAL_HASH_Finish().
  */
/**
  * @brief Compute HASH input buffer in polling mode then retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param  hhash                    Pointer to a hal_hash_handle_t structure.
  * @param  p_input_buffer           Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte          Size of the input buffer in bytes.
  * @param  p_output_buffer          Pointer to the computed digest.
  * @param  output_buffer_size_byte  Size of the output buffer in bytes provided by the user.
  * @param  p_output_hash_size_byte  Pointer to the size of the digest in bytes.
  * @param  timeout_ms               Specify timeout value in millisecond
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_TIMEOUT              A timeout has occurred.
  * @retval HAL_ERROR                An error has occurred.
  * @retval HAL_BUSY                 Process is already ongoing.
  * @retval HAL_OK                   Operation completed.
  */
hal_status_t HAL_HASH_Compute(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                              void *p_output_buffer, uint32_t output_buffer_size_byte,
                              uint32_t *p_output_hash_size_byte, uint32_t timeout_ms)
{
  uint32_t digest_copy_length = 0U;
  uint32_t valid_blocks_nbr   = 0U;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (p_output_hash_size_byte == NULL)
      || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_STATE_CONFIGURED, HAL_HASH_STATE_COMPUTE_ACTIVE);

  tickstart = HAL_GetTick();

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  hhash->p_input_buff            = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->input_data_count_byte   = 0U;
  hhash->input_size_byte         = input_size_byte;
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));

  hhash->phase = HASH_PHASE_PROCESS;
  valid_blocks_nbr = input_size_byte / (hhash->block_size_byte + HASH_WORD_SIZE_BYTE);

  for (uint32_t index = 0U; index < valid_blocks_nbr; index++)
  {
    HASH_WriteBlock(hhash);
  }

  if ((hhash->input_size_byte - hhash->input_data_count_byte) != 0U)
  {
    if (HASH_WriteLastBlock(hhash) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  /* Start the message Digest calculation */
  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  digest_copy_length = (hhash->output_size_byte > hhash->block_size_byte)
                       ? hhash->block_size_byte : hhash->output_size_byte;
  HASH_GetDigestMsg(hhash, (uint8_t *)hhash->p_output_buff, (uint8_t)digest_copy_length);

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tickstart) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  hhash->phase = HASH_PHASE_READY;
  hhash->global_state = HAL_HASH_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Compute HASH input buffer in interrupt mode then retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param  hhash                    Pointer to a hal_hash_handle_t structure.
  * @param  p_input_buffer           Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte          Size of the input buffer in bytes.
  * @param  p_output_buffer          Pointer to the computed digest.
  * @param  output_buffer_size_byte  Size of the output buffer in bytes provided by the user.
  * @param  p_output_hash_size_byte  Pointer to the size of the digest in bytes.
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_ERROR                An error has occurred.
  * @retval HAL_BUSY                 Process is already ongoing.
  * @retval HAL_OK                   Operation started correctly.
  */
hal_status_t HAL_HASH_Compute_IT(hal_hash_handle_t *hhash, const void *p_input_buffer,
                                 uint32_t input_size_byte, void *p_output_buffer,
                                 uint32_t output_buffer_size_byte, uint32_t *p_output_hash_size_byte)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (p_output_hash_size_byte == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_STATE_CONFIGURED, HAL_HASH_STATE_COMPUTE_ACTIVE);

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  hhash->input_data_count_byte   = 0U;
  hhash->p_input_buff            = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->input_size_byte         = input_size_byte;
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));
  hhash->phase = HASH_PHASE_PROCESS;

  /* The number of word expected to be pushed to DIN is set to the expected block size + 1 in words (0x11) */
  if (hhash->input_size_byte >= (hhash->block_size_byte + HASH_WORD_SIZE_BYTE))
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);

  return HAL_OK;
}

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief Compute HASH input buffer in DMA mode then retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param  hhash                    Pointer to a hal_hash_handle_t structure.
  * @param  p_input_buffer           Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte          Size of the input buffer in bytes.
  * @param  p_output_buffer          Pointer to the computed digest.
  * @param  output_buffer_size_byte  Size of the output buffer in bytes provided by the user.
  * @param  p_output_hash_size_byte  Pointer to the size of the digest in bytes.
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_ERROR                An error has occurred.
  * @retval HAL_BUSY                 Process is already ongoing.
  * @retval HAL_OK                   Operation started correctly.
  */
hal_status_t HAL_HASH_Compute_DMA(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                  void *p_output_buffer, uint32_t output_buffer_size_byte,
                                  uint32_t *p_output_hash_size_byte)
{
  uint32_t input_addr = (uint32_t)p_input_buffer;
  uint32_t tmp_input_size;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((hhash == NULL) || (hhash->hdma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash-> digest_size_byte) || (p_output_hash_size_byte == NULL))
  {
    return HAL_INVALID_PARAM;
  }

#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_STATE_CONFIGURED, HAL_HASH_STATE_COMPUTE_ACTIVE);

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
  hhash->last_error_codes = HAL_HASH_ERROR_NONE;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  STM32_SET_BIT(hash_instance->CR, HASH_CR_DMAE);

  hhash->p_input_buff            = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->input_data_count_byte   = 0U;
  hhash->input_size_byte         = input_size_byte;
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  hhash->dma_operation_active_flag = 1U;

  hhash->hdma_in->p_xfer_cplt_cb  = HASH_ComputeDMAXferCplt;
  hhash->hdma_in->p_xfer_error_cb = HASH_DMAError;
  hhash->hdma_in->p_xfer_abort_cb = HASH_DMAAbort;

  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);
  hhash->phase = HASH_PHASE_PROCESS;

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));

  tmp_input_size = (((hhash->input_size_byte % HASH_WORD_SIZE_BYTE) != 0U)
                    ? (hhash->input_size_byte + (HASH_WORD_SIZE_BYTE - (hhash->input_size_byte % 4U)))
                    : (hhash->input_size_byte));

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hhash->hdma_in, input_addr,
                                     (uint32_t) &((HASH_TypeDef *)((uint32_t)(hhash)->instance))->DIN, tmp_input_size,
                                     HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
    hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* USE_HAL_HASH_DMA */

/**
  * @brief HASH update process in polling mode with several input buffers.
  * @param  hhash              Pointer to a hal_hash_handle_t structure.
  * @param  p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte    length of the input buffer in bytes.
  * @param  timeout_ms         Specify timeout value in millisecond.
  * @note   Consecutive calls to HAL_HASH_Update() can be used to feed several input buffers back-to-back to the
  *         peripheral that will yield a single HASH signature once all buffers have been entered.
  *         Wrap-up of input buffers feeding and retrieval of digest is done by a call to HAL_HASH_Finish().
  * @retval HAL_INVALID_PARAM  Invalid parameter.
  * @retval HAL_TIMEOUT        A timeout has occurred.
  * @retval HAL_BUSY           Process is already ongoing.
  * @retval HAL_ERROR          An error has occurred.
  * @retval HAL_OK             Input buffer fed correctly.
  */
hal_status_t HAL_HASH_Update(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte,
                             uint32_t timeout_ms)
{
  uint32_t valid_blocks_nbr    = 0U;
  uint32_t remain_bytes_nbr    = 0U;
  uint32_t remaining_words_nbr = 0U;
  uint32_t tick_start;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);
  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_add_input_buffer == NULL) || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE),
                          HAL_HASH_STATE_UPDATE_ACTIVE);

  tick_start = HAL_GetTick();

  hhash->p_input_buff          = (const uint8_t *)p_add_input_buffer;
  hhash->input_data_count_byte = 0U;
  hhash->input_size_byte       = input_size_byte;

  if (hhash->phase == HASH_PHASE_READY)
  {
    STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);
    hhash->phase = HASH_PHASE_PROCESS;
  }

  if (hhash->remain_bytes_number != 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  valid_blocks_nbr = hhash->input_size_byte / (hhash->block_size_byte + HASH_WORD_SIZE_BYTE);
  for (uint32_t index = 0U; index < valid_blocks_nbr; index++)
  {
    HASH_WriteBlock(hhash);
  }

  remain_bytes_nbr = hhash->input_size_byte - hhash->input_data_count_byte;
  if (remain_bytes_nbr != 0U)
  {
    remaining_words_nbr = remain_bytes_nbr / HASH_WORD_SIZE_BYTE;

    if (remaining_words_nbr != 0U)
    {
      if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
      {
        hhash->phase = HASH_PHASE_READY;
        hhash->global_state = HAL_HASH_STATE_CONFIGURED;
        return HAL_ERROR;
      }
    }

    /* Still remaining bytes to save for next HASH update or Finish operation */
    if ((remain_bytes_nbr % HASH_WORD_SIZE_BYTE) != 0U)
    {
      HASH_SaveRemainingBytes(hhash, remain_bytes_nbr);
    }
  }

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tick_start) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief  HASH update process in interrupt mode several input buffers.
  * @param  hhash              Pointer to a hal_hash_handle_t structure.
  * @param  p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte    length of the input buffer in bytes.
  * @note   Consecutive calls to HAL_HASH_Update_IT() can be used to feed several input buffers back-to-back
  *         to the peripheral that will yield a single HASH signature once all buffers have been entered.
  *         Wrap-up of input buffers feeding and retrieval of digest is done by a call to HAL_HASH_Finish().
  * @retval HAL_INVALID_PARAM  Invalid parameter.
  * @retval HAL_ERROR          An error has occurred.
  * @retval HAL_BUSY           Process is already ongoing.
  * @retval HAL_OK             The update of the given buffer has been processed correctly.
  */
hal_status_t HAL_HASH_Update_IT(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte)
{
  uint32_t remain_bytes_nbr = 0U;
  uint32_t remaining_words_nbr = 0U;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);
  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (p_add_input_buffer == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE),
                          HAL_HASH_STATE_UPDATE_ACTIVE);

  if (hhash->phase == HASH_PHASE_READY)
  {
    STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);
    hhash->phase = HASH_PHASE_PROCESS;
  }

  hhash->p_input_buff = (const uint8_t *)p_add_input_buffer;
  hhash->input_data_count_byte = 0U;
  hhash->input_size_byte = input_size_byte;

  if (hhash->remain_bytes_number != 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  /* The number of word expected to be pushed to DIN is set to the expected block size + 1 in words ( 0x11) */
  if (hhash->input_size_byte >= (hhash->block_size_byte + HASH_WORD_SIZE_BYTE))
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_STATE_CONFIGURED;
      return HAL_ERROR;
    }

    HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN);
  }
  else /* No complete block */
  {
    remain_bytes_nbr = hhash->input_size_byte - hhash->input_data_count_byte;
    remaining_words_nbr = remain_bytes_nbr / HASH_WORD_SIZE_BYTE;

    if (remaining_words_nbr != 0U)
    {
      if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
      {
        hhash->phase = HASH_PHASE_READY;
        hhash->global_state = HAL_HASH_STATE_CONFIGURED;
        return HAL_ERROR;
      }
    }

    /* Still remaining bytes to save for next HASH update or Finish operation */
    if ((remain_bytes_nbr % HASH_WORD_SIZE_BYTE) != 0U)
    {
      HASH_SaveRemainingBytes(hhash, remain_bytes_nbr);
    }
  }

  return HAL_OK;
}

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief  HASH update process in DMA mode with several input buffers.
  * @param  hhash              Pointer to a hal_hash_handle_t structure.
  * @param  p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param  input_size_byte    length of the input buffer in bytes.
  * @retval HAL_INVALID_PARAM  Invalid parameter.
  * @retval HAL_ERROR          An error has occurred.
  * @retval HAL_BUSY           Process is already ongoing.
  * @retval HAL_OK             The update of the given buffer has been processed correctly.
  */
hal_status_t HAL_HASH_Update_DMA(hal_hash_handle_t *hhash, const void *p_add_input_buffer, uint32_t input_size_byte)
{
  uint32_t tmp_input_addr;
  uint8_t remain_size_byte;
  uint32_t valid_words_nbr;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);
  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
  || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (p_add_input_buffer == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((hhash == NULL) || (hhash->hdma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE),
                          HAL_HASH_STATE_UPDATE_ACTIVE);

  HASH_TypeDef *hash_instance      = HASH_GET_INSTANCE(hhash);
  hhash->p_input_buff              = (const uint8_t *)p_add_input_buffer;
  hhash->input_data_count_byte     = 0U;
  hhash->input_size_byte           = input_size_byte;
  hhash->dma_operation_active_flag = 1U;

  hhash->hdma_in->p_xfer_cplt_cb  = HASH_UpdateDMAXferCplt;
  hhash->hdma_in->p_xfer_error_cb = HASH_DMAError;
  hhash->hdma_in->p_xfer_abort_cb = HASH_DMAAbort;

  STM32_SET_BIT(hash_instance->CR, HASH_CR_DMAE);

  if (hhash->phase == HASH_PHASE_READY)
  {
    STM32_MODIFY_REG(hash_instance->CR, HASH_CR_INIT | HASH_CR_MODE, HASH_CR_INIT);
    hhash->phase = HASH_PHASE_PROCESS;
    STM32_SET_BIT(hash_instance->CR, HASH_CR_MDMAT);
  }

  /* Process the remaining bytes of the previous buffer */
  if (hhash->remain_bytes_number != 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  tmp_input_addr = (uint32_t)hhash->p_input_buff;
  remain_size_byte = (uint8_t)(hhash->input_size_byte % HASH_WORD_SIZE_BYTE);

  /* Store remaining bytes in remain_bytes */
  if (remain_size_byte > 0U)
  {
    hhash->input_size_byte = hhash->input_size_byte - remain_size_byte;
    for (uint32_t i = 0U; i < remain_size_byte; i++)
    {
      hhash->remain_bytes[i] = ((uint8_t *)tmp_input_addr)[hhash->input_size_byte + i];
    }
    hhash->remain_bytes_number = remain_size_byte;
  }

  valid_words_nbr = hhash->input_size_byte / HASH_WORD_SIZE_BYTE;
  if (valid_words_nbr > 0U)
  {
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 0U);
    if (HAL_DMA_StartPeriphXfer_IT_Opt(hhash->hdma_in,
                                       tmp_input_addr,
                                       (uint32_t) &((HASH_TypeDef *)((uint32_t)(hhash)->instance))->DIN,
                                       hhash->input_size_byte,
                                       HAL_DMA_OPT_IT_NONE) != HAL_OK)
    {
#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
      hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */
      hhash->global_state = HAL_HASH_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
#endif /* USE_HAL_HASH_DMA */

/**
  * @brief Finish HASH update process.
  * @param  hhash                    Pointer to a hal_hash_handle_t structure.
  * @param  p_output_buffer          Pointer to the computed digest.
  * @param  output_buffer_size_byte  Size of the output buffer in bytes provided by the user.
  * @param  p_output_hash_size_byte  Pointer to the size of the digest in bytes.
  * @param  timeout_ms               Specify timeout value in millisecond.
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_TIMEOUT              A timeout has occurred.
  * @retval HAL_ERROR                An error has occurred.
  * @retval HAL_OK                   Hash operation correctly completed and digest available in the output buffer.
  */
hal_status_t HAL_HASH_Finish(hal_hash_handle_t *hhash, void *p_output_buffer, uint32_t output_buffer_size_byte,
                             uint32_t *p_output_hash_size_byte, uint32_t timeout_ms)
{
  uint32_t digest_copy_length = 0U;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_output_buffer == NULL) || (p_output_hash_size_byte == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  tickstart = HAL_GetTick();

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  if (hhash->remain_bytes_number != 0U)
  {
    /* Configure the number of valid bits in last word of the message */
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, (8U * (uint32_t)hhash->remain_bytes_number));

    HASH_WriteRemainingByte(hhash, (uint8_t *)hhash->remain_bytes, hhash->remain_bytes_number);
  }
  else
  {
    /* Configure the number of valid bits in last word of the message */
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * (hhash->input_size_byte % HASH_WORD_SIZE_BYTE));
  }

  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  digest_copy_length = (hhash->output_size_byte > hhash->block_size_byte)
                       ? hhash->block_size_byte : hhash->output_size_byte;
  HASH_GetDigestMsg(hhash, (uint8_t *)hhash->p_output_buff, (uint8_t)digest_copy_length);

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tickstart) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  if (STM32_IS_BIT_SET(hash_instance->CR, HASH_CR_MDMAT))
  {
    STM32_CLEAR_BIT(((HASH_TypeDef *)((uint32_t)(hhash->instance)))->CR, HASH_CR_MDMAT);
  }

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  hhash->dma_operation_active_flag = 0U;
#endif /* USE_HAL_HASH_DMA */
  hhash->remain_bytes_number = 0U;
  hhash->phase               = HASH_PHASE_READY;
  hhash->global_state        = HAL_HASH_STATE_CONFIGURED;

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group4
  * @{
This subsection provides a set of functions allowing to set and get the HASH HMAC configuration.
- Use the function HAL_HASH_HMAC_SetConfig() to Configure HASH HMAC with the specified parameters in the
  hal_hash_hmac_config_t Parameters which are :
   - Data Swapping : no swap or half word swap or bit swap or byte swap.
   - Key and key size in byte to use for the HMAC operation
- Use the function HAL_HASH_HMAC_GetConfig() to retrieve the HAL HASH HMAC configuration.
  */
/**
  * @brief Configure the HASH HMAC according to the specified parameters in the hal_hash_hmac_config_t.
  * @param hhash              Pointer to a hal_hash_handle_t structure.
  * @param p_config           Pointer to a hal_hash_hmac_config_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_OK            Operation started correctly.
  */
hal_status_t HAL_HASH_HMAC_SetConfig(hal_hash_handle_t *hhash, const hal_hash_hmac_config_t *p_config)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_HASH_DATA_SWAPPING(p_config->data_swapping));
  ASSERT_DBG_PARAM(IS_HASH_HMAC_ALGORITHM(p_config->algorithm));
  ASSERT_DBG_PARAM(p_config->p_hmac_key != NULL);
  ASSERT_DBG_PARAM(p_config->key_size_byte != 0U);

  ASSERT_DBG_STATE(hhash->global_state, ((uint32_t)HAL_HASH_STATE_INIT) | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_STATE_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  /* Set the data type and algorithm */
  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_DATATYPE | HASH_CR_ALGO | HASH_CR_INIT,
                   (((uint32_t)p_config->data_swapping | (uint32_t)p_config->algorithm | HASH_CR_INIT)));

  hhash->phase = HASH_PHASE_READY;

  /* Get the digest size in bytes according to the selected algorithm */
  hhash->digest_size_byte = HASH_GetDigestLength(p_config->algorithm);

  /* Get the block size in bytes which can be deduced from the number of expected words returned by NBWE bits */
  hhash->block_size_byte = (((STM32_READ_REG(hash_instance->SR) >> HASH_SR_NBWE_Pos) - 1U) << 2U);

  hhash->remain_bytes_number   = 0U;
  hhash->p_hmac_key_buff       = p_config->p_hmac_key;
  hhash->key_size_byte         = p_config->key_size_byte;
  hhash->p_hmac_key_saved      = p_config->p_hmac_key;
  hhash->input_data_count_byte = 0U;

  HASH_SetHMACMode(hhash, hhash->key_size_byte);

  hhash->phase = HASH_PHASE_PROCESS;

  /* Configure the number of valid bits in last word of the Key */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((p_config->key_size_byte) % HASH_WORD_SIZE_BYTE));
  HASH_WriteKey(hhash, p_config->p_hmac_key, p_config->key_size_byte);

  /* Start the Key padding then the Digest calculation */
  STM32_SET_BIT(hash_instance->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  hhash->phase           = HASH_PHASE_HMAC_MESSAGE_ENTRY;
  hhash->suspend_request = 0U;
  hhash->global_state    = HAL_HASH_HMAC_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Get HASH HMAC Configuration parameters.
  * @param hhash       Pointer to a hal_hash_handle_t structure.
  * @param p_config    Pointer to a hal_hash_hmac_config_t structure.
  */
void HAL_HASH_HMAC_GetConfig(const hal_hash_handle_t *hhash, hal_hash_hmac_config_t *p_config)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE | (uint32_t)HAL_HASH_STATE_SUSPENDED);

  const HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  p_config->data_swapping = (hal_hash_data_swapping_t)(uint32_t)((hash_instance->CR) & (HASH_CR_DATATYPE));
  p_config->algorithm     = (hal_hash_algo_t)(uint32_t)((hash_instance->CR) & (HASH_CR_ALGO));
  p_config->p_hmac_key    = hhash->p_hmac_key_saved;
  p_config->key_size_byte = hhash->key_size_byte;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group5
  * @{
This section provides API allowing to calculate the HMAC (keyed-hash message authentication code).

To calculate the HMAC for a single buffer, user can resort to one of three processing functions available:
- Polling mode : HAL_HASH_HMAC_Compute()
- Interrupt mode : HAL_HASH_HMAC_Compute_IT()
- DMA mode : HAL_HASH_HMAC_Compute_DMA()

In case of multi-buffer HMAC processing (a single digest is computed while several buffers are fed to the peripheral),
the user can resort to successive calls to :
- Polling mode : Call HAL_HASH_HMAC_Update() to continue HASH HMAC update process.
- Interrupt mode : Call HAL_HASH_HMAC_Update_IT() to continue HASH HMAC update process.
- DMA mode: Call HAL_HASH_HMAC_Update_DMA() to continue HASH HMAC update process.

To retrieve the digest computation, call HAL_HASH_HMAC_Finish().
  */
/**
  * @brief Compute HASH HMAC input buffer in polling mode, then retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param hhash                   Pointer to a hal_hash_handle_t structure.
  * @param p_input_buffer          Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte         length of the input buffer in bytes.
  * @param p_output_buffer         Pointer to the computed digest.
  * @param output_buffer_size_byte Size of the output buffer in bytes provided by the user.
  * @param p_output_hash_size_byte Pointer to the size of the digest in bytes.
  * @param timeout_ms              Specify timeout value in millisecond.
  * @retval                        HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT            A timeout has occurred.
  * @retval HAL_ERROR              An error has occurred.
  * @retval HAL_BUSY               Process is already ongoing.
  * @retval HAL_OK                 Operation completed.
  */
hal_status_t HAL_HASH_HMAC_Compute(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                   void *p_output_buffer, uint32_t output_buffer_size_byte,
                                   uint32_t *p_output_hash_size_byte, uint32_t timeout_ms)
{
  uint32_t digest_copy_length = 0U;
  uint32_t valid_blocks_nbr   = 0U;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (p_output_hash_size_byte == NULL)
      || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_HMAC_STATE_CONFIGURED, HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE);

  tickstart = HAL_GetTick();

  hhash->phase                   = HASH_PHASE_READY;
  hhash->p_input_buff            = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->input_data_count_byte   = 0U;
  hhash->input_size_byte         = input_size_byte;
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));
  hhash->input_data_count_byte = 0U;

  valid_blocks_nbr = input_size_byte / (hhash->block_size_byte + HASH_WORD_SIZE_BYTE);
  for (uint32_t index = 0U; index < valid_blocks_nbr; index++)
  {
    HASH_WriteBlock(hhash);
  }

  if ((hhash->input_size_byte - hhash->input_data_count_byte) != 0U)
  {
    if (HASH_WriteLastBlock(hhash) != HAL_OK)
    {
      hhash->phase        = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(hash_instance->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_BUSY, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase        = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  /* Configure the number of valid bits in last word of the Key */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->key_size_byte) % HASH_WORD_SIZE_BYTE));
  hhash->input_data_count_byte = 0U;
  HASH_WriteKey(hhash, hhash->p_hmac_key_saved, hhash->key_size_byte);

  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  digest_copy_length = (output_buffer_size_byte < hhash->digest_size_byte) ? output_buffer_size_byte :
                       hhash->digest_size_byte;
  HASH_GetDigestMsg(hhash, (uint8_t *)p_output_buffer, (uint8_t)digest_copy_length);

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tickstart) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  hhash->phase        = HASH_PHASE_READY;
  hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Compute HASH HMAC input buffer in interrupt mode, retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param hhash                   Pointer to a hal_hash_handle_t structure.
  * @param p_input_buffer          Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte         length of the input buffer in bytes.
  * @param p_output_buffer         Pointer to the computed digest.
  * @param output_buffer_size_byte Size of the output buffer in bytes provided by the user.
  * @param p_output_hash_size_byte Pointer to the length of the computed digest in bytes.
  * @retval HAL_INVALID_PARAM      Invalid parameter.
  * @retval HAL_ERROR              An error has occurred.
  * @retval HAL_BUSY               Process is already ongoing.
  * @retval HAL_OK                 Operation started correctly.
  */
hal_status_t HAL_HASH_HMAC_Compute_IT(hal_hash_handle_t *hhash, const void *p_input_buffer, uint32_t input_size_byte,
                                      void *p_output_buffer, uint32_t output_buffer_size_byte,
                                      uint32_t *p_output_hash_size_byte)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (p_output_hash_size_byte == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_HMAC_STATE_CONFIGURED, HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE);

  hhash->phase = HASH_PHASE_READY;

  hhash->p_input_buff            = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->p_hmac_key_buff         = hhash->p_hmac_key_saved;
  hhash->input_data_count_byte   = 0U;
  hhash->input_size_byte         = input_size_byte;
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  hhash->phase = HASH_PHASE_HMAC_MESSAGE_ENTRY;

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));

  /* The number of word expected to be pushed to DIN is set to the expected block size +1 in words ( 0x11) */
  if (hhash->input_size_byte >= (hhash->block_size_byte + HASH_WORD_SIZE_BYTE))
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);

  return HAL_OK;
}

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief Compute HASH HMAC input buffer in DMA mode, then retrieve the computed digest in the output buffer
  *        and its size in bytes.
  * @param hhash                   Pointer to a hal_hash_handle_t structure.
  * @param p_input_buffer          Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte         length of the input buffer in bytes.
  * @param p_output_buffer         Pointer to the computed digest.
  * @param output_buffer_size_byte Size of the output buffer in bytes provided by the user.
  * @param p_output_hash_size_byte Pointer to the length of the computed digest in bytes.
  * @retval HAL_INVALID_PARAM      Invalid parameter.
  * @retval HAL_ERROR              An error has occurred.
  * @retval HAL_BUSY               Process is already ongoing.
  * @retval HAL_OK                 Operation started correctly.
  */
hal_status_t HAL_HASH_HMAC_Compute_DMA(hal_hash_handle_t *hhash, const void *p_input_buffer,
                                       uint32_t input_size_byte, void *p_output_buffer,
                                       uint32_t output_buffer_size_byte, uint32_t *p_output_hash_size_byte)
{
  uint32_t src_addr;
  uint32_t size_byte;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_input_buffer != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((hhash == NULL) || (hhash->hdma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_input_buffer == NULL) || (p_output_buffer == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (p_output_hash_size_byte == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_HMAC_STATE_CONFIGURED, HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE);

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
  hhash->last_error_codes = HAL_HASH_ERROR_NONE;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  hhash->p_input_buff              = (const uint8_t *)p_input_buffer;
  hhash->p_output_buff             = (uint8_t *)p_output_buffer;
  hhash->p_hmac_key_buff           = hhash->p_hmac_key_saved;
  hhash->input_data_count_byte     = 0U;
  hhash->input_size_byte           = input_size_byte;
  hhash->output_size_byte          = output_buffer_size_byte;
  hhash->p_output_hash_size_byte   = p_output_hash_size_byte;
  hhash->dma_operation_active_flag = 1U;
  *p_output_hash_size_byte         = hhash->digest_size_byte;

  STM32_CLEAR_BIT(hash_instance->CR, HASH_CR_MDMAT);

  hhash->phase = HASH_PHASE_HMAC_MESSAGE_ENTRY;
  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));

  hhash->hdma_in->p_xfer_cplt_cb = HASH_HMAC_ComputeDMAXferCplt;
  hhash->hdma_in->p_xfer_error_cb = HASH_DMAError;
  hhash->hdma_in->p_xfer_abort_cb = HASH_DMAAbort;

  src_addr = (uint32_t)p_input_buffer;
  size_byte =
    (((hhash->input_size_byte % HASH_WORD_SIZE_BYTE) != 0U)
     ? (hhash->input_size_byte + (HASH_WORD_SIZE_BYTE - (hhash->input_size_byte % HASH_WORD_SIZE_BYTE)))
     : (hhash->input_size_byte));

  STM32_SET_BIT(hash_instance->CR, HASH_CR_DMAE);

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hhash->hdma_in, src_addr,
                                     (uint32_t) &((HASH_TypeDef *)((uint32_t)(hhash)->instance))->DIN, size_byte,
                                     HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
    hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* USE_HAL_HASH_DMA */

/**
  * @brief HASH HMAC update process in polling mode with several input buffers.
  * @param hhash              Pointer to a hal_hash_handle_t structure.
  * @param p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte    length of the input buffer in bytes.
  * @param timeout_ms         Specify timeout value in millisecond.
  * @note  Consecutive calls to HAL_HASH_HMAC_Update() can be used to feed several input buffers back-to-back to
  *        the peripheral that will yield a single HASH signature once all buffers have been entered. Wrap-up of input
  *        buffers feeding and retrieval of digest is done by a call to HAL_HASH_HMAC_Finish().
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       A timeout has occurred.
  * @retval HAL_BUSY          Process is already ongoing.
  * @retval HAL_OK            The update of the given buffer has been processed correctly.
  */
hal_status_t HAL_HASH_HMAC_Update(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                  uint32_t input_size_byte, uint32_t timeout_ms)
{
  uint32_t valid_blocks_nbr    = 0U;
  uint32_t remain_bytes_nbr    = 0U;
  uint32_t remaining_words_nbr = 0U;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);
  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_add_input_buffer == NULL) || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE),
                          HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

  tickstart = HAL_GetTick();

  hhash->p_input_buff = (const uint8_t *)p_add_input_buffer;
  hhash->input_size_byte = input_size_byte;
  hhash->input_data_count_byte = 0U;

  /* Change the number of valid bits in last word of the message */
  STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_NBLW, 0U);

  if (hhash->remain_bytes_number != 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  valid_blocks_nbr = hhash->input_size_byte / (hhash->block_size_byte + HASH_WORD_SIZE_BYTE);
  for (uint32_t index = 0U; index < valid_blocks_nbr; index++)
  {
    HASH_WriteBlock(hhash);
  }

  remain_bytes_nbr = hhash->input_size_byte - hhash->input_data_count_byte;
  remaining_words_nbr = remain_bytes_nbr / HASH_WORD_SIZE_BYTE;

  if (remaining_words_nbr != 0U)
  {
    if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  if ((remain_bytes_nbr % HASH_WORD_SIZE_BYTE) != 0U)
  {
    HASH_SaveRemainingBytes(hhash, remain_bytes_nbr);
  }

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tickstart) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief HASH HMAC update process in IT mode with several input buffers.
  * @param hhash              Pointer to a hal_hash_handle_t structure.
  * @param p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte    length of the input buffer in bytes.
  * @note  Consecutive calls to HAL_HASH_HMAC_Update_IT() can be used to feed several input buffers back-to-back to
  *        the peripheral that will yield a single HASH signature once all buffers have been entered. Wrap-up of input
  *        buffers feeding and retrieval of digest is done by a call to HAL_HASH_HMAC_Finish().
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_BUSY          Process is already ongoing.
  * @retval HAL_OK            The update of the given buffer has been processed correctly.
  */
hal_status_t HAL_HASH_HMAC_Update_IT(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                     uint32_t input_size_byte)
{
  uint32_t remain_bytes_nbr    = 0U;
  uint32_t remaining_words_nbr = 0U;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (p_add_input_buffer == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE),
                          HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

  hhash->p_input_buff          = (const uint8_t *)p_add_input_buffer;
  hhash->input_data_count_byte = 0U;
  hhash->input_size_byte       = input_size_byte;
  hhash->phase                 = HASH_PHASE_HMAC_MESSAGE_ENTRY;

  /* Configure the number of valid bits in last word of the message */
  STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_NBLW, 8U * ((hhash->input_size_byte) % HASH_WORD_SIZE_BYTE));

  if (hhash->remain_bytes_number != 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  /* The number of word expected to be pushed to DIN is set to the expected block size +1 in words ( 0x11) */
  if (hhash->input_size_byte >= (hhash->block_size_byte + HASH_WORD_SIZE_BYTE))
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      hhash->phase = HASH_PHASE_READY;
      hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
      return HAL_ERROR;
    }

    HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN);
  }
  else
  {
    remain_bytes_nbr = hhash->input_size_byte - hhash->input_data_count_byte;
    remaining_words_nbr = remain_bytes_nbr / HASH_WORD_SIZE_BYTE;

    if (remaining_words_nbr != 0U)
    {
      if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
      {
        hhash->phase = HASH_PHASE_READY;
        hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
        return HAL_ERROR;
      }
    }

    if ((remain_bytes_nbr % HASH_WORD_SIZE_BYTE) != 0U)
    {
      HASH_SaveRemainingBytes(hhash, remain_bytes_nbr);
    }
  }

  return HAL_OK;
}

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief HASH HMAC update process in DMA mode with several input buffers.
  * @param hhash              Pointer to a hal_hash_handle_t structure.
  * @param p_add_input_buffer Pointer to the input buffer (buffer to be hashed).
  * @param input_size_byte    length of the input buffer in bytes.
  * @note  Multi-buffer HMAC processing is possible, consecutive calls to HAL_HASH_HMAC_Update_DMA() can be used to
  *        feed several input buffers back-to-back to the peripheral that will yield a single HASH signature once all
  *        buffers have been entered. Wrap-up of input buffers feeding and retrieval of digest is done by a call to
  *        HAL_HASH_HMAC_Finish().
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_BUSY          Process is already ongoing.
  * @retval HAL_OK            The update of the given buffer has been processed correctly.
  */
hal_status_t HAL_HASH_HMAC_Update_DMA(hal_hash_handle_t *hhash, const void *p_add_input_buffer,
                                      uint32_t input_size_byte)
{
  uint32_t tmp_input_addr;
  uint8_t remain_size_byte;
  uint32_t valid_words_nbr;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_add_input_buffer != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (p_add_input_buffer == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((hhash == NULL) || (hhash->hdma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_CHECK_UPDATE_STATE(hhash, global_state,
                          ((uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE),
                          HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  hhash->p_input_buff              = (const uint8_t *)p_add_input_buffer;
  hhash->input_data_count_byte     = 0U;
  hhash->input_size_byte           = input_size_byte;
  hhash->dma_operation_active_flag = 1U;

  hhash->hdma_in->p_xfer_cplt_cb  = HASH_HMAC_UpdateDMAXferCplt;
  hhash->hdma_in->p_xfer_error_cb = HASH_DMAError;
  hhash->hdma_in->p_xfer_abort_cb = HASH_DMAAbort;

  STM32_SET_BIT(hash_instance->CR, HASH_CR_MDMAT);

  /* Process the remaining bytes of the previous buffer */
  if (hhash->remain_bytes_number > 0U)
  {
    HASH_AppendLastIncompleteWord(hhash);
  }

  tmp_input_addr = (uint32_t)hhash->p_input_buff;
  remain_size_byte = (uint8_t)(hhash->input_size_byte % HASH_WORD_SIZE_BYTE);

  /* Store remaining bytes */
  if (remain_size_byte > 0U)
  {
    hhash->input_size_byte = hhash->input_size_byte - remain_size_byte;
    for (uint32_t i = 0U; i < remain_size_byte; i++)
    {
      hhash->remain_bytes[i] = ((uint8_t *)tmp_input_addr)[hhash->input_size_byte + i];
    }
    hhash->remain_bytes_number = remain_size_byte;
  }

  valid_words_nbr = hhash->input_size_byte / HASH_WORD_SIZE_BYTE;
  if (valid_words_nbr > 0U)
  {
    /* Configure the number of valid bits in last word of the message */
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 0U);

    STM32_SET_BIT(hash_instance ->CR, HASH_CR_DMAE);

    if (HAL_DMA_StartPeriphXfer_IT_Opt(hhash->hdma_in,
                                       tmp_input_addr,
                                       (uint32_t) &((HASH_TypeDef *)((uint32_t)(hhash)->instance))->DIN,
                                       hhash->input_size_byte,
                                       HAL_DMA_OPT_IT_NONE) != HAL_OK)
    {
#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
      hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */
      hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
#endif /* USE_HAL_HASH_DMA */

/**
  * @brief Finish HASH update process.
  * @param  hhash                    Pointer to a hal_hash_handle_t structure.
  * @param  p_output_buffer          Pointer to the computed digest.
  * @param  output_buffer_size_byte  Size of the output buffer in bytes provided by the user.
  * @param  p_output_hash_size_byte  Pointer to the size of the digest in bytes.
  * @param  timeout_ms               Specify timeout value in millisecond.
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_TIMEOUT              A timeout has occurred.
  * @retval HAL_ERROR                An error has occurred.
  * @retval HAL_OK                   Hash operation correctly completed and digest available in the output buffer.
  */
hal_status_t HAL_HASH_HMAC_Finish(hal_hash_handle_t *hhash, void *p_output_buffer, uint32_t output_buffer_size_byte,
                                  uint32_t *p_output_hash_size_byte, uint32_t timeout_ms)
{
  uint32_t digest_copy_length = 0U;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_output_buffer != NULL);
  ASSERT_DBG_PARAM(IS_OUT_BUFFER_SIZE_VALID((hhash), (output_buffer_size_byte)));
  ASSERT_DBG_PARAM(p_output_hash_size_byte != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1) \
    || defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((p_output_buffer == NULL) || (p_output_hash_size_byte == NULL)
      || (output_buffer_size_byte < hhash->digest_size_byte) || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM || USE_HAL_SECURE_CHECK_PARAM */

  tickstart = HAL_GetTick();

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  hhash->output_size_byte        = output_buffer_size_byte;
  hhash->p_output_buff           = (uint8_t *)p_output_buffer;
  hhash->p_output_hash_size_byte = p_output_hash_size_byte;
  *p_output_hash_size_byte       = hhash->digest_size_byte;

  if (hhash->remain_bytes_number != 0U)
  {
    /* Configure the number of valid bits in last word of the message */
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, (8U * (uint32_t)hhash->remain_bytes_number));
    HASH_WriteRemainingByte(hhash, (uint8_t *)hhash->remain_bytes, hhash->remain_bytes_number);
  }
  else
  {
    /* Configure the number of valid bits in last word of the message */
    STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 0U);
  }

  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(hash_instance->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_BUSY, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase        = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  hhash->input_data_count_byte = 0U;
  /* Configure the number of valid bits in last word of the Key */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->key_size_byte) % HASH_WORD_SIZE_BYTE));
  HASH_WriteKey(hhash, hhash->p_hmac_key_buff, hhash->key_size_byte);

  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_ERROR;
  }

  if (STM32_IS_BIT_SET(hash_instance->CR, HASH_CR_MDMAT))
  {
    STM32_CLEAR_BIT(((HASH_TypeDef *)((uint32_t)(hhash->instance)))->CR, HASH_CR_MDMAT);
  }

  digest_copy_length = (output_buffer_size_byte < hhash->digest_size_byte) ? output_buffer_size_byte :
                       hhash->digest_size_byte;
  HASH_GetDigestMsg(hhash, (uint8_t *)p_output_buffer, (uint8_t)digest_copy_length);

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tickstart) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  hhash->remain_bytes_number = 0U;
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  hhash->dma_operation_active_flag = 0U;
#endif /* USE_HAL_HASH_DMA */
  hhash->phase        = HASH_PHASE_READY;
  hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group6
  * @{
This section provides functions allowing to:
- Abort in polling mode with HAL_HASH_Abort().
- Abort in IT and DMA mode with HAL_HASH_Abort_IT().
  */
/**
  * @brief Abort HASH/HMAC in polling mode.
  * @param  hhash       Pointer to a hal_hash_handle_t structure.
  * @param  timeout_ms  Specify timeout value in millisecond.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       A timeout has occurred.
  * @retval HAL_ERROR         In case of user timeout.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_HASH_Abort(hal_hash_handle_t *hhash, uint32_t timeout_ms)
{
  uint32_t tick_start;
  hal_hash_state_t tmp_state;
  uint8_t is_hash_compute_active;
  uint8_t is_hash_update_active;
  uint8_t was_hash_compute_active;
  uint8_t was_hash_update_active;

  ASSERT_DBG_PARAM(hhash != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_STATE_SUSPENDED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if ((hhash == NULL) || (timeout_ms == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  tick_start = HAL_GetTick();

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  tmp_state = hhash->global_state;
  is_hash_compute_active = (uint8_t)(tmp_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  is_hash_update_active  = (uint8_t)(tmp_state == HAL_HASH_STATE_UPDATE_ACTIVE);
  hhash->global_state = HAL_HASH_STATE_ABORT;
  HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DC | HAL_HASH_IT_DIN);
  if (tmp_state == HAL_HASH_STATE_SUSPENDED)
  {
    was_hash_compute_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
    was_hash_update_active  = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_UPDATE_ACTIVE);

    HAL_HASH_ClearFlag(hhash, (uint32_t)HAL_HASH_FLAG_DCI | HAL_HASH_FLAG_DINI);
    /* Reset the hash processor core */
    STM32_SET_BIT(hash_instance->CR, HASH_CR_INIT);

    hhash->input_data_count_byte = 0U;
    hhash->input_size_byte       = 0U;
    hhash->suspend_request       = 0U;
    hhash->phase                 = HASH_PHASE_READY;
    hhash->global_state = ((was_hash_compute_active == 1U) || (was_hash_update_active == 1U)) ?
                          HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;

    return HAL_OK;
  }

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  if ((hash_instance->CR & HASH_CR_DMAE) != 0U)
  {
    STM32_CLEAR_BIT(hash_instance->CR, HASH_CR_DMAE);

    if (HAL_DMA_Abort(hhash->hdma_in) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
#endif /* USE_HAL_HASH_DMA */

  if (HASH_WaitUntilTimeout(hhash, timeout_ms, tick_start) != HAL_OK)
  {
    hhash->phase = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
    return HAL_TIMEOUT;
  }

  /* Reset the hash processor core */
  STM32_SET_BIT(hash_instance->CR, HASH_CR_INIT);
  HAL_HASH_ClearFlag(hhash, (uint32_t)HAL_HASH_FLAG_DCI | HAL_HASH_FLAG_DINI);

  hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U)) ?
                        HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Abort HASH/HMAC in interrupt mode.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_HASH_Abort_IT(hal_hash_handle_t *hhash)
{
  hal_hash_state_t tmp_state;
#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  uint8_t is_hash_compute_active;
  uint8_t is_hash_update_active;
#endif /* USE_HAL_HASH_DMA */
  uint8_t was_hash_compute_active;
  uint8_t was_hash_update_active;

  ASSERT_DBG_PARAM(hhash != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_STATE_SUSPENDED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  was_hash_compute_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  was_hash_update_active  = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_UPDATE_ACTIVE);
  tmp_state = hhash->global_state;
#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  is_hash_compute_active = (uint8_t)(tmp_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  is_hash_update_active  = (uint8_t)(tmp_state == HAL_HASH_STATE_UPDATE_ACTIVE);
#endif /* USE_HAL_HASH_DMA */
  hhash->previous_state = tmp_state;
  hhash->global_state = HAL_HASH_STATE_ABORT;

  if (tmp_state == HAL_HASH_STATE_SUSPENDED)
  {
    HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DC | HAL_HASH_IT_DIN);
    HAL_HASH_ClearFlag(hhash, (uint32_t)HAL_HASH_FLAG_DCI | HAL_HASH_FLAG_DINI);
    /* Reset the hash processor core */
    STM32_SET_BIT(hash_instance->CR, HASH_CR_INIT);
    hhash->input_data_count_byte = 0U;
    hhash->input_size_byte       = 0U;
    hhash->suspend_request       = 0U;
    hhash->phase                 = HASH_PHASE_READY;
    hhash->global_state = ((was_hash_compute_active == 1U) || (was_hash_update_active == 1U))
                          ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
    return HAL_OK;
  }

#if defined(USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  if (((hash_instance->CR & HASH_CR_DMAE) != 0U) || (hhash->dma_operation_active_flag == 1U))
  {
    /* Disable the DMA transfer on the HASH side */
    STM32_CLEAR_BIT(hash_instance->CR, HASH_CR_DMAE);
    /* Disable the DMA transmit on the DMA side */
    (void)HAL_DMA_Abort_IT(hhash->hdma_in);
    hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                          ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
  }
#endif /* USE_HAL_HASH_DMA */

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group7
  * @{
This section provides HASH IRQ handler and callback functions.
- HAL_HASH_IRQHandler()         : HASH interrupt request.
- HAL_HASH_InputCpltCallback()  : Input data transfer complete callback.
- HAL_HASH_DigestCpltCallback() : Digest computation complete callback.
- HAL_HASH_ErrorCallback()      : HASH error callback.
- HAL_HASH_SuspendCallback()    : HASH suspend callback.
- HAL_HASH_AbortCallback()      : HASH abort callback.

The compilation define USE_HAL_HASH_REGISTER_CALLBACKS when set to 1 allows the user to register custom callbacks.
- Use the function HAL_HASH_RegisterInputCpltCallback() to register a user input complete callback.
- Use the function HAL_HASH_RegisterDigestComputationCpltCallback() to register a user digest computation
complete callback.
- Use the function HAL_HASH_RegisterErrorCpltCallback() to register a user error callback.
- Use the function HAL_HASH_RegisterSuspendCpltCallback() to register a user suspend callback.
- Use the function HAL_HASH_RegisterAbortCpltCallback() to register a user abort callback.
- Use the function HAL_HASH_SetInDMA() to link the input FIFO HAL DMA handle into the HAL HASH handle.
  */
/**
  * @brief HASH interrupt request.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @note  HAL_HASH_IRQHandler() handles interrupts in HMAC processing as well.
  */
void HAL_HASH_IRQHandler(hal_hash_handle_t *hhash)
{
  uint32_t digest_copy_length = 0U;
  uint32_t itsource;
  uint32_t itflag;
  uint8_t is_hash_compute_active;
  uint8_t is_hash_update_active;

  ASSERT_DBG_PARAM(hhash != NULL);

  const HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  hal_hash_state_t state = hhash->global_state;

  itsource = hash_instance->IMR;
  itflag   = hash_instance->SR;

  if (((itflag & HAL_HASH_FLAG_DCI) == HAL_HASH_FLAG_DCI) && ((itsource & HAL_HASH_IT_DC) == HAL_HASH_IT_DC))
  {
    digest_copy_length = (hhash->output_size_byte > hhash->block_size_byte)
                         ? hhash->block_size_byte : hhash->output_size_byte;
    HASH_GetDigestMsg(hhash, hhash->p_output_buff, (uint8_t)digest_copy_length);
    HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
    hhash->phase        = HASH_PHASE_READY;
    hhash->global_state = (state == HAL_HASH_STATE_COMPUTE_ACTIVE)
                          ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->p_digest_cplt_callback(hhash);
#else
    HAL_HASH_DigestCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
    return;
  }

  if (hhash->suspend_request == 1U)
  {
    hhash->suspend_request = 0U;
    HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
    hhash->previous_state = state;
    hhash->global_state = HAL_HASH_STATE_SUSPENDED;
#if defined(USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->p_suspend_cplt_callback(hhash);
#else
    HAL_HASH_SuspendCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
    return;
  }

  /* If peripheral ready to accept new data */
  if ((itflag & HAL_HASH_FLAG_DINI) == HAL_HASH_FLAG_DINI)
  {
    if ((itsource & HAL_HASH_IT_DIN) == HAL_HASH_IT_DIN)
    {
      if (state == HAL_HASH_STATE_ABORT)
      {
        is_hash_compute_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
        is_hash_update_active  = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_UPDATE_ACTIVE);
        HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
        hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                              ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
        hhash->p_abort_cplt_callback(hhash);
#else
        HAL_HASH_AbortCallback(hhash);
#endif /* (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1) */
        return;
      }

      if ((state == HAL_HASH_STATE_COMPUTE_ACTIVE) || (state == HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE))
      {
        if ((hhash->phase == HASH_PHASE_HMAC_MESSAGE_ENTRY) || (hhash->phase == HASH_PHASE_HMAC_OUTER_KEY_ENTRY)
            || (hhash->phase == HASH_PHASE_PROCESS))
        {
          if (HASH_HMAC_ComputeProcessData_IT(hhash) != HAL_OK)
          {
            HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
            hhash->phase = HASH_PHASE_READY;
            hhash->global_state = (state == HAL_HASH_STATE_COMPUTE_ACTIVE)
                                  ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
            hhash->p_error_callback(hhash);
#else
            HAL_HASH_ErrorCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
            return;
          }
        }

        if (((hhash->input_size_byte == hhash->input_data_count_byte) && (hhash->phase == HASH_PHASE_PROCESS))
            || ((hhash->phase == HASH_PHASE_HMAC_OUTER_KEY_ENTRY)
                && (hhash->key_size_byte == hhash->input_data_count_byte)))
        {
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
          hhash->p_input_cplt_callback(hhash);
#else
          HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
          /* Start the Digest calculation */
          STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);
        }
      }

      if ((state == HAL_HASH_HMAC_STATE_UPDATE_ACTIVE) || (state == HAL_HASH_STATE_UPDATE_ACTIVE))
      {
        if (HASH_HMAC_UpdateProcessData_IT(hhash) != HAL_OK)
        {
          HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
          hhash->phase = HASH_PHASE_READY;
          hhash->global_state = (state == HAL_HASH_STATE_UPDATE_ACTIVE)
                                ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
          hhash->p_error_callback(hhash);
#else
          HAL_HASH_ErrorCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
          return;
        }
#if defined(USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
        hhash->p_input_cplt_callback(hhash);
#else
        HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
      }
    }
  }
}

/**
  * @brief Input data transfer complete callback.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @warning HAL_HASH_InputCpltCallback() is called when the complete input message has been fed to the peripheral.
  *          This API is invoked only when input data are entered in interrupt mode or through DMA mode.
  * @warning In case of HASH or HMAC update DMA feeding case, HAL_HASH_InputCpltCallback() is
  *          called at the end of each buffer feeding to the peripheral.
  */
__WEAK void HAL_HASH_InputCpltCallback(hal_hash_handle_t *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hhash);

  /* WARNING: This function must not be modified, when the callback is needed,
    *          HAL_HASH_InputCpltCallback() can be implemented in the user file.
    */
}

/**
  * @brief Digest computation complete callback.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @note  HAL_HASH_DigestCpltCallback() is used in interrupt mode and is not relevant in DMA mode.
  */
__WEAK void HAL_HASH_DigestCpltCallback(hal_hash_handle_t *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hhash);

  /* WARNING: This function must not be modified, when the callback is needed,
    *          HAL_HASH_DigestCpltCallback() can be implemented in the user file.
    */
}

/**
  * @brief HASH error callback.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  */
__WEAK void HAL_HASH_ErrorCallback(hal_hash_handle_t *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hhash);

  /* WARNING: This function must not be modified, when the callback is needed,
    *          HAL_HASH_ErrorCallback() can be implemented in the user file.
    */
}

/**
  * @brief HASH suspend callback.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  */
__WEAK void HAL_HASH_SuspendCallback(hal_hash_handle_t *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hhash);

  /* WARNING: This function must not be modified, when the callback is needed,
    *          HAL_HASH_SuspendCallback() can be implemented in the user file.
    */
}

/**
  * @brief HASH abort callback.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  */
__WEAK void HAL_HASH_AbortCallback(hal_hash_handle_t *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hhash);

  /* WARNING: This function must not be modified, when the callback is needed,
    *          HAL_HASH_AbortCallback() can be implemented in the user file.
    */
}

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
/**
  * @brief Register the user hash input callback to be used instead of the weak (overridden) predefined callback.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param callback Pointer to the callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register callback completed successfully.
  */
hal_status_t HAL_HASH_RegisterInputCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hhash->p_input_cplt_callback = callback;

  return HAL_OK;
}

/**
  * @brief Register the user hash digest Callback to be used instead of the weak (overridden) predefined callback.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param callback Pointer to the callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register callback completed successfully.
  */
hal_status_t HAL_HASH_RegisterDigestComputationCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hhash->p_digest_cplt_callback = callback;

  return HAL_OK;
}

/**
  * @brief Register the user hash error Callback to be used instead of the weak (overridden) predefined callback.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param callback Pointer to the callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register callback completed successfully.
  */
hal_status_t HAL_HASH_RegisterErrorCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hhash->p_error_callback = callback;

  return HAL_OK;
}

/**
  * @brief Register the user hash suspend Callback to be used instead of the weak (overridden) predefined callback.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param callback Pointer to the Callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register callback completed successfully.
  */
hal_status_t HAL_HASH_RegisterSuspendCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hhash->p_suspend_cplt_callback = callback;

  return HAL_OK;
}

/**
  * @brief Register the user hash abort Callback to be used instead of the weak (overridden) predefined callback.
  * @param hhash    Pointer to a hal_hash_handle_t structure.
  * @param callback Pointer to the Callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register callback completed successfully.
  */
hal_status_t HAL_HASH_RegisterAbortCpltCallback(hal_hash_handle_t *hhash, hal_hash_cb_t callback)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hhash->p_abort_cplt_callback = callback;

  return HAL_OK;
}
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief link/store HAL DMA handle into the HAL HASH handle.
  * @param hhash   Pointer to a hal_hash_handle_t structure.
  * @param hdma_in Pointer to a hal_dma_handle_t.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_HASH_SetInDMA(hal_hash_handle_t *hhash, hal_dma_handle_t *hdma_in)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(hdma_in != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_INIT | (uint32_t)HAL_HASH_STATE_CONFIGURED
                   | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_in == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  hhash->hdma_in = hdma_in;
  hdma_in->p_parent = hhash;

  return HAL_OK;
}
#endif /*USE_HAL_HASH_DMA*/
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group8
  * @{
This section provides HASH suspend and resume functions.
- Use the function HAL_HASH_RequestSuspendComputation() to request an IT computation process suspension.
- Use the function HAL_HASH_RequestSuspendUpdate() to request an IT update process suspension.
- Use the function HAL_HASH_ResumeComputation() to resume the low-priority computation process.
- Use the function HAL_HASH_ResumeUpdate() to resume the low-priority update process.
- Use the function HAL_HASH_SaveContext() to save the context of the suspended process to start another
high priority one.
- Use the function HAL_HASH_RestoreContext() to restore the saved context of the low-priority process.

- Note that these APIs are also valid for HMAC operations.
  */
/**
  * @brief Request suspension for hash interrupt mode computation.
  * @param hhash   Pointer to a hal_hash_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_OK            The HASH processing suspension is well requested.
  */
hal_status_t HAL_HASH_RequestSuspendComputation(hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_COMPUTE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_COMPUTE_ACTIVE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  /* suspension in DMA mode */
  if (HAL_HASH_IsActiveFlag(hhash, HAL_HASH_FLAG_DMA) != 0U)
  {
    if (HASH_SuspendDMA(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
    else
    {
      hhash->previous_state = hhash->global_state;
      hhash->global_state = HAL_HASH_STATE_SUSPENDED;
    }
  }
  else /* suspension when in interruption mode */
  {
#endif /* USE_HAL_HASH_DMA */
    hhash->suspend_request = 1U;
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  }
#endif /* USE_HAL_HASH_DMA */
  return HAL_OK;
}

/**
  * @brief Resumption of the suspended HASH processing computation.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          Process is already ongoing.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_OK            HASH suspended processing is resumed.
  */
hal_status_t HAL_HASH_ResumeComputation(hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_STATE(hhash->global_state, HAL_HASH_STATE_SUSPENDED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_STATE_SUSPENDED, hhash->previous_state);

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  if ((hhash->hdma_in != NULL) && (hhash->dma_operation_active_flag == 1U))
  {
    if (HASH_ResumeDMA(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else
  {
#endif /* USE_HAL_HASH_DMA */
    HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  }
#endif /* USE_HAL_HASH_DMA */

  return HAL_OK;
}

/**
  * @brief  Request suspension for hash interrupt mode processing update.
  * @param  hhash     Pointer to a hal_hash_handle_t structure.
  * @retval HAL_INVALID_PARAM        Invalid parameter.
  * @retval HAL_ERROR An error has been occurred.
  * @retval HAL_OK    The HASH processing suspension is well requested.
  */
hal_status_t HAL_HASH_RequestSuspendUpdate(hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_UPDATE_ACTIVE
                   | (uint32_t)HAL_HASH_HMAC_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  /* suspension in DMA mode */
  if (HAL_HASH_IsActiveFlag(hhash, HAL_HASH_FLAG_DMA) != 0U)
  {
    if (HASH_SuspendDMA(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
    else
    {
      hhash->previous_state = hhash->global_state;
      hhash->global_state = HAL_HASH_STATE_SUSPENDED;
    }
  }
  else /* Suspension in interrupt mode */
  {
#endif /* USE_HAL_HASH_DMA */
    hhash->suspend_request = 1U;
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  }
#endif /* USE_HAL_HASH_DMA */

  return HAL_OK;
}

/**
  * @brief Resumption of the suspended HASH processing update.
  * @param hhash  Pointer to a hal_hash_handle_t structure.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          Process is already ongoing.
  * @retval HAL_ERROR         An error has been occurred.
  * @retval HAL_OK            HASH suspended processing is resumed.
  */
hal_status_t HAL_HASH_ResumeUpdate(hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_STATE(hhash->global_state, HAL_HASH_STATE_SUSPENDED);

#if defined(USE_HAL_SECURE_CHECK_PARAM) && (USE_HAL_SECURE_CHECK_PARAM == 1)
  if (hhash == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_SECURE_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hhash, global_state, HAL_HASH_STATE_SUSPENDED, hhash->previous_state);

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  if ((hhash->hdma_in != NULL) && (hhash->dma_operation_active_flag == 1U))
  {
    if (HASH_ResumeDMA(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else
  {
#endif /* USE_HAL_HASH_DMA */
    HAL_HASH_EnableIT(hhash, HAL_HASH_IT_DIN);
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  }
#endif /* USE_HAL_HASH_DMA */
  return HAL_OK;
}

/**
  * @brief Save parameters of the suspended HASH processing.
  * @param hhash     Pointer to a hal_hash_handle_t structure.
  * @param p_context Pointer to a hal_hash_suspended_context_t structure where to store the parameters of the suspend
  *                  HASH processing.
  */
void HAL_HASH_SaveContext(hal_hash_handle_t *hhash, hal_hash_suspended_context_t *p_context)
{
  uint32_t csr_ptr;
  uint32_t csr_count = 0;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_context != NULL);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_SUSPENDED);

  const HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  uint8_t is_hash_compute_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  uint8_t is_hash_update_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_UPDATE_ACTIVE);

  csr_ptr = (uint32_t)(hash_instance->CSR);

  STM32_UNUSED(hhash);

  p_context->imr_reg = STM32_READ_BIT(hash_instance->IMR, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
  p_context->str_reg = STM32_READ_BIT(hash_instance->STR, HASH_STR_NBLW);
  p_context->cr_reg = STM32_READ_BIT(hash_instance->CR, HASH_CR_DMAE | HASH_CR_DATATYPE | HASH_CR_MODE |
                                     HASH_CR_ALGO | HASH_CR_LKEY | HASH_CR_MDMAT);

  hal_hash_algo_t algorithm = (hal_hash_algo_t)(uint32_t)STM32_READ_BIT(hash_instance->CR, HASH_CR_ALGO);
  uint8_t mode = (uint8_t)STM32_READ_BIT(hash_instance->CR, HASH_CR_MODE);

  switch (algorithm)
  {
    case HAL_HASH_ALGO_SHA1:
    case HAL_HASH_ALGO_SHA256:
      csr_count = (mode == 0U) ? HASH_SHA1_SHA2256_CSR_REGISTER_NUMBER : HASH_HMAC_SHA1_SHA2256_CSR_REGISTER_NUMBER;
      break;
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
    case HAL_HASH_ALGO_SHA384:
    case HAL_HASH_ALGO_SHA512:
      csr_count = (mode == 0U) ? HASH_SHA2384_SHA2512_CSR_REGISTER_NUMBER :
                  HASH_HMAC_SHA2384_SHA2512_CSR_REGISTER_NUMBER;
      break;
#endif /* HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */
    default:
      csr_count = HASH_CSR_REGISTERS_NUMBER;
      break;
  }

  /* Save all CSRs registers */
  for (uint32_t idx = 0U; idx < csr_count; idx++)
  {
    p_context->csr_reg[idx] = *(uint32_t *)csr_ptr;
    csr_ptr += HASH_WORD_SIZE_BYTE;
  }

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  p_context->hdma_in                 = hhash->hdma_in;
#endif /* USE_HAL_HASH_DMA */
  p_context->input_data_count_byte   = hhash->input_data_count_byte;
  p_context->input_size_byte         = hhash->input_size_byte;
  p_context->output_size_byte        = hhash->output_size_byte;
  p_context->digest_size_byte        = hhash->digest_size_byte;
  p_context->key_size_byte           = hhash->key_size_byte;
  p_context->phase                   = hhash->phase;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  p_context->p_abort_cplt_callback   = hhash->p_abort_cplt_callback;
  p_context->p_digest_cplt_callback  = hhash->p_digest_cplt_callback;
  p_context->p_error_callback        = hhash->p_error_callback;
  p_context->p_input_buff            = hhash->p_input_buff;
  p_context->p_output_buff           = hhash->p_output_buff;
  p_context->p_input_cplt_callback   = hhash->p_input_cplt_callback;
  p_context->p_suspend_cplt_callback = hhash->p_suspend_cplt_callback;
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
  p_context->p_hmac_key_buff         = hhash->p_hmac_key_buff;
  p_context->p_hmac_key_saved        = hhash->p_hmac_key_saved;
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  p_context->dma_operation_active_flag = hhash->dma_operation_active_flag;
#endif /* USE_HAL_HASH_DMA */
  p_context->previous_state           = hhash->global_state;

  hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                        ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
}

/**
  * @brief Restore the HASH context in case of processing resumption.
  * @param hhash     Pointer to a hal_hash_handle_t structure.
  * @param p_context Pointer to a hal_hash_suspended_context_t structure where to store the parameters of the suspend
  *                  HASH processing.
  */
void HAL_HASH_RestoreContext(hal_hash_handle_t *hhash, const hal_hash_suspended_context_t *p_context)
{
  uint32_t csr_ptr;
  uint32_t csr_count = 0U;

  ASSERT_DBG_PARAM(hhash != NULL);
  ASSERT_DBG_PARAM(p_context != NULL);
  ASSERT_DBG_PARAM(p_context->previous_state == HAL_HASH_STATE_SUSPENDED);

  ASSERT_DBG_STATE(hhash->global_state, (uint32_t)HAL_HASH_STATE_CONFIGURED | (uint32_t)HAL_HASH_HMAC_STATE_CONFIGURED);

  STM32_UNUSED(hhash);
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  csr_ptr = (uint32_t)(hash_instance->CSR);

  STM32_WRITE_REG(hash_instance->IMR, p_context->imr_reg);
  STM32_WRITE_REG(hash_instance->STR, p_context->str_reg);
  STM32_WRITE_REG(hash_instance->CR, p_context->cr_reg);

  STM32_SET_BIT(hash_instance->CR, HASH_CR_INIT);

  hal_hash_algo_t algorithm = (hal_hash_algo_t)(uint32_t)STM32_READ_BIT(hash_instance->CR, HASH_CR_ALGO);
  uint8_t mode = (uint8_t)STM32_READ_BIT(hash_instance->CR, HASH_CR_MODE);

  switch (algorithm)
  {
    case HAL_HASH_ALGO_SHA1:
    case HAL_HASH_ALGO_SHA256:
      csr_count = (mode == 0U) ? HASH_SHA1_SHA2256_CSR_REGISTER_NUMBER : HASH_HMAC_SHA1_SHA2256_CSR_REGISTER_NUMBER;
      break;
#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
    case HAL_HASH_ALGO_SHA384:
    case HAL_HASH_ALGO_SHA512:
      csr_count = (mode == 0U) ? HASH_SHA2384_SHA2512_CSR_REGISTER_NUMBER :
                  HASH_HMAC_SHA2384_SHA2512_CSR_REGISTER_NUMBER;
      break;
#endif /* HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */
    default:
      csr_count = HASH_CSR_REGISTERS_NUMBER;
      break;
  }

  /* Restore all CSR registers */
  for (uint32_t idx = 0U; idx < csr_count; idx++)
  {
    STM32_WRITE_REG((*(uint32_t *)(csr_ptr)), p_context->csr_reg[idx]);
    csr_ptr += HASH_WORD_SIZE_BYTE;
  }

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  hhash->hdma_in                 = p_context->hdma_in;
#endif /* USE_HAL_HASH_DMA */
  hhash->input_data_count_byte   = p_context->input_data_count_byte;
  hhash->input_size_byte         = p_context->input_size_byte;
  hhash->output_size_byte        = p_context->output_size_byte;
  hhash->digest_size_byte        = p_context->digest_size_byte;
  hhash->key_size_byte           = p_context->key_size_byte;
  hhash->phase                   = p_context->phase;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_abort_cplt_callback   = p_context->p_abort_cplt_callback;
  hhash->p_digest_cplt_callback  = p_context->p_digest_cplt_callback;
  hhash->p_error_callback        = p_context->p_error_callback;
  hhash->p_input_buff            = p_context->p_input_buff;
  hhash->p_output_buff           = p_context->p_output_buff;
  hhash->p_input_cplt_callback   = p_context->p_input_cplt_callback;
  hhash->p_suspend_cplt_callback = p_context->p_suspend_cplt_callback;
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
  hhash->p_hmac_key_buff         = p_context->p_hmac_key_buff;
  hhash->p_hmac_key_saved        = p_context->p_hmac_key_saved;
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
  hhash->dma_operation_active_flag  = p_context->dma_operation_active_flag;
#endif /* USE_HAL_HASH_DMA */

  hhash->global_state = HAL_HASH_STATE_SUSPENDED;
}
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group9
  * @{
This subsection provides a set of functions to get the HASH error information and state:
- Use the function HAL_HASH_GetState() to get the HASH global state.
- Use the function HAL_HASH_GetLastErrorCodes() to get the last error codes.
- Use the function HAL_HASH_SetUserData() to set the user data.
- Use the function HAL_HASH_GetUserData() to get the user data.
  */

/**
  * @brief Return the HASH handle state.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @retval HAL HASH global state.
  */
hal_hash_state_t HAL_HASH_GetState(const hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);

  return hhash->global_state;
}

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
/**
  * @brief Return the HASH handle error code.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @retval HASH last error Codes.
  * @note When the return is 0xAAAAAAAAU this is a HAL_INVALID_PARAM
  */
uint32_t HAL_HASH_GetLastErrorCodes(const hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);

  return hhash->last_error_codes;
}
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

#if defined (USE_HAL_HASH_USER_DATA) && (USE_HAL_HASH_USER_DATA == 1)
/**
  * @brief Store the user data pointer into the HASH handle.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @param p_user_data Pointer to the user data.
  */
void HAL_HASH_SetUserData(hal_hash_handle_t *hhash, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hhash != NULL);

  hhash->p_user_data = p_user_data;
}

/**
  * @brief Retrieve the user data from the HASH handle.
  * @param hhash Pointer to a hal_hash_handle_t structure.
  * @retval Pointer to the user data.
  */
const void *HAL_HASH_GetUserData(const hal_hash_handle_t *hhash)
{
  ASSERT_DBG_PARAM(hhash != NULL);

  return (hhash->p_user_data);
}
#endif /* USE_HAL_HASH_USER_DATA */
/**
  * @}
  */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @addtogroup HASH_Private_Functions
  * @{
  */
#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/**
  * @brief DMA HASH Compute Input Data transfer completion callback.
  * @param hdma DMA handle.
  */
static void HASH_ComputeDMAXferCplt(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)(hdma->p_parent);
  uint32_t digest_copy_length = 0U;
  /* Call Input data transfer complete callback */
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_input_cplt_callback(hhash);
#else
  HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase        = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->p_error_callback(hhash);
#else
    HAL_HASH_ErrorCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
    return;
  }

  digest_copy_length = (hhash->output_size_byte > hhash->block_size_byte)
                       ? hhash->block_size_byte : hhash->output_size_byte;
  HASH_GetDigestMsg(hhash, hhash->p_output_buff, (uint8_t)digest_copy_length);


  hhash->phase = HASH_PHASE_READY;
  hhash->dma_operation_active_flag = 0U;
  hhash->global_state              = HAL_HASH_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_digest_cplt_callback(hhash);
#else
  HAL_HASH_DigestCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}

/**
  * @brief DMA HASH HMAC Compute Input Data transfer completion callback.
  * @param hdma DMA handle.
  */
static void HASH_HMAC_ComputeDMAXferCplt(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)(hdma->p_parent);
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  uint32_t digest_copy_length = 0U;

  /* Call Input data transfer complete callback */
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_input_cplt_callback(hhash);
#else
  HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */

  hhash->phase = HASH_PHASE_HMAC_OUTER_KEY_ENTRY;
  /* Configure the number of valid bits in last word of the Key */
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 8U * ((hhash->key_size_byte) % HASH_WORD_SIZE_BYTE));
  HASH_WriteKey(hhash, hhash->p_hmac_key_saved, hhash->key_size_byte);

  /* Start the Key padding then the Digest calculation */
  STM32_SET_BIT(hash_instance->STR, HASH_STR_DCAL);

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DCI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    hhash->phase        = HASH_PHASE_READY;
    hhash->global_state = HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->p_error_callback(hhash);
#else
    HAL_HASH_ErrorCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
    return;
  }

  digest_copy_length = (hhash->output_size_byte < hhash->digest_size_byte) ? hhash->output_size_byte :
                       hhash->digest_size_byte;
  HASH_GetDigestMsg(hhash, hhash->p_output_buff, (uint8_t)digest_copy_length);
  hhash->phase                    = HASH_PHASE_READY;
  hhash->global_state             = HAL_HASH_HMAC_STATE_CONFIGURED;

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_digest_cplt_callback(hhash);
#else
  HAL_HASH_DigestCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}

/**
  * @brief DMA HASH Update Input Data transfer completion callback.
  * @param hdma DMA handle.
  */
static void HASH_UpdateDMAXferCplt(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)(hdma->p_parent);

  /**
    * Keep the global state to HAL_HASH_STATE_UPDATE_ACTIVE as up to HAL_HASH_Finish() to set
    * the state bck to HAL_HASH_STATE_CONFIGURED
    */
  hhash->global_state = HAL_HASH_STATE_UPDATE_ACTIVE;

#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_input_cplt_callback(hhash);
#else
  HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}

/**
  * @brief DMA HASH HMAC Update Input Data transfer completion callback.
  * @param hdma DMA handle.
  */
static void HASH_HMAC_UpdateDMAXferCplt(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)(hdma->p_parent);

  /**
    * Keep the global state to HAL_HASH_HMAC_STATE_UPDATE_ACTIVE as up to HAL_HASH_HMAC_Finish() to set
    * the state bck to HAL_HASH_HMAC_STATE_CONFIGURED
    */
  hhash->global_state = HAL_HASH_HMAC_STATE_UPDATE_ACTIVE;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_input_cplt_callback(hhash);
#else
  HAL_HASH_InputCpltCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}

/**
  * @brief DMA HASH Abort callback.
  * @param hdma DMA handle.
  */
static void HASH_DMAAbort(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)((hal_dma_handle_t *)hdma)->p_parent;
  uint8_t is_hash_compute_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  uint8_t is_hash_update_active = (uint8_t)(hhash->previous_state == HAL_HASH_STATE_UPDATE_ACTIVE);

  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->CR, HASH_CR_INIT);
  hhash->input_data_count_byte = 0U;
  hhash->input_size_byte = 0U;
  hhash->suspend_request = 0U;
  hhash->phase           = HASH_PHASE_READY;
  hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                        ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_abort_cplt_callback(hhash);
#else
  HAL_HASH_AbortCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}

/**
  * @brief DMA HASH communication error callback.
  * @param hdma DMA handle.
  */
static void HASH_DMAError(hal_dma_handle_t *hdma)
{
  hal_hash_handle_t *hhash = (hal_hash_handle_t *)(hdma->p_parent);
  uint8_t is_hash_compute_active = (uint8_t)(hhash->global_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  uint8_t is_hash_update_active = (uint8_t)(hhash->global_state == HAL_HASH_STATE_UPDATE_ACTIVE);

#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
  hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

  hhash->global_state = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                        ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined (USE_HAL_HASH_REGISTER_CALLBACKS) && (USE_HAL_HASH_REGISTER_CALLBACKS == 1)
  hhash->p_error_callback(hhash);
#else
  HAL_HASH_ErrorCallback(hhash);
#endif /* USE_HAL_HASH_REGISTER_CALLBACKS */
}
#endif /* USE_HAL_HASH_DMA */

/**
  * @brief Feed the input key buffer to the HASH peripheral in polling.
  * @param hhash         Pointer to a hal_hash_handle_t structure.
  * @param p_key         Pointer to input buffer.
  * @param key_size_byte Specifies the size of input buffer in bytes.
  */
static void HASH_WriteKey(hal_hash_handle_t *hhash, const uint8_t *p_key, uint32_t key_size_byte)
{
  uint32_t buffer_counter;
  volatile uint32_t inputaddr = (uint32_t) p_key;
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  for (buffer_counter = 0U; buffer_counter < (key_size_byte / HASH_WORD_SIZE_BYTE); buffer_counter++)
  {
    /* Write input data 4 bytes at a time */
    hash_instance->DIN = *(uint32_t *)inputaddr;
    inputaddr += 4U;
    hhash->input_data_count_byte += 4U;
  }

  if ((key_size_byte % HASH_WORD_SIZE_BYTE) != 0U)
  {
    HASH_WriteRemainingByte(hhash, (uint8_t *)inputaddr, key_size_byte);
  }
}

/**
  * @brief Retrieve the message digest.
  * @param hhash Pointer to a HASH handle structure.
  * @param p_msg_digest Pointer to the computed digest.
  * @param input_size_byte message digest size in bytes.
  */
static void HASH_GetDigestMsg(const hal_hash_handle_t *hhash, uint8_t *p_msg_digest, uint8_t input_size_byte)
{
  uint32_t i;
  uint8_t tmp_input_size_byte = input_size_byte;

  if (tmp_input_size_byte > hhash->digest_size_byte)
  {
    tmp_input_size_byte = (uint8_t)hhash->digest_size_byte;
  }
  uint32_t input_size_word = ((uint32_t)tmp_input_size_byte) >> 2U;
  const HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  for (i = 0U; i < input_size_word; i++)
  {
    uint32_t val = __REV(hash_instance->HR[i]);
    p_msg_digest[(i * 4U) + 0U] = (uint8_t)(val & 0xFFU);
    p_msg_digest[(i * 4U) + 1U] = (uint8_t)((val >> 8U) & 0xFFU);
    p_msg_digest[(i * 4U) + 2U] = (uint8_t)((val >> 16U) & 0xFFU);
    p_msg_digest[(i * 4U) + 3U] = (uint8_t)((val >> 24U) & 0xFFU);
  }
}

/**
  * @brief Handle HASH processing timeout.
  * @param hhash        Pointer to a HASH handle structure.
  * @param timeout_ms   Specify timeout value in millisecond.
  * @param tick_start   Specify HAL tick start value in milliseconds.
  * @retval HAL_TIMEOUT A timeout has occurred.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t HASH_WaitUntilTimeout(hal_hash_handle_t *hhash, uint32_t timeout_ms, uint32_t tick_start)
{
  STM32_UNUSED(hhash);

  /* Check for the timeout */
  if (timeout_ms != HAL_MAX_DELAY)
  {
    if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief Set HASH HMAC mode.
  * @param hhash          Pointer to a HASH handle structure.
  * @param key_size_bytes Specifies the key size in bytes.
  */
static void HASH_SetHMACMode(const hal_hash_handle_t *hhash, uint32_t key_size_bytes)
{
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  uint32_t cr_value = HASH_CR_MODE | HASH_CR_INIT;

  if (key_size_bytes  > hhash->block_size_byte)
  {
    cr_value |= HASH_CR_LKEY;
  }

  STM32_MODIFY_REG(hash_instance->CR, HASH_CR_LKEY | HASH_CR_MODE | HASH_CR_INIT, cr_value);
}

#if defined (USE_HAL_HASH_DMA) && (USE_HAL_HASH_DMA == 1)
/** @brief suspend the DMA.
  * @param hhash specifies the HASH handle.
  * @retval HAL_TIMEOUT A timeout has occurred.
  * @retval HAL_ERROR   An error has been occurred.
  * @retval HAL_OK      DMA suspended.
  */
static hal_status_t HASH_SuspendDMA(hal_hash_handle_t *hhash)
{
  uint32_t remaining_words; /* remaining number in of source block to be transferred. */
  uint32_t size_in_words;   /* number in word of source block to be transferred.*/

  /* Clear the DMAE bit to disable the DMA interface */
  STM32_CLEAR_BIT(HASH_GET_INSTANCE(hhash)->CR, HASH_CR_DMAE);

  /* Wait until the last DMA transfer is complete (DMAS = 0 in the HASH_SR register) */
  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DMA, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  /* At this point, DMA interface is disabled and no transfer is on-going */
  /* Retrieve from the DMA handle how many words remain to be written */
  /* DMA3 used, DMA_CBR1_BNDT in bytes, DMA_CSR_FIFOL in words */
  remaining_words = ((((DMA_Channel_TypeDef *)hhash->hdma_in->instance)->CBR1) & DMA_CBR1_BNDT) / HASH_WORD_SIZE_BYTE;
#if defined (DMA_CSR_FIFOL)
  remaining_words += (((((DMA_Channel_TypeDef *)hhash->hdma_in->instance)->CSR) & DMA_CSR_FIFOL) >> DMA_CSR_FIFOL_Pos) /
                     HASH_WORD_SIZE_BYTE;
#endif /* DMA_CSR_FIFOL */
  /* Disable DMA channel */
  /* Note that the Abort function will
    - Clear the transfer error flags
    - Unlock
    - Set the State
    */
  if (HAL_DMA_Abort(hhash->hdma_in) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait until the hash processor is ready (no block is being processed), that is wait for DINIS=1 in HASH_SR */
  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_DINI, 0U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if (HAL_HASH_IsActiveFlag(hhash, HAL_HASH_FLAG_DCI) != 0U)
  {
    return HAL_ERROR;
  }

  if (remaining_words == 0U)
  {
    /* All the DMA transfer is actually done. Suspension occurred at the very end
    of the transfer. Either the digest computation is about to start (HASH case)
    or processing is about to move from one step to another (HMAC case).
    In both cases, the processing can't be suspended at this point. It is
    safer to
    - retrieve the low priority block digest before starting the high
    priority block processing (HASH case)
    - re-attempt a new suspension (HMAC case)
    */
    return HAL_ERROR;
  }
  else
  {
    /* Compute how many words were supposed to be transferred by DMA */
    size_in_words = ((hhash->input_size_byte + 3U) / HASH_WORD_SIZE_BYTE);

    /* Accordingly, update the input pointer that points at the next word to be transferred to the Peripheral by DMA */
    hhash->p_input_buff += HASH_WORD_SIZE_BYTE * (size_in_words - remaining_words);

    /* And store in input_data_count_byte the remaining size to transfer (in bytes) */
    hhash->input_size_byte = HASH_WORD_SIZE_BYTE * remaining_words;

    return HAL_OK;
  }
}

/** @brief Resume the DMA.
  * @param hhash Pointer to a HASH handle structure.
  * @retval HAL_TIMEOUT A timeout has occurred.
  * @retval HAL_ERROR   An error has been occurred.
  * @retval HAL_OK      DMA resumed.
  */
static hal_status_t HASH_ResumeDMA(hal_hash_handle_t *hhash)
{
  uint32_t tmp_input_size;
  uint8_t is_hash_compute_active = (uint8_t)(hhash->global_state == HAL_HASH_STATE_COMPUTE_ACTIVE);
  uint8_t is_hash_update_active  = (uint8_t)(hhash->global_state == HAL_HASH_STATE_UPDATE_ACTIVE);

  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);
  STM32_MODIFY_REG(hash_instance->STR, HASH_STR_NBLW, 0U);

  tmp_input_size = (hhash->input_size_byte + (HASH_WORD_SIZE_BYTE  - 1U)) & ~(HASH_WORD_SIZE_BYTE  - 1U);

  STM32_SET_BIT(hash_instance->CR, HASH_CR_DMAE);

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hhash->hdma_in,
                                     (uint32_t)hhash->p_input_buff,
                                     (uint32_t) &((HASH_TypeDef *)((uint32_t)(hhash)->instance))->DIN,
                                     tmp_input_size,
                                     HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hhash->phase           = HASH_PHASE_READY;
    hhash->global_state    = ((is_hash_compute_active == 1U) || (is_hash_update_active == 1U))
                             ? HAL_HASH_STATE_CONFIGURED : HAL_HASH_HMAC_STATE_CONFIGURED;
#if defined(USE_HAL_HASH_GET_LAST_ERRORS) && (USE_HAL_HASH_GET_LAST_ERRORS == 1)
    hhash->last_error_codes |= HAL_HASH_ERROR_DMA;
#endif /* USE_HAL_HASH_GET_LAST_ERRORS */

    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* USE_HAL_HASH_DMA */

/** @brief Get the digest length in byte.
  * @param algorithm specifies the HASH algorithm.
  * @retval digest length.
  */
static uint32_t HASH_GetDigestLength(hal_hash_algo_t algorithm)
{
  /* Digest size lookup table for normalized algorithm index */
  static const uint8_t hash_digest_size_lut[16U] =
  {
    /* 0b0000 */ HAL_DIGEST_SIZE_20B,    /* 0: SHA1         */

    /* 0b0001 */ 0U,                     /* 1: unsupported  */

    /* 0b0010 */ HAL_DIGEST_SIZE_28B,    /* 2: SHA224       */

    /* 0b0011 */ HAL_DIGEST_SIZE_32B,    /* 3: SHA256       */

    /* 0b0100 */ 0U,                     /* 4: unsupported  */

    /* 0b0101 */ 0U,                     /* 5: unsupported  */

    /* 0b0110 */ 0U,                     /* 6: unsupported  */

    /* 0b0111 */ 0U,                     /* 7: unsupported  */

    /* 0b1000 */ 0U,                     /* 8: unsupported  */

    /* 0b1001 */ 0U,                     /* 9: unsupported  */

    /* 0b1010 */ 0U,                     /* 10: unsupported */

    /* 0b1011 */ 0U,                     /* 11: unsupported */

#if defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
    /* 0b1100 */ HAL_DIGEST_SIZE_48B,    /* 12: SHA384      */
#endif /* HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */

    /* 0b1101 */ HAL_DIGEST_SIZE_28B,    /* 13: SHA512224   */

    /* 0b1110 */ HAL_DIGEST_SIZE_32B,    /* 14: SHA512256   */

#if defined(HASH_CR_ALGO_0) && defined(HASH_CR_ALGO_1) && defined(HASH_CR_ALGO_2) && defined(HASH_CR_ALGO_3)
    /* 0b1111 */ HAL_DIGEST_SIZE_64B     /* 15: SHA512      */
#endif /* HASH_CR_ALGO_0 | HASH_CR_ALGO_1 | HASH_CR_ALGO_2 | HASH_CR_ALGO_3 */
  };

  uint32_t index = ((uint32_t)algorithm & HASH_CR_ALGO) >> HASH_CR_ALGO_Pos;

  return (uint32_t)hash_digest_size_lut[index];
}

/**
  * @brief Feed the remaining byte to HASH DIN register then the padding is done automatically using
  *        the information provided in NBLW.
  * @param hhash             Pointer to a hal_hash_handle_t structure.
  * @param p_in_buff         Pointer to input buffer.
  * @param input_size_bytes  The number of bytes within the last block
  */
static void HASH_WriteRemainingByte(hal_hash_handle_t *hhash, const uint8_t *p_in_buff, uint32_t input_size_bytes)
{
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  hash_instance->DIN = *(uint32_t *)((uint32_t)p_in_buff);
  hhash->input_data_count_byte += input_size_bytes % HASH_WORD_SIZE_BYTE;
}

/**
  * @brief Process One block of data in polling mode.
  * @param hhash      Pointer to a hal_hash_handle_t structure.
  */
static void HASH_WriteBlock(hal_hash_handle_t *hhash)
{
  volatile uint32_t inputaddr = (uint32_t)hhash->p_input_buff;
  uint32_t buffer_counter = 0U;

  for (; buffer_counter < hhash->block_size_byte; buffer_counter += 4U)
  {
    /* Write input data 4 bytes at a time */
    HASH_GET_INSTANCE(hhash)->DIN = *(uint32_t *)inputaddr;
    inputaddr += 4U;
    hhash->p_input_buff += 4U;
    hhash->input_data_count_byte += 4U;
  }
}

/**
  * @brief Process One block of data in IT mode.
  * @param hhash      Pointer to a hal_hash_handle_t structure.
  * @retval HAL_ERROR An error has been occurred.
  * @retval HAL_OK    Operation completed.
  */
static hal_status_t HASH_WriteBlock_IT(hal_hash_handle_t *hhash)
{
  HASH_TypeDef *hash_instance = HASH_GET_INSTANCE(hhash);

  volatile uint32_t inputaddr = (uint32_t)hhash->p_input_buff;

  /* Number of bytes to enter in HASH FIFO to trigger partial HASH */
  uint32_t NbrOfWordsExpected = (((STM32_READ_REG(hash_instance->SR) >> HASH_SR_NBWE_Pos)) << 2U);

  for (uint32_t buffer_counter = 0U; buffer_counter < NbrOfWordsExpected; buffer_counter += 4U)
  {
    if (hhash->input_data_count_byte == (hhash->input_size_byte - (hhash->input_size_byte % 4U)))
    {
      break;
    }
    /* Write input data 4 bytes at a time */
    hash_instance->DIN = *(uint32_t *)inputaddr;
    inputaddr += 4U;
    hhash->p_input_buff += 4U;
    hhash->input_data_count_byte += 4U;
  }

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_BUSY, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN);
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Write Last block (remaining valid words less than to block size or remaining bytes) to HASH_DIN register.
  * @param hhash      Pointer to a HASH handle structure.
  * @retval HAL_ERROR An error has been occurred.
  * @retval HAL_OK    Operation completed.
  */
static hal_status_t HASH_WriteLastBlock(hal_hash_handle_t *hhash)
{
  uint32_t remaining_words_nbr = (hhash->input_size_byte - hhash->input_data_count_byte) / HASH_WORD_SIZE_BYTE;
  uint32_t remaining_bytes_nbr = (hhash->input_size_byte - hhash->input_data_count_byte) % HASH_WORD_SIZE_BYTE;
  if (remaining_words_nbr != 0U)
  {
    if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (remaining_bytes_nbr != 0U)
  {
    HASH_WriteRemainingByte(hhash, hhash->p_input_buff, remaining_bytes_nbr);
  }

  return HAL_OK;
}

/**
  * @brief Feed the remaining words for the incompleted block to the HASH_DIN register.
  * @param hhash               Pointer to a hal_hash_handle_t structure.
  * @param remaining_words_nbr Number of remaining word to write to HASH_DIN register.
  * @retval HAL_TIMEOUT HASH digest computation exceeds timeout
  * @retval HAL_OK      HASH digest computation is completed
  */
static hal_status_t HASH_WriteIncompleteBlock(hal_hash_handle_t *hhash, uint32_t remaining_words_nbr)
{
  volatile uint32_t inputaddr = (uint32_t)hhash->p_input_buff;

  for (uint32_t index = 0U; index < remaining_words_nbr; index++)
  {
    /* Write input data 4 bytes at a time */
    HASH_GET_INSTANCE(hhash)->DIN = (*(uint32_t *)inputaddr);
    inputaddr += 4U;
    hhash->p_input_buff += 4U;
    hhash->input_data_count_byte += 4U;
  }

  if (HASH_WaitOnFlag(hhash, HAL_HASH_FLAG_BUSY, 1U, HASH_MAX_BLOCK_PROCESS_CYCLE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Handle HASH hardware block timeout when waiting for a flag to be raised.
  * @param hhash        Pointer to a HASH handle structure.
  * @param flag         Specifies the HASH flag to check.
  * @param flag_state   The Flag status (SET or RESET).
  * @param wait_cycles  Specify wait cycles value in millisecond.
  * @retval HAL_ERROR   An error has been occurred.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t HASH_WaitOnFlag(hal_hash_handle_t *hhash, uint32_t flag, uint32_t flag_state,
                                    uint32_t wait_cycles)
{
  uint32_t count = wait_cycles;

  /* Simple polling loop with cycle counter, approximate CPU cycles with iterations */
  while ((HAL_HASH_IsActiveFlag(hhash, flag) == flag_state) && (count > 0U))
  {
    count--;
  }

  if (count == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Saves the remaining bytes (less than a word) from the input buffer.
  * @param hhash            Pointer to a HASH handle structure.
  * @param remain_bytes_nbr remain bytes number to be saved for next HASH/HMAC update or finish operation.
  */
static void HASH_SaveRemainingBytes(hal_hash_handle_t *hhash, uint32_t remain_bytes_nbr)
{
  hhash->remain_bytes_number = (uint8_t)(remain_bytes_nbr % HASH_WORD_SIZE_BYTE);
  /* Save remaining bytes */
  for (uint32_t i = 0U; i < hhash->remain_bytes_number; i++)
  {
    hhash->remain_bytes[i] = *(const uint8_t *)(hhash->p_input_buff + i);
  }
}

/**
  * @brief Append and processes the last incomplete word from the current input buffer.
  * @param hhash Pointer to a HASH handle structure.
  * @note This function combines any remaining bytes with the next bytes in the buffer to form a complete word
  */
static void HASH_AppendLastIncompleteWord(hal_hash_handle_t *hhash)
{
  /* Process additional bytes */
  uint32_t tmp = 0U;
  for (uint32_t i = 0U; i < hhash->remain_bytes_number; i++)
  {
    tmp |= ((uint32_t)hhash->remain_bytes[i] << ((uint32_t)(i * 8U)));
  }

  /* Process the first bytes of the buffer to complete the 4 bytes */
  for (uint32_t i = 0U; (i < (4U - (uint32_t)hhash->remain_bytes_number)) && (i < hhash->input_size_byte); i++)
  {
    tmp |= (*(uint32_t *)(uint32_t)hhash->p_input_buff) << ((i + (uint32_t)hhash->remain_bytes_number) * 8U);
    hhash->p_input_buff += 1U;
  }
  HASH_GET_INSTANCE(hhash)->DIN = tmp;

  /* Adjust buffer size after processing first bytes */
  hhash->input_size_byte -= (hhash->input_size_byte > (4U - (uint32_t)hhash->remain_bytes_number)) ?
                            (4U - (uint32_t)hhash->remain_bytes_number) : hhash->input_size_byte;
  hhash->remain_bytes_number = 0U;
}

/**
  * @brief HASH/HMAC process Data in IT mode: Each Block write to HASH_DIN, then Write the last incomplete block.
  * @param hhash Pointer to a HASH handle structure.
  * @retval HAL_ERROR An error has been occurred.
  * @retval HAL_OK    Operation completed.
  */
static hal_status_t HASH_HMAC_ComputeProcessData_IT(hal_hash_handle_t *hhash)
{
  uint32_t uncompleted_last_block;

  if ((hhash->input_size_byte - hhash->input_data_count_byte) >= hhash->block_size_byte)
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  uncompleted_last_block = (hhash->input_size_byte - hhash->input_data_count_byte) / HASH_WORD_SIZE_BYTE;
  if (uncompleted_last_block < (hhash->block_size_byte / HASH_WORD_SIZE_BYTE))
  {
    if (HASH_WriteLastBlock(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if ((hhash->phase == HASH_PHASE_HMAC_MESSAGE_ENTRY) && (hhash->input_size_byte == hhash->input_data_count_byte))
  {
    HASH_HMAC_SwitchToStep3(hhash);
  }

  return HAL_OK;
}

/**
  * @brief Handles HASH/HMAC update phase in interrupt mode.
  *        This function processes input data during the update phase for both
  *        HASH and HMAC operations when called from an interrupt context. It:
  *        - Writes full data blocks using HASH_WriteBlock_IT()
  *        - Handles the last incomplete block (if any)
  *        - Saves remaining bytes for later processing
  * @param hhash Pointer to a HASH handle structure.
  * @retval HAL_ERROR An error has been occurred.
  * @retval HAL_OK    Operation completed.
  */
static hal_status_t HASH_HMAC_UpdateProcessData_IT(hal_hash_handle_t *hhash)
{
  uint32_t remaining_words_nbr;

  if ((hhash->input_size_byte - hhash->input_data_count_byte) >= hhash->block_size_byte)
  {
    if (HASH_WriteBlock_IT(hhash) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else /* Still remaining words: Last block less than to block size length */
  {
    hhash->remain_bytes_number = (uint8_t)(hhash->input_size_byte - hhash->input_data_count_byte);
    if (hhash->remain_bytes_number != 0U)
    {
      remaining_words_nbr = (uint32_t)hhash->remain_bytes_number / HASH_WORD_SIZE_BYTE;

      if (remaining_words_nbr != 0U)
      {
        if (HASH_WriteIncompleteBlock(hhash, remaining_words_nbr) != HAL_OK)
        {
          return HAL_ERROR;
        }
      }

      HASH_SaveRemainingBytes(hhash, hhash->remain_bytes_number);
    }

    HAL_HASH_DisableIT(hhash, HAL_HASH_IT_DIN | HAL_HASH_IT_DC);
  }

  return HAL_OK;
}

/**
  * @brief  Switch HMAC processing from STEP 2 to STEP 3.
  *         It finalizes padding of the inner hash and reconfigures
  *         the context to process the HMAC key (outer hash).
  * @param  hhash Pointer to a HASH handle structure.
  */
static void HASH_HMAC_SwitchToStep3(hal_hash_handle_t *hhash)
{
  /* Start the message padding then the Digest calculation */
  STM32_SET_BIT(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_DCAL);

  STM32_MODIFY_REG(HASH_GET_INSTANCE(hhash)->STR, HASH_STR_NBLW,
                   8U * ((hhash->key_size_byte) % HASH_WORD_SIZE_BYTE));

  hhash->p_input_buff          = hhash->p_hmac_key_buff;
  hhash->input_size_byte       = hhash->key_size_byte;
  hhash->input_data_count_byte = 0U;
  hhash->phase                 = HASH_PHASE_HMAC_OUTER_KEY_ENTRY;
}
/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_HAL_HASH_MODULE */

#endif /* HASH */
/**
  * @}
  */
