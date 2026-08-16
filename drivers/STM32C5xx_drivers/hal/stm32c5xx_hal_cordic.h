/**
  ******************************************************************************
  * @file    stm32c5xx_hal_cordic.h
  * @brief   Header file of CORDIC HAL module.
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
#ifndef STM32C5XX_HAL_CORDIC_H
#define STM32C5XX_HAL_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_cordic.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(CORDIC)

/** @defgroup CORDIC CORDIC
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Constants HAL CORDIC Constants
  * @{
  */

#if defined (USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
/** @defgroup CORDIC_Error_Code CORDIC Error code
  * @{
  */
#define HAL_CORDIC_ERROR_NONE 0U         /*!< No error  */
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
#define HAL_CORDIC_ERROR_DMA  (1UL << 31U) /*!< DMA error */
#endif /* USE_HAL_CORDIC_DMA */
/**
  * @}
  */
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */

/** @defgroup CORDIC_DMA_optional_Interrupt CORDIC DMA optional interrupt
  * @{
  */
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
#define HAL_CORDIC_OPT_DMA_NONE         0UL         /*!< All optional IT are disabled     */
#define HAL_CORDIC_OPT_DMA_IT_HALF_CPLT (1UL << 1U) /*!< Enable optional IT half complete */
#endif /* USE_HAL_CORDIC_DMA */
/**
  * @}
  */

/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Types HAL CORDIC Types
  * @{
  */

/** @defgroup CORDIC_Exported_Types_Group1 Enumerations
  * @{
  */

/**
  * @brief HAL CORDIC instance.
  */
typedef enum
{
  HAL_CORDIC = CORDIC_BASE /*!< HAL CORDIC Peripheral instance */
} hal_cordic_t;

/**
  * @brief  CORDIC HAL state structure definition.
  */
typedef enum
{
  HAL_CORDIC_STATE_RESET  = 0UL,          /*!< CORDIC not yet initialized or disabled          */
  HAL_CORDIC_STATE_INIT   = (1UL << 31U), /*!< CORDIC initialized but not yet configured       */
  HAL_CORDIC_STATE_IDLE   = (1UL << 30U), /*!< CORDIC initialized  and a global config applied */
  HAL_CORDIC_STATE_ACTIVE = (1UL << 29U), /*!< CORDIC internal process is ongoing              */
  HAL_CORDIC_STATE_ABORT  = (1UL << 28U)  /*!< CORDIC internal process is aborted              */
} hal_cordic_state_t;

/**
  * @brief CORDIC Function.
  */
typedef enum
{
  HAL_CORDIC_FUNCTION_COSINE      = LL_CORDIC_FUNCTION_COSINE,      /*!< Cosine                */
  HAL_CORDIC_FUNCTION_SINE        = LL_CORDIC_FUNCTION_SINE,        /*!< Sine                  */
  HAL_CORDIC_FUNCTION_PHASE       = LL_CORDIC_FUNCTION_PHASE,       /*!< Phase                 */
  HAL_CORDIC_FUNCTION_MODULUS     = LL_CORDIC_FUNCTION_MODULUS,     /*!< Modulus               */
  HAL_CORDIC_FUNCTION_ARCTANGENT  = LL_CORDIC_FUNCTION_ARCTANGENT,  /*!< Arctangent            */
  HAL_CORDIC_FUNCTION_HCOSINE     = LL_CORDIC_FUNCTION_HCOSINE,     /*!< Hyperbolic Cosine     */
  HAL_CORDIC_FUNCTION_HSINE       = LL_CORDIC_FUNCTION_HSINE,       /*!< Hyperbolic Sine       */
  HAL_CORDIC_FUNCTION_HARCTANGENT = LL_CORDIC_FUNCTION_HARCTANGENT, /*!< Hyperbolic Arctangent */
  HAL_CORDIC_FUNCTION_NATURAL_LOG = LL_CORDIC_FUNCTION_NATURALLOG,  /*!< Natural Logarithm     */
  HAL_CORDIC_FUNCTION_SQUARE_ROOT = LL_CORDIC_FUNCTION_SQUAREROOT   /*!< Square Root           */
} hal_cordic_function_t;

/**
  * @brief CORDIC precision in cycle count (Number of iterations /4).
  */
typedef enum
{
  HAL_CORDIC_PRECISION_1_CYCLE  = LL_CORDIC_PRECISION_1_CYCLE,  /*!< 1*4 iterations  */
  HAL_CORDIC_PRECISION_2_CYCLE  = LL_CORDIC_PRECISION_2_CYCLE,  /*!< 2*4 iterations  */
  HAL_CORDIC_PRECISION_3_CYCLE  = LL_CORDIC_PRECISION_3_CYCLE,  /*!< 3*4 iterations  */
  HAL_CORDIC_PRECISION_4_CYCLE  = LL_CORDIC_PRECISION_4_CYCLE,  /*!< 4*4 iterations  */
  HAL_CORDIC_PRECISION_5_CYCLE  = LL_CORDIC_PRECISION_5_CYCLE,  /*!< 5*4 iterations  */
  HAL_CORDIC_PRECISION_6_CYCLE  = LL_CORDIC_PRECISION_6_CYCLE,  /*!< 6*4 iterations  */
  HAL_CORDIC_PRECISION_7_CYCLE  = LL_CORDIC_PRECISION_7_CYCLE,  /*!< 7*4 iterations  */
  HAL_CORDIC_PRECISION_8_CYCLE  = LL_CORDIC_PRECISION_8_CYCLE,  /*!< 8*4 iterations  */
  HAL_CORDIC_PRECISION_9_CYCLE  = LL_CORDIC_PRECISION_9_CYCLE,  /*!< 9*4 iterations  */
  HAL_CORDIC_PRECISION_10_CYCLE = LL_CORDIC_PRECISION_10_CYCLE, /*!< 10*4 iterations */
  HAL_CORDIC_PRECISION_11_CYCLE = LL_CORDIC_PRECISION_11_CYCLE, /*!< 11*4 iterations */
  HAL_CORDIC_PRECISION_12_CYCLE = LL_CORDIC_PRECISION_12_CYCLE, /*!< 12*4 iterations */
  HAL_CORDIC_PRECISION_13_CYCLE = LL_CORDIC_PRECISION_13_CYCLE, /*!< 13*4 iterations */
  HAL_CORDIC_PRECISION_14_CYCLE = LL_CORDIC_PRECISION_14_CYCLE, /*!< 14*4 iterations */
  HAL_CORDIC_PRECISION_15_CYCLE = LL_CORDIC_PRECISION_15_CYCLE  /*!< 15*4 iterations */
} hal_cordic_precision_t;

/**
  * @brief CORDIC scaling factor.
  * Scale factor value 'n' implies that the input data are multiplied
  * by a factor of 2exp(-n), and/or the output data need to be multiplied by 2exp(n).
  */
typedef enum
{
  HAL_CORDIC_SCALING_FACTOR_0 = LL_CORDIC_SCALING_FACTOR_0, /*!< Scaling factor - Arguments Multiplied by 2^0 */
  HAL_CORDIC_SCALING_FACTOR_1 = LL_CORDIC_SCALING_FACTOR_1, /*!< Scaling factor - Arguments Multiplied by 2^1 */
  HAL_CORDIC_SCALING_FACTOR_2 = LL_CORDIC_SCALING_FACTOR_2, /*!< Scaling factor - Arguments Multiplied by 2^2 */
  HAL_CORDIC_SCALING_FACTOR_3 = LL_CORDIC_SCALING_FACTOR_3, /*!< Scaling factor - Arguments Multiplied by 2^3 */
  HAL_CORDIC_SCALING_FACTOR_4 = LL_CORDIC_SCALING_FACTOR_4, /*!< Scaling factor - Arguments Multiplied by 2^4 */
  HAL_CORDIC_SCALING_FACTOR_5 = LL_CORDIC_SCALING_FACTOR_5, /*!< Scaling factor - Arguments Multiplied by 2^5 */
  HAL_CORDIC_SCALING_FACTOR_6 = LL_CORDIC_SCALING_FACTOR_6, /*!< Scaling factor - Arguments Multiplied by 2^6 */
  HAL_CORDIC_SCALING_FACTOR_7 = LL_CORDIC_SCALING_FACTOR_7  /*!< Scaling factor - Arguments Multiplied by 2^7 */
} hal_cordic_scaling_factor_t;

/**
  * @brief CORDIC Number of 32-bit arguments required for one computation.
  */
typedef enum
{
  HAL_CORDIC_NB_ARG_1 = LL_CORDIC_NBWRITE_1, /*!< One 32-bit write for either one data input in q1.31 format
                                                  or two 16-bit data input in q1.15 format packed in one 32-bit */
  HAL_CORDIC_NB_ARG_2 = LL_CORDIC_NBWRITE_2  /*!< Two 32-bit writes for two data inputs in q1.31 format         */
} hal_cordic_nb_arg_t;

/**
  * @brief CORDIC number of 32-bit results required after one computation.
  */
typedef enum
{
  HAL_CORDIC_NB_RESULT_1 = LL_CORDIC_NBREAD_1, /*!< One 32-bit read for either one data output in q1.31 format
                                                    or two 16-bit data outputs in q1.15 format packed in one 32-bit */
  HAL_CORDIC_NB_RESULT_2 = LL_CORDIC_NBREAD_2  /*!< Two 32-bit reads for two 32-bit data output in q1.31 format     */
} hal_cordic_nb_result_t;

/**
  * @brief CORDIC input data width.
  */
typedef enum
{
  HAL_CORDIC_IN_WIDTH_32_BIT = LL_CORDIC_INWIDTH_32_BIT, /*!< 32-bit input data width (q1.31 format) */
  HAL_CORDIC_IN_WIDTH_16_BIT = LL_CORDIC_INWIDTH_16_BIT  /*!< 16-bit input data width (q1.15 format) */
} hal_cordic_in_width_t;

/**
  * @brief CORDIC output data width.
  */
typedef enum
{
  HAL_CORDIC_OUT_WIDTH_32_BIT = LL_CORDIC_OUTWIDTH_32_BIT, /*!< 32-bit output data width (q1.31 format) */
  HAL_CORDIC_OUT_WIDTH_16_BIT = LL_CORDIC_OUTWIDTH_16_BIT  /*!< 16-bit output data width (q1.15 format) */
} hal_cordic_out_width_t;
/**
  * @}
  */

/** @defgroup CORDIC_Exported_Types_Group2 CORDIC Configuration Structure
  * @{
  */

/**
  * @brief CORDIC data buffer descriptor.
  */
typedef struct
{
  int32_t  *p_data;   /*!< data pointer              */
  uint32_t size_word; /*!< data buffer size in words. */
} hal_cordic_buffer_desc_t;

/**
  * @brief CORDIC function global configuration structure definition.
  */
typedef struct
{
  hal_cordic_function_t       function;       /*!< Function to apply (COSINE, SINE, PHASE, MODULOUS, ...)        */
  hal_cordic_scaling_factor_t scaling_factor; /*!< Scaling factor to apply to the arguments and/or results.      */
  hal_cordic_in_width_t       in_width;      /*!< Width of input data (16-bit or 32-bit).                        */
  hal_cordic_out_width_t      out_width;     /*!< Width of output data (16-bit or 32-bit).                       */
  hal_cordic_precision_t      precision;      /*!< Precision required (number of iterations).                    */
  hal_cordic_nb_arg_t         nb_arg;         /*!< Number of arguments (one 32-bit value or two 32-bit values)   */
  hal_cordic_nb_result_t      nb_result;      /*!< Number of results (one 32-bit two 16_bit values or two 32-bit */
} hal_cordic_config_t;

/**
  * @}
  */

/** @defgroup CORDIC_Exported_Types_Group3 CORDIC Handle Structure
  * @{
  */
typedef struct hal_cordic_handle_s hal_cordic_handle_t; /*!< CORDIC handle structure type */

#if defined (USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
typedef void (*hal_cordic_cb_t)(hal_cordic_handle_t *hcordic); /*!< Pointer to CORDIC callback function */
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */

/**
  * @brief HAL CORDIC handle structure definition.
  */
struct hal_cordic_handle_s
{
  hal_cordic_t                instance;             /*!< CORDIC instance                                     */
  const int32_t               *p_input_buffer;      /*!< Pointer to CORDIC input data buffer                 */
  int32_t                     *p_output_buffer;     /*!< Pointer to CORDIC output data buffer                */
  uint32_t                    computation_nb;       /*!< Remaining number of computation to do               */
  uint32_t                    result_nb;            /*!< Remaining number of result to get                   */
  void(*p_wr_arg)(const hal_cordic_handle_t *hcordic, const int32_t **pp_input_buffer);
  void(*p_rd_result)(const hal_cordic_handle_t *hcordic, int32_t **pp_output_buffer);

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
  hal_dma_handle_t            *p_dma_in;            /*!< CORDIC peripheral input data DMA handle parameters  */
  hal_dma_handle_t            *p_dma_out;           /*!< CORDIC peripheral output data DMA handle parameters */
#endif /* USE_HAL_CORDIC_DMA */

  volatile hal_cordic_state_t global_state;         /*!< CORDIC state                                        */

#if defined (USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hal_cordic_cb_t             p_calculate_cplt_cb;  /*!< CORDIC calculate complete callback                  */
  hal_cordic_cb_t             p_write_cplt_cb;      /*!< CORDIC write complete callback                      */
  hal_cordic_cb_t             p_abort_cplt_cb;      /*!< CORDIC abort complete callback                      */
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
  hal_cordic_cb_t             p_write_half_cplt_cb; /*!< DMA write half complete callback                    */
  hal_cordic_cb_t             p_read_half_cplt_cb;  /*!< DMA read half complete callback                     */
#endif /* USE_HAL_CORDIC_DMA */
  hal_cordic_cb_t             p_error_cb;           /*!< CORDIC error callback                               */
#endif /* (USE_HAL_CORDIC_REGISTER_CALLBACKS) */

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  volatile uint32_t           last_error_codes;     /*!< CORDIC peripheral error code                        */
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */

#if defined (USE_HAL_CORDIC_USER_DATA) && (USE_HAL_CORDIC_USER_DATA == 1)
  const void                  *p_user_data;         /*!< User data pointer                                   */
#endif /* USE_HAL_CORDIC_USER_DATA */
};
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions HAL CORDIC functions
  * @{
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup CORDIC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_CORDIC_Init(hal_cordic_handle_t *hcordic, hal_cordic_t instance);
void HAL_CORDIC_DeInit(hal_cordic_handle_t *hcordic);
/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions_Group2 Configuration functions
  * @{
  */
/* Global configuration for Cordic functions */
hal_status_t HAL_CORDIC_SetConfig(hal_cordic_handle_t *hcordic, const hal_cordic_config_t *p_config);
void HAL_CORDIC_GetConfig(const hal_cordic_handle_t *hcordic, hal_cordic_config_t *p_config);

hal_status_t HAL_CORDIC_SetFunction(hal_cordic_handle_t *hcordic, hal_cordic_function_t function);
hal_cordic_function_t HAL_CORDIC_GetFunction(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetInputWidth(const hal_cordic_handle_t *hcordic, const hal_cordic_in_width_t input_width);
hal_cordic_in_width_t HAL_CORDIC_GetInputWidth(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetOutputWidth(const hal_cordic_handle_t *hcordic, const hal_cordic_out_width_t output_width);
hal_cordic_out_width_t HAL_CORDIC_GetOutputWidth(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetNumberArguments(const hal_cordic_handle_t *hcordic, const hal_cordic_nb_arg_t nb_argument);
hal_cordic_nb_arg_t HAL_CORDIC_GetNumberArguments(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetNumberResults(const hal_cordic_handle_t *hcordic, const hal_cordic_nb_result_t nb_result);
hal_cordic_nb_result_t HAL_CORDIC_GetNumberResults(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetPrecision(const hal_cordic_handle_t *hcordic, const hal_cordic_precision_t precision);
hal_cordic_precision_t HAL_CORDIC_GetPrecision(const hal_cordic_handle_t *hcordic);

hal_status_t HAL_CORDIC_SetScalingFactor(const hal_cordic_handle_t *hcordic,
                                         const hal_cordic_scaling_factor_t scaling_factor);
hal_cordic_scaling_factor_t HAL_CORDIC_GetScalingFactor(const hal_cordic_handle_t *hcordic);

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
hal_status_t HAL_CORDIC_SetWriteDMA(hal_cordic_handle_t *hcordic, hal_dma_handle_t *hdma_wr);
hal_status_t HAL_CORDIC_SetReadDMA(hal_cordic_handle_t *hcordic, hal_dma_handle_t *hdma_rd);
#endif /* USE_HAL_CORDIC_DMA */

volatile uint32_t *HAL_CORDIC_GetWriteAddress(const hal_cordic_handle_t *hcordic);
volatile uint32_t *HAL_CORDIC_GetReadAddress(const hal_cordic_handle_t *hcordic);

/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions_Group4 Process functions
  * @{
  */
hal_status_t HAL_CORDIC_Calculate(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                  const hal_cordic_buffer_desc_t *p_out_buff, uint32_t timeout_ms);
hal_status_t HAL_CORDIC_Calculate_IT(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                     const hal_cordic_buffer_desc_t *p_out_buff);
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
hal_status_t HAL_CORDIC_Calculate_DMA(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                      hal_cordic_buffer_desc_t *p_out_buff);
#endif /* USE_HAL_CORDIC_DMA */

hal_status_t HAL_CORDIC_CalculateZeroOverhead(hal_cordic_handle_t *hcordic,
                                              const hal_cordic_buffer_desc_t *p_in_buff,
                                              const hal_cordic_buffer_desc_t *p_out_buff, uint32_t timeout_ms);

hal_status_t HAL_CORDIC_Abort(hal_cordic_handle_t *hcordic);
hal_status_t HAL_CORDIC_Abort_IT(hal_cordic_handle_t *hcordic);

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
hal_status_t HAL_CORDIC_Write_DMA(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff);
hal_status_t HAL_CORDIC_Write_DMA_opt(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                      uint32_t opt_it);
hal_status_t HAL_CORDIC_Read_DMA(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff);
hal_status_t HAL_CORDIC_Read_DMA_opt(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff,
                                     uint32_t opt_it);
#endif /* USE_HAL_CORDIC_DMA */
/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions_Group5 IRQHandler and Callbacks functions
  * @{
  */
void HAL_CORDIC_IRQHandler(hal_cordic_handle_t *hcordic);
void HAL_CORDIC_ErrorCallback(hal_cordic_handle_t *hcordic);
void HAL_CORDIC_CalculateCpltCallback(hal_cordic_handle_t *hcordic);
void HAL_CORDIC_WriteCpltCallback(hal_cordic_handle_t *hcordic);
void HAL_CORDIC_AbortCpltCallback(hal_cordic_handle_t *hcordic);
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
void HAL_CORDIC_WriteHalfCpltCallback(hal_cordic_handle_t *hcordic);
void HAL_CORDIC_ReadHalfCpltCallback(hal_cordic_handle_t *hcordic);
#endif /* USE_HAL_CORDIC_DMA */

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
/* Callbacks Register functions */
hal_status_t HAL_CORDIC_RegisterErrorCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
hal_status_t HAL_CORDIC_RegisterCalculateCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
hal_status_t HAL_CORDIC_RegisterWriteCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
hal_status_t HAL_CORDIC_RegisterAbortCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
hal_status_t HAL_CORDIC_RegisterWriteHalfCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
hal_status_t HAL_CORDIC_RegisterReadHalfCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback);
#endif /* USE_HAL_CORDIC_DMA */
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
/**
  * @}
  */

#if defined (USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
/** @defgroup CORDIC_Exported_Functions_Group6 Error function
  * @{
  */
uint32_t HAL_CORDIC_GetLastErrorCodes(const hal_cordic_handle_t *hcordic);
/**
  * @}
  */
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

/** @defgroup CORDIC_Exported_Functions_Group7 State function
  * @{
  */
hal_cordic_state_t HAL_CORDIC_GetState(const hal_cordic_handle_t *hcordic);
/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions_Group8 User Data and retrieve functions
  * @{
  */
#if defined (USE_HAL_CORDIC_USER_DATA) && (USE_HAL_CORDIC_USER_DATA == 1)
void HAL_CORDIC_SetUserData(hal_cordic_handle_t *hcordic, const void *p_user_data);
const void *HAL_CORDIC_GetUserData(const hal_cordic_handle_t *hcordic);
#endif /* USE_HAL_CORDIC_USER_DATA */

hal_cordic_t HAL_CORDIC_GetInstance(const hal_cordic_handle_t *hcordic);
CORDIC_TypeDef *HAL_CORDIC_GetLLInstance(const hal_cordic_handle_t *hcordic);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* CORDIC */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_CORDIC_H */
