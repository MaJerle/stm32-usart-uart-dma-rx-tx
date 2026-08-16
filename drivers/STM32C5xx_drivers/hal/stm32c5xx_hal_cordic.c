/**
  ******************************************************************************
  * @file    stm32c5xx_hal_cordic.c
  * @brief   CORDIC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the CORDIC peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Callback functions
  *           + IRQ handler management
  *           + Peripheral state and error
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
#if defined(CORDIC)
#if defined(USE_HAL_CORDIC_MODULE) && (USE_HAL_CORDIC_MODULE == 1)

/** @addtogroup CORDIC
  * @{
  */
/** @defgroup CORDIC_Introduction CORDIC Introduction
  * @{

  - The Hardware Abstraction Layer CORDIC provides an efficient interface to the hardware CORDIC coprocessor,
    which accelerates mathematical functions commonly used in motor control, metering, signal processing, and other
    embedded applications.

  - It supports a variety of functions including sine, cosine, hyperbolic sine and cosine, arctangent, modulus,
    square root, and natural logarithm, enabling flexible use across different computational needs.

  - The CORDIC HAL allows configurable precision, scaling factors, and flexible settings for the number and width
    of input arguments and output results to optimize performance and data handling.

  - The HAL CORDIC provides calculation modes including polling, zero-overhead, interrupt, and DMA to accommodate
    different application requirements.

  - These features simplify the integration of complex mathematical calculations in embedded systems, improving
    performance and offloading the processor to handle other processes.

  */
/**
  * @}
  */

/** @defgroup CORDIC_How_To_Use CORDIC How To Use
  * @{

# How to use the CORDIC HAL module driver

1. Declare a hal_cordic_handle_t handle structure and initialize the CORDIC driver with a CORDIC instance using
   HAL_CORDIC_Init().
   HAL_CORDIC_Init() enables the CORDIC clock when USE_HAL_CORDIC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.
2. Configure the low-level hardware (CLOCK, NVIC, DMA...):
  - Enable the CORDICx interface clock unless you have set USE_HAL_CORDIC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO; in this
    case, HAL_CORDIC_Init() enables the clock.
  - NVIC configuration if you need to use interrupt processing:
    - Configure the CORDIC interrupt priority using HAL_CORTEX_NVIC_SetPriority().
    - Enable the CORDIC IRQ handler using HAL_CORTEX_NVIC_EnableIRQ().
    - In the CORDIC IRQ handler, call HAL_CORDIC_IRQHandler().
  - DMA configuration if you need to use DMA processing:
    - Enable the DMAx interface clock.
    - Configure and enable two DMA channels, one for managing data transfer from memory to peripheral (input channel)
      and another channel for managing data transfer from peripheral to memory (output channel).
    - Associate the initialized DMA handle to the CORDIC DMA handle.
    - Configure the priority and enable the NVIC for the transfer complete interrupt on the two DMA channels by calling
      the HAL_CORTEX_NVIC_SetPriority() and HAL_CORTEX_NVIC_EnableIRQ() functions.

3. Configure the minimum configuration needed for the CORDIC driver by calling HAL_CORDIC_SetConfig().
  - This function configures:
    - Processing functions: Cosine, Sine, Phase, Modulus, Arctangent, Hyperbolic cosine, Hyperbolic sine,
      Hyperbolic arctangent, Natural log, Square root.
    - Scaling factor: 1 to 2exp(-7).
    - Width of input data: 32-bit input data width (Q1.31 format) or 16-bit input data width (Q1.15 format).
    - Width of output data: 32-bit output data width (Q1.31 format) or 16-bit output data width (Q1.15 format).
    - Number of 32-bit writes expected for one calculation: One 32-bit write or two 32-bit writes.
    - Number of 32-bit reads expected after one calculation: One 32-bit read or two 32-bit reads.
    - Precision: 1 to 15 cycles for calculation (the more cycles, the better precision).

4. Operation modes:
  - Polling mode operations:

    - The processing API is a blocking function, that is, it processes the data and waits until the output results are
      available.
      Perform this operation by calling HAL_CORDIC_Calculate().

  - Zero-overhead mode operations:

    - The processing API is a blocking function, that is, it writes data to process and reads the result immediately.
      Any attempt to read the result inserts a bus wait state until the
      calculation is completed. Perform this operation by calling HAL_CORDIC_CalculateZeroOverhead().

  - Interrupt mode operations:

    - The processing API is a non-blocking function, and an interrupt is generated whenever the output results are
      available. The result of the calculation is read in the interrupt service routine. However, it is slower than
      directly reading the result or polling the flag because of interrupt handling delays. Perform this operation by
      calling HAL_CORDIC_Calculate_IT().
    - When all the computations are done, HAL_CORDIC_CalculateCpltCallback() is executed. This callback is a weak
      function and can be overridden by the user or by registering a callback function.
    - In case of error during computation, the HAL_CORDIC_ErrorCallback() callback is executed. This callback is a weak
      function and can be overridden by the user or by registering a callback function.

  - DMA mode operations:

    - The processing API is a non-blocking function and allows offloading the CPU. If both channels are enabled,
      the CORDIC can autonomously perform repeated calculations on a buffer of data without any CPU access.
      Perform this operation by calling HAL_CORDIC_Calculate_DMA(). This function operates with
      a DMA channel In and a DMA channel out only.
    - The current DMA transfer can be cancelled using the HAL_CORDIC_Abort() or HAL_CORDIC_Abort_IT() functions.
    - When half of all the data are written, HAL_CORDIC_WriteHalfCpltCallback() is executed. This callback is a weak
      function and can be overridden by the user or by registering a callback function.
    - When half of all the results are read, HAL_CORDIC_ReadHalfCpltCallback() is executed. This callback
      is a weak function and can be overridden by the user or by registering a callback function.
    - When all the computations are done, HAL_CORDIC_CalculateCpltCallback() is executed. This callback is a weak
      function and can be overridden by the user or by registering a callback function.
    - In case of error during computation, the HAL_CORDIC_ErrorCallback() callback is executed. This callback is a weak
      function and can be overridden by the user or by registering a callback function.

5. Write and read operations directly driven by another peripheral (Timer, ADC, DAC, etc) are available through:
    - Use the HAL_CORDIC_GetWriteAddress() and HAL_CORDIC_GetReadAddress() functions to get the addresses of the
      arguments and results as required by the user application.
    - Use HAL_CORDIC_Write_DMA() to manage the DMA write stream to the CORDIC peripheral, while the CORDIC customer
      peripheral (Timer, ADC, DAC, etc) is responsible for managing the corresponding DMA read stream through its
      dedicated DMA channel.
    - Use HAL_CORDIC_Read_DMA() to manage the DMA read stream to the CORDIC peripheral, while the CORDIC customer
      peripheral (Timer, ADC, DAC, etc) is responsible for managing the corresponding DMA write stream through its
      dedicated DMA channel.

6. Call HAL_CORDIC_DeInit() to deinitialize the CORDIC peripheral.

7. Call HAL_CORDIC_GetInstance() or HAL_CORDIC_GetLLInstance() to retrieve the CORDIC instance.

8. Callback definition in Interrupt or DMA mode:

  When the preprocessor directive USE_HAL_CORDIC_REGISTER_CALLBACKS is set to 1, the user can dynamically configure the
  driver callbacks using their own method:

Callback name               | Default value                      | Callback registration function
----------------------------| -----------------------------------| --------------------------------------------
ErrorCallback               | HAL_CORDIC_ErrorCallback           | HAL_CORDIC_RegisterErrorCallback()
CalculationCpltCallback     | HAL_CORDIC_CalculateCpltCallback   | HAL_CORDIC_RegisterCalculateCpltCallback()
WriteDataCpltCallback       | HAL_CORDIC_WriteCpltCallback       | HAL_CORDIC_RegisterWriteCpltCallback()
AbortCpltCallback           | HAL_CORDIC_AbortCpltCallback       | HAL_CORDIC_RegisterAbortCpltCallback()
WriteHalfCpltCallback       | HAL_CORDIC_WriteHalfCpltCallback   | HAL_CORDIC_RegisterWriteHalfCpltCallback()
ReadHalfCpltCallback        | HAL_CORDIC_ReadHalfCpltCallback    | HAL_CORDIC_RegisterReadHalfCpltCallback()

  To unregister a callback, register the default callback.

  By default, after HAL_CORDIC_Init(), and when the state is \ref HAL_CORDIC_STATE_INIT, all callbacks are set to
  the corresponding weak functions.

  Callbacks can be registered in HAL_CORDIC_STATE_INIT or HAL_CORDIC_STATE_IDLE states only.

  When the preprocessor directive USE_HAL_CORDIC_REGISTER_CALLBACKS is set to 0 or undefined, the callback registration
  feature is not available and all callbacks are set to the corresponding weak functions.
  */
/**
  * @}
  */

/** @defgroup CORDIC_Configuration_Table CORDIC Configuration Table
  * @{

# Configuration inside the CORDIC driver

Software configuration defined in stm32c5xx_hal_conf.h:
preprocessor flags                |   Default value   | Comment
--------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_CORDIC_MODULE             |         1         | Enable HAL CORDIC driver module
USE_HAL_CORDIC_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_CORDIC_DMA                |         1         | Enable DMA code inside CORDIC
USE_HAL_CHECK_PARAM               |         0         | Enable runtime parameter check
USE_HAL_CORDIC_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE       |         0         | Enable atomicity of process state check
USE_HAL_CORDIC_USER_DATA          |         0         | Add a user data inside HAL CORDIC handle
USE_HAL_CORDIC_GET_LAST_ERRORS    |         0         | Enable retrieval of last processes error codes

Software configuration defined in preprocessor environment:
preprocessor flags                |   Default value   | Comment
--------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM              |    Not defined    | Enable check param for HAL and LL
USE_ASSERT_DBG_STATE              |    Not defined    | Enable check state for HAL

  */
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup CORDIC_Private_Constants CORDIC Private Constants
  * @{
  */
#define CORDIC_ARGUMENT1              0UL                       /*!< Default argument 1             */
#define CORDIC_ARGUMENT2              0x7FFFFFFFUL              /*!< Default argument 2             */
#define CORDIC_FLAG_NOT_ACTIVE        0UL                       /*!< Flag not active                */
/**
  * @}
  */
/* Private macros -------------------------------------------------------------*/
/** @defgroup CORDIC_Private_Macros CORDIC Private Macros
  * @{
  */
/**
  * @brief Retrieve the CORDIC HW cmsis instance from the hal handle.
  */
#define CORDIC_GET_INSTANCE(handle) \
  ((CORDIC_TypeDef *)((uint32_t)((handle)->instance)))

/**
  * @brief Verify the CORDIC function.
  * @param  function function.
  * @retval SET   function is valid.
  * @retval RESET function is invalid.
  */
#define IS_CORDIC_FUNCTION(function) (((function) == HAL_CORDIC_FUNCTION_COSINE)          \
                                      || ((function) == HAL_CORDIC_FUNCTION_SINE)         \
                                      || ((function) == HAL_CORDIC_FUNCTION_PHASE)        \
                                      || ((function) == HAL_CORDIC_FUNCTION_MODULUS)      \
                                      || ((function) == HAL_CORDIC_FUNCTION_ARCTANGENT)   \
                                      || ((function) == HAL_CORDIC_FUNCTION_HCOSINE)      \
                                      || ((function) == HAL_CORDIC_FUNCTION_HSINE)        \
                                      || ((function) == HAL_CORDIC_FUNCTION_HARCTANGENT)  \
                                      || ((function) == HAL_CORDIC_FUNCTION_NATURAL_LOG)  \
                                      || ((function) == HAL_CORDIC_FUNCTION_SQUARE_ROOT))

/**
  * @brief Verify the CORDIC precision.
  * @param  precision precision value.
  * @retval SET   precision is valid.
  * @retval RESET precision is invalid.
  */
#define IS_CORDIC_PRECISION(precision) (((precision) == HAL_CORDIC_PRECISION_1_CYCLE)      \
                                        || ((precision) == HAL_CORDIC_PRECISION_2_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_3_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_4_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_5_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_6_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_7_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_8_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_9_CYCLE)  \
                                        || ((precision) == HAL_CORDIC_PRECISION_10_CYCLE) \
                                        || ((precision) == HAL_CORDIC_PRECISION_11_CYCLE) \
                                        || ((precision) == HAL_CORDIC_PRECISION_12_CYCLE) \
                                        || ((precision) == HAL_CORDIC_PRECISION_13_CYCLE) \
                                        || ((precision) == HAL_CORDIC_PRECISION_14_CYCLE) \
                                        || ((precision) == HAL_CORDIC_PRECISION_15_CYCLE))

/**
  * @brief Verify the CORDIC scaling factor.
  * @param  scaling_factor scaling factor value.
  * @retval SET   scaling_factor is valid.
  * @retval RESET scaling_factor is invalid.
  */
#define IS_CORDIC_SCALING(scaling_factor) (((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_0)     \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_1)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_2)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_3)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_4)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_5)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_6)  \
                                           || ((scaling_factor) == HAL_CORDIC_SCALING_FACTOR_7))

/**
  * @brief Verify the CORDIC number of 32-bit arguments expected for one calculation.
  * @param  nbargs number of 32-bit arguments.
  * @retval SET   nbargs is valid.
  * @retval RESET nbargs is invalid.
  */
#define IS_CORDIC_NBARGS(nbargs) (((nbargs) == HAL_CORDIC_NB_ARG_1) \
                                  || ((nbargs) == HAL_CORDIC_NB_ARG_2))

/**
  * @brief Verify the CORDIC number of 32-bit results expected after one calculation.
  * @param  result_nb number of 32-bit results.
  * @retval SET   result_nb is valid.
  * @retval RESET result_nb is invalid.
  */
#define IS_CORDIC_RESULT_NB(result_nb) (((result_nb) == HAL_CORDIC_NB_RESULT_1) \
                                        || ((result_nb) == HAL_CORDIC_NB_RESULT_2))

/**
  * @brief Verify the CORDIC input data width for one calculation.
  * @param  in_width input data width.
  * @retval SET   in_width is valid.
  * @retval RESET in_width is invalid.
  */
#define IS_CORDIC_IN_WIDTH(in_width) (((in_width) == HAL_CORDIC_IN_WIDTH_32_BIT) \
                                      || ((in_width) == HAL_CORDIC_IN_WIDTH_16_BIT))

/**
  * @brief Verify the CORDIC output data width for one calculation.
  * @param  out_width output data width.
  * @retval SET   out_width is valid.
  * @retval RESET out_width is invalid.
  */
#define IS_CORDIC_OUT_WIDTH(out_width) (((out_width) == HAL_CORDIC_OUT_WIDTH_32_BIT) \
                                        || ((out_width) == HAL_CORDIC_OUT_WIDTH_16_BIT))

/**
  * @brief Verify write input DMA optional interrupt.
  * @param  it Interrupt selection
  * @retval SET   it is valid
  * @retval RESET it is invalid
  */
#define IS_CORDIC_OPT_DMA_IT_WR(it) (((it) == HAL_CORDIC_OPT_DMA_NONE)\
                                     || ((it) == HAL_CORDIC_OPT_DMA_IT_HALF_CPLT))

/**
  * @brief Verify  read result DMA optional interrupt.
  * @param  it Interrupt selection.
  * @retval SET   it is valid.
  * @retval RESET it is invalid.
  */
#define IS_CORDIC_OPT_DMA_IT_RD(it) (((it) == HAL_CORDIC_OPT_DMA_NONE)\
                                     || ((it) == HAL_CORDIC_OPT_DMA_IT_HALF_CPLT))


/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup CORDIC_Private_Functions CORDIC Private Functions
  * @{
  */
static inline void CORDIC_ResetArguments(const hal_cordic_handle_t *hcordic);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
static inline uint32_t CORDIC_CheckPrecision(hal_cordic_function_t function, hal_cordic_precision_t precision);
static inline uint32_t CORDIC_CheckScalingFactor(hal_cordic_function_t function,
                                                 hal_cordic_scaling_factor_t scaling_factor);
#endif /* USE_HAL_CHECK_PARAM */
static uint32_t CORDIC_ValidateWriteNumber(const hal_cordic_handle_t *hcordic,
                                           const hal_cordic_buffer_desc_t *p_input_buffer_desc);
static uint32_t CORDIC_ValidateReadNumber(const hal_cordic_handle_t *hcordic,
                                          const hal_cordic_buffer_desc_t *p_output_buffer_desc);
static void CORDIC_WriteDataAndIncPtr_1(const hal_cordic_handle_t *hcordic, const int32_t **pp_input_buffer);
static void CORDIC_WriteDataAndIncPtr_2(const hal_cordic_handle_t *hcordic, const int32_t **pp_input_buffer);
static void CORDIC_ReadDataAndIncPtr_1(const hal_cordic_handle_t *hcordic, int32_t **pp_output_buffer);
static void CORDIC_ReadDataAndIncPtr_2(const hal_cordic_handle_t *hcordic, int32_t **pp_output_buffer);
static hal_status_t CORDIC_Abort(hal_cordic_handle_t *hcordic);
#if defined(USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
static void CORDIC_DMAAbort(hal_dma_handle_t *hdma);
static hal_status_t CORDIC_Write_DMA_opt(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                         uint32_t opt_it);
static hal_status_t CORDIC_Read_DMA_opt(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff,
                                        uint32_t opt_it);
static void CORDIC_DMAInCplt(hal_dma_handle_t *hdma);
static void CORDIC_DMAInHalfCplt(hal_dma_handle_t *hdma);
static void CORDIC_DMAOutCplt(hal_dma_handle_t *hdma);
static void CORDIC_DMAOutHalfCplt(hal_dma_handle_t *hdma);
static void CORDIC_DMAError(hal_dma_handle_t *hdma);
#endif /* USE_HAL_CORDIC_DMA */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CORDIC_Exported_Functions
  * @{
  */

/** @addtogroup CORDIC_Exported_Functions_Group1
  * @{

This section provides a set of functions to initialize and deinitialize the CORDIC peripheral:

- Call the function HAL_CORDIC_Init() to initialize the selected CORDIC handle.
- Call the function HAL_CORDIC_DeInit() to deinitialize the selected CORDIC handle.
  */

/**
  * @brief Initialize the HAL CORDIC handle and associate it to an instance of the CORDIC peripheral.
  * @param  hcordic  Pointer to a \ref hal_cordic_handle_t.
  * @param  instance CORDIC instance.
  * @retval HAL_OK              CORDIC instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM   Pointer to HAL CORDIC handle is NULL.
  */
hal_status_t HAL_CORDIC_Init(hal_cordic_handle_t *hcordic, hal_cordic_t instance)
{
  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_ALL_INSTANCE((CORDIC_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hcordic == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->instance = instance;

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
  hcordic->p_dma_in  = NULL;
  hcordic->p_dma_out = NULL;
#endif /* USE_HAL_CORDIC_DMA */

#if defined (USE_HAL_CORDIC_USER_DATA) && (USE_HAL_CORDIC_USER_DATA == 1)
  hcordic->p_user_data = (void *) NULL;
#endif /* USE_HAL_CORDIC_USER_DATA */

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  if (hcordic->global_state == HAL_CORDIC_STATE_RESET)
  {
    /* Register the weak functions as default callback functions */
    hcordic->p_error_cb           = HAL_CORDIC_ErrorCallback;
    hcordic->p_calculate_cplt_cb  = HAL_CORDIC_CalculateCpltCallback;
    hcordic->p_write_cplt_cb      = HAL_CORDIC_WriteCpltCallback;
    hcordic->p_abort_cplt_cb      = HAL_CORDIC_AbortCpltCallback;
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
    hcordic->p_read_half_cplt_cb  = HAL_CORDIC_ReadHalfCpltCallback;
    hcordic->p_write_half_cplt_cb = HAL_CORDIC_WriteHalfCpltCallback;
#endif /* USE_HAL_CORDIC_DMA */
  }
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  hcordic->p_input_buffer  = NULL;
  hcordic->p_output_buffer = NULL;

  hcordic->computation_nb = 0UL;
  hcordic->result_nb      = 0UL;

  hcordic->p_wr_arg        = NULL;
  hcordic->p_rd_result     = NULL;

#if defined(USE_HAL_CORDIC_CLK_ENABLE_MODEL) && (USE_HAL_CORDIC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_CORDIC_EnableClock();
#endif /* USE_HAL_CORDIC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

  hcordic->global_state = HAL_CORDIC_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief DeInitialize the CORDIC peripheral.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  */
void HAL_CORDIC_DeInit(hal_cordic_handle_t *hcordic)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_ALL_INSTANCE((CORDIC_TypeDef *)((uint32_t)hcordic->instance)));

  const hal_cordic_state_t temp_state = hcordic->global_state;
  if ((temp_state == HAL_CORDIC_STATE_IDLE) || (temp_state == HAL_CORDIC_STATE_ACTIVE))
  {
    (void)CORDIC_Abort(hcordic);
  }

  hcordic->global_state = HAL_CORDIC_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group2
  * @{

This section provides a set of functions to configure the CORDIC driver:

- Call the function HAL_CORDIC_SetConfig() to configure the peripheral before starting the CORDIC driver.
- Call the function HAL_CORDIC_GetConfig() to retrieve the configuration.
- Call the function HAL_CORDIC_SetFunction() to set the mathematical function.
- Call the function HAL_CORDIC_GetFunction() to retrieve the current mathematical function.
- Call the function HAL_CORDIC_SetInputWidth() to set the width (16-bit or 32-bit) of the input data.
- Call the function HAL_CORDIC_GetInputWidth() to retrieve the current width of the input data.
- Call the function HAL_CORDIC_SetOutputWidth() to set the width (16-bit or 32-bit) of the output data.
- Call the function HAL_CORDIC_GetOutputWidth() to retrieve the current width of the output data.
- Call the function HAL_CORDIC_SetNumberArguments() to set the number of arguments of the function.
- Call the function HAL_CORDIC_GetNumberArguments() to retrieve the current number of arguments of the function.
- Call the function HAL_CORDIC_SetNumberResults() to set the number of results of the function.
- Call the function HAL_CORDIC_GetNumberResults() to retrieve the current number of results.
- Call the function HAL_CORDIC_SetPrecision() to set the precision required.
- Call the function HAL_CORDIC_GetPrecision() to retrieve the current precision.
- Call the function HAL_CORDIC_SetScalingFactor() to set the scaling factor.
- Call the function HAL_CORDIC_GetScalingFactor() to retrieve the current scaling factor.
- Call the function HAL_CORDIC_SetWriteDMA() to set the DMA channel for writing arguments.
- Call the function HAL_CORDIC_SetReadDMA() to set the DMA channel for reading results.
- Call the function HAL_CORDIC_GetWriteAddress() to get the address of the input arguments.
- Call the function HAL_CORDIC_GetReadAddress() to get the address of the output results.
  */

/**
  * @brief Configure the CORDIC driver.
  * @param  hcordic  Pointer to a \ref hal_cordic_handle_t.
  * @param  p_config Pointer to a \ref hal_cordic_config_t structure.
  * @retval HAL_OK              CORDIC block has been correctly configured.
  * @retval HAL_INVALID_PARAM   When the p_config pointer is NULL.
  */
hal_status_t HAL_CORDIC_SetConfig(hal_cordic_handle_t *hcordic, const hal_cordic_config_t *p_config)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_FUNCTION(p_config->function));
  ASSERT_DBG_PARAM(IS_CORDIC_PRECISION(p_config->precision));
  ASSERT_DBG_PARAM(IS_CORDIC_SCALING(p_config->scaling_factor));
  ASSERT_DBG_PARAM(IS_CORDIC_NBARGS(p_config->nb_arg));
  ASSERT_DBG_PARAM(IS_CORDIC_RESULT_NB(p_config->nb_result));
  ASSERT_DBG_PARAM(IS_CORDIC_IN_WIDTH(p_config->in_width));
  ASSERT_DBG_PARAM(IS_CORDIC_OUT_WIDTH(p_config->out_width));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }

  if (CORDIC_CheckScalingFactor(p_config->function, p_config->scaling_factor) == 0UL)
  {
    return HAL_INVALID_PARAM;
  }

  if (CORDIC_CheckPrecision(p_config->function, p_config->precision) == 0UL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Apply all configuration parameters in CORDIC control register */
  LL_CORDIC_Config(p_cordic, (uint32_t)(p_config->function), (uint32_t)(p_config->precision),
                   (uint32_t)(p_config->scaling_factor), (uint32_t)(p_config->nb_arg),
                   (uint32_t)(p_config->nb_result), (uint32_t)(p_config->in_width),
                   (uint32_t)(p_config->out_width));

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC global configuration.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  p_config pointer to \ref hal_cordic_config_t structure.
  */
void HAL_CORDIC_GetConfig(const hal_cordic_handle_t *hcordic, hal_cordic_config_t *p_config)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  p_config->function = (hal_cordic_function_t)LL_CORDIC_GetFunction(p_cordic);
  p_config->scaling_factor = (hal_cordic_scaling_factor_t)LL_CORDIC_GetScale(p_cordic);
  p_config->in_width = (hal_cordic_in_width_t)LL_CORDIC_GetInWidth(p_cordic);
  p_config->out_width = (hal_cordic_out_width_t)LL_CORDIC_GetOutWidth(p_cordic);
  p_config->precision = (hal_cordic_precision_t)LL_CORDIC_GetPrecision(p_cordic);
  p_config->nb_arg = (hal_cordic_nb_arg_t)LL_CORDIC_GetNbWrite(p_cordic);
  p_config->nb_result  = (hal_cordic_nb_result_t)LL_CORDIC_GetNbRead(p_cordic);
}

/**
  * @brief Set the CORDIC function.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  function  The value can be value of \ref hal_cordic_function_t.
  * @retval HAL_OK    CORDIC function was successfully set.
  */
hal_status_t HAL_CORDIC_SetFunction(hal_cordic_handle_t *hcordic, hal_cordic_function_t function)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_FUNCTION(function));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  /* Flush the argument register when calculation is over */
  CORDIC_ResetArguments(hcordic);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  LL_CORDIC_SetFunction(p_cordic, (uint32_t)function);

  return HAL_OK;
}

/**
  * @brief Retrieve the current CORDIC function.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t structure.
  * @retval hal_cordic_function_t CORDIC function.
  */
hal_cordic_function_t HAL_CORDIC_GetFunction(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_function_t)LL_CORDIC_GetFunction(p_cordic));
}

/**
  * @brief Set the CORDIC precision in multiple of 4 cycles number.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  precision The value can be a value of \ref hal_cordic_precision_t.
  * @retval HAL_OK      CORDIC precision was successfully configured.
  * @retval HAL_INVALID_PARAM The precision does not match the function requirements.
  */
hal_status_t HAL_CORDIC_SetPrecision(const hal_cordic_handle_t *hcordic, const hal_cordic_precision_t precision)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_PRECISION(precision));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  hal_cordic_function_t function = (hal_cordic_function_t)LL_CORDIC_GetFunction(p_cordic);

  /* Check coherency of the precision vs function already set */
  if (CORDIC_CheckPrecision(function, precision) == 0UL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_CORDIC_SetPrecision(p_cordic, (uint32_t)precision);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC precision.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_precision_t CORDIC precision.
  */
hal_cordic_precision_t HAL_CORDIC_GetPrecision(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_precision_t)LL_CORDIC_GetPrecision(p_cordic));
}

/**
  * @brief Set the CORDIC scaling factor.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  scaling_factor The value can be a value of \ref hal_cordic_scaling_factor_t.
  * @retval HAL_OK  CORDIC scaling factor was successfully configured.
  * @retval HAL_INVALID_PARAM The scaling_factor does not match the function requirements.
  */
hal_status_t HAL_CORDIC_SetScalingFactor(const hal_cordic_handle_t *hcordic,
                                         const hal_cordic_scaling_factor_t scaling_factor)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_SCALING(scaling_factor));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  hal_cordic_function_t function = (hal_cordic_function_t)LL_CORDIC_GetFunction(p_cordic);

  /* Check coherency of the scaling factor vs function already set */
  if (CORDIC_CheckScalingFactor(function, scaling_factor) == 0UL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_CORDIC_SetScale(p_cordic, (uint32_t)scaling_factor);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC scaling factor.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_scaling_factor_t CORDIC scaling factor.
  */
hal_cordic_scaling_factor_t HAL_CORDIC_GetScalingFactor(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_scaling_factor_t)LL_CORDIC_GetScale(p_cordic));
}

/**
  * @brief Set the CORDIC argument width.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  input_width The value can be a value of \ref hal_cordic_in_width_t.
  * @retval HAL_OK     Data arguments width was successfully configured.
  */
hal_status_t HAL_CORDIC_SetInputWidth(const hal_cordic_handle_t *hcordic,
                                      const hal_cordic_in_width_t input_width)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_IN_WIDTH(input_width));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  LL_CORDIC_SetInWidth(p_cordic, (uint32_t)input_width);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC argument width.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_in_width_t CORDIC argument width.
  */
hal_cordic_in_width_t HAL_CORDIC_GetInputWidth(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));

  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_in_width_t)LL_CORDIC_GetInWidth(p_cordic));
}

/**
  * @brief Set the CORDIC result width.
  * @param  hcordic     Pointer to a \ref hal_cordic_handle_t.
  * @param  output_width The value can be a value of \ref hal_cordic_out_width_t.
  * @retval HAL_OK      Results width was successfully configured.
  */
hal_status_t HAL_CORDIC_SetOutputWidth(const hal_cordic_handle_t *hcordic,
                                       const hal_cordic_out_width_t output_width)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_OUT_WIDTH(output_width));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  LL_CORDIC_SetOutWidth(p_cordic, (uint32_t)output_width);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC result width.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_out_width_t CORDIC results width.
  */
hal_cordic_out_width_t HAL_CORDIC_GetOutputWidth(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_out_width_t)LL_CORDIC_GetOutWidth(p_cordic));
}

/**
  * @brief Set the CORDIC number of arguments expected.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  nb_argument The value can be a value of \ref hal_cordic_nb_arg_t.
  * @retval HAL_OK      Number of arguments was successfully configured.
  */
hal_status_t HAL_CORDIC_SetNumberArguments(const hal_cordic_handle_t *hcordic, const hal_cordic_nb_arg_t nb_argument)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_NBARGS(nb_argument));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  LL_CORDIC_SetNbWrite(p_cordic, (uint32_t)nb_argument);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC number of arguments.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_nb_arg_t CORDIC number of arguments.
  */
hal_cordic_nb_arg_t HAL_CORDIC_GetNumberArguments(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_nb_arg_t)LL_CORDIC_GetNbWrite(p_cordic));
}

/**
  * @brief Set the CORDIC number of results expected.
  * @param  hcordic   pointer to a \ref hal_cordic_handle_t.
  * @param  nb_result The value can be a value of \ref hal_cordic_nb_result_t.
  * @retval HAL_OK Number of results was successfully configured.
  */
hal_status_t HAL_CORDIC_SetNumberResults(const hal_cordic_handle_t *hcordic, const hal_cordic_nb_result_t nb_result)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM(IS_CORDIC_RESULT_NB(nb_result));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)HAL_CORDIC_STATE_IDLE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  LL_CORDIC_SetNbRead(p_cordic, (uint32_t)nb_result);

  return HAL_OK;
}

/**
  * @brief Retrieve the CORDIC number of results.
  * @param hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval HAL status.
  */
hal_cordic_nb_result_t HAL_CORDIC_GetNumberResults(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return ((hal_cordic_nb_result_t)LL_CORDIC_GetNbRead(p_cordic));
}


/**
  * @brief Get the input arguments address.
  *        Arguments can be directly driven by a timer or other peripheral such as an ADC.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval input argument address.
  */
volatile uint32_t *HAL_CORDIC_GetWriteAddress(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return (uint32_t *)(LL_CORDIC_DMA_GetRegAddr(p_cordic, LL_CORDIC_DMA_REG_DATA_IN));
}

/**
  * @brief Get the output results address.
  *        results can be directly driven by a timer or other peripheral such as a DAC.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval the output results address.
  */
volatile uint32_t *HAL_CORDIC_GetReadAddress(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_IDLE | HAL_CORDIC_STATE_ACTIVE
                                                     | HAL_CORDIC_STATE_ABORT));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  return (uint32_t *)(LL_CORDIC_DMA_GetRegAddr(p_cordic, LL_CORDIC_DMA_REG_DATA_OUT));
}

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
/**
  * @brief Set DMA channel for writing arguments.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  hdma_wr Pointer to a hal_dma_handle_t structure which contains the DMA instance.
  * @retval HAL_OK            The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_wr is NULL.
  */
hal_status_t HAL_CORDIC_SetWriteDMA(hal_cordic_handle_t *hcordic, hal_dma_handle_t *hdma_wr)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(hdma_wr != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_wr == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_dma_in = hdma_wr;
  hdma_wr->p_parent = hcordic;

  return HAL_OK;
}

/**
  * @brief Set DMA channel for reading results.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  hdma_rd Pointer to a hal_dma_handle_t structure which contains the DMA instance.
  * @retval HAL_OK            The channel has been correctly set.
  * @retval HAL_INVALID_PARAM hdma_rd is NULL.
  */
hal_status_t HAL_CORDIC_SetReadDMA(hal_cordic_handle_t *hcordic, hal_dma_handle_t *hdma_rd)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(hdma_rd != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma_rd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_dma_out = hdma_rd;
  hdma_rd->p_parent  = hcordic;

  return HAL_OK;
}

#endif /* USE_HAL_CORDIC_DMA */

/**
  * @}
  */


/** @addtogroup CORDIC_Exported_Functions_Group4
  * @{

This section provides a set of functions for the calculation modes of the CORDIC.

There are four calculation modes:
- Blocking mode: Communication is performed in polling mode.
  - HAL_CORDIC_Calculate() : Perform write and read operations in polling mode.
  - HAL_CORDIC_CalculateZeroOverhead() : Perform write and read zero-overhead operations in polling mode.

- Non-blocking mode functions with interrupts are:
  - HAL_CORDIC_Calculate_IT() : Perform write and read operations in IT mode.

- Non-blocking mode functions with DMA are:
  - HAL_CORDIC_Write_DMA() : Write input arguments in DMA mode (the corresponding read stream is handled by another
    peripheral).
  - HAL_CORDIC_Write_DMA_opt() : Write input arguments in DMA mode and optional interrupt (the corresponding read stream
    is handled by another peripheral).
  - HAL_CORDIC_Read_DMA() : Read output results in DMA mode (the corresponding write stream is handled by another
    peripheral).
  - HAL_CORDIC_Read_DMA_opt() : Read output results in DMA mode and optional interrupt (the corresponding read stream is
    handled by another peripheral).
  - HAL_CORDIC_Calculate_DMA() : Perform a write and read operations in DMA mode.

Note that some functions require one or two arguments for the selected calculation function.
The list hereafter summarizes the number of arguments needed by each function of the CORDIC IP driver.
  -  Cosine                 2 arguments
  -  Sine                   2 arguments
  -  Phase                  2 arguments
  -  Modulus                2 arguments
  -  Arctangent             1 argument
  -  Hyperbolic cosine      1 argument
  -  Hyperbolic sine        1 argument
  -  Hyperbolic arctangent  1 argument
  -  natural logarithm      1 argument
  -  Square root            1 argument

Keep in mind that invoking a function that requires two arguments and setting only one argument can generate
erroneous results. To prevent that case, set the unused argument to the default value +1 (0x7FFFFFFF).

Remember that some functions require a scaling factor to produce correct results. Refer to the
reference manual to configure the required parameters correctly for the selected functions.
  */

/**
  * @brief Perform CORDIC processing in polling mode,
  *         according to the existing CORDIC configuration.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff  Pointer to buffer descriptor containing pointer to the input data buffer and the buffer size.
  * @param  p_out_buff Pointer to buffer descriptor containing pointer to the output data buffer and the buffer size.
  * @param  timeout_ms Specify timeout value in milliseconds.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_TIMEOUT       Operation cancelled due to timeout.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_CORDIC_Calculate(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                  const hal_cordic_buffer_desc_t *p_out_buff, uint32_t timeout_ms)
{
  CORDIC_TypeDef *p_cordic;
  uint32_t nb_write;
  uint32_t tickstart;
  uint32_t index;
  const int32_t *p_tmp_in_buff;
  int32_t *p_tmp_out_buff;

  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_in_buff != NULL);
  ASSERT_DBG_PARAM(p_out_buff != NULL);
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_out_buff == NULL)
      || (p_in_buff->p_data == NULL) || (p_out_buff->p_data == NULL)
      || (p_in_buff->size_word == 0UL) || (p_out_buff->size_word == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  nb_write = CORDIC_ValidateWriteNumber(hcordic, p_in_buff);

#if defined(USE_ASSERT_DBG_PARAM)
  uint32_t nb_read = CORDIC_ValidateReadNumber(hcordic, p_out_buff);
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_PARAM((nb_write <= nb_read));

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_1;
  if (LL_CORDIC_GetNbWrite(p_cordic) == (uint32_t)HAL_CORDIC_NB_ARG_2)
  {
    hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_2;
  }

  hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_1;
  if (LL_CORDIC_GetNbRead(p_cordic) == (uint32_t)HAL_CORDIC_NB_RESULT_2)
  {
    hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_2;
  }

  p_tmp_in_buff = p_in_buff->p_data;
  p_tmp_out_buff = p_out_buff->p_data;

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  /* Init tick_start for timeout management */
  tickstart = HAL_GetTick();

  /* Write of input data in Write Data register, and increment input buffer pointer */
  hcordic->p_wr_arg(hcordic, &p_tmp_in_buff);
  index = (nb_write - 1U);
  /* Calculation is started.
     Provide next set of input data, until number of calculation is achieved */
  do
  {
    /* Write of input data in Write Data register, and increment input buffer pointer */
    hcordic->p_wr_arg(hcordic, &p_tmp_in_buff);
    index--;
    /* Wait for output results to be available */
    do
    {
      if (timeout_ms != HAL_MAX_DELAY)
      {
        if ((HAL_GetTick() - tickstart) > timeout_ms)
        {
          if (LL_CORDIC_IsActiveFlag_RRDY(p_cordic) == CORDIC_FLAG_NOT_ACTIVE)
          {
            hcordic->global_state = HAL_CORDIC_STATE_IDLE;
            return HAL_TIMEOUT;
          }
        }
      }
    } while (LL_CORDIC_IsActiveFlag_RRDY(p_cordic) == CORDIC_FLAG_NOT_ACTIVE);
    /* Read output data from Read Data register, and increment output buffer pointer */
    hcordic->p_rd_result(hcordic, &p_tmp_out_buff);
  } while (index > 0U);

  /* Read output data from Read Data register, and increment output buffer pointer */
  hcordic->p_rd_result(hcordic, &p_tmp_out_buff);

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Perform CORDIC processing in Zero-Overhead mode (output data being read.
  *         soon as input data are written), according to the existing CORDIC configuration.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff  Pointer to buffer descriptor containing pointer to the input data buffer and the buffer size.
  * @param  p_out_buff Pointer to buffer descriptor containing pointer to the output data buffer and the buffer size.
  * @param  timeout_ms Specify timeout value in milliseconds.
  * @retval HAL_OK      Operation completed successfully.
  * @retval HAL_TIMEOUT Operation cancelled due to timeout.
  * @retval HAL_BUSY    Concurrent process ongoing.
  */
hal_status_t HAL_CORDIC_CalculateZeroOverhead(hal_cordic_handle_t *hcordic,
                                              const hal_cordic_buffer_desc_t *p_in_buff,
                                              const hal_cordic_buffer_desc_t *p_out_buff, uint32_t timeout_ms)
{
  const CORDIC_TypeDef *p_cordic;
  const int32_t *p_tmp_in_buff;
  int32_t *p_tmp_out_buff;
  uint32_t nb_write;
  uint32_t tickstart;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_in_buff != NULL));
  ASSERT_DBG_PARAM((p_out_buff != NULL));
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_out_buff == NULL)
      || (p_in_buff->p_data == NULL) || (p_out_buff->p_data == NULL)
      || (p_in_buff->size_word  == 0UL) || (p_out_buff->size_word  == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  nb_write = CORDIC_ValidateWriteNumber(hcordic, p_in_buff);
#if defined(USE_ASSERT_DBG_PARAM)
  uint32_t nb_read = CORDIC_ValidateReadNumber(hcordic, p_out_buff);
#endif /* USE_ASSERT_DBG_PARAM */

  ASSERT_DBG_PARAM((nb_write <= nb_read));

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_1;
  if (LL_CORDIC_GetNbWrite(p_cordic) == (uint32_t)HAL_CORDIC_NB_ARG_2)
  {
    hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_2;
  }

  hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_1;
  if (LL_CORDIC_GetNbRead(p_cordic) == (uint32_t)HAL_CORDIC_NB_RESULT_2)
  {
    hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_2;
  }

  p_tmp_in_buff  = p_in_buff->p_data;
  p_tmp_out_buff = p_out_buff->p_data;

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  /* Init tick_start for timeout management */
  tickstart = HAL_GetTick();

  /* Write of input data in Write Data register, and increment input buffer pointer */
  hcordic->p_wr_arg(hcordic, &p_tmp_in_buff);
  /* Calculation is started. Provide next set of input data, until number of calculation is achieved */
  do
  {
    /* Write of input data in Write Data register, and increment input buffer pointer */
    hcordic->p_wr_arg(hcordic, &p_tmp_in_buff);
    nb_write--;
    /* Read output data from Read Data register, and increment output buffer pointer
       Reading is performed in Zero-Overhead mode: No Result Ready flag, only bus wait insertion. */
    hcordic->p_rd_result(hcordic, &p_tmp_out_buff);

    if (timeout_ms != HAL_MAX_DELAY)
    {
      if ((HAL_GetTick() - tickstart) > timeout_ms)
      {
        if (LL_CORDIC_IsActiveFlag_RRDY(p_cordic) == CORDIC_FLAG_NOT_ACTIVE)
        {
          hcordic->global_state = HAL_CORDIC_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  } while (nb_write > 0U);

  /* Read output data from Read Data register, and increment output buffer pointer.
     The reading is performed in Zero-Overhead mode, reading is done immediately without waiting result ready flag */
  hcordic->p_rd_result(hcordic, &p_tmp_out_buff);

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Perform CORDIC processing in interrupt mode, according to the existing CORDIC configuration.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff  Pointer to buffer descriptor containing pointer to the input data buffer and the buffer size.
  * @param  p_out_buff Pointer to buffer descriptor containing pointer to the output data buffer and the buffer size.
  * @retval HAL_OK   Operation completed successfully.
  * @retval HAL_BUSY Concurrent process ongoing.
  */
hal_status_t HAL_CORDIC_Calculate_IT(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                     const hal_cordic_buffer_desc_t *p_out_buff)
{
  CORDIC_TypeDef *p_cordic;
  const int32_t *tmp_pInBuff;
  uint32_t nb_write;
  uint32_t nb_read;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_in_buff != NULL));
  ASSERT_DBG_PARAM((p_out_buff != NULL));
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_out_buff == NULL)
      || (p_in_buff->p_data == NULL) || (p_out_buff->p_data == NULL)
      || (p_in_buff->size_word == 0UL) || (p_out_buff->size_word == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  nb_write = CORDIC_ValidateWriteNumber(hcordic, p_in_buff);
  nb_read = CORDIC_ValidateReadNumber(hcordic, p_out_buff);

  ASSERT_DBG_PARAM((nb_write <= nb_read));

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);
  tmp_pInBuff = p_in_buff->p_data;

  hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_1;
  tmp_pInBuff++;
  if (LL_CORDIC_GetNbWrite(p_cordic) == (uint32_t)HAL_CORDIC_NB_ARG_2)
  {
    hcordic->p_wr_arg = CORDIC_WriteDataAndIncPtr_2;
    tmp_pInBuff++;
  }

  hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_1;
  if (LL_CORDIC_GetNbRead(p_cordic) == (uint32_t)HAL_CORDIC_NB_RESULT_2)
  {
    hcordic->p_rd_result = CORDIC_ReadDataAndIncPtr_2;
  }

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  hcordic->p_input_buffer  = tmp_pInBuff;
  hcordic->p_output_buffer = p_out_buff->p_data;
  hcordic->computation_nb  = nb_write - 1U;
  hcordic->result_nb       = nb_read;

  LL_CORDIC_EnableIT(p_cordic);

  tmp_pInBuff = p_in_buff->p_data;

  /*write of input data in the Write Data register, and increment input buffer pointer */
  hcordic->p_wr_arg(hcordic, &tmp_pInBuff);

  return HAL_OK;
}

#if defined(USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)

/**
  * @brief Write input arguments in DMA mode.
  * @param  hcordic   Pointer to a \ref hal_cordic_handle_t
  * @param  p_in_buff Pointer to buffer descriptor containing pointer to the input data buffer and the buffer size.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_CORDIC_Write_DMA(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff)
{
  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_in_buff != NULL));
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM((hcordic->p_dma_in != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_in_buff->p_data == NULL)
      || (p_in_buff->size_word == 0UL) || (hcordic->p_dma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  /* Enable the DMA stream managing CORDIC input data write */
  if (CORDIC_Write_DMA_opt(hcordic, p_in_buff, (uint32_t)HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Write input arguments in DMA mode with option. Global state must be IDLE.
  * @param  hcordic   Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff Pointer to input data descriptor: pointer to data and input size in word.
  * @param  opt_it    Optional interruption can be a combination of
  *                   \ref HAL_CORDIC_OPT_DMA_NONE
  *                   \ref HAL_CORDIC_OPT_DMA_IT_HALF_CPLT.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_CORDIC_Write_DMA_opt(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                      uint32_t opt_it)
{
  uint32_t interrupt_opt;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_in_buff != NULL));
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM((hcordic->p_dma_in != NULL));
  ASSERT_DBG_PARAM((IS_CORDIC_OPT_DMA_IT_WR(opt_it)));
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_in_buff->p_data == NULL)
      || (p_in_buff->size_word == 0UL) || (opt_it > HAL_CORDIC_OPT_DMA_IT_HALF_CPLT) || (hcordic->p_dma_in == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  if ((opt_it & HAL_CORDIC_OPT_DMA_IT_HALF_CPLT) != 0UL)
  {
    interrupt_opt  = HAL_DMA_OPT_IT_HT;
  }
  else
  {
    interrupt_opt  = HAL_DMA_OPT_IT_DEFAULT;
  }

  /* Enable the DMA stream managing CORDIC input data write */
  if (CORDIC_Write_DMA_opt(hcordic, p_in_buff, interrupt_opt) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Read output results in DMA mode.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_out_buff Pointer to buffer descriptor containing pointer to the output data buffer and the buffer size.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_CORDIC_Read_DMA(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff)
{
  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_out_buff != NULL));
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_PARAM((hcordic->p_dma_out != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_out_buff == NULL) || (p_out_buff->p_data == NULL)
      || (p_out_buff->size_word == 0UL) || (hcordic->p_dma_out == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  /* Enable the DMA stream managing CORDIC output data read */
  if (CORDIC_Read_DMA_opt(hcordic, p_out_buff, (uint32_t)HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Read output result in DMA mode. Global state must be IDLE.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_out_buff Pointer to output data descriptor: pointer to data and output size in word.
  * @param  opt_it     Optional interruption can be a combination of the following values:
  *                    @arg \ref HAL_CORDIC_OPT_DMA_NONE
  *                    @arg \ref HAL_CORDIC_OPT_DMA_IT_HALF_CPLT
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  * @retval HAL_BUSY          Concurrent process ongoing.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_CORDIC_Read_DMA_opt(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff,
                                     uint32_t opt_it)
{
  uint32_t interrupt_opt ;

  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_out_buff != NULL));
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_PARAM((hcordic->p_dma_out != NULL));
  ASSERT_DBG_PARAM((IS_CORDIC_OPT_DMA_IT_RD(opt_it)));
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_out_buff == NULL) || (p_out_buff->p_data == NULL)
      || (p_out_buff->size_word == 0UL) || (opt_it > HAL_CORDIC_OPT_DMA_IT_HALF_CPLT) || (hcordic->p_dma_out == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  if ((opt_it & HAL_CORDIC_OPT_DMA_IT_HALF_CPLT) != 0UL)
  {
    interrupt_opt  = HAL_DMA_OPT_IT_HT;
  }
  else
  {
    interrupt_opt  = HAL_DMA_OPT_IT_DEFAULT;
  }

  /* Enable the DMA stream managing CORDIC output data read */
  if (CORDIC_Read_DMA_opt(hcordic, p_out_buff, interrupt_opt) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Perform  input data and output data of CORDIC processing in DMA mode according to the
  *        existing CORDIC configuration.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff  Pointer to buffer descriptor containing pointer to the input data buffer and the buffer size.
  * @param  p_out_buff Pointer to buffer descriptor containing pointer to the output data buffer and the buffer size.
  * @note   p_in_buff and output_buffer buffers must be 32-bit aligned to ensure a correct DMA transfer to and from
  *         the Peripheral. The function requires to configure the 2 DMA channels (Input and Output).
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_ERROR         Operation error.
  * @retval HAL_BUSY          Concurrent process ongoing.
  */
hal_status_t HAL_CORDIC_Calculate_DMA(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                      hal_cordic_buffer_desc_t *p_out_buff)
{
  ASSERT_DBG_PARAM((hcordic != NULL));
  ASSERT_DBG_PARAM((p_out_buff != NULL));
  ASSERT_DBG_PARAM((p_in_buff != NULL));
  ASSERT_DBG_PARAM(p_in_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_in_buff->size_word > 0UL);
  ASSERT_DBG_PARAM(p_out_buff->p_data != NULL);
  ASSERT_DBG_PARAM(p_out_buff->size_word > 0UL);
  ASSERT_DBG_PARAM((hcordic->p_dma_in != NULL));
  ASSERT_DBG_PARAM((hcordic->p_dma_out != NULL));
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_in_buff == NULL) || (p_out_buff == NULL) || (hcordic->p_dma_out == NULL) || (hcordic->p_dma_in == NULL)
      || (p_in_buff->p_data == NULL) || (p_out_buff->p_data == NULL)
      || (p_in_buff->size_word == 0UL) || (p_out_buff->size_word == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hcordic, global_state, HAL_CORDIC_STATE_IDLE, HAL_CORDIC_STATE_ACTIVE);

  /* Enable the DMA stream managing CORDIC output data read */
  if (CORDIC_Read_DMA_opt(hcordic, p_out_buff, (uint32_t)HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  /* Enable the DMA stream managing CORDIC input data write */
  if (CORDIC_Write_DMA_opt(hcordic, p_in_buff, (uint32_t)HAL_DMA_OPT_IT_NONE) != HAL_OK)
  {
    hcordic->global_state = HAL_CORDIC_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* USE_HAL_CORDIC_DMA */

/**
  * @brief Abort the ongoing transfer (blocking process).
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval  HAL_OK Operation completed successfully.
  */
hal_status_t HAL_CORDIC_Abort(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_ACTIVE);

  hcordic->global_state = HAL_CORDIC_STATE_ABORT;

  (void)CORDIC_Abort(hcordic);

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Abort a CORDIC process (non-blocking process).
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval  HAL_OK Operation completed successfully.
  */
hal_status_t HAL_CORDIC_Abort_IT(hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;
  uint32_t abort_cplt = 1U;

  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, HAL_CORDIC_STATE_ACTIVE);

  hcordic->global_state = HAL_CORDIC_STATE_ABORT;

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

#if defined(USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
  /* Check if the DMA Read (output) is enabled */
  if (LL_CORDIC_IsEnabledDMAReq_RD(p_cordic) != 0U)
  {
    LL_CORDIC_DisableDMAReq_RD(p_cordic);

    if (hcordic->p_dma_out->global_state == HAL_DMA_STATE_ACTIVE)
    {
      hcordic->p_dma_out->p_xfer_abort_cb = CORDIC_DMAAbort;
      if (HAL_DMA_Abort_IT(hcordic->p_dma_out) == HAL_OK)
      {
        abort_cplt = 0U;
      }
    }
  }

  /* Check if the DMA Write (input) is enabled */
  if (LL_CORDIC_IsEnabledDMAReq_WR(p_cordic) != 0U)
  {
    LL_CORDIC_DisableDMAReq_WR(p_cordic);

    if (hcordic->p_dma_in->global_state == HAL_DMA_STATE_ACTIVE)
    {
      hcordic->p_dma_in->p_xfer_abort_cb = CORDIC_DMAAbort;
      if (HAL_DMA_Abort_IT(hcordic->p_dma_in) == HAL_OK)
      {
        abort_cplt = 0U;
      }
    }
  }
#endif /* USE_HAL_CORDIC_DMA */

  /* if no DMA abort complete callback execution is required => call user abort complete callback */
  if (abort_cplt != 0U)
  {
    LL_CORDIC_DisableIT(p_cordic);

    /* Reset the buffers and counters */
    hcordic->p_input_buffer  = NULL;
    hcordic->p_output_buffer = NULL;
    hcordic->computation_nb  = 0UL;
    hcordic->result_nb       = 0UL;
    hcordic->p_wr_arg        = NULL;
    hcordic->p_rd_result     = NULL;

    hcordic->global_state = HAL_CORDIC_STATE_IDLE;

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
    hcordic->p_abort_cplt_cb(hcordic);
#else
    HAL_CORDIC_AbortCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
    hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */
  }

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group5
  * @{

This section provides functions to:
- Handle the CORDIC interrupt request with HAL_CORDIC_IRQHandler().

There are two ways to use callbacks: override weak callback functions or register user callback functions.
They are used to indicate:
  - When all the computations are done (HAL_CORDIC_CalculateCpltCallback() or callback function registered
    with HAL_CORDIC_RegisterCalculateCpltCallback()).
  - When half of all the computations are read (HAL_CORDIC_ReadHalfCpltCallback() or callback function registered
    with HAL_CORDIC_RegisterReadHalfCpltCallback()).
  - When all the data have been written  (HAL_CORDIC_WriteCpltCallback() or callback function registered
    with HAL_CORDIC_RegisterWriteCpltCallback()).
  - When half of all the data have been written  (HAL_CORDIC_WriteHalfCpltCallback() or callback function registered
    with HAL_CORDIC_RegisterWriteHalfCpltCallback()).
  - When the abort is complete (HAL_CORDIC_AbortCpltCallback() or callback function registered with
    HAL_CORDIC_RegisterAbortCpltCallback()).
  - When an error occurs in the CORDIC driver (HAL_CORDIC_ErrorCallback() or callback function registered with
    HAL_CORDIC_RegisterErrorCallback()).

Depending on the process function you use, different callbacks might be triggered:

| Process API \n \ \n Callbacks    | HAL_CORDIC_Calculate_IT |
|----------------------------------|:-----------------------:|
| HAL_CORDIC_CalculateCpltCallback |            x            |
| HAL_CORDIC_ErrorCallback         |            x            |


| Process API \n \ \n Callbacks    | HAL_CORDIC_Calculate_DMA | HAL_CORDIC_Write_DMA | HAL_CORDIC_Read_DMA |
|----------------------------------|:------------------------:|:--------------------:|:-------------------:|
| HAL_CORDIC_CalculateCpltCallback |            x             |                      |          x          |
| HAL_CORDIC_WriteCpltCallback     |            x             |           x          |                     |
| HAL_CORDIC_ErrorCallback         |            x             |           x          |          x          |


| Process API \n \ \n Callbacks    | HAL_CORDIC_Write_DMA_opt | HAL_CORDIC_Read_DMA_opt |
|----------------------------------|:------------------------:|:-----------------------:|
| HAL_CORDIC_CalculateCpltCallback |                          |             x           |
| HAL_CORDIC_WriteCpltCallback     |             x            |                         |
| HAL_CORDIC_ReadHalfCpltCallback* |                          |             x           |
| HAL_CORDIC_WriteHalfCpltCallback*|             x            |                         |
| HAL_CORDIC_ErrorCallback         |             x            |             x           |

@note with HAL_CORDIC_OPT_DMA_IT_HALF_CPLT argument value for interrupts parameter

| Process API \n \ \n Callbacks | HAL_CORDIC_Abort_IT |
|-------------------------------|:-------------------:|
| HAL_CORDIC_AbortCpltCallback  |          x          |
  */

/**
  * @brief Handle CORDIC interrupt request.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  */
void HAL_CORDIC_IRQHandler(hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  ASSERT_DBG_PARAM((hcordic != NULL));

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Check if calculation complete interrupt is enabled and if result ready flag is raised */
  if (LL_CORDIC_IsEnabledIT(p_cordic) != 0U)
  {
    if (LL_CORDIC_IsActiveFlag_RRDY(p_cordic) != CORDIC_FLAG_NOT_ACTIVE)
    {
      hcordic->result_nb--;

      /* Read output data from Read Data register, and increment output buffer pointer */
      hcordic->p_rd_result(hcordic, &(hcordic->p_output_buffer));

      /* Check if calculations are still to be done */
      if (hcordic->computation_nb > 0U)
      {
        hcordic->computation_nb--;
        /* Continue the processing by providing another write of input data
           in the Write Data register, and increment input buffer pointer */
        hcordic->p_wr_arg(hcordic, &(hcordic->p_input_buffer));
      }

      /* Check if all calculations results are all done */
      if (hcordic->result_nb == 0UL)
      {
        LL_CORDIC_DisableIT(p_cordic);
        hcordic->global_state = HAL_CORDIC_STATE_IDLE;

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
        hcordic->p_calculate_cplt_cb(hcordic);
#else
        HAL_CORDIC_CalculateCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
      }
    }
  }
}

/**
  * @brief CORDIC error callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_ErrorCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}

/**
  * @brief CORDIC calculate complete callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_CalculateCpltCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}
#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
/**
  * @brief CORDIC Write data half complete callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_WriteHalfCpltCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}

/**
  * @brief CORDIC Read data half complete callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_ReadHalfCpltCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}
#endif /* USE_HAL_CORDIC_DMA */

/**
  * @brief CORDIC Write data complete callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_WriteCpltCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}

/**
  * @brief Abort completed callback.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__weak void HAL_CORDIC_AbortCpltCallback(hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hcordic);
}

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
/**
  * @brief Register a User CORDIC callback for Error.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval  HAL_OK            Register completed successfully.
  * @retval  HAL_INVALID_PARAM p_callback pointer is NULL.
  */
hal_status_t HAL_CORDIC_RegisterErrorCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register a User CORDIC callback for DMA Abort complete.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval HAL_OK            Register completed successfully.
  * @retval HAL_INVALID_PARAM p_callback pointer is NULL.
  */
hal_status_t HAL_CORDIC_RegisterAbortCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief Register a User CORDIC callback for write data complete.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval HAL_OK            Register completed successfully.
  * @retval HAL_INVALID_PARAM p_callback pointer is NULL.
  */
hal_status_t HAL_CORDIC_RegisterWriteCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_write_cplt_cb = p_callback;

  return HAL_OK;
}

#if defined (USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
/**
  * @brief Register the CORDIC Write Half complete callback.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_CORDIC_RegisterWriteHalfCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_write_half_cplt_cb = p_callback;

  return HAL_OK;
}


/**
  * @brief Register the CORDIC Read Half complete callback.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval HAL_OK Register completed successfully.
  */
hal_status_t HAL_CORDIC_RegisterReadHalfCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_read_half_cplt_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_CORDIC_DMA */

/**
  * @brief Register a User CORDIC callback for calculation complete.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_callback Pointer to the callback function.
  * @retval HAL_OK            Register completed successfully.
  * @retval HAL_INVALID_PARAM p_callback pointer is NULL.
  */
hal_status_t HAL_CORDIC_RegisterCalculateCpltCallback(hal_cordic_handle_t *hcordic, hal_cordic_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hcordic->global_state, (uint32_t)(HAL_CORDIC_STATE_INIT | HAL_CORDIC_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hcordic->p_calculate_cplt_cb = p_callback;

  return HAL_OK;
}

#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
/**
  * @}
  */

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
/** @addtogroup CORDIC_Exported_Functions_Group6
  * @{

This section permits retrieving at runtime the last error codes of the CORDIC peripheral with
HAL_CORDIC_GetLastErrorCodes().
  */

/**
  * @brief Return the CORDIC peripheral error.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval uint32_t This return value can be a combination of the following values:
  *        @arg \ref HAL_CORDIC_ERROR_NONE
  *        @arg \ref HAL_CORDIC_ERROR_DMA.
  */
uint32_t HAL_CORDIC_GetLastErrorCodes(const hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM((hcordic != NULL));

  return hcordic->last_error_codes;
}
/**
  * @}
  */
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

/** @addtogroup CORDIC_Exported_Functions_Group7
  * @{

This section permits retrieving at runtime the state of the CORDIC peripheral using HAL_CORDIC_GetState().
  */

/**
  * @brief Return the CORDIC handle state.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval hal_cordic_state_t CORDIC state.
  */
hal_cordic_state_t HAL_CORDIC_GetState(const hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM((hcordic != NULL));

  return hcordic->global_state;
}
/**
  * @}
  */

/** @addtogroup CORDIC_Exported_Functions_Group8
  * @{

This section provides functions to set user-specific data to a CORDIC instance:
    - HAL_CORDIC_SetUserData(): Set user data in handler.
    - HAL_CORDIC_GetUserData(): Get user data from handler.
    - HAL_CORDIC_GetInstance(): Get the HAL CORDIC instance.
    - HAL_CORDIC_GetLLInstance(): Get the hardware CORDIC instance.
  */
#if defined(USE_HAL_CORDIC_USER_DATA) && (USE_HAL_CORDIC_USER_DATA == 1)
/**
  * @brief Store user data pointer into the handle.
  * @param   hcordic     Pointer to a \ref hal_cordic_handle_t.
  * @param   p_user_data Pointer to the user data.
  */
void HAL_CORDIC_SetUserData(hal_cordic_handle_t *hcordic, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  hcordic->p_user_data = p_user_data;
}

/**
  * @brief Retrieve user data pointer from the handle.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval void Pointer to the user data.
  */
const void *HAL_CORDIC_GetUserData(const hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);

  return (hcordic->p_user_data);
}
#endif /* USE_HAL_CORDIC_USER_DATA */

/**
  * @brief  Get the CORDIC instance.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t structure.
  * @retval HAL_CORDIC HAL CORDIC instance.
  */
hal_cordic_t HAL_CORDIC_GetInstance(const hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(IS_CORDIC_ALL_INSTANCE((CORDIC_TypeDef *)((uint32_t)hcordic->instance)));

  return hcordic->instance;
}

/**
  * @brief  Get the hardware CORDIC instance.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t structure.
  * @retval CORDIC CORDIC peripheral instance.
  */
CORDIC_TypeDef *HAL_CORDIC_GetLLInstance(const hal_cordic_handle_t *hcordic)
{
  ASSERT_DBG_PARAM(hcordic != NULL);
  ASSERT_DBG_PARAM(IS_CORDIC_ALL_INSTANCE((CORDIC_TypeDef *)((uint32_t)hcordic->instance)));

  return ((CORDIC_TypeDef *)((uint32_t)((hcordic)->instance)));
}
/**
  * @}
  */

/**
  * @}
  */    /* CORDIC_Exported_Functions*/

/** @addtogroup CORDIC_Private_Functions
  * @{
  */


/**
  * @brief Calculate and Validate the number of computation to be performed.
  * @param  hcordic              Pointer to a \ref hal_cordic_handle_t.
  * @param  p_input_buffer_desc Pointer to the input buffer descriptor.
  * @note   This function calculates the number of computation using:
  *             - The buffer size
  *             - The computation configuration (number of arguments and size of arguments).
  * @retval  computed_number_write the number of write given the set parameters and the input buffer size.
  */
static uint32_t CORDIC_ValidateWriteNumber(const hal_cordic_handle_t *hcordic,
                                           const hal_cordic_buffer_desc_t *p_input_buffer_desc)
{
  /* number of word per computation given the function configuration */
  /*                   \NARGS |  (1)  |   (2)                     */
  /*            ARG_SIZE \    |   0   |    1                      */
  /*                 -------------------------                    */
  /*         (32-bit)   0     |   1   |    2                      */
  /*                 -------------------------                    */
  /*         (16-bit)   1     |   1   |    1                      */
  /*                 -------------------------                    */

  CORDIC_TypeDef *p_cordic;
  p_cordic = CORDIC_GET_INSTANCE(hcordic);
  /* Extract argument size and number of arguments */
  hal_cordic_in_width_t arg_width = (hal_cordic_in_width_t)LL_CORDIC_GetInWidth(p_cordic);
  hal_cordic_nb_arg_t arg_number = (hal_cordic_nb_arg_t)LL_CORDIC_GetNbWrite(p_cordic);

  /* Initialize computed_number_write with the size of the input buffer */
  uint32_t computed_number_write = p_input_buffer_desc->size_word;

  /* Adjust computed_number_write based on argument size and number of arguments */
  if ((arg_width == HAL_CORDIC_IN_WIDTH_32_BIT) && (arg_number == HAL_CORDIC_NB_ARG_2))
  {
    computed_number_write >>= 1; /* Equivalent to dividing by 2 */
  }

  return computed_number_write;
}

/**
  * @brief Calculate and Validate the number of computation to read.
  * @param  hcordic              Pointer to a \ref hal_cordic_handle_t.
  * @param  p_output_buffer_desc Pointer to the output buffer descriptor.
  * @note   This function calculates the number of computation using:
  *           - The buffer size
  *           - The computation configuration (number of results and size of results).
  * @retval  computed_number_read the number of results given the set parameters and the output buffer size.
  */
static uint32_t CORDIC_ValidateReadNumber(const hal_cordic_handle_t *hcordic,
                                          const hal_cordic_buffer_desc_t *p_output_buffer_desc)
{
  /* number of word per computation given the function configuration */
  /*                   \NARGS |  (1)  |   (2)                     */
  /*            ARG_SIZE \    |   0   |    1                      */
  /*                 -------------------------                    */
  /*         (32-bit)   0     |   1   |    2                      */
  /*                 -------------------------                    */
  /*         (16-bit)   1     |   1   |    1                      */
  /*                 -------------------------                    */

  CORDIC_TypeDef *p_cordic;
  p_cordic = CORDIC_GET_INSTANCE(hcordic);
  /* Extract result size and number of results */
  hal_cordic_out_width_t res_width  = (hal_cordic_out_width_t)LL_CORDIC_GetOutWidth(p_cordic);
  hal_cordic_nb_result_t res_number = (hal_cordic_nb_result_t)LL_CORDIC_GetNbRead(p_cordic);

  /* Initialize computed_number_read with the size of the output buffer */
  uint32_t computed_number_read = p_output_buffer_desc->size_word;

  /* Adjust computed_number_read based on result size and number of results */
  if ((res_width == HAL_CORDIC_OUT_WIDTH_32_BIT) && (res_number == HAL_CORDIC_NB_RESULT_2))
  {
    computed_number_read >>= 1; /* Equivalent to dividing by 2 */
  }

  return computed_number_read;
}

/**
  * @brief Write blank argument.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
   * @warning   This function has to be called after the calculation is finished in order to avoid an unexpected result
  *         when a 2 arguments function is used with only Arg1 set. Arg2 is set to its default value after reset (+1).
  */
static inline void CORDIC_ResetArguments(const hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Read the actual configuration */
  hal_cordic_config_t config;

  config.function  = (hal_cordic_function_t)LL_CORDIC_GetFunction(p_cordic);
  config.precision = (hal_cordic_precision_t)LL_CORDIC_GetPrecision(p_cordic);
  config.scaling_factor = (hal_cordic_scaling_factor_t)LL_CORDIC_GetScale(p_cordic);
  config.nb_arg = (hal_cordic_nb_arg_t)LL_CORDIC_GetNbWrite(p_cordic);
  config.nb_result = (hal_cordic_nb_result_t)LL_CORDIC_GetNbRead(p_cordic);
  config.in_width = (hal_cordic_in_width_t)LL_CORDIC_GetInWidth(p_cordic);
  config.out_width = (hal_cordic_out_width_t)LL_CORDIC_GetOutWidth(p_cordic);

  /* Program a Sine function with 2 arguments of 32-bit */
  LL_CORDIC_SetFunction(p_cordic, LL_CORDIC_FUNCTION_SINE);   /* Sine function    */
  LL_CORDIC_SetNbWrite(p_cordic, LL_CORDIC_NBWRITE_2);        /* Two arguments    */
  LL_CORDIC_SetInWidth(p_cordic, LL_CORDIC_INWIDTH_32_BIT);   /* 32-bit arguments */
  LL_CORDIC_SetNbRead(p_cordic, LL_CORDIC_NBREAD_2);          /* Two results      */
  LL_CORDIC_SetOutWidth(p_cordic, LL_CORDIC_OUTWIDTH_32_BIT); /* 32-bit results   */

  /* Write ARG1 - ZeroOverhead mode selected no need to wait until the output results are available.*/
  LL_CORDIC_WriteData(p_cordic, CORDIC_ARGUMENT1);
  /* Write ARG2 - ZeroOverhead mode selected no need to wait until the output results are available.*/
  LL_CORDIC_WriteData(p_cordic, CORDIC_ARGUMENT2);

  /* Read RES1 */
  (void)LL_CORDIC_ReadData(p_cordic);
  /* Read RES2 */
  (void)LL_CORDIC_ReadData(p_cordic);

  /* Set Initial configuration */
  /* Apply all configuration parameters in CORDIC control register */
  LL_CORDIC_Config(p_cordic, (uint32_t)(config.function), (uint32_t)(config.precision),
                   (uint32_t)(config.scaling_factor), (uint32_t)(config.nb_arg),
                   (uint32_t)(config.nb_result), (uint32_t)(config.in_width),
                   (uint32_t)(config.out_width));

}

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
/**
  * @brief Check that the given precision matches the selected function.
  * @param  function  Function to be checked.
  * @param  precision Value of the precision to be set.
  * @warning   This function has to be called in ASSERT_DBG_PARAM() macro inside HAL_CORDIC_SetPrecision() function.
  * @retval 1 The precision parameter matches the function requirements.
  * @retval 0 The precision parameter does not match the function requirements.
  */
static inline uint32_t CORDIC_CheckPrecision(hal_cordic_function_t function, hal_cordic_precision_t precision)
{
  /* Analyse the precision vs the function */
  switch (function)
  {
    case HAL_CORDIC_FUNCTION_SQUARE_ROOT:
      return ((precision >= HAL_CORDIC_PRECISION_1_CYCLE) && (precision <= HAL_CORDIC_PRECISION_3_CYCLE)) ? 1UL : 0UL;
      break;

    case HAL_CORDIC_FUNCTION_COSINE:
    case HAL_CORDIC_FUNCTION_SINE:
    case HAL_CORDIC_FUNCTION_PHASE:
    case HAL_CORDIC_FUNCTION_MODULUS:
    case HAL_CORDIC_FUNCTION_ARCTANGENT:
    case HAL_CORDIC_FUNCTION_HCOSINE:
    case HAL_CORDIC_FUNCTION_HSINE:
    case HAL_CORDIC_FUNCTION_HARCTANGENT:
    case HAL_CORDIC_FUNCTION_NATURAL_LOG:
      return ((precision >= HAL_CORDIC_PRECISION_1_CYCLE) && (precision <= HAL_CORDIC_PRECISION_6_CYCLE)) ? 1UL : 0UL;
      break;

    default:
      return 0UL;
      break;
  }
}

/**
  * @brief Check that the given scaling factor matches the selected function.
  * @param  function        Function to be checked.
  * @param  scaling_factor    Value of the scaling factor to be set.
  * @warning   This function has to be called in ASSERT_DBG_PARAM() macro inside HAL_CORDIC_SetScalingFactor() function.
  * @retval 1 The scaling factor parameter matches the function requirements.
  * @retval 0 The scaling factor parameter does not match the function requirements.
  */
static inline uint32_t CORDIC_CheckScalingFactor(hal_cordic_function_t function,
                                                 hal_cordic_scaling_factor_t scaling_factor)
{
  switch (function)
  {
    case HAL_CORDIC_FUNCTION_COSINE:
    case HAL_CORDIC_FUNCTION_SINE:
    case HAL_CORDIC_FUNCTION_PHASE:
    case HAL_CORDIC_FUNCTION_MODULUS:
      return (scaling_factor == HAL_CORDIC_SCALING_FACTOR_0) ? 1UL : 0UL;
      break;

    case HAL_CORDIC_FUNCTION_ARCTANGENT:
      /* Scale can be any value for arctangent */
      return 1UL;
      break;

    case HAL_CORDIC_FUNCTION_HCOSINE:
    case HAL_CORDIC_FUNCTION_HSINE:
    case HAL_CORDIC_FUNCTION_HARCTANGENT:
      return (scaling_factor == HAL_CORDIC_SCALING_FACTOR_1) ? 1UL : 0UL;
      break;

    case HAL_CORDIC_FUNCTION_NATURAL_LOG:
      return ((scaling_factor >= HAL_CORDIC_SCALING_FACTOR_1)
              && (scaling_factor <= HAL_CORDIC_SCALING_FACTOR_4)) ? 1UL : 0UL;
      break;

    case HAL_CORDIC_FUNCTION_SQUARE_ROOT:
      return ((scaling_factor > HAL_CORDIC_SCALING_FACTOR_0)
              && (scaling_factor <= HAL_CORDIC_SCALING_FACTOR_2)) ? 1UL : 0UL;
      break;

    default:
      return 0UL;
      break;
  }
}
#endif /* USE_HAL_CHECK_PARAM */

/**
  * @brief Write ARG1 for CORDIC processing, and increment input buffer pointer.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  pp_input_buffer Pointer to pointer to input buffer.
  */
static void CORDIC_WriteDataAndIncPtr_1(const hal_cordic_handle_t *hcordic, const int32_t **pp_input_buffer)
{
  CORDIC_TypeDef *p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Write input data to the Write Data register and increment the pointer */
  LL_CORDIC_WriteData(p_cordic, (uint32_t) **pp_input_buffer);
  (*pp_input_buffer)++;
}

/**
  * @brief Write ARG1 and ARG2 for CORDIC processing, and increment input buffer pointer.
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @param  pp_input_buffer Pointer to pointer to input buffer.
  */
static void CORDIC_WriteDataAndIncPtr_2(const hal_cordic_handle_t *hcordic, const int32_t **pp_input_buffer)
{
  CORDIC_TypeDef *p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Write input data to the Write Data register and increment the pointer */
  LL_CORDIC_WriteData(p_cordic, (uint32_t) **pp_input_buffer);
  (*pp_input_buffer)++;

  /* Write the second input data to the Write Data register and increment the pointer */
  LL_CORDIC_WriteData(p_cordic, (uint32_t) **pp_input_buffer);
  (*pp_input_buffer)++;
}

/**
  * @brief Read RES1 of CORDIC processing, and increment output buffer pointer.
  * @param  hcordic          Pointer to a \ref hal_cordic_handle_t.
  * @param  pp_output_buffer Pointer to pointer to output buffer.
  */
static void CORDIC_ReadDataAndIncPtr_1(const hal_cordic_handle_t *hcordic, int32_t **pp_output_buffer)
{
  CORDIC_TypeDef *p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Read output data of the Read Data register and increment the pointer */
  **pp_output_buffer = (int32_t)LL_CORDIC_ReadData(p_cordic);
  (*pp_output_buffer)++;
}
/**
  * @brief Read RES1 and RES2 of CORDIC processing, and increment output buffer pointer.
  * @param  hcordic          Pointer to a \ref hal_cordic_handle_t.
  * @param  pp_output_buffer Pointer to pointer to output buffer.
  */
static void CORDIC_ReadDataAndIncPtr_2(const hal_cordic_handle_t *hcordic, int32_t **pp_output_buffer)
{
  CORDIC_TypeDef *p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Read output data of the Read Data register and increment the pointer */
  **pp_output_buffer = (int32_t)LL_CORDIC_ReadData(p_cordic);
  (*pp_output_buffer)++;

  /* Read output data of the Read Data register and increment the pointer */
  **pp_output_buffer = (int32_t)LL_CORDIC_ReadData(p_cordic);
  (*pp_output_buffer)++;
}

/**
  * @brief Abort the ongoing transfer (blocking process).
  * @param  hcordic Pointer to a \ref hal_cordic_handle_t.
  * @retval HAL_OK Operation completed successfully.
  */
static hal_status_t CORDIC_Abort(hal_cordic_handle_t *hcordic)
{
  CORDIC_TypeDef *p_cordic;

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

#if defined(USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
  /* Check if the DMA Read (output) is enabled */
  if (LL_CORDIC_IsEnabledDMAReq_RD(p_cordic) != 0U)
  {
    LL_CORDIC_DisableDMAReq_RD(p_cordic);

    if (hcordic->p_dma_out != NULL)
    {
      (void)HAL_DMA_Abort(hcordic->p_dma_out);

      hcordic->p_output_buffer = NULL;
      hcordic->result_nb = 0U;
    }
  }

  /* Check if the DMA Write (input) is enabled */
  if (LL_CORDIC_IsEnabledDMAReq_WR(p_cordic) != 0U)
  {
    LL_CORDIC_DisableDMAReq_WR(p_cordic);

    if (hcordic->p_dma_in != NULL)
    {
      (void)HAL_DMA_Abort(hcordic->p_dma_in);

      hcordic->p_input_buffer = NULL;
      hcordic->computation_nb = 0U;
    }
  }
#endif /* USE_HAL_CORDIC_DMA */

  hcordic->p_wr_arg        = NULL;
  hcordic->p_rd_result     = NULL;

  LL_CORDIC_DisableIT(p_cordic);
#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  return HAL_OK;
}

#if defined(USE_HAL_CORDIC_DMA) && (USE_HAL_CORDIC_DMA == 1)
/**
  * @brief Write input arguments in DMA mode with option. Global state must be IDLE.
  * @param  hcordic   Pointer to a \ref hal_cordic_handle_t.
  * @param  p_in_buff Pointer to input data descriptor: pointer to data and input size in word.
  * @param  opt_it    Optional interruption can be a combination of
  *                   \ref HAL_CORDIC_OPT_DMA_NONE
  *                   \ref HAL_CORDIC_OPT_DMA_IT_HALF_CPLT.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  */
static hal_status_t CORDIC_Write_DMA_opt(hal_cordic_handle_t *hcordic, const hal_cordic_buffer_desc_t *p_in_buff,
                                         uint32_t opt_it)
{
  CORDIC_TypeDef *p_cordic;
  uint32_t size_input_buffer;
  uint32_t nb_write;

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  nb_write = CORDIC_ValidateWriteNumber(hcordic, p_in_buff);

  /* Check number of input data expected for each calculation, to retrieve the size of input data buffer */
  size_input_buffer = ((LL_CORDIC_GetNbWrite(p_cordic) == (uint32_t)(HAL_CORDIC_NB_ARG_2))
                       ? (2U * nb_write) : nb_write);

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

  /* Prepare DMA write xfer */
  hcordic->p_dma_in->p_xfer_cplt_cb        = CORDIC_DMAInCplt;     /* Set the CORDIC DMA transfer complete callback */
  hcordic->p_dma_in->p_xfer_halfcplt_cb    = CORDIC_DMAInHalfCplt; /* Set the CORDIC DMA transfer half complete cb  */
  hcordic->p_dma_in->p_xfer_error_cb       = CORDIC_DMAError;      /* Set the DMA error callback                    */

  /* Convert the input buffer size into corresponding number of bytes as DMA handles data at byte-level. */
  size_input_buffer = 4U * size_input_buffer;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hcordic->p_dma_in, (uint32_t)p_in_buff->p_data,
                                     (uint32_t)&p_cordic->WDATA, size_input_buffer, opt_it) != HAL_OK)
  {
#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
    hcordic->last_error_codes |= HAL_CORDIC_ERROR_DMA;
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  /* Enable DMA Write request*/
  LL_CORDIC_EnableDMAReq_WR(p_cordic);

  return HAL_OK;
}

/**
  * @brief Read output result in DMA mode. Global state must be IDLE.
  * @param  hcordic    Pointer to a \ref hal_cordic_handle_t.
  * @param  p_out_buff Pointer to output data descriptor: pointer to data and output size in word.
  * @param  opt_it     Optional interruption can be a combination of the following values:
  *                    @arg \ref HAL_CORDIC_OPT_DMA_NONE
  *                    @arg \ref HAL_CORDIC_OPT_DMA_IT_HALF_CPLT.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_ERROR         Operation completed with error.
  */
static hal_status_t CORDIC_Read_DMA_opt(hal_cordic_handle_t *hcordic, hal_cordic_buffer_desc_t *p_out_buff,
                                        uint32_t opt_it)
{
  CORDIC_TypeDef *p_cordic;
  uint32_t size_output_buffer;
  uint32_t nb_read;

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  nb_read = CORDIC_ValidateReadNumber(hcordic, p_out_buff);

  /* Check number of input data expected for each calculation, to retrieve the size of input data buffer */
  size_output_buffer = ((LL_CORDIC_GetNbRead(p_cordic) == (uint32_t)(HAL_CORDIC_NB_RESULT_2))
                        ? (2U * nb_read) : nb_read);

#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes = HAL_CORDIC_ERROR_NONE;

#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */
  /* Prepare DMA write xfer */
  hcordic->p_dma_out->p_xfer_cplt_cb        = CORDIC_DMAOutCplt;     /* Set the CORDIC DMA transfer complete callback */
  hcordic->p_dma_out->p_xfer_halfcplt_cb    = CORDIC_DMAOutHalfCplt; /* Set the CORDIC DMA transfer half complete cb  */
  hcordic->p_dma_out->p_xfer_error_cb       = CORDIC_DMAError;       /* Set the DMA error callback                    */

  /* Convert the input buffer size into corresponding number of bytes as DMA handles data at byte-level. */
  size_output_buffer = 4U * size_output_buffer;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hcordic->p_dma_out, (uint32_t) &(p_cordic->RDATA),
                                     (uint32_t)p_out_buff->p_data, size_output_buffer, opt_it) != HAL_OK)
  {
#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
    hcordic->last_error_codes |= HAL_CORDIC_ERROR_DMA;
#endif /* USE_HAL_CORDIC_GET_LAST_ERRORS */
    return HAL_ERROR;
  }

  /* Enable DMA Read request*/
  LL_CORDIC_EnableDMAReq_RD(p_cordic);

  return HAL_OK;
}

/**
  * @brief DMA CORDIC Input Data process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t.
  */
static void CORDIC_DMAInCplt(hal_dma_handle_t *hdma)
{
  CORDIC_TypeDef *p_cordic;

  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* Check if DMA in circular mode*/
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    hcordic->computation_nb = 0UL;

    p_cordic = CORDIC_GET_INSTANCE(hcordic);

    /* Disable the DMA transfer for input request */
    LL_CORDIC_DisableDMAReq_WR(p_cordic);
  }

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_write_cplt_cb(hcordic);
#else
  HAL_CORDIC_WriteCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief DMA CORDIC Input Data process half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t.
  */
static void CORDIC_DMAInHalfCplt(hal_dma_handle_t *hdma)
{
  CORDIC_TypeDef *p_cordic;

  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Disable the DMA transfer for input request */
  LL_CORDIC_DisableDMAReq_WR(p_cordic);

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_write_half_cplt_cb(hcordic);
#else
  HAL_CORDIC_WriteHalfCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief DMA CORDIC Output Data process complete callback.
  * @param hdma Pointer to a hal_dma_handle_t.
  */
static void CORDIC_DMAOutCplt(hal_dma_handle_t *hdma)
{
  CORDIC_TypeDef *p_cordic;

  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* Check if DMA in circular mode*/
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    hcordic->result_nb = 0UL;

    hcordic->global_state = HAL_CORDIC_STATE_IDLE;

    /* Disable the DMA transfer for output request */
    LL_CORDIC_DisableDMAReq_RD(p_cordic);

    /* Stop the interrupts error handling */
    LL_CORDIC_DisableIT(p_cordic);
  }

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_calculate_cplt_cb(hcordic);
#else
  HAL_CORDIC_CalculateCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief DMA CORDIC Output Data read half complete callback.
  * @param hdma Pointer to a hal_dma_handle_t.
  */
static void CORDIC_DMAOutHalfCplt(hal_dma_handle_t *hdma)
{

  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_read_half_cplt_cb(hcordic);
#else
  HAL_CORDIC_ReadHalfCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief DMA CORDIC communication error callback.
  * @param hdma Pointer to a hal_dma_handle_t.
  */
static void CORDIC_DMAError(hal_dma_handle_t *hdma)
{
  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;
#if defined(USE_HAL_CORDIC_GET_LAST_ERRORS) && (USE_HAL_CORDIC_GET_LAST_ERRORS == 1)
  hcordic->last_error_codes |= HAL_CORDIC_ERROR_DMA;
#endif  /* USE_HAL_CORDIC_GET_LAST_ERRORS */

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_error_cb(hcordic);
#else
  HAL_CORDIC_ErrorCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA  Abort callback.
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void CORDIC_DMAAbort(hal_dma_handle_t *hdma)
{
  CORDIC_TypeDef *p_cordic;

  hal_cordic_handle_t *hcordic = (hal_cordic_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  p_cordic = CORDIC_GET_INSTANCE(hcordic);

  /* Disable any DMA requests */
  LL_CORDIC_DisableDMAReq_WR(p_cordic);

  LL_CORDIC_DisableDMAReq_RD(p_cordic);

  /* Disable interrupt */
  LL_CORDIC_DisableIT(p_cordic);

  hcordic->p_input_buffer  = NULL;
  hcordic->p_output_buffer = NULL;
  hcordic->computation_nb = 0U;
  hcordic->result_nb      = 0U;

  hcordic->global_state = HAL_CORDIC_STATE_IDLE;

#if defined(USE_HAL_CORDIC_REGISTER_CALLBACKS) && (USE_HAL_CORDIC_REGISTER_CALLBACKS == 1)
  hcordic->p_abort_cplt_cb(hcordic);
#else
  HAL_CORDIC_AbortCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

#endif /* USE_HAL_CORDIC_DMA */

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_CORDIC_MODULE */
#endif /* CORDIC */
/**
  * @}
  */
