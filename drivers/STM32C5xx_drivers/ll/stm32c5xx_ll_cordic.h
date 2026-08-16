/**
  ******************************************************************************
  * @file    stm32c5xx_ll_cordic.h
  * @brief   Header file of CORDIC LL module.
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
#ifndef STM32C5XX_LL_CORDIC_H
#define STM32C5XX_LL_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined(CORDIC)

/** @defgroup CORDIC_LL CORDIC
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CORDIC_LL_Exported_Constants LL CORDIC Constants
  * @{
  */

/** @defgroup CORDIC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flag defines that can be used with the LL_CORDIC_READ_REG function.
  * @{
  */
#define LL_CORDIC_FLAG_RRDY CORDIC_CSR_RRDY
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_WRITE_FLAG DMA WRITE Flags Defines
  * @brief    Flag defines whether DMA write is enabled.
  * @{
  */
#define LL_CORDIC_FLAG_DMAWEN CORDIC_CSR_DMAWEN
/**
  * @}
  */

/** @defgroup CORDIC_LL_READ_GET_FLAG DMA READ Flags Defines
  * @brief    Flag defines whether DMA read is enabled.
  * @{
  */
#define LL_CORDIC_FLAG_DMAREN CORDIC_CSR_DMAREN
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_IT IT Defines
  * @brief    IT defines that can be used with LL_CORDIC_READ_REG and LL_CORDIC_WriteReg functions.
  * @{
  */
#define LL_CORDIC_IT_IEN CORDIC_CSR_IEN /*!< Result Ready interrupt enable */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_FUNCTION FUNCTION
  * @{
  */
#define LL_CORDIC_FUNCTION_COSINE      (0x00000000U)                                       /*!< Cosine                */
#define LL_CORDIC_FUNCTION_SINE        ((uint32_t)(CORDIC_CSR_FUNC_0))                     /*!< Sine                  */
#define LL_CORDIC_FUNCTION_PHASE       ((uint32_t)(CORDIC_CSR_FUNC_1))                     /*!< Phase                 */
#define LL_CORDIC_FUNCTION_MODULUS     ((uint32_t)(CORDIC_CSR_FUNC_1 | CORDIC_CSR_FUNC_0)) /*!< Modulus               */
#define LL_CORDIC_FUNCTION_ARCTANGENT  ((uint32_t)(CORDIC_CSR_FUNC_2))                     /*!< Arctangent            */
#define LL_CORDIC_FUNCTION_HCOSINE     ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_0)) /*!< Hyperbolic Cosine     */
#define LL_CORDIC_FUNCTION_HSINE       ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_1)) /*!< Hyperbolic Sine       */
#define LL_CORDIC_FUNCTION_HARCTANGENT ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_1 \
                                                   | CORDIC_CSR_FUNC_0))                   /*!< Hyperbolic Arctangent */
#define LL_CORDIC_FUNCTION_NATURALLOG  ((uint32_t)(CORDIC_CSR_FUNC_3))                     /*!< Natural Logarithm     */
#define LL_CORDIC_FUNCTION_SQUAREROOT  ((uint32_t)(CORDIC_CSR_FUNC_3 | CORDIC_CSR_FUNC_0)) /*!< Square Root           */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_PRECISION PRECISION
  * @{
  */
#define LL_CORDIC_PRECISION_1_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_0))                            /*!< 1 cycle   */
#define LL_CORDIC_PRECISION_2_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_1))                            /*!< 2 cycles  */
#define LL_CORDIC_PRECISION_3_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0))   /*!< 3 cycles  */
#define LL_CORDIC_PRECISION_4_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_2))                            /*!< 4 cycles  */
#define LL_CORDIC_PRECISION_5_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0))   /*!< 5 cycles  */
#define LL_CORDIC_PRECISION_6_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1))   /*!< 6 cycles  */
#define LL_CORDIC_PRECISION_7_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_2 \
                                                 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0)) /*!< 7 cycles  */
#define LL_CORDIC_PRECISION_8_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_3))                            /*!< 8 cycles  */
#define LL_CORDIC_PRECISION_9_CYCLE  ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_0))   /*!< 9 cycles  */
#define LL_CORDIC_PRECISION_10_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_1))   /*!< 10 cycles */
#define LL_CORDIC_PRECISION_11_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 \
                                                 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0)) /*!< 11 cycles */
#define LL_CORDIC_PRECISION_12_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_2))   /*!< 12 cycles */
#define LL_CORDIC_PRECISION_13_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 \
                                                 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0)) /*!< 13 cycles */
#define LL_CORDIC_PRECISION_14_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 \
                                                 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1)) /*!< 14 cycles */
#define LL_CORDIC_PRECISION_15_CYCLE ((uint32_t)(CORDIC_CSR_PRECISION_3 \
                                                 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1 \
                                                 | CORDIC_CSR_PRECISION_0))                          /*!< 15 cycles */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_SCALE SCALE
  * @{
  */
#define LL_CORDIC_SCALING_FACTOR_0 (0x00000000U)                      /*!< Scaling factor - Arguments * 2^0 */
#define LL_CORDIC_SCALING_FACTOR_1 ((uint32_t)(CORDIC_CSR_SCALE_0))   /*!< Scaling factor - Arguments * 2^1 */
#define LL_CORDIC_SCALING_FACTOR_2 ((uint32_t)(CORDIC_CSR_SCALE_1))   /*!< Scaling factor - Arguments * 2^2 */
#define LL_CORDIC_SCALING_FACTOR_3 ((uint32_t)(CORDIC_CSR_SCALE_1 \
                                               | CORDIC_CSR_SCALE_0)) /*!< Scaling factor - Arguments * 2^3 */
#define LL_CORDIC_SCALING_FACTOR_4 ((uint32_t)(CORDIC_CSR_SCALE_2))   /*!< Scaling factor - Arguments * 2^4 */
#define LL_CORDIC_SCALING_FACTOR_5 ((uint32_t)(CORDIC_CSR_SCALE_2 \
                                               | CORDIC_CSR_SCALE_0)) /*!< Scaling factor - Arguments * 2^5 */
#define LL_CORDIC_SCALING_FACTOR_6 ((uint32_t)(CORDIC_CSR_SCALE_2 \
                                               | CORDIC_CSR_SCALE_1)) /*!< Scaling factor - Arguments * 2^6 */
#define LL_CORDIC_SCALING_FACTOR_7 ((uint32_t)(CORDIC_CSR_SCALE_2 | CORDIC_CSR_SCALE_1 \
                                               | CORDIC_CSR_SCALE_0)) /*!< Scaling factor - Arguments * 2^7 */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_NBWRITE NBWRITE
  * @{
  */
#define LL_CORDIC_NBWRITE_1 (0x00000000U)    /*!< One 32-bit write containing either only one
                                                  32-bit data input (Q1.31 format), or two
                                                  16-bit data input (Q1.15 format) packed
                                                  in one 32-bit Data */
#define LL_CORDIC_NBWRITE_2 CORDIC_CSR_NARGS /*!< Two 32-bit write containing two 32-bit data input
                                                  (Q1.31 format) */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_NBREAD NBREAD
  * @{
  */
#define LL_CORDIC_NBREAD_1 (0x00000000U)   /*!< One 32-bit read containing either only one
                                                32-bit data output (Q1.31 format), or two
                                                16-bit data output (Q1.15 format) packed
                                                in one 32-bit Data */
#define LL_CORDIC_NBREAD_2 CORDIC_CSR_NRES /*!< Two 32-bit Data containing two 32-bit data output
                                                (Q1.31 format) */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_INWIDTH INWIDTH
  * @{
  */
#define LL_CORDIC_INWIDTH_32_BIT (0x00000000U)      /*!< 32-bit input data width (Q1.31 format) */
#define LL_CORDIC_INWIDTH_16_BIT CORDIC_CSR_ARGSIZE /*!< 16-bit input data width (Q1.15 format) */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_OUTWIDTH OUTWIDTH
  * @{
  */
#define LL_CORDIC_OUTWIDTH_32_BIT (0x00000000U)      /*!< 32-bit output data width (Q1.31 format) */
#define LL_CORDIC_OUTWIDTH_16_BIT CORDIC_CSR_RESSIZE /*!< 16-bit output data width (Q1.15 format) */
/**
  * @}
  */

/** @defgroup CORDIC_LL_EC_DMA_REG_DATA DMA register data
  * @{
  */
#define LL_CORDIC_DMA_REG_DATA_IN  (0x00000000U) /*!< Get address of input data register  */
#define LL_CORDIC_DMA_REG_DATA_OUT (0x00000001U) /*!< Get address of output data register */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CORDIC_LL_Exported_Macros LL CORDIC Macros
  * @{
  */

/** @defgroup CORDIC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value to a CORDIC register.
  * @param  instance CORDIC Instance
  * @param  reg Register to be written
  * @param value Value to be written in the register
  */
#define LL_CORDIC_WRITE_REG(instance, reg,value) STM32_WRITE_REG(instance->reg, (value))

/**
  * @brief  Read a value from a CORDIC register.
  * @param  instance CORDIC Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_CORDIC_READ_REG(instance, reg) STM32_READ_REG(instance->reg)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

/** @defgroup CORDIC_LL_Exported_Functions LL CORDIC Functions
  * @{
  */

/** @defgroup CORDIC_LL_EF_Configuration CORDIC Configuration functions
  * @{
  */

/**
  * @brief  Configure the CORDIC processing.
  * @param  p_cordic CORDIC instance
  * @param  function parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_FUNCTION_COSINE
  *         @arg @ref LL_CORDIC_FUNCTION_SINE
  *         @arg @ref LL_CORDIC_FUNCTION_PHASE
  *         @arg @ref LL_CORDIC_FUNCTION_MODULUS
  *         @arg @ref LL_CORDIC_FUNCTION_ARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_HCOSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_NATURALLOG
  *         @arg @ref LL_CORDIC_FUNCTION_SQUAREROOT
  * @param  precision parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_PRECISION_1_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_2_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_3_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_4_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_5_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_6_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_7_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_8_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_9_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_10_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_11_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_12_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_13_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_14_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_15_CYCLE
  * @param  scale parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_0
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_1
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_2
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_3
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_4
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_5
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_6
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_7
  * @param  number_write parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_NBWRITE_1
  *         @arg @ref LL_CORDIC_NBWRITE_2
  * @param  number_read parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_NBREAD_1
  *         @arg @ref LL_CORDIC_NBREAD_2
  * @param  input_size parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_INWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_INWIDTH_16_BIT
  * @param  output_size parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_OUTWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_OUTWIDTH_16_BIT
  * @note   This function set all parameters of CORDIC processing.
  *         These parameters can also be set individually using
  *         dedicated functions:
  *         - @ref LL_CORDIC_SetFunction()
  *         - @ref LL_CORDIC_SetPrecision()
  *         - @ref LL_CORDIC_SetScale()
  *         - @ref LL_CORDIC_SetNbWrite()
  *         - @ref LL_CORDIC_SetNbRead()
  *         - @ref LL_CORDIC_SetInWidth()
  *         - @ref LL_CORDIC_SetOutWidth()
  * @rmtoll
  *  CSR          FUNC          LL_CORDIC_Config \n
  *  CSR          PRECISION     LL_CORDIC_Config \n
  *  CSR          SCALE         LL_CORDIC_Config \n
  *  CSR          NARGS         LL_CORDIC_Config \n
  *  CSR          NRES          LL_CORDIC_Config \n
  *  CSR          ARGSIZE       LL_CORDIC_Config \n
  *  CSR          RESIZE        LL_CORDIC_Config
  */
__STATIC_INLINE void LL_CORDIC_Config(CORDIC_TypeDef *p_cordic, uint32_t function, uint32_t precision, uint32_t scale,
                                      uint32_t number_write, uint32_t number_read, uint32_t input_size,
                                      uint32_t output_size)
{
  STM32_MODIFY_REG(p_cordic->CSR,
                   CORDIC_CSR_FUNC | CORDIC_CSR_PRECISION | CORDIC_CSR_SCALE |
                   CORDIC_CSR_NARGS | CORDIC_CSR_NRES | CORDIC_CSR_ARGSIZE | CORDIC_CSR_RESSIZE,
                   function | precision | scale |
                   number_write | number_read | input_size | output_size);
}

/**
  * @brief  Configure the function.
  * @rmtoll
  *  CSR          FUNC          LL_CORDIC_SetFunction
  * @param  p_cordic CORDIC Instance
  * @param  function parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_FUNCTION_COSINE
  *         @arg @ref LL_CORDIC_FUNCTION_SINE
  *         @arg @ref LL_CORDIC_FUNCTION_PHASE
  *         @arg @ref LL_CORDIC_FUNCTION_MODULUS
  *         @arg @ref LL_CORDIC_FUNCTION_ARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_HCOSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_NATURALLOG
  *         @arg @ref LL_CORDIC_FUNCTION_SQUAREROOT
  */
__STATIC_INLINE void LL_CORDIC_SetFunction(CORDIC_TypeDef *p_cordic, uint32_t function)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_FUNC, function);
}

/**
  * @brief  Return the function.
  * @rmtoll
  *  CSR          FUNC          LL_CORDIC_GetFunction
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_FUNCTION_COSINE
  *         @arg @ref LL_CORDIC_FUNCTION_SINE
  *         @arg @ref LL_CORDIC_FUNCTION_PHASE
  *         @arg @ref LL_CORDIC_FUNCTION_MODULUS
  *         @arg @ref LL_CORDIC_FUNCTION_ARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_HCOSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HSINE
  *         @arg @ref LL_CORDIC_FUNCTION_HARCTANGENT
  *         @arg @ref LL_CORDIC_FUNCTION_NATURALLOG
  *         @arg @ref LL_CORDIC_FUNCTION_SQUAREROOT
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetFunction(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_FUNC));
}

/**
  * @brief  Configure precision in cycles number.
  * @rmtoll
  *  CSR          PRECISION     LL_CORDIC_SetPrecision
  * @param  p_cordic CORDIC Instance
  * @param  precision parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_PRECISION_1_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_2_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_3_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_4_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_5_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_6_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_7_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_8_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_9_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_10_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_11_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_12_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_13_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_14_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_15_CYCLE
  */
__STATIC_INLINE void LL_CORDIC_SetPrecision(CORDIC_TypeDef *p_cordic, uint32_t precision)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_PRECISION, precision);
}

/**
  * @brief  Return the precision in cycle count.
  * @rmtoll
  *  CSR          PRECISION     LL_CORDIC_GetPrecision
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_PRECISION_1_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_2_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_3_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_4_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_5_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_6_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_7_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_8_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_9_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_10_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_11_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_12_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_13_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_14_CYCLE
  *         @arg @ref LL_CORDIC_PRECISION_15_CYCLE
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetPrecision(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_PRECISION));
}

/**
  * @brief  Configure scaling factor.
  * @rmtoll
  *  CSR          SCALE         LL_CORDIC_SetScale
  * @param  p_cordic CORDIC Instance
  * @param  scale parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_0
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_1
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_2
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_3
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_4
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_5
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_6
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_7
  */
__STATIC_INLINE void LL_CORDIC_SetScale(CORDIC_TypeDef *p_cordic, uint32_t scale)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_SCALE, scale);
}

/**
  * @brief  Return the scaling factor.
  * @rmtoll
  *  CSR          SCALE         LL_CORDIC_GetScale
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_0
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_1
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_2
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_3
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_4
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_5
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_6
  *         @arg @ref LL_CORDIC_SCALING_FACTOR_7
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetScale(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_SCALE));
}

/**
  * @brief  Configure number of 32-bit write expected for one calculation.
  * @rmtoll
  *  CSR          NARGS         LL_CORDIC_SetNbWrite
  * @param  p_cordic CORDIC Instance
  * @param  number_write parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_NBWRITE_1
  *         @arg @ref LL_CORDIC_NBWRITE_2
  */
__STATIC_INLINE void LL_CORDIC_SetNbWrite(CORDIC_TypeDef *p_cordic, uint32_t number_write)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_NARGS, number_write);
}

/**
  * @brief  Return the number of 32-bit writes expected for one calculation.
  * @rmtoll
  *  CSR          NARGS         LL_CORDIC_GetNbWrite
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_NBWRITE_1
  *         @arg @ref LL_CORDIC_NBWRITE_2
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetNbWrite(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_NARGS));
}

/**
  * @brief  Configure number of 32-bit read expected after one calculation.
  * @rmtoll
  *  CSR          NRES          LL_CORDIC_SetNbRead
  * @param  p_cordic CORDIC Instance
  * @param  number_read parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_NBREAD_1
  *         @arg @ref LL_CORDIC_NBREAD_2
  */
__STATIC_INLINE void LL_CORDIC_SetNbRead(CORDIC_TypeDef *p_cordic, uint32_t number_read)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_NRES, number_read);
}

/**
  * @brief  Return the number of 32-bit reads expected after one calculation.
  * @rmtoll
  *  CSR          NRES          LL_CORDIC_GetNbRead
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_NBREAD_1
  *         @arg @ref LL_CORDIC_NBREAD_2
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetNbRead(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_NRES));
}

/**
  * @brief  Configure width of input data.
  * @rmtoll
  *  CSR          ARGSIZE       LL_CORDIC_SetInWidth
  * @param  p_cordic CORDIC Instance
  * @param  input_size parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_INWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_INWIDTH_16_BIT
  */
__STATIC_INLINE void LL_CORDIC_SetInWidth(CORDIC_TypeDef *p_cordic, uint32_t input_size)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_ARGSIZE, input_size);
}

/**
  * @brief  Return the width of input data.
  * @rmtoll
  *  CSR          ARGSIZE       LL_CORDIC_GetInWidth
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_INWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_INWIDTH_16_BIT
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetInWidth(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_ARGSIZE));
}

/**
  * @brief  Configure width of output data.
  * @rmtoll
  *  CSR          RESIZE       LL_CORDIC_SetOutWidth
  * @param  p_cordic CORDIC Instance
  * @param  output_size parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_OUTWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_OUTWIDTH_16_BIT
  */
__STATIC_INLINE void LL_CORDIC_SetOutWidth(CORDIC_TypeDef *p_cordic, uint32_t output_size)
{
  STM32_MODIFY_REG(p_cordic->CSR, CORDIC_CSR_RESSIZE, output_size);
}

/**
  * @brief  Return the width of output data.
  * @rmtoll
  *  CSR          RESIZE       LL_CORDIC_GetOutWidth
  * @param  p_cordic CORDIC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CORDIC_OUTWIDTH_32_BIT
  *         @arg @ref LL_CORDIC_OUTWIDTH_16_BIT
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetOutWidth(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_RESSIZE));
}

/**
  * @}
  */

/** @defgroup CORDIC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable the CORDIC interrupt when result is ready.
  * @rmtoll
  *  CSR          IEN         LL_CORDIC_EnableIT
  * @param  p_cordic pointer to CORDIC handle.
  */
__STATIC_INLINE void LL_CORDIC_EnableIT(CORDIC_TypeDef *p_cordic)
{
  STM32_SET_BIT(p_cordic->CSR, LL_CORDIC_IT_IEN);
}

/**
  * @brief  Disable the CORDIC interrupt.
  * @rmtoll
  *  CSR          IEN         LL_CORDIC_DisableIT
  * @param  p_cordic pointer to CORDIC handle.
  */
__STATIC_INLINE void LL_CORDIC_DisableIT(CORDIC_TypeDef *p_cordic)
{
  STM32_CLEAR_BIT(p_cordic->CSR, LL_CORDIC_IT_IEN);
}

/**
  * @brief  Check whether the specified CORDIC status flag is set or not.
  * @param  p_cordic pointer to CORDIC handle.
  * @param  mask CORDIC flag to check
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_FLAG_RRDY Result Ready Flag
  * @retval 1UL (flag is set) or 0UL (flag is reset)
  */
__STATIC_INLINE uint32_t LL_CORDIC_IsActiveFlag(const CORDIC_TypeDef *p_cordic, uint32_t mask)
{
  return ((STM32_READ_BIT(p_cordic->CSR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the specified CORDIC interrupt is enabled.
  * @rmtoll
  *  CSR          IEN           LL_CORDIC_IsEnabledIT
  * @param  p_cordic pointer to CORDIC handle.
  * @retval 1UL (flag is set) or 0UL (flag is reset)
  */
__STATIC_INLINE uint32_t LL_CORDIC_IsEnabledIT(const CORDIC_TypeDef *p_cordic)
{
  return ((STM32_READ_BIT(p_cordic->CSR, LL_CORDIC_IT_IEN) == (LL_CORDIC_IT_IEN)) ? 1UL : 0UL);
}

/** @brief  Check whether the specified CORDIC interrupt is enabled or not.
  * @param  p_cordic CORDIC instance.
  * @param  interrupt CORDIC interrupt to check
  * @retval value of the interrupt in the register
  */
__STATIC_INLINE uint32_t LL_CORDIC_GetITSource(const CORDIC_TypeDef *p_cordic, uint32_t interrupt)
{
  return ((LL_CORDIC_READ_REG((p_cordic), CSR) & interrupt));
}
/**
  * @}
  */

/** @defgroup CORDIC_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable CORDIC DMA read channel request.
  * @rmtoll
  *  CSR          DMAREN        LL_CORDIC_EnableDMAReq_RD
  * @param  p_cordic CORDIC Instance
  */
__STATIC_INLINE void LL_CORDIC_EnableDMAReq_RD(CORDIC_TypeDef *p_cordic)
{
  STM32_SET_BIT(p_cordic->CSR, CORDIC_CSR_DMAREN);
}

/**
  * @brief  Disable CORDIC DMA read channel request.
  * @rmtoll
  *  CSR          DMAREN        LL_CORDIC_DisableDMAReq_RD
  * @param  p_cordic CORDIC Instance
  */
__STATIC_INLINE void LL_CORDIC_DisableDMAReq_RD(CORDIC_TypeDef *p_cordic)
{
  STM32_CLEAR_BIT(p_cordic->CSR, CORDIC_CSR_DMAREN);
}

/**
  * @brief  Check the CORDIC DMA read channel request state.
  * @rmtoll
  *  CSR          DMAREN        LL_CORDIC_IsEnabledDMAReq_RD
  * @param  p_cordic CORDIC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CORDIC_IsEnabledDMAReq_RD(const CORDIC_TypeDef *p_cordic)
{
  return ((STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_DMAREN) == (CORDIC_CSR_DMAREN)) ? 1U : 0U);
}

/**
  * @brief  Enable CORDIC DMA write channel request.
  * @rmtoll
  *  CSR          DMAWEN        LL_CORDIC_EnableDMAReq_WR
  * @param  p_cordic CORDIC Instance
  */
__STATIC_INLINE void LL_CORDIC_EnableDMAReq_WR(CORDIC_TypeDef *p_cordic)
{
  STM32_SET_BIT(p_cordic->CSR, CORDIC_CSR_DMAWEN);
}

/**
  * @brief  Disable CORDIC DMA write channel request.
  * @rmtoll
  *  CSR          DMAWEN        LL_CORDIC_DisableDMAReq_WR
  * @param  p_cordic CORDIC Instance
  */
__STATIC_INLINE void LL_CORDIC_DisableDMAReq_WR(CORDIC_TypeDef *p_cordic)
{
  STM32_CLEAR_BIT(p_cordic->CSR, CORDIC_CSR_DMAWEN);
}

/**
  * @brief  Check the CORDIC DMA write channel request state.
  * @rmtoll
  *  CSR          DMAWEN        LL_CORDIC_IsEnabledDMAReq_WR
  * @param  p_cordic CORDIC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CORDIC_IsEnabledDMAReq_WR(const CORDIC_TypeDef *p_cordic)
{
  return ((STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_DMAWEN) == (CORDIC_CSR_DMAWEN)) ? 1U : 0U);
}

/**
  * @brief  Get the CORDIC data register address used for DMA transfer.
  * @rmtoll
  *  RDATA        RES           LL_CORDIC_DMA_GetRegAddr \n
  *  WDATA        ARG           LL_CORDIC_DMA_GetRegAddr
  * @param  p_cordic CORDIC Instance
  * @param  direction parameter can be one of the following values:
  *         @arg @ref LL_CORDIC_DMA_REG_DATA_IN
  *         @arg @ref LL_CORDIC_DMA_REG_DATA_OUT
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_CORDIC_DMA_GetRegAddr(const CORDIC_TypeDef *p_cordic, uint32_t direction)
{
  uint32_t data_reg_addr;

  if (direction == LL_CORDIC_DMA_REG_DATA_OUT)
  {
    /* return address of RDATA register */
    data_reg_addr = (uint32_t) &(p_cordic->RDATA);
  }
  else
  {
    /* return address of WDATA register */
    data_reg_addr = (uint32_t) &(p_cordic->WDATA);
  }

  return data_reg_addr;
}

/**
  * @}
  */


/** @defgroup CORDIC_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check CORDIC result ready flag state.
  * @rmtoll
  *  CSR          RRDY          LL_CORDIC_IsActiveFlag_RRDY
  * @param  p_cordic CORDIC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_CORDIC_IsActiveFlag_RRDY(const CORDIC_TypeDef *p_cordic)
{
  return ((STM32_READ_BIT(p_cordic->CSR, CORDIC_CSR_RRDY) == (CORDIC_CSR_RRDY)) ? 1U : 0U);
}
/**
  * @}
  */


/** @defgroup CORDIC_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write 32-bit input data for the CORDIC processing.
  * @rmtoll
  *  WDATA        ARG           LL_CORDIC_WriteData
  * @param  p_cordic CORDIC Instance
  * @param  input_data 0 .. 0xFFFFFFFF : 32-bit value to be provided as input data for CORDIC processing.
  */
__STATIC_INLINE void LL_CORDIC_WriteData(CORDIC_TypeDef *p_cordic, uint32_t input_data)
{
  STM32_WRITE_REG(p_cordic->WDATA, input_data);
}

/**
  * @brief  Return the 32-bit output data of CORDIC processing.
  * @rmtoll
  *  RDATA        RES           LL_CORDIC_ReadData
  * @param  p_cordic CORDIC Instance
  * @retval 32-bit output data of CORDIC processing.
  */
__STATIC_INLINE uint32_t LL_CORDIC_ReadData(const CORDIC_TypeDef *p_cordic)
{
  return (uint32_t)(STM32_READ_REG(p_cordic->RDATA));
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

#endif /* CORDIC */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_CORDIC_H */
