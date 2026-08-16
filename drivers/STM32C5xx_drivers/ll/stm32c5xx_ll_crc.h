/**
  ******************************************************************************
  * @file    stm32c5xx_ll_crc.h
  * @brief   Header file of CRC LL module.
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
#ifndef STM32C5XX_LL_CRC_H
#define STM32C5XX_LL_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(CRC)

/** @defgroup CRC_LL CRC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup CRC_LL_Exported_Constants LL CRC Constants
  * @{
  */

/** @defgroup CRC_LL_EC_POLYSIZE Polynomial size
  * @{
  */
#define LL_CRC_POLY_SIZE_32B              0x00000000U                              /*!< 32-bit polynomial size */
#define LL_CRC_POLY_SIZE_16B              CRC_CR_POLYSIZE_0                        /*!< 16-bit polynomial size */
#define LL_CRC_POLY_SIZE_8B               CRC_CR_POLYSIZE_1                        /*!< 8-bit polynomial size */
#define LL_CRC_POLY_SIZE_7B               (CRC_CR_POLYSIZE_1 | CRC_CR_POLYSIZE_0)  /*!< 7-bit polynomial size */
/**
  * @}
  */

/** @defgroup CRC_LL_EC_INDATA_REVERSE Input Data Reverse
  * @{
  */
#define LL_CRC_INDATA_REVERSE_NONE             0x00000000U                             /*!< Input Data bit order not
                                                                                       affected                     */
#define LL_CRC_INDATA_REVERSE_BYTE             CRC_CR_REV_IN_0                         /*!< Input Data bit reversal
                                                                                       done by byte                 */
#define LL_CRC_INDATA_REVERSE_HALFWORD         CRC_CR_REV_IN_1                         /*!< Input Data bit reversal
                                                                                       done by half-word            */
#define LL_CRC_INDATA_REVERSE_WORD             (CRC_CR_REV_IN_1 | CRC_CR_REV_IN_0)     /*!< Input Data bit reversal
                                                                                       done by word                 */
#define LL_CRC_INDATA_REVERSE_HALFWORD_BYWORD  (CRC_CR_RTYPE_IN | CRC_CR_REV_IN_0)     /*!< Input Data halfword reversal
                                                                                       done by word                 */
#define LL_CRC_INDATA_REVERSE_BYTE_BYWORD      (CRC_CR_RTYPE_IN | CRC_CR_REV_IN_1)     /*!< Input Data byte reversal
                                                                                       done by word                 */
/**
  * @}
  */

/** @defgroup CRC_LL_EC_OUTDATA_REVERSE Output Data Reverse
  * @{
  */
#define LL_CRC_OUTDATA_REVERSE_NONE            0x00000000U                           /*!< Output Data bit order
                                                                                          not affected              */
#define LL_CRC_OUTDATA_REVERSE_BIT             CRC_CR_REV_OUT_0                      /*!< Output Data bit reversal
                                                                                          done by bit               */
#define LL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD (CRC_CR_RTYPE_OUT | CRC_CR_REV_OUT_0) /*!< Output Data halfword
                                                                                          reversal done by word     */
#define LL_CRC_OUTDATA_REVERSE_BYTE_BYWORD     (CRC_CR_RTYPE_OUT | CRC_CR_REV_OUT_1) /*!< Output Data byte reversal
                                                                                          done by word              */
/**
  * @}
  */

/** @defgroup CRC_LL_EC_Default_Polynomial_Value    Default CRC generating polynomial value
  * @brief    Normal representation of this polynomial value is
  *           X^32 + X^26 + X^23 + X^22 + X^16 + X^12 + X^11 + X^10 +X^8 + X^7 + X^5 + X^4 + X^2 + X + 1 .
  * @{
  */
#define LL_CRC_DEFAULT_CRC32_POLY          0x04C11DB7U /*!< Default CRC generating polynomial value */
/**
  * @}
  */

/** @defgroup CRC_LL_EC_Default_InitValue    Default CRC computation initialization value
  * @{
  */
#define LL_CRC_DEFAULT_CRC_INITVALUE       0xFFFFFFFFU /*!< Default CRC computation initialization value */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRC_LL_Exported_Macros LL CRC Macros
  * @{
  */

/** @defgroup CRC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in CRC register.
  * @param  instance CRC Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_CRC_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in CRC register.
  * @param  instance CRC Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_CRC_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup CRC_LL_Exported_Functions LL CRC Functions
  * @{
  */

/** @defgroup CRC_LL_EF_Configuration CRC Configuration functions
  * @{
  */

/**
  * @brief  Reset the CRC calculation unit.
  * @param  crcx CRC Instance
  * @note   If Programmable Initial CRC value feature
  *         is available, also set the Data Register to the value stored in the
  *         CRC_INIT register, otherwise, reset Data Register to its default value.
  * @rmtoll
  *  CR           RESET         LL_CRC_ResetCRCCalculationUnit
  */
__STATIC_INLINE void LL_CRC_ResetCRCCalculationUnit(CRC_TypeDef *crcx)
{
  STM32_SET_BIT(crcx->CR, CRC_CR_RESET);
}

/**
  * @brief  Configure size of the polynomial.
  * @rmtoll
  *  CR           POLYSIZE      LL_CRC_SetPolynomialSize
  * @param  crcx CRC Instance
  * @param  poly_size This parameter can be one of the following values:
  *         @arg @ref LL_CRC_POLY_SIZE_32B
  *         @arg @ref LL_CRC_POLY_SIZE_16B
  *         @arg @ref LL_CRC_POLY_SIZE_8B
  *         @arg @ref LL_CRC_POLY_SIZE_7B
  */
__STATIC_INLINE void LL_CRC_SetPolynomialSize(CRC_TypeDef *crcx, uint32_t poly_size)
{
  STM32_MODIFY_REG(crcx->CR, CRC_CR_POLYSIZE, poly_size);
}

/**
  * @brief  Return size of the polynomial.
  * @rmtoll
  *  CR           POLYSIZE      LL_CRC_GetPolynomialSize
  * @param  crcx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRC_POLY_SIZE_32B
  *         @arg @ref LL_CRC_POLY_SIZE_16B
  *         @arg @ref LL_CRC_POLY_SIZE_8B
  *         @arg @ref LL_CRC_POLY_SIZE_7B
  */
__STATIC_INLINE uint32_t LL_CRC_GetPolynomialSize(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_BIT(crcx->CR, CRC_CR_POLYSIZE));
}

/**
  * @brief  Configure the reversal of the bit order of the input and Output data.
  * @rmtoll
  *  CR           REV_X     LL_CRC_SetDataReverseMode\n
  *  CR           RTYPE_X   LL_CRC_SetDataReverseMode
  * @param  crcx CRC Instance
  * @param  input_reverse_mode This parameter can be one of the following values:
  *         @arg @ref LL_CRC_INDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_WORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE_BYWORD
  * @param  output_reverse_mode This parameter can be one of the following values:
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BIT
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BYTE_BYWORD
  * @note   REV_X bit value of REV_IN and REV_OUT
  */
__STATIC_INLINE void LL_CRC_SetDataReverseMode(CRC_TypeDef *crcx, uint32_t input_reverse_mode,
                                               uint32_t output_reverse_mode)
{
  STM32_MODIFY_REG(crcx->CR,
                   CRC_CR_REV_IN | CRC_CR_RTYPE_IN | CRC_CR_REV_OUT | CRC_CR_RTYPE_OUT,
                   input_reverse_mode | output_reverse_mode);
}

/**
  * @brief  Configure the reversal of the bit order of the input data.
  * @rmtoll
  *  CR           REV_IN        LL_CRC_SetInputDataReverseMode\n
  *  CR           RTYPE_IN      LL_CRC_SetInputDataReverseMode
  * @param  crcx CRC Instance
  * @param  input_reverse_mode This parameter can be one of the following values:
  *         @arg @ref LL_CRC_INDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_WORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE_BYWORD
  */
__STATIC_INLINE void LL_CRC_SetInputDataReverseMode(CRC_TypeDef *crcx, uint32_t input_reverse_mode)
{
  STM32_MODIFY_REG(crcx->CR, CRC_CR_RTYPE_IN | CRC_CR_REV_IN, input_reverse_mode);
}

/**
  * @brief  Return type of reversal for input data bit order.
  * @rmtoll
  *  CR           REV_IN        LL_CRC_GetInputDataReverseMode\n
  *  CR           RTYPE_IN      LL_CRC_GetInputDataReverseMode
  * @param  crcx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRC_INDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_WORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_INDATA_REVERSE_BYTE_BYWORD
  */
__STATIC_INLINE uint32_t LL_CRC_GetInputDataReverseMode(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_BIT(crcx->CR, CRC_CR_RTYPE_IN | CRC_CR_REV_IN));
}

/**
  * @brief  Configure the reversal of the bit order of the Output data.
  * @rmtoll
  *  CR           REV_OUT       LL_CRC_SetOutputDataReverseMode\n
  *  CR           RTYPE_OUT     LL_CRC_SetOutputDataReverseMode
  * @param  crcx CRC Instance
  * @param  output_reverse_mode This parameter can be one of the following values:
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BIT
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BYTE_BYWORD
  */
__STATIC_INLINE void LL_CRC_SetOutputDataReverseMode(CRC_TypeDef *crcx, uint32_t output_reverse_mode)
{
  STM32_MODIFY_REG(crcx->CR, CRC_CR_RTYPE_OUT | CRC_CR_REV_OUT, output_reverse_mode);
}
/**
  * @brief  Return mode of reversal of the bit order of the Output data.
  * @rmtoll
  *  CR           REV_OUT       LL_CRC_GetOutputDataReverseMode
  * @param  crcx CRC Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_NONE
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BIT
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_HALFWORD_BYWORD
  *         @arg @ref LL_CRC_OUTDATA_REVERSE_BYTE_BYWORD
  */
__STATIC_INLINE uint32_t LL_CRC_GetOutputDataReverseMode(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_BIT(crcx->CR, CRC_CR_RTYPE_OUT | CRC_CR_REV_OUT));
}

/**
  * @brief  Initialize the Programmable initial CRC value.
  * @param  crcx CRC Instance
  * @param  crc_init_value Value to be programmed in Programmable initial CRC value register
  * @note   If the CRC size is less than 32 bits, the least significant bits
  *         are used to write the correct value
  * @note   LL_CRC_DEFAULT_CRC_INITVALUE could be used as value for InitCrc parameter.
  * @rmtoll
  *  INIT         INIT          LL_CRC_SetInitialData
  */
__STATIC_INLINE void LL_CRC_SetInitialData(CRC_TypeDef *crcx, uint32_t crc_init_value)
{
  STM32_WRITE_REG(crcx->INIT, crc_init_value);
}

/**
  * @brief  Return the current initial CRC value.
  * @param  crcx CRC Instance
  * @note   If the CRC size is less than 32 bits, the least significant bits
  *         are used to read the correct value
  * @rmtoll
  *  INIT         INIT          LL_CRC_GetInitialData
  * @retval Value programmed in Programmable initial CRC value register
  */
__STATIC_INLINE uint32_t LL_CRC_GetInitialData(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_REG(crcx->INIT));
}

/**
  * @brief  Initialize the Programmable polynomial value
  *         (coefficients of the polynomial to be used for CRC calculation).
  * @param  crcx CRC Instance
  * @param  polynomial_coefficient Value to be programmed in Programmable Polynomial value register
  * @note   LL_CRC_DEFAULT_CRC32_POLY could be used as value for PolynomCoef parameter.
  * @note   Please check Reference Manual and existing Errata Sheets,
  *         regarding possible limitations for Polynomial values usage.
  *         For example, for a polynomial of degree 7, X^7 + X^6 + X^5 + X^2 + 1 is written 0x65
  * @rmtoll
  *  POL          POL           LL_CRC_SetPolynomialCoef
  */
__STATIC_INLINE void LL_CRC_SetPolynomialCoef(CRC_TypeDef *crcx, uint32_t polynomial_coefficient)
{
  STM32_WRITE_REG(crcx->POL, polynomial_coefficient);
}

/**
  * @brief  Return current Programmable polynomial value.
  * @param  crcx CRC Instance
  * @note   Please check Reference Manual and existing Errata Sheets,
  *         regarding possible limitations for Polynomial values usage.
  *         For example, for a polynomial of degree 7, X^7 + X^6 + X^5 + X^2 + 1 is written 0x65
  * @rmtoll
  *  POL          POL           LL_CRC_GetPolynomialCoef
  * @retval Value programmed in Programmable Polynomial value register
  */
__STATIC_INLINE uint32_t LL_CRC_GetPolynomialCoef(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_REG(crcx->POL));
}

/**
  * @}
  */

/** @defgroup CRC_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write given 32-bit data to the CRC calculator.
  * @rmtoll
  *  DR           DR            LL_CRC_FeedData32
  * @param  crcx CRC Instance
  * @param  in_data value to be provided to CRC calculator between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE void LL_CRC_FeedData32(CRC_TypeDef *crcx, uint32_t in_data)
{
  STM32_WRITE_REG(crcx->DR, in_data);
}

/**
  * @brief  Write given 16-bit data to the CRC calculator.
  * @rmtoll
  *  DR           DR            LL_CRC_FeedData16
  * @param  crcx CRC Instance
  * @param  in_data 16 bit value to be provided to CRC calculator between Min_Data=0 and Max_Data=0xFFFF
  */
__STATIC_INLINE void LL_CRC_FeedData16(CRC_TypeDef *crcx, uint16_t in_data)
{
  __IO uint16_t *p_reg;

  /* Need 16 bits bus write access so in_data is interpreted as 16 bits write to the DR register*/
  p_reg = (__IO uint16_t *)(__IO void *)(&crcx->DR);                             /* Derogation MisraC2012 R.11.5 */
  *p_reg = in_data;
}

/**
  * @brief  Write given 8-bit data to the CRC calculator.
  * @rmtoll
  *  DR           DR            LL_CRC_FeedData8
  * @param  crcx CRC Instance
  * @param  in_data 8 bit value to be provided to CRC calculator between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_CRC_FeedData8(CRC_TypeDef *crcx, uint8_t in_data)
{
  *(uint8_t __IO *)(&crcx->DR) = (uint8_t) in_data;
}

/**
  * @brief  Return current CRC calculation result. 32 bits value is returned.
  * @rmtoll
  *  DR           DR            LL_CRC_ReadData32
  * @param  crcx CRC Instance
  * @retval Current CRC calculation result as stored in CRC_DR register (32 bits).
  */
__STATIC_INLINE uint32_t LL_CRC_ReadData32(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_REG(crcx->DR));
}

/**
  * @brief  Return current CRC calculation result. 16 bits value is returned.
  * @param  crcx CRC Instance
  * @note   This function is expected to be used in a 16 bits CRC polynomial size context.
  * @rmtoll
  *  DR           DR            LL_CRC_ReadData16
  * @retval Current CRC calculation result as stored in CRC_DR register (16 bits).
  */
__STATIC_INLINE uint16_t LL_CRC_ReadData16(const CRC_TypeDef *crcx)
{
  return (uint16_t)STM32_READ_REG(crcx->DR);
}

/**
  * @brief  Return current CRC calculation result. 8 bits value is returned.
  * @param  crcx CRC Instance
  * @note   This function is expected to be used in a 8 bits CRC polynomial size context.
  * @rmtoll
  *  DR           DR            LL_CRC_ReadData8
  * @retval Current CRC calculation result as stored in CRC_DR register (8 bits).
  */
__STATIC_INLINE uint8_t LL_CRC_ReadData8(const CRC_TypeDef *crcx)
{
  return (uint8_t)STM32_READ_REG(crcx->DR);
}

/**
  * @brief  Return current CRC calculation result. 7 bits value is returned.
  * @param  crcx CRC Instance
  * @note   This function is expected to be used in a 7 bits CRC polynomial size context.
  * @rmtoll
  *  DR           DR            LL_CRC_ReadData7
  * @retval Current CRC calculation result as stored in CRC_DR register (7 bits).
  */
__STATIC_INLINE uint8_t LL_CRC_ReadData7(const CRC_TypeDef *crcx)
{
  return (uint8_t)(STM32_READ_REG(crcx->DR) & 0x7FU);
}

/**
  * @brief  Return data stored in the Independent Data(IDR) register.
  * @param  crcx CRC Instance
  * @note   This register can be used as a temporary storage location for one 32-bit long data.
  * @rmtoll
  *  IDR          IDR           LL_CRC_ReadIDR
  * @retval Value stored in CRC_IDR register (General-purpose 32-bit data register).
  */
__STATIC_INLINE uint32_t LL_CRC_ReadIDR(const CRC_TypeDef *crcx)
{
  return (uint32_t)(STM32_READ_REG(crcx->IDR));
}

/**
  * @brief  Store data in the Independent Data(IDR) register.
  * @param  crcx CRC Instance
  * @param  in_data value to be stored in CRC_IDR register (32-bit) between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @note   This register can be used as a temporary storage location for one 32-bit long data.
  * @rmtoll
  *  IDR          IDR           LL_CRC_WriteIDR
  */
__STATIC_INLINE void LL_CRC_WriteIDR(CRC_TypeDef *crcx, uint32_t in_data)
{
  STM32_WRITE_REG(crcx->IDR, in_data);
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

#endif /* defined(CRC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_CRC_H */
