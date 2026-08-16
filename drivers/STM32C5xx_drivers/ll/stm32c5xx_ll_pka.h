/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_pka.h
  * @brief   Header file of PKA LL module.
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
 @verbatim
  ======================================================================================================================
                      --------- LL PKA driver acronyms ---------
  ======================================================================================================================
  [..]  Acronyms table:
                   ========================================================
                   | Acronym |                                            |
                   ========================================================
                   | RSA     | Rivest Shamir Adleman                      |
                   | ECDSA   | Elliptic Curve Digital Signature Algorithm |
                   | ECC     | Elliptic curve cryptography                |
                   | CRT     | Chinese Remainder Theorem                  |
                   | Mod     | Modular                                    |
                   | Exp     | Exponentiation                             |
                   | Mul     | Multiplication                             |
                   | Add     | Addition                                   |
                   | Sub     | Subtraction                                |
                   | Cmp     | Comparison                                 |
                   | Inv     | Inversion                                  |
                   | Red     | Reduction                                  |
                   | Sign    | Signature                                  |
                   | Verif   | Verification                               |
                   ========================================================
 @endverbatim
  **********************************************************************************************************************
  */
/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_LL_PKA_H
#define STM32C5XX_LL_PKA_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined(PKA)
/** @defgroup PKA_LL PKA
  * @{
  */
/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/* Private macros ----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Exported types ----------------------------------------------------------------------------------------------------*/
/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_LL_Exported_Constants LL PKA Constants
  * @{
  */
/** @defgroup PKA_LL_EC_GET_FLAG Get flag definitions
  * @brief    Flag definitions that can be used with the LL_PKA_ReadReg function.
  * @{
  */
#define LL_PKA_FLAG_ADDRERR   PKA_SR_ADDRERRF                         /*!< Address error flag                       */
#define LL_PKA_FLAG_RAMERR    PKA_SR_RAMERRF                          /*!< RAM error flag                           */
#define LL_PKA_FLAG_PROCEND   PKA_SR_PROCENDF                         /*!< End of process flag                      */
#define LL_PKA_FLAG_BUSY      PKA_SR_BUSY                             /*!< Busy flag                                */
#define LL_PKA_FLAG_INITOK    PKA_SR_INITOK                           /*!< Init OK flag                             */
#define LL_PKA_FLAG_OPERR     PKA_SR_OPERRF                           /*!< Operation error flag                     */
#if defined(PKA_SR_LMF)
#define LL_PKA_FLAG_LMF       PKA_SR_LMF                              /*!< Limited mode flag                        */
#endif /* PKA_SR_LMF */
#define LL_PKA_FLAG_CMF       PKA_SR_CMF                              /*!< Chaining mode flags                      */
#define LL_PKA_FLAG_RNGERRF   PKA_SR_RNGERRF                          /*!< RNG error flag                           */
#define LL_PKA_FLAG_MDERRF    PKA_SR_MDERRF                           /*!< Mode error flag error flag               */
#define LL_PKA_FLAG_TRZERRF   PKA_SR_TRZERRF                          /*!< Trailing 0s error flag                   */
#define LL_PKA_FLAG_DATAZF    PKA_SR_DATAZF                           /*!< Data 0-ed error flag                     */
#define LL_PKA_FLAG_INCRERRF  PKA_SR_INCRERRF                         /*!< Increment error flag                     */
#define LL_PKA_FLAG_DATAOKF   PKA_SR_DATAOKF                          /*!< Data OK flag                             */
#define LL_PKA_FLAG_RNGOKF    PKA_SR_RNGOKF                           /*!< RNG OK flag                              */
#define LL_PKA_FLAG_CCEN      PKA_SR_CCEN                             /*!< Coupling and chaining mode enable        */
#define LL_PKA_FLAG_ALL       (PKA_SR_CMF | PKA_SR_ADDRERRF \
                               | PKA_SR_CCEN | PKA_SR_BUSY \
                               | PKA_SR_OPERRF | PKA_SR_RNGERRF \
                               | PKA_SR_MDERRF | PKA_SR_TRZERRF \
                               | PKA_SR_DATAZF | PKA_SR_INCRERRF \
                               | PKA_SR_DATAOKF | PKA_SR_RNGOKF \
                               | PKA_SR_RAMERRF | PKA_SR_PROCENDF)    /*!< All flags                                */
#define LL_PKA_FLAG_ERROR_ALL (PKA_SR_CMF | PKA_SR_ADDRERRF  \
                               | PKA_SR_RAMERRF | PKA_SR_OPERRF \
                               | PKA_SR_RNGERRF | PKA_SR_MDERRF \
                               | PKA_SR_TRZERRF | PKA_SR_INCRERRF)    /*!< All error flags                          */
/**
  * @}
  */
/** @defgroup PKA_LL_EC_IT IT definitions
  * @brief    IT definitions that can be used with the LL_PKA_ReadReg and LL_PKA_WriteReg functions.
  * @{
  */
#define LL_PKA_IT_ADDRERR PKA_CR_ADDRERRIE                            /*!< Address error interrupt                  */
#define LL_PKA_IT_RAMERR  PKA_CR_RAMERRIE                             /*!< RAM error interrupt                      */
#define LL_PKA_IT_PROCEND PKA_CR_PROCENDIE                            /*!< End of process interrupt                 */
#define LL_PKA_IT_OPERR   PKA_CR_OPERRIE                              /*!< Operation error interrupt                */
#define LL_PKA_IT_CMFIE   PKA_CR_CMFIE                                /*!< Chaining mode flags interrupt            */
#define LL_PKA_IT_ALL (PKA_CR_CMFIE | PKA_CR_ADDRERRIE \
                       | PKA_CR_RAMERRIE | PKA_CR_PROCENDIE \
                       | PKA_CR_OPERRIE)                              /*!< All interrupts                           */
/**
  * @}
  */
/** @defgroup PKA_LL_EC_MODE Operation Mode
  * @brief    List of operation modes.
  * @{
  */
#define LL_PKA_MODE_MODULAR_EXP              PKA_MODE_MODULAR_EXP             /*!< Modular exponentiation         */
#define LL_PKA_MODE_MONTGOMERY_PARAM         PKA_MODE_MONTGOMERY_PARAM        /*!< Compute Montgomery parameter only */
#define LL_PKA_MODE_MODULAR_EXP_FAST         PKA_MODE_MODULAR_EXP_FAST        /*!< Modular exponentiation fast mode */
#define LL_PKA_MODE_MODULAR_EXP_PROTECT      PKA_MODE_MODULAR_EXP_PROTECT     /*!< Modular exponentiation protected mode */
#define LL_PKA_MODE_ECC_MUL_PROTECT          PKA_MODE_ECC_MUL_PROTECT         /*!< ECC scalar multiplication protected mode */
#define LL_PKA_MODE_ECC_COMPLETE_ADD         PKA_MODE_ECC_COMPLETE_ADD        /*!< ECC complete addition          */
#define LL_PKA_MODE_ECDSA_SIGNATURE_PROTECT  PKA_MODE_ECDSA_SIGNATURE_PROTECT /*!< ECDSA signing protected mode   */
#define LL_PKA_MODE_ECDSA_VERIFICATION       PKA_MODE_ECDSA_VERIFICATION      /*!< ECDSA verification             */
#define LL_PKA_MODE_POINT_CHECK              PKA_MODE_POINT_CHECK             /*!< Point check                    */
#define LL_PKA_MODE_RSA_CRT_EXP              PKA_MODE_RSA_CRT_EXP             /*!< RSA CRT exponentiation         */
#define LL_PKA_MODE_MODULAR_INV              PKA_MODE_MODULAR_INV             /*!< Modular inversion              */
#define LL_PKA_MODE_ARITHMETIC_ADD           PKA_MODE_ARITHMETIC_ADD          /*!< Arithmetic addition            */
#define LL_PKA_MODE_ARITHMETIC_SUB           PKA_MODE_ARITHMETIC_SUB          /*!< Arithmetic subtraction         */
#define LL_PKA_MODE_ARITHMETIC_MUL           PKA_MODE_ARITHMETIC_MUL          /*!< Arithmetic multiplication      */
#define LL_PKA_MODE_COMPARISON               PKA_MODE_COMPARISON              /*!< Comparison                     */
#define LL_PKA_MODE_MODULAR_REDUC            PKA_MODE_MODULAR_REDUC           /*!< Modular reduction              */
#define LL_PKA_MODE_MODULAR_ADD              PKA_MODE_MODULAR_ADD             /*!< Modular addition               */
#define LL_PKA_MODE_MODULAR_SUB              PKA_MODE_MODULAR_SUB             /*!< Modular subtraction            */
#define LL_PKA_MODE_MONTGOMERY_MUL           PKA_MODE_MONTGOMERY_MUL          /*!< Montgomery multiplication      */
#define LL_PKA_MODE_DOUBLE_BASE_LADDER       PKA_MODE_DOUBLE_BASE_LADDER      /*!< Double base ladder             */
#define LL_PKA_MODE_ECC_PROJECTIVE_AFF       PKA_MODE_ECC_PROJECTIVE_AFF      /*!< ECC projective to affine       */
#define LL_PKA_MODE_RSA_SIGNATURE            PKA_MODE_RSA_SIGNATURE           /*!< RSA signing                    */
#define LL_PKA_MODE_RSA_SIGNATURE_FAST       PKA_MODE_RSA_SIGNATURE_FAST      /*!< RSA signing fast mode          */
#define LL_PKA_MODE_RSA_SIGNATURE_PROTECT    PKA_MODE_RSA_SIGNATURE_PROTECT   /*!< RSA signing protected mode     */
#define LL_PKA_MODE_RSA_VERIFICATION         PKA_MODE_RSA_VERIFICATION        /*!< RSA verification               */
/**
  * @}
  */
/**
  * @}
  */
/* Exported macro ----------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_LL_Exported_Macros LL PKA Macros
  * @{
  */
/** @defgroup PKA_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/**
  * @brief  Write a value in PKA register.
  * @param  instance PKA Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_PKA_WRITE_REG(instance, reg, value) STM32_WRITE_REG(((instance)->reg), (value))
/**
  * @brief  Read a value in PKA register.
  * @param  instance PKA Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_PKA_READ_REG(instance, reg) STM32_READ_REG(((instance)->reg))
/**
  * @}
  */
/**
  * @}
  */
/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup PKA_LL_Exported_Functions LL PKA Functions
  * @{
  */
/** @defgroup PKA_LL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief  Enable PKA peripheral.
  * @rmtoll
  *  CR           EN            LL_PKA_Enable
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_Enable(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_EN);
}
/**
  * @brief  Disable PKA peripheral.
  * @rmtoll
  *  CR           EN            LL_PKA_Disable
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_Disable(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_EN);
}
/**
  * @brief  Check if the PKA peripheral is enabled or disabled.
  * @rmtoll
  *  CR           EN            LL_PKA_IsEnabled
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabled(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_EN) == (PKA_CR_EN)) ? 1UL : 0UL);
}
/**
  * @brief  Set PKA operating mode.
  * @rmtoll
  *  CR           MODE          LL_PKA_SetMode
  * @param  pkax PKA Instance.
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP
  *         @arg @ref LL_PKA_MODE_MONTGOMERY_PARAM
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP_FAST
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP_PROTECT
  *         @arg @ref LL_PKA_MODE_ECC_MUL_PROTECT
  *         @arg @ref LL_PKA_MODE_ECC_COMPLETE_ADD
  *         @arg @ref LL_PKA_MODE_ECDSA_SIGNATURE_PROTECT
  *         @arg @ref LL_PKA_MODE_ECDSA_VERIFICATION
  *         @arg @ref LL_PKA_MODE_POINT_CHECK
  *         @arg @ref LL_PKA_MODE_RSA_CRT_EXP
  *         @arg @ref LL_PKA_MODE_MODULAR_INV
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_ADD
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_SUB
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_MUL
  *         @arg @ref LL_PKA_MODE_COMPARISON
  *         @arg @ref LL_PKA_MODE_MODULAR_REDUC
  *         @arg @ref LL_PKA_MODE_MODULAR_ADD
  *         @arg @ref LL_PKA_MODE_MODULAR_SUB
  *         @arg @ref LL_PKA_MODE_MONTGOMERY_MUL
  *         @arg @ref LL_PKA_MODE_DOUBLE_BASE_LADDER
  *         @arg @ref LL_PKA_MODE_ECC_PROJECTIVE_AFF
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE_FAST
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE_PROTECT
  *         @arg @ref LL_PKA_MODE_RSA_VERIFICATION
  */
__STATIC_INLINE void LL_PKA_SetMode(PKA_TypeDef *pkax, uint32_t mode)
{
  STM32_MODIFY_REG(pkax->CR, (PKA_CR_MODE), (mode));
}
/**
  * @brief  Get PKA operating mode.
  * @rmtoll
  *  CR           MODE          LL_PKA_GetMode
  * @param  pkax PKA Instance.
  * @retval returned value can be one of the following values:
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP
  *         @arg @ref LL_PKA_MODE_MONTGOMERY_PARAM
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP_FAST
  *         @arg @ref LL_PKA_MODE_MODULAR_EXP_PROTECT
  *         @arg @ref LL_PKA_MODE_ECC_MUL_PROTECT
  *         @arg @ref LL_PKA_MODE_ECC_COMPLETE_ADD
  *         @arg @ref LL_PKA_MODE_ECDSA_SIGNATURE_PROTECT
  *         @arg @ref LL_PKA_MODE_ECDSA_VERIFICATION
  *         @arg @ref LL_PKA_MODE_POINT_CHECK
  *         @arg @ref LL_PKA_MODE_RSA_CRT_EXP
  *         @arg @ref LL_PKA_MODE_MODULAR_INV
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_ADD
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_SUB
  *         @arg @ref LL_PKA_MODE_ARITHMETIC_MUL
  *         @arg @ref LL_PKA_MODE_COMPARISON
  *         @arg @ref LL_PKA_MODE_MODULAR_REDUC
  *         @arg @ref LL_PKA_MODE_MODULAR_ADD
  *         @arg @ref LL_PKA_MODE_MODULAR_SUB
  *         @arg @ref LL_PKA_MODE_MONTGOMERY_MUL
  *         @arg @ref LL_PKA_MODE_DOUBLE_BASE_LADDER
  *         @arg @ref LL_PKA_MODE_ECC_PROJECTIVE_AFF
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE_FAST
  *         @arg @ref LL_PKA_MODE_RSA_SIGNATURE_PROTECT
  *         @arg @ref LL_PKA_MODE_RSA_VERIFICATION
  */
__STATIC_INLINE uint32_t LL_PKA_GetMode(const PKA_TypeDef *pkax)
{
  return (uint32_t)STM32_READ_BIT(pkax->CR, PKA_CR_MODE);
}
/**
  * @brief  Start the operation selected using LL_PKA_SetMode.
  * @rmtoll
  *  CR           START         LL_PKA_Start
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_Start(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_START);
}
/**
  * @}
  */
/** @defgroup PKA_LL_EF_IT_Management IT_Management
  * @{
  */
/**
  * @brief  Enable address error interrupt.
  * @rmtoll
  *  CR           ADDRERRIE     LL_PKA_EnableIT_ADDRERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_EnableIT_ADDRERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_ADDRERRIE);
}
/**
  * @brief  Enable RAM error interrupt.
  * @rmtoll
  *  CR           RAMERRIE      LL_PKA_EnableIT_RAMERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_EnableIT_RAMERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_RAMERRIE);
}
/**
  * @brief  Enable OPERATION error interrupt.
  * @rmtoll
  *  CR           OPERRIE      LL_PKA_EnableIT_OPERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_EnableIT_OPERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_OPERRIE);
}
/**
  * @brief  Enable end of operation interrupt.
  * @rmtoll
  *  CR           PROCENDIE     LL_PKA_EnableIT_PROCEND
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_EnableIT_PROCEND(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_PROCENDIE);
}
/**
  * @brief  Enable chaining mode flags interrupt.
  * @rmtoll
  *  CR           CMFIE    LL_PKA_EnableIT_CMFIE
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_EnableIT_CMFIE(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CR, PKA_CR_CMFIE);
}
/** @brief  Enable the specified PKA interrupt.
  * @param  pkax specifies the PKA instance.
  * @param  it_source specifies the interrupt source to enable.
  *         This parameter can be any combination of the following values:
  *            @arg @ref LL_PKA_IT_PROCEND End Of Operation interrupt enable
  *            @arg @ref LL_PKA_IT_ADDRERR Address error interrupt enable
  *            @arg @ref LL_PKA_IT_RAMERR RAM error interrupt enable
  *            @arg @ref LL_PKA_IT_OPERR Operation error interrupt enable
  *            @arg @ref LL_PKA_IT_CMFIE Chaining mode flags interrupt enable
  */
__STATIC_INLINE void LL_PKA_EnableIT(PKA_TypeDef *pkax, uint32_t it_source)
{
  STM32_SET_BIT(pkax->CR, it_source);
}

/**
  * @brief  Disable address error interrupt.
  * @rmtoll
  *  CR           ADDRERRIE     LL_PKA_DisableIT_ADDERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_DisableIT_ADDERR(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_ADDRERRIE);
}

/**
  * @brief  Disable RAM error interrupt.
  * @rmtoll
  *  CR           RAMERRIE      LL_PKA_DisableIT_RAMERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_DisableIT_RAMERR(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_RAMERRIE);
}
/**
  * @brief  Disable End of operation interrupt.
  * @rmtoll
  *  CR           PROCENDIE     LL_PKA_DisableIT_PROCEND
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_DisableIT_PROCEND(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_PROCENDIE);
}
/**
  * @brief  Disable OPERATION error interrupt.
  * @rmtoll
  *  CR           OPERRIE      LL_PKA_DisableIT_OPERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_DisableIT_OPERR(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_OPERRIE);
}
/**
  * @brief  Disable chaining mode flags interrupt.
  * @rmtoll
  *  CR           CMFIE   LL_PKA_DisableIT_CMFIE
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_DisableIT_CMFIE(PKA_TypeDef *pkax)
{
  STM32_CLEAR_BIT(pkax->CR, PKA_CR_CMFIE);
}
/** @brief  Disable the specified PKA interrupt.
  * @param  pkax specifies the PKA instance.
  * @param  it_source specifies the interrupt source to disable.
  *         This parameter can be any combination of the following values:
  *            @arg @ref LL_PKA_IT_PROCEND End Of Operation interrupt enable
  *            @arg @ref LL_PKA_IT_ADDRERR Address error interrupt enable
  *            @arg @ref LL_PKA_IT_RAMERR RAM error interrupt enable
  *            @arg @ref LL_PKA_IT_OPERR Operation error interrupt enable
  *            @arg @ref LL_PKA_IT_CMFIE Chaining mode flags interrupt enable
  */
__STATIC_INLINE void LL_PKA_DisableIT(PKA_TypeDef *pkax, uint32_t it_source)
{
  STM32_CLEAR_BIT(pkax->CR, it_source);
}
/**
  * @brief  Check if address error interrupt is enabled.
  * @rmtoll
  *  CR           ADDRERRIE     LL_PKA_IsEnabledIT_ADDRERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabledIT_ADDRERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_ADDRERRIE) == (PKA_CR_ADDRERRIE)) ? 1UL : 0UL);
}
/**
  * @brief  Check if RAM error interrupt is enabled.
  * @rmtoll
  *  CR           RAMERRIE      LL_PKA_IsEnabledIT_RAMERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabledIT_RAMERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_RAMERRIE) == (PKA_CR_RAMERRIE)) ? 1UL : 0UL);
}
/**
  * @brief  Check if OPERATION error interrupt is enabled.
  * @rmtoll
  *  CR           OPERRIE      LL_PKA_IsEnabledIT_OPERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabledIT_OPERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_OPERRIE) == (PKA_CR_OPERRIE)) ? 1UL : 0UL);
}
/**
  * @brief  Check if end of operation interrupt is enabled.
  * @rmtoll
  *  CR           PROCENDIE     LL_PKA_IsEnabledIT_PROCEND
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabledIT_PROCEND(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_PROCENDIE) == (PKA_CR_PROCENDIE)) ? 1UL : 0UL);
}
/**
  * @brief  Check if chaining mode flags interrupt is enabled.
  * @rmtoll
  *  CR           CMFIE     LL_PKA_IsEnabledIT_CMFIE
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsEnabledIT_CMFIE(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->CR, PKA_CR_CMFIE) == (PKA_CR_CMFIE)) ? 1UL : 0UL);
}
/**
  * @}
  */
/** @defgroup PKA_LL_EF_FLAG_Management PKA flag management
  * @{
  */
/**
  * @brief  Get PKA address error flag.
  * @rmtoll
  *  SR           ADDRERRF      LL_PKA_IsActiveFlag_ADDRERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_ADDRERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_ADDRERRF) == (PKA_SR_ADDRERRF)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA RAM error flag.
  * @rmtoll
  *  SR           RAMERRF      LL_PKA_IsActiveFlag_RAMERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_RAMERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_RAMERRF) == (PKA_SR_RAMERRF)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA OPERATION error flag.
  * @rmtoll
  *  SR           OPERRF      LL_PKA_IsActiveFlag_OPERR
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_OPERR(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_OPERRF) == (PKA_SR_OPERRF)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA end of operation flag.
  * @rmtoll
  *  SR           PROCENDF      LL_PKA_IsActiveFlag_PROCEND
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_PROCEND(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_PROCENDF) == (PKA_SR_PROCENDF)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA busy flag.
  * @rmtoll
  *  SR           BUSY          LL_PKA_IsActiveFlag_BUSY
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_BUSY(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_BUSY) == (PKA_SR_BUSY)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA INITOK flag.
  * @rmtoll
  *  SR           INITOK      LL_PKA_IsActiveFlag_INITOK
  * @param  pkax PKA Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag_INITOK(const PKA_TypeDef *pkax)
{
  return ((STM32_READ_BIT(pkax->SR, PKA_SR_INITOK) == (PKA_SR_INITOK)) ? 1UL : 0UL);
}
/**
  * @brief  Get PKA flag.
  * @rmtoll
  *  SR           RAMERRF       LL_PKA_IsActiveFlag_RAMERR
  * @param  pkax PKA Instance.
  * @param  flag This parameter can be one of the following values:
  *              @arg @ref LL_PKA_FLAG_ADDRERR
  *              @arg @ref LL_PKA_FLAG_RAMERR
  *              @arg @ref LL_PKA_FLAG_PROCEND
  *              @arg @ref LL_PKA_FLAG_BUSY
  *              @arg @ref LL_PKA_FLAG_INITOK
  *              @arg @ref LL_PKA_FLAG_OPERR
  *              @arg @ref LL_PKA_FLAG_CMF
  *              @arg @ref LL_PKA_FLAG_RNGERRF
  *              @arg @ref LL_PKA_FLAG_MDERRF
  *              @arg @ref LL_PKA_FLAG_CMF
  *              @arg @ref LL_PKA_FLAG_TRZERRF
  *              @arg @ref LL_PKA_FLAG_DATAZF
  *              @arg @ref LL_PKA_FLAG_INCRERRF
  *              @arg @ref LL_PKA_FLAG_DATAOKF
  *              @arg @ref LL_PKA_FLAG_RNGOKF
  *              @arg @ref LL_PKA_FLAG_CCEN
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PKA_IsActiveFlag(const PKA_TypeDef *pkax, uint32_t flag)
{
  return ((STM32_READ_BIT(pkax->SR, flag) == (flag)) ? 1UL : 0UL);
}
/**
  * @brief  Clear PKA address error flag.
  * @rmtoll
  *  CLRFR        ADDRERRFC     LL_PKA_ClearFlag_ADDERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_ClearFlag_ADDERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CLRFR, PKA_CLRFR_ADDRERRFC);
}
/**
  * @brief  Clear PKA RAM error flag.
  * @rmtoll
  *  CLRFR        RAMERRFC      LL_PKA_ClearFlag_RAMERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_ClearFlag_RAMERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CLRFR, PKA_CLRFR_RAMERRFC);
}
/**
  * @brief  Clear PKA OPERATION error flag.
  * @rmtoll
  *  CLRFR        OPERRFC      LL_PKA_ClearFlag_OPERR
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_ClearFlag_OPERR(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CLRFR, PKA_CLRFR_OPERRFC);
}
/**
  * @brief   Clear chaining mode flag.
  * @rmtoll
  *  CLRFR        CMFC     LL_PKA_ClearFlag_CMFC
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_ClearFlag_CMFC(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CLRFR, PKA_CLRFR_CMFC);
}
/**
  * @brief  Clear PKA end of operation flag.
  * @rmtoll
  *  CLRFR        PROCENDFC     LL_PKA_ClearFlag_PROCEND
  * @param  pkax PKA Instance.
  */
__STATIC_INLINE void LL_PKA_ClearFlag_PROCEND(PKA_TypeDef *pkax)
{
  STM32_SET_BIT(pkax->CLRFR, PKA_CLRFR_PROCENDFC);
}
/** @brief  Clear the PKA pending flags which are cleared by writing 1 in a specific bit.
  * @param  pkax specifies the PKA instance.
  * @param  clear_flag specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *            @arg @ref LL_PKA_FLAG_PROCEND End Of Operation
  *            @arg @ref LL_PKA_FLAG_ADDRERR Address error
  *            @arg @ref LL_PKA_FLAG_RAMERR RAM error
  *            @arg @ref LL_PKA_FLAG_OPERR Operation error
  *            @arg @ref LL_PKA_FLAG_CMF Chaining mode
  */
__STATIC_INLINE void LL_PKA_ClearFlag(PKA_TypeDef *pkax, uint32_t clear_flag)
{
  STM32_SET_BIT(pkax->CLRFR, clear_flag);
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
#endif /* defined(PKA) */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* STM32C5XX_LL_PKA_H */
