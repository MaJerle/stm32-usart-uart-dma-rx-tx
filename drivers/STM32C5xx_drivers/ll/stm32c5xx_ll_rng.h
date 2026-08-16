/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_rng.h
  * @brief   Header file for the RNG LL module.
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
#ifndef STM32C5XX_LL_RNG_H
#define STM32C5XX_LL_RNG_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"
/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */
#if defined (RNG)
/** @defgroup RNG_LL RNG
  * @{
  */
/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/* Private macros ----------------------------------------------------------------------------------------------------*/
/* Exported types ----------------------------------------------------------------------------------------------------*/
/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Constants LL RNG Constants
  * @{
  */
/** @defgroup RNG_LL_CED Clock Error Detection
  * @{
  */
#define LL_RNG_CED_ENABLE  0x00000000U /*!< Clock error detection enabled  */
#define LL_RNG_CED_DISABLE RNG_CR_CED  /*!< Clock error detection disabled */
/**
  * @}
  */
/** @defgroup RNG_LL_ARDIS Auto reset disable
  * @{
  */
#define LL_RNG_ARDIS_ENABLE   0x00000000U   /*!< ARDIS enabled automatic reset to clear SECS bit     */
#define LL_RNG_ARDIS_DISABLE  RNG_CR_ARDIS  /*!< ARDIS disabled no automatic reset to clear SECS bit */
/**
  * @}
  */
#define LL_RNG_HTCR1        0x01U      /*!< Additional health test register HTCR1     */
#define LL_RNG_HTCR2        0x02U      /*!< Additional health test register HTCR2     */
#define LL_RNG_HTCR3        0x03U      /*!< Additional health test register HTCR3     */
/** @defgroup RNG_LL_Clock_Divider_Factor  Value used to configure an internal
  *            programmable divider acting on the incoming RNG clock
  * @{
  */
#define LL_RNG_CLKDIV_BY_1       (0x00000000UL)                                                           /*!< No clock division                             */
#define LL_RNG_CLKDIV_BY_2       (RNG_CR_CLKDIV_0)                                                        /*!< 2 RNG clock cycles per internal RNG clock     */
#define LL_RNG_CLKDIV_BY_4       (RNG_CR_CLKDIV_1)                                                        /*!< 4 RNG clock cycles per internal RNG clock     */
#define LL_RNG_CLKDIV_BY_8       (RNG_CR_CLKDIV_1 | RNG_CR_CLKDIV_0)                                      /*!< 8 RNG clock cycles per internal RNG clock     */
#define LL_RNG_CLKDIV_BY_16      (RNG_CR_CLKDIV_2)                                                        /*!< 16 RNG clock cycles per internal RNG clock    */
#define LL_RNG_CLKDIV_BY_32      (RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_0)                                      /*!< 32 RNG clock cycles per internal RNG clock    */
#define LL_RNG_CLKDIV_BY_64      (RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_1)                                      /*!< 64 RNG clock cycles per internal RNG clock    */
#define LL_RNG_CLKDIV_BY_128     (RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_1 | RNG_CR_CLKDIV_0)                    /*!< 128 RNG clock cycles per internal RNG clock   */
#define LL_RNG_CLKDIV_BY_256     (RNG_CR_CLKDIV_3)                                                        /*!< 256 RNG clock cycles per internal RNG clock   */
#define LL_RNG_CLKDIV_BY_512     (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_0)                                      /*!< 512 RNG clock cycles per internal RNG clock   */
#define LL_RNG_CLKDIV_BY_1024    (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_1)                                      /*!< 1024 RNG clock cycles per internal RNG clock  */
#define LL_RNG_CLKDIV_BY_2048    (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_1 | RNG_CR_CLKDIV_0)                    /*!< 2048 RNG clock cycles per internal RNG clock  */
#define LL_RNG_CLKDIV_BY_4096    (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_2)                                      /*!< 4096 RNG clock cycles per internal RNG clock  */
#define LL_RNG_CLKDIV_BY_8192    (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_0)                    /*!< 8192 RNG clock cycles per internal RNG clock  */
#define LL_RNG_CLKDIV_BY_16384   (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_1)                    /*!< 16384 RNG clock cycles per internal RNG clock */
#define LL_RNG_CLKDIV_BY_32768   (RNG_CR_CLKDIV_3 | RNG_CR_CLKDIV_2 | RNG_CR_CLKDIV_1 | RNG_CR_CLKDIV_0)  /*!< 32768 RNG clock cycles per internal RNG clock */
/**
  * @}
  */
/** @defgroup RNG_LL_NIST_Compliance  NIST Compliance configuration
  * @{
  */
#define LL_RNG_NIST_COMPLIANT (0x00000000UL) /*!< Default NIST compliant configuration */
#define LL_RNG_CUSTOM_NIST    (RNG_CR_NISTC) /*!< Custom NIST configuration            */
/**
  * @}
  */
/** @defgroup RNG_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_RNG_ReadReg function.
  * @{
  */
#define LL_RNG_SR_DRDY RNG_SR_DRDY /*!< Register contains valid random data */
#define LL_RNG_SR_CECS RNG_SR_CECS /*!< Clock error current status          */
#define LL_RNG_SR_SECS RNG_SR_SECS /*!< Seed error current status           */
#define LL_RNG_SR_BUSY RNG_SR_BUSY /*! < RNG status BUSY or IDLE            */
#define LL_RNG_SR_CEIS RNG_SR_CEIS /*!< Clock error interrupt status        */
#define LL_RNG_SR_SEIS RNG_SR_SEIS /*!< Seed error interrupt status         */
/**
  * @}
  */
/** @defgroup RNG_LL_NSCR_Oscillator_Sources Oscillator Sources Defines
  * @{
  */
#define LL_RNG_OSC_1 RNG_NSCR_EN_OSC1
#define LL_RNG_OSC_2 RNG_NSCR_EN_OSC2
#define LL_RNG_OSC_3 RNG_NSCR_EN_OSC3
/**
  * @}
  */
/** @defgroup RNG_LL_NSCR_Noise_Sources_Ports Noise Sources Ports Defines
  * @{
  */
#define LL_RNG_NOISE_SRC_1 (0x01UL)
#define LL_RNG_NOISE_SRC_2 (0x02UL)
#define LL_RNG_NOISE_SRC_3 (0x04UL)
/**
  * @}
  */
/** @defgroup RNG_LL_EC_IT IT Defines
  * @{
  */
#define LL_RNG_CR_IE RNG_CR_IE /*!< RNG Interrupt enable */
/**
  * @}
  */
/**
  * @}
  */
/* Exported macros ---------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Macros LL RNG Macros
  * @{
  */
/** @defgroup RNG_LL_EM_WRITE_READ Common write and read register macros
  * @{
  */
/**
  * @brief  Write a value to an RNG register.
  * @param  instance RNG Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_RNG_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))
/**
  * @brief  Read a value from an RNG register.
  * @param  instance RNG Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_RNG_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */
/**
  * @}
  */
/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RNG_LL_Exported_Functions LL RNG Functions
  * @{
  */
/** @defgroup RNG_LL_EF_Configuration RNG Configuration functions
  * @{
  */
/**
  * @brief  Enable Random Number Generation.
  * @rmtoll
  *  CR           RNGEN         LL_RNG_Enable
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_Enable(RNG_TypeDef *rngx)
{
  STM32_SET_BIT(rngx->CR, RNG_CR_RNGEN);
}
/**
  * @brief  Disable Random Number Generation.
  * @rmtoll
  *  CR           RNGEN         LL_RNG_Disable
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_Disable(RNG_TypeDef *rngx)
{
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_RNGEN);
}
/**
  * @brief  Check if Random Number Generator is enabled.
  * @rmtoll
  *  CR           RNGEN         LL_RNG_IsEnabled
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabled(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_RNGEN) == (RNG_CR_RNGEN)) ? 1UL : 0UL);
}
/**
  * @brief  Enable Clock Error Detection.
  * @rmtoll
  *  CR           CED           LL_RNG_EnableClkErrorDetect
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_EnableClkErrorDetect(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_CED | RNG_CR_CONDRST, LL_RNG_CED_ENABLE | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Disable RNG Clock Error Detection.
  * @rmtoll
  *  CR           CED         LL_RNG_DisableClkErrorDetect
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_DisableClkErrorDetect(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_CED | RNG_CR_CONDRST, LL_RNG_CED_DISABLE | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Check if RNG Clock Error Detection is enabled.
  * @rmtoll
  *  CR           CED         LL_RNG_IsEnabledClkErrorDetect
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabledClkErrorDetect(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_CED) != (RNG_CR_CED)) ? 1UL : 0UL);
}
/**
  * @brief  Set RNG Conditioning Soft Reset bit.
  * @rmtoll
  *  CR           CONDRST          LL_RNG_EnableCondReset
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_EnableCondReset(RNG_TypeDef *rngx)
{
  STM32_SET_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Reset RNG  Conditioning Soft Reset bit.
  * @rmtoll
  *  CR           CONDRST         LL_RNG_DisableCondReset
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_DisableCondReset(RNG_TypeDef *rngx)
{
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Check if RNG Conditioning Soft Reset bit is set.
  * @rmtoll
  *  CR           CONDRST         LL_RNG_IsEnabledCondReset
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabledCondReset(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_CONDRST) == (RNG_CR_CONDRST)) ? 1UL : 0UL);
}
/**
  * @brief  Enable RNG Config Lock.
  * @rmtoll
  *  CR           CONFIGLOCK          LL_RNG_ConfigLock
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_ConfigLock(RNG_TypeDef *rngx)
{
  STM32_SET_BIT(rngx->CR, RNG_CR_CONFIGLOCK);
}
/**
  * @brief  Check if RNG Config Lock is enabled.
  * @rmtoll
  *  CR           CONFIGLOCK         LL_RNG_IsConfigLocked
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsConfigLocked(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_CONFIGLOCK) == (RNG_CR_CONFIGLOCK)) ? 1UL : 0UL);
}
/**
  * @brief  Enable NIST Compliance.
  * @rmtoll
  *  CR           NISTC         LL_RNG_EnableNistCompliance
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_EnableNistCompliance(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_NISTC | RNG_CR_CONDRST, LL_RNG_NIST_COMPLIANT | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Disable NIST Compliance.
  * @rmtoll
  *  CR           NISTC         LL_RNG_DisableNistCompliance
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_DisableNistCompliance(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_NISTC | RNG_CR_CONDRST, LL_RNG_CUSTOM_NIST | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Check if NIST Compliance is enabled.
  * @rmtoll
  *  CR           NISTC         LL_RNG_IsEnabledNistCompliance
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabledNistCompliance(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_NISTC) != (RNG_CR_NISTC)) ? 1UL : 0UL);
}
/**
  * @brief  Set RNG  Config1 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG1         LL_RNG_SetConfig1
  * @param  rngx RNG Instance
  * @param  config1 Value between 0 and 0x3F
  */
__STATIC_INLINE void LL_RNG_SetConfig1(RNG_TypeDef *rngx, uint32_t config1)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_RNG_CONFIG1 | RNG_CR_CONDRST, (config1 << RNG_CR_RNG_CONFIG1_Pos) | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Get RNG  Config1 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG1         LL_RNG_GetConfig1
  * @param  rngx RNG Instance
  * @retval Returned Value expressed on 6 bits : Value between 0 and 0x3F
  */
__STATIC_INLINE uint32_t LL_RNG_GetConfig1(const RNG_TypeDef *rngx)
{
  return (uint32_t)(STM32_READ_BIT(rngx->CR, RNG_CR_RNG_CONFIG1) >> RNG_CR_RNG_CONFIG1_Pos);
}
/**
  * @brief  Set RNG  Config2 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG2         LL_RNG_SetConfig2
  * @param  rngx RNG Instance
  * @param  config2 Value between 0 and 0x7
  */
__STATIC_INLINE void LL_RNG_SetConfig2(RNG_TypeDef *rngx, uint32_t config2)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_RNG_CONFIG2 | RNG_CR_CONDRST, (config2 << RNG_CR_RNG_CONFIG2_Pos) | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Get RNG  Config2 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG2         LL_RNG_GetConfig2
  * @param  rngx RNG Instance
  * @retval Returned Value expressed on 3 bits : Value between 0 and 0x7
  */
__STATIC_INLINE uint32_t LL_RNG_GetConfig2(const RNG_TypeDef *rngx)
{
  return (uint32_t)(STM32_READ_BIT(rngx->CR, RNG_CR_RNG_CONFIG2) >> RNG_CR_RNG_CONFIG2_Pos);
}
/**
  * @brief  Set RNG  Config3 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG3         LL_RNG_SetConfig3
  * @param  rngx RNG Instance
  * @param  config3 Value between 0 and 0xF
  */
__STATIC_INLINE void LL_RNG_SetConfig3(RNG_TypeDef *rngx, uint32_t config3)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_RNG_CONFIG3 | RNG_CR_CONDRST, (config3 << RNG_CR_RNG_CONFIG3_Pos) | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Get RNG  Config3 Configuration field value.
  * @rmtoll
  *  CR           RNG_CONFIG3         LL_RNG_GetConfig3
  * @param  rngx RNG Instance
  * @retval Returned Value expressed on 4 bits : Value between 0 and 0xF
  */
__STATIC_INLINE uint32_t LL_RNG_GetConfig3(const RNG_TypeDef *rngx)
{
  return (uint32_t)(STM32_READ_BIT(rngx->CR, RNG_CR_RNG_CONFIG3) >> RNG_CR_RNG_CONFIG3_Pos);
}
/**
  * @brief  Set RNG  Clock divider factor.
  * @rmtoll
  *  CR           CLKDIV         LL_RNG_SetClockDivider
  * @param  rngx RNG Instance
  * @param  divider can be one of the following values:
  *         @arg @ref LL_RNG_CLKDIV_BY_1
  *         @arg @ref LL_RNG_CLKDIV_BY_2
  *         @arg @ref LL_RNG_CLKDIV_BY_4
  *         @arg @ref LL_RNG_CLKDIV_BY_8
  *         @arg @ref LL_RNG_CLKDIV_BY_16
  *         @arg @ref LL_RNG_CLKDIV_BY_32
  *         @arg @ref LL_RNG_CLKDIV_BY_64
  *         @arg @ref LL_RNG_CLKDIV_BY_128
  *         @arg @ref LL_RNG_CLKDIV_BY_256
  *         @arg @ref LL_RNG_CLKDIV_BY_512
  *         @arg @ref LL_RNG_CLKDIV_BY_1024
  *         @arg @ref LL_RNG_CLKDIV_BY_2048
  *         @arg @ref LL_RNG_CLKDIV_BY_4096
  *         @arg @ref LL_RNG_CLKDIV_BY_8192
  *         @arg @ref LL_RNG_CLKDIV_BY_16384
  *         @arg @ref LL_RNG_CLKDIV_BY_32768
  */
__STATIC_INLINE void LL_RNG_SetClockDivider(RNG_TypeDef *rngx, uint32_t divider)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_CLKDIV | RNG_CR_CONDRST, divider | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Get RNG  Clock divider factor.
  * @rmtoll
  *  CR           CLKDIV         LL_RNG_GetClockDivider
  * @param  rngx RNG Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RNG_CLKDIV_BY_1
  *         @arg @ref LL_RNG_CLKDIV_BY_2
  *         @arg @ref LL_RNG_CLKDIV_BY_4
  *         @arg @ref LL_RNG_CLKDIV_BY_8
  *         @arg @ref LL_RNG_CLKDIV_BY_16
  *         @arg @ref LL_RNG_CLKDIV_BY_32
  *         @arg @ref LL_RNG_CLKDIV_BY_64
  *         @arg @ref LL_RNG_CLKDIV_BY_128
  *         @arg @ref LL_RNG_CLKDIV_BY_256
  *         @arg @ref LL_RNG_CLKDIV_BY_512
  *         @arg @ref LL_RNG_CLKDIV_BY_1024
  *         @arg @ref LL_RNG_CLKDIV_BY_2048
  *         @arg @ref LL_RNG_CLKDIV_BY_4096
  *         @arg @ref LL_RNG_CLKDIV_BY_8192
  *         @arg @ref LL_RNG_CLKDIV_BY_16384
  *         @arg @ref LL_RNG_CLKDIV_BY_32768
  */
__STATIC_INLINE uint32_t LL_RNG_GetClockDivider(const RNG_TypeDef *rngx)
{
  return (uint32_t)STM32_READ_BIT(rngx->CR, RNG_CR_CLKDIV);
}
/**
  * @brief  Set RNG configuration.
  * @rmtoll
  *  CR         CONFIG1   LL_RNG_SetConfig \n
  *  CR         CONFIG2   LL_RNG_SetConfig \n
  *  CR         CONFIG3   LL_RNG_SetConfig \n
  *  CR         CLKDIV    LL_RNG_SetConfig \n
  *  CR         NISTC     LL_RNG_SetConfig
  * @param  rngx RNG Instance
  * @param  config Specifies the configuration to be used
  */
__STATIC_INLINE void LL_RNG_SetConfig(RNG_TypeDef *rngx, uint32_t config)
{
  STM32_MODIFY_REG(rngx->CR, (RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_CLKDIV | RNG_CR_CED
                              | RNG_CR_NISTC | RNG_CR_CONDRST), (config | RNG_CR_CONDRST));
}
/**
  * @brief  Get RNG configuration.
  * @rmtoll
  *  CR         CONFIG1   LL_RNG_GetConfig \n
  *  CR         CONFIG2   LL_RNG_GetConfig \n
  *  CR         CONFIG3   LL_RNG_GetConfig \n
  *  CR         CLKDIV    LL_RNG_GetConfig \n
  *  CR         NISTC     LL_RNG_GetConfig
  * @param  rngx RNG Instance
  * @retval Return the configuration value
  */
__STATIC_INLINE uint32_t LL_RNG_GetConfig(const RNG_TypeDef *rngx)
{
  return STM32_READ_BIT(rngx->CR,
                        (RNG_CR_RNG_CONFIG1 | RNG_CR_RNG_CONFIG2 | RNG_CR_RNG_CONFIG3 | RNG_CR_CLKDIV | RNG_CR_CED
                         | RNG_CR_NISTC));
}
/**
  * @}
  */
/** @defgroup RNG_LL_EF_FLAG_Management FLAG Management
  * @{
  */
/**
  * @brief  Indicate if the RNG Data ready Flag is set or not.
  * @rmtoll
  *  SR           DRDY          LL_RNG_IsActiveFlag_DRDY
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_DRDY(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_DRDY) == (RNG_SR_DRDY)) ? 1UL : 0UL);
}
/**
  * @brief  Indicate if the Clock Error Current Status Flag is set or not.
  * @rmtoll
  *  SR           CECS          LL_RNG_IsActiveFlag_CECS
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_CECS(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_CECS) == (RNG_SR_CECS)) ? 1UL : 0UL);
}
/**
  * @brief  Indicate if the Seed Error Current Status Flag is set or not.
  * @rmtoll
  *  SR           SECS          LL_RNG_IsActiveFlag_SECS
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_SECS(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_SECS) == (RNG_SR_SECS)) ? 1UL : 0UL);
}
/**
  * @brief  Indicate if the Clock Error Interrupt Status Flag is set or not.
  * @rmtoll
  *  SR           CEIS          LL_RNG_IsActiveFlag_CEIS
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_CEIS(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_CEIS) == (RNG_SR_CEIS)) ? 1UL : 0UL);
}
/**
  * @brief  Indicate if the Seed Error Interrupt Status Flag is set or not.
  * @rmtoll
  *  SR           SEIS          LL_RNG_IsActiveFlag_SEIS
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_SEIS(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_SEIS) == (RNG_SR_SEIS)) ? 1UL : 0UL);
}
/**
  * @brief  Indicate if the RNG Busy Flag is set or not.
  * @rmtoll SR         BUSY          LL_RNG_IsActiveFlag_BUSY
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsActiveFlag_BUSY(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->SR, RNG_SR_BUSY) == (RNG_SR_BUSY)) ? 1UL : 0UL);
}
/**
  * @brief  Clear Clock Error interrupt Status (CEIS) Flag.
  * @rmtoll
  *  SR           CEIS          LL_RNG_ClearFlag_CEIS
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_ClearFlag_CEIS(RNG_TypeDef *rngx)
{
  STM32_WRITE_REG(rngx->SR, ~RNG_SR_CEIS);
}
/**
  * @brief  Clear Seed Error interrupt Status (SEIS) Flag.
  * @rmtoll
  *  SR           SEIS          LL_RNG_ClearFlag_SEIS
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_ClearFlag_SEIS(RNG_TypeDef *rngx)
{
  STM32_WRITE_REG(rngx->SR, ~RNG_SR_SEIS);
}
/**
  * @}
  */
/** @defgroup RNG_LL_EF_IT_Management IT Management
  * @{
  */
/**
  * @brief  Enable Random Number Generator Interrupt.
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @rmtoll
  *  CR           IE            LL_RNG_EnableIT
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_EnableIT(RNG_TypeDef *rngx)
{
  STM32_SET_BIT(rngx->CR, RNG_CR_IE);
}
/**
  * @brief  Disable Random Number Generator Interrupt.
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @rmtoll
  *  CR           IE            LL_RNG_DisableIT
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_DisableIT(RNG_TypeDef *rngx)
{
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_IE);
}
/**
  * @brief  Check if Random Number Generator Interrupt is enabled.
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @rmtoll
  *  CR           IE            LL_RNG_IsEnabledIT
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabledIT(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_IE) == (RNG_CR_IE)) ? 1UL : 0UL);
}
/**
  * @}
  */
/** @defgroup RNG_LL_EF_Data_Management Data Management
  * @{
  */
/**
  * @brief  Return a 32-bit random number value.
  * @rmtoll
  *  DR           RNDATA        LL_RNG_ReadRandData32
  * @param  rngx RNG Instance
  * @retval Generated 32-bit random value
  */
__STATIC_INLINE uint32_t LL_RNG_ReadRandData32(const RNG_TypeDef *rngx)
{
  return (uint32_t)(STM32_READ_REG(rngx->DR));
}
/**
  * @}
  */
/**
  * @brief  Enable Auto reset.
  * @rmtoll
  *  CR           ARDIS           LL_RNG_EnableArdis
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_EnableArdis(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_ARDIS | RNG_CR_CONDRST, LL_RNG_ARDIS_ENABLE | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Disable Auto reset.
  * @rmtoll
  *  CR           ARDIS         LL_RNG_DisableArdis
  * @param  rngx RNG Instance
  */
__STATIC_INLINE void LL_RNG_DisableArdis(RNG_TypeDef *rngx)
{
  STM32_MODIFY_REG(rngx->CR, RNG_CR_ARDIS | RNG_CR_CONDRST, LL_RNG_ARDIS_DISABLE | RNG_CR_CONDRST);
  STM32_CLEAR_BIT(rngx->CR, RNG_CR_CONDRST);
}
/**
  * @brief  Check if RNG Auto reset is enabled.
  * @rmtoll
  *  CR           ARDIS         LL_RNG_IsEnabledArdis
  * @param  rngx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RNG_IsEnabledArdis(const RNG_TypeDef *rngx)
{
  return ((STM32_READ_BIT(rngx->CR, RNG_CR_ARDIS) != (RNG_CR_ARDIS)) ? 1UL : 0UL);
}
/** @defgroup RNG_LL_EF_Noise_Source_Configuration Noise Source Configuration
  * @{
  */
/**
  * @brief  Set RNG Noise Source Configuration.
  * @rmtoll
  *  NSCR       NSCR       LL_RNG_SetOscNoiseSrc
  * @param  rngx RNG Instance
  * @param  osc be one of the following values:
  *         @arg @ref LL_RNG_OSC_1
  *         @arg @ref LL_RNG_OSC_2
  *         @arg @ref LL_RNG_OSC_3
  * @param  src can be one of the following values:
  *         @arg @ref LL_RNG_NOISE_SRC_1
  *         @arg @ref LL_RNG_NOISE_SRC_2
  *         @arg @ref LL_RNG_NOISE_SRC_3
  */
__STATIC_INLINE void LL_RNG_SetOscNoiseSrc(RNG_TypeDef *rngx, uint32_t osc, uint32_t src)
{
  STM32_MODIFY_REG(rngx->NSCR, osc, (src << STM32_POSITION_VAL(osc)));
}
/**
  * @brief  Get RNG Noise Source Configuration.
  * @rmtoll
  *  NSCR       NSCR       LL_RNG_GetOscNoiseSrc
  * @param  rngx RNG Instance
  * @param  osc be one of the following values:
  *         @arg @ref LL_RNG_OSC_1
  *         @arg @ref LL_RNG_OSC_2
  *         @arg @ref LL_RNG_OSC_3
  * @retval can be one of the following values:
  *         @arg @ref LL_RNG_NOISE_SRC_1
  *         @arg @ref LL_RNG_NOISE_SRC_2
  *         @arg @ref LL_RNG_NOISE_SRC_3
  */
__STATIC_INLINE uint32_t LL_RNG_GetOscNoiseSrc(const RNG_TypeDef *rngx, uint32_t osc)
{
  return (STM32_READ_BIT(rngx->NSCR, osc) >> STM32_POSITION_VAL(osc));
}
/**
  * @}
  */
/** @defgroup RNG_LL_EF_Health_Test_Control Health Test Control
  * @{
  */
/**
  * @brief  Set RNG Health Test Control.
  * @rmtoll HTCR[0]       HTCFG       LL_RNG_SetHealthConfig
  * @param  rngx RNG Instance
  * @param  htcfg can be values of 32 bits
  */
__STATIC_INLINE void LL_RNG_SetHealthConfig(RNG_TypeDef *rngx, uint32_t htcfg)
{
  STM32_WRITE_REG(rngx->HTCR[0], htcfg);
}
/**
  * @brief  Get RNG Health Test Control.
  * @rmtoll HTCR[0]         HTCFG        LL_RNG_GetHealthConfig
  * @param  rngx RNG Instance
  * @retval Return 32-bit RNG Health Test configuration
  */
__STATIC_INLINE uint32_t LL_RNG_GetHealthConfig(const RNG_TypeDef *rngx)
{
  return (uint32_t)STM32_READ_REG(rngx->HTCR[0]);
}
/**
  * @brief  Set RNG additional Health Tests Control.
  * @rmtoll HTCR     htcr_idx       HTCFG       LL_RNG_SetHealthFactorConfig
  * @param  rngx       RNG Instance
  * @param  htcr_idx   Additional health tests registers index can be one of the following values
  *         @arg @ref LL_RNG_HTCR1
  *         @arg @ref LL_RNG_HTCR2
  *         @arg @ref LL_RNG_HTCR3
  * @param  htcfg      can be values of 32 bits
  */
__STATIC_INLINE void LL_RNG_SetHealthFactorConfig(RNG_TypeDef *rngx, uint32_t htcr_idx, uint32_t htcfg)
{
  STM32_WRITE_REG(rngx->HTCR[htcr_idx], htcfg);
}
/**
  * @brief  Get RNG additional Health Tests Control.
  * @rmtoll HTCR    htcr_idx      HTCFG        LL_RNG_GetHealthFactorConfig
  * @param  rngx       RNG Instance
  * @param  htcr_idx   Additional health tests registers index
  * @retval Return a 32-bit RNG additional health test configuration
  */
__STATIC_INLINE uint32_t LL_RNG_GetHealthFactorConfig(const RNG_TypeDef *rngx, uint32_t htcr_idx)
{
  return (uint32_t)STM32_READ_REG(rngx->HTCR[htcr_idx]);
}
/**
  * @brief  Get RNG Health Tests Status.
  * @rmtoll HTSR    htsr_idx      LL_RNG_GetHealthTestStatus
  * @param  rngx       RNG Instance
  * @param  htsr_idx   Health tests registers status index
  * @retval Return 32-bit RNG Health Test Status
  */
__STATIC_INLINE uint32_t LL_RNG_GetHealthTestStatus(const RNG_TypeDef *rngx, uint32_t htsr_idx)
{
  return (uint32_t)STM32_READ_REG(rngx->HTSR[htsr_idx]);
}
/**
  * @brief  Set RNG noise source mask.
  * @rmtoll NSMR    htsr_idx      LL_RNG_GetNoiseSourceMask
  * @param  rngx       RNG Instance
  * @param  nsmr      can be values of 32 bits
  */
__STATIC_INLINE void LL_RNG_SetNoiseSourceMask(RNG_TypeDef *rngx, uint32_t nsmr)
{
  STM32_WRITE_REG(rngx->NSMR, nsmr);
}
/**
  * @brief  Get RNG noise source mask.
  * @rmtoll NSMR    htsr_idx      LL_RNG_GetNoiseSourceMask
  * @param  rngx       RNG Instance
  * @retval Return 32-bit RNG Noise Source Mask
  */
__STATIC_INLINE uint32_t LL_RNG_GetNoiseSourceMask(const RNG_TypeDef *rngx)
{
  return (uint32_t)STM32_READ_REG(rngx->NSMR);
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
#endif /* RNG */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* STM32C5XX_LL_RNG_H */
