/**
  ******************************************************************************
  * @file    stm32c5xx_ll_i2c.h
  * @brief   Header file of I2C LL module.
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
#ifndef STM32C5xx_LL_I2C_H
#define STM32C5xx_LL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(I2C1) || defined(I2C2)

/** @defgroup I2C_LL I2C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Constants LL I2C Constants
  * @{
  */

/** @defgroup I2C_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_I2C_WRITE_REG function.
  * @{
  */
#define LL_I2C_ICR_ADDRCF                   I2C_ICR_ADDRCF          /*!< Address Matched flag   */
#define LL_I2C_ICR_NACKCF                   I2C_ICR_NACKCF          /*!< Not Acknowledge flag   */
#define LL_I2C_ICR_STOPCF                   I2C_ICR_STOPCF          /*!< Stop detection flag    */
#define LL_I2C_ICR_BERRCF                   I2C_ICR_BERRCF          /*!< Bus error flag         */
#define LL_I2C_ICR_ARLOCF                   I2C_ICR_ARLOCF          /*!< Arbitration Lost flag  */
#define LL_I2C_ICR_OVRCF                    I2C_ICR_OVRCF           /*!< Overrun/Underrun flag  */
#define LL_I2C_ICR_PECCF                    I2C_ICR_PECCF           /*!< PEC error flag         */
#define LL_I2C_ICR_TIMOUTCF                 I2C_ICR_TIMOUTCF        /*!< Timeout detection flag */
#define LL_I2C_ICR_ALERTCF                  I2C_ICR_ALERTCF         /*!< Alert flag             */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_I2C_READ_REG function.
  * @{
  */
#define LL_I2C_ISR_TXE                      I2C_ISR_TXE             /*!< Transmit data register empty        */
#define LL_I2C_ISR_TXIS                     I2C_ISR_TXIS            /*!< Transmit interrupt status           */
#define LL_I2C_ISR_RXNE                     I2C_ISR_RXNE            /*!< Receive data register not empty     */
#define LL_I2C_ISR_ADDR                     I2C_ISR_ADDR            /*!< Address matched (slave mode)        */
#define LL_I2C_ISR_NACKF                    I2C_ISR_NACKF           /*!< Not Acknowledge received flag       */
#define LL_I2C_ISR_STOPF                    I2C_ISR_STOPF           /*!< Stop detection flag                 */
#define LL_I2C_ISR_TC                       I2C_ISR_TC              /*!< Transfer Complete (master mode)     */
#define LL_I2C_ISR_TCR                      I2C_ISR_TCR             /*!< Transfer Complete Reload            */
#define LL_I2C_ISR_BERR                     I2C_ISR_BERR            /*!< Bus error                           */
#define LL_I2C_ISR_ARLO                     I2C_ISR_ARLO            /*!< Arbitration lost                    */
#define LL_I2C_ISR_OVR                      I2C_ISR_OVR             /*!< Overrun/Underrun (slave mode)       */
#define LL_I2C_ISR_PECERR                   I2C_ISR_PECERR          /*!< PEC Error in reception (SMBus mode) */
#define LL_I2C_ISR_TIMEOUT                  I2C_ISR_TIMEOUT         /*!< Timeout detection flag (SMBus mode) */
#define LL_I2C_ISR_ALERT                    I2C_ISR_ALERT           /*!< SMBus alert (SMBus mode)            */
#define LL_I2C_ISR_BUSY                     I2C_ISR_BUSY            /*!< Bus busy                            */
#define LL_I2C_ISR_DIR                      I2C_ISR_DIR             /*!< Direction */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_I2C_READ_REG and  LL_I2C_WRITE_REG functions.
  * @{
  */
#define LL_I2C_CR1_TXIE                     I2C_CR1_TXIE            /*!< TX Interrupt enable                         */
#define LL_I2C_CR1_RXIE                     I2C_CR1_RXIE            /*!< RX Interrupt enable                         */
#define LL_I2C_CR1_ADDRIE                   I2C_CR1_ADDRIE          /*!< Address match Interrupt enable (slave only) */
#define LL_I2C_CR1_NACKIE                   I2C_CR1_NACKIE          /*!< Not acknowledge received Interrupt enable   */
#define LL_I2C_CR1_STOPIE                   I2C_CR1_STOPIE          /*!< STOP detection Interrupt enable             */
#define LL_I2C_CR1_TCIE                     I2C_CR1_TCIE            /*!< Transfer Complete interrupt enable          */
#define LL_I2C_CR1_ERRIE                    I2C_CR1_ERRIE           /*!< Error interrupts enable                     */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_PERIPHERAL_MODE Peripheral Mode
  * @{
  */
#define LL_I2C_MODE_I2C                    0x00000000U              /*!< I2C Master or Slave mode                 */
#define LL_I2C_MODE_SMBUS_HOST             I2C_CR1_SMBHEN           /*!< SMBus Host address acknowledge           */
#define LL_I2C_MODE_SMBUS_SLAVE            0x00000000U              /*!< SMBus Slave default mode
                                                                         (Default address not acknowledge)        */
#define LL_I2C_MODE_SMBUS_SLAVE_ARP        I2C_CR1_SMBDEN           /*!< SMBus Slave Default address acknowledge */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_ANALOGFILTER_SELECTION Analog Filter Selection
  * @{
  */
#define LL_I2C_ANALOGFILTER_ENABLE          0x00000000U             /*!< Analog filter is enabled.  */
#define LL_I2C_ANALOGFILTER_DISABLE         I2C_CR1_ANFOFF          /*!< Analog filter is disabled. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_ADDRESSING_MODE Master Addressing Mode
  * @{
  */
#define LL_I2C_ADDRESSING_MODE_7BIT         0x00000000U              /*!< Master operates in 7-bit addressing mode. */
#define LL_I2C_ADDRESSING_MODE_10BIT        I2C_CR2_ADD10            /*!< Master operates in 10-bit addressing mode. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_OWNADDRESS1 Own Address 1 Length
  * @{
  */
#define LL_I2C_OWNADDRESS1_7BIT             0x00000000U             /*!< Own address 1 is a 7-bit address. */
#define LL_I2C_OWNADDRESS1_10BIT            I2C_OAR1_OA1MODE        /*!< Own address 1 is a 10-bit address. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_OWNADDRESS2 Own Address 2 Masks
  * @{
  */
#define LL_I2C_OWNADDRESS2_NOMASK           I2C_OAR2_OA2NOMASK      /*!< Own Address2 No mask.                 */
#define LL_I2C_OWNADDRESS2_MASK01           I2C_OAR2_OA2MASK01      /*!< Only Address2 bits[7:2] are compared. */
#define LL_I2C_OWNADDRESS2_MASK02           I2C_OAR2_OA2MASK02      /*!< Only Address2 bits[7:3] are compared. */
#define LL_I2C_OWNADDRESS2_MASK03           I2C_OAR2_OA2MASK03      /*!< Only Address2 bits[7:4] are compared. */
#define LL_I2C_OWNADDRESS2_MASK04           I2C_OAR2_OA2MASK04      /*!< Only Address2 bits[7:5] are compared. */
#define LL_I2C_OWNADDRESS2_MASK05           I2C_OAR2_OA2MASK05      /*!< Only Address2 bits[7:6] are compared. */
#define LL_I2C_OWNADDRESS2_MASK06           I2C_OAR2_OA2MASK06      /*!< Only Address2 bits[7] are compared.   */
#define LL_I2C_OWNADDRESS2_MASK07           I2C_OAR2_OA2MASK07      /*!< No comparison is done.
                                                                         All Address2 are acknowledged.        */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_I2C_ACKNOWLEDGE Acknowledge Generation
  * @{
  */
#define LL_I2C_ACK                          0x00000000U              /*!< ACK is sent after current received byte. */
#define LL_I2C_NACK                         I2C_CR2_NACK             /*!< NACK is sent after current received byte. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_ADDRSLAVE Slave Address Length
  * @{
  */
#define LL_I2C_ADDRSLAVE_7BIT               0x00000000U              /*!< Slave Address in 7-bit. */
#define LL_I2C_ADDRSLAVE_10BIT              I2C_CR2_ADD10            /*!< Slave Address in 10-bit. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_REQUEST Transfer Request Direction
  * @{
  */
#define LL_I2C_REQUEST_WRITE                0x00000000U              /*!< Master request a write transfer. */
#define LL_I2C_REQUEST_READ                 I2C_CR2_RD_WRN           /*!< Master request a read transfer.  */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_MODE Transfer End Mode
  * @{
  */
#define LL_I2C_MODE_RELOAD                  I2C_CR2_RELOAD           /*!< Enable I2C Reload mode.     */
#define LL_I2C_MODE_AUTOEND                 I2C_CR2_AUTOEND          /*!< Enable I2C Automatic end mode
                                                                          with no HW PEC comparison.  */
#define LL_I2C_MODE_SOFTEND                 0x00000000U              /*!< Enable I2C Software end mode
                                                                          with no HW PEC comparison.  */
#define LL_I2C_MODE_SMBUS_RELOAD            LL_I2C_MODE_RELOAD       /*!< Enable SMBUS Automatic end mode
                                                                          with HW PEC comparison.     */
#define LL_I2C_MODE_SMBUS_AUTOEND_NO_PEC    LL_I2C_MODE_AUTOEND      /*!< Enable SMBUS Automatic end mode
                                                                          with HW PEC comparison.     */
#define LL_I2C_MODE_SMBUS_SOFTEND_NO_PEC    LL_I2C_MODE_SOFTEND      /*!< Enable SMBUS Software end mode
                                                                          with HW PEC comparison.     */
#define LL_I2C_MODE_SMBUS_AUTOEND_WITH_PEC  (uint32_t)(LL_I2C_MODE_AUTOEND | I2C_CR2_PECBYTE)
/*!< Enable SMBUS Automatic end mode with HW PEC comparison.   */
#define LL_I2C_MODE_SMBUS_SOFTEND_WITH_PEC  (uint32_t)(LL_I2C_MODE_SOFTEND | I2C_CR2_PECBYTE)
/*!< Enable SMBUS Software end mode with HW PEC comparison.    */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_GENERATE Start And Stop Generation
  * @{
  */
#define LL_I2C_GENERATE_NOSTARTSTOP         0x00000000U
/*!< Don't Generate Stop and Start condition. */
#define LL_I2C_GENERATE_STOP                (uint32_t)(0x80000000U | I2C_CR2_STOP)
/*!< Generate Stop condition (Size must be set to 0).      */
#define LL_I2C_GENERATE_START_READ          (uint32_t)(0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN)
/*!< Generate Start for read request. */
#define LL_I2C_GENERATE_START_WRITE         (uint32_t)(0x80000000U | I2C_CR2_START)
/*!< Generate Start for write request. */
#define LL_I2C_GENERATE_RESTART_7BIT_READ   (uint32_t)(0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN)
/*!< Generate Restart for read request, slave 7Bit address.  */
#define LL_I2C_GENERATE_RESTART_7BIT_WRITE  (uint32_t)(0x80000000U | I2C_CR2_START)
/*!< Generate Restart for write request, slave 7Bit address. */
#define LL_I2C_GENERATE_RESTART_10BIT_READ  (uint32_t)(0x80000000U | I2C_CR2_START | \
                                                       I2C_CR2_RD_WRN | I2C_CR2_HEAD10R)
/*!< Generate Restart for read request, slave 10Bit address. */
#define LL_I2C_GENERATE_RESTART_10BIT_WRITE (uint32_t)(0x80000000U | I2C_CR2_START)
/*!< Generate Restart for write request, slave 10Bit address. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_DIRECTION Read Write Direction
  * @{
  */
#define LL_I2C_DIRECTION_WRITE              0x00000000U              /*!< Write transfer request by master,
                                                                          slave enters receiver mode.  */
#define LL_I2C_DIRECTION_READ               I2C_ISR_DIR              /*!< Read transfer request by master,
                                                                          slave enters transmitter mode. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_DMA_REG_DATA DMA Register Data
  * @{
  */
#define LL_I2C_DMA_REG_DATA_TRANSMIT        0x00000000U              /*!< Get address of data register used for
                                                                          transmission */
#define LL_I2C_DMA_REG_DATA_RECEIVE         0x00000001U              /*!< Get address of data register used for
                                                                          reception */
/**
  * @}
  */


/** @defgroup I2C_LL_EC_SMBUS_TIMEOUTA_MODE SMBus timeout_a Mode SCL SDA Timeout
  * @{
  */
#define LL_I2C_SMBUS_TIMEOUTA_MODE_SCL_LOW      0x00000000U          /*!< timeout_a is used to detect
                                                                          SCL low level timeout.              */
#define LL_I2C_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH I2C_TIMEOUTR_TIDLE   /*!< timeout_a is used to detect
                                                                          both SCL and SDA high level timeout. */
/**
  * @}
  */

/** @defgroup I2C_LL_EC_SMBUS_TIMEOUT_SELECTION SMBus Timeout Selection
  * @{
  */
#define LL_I2C_SMBUS_TIMEOUTA               I2C_TIMEOUTR_TIMOUTEN                 /*!< timeout_a enable bit          */
#define LL_I2C_SMBUS_TIMEOUTB               I2C_TIMEOUTR_TEXTEN                   /*!< timeout_b (extended clock)
                                                                                       enable bit                   */
#define LL_I2C_SMBUS_ALL_TIMEOUT            (uint32_t)(LL_I2C_SMBUS_TIMEOUTA | \
                                                       LL_I2C_SMBUS_TIMEOUTB)     /*!< timeout_a and timeout_b
                                                                                      (extended clock) enable bits */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Macros LL I2C Macros
  * @{
  */

/** @defgroup I2C_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in I2C register.
  * @param  instance I2C instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_I2C_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in I2C register.
  * @param  instance I2C instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_I2C_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/** @defgroup I2C_LL_EM_CONVERT_TIMINGS Convert SDA SCL timings
  * @{
  */
/**
  * @brief  Configure the SDA setup, hold time and the SCL high, low period.
  * @param  prescaller This parameter must be a value between  Min_Data=0 and Max_Data=0xF.
  * @param  setup_time This parameter must be a value between Min_Data=0 and Max_Data=0xF.
                           (tscldel = (SCLDEL+1)xtpresc)
  * @param  hold_time  This parameter must be a value between Min_Data=0 and Max_Data=0xF.
                           (tsdadel = SDADELxtpresc)
  * @param  sclh_priod This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
                            (tsclh = (SCLH+1)xtpresc)
  * @param  scll_period This parameter must be a value between  Min_Data=0 and Max_Data=0xFF.
                            (tscll = (SCLL+1)xtpresc)
  * @retval Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
#define LL_I2C_CONVERT_TIMINGS(prescaller, setup_time, hold_time, sclh_priod, scll_period) \
  ((((uint32_t)(prescaller) << I2C_TIMINGR_PRESC_Pos) & I2C_TIMINGR_PRESC) \
   | (((uint32_t)(setup_time) << I2C_TIMINGR_SCLDEL_Pos) & I2C_TIMINGR_SCLDEL) \
   | (((uint32_t)(hold_time) << I2C_TIMINGR_SDADEL_Pos) & I2C_TIMINGR_SDADEL) \
   | (((uint32_t)(sclh_priod) << I2C_TIMINGR_SCLH_Pos) & I2C_TIMINGR_SCLH) \
   | (((uint32_t)(scll_period) << I2C_TIMINGR_SCLL_Pos) & I2C_TIMINGR_SCLL))
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Functions LL I2C Functions
  * @{
  */

/** @defgroup I2C_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable I2C peripheral (PE = 1).
  * @rmtoll
  *  CR1          PE            LL_I2C_Enable
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_Enable(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_PE);
}

/**
  * @brief  Disable I2C peripheral (PE = 0).
  * @param  p_i2c I2C instance.
  * @note   When PE = 0, the I2C SCL and SDA lines are released.
  *         Internal state machines and status bits are put back to their reset value.
  *         When cleared, PE must be kept low for at least 3 APB clock cycles.
  * @rmtoll
  *  CR1          PE            LL_I2C_Disable
  */
__STATIC_INLINE void LL_I2C_Disable(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_PE);
}

/**
  * @brief  Check if the I2C peripheral is enabled or disabled.
  * @rmtoll
  *  CR1          PE            LL_I2C_IsEnabled
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabled(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_PE) == (I2C_CR1_PE)) ? 1UL : 0UL);
}

/**
  * @brief  Configure Noise Filters (Analog and Digital).
  * @param  p_i2c I2C instance.
  * @param  analog_filter This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ANALOGFILTER_ENABLE
  *         @arg @ref LL_I2C_ANALOGFILTER_DISABLE
  * @param  digital_filter This parameter must be a value between Min_Data=0x00 (Digital filter disabled)
            and Max_Data=0x0F (Digital filter enabled and filtering capability up to 15*ti2cclk).
  *         This parameter is used to configure the digital noise filter on SDA and SCL input.
  *         The digital filter filters spikes with a length of up to DNF[3:0]*ti2cclk.
  * @note   If the analog filter is also enabled, the digital filter is added to analog filter.
  *         The filters can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          ANFOFF        LL_I2C_ConfigFilters \n
  *  CR1          DNF           LL_I2C_ConfigFilters
  */
__STATIC_INLINE void LL_I2C_ConfigFilters(I2C_TypeDef *p_i2c, uint32_t analog_filter, uint32_t digital_filter)
{
  STM32_MODIFY_REG(p_i2c->CR1, I2C_CR1_ANFOFF | I2C_CR1_DNF, analog_filter | (digital_filter << I2C_CR1_DNF_Pos));
}

/**
  * @brief  Configure Digital Noise Filter.
  * @param  p_i2c I2C instance.
  * @param  digital_filter This parameter must be a value between Min_Data=0x00 (Digital filter disabled)
            and Max_Data=0x0F (Digital filter enabled and filtering capability up to 15*ti2cclk).
  *         This parameter is used to configure the digital noise filter on SDA and SCL input.
  *         The digital filter filters spikes with a length of up to DNF[3:0]*ti2cclk.
  * @note   If the analog filter is also enabled, the digital filter is added to analog filter.
  *         This filter can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          DNF           LL_I2C_SetDigitalFilter
  */
__STATIC_INLINE void LL_I2C_SetDigitalFilter(I2C_TypeDef *p_i2c, uint32_t digital_filter)
{
  STM32_MODIFY_REG(p_i2c->CR1, I2C_CR1_DNF, digital_filter << I2C_CR1_DNF_Pos);
}

/**
  * @brief  Get the current Digital Noise Filter configuration.
  * @rmtoll
  *  CR1          DNF           LL_I2C_GetDigitalFilter
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t LL_I2C_GetDigitalFilter(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR1, I2C_CR1_DNF) >> I2C_CR1_DNF_Pos);
}

/**
  * @brief  Enable Analog Noise Filter.
  * @param  p_i2c I2C instance.
  * @note   This filter can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          ANFOFF        LL_I2C_EnableAnalogFilter
  */
__STATIC_INLINE void LL_I2C_EnableAnalogFilter(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_ANFOFF);
}

/**
  * @brief  Disable Analog Noise Filter.
  * @param  p_i2c I2C instance.
  * @note   This filter can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          ANFOFF        LL_I2C_DisableAnalogFilter
  */
__STATIC_INLINE void LL_I2C_DisableAnalogFilter(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_ANFOFF);
}

/**
  * @brief  Check if Analog Noise Filter is enabled or disabled.
  * @rmtoll
  *  CR1          ANFOFF        LL_I2C_IsEnabledAnalogFilter
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledAnalogFilter(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_ANFOFF) != (I2C_CR1_ANFOFF)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA transmission requests.
  * @rmtoll
  *  CR1          TXDMAEN       LL_I2C_EnableDMAReq_TX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableDMAReq_TX(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_TXDMAEN);
}

/**
  * @brief  Disable DMA transmission requests.
  * @rmtoll
  *  CR1          TXDMAEN       LL_I2C_DisableDMAReq_TX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableDMAReq_TX(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_TXDMAEN);
}

/**
  * @brief  Check if DMA transmission requests are enabled or disabled.
  * @rmtoll
  *  CR1          TXDMAEN       LL_I2C_IsEnabledDMAReq_TX
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledDMAReq_TX(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_TXDMAEN) == (I2C_CR1_TXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA reception requests.
  * @rmtoll
  *  CR1          RXDMAEN       LL_I2C_EnableDMAReq_RX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableDMAReq_RX(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_RXDMAEN);
}

/**
  * @brief  Disable DMA reception requests.
  * @rmtoll
  *  CR1          RXDMAEN       LL_I2C_DisableDMAReq_RX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableDMAReq_RX(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_RXDMAEN);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  * @rmtoll
  *  CR1          RXDMAEN       LL_I2C_IsEnabledDMAReq_RX
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledDMAReq_RX(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_RXDMAEN) == (I2C_CR1_RXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Get the data register address used for DMA transfer.
  * @rmtoll
  *  TXDR         TXDATA        LL_I2C_DMA_GetRegAddr \n
  *  RXDR         RXDATA        LL_I2C_DMA_GetRegAddr
  * @param  p_i2c I2C instance
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_I2C_DMA_REG_DATA_TRANSMIT
  *         @arg @ref LL_I2C_DMA_REG_DATA_RECEIVE
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_I2C_DMA_GetRegAddr(const I2C_TypeDef *p_i2c, uint32_t direction)
{
  uint32_t data_reg_addr;

  if (direction == LL_I2C_DMA_REG_DATA_TRANSMIT)
  {
    /* return address of TXDR register */
    data_reg_addr = (uint32_t) &(p_i2c->TXDR);
  }
  else
  {
    /* return address of RXDR register */
    data_reg_addr = (uint32_t) &(p_i2c->RXDR);
  }

  return data_reg_addr;
}

/**
  * @brief  Get the tx data register address used for DMA transfer.
  * @rmtoll
  *  TXDR         TXDATA        LL_I2C_DMA_GetRegAddrTx
  * @param  p_i2c I2C instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_I2C_DMA_GetRegAddrTx(const I2C_TypeDef *p_i2c)
{
  return (uint32_t) &(p_i2c->TXDR);
}

/**
  * @brief  Get the rx data register address used for DMA transfer.
  * @rmtoll
  *  RXDR         RXDATA        LL_I2C_DMA_GetRegAddrRx
  * @param  p_i2c I2C instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_I2C_DMA_GetRegAddrRx(const I2C_TypeDef *p_i2c)
{
  return (uint32_t) &(p_i2c->RXDR);
}

/**
  * @brief  Enable Clock stretching.
  * @param  p_i2c I2C instance.
  * @note   This bit can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          NOSTRETCH     LL_I2C_EnableClockStretching
  */
__STATIC_INLINE void LL_I2C_EnableClockStretching(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_NOSTRETCH);
}

/**
  * @brief  Disable Clock stretching.
  * @param  p_i2c I2C instance.
  * @note   This bit can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  CR1          NOSTRETCH     LL_I2C_DisableClockStretching
  */
__STATIC_INLINE void LL_I2C_DisableClockStretching(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_NOSTRETCH);
}

/**
  * @brief  Check if Clock stretching is enabled or disabled.
  * @rmtoll
  *  CR1          NOSTRETCH     LL_I2C_IsEnabledClockStretching
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledClockStretching(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_NOSTRETCH) != (I2C_CR1_NOSTRETCH)) ? 1UL : 0UL);
}

/**
  * @brief  Enable hardware byte control in slave mode.
  * @rmtoll
  *  CR1          SBC           LL_I2C_EnableSlaveByteControl
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableSlaveByteControl(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_SBC);
}

/**
  * @brief  Disable hardware byte control in slave mode.
  * @rmtoll
  *  CR1          SBC           LL_I2C_DisableSlaveByteControl
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableSlaveByteControl(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_SBC);
}

/**
  * @brief  Check if hardware byte control in slave mode is enabled or disabled.
  * @rmtoll
  *  CR1          SBC           LL_I2C_IsEnabledSlaveByteControl
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledSlaveByteControl(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_SBC) == (I2C_CR1_SBC)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Wakeup from STOP.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_I2C_WAKEUP_FROMSTOP_INSTANCE(p_i2c) can be used to check whether or not
  *         WakeUpFromStop feature is supported by the p_i2c instance.
  * @note   This bit can only be programmed when Digital Filter is disabled.
  * @rmtoll
  *  CR1          WUPEN         LL_I2C_EnableWakeUpFromStop
  */
__STATIC_INLINE void LL_I2C_EnableWakeUpFromStop(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_WUPEN);
}

/**
  * @brief  Disable Wakeup from STOP.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_I2C_WAKEUP_FROMSTOP_INSTANCE(p_i2c) can be used to check whether or not
  *         WakeUpFromStop feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          WUPEN         LL_I2C_DisableWakeUpFromStop
  */
__STATIC_INLINE void LL_I2C_DisableWakeUpFromStop(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_WUPEN);
}

/**
  * @brief  Check if Wakeup from STOP is enabled or disabled.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_I2C_WAKEUP_FROMSTOP_INSTANCE(p_i2c) can be used to check whether or not
  *         WakeUpFromStop feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          WUPEN         LL_I2C_IsEnabledWakeUpFromStop
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledWakeUpFromStop(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_WUPEN) == (I2C_CR1_WUPEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable General Call.
  * @param  p_i2c I2C instance.
  * @note   When enabled the Address 0x00 is ACKed.
  * @rmtoll
  *  CR1          GCEN          LL_I2C_EnableGeneralCall
  */
__STATIC_INLINE void LL_I2C_EnableGeneralCall(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_GCEN);
}

/**
  * @brief  Disable General Call.
  * @param  p_i2c I2C instance.
  * @note   When disabled the Address 0x00 is NACKed.
  * @rmtoll
  *  CR1          GCEN          LL_I2C_DisableGeneralCall
  */
__STATIC_INLINE void LL_I2C_DisableGeneralCall(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_GCEN);
}

/**
  * @brief  Check if General Call is enabled or disabled.
  * @param  p_i2c I2C instance.
  * @rmtoll
  *  CR1          GCEN          LL_I2C_IsEnabledGeneralCall
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledGeneralCall(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_GCEN) == (I2C_CR1_GCEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable I2C Fast Mode Plus (FMP = 1).
  * @param  p_i2c I2C instance.
  * @note   20mA I/O drive enable
  * @rmtoll
  *  CR1          FMP           LL_I2C_EnableFastModePlus
  */
__STATIC_INLINE void LL_I2C_EnableFastModePlus(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_FMP);
}

/**
  * @brief  Disable I2C Fast Mode Plus (FMP = 0).
  * @param  p_i2c I2C instance.
  * @note   20mA I/O drive disable
  * @rmtoll
  *  CR1          FMP           LL_I2C_DisableFastModePlus
  */
__STATIC_INLINE void LL_I2C_DisableFastModePlus(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_FMP);
}

/**
  * @brief  Check if the I2C Fast Mode Plus is enabled or disabled.
  * @rmtoll
  *  CR1          FMP           LL_I2C_IsEnabledFastModePlus
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledFastModePlus(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_FMP) == (I2C_CR1_FMP)) ? 1UL : 0UL);
}

/**
  * @brief  Enable automatic clear of ADDR flag.
  * @rmtoll
  *  CR1          ADDRACLR      LL_I2C_EnableAutoClearFlag_ADDR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableAutoClearFlag_ADDR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_ADDRACLR);
}

/**
  * @brief  Disable automatic clear of ADDR flag.
  * @rmtoll
  *  CR1          ADDRACLR      LL_I2C_DisableAutoClearFlag_ADDR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableAutoClearFlag_ADDR(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_ADDRACLR);
}

/**
  * @brief  Check if the automatic clear of ADDR flag is enabled or disabled.
  * @rmtoll
  *  CR1          ADDRACLR      LL_I2C_IsEnabledAutoClearFlag_ADDR
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledAutoClearFlag_ADDR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_ADDRACLR) == (I2C_CR1_ADDRACLR)) ? 1UL : 0UL);
}

/**
  * @brief  Enable automatic clear of STOP flag.
  * @rmtoll
  *  CR1          STOPFACLR     LL_I2C_EnableAutoClearFlag_STOP
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableAutoClearFlag_STOP(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_STOPFACLR);
}

/**
  * @brief  Disable automatic clear of STOP flag.
  * @rmtoll
  *  CR1          STOPFACLR     LL_I2C_DisableAutoClearFlag_STOP
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableAutoClearFlag_STOP(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_STOPFACLR);
}

/**
  * @brief  Check if the automatic clear of STOP flag is enabled or disabled.
  * @rmtoll
  *  CR1          STOPFACLR     LL_I2C_IsEnabledAutoClearFlag_STOP
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledAutoClearFlag_STOP(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_STOPFACLR) == (I2C_CR1_STOPFACLR)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the Master to operate in 7-bit or 10-bit addressing mode.
  * @param  p_i2c I2C instance.
  * @param  addressing_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  * @note   Changing this bit is not allowed, when the START bit is set.
  * @rmtoll
  *  CR2          ADD10         LL_I2C_SetMasterAddressingMode
  */
__STATIC_INLINE void LL_I2C_SetMasterAddressingMode(I2C_TypeDef *p_i2c, uint32_t addressing_mode)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_ADD10, addressing_mode);
}

/**
  * @brief  Get the Master addressing mode.
  * @rmtoll
  *  CR2          ADD10         LL_I2C_GetMasterAddressingMode
  * @param  p_i2c I2C instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  */
__STATIC_INLINE uint32_t LL_I2C_GetMasterAddressingMode(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR2, I2C_CR2_ADD10));
}

/**
  * @brief  Set the Own Address1.
  * @rmtoll
  *  OAR1         OA1           LL_I2C_SetOwnAddress1 \n
  *  OAR1         OA1MODE       LL_I2C_SetOwnAddress1
  * @param  p_i2c I2C instance.
  * @param  own_address1 This parameter must be a value between Min_Data=0 and Max_Data=0x3FF.
  * @param  own_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_OWNADDRESS1_7BIT
  *         @arg @ref LL_I2C_OWNADDRESS1_10BIT
  */
__STATIC_INLINE void LL_I2C_SetOwnAddress1(I2C_TypeDef *p_i2c, uint32_t own_address1, uint32_t own_addr_size)
{
  STM32_MODIFY_REG(p_i2c->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE, own_address1 | own_addr_size);
}

/**
  * @brief  Get the Own Address1.
  * @rmtoll
  *  OAR1         OA1           LL_I2C_SetOwnAddress1
  * @param  p_i2c I2C instance.
  * @retval Own Address1.
  */
__STATIC_INLINE uint32_t LL_I2C_GetOwnAddress1(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->OAR1, I2C_OAR1_OA1));
}

/**
  * @brief  Enable acknowledge on Own Address1 match address.
  * @rmtoll
  *  OAR1         OA1EN         LL_I2C_EnableOwnAddress1
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableOwnAddress1(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->OAR1, I2C_OAR1_OA1EN);
}

/**
  * @brief  Disable acknowledge on Own Address1 match address.
  * @rmtoll
  *  OAR1         OA1EN         LL_I2C_DisableOwnAddress1
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableOwnAddress1(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->OAR1, I2C_OAR1_OA1EN);
}

/**
  * @brief  Disable acknowledge on Own Address1 match address and mode.
  * @rmtoll
  *  OAR1         OA1EN         LL_I2C_DisableOwnAddress1 \n
  *  OAR1         OA1MODE       LL_I2C_DisableOwnAddress1
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableOwnAddress1AndMode(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->OAR1, I2C_OAR1_OA1EN | I2C_OAR1_OA1MODE);
}

/**
  * @brief  Check if Own Address1 acknowledge is enabled or disabled.
  * @rmtoll
  *  OAR1         OA1EN         LL_I2C_IsEnabledOwnAddress1
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledOwnAddress1(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->OAR1, I2C_OAR1_OA1EN) == (I2C_OAR1_OA1EN)) ? 1UL : 0UL);
}


/**
  * @brief  Configure Own Address1 and enable it.
  * @rmtoll
  *  OAR1         OA1           LL_I2C_ConfigOwnAddress1 \n
  *  OAR1         OA1MODE       LL_I2C_ConfigOwnAddress1 \n
  *  OAR1         OA1EN         LL_I2C_ConfigOwnAddress1
  * @param  p_i2c I2C instance.
  * @param  own_address1 This parameter must be a value between Min_Data=0 and Max_Data=0x3FF.
  * @param  own_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_OWNADDRESS1_7BIT
  *         @arg @ref LL_I2C_OWNADDRESS1_10BIT
  */
__STATIC_INLINE void LL_I2C_ConfigOwnAddress1(I2C_TypeDef *p_i2c, uint32_t own_address1, uint32_t own_addr_size)
{
  STM32_WRITE_REG(p_i2c->OAR1, I2C_OAR1_OA1EN | own_address1 | own_addr_size);
}


/**
  * @brief  Set the 7-bit Own Address2.
  * @param  p_i2c I2C instance.
  * @param  own_address2 Value between Min_Data=0 and Max_Data=0x7F.
  * @param  own_addr_mask This parameter can be one of the following values:
  *         @arg @ref LL_I2C_OWNADDRESS2_NOMASK
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK01
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK02
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK03
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK04
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK05
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK06
  *         @arg @ref LL_I2C_OWNADDRESS2_MASK07
  * @note   This action has no effect if own address2 is enabled.
  * @rmtoll
  *  OAR2         OA2           LL_I2C_SetOwnAddress2 \n
  *  OAR2         OA2MSK        LL_I2C_SetOwnAddress2
  */
__STATIC_INLINE void LL_I2C_SetOwnAddress2(I2C_TypeDef *p_i2c, uint32_t own_address2, uint32_t own_addr_mask)
{
  STM32_MODIFY_REG(p_i2c->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK, own_address2 | own_addr_mask);
}

/**
  * @brief  Get the Own Address2.
  * @rmtoll
  *  OAR2         OA2           LL_I2C_GetOwnAddress2
  * @param  p_i2c I2C instance.
  * @retval Own Address2 value
  */
__STATIC_INLINE uint32_t LL_I2C_GetOwnAddress2(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->OAR2, I2C_OAR2_OA2) & 0xFFFFFFFEUL);
}

/**
  * @brief  Get the Own Address2 mask.
  * @rmtoll
  *  OAR2         OA2MSK          LL_I2C_GetOwnAddress2Mask
  * @param  p_i2c I2C instance.
  * @retval Own Address2 mask
  */
__STATIC_INLINE uint32_t LL_I2C_GetOwnAddress2Mask(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->OAR2, I2C_OAR2_OA2MSK));
}


/**
  * @brief  Enable acknowledge on Own Address2 match address.
  * @rmtoll
  *  OAR2         OA2EN         LL_I2C_EnableOwnAddress2
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableOwnAddress2(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->OAR2, I2C_OAR2_OA2EN);
}

/**
  * @brief  Disable  acknowledge on Own Address2 match address.
  * @rmtoll
  *  OAR2         OA2EN         LL_I2C_DisableOwnAddress2
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableOwnAddress2(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->OAR2, I2C_OAR2_OA2EN);
}

/**
  * @brief  Check if Own Address1 acknowledge is enabled or disabled.
  * @rmtoll
  *  OAR2         OA2EN         LL_I2C_IsEnabledOwnAddress2
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledOwnAddress2(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->OAR2, I2C_OAR2_OA2EN) == (I2C_OAR2_OA2EN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the SDA setup, hold time and the SCL high, low period.
  * @param  p_i2c I2C instance.
  * @param  timing This parameter must be a value between Min_Data=0 and Max_Data=0xF0FFFFFFU.
  *         This parameter is computed with the STM32CubeMX Tool. Bit 24 to 27 are reserved.
  * @note   This bit can only be programmed when the I2C is disabled (PE = 0).
  * @rmtoll
  *  TIMINGR      TIMINGR       LL_I2C_SetTiming
  */
__STATIC_INLINE void LL_I2C_SetTiming(I2C_TypeDef *p_i2c, uint32_t timing)
{
  STM32_WRITE_REG(p_i2c->TIMINGR, timing & (I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH \
                                            | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC));
}


/**
  * @brief  Get the SDA setup, hold time and the SCL high, low period.
  * @rmtoll
  *  TIMINGR      TIMINGR       LL_I2C_GetTiming
  * @param  p_i2c I2C instance.
  * @note   This parameter can be computed with the STM32CubeMX Tool.
  * @retval Value between Min_Data=0 and Max_Data=0xFFFFFFFF.
  */
__STATIC_INLINE uint32_t LL_I2C_GetTiming(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_REG(p_i2c->TIMINGR));
}

/**
  * @brief  Get the timing Prescaler setting.
  * @rmtoll
  *  TIMINGR      PRESC         LL_I2C_GetTimingPrescaler
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t LL_I2C_GetTimingPrescaler(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMINGR, I2C_TIMINGR_PRESC) >> I2C_TIMINGR_PRESC_Pos);
}

/**
  * @brief  Get the SCL low period setting.
  * @rmtoll
  *  TIMINGR      SCLL          LL_I2C_GetClockLowPeriod
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetClockLowPeriod(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMINGR, I2C_TIMINGR_SCLL) >> I2C_TIMINGR_SCLL_Pos);
}

/**
  * @brief  Get the SCL high period setting.
  * @rmtoll
  *  TIMINGR      SCLH          LL_I2C_GetClockHighPeriod
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetClockHighPeriod(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMINGR, I2C_TIMINGR_SCLH) >> I2C_TIMINGR_SCLH_Pos);
}

/**
  * @brief  Get the SDA hold time.
  * @rmtoll
  *  TIMINGR      SDADEL        LL_I2C_GetDataHoldTime
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t LL_I2C_GetDataHoldTime(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMINGR, I2C_TIMINGR_SDADEL) >> I2C_TIMINGR_SDADEL_Pos);
}

/**
  * @brief  Get the SDA setup time.
  * @rmtoll
  *  TIMINGR      SCLDEL        LL_I2C_GetDataSetupTime
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t LL_I2C_GetDataSetupTime(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMINGR, I2C_TIMINGR_SCLDEL) >> I2C_TIMINGR_SCLDEL_Pos);
}

/**
  * @brief  Configure peripheral mode.
  * @param  p_i2c I2C instance.
  * @param  peripheral_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_MODE_I2C
  *         @arg @ref LL_I2C_MODE_SMBUS_HOST
  *         @arg @ref LL_I2C_MODE_SMBUS_SLAVE
  *         @arg @ref LL_I2C_MODE_SMBUS_SLAVE_ARP
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          SMBHEN        LL_I2C_SetMode \n
  *  CR1          SMBDEN        LL_I2C_SetMode
  */
__STATIC_INLINE void LL_I2C_SetMode(I2C_TypeDef *p_i2c, uint32_t peripheral_mode)
{
  STM32_MODIFY_REG(p_i2c->CR1, I2C_CR1_SMBHEN | I2C_CR1_SMBDEN, peripheral_mode);
}

/**
  * @brief  Get peripheral mode.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          SMBHEN        LL_I2C_GetMode \n
  *  CR1          SMBDEN        LL_I2C_GetMode
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_MODE_I2C
  *         @arg @ref LL_I2C_MODE_SMBUS_HOST
  *         @arg @ref LL_I2C_MODE_SMBUS_SLAVE
  *         @arg @ref LL_I2C_MODE_SMBUS_SLAVE_ARP
  */
__STATIC_INLINE uint32_t LL_I2C_GetMode(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR1, I2C_CR1_SMBHEN | I2C_CR1_SMBDEN));
}

/**
  * @brief  Enable SMBus alert (Host or Device mode).
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   SMBus Device mode:
  *         - SMBus Alert pin is drived low and
  *           Alert Response Address Header acknowledge is enabled.
  *         SMBus Host mode:
  *         - SMBus Alert pin management is supported.
  * @rmtoll
  *  CR1          ALERTEN       LL_I2C_EnableSMBusAlert
  */
__STATIC_INLINE void LL_I2C_EnableSMBusAlert(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_ALERTEN);
}

/**
  * @brief  Disable SMBus alert (Host or Device mode).
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   SMBus Device mode:
  *         - SMBus Alert pin is not drived (can be used as a standard GPIO) and
  *           Alert Response Address Header acknowledge is disabled.
  *         SMBus Host mode:
  *         - SMBus Alert pin management is not supported.
  * @rmtoll
  *  CR1          ALERTEN       LL_I2C_DisableSMBusAlert
  */
__STATIC_INLINE void LL_I2C_DisableSMBusAlert(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_ALERTEN);
}

/**
  * @brief  Check if SMBus alert (Host or Device mode) is enabled or disabled.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          ALERTEN       LL_I2C_IsEnabledSMBusAlert
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledSMBusAlert(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_ALERTEN) == (I2C_CR1_ALERTEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SMBus Packet Error Calculation (PEC).
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          PECEN         LL_I2C_EnableSMBusPEC
  */
__STATIC_INLINE void LL_I2C_EnableSMBusPEC(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_PECEN);
}

/**
  * @brief  Disable SMBus Packet Error Calculation (PEC).
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          PECEN         LL_I2C_DisableSMBusPEC
  */
__STATIC_INLINE void LL_I2C_DisableSMBusPEC(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_PECEN);
}

/**
  * @brief  Check if SMBus Packet Error Calculation (PEC) is enabled or disabled.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR1          PECEN         LL_I2C_IsEnabledSMBusPEC
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledSMBusPEC(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_PECEN) == (I2C_CR1_PECEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the SMBus Clock Timeout.
  * @param  p_i2c I2C instance.
  * @param  timeout_a This parameter must be a value between  Min_Data=0 and Max_Data=0xFFF.
  * @param  timeout_a_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SCL_LOW
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH
  * @param  timeout_b
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   This configuration can only be programmed when associated Timeout is disabled (timeout_a and/or timeout_b).
  * @rmtoll
  *  TIMEOUTR     TIMEOUTA      LL_I2C_ConfigSMBusTimeout \n
  *  TIMEOUTR     TIDLE         LL_I2C_ConfigSMBusTimeout \n
  *  TIMEOUTR     TIMEOUTB      LL_I2C_ConfigSMBusTimeout
  */
__STATIC_INLINE void LL_I2C_ConfigSMBusTimeout(I2C_TypeDef *p_i2c, uint32_t timeout_a, uint32_t timeout_a_mode,
                                               uint32_t timeout_b)
{
  STM32_MODIFY_REG(p_i2c->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMEOUTB,
                   timeout_a | timeout_a_mode | (timeout_b << I2C_TIMEOUTR_TIMEOUTB_Pos));
}

/**
  * @brief  Configure the SMBus Clock timeout_a (SCL low timeout or SCL and SDA high timeout depends on timeout_a mode).
  * @param  p_i2c I2C instance.
  * @param  timeout_a This parameter must be a value between  Min_Data=0 and Max_Data=0xFFF.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   These bits can only be programmed when timeout_a is disabled.
  * @rmtoll
  *  TIMEOUTR     TIMEOUTA      LL_I2C_SetSMBusTimeoutA
  */
__STATIC_INLINE void LL_I2C_SetSMBusTimeoutA(I2C_TypeDef *p_i2c, uint32_t timeout_a)
{
  STM32_WRITE_REG(p_i2c->TIMEOUTR, timeout_a);
}

/**
  * @brief  Get the SMBus Clock timeout_a setting.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIMEOUTA      LL_I2C_GetSMBusTimeoutA
  * @retval Value between Min_Data=0 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetSMBusTimeoutA(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA));
}

/**
  * @brief  Set the SMBus Clock timeout_a mode.
  * @param  p_i2c I2C instance.
  * @param  timeout_a_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SCL_LOW
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   This bit can only be programmed when timeout_a is disabled.
  * @rmtoll
  *  TIMEOUTR     TIDLE         LL_I2C_SetSMBusTimeoutAMode
  */
__STATIC_INLINE void LL_I2C_SetSMBusTimeoutAMode(I2C_TypeDef *p_i2c, uint32_t timeout_a_mode)
{
  STM32_WRITE_REG(p_i2c->TIMEOUTR, timeout_a_mode);
}

/**
  * @brief  Get the SMBus Clock timeout_a mode.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIDLE         LL_I2C_GetSMBusTimeoutAMode
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SCL_LOW
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH
  */
__STATIC_INLINE uint32_t LL_I2C_GetSMBusTimeoutAMode(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMEOUTR, I2C_TIMEOUTR_TIDLE));
}

/**
  * @brief  Configure the SMBus Extended Cumulative Clock timeout_b (Master or Slave mode).
  * @param  p_i2c I2C instance.
  * @param  timeout_b This parameter must be a value between  Min_Data=0 and Max_Data=0xFFF.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   These bits can only be programmed when timeout_b is disabled.
  * @rmtoll
  *  TIMEOUTR     TIMEOUTB      LL_I2C_SetSMBusTimeoutB
  */
__STATIC_INLINE void LL_I2C_SetSMBusTimeoutB(I2C_TypeDef *p_i2c, uint32_t timeout_b)
{
  STM32_WRITE_REG(p_i2c->TIMEOUTR, timeout_b << I2C_TIMEOUTR_TIMEOUTB_Pos);
}

/**
  * @brief  Get the SMBus Extended Cumulative Clock timeout_b setting.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIMEOUTB      LL_I2C_GetSMBusTimeoutB
  * @retval Value between Min_Data=0 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetSMBusTimeoutB(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTB) >> I2C_TIMEOUTR_TIMEOUTB_Pos);
}

/**
  * @brief  Enable the SMBus Clock Timeout.
  * @param  p_i2c I2C instance.
  * @param  clock_timeout This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTB
  *         @arg @ref LL_I2C_SMBUS_ALL_TIMEOUT
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIMOUTEN      LL_I2C_EnableSMBusTimeout \n
  *  TIMEOUTR     TEXTEN        LL_I2C_EnableSMBusTimeout
  */
__STATIC_INLINE void LL_I2C_EnableSMBusTimeout(I2C_TypeDef *p_i2c, uint32_t clock_timeout)
{
  STM32_SET_BIT(p_i2c->TIMEOUTR, clock_timeout);
}

/**
  * @brief  Disable the SMBus Clock Timeout.
  * @param  p_i2c I2C instance.
  * @param  clock_timeout This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTB
  *         @arg @ref LL_I2C_SMBUS_ALL_TIMEOUT
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIMOUTEN      LL_I2C_DisableSMBusTimeout \n
  *  TIMEOUTR     TEXTEN        LL_I2C_DisableSMBusTimeout
  */
__STATIC_INLINE void LL_I2C_DisableSMBusTimeout(I2C_TypeDef *p_i2c, uint32_t clock_timeout)
{
  STM32_CLEAR_BIT(p_i2c->TIMEOUTR, clock_timeout);
}

/**
  * @brief  Check if the SMBus Clock Timeout is enabled or disabled.
  * @param  p_i2c I2C instance.
  * @param  clock_timeout This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTA
  *         @arg @ref LL_I2C_SMBUS_TIMEOUTB
  *         @arg @ref LL_I2C_SMBUS_ALL_TIMEOUT
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  TIMEOUTR     TIMOUTEN      LL_I2C_IsEnabledSMBusTimeout \n
  *  TIMEOUTR     TEXTEN        LL_I2C_IsEnabledSMBusTimeout
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledSMBusTimeout(const I2C_TypeDef *p_i2c, uint32_t clock_timeout)
{
  return ((STM32_READ_BIT(p_i2c->TIMEOUTR, (I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TEXTEN)) == \
           (clock_timeout)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup I2C_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable TXIS interrupt.
  * @rmtoll
  *  CR1          TXIE          LL_I2C_EnableIT_TX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableIT_TX(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_TXIE);
}

/**
  * @brief  Disable TXIS interrupt.
  * @rmtoll
  *  CR1          TXIE          LL_I2C_DisableIT_TX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableIT_TX(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_TXIE);
}

/**
  * @brief  Check if the TXIS Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TXIE          LL_I2C_IsEnabledIT_TX
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_TX(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_TXIE) == (I2C_CR1_TXIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable RXNE interrupt.
  * @rmtoll
  *  CR1          RXIE          LL_I2C_EnableIT_RX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableIT_RX(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_RXIE);
}

/**
  * @brief  Disable RXNE interrupt.
  * @rmtoll
  *  CR1          RXIE          LL_I2C_DisableIT_RX
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableIT_RX(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_RXIE);
}

/**
  * @brief  Check if the RXNE Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          RXIE          LL_I2C_IsEnabledIT_RX
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_RX(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_RXIE) == (I2C_CR1_RXIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Address match interrupt (slave mode only).
  * @rmtoll
  *  CR1          ADDRIE        LL_I2C_EnableIT_ADDR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableIT_ADDR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_ADDRIE);
}

/**
  * @brief  Disable Address match interrupt (slave mode only).
  * @rmtoll
  *  CR1          ADDRIE        LL_I2C_DisableIT_ADDR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableIT_ADDR(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_ADDRIE);
}

/**
  * @brief  Check if Address match interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          ADDRIE        LL_I2C_IsEnabledIT_ADDR
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_ADDR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_ADDRIE) == (I2C_CR1_ADDRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Not acknowledge received interrupt.
  * @rmtoll
  *  CR1          NACKIE        LL_I2C_EnableIT_NACK
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableIT_NACK(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_NACKIE);
}

/**
  * @brief  Disable Not acknowledge received interrupt.
  * @rmtoll
  *  CR1          NACKIE        LL_I2C_DisableIT_NACK
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableIT_NACK(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_NACKIE);
}

/**
  * @brief  Check if Not acknowledge received interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          NACKIE        LL_I2C_IsEnabledIT_NACK
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_NACK(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_NACKIE) == (I2C_CR1_NACKIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable STOP detection interrupt.
  * @rmtoll
  *  CR1          STOPIE        LL_I2C_EnableIT_STOP
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_EnableIT_STOP(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_STOPIE);
}

/**
  * @brief  Disable STOP detection interrupt.
  * @rmtoll
  *  CR1          STOPIE        LL_I2C_DisableIT_STOP
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_DisableIT_STOP(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_STOPIE);
}

/**
  * @brief  Check if STOP detection interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          STOPIE        LL_I2C_IsEnabledIT_STOP
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_STOP(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_STOPIE) == (I2C_CR1_STOPIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Transfer Complete interrupt.
  * @param  p_i2c I2C instance.
  * @note   Any of these events generates interrupt :
  *         Transfer Complete (TC)
  *         Transfer Complete Reload (TCR)
  * @rmtoll
  *  CR1          TCIE          LL_I2C_EnableIT_TC
  */
__STATIC_INLINE void LL_I2C_EnableIT_TC(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_TCIE);
}

/**
  * @brief  Disable Transfer Complete interrupt.
  * @param  p_i2c I2C instance.
  * @note   Any of these events generates interrupt :
  *         Transfer Complete (TC)
  *         Transfer Complete Reload (TCR)
  * @rmtoll
  *  CR1          TCIE          LL_I2C_DisableIT_TC
  */
__STATIC_INLINE void LL_I2C_DisableIT_TC(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_TCIE);
}

/**
  * @brief  Check if Transfer Complete interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TCIE          LL_I2C_IsEnabledIT_TC
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_TC(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_TCIE) == (I2C_CR1_TCIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Error interrupts.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   Any of these errors generates interrupt :
  *         Arbitration Loss (ARLO)
  *         Bus Error detection (BERR)
  *         Overrun/Underrun (OVR)
  *         SMBus Timeout detection (TIMEOUT)
  *         SMBus PEC error detection (PECERR)
  *         SMBus Alert pin event detection (ALERT)
  * @rmtoll
  *  CR1          ERRIE         LL_I2C_EnableIT_ERR
  */
__STATIC_INLINE void LL_I2C_EnableIT_ERR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR1, I2C_CR1_ERRIE);
}

/**
  * @brief  Disable Error interrupts.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   Any of these errors generates interrupt :
  *         Arbitration Loss (ARLO)
  *         Bus Error detection (BERR)
  *         Overrun/Underrun (OVR)
  *         SMBus Timeout detection (TIMEOUT)
  *         SMBus PEC error detection (PECERR)
  *         SMBus Alert pin event detection (ALERT)
  * @rmtoll
  *  CR1          ERRIE         LL_I2C_DisableIT_ERR
  */
__STATIC_INLINE void LL_I2C_DisableIT_ERR(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR1, I2C_CR1_ERRIE);
}

/**
  * @brief  Check if Error interrupts are enabled or disabled.
  * @rmtoll
  *  CR1          ERRIE         LL_I2C_IsEnabledIT_ERR
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT_ERR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR1, I2C_CR1_ERRIE) == (I2C_CR1_ERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the specified I2C interrupts.
  * @rmtoll
  *  CR1  TXIE    LL_I2C_EnableIT \n
  *  CR1  RXIE    LL_I2C_EnableIT \n
  *  CR1  ADDRIE  LL_I2C_EnableIT \n
  *  CR1  NACKIE  LL_I2C_EnableIT \n
  *  CR1  STOPIE  LL_I2C_EnableIT \n
  *  CR1  TCIE    LL_I2C_EnableIT \n
  *  CR1  ERRIE   LL_I2C_EnableIT
  * @param  p_i2c I2C instance.
  * @param  mask Interrupt sources to enable.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I2C_CR1_TXIE    I2C_CR1_TXIE    TX Interrupt disa
  *            @arg @ref LL_I2C_CR1_RXIE    I2C_CR1_RXIE    RX Interrupt enable
  *            @arg @ref LL_I2C_CR1_ADDRIE  I2C_CR1_ADDRIE  Address match Interrupt enable (slave only)
  *            @arg @ref LL_I2C_CR1_NACKIE  I2C_CR1_NACKIE  Not acknowledge received Interrupt enable
  *            @arg @ref LL_I2C_CR1_STOPIE  I2C_CR1_STOPIE  STOP detection Interrupt enable
  *            @arg @ref LL_I2C_CR1_TCIE    I2C_CR1_TCIE    Transfer Complete interrupt enable
  *            @arg @ref LL_I2C_CR1_ERRIE   I2C_CR1_ERRIE   Error interrupts enable
  */
__STATIC_INLINE void LL_I2C_EnableIT(I2C_TypeDef *p_i2c, uint32_t mask)
{
  STM32_SET_BIT(p_i2c->CR1, mask);
}

/**
  * @brief  Disable the specified I2C interrupts.
  * @rmtoll
  *  CR1  TXIE    LL_I2C_DisableIT \n
  *  CR1  RXIE    LL_I2C_DisableIT \n
  *  CR1  ADDRIE  LL_I2C_DisableIT \n
  *  CR1  NACKIE  LL_I2C_DisableIT \n
  *  CR1  STOPIE  LL_I2C_DisableIT \n
  *  CR1  TCIE    LL_I2C_DisableIT \n
  *  CR1  ERRIE   LL_I2C_DisableIT
  * @param  p_i2c I2C instance.
  * @param  mask Interrupt sources to disable.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I2C_CR1_TXIE    I2C_CR1_TXIE    TX Interrupt enable
  *            @arg @ref LL_I2C_CR1_RXIE    I2C_CR1_RXIE    RX Interrupt enable
  *            @arg @ref LL_I2C_CR1_ADDRIE  I2C_CR1_ADDRIE  Address match Interrupt enable (slave only)
  *            @arg @ref LL_I2C_CR1_NACKIE  I2C_CR1_NACKIE  Not acknowledge received Interrupt enable
  *            @arg @ref LL_I2C_CR1_STOPIE  I2C_CR1_STOPIE  STOP detection Interrupt enable
  *            @arg @ref LL_I2C_CR1_TCIE    I2C_CR1_TCIE    Transfer Complete interrupt enable
  *            @arg @ref LL_I2C_CR1_ERRIE   I2C_CR1_ERRIE   Error interrupts enable
  */
__STATIC_INLINE void LL_I2C_DisableIT(I2C_TypeDef *p_i2c, uint32_t mask)
{
  STM32_CLEAR_BIT(p_i2c->CR1, mask);
}

/**
  * @brief  Check whether the specified I2C interrupts sources are enabled or not.
  * @rmtoll
  *  CR1  TXIE    LL_I2C_IsEnabledIT \n
  *  CR1  RXIE    LL_I2C_IsEnabledIT \n
  *  CR1  ADDRIE  LL_I2C_IsEnabledIT \n
  *  CR1  NACKIE  LL_I2C_IsEnabledIT \n
  *  CR1  STOPIE  LL_I2C_IsEnabledIT \n
  *  CR1  TCIE    LL_I2C_IsEnabledIT \n
  *  CR1  ERRIE   LL_I2C_IsEnabledIT
  * @param  p_i2c I2C instance.
  * @param  mask Interrupts sources to check.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I2C_CR1_TXIE    I2C_CR1_TXIE    TX Interrupt enable
  *            @arg @ref LL_I2C_CR1_RXIE    I2C_CR1_RXIE    RX Interrupt enable
  *            @arg @ref LL_I2C_CR1_ADDRIE  I2C_CR1_ADDRIE  Address match Interrupt enable (slave only)
  *            @arg @ref LL_I2C_CR1_NACKIE  I2C_CR1_NACKIE  Not acknowledge received Interrupt enable
  *            @arg @ref LL_I2C_CR1_STOPIE  I2C_CR1_STOPIE  STOP detection Interrupt enable
  *            @arg @ref LL_I2C_CR1_TCIE    I2C_CR1_TCIE    Transfer Complete interrupt enable
  *            @arg @ref LL_I2C_CR1_ERRIE   I2C_CR1_ERRIE   Error interrupts enable
  * @retval State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledIT(const I2C_TypeDef *p_i2c, uint32_t mask)
{
  return ((STM32_READ_BIT(p_i2c->CR1, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup I2C_LL_EF_FLAG_management FLAG_management
  * @{
  */

/**
  * @brief  Indicate the status of a mask of flags.
  * @rmtoll
  *  ISR    I2C_ISR_TXE      LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_TXIS     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_RXNE     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_ADDR     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_NACKF    LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_STOPF    LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_TC       LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_TCR      LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_BERR     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_ARLO     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_OVR      LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_PECERR   LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_TIMEOUT  LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_ALERT    LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_BUSY     LL_I2C_IsActiveFlag  \n
  *  ISR    I2C_ISR_DIR      LL_I2C_IsActiveFlag
  * @param  p_i2c I2C instance.
  * @param  mask Interrupts sources to check.
  *         This parameter can be a combination of the following values:
  *            @arg @ref  LL_I2C_ISR_TXE      I2C_ISR_TXE      Transmit data register empty
  *            @arg @ref  LL_I2C_ISR_TXIS     I2C_ISR_TXIS     Transmit interrupt status
  *            @arg @ref  LL_I2C_ISR_RXNE     I2C_ISR_RXNE     Receive data register not empty
  *            @arg @ref  LL_I2C_ISR_ADDR     I2C_ISR_ADDR     Address matched (slave mode)
  *            @arg @ref  LL_I2C_ISR_NACKF    I2C_ISR_NACKF    Not Acknowledge received flag
  *            @arg @ref  LL_I2C_ISR_STOPF    I2C_ISR_STOPF    Stop detection flag
  *            @arg @ref  LL_I2C_ISR_TC       I2C_ISR_TC       Transfer Complete (master mode)
  *            @arg @ref  LL_I2C_ISR_TCR      I2C_ISR_TCR      Transfer Complete Reload
  *            @arg @ref  LL_I2C_ISR_BERR     I2C_ISR_BERR     Bus error
  *            @arg @ref  LL_I2C_ISR_ARLO     I2C_ISR_ARLO     Arbitration lost
  *            @arg @ref  LL_I2C_ISR_OVR      I2C_ISR_OVR      Overrun/Underrun (slave mode)
  *            @arg @ref  LL_I2C_ISR_PECERR   I2C_ISR_PECERR   PEC Error in reception (SMBus mode)
  *            @arg @ref  LL_I2C_ISR_TIMEOUT  I2C_ISR_TIMEOUT  Timeout detection flag (SMBus mode)
  *            @arg @ref  LL_I2C_ISR_ALERT    I2C_ISR_ALERT    SMBus alert (SMBus mode)
  *            @arg @ref  LL_I2C_ISR_BUSY     I2C_ISR_BUSY     Bus busy
  *            @arg @ref  LL_I2C_ISR_DIR      I2C_ISR_DIR      Direction
  * @retval State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag(const I2C_TypeDef *p_i2c, uint32_t mask)
{
  return ((STM32_READ_BIT(p_i2c->ISR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Transmit data register empty flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: When next data is written in Transmit data register.
  *         SET: When Transmit data register is empty.
  * @rmtoll
  *  ISR          TXE           LL_I2C_IsActiveFlag_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_TXE(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_TXE) == (I2C_ISR_TXE)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Transmit interrupt flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: When next data is written in Transmit data register.
  *         SET: When Transmit data register is empty.
  * @rmtoll
  *  ISR          TXIS          LL_I2C_IsActiveFlag_TXIS
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_TXIS(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_TXIS) == (I2C_ISR_TXIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Receive data register not empty flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: When Receive data register is read.
  *         SET: When the received data is copied in Receive data register.
  * @rmtoll
  *  ISR          RXNE          LL_I2C_IsActiveFlag_RXNE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_RXNE(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_RXNE) == (I2C_ISR_RXNE)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Address matched flag (slave mode).
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When the received slave address matched with one of the enabled slave address.
  * @rmtoll
  *  ISR          ADDR          LL_I2C_IsActiveFlag_ADDR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_ADDR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_ADDR) == (I2C_ISR_ADDR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Not Acknowledge received flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When a NACK is received after a byte transmission.
  * @rmtoll
  *  ISR          NACKF         LL_I2C_IsActiveFlag_NACK
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_NACK(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_NACKF) == (I2C_ISR_NACKF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Stop detection flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When a Stop condition is detected.
  * @rmtoll
  *  ISR          STOPF         LL_I2C_IsActiveFlag_STOP
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_STOP(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_STOPF) == (I2C_ISR_STOPF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Transfer complete flag (master mode).
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When RELOAD=0, AUTOEND=0 and NBYTES date have been transferred.
  * @rmtoll
  *  ISR          TC            LL_I2C_IsActiveFlag_TC
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_TC(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_TC) == (I2C_ISR_TC)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Transfer complete flag (master mode).
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When RELOAD=1 and NBYTES date have been transferred.
  * @rmtoll
  *  ISR          TCR           LL_I2C_IsActiveFlag_TCR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_TCR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_TCR) == (I2C_ISR_TCR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Bus error flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When a misplaced Start or Stop condition is detected.
  * @rmtoll
  *  ISR          BERR          LL_I2C_IsActiveFlag_BERR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_BERR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_BERR) == (I2C_ISR_BERR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Arbitration lost flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When arbitration lost.
  * @rmtoll
  *  ISR          ARLO          LL_I2C_IsActiveFlag_ARLO
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_ARLO(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_ARLO) == (I2C_ISR_ARLO)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Overrun/Underrun flag (slave mode).
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When an overrun/underrun error occurs (Clock Stretching Disabled).
  * @rmtoll
  *  ISR          OVR           LL_I2C_IsActiveFlag_OVR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_OVR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_OVR) == (I2C_ISR_OVR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of SMBus PEC error flag in reception.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   RESET: Clear default value.
  *         SET: When the received PEC does not match with the PEC register content.
  * @rmtoll
  *  ISR          PECERR        LL_I2C_IsActiveSMBusFlag_PECERR
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveSMBusFlag_PECERR(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_PECERR) == (I2C_ISR_PECERR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of SMBus Timeout detection flag.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   RESET: Clear default value.
  *         SET: When a timeout or extended clock timeout occurs.
  * @rmtoll
  *  ISR          TIMEOUT       LL_I2C_IsActiveSMBusFlag_TIMEOUT
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveSMBusFlag_TIMEOUT(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_TIMEOUT) == (I2C_ISR_TIMEOUT)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of SMBus alert flag.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   RESET: Clear default value.
  *         SET: When SMBus host configuration, SMBus alert enabled and
  *              a falling edge event occurs on SMBA pin.
  * @rmtoll
  *  ISR          ALERT         LL_I2C_IsActiveSMBusFlag_ALERT
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveSMBusFlag_ALERT(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_ALERT) == (I2C_ISR_ALERT)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate the status of Bus Busy flag.
  * @param  p_i2c I2C instance.
  * @note   RESET: Clear default value.
  *         SET: When a Start condition is detected.
  * @rmtoll
  *  ISR          BUSY          LL_I2C_IsActiveFlag_BUSY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_BUSY(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->ISR, I2C_ISR_BUSY) == (I2C_ISR_BUSY)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Address Matched flag.
  * @rmtoll
  *  ICR          ADDRCF        LL_I2C_ClearFlag_ADDR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_ADDR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_ADDRCF);
}

/**
  * @brief  Clear Not Acknowledge flag.
  * @rmtoll
  *  ICR          NACKCF        LL_I2C_ClearFlag_NACK
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_NACK(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_NACKCF);
}

/**
  * @brief  Clear Stop detection flag.
  * @rmtoll
  *  ICR          STOPCF        LL_I2C_ClearFlag_STOP
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_STOP(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_STOPCF);
}

/**
  * @brief  Clear Transmit data register empty flag (TXE).
  * @param  p_i2c I2C instance.
  * @note   This bit can be clear by software in order to flush the transmit data register (TXDR).
  * @rmtoll
  *  ISR          TXE           LL_I2C_ClearFlag_TXE
  */
__STATIC_INLINE void LL_I2C_ClearFlag_TXE(I2C_TypeDef *p_i2c)
{
  STM32_WRITE_REG(p_i2c->ISR, I2C_ISR_TXE);
}

/**
  * @brief  Clear Bus error flag.
  * @rmtoll
  *  ICR          BERRCF        LL_I2C_ClearFlag_BERR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_BERR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_BERRCF);
}

/**
  * @brief  Clear Arbitration lost flag.
  * @rmtoll
  *  ICR          ARLOCF        LL_I2C_ClearFlag_ARLO
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_ARLO(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_ARLOCF);
}

/**
  * @brief  Clear Overrun/Underrun flag.
  * @rmtoll
  *  ICR          OVRCF         LL_I2C_ClearFlag_OVR
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_ClearFlag_OVR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_OVRCF);
}

/**
  * @brief  Clear SMBus PEC error flag.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  ICR          PECCF         LL_I2C_ClearSMBusFlag_PECERR
  */
__STATIC_INLINE void LL_I2C_ClearSMBusFlag_PECERR(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_PECCF);
}

/**
  * @brief  Clear SMBus Timeout detection flag.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  ICR          TIMOUTCF      LL_I2C_ClearSMBusFlag_TIMEOUT
  */
__STATIC_INLINE void LL_I2C_ClearSMBusFlag_TIMEOUT(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_TIMOUTCF);
}

/**
  * @brief  Clear SMBus Alert flag.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  ICR          ALERTCF       LL_I2C_ClearSMBusFlag_ALERT
  */
__STATIC_INLINE void LL_I2C_ClearSMBusFlag_ALERT(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->ICR, I2C_ICR_ALERTCF);
}

/**
  * @}
  */

/** @defgroup I2C_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Enable automatic STOP condition generation (master mode).
  * @param  p_i2c I2C instance.
  * @note   Automatic end mode : a STOP condition is automatically sent when NBYTES data are transferred.
  *         This bit has no effect in slave mode or when RELOAD bit is set.
  * @rmtoll
  *  CR2          AUTOEND       LL_I2C_EnableAutoEndMode
  */
__STATIC_INLINE void LL_I2C_EnableAutoEndMode(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_AUTOEND);
}

/**
  * @brief  Disable automatic STOP condition generation (master mode).
  * @param  p_i2c I2C instance.
  * @note   Software end mode : TC flag is set when NBYTES data are transferre, stretching SCL low.
  * @rmtoll
  *  CR2          AUTOEND       LL_I2C_DisableAutoEndMode
  */
__STATIC_INLINE void LL_I2C_DisableAutoEndMode(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR2, I2C_CR2_AUTOEND);
}

/**
  * @brief  Check if automatic STOP condition is enabled or disabled.
  * @rmtoll
  *  CR2          AUTOEND       LL_I2C_IsEnabledAutoEndMode
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledAutoEndMode(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR2, I2C_CR2_AUTOEND) == (I2C_CR2_AUTOEND)) ? 1UL : 0UL);
}

/**
  * @brief  Enable reload mode (master mode).
  * @param  p_i2c I2C instance.
  * @note   The transfer is not completed after the NBYTES data transfer, NBYTES is reloaded when TCR flag is set.
  * @rmtoll
  *  CR2          RELOAD       LL_I2C_EnableReloadMode
  */
__STATIC_INLINE void LL_I2C_EnableReloadMode(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_RELOAD);
}

/**
  * @brief  Disable reload mode (master mode).
  * @param  p_i2c I2C instance.
  * @note   The transfer is completed after the NBYTES data transfer(STOP or RESTART follows).
  * @rmtoll
  *  CR2          RELOAD       LL_I2C_DisableReloadMode
  */
__STATIC_INLINE void LL_I2C_DisableReloadMode(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR2, I2C_CR2_RELOAD);
}

/**
  * @brief  Check if reload mode is enabled or disabled.
  * @rmtoll
  *  CR2          RELOAD       LL_I2C_IsEnabledReloadMode
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledReloadMode(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR2, I2C_CR2_RELOAD) == (I2C_CR2_RELOAD)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the number of bytes for transfer.
  * @param  p_i2c I2C instance.
  * @param  xfer_size This parameter must be a value between Min_Data=0x00 and Max_Data=0xFF.
  * @note   Changing these bits when START bit is set is not allowed.
  * @rmtoll
  *  CR2          NBYTES           LL_I2C_SetTransferSize
  */
__STATIC_INLINE void LL_I2C_SetTransferSize(I2C_TypeDef *p_i2c, uint32_t xfer_size)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_NBYTES, xfer_size << I2C_CR2_NBYTES_Pos);
}

/**
  * @brief  Get the number of bytes configured for transfer.
  * @rmtoll
  *  CR2          NBYTES           LL_I2C_GetTransferSize
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetTransferSize(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR2, I2C_CR2_NBYTES) >> I2C_CR2_NBYTES_Pos);
}

/**
  * @brief  Prepare the generation of a ACKnowledge or Non ACKnowledge condition after the address receive match code.
            or next received byte.
  * @param  p_i2c I2C instance.
  * @param  type_acknowledge This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ACK
  *         @arg @ref LL_I2C_NACK
  * @note   Usage in Slave mode only.
  * @rmtoll
  *  CR2          NACK          LL_I2C_AcknowledgeNextData
  */
__STATIC_INLINE void LL_I2C_AcknowledgeNextData(I2C_TypeDef *p_i2c, uint32_t type_acknowledge)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_NACK, type_acknowledge);
}

/**
  * @brief  Disable Address Acknowledge.
  * @rmtoll
  *  CR2          NACK          LL_I2C_AcknowledgeNextData
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_AcknowledgeDisable(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_NACK);
}

/**
  * @brief  Enable Address Acknowledge.
  * @rmtoll
  *  CR2          NACK          LL_I2C_AcknowledgeEnable
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_AcknowledgeEnable(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR2, I2C_CR2_NACK);
}

/**
  * @brief  Generate a START or RESTART condition.
  * @param  p_i2c I2C instance.
  * @note   The START bit can be set even if bus is BUSY or I2C is in slave mode.
  *         This action has no effect when RELOAD is set.
  * @rmtoll
  *  CR2          START           LL_I2C_GenerateStartCondition
  */
__STATIC_INLINE void LL_I2C_GenerateStartCondition(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_START);
}

/**
  * @brief  Generate a STOP condition after the current byte transfer (master mode).
  * @rmtoll
  *  CR2          STOP          LL_I2C_GenerateStopCondition
  * @param  p_i2c I2C instance.
  */
__STATIC_INLINE void LL_I2C_GenerateStopCondition(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_STOP);
}

/**
  * @brief  Enable automatic RESTART Read request condition for 10bit address header (master mode).
  * @param  p_i2c I2C instance.
  * @note   The master sends the complete 10bit slave address read sequence :
  *         Start + 2 bytes 10bit address in Write direction + Restart + first 7 bits of 10bit address
            in Read direction.
  * @rmtoll
  *  CR2          HEAD10R       LL_I2C_EnableAuto10BitRead
  */
__STATIC_INLINE void LL_I2C_EnableAuto10BitRead(I2C_TypeDef *p_i2c)
{
  STM32_CLEAR_BIT(p_i2c->CR2, I2C_CR2_HEAD10R);
}

/**
  * @brief  Disable automatic RESTART Read request condition for 10bit address header (master mode).
  * @param  p_i2c I2C instance.
  * @note   The master only sends the first 7 bits of 10bit address in Read direction.
  * @rmtoll
  *  CR2          HEAD10R       LL_I2C_DisableAuto10BitRead
  */
__STATIC_INLINE void LL_I2C_DisableAuto10BitRead(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_HEAD10R);
}

/**
  * @brief  Check if automatic RESTART Read request condition for 10bit address header is enabled or disabled.
  * @rmtoll
  *  CR2          HEAD10R       LL_I2C_IsEnabledAuto10BitRead
  * @param  p_i2c I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledAuto10BitRead(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR2, I2C_CR2_HEAD10R) != (I2C_CR2_HEAD10R)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the transfer direction (master mode).
  * @param  p_i2c I2C instance.
  * @param  xfer_request This parameter can be one of the following values:
  *         @arg @ref LL_I2C_REQUEST_WRITE
  *         @arg @ref LL_I2C_REQUEST_READ
  * @note   Changing these bits when START bit is set is not allowed.
  * @rmtoll
  *  CR2          RD_WRN           LL_I2C_SetTransferRequest
  */
__STATIC_INLINE void LL_I2C_SetTransferRequest(I2C_TypeDef *p_i2c, uint32_t xfer_request)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_RD_WRN, xfer_request);
}

/**
  * @brief  Get the transfer direction requested (master mode).
  * @rmtoll
  *  CR2          RD_WRN           LL_I2C_GetTransferRequest
  * @param  p_i2c I2C instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_REQUEST_WRITE
  *         @arg @ref LL_I2C_REQUEST_READ
  */
__STATIC_INLINE uint32_t LL_I2C_GetTransferRequest(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR2, I2C_CR2_RD_WRN));
}

/**
  * @brief  Configure the slave address for transfer (master mode).
  * @param  p_i2c I2C instance.
  * @param  slave_addr This parameter must be a value between Min_Data=0x00 and Max_Data=0x3F.
  * @note   Changing these bits when START bit is set is not allowed.
  * @rmtoll
  *  CR2          SADD           LL_I2C_SetSlaveAddr
  */
__STATIC_INLINE void LL_I2C_SetSlaveAddr(I2C_TypeDef *p_i2c, uint32_t slave_addr)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_SADD, slave_addr);
}

/**
  * @brief  Get the slave address programmed for transfer.
  * @rmtoll
  *  CR2          SADD           LL_I2C_GetSlaveAddr
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_I2C_GetSlaveAddr(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->CR2, I2C_CR2_SADD));
}

/**
  * @brief  Handles p_i2c communication when starting transfer or during transfer (TC or TCR flag are set).
  * @rmtoll
  *  CR2          SADD          LL_I2C_HandleTransfer \n
  *  CR2          ADD10         LL_I2C_HandleTransfer \n
  *  CR2          RD_WRN        LL_I2C_HandleTransfer \n
  *  CR2          START         LL_I2C_HandleTransfer \n
  *  CR2          STOP          LL_I2C_HandleTransfer \n
  *  CR2          RELOAD        LL_I2C_HandleTransfer \n
  *  CR2          NBYTES        LL_I2C_HandleTransfer \n
  *  CR2          AUTOEND       LL_I2C_HandleTransfer \n
  *  CR2          HEAD10R       LL_I2C_HandleTransfer
  * @param  p_i2c I2C instance.
  * @param  slave_addr Specifies the slave address to be programmed.
  * @param  slave_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRSLAVE_7BIT
  *         @arg @ref LL_I2C_ADDRSLAVE_10BIT
  * @param  xfer_size Specifies the number of bytes to be programmed.
  *                       This parameter must be a value between Min_Data=0 and Max_Data=255.
  * @param  end_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_MODE_RELOAD
  *         @arg @ref LL_I2C_MODE_AUTOEND
  *         @arg @ref LL_I2C_MODE_SOFTEND
  *         @arg @ref LL_I2C_MODE_SMBUS_RELOAD
  *         @arg @ref LL_I2C_MODE_SMBUS_AUTOEND_NO_PEC
  *         @arg @ref LL_I2C_MODE_SMBUS_SOFTEND_NO_PEC
  *         @arg @ref LL_I2C_MODE_SMBUS_AUTOEND_WITH_PEC
  *         @arg @ref LL_I2C_MODE_SMBUS_SOFTEND_WITH_PEC
  * @param  request This parameter can be one of the following values:
  *         @arg @ref LL_I2C_GENERATE_NOSTARTSTOP
  *         @arg @ref LL_I2C_GENERATE_STOP
  *         @arg @ref LL_I2C_GENERATE_START_READ
  *         @arg @ref LL_I2C_GENERATE_START_WRITE
  *         @arg @ref LL_I2C_GENERATE_RESTART_7BIT_READ
  *         @arg @ref LL_I2C_GENERATE_RESTART_7BIT_WRITE
  *         @arg @ref LL_I2C_GENERATE_RESTART_10BIT_READ
  *         @arg @ref LL_I2C_GENERATE_RESTART_10BIT_WRITE
  */
__STATIC_INLINE void LL_I2C_HandleTransfer(I2C_TypeDef *p_i2c, uint32_t slave_addr, uint32_t slave_addr_size,
                                           uint32_t xfer_size, uint32_t end_mode, uint32_t request)
{
  STM32_MODIFY_REG(p_i2c->CR2, I2C_CR2_SADD | I2C_CR2_ADD10 |
                   (I2C_CR2_RD_WRN & (uint32_t)(request >> (31U - I2C_CR2_RD_WRN_Pos))) |
                   I2C_CR2_START | I2C_CR2_STOP | I2C_CR2_RELOAD |
                   I2C_CR2_NBYTES | I2C_CR2_AUTOEND | I2C_CR2_HEAD10R,
                   slave_addr | slave_addr_size | (xfer_size << I2C_CR2_NBYTES_Pos) | end_mode | request);
}

/**
  * @brief  Indicate the value of transfer direction (slave mode).
  * @param  p_i2c I2C instance.
  * @note   RESET: Write transfer, Slave enters in receiver mode.
  *         SET: Read transfer, Slave enters in transmitter mode.
  * @rmtoll
  *  ISR          DIR           LL_I2C_GetTransferDirection
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_DIRECTION_WRITE
  *         @arg @ref LL_I2C_DIRECTION_READ
  */
__STATIC_INLINE uint32_t LL_I2C_GetTransferDirection(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->ISR, I2C_ISR_DIR));
}

/**
  * @brief  Return the slave matched address.
  * @rmtoll
  *  ISR          ADDCODE       LL_I2C_GetAddressMatchCode
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_I2C_GetAddressMatchCode(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->ISR, I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos << 1);
}

/**
  * @brief  Enable internal comparison of the SMBus Packet Error byte (transmission or reception mode).
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @note   This feature is cleared by hardware when the PEC byte is transferred, or when a STOP condition
            or an Address Matched is received.
  *         This bit has no effect when RELOAD bit is set.
  *         This bit has no effect in device mode when SBC bit is not set.
  * @rmtoll
  *  CR2          PECBYTE       LL_I2C_EnableSMBusPECCompare
  */
__STATIC_INLINE void LL_I2C_EnableSMBusPECCompare(I2C_TypeDef *p_i2c)
{
  STM32_SET_BIT(p_i2c->CR2, I2C_CR2_PECBYTE);
}

/**
  * @brief  Check if the SMBus Packet Error byte internal comparison is requested or not.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  CR2          PECBYTE       LL_I2C_IsEnabledSMBusPECCompare
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I2C_IsEnabledSMBusPECCompare(const I2C_TypeDef *p_i2c)
{
  return ((STM32_READ_BIT(p_i2c->CR2, I2C_CR2_PECBYTE) == (I2C_CR2_PECBYTE)) ? 1UL : 0UL);
}

/**
  * @brief  Get the SMBus Packet Error byte calculated.
  * @param  p_i2c I2C instance.
  * @note   The macro IS_SMBUS_ALL_INSTANCE(p_i2c) can be used to check whether or not
  *         SMBus feature is supported by the p_i2c instance.
  * @rmtoll
  *  PECR         PEC           LL_I2C_GetSMBusPEC
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_I2C_GetSMBusPEC(const I2C_TypeDef *p_i2c)
{
  return (uint32_t)(STM32_READ_BIT(p_i2c->PECR, I2C_PECR_PEC));
}

/**
  * @brief  Read Receive data register.
  * @rmtoll
  *  RXDR         RXDATA        LL_I2C_ReceiveData8
  * @param  p_i2c I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t LL_I2C_ReceiveData8(const I2C_TypeDef *p_i2c)
{
  return (uint8_t)(STM32_READ_REG(p_i2c->RXDR));
}

/**
  * @brief  Write in Transmit data Register .
  * @rmtoll
  *  TXDR         TXDATA        LL_I2C_TransmitData8
  * @param  p_i2c I2C instance.
  * @param  data Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_I2C_TransmitData8(I2C_TypeDef *p_i2c, uint8_t data)
{
  STM32_WRITE_REG(p_i2c->TXDR, data);
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

#endif /* I2C1 || I2C2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5xx_LL_I2C_H */
