/**
  ******************************************************************************
  * @file    stm32c5xx_ll_lpuart.h
  * @brief   Header file of LPUART LL module.
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
#ifndef STM32C5XX_LL_LPUART_H
#define STM32C5XX_LL_LPUART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (LPUART1)

/** @defgroup LPUART_LL LPUART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup LPUART_LL_Private_Variables LPUART Private Variables
  * @{
  */
/* Array used to get the LPUART prescaler division decimal values versus @ref LPUART_LL_EC_PRESCALER values */
static const uint16_t LL_LPUART_PRESCALER_TAB[16] =
{
  1U,
  2U,
  4U,
  6U,
  8U,
  10U,
  12U,
  16U,
  32U,
  64U,
  128U,
  256U,
  256U,
  256U,
  256U,
  256U,
};
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup LPUART_LL_Private_Constants LPUART Private Constants
  * @{
  */
/* Defines used in baud rate-related macros and corresponding register setting computation */
#define LL_LPUART_LPUARTDIV_FREQ_MUL  256U
#define LL_LPUART_BRR_MASK            0x000FFFFFU
#define LL_LPUART_BRR_MIN_VALUE       0x00000300U
#define LL_LPUART_TRIG_MASK           0x20000000U
/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup LPUART_LL_Exported_Constants LL LPUART Constants
  * @{
  */

/** @defgroup LPUART_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flag definitions that can be used with the LL_LPUART_WRITE_REG function.
  * @{
  */
#define LL_LPUART_ICR_PECF                 USART_ICR_PECF                /*!< Parity error clear flag */
#define LL_LPUART_ICR_FECF                 USART_ICR_FECF                /*!< Framing error clear flag */
#define LL_LPUART_ICR_NCF                  USART_ICR_NECF                /*!< Noise error detected clear flag */
#define LL_LPUART_ICR_ORECF                USART_ICR_ORECF               /*!< Overrun error clear flag */
#define LL_LPUART_ICR_IDLECF               USART_ICR_IDLECF              /*!< Idle line detected clear flag */
#define LL_LPUART_ICR_TCCF                 USART_ICR_TCCF                /*!< Transmission complete clear flag */
#define LL_LPUART_ICR_CTSCF                USART_ICR_CTSCF               /*!< CTS clear flag */
#define LL_LPUART_ICR_CMCF                 USART_ICR_CMCF                /*!< Character match clear flag */
#define LL_LPUART_ICR_WUCF                 USART_ICR_WUCF                /*!< Wake-up from stop mode clear flag */
#define LL_LPUART_ICR_TXFECF               USART_ICR_TXFECF              /*!< Tx FIFO empty clear flag */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flag definitions that can be used with the LL_LPUART_READ_REG function.
  * @{
  */
#define LL_LPUART_ISR_PE               USART_ISR_PE         /*!< Parity error flag */
#define LL_LPUART_ISR_FE               USART_ISR_FE         /*!< Framing error flag */
#define LL_LPUART_ISR_NE               USART_ISR_NE         /*!< Noise detected flag */
#define LL_LPUART_ISR_ORE              USART_ISR_ORE        /*!< Overrun error flag */
#define LL_LPUART_ISR_IDLE             USART_ISR_IDLE       /*!< Idle line detected flag */
#define LL_LPUART_ISR_RXNE_RXFNE       USART_ISR_RXNE_RXFNE /*!< Read data register or RX FIFO not empty flag */
#define LL_LPUART_ISR_TC               USART_ISR_TC         /*!< Transmission complete flag */
#define LL_LPUART_ISR_TXE_TXFNF        USART_ISR_TXE_TXFNF  /*!< Transmit data register empty or TX FIFO not full flag*/
#define LL_LPUART_ISR_CTSIF            USART_ISR_CTSIF      /*!< CTS interrupt flag */
#define LL_LPUART_ISR_CTS              USART_ISR_CTS        /*!< CTS flag */
#define LL_LPUART_ISR_BUSY             USART_ISR_BUSY       /*!< Busy flag */
#define LL_LPUART_ISR_CMF              USART_ISR_CMF        /*!< Character match flag */
#define LL_LPUART_ISR_SBKF             USART_ISR_SBKF       /*!< Send break flag */
#define LL_LPUART_ISR_RWU              USART_ISR_RWU        /*!< Receiver wake-up from mute mode flag */
#define LL_LPUART_ISR_WUF              USART_ISR_WUF        /*!< Wake-up from stop mode flag */
#define LL_LPUART_ISR_TEACK            USART_ISR_TEACK      /*!< Transmit enable acknowledge flag */
#define LL_LPUART_ISR_REACK            USART_ISR_REACK      /*!< Receive enable acknowledge flag */
#define LL_LPUART_ISR_TXFE             USART_ISR_TXFE       /*!< TX FIFO empty flag */
#define LL_LPUART_ISR_RXFF             USART_ISR_RXFF       /*!< RX FIFO full flag */
#define LL_LPUART_ISR_RXFT             USART_ISR_RXFT       /*!< RX FIFO threshold flag */
#define LL_LPUART_ISR_TXFT             USART_ISR_TXFT       /*!< TX FIFO threshold flag */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_IT IT Defines
  * @brief    IT definitions that can be used with the LL_LPUART_READ_REG and LL_LPUART_WRITE_REG functions.
  * @{
  */
#define LL_LPUART_CR1_IDLEIE         USART_CR1_IDLEIE         /*!< IDLE interrupt enable */
#define LL_LPUART_CR1_RXNEIE_RXFNEIE USART_CR1_RXNEIE_RXFNEIE /*!< Read data register and Rx FIFO not empty
                                                                   interrupt enable */
#define LL_LPUART_CR1_TCIE           USART_CR1_TCIE           /*!< Transmission complete interrupt enable */
#define LL_LPUART_CR1_TXEIE_TXFNFIE  USART_CR1_TXEIE_TXFNFIE  /*!< Transmit data register empty and Tx FIFO
                                                                   not full interrupt enable */
#define LL_LPUART_CR1_PEIE           USART_CR1_PEIE           /*!< Parity error */
#define LL_LPUART_CR1_CMIE           USART_CR1_CMIE           /*!< Character match interrupt enable */
#define LL_LPUART_CR1_TXFEIE         USART_CR1_TXFEIE         /*!< TX FIFO empty interrupt enable */
#define LL_LPUART_CR1_RXFFIE         USART_CR1_RXFFIE         /*!< RX FIFO full interrupt enable */
#define LL_LPUART_CR3_EIE            USART_CR3_EIE            /*!< Error interrupt enable */
#define LL_LPUART_CR3_CTSIE          USART_CR3_CTSIE          /*!< CTS interrupt enable */
#define LL_LPUART_CR3_WUFIE          USART_CR3_WUFIE          /*!< Wake-up from stop mode interrupt enable */
#define LL_LPUART_CR3_TXFTIE         USART_CR3_TXFTIE         /*!< TX FIFO threshold interrupt enable */
#define LL_LPUART_CR3_RXFTIE         USART_CR3_RXFTIE         /*!< RX FIFO threshold interrupt enable */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_FIFOTHRESHOLD FIFO Threshold
  * @{
  */
#define LL_LPUART_FIFO_THRESHOLD_1_8        0x00000000U /*!< FIFO reaches 1/8 of its depth */
#define LL_LPUART_FIFO_THRESHOLD_1_4        0x00000001U /*!< FIFO reaches 1/4 of its depth */
#define LL_LPUART_FIFO_THRESHOLD_1_2        0x00000002U /*!< FIFO reaches 1/2 of its depth */
#define LL_LPUART_FIFO_THRESHOLD_3_4        0x00000003U /*!< FIFO reaches 3/4 of its depth */
#define LL_LPUART_FIFO_THRESHOLD_7_8        0x00000004U /*!< FIFO reaches 7/8 of its depth */
#define LL_LPUART_FIFO_THRESHOLD_8_8        0x00000005U /*!< FIFO becomes empty for TX and full for RX */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_DIRECTION Direction
  * @{
  */
#define LL_LPUART_DIRECTION_NONE  0x00000000U                  /*!< Transmitter and receiver are disabled           */
#define LL_LPUART_DIRECTION_RX    USART_CR1_RE                 /*!< Transmitter is disabled and receiver is enabled */
#define LL_LPUART_DIRECTION_TX    USART_CR1_TE                 /*!< Transmitter is enabled and receiver is disabled */
#define LL_LPUART_DIRECTION_TX_RX (USART_CR1_TE |USART_CR1_RE) /*!< Transmitter and receiver are enabled            */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_PARITY Parity control
  * @{
  */
#define LL_LPUART_PARITY_NONE 0x00000000U                      /*!< Parity control disabled                            */
#define LL_LPUART_PARITY_EVEN USART_CR1_PCE                    /*!< Parity control enabled and even parity is selected */
#define LL_LPUART_PARITY_ODD  (USART_CR1_PCE | USART_CR1_PS)   /*!< Parity control enabled and odd parity is selected  */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_WAKEUP Wake-up
  * @{
  */
#define LL_LPUART_WAKEUP_IDLELINE    0x00000000U               /*!< LPUART wake-up from mute mode on idle line    */
#define LL_LPUART_WAKEUP_ADDRESSMARK USART_CR1_WAKE            /*!< LPUART wake-up from mute mode on address mark */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_DATAWIDTH Data width
  * @{
  */
#define LL_LPUART_DATAWIDTH_7_BIT    USART_CR1_M1              /*!< 7-bit word length: start bit, 7 data bits, n stop bits */
#define LL_LPUART_DATAWIDTH_8_BIT    0x00000000U               /*!< 8-bit word length: start bit, 8 data bits, n stop bits */
#define LL_LPUART_DATAWIDTH_9_BIT    USART_CR1_M0              /*!< 9-bit word length: start bit, 9 data bits, n stop bits */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_PRESCALER Clock source prescaler
  * @{
  */
#define LL_LPUART_PRESCALER_DIV1   0x00000000U                    /*!< Input clock not divided   */
#define LL_LPUART_PRESCALER_DIV2   (USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 2  */
#define LL_LPUART_PRESCALER_DIV4   (USART_PRESC_PRESCALER_1)      /*!< Input clock divided by 4  */
#define LL_LPUART_PRESCALER_DIV6   (USART_PRESC_PRESCALER_1 |\
                                    USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 6  */
#define LL_LPUART_PRESCALER_DIV8   (USART_PRESC_PRESCALER_2)      /*!< Input clock divided by 8  */
#define LL_LPUART_PRESCALER_DIV10  (USART_PRESC_PRESCALER_2 |\
                                    USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 10 */
#define LL_LPUART_PRESCALER_DIV12  (USART_PRESC_PRESCALER_2 |\
                                    USART_PRESC_PRESCALER_1)      /*!< Input clock divided by 12 */
#define LL_LPUART_PRESCALER_DIV16  (USART_PRESC_PRESCALER_2 |\
                                    USART_PRESC_PRESCALER_1 |\
                                    USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 16 */
#define LL_LPUART_PRESCALER_DIV32  (USART_PRESC_PRESCALER_3)      /*!< Input clock divided by 32 */
#define LL_LPUART_PRESCALER_DIV64  (USART_PRESC_PRESCALER_3 |\
                                    USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 64 */
#define LL_LPUART_PRESCALER_DIV128 (USART_PRESC_PRESCALER_3 |\
                                    USART_PRESC_PRESCALER_1)      /*!< Input clock divided by 128 */
#define LL_LPUART_PRESCALER_DIV256 (USART_PRESC_PRESCALER_3 |\
                                    USART_PRESC_PRESCALER_1 |\
                                    USART_PRESC_PRESCALER_0)      /*!< Input clock divided by 256 */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_STOPBITS Stop bits
  * @{
  */
#define LL_LPUART_STOP_BIT_1            0x00000000U        /*!< 1 stop bit */
#define LL_LPUART_STOP_BIT_2            USART_CR2_STOP_1   /*!< 2 stop bits */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_TXRX Tx/Rx pins swap
  * @{
  */
#define LL_LPUART_TXRX_STANDARD         0x00000000U        /*!< Tx/Rx pins are used as defined in standard pinout */
#define LL_LPUART_TXRX_SWAPPED          (USART_CR2_SWAP)   /*!< Tx/Rx pin functions are swapped.                  */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_RXPIN_LEVEL RX Pin Active Level Inversion
  * @{
  */
#define LL_LPUART_RXPIN_LEVEL_STANDARD  0x00000000U        /*!< RX pin signal works using the standard logic levels */
#define LL_LPUART_RXPIN_LEVEL_INVERTED  (USART_CR2_RXINV)  /*!< RX pin signal values are inverted.                  */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_TXPIN_LEVEL TX Pin Active Level Inversion
  * @{
  */
#define LL_LPUART_TXPIN_LEVEL_STANDARD  0x00000000U        /*!< TX pin signal works using the standard logic levels */
#define LL_LPUART_TXPIN_LEVEL_INVERTED  (USART_CR2_TXINV)  /*!< TX pin signal values are inverted.                  */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_BINARY_LOGIC Binary Data Inversion
  * @{
  */
#define LL_LPUART_BINARY_LOGIC_POSITIVE 0x00000000U        /*!< Logical data from the data register are sent/received
                                                                in positive/direct logic. (1=H, 0=L)                  */
#define LL_LPUART_BINARY_LOGIC_NEGATIVE USART_CR2_DATAINV  /*!< Logical data from the data register are sent/received
                                                                in negative/inverse logic. (1=L, 0=H).
                                                                The parity bit is also inverted.                      */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_BITORDER Bit Order
  * @{
  */
#define LL_LPUART_BITORDER_LSBFIRST     0x00000000U        /*!< Data is transmitted/received with data bit 0 first,
                                                                following the start bit */
#define LL_LPUART_BITORDER_MSBFIRST     USART_CR2_MSBFIRST /*!< Data is transmitted/received with the MSB first,
                                                                following the start bit */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_ADDRESS_DETECT Address Length Detection
  * @{
  */
#define LL_LPUART_ADDRESS_DETECT_4_BIT  0x00000000U        /*!< 4-bit address detection method selected */
#define LL_LPUART_ADDRESS_DETECT_7_BIT  USART_CR2_ADDM7    /*!< 7-bit address detection (in 8-bit data mode) method selected */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_HWCONTROL Hardware Control
  * @{
  */
#define LL_LPUART_HWCONTROL_NONE    0x00000000U                       /*!< CTS and RTS hardware flow control disabled */
#define LL_LPUART_HWCONTROL_RTS     USART_CR3_RTSE                    /*!< RTS output enabled, data is only requested
                                                                           when there is space in the receive buffer  */
#define LL_LPUART_HWCONTROL_CTS     USART_CR3_CTSE                    /*!< CTS mode enabled, data is only transmitted
                                                                           when the nCTS input is asserted (tied to 0)*/
#define LL_LPUART_HWCONTROL_RTS_CTS (USART_CR3_RTSE | USART_CR3_CTSE) /*!< CTS and RTS hardware flow control enabled  */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_WAKEUP_ON Wakeup Activation
  * @{
  */
#define LL_LPUART_WAKEUP_ON_ADDRESS   0x00000000U                          /*!< Wake-up active on address match */
#define LL_LPUART_WAKEUP_ON_STARTBIT  USART_CR3_WUS_1                      /*!< Wake-up active on start bit detection */
#define LL_LPUART_WAKEUP_ON_RXNE      (USART_CR3_WUS_0 | USART_CR3_WUS_1)  /*!< Wake-up active on RXNE */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_DE_POLARITY Driver Enable Polarity
  * @{
  */
#define LL_LPUART_DE_POLARITY_HIGH         0x00000000U    /*!< DE signal is active high */
#define LL_LPUART_DE_POLARITY_LOW          USART_CR3_DEP  /*!< DE signal is active low */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_DMA_REG_DATA DMA Register Data
  * @{
  */
#define LL_LPUART_DMA_REG_DATA_TRANSMIT    0x00000000U    /*!< Get address of data register used for transmission */
#define LL_LPUART_DMA_REG_DATA_RECEIVE     0x00000001U    /*!< Get address of data register used for reception */
/**
  * @}
  */

/** @defgroup LPUART_LL_EC_REQUEST Request
  * @{
  */
#define LL_LPUART_REQUEST_SEND_BREAK        USART_RQR_SBKRQ     /*!< Send break request           */
#define LL_LPUART_REQUEST_MUTE_MODE         USART_RQR_MMRQ      /*!< Mute mode request            */
#define LL_LPUART_REQUEST_RXDATA_FLUSH      USART_RQR_RXFRQ     /*!< Receive data flush request   */
#define LL_LPUART_REQUEST_TXDATA_FLUSH      USART_RQR_TXFRQ     /*!< Transmit data flush request  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup LPUART_LL_Exported_Macros LL LPUART Macros
  * @{
  */

/** @defgroup LPUART_LL_EM_WRITE_READ Common register Write and Read Macros
  * @{
  */

/**
  * @brief  Write a value in LPUART register.
  * @param  instance LPUART Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_LPUART_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in LPUART register.
  * @param  instance LPUART Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_LPUART_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/** @defgroup LPUART_LL_EM_Exported_Macros_Helper Helper Macros
  * @{
  */

/**
  * @brief  Compute LPUARTDIV value according to peripheral clock and
  *         expected baud rate (20-bit value of LPUARTDIV is returned).
  * @param  periph_clock Peripheral clock frequency used for LPUART instance
  * @param  prescaler This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PRESCALER_DIV1
  *         @arg @ref LL_LPUART_PRESCALER_DIV2
  *         @arg @ref LL_LPUART_PRESCALER_DIV4
  *         @arg @ref LL_LPUART_PRESCALER_DIV6
  *         @arg @ref LL_LPUART_PRESCALER_DIV8
  *         @arg @ref LL_LPUART_PRESCALER_DIV10
  *         @arg @ref LL_LPUART_PRESCALER_DIV12
  *         @arg @ref LL_LPUART_PRESCALER_DIV16
  *         @arg @ref LL_LPUART_PRESCALER_DIV32
  *         @arg @ref LL_LPUART_PRESCALER_DIV64
  *         @arg @ref LL_LPUART_PRESCALER_DIV128
  *         @arg @ref LL_LPUART_PRESCALER_DIV256
  * @param  baudrate Baud rate value to achieve
  * @retval LPUARTDIV value to be used for BRR register filling
  */
__STATIC_INLINE uint32_t LL_LPUART_DIV(uint32_t periph_clock, uint32_t prescaler, uint32_t baudrate)
{
  return (uint32_t)((((((uint64_t)(periph_clock) / (uint64_t)(LL_LPUART_PRESCALER_TAB[(uint16_t)(prescaler)]))
                       * LL_LPUART_LPUARTDIV_FREQ_MUL) + (uint32_t)((baudrate) / 2U))
                     / (baudrate)) & LL_LPUART_BRR_MASK);
}

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LPUART_LL_Exported_Functions LL LPUART Functions
  * @{
  */

/** @defgroup LPUART_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  LPUART enable.
  * @rmtoll
  *  CR1          UE            LL_LPUART_Enable
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_Enable(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR1, USART_CR1_UE);
}

/**
  * @brief  LPUART disable.
  * @rmtoll
  *  CR1          UE            LL_LPUART_Disable
  * @param  p_lpuart LPUART Instance
  * @note   When LPUART is disabled, LPUART prescalers and outputs are stopped immediately,
  *         and current operations are discarded. The configuration of the LPUART is kept, but all the status
  *         flags, in the LPUARTx_ISR are set to their default values.
  * @note   In order to go into low-power mode without generating errors on the line,
  *         the TE bit must be reset before and the software must wait
  *         for the TC bit in the LPUART_ISR to be set before resetting the UE bit.
  *         The DMA requests are also reset when UE = 0 so the DMA channel must
  *         be disabled before resetting the UE bit.
  */
__STATIC_INLINE void LL_LPUART_Disable(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR1, USART_CR1_UE);
}

/**
  * @brief  Indicate if LPUART is enabled.
  * @rmtoll
  *  CR1          UE            LL_LPUART_IsEnabled
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabled(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_UE) == (USART_CR1_UE)) ? 1UL : 0UL);
}

/**
  * @brief  FIFO mode enable.
  * @rmtoll
  *  CR1          FIFOEN        LL_LPUART_EnableFIFO
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableFIFO(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR1, USART_CR1_FIFOEN);
}

/**
  * @brief  FIFO mode disable.
  * @rmtoll
  *  CR1          FIFOEN        LL_LPUART_DisableFIFO
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableFIFO(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR1, USART_CR1_FIFOEN);
}

/**
  * @brief  Indicate if FIFO mode is enabled.
  * @rmtoll
  *  CR1          FIFOEN        LL_LPUART_IsEnabledFIFO
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledFIFO(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_FIFOEN) == (USART_CR1_FIFOEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure TX FIFO threshold.
  * @rmtoll
  *  CR3          TXFTCFG       LL_LPUART_SetTXFIFOThreshold
  * @param  p_lpuart LPUART Instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE void LL_LPUART_SetTXFIFOThreshold(USART_TypeDef *p_lpuart, uint32_t threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_lpuart->CR3, USART_CR3_TXFTCFG, threshold << USART_CR3_TXFTCFG_Pos);
}

/**
  * @brief  Return TX FIFO threshold configuration.
  * @rmtoll
  *  CR3          TXFTCFG       LL_LPUART_GetTXFIFOThreshold
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE uint32_t LL_LPUART_GetTXFIFOThreshold(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR3, USART_CR3_TXFTCFG) >> USART_CR3_TXFTCFG_Pos);
}

/**
  * @brief  Configure RX FIFO threshold.
  * @rmtoll
  *  CR3          RXFTCFG       LL_LPUART_SetRXFIFOThreshold
  * @param  p_lpuart LPUART Instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE void LL_LPUART_SetRXFIFOThreshold(USART_TypeDef *p_lpuart, uint32_t threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_lpuart->CR3, USART_CR3_RXFTCFG, threshold << USART_CR3_RXFTCFG_Pos);
}

/**
  * @brief  Return RX FIFO threshold configuration.
  * @rmtoll
  *  CR3          RXFTCFG       LL_LPUART_GetRXFIFOThreshold
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE uint32_t LL_LPUART_GetRXFIFOThreshold(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR3, USART_CR3_RXFTCFG) >> USART_CR3_RXFTCFG_Pos);
}

/**
  * @brief  Configure TX and RX FIFOs threshold.
  * @rmtoll
  *  CR3          TXFTCFG       LL_LPUART_ConfigFIFOsThreshold \n
  *  CR3          RXFTCFG       LL_LPUART_ConfigFIFOsThreshold
  * @param  p_lpuart LPUART Instance
  * @param  tx_threshold This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  * @param  rx_threshold This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_LPUART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE void LL_LPUART_ConfigFIFOsThreshold(USART_TypeDef *p_lpuart, uint32_t tx_threshold,
                                                    uint32_t rx_threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_lpuart->CR3, USART_CR3_TXFTCFG | USART_CR3_RXFTCFG,
                             (tx_threshold << USART_CR3_TXFTCFG_Pos) | (rx_threshold << USART_CR3_RXFTCFG_Pos));
}

/**
  * @brief  LPUART enabled in stop mode.
  * @rmtoll
  *  CR1          UESM          LL_LPUART_EnableInStopMode
  * @param  p_lpuart LPUART Instance
  * @note   When this function is enabled, LPUART is able to wake up the MCU from stop mode, provided that
  *         LPUART clock selection is HSI or LSE in RCC.
  */
__STATIC_INLINE void LL_LPUART_EnableInStopMode(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_UESM);
}

/**
  * @brief  LPUART disabled in stop mode.
  * @rmtoll
  *  CR1          UESM          LL_LPUART_DisableInStopMode
  * @param  p_lpuart LPUART Instance
  * @note   When this function is disabled, LPUART is not able to wake up the MCU from stop mode
  */
__STATIC_INLINE void LL_LPUART_DisableInStopMode(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_UESM);
}

/**
  * @brief  Indicate if LPUART is enabled in stop mode
  *         (able to wake up MCU from stop mode or not).
  * @rmtoll
  *  CR1          UESM          LL_LPUART_IsEnabledInStopMode
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledInStopMode(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_UESM) == (USART_CR1_UESM)) ? 1UL : 0UL);
}


/**
  * @brief  Configure the LPUART instance.
  * @rmtoll
  *  CR1         M0             LL_LPUART_ConfigXfer \n
  *  CR1         M1             LL_LPUART_ConfigXfer \n
  *  CR1         PCE            LL_LPUART_ConfigXfer \n
  *  CR1         PS             LL_LPUART_ConfigXfer \n
  *  CR1         TE             LL_LPUART_ConfigXfer \n
  *  CR1         RE             LL_LPUART_ConfigXfer \n
  *  CR2         STOP_0         LL_LPUART_ConfigXfer \n
  *  CR2         STOP_1         LL_LPUART_ConfigXfer
  * @param  p_lpuart LPUART Instance
  * @param cr1_config: This parameter must be a combination of the following groups:
  *         @arg @ref USART_LL_EC_DATAWIDTH
  *         @arg @ref USART_LL_EC_PARITY
  *         @arg @ref USART_LL_EC_DIRECTION
  * @param cr2_config: This parameter must be a combination of the following groups:
  *         @arg @ref USART_LL_EC_STOPBITS
  */
__STATIC_INLINE void LL_LPUART_ConfigXfer(USART_TypeDef *p_lpuart, uint32_t cr1_config, uint32_t cr2_config)
{
  STM32_MODIFY_REG(p_lpuart->CR1, (USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE
                                   | USART_CR1_RE), cr1_config);

  STM32_MODIFY_REG(p_lpuart->CR2, (USART_CR2_STOP_0 | USART_CR2_STOP_1), cr2_config);
}

/**
  * @brief  Receiver enable (receiver is enabled and begins searching for a start bit).
  * @rmtoll
  *  CR1          RE            LL_LPUART_EnableDirectionRx
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDirectionRx(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_RE);
}

/**
  * @brief  Receiver disable.
  * @rmtoll
  *  CR1          RE            LL_LPUART_DisableDirectionRx
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDirectionRx(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_RE);
}

/**
  * @brief  Indicate if the p_lpuart receiver is enabled.
  * @rmtoll
  *  CR1          RE            LL_LPUART_IsEnabledDirectionRx
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDirectionRx(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_RE) == (USART_CR1_RE));
}

/**
  * @brief  Transmitter enable.
  * @rmtoll
  *  CR1          TE            LL_LPUART_EnableDirectionTx
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDirectionTx(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_TE);
}

/**
  * @brief  Transmitter disable.
  * @rmtoll
  *  CR1          TE            LL_LPUART_DisableDirectionTx
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDirectionTx(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_TE);
}

/**
  * @brief  Indicate if the p_lpuart transmitter is enabled.
  * @rmtoll
  *  CR1          TE            LL_LPUART_IsEnabledDirectionTx
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDirectionTx(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_TE) == (USART_CR1_TE));
}

/**
  * @brief  Configure simultaneously enabled/disabled states
  *         of transmitter and receiver.
  * @rmtoll
  *  CR1          RE            LL_LPUART_SetTransferDirection \n
  *  CR1          TE            LL_LPUART_SetTransferDirection
  * @param  p_lpuart LPUART Instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_DIRECTION_NONE
  *         @arg @ref LL_LPUART_DIRECTION_RX
  *         @arg @ref LL_LPUART_DIRECTION_TX
  *         @arg @ref LL_LPUART_DIRECTION_TX_RX
  */
__STATIC_INLINE void LL_LPUART_SetTransferDirection(USART_TypeDef *p_lpuart, uint32_t transfer_direction)
{
  STM32_ATOMIC_MODIFY_REG_32(p_lpuart->CR1, USART_CR1_RE | USART_CR1_TE, transfer_direction);
}

/**
  * @brief  Return enabled/disabled states of transmitter and receiver.
  * @rmtoll
  *  CR1          RE            LL_LPUART_GetTransferDirection \n
  *  CR1          TE            LL_LPUART_GetTransferDirection
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_DIRECTION_NONE
  *         @arg @ref LL_LPUART_DIRECTION_RX
  *         @arg @ref LL_LPUART_DIRECTION_TX
  *         @arg @ref LL_LPUART_DIRECTION_TX_RX
  */
__STATIC_INLINE uint32_t LL_LPUART_GetTransferDirection(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_RE | USART_CR1_TE));
}

/**
  * @brief  Configure parity (enabled/disabled and parity mode if enabled).
  * @rmtoll
  *  CR1          PS            LL_LPUART_SetParity \n
  *  CR1          PCE           LL_LPUART_SetParity
  * @param  p_lpuart LPUART Instance
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PARITY_NONE
  *         @arg @ref LL_LPUART_PARITY_EVEN
  *         @arg @ref LL_LPUART_PARITY_ODD
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd or Even), computed parity bit is inserted at the MSB position
  *         (depending on data width) and parity is checked on the received data.
  */
__STATIC_INLINE void LL_LPUART_SetParity(USART_TypeDef *p_lpuart, uint32_t parity)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_PS | USART_CR1_PCE, parity);
}

/**
  * @brief  Return parity configuration (enabled/disabled and parity mode if enabled).
  * @rmtoll
  *  CR1          PS            LL_LPUART_GetParity \n
  *  CR1          PCE           LL_LPUART_GetParity
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_PARITY_NONE
  *         @arg @ref LL_LPUART_PARITY_EVEN
  *         @arg @ref LL_LPUART_PARITY_ODD
  */
__STATIC_INLINE uint32_t LL_LPUART_GetParity(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_PS | USART_CR1_PCE));
}

/**
  * @brief  Set receiver wake-up method from mute mode.
  * @rmtoll
  *  CR1          WAKE          LL_LPUART_SetWakeUpMethod
  * @param  p_lpuart LPUART Instance
  * @param  method This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_WAKEUP_IDLELINE
  *         @arg @ref LL_LPUART_WAKEUP_ADDRESSMARK
  */
__STATIC_INLINE void LL_LPUART_SetWakeUpMethod(USART_TypeDef *p_lpuart, uint32_t method)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_WAKE, method);
}

/**
  * @brief  Return receiver wake-up method from mute mode.
  * @rmtoll
  *  CR1          WAKE          LL_LPUART_GetWakeUpMethod
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_WAKEUP_IDLELINE
  *         @arg @ref LL_LPUART_WAKEUP_ADDRESSMARK
  */
__STATIC_INLINE uint32_t LL_LPUART_GetWakeUpMethod(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_WAKE));
}

/**
  * @brief  Set word length (nb of data bits, excluding start and stop bits).
  * @rmtoll
  *  CR1          M             LL_LPUART_SetDataWidth
  * @param  p_lpuart LPUART Instance
  * @param  data_width This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_DATAWIDTH_7_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_8_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_9_BIT
  */
__STATIC_INLINE void LL_LPUART_SetDataWidth(USART_TypeDef *p_lpuart, uint32_t data_width)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_M, data_width);
}

/**
  * @brief  Return word length (i.e. nb of data bits, excluding start and stop bits).
  * @rmtoll
  *  CR1          M             LL_LPUART_GetDataWidth
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_DATAWIDTH_7_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_8_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_9_BIT
  */
__STATIC_INLINE uint32_t LL_LPUART_GetDataWidth(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_M));
}

/**
  * @brief  Allow switching between mute mode and active mode.
  * @rmtoll
  *  CR1          MME           LL_LPUART_EnableMuteMode
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableMuteMode(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_MME);
}

/**
  * @brief  Prevent mute mode use. Set receiver in active mode permanently.
  * @rmtoll
  *  CR1          MME           LL_LPUART_DisableMuteMode
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableMuteMode(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_MME);
}

/**
  * @brief  Indicate if switching between mute mode and active mode is allowed.
  * @rmtoll
  *  CR1          MME           LL_LPUART_IsEnabledMuteMode
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledMuteMode(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_MME) == (USART_CR1_MME)) ? 1UL : 0UL);
}

/**
  * @brief  Configure clock source prescaler for baud rate generator and oversampling.
  * @rmtoll
  *  PRESC        PRESCALER     LL_LPUART_SetPrescaler
  * @param  p_lpuart LPUART Instance
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PRESCALER_DIV1
  *         @arg @ref LL_LPUART_PRESCALER_DIV2
  *         @arg @ref LL_LPUART_PRESCALER_DIV4
  *         @arg @ref LL_LPUART_PRESCALER_DIV6
  *         @arg @ref LL_LPUART_PRESCALER_DIV8
  *         @arg @ref LL_LPUART_PRESCALER_DIV10
  *         @arg @ref LL_LPUART_PRESCALER_DIV12
  *         @arg @ref LL_LPUART_PRESCALER_DIV16
  *         @arg @ref LL_LPUART_PRESCALER_DIV32
  *         @arg @ref LL_LPUART_PRESCALER_DIV64
  *         @arg @ref LL_LPUART_PRESCALER_DIV128
  *         @arg @ref LL_LPUART_PRESCALER_DIV256
  */
__STATIC_INLINE void LL_LPUART_SetPrescaler(USART_TypeDef *p_lpuart, uint32_t prescaler_value)
{
  STM32_MODIFY_REG(p_lpuart->PRESC, USART_PRESC_PRESCALER, (uint16_t)prescaler_value);
}

/**
  * @brief  Retrieve the clock source prescaler for baud rate generator and oversampling.
  * @rmtoll
  *  PRESC        PRESCALER     LL_LPUART_GetPrescaler
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_PRESCALER_DIV1
  *         @arg @ref LL_LPUART_PRESCALER_DIV2
  *         @arg @ref LL_LPUART_PRESCALER_DIV4
  *         @arg @ref LL_LPUART_PRESCALER_DIV6
  *         @arg @ref LL_LPUART_PRESCALER_DIV8
  *         @arg @ref LL_LPUART_PRESCALER_DIV10
  *         @arg @ref LL_LPUART_PRESCALER_DIV12
  *         @arg @ref LL_LPUART_PRESCALER_DIV16
  *         @arg @ref LL_LPUART_PRESCALER_DIV32
  *         @arg @ref LL_LPUART_PRESCALER_DIV64
  *         @arg @ref LL_LPUART_PRESCALER_DIV128
  *         @arg @ref LL_LPUART_PRESCALER_DIV256
  */
__STATIC_INLINE uint32_t LL_LPUART_GetPrescaler(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->PRESC, USART_PRESC_PRESCALER));
}

/**
  * @brief  Set the length of the stop bits.
  * @rmtoll
  *  CR2          STOP          LL_LPUART_SetStopBitsLength
  * @param  p_lpuart LPUART Instance
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_STOP_BIT_1
  *         @arg @ref LL_LPUART_STOP_BIT_2
  */
__STATIC_INLINE void LL_LPUART_SetStopBitsLength(USART_TypeDef *p_lpuart, uint32_t stop_bits)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_STOP, stop_bits);
}

/**
  * @brief  Retrieve the length of the stop bits.
  * @rmtoll
  *  CR2          STOP          LL_LPUART_GetStopBitsLength
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_STOP_BIT_1
  *         @arg @ref LL_LPUART_STOP_BIT_2
  */
__STATIC_INLINE uint32_t LL_LPUART_GetStopBitsLength(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_STOP));
}

/**
  * @brief  Configure character frame format (datawidth, parity control, stop bits).
  * @rmtoll
  *  CR1          PS            LL_LPUART_ConfigCharacter \n
  *  CR1          PCE           LL_LPUART_ConfigCharacter \n
  *  CR1          M             LL_LPUART_ConfigCharacter \n
  *  CR2          STOP          LL_LPUART_ConfigCharacter
  * @param  p_lpuart LPUART Instance
  * @param  data_width This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_DATAWIDTH_7_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_8_BIT
  *         @arg @ref LL_LPUART_DATAWIDTH_9_BIT
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PARITY_NONE
  *         @arg @ref LL_LPUART_PARITY_EVEN
  *         @arg @ref LL_LPUART_PARITY_ODD
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_STOP_BIT_1
  *         @arg @ref LL_LPUART_STOP_BIT_2
  * @note   Call of this function is equivalent to the following function call sequence:
  *         - Data Width configuration using @ref LL_LPUART_SetDataWidth() function
  *         - Parity Control and mode configuration using @ref LL_LPUART_SetParity() function
  *         - Stop bits configuration using @ref LL_LPUART_SetStopBitsLength() function
  */
__STATIC_INLINE void LL_LPUART_ConfigCharacter(USART_TypeDef *p_lpuart, uint32_t data_width, uint32_t parity,
                                               uint32_t stop_bits)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_PS | USART_CR1_PCE | USART_CR1_M, parity | data_width);
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_STOP, stop_bits);
}

/**
  * @brief  Configure TX/RX pin swapping setting.
  * @rmtoll
  *  CR2          SWAP          LL_LPUART_SetTXRXSwap
  * @param  p_lpuart LPUART Instance
  * @param  swap_config This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_TXRX_STANDARD
  *         @arg @ref LL_LPUART_TXRX_SWAPPED
  */
__STATIC_INLINE void LL_LPUART_SetTXRXSwap(USART_TypeDef *p_lpuart, uint32_t swap_config)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_SWAP, swap_config);
}

/**
  * @brief  Retrieve TX/RX pin swapping configuration.
  * @rmtoll
  *  CR2          SWAP          LL_LPUART_GetTXRXSwap
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_TXRX_STANDARD
  *         @arg @ref LL_LPUART_TXRX_SWAPPED
  */
__STATIC_INLINE uint32_t LL_LPUART_GetTXRXSwap(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_SWAP));
}

/**
  * @brief  Configure RX pin active level logic.
  * @rmtoll
  *  CR2          RXINV         LL_LPUART_SetRXPinLevel
  * @param  p_lpuart LPUART Instance
  * @param  pin_inv_method This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_RXPIN_LEVEL_STANDARD
  *         @arg @ref LL_LPUART_RXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE void LL_LPUART_SetRXPinLevel(USART_TypeDef *p_lpuart, uint32_t pin_inv_method)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_RXINV, pin_inv_method);
}

/**
  * @brief  Retrieve RX pin active level logic configuration.
  * @rmtoll
  *  CR2          RXINV         LL_LPUART_GetRXPinLevel
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_RXPIN_LEVEL_STANDARD
  *         @arg @ref LL_LPUART_RXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE uint32_t LL_LPUART_GetRXPinLevel(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_RXINV));
}

/**
  * @brief  Configure TX pin active level logic.
  * @rmtoll
  *  CR2          TXINV         LL_LPUART_SetTXPinLevel
  * @param  p_lpuart LPUART Instance
  * @param  pin_inv_method This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_TXPIN_LEVEL_STANDARD
  *         @arg @ref LL_LPUART_TXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE void LL_LPUART_SetTXPinLevel(USART_TypeDef *p_lpuart, uint32_t pin_inv_method)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_TXINV, pin_inv_method);
}

/**
  * @brief  Retrieve TX pin active level logic configuration.
  * @rmtoll
  *  CR2          TXINV         LL_LPUART_GetTXPinLevel
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_TXPIN_LEVEL_STANDARD
  *         @arg @ref LL_LPUART_TXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE uint32_t LL_LPUART_GetTXPinLevel(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_TXINV));
}

/**
  * @brief  Configure binary data logic.
  *
  * @rmtoll
  *  CR2          DATAINV       LL_LPUART_SetBinaryDataLogic
  * @param  p_lpuart LPUART Instance
  * @param  data_logic This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_BINARY_LOGIC_POSITIVE
  *         @arg @ref LL_LPUART_BINARY_LOGIC_NEGATIVE
  * @note   Allow to define how Logical data from the data register are send/received :
  *         either in positive/direct logic (1=H, 0=L) or in negative/inverse logic (1=L, 0=H)
  */
__STATIC_INLINE void LL_LPUART_SetBinaryDataLogic(USART_TypeDef *p_lpuart, uint32_t data_logic)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_DATAINV, data_logic);
}

/**
  * @brief  Retrieve binary data configuration.
  * @rmtoll
  *  CR2          DATAINV       LL_LPUART_GetBinaryDataLogic
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_BINARY_LOGIC_POSITIVE
  *         @arg @ref LL_LPUART_BINARY_LOGIC_NEGATIVE
  */
__STATIC_INLINE uint32_t LL_LPUART_GetBinaryDataLogic(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_DATAINV));
}

/**
  * @brief  Configure transfer bit order (either less or most significant bit first).
  * @rmtoll
  *  CR2          MSBFIRST      LL_LPUART_SetTransferBitOrder
  * @param  p_lpuart LPUART Instance
  * @param  bit_order This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_BITORDER_LSBFIRST
  *         @arg @ref LL_LPUART_BITORDER_MSBFIRST
  * @note   MSB-first means data is transmitted/received with the MSB first, following the start bit.
  *         LSB-first means data is transmitted/received with data bit 0 first, following the start bit.
  */
__STATIC_INLINE void LL_LPUART_SetTransferBitOrder(USART_TypeDef *p_lpuart, uint32_t bit_order)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_MSBFIRST, bit_order);
}

/**
  * @brief  Return transfer bit order (either less or most significant bit first).
  * @rmtoll
  *  CR2          MSBFIRST      LL_LPUART_GetTransferBitOrder
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_BITORDER_LSBFIRST
  *         @arg @ref LL_LPUART_BITORDER_MSBFIRST
  * @note   MSB-first means data is transmitted/received with the MSB first, following the start bit.
  *         LSB-first means data is transmitted/received with data bit 0 first, following the start bit.
  */
__STATIC_INLINE uint32_t LL_LPUART_GetTransferBitOrder(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_MSBFIRST));
}

/**
  * @brief  Set an 8-bit address of the LPUART node as set in ADD field of CR2.
  * @rmtoll
  *  CR2          ADD           LL_LPUART_SetNodeAddress
  * @param  p_lpuart LPUART Instance
  * @param  node_address 4-bit or 7-bit address of the LPUART node.
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4 bits (b3-b0) of returned value are relevant (b31-b4 are not relevant).
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8 bits (b7-b0) of returned value are relevant (b31-b8 are not relevant).
  */
__STATIC_INLINE void LL_LPUART_SetNodeAddress(USART_TypeDef *p_lpuart, uint32_t node_address)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_ADD, (node_address << USART_CR2_ADD_Pos));
}

/**
  * @brief  Return 8-bit address of the LPUART node as set in ADD field of CR2.
  * @rmtoll
  *  CR2          ADD           LL_LPUART_GetNodeAddress
  * @param  p_lpuart LPUART Instance
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4 bits (b3-b0) of returned value are relevant (b31-b4 are not relevant).
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8 bits (b7-b0) of returned value are relevant (b31-b8 are not relevant).
  * @retval Address of the LPUART node (value between Min_Data=0 and Max_Data=255).
  */
__STATIC_INLINE uint32_t LL_LPUART_GetNodeAddress(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_ADD) >> USART_CR2_ADD_Pos);
}

/**
  * @brief  Set the address length of the LPUART node in ADDM7 field of CR2.
  * @rmtoll
  *  CR2          ADDM7           LL_LPUART_SetNodeAddressLength
  * @param  p_lpuart LPUART Instance
  * @param  address_len This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_7_BIT
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4 bits (b3-b0) of node address value are relevant (b7-b4 are not relevant).
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8 bits (b7-b0) of node address value are relevant.
  */
__STATIC_INLINE void LL_LPUART_SetNodeAddressLength(USART_TypeDef *p_lpuart, uint32_t address_len)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_ADDM7, address_len);
}

/**
  * @brief  Return length of node address used in address detection mode (7-bit or 4-bit).
  * @rmtoll
  *  CR2          ADDM7         LL_LPUART_GetNodeAddressLength
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_7_BIT
  */
__STATIC_INLINE uint32_t LL_LPUART_GetNodeAddressLength(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR2, USART_CR2_ADDM7));
}

/**
  * @brief  Configure address and address length of the LPUART node.
  * @rmtoll
  *  CR2          ADD           LL_LPUART_ConfigNodeAddress \n
  *  CR2          ADDM7         LL_LPUART_ConfigNodeAddress
  * @param  p_lpuart LPUART Instance
  * @param  address_len This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_LPUART_ADDRESS_DETECT_7_BIT
  * @param  node_address 4- or 7-bit address of the LPUART node.
  * @note   This is used in multiprocessor communication during mute mode or stop mode,
  *         for wake-up with address mark detection.
  * @note   4-bit address node is used when 4-bit Address Detection is selected in ADDM7.
  *         (b7-b4 must be set to 0)
  *         8-bit address node is used when 7-bit Address Detection is selected in ADDM7.
  *         (This is used in multiprocessor communication during mute mode or stop mode,
  *         for wake-up with 7-bit address mark detection.
  *         The MSB of the character sent by the transmitter must be equal to 1.
  *         It could also be used for character detection during normal reception,
  *         mute mode inactive (for example, end of block detection in ModBus protocol).
  *         In this case, the whole received character (8-bit) is compared to the ADD[7:0]
  *         value and CMF flag is set on match).
  */
__STATIC_INLINE void LL_LPUART_ConfigNodeAddress(USART_TypeDef *p_lpuart, uint32_t address_len, uint32_t node_address)
{
  STM32_MODIFY_REG(p_lpuart->CR2, USART_CR2_ADD | USART_CR2_ADDM7,
                   (uint32_t)(address_len | (node_address << USART_CR2_ADD_Pos)));
}

/**
  * @brief  Enable RTS HW flow control.
  * @rmtoll
  *  CR3          RTSE          LL_LPUART_EnableRTSHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableRTSHWFlowCtrl(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_RTSE);
}

/**
  * @brief  Disable RTS HW flow control.
  * @rmtoll
  *  CR3          RTSE          LL_LPUART_DisableRTSHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableRTSHWFlowCtrl(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_RTSE);
}

/**
  * @brief  Enable CTS HW flow control.
  * @rmtoll
  *  CR3          CTSE          LL_LPUART_EnableCTSHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableCTSHWFlowCtrl(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_CTSE);
}

/**
  * @brief  Disable CTS HW flow control.
  * @rmtoll
  *  CR3          CTSE          LL_LPUART_DisableCTSHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableCTSHWFlowCtrl(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_CTSE);
}

/**
  * @brief  Configure HW flow control mode (both CTS and RTS).
  * @rmtoll
  *  CR3          RTSE          LL_LPUART_SetHWFlowCtrl \n
  *  CR3          CTSE          LL_LPUART_SetHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  * @param  hardware_flow_control This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_HWCONTROL_NONE
  *         @arg @ref LL_LPUART_HWCONTROL_RTS
  *         @arg @ref LL_LPUART_HWCONTROL_CTS
  *         @arg @ref LL_LPUART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE void LL_LPUART_SetHWFlowCtrl(USART_TypeDef *p_lpuart, uint32_t hardware_flow_control)
{
  STM32_MODIFY_REG(p_lpuart->CR3, USART_CR3_RTSE | USART_CR3_CTSE, hardware_flow_control);
}

/**
  * @brief  Return HW flow control configuration (both CTS and RTS).
  * @rmtoll
  *  CR3          RTSE          LL_LPUART_GetHWFlowCtrl \n
  *  CR3          CTSE          LL_LPUART_GetHWFlowCtrl
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_HWCONTROL_NONE
  *         @arg @ref LL_LPUART_HWCONTROL_RTS
  *         @arg @ref LL_LPUART_HWCONTROL_CTS
  *         @arg @ref LL_LPUART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE uint32_t LL_LPUART_GetHWFlowCtrl(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

/**
  * @brief  Enable overrun detection.
  * @rmtoll
  *  CR3          OVRDIS        LL_LPUART_EnableOverrunDetect
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableOverrunDetect(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR3, USART_CR3_OVRDIS);
}

/**
  * @brief  Disable overrun detection.
  * @rmtoll
  *  CR3          OVRDIS        LL_LPUART_DisableOverrunDetect
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableOverrunDetect(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR3, USART_CR3_OVRDIS);
}

/**
  * @brief  Indicate if overrun detection is enabled.
  * @rmtoll
  *  CR3          OVRDIS        LL_LPUART_IsEnabledOverrunDetect
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledOverrunDetect(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_OVRDIS) != USART_CR3_OVRDIS) ? 1UL : 0UL);
}

/**
  * @brief  Select event type for wake-up interrupt flag (WUS[1:0] bits).
  * @rmtoll
  *  CR3          WUS           LL_LPUART_SetWKUPType
  * @param  p_lpuart LPUART Instance
  * @param  type This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_WAKEUP_ON_ADDRESS
  *         @arg @ref LL_LPUART_WAKEUP_ON_STARTBIT
  *         @arg @ref LL_LPUART_WAKEUP_ON_RXNE
  */
__STATIC_INLINE void LL_LPUART_SetWKUPType(USART_TypeDef *p_lpuart, uint32_t type)
{
  STM32_MODIFY_REG(p_lpuart->CR3, USART_CR3_WUS, type);
}

/**
  * @brief  Return event type for wake-up interrupt flag (WUS[1:0] bits).
  * @rmtoll
  *  CR3          WUS           LL_LPUART_GetWKUPType
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_WAKEUP_ON_ADDRESS
  *         @arg @ref LL_LPUART_WAKEUP_ON_STARTBIT
  *         @arg @ref LL_LPUART_WAKEUP_ON_RXNE
  */
__STATIC_INLINE uint32_t LL_LPUART_GetWKUPType(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR3, USART_CR3_WUS));
}

/**
  * @brief  Configure LPUART BRR register for achieving expected baud rate value.
  * @rmtoll
  *  BRR          BRR           LL_LPUART_SetBaudRate
  * @param  p_lpuart LPUART Instance
  * @param  periph_clk Peripheral Clock
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PRESCALER_DIV1
  *         @arg @ref LL_LPUART_PRESCALER_DIV2
  *         @arg @ref LL_LPUART_PRESCALER_DIV4
  *         @arg @ref LL_LPUART_PRESCALER_DIV6
  *         @arg @ref LL_LPUART_PRESCALER_DIV8
  *         @arg @ref LL_LPUART_PRESCALER_DIV10
  *         @arg @ref LL_LPUART_PRESCALER_DIV12
  *         @arg @ref LL_LPUART_PRESCALER_DIV16
  *         @arg @ref LL_LPUART_PRESCALER_DIV32
  *         @arg @ref LL_LPUART_PRESCALER_DIV64
  *         @arg @ref LL_LPUART_PRESCALER_DIV128
  *         @arg @ref LL_LPUART_PRESCALER_DIV256
  * @param  baud_rate Baud rate.
  * @note   Compute and set LPUARTDIV value in BRR register (full BRR content)
  *         according to used peripheral clock and expected baud rate values.
  * @note   Peripheral clock and baud rate values provided as function parameters must be valid
  *         (baud rate value != 0).
  * @note   Provided that LPUARTx_BRR must be > = 0x300 and LPUART_BRR is 20-bit,
  *         care must be taken when generating high baud rates using high periph_clk
  *         values. periph_clk must be in the range [3 x baud_rate, 4096 x baud_rate].
  */
__STATIC_INLINE void LL_LPUART_SetBaudRate(USART_TypeDef *p_lpuart, uint32_t periph_clk, uint32_t prescaler_value,
                                           uint32_t baud_rate)
{
  if (baud_rate != 0U)
  {
    p_lpuart->BRR = LL_LPUART_DIV(periph_clk, prescaler_value, baud_rate);
  }
}

/**
  * @brief  Return current baud rate value, according to LPUARTDIV present in BRR register
  *         (full BRR content), and to used peripheral clock values.
  * @rmtoll
  *  BRR          BRR           LL_LPUART_GetBaudRate
  * @param  p_lpuart LPUART Instance
  * @param  periph_clk Peripheral Clock
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_PRESCALER_DIV1
  *         @arg @ref LL_LPUART_PRESCALER_DIV2
  *         @arg @ref LL_LPUART_PRESCALER_DIV4
  *         @arg @ref LL_LPUART_PRESCALER_DIV6
  *         @arg @ref LL_LPUART_PRESCALER_DIV8
  *         @arg @ref LL_LPUART_PRESCALER_DIV10
  *         @arg @ref LL_LPUART_PRESCALER_DIV12
  *         @arg @ref LL_LPUART_PRESCALER_DIV16
  *         @arg @ref LL_LPUART_PRESCALER_DIV32
  *         @arg @ref LL_LPUART_PRESCALER_DIV64
  *         @arg @ref LL_LPUART_PRESCALER_DIV128
  *         @arg @ref LL_LPUART_PRESCALER_DIV256
  * @note   In case of non-initialized or invalid value stored in BRR register, value 0 will be returned.
  * @retval Baud rate
  */
__STATIC_INLINE uint32_t LL_LPUART_GetBaudRate(const USART_TypeDef *p_lpuart, uint32_t periph_clk,
                                               uint32_t prescaler_value)
{
  uint32_t lpuartdiv;
  uint32_t brrresult;
  uint32_t periphclkpresc = (uint32_t)(periph_clk / (LL_LPUART_PRESCALER_TAB[(uint16_t)prescaler_value]));

  lpuartdiv = p_lpuart->BRR & LL_LPUART_BRR_MASK;

  if (lpuartdiv >= LL_LPUART_BRR_MIN_VALUE)
  {
    brrresult = (uint32_t)(((uint64_t)(periphclkpresc) * LL_LPUART_LPUARTDIV_FREQ_MUL) / lpuartdiv);
  }
  else
  {
    brrresult = 0x0UL;
  }

  return (brrresult);
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_Configuration_HalfDuplex Configuration functions related to half duplex feature.
  * @{
  */

/**
  * @brief  Enable single wire half-duplex mode.
  * @rmtoll
  *  CR3          HDSEL         LL_LPUART_EnableHalfDuplex
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableHalfDuplex(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Disable single wire half-duplex mode.
  * @rmtoll
  *  CR3          HDSEL         LL_LPUART_DisableHalfDuplex
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableHalfDuplex(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Indicate if single wire half-duplex mode is enabled.
  * @rmtoll
  *  CR3          HDSEL         LL_LPUART_IsEnabledHalfDuplex
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledHalfDuplex(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_HDSEL) == (USART_CR3_HDSEL)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_Configuration_DE Configuration functions related to driver enable feature
  * @{
  */

/**
  * @brief  Set DEDT (Driver Enable De-Assertion Time), time value expressed on 5 bits ([4:0] bits).
  * @rmtoll
  *  CR1          DEDT          LL_LPUART_SetDEDeassertionTime
  * @param  p_lpuart LPUART Instance
  * @param  time Value between Min_Data=0 and Max_Data=31: expressed in lpuart kernel clock cycles.
  */
__STATIC_INLINE void LL_LPUART_SetDEDeassertionTime(USART_TypeDef *p_lpuart, uint32_t time)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_DEDT, time << USART_CR1_DEDT_Pos);
}

/**
  * @brief  Return DEDT (Driver Enable De-Assertion Time).
  * @rmtoll
  *  CR1          DEDT          LL_LPUART_GetDEDeassertionTime
  * @param  p_lpuart LPUART Instance
  * @retval Time value expressed on 5 bits ([4:0] bits) : expressed in lpuart kernel clock cycles
  */
__STATIC_INLINE uint32_t LL_LPUART_GetDEDeassertionTime(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_DEDT) >> USART_CR1_DEDT_Pos);
}

/**
  * @brief  Set DEAT (Driver Enable Assertion Time), time value expressed on 5 bits ([4:0] bits).
  * @rmtoll
  *  CR1          DEAT          LL_LPUART_SetDEAssertionTime
  * @param  p_lpuart LPUART Instance
  * @param  time Value between Min_Data=0 and Max_Data=31: expressed in lpuart kernel clock cycles.
  */
__STATIC_INLINE void LL_LPUART_SetDEAssertionTime(USART_TypeDef *p_lpuart, uint32_t time)
{
  STM32_MODIFY_REG(p_lpuart->CR1, USART_CR1_DEAT, time << USART_CR1_DEAT_Pos);
}

/**
  * @brief  Return DEAT (Driver Enable Assertion Time).
  * @rmtoll
  *  CR1          DEAT          LL_LPUART_GetDEAssertionTime
  * @param  p_lpuart LPUART Instance
  * @retval Time value expressed on 5 bits ([4:0] bits) : expressed in lpuart kernel clock cycles
  */
__STATIC_INLINE uint32_t LL_LPUART_GetDEAssertionTime(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR1, USART_CR1_DEAT) >> USART_CR1_DEAT_Pos);
}

/**
  * @brief  Enable driver enable (DE) mode.
  * @rmtoll
  *  CR3          DEM           LL_LPUART_EnableDEMode
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDEMode(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR3, USART_CR3_DEM);
}

/**
  * @brief  Disable driver enable (DE) mode.
  * @rmtoll
  *  CR3          DEM           LL_LPUART_DisableDEMode
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDEMode(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR3, USART_CR3_DEM);
}

/**
  * @brief  Indicate if driver enable (DE) mode is enabled.
  * @rmtoll
  *  CR3          DEM           LL_LPUART_IsEnabledDEMode
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDEMode(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_DEM) == (USART_CR3_DEM)) ? 1UL : 0UL);
}

/**
  * @brief  Select driver enable polarity.
  * @rmtoll
  *  CR3          DEP           LL_LPUART_SetDESignalPolarity
  * @param  p_lpuart LPUART Instance
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_DE_POLARITY_HIGH
  *         @arg @ref LL_LPUART_DE_POLARITY_LOW
  */
__STATIC_INLINE void LL_LPUART_SetDESignalPolarity(USART_TypeDef *p_lpuart, uint32_t polarity)
{
  STM32_MODIFY_REG(p_lpuart->CR3, USART_CR3_DEP, polarity);
}

/**
  * @brief  Return driver enable polarity.
  * @rmtoll
  *  CR3          DEP           LL_LPUART_GetDESignalPolarity
  * @param  p_lpuart LPUART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPUART_DE_POLARITY_HIGH
  *         @arg @ref LL_LPUART_DE_POLARITY_LOW
  */
__STATIC_INLINE uint32_t LL_LPUART_GetDESignalPolarity(const USART_TypeDef *p_lpuart)
{
  return (uint32_t)(STM32_READ_BIT(p_lpuart->CR3, USART_CR3_DEP));
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if the USART flag mask is set or not.
  * @rmtoll
  *  ISR          PE            LL_LPUART_IsActiveFlag \n
  *  ISR          FE            LL_LPUART_IsActiveFlag \n
  *  ISR          NE            LL_LPUART_IsActiveFlag \n
  *  ISR          ORE           LL_LPUART_IsActiveFlag \n
  *  ISR          IDLE          LL_LPUART_IsActiveFlag \n
  *  ISR          RXNE_RXFNE    LL_LPUART_IsActiveFlag \n
  *  ISR          TC            LL_LPUART_IsActiveFlag \n
  *  ISR          TXE_TXFNF     LL_LPUART_IsActiveFlag \n
  *  ISR          CTSIF         LL_LPUART_IsActiveFlag \n
  *  ISR          CTS           LL_LPUART_IsActiveFlag \n
  *  ISR          RTOF          LL_LPUART_IsActiveFlag \n
  *  ISR          ABRE          LL_LPUART_IsActiveFlag \n
  *  ISR          ABRF          LL_LPUART_IsActiveFlag \n
  *  ISR          BUSY          LL_LPUART_IsActiveFlag \n
  *  ISR          CMF           LL_LPUART_IsActiveFlag \n
  *  ISR          SBKF          LL_LPUART_IsActiveFlag \n
  *  ISR          RWU           LL_LPUART_IsActiveFlag \n
  *  ISR          WUF           LL_LPUART_IsActiveFlag \n
  *  ISR          TEACK         LL_LPUART_IsActiveFlag \n
  *  ISR          REACK         LL_LPUART_IsActiveFlag \n
  *  ISR          TXFE          LL_LPUART_IsActiveFlag \n
  *  ISR          RXFF          LL_LPUART_IsActiveFlag \n
  *  ISR          TXFT          LL_LPUART_IsActiveFlag \n
  *  ISR          RXFT          LL_LPUART_IsActiveFlag
  * @param  p_lpuart LPUART Instance
  * @param  mask Mask to test
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag(const USART_TypeDef *p_lpuart, uint32_t mask)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART parity error flag is set or not.
  * @rmtoll
  *  ISR          PE            LL_LPUART_IsActiveFlag_PE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_PE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_PE) == (USART_ISR_PE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART framing error flag is set or not.
  * @rmtoll
  *  ISR          FE            LL_LPUART_IsActiveFlag_FE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_FE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_FE) == (USART_ISR_FE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART noise error detected flag is set or not.
  * @rmtoll
  *  ISR          NE            LL_LPUART_IsActiveFlag_NE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_NE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_NE) == (USART_ISR_NE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART overrun error flag is set or not.
  * @rmtoll
  *  ISR          ORE           LL_LPUART_IsActiveFlag_ORE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_ORE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_ORE) == (USART_ISR_ORE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART IDLE line detected flag is set or not.
  * @rmtoll
  *  ISR          IDLE          LL_LPUART_IsActiveFlag_IDLE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_IDLE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_IDLE) == (USART_ISR_IDLE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART read data register or LPUART RX FIFO not empty flag is set or not.
  * @rmtoll
  *  ISR          RXNE_RXFNE    LL_LPUART_IsActiveFlag_RXNE_RXFNE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_RXNE_RXFNE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_RXNE_RXFNE) == (USART_ISR_RXNE_RXFNE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART transmission complete flag is set or not.
  * @rmtoll
  *  ISR          TC            LL_LPUART_IsActiveFlag_TC
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_TC(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_TC) == (USART_ISR_TC)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART transmit data register empty or LPUART TX FIFO not full flag is set or not.
  * @rmtoll
  *  ISR          TXE_TXFNF     LL_LPUART_IsActiveFlag_TXE_TXFNF
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_TXE_TXFNF(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_TXE_TXFNF) == (USART_ISR_TXE_TXFNF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART CTS interrupt flag is set or not.
  * @rmtoll
  *  ISR          CTSIF         LL_LPUART_IsActiveFlag_nCTS
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_nCTS(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_CTSIF) == (USART_ISR_CTSIF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART CTS flag is set or not.
  * @rmtoll
  *  ISR          CTS           LL_LPUART_IsActiveFlag_CTS
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_CTS(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_CTS) == (USART_ISR_CTS)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART busy flag is set or not.
  * @rmtoll
  *  ISR          BUSY          LL_LPUART_IsActiveFlag_BUSY
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_BUSY(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_BUSY) == (USART_ISR_BUSY)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART character match flag is set or not.
  * @rmtoll
  *  ISR          CMF           LL_LPUART_IsActiveFlag_CM
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_CM(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_CMF) == (USART_ISR_CMF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART send break flag is set or not.
  * @rmtoll
  *  ISR          SBKF          LL_LPUART_IsActiveFlag_SBK
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_SBK(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_SBKF) == (USART_ISR_SBKF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART receive wake-up from mute mode flag is set or not.
  * @rmtoll
  *  ISR          RWU           LL_LPUART_IsActiveFlag_RWU
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_RWU(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_RWU) == (USART_ISR_RWU)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART wake-up from stop mode flag is set or not.
  * @rmtoll
  *  ISR          WUF           LL_LPUART_IsActiveFlag_WKUP
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_WKUP(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_WUF) == (USART_ISR_WUF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART transmit enable acknowledge flag is set or not.
  * @rmtoll
  *  ISR          TEACK         LL_LPUART_IsActiveFlag_TEACK
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_TEACK(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_TEACK) == (USART_ISR_TEACK)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART receive enable acknowledge flag is set or not.
  * @rmtoll
  *  ISR          REACK         LL_LPUART_IsActiveFlag_REACK
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_REACK(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_REACK) == (USART_ISR_REACK)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART TX FIFO empty flag is set or not.
  * @rmtoll
  *  ISR          TXFE          LL_LPUART_IsActiveFlag_TXFE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_TXFE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_TXFE) == (USART_ISR_TXFE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART RX FIFO full flag is set or not.
  * @rmtoll
  *  ISR          RXFF          LL_LPUART_IsActiveFlag_RXFF
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_RXFF(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_RXFF) == (USART_ISR_RXFF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART TX FIFO threshold flag is set or not.
  * @rmtoll
  *  ISR          TXFT          LL_LPUART_IsActiveFlag_TXFT
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_TXFT(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_TXFT) == (USART_ISR_TXFT)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART RX FIFO threshold flag is set or not.
  * @rmtoll
  *  ISR          RXFT          LL_LPUART_IsActiveFlag_RXFT
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsActiveFlag_RXFT(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->ISR, USART_ISR_RXFT) == (USART_ISR_RXFT)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the flag mask.
  * @rmtoll
  *  ISR          PE            LL_LPUART_ClearFlag \n
  *  ISR          FE            LL_LPUART_ClearFlag \n
  *  ISR          NE            LL_LPUART_ClearFlag \n
  *  ISR          ORE           LL_LPUART_ClearFlag \n
  *  ISR          IDLE          LL_LPUART_ClearFlag \n
  *  ISR          TC            LL_LPUART_ClearFlag \n
  *  ISR          CTS           LL_LPUART_ClearFlag \n
  *  ISR          RTOF          LL_LPUART_ClearFlag \n
  *  ISR          CMF           LL_LPUART_ClearFlag \n
  *  ISR          WUF           LL_LPUART_ClearFlag \n
  *  ISR          TXFE          LL_LPUART_ClearFlag \n
  * @param  p_lpuart LPUART Instance
  * @param  mask Mask to test
  */
__STATIC_INLINE void LL_LPUART_ClearFlag(USART_TypeDef *p_lpuart, uint32_t mask)
{
  STM32_WRITE_REG(p_lpuart->ICR, mask);
}

/**
  * @brief  Clear parity error flag.
  * @rmtoll
  *  ICR          PECF          LL_LPUART_ClearFlag_PE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_PE(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_PECF);
}

/**
  * @brief  Clear framing error flag.
  * @rmtoll
  *  ICR          FECF          LL_LPUART_ClearFlag_FE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_FE(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_FECF);
}

/**
  * @brief  Clear noise detected flag.
  * @rmtoll
  *  ICR          NECF          LL_LPUART_ClearFlag_NE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_NE(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_NECF);
}

/**
  * @brief  Clear overrun error flag.
  * @rmtoll
  *  ICR          ORECF         LL_LPUART_ClearFlag_ORE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_ORE(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_ORECF);
}

/**
  * @brief  Clear IDLE line detected flag.
  * @rmtoll
  *  ICR          IDLECF        LL_LPUART_ClearFlag_IDLE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_IDLE(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_IDLECF);
}

/**
  * @brief  Clear transmission complete flag.
  * @rmtoll
  *  ICR          TCCF          LL_LPUART_ClearFlag_TC
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_TC(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_TCCF);
}

/**
  * @brief  Clear CTS interrupt flag.
  * @rmtoll
  *  ICR          CTSCF         LL_LPUART_ClearFlag_nCTS
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_nCTS(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_CTSCF);
}

/**
  * @brief  Clear character match flag.
  * @rmtoll
  *  ICR          CMCF          LL_LPUART_ClearFlag_CM
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_CM(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_CMCF);
}

/**
  * @brief  Clear wake-up from stop mode flag.
  * @rmtoll
  *  ICR          WUCF          LL_LPUART_ClearFlag_WKUP
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_ClearFlag_WKUP(USART_TypeDef *p_lpuart)
{
  STM32_WRITE_REG(p_lpuart->ICR, USART_ICR_WUCF);
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable IDLE interrupt.
  * @rmtoll
  *  CR1          IDLEIE        LL_LPUART_EnableIT_IDLE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_IDLE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_IDLEIE);
}

/**
  * @brief  Enable RX not empty and RX FIFO not empty interrupt.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_LPUART_EnableIT_RXNE_RXFNE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_RXNE_RXFNE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

/**
  * @brief  Enable transmission complete interrupt.
  * @rmtoll
  *  CR1          TCIE          LL_LPUART_EnableIT_TC
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_TC(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_TCIE);
}

/**
  * @brief  Enable TX empty and TX FIFO not full interrupt.
  * @rmtoll
  *  CR1         TXEIE_TXFNFIE  LL_LPUART_EnableIT_TXE_TXFNF
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_TXE_TXFNF(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
  * @brief  Enable parity error interrupt.
  * @rmtoll
  *  CR1          PEIE          LL_LPUART_EnableIT_PE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_PE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_PEIE);
}

/**
  * @brief  Enable character match interrupt.
  * @rmtoll
  *  CR1          CMIE          LL_LPUART_EnableIT_CM
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_CM(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_CMIE);
}

/**
  * @brief  Enable TX FIFO empty interrupt.
  * @rmtoll
  *  CR1          TXFEIE        LL_LPUART_EnableIT_TXFE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_TXFE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_TXFEIE);
}

/**
  * @brief  Enable RX FIFO full interrupt.
  * @rmtoll
  *  CR1          RXFFIE        LL_LPUART_EnableIT_RXFF
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_RXFF(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR1, USART_CR1_RXFFIE);
}

/**
  * @brief  Enable error interrupt.
  * @rmtoll
  *  CR3          EIE           LL_LPUART_EnableIT_ERROR
  * @param  p_lpuart LPUART Instance
  * @note   When set, error interrupt enable bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the LPUARTx_ISR register).
  *         - 0: Interrupt is inhibited
  *         - 1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the LPUARTx_ISR register.
  */
__STATIC_INLINE void LL_LPUART_EnableIT_ERROR(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_EIE);
}

/**
  * @brief  Enable CTS interrupt.
  * @rmtoll
  *  CR3          CTSIE         LL_LPUART_EnableIT_CTS
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_CTS(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_CTSIE);
}

/**
  * @brief  Enable wake-up from stop mode interrupt.
  * @rmtoll
  *  CR3          WUFIE         LL_LPUART_EnableIT_WKUP
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_WKUP(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_WUFIE);
}

/**
  * @brief  Enable TX FIFO threshold interrupt.
  * @rmtoll
  *  CR3          TXFTIE        LL_LPUART_EnableIT_TXFT
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_TXFT(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_TXFTIE);
}

/**
  * @brief  Enable RX FIFO threshold interrupt.
  * @rmtoll
  *  CR3          RXFTIE        LL_LPUART_EnableIT_RXFT
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableIT_RXFT(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_RXFTIE);
}

/**
  * @brief  Disable IDLE interrupt.
  * @rmtoll
  *  CR1          IDLEIE        LL_LPUART_DisableIT_IDLE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_IDLE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_IDLEIE);
}

/**
  * @brief  Disable RX not empty and RX FIFO not empty interrupt.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_LPUART_DisableIT_RXNE_RXFNE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_RXNE_RXFNE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

/**
  * @brief  Disable transmission complete interrupt.
  * @rmtoll
  *  CR1          TCIE          LL_LPUART_DisableIT_TC
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_TC(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_TCIE);
}

/**
  * @brief  Disable TX empty and TX FIFO not full interrupt.
  * @rmtoll
  *  CR1        TXEIE_TXFNFIE  LL_LPUART_DisableIT_TXE_TXFNF
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_TXE_TXFNF(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
  * @brief  Disable parity error interrupt.
  * @rmtoll
  *  CR1          PEIE          LL_LPUART_DisableIT_PE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_PE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_PEIE);
}

/**
  * @brief  Disable character match interrupt.
  * @rmtoll
  *  CR1          CMIE          LL_LPUART_DisableIT_CM
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_CM(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_CMIE);
}

/**
  * @brief  Disable TX FIFO empty interrupt.
  * @rmtoll
  *  CR1          TXFEIE        LL_LPUART_DisableIT_TXFE
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_TXFE(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_TXFEIE);
}

/**
  * @brief  Disable RX FIFO full interrupt.
  * @rmtoll
  *  CR1          RXFFIE        LL_LPUART_DisableIT_RXFF
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_RXFF(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR1, USART_CR1_RXFFIE);
}

/**
  * @brief  Disable error interrupt.
  * @rmtoll
  *  CR3          EIE           LL_LPUART_DisableIT_ERROR
  * @param  p_lpuart LPUART Instance
  * @note   When set, error interrupt enable bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the LPUARTx_ISR register).
  *         - 0: Interrupt is inhibited
  *         - 1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the LPUARTx_ISR register.
  */
__STATIC_INLINE void LL_LPUART_DisableIT_ERROR(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_EIE);
}

/**
  * @brief  Disable CTS interrupt.
  * @rmtoll
  *  CR3          CTSIE         LL_LPUART_DisableIT_CTS
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_CTS(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_CTSIE);
}

/**
  * @brief  Disable wake-up from stop mode interrupt.
  * @rmtoll
  *  CR3          WUFIE         LL_LPUART_DisableIT_WKUP
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_WKUP(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_WUFIE);
}

/**
  * @brief  Disable TX FIFO threshold interrupt.
  * @rmtoll
  *  CR3          TXFTIE        LL_LPUART_DisableIT_TXFT
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_TXFT(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_TXFTIE);
}

/**
  * @brief  Disable RX FIFO threshold interrupt.
  * @rmtoll
  *  CR3          RXFTIE        LL_LPUART_DisableIT_RXFT
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableIT_RXFT(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_RXFTIE);
}

/**
  * @brief  Check if the LPUART IDLE interrupt source is enabled or disabled.
  * @rmtoll
  *  CR1          IDLEIE        LL_LPUART_IsEnabledIT_IDLE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_IDLE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_IDLEIE) == (USART_CR1_IDLEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART RX not empty and LPUART RX FIFO not empty interrupt is enabled or disabled.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_LPUART_IsEnabledIT_RXNE_RXFNE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_RXNE_RXFNE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_RXNEIE_RXFNEIE) == (USART_CR1_RXNEIE_RXFNEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART transmission complete interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TCIE          LL_LPUART_IsEnabledIT_TC
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_TC(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_TCIE) == (USART_CR1_TCIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART TX empty and LPUART TX FIFO not full interrupt is enabled or disabled.
  * @rmtoll
  *  CR1         TXEIE_TXFNFIE  LL_LPUART_IsEnabledIT_TXE_TXFNF
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_TXE_TXFNF(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_TXEIE_TXFNFIE) == (USART_CR1_TXEIE_TXFNFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART parity error interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          PEIE          LL_LPUART_IsEnabledIT_PE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_PE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_PEIE) == (USART_CR1_PEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART character match interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          CMIE          LL_LPUART_IsEnabledIT_CM
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_CM(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_CMIE) == (USART_CR1_CMIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART TX FIFO empty interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TXFEIE        LL_LPUART_IsEnabledIT_TXFE
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_TXFE(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_TXFEIE) == (USART_CR1_TXFEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART RX FIFO full interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          RXFFIE        LL_LPUART_IsEnabledIT_RXFF
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_RXFF(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR1, USART_CR1_RXFFIE) == (USART_CR1_RXFFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART error interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          EIE           LL_LPUART_IsEnabledIT_ERROR
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_ERROR(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_EIE) == (USART_CR3_EIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART CTS interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          CTSIE         LL_LPUART_IsEnabledIT_CTS
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_CTS(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_CTSIE) == (USART_CR3_CTSIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the LPUART wake-up from stop mode interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          WUFIE         LL_LPUART_IsEnabledIT_WKUP
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_WKUP(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_WUFIE) == (USART_CR3_WUFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if LPUART TX FIFO threshold interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          TXFTIE        LL_LPUART_IsEnabledIT_TXFT
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_TXFT(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_TXFTIE) == (USART_CR3_TXFTIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if LPUART RX FIFO threshold interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          RXFTIE        LL_LPUART_IsEnabledIT_RXFT
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledIT_RXFT(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_RXFTIE) == (USART_CR3_RXFTIE)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA mode for reception.
  * @rmtoll
  *  CR3          DMAR          LL_LPUART_EnableDMAReq_RX
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDMAReq_RX(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Disable DMA mode for reception.
  * @rmtoll
  *  CR3          DMAR          LL_LPUART_DisableDMAReq_RX
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDMAReq_RX(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Check if DMA mode is enabled for reception.
  * @rmtoll
  *  CR3          DMAR          LL_LPUART_IsEnabledDMAReq_RX
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDMAReq_RX(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_DMAR) == (USART_CR3_DMAR)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA mode for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_LPUART_EnableDMAReq_TX
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDMAReq_TX(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_SET_BIT_32(p_lpuart->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Disable DMA mode for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_LPUART_DisableDMAReq_TX
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDMAReq_TX(USART_TypeDef *p_lpuart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_lpuart->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Check if DMA mode is enabled for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_LPUART_IsEnabledDMAReq_TX
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDMAReq_TX(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_DMAT) == (USART_CR3_DMAT)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA disabling on reception error.
  * @rmtoll
  *  CR3          DDRE          LL_LPUART_EnableDMADeactOnRxErr
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_EnableDMADeactOnRxErr(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->CR3, USART_CR3_DDRE);
}

/**
  * @brief  Disable DMA disabling on reception error.
  * @rmtoll
  *  CR3          DDRE          LL_LPUART_DisableDMADeactOnRxErr
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_DisableDMADeactOnRxErr(USART_TypeDef *p_lpuart)
{
  STM32_CLEAR_BIT(p_lpuart->CR3, USART_CR3_DDRE);
}

/**
  * @brief  Indicate if DMA disabling on reception error is disabled.
  * @rmtoll
  *  CR3          DDRE          LL_LPUART_IsEnabledDMADeactOnRxErr
  * @param  p_lpuart LPUART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPUART_IsEnabledDMADeactOnRxErr(const USART_TypeDef *p_lpuart)
{
  return ((STM32_READ_BIT(p_lpuart->CR3, USART_CR3_DDRE) == (USART_CR3_DDRE)) ? 1UL : 0UL);
}

/**
  * @brief  Get the LPUART data register address used for DMA transfer.
  * @rmtoll
  *  RDR          RDR           LL_LPUART_DMA_GetRegAddr \n
  *  TDR          TDR           LL_LPUART_DMA_GetRegAddr
  * @param  p_lpuart LPUART Instance
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_LPUART_DMA_REG_DATA_TRANSMIT
  *         @arg @ref LL_LPUART_DMA_REG_DATA_RECEIVE
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_LPUART_DMA_GetRegAddr(const USART_TypeDef *p_lpuart, uint32_t direction)
{
  uint32_t data_reg_addr;

  if (direction == LL_LPUART_DMA_REG_DATA_TRANSMIT)
  {
    /* Return address of TDR register. */
    data_reg_addr = (uint32_t) &(p_lpuart->TDR);
  }
  else
  {
    /* Return address of RDR register. */
    data_reg_addr = (uint32_t) &(p_lpuart->RDR);
  }

  return data_reg_addr;
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read receiver data register (receive data value, 8 bits).
  * @rmtoll
  *  RDR          RDR           LL_LPUART_ReceiveData8
  * @param  p_lpuart LPUART Instance
  * @retval Time value between Min_Data=0x00 and Max_Data=0xFF.
  */
__STATIC_INLINE uint8_t LL_LPUART_ReceiveData8(const USART_TypeDef *p_lpuart)
{
  return (uint8_t)(STM32_READ_BIT(p_lpuart->RDR, USART_RDR_RDR) & 0xFFU);
}

/**
  * @brief  Read receiver data register (receive data value, 9 bits).
  * @rmtoll
  *  RDR          RDR           LL_LPUART_ReceiveData9
  * @param  p_lpuart LPUART Instance
  * @retval Time value between Min_Data=0x00 and Max_Data=0x1FF.
  */
__STATIC_INLINE uint16_t LL_LPUART_ReceiveData9(const USART_TypeDef *p_lpuart)
{
  return (uint16_t)(STM32_READ_BIT(p_lpuart->RDR, USART_RDR_RDR));
}

/**
  * @brief  Write in transmitter data register (transmit data value, 8 bits).
  * @rmtoll
  *  TDR          TDR           LL_LPUART_TransmitData8
  * @param  p_lpuart LPUART Instance
  * @param  value Value between Min_Data=0x00 and Max_Data=0xFF.
  */
__STATIC_INLINE void LL_LPUART_TransmitData8(USART_TypeDef *p_lpuart, uint8_t value)
{
  p_lpuart->TDR = value;
}

/**
  * @brief  Write in transmitter data register (transmit data value, 9 bits).
  * @rmtoll
  *  TDR          TDR           LL_LPUART_TransmitData9
  * @param  p_lpuart LPUART Instance
  * @param  value Value between Min_Data=0x00 and Max_Data=0x1FF.
  */
__STATIC_INLINE void LL_LPUART_TransmitData9(USART_TypeDef *p_lpuart, uint16_t value)
{
  p_lpuart->TDR = value & 0x1FFUL;
}

/**
  * @}
  */

/** @defgroup LPUART_LL_EF_Execution Execution
  * @{
  */
/**
  * @brief  Set a request.
  * @rmtoll
  *  RQR          SBKRQ         LL_LPUART_SetRequest \n
  *  RQR          MMRQ          LL_LPUART_SetRequest \n
  *  RQR          RXFRQ         LL_LPUART_SetRequest \n
  *  RQR          TXFRQ         LL_LPUART_SetRequest
  * @param  p_lpuart LPUART Instance
  * @param  request Request to set
  *         @arg @ref LL_LPUART_REQUEST_SEND_BREAK
  *         @arg @ref LL_LPUART_REQUEST_MUTE_MODE
  *         @arg @ref LL_LPUART_REQUEST_RXDATA_FLUSH
  *         @arg @ref LL_LPUART_REQUEST_TXDATA_FLUSH
  */
__STATIC_INLINE void LL_LPUART_SetRequest(USART_TypeDef *p_lpuart, uint16_t request)
{
  STM32_SET_BIT(p_lpuart->RQR, request);
}

/**
  * @brief  Request break sending.
  * @rmtoll
  *  RQR          SBKRQ         LL_LPUART_RequestBreakSending
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_RequestBreakSending(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->RQR, (uint16_t)USART_RQR_SBKRQ);
}

/**
  * @brief  Put LPUART in mute mode and set the RWU flag.
  * @rmtoll
  *  RQR          MMRQ          LL_LPUART_RequestEnterMuteMode
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_RequestEnterMuteMode(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->RQR, (uint16_t)USART_RQR_MMRQ);
}

/**
  * @brief  Request a receive data and FIFO flush.
  * @rmtoll
  *  RQR          RXFRQ         LL_LPUART_RequestRxDataFlush
  * @param  p_lpuart LPUART Instance
  * @note   Allows you to discard the received data without reading them and avoid an overrun
  *         condition.
  */
__STATIC_INLINE void LL_LPUART_RequestRxDataFlush(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->RQR, (uint16_t)USART_RQR_RXFRQ);
}

/**
  * @brief  Request a transmit data and FIFO flush.
  * @rmtoll
  *  RQR          TXFRQ         LL_LPUART_RequestTxDataFlush
  * @param  p_lpuart LPUART Instance
  */
__STATIC_INLINE void LL_LPUART_RequestTxDataFlush(USART_TypeDef *p_lpuart)
{
  STM32_SET_BIT(p_lpuart->RQR, (uint16_t)USART_RQR_TXFRQ);
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

#endif /* LPUART1 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_LPUART_H */

