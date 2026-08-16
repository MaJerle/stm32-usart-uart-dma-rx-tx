/**
  ******************************************************************************
  * @file    stm32c5xx_ll_usart.h
  * @brief   Header file of USART LL module.
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
#ifndef STM32C5XX_LL_USART_H
#define STM32C5XX_LL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) \
 || defined(UART7)

/** @defgroup USART_LL USART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup USART_LL_Private_Variables USART Private Variables
  * @{
  */
/* Array used to get the USART prescaler division decimal values versus @ref USART_LL_EC_PRESCALER values */
static const uint32_t USART_PRESCALER_TAB[16] =
{
  1UL,
  2UL,
  4UL,
  6UL,
  8UL,
  10UL,
  12UL,
  16UL,
  32UL,
  64UL,
  128UL,
  256UL,
  256UL,
  256UL,
  256UL,
  256UL,
};
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup USART_LL_Private_Constants USART Private Constants
  * @{
  */
#define LL_USART_TRIG_MASK           0x10000000U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup USART_LL_Exported_Constants LL USART Constants
  * @{
  */

/** @defgroup USART_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_USART_WRITE_REG function.
  * @{
  */
#define LL_USART_ICR_PECF            USART_ICR_PECF                       /*!< Parity error clear flag */
#define LL_USART_ICR_FECF            USART_ICR_FECF                       /*!< Framing error clear flag */
#define LL_USART_ICR_NECF            USART_ICR_NECF                       /*!< Noise error detected clear flag */
#define LL_USART_ICR_ORECF           USART_ICR_ORECF                      /*!< Overrun error clear flag */
#define LL_USART_ICR_IDLECF          USART_ICR_IDLECF                     /*!< Idle line detected clear flag */
#define LL_USART_ICR_TXFECF          USART_ICR_TXFECF                     /*!< TX FIFO Empty clear flag */
#define LL_USART_ICR_TCCF            USART_ICR_TCCF                       /*!< Transmission complete clear flag */
#define LL_USART_ICR_TCBGTCF         USART_ICR_TCBGTCF                    /*!< Transmission completed before guard time clear flag */
#define LL_USART_ICR_LBDCF           USART_ICR_LBDCF                      /*!< LIN break detection clear flag */
#define LL_USART_ICR_CTSCF           USART_ICR_CTSCF                      /*!< CTS clear flag */
#define LL_USART_ICR_RTOCF           USART_ICR_RTOCF                      /*!< Receiver timeout clear flag */
#define LL_USART_ICR_EOBCF           USART_ICR_EOBCF                      /*!< End of block clear flag */
#define LL_USART_ICR_UDRCF           USART_ICR_UDRCF                      /*!< SPI Slave Underrun clear flag */
#define LL_USART_ICR_CMCF            USART_ICR_CMCF                       /*!< Character match clear flag */
#define LL_USART_ICR_WUCF            USART_ICR_WUCF                       /*!< Wakeup from Stop mode clear flag */
/**
  * @}
  */

/** @defgroup USART_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_USART_READ_REG function.
  * @{
  */
#define LL_USART_ISR_PE              USART_ISR_PE                         /*!< Parity error flag */
#define LL_USART_ISR_FE              USART_ISR_FE                         /*!< Framing error flag */
#define LL_USART_ISR_NE              USART_ISR_NE                         /*!< Noise detected flag */
#define LL_USART_ISR_ORE             USART_ISR_ORE                        /*!< Overrun error flag */
#define LL_USART_ISR_IDLE            USART_ISR_IDLE                       /*!< Idle line detected flag */
#define LL_USART_ISR_RXNE_RXFNE      USART_ISR_RXNE_RXFNE                 /*!< Read data register or RX FIFO not empty flag */
#define LL_USART_ISR_TC              USART_ISR_TC                         /*!< Transmission complete flag */
#define LL_USART_ISR_TXE_TXFNF       USART_ISR_TXE_TXFNF                  /*!< Transmit data register empty or TX FIFO Not Full flag*/
#define LL_USART_ISR_LBDF            USART_ISR_LBDF                       /*!< LIN break detection flag */
#define LL_USART_ISR_CTSIF           USART_ISR_CTSIF                      /*!< CTS interrupt flag */
#define LL_USART_ISR_CTS             USART_ISR_CTS                        /*!< CTS flag */
#define LL_USART_ISR_RTOF            USART_ISR_RTOF                       /*!< Receiver timeout flag */
#define LL_USART_ISR_EOBF            USART_ISR_EOBF                       /*!< End of block flag */
#define LL_USART_ISR_UDR             USART_ISR_UDR                        /*!< SPI Slave underrun error flag */
#define LL_USART_ISR_ABRE            USART_ISR_ABRE                       /*!< Auto baud rate error flag */
#define LL_USART_ISR_ABRF            USART_ISR_ABRF                       /*!< Auto baud rate flag */
#define LL_USART_ISR_BUSY            USART_ISR_BUSY                       /*!< Busy flag */
#define LL_USART_ISR_CMF             USART_ISR_CMF                        /*!< Character match flag */
#define LL_USART_ISR_SBKF            USART_ISR_SBKF                       /*!< Send break flag */
#define LL_USART_ISR_RWU             USART_ISR_RWU                        /*!< Receiver wakeup from Mute mode flag */
#define LL_USART_ISR_WUF             USART_ISR_WUF                        /*!< Wakeup from Stop mode flag */
#define LL_USART_ISR_TEACK           USART_ISR_TEACK                      /*!< Transmit enable acknowledge flag */
#define LL_USART_ISR_REACK           USART_ISR_REACK                      /*!< Receive enable acknowledge flag */
#define LL_USART_ISR_TXFE            USART_ISR_TXFE                       /*!< TX FIFO empty flag */
#define LL_USART_ISR_RXFF            USART_ISR_RXFF                       /*!< RX FIFO full flag */
#define LL_USART_ISR_TCBGT           USART_ISR_TCBGT                      /*!< Transmission complete before guard time completion flag */
#define LL_USART_ISR_RXFT            USART_ISR_RXFT                       /*!< RX FIFO threshold flag */
#define LL_USART_ISR_TXFT            USART_ISR_TXFT                       /*!< TX FIFO threshold flag */
/**
  * @}
  */

/** @defgroup USART_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_USART_READ_REG and  LL_USART_WRITE_REG functions.
  * @{
  */
#define LL_USART_CR1_IDLEIE          USART_CR1_IDLEIE                     /*!< IDLE interrupt enable */
#define LL_USART_CR1_RXNEIE_RXFNEIE  USART_CR1_RXNEIE_RXFNEIE             /*!< Read data register and RXFIFO not empty interrupt enable */
#define LL_USART_CR1_TCIE            USART_CR1_TCIE                       /*!< Transmission complete interrupt enable */
#define LL_USART_CR1_TXEIE_TXFNFIE   USART_CR1_TXEIE_TXFNFIE              /*!< Transmit data register empty and TX FIFO not full interrupt enable */
#define LL_USART_CR1_PEIE            USART_CR1_PEIE                       /*!< Parity error interrupt enable */
#define LL_USART_CR1_CMIE            USART_CR1_CMIE                       /*!< Character match interrupt enable */
#define LL_USART_CR1_RTOIE           USART_CR1_RTOIE                      /*!< Receiver timeout interrupt enable */
#define LL_USART_CR1_EOBIE           USART_CR1_EOBIE                      /*!< End of Block interrupt enable */
#define LL_USART_CR1_TXFEIE          USART_CR1_TXFEIE                     /*!< TX FIFO empty interrupt enable */
#define LL_USART_CR1_RXFFIE          USART_CR1_RXFFIE                     /*!< RX FIFO full interrupt enable */
#define LL_USART_CR2_LBDIE           USART_CR2_LBDIE                      /*!< LIN break detection interrupt enable */
#define LL_USART_CR3_EIE             USART_CR3_EIE                        /*!< Error interrupt enable */
#define LL_USART_CR3_CTSIE           USART_CR3_CTSIE                      /*!< CTS interrupt enable */
#define LL_USART_CR3_WUFIE           USART_CR3_WUFIE                      /*!< Wakeup from Stop mode interrupt enable */
#define LL_USART_CR3_TXFTIE          USART_CR3_TXFTIE                     /*!< TX FIFO threshold interrupt enable */
#define LL_USART_CR3_TCBGTIE         USART_CR3_TCBGTIE                    /*!< Transmission complete before guard time interrupt enable */
#define LL_USART_CR3_RXFTIE          USART_CR3_RXFTIE                     /*!< RX FIFO threshold interrupt enable */
/**
  * @}
  */

/** @defgroup USART_LL_EC_FIFOTHRESHOLD FIFO Threshold
  * @{
  */
#define LL_USART_FIFO_THRESHOLD_1_8        0x00000000U                    /*!< FIFO reaches 1/8 of its depth */
#define LL_USART_FIFO_THRESHOLD_1_4        0x00000001U                    /*!< FIFO reaches 1/4 of its depth */
#define LL_USART_FIFO_THRESHOLD_1_2        0x00000002U                    /*!< FIFO reaches 1/2 of its depth */
#define LL_USART_FIFO_THRESHOLD_3_4        0x00000003U                    /*!< FIFO reaches 3/4 of its depth */
#define LL_USART_FIFO_THRESHOLD_7_8        0x00000004U                    /*!< FIFO reaches 7/8 of its depth */
#define LL_USART_FIFO_THRESHOLD_8_8        0x00000005U                    /*!< FIFO becomes empty for TX and full for RX */
/**
  * @}
  */

/** @defgroup USART_LL_EC_DIRECTION Communication Direction
  * @{
  */
#define LL_USART_DIRECTION_NONE           0x00000000U                     /*!< Transmitter and Receiver are disabled */
#define LL_USART_DIRECTION_RX             USART_CR1_RE                    /*!< Transmitter is disabled and Receiver is enabled */
#define LL_USART_DIRECTION_TX             USART_CR1_TE                    /*!< Transmitter is enabled and Receiver is disabled */
#define LL_USART_DIRECTION_TX_RX          (USART_CR1_TE |USART_CR1_RE)    /*!< Transmitter and Receiver are enabled */
/**
  * @}
  */

/** @defgroup USART_LL_EC_PARITY Parity Control
  * @{
  */
#define LL_USART_PARITY_NONE              0x00000000U                     /*!< Parity control disabled */
#define LL_USART_PARITY_EVEN              USART_CR1_PCE                   /*!< Parity control enabled and Even Parity is selected */
#define LL_USART_PARITY_ODD               (USART_CR1_PCE | USART_CR1_PS)  /*!< Parity control enabled and Odd Parity is selected */
/**
  * @}
  */

/** @defgroup USART_LL_EC_WAKEUP Wakeup
  * @{
  */
#define LL_USART_WAKEUP_METHOD_IDLE_LINE          0x00000000U             /*!<  USART wake up from Mute mode on Idle Line */
#define LL_USART_WAKEUP_METHOD_ADDRESS_MARK       USART_CR1_WAKE          /*!<  USART wake up from Mute mode on Address Mark */
/**
  * @}
  */

/** @defgroup USART_LL_EC_DATAWIDTH Datawidth
  * @{
  */
#define LL_USART_DATAWIDTH_7_BIT          USART_CR1_M1                    /*!< 7 bit word length : Start bit, 7 data bits, n stop bits */
#define LL_USART_DATAWIDTH_8_BIT          0x00000000U                     /*!< 8 bit word length : Start bit, 8 data bits, n stop bits */
#define LL_USART_DATAWIDTH_9_BIT          USART_CR1_M0                    /*!< 9 bit word length : Start bit, 9 data bits, n stop bits */
/**
  * @}
  */

/** @defgroup USART_LL_EC_OVERSAMPLING Oversampling
  * @{
  */
#define LL_USART_OVERSAMPLING_16          0x00000000U                     /*!< Oversampling by 16 */
#define LL_USART_OVERSAMPLING_8           USART_CR1_OVER8                 /*!< Oversampling by 8 */
/**
  * @}
  */

/** @defgroup USART_LL_EC_LASTCLKPULSE Last Clock Pulse
  * @{
  */
#define LL_USART_LASTCLKPULSE_DISABLED    0x00000000U                     /*!< The clock pulse of the last data bit is not output to the SCLK pin */
#define LL_USART_LASTCLKPULSE_ENABLED     USART_CR2_LBCL                  /*!< The clock pulse of the last data bit is output to the SCLK pin */
/**
  * @}
  */

/** @defgroup USART_LL_EC_CLOCKOUTPUT Clock Output
  * @{
  */
#define LL_USART_CLOCK_OUTPUT_DISABLED    0x00000000U                     /*!< The clock signal output is disabled */
#define LL_USART_CLOCK_OUTPUT_ENABLED     USART_CR2_CLKEN                 /*!< The clock signal output is enabled */
/**
  * @}
  */

/** @defgroup USART_LL_EC_PHASE Clock Phase
  * @{
  */
#define LL_USART_CLOCK_PHASE_1_EDGE       0x00000000U                     /*!< The first clock transition is the first data capture edge */
#define LL_USART_CLOCK_PHASE_2_EDGE       USART_CR2_CPHA                  /*!< The second clock transition is the first data capture edge */
/**
  * @}
  */

/** @defgroup USART_LL_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_USART_CLOCK_POLARITY_LOW       0x00000000U                     /*!< Steady low value on SCLK pin outside transmission window*/
#define LL_USART_CLOCK_POLARITY_HIGH      USART_CR2_CPOL                  /*!< Steady high value on SCLK pin outside transmission window */
/**
  * @}
  */

/** @defgroup USART_LL_EC_PRESCALER Clock Source Prescaler
  * @{
  */
#define LL_USART_PRESCALER_DIV1           0x00000000U                                                                   /*!< Input clock not divided   */
#define LL_USART_PRESCALER_DIV2           (USART_PRESC_PRESCALER_0)                                                     /*!< Input clock divided by 2  */
#define LL_USART_PRESCALER_DIV4           (USART_PRESC_PRESCALER_1)                                                     /*!< Input clock divided by 4  */
#define LL_USART_PRESCALER_DIV6           (USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_0)                           /*!< Input clock divided by 6  */
#define LL_USART_PRESCALER_DIV8           (USART_PRESC_PRESCALER_2)                                                     /*!< Input clock divided by 8  */
#define LL_USART_PRESCALER_DIV10          (USART_PRESC_PRESCALER_2 | USART_PRESC_PRESCALER_0)                           /*!< Input clock divided by 10 */
#define LL_USART_PRESCALER_DIV12          (USART_PRESC_PRESCALER_2 | USART_PRESC_PRESCALER_1)                           /*!< Input clock divided by 12 */
#define LL_USART_PRESCALER_DIV16          (USART_PRESC_PRESCALER_2 | USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_0) /*!< Input clock divided by 16 */
#define LL_USART_PRESCALER_DIV32          (USART_PRESC_PRESCALER_3)                                                     /*!< Input clock divided by 32 */
#define LL_USART_PRESCALER_DIV64          (USART_PRESC_PRESCALER_3 | USART_PRESC_PRESCALER_0)                           /*!< Input clock divided by 64 */
#define LL_USART_PRESCALER_DIV128         (USART_PRESC_PRESCALER_3 | USART_PRESC_PRESCALER_1)                           /*!< Input clock divided by 128 */
#define LL_USART_PRESCALER_DIV256         (USART_PRESC_PRESCALER_3 | USART_PRESC_PRESCALER_1 | USART_PRESC_PRESCALER_0) /*!< Input clock divided by 256 */
/**
  * @}
  */


/** @defgroup SMARTCARD_LL_EC_PRESCALER SMARTCARD Clock Prescaler
  * @{
  */
#define LL_USART_SMARTCARD_PRESCALER_DIV2           (0x00000001U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /2  */
#define LL_USART_SMARTCARD_PRESCALER_DIV4           (0x00000002U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /4  */
#define LL_USART_SMARTCARD_PRESCALER_DIV6           (0x00000003U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /6  */
#define LL_USART_SMARTCARD_PRESCALER_DIV8           (0x00000004U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /8  */
#define LL_USART_SMARTCARD_PRESCALER_DIV10          (0x00000005U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /10 */
#define LL_USART_SMARTCARD_PRESCALER_DIV12          (0x00000006U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /12 */
#define LL_USART_SMARTCARD_PRESCALER_DIV14          (0x00000007U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /14 */
#define LL_USART_SMARTCARD_PRESCALER_DIV16          (0x00000008U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /16 */
#define LL_USART_SMARTCARD_PRESCALER_DIV18          (0x00000009U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /18 */
#define LL_USART_SMARTCARD_PRESCALER_DIV20          (0x00000010U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /20 */
#define LL_USART_SMARTCARD_PRESCALER_DIV22          (0x00000011U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /22 */
#define LL_USART_SMARTCARD_PRESCALER_DIV24          (0x00000012U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /24 */
#define LL_USART_SMARTCARD_PRESCALER_DIV26          (0x00000013U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /26 */
#define LL_USART_SMARTCARD_PRESCALER_DIV28          (0x00000014U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /28 */
#define LL_USART_SMARTCARD_PRESCALER_DIV30          (0x00000015U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /30 */
#define LL_USART_SMARTCARD_PRESCALER_DIV32          (0x00000016U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /32 */
#define LL_USART_SMARTCARD_PRESCALER_DIV34          (0x00000017U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /34 */
#define LL_USART_SMARTCARD_PRESCALER_DIV36          (0x00000018U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /36 */
#define LL_USART_SMARTCARD_PRESCALER_DIV38          (0x00000019U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /38 */
#define LL_USART_SMARTCARD_PRESCALER_DIV40          (0x00000020U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /40 */
#define LL_USART_SMARTCARD_PRESCALER_DIV42          (0x00000021U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /42 */
#define LL_USART_SMARTCARD_PRESCALER_DIV44          (0x00000022U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /44 */
#define LL_USART_SMARTCARD_PRESCALER_DIV46          (0x00000023U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /46 */
#define LL_USART_SMARTCARD_PRESCALER_DIV48          (0x00000024U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /48 */
#define LL_USART_SMARTCARD_PRESCALER_DIV50          (0x00000025U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /50 */
#define LL_USART_SMARTCARD_PRESCALER_DIV52          (0x00000026U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /52 */
#define LL_USART_SMARTCARD_PRESCALER_DIV54          (0x00000027U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /54 */
#define LL_USART_SMARTCARD_PRESCALER_DIV56          (0x00000028U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /56 */
#define LL_USART_SMARTCARD_PRESCALER_DIV58          (0x00000029U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /58 */
#define LL_USART_SMARTCARD_PRESCALER_DIV60          (0x00000030U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /60 */
#define LL_USART_SMARTCARD_PRESCALER_DIV62          (0x00000031U << USART_GTPR_PSC_Pos) /*!< SMARTCARD Output CLK /62 */
/**
  * @}
  */

/** @defgroup USART_LL_EC_STOPBITS Stop Bits
  * @{
  */
#define LL_USART_STOP_BIT_0_5             USART_CR2_STOP_0                               /*!< 0.5 stop bit */
#define LL_USART_STOP_BIT_1               0x00000000U                                    /*!< 1 stop bit */
#define LL_USART_STOP_BIT_1_5             (USART_CR2_STOP_0 | USART_CR2_STOP_1)          /*!< 1.5 stop bits */
#define LL_USART_STOP_BIT_2               USART_CR2_STOP_1                               /*!< 2 stop bits */
/**
  * @}
  */

/** @defgroup USART_LL_EC_TXRX TX RX Pins Swap
  * @{
  */
#define LL_USART_TXRX_STANDARD            0x00000000U                     /*!< TX and RX pins are used as defined in standard pinout */
#define LL_USART_TXRX_SWAPPED             (USART_CR2_SWAP)                /*!< TX and RX pins functions are swapped.             */
/**
  * @}
  */

/** @defgroup USART_LL_EC_RXPIN_LEVEL RX Pin Active Level Inversion
  * @{
  */
#define LL_USART_RXPIN_LEVEL_STANDARD     0x00000000U                     /*!< RX pin signal works using the standard logic levels */
#define LL_USART_RXPIN_LEVEL_INVERTED     (USART_CR2_RXINV)               /*!< RX pin signal values are inverted.                  */
/**
  * @}
  */

/** @defgroup USART_LL_EC_TXPIN_LEVEL TX Pin Active Level Inversion
  * @{
  */
#define LL_USART_TXPIN_LEVEL_STANDARD     0x00000000U                     /*!< TX pin signal works using the standard logic levels */
#define LL_USART_TXPIN_LEVEL_INVERTED     (USART_CR2_TXINV)               /*!< TX pin signal values are inverted.                  */
/**
  * @}
  */

/** @defgroup USART_LL_EC_BINARY_LOGIC Binary Data Inversion
  * @{
  */
#define LL_USART_BINARY_LOGIC_POSITIVE    0x00000000U                     /*!< Logical data from the data register are send/received in positive/direct logic. (1=H, 0=L) */
#define LL_USART_BINARY_LOGIC_NEGATIVE    USART_CR2_DATAINV               /*!< Logical data from the data register are send/received in negative/inverse logic. (1=L, 0=H). The parity bit is also inverted. */
/**
  * @}
  */

/** @defgroup USART_LL_EC_BITORDER Bit Order
  * @{
  */
#define LL_USART_BITORDER_LSB_FIRST        0x00000000U                     /*!< data is transmitted/received with data bit 0 first, following the start bit */
#define LL_USART_BITORDER_MSB_FIRST        USART_CR2_MSBFIRST              /*!< data is transmitted/received with the MSB first, following the start bit */
/**
  * @}
  */

/** @defgroup USART_LL_EC_AUTOBAUD_DETECT_ON Autobaud Detection
  * @{
  */
#define LL_USART_AUTO_BAUD_DETECT_ON_START_BIT     0x00000000U                                 /*!< Measurement of the start bit is used to detect the baud rate */
#define LL_USART_AUTO_BAUD_DETECT_ON_FALLING_EDGE  USART_CR2_ABRMOD_0                         /*!< Falling edge to falling edge measurement. Received frame must start with a single bit = 1 -> Frame = Start10xxxxxx */
#define LL_USART_AUTO_BAUD_DETECT_ON_0X7F_FRAME    USART_CR2_ABRMOD_1                         /*!< 0x7F frame detection */
#define LL_USART_AUTO_BAUD_DETECT_ON_0X55_FRAME    (USART_CR2_ABRMOD_1 | USART_CR2_ABRMOD_0) /*!< 0x55 frame detection */
/**
  * @}
  */

/** @defgroup USART_LL_EC_ADDRESS_DETECT Address Length Detection
  * @{
  */
#define LL_USART_ADDRESS_DETECT_4_BIT        0x00000000U                  /*!< 4-bit address detection method selected */
#define LL_USART_ADDRESS_DETECT_7_BIT        USART_CR2_ADDM7              /*!< 7-bit address detection (in 8-bit data mode) method selected */
/**
  * @}
  */

/** @defgroup USART_LL_EC_HWCONTROL Hardware Control
  * @{
  */
#define LL_USART_HWCONTROL_NONE           0x00000000U                             /*!< CTS and RTS hardware flow control disabled */
#define LL_USART_HWCONTROL_RTS            USART_CR3_RTSE                          /*!< RTS output enabled, data is only requested when there is space in the receive buffer */
#define LL_USART_HWCONTROL_CTS            USART_CR3_CTSE                          /*!< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
#define LL_USART_HWCONTROL_RTS_CTS        (USART_CR3_RTSE | USART_CR3_CTSE)       /*!< CTS and RTS hardware flow control enabled */
/**
  * @}
  */

/** @defgroup USART_LL_EC_WAKEUP_ON Wakeup Activation
  * @{
  */
#define LL_USART_WAKEUP_ON_ADDRESS        0x00000000U                             /*!< Wake up active on address match */
#define LL_USART_WAKEUP_ON_STARTBIT       USART_CR3_WUS_1                         /*!< Wake up active on Start bit detection */
#define LL_USART_WAKEUP_ON_RXNE           (USART_CR3_WUS_0 | USART_CR3_WUS_1)     /*!< Wake up active on RXNE */
/**
  * @}
  */

/** @defgroup USART_LL_EC_IRDA_POWER IrDA Power
  * @{
  */
#define LL_USART_IRDA_POWER_MODE_NORMAL       0x00000000U                     /*!< IrDA normal power mode */
#define LL_USART_IRDA_POWER_MODE_LOW          USART_CR3_IRLP                  /*!< IrDA low power mode */
/**
  * @}
  */

/** @defgroup USART_LL_EC_LINBREAK_DETECT LIN Break Detection Length
  * @{
  */
#define LL_USART_LIN_BREAK_DETECT_10_BIT      0x00000000U                 /*!< 10-bit break detection method selected */
#define LL_USART_LIN_BREAK_DETECT_11_BIT      USART_CR2_LBDL              /*!< 11-bit break detection method selected */
/**
  * @}
  */

/** @defgroup USART_LL_EC_DE_POLARITY Driver Enable Polarity
  * @{
  */
#define LL_USART_DE_POLARITY_HIGH         0x00000000U                     /*!< DE signal is active high */
#define LL_USART_DE_POLARITY_LOW          USART_CR3_DEP                   /*!< DE signal is active low */
/**
  * @}
  */

/** @defgroup USART_LL_EC_DMA_REG_DATA DMA Register Data
  * @{
  */
#define LL_USART_DMA_REG_DATA_TRANSMIT    0x00000000U                     /*!< Get address of data register used for transmission */
#define LL_USART_DMA_REG_DATA_RECEIVE     0x00000001U                     /*!< Get address of data register used for reception */
/**
  * @}
  */


/** @defgroup USART_LL_EC_Request Request
  * @brief    USART Request.
  * @{
  */
#define LL_USART_REQUEST_SEND_BREAK       USART_RQR_SBKRQ                 /*!< Send Break Request           */
#define LL_USART_REQUEST_MUTE_MODE        USART_RQR_MMRQ                  /*!< Mute Mode Request            */
#define LL_USART_REQUEST_RX_DATA_FLUSH    USART_RQR_RXFRQ                 /*!< Receive Data flush Request   */
#define LL_USART_REQUEST_TX_DATA_FLUSH    USART_RQR_TXFRQ                 /*!< Transmit Data flush Request  */
#define LL_USART_REQUEST_AUTO_BAUD_RATE   USART_RQR_ABRRQ                 /*!< Auto Baud Rate Request       */
/**
  * @}
  */

/** @defgroup USART_LL_EC_One_Bit_Sampling One Bit Sampling Enable
  * @brief    USART One Bit Sampling Enable.
  * @{
  */
#define LL_USART_ONE_BIT_SAMPLE_DISABLE   0x00000000U
#define LL_USART_ONE_BIT_SAMPLE_ENABLE    USART_CR3_ONEBIT

/**
  * @}
  */

/** @defgroup USART_LL_EC_Interruption_Mask    LL_USART Interruptions Flag Mask
  * @{
  */
#define LL_USART_IT_MASK                  0x001FU                         /*!< LL_USART interruptions flags mask */
/**
  * @}
  */

/** @defgroup USART_LL_EC_Slave_Select Slave Select
  * @brief    USART Slave Select.
  * @{
  */
#define LL_USART_NSS_IGNORED              USART_CR2_DIS_NSS
#define LL_USART_NSS_USED                 0x00000000U
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USART_LL_Exported_Macros LL USART Macros
  * @{
  */

/** @defgroup USART_LL_EM_WRITE_READ Common register Write and Read Macros
  * @{
  */

/**
  * @brief  Write a value in USART register.
  * @param  instance USART Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_USART_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in USART register.
  * @param  instance USART Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_USART_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/** @defgroup USART_LL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 8 bits sampling mode (32 bits value of USARTDIV is returned).
  * @param  periph_clock Peripheral Clock frequency used for USART instance
  * @param  prescaler This parameter can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  * @param  baudrate Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_8 case
  */
#define LL_USART_DIV_SAMPLING8(periph_clock, prescaler, baudrate) \
  (((((periph_clock)/(USART_PRESCALER_TAB[(prescaler)]))*2U)/(baudrate)) + 1U)

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 16 bits sampling mode (32 bits value of USARTDIV is returned).
  * @param  periph_clock Peripheral Clock frequency used for USART instance
  * @param  prescaler This parameter can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  * @param  baudrate Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_16 case
  */
#define LL_USART_DIV_SAMPLING16(periph_clock, prescaler, baudrate) \
  ((((periph_clock)/(USART_PRESCALER_TAB[(prescaler)]))\
    + ((baudrate)/2U))/(baudrate))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup USART_LL_Exported_Functions LL USART Functions
  * @{
  */

/** @defgroup USART_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  USART Enable.
  * @rmtoll
  *  CR1          UE            LL_USART_Enable
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_Enable(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR1, USART_CR1_UE);
}

/**
  * @brief  USART Disable (all USART prescalers and outputs are disabled).
  * @rmtoll
  *  CR1          UE            LL_USART_Disable
  * @param  p_usart USART Instance
  * @note   When USART is disabled, USART prescalers and outputs are stopped immediately,
  *         and current operations are discarded. The configuration of the USART is kept, but all the status
  *         flags, in the p_usart ISR register are set to their default values.
  */
__STATIC_INLINE void LL_USART_Disable(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR1, USART_CR1_UE);
}

/**
  * @brief  Indicate if USART is enabled.
  * @rmtoll
  *  CR1          UE            LL_USART_IsEnabled
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabled(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_UE) == (USART_CR1_UE)) ? 1UL : 0UL);
}

/**
  * @brief  FIFO Mode Enable.
  * @rmtoll
  *  CR1          FIFOEN        LL_USART_EnableFIFO
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableFIFO(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR1, USART_CR1_FIFOEN);
}

/**
  * @brief  FIFO Mode Disable.
  * @rmtoll
  *  CR1          FIFOEN        LL_USART_DisableFIFO
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableFIFO(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR1, USART_CR1_FIFOEN);
}

/**
  * @brief  Indicate if FIFO Mode is enabled.
  * @rmtoll
  *  CR1          FIFOEN        LL_USART_IsEnabledFIFO
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledFIFO(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_FIFOEN) == (USART_CR1_FIFOEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure TX FIFO Threshold.
  * @rmtoll
  *  CR3          TXFTCFG       LL_USART_SetTXFIFOThreshold
  * @param  p_usart USART Instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetTXFIFOThreshold(USART_TypeDef *p_usart, uint32_t threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_usart->CR3, USART_CR3_TXFTCFG, threshold << USART_CR3_TXFTCFG_Pos);
}

/**
  * @brief  Return TX FIFO threshold configuration.
  * @rmtoll
  *  CR3          TXFTCFG       LL_USART_GetTXFIFOThreshold
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE uint32_t LL_USART_GetTXFIFOThreshold(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_TXFTCFG) >> USART_CR3_TXFTCFG_Pos);
}

/**
  * @brief  Configure RX FIFO Threshold.
  * @rmtoll
  *  CR3          RXFTCFG       LL_USART_SetRXFIFOThreshold
  * @param  p_usart USART Instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetRXFIFOThreshold(USART_TypeDef *p_usart, uint32_t threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_usart->CR3, USART_CR3_RXFTCFG, threshold << USART_CR3_RXFTCFG_Pos);
}

/**
  * @brief  Return RX FIFO threshold configuration.
  * @rmtoll
  *  CR3          RXFTCFG       LL_USART_GetRXFIFOThreshold
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  */
__STATIC_INLINE uint32_t LL_USART_GetRXFIFOThreshold(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_RXFTCFG) >> USART_CR3_RXFTCFG_Pos);
}

/**
  * @brief  Configure TX and RX FIFO thresholds.
  * @rmtoll
  *  CR3          TXFTCFG       LL_USART_ConfigFIFOsThreshold \n
  *  CR3          RXFTCFG       LL_USART_ConfigFIFOsThreshold
  * @param  p_usart USART Instance
  * @param  tx_threshold This parameter can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  * @param  rx_threshold This parameter can be one of the following values:
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_1_2
  *         @arg @ref LL_USART_FIFO_THRESHOLD_3_4
  *         @arg @ref LL_USART_FIFO_THRESHOLD_7_8
  *         @arg @ref LL_USART_FIFO_THRESHOLD_8_8
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ConfigFIFOsThreshold(USART_TypeDef *p_usart, uint32_t tx_threshold, uint32_t rx_threshold)
{
  STM32_ATOMIC_MODIFY_REG_32(p_usart->CR3, USART_CR3_TXFTCFG | USART_CR3_RXFTCFG,
                             (tx_threshold << USART_CR3_TXFTCFG_Pos) | (rx_threshold << USART_CR3_RXFTCFG_Pos));
}

/**
  * @brief  USART enabled in STOP Mode.
  * @rmtoll
  *  CR1          UESM          LL_USART_EnableInStopMode
  * @param  p_usart USART Instance
  * @note   When this function is enabled, USART is able to wake up the MCU from Stop mode, provided that
  *         USART clock selection is HSI or LSE in RCC.
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableInStopMode(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_UESM);
}

/**
  * @brief  USART disabled in STOP Mode.
  * @rmtoll
  *  CR1          UESM          LL_USART_DisableInStopMode
  * @param  p_usart USART Instance
  * @note   When this function is disabled, USART is not able to wake up the MCU from Stop mode
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableInStopMode(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_UESM);
}

/**
  * @brief  Indicate if USART is enabled in STOP Mode (able to wake up MCU from Stop mode or not).
  * @rmtoll
  *  CR1          UESM          LL_USART_IsEnabledInStopMode
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledInStopMode(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_UESM) == (USART_CR1_UESM)) ? 1UL : 0UL);
}


/**
  * @brief  Configure the USART instance.
  * @rmtoll
  *  CR1         M0             LL_USART_ConfigXfer \n
  *  CR1         M1             LL_USART_ConfigXfer \n
  *  CR1         PCE            LL_USART_ConfigXfer \n
  *  CR1         PS             LL_USART_ConfigXfer \n
  *  CR1         TE             LL_USART_ConfigXfer \n
  *  CR1         RE             LL_USART_ConfigXfer \n
  *  CR1         OVER8          LL_USART_ConfigXfer \n
  *  CR2         STOP_0         LL_USART_ConfigXfer \n
  *  CR2         STOP_1         LL_USART_ConfigXfer \n
  *  CR2         CPOL           LL_USART_ConfigXfer \n
  *  CR2         ABRMODE_0      LL_USART_ConfigXfer \n
  *  CR2         LBLC           LL_USART_ConfigXfer
  * @param  p_usart USART Instance
  * @param cr1_config: This parameter must be a combination of the following groups:
  *         @arg @ref USART_LL_EC_DATAWIDTH
  *         @arg @ref USART_LL_EC_PARITY
  *         @arg @ref USART_LL_EC_DIRECTION
  *         @arg @ref USART_LL_EC_OVERSAMPLING
  * @param cr2_config: This parameter must be a combination of the following groups:
  *         @arg @ref USART_LL_EC_STOPBITS
  *         @arg @ref USART_LL_EC_PHASE
  *         @arg @ref USART_LL_EC_POLARITY
  *         @arg @ref USART_LL_EC_LASTCLKPULSE
  */
__STATIC_INLINE void LL_USART_ConfigXfer(USART_TypeDef *p_usart, uint32_t cr1_config, uint32_t cr2_config)
{
  STM32_MODIFY_REG(p_usart->CR1, (USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE
                                  | USART_CR1_RE | USART_CR1_OVER8), cr1_config);

  STM32_MODIFY_REG(p_usart->CR2, (USART_CR2_STOP_0 | USART_CR2_STOP_1 | USART_CR2_CPOL | USART_CR2_LBCL
                                  | USART_CR2_CPHA), cr2_config);
}

/**
  * @brief  Configure the USART instance for Smartcard.
  * @rmtoll
  *  CR1         M0             LL_USART_SetSmartcardConfig \n
  *  CR1         M1             LL_USART_SetSmartcardConfig \n
  *  CR1         PS             LL_USART_SetSmartcardConfig \n
  *  CR1         PCE            LL_USART_SetSmartcardConfig \n
  *  CR1         OVER8          LL_USART_SetSmartcardConfig \n
  *  CR2         MSBFIRST       LL_USART_SetSmartcardConfig \n
  *  CR2         STOP           LL_USART_SetSmartcardConfig \n
  *  CR3         SCEN           LL_USART_SetSmartcardConfig \n
  *  CR3         SCARCNT        LL_USART_SetSmartcardConfig \n
  *  CR3         NACK           LL_USART_SetSmartcardConfig
  * @param  p_usart USART Instance
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_USART_STOP_BIT_0_5
  *         @arg @ref LL_USART_STOP_BIT_1_5
  * @param  first_bit This parameter can be one of the following values:
  *         @arg @ref LL_USART_BITORDER_LSB_FIRST
  *         @arg @ref LL_USART_BITORDER_MSB_FIRST
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_USART_PARITY_NONE
  *         @arg @ref LL_USART_PARITY_EVEN
  *         @arg @ref LL_USART_PARITY_ODD
  * @param  nack This parameter can be 1U(Enable) or 0U(Disable):
  * @param  auto_retry_count Value between Min_Data=0 and Max_Data=7
  */
__STATIC_INLINE void LL_USART_SetSmartcardConfig(USART_TypeDef *p_usart, uint32_t stop_bits, uint32_t first_bit,
                                                 uint32_t parity, uint32_t nack, uint32_t auto_retry_count)
{
  STM32_MODIFY_REG(p_usart->CR1, (USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_PS | USART_CR1_OVER8),
                   (parity | LL_USART_DATAWIDTH_9_BIT | LL_USART_OVERSAMPLING_16 | USART_CR1_PCE));

  STM32_MODIFY_REG(p_usart->CR2, (USART_CR2_STOP | USART_CR2_MSBFIRST), (first_bit | stop_bits));
  STM32_MODIFY_REG(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_SCARCNT | USART_CR3_NACK),
                   ((nack << USART_CR3_NACK_Pos) | (auto_retry_count << USART_CR3_SCARCNT_Pos) | USART_CR3_SCEN));
}

/**
  * @brief  Configure the USART instance clock for Smartcard.
  * @rmtoll
  *  CR2         LBCL           LL_USART_SetSmartcardClockConfig \n
  *  CR2         CLKEN          LL_USART_SetSmartcardClockConfig \n
  *  CR2         CPHA           LL_USART_SetSmartcardClockConfig \n
  *  CR2         CPOL           LL_USART_SetSmartcardClockConfig
  * @param  p_usart USART Instance
  * @param  clock_output This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_OUTPUT_ENABLED
  *         @arg @ref LL_USART_CLOCK_OUTPUT_DISABLED
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_POLARITY_LOW
  *         @arg @ref LL_USART_CLOCK_POLARITY_HIGH
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_PHASE_1_EDGE
  *         @arg @ref LL_USART_CLOCK_PHASE_2_EDGE
  */
__STATIC_INLINE void LL_USART_SetSmartcardClockConfig(USART_TypeDef *p_usart, uint32_t clock_output,
                                                      uint32_t clock_polarity, uint32_t clock_phase)
{
  STM32_MODIFY_REG(p_usart->CR2, (USART_CR2_LBCL | USART_CR2_CLKEN | USART_CR2_CPHA | USART_CR2_CPOL),
                   (clock_output | clock_polarity | clock_phase | USART_CR2_LBCL));
}


/**
  * @brief  Receiver Enable (Receiver is enabled and begins searching for a start bit).
  * @rmtoll
  *  CR1          RE            LL_USART_EnableDirectionRx
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableDirectionRx(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_RE);
}

/**
  * @brief  Receiver Disable.
  * @rmtoll
  *  CR1          RE            LL_USART_DisableDirectionRx
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableDirectionRx(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_RE);
}

/**
  * @brief  Indicate if the p_usart Receiver is enabled.
  * @rmtoll
  *  CR1          RE            LL_USART_IsEnabledDirectionRx
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDirectionRx(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_RE) == (USART_CR1_RE));
}

/**
  * @brief  Transmitter Enable.
  * @rmtoll
  *  CR1          TE            LL_USART_EnableDirectionTx
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableDirectionTx(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_TE);
}

/**
  * @brief  Transmitter Disable.
  * @rmtoll
  *  CR1          TE            LL_USART_DisableDirectionTx
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableDirectionTx(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_TE);
}

/**
  * @brief  Indicate if the p_usart Transmitter is enabled.
  * @rmtoll
  *  CR1          TE            LL_USART_IsEnabledDirectionTx
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDirectionTx(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_TE) == (USART_CR1_TE));
}

/**
  * @brief  Configure simultaneously enabled/disabled states
  *         of Transmitter and Receiver.
  * @rmtoll
  *  CR1          RE            LL_USART_SetTransferDirection \n
  *  CR1          TE            LL_USART_SetTransferDirection
  * @param  p_usart USART Instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_USART_DIRECTION_NONE
  *         @arg @ref LL_USART_DIRECTION_RX
  *         @arg @ref LL_USART_DIRECTION_TX
  *         @arg @ref LL_USART_DIRECTION_TX_RX
  */
__STATIC_INLINE void LL_USART_SetTransferDirection(USART_TypeDef *p_usart, uint32_t transfer_direction)
{
  STM32_ATOMIC_MODIFY_REG_32(p_usart->CR1, USART_CR1_RE | USART_CR1_TE, transfer_direction);
}

/**
  * @brief  Return enabled/disabled states of Transmitter and Receiver.
  * @rmtoll
  *  CR1          RE            LL_USART_GetTransferDirection \n
  *  CR1          TE            LL_USART_GetTransferDirection
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_DIRECTION_NONE
  *         @arg @ref LL_USART_DIRECTION_RX
  *         @arg @ref LL_USART_DIRECTION_TX
  *         @arg @ref LL_USART_DIRECTION_TX_RX
  */
__STATIC_INLINE uint32_t LL_USART_GetTransferDirection(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_RE | USART_CR1_TE));
}

/**
  * @brief  Configure Parity (enabled/disabled and parity mode if enabled).osition
  *         (9th or 8th bit depending on data width) and parity is checked on the received data.
  * @rmtoll
  *  CR1          PS            LL_USART_SetParity \n
  *  CR1          PCE           LL_USART_SetParity
  * @param  p_usart USART Instance
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_USART_PARITY_NONE
  *         @arg @ref LL_USART_PARITY_EVEN
  *         @arg @ref LL_USART_PARITY_ODD
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd or Even), computed parity bit is inserted at the MSB p
  */
__STATIC_INLINE void LL_USART_SetParity(USART_TypeDef *p_usart, uint32_t parity)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_PS | USART_CR1_PCE, parity);
}

/**
  * @brief  Return Parity configuration (enabled/disabled and parity mode if enabled).
  * @rmtoll
  *  CR1          PS            LL_USART_GetParity \n
  *  CR1          PCE           LL_USART_GetParity
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_PARITY_NONE
  *         @arg @ref LL_USART_PARITY_EVEN
  *         @arg @ref LL_USART_PARITY_ODD
  */
__STATIC_INLINE uint32_t LL_USART_GetParity(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_PS | USART_CR1_PCE));
}

/**
  * @brief  Set Receiver Wake Up method from Mute mode.
  * @rmtoll
  *  CR1          WAKE          LL_USART_SetWakeUpMethod
  * @param  p_usart USART Instance
  * @param  method This parameter can be one of the following values:
  *         @arg @ref LL_USART_WAKEUP_METHOD_IDLE_LINE
  *         @arg @ref LL_USART_WAKEUP_METHOD_ADDRESS_MARK
  */
__STATIC_INLINE void LL_USART_SetWakeUpMethod(USART_TypeDef *p_usart, uint32_t method)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_WAKE, method);
}

/**
  * @brief  Return Receiver Wake Up method from Mute mode.
  * @rmtoll
  *  CR1          WAKE          LL_USART_GetWakeUpMethod
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_WAKEUP_METHOD_IDLE_LINE
  *         @arg @ref LL_USART_WAKEUP_METHOD_ADDRESS_MARK
  */
__STATIC_INLINE uint32_t LL_USART_GetWakeUpMethod(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_WAKE));
}

/**
  * @brief  Set Word length (e.g. nb of data bits, excluding start and stop bits).
  * @rmtoll
  *  CR1          M0            LL_USART_SetDataWidth \n
  *  CR1          M1            LL_USART_SetDataWidth
  * @param  p_usart USART Instance
  * @param  data_width This parameter can be one of the following values:
  *         @arg @ref LL_USART_DATAWIDTH_7_BIT
  *         @arg @ref LL_USART_DATAWIDTH_8_BIT
  *         @arg @ref LL_USART_DATAWIDTH_9_BIT
  */
__STATIC_INLINE void LL_USART_SetDataWidth(USART_TypeDef *p_usart, uint32_t data_width)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_M, data_width);
}

/**
  * @brief  Return Word length (e.g. nb of data bits, excluding start and stop bits).
  * @rmtoll
  *  CR1          M0            LL_USART_GetDataWidth \n
  *  CR1          M1            LL_USART_GetDataWidth
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_DATAWIDTH_7_BIT
  *         @arg @ref LL_USART_DATAWIDTH_8_BIT
  *         @arg @ref LL_USART_DATAWIDTH_9_BIT
  */
__STATIC_INLINE uint32_t LL_USART_GetDataWidth(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_M));
}

/**
  * @brief  Allow switch between Mute Mode and Active mode.
  * @rmtoll
  *  CR1          MME           LL_USART_EnableMuteMode
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableMuteMode(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_MME);
}

/**
  * @brief  Prevent Mute Mode use. Set Receiver in active mode permanently.
  * @rmtoll
  *  CR1          MME           LL_USART_DisableMuteMode
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableMuteMode(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_MME);
}

/**
  * @brief  Indicate if switch between Mute Mode and Active mode is allowed.
  * @rmtoll
  *  CR1          MME           LL_USART_IsEnabledMuteMode
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledMuteMode(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_MME) == (USART_CR1_MME)) ? 1UL : 0UL);
}

/**
  * @brief  Set Oversampling to 8-bit or 16-bit mode.
  * @rmtoll
  *  CR1          OVER8         LL_USART_SetOverSampling
  * @param  p_usart USART Instance
  * @param  over_sampling This parameter can be one of the following values:
  *         @arg @ref LL_USART_OVERSAMPLING_16
  *         @arg @ref LL_USART_OVERSAMPLING_8
  */
__STATIC_INLINE void LL_USART_SetOverSampling(USART_TypeDef *p_usart, uint32_t over_sampling)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_OVER8, over_sampling);
}

/**
  * @brief  Return Oversampling mode.
  * @rmtoll
  *  CR1          OVER8         LL_USART_GetOverSampling
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_OVERSAMPLING_16
  *         @arg @ref LL_USART_OVERSAMPLING_8
  */
__STATIC_INLINE uint32_t LL_USART_GetOverSampling(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_OVER8));
}

/**
  * @brief  Configure if Clock pulse of the last data bit is output to the SCLK pin or not.
  * @rmtoll
  *  CR2          LBCL          LL_USART_SetLastClkPulseOutput
  * @param  p_usart USART Instance
  * @param  last_bit_clock_pulse This parameter can be one of the following values:
  *         @arg @ref LL_USART_LASTCLKPULSE_DISABLED
  *         @arg @ref LL_USART_LASTCLKPULSE_ENABLED
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetLastClkPulseOutput(USART_TypeDef *p_usart, uint32_t last_bit_clock_pulse)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_LBCL, last_bit_clock_pulse);
}

/**
  * @brief  Retrieve Clock pulse of the last data bit output configuration
  *         (Last bit Clock pulse output to the SCLK pin or not).
  * @rmtoll
  *  CR2          LBCL          LL_USART_GetLastClkPulseOutput
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_LASTCLKPULSE_DISABLED
  *         @arg @ref LL_USART_LASTCLKPULSE_ENABLED
  */
__STATIC_INLINE uint32_t LL_USART_GetLastClkPulseOutput(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_LBCL));
}

/**
  * @brief  Select the phase of the clock output on the SCLK pin in synchronous mode.
  * @rmtoll
  *  CR2          CPHA          LL_USART_SetClockPhase
  * @param  p_usart USART Instance
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_PHASE_1_EDGE
  *         @arg @ref LL_USART_CLOCK_PHASE_2_EDGE
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetClockPhase(USART_TypeDef *p_usart, uint32_t clock_phase)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_CPHA, clock_phase);
}

/**
  * @brief  Return phase of the clock output on the SCLK pin in synchronous mode.
  * @rmtoll
  *  CR2          CPHA          LL_USART_GetClockPhase
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_PHASE_1_EDGE
  *         @arg @ref LL_USART_CLOCK_PHASE_2_EDGE
  */
__STATIC_INLINE uint32_t LL_USART_GetClockPhase(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_CPHA));
}

/**
  * @brief  Select the polarity of the clock output on the SCLK pin in synchronous mode.
  * @rmtoll
  *  CR2          CPOL          LL_USART_SetClockPolarity
  * @param  p_usart USART Instance
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_POLARITY_LOW
  *         @arg @ref LL_USART_CLOCK_POLARITY_HIGH
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetClockPolarity(USART_TypeDef *p_usart, uint32_t clock_polarity)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_CPOL, clock_polarity);
}

/**
  * @brief  Return polarity of the clock output on the SCLK pin in synchronous mode.
  * @rmtoll
  *  CR2          CPOL          LL_USART_GetClockPolarity
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_POLARITY_LOW
  *         @arg @ref LL_USART_CLOCK_POLARITY_HIGH
  */
__STATIC_INLINE uint32_t LL_USART_GetClockPolarity(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_CPOL));
}

/**
  * @brief  Configure Clock signal format (Phase Polarity and choice about output of last bit clock pulse).
  * @rmtoll
  *  CR2          CPHA          LL_USART_ConfigClock \n
  *  CR2          CPOL          LL_USART_ConfigClock \n
  *  CR2          LBCL          LL_USART_ConfigClock
  * @param  p_usart USART Instance
  * @param  phase This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_PHASE_1_EDGE
  *         @arg @ref LL_USART_CLOCK_PHASE_2_EDGE
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_USART_CLOCK_POLARITY_LOW
  *         @arg @ref LL_USART_CLOCK_POLARITY_HIGH
  * @param  lbcp_output This parameter can be one of the following values:
  *         @arg @ref LL_USART_LASTCLKPULSE_DISABLED
  *         @arg @ref LL_USART_LASTCLKPULSE_ENABLED
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clock Phase configuration using @ref LL_USART_SetClockPhase() function
  *         - Clock Polarity configuration using @ref LL_USART_SetClockPolarity() function
  *         - Output of Last bit Clock pulse configuration using @ref LL_USART_SetLastClkPulseOutput() function
  */
__STATIC_INLINE void LL_USART_ConfigClock(USART_TypeDef *p_usart, uint32_t phase, uint32_t polarity,
                                          uint32_t lbcp_output)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_LBCL, phase | polarity | lbcp_output);
}

/**
  * @brief  Configure Clock source prescaler for the baud rate generator and oversampling.
  * @rmtoll
  *  PRESC        PRESCALER     LL_USART_SetPrescaler
  * @param  p_usart USART Instance
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetPrescaler(USART_TypeDef *p_usart, uint32_t prescaler_value)
{
  STM32_MODIFY_REG(p_usart->PRESC, USART_PRESC_PRESCALER, (uint16_t)prescaler_value);
}

/**
  * @brief  Retrieve the Clock source prescaler for the baud rate generator and oversampling.
  * @rmtoll
  *  PRESC        PRESCALER     LL_USART_GetPrescaler
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  */
__STATIC_INLINE uint32_t LL_USART_GetPrescaler(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->PRESC, USART_PRESC_PRESCALER));
}

/**
  * @brief  Enable Clock output on SCLK pin.
  * @rmtoll
  *  CR2          CLKEN         LL_USART_EnableSCLKOutput
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableSCLKOutput(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Disable Clock output on SCLK pin.
  * @rmtoll
  *  CR2          CLKEN         LL_USART_DisableSCLKOutput
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableSCLKOutput(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Indicate if Clock output on SCLK pin is enabled.
  * @rmtoll
  *  CR2          CLKEN         LL_USART_IsEnabledSCLKOutput
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledSCLKOutput(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_CLKEN) == (USART_CR2_CLKEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the length of the stop bits.
  * @rmtoll
  *  CR2          STOP          LL_USART_SetStopBitsLength
  * @param  p_usart USART Instance
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_USART_STOP_BIT_0_5
  *         @arg @ref LL_USART_STOP_BIT_1
  *         @arg @ref LL_USART_STOP_BIT_1_5
  *         @arg @ref LL_USART_STOP_BIT_2
  */
__STATIC_INLINE void LL_USART_SetStopBitsLength(USART_TypeDef *p_usart, uint32_t stop_bits)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_STOP, stop_bits);
}

/**
  * @brief  Retrieve the length of the stop bits.
  * @rmtoll
  *  CR2          STOP          LL_USART_GetStopBitsLength
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_STOP_BIT_0_5
  *         @arg @ref LL_USART_STOP_BIT_1
  *         @arg @ref LL_USART_STOP_BIT_1_5
  *         @arg @ref LL_USART_STOP_BIT_2
  */
__STATIC_INLINE uint32_t LL_USART_GetStopBitsLength(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_STOP));
}

/**
  * @brief  Configure Character frame format (Datawidth, Parity control, Stop Bits).
  * @rmtoll
  *  CR1          PS            LL_USART_ConfigCharacter \n
  *  CR1          PCE           LL_USART_ConfigCharacter \n
  *  CR1          M0            LL_USART_ConfigCharacter \n
  *  CR1          M1            LL_USART_ConfigCharacter \n
  *  CR2          STOP          LL_USART_ConfigCharacter
  * @param  p_usart USART Instance
  * @param  data_width This parameter can be one of the following values:
  *         @arg @ref LL_USART_DATAWIDTH_7_BIT
  *         @arg @ref LL_USART_DATAWIDTH_8_BIT
  *         @arg @ref LL_USART_DATAWIDTH_9_BIT
  * @param  parity This parameter can be one of the following values:
  *         @arg @ref LL_USART_PARITY_NONE
  *         @arg @ref LL_USART_PARITY_EVEN
  *         @arg @ref LL_USART_PARITY_ODD
  * @param  stop_bits This parameter can be one of the following values:
  *         @arg @ref LL_USART_STOP_BIT_0_5
  *         @arg @ref LL_USART_STOP_BIT_1
  *         @arg @ref LL_USART_STOP_BIT_1_5
  *         @arg @ref LL_USART_STOP_BIT_2
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Data Width configuration using @ref LL_USART_SetDataWidth() function
  *         - Parity Control and mode configuration using @ref LL_USART_SetParity() function
  *         - Stop bits configuration using @ref LL_USART_SetStopBitsLength() function
  */
__STATIC_INLINE void LL_USART_ConfigCharacter(USART_TypeDef *p_usart, uint32_t data_width, uint32_t parity,
                                              uint32_t stop_bits)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_PS | USART_CR1_PCE | USART_CR1_M, parity | data_width);
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_STOP, stop_bits);
}

/**
  * @brief  Configure TX/RX pins swapping setting.
  * @rmtoll
  *  CR2          SWAP          LL_USART_SetTXRXSwap
  * @param  p_usart USART Instance
  * @param  swap_config This parameter can be one of the following values:
  *         @arg @ref LL_USART_TXRX_STANDARD
  *         @arg @ref LL_USART_TXRX_SWAPPED
  */
__STATIC_INLINE void LL_USART_SetTXRXSwap(USART_TypeDef *p_usart, uint32_t swap_config)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_SWAP, swap_config);
}

/**
  * @brief  Retrieve TX/RX pins swapping configuration.
  * @rmtoll
  *  CR2          SWAP          LL_USART_GetTXRXSwap
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_TXRX_STANDARD
  *         @arg @ref LL_USART_TXRX_SWAPPED
  */
__STATIC_INLINE uint32_t LL_USART_GetTXRXSwap(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_SWAP));
}

/**
  * @brief  Configure RX pin active level logic.
  * @rmtoll
  *  CR2          RXINV         LL_USART_SetRXPinLevel
  * @param  p_usart USART Instance
  * @param  pin_inv_method This parameter can be one of the following values:
  *         @arg @ref LL_USART_RXPIN_LEVEL_STANDARD
  *         @arg @ref LL_USART_RXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE void LL_USART_SetRXPinLevel(USART_TypeDef *p_usart, uint32_t pin_inv_method)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_RXINV, pin_inv_method);
}

/**
  * @brief  Retrieve RX pin active level logic configuration.
  * @rmtoll
  *  CR2          RXINV         LL_USART_GetRXPinLevel
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_RXPIN_LEVEL_STANDARD
  *         @arg @ref LL_USART_RXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE uint32_t LL_USART_GetRXPinLevel(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_RXINV));
}

/**
  * @brief  Configure TX pin active level logic.
  * @rmtoll
  *  CR2          TXINV         LL_USART_SetTXPinLevel
  * @param  p_usart USART Instance
  * @param  pin_inv_method This parameter can be one of the following values:
  *         @arg @ref LL_USART_TXPIN_LEVEL_STANDARD
  *         @arg @ref LL_USART_TXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE void LL_USART_SetTXPinLevel(USART_TypeDef *p_usart, uint32_t pin_inv_method)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_TXINV, pin_inv_method);
}

/**
  * @brief  Retrieve TX pin active level logic configuration.
  * @rmtoll
  *  CR2          TXINV         LL_USART_GetTXPinLevel
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_TXPIN_LEVEL_STANDARD
  *         @arg @ref LL_USART_TXPIN_LEVEL_INVERTED
  */
__STATIC_INLINE uint32_t LL_USART_GetTXPinLevel(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_TXINV));
}

/**
  * @brief  Configure Binary data logic.
  * @rmtoll
  *  CR2          DATAINV       LL_USART_SetBinaryDataLogic
  * @param  p_usart USART Instance
  * @param  data_logic This parameter can be one of the following values:
  *         @arg @ref LL_USART_BINARY_LOGIC_POSITIVE
  *         @arg @ref LL_USART_BINARY_LOGIC_NEGATIVE
  * @note   Allow to define how Logical data from the data register are send/received :
  *         either in positive/direct logic (1=H, 0=L) or in negative/inverse logic (1=L, 0=H)
  */
__STATIC_INLINE void LL_USART_SetBinaryDataLogic(USART_TypeDef *p_usart, uint32_t data_logic)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_DATAINV, data_logic);
}

/**
  * @brief  Retrieve Binary data configuration.
  * @rmtoll
  *  CR2          DATAINV       LL_USART_GetBinaryDataLogic
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_BINARY_LOGIC_POSITIVE
  *         @arg @ref LL_USART_BINARY_LOGIC_NEGATIVE
  */
__STATIC_INLINE uint32_t LL_USART_GetBinaryDataLogic(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_DATAINV));
}

/**
  * @brief  Configure transfer bit order (either Less or Most Significant Bit First).
  * @rmtoll
  *  CR2          MSBFIRST      LL_USART_SetTransferBitOrder
  * @param  p_usart USART Instance
  * @param  bit_order This parameter can be one of the following values:
  *         @arg @ref LL_USART_BITORDER_LSB_FIRST
  *         @arg @ref LL_USART_BITORDER_MSB_FIRST
  * @note   MSB First means data is transmitted/received with the MSB first, following the start bit.
  *         LSB First means data is transmitted/received with data bit 0 first, following the start bit.
  */
__STATIC_INLINE void LL_USART_SetTransferBitOrder(USART_TypeDef *p_usart, uint32_t bit_order)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_MSBFIRST, bit_order);
}

/**
  * @brief  Return transfer bit order (either Less or Most Significant Bit First).
  * @rmtoll
  *  CR2          MSBFIRST      LL_USART_GetTransferBitOrder
  * @param  p_usart USART Instance
  * @note   MSB First means data is transmitted/received with the MSB first, following the start bit.
  *         LSB First means data is transmitted/received with data bit 0 first, following the start bit.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_BITORDER_LSB_FIRST
  *         @arg @ref LL_USART_BITORDER_MSB_FIRST
  */
__STATIC_INLINE uint32_t LL_USART_GetTransferBitOrder(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_MSBFIRST));
}

/**
  * @brief  Enable Auto Baud-Rate Detection.
  * @rmtoll
  *  CR2          ABREN         LL_USART_EnableAutoBaudRate
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableAutoBaudRate(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_ABREN);
}

/**
  * @brief  Disable Auto Baud-Rate Detection.
  * @rmtoll
  *  CR2          ABREN         LL_USART_DisableAutoBaudRate
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableAutoBaudRate(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_ABREN);
}

/**
  * @brief  Indicate if Auto Baud-Rate Detection mechanism is enabled.
  * @rmtoll
  *  CR2          ABREN         LL_USART_IsEnabledAutoBaud
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledAutoBaud(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_ABREN) == (USART_CR2_ABREN)) ? 1UL : 0UL);
}

/**
  * @brief  Set Auto Baud-Rate mode bits.
  * @rmtoll
  *  CR2          ABRMODE       LL_USART_SetAutoBaudRateMode
  * @param  p_usart USART Instance
  * @param  auto_baud_rate_mode This parameter can be one of the following values:
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_START_BIT
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_FALLING_EDGE
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_0X7F_FRAME
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_0X55_FRAME
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetAutoBaudRateMode(USART_TypeDef *p_usart, uint32_t auto_baud_rate_mode)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_ABRMOD, auto_baud_rate_mode);
}

/**
  * @brief  Return Auto Baud-Rate mode.
  * @rmtoll
  *  CR2          ABRMODE       LL_USART_GetAutoBaudRateMode
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_START_BIT
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_FALLING_EDGE
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_0X7F_FRAME
  *         @arg @ref LL_USART_AUTO_BAUD_DETECT_ON_0X55_FRAME
  */
__STATIC_INLINE uint32_t LL_USART_GetAutoBaudRateMode(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_ABRMOD));
}

/**
  * @brief  Enable Receiver Timeout.
  * @rmtoll
  *  CR2          RTOEN         LL_USART_EnableRxTimeout
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableRxTimeout(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_RTOEN);
}

/**
  * @brief  Disable Receiver Timeout.
  * @rmtoll
  *  CR2          RTOEN         LL_USART_DisableRxTimeout
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableRxTimeout(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_RTOEN);
}

/**
  * @brief  Indicate if Receiver Timeout feature is enabled.
  * @rmtoll
  *  CR2          RTOEN         LL_USART_IsEnabledRxTimeout
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledRxTimeout(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_RTOEN) == (USART_CR2_RTOEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set a 8 bit Address of the USART node as set in ADD field of CR2.
  * @rmtoll
  *  CR2          ADD           LL_USART_SetNodeAddress
  * @param  p_usart USART Instance
  * @param  node_address 4 or 7 bit Address of the USART node.
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4bits (b3-b0) of returned value are relevant (b31-b4 are not relevant)
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8bits (b7-b0) of returned value are relevant (b31-b8 are not relevant)
  */
__STATIC_INLINE void LL_USART_SetNodeAddress(USART_TypeDef *p_usart, uint32_t node_address)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_ADD, (node_address << USART_CR2_ADD_Pos));
}

/**
  * @brief  Set the Address length of the USART node in ADDM7 field of CR2.
  * @rmtoll
  *  CR2          ADDM7           LL_USART_SetNodeAddressLength
  * @param  p_usart USART Instance
  * @param  address_len This parameter can be one of the following values:
  *         @arg @ref LL_USART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_USART_ADDRESS_DETECT_7_BIT
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4bits (b3-b0) of returned value are relevant (b31-b4 are not relevant)
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8bits (b7-b0) of returned value are relevant (b31-b8 are not relevant)
  */
__STATIC_INLINE void LL_USART_SetNodeAddressLength(USART_TypeDef *p_usart, uint32_t address_len)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_ADDM7, address_len);
}

/**
  * @brief  Configure Address and Address length of the USART node.
  * @rmtoll
  *  CR2          ADD           LL_USART_ConfigNodeAddress \n
  *  CR2          ADDM7         LL_USART_ConfigNodeAddress
  * @param  p_usart USART Instance
  * @param  address_len This parameter can be one of the following values:
  *         @arg @ref LL_USART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_USART_ADDRESS_DETECT_7_BIT
  * @param  node_address 4 or 7 bit Address of the USART node.
  * @note   This is used in multiprocessor communication during Mute mode or Stop mode,
  *         for wake up with address mark detection.
  * @note   4bits address node is used when 4-bit Address Detection is selected in ADDM7.
  *         (b7-b4 must be set to 0)
  *         8bits address node is used when 7-bit Address Detection is selected in ADDM7.
  *         (This is used in multiprocessor communication during Mute mode or Stop mode,
  *         for wake up with 7-bit address mark detection.
  *         The MSB of the character sent by the transmitter must be equal to 1.
  *         It can also be used for character detection during normal reception,
  *         Mute mode inactive (for example, end of block detection in ModBus protocol).
  *         In this case, the whole received character (8-bit) is compared to the ADD[7:0]
  *         value and CMF flag is set on match)
  */
__STATIC_INLINE void LL_USART_ConfigNodeAddress(USART_TypeDef *p_usart, uint32_t address_len, uint32_t node_address)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_ADD | USART_CR2_ADDM7,
                   (uint32_t)(address_len | (node_address << USART_CR2_ADD_Pos)));
}

/**
  * @brief  Return 8 bit Address of the USART node as set in ADD field of CR2.
  * @rmtoll
  *  CR2          ADD           LL_USART_GetNodeAddress
  * @param  p_usart USART Instance
  * @note   If 4-bit Address Detection is selected in ADDM7,
  *         only 4bits (b3-b0) of returned value are relevant (b31-b4 are not relevant)
  *         If 7-bit Address Detection is selected in ADDM7,
  *         only 8bits (b7-b0) of returned value are relevant (b31-b8 are not relevant)
  * @retval Address of the USART node (Value between Min_Data=0 and Max_Data=255)
  */
__STATIC_INLINE uint32_t LL_USART_GetNodeAddress(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_ADD) >> USART_CR2_ADD_Pos);
}

/**
  * @brief  Return Length of Node Address used in Address Detection mode (7-bit or 4-bit).
  * @rmtoll
  *  CR2          ADDM7         LL_USART_GetNodeAddressLength
  * @param  p_usart USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_ADDRESS_DETECT_4_BIT
  *         @arg @ref LL_USART_ADDRESS_DETECT_7_BIT
  */
__STATIC_INLINE uint32_t LL_USART_GetNodeAddressLength(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_ADDM7));
}

/**
  * @brief  Enable RTS HW Flow Control.
  * @rmtoll
  *  CR3          RTSE          LL_USART_EnableRTSHWFlowCtrl
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableRTSHWFlowCtrl(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_RTSE);
}

/**
  * @brief  Disable RTS HW Flow Control.
  * @rmtoll
  *  CR3          RTSE          LL_USART_DisableRTSHWFlowCtrl
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableRTSHWFlowCtrl(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_RTSE);
}

/**
  * @brief  Enable CTS HW Flow Control.
  * @rmtoll
  *  CR3          CTSE          LL_USART_EnableCTSHWFlowCtrl
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableCTSHWFlowCtrl(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_CTSE);
}

/**
  * @brief  Disable CTS HW Flow Control.
  * @rmtoll
  *  CR3          CTSE          LL_USART_DisableCTSHWFlowCtrl
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableCTSHWFlowCtrl(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_CTSE);
}

/**
  * @brief  Configure HW Flow Control mode (both CTS and RTS).
  * @rmtoll
  *  CR3          RTSE          LL_USART_SetHWFlowCtrl \n
  *  CR3          CTSE          LL_USART_SetHWFlowCtrl
  * @param  p_usart USART Instance
  * @param  hardware_flow_control This parameter can be one of the following values:
  *         @arg @ref LL_USART_HWCONTROL_NONE
  *         @arg @ref LL_USART_HWCONTROL_RTS
  *         @arg @ref LL_USART_HWCONTROL_CTS
  *         @arg @ref LL_USART_HWCONTROL_RTS_CTS
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetHWFlowCtrl(USART_TypeDef *p_usart, uint32_t hardware_flow_control)
{
  STM32_MODIFY_REG(p_usart->CR3, USART_CR3_RTSE | USART_CR3_CTSE, hardware_flow_control);
}

/**
  * @brief  Return HW Flow Control configuration (both CTS and RTS).
  * @rmtoll
  *  CR3          RTSE          LL_USART_GetHWFlowCtrl \n
  *  CR3          CTSE          LL_USART_GetHWFlowCtrl
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_HWCONTROL_NONE
  *         @arg @ref LL_USART_HWCONTROL_RTS
  *         @arg @ref LL_USART_HWCONTROL_CTS
  *         @arg @ref LL_USART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE uint32_t LL_USART_GetHWFlowCtrl(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

/**
  * @brief  Enable One bit sampling method.
  * @rmtoll
  *  CR3          ONEBIT        LL_USART_EnableOneBitSample
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableOneBitSample(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_ONEBIT);
}

/**
  * @brief  Disable One bit sampling method.
  * @rmtoll
  *  CR3          ONEBIT        LL_USART_DisableOneBitSample
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableOneBitSample(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_ONEBIT);
}

/**
  * @brief  Indicate if One bit sampling method is enabled.
  * @rmtoll
  *  CR3          ONEBIT        LL_USART_IsEnabledOneBitSample
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledOneBitSample(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_ONEBIT) == (USART_CR3_ONEBIT)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Overrun detection.
  * @rmtoll
  *  CR3          OVRDIS        LL_USART_EnableOverrunDetect
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableOverrunDetect(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_OVRDIS);
}

/**
  * @brief  Disable Overrun detection.
  * @rmtoll
  *  CR3          OVRDIS        LL_USART_DisableOverrunDetect
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableOverrunDetect(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_OVRDIS);
}

/**
  * @brief  Indicate if Overrun detection is enabled.
  * @rmtoll
  *  CR3          OVRDIS        LL_USART_IsEnabledOverrunDetect
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledOverrunDetect(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_OVRDIS) != USART_CR3_OVRDIS) ? 1UL : 0UL);
}

/**
  * @brief  Select event type for Wake UP Interrupt Flag (WUS[1:0] bits).
  * @rmtoll
  *  CR3          WUS           LL_USART_SetWKUPType
  * @param  p_usart USART Instance
  * @param  Type This parameter can be one of the following values:
  *         @arg @ref LL_USART_WAKEUP_ON_ADDRESS
  *         @arg @ref LL_USART_WAKEUP_ON_STARTBIT
  *         @arg @ref LL_USART_WAKEUP_ON_RXNE
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetWKUPType(USART_TypeDef *p_usart, uint32_t Type)
{
  STM32_MODIFY_REG(p_usart->CR3, USART_CR3_WUS, Type);
}

/**
  * @brief  Return event type for Wake UP Interrupt Flag (WUS[1:0] bits).
  * @rmtoll
  *  CR3          WUS           LL_USART_GetWKUPType
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_WAKEUP_ON_ADDRESS
  *         @arg @ref LL_USART_WAKEUP_ON_STARTBIT
  *         @arg @ref LL_USART_WAKEUP_ON_RXNE
  */
__STATIC_INLINE uint32_t LL_USART_GetWKUPType(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_WUS));
}

/**
  * @brief  Configure USART BRR register for achieving expected Baud Rate value.
  * @rmtoll
  *  BRR          BRR           LL_USART_SetBaudRate
  * @param  p_usart USART Instance
  * @param  periph_clk Peripheral Clock
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  * @param  over_sampling This parameter can be one of the following values:
  *         @arg @ref LL_USART_OVERSAMPLING_16
  *         @arg @ref LL_USART_OVERSAMPLING_8
  * @param  baud_rate Baud Rate
  * @note   Compute and set USARTDIV value in BRR Register (full BRR content)
  *         according to used Peripheral Clock, Oversampling mode, and expected Baud Rate values
  * @note   Peripheral clock and Baud rate values provided as function parameters must be valid
  *         (Baud rate value != 0)
  * @note   In case of oversampling by 16 and 8, BRR content must be greater than or equal to 16d.
  */
__STATIC_INLINE void LL_USART_SetBaudRate(USART_TypeDef *p_usart, uint32_t periph_clk, uint32_t prescaler_value,
                                          uint32_t over_sampling,
                                          uint32_t baud_rate)
{
  uint32_t usartdiv;
  uint32_t brrtemp;

  if (prescaler_value > LL_USART_PRESCALER_DIV256)
  {
    /* Do not overstep the size of USART_PRESCALER_TAB */
  }
  else if (baud_rate == 0U)
  {
    /* Can Not divide per 0 */
  }
  else if (over_sampling == LL_USART_OVERSAMPLING_8)
  {
    usartdiv = (uint16_t)(LL_USART_DIV_SAMPLING8(periph_clk, (uint8_t)prescaler_value, baud_rate));
    brrtemp = usartdiv & 0xFFF0U;
    brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
    p_usart->BRR = brrtemp;
  }
  else
  {
    p_usart->BRR = (uint16_t)(LL_USART_DIV_SAMPLING16(periph_clk, (uint8_t)prescaler_value, baud_rate));
  }
}

/**
  * @brief  Return current Baud Rate value, according to USARTDIV present in BRR register
  *         (full BRR content), and to used Peripheral Clock and Oversampling mode values.
  * @rmtoll
  *  BRR          BRR           LL_USART_GetBaudRate
  * @param  p_usart USART Instance
  * @param  periph_clk Peripheral Clock
  * @param  prescaler_value This parameter can be one of the following values:
  *         @arg @ref LL_USART_PRESCALER_DIV1
  *         @arg @ref LL_USART_PRESCALER_DIV2
  *         @arg @ref LL_USART_PRESCALER_DIV4
  *         @arg @ref LL_USART_PRESCALER_DIV6
  *         @arg @ref LL_USART_PRESCALER_DIV8
  *         @arg @ref LL_USART_PRESCALER_DIV10
  *         @arg @ref LL_USART_PRESCALER_DIV12
  *         @arg @ref LL_USART_PRESCALER_DIV16
  *         @arg @ref LL_USART_PRESCALER_DIV32
  *         @arg @ref LL_USART_PRESCALER_DIV64
  *         @arg @ref LL_USART_PRESCALER_DIV128
  *         @arg @ref LL_USART_PRESCALER_DIV256
  * @param  over_sampling This parameter can be one of the following values:
  *         @arg @ref LL_USART_OVERSAMPLING_16
  *         @arg @ref LL_USART_OVERSAMPLING_8
  * @note   In case of non-initialized or invalid value stored in BRR register, value 0 will be returned.
  * @note   In case of oversampling by 16 and 8, BRR content must be greater than or equal to 16d.
  * @retval Baud Rate
  */
__STATIC_INLINE uint32_t LL_USART_GetBaudRate(const USART_TypeDef *p_usart, uint32_t periph_clk,
                                              uint32_t prescaler_value, uint32_t over_sampling)
{
  uint32_t usartdiv;
  uint32_t brrresult = 0x0U;
  uint32_t periphclkpresc = (uint32_t)(periph_clk / (USART_PRESCALER_TAB[(uint8_t)prescaler_value]));

  usartdiv = p_usart->BRR;

  if (usartdiv == 0U)
  {
    /* Do not perform a division by 0 */
  }
  else if (over_sampling == (uint32_t)LL_USART_OVERSAMPLING_8)
  {
    usartdiv = (uint16_t)((usartdiv & 0xFFF0U) | ((usartdiv & 0x0007U) << 1U)) ;
    if (usartdiv != 0U)
    {
      brrresult = (periphclkpresc * 2U) / usartdiv;
    }
  }
  else
  {
    if ((usartdiv & 0xFFFFU) != 0U)
    {
      brrresult = periphclkpresc / usartdiv;
    }
  }
  return (brrresult);
}

/**
  * @brief  Set Receiver Time Out Value (expressed in nb of bits duration).
  * @rmtoll
  *  RTOR         RTO           LL_USART_SetRxTimeout
  * @param  p_usart USART Instance
  * @param  timeout Value between Min_Data=0x00 and Max_Data=0x00FFFFFF
  */
__STATIC_INLINE void LL_USART_SetRxTimeout(USART_TypeDef *p_usart, uint32_t timeout)
{
  STM32_MODIFY_REG(p_usart->RTOR, USART_RTOR_RTO, timeout);
}

/**
  * @brief  Get Receiver Time Out Value (expressed in nb of bits duration).
  * @rmtoll
  *  RTOR         RTO           LL_USART_GetRxTimeout
  * @param  p_usart USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x00FFFFFF
  */
__STATIC_INLINE uint32_t LL_USART_GetRxTimeout(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->RTOR, USART_RTOR_RTO));
}

/**
  * @brief  Set Block Length value in reception.
  * @rmtoll
  *  RTOR         BLEN          LL_USART_SetBlockLength
  * @param  p_usart USART Instance
  * @param  block_length Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_USART_SetBlockLength(USART_TypeDef *p_usart, uint32_t block_length)
{
  STM32_MODIFY_REG(p_usart->RTOR, USART_RTOR_BLEN, block_length << USART_RTOR_BLEN_Pos);
}

/**
  * @brief  Get Block Length value in reception.
  * @rmtoll
  *  RTOR         BLEN          LL_USART_GetBlockLength
  * @param  p_usart USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_USART_GetBlockLength(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->RTOR, USART_RTOR_BLEN) >> USART_RTOR_BLEN_Pos);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_IRDA Configuration functions related to Irda feature
  * @{
  */

/**
  * @brief  Enable IrDA mode.
  * @rmtoll
  *  CR3          IREN          LL_USART_EnableIrda
  * @param  p_usart USART Instance
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIrda(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_IREN);
}

/**
  * @brief  Disable IrDA mode.
  * @rmtoll
  *  CR3          IREN          LL_USART_DisableIrda
  * @param  p_usart USART Instance
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIrda(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_IREN);
}

/**
  * @brief  Indicate if IrDA mode is enabled.
  * @rmtoll
  *  CR3          IREN          LL_USART_IsEnabledIrda
  * @param  p_usart USART Instance
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIrda(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_IREN) == (USART_CR3_IREN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure IrDA Power Mode (Normal or Low Power).
  * @rmtoll
  *  CR3          IRLP          LL_USART_SetIrdaPowerMode
  * @param  p_usart USART Instance
  * @param  power_mode This parameter can be one of the following values:
  *         @arg @ref LL_USART_IRDA_POWER_MODE_NORMAL
  *         @arg @ref LL_USART_IRDA_POWER_MODE_LOW
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetIrdaPowerMode(USART_TypeDef *p_usart, uint32_t power_mode)
{
  STM32_MODIFY_REG(p_usart->CR3, USART_CR3_IRLP, power_mode);
}

/**
  * @brief  Retrieve IrDA Power Mode configuration (Normal or Low Power).
  * @rmtoll
  *  CR3          IRLP          LL_USART_GetIrdaPowerMode
  * @param  p_usart USART Instance
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_IRDA_POWER_MODE_NORMAL
  *         @arg @ref LL_USART_IRDA_POWER_MODE_LOW
  */
__STATIC_INLINE uint32_t LL_USART_GetIrdaPowerMode(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_IRLP));
}

/**
  * @brief  Set Irda prescaler value, used for dividing the USART clock source
  *         to achieve the Irda Low Power frequency (8 bits value).
  * @rmtoll
  *  GTPR         PSC           LL_USART_SetIrdaPrescaler
  * @param  p_usart USART Instance
  * @param  prescaler_value Value between Min_Data=0x00 and Max_Data=0xFF
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetIrdaPrescaler(USART_TypeDef *p_usart, uint32_t prescaler_value)
{
  STM32_MODIFY_REG(p_usart->GTPR, USART_GTPR_PSC, (uint16_t)prescaler_value);
}

/**
  * @brief  Return Irda prescaler value, used for dividing the USART clock source
  *         to achieve the Irda Low Power frequency (8 bits value).
  * @rmtoll
  *  GTPR         PSC           LL_USART_GetIrdaPrescaler
  * @param  p_usart USART Instance
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  * @retval Irda prescaler value (Value between Min_Data=0x00 and Max_Data=0xFF)
  */
__STATIC_INLINE uint32_t LL_USART_GetIrdaPrescaler(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->GTPR, USART_GTPR_PSC));
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_Smartcard Configuration functions related to Smartcard feature
  * @{
  */

/**
  * @brief  Enable Smartcard NACK transmission.
  * @rmtoll
  *  CR3          NACK          LL_USART_EnableSmartcardNACK
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableSmartcardNACK(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_NACK);
}

/**
  * @brief  Disable Smartcard NACK transmission.
  * @rmtoll
  *  CR3          NACK          LL_USART_DisableSmartcardNACK
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableSmartcardNACK(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_NACK);
}

/**
  * @brief  Indicate if Smartcard NACK transmission is enabled.
  * @rmtoll
  *  CR3          NACK          LL_USART_IsEnabledSmartcardNACK
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledSmartcardNACK(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_NACK) == (USART_CR3_NACK)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Smartcard mode.
  * @rmtoll
  *  CR3          SCEN          LL_USART_EnableSmartcard
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableSmartcard(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_SCEN);
}

/**
  * @brief  Disable Smartcard mode.
  * @rmtoll
  *  CR3          SCEN          LL_USART_DisableSmartcard
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableSmartcard(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_SCEN);
}

/**
  * @brief  Indicate if Smartcard mode is enabled.
  * @rmtoll
  *  CR3          SCEN          LL_USART_IsEnabledSmartcard
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledSmartcard(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_SCEN) == (USART_CR3_SCEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set Smartcard Auto-Retry Count value (SCARCNT[2:0] bits).
  * @rmtoll
  *  CR3          SCARCNT       LL_USART_SetSmartcardAutoRetryCount
  * @param  p_usart USART Instance
  * @param  auto_retry_count Value between Min_Data=0 and Max_Data=7
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @note   This bit-field specifies the number of retries in transmit and receive, in Smartcard mode.
  *         In transmission mode, it specifies the number of automatic retransmission retries, before
  *         generating a transmission error (FE bit set).
  *         In reception mode, it specifies the number or erroneous reception trials, before generating a
  *         reception error (RXNE and PE bits set)
  */
__STATIC_INLINE void LL_USART_SetSmartcardAutoRetryCount(USART_TypeDef *p_usart, uint32_t auto_retry_count)
{
  STM32_MODIFY_REG(p_usart->CR3, USART_CR3_SCARCNT, auto_retry_count << USART_CR3_SCARCNT_Pos);
}

/**
  * @brief  Return Smartcard Auto-Retry Count value (SCARCNT[2:0] bits).
  * @rmtoll
  *  CR3          SCARCNT       LL_USART_GetSmartcardAutoRetryCount
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval Smartcard Auto-Retry Count value (Value between Min_Data=0 and Max_Data=7)
  */
__STATIC_INLINE uint32_t LL_USART_GetSmartcardAutoRetryCount(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_SCARCNT) >> USART_CR3_SCARCNT_Pos);
}

/**
  * @brief  Set Smartcard prescaler value, used for dividing the USART clock
  *         source to provide the SMARTCARD Clock (5 bits value).
  * @rmtoll
  *  GTPR         PSC           LL_USART_SetSmartcardPrescaler
  * @param  p_usart USART Instance
  * @param  prescaler_value Value between Min_Data=0 and Max_Data=31
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetSmartcardPrescaler(USART_TypeDef *p_usart, uint32_t prescaler_value)
{
  STM32_MODIFY_REG(p_usart->GTPR, USART_GTPR_PSC, (uint16_t)prescaler_value);
}

/**
  * @brief  Return Smartcard prescaler value, used for dividing the USART clock
  *         source to provide the SMARTCARD Clock (5 bits value).
  * @rmtoll
  *  GTPR         PSC           LL_USART_GetSmartcardPrescaler
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval Smartcard prescaler value (Value between Min_Data=0 and Max_Data=31)
  */
__STATIC_INLINE uint32_t LL_USART_GetSmartcardPrescaler(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->GTPR, USART_GTPR_PSC));
}

/**
  * @brief  Set Smartcard Guard time value, expressed in nb of baud clocks periods
  *         (GT[7:0] bits : Guard time value).
  * @rmtoll
  *  GTPR         GT            LL_USART_SetSmartcardGuardTime
  * @param  p_usart USART Instance
  * @param  guard_time Value between Min_Data=0x00 and Max_Data=0xFF
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetSmartcardGuardTime(USART_TypeDef *p_usart, uint32_t guard_time)
{
  STM32_MODIFY_REG(p_usart->GTPR, USART_GTPR_GT, (uint16_t)(guard_time << USART_GTPR_GT_Pos));
}

/**
  * @brief  Return Smartcard Guard time value, expressed in nb of baud clocks periods
  *         (GT[7:0] bits : Guard time value).
  * @rmtoll
  *  GTPR         GT            LL_USART_GetSmartcardGuardTime
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval Smartcard Guard time value (Value between Min_Data=0x00 and Max_Data=0xFF)
  */
__STATIC_INLINE uint32_t LL_USART_GetSmartcardGuardTime(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->GTPR, USART_GTPR_GT) >> USART_GTPR_GT_Pos);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_HalfDuplex Configuration functions related to Half Duplex feature
  * @{
  */

/**
  * @brief  Enable Single Wire Half-Duplex mode.
  * @rmtoll
  *  CR3          HDSEL         LL_USART_EnableHalfDuplex
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HALFDUPLEX_INSTANCE(p_usart) can be used to check whether or not
  *         Half-Duplex mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableHalfDuplex(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Disable Single Wire Half-Duplex mode.
  * @rmtoll
  *  CR3          HDSEL         LL_USART_DisableHalfDuplex
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HALFDUPLEX_INSTANCE(p_usart) can be used to check whether or not
  *         Half-Duplex mode is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableHalfDuplex(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Indicate if Single Wire Half-Duplex mode is enabled.
  * @rmtoll
  *  CR3          HDSEL         LL_USART_IsEnabledHalfDuplex
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HALFDUPLEX_INSTANCE(p_usart) can be used to check whether or not
  *         Half-Duplex mode is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledHalfDuplex(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_HDSEL) == (USART_CR3_HDSEL)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_SPI_SLAVE Configuration functions related to SPI Slave feature
  * @{
  */
/**
  * @brief  Enable SPI Synchronous Slave mode.
  * @rmtoll
  *  CR2          SLVEN         LL_USART_EnableSPISlave
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableSPISlave(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_SLVEN);
}

/**
  * @brief  Disable SPI Synchronous Slave mode.
  * @rmtoll
  *  CR2          SLVEN         LL_USART_DisableSPISlave
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableSPISlave(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_SLVEN);
}

/**
  * @brief  Indicate if  SPI Synchronous Slave mode is enabled.
  * @rmtoll
  *  CR2          SLVEN         LL_USART_IsEnabledSPISlave
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledSPISlave(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_SLVEN) == (USART_CR2_SLVEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable SPI Slave Selection using NSS input pin.
  * @rmtoll
  *  CR2          DIS_NSS       LL_USART_EnableSPISlaveSelect
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  * @note   SPI Slave Selection depends on NSS input pin
  *         (The slave is selected when NSS is low and deselected when NSS is high).
  */
__STATIC_INLINE void LL_USART_EnableSPISlaveSelect(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_DIS_NSS);
}

/**
  * @brief  Disable SPI Slave Selection using NSS input pin.
  * @rmtoll
  *  CR2          DIS_NSS       LL_USART_DisableSPISlaveSelect
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  * @note   SPI Slave will be always selected and NSS input pin will be ignored.
  */
__STATIC_INLINE void LL_USART_DisableSPISlaveSelect(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_DIS_NSS);
}

/**
  * @brief  Indicate if  SPI Slave Selection depends on NSS input pin.
  * @rmtoll
  *  CR2          DIS_NSS       LL_USART_IsEnabledSPISlaveSelect
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledSPISlaveSelect(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_DIS_NSS) != (USART_CR2_DIS_NSS)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_LIN Configuration functions related to LIN feature
  * @{
  */

/**
  * @brief  Set LIN Break Detection Length.
  * @rmtoll
  *  CR2          LBDL          LL_USART_SetLINBrkDetectionLen
  * @param  p_usart USART Instance
  * @param  lin_break_detect_length This parameter can be one of the following values:
  *         @arg @ref LL_USART_LIN_BREAK_DETECT_10_BIT
  *         @arg @ref LL_USART_LIN_BREAK_DETECT_11_BIT
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetLINBrkDetectionLen(USART_TypeDef *p_usart, uint32_t lin_break_detect_length)
{
  STM32_MODIFY_REG(p_usart->CR2, USART_CR2_LBDL, lin_break_detect_length);
}

/**
  * @brief  Return LIN Break Detection Length.
  * @rmtoll
  *  CR2          LBDL          LL_USART_GetLINBrkDetectionLen
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_LIN_BREAK_DETECT_10_BIT
  *         @arg @ref LL_USART_LIN_BREAK_DETECT_11_BIT
  */
__STATIC_INLINE uint32_t LL_USART_GetLINBrkDetectionLen(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR2, USART_CR2_LBDL));
}

/**
  * @brief  Enable LIN mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_EnableLIN
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableLIN(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Disable LIN mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_DisableLIN
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableLIN(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Indicate if LIN mode is enabled.
  * @rmtoll
  *  CR2          LINEN         LL_USART_IsEnabledLIN
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledLIN(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_LINEN) == (USART_CR2_LINEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Configuration_DE Configuration functions related to Driver Enable feature
  * @{
  */

/**
  * @brief  Set DEDT (Driver Enable De-Assertion Time), Time value expressed on 5 bits ([4:0] bits).
  * @rmtoll
  *  CR1          DEDT          LL_USART_SetDEDeassertionTime
  * @param  p_usart USART Instance
  * @param  time Value between Min_Data=0 and Max_Data=31
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetDEDeassertionTime(USART_TypeDef *p_usart, uint32_t time)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_DEDT, time << USART_CR1_DEDT_Pos);
}

/**
  * @brief  Return DEDT (Driver Enable De-Assertion Time).
  * @rmtoll
  *  CR1          DEDT          LL_USART_GetDEDeassertionTime
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  * @retval Time value expressed on 5 bits ([4:0] bits) : Value between Min_Data=0 and Max_Data=31
  */
__STATIC_INLINE uint32_t LL_USART_GetDEDeassertionTime(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_DEDT) >> USART_CR1_DEDT_Pos);
}

/**
  * @brief  Set DEAT (Driver Enable Assertion Time), Time value expressed on 5 bits ([4:0] bits).
  * @rmtoll
  *  CR1          DEAT          LL_USART_SetDEAssertionTime
  * @param  p_usart USART Instance
  * @param  time Value between Min_Data=0 and Max_Data=31
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetDEAssertionTime(USART_TypeDef *p_usart, uint32_t time)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_DEAT, time << USART_CR1_DEAT_Pos);
}

/**
  * @brief  Return DEAT (Driver Enable Assertion Time).
  * @rmtoll
  *  CR1          DEAT          LL_USART_GetDEAssertionTime
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  * @retval Time value expressed on 5 bits ([4:0] bits) : Value between Min_Data=0 and Max_Data=31
  */
__STATIC_INLINE uint32_t LL_USART_GetDEAssertionTime(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR1, USART_CR1_DEAT) >> USART_CR1_DEAT_Pos);
}

/**
  * @brief  Set DEAT (Driver Enable Assertion Time) and DEDT (Driver Enable De-Assertion Time),
  *         time values expressed on 5 bits ([4:0] bits).
  * @rmtoll
  *  CR1          DEAT          LL_USART_SetDEAssertionTime \n
  *  CR1          DEDT          LL_USART_GetDEDeassertionTime
  * @param  p_usart USART Instance
  * @param  assertion_time Value between Min_Data=0 and Max_Data=31
  * @param  deassertion_time Value between Min_Data=0 and Max_Data=31
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ConfigDETime(USART_TypeDef *p_usart, uint32_t assertion_time, uint32_t deassertion_time)
{
  STM32_MODIFY_REG(p_usart->CR1, USART_CR1_DEAT | USART_CR1_DEDT,
                   (assertion_time << USART_CR1_DEAT_Pos) | (deassertion_time << USART_CR1_DEDT_Pos));
}

/**
  * @brief  Enable Driver Enable (DE) Mode.
  * @rmtoll
  *  CR3          DEM           LL_USART_EnableDEMode
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableDEMode(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_DEM);
}

/**
  * @brief  Disable Driver Enable (DE) Mode.
  * @rmtoll
  *  CR3          DEM           LL_USART_DisableDEMode
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableDEMode(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_DEM);
}

/**
  * @brief  Indicate if Driver Enable (DE) Mode is enabled.
  * @rmtoll
  *  CR3          DEM           LL_USART_IsEnabledDEMode
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDEMode(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_DEM) == (USART_CR3_DEM)) ? 1UL : 0UL);
}

/**
  * @brief  Select Driver Enable Polarity.
  * @rmtoll
  *  CR3          DEP           LL_USART_SetDESignalPolarity
  * @param  p_usart USART Instance
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_USART_DE_POLARITY_HIGH
  *         @arg @ref LL_USART_DE_POLARITY_LOW
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_SetDESignalPolarity(USART_TypeDef *p_usart, uint32_t polarity)
{
  STM32_MODIFY_REG(p_usart->CR3, USART_CR3_DEP, polarity);
}

/**
  * @brief  Return Driver Enable Polarity.
  * @rmtoll
  *  CR3          DEP           LL_USART_GetDESignalPolarity
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_DRIVER_ENABLE_INSTANCE(p_usart) can be used to check whether or not
  *         Driver Enable feature is supported by the p_usart instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_USART_DE_POLARITY_HIGH
  *         @arg @ref LL_USART_DE_POLARITY_LOW
  */
__STATIC_INLINE uint32_t LL_USART_GetDESignalPolarity(const USART_TypeDef *p_usart)
{
  return (uint32_t)(STM32_READ_BIT(p_usart->CR3, USART_CR3_DEP));
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_AdvancedConfiguration Advanced Configurations services
  * @{
  */

/**
  * @brief  Perform basic configuration of USART for enabling use in Asynchronous Mode (UART).
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigAsyncMode \n
  *  CR2          CLKEN         LL_USART_ConfigAsyncMode \n
  *  CR3          SCEN          LL_USART_ConfigAsyncMode \n
  *  CR3          IREN          LL_USART_ConfigAsyncMode \n
  *  CR3          HDSEL         LL_USART_ConfigAsyncMode
  * @param  p_usart USART Instance
  * @note   In UART mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - CLKEN bit in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear CLKEN in CR2 using @ref LL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Asynchronous Mode
  *         (as Baud Rate, Word length, Parity, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigAsyncMode(USART_TypeDef *p_usart)
{
  /* In Asynchronous mode, the following bits must be kept cleared:
  - LINEN, CLKEN bits in the USART_CR2 register,
  - SCEN, IREN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
}


/**
  * @brief  Perform basic configuration of USART for enabling use in Synchronous Mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigSyncMasterMode \n
  *  CR2          CLKEN         LL_USART_ConfigSyncMasterMode \n
  *  CR3          SCEN          LL_USART_ConfigSyncMasterMode \n
  *  CR3          IREN          LL_USART_ConfigSyncMasterMode \n
  *  CR3          HDSEL         LL_USART_ConfigSyncMasterMode
  * @param  p_usart USART Instance
  * @note   In Synchronous mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  *         This function also sets the USART in Synchronous mode.
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  *         - Set CLKEN in CR2 using @ref LL_USART_EnableSCLKOutput() function
  * @note   Other remaining configurations items related to Synchronous Mode
  *         (as Baud Rate, Word length, Parity, Clock Polarity, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigSyncMasterMode(USART_TypeDef *p_usart)
{
  /* In Synchronous mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CR2 register,
  - SCEN, IREN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_SLVEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
  /* set the UART/USART in Synchronous mode */
  STM32_SET_BIT(p_usart->CR2, USART_CR2_CLKEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Synchronous Slave Mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigSyncMasterMode \n
  *  CR2          CLKEN         LL_USART_ConfigSyncMasterMode \n
  *  CR3          SCEN          LL_USART_ConfigSyncMasterMode \n
  *  CR3          IREN          LL_USART_ConfigSyncMasterMode \n
  *  CR3          HDSEL         LL_USART_ConfigSyncMasterMode
  * @param  p_usart USART Instance
  * @note   In Synchronous mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  *         This function also sets the USART in Synchronous mode.
  * @note   Macro IS_USART_INSTANCE(p_usart) can be used to check whether or not
  *         Synchronous mode is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  *         - Set CLKEN in CR2 using @ref LL_USART_EnableSCLKOutput() function
  * @note   Other remaining configurations items related to Synchronous Mode
  *         (as Baud Rate, Word length, Parity, Clock Polarity, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigSyncSlaveMode(USART_TypeDef *p_usart)
{
  /* In Synchronous mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CR2 register,
  - SCEN, IREN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));
  /* set the UART/USART in Synchronous mode */
  STM32_SET_BIT(p_usart->CR2, USART_CR2_SLVEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in LIN Mode.
  * @rmtoll
  *  CR2          CLKEN         LL_USART_ConfigLINMode \n
  *  CR2          STOP          LL_USART_ConfigLINMode \n
  *  CR2          LINEN         LL_USART_ConfigLINMode \n
  *  CR3          IREN          LL_USART_ConfigLINMode \n
  *  CR3          SCEN          LL_USART_ConfigLINMode \n
  *  CR3          HDSEL         LL_USART_ConfigLINMode
  * @param  p_usart USART Instance
  * @note   In LIN mode, the following bits must be kept cleared:
  *           - STOP and CLKEN bits in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  *         This function also set the UART/USART in LIN mode.
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear CLKEN in CR2 using @ref LL_USART_DisableSCLKOutput() function
  *         - Clear STOP in CR2 using @ref LL_USART_SetStopBitsLength() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  *         - Set LINEN in CR2 using @ref LL_USART_EnableLIN() function
  * @note   Other remaining configurations items related to LIN Mode
  *         (as Baud Rate, Word length, LIN Break Detection Length, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigLINMode(USART_TypeDef *p_usart)
{
  /* In LIN mode, the following bits must be kept cleared:
  - STOP and CLKEN bits in the USART_CR2 register,
  - IREN, SCEN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_CLKEN | USART_CR2_STOP));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_IREN | USART_CR3_SCEN | USART_CR3_HDSEL));
  /* Set the UART/USART in LIN mode */
  STM32_SET_BIT(p_usart->CR2, USART_CR2_LINEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Half Duplex Mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigHalfDuplexMode \n
  *  CR2          CLKEN         LL_USART_ConfigHalfDuplexMode \n
  *  CR3          HDSEL         LL_USART_ConfigHalfDuplexMode \n
  *  CR3          SCEN          LL_USART_ConfigHalfDuplexMode \n
  *  CR3          IREN          LL_USART_ConfigHalfDuplexMode
  * @param  p_usart USART Instance
  * @note   In Half Duplex mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - CLKEN bit in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *         This function also sets the UART/USART in Half Duplex mode.
  * @note   Macro IS_UART_HALFDUPLEX_INSTANCE(p_usart) can be used to check whether or not
  *         Half-Duplex mode is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear CLKEN in CR2 using @ref LL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Set HDSEL in CR3 using @ref LL_USART_EnableHalfDuplex() function
  * @note   Other remaining configurations items related to Half Duplex Mode
  *         (as Baud Rate, Word length, Parity, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigHalfDuplexMode(USART_TypeDef *p_usart)
{
  /* In Half Duplex mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN and IREN bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_IREN));
  /* set the UART/USART in Half Duplex mode */
  STM32_SET_BIT(p_usart->CR3, USART_CR3_HDSEL);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Smartcard Mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigSmartcardMode \n
  *  CR2          STOP          LL_USART_ConfigSmartcardMode \n
  *  CR2          CLKEN         LL_USART_ConfigSmartcardMode \n
  *  CR3          HDSEL         LL_USART_ConfigSmartcardMode \n
  *  CR3          SCEN          LL_USART_ConfigSmartcardMode
  * @param  p_usart USART Instance
  * @note   In Smartcard mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  *         This function also configures Stop bits to 1.5 bits and
  *         sets the USART in Smartcard mode (SCEN bit).
  *         Clock Output is also enabled (CLKEN).
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  *         - Configure STOP in CR2 using @ref LL_USART_SetStopBitsLength() function
  *         - Set CLKEN in CR2 using @ref LL_USART_EnableSCLKOutput() function
  *         - Set SCEN in CR3 using @ref LL_USART_EnableSmartcard() function
  * @note   Other remaining configurations items related to Smartcard Mode
  *         (as Baud Rate, Word length, Parity, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigSmartcardMode(USART_TypeDef *p_usart)
{
  /* In Smartcard mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CR2 register,
  - IREN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_IREN | USART_CR3_HDSEL));
  /* Configure Stop bits to 1.5 bits */
  /* Synchronous mode is activated by default */
  STM32_SET_BIT(p_usart->CR2, (USART_CR2_STOP_0 | USART_CR2_STOP_1 | USART_CR2_CLKEN));
  /* set the UART/USART in Smartcard mode */
  STM32_SET_BIT(p_usart->CR3, USART_CR3_SCEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Irda Mode.
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigIrdaMode \n
  *  CR2          CLKEN         LL_USART_ConfigIrdaMode \n
  *  CR2          STOP          LL_USART_ConfigIrdaMode \n
  *  CR3          SCEN          LL_USART_ConfigIrdaMode \n
  *  CR3          HDSEL         LL_USART_ConfigIrdaMode \n
  *  CR3          IREN          LL_USART_ConfigIrdaMode
  * @param  p_usart USART Instance
  * @note   In IRDA mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - STOP and CLKEN bits in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  *         This function also sets the UART/USART in IRDA mode (IREN bit).
  * @note   Macro IS_IRDA_INSTANCE(p_usart) can be used to check whether or not
  *         IrDA feature is supported by the p_usart instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear CLKEN in CR2 using @ref LL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  *         - Configure STOP in CR2 using @ref LL_USART_SetStopBitsLength() function
  *         - Set IREN in CR3 using @ref LL_USART_EnableIrda() function
  * @note   Other remaining configurations items related to Irda Mode
  *         (as Baud Rate, Word length, Power mode, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigIrdaMode(USART_TypeDef *p_usart)
{
  /* In IRDA mode, the following bits must be kept cleared:
  - LINEN, STOP and CLKEN bits in the USART_CR2 register,
  - SCEN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN | USART_CR2_STOP));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL));
  /* set the UART/USART in IRDA mode */
  STM32_SET_BIT(p_usart->CR3, USART_CR3_IREN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Multi processor Mode
  *         (several USARTs connected in a network, one of the USARTs can be the master,
  *         its TX output connected to the RX inputs of the other slaves USARTs).
  * @rmtoll
  *  CR2          LINEN         LL_USART_ConfigMultiProcessMode \n
  *  CR2          CLKEN         LL_USART_ConfigMultiProcessMode \n
  *  CR3          SCEN          LL_USART_ConfigMultiProcessMode \n
  *  CR3          HDSEL         LL_USART_ConfigMultiProcessMode \n
  *  CR3          IREN          LL_USART_ConfigMultiProcessMode
  * @param  p_usart USART Instance
  * @note   In MultiProcessor mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CR2 register,
  *           - CLKEN bit in the USART_CR2 register,
  *           - SCEN bit in the USART_CR3 register,
  *           - IREN bit in the USART_CR3 register,
  *           - HDSEL bit in the USART_CR3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CR2 using @ref LL_USART_DisableLIN() function
  *         - Clear CLKEN in CR2 using @ref LL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CR3 using @ref LL_USART_DisableSmartcard() function
  *         - Clear IREN in CR3 using @ref LL_USART_DisableIrda() function
  *         - Clear HDSEL in CR3 using @ref LL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Multi processor Mode
  *         (as Baud Rate, Wake Up Method, Node address, ...) must be set using
  *         dedicated functions
  */
__STATIC_INLINE void LL_USART_ConfigMultiProcessMode(USART_TypeDef *p_usart)
{
  /* In Multi Processor mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - IREN, SCEN and HDSEL bits in the USART_CR3 register.
  */
  STM32_CLEAR_BIT(p_usart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  STM32_CLEAR_BIT(p_usart->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if the USART Flag Mask is set or not.
  * @rmtoll
  *  ISR          PE            LL_USART_IsActiveFlag \n
  *  ISR          FE            LL_USART_IsActiveFlag \n
  *  ISR          NE            LL_USART_IsActiveFlag \n
  *  ISR          ORE           LL_USART_IsActiveFlag \n
  *  ISR          IDLE          LL_USART_IsActiveFlag \n
  *  ISR          RXNE_RXFNE    LL_USART_IsActiveFlag \n
  *  ISR          TC            LL_USART_IsActiveFlag \n
  *  ISR          TXE_TXFNF     LL_USART_IsActiveFlag \n
  *  ISR          LBDF          LL_USART_IsActiveFlag \n
  *  ISR          CTSIF         LL_USART_IsActiveFlag \n
  *  ISR          CTS           LL_USART_IsActiveFlag \n
  *  ISR          RTOF          LL_USART_IsActiveFlag \n
  *  ISR          EOBF          LL_USART_IsActiveFlag \n
  *  ISR          UDR           LL_USART_IsActiveFlag \n
  *  ISR          ABRE          LL_USART_IsActiveFlag \n
  *  ISR          ABRF          LL_USART_IsActiveFlag \n
  *  ISR          BUSY          LL_USART_IsActiveFlag \n
  *  ISR          CMF           LL_USART_IsActiveFlag \n
  *  ISR          SBKF          LL_USART_IsActiveFlag \n
  *  ISR          RWU           LL_USART_IsActiveFlag \n
  *  ISR          WUF           LL_USART_IsActiveFlag \n
  *  ISR          TEACK         LL_USART_IsActiveFlag \n
  *  ISR          REACK         LL_USART_IsActiveFlag \n
  *  ISR          TXFE          LL_USART_IsActiveFlag \n
  *  ISR          RXFF          LL_USART_IsActiveFlag \n
  *  ISR          TCBGT         LL_USART_IsActiveFlag \n
  *  ISR          TXFT          LL_USART_IsActiveFlag \n
  *  ISR          RXFT          LL_USART_IsActiveFlag
  * @param  p_usart USART Instance
  * @param  mask Mask to tests
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag(const USART_TypeDef *p_usart, uint32_t mask)
{
  return ((STM32_READ_BIT(p_usart->ISR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Parity Error Flag is set or not.
  * @rmtoll
  *  ISR          PE            LL_USART_IsActiveFlag_PE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_PE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_PE) == (USART_ISR_PE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Framing Error Flag is set or not.
  * @rmtoll
  *  ISR          FE            LL_USART_IsActiveFlag_FE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_FE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_FE) == (USART_ISR_FE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Noise error detected Flag is set or not.
  * @rmtoll
  *  ISR          NE            LL_USART_IsActiveFlag_NE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_NE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_NE) == (USART_ISR_NE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART OverRun Error Flag is set or not.
  * @rmtoll
  *  ISR          ORE           LL_USART_IsActiveFlag_ORE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_ORE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_ORE) == (USART_ISR_ORE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the USART IDLE line detected flag is set.
  * @rmtoll
  *  ISR          IDLE          LL_USART_IsActiveFlag_IDLE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_IDLE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_IDLE) == (USART_ISR_IDLE)) ? 1UL : 0UL);
}


/**
  * @brief  Check whether the USART Read Data Register or USART RX FIFO Not Empty Flag is set.
  * @rmtoll
  *  ISR          RXNE_RXFNE    LL_USART_IsActiveFlag_RXNE_RXFNE
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_RXNE_RXFNE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_RXNE_RXFNE) == (USART_ISR_RXNE_RXFNE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the USART Transmission Complete Flag is set.
  * @rmtoll
  *  ISR          TC            LL_USART_IsActiveFlag_TC
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TC(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TC) == (USART_ISR_TC)) ? 1UL : 0UL);
}


/**
  * @brief  Check whether the USART Transmit Data Register Empty or USART TX FIFO Not Full Flag is set.
  * @rmtoll
  *  ISR          TXE_TXFNF     LL_USART_IsActiveFlag_TXE_TXFNF
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TXE_TXFNF(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TXE_TXFNF) == (USART_ISR_TXE_TXFNF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART LIN Break Detection Flag is set or not.
  * @rmtoll
  *  ISR          LBDF          LL_USART_IsActiveFlag_LBD
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_LBD(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_LBDF) == (USART_ISR_LBDF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART CTS interrupt Flag is set or not.
  * @rmtoll
  *  ISR          CTSIF         LL_USART_IsActiveFlag_nCTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_nCTS(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_CTSIF) == (USART_ISR_CTSIF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART CTS Flag is set or not.
  * @rmtoll
  *  ISR          CTS           LL_USART_IsActiveFlag_CTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_CTS(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_CTS) == (USART_ISR_CTS)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Receiver Time Out Flag is set or not.
  * @rmtoll
  *  ISR          RTOF          LL_USART_IsActiveFlag_RTO
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_RTO(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_RTOF) == (USART_ISR_RTOF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART End Of Block Flag is set or not.
  * @rmtoll
  *  ISR          EOBF          LL_USART_IsActiveFlag_EOB
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_EOB(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_EOBF) == (USART_ISR_EOBF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the SPI Slave Underrun error flag is set or not.
  * @rmtoll
  *  ISR          UDR           LL_USART_IsActiveFlag_UDR
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_UDR(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_UDR) == (USART_ISR_UDR)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Auto-Baud Rate Error Flag is set or not.
  * @rmtoll
  *  ISR          ABRE          LL_USART_IsActiveFlag_ABRE
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_ABRE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_ABRE) == (USART_ISR_ABRE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Auto-Baud Rate Flag is set or not.
  * @rmtoll
  *  ISR          ABRF          LL_USART_IsActiveFlag_ABR
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_ABR(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_ABRF) == (USART_ISR_ABRF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Busy Flag is set or not.
  * @rmtoll
  *  ISR          BUSY          LL_USART_IsActiveFlag_BUSY
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_BUSY(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_BUSY) == (USART_ISR_BUSY)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Character Match Flag is set or not.
  * @rmtoll
  *  ISR          CMF           LL_USART_IsActiveFlag_CM
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_CM(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_CMF) == (USART_ISR_CMF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Send Break Flag is set or not.
  * @rmtoll
  *  ISR          SBKF          LL_USART_IsActiveFlag_SBK
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_SBK(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_SBKF) == (USART_ISR_SBKF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Receive Wake Up from mute mode Flag is set or not.
  * @rmtoll
  *  ISR          RWU           LL_USART_IsActiveFlag_RWU
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_RWU(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_RWU) == (USART_ISR_RWU)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Wake Up from stop mode Flag is set or not.
  * @rmtoll
  *  ISR          WUF           LL_USART_IsActiveFlag_WKUP
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_WKUP(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_WUF) == (USART_ISR_WUF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Transmit Enable Acknowledge Flag is set or not.
  * @rmtoll
  *  ISR          TEACK         LL_USART_IsActiveFlag_TEACK
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TEACK(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TEACK) == (USART_ISR_TEACK)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the USART Receive Enable Acknowledge Flag is set.
  * @rmtoll
  *  ISR          REACK         LL_USART_IsActiveFlag_REACK
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_REACK(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_REACK) == (USART_ISR_REACK)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the USART TX FIFO Empty Flag is set.
  * @rmtoll
  *  ISR          TXFE          LL_USART_IsActiveFlag_TXFE
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TXFE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TXFE) == (USART_ISR_TXFE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether the USART RX FIFO Full Flag is set.
  * @rmtoll
  *  ISR          RXFF          LL_USART_IsActiveFlag_RXFF
  * @param  p_usart USART Instance
  * @note   Use macro IS_UART_FIFO_INSTANCE(p_usart) to check whether
  *         the FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_RXFF(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_RXFF) == (USART_ISR_RXFF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the Smartcard Transmission Complete Before Guard Time Flag is set or not.
  * @rmtoll
  *  ISR          TCBGT         LL_USART_IsActiveFlag_TCBGT
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TCBGT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TCBGT) == (USART_ISR_TCBGT)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART TX FIFO Threshold Flag is set or not.
  * @rmtoll
  *  ISR          TXFT          LL_USART_IsActiveFlag_TXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TXFT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_TXFT) == (USART_ISR_TXFT)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART RX FIFO Threshold Flag is set or not.
  * @rmtoll
  *  ISR          RXFT          LL_USART_IsActiveFlag_RXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsActiveFlag_RXFT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->ISR, USART_ISR_RXFT) == (USART_ISR_RXFT)) ? 1UL : 0UL);
}


/**
  * @brief  Clear the Flag Mask.
  * @rmtoll
  *  ISR          PE            LL_USART_ClearFlag \n
  *  ISR          FE            LL_USART_ClearFlag \n
  *  ISR          NE            LL_USART_ClearFlag \n
  *  ISR          ORE           LL_USART_ClearFlag \n
  *  ISR          IDLE          LL_USART_ClearFlag \n
  *  ISR          TC            LL_USART_ClearFlag \n
  *  ISR          LBDF          LL_USART_ClearFlag \n
  *  ISR          CTS           LL_USART_ClearFlag \n
  *  ISR          RTOF          LL_USART_ClearFlag \n
  *  ISR          EOBF          LL_USART_ClearFlag \n
  *  ISR          UDR           LL_USART_ClearFlag \n
  *  ISR          CMF           LL_USART_ClearFlag \n
  *  ISR          WUF           LL_USART_ClearFlag \n
  *  ISR          TXFE          LL_USART_ClearFlag \n
  *  ISR          TCBGT         LL_USART_ClearFlag
  * @param  p_usart USART Instance
  * @param  mask Mask to tests
  */
__STATIC_INLINE void LL_USART_ClearFlag(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_WRITE_REG(p_usart->ICR, mask);
}

/**
  * @brief  Clear Parity Error Flag.
  * @rmtoll
  *  ICR          PECF          LL_USART_ClearFlag_PE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_PE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_PECF);
}

/**
  * @brief  Clear Framing Error Flag.
  * @rmtoll
  *  ICR          FECF          LL_USART_ClearFlag_FE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_FE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_FECF);
}

/**
  * @brief  Clear Noise Error detected Flag.
  * @rmtoll
  *  ICR          NECF           LL_USART_ClearFlag_NE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_NE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_NECF);
}

/**
  * @brief  Clear OverRun Error Flag.
  * @rmtoll
  *  ICR          ORECF         LL_USART_ClearFlag_ORE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_ORE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_ORECF);
}

/**
  * @brief  Clear IDLE line detected Flag.
  * @rmtoll
  *  ICR          IDLECF        LL_USART_ClearFlag_IDLE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_IDLE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_IDLECF);
}

/**
  * @brief  Clear TX FIFO Empty Flag.
  * @rmtoll
  *  ICR          TXFECF        LL_USART_ClearFlag_TXFE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_TXFE(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_TXFECF);
}

/**
  * @brief  Clear Transmission Complete Flag.
  * @rmtoll
  *  ICR          TCCF          LL_USART_ClearFlag_TC
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_TC(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_TCCF);
}

/**
  * @brief  Clear Smartcard Transmission Complete Before Guard Time Flag.
  * @rmtoll
  *  ICR          TCBGTCF       LL_USART_ClearFlag_TCBGT
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_TCBGT(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_TCBGTCF);
}

/**
  * @brief  Clear LIN Break Detection Flag.
  * @rmtoll
  *  ICR          LBDCF         LL_USART_ClearFlag_LBD
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_LBD(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_LBDCF);
}

/**
  * @brief  Clear CTS Interrupt Flag.
  * @rmtoll
  *  ICR          CTSCF         LL_USART_ClearFlag_nCTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_nCTS(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_CTSCF);
}

/**
  * @brief  Clear Receiver Time Out Flag.
  * @rmtoll
  *  ICR          RTOCF         LL_USART_ClearFlag_RTO
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_RTO(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_RTOCF);
}

/**
  * @brief  Clear End Of Block Flag.
  * @rmtoll
  *  ICR          EOBCF         LL_USART_ClearFlag_EOB
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_EOB(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_EOBCF);
}

/**
  * @brief  Clear SPI Slave Underrun Flag.
  * @rmtoll
  *  ICR          UDRCF         LL_USART_ClearFlag_UDR
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_SPI_SLAVE_INSTANCE(p_usart) can be used to check whether or not
  *         SPI Slave mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_UDR(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_UDRCF);
}

/**
  * @brief  Clear Character Match Flag.
  * @rmtoll
  *  ICR          CMCF          LL_USART_ClearFlag_CM
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_ClearFlag_CM(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_CMCF);
}

/**
  * @brief  Clear Wake Up from stop mode Flag.
  * @rmtoll
  *  ICR          WUCF          LL_USART_ClearFlag_WKUP
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_ClearFlag_WKUP(USART_TypeDef *p_usart)
{
  STM32_WRITE_REG(p_usart->ICR, USART_ICR_WUCF);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable CR1 Interrupt.
  * @rmtoll
  *  CR1          IDLEIE         LL_USART_EnableIT_CR1 \n
  *  CR1          RXNEIE_RXFNEIE LL_USART_EnableIT_CR1 \n
  *  CR1          TCIE           LL_USART_EnableIT_CR1 \n
  *  CR1          TXEIE_TXFNFIE  LL_USART_EnableIT_CR1 \n
  *  CR1          PEIE           LL_USART_EnableIT_CR1 \n
  *  CR1          CMIE           LL_USART_EnableIT_CR1 \n
  *  CR1          RTOIE          LL_USART_EnableIT_CR1 \n
  *  CR1          EOBIE          LL_USART_EnableIT_CR1 \n
  *  CR1          TXFEIE         LL_USART_EnableIT_CR1 \n
  *  CR1          RXFFIE         LL_USART_EnableIT_CR1
  * @param  p_usart USART Instance
  * @param  mask   mask to enable
  */
__STATIC_INLINE void LL_USART_EnableIT_CR1(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, mask);
}
/**
  * @brief  Enable IDLE Interrupt.
  * @rmtoll
  *  CR1          IDLEIE        LL_USART_EnableIT_IDLE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_IDLE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_IDLEIE);
}


/**
  * @brief  Enable RX Not Empty and RX FIFO Not Empty Interrupt.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_USART_EnableIT_RXNE_RXFNE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_RXNE_RXFNE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

/**
  * @brief  Enable Transmission Complete Interrupt.
  * @rmtoll
  *  CR1          TCIE          LL_USART_EnableIT_TC
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_TC(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_TCIE);
}


/**
  * @brief  Enable TX Empty and TX FIFO Not Full Interrupt.
  * @rmtoll
  *  CR1         TXEIE_TXFNFIE  LL_USART_EnableIT_TXE_TXFNF
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_TXE_TXFNF(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
  * @brief  Enable Parity Error Interrupt.
  * @rmtoll
  *  CR1          PEIE          LL_USART_EnableIT_PE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_PE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_PEIE);
}

/**
  * @brief  Enable Character Match Interrupt.
  * @rmtoll
  *  CR1          CMIE          LL_USART_EnableIT_CM
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_CM(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_CMIE);
}

/**
  * @brief  Enable Receiver Timeout Interrupt.
  * @rmtoll
  *  CR1          RTOIE         LL_USART_EnableIT_RTO
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_RTO(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_RTOIE);
}

/**
  * @brief  Enable End Of Block Interrupt.
  * @rmtoll
  *  CR1          EOBIE         LL_USART_EnableIT_EOB
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_EOB(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_EOBIE);
}

/**
  * @brief  Enable TX FIFO Empty Interrupt.
  * @rmtoll
  *  CR1          TXFEIE        LL_USART_EnableIT_TXFE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_TXFE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_TXFEIE);
}

/**
  * @brief  Enable RX FIFO Full Interrupt.
  * @rmtoll
  *  CR1          RXFFIE        LL_USART_EnableIT_RXFF
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableIT_RXFF(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR1, USART_CR1_RXFFIE);
}

/**
  * @brief  Enable CR2 Interrupt.
  * @rmtoll
  *  CR2          LBDIE         LL_USART_EnableIT_CR2
  * @param  p_usart USART Instance
  * @param  mask   mask to enable
  */
__STATIC_INLINE void LL_USART_EnableIT_CR2(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR2, mask);
}

/**
  * @brief  Enable LIN Break Detection Interrupt.
  * @rmtoll
  *  CR2          LBDIE         LL_USART_EnableIT_LBD
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_LBD(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR2, USART_CR2_LBDIE);
}

/**
  * @brief  Enable CR3 Interrupt.
  * @rmtoll
  *  CR3          EIE            LL_USART_EnableIT_CR3 \n
  *  CR3          CTSIE          LL_USART_EnableIT_CR3 \n
  *  CR3          WUFIE          LL_USART_EnableIT_CR3 \n
  *  CR3          TXFTIE         LL_USART_EnableIT_CR3 \n
  *  CR3          TCBGTIE        LL_USART_EnableIT_CR3 \n
  *  CR3          RXFTIE         LL_USART_EnableIT_CR3
  * @param  p_usart USART Instance
  * @param  mask   mask to enable
  */
__STATIC_INLINE void LL_USART_EnableIT_CR3(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, mask);
}

/**
  * @brief  Enable Error Interrupt.
  * @rmtoll
  *  CR3          EIE           LL_USART_EnableIT_ERROR
  * @param  p_usart USART Instance
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the p_usart ISR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the p_usart ISR register.
  */
__STATIC_INLINE void LL_USART_EnableIT_ERROR(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_EIE);
}

/**
  * @brief  Enable CTS Interrupt.
  * @rmtoll
  *  CR3          CTSIE         LL_USART_EnableIT_CTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_CTS(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_CTSIE);
}

/**
  * @brief  Enable Wake Up from Stop Mode Interrupt.
  * @rmtoll
  *  CR3          WUFIE         LL_USART_EnableIT_WKUP
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_WKUP(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_WUFIE);
}

/**
  * @brief  Enable TX FIFO Threshold Interrupt.
  * @rmtoll
  *  CR3          TXFTIE        LL_USART_EnableIT_TXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_TXFT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_TXFTIE);
}

/**
  * @brief  Enable Smartcard Transmission Complete Before Guard Time Interrupt.
  * @rmtoll
  *  CR3          TCBGTIE       LL_USART_EnableIT_TCBGT
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_TCBGT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_TCBGTIE);
}

/**
  * @brief  Enable RX FIFO Threshold Interrupt.
  * @rmtoll
  *  CR3          RXFTIE        LL_USART_EnableIT_RXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_EnableIT_RXFT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_RXFTIE);
}

/**
  * @brief  Disable CR1 Interrupt.
  * @rmtoll
  *  CR1          IDLEIE         LL_USART_DisableIT_CR1 \n
  *  CR1          RXNEIE_RXFNEIE LL_USART_DisableIT_CR1 \n
  *  CR1          TCIE           LL_USART_DisableIT_CR1 \n
  *  CR1          TXEIE_TXFNFIE  LL_USART_DisableIT_CR1 \n
  *  CR1          PEIE           LL_USART_DisableIT_CR1 \n
  *  CR1          CMIE           LL_USART_DisableIT_CR1 \n
  *  CR1          RTOIE          LL_USART_DisableIT_CR1 \n
  *  CR1          EOBIE          LL_USART_DisableIT_CR1 \n
  *  CR1          TXFEIE         LL_USART_DisableIT_CR1 \n
  *  CR1          RXFFIE         LL_USART_DisableIT_CR1
  * @param  p_usart USART Instance
  * @param  mask   mask to disable
  */
__STATIC_INLINE void LL_USART_DisableIT_CR1(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, mask);
}

/**
  * @brief  Disable IDLE Interrupt.
  * @rmtoll
  *  CR1          IDLEIE        LL_USART_DisableIT_IDLE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableIT_IDLE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_IDLEIE);
}


/**
  * @brief  Disable RX Not Empty and RX FIFO Not Empty Interrupt.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_USART_DisableIT_RXNE_RXFNE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_RXNE_RXFNE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

/**
  * @brief  Disable Transmission Complete Interrupt.
  * @rmtoll
  *  CR1          TCIE          LL_USART_DisableIT_TC
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableIT_TC(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_TCIE);
}


/**
  * @brief  Disable TX Empty and TX FIFO Not Full Interrupt.
  * @rmtoll
  *  CR1        TXEIE_TXFNFIE  LL_USART_DisableIT_TXE_TXFNF
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_TXE_TXFNF(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
  * @brief  Disable Parity Error Interrupt.
  * @rmtoll
  *  CR1          PEIE          LL_USART_DisableIT_PE
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableIT_PE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_PEIE);
}

/**
  * @brief  Disable Character Match Interrupt.
  * @rmtoll
  *  CR1          CMIE          LL_USART_DisableIT_CM
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableIT_CM(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_CMIE);
}

/**
  * @brief  Disable Receiver Timeout Interrupt.
  * @rmtoll
  *  CR1          RTOIE         LL_USART_DisableIT_RTO
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableIT_RTO(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_RTOIE);
}

/**
  * @brief  Disable End Of Block Interrupt.
  * @rmtoll
  *  CR1          EOBIE         LL_USART_DisableIT_EOB
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_EOB(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_EOBIE);
}

/**
  * @brief  Disable TX FIFO Empty Interrupt.
  * @rmtoll
  *  CR1          TXFEIE        LL_USART_DisableIT_TXFE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_TXFE(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_TXFEIE);
}

/**
  * @brief  Disable RX FIFO Full Interrupt.
  * @rmtoll
  *  CR1          RXFFIE        LL_USART_DisableIT_RXFF
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_RXFF(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR1, USART_CR1_RXFFIE);
}


/**
  * @brief  Disable CR2 Interrupt.
  * @rmtoll
  *  CR2          LBDIE         LL_USART_DisableIT_CR2
  * @param  p_usart USART Instance
  * @param  mask   mask to disable
  */
__STATIC_INLINE void LL_USART_DisableIT_CR2(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR2, mask);
}

/**
  * @brief  Disable LIN Break Detection Interrupt.
  * @rmtoll
  *  CR2          LBDIE         LL_USART_DisableIT_LBD
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_LBD(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR2, USART_CR2_LBDIE);
}

/**
  * @brief  Disable CR3 Interrupt.
  * @rmtoll
  *  CR3          EIE            LL_USART_DisableIT_CR3 \n
  *  CR3          CTSIE          LL_USART_DisableIT_CR3 \n
  *  CR3          WUFIE          LL_USART_DisableIT_CR3 \n
  *  CR3          TXFTIE         LL_USART_DisableIT_CR3 \n
  *  CR3          TCBGTIE        LL_USART_DisableIT_CR3 \n
  *  CR3          RXFTIE         LL_USART_DisableIT_CR3
  * @param  p_usart USART Instance
  * @param  mask   mask to disable
  */
__STATIC_INLINE void LL_USART_DisableIT_CR3(USART_TypeDef *p_usart, uint32_t mask)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, mask);
}

/**
  * @brief  Disable Error Interrupt.
  * @rmtoll
  *  CR3          EIE           LL_USART_DisableIT_ERROR
  * @param  p_usart USART Instance
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the p_usart ISR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the p_usart ISR register.
  */
__STATIC_INLINE void LL_USART_DisableIT_ERROR(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_EIE);
}

/**
  * @brief  Disable CTS Interrupt.
  * @rmtoll
  *  CR3          CTSIE         LL_USART_DisableIT_CTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_CTS(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_CTSIE);
}

/**
  * @brief  Disable Wake Up from Stop Mode Interrupt.
  * @rmtoll
  *  CR3          WUFIE         LL_USART_DisableIT_WKUP
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_WKUP(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_WUFIE);
}

/**
  * @brief  Disable TX FIFO Threshold Interrupt.
  * @rmtoll
  *  CR3          TXFTIE        LL_USART_DisableIT_TXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_TXFT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_TXFTIE);
}

/**
  * @brief  Disable Smartcard Transmission Complete Before Guard Time Interrupt.
  * @rmtoll
  *  CR3          TCBGTIE       LL_USART_DisableIT_TCBGT
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_TCBGT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_TCBGTIE);
}

/**
  * @brief  Disable RX FIFO Threshold Interrupt.
  * @rmtoll
  *  CR3          RXFTIE        LL_USART_DisableIT_RXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_DisableIT_RXFT(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_RXFTIE);
}

/**
  * @brief  Check if the USART IDLE Interrupt  source is enabled or disabled.
  * @rmtoll
  *  CR1          IDLEIE        LL_USART_IsEnabledIT_IDLE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_IDLE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_IDLEIE) == (USART_CR1_IDLEIE)) ? 1UL : 0UL);
}


/**
  * @brief  Check if the USART RX Not Empty and USART RX FIFO Not Empty Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1        RXNEIE_RXFNEIE  LL_USART_IsEnabledIT_RXNE_RXFNE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_RXNE_RXFNE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_RXNEIE_RXFNEIE) == (USART_CR1_RXNEIE_RXFNEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Transmission Complete Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TCIE          LL_USART_IsEnabledIT_TC
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_TC(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_TCIE) == (USART_CR1_TCIE)) ? 1UL : 0UL);
}


/**
  * @brief  Check if the USART TX Empty and USART TX FIFO Not Full Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1         TXEIE_TXFNFIE  LL_USART_IsEnabledIT_TXE_TXFNF
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_TXE_TXFNF(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_TXEIE_TXFNFIE) == (USART_CR1_TXEIE_TXFNFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Parity Error Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          PEIE          LL_USART_IsEnabledIT_PE
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_PE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_PEIE) == (USART_CR1_PEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Character Match Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          CMIE          LL_USART_IsEnabledIT_CM
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_CM(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_CMIE) == (USART_CR1_CMIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Receiver Timeout Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          RTOIE         LL_USART_IsEnabledIT_RTO
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_RTO(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_RTOIE) == (USART_CR1_RTOIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART End Of Block Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          EOBIE         LL_USART_IsEnabledIT_EOB
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_EOB(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_EOBIE) == (USART_CR1_EOBIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART TX FIFO Empty Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          TXFEIE        LL_USART_IsEnabledIT_TXFE
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_TXFE(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_TXFEIE) == (USART_CR1_TXFEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART RX FIFO Full Interrupt is enabled or disabled.
  * @rmtoll
  *  CR1          RXFFIE        LL_USART_IsEnabledIT_RXFF
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_RXFF(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR1, USART_CR1_RXFFIE) == (USART_CR1_RXFFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART LIN Break Detection Interrupt is enabled or disabled.
  * @rmtoll
  *  CR2          LBDIE         LL_USART_IsEnabledIT_LBD
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_LIN_INSTANCE(p_usart) can be used to check whether or not
  *         LIN feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_LBD(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR2, USART_CR2_LBDIE) == (USART_CR2_LBDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Error Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          EIE           LL_USART_IsEnabledIT_ERROR
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_ERROR(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_EIE) == (USART_CR3_EIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART CTS Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          CTSIE         LL_USART_IsEnabledIT_CTS
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_HWFLOW_INSTANCE(p_usart) can be used to check whether or not
  *         Hardware Flow control feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_CTS(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_CTSIE) == (USART_CR3_CTSIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the USART Wake Up from Stop Mode Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          WUFIE         LL_USART_IsEnabledIT_WKUP
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_WAKEUP_FROMSTOP_INSTANCE(p_usart) can be used to check whether or not
  *         Wake-up from Stop mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_WKUP(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_WUFIE) == (USART_CR3_WUFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if USART TX FIFO Threshold Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          TXFTIE        LL_USART_IsEnabledIT_TXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_TXFT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_TXFTIE) == (USART_CR3_TXFTIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if the Smartcard Transmission Complete Before Guard Time Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          TCBGTIE       LL_USART_IsEnabledIT_TCBGT
  * @param  p_usart USART Instance
  * @note   Macro IS_SMARTCARD_INSTANCE(p_usart) can be used to check whether or not
  *         Smartcard feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_TCBGT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_TCBGTIE) == (USART_CR3_TCBGTIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check if USART RX FIFO Threshold Interrupt is enabled or disabled.
  * @rmtoll
  *  CR3          RXFTIE        LL_USART_IsEnabledIT_RXFT
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledIT_RXFT(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_RXFTIE) == (USART_CR3_RXFTIE)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Mode for reception.
  * @rmtoll
  *  CR3          DMAR          LL_USART_EnableDMAReq_RX
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableDMAReq_RX(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Disable DMA Mode for reception.
  * @rmtoll
  *  CR3          DMAR          LL_USART_DisableDMAReq_RX
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableDMAReq_RX(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_DMAR);
}

/**
  * @brief  Check if DMA Mode is enabled for reception.
  * @rmtoll
  *  CR3          DMAR          LL_USART_IsEnabledDMAReq_RX
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDMAReq_RX(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_DMAR) == (USART_CR3_DMAR)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA Mode for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_USART_EnableDMAReq_TX
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableDMAReq_TX(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_SET_BIT_32(p_usart->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Disable DMA Mode for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_USART_DisableDMAReq_TX
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableDMAReq_TX(USART_TypeDef *p_usart)
{
  STM32_ATOMIC_CLEAR_BIT_32(p_usart->CR3, USART_CR3_DMAT);
}

/**
  * @brief  Check if DMA Mode is enabled for transmission.
  * @rmtoll
  *  CR3          DMAT          LL_USART_IsEnabledDMAReq_TX
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDMAReq_TX(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_DMAT) == (USART_CR3_DMAT)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA Disabling on Reception Error.
  * @rmtoll
  *  CR3          DDRE          LL_USART_EnableDMADeactOnRxErr
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_EnableDMADeactOnRxErr(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->CR3, USART_CR3_DDRE);
}

/**
  * @brief  Disable DMA Disabling on Reception Error.
  * @rmtoll
  *  CR3          DDRE          LL_USART_DisableDMADeactOnRxErr
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_DisableDMADeactOnRxErr(USART_TypeDef *p_usart)
{
  STM32_CLEAR_BIT(p_usart->CR3, USART_CR3_DDRE);
}

/**
  * @brief  Indicate if DMA Disabling on Reception Error is disabled.
  * @rmtoll
  *  CR3          DDRE          LL_USART_IsEnabledDMADeactOnRxErr
  * @param  p_usart USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_USART_IsEnabledDMADeactOnRxErr(const USART_TypeDef *p_usart)
{
  return ((STM32_READ_BIT(p_usart->CR3, USART_CR3_DDRE) == (USART_CR3_DDRE)) ? 1UL : 0UL);
}

/**
  * @brief  Get the data register address used for DMA transfer.
  * @rmtoll
  *  RDR          RDR           LL_USART_DMA_GetRegAddr \n
  *  TDR          TDR           LL_USART_DMA_GetRegAddr
  * @param  p_usart USART Instance
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_USART_DMA_REG_DATA_TRANSMIT
  *         @arg @ref LL_USART_DMA_REG_DATA_RECEIVE
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_USART_DMA_GetRegAddr(const USART_TypeDef *p_usart, uint32_t direction)
{
  uint32_t data_reg_addr;

  if (direction == LL_USART_DMA_REG_DATA_TRANSMIT)
  {
    /* return address of TDR register */
    data_reg_addr = (uint32_t) &(p_usart->TDR);
  }
  else
  {
    /* return address of RDR register */
    data_reg_addr = (uint32_t) &(p_usart->RDR);
  }

  return data_reg_addr;
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read Receiver Data register (Receive Data value, 8 bits).
  * @rmtoll
  *  RDR          RDR           LL_USART_ReceiveData8
  * @param  p_usart USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t LL_USART_ReceiveData8(const USART_TypeDef *p_usart)
{
  return (uint8_t)(STM32_READ_BIT(p_usart->RDR, USART_RDR_RDR) & 0xFFU);
}

/**
  * @brief  Read Receiver Data register (Receive Data value, 9 bits).
  * @rmtoll
  *  RDR          RDR           LL_USART_ReceiveData9
  * @param  p_usart USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE uint16_t LL_USART_ReceiveData9(const USART_TypeDef *p_usart)
{
  return (uint16_t)(STM32_READ_BIT(p_usart->RDR, USART_RDR_RDR));
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 8 bits).
  * @rmtoll
  *  TDR          TDR           LL_USART_TransmitData8
  * @param  p_usart USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_USART_TransmitData8(USART_TypeDef *p_usart, uint8_t Value)
{
  p_usart->TDR = Value;
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 9 bits).
  * @rmtoll
  *  TDR          TDR           LL_USART_TransmitData9
  * @param  p_usart USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE void LL_USART_TransmitData9(USART_TypeDef *p_usart, uint16_t Value)
{
  p_usart->TDR = (uint16_t)(Value & 0x1FFUL);
}

/**
  * @}
  */

/** @defgroup USART_LL_EF_Execution Execution
  * @{
  */

/**
  * @brief  Set a Request.
  * @rmtoll
  *  RQR          SBKRQ         LL_USART_SetRequest \n
  *  RQR          MMRQ          LL_USART_SetRequest \n
  *  RQR          RXFRQ         LL_USART_SetRequest \n
  *  RQR          TXFRQ         LL_USART_SetRequest \n
  *  RQR          ABRRQ         LL_USART_SetRequest
  * @param  p_usart USART Instance
  * @param  request Request to set
  *         @arg @ref LL_USART_REQUEST_SEND_BREAK
  *         @arg @ref LL_USART_REQUEST_MUTE_MODE
  *         @arg @ref LL_USART_REQUEST_RX_DATA_FLUSH
  *         @arg @ref LL_USART_REQUEST_TX_DATA_FLUSH
  *         @arg @ref LL_USART_REQUEST_AUTO_BAUD_RATE
  */
__STATIC_INLINE void LL_USART_SetRequest(USART_TypeDef *p_usart, uint32_t request)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)request);
}

/**
  * @brief  Request an Automatic Baud Rate measurement on next received data frame.
  * @rmtoll
  *  RQR          ABRRQ         LL_USART_RequestAutoBaudRate
  * @param  p_usart USART Instance
  * @note   Macro IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(p_usart) can be used to check whether or not
  *         Auto Baud Rate detection feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_RequestAutoBaudRate(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)USART_RQR_ABRRQ);
}

/**
  * @brief  Request Break sending.
  * @rmtoll
  *  RQR          SBKRQ         LL_USART_RequestBreakSending
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_RequestBreakSending(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)USART_RQR_SBKRQ);
}

/**
  * @brief  Put USART in mute mode and set the RWU flag.
  * @rmtoll
  *  RQR          MMRQ          LL_USART_RequestEnterMuteMode
  * @param  p_usart USART Instance
  */
__STATIC_INLINE void LL_USART_RequestEnterMuteMode(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)USART_RQR_MMRQ);
}

/**
  * @brief  Request a Receive Data and FIFO flush.
  * @rmtoll
  *  RQR          RXFRQ         LL_USART_RequestRxDataFlush
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  * @note   Allows to discard the received data without reading them, and avoid an overrun
  *         condition.
  */
__STATIC_INLINE void LL_USART_RequestRxDataFlush(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)USART_RQR_RXFRQ);
}

/**
  * @brief  Request a Transmit data and FIFO flush.
  * @rmtoll
  *  RQR          TXFRQ         LL_USART_RequestTxDataFlush
  * @param  p_usart USART Instance
  * @note   Macro IS_UART_FIFO_INSTANCE(p_usart) can be used to check whether or not
  *         FIFO mode feature is supported by the p_usart instance.
  */
__STATIC_INLINE void LL_USART_RequestTxDataFlush(USART_TypeDef *p_usart)
{
  STM32_SET_BIT(p_usart->RQR, (uint16_t)USART_RQR_TXFRQ);
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

#endif /* USART1 || USART2 || USART3 || UART4 || UART5 || USART6 || UART7 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_USART_H */

