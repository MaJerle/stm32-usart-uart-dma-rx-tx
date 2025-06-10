/*
 * uart_regs.h
 *
 *  Created on: Jun 9, 2025
 *      Author: admin
 */

#ifndef STM32_TOOLS_UART_ENGINE_UARTDEPENCY_H_
#define STM32_TOOLS_UART_ENGINE_UARTDEPENCY_H_

#include "main.h"

#if defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED)
#include "Uartsettings.h"

#if UART_ENGINE_ON


// In new lines (G4, L4, F7, etc.): RDR and ISR
#if defined(USART_RDR_RDR_Msk) && defined(USART_ISR_PE_Msk)
#	define UART_SR_REG(h)     ((h)->Instance->ISR)	// New series (F7, G4, L4, H7)
#	define UART_DR_REG(h)     ((h)->Instance->RDR)

// In old lines (F1, F3, L1, etc.): DR and SR
#elif defined(USART_DR_DR_Msk) && defined(USART_SR_PE_Msk)
#	define UART_SR_REG(h)     ((h)->Instance->SR)	// Legacy series (F1, F3, L1)
#	define UART_DR_REG(h)     ((h)->Instance->DR)

#else
#	error "[UART ENGINE]: could not determine which RDR/DR or ISR/SR registers are on your MCU"
#endif /* data register depency */


#if !defined(HAL_UART_ERROR_NONE)
#	define  HAL_UART_ERROR_NONE               0    						/*!< No error                */
#endif /* HAL_UART_ERROR_NONE */
#if !defined(HAL_UART_ERROR_PE)
#	define  HAL_UART_ERROR_PE               HAL_UART_ERROR_NONE   	 	/*!< Parity error            */
#endif /* HAL_UART_ERROR_PE */
#if !defined(HAL_UART_ERROR_NE)
#	define  HAL_UART_ERROR_NE               HAL_UART_ERROR_NONE     	/*!< Noise error             */
#endif /* HAL_UART_ERROR_PE */
#if !defined(HAL_UART_ERROR_FE)
#	define  HAL_UART_ERROR_FE               HAL_UART_ERROR_NONE     	/*!< Frame error             */
#endif /* HAL_UART_ERROR_PE */
#if !defined(HAL_UART_ERROR_ORE)
#	define  HAL_UART_ERROR_ORE              HAL_UART_ERROR_NONE     	/*!< Overrun error             */
#endif /* HAL_UART_ERROR_PE */
#if !defined(HAL_UART_ERROR_RTO)
#	define  HAL_UART_ERROR_RTO              HAL_UART_ERROR_NONE      	/*!< Receiver Timeout error  */
#endif /* HAL_UART_ERROR_PE */
#if !defined(HAL_UART_ERROR_DMA)
#	define  HAL_UART_ERROR_DMA               HAL_UART_ERROR_NONE     	/*!< DMA transfer error      */
#endif /* HAL_UART_ERROR_DMA */

#define UART_ENGINE_RX_PART (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)

enum class StatusUART : status_t
{
	/*! Baud rate not supported by system. */
	ERROR_UART_BAUDRATE_NOT_SUPPORTED = -71,

	/*! Receiver buffer hasen't been read before receiving new data.
	 *  Data loss! */
	ERROR_UART_RX = -72,

	/*! Transmitting error stemming from the DMA module. */
	ERROR_UART_DMA_ERR = -73,

	ERROR_UART_STATE = -74,
	ERROR_UART_ORE = -75
};

#endif /* UART_ENGINE_ON */
#endif /* if defined USART or UART */
#endif /* STM32_TOOLS_UART_ENGINE_UARTDEPENCY_H_ */
