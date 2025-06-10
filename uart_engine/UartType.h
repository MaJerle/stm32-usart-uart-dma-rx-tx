/*
 * UartType.h
 *
 *  Created on: Feb 15, 2025
 *      Author: admin
 */

#ifndef STM32_TOOLS_UART_ENGINE_UARTTYPE_H_
#define STM32_TOOLS_UART_ENGINE_UARTTYPE_H_

#include "RS485Engine.h"

using UartIT 		= UartEngine<UartType::UART_IT>;
using UartDma 		= UartEngine<UartType::UART_DMA>;
using UartDmaCirc 	= UartEngine<UartType::UART_DMA_CIRCULAR>;

#if UART_ENGINE_RS485_ON
using RS485IT 		= RS485Engine<UartType::UART_IT>;
using RS485Dma 		= RS485Engine<UartType::UART_DMA>;
using RS485DmaCirc 	= RS485Engine<UartType::UART_DMA_CIRCULAR>;
#endif /* UART_ENGINE_RS485_ON */

#endif /* STM32_TOOLS_UART_ENGINE_UARTTYPE_H_ */
