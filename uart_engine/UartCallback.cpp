/*
 * UartCallback.cpp
 *
 *  Created on: May 21, 2025
 *      Author: admin
 */

#include "main.h"  // Must contain UART_HandleTypeDef definition

#if defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED)
#include "UartBase.h"
#include "Uartsettings.h"

#if UART_ENGINE_INTERNAL_CALLBACKS_ON == 1 /* this define add your Uartsettings.h if you want to keep internal Uart-Callbacks */

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	UartBase::rxCpltHandler(huart, Size);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UartBase::txCpltHandler(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	UartBase::errorHandler(huart);
}

#else
#warning "[UART ENGINE]: You must to realize UART callbacks described in this file"
#endif /* UART_ENGINE_INTERNAL_CALLBACKS_ON */

#endif /* defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED) */
