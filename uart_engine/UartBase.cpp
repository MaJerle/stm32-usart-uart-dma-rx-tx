/*
 * UartBase.cpp
 *
 *  Created on: Mar 15, 2025
 *      Author: admin
 */

#include "UartBase.h"

#if UART_ENGINE_ON && defined(HAL_UART_MODULE_ENABLED) || defined(HAL_USART_MODULE_ENABLED)
#include "irq/IRQGuard.h"  // Provides IRQ_LOCK() and IRQ_UNLOCK() macros
#include <algorithm>

UartBase::~UartBase()
{
	IRQGuard guard;
	auto it = std::find(m_uarts.begin(), m_uarts.end(), this);
	if (it != m_uarts.end()) {
		m_uarts.erase(it);
	}
	m_huart = nullptr;
}

void UartBase::attachUart(UART_HandleTypeDef *const huart)
{
	IRQGuard guard;
    m_huart = huart;
    if (huart != nullptr && std::find(m_uarts.begin(), m_uarts.end(), this) == m_uarts.end()) {
        m_uarts.push_back(this);
    }
}

#endif /* if defined USART or UART  */
