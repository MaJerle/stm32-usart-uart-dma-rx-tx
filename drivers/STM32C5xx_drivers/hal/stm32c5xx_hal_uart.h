/**
  ******************************************************************************
  * @file    stm32c5xx_hal_uart.h
  * @brief   Header file of UART HAL module.
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
#ifndef STM32C5XX_HAL_UART_H
#define STM32C5XX_HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_usart.h"
#include "stm32c5xx_ll_lpuart.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) \
 || defined(UART7) || defined (LPUART1)
/** @defgroup UART UART
  * @{
  */

/** @defgroup UART_Exported_Types HAL UART Types
  * @{
  */
/**
  * @brief HAL UART Instance Definition.
  *
  */
typedef enum
{
  /*! Instance USART1 */
  HAL_UART1 = USART1_BASE,
  /*! Instance USART2 */
  HAL_UART2 = USART2_BASE,
#if defined(USART3)
  /*! Instance USART3 */
  HAL_UART3 = USART3_BASE,
#endif /* USART3 */
  /*! Instance UART4 */
  HAL_UART4 = UART4_BASE,
  /*! Instance UART5 */
  HAL_UART5 = UART5_BASE,
#if defined(USART6)
  /*! Instance USART6 */
  HAL_UART6 = USART6_BASE,
#endif /* USART6 */
#if defined(UART7)
  /*! Instance UART7 */
  HAL_UART7 = UART7_BASE,
#endif /* UART7 */
  /*! Instance LPUART1 */
  HAL_LPUART1 = LPUART1_BASE,
} hal_uart_t;

/**
  * @brief HAL UART State Structure Definition.
  */
typedef enum
{
  /*! Peripheral is not initialized */
  HAL_UART_STATE_RESET                     = 0U,
  /*! Peripheral is initialized but not configured */
  HAL_UART_STATE_INIT                      = (1UL << 31U),
  /*! Peripheral is initialized and a global config is set */
  HAL_UART_STATE_CONFIGURED                = (1UL << 30U),
} hal_uart_state_t;

/**
  * @brief HAL UART Reception State Definition.
  */
typedef enum
{
  /*! Data Reception process is in reset */
  HAL_UART_RX_STATE_RESET                  = (1UL << 31U),
  /*! Data Reception process is in idle */
  HAL_UART_RX_STATE_IDLE                   = (1UL << 30U),
  /*! Data Reception process is ongoing */
  HAL_UART_RX_STATE_ACTIVE                 = (1UL << 29U),
  /*! Data Reception process is paused */
  HAL_UART_RX_STATE_PAUSED                 = (1UL << 28U),
  /*! Data Reception process is aborting */
  HAL_UART_RX_STATE_ABORT                  = (1UL << 27U),
} hal_uart_rx_state_t;

/**
  * @brief HAL UART Transmission State Definition.
  */
typedef enum
{
  /*! Data Transmission process is in reset */
  HAL_UART_TX_STATE_RESET                  = (1UL << 31U),
  /*! Data Transmission process is in idle */
  HAL_UART_TX_STATE_IDLE                   = (1UL << 30U),
  /*! Data Transmission process is ongoing */
  HAL_UART_TX_STATE_ACTIVE                 = (1UL << 29U),
  /*! Data Transmission process is paused */
  HAL_UART_TX_STATE_PAUSED                 = (1UL << 28U),
  /*! Data Transmission process is aborting */
  HAL_UART_TX_STATE_ABORT                  = (1UL << 27U),
} hal_uart_tx_state_t;

/** @addtogroup UART_Basic_config UART Basic configuration Definition
  * @{
  */
/**
  * @brief HAL UART Wordlength Definition.
  */
typedef enum
{
  /*! 7-bit long UART frame */
  HAL_UART_WORD_LENGTH_7_BIT               = LL_USART_DATAWIDTH_7_BIT,
  /*! 8-bit long UART frame */
  HAL_UART_WORD_LENGTH_8_BIT               = LL_USART_DATAWIDTH_8_BIT,
  /*! 9-bit long UART frame */
  HAL_UART_WORD_LENGTH_9_BIT               = LL_USART_DATAWIDTH_9_BIT,
} hal_uart_word_length_t;

/**
  * @brief HAL UART Stop Bits Definition.
  */
typedef enum
{
  /*! UART frame with 0.5 stop bit */
  HAL_UART_STOP_BIT_0_5                    = LL_USART_STOP_BIT_0_5,
  /*! UART frame with 1 stop bit */
  HAL_UART_STOP_BIT_1                      = LL_USART_STOP_BIT_1,
  /*! UART frame with 1.5 stop bits */
  HAL_UART_STOP_BIT_1_5                    = LL_USART_STOP_BIT_1_5,
  /*! UART frame with 2 stop bits */
  HAL_UART_STOP_BIT_2                      = LL_USART_STOP_BIT_2,
} hal_uart_stop_bits_t;

/**
  * @brief HAL UART Parity Definition.
  * @note When parity is enabled, the computed parity is inserted
          at the MSB position of the transmitted data (9th bit when
          the word length is set to 9 data bits; 8th bit when the
          word length is set to 8 data bits).
  */
typedef enum
{
  /*! No parity */
  HAL_UART_PARITY_NONE                     = LL_USART_PARITY_NONE,
  /*! Even parity */
  HAL_UART_PARITY_EVEN                     = LL_USART_PARITY_EVEN,
  /*! Odd parity */
  HAL_UART_PARITY_ODD                      = LL_USART_PARITY_ODD,
} hal_uart_parity_t;

/**
  * @brief HAL UART hardware control definition.
  */
typedef enum
{
  /*! No hardware control */
  HAL_UART_HW_CONTROL_NONE                 = LL_USART_HWCONTROL_NONE,
  /*! Request To Send */
  HAL_UART_HW_CONTROL_RTS                  = LL_USART_HWCONTROL_RTS,
  /*! Clear To Send */
  HAL_UART_HW_CONTROL_CTS                  = LL_USART_HWCONTROL_CTS,
  /*! Request and Clear To Send */
  HAL_UART_HW_CONTROL_RTS_CTS              = LL_USART_HWCONTROL_RTS_CTS,
} hal_uart_hw_control_t;

/**
  * @brief HAL UART direction definition.
  */
typedef enum
{
  /*! RX mode */
  HAL_UART_DIRECTION_RX                    = LL_USART_DIRECTION_RX,
  /*! TX mode */
  HAL_UART_DIRECTION_TX                    = LL_USART_DIRECTION_TX,
  /*! RX and TX mode */
  HAL_UART_DIRECTION_TX_RX                 = LL_USART_DIRECTION_TX_RX,
} hal_uart_direction_t;

/**
  * @brief HAL UART oversampling definition.
  */
typedef enum
{
  /*! Oversampling by 16 */
  HAL_UART_OVERSAMPLING_16                = LL_USART_OVERSAMPLING_16,
  /*! Oversampling by 8. LPUART instances do not support this mode, USARTx instances are configured
      in LIN mode as well. */
  HAL_UART_OVERSAMPLING_8                 = LL_USART_OVERSAMPLING_8,
} hal_uart_oversampling_t;

/**
  * @brief HAL UART one-bit sampling definition.
  * @note Selecting the single sample method increases the receiver tolerance to clock deviations.
  */
typedef enum
{
  /*! One-bit sampling disabled */
  HAL_UART_ONE_BIT_SAMPLE_DISABLE          = LL_USART_ONE_BIT_SAMPLE_DISABLE,
  /*! One-bit sampling enabled */
  HAL_UART_ONE_BIT_SAMPLE_ENABLE           = LL_USART_ONE_BIT_SAMPLE_ENABLE,
} hal_uart_one_bit_sample_t;

/**
  * @brief HAL UART clock prescaler definition.
  */
typedef enum
{
  /*! fclk_pres = fclk */
  HAL_UART_PRESCALER_DIV1                  = LL_USART_PRESCALER_DIV1,
  /*! fclk_pres = fclk/2 */
  HAL_UART_PRESCALER_DIV2                  = LL_USART_PRESCALER_DIV2,
  /*! fclk_pres = fclk/4 */
  HAL_UART_PRESCALER_DIV4                  = LL_USART_PRESCALER_DIV4,
  /*! fclk_pres = fclk/6 */
  HAL_UART_PRESCALER_DIV6                  = LL_USART_PRESCALER_DIV6,
  /*! fclk_pres = fclk/8 */
  HAL_UART_PRESCALER_DIV8                  = LL_USART_PRESCALER_DIV8,
  /*! fclk_pres = fclk/10 */
  HAL_UART_PRESCALER_DIV10                 = LL_USART_PRESCALER_DIV10,
  /*! fclk_pres = fclk/12 */
  HAL_UART_PRESCALER_DIV12                 = LL_USART_PRESCALER_DIV12,
  /*! fclk_pres = fclk/16 */
  HAL_UART_PRESCALER_DIV16                 = LL_USART_PRESCALER_DIV16,
  /*! fclk_pres = fclk/32 */
  HAL_UART_PRESCALER_DIV32                 = LL_USART_PRESCALER_DIV32,
  /*! fclk_pres = fclk/64 */
  HAL_UART_PRESCALER_DIV64                 = LL_USART_PRESCALER_DIV64,
  /*! fclk_pres = fclk/128 */
  HAL_UART_PRESCALER_DIV128                = LL_USART_PRESCALER_DIV128,
  /*! fclk_pres = fclk/256 */
  HAL_UART_PRESCALER_DIV256                = LL_USART_PRESCALER_DIV256,
} hal_uart_prescaler_t;

/**
  * @}
  */
/** @addtogroup UART_AutoBaudRate UART auto baud rate definition
  * @{
  */
/**
  * @brief HAL UART auto baud rate mode definition.
  */
typedef enum
{
  /*! Auto Baud Rate detection on start bit */
  HAL_UART_AUTO_BAUD_DET_ON_START_BIT     = LL_USART_AUTO_BAUD_DETECT_ON_START_BIT,
  /*! Auto Baud Rate detection on falling edge */
  HAL_UART_AUTO_BAUD_DET_ON_FALLING_EDGE  = LL_USART_AUTO_BAUD_DETECT_ON_FALLING_EDGE,
  /*! Auto Baud Rate detection on 0x7F frame detection */
  HAL_UART_AUTO_BAUD_DET_ON_0X7F_FRAME    = LL_USART_AUTO_BAUD_DETECT_ON_0X7F_FRAME,
  /*! Auto Baud Rate detection on 0x55 frame detection */
  HAL_UART_AUTO_BAUD_DET_ON_0X55_FRAME    = LL_USART_AUTO_BAUD_DETECT_ON_0X55_FRAME,
} hal_uart_auto_baud_rate_mode_t;

/**
  * @brief HAL UART auto baud rate detection definition.
  */
typedef enum
{
  /*! Auto baud rate detection not enabled */
  HAL_UART_AUTO_BAUD_RATE_DET_NOT_ENABLED  = 0U,
  /*! Auto baud rate detection started */
  HAL_UART_AUTO_BAUD_RATE_DET_ONGOING      = 1U,
  /*! Auto baud rate detection successful */
  HAL_UART_AUTO_BAUD_RATE_DET_SUCCESS      = 2U,
  /*! Auto baud rate detection error */
  HAL_UART_AUTO_BAUD_RATE_DET_ERROR        = 3U,
} hal_uart_auto_baud_rate_detection_status_t;

/**
  * @brief HAL UART auto baud rate status definition.
  */
typedef enum
{
  /*! UART auto baud rate is disabled */
  HAL_UART_AUTO_BAUD_RATE_DISABLED         = 0U,
  /*! UART auto baud rate is enabled */
  HAL_UART_AUTO_BAUD_RATE_ENABLED          = 1U,
} hal_uart_auto_baud_rate_status_t;
/**
  * @}
  */
/** @addtogroup UART_Modes UART modes definition
  * @{
  */
/**
  * @brief HAL UART multiprocessor mute mode wake-up method definition.
  */
typedef enum
{
  /*! UART wake-up on idle line */
  HAL_UART_WAKEUP_METHOD_IDLE_LINE         = LL_USART_WAKEUP_METHOD_IDLE_LINE,
  /*! UART wake-up on address mark */
  HAL_UART_WAKEUP_METHOD_ADDRESS_MARK      = LL_USART_WAKEUP_METHOD_ADDRESS_MARK,
} hal_uart_wakeup_method_t;

/**
  * @brief HAL UART multiprocessor mute mode status.
  */
typedef enum
{
  /*! UART in active mode */
  HAL_UART_MULTIPROCESSOR_MODE_IN_ACTIVE   = 0U,
  /*! UART in mute mode */
  HAL_UART_MULTIPROCESSOR_MODE_IN_MUTE     = 1U,
} hal_uart_multi_processor_mode_mute_status_t;

/**
  * @brief HAL UART LIN break detect length definition.
  */
typedef enum
{
  /*! LIN 10-bit break detection length */
  HAL_UART_LIN_BREAK_DETECT_10_BIT  = LL_USART_LIN_BREAK_DETECT_10_BIT,
  /*! LIN 11-bit break detection length */
  HAL_UART_LIN_BREAK_DETECT_11_BIT  = LL_USART_LIN_BREAK_DETECT_11_BIT,
} hal_uart_lin_break_detect_length_t;

/**
  * @brief HAL UART driver enable (DE) polarity definition.
  */
typedef enum
{
  /*! Driver Enable(DE) Polarity High */
  HAL_UART_DE_POLARITY_HIGH                = LL_USART_DE_POLARITY_HIGH,
  /*! Driver Enable(DE) Polarity Low */
  HAL_UART_DE_POLARITY_LOW                 = LL_USART_DE_POLARITY_LOW,
} hal_uart_de_polarity_t;

/**
  * @}
  */

/** @addtogroup UART_Stop_Mode UART Stop Mode Definition
  * @{
  */
/**
  * @brief HAL UART stop mode status definition.
  */
typedef enum
{
  /*! UART not functional in low-power mode */
  HAL_UART_STOP_MODE_DISABLED              = 0U,
  /*! UART functional in low-power mode */
  HAL_UART_STOP_MODE_ENABLED               = 1U,
} hal_uart_stop_mode_status_t;

/**
  * @brief HAL UART stop mode wake-up source definition.
  */
typedef enum
{
  /*! UART wake-up on address */
  HAL_UART_WAKEUP_ON_ADDRESS               = LL_USART_WAKEUP_ON_ADDRESS,
  /*! UART wake-up on start bit */
  HAL_UART_WAKEUP_ON_STARTBIT              = LL_USART_WAKEUP_ON_STARTBIT,
  /*! UART wake-up on receive data register not empty or Rx FIFO is not empty */
  HAL_UART_WAKEUP_ON_READDATA_NONEMPTY     = LL_USART_WAKEUP_ON_RXNE
} hal_uart_wakeup_source_t;

/**
  * @brief HAL UART address detection length definition.
  */
typedef enum
{
  /*! 4-bit long wake-up address */
  HAL_UART_ADDRESS_DETECT_4_BIT            = LL_USART_ADDRESS_DETECT_4_BIT,
  /*! 7-bit long wake-up address */
  HAL_UART_ADDRESS_DETECT_7_BIT            = LL_USART_ADDRESS_DETECT_7_BIT,
} hal_uart_address_detect_length_t;

/**
  * @}
  */

/**
  * @brief HAL UART reception mode definition.
  */
typedef enum
{
  /*! Standard reception */
  HAL_UART_RX_STANDARD              = 0U,
  /*! Reception till completion or IDLE event */
  HAL_UART_RX_TO_IDLE               = 1U,
  /*! Reception till completion or Receive TimeOut(RTO) event */
  HAL_UART_RX_TO_RTO                = 2U,
  /*! Reception till completion or character match (CM) event */
  HAL_UART_RX_TO_CHAR_MATCH         = 3U,
} hal_uart_rx_modes_t;

/**
  * @brief HAL UART reception event types definition.
  */
typedef enum
{
  /*! RxEvent linked to Transfer Complete event */
  HAL_UART_RX_EVENT_TC                     = 0U,
  /*! RxEvent linked to IDLE event */
  HAL_UART_RX_EVENT_IDLE                   = 1U,
  /*! RxEvent linked to TimeOut event */
  HAL_UART_RX_EVENT_RTO                = 2U,
  /*! RxEvent linked to Character Match event */
  HAL_UART_RX_EVENT_CHAR_MATCH              = 3U,
} hal_uart_rx_event_types_t;

/** @defgroup UART_FIFO_Mode UART FIFO Mode Definition
  * @{
  */

/**
  * @brief HAL UART FIFO threshold definition.
  */
typedef enum
{
  /*! FIFO reaches 1/8 of its depth */
  HAL_UART_FIFO_THRESHOLD_1_8              = LL_USART_FIFO_THRESHOLD_1_8,
  /*! FIFO reaches 1/4 of its depth */
  HAL_UART_FIFO_THRESHOLD_1_4              = LL_USART_FIFO_THRESHOLD_1_4,
  /*! FIFO reaches 1/2 of its depth */
  HAL_UART_FIFO_THRESHOLD_1_2              = LL_USART_FIFO_THRESHOLD_1_2,
  /*! FIFO reaches 3/4 of its depth */
  HAL_UART_FIFO_THRESHOLD_3_4              = LL_USART_FIFO_THRESHOLD_3_4,
  /*! FIFO reaches 7/8 of its depth */
  HAL_UART_FIFO_THRESHOLD_7_8              = LL_USART_FIFO_THRESHOLD_7_8,
  /*! FIFO reaches 8/8 of its depth */
  HAL_UART_FIFO_THRESHOLD_8_8              = LL_USART_FIFO_THRESHOLD_8_8,
} hal_uart_fifo_threshold_t;

/**
  * @brief HAL UART FIFO mode status definition.
  */
typedef enum
{
  /*! UART FIFO mode is disabled */
  HAL_UART_FIFO_MODE_DISABLED              = 0U,
  /*! UART FIFO mode is enabled */
  HAL_UART_FIFO_MODE_ENABLED               = 1U,
} hal_uart_fifo_mode_status_t;
/**
  * @}
  */

/** @addtogroup UART_Advanced_config UART Advanced Configuration Definition
  * @{
  */
/**
  * @brief HAL UART TX Pin Level Invert Status Definition.
  */
typedef enum
{
  /*! UART Tx Pin Level Inversion is disabled */
  HAL_UART_TX_PIN_LEVEL_INVERT_DISABLED    = 0U,
  /*! UART Tx Pin Level Inversion is enabled */
  HAL_UART_TX_PIN_LEVEL_INVERT_ENABLED     = 1U,
} hal_uart_tx_pin_level_invert_status_t;

/**
  * @brief HAL UART Rx pin level invert status definition.
  */
typedef enum
{
  /*! UART Rx Pin Level Inversion is disabled */
  HAL_UART_RX_PIN_LEVEL_INVERT_DISABLED    = 0U,
  /*! UART Rx Pin Level Inversion is enabled */
  HAL_UART_RX_PIN_LEVEL_INVERT_ENABLED     = 1U,
} hal_uart_rx_pin_level_invert_status_t;

/**
  * @brief HAL UART data invert status definition.
  */
typedef enum
{
  /*! UART Data Binary Inversion is disabled */
  HAL_UART_DATA_INVERT_DISABLED            = 0U,
  /*! UART Data Binary Inversion is enabled */
  HAL_UART_DATA_INVERT_ENABLED             = 1U,
} hal_uart_data_invert_status_t;

/**
  * @brief HAL UART swap Tx/Rx status definition.
  */
typedef enum
{
  /*! UART Tx Rx Swap Pins is disabled */
  HAL_UART_TX_RX_SWAP_DISABLED             = 0U,
  /*! UART Tx Rx Swap Pins is enabled */
  HAL_UART_TX_RX_SWAP_ENABLED              = 1U,
} hal_uart_tx_rx_swap_status_t;

/**
  * @brief HAL UART overrun status definition.
  */
typedef enum
{
  /*! UART Rx Overrun Detection is disabled */
  HAL_UART_RX_OVERRUN_DETECTION_DISABLED   = 0U,
  /*! UART Rx Overrun Detection is enabled */
  HAL_UART_RX_OVERRUN_DETECTION_ENABLED    = 1U,
} hal_uart_rx_overrun_detection_status_t;

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
/**
  * @brief HAL UART DMA disable on Rx error status definition.
  */
typedef enum
{
  /*! UART DMA Stop On Rx Error is disabled */
  HAL_UART_DMA_STOP_ON_RX_ERROR_DISABLED   = 0U,
  /*! UART DMA Stop On Rx Error is enabled */
  HAL_UART_DMA_STOP_ON_RX_ERROR_ENABLED    = 1U,
} hal_uart_dma_stop_on_rx_error_status_t;
#endif /* USE_HAL_UART_DMA */

/**
  * @brief HAL UART Most Significant Bit First Status Definition.
  */
typedef enum
{
  /*! UART Most Significant Bit First is disabled */
  HAL_UART_MSB_FIRST_DISABLED              = 0U,
  /*! UART Most Significant Bit First is enabled */
  HAL_UART_MSB_FIRST_ENABLED               = 1U,
} hal_uart_msb_first_status_t;

/**
  * @brief HAL UART receiver timeout status definition.
  */
typedef enum
{
  /*! UART Receiver TimeOut is disabled */
  HAL_UART_RECEIVER_TIMEOUT_DISABLED       = 0U,
  /*! UART Receiver TimeOut is enabled */
  HAL_UART_RECEIVER_TIMEOUT_ENABLED        = 1U,
} hal_uart_receiver_timeout_status_t;

/**
  * @brief HAL UART transmitter status definition.
  */
typedef enum
{
  /*! UART Transmitter is disabled */
  HAL_UART_TRANSMITTER_DISABLED            = 0U,
  /*! UART transmitter is enabled */
  HAL_UART_TRANSMITTER_ENABLED             = 1U,
} hal_uart_transmitter_status_t;

/**
  * @brief HAL UART receiver status definition.
  */
typedef enum
{
  /*! UART receiver is disabled */
  HAL_UART_RECEIVER_DISABLED               = 0U,
  /*! UART receiver is enabled */
  HAL_UART_RECEIVER_ENABLED                = 1U,
} hal_uart_receiver_status_t;

/**
  * @}
  */

/** @addtogroup UART_Modes UART Modes Definition
  * @{
  */
/**
  * @brief HAL UART LIN mode status definition.
  */
typedef enum
{
  /*! UART LIN mode is disabled */
  HAL_UART_LIN_MODE_DISABLED               = 0U,
  /*! UART LIN mode is enabled */
  HAL_UART_LIN_MODE_ENABLED                = 1U,
} hal_uart_lin_mode_status_t;

/**
  * @brief HAL UART half-duplex mode status definition.
  */
typedef enum
{
  /*! UART Half Duplex Mode is disabled */
  HAL_UART_HALF_DUPLEX_MODE_DISABLED       = 0U,
  /*! UART Half Duplex Mode is enabled */
  HAL_UART_HALF_DUPLEX_MODE_ENABLED        = 1U,
} hal_uart_half_duplex_mode_status_t;

/**
  * @brief HAL UART RS485 Mode Status Definition.
  */
typedef enum
{
  /*! UART RS485 mode is disabled */
  HAL_UART_RS485_MODE_DISABLED             = 0U,
  /*! UART RS485 mode is enabled */
  HAL_UART_RS485_MODE_ENABLED              = 1U,
} hal_uart_rs485_mode_status_t;

/**
  * @brief HAL UART multiprocessor mode status definition.
  */
typedef enum
{
  /*! UART multiprocessor mode is disabled */
  HAL_UART_MULTI_PROCESSOR_MODE_DISABLED   = 0U,
  /*! UART multiprocessor mode is enabled */
  HAL_UART_MULTI_PROCESSOR_MODE_ENABLED    = 1U,
} hal_uart_multi_processor_mode_status_t;
/**
  * @}
  */

/** @addtogroup UART_Advanced_IO UART Advanced I/O operation Definition
  * @{
  */
/**
  * @brief HAL UART request definition.
  */
typedef enum
{
  /*! Auto-baud rate request. LPUART instances do not support this request.*/
  HAL_UART_REQUEST_AUTO_BAUD_RATE          = LL_USART_REQUEST_AUTO_BAUD_RATE,
  /*! Send Break Request */
  HAL_UART_REQUEST_SEND_BREAK              = LL_USART_REQUEST_SEND_BREAK,
  /*! Mute Mode Request */
  HAL_UART_REQUEST_MUTE_MODE               = LL_USART_REQUEST_MUTE_MODE,
  /*! Receive data flush request */
  HAL_UART_REQUEST_RX_DATA_FLUSH           = LL_USART_REQUEST_RX_DATA_FLUSH,
  /*! Transmit data flush request */
  HAL_UART_REQUEST_TX_DATA_FLUSH           = LL_USART_REQUEST_TX_DATA_FLUSH,
} hal_uart_request_t;

/**
  * @}
  */
/** @defgroup UART_IRDA_PowerMode IRDA power mode Definition
  * @{
  */
/**
  * @brief HAL UART IRDA power mode definition.
  */
typedef enum
{
  /*! IRDA mode normal */
  HAL_UART_IRDA_POWER_MODE_NORMAL          = LL_USART_IRDA_POWER_MODE_NORMAL,
  /*! IRDA mode low power */
  HAL_UART_IRDA_POWER_MODE_LOW             = LL_USART_IRDA_POWER_MODE_LOW,
} hal_uart_irda_power_mode_t;
/**
  * @}
  */

/*! HAL UART handler type */
typedef struct hal_uart_handle_s hal_uart_handle_t;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
/*! HAL UART Generic UART callback Type */
typedef void (* hal_uart_cb_t)(hal_uart_handle_t *huart);

/*! HAL UART Reception Complete Callback Pointer Type */
typedef void (* hal_uart_rx_cplt_cb_t)(hal_uart_handle_t *huart, uint32_t size_byte,
                                       hal_uart_rx_event_types_t rx_event);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

/**
  * @brief HAL UART handle structure type.
  */
struct hal_uart_handle_s
{
  /*! Peripheral instance        */
  hal_uart_t instance;

  /*! Pointer to USART Tx transfer buffer */
  const uint8_t *p_tx_buff;

  /*! USART Tx transfer size              */
  volatile uint32_t tx_xfer_size;

  /*! USART Tx transfer counter           */
  volatile uint32_t tx_xfer_count;

  /*! Pointer to USART Rx transfer buffer */
  uint8_t *p_rx_buff;

  /*! USART Rx transfer size              */
  volatile uint32_t rx_xfer_size;

  /*! USART Rx transfer counter           */
  volatile uint32_t rx_xfer_count;

  /*! USART Rx RDR register mask          */
  volatile uint16_t rdr_mask;

  /*! Specifies whether the FIFO mode is being used. */
  hal_uart_fifo_mode_status_t fifo_mode;

  /*! Number of data to process during RX ISR execution */
  uint16_t nb_rx_data_to_process;

  /*! Number of data to process during TX ISR execution */
  uint16_t nb_tx_data_to_process;

  /*! Type of ongoing reception          */
  volatile hal_uart_rx_modes_t reception_type;

  /*! Function pointer on Rx IRQ handler */
  void (*p_rx_isr)(struct hal_uart_handle_s *huart);

  /*! Function pointer on Tx IRQ handler */
  void (*p_tx_isr)(struct hal_uart_handle_s *huart);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
  /*! USART Tx DMA handle parameters      */
  hal_dma_handle_t *hdma_tx;

  /*! USART Rx DMA handle parameters      */
  hal_dma_handle_t *hdma_rx;

#endif /* USE_HAL_UART_DMA */
  /*! USART state information related to global handle management */
  volatile hal_uart_state_t global_state;

  /*! USART state information related to Rx operations. */
  volatile hal_uart_rx_state_t    rx_state;

  /*! USART state information related to Tx operations. */
  volatile hal_uart_tx_state_t    tx_state;

#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  /*! USART Tx half complete callback        */
  hal_uart_cb_t p_tx_half_cplt_callback;

  /*! USART Tx complete callback             */
  hal_uart_cb_t p_tx_cplt_callback;

  /*! USART Rx half complete callback        */
  hal_uart_cb_t p_rx_half_cplt_callback;

  /*! USART Rx complete callback             */
  hal_uart_rx_cplt_cb_t p_rx_cplt_callback;

  /*! USART error callback                   */
  hal_uart_cb_t p_error_callback;

  /*! USART abort complete callback          */
  hal_uart_cb_t p_abort_cplt_callback;

  /*! USART abort transmit complete callback */
  hal_uart_cb_t p_abort_transmit_cplt_callback;

  /*! USART abort receive complete callback  */
  hal_uart_cb_t p_abort_receive_cplt_callback;

  /*! USART wake-up callback                  */
  hal_uart_cb_t p_wakeup_callback;

  /*! USART Rx FIFO full callback            */
  hal_uart_cb_t p_rx_fifo_full_callback;

  /*! USART Tx FIFO empty callback           */
  hal_uart_cb_t p_tx_fifo_empty_callback;

  /*! USART clear-to-send callback         */
  hal_uart_cb_t p_clear_to_send_callback;

  /*! USART LIN break callback         */
  hal_uart_cb_t p_lin_break_callback;

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /*! USART OS semaphore */
  hal_os_semaphore_t semaphore;

#endif /* USE_HAL_MUTEX */
#if defined (USE_HAL_UART_USER_DATA) && (USE_HAL_UART_USER_DATA == 1)
  /*! User Data Pointer */
  const void *p_user_data;

#endif /* USE_HAL_UART_USER_DATA */
#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
  /*! Last error codes on reception side */
  volatile uint32_t last_reception_error_codes;

  /*! Last error codes on transmission side */
  volatile uint32_t last_transmission_error_codes;
#endif /* USE_HAL_UART_GET_LAST_ERRORS */
};


/** @addtogroup UART_Modes UART Modes Definition
  * @{
  */
/**
  * @brief HAL UART multiprocessor communication wake-up from mute mode configuration type.
  */
typedef struct
{
  /*! UART wakeup method (Idle Line/Address). */
  hal_uart_wakeup_method_t wakeup_method;
  /*! Specifies whether the address is 4 or 7-bit long. */
  hal_uart_address_detect_length_t address_length;
  /*! UART node address (7-bit long max). */
  uint8_t address;
} hal_uart_multi_processor_mode_wakeup_config_t;

/**
  * @brief HAL UART RS485 config type.
  */
typedef struct
{
  /*! UART assertion time, value between 0x00 -> 0x1FU in sample time unit (1/8 or 1/16 bit time,
      depending on the oversampling). */
  uint32_t assertion_time_samples;
  /*! UART deassertion time, value between 0x00 -> 0x1FU in sample time unit (1/8 or 1/16 bit time,
      depending on the oversampling). */
  uint32_t deassertion_time_samples;
  /*! UART driver enable (DE) polarity. */
  hal_uart_de_polarity_t polarity;
} hal_uart_rs485_config_t;
/**
  * @}
  */

/** @addtogroup UART_Basic_config
  * @{
  */
/**
  * @brief HAL UART global config structure type.
  */
typedef struct
{
  /*! This member configures the UART communication baud rate.
    Value between Min_Data=0 and Max_Data=18000000.
    For LPUART instances, Min_Data=0 and Max_Data=48000000.
    The baud rate register is computed using the following formula:
        - LPUART:
          - Baud Rate Register = ((256 * lpuart_ker_ckpres) / ((hal_uart_config_t->baud_rate)))
            where lpuart_ker_ck_pres is the UART input clock divided by a prescaler
        - UART:
          - If oversampling is 16 or in LIN mode,
            Baud Rate Register = ((uart_ker_ckpres) / ((hal_uart_config_t->baud_rate)))
          - If oversampling is 8,
            Baud Rate Register[15:4] = ((2 * uart_ker_ckpres) /
            ((hal_uart_config_t->baud_rate)))[15:4]
            Baud Rate Register[3] = 0
            Baud Rate Register[2:0] = (((2 * uart_ker_ckpres) /
            ((hal_uart_config_t->baud_rate)))[3:0]) >> 1
          where uart_ker_ck_pres is the UART input clock divided by a prescaler */
  uint32_t baud_rate;

  /*! Specifies the prescaler value used to divide the UART clock source. */
  hal_uart_prescaler_t clock_prescaler;

  /*! Specifies the number of data bits transmitted or received in a frame. */
  hal_uart_word_length_t word_length;

  /*! Specifies the number of stop bits transmitted. */
  hal_uart_stop_bits_t stop_bits;

  /*! Specifies the parity mode. */
  hal_uart_parity_t parity;

  /*! Specifies whether the receive or transmit mode is enabled or disabled. */
  hal_uart_direction_t direction;

  /*! Specifies whether the hardware flow control mode is enabled or disabled. */
  hal_uart_hw_control_t hw_flow_ctl;

  /*! Specifies whether oversampling by 8 is enabled or disabled. */
  hal_uart_oversampling_t oversampling;

  /*! Specifies whether single-sample or three-sample majority vote is selected. This parameter
   is not available for LPUART instances. */
  hal_uart_one_bit_sample_t one_bit_sampling;
} hal_uart_config_t;
/**
  * @}
  */

/** @addtogroup UART_IRDA_config
  * @{
  */
/**
  * @brief HAL UART IRDA config structure type.
  */
typedef struct
{
  /*! This member configures the IRDA communication baud rate (value to be set at 115200 baud following IRDA
    specifications). You can, however, still set the value between Min_Data=0 and Max_Data=18000000 for
    specific use cases. The baud rate register is computed using the following formula:
      - Baud Rate Register = ((uart_ker_ckpres) / ((hal_uart_irda_config_t->baud_rate)))
        where uart_ker_ck_pres is the UART input clock divided by a prescaler */
  uint32_t baud_rate;

  /*! Specifies the prescaler value used to divide the IRDA clock source. */
  hal_uart_prescaler_t clock_prescaler;

  /*! Specifies the number of data bits transmitted or received in a frame. */
  hal_uart_word_length_t word_length;

  /*! Specifies the IRDA mode to be used. */
  hal_uart_irda_power_mode_t irda_power_mode;

  /*! Specifies whether the receive or transmit mode is enabled or disabled. */
  hal_uart_direction_t direction;

  /*! Specifies the prescaler value for dividing the UART/USART source clock to achieve low-power frequency.
      Value must be between 0x01 and 0xFF. */
  uint32_t irda_prescaler;

  /*! Specifies the parity mode. */
  hal_uart_parity_t parity;

  /*! Specifies whether single-sample or three-sample majority vote is selected. */
  hal_uart_one_bit_sample_t one_bit_sampling;
} hal_uart_irda_config_t;
/**
  * @}
  */

/**
  * @}
  */

/* Exported Constants --------------------------------------------------------*/
/** @defgroup UART_Exported_Constants HAL UART Constants
  * @{
  */

/** @defgroup UART_Receive_Error_Codes UART Receive Error Codes
  * @{
  */
/*! No error on RX */
#define HAL_UART_RECEIVE_ERROR_NONE     (0UL)

/*! Parity error on RX */
#define HAL_UART_RECEIVE_ERROR_PE       (0x1UL << 0)

/*! Noise error on RX */
#define HAL_UART_RECEIVE_ERROR_NE       (0x1UL << 1U)

/*! Frame error on RX */
#define HAL_UART_RECEIVE_ERROR_FE       (0x1UL << 2U)

/*! Overrun error on RX */
#define HAL_UART_RECEIVE_ERROR_ORE      (0x1UL << 3U)

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
/*! DMA transfer error on RX */
#define HAL_UART_RECEIVE_ERROR_DMA      (0x1UL << 4U)
#endif /* USE_HAL_UART_DMA */

/*! Receiver Timeout error on RX */
#define HAL_UART_RECEIVE_ERROR_RTO      (0x1UL << 5U)
/**
  * @}
  */

/** @defgroup UART_Transmit_Error_Codes UART Transmit Error Codes
  * @{
  */
/*! No error on TX */
#define HAL_UART_TRANSMIT_ERROR_NONE    (0UL << 16U)

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
/*! DMA transfer error on TX */
#define HAL_UART_TRANSMIT_ERROR_DMA     (0x1UL << 17U)
#endif /* USE_HAL_UART_DMA */
/**
  * @}
  */

/** @defgroup UART_Optional_Interrupts UART Optional Interrupts
  * @{
  */

/** @defgroup UART_Transmit_IT_Optional_Interrupts UART Optional Interrupts for Transmit interrupt process
  * @{
  */
/*! Do not activate optional interruptions on TX IT process */
#define HAL_UART_OPT_TX_IT_NONE           0U
/*! Activate FIFO Empty optional interruption */
#define HAL_UART_OPT_TX_IT_FIFO_EMPTY     (1UL << 30U)
/*! Activate clear-to-send optional interruption */
#define HAL_UART_OPT_TX_IT_CLEAR_TO_SEND  (1UL << 29U)
/*! Activate FIFO empty and clear-to-send optional interruptions */
#define HAL_UART_OPT_TX_IT_DEFAULT        (HAL_UART_OPT_TX_IT_FIFO_EMPTY | HAL_UART_OPT_TX_IT_CLEAR_TO_SEND)
/**
  * @}
  */

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
/** @defgroup UART_Transmit_DMA_Optional_Interrupts UART Optional Interrupts for Transmit DMA process
  * @{
  */
/*! Do not activate optional interruptions on TX DMA process */
#define HAL_UART_OPT_DMA_TX_IT_NONE       0U
/*! Activate DMA Half Transfer optional interruption */
#define HAL_UART_OPT_DMA_TX_IT_HT         (HAL_DMA_OPT_IT_HT)
/*! Activate DMA Half Transfer optional interruption */
#define HAL_UART_OPT_DMA_TX_IT_DEFAULT    (HAL_UART_OPT_DMA_TX_IT_HT)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Activate Silent Mode on DMA */
#define HAL_UART_OPT_DMA_TX_IT_SILENT     (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_UART_DMA */

/** @defgroup UART_Receive_IT_Optional_Interrupts UART Optional Interrupts for Receive interrupt process
  * @{
  */
/*! Do not activate optional interruptions on RX IT process */
#define HAL_UART_OPT_RX_IT_NONE           0U
/*! Activate FIFO Full optional interruption */
#define HAL_UART_OPT_RX_IT_FIFO_FULL      (1UL << 25U)
/*! Activate LIN Break optional interruption */
#define HAL_UART_OPT_RX_IT_LIN_BREAK      (1UL << 24U)
/*! Activate FIFO Full optional and LIN Break interruptions */
#define HAL_UART_OPT_RX_IT_DEFAULT        (HAL_UART_OPT_RX_IT_FIFO_FULL | HAL_UART_OPT_RX_IT_LIN_BREAK)
/**
  * @}
  */

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
/** @defgroup UART_Receive_DMA_Optional_Interrupts UART Optional Interrupts for Receive DMA process
  * @{
  */
/*! Do not activate optional interruptions on RX DMA process */
#define HAL_UART_OPT_DMA_RX_IT_NONE       0U
/*! Activate DMA Half Transfer optional interruption */
#define HAL_UART_OPT_DMA_RX_IT_HT         HAL_DMA_OPT_IT_HT
/*! Activate DMA Half Transfer optional interruption */
#define HAL_UART_OPT_DMA_RX_IT_DEFAULT    (HAL_UART_OPT_DMA_RX_IT_HT)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Activate Silent Mode on DMA */
#define HAL_UART_OPT_DMA_RX_IT_SILENT     (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_UART_DMA */

/**
  * @}
  */
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @defgroup UART_Exported_Functions HAL UART Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
hal_status_t HAL_UART_Init(hal_uart_handle_t *huart, hal_uart_t instance);
void HAL_UART_DeInit(hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group2 Basic configuration functions
  * @{
  */
hal_status_t HAL_UART_SetConfig(hal_uart_handle_t *huart, const hal_uart_config_t *p_config);
void HAL_UART_GetConfig(const hal_uart_handle_t *huart, hal_uart_config_t *p_config);

hal_status_t HAL_UART_SetWordLength(const hal_uart_handle_t *huart, hal_uart_word_length_t word_length);
hal_uart_word_length_t HAL_UART_GetWordLength(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetParity(const hal_uart_handle_t *huart, hal_uart_parity_t parity);
hal_uart_parity_t HAL_UART_GetParity(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetStopBits(const hal_uart_handle_t *huart, hal_uart_stop_bits_t stop_bits);
hal_uart_stop_bits_t HAL_UART_GetStopBits(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetXferDirection(const hal_uart_handle_t *huart, hal_uart_direction_t direction);
hal_uart_direction_t HAL_UART_GetXferDirection(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetHwFlowCtl(const hal_uart_handle_t *huart, hal_uart_hw_control_t hw_flow_ctl);
hal_uart_hw_control_t HAL_UART_GetHwFlowCtl(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetOneBitSample(const hal_uart_handle_t *huart, hal_uart_one_bit_sample_t one_bit_sample);
hal_uart_one_bit_sample_t HAL_UART_GetOneBitSample(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetBaudRate(const hal_uart_handle_t *huart, uint32_t baud_rate);
uint32_t HAL_UART_GetBaudRate(const hal_uart_handle_t *huart);

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group3 IRDA configuration functions
  * @{
  */
hal_status_t HAL_UART_IRDA_SetConfig(hal_uart_handle_t *huart, const hal_uart_irda_config_t *p_config);
void HAL_UART_IRDA_GetConfig(const hal_uart_handle_t *huart, hal_uart_irda_config_t *p_config);

hal_status_t HAL_UART_IRDA_SetPrescaler(const hal_uart_handle_t *huart, uint32_t irda_prescaler);
uint32_t HAL_UART_IRDA_GetPrescaler(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_IRDA_SetPowerMode(const hal_uart_handle_t *huart,
                                        hal_uart_irda_power_mode_t irda_power_mode);
hal_uart_irda_power_mode_t HAL_UART_IRDA_GetPowerMode(const hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group4 Mode configuration functions
  * @{
  */
hal_status_t HAL_UART_EnableLINMode(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableLINMode(const hal_uart_handle_t *huart);
hal_uart_lin_mode_status_t HAL_UART_IsEnabledLINMode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetLINModeBreakDetectLength(const hal_uart_handle_t *huart,
                                                  hal_uart_lin_break_detect_length_t break_detect_length);
hal_uart_lin_break_detect_length_t HAL_UART_GetLINModeBreakDetectLength(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableRS485Mode(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableRS485Mode(const hal_uart_handle_t *huart);
hal_uart_rs485_mode_status_t HAL_UART_IsEnabledRS485Mode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetConfigRS485Mode(const hal_uart_handle_t *huart, const hal_uart_rs485_config_t *p_config);
void HAL_UART_GetConfigRS485Mode(const hal_uart_handle_t *huart, hal_uart_rs485_config_t *p_config);

hal_status_t HAL_UART_EnableHalfDuplexMode(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableHalfDuplexMode(const hal_uart_handle_t *huart);
hal_uart_half_duplex_mode_status_t HAL_UART_IsEnabledHalfDuplexMode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableMultiProcessorMode(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableMultiProcessorMode(const hal_uart_handle_t *huart);
hal_uart_multi_processor_mode_status_t HAL_UART_IsEnabledMultiProcessorMode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetConfigMultiProcessorMode(const hal_uart_handle_t *huart,
                                                  const hal_uart_multi_processor_mode_wakeup_config_t *p_wakeup_config);
void HAL_UART_GetConfigMultiProcessorMode(const hal_uart_handle_t *huart,
                                          hal_uart_multi_processor_mode_wakeup_config_t *p_wakeup_config);
hal_status_t HAL_UART_EnterMultiProcessorMuteMode(const hal_uart_handle_t *huart);
hal_uart_multi_processor_mode_mute_status_t HAL_UART_IsEnteredMultiProcessorMuteMode(const hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group5 Advanced configuration functions
  * @{
  */

hal_status_t HAL_UART_EnableTxPinLevelInvert(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableTxPinLevelInvert(const hal_uart_handle_t *huart);
hal_uart_tx_pin_level_invert_status_t HAL_UART_IsEnabledTxPinLevelInvert(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableRxPinLevelInvert(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableRxPinLevelInvert(const hal_uart_handle_t *huart);
hal_uart_rx_pin_level_invert_status_t HAL_UART_IsEnabledRxPinLevelInvert(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableDataInvert(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableDataInvert(const hal_uart_handle_t *huart);
hal_uart_data_invert_status_t HAL_UART_IsEnabledDataInvert(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableTxRxSwap(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableTxRxSwap(const hal_uart_handle_t *huart);
hal_uart_tx_rx_swap_status_t HAL_UART_IsEnabledTxRxSwap(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableRxOverRunDetection(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableRxOverRunDetection(const hal_uart_handle_t *huart);
hal_uart_rx_overrun_detection_status_t HAL_UART_IsEnabledRxOverRunDetection(const hal_uart_handle_t *huart);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1U)
hal_status_t HAL_UART_EnableDMAStopOnRxError(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableDMAStopOnRxError(const hal_uart_handle_t *huart);
hal_uart_dma_stop_on_rx_error_status_t HAL_UART_IsEnabledDMAStopOnRxError(const hal_uart_handle_t *huart);
#endif /* USE_HAL_UART_DMA */

hal_status_t HAL_UART_EnableMSBFirst(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableMSBFirst(const hal_uart_handle_t *huart);
hal_uart_msb_first_status_t HAL_UART_IsEnabledMSBFirst(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetConfigReceiverTimeout(const hal_uart_handle_t *huart, uint32_t timeout_bit);
uint32_t HAL_UART_GetConfigReceiverTimeout(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_EnableReceiverTimeout(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableReceiverTimeout(const hal_uart_handle_t *huart);
hal_uart_receiver_timeout_status_t HAL_UART_IsEnabledReceiverTimeout(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableTransmitter(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableTransmitter(const hal_uart_handle_t *huart);
hal_uart_transmitter_status_t HAL_UART_IsEnabledTransmitter(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_EnableReceiver(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableReceiver(const hal_uart_handle_t *huart);
hal_uart_receiver_status_t HAL_UART_IsEnabledReceiver(const hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group6 Auto Baud Rate Configuration functions
  * @{
  */
hal_status_t HAL_UART_EnableAutoBaudRate(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableAutoBaudRate(const hal_uart_handle_t *huart);
hal_uart_auto_baud_rate_status_t HAL_UART_IsEnabledAutoBaudRate(const hal_uart_handle_t *huart);
hal_uart_auto_baud_rate_detection_status_t HAL_UART_GetAutoBaudRateStatus(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetConfigAutoBaudRateMode(const hal_uart_handle_t *huart,
                                                hal_uart_auto_baud_rate_mode_t auto_baud_rate_mode);
hal_uart_auto_baud_rate_mode_t HAL_UART_GetConfigAutoBaudRateMode(const hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group7 Stop Mode Configuration functions
  * @{
  */
hal_status_t HAL_UART_EnableStopMode(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableStopMode(const hal_uart_handle_t *huart);
hal_uart_stop_mode_status_t HAL_UART_IsEnabledStopMode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetStopModeWkUpSource(const hal_uart_handle_t *huart, const hal_uart_wakeup_source_t source);
hal_uart_wakeup_source_t HAL_UART_GetStopModeWkUpSource(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetStopModeWkUpAddrLength(const hal_uart_handle_t *huart,
                                                const hal_uart_address_detect_length_t address_length);
hal_uart_address_detect_length_t HAL_UART_GetStopModeWkUpAddrLength(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetStopModeWkUpAddr(const hal_uart_handle_t *huart, uint8_t address);
uint8_t HAL_UART_GetStopModeWkUpAddr(const hal_uart_handle_t *huart);

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group8 FIFO Configuration functions
  * @{
  */
hal_status_t HAL_UART_EnableFifoMode(hal_uart_handle_t *huart);
hal_status_t HAL_UART_DisableFifoMode(hal_uart_handle_t *huart);
hal_uart_fifo_mode_status_t HAL_UART_IsEnabledFifoMode(const hal_uart_handle_t *huart);

hal_status_t HAL_UART_SetTxFifoThreshold(hal_uart_handle_t *huart, hal_uart_fifo_threshold_t tx_fifo_threshold);
hal_uart_fifo_threshold_t HAL_UART_GetTxFifoThreshold(const hal_uart_handle_t *huart);
hal_status_t HAL_UART_SetRxFifoThreshold(hal_uart_handle_t *huart, hal_uart_fifo_threshold_t rx_fifo_threshold);
hal_uart_fifo_threshold_t HAL_UART_GetRxFifoThreshold(const hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group10 DMA Configuration functions
  * @{
  */
#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
hal_status_t HAL_UART_SetTxDMA(hal_uart_handle_t *huart, hal_dma_handle_t *hdma_tx);
hal_status_t HAL_UART_SetRxDMA(hal_uart_handle_t *huart, hal_dma_handle_t *hdma_rx);
#endif /* USE_HAL_UART_DMA */

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group11 Callbacks Register functions
  * @{
  */
#if defined(USE_HAL_UART_REGISTER_CALLBACKS) && (USE_HAL_UART_REGISTER_CALLBACKS == 1)

hal_status_t HAL_UART_RegisterTxHalfCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterTxCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterRxHalfCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterRxCpltCallback(hal_uart_handle_t *huart, hal_uart_rx_cplt_cb_t p_callback);
hal_status_t HAL_UART_RegisterErrorCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterAbortCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterAbortTransmitCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterAbortReceiveCpltCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);

hal_status_t HAL_UART_RegisterWakeupCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);

hal_status_t HAL_UART_RegisterRxFifoFullCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);
hal_status_t HAL_UART_RegisterTxFifoEmptyCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);

hal_status_t HAL_UART_RegisterClearToSendCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);

hal_status_t HAL_UART_RegisterLINBreakCallback(hal_uart_handle_t *huart, hal_uart_cb_t p_callback);

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group12 I/O operation functions
  * @{
  */

hal_status_t HAL_UART_Transmit(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_UART_Receive(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_UART_Transmit_IT(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte);
hal_status_t HAL_UART_Receive_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte);
hal_status_t HAL_UART_Transmit_IT_Opt(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte,
                                      uint32_t interrupts);
hal_status_t HAL_UART_Receive_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t interrupts);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
hal_status_t HAL_UART_Transmit_DMA(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte);
hal_status_t HAL_UART_Receive_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte);
hal_status_t HAL_UART_Transmit_DMA_Opt(hal_uart_handle_t *huart, const void *p_data, uint32_t size_byte,
                                       uint32_t interrupts);
hal_status_t HAL_UART_Receive_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte, uint32_t interrupts);
hal_status_t HAL_UART_Pause_DMA(hal_uart_handle_t *huart);
hal_status_t HAL_UART_PauseReceive_DMA(hal_uart_handle_t *huart);
hal_status_t HAL_UART_PauseTransmit_DMA(hal_uart_handle_t *huart);

hal_status_t HAL_UART_Resume_DMA(hal_uart_handle_t *huart);
hal_status_t HAL_UART_ResumeReceive_DMA(hal_uart_handle_t *huart);
hal_status_t HAL_UART_ResumeTransmit_DMA(hal_uart_handle_t *huart);
#endif /* USE_HAL_UART_DMA */

/* Transfer Abort functions */
hal_status_t HAL_UART_Abort(hal_uart_handle_t *huart);
hal_status_t HAL_UART_AbortTransmit(hal_uart_handle_t *huart);
hal_status_t HAL_UART_AbortReceive(hal_uart_handle_t *huart);
hal_status_t HAL_UART_Abort_IT(hal_uart_handle_t *huart);
hal_status_t HAL_UART_AbortTransmit_IT(hal_uart_handle_t *huart);
hal_status_t HAL_UART_AbortReceive_IT(hal_uart_handle_t *huart);

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group13 Advanced I/O operation functions
  * @{
  */
hal_status_t HAL_UART_SendLINBreak(hal_uart_handle_t *huart);
hal_status_t HAL_UART_SendRequest(hal_uart_handle_t *huart, hal_uart_request_t request);

hal_status_t HAL_UART_ReceiveToIdle(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                    uint32_t *p_rx_size_byte, uint32_t timeout_ms);
hal_status_t HAL_UART_ReceiveToIdle_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte);

hal_status_t HAL_UART_ReceiveToIdle_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                           uint32_t interrupts);

hal_status_t HAL_UART_ReceiveUntilTMO(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                      uint32_t *p_rx_size_byte, uint32_t char_timeout_bit);
hal_status_t HAL_UART_ReceiveUntilTMO_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                         uint32_t char_timeout_bit);

hal_status_t HAL_UART_ReceiveUntilTMO_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                             uint32_t char_timeout_bit, uint32_t interrupts);

hal_status_t HAL_UART_ReceiveUntilCM(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                     uint8_t character, uint32_t *p_rx_size_byte, uint32_t timeout_ms);
hal_status_t HAL_UART_ReceiveUntilCM_IT(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                        uint8_t character);

hal_status_t HAL_UART_ReceiveUntilCM_IT_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                            uint8_t character, uint32_t interrupts);

#if defined (USE_HAL_UART_DMA) && (USE_HAL_UART_DMA == 1)
hal_status_t HAL_UART_ReceiveToIdle_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte);
hal_status_t HAL_UART_ReceiveUntilTMO_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                          uint32_t char_timeout_bit);
hal_status_t HAL_UART_ReceiveUntilCM_DMA(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                         uint8_t character);

hal_status_t HAL_UART_ReceiveToIdle_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                            uint32_t interrupts);
hal_status_t HAL_UART_ReceiveUntilTMO_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                              uint32_t char_timeout_bit, uint32_t interrupts);
hal_status_t HAL_UART_ReceiveUntilCM_DMA_Opt(hal_uart_handle_t *huart, void *p_data, uint32_t size_byte,
                                             uint8_t character, uint32_t interrupts);
#endif /* USE_HAL_UART_DMA */

/**
  * @}
  */
/** @defgroup UART_Exported_Functions_Group14 Peripheral current frequency, state and error functions
  * @{
  */

/* Peripheral current frequency, state and error functions *********************************/
uint32_t HAL_UART_GetClockFreq(const hal_uart_handle_t *huart);
hal_uart_state_t HAL_UART_GetState(const hal_uart_handle_t *huart);
hal_uart_tx_state_t HAL_UART_GetTxState(const hal_uart_handle_t *huart);
hal_uart_rx_state_t HAL_UART_GetRxState(const hal_uart_handle_t *huart);

#if defined (USE_HAL_UART_GET_LAST_ERRORS) && (USE_HAL_UART_GET_LAST_ERRORS == 1)
uint32_t HAL_UART_GetLastErrorCodes(const hal_uart_handle_t *huart);
#endif /* USE_HAL_UART_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup UART_Exported_Functions_Group15 Bus Operation Function
  * @{
  */
hal_status_t HAL_UART_AcquireBus(hal_uart_handle_t *huart, uint32_t timeout_ms);
hal_status_t HAL_UART_ReleaseBus(hal_uart_handle_t *huart);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_UART_USER_DATA) && (USE_HAL_UART_USER_DATA == 1)
/** @defgroup UART_Exported_Functions_Group16 User Data Function
  * @{
  */
void HAL_UART_SetUserData(hal_uart_handle_t *huart, const void *p_user_data);
const void *HAL_UART_GetUserData(const hal_uart_handle_t *huart);
/**
  * @}
  */
#endif /* USE_HAL_UART_USER_DATA */

/** @defgroup UART_Exported_Functions_Group17 IRQ handling
  * @{
  */
void HAL_UART_IRQHandler(hal_uart_handle_t *huart);
/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group18 Default Callbacks
  * @{
  */

/* Default callback */
void HAL_UART_TxHalfCpltCallback(hal_uart_handle_t *huart);
void HAL_UART_TxCpltCallback(hal_uart_handle_t *huart);
void HAL_UART_RxHalfCpltCallback(hal_uart_handle_t *huart);
void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte, hal_uart_rx_event_types_t rx_event);
void HAL_UART_ErrorCallback(hal_uart_handle_t *huart);
void HAL_UART_AbortCpltCallback(hal_uart_handle_t *huart);
void HAL_UART_AbortTransmitCpltCallback(hal_uart_handle_t *huart);
void HAL_UART_AbortReceiveCpltCallback(hal_uart_handle_t *huart);

void HAL_UART_WakeupCallback(hal_uart_handle_t *huart);

void HAL_UART_RxFifoFullCallback(hal_uart_handle_t *huart);
void HAL_UART_TxFifoEmptyCallback(hal_uart_handle_t *huart);

void HAL_UART_LINBreakCallback(hal_uart_handle_t *huart);
void HAL_UART_ClearToSendCallback(hal_uart_handle_t *huart);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
#endif /* USART1 || USART2 || USART3 || UART4 || UART5 || USART6 || UART7 || LPUART1 */
/**
  * @}
  */
#endif /* STM32C5XX_HAL_UART_H */
