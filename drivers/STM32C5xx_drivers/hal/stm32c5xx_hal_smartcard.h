/**
  ******************************************************************************
  * @file    stm32c5xx_hal_smartcard.h
  * @brief   Header file of SMARTCARD HAL module.
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
#ifndef STM32C5XX_HAL_SMARTCARD_H
#define STM32C5XX_HAL_SMARTCARD_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_usart.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) \
 || defined(UART7)
/** @defgroup SMARTCARD SMARTCARD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Types HAL SMARTCARD Types
  * @{
  */

/**
  * @brief HAL SMARTCARD Instance Definition.
  */
typedef enum
{
  /*! Instance USART1 */
  HAL_SMARTCARD1 = USART1_BASE,
  /*! Instance USART2 */
  HAL_SMARTCARD2 = USART2_BASE,
#if defined(USART3)
  /*! Instance USART3 */
  HAL_SMARTCARD3 = USART3_BASE,
#endif /* USART3 */
#if defined(USART6)
  /*! Instance USART6 */
  HAL_SMARTCARD6 = USART6_BASE,
#endif /* USART6 */
} hal_smartcard_t;

/**
  * @brief HAL SMARTCARD State enum Definition.
  */
typedef enum
{
  /*! Peripheral is not initialized */
  HAL_SMARTCARD_STATE_RESET            = 0U,
  /*! Peripheral is initialized but not configured */
  HAL_SMARTCARD_STATE_INIT             = (1UL << 31U),
  /*! Peripheral is initialized and a config is set */
  HAL_SMARTCARD_STATE_IDLE             = (1UL << 30U),
  /*! Peripheral is receiving */
  HAL_SMARTCARD_STATE_RX_ACTIVE        = (1UL << 29U),
  /*! Peripheral is transmitting */
  HAL_SMARTCARD_STATE_TX_ACTIVE        = (1UL << 28U),
  /*! Peripheral is aborting the current process */
  HAL_SMARTCARD_STATE_ABORT            = (1UL << 27U)
} hal_smartcard_state_t;

/**
  * @brief HAL SMARTCARD Stop bits enum definition.
  */
typedef enum
{
  /*!< 0.5 stop bit */
  HAL_SMARTCARD_STOP_BIT_0_5           =        LL_USART_STOP_BIT_0_5,
  /*!< 1.5 stop bits */
  HAL_SMARTCARD_STOP_BIT_1_5           =        LL_USART_STOP_BIT_1_5
} hal_smartcard_stop_bits_t;

/**
  * @brief HAL SMARTCARD inversion status definition.
  */
typedef enum
{
  /*!< Not Inverted */
  HAL_SMARTCARD_IO_INVERT_DISABLED     =        0U,
  /*!< Inverted */
  HAL_SMARTCARD_IO_INVERT_ENABLED      =        1U
} hal_smartcard_io_invert_status_t;

/**
  * @brief HAL SMARTCARD data status definition.
  */
typedef enum
{
  /*! SMARTCARD Data Binary Inversion is disabled */
  HAL_SMARTCARD_DATA_INVERT_DISABLED   = 0U,
  /*! SMARTCARD Data Binary Inversion is enabled */
  HAL_SMARTCARD_DATA_INVERT_ENABLED    = 1U,
} hal_smartcard_data_invert_status_t;

/**
  * @brief HAL SMARTCARD Swap Tx/Rx Status Definition.
  */
typedef enum
{
  /*! SMARTCARD Tx Rx Swap Pins is disabled */
  HAL_SMARTCARD_TX_RX_SWAP_DISABLED    = 0U,
  /*! SMARTCARD Tx Rx Swap Pins is enabled */
  HAL_SMARTCARD_TX_RX_SWAP_ENABLED     = 1U,
} hal_smartcard_tx_rx_swap_status_t;

/**
  * @brief HAL SMARTCARD Bit order enum definition.
  */
typedef enum
{
  /*!< LSB First */
  HAL_SMARTCARD_BIT_ORDER_LSB_FIRST    =        LL_USART_BITORDER_LSB_FIRST,
  /*!< MSB First */
  HAL_SMARTCARD_BIT_ORDER_MSB_FIRST    =        LL_USART_BITORDER_MSB_FIRST
} hal_smartcard_bit_order_t ;

/**
  * @brief HAL SMARTCARD Parity enum definition.
  */
typedef enum
{
  /*!< Parity control enabled and Even Parity is selected */
  HAL_SMARTCARD_PARITY_ODD     =      LL_USART_PARITY_ODD,
  /*!< Parity control enabled and Odd Parity is selected */
  HAL_SMARTCARD_PARITY_EVEN    =      LL_USART_PARITY_EVEN
} hal_smartcard_parity_t;

/**
  * @brief HAL SMARTCARD Overrun enum definition.
  */
typedef enum
{
  /*!< Overrun Rx errors detection enabled */
  HAL_SMARTCARD_OVERRUN_DETECT_ENABLED         =        1U,
  /*!< Overrun Rx errors detection disabled */
  HAL_SMARTCARD_OVERRUN_DETECT_DISABLED        =        0U
} hal_smartcard_rx_overrun_detection_status_t;

/**
  * @brief HAL SMARTCARD DMA stop on Rx error enum definition.
  */
typedef enum
{
  /*!< No impact on DMA */
  HAL_SMARTCARD_DMA_STOP_NONE          =        0U,
  /*!< DMA disable on rx error */
  HAL_SMARTCARD_DMA_STOP_ON_RX_ERROR   =        1U
} hal_smartcard_dma_stop_status_t;

/**
  * @brief HAL SMARTCARD NACK management enum definition.
  */
typedef enum
{
  /*!< NACK disabled */
  HAL_SMARTCARD_NACK_DISABLE          =        0U,
  /*!< NACK enabled */
  HAL_SMARTCARD_NACK_ENABLE           =        1U
} hal_smartcard_nack_state_t;

/**
  * @brief HAL SMARTCARD Smartcard prescaler enum definition.
  */
typedef enum
{
  /*!< USART input CLK \1 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV1         = LL_USART_PRESCALER_DIV1,
  /*!< USART input CLK \2 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV2         = LL_USART_PRESCALER_DIV2,
  /*!< USART input CLK \4 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV4         = LL_USART_PRESCALER_DIV4,
  /*!< USART input CLK \6 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV6         = LL_USART_PRESCALER_DIV6,
  /*!< USART input CLK \8 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV8         = LL_USART_PRESCALER_DIV8,
  /*!< USART input CLK \10 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV10        = LL_USART_PRESCALER_DIV10,
  /*!< USART input CLK \12 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV12        = LL_USART_PRESCALER_DIV12,
  /*!< USART input CLK \16 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV16        = LL_USART_PRESCALER_DIV16,
  /*!< USART input CLK \32 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV32        = LL_USART_PRESCALER_DIV32,
  /*!< USART input CLK \64 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV64        = LL_USART_PRESCALER_DIV64,
  /*!< USART input CLK \128 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV128       = LL_USART_PRESCALER_DIV128,
  /*!< USART input CLK \256 */
  HAL_SMARTCARD_CLOCK_PRESC_DIV256       = LL_USART_PRESCALER_DIV256
} hal_smartcard_prescaler_t;

/**
  * @brief HAL SMARTCARD Smartcard SCLK prescaler enum definition.
  */
typedef enum
{
  /*!< SMARTCARD Output CLK /2 */
  HAL_SMARTCARD_SCLK_PRESC_DIV2        = LL_USART_SMARTCARD_PRESCALER_DIV2,
  /*!< SMARTCARD Output CLK /4 */
  HAL_SMARTCARD_SCLK_PRESC_DIV4        = LL_USART_SMARTCARD_PRESCALER_DIV4,
  /*!< SMARTCARD Output CLK /6 */
  HAL_SMARTCARD_SCLK_PRESC_DIV6        = LL_USART_SMARTCARD_PRESCALER_DIV6,
  /*!< SMARTCARD Output CLK /8 */
  HAL_SMARTCARD_SCLK_PRESC_DIV8        = LL_USART_SMARTCARD_PRESCALER_DIV8,
  /*!< SMARTCARD Output CLK /10 */
  HAL_SMARTCARD_SCLK_PRESC_DIV10       = LL_USART_SMARTCARD_PRESCALER_DIV10,
  /*!< SMARTCARD Output CLK /12 */
  HAL_SMARTCARD_SCLK_PRESC_DIV12       = LL_USART_SMARTCARD_PRESCALER_DIV12,
  /*!< SMARTCARD Output CLK /14 */
  HAL_SMARTCARD_SCLK_PRESC_DIV14       = LL_USART_SMARTCARD_PRESCALER_DIV14,
  /*!< SMARTCARD Output CLK /16 */
  HAL_SMARTCARD_SCLK_PRESC_DIV16       = LL_USART_SMARTCARD_PRESCALER_DIV16,
  /*!< SMARTCARD Output CLK /18 */
  HAL_SMARTCARD_SCLK_PRESC_DIV18       = LL_USART_SMARTCARD_PRESCALER_DIV18,
  /*!< SMARTCARD Output CLK /20 */
  HAL_SMARTCARD_SCLK_PRESC_DIV20       = LL_USART_SMARTCARD_PRESCALER_DIV20,
  /*!< SMARTCARD Output CLK /22 */
  HAL_SMARTCARD_SCLK_PRESC_DIV22       = LL_USART_SMARTCARD_PRESCALER_DIV22,
  /*!< SMARTCARD Output CLK /24 */
  HAL_SMARTCARD_SCLK_PRESC_DIV24       = LL_USART_SMARTCARD_PRESCALER_DIV24,
  /*!< SMARTCARD Output CLK /26 */
  HAL_SMARTCARD_SCLK_PRESC_DIV26       = LL_USART_SMARTCARD_PRESCALER_DIV26,
  /*!< SMARTCARD Output CLK /28 */
  HAL_SMARTCARD_SCLK_PRESC_DIV28       = LL_USART_SMARTCARD_PRESCALER_DIV28,
  /*!< SMARTCARD Output CLK /30 */
  HAL_SMARTCARD_SCLK_PRESC_DIV30       = LL_USART_SMARTCARD_PRESCALER_DIV30,
  /*!< SMARTCARD Output CLK /32 */
  HAL_SMARTCARD_SCLK_PRESC_DIV32       = LL_USART_SMARTCARD_PRESCALER_DIV32,
  /*!< SMARTCARD Output CLK /34 */
  HAL_SMARTCARD_SCLK_PRESC_DIV34       = LL_USART_SMARTCARD_PRESCALER_DIV34,
  /*!< SMARTCARD Output CLK /36 */
  HAL_SMARTCARD_SCLK_PRESC_DIV36       = LL_USART_SMARTCARD_PRESCALER_DIV36,
  /*!< SMARTCARD Output CLK /38 */
  HAL_SMARTCARD_SCLK_PRESC_DIV38       = LL_USART_SMARTCARD_PRESCALER_DIV38,
  /*!< SMARTCARD Output CLK /40 */
  HAL_SMARTCARD_SCLK_PRESC_DIV40       = LL_USART_SMARTCARD_PRESCALER_DIV40,
  /*!< SMARTCARD Output CLK /42 */
  HAL_SMARTCARD_SCLK_PRESC_DIV42       = LL_USART_SMARTCARD_PRESCALER_DIV42,
  /*!< SMARTCARD Output CLK /44 */
  HAL_SMARTCARD_SCLK_PRESC_DIV44       = LL_USART_SMARTCARD_PRESCALER_DIV44,
  /*!< SMARTCARD Output CLK /46 */
  HAL_SMARTCARD_SCLK_PRESC_DIV46       = LL_USART_SMARTCARD_PRESCALER_DIV46,
  /*!< SMARTCARD Output CLK /48 */
  HAL_SMARTCARD_SCLK_PRESC_DIV48       = LL_USART_SMARTCARD_PRESCALER_DIV48,
  /*!< SMARTCARD Output CLK /50 */
  HAL_SMARTCARD_SCLK_PRESC_DIV50       = LL_USART_SMARTCARD_PRESCALER_DIV50,
  /*!< SMARTCARD Output CLK /52 */
  HAL_SMARTCARD_SCLK_PRESC_DIV52       = LL_USART_SMARTCARD_PRESCALER_DIV52,
  /*!< SMARTCARD Output CLK /54 */
  HAL_SMARTCARD_SCLK_PRESC_DIV54       = LL_USART_SMARTCARD_PRESCALER_DIV54,
  /*!< SMARTCARD Output CLK /56 */
  HAL_SMARTCARD_SCLK_PRESC_DIV56       = LL_USART_SMARTCARD_PRESCALER_DIV56,
  /*!< SMARTCARD Output CLK /58 */
  HAL_SMARTCARD_SCLK_PRESC_DIV58       = LL_USART_SMARTCARD_PRESCALER_DIV58,
  /*!< SMARTCARD Output CLK /60 */
  HAL_SMARTCARD_SCLK_PRESC_DIV60       = LL_USART_SMARTCARD_PRESCALER_DIV60,
  /*!< SMARTCARD Output CLK /62 */
  HAL_SMARTCARD_SCLK_PRESC_DIV62       = LL_USART_SMARTCARD_PRESCALER_DIV62
} hal_smartcard_source_clock_prescaler_t;

/**
  * @brief HAL SMARTCARD Clock Output enum definition.
  */
typedef enum
{
  /*!< Clock signal output on CK pin disabled */
  HAL_SMARTCARD_CLOCK_OUTPUT_DISABLE     = LL_USART_CLOCK_OUTPUT_DISABLED,
  /*!< Clock signal output on CK pin enabled  */
  HAL_SMARTCARD_CLOCK_OUTPUT_ENABLE      = LL_USART_CLOCK_OUTPUT_ENABLED
} hal_smartcard_clock_output_t;

/**
  * @brief HAL SMARTCARD Clock polarity enum definition.
  */
typedef enum
{
  /*!< Polarity Low */
  HAL_SMARTCARD_CLOCK_POLARITY_LOW       = LL_USART_CLOCK_POLARITY_LOW,
  /*!< Polarity High */
  HAL_SMARTCARD_CLOCK_POLARITY_HIGH      = LL_USART_CLOCK_POLARITY_HIGH
} hal_smartcard_clock_polarity_t;

/**
  * @brief HAL SMARTCARD Clock phase enum definition.
  */
typedef enum
{
  /*!< The first clock transition is the first data capture edge */
  HAL_SMARTCARD_CLOCK_PHASE_1_EDGE        = LL_USART_CLOCK_PHASE_1_EDGE,
  /*!< The second clock transition is the first data capture edge */
  HAL_SMARTCARD_CLOCK_PHASE_2_EDGE        = LL_USART_CLOCK_PHASE_2_EDGE
} hal_smartcard_clock_phase_t;

/**
  * @brief HAL SMARTCARD Timeout status definition.
  */
typedef enum
{
  /*!< Timeout disabled */
  HAL_SMARTCARD_TIMEOUT_DISABLED       = 0U,
  /*!< Timeout enabled */
  HAL_SMARTCARD_TIMEOUT_ENABLED        = 1U
} hal_smartcard_timeout_status_t;

/**
  * @brief HAL SMARTCARD Pre guard time Tx complete indication enum definition.
  */
typedef enum
{
  /*!< SMARTCARD transmission complete (flag raised when guard time has elapsed) */
  HAL_SMARTCARD_TX_CPLT_AFTER_GUARD_TIME   = 0U,
  /*!< SMARTCARD transmission complete before guard time */
  HAL_SMARTCARD_TX_CPLT_BEFORE_GUARD_TIME  = 1U

} hal_smartcard_tx_cplt_guard_time_indication_t ;

/**
  * @brief HAL SMARTCARD End of block interrupt status definition.
  */
typedef enum
{
  /*!< End of block interrupt disabled */
  HAL_SMARTCARD_EOB_IT_DISABLED       = 0U,
  /*!< End of block interrupt enabled */
  HAL_SMARTCARD_EOB_IT_ENABLED        = 1U
} hal_smartcard_end_of_block_interrupt_status_t;

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)

/**
  * @brief HAL SMARTCARD Fifo status definition.
  */
typedef enum
{
  /*!< Fifo disabled */
  HAL_SMARTCARD_FIFO_MODE_DISABLED          = 0U,
  /*!< Fifo enabled */
  HAL_SMARTCARD_FIFO_MODE_ENABLED           = 1U
} hal_smartcard_fifo_mode_status_t;

/**
  * @brief HAL SMARTCARD Fifo threshold enum definition.
  */
typedef enum
{
  /*!< FIFO reaches 1/8 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_1_8     = LL_USART_FIFO_THRESHOLD_1_8,
  /*!< FIFO reaches 1/4 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_1_4     = LL_USART_FIFO_THRESHOLD_1_4,
  /*!< FIFO reaches 1/2 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_1_2     = LL_USART_FIFO_THRESHOLD_1_2,
  /*!< FIFO reaches 3/4 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_3_4     = LL_USART_FIFO_THRESHOLD_3_4,
  /*!< FIFO reaches 7/8 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_7_8     = LL_USART_FIFO_THRESHOLD_7_8,
  /*!< FIFO reaches 8/8 of its depth */
  HAL_SMARTCARD_FIFO_THRESHOLD_8_8     = LL_USART_FIFO_THRESHOLD_8_8
} hal_smartcard_fifo_threshold_t;
#endif /* USE_HAL_SMARTCARD_FIFO */

/*! HAL SMARTCARD handler type */
typedef struct hal_smartcard_handle_s hal_smartcard_handle_t;

#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/*! HAL SMARTCARD Generic SMARTCARD callback Type */
typedef void (* hal_smartcard_cb_t)(hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @brief HAL SMARTCARD handle structure type.
  */
struct hal_smartcard_handle_s
{
  /*! Peripheral instance        */
  hal_smartcard_t instance;

  /*! SMARTCARD state information related to global handle management */
  volatile hal_smartcard_state_t global_state;

  /*! Pointer to SMARTCARD Tx transfer buffer */
  const uint8_t *p_tx_buff;

  /*! SMARTCARD Tx Transfer size              */
  volatile uint32_t tx_xfer_size;

  /*! SMARTCARD Tx Transfer Counter           */
  volatile uint32_t tx_xfer_count;

  /*! Pointer to SMARTCARD Rx transfer buffer */
  uint8_t *p_rx_buff;

  /*! SMARTCARD Rx Transfer size              */
  volatile uint32_t rx_xfer_size;

  /*! SMARTCARD Rx Transfer Counter           */
  volatile uint32_t rx_xfer_count;

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /*! Specifies whether the FIFO mode is used */
  hal_smartcard_fifo_mode_status_t fifo_status;

  /*! Number of data to process during RX ISR execution */
  uint16_t nb_rx_data_to_process;

  /*! Number of data to process during TX ISR execution */
  uint16_t nb_tx_data_to_process;

#endif /* USE_HAL_SMARTCARD_FIFO */
  /*! Tx complete indication configuration: before guard time or after */
  hal_smartcard_tx_cplt_guard_time_indication_t   tx_cplt_indication;

  /*! Function pointer to Rx IRQ handler */
  void (*p_rx_isr)(struct hal_smartcard_handle_s *hsmartcard);

  /*! Function pointer to Tx IRQ handler */
  void (*p_tx_isr)(struct hal_smartcard_handle_s *hsmartcard);

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
  /*! SMARTCARD Tx DMA handle parameters      */
  hal_dma_handle_t *hdma_tx;

  /*! SMARTCARD Rx DMA handle parameters      */
  hal_dma_handle_t *hdma_rx;

#endif /* USE_HAL_SMARTCARD_DMA */
#if defined(USE_HAL_SMARTCARD_REGISTER_CALLBACKS) && (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /*! SMARTCARD Tx complete callback             */
  hal_smartcard_cb_t p_tx_cplt_callback;

  /*! SMARTCARD Tx Half complete callback        */
  hal_smartcard_cb_t p_tx_half_cplt_callback;

  /*! SMARTCARD Rx complete callback             */
  hal_smartcard_cb_t p_rx_cplt_callback;

  /*! SMARTCARD Rx Half complete callback        */
  hal_smartcard_cb_t p_rx_half_cplt_callback;

  /*! SMARTCARD Error callback                   */
  hal_smartcard_cb_t p_error_callback;

  /*! SMARTCARD Abort complete callback          */
  hal_smartcard_cb_t p_abort_cplt_callback;

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
  /*! SMARTCARD Rx FIFO full callback            */
  hal_smartcard_cb_t p_rx_fifo_full_callback;

  /*! SMARTCARD Tx FIFO empty callback           */
  hal_smartcard_cb_t p_tx_fifo_empty_callback;

#endif /* USE_HAL_SMARTCARD_FIFO */
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /*! USART OS semaphore */
  hal_os_semaphore_t semaphore;

#endif /* USE_HAL_MUTEX */
#if defined (USE_HAL_SMARTCARD_USER_DATA) && (USE_HAL_SMARTCARD_USER_DATA == 1)
  /*! User data pointer */
  const void *p_user_data;

#endif /* USE_HAL_SMARTCARD_USER_DATA */
#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
  /*! Last error codes */
  volatile uint32_t last_error_codes;
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
};

/**
  * @brief HAL SMARTCARD Global configuration structure definition.
  */
typedef struct
{
  uint32_t                           baud_rate;            /*!< Configures the SMARTCARD communication baud rate. The
                                                                baud rate register is computed using the following
                                                                formula:
                                                                baud rate register = usart_ker_ckpres / baud_rate
                                                                where usart_ker_ckpres is the USART input clock divided
                                                                by a prescaler. */

  hal_smartcard_stop_bits_t          stop_bits;            /*!< Specifies the number of stop bits. */

  hal_smartcard_bit_order_t          first_bit;            /*!< Specifies whether MSB is sent first on the USART line. */

  hal_smartcard_parity_t             parity;               /*!< Specifies the parity mode. The parity is enabled by
                                                                default (PCE is forced to 1). Since the WordLength is
                                                                forced to 8 bits + parity, M is forced to 1 and the
                                                                parity bit is the 9th bit. */

  hal_smartcard_nack_state_t         nack;                 /*!< Specifies whether the SMARTCARD NACK transmission
                                                                is enabled in case of parity error. */

  hal_smartcard_prescaler_t          clock_prescaler;      /*!< Specifies the prescaler value used to divide the USART
                                                                input clock to provide USART clock source. */

  hal_smartcard_source_clock_prescaler_t     sclk_prescaler; /*!< Specifies the SMARTCARD prescaler used to divide the
                                                                USART clock, the clock sent to the smartcard is the
                                                                output clock after the division. */

  hal_smartcard_clock_output_t       clock_output;         /*!< Specifies whether the CLK signal is output or not. */

  hal_smartcard_clock_polarity_t     clock_polarity;       /*!< Specifies the steady state of the serial clock. */

  hal_smartcard_clock_phase_t        clock_phase;          /*!< Specifies the clock transition on which the bit capture
                                                                is made. */

  uint32_t                           guard_time_etu;       /*!< Specifies the SMARTCARD guard time ETU
                                                                (Elementary Time Unit) applied after stop bits. */

  uint32_t                           auto_retry_count;     /*!< Specifies the SMARTCARD auto-retry count (number of
                                                                retries in receive and transmit mode). When set to 0,
                                                                retransmission is disabled. Otherwise, its maximum value
                                                                is 7 (before signalling an error). */
} hal_smartcard_config_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup SMARTCARD_Exported_Constants HAL SMARTCARD Constants
  * @{
  */

/** @defgroup SMARTCARD_Error_Code SMARTCARD Error Codes
  * @{
  */
/*! No error */
#define HAL_SMARTCARD_ERROR_NONE     (0UL)

/*! Parity error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_PE       (0x1UL << 0)

/*! Noise error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_NE       (0x1UL << 1U)

/*! Frame error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_FE       (0x1UL << 2U)

/*! Overrun error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_ORE      (0x1UL << 3U)

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1U)
/*! DMA transfer error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_DMA      (0x1UL << 4U)

#endif /* USE_HAL_SMARTCARD_DMA */
/*! Receiver timeout error on RX */
#define HAL_SMARTCARD_RECEIVE_ERROR_RTO      (0x1UL << 5U)

/*! No ACK after transmit despite trials */
#define HAL_SMARTCARD_TRANSMIT_ERROR_NACK    (0x1UL << 6U)

#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1U)
/*! DMA transfer error on TX */
#define HAL_SMARTCARD_TRANSMIT_ERROR_DMA     (0x1UL << 16U)
#endif /* USE_HAL_SMARTCARD_DMA */
/**
  * @}
  */

/** @defgroup SMARTCARD_Optional_Interrupts SMARTCARD Optional interrupts
  * @{
  */

/** @defgroup SMARTCARD_Transmit_IT_Optional_Interrupts SMARTCARD optional TX IT interrupts
  * @{
  */
/*! All optional interrupts are disabled */
#define HAL_SMARTCARD_OPT_TX_IT_NONE           0U
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/*! Enable optional FIFO EMPTY IT for TX_IT_Opt */
#define HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY     (1UL << 30U)
/*! Activate default optional IT for transmit IT based process */
#define HAL_SMARTCARD_OPT_TX_IT_DEFAULT        (HAL_SMARTCARD_OPT_TX_IT_FIFO_EMPTY)
#endif /* USE_HAL_SMARTCARD_FIFO */
/**
  * @}
  */
#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/** @defgroup SMARTCARD_Transmit_DMA_Optional_Interrupts SMARTCARD Optional TX DMA interrupts
  * @{
  */
/*! All optional interrupts are disabled */
#define HAL_SMARTCARD_OPT_DMA_TX_IT_NONE          0U
/*! Enable optional HT IT for TX_DMA_Opt */
#define HAL_SMARTCARD_OPT_DMA_TX_IT_HT            (HAL_DMA_OPT_IT_HT)
/*! Enable all optional IT for TX_DMA_Opt */
#define HAL_SMARTCARD_OPT_DMA_TX_IT_DEFAULT       (HAL_SMARTCARD_OPT_DMA_TX_IT_HT)
/**
  * @}
  */
#endif /* USE_HAL_SMARTCARD_DMA */

/** @defgroup SMARTCARD_Receive_IT_Optional_Interrupts SMARTCARD Optional RX IT interrupts
  * @{
  */
/*! All optional interrupts are disabled */
#define HAL_SMARTCARD_OPT_RX_IT_NONE           0U
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
/*! Enable optional FIFO FULL IT for RX_IT_Opt */
#define HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL      (1UL << 25U)
/*! Activate default optional IT for receive IT-based process */
#define HAL_SMARTCARD_OPT_RX_IT_DEFAULT        (HAL_SMARTCARD_OPT_RX_IT_FIFO_FULL)
#endif /* USE_HAL_SMARTCARD_FIFO */
/**
  * @}
  */

#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
/** @defgroup SMARTCARD_Receive_DMA_Optional_Interrupts SMARTCARD Optional RX DMA interrupts
  * @{
  */
#define HAL_SMARTCARD_OPT_DMA_RX_IT_NONE         0U                      /*!< All optional interrupts are disabled */
#define HAL_SMARTCARD_OPT_DMA_RX_IT_HT           (HAL_DMA_OPT_IT_HT)     /*!< Enable optional HT IT for RX_DMA_Opt */
#define HAL_SMARTCARD_OPT_DMA_RX_IT_DEFAULT      (HAL_SMARTCARD_OPT_DMA_RX_IT_HT)
/*!< Enable all optional IT for RX_DMA_Opt */
/**
  * @}
  */
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Functions HAL SMARTCARD Functions
  * @{
  */

/* Initialization and de-initialization functions *****************************/
/** @defgroup SMARTCARD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

hal_status_t HAL_SMARTCARD_Init(hal_smartcard_handle_t *hsmartcard, hal_smartcard_t instance);
hal_status_t HAL_SMARTCARD_DeInit(hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */

/* IO operation functions *****************************************************/
/** @defgroup SMARTCARD_Exported_Functions_Group2 General Config functions
  * @{
  */

hal_status_t HAL_SMARTCARD_SetConfig(hal_smartcard_handle_t *hsmartcard, const hal_smartcard_config_t *p_config);
hal_status_t HAL_SMARTCARD_GetConfig(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_config_t *p_config);

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group3 Unitary basic config functions
  * @{
  */

hal_status_t HAL_SMARTCARD_SetBaudRate(const hal_smartcard_handle_t *hsmartcard, uint32_t baud_rate);
uint32_t HAL_SMARTCARD_GetBaudRate(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetStopBits(const hal_smartcard_handle_t *hsmartcard,
                                       hal_smartcard_stop_bits_t stop_bits);
hal_smartcard_stop_bits_t HAL_SMARTCARD_GetStopBits(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetFirstBit(const hal_smartcard_handle_t *hsmartcard,
                                       hal_smartcard_bit_order_t first_bit);
hal_smartcard_bit_order_t HAL_SMARTCARD_GetFirstBit(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetParity(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_parity_t parity);
hal_smartcard_parity_t HAL_SMARTCARD_GetParity(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetNack(const hal_smartcard_handle_t *hsmartcard, hal_smartcard_nack_state_t nack);
hal_smartcard_nack_state_t HAL_SMARTCARD_GetNack(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetClockOutput(const hal_smartcard_handle_t *hsmartcard,
                                          hal_smartcard_clock_output_t clock_output);
hal_smartcard_clock_output_t HAL_SMARTCARD_GetClockOutput(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetClockPolarity(const hal_smartcard_handle_t *hsmartcard,
                                            hal_smartcard_clock_polarity_t clock_polarity);
hal_smartcard_clock_polarity_t HAL_SMARTCARD_GetClockPolarity(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetClockPhase(const hal_smartcard_handle_t *hsmartcard,
                                         hal_smartcard_clock_phase_t clock_phase);
hal_smartcard_clock_phase_t HAL_SMARTCARD_GetClockPhase(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetGuardTime(const hal_smartcard_handle_t *hsmartcard, uint32_t guard_time_etu);
uint32_t HAL_SMARTCARD_GetGuardTime(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetAutoRetryCount(const hal_smartcard_handle_t *hsmartcard, uint32_t retry_count);
uint32_t HAL_SMARTCARD_GetAutoRetryCount(const hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group4 Advanced config functions
  * @{
  */

hal_status_t HAL_SMARTCARD_EnableIOInvert(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableIOInvert(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_io_invert_status_t HAL_SMARTCARD_IsEnabledIOInvert(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_EnableDataInvert(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableDataInvert(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_data_invert_status_t HAL_SMARTCARD_IsEnabledDataInvert(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_EnableTxRxSwap(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableTxRxSwap(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_tx_rx_swap_status_t HAL_SMARTCARD_IsEnabledTxRxSwap(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_EnableRxOverRunDetection(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableRxOverRunDetection(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_rx_overrun_detection_status_t HAL_SMARTCARD_IsEnabledRxOverRunDetection(
  const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_EnableDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_dma_stop_status_t HAL_SMARTCARD_IsEnabledDMAStopOnRxError(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetReceiverTimeout(const hal_smartcard_handle_t *hsmartcard, uint32_t timeout_etu);
uint32_t HAL_SMARTCARD_GetReceiverTimeout(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_EnableReceiverTimeout(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableReceiverTimeout(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_timeout_status_t HAL_SMARTCARD_IsEnabledReceiverTimeout(const hal_smartcard_handle_t *hsmartcard);

hal_status_t HAL_SMARTCARD_SetTxCpltIndication(hal_smartcard_handle_t *hsmartcard,
                                               const hal_smartcard_tx_cplt_guard_time_indication_t  tx_cplt_indication);
hal_smartcard_tx_cplt_guard_time_indication_t HAL_SMARTCARD_GetTxCpltIndication(const hal_smartcard_handle_t
                                                                                *hsmartcard);

hal_status_t HAL_SMARTCARD_SetBlockLength(const hal_smartcard_handle_t *hsmartcard, uint32_t block_length_byte);
uint32_t HAL_SMARTCARD_GetBlockLength(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_EnableEndOfBlockIT(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableEndOfBlockIT(const hal_smartcard_handle_t *hsmartcard);
hal_smartcard_end_of_block_interrupt_status_t HAL_SMARTCARD_IsEnabledEndOfBlockIT(const hal_smartcard_handle_t
    *hsmartcard);

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group5 FIFO config functions
  * @{
  */
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
hal_status_t HAL_SMARTCARD_EnableFifoMode(hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_DisableFifoMode(hal_smartcard_handle_t *hsmartcard);
hal_smartcard_fifo_mode_status_t HAL_SMARTCARD_IsEnabledFifoMode(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_SetTxFifoThreshold(hal_smartcard_handle_t *hsmartcard,
                                              const hal_smartcard_fifo_threshold_t tx_fifo_threshold);
hal_smartcard_fifo_threshold_t HAL_SMARTCARD_GetTxFifoThreshold(const hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_SetRxFifoThreshold(hal_smartcard_handle_t *hsmartcard,
                                              const hal_smartcard_fifo_threshold_t rx_fifo_threshold);
hal_smartcard_fifo_threshold_t HAL_SMARTCARD_GetRxFifoThreshold(const hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_FIFO */
/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group6 IO operation functions
  * @{
  */
hal_status_t HAL_SMARTCARD_Transmit(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint16_t size_byte,
                                    uint32_t timeout_ms);
hal_status_t HAL_SMARTCARD_Receive(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint16_t size_byte,
                                   uint32_t timeout_ms);
hal_status_t HAL_SMARTCARD_Transmit_IT(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint16_t size_byte);
hal_status_t HAL_SMARTCARD_Receive_IT(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint16_t size_byte);
#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
hal_status_t HAL_SMARTCARD_Transmit_IT_Opt(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data,
                                           uint16_t size_byte, uint32_t interrupts);
hal_status_t HAL_SMARTCARD_Receive_IT_Opt(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                          uint16_t size_byte, uint32_t interrupts);
#endif /* USE_HAL_SMARTCARD_FIFO */
#if defined(USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
hal_status_t HAL_SMARTCARD_Transmit_DMA(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data, uint16_t size_byte);
hal_status_t HAL_SMARTCARD_Receive_DMA(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data, uint16_t size_byte);
hal_status_t HAL_SMARTCARD_Transmit_DMA_Opt(hal_smartcard_handle_t *hsmartcard, const uint8_t *p_data,
                                            uint16_t size_byte, uint32_t interrupts);
hal_status_t HAL_SMARTCARD_Receive_DMA_Opt(hal_smartcard_handle_t *hsmartcard, uint8_t *p_data,
                                           uint16_t size_byte, uint32_t interrupts);
#endif /* USE_HAL_SMARTCARD_DMA */
/* Transfer Abort functions */
hal_status_t HAL_SMARTCARD_Abort(hal_smartcard_handle_t *hsmartcard);
hal_status_t HAL_SMARTCARD_Abort_IT(hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group7 DMA Configuration functions
  * @{
  */
#if defined (USE_HAL_SMARTCARD_DMA) && (USE_HAL_SMARTCARD_DMA == 1)
hal_status_t HAL_SMARTCARD_SetTxDMA(hal_smartcard_handle_t *hsmartcard, hal_dma_handle_t *hdma_tx);
hal_status_t HAL_SMARTCARD_SetRxDMA(hal_smartcard_handle_t *hsmartcard, hal_dma_handle_t *hdma_rx);
#endif /* USE_HAL_SMARTCARD_DMA */

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group8 IRQHandler and Default Callbacks
  * @{
  */
void HAL_SMARTCARD_IRQHandler(hal_smartcard_handle_t *hsmartcard);

/* Default callback */
void HAL_SMARTCARD_TxCpltCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_TxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_RxCpltCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_RxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_ErrorCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_AbortCpltCallback(hal_smartcard_handle_t *hsmartcard);

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
void HAL_SMARTCARD_RxFifoFullCallback(hal_smartcard_handle_t *hsmartcard);
void HAL_SMARTCARD_TxFifoEmptyCallback(hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_FIFO */

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group9 Callbacks Register functions
  * @{
  */
#if (USE_HAL_SMARTCARD_REGISTER_CALLBACKS == 1)

hal_status_t HAL_SMARTCARD_RegisterTxCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterTxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard,
                                                      hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterRxCpltCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterRxHalfCpltCallback(hal_smartcard_handle_t *hsmartcard,
                                                      hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterErrorCallback(hal_smartcard_handle_t *hsmartcard, hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterAbortCpltCallback(hal_smartcard_handle_t *hsmartcard,
                                                     hal_smartcard_cb_t p_callback);

#if defined(USE_HAL_SMARTCARD_FIFO) && (USE_HAL_SMARTCARD_FIFO == 1)
hal_status_t HAL_SMARTCARD_RegisterRxFifoFullCallback(hal_smartcard_handle_t *hsmartcard,
                                                      hal_smartcard_cb_t p_callback);
hal_status_t HAL_SMARTCARD_RegisterTxFifoEmptyCallback(hal_smartcard_handle_t *hsmartcard,
                                                       hal_smartcard_cb_t p_callback);
#endif /* USE_HAL_SMARTCARD_FIFO */
#endif /* USE_HAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Peripheral State and Error functions ***************************************/
/** @defgroup SMARTCARD_Exported_Functions_Group10 State, Error and Clock Frequency functions
  * @{
  */

hal_smartcard_state_t HAL_SMARTCARD_GetState(const hal_smartcard_handle_t *hsmartcard);
#if defined(USE_HAL_SMARTCARD_GET_LAST_ERRORS) && (USE_HAL_SMARTCARD_GET_LAST_ERRORS == 1)
uint32_t              HAL_SMARTCARD_GetLastErrorCodes(const hal_smartcard_handle_t *hsmartcard);
#endif /* USE_HAL_SMARTCARD_GET_LAST_ERRORS */
uint32_t              HAL_SMARTCARD_GetClockFreq(const hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)

/** @defgroup SMARTCARD_Exported_Functions_Group11 Acquire/Release Bus functions
  * @{
  */

hal_status_t HAL_SMARTCARD_AcquireBus(hal_smartcard_handle_t *hsmartcard, uint32_t timeout_ms);
hal_status_t HAL_SMARTCARD_ReleaseBus(hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */

#endif /*USE_HAL_MUTEX */

#if defined(USE_HAL_SMARTCARD_USER_DATA) && (USE_HAL_SMARTCARD_USER_DATA == 1)
/** @defgroup SMARTCARD_Exported_Functions_Group12 UserData functions
  * @{
  */

void HAL_SMARTCARD_SetUserData(hal_smartcard_handle_t *hsmartcard, const void *p_user_data);
const void *HAL_SMARTCARD_GetUserData(const hal_smartcard_handle_t *hsmartcard);

/**
  * @}
  */

#endif /* USE_HAL_SMARTCARD_USER_DATA */

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
#endif /* __cplusplus */
#endif /* STM32C5XX_HAL_SMARTCARD_H */
