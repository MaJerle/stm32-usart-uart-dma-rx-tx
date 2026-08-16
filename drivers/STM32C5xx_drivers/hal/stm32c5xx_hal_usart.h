/**
  ******************************************************************************
  * @file    stm32c5xx_hal_usart.h
  * @brief   Header file of USART HAL module.
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
#ifndef STM32C5XX_HAL_USART_H
#define STM32C5XX_HAL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_usart.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined(USART1) || defined(USART2) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) \
 || defined(UART7)

/** @defgroup USART USART
  * @{
  */

/** @defgroup USART_Exported_Types HAL USART Types
  * @{
  */
/**
  * @brief HAL USART Instance Definition.
  *
  */
typedef enum
{
  /*! Instance USART1 */
  HAL_USART1 = USART1_BASE,
  /*! Instance USART2 */
  HAL_USART2 = USART2_BASE,
#if defined(USART3)
  /*! Instance USART3 */
  HAL_USART3 = USART3_BASE,
#endif /* USART3 */
#if defined(USART6)
  /*! Instance USART6 */
  HAL_USART6 = USART6_BASE,
#endif /* USART6 */

} hal_usart_t;

/**
  * @brief HAL USART State Structure Definition.
  */
typedef enum
{
  /*! Peripheral is not initialized */
  HAL_USART_STATE_RESET                     = 0UL,
  /*! Peripheral is initialized but not configured */
  HAL_USART_STATE_INIT                      = (1UL << 31U),
  /*!  Peripheral is initialized and a global config is set */
  HAL_USART_STATE_IDLE                      = (1UL << 30U),
  /*! Peripheral Reception process is ongoing */
  HAL_USART_STATE_RX_ACTIVE                 = (1UL << 29U),
  /*! Peripheral Transmit process is ongoing */
  HAL_USART_STATE_TX_ACTIVE                 = (1UL << 28U),
  /*! Peripheral Transmit/Receive process is ongoing */
  HAL_USART_STATE_TX_RX_ACTIVE              = (1UL << 27U),
  /*! Peripheral process is aborting */
  HAL_USART_STATE_ABORT                     = (1UL << 26U),
} hal_usart_state_t;


/** @addtogroup USART_Basic_config USART Basic configuration Definition
  * @{
  */
/**
  * @brief HAL USART Word Length Definition.
  */
typedef enum
{
  /*! 7-bit long USART frame */
  HAL_USART_WORD_LENGTH_7_BIT               = LL_USART_DATAWIDTH_7_BIT,
  /*! 8-bit long USART frame */
  HAL_USART_WORD_LENGTH_8_BIT               = LL_USART_DATAWIDTH_8_BIT,
  /*! 9-bit long USART frame */
  HAL_USART_WORD_LENGTH_9_BIT               = LL_USART_DATAWIDTH_9_BIT,
} hal_usart_word_length_t;

/**
  * @brief HAL USART Stop Bits Definition.
  */
typedef enum
{
  /*! USART frame with 0.5 stop bit */
  HAL_USART_STOP_BIT_0_5                    = LL_USART_STOP_BIT_0_5,
  /*! USART frame with 1 stop bit */
  HAL_USART_STOP_BIT_1                      = LL_USART_STOP_BIT_1,
  /*! USART frame with 1.5 stop bits */
  HAL_USART_STOP_BIT_1_5                    = LL_USART_STOP_BIT_1_5,
  /*! USART frame with 2 stop bits */
  HAL_USART_STOP_BIT_2                      = LL_USART_STOP_BIT_2,
} hal_usart_stop_bits_t;

/**
  * @brief HAL USART Parity Definition.
  * @note When parity is enabled, the computed parity is inserted
          at the MSB position of the transmitted data (9th bit when
          the word length is set to 9 data bits; 8th bit when the
          word length is set to 8 data bits).
  */
typedef enum
{
  /*! No parity */
  HAL_USART_PARITY_NONE                     = LL_USART_PARITY_NONE,
  /*! Even parity */
  HAL_USART_PARITY_EVEN                     = LL_USART_PARITY_EVEN,
  /*! Odd parity */
  HAL_USART_PARITY_ODD                      = LL_USART_PARITY_ODD,
} hal_usart_parity_t;

/**
  * @brief HAL USART Direction Definition.
  */
typedef enum
{
  /*! RX mode */
  HAL_USART_DIRECTION_RX                    = LL_USART_DIRECTION_RX,
  /*! TX mode */
  HAL_USART_DIRECTION_TX                    = LL_USART_DIRECTION_TX,
  /*! RX and TX mode */
  HAL_USART_DIRECTION_TX_RX                 = LL_USART_DIRECTION_TX_RX,
} hal_usart_direction_t;

/**
  * @brief HAL USART Clock Prescaler Definition.
  */
typedef enum
{
  /*! fclk_pres = fclk */
  HAL_USART_PRESCALER_DIV1                  = LL_USART_PRESCALER_DIV1,
  /*! fclk_pres = fclk/2 */
  HAL_USART_PRESCALER_DIV2                  = LL_USART_PRESCALER_DIV2,
  /*! fclk_pres = fclk/4 */
  HAL_USART_PRESCALER_DIV4                  = LL_USART_PRESCALER_DIV4,
  /*! fclk_pres = fclk/6 */
  HAL_USART_PRESCALER_DIV6                  = LL_USART_PRESCALER_DIV6,
  /*! fclk_pres = fclk/8 */
  HAL_USART_PRESCALER_DIV8                  = LL_USART_PRESCALER_DIV8,
  /*! fclk_pres = fclk/10 */
  HAL_USART_PRESCALER_DIV10                 = LL_USART_PRESCALER_DIV10,
  /*! fclk_pres = fclk/12 */
  HAL_USART_PRESCALER_DIV12                 = LL_USART_PRESCALER_DIV12,
  /*! fclk_pres = fclk/16 */
  HAL_USART_PRESCALER_DIV16                 = LL_USART_PRESCALER_DIV16,
  /*! fclk_pres = fclk/32 */
  HAL_USART_PRESCALER_DIV32                 = LL_USART_PRESCALER_DIV32,
  /*! fclk_pres = fclk/64 */
  HAL_USART_PRESCALER_DIV64                 = LL_USART_PRESCALER_DIV64,
  /*! fclk_pres = fclk/128 */
  HAL_USART_PRESCALER_DIV128                = LL_USART_PRESCALER_DIV128,
  /*! fclk_pres = fclk/256 */
  HAL_USART_PRESCALER_DIV256                = LL_USART_PRESCALER_DIV256,
} hal_usart_prescaler_t;


/**
  *  @brief HAL USART Clock Polarity Definition
  */
typedef enum
{
  /*! Steady low value on SCLK pin outside transmission window */
  HAL_USART_CLOCK_POLARITY_LOW              = LL_USART_CLOCK_POLARITY_LOW,
  /*! Steady high value on SCLK pin outside transmission window */
  HAL_USART_CLOCK_POLARITY_HIGH             = LL_USART_CLOCK_POLARITY_HIGH,
} hal_usart_clock_polarity_t;

/**
  *  @brief HAL USART Clock Phase Definition
  */
typedef enum
{
  /*! USART frame phase on first clock transition  */
  HAL_USART_CLOCK_PHASE_1_EDGE              = LL_USART_CLOCK_PHASE_1_EDGE,
  /*! USART frame phase on second clock transition */
  HAL_USART_CLOCK_PHASE_2_EDGE              = LL_USART_CLOCK_PHASE_2_EDGE,
} hal_usart_clock_phase_t;

/**
  *  @brief HAL USART Clock Last Bit Status Definition
  */
typedef enum
{
  /*! USART frame last data bit clock pulse not output to SCLK pin */
  HAL_USART_CLOCK_LAST_BIT_DISABLED         = LL_USART_LASTCLKPULSE_DISABLED,
  /*! USART frame last data bit clock pulse output to SCLK pin     */
  HAL_USART_CLOCK_LAST_BIT_ENABLED          = LL_USART_LASTCLKPULSE_ENABLED,
} hal_usart_clock_last_bit_state_t;

/**
  * @}
  */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/** @defgroup USART_FIFO_Mode USART FIFO Mode Definition
  * @{
  */

/**
  * @brief HAL USART FIFO Threshold Definition.
  */
typedef enum
{
  /*! FIFO reaches 1/8 of its depth */
  HAL_USART_FIFO_THRESHOLD_1_8              = LL_USART_FIFO_THRESHOLD_1_8,
  /*! FIFO reaches 1/4 of its depth */
  HAL_USART_FIFO_THRESHOLD_1_4              = LL_USART_FIFO_THRESHOLD_1_4,
  /*! FIFO reaches 1/2 of its depth */
  HAL_USART_FIFO_THRESHOLD_1_2              = LL_USART_FIFO_THRESHOLD_1_2,
  /*! FIFO reaches 3/4 of its depth */
  HAL_USART_FIFO_THRESHOLD_3_4              = LL_USART_FIFO_THRESHOLD_3_4,
  /*! FIFO reaches 7/8 of its depth */
  HAL_USART_FIFO_THRESHOLD_7_8              = LL_USART_FIFO_THRESHOLD_7_8,
  /*! FIFO reaches 8/8 of its depth */
  HAL_USART_FIFO_THRESHOLD_8_8              = LL_USART_FIFO_THRESHOLD_8_8,
} hal_usart_fifo_threshold_t;

/**
  * @brief HAL USART FIFO Mode Status Definition.
  */
typedef enum
{
  /*! USART FIFO Mode is disabled */
  HAL_USART_FIFO_MODE_DISABLED              = 0U,
  /*! USART FIFO Mode is enabled */
  HAL_USART_FIFO_MODE_ENABLED               = 1U,
} hal_usart_fifo_mode_status_t;
/**
  * @}
  */
#endif /* USE_HAL_USART_FIFO */

/**
  * @brief HAL USART Request Definition.
  */
typedef enum
{
  /*! Receive Data flush Request */
  HAL_USART_REQUEST_RX_DATA_FLUSH           = LL_USART_REQUEST_RX_DATA_FLUSH,
  /*! Transmit data flush Request */
  HAL_USART_REQUEST_TX_DATA_FLUSH           = LL_USART_REQUEST_TX_DATA_FLUSH,
} hal_usart_request_t;

/**
  * @brief HAL USART Master/Slave Mode Definition.
  */
typedef enum
{
  /*! USART Master mode is configured */
  HAL_USART_MODE_MASTER    = 0U,
  /*! USART Slave mode is configured */
  HAL_USART_MODE_SLAVE     = 1U,
} hal_usart_mode_t;

/**
  * @brief HAL USART Slave Select Configuration Definition.
  */
typedef enum
{
  /*! USART NSS PIN is ignored to select the slave */
  HAL_USART_SLAVE_SELECT_PIN_IGNORED = LL_USART_NSS_IGNORED,
  /*! USART NSS PIN is used to select the slave */
  HAL_USART_SLAVE_SELECT_PIN_USED    = LL_USART_NSS_USED,
} hal_usart_slave_select_config_t;

/*! HAL USART handler type */
typedef struct hal_usart_handle_s hal_usart_handle_t;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
/*! HAL USART Generic USART callback Type */
typedef void (* hal_usart_cb_t)(hal_usart_handle_t *husart);
#endif  /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @brief HAL USART Handle Structure Type.
  */
struct hal_usart_handle_s
{
  /*! Peripheral instance        */
  hal_usart_t instance;

  /*! Pointer to USART Tx transfer Buffer */
  const uint8_t *p_tx_buff;

  /*! USART Tx Transfer size              */
  volatile uint32_t tx_xfer_size;

  /*! USART Tx Transfer Counter           */
  volatile uint32_t tx_xfer_count;

  /*! Pointer to USART Rx transfer Buffer */
  uint8_t *p_rx_buff;

  /*! USART Rx Transfer size              */
  volatile uint32_t rx_xfer_size;

  /*! USART Rx Transfer Counter           */
  volatile uint32_t rx_xfer_count;

  /*! USART Rx RDR register mask          */
  volatile uint16_t rdr_register_mask;

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  /*! Specifies if the FIFO mode is being used.*/
  hal_usart_fifo_mode_status_t fifo_mode;

  /*! Number of data to process during RX ISR execution */
  uint16_t nb_rx_data_to_process;

  /*! Number of data to process during TX ISR execution */
  uint16_t nb_tx_data_to_process;

#endif /* USE_HAL_USART_FIFO */
  /*! USART Master/Slave mode          */
  hal_usart_mode_t usart_mode;

  /*! Function pointer on Rx IRQ handler */
  void (*p_rx_isr)(struct hal_usart_handle_s *husart);

  /*! Function pointer on Tx IRQ handler */
  void (*p_tx_isr)(struct hal_usart_handle_s *husart);

#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
  /*! USART Tx DMA Handle parameters      */
  hal_dma_handle_t *hdma_tx;

  /*! USART Rx DMA Handle parameters      */
  hal_dma_handle_t *hdma_rx;

#endif /* USE_HAL_USART_DMA */
  /*! USART state information related to global Handle management */
  volatile hal_usart_state_t global_state;

#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  /*! USART Tx Half Complete Callback        */
  hal_usart_cb_t p_tx_half_cplt_callback;

  /*! USART Tx Complete Callback             */
  hal_usart_cb_t p_tx_cplt_callback;

  /*! USART Rx Half Complete Callback        */
  hal_usart_cb_t p_rx_half_cplt_callback;

  /*! USART Rx Complete Callback             */
  hal_usart_cb_t p_rx_cplt_callback;

  /*! USART Tx/Rx Complete Callback          */
  hal_usart_cb_t p_tx_rx_cplt_callback;

  /*! USART Error Callback                   */
  hal_usart_cb_t p_error_callback;

  /*! USART Abort Complete Callback          */
  hal_usart_cb_t p_abort_cplt_callback;

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
  /*! USART Rx FIFO Full Callback            */
  hal_usart_cb_t p_rx_fifo_full_callback;

  /*! USART Tx FIFO Empty Callback           */
  hal_usart_cb_t p_tx_fifo_empty_callback;

#endif /* USE_HAL_USART_FIFO */

#endif  /* USE_HAL_USART_REGISTER_CALLBACKS */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /*! USART OS semaphore */
  hal_os_semaphore_t semaphore;

#endif /* USE_HAL_MUTEX  */

#if defined (USE_HAL_USART_USER_DATA) && (USE_HAL_USART_USER_DATA == 1)
  /*! User data pointer */
  const void *p_user_data;

#endif /* USE_HAL_USART_USER_DATA */

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
  /*! Last error codes on reception side */
  volatile uint32_t last_error_codes;

#endif /* USE_HAL_USART_GET_LAST_ERRORS */
};


/** @addtogroup USART_Basic_config
  * @{
  */
/**
  * @brief HAL USART Global Config Structure Type.
  */
typedef struct
{
  /*! This member configures the USART communication baud rate.
      The baud rate register is computed using the following formula:
              Baud Rate Register[15:4] = ((2 * hal_usart_config_t->clock_prescaler) /
              ((hal_usart_config_t->baud_rate)))[15:4]
              Baud Rate Register[3] =  0
              Baud Rate Register[2:0] =  (((2 * hal_usart_config_t->clock_prescaler) /
              ((hal_usart_config_t->baud_rate)))[3:0]) >> 1
            where usart_ker_ck_pres is the USART input clock divided by a prescaler */
  uint32_t baud_rate;
  /*! Specifies the prescaler value used to divide the USART clock source. */
  hal_usart_prescaler_t clock_prescaler;

  /*! Specifies the number of data bits transmitted or received in a frame.*/
  hal_usart_word_length_t word_length;

  /*! Specifies the number of stop bits transmitted.*/
  hal_usart_stop_bits_t stop_bits;

  /*! Specifies the parity mode. */
  hal_usart_parity_t parity;

  /*! Specifies the process direction, receive and/or transmit. */
  hal_usart_direction_t direction;

  /*! Specifies the clock polarity. */
  hal_usart_clock_polarity_t clock_polarity;

  /*! Specifies the clock phase. */
  hal_usart_clock_phase_t clock_phase;

  /*! Specifies the clock last bit enabling. */
  hal_usart_clock_last_bit_state_t clock_last_bit;

  /*! Specifies the Mode (Master or Slave), by default configured in Master Mode. */
  hal_usart_mode_t mode;
} hal_usart_config_t;
/**
  * @}
  */
/**
  * @}
  */

/* Exported Constants --------------------------------------------------------*/
/** @defgroup USART_Exported_Constants HAL USART Constants
  * @{
  */
/** @defgroup USART_Error_Codes USART Error Codes
  * @{
  */
/*! No error */
#define HAL_USART_ERROR_NONE     (0UL)

/*! Parity error on RX */
#define HAL_USART_RECEIVE_ERROR_PE       (0x1UL << 0)

/*! Noise error on RX */
#define HAL_USART_RECEIVE_ERROR_NE       (0x1UL << 1U)

/*! Frame error on RX */
#define HAL_USART_RECEIVE_ERROR_FE       (0x1UL << 2U)

/*! Overrun error on RX */
#define HAL_USART_RECEIVE_ERROR_ORE      (0x1UL << 3U)

#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1U)
/*! DMA transfer error on RX */
#define HAL_USART_RECEIVE_ERROR_DMA      (0x1UL << 4U)
#endif /* USE_HAL_USART_DMA */

/*! Receiver Timeout error on RX */
#define HAL_USART_RECEIVE_ERROR_RTO      (0x1UL << 5U)

#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1U)
/*! DMA transfer error on TX */
#define HAL_USART_TRANSMIT_ERROR_DMA     (0x1UL << 16U)
#endif /* USE_HAL_USART_DMA */

/*! Under Run error on Tx */
#define HAL_USART_TRANSMIT_ERROR_UDR     (0x1UL << 17U)
/**
  * @}
  */


/** @defgroup USART_Optional_Interrupts USART Optional Interrupts
  * @{
  */

/** @defgroup USART_Transmit_IT_Optional_Interrupts USART Optional Interrupts for Transmit IT process
  * @{
  */
/*! No Optional IT for Transmit IT based process */
#define HAL_USART_OPT_TX_IT_NONE             0U

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/*! Activate FIFO_EMPTY Optional IT for Transmit IT based process */
#define HAL_USART_OPT_TX_IT_FIFO_EMPTY       (1UL << 31U)

/*! Activate default Optional IT for Transmit IT based process */
#define HAL_USART_OPT_TX_IT_DEFAULT          (HAL_USART_OPT_TX_IT_FIFO_EMPTY)
#endif /* USE_HAL_USART_FIFO */
/**
  * @}
  */
#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1U)

/** @defgroup USART_Transmit_DMA_Optional_Interrupts USART Optional Interrupts for Transmit DMA process
  * @{
  */
/*! No Optional IT for Transmit DMA based process */
#define HAL_USART_OPT_DMA_TX_IT_NONE         0U

/*! Activate HALF_TRANSFER Optional IT for Transmit DMA based process */
#define HAL_USART_OPT_DMA_TX_IT_HT           (HAL_DMA_OPT_IT_HT)

/*! Activate default Optional IT for Transmit DMA based process */
#define HAL_USART_OPT_DMA_TX_IT_DEFAULT      (HAL_USART_OPT_DMA_TX_IT_HT)

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Activate DMA SILENT MODE for Transmit DMA based process */
#define HAL_USART_OPT_DMA_TX_IT_SILENT       (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_USART_DMA */

/** @defgroup USART_Receive_IT_Optional_Interrupts USART Optional Interrupts for Receive IT process
  * @{
  */
/*! No Optional IT for Receive IT based process */
#define HAL_USART_OPT_RX_IT_NONE             0U

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/*! Activate FIFO_FULL Optional IT for Receive IT based process */
#define HAL_USART_OPT_RX_IT_FIFO_FULL        (1UL << 29U)

/*! Activate default Optional IT for Receive IT based process */
#define HAL_USART_OPT_RX_IT_DEFAULT          (HAL_USART_OPT_RX_IT_FIFO_FULL)
#endif /* USE_HAL_USART_FIFO */
/**
  * @}
  */

#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1U)

/** @defgroup USART_Receive_DMA_Optional_Interrupts USART Optional Interrupts for Receive DMA process
  * @{
  */
/*! No Optional IT for Receive DMA based process */
#define HAL_USART_OPT_DMA_RX_IT_NONE         0U

/*! Activate HALF_TRANSFER Optional IT for Receive DMA based process */
#define HAL_USART_OPT_DMA_RX_IT_HT           (HAL_DMA_OPT_IT_HT)

/*! Activate default Optional IT for Receive DMA based process */
#define HAL_USART_OPT_DMA_RX_IT_DEFAULT      (HAL_USART_OPT_DMA_RX_IT_HT)

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Activate DMA SILENT MODE for Receive DMA based process */
#define HAL_USART_OPT_DMA_RX_IT_SILENT       (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_USART_DMA */


/** @defgroup USART_TransmitReceive_IT_Optional_Interrupts USART Optional Interrupts for TransmitReceive IT process
  * @{
  */
/*! No Optional IT for TransmitReceive IT based process */
#define HAL_USART_OPT_TXRX_IT_NONE           0U

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/*! Activate FIFO_EMPTY Optional IT for TransmitReceive IT based process */
#define HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY  (1UL << 27U)

/*! Activate FIFO_FULL Optional IT for TransmitReceive IT based process */
#define HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL   (1UL << 26U)

/*! Activate default Optional IT for TransmitReceive IT based process */
#define HAL_USART_OPT_TXRX_IT_DEFAULT        (HAL_USART_OPT_TXRX_TX_IT_FIFO_EMPTY | HAL_USART_OPT_TXRX_RX_IT_FIFO_FULL)
#endif /* USE_HAL_USART_FIFO */
/**
  * @}
  */


#if defined(USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1U)

/** @defgroup USART_TransmitReceive_DMA_Optional_Interrupts USART Optional Interrupts for TransmitReceive DMA process
  * @{
  */
/*! No Optional IT for TransmitReceive DMA based process */
#define HAL_USART_OPT_DMA_TXRX_IT_NONE       0U

/*! Activate HALF_TRANSFER on TX Optional IT for TransmitReceive DMA based process */
#define HAL_USART_OPT_DMA_TXRX_TX_IT_HT      (HAL_DMA_OPT_IT_HT)

/*! Activate HALF_TRANSFER on RX Optional IT for TransmitReceive DMA based process */
#define HAL_USART_OPT_DMA_TXRX_RX_IT_HT      (HAL_DMA_OPT_IT_HT)

/*! Activate default Optional IT for TransmitReceive DMA based process */
#define HAL_USART_OPT_DMA_TXRX_IT_DEFAULT    (HAL_USART_OPT_DMA_TXRX_TX_IT_HT | HAL_USART_OPT_DMA_TXRX_RX_IT_HT)

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Activate DMA SILENT MODE on for TransmitReceive DMA based process */
#define HAL_USART_OPT_DMA_TXRX_IT_SILENT     (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */
#endif /* USE_HAL_USART_DMA */


/**
  * @}
  */

/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @defgroup USART_Exported_Functions HAL USART Functions
  * @{
  */

/** @defgroup USART_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
hal_status_t HAL_USART_Init(hal_usart_handle_t *husart, hal_usart_t instance);
void HAL_USART_DeInit(hal_usart_handle_t *husart);
/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group2 Basic configuration functions
  * @{
  */
hal_status_t HAL_USART_SetConfig(hal_usart_handle_t *husart, const hal_usart_config_t *p_config);
void HAL_USART_GetConfig(const hal_usart_handle_t *husart, hal_usart_config_t *p_config);

hal_status_t HAL_USART_SetWordLength(const hal_usart_handle_t *husart, hal_usart_word_length_t word_length);
hal_usart_word_length_t HAL_USART_GetWordLength(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetParity(const hal_usart_handle_t *husart, hal_usart_parity_t parity);
hal_usart_parity_t HAL_USART_GetParity(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetStopBits(const hal_usart_handle_t *husart, hal_usart_stop_bits_t stop_bits);
hal_usart_stop_bits_t HAL_USART_GetStopBits(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetXferDirection(const hal_usart_handle_t *husart, hal_usart_direction_t xfer_direction);
hal_usart_direction_t HAL_USART_GetXferDirection(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetClockPolarity(const hal_usart_handle_t *husart, hal_usart_clock_polarity_t clock_polarity);
hal_usart_clock_polarity_t HAL_USART_GetClockPolarity(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetClockPhase(const hal_usart_handle_t *husart, hal_usart_clock_phase_t clock_phase);
hal_usart_clock_phase_t HAL_USART_GetClockPhase(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetLastBitClockPulse(const hal_usart_handle_t *husart,
                                            hal_usart_clock_last_bit_state_t clock_last_bit);
hal_usart_clock_last_bit_state_t HAL_USART_GetLastBitClockPulse(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetBaudRate(const hal_usart_handle_t *husart, uint32_t baud_rate);
uint32_t HAL_USART_GetBaudRate(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetMode(hal_usart_handle_t *husart, hal_usart_mode_t mode);
hal_usart_mode_t HAL_USART_GetMode(const hal_usart_handle_t *husart);
/**
  * @}
  */

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
/** @defgroup USART_Exported_Functions_Group3 FIFO Configuration functions
  * @{
  */
hal_status_t HAL_USART_EnableFifoMode(hal_usart_handle_t *husart);
hal_status_t HAL_USART_DisableFifoMode(hal_usart_handle_t *husart);
hal_usart_fifo_mode_status_t HAL_USART_IsEnabledFifoMode(const hal_usart_handle_t *husart);

hal_status_t HAL_USART_SetTxFifoThreshold(hal_usart_handle_t *husart, hal_usart_fifo_threshold_t tx_fifo_threshold);
hal_usart_fifo_threshold_t HAL_USART_GetTxFifoThreshold(const hal_usart_handle_t *husart);
hal_status_t HAL_USART_SetRxFifoThreshold(hal_usart_handle_t *husart, hal_usart_fifo_threshold_t rx_fifo_threshold);
hal_usart_fifo_threshold_t HAL_USART_GetRxFifoThreshold(const hal_usart_handle_t *husart);
/**
  * @}
  */
#endif /* USE_HAL_USART_FIFO */


/** @defgroup USART_Exported_Functions_Group5 Advanced configuration functions
  * @{
  */

hal_status_t HAL_USART_SetSlaveSelect(const hal_usart_handle_t *husart, hal_usart_slave_select_config_t slave_select);
hal_usart_slave_select_config_t HAL_USART_GetSlaveSelect(const hal_usart_handle_t *husart);

/**
  * @}
  */
/** @defgroup USART_Exported_Functions_Group6 DMA Configuration functions
  * @{
  */
#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
hal_status_t HAL_USART_SetTxDMA(hal_usart_handle_t *husart, hal_dma_handle_t *hdma_tx);
hal_status_t HAL_USART_SetRxDMA(hal_usart_handle_t *husart, hal_dma_handle_t *hdma_rx);
#endif /* USE_HAL_USART_DMA */

/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group7 Callbacks Register functions
  * @{
  */
#if defined(USE_HAL_USART_REGISTER_CALLBACKS) && (USE_HAL_USART_REGISTER_CALLBACKS == 1)

hal_status_t HAL_USART_RegisterTxHalfCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterTxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterRxHalfCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterRxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterTxRxCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterErrorCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterAbortCpltCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
hal_status_t HAL_USART_RegisterRxFifoFullCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
hal_status_t HAL_USART_RegisterTxFifoEmptyCallback(hal_usart_handle_t *husart, hal_usart_cb_t p_callback);
#endif /* USE_HAL_USART_FIFO */
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group8 IO operation functions
  * @{
  */

hal_status_t HAL_USART_Transmit(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                uint32_t timeout_ms);
hal_status_t HAL_USART_Receive(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_USART_TransmitReceive(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                       uint32_t size_byte, uint32_t timeout_ms);

hal_status_t HAL_USART_Transmit_IT(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte);
hal_status_t HAL_USART_Receive_IT(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte);
hal_status_t HAL_USART_TransmitReceive_IT(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                          uint32_t size_byte);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
hal_status_t HAL_USART_Transmit_IT_Opt(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                       uint32_t interrupts);
hal_status_t HAL_USART_Receive_IT_Opt(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte,
                                      uint32_t interrupts);
hal_status_t HAL_USART_TransmitReceive_IT_Opt(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                              uint32_t size_byte, uint32_t interrupts);
#endif /* USE_HAL_USART_FIFO */
#if defined (USE_HAL_USART_DMA) && (USE_HAL_USART_DMA == 1)
hal_status_t HAL_USART_Transmit_DMA(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte);
hal_status_t HAL_USART_Receive_DMA(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte);
hal_status_t HAL_USART_TransmitReceive_DMA(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                           uint32_t size_byte);

hal_status_t HAL_USART_Transmit_DMA_Opt(hal_usart_handle_t *husart, const void *p_data, uint32_t size_byte,
                                        uint32_t interrupts);
hal_status_t HAL_USART_Receive_DMA_Opt(hal_usart_handle_t *husart, void *p_data, uint32_t size_byte,
                                       uint32_t interrupts);
hal_status_t HAL_USART_TransmitReceive_DMA_Opt(hal_usart_handle_t *husart, const void *p_tx_data, void *p_rx_data,
                                               uint32_t size_byte, uint32_t interrupts);

hal_status_t HAL_USART_Pause_DMA(hal_usart_handle_t *husart);
hal_status_t HAL_USART_Resume_DMA(hal_usart_handle_t *husart);

#endif /* USE_HAL_USART_DMA */

/* Transfer Abort functions */
hal_status_t HAL_USART_Abort(hal_usart_handle_t *husart);
hal_status_t HAL_USART_Abort_IT(hal_usart_handle_t *husart);

hal_status_t HAL_USART_SendRequest(hal_usart_handle_t *husart, hal_usart_request_t request);
/**
  * @}
  */


/** @defgroup USART_Exported_Functions_Group9 Peripheral State and Error functions
  * @{
  */

/* Peripheral State and Errors functions  **************************************************/
hal_usart_state_t HAL_USART_GetState(const hal_usart_handle_t *husart);
uint32_t HAL_USART_GetClockFreq(const hal_usart_handle_t *husart);

#if defined (USE_HAL_USART_GET_LAST_ERRORS) && (USE_HAL_USART_GET_LAST_ERRORS == 1)
uint32_t HAL_USART_GetLastErrorCodes(const hal_usart_handle_t *husart);
#endif /* USE_HAL_USART_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup USART_Exported_Functions_Group10 Bus Operation Function
  * @{
  */
hal_status_t HAL_USART_AcquireBus(hal_usart_handle_t *husart, uint32_t timeout_ms);
hal_status_t HAL_USART_ReleaseBus(hal_usart_handle_t *husart);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_USART_USER_DATA) && (USE_HAL_USART_USER_DATA == 1)
/** @defgroup USART_Exported_Functions_Group11 User Data Functions
  * @{
  */
void HAL_USART_SetUserData(hal_usart_handle_t *husart, const void *p_user_data);
const void *HAL_USART_GetUserData(const hal_usart_handle_t *husart);
/**
  * @}
  */
#endif /* USE_HAL_USART_USER_DATA */

/** @defgroup USART_Exported_Functions_Group12 IRQ handling
  * @{
  */
void HAL_USART_IRQHandler(hal_usart_handle_t *husart);
/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group13 Default Callbacks
  * @{
  */

/* Default callback */
void HAL_USART_TxHalfCpltCallback(hal_usart_handle_t *husart);
void HAL_USART_TxCpltCallback(hal_usart_handle_t *husart);
void HAL_USART_RxHalfCpltCallback(hal_usart_handle_t *husart);
void HAL_USART_RxCpltCallback(hal_usart_handle_t *husart);
void HAL_USART_TxRxCpltCallback(hal_usart_handle_t *husart);

void HAL_USART_ErrorCallback(hal_usart_handle_t *husart);
void HAL_USART_AbortCpltCallback(hal_usart_handle_t *husart);

#if defined(USE_HAL_USART_FIFO) && (USE_HAL_USART_FIFO == 1)
void HAL_USART_RxFifoFullCallback(hal_usart_handle_t *husart);
void HAL_USART_TxFifoEmptyCallback(hal_usart_handle_t *husart);
#endif /* USE_HAL_USART_FIFO */

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

#endif /* STM32C5XX_HAL_USART_H */

