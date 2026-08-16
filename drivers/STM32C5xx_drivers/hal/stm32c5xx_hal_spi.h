/**
  ******************************************************************************
  * @file    stm32c5xx_hal_spi.h
  * @brief   Header file of SPI HAL module.
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
#ifndef STM32C5XX_HAL_SPI_H
#define STM32C5XX_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_spi.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(SPI1) || defined(SPI2) || defined(SPI3)

/** @defgroup SPI SPI
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief HAL SPI instances enumeration definition.
  */
typedef enum
{
#if defined(SPI1)
  HAL_SPI1 = SPI1_BASE,      /*!< SPI1 instance  */
#endif /* SPI1 */
#if defined(SPI2)
  HAL_SPI2 = SPI2_BASE,      /*!< SPI2 instance  */
#endif /* SPI2 */
#if defined(SPI3)
  HAL_SPI3 = SPI3_BASE,      /*!< SPI3 instance  */
#endif /* SPI3 */
} hal_spi_t;

/**
  * @brief HAL SPI direction mode enumeration definition.
  */
typedef enum
{
  HAL_SPI_DIRECTION_FULL_DUPLEX = LL_SPI_FULL_DUPLEX,      /*!< Full-duplex communication                  */
  HAL_SPI_DIRECTION_SIMPLEX_TX  = LL_SPI_SIMPLEX_TX,       /*!< Simplex communication mode: Transmit only  */
  HAL_SPI_DIRECTION_SIMPLEX_RX  = LL_SPI_SIMPLEX_RX,       /*!< Simplex communication mode: Receive only   */
  HAL_SPI_DIRECTION_HALF_DUPLEX = LL_SPI_HALF_DUPLEX,      /*!< Half-duplex communication                  */
} hal_spi_direction_t;

/**
  * @brief HAL SPI State enumeration definition.
  */
typedef enum
{
  HAL_SPI_STATE_RESET         = 0U,           /*!< 0x00000000 : SPI is not yet initialized or de-initialized        */
  HAL_SPI_STATE_INIT          = (1U << 25U),  /*!< 0x02000000 : SPI is initialized but not yet configured           */
  HAL_SPI_STATE_IDLE          = (1U << 26U),  /*!< 0x04000000 : SPI is initialized and global config applied        */
  HAL_SPI_STATE_TX_ACTIVE     = (1U << 27U),  /*!< 0x08000000 : Data Transmission process is ongoing                */
  HAL_SPI_STATE_RX_ACTIVE     = (1U << 28U),  /*!< 0x10000000 : Data Reception process is ongoing                   */
  HAL_SPI_STATE_TX_RX_ACTIVE  = (1U << 29U),  /*!< 0x20000000 : Data Transmission and Reception process is ongoing  */
  HAL_SPI_STATE_ABORT         = (1U << 30U),  /*!< 0x40000000 : SPI abort is ongoing                                */
  HAL_SPI_STATE_FAULT         = (1U << 31U),  /*!< 0x80000000 : SPI encountered an unrecoverable error and a recovery
                                                  sequence is needed*/
} hal_spi_state_t;


/*! HAL SPI handler type */
typedef struct hal_spi_handle_s hal_spi_handle_t;

/**
  * @brief HAL SPI handle structure definition.
  */
struct hal_spi_handle_s
{
  hal_spi_t                  instance;                                    /*!< SPI instance                       */
  hal_spi_direction_t        direction;                                   /*!< SPI direction                      */
  volatile hal_spi_state_t   global_state;                                /*!< SPI state                          */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
  volatile uint32_t          last_error_codes;                            /*!< SPI Error code                     */
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t         semaphore;                                   /*!< SPI OS semaphore                   */
#endif /* USE_HAL_MUTEX */
  const uint8_t              *p_tx_buff;                                  /*!< Pointer to SPI Tx transfer Buffer  */
  uint16_t                   tx_xfer_size;                                /*!< SPI Tx Transfer size               */
  volatile uint16_t          tx_xfer_count;                               /*!< SPI Tx Transfer Counter            */
  uint8_t                    *p_rx_buff;                                  /*!< Pointer to SPI Rx transfer Buffer  */
  uint16_t                   rx_xfer_size;                                /*!< SPI Rx Transfer size               */
  volatile uint16_t          rx_xfer_count;                               /*!< SPI Rx Transfer Counter            */
  void (*p_rx_isr)(struct hal_spi_handle_s *hspi);                        /*!< Function pointer on Rx ISR         */
  void (*p_tx_isr)(struct hal_spi_handle_s *hspi);                        /*!< Function pointer on Tx ISR         */
#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
  hal_dma_handle_t           *hdma_tx;                                    /*!< SPI Tx DMA Handle parameters       */
  hal_dma_handle_t           *hdma_rx;                                    /*!< SPI Rx DMA Handle parameters       */
#endif /* USE_HAL_SPI_DMA */
#if defined(USE_HAL_SPI_USER_DATA) && (USE_HAL_SPI_USER_DATA == 1)
  const void                 *p_user_data;                                /*!< User Data Pointer                  */
#endif /* USE_HAL_SPI_USER_DATA */
#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
  void (* p_tx_cplt_cb)(struct hal_spi_handle_s *hspi);                   /*!< SPI Tx Completed callback          */
  void (* p_rx_cplt_cb)(struct hal_spi_handle_s *hspi);                   /*!< SPI Rx Completed callback          */
  void (* p_tx_rx_cplt_cb)(struct hal_spi_handle_s *hspi);                /*!< SPI TxRx Completed callback        */
  void (* p_tx_half_cplt_cb)(struct hal_spi_handle_s *hspi);              /*!< SPI Tx Half Completed callback     */
  void (* p_rx_half_cplt_cb)(struct hal_spi_handle_s *hspi);              /*!< SPI Rx Half Completed callback     */
  void (* p_tx_rx_half_cplt_cb)(struct hal_spi_handle_s *hspi);           /*!< SPI TxRx Half Completed callback   */
  void (* p_error_cb)(struct hal_spi_handle_s *hspi);                     /*!< SPI Error callback                 */
  void (* p_abort_cplt_cb)(struct hal_spi_handle_s *hspi);                /*!< SPI Abort callback                 */
  void (* p_suspend_cb)(struct hal_spi_handle_s *hspi);                   /*!< SPI Suspend callback               */
#endif  /* USE_HAL_SPI_REGISTER_CALLBACKS */
};

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
typedef  void (*hal_spi_cb_t)(hal_spi_handle_t *hspi);                    /*!< HAL SPI Callback pointer           */
#endif  /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @brief HAL SPI mode enumeration definition.
  */
typedef enum
{
  HAL_SPI_MODE_SLAVE  = LL_SPI_MODE_SLAVE,          /*!< Slave mode */
  HAL_SPI_MODE_MASTER = LL_SPI_MODE_MASTER,         /*!< Master mode */
} hal_spi_mode_t;

/**
  * @brief HAL SPI data width enumeration definition.
  */
typedef enum
{
  HAL_SPI_DATA_WIDTH_4_BIT  = LL_SPI_DATA_WIDTH_4_BIT,    /*!< 4-bits            */
  HAL_SPI_DATA_WIDTH_5_BIT  = LL_SPI_DATA_WIDTH_5_BIT,    /*!< 5-bits            */
  HAL_SPI_DATA_WIDTH_6_BIT  = LL_SPI_DATA_WIDTH_6_BIT,    /*!< 6-bits            */
  HAL_SPI_DATA_WIDTH_7_BIT  = LL_SPI_DATA_WIDTH_7_BIT,    /*!< 7-bits            */
  HAL_SPI_DATA_WIDTH_8_BIT  = LL_SPI_DATA_WIDTH_8_BIT,    /*!< 8-bits            */
  HAL_SPI_DATA_WIDTH_9_BIT  = LL_SPI_DATA_WIDTH_9_BIT,    /*!< 9-bits            */
  HAL_SPI_DATA_WIDTH_10_BIT = LL_SPI_DATA_WIDTH_10_BIT,   /*!< 10-bits           */
  HAL_SPI_DATA_WIDTH_11_BIT = LL_SPI_DATA_WIDTH_11_BIT,   /*!< 11-bits           */
  HAL_SPI_DATA_WIDTH_12_BIT = LL_SPI_DATA_WIDTH_12_BIT,   /*!< 12-bits           */
  HAL_SPI_DATA_WIDTH_13_BIT = LL_SPI_DATA_WIDTH_13_BIT,   /*!< 13-bits           */
  HAL_SPI_DATA_WIDTH_14_BIT = LL_SPI_DATA_WIDTH_14_BIT,   /*!< 14-bits           */
  HAL_SPI_DATA_WIDTH_15_BIT = LL_SPI_DATA_WIDTH_15_BIT,   /*!< 15-bits           */
  HAL_SPI_DATA_WIDTH_16_BIT = LL_SPI_DATA_WIDTH_16_BIT,   /*!< 16-bits           */
  HAL_SPI_DATA_WIDTH_17_BIT = LL_SPI_DATA_WIDTH_17_BIT,   /*!< 17-bits           */
  HAL_SPI_DATA_WIDTH_18_BIT = LL_SPI_DATA_WIDTH_18_BIT,   /*!< 18-bits           */
  HAL_SPI_DATA_WIDTH_19_BIT = LL_SPI_DATA_WIDTH_19_BIT,   /*!< 19-bits           */
  HAL_SPI_DATA_WIDTH_20_BIT = LL_SPI_DATA_WIDTH_20_BIT,   /*!< 20-bits           */
  HAL_SPI_DATA_WIDTH_21_BIT = LL_SPI_DATA_WIDTH_21_BIT,   /*!< 21-bits           */
  HAL_SPI_DATA_WIDTH_22_BIT = LL_SPI_DATA_WIDTH_22_BIT,   /*!< 22-bits           */
  HAL_SPI_DATA_WIDTH_23_BIT = LL_SPI_DATA_WIDTH_23_BIT,   /*!< 23-bits           */
  HAL_SPI_DATA_WIDTH_24_BIT = LL_SPI_DATA_WIDTH_24_BIT,   /*!< 24-bits           */
  HAL_SPI_DATA_WIDTH_25_BIT = LL_SPI_DATA_WIDTH_25_BIT,   /*!< 25-bits           */
  HAL_SPI_DATA_WIDTH_26_BIT = LL_SPI_DATA_WIDTH_26_BIT,   /*!< 26-bits           */
  HAL_SPI_DATA_WIDTH_27_BIT = LL_SPI_DATA_WIDTH_27_BIT,   /*!< 27-bits           */
  HAL_SPI_DATA_WIDTH_28_BIT = LL_SPI_DATA_WIDTH_28_BIT,   /*!< 28-bits           */
  HAL_SPI_DATA_WIDTH_29_BIT = LL_SPI_DATA_WIDTH_29_BIT,   /*!< 29-bits           */
  HAL_SPI_DATA_WIDTH_30_BIT = LL_SPI_DATA_WIDTH_30_BIT,   /*!< 30-bits           */
  HAL_SPI_DATA_WIDTH_31_BIT = LL_SPI_DATA_WIDTH_31_BIT,   /*!< 31-bits           */
  HAL_SPI_DATA_WIDTH_32_BIT = LL_SPI_DATA_WIDTH_32_BIT,   /*!< 32-bits           */
} hal_spi_data_width_t;

/**
  * @brief HAL SPI clock polarity enumeration definition.
  */
typedef enum
{
  HAL_SPI_CLOCK_POLARITY_LOW  = LL_SPI_CLOCK_POLARITY_LOW,     /*!< SCK signal is at 0 when idle */
  HAL_SPI_CLOCK_POLARITY_HIGH = LL_SPI_CLOCK_POLARITY_HIGH,    /*!< SCK signal is at 1 when idle */
} hal_spi_clock_polarity_t;

/**
  * @brief HAL SPI clock phase enumeration definition.
  */
typedef enum
{
  HAL_SPI_CLOCK_PHASE_1_EDGE = LL_SPI_CLOCK_PHASE_1_EDGE,      /*!< The first clock transition is the
                                                                    first data capture edge  */
  HAL_SPI_CLOCK_PHASE_2_EDGE = LL_SPI_CLOCK_PHASE_2_EDGE,      /*!< The second clock transition is the
                                                                    first data capture edge */
} hal_spi_clock_phase_t;

/**
  * @brief HAL SPI driver baudrate prescaler enumeration definition.
  */
typedef enum
{
  HAL_SPI_BAUD_RATE_PRESCALER_2         = LL_SPI_BAUD_RATE_PRESCALER_2,          /*!< SPI master clock/2               */
  HAL_SPI_BAUD_RATE_PRESCALER_4         = LL_SPI_BAUD_RATE_PRESCALER_4,          /*!< SPI master clock/4               */
  HAL_SPI_BAUD_RATE_PRESCALER_8         = LL_SPI_BAUD_RATE_PRESCALER_8,          /*!< SPI master clock/8               */
  HAL_SPI_BAUD_RATE_PRESCALER_16        = LL_SPI_BAUD_RATE_PRESCALER_16,         /*!< SPI master clock/16              */
  HAL_SPI_BAUD_RATE_PRESCALER_32        = LL_SPI_BAUD_RATE_PRESCALER_32,         /*!< SPI master clock/32              */
  HAL_SPI_BAUD_RATE_PRESCALER_64        = LL_SPI_BAUD_RATE_PRESCALER_64,         /*!< SPI master clock/64              */
  HAL_SPI_BAUD_RATE_PRESCALER_128       = LL_SPI_BAUD_RATE_PRESCALER_128,        /*!< SPI master clock/128             */
  HAL_SPI_BAUD_RATE_PRESCALER_256       = LL_SPI_BAUD_RATE_PRESCALER_256,        /*!< SPI master clock/256             */
  HAL_SPI_BAUD_RATE_PRESCALER_BYPASS    = LL_SPI_BAUD_RATE_PRESCALER_BYPASS,     /*!< Bypass from RCC in Master mode   */
} hal_spi_baud_rate_prescaler_t;

/**
  * @brief HAL SPI MSB LSB transmission enumeration definition.
  */
typedef enum
{
  HAL_SPI_MSB_FIRST = LL_SPI_MSB_FIRST,              /*!< MSB transmitted first  */
  HAL_SPI_LSB_FIRST = LL_SPI_LSB_FIRST,              /*!< LSB transmitted first  */
} hal_spi_first_bit_t;

/**
  * @brief HAL SPI slave select management enumeration definition.
  */
typedef enum
{
  HAL_SPI_NSS_PIN_MGMT_INTERNAL   = LL_SPI_NSS_SOFT,        /*!< In this configuration the Slave select is driven
                                                                 internally. The external slave select pin is free
                                                                 for other application uses. */
  HAL_SPI_NSS_PIN_MGMT_INPUT      = LL_SPI_NSS_HARD_INPUT,  /*!< In Slave mode, the slave select pin works as a standard
                                                                 chip select input and the slave is selected while the
                                                                 slave select line is at its active level. In Master
                                                                 mode, this configuration allows multi-master
                                                                 capability. If the slave select pin is pulled into an
                                                                 active level in this mode, the SPI enters Master mode
                                                                 fault state and the SPI device is automatically
                                                                 reconfigured in Slave mode (MASTER = 0) */
  HAL_SPI_NSS_PIN_MGMT_OUTPUT     = LL_SPI_NSS_HARD_OUTPUT, /*!< This configuration is only used when the MCU is set as
                                                                 master (multi-master not allowed). The slave select pin
                                                                 active level is managed by the hardware. The
                                                                 functionality is tied to CSTART and EOT control. */
} hal_spi_nss_pin_management_t;

/**
  * @brief HAL SPI configuration structure definition.
  */
typedef struct
{
  hal_spi_mode_t                 mode;               /*!< The SPI operating mode. */
  hal_spi_direction_t            direction;          /*!< The SPI bidirectional mode state. */
  hal_spi_data_width_t           data_width;         /*!< The SPI data width.*/
  hal_spi_clock_polarity_t       clock_polarity;     /*!< The serial clock steady state.*/
  hal_spi_clock_phase_t          clock_phase;        /*!< The clock active edge for the bit capture.*/
  hal_spi_baud_rate_prescaler_t   baud_rate_prescaler; /*!< The Baud Rate prescaler value which will be used to
                                                          configure the transmit and receive SCK clock.*/
  hal_spi_first_bit_t            first_bit;          /*!< Specifies whether data transfers start from MSB or
                                                          LSB bit.*/
  hal_spi_nss_pin_management_t   nss_pin_management; /*!< SPI Slave Select Pin Management .*/

} hal_spi_config_t;

#if defined(USE_HAL_SPI_CRC) && (USE_HAL_SPI_CRC == 1)

/**
  * @brief HAL SPI CRC feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_CRC_DISABLED = 0U,  /*!<  CRC feature disabled */
  HAL_SPI_CRC_ENABLED  = 1U,  /*!<  CRC feature enabled  */
} hal_spi_crc_status_t;

/**
  * @brief HAL SPI CRC length enumeration definition.
  */
typedef enum
{
  HAL_SPI_CRC_LENGTH_DATASIZE = (0UL),                       /*!< Datasize length CRC   */
  HAL_SPI_CRC_LENGTH_4_BIT     = LL_SPI_CRC_LENGTH_4_BIT,    /*!< 4-bit length CRC      */
  HAL_SPI_CRC_LENGTH_5_BIT     = LL_SPI_CRC_LENGTH_5_BIT,    /*!< 5-bit length CRC      */
  HAL_SPI_CRC_LENGTH_6_BIT     = LL_SPI_CRC_LENGTH_6_BIT,    /*!< 6-bit length CRC      */
  HAL_SPI_CRC_LENGTH_7_BIT     = LL_SPI_CRC_LENGTH_7_BIT,    /*!< 7-bit length CRC      */
  HAL_SPI_CRC_LENGTH_8_BIT     = LL_SPI_CRC_LENGTH_8_BIT,    /*!< 8-bit length CRC      */
  HAL_SPI_CRC_LENGTH_9_BIT     = LL_SPI_CRC_LENGTH_9_BIT,    /*!< 9-bit length CRC      */
  HAL_SPI_CRC_LENGTH_10_BIT    = LL_SPI_CRC_LENGTH_10_BIT,   /*!< 10-bit length CRC     */
  HAL_SPI_CRC_LENGTH_11_BIT    = LL_SPI_CRC_LENGTH_11_BIT,   /*!< 11-bit length CRC     */
  HAL_SPI_CRC_LENGTH_12_BIT    = LL_SPI_CRC_LENGTH_12_BIT,   /*!< 12-bit length CRC     */
  HAL_SPI_CRC_LENGTH_13_BIT    = LL_SPI_CRC_LENGTH_13_BIT,   /*!< 13-bit length CRC     */
  HAL_SPI_CRC_LENGTH_14_BIT    = LL_SPI_CRC_LENGTH_14_BIT,   /*!< 14-bit length CRC     */
  HAL_SPI_CRC_LENGTH_15_BIT    = LL_SPI_CRC_LENGTH_15_BIT,   /*!< 15-bit length CRC     */
  HAL_SPI_CRC_LENGTH_16_BIT    = LL_SPI_CRC_LENGTH_16_BIT,   /*!< 16-bit length CRC     */
  HAL_SPI_CRC_LENGTH_17_BIT    = LL_SPI_CRC_LENGTH_17_BIT,   /*!< 17-bit length CRC     */
  HAL_SPI_CRC_LENGTH_18_BIT    = LL_SPI_CRC_LENGTH_18_BIT,   /*!< 18-bit length CRC     */
  HAL_SPI_CRC_LENGTH_19_BIT    = LL_SPI_CRC_LENGTH_19_BIT,   /*!< 19-bit length CRC     */
  HAL_SPI_CRC_LENGTH_20_BIT    = LL_SPI_CRC_LENGTH_20_BIT,   /*!< 20-bit length CRC     */
  HAL_SPI_CRC_LENGTH_21_BIT    = LL_SPI_CRC_LENGTH_21_BIT,   /*!< 21-bit length CRC     */
  HAL_SPI_CRC_LENGTH_22_BIT    = LL_SPI_CRC_LENGTH_22_BIT,   /*!< 22-bit length CRC     */
  HAL_SPI_CRC_LENGTH_23_BIT    = LL_SPI_CRC_LENGTH_23_BIT,   /*!< 23-bit length CRC     */
  HAL_SPI_CRC_LENGTH_24_BIT    = LL_SPI_CRC_LENGTH_24_BIT,   /*!< 24-bit length CRC     */
  HAL_SPI_CRC_LENGTH_25_BIT    = LL_SPI_CRC_LENGTH_25_BIT,   /*!< 25-bit length CRC     */
  HAL_SPI_CRC_LENGTH_26_BIT    = LL_SPI_CRC_LENGTH_26_BIT,   /*!< 26-bit length CRC     */
  HAL_SPI_CRC_LENGTH_27_BIT    = LL_SPI_CRC_LENGTH_27_BIT,   /*!< 27-bit length CRC     */
  HAL_SPI_CRC_LENGTH_28_BIT    = LL_SPI_CRC_LENGTH_28_BIT,   /*!< 28-bit length CRC     */
  HAL_SPI_CRC_LENGTH_29_BIT    = LL_SPI_CRC_LENGTH_29_BIT,   /*!< 29-bit length CRC     */
  HAL_SPI_CRC_LENGTH_30_BIT    = LL_SPI_CRC_LENGTH_30_BIT,   /*!< 30-bit length CRC     */
  HAL_SPI_CRC_LENGTH_31_BIT    = LL_SPI_CRC_LENGTH_31_BIT,   /*!< 31-bit length CRC     */
  HAL_SPI_CRC_LENGTH_32_BIT    = LL_SPI_CRC_LENGTH_32_BIT,   /*!< 32-bit length CRC     */
} hal_spi_crc_length_t;

/**
  * @brief HAL SPI TX CRC calculation initialization pattern enumeration definition.
  */
typedef enum
{
  HAL_SPI_CRC_TX_INIT_PATTERN_ALL_ZERO = LL_SPI_CRC_TX_INIT_PATTERN_ALL_ZERO,   /*!< CRC TX Initialization patterns
                                                                                   configured to zero  */
  HAL_SPI_CRC_TX_INIT_PATTERN_ALL_ONE  = LL_SPI_CRC_TX_INIT_PATTERN_ALL_ONE,   /*!< CRC TX Initialization patterns
                                                                                   configured to one   */
} hal_spi_crc_tx_init_pattern_t;

/**
  * @brief HAL SPI RX CRC calculation initialization pattern enumeration definition.
  */
typedef enum
{
  HAL_SPI_CRC_RX_INIT_PATTERN_ALL_ZERO = LL_SPI_CRC_RX_INIT_PATTERN_ALL_ZERO,   /*!< CRC RX Initialization patterns
                                                                                   configured to zero  */
  HAL_SPI_CRC_RX_INIT_PATTERN_ALL_ONE  = LL_SPI_CRC_RX_INIT_PATTERN_ALL_ONE,   /*!< CRC RX Initialization patterns
                                                                                   configured to one   */
} hal_spi_crc_rx_init_pattern_t;

/**
  * @brief HAL SPI CRC configuration structure definition.
  */
typedef struct
{
  uint32_t                      crc_polynomial;       /*!< The polynomial used for the CRC calculation. This
                                                           parameter must be an odd number between Min_Data = 0 and
                                                           Max_Data = 65535 */
  hal_spi_crc_length_t          crc_length;           /*!< The CRC Length used for the CRC calculation. */
  hal_spi_crc_tx_init_pattern_t crc_tx_init_pattern;  /*!< The transmitter CRC initialization Pattern used for the CRC
                                                           calculation.*/
  hal_spi_crc_rx_init_pattern_t crc_rx_init_pattern;  /*!< The receiver CRC initialization Pattern used for the CRC
                                                           calculation.*/
} hal_spi_crc_config_t;

#endif /* USE_HAL_SPI_CRC */

/**
  * @brief HAL SPI NSS pulse mode enumeration definition.
  */
typedef enum
{
  HAL_SPI_NSS_PULSE_DISABLE = LL_SPI_NSS_PULSE_DISABLE,   /*!< Slave select IO pin is kept at active level till data transfer is
                                                               completed, it becomes inactive with EOT flag  */
  HAL_SPI_NSS_PULSE_ENABLE  = LL_SPI_NSS_PULSE_ENABLE,    /*!< SPI data frames are interleaved with slave select IO pin non active
                                                               pulses when MIDI[3:0]>1  */
} hal_spi_nss_pulse_t;

/**
  * @brief HAL SPI NSS polarity enumeration definition.
  */
typedef enum
{
  HAL_SPI_NSS_POLARITY_LOW  = LL_SPI_NSS_POLARITY_LOW,     /*!< Low level is active for slave select signal  */
  HAL_SPI_NSS_POLARITY_HIGH = LL_SPI_NSS_POLARITY_HIGH,    /*!< High level is active for slave select signal */
} hal_spi_nss_polarity_t;

/**
  * @brief HAL SPI Master slave select IO pin  Idleness enumeration definition.
  */
typedef enum
{
  HAL_SPI_NSS_MSSI_DELAY_0_CYCLE  = LL_SPI_MSSI_DELAY_0_CYCLE,   /*!< No extra delay                     */
  HAL_SPI_NSS_MSSI_DELAY_1_CYCLE  = LL_SPI_MSSI_DELAY_1_CYCLE,   /*!< 1 clock cycle period delay added   */
  HAL_SPI_NSS_MSSI_DELAY_2_CYCLE  = LL_SPI_MSSI_DELAY_2_CYCLE,   /*!< 2 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_3_CYCLE  = LL_SPI_MSSI_DELAY_3_CYCLE,   /*!< 3 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_4_CYCLE  = LL_SPI_MSSI_DELAY_4_CYCLE,   /*!< 4 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_5_CYCLE  = LL_SPI_MSSI_DELAY_5_CYCLE,   /*!< 5 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_6_CYCLE  = LL_SPI_MSSI_DELAY_6_CYCLE,   /*!< 6 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_7_CYCLE  = LL_SPI_MSSI_DELAY_7_CYCLE,   /*!< 7 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_8_CYCLE  = LL_SPI_MSSI_DELAY_8_CYCLE,   /*!< 8 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_9_CYCLE  = LL_SPI_MSSI_DELAY_9_CYCLE,   /*!< 9 clock cycles period delay added  */
  HAL_SPI_NSS_MSSI_DELAY_10_CYCLE = LL_SPI_MSSI_DELAY_10_CYCLE,  /*!< 10 clock cycles period delay added */
  HAL_SPI_NSS_MSSI_DELAY_11_CYCLE = LL_SPI_MSSI_DELAY_11_CYCLE,  /*!< 11 clock cycles period delay added */
  HAL_SPI_NSS_MSSI_DELAY_12_CYCLE = LL_SPI_MSSI_DELAY_12_CYCLE,  /*!< 12 clock cycles period delay added */
  HAL_SPI_NSS_MSSI_DELAY_13_CYCLE = LL_SPI_MSSI_DELAY_13_CYCLE,  /*!< 13 clock cycles period delay added */
  HAL_SPI_NSS_MSSI_DELAY_14_CYCLE = LL_SPI_MSSI_DELAY_14_CYCLE,  /*!< 14 clock cycles period delay added */
  HAL_SPI_NSS_MSSI_DELAY_15_CYCLE = LL_SPI_MSSI_DELAY_15_CYCLE,  /*!< 15 clock cycles period delay added */
} hal_spi_nss_master_slave_signal_idleness_delay_t;

/**
  * @brief HAL SPI NSS configuration structure definition.
  */
typedef struct
{
  hal_spi_nss_pulse_t           nss_pulse;           /*!< Specifies whether the NSS signal is managed by hardware */
  hal_spi_nss_polarity_t        nss_polarity;        /*!< Specifies which level of slave select input/output external
                                                          signal (present on SS pin) considered as active one. */
  hal_spi_nss_master_slave_signal_idleness_delay_t      nss_mssi_delay;      /*!< Specifies an extra delay, expressed in number of SPI clock
                                                          cycle periods, inserted additionally between active edge of
                                                          slave select opening a session and the beginning of the first
                                                          data frame of the session in Master mode when SSOE is enabled.
                                                          This feature is not supported in TI mode.
                                                          To include the delay, the SPI must be disabled and re-enabled
                                                          between sessions.*/
} hal_spi_nss_config_t;

/**
  * @brief HAL SPI Master Inter-Data Idleness enumeration definition.
  */
typedef enum
{
  HAL_SPI_MIDI_DELAY_0_CYCLE  = LL_SPI_MIDI_DELAY_0_CYCLE,       /*!< No delay                      */
  HAL_SPI_MIDI_DELAY_1_CYCLE  = LL_SPI_MIDI_DELAY_1_CYCLE,       /*!< 1 clock cycle period delay    */
  HAL_SPI_MIDI_DELAY_2_CYCLE  = LL_SPI_MIDI_DELAY_2_CYCLE,       /*!< 2 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_3_CYCLE  = LL_SPI_MIDI_DELAY_3_CYCLE,       /*!< 3 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_4_CYCLE  = LL_SPI_MIDI_DELAY_4_CYCLE,       /*!< 4 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_5_CYCLE  = LL_SPI_MIDI_DELAY_5_CYCLE,       /*!< 5 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_6_CYCLE  = LL_SPI_MIDI_DELAY_6_CYCLE,       /*!< 6 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_7_CYCLE  = LL_SPI_MIDI_DELAY_7_CYCLE,       /*!< 7 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_8_CYCLE  = LL_SPI_MIDI_DELAY_8_CYCLE,       /*!< 8 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_9_CYCLE  = LL_SPI_MIDI_DELAY_9_CYCLE,       /*!< 9 clock cycles period delay   */
  HAL_SPI_MIDI_DELAY_10_CYCLE = LL_SPI_MIDI_DELAY_10_CYCLE,      /*!< 10 clock cycles period delay  */
  HAL_SPI_MIDI_DELAY_11_CYCLE = LL_SPI_MIDI_DELAY_11_CYCLE,      /*!< 11 clock cycles period delay  */
  HAL_SPI_MIDI_DELAY_12_CYCLE = LL_SPI_MIDI_DELAY_12_CYCLE,      /*!< 12 clock cycles period delay  */
  HAL_SPI_MIDI_DELAY_13_CYCLE = LL_SPI_MIDI_DELAY_13_CYCLE,      /*!< 13 clock cycles period delay  */
  HAL_SPI_MIDI_DELAY_14_CYCLE = LL_SPI_MIDI_DELAY_14_CYCLE,      /*!< 14 clock cycles period delay  */
  HAL_SPI_MIDI_DELAY_15_CYCLE = LL_SPI_MIDI_DELAY_15_CYCLE,      /*!< 15 clock cycles period delay  */
} hal_spi_master_inter_data_idleness_delay_t;

/**
  * @brief HAL SPI underrun behavior enumeration definition.
  */
typedef enum
{
  HAL_SPI_UNDERRUN_BEHAV_REGISTER_PATTERN  = LL_SPI_UNDERRUN_CONFIG_REGISTER_PATTERN,    /*!< Slave sends a constant
                                                                                          pattern defined by the user
                                                                                          at the SPI_UDRDR register */
  HAL_SPI_UNDERRUN_BEHAV_LAST_RECEIVED     = LL_SPI_UNDERRUN_CONFIG_LAST_RECEIVED,       /*!< Slave repeats last
                                                                                          received data from master */
} hal_spi_underrun_behavior_t;


/**
  * @brief HAL SPI underrun detection configuration structure definition.
  */
typedef struct
{
  hal_spi_underrun_behavior_t     underrun_behavior;   /*!< Behavior of slave transmitter at underrun condition  */
} hal_spi_underrun_config_t;

/**
  * @brief HAL SPI TI mode feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_TI_MODE_DISABLED = 0U,                 /*!< TI mode feature disabled */
  HAL_SPI_TI_MODE_ENABLED  = 1U,                 /*!< TI mode feature enabled  */
} hal_spi_ti_mode_status_t;

/**
  * @brief HAL SPI Master Receiver automatic suspension feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_MASTER_RX_AUTO_SUSPEND_DISABLED = 0U,     /*!< Master receiver automatic suspension feature disabled */
  HAL_SPI_MASTER_RX_AUTO_SUSPEND_ENABLED  = 1U,     /*!< Master receiver automatic suspension feature enabled  */
} hal_spi_master_rx_auto_suspend_status_t;

/**
  * @brief HAL SPI Master keep IO state feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_MASTER_KEEP_IO_STATE_DISABLED = 0U,   /*!< Master keep IO state feature disabled */
  HAL_SPI_MASTER_KEEP_IO_STATE_ENABLED  = 1U,   /*!< Master keep IO state feature enabled  */
} hal_spi_master_keep_io_state_status_t;

/**
  * @brief HAL SPI io swap feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_MOSI_MISO_SWAP_DISABLED = 0U,                          /*!< IO swap feature disabled */
  HAL_SPI_MOSI_MISO_SWAP_ENABLED  = 1U,                          /*!< IO swap feature enabled  */
} hal_spi_mosi_miso_swap_status_t;


/**
  * @brief HAL SPI ready pin feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_READY_PIN_DISABLED = 0U,               /*!<  Ready pin feature disabled */
  HAL_SPI_READY_PIN_ENABLED  = 1U,               /*!<  Ready pin feature enabled  */
} hal_spi_ready_pin_status_t;

/**
  * @brief HAL SPI ready pin input/output polarity enumeration definition.
  */
typedef enum
{
  HAL_SPI_READY_PIN_POLARITY_HIGH = LL_SPI_READY_PIN_POLARITY_HIGH, /*!< High level of the signal means the slave is
                                                                     ready for communication */
  HAL_SPI_READY_PIN_POLARITY_LOW  = LL_SPI_READY_PIN_POLARITY_LOW,  /*!< Low level of the signal means the slave is
                                                                     ready for communication  */
} hal_spi_ready_pin_polarity_t;

/**
  * @brief HAL SPI Delay Read Data Sampling feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_DRDS_DISABLED = 0U,               /*!<  Delay Read Data Sampling feature disabled */
  HAL_SPI_DRDS_ENABLED  = 1U,               /*!<  Delay Read Data Sampling feature enabled  */
} hal_spi_drds_status_t;

/**
  * @brief HAL SPI io configuration feature status enumeration definition.
  */
typedef enum
{
  HAL_SPI_IO_CFG_UNLOCKED = 0U,               /*!< IO configuration feature unlocked */
  HAL_SPI_IO_CFG_LOCKED   = 1U,               /*!< IO configuration feature locked   */
} hal_spi_io_cfg_status_t;

/**
  * @brief HAL SPI FIFO threshold level enumeration definition.
  */
typedef enum
{
  HAL_SPI_FIFO_THRESHOLD_1_DATA  = LL_SPI_FIFO_THRESHOLD_1_DATA,    /*!< 1-data  */
  HAL_SPI_FIFO_THRESHOLD_2_DATA  = LL_SPI_FIFO_THRESHOLD_2_DATA,    /*!< 2-data  */
  HAL_SPI_FIFO_THRESHOLD_3_DATA  = LL_SPI_FIFO_THRESHOLD_3_DATA,    /*!< 3-data  */
  HAL_SPI_FIFO_THRESHOLD_4_DATA  = LL_SPI_FIFO_THRESHOLD_4_DATA,    /*!< 4-data  */
  HAL_SPI_FIFO_THRESHOLD_5_DATA  = LL_SPI_FIFO_THRESHOLD_5_DATA,    /*!< 5-data  */
  HAL_SPI_FIFO_THRESHOLD_6_DATA  = LL_SPI_FIFO_THRESHOLD_6_DATA,    /*!< 6-data  */
  HAL_SPI_FIFO_THRESHOLD_7_DATA  = LL_SPI_FIFO_THRESHOLD_7_DATA,    /*!< 7-data  */
  HAL_SPI_FIFO_THRESHOLD_8_DATA  = LL_SPI_FIFO_THRESHOLD_8_DATA,    /*!< 8-data  */
  HAL_SPI_FIFO_THRESHOLD_9_DATA  = LL_SPI_FIFO_THRESHOLD_9_DATA,    /*!< 9-data  */
  HAL_SPI_FIFO_THRESHOLD_10_DATA = LL_SPI_FIFO_THRESHOLD_10_DATA,   /*!< 10-data  */
  HAL_SPI_FIFO_THRESHOLD_11_DATA = LL_SPI_FIFO_THRESHOLD_11_DATA,   /*!< 11-data  */
  HAL_SPI_FIFO_THRESHOLD_12_DATA = LL_SPI_FIFO_THRESHOLD_12_DATA,   /*!< 12-data  */
  HAL_SPI_FIFO_THRESHOLD_13_DATA = LL_SPI_FIFO_THRESHOLD_13_DATA,   /*!< 13-data  */
  HAL_SPI_FIFO_THRESHOLD_14_DATA = LL_SPI_FIFO_THRESHOLD_14_DATA,   /*!< 14-data  */
  HAL_SPI_FIFO_THRESHOLD_15_DATA = LL_SPI_FIFO_THRESHOLD_15_DATA,   /*!< 15-data  */
  HAL_SPI_FIFO_THRESHOLD_16_DATA = LL_SPI_FIFO_THRESHOLD_16_DATA,   /*!< 16-data  */
} hal_spi_fifo_threshold_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */
#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)

/** @defgroup SPI_Error_Code SPI Error Codes
  * @{
  */
#define HAL_SPI_ERROR_NONE                     (0UL)           /*!< 0x00000000: No error                              */
#define HAL_SPI_ERROR_MODF                     (0x01UL << 0U)  /*!< 0x00000001: Mode fault error                      */
#if defined(USE_HAL_SPI_CRC) && (USE_HAL_SPI_CRC == 1)
#define HAL_SPI_ERROR_CRC                      (0x01UL << 1U)  /*!< 0x00000002: CRC error                             */
#endif /* USE_HAL_SPI_CRC */
#define HAL_SPI_ERROR_OVR                      (0x01UL << 2U)  /*!< 0x00000004: Overrun error                         */
#define HAL_SPI_ERROR_FRE                      (0x01UL << 3U)  /*!< 0x00000008: Frame format error                    */
#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
#define HAL_SPI_ERROR_DMA                      (0x01UL << 4U)  /*!< 0x00000010: DMA transfer error                    */
#endif /* USE_HAL_SPI_DMA */
#define HAL_SPI_ERROR_ABORT                    (0x01UL << 5U)  /*!< 0x00000020: Error during SPI Abort procedure      */
#define HAL_SPI_ERROR_UDR                      (0x01UL << 6U)  /*!< 0x00000040: Underrun error                        */
/**
  * @}
  */
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization / De-Initialization functions
  * @{
  */
hal_status_t HAL_SPI_Init(hal_spi_handle_t *hspi, hal_spi_t instance);
void HAL_SPI_DeInit(hal_spi_handle_t *hspi);

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group2 General Config functions
  * @{
  */
hal_status_t HAL_SPI_SetConfig(hal_spi_handle_t *hspi, const hal_spi_config_t *p_config);
void HAL_SPI_GetConfig(const hal_spi_handle_t *hspi, hal_spi_config_t *p_config);

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group3 Features functions
  * @{
  */
#if defined(USE_HAL_SPI_CRC) && (USE_HAL_SPI_CRC == 1)
hal_status_t HAL_SPI_SetConfigCRC(hal_spi_handle_t *hspi, const hal_spi_crc_config_t *p_config);
void         HAL_SPI_GetConfigCRC(const hal_spi_handle_t *hspi, hal_spi_crc_config_t *p_config);
hal_status_t HAL_SPI_EnableCRC(const hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_DisableCRC(const hal_spi_handle_t *hspi);
hal_spi_crc_status_t HAL_SPI_IsEnabledCRC(const hal_spi_handle_t *hspi);
#endif /* USE_HAL_SPI_CRC */

hal_status_t HAL_SPI_SetConfigNSS(hal_spi_handle_t *hspi, const hal_spi_nss_config_t *p_config);
void HAL_SPI_GetConfigNSS(const hal_spi_handle_t *hspi, hal_spi_nss_config_t *p_config);

hal_status_t HAL_SPI_SLAVE_SetConfigUnderrun(const hal_spi_handle_t *hspi, const hal_spi_underrun_config_t *p_config);
void HAL_SPI_SLAVE_GetConfigUnderrun(const hal_spi_handle_t *hspi, hal_spi_underrun_config_t *p_config);

hal_status_t HAL_SPI_EnableTIMode(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_DisableTIMode(hal_spi_handle_t *hspi);
hal_spi_ti_mode_status_t HAL_SPI_IsEnabledTIMode(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_MASTER_EnableReceiverAutoSuspend(const hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_MASTER_DisableReceiverAutoSuspend(const hal_spi_handle_t *hspi);
hal_spi_master_rx_auto_suspend_status_t HAL_SPI_MASTER_IsEnabledReceiverAutoSuspend(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_MASTER_EnableKeepIOState(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_MASTER_DisableKeepIOState(hal_spi_handle_t *hspi);
hal_spi_master_keep_io_state_status_t HAL_SPI_MASTER_IsEnabledKeepIOState(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_EnableMosiMisoSwap(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_DisableMosiMisoSwap(hal_spi_handle_t *hspi);
hal_spi_mosi_miso_swap_status_t HAL_SPI_IsEnabledMosiMisoSwap(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_EnableReadyPin(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_DisableReadyPin(hal_spi_handle_t *hspi);
hal_spi_ready_pin_status_t HAL_SPI_IsEnabledReadyPin(const hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_SetReadyPinPolarity(hal_spi_handle_t *hspi, hal_spi_ready_pin_polarity_t polarity);
hal_spi_ready_pin_polarity_t HAL_SPI_GetReadyPinPolarity(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_EnableDelayReadDataSampling(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_DisableDelayReadDataSampling(hal_spi_handle_t *hspi);
hal_spi_drds_status_t HAL_SPI_IsEnabledDelayReadDataSampling(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_LockIOConfig(const hal_spi_handle_t *hspi);
hal_spi_io_cfg_status_t HAL_SPI_IsLockedIOConfig(const hal_spi_handle_t *hspi);

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group5 Items functions
  * @{
  */
hal_status_t HAL_SPI_SetMode(hal_spi_handle_t *hspi, const hal_spi_mode_t mode);
hal_spi_mode_t HAL_SPI_GetMode(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetDirection(hal_spi_handle_t *hspi, const hal_spi_direction_t direction);
hal_spi_direction_t HAL_SPI_GetDirection(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetDataWidth(const hal_spi_handle_t *hspi, const hal_spi_data_width_t data_width);
hal_spi_data_width_t HAL_SPI_GetDataWidth(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetClockPolarity(hal_spi_handle_t *hspi, const hal_spi_clock_polarity_t clock_polarity);
hal_spi_clock_polarity_t HAL_SPI_GetClockPolarity(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetClockPhase(hal_spi_handle_t *hspi, const hal_spi_clock_phase_t clock_phase);
hal_spi_clock_phase_t HAL_SPI_GetClockPhase(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetBaudRatePrescaler(const hal_spi_handle_t *hspi,
                                          const hal_spi_baud_rate_prescaler_t baud_rate_prescaler);
hal_spi_baud_rate_prescaler_t HAL_SPI_GetBaudRatePrescaler(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetFirstBit(hal_spi_handle_t *hspi, const hal_spi_first_bit_t first_bit);
hal_spi_first_bit_t HAL_SPI_GetFirstBit(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetNSSPinManagement(hal_spi_handle_t *hspi, hal_spi_nss_pin_management_t nss_pin_management);
hal_spi_nss_pin_management_t HAL_SPI_GetNSSPinManagement(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_SetFifoThreshold(const hal_spi_handle_t *hspi, const hal_spi_fifo_threshold_t fifo_threshold);
hal_spi_fifo_threshold_t HAL_SPI_GetFifoThreshold(const hal_spi_handle_t *hspi);

hal_status_t HAL_SPI_MASTER_SetInterDataIdlenessDelay(hal_spi_handle_t *hspi,
                                                      const hal_spi_master_inter_data_idleness_delay_t nb_cycles);
hal_spi_master_inter_data_idleness_delay_t HAL_SPI_MASTER_GetInterDataIdlenessDelay(const hal_spi_handle_t *hspi);

#if defined(USE_HAL_SPI_USER_DATA) && (USE_HAL_SPI_USER_DATA == 1)
void HAL_SPI_SetUserData(hal_spi_handle_t *hspi, const void *p_user_data);
const void *HAL_SPI_GetUserData(const hal_spi_handle_t *hspi);
#endif /* USE_HAL_SPI_USER_DATA == 1 */

#if defined (USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
hal_status_t HAL_SPI_SetTxDMA(hal_spi_handle_t *hspi, hal_dma_handle_t *hdma);
hal_status_t HAL_SPI_SetRxDMA(hal_spi_handle_t *hspi, hal_dma_handle_t *hdma);
#endif /* USE_HAL_SPI_DMA */
/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group6 IO operation functions
  * @{
  */
hal_status_t HAL_SPI_Transmit(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet, uint32_t timeout_ms);
hal_status_t HAL_SPI_Receive(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet, uint32_t timeout_ms);
hal_status_t HAL_SPI_TransmitReceive(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                     uint32_t count_packet, uint32_t timeout_ms);

hal_status_t HAL_SPI_Transmit_IT(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet);
hal_status_t HAL_SPI_Receive_IT(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet);
hal_status_t HAL_SPI_TransmitReceive_IT(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                        uint32_t count_packet);

#if defined(USE_HAL_SPI_DMA) && (USE_HAL_SPI_DMA == 1)
hal_status_t HAL_SPI_Transmit_DMA(hal_spi_handle_t *hspi, const void *p_data, uint32_t count_packet);
hal_status_t HAL_SPI_Receive_DMA(hal_spi_handle_t *hspi, void *p_data, uint32_t count_packet);
hal_status_t HAL_SPI_TransmitReceive_DMA(hal_spi_handle_t *hspi, const void *p_tx_data, void *p_rx_data,
                                         uint32_t count_packet);
#endif /* USE_HAL_SPI_DMA */


hal_status_t HAL_SPI_Abort(hal_spi_handle_t *hspi);
hal_status_t HAL_SPI_Abort_IT(hal_spi_handle_t *hspi);
/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group7 IRQ Handler/Callbacks/Register Callbacks functions
  * @{
  */
void HAL_SPI_IRQHandler(hal_spi_handle_t *hspi);
void HAL_SPI_ErrorCallback(hal_spi_handle_t *hspi);
void HAL_SPI_TxCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_RxCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_TxRxCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_TxHalfCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_RxHalfCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_TxRxHalfCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_AbortCpltCallback(hal_spi_handle_t *hspi);
void HAL_SPI_SuspendCallback(hal_spi_handle_t *hspi);

#if defined(USE_HAL_SPI_REGISTER_CALLBACKS) && (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
hal_status_t HAL_SPI_RegisterErrorCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterTxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterRxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterTxRxCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterTxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterRxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterTxRxHalfCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterAbortCpltCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
hal_status_t HAL_SPI_RegisterSuspendCallback(hal_spi_handle_t *hspi, hal_spi_cb_t p_callback);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group8 Peripheral current frequency, state and errors functions
  * @{
  */
uint32_t HAL_SPI_GetClockFreq(const hal_spi_handle_t *hspi);
hal_spi_state_t HAL_SPI_GetState(const hal_spi_handle_t *hspi);

#if defined(USE_HAL_SPI_GET_LAST_ERRORS) && (USE_HAL_SPI_GET_LAST_ERRORS == 1)
uint32_t HAL_SPI_GetLastErrorCodes(const hal_spi_handle_t *hspi);
#endif /* USE_HAL_SPI_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup SPI_Exported_Functions_Group9 Acquire/release Bus functions
  * @{
  */
hal_status_t HAL_SPI_AcquireBus(hal_spi_handle_t *hspi, uint32_t timeout_ms);
hal_status_t HAL_SPI_ReleaseBus(hal_spi_handle_t *hspi);

/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

/**
  * @}
  */

/**
  * @}
  */
#endif /* SPI1 || SPI2 || SPI3 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_SPI_H */
