/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_fdcan.h
  * @brief   Header file for the FDCAN HAL module.
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_FDCAN_H
#define STM32C5XX_HAL_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"

/** @addtogroup STM32C5XX_HAL_Driver
  * @{
  */

#if defined(FDCAN1) || defined(FDCAN2)

/** @defgroup FDCAN FDCAN
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Exported_Constants HAL FDCAN Constants
  * @{
  */

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
/** @defgroup FDCAN_Error_Codes FDCAN error codes
  * @{
  */

#define HAL_FDCAN_ERROR_NONE               (0UL)         /*!< No error                                                */
#define HAL_FDCAN_ERROR_LOG_OVERFLOW       FDCAN_IR_ELO  /*!< Overflow of CAN error logging counter                   */
#define HAL_FDCAN_ERROR_RAM_WDG            FDCAN_IR_WDI  /*!< Message RAM watchdog event occurred                     */
#define HAL_FDCAN_ERROR_PROTOCOL_ARBT      FDCAN_IR_PEA  /*!< Protocol error in arbitration phase (nominal bit time)  */
#define HAL_FDCAN_ERROR_PROTOCOL_DATA      FDCAN_IR_PED  /*!< Protocol error in data phase (data bit time)            */
#define HAL_FDCAN_ERROR_RESERVED_AREA      FDCAN_IR_ARA  /*!< Access to reserved address                              */
#define HAL_FDCAN_ERROR_TIMEOUT_OCCURRED   FDCAN_IR_TOO  /*!< Timeout occurred                                        */
#define HAL_FDCAN_ERROR_RAM_ACCESS_FAILURE FDCAN_IR_MRAF /*!< Message RAM access failure occurred                     */
#define HAL_FDCAN_ERROR_BUS_FAULT_OFF      FDCAN_IR_BO   /*!< Bus off error                                           */
#define HAL_FDCAN_ERROR_BUS_FAULT_PASSIVE  FDCAN_IR_EP   /*!< Error passive error                                     */
#define HAL_FDCAN_ERROR_BUS_FAULT_WARNING  FDCAN_IR_EW   /*!< Error warning error                                     */

/**
  * @}
  */
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

/** @defgroup FDCAN_Interrupt_Groups FDCAN interrupt groups
  * @{
  */

/**
  * @brief Rx FIFO 0 group.
  *        This group contains the following interrupts:
  *        - Rx FIFO 0 new message
  *        - Rx FIFO 0 full
  *        - Rx FIFO 0 message lost
  */
#define HAL_FDCAN_IT_GROUP_RX_FIFO_0      (FDCAN_ILS_RXFIFO0)

/**
  * @brief Rx FIFO 1 group.
  *        This group contains the following interrupts:
  *        - Rx FIFO 1 new message
  *        - Rx FIFO 1 full
  *        - Rx FIFO 1 message lost
  */
#define HAL_FDCAN_IT_GROUP_RX_FIFO_1      (FDCAN_ILS_RXFIFO1)

/**
  * @brief Status message group.
  *        This group contains the following interrupts:
  *        - Rx high priority message
  *        - Tx complete
  *        - Tx abort complete
  */
#define HAL_FDCAN_IT_GROUP_STATUS_MSG     (FDCAN_ILS_SMSG)

/**
  * @brief Tx FIFO error group.
  *        This group contains the following interrupts:
  *        - Tx FIFO empty
  *        - Tx event FIFO new data
  *        - Tx event FIFO full
  *        - Tx event FIFO element lost
  */
#define HAL_FDCAN_IT_GROUP_TX_FIFO_ERROR  (FDCAN_ILS_TFERR)

/**
  * @brief Miscellaneous group.
  *        This group contains the following interrupts:
  *        - Timestamp wraparound
  *        - Message RAM access failure
  *        - Timeout occurred
  */
#define HAL_FDCAN_IT_GROUP_MISC           (FDCAN_ILS_MISC)

/**
  * @brief Bit and line error group.
  *        This group contains the following interrupts:
  *        - Error logging overflow
  *        - Error passive
  */
#define HAL_FDCAN_IT_GROUP_BIT_LINE_ERROR (FDCAN_ILS_BERR)

/**
  * @brief Protocol error group.
  *        This group contains the following interrupts:
  *        - Error warning
  *        - Bus off
  *        - Message RAM watchdog
  *        - Protocol error in arbitration phase
  *        - Protocol error in data phase
  *        - Reserved address access error
  */
#define HAL_FDCAN_IT_GROUP_PROTOCOL_ERROR (FDCAN_ILS_PERR)

/**
  * @}
  */

/** @defgroup FDCAN_Interrupt_Sources FDCAN interrupt sources
  * @{
  */

/* FDCAN interrupt group Rx FIFO 0 */
#define HAL_FDCAN_IT_RX_FIFO_0_NEW_MSG        FDCAN_IE_RF0NE /*!< Rx FIFO 0 new message interrupt                     */
#define HAL_FDCAN_IT_RX_FIFO_0_FULL           FDCAN_IE_RF0FE /*!< Rx FIFO 0 full interrupt                            */
#define HAL_FDCAN_IT_RX_FIFO_0_MSG_LOST       FDCAN_IE_RF0LE /*!< Rx FIFO 0 message lost interrupt                    */

/* FDCAN interrupt group Rx FIFO 1 */
#define HAL_FDCAN_IT_RX_FIFO_1_NEW_MSG        FDCAN_IE_RF1NE /*!< Rx FIFO 1 new message interrupt                     */
#define HAL_FDCAN_IT_RX_FIFO_1_FULL           FDCAN_IE_RF1FE /*!< Rx FIFO 1 full interrupt                            */
#define HAL_FDCAN_IT_RX_FIFO_1_MSG_LOST       FDCAN_IE_RF1LE /*!< Rx FIFO 1 message lost interrupt                    */

/* FDCAN interrupt group Tx event FIFO */
#define HAL_FDCAN_IT_TX_FIFO_EMPTY           FDCAN_IE_TFEE  /*!< Tx FIFO empty                                        */
#define HAL_FDCAN_IT_TX_EVT_FIFO_NEW_DATA    FDCAN_IE_TEFNE /*!< Tx handler wrote Tx event FIFO element               */
#define HAL_FDCAN_IT_TX_EVT_FIFO_FULL        FDCAN_IE_TEFFE /*!< Tx event FIFO full                                   */
#define HAL_FDCAN_IT_TX_EVT_FIFO_ELEM_LOST   FDCAN_IE_TEFLE /*!< Tx event FIFO element lost                           */

/* FDCAN interrupt group status message */
#define HAL_FDCAN_IT_RX_HIGH_PRIORITY_MSG    FDCAN_IE_HPME  /*!< High priority message received                       */
#define HAL_FDCAN_IT_TX_COMPLETE             FDCAN_IE_TCE   /*!< Transmission completed                               */
#define HAL_FDCAN_IT_TX_ABORT_COMPLETE       FDCAN_IE_TCFE  /*!< Transmission cancellation finished                   */

/* FDCAN interrupt group misc. */
#define HAL_FDCAN_IT_TIMESTAMP_WRAPAROUND    FDCAN_IE_TSWE  /*!< Timestamp counter wrapped around                     */
#define HAL_FDCAN_IT_RAM_ACCESS_FAILURE      FDCAN_IE_MRAFE /*!< Message RAM access failure occurred                  */
#define HAL_FDCAN_IT_TIMEOUT_OCCURRED        FDCAN_IE_TOOE  /*!< Timeout reached                                      */

/* FDCAN interrupt group bit and line error */
#define HAL_FDCAN_IT_ERROR_LOGGING_OVERFLOW  FDCAN_IE_ELOE  /*!< Overflow of FDCAN error logging counter occurred     */
#define HAL_FDCAN_IT_ERROR_PASSIVE           FDCAN_IE_EPE   /*!< Error passive status changed                         */

/* FDCAN interrupt group protocol error */
#define HAL_FDCAN_IT_ERROR_WARNING           FDCAN_IE_EWE   /*!< Error warning status changed                         */
#define HAL_FDCAN_IT_BUS_OFF                 FDCAN_IE_BOE   /*!< Bus off status changed                               */
#define HAL_FDCAN_IT_RAM_WATCHDOG            FDCAN_IE_WDIE  /*!< Message RAM watchdog event due to missing READY      */
#define HAL_FDCAN_IT_ARB_PROTOCOL_ERROR      FDCAN_IE_PEAE  /*!< Protocol error in arbitration phase detected         */
#define HAL_FDCAN_IT_DATA_PROTOCOL_ERROR     FDCAN_IE_PEDE  /*!< Protocol error in data phase detected                */
#define HAL_FDCAN_IT_RESERVED_ADDRESS_ACCESS FDCAN_IE_ARAE  /*!< Access to reserved address occurred                  */

/**
  * @}
  */

/** @defgroup FDCAN_Interrupt_Flags FDCAN interrupt flags
  * @brief FDCAN interrupt register (FDCAN_IR): The flags are set when one of the listed conditions is detected.
  * @{
  */

/* FDCAN interrupt flags for Rx FIFO 0 */
#define HAL_FDCAN_FLAG_RX_FIFO_0_MSG_LOST     FDCAN_IR_RF0L /*!< Rx FIFO 0 message lost                               */
#define HAL_FDCAN_FLAG_RX_FIFO_0_FULL         FDCAN_IR_RF0F /*!< Rx FIFO 0 full                                       */
#define HAL_FDCAN_FLAG_RX_FIFO_0_NEW_MSG      FDCAN_IR_RF0N /*!< New message written to Rx FIFO 0                     */

/* FDCAN interrupt flags for Rx FIFO 1 */
#define HAL_FDCAN_FLAG_RX_FIFO_1_MSG_LOST      FDCAN_IR_RF1L /*!< Rx FIFO 1 message lost                              */
#define HAL_FDCAN_FLAG_RX_FIFO_1_FULL          FDCAN_IR_RF1F /*!< Rx FIFO 1 full                                      */
#define HAL_FDCAN_FLAG_RX_FIFO_1_NEW_MSG       FDCAN_IR_RF1N /*!< New message written to Rx FIFO 1                    */

/* FDCAN interrupt flags for Tx event FIFO */
#define HAL_FDCAN_FLAG_TX_EVT_FIFO_ELEM_LOST  FDCAN_IR_TEFL  /*!< Tx event FIFO element lost                          */
#define HAL_FDCAN_FLAG_TX_EVT_FIFO_FULL       FDCAN_IR_TEFF  /*!< Tx event FIFO full                                  */
#define HAL_FDCAN_FLAG_TX_EVT_FIFO_NEW_DATA   FDCAN_IR_TEFN  /*!< New Tx event FIFO element written                   */

/**
  * @}
  */

/** @defgroup FDCAN_IT_Tx_Complete_Buffers_Select FDCAN interrupt Tx complete buffers select
  * @{
  */

#define HAL_FDCAN_IT_TX_CPLT_BUFFER_0   (1UL << 0U)      /*!< Tx complete interrupt on Tx buffer 0     */
#define HAL_FDCAN_IT_TX_CPLT_BUFFER_1   (1UL << 1U)      /*!< Tx complete interrupt on Tx buffer 1     */
#define HAL_FDCAN_IT_TX_CPLT_BUFFER_2   (1UL << 2U)      /*!< Tx complete interrupt on Tx buffer 2     */
#define HAL_FDCAN_IT_TX_CPLT_BUFFER_ALL FDCAN_TXBTIE_TIE /*!< Tx complete interrupt on all Tx buffers  */

/**
  * @}
  */

/**  @defgroup FDCAN_IT_Tx_Abort_Buffers_Select FDCAN interrupt Tx abort buffer select
  * @{
  */

#define HAL_FDCAN_IT_TX_ABORT_BUFFER_0   (1UL << 0U)       /*!< Tx abort interrupt on Tx buffer 0     */
#define HAL_FDCAN_IT_TX_ABORT_BUFFER_1   (1UL << 1U)       /*!< Tx abort interrupt on Tx buffer 1     */
#define HAL_FDCAN_IT_TX_ABORT_BUFFER_2   (1UL << 2U)       /*!< Tx abort interrupt on Tx buffer 2     */
#define HAL_FDCAN_IT_TX_ABORT_BUFFER_ALL FDCAN_TXBCIE_CFIE /*!< Tx abort interrupt on all Tx buffers  */

/**
  * @}
  */

/** @defgroup FDCAN_Tx_Buffer_Location HAL FDCAN Tx location
  * @{
  */

#define HAL_FDCAN_TX_BUFFER_0   (1UL << 0U)    /*!< Tx buffer0     */
#define HAL_FDCAN_TX_BUFFER_1   (1UL << 1U)    /*!< Tx buffer1     */
#define HAL_FDCAN_TX_BUFFER_2   (1UL << 2U)    /*!< Tx buffer2     */
#define HAL_FDCAN_TX_BUFFER_ALL FDCAN_TXBAR_AR /*!< All Tx buffers */

/**
  * @}
  */

/** @defgroup FDCAN_Interrupt_Lines FDCAN interrupt lines
  * @{
  */

#define HAL_FDCAN_IT_LINE_0 FDCAN_ILE_EINT0 /*!< Interrupt line 0 */
#define HAL_FDCAN_IT_LINE_1 FDCAN_ILE_EINT1 /*!< Interrupt line 1 */

/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Exported_Types HAL FDCAN Types
  * @{
  */

/**
  * @brief  HAL FDCAN frame type.
  * @note   When a Tx frame is configured as @ref HAL_FDCAN_FRAME_REMOTE, the FDCAN core always transmits a classic CAN
  *         remote frame @ref HAL_FDCAN_HEADER_FRAME_FORMAT_CAN according to ISO 11898-1, even if CAN FD operation is
  *         @ref HAL_FDCAN_FRAME_FORMAT_FD_NO_BRS or @ref HAL_FDCAN_FRAME_FORMAT_FD_BRS.
  *         There is no remote frame in CAN FD format by design.
  */
typedef enum
{
  HAL_FDCAN_FRAME_DATA   = 0U,  /*!< Data frame type   */
  HAL_FDCAN_FRAME_REMOTE = 1U   /*!< Remote frame type */
} hal_fdcan_frame_type_t;

/**
  * @brief  HAL FDCAN ID type.
  */
typedef enum
{
  HAL_FDCAN_ID_STANDARD = 0U, /*!< Standard ID type */
  HAL_FDCAN_ID_EXTENDED = 1U  /*!< Extended ID type */
} hal_fdcan_id_type_t;

/**
  * @brief  HAL FDCAN error state indicator.
  * @note   This field controls the ESI bit in the transmitted frame only for CAN FD data
  *         @ref HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN.
  *         For classic CAN frames @ref HAL_FDCAN_HEADER_FRAME_FORMAT_CAN there is no ESI bit in the frame format;
  *         in that case the core ignores this field and behaves according to its own CAN error state machine.
  */
typedef enum
{
  HAL_FDCAN_ERROR_STATE_IND_ACTIVE  = 0U, /*!< Active error state indicator  */
  HAL_FDCAN_ERROR_STATE_IND_PASSIVE = 1U  /*!< Passive error state indicator */
} hal_fdcan_error_state_indicator_t;

/**
  * @brief  HAL FDCAN bitrate switching.
  * @note   Bit rate switch is only used for CAN FD data frames. For classic CAN @ref HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN
  *         or remote frames @ref HAL_FDCAN_FRAME_REMOTE, this field is ignored by the hardware.
  */
typedef enum
{
  HAL_FDCAN_BIT_RATE_SWITCH_OFF = 0U, /*!< Bit rate switching off */
  HAL_FDCAN_BIT_RATE_SWITCH_ON  = 1U  /*!< Bit rate switching on  */
} hal_fdcan_bit_rate_switch_t;

/**
  * @brief  HAL FDCAN header frame format.
  * @note   This field reflects the intended frame format, but the FDCAN core will override it to classic format
  *         to comply with the CAN/CAN FD protocol:
  *         - When a Tx frame is configured as @ref HAL_FDCAN_FRAME_REMOTE, the core always transmits a classic CAN
  *           remote frame, even if CAN FD operation is enabled i.e @ref HAL_FDCAN_FRAME_FORMAT_FD_NO_BRS or
  *           @ref HAL_FDCAN_FRAME_FORMAT_FD_BRS and this field is set to @ref HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN.
  *         - When CAN FD operation is disabled @ref HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN, all frames are
  *           transmitted as classic CAN frames @ref HAL_FDCAN_HEADER_FRAME_FORMAT_CAN regardless of this field.
  */
typedef enum
{
  HAL_FDCAN_HEADER_FRAME_FORMAT_CAN    = 0U, /*!< Standard frame format                        */
  HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN = 1U  /*!< CAN FD frame format (new DLC-coding and CRC) */
} hal_fdcan_header_frame_format_t;

/**
  * @brief  HAL FDCAN event FIFO.
  */
typedef enum
{
  HAL_FDCAN_TX_EVENTS_FIFO_DISCARD = 0U, /*!< Do not store Tx event in FIFO */
  HAL_FDCAN_TX_EVENTS_FIFO_STORE   = 1U  /*!< Store Tx event in FIFO        */
} hal_fdcan_tx_event_fifo_t;

/**
  * @brief  HAL FDCAN Tx event type.
  */
typedef enum
{
  HAL_FDCAN_TX_EVENT                   = 1U, /*!< A successful frame transmission                                     */
  HAL_FDCAN_TX_EVENT_IN_SPITE_OF_ABORT = 2U  /*!< Transmission in spite of cancellation. Both Corresponding Tx buffer
                                                  transmission occurred bit @ref FDCAN_IT_Tx_Complete_Buffers_Select
                                                  and cancellation finished bit @ref FDCAN_IT_Tx_Abort_Buffers_Select
                                                  are set.
                                                  This event type is set in case of:
                                                  - A successful frame transmission.
                                                  - A disabled auto-retransmission (DAR), that means:
                                                  @ref hal_fdcan_config_t::auto_retransmission equals to
                                                  @ref HAL_FDCAN_AUTO_RETRANSMISSION_DISABLE.
                                                  - Storage of Tx events is enabled, that means:
                                                  @ref hal_fdcan_tx_header_t::event_fifo_control equals to
                                                  @ref HAL_FDCAN_TX_EVENTS_FIFO_STORE.                                */
} hal_fdcan_tx_event_type_t;

/**
  * @brief  HAL FDCAN data length code.
  *         The definition is the following one:
  *          - For classic CAN:
  *            - 0 to 8 : received frame has 0 to 8 data bytes
  *            - 9 to 15: received frame has 8 data bytes (max)
  *          - For CAN FD:
  *            - 0 to 8 : received frame has 0 to 8 data bytes
  *            - 9 to 15: received frame has 12/16/20/24/32/48/64 data bytes
  * @note   For remote frames @ref HAL_FDCAN_FRAME_REMOTE, only the DLC encoded by this field is
  *         transmitted; no data bytes are sent and the contents of the Tx data buffer are ignored by the core.
  */
typedef enum
{
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_0_BYTE = 0x0U, /*!< 0 byte data length code   */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_1_BYTE = 0x1U, /*!< 1 byte data length code   */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_2_BYTE = 0x2U, /*!< 2 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_3_BYTE = 0x3U, /*!< 3 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_4_BYTE = 0x4U, /*!< 4 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_5_BYTE = 0x5U, /*!< 5 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_6_BYTE = 0x6U, /*!< 6 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_7_BYTE = 0x7U, /*!< 7 bytes data length code  */
  HAL_FDCAN_DATA_LEN_CAN_FDCAN_8_BYTE = 0x8U, /*!< 8 bytes data length code  */
  HAL_FDCAN_DATA_LEN_FDCAN_12_BYTE    = 0x9U, /*!< 12 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_16_BYTE    = 0xAU, /*!< 16 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_20_BYTE    = 0xBU, /*!< 20 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_24_BYTE    = 0xCU, /*!< 24 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_32_BYTE    = 0xDU, /*!< 32 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_48_BYTE    = 0xEU, /*!< 48 bytes data length code */
  HAL_FDCAN_DATA_LEN_FDCAN_64_BYTE    = 0xFU  /*!< 64 bytes data length code */
} hal_fdcan_data_length_code_t;

/**
  * @brief  HAL FDCAN frame format.
  */
typedef enum
{
  HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN = 0U,                                  /*!< Classic mode                      */
  HAL_FDCAN_FRAME_FORMAT_FD_NO_BRS   = FDCAN_CCCR_FDOE,                     /*!< FD mode without bitrate switching */
  HAL_FDCAN_FRAME_FORMAT_FD_BRS      = (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE), /*!< FD mode with bitrate switching    */
} hal_fdcan_frame_format_t;

/**
  * @brief  HAL FDCAN timestamp selection.
  */
typedef enum
{
  HAL_FDCAN_TIMESTAMP_SOURCE_ZERO     = 0U,                            /*!< Timestamp counter disabled (value is 0) */
  HAL_FDCAN_TIMESTAMP_SOURCE_INTERNAL = (0x1UL << FDCAN_TSCC_TSS_Pos), /*!< Internal timestamp counter              */
  HAL_FDCAN_TIMESTAMP_SOURCE_EXTERNAL = (0x2UL << FDCAN_TSCC_TSS_Pos), /*!< External timestamp counter              */
} hal_fdcan_timestamp_source_t;

/**
  * @brief  HAL FDCAN high priority filter list definition.
  */
typedef enum
{
  HAL_FDCAN_HIGH_PRIO_FILTER_STANDARD = 0U,             /*!< Standard filter list */
  HAL_FDCAN_HIGH_PRIO_FILTER_EXTENDED = FDCAN_HPMS_FLST /*!< Extended filter list */
} hal_fdcan_high_prio_filter_list_t;

/**
  * @brief  HAL FDCAN receive error passive level definition.
  */
typedef enum
{
  HAL_FDCAN_RX_ERROR_PASSIVE_BELOW_LEVEL = 0U,           /*!< The receive error counter is below
                                                              the error passive level of 128 */
  HAL_FDCAN_RX_ERROR_PASSIVE_REACH_LEVEL = FDCAN_ECR_RP, /*!< The receive error counter has reached
                                                              the error passive level of 128 */
} hal_fdcan_rx_error_passive_level_t;

/**
  * @brief  HAL FDCAN state structures definition.
  */
typedef enum
{
  HAL_FDCAN_STATE_RESET      = 0U,           /*!< Not yet initialized                               */
  HAL_FDCAN_STATE_INIT       = (1UL << 31U), /*!< Initialized but not yet configured                */
  HAL_FDCAN_STATE_IDLE       = (1UL << 30U), /*!< Initialized and a global config applied           */
  HAL_FDCAN_STATE_ACTIVE     = (1UL << 29U), /*!< The peripheral is running                         */
  HAL_FDCAN_STATE_POWER_DOWN = (1UL << 28U), /*!< The peripheral is in power down mode (sleep mode) */
} hal_fdcan_state_t;

/**
  * @brief  HAL FDCAN mode enumeration definition.
  */
typedef enum
{
  HAL_FDCAN_MODE_NORMAL               = 0U,                               /*!< Normal mode               */
  HAL_FDCAN_MODE_RESTRICTED_OPERATION = FDCAN_CCCR_ASM,                   /*!< Restricted operation mode */
  HAL_FDCAN_MODE_BUS_MONITORING       = FDCAN_CCCR_MON,                   /*!< Bus monitoring mode       */
  HAL_FDCAN_MODE_EXTERNAL_LOOPBACK    = FDCAN_CCCR_TEST,                  /*!< External loopback mode    */
  HAL_FDCAN_MODE_INTERNAL_LOOPBACK    = FDCAN_CCCR_TEST | FDCAN_CCCR_MON, /*!< Internal loopback mode    */
  HAL_FDCAN_MODE_INVALID              = FDCAN_CCCR_ASM | FDCAN_CCCR_MON
                                        | FDCAN_CCCR_TEST                 /*!< FDCAN invalid mode        */
} hal_fdcan_mode_t;

/**
  * @brief  HAL FDCAN FIFO operation mode.
  */
typedef enum
{
  HAL_FDCAN_RX_FIFO_MODE_BLOCKING  = 0U, /*!< Rx FIFO blocking mode  */
  HAL_FDCAN_RX_FIFO_MODE_OVERWRITE = 1U, /*!< Rx FIFO overwrite mode */
} hal_fdcan_rx_fifo_mode_t;

/**
  * @brief  HAL FDCAN auto retransmission setting.
  */
typedef enum
{
  HAL_FDCAN_AUTO_RETRANSMISSION_ENABLE  = 0U,             /*!< Enable transmitter auto retransmission */
  HAL_FDCAN_AUTO_RETRANSMISSION_DISABLE = FDCAN_CCCR_DAR, /*!< Disable transmitter auto retransmission */
} hal_fdcan_auto_retransmission_state_t;

/**
  * @brief  HAL FDCAN transmit pause setting.
  */
typedef enum
{
  HAL_FDCAN_TRANSMIT_PAUSE_DISABLE = 0U,             /*!< Disable transmitter transmit pause  */
  HAL_FDCAN_TRANSMIT_PAUSE_ENABLE  = FDCAN_CCCR_TXP, /*!< Enable transmitter transmit pause */
} hal_fdcan_transmit_pause_state_t;

/**
  * @brief  HAL FDCAN protocol exception setting.
  */
typedef enum
{
  HAL_FDCAN_PROTOCOL_EXCEPTION_ENABLE  = 0U,              /*!< Enable transmitter protocol exception  */
  HAL_FDCAN_PROTOCOL_EXCEPTION_DISABLE = FDCAN_CCCR_PXHD, /*!< Disable transmitter protocol exception */
} hal_fdcan_protocol_exception_state_t;

/**
  * @brief  HAL FDCAN transmitter delay Compensation status.
  */
typedef enum
{
  HAL_FDCAN_TX_DLY_COMPENSATION_DISABLED = 0U,             /*!< Transmitter delay compensation disabled */
  HAL_FDCAN_TX_DLY_COMPENSATION_ENABLED  = FDCAN_DBTP_TDC, /*!< Transmitter delay compensation enabled  */
} hal_fdcan_tx_delay_comp_status_t;

/**
  * @brief  HAL FDCAN ISO mode status.
  */
typedef enum
{
  HAL_FDCAN_ISO_MODE_ENABLED  = 0U,              /*!< Protocol configured for ISO mode         */
  HAL_FDCAN_ISO_MODE_DISABLED = FDCAN_CCCR_NISO, /*!< Protocol configured for CAN FD v2.0 mode */
} hal_fdcan_iso_mode_status_t;

/**
  * @brief  HAL FDCAN Tx buffer pending status.
  */
typedef enum
{
  HAL_FDCAN_TX_BUFFER_NOT_PENDING = 0U, /*!< No buffer pending           */
  HAL_FDCAN_TX_BUFFER_PENDING     = 1U, /*!< At least one buffer pending */
} hal_fdcan_tx_buffer_status_t;

/**
  * @brief  HAL FDCAN received frame status.
  */
typedef enum
{
  HAL_FDCAN_RX_FRAME_MATCHED     = 0U, /*!< Received frame matches filter index FIDx           */
  HAL_FDCAN_RX_FRAME_NOT_MATCHED = 1U  /*!< Received frame did not match any Rx filter element */
} hal_fdcan_rx_frame_status_t;

/**
  * @brief  HAL FDCAN restricted operation mode status.
  */
typedef enum
{
  HAL_FDCAN_RESTRICTED_OP_MODE_DISABLED = 0U,            /*!< Restricted mode disabled */
  HAL_FDCAN_RESTRICTED_OP_MODE_ENABLED  = FDCAN_CCCR_ASM /*!< Restricted mode enabled  */
} hal_fdcan_restricted_op_mode_status_t;

/**
  * @brief  HAL FDCAN edge filtering status.
  */
typedef enum
{
  HAL_FDCAN_EDGE_FILTERING_DISABLED = 0U,              /*!< Edge filtering disabled */
  HAL_FDCAN_EDGE_FILTERING_ENABLED  = FDCAN_CCCR_EFBI, /*!< Edge filtering enabled  */
} hal_fdcan_edge_filtering_status_t;

/**
  * @brief  HAL FDCAN FIFO/queue status - free or full.
  */
typedef enum
{
  HAL_FDCAN_FIFO_STATUS_FREE = 0U,               /*!< Tx FIFO/Queue not full */
  HAL_FDCAN_FIFO_STATUS_FULL = FDCAN_TXFQS_TFQF, /*!< Tx FIFO/Queue full     */
} hal_fdcan_fifo_status_t;

/**
  * @brief  HAL FDCAN timeout counter status.
  */
typedef enum
{
  HAL_FDCAN_TIMEOUT_COUNTER_DISABLED = 0U,             /*!< Timeout counter disabled */
  HAL_FDCAN_TIMEOUT_COUNTER_ENABLED  = FDCAN_TOCC_ETOC /*!< Timeout counter enabled  */
} hal_fdcan_timeout_counter_status_t;

/**
  * @brief  HAL FDCAN interrupts status.
  */
typedef enum
{
  HAL_FDCAN_IT_DISABLED = 0U, /*!< Interrupt disabled */
  HAL_FDCAN_IT_ENABLED  = 1U, /*!< Interrupt enabled  */
} hal_fdcan_it_status_t;

/**
  * @brief  HAL FDCAN interrupt lines status.
  */
typedef enum
{
  HAL_FDCAN_IT_LINE_DISABLED = 0U, /*!< Interrupt line disabled */
  HAL_FDCAN_IT_LINE_ENABLED  = 1U, /*!< Interrupt line enabled  */
} hal_fdcan_it_lines_status_t;

/**
  * @brief  HAL FDCAN Tx buffer transmission complete interrupt status.
  */
typedef enum
{
  HAL_FDCAN_IT_TX_BUFFER_CPLT_DISABLED = 0U, /*!< Tx buffer transmission complete interrupt disabled */
  HAL_FDCAN_IT_TX_BUFFER_CPLT_ENABLED  = 1U, /*!< Tx buffer transmission complete interrupt enabled  */
} hal_fdcan_it_tx_buffer_complete_status_t;

/**
  * @brief  HAL FDCAN Tx buffer abort finished interrupt status.
  */
typedef enum
{
  HAL_FDCAN_IT_TX_BUFFER_ABORT_DISABLED  = 0U, /*!< Tx buffer abort finished interrupt disabled */
  HAL_FDCAN_IT_TX_BUFFER_ABORT_ENABLED   = 1U, /*!< Tx buffer abort finished interrupt enabled  */
} hal_fdcan_it_tx_buffer_abort_status_t;

/**
  * @brief  HAL FDCAN warning status.
  */
typedef enum
{
  HAL_FDCAN_WARNING_ERROR_ALL_COUNTERS_UNDER_LIMIT = 0U,          /*!< Both error counters are below
                                                                       the error_warning of 96       */
  HAL_FDCAN_WARNING_ERROR_ANY_COUNTER_OVER_LIMIT   = FDCAN_PSR_EW /*!< At least one of the error counter has reached
                                                                       the error_warning limit of 96 */
} hal_fdcan_warning_status_t;

/**
  * @brief  HAL FDCAN bus_off status.
  */
typedef enum
{
  HAL_FDCAN_BUS_OFF_FLAG_RESET = 0U,          /*!< The FDCAN is not in bus_off state */
  HAL_FDCAN_BUS_OFF_FLAG_SET   = FDCAN_PSR_BO /*!< The FDCAN is in bus_off state     */
} hal_fdcan_bus_off_status_t;

/**
  * @brief  HAL FDCAN  ESI (Error Status Indicator) flag of last received FDCAN message.
  */
typedef enum
{
  HAL_FDCAN_ESI_FLAG_RESET = 0U,            /*!< Last received FDCAN message did not have its ESI flag set */
  HAL_FDCAN_ESI_FLAG_SET   = FDCAN_PSR_RESI /*!< Last received FDCAN message has its ESI flag set          */
} hal_fdcan_esi_flag_status_t;

/**
  * @brief  HAL FDCAN BSR (Bit Rate Switch) flag of last received FDCAN message.
  */
typedef enum
{
  HAL_FDCAN_BRS_FLAG_RESET = 0U,             /*!< Last received FDCAN message did not have its BRS flag set */
  HAL_FDCAN_BRS_FLAG_SET   = FDCAN_PSR_RBRS, /*!< Last received FDCAN message has its BRS flag set          */
} hal_fdcan_brs_flag_status_t;

/**
  * @brief  HAL FDCAN received EDL (Extended Data length) message.
  */
typedef enum
{
  HAL_FDCAN_EDL_FLAG_RESET = 0U,            /*!< No FDCAN message has been received since this bit was reset by CPU */
  HAL_FDCAN_EDL_FLAG_SET   = FDCAN_PSR_REDL /*!< Message in FDCAN format with EDL flag                              */
} hal_fdcan_edl_flag_status_t;

/**
  * @brief  HAL FDCAN error status definition.
  */
typedef enum
{
  HAL_FDCAN_ERROR_ACTIVE  = 0U,          /*!< Error active state  */
  HAL_FDCAN_ERROR_PASSIVE = FDCAN_PSR_EP /*!< Error passive state */
} hal_fdcan_protocol_error_status_t;

/**
  * @brief  HAL FDCAN filter type, defines the filtering method used for FDCAN standard and extended filters.
  * The case @ref HAL_FDCAN_FILTER_TYPE_RANGE_NO_EIDM must be processed differently depending
  * on standard or extended filter.
  */
typedef enum
{
  HAL_FDCAN_FILTER_TYPE_RANGE         = 0U,           /*!< Range filter from FilterID1 to FilterID2             */
  HAL_FDCAN_FILTER_TYPE_DUAL          = (1UL << 30U), /*!< Dual ID filter for FilterID1 or FilterID2            */
  HAL_FDCAN_FILTER_TYPE_CLASSIC       = (2UL << 30U), /*!< Classic filter: FilterID1 = filter, FilterID2 = mask */
  HAL_FDCAN_FILTER_TYPE_RANGE_NO_EIDM = (3UL << 30U)  /*!< Range filter from FilterID1 to FilterID2,
                                                           EIDM mask not applied                                */
} hal_fdcan_filter_type_t;

/**
  * @brief  HAL FDCAN kernel clock divider.
  */
typedef enum
{
  HAL_FDCAN_CLOCK_DIV_1  = 0UL,  /*!< Divide kernel clock by 1  */
  HAL_FDCAN_CLOCK_DIV_2  = 1UL,  /*!< Divide kernel clock by 2  */
  HAL_FDCAN_CLOCK_DIV_4  = 2UL,  /*!< Divide kernel clock by 4  */
  HAL_FDCAN_CLOCK_DIV_6  = 3UL,  /*!< Divide kernel clock by 6  */
  HAL_FDCAN_CLOCK_DIV_8  = 4UL,  /*!< Divide kernel clock by 8  */
  HAL_FDCAN_CLOCK_DIV_10 = 5UL,  /*!< Divide kernel clock by 10 */
  HAL_FDCAN_CLOCK_DIV_12 = 6UL,  /*!< Divide kernel clock by 12 */
  HAL_FDCAN_CLOCK_DIV_14 = 7UL,  /*!< Divide kernel clock by 14 */
  HAL_FDCAN_CLOCK_DIV_16 = 8UL,  /*!< Divide kernel clock by 16 */
  HAL_FDCAN_CLOCK_DIV_18 = 9UL,  /*!< Divide kernel clock by 18 */
  HAL_FDCAN_CLOCK_DIV_20 = 10UL, /*!< Divide kernel clock by 20 */
  HAL_FDCAN_CLOCK_DIV_22 = 11UL, /*!< Divide kernel clock by 22 */
  HAL_FDCAN_CLOCK_DIV_24 = 12UL, /*!< Divide kernel clock by 24 */
  HAL_FDCAN_CLOCK_DIV_26 = 13UL, /*!< Divide kernel clock by 26 */
  HAL_FDCAN_CLOCK_DIV_28 = 14UL, /*!< Divide kernel clock by 28 */
  HAL_FDCAN_CLOCK_DIV_30 = 15UL, /*!< Divide kernel clock by 30 */
} hal_fdcan_clock_divider_t;

/**
  * @brief  HAL FDCAN Tx FIFO/queue mode.
  */
typedef enum
{
  HAL_FDCAN_TX_MODE_FIFO  = 0U,              /*!< Tx FIFO mode  */
  HAL_FDCAN_TX_MODE_QUEUE = FDCAN_TXBC_TFQM, /*!< Tx queue mode */
} hal_fdcan_tx_mode_t;

/**
  * @brief  HAL FDCAN high priority storage.
  */
typedef enum
{
  HAL_FDCAN_HP_MSG_FIFO_NONE = 0U,                            /*!< No FIFO selected         */
  HAL_FDCAN_HP_MSG_FIFO_LOST = (0x1UL << FDCAN_HPMS_MSI_Pos), /*!< FIFO message lost        */
  HAL_FDCAN_HP_MSG_RX_FIFO_0 = (0x2UL << FDCAN_HPMS_MSI_Pos), /*!< Message stored in FIFO 0 */
  HAL_FDCAN_HP_MSG_RX_FIFO_1 = FDCAN_HPMS_MSI,                /*!< Message stored in FIFO 1 */
} hal_fdcan_high_prio_message_storage_t;

/**
  * @brief  HAL FDCAN protocol exception event.
  */
typedef enum
{
  HAL_FDCAN_PROTOCOL_NO_EVENT_OCCURRED = 0U,           /*!< No protocol event occurred since last read access */
  HAL_FDCAN_PROTOCOL_EVENT_OCCURRED    = FDCAN_PSR_PXE /*!< Protocol event occurred                           */
} hal_fdcan_protocol_exception_event_t;

/**
  * @brief  HAL FDCAN filter Configuration.
  */
typedef enum
{
  HAL_FDCAN_FILTER_DISABLE         = 0U, /*!< Disable filter element                                          */
  HAL_FDCAN_FILTER_TO_RX_FIFO_0    = 1U, /*!< Store in Rx FIFO 0 if filter matches                            */
  HAL_FDCAN_FILTER_TO_RX_FIFO_1    = 2U, /*!< Store in Rx FIFO 1 if filter matches                            */
  HAL_FDCAN_FILTER_REJECT          = 3U, /*!< Reject ID if filter matches                                     */
  HAL_FDCAN_FILTER_HP              = 4U, /*!< Set high priority if filter matches                             */
  HAL_FDCAN_FILTER_TO_RX_FIFO_0_HP = 5U, /*!< Set high priority and store in FIFO 0 if filter matches         */
  HAL_FDCAN_FILTER_TO_RX_FIFO_1_HP = 6U, /*!< Set high priority and store in FIFO 1 if filter matches         */
} hal_fdcan_filter_config_t;

/**
  * @brief  HAL FDCAN Rx location.
  */
typedef enum
{
  HAL_FDCAN_RX_FIFO_0    = 0U, /*!< Index for access to Rx FIFO 0    */
  HAL_FDCAN_RX_FIFO_1    = 1U, /*!< Index for access to Rx FIFO 1    */
} hal_fdcan_rx_location_t;

/**
  * @brief  HAL FDCAN protocol error code.
  */
typedef enum
{
  HAL_FDCAN_PROTOCOL_ERROR_NONE      = 0U, /*!< No error occurred         */
  HAL_FDCAN_PROTOCOL_ERROR_STUFF     = 1U, /*!< Stuff error               */
  HAL_FDCAN_PROTOCOL_ERROR_FORM      = 2U, /*!< Form error                */
  HAL_FDCAN_PROTOCOL_ERROR_ACK       = 3U, /*!< Acknowledge error         */
  HAL_FDCAN_PROTOCOL_ERROR_BIT1      = 4U, /*!< Bit 1 (recessive) error   */
  HAL_FDCAN_PROTOCOL_ERROR_BIT0      = 5U, /*!< Bit 0 (dominant) error    */
  HAL_FDCAN_PROTOCOL_ERROR_CRC       = 6U, /*!< CRC check sum error       */
  HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE = 7U, /*!< No change since last read */
} hal_fdcan_protocol_error_code_t;

/**
  * @brief  HAL FDCAN communication state.
  */
typedef enum
{
  HAL_FDCAN_COM_STATE_SYNC = 0U,                           /*!< Node is synchronizing on CAN communication */
  HAL_FDCAN_COM_STATE_IDLE = (0x1UL << FDCAN_PSR_ACT_Pos), /*!< Node is neither receiver nor transmitter   */
  HAL_FDCAN_COM_STATE_RX   = (0x2UL << FDCAN_PSR_ACT_Pos), /*!< Node is operating as receiver              */
  HAL_FDCAN_COM_STATE_TX   = FDCAN_PSR_ACT,                /*!< Node is operating as transmitter           */
} hal_fdcan_communication_state_t;

/**
  * @brief  HAL FDCAN non-matching frames acceptance rules.
  */
typedef enum
{
  HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0 = 0U, /*!< Accept non-matching frames to Rx FIFO0 */
  HAL_FDCAN_NO_MATCH_TO_RX_FIFO_1 = 1U, /*!< Accept non-matching frames to Rx FIFO1 */
  HAL_FDCAN_NO_MATCH_REJECT       = 2U, /*!< Reject non-matching frames             */
} hal_fdcan_non_matching_acceptance_rules_t;

/**
  * @brief  HAL FDCAN remote frames acceptance rules.
  */
typedef enum
{
  HAL_FDCAN_REMOTE_ACCEPT = 0U, /*!< Accept remote frames     */
  HAL_FDCAN_REMOTE_REJECT = 1U, /*!< Reject all remote frames */
} hal_fdcan_remote_acceptance_frame_t;

/**
  * @brief  HAL FDCAN timestamp prescaler.
  */
typedef enum
{
  HAL_FDCAN_TIMESTAMP_PRESC_1  = 0U,                            /*!< Timestamp counter time unit is x1 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_2  = (0x1UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x2 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_3  = (0x2UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x3 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_4  = (0x3UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x4 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_5  = (0x4UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x5 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_6  = (0x5UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x6 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_7  = (0x6UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x7 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_8  = (0x7UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x8 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_9  = (0x8UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x9 CAN bit time   */
  HAL_FDCAN_TIMESTAMP_PRESC_10 = (0x9UL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x10 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_11 = (0xAUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x11 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_12 = (0xBUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x12 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_13 = (0xCUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x13 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_14 = (0xDUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x14 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_15 = (0xEUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x15 CAN bit time  */
  HAL_FDCAN_TIMESTAMP_PRESC_16 = (0xFUL << FDCAN_TSCC_TCP_Pos), /*!< Timestamp counter time unit is x16 CAN bit time  */
} hal_fdcan_timestamp_prescaler_t;

/**
  * @brief  HAL FDCAN Tx FIFO free level.
  */
typedef enum
{
  HAL_FDCAN_TX_FIFO_FREE_LEVEL_0  = 0U,  /*!< Tx FIFO Full - no free FIFO slot        */
  HAL_FDCAN_TX_FIFO_FREE_LEVEL_1  = 1U,  /*!< 1 free FIFO slot available in Tx FIFO   */
  HAL_FDCAN_TX_FIFO_FREE_LEVEL_2  = 2U,  /*!< 2 free FIFO slots available in Tx FIFO  */
  HAL_FDCAN_TX_FIFO_FREE_LEVEL_3  = 3U,  /*!< 3 free FIFO slots available in Tx FIFO  */
} hal_fdcan_tx_fifo_free_level_t;

/**
  * @brief  HAL FDCAN timeout operation.
  */
typedef enum
{
  HAL_FDCAN_TIMEOUT_CONTINUOUS     = 0U,                          /*!< Timeout continuous operation        */
  HAL_FDCAN_TIMEOUT_TX_EVENT_FIFO  = (1UL << FDCAN_TOCC_TOS_Pos), /*!< Timeout controlled by Tx event FIFO */
  HAL_FDCAN_TIMEOUT_RX_FIFO_0      = (2UL << FDCAN_TOCC_TOS_Pos), /*!< Timeout controlled by Rx FIFO 0     */
  HAL_FDCAN_TIMEOUT_RX_FIFO_1      = (3UL << FDCAN_TOCC_TOS_Pos), /*!< Timeout controlled by Rx FIFO 1     */
} hal_fdcan_timeout_operation_t;

/**
  * @brief  HAL FDCAN instance definition.
  */
typedef enum
{
  HAL_FDCAN1 = FDCAN1_BASE, /*!< Peripheral instance FDCAN1 */
#if defined(FDCAN2)
  HAL_FDCAN2 = FDCAN2_BASE, /*!< Peripheral instance FDCAN2 */
#endif /* FDCAN2 */
} hal_fdcan_t;

/**
  * @brief  HAL FDCAN Rx element header definition.
  */
typedef union
{
  uint64_t                            d64;                        /*!< 64-bit FDCAN Rx element header                 */
  struct
  {
    uint32_t                          identifier            : 29; /*!< Received identifier. A standard identifier
                                                                       is stored in bits [28:18]                      */
    hal_fdcan_frame_type_t            frame_type            : 1;  /*!< Frame type                                     */
    hal_fdcan_id_type_t               identifier_type       : 1;  /*!< Received identifier type                       */
    hal_fdcan_error_state_indicator_t error_state_indicator : 1;  /*!< Error state indicator                          */
    uint32_t                          rx_timestamp          : 16; /*!< Rx timestamp. Timestamp counter value is
                                                                       captured at the start of frame reception.
                                                                       Resolution depends on the timestamp counter
                                                                       prescaler                                      */
    hal_fdcan_data_length_code_t      data_length           : 4;  /*!< Received frame data length code                */
    hal_fdcan_bit_rate_switch_t       bit_rate_switch       : 1;  /*!< Bit rate switch indicator                      */
    hal_fdcan_header_frame_format_t   frame_format          : 1;  /*!< Received frame format                          */
    uint32_t                                                : 2;  /*!< Rx reserved                                    */
    uint32_t                          filter_index          : 7;  /*!< Filter index.
                                                                       - 0-127: index of matching Rx acceptance
                                                                       filter element, invalid if the received frame
                                                                       did not match any Rx filter element:
                                                                       @ref hal_fdcan_rx_header_t::frame_status
                                                                       is @ref HAL_FDCAN_RX_FRAME_NOT_MATCHED
                                                                       - Range is:
                                                                       0 to @ref hal_fdcan_config_t::std_filters_nbr or
                                                                       0 to @ref hal_fdcan_config_t::ext_filters_nbr  */
    hal_fdcan_rx_frame_status_t       frame_status          : 1;  /*!< Bit fields of the received frame status.
                                                                       It is useful when the Acceptance rule for
                                                                       non-matching frames is enabled by selecting
                                                                       @ref HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0
                                                                       or HAL_FDCAN_NO_MATCH_TO_RX_FIFO_1             */
  } b;                                                            /*!< Bit fields of the 64-bit Rx header             */
} hal_fdcan_rx_header_t;

/**
  * @brief  HAL FDCAN Tx element header definition.
  */
typedef union
{
  uint64_t                            d64;                           /*!< 64-bit FDCAN Tx element header              */
  struct
  {
    uint32_t                          identifier               : 29; /*!< Frame identifier. A standard identifier
                                                                          is stored in bits [28:18]                   */
    hal_fdcan_frame_type_t            frame_type               : 1;  /*!< Frame type                                  */
    hal_fdcan_id_type_t               identifier_type          : 1;  /*!< Identifier type                             */
    hal_fdcan_error_state_indicator_t error_state_indicator    : 1;  /*!< Error state indicator                       */
    uint32_t                                                   : 16; /*!< Tx reserved field                           */
    hal_fdcan_data_length_code_t      data_length              : 4;  /*!< Data length code                            */
    hal_fdcan_bit_rate_switch_t       bit_rate_switch          : 1;  /*!< Bit rate switch                             */
    hal_fdcan_header_frame_format_t   frame_format             : 1;  /*!< Frame format                                */
    uint32_t                                                   : 1;  /*!< Tx reserved bit                             */
    hal_fdcan_tx_event_fifo_t         event_fifo_control       : 1;  /*!< Event FIFO control                          */
    uint32_t                          message_marker           : 8;  /*!< Message marker.
                                                                          Written by CPU during Tx buffer configuration.
                                                                          Copied into Tx event FIFO element
                                                                          for identification of Tx message status     */
  } b;                                                               /*!< Bit fields of the 64-bit Tx header          */
} hal_fdcan_tx_header_t;

/**
  * @brief  HAL FDCAN Tx Event FIFO element header definition.
  */
typedef union
{
  uint64_t                            d64;                         /*!< 64-bit FDCAN Tx Event FIFO element header     */
  struct
  {
    uint32_t                          identifier            : 29; /*!< Frame identifier, standard or extended.
                                                                       A standard identifier is stored into ID[28:18]
                                                                       field of Rx element                            */
    hal_fdcan_frame_type_t            frame_type            : 1;  /*!< Frame type                                     */
    hal_fdcan_id_type_t               identifier_type       : 1;  /*!< Identifier type                                */
    hal_fdcan_error_state_indicator_t error_state_indicator : 1;  /*!< Error state indicator                          */
    uint32_t                          tx_timestamp          : 16; /*!< Tx timestamp. The timestamp counter value is
                                                                       captured on start of frame transmission.
                                                                       Resolution depends on the configuration of the
                                                                       timestamp counter prescaler                    */
    hal_fdcan_data_length_code_t      data_length           : 4;  /*!< Data length code                               */
    hal_fdcan_bit_rate_switch_t       bit_rate_switch       : 1;  /*!< Bit rate switch                                */
    hal_fdcan_header_frame_format_t   frame_format          : 1;  /*!< Frame format                                   */
    hal_fdcan_tx_event_type_t         event_type            : 2;  /*!< Event type                                     */
    uint32_t                          message_marker        : 8;  /*!< Message marker.
                                                                       Copied from Tx event FIFO element for
                                                                       identification of Tx message status            */
  } b;                                                            /*!< Bit fields of the 64-bit Tx event FIFO header  */
} hal_fdcan_tx_evt_fifo_header_t;

/**
  * @brief  HAL FDCAN global filter parameters.
  */
typedef struct
{
  hal_fdcan_non_matching_acceptance_rules_t acceptance_non_matching_std; /*!< Acceptance rule for standard non-matching
                                                                              frames */
  hal_fdcan_non_matching_acceptance_rules_t acceptance_non_matching_ext; /*!< Acceptance rule for extended non-matching
                                                                              frames */
  hal_fdcan_remote_acceptance_frame_t       acceptance_remote_std;       /*!< Acceptance rule for standard remote
                                                                              frames */
  hal_fdcan_remote_acceptance_frame_t       acceptance_remote_ext;       /*!< Acceptance rule for extended remote
                                                                              frames */
} hal_fdcan_global_filter_config_t;

/**
  * @brief  HAL FDCAN timestamp parameters.
  */
typedef struct
{
  hal_fdcan_timestamp_source_t    timestamp_source;    /*!< Timestamp source                         */
  hal_fdcan_timestamp_prescaler_t timestamp_prescaler; /*!< Value of the timestamp prescaler counter */
} hal_fdcan_timestamp_config_t;

/**
  * @brief  HAL FDCAN timeout parameters.
  */
typedef struct
{
  hal_fdcan_timeout_operation_t timeout_operation; /*!< Timeout select                              */
  uint32_t                      timeout_period;    /*!< Value of the timeout counter (down-counter) */
} hal_fdcan_timeout_config_t;

/**
  * @brief  HAL FDCAN Tx delay compensation parameters.
  */
typedef struct
{
  uint32_t tx_delay_comp_offset;     /*!< Transmitter delay compensation offset between 0 and 0x7F               */
  uint32_t tx_delay_comp_win_length; /*!< Transmitter delay compensation filter window length between 0 and 0x7F */
} hal_fdcan_tx_delay_comp_config_t;

/**
  * @brief  HAL FDCAN filter structure definition.
  */
typedef struct
{
  hal_fdcan_id_type_t          id_type;            /*!< Specifies the identifier type.                                */
  uint32_t                     filter_index;       /*!< Specifies the filter index. The range of this parameter depends
                                                        on the filter identifier type: @ref hal_fdcan_filter_t::id_type.
                                                        - if @ref HAL_FDCAN_ID_STANDARD :
                                                          value between 0 and 27
                                                        - if @ref HAL_FDCAN_ID_EXTENDED :
                                                          value between 0 and 7 */
  hal_fdcan_filter_type_t      filter_type;        /*!< Specifies the filter type.
                                                        @ref HAL_FDCAN_FILTER_TYPE_RANGE_NO_EIDM
                                                        is permitted only when @ref hal_fdcan_filter_t::id_type is
                                                        @ref HAL_FDCAN_ID_EXTENDED                                    */
  hal_fdcan_filter_config_t    filter_config;      /*!< Specifies the filter configuration                            */
  uint32_t                     filter_id1;         /*!< Specifies the filter first identifier.
                                                        The range of this parameter depends on the configured
                                                        @ref hal_fdcan_filter_t::id_type, if:
                                                        - @ref HAL_FDCAN_ID_STANDARD : value between 0 and 0x7FF,
                                                        - @ref HAL_FDCAN_ID_EXTENDED : value between 0 and 0x1FFFFFFF */
  uint32_t                     filter_id2;         /*!< Specifies the filter second identifier.
                                                        The range of this parameter depends on the configured
                                                        @ref hal_fdcan_filter_t::id_type, if:
                                                        - @ref HAL_FDCAN_ID_STANDARD : value between 0 and 0x7FF,
                                                        - @ref HAL_FDCAN_ID_EXTENDED : value between 0 and 0x1FFFFFFF */
} hal_fdcan_filter_t;

/**
  * @brief  HAL FDCAN high priority message status structure definition.
  */
typedef struct
{
  hal_fdcan_high_prio_filter_list_t     filter_list;             /*!< Filter list of the matching filter elements     */
  uint32_t                              filter_index;            /*!< Index of matching filter element. This index only
                                                                      correspond to a previously configured filter    */
  hal_fdcan_high_prio_message_storage_t message_location_status; /*!< High priority message storage                   */
  uint32_t                              message_index;           /*!< Index of Rx FIFO element to which the message was
                                                                     stored. This parameter is valid only when
                                                                     @ref HAL_FDCAN_HP_MSG_RX_FIFO_0 or
                                                                     @ref HAL_FDCAN_HP_MSG_RX_FIFO_1
                                                                     are selected as high priority message storage    */
} hal_fdcan_high_prio_msg_status_t;

/**
  * @brief  HAL FDCAN protocol status structure definition.
  */
typedef struct
{
  hal_fdcan_protocol_error_code_t      last_error_code;      /*!< Type of the last error that occurred on the FDCAN
                                                                   bus                                                */
  hal_fdcan_protocol_error_code_t      data_last_error_code; /*!< Type of the last error that occurred in the data
                                                                  phase of a CAN FD format frame with its bitrate
                                                                  switching flag set                                  */
  hal_fdcan_communication_state_t      activity;             /*!< Communication state                                 */
  hal_fdcan_protocol_error_status_t    error_status;         /*!< Error status                                        */
  hal_fdcan_warning_status_t           error_warning;        /*!< Warning status                                      */
  hal_fdcan_bus_off_status_t           bus_off;              /*!< Bus off status                                      */
  hal_fdcan_esi_flag_status_t          rx_esi_flag;          /*!< Error state indicator flag of last received
                                                                  CAN FD message                                      */
  hal_fdcan_brs_flag_status_t          rx_brs_flag;          /*!< Switching flag of last received CAN FD message      */
  hal_fdcan_edl_flag_status_t          rx_fdf_flag;          /*!< Specifies if CAN FD message (FDF flag set) has been
                                                                  received since last protocol status                 */
  hal_fdcan_protocol_exception_event_t protocol_exception;   /*!< Protocol exception status                           */
  uint32_t                             tdc_value;            /*!< Transmitter Delay Compensation value.
                                                                  This parameter can be a number between 0 and 127    */
} hal_fdcan_protocol_status_t;

/**
  * @brief  HAL FDCAN error counters structure definition.
  */
typedef struct
{
  uint32_t                           tx_error_cnt;            /*!< Specifies the transmit error counter value.
                                                                   This parameter can be a number between 0 and 255   */
  uint32_t                           rx_error_cnt;            /*!< Specifies the receive error counter value.
                                                                   This parameter can be a number between 0 and 127   */
  hal_fdcan_rx_error_passive_level_t rx_error_passive_status; /*!< Specifies the receive error passive status         */
  uint32_t                           global_cnt;              /*!< Specifies the transmit/receive error logging
                                                                   counter value. This parameter can be a number
                                                                   between 0 and 255. This counter is incremented
                                                                   each time when a FDCAN protocol error causes the
                                                                   @ref hal_fdcan_error_counters_t::tx_error_cnt or the
                                                                   @ref hal_fdcan_error_counters_t::rx_error_cnt to be
                                                                   incremented.
                                                                   The counter stops at 255, the next increment of
                                                                   @ref hal_fdcan_error_counters_t::tx_error_cnt or
                                                                   @ref hal_fdcan_error_counters_t::rx_error_cnt sets
                                                                   the flag of interrupt
                                                                   @ref HAL_FDCAN_IT_ERROR_LOGGING_OVERFLOW           */
} hal_fdcan_error_counters_t;

/**
  * @brief  HAL FDCAN message RAM blocks.
  */
typedef struct
{
  uint32_t std_filter_start_addr;  /*!< Specifies the standard filter list start address */
  uint32_t ext_filter_start_addr;  /*!< Specifies the extended filter list start address */
  uint32_t rx_fifo0_start_addr;    /*!< Specifies the Rx FIFO 0 start address            */
  uint32_t rx_fifo1_start_addr;    /*!< Specifies the Rx FIFO 1 start address            */
  uint32_t tx_event_start_addr;    /*!< Specifies the Tx event FIFO start address        */
  uint32_t tx_fifo_start_addr;     /*!< Specifies the Tx FIFO/queue start address        */
} hal_fdcan_msg_ram_address_t;

/**
  * @brief  HAL FDCAN nominal bit timing structure definition.
  */
typedef struct
{
  uint32_t nominal_prescaler;  /*!< Specifies the value by which the oscillator frequency is divided for generating
                                    the nominal bit time quanta. This parameter must be a number between 1 and 512    */
  uint32_t nominal_jump_width; /*!< Specifies the maximum number of time quanta the FDCAN hardware is allowed to
                                    lengthen or shorten a bit to perform resynchronization.
                                    This parameter must be a number between 1 and 128                                 */
  uint32_t nominal_time_seg1;  /*!< Specifies the number of time quanta in bit segment 1.
                                    This parameter must be a number between 2 and 256                                 */
  uint32_t nominal_time_seg2;  /*!< Specifies the number of time quanta in bit segment 2.
                                    This parameter must be a number between 2 and 128                                 */
} hal_fdcan_nominal_bit_timing_t;

/**
  * @brief  HAL FDCAN data bit timing structure definition.
  */
typedef struct
{
  uint32_t data_prescaler;  /*!< Specifies the value by which the oscillator frequency is divided for generating
                                 the data bit time quanta. This parameter must be a number between 1 and 32           */
  uint32_t data_jump_width; /*!< Specifies the maximum number of time quanta the FDCAN hardware is allowed to
                                 lengthen or shorten a data bit to perform resynchronization.
                                 This parameter must be a number between 1 and 16                                     */
  uint32_t data_time_seg1;  /*!< Specifies the number of time quanta in data bit segment 1.
                                 This parameter must be a number between 1 and 32                                     */
  uint32_t data_time_seg2;  /*!< Specifies the number of time quanta in Data Bit Segment 2.
                                 This parameter must be a number between 1 and 16                                     */
} hal_fdcan_data_bit_timing_t;

/**
  * @brief  HAL FDCAN global configuration structure definition.
  */
typedef struct
{
  hal_fdcan_nominal_bit_timing_t        nominal_bit_timing;      /*!< Nominal bit timing                              */
  hal_fdcan_data_bit_timing_t           data_bit_timing;         /*!< Data bit Timing                                 */
  hal_fdcan_mode_t                      mode;                    /*!< Mode                                            */
  hal_fdcan_frame_format_t              frame_format;            /*!< Frame format                                    */
  hal_fdcan_auto_retransmission_state_t auto_retransmission;     /*!< Automatic retransmission feature                */
  hal_fdcan_transmit_pause_state_t      transmit_pause;          /*!< Transmit pause feature                          */
  hal_fdcan_protocol_exception_state_t  protocol_exception;      /*!< Protocol exception handling                     */
  uint32_t                              std_filters_nbr;         /*!< Number of standard message ID filters.
                                                                   This parameter must be a number between 0 and 28   */
  uint32_t                              ext_filters_nbr;         /*!< Number of extended message ID filters.
                                                                   This parameter must be a number between 0 and 8    */
  hal_fdcan_tx_mode_t                   tx_fifo_queue_mode;      /*!< Tx FIFO/queue mode selection                    */
} hal_fdcan_config_t;

typedef struct hal_fdcan_handle_s hal_fdcan_handle_t; /*!< FDCAN handle structure type */

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U)
/**
  * @brief  HAL FDCAN callback pointer definition.
  */

typedef void (*hal_fdcan_fifo_cb_t)(hal_fdcan_handle_t *hfdcan,
                                    uint32_t interrupts_list);      /*!< Pointer to interrupts list FDCAN callback
                                                                         function                                     */
typedef void (*hal_fdcan_tx_buffer_cb_t)(hal_fdcan_handle_t *hfdcan,
                                         uint32_t tx_buffers_idx);  /*!< Pointer to interrupts Tx buffer
                                                                         complete/abort FDCAN callback function       */
typedef void (*hal_fdcan_cb_t)(hal_fdcan_handle_t *hfdcan);         /*!< Pointer to a generic FDCAN callback function */

#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */

/**
  * @brief  HAL FDCAN handle structure definition.
  */
struct hal_fdcan_handle_s
{
  hal_fdcan_t                    instance;                  /*!< FDCAN instance                                  */

  hal_fdcan_msg_ram_address_t    msg_ram;                   /*!< Message RAM blocks                              */
  volatile hal_fdcan_state_t     global_state;              /*!< Communication current state                     */

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
  volatile uint32_t              last_error_codes;          /*!< Last error codes                                */
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

  uint32_t                       latest_tx_fifo_q_request;  /*!< Tx buffer index of latest Tx FIFO/queue request */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t             semaphore;                 /*!< FDCAN OS semaphore                              */
#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_FDCAN_USER_DATA) && (USE_HAL_FDCAN_USER_DATA == 1)
  const void                    *p_user_data;               /*!< User data pointer                               */
#endif /* USE_HAL_FDCAN_USER_DATA */

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U)
  hal_fdcan_fifo_cb_t           p_tx_event_fifo_cb;         /*!< Tx event FIFO callback                          */
  hal_fdcan_fifo_cb_t           p_rx_fifo_0_cb;             /*!< Rx FIFO 0 callback                              */
  hal_fdcan_fifo_cb_t           p_rx_fifo_1_cb;             /*!< Rx FIFO 1 callback                              */
  hal_fdcan_cb_t                p_tx_fifo_empty_cb;         /*!< Tx FIFO empty callback                          */
  hal_fdcan_tx_buffer_cb_t      p_tx_buffer_complete_cb;    /*!< Tx buffer complete callback                     */
  hal_fdcan_tx_buffer_cb_t      p_tx_buffer_abort_cb;       /*!< Tx buffer abort callback                        */
  hal_fdcan_cb_t                p_high_priority_msg_cb;     /*!< High priority message callback                  */
  hal_fdcan_cb_t                p_ts_wraparound_cb;         /*!< Timestamp wraparound callback                   */
  hal_fdcan_cb_t                p_error_cb;                 /*!< Error callback                                  */
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
};

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Exported_Functions HAL FDCAN Functions
  * @{
  */

/** @defgroup FDCAN_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

hal_status_t HAL_FDCAN_Init(hal_fdcan_handle_t *hfdcan, hal_fdcan_t instance);
void         HAL_FDCAN_DeInit(hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

/** @defgroup FDCAN_Exported_Functions_Group2 Power management functions
  * @{
  */

/* Power down mode */
hal_status_t HAL_FDCAN_EnterPowerDownMode(hal_fdcan_handle_t *hfdcan);
hal_status_t HAL_FDCAN_ExitPowerDownMode(hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

/** @defgroup FDCAN_Exported_Functions_Group3 Configuration functions
  * @{
  */

/* Global configuration */
hal_status_t HAL_FDCAN_SetConfig(hal_fdcan_handle_t *hfdcan, const hal_fdcan_config_t *p_config);
void         HAL_FDCAN_GetConfig(const hal_fdcan_handle_t *hfdcan, hal_fdcan_config_t *p_config);

/* Nominal bit timing */
hal_status_t HAL_FDCAN_SetNominalBitTiming(const hal_fdcan_handle_t *hfdcan,
                                           const hal_fdcan_nominal_bit_timing_t *p_nominal_bit_timing);
void         HAL_FDCAN_GetNominalBitTiming(const hal_fdcan_handle_t *hfdcan,
                                           hal_fdcan_nominal_bit_timing_t *p_nominal_bit_timing);

/* Data bit timing */
hal_status_t HAL_FDCAN_SetDataBitTiming(const hal_fdcan_handle_t *hfdcan,
                                        const hal_fdcan_data_bit_timing_t *p_data_bit_timing);
void HAL_FDCAN_GetDataBitTiming(const hal_fdcan_handle_t *hfdcan, hal_fdcan_data_bit_timing_t *p_data_bit_timing);

/* Filter - To set/get the acceptance filters parameters for standard and extended filters */
hal_status_t HAL_FDCAN_SetFilter(const hal_fdcan_handle_t *hfdcan, const hal_fdcan_filter_t *p_filter_config);
void         HAL_FDCAN_GetFilter(const hal_fdcan_handle_t *hfdcan, hal_fdcan_filter_t *p_filter_config,
                                 uint32_t filter_index, hal_fdcan_id_type_t id_type);

/* Global filter configuration */
hal_status_t HAL_FDCAN_SetGlobalFilter(const hal_fdcan_handle_t *hfdcan,
                                       const hal_fdcan_global_filter_config_t *p_global_filter_config);
void         HAL_FDCAN_GetGlobalFilter(const hal_fdcan_handle_t *hfdcan,
                                       hal_fdcan_global_filter_config_t *p_global_filter_config);

/* Extended ID mask for acceptance filtering */
hal_status_t HAL_FDCAN_SetExtendedIdMask(const hal_fdcan_handle_t *hfdcan, uint32_t mask);
uint32_t     HAL_FDCAN_GetExtendedIdMask(const hal_fdcan_handle_t *hfdcan);

/* Clock divider */
hal_status_t              HAL_FDCAN_SetClockDivider(const hal_fdcan_handle_t *hfdcan,
                                                    hal_fdcan_clock_divider_t clock_divider);
hal_fdcan_clock_divider_t HAL_FDCAN_GetClockDivider(const hal_fdcan_handle_t *hfdcan);

/* Rx FIFO overwrite */
hal_status_t HAL_FDCAN_SetRxFifoOverwrite(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          hal_fdcan_rx_fifo_mode_t operation_mode);
void         HAL_FDCAN_GetRxFifoOverwrite(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          hal_fdcan_rx_fifo_mode_t *p_operation_mode);

/* RAM watchdog */
hal_status_t HAL_FDCAN_SetRamWatchdog(const hal_fdcan_handle_t *hfdcan, uint8_t counter_start_value);
uint8_t      HAL_FDCAN_GetRamWatchdog(const hal_fdcan_handle_t *hfdcan);

/* Timestamp counter */
hal_status_t HAL_FDCAN_SetConfigTimestampCounter(const hal_fdcan_handle_t *hfdcan,
                                                 const hal_fdcan_timestamp_config_t *p_timestamp_config);
void         HAL_FDCAN_GetConfigTimestampCounter(const hal_fdcan_handle_t *hfdcan,
                                                 hal_fdcan_timestamp_config_t *p_timestamp_config);

/* Timeout counter */
hal_status_t HAL_FDCAN_SetConfigTimeoutCounter(const hal_fdcan_handle_t *hfdcan,
                                               const hal_fdcan_timeout_config_t *p_timeout_param);
void         HAL_FDCAN_GetConfigTimeoutCounter(const hal_fdcan_handle_t *hfdcan,
                                               hal_fdcan_timeout_config_t *p_timeout_param);
hal_status_t HAL_FDCAN_EnableTimeoutCounter(const hal_fdcan_handle_t *hfdcan);
hal_status_t HAL_FDCAN_DisableTimeoutCounter(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_timeout_counter_status_t HAL_FDCAN_IsEnabledTimeoutCounter(const hal_fdcan_handle_t *hfdcan);

/* Delay compensation mechanism to compensate the CAN transmitter loop delay */
hal_status_t HAL_FDCAN_SetConfigTxDelayCompensation(const hal_fdcan_handle_t *hfdcan,
                                                    const hal_fdcan_tx_delay_comp_config_t *p_tx_delay_param);
void         HAL_FDCAN_GetConfigTxDelayCompensation(const hal_fdcan_handle_t *hfdcan,
                                                    hal_fdcan_tx_delay_comp_config_t *p_tx_delay_param);

hal_status_t                     HAL_FDCAN_EnableTxDelayCompensation(const hal_fdcan_handle_t *hfdcan);
hal_status_t                     HAL_FDCAN_DisableTxDelayCompensation(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_tx_delay_comp_status_t HAL_FDCAN_IsEnabledTxDelayCompensation(const hal_fdcan_handle_t *hfdcan);

/* ISO mode - ISO11898-1 or CAN FD specification V1.0 */
hal_status_t                HAL_FDCAN_EnableISOMode(const hal_fdcan_handle_t *hfdcan);
hal_status_t                HAL_FDCAN_DisableISOMode(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_iso_mode_status_t HAL_FDCAN_IsEnabledISOMode(const hal_fdcan_handle_t *hfdcan);

/* Edge filtering */
hal_status_t                      HAL_FDCAN_EnableEdgeFiltering(const hal_fdcan_handle_t *hfdcan);
hal_status_t                      HAL_FDCAN_DisableEdgeFiltering(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_edge_filtering_status_t HAL_FDCAN_IsEnabledEdgeFiltering(const hal_fdcan_handle_t *hfdcan);

/* Operating mode */
hal_status_t     HAL_FDCAN_SetMode(const hal_fdcan_handle_t *hfdcan, hal_fdcan_mode_t mode);
hal_fdcan_mode_t HAL_FDCAN_GetMode(const hal_fdcan_handle_t *hfdcan);

/* Restricted operation mode */
hal_status_t                          HAL_FDCAN_EnableRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan);
hal_status_t                          HAL_FDCAN_DisableRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_restricted_op_mode_status_t HAL_FDCAN_IsEnabledRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan);

/* Frame format */
hal_status_t HAL_FDCAN_SetFrameFormat(const hal_fdcan_handle_t *hfdcan, hal_fdcan_frame_format_t frame_format);
hal_fdcan_frame_format_t HAL_FDCAN_GetFrameFormat(const hal_fdcan_handle_t *hfdcan);

/* Auto retransmission of unsuccessful messages */
hal_status_t                          HAL_FDCAN_EnableAutoRetransmission(const hal_fdcan_handle_t *hfdcan);
hal_status_t                          HAL_FDCAN_DisableAutoRetransmission(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_auto_retransmission_state_t HAL_FDCAN_IsEnabledAutoRetransmission(const hal_fdcan_handle_t *hfdcan);

/* Transmit pause */
hal_status_t                     HAL_FDCAN_EnableTransmitPause(const hal_fdcan_handle_t *hfdcan);
hal_status_t                     HAL_FDCAN_DisableTransmitPause(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_transmit_pause_state_t HAL_FDCAN_IsEnabledTransmitPause(const hal_fdcan_handle_t *hfdcan);

/* Protocol exception */
hal_status_t                         HAL_FDCAN_EnableProtocolException(const hal_fdcan_handle_t *hfdcan);
hal_status_t                         HAL_FDCAN_DisableProtocolException(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_protocol_exception_state_t HAL_FDCAN_IsEnabledProtocolException(const hal_fdcan_handle_t *hfdcan);

/* Tx mode */
hal_status_t        HAL_FDCAN_SetTxMode(const hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_mode_t tx_mode);
hal_fdcan_tx_mode_t HAL_FDCAN_GetTxMode(const hal_fdcan_handle_t *hfdcan);

/* FDCAN current clock frequency */
uint32_t HAL_FDCAN_GetClockFreq(const hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U)
/** @defgroup FDCAN_Exported_Functions_Group4 Callback registration functions
  * @{
  */

/* Callbacks registration */
hal_status_t HAL_FDCAN_RegisterTxEventFifoCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterRxFifo0Callback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterRxFifo1Callback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterTxFifoEmptyCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterTxBufferCompleteCallback(hal_fdcan_handle_t *hfdcan,
                                                        hal_fdcan_tx_buffer_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterTxBufferAbortCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_buffer_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterHighPriorityMessageCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterTimestampWraparoundCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback);
hal_status_t HAL_FDCAN_RegisterErrorCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback);

/**
  * @}
  */
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */

/** @defgroup FDCAN_Exported_Functions_Group5 Control functions
  * @{
  */

/* Start/Stop the FDCAN module */
hal_status_t HAL_FDCAN_Start(hal_fdcan_handle_t *hfdcan);
hal_status_t HAL_FDCAN_Stop(hal_fdcan_handle_t *hfdcan);

/* Functions involved in transmission process */
hal_status_t                   HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hal_fdcan_handle_t *hfdcan,
                                                                 const hal_fdcan_tx_header_t *p_tx_element_header,
                                                                 const uint8_t *p_tx_data);
hal_fdcan_fifo_status_t        HAL_FDCAN_GetTxFifoStatus(const hal_fdcan_handle_t *hfdcan);
uint32_t                       HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const hal_fdcan_handle_t *hfdcan);
hal_fdcan_tx_fifo_free_level_t HAL_FDCAN_GetTxFifoFreeLevel(const hal_fdcan_handle_t *hfdcan);
hal_status_t                   HAL_FDCAN_ReqAbortOfTxBuffer(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffer_idx);
hal_status_t                   HAL_FDCAN_GetTxEvent(const hal_fdcan_handle_t *hfdcan,
                                                    hal_fdcan_tx_evt_fifo_header_t *p_tx_event);
void                           HAL_FDCAN_GetTxEventFifoFillLevel(const hal_fdcan_handle_t *hfdcan,
                                                                 uint32_t *p_fill_level);
hal_fdcan_tx_buffer_status_t   HAL_FDCAN_GetTxBufferMessageStatus(const hal_fdcan_handle_t *hfdcan,
                                                                  uint32_t tx_buffer_idx);

/* Functions involved in receiving process */
hal_status_t HAL_FDCAN_GetReceivedMessage(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          hal_fdcan_rx_header_t *p_rx_header, uint8_t *p_rx_data);
void         HAL_FDCAN_GetRxFifoFillLevel(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          uint32_t *p_fill_level);

uint16_t     HAL_FDCAN_GetTimestampCounter(const hal_fdcan_handle_t *hfdcan);
hal_status_t HAL_FDCAN_ResetTimestampCounter(const hal_fdcan_handle_t *hfdcan);

uint16_t     HAL_FDCAN_GetTimeoutCounter(const hal_fdcan_handle_t *hfdcan);
hal_status_t HAL_FDCAN_ResetTimeoutCounter(const hal_fdcan_handle_t *hfdcan);

/* Functions involved in general Tx/Rx process */
hal_status_t HAL_FDCAN_GetHighPriorityMessageStatus(const hal_fdcan_handle_t *hfdcan,
                                                    hal_fdcan_high_prio_msg_status_t *p_hp_msg_status);
hal_status_t HAL_FDCAN_GetProtocolStatus(const hal_fdcan_handle_t *hfdcan,
                                         hal_fdcan_protocol_status_t *p_protocol_status);
hal_status_t HAL_FDCAN_GetErrorCounters(const hal_fdcan_handle_t *hfdcan, hal_fdcan_error_counters_t *p_error_counters);

/* Bus off error recovery  */
hal_status_t HAL_FDCAN_Recover(const hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

/** @defgroup FDCAN_Exported_Functions_Group6 Interrupts management functions
  * @{
  */

void HAL_FDCAN_IRQHandler(hal_fdcan_handle_t *hfdcan);
void HAL_FDCAN_Line0_IRQHandler(hal_fdcan_handle_t *hfdcan);
void HAL_FDCAN_Line1_IRQHandler(hal_fdcan_handle_t *hfdcan);

hal_status_t          HAL_FDCAN_EnableInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t interrupts);
hal_status_t          HAL_FDCAN_DisableInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t interrupts);
hal_fdcan_it_status_t HAL_FDCAN_IsEnabledInterrupt(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt);

hal_status_t HAL_FDCAN_EnableTxBufferCompleteInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
hal_status_t HAL_FDCAN_DisableTxBufferCompleteInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
hal_fdcan_it_tx_buffer_complete_status_t HAL_FDCAN_IsEnabledTxBufferCompleteInterrupt(const hal_fdcan_handle_t *hfdcan,
    uint32_t tx_buffer_idx);

hal_status_t HAL_FDCAN_EnableTxBufferCancellationInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
hal_status_t HAL_FDCAN_DisableTxBufferCancellationInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
hal_fdcan_it_tx_buffer_abort_status_t HAL_FDCAN_IsEnabledTxBufferCancellationInterrupt(const hal_fdcan_handle_t *hfdcan,
    uint32_t tx_buffer_idx);

hal_status_t HAL_FDCAN_SetInterruptGroupsToLine(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt_groups,
                                                uint32_t it_line);
uint32_t     HAL_FDCAN_GetLineFromInterruptGroup(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt_group);

hal_status_t                HAL_FDCAN_EnableInterruptLines(const hal_fdcan_handle_t *hfdcan, uint32_t it_lines);
hal_status_t                HAL_FDCAN_DisableInterruptLines(const hal_fdcan_handle_t *hfdcan, uint32_t it_lines);
hal_fdcan_it_lines_status_t HAL_FDCAN_IsEnabledInterruptLine(const hal_fdcan_handle_t *hfdcan, uint32_t it_line);

/**
  * @}
  */

/** @defgroup FDCAN_Exported_Functions_Group7 Weak callback functions
  * @{
  */

void HAL_FDCAN_TxEventFifoCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_event_fifo_interrupts);
void HAL_FDCAN_RxFifo0Callback(hal_fdcan_handle_t *hfdcan, uint32_t rx_fifo0_interrupts);
void HAL_FDCAN_RxFifo1Callback(hal_fdcan_handle_t *hfdcan, uint32_t rx_fifo1_interrupts);
void HAL_FDCAN_TxFifoEmptyCallback(hal_fdcan_handle_t *hfdcan);
void HAL_FDCAN_TxBufferCompleteCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
void HAL_FDCAN_TxBufferAbortCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx);
void HAL_FDCAN_HighPriorityMessageCallback(hal_fdcan_handle_t *hfdcan);
void HAL_FDCAN_TimestampWraparoundCallback(hal_fdcan_handle_t *hfdcan);
void HAL_FDCAN_ErrorCallback(hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

/** @defgroup FDCAN_Exported_Functions_Group8 Error and state functions
  * @{
  */

hal_fdcan_state_t HAL_FDCAN_GetState(const hal_fdcan_handle_t *hfdcan);

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
uint32_t HAL_FDCAN_GetLastErrorCodes(const hal_fdcan_handle_t *hfdcan);
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup FDCAN_Exported_Functions_Group9 Peripheral acquire/release/free the bus
  * @{
  */

hal_status_t HAL_FDCAN_AcquireBus(hal_fdcan_handle_t *hfdcan, uint32_t timeout_ms);
hal_status_t HAL_FDCAN_ReleaseBus(hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */

#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_FDCAN_USER_DATA) && (USE_HAL_FDCAN_USER_DATA == 1)
/** @defgroup FDCAN_Exported_Functions_Group10 User data Set/Get
  * @{
  */

void       HAL_FDCAN_SetUserData(hal_fdcan_handle_t *hfdcan, const void *p_user_data);
const void *HAL_FDCAN_GetUserData(const hal_fdcan_handle_t *hfdcan);

/**
  * @}
  */
#endif /* USE_HAL_FDCAN_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

#endif /* FDCAN1 || FDCAN2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_FDCAN_H */
