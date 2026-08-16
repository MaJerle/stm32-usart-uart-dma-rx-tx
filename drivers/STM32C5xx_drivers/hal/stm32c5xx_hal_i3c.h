/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_i3c.h
  * @brief   Header file of I3C HAL module.
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
#ifndef STM32C5XX_HAL_I3C_H
#define STM32C5XX_HAL_I3C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_i3c.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined (I3C1)

/** @defgroup I3C I3C
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Exported_Constants HAL I3C constants
  * @{
  */

/** @defgroup I3C_ERROR_CODE_DEFINITION Error code definition
  * @{
  */
#define HAL_I3C_ERROR_NONE            (0x00000000U) /*!< 0x0
                                      No error */

#define HAL_I3C_CTRL_ERROR_0          (LL_I3C_CONTROLLER_ERROR_CE0 | I3C_SER_PERR) /*!< 0x10 \
                                      Controller detected an illegally formatted CCC */

#define HAL_I3C_CTRL_ERROR_1          (LL_I3C_CONTROLLER_ERROR_CE1 | I3C_SER_PERR) /*!< 0x11 \
                                      Controller detected that transmitted data on the bus is different from expected */

#define HAL_I3C_CTRL_ERROR_2          (LL_I3C_CONTROLLER_ERROR_CE2 | I3C_SER_PERR) /*!< 0x12 \
                                      Controller detected an unacknowledged broadcast address 7'h7E  */

#define HAL_I3C_CTRL_ERROR_3          (LL_I3C_CONTROLLER_ERROR_CE3 | I3C_SER_PERR) /*!< 0x13
                                      Controller detected the new controller did not drive bus after controller-role
                                      hand-off */

#define HAL_I3C_TGT_ERROR_0           (LL_I3C_TARGET_ERROR_TE0 | I3C_SER_PERR) /*!< 0x18 \
                                      Target detected an invalid broadcast address 7'h7E + W */

#define HAL_I3C_TGT_ERROR_1           (LL_I3C_TARGET_ERROR_TE1 | I3C_SER_PERR) /*!< 0x19 \
                                      Target detected a parity error on a CCC code via a parity check (vs. T bit) */

#define HAL_I3C_TGT_ERROR_2           (LL_I3C_TARGET_ERROR_TE2 | I3C_SER_PERR) /*!< 0x1A \
                                      Target detected a parity error on a write data via a parity check (vs. T bit) */

#define HAL_I3C_TGT_ERROR_3           (LL_I3C_TARGET_ERROR_TE3 | I3C_SER_PERR) /*!< 0x1B \
                                      Target detected a parity error on the assigned address during dynamic address \
                                      arbitration via a parity check (vs. PAR bit) */

#define HAL_I3C_TGT_ERROR_4           (LL_I3C_TARGET_ERROR_TE4 | I3C_SER_PERR) /*!< 0x1C \
                                      Target detected a 7'h7E + R missing after Sr during dynamic address arbitration */

#define HAL_I3C_TGT_ERROR_5           (LL_I3C_TARGET_ERROR_TE5 | I3C_SER_PERR) /*!< 0x1D \
                                      Target detected an illegally formatted CCC */

#define HAL_I3C_TGT_ERROR_6           (LL_I3C_TARGET_ERROR_TE6 | I3C_SER_PERR) /*!< 0x1E \
                                      Target detected that transmitted data on the bus is different from expected */

#define HAL_I3C_TGT_ERROR_STALL       LL_I3C_SER_STALL /*!< 0x20 \
                                      Target detected that SCL was stable for more than 125 microsecond during \
                                      an I3C SDR data read (during a direct CCC read, a private read, or an IB) */

#define HAL_I3C_ERROR_DOVR            LL_I3C_SER_DOVR /*!< 0x40 \
                                      Rx FIFO over-run or Tx FIFO under-run */

#define HAL_I3C_ERROR_COVR            LL_I3C_SER_COVR /*!< 0x80 \
                                      S FIFO over-run or C FIFO under-run */

#define HAL_I3C_ERROR_ADDRESS_NACK    LL_I3C_SER_ANACK /*!< 0x100 \
                                      Address not acknowledged */

#define HAL_I3C_ERROR_DATA_NACK       LL_I3C_SER_DNACK /*!< 0x200 \
                                      Data not acknowledged */

#define HAL_I3C_ERROR_DATA_HAND_OFF   LL_I3C_SER_DERR /*!< 0x400 \
                                      Data error during Controller-Role hand-off. Active controller keeps \
                                      controller-role */

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
#define HAL_I3C_ERROR_DMA             0x00010000U /*!< 0x10000 \
                                      DMA transfer error */
#endif /* USE_HAL_I3C_DMA */

#define HAL_I3C_ERROR_DYNAMIC_ADDR    0x00020000U /*!< 0x20000 \
                                      Dynamic address error */
/**
  * @}
  */


/** @defgroup I3C_CTRL_STALL_FEATURE_DEFINITION Stall feature definition
  * @brief    Optional controller clock stall insertion points on the I3C/I2C bus.
  * @details  Each define enables a programmable pause on a specific protocol phase to allow targets extra
  *           time (e.g. internal processing, data fetch) before the controller continues toggling SCL.
  *           These bits map to timing register fields (TIMINGR2) and must be used sparingly to avoid
  *           throughput degradation.
  * @{
  */
#define HAL_I3C_CTRL_STALL_ACK     LL_I3C_CTRL_STALL_ACK /*!< Inserts a stall after each address or data byte
                                                              ACK/NACK extend target processing time. */

#define HAL_I3C_CTRL_STALL_CCC     LL_I3C_CTRL_STALL_CCC /*!< Inserts a stall on the parity (T) bit phase of a
                                                              direct/broadcast CCC so targets can decode the
                                                              opcode before data phase. */

#define HAL_I3C_CTRL_STALL_TX      LL_I3C_CTRL_STALL_TX  /*!< Inserts a stall on the parity (T) bit of transmitted
                                                              data allowing target additional time to latch received
                                                              byte. */

#define HAL_I3C_CTRL_STALL_RX      LL_I3C_CTRL_STALL_RX  /*!< Inserts a stall before the controller samples target
                                                              data (read path) so target can prepare next byte. */

#define HAL_I3C_CTRL_STALL_I2C_ACK LL_I3C_CTRL_STALL_I2C_ACK /*!< Inserts a stall after address ACK/NACK in legacy I2C
                                                             read/write so target can prepare for next phase. */

#define HAL_I3C_CTRL_STALL_I2C_TX  LL_I3C_CTRL_STALL_I2C_TX  /*!< Inserts a stall after data ACK/NACK in legacy I2C
                                                             write allowing target to process/write incoming byte. */

#define HAL_I3C_CTRL_STALL_I2C_RX  LL_I3C_CTRL_STALL_I2C_RX  /*!< Inserts a stall after data ACK/NACK in legacy I2C
                                                             read allowing target to load the next byte to transmit. */

#define HAL_I3C_CTRL_STALL_ALL     LL_I3C_CTRL_STALL_ALL  /*!< Enable all stall points. */

#define HAL_I3C_CTRL_STALL_NONE    LL_I3C_CTRL_STALL_NONE /*!< Disable all stall insertion (default high throughput). */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Exported_Types HAL I3C Types
  * @{
  */

/**
  * @brief    Communication role (none / Controller / Target).
  */
typedef enum
{
  HAL_I3C_MODE_NONE = 0U,  /*!< No I3C communication ongoing */
  HAL_I3C_MODE_CTRL = 1U,  /*!< I3C communication is in controller Mode */
  HAL_I3C_MODE_TGT  = 2U   /*!< I3C communication is in target Mode */
} hal_i3c_mode_t;

/**
  * @brief    State structure definition.
  */
typedef enum
{
  HAL_I3C_STATE_RESET    = (0UL),        /*!< Not yet Initialized */
  HAL_I3C_STATE_INIT     = (1UL << 31),  /*!< I3C is initialized but not yet configured */
  HAL_I3C_STATE_IDLE     = (1UL << 30),  /*!< I3C initialized and a global config applied */
  HAL_I3C_STATE_TX       = (1UL << 29),  /*!< Data transmission process is ongoing */
  HAL_I3C_STATE_RX       = (1UL << 28),  /*!< Data reception process is ongoing */
  HAL_I3C_STATE_TX_RX    = (1UL << 27),  /*!< Data multiple Transfer process is ongoing */
  HAL_I3C_STATE_DAA      = (1UL << 26),  /*!< Dynamic address assignment process is ongoing */
  HAL_I3C_STATE_TGT_REQ  = (1UL << 25),  /*!< Target request process is ongoing */
  HAL_I3C_STATE_ABORT    = (1UL << 24)   /*!< Abort user request is ongoing */
} hal_i3c_state_t;

/** @defgroup I3C_CTRL_NOTIFICATION Controller notification ID
  * @{
  */
#define HAL_I3C_CTRL_NOTIFICATION_IBI  LL_I3C_IER_IBIIE    /*!< Receive IBI */
#define HAL_I3C_CTRL_NOTIFICATION_CR   LL_I3C_IER_CRIE     /*!< Controller-Role request */
#define HAL_I3C_CTRL_NOTIFICATION_HJ   LL_I3C_IER_HJIE     /*!< Hot-Join */
#define HAL_I3C_CTRL_NOTIFICATION_ALL (LL_I3C_IER_IBIIE | LL_I3C_IER_CRIE | LL_I3C_IER_HJIE) /*!< All notif */
/**
  * @}
  */

/** @defgroup I3C_TGT_NOTIFICATION Target notification ID
  * @{
  */
#define HAL_I3C_TGT_NOTIFICATION_GETACCCR   LL_I3C_IER_CRUPDIE  /*!< Controller-Role hand-off, direct GETACCR CCC */
#define HAL_I3C_TGT_NOTIFICATION_IBIEND     LL_I3C_IER_IBIENDIE /*!< IBI end process */
#define HAL_I3C_TGT_NOTIFICATION_DAU        LL_I3C_IER_DAUPDIE  /*!< Dynamic Address Update: ENTDAA, RSTDAA or SETNEWDA.
  To discriminate RSTDAA from other CCC, call HAL_I3C_GetCCCInfo() and check the field dynamic_addr_valid of
        hal_i3c_ccc_info_t structure */
#define HAL_I3C_TGT_NOTIFICATION_GET_X      LL_I3C_IER_GETIE    /*!< Any direct GETxxx CCC */
#define HAL_I3C_TGT_NOTIFICATION_GET_STATUS LL_I3C_IER_STAIE    /*!< Get status command, direct GETstatus CCC */
#define HAL_I3C_TGT_NOTIFICATION_SETMWL     LL_I3C_IER_MWLUPDIE /*!< Max write length update, direct SETMWL CCC */
#define HAL_I3C_TGT_NOTIFICATION_SETMRL     LL_I3C_IER_MRLUPDIE /*!< Max read length update, direct SETMRL CCC */
#define HAL_I3C_TGT_NOTIFICATION_RSTACT     LL_I3C_IER_RSTIE    /*!< Reset pattern,  broadcast or direct RSTACT CCC */
#define HAL_I3C_TGT_NOTIFICATION_ENTAS_X    LL_I3C_IER_ASUPDIE  /*!< Activity state update, direct or broadcast ENTASx */
#define HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC LL_I3C_IER_INTUPDIE /*!< Receive a direct or broadcast ENEC/DISEC CCC */
#define HAL_I3C_TGT_NOTIFICATION_WKP        LL_I3C_IER_WKPIE    /*!< Wakeup */
#define HAL_I3C_TGT_NOTIFICATION_DEFTGTS    (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE) /*!< Broadcast  DEFTGTS CCC */
#define HAL_I3C_TGT_NOTIFICATION_DEFGRPA    (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE) /*!< Group addressing, broadcast
                                                                                         DEFGRPA CCC */
#define HAL_I3C_TGT_NOTIFICATION_ALL  ( \
                                        HAL_I3C_TGT_NOTIFICATION_GETACCCR   | \
                                        HAL_I3C_TGT_NOTIFICATION_IBIEND     | \
                                        HAL_I3C_TGT_NOTIFICATION_DAU        | \
                                        HAL_I3C_TGT_NOTIFICATION_GET_X      | \
                                        HAL_I3C_TGT_NOTIFICATION_GET_STATUS | \
                                        HAL_I3C_TGT_NOTIFICATION_SETMWL     | \
                                        HAL_I3C_TGT_NOTIFICATION_SETMRL     | \
                                        HAL_I3C_TGT_NOTIFICATION_RSTACT     | \
                                        HAL_I3C_TGT_NOTIFICATION_ENTAS_X    | \
                                        HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC | \
                                        HAL_I3C_TGT_NOTIFICATION_WKP        | \
                                        HAL_I3C_TGT_NOTIFICATION_DEFTGTS    | \
                                        HAL_I3C_TGT_NOTIFICATION_DEFGRPA      ) /*!< All notifications */
/**
  * @}
  */

/**
  * @brief  Bitfield encoding message type, stop/restart policy, arbitration header & defining byte.
  *  HAL I3C mode value coding follow below described bitmap:
  *  b31
  *       0: message end type restart
  *       1: message end type stop (I3C_CR_MEND / LL_I3C_GENERATE_STOP)
  *  b30-b29-b28-b27
  *       0010: Private I3C message   (I3C_CR_MTYPE_1 / LL_I3C_CONTROLLER_MTYPE_PRIVATE)
  *       0100: Private I2C message   (I3C_CR_MTYPE_2 / LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C)
  *       0011: CCC direct message    (I3C_CR_MTYPE_1 | I3C_CR_MTYPE_0 / LL_I3C_CONTROLLER_MTYPE_DIRECT)
  *       0110: CCC broadcast message (I3C_CR_MTYPE_2 | I3C_CR_MTYPE_1 / LL_I3C_CONTROLLER_MTYPE_CCC)
  *  b2
  *       1: message without arbitration header (I3C_CFGR_NOARBH)
  *       0: message with arbitration header
  *  b0
  *       0: message without defining byte
  *       1: message with defining byte (LL_I3C_DEFINE_BYTE)
  *
  *  other bits (not used).
    */
typedef enum
{
  HAL_I3C_PRIVATE_WITH_ARB_RESTART = (LL_I3C_GENERATE_STOP), /*!<  Restart between each I3C private message then Stop
  request for last message. */

  HAL_I3C_PRIVATE_WITH_ARB_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_PRIVATE), /*!<  Stop between each I3C
  private message. Each message start with an arbitration header after start bit condition. */

  HAL_I3C_PRIVATE_WITHOUT_ARB_RESTART = (LL_I3C_CONTROLLER_MTYPE_PRIVATE | I3C_CFGR_NOARBH), /*!<  Restart between each
  I3C message then stop request for last message. Each message start with target address after start bit condition. */

  HAL_I3C_PRIVATE_WITHOUT_ARB_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_PRIVATE | I3C_CFGR_NOARBH), /*!<
  Stop between each I3C private message. Each message start with target address after start bit condition. */

  HAL_I2C_PRIVATE_WITH_ARB_RESTART = (LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C), /*!<  Restart between each I2C private
  message then stop request for last message. Each message start with an arbitration header after start bit condition.*/

  HAL_I2C_PRIVATE_WITH_ARB_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C), /*!<  Stop between each
  I2C private message. Each message start with an arbitration header after start bit condition. */

  HAL_I2C_PRIVATE_WITHOUT_ARB_RESTART = (LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C | I3C_CFGR_NOARBH), /*!<  Restart between
  each I2C message then stop request for last message. Each message start with target address after start bit condition.
  */

  HAL_I2C_PRIVATE_WITHOUT_ARB_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C | I3C_CFGR_NOARBH), /*!<
  Stop between each I2C private message. Each message start with target address after start bit condition. */

  HAL_I3C_CCC_DIRECT_WITH_DEFBYTE_RESTART = (LL_I3C_CONTROLLER_MTYPE_DIRECT | LL_I3C_DEFINE_BYTE), /*!< Restart
  between each direct command then stop request for last command. Each command has an associated defining byte */

  HAL_I3C_CCC_DIRECT_WITH_DEFBYTE_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_DIRECT  | LL_I3C_DEFINE_BYTE),
  /*!< Stop between each direct command.  Each command has an associated defining byte. */

  HAL_I3C_CCC_DIRECT_WITHOUT_DEFBYTE_RESTART = (LL_I3C_CONTROLLER_MTYPE_DIRECT), /*!< Restart between each direct
  command then stop request for last command. Each command does not have an associated defining byte. */

  HAL_I3C_CCC_DIRECT_WITHOUT_DEFBYTE_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_DIRECT), /*!< Stop
  between each direct command. Each command does not have an associated defining byte. */

  HAL_I3C_CCC_BROADCAST_WITH_DEFBYTE_RESTART = (LL_I3C_CONTROLLER_MTYPE_CCC | LL_I3C_DEFINE_BYTE), /*!< Restart between
  each broadcast command then stop request for last command. Each command has an associated defining byte. */

  HAL_I3C_CCC_BROADCAST_WITH_DEFBYTE_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_CCC | LL_I3C_DEFINE_BYTE),
  /*!< Stop between each broadcast command. Each command has an associated defining byte. */

  HAL_I3C_CCC_BROADCAST_WITHOUT_DEFBYTE_RESTART = (LL_I3C_CONTROLLER_MTYPE_CCC), /*!< Restart between each broadcast
  command then stop request for last command. Each command does not have an associated defining byte. */

  HAL_I3C_CCC_BROADCAST_WITHOUT_DEFBYTE_STOP = (LL_I3C_GENERATE_STOP | LL_I3C_CONTROLLER_MTYPE_CCC) /*!< Stop between
  each broadcast command. Each command does not have an associated defining byte. */
} hal_i3c_transfer_mode_t;

/**
  * @brief    Strategy used by Controller to (re)enumerate dynamic addresses.
  * @details  Selects how the controller manages dynamic address distribution:
  *           - RSTDAA+ENTDAA: full reset of any previously assigned dynamic addresses followed by a fresh
  *             enumeration (use after topology changes or suspected collisions).
  *           - ENTDAA only: perform enumeration assuming no stale dynamic addresses are present (faster
  *             startup on a clean bus or after global reset).
  */
typedef enum
{
  HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA = 0U, /*!< Full re-enumeration: issue RSTDAA to clear all dynamic addresses
                                                 then run ENTDAA to assign new ones. Ensures a clean address map. */
  HAL_I3C_DYN_ADDR_ONLY_ENTDAA        = 1U  /*!< Quick enumeration: run ENTDAA directly. Use when bus is known to be
                                                 in pristine state (e.g. initial power-up) to save time. */
} hal_i3c_dyn_addr_opt_t;

/**
  * @brief    Special bus patterns (reset, HDR exit) emitted by controller.
  * @details  Patterns are special electrical/sequence encodings placed on the bus outside normal frame transfers.
  *           They are used to signal global state changes:
  *           - Target Reset: issues a bus-level reset indication recognized by compliant targets.
  *           - HDR Exit: transitions all devices from High Data Rate mode back to Standard Data Rate (SDR).
  */
typedef enum
{
  HAL_I3C_PATTERN_TGT_RESET = 0U, /*!< Inject reset pattern to request targets reinitialize transient state. */
  HAL_I3C_PATTERN_HDR_EXIT  = 1U  /*!< Inject exit pattern to terminate HDR mode and restore SDR signaling. */
} hal_i3c_pattern_opt_t;

/**
  * @brief    Data phase direction (controller write or read).
  */
typedef enum
{
  HAL_I3C_DIRECTION_WRITE = LL_I3C_DIRECTION_WRITE, /*!< Controller sends data bytes to the addressed target */
  HAL_I3C_DIRECTION_READ  = LL_I3C_DIRECTION_READ,  /*!< Controller receives data bytes from the addressed target */
} hal_i3c_direction_t;

/**
  * @brief    Rx FIFO service granularity (byte vs word trigger level).
  * @details  The Rx FIFO size is 8 bytes (2 words) and an interrupt is generated while it is not empty.
  *           The selected threshold guides whether Software/DMA reads the FIFO one byte at a time or one word
  *           at a time. Advantages:
  *             - 1/8 (1 bytes threshold / 8 bytes FIFO): minimal read latency; finer flow control;
  *               better for short or sporadic transfers; simpler handling when parsing variable-length headers.
  *             - 1/2 (1 word threshold / 2 words FIFO): fewer service events (lower CPU/IRQ/DMA overhead);
  *               higher effective throughput on long bursts; improved bus efficiency when payload size is large.
  *           Choose based on latency sensitivity versus servicing overhead.
  */
typedef enum
{
  HAL_I3C_RX_FIFO_THRESHOLD_1_8 = LL_I3C_RXFIFO_THRESHOLD_1_8, /*!< 1 byte threshold / 8 bytes FIFO.
                                                                    Software/DMA reads byte while Rx FIFO is not
                                                                    empty. */

  HAL_I3C_RX_FIFO_THRESHOLD_1_2 = LL_I3C_RXFIFO_THRESHOLD_1_2, /*!< 1 word threshold / 2 words FIFO.
                                                                    Software/DMA reads word while Rx FIFO is not
                                                                    empty. */
} hal_i3c_rx_fifo_threshold_t;

/**
  * @brief    Tx FIFO service granularity (byte vs word trigger level).
  * @details  The Tx FIFO size is 8 bytes (2 words) and an interrupt is generated while the Tx FIFO is not full.
  *           The selected threshold guides whether Software/DMA writes the FIFO one byte at a time or one word
  *           at a time. Advantages:
  *             - 1/8 (1 bytes threshold / 8 bytes FIFO): minimal read latency; finer flow control;
  *               better for short or sporadic transfers; simpler handling when parsing variable-length headers.
  *             - 1/2 (1 word threshold / 2 words FIFO): fewer service events (lower CPU/IRQ/DMA overhead);
  *               higher effective throughput on long bursts; improved bus efficiency when payload size is large.
  *           Choose based on latency sensitivity versus servicing overhead.
  */
typedef enum
{
  HAL_I3C_TX_FIFO_THRESHOLD_1_8 = LL_I3C_TXFIFO_THRESHOLD_1_8, /*!< 1 byte threshold / 8 bytes FIFO.
                                                                    Software/DMA writes byte while Tx FIFO is not
                                                                    full. */

  HAL_I3C_TX_FIFO_THRESHOLD_1_2 = LL_I3C_TXFIFO_THRESHOLD_1_2, /*!< 1 word threshold / 2 words FIFO.
                                                                    Software/DMA writes word while Tx FIFO is not
                                                                    full. */
} hal_i3c_tx_fifo_threshold_t;

/**
  * @brief    Selection of Transmit Control (TC) and Receive Status (RS) FIFOs activation.
  */
typedef enum
{
  HAL_I3C_CTRL_FIFO_NONE         = LL_I3C_CTRL_FIFO_NONE,         /*!< Transmit Control (TC) and Receive Status (RS)
                                                                       FIFOs are disabled */

  HAL_I3C_CTRL_FIFO_CONTROL_ONLY = LL_I3C_CTRL_FIFO_CONTROL_ONLY, /*!< Enable Transmit Control (TC) to queue control
                                                                       words to be transmitted */

  HAL_I3C_CTRL_FIFO_STATUS_ONLY  = LL_I3C_CTRL_FIFO_STATUS_ONLY,  /*!< Enable Receive Status (RS) FIFO to capture
                                                                       status words received */

  HAL_I3C_CTRL_FIFO_ALL          = LL_I3C_CTRL_FIFO_ALL           /*!< Both TC and RS FIFOs are enabled */
} hal_i3c_ctrl_fifo_t;

/**
  * @brief     Control FIFO (C-FIFO) Status.
  */
typedef enum
{
  HAL_I3C_CONTROL_FIFO_DISABLED = 0U, /*!< Control FIFO is disabled */
  HAL_I3C_CONTROL_FIFO_ENABLED  = 1U  /*!< Control FIFO is enabled */
} hal_i3c_control_fifo_status_t;

/**
  * @brief     Status FIFO (S-FIFO) Status.
  */
typedef enum
{
  HAL_I3C_STATUS_FIFO_DISABLED = 0U, /*!< Status FIFO is disabled */
  HAL_I3C_STATUS_FIFO_ENABLED  = 1U  /*!< Status FIFO is enabled */
} hal_i3c_status_fifo_status_t;

/**
  * @brief    Number of data bytes appended after IBI acknowledge.
  * @details  Specifies how many data bytes follow the IBI acknowledge phase when a target asserts
  *           an In-Band Interrupt. These bytes typically convey a cause code, sensor snapshot or
  *           status flags allowing the initiator to react without issuing a separate read.
  *           Selecting the minimal size reduces bus occupancy; larger payloads allow richer
  *           contextual information at the cost of a few extra cycles.
  */
typedef enum
{
  HAL_I3C_TGT_PAYLOAD_EMPTY  = LL_I3C_PAYLOAD_EMPTY,   /*!< Empty payload, no additional data after IBI acknowledge */
  HAL_I3C_TGT_PAYLOAD_1_BYTE = LL_I3C_PAYLOAD_1_BYTE,  /*!< 1 additional data byte after IBI acknowledge */
  HAL_I3C_TGT_PAYLOAD_2_BYTE = LL_I3C_PAYLOAD_2_BYTE,  /*!< 2 additional data bytes after IBI acknowledge */
  HAL_I3C_TGT_PAYLOAD_3_BYTE = LL_I3C_PAYLOAD_3_BYTE,  /*!< 3 additional data bytes after IBI acknowledge */
  HAL_I3C_TGT_PAYLOAD_4_BYTE = LL_I3C_PAYLOAD_4_BYTE   /*!< 4 additional data bytes after IBI acknowledge */
} hal_i3c_tgt_payload_size_t;

/**
  * @brief    Advertised recent bus activity level (power/perf hint).
  * @details  Encodes recent bus traffic level for power/performance heuristics. Targets can adjust
  *           internal low-power policies or clocking based on the advertised activity state. Higher
  *           numbers generally indicate more frequent transfers or reduced idle periods.
  */
typedef enum
{
  HAL_I3C_ACTIVITY_STATE_0 = LL_I3C_BUS_ACTIVITY_STATE_0, /*!< Activity state 0 */
  HAL_I3C_ACTIVITY_STATE_1 = LL_I3C_BUS_ACTIVITY_STATE_1, /*!< Activity state 1 */
  HAL_I3C_ACTIVITY_STATE_2 = LL_I3C_BUS_ACTIVITY_STATE_2, /*!< Activity state 2 */
  HAL_I3C_ACTIVITY_STATE_3 = LL_I3C_BUS_ACTIVITY_STATE_3  /*!< Activity state 3 */
} hal_i3c_activity_state_t;

/**
  * @brief    Scope of peripheral reset to apply.
  */
typedef enum
{
  HAL_I3C_RESET_ACTION_NONE    = LL_I3C_RESET_ACTION_NONE,    /*!< No reset action required */
  HAL_I3C_RESET_ACTION_PARTIAL = LL_I3C_RESET_ACTION_PARTIAL, /*!< Reset some internal registers of the peripheral */
  HAL_I3C_RESET_ACTION_FULL    = LL_I3C_RESET_ACTION_FULL     /*!< Reset all internal registers of the peripheral */
} hal_i3c_reset_action_t;

/**
  * @brief    Intended bus activity level immediately after controller handoff.
  * @details  Advertised bus activity level the device intends to maintain immediately after it
  *           assumes Controller-Role (post handoff). Guides peers power management expectations.
  *           Higher states imply shorter idle windows and potentially higher average power draw.
  */
typedef enum
{
  HAL_I3C_HANDOFF_ACTIVITY_STATE_0 = LL_I3C_HANDOFF_ACTIVITY_STATE_0,  /*!< Activity state 0 after handoff */
  HAL_I3C_HANDOFF_ACTIVITY_STATE_1 = LL_I3C_HANDOFF_ACTIVITY_STATE_1,  /*!< Activity state 1 after handoff */
  HAL_I3C_HANDOFF_ACTIVITY_STATE_2 = LL_I3C_HANDOFF_ACTIVITY_STATE_2,  /*!< Activity state 2 after handoff */
  HAL_I3C_HANDOFF_ACTIVITY_STATE_3 = LL_I3C_HANDOFF_ACTIVITY_STATE_3   /*!< Activity state 3 after handoff */
} hal_i3c_handoff_activity_state_t;

/**
  * @brief    Target device clock-to-valid-data timing class (tSCO capability).
  * @details  Declares the device's timing capability from the rising edge of SCL to valid data on SDA.
  *           Used during timing negotiation so the controller can honour the slowest participant.
  *           LESS_12NS indicates a faster device (tSCO <= 12 ns); GREATER_12NS requires relaxed sampling.
  */
typedef enum
{
  HAL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS    = LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS,   /*!< clock-to-data turnaround
                                                                                            time tSCO <= 12ns */
  HAL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS = LL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS /*!< clock-to-data turnaround
                                                                                            time tSCO > 12ns  */
} hal_i3c_turnaround_time_tsco_t;


/**
  * @brief    Max data speed limitation status (BCR bit 0).
  */
typedef enum
{
  HAL_I3C_MAX_SPEED_LIMITATION_DISABLED = LL_I3C_NO_DATA_SPEED_LIMITATION, /*!< BCR[0]=0: max speed limitation disabled */
  HAL_I3C_MAX_SPEED_LIMITATION_ENABLED  = LL_I3C_MAX_DATA_SPEED_LIMITATION /*!< BCR[0]=1: max speed limitation enabled */
} hal_i3c_max_speed_limitation_status_t;

/**
  * @brief    IBI request status (BCR bit 1).
  */
typedef enum
{
  HAL_I3C_IBI_REQ_DISABLED = LL_I3C_IBI_REQUEST_NOT_SUPPORTED, /*!< BCR[1]=0: IBI request disabled */
  HAL_I3C_IBI_REQ_ENABLED  = LL_I3C_IBI_REQUEST_SUPPORTED      /*!< BCR[1]=1: IBI request enabled */
} hal_i3c_ibi_req_status_t;

/**
  * @brief    Ability to send data byte(s) with an IBI (BCR bit 2).
  */
typedef enum
{
  HAL_I3C_IBI_PAYLOAD_DISABLED = LL_I3C_IBI_NO_ADDITIONAL_DATA, /*!< BCR[2]=0: IBI payload disabled */
  HAL_I3C_IBI_PAYLOAD_ENABLED  = LL_I3C_IBI_ADDITIONAL_DATA     /*!< BCR[2]=1: IBI payload enabled */
} hal_i3c_ibi_payload_status_t;

/**
  * @brief    Offline capable status (BCR bit 3).
  */
typedef enum
{
  HAL_I3C_OFFLINE_CAPABLE_DISABLED = LL_I3C_NO_OFFLINE_CAPABLE, /*!< BCR[3]=0: offline not supported */
  HAL_I3C_OFFLINE_CAPABLE_ENABLED  = LL_I3C_OFFLINE_CAPABLE     /*!< BCR[3]=1: offline supported */
} hal_i3c_offline_capable_status_t;

/**
  * @brief    Virtual target support status (BCR bit 4).
  */
typedef enum
{
  HAL_I3C_VIRTUAL_TGT_DISABLED = LL_I3C_VIRTUAL_TARGET_NOT_SUPPORTED, /*!< BCR[4]=0: virtual target not supported */
  HAL_I3C_VIRTUAL_TGT_ENABLED  = LL_I3C_VIRTUAL_TARGET_SUPPORTED      /*!< BCR[4]=1: virtual target supported */
} hal_i3c_virtual_tgt_status_t;

/**
  * @brief    Advanced capabilities status (BCR bit 5).
  */
typedef enum
{
  HAL_I3C_ADV_CAPABILITIES_DISABLED = LL_I3C_ADV_CAP_NOT_SUPPORTED, /*!< BCR[5]=0: advanced capabilities disabled */
  HAL_I3C_ADV_CAPABILITIES_ENABLED  = LL_I3C_ADV_CAP_SUPPORTED      /*!< BCR[5]=1: advanced capabilities enabled */
} hal_i3c_adv_capabilities_status_t;

/**
  * @brief    Target device Controller-Role status  (BCR bit 5).
  */
typedef enum
{
  HAL_I3C_CTRL_ROLE_DISABLED = LL_I3C_DEVICE_ROLE_AS_TARGET,    /*!< BCR[6]=0: Target device must not request
                                                                     Controller-Role handoff */
  HAL_I3C_CTRL_ROLE_ENABLED  = LL_I3C_DEVICE_ROLE_AS_CONTROLLER /*!< BCR[6]=1:Target device can request
                                                                     Controller-Role handoff */
} hal_i3c_ctrl_role_status_t;


/**
  * @brief    Controller acknowledge policy for target IBI requests.
  */
typedef enum
{
  HAL_I3C_CTRL_IBI_ACK_DISABLED = LL_I3C_IBI_NO_CAPABILITY, /*!< Controller NACKs the IBI requests from the target */
  HAL_I3C_CTRL_IBI_ACK_ENABLED  = LL_I3C_IBI_CAPABILITY     /*!< Controller ACKs the IBI requests from the target */
} hal_i3c_ctrl_ibi_ack_status_t;

/**
  * @brief    Controller response status to a Controller-Role request from target device.
  */
typedef enum
{
  HAL_I3C_CTRL_ROLE_ACK_DISABLED = LL_I3C_CR_NO_CAPABILITY, /*!< Controller NACK when receiving a Controller-Role
                                                                 request from the target device */
  HAL_I3C_CTRL_ROLE_ACK_ENABLED  = LL_I3C_CR_CAPABILITY     /*!< Controller ACK when receiving a Controller-Role
                                                                 request from the target device */
} hal_i3c_ctrl_role_ack_status_t;


/**
  * @brief     Controller suspend/stop policy on IBI reception (SUSP bit).
  */
typedef enum
{
  HAL_I3C_CTRL_STOP_TRANSFER_DISABLED = LL_I3C_SUSP_DISABLE, /*!< Do not auto STOP/flush on IBI completion */
  HAL_I3C_CTRL_STOP_TRANSFER_ENABLED  = LL_I3C_SUSP_ENABLE   /*!< Auto STOP + flush C-FIFO/TX-FIFO on IBI completion */
} hal_i3c_ctrl_stop_transfer_status_t;

/**
  * @brief    Controller IBI payload policy.
  */
typedef enum
{
  HAL_I3C_CTRL_IBI_PAYLOAD_DISABLED = LL_I3C_IBI_DATA_DISABLE, /*!< Data follows the IBI ACK */
  HAL_I3C_CTRL_IBI_PAYLOAD_ENABLED  = LL_I3C_IBI_DATA_ENABLE   /*!< No data follows the IBI ACK */
} hal_i3c_ctrl_ibi_payload_status_t;

/**
  * @brief    IBI capability from target point of view.
  */
typedef enum
{
  HAL_I3C_TGT_IBI_DISABLED = 0U, /*!< IBI is not supported by target device */
  HAL_I3C_TGT_IBI_ENABLED  = 1U  /*!< IBI supported by target device */
} hal_i3c_tgt_ibi_status_t;

/**
  * @brief    Controller-Role capability from target point of view.
  */
typedef enum
{
  HAL_I3C_TGT_CTRL_ROLE_DISABLED = 0U, /*!< Controller-Role is not supported by target device */
  HAL_I3C_TGT_CTRL_ROLE_ENABLED  = 1U  /*!< Controller-Role is supported by target device */
} hal_i3c_tgt_ctrl_role_status_t;

/**
  * @brief    Target detection status during DAA procedure.
  */
typedef enum
{
  HAL_I3C_TGT_NOT_DETECTED = 0U, /*!< No target has been detected */
  HAL_I3C_TGT_DETECTED     = 1U  /*!< A target has been detected  */
} hal_i3c_target_detection_status_t;
/**
  * @brief    Handoff delay status.
  */
typedef enum
{
  HAL_I3C_HANDOFF_DELAY_DISABLED = 0U, /*!< Handoff delay is disabled */
  HAL_I3C_HANDOFF_DELAY_ENABLED  = 1U  /*!< Handoff delay is enabled */
} hal_i3c_handoff_delay_status_t;

/**
  * @brief    Group address capability status.
  */
typedef enum
{
  HAL_I3C_GRP_ADDR_CAPABILITY_DISABLED = 0U, /*!< Group address capability is disabled */
  HAL_I3C_GRP_ADDR_CAPABILITY_ENABLED  = 1U  /*!< Group address capability is enabled */
} hal_i3c_grp_addr_capability_status_t;

/**
  * @brief GETMXDS CCC format and maximum read turnaround byte latency.
  * Format 1 : No 3-byte MaxRdTurn (maximum read turnaround byte) is returned.
  * Format 2 : 3-byte MaxRdTurn (maximum read turnaround byte) is returned.
  */
typedef enum
{
  HAL_I3C_GETMXDS_FORMAT_1     = LL_I3C_GETMXDS_FORMAT_1, /*!< No 3-byte MaxRdTurn (maximum read turnaround byte). */

  HAL_I3C_GETMXDS_FORMAT_2_LSB = LL_I3C_GETMXDS_FORMAT_2_LSB, /*!< (<256 us) Target maximum read turnaround byte
                                                  (RDTURN[7:0]) placed in the least-significant byte of the 3-byte
                                                  MaxRdTurn (middle & MSB = 0)  */

  HAL_I3C_GETMXDS_FORMAT_2_MID = LL_I3C_GETMXDS_FORMAT_2_MID, /*!< (256 us to 65 ms) Target maximum read turnaround byte
                                                  (RDTURN[7:0]) placed in the middle byte of the 3-byte
                                                  MaxRdTurn  (LSB & MSB = 0) */

  HAL_I3C_GETMXDS_FORMAT_2_MSB = LL_I3C_GETMXDS_FORMAT_2_MSB /*!< (65 ms to 16 s) Target maximum read turnaround byte
                                                  (RDTURN[7:0]) placed in the most-significant byte of the 3-byte
                                                  MaxRdTurn (LSB & MID = 0)   */
} hal_i3c_getmxds_format_t;

/**
  * @brief    Hot-Join status.
  */
typedef enum
{
  HAL_I3C_HOT_JOIN_DISABLED = 0U, /*!< Hot-Join is disabled */
  HAL_I3C_HOT_JOIN_ENABLED  = 1U  /*!< Hot-Join is enabled */
} hal_i3c_hot_join_status_t;

/**
  * @brief    SDA high keeper enable state.
  */
typedef enum
{
  HAL_I3C_HIGH_KEEPER_SDA_DISABLED = 0U, /*!< The controller SDA high keeper is disabled */
  HAL_I3C_HIGH_KEEPER_SDA_ENABLED  = 1U  /*!< The controller SDA high keeper is enabled */
} hal_i3c_high_keeper_sda_status_t;

/**
  * @brief    Reset Pattern status.
  */
typedef enum
{
  HAL_I3C_RESET_PATTERN_DISABLED = 0U, /*!< Standard STOP condition emitted at the end of a frame */
  HAL_I3C_RESET_PATTERN_ENABLED  = 1U  /*!< Reset pattern is inserted before the STOP condition of any emitted frame */
} hal_i3c_reset_pattern_status_t;

/**
  * @brief    Pending mandatory data bytes notification with GETCAPR.
  */
typedef enum
{
  HAL_I3C_PENDING_READ_MDB_DISABLED = LL_I3C_MDB_NO_PENDING_READ_NOTIFICATION, /*!< Pending read mandatory data bytes
                                                                                    is disabled */
  HAL_I3C_PENDING_READ_MDB_ENABLED  = LL_I3C_MDB_PENDING_READ_NOTIFICATION     /*!< Pending read mandatory data bytes
                                                                                    is enabled */
} hal_i3c_tgt_read_mdb_status_t;

/**
  * @brief    Software descriptor for direct or broadcast CCC frame.
  */
typedef struct
{
  uint8_t  tgt_addr;               /*!< 7-bit dynamic or static target address placed on the bus.
                                        For broadcast CCC the broadcast address (7'h7E) is used. */

  uint8_t  ccc;                    /*!< 7-bit CCC opcode (direct or broadcast) per I3C specification section CCC.
                                        Distinguishes the command semantic. */

  uint32_t data_size_byte;         /*!< Number of data bytes associated with the CCC (including defining byte when present).
                                        Set to 0 if no data phase is required. */

  hal_i3c_direction_t direction;   /*!< Data phase direction: READ or WRITE. Must be WRITE for broadcast CCC.
                                        Direct CCC can be READ or WRITE depending on opcode definition. */
} hal_i3c_ccc_desc_t;

/**
  * @brief    Decoded BCR capability/status bits for a target.
  * @details  Populated from ENTDAA payload; each field mirrors one BCR capability bit or status for later use in
  *           controller configuration and application policy.
  */
typedef struct
{
  hal_i3c_max_speed_limitation_status_t max_data_speed_limitation;  /*!<
                                        HAL_I3C_MAX_SPEED_LIMITATION_ENABLED => target cannot operate at highest
                                        bus speed.
                                        HAL_I3C_MAX_SPEED_LIMITATION_DISABLED => no speed limitation declared. */

  hal_i3c_ibi_req_status_t              ibi_request_capable; /*!<
                                        HAL_I3C_IBI_REQ_ENABLED => target can initiate an In-Band Interrupt (IBI)
                                        to request service.
                                        HAL_I3C_IBI_REQ_DISABLED => target never issues IBI requests. */

  hal_i3c_ibi_payload_status_t          ibi_payload; /*!<
                                        Capability to append one data byte (or more depending on config)
                                        after IBI acceptance to qualify the interrupt source. */

  hal_i3c_offline_capable_status_t      offline_capable;  /*!< Target can temporarily not respond to bus commands.
                                        HAL_I3C_OFFLINE_CAPABLE_ENABLED => controller must tolerate silent periods.
                                        HAL_I3C_OFFLINE_CAPABLE_DISABLED => target expected to respond consistently. */

  hal_i3c_virtual_tgt_status_t          virtual_target_support;
  /*!< Identifies a virtual (composite / logical) target rather than a
  discrete physical device. */

  hal_i3c_adv_capabilities_status_t     advanced_capabilities; /*!< Target implements optional advanced I3C
                                        capabilities beyond the baseline feature set (e.g. controller role
                                        hand-off readiness). */

  hal_i3c_ctrl_role_status_t            ctrl_role; /*!< Target Controller-Role.
                                        HAL_I3C_CTRL_ROLE_ENABLED => target can request a Controller-Role
                                        handoff during Dynamic Address Assignment (DAA) or via a Controller-Role (CR)
                                        request sequence.
                                        HAL_I3C_CTRL_ROLE_DISABLED => target cannot request a Controller-Role
                                        handoff and any such attempt is ignored by higher-level policy. */
} hal_i3c_bcr_t;

/**
  * @brief    Decoded Provisioned ID (PID) fields.
  */
typedef struct
{
  uint16_t  mipi_manuf_id; /*!< MIPI-assigned manufacturer ID (PID bits [47:33]). */
  uint8_t   id_type_sel;   /*!< PID type selector / IDTSEL (format discriminator, PID bit [32]). */
  uint16_t  part_id;       /*!< Vendor part identifier (PID bits [31:21]) distinguishing device model/revision. */
  uint8_t   mipi_id;       /*!< Instance ID within the part family (PID bits [20:17]) for multiple identical
                                targets. */
} hal_i3c_pid_t;

/**
  * @brief    Software descriptor for a private frame (I3C or legacy I2C).
  */
typedef struct
{
  uint8_t              tgt_addr;        /*!< 7-bit dynamic or static target address placed on the bus */
  uint32_t             data_size_byte;  /*!< Number of data bytes to transmit or receive in the data phase.
                                             Set to 0 for pure header-only operations (rare). */
  hal_i3c_direction_t  direction;       /*!< Data direction for the frame: WRITE sends bytes from controller to target,
                                             READ requests bytes from target. */
} hal_i3c_private_desc_t;

/**
  * @brief    Aggregated multi-frame transfer context (descriptors + buffers).
  * @note     All frames aggregated in a transfer context must share the same high-level transfer mode
  *           @ref hal_i3c_transfer_mode_t
  */
typedef struct
{
  uint32_t *p_tc_data;      /*!< Pointer to transmit control descriptor words populated by
                                 @ref HAL_I3C_CTRL_BuildTransferCtxPrivate or
                                 @ref HAL_I3C_CTRL_BuildTransferCtxCCC */

  uint32_t tc_size_word;    /*!< Control buffer size depends on the transfer mode and the number of aggregated frames.
                                 The helper @ref HAL_I3C_GET_CTRL_BUFFER_SIZE_WORD can be used to get it. */

  const uint8_t *p_tx_data; /*!< Pointer to concatenated transmit payload(s) for all frames needing TX phase.
                                 Frames reference offsets implicitly in the order they are executed. */

  uint32_t tx_size_byte;    /*!< Total number of transmit bytes across all frames. Set to 0 if no TX phase. */

  uint8_t *p_rx_data;       /*!< Pointer to receive buffer storing concatenated RX data of frames with read phase. */

  uint32_t rx_size_byte;    /*!< Total expected receive bytes across all frames. Set to 0 if no RX phase. */

  hal_i3c_transfer_mode_t transfer_mode; /*!< Transfer mode bitfield (HAL_I3C_PRIVATE_xxx or HAL_I3C_CCC_xxx) governing
                                              arbitration, stop/restart behavior, and message type. */

  uint32_t nb_tx_frame;     /*!< Number of frames that have a transmit portion (debug / parameter checking aid). */
} hal_i3c_transfer_ctx_t;

/**
  * @brief    Controller timing configuration.
  * @details  Pre-computed I3C_TIMINGR0 / I3C_TIMINGR1 register values (prescalers + timing segments) generated by
  *           tooling (CubeMX2 / helpers). Used verbatim by HAL_I3C_CTRL_SetConfig(). Keep the pair consistent.
  */
typedef struct
{
  uint32_t  timing_reg0;  /*!< Encoded I3C_TIMINGR0 register value (prescalers + low/high periods for SDR/legacy).
                               Must be generated to satisfy bus frequency constraints of all attached targets. */

  uint32_t  timing_reg1;  /*!< Encoded I3C_TIMINGR1 register value (additional setup/hold segments and filter tuning).
                               Derived together with timing_reg0; keep the pair consistent. */
} hal_i3c_ctrl_config_t;

/**
  * @brief    Target timing configuration.
  * @details  Pre-computed I3C_TIMINGR1 register values (target turnaround / filtering) generated by
  *           tooling (CubeMX2 / helpers). Used verbatim by HAL_I3C_TGT_SetConfig().
  */
typedef struct
{
  uint32_t timing_reg1;   /*!< Encoded I3C_TIMINGR1 register value used when peripheral operates as target
                               to meet bus timing constraints. */
} hal_i3c_tgt_config_t;

/**
  * @brief    ENTDAA advertised capability payload (target identity fields for BCR/DCR/PID synthesis).
  */
typedef struct
{
  uint8_t identifier;      /*!< Target characteristic ID (MIPI named reference DCR). This parameter must be a number
                                between Min_Data=0x00 and Max_Data=0xFF. */

  uint8_t mipi_identifier; /*!< Bits [12-15] of the 48 bit provisioned ID (MIPI named reference PID).
                                This parameter must be a number between Min_Data=0x0 and Max_Data=0xF. */

  hal_i3c_ctrl_role_status_t            ctrl_role; /*!< Target can request Controller-Role handoff (BCR bit) */
  hal_i3c_ibi_payload_status_t          ibi_payload; /*!< Target can send data after an acknowledged IBI (BCR bit) */
  hal_i3c_max_speed_limitation_status_t max_data_speed_limitation; /*!< Target max data speed limitation (BCR bit) */
} hal_i3c_tgt_config_payload_entdaa_t;

/**
  * @brief    Controller FIFOs thresholds and enable selection.
  */
typedef struct
{
  hal_i3c_rx_fifo_threshold_t  rx_fifo_threshold; /*!< I3C Rx FIFO threshold level */
  hal_i3c_tx_fifo_threshold_t  tx_fifo_threshold; /*!< I3C Tx FIFO threshold level */
  hal_i3c_ctrl_fifo_t          ctrl_fifo;         /*!< I3C control and status activation */
} hal_i3c_ctrl_fifo_config_t;

/**
  * @brief    Target FIFOs thresholds configuration.
  */
typedef struct
{
  hal_i3c_rx_fifo_threshold_t  rx_fifo_threshold; /*!< I3C Rx FIFO threshold level */
  hal_i3c_tx_fifo_threshold_t  tx_fifo_threshold; /*!< I3C Tx FIFO threshold level */
} hal_i3c_tgt_fifo_config_t;

/**
  * @brief    Target IBI payload size & pending-read notification configuration.
  */
typedef struct
{
  hal_i3c_tgt_payload_size_t     ibi_payload_size_byte; /*!< I3C target payload data size */
  hal_i3c_tgt_read_mdb_status_t  pending_read_mdb;      /*!< Transmission of a mandatory data bytes indicating a pending
                                                             read notification for GETCAPR CCC command */
} hal_i3c_tgt_ibi_config_t;

/**
  * @brief    Target GETMXDS response parameters (format, activity, timing).
  */
typedef struct
{
  hal_i3c_getmxds_format_t          getmxds_format;           /*!< GETMXDS CCC Format */
  hal_i3c_handoff_activity_state_t  ctrl_handoff_activity;    /*!< I3C Target activity when becoming controller */
  hal_i3c_turnaround_time_tsco_t    data_turnaround_duration; /*!< I3C target clock-to-data turnaround time */
  uint8_t max_read_turnaround;  /*!< Target maximum read turnaround byte (RDTURN[7:0]). This parameter must be a number
                                     between Min_Data=0x00 and Max_Data=0xFF*/
} hal_i3c_tgt_getmxds_config_t;

/**
  * @brief    Target device configuration from controller point of view. Store in DEVRx registers.
  */
typedef struct
{
  uint8_t device_index;     /*!< Index value of the target device in the DEVRx register.
                                 This parameter must be a number between Min_Data=0 and Max_Data=3 */

  uint8_t tgt_dynamic_addr; /*!< Dynamic address of the target device.
                                 This parameter must be a number between Min_Data=0x00 and Max_Data=0x7F */

  hal_i3c_ctrl_ibi_ack_status_t        ibi_ack;            /*!< Controller ack when receiving an IBI from the target
                                                                device */

  hal_i3c_ctrl_ibi_payload_status_t    ibi_payload;        /*!< Target IBI payload after an IBI. Information retrieved
                                                                from the target device during broadcast ENTDAA or
                                                                direct GETBCR CCC */

  hal_i3c_ctrl_role_ack_status_t       ctrl_role_req_ack;  /*!< Controller ack when receiving a Controller-Role request
                                                                from the target device */

  hal_i3c_ctrl_stop_transfer_status_t  ctrl_stop_transfer; /*!< Controller stops transfer after receiving an IBI
                                                                from the target device */
} hal_i3c_ctrl_device_config_t;

/**
  * @brief    ENTDAA combined BCR/DCR/PID payload fields.
  */
typedef struct
{
  hal_i3c_bcr_t  bcr; /*!< Bus characteristics register */
  uint32_t       dcr; /*!< Device characteristics register */
  hal_i3c_pid_t  pid; /*!< Provisioned ID */
} hal_i3c_entdaa_payload_t;

/**
  * @brief  Target/Controller gets the Common Command Code (CCC) information updated after notifications.
  *   CCC Notification                    | Updated fields in p_ccc_info
  *  -------------------------------------|-----------------------------------------------------
  *   HAL_I3C_TGT_NOTIFICATION_DAU        | dynamic_addr, dynamic_addr_valid
  *   HAL_I3C_TGT_NOTIFICATION_SETMWL     | max_write_data_size_byte
  *   HAL_I3C_TGT_NOTIFICATION_SETMRL     | max_read_data_size_byte
  *   HAL_I3C_TGT_NOTIFICATION_RSTACT     | reset_action
  *   HAL_I3C_TGT_NOTIFICATION_ENTAS_X    | activity_state
  *   HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC | hot_join_allowed, in_band_allowed, ctrl_role_allowed
  *   HAL_I3C_CTRL_NOTIFICATION_IBI       | ibi_cr_tgt_addr, ibi_tgt_nb_payload, ibi_tgt_payload
  *   HAL_I3C_CTRL_NOTIFICATION_CR        | ibi_cr_tgt_addr
  */
typedef struct
{
  uint32_t dynamic_addr_valid; /*!< I3C target dynamic address validity (updated during ENTDAA/RSTDAA/SETNEWDA CCC)
                                    This parameter=1U after an ENTDAA or a SETNEWDA
                                    This parameter=0U after a RSTDAA */

  uint32_t dynamic_addr;       /*!< I3C target dynamic address (updated during ENTDAA/RSTDAA/SETNEWDA CCC)
                                    This parameter can be between Min_Data=0 and Max_Data=0x7F. */

  uint32_t  max_read_data_size_byte; /*!< Maximum read data length (in byte) that the target advertises to the
                                          Controller. Updated during SETMWL CCC.
                                          This parameter can be between Min_Data=0 and Max_Data=0xFFFF. */

  uint32_t  max_write_data_size_byte; /*!< Maximum write data length (in byte) that the target guarantees it
                                           can accept. Updated during SETMRL CCC.
                                           This parameter can be between Min_Data=0 and Max_Data=0xFFFF. */

  hal_i3c_reset_action_t reset_action;  /*!< I3C target reset action level (updated during RSTACT CCC) */

  hal_i3c_activity_state_t  activity_state;  /*!< I3C target activity state (updated during ENTASx CCC) */

  uint32_t hot_join_allowed;   /*!< I3C target Hot-Join (updated during ENEC/DISEC CCC)
                                    This parameter can be allowed=1U or not allowed=0U */

  uint32_t in_band_allowed;    /*!< I3C target in-band interrupt (updated during ENEC/DISEC CCC)
                                    This parameter can be allowed=1U or not allowed=0U */

  uint32_t ctrl_role_allowed;  /*!< I3C Target Controller-Role request permission (updated during ENEC/DISEC CCC).
                                    Value 1U => Target can request Controller role; 0U => not permitted. */

  uint32_t ibi_cr_tgt_addr;    /*!< I3C controller receives target address during IBI or Controller-Role request event
                                    This parameter can be between Min_Data=0 to Max_Data=0x3F */

  uint32_t ibi_tgt_nb_payload; /*!< I3C controller gets the number of data payload bytes after an IBI event
                                    This parameter can be between Min_Data=0 to Max_Data=0x7 */

  uint32_t ibi_tgt_payload;    /*!< I3C controller receives IBI payload after an IBI event
                                    Content of register is filled in Little Endian:
                                     - The MSB corresponds to the last IBI data byte,
                                     - LSB corresponds to first IBI data byte.
                                    This parameter can be between  Min_Data=0 to Max_Data=0xFFFFFFFF  */
} hal_i3c_ccc_info_t;

/**
  * @brief HAL I3C instance.
  */
typedef enum
{
  HAL_I3C1 = I3C1_BASE, /*!< Peripheral instance I3C1 */
} hal_i3c_t;

typedef struct hal_i3c_handle_s hal_i3c_handle_t; /*!< I3C handle structure type */

#if defined(USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1U)

/**
  * @brief Pointer to an I3C callback function.
  */
typedef  void (*hal_i3c_cb_t)(hal_i3c_handle_t *hi3c);

/**
  * @brief Pointer to an I3C notification callback function.
  */
typedef  void (*hal_i3c_notify_cb_t)(hal_i3c_handle_t *hi3c, uint32_t notify_id);

/**
  * @brief Pointer to an I3C target Hot-Join callback function.
  */
typedef  void (*hal_i3c_tgt_hot_join_cb_t)(hal_i3c_handle_t *hi3c, uint8_t dynamic_address);

/**
  * @brief Pointer to a target request dynamic address I3C callback function.
  */
typedef  void (*hal_i3c_req_dyn_addr_cb_t)(hal_i3c_handle_t *hi3c, uint64_t target_payload);

#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */

/**
  * @brief    I3C handle structure definition.
  */
struct hal_i3c_handle_s
{
  hal_i3c_t instance;         /*!< Peripheral instance */
  hal_i3c_mode_t mode;        /*!< Communication mode */
  uint32_t listen;            /*!< Listen mode */
  volatile hal_i3c_state_t global_state;  /*!< Communication state */
#if (defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)) \
    || (defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1))
  volatile uint32_t last_error_codes;  /*!< Errors limited to the last process
                                            This parameter can be a combination of @ref I3C_ERROR_CODE_DEFINITION */
#endif /* USE_HAL_I3C_GET_LAST_ERRORS || USE_HAL_I3C_DMA */
  const uint32_t *p_tc_data;  /*!< Transmit Control descriptor buffer */
  const uint8_t *p_tx_data;   /*!< Transmit data buffer */
  uint8_t *p_rx_data;         /*!< Receive data buffer */
  uint32_t tc_count_word;     /*!< Remaining control descriptor to transmit in word */
  uint32_t data_size_byte;    /*!< Data size to transmit or receive in byte */
  uint32_t tx_count_byte;     /*!< Remaining data to transmit in byte */
  uint32_t rx_count_byte;     /*!< Remaining data to receive in byte */
  hal_status_t(*p_isr_func)(hal_i3c_handle_t *hi3c, uint32_t it_masks); /*!< Dynamically selected IRQ handler for
                                   the current transfer/use case. Updated at operation start to route interrupts
                                   to the appropriate optimized routine */
  void(*p_tx_func)(hal_i3c_handle_t *hi3c);  /*!< Transmit function pointer */
  void(*p_rx_func)(hal_i3c_handle_t *hi3c);  /*!< Receive function pointer */

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
  hal_dma_handle_t *hdma_tc;  /*!< Transmit Control descriptor DMA handle (Controller side only) */
  hal_dma_handle_t *hdma_tx;  /*!< Transmit data DMA handle */
  hal_dma_handle_t *hdma_rx;  /*!< Receive data DMA handle */
#endif /* USE_HAL_I3C_DMA */

#if defined(USE_HAL_I3C_USER_DATA) && (USE_HAL_I3C_USER_DATA == 1)
  const void *p_user_data;  /*!< User data pointer */
#endif /* USE_HAL_I3C_USER_DATA */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t semaphore;  /*!< OS semaphore */
#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1U)
  hal_i3c_notify_cb_t       p_notify_cb;         /*!< Target/Controller asynchronous events callback */
  hal_i3c_cb_t              p_error_cb;          /*!< Target/Controller error callback */
  hal_i3c_cb_t              p_abort_cplt_cb;     /*!< Target/Controller abort complete callback */
  hal_i3c_cb_t              p_ctrl_daa_cplt_cb;  /*!< Controller Dynamic Address Assignment completed callback */
  hal_i3c_cb_t              p_ctrl_transfer_cplt_cb; /*!< Controller multiple direct CCC, I3C private or I2C transfer
                                                      completed callback */
  hal_i3c_req_dyn_addr_cb_t p_ctrl_tgt_req_dyn_addr_cb; /*!< Controller gets target dynamic address request
                                                      during DAA process */
  hal_i3c_cb_t              p_tgt_tx_cplt_cb;    /*!< Target private data transmit transfer completed callback */
  hal_i3c_cb_t              p_tgt_rx_cplt_cb;    /*!< Target private data receive transfer completed callback */
  hal_i3c_tgt_hot_join_cb_t p_tgt_hot_join_cb;   /*!< Target Hot-Join callback */
#endif  /* USE_HAL_I3C_REGISTER_CALLBACKS */
};

/**
  * @}
  */

/* Exported macro ----------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Exported_Macros HAL I3C Macros
  * @{
  */

/**
  * @brief  Compute required control buffer size in word for a transfer context.
  * @param  NB_DESC  Number of descriptions used to build the transfer context
  * @param  MODE     Mode of the transfer used to build transfer context See @ref hal_i3c_transfer_mode_t
  * @retval 2*NB_DESC in case of direct CCC transfers or NB_DESC for all other transfers
  */
#define HAL_I3C_GET_CTRL_BUFFER_SIZE_WORD(NB_DESC, MODE) \
  (((MODE & LL_I3C_CONTROLLER_MTYPE_DIRECT) == LL_I3C_CONTROLLER_MTYPE_DIRECT) ? (2U * NB_DESC) : NB_DESC)

/**
  * @brief  Extract the 48-bit Provisional ID (PID) from an ENTDAA payload.
  * @param  PAYLOAD Raw ENTDAA payload (up to 64 bits) containing the PID in bits [47:0].
  * @return 48-bit PID value (upper bits [63:48] forced to 0).
  */
#define HAL_I3C_GET_PID(PAYLOAD) ((uint64_t)(PAYLOAD) & LL_I3C_PID_IN_PAYLOAD_MASK)

/**
  * @brief  Extract MIPI Manufacturer (Vendor) ID from PID.
  * @param  PID Provisional ID (bits [47:0]).
  * @return Vendor ID field (bits [47:33], width 15 bits).
  */
#define HAL_I3C_GET_MIPIMID(PID) ((uint16_t)((uint64_t)(PID) >> LL_I3C_MIPIMID_PID_SHIFT) & LL_I3C_MIPIMID_PID_MASK)

/**
  * @brief  Extract Type selector (bit) from PID (bits 32).
  * @param  PID Provisional ID.
  * @return Type selector bit value 0 or 1.
  */
#define HAL_I3C_GET_IDTSEL(PID) ((uint8_t)((uint64_t)(PID) >> LL_I3C_IDTSEL_PID_SHIFT) &  LL_I3C_IDTSEL_PID_MASK)

/**
  * @brief  Extract Part ID from PID (bits [31:16], width 16 bits).
  * @param  PID Provisional ID.
  * @return Part ID.
  */
#define HAL_I3C_GET_PART_ID(PID) ((uint16_t)((uint64_t)(PID) >> LL_I3C_PART_ID_PID_SHIFT) & LL_I3C_PART_ID_PID_MASK)

/**
  * @brief  Extract Instance ID (device instance) from PID (bits [15:12], width 4 bits).
  * @param  PID Provisional ID.
  * @return Instance ID.
  */
#define HAL_I3C_GET_MIPIID(PID) ((uint8_t)((uint64_t)(PID) >> LL_I3C_MIPIID_PID_SHIFT) & LL_I3C_MIPIID_PID_MASK)

/**
  * @brief  Extract DCR (Device Characteristics Register) from ENTDAA payload (bits [63:56], width 8 bits).
  * @param  PAYLOAD Raw ENTDAA payload.
  * @return DCR .
  */
#define HAL_I3C_GET_DCR(PAYLOAD) (((uint32_t)((uint64_t)(PAYLOAD) >> LL_I3C_DCR_IN_PAYLOAD_SHIFT)) & I3C_DCR_DCR)

/**
  * @brief  Extract BCR (Bus Characteristics Register) from ENTDAA payload (bits [55:48], width 8 bits).
  * @param  PAYLOAD Raw ENTDAA payload.
  * @return BCR .
  */
#define HAL_I3C_GET_BCR(PAYLOAD) (((uint32_t)((uint64_t)(PAYLOAD) >> LL_I3C_BCR_IN_PAYLOAD_SHIFT)) \
                                  & LL_I3C_BCR_BCR_MSK)

/**
  * @brief  Extract max data speed limitation flag from BCR (bit 0).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_MAX_SPEED_LIMITATION_ENABLED or HAL_I3C_MAX_SPEED_LIMITATION_DISABLED.
  */
#define HAL_I3C_GET_MAX_DATA_SPEED_LIMIT(BCR) ((hal_i3c_max_speed_limitation_status_t)(uint32_t)((BCR) \
                                               & LL_I3C_BCR_BCR0_MSK))

/**
  * @brief  Extract In-Band Interrupt (IBI) request capability flag from BCR (bit 1).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_IBI_REQ_ENABLED or HAL_I3C_IBI_REQ_DISABLED.
 */
#define HAL_I3C_GET_IBI_CAPABLE(BCR) ((hal_i3c_ibi_req_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR1_MSK))

/**
  * @brief  Extract IBI additional data payload capability flag from BCR (bit 2).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_IBI_PAYLOAD_ENABLED or HAL_I3C_IBI_PAYLOAD_DISABLED.
  */
#define HAL_I3C_GET_IBI_PAYLOAD(BCR) ((hal_i3c_ibi_payload_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR2_MSK))

/**
  * @brief  Extract Offline Capable flag from BCR (bit 3).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_OFFLINE_CAPABLE_ENABLED or HAL_I3C_OFFLINE_CAPABLE_DISABLED.
  */
#define HAL_I3C_GET_OFFLINE_CAPABLE(BCR) ((hal_i3c_offline_capable_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR3_MSK))

/**
  * @brief  Extract Virtual Target support flag from BCR (bit 4).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_VIRTUAL_TGT_ENABLED or HAL_I3C_VIRTUAL_TGT_DISABLED.
  */
#define HAL_I3C_GET_VIRTUAL_TGT(BCR) ((hal_i3c_virtual_tgt_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR4_MSK))

/**
  * @brief  Extract advanced capabilities flag from BCR (bit 5).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_ADV_CAPABILITIES_ENABLED or HAL_I3C_ADV_CAPABILITIES_DISABLED.
  */
#define HAL_I3C_GET_ADVANCED_CAPABLE(BCR) ((hal_i3c_adv_capabilities_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR5_MSK))

/**
  * @brief  Extract Controller-Role request capability flag from BCR (bit 6).
  * @param  BCR BCR byte.
  * @retval HAL_I3C_CTRL_ROLE_ENABLED or HAL_I3C_CTRL_ROLE_DISABLED.
  */
#define HAL_I3C_GET_CTRL_ROLE_CAPABLE(BCR) ((hal_i3c_ctrl_role_status_t)(uint32_t)((BCR) & LL_I3C_BCR_BCR6_MSK))

/** @brief  Change uint32_t variable form big endian to little endian.
  * @param  DATA uint32_t variable in big endian
  *         This parameter must be a number between Min_Data=0x00(uint32_t) and Max_Data=0xFFFFFFFF
  * @return uint32_t variable in little endian
  */
#define HAL_I3C_BIG_TO_LITTLE_ENDIAN(DATA) ((uint32_t)((((DATA) & 0xff000000U) >> 24) \
                                                       | (((DATA) & 0x00ff0000U) >> 8) \
                                                       | (((DATA) & 0x0000ff00U) << 8) \
                                                       | (((DATA) & 0x000000ffU) << 24)))

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup I3C_Exported_Functions HAL I3C Functions
  * @{
  */

/** @addtogroup I3C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_I3C_Init(hal_i3c_handle_t *hi3c, hal_i3c_t instance);
void HAL_I3C_DeInit(hal_i3c_handle_t *hi3c);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group2 Configuration functions
  * @{
  */

/* Global configuration */
/* Global configuration (controller) */
hal_status_t HAL_I3C_CTRL_SetConfig(hal_i3c_handle_t *hi3c, const hal_i3c_ctrl_config_t *p_config);
void HAL_I3C_CTRL_GetConfig(const hal_i3c_handle_t *hi3c, hal_i3c_ctrl_config_t *p_config);

/* Global configuration (target) */
hal_status_t HAL_I3C_TGT_SetConfig(hal_i3c_handle_t *hi3c, const hal_i3c_tgt_config_t *p_config);
void HAL_I3C_TGT_GetConfig(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_config_t *p_config);

/* Payload ENTDAA configuration (target) */
hal_status_t HAL_I3C_TGT_SetPayloadENTDAAConfig(const hal_i3c_handle_t *hi3c,
                                                const hal_i3c_tgt_config_payload_entdaa_t *p_config);
void HAL_I3C_TGT_GetPayloadENTDAAConfig(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_config_payload_entdaa_t *p_config);

/* FIFO configuration (controller) */
hal_status_t HAL_I3C_CTRL_SetConfigFifo(const hal_i3c_handle_t *hi3c, const hal_i3c_ctrl_fifo_config_t *p_config);
void HAL_I3C_CTRL_GetConfigFifo(const hal_i3c_handle_t *hi3c, hal_i3c_ctrl_fifo_config_t *p_config);

/* FIFO configuration (target) */
hal_status_t HAL_I3C_TGT_SetConfigFifo(const hal_i3c_handle_t *hi3c, const hal_i3c_tgt_fifo_config_t *p_config);
void HAL_I3C_TGT_GetConfigFifo(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_fifo_config_t *p_config);

/* FIFO configuration unitary functions */
hal_status_t HAL_I3C_SetRxFifoThreshold(const hal_i3c_handle_t *hi3c, const hal_i3c_rx_fifo_threshold_t threshold);
hal_i3c_rx_fifo_threshold_t HAL_I3C_GetRxFifoThreshold(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_SetTxFifoThreshold(const hal_i3c_handle_t *hi3c, const hal_i3c_tx_fifo_threshold_t threshold);
hal_i3c_tx_fifo_threshold_t HAL_I3C_GetTxFifoThreshold(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_EnableControlFifo(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_DisableControlFifo(hal_i3c_handle_t *hi3c);
hal_i3c_control_fifo_status_t HAL_I3C_CTRL_IsEnabledControlFifo(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_EnableStatusFifo(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_DisableStatusFifo(hal_i3c_handle_t *hi3c);
hal_i3c_status_fifo_status_t HAL_I3C_CTRL_IsEnabledStatusFifo(const hal_i3c_handle_t *hi3c);

/* Own dynamic address (controller) */
hal_status_t HAL_I3C_CTRL_SetConfigOwnDynamicAddress(hal_i3c_handle_t *hi3c, uint32_t dynamic_address);
uint32_t HAL_I3C_CTRL_GetConfigOwnDynamicAddress(const hal_i3c_handle_t *hi3c);

/* Hot-Join allowed (controller) */
hal_status_t HAL_I3C_CTRL_EnableHotJoinAllowed(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_DisableHotJoinAllowed(hal_i3c_handle_t *hi3c);
hal_i3c_hot_join_status_t HAL_I3C_CTRL_IsEnabledHotJoinAllowed(const hal_i3c_handle_t *hi3c);

/* High keeper SDA configuration (controller) */
hal_status_t HAL_I3C_CTRL_EnableHighKeeperSDA(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_DisableHighKeeperSDA(const hal_i3c_handle_t *hi3c);
hal_i3c_high_keeper_sda_status_t HAL_I3C_CTRL_IsEnabledHighKeeperSDA(const hal_i3c_handle_t *hi3c);

/* Stall time configuration (controller) */
hal_status_t HAL_I3C_CTRL_SetConfigStallTime(const hal_i3c_handle_t *hi3c, uint32_t stall_time_cycle,
                                             uint32_t stall_features);
hal_status_t HAL_I3C_CTRL_GetConfigStallTime(const hal_i3c_handle_t *hi3c, uint32_t *stall_time_cycle,
                                             uint32_t *stall_features);

/* Controller-Role request configuration (target) */
hal_status_t HAL_I3C_TGT_EnableCtrlRoleRequest(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_DisableCtrlRoleRequest(const hal_i3c_handle_t *hi3c);
hal_i3c_tgt_ctrl_role_status_t HAL_I3C_TGT_IsEnabledCtrlRoleRequest(const hal_i3c_handle_t *hi3c);

hal_status_t HAL_I3C_TGT_EnableHandOffDelay(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_DisableHandOffDelay(const hal_i3c_handle_t *hi3c);
hal_i3c_handoff_delay_status_t HAL_I3C_TGT_IsEnabledHandOffDelay(const hal_i3c_handle_t *hi3c);

/* Group management support configuration (target) */
hal_status_t HAL_I3C_TGT_EnableGroupAddrCapability(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_DisableGroupAddrCapability(const hal_i3c_handle_t *hi3c);
hal_i3c_grp_addr_capability_status_t HAL_I3C_TGT_IsEnabledGroupAddrCapability(const hal_i3c_handle_t *hi3c);

/* Hot-Join configuration (target) */
hal_status_t HAL_I3C_TGT_EnableHotJoinRequest(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_DisableHotJoinRequest(const hal_i3c_handle_t *hi3c);
hal_i3c_hot_join_status_t HAL_I3C_TGT_IsEnabledHotJoinRequest(const hal_i3c_handle_t *hi3c);

/* IBI configuration (target) */
hal_status_t HAL_I3C_TGT_SetConfigIBI(const hal_i3c_handle_t *hi3c, const hal_i3c_tgt_ibi_config_t *p_config);
void HAL_I3C_TGT_GetConfigIBI(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_ibi_config_t *p_config);
hal_status_t HAL_I3C_TGT_EnableIBI(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_DisableIBI(const hal_i3c_handle_t *hi3c);
hal_i3c_tgt_ibi_status_t HAL_I3C_TGT_IsEnabledIBI(const hal_i3c_handle_t *hi3c);

/* Max data size configuration (target) */
hal_status_t HAL_I3C_TGT_SetConfigMaxDataSize(const hal_i3c_handle_t *hi3c, uint32_t max_read_data_size_byte,
                                              uint32_t max_write_data_size_byte);
hal_status_t HAL_I3C_TGT_GetConfigMaxDataSize(const hal_i3c_handle_t *hi3c, uint32_t *p_max_read_data_size_byte,
                                              uint32_t *p_max_write_data_size_byte);

/* GET MaX Data Speed (GETMXDS) configuration (target) */
hal_status_t HAL_I3C_TGT_SetConfigGETMXDS(const hal_i3c_handle_t *hi3c, const hal_i3c_tgt_getmxds_config_t *p_config);
void HAL_I3C_TGT_GetConfigGETMXDS(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_getmxds_config_t *p_config);
/* GET MaX Data Speed (GETMXDS) unitary functions */
hal_status_t HAL_I3C_TGT_SetConfigGETMXDS_Format(const hal_i3c_handle_t *hi3c, hal_i3c_getmxds_format_t format);
hal_i3c_getmxds_format_t HAL_I3C_TGT_GetConfigGETMXDS_Format(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_SetConfigCtrlHandOffActivity(const hal_i3c_handle_t *hi3c,
                                                      hal_i3c_handoff_activity_state_t state);
hal_i3c_handoff_activity_state_t HAL_I3C_TGT_GetConfigCtrlHandOffActivity(const hal_i3c_handle_t *hi3c);

/* Bus device configuration configuration (controller) */
hal_status_t HAL_I3C_CTRL_SetConfigBusDevices(const hal_i3c_handle_t *hi3c, const hal_i3c_ctrl_device_config_t *p_desc,
                                              uint32_t nb_device);
void HAL_I3C_CTRL_GetConfigBusDevices(const hal_i3c_handle_t *hi3c, hal_i3c_ctrl_device_config_t *p_desc,
                                      uint32_t nb_device);

/* Reset pattern configuration (controller) */
hal_status_t HAL_I3C_CTRL_EnableResetPattern(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_DisableResetPattern(hal_i3c_handle_t *hi3c);
hal_i3c_reset_pattern_status_t HAL_I3C_CTRL_IsEnabledResetPattern(const hal_i3c_handle_t *hi3c);

#if defined(USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1U)
/* Register callbacks */
hal_status_t HAL_I3C_CTRL_RegisterTransferCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
hal_status_t HAL_I3C_CTRL_RegisterDAACpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
hal_status_t HAL_I3C_CTRL_RegisterTgtReqDynAddrCallback(hal_i3c_handle_t *hi3c,
                                                        hal_i3c_req_dyn_addr_cb_t p_callback);
hal_status_t HAL_I3C_TGT_RegisterTxCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
hal_status_t HAL_I3C_TGT_RegisterRxCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
hal_status_t HAL_I3C_TGT_RegisterHotJoinCallback(hal_i3c_handle_t *hi3c, hal_i3c_tgt_hot_join_cb_t p_callback);
hal_status_t HAL_I3C_RegisterNotifyCallback(hal_i3c_handle_t *hi3c, hal_i3c_notify_cb_t p_callback);
hal_status_t HAL_I3C_RegisterAbortCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
hal_status_t HAL_I3C_RegisterErrorCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
hal_status_t HAL_I3C_SetTxDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma);
hal_status_t HAL_I3C_SetRxDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma);
hal_status_t HAL_I3C_SetTcDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma);
#endif /* USE_HAL_I3C_DMA */
/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group3 Interrupt and callback functions
  * @{
  */
hal_status_t HAL_I3C_CTRL_ActivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications);
hal_status_t HAL_I3C_CTRL_DeactivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications);
hal_status_t HAL_I3C_TGT_ActivateNotification(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte,
                                              uint32_t notifications);
hal_status_t HAL_I3C_TGT_DeactivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group4 IRQ Handlers
  * @{
  */
void HAL_I3C_ERR_IRQHandler(hal_i3c_handle_t *hi3c);
void HAL_I3C_EV_IRQHandler(hal_i3c_handle_t *hi3c);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group5 FIFO flush functions
  * @{
  */
hal_status_t HAL_I3C_FlushAllFifos(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_FlushTxFifo(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_FlushRxFifo(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_FlushControlFifo(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_CTRL_FlushStatusFifo(const hal_i3c_handle_t *hi3c);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group6 Controller transfer operation functions
  * @{
  */
/* Controller transfer operation */
hal_status_t HAL_I3C_CTRL_ResetTransferCtx(hal_i3c_transfer_ctx_t *p_ctx);
hal_status_t HAL_I3C_CTRL_InitTransferCtxTc(hal_i3c_transfer_ctx_t *p_ctx, uint32_t *p_ctrl_buf,
                                            uint32_t size_word);
hal_status_t HAL_I3C_CTRL_InitTransferCtxTx(hal_i3c_transfer_ctx_t *p_ctx, const uint8_t *p_tx_data,
                                            uint32_t size_byte);
hal_status_t HAL_I3C_CTRL_InitTransferCtxRx(hal_i3c_transfer_ctx_t *p_ctx, uint8_t *p_rx_data,
                                            uint32_t size_byte);

hal_status_t HAL_I3C_CTRL_BuildTransferCtxPrivate(hal_i3c_transfer_ctx_t *p_ctx, const hal_i3c_private_desc_t *p_desc,
                                                  uint32_t nb_desc, hal_i3c_transfer_mode_t mode);
hal_status_t HAL_I3C_CTRL_BuildTransferCtxCCC(hal_i3c_transfer_ctx_t *p_ctx, const hal_i3c_ccc_desc_t *p_desc,
                                              uint32_t nb_desc, hal_i3c_transfer_mode_t mode);
hal_status_t HAL_I3C_CTRL_Transfer(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx, uint32_t timeout_ms);
hal_status_t HAL_I3C_CTRL_Transfer_IT(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
hal_status_t HAL_I3C_CTRL_Transfer_DMA(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx);
#endif /* USE_HAL_I3C_DMA */

/* Controller assign dynamic address APIs */
hal_status_t HAL_I3C_CTRL_SetDynAddr(const hal_i3c_handle_t *hi3c, uint8_t target_address);
hal_status_t HAL_I3C_CTRL_DynAddrAssign_IT(hal_i3c_handle_t *hi3c, hal_i3c_dyn_addr_opt_t option);
hal_status_t HAL_I3C_CTRL_DynAddrAssign(hal_i3c_handle_t *hi3c, uint64_t *p_target_payload,
                                        hal_i3c_dyn_addr_opt_t option,
                                        hal_i3c_target_detection_status_t *p_target_detection_status,
                                        uint32_t timeout_ms);

/* Controller check device ready APIs */
hal_status_t HAL_I3C_CTRL_PoolForDeviceI3cReady(hal_i3c_handle_t *hi3c, uint8_t target_address, uint32_t timeout_ms);
hal_status_t HAL_I3C_CTRL_PoolForDeviceI2cReady(hal_i3c_handle_t *hi3c, uint8_t target_address, uint32_t timeout_ms);
/* Controller patterns APIs */
hal_status_t HAL_I3C_CTRL_GeneratePatterns(hal_i3c_handle_t *hi3c, hal_i3c_pattern_opt_t pattern, uint32_t timeout_ms);

/* Controller arbitration API */
hal_status_t HAL_I3C_CTRL_GenerateArbitration(hal_i3c_handle_t *hi3c, uint32_t timeout_ms);

/* Controller stop SCL API in case of CE1 error */
hal_status_t HAL_I3C_CTRL_RecoverSCLToIDLE(const hal_i3c_handle_t *hi3c);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group7 Target operational functions
  * @{
  */
hal_status_t HAL_I3C_TGT_Transmit(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte,
                                  uint32_t timeout_ms);
hal_status_t HAL_I3C_TGT_Transmit_IT(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
hal_status_t HAL_I3C_TGT_Transmit_DMA(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte);
#endif /* USE_HAL_I3C_DMA */
hal_status_t HAL_I3C_TGT_Receive(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_I3C_TGT_Receive_IT(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
hal_status_t HAL_I3C_TGT_Receive_DMA(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte);
#endif /* USE_HAL_I3C_DMA */
hal_status_t HAL_I3C_TGT_ControlRoleReq(hal_i3c_handle_t *hi3c, uint32_t timeout_ms);
hal_status_t HAL_I3C_TGT_ControlRoleReq_IT(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_HotJoinReq(hal_i3c_handle_t *hi3c, uint8_t *p_own_dynamic_address, uint32_t timeout_ms);
hal_status_t HAL_I3C_TGT_HotJoinReq_IT(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_TGT_IBIReq(hal_i3c_handle_t *hi3c, const uint8_t *p_payload, uint32_t payload_size_byte,
                                uint32_t timeout_ms);
hal_status_t HAL_I3C_TGT_IBIReq_IT(hal_i3c_handle_t *hi3c, const uint8_t *p_payload, uint32_t payload_size_byte);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group8 Weak callback functions
  * @{
  */
void HAL_I3C_CTRL_TransferCpltCallback(hal_i3c_handle_t *hi3c);
void HAL_I3C_CTRL_DAACpltCallback(hal_i3c_handle_t *hi3c);
void HAL_I3C_CTRL_TgtReqDynAddrCallback(hal_i3c_handle_t *hi3c, uint64_t target_payload);
void HAL_I3C_TGT_TxCpltCallback(hal_i3c_handle_t *hi3c);
void HAL_I3C_TGT_RxCpltCallback(hal_i3c_handle_t *hi3c);
void HAL_I3C_TGT_HotJoinCallback(hal_i3c_handle_t *hi3c, uint8_t dynamic_address);
void HAL_I3C_NotifyCallback(hal_i3c_handle_t *hi3c, uint32_t notify_id);
void HAL_I3C_ErrorCallback(hal_i3c_handle_t *hi3c);
void HAL_I3C_AbortCpltCallback(hal_i3c_handle_t *hi3c);
/**
  * @}
  */

/** @defgroup I3C_Exported_Functions_Group9 Generic and common functions
  * @{
  */
hal_i3c_state_t HAL_I3C_GetState(const hal_i3c_handle_t *hi3c);
hal_i3c_mode_t HAL_I3C_GetMode(const hal_i3c_handle_t *hi3c);
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
uint32_t HAL_I3C_GetLastErrorCodes(const hal_i3c_handle_t *hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
uint32_t HAL_I3C_GetClockFreq(const hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_Abort_IT(hal_i3c_handle_t *hi3c);
hal_status_t HAL_I3C_GetCCCInfo(const hal_i3c_handle_t *hi3c, uint32_t notifications, hal_i3c_ccc_info_t *p_ccc_info);
hal_status_t HAL_I3C_CTRL_GetENTDAA_PayloadInfo(uint64_t entdaa_payload, hal_i3c_entdaa_payload_t *p_entdaa_payload);
uint32_t HAL_I3C_GetDataCounter(hal_i3c_handle_t *hi3c);
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup I3C_Exported_Functions_Group10 Acquire/release the bus
  * @{
  */
hal_status_t HAL_I3C_AcquireBus(hal_i3c_handle_t *hi3c, uint32_t timeout_ms);
hal_status_t HAL_I3C_ReleaseBus(hal_i3c_handle_t *hi3c);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_I3C_USER_DATA) && (USE_HAL_I3C_USER_DATA == 1)
/** @defgroup I3C_Exported_Functions_Group11 Set/get user data
  * @{
  */
void HAL_I3C_SetUserData(hal_i3c_handle_t *hi3c, const void *p_user_data);
const void *HAL_I3C_GetUserData(const hal_i3c_handle_t *hi3c);
/**
  * @}
  */
#endif /* USE_HAL_I3C_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* I3C1 */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_I3C_H */
