/**
  ******************************************************************************
  * @file    stm32c5xx_ll_i3c.h
  * @brief   Header file of I3C LL module.
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
#ifndef STM32C5xx_LL_I3C_H
#define STM32C5xx_LL_I3C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined (I3C1)

/** @defgroup I3C_LL I3C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup I3C_LL_Exported_Constants LL I3C Constants
  * @{
  */

/** @defgroup I3C_LL_EC_GET_FLAG Get flags defines
  * @brief    Flags defines which can be used with LL_I3C_READ_REG function
  * @{
  */
#define LL_I3C_EVR_CFEF        I3C_EVR_CFEF
#define LL_I3C_EVR_TXFEF       I3C_EVR_TXFEF
#define LL_I3C_EVR_CFNFF       I3C_EVR_CFNFF
#define LL_I3C_EVR_SFNEF       I3C_EVR_SFNEF
#define LL_I3C_EVR_TXFNFF      I3C_EVR_TXFNFF
#define LL_I3C_EVR_RXFNEF      I3C_EVR_RXFNEF
#define LL_I3C_EVR_RXLASTF     I3C_EVR_RXLASTF
#define LL_I3C_EVR_TXLASTF     I3C_EVR_TXLASTF
#define LL_I3C_EVR_FCF         I3C_EVR_FCF
#define LL_I3C_EVR_RXTGTENDF   I3C_EVR_RXTGTENDF
#define LL_I3C_EVR_ERRF        I3C_EVR_ERRF
#define LL_I3C_EVR_IBIF        I3C_EVR_IBIF
#define LL_I3C_EVR_IBIENDF     I3C_EVR_IBIENDF
#define LL_I3C_EVR_CRF         I3C_EVR_CRF
#define LL_I3C_EVR_CRUPDF      I3C_EVR_CRUPDF
#define LL_I3C_EVR_HJF         I3C_EVR_HJF
#define LL_I3C_EVR_WKPF        I3C_EVR_WKPF
#define LL_I3C_EVR_GETF        I3C_EVR_GETF
#define LL_I3C_EVR_STAF        I3C_EVR_STAF
#define LL_I3C_EVR_DAUPDF      I3C_EVR_DAUPDF
#define LL_I3C_EVR_MWLUPDF     I3C_EVR_MWLUPDF
#define LL_I3C_EVR_MRLUPDF     I3C_EVR_MRLUPDF
#define LL_I3C_EVR_RSTF        I3C_EVR_RSTF
#define LL_I3C_EVR_ASUPDF      I3C_EVR_ASUPDF
#define LL_I3C_EVR_INTUPDF     I3C_EVR_INTUPDF
#define LL_I3C_EVR_DEFF        I3C_EVR_DEFF
#define LL_I3C_EVR_GRPF        I3C_EVR_GRPF
#define LL_I3C_SER_PERR        I3C_SER_PERR
#define LL_I3C_SER_STALL       I3C_SER_STALL
#define LL_I3C_SER_DOVR        I3C_SER_DOVR
#define LL_I3C_SER_COVR        I3C_SER_COVR
#define LL_I3C_SER_ANACK       I3C_SER_ANACK
#define LL_I3C_SER_DNACK       I3C_SER_DNACK
#define LL_I3C_SER_DERR        I3C_SER_DERR
/**
  * @}
  */

/** @defgroup I3C_LL_EC_IT IT Defines
  * @brief    IT defines and combinations of defines
  * @{
  */
#define LL_I3C_IER_CFNFIE       I3C_IER_CFNFIE
#define LL_I3C_IER_SFNEIE       I3C_IER_SFNEIE
#define LL_I3C_IER_TXFNFIE      I3C_IER_TXFNFIE
#define LL_I3C_IER_RXFNEIE      I3C_IER_RXFNEIE
#define LL_I3C_IER_FCIE         I3C_IER_FCIE
#define LL_I3C_IER_RXTGTENDIE   I3C_IER_RXTGTENDIE
#define LL_I3C_IER_ERRIE        I3C_IER_ERRIE
#define LL_I3C_IER_IBIIE        I3C_IER_IBIIE
#define LL_I3C_IER_IBIENDIE     I3C_IER_IBIENDIE
#define LL_I3C_IER_CRIE         I3C_IER_CRIE
#define LL_I3C_IER_CRUPDIE      I3C_IER_CRUPDIE
#define LL_I3C_IER_HJIE         I3C_IER_HJIE
#define LL_I3C_IER_WKPIE        I3C_IER_WKPIE
#define LL_I3C_IER_GETIE        I3C_IER_GETIE
#define LL_I3C_IER_STAIE        I3C_IER_STAIE
#define LL_I3C_IER_DAUPDIE      I3C_IER_DAUPDIE
#define LL_I3C_IER_MWLUPDIE     I3C_IER_MWLUPDIE
#define LL_I3C_IER_MRLUPDIE     I3C_IER_MRLUPDIE
#define LL_I3C_IER_RSTIE        I3C_IER_RSTIE
#define LL_I3C_IER_ASUPDIE      I3C_IER_ASUPDIE
#define LL_I3C_IER_INTUPDIE     I3C_IER_INTUPDIE
#define LL_I3C_IER_DEFIE        I3C_IER_DEFIE
#define LL_I3C_IER_GRPIE        I3C_IER_GRPIE
#define LL_I3C_CTRL_TX_IT       (I3C_IER_FCIE | I3C_IER_CFNFIE | I3C_IER_TXFNFIE | I3C_IER_ERRIE) /* Ctrl TX IT */
#define LL_I3C_CTRL_RX_IT       (I3C_IER_FCIE | I3C_IER_CFNFIE | I3C_IER_RXFNEIE | I3C_IER_ERRIE) /* Ctrl RX IT */
#define LL_I3C_CTRL_DAA_IT      (I3C_IER_FCIE | I3C_IER_CFNFIE | I3C_IER_TXFNFIE | I3C_IER_ERRIE) /* Ctrl DAA  IT */
#define LL_I3C_TGT_TX_IT        (I3C_IER_FCIE | I3C_IER_TXFNFIE | I3C_IER_ERRIE) /*!< Tgt TX IT  */
#define LL_I3C_TGT_RX_IT        (I3C_IER_FCIE | I3C_IER_RXFNEIE | I3C_IER_ERRIE)  /*!< Tgt RX IT  */
#define LL_I3C_TGT_CTRLROLE_IT  (I3C_IER_CRUPDIE | I3C_IER_ERRIE)  /*!< Tgt Controller-Role IT */
#define LL_I3C_TGT_HOTJOIN_IT   (I3C_IER_DAUPDIE | I3C_IER_ERRIE)  /*!< Tgt Hot-Join IT */
#define LL_I3C_TGT_IBI_IT       (I3C_IER_IBIENDIE | I3C_IER_ERRIE)  /*!< Tgt IBI IT */
#define LL_I3C_XFER_DMA         (I3C_IER_FCIE | I3C_IER_ERRIE)  /*!< Tgt or Ctrl DMA IT */
#define LL_I3C_IER_ALL          0xFFFFFFFFU  /*!< All */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_MASK_IT MASKS IT Defines
  * @brief    Masks interrupt defines which can be used with LL_I3C_READ_REG and LL_I3C_WRITE_REG functions
  * @{
  */
#define LL_I3C_MISR_CFNFMIS       I3C_MISR_CFNFMIS
#define LL_I3C_MISR_SFNEMIS       I3C_MISR_SFNEMIS
#define LL_I3C_MISR_TXFNFMIS      I3C_MISR_TXFNFMIS
#define LL_I3C_MISR_RXFNEMIS      I3C_MISR_RXFNEMIS
#define LL_I3C_MISR_FCMIS         I3C_MISR_FCMIS
#define LL_I3C_MISR_RXTGTENDMIS   I3C_MISR_RXTGTENDMIS
#define LL_I3C_MISR_ERRMIS        I3C_MISR_ERRMIS
#define LL_I3C_MISR_IBIMIS        I3C_MISR_IBIMIS
#define LL_I3C_MISR_IBIENDMIS     I3C_MISR_IBIENDMIS
#define LL_I3C_MISR_CRMIS         I3C_MISR_CRMIS
#define LL_I3C_MISR_CRUPDMIS      I3C_MISR_CRUPDMIS
#define LL_I3C_MISR_HJMIS         I3C_MISR_HJMIS
#define LL_I3C_MISR_WKPMIS        I3C_MISR_WKPMIS
#define LL_I3C_MISR_GETMIS        I3C_MISR_GETMIS
#define LL_I3C_MISR_STAMIS        I3C_MISR_STAMIS
#define LL_I3C_MISR_DAUPDMIS      I3C_MISR_DAUPDMIS
#define LL_I3C_MISR_MWLUPDMIS     I3C_MISR_MWLUPDMIS
#define LL_I3C_MISR_MRLUPDMIS     I3C_MISR_MRLUPDMIS
#define LL_I3C_MISR_RSTMIS        I3C_MISR_RSTMIS
#define LL_I3C_MISR_ASUPDMIS      I3C_MISR_ASUPDMIS
#define LL_I3C_MISR_INTUPDMIS     I3C_MISR_INTUPDMIS
#define LL_I3C_MISR_DEFMIS        I3C_MISR_DEFMIS
#define LL_I3C_MISR_GRPMIS        I3C_MISR_GRPMIS
/**
  * @}
  */

/** @defgroup I3C_LL_EC_MODE MODE
  * @{
  */
#define LL_I3C_MODE_CONTROLLER   I3C_CFGR_CRINIT  /*!< I3C controller mode */
#define LL_I3C_MODE_TARGET       0x00000000U      /*!< I3C target (controller capable) mode */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_DMA_REG_DATA DMA Register data
  * @{
  */
#define LL_I3C_DMA_REG_DATA_TRANSMIT_BYTE  0x0U  /*!< Get addr of data register used for transmission in byte */
#define LL_I3C_DMA_REG_DATA_RECEIVE_BYTE   0x1U  /*!< Get addr of data register used for reception in byte */
#define LL_I3C_DMA_REG_DATA_TRANSMIT_WORD  0x2U  /*!< Get addr of data register used for transmission in Word */
#define LL_I3C_DMA_REG_DATA_RECEIVE_WORD   0x3U  /*!< Get addr of data register used for reception in Word */
#define LL_I3C_DMA_REG_STATUS              0x4U  /*!< Get addr of status register used for transfer status in Word */
#define LL_I3C_DMA_REG_CONTROL             0x5U  /*!< Get addr of control register used for transfer control in Word */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_RX_THRESHOLD Rx threshold
  * @{
  */
#define LL_I3C_RXFIFO_THRESHOLD_1_8   0x00000000U      /*!< Rx FIFO threshold is 1 byte in a FIFO depth of 8 bytes */
#define LL_I3C_RXFIFO_THRESHOLD_1_2   I3C_CFGR_RXTHRES /*!< Rx FIFO threshold is 1 word in a FIFO depth of 2 word */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_TX_THRESHOLD Tx threshold
  * @{
  */
#define LL_I3C_TXFIFO_THRESHOLD_1_8   0x00000000U      /*!< Tx FIFO threshold is 1 byte in a FIFO depth of 8 bytes */
#define LL_I3C_TXFIFO_THRESHOLD_1_2   I3C_CFGR_TXTHRES /*!< Tx FIFO threshold is 1 word in a FIFO depth of 2 words */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_CTRL_FIFO_CONFIG Controller fifo configuration
  * @{
  */
#define LL_I3C_CTRL_FIFO_NONE         0                                 /*!< Control and status FIFO disable */
#define LL_I3C_CTRL_FIFO_CONTROL_ONLY I3C_CFGR_TMODE                    /*!< Control FIFO enable */
#define LL_I3C_CTRL_FIFO_STATUS_ONLY  I3C_CFGR_SMODE                    /*!< Status FIFO enable */
#define LL_I3C_CTRL_FIFO_ALL          (I3C_CFGR_SMODE | I3C_CFGR_TMODE) /*!< Control and status FIFO enable */


/** @defgroup I3C_LL_EC_End_Of_FrameCompletion End of frame completion
  * @{
  */
#define LL_I3C_END_OF_FRAME_CPLT_DISABLE  I3C_CFGR_FCFDIS /*!< Frame complete flag is clear by HW */
#define LL_I3C_END_OF_FRAME_CPLT_ENABLE   0x00000000U /*!< Frame complete flag must be clear by SW (default config) */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_PAYLOAD Payload
  * @{
  */
#define LL_I3C_PAYLOAD_EMPTY   0x00000000U  /*!< Empty payload, no additional data after IBI ack */
#define LL_I3C_PAYLOAD_1_BYTE  I3C_MAXRLR_IBIP_0                        /*!< 1 additional data byte after IBI ack */
#define LL_I3C_PAYLOAD_2_BYTE  I3C_MAXRLR_IBIP_1                        /*!< 2 additional data bytes after IBI ack */
#define LL_I3C_PAYLOAD_3_BYTE  (I3C_MAXRLR_IBIP_1 | I3C_MAXRLR_IBIP_0)  /*!< 3 additional data bytes after IBI ack */
#define LL_I3C_PAYLOAD_4_BYTE  I3C_MAXRLR_IBIP_2                        /*!< 4 additional data bytes after IBI ack */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_SDA_HOLD_TIME SDA HOLD TIME 0
  * @{
  */
#define LL_I3C_SDA_HOLD_TIME_0_5  0x00000000U                                     /*!< SDA hold time is 0.5 x ti3cclk */
#define LL_I3C_SDA_HOLD_TIME_1_5  I3C_TIMINGR1_SDA_HD_0                           /*!< SDA hold time is 1.5 x ti3cclk */
#define LL_I3C_SDA_HOLD_TIME_2_5  I3C_TIMINGR1_SDA_HD_1                           /*!< SDA hold time is 2.5 x ti3cclk */
#define LL_I3C_SDA_HOLD_TIME_3_5  (I3C_TIMINGR1_SDA_HD_1 | I3C_TIMINGR1_SDA_HD_0) /*!< SDA hold time is 3.5 x ti3cclk */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_OWN_ACTIVITY_STATE Own activity state
  * @{
  */
#define LL_I3C_OWN_ACTIVITY_STATE_0  0x00000000U                                   /*!< Own ctrl activity state 0 */
#define LL_I3C_OWN_ACTIVITY_STATE_1  I3C_TIMINGR1_ASNCR_0                          /*!< Own ctrl activity state 1 */
#define LL_I3C_OWN_ACTIVITY_STATE_2  I3C_TIMINGR1_ASNCR_1                          /*!< Own ctrl activity state 2 */
#define LL_I3C_OWN_ACTIVITY_STATE_3  (I3C_TIMINGR1_ASNCR_1 | I3C_TIMINGR1_ASNCR_0) /*!< Own ctrl activity state 3 */
/**
  * @}
  */


/** @defgroup I3C_LL_EC_MAX_DATA_SPEED_LIMITATION Max data speed limitation
  * @{
  */
#define LL_I3C_NO_DATA_SPEED_LIMITATION   0x00000000U   /*!< No max data speed limitation */
#define LL_I3C_MAX_DATA_SPEED_LIMITATION  I3C_BCR_BCR0  /*!< Max data speed limitation */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_IBI_REQUEST_CAPABILITY In-Band Interrupt (IBI) request capability
  * @{
  */
#define LL_I3C_IBI_REQUEST_NOT_SUPPORTED 0x00000000U /*!< IBI request not supported (BCR1 = 0) */
#define LL_I3C_IBI_REQUEST_SUPPORTED     I3C_BCR_BCR1 /*!< IBI request supported (BCR1 = 1) */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_IBI_NO_ADDITIONAL IBI no additional
  * @{
  */
#define LL_I3C_IBI_NO_ADDITIONAL_DATA  0x00000000U   /*!< No data byte follows the accepted IBI */
#define LL_I3C_IBI_ADDITIONAL_DATA     I3C_BCR_BCR2  /*!< A Mandatory data Byte (MDB) follows the accepted IBI */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_OFFLINE_CAPABLE Offline capable
  * @{
  */
#define LL_I3C_NO_OFFLINE_CAPABLE  0x00000000U   /*!< No Offline capable */
#define LL_I3C_OFFLINE_CAPABLE     I3C_BCR_BCR3  /*!< Offline capable */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_VIRTUAL_TARGET Virtual target support
  * @{
  */
#define LL_I3C_VIRTUAL_TARGET_NOT_SUPPORTED 0x00000000U /*!< Virtual target not supported (BCR4 = 0) */
#define LL_I3C_VIRTUAL_TARGET_SUPPORTED     I3C_BCR_BCR4 /*!< Virtual target supported (BCR4 = 1) */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_ADV_CAPABILITIES Advanced capabilities
  * @{
  */
#define LL_I3C_ADV_CAP_NOT_SUPPORTED 0x00000000U /*!< Advanced capabilities not supported (BCR5 = 0) */
#define LL_I3C_ADV_CAP_SUPPORTED     I3C_BCR_BCR5 /*!< Advanced capabilities supported (BCR5 = 1) */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_DEVICE_ROLE_AS Device role as
  * @{
  */
#define LL_I3C_DEVICE_ROLE_AS_TARGET      0x00000000U   /*!< I3C target */
#define LL_I3C_DEVICE_ROLE_AS_CONTROLLER  I3C_BCR_BCR6  /*!< I3C controller */
/**
  * @}
  */


/** @defgroup I3C_LL_EC_IBI_MDB_READ_NOTIFICATION IBI mdb read notification
  * @{
  */
#define LL_I3C_MDB_NO_PENDING_READ_NOTIFICATION  0x00000000U  /*!< No support of pending read notification via
                                                                   the IBI MDB[7:0] value */
#define LL_I3C_MDB_PENDING_READ_NOTIFICATION     I3C_GETCAPR_CAPPEND  /*!< Support of pending read notification via
                                                                           the IBI MDB[7:0] value */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_HANDOFF_GRP_ADDR_NOT Handoff grp addr not
  * @{
  */
#define LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED 0x00000000U        /*!< Group address handoff is not supported */
#define LL_I3C_HANDOFF_GRP_ADDR_SUPPORTED     I3C_CRCAPR_CAPGRP  /*!< Group address handoff is supported */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_HANDOFF Handoff
  * @{
  */
#define LL_I3C_HANDOFF_NOT_DELAYED  0x00000000U         /*!< Additional time to controllership handoff is not needed */
#define LL_I3C_HANDOFF_DELAYED      I3C_CRCAPR_CAPDHOFF /*!< Additional time to controllership handoff is needed */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_HANDOFF_ACTIVITY_STATE Handoff activity state
  * @{
  */
#define LL_I3C_HANDOFF_ACTIVITY_STATE_0   0x00000000U /*!< Activity state 0 after controllership handoff */
#define LL_I3C_HANDOFF_ACTIVITY_STATE_1   I3C_GETMXDSR_HOFFAS_0 /*!< Activity state 1 after controllership handoff */
#define LL_I3C_HANDOFF_ACTIVITY_STATE_2   I3C_GETMXDSR_HOFFAS_1 /*!< Activity state 2 after controllership handoff */
#define LL_I3C_HANDOFF_ACTIVITY_STATE_3   (I3C_GETMXDSR_HOFFAS_1 | I3C_GETMXDSR_HOFFAS_0) /*!< Activity state 3 after
                                                                                               controllership handoff */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_GETMXDS_FORMAT Getmxds format
  * @{
  */
#define LL_I3C_GETMXDS_FORMAT_1      0x00000000U /*!< GETMXDS CCC format 1 is used, no MaxRdTurn field in response */
#define LL_I3C_GETMXDS_FORMAT_2_LSB  I3C_GETMXDSR_FMT_0 /*!< GETMXDS CCC format 2 is used, MaxRdTurn field in response,
                                                             LSB = RDTURN[7:0] */
#define LL_I3C_GETMXDS_FORMAT_2_MID  I3C_GETMXDSR_FMT_1 /*!< GETMXDS CCC format 2 is used, MaxRdTurn field in response,
                                                             middle byte = RDTURN[7:0] */
#define LL_I3C_GETMXDS_FORMAT_2_MSB  (I3C_GETMXDSR_FMT_1 | I3C_GETMXDSR_FMT_0) /*!< GETMXDS CCC format 2 is used,
                                                                       MaxRdTurn field in response, MSB = RDTURN[7:0] */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_GETMXDS_TSCO Getmxds TSCO
  * @{
  */
#define LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS     0x00000000U       /*!< clock-to-data turnaround time tSCO <= 12ns */
#define LL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS  I3C_GETMXDSR_TSCO /*!< clock-to-data turnaround time tSCO > 12ns */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_BUS_ACTIVITY_STATE Bus activity state
  * @{
  */
#define LL_I3C_BUS_ACTIVITY_STATE_0    0x00000000U                       /*!< Controller on the bus activity state 0 */
#define LL_I3C_BUS_ACTIVITY_STATE_1    I3C_DEVR0_AS_0                    /*!< Controller on the bus activity state 1 */
#define LL_I3C_BUS_ACTIVITY_STATE_2    I3C_DEVR0_AS_1                    /*!< Controller on the bus activity state 2 */
#define LL_I3C_BUS_ACTIVITY_STATE_3    (I3C_DEVR0_AS_1 | I3C_DEVR0_AS_0) /*!< Controller on the bus activity state 3 */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_RESET_ACTION Reset action
  * @{
  */
#define LL_I3C_RESET_ACTION_NONE     0x00000000U        /*!< No reset action required */
#define LL_I3C_RESET_ACTION_PARTIAL  I3C_DEVR0_RSTACT_0 /*!< Reset of some internal registers of the peripheral */
#define LL_I3C_RESET_ACTION_FULL     I3C_DEVR0_RSTACT_1 /*!< Reset all internal registers of the peripheral */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_DIRECTION direction
  * @{
  */
#define LL_I3C_DIRECTION_WRITE  0x00000000U   /*!< Write transfer */
#define LL_I3C_DIRECTION_READ   I3C_CR_RNW    /*!< Read transfer */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_GENERATE Generate
  * @{
  */
#define LL_I3C_GENERATE_STOP     I3C_CR_MEND  /*!< Generate stop condition after sending a message */
#define LL_I3C_GENERATE_RESTART  0x00000000U  /*!< Generate Restart condition after sending a message */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_CONTROLLER_MTYPE Controller mtype
  * @{
  */
#define LL_I3C_CONTROLLER_MTYPE_RELEASE     0x00000000U                        /*!< SCL output clock stops running
                                                                                    until next instruction executed */
#define LL_I3C_CONTROLLER_MTYPE_HEADER      I3C_CR_MTYPE_0                     /*!< Header message */
#define LL_I3C_CONTROLLER_MTYPE_PRIVATE     I3C_CR_MTYPE_1                     /*!< Private message Type */
#define LL_I3C_CONTROLLER_MTYPE_DIRECT      (I3C_CR_MTYPE_1 | I3C_CR_MTYPE_0)  /*!< Direct message Type */
#define LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C  I3C_CR_MTYPE_2                     /*!< Legacy I2C message Type */
#define LL_I3C_CONTROLLER_MTYPE_CCC         (I3C_CR_MTYPE_2 | I3C_CR_MTYPE_1)  /*!< Common command Code */
/**
  * @}
  */

/** @defgroup I3C_LL_DEFINE_BYTE Define byte
  * @{
  */
#define LL_I3C_DEFINE_BYTE (0x00000001U) /*!< Define used to construct enum @ref hal_i3c_transfer_mode_t */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_TARGET_MTYPE Target mtype
  * @{
  */
#define LL_I3C_TARGET_MTYPE_HOT_JOIN             I3C_CR_MTYPE_3                     /*!< Hot-Join */
#define LL_I3C_TARGET_MTYPE_CONTROLLER_ROLE_REQ  (I3C_CR_MTYPE_3 | I3C_CR_MTYPE_0)  /*!< Controller-role request */
#define LL_I3C_TARGET_MTYPE_IBI                  (I3C_CR_MTYPE_3 | I3C_CR_MTYPE_1)  /*!< In Band interrupt (IBI) */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_MESSAGE Message
  * @{
  */
#define LL_I3C_MESSAGE_ERROR    0x00000000U   /*!< An error has been detected in the message */
#define LL_I3C_MESSAGE_SUCCESS  I3C_SR_OK     /*!< The message ended with success */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_MESSAGE_DIRECTION Message direction
  * @{
  */
#define LL_I3C_MESSAGE_DIRECTION_WRITE  0x00000000U  /*!< write data or command */
#define LL_I3C_MESSAGE_DIRECTION_READ   I3C_SR_DIR   /*!< read data */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_CONTROLLER_ERROR Controller error
  * @{
  */
#define LL_I3C_CONTROLLER_ERROR_CE0  0x00000000U       /*!< Ctrl detected an illegally formatted CCC */
#define LL_I3C_CONTROLLER_ERROR_CE1  I3C_SER_CODERR_0 /*!< Ctrl detected that Tx data are different than expected */
#define LL_I3C_CONTROLLER_ERROR_CE2  I3C_SER_CODERR_1 /*!< Ctrl detected that broadcast address 7'h7E has been NACKed */
#define LL_I3C_CONTROLLER_ERROR_CE3  (I3C_SER_CODERR_1 | I3C_SER_CODERR_0) /*!< Ctrl detected that new ctrl did not
                                                                          drive the bus after ctrl-role handoff */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_TARGET_ERROR Target error
  * @{
  */
#define LL_I3C_TARGET_ERROR_TE0  I3C_SER_CODERR_3                      /*!< Tgt detected an invalid broadcast address */
#define LL_I3C_TARGET_ERROR_TE1  (I3C_SER_CODERR_3 | I3C_SER_CODERR_0) /*!< Tgt detected an invalid CCC Code */
#define LL_I3C_TARGET_ERROR_TE2  (I3C_SER_CODERR_3 | I3C_SER_CODERR_1) /*!< Tgt detected an invalid write data */
#define LL_I3C_TARGET_ERROR_TE3  (I3C_SER_CODERR_3 | I3C_SER_CODERR_1 | I3C_SER_CODERR_0) /*!< Tgt detected an invalid
                                                                                         assigned address during DAA */
#define LL_I3C_TARGET_ERROR_TE4  (I3C_SER_CODERR_3 | I3C_SER_CODERR_2) /*!< Tgt detected 7'h7E missing after restart
                                                                            during DAA */
#define LL_I3C_TARGET_ERROR_TE5  (I3C_SER_CODERR_3 | I3C_SER_CODERR_2 | I3C_SER_CODERR_0) /*!< Tgt detected an illegally
                                                                                               formatted CCC */
#define LL_I3C_TARGET_ERROR_TE6  (I3C_SER_CODERR_3 | I3C_SER_CODERR_2 | I3C_SER_CODERR_1) /*!< Tgt detected  that
                                                               transmitted data on the bus is different than expected */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_IBI_CAPABILITY In Band Interrupt capability
  * @{
  */
#define LL_I3C_IBI_CAPABILITY     I3C_DEVRX_IBIACK /*!< Ctrl acknowledge tgt IBI capable */
#define LL_I3C_IBI_NO_CAPABILITY  0x00000000U      /*!< Ctrl no acknowledge tgt IBI capable */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_IBI_ADDITIONAL_DATA In Band Interrupt additional data
  * @{
  */
#define LL_I3C_IBI_DATA_ENABLE   I3C_DEVRX_IBIDEN /*!< A mandatory data byte follows the IBI acknowledgement */
#define LL_I3C_IBI_DATA_DISABLE  0x00000000U      /*!< No mandatory data byte follows the IBI acknowledgement */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_CR_CAPABILITY CR capability
  * @{
  */
#define LL_I3C_CR_CAPABILITY     I3C_DEVRX_CRACK  /*!< Ctrl acknowledge tgt ctrl-role capable */
#define LL_I3C_CR_NO_CAPABILITY  0x00000000U      /*!< Ctrl no acknowledge tgt ctrl-role capable */
/**
  * @}
  */

/** @defgroup I3C_LL_EC_SUSPEND_ON_IBI Suspend behavior on IBI
  * @{
  */
#define LL_I3C_SUSP_ENABLE  I3C_DEVRX_SUSP /*!< Suspend current transfer: emit STOP and flush C-FIFO/TX-FIFO after IBI */
#define LL_I3C_SUSP_DISABLE 0x00000000U    /*!< Do not suspend: normal next control flow (restart/stop depends on CR) */
/**
  * @}
  */


/** @defgroup LL_I3C_PAYLOAD_BIT_DEFINITION Payload bit definition
  * @{
  */

/* Private define for CCC command */
#define LL_I3C_BROADCAST_RSTDAA     (0x00000006U) /*!< Bit definition to manage RSTDAA */
#define LL_I3C_BROADCAST_ENTDAA     (0x00000007U) /*!< Bit definition to manage ENTDAA */

/* Private define to split ENTDAA payload */
#define LL_I3C_BCR_IN_PAYLOAD_SHIFT 48 /*!< BCR Position can be used with in ENTDAA payload */
#define LL_I3C_DCR_IN_PAYLOAD_SHIFT 56 /*!< DCR Position can be used with ENTDAA payload */
#define LL_I3C_PID_IN_PAYLOAD_MASK  0xFFFFFFFFFFFFU /*!< Mask can be combined with ENTDAA payload */

/* Private define to split PID */
/* Bits[47:33]: MIPI Manufacturer ID */
#define LL_I3C_MIPIMID_PID_SHIFT    33      /*!< MIPIMID Position can be used with PID */
#define LL_I3C_MIPIMID_PID_MASK     0x7FFFU /*!< Mask can be combined with PID */

/* Bit[32]: Provisioned ID Type Selector */
#define LL_I3C_IDTSEL_PID_SHIFT     32      /*!< IDTSEL Position can be used with PID */
#define LL_I3C_IDTSEL_PID_MASK      0x01U   /*!< Mask can be combined with PID */

/* Bits[31:16]: Part ID */
#define LL_I3C_PART_ID_PID_SHIFT    16      /*!< Part ID Position can be used with PID */
#define LL_I3C_PART_ID_PID_MASK     0xFFFFU /*!< Mask can be combined with PID */

/* Bits[15:12]: MIPI Instance ID */
#define LL_I3C_MIPIID_PID_SHIFT     12      /*!< MIPI Instance ID Position can be used with PID */
#define LL_I3C_MIPIID_PID_MASK      0xFU    /*!< Mask can be combined with PID */

/*  MIPI BCR Bits */
#define LL_I3C_BCR_BCR_POS          (0U)                           /*!< Bus Characteristics */
#define LL_I3C_BCR_BCR_MSK          (0xFFUL << LL_I3C_BCR_BCR_POS) /*!< 0x000000FF */
#define LL_I3C_BCR_BCR0_POS         (0U)                           /*!< Max Data Speed Limitation */
#define LL_I3C_BCR_BCR0_MSK         (0x1UL << LL_I3C_BCR_BCR0_POS) /*!< 0x00000001 */
#define LL_I3C_BCR_BCR1_POS         (1U)                           /*!< IBI Request capable */
#define LL_I3C_BCR_BCR1_MSK         (0x1UL << LL_I3C_BCR_BCR1_POS) /*!< 0x00000002 */
#define LL_I3C_BCR_BCR2_POS         (2U)                           /*!< IBI Payload additional Mandatory Data Byte */
#define LL_I3C_BCR_BCR2_MSK         (0x1UL << LL_I3C_BCR_BCR2_POS) /*!< 0x00000004 */
#define LL_I3C_BCR_BCR3_POS         (3U)                           /*!< Offline capable */
#define LL_I3C_BCR_BCR3_MSK         (0x1UL << LL_I3C_BCR_BCR3_POS) /*!< 0x00000008 */
#define LL_I3C_BCR_BCR4_POS         (4U)                           /*!< Virtual target support */
#define LL_I3C_BCR_BCR4_MSK         (0x1UL << LL_I3C_BCR_BCR4_POS) /*!< 0x00000010 */
#define LL_I3C_BCR_BCR5_POS         (5U)                           /*!< Advanced capabilities */
#define LL_I3C_BCR_BCR5_MSK         (0x1UL << LL_I3C_BCR_BCR5_POS) /*!< 0x00000020 */
#define LL_I3C_BCR_BCR6_POS         (6U)                           /*!< Device role shared during Dyn Addr Assignment */
#define LL_I3C_BCR_BCR6_MSK         (0x1UL << LL_I3C_BCR_BCR6_POS) /*!< 0x00000040 */
/**
  * @}
  */


/** @defgroup LL_I3C_STALL_ENABLE_BIT_DEFINITION stall enable bit definition
  * @{
  */
#define LL_I3C_CTRL_STALL_ACK      I3C_TIMINGR2_STALLA /*!< Inserts a stall after each address or data byte
                                                            ACK/NACK extend target processing time. */

#define LL_I3C_CTRL_STALL_CCC      I3C_TIMINGR2_STALLC /*!< Inserts a stall on the parity (T) bit phase of a
                                                            direct/broadcast CCC so targets can decode the
                                                            opcode before data phase. */

#define LL_I3C_CTRL_STALL_TX       I3C_TIMINGR2_STALLD /*!< Inserts a stall on the parity (T) bit of transmitted
                                                            data allowing target additional time to latch received
                                                            byte. */

#define LL_I3C_CTRL_STALL_RX       I3C_TIMINGR2_STALLT /*!< Inserts a stall before the controller samples target
                                                            data (read path) so target can prepare next byte. */

#define LL_I3C_CTRL_STALL_I2C_ACK  I3C_TIMINGR2_STALLL /*!< Inserts a stall after address ACK/NACK in legacy I2C
                                                            read/write so target can prepare for next phase. */

#define LL_I3C_CTRL_STALL_I2C_TX   I3C_TIMINGR2_STALLS /*!< Inserts a stall after data ACK/NACK in legacy I2C write
                                                            allowing target to process/write incoming byte. */

#define LL_I3C_CTRL_STALL_I2C_RX   I3C_TIMINGR2_STALLR /*!< Inserts a stall after data ACK/NACK in legacy I2C read
                                                            so target can load the next byte to transmit. */

#define LL_I3C_CTRL_STALL_ALL      (LL_I3C_CTRL_STALL_ACK | LL_I3C_CTRL_STALL_CCC | LL_I3C_CTRL_STALL_TX \
                                    | LL_I3C_CTRL_STALL_RX | LL_I3C_CTRL_STALL_I2C_ACK | LL_I3C_CTRL_STALL_I2C_TX \
                                    | LL_I3C_CTRL_STALL_I2C_RX) /*!< Enable all stall points. */
#define LL_I3C_CTRL_STALL_NONE     0UL                    /*!< Disable all stall insertion (default high throughput). */
/**
  * @}
  */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I3C_LL_Exported_Macros LL I3C Macros
  * @{
  */

/** @defgroup I3C_LL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/** @brief  Get Bus Characteristics from payload (64bits) receive during ENTDAA procedure.
  * @param  PAYLOAD specifies the Bus Characteristics capabilities retrieve during ENTDAA procedure.
  *         This parameter must be a number between Min_Data=0x00(uint64_t) and Max_Data=0xFFFFFFFFFFFFFFFFFF.
  * @retval The value of BCR Return value between Min_Data=0x00 and Max_Data=0xFF.
  */
#define LL_I3C_GET_BCR(PAYLOAD) (((uint32_t)((uint64_t)(PAYLOAD) >> LL_I3C_BCR_IN_PAYLOAD_SHIFT)) & I3C_BCR_BCR)

/** @brief  Check IBI request capabilities.
  * @param  BCR specifies the Bus Characteristics capabilities retrieve during ENTDAA procedure.
  *         This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @retval Value of @ref I3C_LL_EC_IBI_CAPABILITY.
  */
#define LL_I3C_GET_IBI_CAPABLE(BCR) (((((BCR) & I3C_BCR_BCR1_Msk) >> I3C_BCR_BCR1_Pos) == 1U) \
                                     ? LL_I3C_IBI_CAPABILITY : LL_I3C_IBI_NO_CAPABILITY)

/** @brief  Check IBI additional data byte capabilities.
  * @param  BCR specifies the Bus Characteristics capabilities retrieve during ENTDAA procedure.
  *         This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @retval Value of @ref I3C_LL_EC_IBI_ADDITIONAL_DATA.
  */
#define LL_I3C_GET_IBI_PAYLOAD(BCR) (((((BCR) & I3C_BCR_BCR2_Msk) >> I3C_BCR_BCR2_Pos) == 1U) \
                                     ? LL_I3C_IBI_DATA_ENABLE : LL_I3C_IBI_DATA_DISABLE)

/** @brief  Check controller role request capabilities.
  * @param  BCR specifies the Bus Characteristics capabilities retrieve during ENTDAA procedure.
  *         This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @retval Value of @ref I3C_LL_EC_CR_CAPABILITY.
  */
#define LL_I3C_GET_CR_CAPABLE(BCR) (((((BCR) & I3C_BCR_BCR6_Msk) >> I3C_BCR_BCR6_Pos) == 1U) \
                                    ? LL_I3C_CR_CAPABILITY : LL_I3C_CR_NO_CAPABILITY)

/**
  * @brief  Write a value in I3C register.
  * @param  INSTANCE I3C Instance
  * @param  REG Register to be written
  * @param  VALUE Value to be written in the register
  */
#define LL_I3C_WRITE_REG(INSTANCE, REG, VALUE) STM32_WRITE_REG((INSTANCE)->REG, (VALUE))

/**
  * @brief  Read a value in I3C register.
  * @param  INSTANCE I3C Instance
  * @param  REG Register to be read
  * @retval Register value
  */
#define LL_I3C_READ_REG(INSTANCE, REG) STM32_READ_REG((INSTANCE)->REG)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup I3C_LL_Exported_Functions LL I3C Functions
  * @{
  */

/** @defgroup I3C_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable I3C peripheral (EN = 1).
  * @rmtoll
  *  CFGR      EN            LL_I3C_Enable
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_Enable(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_EN);
}

/**
  * @brief  Disable I3C peripheral (EN = 0).
  * @rmtoll
  *  CFGR      EN            LL_I3C_Disable
  * @param  p_i3c I3C Instance.
  * @note   Controller mode: before clearing EN, all possible target requests must be disabled using DISEC CCC.
  *         Target mode: software is not expected clearing EN unless a partial reset of the IP is needed
  */
__STATIC_INLINE void LL_I3C_Disable(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_EN);
}

/**
  * @brief  Check if the I3C peripheral is enabled or disabled.
  * @rmtoll
  *  CFGR      EN            LL_I3C_IsEnabled
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabled(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_EN) == (I3C_CFGR_EN)) ? 1UL : 0UL);
}

/**
  * @brief  Return raw CFGR register value.
  * @rmtoll CFGR * LL_I3C_GetRegister_CFGR
  * @param  p_i3c I3C Instance.
  * @retval 32-bit CFGR register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_CFGR(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->CFGR);
}

/**
  * @brief  Return raw IER register value.
  * @rmtoll IER *  LL_I3C_GetEnabledIT
  * @param  p_i3c I3C Instance.
  * @retval 32-bit IER register content.
  */
__STATIC_INLINE uint32_t  LL_I3C_GetEnabledIT(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->IER);
}

/**
  * @brief  Return masked EVR flags state.
  * @rmtoll EVR * LL_I3C_GetFlag
  * @param  p_i3c I3C Instance.
  * @param  flags Mask of EVR bits to check (OR-combination of EVR flags).
  * @retval Masked EVR value (non-zero bits indicate set flags).
  */
__STATIC_INLINE uint32_t LL_I3C_GetFlag(const I3C_TypeDef *p_i3c, uint32_t flags)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->EVR) & (uint32_t)flags);
}

/**
  * @brief  Return raw GETMXDSR register value.
  * @rmtoll GETMXDSR * LL_I3C_GetRegister_GETMXDSR
  * @param  p_i3c I3C Instance.
  * @retval 32-bit GETMXDSR register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_GETMXDSR(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->GETMXDSR);
}

/**
  * @brief  Return raw MISR register value.
  * @rmtoll MISR * LL_I3C_GetRegister_MISR
  * @param  p_i3c I3C Instance.
  * @retval 32-bit MISR register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_MISR(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->MISR);
}

/**
  * @brief  Return raw BCR register value.
  * @rmtoll BCR * LL_I3C_GetRegister_BCR
  * @param  p_i3c I3C Instance.
  * @retval 32-bit BCR register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_BCR(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->BCR);
}

/**
  * @brief  Return raw SER register value.
  * @rmtoll SER * LL_I3C_GetRegister_SER
  * @param  p_i3c I3C Instance.
  * @retval 32-bit SER register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_SER(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->SER);
}

/**
  * @brief  Return raw TIMINGR2 register value.
  * @rmtoll TIMINGR2 * LL_I3C_GetRegister_TIMINGR2
  * @param  p_i3c I3C Instance.
  * @retval 32-bit TIMINGR2 register content.
  */
__STATIC_INLINE uint32_t LL_I3C_GetRegister_TIMINGR2(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_REG(p_i3c->TIMINGR2);
}

/**
  * @brief  Write raw value into CR register.
  * @rmtoll CR * LL_I3C_SetRegister_CR
  * @param  p_i3c I3C Instance.
  * @param  cr_value 32-bit value to program in CR.
  */
__STATIC_INLINE void LL_I3C_SetRegister_CR(I3C_TypeDef *p_i3c, uint32_t cr_value)
{
  LL_I3C_WRITE_REG(p_i3c, CR, cr_value);
}

/**
  * @brief  Check if reset action is required or not required.
  * @rmtoll
  *  DEVR0        RSTVAL        LL_I3C_IsEnabledReset
  * @param  p_i3c I3C Instance.
  * @note   This bit indicates if reset action field has been updated by HW upon reception
  *         of RSTACT during current frame.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledReset(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_RSTVAL) == (I3C_DEVR0_RSTVAL)) ? 1UL : 0UL);
}

/**
  * @brief  Configure peripheral mode.
  * @rmtoll
  *  CFGR      CRINIT       LL_I3C_SetMode
  * @param  p_i3c I3C Instance.
  * @param  peripheral_mode This parameter can be one of the following values:
  *         @arg @ref LL_I3C_MODE_CONTROLLER
  *         @arg @ref LL_I3C_MODE_TARGET
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetMode(I3C_TypeDef *p_i3c, uint32_t peripheral_mode)
{
  STM32_MODIFY_REG(p_i3c->CFGR, I3C_CFGR_CRINIT, peripheral_mode);
}

/**
  * @brief  Get peripheral mode.
  * @rmtoll
  *  CFGR      CRINIT       LL_I3C_GetMode
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_MODE_CONTROLLER
  *         @arg @ref LL_I3C_MODE_TARGET
  */
__STATIC_INLINE uint32_t LL_I3C_GetMode(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_CRINIT);
}

/**
  * @brief  An arbitration header (7'h7E) is sent after start in case of legacy I2C or I3C private transfers.
  * @rmtoll
  *  CFGR      NOARBH        LL_I3C_EnableArbitrationHeader
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_EnableArbitrationHeader(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_NOARBH);
}

/**
  * @brief  Target address is sent directly after a start in case of legacy I2C or I3C private transfers.
  * @rmtoll
  *  CFGR      NOARBH        LL_I3C_DisableArbitrationHeader
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing

  */
__STATIC_INLINE void LL_I3C_DisableArbitrationHeader(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_NOARBH);
}

/**
  * @brief  Check if the arbitration header is enabled or disabled.
  * @rmtoll
  *  CFGR      NOARBH        LL_I3C_IsEnabledArbitrationHeader
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledArbitrationHeader(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_NOARBH) == (I3C_CFGR_NOARBH)) ? 0UL : 1UL);
}

/**
  * @brief  A reset pattern is inserted before the STOP at the end of a frame when the last CCC
  *         of the frame was RSTACT CCC.
  * @rmtoll
  *  CFGR      RSTPTRN       LL_I3C_EnableResetPattern
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_EnableResetPattern(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_RSTPTRN);
}

/**
  * @brief  A single STOP is emitted at the end of a frame.
  * @rmtoll
  *  CFGR      RSTPTRN       LL_I3C_DisableResetPattern
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_DisableResetPattern(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_RSTPTRN);
}

/**
  * @brief  Check if reset pattern is enabled or disabled.
  * @rmtoll
  *  CFGR      RSTPTRN       LL_I3C_IsEnabledResetPattern
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledResetPattern(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_RSTPTRN) == (I3C_CFGR_RSTPTRN)) ? 1UL : 0UL);
}

/**
  * @brief  An exit pattern is sent after header (MTYPE = header) to program an escalation fault.
  * @rmtoll
  *  CFGR      EXITPTRN      LL_I3C_EnableExitPattern
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_EnableExitPattern(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_EXITPTRN);
}

/**
  * @brief  An exit pattern is not sent after header (MTYPE = header).
  * @rmtoll
  *  CFGR      EXITPTRN      LL_I3C_DisableExitPattern
  * @param  p_i3c I3C Instance.
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_DisableExitPattern(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_EXITPTRN);
}

/**
  * @brief  Check if exit pattern is enabled or disabled.
  * @rmtoll
  *  CFGR      EXITPTRN      LL_I3C_IsEnabledExitPattern
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledExitPattern(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_EXITPTRN) == (I3C_CFGR_EXITPTRN)) ? 1UL : 0UL);
}

/**
  * @brief  High keeper is enabled and will be used in place of standard open drain pull up device
  *         during handoff procedures.
  * @rmtoll
  *  CFGR      HKSDAEN       LL_I3C_EnableHighKeeperSDA
  * @param  p_i3c I3C Instance.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableHighKeeperSDA(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_HKSDAEN);
}

/**
  * @brief  High keeper is disabled.
  * @rmtoll
  *  CFGR      HKSDAEN       LL_I3C_DisableHighKeeperSDA
  * @param  p_i3c I3C Instance.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableHighKeeperSDA(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_HKSDAEN);
}

/**
  * @brief  Check if High keeper is enabled or disabled.
  * @rmtoll
  *  CFGR      HKSDAEN       LL_I3C_IsEnabledHighKeeperSDA
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledHighKeeperSDA(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_HKSDAEN) == (I3C_CFGR_HKSDAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Hot-Join request is acked. Current frame on the bus is continued.
  *         A Hot-Join interrupt is sent through HJF flag.
  * @rmtoll
  *  CFGR      HJACK         LL_I3C_EnableHJAck
  * @param  p_i3c I3C Instance.
  * @note   This bit can be used when I3C is acting as a controller.
  */
__STATIC_INLINE void LL_I3C_EnableHJAck(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_HJACK);
}

/**
  * @brief  Hot-Join request is NACKed. Current frame on the bus is continued.
  *         No Hot-Join interrupt is generated.
  * @rmtoll
  *  CFGR      HJACK         LL_I3C_DisableHJAck
  * @param  p_i3c I3C Instance.
  * @note   This bit can be used when I3C is acting as a controller.
  */
__STATIC_INLINE void LL_I3C_DisableHJAck(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_HJACK);
}

/**
  * @brief  Check if Hot-Join request acknowledgement is enabled or disabled.
  * @rmtoll
  *  CFGR      HJACK         LL_I3C_IsEnabledHJAck
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledHJAck(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_HJACK) == (I3C_CFGR_HJACK)) ? 1UL : 0UL);
}

/**
  * @brief  Get the data register address used for DMA transfer.
  * @rmtoll
  *  TDR          TDB0          LL_I3C_DMA_GetRegAddr \n
  *  TDWR         TDWR          LL_I3C_DMA_GetRegAddr \n
  *  RDR          RXRB0         LL_I3C_DMA_GetRegAddr \n
  *  RDWR         RDWR          LL_I3C_DMA_GetRegAddr \n
  *  SR           SR            LL_I3C_DMA_GetRegAddr \n
  *  CR           CR            LL_I3C_DMA_GetRegAddr
  * @param  p_i3c I3C Instance
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_I3C_DMA_REG_DATA_TRANSMIT_BYTE
  *         @arg @ref LL_I3C_DMA_REG_DATA_RECEIVE_BYTE
  *         @arg @ref LL_I3C_DMA_REG_DATA_TRANSMIT_WORD
  *         @arg @ref LL_I3C_DMA_REG_DATA_RECEIVE_WORD
  *         @arg @ref LL_I3C_DMA_REG_STATUS
  *         @arg @ref LL_I3C_DMA_REG_CONTROL
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t LL_I3C_DMA_GetRegAddr(const I3C_TypeDef *p_i3c, uint32_t direction)
{
  uint32_t data_reg_addr;

  if (direction == LL_I3C_DMA_REG_DATA_TRANSMIT_BYTE)
  {
    /* return address of TDR register */
    data_reg_addr = (uint32_t) &(p_i3c->TDR);
  }
  else if (direction == LL_I3C_DMA_REG_DATA_RECEIVE_BYTE)
  {
    /* return address of RDR register */
    data_reg_addr = (uint32_t) &(p_i3c->RDR);
  }
  else if (direction == LL_I3C_DMA_REG_DATA_TRANSMIT_WORD)
  {
    /* return address of TDWR register */
    data_reg_addr = (uint32_t) &(p_i3c->TDWR);
  }
  else if (direction == LL_I3C_DMA_REG_DATA_RECEIVE_WORD)
  {
    /* return address of RDWR register */
    data_reg_addr = (uint32_t) &(p_i3c->RDWR);
  }
  else if (direction == LL_I3C_DMA_REG_STATUS)
  {
    /* return address of SR register */
    data_reg_addr = (uint32_t) &(p_i3c->SR);
  }
  else
  {
    /* return address of CR register */
    data_reg_addr = (uint32_t) &(p_i3c->CR);
  }

  return data_reg_addr;
}

/**
  * @brief  Enable DMA FIFO reception requests.
  * @rmtoll
  *  CFGR      RXDMAEN       LL_I3C_EnableDMAReq_RX
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableDMAReq_RX(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_RXDMAEN);
}

/**
  * @brief  Disable DMA FIFO reception requests.
  * @rmtoll
  *  CFGR      RXDMAEN       LL_I3C_DisableDMAReq_RX
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableDMAReq_RX(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_RXDMAEN);
}

/**
  * @brief  Check if DMA FIFO reception requests are enabled or disabled.
  * @rmtoll
  *  CFGR      RXDMAEN       LL_I3C_IsEnabledDMAReq_RX
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledDMAReq_RX(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_RXDMAEN) == (I3C_CFGR_RXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the receive FIFO threshold level.
  * @rmtoll
  *  CFGR      RXTHRES       LL_I3C_SetRxFIFOThreshold
  * @param  p_i3c I3C Instance.
  * @param  rx_fifo_threshold This parameter can be one of the following values:
  *         @arg @ref LL_I3C_RXFIFO_THRESHOLD_1_8
  *         @arg @ref LL_I3C_RXFIFO_THRESHOLD_1_2
  */
__STATIC_INLINE void LL_I3C_SetRxFIFOThreshold(I3C_TypeDef *p_i3c, uint32_t rx_fifo_threshold)
{
  STM32_MODIFY_REG(p_i3c->CFGR, I3C_CFGR_RXTHRES, rx_fifo_threshold);
}

/**
  * @brief  Get the receive FIFO threshold level.
  * @rmtoll
  *  CFGR      RXTHRES       LL_I3C_GetRxFIFOThreshold
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_RXFIFO_THRESHOLD_1_8
  *         @arg @ref LL_I3C_RXFIFO_THRESHOLD_1_2
  */
__STATIC_INLINE uint32_t LL_I3C_GetRxFIFOThreshold(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_RXTHRES));
}

/**
  * @brief  Enable DMA FIFO transmission requests.
  * @rmtoll
  *  CFGR      TXDMAEN       LL_I3C_EnableDMAReq_TX
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableDMAReq_TX(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_TXDMAEN);
}

/**
  * @brief  Disable DMA FIFO transmission requests.
  * @rmtoll
  *  CFGR      TXDMAEN       LL_I3C_DisableDMAReq_TX
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableDMAReq_TX(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_TXDMAEN);
}

/**
  * @brief  Check if DMA FIFO transmission requests are enabled or disabled.
  * @rmtoll
  *  CFGR      TXDMAEN       LL_I3C_IsEnabledDMAReq_TX
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledDMAReq_TX(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_TXDMAEN) == (I3C_CFGR_TXDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the transmit FIFO threshold level.
  * @rmtoll
  *  CFGR      TXTHRES       LL_I3C_SetTxFIFOThreshold
  * @param  p_i3c I3C Instance.
  * @param  tx_fifo_threshold This parameter can be one of the following values:
  *         @arg @ref LL_I3C_TXFIFO_THRESHOLD_1_8
  *         @arg @ref LL_I3C_TXFIFO_THRESHOLD_1_2
  */
__STATIC_INLINE void LL_I3C_SetTxFIFOThreshold(I3C_TypeDef *p_i3c, uint32_t tx_fifo_threshold)
{
  STM32_MODIFY_REG(p_i3c->CFGR, I3C_CFGR_TXTHRES, tx_fifo_threshold);
}

/**
  * @brief  Get the transmit FIFO threshold level.
  * @rmtoll
  *  CFGR      TXTHRES       LL_I3C_GetTxFIFOThreshold
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_TXFIFO_THRESHOLD_1_8
  *         @arg @ref LL_I3C_TXFIFO_THRESHOLD_1_2
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_GetTxFIFOThreshold(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_TXTHRES));
}

/**
  * @brief  Get controller FIFO configuration.
  * @rmtoll
  *  CFGR     TMODE    LL_I3C_GetCtrlFifo \n
  *  CFGR     SMODE    LL_I3C_GetCtrlFifo
  * @param  p_i3c I3C Instance.
  * @retval One of:
  *         @ref LL_I3C_CTRL_FIFO_NONE
  *         @ref LL_I3C_CTRL_FIFO_CONTROL_ONLY
  *         @ref LL_I3C_CTRL_FIFO_STATUS_ONLY
  *         @ref LL_I3C_CTRL_FIFO_ALL
  */
__STATIC_INLINE uint32_t LL_I3C_GetCtrlFifo(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CFGR, (I3C_CFGR_TMODE | I3C_CFGR_SMODE)));
}

/**
  * @brief  Enable DMA FIFO status requests.
  * @rmtoll
  *  CFGR      SDMAEN        LL_I3C_EnableDMAReq_Status
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableDMAReq_Status(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_SDMAEN);
}

/**
  * @brief  Disable DMA FIFO status requests.
  * @rmtoll
  *  CFGR      SDMAEN        LL_I3C_DisableDMAReq_Status
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableDMAReq_Status(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_SDMAEN);
}

/**
  * @brief  Check if DMA FIFO status requests are enabled or disabled.
  * @rmtoll
  *  CFGR      SDMAEN        LL_I3C_IsEnabledDMAReq_Status
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledDMAReq_Status(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_SDMAEN) == (I3C_CFGR_SDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the controller fifo.
  * @rmtoll
  *  CFGR     TXTHRES    LL_I3C_ConfigCtrlFifo \n
  *  CFGR     RXTHRES    LL_I3C_ConfigCtrlFifo \n
  *  CFGR     TMODE      LL_I3C_ConfigCtrlFifo \n
  *  CFGR     SMODE      LL_I3C_ConfigCtrlFifo
  * @param  p_i3c I3C Instance.
  * @param  tx_fifo_threshold One of @ref LL_I3C_TXFIFO_THRESHOLD_1_8 or @ref LL_I3C_TXFIFO_THRESHOLD_1_2.
  * @param  rx_fifo_threshold One of @ref LL_I3C_RXFIFO_THRESHOLD_1_8 or @ref LL_I3C_RXFIFO_THRESHOLD_1_2.
  * @param  ctrl_fifo One of @ref LL_I3C_CTRL_FIFO_NONE, @ref LL_I3C_CTRL_FIFO_CONTROL_ONLY,
  *                   @ref LL_I3C_CTRL_FIFO_STATUS_ONLY, @ref LL_I3C_CTRL_FIFO_ALL.
  * @note   These bits can only be programmed when EN = 0.
  */
__STATIC_INLINE void LL_I3C_ConfigCtrlFifo(I3C_TypeDef *p_i3c,
                                           uint32_t tx_fifo_threshold,
                                           uint32_t rx_fifo_threshold,
                                           uint32_t ctrl_fifo)
{
  STM32_MODIFY_REG(p_i3c->CFGR, (I3C_CFGR_TXTHRES | I3C_CFGR_RXTHRES | I3C_CFGR_TMODE | I3C_CFGR_SMODE), \
                   tx_fifo_threshold | rx_fifo_threshold | ctrl_fifo);
}

/**
  * @brief  Configure the target fifo.
  * @rmtoll
  *  CFGR     TXTHRES    LL_I3C_ConfigTgtFifo \n
  *  CFGR     RXTHRES    LL_I3C_ConfigTgtFifo
  * @param  p_i3c I3C Instance.
  * @param  tx_fifo_threshold One of @ref LL_I3C_TXFIFO_THRESHOLD_1_8 or @ref LL_I3C_TXFIFO_THRESHOLD_1_2.
  * @param  rx_fifo_threshold One of @ref LL_I3C_RXFIFO_THRESHOLD_1_8 or @ref LL_I3C_RXFIFO_THRESHOLD_1_2.
  * @note   These bits can only be programmed when EN = 0.
  */
__STATIC_INLINE void LL_I3C_ConfigTgtFifo(I3C_TypeDef *p_i3c,
                                          uint32_t tx_fifo_threshold,
                                          uint32_t rx_fifo_threshold)
{
  STM32_MODIFY_REG(p_i3c->CFGR, (I3C_CFGR_TXTHRES | I3C_CFGR_RXTHRES), tx_fifo_threshold | rx_fifo_threshold);
}

/**
  * @brief  Enable the status FIFO.
  * @rmtoll
  *  CFGR      SMODE         LL_I3C_EnableStatusFIFO
  * @param  p_i3c I3C Instance.
  * @note   Not applicable in target mode. status FIFO always disabled in target mode.
  * @rmtoll
  *  CFGR      SMODE         LL_I3C_EnableStatusFIFO
  */
__STATIC_INLINE void LL_I3C_EnableStatusFIFO(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_SMODE);
}

/**
  * @brief  Disable the status FIFO threshold.
  * @rmtoll
  *  CFGR      SMODE         LL_I3C_DisableStatusFIFO
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableStatusFIFO(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_SMODE);
}

/**
  * @brief  Check if the status FIFO threshold is enabled or disabled.
  * @rmtoll
  *  CFGR      SMODE         LL_I3C_IsEnabledStatusFIFO
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStatusFIFO(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_SMODE) == (I3C_CFGR_SMODE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the control and transmit FIFO preloaded before starting a transfer on I3C bus.
  * @param  p_i3c I3C Instance.
  * @note   Not applicable in target mode. control FIFO always disabled in target mode.
  * @rmtoll
  *  CFGR      TMODE         LL_I3C_EnableControlFIFO
  */
__STATIC_INLINE void LL_I3C_EnableControlFIFO(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_TMODE);
}

/**
  * @brief  Disable the control and transmit FIFO preloaded before starting a transfer on I3C bus.
  * @rmtoll
  *  CFGR      TMODE         LL_I3C_DisableControlFIFO
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableControlFIFO(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_TMODE);
}

/**
  * @brief  Check if the control and transmit FIFO preloaded is enabled or disabled.
  * @rmtoll
  *  CFGR      TMODE         LL_I3C_IsEnabledControlFIFO
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledControlFIFO(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_TMODE) == (I3C_CFGR_TMODE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable DMA FIFO control word transfer requests.
  * @rmtoll
  *  CFGR      CDMAEN        LL_I3C_EnableDMAReq_Control
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableDMAReq_Control(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_CDMAEN);
}

/**
  * @brief  Disable DMA FIFO control word transfer requests.
  * @rmtoll
  *  CFGR      CDMAEN        LL_I3C_DisableDMAReq_Control
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableDMAReq_Control(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, I3C_CFGR_CDMAEN);
}

/**
  * @brief  Disable all I3C DMA request sources (Control, RX, TX, Status) with a single register access.
  * @rmtoll
  *  CFGR      CDMAEN     LL_I3C_DisableAllDMARequests \n
  *  CFGR      RXDMAEN    LL_I3C_DisableAllDMARequests \n
  *  CFGR      TXDMAEN    LL_I3C_DisableAllDMARequests \n
  *  CFGR      SDMAEN     LL_I3C_DisableAllDMARequests
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableAllDMARequests(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CFGR, (I3C_CFGR_CDMAEN | I3C_CFGR_RXDMAEN | I3C_CFGR_TXDMAEN | I3C_CFGR_SDMAEN));
}

/**
  * @brief  Check if DMA FIFO control word transfer requests are enabled or disabled.
  * @rmtoll
  *  CFGR      CDMAEN        LL_I3C_IsEnabledDMAReq_Control
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledDMAReq_Control(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_CDMAEN) == (I3C_CFGR_CDMAEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set Own dynamic address as Valid.
  * @rmtoll
  *  DEVR0        DAVAL         LL_I3C_EnableOwnDynAddress
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableOwnDynAddress(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->DEVR0, I3C_DEVR0_DAVAL);
}

/**
  * @brief  Configure Own dynamic address and set it valid in one operation.
  * @rmtoll
  *  DEVR0        DA,DAVAL     LL_I3C_SetAndEnableOwnDynamicAddress
  * @param  p_i3c I3C Instance.
  * @param  own_dynamic_address Value between 0 and 0x7F.
  */
__STATIC_INLINE void LL_I3C_SetAndEnableOwnDynamicAddress(I3C_TypeDef *p_i3c, uint32_t own_dynamic_address)
{
  /* Program dynamic address and set address valid */
  STM32_MODIFY_REG(p_i3c->DEVR0, I3C_DEVR0_DA | I3C_DEVR0_DAVAL, \
                   (own_dynamic_address << I3C_DEVR0_DA_Pos) | I3C_DEVR0_DAVAL);
}

/**
  * @brief  Set Own dynamic address as not-Valid.
  * @rmtoll
  *  DEVR0        DAVAL         LL_I3C_DisableOwnDynAddress
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableOwnDynAddress(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->DEVR0, I3C_DEVR0_DAVAL);
}

/**
  * @brief  Check if own dynamic address is valid or not valid.
  * @rmtoll
  *  DEVR0        DAVAL         LL_I3C_IsEnabledOwnDynAddress
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledOwnDynAddress(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_DAVAL) == (I3C_DEVR0_DAVAL)) ? 1UL : 0UL);
}

/**
  * @brief  Configure Own dynamic address.
  * @rmtoll
  *  DEVR0        DA            LL_I3C_SetOwnDynamicAddress
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed in controller mode or during dynamic address procedure from current controller.
  * @param  own_dynamic_address This parameter must be a value between Min_Data=0 and Max_Data=0x7F
  */
__STATIC_INLINE void LL_I3C_SetOwnDynamicAddress(I3C_TypeDef *p_i3c, uint32_t own_dynamic_address)
{
  STM32_MODIFY_REG(p_i3c->DEVR0, I3C_DEVR0_DA, (own_dynamic_address << I3C_DEVR0_DA_Pos));
}

/**
  * @brief  Get Own dynamic address.
  * @rmtoll
  *  DEVR0        DA            LL_I3C_GetOwnDynamicAddress
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0x7F
  */
__STATIC_INLINE uint8_t LL_I3C_GetOwnDynamicAddress(const I3C_TypeDef *p_i3c)
{
  return (uint8_t)(STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_DA) >> I3C_DEVR0_DA_Pos);
}

/**
  * @brief  Set IBI procedure allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        IBIEN         LL_I3C_EnableIBI
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_EnableIBI(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->DEVR0, I3C_DEVR0_IBIEN);
}

/**
  * @brief  Set IBI procedure not-allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        IBIEN         LL_I3C_DisableIBI
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_DisableIBI(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->DEVR0, I3C_DEVR0_IBIEN);
}

/**
  * @brief  Check if IBI procedure is allowed or not allowed.
  * @rmtoll
  *  DEVR0        IBIEN         LL_I3C_IsEnabledIBI
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIBI(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_IBIEN) == (I3C_DEVR0_IBIEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set controller-role request allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        CREN          LL_I3C_EnableControllerRoleReq
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_EnableControllerRoleReq(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->DEVR0, I3C_DEVR0_CREN);
}

/**
  * @brief  Set controller-role request as not-allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        CREN          LL_I3C_DisableControllerRoleReq
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_DisableControllerRoleReq(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->DEVR0, I3C_DEVR0_CREN);
}

/**
  * @brief  Check if controller-role request is allowed or not-allowed.
  * @rmtoll
  *  DEVR0        CREN          LL_I3C_IsEnabledControllerRoleReq
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledControllerRoleReq(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_CREN) == (I3C_DEVR0_CREN)) ? 1UL : 0UL);
}

/**
  * @brief  Set Hot-Join allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        HJEN          LL_I3C_EnableHotJoin
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_EnableHotJoin(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->DEVR0, I3C_DEVR0_HJEN);
}

/**
  * @brief  Set Hot-Join as not-allowed (when the I3C is acting as target).
  * @rmtoll
  *  DEVR0        HJEN          LL_I3C_DisableHotJoin
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0) or updated by HW upon reception of DISEC CCC.
  */
__STATIC_INLINE void LL_I3C_DisableHotJoin(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->DEVR0, I3C_DEVR0_HJEN);
}

/**
  * @brief  Check if Hot-Join is allowed or not-allowed.
  * @rmtoll
  *  DEVR0        HJEN          LL_I3C_IsEnabledHotJoin
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledHotJoin(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_HJEN) == (I3C_DEVR0_HJEN)) ? 1UL : 0UL);
}

/**
  * @brief  Configure maximum read length (target mode).
  * @rmtoll
  *  MAXRLR       MRL           LL_I3C_SetMaxReadLength
  * @param  p_i3c I3C Instance.
  * @note   Those bits can be updated by HW upon reception of GETMRL CCC.
  * @param  max_read_length This parameter must be a value between Min_Data=0x0 and Max_Data=0xFFFF
  */
__STATIC_INLINE void LL_I3C_SetMaxReadLength(I3C_TypeDef *p_i3c, uint32_t max_read_length)
{
  STM32_MODIFY_REG(p_i3c->MAXRLR, I3C_MAXRLR_MRL, max_read_length);
}

/**
  * @brief  Return maximum read length (target mode).
  * @rmtoll
  *  MAXRLR       MRL           LL_I3C_GetMaxReadLength
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xFFFFF
  */
__STATIC_INLINE uint32_t LL_I3C_GetMaxReadLength(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->MAXRLR, I3C_MAXRLR_MRL));
}

/**
  * @brief  Configure the number of additional Mandatory data Byte (MDB) sent to the controller
  *         after an acknowledge of the IBI (target mode).
  * @rmtoll
  *  MAXRLR       IBIP          LL_I3C_ConfigNbIBIAddData
  * @param  p_i3c I3C Instance.
  * @param  nb_ibi_add_data This parameter can be one of the following values:
  *         @arg @ref LL_I3C_PAYLOAD_EMPTY << I3C_MAXRLR_IBIP_Pos
  *         @arg @ref LL_I3C_PAYLOAD_1_BYTE << I3C_MAXRLR_IBIP_Pos
  *         @arg @ref LL_I3C_PAYLOAD_2_BYTE << I3C_MAXRLR_IBIP_Pos
  *         @arg @ref LL_I3C_PAYLOAD_3_BYTE << I3C_MAXRLR_IBIP_Pos
  *         @arg @ref LL_I3C_PAYLOAD_4_BYTE << I3C_MAXRLR_IBIP_Pos
  */
__STATIC_INLINE void LL_I3C_ConfigNbIBIAddData(I3C_TypeDef *p_i3c, uint32_t nb_ibi_add_data)
{
  STM32_MODIFY_REG(p_i3c->MAXRLR, I3C_MAXRLR_IBIP, nb_ibi_add_data);
}

/**
  * @brief  Set the number of additional Mandatory data Byte (MDB) sent to the controller
  *         after an acknowledge of the IBI (target mode).
  *         I3C_MAXRLR_IBIP must be at reset value. It means after a block reset of the I3C Instance
  * @rmtoll
  *  MAXRLR       IBIP          LL_I3C_SetNbIBIAddData
  * @param  p_i3c I3C Instance.
  * @param  nb_ibi_add_data This parameter can be one of the following values:
  *         @arg @ref LL_I3C_PAYLOAD_EMPTY
  *         @arg @ref LL_I3C_PAYLOAD_1_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_2_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_3_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_4_BYTE
  */
__STATIC_INLINE void LL_I3C_SetNbIBIAddData(I3C_TypeDef *p_i3c, uint32_t nb_ibi_add_data)
{
  STM32_SET_BIT(p_i3c->MAXRLR, nb_ibi_add_data);
}

/**
  * @brief  Return the number of additional Mandatory data Byte (MDB) sent to the controller
  *         after an acknowledge of the IBI (target mode).
  * @rmtoll
  *  MAXRLR       IBIP          LL_I3C_GetConfigNbIBIAddData
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_PAYLOAD_EMPTY
  *         @arg @ref LL_I3C_PAYLOAD_1_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_2_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_3_BYTE
  *         @arg @ref LL_I3C_PAYLOAD_4_BYTE
  */
__STATIC_INLINE uint32_t LL_I3C_GetConfigNbIBIAddData(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->MAXRLR, I3C_MAXRLR_IBIP));
}

/**
  * @brief  Configure maximum write length (target mode).
  * @rmtoll
  *  MAXWLR       MWL           LL_I3C_SetMaxWriteLength
  * @param  p_i3c I3C Instance.
  * @param  max_write_length_byte This parameter must be a value between Min_Data=0x0 and Max_Data=0xFFFF
  * @note   Those bits can be updated by HW upon reception of GETMWL CCC.
  */
__STATIC_INLINE void LL_I3C_SetMaxWriteLength(I3C_TypeDef *p_i3c, uint32_t max_write_length_byte)
{
  STM32_MODIFY_REG(p_i3c->MAXWLR, I3C_MAXWLR_MWL, max_write_length_byte);
}

/**
  * @brief  Return maximum write length (target mode).
  * @rmtoll
  *  MAXWLR       MWL           LL_I3C_GetMaxWriteLength
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0xFFFFF
  */
__STATIC_INLINE uint32_t LL_I3C_GetMaxWriteLength(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->MAXWLR, I3C_MAXWLR_MWL));
}

/**
  * @brief  Configure the SCL clock signal waveform.
  * @rmtoll
  *  TIMINGR0     TIMINGR0      LL_I3C_ConfigClockWaveForm
  * @param  p_i3c I3C Instance.
  * @param  clock_wave_form This parameter must be a value between Min_Data=0 and Max_Data=0xFFFFFFFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_ConfigClockWaveForm(I3C_TypeDef *p_i3c, uint32_t clock_wave_form)
{
  STM32_WRITE_REG(p_i3c->TIMINGR0, clock_wave_form);
}

/**
  * @brief  Get the SCL clock signal waveform.
  * @rmtoll
  *  TIMINGR0     TIMINGR0      LL_I3C_GetClockWaveForm
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFFFFFFFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetClockWaveForm(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->TIMINGR0));
}

/**
  * @brief  Configure the SCL clock low period during I3C push-pull phases.
  * @rmtoll
  *  TIMINGR0     SCLL_PP       LL_I3C_SetPeriodClockLowPP
  * @param  p_i3c I3C Instance.
  * @param  period_clock_low_pp This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetPeriodClockLowPP(I3C_TypeDef *p_i3c, uint32_t period_clock_low_pp)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLL_PP, (period_clock_low_pp << I3C_TIMINGR0_SCLL_PP_Pos));
}

/**
  * @brief  Get the SCL clock low period during I3C push-pull phases.
  * @rmtoll
  *  TIMINGR0     SCLL_PP       LL_I3C_GetPeriodClockLowPP
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetPeriodClockLowPP(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLL_PP) >> I3C_TIMINGR0_SCLL_PP_Pos);
}

/**
  * @brief  Configure the SCL clock High period during I3C open drain and push-pull phases.
  * @rmtoll
  *  TIMINGR0     SCLH_I3C      LL_I3C_SetPeriodClockHighI3C
  * @param  p_i3c I3C Instance.
  * @param  period_clock_high_i3c This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetPeriodClockHighI3C(I3C_TypeDef *p_i3c, uint32_t period_clock_high_i3c)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLH_I3C, (period_clock_high_i3c << I3C_TIMINGR0_SCLH_I3C_Pos));
}

/**
  * @brief  Get the SCL clock high period during I3C open drain and push-pull phases.
  * @rmtoll
  *  TIMINGR0     SCLH_I3C      LL_I3C_GetPeriodClockHighI3C
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetPeriodClockHighI3C(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLH_I3C) >> I3C_TIMINGR0_SCLH_I3C_Pos);
}

/**
  * @brief  Configure the SCL clock low period during I3C open drain phases.
  * @rmtoll
  *  TIMINGR0     SCLL_OD       LL_I3C_SetPeriodClockLowOD
  * @param  p_i3c I3C Instance.
  * @param  period_clock_low_od This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetPeriodClockLowOD(I3C_TypeDef *p_i3c, uint32_t period_clock_low_od)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLL_OD, (period_clock_low_od << I3C_TIMINGR0_SCLL_OD_Pos));
}

/**
  * @brief  Get the SCL clock low period during I3C open phases.
  * @rmtoll
  *  TIMINGR0     SCLL_OD       LL_I3C_GetPeriodClockLowOD
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetPeriodClockLowOD(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLL_OD) >> I3C_TIMINGR0_SCLL_OD_Pos);
}

/**
  * @brief  Configure the SCL clock High period during I2C open drain phases.
  * @rmtoll
  *  TIMINGR0     SCLH_I2C      LL_I3C_SetPeriodClockHighI2C
  * @param  p_i3c I3C Instance.
  * @param  period_clock_high_i2c This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetPeriodClockHighI2C(I3C_TypeDef *p_i3c, uint32_t period_clock_high_i2c)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLH_I2C, period_clock_high_i2c << I3C_TIMINGR0_SCLH_I2C_Pos);
}

/**
  * @brief  Get the SCL clock high period during I2C open drain phases.
  * @rmtoll
  *  TIMINGR0     SCLH_I2C      LL_I3C_GetPeriodClockHighI2C
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetPeriodClockHighI2C(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR0, I3C_TIMINGR0_SCLH_I2C) >> I3C_TIMINGR0_SCLH_I2C_Pos);
}

/**
  * @brief  Configure the controller additional hold time on SDA line.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_SetDataHoldTime
  * @param  p_i3c I3C Instance.
  * @param  data_hold_time This parameter can be one of the following values:
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_0_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_1_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_2_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_3_5
  */
__STATIC_INLINE void LL_I3C_SetDataHoldTime(I3C_TypeDef *p_i3c, uint32_t data_hold_time)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR1, I3C_TIMINGR1_SDA_HD, data_hold_time);
}

/**
  * @brief  Get the controller additional hold time on SDA line.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_GetDataHoldTime
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_0_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_1_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_2_5
  *         @arg @ref LL_I3C_SDA_HOLD_TIME_3_5
  */
__STATIC_INLINE uint32_t LL_I3C_GetDataHoldTime(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR1, I3C_TIMINGR1_SDA_HD));
}

/**
  * @brief  Configure the Idle, Available state.
  * @rmtoll
  *  TIMINGR1     AVAL          LL_I3C_SetAvalTiming
  * @param  p_i3c I3C Instance.
  * @param  aval_timing This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetAvalTiming(I3C_TypeDef *p_i3c, uint32_t aval_timing)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR1, I3C_TIMINGR1_AVAL, (aval_timing << I3C_TIMINGR1_AVAL_Pos));
}

/**
  * @brief  Get the Idle, Available integer value state.
  * @rmtoll
  *  TIMINGR1     AVAL          LL_I3C_GetAvalTiming
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetAvalTiming(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR1, I3C_TIMINGR1_AVAL) >> I3C_TIMINGR1_AVAL_Pos);
}

/**
  * @brief  Configure the Free state.
  * @rmtoll
  *  TIMINGR1     FREE          LL_I3C_SetFreeTiming
  * @param  p_i3c I3C Instance.
  * @param  free_timing This parameter must be a value between Min_Data=0 and Max_Data=0x3F.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetFreeTiming(I3C_TypeDef *p_i3c, uint32_t free_timing)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR1, I3C_TIMINGR1_FREE, (free_timing << I3C_TIMINGR1_FREE_Pos));
}

/**
  * @brief  Get the free integer value state.
  * @rmtoll
  *  TIMINGR1     FREE          LL_I3C_GetFreeTiming
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0x3F.
  */
__STATIC_INLINE uint32_t LL_I3C_GetFreeTiming(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR1, I3C_TIMINGR1_FREE) >> I3C_TIMINGR1_FREE_Pos);
}

/**
  * @brief  Configure the activity state of the new controller.
  * @rmtoll
  *  TIMINGR1     ASNCR            LL_I3C_SetControllerActivityState
  * @param  p_i3c I3C Instance.
  * @param  controller_activity_state This parameter can be one of the following values:
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_0
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_1
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_2
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_3
  * @note   Refer to MIPI I3C specification (https:__www.mipi.org_specifications)
  *         for more details related to activity state.
  */
__STATIC_INLINE void LL_I3C_SetControllerActivityState(I3C_TypeDef *p_i3c, uint32_t controller_activity_state)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR1, I3C_TIMINGR1_ASNCR, controller_activity_state);
}

/**
  * @brief  Get the activity state of the new controller.
  * @rmtoll
  *  TIMINGR1     ASNCR            LL_I3C_GetControllerActivityState
  * @param  p_i3c I3C Instance.
  * @note   Refer to MIPI I3C specification (https:__www.mipi.org_specifications)
  *         for more details related to activity state.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_0
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_1
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_2
  *         @arg @ref LL_I3C_OWN_ACTIVITY_STATE_3
  */
__STATIC_INLINE uint32_t LL_I3C_GetControllerActivityState(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR1, I3C_TIMINGR1_ASNCR));
}


/**
  * @brief  Configure the controller SDA hold time, Bus Free, Activity state, Idle state.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_SetBusCharacteristic \n
  *  TIMINGR1     FREE          LL_I3C_SetBusCharacteristic \n
  *  TIMINGR1     ASNCR         LL_I3C_SetBusCharacteristic \n
  *  TIMINGR1     AVAL          LL_I3C_SetBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @param  ctrl_bus_characteristic This parameter must be a value between Min_Data=0 and Max_Data=0x107F03FF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetBusCharacteristic(I3C_TypeDef *p_i3c, uint32_t ctrl_bus_characteristic)
{
  STM32_WRITE_REG(p_i3c->TIMINGR1, ctrl_bus_characteristic);
}

/**
  * @brief  Get the controller SDA hold time, Bus Free, Activity state, Idle state.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_GetBusCharacteristic \n
  *  TIMINGR1     FREE          LL_I3C_GetBusCharacteristic \n
  *  TIMINGR1     ASNCR         LL_I3C_GetBusCharacteristic \n
  *  TIMINGR1     AVAL          LL_I3C_GetBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0x107F03FF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetBusCharacteristic(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->TIMINGR1));
}


/**
  * @brief  Configure the controller SDA hold time, Bus Free, Activity state, Idle state.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_SetCtrlBusCharacteristic \n
  *  TIMINGR1     FREE          LL_I3C_SetCtrlBusCharacteristic \n
  *  TIMINGR1     ASNCR         LL_I3C_SetCtrlBusCharacteristic \n
  *  TIMINGR1     IDLE          LL_I3C_SetCtrlBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @param  ctrl_bus_characteristic This parameter must be a value between Min_Data=0 and Max_Data=0x107F03FF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetCtrlBusCharacteristic(I3C_TypeDef *p_i3c, uint32_t ctrl_bus_characteristic)
{
  STM32_WRITE_REG(p_i3c->TIMINGR1, ctrl_bus_characteristic);
}

/**
  * @brief  Get the controller SDA hold time, Bus Free, Activity state, Idle state.
  * @rmtoll
  *  TIMINGR1     SDA_HD        LL_I3C_GetCtrlBusCharacteristic \n
  *  TIMINGR1     FREE          LL_I3C_GetCtrlBusCharacteristic \n
  *  TIMINGR1     ASNCR         LL_I3C_GetCtrlBusCharacteristic \n
  *  TIMINGR1     IDLE          LL_I3C_GetCtrlBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0x107F03FF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetCtrlBusCharacteristic(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->TIMINGR1));
}

/**
  * @brief  Configure the target available state.
  * @rmtoll
  *  TIMINGR1     IDLE          LL_I3C_SetTgtBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @param  tgt_bus_characteristic This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetTgtBusCharacteristic(I3C_TypeDef *p_i3c, uint32_t tgt_bus_characteristic)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR1, I3C_TIMINGR1_AVAL, (tgt_bus_characteristic & I3C_TIMINGR1_AVAL));
}

/**
  * @brief  Get the target available state.
  * @rmtoll
  *  TIMINGR1     IDLE          LL_I3C_GetTgtBusCharacteristic
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetTgtBusCharacteristic(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR1, I3C_TIMINGR1_AVAL));
}

/**
  * @brief  Set the SCL clock stalling configuration. All stall features not selected are disabled.
  * @param  hi3c  Pointer to a hal_i3c_handle_t
  * @param  stall_time_cycle Controller clock stall time in number of kernel clock cycles.
            This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @param  stall_features Features of the I3C controller to which the stall time will be applied.
            See @ref I3C_CTRL_STALL_FEATURE_DEFINITION, this parameter is a combination of the following values:
              @ref LL_I3C_CTRL_STALL_ACK
              @ref LL_I3C_CTRL_STALL_CCC
              @ref LL_I3C_CTRL_STALL_TX
              @ref LL_I3C_CTRL_STALL_RX
              @ref LL_I3C_CTRL_STALL_I2C_ACK
              @ref LL_I3C_CTRL_STALL_I2C_TX
              @ref LL_I3C_CTRL_STALL_I2C_RX
              @ref LL_I3C_CTRL_STALL_ALL
              @ref LL_I3C_CTRL_STALL_NONE
  * @note   This configuration can be set when the I3C is acting as controller.
  */
__STATIC_INLINE void LL_I3C_ConfigStallTime(I3C_TypeDef *p_i3c, uint32_t stall_time_cycle, uint32_t stall_features)
{
  STM32_WRITE_REG(p_i3c->TIMINGR2, (((uint32_t) stall_time_cycle << I3C_TIMINGR2_STALL_Pos) | stall_features));
}

/**
  * @brief  Configure the SCL clock stalling time on I3C Bus (controller mode).
  * @rmtoll
  *  TIMINGR2     STALL        LL_I3C_SetStallTime
  * @param  p_i3c I3C Instance.
  * @param  ctrl_stall_time This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   This parameter is computed with the STM32CubeMX Tool.
  */
__STATIC_INLINE void LL_I3C_SetStallTime(I3C_TypeDef *p_i3c, uint32_t ctrl_stall_time)
{
  STM32_MODIFY_REG(p_i3c->TIMINGR2, I3C_TIMINGR2_STALL, (ctrl_stall_time << I3C_TIMINGR2_STALL_Pos));
}

/**
  * @brief  Get the SCL clock stalling time on I3C Bus (controller mode).
  * @rmtoll
  *  TIMINGR2     STALL        LL_I3C_GetStallTime
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetStallTime(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALL));
}

/**
  * @brief  Set stall on data ACK/NACK bit of legacy I2C address (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLL       LL_I3C_EnableStallACKI2CAddr
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallACKI2CAddr(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLL);
}

/**
  * @brief  Disable stall on data ACK/NACK bit of legacy I2C address (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLA       LL_I3C_DisableStallACKI2CAddr
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallACKI2CAddr(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLL);
}

/**
  * @brief  Check if stall on data ACK/NACK bit of legacy I2C address is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLA       LL_I3C_IsEnabledStallACKI2CAddr
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallACKI2CAddr(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLL) == (I3C_TIMINGR2_STALLL)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on data ACK/NACK bit of legacy I2C write message (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLS       LL_I3C_EnableStallACKI2CWrite
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallACKI2CWrite(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLS);
}

/**
  * @brief  Disable stall on data ACK/NACK bit of legacy I2C write message (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLS       LL_I3C_DisableStallACKI2CWrite
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallACKI2CWrite(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLS);
}

/**
  * @brief  Check if stall on data ACK/NACK bit of legacy I2C write message is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLS       LL_I3C_IsEnabledStallACKI2CWrite
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallACKI2CWrite(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLS) == (I3C_TIMINGR2_STALLS)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on data ACK/NACK bit of legacy I2C read message (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLR       LL_I3C_EnableStallACKI2CRead
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallACKI2CRead(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLR);
}

/**
  * @brief  Disable stall on data ACK/NACK bit of legacy I2C read message (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLR       LL_I3C_DisableStallACKI2CRead
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallACKI2CRead(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLR);
}

/**
  * @brief  Check if stall on data ACK/NACK bit of legacy I2C read message is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLR       LL_I3C_IsEnabledStallACKI2CRead
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallACKI2CRead(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLR) == (I3C_TIMINGR2_STALLR)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on ACK bit (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLA       LL_I3C_EnableStallACK
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallACK(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLA);
}

/**
  * @brief  Disable stall on ACK bit (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLA       LL_I3C_DisableStallACK
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallACK(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLA);
}

/**
  * @brief  Check if stall on ACK bit is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLA       LL_I3C_IsEnabledStallACK
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallACK(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLA) == (I3C_TIMINGR2_STALLA)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on Parity bit of command Code byte (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLC       LL_I3C_EnableStallParityCCC
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallParityCCC(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLC);
}

/**
  * @brief  Disable stall on Parity bit of command Code byte (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLC       LL_I3C_DisableStallParityCCC
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallParityCCC(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLC);
}

/**
  * @brief  Check if stall on Parity bit of command Code byte is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLC       LL_I3C_IsEnabledStallParityCCC
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallParityCCC(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLC) == (I3C_TIMINGR2_STALLC)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on Parity bit of data bytes (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLD       LL_I3C_EnableStallParityData
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallParityData(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLD);
}

/**
  * @brief  Disable stall on Parity bit of data bytes (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLD       LL_I3C_DisableStallParityData
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallParityData(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLD);
}

/**
  * @brief  Check if stall on Parity bit of data bytes is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLD       LL_I3C_IsEnabledStallParityData
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallParityData(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLD) == (I3C_TIMINGR2_STALLD)) ? 1UL : 0UL);
}

/**
  * @brief  Set stall on T bit (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLT       LL_I3C_EnableStallTbit
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableStallTbit(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLT);
}

/**
  * @brief  Disable stall on T bit (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLT       LL_I3C_DisableStallTbit
  * @param  p_i3c I3C Instance.
  * @note   This bit can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_DisableStallTbit(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLT);
}

/**
  * @brief  Check if stall on T bit is enabled or disabled (controller mode).
  * @rmtoll
  *  TIMINGR2     STALLT       LL_I3C_IsEnabledStallTbit
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledStallTbit(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TIMINGR2, I3C_TIMINGR2_STALLT) == (I3C_TIMINGR2_STALLT)) ? 1UL : 0UL);
}

/**
  * @brief   Set control capability, IBI payload support and max speed limitation in BCR register.
  * @rmtoll
  *  BCR     BCR      LL_I3C_ConfigPayloadEntDAA
  * @param  p_i3c I3C Instance.
  * @param  max_data_speed_limitation This parameter can be one of the following values:
  *         @arg @ref LL_I3C_NO_DATA_SPEED_LIMITATION
  *         @arg @ref LL_I3C_MAX_DATA_SPEED_LIMITATION
  * @param  ibi_payload This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_NO_ADDITIONAL_DATA
  *         @arg @ref LL_I3C_IBI_ADDITIONAL_DATA
  * @param  ctrl_role This parameter can be one of the following values:
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_TARGET
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_CONTROLLER
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_ConfigPayloadEntDAA(I3C_TypeDef *p_i3c, uint32_t  max_data_speed_limitation,
                                                uint32_t ibi_payload, uint32_t  ctrl_role)
{
  STM32_WRITE_REG(p_i3c->BCR, max_data_speed_limitation | ibi_payload  | ctrl_role);
}

/**
  * @brief  Configure the device capability on bus as target or controller (MIPI Bus Characteristics Register BCR6).
  * @rmtoll
  *  BCR          BCR6          LL_I3C_SetDeviceCapabilityOnBus
  * @param  p_i3c I3C Instance.
  * @param  device_capability_on_bus This parameter can be one of the following values:
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_TARGET
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_CONTROLLER
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetDeviceCapabilityOnBus(I3C_TypeDef *p_i3c, uint32_t device_capability_on_bus)
{
  STM32_MODIFY_REG(p_i3c->BCR, I3C_BCR_BCR6, device_capability_on_bus);
}

/**
  * @brief  Get the device capability on bus as target or controller (MIPI Bus Characteristics Register BCR6).
  * @rmtoll
  *  BCR          BCR6          LL_I3C_GetDeviceCapabilityOnBus
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_TARGET
  *         @arg @ref LL_I3C_DEVICE_ROLE_AS_CONTROLLER
  */
__STATIC_INLINE uint32_t LL_I3C_GetDeviceCapabilityOnBus(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->BCR, I3C_BCR_BCR6));
}

/**
  * @brief  Configure the device IBI payload (MIPI Bus Characteristics Register BCR2).
  * @rmtoll
  *  BCR          BCR2          LL_I3C_SetDeviceIBIPayload
  * @param  p_i3c I3C Instance.
  * @param  device_ibi_payload This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_NO_ADDITIONAL_DATA
  *         @arg @ref LL_I3C_IBI_ADDITIONAL_DATA
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetDeviceIBIPayload(I3C_TypeDef *p_i3c, uint32_t device_ibi_payload)
{
  STM32_MODIFY_REG(p_i3c->BCR, I3C_BCR_BCR2, device_ibi_payload);
}

/**
  * @brief  Get the device IBI payload (MIPI Bus Characteristics Register BCR2).
  * @rmtoll
  *  BCR          BCR2          LL_I3C_GetDeviceIBIPayload
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_IBI_NO_ADDITIONAL_DATA
  *         @arg @ref LL_I3C_IBI_ADDITIONAL_DATA
  */
__STATIC_INLINE uint32_t LL_I3C_GetDeviceIBIPayload(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->BCR, I3C_BCR_BCR2));
}

/**
  * @brief  Configure the data Speed Limitation (limitation, as described by I3C_GETMXDSR).
  * @rmtoll
  *  BCR          BCR0          LL_I3C_SetDataSpeedLimitation
  * @param  p_i3c I3C Instance.
  * @param  data_speed_limitation This parameter can be one of the following values:
  *         @arg @ref LL_I3C_NO_DATA_SPEED_LIMITATION
  *         @arg @ref LL_I3C_MAX_DATA_SPEED_LIMITATION
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetDataSpeedLimitation(I3C_TypeDef *p_i3c, uint32_t data_speed_limitation)
{
  STM32_MODIFY_REG(p_i3c->BCR, I3C_BCR_BCR0, data_speed_limitation);
}

/**
  * @brief  Get  the data Speed Limitation (limitation, as described by I3C_GETMXDSR).
  * @rmtoll
  *  BCR          BCR0          LL_I3C_GetDataSpeedLimitation
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_NO_DATA_SPEED_LIMITATION
  *         @arg @ref LL_I3C_MAX_DATA_SPEED_LIMITATION
  */
__STATIC_INLINE uint32_t LL_I3C_GetDataSpeedLimitation(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->BCR, I3C_BCR_BCR0));
}

/**
  * @brief  Configure the device Characteristics Register (DCR).
  * @rmtoll
  *  DCR          DC            LL_I3C_SetDeviceCharacteristics
  * @param  p_i3c I3C Instance.
  * @param  device_characteristics This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @note   This bit can only be programmed when the I3C is disabled (EN = 0).
  *
  * @note   Refer MIPI web site for the list of device code available.
  */
__STATIC_INLINE void LL_I3C_SetDeviceCharacteristics(I3C_TypeDef *p_i3c, uint32_t device_characteristics)
{
  STM32_MODIFY_REG(p_i3c->DCR, I3C_DCR_DCR, device_characteristics);
}

/**
  * @brief  Get the device Characteristics Register (DCR).
  * @rmtoll
  *  DCR          DCR            LL_I3C_GetDeviceCharacteristics
  * @param  p_i3c I3C Instance.
  * @note   Refer MIPI web site to associated value with the list of device code available.
  * @retval Value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetDeviceCharacteristics(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->DCR, I3C_DCR_DCR));
}

/**
  * @brief  Configure IBI MDB support for pending read notification.
  * @rmtoll
  *  GETCAPR     CAPPEND          LL_I3C_SetPendingReadMDB
  * @param  p_i3c I3C Instance.
  * @param  pending_read_mdb This parameter can be one of the following values:
  *         @arg @ref LL_I3C_MDB_NO_PENDING_READ_NOTIFICATION
  *         @arg @ref LL_I3C_MDB_PENDING_READ_NOTIFICATION
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetPendingReadMDB(I3C_TypeDef *p_i3c, uint32_t pending_read_mdb)
{
  STM32_MODIFY_REG(p_i3c->GETCAPR, I3C_GETCAPR_CAPPEND, pending_read_mdb);
}

/**
  * @brief  Enable IBI MDB support for pending read notification.
  * @rmtoll
  *  GETCAPR     CAPPEND          LL_I3C_EnablePendingReadMDB
  * @param  p_i3c I3C Instance.
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnablePendingReadMDB(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->GETCAPR, I3C_GETCAPR_CAPPEND);
}


/**
  * @brief  Get IBI MDB support for pending read notification value.
  * @rmtoll
  *  GETCAPR     CAPPEND          LL_I3C_GetPendingReadMDB
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_MDB_NO_PENDING_READ_NOTIFICATION
  *         @arg @ref LL_I3C_MDB_PENDING_READ_NOTIFICATION
  */
__STATIC_INLINE uint32_t LL_I3C_GetPendingReadMDB(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->GETCAPR, I3C_GETCAPR_CAPPEND));
}

/**
  * @brief  Configure the Group Management Support bit of MSTCAP1.
  * @rmtoll
  *  CRCAPR      CAPGRP        LL_I3C_SetGrpAddrHandoffSupport
  * @param  p_i3c I3C Instance.
  * @param  grp_addr_handoff_support This parameter can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED
  *         @arg @ref LL_I3C_HANDOFF_GRP_ADDR_SUPPORTED
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetGrpAddrHandoffSupport(I3C_TypeDef *p_i3c, uint32_t grp_addr_handoff_support)
{
  STM32_MODIFY_REG(p_i3c->CRCAPR, I3C_CRCAPR_CAPGRP, grp_addr_handoff_support);
}

/**
  * @brief  Get the Group Management Support bit of MSTCAP1.
  * @rmtoll
  *  CRCAPR      CAPGRP        LL_I3C_GetGrpAddrHandoffSupport
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED
  *         @arg @ref LL_I3C_HANDOFF_GRP_ADDR_SUPPORTED
  */
__STATIC_INLINE uint32_t LL_I3C_GetGrpAddrHandoffSupport(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CRCAPR, I3C_CRCAPR_CAPGRP));
}

/**
  * @brief  Configure the Delayed controller Handoff bit in MSTCAP2.
  * @rmtoll
  *  CRCAPR      CAPDHOFF      LL_I3C_SetControllerHandoffDelayed
  * @param  p_i3c I3C Instance.
  * @param  controller_handoff_delayed This parameter can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_NOT_DELAYED
  *         @arg @ref LL_I3C_HANDOFF_DELAYED
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetControllerHandoffDelayed(I3C_TypeDef *p_i3c, uint32_t controller_handoff_delayed)
{
  STM32_MODIFY_REG(p_i3c->CRCAPR, I3C_CRCAPR_CAPDHOFF, controller_handoff_delayed);
}

/**
  * @brief  Get the Delayed controller Handoff bit in MSTCAP2.
  * @rmtoll
  *  CRCAPR      CAPDHOFF      LL_I3C_GetControllerHandoffDelayed
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_NOT_DELAYED
  *         @arg @ref LL_I3C_HANDOFF_DELAYED
  */
__STATIC_INLINE uint32_t LL_I3C_GetControllerHandoffDelayed(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CRCAPR, I3C_CRCAPR_CAPDHOFF));
}

/**
  * @brief  Enable the Delayed controller Handoff bit in MSTCAP2.
  * @rmtoll
  *  CRCAPR      CAPDHOFF      LL_I3C_EnableHandOffDelay
  * @param  p_i3c I3C Instance.
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_EnableHandOffDelay(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CRCAPR, I3C_CRCAPR_CAPDHOFF);
}

/**
  * @brief  Disable the Delayed controller Handoff bit in MSTCAP2.
  * @rmtoll
  *  CRCAPR      CAPDHOFF      LL_I3C_DisableHandOffDelay
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableHandOffDelay(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->CRCAPR, I3C_CRCAPR_CAPDHOFF);
}

/**
  * @brief  Indicates if the Delayed controller Handoff bit in MSTCAP2 is enabled.
  *         RESET: IBI not Acknowledged.
  *         SET: IBI Acknowledged.
  * @rmtoll
  *  CRCAPR      CAPDHOFF        LL_I3C_IsEnabledHandOffDelay
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledHandOffDelay(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->CRCAPR, I3C_CRCAPR_CAPDHOFF) == I3C_CRCAPR_CAPDHOFF) ? 1UL : 0UL);
}

/**
  * @brief  Configure GETMXDS response fields.
  * @rmtoll
  *  GETMXDSR  HOFFAS,FMT,TSCO,RDTURN  LL_I3C_SetConfigGETMXDS
  * @param  p_i3c I3C Instance.
  * @param  ctrl_handoff_activity One of @ref LL_I3C_HANDOFF_ACTIVITY_STATE_0..3 (HOFFAS bits).
  * @param  getmxds_format One of @ref LL_I3C_GETMXDS_FORMAT_1 / _2_LSB / _2_MID / _2_MSB (FMT bits).
  * @param  data_turnaround_duration One of @ref LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS or
  *         @ref LL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS (TSCO bit).
  * @param  max_read_turnaround Value 0..0xF placed in RDTURN[3:0] (only used for format 2 variants).
  * @note   Programmable only when EN = 0.
  */
__STATIC_INLINE void LL_I3C_SetConfigGETMXDS(I3C_TypeDef *p_i3c,
                                             uint32_t ctrl_handoff_activity,
                                             uint32_t getmxds_format,
                                             uint32_t data_turnaround_duration,
                                             uint32_t max_read_turnaround)
{
  uint32_t getmxdsr_value = (ctrl_handoff_activity |
                             getmxds_format |
                             data_turnaround_duration |
                             ((uint32_t)max_read_turnaround << I3C_GETMXDSR_RDTURN_Pos));
  STM32_WRITE_REG(p_i3c->GETMXDSR, getmxdsr_value);
}

/**
  * @brief  Configure the activity state after controllership handoff.
  * @rmtoll
  *  GETMXDSR     HOFFAS        LL_I3C_SetHandoffActivityState
  * @param  p_i3c I3C Instance.
  * @param  handoff_activity_state This parameter can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_0
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_1
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_2
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_3
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetHandoffActivityState(I3C_TypeDef *p_i3c, uint32_t handoff_activity_state)
{
  STM32_MODIFY_REG(p_i3c->GETMXDSR, I3C_GETMXDSR_HOFFAS, handoff_activity_state);
}

/**
  * @brief  Get the activity state after controllership handoff.
  * @rmtoll
  *  GETMXDSR     HOFFAS        LL_I3C_GetHandoffActivityState
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_0
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_1
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_2
  *         @arg @ref LL_I3C_HANDOFF_ACTIVITY_STATE_3
  */
__STATIC_INLINE uint32_t LL_I3C_GetHandoffActivityState(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->GETMXDSR, I3C_GETMXDSR_HOFFAS));
}

/**
  * @brief  Configure the max data speed format response for GETMXDS CCC.
  * @rmtoll
  *  GETMXDSR     FMT          LL_I3C_SetMaxDataSpeedFormat
  * @param  p_i3c I3C Instance.
  * @param  max_data_speed_format This parameter can be one of the following values:
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_1
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_LSB
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_MID
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_MSB
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetMaxDataSpeedFormat(I3C_TypeDef *p_i3c, uint32_t max_data_speed_format)
{
  STM32_MODIFY_REG(p_i3c->GETMXDSR, I3C_GETMXDSR_FMT, max_data_speed_format);
}

/**
  * @brief  Get the max data Speed format response for GETMXDS CCC.
  * @rmtoll
  *  GETMXDSR     FMT          LL_I3C_GetMaxDataSpeedFormat
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_1
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_LSB
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_MID
  *         @arg @ref LL_I3C_GETMXDS_FORMAT_2_MSB
  */
__STATIC_INLINE uint32_t LL_I3C_GetMaxDataSpeedFormat(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->GETMXDSR, I3C_GETMXDSR_FMT));
}

/**
  * @brief  Configure the middle byte of MaxRdTurn field of GETMXDS CCC format 2 with turnaround.
  * @rmtoll
  *  GETMXDSR     RDTURN        LL_I3C_SetMiddleByteTurnAround
  * @param  p_i3c I3C Instance.
  * @param  middle_byte_turn_around This parameter must be a value between Min_Data=0 and Max_Data=0xF.
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetMiddleByteTurnAround(I3C_TypeDef *p_i3c, uint32_t middle_byte_turn_around)
{
  STM32_MODIFY_REG(p_i3c->GETMXDSR, I3C_GETMXDSR_RDTURN, (middle_byte_turn_around << I3C_GETMXDSR_RDTURN_Pos));
}

/**
  * @brief  Get the value of middle byte of MaxRdTurn field of GETMXDS CCC format 2 with turnaround.
  * @rmtoll
  *  GETMXDSR     RDTURN        LL_I3C_GetMiddleByteTurnAround
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetMiddleByteTurnAround(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->GETMXDSR, I3C_GETMXDSR_RDTURN));
}

/**
  * @brief  Configure clock-to-data turnaround time.
  * @rmtoll
  *  GETMXDSR     TSCO          LL_I3C_SetDataTurnAroundTime
  * @param  p_i3c I3C Instance.
  * @param  data_turn_around_time This parameter can be one of the following values:
  *         @arg @ref LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS
  *         @arg @ref LL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetDataTurnAroundTime(I3C_TypeDef *p_i3c, uint32_t data_turn_around_time)
{
  STM32_MODIFY_REG(p_i3c->GETMXDSR, I3C_GETMXDSR_TSCO, data_turn_around_time);
}

/**
  * @brief  Get clock-to-data turnaround time.
  * @rmtoll
  *  GETMXDSR     TSCO          LL_I3C_GetDataTurnAroundTime
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS
  *         @arg @ref LL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS
  */
__STATIC_INLINE uint32_t LL_I3C_GetDataTurnAroundTime(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->GETMXDSR, I3C_GETMXDSR_TSCO));
}

/**
  * @brief  Configure the MIPI Instance ID.
  * @rmtoll
  *  EPIDR        MIPIID       LL_I3C_SetMIPIInstanceID
  * @param  p_i3c I3C Instance.
  * @param  mipi_instance_id This parameter must be a value between Min_Data=0 and Max_Data=0xF.
  * @note   Those bits can be programmed when the I3C is disabled (EN = 0).
  */
__STATIC_INLINE void LL_I3C_SetMIPIInstanceID(I3C_TypeDef *p_i3c, uint32_t mipi_instance_id)
{
  STM32_MODIFY_REG(p_i3c->EPIDR, I3C_EPIDR_MIPIID, (mipi_instance_id << I3C_EPIDR_MIPIID_Pos));
}

/**
  * @brief  Get the MIPI Instance ID.
  * @rmtoll
  *  EPIDR        MIPIID       LL_I3C_GetMIPIInstanceID
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0xF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetMIPIInstanceID(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->EPIDR, I3C_EPIDR_MIPIID) >> I3C_EPIDR_MIPIID_Pos);
}

/**
  * @brief  Get the ID type selector.
  * @rmtoll
  *  EPIDR        IDTSEL        LL_I3C_GetIDTypeSelector
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0x0 and Max_Data=0x1
  */
__STATIC_INLINE uint32_t LL_I3C_GetIDTypeSelector(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->EPIDR, I3C_EPIDR_IDTSEL) >> I3C_EPIDR_IDTSEL_Pos);
}

/**
  * @brief  Get the MIPI Manufacturer ID.
  * @rmtoll
  *  EPIDR        MIPIMID       LL_I3C_GetMIPIManufacturerID
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 and Max_Data=0x7FFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetMIPIManufacturerID(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->EPIDR, I3C_EPIDR_MIPIMID) >> I3C_EPIDR_MIPIMID_Pos);
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_Data Management
  * @{
  */

/**
  * @brief  Request a reception data FIFO flush.
  * @rmtoll
  *  CFGR      RXFLUSH       LL_I3C_RequestRxFIFOFlush
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_RequestRxFIFOFlush(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_RXFLUSH);
}

/**
  * @brief  Request a transmission data FIFO Flush.
  * @rmtoll
  *  CFGR      TXFLUSH       LL_I3C_RequestTxFIFOFlush
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_RequestTxFIFOFlush(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_TXFLUSH);
}

/**
  * @brief  Request a status data FIFO Flush.
  * @rmtoll
  *  CFGR      SFLUSH        LL_I3C_RequestStatusFIFOFlush
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_RequestStatusFIFOFlush(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_SFLUSH);
}

/**
  * @brief  Set End of Frame mode.
  * @rmtoll
  *  CFGR      FCFDIS        LL_I3C_SetEndOfFrameMode
  * @param  p_i3c I3C Instance.
  * @param  eof_frame_config This parameter can be one of the following values:
  *         @arg @ref LL_I3C_END_OF_FRAME_CPLT_DISABLE
  *         @arg @ref LL_I3C_END_OF_FRAME_CPLT_ENABLE
  * @note   This bit can be modified only when there is no frame ongoing
  */
__STATIC_INLINE void LL_I3C_SetEndOfFrameMode(I3C_TypeDef *p_i3c, uint32_t eof_frame_config)
{
  STM32_MODIFY_REG(p_i3c->CFGR, I3C_CFGR_FCFDIS, eof_frame_config);
}

/**
  * @brief  Get End of Frame mode.
  * @rmtoll
  *  CFGR      FCFDIS        LL_I3C_GetEndOfFrameMode
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_END_OF_FRAME_CPLT_DISABLE
  *         @arg @ref LL_I3C_END_OF_FRAME_CPLT_ENABLE
  */
__STATIC_INLINE uint32_t LL_I3C_GetEndOfFrameMode(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->CFGR, I3C_CFGR_FCFDIS));
}

/**
  * @brief  Get activity state of controller on the I3C Bus (target only).
  * @rmtoll
  *  DEVR0        AS            LL_I3C_GetActivityState
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_BUS_ACTIVITY_STATE_0
  *         @arg @ref LL_I3C_BUS_ACTIVITY_STATE_1
  *         @arg @ref LL_I3C_BUS_ACTIVITY_STATE_2
  *         @arg @ref LL_I3C_BUS_ACTIVITY_STATE_3
  */
__STATIC_INLINE uint32_t LL_I3C_GetActivityState(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_AS));
}

/**
  * @brief  Get reset action (target only).
  * @rmtoll
  *  DEVR0        RSTACT        LL_I3C_GetResetAction
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_RESET_ACTION_NONE
  *         @arg @ref LL_I3C_RESET_ACTION_PARTIAL
  *         @arg @ref LL_I3C_RESET_ACTION_FULL
  */
__STATIC_INLINE uint32_t LL_I3C_GetResetAction(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->DEVR0, I3C_DEVR0_RSTACT));
}

/**
  * @brief  Request a control word FIFO Flush.
  * @rmtoll
  *  CFGR      CFLUSH        LL_I3C_RequestControlFIFOFlush
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_RequestControlFIFOFlush(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_CFLUSH);
}

/**
  * @brief  Request a combined flush of selected FIFOs (TX / RX / Status / Control).
  * @rmtoll
  *  CFGR      TXFLUSH        LL_I3C_RequestFifosFlush\n
  *  CFGR      RXFLUSH        LL_I3C_RequestFifosFlush\n
  *  CFGR      SFLUSH         LL_I3C_RequestFifosFlush\n
  *  CFGR      CFLUSH         LL_I3C_RequestFifosFlush
  * @param  p_i3c I3C instance.
  * @param  flush_mask this parameter is a combination of the following values:
  *         I3C_CFGR_TXFLUSH (TX FIFO)
  *         I3C_CFGR_RXFLUSH (RX FIFO)
  *         I3C_CFGR_SFLUSH  (Status FIFO)
  *         I3C_CFGR_CFLUSH  (Control FIFO)
  * @note   Each bit is auto cleared by hardware after the flush completes.
  */
__STATIC_INLINE void LL_I3C_RequestFifosFlush(I3C_TypeDef *p_i3c, uint32_t flush_mask)
{
  STM32_SET_BIT(p_i3c->CFGR, flush_mask);
}

/**
  * @brief  Request a Transfer start.
  * @rmtoll
  *  CFGR      TSFSET        LL_I3C_RequestTransfer
  * @param  p_i3c I3C Instance.
  * @note   After request, the current instruction in control Register is executed on I3C Bus.
  */
__STATIC_INLINE void LL_I3C_RequestTransfer(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->CFGR, I3C_CFGR_TSFSET);
}

/**
  * @brief  Handles I3C message content on the I3C Bus as controller.
  * @rmtoll
  *  CR           ADD           LL_I3C_ControllerHandleMessage \n
  *  CR           DCNT          LL_I3C_ControllerHandleMessage \n
  *  CR           RNW           LL_I3C_ControllerHandleMessage \n
  *  CR           MTYPE         LL_I3C_ControllerHandleMessage \n
  *  CR           MEND          LL_I3C_ControllerHandleMessage
  * @param  p_i3c I3C Instance.
  * @param  tgt_addr Specifies the target address to be programmed.
  *               This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  * @param  xfer_size Specifies the number of bytes to be programmed.
  *               This parameter must be a value between Min_Data=0 and Max_Data=65535.
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_I3C_DIRECTION_WRITE
  *         @arg @ref LL_I3C_DIRECTION_READ
  * @param  msg_type This parameter can be one of the following values:
  *         @arg @ref LL_I3C_CONTROLLER_MTYPE_RELEASE
  *         @arg @ref LL_I3C_CONTROLLER_MTYPE_HEADER
  *         @arg @ref LL_I3C_CONTROLLER_MTYPE_PRIVATE
  *         @arg @ref LL_I3C_CONTROLLER_MTYPE_DIRECT
  *         @arg @ref LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C
  * @param  end_mode This parameter can be one of the following values:
  *         @arg @ref LL_I3C_GENERATE_STOP
  *         @arg @ref LL_I3C_GENERATE_RESTART
  */
__STATIC_INLINE void LL_I3C_ControllerHandleMessage(I3C_TypeDef *p_i3c, uint32_t tgt_addr, uint32_t xfer_size,
                                                    uint32_t direction, uint32_t msg_type, uint32_t end_mode)
{
  STM32_WRITE_REG(p_i3c->CR, ((tgt_addr << I3C_CR_ADD_Pos) | xfer_size | direction | msg_type | end_mode) \
                  & (I3C_CR_ADD | I3C_CR_DCNT | I3C_CR_RNW | I3C_CR_MTYPE | I3C_CR_MEND));
}

/**
  * @brief  Issue a header message followed by STOP (no data, no address field).
  * @rmtoll
  *  CR           MTYPE        LL_I3C_ControllerHeaderStop \n
  *  CR           MEND         LL_I3C_ControllerHeaderStop
  * @param  p_i3c I3C Instance.
  * @note   Only header (7'h7E arbitration) then STOP. Use when control FIFO disabled and
  *         a pure header cycle is required (e.g. bus idle insertion).
  */
__STATIC_INLINE void LL_I3C_ControllerHeaderStop(I3C_TypeDef *p_i3c)
{
  LL_I3C_WRITE_REG(p_i3c, CR, (LL_I3C_CONTROLLER_MTYPE_HEADER | LL_I3C_GENERATE_STOP) & (I3C_CR_MTYPE | I3C_CR_MEND));
}

/**
  * @brief  Issue a RELEASE message followed by STOP (no data, no address field).
  * @rmtoll
  *  CR           MTYPE        LL_I3C_ControllerReleaseStop \n
  *  CR           MEND         LL_I3C_ControllerReleaseStop
  * @param  p_i3c I3C Instance.
  * @note   RELEASE stops SCL until next instruction; then STOP is generated.
  */
__STATIC_INLINE void LL_I3C_ControllerReleaseStop(I3C_TypeDef *p_i3c)
{
  LL_I3C_WRITE_REG(p_i3c, CR, (LL_I3C_CONTROLLER_MTYPE_RELEASE | LL_I3C_GENERATE_STOP) & (I3C_CR_MTYPE | I3C_CR_MEND));
}

/**
  * @brief  Handles I3C Common command Code content on the I3C Bus as controller.
  * @rmtoll
  *  CR           CCC           LL_I3C_ControllerHandleCCC \n
  *  CR           DCNT          LL_I3C_ControllerHandleCCC \n
  *  CR           MTYPE         LL_I3C_ControllerHandleCCC \n
  *  CR           MEND          LL_I3C_ControllerHandleCCC
  * @param  p_i3c I3C Instance.
  * @param  ccc_value Specifies the command Code to be programmed.
  *               This parameter must be a value between Min_Data=0 and Max_Data=0x1FF.
  * @param  add_byte_size Specifies the number of CCC additional bytes to be programmed.
  *               This parameter must be a value between Min_Data=0 and Max_Data=65535.
  * @param  end_mode This parameter can be one of the following values:
  *         @arg @ref LL_I3C_GENERATE_STOP
  *         @arg @ref LL_I3C_GENERATE_RESTART
  */
__STATIC_INLINE void LL_I3C_ControllerHandleCCC(I3C_TypeDef *p_i3c, uint32_t ccc_value,
                                                uint32_t add_byte_size, uint32_t end_mode)
{
  STM32_WRITE_REG(p_i3c->CR, ((ccc_value << I3C_CR_CCC_Pos) | add_byte_size | end_mode | LL_I3C_CONTROLLER_MTYPE_CCC) \
                  & (I3C_CR_CCC | I3C_CR_DCNT | I3C_CR_MTYPE | I3C_CR_MEND));
}

/**
  * @brief  Handles I3C message content on the I3C Bus as target.
  * @rmtoll
  *  CR           MTYPE         LL_I3C_TargetHandleMessage \n
  *  CR           DCNT          LL_I3C_TargetHandleMessage
  * @param  p_i3c I3C Instance.
  * @param  msg_type This parameter can be one of the following values:
  *         @arg @ref LL_I3C_TARGET_MTYPE_HOT_JOIN
  *         @arg @ref LL_I3C_TARGET_MTYPE_CONTROLLER_ROLE_REQ
  *         @arg @ref LL_I3C_TARGET_MTYPE_IBI
  * @param  ibi_size_byte Specifies the number of IBI bytes.
  *               This parameter must be a value between Min_Data=0 and Max_Data=65535.
  */
__STATIC_INLINE void LL_I3C_TargetHandleMessage(I3C_TypeDef *p_i3c, uint32_t msg_type, uint32_t ibi_size_byte)
{
  STM32_WRITE_REG(p_i3c->CR, (msg_type | ibi_size_byte) & (I3C_CR_DCNT | I3C_CR_MTYPE));
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read receive data byte register.
  * @rmtoll
  *  RDR          RDB0         LL_I3C_ReceiveData8
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 to Max_Data=0xFF
  */
__STATIC_INLINE uint8_t LL_I3C_ReceiveData8(const I3C_TypeDef *p_i3c)
{
  return (uint8_t)(STM32_READ_BIT(p_i3c->RDR, I3C_RDR_RDB0));
}

/**
  * @brief  Write in transmit data byte Register.
  * @rmtoll
  *  TDR          TDB0         LL_I3C_TransmitData8
  * @param  p_i3c I3C Instance.
  * @param  data This parameter must be a value between Min_Data=0 and Max_Data=0xFF.
  */
__STATIC_INLINE void LL_I3C_TransmitData8(I3C_TypeDef *p_i3c, uint8_t data)
{
  STM32_WRITE_REG(p_i3c->TDR, data);
}

/**
  * @brief  Read receive data word register.
  * @rmtoll
  *  RDWR         RDWR          LL_I3C_ReceiveData32
  * @param  p_i3c I3C Instance.
  * @note   Content of register is filled in Little Endian.
  *         The MSB corresponds to the last data byte received,
  *         LSB corresponds to first data byte received.
  * @retval Value between Min_Data=0 to Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_I3C_ReceiveData32(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->RDWR));
}

/**
  * @brief  Write in transmit data word register.
  * @rmtoll
  *  TDWR         TDWR          LL_I3C_TransmitData32
  * @param  p_i3c I3C Instance.
  * @param  data This parameter must be a value between Min_Data=0 and Max_Data=0xFFFFFFFF.
  * @note   Content of register is filled in Little Endian.
  *         The MSB corresponds to the last data byte transmitted,
  *         LSB corresponds to first data byte transmitted.
  */
__STATIC_INLINE void LL_I3C_TransmitData32(I3C_TypeDef *p_i3c, uint32_t data)
{
  STM32_WRITE_REG(p_i3c->TDWR, data);
}

/**
  * @brief  Write a single control word to CR.
  * @rmtoll       CR           LL_I3C_WriteControlWord
  * @param  p_i3c I3C Instance.
  * @param  control_word 32-bit control word (ADD/DCNT/RNW/MTYPE/MEND bits as needed).
  * @note   Use when control FIFO (TMODE) is disabled and one instruction must be issued.
  */
__STATIC_INLINE void LL_I3C_WriteControlWord(I3C_TypeDef *p_i3c, uint32_t control_word)
{
  STM32_WRITE_REG(p_i3c->CR, control_word);
}

/**
  * @brief  Configure the IBI data payload to be sent during IBI (target mode).
  * @rmtoll
  *  IBIDR        IBIDR         LL_I3C_SetIBIPayload
  * @param  p_i3c I3C Instance.
  * @param  own_ibi_payload This parameter must be a value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @note   Content of register is filled in Little Endian.
  *         Mean MSB corresponds to last IBI data byte,
  *         LSB corresponds to first IBI data byte.
  */
__STATIC_INLINE void LL_I3C_SetIBIPayload(I3C_TypeDef *p_i3c, uint32_t own_ibi_payload)
{
  STM32_WRITE_REG(p_i3c->IBIDR, own_ibi_payload);
}

/**
  * @brief  Get the own IBI data payload (target mode), or get the target IBI received (controller mode).
  * @rmtoll
  *  IBIDR        IBIDR         LL_I3C_GetIBIPayload
  * @param  p_i3c I3C Instance.
  * @note   Content of register is filled in Little Endian.
  *         Mean MSB corresponds to last IBI data byte,
  *         LSB corresponds to first IBI data byte.
  * @retval Value between Min_Data=0 to Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_I3C_GetIBIPayload(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_REG(p_i3c->IBIDR));
}

/**
  * @brief  Get the number of data bytes received when reading IBI data (controller mode).
  * @rmtoll
  *  RMR         IBIRDCNT     LL_I3C_GetNbIBIAddData
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 to Max_Data=0x7
  */
__STATIC_INLINE uint32_t LL_I3C_GetNbIBIAddData(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->RMR, I3C_RMR_IBIRDCNT));
}

/**
  * @brief  Get the target address received during accepted IBI or controller-role request.
  * @rmtoll
  *  RMR         RADD         LL_I3C_GetIBITargetAddr
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 to Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_I3C_GetIBITargetAddr(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->RMR, I3C_RMR_RADD) >> I3C_RMR_RADD_Pos);
}

/**
  * @brief  Set TX FIFO Preload (target mode).
  * @rmtoll
  *  TGTTDR       PRELOAD       LL_I3C_ConfigTxPreload \n
  *  TGTTDR       TDCNT         LL_I3C_ConfigTxPreload
  * @param  p_i3c I3C Instance.
  * @param  tx_data_count This parameter must be a value between Min_Data=0 and Max_Data=0xFFFF
  * @note   Set high by Software, cleared by hardware when all the bytes to transmit have been loaded to TX FIFO.
  */
__STATIC_INLINE void LL_I3C_ConfigTxPreload(I3C_TypeDef *p_i3c, uint16_t tx_data_count)
{
  STM32_MODIFY_REG(p_i3c->TGTTDR, (I3C_TGTTDR_PRELOAD | I3C_TGTTDR_TGTTDCNT), (I3C_TGTTDR_PRELOAD | tx_data_count));
}

/**
  * @brief  Indicates the status of TX FIFO preload (target mode).
  *         RESET: no preload of TX FIFO.
  *         SET: preload of TX FIFO ongoing.
  * @rmtoll
  *  TGTTDR       PRELOAD       LL_I3C_IsActiveTxPreload
  * @param  p_i3c I3C Instance.
  * @note   Set high by Software, cleared by hardware when all the bytes to transmit have been loaded to TX FIFO.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveTxPreload(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->TGTTDR, I3C_TGTTDR_PRELOAD) == (I3C_TGTTDR_PRELOAD)) ? 1UL : 0UL);
}

/**
  * @brief  Get the number of bytes to transmit (target mode).
  * @rmtoll
  *  TGTTDR       TDCNT        LL_I3C_GetTxPreloadDataCount
  * @param  p_i3c I3C Instance.
  * @note   The return value corresponds to the remaining number of bytes to load in TX FIFO.
  * @retval Value between Min_Data=0 to Max_Data=0xFFFF
  */
__STATIC_INLINE uint16_t LL_I3C_GetTxPreloadDataCount(const I3C_TypeDef *p_i3c)
{
  return (uint16_t)(STM32_READ_BIT(p_i3c->TGTTDR, I3C_TGTTDR_TGTTDCNT));
}

/**
  * @brief  Get the number of data during a transfer.
  * @rmtoll
  *  SR           XDCNT      LL_I3C_GetXferDataCount
  * @param  p_i3c I3C Instance.
  * @note   The return value corresponds to number of transmitted bytes reported
  *         during Address Assignment process in target mode.
  * The return value  corresponds to number of target detected
  * during address Assignment process in controller mode.
  * The return value  corresponds to number of data bytes read from or sent to the I3C bus
  * during the message link to MID current value.
  * @retval Value between Min_Data=0 to Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_I3C_GetXferDataCount(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->SR, I3C_SR_XDCNT));
}

/**
  * @brief  Indicates if a target abort a private read command.
  * @rmtoll
  *  SR           ABT           LL_I3C_IsTargetAbortPrivateRead
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsTargetAbortPrivateRead(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SR, I3C_SR_ABT) == (I3C_SR_ABT)) ? 1UL : 0UL);
}

/**
  * @brief  Get direction of the message.
  * @rmtoll
  *  SR           DIR           LL_I3C_GetMessageDirection
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_MESSAGE_DIRECTION_WRITE
  *         @arg @ref LL_I3C_MESSAGE_DIRECTION_READ
  */
__STATIC_INLINE uint32_t LL_I3C_GetMessageDirection(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->SR, I3C_SR_DIR));
}

/**
  * @brief  Get message identifier.
  * @rmtoll
  *  SR           MID           LL_I3C_GetMessageIdentifier
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 to Max_Data=0xFF, representing the internal hardware counter value.
  */
__STATIC_INLINE uint32_t LL_I3C_GetMessageIdentifier(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->SR, I3C_SR_MID) >> I3C_SR_MID_Pos);
}

/**
  * @brief  Get message error code.
  * @rmtoll
  *  SER          CODERR        LL_I3C_GetMessageErrorCode
  * @param  p_i3c I3C Instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I3C_CONTROLLER_ERROR_CE0
  *         @arg @ref LL_I3C_CONTROLLER_ERROR_CE1
  *         @arg @ref LL_I3C_CONTROLLER_ERROR_CE2
  *         @arg @ref LL_I3C_CONTROLLER_ERROR_CE3
  *         @arg @ref LL_I3C_TARGET_ERROR_TE0
  *         @arg @ref LL_I3C_TARGET_ERROR_TE1
  *         @arg @ref LL_I3C_TARGET_ERROR_TE2
  *         @arg @ref LL_I3C_TARGET_ERROR_TE3
  *         @arg @ref LL_I3C_TARGET_ERROR_TE4
  *         @arg @ref LL_I3C_TARGET_ERROR_TE5
  *         @arg @ref LL_I3C_TARGET_ERROR_TE6
  */
__STATIC_INLINE uint32_t LL_I3C_GetMessageErrorCode(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->SER, I3C_SER_CODERR));
}

/**
  * @brief  Get CCC code of received command.
  * @rmtoll
  *  RMR         RCODE        LL_I3C_GetReceiveCommandCode
  * @param  p_i3c I3C Instance.
  * @retval Value between Min_Data=0 to Max_Data=0xFF.
  */
__STATIC_INLINE uint32_t LL_I3C_GetReceiveCommandCode(const I3C_TypeDef *p_i3c)
{
  return (uint32_t)(STM32_READ_BIT(p_i3c->RMR, I3C_RMR_RCODE) >> I3C_RMR_RCODE_Pos);
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_Target Payload
  * @{
  */

/**
  * @brief  Set dynamic address assigned to target x.
  * @rmtoll
  *  DEVRX        DA            LL_I3C_SetTargetDynamicAddress
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @param  dynamic_addr Value between Min_Data=0 to Max_Data=0x7F
  */
__STATIC_INLINE void LL_I3C_SetTargetDynamicAddress(I3C_TypeDef *p_i3c, uint32_t target_id, uint32_t dynamic_addr)
{
  STM32_MODIFY_REG(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_DA, (dynamic_addr << I3C_DEVRX_DA_Pos));
}

/**
  * @brief  Get dynamic address assigned to target x.
  * @rmtoll
  *  DEVRX        DA            LL_I3C_GetTargetDynamicAddress
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval Value between Min_Data=0 to Max_Data=0x7F
  */
__STATIC_INLINE uint32_t LL_I3C_GetTargetDynamicAddress(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return (uint32_t)((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_DA)) >> I3C_DEVRX_DA_Pos);
}

/**
  * @brief  Enable IBI acknowledgement from target x(controller mode).
  * @rmtoll
  *  DEVRX        IBIACK        LL_I3C_EnableTargetIBIAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @note   The bit DIS is automatically set when CRACK or IBIACK are set.
  *         This means DEVRX register access is not allowed.
  *         Reset CRACK and IBIACK will reset DIS bit.
  */
__STATIC_INLINE void LL_I3C_EnableTargetIBIAck(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_SET_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIACK);
}

/**
  * @brief  Disable IBI acknowledgement from target x (controller mode).
  * @rmtoll
  *  DEVRX        IBIACK        LL_I3C_DisableTargetIBIAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  */
__STATIC_INLINE void LL_I3C_DisableTargetIBIAck(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_CLEAR_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIACK);
}

/**
  * @brief  Indicates if IBI from target x will be Acknowledged or not Acknowledged (controller mode).
  *         RESET: IBI not Acknowledged.
  *         SET: IBI Acknowledged.
  * @rmtoll
  *  DEVRX        IBIACK        LL_I3C_IsEnabledTargetIBIAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledTargetIBIAck(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return ((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIACK) == I3C_DEVRX_IBIACK) ? 1UL : 0UL);
}

/**
  * @brief  Enable controller-role request acknowledgement from target x(controller mode).
  * @rmtoll
  *  DEVRX        CRACK         LL_I3C_EnableTargetCRAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @note   The bit DIS is automatically set when CRACK or IBIACK are set.
  *         This means DEVRX register access is not allowed.
  *         Reset CRACK and IBIACK will reset DIS bit.
  */
__STATIC_INLINE void LL_I3C_EnableTargetCRAck(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_SET_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_CRACK);
}

/**
  * @brief  Disable controller-role request acknowledgement from target x (controller mode).
  * @rmtoll
  *  DEVRX        CRACK         LL_I3C_DisableTargetCRAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  */
__STATIC_INLINE void LL_I3C_DisableTargetCRAck(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_CLEAR_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_CRACK);
}

/**
  * @brief  Indicates if controller-role request from target x will be
  *         Acknowledged or not Acknowledged (controller mode).
  *         RESET: controller-role request not Acknowledged.
  *         SET: controller-role request Acknowledged.
  * @rmtoll
  *  DEVRX        CRACK         LL_I3C_IsEnabledTargetCRAck
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledTargetCRAck(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return ((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_CRACK) == I3C_DEVRX_CRACK) ? 1UL : 0UL);
}

/**
  * @brief  Enable additional Mandatory data Byte (MDB) follows the accepted IBI from target x.
  * @rmtoll
  *  DEVRX        IBIDEN          LL_I3C_EnableIBIAddData
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  */
__STATIC_INLINE void LL_I3C_EnableIBIAddData(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_SET_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIDEN);
}

/**
  * @brief  Disable additional Mandatory data Byte (MDB) follows the accepted IBI from target x.
  * @rmtoll
  *  DEVRX        IBIDEN          LL_I3C_DisableIBIAddData
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  */
__STATIC_INLINE void LL_I3C_DisableIBIAddData(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_CLEAR_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIDEN);
}

/**
  * @brief  Indicates if additional Mandatory data Byte (MDB) follows the accepted IBI from target x.
  *         RESET: no Mandatory data Byte follows IBI.
  *         SET: mandatory data Byte follows IBI.
  * @rmtoll
  *  DEVRX        IBIDEN          LL_I3C_IsEnabledIBIAddData
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIBIAddData(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return ((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_IBIDEN) == I3C_DEVRX_IBIDEN) ? 1UL : 0UL);
}

/**
  * @brief  Enable Suspension of Current transfer during IBI treatment.
  * @rmtoll
  *  DEVRX        SUSP          LL_I3C_EnableFrameSuspend
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @note   When set, this feature will allow controller to send a stop condition and CR FIFO is flushed after
  *         IBI treatment.
  *         Software has to rewrite instructions in control Register to start a new transfer.
  */
__STATIC_INLINE void LL_I3C_EnableFrameSuspend(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_SET_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_SUSP);
}

/**
  * @brief  Disable Suspension of Current transfer during IBI treatment.
  * @rmtoll
  *  DEVRX        SUSP          LL_I3C_DisableFrameSuspend
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @note   When set, this feature will allow controller to continue CR FIFO treatment after IBI treatment.
  */
__STATIC_INLINE void LL_I3C_DisableFrameSuspend(I3C_TypeDef *p_i3c, uint32_t target_id)
{
  STM32_CLEAR_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_SUSP);
}

/**
  * @brief  Indicates if I3C transfer must be Suspended or not Suspended during IBI treatment from target x.
  *         RESET: transfer is not suspended. Instruction in CR FIFO are executed after IBI.
  *         SET: transfer is suspended (a stop condition is sent). CR FIFO is flushed.
  * @rmtoll
  *  DEVRX        SUSP          LL_I3C_IsFrameMustBeSuspended
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsFrameMustBeSuspended(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return ((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_SUSP) == I3C_DEVRX_SUSP) ? 1UL : 0UL);
}

/**
  * @brief  Indicates if update of the device Characteristics Register is Allowed or not Allowed.
  *         RESET: device Characteristics Register update is not Allowed.
  *         SET: device Characteristics Register update is Allowed.
  * @rmtoll
  *  DEVRX        DIS           LL_I3C_IsAllowedPayloadUpdate
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @note   Used to prevent software writing during reception of an IBI or controller-role request from target x.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsAllowedPayloadUpdate(const I3C_TypeDef *p_i3c, uint32_t target_id)
{
  return ((STM32_READ_BIT(p_i3c->DEVRX[target_id - 1U], I3C_DEVRX_DIS) != I3C_DEVRX_DIS) ? 1UL : 0UL);
}

/**
  * @brief  Set I3C bus devices configuration.
  * @rmtoll
  *  DEVRX        DA            LL_I3C_ConfigDeviceCapabilities \n
  *  DEVRX        IBIACK        LL_I3C_ConfigDeviceCapabilities \n
  *  DEVRX        IBIDEN        LL_I3C_ConfigDeviceCapabilities \n
  *  DEVRX        CRACK         LL_I3C_ConfigDeviceCapabilities
  * @param  p_i3c I3C Instance.
  * @param  target_id This parameter must be a value between Min_Data=1 and Max_Data=4
  * @param  dynamic_addr Value between Min_Data=0 to Max_Data=0x7F
  * @param  IBIAck Value This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_CAPABILITY
  *         @arg @ref LL_I3C_IBI_NO_CAPABILITY
  * @param  IBIAddData This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_DATA_ENABLE
  *         @arg @ref LL_I3C_IBI_DATA_DISABLE
  * @param  CRAck This parameter can be one of the following values:
  *         @arg @ref LL_I3C_CR_CAPABILITY
  *         @arg @ref LL_I3C_CR_NO_CAPABILITY
  * @note   This function is called only when the I3C instance is initialized as controller.
  *         This function can be called by the controller application to help the automatic treatment when target have
  *         capability of IBI and/or control-Role.
  */
__STATIC_INLINE void LL_I3C_ConfigDeviceCapabilities(I3C_TypeDef *p_i3c,
                                                     uint32_t target_id,
                                                     uint32_t dynamic_addr,
                                                     uint32_t IBIAck,
                                                     uint32_t IBIAddData,
                                                     uint32_t CRAck)
{
  STM32_MODIFY_REG(p_i3c->DEVRX[target_id - 1U], \
                   (I3C_DEVRX_DA | I3C_DEVRX_IBIACK | I3C_DEVRX_CRACK | I3C_DEVRX_IBIDEN), \
                   ((dynamic_addr << I3C_DEVRX_DA_Pos) | IBIAck | IBIAddData | CRAck));
}

/**
  * @brief  Set DEVRx.
  * @rmtoll
  *  DEVRX        DA            LL_I3C_SetDevrx \n
  *  DEVRX        IBIACK        LL_I3C_SetDevrx \n
  *  DEVRX        IBIDEN        LL_I3C_SetDevrx \n
  *  DEVRX        SUSP          LL_I3C_SetDevrx \n
  *  DEVRX        CRACK         LL_I3C_SetDevrx
  * @param  p_i3c I3C Instance.
  * @param  devrx_index This parameter must be a value between Min_Data=0 and Max_Data=4
  * @param  dynamic_addr Value between Min_Data=0 to Max_Data=0x7F
  * @param  IBIAck Value This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_CAPABILITY
  *         @arg @ref LL_I3C_IBI_NO_CAPABILITY
   * @param  Susp This parameter can be one of the following values:
   *         @arg @ref LL_I3C_SUSP_ENABLE
   *         @arg @ref LL_I3C_SUSP_DISABLE
  * @param  IBIAddData This parameter can be one of the following values:
  *         @arg @ref LL_I3C_IBI_DATA_ENABLE
  *         @arg @ref LL_I3C_IBI_DATA_DISABLE
   * @param  CRAck This parameter can be one of the following values:
  *         @arg @ref LL_I3C_CR_CAPABILITY
  *         @arg @ref LL_I3C_CR_NO_CAPABILITY
  * @note   This function is called only when the I3C instance is initialized as controller.
  *         This function can be called by the controller application to help the automatic treatment when target have
  *         capability of IBI and/or control-Role.
  */
__STATIC_INLINE void LL_I3C_SetDevrx(I3C_TypeDef *p_i3c,
                                     uint32_t devrx_index,
                                     uint32_t dynamic_addr,
                                     uint32_t IBIAck,
                                     uint32_t Susp,
                                     uint32_t IBIAddData,
                                     uint32_t CRAck)
{
  STM32_WRITE_REG(p_i3c->DEVRX[devrx_index], ((dynamic_addr << I3C_DEVRX_DA_Pos) | IBIAck | Susp | IBIAddData | CRAck));
}

/**
  * @brief  Get DEVRx raw register value.
  * @rmtoll
  *  DEVRX        DA            LL_I3C_GetDevrx \n
  *  DEVRX        IBIACK        LL_I3C_GetDevrx \n
  *  DEVRX        IBIDEN        LL_I3C_GetDevrx \n
  *  DEVRX        SUSP          LL_I3C_GetDevrx \n
  *  DEVRX        CRACK         LL_I3C_GetDevrx
  * @param  p_i3c I3C Instance.
  * @param  devrx_index This parameter must be a value between Min_Data=0 and Max_Data=4
  * @retval Raw 32-bit value of DEVRX[devrx_index].
  */
__STATIC_INLINE uint32_t LL_I3C_GetDevrx(const I3C_TypeDef *p_i3c, uint32_t devrx_index)
{
  return (uint32_t)STM32_READ_REG(p_i3c->DEVRX[devrx_index]);
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_FLAG_management FLAG_management
  * @{
  */

/**
  * @brief  Check if one of the flags is active.
  * @rmtoll EVR <flag> LL_I3C_IsActiveFlag
  * @param  p_i3c I3C Instance.
  * @param  flags  I3C_LL_EC_GET_FLAG.
  * @retval 0 no flag is set
  * @retval 1 at least one of the flags is set
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag(const I3C_TypeDef *p_i3c, uint32_t flags)
{
  return ((STM32_READ_BIT(p_i3c->EVR, (flags)) != 0U) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of control FIFO empty flag.
  *         RESET: one or more data are available in control FIFO.
  *         SET: no more data available in control FIFO.
  * @rmtoll
  *  EVR          CFEF          LL_I3C_IsActiveFlag_CFE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_CFE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_CFEF) == (I3C_EVR_CFEF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of transmit FIFO empty flag.
  *         RESET: one or more data are available in transmit FIFO.
  *         SET: no more data available in transmit FIFO.
  * @rmtoll
  *  EVR          TXFEF         LL_I3C_IsActiveFlag_TXFE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_TXFE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_TXFEF) == (I3C_EVR_TXFEF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of control FIFO not full flag.
  *         RESET: one or more free space available in control FIFO.
  *         SET: no more free space available in control FIFO.
  * @rmtoll
  *  EVR          CFNFF         LL_I3C_IsActiveFlag_CFNF
  * @param  p_i3c I3C Instance.
  * @note   When a transfer is ongoing, the control FIFO must not be written unless this flag is set.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_CFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_CFNFF) == (I3C_EVR_CFNFF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of status FIFO not empty flag.
  *         RESET: one or more free space available in status FIFO.
  *         SET: no more free space available in status FIFO.
  * @rmtoll
  *  EVR          SFNEF         LL_I3C_IsActiveFlag_SFNE
  * @param  p_i3c I3C Instance.
  * @note   This flag is updated only when the FIFO is used, mean SMODE = 1.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_SFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_SFNEF) == (I3C_EVR_SFNEF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of transmit FIFO not full flag.
  *         RESET: one or more free space available in transmit FIFO.
  *         SET: no more free space available in transmit FIFO.
  * @rmtoll
  *  EVR          TXFNFF        LL_I3C_IsActiveFlag_TXFNF
  * @param  p_i3c I3C Instance.
  * @note   When a transfer is ongoing, the transmit FIFO must not be written unless this flag is set.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_TXFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_TXFNFF) == (I3C_EVR_TXFNFF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of receive FIFO not full flag.
  *         RESET: one or more data are available in receive FIFO.
  *         SET: no more data available in receive FIFO.
  * @rmtoll
  *  EVR          RXFNEF        LL_I3C_IsActiveFlag_RXFNE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_RXFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_RXFNEF) == (I3C_EVR_RXFNEF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates that the last receive byte is available.
  *         RESET: clear default value.
  *         SET: last receive byte ready to read from receive FIFO.
  * @rmtoll
  *  EVR          RXLASTF       LL_I3C_IsActiveFlag_RXLAST
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_RXLAST(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_RXLASTF) == (I3C_EVR_RXLASTF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates that the last transmit byte is written in FIFO.
  *         RESET: transmission is not finalized.
  *         SET: last transmit byte is written in transmit FIFO.
  * @rmtoll
  *  EVR          TXLASTF       LL_I3C_IsActiveFlag_TXLAST
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_TXLAST(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_TXLASTF) == (I3C_EVR_TXLASTF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of Frame Complete flag (controller and target mode).
  *         RESET: current Frame transfer is not finalized.
  *         SET: current Frame transfer is completed.
  * @rmtoll
  *  EVR          FCF           LL_I3C_IsActiveFlag_FC
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_FC(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_FCF) == (I3C_EVR_FCF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of reception target end flag (controller mode).
  *         RESET: clear default value.
  *         SET: target prematurely ended a read command.
  * @rmtoll
  *  EVR          RXTGTENDF     LL_I3C_IsActiveFlag_RXTGTEND
  * @param  p_i3c I3C Instance.
  * @note   This flag is set only when status FIFO is not used, mean SMODE = 0.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_RXTGTEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_RXTGTENDF) == (I3C_EVR_RXTGTENDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of error flag (controller and target mode).
  *         RESET: clear default value.
  *         SET: One or more Errors are detected.
  * @rmtoll
  *  EVR          ERRF          LL_I3C_IsActiveFlag_ERR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_ERR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_ERRF) == (I3C_EVR_ERRF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of IBI flag (controller mode).
  *         RESET: clear default value.
  *         SET: An IBI has been received.
  * @rmtoll
  *  EVR          IBIF          LL_I3C_IsActiveFlag_IBI
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_IBI(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_IBIF) == (I3C_EVR_IBIF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of IBI end flag (target mode).
  *         RESET: clear default value.
  *         SET: IBI procedure is finished.
  * @rmtoll
  *  EVR          IBIENDF       LL_I3C_IsActiveFlag_IBIEND
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_IBIEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_IBIENDF) == (I3C_EVR_IBIENDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of controller-role request flag (controller mode).
  *         RESET: clear default value.
  *         SET: a controller-role request procedure has been received.
  * @rmtoll
  *  EVR          CRF           LL_I3C_IsActiveFlag_CR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_CR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_CRF) == (I3C_EVR_CRF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of controller-role request update flag (target mode).
  *         RESET: clear default value.
  *         SET: I3C device has gained controller-role of the I3C Bus.
  * @rmtoll
  *  EVR          BCUPDF        LL_I3C_IsActiveFlag_CRUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_CRUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_CRUPDF) == (I3C_EVR_CRUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of Hot-Join flag (controller mode).
  *         RESET: clear default value.
  *         SET: a Hot-Join request has been received.
  * @rmtoll
  *  EVR          HJF           LL_I3C_IsActiveFlag_HJ
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_HJ(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_HJF) == (I3C_EVR_HJF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of wakeup flag (target mode).
  *         RESET: clear default value.
  *         SET: I3C Internal clock not available on time to treat the falling edge on SCL.
  * @rmtoll
  *  EVR          WKPF          LL_I3C_IsActiveFlag_WKP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_WKP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_WKPF) == (I3C_EVR_WKPF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of get flag (target mode).
  *         RESET: clear default value.
  *         SET: a "get" type CCC has been received.
  * @rmtoll
  *  EVR          GETF          LL_I3C_IsActiveFlag_GET
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_GET(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_GETF) == (I3C_EVR_GETF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of get status flag (target mode).
  *         RESET: clear default value.
  *         SET: a GETSTATUS command has been received.
  * @rmtoll
  *  EVR          STAF          LL_I3C_IsActiveFlag_STA
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_STA(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_STAF) == (I3C_EVR_STAF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of dynamic address update flag (target mode).
  *         RESET: clear default value.
  *         SET: own dynamic address has been updated.
  * @rmtoll
  *  EVR          DAUPDF        LL_I3C_IsActiveFlag_DAUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_DAUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_DAUPDF) == (I3C_EVR_DAUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of max write length flag (target mode).
  *         RESET: clear default value.
  *         SET: max write length has been updated.
  * @rmtoll
  *  EVR          MWLUPDF       LL_I3C_IsActiveFlag_MWLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_MWLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_MWLUPDF) == (I3C_EVR_MWLUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of max read length flag (target mode).
  *         RESET: clear default value.
  *         SET: max read length has been updated.
  * @rmtoll
  *  EVR          MRLUPDF       LL_I3C_IsActiveFlag_MRLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_MRLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_MRLUPDF) == (I3C_EVR_MRLUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of reset flag (target mode).
  *         RESET: clear default value.
  *         SET: a reset pattern has been received.
  * @rmtoll
  *  EVR          RSTF          LL_I3C_IsActiveFlag_RST
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_RST(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_RSTF) == (I3C_EVR_RSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of active state flag (target mode).
  *         RESET: clear default value.
  *         SET: the activity state has been updated.
  * @rmtoll
  *  EVR          ASUPDF        LL_I3C_IsActiveFlag_ASUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_ASUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_ASUPDF) == (I3C_EVR_ASUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of interrupt update flag (target mode).
  *         RESET: clear default value.
  *         SET: One or more authorized interrupts have been updated.
  * @rmtoll
  *  EVR          INTUPDF       LL_I3C_IsActiveFlag_INTUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_INTUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_INTUPDF) == (I3C_EVR_INTUPDF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of define list targets flag (target mode).
  *         RESET: clear default value.
  *         SET: a define list targets command has been received.
  * @rmtoll
  *  EVR          DEFF          LL_I3C_IsActiveFlag_DEF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_DEF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_DEFF) == (I3C_EVR_DEFF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of define list Group addresses flag.
  *         RESET: clear default value.
  *         SET: define list Group addresses have been received.
  * @rmtoll
  *  EVR          GRPF          LL_I3C_IsActiveFlag_GRP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_GRP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->EVR, I3C_EVR_GRPF) == (I3C_EVR_GRPF)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of Protocol error flag.
  *         RESET: clear default value.
  *         SET: protocol error detected.
  * @rmtoll
  *  SER          PERR          LL_I3C_IsActiveFlag_PERR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_PERR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_PERR) == (I3C_SER_PERR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of SCL Stall error flag (target mode).
  *         RESET: clear default value.
  *         SET: target detected that SCL was stable for more than 125us during I3C SDR read.
  * @rmtoll
  *  SER          STALL         LL_I3C_IsActiveFlag_STALL
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_STALL(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_STALL) == (I3C_SER_STALL)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of RX or TX FIFO Overrun flag.
  *         RESET: clear default value.
  *         SET: RX FIFO full or TX FIFO empty depending of direction of message.
  * @rmtoll
  *  SER          DOVR          LL_I3C_IsActiveFlag_DOVR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_DOVR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_DOVR) == (I3C_SER_DOVR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of control or status FIFO Overrun flag (controller mode).
  *         RESET: clear default value.
  *         SET: status FIFO full or control FIFO empty after Restart.
  * @rmtoll
  *  SER          COVR          LL_I3C_IsActiveFlag_COVR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_COVR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_COVR) == (I3C_SER_COVR)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of address not acknowledged flag (controller mode).
  *         RESET: clear default value.
  *         SET: controller detected that target NACKed static or dynamic address.
  * @rmtoll
  *  SER          ANACK         LL_I3C_IsActiveFlag_ANACK
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_ANACK(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_ANACK) == (I3C_SER_ANACK)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of data not acknowledged flag (controller mode).
  *         RESET: clear default value.
  *         SET: controller detected that target NACKed data byte.
  * @rmtoll
  *  SER          DNACK         LL_I3C_IsActiveFlag_DNACK
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_DNACK(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_DNACK) == (I3C_SER_DNACK)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of data error flag (controller mode).
  *         RESET: clear default value.
  *         SET: controller detected data error during controller-role handoff process.
  * @rmtoll
  *  SER          DERR          LL_I3C_IsActiveFlag_DERR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveFlag_DERR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->SER, I3C_SER_DERR) == (I3C_SER_DERR)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_IT_Management IT_Management
  * @{
  */


/**
  * @brief  Enable the specified I3C interrupts.
  * @rmtoll
  *  IER  CFNFIE     LL_I3C_EnableIT  \n
  *  IER  SFNEIE     LL_I3C_EnableIT  \n
  *  IER  TXFNFIE    LL_I3C_EnableIT  \n
  *  IER  RXFNEIE    LL_I3C_EnableIT  \n
  *  IER  FCIE       LL_I3C_EnableIT  \n
  *  IER  RXTGTENDIE LL_I3C_EnableIT  \n
  *  IER  ERRIE      LL_I3C_EnableIT  \n
  *  IER  IBIIE      LL_I3C_EnableIT  \n
  *  IER  IBIENDIE   LL_I3C_EnableIT  \n
  *  IER  CRIE       LL_I3C_EnableIT  \n
  *  IER  CRUPDIE    LL_I3C_EnableIT  \n
  *  IER  HJIE       LL_I3C_EnableIT  \n
  *  IER  WKPIE      LL_I3C_EnableIT  \n
  *  IER  GETIE      LL_I3C_EnableIT  \n
  *  IER  STAIE      LL_I3C_EnableIT  \n
  *  IER  DAUPDIE    LL_I3C_EnableIT  \n
  *  IER  MWLUPDIE   LL_I3C_EnableIT  \n
  *  IER  MRLUPDIE   LL_I3C_EnableIT  \n
  *  IER  RSTIE      LL_I3C_EnableIT  \n
  *  IER  ASUPDIE    LL_I3C_EnableIT  \n
  *  IER  INTUPDIE   LL_I3C_EnableIT  \n
  *  IER  DEFIE      LL_I3C_EnableIT  \n
  *  IER  GRPIE      LL_I3C_EnableIT
  * @param  p_i3c I3C instance.
  * @param  mask interrupt sources to enable.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I3C_IER_CFNFIE
  *            @arg @ref LL_I3C_IER_SFNEIE
  *            @arg @ref LL_I3C_IER_TXFNFIE
  *            @arg @ref LL_I3C_IER_RXFNEIE
  *            @arg @ref LL_I3C_IER_FCIE
  *            @arg @ref LL_I3C_IER_RXTGTENDIE
  *            @arg @ref LL_I3C_IER_ERRIE
  *            @arg @ref LL_I3C_IER_IBIIE
  *            @arg @ref LL_I3C_IER_IBIENDIE
  *            @arg @ref LL_I3C_IER_CRIE
  *            @arg @ref LL_I3C_IER_CRUPDIE
  *            @arg @ref LL_I3C_IER_HJIE
  *            @arg @ref LL_I3C_IER_WKPIE
  *            @arg @ref LL_I3C_IER_GETIE
  *            @arg @ref LL_I3C_IER_STAIE
  *            @arg @ref LL_I3C_IER_DAUPDIE
  *            @arg @ref LL_I3C_IER_MWLUPDIE
  *            @arg @ref LL_I3C_IER_MRLUPDIE
  *            @arg @ref LL_I3C_IER_RSTIE
  *            @arg @ref LL_I3C_IER_ASUPDIE
  *            @arg @ref LL_I3C_IER_INTUPDIE
  *            @arg @ref LL_I3C_IER_DEFIE
  *            @arg @ref LL_I3C_IER_GRPIE
  */
__STATIC_INLINE void LL_I3C_EnableIT(I3C_TypeDef *p_i3c, uint32_t mask)
{
  STM32_SET_BIT(p_i3c->IER, mask);
}

/**
  * @brief  Disable the specified I3C interrupts.
  * @rmtoll
  *  IER  CFNFIE     LL_I3C_DisableIT  \n
  *  IER  SFNEIE     LL_I3C_DisableIT  \n
  *  IER  TXFNFIE    LL_I3C_DisableIT  \n
  *  IER  RXFNEIE    LL_I3C_DisableIT  \n
  *  IER  FCIE       LL_I3C_DisableIT  \n
  *  IER  RXTGTENDIE LL_I3C_DisableIT  \n
  *  IER  ERRIE      LL_I3C_DisableIT  \n
  *  IER  IBIIE      LL_I3C_DisableIT  \n
  *  IER  IBIENDIE   LL_I3C_DisableIT  \n
  *  IER  CRIE       LL_I3C_DisableIT  \n
  *  IER  CRUPDIE    LL_I3C_DisableIT  \n
  *  IER  HJIE       LL_I3C_DisableIT  \n
  *  IER  WKPIE      LL_I3C_DisableIT  \n
  *  IER  GETIE      LL_I3C_DisableIT  \n
  *  IER  STAIE      LL_I3C_DisableIT  \n
  *  IER  DAUPDIE    LL_I3C_DisableIT  \n
  *  IER  MWLUPDIE   LL_I3C_DisableIT  \n
  *  IER  MRLUPDIE   LL_I3C_DisableIT  \n
  *  IER  RSTIE      LL_I3C_DisableIT  \n
  *  IER  ASUPDIE    LL_I3C_DisableIT  \n
  *  IER  INTUPDIE   LL_I3C_DisableIT  \n
  *  IER  DEFIE      LL_I3C_DisableIT  \n
  *  IER  GRPIE      LL_I3C_DisableIT
  * @param  p_i3c I3C instance.
  * @param  mask interrupt sources to disable.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I3C_IER_CFNFIE
  *            @arg @ref LL_I3C_IER_SFNEIE
  *            @arg @ref LL_I3C_IER_TXFNFIE
  *            @arg @ref LL_I3C_IER_RXFNEIE
  *            @arg @ref LL_I3C_IER_FCIE
  *            @arg @ref LL_I3C_IER_RXTGTENDIE
  *            @arg @ref LL_I3C_IER_ERRIE
  *            @arg @ref LL_I3C_IER_IBIIE
  *            @arg @ref LL_I3C_IER_IBIENDIE
  *            @arg @ref LL_I3C_IER_CRIE
  *            @arg @ref LL_I3C_IER_CRUPDIE
  *            @arg @ref LL_I3C_IER_HJIE
  *            @arg @ref LL_I3C_IER_WKPIE
  *            @arg @ref LL_I3C_IER_GETIE
  *            @arg @ref LL_I3C_IER_STAIE
  *            @arg @ref LL_I3C_IER_DAUPDIE
  *            @arg @ref LL_I3C_IER_MWLUPDIE
  *            @arg @ref LL_I3C_IER_MRLUPDIE
  *            @arg @ref LL_I3C_IER_RSTIE
  *            @arg @ref LL_I3C_IER_ASUPDIE
  *            @arg @ref LL_I3C_IER_INTUPDIE
  *            @arg @ref LL_I3C_IER_DEFIE
  *            @arg @ref LL_I3C_IER_GRPIE
  */
__STATIC_INLINE void LL_I3C_DisableIT(I3C_TypeDef *p_i3c, uint32_t mask)
{
  STM32_CLEAR_BIT(p_i3c->IER, mask);
}

/**
  * @brief  Check whether the specified I3C interrupts sources are enabled or not.
  * @rmtoll
  *  IER  CFNFIE     LL_I3C_IsEnabledIT  \n
  *  IER  SFNEIE     LL_I3C_IsEnabledIT  \n
  *  IER  TXFNFIE    LL_I3C_IsEnabledIT  \n
  *  IER  RXFNEIE    LL_I3C_IsEnabledIT  \n
  *  IER  FCIE       LL_I3C_IsEnabledIT  \n
  *  IER  RXTGTENDIE LL_I3C_IsEnabledIT  \n
  *  IER  ERRIE      LL_I3C_IsEnabledIT  \n
  *  IER  IBIIE      LL_I3C_IsEnabledIT  \n
  *  IER  IBIENDIE   LL_I3C_IsEnabledIT  \n
  *  IER  CRIE       LL_I3C_IsEnabledIT  \n
  *  IER  CRUPDIE    LL_I3C_IsEnabledIT  \n
  *  IER  HJIE       LL_I3C_IsEnabledIT  \n
  *  IER  WKPIE      LL_I3C_IsEnabledIT  \n
  *  IER  GETIE      LL_I3C_IsEnabledIT  \n
  *  IER  STAIE      LL_I3C_IsEnabledIT  \n
  *  IER  DAUPDIE    LL_I3C_IsEnabledIT  \n
  *  IER  MWLUPDIE   LL_I3C_IsEnabledIT  \n
  *  IER  MRLUPDIE   LL_I3C_IsEnabledIT  \n
  *  IER  RSTIE      LL_I3C_IsEnabledIT  \n
  *  IER  ASUPDIE    LL_I3C_IsEnabledIT  \n
  *  IER  INTUPDIE   LL_I3C_IsEnabledIT  \n
  *  IER  DEFIE      LL_I3C_IsEnabledIT  \n
  *  IER  GRPIE      LL_I3C_IsEnabledIT
  * @param  p_i3c I3C instance.
  * @param  mask Interrupts sources to check.
  *         This parameter can be a combination of the following values:
  *            @arg @ref LL_I3C_IER_CFNFIE
  *            @arg @ref LL_I3C_IER_SFNEIE
  *            @arg @ref LL_I3C_IER_TXFNFIE
  *            @arg @ref LL_I3C_IER_RXFNEIE
  *            @arg @ref LL_I3C_IER_FCIE
  *            @arg @ref LL_I3C_IER_RXTGTENDIE
  *            @arg @ref LL_I3C_IER_ERRIE
  *            @arg @ref LL_I3C_IER_IBIIE
  *            @arg @ref LL_I3C_IER_IBIENDIE
  *            @arg @ref LL_I3C_IER_CRIE
  *            @arg @ref LL_I3C_IER_CRUPDIE
  *            @arg @ref LL_I3C_IER_HJIE
  *            @arg @ref LL_I3C_IER_WKPIE
  *            @arg @ref LL_I3C_IER_GETIE
  *            @arg @ref LL_I3C_IER_STAIE
  *            @arg @ref LL_I3C_IER_DAUPDIE
  *            @arg @ref LL_I3C_IER_MWLUPDIE
  *            @arg @ref LL_I3C_IER_MRLUPDIE
  *            @arg @ref LL_I3C_IER_RSTIE
  *            @arg @ref LL_I3C_IER_ASUPDIE
  *            @arg @ref LL_I3C_IER_INTUPDIE
  *            @arg @ref LL_I3C_IER_DEFIE
  *            @arg @ref LL_I3C_IER_GRPIE
  *            @arg @ref LL_I3C_IER_ALL
  * @retval State of interrupts sources (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT(const I3C_TypeDef *p_i3c, uint32_t mask)
{
  return ((STM32_READ_BIT(p_i3c->IER, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Enable control FIFO not full interrupt.
  * @rmtoll
  *  IER          CFNFIE        LL_I3C_EnableIT_CFNF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_CFNF(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_CFNFIE);
}

/**
  * @brief  Disable control FIFO not full interrupt.
  * @rmtoll
  *  IER          CFNFIE        LL_I3C_DisableIT_CFNF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_CFNF(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_CFNFIE);
}

/**
  * @brief  Check if control FIFO not full interrupt is enabled or disabled.
  * @rmtoll
  *  IER          CFNFIE        LL_I3C_IsEnabledIT_CFNF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_CFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_CFNFIE) == (I3C_IER_CFNFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable status FIFO not empty interrupt.
  * @rmtoll
  *  IER          SFNEIE        LL_I3C_EnableIT_SFNE
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_SFNE(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_SFNEIE);
}

/**
  * @brief  Disable status FIFO not empty interrupt.
  * @rmtoll
  *  IER          SFNEIE        LL_I3C_DisableIT_SFNE
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_SFNE(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_SFNEIE);
}

/**
  * @brief  Check if status FIFO not empty interrupt is enabled or disabled.
  * @rmtoll
  *  IER          SFNEIE        LL_I3C_IsEnabledIT_SFNE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_SFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_SFNEIE) == (I3C_IER_SFNEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable transmit FIFO not full interrupt.
  * @rmtoll
  *  IER          TXFNFIE       LL_I3C_EnableIT_TXFNF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_TXFNF(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_TXFNFIE);
}

/**
  * @brief  Disable transmit FIFO not full interrupt.
  * @rmtoll
  *  IER          TXFNFIE       LL_I3C_DisableIT_TXFNF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_TXFNF(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_TXFNFIE);
}

/**
  * @brief  Check if transmit FIFO not full interrupt is enabled or disabled.
  * @rmtoll
  *  IER          TXFNFIE       LL_I3C_IsEnabledIT_TXFNF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_TXFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_TXFNFIE) == (I3C_IER_TXFNFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable receive FIFO not empty interrupt.
  * @rmtoll
  *  IER          RXFNEIE       LL_I3C_EnableIT_RXFNE
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_RXFNE(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_RXFNEIE);
}

/**
  * @brief  Disable receive FIFO not empty interrupt.
  * @rmtoll
  *  IER          RXFNEIE       LL_I3C_DisableIT_RXFNE
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_RXFNE(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_RXFNEIE);
}

/**
  * @brief  Check if receive FIFO not empty interrupt is enabled or disabled.
  * @rmtoll
  *  IER          RXFNEIE       LL_I3C_IsEnabledIT_RXFNE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_RXFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_RXFNEIE) == (I3C_IER_RXFNEIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Frame Complete interrupt.
  * @rmtoll
  *  IER          FCIE          LL_I3C_EnableIT_FC
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_FC(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_FCIE);
}

/**
  * @brief  Disable Frame Complete interrupt.
  * @rmtoll
  *  IER          FCIE          LL_I3C_DisableIT_FC
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_FC(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_FCIE);
}

/**
  * @brief  Check if Frame Complete interrupt is enabled or disabled.
  * @rmtoll
  *  IER          FCIE          LL_I3C_IsEnabledIT_FC
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_FC(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_FCIE) == (I3C_IER_FCIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable reception target End interrupt.
  * @rmtoll
  *  IER          RXTGTENDIE    LL_I3C_EnableIT_RXTGTEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_RXTGTEND(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_RXTGTENDIE);
}

/**
  * @brief  Disable reception target End interrupt.
  * @rmtoll
  *  IER          RXTGTENDIE    LL_I3C_DisableIT_RXTGTEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_RXTGTEND(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_RXTGTENDIE);
}

/**
  * @brief  Check if reception target End interrupt is enabled or disabled.
  * @rmtoll
  *  IER          RXTGTENDIE    LL_I3C_IsEnabledIT_RXTGTEND
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_RXTGTEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_RXTGTENDIE) == (I3C_IER_RXTGTENDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable error interrupt.
  * @rmtoll
  *  IER          ERRIE         LL_I3C_EnableIT_ERR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_ERR(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_ERRIE);
}

/**
  * @brief  Disable error interrupt.
  * @rmtoll
  *  IER          ERRIE         LL_I3C_DisableIT_ERR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_ERR(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_ERRIE);
}

/**
  * @brief  Check if error interrupt is enabled or disabled.
  * @rmtoll
  *  IER          ERRIE         LL_I3C_IsEnabledIT_ERR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_ERR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_ERRIE) == (I3C_IER_ERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable IBI interrupt.
  * @rmtoll
  *  IER          IBIIE         LL_I3C_EnableIT_IBI
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_IBI(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_IBIIE);
}

/**
  * @brief  Disable IBI interrupt.
  * @rmtoll
  *  IER          IBIIE         LL_I3C_DisableIT_IBI
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_IBI(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_IBIIE);
}

/**
  * @brief  Check if IBI interrupt is enabled or disabled.
  * @rmtoll
  *  IER          IBIIE         LL_I3C_IsEnabledIT_IBI
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_IBI(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_IBIIE) == (I3C_IER_IBIIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable IBI End interrupt.
  * @rmtoll
  *  IER          IBIENDIE      LL_I3C_EnableIT_IBIEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_IBIEND(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_IBIENDIE);
}

/**
  * @brief  Disable IBI End interrupt.
  * @rmtoll
  *  IER          IBIENDIE      LL_I3C_DisableIT_IBIEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_IBIEND(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_IBIENDIE);
}

/**
  * @brief  Check if IBI End interrupt is enabled or disabled.
  * @rmtoll
  *  IER          IBIENDIE      LL_I3C_IsEnabledIT_IBIEND
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_IBIEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_IBIENDIE) == (I3C_IER_IBIENDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable controller-role interrupt.
  * @rmtoll
  *  IER          CRIE          LL_I3C_EnableIT_CR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_CR(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_CRIE);
}

/**
  * @brief  Disable controller-role interrupt.
  * @rmtoll
  *  IER          CRIE          LL_I3C_DisableIT_CR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_CR(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_CRIE);
}

/**
  * @brief  Check if controller-role interrupt is enabled or disabled.
  * @rmtoll
  *  IER          CRIE          LL_I3C_IsEnabledIT_CR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_CR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_CRIE) == (I3C_IER_CRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable controller-role update interrupt.
  * @rmtoll
  *  IER          CRUPDIE       LL_I3C_EnableIT_CRUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_CRUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_CRUPDIE);
}

/**
  * @brief  Disable controller-role update interrupt.
  * @rmtoll
  *  IER          CRUPDIE       LL_I3C_DisableIT_CRUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_CRUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_CRUPDIE);
}

/**
  * @brief  Check if controller-role update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          CRUPDIE       LL_I3C_IsEnabledIT_CRUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_CRUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_CRUPDIE) == (I3C_IER_CRUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable Hot-Join interrupt.
  * @rmtoll
  *  IER          HJIE          LL_I3C_EnableIT_HJ
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_HJ(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_HJIE);
}

/**
  * @brief  Disable Hot-Join interrupt.
  * @rmtoll
  *  IER          HJIE          LL_I3C_DisableIT_HJ
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_HJ(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_HJIE);
}

/**
  * @brief  Check if Hot-Join interrupt is enabled or disabled.
  * @rmtoll
  *  IER          HJIE          LL_I3C_IsEnabledIT_HJ
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_HJ(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_HJIE) == (I3C_IER_HJIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable wake up interrupt.
  * @rmtoll
  *  IER          WKPIE         LL_I3C_EnableIT_WKP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_WKP(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_WKPIE);
}

/**
  * @brief  Disable wake up interrupt.
  * @rmtoll
  *  IER          WKPIE         LL_I3C_DisableIT_WKP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_WKP(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_WKPIE);
}

/**
  * @brief  Check if wakeup is enabled or disabled.
  * @rmtoll
  *  IER          WKPIE         LL_I3C_IsEnabledIT_WKP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_WKP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_WKPIE) == (I3C_IER_WKPIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable get command interrupt.
  * @rmtoll
  *  IER          GETIE         LL_I3C_EnableIT_GET
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_GET(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_GETIE);
}

/**
  * @brief  Disable get command interrupt.
  * @rmtoll
  *  IER          GETIE         LL_I3C_DisableIT_GET
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_GET(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_GETIE);
}

/**
  * @brief  Check if get command is enabled or disabled.
  * @rmtoll
  *  IER          GETIE         LL_I3C_IsEnabledIT_GET
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_GET(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_GETIE) == (I3C_IER_GETIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable get status interrupt.
  * @rmtoll
  *  IER          STAIE         LL_I3C_EnableIT_STA
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_STA(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_STAIE);
}

/**
  * @brief  Disable get status interrupt.
  * @rmtoll
  *  IER          STAIE         LL_I3C_DisableIT_STA
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_STA(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_STAIE);
}

/**
  * @brief  Check if get status interrupt is enabled or disabled.
  * @rmtoll
  *  IER          STAIE         LL_I3C_IsEnabledIT_STA
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_STA(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_STAIE) == (I3C_IER_STAIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable dynamic address update interrupt.
  * @rmtoll
  *  IER          DAUPDIE       LL_I3C_EnableIT_DAUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_DAUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_DAUPDIE);
}

/**
  * @brief  Disable dynamic address update interrupt.
  * @rmtoll
  *  IER          DAUPDIE       LL_I3C_DisableIT_DAUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_DAUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_DAUPDIE);
}

/**
  * @brief  Check if dynamic address update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          DAUPDIE       LL_I3C_IsEnabledIT_DAUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_DAUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_DAUPDIE) == (I3C_IER_DAUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable max write length update interrupt.
  * @rmtoll
  *  IER          MWLUPDIE      LL_I3C_EnableIT_MWLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_MWLUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_MWLUPDIE);
}

/**
  * @brief  Disable max write length update interrupt.
  * @rmtoll
  *  IER          MWLUPDIE      LL_I3C_DisableIT_MWLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_MWLUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_MWLUPDIE);
}

/**
  * @brief  Check if max write length update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          MWLUPDIE      LL_I3C_IsEnabledIT_MWLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_MWLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_MWLUPDIE) == (I3C_IER_MWLUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable max read length update interrupt.
  * @rmtoll
  *  IER          MRLUPDIE      LL_I3C_EnableIT_MRLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_MRLUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_MRLUPDIE);
}

/**
  * @brief  Disable max read length update interrupt.
  * @rmtoll
  *  IER          MRLUPDIE      LL_I3C_DisableIT_MRLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_MRLUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_MRLUPDIE);
}

/**
  * @brief  Check if max read length update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          MRLUPDIE      LL_I3C_IsEnabledIT_MRLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_MRLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_MRLUPDIE) == (I3C_IER_MRLUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable reset interrupt.
  * @rmtoll
  *  IER          RSTIE         LL_I3C_EnableIT_RST
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_RST(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_RSTIE);
}

/**
  * @brief  Disable reset interrupt.
  * @rmtoll
  *  IER          RSTIE         LL_I3C_DisableIT_RST
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_RST(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_RSTIE);
}

/**
  * @brief  Check if reset interrupt is enabled or disabled.
  * @rmtoll
  *  IER          RSTIE         LL_I3C_IsEnabledIT_RST
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_RST(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_RSTIE) == (I3C_IER_RSTIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable activity state update interrupt.
  * @rmtoll
  *  IER          ASUPDIE       LL_I3C_EnableIT_ASUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_ASUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_ASUPDIE);
}

/**
  * @brief  Disable activity state update interrupt.
  * @rmtoll
  *  IER          ASUPDIE       LL_I3C_DisableIT_ASUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_ASUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_ASUPDIE);
}

/**
  * @brief  Check if activity state update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          ASUPDIE       LL_I3C_IsEnabledIT_ASUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_ASUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_ASUPDIE) == (I3C_IER_ASUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable interrupt update interrupt.
  * @rmtoll
  *  IER          INTUPDIE      LL_I3C_EnableIT_INTUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_INTUPD(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_INTUPDIE);
}

/**
  * @brief  Disable interrupt update interrupt.
  * @rmtoll
  *  IER          INTUPDIE      LL_I3C_DisableIT_INTUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_INTUPD(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_INTUPDIE);
}

/**
  * @brief  Check if interrupt update interrupt is enabled or disabled.
  * @rmtoll
  *  IER          INTUPDIE      LL_I3C_IsEnabledIT_INTUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_INTUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_INTUPDIE) == (I3C_IER_INTUPDIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable define list target interrupt.
  * @rmtoll
  *  IER          DEFIE         LL_I3C_EnableIT_DEF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_DEF(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_DEFIE);
}

/**
  * @brief  Disable define list target interrupt.
  * @rmtoll
  *  IER          DEFIE         LL_I3C_DisableIT_DEF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_DEF(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_DEFIE);
}

/**
  * @brief  Check if define list target interrupt is enabled or disabled.
  * @rmtoll
  *  IER          DEFIE         LL_I3C_IsEnabledIT_DEF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_DEF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_DEFIE) == (I3C_IER_DEFIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable define list Group addresses interrupt.
  * @rmtoll
  *  IER          GRPIE         LL_I3C_EnableIT_GRP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_EnableIT_GRP(I3C_TypeDef *p_i3c)
{
  STM32_SET_BIT(p_i3c->IER, I3C_IER_GRPIE);
}

/**
  * @brief  Disable define list Group addresses interrupt.
  * @rmtoll
  *  IER          GRPIE         LL_I3C_DisableIT_GRP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_DisableIT_GRP(I3C_TypeDef *p_i3c)
{
  STM32_CLEAR_BIT(p_i3c->IER, I3C_IER_GRPIE);
}

/**
  * @brief  Check if define list Group addresses interrupt is enabled or disabled.
  * @rmtoll
  *  IER          GRPIE         LL_I3C_IsEnabledIT_GRP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsEnabledIT_GRP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->IER, I3C_IER_GRPIE) == (I3C_IER_GRPIE)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @addtogroup I3C_LL_EF_FLAG_management FLAG_management
  * @{
  */

/**
  * @brief  Clear Frame Complete flag (controller and target mode).
  * @rmtoll
  *  CEVR         CFCF          LL_I3C_ClearFlag_FC
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_FC(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CFCF);
}

/**
  * @brief  Clear reception target end flag (controller mode).
  * @rmtoll
  *  CEVR         CRXTGTENDF    LL_I3C_ClearFlag_RXTGTEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_RXTGTEND(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CRXTGTENDF);
}

/**
  * @brief  Clear error flag (controller and target mode).
  * @rmtoll
  *  CEVR         CERRF         LL_I3C_ClearFlag_ERR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_ERR(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CERRF);
}

/**
  * @brief  Clear IBI flag (controller mode).
  * @rmtoll
  *  CEVR         CIBIF         LL_I3C_ClearFlag_IBI
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_IBI(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CIBIF);
}

/**
  * @brief  Clear IBI end flag (target mode).
  * @rmtoll
  *  CEVR         CIBIENDF      LL_I3C_ClearFlag_IBIEND
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_IBIEND(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CIBIENDF);
}

/**
  * @brief  Clear controller-role request flag (controller mode).
  * @rmtoll
  *  CEVR         CCRF          LL_I3C_ClearFlag_CR
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_CR(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CCRF);
}

/**
  * @brief  Clear controller-role request update flag (target mode).
  * @rmtoll
  *  CEVR         CCRUPDF       LL_I3C_ClearFlag_CRUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_CRUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CCRUPDF);
}

/**
  * @brief  Clear Hot-Join flag (controller mode).
  * @rmtoll
  *  CEVR         CHJF          LL_I3C_ClearFlag_HJ
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_HJ(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CHJF);
}

/**
  * @brief  Clear wakeup flag (target mode).
  * @rmtoll
  *  CEVR         CWKPF         LL_I3C_ClearFlag_WKP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_WKP(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CWKPF);
}

/**
  * @brief  Clear get flag (target mode).
  * @rmtoll
  *  CEVR         CGETF         LL_I3C_ClearFlag_GET
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_GET(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CGETF);
}

/**
  * @brief  Clear get status flag (target mode).
  * @rmtoll
  *  CEVR         CSTAF         LL_I3C_ClearFlag_STA
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_STA(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CSTAF);
}

/**
  * @brief  Clear dynamic address update flag (target mode).
  * @rmtoll
  *  CEVR         CDAUPDF       LL_I3C_ClearFlag_DAUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_DAUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CDAUPDF);
}

/**
  * @brief  Clear max write length flag (target mode).
  * @rmtoll
  *  CEVR         CMWLUPDF      LL_I3C_ClearFlag_MWLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_MWLUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CMWLUPDF);
}

/**
  * @brief  Clear max read length flag (target mode).
  * @rmtoll
  *  CEVR         CMRLUPDF      LL_I3C_ClearFlag_MRLUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_MRLUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CMRLUPDF);
}

/**
  * @brief  Clear reset flag (target mode).
  * @rmtoll
  *  CEVR         CRSTF         LL_I3C_ClearFlag_RST
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_RST(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CRSTF);
}

/**
  * @brief  Clear active state flag (target mode).
  * @rmtoll
  *  CEVR         CASUPDF       LL_I3C_ClearFlag_ASUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_ASUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CASUPDF);
}

/**
  * @brief  Clear interrupt update flag (target mode).
  * @rmtoll
  *  CEVR         CINTUPDF      LL_I3C_ClearFlag_INTUPD
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_INTUPD(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CINTUPDF);
}

/**
  * @brief  Clear define list targets flag (target mode).
  * @rmtoll
  *  CEVR         CDEFF         LL_I3C_ClearFlag_DEF
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_DEF(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CDEFF);
}

/**
  * @brief  Clear define list Group addresses flag.
  * @rmtoll
  *  CEVR         CGRPF         LL_I3C_ClearFlag_GRP
  * @param  p_i3c I3C Instance.
  */
__STATIC_INLINE void LL_I3C_ClearFlag_GRP(I3C_TypeDef *p_i3c)
{
  STM32_WRITE_REG(p_i3c->CEVR, I3C_CEVR_CGRPF);
}

/**
  * @}
  */

/** @defgroup I3C_LL_EF_MASK_IT_Management Mask_IT_Management
  * @{
  */

/**
  * @brief  Indicates the status of masked interrupt control FIFO not full flag.
  * @rmtoll
  *  MISR         CFNFMIS       LL_I3C_IsActiveMaskFlag_CFNF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_CFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_CFNFMIS) == (I3C_MISR_CFNFMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt status FIFO not empty flag.
  * @rmtoll
  *  MISR         SFNEMIS       LL_I3C_IsActiveMaskFlag_SFNE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_SFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_SFNEMIS) == (I3C_MISR_SFNEMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt transmit FIFO not full flag.
  * @rmtoll
  *  MISR         TXFNFMIS      LL_I3C_IsActiveMaskFlag_TXFNF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_TXFNF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_TXFNFMIS) == (I3C_MISR_TXFNFMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt receive FIFO not empty flag.
  * @rmtoll
  *  MISR         RXFNEMIS      LL_I3C_IsActiveMaskFlag_RXFNE
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_RXFNE(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_RXFNEMIS) == (I3C_MISR_RXFNEMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt Frame Complete flag.
  * @rmtoll
  *  MISR         FCMIS         LL_I3C_IsActiveMaskFlag_FC
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_FC(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_FCMIS) == (I3C_MISR_FCMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt reception target end flag.
  * @rmtoll
  *  MISR         RXTGTENDMIS   LL_I3C_IsActiveMaskFlag_RXTGTEND
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_RXTGTEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_RXTGTENDMIS) == (I3C_MISR_RXTGTENDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt error flag.
  * @rmtoll
  *  MISR         ERRMIS        LL_I3C_IsActiveMaskFlag_ERR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_ERR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_ERRMIS) == (I3C_MISR_ERRMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt IBI flag.
  * @rmtoll
  *  MISR         IBIMIS        LL_I3C_IsActiveMaskFlag_IBI
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_IBI(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_IBIMIS) == (I3C_MISR_IBIMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt IBI end flag.
  * @rmtoll
  *  MISR         IBIENDMIS     LL_I3C_IsActiveMaskFlag_IBIEND
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_IBIEND(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_IBIENDMIS) == (I3C_MISR_IBIENDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt controller-role flag.
  * @rmtoll
  *  MISR         CRMIS         LL_I3C_IsActiveMaskFlag_CR
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_CR(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_CRMIS) == (I3C_MISR_CRMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt controller-role update flag.
  * @rmtoll
  *  MISR         CRUPDMIS      LL_I3C_IsActiveMaskFlag_CRUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_CRUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_CRUPDMIS) == (I3C_MISR_CRUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt Hot-Join flag.
  * @rmtoll
  *  MISR         HJMIS         LL_I3C_IsActiveMaskFlag_HJ
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_HJ(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_HJMIS) == (I3C_MISR_HJMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt wakeup is enabled or disabled.
  * @rmtoll
  *  MISR         WKPMIS        LL_I3C_IsActiveMaskFlag_WKP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_WKP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_WKPMIS) == (I3C_MISR_WKPMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt get command is enabled or disabled.
  * @rmtoll
  *  MISR         GETMIS        LL_I3C_IsActiveMaskFlag_GET
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_GET(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_GETMIS) == (I3C_MISR_GETMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt get status flag.
  * @rmtoll
  *  MISR         STAMIS        LL_I3C_IsActiveMaskFlag_STA
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_STA(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_STAMIS) == (I3C_MISR_STAMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt dynamic address update flag.
  * @rmtoll
  *  MISR         DAUPDMIS      LL_I3C_IsActiveMaskFlag_DAUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_DAUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_DAUPDMIS) == (I3C_MISR_DAUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt max write length update flag.
  * @rmtoll
  *  MISR         MWLUPDMIS     LL_I3C_IsActiveMaskFlag_MWLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_MWLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_MWLUPDMIS) == (I3C_MISR_MWLUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt max read length update flag.
  * @rmtoll
  *  MISR         MRLUPDMIS     LL_I3C_IsActiveMaskFlag_MRLUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_MRLUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_MRLUPDMIS) == (I3C_MISR_MRLUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt reset flag.
  * @rmtoll
  *  MISR         RSTMIS        LL_I3C_IsActiveMaskFlag_RST
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_RST(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_RSTMIS) == (I3C_MISR_RSTMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt activity state update flag.
  * @rmtoll
  *  MISR         ASUPDMIS      LL_I3C_IsActiveMaskFlag_ASUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_ASUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_ASUPDMIS) == (I3C_MISR_ASUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt update flag.
  * @rmtoll
  *  MISR         INTUPDMIS     LL_I3C_IsActiveMaskFlag_INTUPD
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_INTUPD(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_INTUPDMIS) == (I3C_MISR_INTUPDMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt define list target flag.
  * @rmtoll
  *  MISR         DEFMIS        LL_I3C_IsActiveMaskFlag_DEF
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_DEF(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_DEFMIS) == (I3C_MISR_DEFMIS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicates the status of masked interrupt define list Group addresses flag.
  * @rmtoll
  *  MISR         GRPMIS        LL_I3C_IsActiveMaskFlag_GRP
  * @param  p_i3c I3C Instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_I3C_IsActiveMaskFlag_GRP(const I3C_TypeDef *p_i3c)
{
  return ((STM32_READ_BIT(p_i3c->MISR, I3C_MISR_GRPMIS) == (I3C_MISR_GRPMIS)) ? 1UL : 0UL);
}

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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32C5xx_LL_I3C_H */
