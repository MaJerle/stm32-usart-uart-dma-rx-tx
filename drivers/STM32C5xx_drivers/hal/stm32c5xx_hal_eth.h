/**
  ******************************************************************************
  * @file    stm32c5xx_hal_eth.h
  * @brief   Header file of ETH (Ethernet) HAL module.
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
  *********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_HAL_ETH_H
#define STM32C5XX_HAL_ETH_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_exti.h"
#include "stm32c5xx_ll_sbs.h"

/** @addtogroup STM32C5XX_HAL_Driver
  * @{
  */
#if defined(ETH1)

/** @defgroup ETH ETH
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup ETH_Exported_Constants HAL ETH constants
  * @{
  */
/** @defgroup ETH_Peripheral_Global_Params ETH peripheral global parameters
  * @{
  */
#ifndef USE_HAL_ETH_MAX_TX_CH_NB
#define USE_HAL_ETH_MAX_TX_CH_NB         ETH_NB_OF_TX_CHANNEL     /*!< Number of available ETH Tx DMA channels */
#endif /* USE_HAL_ETH_MAX_TX_CH_NB */

#ifndef USE_HAL_ETH_MAX_RX_CH_NB
#define USE_HAL_ETH_MAX_RX_CH_NB         ETH_NB_OF_RX_CHANNEL     /*!< Number of available ETH Rx DMA channels */
#endif /* USE_HAL_ETH_MAX_RX_CH_NB */
/**
  * @}
  */

/** @defgroup ETH_Frame_Settings ETH frame settings
  * @{
  */
#define HAL_ETH_MAX_PACKET_SIZE_BYTE          (1528UL)            /*!< Ethernet Maximum Packet Size in bytes unit
                                                                       (ETH_HEADER + 2*VLAN_TAG + MAX_ETH_PAYLOAD
                                                                       + ETH_CRC) */
#define HAL_ETH_HEADER_SIZE_BYTE              (14UL)              /*!< Ethernet Header Size in bytes unit
                                                                       (6 byte Dest addr, 6 byte Src addr, 2 byte
                                                                       length/type) */
#define HAL_ETH_CRC_SIZE_BYTE                 (4UL)               /*!< Ethernet CRC Size in bytes unit */
#define HAL_ETH_VLAN_TAG_SIZE_BYTE            (4UL)               /*!< VLAN Tag Size in bytes unit (Optional 802.1q) */
#define HAL_ETH_DOUBLE_VLAN_TAG_SIZE_BYTE     (8UL)               /*!< Double VLAN Tag Size in bytes unit (Optional
                                                                       802.1q) */
#define HAL_ETH_MIN_PAYLOAD_SIZE_BYTE         (46UL)              /*!< Minimum Ethernet Payload Size in bytes unit */
#define HAL_ETH_MAX_PAYLOAD_SIZE_BYTE         (1500UL)            /*!< Maximum Ethernet Payload Size in bytes unit */
#define HAL_ETH_JUMBO_FRAME_PAYLOAD_SIZE_BYTE (9000UL)            /*!< Jumbo Frame Payload Size in bytes unit */
/**
  * @}
  */

/** @defgroup ETH_Channel_Identifiers ETH Tx and Rx channel identifiers
  * @{
  */
#define HAL_ETH_TX_CHANNEL_0             0x00000001UL              /*!< ETH Tx Channel 0 id         */
#define HAL_ETH_TX_CHANNEL_ALL           0x00000001UL               /*!< ETH All Tx Channels         */
#define HAL_ETH_RX_CHANNEL_0             0x00010000UL              /*!< ETH Rx Channel 0 id         */
#define HAL_ETH_RX_CHANNEL_ALL           0x00010000UL               /*!< ETH All Rx Channels         */
#define HAL_ETH_CHANNEL_ALL              (HAL_ETH_TX_CHANNEL_ALL | \
                                          HAL_ETH_RX_CHANNEL_ALL)  /*!< ETH All Tx and Rx Channels  */
/**
  * @}
  */

/** @defgroup ETH_Queue_Indexes ETH Tx and Rx queue indexes
  * @{
  */
#define HAL_ETH_TX_Q0                    0x00000000UL              /*!< ETH Tx Queue0 Index */
#define HAL_ETH_RX_Q0                    0x00000000UL              /*!< ETH Rx Queue0 Index */
/**
  * @}
  */

/** @defgroup ETH_Error_Code ETH peripheral error codes
  * @brief Error codes for the Ethernet (ETH) peripheral.
  *
  * These constants define the possible error codes returned by the Ethernet
  * (ETH) peripheral HAL layer. They are typically used as bitmasks, allowing
  * multiple errors conditions to be reported simultaneously.
  *
  * @note These error are synchronously reported through @p p_error_cb Callback.
  * @note Use @ref HAL_ETH_GetLastErrorCodes to retrieve the last error code.
  * @{
  */
#define HAL_ETH_ERROR_NONE               0x00000000UL                 /*!< No error                */
#define HAL_ETH_ERROR_FBE                ETH_DMACSR_FBE               /*!< Fatal Bus Error          */
#define HAL_ETH_ERROR_CDE                ETH_DMACSR_CDE               /*!< Context Descriptor Error */
/* Errors set when a bus error occurs during Rx DMA operation */
#define HAL_ETH_ERROR_FBE_DMA_RX_RD      (0x1UL << ETH_DMACSR_REB_Pos) /*!< Bus Fault Error during read transfer
                                                                                 by Rx DMA */
#define HAL_ETH_ERROR_FBE_DMA_RX_AC      (0x2UL << ETH_DMACSR_REB_Pos) /*!< Bus Fault Error during descriptor
                                                                                 access by Rx DMA */
#define HAL_ETH_ERROR_FBE_DMA_RX_WR      (0x4UL << ETH_DMACSR_REB_Pos) /*!< Bus Fault Error during data transfer
                                                                                 by Rx DMA */
/* Errors set when a bus error occurs during Tx DMA operation */
#define HAL_ETH_ERROR_FBE_DMA_TX_RD      (0x1UL << ETH_DMACSR_TEB_Pos) /*!< Bus Fault Error during read transfer
                                                                                 by TxDMA */
#define HAL_ETH_ERROR_FBE_DMA_TX_AC      (0x2UL << ETH_DMACSR_TEB_Pos) /*!< Bus Fault Error during descriptor
                                                                                 access by Tx DMA */
#define HAL_ETH_ERROR_FBE_DMA_TX_WR      (0x4UL << ETH_DMACSR_TEB_Pos) /*!< Bus Fault Error during data transfer
                                                                                 by Tx DMA */
#define HAL_ETH_ERROR_UNDEFINED          0x80000000UL                 /*!< Undefined or unexpected error */
/**
  * @}
  */

/** @defgroup ETH_MAC_Event_Codes ETH MAC peripheral status events
  * @brief MAC Event codes for the Ethernet (ETH) peripheral.
  *
  * These constants define the possible events codes returned by the Ethernet
  * (ETH) peripheral HAL layer. They are typically used as bitmasks, allowing
  * multiple events conditions to be reported simultaneously.
  *
  * @note These events are synchronously reported through @p p_event_cb Callback.
  * @{
  */
/* ETH MAC Rx Tx Status Events */
#define HAL_ETH_EVENT_MAC_RWT            ETH_MACRXTXSR_RWT         /*!< Receive Watchdog Timeout Event */
#define HAL_ETH_EVENT_MAC_EXCOL          ETH_MACRXTXSR_EXCOL       /*!< Excessive Collisions Event */
#define HAL_ETH_EVENT_MAC_LCOL           ETH_MACRXTXSR_LCOL        /*!< Late Collision Event */
#define HAL_ETH_EVENT_MAC_EXDEF          ETH_MACRXTXSR_EXDEF       /*!< Excessive Deferral Event */
#define HAL_ETH_EVENT_MAC_LCARR          ETH_MACRXTXSR_LCARR       /*!< Loss of Carrier Event */
#define HAL_ETH_EVENT_MAC_NCARR          ETH_MACRXTXSR_NCARR       /*!< No Carrier Event */
#define HAL_ETH_EVENT_MAC_TJT            ETH_MACRXTXSR_TJT         /*!< Transmit Jabber Timeout Event */
/**
  * @}
  */

/** @defgroup ETH_PMT_Event_Codes ETH PMT peripheral status events
  * @brief PMT Event codes for the Ethernet (ETH) peripheral.
  *
  * These constants define the possible events codes returned by the Ethernet
  * (ETH) peripheral HAL layer. They are typically used as bitmasks, allowing
  * multiple events conditions to be reported simultaneously.
  *
  * @note These events are synchronously reported through @p p_pmt_cb Callback.
  * @{
  */
/* ETH PMT Status Events */
#define HAL_ETH_EVENT_PMT_MAGIC_PACKET   ETH_MACPCSR_MGKPRCVD    /*!< Magic Packet Received          */
#define HAL_ETH_EVENT_PMT_RWK_PACKET     ETH_MACPCSR_RWKPRCVD    /*!< Remote wake-up Packet Received */
/**
  * @}
  */

/** @defgroup ETH_LPI_Event_Codes ETH LPI peripheral status events
  * @brief LPI Event codes for the Ethernet (ETH) peripheral.
  *
  * These constants define the possible events codes returned by the Ethernet
  * (ETH) peripheral HAL layer. They are typically used as bitmasks, allowing
  * multiple events conditions to be reported simultaneously.
  *
  * @note These events are synchronously reported through @p p_eee_cb Callback.
  * @{
  */
/* ETH LPI Status Events */
#define HAL_ETH_EVENT_LPI_PLS_DOWN       0UL                     /*!< PHY Link Status is Down            */
#define HAL_ETH_EVENT_LPI_PLS_UP         ETH_MACLCSR_PLS         /*!< PHY Link Status is Up              */
#define HAL_ETH_EVENT_LPI_TX_LPI_ST      ETH_MACLCSR_TLPIST      /*!< Transmit LPI State Active          */
#define HAL_ETH_EVENT_LPI_RX_LPI_ST      ETH_MACLCSR_RLPIST      /*!< Receive LPI State Active           */
#define HAL_ETH_EVENT_LPI_TX_LPI_EN      ETH_MACLCSR_TLPIEN      /*!< Transmit LPI State Entry performed */
#define HAL_ETH_EVENT_LPI_RX_LPI_EN      ETH_MACLCSR_RLPIEN      /*!< Receive LPI State Entry performed  */
#define HAL_ETH_EVENT_LPI_TX_LPI_EX      ETH_MACLCSR_TLPIEX      /*!< Transmit LPI State Exit performed  */
#define HAL_ETH_EVENT_LPI_RX_LPI_EX      ETH_MACLCSR_RLPIEX      /*!< Receive LPI State Exit performed   */
/**
  * @}
  */

/** @defgroup ETH_Channel_Event_Codes ETH channel event codes
  * @brief Event codes for the Ethernet (ETH) channel.
  *
  * These constants define the possible events codes returned by the Ethernet
  * (ETH) channel HAL layer. They are typically used as bitmasks, allowing
  * multiple events conditions to be reported simultaneously.
  *
  * @note These events are reported through registered @p p_ch_event_cb to the user.
  * @{
  */
#define HAL_ETH_CH_EVENT_DMA_RBU         ETH_DMACSR_RBU           /*!< Receive Buffer Unavailable Event (bit[7]),
                                                                        This event indicates that there is no more
                                                                        available resources (memory) to perform the
                                                                        incoming packet reception. The Rx process is
                                                                        suspended. To resume processing Rx packets, the
                                                                        application must provide more memory to the
                                                                        driver to allow resuming Rx process of the next
                                                                        recognized incoming packet.
                                                                        This event is available ONLY for Receive
                                                                        Channels. */
#define HAL_ETH_CH_EVENT_DMA_TBU         ETH_DMACSR_TBU           /*!< Transmit Buffer Unavailable Event (bit[2]),
                                                                        This event indicates that the application owns
                                                                        the next descriptor in the Transmit list, and
                                                                        the DMA cannot acquire it. Transmission is
                                                                        suspended.
                                                                        This event is available ONLY for Transmit
                                                                        Channels. */
#define HAL_ETH_CH_EVENT_DMA_RWT         ETH_DMACSR_RWT           /*!< Receive Watchdog Timeout Event (bit[9]).
                                                                        This error is reported when a packet with
                                                                        length greater than 2,048 bytes (10,240 bytes
                                                                        when Jumbo Packet mode is enabled) is received.
                                                                        This error is reported ONLY for Receive
                                                                        Channels. */
/* ETH MTL Error Events */
#define HAL_ETH_CH_EVENT_MTL_RX_OF       ETH_MTLQICSR_RXOVFIS     /*!< MTL Receive Queue Overflow Interrupt
                                                                        Event (bit[16]).
                                                                        This event indicates that the Receive Queue had
                                                                        an overflow while receiving the packet. If a
                                                                        partial packet is transferred to the
                                                                        application, the overflow channel event
                                                                        HAL_ETH_RX_ERROR_OFL is reported. */
#define HAL_ETH_CH_EVENT_MTL_TX_OF       ETH_MTLQICSR_TXUNFIS     /*!< MTL Transmit Queue Underflow Interrupt
                                                                        Event (bit[0]).
                                                                        This event indicates that the Transmit Queue
                                                                        had an underflow while transmitting the packet.
                                                                        Transmission is suspended and an Underflow
                                                                        Error HAL_ETH_TX_ERROR_UF is reported
                                                                        to the application. */
/**
  * @}
  */

/** @defgroup ETH_Channel_Rx_Errors ETH Rx channel errors
  * @brief Errors for the Ethernet (ETH) Rx channel.
  *
  * These constants define the possible errors returned by the Ethernet
  * (ETH) channel HAL layer. They are typically used as bitmasks, allowing
  * multiple errors conditions to be reported simultaneously.
  *
  * @note These errors are reported through registered @p p_rx_complete_cb to the user.
  * @{
  */
#define HAL_ETH_RX_ERROR_IPH             (1UL <<  3U)              /*!< IP Header Error.
                                                                        When this error is reported, it indicates
                                                                        either of the following:
                                                                        * The 16-bit IPv4 header checksum calculated by
                                                                        the MAC does not match the received checksum
                                                                        bytes.
                                                                        * The IP datagram version is not consistent
                                                                        with the Ethernet Type value.
                                                                        * Ethernet packet does not have the expected
                                                                        number of IP header bytes.
                                                                        This error is valid when IPv6 or IPv4 header
                                                                        is detected. */
#define HAL_ETH_RX_ERROR_IPC             (1UL <<  7U)              /*!< IP Payload Error.
                                                                        When this error is reported, it indicates
                                                                        either of the following:
                                                                        * The 16-bit IP payload checksum (that is, the
                                                                        TCP, UDP, or ICMP checksum) calculated by the
                                                                        MAC does not match the corresponding checksum
                                                                        field in the received segment.
                                                                        * The TCP, UDP, or ICMP segment length does not
                                                                        match the payload length value in the IP Header
                                                                        field.
                                                                        * The TCP, UDP, or ICMP segment length is less
                                                                        than minimum allowed segment length for TCP,
                                                                        UDP, or ICMP.
                                                                        This error is valid when IPv6 or IPv4 header is
                                                                        detected. */
#define HAL_ETH_RX_ERROR_DB              (1UL << 19U)              /*!< Dribble Bit Error.
                                                                        When this error is reported, it indicates that
                                                                        the received packet has a noninteger multiple
                                                                        of bytes (odd nibbles). This bit is valid only
                                                                        in the MII Mode. */
#define HAL_ETH_RX_ERROR_REC             (1UL << 20U)              /*!< Receive Error.
                                                                        When this error is reported, it indicates that
                                                                        the ETH_RX_ER signal is asserted while the
                                                                        ETH_RX_DV signal is asserted during packet
                                                                        reception. This error also includes carrier
                                                                        extension error in the GMII and Half-duplex
                                                                        mode. Error can be of less or no extension, or
                                                                        error (rxd!= 0f) during extension. */
#define HAL_ETH_RX_ERROR_OFL             (1UL << 21U)              /*!< Overflow Error.
                                                                        When this error is reported, it indicates that
                                                                        the received packet is damaged because of
                                                                        buffer overflow in Rx FIFO.
                                                                        This error is reported only when the DMA
                                                                        transfers a partial packet to the application.
                                                                        */
#define HAL_ETH_RX_ERROR_RWT             (1UL << 22U)              /*!< Receive Watchdog Timeout.
                                                                        When this error is reported, it indicates that
                                                                        the Receive Watchdog Timer has expired while
                                                                        receiving the current packet. The current
                                                                        packet is truncated after watchdog timeout. */
#define HAL_ETH_RX_ERROR_GP              (1UL << 23U)              /*!< Giant Packet.
                                                                        When this error is reported, it indicates that
                                                                        the packet length exceeds the specified maximum
                                                                        Ethernet size of 1518, 1522, or 2000 bytes
                                                                        (9018 or 9022 bytes if jumbo packet enable is
                                                                        set).
                                                                        Giant packet indicates only the packet length.
                                                                        It does not cause any packet truncation. */
#define HAL_ETH_RX_ERROR_CRC             (1UL << 24U)              /*!< CRC Error.
                                                                        When this error is reported, it indicates that
                                                                        a Cyclic Redundancy Check (CRC) error occurred
                                                                        on the received packet. */
/**
  * @}
  */

/** @defgroup ETH_Channel_Tx_Errors ETH Tx channel errors
  * @brief Errors for the Ethernet (ETH) Tx channel.
  *
  * These constants define the possible errors returned by the Ethernet
  * (ETH) channel HAL layer. They are typically used as bitmasks, allowing
  * multiple errors conditions to be reported simultaneously.
  *
  * @note These errors are reported through registered @p p_tx_complete_cb to the user.
  * @{
  */
#define HAL_ETH_TX_ERROR_IH              (1UL <<  0U)              /*!< IP Header Error.
                                                                        This error indicates that the Checksum
                                                                        Offload engine detected an IP header error. If
                                                                        COE detects an IP header error, it still
                                                                        inserts an IPv4 header checksum if the Ethernet
                                                                        Type field indicates an IPv4 payload.
                                                                        In Full-duplex mode, when EST/Qbv is enabled
                                                                        and this bit is set, it indicates the frame
                                                                        drop status due to Frame Size error or Schedule
                                                                        Error. */
#define HAL_ETH_TX_ERROR_DB              (1UL <<  1U)              /*!< Deferred Bit.
                                                                        This error indicates that the MAC deferred
                                                                        before transmitting because of presence of
                                                                        carrier. */
#define HAL_ETH_TX_ERROR_UF              (1UL <<  2U)              /*!< Underflow Error.
                                                                        This error indicates that the MAC aborted the
                                                                        packet because the data arrived late from the
                                                                        system memory. */
#define HAL_ETH_TX_ERROR_ED              (1UL <<  3U)              /*!< Excessive Deferral.
                                                                        This error indicates that the transmission
                                                                        ended because of excessive deferral of over
                                                                        24,288 bit times (155,680 bits times in
                                                                        1000-Mbps mode or Jumbo Packet enabled mode).
                                                                        */
#define HAL_ETH_TX_ERROR_EC              (1UL <<  8U)              /*!< Excessive Collision.
                                                                        This error indicates that the transmission was
                                                                        aborted after 16 successive collisions while
                                                                        attempting to transmit the current packet. */
#define HAL_ETH_TX_ERROR_LC              (1UL <<  9U)              /*!< Late Collision.
                                                                        This error indicates that packet transmission
                                                                        was aborted because a collision occurred after
                                                                        the collision window (64 byte times including
                                                                        Preamble in MII mode and 512 byte times
                                                                        including Preamble and Carrier Extension in
                                                                        GMII mode). */
#define HAL_ETH_TX_ERROR_NC              (1UL << 10U)              /*!< No Carrier.
                                                                        This error indicates that the carrier sense
                                                                        signal form the PHY was not asserted during
                                                                        transmission. */
#define HAL_ETH_TX_ERROR_LOC             (1UL << 11U)              /*!< Loss of Carrier.
                                                                        This error indicates that Loss of Carrier
                                                                        occurred during packet transmission. */
#define HAL_ETH_TX_ERROR_PC              (1UL << 12U)              /*!< Payload Checksum Error.
                                                                        This error indicates that the Checksum Offload
                                                                        engine had a failure and did not insert any
                                                                        checksum into the encapsulated TCP, UDP, or
                                                                        ICMP payload. */
#define HAL_ETH_TX_ERROR_JT              (1UL << 14U)              /*!< Jabber Timeout.
                                                                        This error indicates that the MAC transmitter
                                                                        has experienced a jabber timeout. */
#define HAL_ETH_TX_ERROR_DE              (1UL << 23U)              /*!< Descriptor Error.
                                                                        This error indicates that the descriptor
                                                                        content is incorrect. */
/**
  * @}
  */

/** @defgroup ETH_Channel_Rx_Status ETH Rx channel status
  * @brief Status for the Ethernet (ETH) Rx channel.
  *
  * These constants define the possible status returned by the Ethernet
  * (ETH) channel HAL layer. They are typically used as bitmasks, allowing
  * multiple status conditions to be reported simultaneously.
  *
  * @note These status are reported through registered @p p_rx_complete_cb to the user.
  * @{
  */
#define HAL_ETH_RX_STATUS_IPV4           (1UL <<  4U)              /*!< IPv4 header Present.
                                                                      This status indicates that an IPV4 header is
                                                                      detected. */
#define HAL_ETH_RX_STATUS_IPV6           (1UL <<  5U)              /*!< IPv6 header Present.
                                                                      This status indicates that an IPV6 header is
                                                                      detected. */
#define HAL_ETH_RX_STATUS_IPCB           (1UL <<  6U)              /*!< IP Checksum Bypassed.
                                                                      This status indicates that the checksum offload
                                                                      engine is bypassed. */
#define HAL_ETH_RX_STATUS_ARPNR          (1UL << 10U)              /*!< ARP Reply Not Generated.
                                                                      This status indicates that the MAC did not
                                                                      generate the ARP Reply for received ARP Request
                                                                      packet. This error is reported when the MAC is
                                                                      busy transmitting ARP reply to earlier ARP
                                                                      request (only one ARP request is processed at
                                                                      a time). */
#define HAL_ETH_RX_STATUS_TSA            (1UL << 14U)              /*!< Timestamp Available.
                                                                      This status indicates that the Timestamp value is
                                                                      available for the received packet.
                                                                      This is valid only when the last received frame
                                                                      of the packet is completed. */
#define HAL_ETH_RX_STATUS_TD             (1UL << 15U)              /*!< Timestamp Dropped.
                                                                      This status indicates that the timestamp was
                                                                      captured for this packet but got dropped in the
                                                                      MTL Rx FIFO because of overflow. */
#define HAL_ETH_RX_STATUS_VLAN           (1UL << 25U)              /*!< Outer (and Inner) VLAG Tag Present.
                                                                      This status indicates that an Outer VLAG tag is
                                                                      detected. If Double VLAN tag processing and VLAN
                                                                      tag stripping are enabled the Inner VLAN tag is
                                                                      detected too. */
#define HAL_ETH_RX_STATUS_LD             (1UL << 28U)              /*!< Last Descriptor.
                                                                      This status indicates indicates that the buffers
                                                                      to which this descriptor is pointing are the last
                                                                      segment of the packet. */
#define HAL_ETH_RX_STATUS_FD             (1UL << 29U)              /*!< First Descriptor.
                                                                      This status indicates that this descriptor
                                                                      contains the first segment of the packet. */
#define HAL_ETH_RX_STATUS_INVALID        (1UL << 31U)              /*!< Rx Buffer Status.
                                                                      This status status indicates that the buffer
                                                                      which this descriptor is pointing is dirty (no
                                                                      data were received). */
/**
  * @}
  */

/** @defgroup ETH_Channel_Tx_Status ETH Tx channel status
  * @brief Status for the Ethernet (ETH) Tx channel.
  *
  * These constants define the possible status returned by the Ethernet
  * (ETH) channel HAL layer. They are typically used as bitmasks, allowing
  * multiple status conditions to be reported simultaneously.
  *
  * @note These status are reported through registered @p p_tx_complete_cb to the user.
  * @{
  */
#define HAL_ETH_TX_STATUS_TTSS           (1UL << 17U)              /*!< Tx Timestamp Status.
                                                                      This status indicates that a timestamp has been
                                                                      captured for the corresponding transmit packet.
                                                                      */
#define HAL_ETH_TX_STATUS_LD             (1UL << 28U)              /*!< Last Descriptor.
                                                                      This status indicates indicates that the buffers
                                                                      to which this descriptor is pointing are the last
                                                                      segment of the packet. */
#define HAL_ETH_TX_STATUS_FD             (1UL << 29U)              /*!< First Descriptor.
                                                                      This status indicates that this descriptor
                                                                      contains the first segment of the packet. */
#define HAL_ETH_TX_STATUS_INVALID        (1UL << 31U)              /*!< Tx Buffer Status.
                                                                      This status indicates that the buffer which this
                                                                      descriptor is pointing is dirty (not yet
                                                                      transmitted). */
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_Control_Attributes Ethernet Tx Packet Control Attributes
  * @brief Constants to configure controls for Ethernet transmit (Tx) packets.
  * @{
  */
/**
  * @def HAL_ETH_TX_PKT_CTRL_CSUM
  * @brief Enable IP header checksum calculation and insertion.
  *
  * When this control is enabled, the hardware will automatically calculate and insert
  * the IP header checksum for the outgoing packet.
  */
#define HAL_ETH_TX_PKT_CTRL_CSUM         0x00000001UL

/**
  * @def HAL_ETH_TX_PKT_CTRL_SAIC
  * @brief Enable MAC Source Address (SA) Insertion Control.
  *
  * When this control is enabled, the hardware will insert or replace the source MAC address
  * in the outgoing packet as per the configuration.
  */
#define HAL_ETH_TX_PKT_CTRL_SAIC         0x00000002UL

/**
  * @def HAL_ETH_TX_PKT_CTRL_VLANTAG
  * @brief Enable VLAN Tag Insertion or Replacement.
  *
  * When this control is enabled, the hardware will insert or replace the VLAN tag in the
  * outgoing packet according to the specified VLAN configuration.
  */
#define HAL_ETH_TX_PKT_CTRL_VLANTAG      0x00000004UL

/**
  * @def HAL_ETH_TX_PKT_CTRL_INNERVLANTAG
  * @brief Enable Inner VLAN Tag Insertion or Replacement.
  *
  * When this control is enabled, the hardware will insert or replace the inner VLAN tag
  * (for double-tagged or Q-in-Q packets) in the outgoing packet.
  */
#define HAL_ETH_TX_PKT_CTRL_INNERVLANTAG 0x00000008UL

/**
  * @def HAL_ETH_TX_PKT_CTRL_CRCPAD
  * @brief Enable MAC CRC and Padding Insertion.
  *
  * When this control is enabled, the hardware will automatically append the CRC and
  * perform padding to meet the minimum Ethernet frame size requirements.
  */
#define HAL_ETH_TX_PKT_CTRL_CRCPAD       0x00000020UL
/**
  * @}
  */

/** @defgroup ETH_Bus_Burst_Length ETH system bus burst length
  * @{
  */
#define HAL_ETH_BUS_BURST_LEN_4_BEAT     ETH_DMASBMR_BLEN4         /*!< System Bus Burst Length 4 beats   */
#define HAL_ETH_BUS_BURST_LEN_8_BEAT     ETH_DMASBMR_BLEN8         /*!< System Bus Burst Length 8 beats   */
#define HAL_ETH_BUS_BURST_LEN_16_BEAT    ETH_DMASBMR_BLEN16        /*!< System Bus Burst Length 16 beats  */
#define HAL_ETH_BUS_BURST_LEN_32_BEAT    ETH_DMASBMR_BLEN32        /*!< System Bus Burst Length 32 beats  */
#define HAL_ETH_BUS_BURST_LEN_64_BEAT    ETH_DMASBMR_BLEN64        /*!< System Bus Burst Length 64 beats  */
#define HAL_ETH_BUS_BURST_LEN_128_BEAT   ETH_DMASBMR_BLEN128       /*!< System Bus Burst Length 128 beats */
#define HAL_ETH_BUS_BURST_LEN_256_BEAT   ETH_DMASBMR_BLEN256       /*!< System Bus Burst Length 256 beats */

/**
  * @}
  */

/**************************************** PMT Control and Status definitions *****************************************/
/** @defgroup ETH_PMT_Triggers ETH PMT control flags
  * @brief Constants for configuring Power Management Timer (PMT) control flags for Ethernet.
  * @{
  */
/**
  * @def HAL_ETH_PMT_CTRL_FWD_WAKEUP_PKT
  * @brief Enable Remote Wake-up Packet Forwarding.
  *
  * When this control is set, the Ethernet controller will forward remote wake-up packets
  * to the host system.
  *
  * @see HAL_ETH_EnterPowerDownMode
  */
#define HAL_ETH_PMT_CTRL_FWD_WAKEUP_PKT  ETH_MACPCSR_RWKPFE

/**
  * @def HAL_ETH_PMT_CTRL_TRIG_MAGIC_PKT
  * @brief Enable Magic Packet detection in power down mode.
  *
  * When this control is set, the Ethernet controller will detect Magic Packets
  * and trigger a wake-up event while in power down mode.
  *
  * @see HAL_ETH_EnterPowerDownMode
  */
#define HAL_ETH_PMT_CTRL_TRIG_MAGIC_PKT  ETH_MACPCSR_MGKPKTEN

/**
  * @def HAL_ETH_PMT_CTRL_TRIG_RWKUP_PKT
  * @brief Enable Wake Up Packet detection in power down mode.
  *
  * When this control is set, the Ethernet controller will detect remote wake-up packets
  * and trigger a wake-up event while in power down mode.
  *
  * @see HAL_ETH_EnterPowerDownMode
  */
#define HAL_ETH_PMT_CTRL_TRIG_RWKUP_PKT  ETH_MACPCSR_RWKPKTEN

/**
  * @def HAL_ETH_PMT_CTRL_TRIG_GLBL_UCAST
  * @brief Enable Global Unicast packet detection in power down mode.
  *
  * When this control is set, the Ethernet controller will detect global unicast packets
  * and trigger a wake-up event while in power down mode.
  *
  * @see HAL_ETH_EnterPowerDownMode
  */
#define HAL_ETH_PMT_CTRL_TRIG_GLBL_UCAST ETH_MACPCSR_GLBLUCAST

/**
  * @def HAL_ETH_PMT_CTRL_TRIG_ALL
  * @brief Enable detection of Magic, Remote, and Global Unicast wake-up packets in power down mode.
  *
  * This control enables simultaneous detection of Magic Packets, remote wake-up packets,
  * and global unicast packets for wake-up events while in power down mode.
  *
  * @see HAL_ETH_EnterPowerDownMode
  */
#define HAL_ETH_PMT_CTRL_TRIG_ALL        (ETH_MACPCSR_RWKPFE | ETH_MACPCSR_MGKPKTEN | \
                                          ETH_MACPCSR_RWKPKTEN | ETH_MACPCSR_GLBLUCAST)
/**
  * @}
  */

/**************************************** LPI Control and Status definitions *****************************************/
/** @defgroup ETH_LPI_Controls ETH LPI controls
  * @brief Constants for configuring Low Power Idle (LPI) controls for Ethernet.
  * @{
  */
/**
  * @def HAL_ETH_LPI_CTRL_TX_CLK_STOP_ENABLE
  * @brief Enable Tx Clock Stop in LPI mode.
  *
  * When this control is set, the Tx clock can be stopped after entering Tx LPI (Low Power Idle) mode.
  * If the RGMII interface is selected, the Tx clock is still required for transmitting the LPI pattern.
  *
  * @see HAL_ETH_EnterLPIMode
  */
#define HAL_ETH_LPI_CTRL_TX_CLK_STOP_ENABLE                  ETH_MACLCSR_LPITCSE

/**
  * @def HAL_ETH_LPI_CTRL_TX_AUTOMATE_ENABLE
  * @brief Enable Tx LPI Automate.
  *
  * When this control is set, the MAC will enter LPI mode only after all outstanding packets
  * (in the core) and pending packets (in the application interface) have been transmitted.
  *
  * @see HAL_ETH_EnterLPIMode
  */
#define HAL_ETH_LPI_CTRL_TX_AUTOMATE_ENABLE                  ETH_MACLCSR_LPITXA
/**
  * @}
  */

/**************************************** MDIO Command Attributes definitions ****************************************/
/** @defgroup ETH_MDIO_Command_Attributes ETH MDIO command attributes
  * @brief Constants for configuring MDIO (Management Data Input/Output) command attributes for Ethernet.
  * @{
  */

/**
  * @def HAL_ETH_MDIO_FEAT_PSE
  * @brief Enable MDIO preamble suppression feature.
  *
  * Selects the preamble length for the MDIO frame.
  * - Feature disabled: 32-bit preamble (standard, recommended for compatibility)
  * - Feature enabled : 1-bit preamble (suppressed, for PHYs that support preamble suppression)
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_FEAT_PSE                                ETH_MACMDIOAR_PSE

/**
  * @def HAL_ETH_MDIO_FEAT_BTB
  * @brief Enable MDIO back-to-back operation mode.
  *
  * Controls whether MDIO operations are performed in back-to-back mode.
  * - Feature disabled: Normal operation (default)
  * - Feature enabled : Back-to-back operation (for consecutive MDIO transactions)
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_FEAT_BTB                                ETH_MACMDIOAR_BTB

/**
  * @def HAL_ETH_MDIO_NTC_1_CYCLE
  * @brief Trim MDIO clock by 1 cycle.
  *
  * Sets the MDIO clock trimming to 1 cycle.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_1_CYCLE                             (1UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_2_CYCLES
  * @brief Trim MDIO clock by 2 cycles.
  *
  * Sets the MDIO clock trimming to 2 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_2_CYCLES                            (2UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_3_CYCLES
  * @brief Trim MDIO clock by 3 cycles.
  *
  * Sets the MDIO clock trimming to 3 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_3_CYCLES                            (3UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_4_CYCLES
  * @brief Trim MDIO clock by 4 cycles.
  *
  * Sets the MDIO clock trimming to 4 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_4_CYCLES                            (4UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_5_CYCLES
  * @brief Trim MDIO clock by 5 cycles.
  *
  * Sets the MDIO clock trimming to 5 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_5_CYCLES                            (5UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_6_CYCLES
  * @brief Trim MDIO clock by 6 cycles.
  *
  * Sets the MDIO clock trimming to 6 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_6_CYCLES                            (6UL << ETH_MACMDIOAR_NTC_Pos)

/**
  * @def HAL_ETH_MDIO_NTC_7_CYCLES
  * @brief Trim MDIO clock by 7 cycles.
  *
  * Sets the MDIO clock trimming to 7 cycles.
  *
  * @see HAL_ETH_MDIO_SetOpAttributes
  */
#define HAL_ETH_MDIO_NTC_7_CYCLES                            (7UL << ETH_MACMDIOAR_NTC_Pos)
/**
  * @}
  */

/**************************************** Remote Wakeup Command List definitions *************************************/
/** @defgroup ETH_RWK_Packet_Filters_Number ETH remote wake-up filter number
  * @brief Constants defining the number of remote wake-up (RWK) packet filters supported by the Ethernet hardware.
  * @{
  */
/**
  * @def HAL_ETH_RWK_FILT_PER_BLOCK
  * @brief Number of remote wake-up (RWK) packet filters per filter block.
  *
  * This constant specifies the maximum number of RWK packet filters that can be configured
  * within a single filter block. Each filter can be programmed to match specific packet patterns
  * for remote wake-up functionality.
  *
  * @see hal_eth_rwk_filter_block_t
  */
#define HAL_ETH_RWK_FILT_PER_BLOCK                           ETH_NB_OF_RWK_FILT_PER_BLOCK

/**
  * @def HAL_ETH_RWK_FILT_BLOCK_NUM
  * @brief Number of available RWK filter blocks.
  *
  * This constant defines the total number of filter blocks supported by the hardware for
  * remote wake-up packet filtering. The total number of RWK filters is the product of
  * @ref HAL_ETH_RWK_FILT_PER_BLOCK and this value.
  *
  * @see hal_eth_rwk_filter_lut_t
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
#define HAL_ETH_RWK_FILT_BLOCK_NUM                           ETH_NB_OF_RWK_FILT_BLOCKS
/**
  * @}
  */

/** @defgroup ETH_RWK_Filter_Commands ETH remote wake-up command list
  * @brief Constants for configuring the 4-bit filter command for remote wakeup filters.
  * @{
  */
/**
  * @def HAL_ETH_RWK_FLT_CMD_ENABLE
  * @brief Enable filter i.
  *
  * Bit 0: When set, enables filter i.
  * If this bit is not set, filter i is disabled and will not participate in remote wakeup packet matching.
  *
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
#define HAL_ETH_RWK_FLT_CMD_ENABLE                           (1UL << 0U)

/**
  * @def HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  * @brief AND logic with previous filter.
  *
  * Bit 1: When set, the result of the current filter is logically ANDed with the result of the previous filter.
  * This allows for filter patterns longer than 32 bytes by splitting the mask among multiple filters.
  * Multiple filters can be chained using this bit to create complex matching logic.
  *
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
#define HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS                     (1UL << 1U)

/**
  * @def HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  * @brief Inverse mode for CRC16 hash function.
  *
  * Bit 2: When set, reverses the logic of the CRC16 hash function signal, causing the filter to reject packets
  * with a matching CRC16 value.
  * This bit, in combination with @ref HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS, allows the creation of filter logic such as
  * "Pattern 1 AND NOT Pattern 2" to reject a subset of remote wakeup packets.
  *
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
#define HAL_ETH_RWK_FLT_CMD_INVERSE_MODE                     (1UL << 2U)

/**
  * @def HAL_ETH_RWK_FLT_CMD_MULTICAST
  * @brief Multicast address type selection.
  *
  * Bit 3: Specifies the address type for the pattern.
  * - When set, the pattern applies only to multicast packets.
  * - When reset, the pattern applies only to unicast packets.
  *
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
#define HAL_ETH_RWK_FLT_CMD_MULTICAST                        (1UL << 3U)
/**
  * @}
  */
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup ETH_Exported_Types HAL ETH types
  * @{
  */
/** @defgroup ETH_Exported_Types_Group1 Enumerations
  * @{
  */
/**************************************  ETH Peripheral Handle Enumerations  *****************************************/
/**
  * @brief HAL ETH instances enumeration definition
  *
  */
typedef enum
{
  HAL_ETH1  = ETH1_BASE        /*!< Instance ETH */
} hal_eth_t;

/********************************  ETH Peripheral and Channel State Enumerations  ************************************/
/**
  * @brief HAL ETH state enumeration definition
  */
typedef enum
{
  HAL_ETH_STATE_RESET               = 0UL,                 /*!< ETH Peripheral is not yet Initialized or De-Initialized
                                                                */
  HAL_ETH_STATE_INIT                = (1UL << 31U),        /*!< ETH Peripheral is initialized but not yet configured
                                                                */
  HAL_ETH_STATE_CONFIGURED          = (1UL << 30U),        /*!< ETH Peripheral initialized and a global config applied
                                                                */
  HAL_ETH_STATE_POWER_DOWN          = (1UL << 28U),        /*!< ETH peripheral is in power down mode
                                                                */
  HAL_ETH_STATE_FAULT               = (1UL << 27U)         /*!< ETH Peripheral encountered an unrecoverable error and
                                                                a recovery sequence is needed */
} hal_eth_state_t;

/**
  * @brief  HAL ETH channel state structures definition
  */
typedef enum
{
  HAL_ETH_CHANNEL_STATE_RESET       = (1UL << 0U),         /*!< Channel is disabled (Reset state) */
  HAL_ETH_CHANNEL_STATE_CONFIGURED  = (1UL << 31U),        /*!< Channel is configured and stopped */
  HAL_ETH_CHANNEL_STATE_ACTIVE      = (1UL << 30U),        /*!< Channel is started and running (Active state) */
  HAL_ETH_CHANNEL_STATE_SUSPENDED   = (1UL << 29U)         /*!< Channel is started and suspended (Idle state) */
} hal_eth_channel_state_t;

/***********************************  ETH request FIFO event enumerations  ****************************************/
/**
  * @brief  ETH FIFO event mode type enum definition
  */
typedef enum
{
  HAL_ETH_FIFO_EVENT_NONE           = 0x00000001UL,        /*!< ETH Interrupt is disabled - Polling mode, no interrupt
                                                                */
  HAL_ETH_FIFO_EVENT_ALWAYS         = 0x00000002UL,        /*!< ETH FIFO event on every executed descriptor */
  HAL_ETH_FIFO_EVENT_CYCLIC         = 0x00000004UL         /*!< ETH Interrupt after every N descriptors.
                                                                This mode reduces interrupt load for high-throughput
                                                                 operation. */
} hal_eth_fifo_event_mode_t;

/************************************  ETH Peripheral Configuration Enumerations  ************************************/
/**
  * @brief  HAL ETH Media Interfaces enum definition
  */
typedef enum
{
  HAL_ETH_MEDIA_IF_MII              = 0x00UL,              /*!< Media Independent Interface               */
  HAL_ETH_MEDIA_IF_RMII             = 0x02UL,              /*!< Reduced Media Independent Interface       */
} hal_eth_media_interface_t;

/************************************  ETH Transmit Packet Controls Enumerations  ************************************/
/**
  * @brief HAL ETH Tx Packet Source Address Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_SRC_ADDR_CONTROL_DISABLE = 0x00000000UL,   /*!< SA Insertion Control: Do not include the source
                                                                 address. */
  HAL_ETH_TX_PKT_SRC_ADDR_INSERT    = 0x00800000UL,         /*!< SA Insertion Control: Include or insert the source
                                                                address. */
  HAL_ETH_TX_PKT_SRC_ADDR_REPLACE   = 0x01000000UL          /*!< SA Insertion Control: Replace the source address */
} hal_eth_tx_pkt_src_addr_ctrl_t;

/**
  * @brief HAL ETH Tx Packet CRC Pad Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_CRC_PAD_DISABLE    = 0x08000000UL,        /*!< CRC Pad Control: Disable CRC Insertion */
  HAL_ETH_TX_PKT_CRC_PAD_INSERT     = 0x00000000UL,        /*!< CRC Pad Control: CRC and Pad Insertion */
  HAL_ETH_TX_PKT_CRC_INSERT         = 0x04000000UL,        /*!< CRC Pad Control: CRC Insertion (Disable Pad Insertion)
                                                                */
  HAL_ETH_TX_PKT_CRC_REPLACE        = 0x0C000000UL         /*!< CRC Pad Control: CRC Replacement */
} hal_eth_tx_pkt_crc_pad_ctrl_t;

/**
  * @brief HAL ETH Tx Packet Checksum Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_CSUM_DISABLE               = 0x00000000UL, /*!< Do Nothing: Checksum Engine is disabled */
  HAL_ETH_TX_PKT_CSUM_HEADER_INSERT         = 0x00010000UL, /*!< Only IP header checksum calculation and insertion are
                                                                enabled. */
  HAL_ETH_TX_PKT_CSUM_PAYLOAD_INSERT        = 0x00020000UL, /*!< IP header checksum and payload checksum calculation
                                                                and insertion are
                                                                enabled, but pseudo header checksum is not calculated
                                                                in hardware */
  HAL_ETH_TX_PKT_CSUM_PAYLOAD_HEADER_INSERT = 0x00030000UL  /*!< IP Header checksum and payload checksum calculation and
                                                                insertion are enabled, and pseudo header checksum is
                                                                calculated in hardware. */
} hal_eth_tx_pkt_csum_ctrl_t;

/**
  * @brief HAL ETH Tx Packet VLAN Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_VLAN_DISABLE       = 0x00000000UL,        /*!< Do not add a VLAN tag. */
  HAL_ETH_TX_PKT_VLAN_REMOVE        = 0x00004000UL,        /*!< Remove the VLAN tag from the packets before
                                                                transmission. */
  HAL_ETH_TX_PKT_VLAN_INSERT        = 0x00008000UL,        /*!< Insert a VLAN tag. */
  HAL_ETH_TX_PKT_VLAN_REPLACE       = 0x0000C000UL         /*!< Replace the VLAN tag. */
} hal_eth_tx_pkt_vlan_ctrl_t;

/**
  * @brief HAL ETH Tx Packet Inner VLAN Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_INNER_VLAN_DISABLE = 0x00000000UL,        /*!< Do not add the inner VLAN tag. */
  HAL_ETH_TX_PKT_INNER_VLAN_REMOVE  = 0x00040000UL,        /*!< Remove the inner VLAN tag from the packets before
                                                                transmission. */
  HAL_ETH_TX_PKT_INNER_VLAN_INSERT  = 0x00080000UL,        /*!< Insert the inner VLAN tag. */
  HAL_ETH_TX_PKT_INNER_VLAN_REPLACE = 0x000C0000UL         /*!< Replace the inner VLAN tag. */
} hal_eth_tx_pkt_inner_vlan_ctrl_t;

/**
  * @brief HAL ETH Tx Packet Notification Controls
  */
typedef enum
{
  HAL_ETH_TX_PKT_NOTIFY_DISABLE     = 0UL,                 /*!< Do not notify application when the packet transmit
                                                                request is completed */
  HAL_ETH_TX_PKT_NOTIFY_ENABLE      = 1UL                  /*!< Notify application when the packet transmit request is
                                                                completed  */
} hal_eth_tx_pkt_notify_ctrl_t;

/**************************************  ETH MAC Configuration Enumerations  *****************************************/
/**
  * @brief HAL ETH Speed
  */
typedef enum
{
  HAL_ETH_MAC_SPEED_10M             = 0x00000000UL,       /*!< ETH MAC Speed 10M   */
  HAL_ETH_MAC_SPEED_100M            = ETH_MACCR_FES,      /*!< ETH MAC Speed 100M  */
} hal_eth_mac_speed_t;

/**
  * @brief HAL ETH Duplex Mode
  */
typedef enum
{
  HAL_ETH_MAC_FULL_DUPLEX_MODE      = ETH_MACCR_DM,        /*!< ETH MAC Full Duplex Mode */
  HAL_ETH_MAC_HALF_DUPLEX_MODE      = 0x00000000UL         /*!< ETH MAC Half Duplex Mode */
} hal_eth_mac_duplex_mode_t;

/**
  * @brief  HAL ETH Loopback mode Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_LOOPBACK_DISABLE      = 0UL,                 /*!< ETH MAC Loopback mode Disable */
  HAL_ETH_MAC_LOOPBACK_ENABLE       = ETH_MACCR_LM         /*!< ETH MAC Loopback mode Enable  */
} hal_eth_mac_loopback_ctrl_t;

/**
  * @brief HAL ETH Source Address Control (Disable, Insertion or Replacement)
  */
typedef enum
{
  HAL_ETH_MAC_SA_DISABLE        = 0x00000000UL,       /*!< Disable SA insertion or replacement in the transmitted
                                                                packets */
  HAL_ETH_MAC_SA_MAC0_INS       = 0x20000000UL,       /*!< Inserts the content of the MAC Address 0 registers in
                                                                the SA field of all transmitted packets */
  HAL_ETH_MAC_SA_MAC1_INS       = 0x60000000UL,       /*!< Inserts the content of the MAC Address 1 registers in
                                                                the SA field of all transmitted packets */
  HAL_ETH_MAC_SA_MAC0_REP       = 0x30000000UL,      /*!< Replace the content of the MAC Address 0 registers in
                                                                the SA field of all transmitted packets */
  HAL_ETH_MAC_SA_MAC1_REP       = ETH_MACCR_SARC     /*!< Replace the content of the MAC Address 1 registers in
                                                                the SA field of all transmitted packets */
} hal_eth_mac_src_addr_ctrl_t;

/**
  * @brief HAL ETH Inter Packet Gap
  */
typedef enum
{
  HAL_ETH_MAC_INTER_PKT_GAP_96_BIT  = 0x00000000UL,        /*!< ETH MAC Inter Packet Gap 96 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_88_BIT  = 0x01000000UL,        /*!< ETH MAC Inter Packet Gap 88 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_80_BIT  = 0x02000000UL,        /*!< ETH MAC Inter Packet Gap 80 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_72_BIT  = 0x03000000UL,        /*!< ETH MAC Inter Packet Gap 72 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_64_BIT  = 0x04000000UL,        /*!< ETH MAC Inter Packet Gap 64 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_56_BIT  = 0x05000000UL,        /*!< ETH MAC Inter Packet Gap 56 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_48_BIT  = 0x06000000UL,        /*!< ETH MAC Inter Packet Gap 48 Bit  */
  HAL_ETH_MAC_INTER_PKT_GAP_40_BIT  = ETH_MACCR_IPG        /*!< ETH MAC Inter Packet Gap 40 Bit  */
} hal_eth_mac_inter_pkt_gap_t;

/**
  * @brief HAL ETH MAC Back Off Limit
  */
typedef enum
{
  HAL_ETH_MAC_BACK_OFF_LIMIT_10     = 0x00000000UL,        /*!< ETH MAC Back Off Limit 10 */
  HAL_ETH_MAC_BACK_OFF_LIMIT_8      = 0x00000020UL,        /*!< ETH MAC Back Off Limit 8 */
  HAL_ETH_MAC_BACK_OFF_LIMIT_4      = 0x00000040UL,        /*!< ETH MAC Back Off Limit 4 */
  HAL_ETH_MAC_BACK_OFF_LIMIT_1      = ETH_MACCR_BL         /*!< ETH MAC Back Off Limit 1 */
} hal_eth_mac_back_off_limit_t;

/**
  * @brief HAL ETH MAC Preamble Length
  */
typedef enum
{
  HAL_ETH_MAC_PREAMBLE_LENGTH_7     = 0x00000000UL,        /*!< ETH MAC Preamble Length 7 */
  HAL_ETH_MAC_PREAMBLE_LENGTH_5     = 0x00000004UL,        /*!< ETH MAC Preamble Length 5 */
  HAL_ETH_MAC_PREAMBLE_LENGTH_3     = 0x00000008UL         /*!< ETH MAC Preamble Length 3 */
} hal_eth_mac_preeamble_length_t;

/**
  * @brief HAL ETH MAC Watchdog Timeout
  */
typedef enum
{
  HAL_ETH_MAC_RX_WDT_2KB            = 0x00000000UL,        /*!< ETH MAC Watchdog Timeout 2KB  */
  HAL_ETH_MAC_RX_WDT_3KB            = 0x00000001UL,        /*!< ETH MAC Watchdog Timeout 3KB  */
  HAL_ETH_MAC_RX_WDT_4KB            = 0x00000002UL,        /*!< ETH MAC Watchdog Timeout 4KB  */
  HAL_ETH_MAC_RX_WDT_5KB            = 0x00000003UL,        /*!< ETH MAC Watchdog Timeout 5KB  */
  HAL_ETH_MAC_RX_WDT_6KB            = 0x00000004UL,        /*!< ETH MAC Watchdog Timeout 6KB  */
  HAL_ETH_MAC_RX_WDT_7KB            = 0x00000005UL,        /*!< ETH MAC Watchdog Timeout 7KB  */
  HAL_ETH_MAC_RX_WDT_8KB            = 0x00000006UL,        /*!< ETH MAC Watchdog Timeout 8KB  */
  HAL_ETH_MAC_RX_WDT_9KB            = 0x00000007UL,        /*!< ETH MAC Watchdog Timeout 9KB  */
  HAL_ETH_MAC_RX_WDT_10KB           = 0x00000008UL,        /*!< ETH MAC Watchdog Timeout 10KB */
  HAL_ETH_MAC_RX_WDT_11KB           = 0x00000009UL,        /*!< ETH MAC Watchdog Timeout 11KB */
  HAL_ETH_MAC_RX_WDT_12KB           = 0x0000000AUL,        /*!< ETH MAC Watchdog Timeout 12KB */
  HAL_ETH_MAC_RX_WDT_13KB           = 0x0000000BUL,        /*!< ETH MAC Watchdog Timeout 13KB */
  HAL_ETH_MAC_RX_WDT_14KB           = 0x0000000CUL,        /*!< ETH MAC Watchdog Timeout 14KB */
  HAL_ETH_MAC_RX_WDT_15KB           = 0x0000000DUL,        /*!< ETH MAC Watchdog Timeout 15KB */
  HAL_ETH_MAC_RX_WDT_16KB           = 0x0000000EUL         /*!< ETH MAC Watchdog Timeout 16KB */
} hal_eth_mac_rx_wd_timeout_t;

/**
  * @brief  HAL ETH MAC Giant Packet Size Limit Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_GPKT_SZ_LIMIT_DISABLE = 0UL,                 /*!< ETH MAC Giant Packet Size Limit Disable */
  HAL_ETH_MAC_GPKT_SZ_LIMIT_ENABLE  = ETH_MACCR_GPSLCE     /*!< ETH MAC Giant Packet Size Limit Enable  */
} hal_eth_mac_gpkt_sz_limit_ctrl_t;

/**
  * @brief  HAL ETH MAC IEEE 802.3as Support for 2K length Packets Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_2K_PKT_LEN_DISABLE    = 0UL,                 /*!< ETH MAC IEEE 802.3as Support for 2K length Packets
                                                                Disable */
  HAL_ETH_MAC_2K_PKT_LEN_ENABLE     = ETH_MACCR_S2KP       /*!< ETH MAC IEEE 802.3as Support for 2K length Packets
                                                                Enable  */
} hal_eth_mac_2k_pkt_len_ctrl_t;

/**
  * @brief  HAL ETH MAC CRC stripping for Type Packets Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_CRC_STRIP_PKT_DISABLE = 0UL,                 /*!< ETH MAC CRC stripping for Type Packets Disable */
  HAL_ETH_MAC_CRC_STRIP_PKT_ENABLE  = ETH_MACCR_CST        /*!< ETH MAC CRC stripping for Type Packets Enable  */
} hal_eth_mac_crc_strip_pkt_ctrl_t;

/**
  * @brief  HAL ETH MAC Automatic MAC Pad/CRC Stripping Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_AUTO_PAD_CRC_S_DISABLE = 0UL,                /*!< ETH MAC Automatic Pad/CRC Stripping Disable */
  HAL_ETH_MAC_AUTO_PAD_CRC_S_ENABLE  = ETH_MACCR_ACS       /*!< ETH MAC Automatic Pad/CRC Stripping Enable  */
} hal_eth_mac_auto_pad_crc_s_ctrl_t;

/**
  * @brief  HAL ETH MAC Watchdog Timer on Rx Path Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_RX_WD_TIM_DISABLE     = ETH_MACCR_WD,        /*!< ETH MAC Watchdog timer on Rx path Disable */
  HAL_ETH_MAC_RX_WD_TIM_ENABLE      = 0UL                  /*!< ETH MAC Watchdog timer on Rx path Enable  */
} hal_eth_mac_rx_wd_tim_ctrl_t;

/**
  * @brief  HAL ETH MAC Jabber Timer on Tx Path Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_TX_JABBER_TIM_DISABLE = ETH_MACCR_JD,        /*!< ETH MAC Jabber timer on Tx path Disable */
  HAL_ETH_MAC_TX_JABBER_TIM_ENABLE  = 0UL                  /*!< ETH MAC Jabber timer on Tx path Enable  */
} hal_eth_mac_tx_jabber_tim_ctrl_t;

/**
  * @brief  HAL ETH MAC Jumbo Packet on Rx Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_RX_JUMBO_PKT_DISABLE  = 0UL,                 /*!< ETH MAC Jumbo Packet on Rx path Disable */
  HAL_ETH_MAC_RX_JUMBO_PKT_ENABLE   = ETH_MACCR_JE         /*!< ETH MAC Jumbo Packet on Rx path Enable  */
} hal_eth_mac_rx_jumbo_pkt_ctrl_t;

/**
  * @brief  HAL ETH MAC Checksum Packet on Rx path Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_RX_CSUM_PKT_DISABLE   = 0UL,                 /*!< ETH Checksum Packet on Rx path Disable */
  HAL_ETH_MAC_RX_CSUM_PKT_ENABLE    = ETH_MACCR_IPC        /*!< ETH Checksum Packet on Rx path Enable  */
} hal_eth_mac_rx_csum_pkt_ctrl_t;

/**
  * @brief  HAL ETH MAC Receive Own on Rx path Control enumeration definition
  */
typedef enum
{
  HAL_ETH_MAC_RX_RECEIVE_OWN_DISABLE = ETH_MACCR_DO,        /*!< ETH MAC Receive Own on Rx path Disable */
  HAL_ETH_MAC_RX_RECEIVE_OWN_ENABLE  = 0UL                  /*!< ETH MAC Receive Own on Rx path Enable  */
} hal_eth_mac_rx_receive_own_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the Carrier Sense Before Transmission in Full Duplex mode
  */
typedef enum
{
  HAL_ETH_MAC_CS_BEFORE_TR_DISABLE  = 0UL,                 /*!< ETH MAC Carrier Sense Before Transmission in
                                                                Full-duplex mode Disable */
  HAL_ETH_MAC_CS_BEFORE_TR_ENABLE   = ETH_MACCR_ECRSFD     /*!< ETH MAC Carrier Sense Before Transmission in
                                                                Full-duplex mode Enable  */
} hal_eth_mac_cs_before_tr_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the Carrier Sense During Transmission in the Half Duplex mode
  */
typedef enum
{
  HAL_ETH_MAC_CS_DURING_TR_DISABLE  = ETH_MACCR_DCRS,      /*!< ETH MAC Disable Carrier Sense During Transmission
                                                                errors. */
  HAL_ETH_MAC_CS_DURING_TR_ENABLE   = 0UL                  /*!< ETH MAC Enable Carrier Sense During Transmission
                                                                errors. */
} hal_eth_mac_cs_during_tr_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the MAC retry transmission, when a collision occurs in Half Duplex mode
  */
typedef enum
{
  HAL_ETH_MAC_RETRY_TR_DISABLE      = ETH_MACCR_DR,        /*!< ETH MAC Disable Retry Transmission */
  HAL_ETH_MAC_RETRY_TR_ENABLE       = 0UL                  /*!< ETH MAC Enable Retry Transmission  */
} hal_eth_mac_retry_tr_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the deferral check function in Half Duplex mode
  */
typedef enum
{
  HAL_ETH_MAC_DEFERRAL_CHECK_DISABLE = 0UL,                /*!< ETH MAC Disable Deferral Check  */
  HAL_ETH_MAC_DEFERRAL_CHECK_ENABLE  = ETH_MACCR_DC        /*!< ETH MAC Enable Deferral Check  */
} hal_eth_mac_deferral_check_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the Detection of Slow Protocol Packets with unicast address
  */
typedef enum
{
  HAL_ETH_MAC_UC_SLOW_PROTO_DISABLE = 0UL,                 /*!< ETH MAC Disable Unicast Slow Protocol Packet Detect */
  HAL_ETH_MAC_UC_SLOW_PROTO_ENABLE  = ETH_MACECR_USP       /*!< ETH MAC Enable Unicast Slow Protocol Packet Detect  */
} hal_eth_mac_uc_slow_proto_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the Slow Protocol Detection
  */
typedef enum
{
  HAL_ETH_MAC_SLOW_PROTO_DISABLE    = 0UL,                 /*!< ETH MAC Disable Slow Protocol Detection  */
  HAL_ETH_MAC_SLOW_PROTO_ENABLE     = ETH_MACECR_SPEN      /*!< ETH MAC Enable Slow Protocol Detection   */
} hal_eth_mac_slow_proto_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the CRC Checking for Received Packets
  */
typedef enum
{
  HAL_ETH_MAC_RX_CRC_PKT_CHK_DISABLE = ETH_MACECR_DCRCC,   /*!< ETH MAC Disable CRC Checking for Received Packets */
  HAL_ETH_MAC_RX_CRC_PKT_CHK_ENABLE  = 0UL                 /*!< ETH MAC Enable CRC Checking for Received Packets  */
} hal_eth_mac_rx_crc_pkt_chk_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the extended inter packet gap
  */
typedef enum
{
  HAL_ETH_MAC_E_INTER_PKT_GAP_DISABLE = 0UL,               /*!< ETH MAC Disable Extended Inter-Packet Gap */
  HAL_ETH_MAC_E_INTER_PKT_GAP_ENABLE  = ETH_MACECR_EIPGEN  /*!< ETH MAC Enable Extended Inter-Packet Gap  */
} hal_eth_mac_ex_int_pkt_gap_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the Programmable Watchdog
  */
typedef enum
{
  HAL_ETH_MAC_PROG_WD_DISABLE       = 0UL,                 /*!< ETH MAC Disable Programmable Watchdog */
  HAL_ETH_MAC_PROG_WD_ENABLE        = ETH_MACWJBTR_PWE     /*!< ETH MAC Enable Programmable Watchdog  */
} hal_eth_mac_prog_wd_ctrl_t;

/**
  * @brief  HAL ETH MAC Enable or disables the automatic generation of Zero Quanta Pause Control packets
  */
typedef enum
{
  HAL_ETH_MAC_ZERO_Q_PAUSE_DISABLE  = ETH_MACQTXFCR_DZPQ, /*!< ETH MAC Disable Zero-Quanta Pause */
  HAL_ETH_MAC_ZERO_Q_PAUSE_ENABLE   = 0UL                  /*!< ETH MAC Enable Zero-Quanta Pause  */
} hal_eth_mac_zero_q_pause_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the MAC to transmit Pause packets in Full Duplex mode or the MAC back
    pressure operation in Half Duplex mode
  */
typedef enum
{
  HAL_ETH_MAC_TR_FLOW_CTRL_DISABLE  = 0UL,                 /*!< ETH MAC Disable Transmit Flow Control */
  HAL_ETH_MAC_TR_FLOW_CTRL_ENABLE   = ETH_MACQTXFCR_TFE   /*!< ETH MAC Enable Transmit Flow Control  */
} hal_eth_mac_tr_flow_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the MAC to detect Pause packets with unicast address of the station
  */
typedef enum
{
  HAL_ETH_MAC_UC_PAUSE_PKT_DISABLE  = 0UL,                 /*!< ETH MAC Disable Unicast Pause Packet Detect */
  HAL_ETH_MAC_UC_PAUSE_PKT_ENABLE   = ETH_MACRXFCR_UP      /*!< ETH MAC Enable Unicast Pause Packet Detect  */
} hal_eth_mac_uc_pause_pkt_ctrl_t;

/**
  * @brief  HAL ETH MAC Enables or disables the MAC to decodes the received Pause packet and disables its transmitter
    for a specified (Pause) time
  */
typedef enum
{
  HAL_ETH_MAC_RECEIVE_FLOW_DISABLE  = 0UL,                 /*!< ETH MAC Disable Receive Flow Control */
  HAL_ETH_MAC_RECEIVE_FLOW_ENABLE   = ETH_MACRXFCR_RFE     /*!< ETH MAC Enable Receive Flow Control  */
} hal_eth_mac_receive_flow_ctrl_t;


/**
  * @brief HAL ETH MAC Pause Low Threshold
  */
typedef enum
{
  HAL_ETH_MAC_PLT_MINUS_4_SLOT_TIME   = 0x00000000UL,      /*!< ETH MAC Pause Low Threshold Minus 4   */
  HAL_ETH_MAC_PLT_MINUS_28_SLOT_TIME  = 0x00000010UL,      /*!< ETH MAC Pause Low Threshold Minus 28  */
  HAL_ETH_MAC_PLT_MINUS_36_SLOT_TIME  = 0x00000020UL,      /*!< ETH MAC Pause Low Threshold Minus 36  */
  HAL_ETH_MAC_PLT_MINUS_144_SLOT_TIME = 0x00000030UL,      /*!< ETH MAC Pause Low Threshold Minus 144 */
  HAL_ETH_MAC_PLT_MINUS_256_SLOT_TIME = 0x00000040UL,      /*!< ETH MAC Pause Low Threshold Minus 256 */
  HAL_ETH_MAC_PLT_MINUS_512_SLOT_TIME = 0x00000050UL       /*!< ETH MAC Pause Low Threshold Minus 512 */
} hal_eth_mac_pause_low_thr_t;

/**************************************  ETH DMA Configuration Enumerations  *****************************************/

/**
  * @brief  Enables or disables the Peripheral Bus Master interface address aligned
  *         burst transfers on Read and Write channels
  */
typedef enum
{
  HAL_ETH_DMA_ADDR_ALIGN_DISABLE    = 0UL,                 /*!< ETH DMA Disable Address-Aligned Beats  */
  HAL_ETH_DMA_ADDR_ALIGN_ENABLE     = ETH_DMASBMR_AAL      /*!< ETH DMA Enable Address-Aligned Beats   */
} hal_eth_dma_addr_align_ctrl_t;

/**
  * @brief HAL ETH Burst Length Mode
  */
typedef enum
{
  HAL_ETH_DMA_BURST_LEN_FIXED       = ETH_DMASBMR_FB,      /*!< Fixed Burst Length as specified by the BLEN256,
                                                                BLEN128, BLEN64, BLEN32, BLEN16, BLEN8
                                                                or BLEN4 field */
  HAL_ETH_DMA_BURST_LEN_MAX_ALLOWED = 0x00000000UL         /*!< Burst transfers of length equal to or less than the
                                                                maximum allowed burst length */
} hal_eth_dma_burst_len_mode_t;

/**
  * @brief  ETH DMA Mixed Burst Mode Enumeration definition
  */
typedef enum
{
  HAL_ETH_DMA_MIXED_BURST_MODE_ENABLED   = ETH_DMASBMR_MB, /*!< Mixed Burst is enabled.
                                                                When this control is enabled, the BUS master performs
                                                                undefined bursts transfers (INCR) for burst length of
                                                                16 or more. For burst length of 16 or less, the BUS
                                                                master performs fixed burst transfers
                                                                (INCRx and SINGLE). */
  HAL_ETH_DMA_MIXED_BURST_MODE_DISABLED  = 0UL,            /*!<  Mixed Burst is disabled.*/
} hal_eth_dma_mixed_burst_ctrl_t;

/**
  * @brief  ETH DMA Rebuild Increment Burst Enumeration definition
  */
typedef enum
{
  HAL_ETH_DMA_REBUILD_INC_BURST_ENABLED   = ETH_DMASBMR_RB, /*!< Rebuild INCRx Burst is enabled.
                                                                 When this control is enabled and the System Bus master
                                                                 gets SPLIT, RETRY, or Early Burst Termination (EBT)
                                                                 response, the AHB master interface rebuilds the pending
                                                                 beats of any initiated burst transfer with INCRx and
                                                                 SINGLE transfers. By default, the BUS master interface
                                                                 rebuilds the pending beats of an EBT with an
                                                                 unspecified (INCR) burst. */
  HAL_ETH_DMA_REBUILD_INC_BURST_DISABLED  = 0UL,            /*!< Rebuild INCRx Burst is disabled. For EBT/SPLIT/RETRY,
                                                                 pending beats are rebuilt using unspecified-length
                                                                 (INCR) bursts (default behavior) */
} hal_eth_dma_rebuild_inc_ctrl_t;

/**
  * @brief  DMA Transmit Priority Control enumeration.
  *
  * This enumeration controls whether the DMA gives priority to transmit operations
  * over receive operations. Enabling this can improve transmit performance in
  * scenarios where transmit latency is critical.
  */
typedef enum
{
  HAL_ETH_DMA_TR_PRIO_DISABLE       = 0UL,                 /*!< Transmit priority is disabled; receive operations are
                                                                prioritized over transmit. */
  HAL_ETH_DMA_TR_PRIO_ENABLE        = ETH_DMAMR_TXPR       /*!< Transmit priority is enabled; transmit operations are
                                                                prioritized over receive. */
} hal_eth_dma_tr_prio_ctrl_t;

/**********************************  ETH DMA Tx Channel Configuration Enumerations  **********************************/
/**
  * @brief ETH DMA Tx PBLx8 mode control enumeration definition.
  *
  * This enumeration enables or disables the multiplication by eight of the
  * programmable burst length (PBL) via the PBLX8 bit in ETH_DMACCR.
  */
typedef enum
{
  HAL_ETH_DMA_TX_PBL_X8_DISABLE     = 0UL,                 /*!< ETH TX DMA Disable 8xPBL mode */
  HAL_ETH_DMA_TX_PBL_X8_ENABLE      = ETH_DMACCR_PBLX8    /*!< ETH TX DMA Enable 8xPBL mode */
} hal_eth_dma_tx_pbl_x8_mode_ctrl_t;

/**
  * @brief HAL ETH DMA Tx Burst Length
  */
typedef enum
{
  HAL_ETH_DMA_TX_BLEN_1_BEAT        = 0x00010000UL,        /*!< DMA Transmit Programmable Burst Length = 1 beat   */
  HAL_ETH_DMA_TX_BLEN_2_BEAT        = 0x00020000UL,        /*!< DMA Transmit Programmable Burst Length = 2 beats  */
  HAL_ETH_DMA_TX_BLEN_4_BEAT        = 0x00040000UL,        /*!< DMA Transmit Programmable Burst Length = 4 beats  */
  HAL_ETH_DMA_TX_BLEN_8_BEAT        = 0x00080000UL,        /*!< DMA Transmit Programmable Burst Length = 8 beats  */
  HAL_ETH_DMA_TX_BLEN_16_BEAT       = 0x00100000UL,        /*!< DMA Transmit Programmable Burst Length = 16 beats */
  HAL_ETH_DMA_TX_BLEN_32_BEAT       = 0x00200000UL         /*!< DMA Transmit Programmable Burst Length = 32 beats */
} hal_eth_dma_tx_burst_length_t;

/**
  * @brief  Enables or disables the Operate on second Packet mode, which allows the DMA to process a second
  *         Packet of Transmit data even before obtaining the status for the first one.
  */
typedef enum
{
  HAL_ETH_DMA_TX_SEC_PKT_OP_DISABLE = 0UL,                 /*!< ETH TX DMA Disable Operate on Second Packet */
  HAL_ETH_DMA_TX_SEC_PKT_OP_ENABLE  = ETH_DMACTXCR_OSF    /*!< ETH TX DMA Enable Operate on Second Packet  */
} hal_eth_dma_tx_sec_pkt_op_ctrl_t;

/**********************************  ETH DMA Rx Channel Configuration Enumerations  **********************************/
/**
  * @brief HAL ETH DMA Rx Burst Length
  */
typedef enum
{
  HAL_ETH_DMA_RX_BLEN_1_BEAT        = 0x00010000UL,        /*!< DMA Receive Programmable Burst Length = 1 beat   */
  HAL_ETH_DMA_RX_BLEN_2_BEAT        = 0x00020000UL,        /*!< DMA Receive Programmable Burst Length = 2 beats  */
  HAL_ETH_DMA_RX_BLEN_4_BEAT        = 0x00040000UL,        /*!< DMA Receive Programmable Burst Length = 4 beats  */
  HAL_ETH_DMA_RX_BLEN_8_BEAT        = 0x00080000UL,        /*!< DMA Receive Programmable Burst Length = 8 beats  */
  HAL_ETH_DMA_RX_BLEN_16_BEAT       = 0x00100000UL,        /*!< DMA Receive Programmable Burst Length = 16 beats */
  HAL_ETH_DMA_RX_BLEN_32_BEAT       = 0x00200000UL         /*!< DMA Receive Programmable Burst Length = 32 beats */
} hal_eth_dma_rx_burst_length_t;

/**************************************  ETH MTL Configuration Enumerations  *****************************************/

/**
  * @brief HAL ETH MTL Tx Forward Status to Application
  */
typedef enum
{
  HAL_ETH_MTL_TX_FWD_STATUS_DISABLE = ETH_MTLOMR_DTXSTS,   /*!< Tx Forward Status to application Disable */
  HAL_ETH_MTL_TX_FWD_STATUS_ENABLE  = 0UL                  /*!< Tx Forward Status to application Enable  */
} hal_eth_mtl_tx_fwd_status_ctrl_t;

/**********************************  ETH MTL Tx Queue Configuration Enumerations  ************************************/
/**
  * @brief HAL ETH MTL Tx Queue Operation Mode
  */
typedef enum
{
  HAL_ETH_MTL_TX_QUEUE_ENABLED      = 0x00000008UL         /*!< Transmit Queue Enabled            */
} hal_eth_mtl_tx_ops_mode_t;

/**
  * @brief  MTL Transmit Queue Size enumeration.
  *
  * This enumeration specifies the size in bytes of the Ethernet MTL Transmit queue.
  */
typedef enum
{
  HAL_ETH_MTL_TX_QUEUE_SZ_2048_BYTE = (7UL << ETH_MTLTXQOMR_TQS_Pos), /*!< Transmit Queue Size 2048 bytes */
} hal_eth_mtl_tx_queue_size_t;

/**
  * @brief HAL ETH MTL Transmit Mode
  */
typedef enum
{
  HAL_ETH_MTL_TX_Q_STORE_AND_FORWARD  = ETH_MTLTXQOMR_TSF,    /*!< Transmit Store and Forward Mode      */
  HAL_ETH_MTL_TX_Q_THRESHOLD_32_BYTE  = 0x00000000UL,          /*!< Transmit Threshold Control 32 bytes  */
  HAL_ETH_MTL_TX_Q_THRESHOLD_64_BYTE  = 0x00000010UL,          /*!< Transmit Threshold Control 64 bytes  */
  HAL_ETH_MTL_TX_Q_THRESHOLD_96_BYTE  = 0x00000020UL,          /*!< Transmit Threshold Control 96 bytes  */
  HAL_ETH_MTL_TX_Q_THRESHOLD_128_BYTE = 0x00000030UL,          /*!< Transmit Threshold Control 128 bytes */
  HAL_ETH_MTL_TX_Q_THRESHOLD_192_BYTE = 0x00000040UL,          /*!< Transmit Threshold Control 192 bytes */
  HAL_ETH_MTL_TX_Q_THRESHOLD_256_BYTE = 0x00000050UL,          /*!< Transmit Threshold Control 256 bytes */
  HAL_ETH_MTL_TX_Q_THRESHOLD_384_BYTE = 0x00000060UL,          /*!< Transmit Threshold Control 384 bytes */
  HAL_ETH_MTL_TX_Q_THRESHOLD_512_BYTE = ETH_MTLTXQOMR_TTC     /*!< Transmit Threshold Control 512 bytes */
} hal_eth_mtl_tx_transmit_mode_t;

/**********************************  ETH MTL Rx Queue Configuration Enumerations  ************************************/
/**
  * @brief HAL ETH MTL Rx Queue Operation Mode
  */
typedef enum
{
  HAL_ETH_MTL_RX_QUEUE_ENABLED      = 0x00000002UL,        /*!< Receive Queue Enabled            */
} hal_eth_mtl_rx_ops_mode_t;

/**
  * @brief  ETH MTL Receive Queue Size enumeration.
  *
  * This enumeration specifies the size in bytes of the Ethernet MTL Receive queue.
  */
typedef enum
{
  HAL_ETH_MTL_RX_QUEUE_SZ_2048_BYTE = (7UL << ETH_MTLRXQOMR_RQS_Pos), /*!< Receive Queue Size 2048 bytes */
} hal_eth_mtl_rx_queue_size_t;

/**
  * @brief  ETH MTL Drop Checksum Error Packet Control enumeration.
  *
  * This enumeration controls whether the Ethernet MAC MTL (MAC Transmission Layer)
  * will drop received packets with checksum errors or forward them to the application.
  */
typedef enum
{
  HAL_ETH_MTL_RX_DROP_CS_ERR_DISABLE = ETH_MTLRXQOMR_DIS_TCP_EF,   /*!< TCP-IP Packets with checksum errors
                                                                         are forwarded to the application. */
  HAL_ETH_MTL_RX_DROP_CS_ERR_ENABLE  = 0UL                          /*!< TCP-IP Packets with checksum errors
                                                                         are dropped and not forwarded to the
                                                                         application. */
} hal_eth_mtl_rx_drop_cs_err_ctrl_t;

/**
  * @brief  ETH MTL Forward Error Packets Control enumeration.
  *
  * This enumeration controls whether the Ethernet MTL will forward received
  * packets that have errors to the application or discard them.
  */
typedef enum
{
  HAL_ETH_MTL_RX_FWD_ERR_PKT_DISABLE = 0UL,                         /*!< Received packets with errors are discarded
                                                                         and not forwarded to the application. */
  HAL_ETH_MTL_RX_FWD_ERR_PKT_ENABLE  = ETH_MTLRXQOMR_FEP           /*!< Received packets with errors are
                                                                         forwarded to the application. */
} hal_eth_mtl_rx_fwd_err_pkt_ctrl_t;

/**
  * @brief ETH MTL Control for forwarding received undersized good packets in the MTL RX path.
  *
  * This enumeration defines the options for handling received undersized good packets
  * in the Ethernet MAC MTL (MAC Transmission Layer) RX path. Depending on the selected
  * value, the hardware will either discard or forward such packets to the application.
  */
typedef enum
{
  HAL_ETH_MTL_RX_FWD_USZ_PKT_DISABLE = 0UL,                         /*!< Received undersized good packets are discarded
                                                                         and not forwarded to the application. */
  HAL_ETH_MTL_RX_FWD_USZ_PKT_ENABLE  = ETH_MTLRXQOMR_FUP          /*!< Received undersized good packets are forwarded
                                                                         to the application. */
} hal_eth_mtl_rx_fwd_usz_pkt_ctrl_t;

/**
  * @brief HAL ETH MTL Receive Mode
  */
typedef enum
{
  HAL_ETH_MTL_RX_Q_STORE_AND_FORWARD = ETH_MTLRXQOMR_RSF, /*!< Receive Store and Forward operating mode */
  HAL_ETH_MTL_RX_Q_THRESHOLD_64      = 0x00000000UL,       /*!< Receive Threshold Control                */
  HAL_ETH_MTL_RX_Q_THRESHOLD_32      = 0x00000001UL,       /*!< Receive Threshold Control                */
  HAL_ETH_MTL_RX_Q_THRESHOLD_96      = 0x00000002UL,       /*!< Receive Threshold Control                */
  HAL_ETH_MTL_RX_Q_THRESHOLD_128     = ETH_MTLRXQOMR_RTC  /*!< Receive Threshold Control                */
} hal_eth_mtl_rx_queue_mode_t;
/**********************************  ETH MAC ARP Offloading Status Enumerations  *************************************/
/**
  * @brief HAL ETH MAC ARP Offloading Status
  */
typedef enum
{
  HAL_ETH_ARP_OFFLOAD_DISABLED      = 0UL,                 /*!< ETH ARP Offloading Disabled */
  HAL_ETH_ARP_OFFLOAD_ENABLED       = ETH_MACCR_ARPEN      /*!< ETH ARP Offloading Enabled */
} hal_eth_arp_offload_status_t;

/**********************************  ETH External WKUP Triggers Enumerations  ****************************************/
/**
  * @brief  Ethernet Wakeup trigger type enumeration.
  *
  * This enumeration defines the possible trigger conditions for the Ethernet
  * Wakeup External line. It is used to indicate which edge(s) have caused a pending
  * interrupt on the External line.
  */
typedef enum
{
  HAL_ETH_WAKEUP_TRIGGER_NONE           = LL_EXTI_TRIGGER_NONE,            /*!< No trigger pending */
  HAL_ETH_WAKEUP_TRIGGER_RISING         = LL_EXTI_TRIGGER_RISING,          /*!< Rising edge trigger pending */
  HAL_ETH_WAKEUP_TRIGGER_FALLING        = LL_EXTI_TRIGGER_FALLING,         /*!< Falling edge trigger pending */
  HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING = LL_EXTI_TRIGGER_RISING_FALLING   /*!< Both rising and falling edge triggers
                                                                                pending */
} hal_eth_wakeup_trigger_t;

/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group2 ETH handles types definitions
  * @{
  */
/**
  * @brief HAL ETH handle type definition
  */
typedef struct hal_eth_handle_s hal_eth_handle_t;         /*!< HAL ETH handle structure type */
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group3 ETH DMA descriptor lists types definitions
  * @{
  */
/**
  * @brief  DMA Transmit Descriptors Wrapper structure definition
  */
typedef struct
{
  uint32_t                         *p_desc_list_addr;                 /*!< Tx Descriptors Base address */
  uint32_t                          total_desc_cnt;                   /*!< TX total Number of descriptors */
  uint32_t                          desc_len_byte;                    /*!< TX Desc len in byte*/
  uint32_t                          curr_desc_id;                     /*!< TX current available Desc ID */
  uint32_t                          built_desc_id;                    /*!< TX built Desc ID */
  uint32_t                          buff_in_use;                      /*!< Buffers in Use */
} hal_eth_tx_descriptor_list_t;

/**
  * @brief  DMA Receive Descriptors Wrapper structure definition
  */
typedef struct
{
  uint32_t                         *p_desc_list_addr;                 /*!< Rx Descriptors base address */
  uint32_t                          total_desc_cnt;                   /*!< RX total Number of descriptors */
  uint32_t                          desc_len_byte;                    /*!< RX Desc len in byte*/
  uint32_t                          curr_desc_id;                     /*!< RX current available Desc */
  uint32_t                          built_desc_id;                    /*!< RX Built Desc */
  uint32_t                          buff_in_use;                      /*!< Rx Buffers in Use */
} hal_eth_rx_descriptor_list_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group4 ETH common types definitions
  * @{
  */
/**
  * @brief ETH FIFO Event Configuration structure definition
  */
typedef struct
{
  hal_eth_fifo_event_mode_t         event_mode;            /*!< ETH FIFO Event Mode */

  uint32_t                          event_params;          /*!< Event parameter. The value could be one of the
                                                                following :
                                                                  - Number of buffers to transfer by the DMA to trig
                                                                  the interruption when mode=HAL_ETH_FIFO_EVENT_CYCLIC.
                                                                  - or anything (to be ignored by the driver) when
                                                                  mode=HAL_ETH_FIFO_EVENT_ALWAYS or
                                                                  mode=HAL_ETH_FIFO_EVENT_NONE */
} hal_eth_fifo_event_config_t;

/**
  * @brief ETH Tx Callback Data structure definition
  */
typedef struct
{
  uint32_t                          status;                /*!< ETH Transmission Status.
                                                                This parameter can be a combination
                                                                of @ref ETH_Channel_Tx_Status */
  uint32_t                          errors;                /*!< ETH Transmission Errors.
                                                                This parameter can be a combination
                                                                of @ref ETH_Channel_Tx_Errors */
  void                              *p_data;               /*!< Specifies Application packet pointer */
} hal_eth_tx_cb_pkt_data_t;

/**
  * @brief ETH Rx Callback Packet Data structure definition
  */
typedef struct
{
  uint32_t                          status;                /*!< ETH Reception Status.
                                                                This parameter can be a combination
                                                                of @ref ETH_Channel_Rx_Status */
  uint32_t                          errors;                /*!< ETH Reception Errors.
                                                                This parameter can be a combination
                                                                of @ref ETH_Channel_Rx_Errors */
  void                             *p_data;                /*!< Specifies Application packet pointer */
  uint32_t                          vlan_tag_ids;          /*!< ETH Reception VLAN Tag ID.
                                                                This parameter can be a value from 0x0 to 0xFFFF */
} hal_eth_rx_cb_pkt_data_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group5 ETH callbacks types definitions
  * @{
  */
/**
  * @brief Pointer to a function for handling Ethernet Tx completion events.
  *
  * **Function Signature**
  * ```c
  *   hal_status_t callback(hal_eth_handle_t *heth, uint32_t channel, void *p_buf,
  *                         hal_eth_tx_cb_pkt_data_t tx_pkt_data);
  * ```
  *
  * @note
  *   The @p p_tx_complete_cb callback is invoked in the following scenarios:
  *   - **Peripheral De-Initialization:** When @ref HAL_ETH_DeInit is called to de-initialize the Ethernet peripheral,
  *     all allocated buffers and their metadata are released, and the transmission path is stopped.
  *   - **Channel Stop:** When @ref HAL_ETH_StopChannel is called to stop the channel, all allocated buffers and their
  *     metadata are released, and the transmission path is stopped.
  *   - **Data Handling:** When @ref HAL_ETH_ExecDataHandler is called to process transmitted data, transmitted frames
  *     and their metadata are returned to the application.
  *
  * **Parameters:**
  *
  * | Type                         | Dir  | Name         | Description                               |
  * |------------------------------|------|--------------|-------------------------------------------|
  * | `hal_eth_handle_t *`         | [in] | heth         | Pointer to the Ethernet handle.           |
  * | `uint32_t`                   | [in] | channel      | Channel number.                           |
  * | `void *`                     | [in] | p_buf        | Pointer to the transmitted buffer.        |
  * | `hal_eth_tx_cb_pkt_data_t *` | [in] | tx_pkt_data  | Packet-specific data for the Tx callback. |
  *
  * @retval     HAL_OK       Callback executed successfully.
  * @retval     HAL_ERROR    An error occurred during callback execution.
  *
  * @note If the application does not complete the callback execution successfully, the driver's behavior will vary
  *   depending on the context:
  *   - **Peripheral De-Initialization or Channel Stop:** If the failure occurs during peripheral de-initialization or
  *     while stopping a channel, the driver will continue processing data for the affected channel and will release
  *     all resources associated with it. The application can choose to treat any unreleased resources as unusable for
  *     future Ethernet HAL driver operations.
  *   - **Data Handling:** If the failure occurs during data handling, the driver will suspend data processing for the
  *     affected channel. The index of this channel will be included in the list of output channels for which data
  *     handling was not completed (as returned by @p p_output_channel_mask parameter of @ref HAL_ETH_ExecDataHandler.
  *     To resume data handling on the affected channel, invoke the @ref HAL_ETH_ExecDataHandler API.
  *
  * @see HAL_ETH_ExecDataHandler
  * @see hal_eth_tx_cb_pkt_data_t
  * @see hal_eth_handle_t
  */
typedef hal_status_t (*hal_eth_tx_complete_cb_t)(hal_eth_handle_t *, uint32_t, void *, hal_eth_tx_cb_pkt_data_t);

/**
  * @brief Pointer to a function for handling Ethernet Rx completion events.
  *
  * **Function Signature**
  * ```c
  *   hal_status_t callback(hal_eth_handle_t *heth, uint32_t channel, void *p_buf, uint32_t size,
  *                         hal_eth_rx_cb_pkt_data_t rx_pkt_data);
  * ```
  *
  * @note
  *   The @p p_rx_complete_cb callback is invoked in the following scenarios:
  *   - **Peripheral De-Initialization:** When @ref HAL_ETH_DeInit is called to de-initialize the Ethernet peripheral,
  *     all allocated buffers and their metadata are released, and the reception path is stopped.
  *   - **Channel Stop:** When @ref HAL_ETH_StopChannel is called to stop the channel, all allocated buffers and their
  *     metadata are released, and the reception path is stopped.
  *   - **Data Handling:** When @ref HAL_ETH_ExecDataHandler is called to process received data, received frames and
  *     their metadata are returned to the application.
  *
  * **Parameters:**
  *
  * | Type                         | Dir  | Name         | Description                               |
  * |------------------------------|------|--------------|-------------------------------------------|
  * | `hal_eth_handle_t *`         | [in] | heth         | Pointer to the Ethernet handle.           |
  * | `uint32_t`                   | [in] | channel      | Channel number.                           |
  * | `void *`                     | [in] | p_buf        | Pointer to the received buffer.           |
  * | `uint32_t`                   | [in] | size         | Size of the received data in bytes.       |
  * | `hal_eth_rx_cb_pkt_data_t *` | [in] | rx_pkt_data  | Packet-specific data for the Rx callback. |
  *
  * @retval     HAL_OK       Callback executed successfully.
  * @retval     HAL_ERROR    An error occurred during callback execution.
  *
  * @note If the application does not complete the callback execution successfully, the driver's behavior will vary
  *   depending on the context:
  *   - **Peripheral De-Initialization or Channel Stop:** If the failure occurs during peripheral de-initialization or
  *     while stopping a channel, the driver will continue processing data for the affected channel and will return
  *     ownership (give back) of all resources associated with that channel to the application. The application can
  *     choose to treat any unreleased resources as unusable for future Ethernet HAL driver operations.
  *   - **Data Handling:** If the failure occurs during data handling, the driver will halt data processing for the
  *     affected channel. The index of this channel will be included in the list of output channels for which data
  *     handling was not completed (as returned by @p p_output_channel_mask parameter of @ref HAL_ETH_ExecDataHandler.
  *     To resume data handling on the affected channel, invoke the @ref HAL_ETH_ExecDataHandler API.
  *
  * @see HAL_ETH_StopChannel
  * @see HAL_ETH_ExecDataHandler
  * @see hal_eth_rx_cb_pkt_data_t
  * @see hal_eth_handle_t
  */
typedef hal_status_t (*hal_eth_rx_complete_cb_t)(hal_eth_handle_t *, uint32_t, void *, uint32_t,
                                                 hal_eth_rx_cb_pkt_data_t);

/**
  * @brief Pointer to a function for allocating buffers for Ethernet Rx operations.
  *
  * **Function Signature**
  * ```c
  *   void callback(hal_eth_handle_t *heth, uint32_t channel, uint32_t size, void **p_buf, void **p_data);
  * ```
  *
  * @note
  *   The @p p_rx_allocate_cb callback is invoked in the following scenarios:
  *   - **Channel Start:** When @ref HAL_ETH_StartChannel() is called, the callback allocates initial buffers and
  *     prepares the reception path.
  *   - **Data Handling:** When @ref HAL_ETH_ExecDataHandler() is called, the callback requests memory for upcoming
  *     reception requests.
  *
  *   **Buffer Allocation Behavior:**
  *   - For the channel start scenario, if the application does not succeed in allocating a buffer for reception,
  *     the driver will stop building the list of initial buffers for the affected channel. The maximum number of
  *     buffers to be requested, as specified by the application during channel configuration, will be reduced to the
  *     number of buffers that were successfully allocated.
  *     - *Example:* If the maximum number of buffers is set to 10 but the application is only able to allocate
  *     3 buffers, the driver will use 3 as the new maximum for that channel.
  *   - For the data handling scenario, if the application does not succeed in allocating buffers for reception,
  *     the driver will halt data handling for the affected channel. The index of this channel will appear in the list
  *     of output channels where data handling was not completed. To resume data handling on the affected channel,
  *     call the @ref HAL_ETH_ExecDataHandler() API.
  *
  *   **Reception Path Requirements:**
  *   - The driver requires at least 1 buffer to start the reception path; otherwise, the reception process will
  *     not be started.
  *   - The allocated memory must meet all constraints specified by the driver, such as memory address alignment and
  *     minimum size. These constraints are provided by the @ref HAL_ETH_GetChannelAllocNeeds API, which must be
  *     called before making the allocation request.
  *
  *   **Buffer Count Query:**
  *   - The application can check the number of initial buffers that the driver will be using for the reception path
  *     by calling @ref HAL_ETH_GetChannelBufferInUseCount() API immediately after starting the channel.
  *
  * **Parameters:**
  *
  * | Type                | Dir  | Name    | Description                                           |
  * |---------------------|------|---------|-------------------------------------------------------|
  * | `hal_eth_handle_t *`| [in] | heth    | Pointer to the Ethernet handle. @ref hal_eth_handle_t |
  * | `uint32_t`          | [in] | channel | Channel number.                                       |
  * | `uint32_t`          | [in] | size    | Required buffer size in bytes.                        |
  * | `void **`           | [out]| p_buf   | Pointer to the allocated buffer.                      |
  * | `void **`           | [out]| p_data  | Pointer to the allocated buffer metadata.             |
  *
  * @see HAL_ETH_StartChannel
  * @see HAL_ETH_ExecDataHandler
  * @see HAL_ETH_GetChannelBufferInUseCount
  * @see HAL_ETH_GetChannelAllocNeeds
  * @see hal_eth_rx_channel_config_t::max_app_buffers_num
  * @see hal_eth_handle_t
  */
typedef void (*hal_eth_rx_allocate_cb_t)(hal_eth_handle_t *, uint32_t, uint32_t, void **, void **);

#if defined(USE_HAL_ETH_REGISTER_CALLBACKS) && (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
/**
  * @brief Function pointer type for Ethernet event or error callbacks.
  *
  * This type defines a pointer to a callback function that is invoked to handle
  * generic Ethernet events or errors. The callback allows the application to
  * respond to various Ethernet-related notifications, such as low power status
  * changes, data events or peripheral error conditions.
  *
  * **Function Signature**
  * ```c
  * void callback(hal_eth_handle_t *heth, uint32_t events);
  * ```
  *
  * **Parameters:**
  *
  * | Type                 | Dir   | Name    | Description                                                          |
  * |----------------------|-------|---------|----------------------------------------------------------------------|
  * | `hal_eth_handle_t *` | [in]  | heth    | Pointer to the Ethernet handle structure.                            |
  * | `uint32_t`           | [in]  | events  | 32-bit value containing a combination of event flags (bitwise ORed). |
  *
  * @note
  *   - The interpretation of `events` value depends on the context in which the callback is
  *     invoked. Typically, it can be a bitmask or an enumerated value defined elsewhere
  *     in the driver.
  *   - The callback function is invoked in handler mode; therefore, it must
  *     execute quickly and must not perform any blocking operations.
  *   - If additional processing is required, it is recommended to defer such
  *     operations to a separate task or thread.
  *
  * **Associated Function Pointers**
  *   This callback type is used to implement the following Ethernet driver callbacks:
  *   - `p_data_cb`  : ETH Data Event Callback. The callback returns a bitwise ORed
  *   value for all channels that have a data event. The list of channels is provided
  *   in @ref ETH_Channel_Identifiers.
  *
  *   - `p_error_cb` : ETH Error Callback
  *   The following Errors are supported:
  *
  *   | Event                        | Description                                         |
  *   |------------------------------|-----------------------------------------------------|
  *   | HAL_ETH_ERROR_NONE           | No error.                                           |
  *   | HAL_ETH_ERROR_FBE            | Fatal Bus Error.                                    |
  *   | HAL_ETH_ERROR_CDE            | Context Descriptor Error.                           |
  *   | HAL_ETH_ERROR_FBE_DMA_TX_RD  | Bus Fault Error during read transfer by Tx DMA      |
  *   | HAL_ETH_ERROR_FBE_DMA_TX_WR  | Bus Fault Error during write transfer by Tx DMA     |
  *   | HAL_ETH_ERROR_FBE_DMA_TX_AC  | Bus Fault Error during descriptor access by Tx DMA  |
  *   | HAL_ETH_ERROR_FBE_DMA_RX_RD  | Bus Fault Error during read transfer by Rx DMA      |
  *   | HAL_ETH_ERROR_FBE_DMA_RX_WR  | Bus Fault Error during write transfer by Rx DMA     |
  *   | HAL_ETH_ERROR_FBE_DMA_RX_AC  | Bus Fault Error during descriptor access by Rx DMA  |
  *
  *   - `p_event_cb` : ETH Event Callback.
  *   The following Peripheral Events are supported:
  *
  *   | Event                        | Description                 |
  *   |------------------------------|-----------------------------|
  *   | HAL_ETH_EVENT_MAC_RWT    | Receive Watchdog Timeout Event. |
  *   | HAL_ETH_EVENT_MAC_EXCOL  | Excessive Collisions Event.     |
  *   | HAL_ETH_EVENT_MAC_LCOL   | Late Collision Event.           |
  *   | HAL_ETH_EVENT_MAC_EXDEF  | Excessive Deferral Event.       |
  *   | HAL_ETH_EVENT_MAC_LCARR  | Loss of Carrier Event.          |
  *   | HAL_ETH_EVENT_MAC_NCARR  | No Carrier Event.               |
  *   | HAL_ETH_EVENT_MAC_TJT    | Transmit Jabber Timeout Event.  |
  *
  *   - `p_pmt_cb`   : ETH Power Management Callback
  *   The following Peripheral PMT Events are supported:
  *
  *   | Event                           | Description                     |
  *   |---------------------------------|---------------------------------|
  *   | HAL_ETH_EVENT_PMT_MAGIC_PACKET  | Magic Packet Received.          |
  *   | HAL_ETH_EVENT_PMT_RWK_PACKET    | Remote wake-up Packet Received. |
  *
  *   - `p_eee_cb`   : ETH EEE (Energy Efficient Ethernet) Callback
  *   The following Peripheral EEE/LPI Events are supported:
  *
  *   | Event                        | Description                          |
  *   |------------------------------|--------------------------------------|
  *   | HAL_ETH_EVENT_LPI_PLS_DOWN   | PHY Link Status is Down.             |
  *   | HAL_ETH_EVENT_LPI_PLS_UP     | PHY Link Status is Up.               |
  *   | HAL_ETH_EVENT_LPI_TX_LPI_ST  | Transmit LPI State Active.           |
  *   | HAL_ETH_EVENT_LPI_RX_LPI_ST  | Receive LPI State Active.            |
  *   | HAL_ETH_EVENT_LPI_TX_LPI_EN  | Transmit LPI State Entry performed.  |
  *   | HAL_ETH_EVENT_LPI_RX_LPI_EN  | Receive LPI State Entry performed.   |
  *   | HAL_ETH_EVENT_LPI_TX_LPI_EX  | Transmit LPI State Entry performed.  |
  *   | HAL_ETH_EVENT_LPI_RX_LPI_EX  | Receive LPI State Entry performed.   |
  *
  * @see hal_eth_handle_t
  * @see HAL_ETH_DataCallback
  * @see HAL_ETH_ErrorCallback
  * @see HAL_ETH_EventCallback
  * @see HAL_ETH_PMTCallback
  * @see HAL_ETH_EEECallback
  */
typedef void (*hal_eth_cb_t)(hal_eth_handle_t *, uint32_t);

/**
  * @brief Function pointer type for Ethernet wakeup event callbacks.
  *
  * This type defines a pointer to a callback function that is invoked to handle
  * Ethernet wakeup events. The callback allows the application to respond when
  * the Ethernet peripheral wakes up from a low-power state.
  *
  * **Function Signature**
  * ```c
  * void callback(hal_eth_handle_t *heth);
  * ```
  *
  * **Parameters:**
  *
  * | Type                 | Dir   | Name | Description                                 |
  * |----------------------|-------|------|---------------------------------------------|
  * | `hal_eth_handle_t *` | [in]  | heth    | Pointer to the Ethernet handle structure |
  *
  * @note
  *   - The callback function is invoked in handler mode; therefore, it must
  *     execute quickly and must not perform any blocking operations.
  *   - If additional processing is required, it is recommended to defer such
  *     operations to a separate task or thread.
  *
  * @see hal_eth_handle_t
  * @see HAL_ETH_WakeUpCallback
  */
typedef void (*hal_eth_wakeup_cb_t)(const hal_eth_handle_t *);

/**
  * @brief Function pointer type for Ethernet Tx/Rx channel event callbacks.
  *
  * This type defines a pointer to a callback function that is invoked to handle
  * events related to specific Ethernet transmit (Tx) or receive (Rx) channels.
  * The callback allows the application to respond to channel-specific notifications,
  * such as data transfer completions status or error conditions.
  *
  * **Function Signature**
  * ```c
  * void callback(hal_eth_handle_t *heth, uint32_t channel, uint32_t events);
  * ```
  *
  * **Parameters:**
  *
  * | Type                 | Dir   | Name    | Description                                                          |
  * |----------------------|-------|---------|----------------------------------------------------------------------|
  * | `hal_eth_handle_t *` | [in]  | heth    | Pointer to the Ethernet handle structure                             |
  * | `uint32_t`           | [in]  | channel | Channel number (Tx or Rx) on which the event occurred.               |
  * | `uint32_t`           | [in]  | events  | 32-bit value containing a combination of event flags (bitwise ORed). |
  *
  * @note
  *   The following events are supported:
  *
  *   | Event                       | Description                   |
  *   |-----------------------------|-------------------------------|
  *   | HAL_ETH_CH_EVENT_DMA_RBU    | Receive Buffer Unavailable.   |
  *   | HAL_ETH_CH_EVENT_DMA_TBU    | Transmit Buffer Unavailable.  |
  *   | HAL_ETH_CH_EVENT_DMA_RWT    | Receive Watchdog Timeout.     |
  *   | HAL_ETH_CH_EVENT_MTL_RX_OF  | MTL Receive Queue Overflow.   |
  *   | HAL_ETH_CH_EVENT_MTL_TX_OF  | MTL Transmit Queue Underflow. |
  *
  * @note
  *   - The callback function can be invoked in handler mode; therefore, it must
  *     execute quickly and must not perform any blocking operations.
  *   - If additional processing is required, it is recommended to defer such
  *     operations to a separate task or thread.
  *
  * @see hal_eth_handle_t
  * @see ETH_Channel_Event_Codes
  * @see HAL_ETH_TxEventCallback
  * @see HAL_ETH_RxEventCallback
  */
typedef void (*hal_eth_channel_cb_t)(hal_eth_handle_t *, uint32_t, uint32_t);

/**
  * @brief Function pointer type for Ethernet cache flush/invalidate callbacks.
  *
  * This type defines a pointer to a callback function that is invoked to handle
  * cache maintenance operations (flush or invalidate) for the Ethernet driver
  * shared memory associated with specific transmit (Tx) or receive (Rx) channel.
  * The callback allows the application to ensure data coherency between the CPU
  * and DMA by performing appropriate cache maintenance operations on the buffer.
  *
  * **Function Signature**
  * ```c
  * void callback(hal_eth_handle_t *heth, uint32_t channel, void *p_buf, uint32_t size);
  * ```
  *
  * **Parameters:**
  *
  * | Type                 | Dir   | Name    | Description                                                           |
  * |----------------------|-------|---------|-----------------------------------------------------------------------|
  * | `hal_eth_handle_t *` | [in]  | heth    | Pointer to the Ethernet handle structure                              |
  * | `uint32_t`           | [in]  | channel | Channel number (Tx or Rx) for which the cache operation is performed. |
  * | `void *`             | [in]  | p_buf   | Pointer to the data buffer that requires cache flush or invalidate.   |
  * | `uint32_t`           | [in]  | size    | Size of the data buffer in bytes.                                     |
  *
  * @note
  *   - Proper cache maintenance is essential to ensure data integrity when using
  *     cached memory.
  *   - Synchronized memory regions must be aligned to cache line boundaries, as
  *     specified by the channel configuration alignment requirements provided
  *     by the user.
  *   - The callback function can be invoked in handler mode; therefore, it must
  *     execute quickly and must not perform any blocking operations.
  *   - Cache maintenance operations must be completed before returning control to
  *     the callee, ensuring that synchronized memory is ready for the next execution.
  *   - The Invalidate() operation must be performed when ownership of the driver's
  *     shared memory is transferred to the CPU.
  *   - The Flush() operation must be performed when ownership of the driver's shared
  *     memory is transferred to the hardware. For optimal performance, consider using a
  *     CleanInvalidate instruction, as there will be no further CPU data reuse (read)
  *     until the driver's shared memory is updated (write-back) by the Ethernet DMA
  *     hardware.
  *
  * **Associated Function Pointers**
  *   This callback type is used to implement the following Ethernet driver callbacks:
  *   - `p_cache_invalidate_cb` : ETH Cache Invalidate (Peripheral to CPU direction).
  *   - `p_cache_flush_cb`      : ETH Cache Flush (CPU to Peripheral direction).
  *
  * @see hal_eth_handle_t
  * @see HAL_ETH_CacheInvalidateCallback
  * @see HAL_ETH_CacheFlushCallback
  */
typedef void (*hal_eth_cache_cb_t)(hal_eth_handle_t *, uint32_t, void *, uint32_t);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup ETH_Exported_Types_Group2
  * @{
  */
/**
  * @brief  ETH Tx Channel Handle Structure definition
  */
typedef struct
{
  ETH_DMA_Channel_TypeDef          *p_dma_instance;        /*!< Tx/Rx DMA Instances Register base address    */

  ETH_MTL_Queue_TypeDef            *p_mtl_instance;        /*!< Tx/Rx MTL Instances Register base address    */

  volatile hal_eth_channel_state_t  channel_state;         /*!< ETH channel state information related to
                                                                channel handle management
                                                                and also related to Tx operations */

  volatile uint32_t                 channel_lock_state;    /*!< ETH channel lock state for managing atomic
                                                                access and preventing concurrent
                                                                operations */

  hal_eth_tx_descriptor_list_t      tx_desc_list;          /*!< DMA Channel Tx descriptor wrapper: holds
                                                                all Tx descriptors
                                                                list addresses and current descriptor index  */

  hal_eth_fifo_event_config_t      fifo_event_config;      /*!< Request FIFO event Configuration for the Channel */

  uint32_t                         event_cnt;              /*!< counter of fifo events */

  /* Required Channel Callbacks */
  hal_eth_tx_complete_cb_t          p_tx_complete_cb;      /*!< ETH Tx Complete Callback     */

  /* Optional Channel Callbacks */
#if defined (USE_HAL_ETH_REGISTER_CALLBACKS) && (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
  hal_eth_channel_cb_t              p_ch_event_cb;         /*!< ETH DMA Tx Event Callback    */
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */

} hal_eth_tx_channel_handle_t;

/**
  * @brief  ETH Rx Channel Handle Structure definition
  */
typedef struct
{
  void                             *p_dma_instance;        /*!< Tx/Rx DMA Instances Register base address    */

  void                             *p_mtl_instance;        /*!< Tx/Rx MTL Instances Register base address    */

  volatile hal_eth_channel_state_t  channel_state;         /*!< ETH channel state information related to
                                                                channel handle management
                                                                and also related to Tx operations */

  volatile uint32_t                 channel_lock_state;    /*!< ETH channel lock state for managing atomic
                                                                access and preventing concurrent
                                                                operations */

  hal_eth_rx_descriptor_list_t      rx_desc_list;          /*!< DMA Channel Rx descriptor wrapper: holds
                                                                all Rx descriptors
                                                                list addresses and current descriptor index  */

  uint32_t                          rx_buff_size_byte;     /*!< Rx buffers specified in bytes
                                                              The maximum buffer size is limited to 16 Kbytes.
                                                              this field is an arg of @see hal_eth_rx_allocate_cb_t*/

  hal_eth_fifo_event_config_t       fifo_event_config;     /*!< Request FIFO event Configuration for the Channel */

  uint32_t                          event_cnt;             /*!< counter of fifo events */

  /* Mandatory Rx Channel Callbacks */
  hal_eth_rx_complete_cb_t          p_rx_complete_cb;      /*!< ETH Rx Complete Callback     */

  hal_eth_rx_allocate_cb_t          p_rx_allocate_cb;      /*!< ETH Rx Get Buffer Function   */

  /* Optional Rx Channel Callbacks */
#if defined (USE_HAL_ETH_REGISTER_CALLBACKS) && (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
  hal_eth_channel_cb_t              p_ch_event_cb;         /*!< ETH DMA Rx Event Callback    */
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */

} hal_eth_rx_channel_handle_t;

/**
  * @brief  ETH Handle Structure definition
  */
struct hal_eth_handle_s
{
  hal_eth_t                         instance;                             /*!< Peripheral instance */

  hal_eth_tx_channel_handle_t       tx_channels[ETH_NB_OF_TX_CHANNEL]; /*!< Tx Channels Handles    */

  hal_eth_rx_channel_handle_t       rx_channels[ETH_NB_OF_RX_CHANNEL]; /*!< Rx Channels Handles    */

  volatile hal_eth_state_t          global_state;                         /*!< ETH state information related to global
                                                                               Handle management */

#if defined (USE_HAL_ETH_USER_DATA) && (USE_HAL_ETH_USER_DATA == 1)
  const void                       *p_user_data;                          /*!< User data pointer */
#endif /* USE_HAL_ETH_USER_DATA */

#if defined (USE_HAL_ETH_REGISTER_CALLBACKS) && (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
  hal_eth_cb_t                      p_data_cb;                            /*!< ETH Data Event Callback       */

  hal_eth_cb_t                      p_error_cb;                           /*!< ETH Error Callback            */

  hal_eth_cb_t                      p_event_cb;                           /*!< ETH Event Callback            */

  hal_eth_cb_t                      p_pmt_cb;                             /*!< ETH Power Management Callback */

  hal_eth_cb_t                      p_eee_cb;                             /*!< ETH EEE Callback              */

  hal_eth_wakeup_cb_t               p_wake_up_cb;                         /*!< ETH Wake UP Callback          */

  hal_eth_cache_cb_t                p_cache_invalidate_cb;                /*!< ETH Cache Invalidate (Peripheral to CPU
                                                                                direction) Callback.
                                                                                This callback are called AFTER
                                                                                peripheral/DMA writes. */

  hal_eth_cache_cb_t                p_cache_flush_cb;                     /*!< ETH Cache Flush (CPU to Peripheral
                                                                                direction) Callback.
                                                                                This callback are called BEFORE
                                                                                peripheral/DMA reads. */
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t                semaphore;                            /*!< ETH OS semaphore               */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_ETH_GET_LAST_ERRORS) && (USE_HAL_ETH_GET_LAST_ERRORS == 1)
  /*! Last error codes on ETH peripheral side */
  volatile uint32_t                 last_error_codes;
#endif /* USE_HAL_USART_GET_LAST_ERRORS */
};
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group6 ETH peripheral configuration structures
  * @{
  */
/**
  * @brief  ETH Peripheral Configuration structure definition
  */
typedef struct
{
  uint8_t                           mac_addr[6];           /*!< MAC Address of used Hardware: must be pointer on an
                                                                array of 6 bytes */

  hal_eth_media_interface_t         media_interface;       /*!< Selects the Ethernet Media interface. */

} hal_eth_config_t;

/**
  * @brief  ETH Peripheral Auto-Negotiation Link Configuration structure definition.
  *         Auto-negotiation is a process between PHY layer devices that enables
  *         them to automatically exchange information about their capabilities,
  *         such as speed, duplex mode, and flow control. This allows both devices
  *         to select the highest performance mode supported by both ends of the
  *         link.
  *         The MAC is responsible for final Auto-Negotiation link configuration
  *         resolution after a link is established, and is responsible for correct
  *         flow control, energy efficient ethernet actions thereafter.
  */
typedef struct
{
  /* LINK Speed/Duplex configuration */
  hal_eth_mac_speed_t               speed;                 /*!< Sets the Ethernet speed: 10/100/1000 Mbps */

  hal_eth_mac_duplex_mode_t         duplex_mode;           /*!< Selects the MAC duplex mode: Half-Duplex or Full-Duplex
                                                                mode */
} hal_eth_link_config_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group7 ETH MAC - DMA - MTL Sub-Blocks configuration structures
  * @{
  */
/**
  * @brief  ETH MAC Configuration structure definition
  */
typedef struct
{
  hal_eth_link_config_t             link_config;           /*!< Sets the Ethernet Link configuration */

  hal_eth_mac_loopback_ctrl_t       loopback_mode;         /*!< Enables or disables the loopback mode */

  hal_eth_mac_src_addr_ctrl_t       src_addr_ctrl;         /*!< Selects the Source Address Insertion or Replacement
                                                                Control */

  hal_eth_mac_inter_pkt_gap_t       inter_pkt_gap_value;   /*!< Sets the minimum IPG between Packet during transmission
                                                                */

  hal_eth_mac_back_off_limit_t      back_off_limit;        /*!< Selects the BackOff limit value */

  hal_eth_mac_preeamble_length_t    preamble_length;       /*!< Selects or not the Preamble Length for Transmit packets
                                                                (Full Duplex mode) */

  hal_eth_mac_gpkt_sz_limit_ctrl_t  giant_pkt_size_limit_ctrl;  /*!< Enables or disables the Giant Packet Size Limit
                                                                Control. */

  hal_eth_mac_2k_pkt_len_ctrl_t     support_2K_pkt;        /*!< Enables or disables the IEEE 802.3as Support for 2K
                                                                length Packets */

  hal_eth_mac_crc_strip_pkt_ctrl_t  crc_strip_type_pkt;    /*!< Enables or disables the CRC stripping for Type
                                                                packets.*/

  hal_eth_mac_auto_pad_crc_s_ctrl_t auto_pad_crc_strip;    /*!< Enables or disables  the Automatic MAC Pad/CRC
                                                                Stripping.*/

  hal_eth_mac_tx_jabber_tim_ctrl_t  tx_jabber;             /*!< Enables or disables Jabber timer on Tx path.*/

  hal_eth_mac_cs_before_tr_ctrl_t   cs_before_transmit;    /*!< Enables or disables the Carrier Sense Before
                                                                Transmission in Full Duplex mode */

  hal_eth_mac_cs_during_tr_ctrl_t   cs_during_transmit;    /*!< Enables or disables the Carrier Sense During
                                                                Transmission in the Half Duplex mode */

  hal_eth_mac_retry_tr_ctrl_t       retry_transmission;    /*!< Enables or disables the MAC retry transmission, when a
                                                                collision occurs in Half Duplex mode.*/

  hal_eth_mac_rx_wd_tim_ctrl_t      rx_watchdog;           /*!< Enables or disables the Watchdog timer on Rx path.*/

  hal_eth_mac_rx_jumbo_pkt_ctrl_t   rx_jumbo_pkt;          /*!< Enables or disables receiving Jumbo
                                                                Packet
                                                                When enabled, the MAC allows jumbo packets of 9,018
                                                                bytes without reporting a giant packet error */

  hal_eth_mac_rx_csum_pkt_ctrl_t    rx_csum_offload;       /*!< Enables or Disable the checksum checking for received
                                                                packet payloads TCP, UDP or ICMP headers */

  hal_eth_mac_rx_receive_own_ctrl_t rx_receive_own;        /*!< Enables or disables the Receive Own in Half Duplex mode
                                                                */

  hal_eth_mac_rx_crc_pkt_chk_ctrl_t crc_checking_rx_pkts;  /*!< Enable or disables the CRC Checking for Received
                                                                Packets. */

  hal_eth_mac_deferral_check_ctrl_t deferral_check;        /*!< Enables or disables the deferral check function in Half
                                                                Duplex mode. */

  hal_eth_mac_uc_slow_proto_ctrl_t  uc_slow_proto_detect;  /*!< Enable or disables the Detection of Slow Protocol
                                                                Packets with unicast address. */

  hal_eth_mac_slow_proto_ctrl_t     slow_proto_detect;     /*!< Enable or disables the Slow Protocol Detection. */


  uint32_t                          giant_pkt_size_limit;  /*!< Specifies the packet size that the MAC
                                                                will declare it as Giant, If it's size is
                                                                greater than the value programmed in this field in
                                                                This parameter must be a number between
                                                                Min_Data = 0x618 (1518 byte) and
                                                                Max_Data = 0x3FFF (32 Kbyte). */

  hal_eth_mac_ex_int_pkt_gap_ctrl_t ext_inter_pkt_gap_ctrl; /*!< Enable or disables the extended inter packet gap. */

  uint32_t                          ext_inter_pkt_gap;     /*!< Sets the Extended IPG between Packet
                                                                during transmission.
                                                                This parameter can be a value from 0x0 to 0xFF */

  hal_eth_mac_prog_wd_ctrl_t        programmable_wd;       /*!< Enable or disables the Programmable Watchdog. */

  hal_eth_mac_rx_wd_timeout_t       rx_wd_timeout_byte;    /*!< This field is used as watchdog timeout for a received
                                                                packet */

  uint32_t                          tx_pause_time;         /*!< This field holds the value to be used in
                                                                the Pause Time field in the transmit
                                                                control packet.
                                                                This parameter must be a number between
                                                                Min_Data = 0x0 and Max_Data = 0xFFFF.*/

  hal_eth_mac_zero_q_pause_ctrl_t   zero_quanta_pause;     /*!< Enable or disables the automatic generation of Zero
                                                                Quanta Pause Control packets. */

  hal_eth_mac_pause_low_thr_t       pause_low_threshold;   /*!< This field configures the threshold of the PAUSE to be
                                                                checked for automatic retransmission of PAUSE Packet */

  hal_eth_mac_tr_flow_ctrl_t        tr_flow_ctrl;          /*!< Enables or disables the MAC to transmit
                                                                Pause packets in Full Duplex mode
                                                                or the MAC back pressure operation in Half Duplex
                                                                mode */

  hal_eth_mac_uc_pause_pkt_ctrl_t   uc_pause_pkt_detect;   /*!< Enables or disables the MAC to detect Pause packets
                                                                with unicast address of the station */

  hal_eth_mac_receive_flow_ctrl_t   receive_flow_ctrl;     /*!< Enables or disables the MAC to decodes
                                                                the received Pause packet
                                                                and disables its transmitter for a specified (Pause)
                                                                time */
} hal_eth_mac_config_t;

/**
  * @brief  ETH DMA Configuration Structure definition
  */
typedef struct
{
  hal_eth_dma_addr_align_ctrl_t     addr_aligned_beats;    /*!< Enables or disables the Peripheral Bus
                                                                Master interface address aligned burst transfers
                                                                on Read and Write channels  */

  hal_eth_dma_burst_len_mode_t      burst_mode;            /*!< Sets the Peripheral Bus Master interface burst
                                                                transfers length mode */
  hal_eth_dma_mixed_burst_ctrl_t    mixed_burst;           /*!< Sets the maximum outstanding request on the System Bus
                                                                read interface */

  hal_eth_dma_rebuild_inc_ctrl_t    rebuild_inc_burst;     /*!< Sets the maximum outstanding request on the System Bus
                                                                read interface */

  hal_eth_dma_tr_prio_ctrl_t        tr_priority;           /*!< Sets Transmit priority over Receive */

} hal_eth_dma_config_t;

/**
  * @brief  ETH DMA Tx Configuration Structure definition
  */
typedef struct
{
  hal_eth_dma_tx_pbl_x8_mode_ctrl_t tx_pbl_x8_mode;        /*!< Enables or disables the PBL multiplication by eight. */

  hal_eth_dma_tx_burst_length_t     tx_dma_burst_length;   /*!< Indicates the maximum number of beats to be transferred
                                                                in one Tx DMA transaction */

  hal_eth_dma_tx_sec_pkt_op_ctrl_t  tx_second_pkt_operate; /*!< Enables or disables the Operate on second
                                                                Packet mode, which allows the DMA to
                                                                process a second
                                                                Packet of Transmit data even before obtaining the
                                                                status for the first one. */

} hal_eth_dma_tx_channel_config_t;

/**
  * @brief  ETH DMA Rx Configuration Structure definition
  */
typedef struct
{
  hal_eth_dma_rx_burst_length_t     rx_dma_burst_length;   /*!< Indicates the maximum number of beats to be transferred
                                                                in one Rx DMA transaction */

  uint32_t                          rx_buffer_len_byte;    /*!< Provides the length of Rx buffers size in byte unit */

} hal_eth_dma_rx_channel_config_t;

/**
  * @brief  ETH MTL Configuration Structure definition
  */
typedef struct
{
  hal_eth_mtl_tx_fwd_status_ctrl_t  tx_fwd_status;         /*!< Enables or disables forwarding Tx Packet Status to the
                                                                application. */

} hal_eth_mtl_config_t;

/**
  * @brief  ETH MTL Tx Queue Configuration Structure definition
  */
typedef struct
{
  hal_eth_mtl_tx_ops_mode_t         queue_op_mode;         /*!< Specifies the Tx Queue operating mode */

  hal_eth_mtl_tx_queue_size_t       queue_size_byte;       /*!< Specifies the Tx Queue Size */

  hal_eth_mtl_tx_transmit_mode_t    transmit_queue_mode;   /*!< Specifies the Transmit Queue operating mode */

} hal_eth_mtl_tx_queue_config_t;

/**
  * @brief  ETH MTL Rx Queue Configuration Structure definition
  */
typedef struct
{
  hal_eth_mtl_rx_ops_mode_t         queue_op_mode;         /*!< Specifies the Rx Queue operating mode */

  hal_eth_mtl_rx_queue_size_t       queue_size_byte;       /*!< Specifies the Rx Queue Size */

  hal_eth_mtl_rx_drop_cs_err_ctrl_t drop_tcp_ip_csum_error_pkt; /*!< Enables or disables Dropping of TCPIP Checksum
                                                                     Error Packets */

  hal_eth_mtl_rx_fwd_err_pkt_ctrl_t fwd_error_pkt;         /*!< Enables or disables  forwarding Error Packets. */

  hal_eth_mtl_rx_fwd_usz_pkt_ctrl_t fwd_undersized_good_pkt; /*!< Enables or disables  forwarding Undersized Good
                                                                  Packets.*/

  hal_eth_mtl_rx_queue_mode_t       receive_queue_mode;    /*!< Specifies the Receive Queue operating mode */
} hal_eth_mtl_rx_queue_config_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group8 ETH channels configuration structures
  * @{
  */
/**
  * @brief  ETH Tx Channel Configuration Structure definition
  */
typedef struct
{
  uint32_t                          max_app_buffers_num;   /*!< Provides the Maximum number of Application Buffers to
                                                                be hold in the DMA FIFO */

  uint32_t                          req_desc_size_align_byte; /*!< Provides the Application requested memory size
                                                                   alignment in bytes.
                                                                   The application can request specific memory size
                                                                   alignment to implement proper CACHE maintenance
                                                                   @see hal_eth_cache_cb_t callback.
                                                                   - If the requested size alignment is zero then the
                                                                   driver request the alignment according to its own
                                                                   constraints.
                                                                   - Otherwise, the driver will use size alignment
                                                                   value which respect both descriptor size alignment
                                                                   and requested size alignment conditions. */

  hal_eth_dma_tx_channel_config_t   dma_channel_config;    /*!< DMA Tx Channel Configuration */

  hal_eth_mtl_tx_queue_config_t     mtl_queue_config;      /*!< MTL Tx Queue Configuration */

  hal_eth_fifo_event_config_t       fifo_event_config;     /*!< FIFO event Configuration */

} hal_eth_tx_channel_config_t;

/**
  * @brief  ETH Rx Channel Configuration Structure definition
  */
typedef struct
{
  uint32_t                          max_app_buffers_num;   /*!< Provides the Maximum number of Application Buffers to
                                                                be hold in the DMA FIFO */

  uint32_t                          req_desc_size_align_byte; /*!< Provides the Application requested
                                                                   descriptor memory size alignment in
                                                                   bytes.
                                                                   The application can request specific descriptor size
                                                                   alignment to implement proper CACHE maintenance
                                                                   @see hal_eth_cache_cb_t callback.
                                                                   - If the requested size alignment is zero then the
                                                                   driver will not align the descriptor size.
                                                                   - Otherwise, the driver will use size alignment
                                                                   value which respect both descriptor size alignment
                                                                   and requested size aligenement conditions. */

  hal_eth_dma_rx_channel_config_t   dma_channel_config;    /*!< DMA Rx Channel Configuration */

  hal_eth_mtl_rx_queue_config_t     mtl_queue_config;      /*!< MTL Rx Queue Configuration */

  hal_eth_fifo_event_config_t       fifo_event_config;     /*!< FIFO event Configuration */

} hal_eth_rx_channel_config_t;

/**
  * @brief  ETH Channel Memory Allocation Requirements Structure definition
  */
typedef struct
{
  uint32_t                          mem_size_byte;         /*!< Provides the minimal Size required by the driver */

  uint32_t                          mem_addr_align_byte;   /*!< Provides the minimum alignment for memory to be used by
                                                                the driver */

} hal_eth_channel_alloc_needs_t;

/**
  * @brief  ETH Buffers List structure definition
  */
typedef struct
{
  void                             *p_buffer;              /*!< The ETH Frame buffer memory */

  uint32_t                          len_byte;              /*!< The ETH Frame buffer length in bytes */

} hal_eth_buffer_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group9 ETH channels process and I/O structures
  * @{
  */
/**
  * @brief  Transmit Packet Configuration structure definition
  */
typedef struct
{
  uint32_t                          attributes;            /*!< Tx packet HW features capabilities.
                                                                This parameter can be a combination
                                                                of @ref ETH_Tx_Packet_Control_Attributes */

  hal_eth_tx_pkt_src_addr_ctrl_t    src_addr_ctrl;         /*!< Specifies the source address insertion control */

  hal_eth_tx_pkt_crc_pad_ctrl_t     crc_pad_ctrl;          /*!< Specifies the CRC and Pad insertion and replacement
                                                                control */

  hal_eth_tx_pkt_csum_ctrl_t        csum_ctrl;             /*!< Specifies the checksum insertion control */

  uint16_t                          vlan_tag_id;           /*!< Sets VLAN Tag ID only when VLAN is
                                                                enabled.
                                                                This parameter can be a value from 0x0 to 0xFFFF */

  hal_eth_tx_pkt_vlan_ctrl_t        vlan_ctrl;             /*!< Specifies VLAN Tag insertion control only when VLAN is
                                                                enabled */

  uint32_t                          inner_vlan_tag_id;     /*!< Sets Inner VLAN Tag ID only when Inner
                                                                VLAN is enabled.
                                                                This parameter can be a value from 0x0 to 0x3FFFF */

  hal_eth_tx_pkt_inner_vlan_ctrl_t  inner_vlan_ctrl;       /*!< Specifies Inner VLAN Tag insertion control only when
                                                                Inner VLAN is enabled */

  void                             *p_data;                /*!< Specifies Application packet pointer to save for
                                                                completion notification */

  hal_eth_tx_pkt_notify_ctrl_t      notify;                /*!< Enable or Disable notification for this packet transmit
                                                                request */

} hal_eth_tx_pkt_config_t;
/**
  * @}
  */

/** @defgroup ETH_Exported_Types_Group10 ETH peripheral power management structures
  * @{
  */
/**
  * @brief ETH Remote Wakeup Packet Filter structure definition.
  *
  * This structure defines a single remote wakeup packet filter used by the Ethernet peripheral.
  * Each filter can be configured to match specific packet patterns and control the filter logic
  * using a combination of mask, offset, CRC, and command bits.
  *
  * The 4-bit command field uses the following constants to control filter behavior:
  *   - @ref HAL_ETH_RWK_FLT_CMD_ENABLE         : Enable or disable the filter.
  *   - @ref HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS   : AND logic with the previous filter for extended pattern matching.
  *   - @ref HAL_ETH_RWK_FLT_CMD_INVERSE_MODE   : Inverse mode for CRC16 hash function (reject on match).
  *   - @ref HAL_ETH_RWK_FLT_CMD_MULTICAST      : Apply filter to multicast or unicast packets.
  *
  * @see HAL_ETH_RWK_FLT_CMD_ENABLE
  * @see HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  * @see HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  * @see HAL_ETH_RWK_FLT_CMD_MULTICAST
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
typedef struct
{
  /**
    * @brief Filter 32-bit Mask.
    *
    * Each bit in the mask corresponds to one byte in the detected packet.
    * - If a bit is set to 1, the corresponding byte is included in the CRC16 calculation.
    * - The MSB (31st bit) must be zero.
    * - Bits [30:0] represent the byte mask.
    * - If bit j is set, the CRC block processes the (offset + j)th byte of the incoming packet.
    *   Otherwise, that byte is ignored.
    */
  uint32_t                          byte_mask;

  /**
    * @brief Filter 4-bit command stored in bits [3:0].
    *
    * The 4-bit command controls the filter operation:
    * - Bit 3 (@ref HAL_ETH_RWK_FLT_CMD_MULTICAST): Address type. Set for multicast, reset for unicast.
    * - Bit 2 (@ref HAL_ETH_RWK_FLT_CMD_INVERSE_MODE): Inverse mode. Set to reject packets with matching CRC16.
    * - Bit 1 (@ref HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS): AND logic with previous filter for extended patterns.
    * - Bit 0 (@ref HAL_ETH_RWK_FLT_CMD_ENABLE): Enable filter. If not set, the filter is disabled.
    *
    * Use the defined constants to set or clear these bits.
    */
  uint8_t                           commands;

  /**
    * @brief Filter 8-bit Offset pattern.
    *
    * Defines the offset (within the packet) from which the filter examines the bytes.
    * - This 8-bit value is the offset for the filter's first byte to be examined.
    * - The minimum allowed offset is 12 (refers to the 13th byte of the packet).
    * - An offset value of 0 refers to the first byte of the packet.
    */
  uint8_t                           offsets;

  /**
    * @brief Filter CRC-16 value.
    *
    * Contains the CRC-16 value calculated from the pattern and the byte mask.
    * - The 16-bit CRC calculation uses the polynomial: G(x) = x^16 + x^15 + x^2 + 1.
    * - Each filter compares the computed CRC16 with this value for packet matching.
    * - The mask and offset together determine which bytes are used in the CRC16 calculation.
    */
  uint16_t                          crc16;

} hal_eth_rwk_pkt_filter_t;

/**
  * @brief ETH Remote Wakeup Packet Filter Block structure definition.
  *
  * This structure represents a block of remote wakeup packet filters for the Ethernet peripheral.
  * Each block contains a fixed number of filters, as defined by @ref HAL_ETH_RWK_FILT_PER_BLOCK.
  *
  * The hardware arranges remote wake-up filters in blocks, with each block containing four filters.
  * Each filter is described by @ref hal_eth_rwk_pkt_filter_t and can be individually configured.
  *
  * **Register Layout for Each Block:**
  * @code
  * | Register Name      | Description                                             |
  * |--------------------|--------------------------------------------------------|
  * | BYTE MASK (0)      | Filter #0 byte mask  [30:0]                            |
  * | BYTE MASK (1)      | Filter #1 byte mask  [30:0]                            |
  * | BYTE MASK (2)      | Filter #2 byte mask  [30:0]                            |
  * | BYTE MASK (3)      | Filter #3 byte mask  [30:0]                            |
  * | FILT_CMDS [0..3]   | FILT_3 CMD | FILT_2 CMD | FILT_1 CMD | FILT_0 CMD      |
  * | FILT_OFFSET [0..3] | FILT_3 OFFSET | FILT_2 OFFSET | FILT_1 OFFSET | FILT_0 OFFSET |
  * | FILT_CRC [1..0]    | FILT_1 CRC 16bits | FILT_0 CRC 16bits                  |
  * | FILT_CRC [3..2]    | FILT_3 CRC 16bits | FILT_2 CRC 16bits                  |
  * @endcode
  *
  * - The number of filters per block is defined by @ref HAL_ETH_RWK_FILT_PER_BLOCK.
  * - The maximum number of filter blocks is defined by @ref HAL_ETH_RWK_FILT_BLOCK_NUM.
  * - Each filter in the block uses the command constants:
  *   - @ref HAL_ETH_RWK_FLT_CMD_ENABLE
  *   - @ref HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  *   - @ref HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  *   - @ref HAL_ETH_RWK_FLT_CMD_MULTICAST
  *
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_RWK_FILT_PER_BLOCK
  * @see HAL_ETH_RWK_FILT_BLOCK_NUM
  * @see HAL_ETH_RWK_FLT_CMD_ENABLE
  * @see HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  * @see HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  * @see HAL_ETH_RWK_FLT_CMD_MULTICAST
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
typedef struct
{
  /**
    * @brief Array of remote wakeup packet filters in this block.
    *
    * Each filter is defined by @ref hal_eth_rwk_pkt_filter_t and can be individually configured.
    * The number of filters in the block is given by @ref HAL_ETH_RWK_FILT_PER_BLOCK.
    */
  hal_eth_rwk_pkt_filter_t          filter[HAL_ETH_RWK_FILT_PER_BLOCK];
} hal_eth_rwk_filter_block_t;

/**
  * @brief ETH Remote Wakeup Packet Filters Lookup Table (LUT) structure definition.
  *
  * This structure represents the lookup table (LUT) for all remote wakeup packet filter blocks
  * supported by the Ethernet peripheral. Each LUT contains one or more filter blocks, and each
  * block contains multiple filters.
  *
  * The number of filter blocks in the LUT is defined by @ref HAL_ETH_RWK_FILT_BLOCK_NUM.
  * Each block is represented by @ref hal_eth_rwk_filter_block_t, which contains
  * @ref HAL_ETH_RWK_FILT_PER_BLOCK filters of type @ref hal_eth_rwk_pkt_filter_t.
  *
  * **LUT Layout for N Blocks:**
  * @code
  * |--------------------------|-----------------------------------------------|
  * |      Filter Block #0     | Data for block 0 (see @ref hal_eth_rwk_filter_block_t) |
  * |--------------------------|-----------------------------------------------|
  * |      Filter Block #1     | Data for block 1                              |
  * |--------------------------|-----------------------------------------------|
  * |           ...            | ...                                           |
  * |--------------------------|-----------------------------------------------|
  * |      Filter Block #N     | Data for block N                              |
  * |--------------------------|-----------------------------------------------|
  * @endcode
  *
  * - The maximum number of filter blocks is defined by @ref HAL_ETH_RWK_FILT_BLOCK_NUM.
  * - The total number of filters supported is
  *   @ref HAL_ETH_RWK_FILT_PER_BLOCK * @ref HAL_ETH_RWK_FILT_BLOCK_NUM.
  * - Each filter uses the command constants:
  *   - @ref HAL_ETH_RWK_FLT_CMD_ENABLE
  *   - @ref HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  *   - @ref HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  *   - @ref HAL_ETH_RWK_FLT_CMD_MULTICAST
  *
  * @see hal_eth_rwk_filter_block_t
  * @see hal_eth_rwk_pkt_filter_t
  * @see HAL_ETH_RWK_FILT_BLOCK_NUM
  * @see HAL_ETH_RWK_FILT_PER_BLOCK
  * @see HAL_ETH_RWK_FLT_CMD_ENABLE
  * @see HAL_ETH_RWK_FLT_CMD_AND_PREVIOUS
  * @see HAL_ETH_RWK_FLT_CMD_INVERSE_MODE
  * @see HAL_ETH_RWK_FLT_CMD_MULTICAST
  * @see HAL_ETH_SetRemoteWakeUpPcktFilter
  */
typedef struct
{
  /**
    * @brief Filter block 0.
    *
    * Each block is defined by @ref hal_eth_rwk_filter_block_t and contains
    * @ref HAL_ETH_RWK_FILT_PER_BLOCK filters.
    *
    * @note For hardware supporting multiple blocks, additional block members
    *       (e.g., block1, block2, ...) must be added up to
    *       @ref HAL_ETH_RWK_FILT_BLOCK_NUM.
    */
  hal_eth_rwk_filter_block_t        block0;
} hal_eth_rwk_filter_lut_t;
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup ETH_Exported_Functions HAL ETH functions
  * @{
  */
/** @defgroup ETH_Exported_Functions_Group1 ETH initialization and deInitialization functions
  * @{
  */
hal_status_t HAL_ETH_Init(hal_eth_handle_t *heth, hal_eth_t instance);
void HAL_ETH_DeInit(hal_eth_handle_t *heth);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group2 ETH peripheral configuration functions
  * @{
  */
hal_status_t HAL_ETH_SetConfig(hal_eth_handle_t *heth, const hal_eth_config_t *p_config);
void HAL_ETH_GetConfig(const hal_eth_handle_t *heth, hal_eth_config_t *p_config);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group3 ETH peripheral sub-blocks configuration functions
  * @{
  */
void HAL_ETH_MAC_GetConfig(const hal_eth_handle_t *heth, hal_eth_mac_config_t *p_macconf);
hal_status_t HAL_ETH_MAC_SetConfig(hal_eth_handle_t *heth, const hal_eth_mac_config_t *p_macconf);
void HAL_ETH_DMA_GetConfig(const hal_eth_handle_t *heth, hal_eth_dma_config_t *p_dmaconf);
hal_status_t HAL_ETH_DMA_SetConfig(hal_eth_handle_t *heth, const hal_eth_dma_config_t *p_dmaconf);
void HAL_ETH_MTL_GetConfig(const hal_eth_handle_t *heth, hal_eth_mtl_config_t *p_mtlconf);
hal_status_t HAL_ETH_MTL_SetConfig(hal_eth_handle_t *heth, const hal_eth_mtl_config_t *p_mtlconf);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group4 ETH channels configuration functions
  * @{
  */
hal_status_t HAL_ETH_SetConfigTxChannel(hal_eth_handle_t *heth, uint32_t channel,
                                        const hal_eth_tx_channel_config_t *p_chconf);
hal_status_t HAL_ETH_SetConfigRxChannel(hal_eth_handle_t *heth, uint32_t channel,
                                        const hal_eth_rx_channel_config_t *p_chconf);
void HAL_ETH_GetConfigTxChannel(const hal_eth_handle_t *heth, uint32_t channel,
                                hal_eth_tx_channel_config_t *p_chconf);
void HAL_ETH_GetConfigRxChannel(const hal_eth_handle_t *heth, uint32_t channel,
                                hal_eth_rx_channel_config_t *p_chconf);
void HAL_ETH_GetChannelAllocNeeds(const hal_eth_handle_t *heth, uint32_t channel,
                                  hal_eth_channel_alloc_needs_t *p_ch_alloc_req);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group5 ETH peripheral optional control functions
  * @{
  */
hal_status_t                 HAL_ETH_UpdateConfigLink(hal_eth_handle_t *heth, const hal_eth_link_config_t *p_config);
void                         HAL_ETH_EnableARPOffload(hal_eth_handle_t *heth);
void                         HAL_ETH_DisableARPOffload(hal_eth_handle_t *heth);
hal_eth_arp_offload_status_t HAL_ETH_IsEnabledARPOffload(const hal_eth_handle_t *heth);
void                         HAL_ETH_SetARPTargetIP(hal_eth_handle_t *heth, uint32_t tpa);
hal_status_t                 HAL_ETH_EnterPowerDownMode(hal_eth_handle_t *heth, uint32_t pmt_ctrl);
hal_status_t                 HAL_ETH_ExitPowerDownMode(hal_eth_handle_t *heth);
hal_status_t                 HAL_ETH_SetRemoteWakeUpPcktFilter(const hal_eth_handle_t *heth,
                                                               const hal_eth_rwk_filter_lut_t *p_filter_lut);
void                         HAL_ETH_EnterLPIMode(hal_eth_handle_t *heth, uint32_t lpi_ctrl);
void                         HAL_ETH_ExitLPIMode(hal_eth_handle_t *heth);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group6 callbacks register functions
  * @{
  */
hal_status_t HAL_ETH_RegisterChannelRxAllocateCallback(hal_eth_handle_t *heth, uint32_t channel,
                                                       hal_eth_rx_allocate_cb_t p_callback);
hal_status_t HAL_ETH_RegisterChannelRxCptCallback(hal_eth_handle_t *heth, uint32_t channel,
                                                  hal_eth_rx_complete_cb_t p_callback);
hal_status_t HAL_ETH_RegisterChannelTxCptCallback(hal_eth_handle_t *heth, uint32_t channel,
                                                  hal_eth_tx_complete_cb_t p_callback);
#if defined (USE_HAL_ETH_REGISTER_CALLBACKS) && (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
hal_status_t HAL_ETH_RegisterDataCallback(hal_eth_handle_t *heth, hal_eth_cb_t p_callback);
hal_status_t HAL_ETH_RegisterWKUPCallback(hal_eth_handle_t *heth, hal_eth_wakeup_cb_t p_callback);
hal_status_t HAL_ETH_RegisterPMTCallback(hal_eth_handle_t *heth, hal_eth_cb_t p_callback);
hal_status_t HAL_ETH_RegisterEEECallback(hal_eth_handle_t *heth, hal_eth_cb_t p_callback);
hal_status_t HAL_ETH_RegisterErrorCallback(hal_eth_handle_t *heth, hal_eth_cb_t p_callback);
hal_status_t HAL_ETH_RegisterEventCallback(hal_eth_handle_t *heth, hal_eth_cb_t p_callback);
hal_status_t HAL_ETH_RegisterCacheInvalidateCallback(hal_eth_handle_t *heth, hal_eth_cache_cb_t p_callback);
hal_status_t HAL_ETH_RegisterCacheFlushCallback(hal_eth_handle_t *heth, hal_eth_cache_cb_t p_callback);
hal_status_t HAL_ETH_RegisterChannelRxEventCallback(hal_eth_handle_t *heth, uint32_t channel,
                                                    hal_eth_channel_cb_t p_callback);
hal_status_t HAL_ETH_RegisterChannelTxEventCallback(hal_eth_handle_t *heth, uint32_t channel,
                                                    hal_eth_channel_cb_t p_callback);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group7 interrupts management functions
  * @{
  */
void         HAL_ETH_IRQHandler(hal_eth_handle_t *heth);
void         HAL_ETH_WKUP_IRQHandler(const hal_eth_handle_t *heth);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group8 Weak callback functions
  * @{
  */
void         HAL_ETH_DataCallback(hal_eth_handle_t *heth, uint32_t channels_mask);
void         HAL_ETH_ErrorCallback(hal_eth_handle_t *heth, uint32_t errors);
void         HAL_ETH_EventCallback(hal_eth_handle_t *heth, uint32_t events);
void         HAL_ETH_PMTCallback(hal_eth_handle_t *heth, uint32_t wake_up_event);
void         HAL_ETH_EEECallback(hal_eth_handle_t *heth, uint32_t lpi_event);
void         HAL_ETH_WakeUpCallback(const hal_eth_handle_t *heth);
void         HAL_ETH_CacheInvalidateCallback(hal_eth_handle_t *heth, uint32_t channel, void *p_addr,  uint32_t size);
void         HAL_ETH_CacheFlushCallback(hal_eth_handle_t *heth, uint32_t channel, void *p_addr,  uint32_t size);
void         HAL_ETH_TxEventCallback(hal_eth_handle_t *heth, uint32_t channel, uint32_t events);
void         HAL_ETH_RxEventCallback(hal_eth_handle_t *heth, uint32_t channel, uint32_t events);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group9 Process and I/O operations functions
  * @{
  */
/* ETH Peripheral I/O functions  **********************************************/
hal_status_t HAL_ETH_ExecDataHandler(hal_eth_handle_t *heth, uint32_t input_channel_mask,
                                     uint32_t *p_output_channel_mask);
/* ETH Channels I/O functions  **********************************************/
hal_status_t HAL_ETH_RequestTx(hal_eth_handle_t *heth, uint32_t channel, hal_eth_buffer_t *p_buff_array,
                               uint32_t buf_count, hal_eth_tx_pkt_config_t *p_tx_conf);
hal_status_t HAL_ETH_StartChannel(hal_eth_handle_t *heth, uint32_t channel, uint32_t *p_desc_mem,
                                  uint32_t desc_size_byte);
hal_status_t HAL_ETH_StopChannel(hal_eth_handle_t *heth, uint32_t channel);
hal_status_t HAL_ETH_SuspendChannel(hal_eth_handle_t *heth, uint32_t channel);
hal_status_t HAL_ETH_ResumeChannel(hal_eth_handle_t *heth, uint32_t channel);
uint32_t     HAL_ETH_GetChannelBufferInUseCount(const hal_eth_handle_t *heth, uint32_t channel);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group10 Multi-Queue functions
  * @{
  */
/* Multi-Queue functions *********************************************/
uint32_t     HAL_ETH_GetRxDMAChNumber(const hal_eth_handle_t *heth);
uint32_t     HAL_ETH_GetTxDMAChNumber(const hal_eth_handle_t *heth);
uint32_t     HAL_ETH_GetRxMTLQNumber(const hal_eth_handle_t *heth);
uint32_t     HAL_ETH_GetTxMTLQNumber(const hal_eth_handle_t *heth);
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group11 state and error functions
  * @{
  */
/* Peripheral State functions  ************************************************/
hal_eth_state_t          HAL_ETH_GetState(const hal_eth_handle_t *heth);
/* Channels State and Error functions  ****************************************/
hal_eth_channel_state_t  HAL_ETH_GetChannelState(const hal_eth_handle_t *heth, uint32_t channel);
#if defined (USE_HAL_ETH_GET_LAST_ERRORS) && (USE_HAL_ETH_GET_LAST_ERRORS == 1)
uint32_t                 HAL_ETH_GetLastErrorCodes(const hal_eth_handle_t *heth);
#endif /* USE_HAL_ETH_GET_LAST_ERRORS */
/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group12 MDIO control and PHY I/O operations functions
  * @{
  */
void         HAL_ETH_MDIO_UpdateClockRange(hal_eth_handle_t *heth);
void         HAL_ETH_MDIO_SetOpAttributes(hal_eth_handle_t *heth, uint32_t cmd_attributes);
hal_status_t HAL_ETH_MDIO_C22WriteData(const hal_eth_handle_t *heth, uint8_t phy_dev_addr, uint8_t reg_addr,
                                       uint16_t data);
hal_status_t HAL_ETH_MDIO_C22ReadData(hal_eth_handle_t *heth, uint8_t phy_dev_addr, uint8_t reg_addr,
                                      uint16_t *p_data);
hal_status_t HAL_ETH_MDIO_C45WriteData(const hal_eth_handle_t *heth, uint8_t phy_addr, uint8_t dev_addr,
                                       uint16_t reg_addr,
                                       uint16_t data);
hal_status_t HAL_ETH_MDIO_C45ReadData(hal_eth_handle_t *heth, uint8_t phy_addr, uint8_t dev_addr, uint16_t reg_addr,
                                      uint16_t *p_data);
hal_status_t HAL_ETH_MDIO_C45ReadDataRange(hal_eth_handle_t *heth, uint8_t phy_addr, uint8_t dev_addr,
                                           uint16_t start_reg_addr,
                                           uint16_t *p_data, uint16_t count);
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup ETH_Exported_Functions_Group13 bus operation functions
  * @{
  */
hal_status_t HAL_ETH_AcquireBus(hal_eth_handle_t *heth, uint32_t timeout_ms);
hal_status_t HAL_ETH_ReleaseBus(hal_eth_handle_t *heth);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_ETH_USER_DATA) && (USE_HAL_ETH_USER_DATA == 1)
/** @defgroup ETH_Exported_Functions_Group14 user data functions
  * @{
  */
void        HAL_ETH_SetUserData(hal_eth_handle_t *heth, const void *p_user_data);
const void *HAL_ETH_GetUserData(const hal_eth_handle_t *heth);
/**
  * @}
  */
#endif /* USE_HAL_ETH_USER_DATA */

/** @defgroup ETH_Exported_Functions_Group15 ETH external WAKEUP management functions
  * @{
  */
/**
  * @brief  Enable the Ethernet Wakeup External interrupt.
  *
  * This function enables the interrupt request for the Ethernet Wakeup External line
  * by setting the corresponding bit in the External Interrupt Mask Register 2 (IMR2).
  * It uses an atomic operation to ensure thread safety.
  *
  * @note   This function is typically used to allow the Ethernet peripheral to
  *         wake up the MCU from low-power modes via an external interrupt.
  *
  * @note   To disable the interrupt, use @ref HAL_ETH_WAKEUP_DisableIT.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_EnableIT(void)
{
  LL_EXTI_EnableIT_32_63(ETH_WAKEUP_EXTI_LINE);
}

/**
  * @brief  Disable the Ethernet Wakeup External interrupt.
  *
  * This function disables the interrupt request for the Ethernet Wakeup External line
  * by clearing the corresponding bit in the External Interrupt Mask Register 2 (IMR2).
  *
  * @note   To enable the interrupt, use @ref HAL_ETH_WAKEUP_EnableIT.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_DisableIT(void)
{
  LL_EXTI_DisableIT_32_63(ETH_WAKEUP_EXTI_LINE);
}

/**
  * @brief  Get the interrupt pending bit for the Ethernet Wakeup External line.
  *
  * This function checks the External Rising Pending Register 2 (RPR2) and
  * Falling Pending Register 2 (FPR2) to determine which edge(s) have
  * triggered a pending interrupt for the Ethernet Wakeup External line.
  *
  * @note   The function returns one of the following values:
  *         - HAL_ETH_WAKEUP_TRIGGER_NONE: No pending trigger.
  *         - HAL_ETH_WAKEUP_TRIGGER_RISING: Rising edge trigger is pending.
  *         - HAL_ETH_WAKEUP_TRIGGER_FALLING: Falling edge trigger is pending.
  *         - HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING: Both rising and falling edge triggers are pending.
  *
  * @retval hal_eth_wakeup_trigger_t
  *         The pending trigger flag(s) for the Ethernet Wakeup External line.
  */
__STATIC_INLINE hal_eth_wakeup_trigger_t HAL_ETH_WAKEUP_GetPendingIT(void)
{
  hal_eth_wakeup_trigger_t pending_edge = HAL_ETH_WAKEUP_TRIGGER_NONE;

  if (LL_EXTI_IsActiveRisingFlag_32_63(ETH_WAKEUP_EXTI_LINE) != 0UL)
  {
    /* ETH EXTI Rising edge trigger is pending */
    pending_edge = HAL_ETH_WAKEUP_TRIGGER_RISING;
  }

  if (LL_EXTI_IsActiveFallingFlag_32_63(ETH_WAKEUP_EXTI_LINE) != 0UL)
  {
    if (pending_edge == HAL_ETH_WAKEUP_TRIGGER_RISING)
    {
      pending_edge = HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING;
    }
    else
    {
      pending_edge = HAL_ETH_WAKEUP_TRIGGER_FALLING;
    }
  }
  return pending_edge;
}

/**
  * @brief  Clear the pending interrupt flag(s) for the Ethernet Wakeup External line.
  *
  * This function clears the pending interrupt flag(s) for the Ethernet Wakeup External line
  * based on the specified edge(s). It writes to the External Rising Pending Register 2 (RPR2)
  * and/or Falling Pending Register 2 (FPR2) to clear the corresponding pending bits.
  *
  * @param  edge  Specifies which edge(s) to clear pending flags for.
  *               This parameter can be one or a combination of the following values:
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING:   Clear rising edge pending flag.
  *               - HAL_ETH_WAKEUP_TRIGGER_FALLING:  Clear falling edge pending flag.
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING: Clear both rising and falling edge pending flags.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_ClearPendingIT(hal_eth_wakeup_trigger_t edge)
{
  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_RISING) != 0UL)
  {
    /* Clear rising edge trigger pending bit */
    LL_EXTI_ClearRisingFlag_32_63(ETH_WAKEUP_EXTI_LINE);
  }

  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_FALLING) != 0UL)
  {
    /* Clear falling edge trigger pending bit */
    LL_EXTI_ClearFallingFlag_32_63(ETH_WAKEUP_EXTI_LINE);
  }
}

/**
  * @brief  Enable trigger edge(s) for the Ethernet Wakeup External line.
  *
  * This function enables the specified edge trigger(s) for the Ethernet Wakeup External line
  * by setting the corresponding bits in the External Rising Trigger Selection Register 2 (RTSR2)
  * and/or Falling Trigger Selection Register 2 (FTSR2). It also clears the rising edge pending
  * bit when enabling the rising edge trigger.
  *
  * @param  edge  Specifies which edge(s) to enable as triggers.
  *               This parameter can be one or a combination of the following values:
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING:   Enable rising edge trigger.
  *               - HAL_ETH_WAKEUP_TRIGGER_FALLING:  Enable falling edge trigger.
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING: Enable both rising and falling edge triggers.
  *
  * @note   To disable trigger edge(s), use @ref HAL_ETH_WAKEUP_DisableTrigger.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_EnableTrigger(hal_eth_wakeup_trigger_t edge)
{
  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_RISING) != 0UL)
  {
    /* Enable rising edge trigger bit */
    LL_EXTI_EnableRisingTrig_32_63(ETH_WAKEUP_EXTI_LINE);
  }

  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_FALLING) != 0UL)
  {
    /* Enable falling edge trigger bit */
    LL_EXTI_EnableFallingTrig_32_63(ETH_WAKEUP_EXTI_LINE);
  }
}

/**
  * @brief  Disable trigger edge(s) for the Ethernet Wakeup External line.
  *
  * This function disables the specified edge trigger(s) for the Ethernet Wakeup External line
  * by clearing the corresponding bits in the External Rising Trigger Selection Register 2 (RTSR2)
  * and/or Falling Trigger Selection Register 2 (FTSR2).
  *
  * @param  edge  Specifies which edge(s) to disable as triggers.
  *               This parameter can be one or a combination of the following values:
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING:   Disable rising edge trigger.
  *               - HAL_ETH_WAKEUP_TRIGGER_FALLING:  Disable falling edge trigger.
  *               - HAL_ETH_WAKEUP_TRIGGER_RISING_FALLING: Disable both rising and falling edge triggers.
  *
  * @note   To enable trigger edge(s), use @ref HAL_ETH_WAKEUP_EnableTrigger.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_DisableTrigger(hal_eth_wakeup_trigger_t edge)
{
  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_RISING) != 0UL)
  {
    /* Disable rising edge trigger bit */
    LL_EXTI_DisableRisingTrig_32_63(ETH_WAKEUP_EXTI_LINE);
  }

  if (((uint32_t)edge & (uint32_t)HAL_ETH_WAKEUP_TRIGGER_FALLING) != 0UL)
  {
    /* Disable falling edge trigger bit */
    LL_EXTI_DisableFallingTrig_32_63(ETH_WAKEUP_EXTI_LINE);
  }
}

/**
  * @brief  Generate a software interrupt for the Ethernet Wakeup External line.
  *
  * This function generates a software interrupt request for the Ethernet Wakeup External line
  * by setting the corresponding bit in the External Software Interrupt Event Register 2 (SWIER2).
  * This can be used to simulate an external interrupt event in software, which is useful for
  * testing or triggering interrupt handlers manually.
  *
  * @note   The software interrupt will be handled in the same way as a hardware-generated
  *         interrupt for the specified External line.
  */
__STATIC_INLINE void HAL_ETH_WAKEUP_GenerateSWIT(void)
{
  LL_EXTI_GenerateSWI_32_63(ETH_WAKEUP_EXTI_LINE);
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

#endif /* ETH1 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_ETH_H */
