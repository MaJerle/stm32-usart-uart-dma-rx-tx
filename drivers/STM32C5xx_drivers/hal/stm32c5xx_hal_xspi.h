/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_xspi.h
  * @brief   Header file of XSPI HAL module.
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
#ifndef STM32C5XX_HAL_XSPI_H
#define STM32C5XX_HAL_XSPI_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"

#if defined(XSPI1)

#include "stm32c5xx_dlyb_core.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup XSPI XSPI
  * @{
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup XSPI_Exported_Constants HAL XSPI Constants
  * @{
  */


/** @defgroup XSPI_Flag Flags
  * @{
  */
#define HAL_XSPI_FLAG_BUSY XSPI_SR_BUSY                                                        /*!< Busy flag: operation is ongoing                                                                          */
#define HAL_XSPI_FLAG_TO   XSPI_SR_TOF                                                         /*!< Timeout flag: timeout occurs in memory-mapped mode                                                       */
#if defined(XSPI_SR_SMF)
#define HAL_XSPI_FLAG_SM   XSPI_SR_SMF                                                         /*!< Status match flag: received data matches in auto-polling mode                                            */
#endif /* XSPI_SR_SMF */
#define HAL_XSPI_FLAG_FT   XSPI_SR_FTF                                                         /*!< FIFO threshold flag: FIFO threshold reached or data left after read from memory is complete              */
#define HAL_XSPI_FLAG_TC   XSPI_SR_TCF                                                         /*!< Transfer complete flag: programmed number of data have been transferred or the transfer has been aborted */
#define HAL_XSPI_FLAG_TE   XSPI_SR_TEF                                                         /*!< Transfer error flag: invalid address is being accessed                                                   */
#if defined(XSPI_SR_SMF)
#define HAL_XSPI_FLAG_ALL  XSPI_SR_TOF | XSPI_SR_SMF | XSPI_SR_FTF | XSPI_SR_TCF | XSPI_SR_TEF /*!< All flags                                                                                                */
#else
#define HAL_XSPI_FLAG_ALL  XSPI_SR_TOF | XSPI_SR_FTF | XSPI_SR_TCF | XSPI_SR_TEF               /*!< All flags                                                                                                */
#endif /* XSPI_SR_SMF */
/**
  * @}
  */

/** @defgroup XSPI_Interrupts Interrupts
  * @{
  */
#define HAL_XSPI_IT_TO  XSPI_CR_TOIE                                                             /*!< Interrupt on the timeout flag           */
#define HAL_XSPI_IT_SM  XSPI_CR_SMIE                                                             /*!< Interrupt on the status match flag      */
#define HAL_XSPI_IT_FT  XSPI_CR_FTIE                                                             /*!< Interrupt on the FIFO threshold flag    */
#define HAL_XSPI_IT_TC  XSPI_CR_TCIE                                                             /*!< Interrupt on the transfer complete flag */
#define HAL_XSPI_IT_TE  XSPI_CR_TEIE                                                             /*!< Interrupt on the transfer error flag    */
#if defined(XSPI_SR_SMF)
#define HAL_XSPI_IT_ALL XSPI_CR_TOIE | XSPI_CR_SMIE | XSPI_CR_FTIE | XSPI_CR_TCIE | XSPI_CR_TEIE /*!< All interrupts                          */
#else
#define HAL_XSPI_IT_ALL XSPI_CR_TOIE | XSPI_CR_FTIE | XSPI_CR_TCIE | XSPI_CR_TEIE /*!< All interrupts                          */
#endif /* XSPI_SR_SMF */
/**
  * @}
  */

/** @defgroup XSPI_Optional_Interrupt Optional interrupts
  * @{
  */
#define HAL_XSPI_OPT_IT_NONE    HAL_DMA_OPT_IT_NONE    /*!< DMA channel optional interrupts disabled    */
#define HAL_XSPI_OPT_IT_HT      HAL_DMA_OPT_IT_HT      /*!< DMA channel half transfer interrupt enabled */
#define HAL_XSPI_OPT_IT_DEFAULT HAL_DMA_OPT_IT_DEFAULT /*!< DMA channel all optional interrupts enabled */
/**
  * @}
  */

/** @defgroup XSPI_Error_Code Error Code definition reflecting the processes asynchronous errors
  * @{
  */
#if (defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U))
#define HAL_XSPI_ERROR_NONE      (0x00UL << 0U) /*!< XSPI error none     */
#define HAL_XSPI_ERROR_TRANSFER  (0x01UL << 0U) /*!< XSPI transfer error */
#define HAL_XSPI_ERROR_TIMEOUT   (0x01UL << 2U) /*!< XSPI timeout error  */
#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
#define HAL_XSPI_ERROR_DMA       (0x01UL << 1U) /*!< DMA transfer error  */
#endif /* USE_HAL_XSPI_DMA */
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/

/** @defgroup XSPI_Exported_Types HAL XSPI Types
  * @{
  */

/**
  * @brief HAL XSPI instance enumeration definition.
  */
typedef enum
{
  HAL_XSPI1    = XSPI1_R_BASE,    /*!< HAL XSPI instance 1 */
} hal_xspi_t;

/**
  * @brief HAL XSPI state enumeration definition.
  */
typedef enum
{
  HAL_XSPI_STATE_RESET                = 0U,           /*!< Reset state                                  */
  HAL_XSPI_STATE_INIT                 = (1U  << 31U), /*!< XSPI is initialized but not yet configured   */
  HAL_XSPI_STATE_IDLE                 = (1U  << 30U), /*!< XSPI initialized and a global config applied */
  HAL_XSPI_STATE_CMD_ACTIVE           = (1U  << 29U), /*!< Command ongoing                              */
  HAL_XSPI_STATE_AUTO_POLLING_ACTIVE  = (1U  << 28U), /*!< Auto-polling ongoing                         */
  HAL_XSPI_STATE_TX_ACTIVE            = (1U  << 27U), /*!< Indirect Tx ongoing                          */
  HAL_XSPI_STATE_RX_ACTIVE            = (1U  << 26U), /*!< Indirect Rx ongoing                          */
  HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE = (1U  << 25U), /*!< Memory-mapped ongoing                        */
  HAL_XSPI_STATE_ABORT                = (1U  << 24U), /*!< Abort ongoing                                */
} hal_xspi_state_t;

/**
  * @brief HAL XSPI DLYB State enumeration definition.
  */
typedef enum
{
  HAL_XSPI_DLYB_DISABLED = DLYB_DISABLED, /*!< XSPI DLYB disabled */
  HAL_XSPI_DLYB_ENABLED  = DLYB_ENABLED   /*!< XSPI DLYB enabled  */
} hal_xspi_dlyb_status_t;

/**
  * @brief HAL XSPI flag state enumeration definition.
  */
typedef enum
{
  HAL_XSPI_FLAG_NOT_ACTIVE = 0U,      /*!< Flag not active */
  HAL_XSPI_FLAG_ACTIVE                /*!< Flag active     */
} hal_xspi_flag_status_t;

/**
  * @brief HAL XSPI Memory Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_MEMORY_SINGLE = 0U,          /*!< Dual memory mode disabled */
  HAL_XSPI_MEMORY_DUAL   = XSPI_CR_DMM  /*!< Dual memory mode enabled  */
} hal_xspi_memory_mode_t;

/**
  * @brief HAL XSPI Memory Type enumeration definition.
  */
typedef enum
{
  HAL_XSPI_MEMORY_TYPE_MICRON       = 0U,                                    /*!< Micron mode       */
  HAL_XSPI_MEMORY_TYPE_MACRONIX     = XSPI_DCR1_MTYP_0,                      /*!< Macronix mode     */
  HAL_XSPI_MEMORY_TYPE_APMEM        = XSPI_DCR1_MTYP_1,                      /*!< AP Memory mode    */
  HAL_XSPI_MEMORY_TYPE_MACRONIX_RAM = (XSPI_DCR1_MTYP_1 | XSPI_DCR1_MTYP_0), /*!< Macronix RAM mode */
#if defined(XSPI_DCR1_MTYP_2)
  HAL_XSPI_MEMORY_TYPE_HYPERBUS     = XSPI_DCR1_MTYP_2,                      /*!< Hyperbus mode     */
#endif /* XSPI_DCR1_MTYP_2 */
} hal_xspi_memory_type_t;

/**
  * @brief HAL XSPI Memory Size enumeration definition.
  */
typedef enum
{
  HAL_XSPI_MEMORY_SIZE_16BIT   = (0x00U << XSPI_DCR1_DEVSIZE_Pos), /*!<  16 bits  (  2  Bytes = 2^( 0+1)) */
  HAL_XSPI_MEMORY_SIZE_32BIT   = (0x01U << XSPI_DCR1_DEVSIZE_Pos), /*!<  32 bits  (  4  Bytes = 2^( 1+1)) */
  HAL_XSPI_MEMORY_SIZE_64BIT   = (0x02U << XSPI_DCR1_DEVSIZE_Pos), /*!<  64 bits  (  8  Bytes = 2^( 2+1)) */
  HAL_XSPI_MEMORY_SIZE_128BIT  = (0x03U << XSPI_DCR1_DEVSIZE_Pos), /*!< 128 bits  ( 16  Bytes = 2^( 3+1)) */
  HAL_XSPI_MEMORY_SIZE_256BIT  = (0x04U << XSPI_DCR1_DEVSIZE_Pos), /*!< 256 bits  ( 32  Bytes = 2^( 4+1)) */
  HAL_XSPI_MEMORY_SIZE_512BIT  = (0x05U << XSPI_DCR1_DEVSIZE_Pos), /*!< 512 bits  ( 64  Bytes = 2^( 5+1)) */
  HAL_XSPI_MEMORY_SIZE_1KBIT   = (0x06U << XSPI_DCR1_DEVSIZE_Pos), /*!<   1 Kbits (128  Bytes = 2^( 6+1)) */
  HAL_XSPI_MEMORY_SIZE_2KBIT   = (0x07U << XSPI_DCR1_DEVSIZE_Pos), /*!<   2 Kbits (256  Bytes = 2^( 7+1)) */
  HAL_XSPI_MEMORY_SIZE_4KBIT   = (0x08U << XSPI_DCR1_DEVSIZE_Pos), /*!<   4 Kbits (512  Bytes = 2^( 8+1)) */
  HAL_XSPI_MEMORY_SIZE_8KBIT   = (0x09U << XSPI_DCR1_DEVSIZE_Pos), /*!<   8 Kbits (  1 KBytes = 2^( 9+1)) */
  HAL_XSPI_MEMORY_SIZE_16KBIT  = (0x0AU << XSPI_DCR1_DEVSIZE_Pos), /*!<  16 Kbits (  2 KBytes = 2^(10+1)) */
  HAL_XSPI_MEMORY_SIZE_32KBIT  = (0x0BU << XSPI_DCR1_DEVSIZE_Pos), /*!<  32 Kbits (  4 KBytes = 2^(11+1)) */
  HAL_XSPI_MEMORY_SIZE_64KBIT  = (0x0CU << XSPI_DCR1_DEVSIZE_Pos), /*!<  64 Kbits (  8 KBytes = 2^(12+1)) */
  HAL_XSPI_MEMORY_SIZE_128KBIT = (0x0DU << XSPI_DCR1_DEVSIZE_Pos), /*!< 128 Kbits ( 16 KBytes = 2^(13+1)) */
  HAL_XSPI_MEMORY_SIZE_256KBIT = (0x0EU << XSPI_DCR1_DEVSIZE_Pos), /*!< 256 Kbits ( 32 KBytes = 2^(14+1)) */
  HAL_XSPI_MEMORY_SIZE_512KBIT = (0x0FU << XSPI_DCR1_DEVSIZE_Pos), /*!< 512 Kbits ( 64 KBytes = 2^(15+1)) */
  HAL_XSPI_MEMORY_SIZE_1MBIT   = (0x10U << XSPI_DCR1_DEVSIZE_Pos), /*!<   1 Mbits (128 KBytes = 2^(16+1)) */
  HAL_XSPI_MEMORY_SIZE_2MBIT   = (0x11U << XSPI_DCR1_DEVSIZE_Pos), /*!<   2 Mbits (256 KBytes = 2^(17+1)) */
  HAL_XSPI_MEMORY_SIZE_4MBIT   = (0x12U << XSPI_DCR1_DEVSIZE_Pos), /*!<   4 Mbits (512 KBytes = 2^(18+1)) */
  HAL_XSPI_MEMORY_SIZE_8MBIT   = (0x13U << XSPI_DCR1_DEVSIZE_Pos), /*!<   8 Mbits (  1 MBytes = 2^(19+1)) */
  HAL_XSPI_MEMORY_SIZE_16MBIT  = (0x14U << XSPI_DCR1_DEVSIZE_Pos), /*!<  16 Mbits (  2 MBytes = 2^(20+1)) */
  HAL_XSPI_MEMORY_SIZE_32MBIT  = (0x15U << XSPI_DCR1_DEVSIZE_Pos), /*!<  32 Mbits (  4 MBytes = 2^(21+1)) */
  HAL_XSPI_MEMORY_SIZE_64MBIT  = (0x16U << XSPI_DCR1_DEVSIZE_Pos), /*!<  64 Mbits (  8 MBytes = 2^(22+1)) */
  HAL_XSPI_MEMORY_SIZE_128MBIT = (0x17U << XSPI_DCR1_DEVSIZE_Pos), /*!< 128 Mbits ( 16 MBytes = 2^(23+1)) */
  HAL_XSPI_MEMORY_SIZE_256MBIT = (0x18U << XSPI_DCR1_DEVSIZE_Pos), /*!< 256 Mbits ( 32 MBytes = 2^(24+1)) */
  HAL_XSPI_MEMORY_SIZE_512MBIT = (0x19U << XSPI_DCR1_DEVSIZE_Pos), /*!< 512 Mbits ( 64 MBytes = 2^(25+1)) */
  HAL_XSPI_MEMORY_SIZE_1GBIT   = (0x1AU << XSPI_DCR1_DEVSIZE_Pos), /*!<   1 Gbits (128 MBytes = 2^(26+1)) */
  HAL_XSPI_MEMORY_SIZE_2GBIT   = (0x1BU << XSPI_DCR1_DEVSIZE_Pos), /*!<   2 Gbits (256 MBytes = 2^(27+1)) */
  HAL_XSPI_MEMORY_SIZE_4GBIT   = (0x1CU << XSPI_DCR1_DEVSIZE_Pos), /*!<   4 Gbits (512 MBytes = 2^(28+1)) */
  HAL_XSPI_MEMORY_SIZE_8GBIT   = (0x1DU << XSPI_DCR1_DEVSIZE_Pos), /*!<   8 Gbits (  1 GBytes = 2^(29+1)) */
  HAL_XSPI_MEMORY_SIZE_16GBIT  = (0x1EU << XSPI_DCR1_DEVSIZE_Pos), /*!<  16 Gbits (  2 GBytes = 2^(30+1)) */
  HAL_XSPI_MEMORY_SIZE_32GBIT  = (0x1FU << XSPI_DCR1_DEVSIZE_Pos)  /*!<  32 Gbits (  4 GBytes = 2^(31+1)) */
} hal_xspi_memory_size_t;

/**
  * @brief HAL XSPI Free Running Clock enumeration definition.
  */
typedef enum
{
  HAL_XSPI_FREE_RUNNING_CLK_DISABLED = 0U,            /*!< CLK is not free running          */
  HAL_XSPI_FREE_RUNNING_CLK_ENABLED  = XSPI_DCR1_FRCK /*!< CLK is always provided (running) */
} hal_xspi_free_running_clk_status_t;

/**
  * @brief HAL XSPI Prefetch Data enumeration definition.
  */
typedef enum
{
  HAL_XSPI_PREFETCH_DATA_ENABLED  = 0U,             /*!< Automatic prefetch for data is enabled */
  HAL_XSPI_PREFETCH_DATA_DISABLED = XSPI_CR_NOPREF  /*!< Automatic prefetch disabled            */
} hal_xspi_prefetch_data_status_t;


#if defined(XSPI_DCR2_WRAPSIZE)
/**
  * @brief HAL XSPI Wrap Size enumeration definition.
  */
typedef enum
{
  HAL_XSPI_WRAP_NOT_SUPPORTED = 0U,                                            /*!< wrapped reads are not supported by the memory   */
  HAL_XSPI_WRAP_16BYTE        = XSPI_DCR2_WRAPSIZE_1,                          /*!< external memory supports wrap size of 16 bytes  */
  HAL_XSPI_WRAP_32BYTE        = (XSPI_DCR2_WRAPSIZE_0 | XSPI_DCR2_WRAPSIZE_1), /*!< external memory supports wrap size of 32 bytes  */
  HAL_XSPI_WRAP_64BYTE        = XSPI_DCR2_WRAPSIZE_2,                          /*!< external memory supports wrap size of 64 bytes  */
  HAL_XSPI_WRAP_128BYTE       = (XSPI_DCR2_WRAPSIZE_0 | XSPI_DCR2_WRAPSIZE_2)  /*!< external memory supports wrap size of 128 bytes */
} hal_xspi_wrap_size_t;
#endif /* XSPI_DCR2_WRAPSIZE */

/**
  * @brief HAL XSPI Sample Shift enumeration definition.
  */
typedef enum
{
  HAL_XSPI_SAMPLE_SHIFT_NONE      = 0U,             /*!< No shift        */
  HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE = XSPI_TCR_SSHIFT /*!< 1/2 cycle shift */
} hal_xspi_sample_shift_t;

/**
  * @brief HAL XSPI Delay Hold Quarter Cycle enumeration definition.
  */
typedef enum
{
  HAL_XSPI_DELAY_HOLD_NONE       = 0U,           /*!< No Delay             */
  HAL_XSPI_DELAY_HOLD_QUARTCYCLE = XSPI_TCR_DHQC /*!< Delay Hold 1/4 cycle */
} hal_xspi_delay_hold_t;

#if defined(XSPI_DCR3_CSBOUND)
/**
  * @brief HAL XSPI Chip Select Boundary enumeration definition.
  */
typedef enum
{
  HAL_XSPI_CS_BOUNDARY_NONE    = 0x00U, /*!< CS boundary disabled           */
  HAL_XSPI_CS_BOUNDARY_16BIT   = 0x01U, /*!<  16 bits  (  2  Bytes = 2^(1))  */
  HAL_XSPI_CS_BOUNDARY_32BIT   = 0x02U, /*!<  32 bits  (  4  Bytes = 2^(2))  */
  HAL_XSPI_CS_BOUNDARY_64BIT   = 0x03U, /*!<  64 bits  (  8  Bytes = 2^(3))  */
  HAL_XSPI_CS_BOUNDARY_128BIT  = 0x04U, /*!< 128 bits  ( 16  Bytes = 2^(4))  */
  HAL_XSPI_CS_BOUNDARY_256BIT  = 0x05U, /*!< 256 bits  ( 32  Bytes = 2^(5))  */
  HAL_XSPI_CS_BOUNDARY_512BIT  = 0x06U, /*!< 512 bits  ( 64  Bytes = 2^(6))  */
  HAL_XSPI_CS_BOUNDARY_1KBIT   = 0x07U, /*!<   1 Kbits (128  Bytes = 2^(7))  */
  HAL_XSPI_CS_BOUNDARY_2KBIT   = 0x08U, /*!<   2 Kbits (256  Bytes = 2^(8))  */
  HAL_XSPI_CS_BOUNDARY_4KBIT   = 0x09U, /*!<   4 Kbits (512  Bytes = 2^(9))  */
  HAL_XSPI_CS_BOUNDARY_8KBIT   = 0x0AU, /*!<   8 Kbits (  1 KBytes = 2^(10)) */
  HAL_XSPI_CS_BOUNDARY_16KBIT  = 0x0BU, /*!<  16 Kbits (  2 KBytes = 2^(11)) */
  HAL_XSPI_CS_BOUNDARY_32KBIT  = 0x0CU, /*!<  32 Kbits (  4 KBytes = 2^(12)) */
  HAL_XSPI_CS_BOUNDARY_64KBIT  = 0x0DU, /*!< 64 Kbits  (  8 KBytes = 2^(13)) */
  HAL_XSPI_CS_BOUNDARY_128KBIT = 0x0EU, /*!< 128 Kbits ( 16 KBytes = 2^(14)) */
  HAL_XSPI_CS_BOUNDARY_256KBIT = 0x0FU, /*!< 256 Kbits ( 32 KBytes = 2^(15)) */
  HAL_XSPI_CS_BOUNDARY_512KBIT = 0x10U, /*!< 512 Kbits ( 64 KBytes = 2^(16)) */
  HAL_XSPI_CS_BOUNDARY_1MBIT   = 0x11U, /*!<   1 Mbits (128 KBytes = 2^(17)) */
  HAL_XSPI_CS_BOUNDARY_2MBIT   = 0x12U, /*!<   2 Mbits (256 KBytes = 2^(18)) */
  HAL_XSPI_CS_BOUNDARY_4MBIT   = 0x13U, /*!<   4 Mbits (512 KBytes = 2^(19)) */
  HAL_XSPI_CS_BOUNDARY_8MBIT   = 0x14U, /*!<   8 Mbits (  1 MBytes = 2^(20)) */
  HAL_XSPI_CS_BOUNDARY_16MBIT  = 0x15U, /*!<  16 Mbits (  2 MBytes = 2^(21)) */
  HAL_XSPI_CS_BOUNDARY_32MBIT  = 0x16U, /*!<  32 Mbits (  4 MBytes = 2^(22)) */
  HAL_XSPI_CS_BOUNDARY_64MBIT  = 0x17U, /*!<  64 Mbits (  8 MBytes = 2^(23)) */
  HAL_XSPI_CS_BOUNDARY_128MBIT = 0x18U, /*!< 128 Mbits ( 16 MBytes = 2^(24)) */
  HAL_XSPI_CS_BOUNDARY_256MBIT = 0x19U, /*!< 256 Mbits ( 32 MBytes = 2^(25)) */
  HAL_XSPI_CS_BOUNDARY_512MBIT = 0x1AU, /*!< 512 Mbits ( 64 MBytes = 2^(26)) */
  HAL_XSPI_CS_BOUNDARY_1GBIT   = 0x1BU, /*!<   1 Gbits (128 MBytes = 2^(27)) */
  HAL_XSPI_CS_BOUNDARY_2GBIT   = 0x1CU, /*!<   2 Gbits (256 MBytes = 2^(28)) */
  HAL_XSPI_CS_BOUNDARY_4GBIT   = 0x1DU, /*!<   4 Gbits (512 MBytes = 2^(29)) */
  HAL_XSPI_CS_BOUNDARY_8GBIT   = 0x1EU, /*!<   8 Gbits (  1 GBytes = 2^(30)) */
  HAL_XSPI_CS_BOUNDARY_16GBIT  = 0x1FU  /*!<  16 Gbits (  2 GBytes = 2^(31)) */
} hal_xspi_cs_boundary_t;
#endif /* XSPI_DCR3_CSBOUND */

/**
  * @brief HAL XSPI Delay Block Bypass enumeration definition.
  */
typedef enum
{
  HAL_XSPI_DLYB_ON     = 0U,                 /*!< Sampling clock is delayed by the delay block */
  HAL_XSPI_DLYB_BYPASS = XSPI_DCR1_DLYBYP    /*!< Delay block is bypassed                      */
} hal_xspi_dlyb_state_t;

/**
  * @brief HAL XSPI Memory Select enumeration definition.
  */
typedef enum
{
  HAL_XSPI_MEMORY_SELECTION_NCS1 = 0U,           /*!<  The output of nCS is nCS1       */
  HAL_XSPI_MEMORY_SELECTION_NCS2 = XSPI_CR_CSSEL /*!<  The output of nCS is nCS2       */
} hal_xspi_memory_selection_t;

/**
  * @brief HAL XSPI Operation Type enumeration definition.
  */
typedef enum
{
  HAL_XSPI_OPERATION_COMMON_CFG = 0x00U, /*!< Common configuration (indirect or auto-polling mode) */
  HAL_XSPI_OPERATION_READ_CFG   = 0x00U, /*!< Read configuration (memory-mapped mode)              */
#if defined(XSPI_WIR_INSTRUCTION)
  HAL_XSPI_OPERATION_WRITE_CFG  = 0x80U, /*!< Write configuration (memory-mapped mode)             */
#endif /* XSPI_WIR_INSTRUCTION */
#if defined(XSPI_DCR2_WRAPSIZE)
  HAL_XSPI_OPERATION_WRAP_CFG   = 0x40U  /*!< Wrap configuration (memory-mapped mode)              */
#endif /* XSPI_DCR2_WRAPSIZE */
} hal_xspi_operation_type_t;

/**
  * @brief HAL XSPI IO Select enumeration definition.
  */
typedef enum
{
  HAL_XSPI_IO_3_0   = 0U,                               /*!< Data exchanged over IO[3:0]   */
  HAL_XSPI_IO_7_4   = XSPI_CR_MSEL,                     /*!< Data exchanged over IO[7:4]   */
  HAL_XSPI_IO_7_0   = 0U                                /*!< Data exchanged over IO[7:0]   */
} hal_xspi_io_select_t;

/**
  * @brief HAL XSPI Instruction Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_INSTRUCTION_NONE   = 0U,                                    /*!< No instruction               */
  HAL_XSPI_INSTRUCTION_1LINE  = XSPI_CCR_IMODE_0,                      /*!< Instruction on a single line */
  HAL_XSPI_INSTRUCTION_2LINES = XSPI_CCR_IMODE_1,                      /*!< Instruction on two lines     */
  HAL_XSPI_INSTRUCTION_4LINES = (XSPI_CCR_IMODE_0 | XSPI_CCR_IMODE_1), /*!< Instruction on four lines    */
  HAL_XSPI_INSTRUCTION_8LINES = XSPI_CCR_IMODE_2                       /*!< Instruction on eight lines   */
} hal_xspi_instruction_mode_t;

/**
  * @brief HAL XSPI Instruction Width enumeration definition.
  */
typedef enum
{
  HAL_XSPI_INSTRUCTION_8BIT  = 0U,               /*!< 8-bit instruction  */
  HAL_XSPI_INSTRUCTION_16BIT = XSPI_CCR_ISIZE_0, /*!< 16-bit instruction */
  HAL_XSPI_INSTRUCTION_24BIT = XSPI_CCR_ISIZE_1, /*!< 24-bit instruction */
  HAL_XSPI_INSTRUCTION_32BIT = XSPI_CCR_ISIZE    /*!< 32-bit instruction */
} hal_xspi_instruction_width_t;

/**
  * @brief HAL XSPI Instruction DTR Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_INSTRUCTION_DTR_DISABLED = 0U,           /*!< DTR mode disabled for instruction phase */
  HAL_XSPI_INSTRUCTION_DTR_ENABLED  = XSPI_CCR_IDTR /*!< DTR mode enabled for instruction phase  */
} hal_xspi_instruction_dtr_status_t;

/**
  * @brief HAL XSPI Address Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ADDR_NONE    = 0U,                                      /*!< No address               */
  HAL_XSPI_ADDR_1LINE   = XSPI_CCR_ADMODE_0,                       /*!< Address on a single line */
  HAL_XSPI_ADDR_2LINES  = XSPI_CCR_ADMODE_1,                       /*!< Address on two lines     */
  HAL_XSPI_ADDR_4LINES  = (XSPI_CCR_ADMODE_0 | XSPI_CCR_ADMODE_1), /*!< Address on four lines    */
  HAL_XSPI_ADDR_8LINES  = XSPI_CCR_ADMODE_2                        /*!< Address on eight lines   */
} hal_xspi_addr_mode_t;

/**
  * @brief HAL XSPI Address width enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ADDR_8BIT  = 0U,                /*!< 8-bit address  */
  HAL_XSPI_ADDR_16BIT = XSPI_CCR_ADSIZE_0, /*!< 16-bit address */
  HAL_XSPI_ADDR_24BIT = XSPI_CCR_ADSIZE_1, /*!< 24-bit address */
  HAL_XSPI_ADDR_32BIT = XSPI_CCR_ADSIZE    /*!< 32-bit address */
} hal_xspi_addr_width_t;

/**
  * @brief HAL XSPI Address DTR Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ADDR_DTR_DISABLED = 0U,            /*!< DTR mode disabled for address phase */
  HAL_XSPI_ADDR_DTR_ENABLED  = XSPI_CCR_ADDTR /*!< DTR mode enabled for address phase  */
} hal_xspi_addr_dtr_status_t;

/**
  * @brief HAL XSPI Alternate Bytes Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ALTERNATE_BYTES_NONE   = 0U,                                      /*!< No alternate bytes               */
  HAL_XSPI_ALTERNATE_BYTES_1LINE  = XSPI_CCR_ABMODE_0,                       /*!< Alternate bytes on a single line */
  HAL_XSPI_ALTERNATE_BYTES_2LINES = XSPI_CCR_ABMODE_1,                       /*!< Alternate bytes on two lines     */
  HAL_XSPI_ALTERNATE_BYTES_4LINES = (XSPI_CCR_ABMODE_0 | XSPI_CCR_ABMODE_1), /*!< Alternate bytes on four lines    */
  HAL_XSPI_ALTERNATE_BYTES_8LINES = XSPI_CCR_ABMODE_2                        /*!< Alternate bytes on eight lines   */
} hal_xspi_alternate_bytes_mode_t;

/**
  * @brief HAL XSPI Alternate Bytes Width enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ALTERNATE_BYTES_8BIT  = 0U,                /*!< 8-bit alternate bytes  */
  HAL_XSPI_ALTERNATE_BYTES_16BIT = XSPI_CCR_ABSIZE_0, /*!< 16-bit alternate bytes */
  HAL_XSPI_ALTERNATE_BYTES_24BIT = XSPI_CCR_ABSIZE_1, /*!< 24-bit alternate bytes */
  HAL_XSPI_ALTERNATE_BYTES_32BIT = XSPI_CCR_ABSIZE    /*!< 32-bit alternate bytes */
} hal_xspi_alternate_bytes_width_t;

/**
  * @brief HAL XSPI Alternate Bytes DTR Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED = 0U,            /*!< DTR mode disabled for alternate bytes phase */
  HAL_XSPI_ALTERNATE_BYTES_DTR_ENABLED  = XSPI_CCR_ABDTR /*!< DTR mode enabled for alternate bytes phase  */
} hal_xspi_alternate_bytes_dtr_status_t;

/**
  * @brief HAL XSPI Regular Data Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_REGULAR_DATA_NONE     = 0U,                                   /*!< No data               */
  HAL_XSPI_REGULAR_DATA_1LINE   = XSPI_CCR_DMODE_0,                      /*!< Data on a single line */
  HAL_XSPI_REGULAR_DATA_2LINES  = XSPI_CCR_DMODE_1,                      /*!< Data on two lines     */
  HAL_XSPI_REGULAR_DATA_4LINES  = (XSPI_CCR_DMODE_0 | XSPI_CCR_DMODE_1), /*!< Data on four lines    */
  HAL_XSPI_REGULAR_DATA_8LINES  = XSPI_CCR_DMODE_2,                      /*!< Data on eight lines   */
} hal_xspi_regular_data_mode_t;

/**
  * @brief HAL XSPI Data DTR Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_DATA_DTR_DISABLED = 0U,           /*!< DTR mode disabled for data phase */
  HAL_XSPI_DATA_DTR_ENABLED  = XSPI_CCR_DDTR /*!< DTR mode enabled for data phase  */
} hal_xspi_data_dtr_status_t;

/**
  * @brief HAL XSPI DQS Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_DQS_DISABLED = 0U,           /*!< DQS disabled */
  HAL_XSPI_DQS_ENABLED  = XSPI_CCR_DQSE /*!< DQS enabled  */
} hal_xspi_dqs_status_t;

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
/**
  * @brief HAL XSPI Hyperbus Write Zero Latency Activation enumeration definition.
  */
typedef enum
{
  HAL_XSPI_WRITE_ZERO_LATENCY_ENABLED  = 0U,           /*!< Latency on write accesses    */
  HAL_XSPI_WRITE_ZERO_LATENCY_DISABLED = XSPI_HLCR_WZL /*!< No latency on write accesses */
} hal_xspi_write_zero_latency_status_t;

/**
  * @brief HAL XSPI Hyperbus Latency Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_LATENCY_VARIABLE = 0U,          /*!< Variable initial latency */
  HAL_XSPI_LATENCY_FIXED    = XSPI_HLCR_LM /*!< Fixed latency            */
} hal_xspi_latency_mode_t;

/**
  * @brief HAL XSPI Hyperbus Address Space enumeration definition.
  */
typedef enum
{
  HAL_XSPI_ADDR_MEMORY   = 0U,              /*!< HyperBus memory mode   */
  HAL_XSPI_ADDR_REGISTER = XSPI_DCR1_MTYP_0 /*!< HyperBus register mode */
} hal_xspi_addr_space_t;

/**
  * @brief HAL XSPI Hyperbus Data Mode enumeration definition.
  */
typedef enum
{
  HAL_XSPI_HYPERBUS_DATA_8LINES  = XSPI_CCR_DMODE_2,                     /*!< Data on eight lines   */
} hal_xspi_hyperbus_data_mode_t;
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

/**
  * @brief HAL XSPI Match Mode enumeration definition.
  */
#if defined(XSPI_SR_SMF)
typedef enum
{
  HAL_XSPI_MATCH_MODE_AND = 0U,         /*!< AND match mode between unmasked bits */
  HAL_XSPI_MATCH_MODE_OR  = XSPI_CR_PMM /*!< OR match mode between unmasked bits  */
} hal_xspi_match_mode_t;
#else
typedef enum
{
  HAL_XSPI_MATCH_MODE_AND = 0U,         /*!< AND match mode between unmasked bits */
  HAL_XSPI_MATCH_MODE_OR  = 1U          /*!< OR match mode between unmasked bits  */
} hal_xspi_match_mode_t;
#endif /* XSPI_SR_SMF */

/**
  * @brief HAL XSPI Automatic Stop enumeration definition.
  */
#if defined(XSPI_SR_SMF)
typedef enum
{
  HAL_XSPI_AUTOMATIC_STOP_DISABLED = 0U,          /*!< AutoPolling stops only with abort or XSPI disabling */
  HAL_XSPI_AUTOMATIC_STOP_ENABLED  = XSPI_CR_APMS /*!< AutoPolling stops as soon as there is a match       */
} hal_xspi_automatic_stop_status_t;
#else
typedef enum
{
  HAL_XSPI_AUTOMATIC_STOP_DISABLED = 0U,          /*!< AutoPolling stops only with abort or XSPI disabling */
  HAL_XSPI_AUTOMATIC_STOP_ENABLED  = 1U           /*!< AutoPolling stops as soon as there is a match       */
} hal_xspi_automatic_stop_status_t;
#endif /* XSPI_SR_SMF */

/**
  * @brief HAL XSPI Timeout Activation enumeration definition.
  */
typedef enum
{
  HAL_XSPI_TIMEOUT_DISABLE = 0U,          /*!< Timeout counter disabled, nCS remains active               */
  HAL_XSPI_TIMEOUT_ENABLE  = XSPI_CR_TCEN /*!< Timeout counter enabled, nCS released when timeout expires */
} hal_xspi_timeout_activation_t;


#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
/**
  * @brief HAL XSPI Hyperbus Configuration Structure definition.
  */
typedef struct
{
  uint32_t rw_recovery_time_cycle;                         /*!< It indicates the number of cycles for the device
                                                                recovery time.
                                                                This parameter can be a value between 0 and 255.  */
  uint32_t access_time_cycle;                              /*!< It indicates the number of cycles for the device access
                                                                time.
                                                                This parameter can be a value between 0 and 255.  */
  hal_xspi_write_zero_latency_status_t write_zero_latency; /*!< It enables or disables the latency for the write access. */
  hal_xspi_latency_mode_t latency_mode;                    /*!< It configures the latency mode.                   */
} hal_xspi_hyperbus_config_t;
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

/**
  * @brief HAL XSPI Timing Config structure definition.
  */
typedef struct
{
  uint32_t clk_prescaler;              /*!< It specifies the prescaler factor used for generating
                                            the external clock based on the kernel clock.
                                            This parameter can be a value between 0 and 255.
                                            Choosing a prescaler value of N means dividing the clock by N+1.     */
  hal_xspi_sample_shift_t shift;       /*!< It allows delaying the data sampling by 1/2 cycle to
                                            take into account external signal delays.             */
  hal_xspi_delay_hold_t hold;          /*!< It allows holding the data for 1/4 cycle.             */
  uint32_t cs_high_time_cycle;         /*!< It defines the minimum number of clocks which the chip
                                            select must remain high between commands.
                                            This parameter can be a value between 1 and 64.       */
#if defined(XSPI_DCR4_REFRESH)
  uint32_t cs_refresh_time_cycle;      /*!< It enables the refresh rate feature. The chip select is
                                            released every Refresh+1 clock cycles.
                                            This parameter can be a value between 0 and 0xFFFFFFFF. */
#endif /* XSPI_DCR4_REFRESH */
  hal_xspi_dlyb_state_t dlyb_state;    /*!< It enables the delay block bypass, so the sampling is
                                            not affected by the delay block.                      */
} hal_xspi_timing_config_t;

/**
  * @brief HAL XSPI Memory Configuration structure definition.
  */
typedef struct
{
  hal_xspi_memory_mode_t mode;              /*!< It specifies the memory mode.                                    */

  hal_xspi_memory_type_t type;              /*!< It indicates the external device type connected to the XSPI.     */

  hal_xspi_memory_size_t size_bit;          /*!< It defines the size of the external device connected to the XSPI.
                                                 It corresponds to the number of address bits required to access
                                                 the external device.                                             */

#if defined(XSPI_DCR2_WRAPSIZE)
  hal_xspi_wrap_size_t wrap_size_byte;      /*!< It indicates the wrap-size corresponding to the external device. */
#endif /* XSPI_DCR2_WRAPSIZE */

#if defined(XSPI_DCR3_CSBOUND)
  hal_xspi_cs_boundary_t cs_boundary;       /*!< It enables the transaction boundary feature and
                                                 defines the boundary of bytes to release the chip select.       */
#endif /* XSPI_DCR3_CSBOUND */
} hal_xspi_memory_config_t;

/**
  * @brief HAL XSPI Global Configuration structure definition.
  */
typedef struct
{
  hal_xspi_memory_config_t memory;     /*!< It specifies the XSPI memory configuration structure definition */

  hal_xspi_timing_config_t timing;     /*!< It specifies the XSPI timing configuration structure definition */

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  hal_xspi_hyperbus_config_t hyperbus; /*!< It specifies the XSPI Hyperbus configuration structure definition */
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */
} hal_xspi_config_t;

/**
  * @brief HAL XSPI Auto Polling mode configuration structure definition.
  */
typedef struct
{
  uint32_t match_value;                                   /*!< Specifies the value to be compared with the masked status
                                                               register. This parameter can be any value between
                                                               0 and 0xFFFFFFFFU.                                   */
  uint32_t match_mask;                                    /*!< Specifies the mask to be applied to the status bytes
                                                               received. This parameter can be any value between
                                                               0 and 0xFFFFFFFFU.                                   */
  hal_xspi_match_mode_t match_mode;                       /*!< Specifies the method used for determining a match.   */

  hal_xspi_automatic_stop_status_t automatic_stop_status; /*!< Specifies whether automatic polling is stopped after a
                                                               match.                                               */
  uint32_t interval_cycle;                                /*!< Specifies the number of clock cycles between two reads
                                                               during automatic polling phases.
                                                               This parameter can be any value between 0 and 0xFFFF. */
} hal_xspi_auto_polling_config_t;

/**
  * @brief HAL XSPI Regular Command Structure definition.
  */
typedef struct
{
  hal_xspi_operation_type_t operation_type;                              /*!< It indicates whether the configuration applies
                                                                              to the common registers or
                                                                              to the registers for the write operation
                                                                              (these registers are only used for
                                                                               memory-mapped mode).                  */
  hal_xspi_io_select_t  io_select;                                       /*!< It indicates the I/Os used to exchange
                                                                              data with external memory.             */

  uint32_t instruction;                                                  /*!< It contains the instruction to be sent to
                                                                              the device.
                                                                              This parameter can be a value between
                                                                              0 and 0xFFFFFFFFU.                     */
  hal_xspi_instruction_mode_t instruction_mode;                          /*!< It indicates the instruction mode.     */

  hal_xspi_instruction_width_t instruction_width;                        /*!< It indicates the width of the
                                                                              instruction.                           */

  hal_xspi_instruction_dtr_status_t instruction_dtr_mode_status;         /*!< It enables or disables the DTR mode for the
                                                                              instruction phase.                     */

  uint32_t addr;                                                         /*!< It contains the address to be sent to the
                                                                              device.
                                                                              This parameter can be a value between
                                                                              0 and 0xFFFFFFFF.                      */
  hal_xspi_addr_mode_t addr_mode;                                        /*!< It indicates the address mode. Address
                                                                              mode specifies the number of lines
                                                                              for address (except no address).       */
  hal_xspi_addr_width_t addr_width;                                      /*!< It indicates the width of the address. */

  hal_xspi_addr_dtr_status_t addr_dtr_mode_status;                       /*!< It enables or disables the DTR mode for the
                                                                              address phase.                         */

  uint32_t alternate_bytes;                                              /*!< It contains the alternate bytes to be sent
                                                                              to the device.
                                                                              This parameter can be a value between
                                                                              0 and 0xFFFFFFFF.                      */
  hal_xspi_alternate_bytes_mode_t alternate_bytes_mode;                  /*!< It indicates the mode of the alternate
                                                                              bytes.                                 */

  hal_xspi_alternate_bytes_width_t alternate_bytes_width;                /*!< It indicates the width of the alternate
                                                                              bytes.                                 */

  hal_xspi_alternate_bytes_dtr_status_t alternate_bytes_dtr_mode_status; /*!< It enables or disables the DTR mode for the
                                                                              alternate bytes phase.                 */

  hal_xspi_regular_data_mode_t data_mode;                                /*!< It indicates the data mode. Data mode
                                                                              specifies the number of lines
                                                                              for data exchange (except no data).    */
  hal_xspi_data_dtr_status_t data_dtr_mode_status;                       /*!< It enables or disables the DTR mode for the
                                                                              data phase.                            */

  uint32_t dummy_cycle;                                                  /*!< It indicates the number of dummy cycles
                                                                              inserted before data phase.
                                                                              This parameter can be a value between
                                                                              0 and 31U. */
  hal_xspi_dqs_status_t dqs_mode_status;                                 /*!< It enables or disables the data strobe
                                                                              management.                            */
  uint32_t size_byte;                                                    /*!< It indicates the number of data
                                                                              transferred with this command.
                                                                              This field is only used for indirect mode.
                                                                              This parameter can be a value between
                                                                              1 and 0xFFFFFFFFU.                     */
} hal_xspi_regular_cmd_t;

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
/**
  * @brief HAL XSPI Hyperbus Command Structure definition.
  */
typedef struct
{
  hal_xspi_addr_space_t         addr_space;      /*!< It indicates the address space accessed by the command. */

  uint32_t                      addr;            /*!< It contains the address to be sent to the device.
                                                      This parameter can be a value between 0 and 0xFFFFFFFF. */
  hal_xspi_addr_width_t         addr_width;      /*!< It indicates the width of the address.                */

  uint32_t                      size_byte;       /*!< It indicates the number of data transferred with this command.
                                                      This field is only used for indirect mode.
                                                      This parameter can be a value between 1 and 0xFFFFFFFF. */
  hal_xspi_dqs_status_t         dqs_mode_status; /*!< It enables or disables the data strobe management.    */

  hal_xspi_hyperbus_data_mode_t data_mode;       /*!< It indicates the data mode. Data mode specifies the number of lines for
                                                      data exchange (except no data).                       */
} hal_xspi_hyperbus_cmd_t;
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

/**
  * @brief HAL XSPI Memory Mapped mode configuration structure definition.
  */
typedef struct
{
  hal_xspi_timeout_activation_t timeout_activation; /*!< Specifies if the timeout counter is enabled to release the chip
                                                         select.                                              */

  uint32_t timeout_period_cycle;                    /*!< Specifies the number of clock cycles to wait when the FIFO is full
                                                         before releasing the chip select.
                                                         This parameter can be any value between 0 and 0xFFFF. */
} hal_xspi_memory_mapped_config_t;


typedef struct hal_xspi_handle_s hal_xspi_handle_t;/*!< XSPI Handle Structure type */

#if defined(USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
typedef void (*hal_xspi_cb_t)(hal_xspi_handle_t *hxspi);/*!< XSPI Callback pointer definition */
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

/**
  * @brief HAL XSPI handle Structure definition.
  */
struct hal_xspi_handle_s
{
  hal_xspi_t                instance;       /*!< XSPI registers base address                 */

  volatile hal_xspi_state_t global_state;   /*!< Internal state of the XSPI HAL driver       */


  uint8_t                   *p_buffer;      /*!< Address of the XSPI buffer for transfer     */

  volatile uint32_t         xfer_size;      /*!< Number of data to transfer                  */

  volatile uint32_t         xfer_count;     /*!< Counter of data transferred                 */

  hal_xspi_memory_mode_t    mode;           /*!< Specifies the memory mode                   */

  hal_xspi_delay_hold_t     hold;           /*!< It allows holding the data for 1/4 cycle    */

  hal_xspi_memory_type_t    type;           /*!< Indicates the external device type          */

  uint32_t                  fifo_threshold; /*!< Specifies the FIFO configuration value      */

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
  uint32_t                  is_dma_error;   /*!< Indicates an error occurs in DMA mode       */

  hal_dma_handle_t          *hdma_tx;       /*!< Handle of the DMA channel used for transmit */

  hal_dma_handle_t          *hdma_rx;       /*!< Handle of the DMA channel used for receive  */
#endif /* USE_HAL_XSPI_DMA */

#if defined(USE_HAL_XSPI_USER_DATA) && (USE_HAL_XSPI_USER_DATA == 1U)
  const void                *p_user_data;   /*!< User Data Pointer                           */
#endif /* USE_HAL_XSPI_USER_DATA */

#if defined(USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
  hal_xspi_cb_t p_error_cb;                 /*!< XSPI error callback                         */
  hal_xspi_cb_t p_abort_cplt_cb;            /*!< XSPI abort callback                         */
  hal_xspi_cb_t p_fifo_threshold_cb;        /*!< XSPI FIFO threshold callback                */
  hal_xspi_cb_t p_cmd_cplt_cb;              /*!< XSPI command complete callback              */
  hal_xspi_cb_t p_rx_cplt_cb;               /*!< XSPI receive complete callback              */
  hal_xspi_cb_t p_tx_cplt_cb;               /*!< XSPI transfer complete callback             */
  hal_xspi_cb_t p_rx_half_cplt_cb;          /*!< XSPI half receive complete callback         */
  hal_xspi_cb_t p_tx_half_cplt_cb;          /*!< XSPI half transfer complete callback        */
  hal_xspi_cb_t p_status_match_cb;          /*!< XSPI status match callback                  */
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  /* in case of single process at a time: one single variable storing the last errors */
  volatile uint32_t last_error_codes;       /*!< XSPI error codes                            */
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
};

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup XSPI_Exported_Functions HAL XSPI Functions
  * @{
  */

/** @defgroup XSPI_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
hal_status_t HAL_XSPI_Init(hal_xspi_handle_t *hxspi, hal_xspi_t instance);
void         HAL_XSPI_DeInit(hal_xspi_handle_t *hxspi);
/**
  * @}
  */

/** @defgroup XSPI_Exported_Functions_Group2 XSPI Configuration functions
  * @{
  */
hal_status_t HAL_XSPI_SetConfig(hal_xspi_handle_t *hxspi, const hal_xspi_config_t *p_config);
void         HAL_XSPI_GetConfig(hal_xspi_handle_t *hxspi, hal_xspi_config_t *p_config);
hal_status_t HAL_XSPI_SetFifoThreshold(hal_xspi_handle_t *hxspi, uint32_t threshold);
uint32_t     HAL_XSPI_GetFifoThreshold(const hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_SetPrescaler(hal_xspi_handle_t *hxspi, uint32_t clk_prescaler);
uint32_t     HAL_XSPI_GetPrescaler(const hal_xspi_handle_t *hxspi);
hal_status_t           HAL_XSPI_SetMemorySize(hal_xspi_handle_t *hxspi, hal_xspi_memory_size_t size);
hal_xspi_memory_size_t HAL_XSPI_GetMemorySize(const hal_xspi_handle_t *hxspi);
hal_status_t           HAL_XSPI_SetMemoryType(hal_xspi_handle_t *hxspi, hal_xspi_memory_type_t type);
hal_xspi_memory_type_t HAL_XSPI_GetMemoryType(const hal_xspi_handle_t *hxspi);
hal_status_t                HAL_XSPI_SetMemorySelection(hal_xspi_handle_t *hxspi, hal_xspi_memory_selection_t select);
hal_xspi_memory_selection_t HAL_XSPI_GetMemorySelection(const hal_xspi_handle_t *hxspi);
uint32_t HAL_XSPI_GetMemoryMappedBaseAddress(const hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_EnableFreeRunningClock(hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_DisableFreeRunningClock(hal_xspi_handle_t *hxspi);
hal_xspi_free_running_clk_status_t HAL_XSPI_IsEnabledFreeRunningClock(const hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_EnablePrefetchData(hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_DisablePrefetchData(hal_xspi_handle_t *hxspi);
hal_xspi_prefetch_data_status_t HAL_XSPI_IsEnabledPrefetchData(const hal_xspi_handle_t *hxspi);
/**
  * @}
  */

/** @defgroup XSPI_Exported_Functions_Group3 XSPI Command and I/O operation functions
  * @{
  */
hal_status_t HAL_XSPI_StartMemoryMappedMode(hal_xspi_handle_t *hxspi, const hal_xspi_memory_mapped_config_t *p_config);
hal_status_t HAL_XSPI_StopMemoryMappedMode(hal_xspi_handle_t *hxspi);

/* XSPI command configuration functions */
hal_status_t HAL_XSPI_SendRegularCmd(hal_xspi_handle_t *hxspi, const hal_xspi_regular_cmd_t *p_cmd,
                                     uint32_t timeout_ms);
hal_status_t HAL_XSPI_SendRegularCmd_IT(hal_xspi_handle_t *hxspi, const hal_xspi_regular_cmd_t *p_cmd);

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
hal_status_t HAL_XSPI_SendHyperbusCmd(hal_xspi_handle_t *hxspi, const hal_xspi_hyperbus_cmd_t *p_cmd,
                                      uint32_t timeout_ms);
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

hal_status_t HAL_XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi, const hal_xspi_auto_polling_config_t *p_config,
                                          uint32_t timeout_ms);
#if defined(XSPI_SR_SMF)
hal_status_t HAL_XSPI_ExecRegularAutoPoll_IT(hal_xspi_handle_t *hxspi, const hal_xspi_auto_polling_config_t *p_config);
#endif /* XSPI_SR_SMF */

/* IO operation functions */
hal_status_t HAL_XSPI_Transmit(hal_xspi_handle_t *hxspi, const void *p_data, uint32_t timeout_ms);
hal_status_t HAL_XSPI_Receive(hal_xspi_handle_t *hxspi, void *p_data, uint32_t timeout_ms);
hal_status_t HAL_XSPI_Transmit_IT(hal_xspi_handle_t *hxspi, const void *p_data);
hal_status_t HAL_XSPI_Receive_IT(hal_xspi_handle_t *hxspi, void *p_data);
#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
hal_status_t HAL_XSPI_Transmit_DMA(hal_xspi_handle_t *hxspi, const void *p_data);
hal_status_t HAL_XSPI_Receive_DMA(hal_xspi_handle_t *hxspi, void *p_data);
hal_status_t HAL_XSPI_Transmit_DMA_Opt(hal_xspi_handle_t *hxspi, const void *p_data, uint32_t interrupts);
hal_status_t HAL_XSPI_Receive_DMA_Opt(hal_xspi_handle_t *hxspi, void *p_data, uint32_t interrupts);
#endif /* USE_HAL_XSPI_DMA */
hal_status_t HAL_XSPI_Abort(hal_xspi_handle_t *hxspi, uint32_t timeout_ms);
hal_status_t HAL_XSPI_Abort_IT(hal_xspi_handle_t *hxspi);
/**
  * @}
  */

/** @defgroup XSPI_Exported_Functions_Group4 IRQHandler, link DMA, and callback functions
  * @{
  */
void HAL_XSPI_IRQHandler(hal_xspi_handle_t *hxspi);
/* Callback functions in non-blocking modes */
void HAL_XSPI_ErrorCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_AbortCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_FifoThresholdCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_CmdCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_RxCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_TxCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_RxHalfCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_TxHalfCpltCallback(hal_xspi_handle_t *hxspi);
void HAL_XSPI_StatusMatchCallback(hal_xspi_handle_t *hxspi);
#if defined(USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
/* XSPI callback registration */
hal_status_t HAL_XSPI_RegisterErrorCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterCmdCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterRxCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterTxCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterRxHalfCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterTxHalfCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
#if defined(XSPI_SR_SMF)
hal_status_t HAL_XSPI_RegisterStatusMatchCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
#endif /* XSPI_SR_SMF */
hal_status_t HAL_XSPI_RegisterAbortCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
hal_status_t HAL_XSPI_RegisterFifoThresholdCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback);
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */
#if defined(USE_HAL_XSPI_USER_DATA) && (USE_HAL_XSPI_USER_DATA == 1U)
void HAL_XSPI_SetUserData(hal_xspi_handle_t *hxspi, const void *p_user_data);
const void *HAL_XSPI_GetUserData(const hal_xspi_handle_t *hxspi);
#endif /* USE_HAL_XSPI_USER_DATA */
#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
hal_status_t HAL_XSPI_SetTxDMA(hal_xspi_handle_t *hxspi, hal_dma_handle_t *hdma_tx);
hal_status_t HAL_XSPI_SetRxDMA(hal_xspi_handle_t *hxspi, hal_dma_handle_t *hdma_rx);
#endif /* USE_HAL_XSPI_DMA */
/**
  * @}
  */

/** @defgroup XSPI_Exported_Functions_Group5 Peripheral current frequency, state and errors functions
  * @{
  */
uint32_t         HAL_XSPI_GetClockFreq(const hal_xspi_handle_t *hxspi);
hal_xspi_state_t HAL_XSPI_GetState(const hal_xspi_handle_t *hxspi);
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
uint32_t         HAL_XSPI_GetLastErrorCodes(const hal_xspi_handle_t *hxspi);
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
/**
  * @}
  */

/** @defgroup XSPI_Exported_Functions_Group6 XSPI Delay Block functions
  * @{
  */
hal_status_t HAL_XSPI_DLYB_SetConfigDelay(hal_xspi_handle_t *hxspi, uint32_t clock_phase_value);
hal_status_t HAL_XSPI_DLYB_GetConfigDelay(const hal_xspi_handle_t *hxspi, uint32_t *p_clock_phase);
hal_status_t HAL_XSPI_DLYB_CalculateMaxClockPhase(hal_xspi_handle_t *hxspi, uint32_t *p_max_clock_phase);
hal_status_t HAL_XSPI_DLYB_Enable(hal_xspi_handle_t *hxspi);
hal_status_t HAL_XSPI_DLYB_Disable(hal_xspi_handle_t *hxspi);
hal_xspi_dlyb_status_t HAL_XSPI_DLYB_IsEnabled(const hal_xspi_handle_t *hxspi);
/**
  * @}
  */


/** @addtogroup XSPI_Exported_Functions_Group9 Interrupt functions
  * @{
  */
/** @brief  Enable the specified XSPI interrupt.
  * @param  hxspi specifies the XSPI Handle.
  * @param  it_source specifies the XSPI interrupt source to enable
  *         This parameter can be any combination of the following values:
  *            @arg HAL_XSPI_IT_TO:  XSPI Timeout interrupt
  *            @arg HAL_XSPI_IT_SM:  XSPI Status match interrupt
  *            @arg HAL_XSPI_IT_FT:  XSPI FIFO threshold interrupt
  *            @arg HAL_XSPI_IT_TC:  XSPI Transfer complete interrupt
  *            @arg HAL_XSPI_IT_TE:  XSPI Transfer error interrupt
  *            @arg HAL_XSPI_IT_ALL: XSPI All interrupts
  */
__STATIC_INLINE void HAL_XSPI_EnableIT(hal_xspi_handle_t *hxspi, uint32_t it_source)
{
  STM32_SET_BIT(((XSPI_TypeDef *)((uint32_t) hxspi->instance))->CR, it_source);
}

/** @brief  Disable the specified XSPI interrupt.
  * @param  hxspi specifies the XSPI Handle.
  * @param  it_source specifies the XSPI interrupt source to disable
  *         This parameter can be any combination of the following values:
  *            @arg HAL_XSPI_IT_TO: XSPI Timeout interrupt
  *            @arg HAL_XSPI_IT_SM: XSPI Status match interrupt
  *            @arg HAL_XSPI_IT_FT: XSPI FIFO threshold interrupt
  *            @arg HAL_XSPI_IT_TC: XSPI Transfer complete interrupt
  *            @arg HAL_XSPI_IT_TE: XSPI Transfer error interrupt
  *            @arg HAL_XSPI_IT_ALL: XSPI All interrupts
  */
__STATIC_INLINE void HAL_XSPI_DisableIT(hal_xspi_handle_t *hxspi, uint32_t it_source)
{
  STM32_CLEAR_BIT(((XSPI_TypeDef *)((uint32_t) hxspi->instance))->CR, it_source);
}

/** @brief  Check whether the specified XSPI interrupt source is enabled or not.
  * @param  hxspi specifies the XSPI Handle.
  * @param  it_source specifies the XSPI interrupt source to check
  *         This parameter can be one of the following values:
  *            @arg HAL_XSPI_IT_TO: XSPI Timeout interrupt
  *            @arg HAL_XSPI_IT_SM: XSPI Status match interrupt
  *            @arg HAL_XSPI_IT_FT: XSPI FIFO threshold interrupt
  *            @arg HAL_XSPI_IT_TC: XSPI Transfer complete interrupt
  *            @arg HAL_XSPI_IT_TE: XSPI Transfer error interrupt
  * @return retrieve the state of the selected XSPI interrupt.
  */
__STATIC_INLINE uint32_t HAL_XSPI_IsEnabledIT(const hal_xspi_handle_t *hxspi, uint32_t it_source)
{
  return ((STM32_READ_BIT(((XSPI_TypeDef *)((uint32_t) hxspi->instance))->CR,
                          it_source) == it_source) ? 1U : 0U);
}

/**
  * @brief  Check whether the selected XSPI flag is set or not.
  * @param  hxspi specifies the XSPI Handle.
  * @param  flag specifies the XSPI flag to check
  *         This parameter can be one of the following values:
  *            @arg HAL_XSPI_FLAG_BUSY: XSPI Busy flag
  *            @arg HAL_XSPI_FLAG_TO:   XSPI Timeout flag
  *            @arg HAL_XSPI_FLAG_SM:   XSPI Status match flag
  *            @arg HAL_XSPI_FLAG_FT:   XSPI FIFO threshold flag
  *            @arg HAL_XSPI_FLAG_TC:   XSPI Transfer complete flag
  *            @arg HAL_XSPI_FLAG_TE:   XSPI Transfer error flag
  * @return retrieve the state of the selected XSPI flag.
  */
__STATIC_INLINE hal_xspi_flag_status_t HAL_XSPI_IsActiveFlag(const hal_xspi_handle_t *hxspi, uint32_t flag)
{
  return ((STM32_READ_BIT(((XSPI_TypeDef *)((uint32_t) hxspi->instance))->SR, flag) != 0U) ? HAL_XSPI_FLAG_ACTIVE \
          : HAL_XSPI_FLAG_NOT_ACTIVE);
}

/** @brief  Clears the specified XSPI's flag status.
  * @param  hxspi specifies the XSPI Handle.
  * @param  flag specifies the XSPI clear register flag that needs to be set
  *         This parameter can be any combination of the following values:
  *            @arg HAL_XSPI_FLAG_TO:   XSPI Timeout flag
  *            @arg HAL_XSPI_FLAG_SM:   XSPI Status match flag
  *            @arg HAL_XSPI_FLAG_TC:   XSPI Transfer complete flag
  *            @arg HAL_XSPI_FLAG_TE:   XSPI Transfer error flag
  *            @arg HAL_XSPI_FLAG_ALL:  XSPI All flags
  */
__STATIC_INLINE void HAL_XSPI_ClearFlag(hal_xspi_handle_t *hxspi, uint32_t flag)
{
  STM32_WRITE_REG(((XSPI_TypeDef *)((uint32_t) hxspi->instance))->FCR, flag);
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

/**
  * @}
  */
#endif /* XSPI1 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_XSPI_H */
