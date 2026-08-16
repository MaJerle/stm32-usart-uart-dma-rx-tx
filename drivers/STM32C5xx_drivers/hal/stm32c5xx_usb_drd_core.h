/**
  ******************************************************************************
  * @file    stm32c5xx_usb_drd_core.h
  * @brief   USB DRD core module header file.
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
#ifndef STM32C5XX_USB_DRD_CORE_H
#define STM32C5XX_USB_DRD_CORE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"
#include "stm32c5xx_usb_core_def.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (USB_DRD_FS)
/** @defgroup USB_DRD_CORE USB DRD Core
  * @{
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup USB_DRD_Private_Constants Private Constants
  * @{
  */
/**
  * @brief  USB DRD timeout value.
  */
#ifndef USB_DRD_TIMEOUT
#define USB_DRD_TIMEOUT                                            (0xF000000U)
#endif /* USB_DRD_TIMEOUT */

#ifndef USB_EP_SETUP
#define USB_EP_SETUP                                           (USB_CHEP_SETUP)
#endif /* USB_EP_SETUP */

#ifndef USB_EP_DTOGTX
#define USB_EP_DTOGTX                                         (USB_CHEP_DTOGTX)
#endif /* USB_EP_DTOGTX */

#ifndef USB_CH_DTOGTX
#define USB_CH_DTOGTX                                         (USB_CHEP_DTOGTX)
#endif /* USB_EP_DTOGTX */

#ifndef USB_EP_DTOGRX
#define USB_EP_DTOGRX                                         (USB_CHEP_DTOGRX)
#endif /* USB_EP_DTOGTX */

#ifndef USB_CH_DTOGRX
#define USB_CH_DTOGRX                                         (USB_CHEP_DTOGRX)
#endif /* USB_EP_DTOGTX */

#ifndef USB_EP_VTRX
#define USB_EP_VTRX                                             (USB_CHEP_VTRX)
#endif /* USB_EP_VTRX */

#ifndef USB_CH_VTRX
#define USB_CH_VTRX                                             (USB_CHEP_VTRX)
#endif /* USB_CH_VTRX */

#ifndef USB_EP_VTTX
#define USB_EP_VTTX                                             (USB_CHEP_VTTX)
#endif /* USB_EP_VTTX */

#ifndef USB_CH_VTTX
#define USB_CH_VTTX                                             (USB_CHEP_VTTX)
#endif /* USB_CH_VTTX */

#ifndef USB_EP_STATRX
#define USB_EP_STATRX                                         (USB_CHEP_STATRX)
#endif /* USB_EP_STATRX */

#ifndef USB_CH_STATRX
#define USB_CH_STATRX                                         (USB_CHEP_STATRX)
#endif /* USB_CH_STATRX */

#ifndef USB_CH_ERR_TX
#define USB_CH_ERR_TX                                         (USB_CHEP_ERR_TX)
#endif /* USB_CH_ERR_TX */

#ifndef USB_CH_ERR_RX
#define USB_CH_ERR_RX                                         (USB_CHEP_ERR_RX)
#endif /* USB_CH_ERR_RX */

#ifndef USB_EP_UTYPE
#define USB_EP_UTYPE                                           (USB_CHEP_UTYPE)
#endif /* USB_EP_UTYPE */

#ifndef USB_CH_UTYPE
#define USB_CH_UTYPE                                           (USB_CHEP_UTYPE)
#endif /* USB_CH_UTYPE */

#ifndef USB_CH_STATTX
#define USB_CH_STATTX                                         (USB_CHEP_STATTX)
#endif /* USB_CH_STATTX */


/**
  * @brief  Maximum number of channels/endpoints.
  */
#define USB_DRD_MAX_CHEP_NBR                                               (8U)

/**
  * @brief  Maximum number of CTR iterations.
  */
#define USB_DRD_MAX_CTR_ITERATIONS                                        (32U)

/**
  * @brief  USB DRD CNTRX_NBLK mask.
  */
#define USB_DRD_CNTRX_NBLK_MSK                                  (0x1FUL << 26U)

/**
  * @brief  USB DRD CNTRX_BLSIZE mask.
  */
#define USB_DRD_CNTRX_BLSIZE                                     (0x1UL << 31U)

/**
  * @brief  PMA RX counter.
  */
#ifndef USB_DRD_RX_PMA_CNT
#define USB_DRD_RX_PMA_CNT                                                (10U)
#endif /* USB_DRD_RX_PMA_CNT */

/**
  * @brief  Power-down exit count.
  */
#define USB_DRD_PDWN_EXIT_CNT                                          (0x100U)

/**
  * @brief  USB DRD PMA lookup table size.
  *         8 bytes per block, 32 bits per word.
  */
#define USB_DRD_PMA_BLOCKS                    ((USB_DRD_PMA_SIZE) / (8U * 32U))
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup USB_DRD_CORE_Exported_Types Exported Types
  * @{
  */
/**
  * @brief  USB DRD core typedef definition.
  */
typedef USB_DRD_TypeDef usb_drd_global_t;

/**
  * @brief  USB endpoint configuration structure definition.
  */
typedef struct
{
  usb_core_channel_t virtual_ch_num;  /*!< Virtual channel number associated to the endpoint number
                                           This parameter can be a number between Min_Data = 1 and Max_Data = 15 */

  usb_core_ep_direction_t dir;        /*!< Endpoint direction state
                                           This parameter can be a number between Min_Data = 0 and Max_Data = 1  */

  uint8_t is_allocated;               /*!< Endpoint allocation state
                                           This parameter can be a number between Min_Data = 0 and Max_Data = 1  */

  uint8_t is_dual_allocated;          /*!< Endpoint dual allocation state
                                           This parameter can be a number between Min_Data = 0 and Max_Data = 1  */

  uint16_t pma_address;               /*!< PMA Address
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K */

  uint16_t pma_addr0;                 /*!< PMA Address0
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K */

  uint16_t pma_addr1;                 /*!< PMA Address1
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K */

} usb_drd_ep_config_t;


/**
  * @brief  USB DRD double buffer structure definition.
  */
typedef enum
{
  USB_DRD_SNG_BUF = USB_CORE_CONFIG_DISABLED,   /*!< USB double buffer state disabled: 0 */
  USB_DRD_DBL_BUF = USB_CORE_CONFIG_ENABLED,    /*!< USB double buffer state enabled:  1 */
} usb_drd_doublebuffer_t;


/**
  * @brief  USB endpoint double buffer configuration structure definition.
  */
typedef struct
{
  usb_drd_doublebuffer_t is_bulk_db;      /*!< Bulk endpoint double buffer state.
                                               This parameter can be a number between Min_Data = 0 and Max_Data = 1  */

  usb_drd_doublebuffer_t is_iso_db;       /*!< Isochronous endpoint double buffer state.
                                               This parameter can be a number between Min_Data = 0 and Max_Data = 1  */
} usb_drd_ep_db_config_t;

/**
  * @brief  USB DRD DoubleBuffer API structure definition.
  */
typedef enum
{
  USB_DRD_BULK_DB_ENABLE  = 0x01U,    /*!< USB DRD bulk double buffer enable */
  USB_DRD_BULK_DB_DISABLE = 0x02U,    /*!< USB DRD bulk double buffer disable */
  USB_DRD_ISOC_DB_ENABLE  = 0x03U,    /*!< USB DRD isochronous double buffer enable */
  USB_DRD_ISOC_DB_DISABLE = 0x04U,    /*!< USB DRD isochronous double buffer disable */
} usb_drd_db_status_t;

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DRD_Exported_Macros Exported Macros
  * @brief Macros to handle interrupts and specific clock configurations.
  * @{
  */
/**
  * @brief Get the USB DRD instance pointer.
  */
#define USB_DRD_GET_INSTANCE(instance)       ((usb_drd_global_t *)((uint32_t)instance))

/**
  * @brief Get the physical channel number from ISTR.
  */
#define USB_DRD_GET_CHNUM(instance)          (((instance)->ISTR) & USB_ISTR_IDN)

/**
  * @brief Get the channel direction from ISTR.
  */
#define USB_DRD_GET_CHDIR(instance)          (((instance)->ISTR) & USB_ISTR_DIR)

/**
  * @brief Get buffer 0 count.
  */
#define USB_DRD_GET_CHEP_DBUF0_CNT              USB_DRD_GET_CHEP_TX_CNT

/**
  * @brief  Get buffer 1 count.
  */
#define USB_DRD_GET_CHEP_DBUF1_CNT              USB_DRD_GET_CHEP_RX_CNT

/**
  * @brief  Get TX endpoint count.
  */
#define USB_DRD_GET_EP_TX_CNT                   USB_DRD_GET_CHEP_TX_CNT

/**
  * @brief  Get TX channel count.
  */
#define USB_DRD_GET_CH_TX_CNT                   USB_DRD_GET_CHEP_TX_CNT

/**
  * @brief Set endpoint register value.
  */
#define USB_DRD_PCD_SET_ENDPOINT                       USB_DRD_SET_CHEP

/**
  * @brief Get endpoint register value.
  */
#define USB_DRD_PCD_GET_ENDPOINT                       USB_DRD_GET_CHEP

/**
  * @brief  Set the TX status bits (STAT_TX[1:0]).
  */
#define USB_DRD_PCD_SET_EP_TX_STATUS                   USB_DRD_SET_CHEP_TX_STATUS

/**
  * @brief  Set the RX status bits (STAT_RX[1:0]).
  */
#define USB_DRD_PCD_SET_EP_RX_STATUS                   USB_DRD_SET_CHEP_RX_STATUS

/**
  * @brief  Set the EP_KIND bit.
  */
#define USB_DRD_PCD_SET_EP_KIND                        USB_DRD_SET_CHEP_KIND

/**
  * @brief  Clear the EP_KIND bit.
  */
#define USB_DRD_PCD_CLEAR_EP_KIND                      USB_DRD_CLEAR_CHEP_KIND

/**
  * @brief  Set endpoint bulk double-buffer mode.
  */
#define USB_DRD_PCD_SET_BULK_EP_DBUF                   USB_DRD_SET_CHEP_KIND

/**
  * @brief  Clear endpoint bulk double-buffer mode.
  */
#define USB_DRD_PCD_CLEAR_BULK_EP_DBUF                 USB_DRD_CLEAR_CHEP_KIND

/**
  * @brief  Set STATUS_OUT in the endpoint register.
  */
#define USB_DRD_PCD_SET_OUT_STATUS                     USB_DRD_SET_CHEP_KIND

/**
  * @brief  Clear STATUS_OUT in the endpoint register.
  */
#define USB_DRD_PCD_CLEAR_OUT_STATUS                   USB_DRD_CLEAR_CHEP_KIND

/**
  * @brief  Clear CTR_RX in the endpoint register.
  */
#define USB_DRD_PCD_CLEAR_RX_EP_CTR                    USB_DRD_CLEAR_RX_CHEP_CTR

/**
  * @brief  Clear CTR_TX in the endpoint register.
  */
#define USB_DRD_PCD_CLEAR_TX_EP_CTR                    USB_DRD_CLEAR_TX_CHEP_CTR

/**
  * @brief  Toggle DTOG_RX in the endpoint register.
  */
#define USB_DRD_PCD_RX_DTOG                            USB_DRD_RX_DTOG

/**
  * @brief  Toggle DTOG_TX in the endpoint register.
  */
#define USB_DRD_PCD_TX_DTOG                            USB_DRD_TX_DTOG

/**
  * @brief  Clear DTOG_RX in the endpoint register.
  */
#define USB_DRD_PCD_CLEAR_RX_DTOG                      USB_DRD_CLEAR_RX_DTOG

/**
  * @brief  Clear DTOG_TX in the endpoint register.
  */
#define USB_DRD_PCD_CLEAR_TX_DTOG                      USB_DRD_CLEAR_TX_DTOG

/**
  * @brief  Set the address field in an endpoint register.
  */
#define USB_DRD_PCD_SET_EP_ADDRESS                     USB_DRD_SET_CHEP_ADDRESS

/**
  * @brief  Set the TX buffer address.
  */
#define USB_DRD_PCD_SET_EP_TX_ADDRESS                  USB_DRD_SET_CHEP_TX_ADDRESS

/**
  * @brief  Set the RX buffer address.
  */
#define USB_DRD_PCD_SET_EP_RX_ADDRESS                  USB_DRD_SET_CHEP_RX_ADDRESS

/**
  * @brief  Set the TX buffer count.
  */
#define USB_DRD_PCD_SET_EP_TX_CNT                      USB_DRD_SET_CHEP_TX_CNT

/**
  * @brief  Set the RX buffer count.
  */
#define USB_DRD_PCD_SET_EP_RX_CNT                      USB_DRD_SET_CHEP_RX_CNT

/**
  * @brief  Get the TX buffer count.
  */
#define USB_DRD_PCD_GET_EP_TX_CNT                      USB_DRD_GET_CHEP_TX_CNT

/**
  * @brief  Get the RX buffer count.
  */
#define USB_DRD_PCD_GET_EP_RX_CNT                      USB_DRD_GET_EP_RX_CNT

/**
  * @brief  Set buffer addresses for a double-buffer endpoint.
  */
#define USB_DRD_PCD_SET_EP_DBUF_ADDR                   USB_DRD_SET_CHEP_DBUF_ADDR

/**
  * @brief  Set buffer 0 count for a double-buffer endpoint.
  */
#define USB_DRD_PCD_SET_EP_DBUF0_CNT                   USB_DRD_SET_CHEP_DBUF0_CNT

/**
  * @brief  Set buffer 1 count for a double-buffer endpoint.
  */
#define USB_DRD_PCD_SET_EP_DBUF1_CNT                   USB_DRD_SET_CHEP_DBUF1_CNT

/**
  * @brief  Set buffer counts for a double-buffer endpoint.
  */
#define USB_DRD_PCD_SET_EP_DBUF_CNT                    USB_DRD_SET_CHEP_DBUF_CNT

/**
  * @brief  Get endpoint double-buffer 0 RX count.
  */
#define USB_DRD_PCD_GET_EP_DBUF0_CNT                   USB_DRD_GET_EP_DBUF0_CNT

/**
  * @brief  Get endpoint double-buffer 1 RX count.
  */
#define USB_DRD_PCD_GET_EP_DBUF1_CNT                   USB_DRD_GET_EP_DBUF1_CNT

/**
  * @brief  Set channel register value.
  */
#define USB_DRD_HCD_SET_CHANNEL                        USB_DRD_SET_CHEP

/**
  * @brief  Get channel register value.
  */
#define USB_DRD_HCD_GET_CHANNEL                        USB_DRD_GET_CHEP

/**
  * @brief Set the SETUP bit to request a setup transaction.
  */
#define USB_DRD_HCD_SET_CH_TX_SETUP                    USB_DRD_CHEP_TX_SETUP

/**
  * @brief  Set the TX status bits (STAT_TX[1:0]).
  */
#define USB_DRD_HCD_SET_CH_TX_STATUS                   USB_DRD_SET_CHEP_TX_STATUS

/**
  * @brief  Set the RX status bits (STAT_RX[1:0]).
  */
#define USB_DRD_HCD_SET_CH_RX_STATUS                   USB_DRD_SET_CHEP_RX_STATUS

/**
  * @brief  Get the TX status bits (STAT_TX[1:0]).
  */
#define USB_DRD_HCD_GET_CH_TX_STATUS                   USB_DRD_GET_CHEP_TX_STATUS

/**
  * @brief  Get the RX status bits (STAT_RX[1:0]).
  */
#define USB_DRD_HCD_GET_CH_RX_STATUS                   USB_DRD_GET_CHEP_RX_STATUS

/**
  * @brief  Set the CH_KIND bit in the channel register.
  */
#define USB_DRD_HCD_SET_CH_KIND                        USB_DRD_SET_CH_KIND

/**
  * @brief  Clear the CH_KIND bit in the channel register.
  */
#define USB_DRD_HCD_CLEAR_CH_KIND                      USB_DRD_CLEAR_CH_KIND

/**
  * @brief  Set bulk channel double-buffer mode.
  */
#define USB_DRD_HCD_SET_BULK_CH_DBUF                   USB_DRD_HCD_SET_CH_KIND

/**
  * @brief  Clear bulk channel double-buffer mode.
  */
#define USB_DRD_HCD_CLEAR_BULK_CH_DBUF                 USB_DRD_HCD_CLEAR_CH_KIND

/**
  * @brief  Clear ERR_RX in the channel register.
  */
#define USB_DRD_HCD_CLEAR_RX_CH_ERR                    USB_DRD_CLEAR_CHEP_RX_ERR

/**
  * @brief  Clear ERR_TX in the channel register.
  */
#define USB_DRD_HCD_CLEAR_TX_CH_ERR                    USB_DRD_CLEAR_CHEP_TX_ERR

/**
  * @brief  Clear CTR_RX in the endpoint register.
  */
#define USB_DRD_HCD_CLEAR_RX_CH_CTR                    USB_DRD_CLEAR_RX_CHEP_CTR

/**
  * @brief  Clear CTR_TX in the endpoint register.
  */
#define USB_DRD_HCD_CLEAR_TX_CH_CTR                    USB_DRD_CLEAR_TX_CHEP_CTR

/**
  * @brief  Toggle DTOG_RX in the endpoint register.
  */
#define USB_DRD_HCD_RX_DTOG                            USB_DRD_RX_DTOG

/**
  * @brief  Toggle DTOG_TX in the endpoint register.
  */
#define USB_DRD_HCD_TX_DTOG                            USB_DRD_TX_DTOG

/**
  * @brief  Clear DTOG_RX in the endpoint register.
  */
#define USB_DRD_HCD_CLEAR_RX_DTOG                      USB_DRD_CLEAR_RX_DTOG

/**
  * @brief  Clear DTOG_TX in the endpoint register.
  */
#define USB_DRD_HCD_CLEAR_TX_DTOG                      USB_DRD_CLEAR_TX_DTOG

/**
  * @brief  Set the TX buffer count.
  */
#define USB_DRD_HCD_SET_CH_TX_CNT                      USB_DRD_SET_CHEP_TX_CNT

/**
  * @brief  Set the RX buffer count.
  */
#define USB_DRD_HCD_SET_CH_RX_CNT                      USB_DRD_SET_CHEP_RX_CNT

/**
  * @brief  Get the TX buffer count.
  */
#define USB_DRD_HCD_GET_CH_TX_CNT                      USB_DRD_GET_CHEP_TX_CNT

/**
  * @brief  Get the RX buffer count.
  */
#define USB_DRD_HCD_GET_CH_RX_CNT                      USB_DRD_GET_CH_RX_CNT

/**
  * @brief  Set buffer 0 count for a double-buffer endpoint.
  */
#define USB_DRD_HCD_SET_CH_DBUF0_CNT                   USB_DRD_SET_CHEP_DBUF0_CNT

/**
  * @brief  Set buffer 1 count for a double-buffer endpoint.
  */
#define USB_DRD_HCD_SET_CH_DBUF1_CNT                   USB_DRD_SET_CHEP_DBUF1_CNT

/**
  * @brief  Set buffer counts for a double-buffer endpoint.
  */
#define USB_DRD_HCD_SET_CH_DBUF_CNT                    USB_DRD_SET_CHEP_DBUF_CNT

/**
  * @brief  Get channel double-buffer 0 RX count.
  */
#define USB_DRD_HCD_GET_CH_DBUF0_CNT                   USB_DRD_GET_CH_DBUF0_CNT

/**
  * @brief  Get channel double-buffer 1 RX count.
  */
#define USB_DRD_HCD_GET_CH_DBUF1_CNT                   USB_DRD_GET_CH_DBUF1_CNT

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup USB_DRD_CORE_Exported_Functions Exported Core Functions
  * @{
  */
usb_core_status_t USB_DRD_InitCore(uint32_t instance, const usb_core_config_params_t *p_core_config);
usb_core_status_t USB_DRD_DeInitCore(uint32_t instance);
usb_core_status_t USB_DRD_EnableGlobalInterrupt(uint32_t instance);
usb_core_status_t USB_DRD_DisableGlobalInterrupt(uint32_t instance);
usb_core_status_t USB_DRD_SetCurrentMode(uint32_t instance, usb_core_mode_t core_mode);
usb_core_mode_t USB_DRD_GetCurrentMode(uint32_t instance);

uint32_t USB_DRD_ReadInterrupts(uint32_t instance);
void     USB_DRD_ClearInterrupts(uint32_t instance, uint32_t interrupt);
void     USB_DRD_WritePMA(uint32_t instance, uint8_t *p_src, uint16_t pma_address, uint16_t size_byte);
void     USB_DRD_ReadPMA(uint32_t instance, uint8_t *p_dest, uint16_t pma_address, uint16_t size_byte);

uint32_t USB_DRD_GET_CHEP(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_SET_CHEP(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t reg_value);
void USB_DRD_CLEAR_TX_CHEP_CTR(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_CLEAR_RX_CHEP_CTR(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_SET_CHEP_TX_STATUS(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t ep_ch_state);
void USB_DRD_SET_CHEP_RX_STATUS(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t ep_ch_state);
void USB_DRD_CLEAR_TX_DTOG(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_CLEAR_RX_DTOG(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_SET_CHEP_TX_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t tx_count);
void USB_DRD_SET_CHEP_RX_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t rx_count);
void USB_DRD_CLEAR_CHEP_TX_ERR(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_CLEAR_CHEP_RX_ERR(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_TX_DTOG(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_RX_DTOG(uint32_t instance, usb_core_phy_chep_t ch_ep_num);

void USB_DRD_SET_CHEP_DBUF0_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num,
                                usb_core_ep_direction_t direction, uint32_t count);

void USB_DRD_SET_CHEP_DBUF1_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num,
                                usb_core_ep_direction_t direction, uint32_t count);

void USB_DRD_CHEP_TX_SETUP(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_SET_CHEP_KIND(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_CLEAR_CHEP_KIND(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
void USB_DRD_SET_CHEP_ADDRESS(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t address);
void USB_DRD_SET_CHEP_TX_ADDRESS(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t address);
void USB_DRD_SET_CHEP_RX_ADDRESS(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t address);
void USB_DRD_SET_CHEP_CNT_RX_REG(volatile uint32_t *p_rx_count, uint32_t rx_count);
void USB_DRD_SET_CHEP_RX_DBUF0_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t rx_count);
void USB_DRD_SET_CHEP_DBUF0_ADDR(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t buff0_addr);
void USB_DRD_SET_CHEP_DBUF1_ADDR(uint32_t instance, usb_core_phy_chep_t ch_ep_num, uint32_t buff1_addr);

void USB_DRD_SET_CHEP_DBUF_ADDR(uint32_t instance, usb_core_phy_chep_t ch_ep_num,
                                uint32_t buff0_addr, uint32_t buff1_addr);

void USB_DRD_SET_CHEP_DBUF_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num,
                               usb_core_ep_direction_t direction, uint32_t count);

uint16_t USB_DRD_GET_CHEP_TX_STATUS(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
uint16_t USB_DRD_GET_CHEP_RX_STATUS(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
uint16_t USB_DRD_GET_CHEP_TX_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
uint16_t USB_DRD_GET_CHEP_RX_CNT(uint32_t instance, usb_core_phy_chep_t ch_ep_num);
/**
  * @}
  */

/** @defgroup USB_DRD_CORE_Device_Exported_Functions Exported Device Functions
  * @{
  */
usb_core_status_t USB_DRD_PCD_InitDriver(usb_core_pcd_driver_t *p_driver);
usb_core_status_t USB_DRD_InitDevice(uint32_t instance, const usb_core_config_params_t *p_core_config);
usb_core_status_t USB_DRD_StartDevice(uint32_t instance);
usb_core_status_t USB_DRD_StopDevice(uint32_t instance);
usb_core_status_t USB_DRD_ActivateEndpoint(uint32_t instance, usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_DeactivateEndpoint(uint32_t instance, const usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_StartEndpointXfer(uint32_t instance, usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_StopEndpointXfer(uint32_t instance, const usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_SetEndpointStall(uint32_t instance, const usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_ClearEndpointStall(uint32_t instance, const usb_core_ep_t *p_ep);
usb_core_status_t USB_DRD_SetDeviceAddress(uint32_t instance, uint8_t address);
usb_core_status_t USB_DRD_ConnectDevice(uint32_t instance);
usb_core_status_t USB_DRD_DisconnectDevice(uint32_t instance);
usb_core_device_speed_t USB_DRD_GetDeviceSpeed(uint32_t instance);

usb_core_status_t  USB_DRD_BCD_SetMode(uint32_t instance,
                                       usb_core_bcd_config_t bcd_config, usb_core_bcd_config_sts_t bcd_sts);
usb_core_bcd_port_status_t USB_DRD_BCD_SetPortDetection(uint32_t instance, usb_core_bcd_detection_t detection);
usb_core_status_t USB_DRD_BCD_Activate(uint32_t instance);
usb_core_status_t USB_DRD_BCD_DeActivate(uint32_t instance);

usb_core_status_t USB_DRD_LPM_Activate(uint32_t instance);
usb_core_status_t USB_DRD_LPM_DeActivate(uint32_t instance);

usb_core_status_t USB_DRD_ActivateRemoteWakeup(uint32_t instance);
usb_core_status_t USB_DRD_DeActivateRemoteWakeup(uint32_t instance);

uint16_t USB_DRD_GET_EP_RX_CNT(uint32_t instance, usb_core_phy_chep_t ep_num);
uint16_t USB_DRD_GET_EP_DBUF0_CNT(uint32_t instance, usb_core_phy_chep_t ep_num);
uint16_t USB_DRD_GET_EP_DBUF1_CNT(uint32_t instance, usb_core_phy_chep_t ep_num);
/**
  * @}
  */

/** @defgroup USB_DRD_CORE_Host_Exported_Functions Exported Host Functions
  * @{
  */
usb_core_status_t USB_DRD_HCD_InitDriver(usb_core_hcd_driver_t *p_driver);
usb_core_status_t USB_DRD_PortReset(uint32_t instance, usb_core_port_reset_sts_t reset_status);
usb_core_status_t USB_DRD_PortSuspend(uint32_t instance);
usb_core_status_t USB_DRD_PortResume(uint32_t instance, usb_core_port_resume_sts_t resume_status);
usb_core_status_t USB_DRD_InitHost(uint32_t instance, const usb_core_config_params_t *p_core_config);
usb_core_status_t USB_DRD_StartHost(uint32_t instance);
usb_core_status_t USB_DRD_StopHost(uint32_t instance);
usb_core_port_speed_t USB_DRD_GetHostPortSpeed(uint32_t instance);
usb_core_status_t USB_DRD_PMAReset(void);
uint32_t USB_DRD_GetCurrentFrame(uint32_t instance);
uint32_t USB_DRD_GetDmaStatus(uint32_t instance);

usb_core_status_t USB_DRD_InitChannel(uint32_t instance, usb_core_ch_t *p_ch);
usb_core_status_t USB_DRD_CloseChannel(uint32_t instance, usb_core_ch_t *p_ch);
usb_core_status_t USB_DRD_HaltInChannel(uint32_t instance, usb_core_phy_chep_t phy_ch_num);
usb_core_status_t USB_DRD_HaltOutChannel(uint32_t instance, usb_core_phy_chep_t phy_ch_num);
usb_core_status_t USB_DRD_HaltChannel(uint32_t instance, const usb_core_ch_t *p_ch);
usb_core_status_t USB_DRD_StartChannelXfer(uint32_t instance, usb_core_ch_t *p_ch);
usb_core_channel_t USB_DRD_GetLogicalChannel(usb_core_phy_chep_t phy_ch_num, usb_core_ch_direction_t ch_dir);
void     USB_DRD_ClearPhysicalChannels(void);

uint16_t USB_DRD_GET_CH_RX_CNT(uint32_t instance, usb_core_phy_chep_t phy_ch_num);
uint16_t USB_DRD_GET_CH_DBUF0_CNT(uint32_t instance, usb_core_phy_chep_t phy_ch_num);
uint16_t USB_DRD_GET_CH_DBUF1_CNT(uint32_t instance, usb_core_phy_chep_t phy_ch_num);
/**
  * @}
  */
/**
  * @}
  */
#endif /* defined (USB_DRD_FS) */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_USB_DRD_CORE_H */
