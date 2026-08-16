/**
  ******************************************************************************
  * @file    stm32c5xx_hal_hcd.h
  * @brief   Header file for the HCD HAL module.
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
#ifndef STM32C5XX_HAL_HCD_H
#define STM32C5XX_HAL_HCD_H

#ifdef __cplusplus
extern "C" {
#endif /* defined __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_usb_drd_core.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (USB_DRD_FS)

/** @defgroup HCD USB Host controller driver
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros Private Macros
  * @{
  */
/*! Macro to get the min value */
#define HCD_MIN(a,b) (((a) < (b)) ? (a) : (b))

/*! Macro to get the max value */
#define HCD_MAX(a,b) (((a) > (b)) ? (a) : (b))
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup HCD_Private_Constants Private Constants
  * @{
  */
/**
  * @brief  USE HAL HCD MAX Channel number.
  */
#ifndef USE_HAL_HCD_MAX_CHANNEL_NB
#define USE_HAL_HCD_MAX_CHANNEL_NB                                          16U
#endif /* USE_HAL_HCD_MAX_CHANNEL_NB */

/**
  * @brief  USE HAL HCD USB EP TYPE ISOC.
  */
#ifndef USE_HAL_HCD_USB_EP_TYPE_ISOC
#define USE_HAL_HCD_USB_EP_TYPE_ISOC                                         1U
#endif /* USE_HAL_HCD_USB_EP_TYPE_ISOC */

/**
  * @brief  USE HAL HCD USB Channel Double buffer.
  */
#ifndef USE_HAL_HCD_USB_DOUBLE_BUFFER
#define USE_HAL_HCD_USB_DOUBLE_BUFFER                                        1U
#endif /* USE_HAL_HCD_USB_DOUBLE_BUFFER */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HCD_Exported_Constants Exported Constants
  * @{
  */

#if defined (USE_HAL_HCD_GET_LAST_ERRORS) && (USE_HAL_HCD_GET_LAST_ERRORS == 1)
/** @defgroup HCD_Error_Codes Error Codes
  * @{
  */
/*!< No error */
#define HAL_HCD_ERROR_NONE                                               (0UL)
/*!< HAL HCD CHANNEL transfer ERROR */
#define HAL_HCD_ERROR_CHANNEL_TRANSFER                                   (1UL << 0U)
/*!< HAL HCD CHANNEL babble ERROR */
#define HAL_HCD_ERROR_CHANNEL_BABBLE                                     (1UL << 1U)
/*!< HAL HCD CHANNEL DATA Toggle ERROR */
#define HAL_HCD_ERROR_CHANNEL_DATA_TOGGLE                                (1UL << 2U)
/**
  * @}
  */
#endif /* USE_HAL_HCD_GET_LAST_ERRORS */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HCD_Exported_Types Exported Types
  * @{
  */

/**
  * @brief  HAL USB Instance definition
  */
typedef enum
{
  HAL_HCD_DRD_FS = USB_DRD_FS_BASE, /*!< USB DRD FS Instance */
} hal_hcd_t;


/**
  * @brief  HAL USB HCD States definition
  */
typedef enum
{
  HAL_HCD_STATE_RESET  = 0U,            /*!< HAL HCD STATE RESET  */
  HAL_HCD_STATE_INIT   = (1U << 31U),   /*!< HAL HCD STATE INIT   */
  HAL_HCD_STATE_IDLE   = (1U << 30U),   /*!< HAL HCD STATE IDLE   */
  HAL_HCD_STATE_ACTIVE = (1U << 29U),   /*!< HAL HCD STATE ACTIVE */
  HAL_HCD_STATE_FAULT  = (1U << 28U),   /*!< HAL HCD STATE FAULT  */
} hal_hcd_state_t;


/**
  * @brief HAL USB HCD Port State definition
  */
typedef enum
{
  HAL_HCD_PORT_STATE_DEV_DISCONNECT = (1U << 31U),   /*!< HAL HCD PORT STATE Device DISCONNECT */
  HAL_HCD_PORT_STATE_DEV_CONNECT    = (1U << 30U),   /*!< HAL HCD PORT STATE Device CONNECT    */
  HAL_HCD_PORT_STATE_DEV_RESET      = (1U << 29U),   /*!< HAL HCD PORT STATE Device RESET      */
  HAL_HCD_PORT_STATE_DEV_RUN        = (1U << 28U),   /*!< HAL HCD PORT STATE Device RUN        */
  HAL_HCD_PORT_STATE_DEV_SUSPEND    = (1U << 27U),   /*!< HAL HCD PORT STATE Device SUSPEND    */
  HAL_HCD_PORT_STATE_DEV_RESUME     = (1U << 26U),   /*!< HAL HCD PORT STATE Device RESUME     */
} hal_hcd_port_state_t;


/**
  * @brief  HAL USB HCD Host channel States definition
  */
typedef enum
{
  HAL_HCD_CHANNEL_STATE_RESET      = (1U << 31U),   /*!< HAL HCD CHANNEL STATE RESET      */
  HAL_HCD_CHANNEL_STATE_IDLE       = (1U << 30U),   /*!< HAL HCD CHANNEL STATE IDLE       */
  HAL_HCD_CHANNEL_STATE_XFRC       = (1U << 29U),   /*!< HAL HCD CHANNEL STATE XFRC       */
  HAL_HCD_CHANNEL_STATE_HALTED     = (1U << 28U),   /*!< HAL HCD CHANNEL STATE HALTED     */
  HAL_HCD_CHANNEL_STATE_ACK        = (1U << 27U),   /*!< HAL HCD CHANNEL STATE ACK        */
  HAL_HCD_CHANNEL_STATE_NAK        = (1U << 26U),   /*!< HAL HCD CHANNEL STATE NAK        */
  HAL_HCD_CHANNEL_STATE_STALL      = (1U << 25U),   /*!< HAL HCD CHANNEL STATE STALL      */
  HAL_HCD_CHANNEL_STATE_XFR        = (1U << 24U),   /*!< HAL HCD CHANNEL STATE XFR        */
  HAL_HCD_CHANNEL_STATE_XACTERR    = (1U << 23U),   /*!< HAL HCD CHANNEL STATE XACTERR    */
} hal_hcd_channel_state_t;


/**
  * @brief  HAL USB HCD CHANNEL URB States definition
  */
typedef enum
{
  HAL_HCD_CHANNEL_URB_STATE_RESET    = (1U << 31U),   /*!< HAL HCD CHANNEL URB STATE RESET    */
  HAL_HCD_CHANNEL_URB_STATE_IDLE     = (1U << 30U),   /*!< HAL HCD CHANNEL URB STATE IDLE     */
  HAL_HCD_CHANNEL_URB_STATE_DONE     = (1U << 29U),   /*!< HAL HCD CHANNEL URB STATE DONE     */
  HAL_HCD_CHANNEL_URB_STATE_NOTREADY = (1U << 28U),   /*!< HAL HCD CHANNEL URB STATE NOTREADY */
  HAL_HCD_CHANNEL_URB_STATE_ERROR    = (1U << 27U),   /*!< HAL HCD CHANNEL URB STATE ERROR    */
  HAL_HCD_CHANNEL_URB_STATE_STALL    = (1U << 26U)    /*!< HAL HCD CHANNEL URB STATE STALL    */
} hal_hcd_channel_urb_state_t;


/**
  * @brief HAL USB HCD CHANNEL identifier definition
  */
typedef enum
{
  HAL_HCD_CHANNEL_0  = USB_CORE_CHANNEL_0,    /*!< HAL HCD CHANNEL 0 */
  HAL_HCD_CHANNEL_1  = USB_CORE_CHANNEL_1,    /*!< HAL HCD CHANNEL 1 */
  HAL_HCD_CHANNEL_2  = USB_CORE_CHANNEL_2,    /*!< HAL HCD CHANNEL 2 */
  HAL_HCD_CHANNEL_3  = USB_CORE_CHANNEL_3,    /*!< HAL HCD CHANNEL 3 */
  HAL_HCD_CHANNEL_4  = USB_CORE_CHANNEL_4,    /*!< HAL HCD CHANNEL 4 */
  HAL_HCD_CHANNEL_5  = USB_CORE_CHANNEL_5,    /*!< HAL HCD CHANNEL 5 */
  HAL_HCD_CHANNEL_6  = USB_CORE_CHANNEL_6,    /*!< HAL HCD CHANNEL 6 */
  HAL_HCD_CHANNEL_7  = USB_CORE_CHANNEL_7,    /*!< HAL HCD CHANNEL 7 */
  HAL_HCD_CHANNEL_8  = USB_CORE_CHANNEL_8,    /*!< HAL HCD CHANNEL 8 */
  HAL_HCD_CHANNEL_9  = USB_CORE_CHANNEL_9,    /*!< HAL HCD CHANNEL 9 */
  HAL_HCD_CHANNEL_10 = USB_CORE_CHANNEL_10,   /*!< HAL HCD CHANNEL 10 */
  HAL_HCD_CHANNEL_11 = USB_CORE_CHANNEL_11,   /*!< HAL HCD CHANNEL 11 */
  HAL_HCD_CHANNEL_12 = USB_CORE_CHANNEL_12,   /*!< HAL HCD CHANNEL 12 */
  HAL_HCD_CHANNEL_13 = USB_CORE_CHANNEL_13,   /*!< HAL HCD CHANNEL 13 */
  HAL_HCD_CHANNEL_14 = USB_CORE_CHANNEL_14,   /*!< HAL HCD CHANNEL 14 */
  HAL_HCD_CHANNEL_15 = USB_CORE_CHANNEL_15,   /*!< HAL HCD CHANNEL 15 */
  HAL_HCD_CHANNEL_FF = USB_CORE_CHANNEL_FF,   /*!< HAL HCD CHANNEL FF */
} hal_hcd_channel_t;


/**
  * @brief  HAL USB HCD Speed structure definition
  */
typedef enum
{
  HAL_HCD_SPEED_FS       = USB_CORE_SPEED_FS,         /*!< HAL HCD SPEED FULL SPEED               */
} hal_hcd_speed_t;


/**
  * @brief HAL USB HCD Device Speed structure definition
  */
typedef enum
{
  HAL_HCD_DEVICE_SPEED_LS    = USB_CORE_DEVICE_SPEED_LS,     /*!< HAL HCD DEVICE SPEED LOW   */
  HAL_HCD_DEVICE_SPEED_FS    = USB_CORE_DEVICE_SPEED_FS,     /*!< HAL HCD DEVICE SPEED FULL  */
  HAL_HCD_DEVICE_SPEED_ERROR = USB_CORE_DEVICE_SPEED_ERROR   /*!< HAL HCD DEVICE SPEED ERROR */
} hal_hcd_device_speed_t;


/**
  * @brief  HAL USB HCD Port Speed structure definition
  */
typedef enum
{
  HAL_HCD_PORT_SPEED_FS = USB_CORE_PORT_SPEED_FS,   /*!< HAL HCD PORT SPEED FULL */
  HAL_HCD_PORT_SPEED_LS = USB_CORE_PORT_SPEED_LS,   /*!< HAL HCD PORT SPEED LOW  */
} hal_hcd_port_speed_t;


/**
  * @brief  HAL USB HCD PHY Module structure definition
  */
typedef enum
{
  HAL_HCD_PHY_EMBEDDED_FS   = USB_CORE_PHY_EMBEDDED_FS,     /*!< HAL HCD EMBEDDED FS PHY      */
} hal_hcd_phy_module_t;


/**
  * @brief  HAL USB HCD Channel Direction structure definition
  */
typedef enum
{
  HAL_HCD_CH_OUT_DIR = USB_CORE_CH_OUT_DIR,   /*!< HAL HCD CH OUT DIR: 0 */
  HAL_HCD_CH_IN_DIR  = USB_CORE_CH_IN_DIR,    /*!< HAL HCD CH IN DIR:  1 */
} hal_hcd_ch_direction_t;


/**
  * @brief  HAL USB HCD Endpoint Type structure definition
  */
typedef enum
{
  HAL_HCD_EP_TYPE_CTRL = USB_CORE_EP_TYPE_CTRL,
  HAL_HCD_EP_TYPE_ISOC = USB_CORE_EP_TYPE_ISOC,
  HAL_HCD_EP_TYPE_BULK = USB_CORE_EP_TYPE_BULK,
  HAL_HCD_EP_TYPE_INTR = USB_CORE_EP_TYPE_INTR,
} hal_hcd_ep_type_t;


/**
  * @brief  HAL USB HCD DMA status definition
  */
typedef enum
{
  HAL_HCD_DMA_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL HCD DMA DISABLED */
  HAL_HCD_DMA_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL HCD DMA ENABLED  */
} hal_hcd_dma_status_t;


/**
  * @brief  HAL USB HCD Bulk Double buffer status definition
  */
typedef enum
{
  HAL_HCD_BULK_DB_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL HCD BULK DB DISABLED */
  HAL_HCD_BULK_DB_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL HCD BULK DB ENABLED  */
} hal_hcd_bulk_db_status_t;


/**
  * @brief  HAL USB HCD ISO Double buffer status definition
  */
typedef enum
{
  HAL_HCD_ISO_DB_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL HCD ISO DB DISABLED */
  HAL_HCD_ISO_DB_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL HCD ISO DB ENABLED  */
} hal_hcd_iso_db_status_t;


/**
  * @brief  HAL USB HCD Instance configuration Structure definition
  */
typedef struct
{
  hal_hcd_speed_t hcd_speed;                                /*!< USB core speed                                       */

  hal_hcd_phy_module_t phy_interface;                       /*!< Select the used PHY interface.                       */

  hal_hcd_bulk_db_status_t bulk_doublebuffer_enable; /*!< Enable or disable the double buffer mode on bulk EP         */

  hal_hcd_iso_db_status_t iso_doublebuffer_enable;   /*!< Enable or disable the Single buffer mode on Isochronous  EP */
} hal_hcd_config_t;


/**
  * @brief  HAL USB HCD Channel configuration Structure definition
  */
typedef struct
{
  uint8_t ep_address;                    /*!< Endpoint address  */
  uint8_t device_address;                /*!< Device address    */
  uint16_t ep_mps;                       /*!< Endpoint MPS      */
  hal_hcd_device_speed_t device_speed;   /*!< Device speed      */
  hal_hcd_ep_type_t ep_type;             /*!< Endpoint type     */
} hal_hcd_channel_config_t;


/**
  * @brief  HAL USB HCD Channel transfer request Structure definition
  */
typedef struct
{
  hal_hcd_ch_direction_t ch_dir;       /*!< Channel direction  */
  hal_hcd_ep_type_t ep_type;           /*!< Endpoint type      */
  uint8_t token_type;                  /*!< token type         */
  uint8_t do_ping;                     /*!< Do ping            */
  uint16_t transfer_length;            /*!< transfer length    */
  uint8_t *p_buffer;                   /*!< buffer pointer     */
} hal_hcd_channel_transfer_req_t;


/**
  * @brief  HAL USB HCD channel Structure definition
  */
typedef struct
{
  usb_core_ch_t core_ch;                  /*!< Core Channel                                                           */

  hal_hcd_channel_state_t state;          /*!< Host Channel state
                                               This parameter can be any value of @ref hal_hcd_channel_state_t        */

  hal_hcd_channel_urb_state_t urb_state;  /*!< Channel URB state
                                               This parameter can be any value of @ref hal_hcd_channel_urb_state_t    */

  uint8_t toggle_in;                      /*!< IN transfer current toggle flag
                                               This parameter must be a number between Min_Data = 0 and Max_Data = 1  */

  uint8_t toggle_out;                     /*!< OUT transfer current toggle flag
                                               This parameter must be a number between Min_Data = 0 and Max_Data = 1  */

  uint32_t err_cnt;                       /*!< Host channel error count                                               */

  uint8_t ep_ss_schedule;                 /*!< Enable periodic endpoint start split schedule                          */

} hal_hcd_ch_t;

typedef struct hal_hcd_handle_s   hal_hcd_handle_t;   /*!< HCD handle structure type */

#if defined (USE_HAL_HCD_REGISTER_CALLBACKS) && (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  HAL USB HCD Callback pointer definition
  */
typedef void (*hal_hcd_cb_t)(hal_hcd_handle_t *hhcd);


/**
  * @brief  HAL USB HCD Host Channel Notify URB Change Callback pointer definition
  */
typedef void (*hal_hcd_ch_notify_urb_change_cb_t)(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num,
                                                  hal_hcd_channel_urb_state_t urb_state);

/**
  * @brief  HAL USB HCD Host Channel Callback pointer definition
  */
typedef void (*hal_hcd_ch_cb_t)(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);

#endif /* (USE_HAL_HCD_REGISTER_CALLBACKS) && (USE_HAL_HCD_REGISTER_CALLBACKS == 1U) */

/**
  * @brief  HAL USB HCD Handle Structure definition
  */
struct hal_hcd_handle_s
{
  hal_hcd_t instance;                                           /*!< Register base address                            */
  volatile hal_hcd_state_t global_state;                        /*!< HCD communication state                          */
  volatile hal_hcd_port_state_t port_state;                     /*!< HCD Port state                                   */

#if defined (USE_HAL_HCD_GET_LAST_ERRORS) && (USE_HAL_HCD_GET_LAST_ERRORS == 1)
  volatile uint32_t last_error_codes;                           /*!< Errors limited to the last process
                                                                     This parameter can be a combination of
                                                                     @ref HCD_Error_Codes                             */
#endif /* USE_HAL_HCD_GET_LAST_ERRORS */

  uint8_t host_channels_nbr;                                    /*!< Number of host channels                          */
  hal_hcd_ch_t channel[USE_HAL_HCD_MAX_CHANNEL_NB];             /*!< Host channels parameters                         */

  usb_core_mode_t current_mode;                                 /*!< store Current Mode                               */

  usb_core_hcd_driver_t driver;                                 /*!< USB low layer driver                             */
  void (* p_irq_handler)(struct hal_hcd_handle_s *hhcd);        /*!< USB instance interrupt handler                   */

#if defined (USE_HAL_HCD_USER_DATA) && (USE_HAL_HCD_USER_DATA == 1)
  const void *p_user_data;                                      /*!< User Data Pointer                                */
#endif /* USE_HAL_HCD_USER_DATA */

#if defined (USE_HAL_HCD_REGISTER_CALLBACKS) && (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
  hal_hcd_cb_t p_port_connect_cb;                               /*!< USB HCD Port Connect callback                    */
  hal_hcd_cb_t p_port_disconnect_cb;                            /*!< USB HCD Port Disconnect callback                 */
  hal_hcd_cb_t p_port_enable_cb;                                /*!< USB HCD Port Enable callback                     */
  hal_hcd_cb_t p_port_disable_cb;                               /*!< USB HCD Port Disable callback                    */
  hal_hcd_cb_t p_port_suspend_cb;                               /*!< USB HCD Port Suspend callback                    */
  hal_hcd_cb_t p_port_resume_cb;                                /*!< USB HCD Port Resume callback                     */
  hal_hcd_cb_t p_sof_cb;                                        /*!< USB HCD SOF callback                             */
  hal_hcd_ch_notify_urb_change_cb_t p_ch_notify_urb_change_cb;  /*!< USB HCD Host Channel Notify URB Change callback  */
  hal_hcd_ch_cb_t p_ch_abort_transfer_cb;                       /*!< USB HCD Channel Abort Transfer callback          */
  hal_hcd_cb_t p_error_cb;                                      /*!< USB HCD Error callback                           */
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */
};

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup HCD_Exported_Functions Exported Functions
  * @{
  */

/** @defgroup HCD_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
hal_status_t HAL_HCD_Init(hal_hcd_handle_t *hhcd, hal_hcd_t instance);
void         HAL_HCD_DeInit(hal_hcd_handle_t *hhcd);
/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group2 Global Configuration functions
  * @{
  */
hal_status_t HAL_HCD_SetConfig(hal_hcd_handle_t *hhcd, const hal_hcd_config_t *p_config);
/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group3 User Data functions
  * @{
  */
#if defined (USE_HAL_HCD_GET_LAST_ERRORS) && (USE_HAL_HCD_GET_LAST_ERRORS == 1)
uint32_t HAL_HCD_GetLastErrorCodes(const hal_hcd_handle_t *hhcd);
#endif /* USE_HAL_HCD_GET_LAST_ERRORS */

#if defined (USE_HAL_HCD_USER_DATA) && (USE_HAL_HCD_USER_DATA == 1)
void        HAL_HCD_SetUserData(hal_hcd_handle_t *hhcd, const void *p_user_data);
const void *HAL_HCD_GetUserData(const hal_hcd_handle_t *hhcd);
#endif /* USE_HAL_HCD_USER_DATA */
/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group4 Peripheral Control functions
  * @{
  */
hal_status_t HAL_HCD_Start(hal_hcd_handle_t *hhcd);
hal_status_t HAL_HCD_Stop(hal_hcd_handle_t *hhcd);
hal_status_t HAL_HCD_ResetPort(hal_hcd_handle_t *hhcd);
hal_status_t HAL_HCD_SuspendPort(hal_hcd_handle_t *hhcd);
hal_status_t HAL_HCD_ResumePort(hal_hcd_handle_t *hhcd);

uint32_t             HAL_HCD_GetChannelTransferCount(const hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);
uint32_t             HAL_HCD_GetCurrentFrame(const hal_hcd_handle_t *hhcd);
hal_hcd_port_speed_t HAL_HCD_GetPortSpeed(const hal_hcd_handle_t *hhcd);
hal_hcd_dma_status_t HAL_HCD_IsEnabledDMA(const hal_hcd_handle_t *hhcd);

hal_status_t HAL_HCD_HaltChannel(const hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);

hal_status_t HAL_HCD_SetConfigChannel(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num,
                                      const hal_hcd_channel_config_t *p_channel_config);

hal_status_t HAL_HCD_RequestChannelTransfer(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num,
                                            const hal_hcd_channel_transfer_req_t *p_channel_transfer_req);

hal_status_t HAL_HCD_SetChannelHubInfo(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num,
                                       uint8_t hub_addr, uint8_t port_nbr);

hal_status_t HAL_HCD_ClearChannelHubInfo(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);


hal_status_t HAL_HCD_CloseChannel(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group5 Peripheral State functions
  * @{
  */

hal_hcd_state_t             HAL_HCD_GetState(const hal_hcd_handle_t *hhcd);
hal_hcd_channel_urb_state_t HAL_HCD_GetChannelURBState(const hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);
hal_hcd_channel_state_t     HAL_HCD_GetChannelState(const hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group6 IRQ handling functions
  * @{
  */
/* Non-Blocking mode: Interrupt */
void HAL_HCD_IRQHandler(hal_hcd_handle_t *hhcd);


void HAL_HCD_DRD_IRQHandler(hal_hcd_handle_t *hhcd);


/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group7 Default Callbacks functions
  * @{
  */
void HAL_HCD_ErrorCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_SofCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortConnectCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortDisconnectCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortEnabledCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortDisabledCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortSuspendCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_PortResumeCallback(hal_hcd_handle_t *hhcd);
void HAL_HCD_ChannelNotifyURBChangeCallback(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num,
                                            hal_hcd_channel_urb_state_t urb_state);
void HAL_HCD_ChannelAbortTransferCallback(hal_hcd_handle_t *hhcd, hal_hcd_channel_t ch_num);

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group8 Register Callbacks functions
  * @{
  */
#if defined (USE_HAL_HCD_REGISTER_CALLBACKS) && (USE_HAL_HCD_REGISTER_CALLBACKS == 1U)
hal_status_t HAL_HCD_RegisterSofCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortConnectCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortDisconnectCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortEnabledCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortDisabledCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortSuspendCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterPortResumeCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterErrorCallback(hal_hcd_handle_t *hhcd, hal_hcd_cb_t p_callback);
hal_status_t HAL_HCD_RegisterChannelNotifyURBChangeCallback(hal_hcd_handle_t *hhcd,
                                                            hal_hcd_ch_notify_urb_change_cb_t p_callback);
hal_status_t HAL_HCD_RegisterChannelAbortTransferCallback(hal_hcd_handle_t *hhcd, hal_hcd_ch_cb_t p_callback);
#endif /* USE_HAL_HCD_REGISTER_CALLBACKS */

/**
  * @}
  */

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
#endif /* defined __cplusplus */

#endif /* STM32C5XX_HAL_HCD_H */
