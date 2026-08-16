/**
  ******************************************************************************
  * @file    stm32c5xx_hal_pcd.h
  * @brief   Header file for the PCD HAL module.
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
#ifndef STM32C5XX_HAL_PCD_H
#define STM32C5XX_HAL_PCD_H

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
/** @defgroup PCD USB Peripheral Controller Driver
  * @{
  */
/* Private constants ---------------------------------------------------------*/
/** @defgroup PCD_Private_Constants Private Constants
  * @{
  */

/**
  * @brief  USE HAL PCD USB EP TYPE ISOC.
  */
#ifndef USE_HAL_PCD_USB_EP_TYPE_ISOC
#define USE_HAL_PCD_USB_EP_TYPE_ISOC                                         1U
#endif /* USE_HAL_PCD_USB_EP_TYPE_ISOC */

/**
  * @brief  USE HAL PCD USB BCD.
  */
#ifndef USE_HAL_PCD_USB_BCD
#define USE_HAL_PCD_USB_BCD                                                  0U
#endif /* USE_HAL_PCD_USB_BCD */

/**
  * @brief  USE HAL PCD USB LPM.
  */
#ifndef USE_HAL_PCD_USB_LPM
#define USE_HAL_PCD_USB_LPM                                                  0U
#endif /* USE_HAL_PCD_USB_LPM */

/**
  * @brief  USE HAL PCD USB DOUBLE BUFFER.
  */
#ifndef USE_HAL_PCD_USB_DOUBLE_BUFFER
#define USE_HAL_PCD_USB_DOUBLE_BUFFER                                        1U
#endif /* USE_HAL_PCD_USB_DOUBLE_BUFFER */

/**
  * @brief  USE HAL PCD MAX ENDPOINT NB.
  */
#ifndef USE_HAL_PCD_MAX_ENDPOINT_NB
#define USE_HAL_PCD_MAX_ENDPOINT_NB                                          8U
#endif /* USE_HAL_PCD_MAX_ENDPOINT_NB */


/**
  * @brief  HAL USB PCD WAKEUP EXTI LINE.
  */
#define USB_WAKEUP_EXTI_LINE                                          (0x1U << 0U)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros Private Macros
  * @{
  */
/*! Macro to get the min value */
#define PCD_MIN(a,b) (((a) < (b)) ? (a) : (b))

/*! Macro to get the max value */
#define PCD_MAX(a,b) (((a) > (b)) ? (a) : (b))
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PCD_Exported_Constants Exported Constants
  * @{
  */
#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
/** @defgroup PCD_Error_Codes Error Codes
  * @{
  */
#define HAL_PCD_ERROR_NONE                          (0x0U)           /*!< No error                   */
#define HAL_PCD_ERROR_BCD                           (0x1U << 0U)     /*!< USB Battery Charging error */
#define HAL_PCD_ERROR_IN_EP_AHB                     (0x1U << 1U)     /*!< USB IN EP AHB error        */
#define HAL_PCD_ERROR_IN_EP_TIMEOUT                 (0x1U << 2U)     /*!< USB IN EP TIMEOUT error    */
#define HAL_PCD_ERROR_IN_EP_BABBLE                  (0x1U << 3U)     /*!< USB IN EP BABBLE error     */
#define HAL_PCD_ERROR_OUT_EP_AHB                    (0x1U << 4U)     /*!< USB OUT EP AHB error       */
#define HAL_PCD_ERROR_OUT_EP_PACKET                 (0x1U << 5U)     /*!< USB OUT EP PACKET  error   */
#define HAL_PCD_ERROR_CTR_STUCK                     (0x1U << 6U)     /*!< USB Transaction error      */
#define HAL_PCD_ERROR_EP_INDEX                      (0x1U << 7U)     /*!< USB Endpoint index error   */

/**
  * @}
  */
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */
#define HAL_PCD_EP_ADDR_MSK                         (0x7FU)          /*!< Endpoint Address Mask      */
#define HAL_PCD_EP_IN_DIR              (USB_CORE_EP_IN_DIR)          /*!< Endpoint IN direction      */
#define HAL_PCD_EP_OUT_DIR            (USB_CORE_EP_OUT_DIR)          /*!< Endpoint OUT direction     */
#define HAL_PCD_SNG_BUF                   (USB_DRD_SNG_BUF)          /*!< USB Endpoint single buffer */
#define HAL_PCD_DBL_BUF                   (USB_DRD_DBL_BUF)          /*!< USB Endpoint double buffer */
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PCD_Exported_Types Exported Types
  * @{
  */

/**
  * @brief HAL USB Instance definition.
  */
typedef enum
{
  HAL_PCD_DRD_FS = USB_DRD_FS_BASE, /*!< USB DRD FS Instance */
} hal_pcd_t;

/**
  * @brief  HAL USB PCD State structure definition.
  */
typedef enum
{
  HAL_PCD_STATE_RESET     = 0U,            /*!< HAL PCD STATE RESET */
  HAL_PCD_STATE_INIT      = (1U << 31U),   /*!< HAL PCD STATE INIT  */
  HAL_PCD_STATE_IDLE      = (1U << 30U),   /*!< HAL PCD STATE IDLE  */
  HAL_PCD_STATE_ACTIVE    = (1U << 29U),   /*!< HAL PCD STATE ACTIVE */
  HAL_PCD_STATE_XFR_ABORT = (1U << 28U),   /*!< HAL PCD STATE TRANSFER ABORT */
  HAL_PCD_STATE_FAULT     = (1U << 27U),   /*!< HAL PCD STATE FAULT */
} hal_pcd_state_t;


/**
  * @brief HAL USB PCD Device State Options.
  */
typedef enum
{
  HAL_PCD_PORT_STATE_DEV_DISCONNECT = (1U << 31U),   /*!< HAL PCD PORT STATE Device DISCONNECT */
  HAL_PCD_PORT_STATE_DEV_CONNECT    = (1U << 30U),   /*!< HAL PCD PORT STATE Device CONNECT    */
  HAL_PCD_PORT_STATE_DEV_RESET      = (1U << 29U),   /*!< HAL PCD PORT STATE Device RESET      */
  HAL_PCD_PORT_STATE_DEV_RUN        = (1U << 28U),   /*!< HAL PCD PORT STATE Device RUN        */
  HAL_PCD_PORT_STATE_DEV_SUSPEND    = (1U << 27U),   /*!< HAL PCD PORT STATE Device SUSPEND    */
  HAL_PCD_PORT_STATE_DEV_RESUME     = (1U << 26U),   /*!< HAL PCD PORT STATE Device RESUME     */
} hal_pcd_port_state_t;


#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
/**
  * @brief  HAL USB PCD LPM State structure definition.
  */
typedef enum
{
  HAL_PCD_LPM_STATE_L0     = (1U << 31U),   /*!< PCD LPM STATE L0: on      */
  HAL_PCD_LPM_STATE_L1     = (1U << 30U),   /*!< PCD LPM STATE L1: sleep   */
} hal_pcd_lpm_state_t;
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */


/**
  * @brief  HAL USB PCD LPM message structure definition.
  */
typedef enum
{
  HAL_PCD_LPM_L0_ACTIVE = 0x00U, /*!< PCD LPM ACTIVE STATE L0: on    */
  HAL_PCD_LPM_L1_ACTIVE = 0x01U, /*!< PCD LPM ACTIVE STATE L1: sleep */
} hal_pcd_lpm_active_status_t;


/**
  * @brief  HAL USB PCD BCD State structure definition.
  */
typedef enum
{
  HAL_PCD_BCD_STD_DOWNSTREAM_PORT       = USB_CORE_BCD_PORT_STATUS_STD_DOWNSTREAM,        /*!< PCD BCD STANDARD DOWNSTREAM PORT */
  HAL_PCD_BCD_CHARGING_DOWNSTREAM_PORT  = USB_CORE_BCD_PORT_STATUS_CHARGING_DOWNSTREAM,   /*!< PCD BCD CHARGING DOWNSTREAM PORT */
  HAL_PCD_BCD_DEDICATED_CHARGING_PORT   = USB_CORE_BCD_PORT_STATUS_DEDICATED_CHARGING,    /*!< PCD BCD DEDICATED CHARGING PORT  */
  HAL_PCD_BCD_DISCOVERY_COMPLETED       = 0xFEU,                                          /*!< PCD BCD DISCOVERY COMPLETED      */
  HAL_PCD_BCD_ERROR                     = 0xFFU,                                          /*!< PCD BCD ERROR                    */
} hal_pcd_bcd_port_type_t;


#if defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1)
/**
  * @brief  HAL USB BCD Structure definition.
  */
typedef enum
{
  HAL_PCD_BCD_PORT_STATUS_DEFAULT             = USB_CORE_BCD_PORT_STATUS_DEFAULT,               /*!< USB PCD Default BCD Status Port  */
  HAL_PCD_BCD_PORT_STATUS_NOT_STD_DOWNSTREAM  = USB_CORE_BCD_PORT_STATUS_NOT_STD_DOWNSTREAM,    /*!< USB PCD NOT STD Downstream Port  */
  HAL_PCD_BCD_PORT_STATUS_STD_DOWNSTREAM      = USB_CORE_BCD_PORT_STATUS_STD_DOWNSTREAM,        /*!< USB PCD STD Downstream Port      */
  HAL_PCD_BCD_PORT_STATUS_DEDICATED_CHARGING  = USB_CORE_BCD_PORT_STATUS_DEDICATED_CHARGING,    /*!< USB PCD Dedicated Charging Port  */
  HAL_PCD_BCD_PORT_STATUS_CHARGING_DOWNSTREAM = USB_CORE_BCD_PORT_STATUS_CHARGING_DOWNSTREAM,   /*!< USB PCD Charging Downstream Port */
} hal_pcd_bcd_port_status_t;
#endif /* defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1) */


/**
  * @brief  HAL USB PCD Speed structure definition.
  */
typedef enum
{
  HAL_PCD_SPEED_FS       = USB_CORE_SPEED_FS,         /*!< HAL PCD SPEED FULL SPEED               */
} hal_pcd_speed_t;


/**
  * @brief  HAL USB PCD Device Speed structure definition.
  */
typedef enum
{
  HAL_PCD_DEVICE_SPEED_FS    = USB_CORE_DEVICE_SPEED_FS,            /*!< HAL PCD DEVICE FULL SPEED    */
  HAL_PCD_DEVICE_SPEED_ERROR = USB_CORE_DEVICE_SPEED_ERROR          /*!< HAL PCD DEVICE SPEED ERROR   */
} hal_pcd_device_speed_t;


/**
  * @brief  HAL USB PCD PHY Module structure definition.
  */
typedef enum
{
  HAL_PCD_PHY_EMBEDDED_FS   = USB_CORE_PHY_EMBEDDED_FS,     /*!< PCD PHY EMBEDDED */
} hal_pcd_phy_module_t;


/**
  * @brief  HAL USB PCD DMA status definition.
  */
typedef enum
{
  HAL_PCD_DMA_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD DMA DISABLED */
  HAL_PCD_DMA_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD DMA ENABLED  */
} hal_pcd_dma_status_t;


/**
  * @brief  HAL USB PCD SOF status definition.
  */
typedef enum
{
  HAL_PCD_SOF_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD SOF DISABLED */
  HAL_PCD_SOF_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD SOF ENABLED  */
} hal_pcd_sof_status_t;


/**
  * @brief  HAL USB PCD Low Power Management status definition.
  */
typedef enum
{
  HAL_PCD_LPM_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD Low Power Management DISABLED */
  HAL_PCD_LPM_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD Low Power Management ENABLED  */
} hal_pcd_lpm_status_t;


/**
  * @brief  HAL USB PCD Battery Charging status definition.
  */
typedef enum
{
  HAL_PCD_BCD_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD USB Battery Charging DISABLED */
  HAL_PCD_BCD_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD USB Battery Charging ENABLED  */
} hal_pcd_bcd_status_t;


/**
  * @brief  HAL USB PCD Vbus Sensing status definition.
  */
typedef enum
{
  HAL_PCD_VBUS_SENSE_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD USB Vbus sensing DISABLED */
  HAL_PCD_VBUS_SENSE_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD USB Vbus sensing ENABLED  */
} hal_pcd_vbus_sense_status_t;


/**
  * @brief  HAL USB PCD bulk double buffer status definition.
  */
typedef enum
{
  HAL_PCD_BULK_DB_DISABLED = USB_CORE_CONFIG_DISABLED,   /*!< HAL PCD USB Bulk Double buffer mode DISABLED */
  HAL_PCD_BULK_DB_ENABLED  = USB_CORE_CONFIG_ENABLED,    /*!< HAL PCD USB Bulk Double buffer mode ENABLED  */
} hal_pcd_bulk_db_status_t;


/**
  * @brief  HAL USB PCD Endpoint Type structure definition.
  */
typedef enum
{
  HAL_PCD_EP_TYPE_CTRL = USB_CORE_EP_TYPE_CTRL,
  HAL_PCD_EP_TYPE_ISOC = USB_CORE_EP_TYPE_ISOC,
  HAL_PCD_EP_TYPE_BULK = USB_CORE_EP_TYPE_BULK,
  HAL_PCD_EP_TYPE_INTR = USB_CORE_EP_TYPE_INTR,
} hal_pcd_ep_type_t;


/**
  * @brief  HAL USB PCD Instance configuration Structure definition.
  */
typedef struct
{
  hal_pcd_dma_status_t dma_enable;                         /*!< USB DMA state                                         */

  hal_pcd_speed_t pcd_speed;                               /*!< USB PCD core speed                                    */

  hal_pcd_phy_module_t phy_interface;                      /*!< PHY interface                                         */

  hal_pcd_sof_status_t sof_enable;                         /*!< SOF signal output enable status                       */

  hal_pcd_lpm_status_t lpm_enable;                         /*!< Link power management enable status                   */

  hal_pcd_bcd_status_t battery_charging_enable;            /*!< Battery charging enable status                        */

  hal_pcd_vbus_sense_status_t vbus_sensing_enable;         /*!< VBUS sensing enable status                            */

  hal_pcd_bulk_db_status_t bulk_doublebuffer_enable;       /*!< Bulk endpoint double buffer mode enable status        */
} hal_pcd_config_t;

typedef usb_core_ep_t             hal_pcd_ep_t;     /*!< PCD endpoint structure type */
typedef struct hal_pcd_handle_s   hal_pcd_handle_t; /*!< PCD handle structure type */

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief pointer to a PCD callback function.
  */
typedef void (*hal_pcd_cb_t)(hal_pcd_handle_t *hpcd);

/**
  * @brief pointer to a data callback function.
  */
typedef void (*hal_pcd_data_cb_t)(hal_pcd_handle_t *hpcd, uint8_t ep_num);

/**
  * @brief pointer to an iso callback function.
  */
typedef void (*hal_pcd_iso_cb_t)(hal_pcd_handle_t *hpcd, uint8_t ep_num);

/**
  * @brief pointer to an LPM callback function.
  */
typedef void (*hal_pcd_lpm_cb_t)(hal_pcd_handle_t *hpcd, hal_pcd_lpm_active_status_t lpm_status);

/**
  * @brief pointer to a BCD callback function.
  */
typedef void (*hal_pcd_bcd_cb_t)(hal_pcd_handle_t *hpcd, hal_pcd_bcd_port_type_t port_type);

#endif /* (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U) */


/**
  * @brief  HAL USB PCD Handle Structure definition.
  */
struct hal_pcd_handle_s
{
  hal_pcd_t instance;                                      /*!< Register base address                                 */
  volatile hal_pcd_state_t global_state;                   /*!< PCD communication state                               */
  volatile hal_pcd_port_state_t device_state;              /*!< PCD Port Device state                                 */

#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
  volatile hal_pcd_lpm_state_t lpm_state;                  /*!< LPM State                                             */
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */
  volatile uint8_t usb_address;                            /*!< USB Device Address                                    */

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
  volatile uint32_t last_error_codes;                      /*!< Errors limited to the last process
                                                                This parameter can be a combination of
                                                                @ref PCD_Error_Codes                                  */
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

  uint8_t endpoints_nbr;                                   /*!< Number of device endpoints
                                                                This parameter depends on the used USB core.          */

  uint32_t setup[12];                                      /*!< Setup packet buffer                                   */
  uint8_t *p_setup;                                        /*!< Setup packet buffer pointer                           */

  hal_pcd_ep_t in_ep[USE_HAL_PCD_MAX_ENDPOINT_NB];         /*!< IN endpoint parameters                                */
  hal_pcd_ep_t out_ep[USE_HAL_PCD_MAX_ENDPOINT_NB];        /*!< OUT endpoint parameters                               */

  usb_core_mode_t current_mode;                            /*!< store Current Mode                                    */

  hal_pcd_lpm_status_t lpm_active;                         /*!< Link power management active status
                                                                This parameter can be set to ENABLE or DISABLE        */
  uint32_t lpm_besl;                                       /*!< Best Effort Service Latency                           */

  hal_pcd_bcd_status_t battery_charging_active;            /*!< Battery charging active status
                                                                This parameter can be set to ENABLE or DISABLE        */

  usb_core_pcd_driver_t driver;                            /*!< USB low layer driver                                  */
  void (* p_irq_handler)(struct hal_pcd_handle_s *hpcd);   /*!< USB instance interrupt handler                        */

#if defined (USE_HAL_PCD_USER_DATA) && (USE_HAL_PCD_USER_DATA == 1)
  const void *p_user_data;                                 /*!< User Data Pointer                                     */
#endif /* USE_HAL_PCD_USER_DATA */

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)

  hal_pcd_cb_t p_sof_cb;                                   /*!< USB HAL PCD SOF callback                              */
  hal_pcd_cb_t p_setup_stage_cb;                           /*!< USB HAL PCD Setup Stage callback                      */
  hal_pcd_cb_t p_reset_cb;                                 /*!< USB HAL PCD Reset callback                            */
  hal_pcd_cb_t p_suspend_cb;                               /*!< USB HAL PCD Suspend callback                          */
  hal_pcd_cb_t p_resume_cb;                                /*!< USB HAL PCD Resume callback                           */
  hal_pcd_cb_t p_connect_cb;                               /*!< USB HAL PCD Connect callback                          */
  hal_pcd_cb_t p_disconnect_cb;                            /*!< USB HAL PCD Disconnect callback                       */
  hal_pcd_data_cb_t p_data_out_stage_cb;                   /*!< USB HAL PCD Data OUT Stage callback                   */
  hal_pcd_data_cb_t p_data_in_stage_cb;                    /*!< USB HAL PCD Data IN Stage callback                    */
  hal_pcd_iso_cb_t p_iso_out_incomplete_cb;                /*!< USB HAL PCD ISO OUT Incomplete callback               */
  hal_pcd_iso_cb_t p_iso_in_incomplete_cb;                 /*!< USB HAL PCD ISO IN Incomplete callback                */
  hal_pcd_cb_t p_error_cb;                                 /*!< USB HAL PCD Error callback                            */
  hal_pcd_data_cb_t p_ep_abort_transfer_cb;                /*!< USB HAL PCD Endpoint Abort Transfer callback          */
  hal_pcd_bcd_cb_t p_battery_charging_cb;                  /*!< USB HAL PCD Battery charging callback                 */
  hal_pcd_lpm_cb_t p_low_power_management_cb;              /*!< USB HAL PCD USB Link Power management callback        */
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
};

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup PCD_Exported_Macros Exported Macros
  * @{
  */

/**
  * @brief  HAL USB PCD WAKEUP EXTI line enable.
  */
#define HAL_PCD_USB_WAKEUP_EXTI_ENABLE_IT()                         (EXTI->IMR2 |= USB_WAKEUP_EXTI_LINE)

/**
  * @brief  HAL USB PCD WAKEUP EXTI line disable.
  */
#define HAL_PCD_USB_WAKEUP_EXTI_DISABLE_IT()                        (EXTI->IMR2 &= ~USB_WAKEUP_EXTI_LINE)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PCD_Exported_Functions Exported Functions
  * @{
  */

/** @defgroup PCD_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */
hal_status_t HAL_PCD_Init(hal_pcd_handle_t *hpcd, hal_pcd_t instance);
void         HAL_PCD_DeInit(hal_pcd_handle_t *hpcd);
/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group2 Global Configuration functions
  * @{
  */

hal_status_t HAL_PCD_SetConfig(hal_pcd_handle_t *hpcd, const hal_pcd_config_t *p_config);

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group3 User Data functions
  * @{
  */
#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
uint32_t HAL_PCD_GetLastErrorCodes(const hal_pcd_handle_t *hpcd);
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

#if defined (USE_HAL_PCD_USER_DATA) && (USE_HAL_PCD_USER_DATA == 1)
void        HAL_PCD_SetUserData(hal_pcd_handle_t *hpcd, const void *p_user_data);
const void *HAL_PCD_GetUserData(const hal_pcd_handle_t *hpcd);
#endif /* USE_HAL_PCD_USER_DATA */

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group4 Peripheral Control functions
  * @{
  */
hal_status_t HAL_PCD_Start(hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_Stop(hal_pcd_handle_t *hpcd);


#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
hal_status_t HAL_PCD_LPM_Start(hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_LPM_Stop(hal_pcd_handle_t *hpcd);
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */


#if defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1)
hal_status_t HAL_PCD_BCD_PortTypeDetection(hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_BCD_Start(hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_BCD_Stop(hal_pcd_handle_t *hpcd);
#endif /* defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1) */

hal_status_t HAL_PCD_DeviceConnect(const hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_DeviceDisconnect(const hal_pcd_handle_t *hpcd);
hal_pcd_device_speed_t HAL_PCD_GetDeviceSpeed(const hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_SetDeviceAddress(hal_pcd_handle_t *hpcd, uint8_t address);
hal_status_t HAL_PCD_OpenEndpoint(hal_pcd_handle_t *hpcd, uint8_t ep_addr, uint16_t ep_mps, hal_pcd_ep_type_t ep_type);
hal_status_t HAL_PCD_CloseEndpoint(hal_pcd_handle_t *hpcd, uint8_t ep_addr);
hal_status_t HAL_PCD_FlushEndpoint(const hal_pcd_handle_t *hpcd, uint8_t ep_addr);

hal_status_t HAL_PCD_SetEndpointReceive(hal_pcd_handle_t *hpcd, uint8_t ep_addr, uint8_t *p_buffer, uint32_t size_byte);
hal_status_t HAL_PCD_SetEndpointTransmit(hal_pcd_handle_t *hpcd, uint8_t ep_addr,
                                         uint8_t *p_buffer, uint32_t size_byte);

hal_status_t HAL_PCD_SetEndpointStall(hal_pcd_handle_t *hpcd, uint8_t ep_addr);
hal_status_t HAL_PCD_ClearEndpointStall(hal_pcd_handle_t *hpcd, uint8_t ep_addr);
hal_status_t HAL_PCD_AbortEndpointTransfer(hal_pcd_handle_t *hpcd, uint8_t ep_addr);
uint32_t HAL_PCD_EP_GetRxCount(const hal_pcd_handle_t *hpcd, uint8_t ep_addr);

hal_status_t HAL_PCD_RemoteWakeup_Start(const hal_pcd_handle_t *hpcd);
hal_status_t HAL_PCD_RemoteWakeup_Stop(const hal_pcd_handle_t *hpcd);

hal_status_t HAL_PCD_PMAConfig(hal_pcd_handle_t *hpcd, uint16_t ep_addr, uint16_t ep_kind, uint32_t pma_address);

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group5 Peripheral State functions
  * @{
  */
hal_pcd_state_t HAL_PCD_GetState(const hal_pcd_handle_t *hpcd);
/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group6 IRQ handling functions
  * @{
  */

void HAL_PCD_IRQHandler(hal_pcd_handle_t *hpcd);


void HAL_PCD_DRD_IRQHandler(hal_pcd_handle_t *hpcd);


/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group7 Default Callbacks functions
  * @{
  */

void HAL_PCD_SofCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_SetupStageCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_ResetCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_SuspendCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_ResumeCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_ConnectCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_DisconnectCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_DataOutStageCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num);
void HAL_PCD_DataInStageCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num);
void HAL_PCD_ISOOUTIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num);
void HAL_PCD_ISOINIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num);
void HAL_PCD_LpmCallback(hal_pcd_handle_t *hpcd, hal_pcd_lpm_active_status_t lpm_status);
void HAL_PCD_BcdCallback(hal_pcd_handle_t *hpcd, hal_pcd_bcd_port_type_t port_type);
void HAL_PCD_ErrorCallback(hal_pcd_handle_t *hpcd);
void HAL_PCD_EndpointAbortTransferCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num);

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group8 Register Callbacks functions
  * @{
  */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/* Register callbacks */
hal_status_t HAL_PCD_RegisterSofCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterSetupCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterResetCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterSuspendCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterResumeCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterConnectCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterDisconnectCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterDataOutStageCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback);
hal_status_t HAL_PCD_RegisterDataInStageCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback);
hal_status_t HAL_PCD_RegisterIsoOutIncpltCallback(hal_pcd_handle_t *hpcd, hal_pcd_iso_cb_t p_callback);
hal_status_t HAL_PCD_RegisterIsoInIncpltCallback(hal_pcd_handle_t *hpcd, hal_pcd_iso_cb_t p_callback);
hal_status_t HAL_PCD_RegisterErrorCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterEndpointAbortTransferCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback);
hal_status_t HAL_PCD_RegisterBcdCallback(hal_pcd_handle_t *hpcd, hal_pcd_bcd_cb_t p_callback);
hal_status_t HAL_PCD_RegisterLpmCallback(hal_pcd_handle_t *hpcd, hal_pcd_lpm_cb_t p_callback);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/

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

#endif /* STM32C5XX_HAL_PCD_H */
