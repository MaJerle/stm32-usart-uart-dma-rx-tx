/**
  ******************************************************************************
  * @file    stm32c5xx_usb_core_def.h
  * @brief   Header file for the USB CORE driver module.
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
#ifndef STM32C5XX_USB_CORE_DEF_H
#define STM32C5XX_USB_CORE_DEF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Private functions ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#ifndef USB_CORE_CURRENT_MODE_MAX_DELAY_CYCLES
#define USB_CORE_CURRENT_MODE_MAX_DELAY_CYCLES        (200U * (SystemCoreClock / 1000U))
#endif /* USB_CORE_CURRENT_MODE_MAX_DELAY_CYCLES */

#define USB_CORE_IN_EP_DIR_MSK                (0x80U)    /*!< IN Endpoint Direction Mask */

/**
  * @brief  USB CORE status definitions
  */
typedef enum
{
  USB_CORE_OK    = 0x00000000U,   /*!< USB CORE operation completed successfully */
  USB_CORE_ERROR = 0xFFFFFFFFU,   /*!< USB CORE operation completed with error   */
} usb_core_status_t;


/**
  * @brief  USB CORE configuration state definitions
  */
typedef enum
{
  USB_CORE_CONFIG_DISABLED = 0x0U,               /*!< USB CORE configuration state disabled: 0 */
  USB_CORE_CONFIG_ENABLED  = 0x1U,               /*!< USB CORE configuration state enabled: 1  */
} usb_core_config_status_t;


/**
  * @brief  USB CORE mode definitions
  */
typedef enum
{
  USB_CORE_DEVICE_MODE = 0x00U,                  /*!< USB CORE device mode */
  USB_CORE_HOST_MODE   = 0x01U,                  /*!< USB CORE host mode   */
} usb_core_mode_t;

/**
  * @brief  USB CORE speed definitions
  */
typedef enum
{
  USB_CORE_SPEED_FS = 0x01U,                     /*!< USB CORE full speed         */
} usb_core_speed_t;


/**
  * @brief  USB CORE device speed definitions
  */
typedef enum
{
  USB_CORE_DEVICE_SPEED_LS = 0x00U,              /*!< USB CORE device low speed   */
  USB_CORE_DEVICE_SPEED_FS = 0x01U,              /*!< USB CORE device full speed  */
  USB_CORE_DEVICE_SPEED_ERROR = 0xFFU,           /*!< USB CORE device speed error */
} usb_core_device_speed_t;


/**
  * @brief  USB CORE host port speed definitions
  */
typedef enum
{
  USB_CORE_PORT_SPEED_FS = 0x01U,                /*!< USB CORE host port full speed */
  USB_CORE_PORT_SPEED_LS = 0x02U,                /*!< USB CORE host port low speed  */
} usb_core_port_speed_t;


/**
  * @brief  USB CORE PHY module definitions
  */
typedef enum
{
  USB_CORE_PHY_EMBEDDED_FS   = 0x2U,             /*!< Embedded FS PHY   */
} usb_core_phy_module_t;


/**
  * @brief  USB endpoint type definitions
  */
typedef enum
{
  USB_CORE_EP_TYPE_CTRL = 0x00U,
  USB_CORE_EP_TYPE_ISOC = 0x01U,
  USB_CORE_EP_TYPE_BULK = 0x02U,
  USB_CORE_EP_TYPE_INTR = 0x03U,
} usb_core_ep_type_t;


/**
  * @brief  USB CORE endpoint direction definitions
  */
typedef enum
{
  USB_CORE_EP_OUT_DIR = 0x00U,                   /*!< USB CORE endpoint OUT direction: 0 */
  USB_CORE_EP_IN_DIR  = 0x01U,                   /*!< USB CORE endpoint IN direction:  1 */
} usb_core_ep_direction_t;


/**
  * @brief  USB CORE BCD detection definitions
  */
typedef enum
{
  USB_CORE_BCD_PRIMARY_DETECTION   = 0x01U,
  USB_CORE_BCD_SECONDARY_DETECTION = 0x02U,
} usb_core_bcd_detection_t;


/**
  * @brief  USB CORE BCD port status definitions
  */
typedef enum
{
  USB_CORE_BCD_PORT_STATUS_DEFAULT             = 0x00U,     /*!< USB CORE BCD default port status          */
  USB_CORE_BCD_PORT_STATUS_NOT_STD_DOWNSTREAM  = 0x01U,     /*!< USB CORE BCD non-standard downstream port */
  USB_CORE_BCD_PORT_STATUS_STD_DOWNSTREAM      = 0x02U,     /*!< USB CORE BCD standard downstream port     */
  USB_CORE_BCD_PORT_STATUS_DEDICATED_CHARGING  = 0x03U,     /*!< USB CORE BCD dedicated charging port      */
  USB_CORE_BCD_PORT_STATUS_CHARGING_DOWNSTREAM = 0x04U,     /*!< USB CORE BCD charging downstream port     */
} usb_core_bcd_port_status_t;


/**
  * @brief  USB CORE BCD mode definitions
  */
typedef enum
{
  USB_CORE_BCD_CONFIG_DCD = 0x00U,    /*!< USB CORE BCD Data Contact Detection */
  USB_CORE_BCD_CONFIG_PD = 0x01U,     /*!< USB CORE BCD Primary Detection      */
  USB_CORE_BCD_CONFIG_SD = 0x02U,     /*!< USB CORE BCD Secondary Detection    */
} usb_core_bcd_config_t;


/**
  * @brief  USB CORE BCD status definitions
  */
typedef enum
{
  USB_CORE_BCD_CONFIG_STS_CLEAR = 0x00U,   /*!< USB CORE BCD mode cleared */
  USB_CORE_BCD_CONFIG_STS_SET = 0x01U,     /*!< USB CORE BCD mode set     */
} usb_core_bcd_config_sts_t;


/**
  * @brief  USB CORE endpoint identifier definitions
  */
typedef enum
{
  USB_CORE_ENDPOINT_0  = 0U,     /*!< USB CORE ENDPOINT 0  */
  USB_CORE_ENDPOINT_1  = 1U,     /*!< USB CORE ENDPOINT 1  */
  USB_CORE_ENDPOINT_2  = 2U,     /*!< USB CORE ENDPOINT 2  */
  USB_CORE_ENDPOINT_3  = 3U,     /*!< USB CORE ENDPOINT 3  */
  USB_CORE_ENDPOINT_4  = 4U,     /*!< USB CORE ENDPOINT 4  */
  USB_CORE_ENDPOINT_5  = 5U,     /*!< USB CORE ENDPOINT 5  */
  USB_CORE_ENDPOINT_6  = 6U,     /*!< USB CORE ENDPOINT 6  */
  USB_CORE_ENDPOINT_7  = 7U,     /*!< USB CORE ENDPOINT 7  */
  USB_CORE_ENDPOINT_8  = 8U,     /*!< USB CORE ENDPOINT 8  */
  USB_CORE_ENDPOINT_9  = 9U,     /*!< USB CORE ENDPOINT 9  */
  USB_CORE_ENDPOINT_10 = 10U,    /*!< USB CORE ENDPOINT 10 */
  USB_CORE_ENDPOINT_11 = 11U,    /*!< USB CORE ENDPOINT 11 */
  USB_CORE_ENDPOINT_12 = 12U,    /*!< USB CORE ENDPOINT 12 */
  USB_CORE_ENDPOINT_13 = 13U,    /*!< USB CORE ENDPOINT 13 */
  USB_CORE_ENDPOINT_14 = 14U,    /*!< USB CORE ENDPOINT 14 */
  USB_CORE_ENDPOINT_15 = 15U,    /*!< USB CORE ENDPOINT 15 */
  USB_CORE_ENDPOINT_FF = 0xFFU,  /*!< USB CORE endpoint invalid value (0xFF) */
} usb_core_endpoint_t;


/**
  * @brief  USB CORE CHEP identifier definitions
  */
typedef enum
{
  USB_CORE_PHY_CHEP_0  = 0U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 0  */
  USB_CORE_PHY_CHEP_1  = 1U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 1  */
  USB_CORE_PHY_CHEP_2  = 2U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 2  */
  USB_CORE_PHY_CHEP_3  = 3U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 3  */
  USB_CORE_PHY_CHEP_4  = 4U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 4  */
  USB_CORE_PHY_CHEP_5  = 5U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 5  */
  USB_CORE_PHY_CHEP_6  = 6U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 6  */
  USB_CORE_PHY_CHEP_7  = 7U,      /*!< USB CORE PHYSICAL CHANNEL/ENDPOINT 7  */
  USB_CORE_PHY_CHEP_FF = 0xFFU,   /*!< USB CORE invalid value (0xFF)         */
} usb_core_phy_chep_t;

typedef usb_core_phy_chep_t usb_core_phy_ch_t;
typedef usb_core_phy_chep_t usb_core_phy_ep_t;


/**
  * @brief  USB CORE host port reset status definitions
  */
typedef enum
{
  USB_CORE_PORT_RESET_STS_CLEAR = 0U,    /*!< USB CORE clear port reset */
  USB_CORE_PORT_RESET_STS_SET = 1U,      /*!< USB CORE set port reset   */
} usb_core_port_reset_sts_t;


/**
  * @brief  USB CORE port resume status definitions
  */
typedef enum
{
  USB_CORE_PORT_RESUME_STS_CLEAR = 0x00U,   /*!< USB CORE Port Resume status cleared */
  USB_CORE_PORT_RESUME_STS_SET = 0x01U,     /*!< USB CORE Port Resume status set     */
} usb_core_port_resume_sts_t;

/**
  * @brief  USB CORE channel PID type definitions
  */
typedef enum
{
  USB_CORE_CH_PID_DATA0 = 0x00U,     /*!< USB CORE HC PID DATA0 */
  USB_CORE_CH_PID_DATA2 = 0x01U,     /*!< USB CORE HC PID DATA2 */
  USB_CORE_CH_PID_DATA1 = 0x02U,     /*!< USB CORE HC PID DATA1 */
  USB_CORE_CH_PID_SETUP = 0x03U,     /*!< USB CORE HC PID SETUP */
} usb_core_ch_pid_type_t;


/**
  * @brief  USB CORE host channel direction definitions
  */
typedef enum
{
  USB_CORE_CH_OUT_DIR = USB_CORE_EP_OUT_DIR,      /*!< USB CORE channel OUT direction: 0 */
  USB_CORE_CH_IN_DIR  = USB_CORE_EP_IN_DIR,       /*!< USB CORE channel IN direction:  1 */
} usb_core_ch_direction_t;


/**
  * @brief  USB CORE channel identifier definitions
  */
typedef enum
{
  USB_CORE_CHANNEL_0  = 0U,    /*!< USB CORE CHANNEL 0  */
  USB_CORE_CHANNEL_1  = 1U,    /*!< USB CORE CHANNEL 1  */
  USB_CORE_CHANNEL_2  = 2U,    /*!< USB CORE CHANNEL 2  */
  USB_CORE_CHANNEL_3  = 3U,    /*!< USB CORE CHANNEL 3  */
  USB_CORE_CHANNEL_4  = 4U,    /*!< USB CORE CHANNEL 4  */
  USB_CORE_CHANNEL_5  = 5U,    /*!< USB CORE CHANNEL 5  */
  USB_CORE_CHANNEL_6  = 6U,    /*!< USB CORE CHANNEL 6  */
  USB_CORE_CHANNEL_7  = 7U,    /*!< USB CORE CHANNEL 7  */
  USB_CORE_CHANNEL_8  = 8U,    /*!< USB CORE CHANNEL 8  */
  USB_CORE_CHANNEL_9  = 9U,    /*!< USB CORE CHANNEL 9  */
  USB_CORE_CHANNEL_10 = 10U,   /*!< USB CORE CHANNEL 10 */
  USB_CORE_CHANNEL_11 = 11U,   /*!< USB CORE CHANNEL 11 */
  USB_CORE_CHANNEL_12 = 12U,   /*!< USB CORE CHANNEL 12 */
  USB_CORE_CHANNEL_13 = 13U,   /*!< USB CORE CHANNEL 13 */
  USB_CORE_CHANNEL_14 = 14U,   /*!< USB CORE CHANNEL 14 */
  USB_CORE_CHANNEL_15 = 15U,   /*!< USB CORE CHANNEL 15 */
  USB_CORE_CHANNEL_FF = 0xFFU, /*!< USB CORE channel invalid value (0xFF) */
} usb_core_channel_t;


/**
  * @brief  USB CORE configuration parameters definition
  */
typedef struct
{
  uint8_t endpoints_nbr;
  uint8_t channels_nbr;
  usb_core_phy_module_t phy_interface;
  usb_core_speed_t core_speed;
  usb_core_config_status_t dma_state;
  usb_core_config_status_t sof_state;
  usb_core_config_status_t bcd_state;
  usb_core_config_status_t vbus_sense_state;
  usb_core_config_status_t iso_db_state;
  usb_core_config_status_t bulk_db_state;
} usb_core_config_params_t;


/**
  * @brief  PCD endpoint structure definition
  */
typedef struct
{
  usb_core_endpoint_t num;           /*!< Endpoint number
                                          This parameter must be a value between Min_Data = 0 and Max_Data = 15       */

  usb_core_ep_direction_t dir;       /*!< Endpoint direction
                                          This parameter stores the physical channel direction (IN/OUT)               */

  usb_core_ep_type_t type;           /*!< Endpoint type
                                          This parameter can be any value of @ref usb_core_ep_type_t                  */

  uint32_t max_packet;               /*!< Endpoint maximum packet size                                                */

  uint32_t xfer_length;              /*!< Current transfer length                                                     */

  uint32_t xfer_count;               /*!< Partial transfer length in case of multi-packet transfer                    */

  uint8_t *p_xfer_buffer;            /*!< Pointer to transfer buffer                                                  */

  uint32_t xfer_size;                /*!< Requested transfer size                                                     */

  uint16_t pma_address;              /*!< PMA address
                                          This parameter can be any value between Min_addr = 0 and Max_addr = 1K      */

  uint16_t pma_addr0;                /*!< PMA address 0
                                          This parameter can be any value between Min_addr = 0 and Max_addr = 1K      */

  uint16_t pma_addr1;                /*!< PMA address 1
                                          This parameter can be any value between Min_addr = 0 and Max_addr = 1K      */

  uint8_t double_buffer_en;          /*!< Double buffer enable
                                          This parameter can be 0 or 1                                                */

  uint8_t xfer_fill_db;              /*!< Double-buffer: need to fill a new buffer (used with bulk IN)                */
} usb_core_ep_t;


/**
  * @brief  USB instance host channel structure definition
  */
typedef struct
{
  usb_core_phy_ch_t phy_ch_num;       /*!< Host physical channel number
                                           This parameter must be a value between Min_Data = 0 and Max_Data = 15      */

  usb_core_channel_t ch_num;          /*!< Host channel number
                                           This parameter must be a value between Min_Data = 0 and Max_Data = 15      */

  usb_core_ch_direction_t ch_dir;     /*!< Channel direction
                                           This parameter stores the physical channel direction (IN/OUT)              */

  usb_core_device_speed_t speed;      /*!< USB host channel device speed
                                           This parameter can be any value of @ref usb_core_device_speed_t
                                                                                   (USB_CORE_DEVICE_SPEED_xxx)        */

  usb_core_ch_pid_type_t data_pid;    /*!< Initial data PID
                                           This parameter must be a value between Min_Data = 0 and Max_Data = 1       */

  usb_core_ep_type_t ep_type;         /*!< Endpoint type
                                           This parameter can be any value of @ref usb_core_ep_type_t                 */

  usb_core_endpoint_t ep_num;         /*!< Endpoint number
                                           This parameter must be a value between Min_Data = 0 and Max_Data = 15      */

  uint8_t dev_addr;                   /*!< USB device address
                                           This parameter must be a value between Min_Data = 1 and Max_Data = 255     */

  uint16_t max_packet;                /*!< Endpoint maximum packet size                                               */
  uint8_t hub_port_nbr;               /*!< USB hub port number                                                        */
  uint8_t hub_addr;                   /*!< USB hub address                                                            */

  uint8_t *p_xfer_buffer;             /*!< Pointer to transfer buffer                                                 */
  uint32_t xfer_count;                /*!< Partial transfer length in case of multi-packet transfer                   */
  uint32_t xfer_length;               /*!< Current transfer length                                                    */
  uint32_t xfer_size;                 /*!< Host channel transfer size                                                 */

  uint32_t err_cnt;                   /*!< Host channel error count                                                   */

  uint16_t pma_address;               /*!< PMA address
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K     */

  uint16_t pma_addr0;                 /*!< PMA address 0
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K     */

  uint16_t pma_addr1;                 /*!< PMA address 1
                                           This parameter can be any value between Min_addr = 0 and Max_addr = 1K     */

  uint8_t double_buffer_en;           /*!< Double buffer enable
                                           This parameter can be 0 or 1                                               */
} usb_core_ch_t;


/**
  * @brief  USB CORE PCD driver structure definition
  */
typedef struct
{
  usb_core_status_t (* core_init)(uint32_t instance,
                                  const usb_core_config_params_t *p_core_config);    /*!< Initialize USB core         */

  usb_core_status_t (* core_deinit)(uint32_t instance);                              /*!< De-initialize USB core      */

  usb_core_status_t (* core_set_mode)(uint32_t instance, usb_core_mode_t core_mode); /*!< Set USB core mode           */
  usb_core_mode_t (* core_get_mode)(uint32_t instance);                              /*!< Get USB core mode           */
  usb_core_status_t (* core_enable_interrupts)(uint32_t instance);                   /*!< Enable USB core interrupts  */
  usb_core_status_t (* core_disable_interrupts)(uint32_t instance);                  /*!< Disable USB core interrupts */

  usb_core_status_t (* device_init)(uint32_t instance,
                                    const usb_core_config_params_t *p_core_config);  /*!< Initialize device mode      */

  usb_core_status_t (* device_start)(uint32_t instance);                             /*!< Start the device            */
  usb_core_status_t (* device_stop)(uint32_t instance);                              /*!< Stop the device             */
  usb_core_status_t (* device_connect)(uint32_t instance);                           /*!< Connect the device          */
  usb_core_status_t (* device_disconnect)(uint32_t instance);                        /*!< Disconnect the device       */
  usb_core_status_t (* device_set_address)(uint32_t instance, uint8_t address);      /*!< Set device address          */

  usb_core_device_speed_t (* device_get_speed)(uint32_t instance);                   /*!< Get device speed            */

  usb_core_status_t (* ep_activate)(uint32_t instance, usb_core_ep_t *p_ep);         /*!< Activate the endpoint       */

  usb_core_status_t (* ep_deactivate)(uint32_t instance,
                                      const usb_core_ep_t *p_ep);                    /*!< Deactivate the endpoint     */

  usb_core_status_t (* ep_start_transfer)(uint32_t instance, usb_core_ep_t *p_ep);   /*!< Start the endpoint transfer */

  usb_core_status_t (* ep_stop_transfer)(uint32_t instance,
                                         const usb_core_ep_t *p_ep);                 /*!< Stop the endpoint transfer  */

  usb_core_status_t (* ep_set_stall)(uint32_t instance,
                                     const usb_core_ep_t *p_ep);                     /*!< Endpoint set stall          */

  usb_core_status_t (* ep_clear_stall)(uint32_t instance,
                                       const usb_core_ep_t *p_ep);                   /*!< Endpoint clear stall        */

  usb_core_status_t (* ep0_out_start)(uint32_t instance,
                                      const uint8_t *p_setup);                       /*!< Start endpoint 0 OUT        */

  usb_core_status_t (* set_tx_fifo)(uint32_t instance, uint8_t fifo,
                                    uint16_t size_words);                            /*!< Set Tx FIFO                 */

  usb_core_status_t (* set_rx_fifo)(uint32_t instance, uint16_t size_words);         /*!< Set Rx FIFO                 */

  usb_core_status_t (* flush_tx_fifo)(uint32_t instance, uint32_t tx_fifo);          /*!< Flush Tx FIFO               */
  usb_core_status_t (* flush_rx_fifo)(uint32_t instance);                            /*!< Flush Rx FIFO               */

  void *(* read_packet)(uint32_t instance, uint8_t *p_dest,
                        uint8_t ep_num, uint32_t size_byte);                         /*!< Read packet                 */

  usb_core_status_t (* write_packet)(uint32_t instance, const uint8_t *p_src,
                                     uint8_t ep_num, uint32_t size_byte);            /*!< Write packet                */

  usb_core_status_t (* remote_wakeup_activate)(uint32_t instance);                   /*!< Activate remote wake-up     */
  usb_core_status_t (* remote_wakeup_deactivate)(uint32_t instance);                 /*!< Deactivate remote wake-up   */

  usb_core_status_t (* lpm_activate)(uint32_t instance);                             /*!< Activate LPM                */
  usb_core_status_t (* lpm_deactivate)(uint32_t instance);                           /*!< Deactivate LPM              */

  usb_core_status_t (* bcd_activate)(uint32_t instance);                             /*!< Activate BCD                */
  usb_core_status_t (* bcd_deactivate)(uint32_t instance);                           /*!< Deactivate BCD              */
  usb_core_status_t (* bcd_set_mode)(uint32_t instance,
                                     usb_core_bcd_config_t bcd_config,
                                     usb_core_bcd_config_sts_t bcd_sts);             /*!< Set BCD mode                */

  usb_core_bcd_port_status_t (* bcd_detect_port_type)(uint32_t instance,
                                                      usb_core_bcd_detection_t detection); /*!< Detect BCD port type  */
} usb_core_pcd_driver_t;

/**
  * @brief  USB CORE HCD driver structure definition
  */
typedef struct
{
  usb_core_status_t (* core_init)(uint32_t instance,
                                  const usb_core_config_params_t *p_core_config);    /*!< Initialize USB core         */

  usb_core_status_t (* core_deinit)(uint32_t instance);                              /*!< De-initialize USB core      */

  usb_core_status_t (* core_set_mode)(uint32_t instance, usb_core_mode_t core_mode); /*!< Set USB core mode           */
  usb_core_mode_t (* core_get_mode)(uint32_t instance);                              /*!< Get USB core mode           */
  usb_core_status_t (* core_enable_interrupts)(uint32_t instance);                   /*!< Enable USB core interrupts  */
  usb_core_status_t (* core_disable_interrupts)(uint32_t instance);                  /*!< Disable USB core interrupts */

  usb_core_status_t (* host_init)(uint32_t instance,
                                  const usb_core_config_params_t *p_core_config);    /*!< Initialize USB host         */

  usb_core_status_t (* host_start)(uint32_t instance);                               /*!< USB host start              */
  usb_core_status_t (* host_stop)(uint32_t instance);                                /*!< USB host stop               */
  usb_core_status_t (* host_channel_init)(uint32_t instance, usb_core_ch_t *p_ch);   /*!< USB host channel init       */
  usb_core_status_t (* host_channel_start)(uint32_t instance, usb_core_ch_t *p_ch);  /*!< USB host channel start      */
  usb_core_status_t (* host_channel_stop)(uint32_t instance, usb_core_ch_t *p_ch);   /*!< USB host channel stop       */
  usb_core_status_t (* host_channel_halt)(uint32_t instance,
                                          const usb_core_ch_t *p_ch);                /*!< USB host channel halt       */

  usb_core_status_t (* host_channel_close)(uint32_t instance, usb_core_ch_t *p_ch);  /*!< USB host channel close      */
  usb_core_status_t (* host_port_power)(uint32_t instance, uint8_t state);           /*!< USB host port power         */
  usb_core_status_t (* host_port_reset)(uint32_t instance,
                                        usb_core_port_reset_sts_t reset_status);     /*!< USB host port reset         */
  usb_core_status_t (* host_port_suspend)(uint32_t instance);                        /*!< USB host port suspend       */
  usb_core_status_t (* host_port_resume)(uint32_t instance,
                                         usb_core_port_resume_sts_t resume_status);  /*!< USB host port resume        */

  uint32_t (* host_get_current_frame)(uint32_t instance);                            /*!< USB host get current frame  */
  usb_core_port_speed_t (* host_get_port_speed)(uint32_t instance);                  /*!< USB host get port speed     */
  uint32_t (* core_get_dma_status)(uint32_t instance);                               /*!< USB core get DMA status     */
} usb_core_hcd_driver_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* STM32C5XX_USB_CORE_DEF_H */
