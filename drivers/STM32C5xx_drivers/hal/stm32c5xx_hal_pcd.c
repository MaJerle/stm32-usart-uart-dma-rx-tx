/**
  ******************************************************************************
  * @file    stm32c5xx_hal_pcd.c
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following
  *          features of the USB Peripheral Controller:
  *           + Initialization and deinitialization functions
  *           + I/O operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if defined (USB_DRD_FS)
#if defined (USE_HAL_PCD_MODULE) && (USE_HAL_PCD_MODULE == 1)

/** @addtogroup PCD
  * @brief Peripheral Controller Driver, including the functions that allow the USB to be used in device mode.
  * @{
  */
/** @defgroup PCD_Introduction Introduction
  * @{

  USB Peripheral Controller Driver (PCD) hardware abstraction layer provides all required APIs to interface with the
  USB instance in device mode.
  It simplifies the initialization, configuration, and management of the USB depending on the user's required device
  function.

  This abstraction layer ensures portability and ease of use across different STM32 series. The PCD HAL abstracts
  different USB hardware instances that can be (depending on the STM32 MCU):
  * USB OTG FS (On-The-GO Full Speed)
  * USB OTG HS (On-The-GO High Speed)
  * USB DRD FS (Dual Role Device Full Speed)

  */
/**
  * @}
  */

/** @defgroup PCD_How_To_Use HAL How To Use
  * @{

  # How to use the HAL USB PCD module driver

  ## Use the USB Peripheral Controller Driver (PCD) HAL driver as follows:

  1. Declare a hal_pcd_handle_t handle structure, for example:
      hal_pcd_handle_t gh_pcd_usb_drd_fs;

  2. Initialize the USB PCD low-level resources:
    - USB PCD interface clock configuration:
      - Enable USB peripheral clock.

    - USB PCD interface power configuration:
      - Enable USB peripheral VddUSB power supply if applicable.

    - USB device pins configuration:
      USB data pins are automatically configured by the hardware during USB peripheral initialization;
      no additional user action is required.

    - NVIC configuration for interrupt handling with HAL_PCD_IRQHandler():
      - Set the USB PCD interrupt priority.
      - Enable the USB IRQ channel in the NVIC.

  3. Initialize the USB PCD driver with HAL_PCD_Init() by selecting an instance, for example:

     - HAL_PCD_Init(&gh_pcd_usb_drd_fs, HAL_PCD_DRD_FS);

  - Declare a hal_pcd_config_t structure, for example:
     - hal_pcd_config_t config_pcd_usb_drd_fs;

  - In the configuration structure,
    program the PHY interface, core speed, and other parameters as required.

  - Apply the configuration with:
     - HAL_PCD_SetConfig(&gh_pcd_usb_drd_fs, &config_pcd_usb_drd_fs);

  - Configure required USB device endpoints.

  ## USB PCD callback definitions:
  By default, all callbacks are initialized to their corresponding default weak functions.
  When the compilation define USE_HAL_PCD_REGISTER_CALLBACKS is set to 1U, configure the
  driver callbacks dynamically using the callback registration functions:

  | Default callback weak function                   | Callback registration function
  |--------------------------------------------------|------------------------------------------------------------------
  | HAL_PCD_SofCallback()                            | HAL_PCD_RegisterSofCallback()
  | HAL_PCD_SetupStageCallback()                     | HAL_PCD_RegisterSetupCallback()
  | HAL_PCD_ResetCallback()                          | HAL_PCD_RegisterResetCallback()
  | HAL_PCD_SuspendCallback()                        | HAL_PCD_RegisterSuspendCallback()
  | HAL_PCD_ResumeCallback()                         | HAL_PCD_RegisterResumeCallback()
  | HAL_PCD_ConnectCallback()                        | HAL_PCD_RegisterConnectCallback()
  | HAL_PCD_DisconnectCallback()                     | HAL_PCD_RegisterDisconnectCallback()
  | HAL_PCD_DataOutStageCallback()                   | HAL_PCD_RegisterDataOutStageCallback()
  | HAL_PCD_DataInStageCallback()                    | HAL_PCD_RegisterDataInStageCallback()
  | HAL_PCD_ISOOUTIncompleteCallback()               | HAL_PCD_RegisterIsoOutIncpltCallback()
  | HAL_PCD_ISOINIncompleteCallback()                | HAL_PCD_RegisterIsoInIncpltCallback()
  | HAL_PCD_ErrorCallback()                          | HAL_PCD_RegisterErrorCallback()
  | HAL_PCD_EndpointAbortTransferCallback()          | HAL_PCD_RegisterEndpointAbortTransferCallback()
  | HAL_PCD_BcdCallback()                            | HAL_PCD_RegisterBcdCallback()
  | HAL_PCD_LpmCallback()                            | HAL_PCD_RegisterLpmCallback()
  */

/**
  * @}
  */

/** @defgroup PCD_Configuration_Table Configuration Table
  * @{
  ## Configuration inside the USB PCD driver:

  | Config defines                 | Description     | Default value | Note
  |--------------------------------|-----------------|---------------|--------------------------------------------------
  | USE_ASSERT_DBG_PARAM           | from IDE        |       NA      | Enable the parameter assert.
  | USE_ASSERT_DBG_STATE           | from IDE        |       NA      | Enable the state assert.
  | USE_HAL_PCD_MODULE             | from hal_conf.h |        1      | Enable the HAL USB PCD module.
  | USE_HAL_PCD_REGISTER_CALLBACKS | from hal_conf.h |        0      | Enable the register callbacks.
  | USE_HAL_PCD_USB_EP_TYPE_ISOC   | from hal_conf.h |        1      | Enable support for isochronous endpoints.
  | USE_HAL_PCD_USB_BCD            | from hal_conf.h |        0      | Enable USB Battery Charging Detection support.
  | USE_HAL_PCD_USB_LPM            | from hal_conf.h |        0      | Enable USB Link Power Management support.
  | USE_HAL_PCD_USB_DOUBLE_BUFFER  | from hal_conf.h |        1      | Enable double-buffering for USB transfers.
  | USE_HAL_PCD_MAX_ENDPOINT_NB    | from hal_conf.h |        8      | Maximum number of USB PCD endpoints.
  | USE_HAL_PCD_USER_DATA          | from hal_conf.h |        0      | Add user data to the HAL USB PCD handle.
  | USE_HAL_PCD_GET_LAST_ERRORS    | from hal_conf.h |        0      | Add an error value to the HAL USB PCD handle.
  | USE_HAL_CHECK_PARAM            | from hal_conf.h |        0      | Enable checking of PCD API parameters at runtime.
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros Private Macros
  * @{
  */

/*! Macro to check ep_type */
#define HAL_PCD_CHECK_EP_TYPE(ep_type) ((((ep_type) == HAL_PCD_EP_TYPE_CTRL) \
                                         || ((ep_type) == HAL_PCD_EP_TYPE_BULK) \
                                         || ((ep_type) == HAL_PCD_EP_TYPE_INTR) \
                                         || ((ep_type) == HAL_PCD_EP_TYPE_ISOC)) ? 1U : 0U)

/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup PCD_Private_Functions Private Functions
  * @{
  */

static hal_status_t PCD_DRD_EP_ISR_Handler(hal_pcd_handle_t *hpcd);
#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
static hal_status_t HAL_PCD_EP_DB_Transmit(hal_pcd_handle_t *hpcd, hal_pcd_ep_t *p_ep, uint16_t ep_value);
static uint16_t HAL_PCD_EP_DB_Receive(hal_pcd_handle_t *hpcd, hal_pcd_ep_t *p_ep, uint16_t ep_value);
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCD_Exported_Functions Exported Functions
  * @{
  */

/** @addtogroup PCD_Exported_Functions_Group1 Initialization and deinitialization functions
  *
  * @{
  This subsection provides a set of functions allowing to initialize and deinitialize the PCD.
  - Call the function HAL_PCD_Init() to initialize the selected PCD handle and associate an instance.
  - Call the function HAL_PCD_DeInit() to de-initialize the given HAL PCD instance by resetting the state machine.
  */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the hal_pcd_handle_t and initialize the associated instance.
  * @param  hpcd PCD handler
  * @param  instance PCD instance
  * @retval HAL status
  */
hal_status_t HAL_PCD_Init(hal_pcd_handle_t *hpcd, hal_pcd_t instance)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  ASSERT_DBG_PARAM(IS_PCD_ALL_INSTANCE((usb_drd_global_t *)((uint32_t)instance)));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }

  if (!IS_PCD_ALL_INSTANCE((usb_drd_global_t *)((uint32_t)instance)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->instance = instance;

  switch (instance)
  {
    case HAL_PCD_DRD_FS:

      /* Register USB core instance operational functions */
      (void)USB_DRD_PCD_InitDriver(&hpcd->driver);

      hpcd->p_irq_handler = HAL_PCD_DRD_IRQHandler;

      /* Get the device endpoints number */
      hpcd->endpoints_nbr = (uint8_t)(USB_DRD_FS_EP_NBR);

      break;

    default:
      break;
  }

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
  hpcd->last_error_codes = HAL_PCD_ERROR_NONE;
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  hpcd->p_sof_cb = HAL_PCD_SofCallback;
  hpcd->p_setup_stage_cb = HAL_PCD_SetupStageCallback;
  hpcd->p_reset_cb = HAL_PCD_ResetCallback;
  hpcd->p_suspend_cb = HAL_PCD_SuspendCallback;
  hpcd->p_resume_cb = HAL_PCD_ResumeCallback;
  hpcd->p_connect_cb = HAL_PCD_ConnectCallback;
  hpcd->p_disconnect_cb = HAL_PCD_DisconnectCallback;
  hpcd->p_data_out_stage_cb = HAL_PCD_DataOutStageCallback;
  hpcd->p_data_in_stage_cb = HAL_PCD_DataInStageCallback;
  hpcd->p_iso_out_incomplete_cb = HAL_PCD_ISOOUTIncompleteCallback;
  hpcd->p_iso_in_incomplete_cb = HAL_PCD_ISOINIncompleteCallback;
  hpcd->p_error_cb = HAL_PCD_ErrorCallback;
  hpcd->p_ep_abort_transfer_cb = HAL_PCD_EndpointAbortTransferCallback;
  hpcd->p_low_power_management_cb = HAL_PCD_LpmCallback;
  hpcd->p_battery_charging_cb = HAL_PCD_BcdCallback;
#endif /* (USE_HAL_PCD_REGISTER_CALLBACKS) */

  /* Reset device address */
  hpcd->usb_address = 0U;

#if defined (USE_HAL_PCD_USER_DATA) && (USE_HAL_PCD_USER_DATA == 1U)
  hpcd->p_user_data = (void *) NULL;
#endif /* USE_HAL_PCD_USER_DATA */

  hpcd->device_state = HAL_PCD_PORT_STATE_DEV_DISCONNECT;
  hpcd->global_state = HAL_PCD_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handler
  */
void HAL_PCD_DeInit(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  ASSERT_DBG_PARAM(IS_PCD_ALL_INSTANCE((usb_drd_global_t *)((uint32_t)hpcd->instance)));

  /* Stop device operation before de-initializing */
  (void)hpcd->driver.device_stop((uint32_t)hpcd->instance);

#if defined (USE_HAL_PCD_USER_DATA) && (USE_HAL_PCD_USER_DATA == 1U)
  hpcd->p_user_data = (void *) NULL;
#endif /* USE_HAL_PCD_USER_DATA */

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
  hpcd->last_error_codes = HAL_PCD_ERROR_NONE;
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */
  hpcd->device_state = HAL_PCD_PORT_STATE_DEV_DISCONNECT;
  hpcd->global_state = HAL_PCD_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group2 Global Configuration functions
  * @{
  This subsection provides functions allowing to configure the USB in Device mode:
  - Call HAL_PCD_SetConfig() to configure the initialized instance with a set of parameters containing:
     * phy_interface
     * endpoints_nbr
     * core_speed
     * bulk_db_state
     * iso_db_state
  */

/**
  * @brief  Configure the PCD according to the specified
  *         parameters in the hal_pcd_handle_t and initialize the associated handle.
  * @param  hpcd PCD handler
  * @param  p_config pointer to the peripheral configuration structure
  * @retval HAL status
  */
hal_status_t HAL_PCD_SetConfig(hal_pcd_handle_t *hpcd, const hal_pcd_config_t *p_config)
{
  hal_status_t ret = HAL_OK;
  uint8_t ep_idx;
  usb_core_config_params_t usb_core_config = {0U};

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_config == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_INIT);

  switch (hpcd->instance)
  {
    case HAL_PCD_DRD_FS:

      usb_core_config.phy_interface = (usb_core_phy_module_t)p_config->phy_interface;
      usb_core_config.endpoints_nbr = hpcd->endpoints_nbr;
      usb_core_config.core_speed = (usb_core_speed_t)p_config->pcd_speed;
      usb_core_config.bcd_state = (usb_core_config_status_t)p_config->battery_charging_enable;

      break;

    default:
      break;
  }

  /* Prevent interrupts during USB device configuration */
  (void)hpcd->driver.core_disable_interrupts((uint32_t)hpcd->instance);

  /* Initialize the USB core */
  if (hpcd->driver.core_init((uint32_t)hpcd->instance, &usb_core_config) != USB_CORE_OK)
  {
    hpcd->global_state = HAL_PCD_STATE_FAULT;
    return HAL_ERROR;
  }

  /* Switch the USB core to device mode */
  if (hpcd->driver.core_set_mode((uint32_t)hpcd->instance, USB_CORE_DEVICE_MODE) != USB_CORE_OK)
  {
    hpcd->global_state = HAL_PCD_STATE_FAULT;
    ret = HAL_ERROR;
  }

  /* Init endpoints structures */
  for (ep_idx = 0U; ep_idx < USE_HAL_PCD_MAX_ENDPOINT_NB; ep_idx++)
  {
    /* Init ep structure */
    hpcd->in_ep[ep_idx].dir = HAL_PCD_EP_IN_DIR;
    hpcd->in_ep[ep_idx].num = (usb_core_endpoint_t)ep_idx;

    /* Control until ep is activated */
    hpcd->in_ep[ep_idx].type = USB_CORE_EP_TYPE_CTRL;
    hpcd->in_ep[ep_idx].max_packet = 0U;
    hpcd->in_ep[ep_idx].p_xfer_buffer = 0U;
    hpcd->in_ep[ep_idx].xfer_length = 0U;
  }

  for (ep_idx = 0U; ep_idx < USE_HAL_PCD_MAX_ENDPOINT_NB; ep_idx++)
  {
    hpcd->out_ep[ep_idx].dir = HAL_PCD_EP_OUT_DIR;
    hpcd->out_ep[ep_idx].num = (usb_core_endpoint_t)ep_idx;

    /* Control until ep is activated */
    hpcd->out_ep[ep_idx].type = USB_CORE_EP_TYPE_CTRL;
    hpcd->out_ep[ep_idx].max_packet = 0U;
    hpcd->out_ep[ep_idx].p_xfer_buffer = 0U;
    hpcd->out_ep[ep_idx].xfer_length = 0U;
  }

  /* Initialize device-specific registers and configuration */
  if (hpcd->driver.device_init((uint32_t)hpcd->instance, &usb_core_config) != USB_CORE_OK)
  {
    hpcd->global_state = HAL_PCD_STATE_FAULT;
    ret = HAL_ERROR;
  }

#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
  if (p_config->lpm_enable == HAL_PCD_LPM_ENABLED)
  {
    hpcd->driver.lpm_activate((uint32_t)hpcd->instance);
  }
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */

#if defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1)
  if (p_config->battery_charging_enable == HAL_PCD_BCD_ENABLED)
  {
    hpcd->driver.bcd_activate((uint32_t)hpcd->instance);
  }
#endif /* defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1) */

  hpcd->driver.device_disconnect((uint32_t)hpcd->instance);

  if (ret != HAL_ERROR)
  {
    hpcd->global_state = HAL_PCD_STATE_IDLE;
  }

  return ret;
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group3 User Data functions
  * @{
 A set of functions allowing to manage a user data pointer stored to the PCD handle:
 - HAL_PCD_SetUserData() Set the user data into the handle
 - HAL_PCD_GetUserData() Get the user data from the handle
  */
#if defined (USE_HAL_PCD_USER_DATA) && (USE_HAL_PCD_USER_DATA == 1)
/**
  * @brief Set the user data pointer into the handle.
  * @param hpcd Pointer to a hal_pcd_handle_t
  * @param p_user_data Pointer to the user data.
  */
void HAL_PCD_SetUserData(hal_pcd_handle_t *hpcd, const void *p_user_data)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  hpcd->p_user_data = p_user_data;
}

/**
  * @brief Get the user data pointer from the handle.
  * @param hpcd Pointer to a hal_pcd_handle_t
  * @retval Pointer to the user data.
  */
const void *HAL_PCD_GetUserData(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  return (hpcd->p_user_data);
}
#endif /* USE_HAL_PCD_USER_DATA */

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
/**
  * @brief Get Last Error codes.
  * @param hpcd Pointer to a hal_pcd_handle_t
  * @retval last error code.
  */
uint32_t HAL_PCD_GetLastErrorCodes(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  return (hpcd->last_error_codes);
}
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */
/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group4 Peripheral Control functions
  * @{
  */
/**
  * @brief  Start the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_Start(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hpcd, global_state, HAL_PCD_STATE_IDLE, HAL_PCD_STATE_ACTIVE);

  hpcd->driver.device_start((uint32_t)hpcd->instance);

  return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_Stop(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  (void)hpcd->driver.core_disable_interrupts((uint32_t)hpcd->instance);
  (void)hpcd->driver.device_disconnect((uint32_t)hpcd->instance);
  (void)hpcd->driver.device_stop((uint32_t)hpcd->instance);

  hpcd->device_state = HAL_PCD_PORT_STATE_DEV_DISCONNECT;
  hpcd->global_state = HAL_PCD_STATE_IDLE;

  return HAL_OK;
}


#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
/**
  * @brief  Start LPM (low Power Management) feature.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_LPM_Start(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  hpcd->driver.lpm_activate((uint32_t)hpcd->instance);
  hpcd->lpm_active = HAL_PCD_LPM_ENABLED;
  hpcd->lpm_state = HAL_PCD_LPM_STATE_L0;

  return HAL_OK;
}

/**
  * @brief  Stop LPM (low Power Management) feature.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_LPM_Stop(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  hpcd->driver.lpm_deactivate((uint32_t)hpcd->instance);
  hpcd->lpm_active = HAL_PCD_LPM_DISABLED;

  return HAL_OK;
}
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */


#if defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1)

/**
  * @brief  Start Battery Charging feature.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_BCD_Start(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_IDLE);

  hpcd->driver.bcd_activate((uint32_t)hpcd->instance);
  hpcd->battery_charging_active = HAL_PCD_BCD_ENABLED;

  return HAL_OK;
}

/**
  * @brief  Stop Battery Charging feature.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_BCD_Stop(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_IDLE);

  hpcd->driver.bcd_deactivate((uint32_t)hpcd->instance);
  hpcd->battery_charging_active = HAL_PCD_BCD_DISABLED;

  return HAL_OK;
}

/**
  * @brief  Port Type Detection Process.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_BCD_PortTypeDetection(hal_pcd_handle_t *hpcd)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait_start;
  hal_pcd_bcd_port_status_t port_type;

  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_IDLE);

  /* Wait for Min DCD Timeout (T_DCD_TIMEOUT >= 300ms per USB BCD spec 1.2) */
  wait_start = HAL_GetTick();
  while ((HAL_GetTick() - wait_start) < 350U)
  {
    /* Busy-wait */
  }

  /* Start BCD Primary detection */
  hpcd->driver.bcd_set_mode((uint32_t)hpcd->instance, USB_CORE_BCD_CONFIG_DCD, USB_CORE_BCD_CONFIG_STS_CLEAR);

  /* Wait for T_VDP_SRC_ON >= 40ms */
  wait_start = HAL_GetTick();
  while ((HAL_GetTick() - wait_start) < 50U)
  {
    /* Busy-wait */
  }

  hpcd->driver.bcd_set_mode((uint32_t)hpcd->instance, USB_CORE_BCD_CONFIG_PD, USB_CORE_BCD_CONFIG_STS_SET);

  /* Wait for primary detection result to stabilize */
  wait_start = HAL_GetTick();
  while ((HAL_GetTick() - wait_start) < 50U)
  {
    /* Busy-wait */
  }

  /* Determine the detected BCD port type */
  port_type = (hal_pcd_bcd_port_status_t)hpcd->driver.bcd_detect_port_type((uint32_t)hpcd->instance,
                                                                           USB_CORE_BCD_PRIMARY_DETECTION);

  if (port_type == HAL_PCD_BCD_PORT_STATUS_STD_DOWNSTREAM)
  {
    /* Standard Downstream Port */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_battery_charging_cb(hpcd, HAL_PCD_BCD_STD_DOWNSTREAM_PORT);
#else
    HAL_PCD_BcdCallback(hpcd, HAL_PCD_BCD_STD_DOWNSTREAM_PORT);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }
  else
  {
    /* Start BCD Secondary detection */
    hpcd->driver.bcd_set_mode((uint32_t)hpcd->instance, USB_CORE_BCD_CONFIG_PD, USB_CORE_BCD_CONFIG_STS_CLEAR);

    /* Wait for T_VDM_SRC_ON >= 40ms */
    wait_start = HAL_GetTick();
    while ((HAL_GetTick() - wait_start) < 50U)
    {
      /* Busy-wait */
    }

    hpcd->driver.bcd_set_mode((uint32_t)hpcd->instance, USB_CORE_BCD_CONFIG_SD, USB_CORE_BCD_CONFIG_STS_SET);

    /* Wait for secondary detection result to stabilize */
    wait_start = HAL_GetTick();
    while ((HAL_GetTick() - wait_start) < 50U)
    {
      /* Busy-wait */
    }

    /* Determine the detected BCD port type */
    port_type = (hal_pcd_bcd_port_status_t)hpcd->driver.bcd_detect_port_type((uint32_t)hpcd->instance,
                                                                             USB_CORE_BCD_SECONDARY_DETECTION);


    if (port_type == HAL_PCD_BCD_PORT_STATUS_DEDICATED_CHARGING)
    {
      /* Case Dedicated Charging Port  */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_battery_charging_cb(hpcd, HAL_PCD_BCD_DEDICATED_CHARGING_PORT);
#else
      HAL_PCD_BcdCallback(hpcd, HAL_PCD_BCD_DEDICATED_CHARGING_PORT);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
    else
    {
      /* Case Charging Downstream Port */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_battery_charging_cb(hpcd, HAL_PCD_BCD_CHARGING_DOWNSTREAM_PORT);
#else
      HAL_PCD_BcdCallback(hpcd, HAL_PCD_BCD_CHARGING_DOWNSTREAM_PORT);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
  }

  /* Battery Charging capability discovery finished */
  hpcd->driver.bcd_deactivate((uint32_t)hpcd->instance);
  hpcd->battery_charging_active = HAL_PCD_BCD_DISABLED;

  /* Check for the Timeout, else start USB Device */
  if ((HAL_GetTick() - tickstart) > 1000U)
  {
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_battery_charging_cb(hpcd, HAL_PCD_BCD_ERROR);
#else
    HAL_PCD_BcdCallback(hpcd, HAL_PCD_BCD_ERROR);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }
  else
  {
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_battery_charging_cb(hpcd, HAL_PCD_BCD_DISCOVERY_COMPLETED);
#else
    HAL_PCD_BcdCallback(hpcd, HAL_PCD_BCD_DISCOVERY_COMPLETED);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}
#endif /* defined (USE_HAL_PCD_USB_BCD) && (USE_HAL_PCD_USB_BCD == 1) */


/**
  * @brief  Connect the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_DeviceConnect(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  hpcd->driver.device_connect((uint32_t)hpcd->instance);

  return HAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_DeviceDisconnect(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  (void)hpcd->driver.device_disconnect((uint32_t)hpcd->instance);

  return HAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
hal_status_t HAL_PCD_SetDeviceAddress(hal_pcd_handle_t *hpcd, uint8_t address)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_ACTIVE);
  ASSERT_DBG_STATE(hpcd->device_state, HAL_PCD_PORT_STATE_DEV_RUN);

  hpcd->usb_address = address;
  (void)hpcd->driver.device_set_address((uint32_t)hpcd->instance, address);

  return HAL_OK;
}

/**
  * @brief  Get the USB Device speed.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_pcd_device_speed_t HAL_PCD_GetDeviceSpeed(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_ACTIVE);
  ASSERT_DBG_STATE(hpcd->device_state, HAL_PCD_PORT_STATE_DEV_RUN);

  return (hal_pcd_device_speed_t)hpcd->driver.device_get_speed((uint32_t)hpcd->instance);
}

/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
hal_status_t HAL_PCD_OpenEndpoint(hal_pcd_handle_t *hpcd, uint8_t ep_addr, uint16_t ep_mps, hal_pcd_ep_type_t ep_type)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));
  ASSERT_DBG_PARAM((HAL_PCD_CHECK_EP_TYPE(ep_type) != 0U));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL)
      || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB))
      || (HAL_PCD_CHECK_EP_TYPE(ep_type) == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;
    p_ep = &hpcd->out_ep[ep_num];
  }

  p_ep->num = (usb_core_endpoint_t)ep_num;
  p_ep->max_packet = (uint32_t)ep_mps & 0x7FFU;
  p_ep->type = (usb_core_ep_type_t)ep_type;

  (void)hpcd->driver.ep_activate((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
hal_status_t HAL_PCD_CloseEndpoint(hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;
    p_ep = &hpcd->out_ep[ep_num];
  }

  p_ep->num = (usb_core_endpoint_t)ep_num;

  (void)hpcd->driver.ep_deactivate((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}


/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  p_buffer pointer to the reception buffer
  * @param  size_byte amount of data to be received
  * @retval HAL status
  */
hal_status_t HAL_PCD_SetEndpointReceive(hal_pcd_handle_t *hpcd, uint8_t ep_addr, uint8_t *p_buffer, uint32_t size_byte)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;

  /* Get Endpoint OUT object address */
  p_ep = &hpcd->out_ep[ep_num];

  /* Configure the endpoint buffer and initiate the transfer */
  p_ep->p_xfer_buffer = p_buffer;
  p_ep->xfer_length = size_byte;
  p_ep->xfer_count = 0U;
  p_ep->num = (usb_core_endpoint_t)ep_num;

  p_ep = &hpcd->out_ep[ep_num];

  (void)hpcd->driver.ep_start_transfer((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint32_t HAL_PCD_EP_GetRxCount(const hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  uint16_t ep_num = (uint16_t)ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_addr & USB_CORE_IN_EP_DIR_MSK) == 0x0U);
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return 0U;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, HAL_PCD_STATE_ACTIVE);

  return hpcd->out_ep[ep_num].xfer_count;
}

/**
  * @brief  Send an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  p_buffer pointer to the transmission buffer
  * @param  size_byte amount of data to be sent
  * @retval HAL status
  */
hal_status_t HAL_PCD_SetEndpointTransmit(hal_pcd_handle_t *hpcd, uint8_t ep_addr, uint8_t *p_buffer, uint32_t size_byte)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }

  if ((p_buffer == NULL) && (size_byte != 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;

  p_ep = &hpcd->in_ep[ep_num];

  /* Configure the endpoint buffer and initiate the transfer */
  p_ep->p_xfer_buffer = p_buffer;
  p_ep->xfer_length = size_byte;
  p_ep->xfer_size = size_byte;
  p_ep->xfer_count = 0U;
  p_ep->num = (usb_core_endpoint_t)ep_num;

  (void)hpcd->driver.ep_start_transfer((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
hal_status_t HAL_PCD_SetEndpointStall(hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;
    p_ep = &hpcd->out_ep[ep_num];
  }

  p_ep->num = (usb_core_endpoint_t)ep_num;

  (void)hpcd->driver.ep_set_stall((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
hal_status_t HAL_PCD_ClearEndpointStall(hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;
    p_ep = &hpcd->out_ep[ep_num];
  }

  p_ep->num = (usb_core_endpoint_t)ep_num;

  (void)hpcd->driver.ep_clear_stall((uint32_t)hpcd->instance, p_ep);

  return HAL_OK;
}

/**
   * @brief  Abort an USB endpoint transaction.
   * @param  hpcd PCD handle
   * @param  ep_addr endpoint address
   * @retval HAL status
   */
hal_status_t HAL_PCD_AbortEndpointTransfer(hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  const hal_pcd_ep_t *p_ep;
  uint8_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    p_ep = &hpcd->out_ep[ep_num];
  }

  /* Abort any ongoing transfer on this endpoint */
  if (hpcd->driver.ep_stop_transfer((uint32_t)hpcd->instance, p_ep) != USB_CORE_OK)
  {
    return HAL_ERROR;
  }

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  hpcd->p_ep_abort_transfer_cb(hpcd, ep_num);
#else
  HAL_PCD_EndpointAbortTransferCallback(hpcd, ep_num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  return HAL_OK;
}

/**
  * @brief  Flush an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
hal_status_t HAL_PCD_FlushEndpoint(const hal_pcd_handle_t *hpcd, uint8_t ep_addr)
{
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_addr);

  return HAL_OK;
}

/**
  * @brief  Start remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_RemoteWakeup_Start(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM(hpcd != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  (void)hpcd->driver.remote_wakeup_activate((uint32_t)hpcd->instance);

  return HAL_OK;
}

/**
  * @brief  Stop remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
hal_status_t HAL_PCD_RemoteWakeup_Stop(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM(hpcd != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  (void)hpcd->driver.remote_wakeup_deactivate((uint32_t)hpcd->instance);

  return HAL_OK;
}


/**
  * @brief  Configure PMA for endpoints.
  * @param  hpcd  Device instance
  * @param  ep_addr endpoint address
  * @param  ep_kind endpoint Kind
  *                  HAL_PCD_SNG_BUF: Single Buffer used
  *                  HAL_PCD_DBL_BUF: Double Buffer used
  * @param  pma_address: endpoint buffer address in The PMA: In case of single buffer endpoint
  *                   this parameter is 16-bit value providing the address of endpoint buffer in PMA.
  *                   In case of double buffer endpoint this parameter
  *                   is a 32-bit value providing the endpoint buffer 0 address
  *                   in the LSB part of 32-bit value and endpoint buffer 1 address
  *                   in the MSB part of 32-bit value.
  * @retval HAL status
  */
hal_status_t HAL_PCD_PMAConfig(hal_pcd_handle_t *hpcd, uint16_t ep_addr, uint16_t ep_kind, uint32_t pma_address)
{
  hal_pcd_ep_t *p_ep;
  uint16_t ep_num = ep_addr & HAL_PCD_EP_ADDR_MSK;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((ep_num < (uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (ep_num >= ((uint8_t)USE_HAL_PCD_MAX_ENDPOINT_NB)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hpcd->global_state, ((uint32_t)HAL_PCD_STATE_IDLE)
                   | ((uint32_t)HAL_PCD_STATE_ACTIVE));

  /* Initialize ep structure */
  if ((ep_addr & USB_CORE_IN_EP_DIR_MSK) == USB_CORE_IN_EP_DIR_MSK)
  {
    hpcd->in_ep[ep_num].dir = HAL_PCD_EP_IN_DIR;
    p_ep = &hpcd->in_ep[ep_num];
  }
  else
  {
    hpcd->out_ep[ep_num].dir = HAL_PCD_EP_OUT_DIR;
    p_ep = &hpcd->out_ep[ep_num];
  }

  /* Here we check if the endpoint is single or double Buffer */
  if (ep_kind == (uint16_t)HAL_PCD_SNG_BUF)
  {
    /* Disable endpoint Double Buffer mode */
    p_ep->double_buffer_en = 0U;

    /* Configure the PMA buffer address */
    p_ep->pma_address = (uint16_t)pma_address;
  }
#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
  else /* HAL_PCD_DBL_BUF */
  {
    /* Enable endpoint Double Buffer mode */
    p_ep->double_buffer_en = 1U;

    /* Configure the PMA double buffer address */
    p_ep->pma_addr0 = (uint16_t)(pma_address & 0xFFFFU);
    p_ep->pma_addr1 = (uint16_t)((pma_address & 0xFFFF0000U) >> 16);
  }
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group5 Peripheral State functions
  * @{
  */

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval HAL state
  */
hal_pcd_state_t HAL_PCD_GetState(const hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));

  return hpcd->global_state;
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group6 IRQ handling functions
  * @{
  */
/**
  * @brief  Dispatch the PCD interrupt request.
  * @param  hpcd PCD handle
  */
void HAL_PCD_IRQHandler(hal_pcd_handle_t *hpcd)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((hpcd->p_irq_handler != NULL));

  hpcd->current_mode = hpcd->driver.core_get_mode((uint32_t)hpcd->instance);

  hpcd->p_irq_handler(hpcd);
}


/**
  * @brief  Handle the USB DRD FS interrupt request.
  * @param  hpcd PCD handle
  */
void HAL_PCD_DRD_IRQHandler(hal_pcd_handle_t *hpcd)
{
  hal_status_t ep_istr_status;
  usb_drd_global_t *p_usb;
  uint32_t istr_reg;


  ASSERT_DBG_PARAM((hpcd != NULL));

  p_usb = USB_DRD_GET_INSTANCE((uint32_t)hpcd->instance);
  istr_reg = USB_DRD_ReadInterrupts((uint32_t)p_usb);

  if ((istr_reg & USB_ISTR_CTR) == USB_ISTR_CTR)
  {
    /* Servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the endpoint routine */
    ep_istr_status = PCD_DRD_EP_ISR_Handler(hpcd);

    if (ep_istr_status != HAL_OK)
    {
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_error_cb(hpcd);
#else
      HAL_PCD_ErrorCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }

    return;
  }

  if ((istr_reg & USB_ISTR_RESET) == USB_ISTR_RESET)
  {
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_RESET);

    /* Ensure EP0 is disabled */
    USB_DRD_PCD_SET_EP_TX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_TX_DIS);
    USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_RX_DIS);

    hpcd->device_state = HAL_PCD_PORT_STATE_DEV_RESET;

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_reset_cb(hpcd);
#else
    HAL_PCD_ResetCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    hpcd->usb_address = 0U;
    (void)hpcd->driver.device_set_address((uint32_t)p_usb, hpcd->usb_address);

    return;
  }

  if ((istr_reg & USB_ISTR_PMAOVR) == USB_ISTR_PMAOVR)
  {
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_PMAOVR);

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_error_cb(hpcd);
#else
    HAL_PCD_ErrorCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

  if ((istr_reg & USB_ISTR_ERR) == USB_ISTR_ERR)
  {
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_ERR);

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_error_cb(hpcd);
#else
    HAL_PCD_ErrorCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

  if ((istr_reg & USB_ISTR_WKUP) == USB_ISTR_WKUP)
  {
    p_usb->CNTR &= ~(USB_CNTR_SUSPRDY);
    p_usb->CNTR &= ~(USB_CNTR_SUSPEN);

    hpcd->device_state = HAL_PCD_PORT_STATE_DEV_RUN;

#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
    if (hpcd->lpm_state == HAL_PCD_LPM_STATE_L1)
    {
      hpcd->lpm_state = HAL_PCD_LPM_STATE_L0;
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_low_power_management_cb(hpcd, HAL_PCD_LPM_L0_ACTIVE);
#else
      HAL_PCD_LpmCallback(hpcd, HAL_PCD_LPM_L0_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_resume_cb(hpcd);
#else
    HAL_PCD_ResumeCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_WKUP);

    return;
  }

  if ((istr_reg & USB_ISTR_SUSP) == USB_ISTR_SUSP)
  {
    /* Force low-power mode in the macrocell */
    p_usb->CNTR |= USB_CNTR_SUSPEN;

    /* Clear of the ISTR bit is done after setting CNTR_FSUSP */
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_SUSP);

    p_usb->CNTR |= USB_CNTR_SUSPRDY;

    hpcd->device_state = HAL_PCD_PORT_STATE_DEV_SUSPEND;

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_suspend_cb(hpcd);
#else
    HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

#if defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1)
  /* Handle LPM (low Power Management) Interrupt */
  if ((istr_reg & USB_ISTR_L1REQ) == USB_ISTR_L1REQ)
  {
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_L1REQ);
    if (hpcd->lpm_state == HAL_PCD_LPM_STATE_L0)
    {
      /* Force suspend and low-power mode before going to L1 state*/
      p_usb->CNTR |= USB_CNTR_SUSPRDY;
      p_usb->CNTR |= USB_CNTR_SUSPEN;

      hpcd->lpm_state = HAL_PCD_LPM_STATE_L1;
      hpcd->lpm_besl = ((uint32_t)p_usb->LPMCSR & USB_LPMCSR_BESL) >> 2;
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_low_power_management_cb(hpcd, HAL_PCD_LPM_L1_ACTIVE);
#else
      HAL_PCD_LpmCallback(hpcd, HAL_PCD_LPM_L1_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
    else
    {
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_suspend_cb(hpcd);
#else
      HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }

    return;
  }
#endif /* defined (USE_HAL_PCD_USB_LPM) && (USE_HAL_PCD_USB_LPM == 1) */

  if ((istr_reg & USB_ISTR_SOF) == USB_ISTR_SOF)
  {
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_SOF);

#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->p_sof_cb(hpcd);
#else
    HAL_PCD_SofCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

  if ((istr_reg & USB_ISTR_ESOF) == USB_ISTR_ESOF)
  {
    /* Clear ESOF flag in ISTR */
    USB_DRD_ClearInterrupts((uint32_t)p_usb, USB_ISTR_ESOF);

    return;
  }
}


/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group7 Default Callbacks functions
  * @{
  */
/**
  * @brief  Send LPM (low Power Management) active status to user layer callback.
  * @param  hpcd PCD handle
  * @param  lpm_status LPM active status
  */
__WEAK void HAL_PCD_LpmCallback(hal_pcd_handle_t *hpcd, hal_pcd_lpm_active_status_t lpm_status)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(lpm_status);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_LpmCallback could be implemented in the user file
   */
}

/**
  * @brief  Send BatteryCharging port type to user layer callback.
  * @param  hpcd PCD handle
  * @param  port_type port type
  */
__WEAK void HAL_PCD_BcdCallback(hal_pcd_handle_t *hpcd, hal_pcd_bcd_port_type_t port_type)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(port_type);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_BcdCallback could be implemented in the user file
   */
}

/**
  * @brief  Data OUT stage callback.
  * @param  hpcd PCD handle
  * @param  ep_num endpoint number
  */
__WEAK void HAL_PCD_DataOutStageCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_num);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */
}

/**
  * @brief  Data IN stage callback.
  * @param  hpcd PCD handle
  * @param  ep_num endpoint number
  */
__WEAK void HAL_PCD_DataInStageCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_num);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_DataInStageCallback could be implemented in the user file
   */
}
/**
  * @brief  Setup stage callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_SetupStageCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_SetupStageCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Start Of Frame callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_SofCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_SofCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_ResetCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ResetCallback could be implemented in the user file
   */
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_SuspendCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_SuspendCallback could be implemented in the user file
   */
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_ResumeCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ResumeCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd PCD handle
  * @param  ep_num endpoint number
  */
__WEAK void HAL_PCD_ISOOUTIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_num);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd PCD handle
  * @param  ep_num endpoint number
  */
__WEAK void HAL_PCD_ISOINIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_num);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Connection event callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_ConnectCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ConnectCallback could be implemented in the user file
   */
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_DisconnectCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_DisconnectCallback could be implemented in the user file
   */
}

/**
  * @brief  PCD Error callback.
  * @param  hpcd PCD handle
  */
__WEAK void HAL_PCD_ErrorCallback(hal_pcd_handle_t *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  PCD Endpoint Abort Transfer callback.
  * @param  hpcd PCD handle
  * @param  ep_num endpoint number
  */
__WEAK void HAL_PCD_EndpointAbortTransferCallback(hal_pcd_handle_t *hpcd, uint8_t ep_num)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hpcd);
  STM32_UNUSED(ep_num);

  /* WARNING : This function must not be modified, when the callback is needed,
            the HAL_PCD_EndpointAbortTransferCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group8 Register Callbacks functions
  * @{
  */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register USB PCD SOF callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the SOF callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterSofCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_sof_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Setup callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Setup callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterSetupCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_setup_stage_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Reset callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Reset callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterResetCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_reset_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Suspend callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Suspend callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterSuspendCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_suspend_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Resume callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Resume callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterResumeCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_resume_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Connect callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Connect callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterConnectCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_connect_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a USB PCD Disconnect callback
  *         To be used instead of the weak predefined callback.
  * @param  hpcd USB PCD handle
  * @param  p_callback pointer to the Disconnect callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterDisconnectCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_disconnect_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Data OUT Stage callback
  *         To be used instead of the weak HAL_PCD_DataOutStageCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Data OUT Stage callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterDataOutStageCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_data_out_stage_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Data IN Stage callback
  *         To be used instead of the weak HAL_PCD_DataInStageCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Data IN Stage callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterDataInStageCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_data_in_stage_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Iso OUT incomplete callback
  *         To be used instead of the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Iso OUT incomplete callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterIsoOutIncpltCallback(hal_pcd_handle_t *hpcd, hal_pcd_iso_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_iso_out_incomplete_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Iso IN incomplete callback
  *         To be used instead of the weak HAL_PCD_ISOINIncompleteCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Iso IN incomplete callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterIsoInIncpltCallback(hal_pcd_handle_t *hpcd, hal_pcd_iso_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_iso_in_incomplete_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Error callback
  *         To be used instead of the weak HAL_PCD_ErrorCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Error callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterErrorCallback(hal_pcd_handle_t *hpcd, hal_pcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD Endpoint Abort Transfer callback
  *         To be used instead of the weak HAL_PCD_EndpointAbortTransferCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD Endpoint Abort Transfer callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterEndpointAbortTransferCallback(hal_pcd_handle_t *hpcd, hal_pcd_data_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_ep_abort_transfer_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD BCD callback
  *         To be used instead of the weak HAL_PCD_BcdCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD BCD callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterBcdCallback(hal_pcd_handle_t *hpcd, hal_pcd_bcd_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_battery_charging_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register USB PCD LPM callback
  *         To be used instead of the weak HAL_PCD_LpmCallback() predefined callback.
  * @param  hpcd PCD handle
  * @param  p_callback pointer to the USB PCD LPM callback function
  * @retval HAL status
  */
hal_status_t HAL_PCD_RegisterLpmCallback(hal_pcd_handle_t *hpcd, hal_pcd_lpm_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hpcd->p_low_power_management_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup PCD_Private_Functions
  * @{
  */


/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static hal_status_t PCD_DRD_EP_ISR_Handler(hal_pcd_handle_t *hpcd)
{
  usb_drd_global_t *p_usb;
  hal_pcd_ep_t *p_ep;
  uint32_t ctr_count = 0U;
  uint16_t rx_count;
  uint16_t istr_reg;
  uint16_t ep_value;
  uint16_t tx_packet_size;
  uint8_t ep_idx;
  usb_core_phy_ep_t phy_ep_num;

  ASSERT_DBG_PARAM((hpcd != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hpcd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 0U)
  rx_count = 0U;
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 0U) */

  p_usb = USB_DRD_GET_INSTANCE((uint32_t)hpcd->instance);

  /* Stay in loop while pending interrupts */
  while (((p_usb->ISTR & USB_ISTR_CTR) != 0U) && (ctr_count < USB_DRD_MAX_CTR_ITERATIONS))
  {
    hpcd->device_state = HAL_PCD_PORT_STATE_DEV_RUN;

    istr_reg = (uint16_t)p_usb->ISTR;

    /* extract highest priority endpoint number */
    ep_idx = (uint8_t)(istr_reg & USB_ISTR_IDN);

    if (ep_idx >= USE_HAL_PCD_MAX_ENDPOINT_NB)
    {
      (void)hpcd->driver.core_disable_interrupts((uint32_t)p_usb);

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
      hpcd->last_error_codes |= HAL_PCD_ERROR_EP_INDEX;
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

      return HAL_ERROR;
    }

    if (ep_idx == 0U)
    {
      /* Decode and service control endpoint interrupt */

      /* DIR bit = origin of the interrupt */
      if ((istr_reg & USB_ISTR_DIR) == 0U)
      {
        /* DIR = 0 */

        /* DIR = 0 => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always */
        USB_DRD_PCD_CLEAR_TX_EP_CTR((uint32_t)p_usb, USB_CORE_PHY_CHEP_0);
        p_ep = &hpcd->in_ep[0];

        /* Get Endpoint Physical number */
        phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

        p_ep->xfer_count = USB_DRD_PCD_GET_EP_TX_CNT((uint32_t)p_usb, phy_ep_num);
        p_ep->p_xfer_buffer += p_ep->xfer_count;

        /* TX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->p_data_in_stage_cb(hpcd, 0U);
#else
        HAL_PCD_DataInStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

        if ((hpcd->usb_address > 0U) && (p_ep->xfer_length == 0U))
        {
          p_usb->DADDR = ((uint16_t)hpcd->usb_address | USB_DADDR_EF);
          hpcd->usb_address = 0U;
        }
      }
      else
      {
        /* DIR = 1 */

        /* DIR = 1 & CTR_RX => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
        p_ep = &hpcd->out_ep[0];

        /* Get Endpoint Physical number */
        phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

        ep_value = (uint16_t)USB_DRD_PCD_GET_ENDPOINT((uint32_t)p_usb, USB_CORE_PHY_CHEP_0);

        if ((ep_value & USB_EP_SETUP) != 0U)
        {
          /* Get SETUP Packet */
          p_ep->xfer_count = USB_DRD_PCD_GET_EP_RX_CNT((uint32_t)p_usb, phy_ep_num);

          if (p_ep->xfer_count != 8U)
          {
            /* Set Stall condition for EP0 IN/OUT */
            USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_RX_STALL);
            USB_DRD_PCD_SET_EP_TX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_TX_STALL);

            /* SETUP bit kept frozen while CTR_RX = 1 */
            USB_DRD_PCD_CLEAR_RX_EP_CTR((uint32_t)p_usb, USB_CORE_PHY_CHEP_0);

            return HAL_OK;
          }

          USB_DRD_ReadPMA((uint32_t)p_usb, (uint8_t *)hpcd->setup,
                          p_ep->pma_address, (uint16_t)p_ep->xfer_count);

          /* SETUP bit kept frozen while CTR_RX = 1 */
          USB_DRD_PCD_CLEAR_RX_EP_CTR((uint32_t)p_usb, USB_CORE_PHY_CHEP_0);

          /* Process SETUP Packet */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->p_setup_stage_cb(hpcd);
#else
          HAL_PCD_SetupStageCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else if ((ep_value & USB_EP_VTRX) != 0U)
        {
          USB_DRD_PCD_CLEAR_RX_EP_CTR((uint32_t)p_usb, USB_CORE_PHY_CHEP_0);

          /* Get Control Data OUT Packet */
          p_ep->xfer_count = USB_DRD_PCD_GET_EP_RX_CNT((uint32_t)p_usb, phy_ep_num);

          if (p_ep->xfer_count == 0U)
          {
            /* Status phase re-arm for next setup */
            USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_RX_VALID);
          }
          else
          {
            if (p_ep->p_xfer_buffer != NULL)
            {
              USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer,
                              p_ep->pma_address, (uint16_t)p_ep->xfer_count);

              p_ep->p_xfer_buffer += p_ep->xfer_count;

              /* Process Control Data OUT Packet */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->p_data_out_stage_cb(hpcd, 0U);
#else
              HAL_PCD_DataOutStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }
            else
            {
              USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, USB_CORE_PHY_CHEP_0, USB_EP_RX_STALL);

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
              hpcd->last_error_codes |= HAL_PCD_ERROR_OUT_EP_PACKET;
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

              return HAL_ERROR;
            }
          }
        }
        else
        {
          /* .. */
        }
      }
    }
    else
    {
      /* Decode and service non control endpoints interrupt */
      /* Process related endpoint register */
      ep_value = (uint16_t)USB_DRD_PCD_GET_ENDPOINT((uint32_t)p_usb, (usb_core_phy_chep_t)ep_idx);

      if ((ep_value & USB_EP_VTRX) != 0U)
      {
        /* Clear int flag */
        USB_DRD_PCD_CLEAR_RX_EP_CTR((uint32_t)p_usb, (usb_core_phy_chep_t)ep_idx);
        p_ep = &hpcd->out_ep[ep_idx];

        /* Get Endpoint Physical number */
        phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

        /* OUT Single Buffering */
        if (p_ep->double_buffer_en == 0U)
        {
          rx_count = (uint16_t)USB_DRD_PCD_GET_EP_RX_CNT((uint32_t)p_usb, phy_ep_num);

          if (rx_count != 0U)
          {
            USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_address, rx_count);
          }
        }
#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
        else
        {
          /* Manage double buffer bulk out */
          if (p_ep->type == USB_CORE_EP_TYPE_BULK)
          {
            rx_count = HAL_PCD_EP_DB_Receive(hpcd, p_ep, ep_value);
          }
#if defined (USE_HAL_PCD_USB_EP_TYPE_ISOC) && (USE_HAL_PCD_USB_EP_TYPE_ISOC == 1)
          else /* Manage double buffer iso out */
          {
            /* OUT double buffered endpoint */
            USB_DRD_TX_DTOG((uint32_t)p_usb, phy_ep_num);

            if ((USB_DRD_PCD_GET_ENDPOINT((uint32_t)p_usb, phy_ep_num) & USB_EP_DTOGRX) != 0U)
            {
              /* Read from endpoint BUF0Addr buffer */
              rx_count = (uint16_t)USB_DRD_PCD_GET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num);

              if (rx_count != 0U)
              {
                USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr0, rx_count);
              }
            }
            else
            {
              /* Read from endpoint BUF1Addr buffer */
              rx_count = (uint16_t)USB_DRD_PCD_GET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num);

              if (rx_count != 0U)
              {
                USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr1, rx_count);
              }
            }
          }
#endif /* defined (USE_HAL_PCD_USB_EP_TYPE_ISOC) && (USE_HAL_PCD_USB_EP_TYPE_ISOC == 1) */
        }
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */

        /* Multi-packet on the NON control OUT endpoint */
        p_ep->xfer_count += rx_count;

        if ((p_ep->xfer_length == 0U) || (rx_count < p_ep->max_packet))
        {
          /* RX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->p_data_out_stage_cb(hpcd, p_ep->num);
#else
          HAL_PCD_DataOutStageCallback(hpcd, p_ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          p_ep->p_xfer_buffer += rx_count;
          (void)USB_DRD_StartEndpointXfer((uint32_t)p_usb, p_ep);
        }
      }

      if ((ep_value & USB_EP_VTTX) != 0U)
      {
        p_ep = &hpcd->in_ep[ep_idx];

        /* Get Endpoint Physical number */
        phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

        /* Clear int flag */
        USB_DRD_PCD_CLEAR_TX_EP_CTR((uint32_t)p_usb, phy_ep_num);

#if defined (USE_HAL_PCD_USB_EP_TYPE_ISOC) && (USE_HAL_PCD_USB_EP_TYPE_ISOC == 1)
        if (p_ep->type == USB_CORE_EP_TYPE_ISOC)
        {
          p_ep->xfer_length = 0U;

#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
          if (p_ep->double_buffer_en != 0U)
          {
            if ((ep_value & USB_EP_DTOGTX) != 0U)
            {
              USB_DRD_PCD_SET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);
            }
            else
            {
              USB_DRD_PCD_SET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);
            }
          }
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */

          /* TX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->p_data_in_stage_cb(hpcd, p_ep->num);
#else
          HAL_PCD_DataInStageCallback(hpcd, p_ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
#endif /* defined (USE_HAL_PCD_USB_EP_TYPE_ISOC) && (USE_HAL_PCD_USB_EP_TYPE_ISOC == 1) */
        {
          /* Manage Single Buffer Transaction */
          if ((ep_value & USB_EP_EPKIND) == 0U)
          {
            /* Multi-packet on the NON control IN endpoint */
            tx_packet_size = (uint16_t)USB_DRD_PCD_GET_EP_TX_CNT((uint32_t)p_usb, phy_ep_num);

            if (p_ep->xfer_length > tx_packet_size)
            {
              p_ep->xfer_length -= tx_packet_size;
            }
            else
            {
              p_ep->xfer_length = 0U;
            }

            /* Zero Length Packet? */
            if (p_ep->xfer_length == 0U)
            {
              /* TX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->p_data_in_stage_cb(hpcd, p_ep->num);
#else
              HAL_PCD_DataInStageCallback(hpcd, p_ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }
            else
            {
              /* Transfer is not yet Done */
              p_ep->p_xfer_buffer += tx_packet_size;
              p_ep->xfer_count += tx_packet_size;
              (void)USB_DRD_StartEndpointXfer((uint32_t)p_usb, p_ep);
            }
          }
#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
          /* Double Buffer bulk IN (bulk transfer Len > Ep_Mps) */
          else
          {
            (void)HAL_PCD_EP_DB_Transmit(hpcd, p_ep, ep_value);
          }
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */
        }
      }
    }

    ctr_count++;
  }

#if defined (USE_HAL_PCD_GET_LAST_ERRORS) && (USE_HAL_PCD_GET_LAST_ERRORS == 1)
  /* If still pending CTR, report error */
  if ((p_usb->ISTR & USB_ISTR_CTR) != 0U)
  {
    hpcd->last_error_codes |= HAL_PCD_ERROR_CTR_STUCK;
  }
#endif /* USE_HAL_PCD_GET_LAST_ERRORS */

  return HAL_OK;
}


#if defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1)
/**
  * @brief  Manage double buffer bulk out transaction from ISR.
  * @param  hpcd PCD handle
  * @param  p_ep current endpoint handle
  * @param  ep_value Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static uint16_t HAL_PCD_EP_DB_Receive(hal_pcd_handle_t *hpcd, hal_pcd_ep_t *p_ep, uint16_t ep_value)
{
  const usb_drd_global_t *p_usb;
  uint16_t rx_count;
  usb_core_phy_ep_t phy_ep_num;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_ep != NULL));

  /* Get Endpoint Physical number */
  phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

  p_usb = USB_DRD_GET_INSTANCE((uint32_t)hpcd->instance);

  /* Manage Buffer0 OUT */
  if ((ep_value & USB_EP_DTOGRX) != 0U)
  {
    /* Get count of received Data on buffer0 */
    rx_count = (uint16_t)USB_DRD_PCD_GET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num);

    if (p_ep->xfer_length >= rx_count)
    {
      p_ep->xfer_length -= rx_count;
    }
    else
    {
      p_ep->xfer_length = 0U;
    }

    if (p_ep->xfer_length == 0U)
    {
      /* Set NAK to OUT endpoint since double buffer is enabled */
      USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, phy_ep_num, USB_EP_RX_NAK);
    }

    /* Check if Buffer1 is in blocked state which requires to toggle */
    if ((ep_value & USB_EP_DTOGTX) != 0U)
    {
      /* OUT double buffered endpoint */
      USB_DRD_TX_DTOG((uint32_t)p_usb, phy_ep_num);
    }

    if (rx_count != 0U)
    {
      USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr0, rx_count);
    }
  }
  /* Manage Buffer 1 DTOG_RX=0 */
  else
  {
    /* Get count of received data */
    rx_count = (uint16_t)USB_DRD_PCD_GET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num);

    if (p_ep->xfer_length >= rx_count)
    {
      p_ep->xfer_length -= rx_count;
    }
    else
    {
      p_ep->xfer_length = 0U;
    }

    if (p_ep->xfer_length == 0U)
    {
      /* Set NAK on the current endpoint */
      USB_DRD_PCD_SET_EP_RX_STATUS((uint32_t)p_usb, phy_ep_num, USB_EP_RX_NAK);
    }

    /* Need to FreeUser Buffer */
    if ((ep_value & USB_EP_DTOGTX) == 0U)
    {
      /* OUT double buffered endpoint */
      USB_DRD_TX_DTOG((uint32_t)p_usb, phy_ep_num);
    }

    if (rx_count != 0U)
    {
      USB_DRD_ReadPMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr1, rx_count);
    }
  }

  return rx_count;
}

/**
  * @brief  Manage double buffer bulk IN transaction from ISR.
  * @param  hpcd PCD handle
  * @param  p_ep current endpoint handle
  * @param  ep_value Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static hal_status_t HAL_PCD_EP_DB_Transmit(hal_pcd_handle_t *hpcd, hal_pcd_ep_t *p_ep, uint16_t ep_value)
{
  const usb_drd_global_t *p_usb;
  uint32_t length;
  uint16_t tx_packet_size;
  usb_core_phy_ep_t phy_ep_num;

  ASSERT_DBG_PARAM((hpcd != NULL));
  ASSERT_DBG_PARAM((p_ep != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((hpcd == NULL) || (p_ep == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Get Endpoint Physical number */
  phy_ep_num = (usb_core_phy_ep_t)p_ep->num;

  p_usb = USB_DRD_GET_INSTANCE((uint32_t)hpcd->instance);

  /* Data Buffer0 ACK received */
  if ((ep_value & USB_EP_DTOGTX) != 0U)
  {
    /* Multi-packet on the NON control IN endpoint */
    tx_packet_size = (uint16_t)USB_DRD_PCD_GET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num);

    if (p_ep->xfer_length > tx_packet_size)
    {
      p_ep->xfer_length -= tx_packet_size;
    }
    else
    {
      p_ep->xfer_length = 0U;
    }

    /* Transfer is completed */
    if (p_ep->xfer_length == 0U)
    {
      USB_DRD_PCD_SET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);
      USB_DRD_PCD_SET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);

      if (p_ep->type == USB_CORE_EP_TYPE_BULK)
      {
        /* Set Bulk endpoint in NAK state */
        USB_DRD_PCD_SET_EP_TX_STATUS((uint32_t)p_usb, phy_ep_num, USB_EP_TX_NAK);
      }

      /* TX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_data_in_stage_cb(hpcd, p_ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, p_ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      if ((ep_value & USB_EP_DTOGRX) != 0U)
      {
        /* IN double buffered endpoint */
        USB_DRD_RX_DTOG((uint32_t)p_usb, phy_ep_num);
      }

      return HAL_OK;
    }
    else /* Transfer is not yet Done */
    {
      /* Need to Free USB Buffer */
      if ((ep_value & USB_EP_DTOGRX) != 0U)
      {
        /* IN double buffered endpoint */
        USB_DRD_RX_DTOG((uint32_t)p_usb, phy_ep_num);
      }

      /* Still there is data to Fill in the next Buffer */
      if (p_ep->xfer_fill_db == 1U)
      {
        p_ep->p_xfer_buffer += tx_packet_size;
        p_ep->xfer_count += tx_packet_size;

        /* Calculate the length of the new buffer to fill */
        if (p_ep->xfer_size >= p_ep->max_packet)
        {
          length = p_ep->max_packet;
          p_ep->xfer_size -= length;
        }
        else if (p_ep->xfer_size == 0U)
        {
          length = tx_packet_size;
          p_ep->xfer_fill_db = 0U;
        }
        else
        {
          p_ep->xfer_fill_db = 0U;
          length = p_ep->xfer_size;
          p_ep->xfer_size = 0U;
        }

        /* Write remaining Data to Buffer */
        /* Set the Double buffer counter for pma buffer0 */
        USB_DRD_PCD_SET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, length);

        /* Copy user buffer to USB PMA */
        USB_DRD_WritePMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr0, (uint16_t)length);
      }
    }
  }
  else /* Data Buffer1 ACK received */
  {
    /* multi-packet on the NON control IN endpoint */
    tx_packet_size = (uint16_t)USB_DRD_PCD_GET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num);

    if (p_ep->xfer_length >= tx_packet_size)
    {
      p_ep->xfer_length -= tx_packet_size;
    }
    else
    {
      p_ep->xfer_length = 0U;
    }

    /* Transfer is completed */
    if (p_ep->xfer_length == 0U)
    {
      USB_DRD_PCD_SET_EP_DBUF0_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);
      USB_DRD_PCD_SET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, 0U);

      if (p_ep->type == USB_CORE_EP_TYPE_BULK)
      {
        /* Set Bulk endpoint in NAK state */
        USB_DRD_PCD_SET_EP_TX_STATUS((uint32_t)p_usb, phy_ep_num, USB_EP_TX_NAK);
      }

      /* TX COMPLETE */
#if defined (USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->p_data_in_stage_cb(hpcd, p_ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, p_ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      /* Need to Free USB Buff */
      if ((ep_value & USB_EP_DTOGRX) == 0U)
      {
        /* IN double buffered endpoint */
        USB_DRD_RX_DTOG((uint32_t)p_usb, phy_ep_num);
      }

      return HAL_OK;
    }
    else /* Transfer is not yet Done */
    {
      /* Need to Free USB Buffer */
      if ((ep_value & USB_EP_DTOGRX) == 0U)
      {
        /* IN double buffered endpoint */
        USB_DRD_RX_DTOG((uint32_t)p_usb, phy_ep_num);
      }

      /* Still there is data to Fill in the next Buffer */
      if (p_ep->xfer_fill_db == 1U)
      {
        p_ep->p_xfer_buffer += tx_packet_size;
        p_ep->xfer_count += tx_packet_size;

        /* Calculate the length of the new buffer to fill */
        if (p_ep->xfer_size >= p_ep->max_packet)
        {
          length = p_ep->max_packet;
          p_ep->xfer_size -= length;
        }
        else if (p_ep->xfer_size == 0U)
        {
          length = tx_packet_size;
          p_ep->xfer_fill_db = 0U;
        }
        else
        {
          length = p_ep->xfer_size;
          p_ep->xfer_size = 0U;
          p_ep->xfer_fill_db = 0U;
        }

        /* Set the Double buffer counter for pma buffer1 */
        USB_DRD_PCD_SET_EP_DBUF1_CNT((uint32_t)p_usb, phy_ep_num, p_ep->dir, length);

        /* Copy the user buffer to USB PMA */
        USB_DRD_WritePMA((uint32_t)p_usb, p_ep->p_xfer_buffer, p_ep->pma_addr1, (uint16_t)length);
      }
    }
  }

  /* Enable endpoint IN */
  USB_DRD_PCD_SET_EP_TX_STATUS((uint32_t)p_usb, phy_ep_num, USB_EP_TX_VALID);

  return HAL_OK;
}
#endif /* defined (USE_HAL_PCD_USB_DOUBLE_BUFFER) && (USE_HAL_PCD_USB_DOUBLE_BUFFER == 1) */


/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (USE_HAL_PCD_MODULE) && (USE_HAL_PCD_MODULE == 1) */
#endif /* defined (USB_DRD_FS) */

/**
  * @}
  */
