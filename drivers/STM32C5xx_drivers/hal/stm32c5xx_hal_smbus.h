/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_smbus.h
  * @brief   Header file for the SMBUS HAL module.
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
#ifndef STM32C5XX_HAL_SMBUS_H
#define STM32C5XX_HAL_SMBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_i2c.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(I2C1) || defined(I2C2)

/** @defgroup SMBUS SMBUS
  * @{
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup SMBUS_Exported_Types HAL SMBUS Types
  * @{
  */

/**
  * @brief HAL SMBUS instance.
  */
typedef enum
{
  HAL_SMBUS1 = I2C1_BASE,                                           /*!< Peripheral instance I2C1 */
#if defined(I2C2)
  HAL_SMBUS2 = I2C2_BASE,                                           /*!< Peripheral instance I2C2 */
#endif /* I2C2 */
} hal_smbus_t;

/**
  * @brief HAL state structure definition.
  */
typedef enum
{
  HAL_SMBUS_STATE_RESET        = (0UL),        /*!< Not yet initialized                                          */
  HAL_SMBUS_STATE_INIT         = (1UL << 31U), /*!< Initialized but not yet configured                           */
  HAL_SMBUS_STATE_IDLE         = (1UL << 30U), /*!< Initialized and a global configuration applied               */
  HAL_SMBUS_STATE_TX           = (1UL << 29U), /*!< Data transmission process is ongoing                         */
  HAL_SMBUS_STATE_RX           = (1UL << 28U), /*!< Data reception process is ongoing                            */
  HAL_SMBUS_STATE_LISTEN       = (1UL << 27U), /*!< Address listen mode is ongoing                               */
  HAL_SMBUS_STATE_TX_LISTEN    = (1UL << 26U), /*!< Address listen mode and data transmission process is ongoing */
  HAL_SMBUS_STATE_RX_LISTEN    = (1UL << 25U), /*!< Address listen mode and data reception process is ongoing    */
  HAL_SMBUS_STATE_ABORT        = (1UL << 24U), /*!< Abort user request is ongoing                                */
} hal_smbus_state_t;

/**
  * @brief SMBUS Transfer Options.
  */
typedef enum
{
  /*! First Frame Transfer Option */
  HAL_SMBUS_XFER_FIRST_FRAME                   = LL_I2C_MODE_SOFTEND,
  /*! Next Frame Transfer Option */
  HAL_SMBUS_XFER_NEXT_FRAME                    = ((uint32_t)(LL_I2C_MODE_RELOAD | LL_I2C_MODE_SOFTEND)),
  /*! First and Last Frame Transfer Option without PEC */
  HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_NO_PEC   = LL_I2C_MODE_AUTOEND,
  /*! Last Frame Transfer Option without PEC */
  HAL_SMBUS_XFER_LAST_FRAME_NO_PEC             = LL_I2C_MODE_AUTOEND,
  /*! First Frame Transfer Option with PEC */
  HAL_SMBUS_XFER_FIRST_FRAME_WITH_PEC          = ((uint32_t)(LL_I2C_MODE_SOFTEND | I2C_CR2_PECBYTE)),
  /*! First and Last Frame Transfer Option with PEC */
  HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_WITH_PEC = ((uint32_t)(LL_I2C_MODE_AUTOEND | I2C_CR2_PECBYTE)),
  /*! Last Frame Transfer Option with PEC */
  HAL_SMBUS_XFER_LAST_FRAME_WITH_PEC           = ((uint32_t)(LL_I2C_MODE_AUTOEND | I2C_CR2_PECBYTE)),
  /*! Other Frame Transfer Option without PEC with restart at each frame */
  HAL_SMBUS_XFER_OTHER_FRAME_NO_PEC            = (0x000000AAU),
  /*! Other Frame Transfer Option with PEC and restart at each frame */
  HAL_SMBUS_XFER_OTHER_FRAME_WITH_PEC          = (0x0000AA00U),
  /*! Other and last Frame Transfer Option without PEC ended with stop condition */
  HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_NO_PEC   = (0x00AA0000U),
  /*! Other and last Frame Transfer Option without PEC ended with stop condition */
  HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_WITH_PEC = (0xAA000000U)
} hal_smbus_xfer_opt_t;

/**
  * @brief SMBUS slave transfer direction master point of view.
  */
typedef enum
{
  HAL_SMBUS_SLAVE_DIRECTION_TRANSMIT = LL_I2C_DIRECTION_WRITE,           /*!< Transmit */
  HAL_SMBUS_SLAVE_DIRECTION_RECEIVE  = LL_I2C_DIRECTION_READ             /*!< Receive */
} hal_smbus_slave_xfer_direction_t;
typedef struct hal_smbus_handle_s hal_smbus_handle_t;                    /*!< SMBUS handle structure type */

#if defined(USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
/**
  * @brief Pointer to an SMBUS callback function.
  */
typedef  void (*hal_smbus_cb_t)(hal_smbus_handle_t *hsmbus);

/**
  * @brief Pointer to an SMBUS slave address match callback function.
  */
typedef  void (*hal_smbus_slave_addr_cb_t)(hal_smbus_handle_t *hsmbus,
                                           hal_smbus_slave_xfer_direction_t xfer_direction,
                                           uint32_t addr_match_code);
#endif  /* USE_HAL_SMBUS_REGISTER_CALLBACKS */

/**
  * @brief  SMBUS handle Structure definition.
  */
struct hal_smbus_handle_s
{
  hal_smbus_t                     instance;               /*!< SMBUS registers base address       */
  volatile hal_smbus_state_t      global_state;           /*!< Current state */
  uint32_t                        previous_state;         /*!< Previous state */
  uint8_t                         *p_buf_rx;              /*!< Transfer buffer rx */
  const uint8_t                   *p_buf_tx;              /*!< Transfer buffer tx */
  uint32_t                        xfer_size;              /*!< Transfer size in bytes */
  volatile uint32_t               xfer_count;             /*!< Transfer counter in bytes */
  volatile hal_smbus_xfer_opt_t   xfer_opt;               /*!< Transfer options */
  hal_status_t(*xfer_isr)(hal_smbus_handle_t *hsmbus,
                          uint32_t it_flags,
                          uint32_t it_sources);           /*!< Transfer IRQ handler function pointer */
  volatile uint32_t               last_error_codes;       /*!< Errors limited to the last process.
                                                           This parameter can be a combination
                                                           of @ref SMBUS_Error_Codes */
#if defined (USE_HAL_SMBUS_USER_DATA) && (USE_HAL_SMBUS_USER_DATA == 1)
  const void                      *p_user_data;                          /*!< User Data Pointer */
#endif /* USE_HAL_SMBUS_USER_DATA */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t              semaphore;                             /*!< SMBUS OS semaphore */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
  hal_smbus_cb_t p_master_tx_cplt_cb;                                    /*!< SMBUS Master Tx Completed callback   */
  hal_smbus_cb_t p_master_rx_cplt_cb;                                    /*!< SMBUS Master Rx Completed callback   */
  hal_smbus_cb_t p_slave_tx_cplt_cb;                                     /*!< SMBUS Slave Tx Completed callback    */
  hal_smbus_cb_t p_slave_rx_cplt_cb;                                     /*!< SMBUS Slave Rx Completed callback    */
  hal_smbus_cb_t p_slave_listen_cplt_cb;                                 /*!< SMBUS Slave Listen complete callback */
  hal_smbus_slave_addr_cb_t p_slave_addr_cb;                             /*!< SMBUS Slave Address Match callback   */
  hal_smbus_cb_t p_abort_cplt_cb;                                        /*!< SMBUS Abort callback                 */
  hal_smbus_cb_t p_error_cb;                                             /*!< SMBUS Error callback                 */
#endif  /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
};

/**
  * @brief SMBUS Analog Filter Status.
  */
typedef enum
{
  HAL_SMBUS_ANALOG_FILTER_DISABLED = 0U,                                 /*!< Analog Filter is disabled */
  HAL_SMBUS_ANALOG_FILTER_ENABLED  = 1U                                  /*!< Analog Filter is enabled  */
} hal_smbus_analog_filter_status_t;

/**
  * @brief SMBUS Own Address2 Status.
  */
typedef enum
{
  HAL_SMBUS_OWN_ADDR2_DISABLED = 0U,                                     /*!< SMBUS Own Address2 is disabled */
  HAL_SMBUS_OWN_ADDR2_ENABLED  = 1U                                      /*!< SMBUS Own Address2 is enabled */
} hal_smbus_own_addr2_status_t;

/**
  * @brief SMBUS Own Address2 Masks.
  */
typedef enum
{
  HAL_SMBUS_OWN_ADDR2_NOMASK = LL_I2C_OWNADDRESS2_NOMASK,                /*!< SMBUS Own Address2 No Mask */
  HAL_SMBUS_OWN_ADDR2_MASK01 = LL_I2C_OWNADDRESS2_MASK01,                /*!< SMBUS Own Address2 Mask 01 */
  HAL_SMBUS_OWN_ADDR2_MASK02 = LL_I2C_OWNADDRESS2_MASK02,                /*!< SMBUS Own Address2 Mask 02 */
  HAL_SMBUS_OWN_ADDR2_MASK03 = LL_I2C_OWNADDRESS2_MASK03,                /*!< SMBUS Own Address2 Mask 03 */
  HAL_SMBUS_OWN_ADDR2_MASK04 = LL_I2C_OWNADDRESS2_MASK04,                /*!< SMBUS Own Address2 Mask 04 */
  HAL_SMBUS_OWN_ADDR2_MASK05 = LL_I2C_OWNADDRESS2_MASK05,                /*!< SMBUS Own Address2 Mask 05 */
  HAL_SMBUS_OWN_ADDR2_MASK06 = LL_I2C_OWNADDRESS2_MASK06,                /*!< SMBUS Own Address2 Mask 06 */
  HAL_SMBUS_OWN_ADDR2_MASK07 = LL_I2C_OWNADDRESS2_MASK07                 /*!< SMBUS Own Address2 Mask 07 */
} hal_smbus_own_addr2_mask_t;

/**
  * @brief SMBUS Slave Acknowledge General Call Status.
  */
typedef enum
{
  HAL_SMBUS_SLAVE_ACK_GENERAL_CALL_DISABLED = 0U,                     /*!< Slave Acknowledge General Call is disabled */
  HAL_SMBUS_SLAVE_ACK_GENERAL_CALL_ENABLED  = 1U                      /*!< Slave Acknowledge General Call is enabled  */
} hal_smbus_slave_ack_general_call_status_t;

/**
  * @brief SMBUS Packet Error Check Status.
  */
typedef enum
{
  HAL_SMBUS_PEC_DISABLED  = 0U,                                          /*!< SMBUS packet error check is disabled */
  HAL_SMBUS_PEC_ENABLED   = 1U                                           /*!< SMBUS packet error check is enabled  */
} hal_smbus_pec_status_t;

/**
  * @brief SMBUS AlertIT Status.
  */
typedef enum
{
  HAL_SMBUS_ALERT_DISABLED  = 0U,                                        /*!< SMBUS Alert IT is disabled */
  HAL_SMBUS_ALERT_ENABLED   = 1U                                         /*!< SMBUS Alert IT is enabled  */
} hal_smbus_alert_status_t;

/**
  * @brief SMBUS functional mode.
  */
typedef enum
{
  HAL_SMBUS_PERIPHERAL_MODE_HOST       = LL_I2C_MODE_SMBUS_HOST,         /*!< SMBUS mode host */
  HAL_SMBUS_PERIPHERAL_MODE_SLAVE      = LL_I2C_MODE_SMBUS_SLAVE,        /*!< SMBUS mode slave */
  HAL_SMBUS_PERIPHERAL_MODE_SLAVE_ARP  = LL_I2C_MODE_SMBUS_SLAVE_ARP     /*!< SMBUS mode slave ARP */
} hal_smbus_mode_t;

/**
  * @brief SMBUS global configuration structure definition.
  */
typedef struct
{
  uint32_t         timing;          /*!< SMBUS_TIMINGR register value calculated by referring to SMBUS initialization
                                     section in the Reference Manual. This timing is directly calculated by CubeMX2.
                                     Bit 24 to 27 are reserved.
                                     A calculation helper is also available in the package. See
                                     stm32_utils_i2c.c/.h */

  uint32_t         own_address1;    /*!< First device own address. The device 7-bit address value must be
                                     shifted left by 1 bit. In other words, an 8-bit value is required and the bit 0
                                     is not considered. */
  hal_smbus_mode_t device_mode;     /*!< Master, slave, or slave ARP mode */

} hal_smbus_config_t;

/**
  * @brief SMBUS Slave Wake Up Status.
  */
typedef enum
{
  HAL_SMBUS_SLAVE_WAKE_UP_DISABLED = 0U,                                 /*!< Slave Wake Up is disabled */
  HAL_SMBUS_SLAVE_WAKE_UP_ENABLED  = 1U                                  /*!< Slave Wake Up is enabled */
} hal_smbus_slave_wake_up_status_t;

/**
  * @brief SMBUS Fast Mode plus Status.
  */
typedef enum
{
  HAL_SMBUS_FAST_MODE_PLUS_DISABLED = 0U,                                /*!< Fast mode plus disabled */
  HAL_SMBUS_FAST_MODE_PLUS_ENABLED  = 1U                                 /*!< Fast mode plus enabled */
} hal_smbus_fast_mode_plus_status_t;

/**
  * @brief HAL SMBUS Timeout Status.
  */
typedef enum
{
  HAL_SMBUS_TIMEOUT_NONE  = 0U,                                          /*!< No Timeout              */
  HAL_SMBUS_TIMEOUT_A     = LL_I2C_SMBUS_TIMEOUTA,                       /*!< Timeout A Selected      */
  HAL_SMBUS_TIMEOUT_B     = LL_I2C_SMBUS_TIMEOUTB,                       /*!< Timeout B Selected      */
  HAL_SMBUS_TIMEOUT_ALL   = LL_I2C_SMBUS_ALL_TIMEOUT                     /*!< Timeout A and B Selected*/
} hal_smbus_timeout_t;

/**
  * @brief HAL SMBUS Timeout A mode.
  */
typedef enum
{
  HAL_SMBUS_TIMEOUTA_MODE_SCL_LOW      = LL_I2C_SMBUS_TIMEOUTA_MODE_SCL_LOW,     /*!< Timeout increase on SCL low     */
  HAL_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH = LL_I2C_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH /*!< Timeout increase on SCL/SDA High*/
} hal_smbus_timeout_a_mode_t;

/**
  * @brief HAL SMBUS Timeout Config.
  */
typedef struct
{
  uint32_t                       timeout_a;                              /*!< SMBUS Timeout A timeout value */
  hal_smbus_timeout_a_mode_t     timeout_a_mode;                         /*!< SMBUS Timeout A mode          */
  uint32_t                       timeout_b;                              /*!< SMBUS Timeout B timeout value */
} hal_smbus_timeout_config_t;

/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/

/** @defgroup SMBUS_Exported_Constants HAL SMBUS Constants
  * @{
  */

/** @defgroup SMBUS_Error_Codes SMBUS Error Codes
  * @{
  */
#define HAL_SMBUS_ERROR_NONE              (0UL)                          /*!< No error                 */
#define HAL_SMBUS_ERROR_BERR              (0x01UL << 0U)                 /*!< Bus error                */
#define HAL_SMBUS_ERROR_ARLO              (0x01UL << 1U)                 /*!< Arbitration lost error   */
#define HAL_SMBUS_ERROR_ACKF              (0x01UL << 2U)                 /*!< Acknowledge error        */
#define HAL_SMBUS_ERROR_OVR               (0x01UL << 3U)                 /*!< Overflow error           */
#define HAL_SMBUS_ERROR_BUSTIMEOUT        (0x01UL << 4U)                 /*!< Bus Timeout error        */
#define HAL_SMBUS_ERROR_ALERT             (0x01UL << 5U)                 /*!< Alert error              */
#define HAL_SMBUS_ERROR_PECERR            (0x01UL << 6U)                 /*!< Packet error check error */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup SMBUS_Exported_Functions HAL SMBUS Functions
  * @{
  */

/** @defgroup SMBUS_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization and de-initialization functions  ****************************/
hal_status_t HAL_SMBUS_Init(hal_smbus_handle_t *hsmbus, hal_smbus_t instance);
void HAL_SMBUS_DeInit(hal_smbus_handle_t *hsmbus);
/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group2 Configuration functions
  * @{
  */

/* Global Configuration functions ****************************************************/
hal_status_t HAL_SMBUS_SetConfig(hal_smbus_handle_t *hsmbus, const hal_smbus_config_t *p_config);
void HAL_SMBUS_GetConfig(const hal_smbus_handle_t *hsmbus, hal_smbus_config_t *p_config);
hal_status_t HAL_SMBUS_SetTiming(hal_smbus_handle_t *hsmbus, uint32_t value);
uint32_t HAL_SMBUS_GetTiming(const hal_smbus_handle_t *hsmbus);

/* Filters Configuration functions ****************************************************/
hal_status_t HAL_SMBUS_SetConfigTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_config_t *p_config);
void HAL_SMBUS_GetConfigTimeout(const hal_smbus_handle_t *hsmbus, hal_smbus_timeout_config_t *p_config);

hal_status_t HAL_SMBUS_EnableTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_t timeout);
hal_status_t HAL_SMBUS_DisableTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_t timeout);
hal_smbus_timeout_t HAL_SMBUS_IsEnabledTimeoutA(const hal_smbus_handle_t *hsmbus);
hal_smbus_timeout_t HAL_SMBUS_IsEnabledTimeoutB(const hal_smbus_handle_t *hsmbus);

hal_status_t HAL_SMBUS_EnableAnalogFilter(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_DisableAnalogFilter(hal_smbus_handle_t *hsmbus);
hal_smbus_analog_filter_status_t HAL_SMBUS_IsEnabledAnalogFilter(const hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_SetDigitalFilter(hal_smbus_handle_t *hsmbus, uint32_t noise_filtering_in_bus_clk_period);
uint32_t HAL_SMBUS_GetDigitalFilter(const hal_smbus_handle_t *hsmbus);

/* Wakeup from Stop mode(s) */
hal_status_t HAL_SMBUS_SLAVE_EnableWakeUp(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_SLAVE_DisableWakeUp(hal_smbus_handle_t *hsmbus);
hal_smbus_slave_wake_up_status_t HAL_SMBUS_SLAVE_IsEnabledWakeUp(const hal_smbus_handle_t *hsmbus);

/* Fast mode plus driving capability */
hal_status_t HAL_SMBUS_EnableFastModePlus(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_DisableFastModePlus(hal_smbus_handle_t *hsmbus);
hal_smbus_fast_mode_plus_status_t HAL_SMBUS_IsEnabledFastModePlus(const hal_smbus_handle_t *hsmbus);


/* Acknowledge General Call */
hal_status_t HAL_SMBUS_SLAVE_EnableAckGeneralCall(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_SLAVE_DisableAckGeneralCall(hal_smbus_handle_t *hsmbus);
hal_smbus_slave_ack_general_call_status_t HAL_SMBUS_SLAVE_IsEnabledAckGeneralCall(const hal_smbus_handle_t *hsmbus);

/* Packet Error Check */
hal_status_t HAL_SMBUS_EnablePacketErrorCheck(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_DisablePacketErrorCheck(hal_smbus_handle_t *hsmbus);
hal_smbus_pec_status_t HAL_SMBUS_IsEnabledPacketErrorCheck(const hal_smbus_handle_t *hsmbus);

/* Master alert interrupt */
hal_status_t HAL_SMBUS_MASTER_EnableAlertIT(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_MASTER_DisableAlertIT(hal_smbus_handle_t *hsmbus);
hal_smbus_alert_status_t HAL_SMBUS_MASTER_IsEnabledAlertIT(const hal_smbus_handle_t *hsmbus);

/* Own Address 2 */
hal_status_t HAL_SMBUS_SetConfigOwnAddress2(hal_smbus_handle_t *hsmbus, uint32_t addr, hal_smbus_own_addr2_mask_t mask);
void HAL_SMBUS_GetConfigOwnAddress2(const hal_smbus_handle_t *hsmbus, uint32_t *p_addr,
                                    hal_smbus_own_addr2_mask_t *p_mask);
hal_status_t HAL_SMBUS_EnableOwnAddress2(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_DisableOwnAddress2(hal_smbus_handle_t *hsmbus);
hal_smbus_own_addr2_status_t HAL_SMBUS_IsEnabledOwnAddress2(const hal_smbus_handle_t *hsmbus);

/* Callbacks Register/UnRegister functions  ***********************************/
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
hal_status_t HAL_SMBUS_MASTER_RegisterTxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_MASTER_RegisterRxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_SLAVE_RegisterTxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_SLAVE_RegisterRxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_SLAVE_RegisterListenCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_SLAVE_RegisterAddrMatchCallback(hal_smbus_handle_t *hsmbus,              \
                                                       hal_smbus_slave_addr_cb_t p_callback);
hal_status_t HAL_SMBUS_RegisterAbortCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
hal_status_t HAL_SMBUS_RegisterErrorCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */

hal_status_t HAL_SMBUS_SetMode(hal_smbus_handle_t *hsmbus, const hal_smbus_mode_t mode);
hal_smbus_mode_t HAL_SMBUS_GetMode(const hal_smbus_handle_t *hsmbus);
/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group3 Input and Output operation functions
  * @{
  */

/******* Blocking mode: Polling */
hal_status_t HAL_SMBUS_MASTER_PollForSlaveReady(hal_smbus_handle_t *hsmbus, uint32_t device_addr, uint32_t timeout_ms);

/******* Non-Blocking mode: Interrupt */
hal_status_t HAL_SMBUS_MASTER_SEQ_Transmit_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr, const void *p_data,
                                              uint32_t size_byte, hal_smbus_xfer_opt_t xfer_opt);
hal_status_t HAL_SMBUS_MASTER_SEQ_Receive_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr, void *p_data,
                                             uint32_t size_byte, hal_smbus_xfer_opt_t xfer_opt);
hal_status_t HAL_SMBUS_MASTER_Abort_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr);
hal_status_t HAL_SMBUS_SLAVE_Abort_IT(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_SLAVE_SEQ_Transmit_IT(hal_smbus_handle_t *hsmbus, const void *p_data, uint32_t size_byte,
                                             hal_smbus_xfer_opt_t xfer_opt);
hal_status_t HAL_SMBUS_SLAVE_SEQ_Receive_IT(hal_smbus_handle_t *hsmbus, void *p_data, uint32_t size_byte,
                                            hal_smbus_xfer_opt_t xfer_opt);

/* Slave listen interrupt */
hal_status_t HAL_SMBUS_SLAVE_EnableListen_IT(hal_smbus_handle_t *hsmbus);
hal_status_t HAL_SMBUS_SLAVE_DisableListen_IT(hal_smbus_handle_t *hsmbus);

/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group4 IRQ Handlers
  * @{
  */
/******* SMBUS IRQHandler and Callbacks used in non blocking modes (Interrupt) */
void HAL_SMBUS_EV_IRQHandler(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_ERR_IRQHandler(hal_smbus_handle_t *hsmbus);

/**
  * @}
  */
/** @defgroup SMBUS_Exported_Functions_Group5 Weak Callback Functions
  * @{
  */
void HAL_SMBUS_MASTER_TxCpltCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_MASTER_RxCpltCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_SLAVE_TxCpltCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_SLAVE_RxCpltCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_SLAVE_AddrCallback(hal_smbus_handle_t *hsmbus, hal_smbus_slave_xfer_direction_t xfer_direction,
                                  uint32_t addr_match_code);
void HAL_SMBUS_SLAVE_ListenCpltCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_ErrorCallback(hal_smbus_handle_t *hsmbus);
void HAL_SMBUS_AbortCpltCallback(hal_smbus_handle_t *hsmbus);

/**
  * @}
  */
/** @defgroup SMBUS_Exported_Functions_Group6 Peripheral State, Peripheral Clock Frequency, Mode and Errors functions
  *  @{
  */
/* Peripheral State and Errors functions  **************************************************/
hal_smbus_state_t HAL_SMBUS_GetState(const hal_smbus_handle_t *hsmbus);
#if defined (USE_HAL_SMBUS_GET_LAST_ERRORS) && (USE_HAL_SMBUS_GET_LAST_ERRORS == 1)
uint32_t HAL_SMBUS_GetLastErrorCodes(const hal_smbus_handle_t *hsmbus);
#endif /* USE_HAL_SMBUS_GET_LAST_ERRORS */
uint32_t HAL_SMBUS_GetClockFreq(const hal_smbus_handle_t *hsmbus);
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup SMBUS_Exported_Functions_Group7 Acquire/Release/Free the bus
  * @{
  */
hal_status_t HAL_SMBUS_AcquireBus(hal_smbus_handle_t *hsmbus, uint32_t timeout_ms);
hal_status_t HAL_SMBUS_ReleaseBus(hal_smbus_handle_t *hsmbus);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_SMBUS_USER_DATA) && (USE_HAL_SMBUS_USER_DATA == 1)
/** @defgroup SMBUS_Exported_Functions_Group8 Set and get user data functions
  * @{
  */
void HAL_SMBUS_SetUserData(hal_smbus_handle_t *hsmbus, const void *p_user_data);
const void *HAL_SMBUS_GetUserData(const hal_smbus_handle_t *hsmbus);
/**
  * @}
  */
#endif /* USE_HAL_SMBUS_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

#endif /* I2C1 || I2C2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_SMBUS_H */
