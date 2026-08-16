/**
  ******************************************************************************
  * @file    stm32c5xx_hal_i2c.h
  * @brief   Header file for the I2C HAL module.
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
#ifndef STM32C5XX_HAL_I2C_H
#define STM32C5XX_HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_i2c.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(I2C1) || defined(I2C2)

/** @defgroup I2C I2C
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2C_Exported_Types HAL I2C Types
  * @{
  */

/**
  * @brief I2C Sequential Transfer Options.
  */
typedef enum
{
  HAL_I2C_XFER_FIRST_FRAME          = ((uint32_t) LL_I2C_MODE_SOFTEND),  /*!< First frame */
  HAL_I2C_XFER_FIRST_AND_NEXT_FRAME = ((uint32_t)(LL_I2C_MODE_RELOAD | \
                                                  LL_I2C_MODE_SOFTEND)), /*!< First and next frame */
  HAL_I2C_XFER_NEXT_FRAME           = ((uint32_t)(LL_I2C_MODE_RELOAD | \
                                                  LL_I2C_MODE_SOFTEND)), /*!< Next frame */
  HAL_I2C_XFER_FIRST_AND_LAST_FRAME = ((uint32_t) LL_I2C_MODE_AUTOEND),  /*!< First and last frame */
  HAL_I2C_XFER_LAST_FRAME           = ((uint32_t) LL_I2C_MODE_AUTOEND),  /*!< Last frame */
  HAL_I2C_XFER_LAST_FRAME_NO_STOP   = ((uint32_t) LL_I2C_MODE_SOFTEND),  /*!< Frame with no stop */
  HAL_I2C_XFER_OTHER_FRAME          = (0x000000AAU), /*!< Other frame with restart at each frame */
  HAL_I2C_XFER_OTHER_AND_LAST_FRAME = (0x0000AA00U), /*!< Other and last frame terminated with a stop condition */
} hal_i2c_xfer_opt_t;

/**
  * @brief I2C Addressing Mode.
  */
typedef enum
{
  HAL_I2C_ADDRESSING_7BIT  = LL_I2C_ADDRESSING_MODE_7BIT, /*!< 7-bit addressing */
  HAL_I2C_ADDRESSING_10BIT = LL_I2C_ADDRESSING_MODE_10BIT /*!< 10-bit addressing */
} hal_i2c_addressing_mode_t;

/**
  * @brief I2C Slave Stretch Mode Status.
  */
typedef enum
{
  HAL_I2C_SLAVE_STRETCH_DISABLED = 0U, /*!< Slave Stretch Mode is disabled */
  HAL_I2C_SLAVE_STRETCH_ENABLED  = 1U  /*!< Slave Stretch Mode is enabled */
} hal_i2c_slave_stretch_mode_status_t;

/**
  * @brief I2C Slave Acknowledge General Call Status.
  */
typedef enum
{
  HAL_I2C_SLAVE_ACK_GENERAL_CALL_DISABLED = 0U, /*!< Slave Acknowledge General Call is disabled */
  HAL_I2C_SLAVE_ACK_GENERAL_CALL_ENABLED  = 1U  /*!< Slave Acknowledge General Call is enabled */
} hal_i2c_slave_ack_general_call_status_t;

/**
  * @brief I2C Own Address2 Masks.
  */
typedef enum
{
  HAL_I2C_OWN_ADDR2_NOMASK = LL_I2C_OWNADDRESS2_NOMASK, /*!< No mask */
  HAL_I2C_OWN_ADDR2_MASK01 = LL_I2C_OWNADDRESS2_MASK01, /*!< Mask 01 */
  HAL_I2C_OWN_ADDR2_MASK02 = LL_I2C_OWNADDRESS2_MASK02, /*!< Mask 02 */
  HAL_I2C_OWN_ADDR2_MASK03 = LL_I2C_OWNADDRESS2_MASK03, /*!< Mask 03 */
  HAL_I2C_OWN_ADDR2_MASK04 = LL_I2C_OWNADDRESS2_MASK04, /*!< Mask 04 */
  HAL_I2C_OWN_ADDR2_MASK05 = LL_I2C_OWNADDRESS2_MASK05, /*!< Mask 05 */
  HAL_I2C_OWN_ADDR2_MASK06 = LL_I2C_OWNADDRESS2_MASK06, /*!< Mask 06 */
  HAL_I2C_OWN_ADDR2_MASK07 = LL_I2C_OWNADDRESS2_MASK07  /*!< Mask 07 */
} hal_i2c_own_addr2_mask_t;

/**
  * @brief I2C Own Address2 Status.
  */
typedef enum
{
  HAL_I2C_OWN_ADDR2_DISABLED = 0U, /*!< I2C Own Address2 is disabled */
  HAL_I2C_OWN_ADDR2_ENABLED  = 1U  /*!< I2C Own Address2 is enabled */
} hal_i2c_own_addr2_status_t;

/**
  * @brief I2C Memory Address size.
  */
typedef enum
{
  HAL_I2C_MEM_ADDR_8BIT  = 1U, /*!< 8-bit memory */
  HAL_I2C_MEM_ADDR_16BIT = 2U  /*!< 16-bit memory */
} hal_i2c_mem_addr_size_t;

/**
  * @brief I2C Slave Transfer Direction Master Point of View.
  */
typedef enum
{
  HAL_I2C_SLAVE_DIRECTION_TRANSMIT = LL_I2C_DIRECTION_WRITE, /*!< Transmit */
  HAL_I2C_SLAVE_DIRECTION_RECEIVE  = LL_I2C_DIRECTION_READ   /*!< Receive */
} hal_i2c_slave_xfer_direction_t;

/**
  * @brief I2C Analog Filter Status.
  */
typedef enum
{
  HAL_I2C_ANALOG_FILTER_DISABLED = 0U, /*!< Analog Filter is disabled */
  HAL_I2C_ANALOG_FILTER_ENABLED  = 1U  /*!< Analog Filter is enabled */
} hal_i2c_analog_filter_status_t;

/**
  * @brief I2C Slave Wake Up Status.
  */
typedef enum
{
  HAL_I2C_SLAVE_WAKE_UP_DISABLED = 0U, /*!< Slave Wake Up is disabled */
  HAL_I2C_SLAVE_WAKE_UP_ENABLED  = 1U  /*!< Slave Wake Up is enabled */
} hal_i2c_slave_wake_up_status_t;

/**
  * @brief I2C Fast Mode Plus Status.
  */
typedef enum
{
  HAL_I2C_FAST_MODE_PLUS_DISABLED = 0U, /*!< Fast mode plus disabled */
  HAL_I2C_FAST_MODE_PLUS_ENABLED  = 1U  /*!< Fast mode plus enabled */
} hal_i2c_fast_mode_plus_status_t;

/**
  * @brief HAL State structure definition.
  */
typedef enum
{
  HAL_I2C_STATE_RESET        = (0UL),       /*!< Not yet initialized                                          */
  HAL_I2C_STATE_INIT         = (1UL << 31), /*!< Initialized but not yet configured                           */
  HAL_I2C_STATE_IDLE         = (1UL << 30), /*!< Initialized and a global configuration applied               */
  HAL_I2C_STATE_TX           = (1UL << 29), /*!< Data transmission process is ongoing                         */
  HAL_I2C_STATE_RX           = (1UL << 28), /*!< Data reception process is ongoing                            */
  HAL_I2C_STATE_LISTEN       = (1UL << 27), /*!< Address listen mode is ongoing                               */
  HAL_I2C_STATE_TX_LISTEN    = (1UL << 26), /*!< Address listen mode and data transmission process is ongoing */
  HAL_I2C_STATE_RX_LISTEN    = (1UL << 25), /*!< Address listen mode and data reception process is ongoing    */
  HAL_I2C_STATE_ABORT        = (1UL << 24), /*!< Abort user request is ongoing                                */
} hal_i2c_state_t;

/**
  * @brief  I2C global configuration structure definition.
  */
typedef struct
{
  uint32_t timing;          /*!< I2C_TIMINGR register value calculated by referring to I2C initialization section
                                 in the Reference Manual. This timing is directly calculated by CubeMX2.
                                 Bit 24 to 27 are reserved.
                                 A calculation helper is also available in the package. See
                                 stm32_utils_i2c.c/.h */

  uint32_t own_address1;    /*!< First device own address. It can be a 7-bit or a 10-bit address.
                                 If the 7-bit addressing mode is selected, the device 7-bit address value must be
                                 shifted left by 1 bit. In other words, an 8-bit value is required and the bit 0
                                 is not considered. */

  hal_i2c_addressing_mode_t addressing_mode; /*!< 7-bit or 10-bit addressing mode */
} hal_i2c_config_t;

/**
  * @brief HAL functional mode.
  */
typedef enum
{
  HAL_I2C_MODE_NONE       = 0U, /*!< No I2C communication ongoing        */
  HAL_I2C_MODE_MASTER     = 1U, /*!< I2C communication is in master mode */
  HAL_I2C_MODE_SLAVE      = 2U, /*!< I2C communication is in slave mode  */
  HAL_I2C_MODE_MASTER_MEM = 3U  /*!< I2C communication is in memory mode */
} hal_i2c_mode_t;

/**
  * @brief HAL I2C instance.
  */
typedef enum
{
  HAL_I2C1 = I2C1_BASE, /*!< Peripheral instance I2C1 */
#if defined(I2C2)
  HAL_I2C2 = I2C2_BASE, /*!< Peripheral instance I2C2 */
#endif /* I2C2 */
} hal_i2c_t;

typedef struct hal_i2c_handle_s hal_i2c_handle_t; /*!< I2C handle structure type */

#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/**
  * @brief pointer to an I2C callback function.
  */
typedef  void (*hal_i2c_cb_t)(hal_i2c_handle_t *hi2c);

/**
  * @brief pointer to an I2C slave address match callback function.
  */
typedef  void (*hal_i2c_slave_addr_cb_t)(hal_i2c_handle_t *hi2c,
                                         hal_i2c_slave_xfer_direction_t xfer_direction,
                                         uint32_t addr_match_code);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief I2C handle structure definition.
  */
struct hal_i2c_handle_s
{
  hal_i2c_t                   instance;         /*!< Peripheral instance */
  volatile hal_i2c_state_t    global_state;     /*!< Current state */
  uint32_t                    previous_state;   /*!< Previous state and mode */
  uint8_t                     *p_buf_rx;        /*!< Transfer buffer rx */
  const uint8_t               *p_buf_tx;        /*!< Transfer buffer tx */
  uint32_t                    xfer_size;        /*!< Transfer size in bytes */
  volatile uint32_t           xfer_count;       /*!< Transfer counter in bytes */
  volatile hal_i2c_xfer_opt_t xfer_opt;         /*!< Sequential transfer options */
  hal_status_t(*xfer_isr)(hal_i2c_handle_t *hi2c,
                          uint32_t it_flags,
                          uint32_t it_sources); /*!< Transfer IRQ handler function pointer */
  volatile hal_i2c_mode_t     mode;             /*!< Communication mode */
  volatile uint32_t           last_error_codes; /*!< Errors limited to the last process
                                                     This parameter can be a combination of @ref I2C_Error_Codes */
  volatile uint32_t           addr_event_count; /*!< Address Event counter */
  volatile uint32_t           dev_addr;         /*!< Target device address */
  volatile uint32_t           mem_addr;         /*!< Target memory address */
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  hal_dma_handle_t            *hdma_tx;         /*!< Tx DMA handle */
  hal_dma_handle_t            *hdma_rx;         /*!< Rx DMA handle */
#endif /* USE_HAL_I2C_DMA */
#if defined (USE_HAL_I2C_USER_DATA) && (USE_HAL_I2C_USER_DATA == 1)
  const void                  *p_user_data;     /*!< User data pointer */
#endif /* USE_HAL_I2C_USER_DATA */
#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  hal_os_semaphore_t          semaphore;        /*!< I2C OS semaphore */
#endif /* USE_HAL_MUTEX */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
  hal_i2c_cb_t p_master_tx_cplt_cb;        /*!< I2C Master Tx transfer completed callback */
  hal_i2c_cb_t p_master_rx_cplt_cb;        /*!< I2C Master Rx transfer completed callback */
  hal_i2c_cb_t p_slave_tx_cplt_cb;         /*!< I2C Slave Tx transfer completed callback  */
  hal_i2c_cb_t p_slave_rx_cplt_cb;         /*!< I2C Slave Rx transfer completed callback  */
  hal_i2c_cb_t p_slave_listen_cplt_cb;     /*!< I2C Slave Listen complete callback        */
  hal_i2c_slave_addr_cb_t p_slave_addr_cb; /*!< I2C Slave Address Match callback          */
  hal_i2c_cb_t p_mem_tx_cplt_cb;           /*!< I2C Memory Tx transfer completed callback */
  hal_i2c_cb_t p_mem_rx_cplt_cb;           /*!< I2C Memory Rx transfer completed callback */
  hal_i2c_cb_t p_abort_cplt_cb;            /*!< I2C Abort completed callback              */
  hal_i2c_cb_t p_error_cb;                 /*!< I2C Error callback                        */
#endif  /* USE_HAL_I2C_REGISTER_CALLBACKS */
};

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_Exported_Constants HAL I2C Constants
  * @{
  */

/** @defgroup I2C_Error_Codes I2C Error Codes
  * @{
  */
#define  HAL_I2C_ERROR_NONE  (0UL)      /*!< No error                      */
#define  HAL_I2C_ERROR_BERR  (1UL << 0) /*!< Bus error                     */
#define  HAL_I2C_ERROR_ARLO  (1UL << 1) /*!< Arbitration lost              */
#define  HAL_I2C_ERROR_AF    (1UL << 2) /*!< Acknowledge not received      */
#define  HAL_I2C_ERROR_OVR   (1UL << 3) /*!< Overrun/Underrun (slave mode) */
#define  HAL_I2C_ERROR_SIZE  (1UL << 4) /*!< Size management error         */
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
#define  HAL_I2C_ERROR_DMA   (1UL << 5) /*!< DMA transfer error            */
#endif /* USE_HAL_I2C_DMA */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2C_Exported_Functions HAL I2C Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
hal_status_t HAL_I2C_Init(hal_i2c_handle_t *hi2c, hal_i2c_t instance);
void HAL_I2C_DeInit(hal_i2c_handle_t *hi2c);
/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2 Configuration functions
  * @{
  */
/* Global configuration */
hal_status_t HAL_I2C_SetConfig(hal_i2c_handle_t *hi2c, const hal_i2c_config_t *p_config);
void HAL_I2C_GetConfig(const hal_i2c_handle_t *hi2c, hal_i2c_config_t *p_config);
hal_status_t HAL_I2C_SetTiming(hal_i2c_handle_t *hi2c, uint32_t value);
uint32_t HAL_I2C_GetTiming(const hal_i2c_handle_t *hi2c);

/* Filter */
hal_status_t HAL_I2C_EnableAnalogFilter(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_DisableAnalogFilter(hal_i2c_handle_t *hi2c);
hal_i2c_analog_filter_status_t HAL_I2C_IsEnabledAnalogFilter(const hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SetDigitalFilter(hal_i2c_handle_t *hi2c, uint32_t noise_filtering_in_bus_clk_period);
uint32_t HAL_I2C_GetDigitalFilter(const hal_i2c_handle_t *hi2c);

/* Wakeup from Stop mode(s) */
hal_status_t HAL_I2C_SLAVE_EnableWakeUp(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SLAVE_DisableWakeUp(hal_i2c_handle_t *hi2c);
hal_i2c_slave_wake_up_status_t HAL_I2C_SLAVE_IsEnabledWakeUp(const hal_i2c_handle_t *hi2c);

/* Fast mode plus driving capability */
hal_status_t HAL_I2C_EnableFastModePlus(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_DisableFastModePlus(hal_i2c_handle_t *hi2c);
hal_i2c_fast_mode_plus_status_t HAL_I2C_IsEnabledFastModePlus(const hal_i2c_handle_t *hi2c);

/* Clock stretching */
hal_status_t HAL_I2C_SLAVE_EnableClockStretching(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SLAVE_DisableClockStretching(hal_i2c_handle_t *hi2c);
hal_i2c_slave_stretch_mode_status_t HAL_I2C_SLAVE_IsEnabledClockStretching(const hal_i2c_handle_t *hi2c);

/* Acknowledge General Call */
hal_status_t HAL_I2C_SLAVE_EnableAckGeneralCall(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SLAVE_DisableAckGeneralCall(hal_i2c_handle_t *hi2c);
hal_i2c_slave_ack_general_call_status_t HAL_I2C_SLAVE_IsEnabledAckGeneralCall(const hal_i2c_handle_t *hi2c);

/* Own Address 2 */
hal_status_t HAL_I2C_SetConfigOwnAddress2(hal_i2c_handle_t *hi2c, uint32_t addr, hal_i2c_own_addr2_mask_t mask);
void HAL_I2C_GetConfigOwnAddress2(const hal_i2c_handle_t *hi2c, uint32_t *addr, hal_i2c_own_addr2_mask_t *mask);
hal_status_t HAL_I2C_EnableOwnAddress2(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_DisableOwnAddress2(hal_i2c_handle_t *hi2c);
hal_i2c_own_addr2_status_t HAL_I2C_IsEnabledOwnAddress2(const hal_i2c_handle_t *hi2c);

#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)
/* Register callbacks */
hal_status_t HAL_I2C_MASTER_RegisterTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_MASTER_RegisterRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_MASTER_RegisterMemTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_MASTER_RegisterMemRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_SLAVE_RegisterTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_SLAVE_RegisterRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_SLAVE_RegisterListenCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_SLAVE_RegisterAddrMatchCallback(hal_i2c_handle_t *hi2c, hal_i2c_slave_addr_cb_t p_callback);
hal_status_t HAL_I2C_RegisterAbortCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
hal_status_t HAL_I2C_RegisterErrorCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
hal_status_t HAL_I2C_SetTxDMA(hal_i2c_handle_t *hi2c, hal_dma_handle_t *hdma);
hal_status_t HAL_I2C_SetRxDMA(hal_i2c_handle_t *hi2c, hal_dma_handle_t *hdma);
#endif /* USE_HAL_I2C_DMA */
/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group3 Input and Output operation functions
  * @{
  */
/* Blocking mode: Polling */
hal_status_t HAL_I2C_MASTER_Transmit(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                     uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_I2C_MASTER_Receive(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                    uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_I2C_MASTER_MemWrite(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                     hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                     uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_I2C_MASTER_MemRead(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                    hal_i2c_mem_addr_size_t memory_addr_size, void *p_data, uint32_t size_byte,
                                    uint32_t timeout_ms);
hal_status_t HAL_I2C_MASTER_PollForSlaveReady(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t timeout_ms);
hal_status_t HAL_I2C_SLAVE_Transmit(hal_i2c_handle_t *hi2c, const void *p_data,
                                    uint32_t size_byte, uint32_t timeout_ms);
hal_status_t HAL_I2C_SLAVE_Receive(hal_i2c_handle_t *hi2c, void *p_data,
                                   uint32_t size_byte, uint32_t timeout_ms);
/* Non-Blocking mode: Interrupt */
hal_status_t HAL_I2C_MASTER_Transmit_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                        uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_Receive_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                       uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_MemWrite_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                        hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                        uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_MemRead_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                       hal_i2c_mem_addr_size_t memory_addr_size, void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_SEQ_Transmit_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                            uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_MASTER_SEQ_Receive_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                           uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_MASTER_Abort_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr);
hal_status_t HAL_I2C_SLAVE_Transmit_IT(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_SLAVE_Receive_IT(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_SLAVE_SEQ_Transmit_IT(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte,
                                           hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_SLAVE_SEQ_Receive_IT(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte,
                                          hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_SLAVE_EnableListen_IT(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SLAVE_DisableListen_IT(hal_i2c_handle_t *hi2c);
hal_status_t HAL_I2C_SLAVE_Abort_IT(hal_i2c_handle_t *hi2c);

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/* Non-Blocking mode: DMA */
hal_status_t HAL_I2C_MASTER_Transmit_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                         uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_Receive_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                        uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_MemWrite_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                         hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                         uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_MemRead_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                        hal_i2c_mem_addr_size_t memory_addr_size, void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_MASTER_SEQ_Transmit_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                             uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_MASTER_SEQ_Receive_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                            uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_SLAVE_Transmit_DMA(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_SLAVE_Receive_DMA(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte);
hal_status_t HAL_I2C_SLAVE_SEQ_Transmit_DMA(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte,
                                            hal_i2c_xfer_opt_t xfer_opt);
hal_status_t HAL_I2C_SLAVE_SEQ_Receive_DMA(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte,
                                           hal_i2c_xfer_opt_t xfer_opt);
#endif /* USE_HAL_I2C_DMA */
/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group4 IRQ Handlers
  * @{
  */
void HAL_I2C_EV_IRQHandler(hal_i2c_handle_t *hi2c);
void HAL_I2C_ERR_IRQHandler(hal_i2c_handle_t *hi2c);
/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group5 Weak Callback Functions
  * @{
  */
void HAL_I2C_MASTER_TxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_MASTER_RxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_MASTER_MemTxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_MASTER_MemRxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_SLAVE_TxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_SLAVE_RxCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_SLAVE_AddrCallback(hal_i2c_handle_t *hi2c, hal_i2c_slave_xfer_direction_t xfer_direction,
                                uint32_t addr_match_code);
void HAL_I2C_SLAVE_ListenCpltCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_ErrorCallback(hal_i2c_handle_t *hi2c);
void HAL_I2C_AbortCpltCallback(hal_i2c_handle_t *hi2c);
/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group6 Peripheral State, Mode and Error functions, Kernel Clock Frequency
  * @{
  */
hal_i2c_state_t HAL_I2C_GetState(const hal_i2c_handle_t *hi2c);
hal_i2c_mode_t HAL_I2C_GetMode(const hal_i2c_handle_t *hi2c);
#if defined (USE_HAL_I2C_GET_LAST_ERRORS) && (USE_HAL_I2C_GET_LAST_ERRORS == 1)
uint32_t HAL_I2C_GetLastErrorCodes(const hal_i2c_handle_t *hi2c);
#endif /* USE_HAL_I2C_GET_LAST_ERRORS */
uint32_t HAL_I2C_GetClockFreq(const hal_i2c_handle_t *hi2c);
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup I2C_Exported_Functions_Group7 Acquire/Release/Free the bus
  * @{
  */
hal_status_t HAL_I2C_AcquireBus(hal_i2c_handle_t *hi2c, uint32_t timeout_ms);
hal_status_t HAL_I2C_ReleaseBus(hal_i2c_handle_t *hi2c);
/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_I2C_USER_DATA) && (USE_HAL_I2C_USER_DATA == 1)
/** @defgroup I2C_Exported_Functions_Group8 Set/Get user data
  * @{
  */
void HAL_I2C_SetUserData(hal_i2c_handle_t *hi2c, const void *p_user_data);
const void *HAL_I2C_GetUserData(const hal_i2c_handle_t *hi2c);
/**
  * @}
  */
#endif /* USE_HAL_I2C_USER_DATA */

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

#endif /* STM32C5XX_HAL_I2C_H */
