/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_i3c.c
  * @brief   I3C HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Improvement Inter Integrated Circuit (I3C) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral state and errors functions
  *
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
  *********************************************************************************************************************/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @addtogroup I3C I3C
  * @{
  */
/** @defgroup I3C_Introduction I3C Introduction
  * @{
The STM32 Hardware Abstraction Layer for I3C provides a comprehensive software interface to support the I3C
communication protocol on STM32 microcontrollers. I3C (Improved Inter-Integrated Circuit) is an advanced, high-speed,
and energy-efficient serial bus protocol designed to enhance and replace traditional I2C, offering features such as
dynamic device management and higher data rates. While I3C maintains backward compatibility with many I2C devices,
certain I2C features such as stretch mode are not supported.
This HAL driver abstracts low-level hardware details, enabling configuration, data transfer, and interrupt handling,
thereby accelerating development and ensuring robust, scalable embedded applications that leverage the full
capabilities of the I3C bus.
  */
/**
  * @}
  */

/** @defgroup I3C_How_To_Use I3C How To Use
  * @{

# How to use the I3C HAL

Use the I3C HAL driver as follows:

## 1. Declare
  - A @ref hal_i3c_handle_t handle structure,
    for example: hal_i3c_handle_t hi3c;
  - A @ref hal_i3c_transfer_ctx_t transfer descriptor structure (controller only),
    for example: hal_i3c_transfer_ctx_t my_transfer_ctx;

## 2. Initialize
  - Initialize the I3Cx driver with an I3C HW instance by calling HAL_I3C_Init().
  - The I3Cx clock is enabled inside HAL_I3C_Init() if USE_HAL_I3C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.

## 3. Configure low-level hardware
  - Enable the I3Cx clock if USE_HAL_I3C_CLK_ENABLE_MODEL = HAL_CLK_ENABLE_NO.
  - I3Cx pin configuration:
    - Enable the clock for the I3Cx GPIOs.
    - Configure I3C pins as alternate function push-pull with no-pull.
  - NVIC configuration for interrupt processing:
    - Configure the I3Cx interrupt priority.
    - Enable the NVIC I3C IRQ Channel.
  - DMA configuration for DMA processing:
    - Declare a hal_dma_handle_t handle structure for the Control Register (CR) management channel.
    - Declare a hal_dma_handle_t handle structure for the transmit channel.
    - Declare a hal_dma_handle_t handle structure for the receive channel.
    - Enable the DMAx interface clock.
    - Configure the DMA handle parameters.
    - Configure the DMA Control Register (CR) channel.
    - Configure the DMA Tx channel.
    - Configure the DMA Rx channel.
    - Associate the initialized DMA handle with the hi3c DMA CR, Tx, or Rx as necessary.
    - Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA CR, Tx or Rx instance.

## 4. Configure communication mode
  - Controller mode: HAL_I3C_CTRL_SetConfig()
  - Target mode: HAL_I3C_TGT_SetConfig()

## 5. Configure FIFOs (optional)
  - Controller: HAL_I3C_CTRL_SetConfigFifo()
  - Target: HAL_I3C_TGT_SetConfigFifo()
  - At the end of a transfer, all FIFOs can be flushed: HAL_I3C_FlushAllFifos(), or individually:
    HAL_I3C_FlushTxFifo(), HAL_I3C_FlushRxFifo(), HAL_I3C_CTRL_FlushControlFifo(), HAL_I3C_FlushstatusFifo().

## 6. Target advanced features

  - HAL_I3C_TGT_SetConfigGETMXDS()
  - HAL_I3C_TGT_EnableGroupAddrCapability()
  - HAL_I3C_TGT_SetConfigMaxDataSize()
  - HAL_I3C_TGT_SetConfigIBI()
  - HAL_I3C_TGT_EnableIBI()
  - HAL_I3C_TGT_EnableHotJoinRequest()
  - HAL_I3C_TGT_EnableCtrlRoleRequest()

### 6.1 ENTDAA payload configuration
  Before initiating any ENTDAA, the target must configure the target payload of the ENTDAA by using
  HAL_I3C_TGT_SetPayloadENTDAAConfig().

### 6.2 Target Hot-Join (interrupt mode)
  Request a Hot-Join in target mode: HAL_I3C_TGT_HotJoinReq_IT().
  At completion, HAL_I3C_TGT_HotJoinCallback() is executed, and user code can add custom handling by overriding this
  weak callback function or by registering a callback function.

### 6.3 Target In-Band Interrupt (interrupt mode)
  Request an In-Band Interrupt in target mode: HAL_I3C_TGT_IBIReq_IT()
  At completion, HAL_I3C_NotifyCallback() is executed, and user code can add custom handling by overriding this weak
  callback function or by registering a callback function.

### 6.4 Target Controller-Role request (interrupt mode)
  Request a Controller Role in target mode: HAL_I3C_TGT_ControlRoleReq_IT()
  At completion, HAL_I3C_NotifyCallback() is executed, and user code can add custom handling by overriding this weak
  callback function or by registering a callback function.

### 6.5 Target Wakeup capability
  To manage the wakeup capability, use HAL_I3C_TGT_ActivateNotification() or HAL_I3C_TGT_DeactivateNotification() to
  enable or disable the wakeup interrupt. At wakeup detection, the associated HAL_I3C_NotifyCallback() is executed.

## 7. Controller advanced features
  - HAL_I3C_CTRL_SetConfigOwnDynamicAddress()
  - HAL_I3C_CTRL_EnableHotJoinAllowed()
  - HAL_I3C_CTRL_EnableHighKeeperSDA()
  - HAL_I3C_CTRL_SetConfigStallTime()
  - HAL_I3C_CTRL_SetConfigBusDevices()
  - HAL_I3C_CTRL_EnableResetPattern()

### 7.1 Dynamic Address Assignment procedure
  - Before initiating any IO operation, launch an assignment of the different target dynamic addresses by using
    HAL_I3C_CTRL_DynAddrAssign() in polling mode or HAL_I3C_CTRL_DynAddrAssign_IT() in interrupt mode. This procedure
    is named Enter Dynamic Address Assignment (ENTDAA CCC command).
  - For the initiation of ENTDAA procedure, each target connected and powered on the I3C bus must respond to this
    particular Command Common Code (CCC) by sending its proper Payload (an amount of 48-bits which contain the target
    characteristics). Each time a target responds to ENTDAA sequence, the controller application is informed through
    HAL_I3C_CTRL_TgtReqDynAddrCallback() of the reception of the target payload.
  - Then send an associated dynamic address through HAL_I3C_CTRL_SetDynAddr().
    This procedure loops automatically in hardware side until a target responds to repeated ENTDAA sequence.
  - The controller application is informed of the end of the procedure at reception of HAL_I3C_CTRL_DAACpltCallback().
  - Then retrieve ENTDAA payload information through HAL_I3C_CTRL_Get_ENTDAA_Payload_Info().
  - At the end of the procedure, the function HAL_I3C_CTRL_SetConfigBusDevices() must be called to store the target
    capabilities in the hardware register, including dynamic address, IBI support with or without additional data byte,
    Controller-Role request support, or controller automatic stop transfer after IBI.

### 7.3 Device ready checks
  - To check if I3C target device is ready for communication, use the function HAL_I3C_CTRL_PoolForDeviceI3cReady().
  - To check if I2C target device is ready for communication, use the function HAL_I3C_CTRL_PoolForDeviceI2cReady().

### 7.4 Bus arbitration generation
  - To send a message header {S + 0x7E + W + STOP}, use the function HAL_I3C_CTRL_GenerateArbitration().

### 7.5 Reset pattern insertion
   The controller app must enable the reset pattern configuration using HAL_I3C_CTRL_EnableResetPattern()
   before calling HAL_I3C_CTRL_Transfer().
   To have a standard STOP emitted at the end of a frame containing a RSTACT CCC command, the application must
   disable the reset pattern configuration using HAL_I3C_CTRL_DisableResetPattern() before calling
   HAL_I3C_CTRL_Transfer().
   Use HAL_I3C_CTRL_IsEnabledResetPattern() to check Reset pattern configuration.

### 7.6 Pattern generation
   - To send a target reset pattern or HDR exit pattern, use the function HAL_I3C_CTRL_GeneratePatterns().

## 8. Controller IO operations
### 8.1 Prepare transfer context
  Before initiating any IO operation, prepare the frame descriptors in the transfer context with their associated
  buffer allocation. It is purely software; no I3C handle is needed.
  Respect the following steps:
  - Reset the transfer context: HAL_I3C_CTRL_ResetTransferCtx().
  - Provide the control buffer: HAL_I3C_CTRL_InitTransferCtxTc()
  - Provide the Tx buffer (if needed): HAL_I3C_CTRL_InitTransferCtxTc()
  - Provide the Rx buffer (if needed): HAL_I3C_CTRL_InitTransferCtxRx()
  - Build the transfer context. Private: HAL_I3C_CTRL_BuildTransferCtxPrivate() or
    CCC: HAL_I3C_CTRL_BuildTransferCtxCCC()
  - The built transfer context is ready for controller IO operation. The driver does not modify it during the
  operation. It can be reused or stored.

### 8.2 Controller polling IO operation (blocking)
  - Start a controller-mode transfer of a Common Command Code in a direct or direct receive CCC with defbyte or
    private data in an I2C or I3C communication: HAL_I3C_CTRL_Transfer().

### 8.3 Controller DMA and interrupt mode IO operation
  - Transmit and/or receive a quantity of private data in an I3C or an I2C or a broadcast or a direct communication
    in non-blocking mode: HAL_I3C_CTRL_Transfer_IT() or HAL_I3C_CTRL_Transfer_DMA().
  - At the end of the transfer, HAL_I3C_CTRL_TransferCpltCallback() is executed, and user code can add custom handling
    by overriding this weak callback function or by registering a callback function.

## 9. Target IO operations
### 9.1 Target polling IO operation (blocking)
  - Transmit private data in target mode: HAL_I3C_TGT_Transmit()
  - Receive private data in target mode: HAL_I3C_TGT_Receive()
  - At the end of a transfer, flush FIFOs if needed by using HAL_I3C_FlushAllFifos() to flush all FIFOs, or flush
    individual FIFOs by using HAL_I3C_FlushTxFifo(), HAL_I3C_FlushRxFifo().
  - Request a Hot-Join in target mode: HAL_I3C_TGT_HotJoinReq()
  - Request an In-Band Interrupt in target mode: HAL_I3C_TGT_IBIReq()
  - Request a Controller-Role in target mode: HAL_I3C_TGT_ControlRoleReq()

### 9.2 Target DMA and Interrupt mode IO operation
  - Transmit private data in target mode in an I3C communication by using HAL_I3C_TGT_Transmit_IT() or
    HAL_I3C_TGT_Transmit_DMA().
  - At the end of the transfer, HAL_I3C_TGT_TxCpltCallback() is executed, and user code can add custom handling by
    overriding this weak callback function or by registering a callback function.
  - Receive private data in target mode in an I3C communication: HAL_I3C_TGT_Receive_IT() or
    HAL_I3C_TGT_Receive_DMA().
  - At the end of the transfer, HAL_I3C_TGT_RxCpltCallback() is executed, and user code can add custom handling by
    overriding this weak callback function or by registering a callback function.

## 10. Notifications and asynchronous events
  To get asynchronous events, use HAL_I3C_CTRL_ActivateNotification() or HAL_I3C_TGT_ActivateNotification().
  Each time one or more events are detected by hardware, HAL_I3C_NotifyCallback() is executed, and user code can add
  custom handling by overriding this weak callback function or by registering a callback function.
  Then retrieve specific associated event data through HAL_I3C_GetCCCInfo().

## 11. Error management
  In case of transfer error, HAL_I3C_ErrorCallback() is executed, and user code can add custom handling by overriding
  this weak callback function or by registering a callback function.

## 12. Abort operation
  Abort an I3C communication process with interrupt using HAL_I3C_Abort_IT().
  At the end of the abort process, HAL_I3C_AbortCpltCallback() is executed, and user code can add custom handling by
  overriding this weak callback function or by registering a callback function.

## 13. Callback registration
  The compilation flag USE_HAL_I3C_REGISTER_CALLBACKS allows dynamic configuration of
  the driver callbacks via dedicated registration APIs, instead of relying on weak default functions.

  Pointer in hal_i3c_handle_t  | weak default functions               | Callback registration function
  ---------------------------- | ------------------------------------ | --------------------------------------------
  DAACpltCallback              | HAL_I3C_CTRL_DAACpltCallback()       | HAL_I3C_CTRL_RegisterDAACpltCallback()
  TransferCpltCallback         | HAL_I3C_CTRL_TransferCpltCallback()  | HAL_I3C_CTRL_RegisterTransferCpltCallback()
  TgtReqDynAddrCallback        | HAL_I3C_CTRL_TgtReqDynAddrCallback() | HAL_I3C_CTRL_RegisterTgtReqDynAddrCallback()
  TgtTxCpltCallback            | HAL_I3C_TGT_TxCpltCallback()         | HAL_I3C_TGT_RegisterTxCpltCallback()
  TgtRxCpltCallback            | HAL_I3C_TGT_RxCpltCallback()         | HAL_I3C_TGT_RegisterRxCpltCallback()
  TgtHotJoinCallback           | HAL_I3C_TGT_HotJoinCallback()        | HAL_I3C_TGT_RegisterHotJoinCallback()
  NotifyCallback               | HAL_I3C_NotifyCallback()             | HAL_I3C_RegisterNotifyCallback()
  AbortCpltCallback            | HAL_I3C_AbortCpltCallback()          | HAL_I3C_RegisterAbortCpltCallback()
  ErrorCallback                | HAL_I3C_ErrorCallback()              | HAL_I3C_RegisterErrorCallback()

  Notes:
  - To unregister and restore the default weak callback, re-register the corresponding default function.
  - By default, after HAL_I3C_Init() and when the state is HAL_I3C_STATE_INIT, all callbacks are set to their
    default weak implementations.
  - Register callbacks when handle global_state is HAL_I3C_STATE_INIT or HAL_I3C_STATE_IDLE.
  - When USE_HAL_I3C_REGISTER_CALLBACKS is set to 0 or not defined, the callback registration feature is not
    available and weak callbacks are used.

## 14. Acquire/Release the I3C bus
  - When the compilation flag USE_HAL_MUTEX is set to 1, it allows the user to acquire/reserve the whole I3C bus for
    executing a process.
    The HAL Acquire/Release are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
      - HAL_I3C_AcquireBus() to acquire the bus or wait for it.
      - HAL_I3C_ReleaseBus() to release the bus.

  - When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_I3C_AcquireBus() and HAL_I3C_ReleaseBus()
    are not available.
  */
/**
  * @}
  */

/** @defgroup I3C_Configuration_Table I3C Configuration Table
  * @{

## 15. Software configuration

### 15.1 Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_I3C_MODULE               |         1         | Enable HAL I3C driver module
USE_HAL_I3C_REGISTER_CALLBACKS   |         0         | Allow user-defined callbacks
USE_HAL_I3C_DMA                  |         1         | Enable DMA code inside I3C
USE_HAL_CHECK_PARAM              |         0         | Enable runtime parameter check
USE_HAL_I3C_CLK_ENABLE_MODEL     | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE      |         0         | Enable atomicity of process state check
USE_HAL_MUTEX                    |         0         | Enable semaphore creation for OS
USE_HAL_I3C_USER_DATA            |         0         | Add user data inside HAL I3C handle
USE_HAL_I3C_GET_LAST_ERRORS      |         0         | Enable retrieval of last process error codes

### 15.2 Software configuration defined in preprocessor environment:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM             |    Not defined    | Enable check param for HAL and LL
USE_ASSERT_DBG_STATE             |    Not defined    | Enable check state for HAL

  */
/**
  * @}
  */
#if defined (I3C1)
#if defined(USE_HAL_I3C_MODULE) && (USE_HAL_I3C_MODULE == 1)

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Private_Types I3C Private Types
  * @{
  */

/**
  * @brief Structure containing address device and message type used for
  *        the private function I3C_Ctrl_PoolForDeviceReady()
  */
typedef struct
{
  uint8_t address; /*!< Dynamic or Static target Address */
  uint32_t message_type; /*!< Message Type */
} i3c_device_t;

/**
  * @}
  */

/* Private define ----------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Private_Constants I3C Private Constants
  * @{
  */

/* Private defines for control buffer prior preparation */
#define I3C_OPERATION_TYPE_MASK     I3C_CR_MTYPE         /*!< Mask can be combined with hal_i3c_transfer_mode_t */
#define I3C_RESTART_STOP_MASK       LL_I3C_GENERATE_STOP /*!< Mask can be combined with hal_i3c_transfer_mode_t */
#define I3C_ARBITRATION_HEADER_MASK I3C_CFGR_NOARBH      /*!< Mask can be combined with hal_i3c_transfer_mode_t */
#define I3C_DEFINE_BYTE_MASK        LL_I3C_DEFINE_BYTE   /*!< Mask can be combined with hal_i3c_transfer_mode_t */

/* Private defines for listen mode control */
#define I3C_LISTEN_DISABLED  (0U) /*!< Listen mode disabled */
#define I3C_LISTEN_ENABLED   (1U) /*!< Listen mode enabled */

/* Private defines for Transfer max size in interrupt mode */
#define I3C_TRANSFER_MAX_BYTE   (16U) /*!< 2 * FIFO size in byte */
#define I3C_TRANSFER_MAX_WORD   (4U)  /*!< 2 * FIFO size in word */

#define I3C_DEFAULT_TIMEOUT_MS  (25U) /*!< 25 ms */

/**
  * @brief I3C timing register 1 (I3C_TIMINGR1) valid bit mask
  */
#define I3C_TIMINGR1_VALID_MSK (I3C_TIMINGR1_AVAL_Msk |\
                                I3C_TIMINGR1_ASNCR_Msk | I3C_TIMINGR1_FREE_Msk | I3C_TIMINGR1_SDA_HD_Msk)

/**
  * @}
  */

/* Private macro -----------------------------------------------------------------------------------------------------*/
/** @defgroup I3C_Private_Macros I3C Private Macros
  * @{
  */

/**
  * @brief Validate ENTDAA option.
  * @param OPTION Value to test (HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA or HAL_I3C_DYN_ADDR_ONLY_ENTDAA).
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_ENTDAA_OPTION(OPTION) (((OPTION) == HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA) \
                                      || ((OPTION) == HAL_I3C_DYN_ADDR_ONLY_ENTDAA))

/**
  * @brief Validate TX FIFO threshold selection.
  * @param VALUE Threshold enum (HAL_I3C_TX_FIFO_THRESHOLD_1_8 or HAL_I3C_TX_FIFO_THRESHOLD_1_2).
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_TX_FIFO_THRESHOLD(VALUE) (((VALUE) == HAL_I3C_TX_FIFO_THRESHOLD_1_8)\
                                         || ((VALUE) == HAL_I3C_TX_FIFO_THRESHOLD_1_2))

/**
  * @brief Validate RX FIFO threshold selection.
  * @param VALUE Threshold enum (HAL_I3C_RX_FIFO_THRESHOLD_1_8 or HAL_I3C_RX_FIFO_THRESHOLD_1_2).
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_RX_FIFO_THRESHOLD(VALUE) (((VALUE) == HAL_I3C_RX_FIFO_THRESHOLD_1_8) \
                                         || ((VALUE) == HAL_I3C_RX_FIFO_THRESHOLD_1_2))

/**
  * @brief Validate controller FIFO mode mask.
  * @param CONTROLFIFO One of HAL_I3C_CTRL_FIFO_* values.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_FIFO(CONTROLFIFO) (((CONTROLFIFO) == HAL_I3C_CTRL_FIFO_NONE) \
                                       || ((CONTROLFIFO) == HAL_I3C_CTRL_FIFO_CONTROL_ONLY) \
                                       || ((CONTROLFIFO) == HAL_I3C_CTRL_FIFO_STATUS_ONLY) \
                                       || ((CONTROLFIFO) == HAL_I3C_CTRL_FIFO_ALL))

/**
  * @brief Validate number of devices to configure.
  * @param VALUE Device count (1..4).
  * @retval 1 if in range, 0 otherwise.
  */
#define IS_I3C_DEVICE(VALUE) (((VALUE) >= 1U) && ((VALUE) <= 4U))

/**
  * @brief Validate device index.
  * @param VALUE Index (0..3).
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DEVICE_INDEX(VALUE) ((VALUE) <= 3U)

/**
  * @brief Validate dynamic address value.
  * @param VALUE 7-bit dynamic address (0x00..0x7F).
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DYNAMIC_ADDRESS(VALUE) ((VALUE) <= 0x7FU)

/**
  * @brief Validate handoff activity state.
  * @param VALUE One of HAL_I3C_HANDOFF_ACTIVITY_STATE_{0..3}.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_HANDOFF_ACTIVITY_STATE(VALUE) (((VALUE) == HAL_I3C_HANDOFF_ACTIVITY_STATE_0) \
                                              || ((VALUE) == HAL_I3C_HANDOFF_ACTIVITY_STATE_1) \
                                              || ((VALUE) == HAL_I3C_HANDOFF_ACTIVITY_STATE_2) \
                                              || ((VALUE) == HAL_I3C_HANDOFF_ACTIVITY_STATE_3))

/**
  * @brief Validate TSCO turnaround time selection.
  * @param VALUE HAL_I3C_TURNAROUND_TIME_TSCO_* value.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_TSCO_TIME(VALUE) (((VALUE) == HAL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS) \
                                 || ((VALUE) == HAL_I3C_TURNAROUND_TIME_TSCO_GREATER_12NS))

/**
  * @brief Validate GETMXDS format value.
  * @param VALUE Format enum HAL_I3C_GETMXDS_FORMAT_*.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_MAX_SPEED_DATA(VALUE) (((VALUE) == HAL_I3C_GETMXDS_FORMAT_1    ) \
                                      || ((VALUE) == HAL_I3C_GETMXDS_FORMAT_2_LSB) \
                                      || ((VALUE) == HAL_I3C_GETMXDS_FORMAT_2_MID) \
                                      || ((VALUE) == HAL_I3C_GETMXDS_FORMAT_2_MSB))

/**
  * @brief Validate IBI payload size.
  * @param VALUE HAL_I3C_TGT_PAYLOAD_* size selection.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_IBI_PAYLOAD_SIZE(VALUE) (((VALUE) == HAL_I3C_TGT_PAYLOAD_EMPTY  ) \
                                        || ((VALUE) == HAL_I3C_TGT_PAYLOAD_1_BYTE ) \
                                        || ((VALUE) == HAL_I3C_TGT_PAYLOAD_2_BYTE) \
                                        || ((VALUE) == HAL_I3C_TGT_PAYLOAD_3_BYTE) \
                                        || ((VALUE) == HAL_I3C_TGT_PAYLOAD_4_BYTE))

/**
  * @brief Validate MIPI identifier nibble.
  * @param VALUE 0x0..0xF.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_MIPI_IDENTIFIER(VALUE) ((VALUE) <= 0x0FU)

/**
  * @brief Test if interrupt source bit(s) are set.
  * @param IER Interrupt enable register value.
  * @param IT Mask to test.
  * @retval 1 if all bits set, 0 otherwise.
  */
#define I3C_CHECK_IT_SOURCE(IER, IT)  ((((IER) & (IT)) == (IT)) ? 1U : 0U)

/**
  * @brief Test if flag bit(s) are set.
  * @param ISR Flag/status register value.
  * @param FLAG Mask to test.
  * @retval 1 if all bits set, 0 otherwise.
  */
#define I3C_CHECK_FLAG(ISR, FLAG) ((((ISR) & (FLAG)) == (FLAG)) ? 1U : 0U)

/**
  * @brief Validate DMA source width is byte.
  * @param VALUE Expected HAL_DMA_SRC_DATA_WIDTH_BYTE.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DMA_SOURCE_BYTE(VALUE) ((VALUE) == HAL_DMA_SRC_DATA_WIDTH_BYTE)

/**
  * @brief Validate DMA source width is word.
  * @param VALUE Expected HAL_DMA_SRC_DATA_WIDTH_WORD.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DMA_SOURCE_WORD(VALUE) ((VALUE) == HAL_DMA_SRC_DATA_WIDTH_WORD)

/**
  * @brief Validate DMA destination width is byte.
  * @param VALUE Expected HAL_DMA_DEST_DATA_WIDTH_BYTE.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DMA_DESTINATION_BYTE(VALUE) ((VALUE) == HAL_DMA_DEST_DATA_WIDTH_BYTE)

/**
  * @brief Validate DMA destination width is word.
  * @param VALUE Expected HAL_DMA_DEST_DATA_WIDTH_WORD.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DMA_DESTINATION_WORD(VALUE) ((VALUE) == HAL_DMA_DEST_DATA_WIDTH_WORD)

/**
  * @brief Validate generated pattern type.
  * @param PATTERN HAL_I3C_PATTERN_TGT_RESET or HAL_I3C_PATTERN_HDR_EXIT.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_PATTERN(PATTERN) (((PATTERN) == HAL_I3C_PATTERN_TGT_RESET) \
                                 || ((PATTERN) == HAL_I3C_PATTERN_HDR_EXIT))


/**
  * @brief Validate controller-role request status.
  * @param CTRLROLE HAL_I3C_CTRL_ROLE_DISABLED or HAL_I3C_CTRL_ROLE_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_ROLE(CTRLROLE) (((CTRLROLE) == HAL_I3C_CTRL_ROLE_DISABLED) \
                                    || ((CTRLROLE) == HAL_I3C_CTRL_ROLE_ENABLED))

/**
  * @brief Validate controller-role acknowledgment configuration.
  * @param CTRLROLEACK HAL_I3C_CTRL_ROLE_ACK_DISABLED or HAL_I3C_CTRL_ROLE_ACK_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_ROLE_ACK(CTRLROLEACK) (((CTRLROLEACK) == HAL_I3C_CTRL_ROLE_ACK_DISABLED) \
                                           || ((CTRLROLEACK) == HAL_I3C_CTRL_ROLE_ACK_ENABLED))

/**
  * @brief Validate IBI payload enable status.
  * @param IBIPAYLOAD HAL_I3C_IBI_PAYLOAD_DISABLED or HAL_I3C_IBI_PAYLOAD_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_IBI_PAYLOAD(IBIPAYLOAD) (((IBIPAYLOAD) == HAL_I3C_IBI_PAYLOAD_DISABLED) \
                                        || ((IBIPAYLOAD) == HAL_I3C_IBI_PAYLOAD_ENABLED))
/**
  * @brief Validate IBI payload enable status.
  * @param IBIPAYLOAD HAL_I3C_CTRL_IBI_PAYLOAD_DISABLED or HAL_I3C_CTRL_IBI_PAYLOAD_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_IBI_PAYLOAD(IBIPAYLOAD) (((IBIPAYLOAD) == HAL_I3C_CTRL_IBI_PAYLOAD_DISABLED) \
                                             || ((IBIPAYLOAD) == HAL_I3C_CTRL_IBI_PAYLOAD_ENABLED))
/**
  * @brief Validate max speed limitation status.
  * @param MAX_SPEED HAL_I3C_MAX_SPEED_LIMITATION_DISABLED or HAL_I3C_MAX_SPEED_LIMITATION_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_MAX_SPEED_LIMIT(MAX_SPEED) (((MAX_SPEED) == HAL_I3C_MAX_SPEED_LIMITATION_DISABLED) \
                                           || ((MAX_SPEED) == HAL_I3C_MAX_SPEED_LIMITATION_ENABLED))

/**
  * @brief Validate stop transfer configuration.
  * @param VALUE HAL_I3C_CTRL_STOP_TRANSFER_DISABLED or HAL_I3C_CTRL_STOP_TRANSFER_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_STOP_XFER(VALUE) (((VALUE) == HAL_I3C_CTRL_STOP_TRANSFER_DISABLED) \
                                      || ((VALUE) == HAL_I3C_CTRL_STOP_TRANSFER_ENABLED))

/**
  * @brief Validate IBI ACK enable status.
  * @param IBI HAL_I3C_CTRL_IBI_ACK_DISABLED or HAL_I3C_CTRL_IBI_ACK_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_IBI_ACK(IBI) (((IBI) == HAL_I3C_CTRL_IBI_ACK_DISABLED) \
                                  || ((IBI) == HAL_I3C_CTRL_IBI_ACK_ENABLED))

/**
  * @brief Validate pending read MDB enable status.
  * @param PENDING_READ HAL_I3C_PENDING_READ_MDB_DISABLED or HAL_I3C_PENDING_READ_MDB_ENABLED.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_PENDING_READ(PENDING_READ) (((PENDING_READ) == HAL_I3C_PENDING_READ_MDB_DISABLED) \
                                           || ((PENDING_READ) == HAL_I3C_PENDING_READ_MDB_ENABLED))

/**
  * @brief Validate private transfer mode selection.
  * @param MODE One HAL_I3C_PRIVATE / HAL_I2C_PRIVATE mode enum.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_PRIVATE_MODE(MODE) (((MODE) == HAL_I3C_PRIVATE_WITH_ARB_STOP)                 \
                                   || ((MODE) == HAL_I3C_PRIVATE_WITHOUT_ARB_RESTART)        \
                                   || ((MODE) == HAL_I3C_PRIVATE_WITHOUT_ARB_STOP)           \
                                   || ((MODE) == HAL_I2C_PRIVATE_WITH_ARB_RESTART)           \
                                   || ((MODE) == HAL_I2C_PRIVATE_WITH_ARB_STOP)              \
                                   || ((MODE) == HAL_I2C_PRIVATE_WITHOUT_ARB_RESTART)        \
                                   || ((MODE) == HAL_I2C_PRIVATE_WITHOUT_ARB_STOP))

/**
  * @brief Validate CCC transfer mode selection.
  * @param MODE One HAL_I3C_CCC_* mode enum.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CCC_MODE(MODE) (((MODE) == HAL_I3C_CCC_DIRECT_WITH_DEFBYTE_RESTART)           \
                               || ((MODE) == HAL_I3C_CCC_DIRECT_WITH_DEFBYTE_STOP)           \
                               || ((MODE) == HAL_I3C_CCC_DIRECT_WITHOUT_DEFBYTE_RESTART)     \
                               || ((MODE) == HAL_I3C_CCC_DIRECT_WITHOUT_DEFBYTE_STOP)        \
                               || ((MODE) == HAL_I3C_CCC_BROADCAST_WITH_DEFBYTE_RESTART)     \
                               || ((MODE) == HAL_I3C_CCC_BROADCAST_WITH_DEFBYTE_STOP)        \
                               || ((MODE) == HAL_I3C_CCC_BROADCAST_WITHOUT_DEFBYTE_RESTART)  \
                               || ((MODE) == HAL_I3C_CCC_BROADCAST_WITHOUT_DEFBYTE_STOP))


/**
  * @brief Validate transfer direction for private/direct CCC.
  * @param DIRECTION HAL_I3C_DIRECTION_WRITE or HAL_I3C_DIRECTION_READ.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DIRECTION(DIRECTION) (((DIRECTION) == HAL_I3C_DIRECTION_WRITE) \
                                     || ((DIRECTION) == HAL_I3C_DIRECTION_READ))


/**
  * @brief Validate direction for broadcast CCC (must be write).
  * @param DIRECTION Expected HAL_I3C_DIRECTION_WRITE.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_DIRECTION_CCC_BROADCAST(DIRECTION) ((DIRECTION) == HAL_I3C_DIRECTION_WRITE)

/**
  * @brief Validate controller notification mask.
  * @param VALUE Combination of I3C_CTRL_NOTIFICATION.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_CTRL_NOTIFICATIONS(VALUE) (((VALUE) & ~HAL_I3C_CTRL_NOTIFICATION_ALL) == 0U)

/**
  * @brief Validate target notification mask.
  * @param VALUE Combination of I3C_TGT_NOTIFICATION.
  * @retval 1 if valid, 0 otherwise.
  */
#define IS_I3C_TGT_NOTIFICATIONS(VALUE) (((VALUE) & ~HAL_I3C_TGT_NOTIFICATION_ALL) == 0U)


/**
  * @brief Get CMSIS peripheral instance from HAL handle.
  * @param handle Pointer to @ref hal_i3c_handle_t.
  * @retval I3C_TypeDef* Peripheral base address.
  */
#define I3C_GET_INSTANCE(handle) ((I3C_TypeDef *)((uint32_t)((handle)->instance)))
/**
  * @}
  */

/* Private function prototypes ---------------------------------------------------------------------------------------*/
/** @defgroup I3C_Private_Functions I3C Private Functions
  * @{
  */
static hal_status_t I3C_Tgt_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Ctrl_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Tgt_Tx_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Tgt_Rx_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
static hal_status_t I3C_Tgt_Tx_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Tgt_Rx_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
#endif /* USE_HAL_I3C_DMA */
static hal_status_t I3C_Tgt_HotJoin_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Tgt_CtrlRole_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Tgt_IBI_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Ctrl_Multiple_Xfer_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Ctrl_Multiple_Xfer_Listen_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
static hal_status_t I3C_Ctrl_Multiple_Xfer_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
#endif /* USE_HAL_I3C_DMA */
static hal_status_t I3C_Ctrl_DAA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_Abort_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks);
static hal_status_t I3C_WaitForFlagSet(hal_i3c_handle_t *hi3c, uint32_t flag, uint32_t timeout_ms);
static void I3C_TransmitByteTreatment(hal_i3c_handle_t *hi3c);
static void I3C_TransmitByteTreatment_IT(hal_i3c_handle_t *hi3c);
static void I3C_TransmitWordTreatment(hal_i3c_handle_t *hi3c);
static void I3C_TransmitWordTreatment_IT(hal_i3c_handle_t *hi3c);
static void I3C_ReceiveByteTreatment(hal_i3c_handle_t *hi3c);
static void I3C_ReceiveByteTreatment_IT(hal_i3c_handle_t *hi3c);
static void I3C_ReceiveWordTreatment(hal_i3c_handle_t *hi3c);
static void I3C_ReceiveWordTreatment_IT(hal_i3c_handle_t *hi3c);
static void I3C_ControlDataTreatment(hal_i3c_handle_t *hi3c);
static void I3C_ErrorTreatment(hal_i3c_handle_t *hi3c);
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
static void I3C_GetErrorSources(hal_i3c_handle_t *hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

static void I3C_StateIdle(hal_i3c_handle_t *hi3c);
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
static void I3C_DMAAbort(hal_dma_handle_t *hdma);
static void I3C_DMAControlTransmitCplt(hal_dma_handle_t *hdma);
static void I3C_DMADataTransmitCplt(hal_dma_handle_t *hdma);
static void I3C_DMADataReceiveCplt(hal_dma_handle_t *hdma);
static void I3C_DMAError(hal_dma_handle_t *hdma);
static uint32_t I3C_RoundUp4(uint32_t size_byte);
#endif /* USE_HAL_I3C_DMA */
static hal_status_t I3C_Ctrl_PoolForDeviceReady(hal_i3c_handle_t *hi3c, const i3c_device_t *p_device,
                                                uint32_t timeout_ms);
static void I3C_TreatErrorCallback(hal_i3c_handle_t *hi3c);

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup I3C_Exported_Functions
  * @{
  */

/** @addtogroup I3C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
A set of functions that allow initialization and deinitialization of the I3Cx peripheral:
 - HAL_I3C_Init(): initialize the selected device with the I3C instance.
 - HAL_I3C_DeInit(): disable the peripheral.
  */

/**
  * @brief  Initialize the I3C according to the associated handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  instance  HAL I3C instance
  * @retval HAL_OK  HAL I3C instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM  HAL I3C instance is NULL
  * @retval HAL_ERROR  HAL I3C semaphore creation is failed (USE_HAL_MUTEX is Set to 1)
  */
hal_status_t HAL_I3C_Init(hal_i3c_handle_t *hi3c, hal_i3c_t instance)
{

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_ALL_INSTANCE((I3C_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hi3c == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->instance = instance;

#if defined(USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
  /* Set I3C Callbacks to the weak function */
  hi3c->p_ctrl_transfer_cplt_cb = HAL_I3C_CTRL_TransferCpltCallback;
  hi3c->p_ctrl_daa_cplt_cb = HAL_I3C_CTRL_DAACpltCallback;
  hi3c->p_ctrl_tgt_req_dyn_addr_cb = HAL_I3C_CTRL_TgtReqDynAddrCallback;
  hi3c->p_tgt_tx_cplt_cb = HAL_I3C_TGT_TxCpltCallback;
  hi3c->p_tgt_rx_cplt_cb = HAL_I3C_TGT_RxCpltCallback;
  hi3c->p_tgt_hot_join_cb = HAL_I3C_TGT_HotJoinCallback;
  hi3c->p_notify_cb = HAL_I3C_NotifyCallback;
  hi3c->p_error_cb = HAL_I3C_ErrorCallback;
  hi3c->p_abort_cplt_cb = HAL_I3C_AbortCpltCallback;
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */

  hi3c->mode = HAL_I3C_MODE_NONE;
  hi3c->listen = I3C_LISTEN_DISABLED;

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
  hi3c->hdma_tx = (hal_dma_handle_t *) NULL;
  hi3c->hdma_rx = (hal_dma_handle_t *) NULL;
  hi3c->hdma_tc = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_I3C_DMA */

#if defined(USE_HAL_I3C_USER_DATA) && (USE_HAL_I3C_USER_DATA == 1)
  hi3c->p_user_data = (void *) NULL;
#endif /* USE_HAL_I3C_USER_DATA */

#if defined(USE_HAL_I3C_CLK_ENABLE_MODEL) && (USE_HAL_I3C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  /* Enable I3C1 Clock */
  HAL_RCC_I3C1_EnableClock();
#endif /* USE_HAL_I3C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  if (HAL_OS_SemaphoreCreate(&hi3c->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  /* Update I3C state */
  hi3c->global_state = HAL_I3C_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the HAL I3C driver for the given handle and disable the peripheral.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t structure
  */
void HAL_I3C_DeInit(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  p_i3cx = I3C_GET_INSTANCE(hi3c);
  ASSERT_DBG_PARAM(IS_I3C_ALL_INSTANCE(p_i3cx));

  LL_I3C_Disable(p_i3cx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void) HAL_OS_SemaphoreDelete(&hi3c->semaphore);
#endif /* USE_HAL_MUTEX */

  hi3c->global_state = HAL_I3C_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group2 Configuration functions
  * @{
A set of functions that allow configuration of the I3Cx peripheral:

- Global configuration (controller):
  - HAL_I3C_CTRL_SetConfig()
  - HAL_I3C_CTRL_GetConfig()

- Global configuration (target):
  - HAL_I3C_TGT_SetConfig()
  - HAL_I3C_TGT_GetConfig()

- Payload ENTDAA configuration (target):
  - HAL_I3C_TGT_SetPayloadENTDAAConfig()
  - HAL_I3C_TGT_GetPayloadENTDAAConfig()

- FIFO configuration (target):
  - HAL_I3C_TGT_SetConfigFifo()
  - HAL_I3C_TGT_GetConfigFifo()

- FIFO configuration (controller):
  - HAL_I3C_CTRL_SetConfigFifo()
  - HAL_I3C_CTRL_GetConfigFifo()

- FIFO configuration unitary functions:
  - HAL_I3C_SetRxFifoThreshold()
  - HAL_I3C_GetRxFifoThreshold()
  - HAL_I3C_SetTxFifoThreshold()
  - HAL_I3C_GetTxFifoThreshold()
  - HAL_I3C_CTRL_EnableControlFifo()
  - HAL_I3C_CTRL_DisableControlFifo()
  - HAL_I3C_CTRL_IsEnabledControlFifo()
  - HAL_I3C_CTRL_EnableStatusFifo()
  - HAL_I3C_CTRL_DisableStatusFifo()
  - HAL_I3C_CTRL_IsEnabledStatusFifo()

- Own dynamic address (controller):
  - HAL_I3C_CTRL_SetConfigOwnDynamicAddress()
  - HAL_I3C_CTRL_GetConfigOwnDynamicAddress()

- Hot-Join allowed (controller):
  - HAL_I3C_CTRL_EnableHotJoinAllowed()
  - HAL_I3C_CTRL_DisableHotJoinAllowed()
  - HAL_I3C_CTRL_IsEnabledHotJoinAllowed()

- High keeper SDA configuration (controller):
  - HAL_I3C_CTRL_EnableHighKeeperSDA()
  - HAL_I3C_CTRL_DisableHighKeeperSDA()
  - HAL_I3C_CTRL_IsEnabledHighKeeperSDA()

- Stall time configuration (controller):
  - HAL_I3C_CTRL_SetConfigStallTime()
  - HAL_I3C_CTRL_GetConfigStallTime()

- Controller-Role request configuration (target):
  - HAL_I3C_TGT_EnableCtrlRoleRequest()
  - HAL_I3C_TGT_DisableCtrlRoleRequest()
  - HAL_I3C_TGT_IsEnabledCtrlRoleRequest()

- Group management support configuration (target):
  - HAL_I3C_TGT_EnableGroupAddrCapability()
  - HAL_I3C_TGT_DisableGroupAddrCapability()
  - HAL_I3C_TGT_IsEnabledGroupAddrCapability()

- Hot-Join configuration (target):
  - HAL_I3C_TGT_EnableHotJoinRequest()
  - HAL_I3C_TGT_DisableHotJoinRequest()
  - HAL_I3C_TGT_IsEnabledHotJoinRequest()

- In-Band Interrupt configuration (target):
  - HAL_I3C_TGT_SetConfigIBI()
  - HAL_I3C_TGT_GetConfigIBI()
  - HAL_I3C_TGT_EnableIBI()
  - HAL_I3C_TGT_DisableIBI()
  - HAL_I3C_TGT_IsEnabledIBI()

- Max data size configuration (target):
  - HAL_I3C_TGT_SetConfigMaxDataSize()
  - HAL_I3C_TGT_GetConfigMaxDataSize()

- GET MaX Data Speed (GETMXDS) configuration (target):
  - HAL_I3C_TGT_SetConfigGETMXDS()
  - HAL_I3C_TGT_GetConfigGETMXDS()
  - HAL_I3C_TGT_SetConfigGETMXDS_Format()
  - HAL_I3C_TGT_GetConfigGETMXDS_Format()
  - HAL_I3C_TGT_SetConfigCtrlHandOffActivity()
  - HAL_I3C_TGT_GetConfigCtrlHandOffActivity()

- Bus device configuration in DEVRX[] (controller):
  - HAL_I3C_CTRL_SetConfigBusDevices()
  - HAL_I3C_CTRL_GetConfigBusDevices()

- Reset pattern configuration (Controller):
  - HAL_I3C_CTRL_EnableResetPattern()
  - HAL_I3C_CTRL_DisableResetPattern()
  - HAL_I3C_CTRL_IsEnabledResetPattern()
  */

/**
  * @brief  Configure the I3C as controller.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_SetConfig(hal_i3c_handle_t *hi3c, const hal_i3c_ctrl_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM((p_config->timing_reg1 & ~I3C_TIMINGR1_VALID_MSK) == 0);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetMode(p_i3cx, LL_I3C_MODE_CONTROLLER);
  hi3c->mode = HAL_I3C_MODE_CTRL;

  /* SCL signal waveform configuration: I3C timing register 0 (I3C_TIMINGR0) */
  LL_I3C_ConfigClockWaveForm(p_i3cx, p_config->timing_reg0);

  /* Timing configuration: I3C timing register 1 (I3C_TIMINGR1) */
  LL_I3C_SetBusCharacteristic(p_i3cx, p_config->timing_reg1);

  LL_I3C_Enable(p_i3cx);
  hi3c->global_state = HAL_I3C_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the I3C controller configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_CTRL_GetConfig(const hal_i3c_handle_t *hi3c, hal_i3c_ctrl_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  p_config->timing_reg0 = LL_I3C_GetClockWaveForm(p_i3cx);
  p_config->timing_reg1 = LL_I3C_GetBusCharacteristic(p_i3cx);
}

/**
  * @brief  Configure the I3C as target.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_SetConfig(hal_i3c_handle_t *hi3c, const hal_i3c_tgt_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM((p_config->timing_reg1 & ~I3C_TIMINGR1_VALID_MSK) == 0);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetMode(p_i3cx, LL_I3C_MODE_TARGET);
  hi3c->mode = HAL_I3C_MODE_TGT;

  /* Timing configuration: I3C timing register 1 (I3C_TIMINGR1) */
  LL_I3C_SetBusCharacteristic(p_i3cx, p_config->timing_reg1);

  LL_I3C_Enable(p_i3cx);
  hi3c->global_state = HAL_I3C_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the I3C target configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_TGT_GetConfig(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_config_t *p_config)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_config->timing_reg1 = LL_I3C_GetBusCharacteristic(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set payload ENTDAA configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_SetPayloadENTDAAConfig(const hal_i3c_handle_t *hi3c,
                                                const hal_i3c_tgt_config_payload_entdaa_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I3C_MIPI_IDENTIFIER(p_config->mipi_identifier));
  ASSERT_DBG_PARAM(IS_I3C_CTRL_ROLE(p_config->ctrl_role));
  ASSERT_DBG_PARAM(IS_I3C_IBI_PAYLOAD(p_config->ibi_payload));
  ASSERT_DBG_PARAM(IS_I3C_MAX_SPEED_LIMIT(p_config->max_data_speed_limitation));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  /* Set identifier value in DCR register */
  LL_I3C_SetDeviceCharacteristics(p_i3cx, p_config->identifier);
  /* Set MIPI identifier value in EPIDR register */
  LL_I3C_SetMIPIInstanceID(p_i3cx, p_config->mipi_identifier);
  /* Set control capability, IBI payload support and max speed limitation in BCR register */
  LL_I3C_ConfigPayloadEntDAA(p_i3cx, (uint32_t) p_config->max_data_speed_limitation, (uint32_t) p_config->ibi_payload,
                             (uint32_t) p_config->ctrl_role);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Retrieve payload ENTDAA configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_TGT_GetPayloadENTDAAConfig(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_config_payload_entdaa_t *p_config)
{
  uint32_t bcr_value;
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);
  /* Get identifier value in DCR register */
  p_i3cx = I3C_GET_INSTANCE(hi3c);
  p_config->identifier = (uint8_t) LL_I3C_GetDeviceCharacteristics(p_i3cx);
  /* Get MIPI identifier value in EPIDR register */
  p_config->mipi_identifier = (uint8_t) LL_I3C_GetMIPIInstanceID(p_i3cx);

  bcr_value = LL_I3C_GetRegister_BCR(I3C_GET_INSTANCE(hi3c));
  p_config->max_data_speed_limitation = HAL_I3C_GET_MAX_DATA_SPEED_LIMIT(bcr_value);
  p_config->ibi_payload = HAL_I3C_GET_IBI_PAYLOAD(bcr_value);
  p_config->ctrl_role = HAL_I3C_GET_CTRL_ROLE_CAPABLE(bcr_value);
}

/**
  * @brief  Set FIFO configuration when the I3C is acting as controller.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_SetConfigFifo(const hal_i3c_handle_t *hi3c, const hal_i3c_ctrl_fifo_config_t *p_config)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I3C_RX_FIFO_THRESHOLD(p_config->rx_fifo_threshold));
  ASSERT_DBG_PARAM(IS_I3C_TX_FIFO_THRESHOLD(p_config->tx_fifo_threshold));
  ASSERT_DBG_PARAM(IS_I3C_CTRL_FIFO(p_config->ctrl_fifo));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_I3C_ConfigCtrlFifo(I3C_GET_INSTANCE(hi3c), (uint32_t)p_config->tx_fifo_threshold,
                        (uint32_t)p_config->rx_fifo_threshold, (uint32_t)p_config->ctrl_fifo);

  return HAL_OK;
}

/**
  * @brief  Set FIFO configuration when the I3C is acting as target.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_SetConfigFifo(const hal_i3c_handle_t *hi3c, const hal_i3c_tgt_fifo_config_t *p_config)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I3C_RX_FIFO_THRESHOLD(p_config->rx_fifo_threshold));
  ASSERT_DBG_PARAM(IS_I3C_TX_FIFO_THRESHOLD(p_config->tx_fifo_threshold));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_I3C_ConfigTgtFifo(I3C_GET_INSTANCE(hi3c), (uint32_t)p_config->tx_fifo_threshold,
                       (uint32_t)p_config->rx_fifo_threshold);

  return HAL_OK;
}

/**
  * @brief  Retrieve FIFO configuration when the I3C is acting as controller.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_CTRL_GetConfigFifo(const hal_i3c_handle_t *hi3c, hal_i3c_ctrl_fifo_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  p_config->ctrl_fifo = (hal_i3c_ctrl_fifo_t) LL_I3C_GetCtrlFifo(p_i3cx);
  p_config->tx_fifo_threshold = (hal_i3c_tx_fifo_threshold_t) LL_I3C_GetTxFIFOThreshold(p_i3cx);
  p_config->rx_fifo_threshold = (hal_i3c_rx_fifo_threshold_t) LL_I3C_GetRxFIFOThreshold(p_i3cx);
}

/**
  * @brief  Retrieve FIFO configuration when the I3C is acting as target.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_TGT_GetConfigFifo(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_fifo_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  p_config->tx_fifo_threshold = (hal_i3c_tx_fifo_threshold_t) LL_I3C_GetTxFIFOThreshold(p_i3cx);
  p_config->rx_fifo_threshold = (hal_i3c_rx_fifo_threshold_t) LL_I3C_GetRxFIFOThreshold(p_i3cx);
}

/**
  * @brief  Set the Receive FIFO Threshold level configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  threshold the Rx FIFO threshold
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_SetRxFifoThreshold(const hal_i3c_handle_t *hi3c, const hal_i3c_rx_fifo_threshold_t threshold)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_RX_FIFO_THRESHOLD(threshold));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_SetRxFIFOThreshold(I3C_GET_INSTANCE(hi3c), (uint32_t)threshold);

  return HAL_OK;
}

/**
  * @brief  Get the receive FIFO threshold level configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_RX_FIFO_THRESHOLD_1_8  Rx Fifo Threshold is 1 byte.
  * @retval HAL_I3C_RX_FIFO_THRESHOLD_1_2  Rx Fifo Threshold is 4 bytes.
  */
hal_i3c_rx_fifo_threshold_t HAL_I3C_GetRxFifoThreshold(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_rx_fifo_threshold_t) LL_I3C_GetRxFIFOThreshold(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set the TX FIFO Threshold level configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  threshold the Tx FIFO threshold
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_SetTxFifoThreshold(const hal_i3c_handle_t *hi3c, const hal_i3c_tx_fifo_threshold_t threshold)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_TX_FIFO_THRESHOLD(threshold));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_SetTxFIFOThreshold(I3C_GET_INSTANCE(hi3c), (uint32_t)threshold);

  return HAL_OK;
}

/**
  * @brief  Get the TX FIFO Threshold level configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_TX_FIFO_THRESHOLD_1_8  Tx Fifo Threshold is 1 byte
  * @retval HAL_I3C_TX_FIFO_THRESHOLD_1_2  Tx Fifo Threshold is 4 bytes
  */
hal_i3c_tx_fifo_threshold_t HAL_I3C_GetTxFifoThreshold(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_tx_fifo_threshold_t)LL_I3C_GetTxFIFOThreshold(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Enable the Control FIFO.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_EnableControlFifo(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_EnableControlFIFO(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Disable the Control FIFO.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DisableControlFifo(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_DisableControlFIFO(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Check if the Control FIFO is enabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_CONTROL_FIFO_DISABLED  Control FIFO mode disabled
  * @retval HAL_I3C_CONTROL_FIFO_ENABLED   Control FIFO mode enabled
  */
hal_i3c_control_fifo_status_t HAL_I3C_CTRL_IsEnabledControlFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_control_fifo_status_t) LL_I3C_IsEnabledControlFIFO(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Enable the Status FIFO.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_EnableStatusFifo(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_EnableStatusFIFO(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Disable the Status FIFO.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DisableStatusFifo(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_DisableStatusFIFO(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Check if the Status FIFO is enabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_STATUS_FIFO_DISABLED  Status FIFO mode disabled
  * @retval HAL_I3C_STATUS_FIFO_ENABLED   Status FIFO mode enabled
  */
hal_i3c_status_fifo_status_t HAL_I3C_CTRL_IsEnabledStatusFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_status_fifo_status_t) LL_I3C_IsEnabledStatusFIFO(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set dynamic address value.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  dynamic_address  Dynamic address
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_SetConfigOwnDynamicAddress(hal_i3c_handle_t *hi3c, uint32_t dynamic_address)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_DYNAMIC_ADDRESS(dynamic_address));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_SetAndEnableOwnDynamicAddress(I3C_GET_INSTANCE(hi3c), dynamic_address);

  return HAL_OK;
}

/**
  * @brief  Get dynamic address value.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval The dynamic address
  */
uint32_t HAL_I3C_CTRL_GetConfigOwnDynamicAddress(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (LL_I3C_GetOwnDynamicAddress(I3C_GET_INSTANCE(hi3c)));
}

/**
  * @brief  Enable Hot-Join request acknowledgement allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_EnableHotJoinAllowed(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_EnableHJAck(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Disable Hot-Join request acknowledgement allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DisableHotJoinAllowed(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_DisableHJAck(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Check if the Hot-Join request acknowledgement is enabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_HOT_JOIN_DISABLED  Hot-Join disable
  * @retval HAL_I3C_HOT_JOIN_ENABLED   Hot-Join enable
  */
hal_i3c_hot_join_status_t HAL_I3C_CTRL_IsEnabledHotJoinAllowed(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_hot_join_status_t) LL_I3C_IsEnabledHJAck(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Enable the high keeper SDA.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @note   This configuration will be used in place of standard Open drain Pull Up device
  *         during handoff procedures
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_EnableHighKeeperSDA(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_EnableHighKeeperSDA(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Disable the high keeper SDA.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DisableHighKeeperSDA(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_DisableHighKeeperSDA(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if high keeper SDA is enabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_HIGH_KEEPER_SDA_DISABLED  The controller SDA high keeper disable
  * @retval HAL_I3C_HIGH_KEEPER_SDA_ENABLED   The controller SDA high keeper enable
  */
hal_i3c_high_keeper_sda_status_t HAL_I3C_CTRL_IsEnabledHighKeeperSDA(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_high_keeper_sda_status_t) LL_I3C_IsEnabledHighKeeperSDA(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set the SCL clock stalling configuration. All stall features not selected are disabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  stall_time_cycle Controller clock stall time in number of kernel clock cycles.
            This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @param  stall_features Features of the I3C controller to which the stall time will be applied.
            See @ref I3C_CTRL_STALL_FEATURE_DEFINITION, this parameter is a combination of the following values:
              @ref HAL_I3C_CTRL_STALL_ACK
              @ref HAL_I3C_CTRL_STALL_CCC
              @ref HAL_I3C_CTRL_STALL_TX
              @ref HAL_I3C_CTRL_STALL_RX
              @ref HAL_I3C_CTRL_STALL_I2C_ACK
              @ref HAL_I3C_CTRL_STALL_I2C_TX
              @ref HAL_I3C_CTRL_STALL_I2C_RX
              @ref HAL_I3C_CTRL_STALL_ALL
              @ref HAL_I3C_CTRL_STALL_NONE
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_SetConfigStallTime(const hal_i3c_handle_t *hi3c, uint32_t stall_time_cycle,
                                             uint32_t stall_features)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(stall_time_cycle <= 0xFF);
  ASSERT_DBG_PARAM((stall_features & HAL_I3C_CTRL_STALL_ALL) == stall_features);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_ConfigStallTime(I3C_GET_INSTANCE(hi3c), stall_time_cycle, stall_features);

  return HAL_OK;
}

/**
  * @brief  Retrieve the SCL clock stalling configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  stall_time_cycle  Controller clock stall time in number of kernel clock cycles.
  *                           This parameter must be a number between Min_Data=0x00 and Max_Data=0xFF.
  * @param  stall_features  Features of the I3C controller to which the stall time is applied.
                            See @ref I3C_CTRL_STALL_FEATURE_DEFINITION,
                            this parameter is a combination of the following values:
                              @ref HAL_I3C_CTRL_STALL_ACK
                              @ref HAL_I3C_CTRL_STALL_CCC
                              @ref HAL_I3C_CTRL_STALL_TX
                              @ref HAL_I3C_CTRL_STALL_RX
                              @ref HAL_I3C_CTRL_STALL_I2C_ACK
                              @ref HAL_I3C_CTRL_STALL_I2C_TX
                              @ref HAL_I3C_CTRL_STALL_I2C_RX
                              @ref HAL_I3C_CTRL_STALL_ALL
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_GetConfigStallTime(const hal_i3c_handle_t *hi3c, uint32_t *stall_time_cycle,
                                             uint32_t *stall_features)
{
  uint32_t timing2_value;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(stall_time_cycle != NULL);
  ASSERT_DBG_PARAM(stall_features != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((stall_time_cycle == NULL) || (stall_features == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  /* Get value from timing 2 register */
  timing2_value = LL_I3C_GetRegister_TIMINGR2(I3C_GET_INSTANCE(hi3c));
  *stall_time_cycle = (uint32_t)(timing2_value >> I3C_TIMINGR2_STALL_Pos);
  *stall_features = timing2_value & (uint32_t) HAL_I3C_CTRL_STALL_ALL;

  return HAL_OK;
}

/**
  * @brief  Set Controller-Role Request allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_EnableCtrlRoleRequest(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_EnableControllerRoleReq(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Set Controller-Role Request as not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DisableCtrlRoleRequest(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_DisableControllerRoleReq(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if Controller-Role Request is allowed or not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_TGT_CTRL_ROLE_DISABLED  Controller-Role disable
  * @retval HAL_I3C_TGT_CTRL_ROLE_ENABLED   Controller-Role enable
  */
hal_i3c_tgt_ctrl_role_status_t HAL_I3C_TGT_IsEnabledCtrlRoleRequest(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_tgt_ctrl_role_status_t) LL_I3C_IsEnabledControllerRoleReq(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set hand off delay allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_EnableHandOffDelay(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_EnableHandOffDelay(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Set hand off delay as not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DisableHandOffDelay(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_DisableHandOffDelay(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if hand off delay is allowed or not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_HANDOFF_DELAY_DISABLED  Handoff delay is disabled
  * @retval HAL_I3C_HANDOFF_DELAY_ENABLED   Handoff delay is enabled
  */
hal_i3c_handoff_delay_status_t HAL_I3C_TGT_IsEnabledHandOffDelay(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_handoff_delay_status_t) LL_I3C_IsEnabledHandOffDelay(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set the group address capability as supported.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_EnableGroupAddrCapability(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetGrpAddrHandoffSupport(p_i3cx, LL_I3C_HANDOFF_GRP_ADDR_SUPPORTED);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Set the group address capability as not supported.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DisableGroupAddrCapability(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetGrpAddrHandoffSupport(p_i3cx, LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if the group address capability is supported or not-supported.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_GRP_ADDR_CAPABILITY_DISABLED  Group address capability disable
  * @retval HAL_I3C_GRP_ADDR_CAPABILITY_ENABLED   Group address capability enable
  */
hal_i3c_grp_addr_capability_status_t HAL_I3C_TGT_IsEnabledGroupAddrCapability(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  if (LL_I3C_GetGrpAddrHandoffSupport(I3C_GET_INSTANCE(hi3c)) == LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED)
  {
    return HAL_I3C_GRP_ADDR_CAPABILITY_DISABLED;
  }
  else
  {
    return HAL_I3C_GRP_ADDR_CAPABILITY_ENABLED;
  }
}

/**
  * @brief  Set Hot-Join allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_EnableHotJoinRequest(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_EnableHotJoin(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Set Hot-Join as not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DisableHotJoinRequest(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_DisableHotJoin(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if Hot-Join is allowed or not-allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_HOT_JOIN_DISABLED  Hot-Join disable
  * @retval HAL_I3C_HOT_JOIN_ENABLED   Hot-Join enable
  */
hal_i3c_hot_join_status_t HAL_I3C_TGT_IsEnabledHotJoinRequest(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_hot_join_status_t) LL_I3C_IsEnabledHotJoin(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set IBI configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_SetConfigIBI(const hal_i3c_handle_t *hi3c, const hal_i3c_tgt_ibi_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I3C_IBI_PAYLOAD_SIZE(p_config->ibi_payload_size_byte));
  ASSERT_DBG_PARAM(IS_I3C_PENDING_READ(p_config->pending_read_mdb));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_ConfigNbIBIAddData(p_i3cx, (uint32_t)p_config->ibi_payload_size_byte);
  LL_I3C_SetPendingReadMDB(p_i3cx, (uint32_t)p_config->pending_read_mdb);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Get IBI configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_TGT_GetConfigIBI(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_ibi_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  p_config->ibi_payload_size_byte = (hal_i3c_tgt_payload_size_t)LL_I3C_GetConfigNbIBIAddData(p_i3cx);
  p_config->pending_read_mdb = (hal_i3c_tgt_read_mdb_status_t) LL_I3C_GetPendingReadMDB(p_i3cx);
}

/**
  * @brief  Enable IBI request.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_EnableIBI(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_EnableIBI(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Disable IBI request.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DisableIBI(const hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_DisableIBI(p_i3cx);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Check if IBI procedure is allowed or not allowed.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_TGT_IBI_DISABLED  IBI request disable
  * @retval HAL_I3C_TGT_IBI_ENABLED   IBI request enable
  */
hal_i3c_tgt_ibi_status_t HAL_I3C_TGT_IsEnabledIBI(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_tgt_ibi_status_t) LL_I3C_IsEnabledIBI(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set the max data size configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  max_read_data_size_byte  Maximum read length the target advertises.
  *                                  This parameter must be a number between Min_Data=0x0 and Max_Data=0xFFFF.
  * @param  max_write_data_size_byte  Maximum read length the target advertises.
  *                                   This parameter must be a number between Min_Data=0x0 and Max_Data=0xFFFF.
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_SetConfigMaxDataSize(const hal_i3c_handle_t *hi3c, uint32_t  max_read_data_size_byte,
                                              uint32_t  max_write_data_size_byte)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(max_read_data_size_byte <= 0xFFFF);
  ASSERT_DBG_PARAM(max_write_data_size_byte <= 0xFFFF);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetMaxWriteLength(p_i3cx, max_write_data_size_byte);
  LL_I3C_SetMaxReadLength(p_i3cx, max_read_data_size_byte);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Retrieve max data size configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_max_read_data_size_byte  Pointer to maximum read length the target advertises.
  * @param  p_max_write_data_size_byte  Pointer to maximum write length the target advertises.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_GetConfigMaxDataSize(const hal_i3c_handle_t *hi3c, uint32_t *p_max_read_data_size_byte,
                                              uint32_t *p_max_write_data_size_byte)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_max_read_data_size_byte != NULL);
  ASSERT_DBG_PARAM(p_max_write_data_size_byte != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_max_read_data_size_byte == NULL) || (p_max_write_data_size_byte == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  *p_max_write_data_size_byte = LL_I3C_GetMaxWriteLength(p_i3cx);
  *p_max_read_data_size_byte = LL_I3C_GetMaxReadLength(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Set the Max Data Speed configuration response for GETMXDS CCC.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_SetConfigGETMXDS(const hal_i3c_handle_t *hi3c,
                                          const hal_i3c_tgt_getmxds_config_t *p_config)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_I3C_MAX_SPEED_DATA(p_config->getmxds_format));
  ASSERT_DBG_PARAM(IS_I3C_HANDOFF_ACTIVITY_STATE(p_config->ctrl_handoff_activity));
  ASSERT_DBG_PARAM(IS_I3C_TSCO_TIME(p_config->data_turnaround_duration));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetConfigGETMXDS(p_i3cx,
                          (uint32_t)p_config->ctrl_handoff_activity,
                          (uint32_t)p_config->getmxds_format,
                          (uint32_t)p_config->data_turnaround_duration,
                          (uint32_t)p_config->max_read_turnaround);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Retrieve the Max Data Speed configuration response for GETMXDS CCC.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_config  Pointer to the configuration structure
  */
void HAL_I3C_TGT_GetConfigGETMXDS(const hal_i3c_handle_t *hi3c, hal_i3c_tgt_getmxds_config_t *p_config)
{
  uint32_t getmxdsr_value;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  getmxdsr_value = LL_I3C_GetRegister_GETMXDSR(I3C_GET_INSTANCE(hi3c));

  p_config->ctrl_handoff_activity = (hal_i3c_handoff_activity_state_t)(uint32_t)(getmxdsr_value & I3C_GETMXDSR_HOFFAS);
  p_config->getmxds_format = (hal_i3c_getmxds_format_t)(uint32_t)(getmxdsr_value & I3C_GETMXDSR_FMT);
  p_config->data_turnaround_duration = (hal_i3c_turnaround_time_tsco_t)(uint32_t)(getmxdsr_value & I3C_GETMXDSR_TSCO);
  p_config->max_read_turnaround = (uint8_t)((getmxdsr_value & I3C_GETMXDSR_RDTURN) >> I3C_GETMXDSR_RDTURN_Pos);
}

/**
  * @brief  Set the format of the response for GETMXDS CCC.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  format  GETMXDS format
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_SetConfigGETMXDS_Format(const hal_i3c_handle_t *hi3c, hal_i3c_getmxds_format_t format)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_MAX_SPEED_DATA(format));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetMaxDataSpeedFormat(p_i3cx, (uint32_t)format);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Get the format of the response for GETMXDS CCC.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_GETMXDS_FORMAT_1
  * @retval HAL_I3C_GETMXDS_FORMAT_2_LSB
  * @retval HAL_I3C_GETMXDS_FORMAT_2_MID
  * @retval HAL_I3C_GETMXDS_FORMAT_2_MSB
  */
hal_i3c_getmxds_format_t HAL_I3C_TGT_GetConfigGETMXDS_Format(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_getmxds_format_t) LL_I3C_GetMaxDataSpeedFormat(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set the activity state after Controller-Role handoff configuration.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  state Handoff activity state
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_SetConfigCtrlHandOffActivity(const hal_i3c_handle_t *hi3c,
                                                      hal_i3c_handoff_activity_state_t state)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_HANDOFF_ACTIVITY_STATE(state));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_Disable(p_i3cx);
  LL_I3C_SetHandoffActivityState(p_i3cx, (uint32_t)state);
  LL_I3C_Enable(p_i3cx);

  return HAL_OK;
}

/**
  * @brief  Get the Activity State after Controller-Role handoff.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_HANDOFF_ACTIVITY_STATE_0  Activity state 0 after handoff
  * @retval HAL_I3C_HANDOFF_ACTIVITY_STATE_1  Activity state 1 after handoff
  * @retval HAL_I3C_HANDOFF_ACTIVITY_STATE_2  Activity state 2 after handoff
  * @retval HAL_I3C_HANDOFF_ACTIVITY_STATE_3  Activity state 3 after handoff
  */
hal_i3c_handoff_activity_state_t HAL_I3C_TGT_GetConfigCtrlHandOffActivity(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_handoff_activity_state_t) LL_I3C_GetHandoffActivityState(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Set I3C bus devices configuration in DEVRX[].
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_desc Pointer to the configuration structure
  * @param  nb_device Number of devices to configure
  *         This parameter must be a value between Min_Data=1 and Max_Data=4
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_SetConfigBusDevices(const hal_i3c_handle_t *hi3c,
                                              const hal_i3c_ctrl_device_config_t *p_desc,
                                              uint32_t nb_device)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_desc != NULL);
  ASSERT_DBG_PARAM(IS_I3C_DEVICE(nb_device));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_desc == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Loop on the nb_device to be treated */
  for (uint32_t i = 0U; i < nb_device; i++)
  {
    ASSERT_DBG_PARAM(IS_I3C_DEVICE_INDEX(p_desc[i].device_index));
    ASSERT_DBG_PARAM(IS_I3C_DYNAMIC_ADDRESS(p_desc[i].tgt_dynamic_addr));
    ASSERT_DBG_PARAM(IS_I3C_CTRL_IBI_ACK(p_desc[i].ibi_ack));
    ASSERT_DBG_PARAM(IS_I3C_CTRL_ROLE_ACK(p_desc[i].ctrl_role_req_ack));
    ASSERT_DBG_PARAM(IS_I3C_CTRL_STOP_XFER(p_desc[i].ctrl_stop_transfer));
    ASSERT_DBG_PARAM(IS_I3C_CTRL_IBI_PAYLOAD(p_desc[i].ibi_payload));

    LL_I3C_SetDevrx(I3C_GET_INSTANCE(hi3c),
                    p_desc[i].device_index,
                    (uint32_t)p_desc[i].tgt_dynamic_addr,
                    (uint32_t)p_desc[i].ibi_ack,
                    (uint32_t)p_desc[i].ibi_payload,
                    (uint32_t)p_desc[i].ctrl_stop_transfer,
                    (uint32_t)p_desc[i].ctrl_role_req_ack);
  }

  return HAL_OK;
}

/**
  * @brief  Retrieve I3C bus devices configuration from DEVRX[].
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_desc  Pointer to the configuration structure
  * @param  nb_device  Number of devices to retrieve the configuration
  *         This parameter must be a value between Min_Data=1 and Max_Data=4
  */
void HAL_I3C_CTRL_GetConfigBusDevices(const hal_i3c_handle_t *hi3c,
                                      hal_i3c_ctrl_device_config_t *p_desc,
                                      uint32_t nb_device)
{
  uint32_t read_value;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_DEVICE(nb_device));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  /* Loop on the nb_device to be treated */
  for (uint32_t i = 0U; i < nb_device; i++)
  {
    ASSERT_DBG_PARAM((&p_desc[i]) != NULL);
    read_value = LL_I3C_GetDevrx(I3C_GET_INSTANCE(hi3c), i);
    p_desc[i].device_index       = (uint8_t)i;
    p_desc[i].tgt_dynamic_addr   = (uint8_t)(uint32_t)((read_value & I3C_DEVRX_DA) >> I3C_DEVRX_DA_Pos);
    p_desc[i].ibi_ack            = (hal_i3c_ctrl_ibi_ack_status_t)(uint32_t)(read_value & I3C_DEVRX_IBIACK);
    p_desc[i].ctrl_role_req_ack  = (hal_i3c_ctrl_role_ack_status_t)(uint32_t)(read_value & I3C_DEVRX_CRACK);
    p_desc[i].ctrl_stop_transfer = (hal_i3c_ctrl_stop_transfer_status_t)(uint32_t)(read_value & I3C_DEVRX_SUSP);
    p_desc[i].ibi_payload        = (hal_i3c_ctrl_ibi_payload_status_t)(uint32_t)(read_value & I3C_DEVRX_IBIDEN);
  }
}

/**
  * @brief  Enable the inserted reset pattern at the end of a frame.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_EnableResetPattern(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_EnableResetPattern(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Disable the inserted reset pattern at the end of a frame.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DisableResetPattern(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  LL_I3C_DisableResetPattern(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Check if the inserted reset pattern at the end of a frame is enabled or disabled.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_I3C_RESET_PATTERN_DISABLED  Standard STOP condition emitted at the end of a frame
  * @retval HAL_I3C_RESET_PATTERN_ENABLED  Reset pattern is inserted before the STOP condition of any emitted frame
  */
hal_i3c_reset_pattern_status_t HAL_I3C_CTRL_IsEnabledResetPattern(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return (hal_i3c_reset_pattern_status_t) LL_I3C_IsEnabledResetPattern(I3C_GET_INSTANCE(hi3c));
}

#if defined(USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register the controller transfer complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the controller transfer complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_RegisterTransferCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_ctrl_transfer_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the controller dynamic address assignment complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the controller dynamic address assignment complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_RegisterDAACpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_ctrl_daa_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the target request dynamic address callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the target request dynamic address callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_RegisterTgtReqDynAddrCallback(hal_i3c_handle_t *hi3c,
                                                        hal_i3c_req_dyn_addr_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_ctrl_tgt_req_dyn_addr_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the target transmission complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the target transmission complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_RegisterTxCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_tgt_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the target Reception complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the target reception complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_RegisterRxCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_tgt_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the target Hot-Join process complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the target Hot-Join process complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_RegisterHotJoinCallback(hal_i3c_handle_t *hi3c, hal_i3c_tgt_hot_join_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_tgt_hot_join_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the target/controller Notification event callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the target/controller Notification event callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_RegisterNotifyCallback(hal_i3c_handle_t *hi3c, hal_i3c_notify_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_notify_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the abort complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the abort complete callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_RegisterAbortCpltCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the error callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_callback  Pointer to the error callback function
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_RegisterErrorCallback(hal_i3c_handle_t *hi3c, hal_i3c_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->p_error_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Link the transmit DMA handle to the I3C handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  hdma  Pointer to a hal_dma_handle_t structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_SetTxDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->hdma_tx = hdma;
  hdma->p_parent = hi3c;

  return HAL_OK;
}

/**
  * @brief  Link the receive DMA handle to the I3C handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  hdma  Pointer to a hal_dma_handle_t structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_SetRxDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->hdma_rx = hdma;
  hdma->p_parent = hi3c;

  return HAL_OK;
}

/**
  * @brief  Link the CR DMA handle to the I3C handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  hdma  Pointer to a hal_dma_handle_t structure
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_SetTcDMA(hal_i3c_handle_t *hi3c, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi3c->hdma_tc = hdma;
  hdma->p_parent = hi3c;

  return HAL_OK;
}
#endif /* USE_HAL_I3C_DMA */
/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group3 Interrupt and callback functions

A set of functions allowing the management of the notification feature of the I3Cx peripheral:
 - HAL_I3C_CTRL_ActivateNotification() to activate the I3C notifications in controller mode.
 - HAL_I3C_CTRL_DeactivateNotification() to deactivate the I3C notifications in controller mode.
 - HAL_I3C_TGT_ActivateNotification() to activate the I3C notifications in target mode.
 - HAL_I3C_TGT_DeactivateNotification() to deactivate the I3C notifications in target mode.
  * @{
  */

/**
  * @brief  Activate the I3C notifications in controller mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  notifications  Notification. It can be a combination value of @ref I3C_CTRL_NOTIFICATION
  * @warning This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_ActivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_CTRL_NOTIFICATIONS(notifications));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  hi3c->listen = I3C_LISTEN_ENABLED;
  hi3c->p_isr_func = I3C_Ctrl_Event_ISR;
  LL_I3C_EnableIT(I3C_GET_INSTANCE(hi3c), (notifications | LL_I3C_IER_ERRIE));

  return HAL_OK;
}

/**
  * @brief  Deactivate the I3C notifications.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  notifications  Notification. It can be a combination value of @ref I3C_CTRL_NOTIFICATION
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_DeactivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_CTRL_NOTIFICATIONS(notifications));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_DisableIT(p_i3cx, (notifications | LL_I3C_IER_ERRIE));

  if (LL_I3C_GetEnabledIT(p_i3cx) == 0U)
  {
    hi3c->p_isr_func = NULL;
    hi3c->listen = I3C_LISTEN_DISABLED;
  }

  return HAL_OK;
}

/**
  * @brief  Activate the I3C notifications in target mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the retrieve data during broadcast CCC DEFTGTS and DEFGRPA.
  * @param  size_byte  Size of retrieved data.
  * @param  notifications  Notification. It can be a combination value of @ref I3C_TGT_NOTIFICATION
  * @note   If HAL_I3C_TGT_NOTIFICATION_DEFTGTS or HAL_I3C_TGT_NOTIFICATION_DEFGRPA is requested,
  *         p_data must be non-NULL and size_byte non-zero to capture the broadcast payload bytes.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_ActivateNotification(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte,
                                              uint32_t notifications)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_TGT_NOTIFICATIONS(notifications));
  ASSERT_DBG_PARAM((notifications & (HAL_I3C_TGT_NOTIFICATION_DEFTGTS | HAL_I3C_TGT_NOTIFICATION_DEFGRPA)) == 0U \
                   || (p_data != NULL && size_byte != 0U));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (((notifications & (HAL_I3C_TGT_NOTIFICATION_DEFTGTS | HAL_I3C_TGT_NOTIFICATION_DEFGRPA)) != 0U)
      && ((p_data == NULL) || (size_byte == 0U)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  hi3c->p_isr_func = I3C_Tgt_Event_ISR;
  hi3c->listen = I3C_LISTEN_ENABLED;

  if ((notifications & (HAL_I3C_TGT_NOTIFICATION_DEFTGTS | HAL_I3C_TGT_NOTIFICATION_DEFGRPA)) != 0U)
  {
    hi3c->p_rx_data = p_data;
    hi3c->rx_count_byte = size_byte;

    if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
    {
      hi3c->p_rx_func = &I3C_ReceiveByteTreatment_IT;
    }
    else
    {
      hi3c->p_rx_func = &I3C_ReceiveWordTreatment_IT;
    }
  }

  LL_I3C_EnableIT(p_i3cx, (notifications | LL_I3C_IER_ERRIE));

  return HAL_OK;
}


/**
  * @brief  Deactivate the I3C notifications.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  notifications  Notification. It can be a combination value of @ref I3C_TGT_NOTIFICATION
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_TGT_DeactivateNotification(hal_i3c_handle_t *hi3c, uint32_t notifications)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_TGT_NOTIFICATIONS(notifications));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);
  LL_I3C_DisableIT(p_i3cx, (notifications | LL_I3C_IER_ERRIE));

  if (LL_I3C_GetEnabledIT(p_i3cx) == 0U)
  {
    hi3c->p_isr_func = NULL;
    hi3c->listen = I3C_LISTEN_DISABLED;
    hi3c->global_state = HAL_I3C_STATE_IDLE;
  }

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group4 IRQ Handlers
  * @{
A set of functions to handle the I3C interruptions:
  - I3C Event IRQ Handler: HAL_I3C_EV_IRQHandler()
  - I3C Error IRQ Handler: HAL_I3C_ERR_IRQHandler()

Depending on the process function used, different callbacks might be triggered:

| Process API \ Callbacks                      | HAL_I3C_CTRL_DynAddrAssign_IT() |
|----------------------------------------------|:-------------------------------:|
| HAL_I3C_CTRL_DAACpltCallback()               |               x                 |
| HAL_I3C_CTRL_TgtReqDynAddrCallback()         |               x                 |
| HAL_I3C_ErrorCallback()                      |               x                 |

| Process API \ Callbacks                      | HAL_I3C_CTRL_Transfer_IT()      | HAL_I3C_CTRL_Transfer_DMA()     |
|----------------------------------------------|:-------------------------------:|:-------------------------------:|
| HAL_I3C_CTRL_TransferCpltCallback()          |               x                 |               x                 |
| HAL_I3C_NotifyCallback()*                    |               x                 |               x                 |
| HAL_I3C_ErrorCallback()                      |               x                 |               x                 |

| Process API \ Callbacks                      | HAL_I3C_TGT_Transmit_IT()       | HAL_I3C_TGT_Transmit_DMA()      |
|----------------------------------------------|:-------------------------------:|:-------------------------------:|
| HAL_I3C_TGT_TxCpltCallback()                 |               x                 |               x                 |
| HAL_I3C_NotifyCallback()*                    |               x                 |               x                 |
| HAL_I3C_ErrorCallback()                      |               x                 |               x                 |

| Process API \ Callbacks                      | HAL_I3C_TGT_Receive_IT()        | HAL_I3C_TGT_Receive_DMA()       |
|----------------------------------------------|:-------------------------------:|:-------------------------------:|
| HAL_I3C_TGT_RxCpltCallback()                 |               x                 |               x                 |
| HAL_I3C_NotifyCallback()*                    |               x                 |               x                 |
| HAL_I3C_ErrorCallback()                      |               x                 |               x                 |

| Process API \ Callbacks                      | HAL_I3C_TGT_HotJoinReq_IT()     |
|----------------------------------------------|:-------------------------------:|
| HAL_I3C_TGT_HotJoinCallback()                |               x                 |
| HAL_I3C_ErrorCallback()                      |               x                 |

| Process API \ Callbacks                                  |  HAL_I3C_TGT_ControlRoleReq_IT()  |
|----------------------------------------------------------|:---------------------------------:|
| HAL_I3C_NotifyCallback(HAL_I3C_TGT_NOTIFICATION_GETACCCR)|                 x                 |
| HAL_I3C_ErrorCallback()                                  |                 x                 |

| Process API \ Callbacks                                  |  HAL_I3C_TGT_IBIReq_IT()          |
|----------------------------------------------------------|:---------------------------------:|
| HAL_I3C_NotifyCallback(HAL_I3C_TGT_NOTIFICATION_IBIEND)  |                 x                 |
| HAL_I3C_ErrorCallback()                                  |                 x                 |

| Process API \ Callbacks                                  | HAL_I3C_Abort_IT()                |
|----------------------------------------------------------|:---------------------------------:|
| HAL_I3C_AbortCpltCallback()                              |                 x                 |

@note * HAL_I3C_NotifyCallback() is triggered if HAL_I3C_CTRL_ActivateNotification or HAL_I3C_TGT_ActivateNotification
        have been previously called in state HAL_I3C_STATE_IDLE
  */

/**
  * @brief  Handle I3C error interrupt request.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
void HAL_I3C_ERR_IRQHandler(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check on the error interrupt flag and source */
  if (LL_I3C_IsActiveMaskFlag_ERR(p_i3cx) != 0U)
  {
    LL_I3C_ClearFlag_ERR(p_i3cx);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    if (hi3c->global_state != HAL_I3C_STATE_ABORT)
    {
      I3C_GetErrorSources(hi3c);
    }
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

    I3C_ErrorTreatment(hi3c);
  }
}

/**
  * @brief  Handle I3C event interrupt request.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
void HAL_I3C_EV_IRQHandler(hal_i3c_handle_t *hi3c)
{
  const I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  uint32_t it_masks = LL_I3C_GetRegister_MISR(p_i3cx);

  /* I3C events treatment */
  if (hi3c->p_isr_func != NULL)
  {
    hi3c->p_isr_func(hi3c, it_masks);
  }
}

/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group5 FIFO flush functions
  * @{
A set of functions to flush FIFOs :
  - HAL_I3C_FlushAllFifos() to flush the content of all used FIFOs (Control, Status, Tx and Rx FIFO).
  - HAL_I3C_FlushTxFifo() to flush only the content of Tx FIFO.
  - HAL_I3C_FlushRxFifo() to flush only the content of Rx FIFO.
  - HAL_I3C_CTRL_FlushControlFifo() to flush only the content of Control FIFO.
  - HAL_I3C_CTRL_FlushStatusFifo() to flush only the content of Status FIFO.
  */

/**
  * @brief  Flush all I3C FIFOs content.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_FlushAllFifos(const hal_i3c_handle_t *hi3c)
{
  uint32_t cfgr_value;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  /* Flush the content of Tx and Rx Fifo */
  cfgr_value = (I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH);

  /* Check on the I3C mode: Control and status FIFOs available only with controller mode */
  if (hi3c->mode == HAL_I3C_MODE_CTRL)
  {
    /* Flush the content of control and status Fifo */
    cfgr_value |= (I3C_CFGR_SFLUSH | I3C_CFGR_CFLUSH);
  }

  LL_I3C_RequestFifosFlush(I3C_GET_INSTANCE(hi3c), cfgr_value);

  return HAL_OK;
}

/**
  * @brief  Flush I3C Tx FIFO content.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_FlushTxFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_RequestTxFIFOFlush(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Flush I3C Rx FIFO content.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_FlushRxFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_RequestRxFIFOFlush(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Flush I3C control FIFO content.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_FlushControlFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_RequestControlFIFOFlush(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @brief  Flush I3C status FIFO content.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_FlushStatusFifo(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  LL_I3C_RequestStatusFIFOFlush(I3C_GET_INSTANCE(hi3c));

  return HAL_OK;
}

/**
  * @}
  */


/** @addtogroup I3C_Exported_Functions_Group6 Controller transfer operation functions
  * @{
 A set of functions that manage controller I3C transfer operations:
  - HAL_I3C_CTRL_ResetTransferCtx() to reset a transfer context.
  - HAL_I3C_CTRL_InitTransferCtxTc() to provide a ctrl buffer. It will be filled by
    HAL_I3C_CTRL_BuildTransferCtxPrivate() or HAL_I3C_CTRL_BuildTransferCtxCCC().
  - HAL_I3C_CTRL_InitTransferCtxTx() to provide the Tx buffer. It must be filled by application
    with concatenated Tx data.
  - HAL_I3C_CTRL_InitTransferCtxRx() to provide the Rx buffer.
  - HAL_I3C_CTRL_BuildTransferCtxPrivate() or HAL_I3C_CTRL_BuildTransferCtxCCC() to build transfer context
  - HAL_I3C_CTRL_Transfer() to start transfer I3C or I2C private data or CCC command in multiple direction
    in polling mode.
  - HAL_I3C_CTRL_Transfer_IT() to start transfer I3C or I2C private data or CCC command in multiple direction
    in interrupt mode.
  - HAL_I3C_CTRL_Transfer_DMA() to start transfer I3C or I2C private data or CCC command in multiple direction
    in DMA mode.
  - HAL_I3C_CTRL_DynAddrAssign() to send a broadcast ENTDAA CCC command in polling mode.
  - HAL_I3C_CTRL_DynAddrAssign_IT() to send a broadcast ENTDAA CCC command in interrupt mode.
  - HAL_I3C_CTRL_SetDynAddr() to set, associate the target dynamic address during the Dynamic Address Assignment
    process.
  - HAL_I3C_CTRL_PoolForDeviceI3cReady() to check if I3C target device is ready.
  - HAL_I3C_CTRL_PoolForDeviceI2cReady() to check if I2C target device is ready.
  - HAL_I3C_CTRL_GeneratePatterns() to send HDR exit pattern or target reset pattern.
  - HAL_I3C_CTRL_GenerateArbitration() to send arbitration (message header {S + 0x7E + W + STOP}) in polling mode.
  - HAL_I3C_CTRL_RecoverSCLToIDLE() to force the stop of the SCL clock.
  */


/**
  * @brief  Reset a controller transfer context.
  * @param  p_ctx  Pointer to the transfer context
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid  parameter
  */
hal_status_t HAL_I3C_CTRL_ResetTransferCtx(hal_i3c_transfer_ctx_t *p_ctx)
{
  ASSERT_DBG_PARAM(p_ctx != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_ctx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->p_tc_data = NULL;
  p_ctx->tc_size_word = 0U;
  p_ctx->p_tc_data = NULL;
  p_ctx->tc_size_word = 0U;
  p_ctx->p_tx_data = NULL;
  p_ctx->tx_size_byte = 0U;
  p_ctx->p_rx_data = NULL;
  p_ctx->rx_size_byte = 0U;
  p_ctx->transfer_mode = (hal_i3c_transfer_mode_t) 0U;
  p_ctx->nb_tx_frame = 0U;

  return HAL_OK;
}


/**
  * @brief  Initialize the transfer context with pointer to Transmit Control (TC) descriptor words buffer.
  * @param  p_ctx  Pointer to the transfer context
  * @param  p_ctrl_buf Pointer to the Transmit Control (TC) buffer.
  * @param  size_word  Size in word of the Transmit Control (TC) buffer.
  *                    size word = 2* number of description in case of direct CCC transfers
  *                    size word = number of description for all other transfers
  *                    Use helper macro @ref HAL_I3C_GET_CTRL_BUFFER_SIZE_WORD
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_InitTransferCtxTc(hal_i3c_transfer_ctx_t *p_ctx, uint32_t *p_ctrl_buf, uint32_t size_word)
{
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM(p_ctrl_buf != NULL);
  ASSERT_DBG_PARAM(size_word != 0U);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL) || (p_ctrl_buf == NULL) || (size_word == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->p_tc_data = p_ctrl_buf;
  p_ctx->tc_size_word = size_word;

  return hal_status;
}

/**
  * @brief  Initialize the transfer context with Tx data.
  * @param  p_ctx  Pointer to the transfer context
  * @param  p_tx_data  Pointer to the cumulated Tx buffer (@ref HAL_I3C_DIRECTION_WRITE)
  * @param  size_byte  Size in byte of the cumulated Tx data
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_InitTransferCtxTx(hal_i3c_transfer_ctx_t *p_ctx, const uint8_t *p_tx_data,
                                            uint32_t size_byte)
{
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM(p_tx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL) || (p_tx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->p_tx_data = p_tx_data;
  p_ctx->tx_size_byte = size_byte;

  return hal_status;
}

/**
  * @brief  Initialize the transfer context with Rx data.
  * @param  p_ctx  Pointer to the transfer context
  * @param  p_rx_data  Pointer to the cumulated Rx buffer (@ref HAL_I3C_DIRECTION_READ)
  * @param  size_byte  Size in byte of the cumulated Rx data
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_InitTransferCtxRx(hal_i3c_transfer_ctx_t *p_ctx, uint8_t *p_rx_data, uint32_t size_byte)
{
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM(p_rx_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL) || (p_rx_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->p_rx_data = p_rx_data;
  p_ctx->rx_size_byte = size_byte;

  return hal_status;
}

/**
  * @brief  Build a transfer context from private transfer descriptors.
  * @param  p_ctx  Pointer to the transfer context
  * @param  p_desc  Pointer to the private transfer descriptor table.
  * @param  nb_desc  The number private transfer descriptor.
  * @param  mode  Transfer mode. It must be one of PRIVATE mode from @ref hal_i3c_transfer_mode_t
  * @note   Preconditions on p_ctx (all must be satisfied before calling this function):
  *         1. HAL_I3C_CTRL_ResetTransferCtx() was called (context cleared).
  *         2. Control buffer provided via HAL_I3C_CTRL_InitTransferCtxTc():
  *              - p_ctx->p_tc_data != NULL
  *              - p_ctx->tc_size_word == nb_desc (exactly one control word per private descriptor)
  *         3. If any descriptor has direction HAL_I3C_DIRECTION_WRITE:
  *              - HAL_I3C_CTRL_InitTransferCtxTx() was called
  *              - p_ctx->p_tx_data != NULL
  *              - p_ctx->tx_size_byte == sum of data_size_byte for all WRITE descriptors
  *         4. If any descriptor has direction HAL_I3C_DIRECTION_READ:
  *              - HAL_I3C_CTRL_InitTransferCtxRx() was called
  *              - p_ctx->p_rx_data != NULL
  *              - p_ctx->rx_size_byte == sum of data_size_byte for all READ descriptors
  *         5. If total write size or read size is zero, the corresponding pointer can be NULL.
  *         6. mode must satisfy IS_I3C_PRIVATE_MODE(mode).
  *         7. Function neither allocates nor modifies user data buffers; it only fills control words.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_BuildTransferCtxPrivate(hal_i3c_transfer_ctx_t *p_ctx, const hal_i3c_private_desc_t *p_desc,
                                                  uint32_t nb_desc, hal_i3c_transfer_mode_t mode)
{
  hal_status_t hal_status = HAL_OK;
  uint32_t stop_condition;
#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
  uint32_t tx_cumul_size_byte = 0U;
  uint32_t rx_cumul_size_byte = 0U;
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM(p_desc != NULL);
  ASSERT_DBG_PARAM(nb_desc != 0U);
  ASSERT_DBG_PARAM(IS_I3C_PRIVATE_MODE(mode));
  ASSERT_DBG_PARAM(p_ctx->p_tc_data != NULL);
  ASSERT_DBG_PARAM(p_ctx->tc_size_word == nb_desc);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_desc == NULL) || (nb_desc == 0U) || (p_ctx == NULL) || (p_ctx->p_tc_data == NULL)
      || (p_ctx->tc_size_word != nb_desc))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->transfer_mode = mode;
  stop_condition = (uint32_t)mode & (uint32_t)I3C_RESTART_STOP_MASK;
  /*-------------------------------------------- Private -------------------------------------------------------------*/
  for (uint32_t i = 0U; i < nb_desc; i++)
  {
    uint32_t data_size_byte = p_desc[i].data_size_byte;
    uint32_t direction = (uint32_t) p_desc[i].direction;

    /* At the end, generate a stop condition */
    if (i == (nb_desc - 1U))
    {
      stop_condition = LL_I3C_GENERATE_STOP;
    }

    /* Update control buffer value */
    p_ctx->p_tc_data[i] = (data_size_byte | direction | ((uint32_t)p_desc[i].tgt_addr << I3C_CR_ADD_Pos)
                           | ((uint32_t)mode & (uint32_t)I3C_OPERATION_TYPE_MASK) | stop_condition);

#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
    if (direction == (uint32_t) HAL_I3C_DIRECTION_WRITE)
    {
      tx_cumul_size_byte += data_size_byte;
    }
    else
    {
      rx_cumul_size_byte += data_size_byte;
    }
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */
  }

  ASSERT_DBG_PARAM(p_ctx->tx_size_byte == tx_cumul_size_byte);
  ASSERT_DBG_PARAM(p_ctx->rx_size_byte == rx_cumul_size_byte);
  ASSERT_DBG_PARAM((tx_cumul_size_byte == 0U) || (p_ctx->p_tx_data != NULL));
  ASSERT_DBG_PARAM((rx_cumul_size_byte == 0U) || (p_ctx->p_rx_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx->tx_size_byte != tx_cumul_size_byte)
      || (p_ctx->rx_size_byte != rx_cumul_size_byte)
      || ((tx_cumul_size_byte != 0U) && (p_ctx->p_tx_data == NULL))
      || ((rx_cumul_size_byte != 0U) && (p_ctx->p_rx_data == NULL)))
  {
    hal_status = HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  return hal_status;
}

/**
  * @brief  Build a transfer context from CCC transfer descriptor.
  * @param  p_desc  Pointer to the CCC transfer descriptor table
  * @param  nb_desc  The number CCC transfer descriptor
  * @param  p_ctx  Pointer to the transfer context
  * @param  mode  Transfer mode. It must be one of CCC mode from @ref hal_i3c_transfer_mode_t
  * @note   Preconditions on p_ctx (all must be satisfied before calling this function):
  *         1. HAL_I3C_CTRL_ResetTransferCtx() was called (context cleared).
  *         2. Control buffer provided via HAL_I3C_CTRL_InitTransferCtxTc():
  *              - p_ctx->p_tc_data != NULL
  *              - For broadcast CCC: p_ctx->tc_size_word >= nb_desc (1 word per descriptor)
  *              - For direct CCC: p_ctx->tc_size_word >= (2 * nb_desc) (2 words per descriptor: CCC + target)
  *            After build, driver overwrites p_ctx->tc_size_word with the exact used size (nb_desc or 2*nb_desc).
  *         3. Tx buffer requirements:
  *              - If any descriptor has direction HAL_I3C_DIRECTION_WRITE (broadcast or direct write):
  *                  HAL_I3C_CTRL_InitTransferCtxTx() was called
  *                  p_ctx->p_tx_data != NULL
  *                  p_ctx->tx_size_byte ==
  *                     (sum of data_size_byte for all WRITE descriptors) +
  *                     (sum of nb_define_bytes for all READ descriptors using a define byte)
  *         4. Rx buffer requirements:
  *              - If any descriptor has direction HAL_I3C_DIRECTION_READ (direct read CCC only):
  *                  HAL_I3C_CTRL_InitTransferCtxRx() was called
  *                  p_ctx->p_rx_data != NULL
  *                  p_ctx->rx_size_byte ==
  *                     sum over READ descriptors of (data_size_byte - nb_define_bytes)
  *         5. If total computed tx_size_byte or rx_size_byte is zero, the corresponding pointer can be NULL.
  *         6. mode must satisfy IS_I3C_CCC_MODE(mode).
  *         7. Function only fills control words; it does not allocate or modify user data buffers.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_BuildTransferCtxCCC(hal_i3c_transfer_ctx_t *p_ctx, const hal_i3c_ccc_desc_t *p_desc,
                                              uint32_t nb_desc, hal_i3c_transfer_mode_t mode)
{
  hal_status_t hal_status = HAL_OK;
  uint32_t stop_condition;
#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
  uint32_t tx_cumul_size_byte = 0U;
  uint32_t rx_cumul_size_byte = 0U;
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(p_desc != NULL);
  ASSERT_DBG_PARAM(nb_desc != 0U);
  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM(p_ctx->p_tc_data != NULL);
  ASSERT_DBG_PARAM(IS_I3C_CCC_MODE(mode));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_desc == NULL) || (nb_desc == 0U) || (p_ctx == NULL) || (p_ctx->p_tc_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_ctx->nb_tx_frame = 0U;
  stop_condition = (uint32_t)mode & (uint32_t)I3C_RESTART_STOP_MASK;
  p_ctx->transfer_mode = mode;

  if (((uint32_t)mode & (uint32_t)I3C_OPERATION_TYPE_MASK) == LL_I3C_CONTROLLER_MTYPE_CCC)
  {
    /*------------------------------------------ Broadcast CCC -------------------------------------------------------*/
    ASSERT_DBG_PARAM(p_ctx->tc_size_word >= nb_desc);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
    if (p_ctx->tc_size_word !=  nb_desc)
    {
      return HAL_INVALID_PARAM;
    }
#endif /* USE_HAL_CHECK_PARAM */

    p_ctx->tc_size_word = nb_desc;

    for (uint32_t i = 0U; i < nb_desc; i++)
    {
      ASSERT_DBG_PARAM(IS_I3C_DIRECTION_CCC_BROADCAST(p_desc[i].direction));
      uint32_t data_size_byte = p_desc[i].data_size_byte;

      /* Only HAL_I3C_DIRECTION_WRITE is allowed */
#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
      tx_cumul_size_byte += data_size_byte;
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */
      p_ctx->nb_tx_frame++;

      /* Generate a stop condition at the end */
      if (i == (nb_desc - 1U))
      {
        stop_condition = LL_I3C_GENERATE_STOP;
      }

      /* Update control buffer value */
      p_ctx->p_tc_data[i] = data_size_byte | ((uint32_t)p_desc[i].ccc << I3C_CR_CCC_Pos)
                            | LL_I3C_CONTROLLER_MTYPE_CCC | stop_condition;
    }
  }
  else if (((uint32_t)mode & (uint32_t)I3C_OPERATION_TYPE_MASK) == LL_I3C_CONTROLLER_MTYPE_DIRECT)
  {
    /*------------------------------------------ Direct CCC ----------------------------------------------------------*/
    ASSERT_DBG_PARAM(p_ctx->tc_size_word >= (2U * nb_desc));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
    if (p_ctx->tc_size_word != (2U * nb_desc))
    {
      return HAL_INVALID_PARAM;
    }
#endif /* USE_HAL_CHECK_PARAM */

    uint32_t nb_define_bytes = ((uint32_t)mode & (uint32_t)I3C_DEFINE_BYTE_MASK);  /* 0 or 1 */
    p_ctx->tc_size_word = 2U * nb_desc;

    uint32_t i = 0U;
    uint32_t double_i = 0U;
    while (i < nb_desc)
    {
      uint32_t direction = (uint32_t)p_desc[i].direction;
      uint32_t data_size_byte = p_desc[i].data_size_byte;
      ASSERT_DBG_PARAM(IS_I3C_DIRECTION(direction));

      if (direction == (uint32_t) HAL_I3C_DIRECTION_WRITE)
      {
#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
        tx_cumul_size_byte += data_size_byte;
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */
        p_ctx->nb_tx_frame++;
      }
      else  /* direction == HAL_I3C_DIRECTION_READ */
      {
#if defined(USE_ASSERT_DBG_PARAM) || (defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1))
        tx_cumul_size_byte += nb_define_bytes;
        rx_cumul_size_byte += (data_size_byte - nb_define_bytes);
#endif /* USE_ASSERT_DBG_PARAM | USE_HAL_CHECK_PARAM */
      }

      /* Generate a stop condition at the end */
      if (i == (nb_desc - 1U))
      {
        stop_condition = LL_I3C_GENERATE_STOP;
      }

      /* Update control buffer value for the CCC command */
      p_ctx->p_tc_data[double_i] = nb_define_bytes | ((uint32_t)p_desc[i].ccc << I3C_CR_CCC_Pos)
                                   | LL_I3C_CONTROLLER_MTYPE_CCC | LL_I3C_GENERATE_RESTART;

      /* Update control buffer value for target address */
      p_ctx->p_tc_data[double_i + 1U] = (data_size_byte - nb_define_bytes) | direction |
                                        ((uint32_t)p_desc[i].tgt_addr << I3C_CR_ADD_Pos)
                                        | LL_I3C_CONTROLLER_MTYPE_DIRECT | stop_condition;
      i++;
      double_i += 2U;
    }
  }

  ASSERT_DBG_PARAM(p_ctx->tx_size_byte == tx_cumul_size_byte);
  ASSERT_DBG_PARAM(p_ctx->rx_size_byte == rx_cumul_size_byte);
  ASSERT_DBG_PARAM((tx_cumul_size_byte == 0U) || (p_ctx->p_tx_data != NULL));
  ASSERT_DBG_PARAM((rx_cumul_size_byte == 0U) || (p_ctx->p_rx_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx->tx_size_byte != tx_cumul_size_byte)
      || (p_ctx->rx_size_byte != rx_cumul_size_byte)
      || ((tx_cumul_size_byte != 0U) && (p_ctx->p_tx_data == NULL))
      || ((rx_cumul_size_byte != 0U) && (p_ctx->p_rx_data == NULL)))
  {
    hal_status = HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  return hal_status;
}

/**
  * @brief  Start transfer Direct CCC Command, I3C private or I2C transfer in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_ctx  Pointer to a hal_i3c_transfer_ctx_t structure. It can be reused to start again the
                   same transfer. This transfer context is built by @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() or
  *                @ref HAL_I3C_CTRL_BuildTransferCtxCCC().
  * @param  timeout_ms  Timeout duration in milliseconds
  * @note   The function @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() must be called before initiating a private
  *         transfer or the function @ref HAL_I3C_CTRL_BuildTransferCtxCCC() must be called before initiating a
  *         CCC transfer.
  * @note   The Tx FIFO threshold @ref HAL_I3C_TX_FIFO_THRESHOLD_1_2 is not allowed when the transfer descriptor
  *         contains multiple CCC direct frames.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_Transfer(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t tickstart;
  uint32_t exit_condition;
  uint32_t it_source;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM((p_ctx->rx_size_byte == 0U) || (p_ctx->p_rx_data != NULL));
  ASSERT_DBG_PARAM((p_ctx->tx_size_byte == 0U) || (p_ctx->p_tx_data != NULL));
  ASSERT_DBG_PARAM(p_ctx->p_tc_data != NULL);
  ASSERT_DBG_PARAM(p_ctx->tc_size_word != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL)
      || ((p_ctx->p_rx_data == NULL) && (p_ctx->rx_size_byte != 0U))
      || ((p_ctx->p_tx_data == NULL) && (p_ctx->tx_size_byte != 0U))
      || (p_ctx->p_tc_data == NULL)
      || (p_ctx->tc_size_word == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if ((LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_2)
      && (p_ctx->nb_tx_frame > 1U)
      && (((uint32_t)p_ctx->transfer_mode & I3C_OPERATION_TYPE_MASK) == LL_I3C_CONTROLLER_MTYPE_DIRECT))
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX_RX);

  /* Disable notification IT */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);
  LL_I3C_DisableIT(p_i3cx, it_source);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_tc_data = p_ctx->p_tc_data;
  hi3c->p_tx_data = p_ctx->p_tx_data;
  hi3c->p_rx_data = p_ctx->p_rx_data;
  hi3c->tc_count_word = p_ctx->tc_size_word;
  hi3c->tx_count_byte = p_ctx->tx_size_byte;
  hi3c->rx_count_byte = p_ctx->rx_size_byte;

  /* Check on the deactivation of the arbitration */
  if (((uint32_t)p_ctx->transfer_mode & I3C_ARBITRATION_HEADER_MASK) == I3C_ARBITRATION_HEADER_MASK)
  {
    LL_I3C_DisableArbitrationHeader(p_i3cx);
  }
  else
  {
    LL_I3C_EnableArbitrationHeader(p_i3cx);
  }

  /* Check on the Tx threshold to know the Tx treatment process: byte or word */
  if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_tx_func = &I3C_TransmitByteTreatment;
  }
  else
  {
    hi3c->p_tx_func = &I3C_TransmitWordTreatment;
  }

  /* Check on the Rx threshold to know the Rx treatment process: byte or word */
  if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_rx_func = &I3C_ReceiveByteTreatment;
  }
  else
  {
    hi3c->p_rx_func = &I3C_ReceiveWordTreatment;
  }

  tickstart = HAL_GetTick();

  if (LL_I3C_IsEnabledControlFIFO(p_i3cx) != 0U)
  {
    /* Initiate a Start condition */
    LL_I3C_RequestTransfer(p_i3cx);
  }
  else
  {
    LL_I3C_WriteControlWord(p_i3cx, *hi3c->p_tc_data);
    hi3c->p_tc_data++;
    hi3c->tc_count_word--;
  }

  /* Do while until FC (Frame Complete) is 1U or timeout */
  do
  {
    I3C_ControlDataTreatment(hi3c);
    hi3c->p_tx_func(hi3c);
    hi3c->p_rx_func(hi3c);

    /* Check for the timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
        break;
      }
    }

    if ((LL_I3C_IsActiveFlag_FC(p_i3cx) != 0U) && (hi3c->tc_count_word > 0U))
    {
      LL_I3C_ClearFlag_FC(p_i3cx);
      /* Then Initiate a Start condition */
      LL_I3C_RequestTransfer(p_i3cx);
    }

    /* Calculate exit_condition value based on frame complete or error flags */
    exit_condition = LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF);
  } while ((exit_condition  == 0U) || ((exit_condition != 0U) && (hi3c->tc_count_word > 0U)));

  LL_I3C_ClearFlag_FC(p_i3cx);

  if (hal_status == HAL_OK)
  {
    if ((hi3c->tx_count_byte != 0U) && (hi3c->rx_count_byte != 0U))
    {
      hal_status = HAL_ERROR;
    }

    if (LL_I3C_IsActiveFlag_ERR(p_i3cx) != 0U)
    {
      LL_I3C_ClearFlag_ERR(p_i3cx);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
      I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

      hal_status = HAL_ERROR;
    }
  }

  I3C_StateIdle(hi3c);

  /* Enable notification IT */
  LL_I3C_EnableIT(p_i3cx, it_source);

  return hal_status;
}

/**
  * @brief  Start transfer Direct CCC Command, I3C private or I2C transfer in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_ctx  Pointer to a hal_i3c_transfer_ctx_t structure. It can be reused to start again the
                   same transfer. This transfer context is filled by @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() or
  *                @ref HAL_I3C_CTRL_BuildTransferCtxCCC().
  * @note   The function @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() must be called before initiate a private transfer
  *         or the function @ref HAL_I3C_CTRL_BuildTransferCtxCCC() must be called before initiate a CCC transfer.
  * @note   The Tx FIFO threshold @ref HAL_I3C_TX_FIFO_THRESHOLD_1_2 is not allowed when the transfer descriptor
  *         contains multiple CCC direct frames.
  * @note   This function must be called to transfer read/write I3C or I2C private data or a direct read/write CCC.
  * @note   The tx_buf.size_byte must be equal to the sum of all tx_buf.size_byte exist in the descriptor.
  * @note   The rx_buf.size_byte must be equal to the sum of all rx_buf.size_byte exist in the descriptor.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_Transfer_IT(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM((p_ctx->rx_size_byte == 0U) || (p_ctx->p_rx_data != NULL));
  ASSERT_DBG_PARAM((p_ctx->tx_size_byte == 0U) || (p_ctx->p_tx_data != NULL));
  ASSERT_DBG_PARAM(p_ctx->p_tc_data != NULL);
  ASSERT_DBG_PARAM(p_ctx->tc_size_word != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL)
      || ((p_ctx->p_rx_data == NULL) && (p_ctx->rx_size_byte != 0U))
      || ((p_ctx->p_tx_data == NULL) && (p_ctx->tx_size_byte != 0U))
      || (p_ctx->p_tc_data == NULL)
      || (p_ctx->tc_size_word == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if ((LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_2)
      && (p_ctx->nb_tx_frame > 1U)
      && (((uint32_t)p_ctx->transfer_mode & I3C_OPERATION_TYPE_MASK) == LL_I3C_CONTROLLER_MTYPE_DIRECT))
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX_RX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  if (hi3c->listen == I3C_LISTEN_DISABLED)
  {
    hi3c->p_isr_func = I3C_Ctrl_Multiple_Xfer_ISR;
  }
  else
  {
    hi3c->p_isr_func = I3C_Ctrl_Multiple_Xfer_Listen_Event_ISR;
  }

  /* Check on the deactivation of the arbitration */
  if (((uint32_t)p_ctx->transfer_mode & I3C_ARBITRATION_HEADER_MASK) == I3C_ARBITRATION_HEADER_MASK)
  {
    LL_I3C_DisableArbitrationHeader(p_i3cx);
  }
  else
  {
    LL_I3C_EnableArbitrationHeader(p_i3cx);
  }

  hi3c->p_tc_data = p_ctx->p_tc_data;
  hi3c->p_tx_data = p_ctx->p_tx_data;
  hi3c->p_rx_data = p_ctx->p_rx_data;
  hi3c->tc_count_word = p_ctx->tc_size_word;
  hi3c->tx_count_byte = p_ctx->tx_size_byte;
  hi3c->rx_count_byte = p_ctx->rx_size_byte;

  /* Check on the Tx threshold to know the Tx treatment process: byte or word */
  if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_tx_func = &I3C_TransmitByteTreatment_IT;
  }
  else
  {
    hi3c->p_tx_func = &I3C_TransmitWordTreatment_IT;
  }

  /* Check on the Rx threshold to know the Rx treatment process: byte or word */
  if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_rx_func = &I3C_ReceiveByteTreatment_IT;
  }
  else
  {
    hi3c->p_rx_func = &I3C_ReceiveWordTreatment_IT;
  }

  LL_I3C_EnableIT(p_i3cx, (LL_I3C_CTRL_TX_IT | LL_I3C_CTRL_RX_IT));

  /* Initiate a Start condition */
  LL_I3C_RequestTransfer(p_i3cx);

  return hal_status;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Start transfer Direct CCC Command, I3C private or I2C transfer in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_ctx  Pointer to a hal_i3c_transfer_ctx_t structure. It can be reused to start again the
                   same transfer. This transfer context is filled by @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() or
  *                @ref HAL_I3C_CTRL_BuildTransferCtxCCC().
  * @note   The function @ref HAL_I3C_CTRL_BuildTransferCtxPrivate() must be called before initiate a private transfer
  *         or the function @ref HAL_I3C_CTRL_BuildTransferCtxCCC() must be called before initiate a CCC transfer.
  * @note   The Tx FIFO threshold @ref HAL_I3C_TX_FIFO_THRESHOLD_1_2 is not allowed when the transfer descriptor
  *         contains multiple CCC direct frames.
  * @note   The tx_buf.size_byte must be equal to the sum of all tx_buf.size_byte exist in the descriptor.
  * @note   The rx_buf.size_byte must be equal to the sum of all rx_buf.size_byte exist in the descriptor.
  * @note   This function must be called to transfer read/write private data or a direct read/write CCC command.
  * @note   DMA widths:
  *         - Tc: always words.
  *         - Tx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  *         - Rx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_Transfer_DMA(hal_i3c_handle_t *hi3c, const hal_i3c_transfer_ctx_t *p_ctx)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  hal_status_t control_dma_status;
  hal_status_t tx_dma_status = HAL_OK;
  hal_status_t rx_dma_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_ctx != NULL);
  ASSERT_DBG_PARAM((p_ctx->rx_size_byte == 0U) || (p_ctx->p_rx_data != NULL));
  ASSERT_DBG_PARAM((p_ctx->rx_size_byte == 0U) || (hi3c->hdma_rx != NULL));
  ASSERT_DBG_PARAM((p_ctx->tx_size_byte == 0U) || (p_ctx->p_tx_data != NULL));
  ASSERT_DBG_PARAM((p_ctx->tx_size_byte == 0U) || (hi3c->hdma_tx != NULL));
  ASSERT_DBG_PARAM(p_ctx->p_tc_data != NULL);
  ASSERT_DBG_PARAM(p_ctx->tc_size_word != 0U);
  ASSERT_DBG_PARAM(hi3c->hdma_tc != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ctx == NULL)
      || (((p_ctx->p_rx_data == NULL) || (hi3c->hdma_rx == NULL)) && (p_ctx->rx_size_byte != 0U))
      || (((p_ctx->p_tx_data == NULL) || (hi3c->hdma_tx == NULL)) && (p_ctx->tx_size_byte != 0U))
      || (p_ctx->p_tc_data == NULL)
      || (hi3c->hdma_tc == NULL)
      || (p_ctx->tc_size_word == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if ((LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_2)
      && (p_ctx->nb_tx_frame > 1U)
      && (((uint32_t)p_ctx->transfer_mode & I3C_OPERATION_TYPE_MASK) == LL_I3C_CONTROLLER_MTYPE_DIRECT))
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX_RX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_isr_func = I3C_Ctrl_Multiple_Xfer_DMA_ISR;

  /* Check on the deactivation of the arbitration */
  if (((uint32_t)p_ctx->transfer_mode & I3C_ARBITRATION_HEADER_MASK) == I3C_ARBITRATION_HEADER_MASK)
  {
    LL_I3C_DisableArbitrationHeader(p_i3cx);
  }
  else
  {
    LL_I3C_EnableArbitrationHeader(p_i3cx);
  }

  /*------------------------------------ I3C DMA channel for Control Data --------------------------------------------*/
  hi3c->hdma_tc->p_xfer_cplt_cb = I3C_DMAControlTransmitCplt;
  hi3c->hdma_tc->p_xfer_error_cb = I3C_DMAError;
  hi3c->hdma_tc->p_xfer_abort_cb = I3C_DMAError;

  control_dma_status = HAL_DMA_StartDirectXfer_IT_Opt(hi3c->hdma_tc, (uint32_t)p_ctx->p_tc_data,
                                                      (uint32_t)&p_i3cx->CR, p_ctx->tc_size_word * 4U,
                                                      HAL_DMA_OPT_IT_NONE);

  /*------------------------------------ I3C DMA channel for the Rx Data ---------------------------------------------*/
  if (hi3c->hdma_rx != NULL)
  {
    hi3c->hdma_rx->p_xfer_abort_cb = I3C_DMAError;
    if (p_ctx->rx_size_byte != 0U)
    {
      hi3c->hdma_rx->p_xfer_cplt_cb = I3C_DMADataReceiveCplt;
      hi3c->hdma_rx->p_xfer_error_cb = I3C_DMAError;

      /* Check on the Rx threshold to know the Rx treatment process: byte or word */
      if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
      {
        rx_dma_status = HAL_DMA_StartDirectXfer_IT_Opt(hi3c->hdma_rx, (uint32_t)&p_i3cx->RDR,
                                                       (uint32_t)p_ctx->p_rx_data, p_ctx->rx_size_byte,
                                                       HAL_DMA_OPT_IT_NONE);
      }
      else
      {
        rx_dma_status = HAL_DMA_StartDirectXfer_IT_Opt(hi3c->hdma_rx, (uint32_t)&p_i3cx->RDWR,
                                                       (uint32_t)p_ctx->p_rx_data, I3C_RoundUp4(p_ctx->rx_size_byte),
                                                       HAL_DMA_OPT_IT_NONE);
      }
    }
  }

  /*------------------------------------ I3C DMA channel for the Tx Data ---------------------------------------------*/
  if (hi3c->hdma_tx != NULL)
  {
    hi3c->hdma_tx->p_xfer_abort_cb = I3C_DMAError;
    if (p_ctx->tx_size_byte != 0U)
    {
      hi3c->hdma_tx->p_xfer_cplt_cb = I3C_DMADataTransmitCplt;
      hi3c->hdma_tx->p_xfer_error_cb = I3C_DMAError;

      /* Check on the Tx threshold to know the Tx treatment process: byte or word */
      if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
      {
        tx_dma_status = HAL_DMA_StartDirectXfer_IT_Opt(hi3c->hdma_tx, (uint32_t)p_ctx->p_tx_data,
                                                       (uint32_t)&p_i3cx->TDR, p_ctx->tx_size_byte,
                                                       HAL_DMA_OPT_IT_NONE);
      }
      else
      {
        tx_dma_status = HAL_DMA_StartDirectXfer_IT_Opt(hi3c->hdma_tx, (uint32_t)p_ctx->p_tx_data,
                                                       (uint32_t)&p_i3cx->TDWR, I3C_RoundUp4(p_ctx->tx_size_byte),
                                                       HAL_DMA_OPT_IT_NONE);
      }
    }
  }

  /* Check if DMA process is well started */
  uint32_t last_error_codes = hi3c->last_error_codes;
  if ((control_dma_status == HAL_OK) && (tx_dma_status == HAL_OK)
      && (rx_dma_status == HAL_OK) && (last_error_codes == HAL_I3C_ERROR_NONE))
  {
    LL_I3C_EnableIT(p_i3cx, LL_I3C_XFER_DMA);
    LL_I3C_EnableDMAReq_Control(p_i3cx);

    if (p_ctx->rx_size_byte != 0U)
    {
      LL_I3C_EnableDMAReq_RX(p_i3cx);
    }

    if (p_ctx->tx_size_byte != 0U)
    {
      LL_I3C_EnableDMAReq_TX(p_i3cx);
    }

    /* Initiate a Start condition */
    LL_I3C_RequestTransfer(p_i3cx);
  }
  else
  {
    /* Set callback to NULL if DMA started */
    if (HAL_DMA_Abort(hi3c->hdma_tc) == HAL_OK)
    {
      hi3c->hdma_tc->p_xfer_cplt_cb = NULL;
      hi3c->hdma_tc->p_xfer_error_cb = NULL;
    }

    /* Set callback to NULL if DMA started */
    if (hi3c->hdma_tx != NULL)
    {
      if (HAL_DMA_Abort(hi3c->hdma_tx) == HAL_OK)
      {
        hi3c->hdma_tx->p_xfer_cplt_cb = NULL;
        hi3c->hdma_tx->p_xfer_error_cb = NULL;
      }
    }

    /* Set callback to NULL if DMA started */
    if (hi3c->hdma_rx != NULL)
    {
      if (HAL_DMA_Abort(hi3c->hdma_rx) == HAL_OK)
      {
        hi3c->hdma_rx->p_xfer_cplt_cb = NULL;
        hi3c->hdma_rx->p_xfer_error_cb = NULL;
      }
    }
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    hi3c->last_error_codes = HAL_I3C_ERROR_DMA;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
    hal_status = HAL_ERROR;
    I3C_StateIdle(hi3c);
  }

  return hal_status;
}
#endif /* USE_HAL_I3C_DMA */
/**
  * @brief  Controller assigns dynamic addresses (broadcast ENTDAA CCC) in polling mode.
  * ENTDAA is an iterative bus procedure: each target responds in turn with its 48-bit payload,
  * after which the controller application must immediately associate a dynamic address via HAL_I3C_CTRL_SetDynAddr().
  * The hardware then automatically re-issues ENTDAA until "target not detected". This function must be called in loop
  * until p_target_detection_status indicates that no target is detected.
  * @param  hi3c   I3C handle (controller mode required).
  * @param  p_target_payload  Pointer receiving the 48-bit target payload (PID[32] | BCR[8] | DCR[8]).
  * @param  option HAL_I3C_DYN_ADDR_ONLY_ENTDAA or HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA.
  * @param  p_target_detection_status  Pointer to the target detection status.
  * @param  timeout_ms Timeout in milliseconds for internal flag waits.
  * @note Arbitration header is enabled.
  * @note While (*p_target_detection_status == HAL_I3C_TGT_DETECTED):
  *        - Retrieve the payload.
  *        - Call HAL_I3C_CTRL_SetDynAddr() to assign a dynamic address.
  *        - Call HAL_I3C_CTRL_DynAddrAssign() again to detect the next target.
  *       At the end of the DAA procedure (*p_target_detection_status == HAL_I3C_TGT_NOT_DETECTED):
  *        - Call HAL_I3C_CTRL_SetConfigBusDevices() to register capabilities.
  * @note Option HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA inserts an initial RSTDAA CCC frame before ENTDAA.
  * @retval HAL_OK       Operation completed successfully. The DAA process is completed if
  *                      (*p_target_detection_status == HAL_I3C_TGT_NOT_DETECTED).
  * @retval HAL_ERROR    Operation completed with error.
  * @retval HAL_TIMEOUT  Operation exceeds user timeout.
  * @retval HAL_BUSY     Concurrent process ongoing.
  */
hal_status_t HAL_I3C_CTRL_DynAddrAssign(hal_i3c_handle_t *hi3c, uint64_t *p_target_payload,
                                        hal_i3c_dyn_addr_opt_t option,
                                        hal_i3c_target_detection_status_t *p_target_detection_status,
                                        uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_target_payload != NULL);
  ASSERT_DBG_PARAM(IS_I3C_ENTDAA_OPTION(option));
  ASSERT_DBG_PARAM(p_target_detection_status != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_DAA);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_target_payload == NULL) || (p_target_detection_status == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  *p_target_detection_status = HAL_I3C_TGT_NOT_DETECTED;

  if (hi3c->global_state == HAL_I3C_STATE_IDLE)
  {
    HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_DAA);

    LL_I3C_EnableArbitrationHeader(p_i3cx);

    if (option == HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA)
    {
      /* Launch a RSTDAA procedure */
      LL_I3C_ControllerHandleCCC(p_i3cx, LL_I3C_BROADCAST_RSTDAA, 0UL, LL_I3C_GENERATE_STOP);

      hal_status = I3C_WaitForFlagSet(hi3c, LL_I3C_EVR_FCF, timeout_ms);
      LL_I3C_ClearFlag_FC(p_i3cx);
    }

    if (hal_status == HAL_OK)
    {
      /* Launch a ENTDAA procedure */
      LL_I3C_ControllerHandleCCC(p_i3cx, LL_I3C_BROADCAST_ENTDAA, 0UL, LL_I3C_GENERATE_STOP);
    }

  }

  if (hal_status == HAL_OK)
  {
#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
    if (hi3c->global_state != HAL_I3C_STATE_DAA)
    {
      return HAL_BUSY;
    }
#endif /* USE_HAL_CHECK_PROCESS_STATE */

    hal_status = I3C_WaitForFlagSet(hi3c, (LL_I3C_EVR_FCF | I3C_EVR_TXFNFF), timeout_ms);

    /* Check Tx FIFO is not full */
    if (LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U)
    {
      /* Check on the Rx FIFO threshold to know the Rx treatment process: byte or word */
      if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
      {
        for (uint32_t index = 0U; index < 8U; index++)
        {
          *p_target_payload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData8(p_i3cx) << (index * 8U));
        }
      }
      else
      {
        *p_target_payload = (uint64_t)LL_I3C_ReceiveData32(p_i3cx);
        *p_target_payload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData32(p_i3cx) << 32U);
      }

      *p_target_detection_status = HAL_I3C_TGT_DETECTED;
    }
    else
    {
      LL_I3C_ClearFlag_FC(p_i3cx);
    }
  }

  if ((hal_status != HAL_OK) || (*p_target_detection_status == HAL_I3C_TGT_NOT_DETECTED))
  {
    I3C_StateIdle(hi3c);
  }

  return hal_status;
}

/**
  * @brief  Controller assigns dynamic addresses (broadcast ENTDAA CCC) in interrupt mode.
  * ENTDAA is iterative: each target responds with a 48-bit payload (PID + BCR + DCR), after which
  * the controller application must immediately associate a dynamic address via HAL_I3C_CTRL_SetDynAddr().
  * The hardware continues issuing ENTDAA until no further target responds.
  * @param  hi3c   I3C handle (controller mode required).
  * @param  option HAL_I3C_DYN_ADDR_ONLY_ENTDAA or HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA.
  * @note Option HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA performs a preliminary RSTDAA CCC frame, then ENTDAA.
  * @note Arbitration header is forced enabled during ENTDAA.
  * @note After completion, call HAL_I3C_CTRL_SetConfigBusDevices() to register capabilities (addresses, IBI, etc.).
  * @retval HAL_OK    ENTDAA sequence accepted and configured; ISR will handle progress.
  * @retval HAL_BUSY  Concurrent process ongoing.
  */
hal_status_t HAL_I3C_CTRL_DynAddrAssign_IT(hal_i3c_handle_t *hi3c, hal_i3c_dyn_addr_opt_t option)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_ENTDAA_OPTION(option));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_DAA);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_isr_func = I3C_Ctrl_DAA_ISR;

  LL_I3C_EnableIT(p_i3cx, LL_I3C_CTRL_DAA_IT);
  LL_I3C_EnableArbitrationHeader(p_i3cx);

  /* Launch a RSTDAA procedure before launch ENTDAA */
  if (option == HAL_I3C_DYN_ADDR_RSTDAA_THEN_ENTDAA)
  {
    /* Write RSTDAA CCC information in the control register */
    LL_I3C_ControllerHandleCCC(p_i3cx, LL_I3C_BROADCAST_RSTDAA, 0UL, LL_I3C_GENERATE_RESTART);
  }
  else
  {
    /* Write ENTDAA CCC information in the control register */
    LL_I3C_ControllerHandleCCC(p_i3cx, LL_I3C_BROADCAST_ENTDAA, 0UL, LL_I3C_GENERATE_STOP);
  }

  return hal_status;
}

/**
  * @brief  Controller Set dynamic address.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  target_address  Value of the dynamic address to be assigned
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_ERROR  Operation completed with error
  */
hal_status_t HAL_I3C_CTRL_SetDynAddr(const hal_i3c_handle_t *hi3c, uint8_t target_address)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_DAA);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check if Tx FIFO requests data */
  if (LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U)
  {
    /* Write device address in the TDR register */
    LL_I3C_TransmitData8(p_i3cx, target_address);
  }
  else
  {
    hal_status = HAL_ERROR;
  }

  return hal_status;
}

/**
  * @brief  Check whether an I3C target device is ready.
  * @param  hi3c              Pointer to a @ref hal_i3c_handle_t
  * @param  target_address    Dynamic address of the target device
  * @param  timeout_ms        Polling timeout (in ms)
  * @retval HAL_OK       Frame complete (FCF) detected: target is ready.
  * @retval HAL_TIMEOUT  User timeout elapsed: target not ready
  * @retval HAL_ERROR    Internal failure while waiting for hardware flags
  */
hal_status_t HAL_I3C_CTRL_PoolForDeviceI3cReady(hal_i3c_handle_t *hi3c, uint8_t target_address, uint32_t timeout_ms)
{
  i3c_device_t device;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

  device.address = target_address;
  device.message_type = LL_I3C_CONTROLLER_MTYPE_PRIVATE;

  return I3C_Ctrl_PoolForDeviceReady(hi3c, &device, timeout_ms);
}

/**
  * @brief  Check whether an I2C target device is ready.
  * @param  hi3c              Pointer to a @ref hal_i3c_handle_t
  * @param  target_address    Address of the target device
  * @param  timeout_ms        Polling timeout (in ms)
  * @retval HAL_OK       Frame complete (FCF) detected: target is ready.
  * @retval HAL_TIMEOUT  User timeout elapsed: target not ready
  * @retval HAL_ERROR    Internal failure while waiting for hardware flags
  */
hal_status_t HAL_I3C_CTRL_PoolForDeviceI2cReady(hal_i3c_handle_t *hi3c, uint8_t target_address, uint32_t timeout_ms)
{
  i3c_device_t device;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

  device.address = target_address;
  device.message_type = LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C;

  return I3C_Ctrl_PoolForDeviceReady(hi3c, &device, timeout_ms);
}

/**
  * @brief  Controller generates patterns (target reset pattern or HDR exit pattern) with arbitration in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  pattern  The generated pattern.
  * @param  timeout_ms  Timeout duration in ms.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  */
hal_status_t HAL_I3C_CTRL_GeneratePatterns(hal_i3c_handle_t *hi3c, hal_i3c_pattern_opt_t pattern, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  __IO uint32_t exit_condition;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(IS_I3C_PATTERN(pattern));
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  /* The target reset pattern is sent after the issued message header */
  if (pattern == HAL_I3C_PATTERN_TGT_RESET)
  {
    LL_I3C_EnableResetPattern(p_i3cx);
    LL_I3C_DisableExitPattern(p_i3cx);
  }
  /* The HDR exit pattern is sent after the issued message header */
  else
  {
    LL_I3C_DisableResetPattern(p_i3cx);
    LL_I3C_EnableExitPattern(p_i3cx);
  }

  /* Write message control register */
  LL_I3C_ControllerHeaderStop(p_i3cx);

  /* Calculate exit_condition value based on frame complete or error flags */
  exit_condition = LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF);

  tickstart = HAL_GetTick();

  while (exit_condition == 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
        break;
      }
    }
    /* Calculate exit_condition value based on frame complete or error flags */
    exit_condition = LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF);
  }

  if (hal_status == HAL_OK)
  {
    if (LL_I3C_IsActiveFlag_FC(p_i3cx) != 0U)
    {
      LL_I3C_ClearFlag_FC(p_i3cx);
    }
    else
    {
      LL_I3C_ClearFlag_ERR(p_i3cx);
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
      I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
      hal_status = HAL_ERROR;
    }
  }

  I3C_StateIdle(hi3c);

  return hal_status;
}

/**
  * @brief  Controller generates arbitration (message header {S/Sr + 0x7E addr + W}) in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  timeout_ms  Timeout duration
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  */
hal_status_t HAL_I3C_CTRL_GenerateArbitration(hal_i3c_handle_t *hi3c, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  volatile uint32_t exit_condition;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  LL_I3C_DisableExitPattern(p_i3cx);
  LL_I3C_DisableResetPattern(p_i3cx);

  /* Write message control register */
  LL_I3C_ControllerHeaderStop(p_i3cx);

  /* Calculate exit_condition value based on frame complete or error flags */
  exit_condition = LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF);

  tickstart = HAL_GetTick();

  while (exit_condition == 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
        break;
      }
    }
    /* Calculate exit_condition value based on frame complete or error flags */
    exit_condition = LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF);
  }

  if (hal_status == HAL_OK)
  {
    if (LL_I3C_IsActiveFlag_FC(p_i3cx) != 0U)
    {
      LL_I3C_ClearFlag_FC(p_i3cx);
    }
    else
    {
      LL_I3C_ClearFlag_ERR(p_i3cx);
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
      I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
      hal_status = HAL_ERROR;
    }
  }

  I3C_StateIdle(hi3c);

  return hal_status;
}

/**
  * @brief  Recover the stuck SCL in case of CE1 error. It Forces the stop of the SCL clock.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @note   A minimum delay of 150 microseconds is required before emitting another message.
  *         This delay is approximately managed within this function.
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_CTRL_RecoverSCLToIDLE(const hal_i3c_handle_t *hi3c)
{
  hal_status_t hal_status = HAL_OK;
  volatile uint32_t wait_loop_index;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  /* Write message control register */
  LL_I3C_ControllerReleaseStop(I3C_GET_INSTANCE(hi3c));

  /**
    * Wait at least 150 micro seconds
    * Compute number of CPU cycles to wait for, using CMSIS global variable "SystemCoreClock".
    * Delay is approximate (depends on compilation optimization).
    * Computation: variable is divided by 2 to compensate partially CPU processing cycles of wait loop
    * (total shift right of 21 bits, including conversion from frequency in MHz).
    * If system core clock frequency is below 500kHz, delay is fulfilled by few CPU processing cycles.
    */
  wait_loop_index = ((150U * (SystemCoreClock >> 19U)) >> 2U);
  while (wait_loop_index != 0U)
  {
    wait_loop_index--;
  }

  return hal_status;
}
/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group7 Target operational functions
  * @{
A set of functions that manage target I3C operations:
  - HAL_I3C_TGT_Transmit() to transmit private data in polling mode.
  - HAL_I3C_TGT_Transmit_IT() to transmit private data in interrupt mode.
  - HAL_I3C_TGT_Transmit_DMA() to transmit private data in DMA mode.
  - HAL_I3C_TGT_Receive() to receive private data in polling mode.
  - HAL_I3C_TGT_Receive_IT() to receive private data in interrupt mode.
  - HAL_I3C_TGT_Receive_DMA() to receive private data in DMA mode.
  - HAL_I3C_TGT_ControlRoleReq() to send a control-role request in polling mode.
  - HAL_I3C_TGT_ControlRoleReq_IT() to send a control-role request in interrupt mode.
  - HAL_I3C_TGT_HotJoinReq() to send a Hot-Join request in polling mode.
  - HAL_I3C_TGT_HotJoinReq_IT() to send a Hot-Join request in interrupt mode.
  - HAL_I3C_TGT_IBIReq() to send an IBI request in polling mode.
  - HAL_I3C_TGT_IBIReq_IT() to send an IBI request in interrupt mode.
  */

/**
  * @brief  Target transmit private data in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @param  timeout_ms  Timeout duration in milliseconds
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   Target FIFO preload data is forced within this API for timing purpose.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Transmit(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte,
                                  uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t tickstart;
  uint32_t it_source;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

  /* Disable notification IT */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);
  LL_I3C_DisableIT(p_i3cx, it_source);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_tx_data = p_data;
  hi3c->tx_count_byte = size_byte;

  /* Check on the Tx threshold to know the Tx treatment process: byte or word */
  if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_tx_func = &I3C_TransmitByteTreatment;
  }
  else
  {
    hi3c->p_tx_func = &I3C_TransmitWordTreatment;
  }

  /* Set Preload information */
  LL_I3C_ConfigTxPreload(p_i3cx, (uint16_t)hi3c->tx_count_byte);

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /* Do while until FC (Frame Complete) is 1U or timeout */
  do
  {
    hi3c->p_tx_func(hi3c);

    /* Check for the timeout_ms */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
        break;
      }
    }
    /* Exit loop on Frame complete or error flags */
  } while (LL_I3C_GetFlag(p_i3cx, (LL_I3C_EVR_FCF | I3C_EVR_ERRF)) == 0U);

  LL_I3C_ClearFlag_FC(p_i3cx);

  /* Check if all data bytes are transmitted */
  if ((LL_I3C_GetXferDataCount(p_i3cx) != size_byte) && (hal_status == HAL_OK))
  {
    hal_status = HAL_ERROR;
  }

  if (LL_I3C_IsActiveFlag_ERR(p_i3cx) != 0U)
  {
    LL_I3C_ClearFlag_ERR(p_i3cx);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

    hal_status = HAL_ERROR;
  }

  I3C_StateIdle(hi3c);

  /* Enable notification IT */
  LL_I3C_EnableIT(p_i3cx, it_source);

  return hal_status;
}

/**
  * @brief  Target transmit private data in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   This function returns HAL_ERROR if (DEFIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFTGTS) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   This function returns HAL_ERROR if (GRPIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFGRPA) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Transmit_IT(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t it_source;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Read IER register */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);

  /* Check if DEF or GRP CCC notifications are enabled */
  if ((I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE)) != 0U)
      || (I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE)) != 0U))
  {
    return HAL_ERROR;
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_tx_data = p_data;
  hi3c->data_size_byte = size_byte;
  hi3c->tx_count_byte = size_byte;
  hi3c->p_isr_func = I3C_Tgt_Tx_ISR;

  /* Check on the Tx threshold to know the Tx treatment process: byte or word */
  if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_tx_func = &I3C_TransmitByteTreatment_IT;
  }
  else
  {
    hi3c->p_tx_func = &I3C_TransmitWordTreatment_IT;
  }

  LL_I3C_ConfigTxPreload(p_i3cx, (uint16_t)hi3c->tx_count_byte);
  LL_I3C_EnableIT(p_i3cx, LL_I3C_TGT_TX_IT);

  return hal_status;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Target transmit private data in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   This function returns HAL_ERROR if (DEFIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFTGTS) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   This function returns HAL_ERROR if (GRPIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFGRPA) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   DMA widths:
  *         - Tx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  *         - Rx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Transmit_DMA(hal_i3c_handle_t *hi3c, const uint8_t *p_data, uint32_t size_byte)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t it_source;
  hal_status_t tx_dma_status;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Read IER register */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);

  /* Check if DEF or GRP CCC notifications are enabled */
  if ((I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE)) != 0U)
      || (I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE)) != 0U))
  {
    return HAL_ERROR;
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_tx_data = p_data;
  hi3c->tx_count_byte = size_byte;
  hi3c->p_isr_func = I3C_Tgt_Tx_DMA_ISR;

  /* Set Preload information */
  LL_I3C_ConfigTxPreload(p_i3cx, (uint16_t)hi3c->tx_count_byte);

  /*------------------------------------ I3C DMA channel for the tx data ---------------------------------------------*/
  hi3c->hdma_tx->p_xfer_cplt_cb = I3C_DMADataTransmitCplt;
  hi3c->hdma_tx->p_xfer_error_cb = I3C_DMAError;
  hi3c->hdma_tx->p_xfer_abort_cb = I3C_DMAError;
  hi3c->hdma_rx->p_xfer_abort_cb = I3C_DMAError;

  /* Check on the Tx threshold to know the Tx treatment process: byte or word */
  if (LL_I3C_GetTxFIFOThreshold(p_i3cx) == LL_I3C_TXFIFO_THRESHOLD_1_8)
  {
    tx_dma_status = HAL_DMA_StartDirectXfer_IT(hi3c->hdma_tx, (uint32_t)hi3c->p_tx_data, (uint32_t)&p_i3cx->TDR,
                                               hi3c->tx_count_byte);
  }
  else
  {
    tx_dma_status = HAL_DMA_StartDirectXfer_IT(hi3c->hdma_tx, (uint32_t)hi3c->p_tx_data, (uint32_t)&p_i3cx->TDWR,
                                               I3C_RoundUp4(hi3c->tx_count_byte));
  }

  if (tx_dma_status == HAL_OK)
  {
    LL_I3C_EnableIT(p_i3cx, LL_I3C_XFER_DMA);
    hi3c->tx_count_byte = 0U;
    LL_I3C_EnableDMAReq_TX(p_i3cx);
  }
  else
  {
    hi3c->hdma_tx->p_xfer_cplt_cb = NULL;
    hi3c->hdma_tx->p_xfer_error_cb = NULL;

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    hi3c->last_error_codes = HAL_I3C_ERROR_DMA;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
    hal_status = HAL_ERROR;
    I3C_StateIdle(hi3c);
  }

  return hal_status;
}
#endif /* USE_HAL_I3C_DMA */

/**
  * @brief  Target receive private data in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @param  timeout_ms    Timeout duration in milliseconds
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   This function returns HAL_ERROR if (DEFIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFTGTS) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   This function returns HAL_ERROR if (GRPIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFGRPA) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Receive(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t it_source;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Read IER register */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);

  /* Check if DEF or GRP CCC notifications are enabled */
  if ((I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE)) != 0U)
      || (I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE)) != 0U))
  {
    return HAL_ERROR;
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_RX);

  /* Disable notification IT */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);
  LL_I3C_DisableIT(p_i3cx, it_source);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_rx_data = p_data;
  hi3c->rx_count_byte = size_byte;

  /* Check on the Rx threshold to know the Rx treatment process: byte or word */
  if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_rx_func = &I3C_ReceiveByteTreatment;
  }
  else
  {
    hi3c->p_rx_func = &I3C_ReceiveWordTreatment;
  }

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /* Do while until FC (Frame Complete) is 1U or timeout */
  do
  {
    if (hi3c->rx_count_byte > 0U)
    {
      hi3c->p_rx_func(hi3c);
    }

    /* Check for the timeout_ms */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
        break;
      }
    }
    /* Exit loop on Frame complete or error flags */
  } while (LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF) == 0U);

  LL_I3C_ClearFlag_FC(p_i3cx);

  /* Check if all data bytes are received */
  if ((LL_I3C_GetXferDataCount(p_i3cx) != size_byte) && (hal_status == HAL_OK))
  {
    hal_status = HAL_ERROR;
  }

  if (LL_I3C_IsActiveFlag_ERR(p_i3cx) != 0U)
  {
    LL_I3C_ClearFlag_ERR(p_i3cx);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

    hal_status = HAL_ERROR;
  }

  I3C_StateIdle(hi3c);

  /* Enable notification IT */
  LL_I3C_EnableIT(p_i3cx, it_source);

  return hal_status;
}

/**
  * @brief  Target receive private data in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   This function returns HAL_ERROR if (DEFIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFTGTS) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   This function returns HAL_ERROR if (GRPIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFGRPA) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Receive_IT(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t it_source;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Read IER register */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);

  /* Check if DEF or GRP CCC notifications are enabled */
  if ((I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE)) != 0U)
      || (I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE)) != 0U))
  {
    return HAL_ERROR;
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_RX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_rx_data = p_data;
  hi3c->data_size_byte = size_byte;
  hi3c->rx_count_byte = size_byte;
  hi3c->p_isr_func = I3C_Tgt_Rx_ISR;

  /* Check on the Rx threshold to know the Rx treatment process: byte or word */
  if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
  {
    hi3c->p_rx_func = &I3C_ReceiveByteTreatment_IT;
  }
  else
  {
    hi3c->p_rx_func = &I3C_ReceiveWordTreatment_IT;
  }

  LL_I3C_EnableIT(p_i3cx, LL_I3C_TGT_RX_IT);

  return hal_status;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Target receive private data in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_data  Pointer to the data
  * @param  size_byte  Size of the data in bytes
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @note   This function returns HAL_ERROR if (DEFIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFTGTS) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   This function returns HAL_ERROR if (GRPIE | RXFNEIE) are set,
  *         HAL_I3C_TGT_ActivateNotification_IT(HAL_I3C_TGT_NOTIFICATION_DEFGRPA) enables these interrupts.
  *         This prevents Rx FIFO contention and mixing CCC payload bytes with private Tx data.
  * @note   DMA widths:
  *         - Tx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  *         - Rx: threshold 1/8 -> bytes; threshold 1/2 -> words.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_Receive_DMA(hal_i3c_handle_t *hi3c, uint8_t *p_data, uint32_t size_byte)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t it_source;
  hal_status_t rx_dma_status;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(size_byte != 0U);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0U))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Read IER register */
  it_source =  LL_I3C_GetEnabledIT(p_i3cx);

  /* Check if DEF or GRP CCC notifications are enabled */
  if ((I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_DEFIE | LL_I3C_IER_RXFNEIE)) != 0U)
      || (I3C_CHECK_IT_SOURCE(it_source, (LL_I3C_IER_GRPIE | LL_I3C_IER_RXFNEIE)) != 0U))
  {
    return HAL_ERROR;
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_RX);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_rx_data = p_data;
  hi3c->rx_count_byte = size_byte;
  hi3c->p_isr_func = I3C_Tgt_Rx_DMA_ISR;

  /*------------------------------------ I3C DMA channel for the Rx Data ---------------------------------------------*/
  hi3c->hdma_rx->p_xfer_cplt_cb = I3C_DMADataReceiveCplt;
  hi3c->hdma_rx->p_xfer_error_cb = I3C_DMAError;
  hi3c->hdma_rx->p_xfer_abort_cb = I3C_DMAError;
  hi3c->hdma_tx->p_xfer_abort_cb = I3C_DMAError;

  /* Check on the Rx threshold to know the Rx treatment process: byte or word */
  if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
  {
    rx_dma_status = HAL_DMA_StartDirectXfer_IT(hi3c->hdma_rx, (uint32_t)&p_i3cx->RDR, (uint32_t)hi3c->p_rx_data,
                                               hi3c->rx_count_byte);
  }
  else
  {
    rx_dma_status = HAL_DMA_StartDirectXfer_IT(hi3c->hdma_rx, (uint32_t)&p_i3cx->RDWR, (uint32_t)hi3c->p_rx_data,
                                               I3C_RoundUp4(hi3c->rx_count_byte));
  }

  if (rx_dma_status == HAL_OK)
  {
    LL_I3C_EnableIT(p_i3cx, LL_I3C_XFER_DMA);
    hi3c->rx_count_byte = 0U;
    LL_I3C_EnableDMAReq_RX(p_i3cx);
  }
  else
  {
    hi3c->hdma_rx->p_xfer_cplt_cb = NULL;
    hi3c->hdma_rx->p_xfer_error_cb = NULL;
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    hi3c->last_error_codes = HAL_I3C_ERROR_DMA;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
    hal_status = HAL_ERROR;
    I3C_StateIdle(hi3c);
  }

  return hal_status;
}
#endif /* USE_HAL_I3C_DMA */

/**
  * @brief  Target sends Controller-Role request in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  timeout_ms    Timeout duration in milliseconds
  * @note   After receiving the controller's response to the Controller-Role request, the application must configure
  *         the I3C as a controller using the HAL_I3C_CTRL_SetConfig() function.
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  */
hal_status_t HAL_I3C_TGT_ControlRoleReq(hal_i3c_handle_t *hi3c, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  if (LL_I3C_IsEnabledControllerRoleReq(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  /* Request Controller-Role */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_CONTROLLER_ROLE_REQ, 0UL);

  /* Wait Controller-Role completion confirmation flag */
  hal_status = I3C_WaitForFlagSet(hi3c, LL_I3C_EVR_CRUPDF, timeout_ms);

  if (hal_status == HAL_OK)
  {
    LL_I3C_ClearFlag_CRUPD(p_i3cx);
  }

  I3C_StateIdle(hi3c);

  return hal_status;
}

/**
  * @brief  Target sends Controller-Role request in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @note   After receiving the controller's response to the Controller-Role request, the application must configure
  *         the I3C as a controller using the HAL_I3C_CTRL_SetConfig() function.
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  */
hal_status_t HAL_I3C_TGT_ControlRoleReq_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  if (LL_I3C_IsEnabledControllerRoleReq(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_isr_func = I3C_Tgt_CtrlRole_ISR;

  LL_I3C_EnableIT(p_i3cx, LL_I3C_TGT_CTRLROLE_IT);

  /* Request Controller-Role */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_CONTROLLER_ROLE_REQ, 0UL);

  return HAL_OK;
}

/**
  * @brief  Target sends Hot-Join request in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_own_dynamic_address  Pointer to the target own dynamic address assigned by the controller.
  * @param  timeout_ms  Timeout duration in milliseconds
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_HotJoinReq(hal_i3c_handle_t *hi3c, uint8_t *p_own_dynamic_address, uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_own_dynamic_address != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_own_dynamic_address == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if (LL_I3C_IsEnabledHotJoin(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  /* Request Hot-Join */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_HOT_JOIN, 0UL);

  /* Wait Hot-Join completion confirmation flag */
  hal_status = I3C_WaitForFlagSet(hi3c, LL_I3C_EVR_DAUPDF, timeout_ms);

  if (hal_status == HAL_OK)
  {
    /* Clear dynamic address update flag */
    LL_I3C_ClearFlag_DAUPD(p_i3cx);
  }

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    hi3c->last_error_codes |= HAL_I3C_ERROR_DYNAMIC_ADDR;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
    hal_status = HAL_ERROR;
  }
  else
  {
    *p_own_dynamic_address = LL_I3C_GetOwnDynamicAddress(p_i3cx);
  }

  I3C_StateIdle(hi3c);

  return hal_status;
}

/**
  * @brief  Target sends Hot-Join request in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  */
hal_status_t HAL_I3C_TGT_HotJoinReq_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check on the Hot-Join request feature */
  if (LL_I3C_IsEnabledHotJoin(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_isr_func = I3C_Tgt_HotJoin_ISR;

  LL_I3C_EnableIT(p_i3cx, LL_I3C_TGT_HOTJOIN_IT);

  /* Request Hot-Join */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_HOT_JOIN, 0UL);

  return HAL_OK;
}

/**
  * @brief  Target sends IBI request in polling mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_payload     Pointer to the buffer contains the payload data
  * @param  payload_size_byte  Payload buffer size in bytes
  * @param  timeout_ms    Timeout duration in milliseconds
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_IBIReq(hal_i3c_handle_t *hi3c,
                                const uint8_t *p_payload,
                                uint32_t payload_size_byte,
                                uint32_t timeout_ms)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status;
  uint32_t payload_value = 0U;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check the validity of the own dynamic address */
  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  if (LL_I3C_IsEnabledIBI(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  /* Update handle parameters */
  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

  /* Check on the IBI additional data */
  if (LL_I3C_GetDeviceIBIPayload(p_i3cx) == LL_I3C_IBI_ADDITIONAL_DATA)
  {
    ASSERT_DBG_PARAM(p_payload != NULL);
    ASSERT_DBG_PARAM(payload_size_byte != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
    /* Check on the p_payload and payload_size values */
    if ((p_payload == NULL) || (payload_size_byte == 0U))
    {
      I3C_StateIdle(hi3c);
      return HAL_INVALID_PARAM;
    }
#endif /* USE_HAL_CHECK_PARAM */

    /* For loop to calculate the payload value */
    for (uint32_t index = 0U; index < payload_size_byte; index++)
    {
      payload_value |= ((uint32_t)p_payload[index] << (index * 8U));
    }

    /* Load IBI payload data */
    LL_I3C_SetIBIPayload(p_i3cx, payload_value);
  }

  /* Request IBI */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_IBI, payload_size_byte);

  /* Wait IBI completion confirmation flag */
  hal_status = I3C_WaitForFlagSet(hi3c, LL_I3C_EVR_IBIENDF, timeout_ms);

  if (hal_status == HAL_OK)
  {
    /* Clear IBI end process flag */
    LL_I3C_ClearFlag_IBIEND(p_i3cx);
  }

  I3C_StateIdle(hi3c);

  return hal_status;
}

/**
  * @brief  Target sends IBI request in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_payload     Pointer to the buffer contains the payload data
  * @param  payload_size_byte  Payload buffer size in bytes
  * @note   The dynamic own address must be valid before calling this function.
  *         This function returns HAL_ERROR in these situations:
  *           - Before the ENTDAA (Dynamic Address Assignment) procedure completes.
  *           - Immediately after a RSTDAA (Reset Dynamic Address Assignment) broadcast.
  *           - Before a successful Hot-Join sequence (target not yet assigned a dynamic address).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  * @retval HAL_ERROR  Operation completed with error
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_TGT_IBIReq_IT(hal_i3c_handle_t *hi3c, const uint8_t *p_payload, uint32_t payload_size_byte)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t payload_value = 0U;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  if (LL_I3C_IsEnabledIBI(p_i3cx) == 0U)
  {
    return HAL_ERROR;
  }

  HAL_CHECK_UPDATE_STATE(hi3c, global_state, HAL_I3C_STATE_IDLE, HAL_I3C_STATE_TGT_REQ);

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
  hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
  hi3c->p_isr_func = I3C_Tgt_IBI_ISR;

  /* Check on the IBI additional data */
  if (LL_I3C_GetDeviceIBIPayload(p_i3cx) == LL_I3C_IBI_ADDITIONAL_DATA)
  {
    ASSERT_DBG_PARAM(p_payload != NULL);
    ASSERT_DBG_PARAM(payload_size_byte != 0U);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
    /* Check on the p_payload and payload_size values */
    if ((p_payload == NULL) || (payload_size_byte == 0U))
    {
      I3C_StateIdle(hi3c);
      return HAL_INVALID_PARAM;
    }
#endif /* USE_HAL_CHECK_PARAM */
    /* For loop to calculate the payload value */
    for (uint32_t index = 0U; index < payload_size_byte; index++)
    {
      payload_value |= ((uint32_t)p_payload[index] << (index * 8U));
    }

    /* Load IBI payload data */
    LL_I3C_SetIBIPayload(p_i3cx, payload_value);
  }

  LL_I3C_EnableIT(p_i3cx, LL_I3C_TGT_IBI_IT);

  /* Request IBI */
  LL_I3C_TargetHandleMessage(p_i3cx, LL_I3C_TARGET_MTYPE_IBI, payload_size_byte);

  return hal_status;
}

/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group8 Weak callback functions
  * @{
A set of Weak functions (or default callback functions if USE_HAL_I3C_REGISTER_CALLBACKS is set to 1) are used
to asynchronously inform the application in non-blocking modes (interrupt and DMA):
 - HAL_I3C_CTRL_TransferCpltCallback()     : Controller multiple transfer completed callback.
 - HAL_I3C_CTRL_DAACpltCallback()          : Controller Dynamic Address Assignment complete callback.
 - HAL_I3C_CTRL_TgtReqDynAddrCallback()    : Target request Dynamic Address callback.
 - HAL_I3C_TGT_TxCpltCallback()            : Target transmission complete callback.
 - HAL_I3C_TGT_RxCpltCallback()            : Target reception complete callback.
 - HAL_I3C_TGT_HotJoinCallback()           : Target Hot-Join process complete callback.
 - HAL_I3C_NotifyCallback()                : Target/Controller notification event callback.
 - HAL_I3C_ErrorCallback()                 : Target/Controller error callback.
 - HAL_I3C_AbortCpltCallback()             : Target/Controller abort callback.
  */

/**
  * @brief  Controller Multiple transfer completed callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_CTRL_TransferCpltCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /* WARNING: This function must not be modified, when the callback is needed,
     the HAL_I3C_CTRL_TransferCpltCallback must be implemented in the user file
  */
}

/**
  * @brief  Controller dynamic address assignment complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_CTRL_DAACpltCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /* WARNING: This function must not be modified, when the callback is needed,
     the HAL_I3C_CTRL_DAACpltCallback must be implemented in the user file
  */
}

/**
  * @brief  Target Request Dynamic Address callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  target_payload  Parameter indicates the target payload
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_CTRL_TgtReqDynAddrCallback(hal_i3c_handle_t *hi3c, uint64_t target_payload)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);
  STM32_UNUSED(target_payload);

  /* WARNING: This function must not be modified, when the callback is needed,
     the HAL_I3C_CTRL_TgtReqDynAddrCallback must be implemented in the user file
  */
}

/**
  * @brief  Target Transmission complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_TGT_TxCpltCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /* WARNING: This function must not be modified, when the callback is needed,
     the HAL_I3C_TGT_TxCpltCallback must be implemented in the user file
  */
}

/**
  * @brief  Target Reception complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_TGT_RxCpltCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /** NOTE: This function must not be modified, when the callback is needed,
    * the HAL_I3C_TGT_RxCpltCallback must be implemented in the user file
    */
}

/**
  * @brief  Target Hot-Join process complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  dynamic_address The returned dynamic address value after the Hot-Join process
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_TGT_HotJoinCallback(hal_i3c_handle_t *hi3c, uint8_t dynamic_address)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);
  STM32_UNUSED(dynamic_address);

  /** NOTE: This function must not be modified, when the callback is needed,
    * the HAL_I3C_TGT_HotJoinCallback must be implemented in the user file
    */
}

/**
  * @brief  Target/Controller Notification event callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  notifications  Parameter indicates which notification is signaled
  *                            It can be a combination value of @ref I3C_CTRL_NOTIFICATION or @ref I3C_TGT_NOTIFICATION.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_NotifyCallback(hal_i3c_handle_t *hi3c, uint32_t notifications)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);
  STM32_UNUSED(notifications);

  /**  NOTE: This function must not be modified, when the callback is needed,
    *  the HAL_I3C_NotifyCallback must be implemented in the user file
    */
}

/**
  * @brief  Error callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_ErrorCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /**  NOTE: This function must not be modified, when the callback is needed,
    *  the HAL_I3C_ErrorCallback must be implemented in the user file
    */
}

/**
  * @brief  Abort complete callback.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I3C_AbortCpltCallback(hal_i3c_handle_t *hi3c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi3c);

  /**  NOTE: This function must not be modified, when the callback is needed,
    *  the HAL_I3C_AbortCpltCallback must be implemented in the user file
    */
}

/**
  * @}
  */

/** @addtogroup I3C_Exported_Functions_Group9 Generic and common functions
  * @{
A set of functions that abort transfers or retrieve the runtime status of the peripheral:
  - HAL_I3C_Abort_IT() to abort the current transfer either in DMA or IT.
  - HAL_I3C_GetState() to get the I3C handle state.
  - HAL_I3C_GetMode() to get the I3C handle mode.
  - HAL_I3C_GetLastErrorCodes() to get the last error code.
  - HAL_I3C_GetClockFreq() to get the kernel clock frequency
  - HAL_I3C_CTRL_GetENTDAA_PayloadInfo() to get BCR, DCR and PID information after ENTDAA.
  - HAL_I3C_GetCCCInfo() to retrieve some specific associated CCC event data.
  - HAL_I3C_GetDataCounter() to get the counter data.
  */

/**
  * @brief  Abort an I3C IT or DMA process communication with Interrupt.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval HAL_OK  Operation completed successfully
  */
hal_status_t HAL_I3C_Abort_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx;
  uint32_t flush_mask;
  uint32_t it_source;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if (hi3c->global_state != HAL_I3C_STATE_ABORT)
  {
    hi3c->global_state = HAL_I3C_STATE_ABORT;
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
    hi3c->last_error_codes = HAL_I3C_ERROR_NONE;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

    LL_I3C_DisableIT_ERR(p_i3cx);
    hi3c->p_isr_func = I3C_Abort_ISR;

    /* Flush the different Fifos to generate an automatic stop mode link to underflow or overflow detection timeout */
    flush_mask = (I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH);

    if (hi3c->mode == HAL_I3C_MODE_CTRL)
    {
      flush_mask |= (I3C_CFGR_SFLUSH | I3C_CFGR_CFLUSH);
    }

    LL_I3C_RequestFifosFlush(p_i3cx, flush_mask);

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
    /* Disable all DMA Requests */
    LL_I3C_DisableAllDMARequests(p_i3cx);
#endif /* USE_HAL_I3C_DMA */

    if (hi3c->mode == HAL_I3C_MODE_CTRL)
    {
      it_source = (LL_I3C_CTRL_RX_IT | LL_I3C_CTRL_TX_IT);
    }
    else
    {
      it_source = LL_I3C_TGT_RX_IT;
    }
    LL_I3C_EnableIT(p_i3cx, it_source);
  }

  return HAL_OK;
}

/**
  * @brief  Return the I3C handle state.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval global_state:
  *  - HAL_I3C_STATE_RESET    Not yet Initialized
  *  - HAL_I3C_STATE_INIT     I3C is initialized but not yet configured
  *  - HAL_I3C_STATE_IDLE     I3C initialized and a global config applied
  *  - HAL_I3C_STATE_TX       Data Transmission process is ongoing
  *  - HAL_I3C_STATE_RX       Data Reception process is ongoing
  *  - HAL_I3C_STATE_TX_RX    Data Multiple Transfer process is ongoing
  *  - HAL_I3C_STATE_DAA      Dynamic address assignment process is ongoing
  *  - HAL_I3C_STATE_TGT_REQ  Target request process is ongoing
  *  - HAL_I3C_STATE_ABORT    Abort user request ongoing
  */
hal_i3c_state_t HAL_I3C_GetState(const hal_i3c_handle_t *hi3c)
{
  return hi3c->global_state;
}

/**
  * @brief  Returns the I3C mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval Mode:
  *  - HAL_I3C_MODE_NONE  No I3C communication on going
  *  - HAL_I3C_MODE_CTRL  I3C communication is in controller mode
  *  - HAL_I3C_MODE_TGT   I3C communication is in target mode
  */
hal_i3c_mode_t HAL_I3C_GetMode(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);
  return hi3c->mode;
}
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
/**
  * @brief  Returns errors limited to the last process.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @return uint32_t last error code. It can be HAL_I3C_ERROR_NONE or a combination of the following values:
  * Controller protocol / parity / address phase errors:
  *   - HAL_I3C_CTRL_ERROR_0  : Illegally formatted CCC
  *   - HAL_I3C_CTRL_ERROR_1  : Transmitted data differs from expected
  *   - HAL_I3C_CTRL_ERROR_2  : Broadcast address 0x7E not acknowledged
  *   - HAL_I3C_CTRL_ERROR_3  : New controller did not drive the bus after controller-role hand-off
  * Target protocol / parity / address phase errors:
  *   - HAL_I3C_TGT_ERROR_0   : Invalid broadcast address 0x7E + W
  *   - HAL_I3C_TGT_ERROR_1   : Parity error on a CCC code
  *   - HAL_I3C_TGT_ERROR_2   : Parity error on a write data byte
  *   - HAL_I3C_TGT_ERROR_3   : Parity error on the assigned address during dynamic address arbitration
  *   - HAL_I3C_TGT_ERROR_4   : Missing 0x7E + R after Sr during dynamic address arbitration
  *   - HAL_I3C_TGT_ERROR_5   : Illegally formatted CCC
  *   - HAL_I3C_TGT_ERROR_6   : Transmitted data differs from expected
  * Common data / flow errors:
  *   - HAL_I3C_ERROR_DATA_HAND_OFF  : Data error during Controller-Role hand-off; active controller keeps role
  *   - HAL_I3C_ERROR_DATA_NACK      : Data not acknowledged
  *   - HAL_I3C_ERROR_ADDRESS_NACK   : Address not acknowledged
  *   - HAL_I3C_ERROR_COVR           : Status FIFO over-run or Control FIFO under-run
  *   - HAL_I3C_ERROR_DOVR           : Rx FIFO over-run or Tx FIFO under-run
  *   - HAL_I3C_TGT_ERROR_STALL      : SCL held stable > timeout during SDR data read
  *   - HAL_I3C_ERROR_DMA            : DMA transfer error
  *   - HAL_I3C_ERROR_DYNAMIC_ADDR   : Dynamic address error
  */
uint32_t HAL_I3C_GetLastErrorCodes(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return hi3c->last_error_codes;
}
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

/**
  * @brief  Get the data counter according to the current usecase (tgt/ctrl, transfer or not).
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @return data counter:
  *    - During the Dynamic Address Assignment process (ENTDAA CCC):
  *        - When the I3C acts as controller: number of targets detected.
  *        - When the I3C acts as target: number of transmitted bytes.
  *    - During the transfer:
  *        - Whatever the I3C acts as controller or target: number of data bytes read from or transmitted
             on the I3C bus during the message.
  */
uint32_t HAL_I3C_GetDataCounter(hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX | HAL_I3C_STATE_TX_RX \
                   | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return LL_I3C_GetXferDataCount(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Target/Controller get the Common Command Code (CCC) information updated after notifications.
  *
  *  | CCC Notification                    | Updated fields in p_ccc_info                         |
  *  |-------------------------------------|:----------------------------------------------------:|
  *  | HAL_I3C_TGT_NOTIFICATION_DAU        | dynamic_addr, dynamic_addr_valid                     |
  *  | HAL_I3C_TGT_NOTIFICATION_SETMWL     | max_write_data_size_byte                             |
  *  | HAL_I3C_TGT_NOTIFICATION_SETMRL     | max_read_data_size_byte                              |
  *  | HAL_I3C_TGT_NOTIFICATION_RSTACT     | reset_action                                         |
  *  | HAL_I3C_TGT_NOTIFICATION_ENTAS_X    | activity_state                                       |
  *  | HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC | hot_join_allowed, in_band_allowed, ctrl_role_allowed |
  *  | HAL_I3C_CTRL_NOTIFICATION_IBI       | ibi_cr_tgt_addr, ibi_tgt_nb_payload, ibi_tgt_payload |
  *  | HAL_I3C_CTRL_NOTIFICATION_CR        | ibi_cr_tgt_addr                                      |
  *
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  notifications  Notification. It can be a combination value of @ref I3C_CTRL_NOTIFICATION
                               or @ref I3C_TGT_NOTIFICATION
  * @param  p_ccc_info  Pointer to an I3C_CCCInfoTypeDef structure that contains the CCC information
  *                     updated after CCC event.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_GetCCCInfo(const hal_i3c_handle_t *hi3c, uint32_t notifications,
                                hal_i3c_ccc_info_t *p_ccc_info)
{
  I3C_TypeDef *p_i3cx;
  hal_status_t hal_status = HAL_OK;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_PARAM(p_ccc_info != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX \
                   | HAL_I3C_STATE_TX_RX | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  p_i3cx = I3C_GET_INSTANCE(hi3c);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_ccc_info == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Retrieve Target Dynamic Address value and Validity (target/controller) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_DAU) == HAL_I3C_TGT_NOTIFICATION_DAU)
  {
    p_ccc_info->dynamic_addr_valid = LL_I3C_IsEnabledOwnDynAddress(p_i3cx);
    p_ccc_info->dynamic_addr = LL_I3C_GetOwnDynamicAddress(p_i3cx);
  }

  /* Retrieve Maximum Write Data Length (target) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_SETMWL) == HAL_I3C_TGT_NOTIFICATION_SETMWL)
  {
    p_ccc_info->max_write_data_size_byte = LL_I3C_GetMaxWriteLength(p_i3cx);
  }

  /* Retrieve Maximum Read Data Length (target) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_SETMRL) == HAL_I3C_TGT_NOTIFICATION_SETMRL)
  {
    p_ccc_info->max_read_data_size_byte = LL_I3C_GetMaxReadLength(p_i3cx);
  }

  /* RetrieveResetAction/Level on receivedResetpattern (target) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_RSTACT) == HAL_I3C_TGT_NOTIFICATION_RSTACT)
  {
    p_ccc_info->reset_action = (hal_i3c_reset_action_t) LL_I3C_GetResetAction(p_i3cx);
  }

  /* Retrieve Activity State (target) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_ENTAS_X) == HAL_I3C_TGT_NOTIFICATION_ENTAS_X)
  {
    p_ccc_info->activity_state = (hal_i3c_activity_state_t) LL_I3C_GetActivityState(p_i3cx);
  }

  /* Retrieve Interrupt allowed status (target) */
  if ((notifications & HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC) == HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC)
  {
    p_ccc_info->hot_join_allowed = LL_I3C_IsEnabledHotJoin(p_i3cx);
    p_ccc_info->in_band_allowed = LL_I3C_IsEnabledIBI(p_i3cx);
    p_ccc_info->ctrl_role_allowed = LL_I3C_IsEnabledControllerRoleReq(p_i3cx);
  }

  /* Retrieve In Band Interrupt information (controller) */
  if ((notifications & HAL_I3C_CTRL_NOTIFICATION_IBI) == HAL_I3C_CTRL_NOTIFICATION_IBI)
  {
    p_ccc_info->ibi_cr_tgt_addr = LL_I3C_GetIBITargetAddr(p_i3cx);
    p_ccc_info->ibi_tgt_nb_payload = LL_I3C_GetNbIBIAddData(p_i3cx);
    p_ccc_info->ibi_tgt_payload = LL_I3C_GetIBIPayload(p_i3cx);
  }

  /* Retrieve Controller-Role request Interrupt information (controller) */
  if ((notifications & HAL_I3C_CTRL_NOTIFICATION_CR) == HAL_I3C_CTRL_NOTIFICATION_CR)
  {
    p_ccc_info->ibi_cr_tgt_addr = LL_I3C_GetIBITargetAddr(p_i3cx);
  }

  return hal_status;
}

/** @brief  Return the peripheral clock frequency for I3C.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval uint32_t Frequency in Hz.
  *                  0 if the source clock of the I3C is not configured or not ready.
  */
uint32_t HAL_I3C_GetClockFreq(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX \
                   | HAL_I3C_STATE_TX_RX | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  return HAL_RCC_I3C_GetKernelClkFreq(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  Get BCR, DCR and PID information after ENTDAA.
  * @param  entdaa_payload   Payload received after ENTDAA
  * @param  p_entdaa_payload Pointer to an I3C_ENTDAAPayloadTypeDef structure that contains the BCR, DCR and PID
  *                          information.
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_INVALID_PARAM  Invalid parameter
  */
hal_status_t HAL_I3C_CTRL_GetENTDAA_PayloadInfo(uint64_t entdaa_payload,
                                                hal_i3c_entdaa_payload_t *p_entdaa_payload)
{
  hal_status_t hal_status = HAL_OK;
  uint32_t bcr;
  uint64_t pid;

  ASSERT_DBG_PARAM(p_entdaa_payload != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_entdaa_payload == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Get bus characterics */
  bcr = HAL_I3C_GET_BCR(entdaa_payload);

  /* Retrieve BCR information */
  p_entdaa_payload->bcr.ibi_payload = HAL_I3C_GET_IBI_PAYLOAD(bcr);
  p_entdaa_payload->bcr.ibi_request_capable = HAL_I3C_GET_IBI_CAPABLE(bcr);
  p_entdaa_payload->bcr.ctrl_role = HAL_I3C_GET_CTRL_ROLE_CAPABLE(bcr);
  p_entdaa_payload->bcr.advanced_capabilities = HAL_I3C_GET_ADVANCED_CAPABLE(bcr);
  p_entdaa_payload->bcr.offline_capable = HAL_I3C_GET_OFFLINE_CAPABLE(bcr);
  p_entdaa_payload->bcr.virtual_target_support = HAL_I3C_GET_VIRTUAL_TGT(bcr);
  p_entdaa_payload->bcr.max_data_speed_limitation = HAL_I3C_GET_MAX_DATA_SPEED_LIMIT(bcr);

  /* Get device characterics */
  p_entdaa_payload->dcr = HAL_I3C_GET_DCR(entdaa_payload);

  /* Get provisioned ID */
  pid = HAL_I3C_GET_PID(entdaa_payload);

  /* Change PID from big endian to litlle endian */
  pid = (uint64_t)((((uint64_t)HAL_I3C_BIG_TO_LITTLE_ENDIAN((uint32_t) pid) << 32) |
                    ((uint64_t)HAL_I3C_BIG_TO_LITTLE_ENDIAN((uint32_t)(pid >> 32)))) >> 16);

  /* Retrieve PID information */
  p_entdaa_payload->pid.mipi_manuf_id = HAL_I3C_GET_MIPIMID(pid);
  p_entdaa_payload->pid.id_type_sel = HAL_I3C_GET_IDTSEL(pid);
  p_entdaa_payload->pid.part_id = HAL_I3C_GET_PART_ID(pid);
  p_entdaa_payload->pid.mipi_id = HAL_I3C_GET_MIPIID(pid);

  return hal_status;
}

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @defgroup I3C_Exported_Functions_Group10 Acquire/release the bus
  * @{
A set of functions that acquire or release the bus based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
 - HAL_I3C_AcquireBus() acquires the I3C bus.
 - HAL_I3C_ReleaseBus() releases the I3C bus.
  */

/**
  * @brief  Acquire the I3C bus thanks to the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  timeout_ms  Timeout duration in milliseconds
  * @note   The HAL_I3C_AcquireBus() must be called from thread mode only (not from handler mode i.e from ISR).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_ERROR Operation completed with error
  */
hal_status_t HAL_I3C_AcquireBus(hal_i3c_handle_t *hi3c, uint32_t timeout_ms)
{
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX \
                   | HAL_I3C_STATE_TX_RX | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  if (HAL_OS_SemaphoreTake(&hi3c->semaphore, timeout_ms) == HAL_OS_OK)
  {
    hal_status = HAL_OK;
  }

  return hal_status;
}

/**
  * @brief  Release the I3C bus thanks to the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @note   The HAL_I3C_ReleaseBus() must be called from thread mode only (not from handler mode i.e from ISR).
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_ERROR Operation completed with error
  */
hal_status_t HAL_I3C_ReleaseBus(hal_i3c_handle_t *hi3c)
{
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM(hi3c != NULL);
  ASSERT_DBG_STATE(hi3c->global_state, HAL_I3C_STATE_INIT | HAL_I3C_STATE_IDLE | HAL_I3C_STATE_TX | HAL_I3C_STATE_RX \
                   | HAL_I3C_STATE_TX_RX | HAL_I3C_STATE_DAA | HAL_I3C_STATE_TGT_REQ | HAL_I3C_STATE_ABORT);

  if (HAL_OS_SemaphoreRelease(&hi3c->semaphore) == HAL_OS_OK)
  {
    hal_status = HAL_OK;
  }

  return hal_status;
}

/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_I3C_USER_DATA) && (USE_HAL_I3C_USER_DATA == 1)
/** @defgroup I3C_Exported_Functions_Group11 Set/get user data
  * @{
A set of functions that manage a user data pointer stored in the I3C handle:
 - HAL_I3C_SetUserData() set the user data into the handle
 - HAL_I3C_GetUserData() get the user data from the handle
  */

/**
  * @brief  Set the user data pointer into the handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  p_user_data  Pointer to the user data
  */
void HAL_I3C_SetUserData(hal_i3c_handle_t *hi3c, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hi3c != NULL);

  hi3c->p_user_data = p_user_data;
}

/**
  * @brief  Get the user data pointer from the handle.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @retval void*  Pointer to the user data
  */
const void *HAL_I3C_GetUserData(const hal_i3c_handle_t *hi3c)
{
  ASSERT_DBG_PARAM(hi3c != NULL);

  return (hi3c->p_user_data);
}

/**
  * @}
  */
#endif /* USE_HAL_I3C_USER_DATA */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @addtogroup I3C_Private_Functions I3C Private Functions
  * @{
  */

/**
  * @brief  Interrupt Sub-Routine which handles target received events.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  uint32_t event = 0U;
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C Rx FIFO not empty interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_RXFNEMIS) != 0U)
  {
    hi3c->p_rx_func(hi3c);
  }

  /* I3C target complete Controller-Role hand-off procedure (direct GETACCR CCC) event management --------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CRUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_CRUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_GETACCCR;
  }

  /* I3C target receive any direct GETxxx CCC event management -------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_GETMIS) != 0U)
  {
    LL_I3C_ClearFlag_GET(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_GET_X;
  }

  /* I3C target receive get status command (direct GETSTATUS CCC) event management -----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_STAMIS) != 0U)
  {
    LL_I3C_ClearFlag_STA(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_GET_STATUS;
  }

  /* I3C target receive a dynamic address update (ENTDAA/RSTDAA/SETNEWDA CCC) event management -----------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_DAUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_DAUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_DAU;
  }

  /* I3C target receive maximum write length update (direct SETMWL CCC) event management -----------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_MWLUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_MWLUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_SETMWL;
  }

  /* I3C target receive maximum read length update(direct SETMRL CCC) event management -------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_MRLUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_MRLUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_SETMRL;
  }

  /* I3C target detectResetpattern (broadcast or direct RSTACT CCC) event management -------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_RSTMIS) != 0U)
  {
    LL_I3C_ClearFlag_RST(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_RSTACT;
  }

  /* I3C target receive activity state update (direct or broadcast ENTASx) CCC event management ----------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_ASUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_ASUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_ENTAS_X;
  }

  /* I3C target receive a direct or broadcast ENEC/DISEC CCC event management ----------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_INTUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_INTUPD(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_ENEC_DISEC;
  }

  /* I3C target receive a broadcast DEFTGTS CCC event management -----------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_DEFMIS) != 0U)
  {
    LL_I3C_ClearFlag_DEF(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_DEFTGTS;
  }

  /* I3C target receive a group addressing (broadcast DEFGRPA CCC) event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_GRPMIS) != 0U)
  {
    LL_I3C_ClearFlag_GRP(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_DEFGRPA;
  }

  /* I3C target wakeup event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_WKPMIS) != 0U)
  {
    LL_I3C_ClearFlag_WKP(p_i3cx);
    event |= HAL_I3C_TGT_NOTIFICATION_WKP;
  }

  if (event != 0U)
  {
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, event);
#else
    HAL_I3C_NotifyCallback(hi3c, event);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles Controller received events.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Ctrl_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C controller receive IBI event management ---------------------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_IBIMIS) != 0U)
  {
    /* Clear IBI request flag */
    LL_I3C_ClearFlag_IBI(p_i3cx);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_IBI);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_IBI);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* I3C controller Controller-Role request event management ---------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CRMIS) != 0U)
  {
    /* Clear Controller-Role request flag */
    LL_I3C_ClearFlag_CR(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_CR);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_CR);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* I3C controller Hot-Join event management ------------------------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_HJMIS) != 0U)
  {
    /* Clear Hot-Join flag */
    LL_I3C_ClearFlag_HJ(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_HJ);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_HJ);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target Hot-Join event.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_HotJoin_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C target receive a dynamic address update event management */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_DAUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_DAUPD(p_i3cx);
    LL_I3C_DisableIT(p_i3cx, LL_I3C_TGT_HOTJOIN_IT);

    /* Check the validity of the own dynamic address */
    if (LL_I3C_IsEnabledOwnDynAddress(p_i3cx) != 0U)
    {
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_tgt_hot_join_cb(hi3c, (uint8_t)LL_I3C_GetOwnDynamicAddress(p_i3cx));
#else
      HAL_I3C_TGT_HotJoinCallback(hi3c, (uint8_t)LL_I3C_GetOwnDynamicAddress(p_i3cx));
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
      hi3c->last_error_codes |= HAL_I3C_ERROR_DYNAMIC_ADDR;
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

      I3C_ErrorTreatment(hi3c);
    }
  }
  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target Controller-Role event.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_CtrlRole_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CRUPDMIS) != 0U)
  {
    LL_I3C_ClearFlag_CRUPD(p_i3cx);
    LL_I3C_DisableIT(p_i3cx, LL_I3C_TGT_CTRLROLE_IT);
    I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_GETACCCR);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_GETACCCR);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }
  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target IBI event.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_IBI_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C target IBI end process event management ---------------------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_IBIENDMIS) != 0U)
  {
    LL_I3C_ClearFlag_IBIEND(p_i3cx);
    LL_I3C_DisableIT(p_i3cx, LL_I3C_TGT_IBI_IT);
    I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_IBIEND);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_IBIEND);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }
  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target transmit data in Interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_Tx_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C Tx FIFO not full interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_TXFNFMIS) != 0U)
  {
    if (hi3c->tx_count_byte > 0U)
    {
      hi3c->p_tx_func(hi3c);
    }
  }

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    /* Check if all data bytes are transmitted */
    if (LL_I3C_GetXferDataCount(p_i3cx) == hi3c->data_size_byte)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_TGT_TX_IT);
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_tgt_tx_cplt_cb(hi3c);
#else
      HAL_I3C_TGT_TxCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      I3C_ErrorTreatment(hi3c);
    }
  }

  /* I3C target wakeup event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_WKPMIS) != 0U)
  {
    /* Clear WKP flag */
    LL_I3C_ClearFlag_WKP(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target receive data in Interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_Rx_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C Rx FIFO not empty interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_RXFNEMIS) != 0U)
  {
    if (hi3c->rx_count_byte > 0U)
    {
      hi3c->p_rx_func(hi3c);
    }
  }

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    /* Check if all data bytes are received */
    if (LL_I3C_GetXferDataCount(p_i3cx) == hi3c->data_size_byte)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_TGT_RX_IT);
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_tgt_rx_cplt_cb(hi3c);
#else
      HAL_I3C_TGT_RxCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      I3C_ErrorTreatment(hi3c);
    }
  }

  /* I3C target wakeup event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_WKPMIS) != 0U)
  {
    /* Clear WKP flag */
    LL_I3C_ClearFlag_WKP(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Interrupt Sub-Routine which handles target transmit data in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_Tx_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    /* Check if all data bytes are transmitted */
    if (HAL_DMA_GetDirectXferRemainingDataByte(hi3c->hdma_tx) == 0U)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_XFER_DMA);
      hi3c->tx_count_byte = 0U;
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_tgt_tx_cplt_cb(hi3c);
#else
      HAL_I3C_TGT_TxCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      I3C_ErrorTreatment(hi3c);
    }
  }

  /* I3C target wakeup event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_WKPMIS) != 0U)
  {
    /* Clear WKP flag */
    LL_I3C_ClearFlag_WKP(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles target receive data in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Tgt_Rx_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    /* Check if all data bytes are received */
    if (HAL_DMA_GetDirectXferRemainingDataByte(hi3c->hdma_rx) == 0U)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_XFER_DMA);
      hi3c->rx_count_byte = 0U;
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_tgt_rx_cplt_cb(hi3c);
#else
      HAL_I3C_TGT_RxCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      I3C_ErrorTreatment(hi3c);
    }
  }

  /* I3C target wakeup event management ----------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_WKPMIS) != 0U)
  {
    /* Clear WKP flag */
    LL_I3C_ClearFlag_WKP(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_TGT_NOTIFICATION_WKP);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}
#endif /* USE_HAL_I3C_DMA */

/**
  * @brief  Interrupt Sub-Routine which handles controller multiple transmission/reception in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Ctrl_Multiple_Xfer_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check if Control FIFO requests data */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CFNFMIS) != 0U)
  {
    if (hi3c->tc_count_word > 0U)
    {
      I3C_ControlDataTreatment(hi3c);
    }
  }

  /* I3C Tx FIFO not full interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_TXFNFMIS) != 0U)
  {
    if (hi3c->tx_count_byte > 0U)
    {
      hi3c->p_tx_func(hi3c);
    }
  }

  /* I3C Rx FIFO not empty interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_RXFNEMIS) != 0U)
  {
    if (hi3c->rx_count_byte > 0U)
    {
      hi3c->p_rx_func(hi3c);
    }
  }

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    if (hi3c->tc_count_word == 0U)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_CTRL_TX_IT | LL_I3C_CTRL_RX_IT);
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_ctrl_transfer_cplt_cb(hi3c);
#else
      HAL_I3C_CTRL_TransferCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      /* Then Initiate a Start condition */
      LL_I3C_RequestTransfer(p_i3cx);
    }
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handles controller multiple transmission/reception and controller received
  *         events in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Ctrl_Multiple_Xfer_Listen_Event_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C controller receive IBI event management ---------------------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_IBIMIS) != 0U)
  {
    /* Clear IBI request flag */
    LL_I3C_ClearFlag_IBI(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_IBI);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_IBI);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* I3C controller Controller-Role request event management ---------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CRMIS) != 0U)
  {
    /* Clear Controller-Role request flag */
    LL_I3C_ClearFlag_CR(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_CR);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_CR);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* I3C controller Hot-Join event management ------------------------------------------------------------------------*/
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_HJMIS) != 0U)
  {
    /* Clear Hot-Join flag */
    LL_I3C_ClearFlag_HJ(p_i3cx);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_notify_cb(hi3c, HAL_I3C_CTRL_NOTIFICATION_HJ);
#else
    HAL_I3C_NotifyCallback(hi3c, HAL_I3C_CTRL_NOTIFICATION_HJ);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* ISR controller transmission/reception */
  return (I3C_Ctrl_Multiple_Xfer_ISR(hi3c, it_masks));
}
/**
  * @brief  Interrupt Sub-Routine which handles controller CCC Dynamic Address Assignment command in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Ctrl_DAA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  uint64_t target_payload = 0U;

  /* I3C Control FIFO not full interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_CFNFMIS) != 0U)
  {
    /* Write ENTDAA CCC information in the control register */
    LL_I3C_ControllerHandleCCC(p_i3cx, LL_I3C_BROADCAST_ENTDAA, 0UL, LL_I3C_GENERATE_STOP);
  }

  /* I3C Tx FIFO not full interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_TXFNFMIS) != 0U)
  {
    /* Check on the Rx FIFO threshold to know the Dynamic Address Assignment treatment process: byte or word */
    if (LL_I3C_GetRxFIFOThreshold(p_i3cx) == LL_I3C_RXFIFO_THRESHOLD_1_8)
    {
      /* For loop to get target payload */
      for (uint32_t index = 0U; index < 8U; index++)
      {
        /* Retrieve payload byte by byte */
        target_payload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData8(p_i3cx) << (index * 8U));
      }
    }
    else
    {
      /* Retrieve first 32 bits payload */
      target_payload = (uint64_t)LL_I3C_ReceiveData32(p_i3cx);

      /* Retrieve second 32 bits payload */
      target_payload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData32(p_i3cx) << 32U);
    }

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_ctrl_tgt_req_dyn_addr_cb(hi3c, target_payload);
#else
    HAL_I3C_CTRL_TgtReqDynAddrCallback(hi3c, target_payload);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  /* I3C frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);
    LL_I3C_DisableIT(p_i3cx, LL_I3C_CTRL_DAA_IT);
    I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_ctrl_daa_cplt_cb(hi3c);
#else
    HAL_I3C_CTRL_DAACpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  Interrupt Sub-Routine which handles controller multiple receive and transmit data in DMA mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Ctrl_Multiple_Xfer_DMA_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C target frame complete event Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    LL_I3C_ClearFlag_FC(p_i3cx);

    if (HAL_DMA_GetDirectXferRemainingDataByte(hi3c->hdma_tc) == 0U)
    {
      LL_I3C_DisableIT(p_i3cx, LL_I3C_XFER_DMA);
      I3C_StateIdle(hi3c);
#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
      hi3c->p_ctrl_transfer_cplt_cb(hi3c);
#else
      HAL_I3C_CTRL_TransferCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
    }
    else
    {
      /* Then Initiate a Start condition */
      LL_I3C_RequestTransfer(p_i3cx);
    }
  }
  return HAL_OK;
}
#endif /* USE_HAL_I3C_DMA */

/**
  * @brief  Interrupt Sub-Routine which handles abort process in interrupt mode.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  it_masks  Flag Interrupt Masks flags to handle
  * @retval HAL_OK  Operation completed successfully
  */
static hal_status_t I3C_Abort_ISR(hal_i3c_handle_t *hi3c, uint32_t it_masks)
{
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* I3C Rx FIFO not empty interrupt Check */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_RXFNEMIS) != 0U)
  {
    if (LL_I3C_IsActiveFlag_DOVR(p_i3cx) != 0U)
    {
      /* Flush remaining Rx data */
      LL_I3C_RequestRxFIFOFlush(p_i3cx);
    }
  }

  /** I3C Abort frame complete event Check
    * Even if abort is called, the Frame completion can arrive if abort is requested at the end of the process
    * Even if completion occurs, treat this end of process as abort completion process
    */
  if (I3C_CHECK_FLAG(it_masks, LL_I3C_MISR_FCMIS) != 0U)
  {
    /* Clear frame complete flag */
    LL_I3C_ClearFlag_FC(p_i3cx);

    I3C_ErrorTreatment(hi3c);
  }

  return HAL_OK;
}

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
/**
  * @brief  DMA I3C control transmit process complete callback.
  * @param  hdma  Pointer to a hal_dma_handle_t structure that contains the configuration information
  *               for the specified DMA channel
  */
static void I3C_DMAControlTransmitCplt(hal_dma_handle_t *hdma)
{
  hal_i3c_handle_t *hi3c = (hal_i3c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  LL_I3C_DisableDMAReq_Control(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  DMA I3C transmit data process complete callback.
  * @param  hdma  Pointer to a hal_dma_handle_t structure that contains the configuration information
  *               for the specified DMA channel
  */
static void I3C_DMADataTransmitCplt(hal_dma_handle_t *hdma)
{
  hal_i3c_handle_t *hi3c = (hal_i3c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  LL_I3C_DisableDMAReq_TX(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  DMA I3C receive data process complete callback.
  * @param  hdma  Pointer to a hal_dma_handle_t structure that contains the configuration information
  *               for the specified DMA channel
  */
static void I3C_DMADataReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_i3c_handle_t *hi3c = (hal_i3c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  LL_I3C_DisableDMAReq_RX(I3C_GET_INSTANCE(hi3c));
}

/**
  * @brief  DMA I3C communication error callback.
  * @param  hdma  Pointer to a hal_dma_handle_t structure that contains the configuration information
  *               for the specified DMA channel
  */
static void I3C_DMAError(hal_dma_handle_t *hdma)
{
  hal_i3c_handle_t *hi3c = (hal_i3c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  hi3c->last_error_codes |= HAL_I3C_ERROR_DMA;
}

/**
  * @brief  DMA I3C communication abort callback to be called at end of DMA Abort procedure.
  * @param  hdma  Pointer to a hal_dma_handle_t structure that contains the configuration information
  *               for the specified DMA channel
  */
static void I3C_DMAAbort(hal_dma_handle_t *hdma)
{
  hal_i3c_handle_t *hi3c = (hal_i3c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  hal_dma_channel_t dma_instance = hdma->instance;
  uint32_t no_callback = 0U;

  if (hi3c->hdma_tx != NULL)
  {
    if (hi3c->hdma_tx->instance == dma_instance)
    {
      hi3c->hdma_tx->p_xfer_abort_cb = I3C_DMAError;
    }
    else if (hi3c->hdma_tx->p_xfer_abort_cb != I3C_DMAError)
    {
      no_callback++;
    }
    else
    {
      /* nothing to do */
    }
  }

  if (hi3c->hdma_rx != NULL)
  {
    if (hi3c->hdma_rx->instance == dma_instance)
    {
      hi3c->hdma_rx->p_xfer_abort_cb = I3C_DMAError;
    }
    else if (hi3c->hdma_rx->p_xfer_abort_cb != I3C_DMAError)
    {
      no_callback++;
    }
    else
    {
      /* nothing to do */
    }
  }

  if (hi3c->hdma_tc != NULL)
  {
    if (hi3c->hdma_tc->instance == dma_instance)
    {
      hi3c->hdma_tc->p_xfer_abort_cb = I3C_DMAError;
    }
    else if (hi3c->hdma_tc->p_xfer_abort_cb != I3C_DMAError)
    {
      no_callback++;
    }
    else
    {
      /* nothing to do */
    }
  }

  if (no_callback == 0U)
  {
    I3C_TreatErrorCallback(hi3c);
  }
}

/**
  * @brief Round up size to a multiple of 4.
  * @param size_byte Size in bytes.
  * @return Size rounded up to a multiple of 4.
  * @note If size_byte is already a multiple of 4 it is returned unchanged.
  * @par Examples
  *  - 3 -> 4
  *  - 4 -> 4
  *  - 6 -> 8
  *  - 12 -> 12
  */
static uint32_t I3C_RoundUp4(uint32_t size_byte)
{
  return (size_byte + 3U) & ~3U;
}
#endif /* USE_HAL_I3C_DMA */

/**
  * @brief  Wait on flag set until timeout.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  * @param  flag  Flag to check.
  * @param  timeout_ms  Timeout duration in milliseconds
  * @retval HAL_OK  Operation completed successfully
  * @retval HAL_TIMEOUT  Operation exceeds user timeout
  * @retval HAL_ERROR  Operation completed with error
  */
static hal_status_t I3C_WaitForFlagSet(hal_i3c_handle_t *hi3c, uint32_t flag, uint32_t timeout_ms)
{
  hal_status_t hal_status = HAL_OK;
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);
  uint32_t tickstart = HAL_GetTick();

  do
  {
    /* Check if an error occurs during Flag waiting */
    if (LL_I3C_IsActiveFlag_ERR(p_i3cx) != 0U)
    {
      LL_I3C_ClearFlag_ERR(p_i3cx);
#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
      I3C_GetErrorSources(hi3c);
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */
      hal_status = HAL_ERROR;
    }
    /* Check for the timeout */
    else if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        if (LL_I3C_IsActiveFlag(p_i3cx, flag) == 0U)
        {
          hal_status = HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* do nothing */
    }
  } while ((LL_I3C_IsActiveFlag(p_i3cx, flag) == 0U) && (hal_status == HAL_OK));

  return hal_status;
}

/**
  * @brief  I3C transmit by byte.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_TransmitByteTreatment(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check TX FIFO not full flag */
  while ((LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U) && (hi3c->tx_count_byte > 0U))
  {
    LL_I3C_TransmitData8(p_i3cx, *hi3c->p_tx_data);
    hi3c->p_tx_data++;
    hi3c->tx_count_byte--;
  }
}

/**
  * @brief  I3C transmit by byte, at most I3C_TRANSFER_MAX_BYTE.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_TransmitByteTreatment_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);
  uint32_t transfer_count = hi3c->tx_count_byte;
  if (transfer_count > I3C_TRANSFER_MAX_BYTE)
  {
    transfer_count = I3C_TRANSFER_MAX_BYTE;
  }

  /* Transmit transfer_count words while TX FIFO not full */
  while ((LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U) && (transfer_count != 0U))
  {
    LL_I3C_TransmitData8(p_i3cx, *hi3c->p_tx_data);
    hi3c->p_tx_data++;
    hi3c->tx_count_byte--;
    transfer_count--;
  }
}

/**
  * @brief  I3C transmit by word.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_TransmitWordTreatment(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check TX FIFO not full flag */
  while (LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U)
  {
    LL_I3C_TransmitData32(p_i3cx, *((uint32_t *)(uint32_t)hi3c->p_tx_data));
    hi3c->p_tx_data += 4U;

    if (hi3c->tx_count_byte > 4U)
    {
      hi3c->tx_count_byte -= 4U;
    }
    else
    {
      hi3c->tx_count_byte = 0U;
    }
  }
}

/**
  * @brief  I3C transmit by word, at most I3C_TRANSFER_MAX_WORD.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_TransmitWordTreatment_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);
  uint32_t transfer_count = (hi3c->tx_count_byte + 3U) / 4U;
  if (transfer_count > I3C_TRANSFER_MAX_WORD)
  {
    transfer_count = I3C_TRANSFER_MAX_WORD;
  }

  /* Transmit transfer_count while TX FIFO not full */
  while ((LL_I3C_IsActiveFlag_TXFNF(p_i3cx) != 0U) && (transfer_count != 0U))
  {
    LL_I3C_TransmitData32(p_i3cx, *((uint32_t *)(uint32_t)hi3c->p_tx_data));
    hi3c->p_tx_data += 4U;
    if (hi3c->tx_count_byte > 4U)
    {
      hi3c->tx_count_byte -= 4U;
    }
    else
    {
      hi3c->tx_count_byte = 0U;
    }
    transfer_count--;
  }
}

/**
  * @brief  I3C receive by byte.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ReceiveByteTreatment(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check RX FIFO not empty flag */
  while (LL_I3C_IsActiveFlag_RXFNE(p_i3cx) != 0U)
  {
    *hi3c->p_rx_data = LL_I3C_ReceiveData8(p_i3cx);
    hi3c->p_rx_data++;
    hi3c->rx_count_byte--;
  }
}

/**
  * @brief  I3C receive by byte, at most I3C_TRANSFER_MAX_BYTE.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ReceiveByteTreatment_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);
  uint32_t transfer_count = hi3c->rx_count_byte;
  if (transfer_count > I3C_TRANSFER_MAX_BYTE)
  {
    transfer_count = I3C_TRANSFER_MAX_BYTE;
  }

  /* Receive transfer_count while RX FIFO not empty */
  while ((LL_I3C_IsActiveFlag_RXFNE(p_i3cx) != 0U) && (transfer_count != 0U))
  {
    *hi3c->p_rx_data = LL_I3C_ReceiveData8(p_i3cx);
    hi3c->p_rx_data++;
    hi3c->rx_count_byte--;
    transfer_count--;
  }
}

/**
  * @brief  I3C receive by word.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ReceiveWordTreatment(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check RX FIFO not empty flag */
  while (LL_I3C_IsActiveFlag_RXFNE(p_i3cx) != 0U)
  {
    *((uint32_t *)(uint32_t)hi3c->p_rx_data) = LL_I3C_ReceiveData32(p_i3cx);
    hi3c->p_rx_data += sizeof(uint32_t);
    if (hi3c->rx_count_byte > 4U)
    {
      hi3c->rx_count_byte -= 4U;
    }
    else
    {
      hi3c->rx_count_byte = 0U;
    }
  }
}

/**
  * @brief  I3C receive by word, at most I3C_TRANSFER_MAX_WORD.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ReceiveWordTreatment_IT(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);
  uint32_t transfer_count = (hi3c->rx_count_byte + 3U) / 4U;
  if (transfer_count > I3C_TRANSFER_MAX_WORD)
  {
    transfer_count = I3C_TRANSFER_MAX_WORD;
  }

  /* Receive transfer_count while RX FIFO not empty */
  while ((LL_I3C_IsActiveFlag_RXFNE(p_i3cx) != 0U) && (transfer_count != 0U))
  {
    *((uint32_t *)(uint32_t)hi3c->p_rx_data) = LL_I3C_ReceiveData32(p_i3cx);
    hi3c->p_rx_data += 4U;
    if (hi3c->rx_count_byte > 4U)
    {
      hi3c->rx_count_byte -= 4U;
    }
    else
    {
      hi3c->rx_count_byte = 0U;
    }
    transfer_count++;
  }
}

/**
  * @brief  I3C Control data treatment.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ControlDataTreatment(hal_i3c_handle_t *hi3c)
{
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  /* Check if Control FIFO requests data */
  if (LL_I3C_IsActiveFlag_CFNF(p_i3cx) != 0U)
  {
    LL_I3C_WriteControlWord(p_i3cx, *hi3c->p_tc_data);
    hi3c->p_tc_data++;
    hi3c->tc_count_word--;
  }
}

/**
  * @brief  I3C state change to HAL_I3C_STATE_IDLE and must be called at
  *         the very end of a process to avoid state issue.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_StateIdle(hal_i3c_handle_t *hi3c)
{
  if (hi3c->listen == I3C_LISTEN_ENABLED)
  {
    if (hi3c->mode == HAL_I3C_MODE_TGT)
    {
      hi3c->p_isr_func = I3C_Tgt_Event_ISR;
    }
    else
    {
      hi3c->p_isr_func = I3C_Ctrl_Event_ISR;
    }
  }
  else
  {
    hi3c->p_isr_func = NULL;
  }

  hi3c->global_state = HAL_I3C_STATE_IDLE;
}

#if defined(USE_HAL_I3C_GET_LAST_ERRORS) && (USE_HAL_I3C_GET_LAST_ERRORS == 1)
/**
  * @brief  I3C get error source.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_GetErrorSources(hal_i3c_handle_t *hi3c)
{
  hi3c->last_error_codes |= LL_I3C_GetRegister_SER(I3C_GET_INSTANCE(hi3c));
}
#endif /* USE_HAL_I3C_GET_LAST_ERRORS */

/**
  * @brief  Check if the target device is ready.
  * @param  hi3c        Pointer to a @ref hal_i3c_handle_t
  * @param  p_device    Structure with device address and message type
  * @param  timeout_ms  Polling timeout (in ms)
  * @note   This function uses a polling strategy: repeatedly issues START + address
  *         and waits for hardware flags. It returns at the first successful frame
  *         complete or when the user timeout elapses
  * @retval HAL_OK       Frame complete (FCF) detected: device is ready
  * @retval HAL_TIMEOUT  User timeout elapsed: device not ready in time
  * @retval HAL_ERROR    Internal failure while waiting for hardware flags
  */
static hal_status_t I3C_Ctrl_PoolForDeviceReady(hal_i3c_handle_t *hi3c,
                                                const i3c_device_t *p_device,
                                                uint32_t timeout_ms)
{
  hal_status_t hal_status = HAL_OK;
  uint32_t cr_tmp;
  uint32_t arbitration_previous_state;
  uint32_t tickstart = HAL_GetTick();
  I3C_TypeDef *p_i3cx;

  p_i3cx = I3C_GET_INSTANCE(hi3c);

  arbitration_previous_state = LL_I3C_IsEnabledArbitrationHeader(p_i3cx);
  LL_I3C_DisableArbitrationHeader(p_i3cx);

  /* HAL_I3C_DIRECTION_WRITE = 0 */
  cr_tmp = (((uint32_t)p_device->address << I3C_CR_ADD_Pos) |
            p_device->message_type | LL_I3C_GENERATE_STOP);

  /** Retry strategy: attempt start + address until either:
    *  - FCF detected (device ready -> HAL_OK),
    *  - User timeout elapses (device not ready -> HAL_TIMEOUT), or
    *  - Internal wait exceeds I3C_DEFAULT_TIMEOUT_MS (internal failure -> HAL_ERROR).
    */
  while (hal_status == HAL_OK)
  {
    /* Initiate a start condition by writing in the CR register */
    LL_I3C_SetRegister_CR(p_i3cx, cr_tmp);
    uint32_t tickstart_hw = HAL_GetTick();

    while (LL_I3C_IsActiveFlag(p_i3cx, LL_I3C_EVR_FCF | LL_I3C_EVR_ERRF) == 0U)
    {
      if ((HAL_GetTick() - tickstart_hw) > I3C_DEFAULT_TIMEOUT_MS)
      {
        /* Internal failure */
        hal_status = HAL_ERROR;
        break;
      }
    }

    if (hal_status == HAL_OK)
    {
      if (LL_I3C_IsActiveFlag_FC(p_i3cx) != 0U)
      {
        LL_I3C_ClearFlag_FC(p_i3cx);
        /* Device is ready, break the loop and return HAL_OK */
        break;
      }
      else
      {
        /* Address is nack, device is not detected, let's try again */
        LL_I3C_ClearFlag_ERR(p_i3cx);
      }

      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        hal_status = HAL_TIMEOUT;
      }
    }
  }

  I3C_StateIdle(hi3c);

  if (arbitration_previous_state != 0U)
  {
    LL_I3C_EnableArbitrationHeader(p_i3cx);
  }

  return hal_status;
}

/**
  * @brief  I3C error treatment.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_ErrorTreatment(hal_i3c_handle_t *hi3c)
{
  uint32_t flush_mask;
#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
  uint32_t dma_abort_ctrl = 0U;
  uint32_t dma_abort_rx = 0U;
  uint32_t dma_abort_tx = 0U;
  uint32_t dma_abort_it = 0U;
#endif /* USE_HAL_I3C_DMA */
  I3C_TypeDef *p_i3cx = I3C_GET_INSTANCE(hi3c);

  if (hi3c->global_state == HAL_I3C_STATE_TGT_REQ)
  {
    I3C_StateIdle(hi3c);
    LL_I3C_DisableIT(p_i3cx, (LL_I3C_TGT_IBI_IT | LL_I3C_TGT_HOTJOIN_IT | LL_I3C_TGT_CTRLROLE_IT));
  }
  else
  {
    LL_I3C_DisableIT(p_i3cx, LL_I3C_CTRL_RX_IT | LL_I3C_CTRL_TX_IT);
    hi3c->tx_count_byte = 0U;
    hi3c->rx_count_byte = 0U;
    hi3c->tc_count_word = 0U;
    hi3c->p_tx_func = NULL;
    hi3c->p_rx_func = NULL;

    /* Flush the different Fifos to generate an automatic stop mode link to underflow or overflow detection timeout */
    flush_mask = (I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH);

    if (hi3c->mode == HAL_I3C_MODE_CTRL)
    {
      flush_mask |= (I3C_CFGR_SFLUSH | I3C_CFGR_CFLUSH);
    }

    LL_I3C_RequestFifosFlush(p_i3cx, flush_mask);

#if defined(USE_HAL_I3C_DMA) && (USE_HAL_I3C_DMA == 1)
    /* Prepare abort DMA transfer if any */
    if (hi3c->hdma_tc != NULL)
    {
      LL_I3C_DisableDMAReq_Control(p_i3cx);

      if (HAL_DMA_GetState(hi3c->hdma_tc) != HAL_DMA_STATE_IDLE)
      {
        hi3c->hdma_tc->p_xfer_abort_cb = I3C_DMAAbort;
        dma_abort_ctrl = 1U;
      }
    }

    if (hi3c->hdma_rx != NULL)
    {
      LL_I3C_DisableDMAReq_RX(p_i3cx);

      if (HAL_DMA_GetState(hi3c->hdma_rx) != HAL_DMA_STATE_IDLE)
      {
        hi3c->hdma_rx->p_xfer_abort_cb = I3C_DMAAbort;
        dma_abort_rx = 1U;
      }
    }

    if (hi3c->hdma_tx != NULL)
    {
      LL_I3C_DisableDMAReq_TX(p_i3cx);

      if (HAL_DMA_GetState(hi3c->hdma_tx) != HAL_DMA_STATE_IDLE)
      {
        hi3c->hdma_tx->p_xfer_abort_cb = I3C_DMAAbort;
        dma_abort_tx = 1U;
      }
    }

    /* Abort control DMA transfer if any */
    if (dma_abort_ctrl != 0U)
    {
      dma_abort_it++;
      if (HAL_DMA_Abort_IT(hi3c->hdma_tc) != HAL_OK)
      {
        /* Call Directly XferAbortCallback function in case of error */
        hi3c->hdma_tc->p_xfer_abort_cb(hi3c->hdma_tc);
      }
    }

    if (dma_abort_rx != 0U)
    {
      dma_abort_it++;
      if (HAL_DMA_Abort_IT(hi3c->hdma_rx) != HAL_OK)
      {
        /* Call Directly XferAbortCallback function in case of error */
        hi3c->hdma_rx->p_xfer_abort_cb(hi3c->hdma_rx);
      }
    }

    if (dma_abort_tx != 0U)
    {
      dma_abort_it++;
      if (HAL_DMA_Abort_IT(hi3c->hdma_tx) != HAL_OK)
      {
        /* Call Directly XferAbortCallback function in case of error */
        hi3c->hdma_tx->p_xfer_abort_cb(hi3c->hdma_tx);
      }
    }

    if (dma_abort_it == 0U)
    {
      I3C_TreatErrorCallback(hi3c);
    }
#else /* USE_HAL_I3C_DMA */
    I3C_TreatErrorCallback(hi3c);
#endif /* USE_HAL_I3C_DMA */
  }
}

/**
  * @brief  I3C Error callback treatment.
  * @param  hi3c  Pointer to a @ref hal_i3c_handle_t
  */
static void I3C_TreatErrorCallback(hal_i3c_handle_t *hi3c)
{
  if (hi3c->global_state == HAL_I3C_STATE_ABORT)
  {
    I3C_StateIdle(hi3c);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_abort_cplt_cb(hi3c);
#else
    HAL_I3C_AbortCpltCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }
  else
  {
    I3C_StateIdle(hi3c);

#if (USE_HAL_I3C_REGISTER_CALLBACKS) && (USE_HAL_I3C_REGISTER_CALLBACKS == 1)
    hi3c->p_error_cb(hi3c);
#else
    HAL_I3C_ErrorCallback(hi3c);
#endif /* USE_HAL_I3C_REGISTER_CALLBACKS */
  }
}

/**
  * @}
  */

#endif /* USE_HAL_I3C_MODULE */
#endif /* I3C1 */

/**
  * @}
  */

/**
  * @}
  */
