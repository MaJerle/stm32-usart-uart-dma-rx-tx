/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_fdcan.c
  * @brief   FDCAN HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Flexible Data-rate Controller Area Network
  *          (FDCAN) peripheral:
  *           + Initialization and de-initialization functions
  *           + I/O operation functions
  *           + Peripheral Configuration and Control functions
  *           + Peripheral State and Error functions.
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

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(FDCAN1) || defined(FDCAN2)
#if defined(USE_HAL_FDCAN_MODULE) && (USE_HAL_FDCAN_MODULE == 1)

/** @addtogroup FDCAN FDCAN
  * @{
  */
/** @defgroup FDCAN_Introduction FDCAN Introduction
  * @{

The FDCAN hardware abstraction layer provides a comprehensive software interface to support the communication
protocol on STM32 microcontrollers.
FDCAN (Flexible Data-rate Controller Area Network) is an advanced communication protocol that extends the classical
CAN protocol to support higher data rates and improved error handling capabilities.
It is designed to meet the requirements of modern automotive and industrial applications.

This peripheral conforms to CAN protocol version 2.0 part A, part B, and ISO 11898-1.
This HAL driver abstracts low-level hardware details, enabling configuration, data transfer, and interrupt handling,
thereby accelerating development and ensuring robust and scalable embedded applications leveraging the full
capabilities of the CAN-FD bus.

  */
/**
  * @}
  */

/** @defgroup FDCAN_How_To_Use FDCAN How To Use
  * @{

# How to use this driver

1. Declare a hal_fdcan_handle_t handle structure and initialize the FDCAN driver with an FDCAN instance.

2. Initialize the FDCAN peripheral using the HAL_FDCAN_Init() function. The FDCAN clock is enabled inside
   HAL_FDCAN_Init() if USE_HAL_FDCAN_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.

3. Configure the low-level hardware (GPIO, CLOCK, NVIC, etc.)
- Enable the FDCAN clock if USE_HAL_FDCAN_CLK_ENABLE_MODEL = HAL_CLK_ENABLE_NO
- FDCAN pins configuration:
  - Enable the clock for the FDCAN GPIOs
  - Configure the NVIC for interrupt processing

4. Configure the peripheral using the HAL_FDCAN_SetConfig()
- If needed, the kernel clock divider can be changed by the first instance after the global configuration
  using HAL_FDCAN_SetClockDivider(). Only the first instance is allowed to modify the divider.
- If needed, configure and retrieve the reception filters and optional features using the following configuration
  functions:
  - HAL_FDCAN_GetConfig()
  - HAL_FDCAN_SetNominalBitTiming()
  - HAL_FDCAN_GetNominalBitTiming()
  - HAL_FDCAN_SetDataBitTiming()
  - HAL_FDCAN_GetDataBitTiming()
  - HAL_FDCAN_SetClockDivider()
  - HAL_FDCAN_GetClockDivider()
  - HAL_FDCAN_SetFilter()
  - HAL_FDCAN_GetFilter()
  - HAL_FDCAN_SetGlobalFilter()
  - HAL_FDCAN_GetGlobalFilter()
  - HAL_FDCAN_SetExtendedIdMask()
  - HAL_FDCAN_GetExtendedIdMask()
  - HAL_FDCAN_SetRxFifoOverwrite()
  - HAL_FDCAN_GetRxFifoOverwrite()
  - HAL_FDCAN_SetRamWatchdog()
  - HAL_FDCAN_GetRamWatchdog()
  - HAL_FDCAN_SetConfigTimestampCounter()
  - HAL_FDCAN_GetConfigTimestampCounter()
  - HAL_FDCAN_SetConfigTimeoutCounter()
  - HAL_FDCAN_GetConfigTimeoutCounter()
  - HAL_FDCAN_EnableTimeoutCounter()
  - HAL_FDCAN_DisableTimeoutCounter()
  - HAL_FDCAN_IsEnabledTimeoutCounter()
  - HAL_FDCAN_SetConfigTxDelayCompensation()
  - HAL_FDCAN_GetConfigTxDelayCompensation()
  - HAL_FDCAN_EnableTxDelayCompensation()
  - HAL_FDCAN_DisableTxDelayCompensation()
  - HAL_FDCAN_IsEnabledTxDelayCompensation()
  - HAL_FDCAN_EnableISOMode()
  - HAL_FDCAN_DisableISOMode()
  - HAL_FDCAN_IsEnabledISOMode()
  - HAL_FDCAN_EnableEdgeFiltering()
  - HAL_FDCAN_DisableEdgeFiltering()
  - HAL_FDCAN_IsEnabledEdgeFiltering()
  - HAL_FDCAN_SetMode()
  - HAL_FDCAN_GetMode()
  - HAL_FDCAN_EnableRestrictedOperationMode()
  - HAL_FDCAN_DisableRestrictedOperationMode()
  - HAL_FDCAN_IsEnabledRestrictedOperationMode()
  - HAL_FDCAN_SetFrameFormat()
  - HAL_FDCAN_GetFrameFormat()
  - HAL_FDCAN_EnableAutoRetransmission()
  - HAL_FDCAN_DisableAutoRetransmission()
  - HAL_FDCAN_IsEnabledAutoRetransmission()
  - HAL_FDCAN_EnableTransmitPause()
  - HAL_FDCAN_DisableTransmitPause()
  - HAL_FDCAN_IsEnabledTransmitPause()
  - HAL_FDCAN_EnableProtocolException()
  - HAL_FDCAN_DisableProtocolException()
  - HAL_FDCAN_IsEnabledProtocolException()
  - HAL_FDCAN_SetTxMode()
  - HAL_FDCAN_GetTxMode()
  - HAL_FDCAN_GetClockFreq()

5. Operation Modes
- FDCAN communication is primarily handled in interrupt (IT) mode.
After configuring the peripheral and enabling the necessary interrupts, the application starts the FDCAN module.
Events such as transmission complete, transmission cancellation, or message reception are handled asynchronously via
their respective callbacks (e.g., HAL_FDCAN_TxEventFifoCallback(), HAL_FDCAN_TxBufferCompleteCallback()).
The interrupt configuration sequence is as follows:
  - Enable individual interrupt sources using HAL_FDCAN_EnableInterrupts().
  - Map interrupt groups to an interrupt line using HAL_FDCAN_SetInterruptGroupsToLine().
  - Enable the selected interrupt line(s) using HAL_FDCAN_EnableInterruptLines().
The status of individual interrupt sources can be checked with HAL_FDCAN_IsEnabledInterrupt(), and the status
of interrupt lines can be checked with HAL_FDCAN_IsEnabledInterruptLine().

- For transmission complete and transmission cancellation finished interrupts, buffer-specific interrupt enables are
  required.
  - Enable interrupts for the relevant transmit buffers using HAL_FDCAN_EnableTxBufferCompleteInterrupts()
  and HAL_FDCAN_EnableTxBufferCancellationInterrupts(). These functions allow enabling one or multiple buffers
  simultaneously.
  - Disable interrupts for specific buffers using HAL_FDCAN_DisableTxBufferCompleteInterrupts()
  and HAL_FDCAN_DisableTxBufferCancellationInterrupts(). These functions allow disabling one or multiple buffers
  simultaneously.
  - Check the enable status of a specific buffer interrupt using HAL_FDCAN_IsEnabledTxBufferCompleteInterrupt()
  or HAL_FDCAN_IsEnabledTxBufferCancellationInterrupt(). These status functions operate on a single buffer at a time.

- While interrupt-driven operation is recommended for most use cases, the driver also provides blocking (polling) APIs
for synchronous state monitoring.
  - Use HAL_FDCAN_GetTxBufferMessageStatus() to check if a transmission request is pending on a specific Tx buffer.
  - Use HAL_FDCAN_GetRxFifoFillLevel() to retrieve the current number of elements in an Rx FIFO.
  - Use HAL_FDCAN_GetTxFifoFreeLevel() to determine the number of free elements available in the Tx FIFO.
Use these polling APIs to monitor transmission and reception states synchronously, which can be useful
in scenarios where deterministic, blocking behavior is required or when interrupts are not suitable or available.

6. Use the control functions to initiate Rx/Tx transfers over the FDCAN bus, either sending frames,
or receiving frames, or checking and managing the whole transfer process and resources (status, buffers, events).
Most of the control functions can be called in IDLE, ACTIVE, or POWER_DOWN states; however, some control functions are
restricted to specific states. For example, HAL_FDCAN_Start() can be called in IDLE but not in ACTIVE or POWER_DOWN.
The control functions include the following set of functions:
  - HAL_FDCAN_Start()
  - HAL_FDCAN_Stop()
  - HAL_FDCAN_ReqTransmitMsgFromFIFOQ()
  - HAL_FDCAN_GetTxFifoStatus()
  - HAL_FDCAN_GetLatestTxFifoQRequestBuffer()
  - HAL_FDCAN_GetTxFifoFreeLevel()
  - HAL_FDCAN_ReqAbortOfTxBuffer()
  - HAL_FDCAN_GetTxEvent()
  - HAL_FDCAN_GetTxEventFifoFillLevel()
  - HAL_FDCAN_GetTxBufferMessageStatus()
  - HAL_FDCAN_GetReceivedMessage()
  - HAL_FDCAN_GetRxFifoFillLevel()
  - HAL_FDCAN_GetHighPriorityMessageStatus()
  - HAL_FDCAN_GetProtocolStatus()
  - HAL_FDCAN_GetErrorCounters()
  - HAL_FDCAN_Recover()

Call HAL_FDCAN_Start() to start the FDCAN module. At this step, the node is active
on the bus:
- It can send and receive messages:
  - The following Tx control functions can be called when the FDCAN module is started or in power down mode, but it
    must be operating only when the FDCAN is in active mode.
    - HAL_FDCAN_ReqTransmitMsgFromFIFOQ()
    - HAL_FDCAN_ReqAbortOfTxBuffer()

- After submitting a Tx request in the Tx FIFO or queue, call HAL_FDCAN_GetLatestTxFifoQRequestBuffer() to determine
  which Tx buffer was used for the latest request.
  The corresponding Tx request can then be aborted later using the HAL_FDCAN_ReqAbortOfTxBuffer() function.

- In Queue mode, check whether a new Tx request can be submitted with HAL_FDCAN_GetTxFifoStatus().
- In FIFO mode, use HAL_FDCAN_GetTxFifoFreeLevel() to retrieve the number of available FIFO elements.

- Retrieve a message received into the FDCAN message RAM using the HAL_FDCAN_GetReceivedMessage() function.

- Call HAL_FDCAN_Stop() to stop the FDCAN module by switching it to initialization mode, which re-enables
  access to configuration registers through the configuration functions listed above.

8. Callbacks definition in interrupt mode:

  When the compilation define USE_HAL_FDCAN_REGISTER_CALLBACKS is set to 1U, configure the
  driver callbacks dynamically using the registration functions below:

    Callback name    | Default value                               | Callback registration function
  -------------------| --------------------------------------------| ---------------------------
  Rx FIFO 0          | HAL_FDCAN_RxFifo0Callback()                 | HAL_FDCAN_RegisterRxFifo0Callback()
  Rx FIFO 1          | HAL_FDCAN_RxFifo1Callback()                 | HAL_FDCAN_RegisterRxFifo1Callback()
  Tx event FIFO      | HAL_FDCAN_TxEventFifoCallback()             | HAL_FDCAN_RegisterTxEventFifoCallback()
  Tx FIFO empty      | HAL_FDCAN_TxFifoEmptyCallback()             | HAL_FDCAN_RegisterTxFifoEmptyCallback()
  Tx buffer complete | HAL_FDCAN_TxBufferCompleteCallback()        | HAL_FDCAN_RegisterTxBufferCompleteCallback()
  Tx buffer abort    | HAL_FDCAN_TxBufferAbortCallback()           | HAL_FDCAN_RegisterTxBufferAbortCallback()
  High priority MSG  | HAL_FDCAN_HighPriorityMessageCallback()     | HAL_FDCAN_RegisterHighPriorityMessageCallback()
  TS wraparound      | HAL_FDCAN_TimestampWraparoundCallback()     | HAL_FDCAN_RegisterTimestampWraparoundCallback()
  Error              | HAL_FDCAN_ErrorCallback()                   | HAL_FDCAN_RegisterErrorCallback()

  If one needs to unregister a callback, register the default callback via the registration function.

  By default, after the HAL_FDCAN_Init() and when the state is HAL_FDCAN_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

- Callbacks can be registered in HAL_FDCAN_STATE_INIT or HAL_FDCAN_STATE_IDLE states only.

- When HAL_FDCAN_ErrorCallback() (or its registered callback) is called, check if the error code contains
  HAL_FDCAN_ERROR_BUS_FAULT_OFF. If a bus-off condition is detected, initiate recovery by calling HAL_FDCAN_Recover().

- Restricted operation mode handling:
  - The hardware in some cases automatically sets the ASM bit (restricted operation). Calling HAL_FDCAN_GetMode() can
    return HAL_FDCAN_MODE_INVALID because the CCCR bits combination does not match a standard user-configurable mode.
  - Scenarios forcing restricted operation (ASM bit set):
    - Message RAM access failure:
      - A RAM access fault (e.g., timing violation or HW fault) sets ASM and raises HAL_FDCAN_ERROR_RAM_ACCESS_FAILURE.
      - Required actions:
        - Diagnose and fix the RAM access issue (memory layout, timing, HW fault).
        - Reinitialize or reset the FDCAN peripheral if needed.
        - Clear ASM by calling HAL_FDCAN_DisableRestrictedOperationMode() when fixed.
  - Normal communication is suspended while ASM remains set; clear it after resolving the root cause.

- If USE_HAL_FDCAN_REGISTER_CALLBACKS is 0U (or undefined) callback registration is not available and the default weak
  callbacks listed above are used.

9. Acquire/Release the FDCAN bus
- When the compilation flag USE_HAL_MUTEX is set to 1, it allows the user to acquire/reserve the whole FDCAN bus for
  executing process. The HAL_FDCAN_Acquire and HAL_FDCAN_Release are based on the HAL OS abstraction layer
  (stm32_hal_os.c/.h osal):
  - HAL_FDCAN_AcquireBus() for acquiring the bus or wait for it.
  - HAL_FDCAN_ReleaseBus() for releasing the bus.

- When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_FDCAN_AcquireBus/HAL_FDCAN_ReleaseBus
  are not available.

  */
/**
  * @}
  */

/** @defgroup FDCAN_Configuration_Table FDCAN Configuration Table
  * @{
10. Configuration inside the FDCAN driver

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_FDCAN_MODULE             |         1         | Enable HAL FDCAN driver module
USE_HAL_FDCAN_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_CHECK_PARAM              |         0         | Enable runtime parameter check
USE_HAL_FDCAN_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE      |         0         | Enable atomicity of process state check
USE_HAL_MUTEX                    |         0         | Enable semaphore creation for OS
USE_HAL_FDCAN_USER_DATA          |         0         | Add a user data inside HAL FDCAN handle
USE_HAL_FDCAN_GET_LAST_ERRORS    |         0         | Enable retrieval of the last process error codes

Software configuration defined in preprocessor environment:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM             |    Not defined    | Enable check param for HAL
USE_ASSERT_DBG_STATE             |    Not defined    | Enable check state for HAL

  */
/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Private_Constants FDCAN Private Constants
  * @{
  */

/*! LUT with Data Length Code (DLC) values to corresponding number of bytes */
static const uint8_t fdcan_lut_dlc2bytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

#define FDCAN_IR_MSK       (FDCAN_IR_RF0N | FDCAN_IR_RF0F | FDCAN_IR_RF0L | FDCAN_IR_RF1N | FDCAN_IR_RF1F          \
                            | FDCAN_IR_RF1L | FDCAN_IR_HPM | FDCAN_IR_TC | FDCAN_IR_TFE | FDCAN_IR_TEFN            \
                            | FDCAN_IR_TEFF | FDCAN_IR_TEFL | FDCAN_IR_TSW | FDCAN_IR_MRAF | FDCAN_IR_TOO          \
                            | FDCAN_IR_ELO | FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO | FDCAN_IR_WDI | FDCAN_IR_PEA \
                            | FDCAN_IR_PED | FDCAN_IR_ARA | FDCAN_IR_TCF) /*!< FDCAN interrupts mask */

#define FDCAN_IE_MSK       (FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE | FDCAN_IE_RF1NE | FDCAN_IE_RF1FE \
                            | FDCAN_IE_RF1LE | FDCAN_IE_HPME | FDCAN_IE_TCE | FDCAN_IE_TFEE | FDCAN_IE_TEFNE   \
                            | FDCAN_IE_TEFFE | FDCAN_IE_TEFLE | FDCAN_IE_TSWE | FDCAN_IE_MRAFE | FDCAN_IE_TOOE \
                            | FDCAN_IE_ELOE | FDCAN_IE_EPE | FDCAN_IE_EWE | FDCAN_IE_BOE | FDCAN_IE_WDIE       \
                            | FDCAN_IE_PEAE | FDCAN_IE_PEDE | FDCAN_IE_ARAE | FDCAN_IE_TCFE) /*!< FDCAN interrupts enable mask */

#define FDCAN_ILS_MSK      (FDCAN_ILS_RXFIFO0 | FDCAN_ILS_RXFIFO1 | FDCAN_ILS_SMSG | FDCAN_ILS_TFERR | FDCAN_ILS_MISC \
                            | FDCAN_ILS_BERR | FDCAN_ILS_PERR) /*!< FDCAN interrupts group mask in ILS register */

#define FDCAN_GLOBAL_TIMEOUT_MS (10U) /*!< 10ms timeout */

#define FDCAN_TX_EVENT_FIFO_MSK (HAL_FDCAN_FLAG_TX_EVT_FIFO_ELEM_LOST | HAL_FDCAN_FLAG_TX_EVT_FIFO_FULL \
                                 | HAL_FDCAN_FLAG_TX_EVT_FIFO_NEW_DATA) /*!< Define the Tx event FIFO IT related mask */
#define FDCAN_RX_FIFO_0_MSK (HAL_FDCAN_FLAG_RX_FIFO_0_MSG_LOST | HAL_FDCAN_FLAG_RX_FIFO_0_FULL \
                             | HAL_FDCAN_FLAG_RX_FIFO_0_NEW_MSG) /*!< Define the Rx FIFO 0 IT related mask     */
#define FDCAN_RX_FIFO_1_MSK (HAL_FDCAN_FLAG_RX_FIFO_1_MSG_LOST | HAL_FDCAN_FLAG_RX_FIFO_1_FULL \
                             | HAL_FDCAN_FLAG_RX_FIFO_1_NEW_MSG) /*!< Define the Rx FIFO 1 IT related mask     */


#define FDCAN_RAM_FLS_NBR  (28U) /*!< Max. filter List Standard number  */
#define FDCAN_RAM_FLE_NBR  (8U)  /*!< Max. filter List Extended number  */
#define FDCAN_RAM_RF0_NBR  (3U)  /*!< Rx FIFO 0 elements number         */
#define FDCAN_RAM_RF1_NBR  (3U)  /*!< Rx FIFO 1 elements number         */
#define FDCAN_RAM_TEF_NBR  (3U)  /*!< Tx event FIFO elements number     */
#define FDCAN_RAM_TFQ_NBR  (3U)  /*!< Tx FIFO/Queue elements number     */

#define FDCAN_RAM_FLS_SIZE (4U)  /*!< Filter Standard element size in bytes */
#define FDCAN_RAM_FLE_SIZE (8U)  /*!< Filter Extended element size in bytes */
#define FDCAN_RAM_RF0_SIZE (72U) /*!< Rx FIFO 0 elements size in bytes      */
#define FDCAN_RAM_RF1_SIZE (72U) /*!< Rx FIFO 1 elements size in bytes      */
#define FDCAN_RAM_TEF_SIZE (8U)  /*!< Tx event FIFO elements size in bytes  */
#define FDCAN_RAM_TFQ_SIZE (72U) /*!< Tx FIFO/Queue elements size in bytes  */

#define FDCAN_RAM_FLSSA (0U)                                                           /*!< Filter List Standard Start Address */
#define FDCAN_RAM_FLESA ((FDCAN_RAM_FLSSA + (FDCAN_RAM_FLS_NBR * FDCAN_RAM_FLS_SIZE))) /*!< Filter List Extended Start Address */
#define FDCAN_RAM_RF0SA ((FDCAN_RAM_FLESA + (FDCAN_RAM_FLE_NBR * FDCAN_RAM_FLE_SIZE))) /*!< Rx FIFO 0 Start Address            */
#define FDCAN_RAM_RF1SA ((FDCAN_RAM_RF0SA + (FDCAN_RAM_RF0_NBR * FDCAN_RAM_RF0_SIZE))) /*!< Rx FIFO 1 Start Address            */
#define FDCAN_RAM_TEFSA ((FDCAN_RAM_RF1SA + (FDCAN_RAM_RF1_NBR * FDCAN_RAM_RF1_SIZE))) /*!< Tx event FIFO Start Address        */
#define FDCAN_RAM_TFQSA ((FDCAN_RAM_TEFSA + (FDCAN_RAM_TEF_NBR * FDCAN_RAM_TEF_SIZE))) /*!< Tx FIFO/Queue Start Address        */
#define FDCAN_RAM_SIZE  ((FDCAN_RAM_TFQSA + (FDCAN_RAM_TFQ_NBR * FDCAN_RAM_TFQ_SIZE))) /*!< Message RAM size                   */

#define FDCAN_STD_FILTER_TYPE_POS   (30U)                                      /*!< Position of the Standard filter type field          */
#define FDCAN_STD_FILTER_TYPE_MSK   (0x3UL << FDCAN_STD_FILTER_TYPE_POS)       /*!< Standard filter type mask field                     */

#define FDCAN_STD_FILTER_CONFIG_POS (27U)                                      /*!< Position of the Standard filter configuration field */
#define FDCAN_STD_FILTER_CONFIG_MSK (0x7UL << FDCAN_STD_FILTER_CONFIG_POS)     /*!< Standard filter config mask field                   */

#define FDCAN_STD_FILTER_ID1_POS    (16U)                                      /*!< Position of the Standard ID1 field                  */
#define FDCAN_STD_FILTER_ID1_MSK    (0x7FFUL << FDCAN_STD_FILTER_ID1_POS)      /*!< Standard filter ID1 mask field                      */

#define FDCAN_STD_FILTER_ID2_POS    (0U)                                       /*!< Position of the Standard ID2 field                  */
#define FDCAN_STD_FILTER_ID2_MSK    (0x7FFUL << FDCAN_STD_FILTER_ID2_POS)      /*!< Standard filter ID2 mask field                      */

#define FDCAN_EXT_FILTER_CONFIG_POS (29U)                                      /*!< Position of the Extended filter configuration field */
#define FDCAN_EXT_FILTER_CONFIG_MSK (0x7UL << FDCAN_EXT_FILTER_CONFIG_POS)     /*!< Extended filter config mask field                   */

#define FDCAN_EXT_FILTER_TYPE_POS   (30U)                                      /*!< Position of the Extended filter type field          */
#define FDCAN_EXT_FILTER_TYPE_MSK   (0x3UL << FDCAN_EXT_FILTER_TYPE_POS)       /*!< Extended filter type mask field                     */

#define FDCAN_EXT_FILTER_ID1_POS    (0U)                                       /*!< Position of the Extended ID1 field                  */
#define FDCAN_EXT_FILTER_ID1_MSK    (0x1FFFFFFFUL << FDCAN_EXT_FILTER_ID1_POS) /*!< Extended filter ID1 mask field                      */

#define FDCAN_EXT_FILTER_ID2_POS    (0U)                                       /*!< Position of the Extended ID2 field                  */
#define FDCAN_EXT_FILTER_ID2_MSK    (0x1FFFFFFFUL << FDCAN_EXT_FILTER_ID2_POS) /*!< Extended filter ID2 mask field                      */

#define FDCAN_STD_FILTER_ID_POS     (18U)                                      /*!< ID standard filter position in Tx/Rx/event header   */

#define HAL_FDCAN_IT_GROUP_MSK (HAL_FDCAN_IT_GROUP_RX_FIFO_0        \
                                | HAL_FDCAN_IT_GROUP_RX_FIFO_1      \
                                | HAL_FDCAN_IT_GROUP_STATUS_MSG     \
                                | HAL_FDCAN_IT_GROUP_TX_FIFO_ERROR  \
                                | HAL_FDCAN_IT_GROUP_MISC           \
                                | HAL_FDCAN_IT_GROUP_BIT_LINE_ERROR \
                                | HAL_FDCAN_IT_GROUP_PROTOCOL_ERROR)  /*!< Interrupts group mask */

/**
  * @brief Minimum and maximum values for FDCAN nominal bit timing parameters.
  */
#define FDCAN_NOMINAL_PRES_MIN  (1U)    /*!< Minimum nominal prescaler value      */
#define FDCAN_NOMINAL_PRES_MAX  (512U)  /*!< Maximum nominal prescaler value      */
#define FDCAN_NOMINAL_SJW_MIN   (1U)    /*!< Minimum nominal SJW value            */
#define FDCAN_NOMINAL_SJW_MAX   (128U)  /*!< Maximum nominal SJW value            */
#define FDCAN_NOMINAL_TSEG1_MIN (1U)    /*!< Minimum nominal time segment 1 value */
#define FDCAN_NOMINAL_TSEG1_MAX (256U)  /*!< Maximum nominal time segment 1 value */
#define FDCAN_NOMINAL_TSEG2_MIN (1U)    /*!< Minimum nominal time segment 2 value */
#define FDCAN_NOMINAL_TSEG2_MAX (128U)  /*!< Maximum nominal time segment 2 value */

/**
  * @brief Minimum and maximum values for FDCAN data bit timing parameters.
  */
#define FDCAN_DATA_PRES_MIN     (1U)    /*!< Minimum data prescaler value      */
#define FDCAN_DATA_PRES_MAX     (32U)   /*!< Maximum data prescaler value      */
#define FDCAN_DATA_SJW_MIN      (1U)    /*!< Minimum data SJW value            */
#define FDCAN_DATA_SJW_MAX      (16U)   /*!< Maximum data SJW value            */
#define FDCAN_DATA_TSEG1_MIN    (1U)    /*!< Minimum data time segment 1 value */
#define FDCAN_DATA_TSEG1_MAX    (32U)   /*!< Maximum data time segment 1 value */
#define FDCAN_DATA_TSEG2_MIN    (1U)    /*!< Minimum data time segment 2 value */
#define FDCAN_DATA_TSEG2_MAX    (16U)   /*!< Maximum data time segment 2 value */

/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Private_Macros FDCAN Private Macros
  * @{
  */

/**
  * @brief  Get the FDCAN hardware instance from a handle.
  * @param  handle Pointer to a @ref hal_fdcan_handle_t structure.
  * @retval FDCAN_GlobalTypeDef* Pointer to the FDCAN peripheral instance.
  */
#define FDCAN_GET_INSTANCE(handle) ((FDCAN_GlobalTypeDef *)((uint32_t)(handle)->instance))

/**
  * @brief  Get the FDCAN CONFIG hardware instance from a handle.
  * @param  handle Pointer to a @ref hal_fdcan_handle_t structure.
  * @retval FDCAN_Config_TypeDef* Pointer to the FDCAN CONFIG peripheral instance.
  */
#define FDCAN_CONFIG_GET_INSTANCE(handle) FDCAN_MULTI_INSTANCE_CONFIG((FDCAN_GlobalTypeDef *) \
                                                                      (uint32_t)(handle)->instance)

/**
  * @brief  Get the FDCAN message element size in 32-bit words.
  * @param  element FDCAN data field size encoding. See @ref hal_fdcan_data_field_size_t.
  * @retval uint32_t Element size in words, including the FDCAN header words.
  */
#define FDCAN_GET_ELEMENT_SIZE_WORDS(element) ((uint32_t)(fdcan_lut_eltsize2words[(uint32_t)element]))

/**
  * @brief  Get payload size in bytes from a Data Length Code (DLC).
  * @param  data_length_code Data length code. See @ref hal_fdcan_data_length_code_t.
  * @retval uint32_t Payload length in bytes.
  */
#define FDCAN_GET_DATA_LENGTH_BYTES(data_length_code) ((uint32_t)(fdcan_lut_dlc2bytes[(uint32_t)data_length_code]))

/**
  * @brief  Check if the frame format value is valid.
  * @param  format Frame format value to check. See @ref hal_fdcan_frame_format_t.
  * @retval SET   Frame format is valid.
  * @retval RESET Frame format is invalid.
  */
#define IS_FDCAN_FRAME_FORMAT(format) (((format) == HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN)  \
                                       || ((format) == HAL_FDCAN_FRAME_FORMAT_FD_NO_BRS) \
                                       || ((format) == HAL_FDCAN_FRAME_FORMAT_FD_BRS))

/**
  * @brief  Check if the transmit pause value is valid.
  * @param  transmit Transmit pause value to check. See @ref hal_fdcan_transmit_pause_state_t.
  * @retval SET   Transmit pause value is valid.
  * @retval RESET Transmit pause value is invalid.
  */
#define IS_FDCAN_TRANSMIT_PAUSE(transmit) (((transmit) == HAL_FDCAN_TRANSMIT_PAUSE_DISABLE)   \
                                           || ((transmit) == HAL_FDCAN_TRANSMIT_PAUSE_ENABLE))

/**
  * @brief  Check if the protocol exception value is valid.
  * @param  protocol Protocol exception value to check. See @ref hal_fdcan_protocol_exception_state_t.
  * @retval SET   Protocol exception value is valid.
  * @retval RESET Protocol exception value is invalid.
  */
#define IS_FDCAN_PROTOCOL_EXCEPTION(protocol) (((protocol) == HAL_FDCAN_PROTOCOL_EXCEPTION_DISABLE)   \
                                               || ((protocol) == HAL_FDCAN_PROTOCOL_EXCEPTION_ENABLE))

/**
  * @brief  Check if the automatic retransmission value is valid.
  * @param  transmission Automatic retransmission value to check. See @ref hal_fdcan_auto_retransmission_state_t.
  * @retval SET   Automatic retransmission value is valid.
  * @retval RESET Automatic retransmission value is invalid.
  */
#define IS_FDCAN_AUTO_RETRANSMISSION(transmission) (((transmission) == HAL_FDCAN_AUTO_RETRANSMISSION_DISABLE)   \
                                                    || ((transmission) == HAL_FDCAN_AUTO_RETRANSMISSION_ENABLE))

/**
  * @brief  Check if the mode value is valid.
  * @param  mode Mode value to check. See @ref hal_fdcan_mode_t.
  * @retval SET   Mode value is valid.
  * @retval RESET Mode value is invalid.
  */
#define IS_FDCAN_MODE(mode) (((mode) == HAL_FDCAN_MODE_NORMAL)                  \
                             || ((mode) == HAL_FDCAN_MODE_RESTRICTED_OPERATION) \
                             || ((mode) == HAL_FDCAN_MODE_BUS_MONITORING)       \
                             || ((mode) == HAL_FDCAN_MODE_INTERNAL_LOOPBACK)    \
                             || ((mode) == HAL_FDCAN_MODE_EXTERNAL_LOOPBACK))

/**
  * @brief  Check if the clock divider value is valid.
  * @param  ckdiv Clock divider value to check. See @ref hal_fdcan_clock_divider_t.
  * @retval SET   Clock divider value is valid.
  * @retval RESET Clock divider value is invalid.
  */
#define IS_FDCAN_CKDIV(ckdiv) (((ckdiv) == HAL_FDCAN_CLOCK_DIV_1)     \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_2)  \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_4)  \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_6)  \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_8)  \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_10) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_12) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_14) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_16) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_18) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_20) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_22) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_24) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_26) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_28) \
                               || ((ckdiv) == HAL_FDCAN_CLOCK_DIV_30))

/**
  * @brief  Check if the value is less than or equal to the maximum allowed value.
  * @param  value Value to check.
  * @param  max   Maximum allowed value.
  * @retval SET   Value is within the maximum limit.
  * @retval RESET Value exceeds the maximum limit.
  */
#define IS_FDCAN_MAX_VALUE(value, max) ((value) <= (max))

/**
  * @brief  Check if the value is greater than or equal to the minimum allowed value.
  * @param  value Value to check.
  * @param  min   Minimum allowed value.
  * @retval SET   Value is within the minimum limit.
  * @retval RESET Value is below the minimum limit.
  */
#define IS_FDCAN_MIN_VALUE(value, min) ((value) >= (min))

/**
  * @brief  Check if a value is within a specified [min, max] range (inclusive).
  * @param  value Value to check.
  * @param  min   Minimum allowed value.
  * @param  max   Maximum allowed value.
  * @retval SET   Value is within the specified range.
  * @retval RESET Value is outside the specified range.
  */
#define IS_FDCAN_IN_RANGE(value, min, max) (IS_FDCAN_MIN_VALUE(value, min) && IS_FDCAN_MAX_VALUE(value, max))

/**
  * @brief  Check if the nominal prescaler value is valid.
  * @param  prescaler Nominal prescaler value to check.
  * @retval SET   Prescaler value is valid.
  * @retval RESET Prescaler value is invalid.
  */
#define IS_FDCAN_NOMINAL_PRESC(prescaler) IS_FDCAN_IN_RANGE((prescaler), FDCAN_NOMINAL_PRES_MIN, FDCAN_NOMINAL_PRES_MAX)

/**
  * @brief  Check if the nominal jump width value is valid.
  * @param  sjw Nominal jump width value to check.
  * @retval SET   Jump width value is valid.
  * @retval RESET Jump width value is invalid.
  */
#define IS_FDCAN_NOMINAL_SJW(sjw)         IS_FDCAN_IN_RANGE((sjw), FDCAN_NOMINAL_SJW_MIN, FDCAN_NOMINAL_SJW_MAX)

/**
  * @brief  Check if the nominal time segment 1 value is valid.
  * @param  tseg1 Nominal time segment 1 value to check.
  * @retval SET   Time segment 1 value is valid.
  * @retval RESET Time segment 1 value is invalid.
  */
#define IS_FDCAN_NOMINAL_TSEG1(tseg1)     IS_FDCAN_IN_RANGE((tseg1), FDCAN_NOMINAL_TSEG1_MIN, FDCAN_NOMINAL_TSEG1_MAX)

/**
  * @brief  Check if the nominal time segment 2 value is valid.
  * @param  tseg2 Nominal time segment 2 value to check.
  * @retval SET   Time segment 2 value is valid.
  * @retval RESET Time segment 2 value is invalid.
  */
#define IS_FDCAN_NOMINAL_TSEG2(tseg2)     IS_FDCAN_IN_RANGE((tseg2), FDCAN_NOMINAL_TSEG2_MIN, FDCAN_NOMINAL_TSEG2_MAX)

/**
  * @brief  Check if the data prescaler value is valid.
  * @param  prescaler Data prescaler value to check.
  * @retval SET   Data prescaler value is valid.
  * @retval RESET Data prescaler value is invalid.
  */
#define IS_FDCAN_DATA_PRESC(prescaler)     IS_FDCAN_IN_RANGE((prescaler), FDCAN_DATA_PRES_MIN, FDCAN_DATA_PRES_MAX)

/**
  * @brief  Check if the data jump width value is valid.
  * @param  sjw Data jump width value to check.
  * @retval SET   Data jump width value is valid.
  * @retval RESET Data jump width value is invalid.
  */
#define IS_FDCAN_DATA_SJW(sjw)            IS_FDCAN_IN_RANGE((sjw), FDCAN_DATA_SJW_MIN, FDCAN_DATA_SJW_MAX)

/**
  * @brief  Check if the data time segment 1 value is valid.
  * @param  tseg1 Data time segment 1 value to check.
  * @retval SET   Data time segment 1 value is valid.
  * @retval RESET Data time segment 1 value is invalid.
  */
#define IS_FDCAN_DATA_TSEG1(tseg1)        IS_FDCAN_IN_RANGE((tseg1), FDCAN_DATA_TSEG1_MIN, FDCAN_DATA_TSEG1_MAX)

/**
  * @brief  Check if the data time segment 2 value is valid.
  * @param  tseg2 Data time segment 2 value to check.
  * @retval SET   Data time segment 2 value is valid.
  * @retval RESET Data time segment 2 value is invalid.
  */
#define IS_FDCAN_DATA_TSEG2(tseg2)        IS_FDCAN_IN_RANGE((tseg2), FDCAN_DATA_TSEG2_MIN, FDCAN_DATA_TSEG2_MAX)


/**
  * @brief  Check if the ID type value is valid.
  * @param  id_type ID type value to check. See @ref hal_fdcan_id_type_t.
  * @retval SET   ID type is valid.
  * @retval RESET ID type is invalid.
  */
#define IS_FDCAN_ID_TYPE(id_type) (((id_type) == HAL_FDCAN_ID_STANDARD)    \
                                   || ((id_type) == HAL_FDCAN_ID_EXTENDED))

/**
  * @brief  Check if the Tx mode value is valid.
  * @param  tx_mode Tx mode value to check. See @ref hal_fdcan_tx_mode_t.
  * @retval SET   Tx mode is valid.
  * @retval RESET Tx mode is invalid.
  */
#define IS_FDCAN_TX_MODE(tx_mode) (((tx_mode) == HAL_FDCAN_TX_MODE_FIFO)    \
                                   || ((tx_mode) == HAL_FDCAN_TX_MODE_QUEUE))

/**
  * @brief  Check if the filter configuration value is valid.
  * @param  config Filter configuration value to check. See @ref hal_fdcan_filter_config_t.
  * @retval SET   Filter configuration is valid.
  * @retval RESET Filter configuration is invalid.
  */
#define IS_FDCAN_FILTER_CFG(config) (((config) == HAL_FDCAN_FILTER_DISABLE)          \
                                     || ((config) == HAL_FDCAN_FILTER_TO_RX_FIFO_0)    \
                                     || ((config) == HAL_FDCAN_FILTER_TO_RX_FIFO_1)    \
                                     || ((config) == HAL_FDCAN_FILTER_REJECT)        \
                                     || ((config) == HAL_FDCAN_FILTER_HP)            \
                                     || ((config) == HAL_FDCAN_FILTER_TO_RX_FIFO_0_HP) \
                                     || ((config) == HAL_FDCAN_FILTER_TO_RX_FIFO_1_HP))


/**
  * @brief  Check if the Tx location list is valid.
  * @param  location Tx buffer location value to check.
  * @retval SET   Tx location is valid.
  * @retval RESET Tx location is invalid.
  */
#define IS_FDCAN_TX_LOCATION_LIST(location) IS_FDCAN_IN_RANGE((location), HAL_FDCAN_TX_BUFFER_0, \
                                                              HAL_FDCAN_TX_BUFFER_ALL)

/**
  * @brief  Check if the Rx FIFO selection is valid.
  * @param  fifo Rx FIFO selection value to check. See @ref hal_fdcan_rx_location_t.
  * @retval SET   Rx FIFO selection is valid.
  * @retval RESET Rx FIFO selection is invalid.
  */
#define IS_FDCAN_RX_FIFO(fifo) (((fifo) == HAL_FDCAN_RX_FIFO_0)    \
                                || ((fifo) == HAL_FDCAN_RX_FIFO_1))

/**
  * @brief  Check if the Rx FIFO mode value is valid.
  * @param  mode Rx FIFO mode value to check. See @ref hal_fdcan_rx_fifo_mode_t.
  * @retval SET   Rx FIFO mode is valid.
  * @retval RESET Rx FIFO mode is invalid.
  */
#define IS_FDCAN_RX_FIFO_MODE(mode) (((mode) == HAL_FDCAN_RX_FIFO_MODE_BLOCKING)     \
                                     || ((mode) == HAL_FDCAN_RX_FIFO_MODE_OVERWRITE))

/**
  * @brief  Check that the address to retrieve the filter is valid.
  * @param  address Address pointer to check.
  * @retval SET   Address is valid (not NULL).
  * @retval RESET Address is invalid (NULL).
  */
#define IS_ADDRESS_VALID(address) ((address) != NULL)

/**
  * @brief  Check if the standard filter type value is valid.
  * @param  type Standard filter type value to check. See @ref hal_fdcan_filter_type_t.
  * @retval SET   Standard filter type is valid.
  * @retval RESET Standard filter type is invalid.
  */
#define IS_FDCAN_STD_FILTER_TYPE(type) (((type) == HAL_FDCAN_FILTER_TYPE_RANGE)      \
                                        || ((type) == HAL_FDCAN_FILTER_TYPE_DUAL)    \
                                        || ((type) == HAL_FDCAN_FILTER_TYPE_CLASSIC))

/**
  * @brief  Check if the extended filter type value is valid.
  * @param  type Extended filter type value to check. See @ref hal_fdcan_filter_type_t.
  * @retval SET   Extended filter type is valid.
  * @retval RESET Extended filter type is invalid.
  */
#define IS_FDCAN_EXT_FILTER_TYPE(type) (((type) == HAL_FDCAN_FILTER_TYPE_RANGE)            \
                                        || ((type) == HAL_FDCAN_FILTER_TYPE_DUAL)          \
                                        || ((type) == HAL_FDCAN_FILTER_TYPE_CLASSIC)       \
                                        || ((type) == HAL_FDCAN_FILTER_TYPE_RANGE_NO_EIDM))

/**
  * @brief  Check if the buffer belongs to the allowed Tx complete buffers list.
  * @param  buff Buffer bitmask to check.
  * @retval SET   Buffer(s) are valid.
  * @retval RESET Buffer(s) are invalid.
  */
#define IS_FDCAN_TX_COMPLETE_BUFFERS(buff) ((((buff) & ~HAL_FDCAN_IT_TX_CPLT_BUFFER_ALL) == 0U) && ((buff) != 0U))

/**
  * @brief  Check if the buffer belongs to the allowed Tx abort buffers list.
  * @param  buff Buffer bitmask to check.
  * @retval SET   Buffer(s) are valid.
  * @retval RESET Buffer(s) are invalid.
  */
#define IS_FDCAN_TX_ABORT_BUFFERS(buff) ((((buff) & ~HAL_FDCAN_IT_TX_ABORT_BUFFER_ALL) == 0U) && ((buff) != 0U))

/**
  * @brief  Check if the interrupt line value is valid.
  * @param  it_line Interrupt line value to check.
  * @retval SET   Interrupt line is valid.
  * @retval RESET Interrupt line is invalid.
  */
#define IS_FDCAN_IT_LINE(it_line) ((((it_line) & ~(HAL_FDCAN_IT_LINE_0 | HAL_FDCAN_IT_LINE_1)) == 0U) \
                                   && ((it_line) != 0U))

/**
  * @brief  Check if only a single bit is set in the value.
  * @param  value Value to check.
  * @retval SET   Only a single bit is set.
  * @retval RESET Zero or multiple bits are set.
  */
#define IS_FDCAN_SINGLE_BIT_SET(value) (((value) > 0U) && (((value) & ((value) - 1U)) == 0U))

/**
  * @brief  Check if the interrupt list is valid.
  * @param  it Interrupt bitmask to check.
  * @retval SET   Interrupt list is valid.
  * @retval RESET Interrupt list is invalid.
  */
#define IS_FDCAN_IT(it) (((it) != 0U) &&  (((it) & (FDCAN_IR_MSK)) != 0U))

/**
  * @brief  Check if the interrupt group is valid.
  * @param  group Interrupt group bitmask to check.
  * @retval SET   Interrupt group is valid.
  * @retval RESET Interrupt group is invalid.
  */
#define IS_FDCAN_IT_GROUP(group) (((group) & ~HAL_FDCAN_IT_GROUP_MSK) == 0U)

/**
  * @brief  Check if the non-matching frame destination is valid.
  * @param  destination Non-matching frame destination value. See @ref hal_fdcan_non_matching_acceptance_rules_t.
  * @retval SET   Destination is valid.
  * @retval RESET Destination is invalid.
  */
#define IS_FDCAN_NON_MATCHING(destination) (((destination) == HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0)    \
                                            || ((destination) == HAL_FDCAN_NO_MATCH_TO_RX_FIFO_1) \
                                            || ((destination) == HAL_FDCAN_NO_MATCH_REJECT))

/**
  * @brief  Check if the reject remote mode value is valid.
  * @param  destination Reject remote mode value to check. See @ref hal_fdcan_remote_acceptance_frame_t.
  * @retval SET   Reject remote mode is valid.
  * @retval RESET Reject remote mode is invalid.
  */
#define IS_FDCAN_REJECT_REMOTE(destination) (((destination) == HAL_FDCAN_REMOTE_ACCEPT)    \
                                             || ((destination) == HAL_FDCAN_REMOTE_REJECT))

/**
  * @brief  Check if the timestamp source value is valid.
  * @param  operation Timestamp source value to check. See @ref hal_fdcan_timestamp_source_t.
  * @retval SET   Timestamp source is valid.
  * @retval RESET Timestamp source is invalid.
  */
#define IS_FDCAN_TIMESTAMP_SOURCE(operation) (((operation) == HAL_FDCAN_TIMESTAMP_SOURCE_INTERNAL)    \
                                              || ((operation) == HAL_FDCAN_TIMESTAMP_SOURCE_ZERO)     \
                                              || ((operation) == HAL_FDCAN_TIMESTAMP_SOURCE_EXTERNAL))

/**
  * @brief  Check if the timestamp prescaler value is valid.
  * @param  prescaler Timestamp prescaler value to check. See @ref hal_fdcan_timestamp_prescaler_t.
  * @retval SET   Timestamp prescaler is valid.
  * @retval RESET Timestamp prescaler is invalid.
  */
#define IS_FDCAN_TIMESTAMP_PRESCALER(prescaler) (((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_1)     \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_2)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_3)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_4)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_5)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_6)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_7)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_8)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_9)  \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_10) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_11) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_12) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_13) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_14) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_15) \
                                                 || ((prescaler) == HAL_FDCAN_TIMESTAMP_PRESC_16))

/**
  * @brief  Check if the timeout operation value is valid.
  * @param  operation Timeout operation value to check. See @ref hal_fdcan_timeout_operation_t.
  * @retval SET   Timeout operation is valid.
  * @retval RESET Timeout operation is invalid.
  */
#define IS_FDCAN_TIMEOUT(operation) (((operation) == HAL_FDCAN_TIMEOUT_CONTINUOUS)       \
                                     || ((operation) == HAL_FDCAN_TIMEOUT_TX_EVENT_FIFO) \
                                     || ((operation) == HAL_FDCAN_TIMEOUT_RX_FIFO_0)     \
                                     || ((operation) == HAL_FDCAN_TIMEOUT_RX_FIFO_1))

/**
  * @}
  */

/* Private function prototypes ---------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Private_Functions FDCAN Private Functions
  * @{
  */

/* Private function to set the FDCAN nominal bit timing */
static void FDCAN_SetNominalBitTiming(FDCAN_GlobalTypeDef *fdcan, const hal_fdcan_nominal_bit_timing_t *p_timing);
/* Private function to retrieve the FDCAN nominal bit timing */
static void FDCAN_GetNominalBitTiming(const FDCAN_GlobalTypeDef *fdcan, hal_fdcan_nominal_bit_timing_t *p_timing);
/* Private function to set the FDCAN data bit timing */
static void FDCAN_SetDataBitTiming(FDCAN_GlobalTypeDef *fdcan, const hal_fdcan_data_bit_timing_t *p_timing);
/* Private function to retrieve the FDCAN data bit timing */
static void FDCAN_GetDataBitTiming(const FDCAN_GlobalTypeDef *fdcan, hal_fdcan_data_bit_timing_t *p_timing);

/* Private function to  calculate each RAM block start address and size */
static void FDCAN_ComputeRAMBlockAddresses(hal_fdcan_handle_t *hfdcan, const hal_fdcan_config_t *p_config);

/* Private function to get the message RAM configurations */
static void FDCAN_GetMessageRAMConfig(const FDCAN_GlobalTypeDef *p_fdcan, hal_fdcan_config_t *p_config);

/* Private function to copy Tx message to the message RAM */
static void FDCAN_CopyMessageToRAM(const hal_fdcan_handle_t *hfdcan,
                                   const hal_fdcan_tx_header_t *p_tx_element_header,
                                   const uint8_t *p_tx_data, uint32_t tx_buffer_idx);

/* Private function to retrieve the fdcan operation mode */
static hal_fdcan_mode_t FDCAN_GetMode(const uint32_t register_value);

/* Private function to handle flags during polling */
static hal_status_t FDCAN_WaitOnFlagUntilTimeout(const FDCAN_GlobalTypeDef *fdcan, uint32_t flag, uint32_t status,
                                                 uint32_t timeout_ms);

/* Private function to handle clock stop request */
static hal_status_t FDCAN_ResetClockStopRequest(FDCAN_GlobalTypeDef *fdcan);

/* Private function to handle initialization request */
static hal_status_t FDCAN_InitRequest(FDCAN_GlobalTypeDef *fdcan);

/* Private function to handles and dispatches FDCAN interrupt flags for a given driver instance */
static void FDCAN_HandleInterruptFlags(hal_fdcan_handle_t *hfdcan, uint32_t it_flags);

/* Private function to returns interrupt flags associated with a specific interrupt line */
static uint32_t FDCAN_GetLineSpecificInterruptFlags(const FDCAN_GlobalTypeDef *p_fdcanx, uint32_t it_line);
/**
  * @}
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup FDCAN_Exported_Functions HAL FDCAN Functions
  * @{
  */

/** @addtogroup FDCAN_Exported_Functions_Group1
  * @{
A set of functions allowing to initialize and deinitialize the FDCAN peripheral:
    - HAL_FDCAN_Init()    : Initialize the selected device with the FDCAN instance.
    - HAL_FDCAN_DeInit()  : Restore the default configuration of the selected FDCAN peripheral.
  */

/**
  * @brief  Initializes the FDCAN peripheral according to the associated handle.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  instance HAL FDCAN instance.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM hfdcan is NULL.
  * @retval HAL_ERROR         HAL FDCAN semaphore creation has failed (USE_HAL_MUTEX is set to 1).
  */
hal_status_t HAL_FDCAN_Init(hal_fdcan_handle_t *hfdcan, hal_fdcan_t instance)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hfdcan == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_ALL_INSTANCE((FDCAN_GlobalTypeDef *)((uint32_t)instance)));

  hfdcan->instance = instance;

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
  hfdcan->p_tx_event_fifo_cb         = HAL_FDCAN_TxEventFifoCallback;             /* TxEventFifoCallback              */
  hfdcan->p_rx_fifo_0_cb             = HAL_FDCAN_RxFifo0Callback;                 /* RxFifo0Callback                  */
  hfdcan->p_rx_fifo_1_cb             = HAL_FDCAN_RxFifo1Callback;                 /* RxFifo1Callback                  */
  hfdcan->p_tx_fifo_empty_cb         = HAL_FDCAN_TxFifoEmptyCallback;             /* TxFifoEmptyCallback              */
  hfdcan->p_tx_buffer_complete_cb    = HAL_FDCAN_TxBufferCompleteCallback;        /* TxBufferCompleteCallback         */
  hfdcan->p_tx_buffer_abort_cb       = HAL_FDCAN_TxBufferAbortCallback;           /* TxBufferAbortCallback            */
  hfdcan->p_high_priority_msg_cb     = HAL_FDCAN_HighPriorityMessageCallback;     /* HighPriorityMessageCallback      */
  hfdcan->p_ts_wraparound_cb         = HAL_FDCAN_TimestampWraparoundCallback;     /* TimestampWraparoundCallback      */
  hfdcan->p_error_cb                 = HAL_FDCAN_ErrorCallback;                   /* ErrorCallback                    */
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */

  hfdcan->latest_tx_fifo_q_request = 0U;

#if defined(USE_HAL_FDCAN_USER_DATA) && (USE_HAL_FDCAN_USER_DATA == 1)
  hfdcan->p_user_data = (void *) NULL;
#endif /* USE_HAL_FDCAN_USER_DATA */

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
  hfdcan->last_error_codes = HAL_FDCAN_ERROR_NONE;
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

#if defined(USE_HAL_FDCAN_CLK_ENABLE_MODEL) && (USE_HAL_FDCAN_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_FDCAN_EnableClock();
#endif /* USE_HAL_FDCAN_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  if (HAL_OS_SemaphoreCreate(&hfdcan->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  hfdcan->global_state = HAL_FDCAN_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the FDCAN driver for the given handle and disable the peripheral.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  */
void HAL_FDCAN_DeInit(hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  ASSERT_DBG_PARAM(IS_FDCAN_ALL_INSTANCE(FDCAN_GET_INSTANCE(hfdcan)));

  /* Disable interrupt lines */
  STM32_CLEAR_BIT(p_fdcanx->ILE, (HAL_FDCAN_IT_LINE_0 | HAL_FDCAN_IT_LINE_1));

  /* Clear all interrupts enable */
  STM32_CLEAR_BIT(p_fdcanx->IE, FDCAN_IE_MSK);

  /* Abort any pending Tx buffer */
  STM32_WRITE_REG(p_fdcanx->TXBCR, HAL_FDCAN_TX_BUFFER_ALL);

  (void)FDCAN_ResetClockStopRequest(p_fdcanx);

  /* Stop the FDCAN by entering initialization mode */
  (void)FDCAN_InitRequest(p_fdcanx);

  /* Clear all the potentially pending interrupts */
  STM32_SET_BIT(p_fdcanx->IR, FDCAN_IR_MSK);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&hfdcan->semaphore);
#endif /* USE_HAL_MUTEX */

  hfdcan->global_state = HAL_FDCAN_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup FDCAN_Exported_Functions_Group2
  * @{
A set of functions allowing to enter and exit sleep mode for the FDCAN peripheral:
    - HAL_FDCAN_EnterPowerDownMode(): Enter in power down (sleep mode).
    - HAL_FDCAN_ExitPowerDownMode():  Exit power down (sleep mode).
  */

/**
  * @brief  Sets the FDCAN peripheral into power down (sleep) mode.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @note   In this mode, the FDCAN peripheral stops all bus activity, disables its clock, and reduces power
  *         consumption. The FDCAN can be woken up by a dedicated wakeup event or by software.
  *         This mode is specific to the FDCAN peripheral and is not to be confused with the STM32 MCU stop or standby.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR FDCAN cannot be set in power down mode.
  */
hal_status_t HAL_FDCAN_EnterPowerDownMode(hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  STM32_SET_BIT(p_fdcanx->CCCR, FDCAN_CCCR_CSR);

  /* Wait until FDCAN is ready for power down */
  if (FDCAN_WaitOnFlagUntilTimeout(p_fdcanx, FDCAN_CCCR_CSA, 0U, FDCAN_GLOBAL_TIMEOUT_MS) != HAL_TIMEOUT)
  {
    hfdcan->global_state = HAL_FDCAN_STATE_POWER_DOWN;
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Exit the FDCAN peripheral power down mode (sleep mode).
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR FDCAN cannot leave the power down mode.
  */
hal_status_t HAL_FDCAN_ExitPowerDownMode(hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (FDCAN_ResetClockStopRequest(p_fdcanx) != HAL_ERROR)
  {
    /* Return to normal operation */
    STM32_CLEAR_BIT(p_fdcanx->CCCR, FDCAN_CCCR_INIT);

    hfdcan->global_state = HAL_FDCAN_STATE_ACTIVE;

    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */

/** @addtogroup FDCAN_Exported_Functions_Group3
  * @{
There are two categories of HAL configuration set of functions for the peripheral configuration:
   - Global configuration functions that set or get the overall peripheral configuration.
   - Unitary configuration functions that modify or retrieve individual configuration items.

Items that can alter other config parameters must not be handled within unitary set of functions.

   - This section provides functions allowing to:
      - HAL_FDCAN_SetConfig()                        : Configure the HAL FDCAN peripheral instance into a ready to
                                                       use state (idle) according to the user parameters.
      - HAL_FDCAN_GetConfig()                        : Get the peripheral configuration.
      - HAL_FDCAN_SetNominalBitTiming()              : Configure the nominal bit timing.
      - HAL_FDCAN_GetNominalBitTiming()              : Get the nominal bit timing configuration.
      - HAL_FDCAN_SetDataBitTiming()                 : Configure the data bit timing.
      - HAL_FDCAN_GetDataBitTiming()                 : Get the data bit timing configuration.
      - HAL_FDCAN_SetFilter()                        : Configure the FDCAN reception filter.
      - HAL_FDCAN_GetFilter()                        : Get the FDCAN reception filter configuration.
      - HAL_FDCAN_SetGlobalFilter()                  : Configure the FDCAN global filter.
      - HAL_FDCAN_GetGlobalFilter()                  : Get the FDCAN global filter configuration.
      - HAL_FDCAN_SetExtendedIdMask()                : Configure the extended ID mask.
      - HAL_FDCAN_GetExtendedIdMask()                : Get the extended ID mask configuration.
      - HAL_FDCAN_SetClockDivider()                  : Configure the clock divider.
      - HAL_FDCAN_GetClockDivider()                  : Get the clock divider configuration.
      - HAL_FDCAN_SetRxFifoOverwrite()               : Configure the Rx FIFO operation mode.
      - HAL_FDCAN_GetRxFifoOverwrite()               : Get the Rx FIFO operation mode configuration.
      - HAL_FDCAN_SetRamWatchdog()                   : Configure the RAM watchdog.
      - HAL_FDCAN_GetRamWatchdog()                   : Get the RAM watchdog value.
      - HAL_FDCAN_SetConfigTimestampCounter()        : Configure the timestamp counter.
      - HAL_FDCAN_GetConfigTimestampCounter()        : Get the timestamp counter configuration.
      - HAL_FDCAN_SetConfigTimeoutCounter()          : Configure the timeout counter.
      - HAL_FDCAN_GetConfigTimeoutCounter()          : Get the timeout counter configuration.
      - HAL_FDCAN_EnableTimeoutCounter()             : Enable the timeout counter.
      - HAL_FDCAN_DisableTimeoutCounter()            : Disable the timeout counter.
      - HAL_FDCAN_IsEnabledTimeoutCounter()          : Check if the timeout counter is enabled.
      - HAL_FDCAN_SetConfigTxDelayCompensation()     : Configure the transmitter delay compensation.
      - HAL_FDCAN_GetConfigTxDelayCompensation()     : Get the transmitter delay compensation configuration.
      - HAL_FDCAN_EnableTxDelayCompensation()        : Enable the transmitter delay compensation.
      - HAL_FDCAN_DisableTxDelayCompensation()       : Disable the transmitter delay compensation.
      - HAL_FDCAN_IsEnabledTxDelayCompensation()     : Check if the transmitter delay compensation is enabled.
      - HAL_FDCAN_EnableISOMode()                    : Enable ISO 11898-1 protocol mode.
      - HAL_FDCAN_DisableISOMode()                   : Disable ISO 11898-1 protocol mode.
      - HAL_FDCAN_IsEnabledISOMode()                 : Check if the ISO mode is enabled.
      - HAL_FDCAN_EnableEdgeFiltering()              : Enable edge filtering during bus integration.
      - HAL_FDCAN_DisableEdgeFiltering()             : Disable edge filtering during bus integration.
      - HAL_FDCAN_IsEnabledEdgeFiltering()           : Check if the edge filtering is enabled.
      - HAL_FDCAN_SetMode()                          : Configure operating mode.
      - HAL_FDCAN_GetMode()                          : Get the current operative mode configuration.
      - HAL_FDCAN_EnableRestrictedOperationMode()    : Enable the restricted operation mode.
      - HAL_FDCAN_DisableRestrictedOperationMode()   : Disable the restricted operation mode.
      - HAL_FDCAN_IsEnabledRestrictedOperationMode() : Check if the FDCAN peripheral entered restricted operation mode.
      - HAL_FDCAN_SetFrameFormat()                   : Configure the frame format.
      - HAL_FDCAN_GetFrameFormat()                   : Get the frame format configuration.
      - HAL_FDCAN_EnableAutoRetransmission()         : Enable the auto retransmission.
      - HAL_FDCAN_DisableAutoRetransmission()        : Disable the auto retransmission.
      - HAL_FDCAN_IsEnabledAutoRetransmission()      : Get the automatic retransmission state.
      - HAL_FDCAN_EnableTransmitPause()              : Enable the transmit pause.
      - HAL_FDCAN_DisableTransmitPause()             : Disable the transmit pause.
      - HAL_FDCAN_IsEnabledTransmitPause()           : Get the transmit pause state.
      - HAL_FDCAN_EnableProtocolException()          : Enable the protocol exception.
      - HAL_FDCAN_DisableProtocolException()         : Disable the protocol exception.
      - HAL_FDCAN_IsEnabledProtocolException()       : Get the protocol exception state.
      - HAL_FDCAN_SetTxMode()                        : Configure the transmission FIFO/Queue mode.
      - HAL_FDCAN_GetTxMode()                        : Get the transmission FIFO/Queue mode configuration.
      - HAL_FDCAN_GetClockFreq()                     : Get the current FDCAN kernel clock.

  @note Some configuration functions require the FDCAN peripheral to be in the IDLE state before being called.
        However, several configuration functions are allowed to be called when the peripheral is in ACTIVE or IDLE
        states, as these configurations can be applied dynamically without stopping the peripheral.
  - The following functions are allowed in ACTIVE or IDLE states include:
      - HAL_FDCAN_SetFilter()
      - HAL_FDCAN_ResetTimeoutCounter()
      - HAL_FDCAN_EnableISOMode()
      - HAL_FDCAN_DisableISOMode()
      - HAL_FDCAN_EnableEdgeFiltering()
      - HAL_FDCAN_DisableEdgeFiltering()
      - HAL_FDCAN_DisableRestrictedOperationMode()
      - HAL_FDCAN_EnableAutoRetransmission()
      - HAL_FDCAN_DisableAutoRetransmission()
      - HAL_FDCAN_EnableTransmitPause()
      - HAL_FDCAN_DisableTransmitPause()
      - HAL_FDCAN_EnableProtocolException()
      - HAL_FDCAN_DisableProtocolException()
      - HAL_FDCAN_SetFrameFormat()
 */

/**
  * @brief  Configures the FDCAN according to the user parameters.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_config Pointer to the configuration structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_config is NULL.
  * @retval HAL_ERROR         FDCAN cannot leave the power down mode..
  */
hal_status_t HAL_FDCAN_SetConfig(hal_fdcan_handle_t *hfdcan, const hal_fdcan_config_t *p_config)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_FRAME_FORMAT(p_config->frame_format));
  ASSERT_DBG_PARAM(IS_FDCAN_MODE(p_config->mode));
  ASSERT_DBG_PARAM(IS_FDCAN_AUTO_RETRANSMISSION(p_config->auto_retransmission));
  ASSERT_DBG_PARAM(IS_FDCAN_TRANSMIT_PAUSE(p_config->transmit_pause));
  ASSERT_DBG_PARAM(IS_FDCAN_PROTOCOL_EXCEPTION(p_config->protocol_exception));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_PRESC(p_config->nominal_bit_timing.nominal_prescaler));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_SJW(p_config->nominal_bit_timing.nominal_jump_width));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_TSEG1(p_config->nominal_bit_timing.nominal_time_seg1));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_TSEG2(p_config->nominal_bit_timing.nominal_time_seg2));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_config->nominal_bit_timing.nominal_jump_width,
                                      p_config->nominal_bit_timing.nominal_time_seg2));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_config->std_filters_nbr, FDCAN_RAM_FLS_NBR));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_config->ext_filters_nbr, FDCAN_RAM_FLE_NBR));
  ASSERT_DBG_PARAM(IS_FDCAN_TX_MODE(p_config->tx_fifo_queue_mode));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (FDCAN_ResetClockStopRequest(p_fdcanx) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (FDCAN_InitRequest(p_fdcanx) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Request configuration change */
  STM32_SET_BIT(p_fdcanx->CCCR, FDCAN_CCCR_CCE);

  STM32_MODIFY_REG(p_fdcanx->CCCR,
                   (FDCAN_CCCR_DAR | FDCAN_CCCR_TXP | FDCAN_CCCR_PXHD
                    | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM),
                   ((uint32_t)(p_config->auto_retransmission) | (uint32_t)(p_config->transmit_pause)
                    | (uint32_t)(p_config->protocol_exception) | ((uint32_t)(p_config->frame_format))
                    | (uint32_t)p_config->mode));

  /* Disable test mode if loop-back mode is not selected else enable it */
  if (((uint32_t)p_config->mode & FDCAN_CCCR_TEST) != FDCAN_CCCR_TEST)
  {
    STM32_CLEAR_BIT(p_fdcanx->TEST, FDCAN_TEST_LBCK);
  }
  else
  {
    STM32_SET_BIT(p_fdcanx->TEST, FDCAN_TEST_LBCK);
  }

  FDCAN_SetNominalBitTiming(p_fdcanx, &p_config->nominal_bit_timing);

  if (p_config->frame_format == HAL_FDCAN_FRAME_FORMAT_FD_BRS)
  {
    ASSERT_DBG_PARAM(IS_FDCAN_DATA_PRESC(p_config->data_bit_timing.data_prescaler));
    ASSERT_DBG_PARAM(IS_FDCAN_DATA_SJW(p_config->data_bit_timing.data_jump_width));
    ASSERT_DBG_PARAM(IS_FDCAN_DATA_TSEG1(p_config->data_bit_timing.data_time_seg1));
    ASSERT_DBG_PARAM(IS_FDCAN_DATA_TSEG2(p_config->data_bit_timing.data_time_seg2));
    FDCAN_SetDataBitTiming(p_fdcanx, &p_config->data_bit_timing);
  }

  STM32_MODIFY_REG(p_fdcanx->TXBC, FDCAN_TXBC_TFQM, (uint32_t)p_config->tx_fifo_queue_mode);

  /* Calculate each RAM block address */
  FDCAN_ComputeRAMBlockAddresses(hfdcan, p_config);

  hfdcan->global_state = HAL_FDCAN_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the FDCAN global configuration.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_config Pointer to the configuration structure to be filled with current configuration.
  */
void HAL_FDCAN_GetConfig(const hal_fdcan_handle_t *hfdcan, hal_fdcan_config_t *p_config)
{
  const FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  register_value = STM32_READ_REG(p_fdcanx->CCCR);

  p_config->mode                = FDCAN_GetMode(register_value);
  p_config->auto_retransmission = (hal_fdcan_auto_retransmission_state_t)(uint32_t)
                                  STM32_READ_BIT(register_value, FDCAN_CCCR_DAR);
  p_config->transmit_pause      = (hal_fdcan_transmit_pause_state_t)(uint32_t)
                                  STM32_READ_BIT(register_value, FDCAN_CCCR_TXP);
  p_config->protocol_exception  = (hal_fdcan_protocol_exception_state_t)(uint32_t)
                                  STM32_READ_BIT(register_value, FDCAN_CCCR_PXHD);
  p_config->frame_format        = (hal_fdcan_frame_format_t)(uint32_t)
                                  STM32_READ_BIT(register_value, (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE));

  FDCAN_GetMessageRAMConfig(p_fdcanx, p_config);

  p_config->tx_fifo_queue_mode = (hal_fdcan_tx_mode_t)(uint32_t)(STM32_READ_BIT(p_fdcanx->TXBC, FDCAN_TXBC_TFQM));

  FDCAN_GetNominalBitTiming(p_fdcanx, &p_config->nominal_bit_timing);

  if (p_config->frame_format == HAL_FDCAN_FRAME_FORMAT_FD_BRS)
  {
    FDCAN_GetDataBitTiming(p_fdcanx, &p_config->data_bit_timing);
  }
}

/**
  * @brief  Set the FDCAN nominal bit timing configuration.
  * @param  hfdcan               Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_nominal_bit_timing Pointer to nominal bit timing structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_nominal_bit_timing is NULL.
  */
hal_status_t HAL_FDCAN_SetNominalBitTiming(const hal_fdcan_handle_t *hfdcan,
                                           const hal_fdcan_nominal_bit_timing_t *p_nominal_bit_timing)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_nominal_bit_timing != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_nominal_bit_timing == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_PRESC(p_nominal_bit_timing->nominal_prescaler));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_SJW(p_nominal_bit_timing->nominal_jump_width));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_TSEG1(p_nominal_bit_timing->nominal_time_seg1));
  ASSERT_DBG_PARAM(IS_FDCAN_NOMINAL_TSEG2(p_nominal_bit_timing->nominal_time_seg2));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_nominal_bit_timing->nominal_jump_width,
                                      p_nominal_bit_timing->nominal_time_seg2));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  FDCAN_SetNominalBitTiming(FDCAN_GET_INSTANCE(hfdcan), p_nominal_bit_timing);

  return HAL_OK;
}

/**
  * @brief  Get the FDCAN nominal bit timing configuration.
  * @param  hfdcan               Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_nominal_bit_timing Pointer to the nominal bit timing structure to be filled with current configuration.
  */
void HAL_FDCAN_GetNominalBitTiming(const hal_fdcan_handle_t *hfdcan,
                                   hal_fdcan_nominal_bit_timing_t *p_nominal_bit_timing)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_nominal_bit_timing != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  FDCAN_GetNominalBitTiming(FDCAN_GET_INSTANCE(hfdcan), p_nominal_bit_timing);
}

/**
  * @brief  Set the FDCAN data bit timing configuration.
  * @param  hfdcan             Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_data_bit_timing  Pointer to data bit timing structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_data_bit_timing is NULL.
  */
hal_status_t HAL_FDCAN_SetDataBitTiming(const hal_fdcan_handle_t *hfdcan,
                                        const hal_fdcan_data_bit_timing_t *p_data_bit_timing)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_data_bit_timing != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data_bit_timing == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_DATA_PRESC(p_data_bit_timing->data_prescaler));
  ASSERT_DBG_PARAM(IS_FDCAN_DATA_SJW(p_data_bit_timing->data_jump_width));
  ASSERT_DBG_PARAM(IS_FDCAN_DATA_TSEG1(p_data_bit_timing->data_time_seg1));
  ASSERT_DBG_PARAM(IS_FDCAN_DATA_TSEG2(p_data_bit_timing->data_time_seg2));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  FDCAN_SetDataBitTiming(FDCAN_GET_INSTANCE(hfdcan), p_data_bit_timing);

  return HAL_OK;
}

/**
  * @brief  Get the FDCAN data bit timing configuration.
  * @param  hfdcan            Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_data_bit_timing Pointer to the data bit timing structure to be filled with current configuration.
  */
void HAL_FDCAN_GetDataBitTiming(const hal_fdcan_handle_t *hfdcan, hal_fdcan_data_bit_timing_t *p_data_bit_timing)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_data_bit_timing != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  FDCAN_GetDataBitTiming(FDCAN_GET_INSTANCE(hfdcan), p_data_bit_timing);
}

/**
  * @brief  Configure the FDCAN reception filter according to the user parameters.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_filter_config Pointer to a filter structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_filter_config is NULL.
  */
hal_status_t HAL_FDCAN_SetFilter(const hal_fdcan_handle_t *hfdcan, const hal_fdcan_filter_t *p_filter_config)
{
  uint32_t filter_element_w1;
  uint32_t filter_element_w2;
  uint32_t *filter_address;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_filter_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  hal_fdcan_config_t ram_config;

  FDCAN_GetMessageRAMConfig(FDCAN_GET_INSTANCE(hfdcan), &ram_config);

  if ((p_filter_config == NULL)
      || ((p_filter_config->id_type == HAL_FDCAN_ID_STANDARD) && (p_filter_config->filter_index
                                                                  >= ram_config.std_filters_nbr))
      || ((p_filter_config->id_type == HAL_FDCAN_ID_EXTENDED) && (p_filter_config->filter_index
                                                                  >= ram_config.ext_filters_nbr)))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  ASSERT_DBG_PARAM(IS_FDCAN_ID_TYPE(p_filter_config->id_type));
  ASSERT_DBG_PARAM(IS_FDCAN_FILTER_CFG(p_filter_config->filter_config));

  if (p_filter_config->id_type == HAL_FDCAN_ID_STANDARD)
  {
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_filter_config->filter_id1,
                                        FDCAN_STD_FILTER_ID1_MSK >> FDCAN_STD_FILTER_ID1_POS));
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_filter_config->filter_id2,
                                        FDCAN_STD_FILTER_ID2_MSK >> FDCAN_STD_FILTER_ID2_POS));
    ASSERT_DBG_PARAM(IS_FDCAN_STD_FILTER_TYPE(p_filter_config->filter_type));

    /* Build filter element */
    filter_element_w1 = ((uint32_t)(p_filter_config->filter_type)
                         | ((uint32_t)(p_filter_config->filter_config) << FDCAN_STD_FILTER_CONFIG_POS)
                         | ((p_filter_config->filter_id1) << FDCAN_STD_FILTER_ID1_POS)
                         |  p_filter_config->filter_id2);

    /* Calculate filter address */
    filter_address = (uint32_t *)(hfdcan->msg_ram.std_filter_start_addr +
                                  (p_filter_config->filter_index * FDCAN_RAM_FLS_SIZE));

    /* Write filter element to the message RAM */
    *filter_address = filter_element_w1;
  }
  else /* p_filter_config->id_type == FDCAN_EXTENDED_ID */
  {
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_filter_config->filter_id1,
                                        FDCAN_EXT_FILTER_ID1_MSK >> FDCAN_EXT_FILTER_ID1_POS));
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_filter_config->filter_id2,
                                        FDCAN_EXT_FILTER_ID2_MSK >> FDCAN_EXT_FILTER_ID2_POS));
    ASSERT_DBG_PARAM(IS_FDCAN_EXT_FILTER_TYPE(p_filter_config->filter_type));

    /* Build first word of filter element */
    filter_element_w1 = (((uint32_t)(p_filter_config->filter_config) << FDCAN_EXT_FILTER_CONFIG_POS)
                         | p_filter_config->filter_id1);

    /* Build second word of filter element */
    filter_element_w2 = ((uint32_t)(p_filter_config->filter_type) | p_filter_config->filter_id2);

    /* Calculate filter address */
    filter_address = (uint32_t *)(hfdcan->msg_ram.ext_filter_start_addr +
                                  (p_filter_config->filter_index * FDCAN_RAM_FLE_SIZE));

    /* Write filter element to the message RAM as two 32bit words */
    /* Write the first 32bit word w1 */
    *filter_address = filter_element_w1;
    /* Increment to the next word address */
    filter_address++;
    /* Write the second 32bit word w2 */
    *filter_address = filter_element_w2;
  }

  return HAL_OK;
}

/**
  * @brief  Configure the FDCAN reception filter according to the specified
  *         parameters in the @ref hal_fdcan_filter_t structure.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_filter_config Pointer to a filter structure to be filled with current configuration.
  * @param  filter_index    Index of the filter to be set.
  * @param  id_type         Filter type, for standard or extended ID.
  */
void HAL_FDCAN_GetFilter(const hal_fdcan_handle_t *hfdcan, hal_fdcan_filter_t *p_filter_config,
                         uint32_t filter_index, hal_fdcan_id_type_t id_type)
{
  uint32_t *filter_address;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_filter_config != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_ID_TYPE(id_type));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_filter_config->id_type = id_type;

  if (id_type == HAL_FDCAN_ID_STANDARD)
  {
    /* Filter index must be within the range of configured filters */
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(filter_index + 1U,
                                        (STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->RXGFC, FDCAN_RXGFC_LSS)
                                         >> FDCAN_RXGFC_LSS_Pos)));

    /* Calculate filter address */
    filter_address = (uint32_t *)(hfdcan->msg_ram.std_filter_start_addr + (filter_index * FDCAN_RAM_FLS_SIZE));

    /* Check that the address is not 0x00000000 */
    ASSERT_DBG_PARAM(IS_ADDRESS_VALID(filter_address));

    /* Process and read the S0 word */
    /* Standard filter type SFT */
    p_filter_config->filter_type   = (hal_fdcan_filter_type_t)(uint32_t)
                                     STM32_READ_BIT((uint32_t) * filter_address, FDCAN_STD_FILTER_TYPE_MSK);
    /* Standard filter element configuration SFEC */
    p_filter_config->filter_config = (hal_fdcan_filter_config_t)(uint32_t)
                                     (STM32_READ_BIT((uint32_t) * filter_address, FDCAN_STD_FILTER_CONFIG_MSK)
                                      >> FDCAN_STD_FILTER_CONFIG_POS);
    p_filter_config->filter_index  = filter_index;
    /* Standard filter ID1 SFID1 */
    p_filter_config->filter_id1    = (STM32_READ_BIT((uint32_t) * filter_address, FDCAN_STD_FILTER_ID1_MSK)
                                      >> FDCAN_STD_FILTER_ID1_POS);
    /* Standard filter ID2 SFID2 */
    p_filter_config->filter_id2    = (STM32_READ_BIT((uint32_t) * filter_address, FDCAN_STD_FILTER_ID2_MSK)
                                      >> FDCAN_STD_FILTER_ID2_POS);
  }
  else
  {
    /* Filter index must be within the range of configured filters */
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(filter_index + 1U,
                                        (STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->RXGFC, FDCAN_RXGFC_LSE)
                                         >> FDCAN_RXGFC_LSE_Pos)));

    /* Calculate filter address */
    filter_address = (uint32_t *)(hfdcan->msg_ram.ext_filter_start_addr + (filter_index * FDCAN_RAM_FLE_SIZE));

    /* Check that the address is not 0x00000000 */
    ASSERT_DBG_PARAM(IS_ADDRESS_VALID(filter_address));
    /* Process the F0 word */
    /* Extended filter element configuration EFEC */
    p_filter_config->filter_config = (hal_fdcan_filter_config_t)(uint32_t)
                                     (STM32_READ_BIT(*filter_address, FDCAN_EXT_FILTER_CONFIG_MSK)
                                      >> FDCAN_EXT_FILTER_CONFIG_POS);
    /* Extended filter ID 1 EFID1 */
    p_filter_config->filter_id1    = (STM32_READ_BIT(*filter_address, FDCAN_EXT_FILTER_ID1_MSK)
                                      >> FDCAN_EXT_FILTER_ID1_POS);
    p_filter_config->filter_index  = filter_index;

    /* Read the next word - F1 word */
    filter_address++;

    /* Extended filter type EFT */
    p_filter_config->filter_type   = (hal_fdcan_filter_type_t)(uint32_t)
                                     STM32_READ_BIT(*filter_address, FDCAN_EXT_FILTER_TYPE_MSK);
    /* Extended filter ID 2 EFID2 */
    p_filter_config->filter_id2    = (STM32_READ_BIT(*filter_address, FDCAN_EXT_FILTER_ID2_MSK)
                                      >> FDCAN_EXT_FILTER_ID2_POS);
  }
}

/**
  * @brief  Configure the FDCAN global filter.
  * @param  hfdcan                 Pointer to a @ref hal_fdcan_handle_t handle structure.
  * @param  p_global_filter_config Pointer to a global filter structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_global_filter_config is NULL.
  */
hal_status_t HAL_FDCAN_SetGlobalFilter(const hal_fdcan_handle_t *hfdcan,
                                       const hal_fdcan_global_filter_config_t *p_global_filter_config)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_global_filter_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_global_filter_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_NON_MATCHING(p_global_filter_config->acceptance_non_matching_std));
  ASSERT_DBG_PARAM(IS_FDCAN_NON_MATCHING(p_global_filter_config->acceptance_non_matching_ext));
  ASSERT_DBG_PARAM(IS_FDCAN_REJECT_REMOTE(p_global_filter_config->acceptance_remote_std));
  ASSERT_DBG_PARAM(IS_FDCAN_REJECT_REMOTE(p_global_filter_config->acceptance_remote_ext));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  /* Set global filter */
  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->RXGFC,
                   (FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE | FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE),
                   (((uint32_t)(p_global_filter_config->acceptance_non_matching_std) << FDCAN_RXGFC_ANFS_Pos)
                    | ((uint32_t)(p_global_filter_config->acceptance_non_matching_ext) << FDCAN_RXGFC_ANFE_Pos)
                    | ((uint32_t)(p_global_filter_config->acceptance_remote_std) << FDCAN_RXGFC_RRFS_Pos)
                    | ((uint32_t)(p_global_filter_config->acceptance_remote_ext) << FDCAN_RXGFC_RRFE_Pos)));

  return HAL_OK;
}

/**
  * @brief  Get the FDCAN global filter.
  * @param  hfdcan                 Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_global_filter_config Pointer to a global filter structure.
  */
void HAL_FDCAN_GetGlobalFilter(const hal_fdcan_handle_t *hfdcan,
                               hal_fdcan_global_filter_config_t *p_global_filter_config)
{
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_global_filter_config != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  register_value = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->RXGFC);

  /* Get the accept non-matching frames standard value */
  p_global_filter_config->acceptance_non_matching_std = (hal_fdcan_non_matching_acceptance_rules_t)(uint32_t)
                                                        (STM32_READ_BIT(register_value, FDCAN_RXGFC_ANFS)
                                                         >> FDCAN_RXGFC_ANFS_Pos);
  /* Get the accept non-matching frames extended value */
  p_global_filter_config->acceptance_non_matching_ext = (hal_fdcan_non_matching_acceptance_rules_t)(uint32_t)
                                                        (STM32_READ_BIT(register_value, FDCAN_RXGFC_ANFE)
                                                         >> FDCAN_RXGFC_ANFE_Pos);
  /* Get the reject remote frames standard value */
  p_global_filter_config->acceptance_remote_std       = (hal_fdcan_remote_acceptance_frame_t)(uint32_t)
                                                        (STM32_READ_BIT(register_value, FDCAN_RXGFC_RRFS)
                                                         >> FDCAN_RXGFC_RRFS_Pos);
  /* Get the reject remote frames extended */
  p_global_filter_config->acceptance_remote_ext       = (hal_fdcan_remote_acceptance_frame_t)(uint32_t)
                                                        (STM32_READ_BIT(register_value, FDCAN_RXGFC_RRFE)
                                                         >> FDCAN_RXGFC_RRFE_Pos);
}

/**
  * @brief  Set the extended ID mask value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  mask   Value of the extended ID mask, this parameter must be a number between 0 and 0x1FFFFFFF.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetExtendedIdMask(const hal_fdcan_handle_t *hfdcan, uint32_t mask)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(mask, FDCAN_EXT_FILTER_ID2_MSK >> FDCAN_EXT_FILTER_ID2_POS));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_WRITE_REG(FDCAN_GET_INSTANCE(hfdcan)->XIDAM, mask);

  return HAL_OK;
}

/**
  * @brief  Get the extended ID mask value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint32_t Current extended ID mask (0x00000000..0xFFFFFFFF).
  */
uint32_t HAL_FDCAN_GetExtendedIdMask(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->XIDAM, FDCAN_XIDAM_EIDM);
}

/**
  * @brief  Set the FDCAN clock divider value.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  clock_divider Value of the FDCAN clock divider.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetClockDivider(const hal_fdcan_handle_t *hfdcan, hal_fdcan_clock_divider_t clock_divider)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_CKDIV(clock_divider));

  /* Only the first FDCAN instance is allowed to modify the configuration */
  ASSERT_DBG_PARAM(FDCAN_GET_INSTANCE(hfdcan) == FDCAN1);

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  /* Configure clock divider */
  STM32_WRITE_REG(FDCAN_CONFIG_GET_INSTANCE(hfdcan)->CKDIV, (uint32_t)clock_divider);

  return HAL_OK;
}

/**
  * @brief  Get the clock divider for FDCAN.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current FDCAN kernel clock divider.
  */
hal_fdcan_clock_divider_t HAL_FDCAN_GetClockDivider(const hal_fdcan_handle_t *hfdcan)
{
  /* Variable containing the result of the register FDCAN_CKDIV */
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  register_value = STM32_READ_BIT(FDCAN_CONFIG_GET_INSTANCE(hfdcan)->CKDIV, FDCAN_CKDIV_PDIV);

  return (hal_fdcan_clock_divider_t)register_value;
}

/**
  * @brief  Configure the Rx FIFO operation mode.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  rx_location_idx Rx FIFO selector.
  * @param  operation_mode Operation mode.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetRxFifoOverwrite(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          hal_fdcan_rx_fifo_mode_t operation_mode)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_RX_FIFO(rx_location_idx));
  ASSERT_DBG_PARAM(IS_FDCAN_RX_FIFO_MODE(operation_mode));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (rx_location_idx == HAL_FDCAN_RX_FIFO_0)
  {
    /* Select FIFO 0 operation mode */
    STM32_MODIFY_REG(p_fdcanx->RXGFC, FDCAN_RXGFC_F0OM, ((uint32_t)operation_mode << FDCAN_RXGFC_F0OM_Pos));
  }
  else /* rx_location_idx == FDCAN_RX_FIFO_1 */
  {
    /* Select FIFO 1 operation mode */
    STM32_MODIFY_REG(p_fdcanx->RXGFC, FDCAN_RXGFC_F1OM, ((uint32_t)operation_mode << FDCAN_RXGFC_F1OM_Pos));
  }

  return HAL_OK;
}

/**
  * @brief  Get the Rx FIFO operation mode.
  * @param  hfdcan           Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  rx_location_idx  Rx FIFO selector.
  * @param  p_operation_mode Pointer to the selected Rx FIFO operation mode value.

  */
void HAL_FDCAN_GetRxFifoOverwrite(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                  hal_fdcan_rx_fifo_mode_t *p_operation_mode)
{
  const FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_operation_mode != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_RX_FIFO(rx_location_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (rx_location_idx == HAL_FDCAN_RX_FIFO_0)
  {
    *p_operation_mode = (hal_fdcan_rx_fifo_mode_t)(uint32_t)(STM32_READ_BIT(p_fdcanx->RXGFC, FDCAN_RXGFC_F0OM)
                                                             >> FDCAN_RXGFC_F0OM_Pos);
  }
  else
  {
    *p_operation_mode = (hal_fdcan_rx_fifo_mode_t)(uint32_t)(STM32_READ_BIT(p_fdcanx->RXGFC, FDCAN_RXGFC_F1OM)
                                                             >> FDCAN_RXGFC_F1OM_Pos);
  }
}

/**
  * @brief   Configure the RAM watchdog.
  * @param   hfdcan              Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   counter_start_value Start value of the message RAM watchdog counter,
  *          this parameter must be a number between 0x00 and 0xFF.
  * @note    With the reset value of 0x00 the counter is disabled.
  * @retval  HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetRamWatchdog(const hal_fdcan_handle_t *hfdcan, uint8_t counter_start_value)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);
  /* Configure the RAM watchdog counter start value */
  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->RWD, FDCAN_RWD_WDC, (uint32_t)counter_start_value);

  return HAL_OK;
}

/**
  * @brief  Get the current RAM watchdog counter - not the configured value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint8_t Current message RAM watchdog counter value (0x00..0xFF).
  */
uint8_t HAL_FDCAN_GetRamWatchdog(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (uint8_t)(STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->RWD, FDCAN_RWD_WDV) >> FDCAN_RWD_WDV_Pos);
}

/**
  * @brief  Configure the timestamp counter.
  * @param  hfdcan             Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_timestamp_config Pointer to a timestamp configuration structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_timestamp_config is NULL.
  */
hal_status_t HAL_FDCAN_SetConfigTimestampCounter(const hal_fdcan_handle_t *hfdcan,
                                                 const hal_fdcan_timestamp_config_t *p_timestamp_config)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_timestamp_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_timestamp_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_TIMESTAMP_PRESCALER(p_timestamp_config->timestamp_prescaler));
  ASSERT_DBG_PARAM(IS_FDCAN_TIMESTAMP_SOURCE(p_timestamp_config->timestamp_source));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->TSCC, FDCAN_TSCC_TSS | FDCAN_TSCC_TCP,
                   ((uint32_t)p_timestamp_config->timestamp_source
                    | (uint32_t)p_timestamp_config->timestamp_prescaler));
  return HAL_OK;
}

/**
  * @brief  Get the timestamp counter configuration.
  * @param  hfdcan             Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_timestamp_config Pointer to a timestamp configuration structure.
  */
void HAL_FDCAN_GetConfigTimestampCounter(const hal_fdcan_handle_t *hfdcan,
                                         hal_fdcan_timestamp_config_t *p_timestamp_config)
{
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_timestamp_config != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  register_value = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->TSCC);

  /* Get the timestamp select mode setting */
  p_timestamp_config->timestamp_source    = (hal_fdcan_timestamp_source_t)(uint32_t)
                                            STM32_READ_BIT(register_value, FDCAN_TSCC_TSS);

  /* Get the timestamp counter prescaler */
  p_timestamp_config->timestamp_prescaler = (hal_fdcan_timestamp_prescaler_t)(uint32_t)
                                            STM32_READ_BIT(register_value, FDCAN_TSCC_TCP);
}

/**
  * @brief  Configure the timeout counter.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_timeout_param Pointer to a timeout configuration structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_timeout_param is NULL.
  */
hal_status_t HAL_FDCAN_SetConfigTimeoutCounter(const hal_fdcan_handle_t *hfdcan,
                                               const hal_fdcan_timeout_config_t *p_timeout_param)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_timeout_param != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_timeout_param == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_TIMEOUT(p_timeout_param->timeout_operation));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_timeout_param->timeout_period, (FDCAN_TOCC_TOP >> FDCAN_TOCC_TOP_Pos)));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  /* Modify the timeout select(TOS) and timeout period (TOP) */
  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->TOCC, (FDCAN_TOCC_TOS | FDCAN_TOCC_TOP),
                   ((uint32_t)(p_timeout_param->timeout_operation)
                    | ((uint32_t)(p_timeout_param->timeout_period) << FDCAN_TOCC_TOP_Pos)));

  return HAL_OK;
}

/**
  * @brief  Get the timeout counter configuration.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_timeout_param Pointer to a timeout configuration structure.
  */
void HAL_FDCAN_GetConfigTimeoutCounter(const hal_fdcan_handle_t *hfdcan, hal_fdcan_timeout_config_t *p_timeout_param)
{
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_timeout_param != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  register_value = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->TOCC);

  p_timeout_param->timeout_operation = (hal_fdcan_timeout_operation_t)(uint32_t)
                                       STM32_READ_BIT(register_value, FDCAN_TOCC_TOS);
  p_timeout_param->timeout_period    = (STM32_READ_BIT(register_value, FDCAN_TOCC_TOP) >> FDCAN_TOCC_TOP_Pos);

}

/**
  * @brief  Enable the timeout counter.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableTimeoutCounter(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->TOCC, FDCAN_TOCC_ETOC);

  return HAL_OK;
}

/**
  * @brief  Disable the timeout counter.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableTimeoutCounter(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->TOCC, FDCAN_TOCC_ETOC);

  return HAL_OK;
}

/**
  * @brief  Check timeout counter status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current timeout counter status.
  */
hal_fdcan_timeout_counter_status_t HAL_FDCAN_IsEnabledTimeoutCounter(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_timeout_counter_status_t)
         (uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->TOCC, FDCAN_TOCC_ETOC);
}

/**
  * @brief  Configure the transmitter delay compensation.
  * @param  hfdcan           Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_tx_delay_param Pointer to a Tx delay compensation structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_tx_delay_param is NULL.
  */
hal_status_t HAL_FDCAN_SetConfigTxDelayCompensation(const hal_fdcan_handle_t *hfdcan,
                                                    const hal_fdcan_tx_delay_comp_config_t *p_tx_delay_param)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_tx_delay_param != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_tx_delay_param == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_tx_delay_param->tx_delay_comp_offset,
                                      (FDCAN_TDCR_TDCO_Msk >> FDCAN_TDCR_TDCO_Pos)));
  ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(p_tx_delay_param->tx_delay_comp_win_length,
                                      (FDCAN_TDCR_TDCF_Msk >> FDCAN_TDCR_TDCF_Pos)));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  /* Configure TDC offset and filter window */
  STM32_WRITE_REG(FDCAN_GET_INSTANCE(hfdcan)->TDCR,
                  (((p_tx_delay_param->tx_delay_comp_win_length) << FDCAN_TDCR_TDCF_Pos)
                   | ((p_tx_delay_param->tx_delay_comp_offset) << FDCAN_TDCR_TDCO_Pos)));

  return HAL_OK;
}

/**
  * @brief  Get the transmitter delay compensation offset.
  * @param  hfdcan           Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_tx_delay_param Pointer to a Tx delay compensation structure.
  */
void HAL_FDCAN_GetConfigTxDelayCompensation(const hal_fdcan_handle_t *hfdcan,
                                            hal_fdcan_tx_delay_comp_config_t *p_tx_delay_param)
{
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_tx_delay_param != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  register_value = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->TDCR);

  /* Get TdcOffset transmitter delay compensation offset */
  p_tx_delay_param->tx_delay_comp_offset     = (STM32_READ_BIT(register_value, FDCAN_TDCR_TDCO) >> FDCAN_TDCR_TDCO_Pos);
  /* Get TdcFilter transmitter delay compensation filter window length */
  p_tx_delay_param->tx_delay_comp_win_length = (STM32_READ_BIT(register_value, FDCAN_TDCR_TDCF) >> FDCAN_TDCR_TDCF_Pos);
}

/**
  * @brief  Enable the transmitter delay compensation.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableTxDelayCompensation(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->DBTP, FDCAN_DBTP_TDC);

  return HAL_OK;
}

/**
  * @brief  Disable the transmitter delay compensation.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableTxDelayCompensation(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->DBTP, FDCAN_DBTP_TDC);

  return HAL_OK;
}

/**
  * @brief  Check transmitter delay compensation status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current transmitter delay compensation status.
  */
hal_fdcan_tx_delay_comp_status_t HAL_FDCAN_IsEnabledTxDelayCompensation(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_tx_delay_comp_status_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->DBTP, FDCAN_DBTP_TDC);
}

/**
  * @brief  Enable ISO 11898-1 protocol mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableISOMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_NISO);

  return HAL_OK;
}

/**
  * @brief  Disable ISO 11898-1 protocol mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableISOMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_NISO);

  return HAL_OK;
}

/**
  * @brief  Check ISO 11898-1 protocol mode status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current ISO mode status.
  */
hal_fdcan_iso_mode_status_t HAL_FDCAN_IsEnabledISOMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_iso_mode_status_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_NISO);
}

/**
  * @brief  Enable edge filtering during bus integration.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @note   Two consecutive dominant tq's are required to detect an edge for hard synchronization.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableEdgeFiltering(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_EFBI);

  return HAL_OK;
}

/**
  * @brief  Disable edge filtering during bus integration.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @note   One dominant tq is required to detect an edge for hard synchronization.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableEdgeFiltering(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_EFBI);

  return HAL_OK;
}

/**
  * @brief  Check edge filtering during bus integration status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @note   One dominant tq is required to detect an edge for hard synchronization.
  * @return Current edge filtering status.
  */
hal_fdcan_edge_filtering_status_t HAL_FDCAN_IsEnabledEdgeFiltering(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_edge_filtering_status_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_EFBI);
}

/**
  * @brief  Set the FDCAN mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t
  * @param  mode   Value of the mode to set.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetMode(const hal_fdcan_handle_t *hfdcan, hal_fdcan_mode_t mode)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_MODE(mode));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  STM32_MODIFY_REG(p_fdcanx->CCCR, FDCAN_CCCR_TEST | FDCAN_CCCR_ASM | FDCAN_CCCR_MON, (uint32_t)mode);

  /* Disable test mode if loop-back mode is not selected else enable it */
  if (((uint32_t)mode & FDCAN_CCCR_TEST) != FDCAN_CCCR_TEST)
  {
    STM32_CLEAR_BIT(p_fdcanx->TEST, FDCAN_TEST_LBCK);
  }
  else
  {
    STM32_SET_BIT(p_fdcanx->TEST, FDCAN_TEST_LBCK);
  }

  return HAL_OK;
}

/**
  * @brief  Get the FDCAN mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current FDCAN mode.
  */
hal_fdcan_mode_t HAL_FDCAN_GetMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return FDCAN_GetMode(STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->CCCR));
}

/**
  * @brief  Enable the restricted operation mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR LoopBack mode is already used and cannot be combined with restricted operation mode.
  */
hal_status_t HAL_FDCAN_EnableRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  /* The restricted operation mode must not be combined with the loop-back mode (internal or external) */
  if (STM32_READ_BIT(p_fdcanx->CCCR, FDCAN_CCCR_TEST) != 0U)
  {
    return HAL_ERROR;
  }

  /* When INIT and CCE set to 1 then the bit ASM can be set */
  STM32_SET_BIT(p_fdcanx->CCCR, FDCAN_CCCR_ASM);

  return HAL_OK;
}

/**
  * @brief  Disable the restricted operation mode.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  /* When INIT and CCE set to 1 then the bit ASM can be set */
  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_ASM);

  return HAL_OK;
}

/**
  * @brief  Check the FDCAN restricted operation mode status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current restricted operation mode status.
  */
hal_fdcan_restricted_op_mode_status_t HAL_FDCAN_IsEnabledRestrictedOperationMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_restricted_op_mode_status_t)
         (uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_ASM);
}

/**
  * @brief  Set the frame format.
  * @param  hfdcan       Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  frame_format The frame format to set.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetFrameFormat(const hal_fdcan_handle_t *hfdcan, hal_fdcan_frame_format_t frame_format)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_FRAME_FORMAT(frame_format));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  /* Set FDCAN frame format */
  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->CCCR, (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE), (uint32_t)frame_format);

  return HAL_OK;
}

/**
  * @brief  Get the frame format.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current frame format.
  */
hal_fdcan_frame_format_t HAL_FDCAN_GetFrameFormat(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_frame_format_t)
         (uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE));
}

/**
  * @brief  Enable automatic retransmission of messages.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableAutoRetransmission(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_DAR);

  return HAL_OK;
}

/**
  * @brief  Disable auto retransmission of messages.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableAutoRetransmission(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_DAR);

  return HAL_OK;
}

/**
  * @brief  Get automatic retransmission state.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current auto retransmission setting.
  */
hal_fdcan_auto_retransmission_state_t HAL_FDCAN_IsEnabledAutoRetransmission(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_auto_retransmission_state_t)
         (uint32_t)(STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_DAR));
}

/**
  * @brief  Enable transmit pause.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableTransmitPause(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_TXP);

  return HAL_OK;
}

/**
  * @brief  Disable transmit pause.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableTransmitPause(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_TXP);

  return HAL_OK;
}

/**
  * @brief  Get the transmit pause state.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current transmit pause setting.
  */
hal_fdcan_transmit_pause_state_t HAL_FDCAN_IsEnabledTransmitPause(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_transmit_pause_state_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_TXP);
}

/**
  * @brief  Enable protocol exception.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableProtocolException(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_PXHD);

  return HAL_OK;
}

/**
  * @brief  Disable protocol exception.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableProtocolException(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_PXHD);

  return HAL_OK;
}

/**
  * @brief  Get the protocol exception status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current protocol exception setting.
  */
hal_fdcan_protocol_exception_state_t HAL_FDCAN_IsEnabledProtocolException(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_protocol_exception_state_t)
         (uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_PXHD);
}

/**
  * @brief  Set the transmission FIFO/Queue mode.
  * @param  hfdcan  Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_mode Transmission mode to configure.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetTxMode(const hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_mode_t tx_mode)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_TX_MODE(tx_mode));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE);

  STM32_MODIFY_REG(FDCAN_GET_INSTANCE(hfdcan)->TXBC, FDCAN_TXBC_TFQM, (uint32_t)tx_mode);

  return HAL_OK;
}

/**
  * @brief  Get the transmission FIFO/Queue mode configuration.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current Tx FIFO/queue mode.
  */
hal_fdcan_tx_mode_t HAL_FDCAN_GetTxMode(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_tx_mode_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXBC, FDCAN_TXBC_TFQM);
}


/** @brief  Return the peripheral clock frequency for FDCAN.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint32_t Current clock frequency(Hz) (0x00000000..0xFFFFFFFF). 0U if FDCAN is not configured or not ready.
  */
uint32_t HAL_FDCAN_GetClockFreq(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE
                   | (uint32_t)HAL_FDCAN_STATE_ACTIVE | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_UNUSED(hfdcan);

  return HAL_RCC_FDCAN_GetKernelClkFreq();
}

/**
  * @}
  */

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
/** @addtogroup FDCAN_Exported_Functions_Group4
  * @{
A set of functions allowing to register custom callback handlers for specific FDCAN events:
    - HAL_FDCAN_RegisterTxEventFifoCallback()          : Register callback for Tx Event FIFO interrupts.
    - HAL_FDCAN_RegisterRxFifo0Callback()              : Register callback for Rx FIFO 0 interrupts.
    - HAL_FDCAN_RegisterRxFifo1Callback()              : Register callback for Rx FIFO 1 interrupts.
    - HAL_FDCAN_RegisterTxFifoEmptyCallback()          : Register callback for Tx FIFO empty event.
    - HAL_FDCAN_RegisterTxBufferCompleteCallback()     : Register callback for Tx buffer transmission complete.
    - HAL_FDCAN_RegisterTxBufferAbortCallback()        : Register callback for Tx buffer transmission abort complete.
    - HAL_FDCAN_RegisterHighPriorityMessageCallback()  : Register callback for high priority message reception.
    - HAL_FDCAN_RegisterTimestampWraparoundCallback()  : Register callback for timestamp wraparound event.
    - HAL_FDCAN_RegisterErrorCallback()                : Register callback for error events.

    These registration functions must be called when the FDCAN handle is in the INIT or IDLE state.
    The default callbacks are weak functions that can be overridden or replaced by these registration APIs.
  */

/**
  * @brief  Register Tx event FIFO callback to be used instead of the weak HAL_FDCAN_TxEventFifoCallback()
  *         predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Tx event FIFO callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterTxEventFifoCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_tx_event_fifo_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register Rx FIFO 0 callback to be used instead of the weak HAL_FDCAN_RxFifo0Callback() predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Rx FIFO 0 callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterRxFifo0Callback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_rx_fifo_0_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register Rx FIFO 1 callback to be used instead of the weak HAL_FDCAN_RxFifo1Callback() predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Rx FIFO 1 callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterRxFifo1Callback(hal_fdcan_handle_t *hfdcan, hal_fdcan_fifo_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_rx_fifo_1_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register Tx FIFO empty callback to be used instead of the weak HAL_FDCAN_TxFifoEmptyCallback()
  *         predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Tx FIFO empty callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterTxFifoEmptyCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_tx_fifo_empty_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register Tx buffer complete callback to be used instead of the weak HAL_FDCAN_TxBufferCompleteCallback()
  *         predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Tx buffer complete callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterTxBufferCompleteCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_buffer_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_tx_buffer_complete_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register Tx buffer abort callback to be used instead of the weak HAL_FDCAN_TxBufferAbortCallback()
  *         predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the Tx buffer abort callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterTxBufferAbortCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_buffer_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_tx_buffer_abort_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register high priority message callback to be used instead of the weak
  *         HAL_FDCAN_HighPriorityMessageCallback() predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the high priority message callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterHighPriorityMessageCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_high_priority_msg_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register timestamp wrap around callback to be used instead of the weak
  *         HAL_FDCAN_TimestampWraparoundCallback() predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the timestamp wrap around callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterTimestampWraparoundCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);
  hfdcan->p_ts_wraparound_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register error callback to be used instead of the weak HAL_FDCAN_ErrorCallback() predefined callback.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_callback Pointer to the FDCAN error callback function.
  * @retval HAL_OK            Registering completed successfully.
  * @retval HAL_INVALID_PARAM p_callback is NULL.
  */
hal_status_t HAL_FDCAN_RegisterErrorCallback(hal_fdcan_handle_t *hfdcan, hal_fdcan_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_INIT | (uint32_t)HAL_FDCAN_STATE_IDLE);

  hfdcan->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @}
  */
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */

/** @addtogroup FDCAN_Exported_Functions_Group5
  * @{
A set of functions allowing to control the peripheral and initiate an operation on the bus:
    - HAL_FDCAN_Start()                          : Start the FDCAN module.
    - HAL_FDCAN_Stop()                           : Stop the FDCAN module.
    - HAL_FDCAN_ReqTransmitMsgFromFIFOQ()        : Add a message to the Tx FIFO/Queue and activate
                                                   the corresponding transmission request.
    - HAL_FDCAN_GetLatestTxFifoQRequestBuffer()  : Get the Tx buffer index of latest Tx FIFO/Queue request.
    - HAL_FDCAN_GetTxFifoFreeLevel()             : Get the Tx FIFO free level.
    - HAL_FDCAN_ReqAbortOfTxBuffer()             : Abort transmission request.
    - HAL_FDCAN_GetTxEvent()                     : Get a FDCAN Tx event from the Tx event FIFO zone
                                                   into the message RAM.
    - HAL_FDCAN_GetTxEventFifoFillLevel()        : Get the Tx event FIFO fill level.
    - HAL_FDCAN_GetTxBufferMessageStatus()       : Check if a transmission request is pending on any
                                                   of the selected Tx buffers.
    - HAL_FDCAN_GetReceivedMessage()             : Get a FDCAN frame from the Rx FIFO zone into the message RAM.
    - HAL_FDCAN_GetRxFifoFillLevel()             : Get the Rx FIFO fill level.
    - HAL_FDCAN_GetTimestampCounter()            : Get the timestamp counter value.
    - HAL_FDCAN_ResetTimestampCounter()          : Reset the timestamp counter to zero.
    - HAL_FDCAN_GetTimeoutCounter()              : Get the timeout counter value.
    - HAL_FDCAN_ResetTimeoutCounter()            : Reset the timeout counter to its starting value.
    - HAL_FDCAN_GetHighPriorityMessageStatus()   : Get the high priority message status.
    - HAL_FDCAN_GetProtocolStatus()              : Get the protocol status.
    - HAL_FDCAN_GetErrorCounters()               : Get the error counter values.
    - HAL_FDCAN_Recover()                        : Recover the bus-off error.
  */

/**
  * @brief  Start the FDCAN module.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM hfdcan is NULL.
  * @retval HAL_BUSY          There is ongoing process.
  */
hal_status_t HAL_FDCAN_Start(hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hfdcan == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, HAL_FDCAN_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hfdcan, global_state, HAL_FDCAN_STATE_IDLE, HAL_FDCAN_STATE_ACTIVE);

  /* Request leave initialisation */
  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->CCCR, FDCAN_CCCR_INIT);

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
  hfdcan->last_error_codes = HAL_FDCAN_ERROR_NONE;
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

  return HAL_OK;
}

/**
  * @brief  Stop the FDCAN module and enable access to configuration registers.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM hfdcan is NULL.
  * @retval HAL_ERROR         FDCAN cannot leave the power down mode.
  */
hal_status_t HAL_FDCAN_Stop(hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hfdcan == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_ACTIVE | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (FDCAN_ResetClockStopRequest(p_fdcanx) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* To stop the FDCAN, initialization mode is entered */
  if (FDCAN_InitRequest(p_fdcanx) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Enable configuration change */
  STM32_SET_BIT(p_fdcanx->CCCR, FDCAN_CCCR_CCE);

  /* Reset latest Tx FIFO/Queue request buffer index */
  hfdcan->latest_tx_fifo_q_request = 0U;

  hfdcan->global_state = HAL_FDCAN_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Add a message to the Tx FIFO/Queue and activate the corresponding transmission request.
  * @param  hfdcan              Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_tx_element_header Pointer to a Tx element header structure.
  * @param  p_tx_data           Pointer to a buffer containing the payload of the Tx frame.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_tx_element_header or p_tx_data are NULL.
  * @retval HAL_ERROR         The Tx FIFO/Queue is full
  */
hal_status_t HAL_FDCAN_ReqTransmitMsgFromFIFOQ(hal_fdcan_handle_t *hfdcan,
                                               const hal_fdcan_tx_header_t *p_tx_element_header,
                                               const uint8_t *p_tx_data)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t put_index;
  hal_fdcan_tx_header_t tx_element_header;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_tx_data != NULL));
  ASSERT_DBG_PARAM((p_tx_element_header != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_tx_element_header == NULL) || (p_tx_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  tx_element_header = *p_tx_element_header;

  if (tx_element_header.b.identifier_type == HAL_FDCAN_ID_STANDARD)
  {
    ASSERT_DBG_PARAM(IS_FDCAN_MAX_VALUE(tx_element_header.b.identifier,
                                        FDCAN_STD_FILTER_ID2_MSK >> FDCAN_STD_FILTER_ID2_POS));

    /* A standard identifier has to be written to ID[28:18] */
    tx_element_header.b.identifier <<= FDCAN_STD_FILTER_ID_POS;
  }

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  /* Check that the Tx FIFO/Queue is not full */
  if (STM32_READ_BIT(p_fdcanx->TXFQS, FDCAN_TXFQS_TFQF) == 0U)
  {
    /* Get the Tx FIFO put_index */
    put_index = STM32_READ_BIT(p_fdcanx->TXFQS, FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    /* Add the message to the Tx FIFO/Queue */
    FDCAN_CopyMessageToRAM(hfdcan, &tx_element_header, p_tx_data, put_index);

    /* Activate the corresponding transmission request */
    STM32_WRITE_REG(p_fdcanx->TXBAR, 1UL << put_index);

    /* Store the latest Tx FIFO/Queue request buffer index */
    hfdcan->latest_tx_fifo_q_request = (1UL << put_index);

    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Get the Tx FIFO/Queue status.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current FIFO/Queue status.
  */
hal_fdcan_fifo_status_t HAL_FDCAN_GetTxFifoStatus(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_fifo_status_t)(uint32_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXFQS, FDCAN_TXFQS_TFQF);
}

/**
  * @brief  Get the Tx buffer index of latest Tx FIFO/Queue request.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint32_t Current index of last Tx FIFO/Queue request (0x00000000..0xFFFFFFFF).
            0U if No Tx submitted request.
  */
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  /* Return last Tx FIFO/Queue request buffer */
  return (hfdcan->latest_tx_fifo_q_request);
}

/**
  * @brief   Return the Tx FIFO free level - number of consecutive free Tx FIFO elements starting
  *          from Tx FIFO get_index.
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @warning This function is relevant only when the Tx FIFO operates in FIFO mode.
  *          In Queue mode, the Tx FIFO free level is always @ref HAL_FDCAN_TX_FIFO_FREE_LEVEL_0.
  * @return  Current Tx FIFO free level.
  */
hal_fdcan_tx_fifo_free_level_t HAL_FDCAN_GetTxFifoFreeLevel(const hal_fdcan_handle_t *hfdcan)
{
  const FDCAN_GlobalTypeDef *p_fdcanx;
  hal_fdcan_tx_fifo_free_level_t fifo_free_level = HAL_FDCAN_TX_FIFO_FREE_LEVEL_0;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (STM32_READ_BIT(p_fdcanx->TXBC, FDCAN_TXBC_TFQM) != (uint32_t)HAL_FDCAN_TX_MODE_QUEUE)
  {
    fifo_free_level = (hal_fdcan_tx_fifo_free_level_t)(uint32_t)STM32_READ_BIT(p_fdcanx->TXFQS, FDCAN_TXFQS_TFFL);
  }

  return fifo_free_level;
}

/**
  * @brief  Abort transmission request.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffer_idx Transmission buffer index.
  *         This parameter can be any combination of @ref FDCAN_IT_Tx_Abort_Buffers_Select.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_ReqAbortOfTxBuffer(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffer_idx)
{
  FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  ASSERT_DBG_PARAM(IS_FDCAN_TX_LOCATION_LIST(tx_buffer_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_ACTIVE | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_WRITE_REG(p_fdcanx->TXBCR, tx_buffer_idx);

  return HAL_OK;
}

/**
  * @brief  Get a FDCAN Tx event from the Tx event FIFO zone into the message RAM.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_tx_event Pointer to a hal_fdcan_tx_evt_fifo_header_t structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_tx_event is NULL.
  * @retval HAL_ERROR         FIFO is empty or no RAM was allocated for it.
  */
hal_status_t HAL_FDCAN_GetTxEvent(const hal_fdcan_handle_t *hfdcan, hal_fdcan_tx_evt_fifo_header_t *p_tx_event)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  const uint32_t *tx_event_address;
  uint32_t get_index;
  uint32_t register_txefs_value;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_tx_event != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_tx_event == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  register_txefs_value = STM32_READ_BIT(p_fdcanx->TXEFS, FDCAN_TXEFS_EFFL);
  /* Check that the Tx event FIFO is not empty */
  if (register_txefs_value != 0U)
  {
    /* Calculate Tx event FIFO element address */
    get_index = (STM32_READ_BIT(p_fdcanx->TXEFS, FDCAN_TXEFS_EFGI) >> FDCAN_TXEFS_EFGI_Pos);
    tx_event_address = (uint32_t *)(hfdcan->msg_ram.tx_event_start_addr + (get_index * FDCAN_RAM_TEF_SIZE));

    /* Build the 64-bit Tx event header */
    p_tx_event->d64 = ((uint64_t)tx_event_address[1U] << 32U) | (uint64_t)tx_event_address[0U];

    /* A standard identifier has to be written to ID[28:18] */
    if (p_tx_event->b.identifier_type == HAL_FDCAN_ID_STANDARD)
    {
      /* Shift ID */
      p_tx_event->b.identifier >>= FDCAN_STD_FILTER_ID_POS;
    }

    /* Acknowledge the Tx event FIFO that the oldest element is read so that it increments the get_index */
    STM32_WRITE_REG(p_fdcanx->TXEFA, get_index);

    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Return the number of unread Tx event FIFO elements.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_fill_level Pointer to variable to receive the current fill level.
  */
void HAL_FDCAN_GetTxEventFifoFillLevel(const hal_fdcan_handle_t *hfdcan, uint32_t *p_fill_level)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_fill_level != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  *p_fill_level = STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXEFS, FDCAN_TXEFS_EFFL) >> FDCAN_TXEFS_EFFL_Pos;
}

/**
  * @brief  Check if a transmission request is pending on any of the selected Tx buffers.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffer_idx Transmission buffer index.
  *         This parameter can be any combination of @ref FDCAN_Tx_Buffer_Location.
  * @return Current Tx buffer pending status.
  */
hal_fdcan_tx_buffer_status_t HAL_FDCAN_GetTxBufferMessageStatus(const hal_fdcan_handle_t *hfdcan,
                                                                uint32_t tx_buffer_idx)
{
  const FDCAN_GlobalTypeDef *p_fdcanx;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  ASSERT_DBG_PARAM(IS_FDCAN_TX_LOCATION_LIST(tx_buffer_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return STM32_IS_BIT_SET(p_fdcanx->TXBRP, tx_buffer_idx) ? HAL_FDCAN_TX_BUFFER_PENDING
         : HAL_FDCAN_TX_BUFFER_NOT_PENDING;
}

/**
  * @brief  Get a FDCAN frame from the Rx FIFO zone into the message RAM.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  rx_location_idx Location of received message to read.
  * @param  p_rx_header     Pointer to an Rx element header structure to be filled.
  * @param  p_rx_data       Pointer to a buffer where the payload of the Rx message has to be stored.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_rx_header or p_rx_data are NULL.
  * @retval HAL_ERROR         Rx FIFO empty.
  */
hal_status_t HAL_FDCAN_GetReceivedMessage(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                          hal_fdcan_rx_header_t *p_rx_header, uint8_t *p_rx_data)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t *rx_address;
  uint32_t byte_count;
  uint32_t get_index = 0U;
  uint32_t most_significant_word;
  uint32_t least_significant_word;
  uint8_t  *p_data;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_rx_header != NULL));
  ASSERT_DBG_PARAM((p_rx_data != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_RX_FIFO(rx_location_idx));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_rx_header == NULL) || (p_rx_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  /* Rx element is assigned to the Rx FIFO 0 while it is not empty */
  if ((STM32_READ_BIT(p_fdcanx->RXF0S, FDCAN_RXF0S_F0FL) != 0U) && (rx_location_idx == HAL_FDCAN_RX_FIFO_0))
  {
    /* Calculate Rx FIFO 0 element index */
    get_index = STM32_READ_BIT(p_fdcanx->RXF0S, FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;

    /* Check if the Rx FIFO 0 is full & overwrite mode is on */
    if (STM32_READ_BIT(p_fdcanx->RXF0S, FDCAN_RXF0S_F0F) == FDCAN_RXF0S_F0F)
    {
      if (STM32_READ_BIT(p_fdcanx->RXGFC, FDCAN_RXGFC_F0OM) == FDCAN_RXGFC_F0OM)
      {
        /* When overwrite status is on discard first message in FIFO */
        /* get_index is incremented by one and wraps to 0 in case it overflows the FIFO size */
        get_index = (get_index + 1U) % FDCAN_RAM_RF0_NBR;
      }
    }
    /* Calculate Rx FIFO 0 element address */
    rx_address = (uint32_t *)(hfdcan->msg_ram.rx_fifo0_start_addr + (get_index * FDCAN_RAM_RF0_SIZE));
  }
  /* Rx element is assigned to the Rx FIFO 1 while it is not empty */
  else if (STM32_READ_BIT(p_fdcanx->RXF1S, FDCAN_RXF1S_F1FL) != 0U)
  {
    /* Calculate Rx FIFO 1 element index */
    get_index = (STM32_READ_BIT(p_fdcanx->RXF1S, FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos);

    /* Check if the Rx FIFO 1 is full & overwrite mode is on */
    if (STM32_READ_BIT(p_fdcanx->RXF1S, FDCAN_RXF1S_F1F) == FDCAN_RXF1S_F1F)
    {
      if (STM32_READ_BIT(p_fdcanx->RXGFC, FDCAN_RXGFC_F1OM) == FDCAN_RXGFC_F1OM)
      {
        /* When overwrite status is on discard first message in FIFO */
        /* get_index is incremented by one and wraps to 0 in case it overflows the FIFO size */
        get_index = (get_index + 1U) % FDCAN_RAM_RF1_NBR;
      }
    }
    /* Calculate Rx FIFO 1 element address */
    rx_address = (uint32_t *)(hfdcan->msg_ram.rx_fifo1_start_addr + (get_index * FDCAN_RAM_RF1_SIZE));
  }
  else
  {
    return HAL_ERROR;
  }

  /* Read the first word of the Rx FIFO element - R0 */
  least_significant_word = (uint32_t)(*rx_address);

  /* Increment rx_address pointer to payload of Rx FIFO element - R2....Rn */
  rx_address++;

  /* Read the second word of the Rx FIFO element - R1 */
  most_significant_word = (uint32_t)(*rx_address);

  /* Build the 64-bit Tx event header */
  p_rx_header->d64 = ((uint64_t)most_significant_word << 32U) | (uint64_t)least_significant_word;

  /* A standard identifier has to be written to ID[28:18] */
  if (p_rx_header->b.identifier_type == HAL_FDCAN_ID_STANDARD)
  {
    p_rx_header->b.identifier >>= FDCAN_STD_FILTER_ID_POS;
  }

  /* Increment rx_address pointer to payload of Rx FIFO element - R2....Rn */
  rx_address++;

  /* Get Rx payload */
  p_data = (uint8_t *)rx_address;

  for (byte_count = 0; byte_count < FDCAN_GET_DATA_LENGTH_BYTES(p_rx_header->b.data_length); byte_count++)
  {
    p_rx_data[byte_count] = p_data[byte_count];
  }

  /* Rx element is assigned to the Rx FIFO 0 */
  if (rx_location_idx == HAL_FDCAN_RX_FIFO_0)
  {
    /* Acknowledge the Rx FIFO 0 that the oldest element is read so that it increments the get_index */
    STM32_WRITE_REG(p_fdcanx->RXF0A, get_index);
  }
  else /* Rx element is assigned to the Rx FIFO 1 */
  {
    /* Acknowledge the Rx FIFO 1 that the oldest element is read so that it increments the get_index */
    STM32_WRITE_REG(p_fdcanx->RXF1A, get_index);
  }

  return HAL_OK;
}

/**
  * @brief  Get the fill level of the specified Rx FIFO.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  rx_location_idx Rx FIFO selector.
  * @param  p_fill_level Pointer to variable to receive the current fill level.
  */
void HAL_FDCAN_GetRxFifoFillLevel(const hal_fdcan_handle_t *hfdcan, hal_fdcan_rx_location_t rx_location_idx,
                                  uint32_t *p_fill_level)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_fill_level != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_RX_FIFO(rx_location_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  if (rx_location_idx == HAL_FDCAN_RX_FIFO_0)
  {
    *p_fill_level = STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->RXF0S, FDCAN_RXF0S_F0FL);
  }
  else /* rx_location_idx == FDCAN_RX_FIFO_1 */
  {
    *p_fill_level = STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->RXF1S, FDCAN_RXF1S_F1FL);
  }
}


/**
  * @brief  Get the timestamp counter value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint16_t Current timestamp counter value (0x0000..0xFFFF).
  */
uint16_t HAL_FDCAN_GetTimestampCounter(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return ((uint16_t)(STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->TSCV)));
}

/**
  * @brief  Reset the timestamp counter to zero.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR Timestamp counter source is not internal.
  */
hal_status_t HAL_FDCAN_ResetTimestampCounter(const hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (STM32_READ_BIT(p_fdcanx->TSCC, FDCAN_TSCC_TSS) == (uint32_t)HAL_FDCAN_TIMESTAMP_SOURCE_INTERNAL)
  {
    STM32_CLEAR_REG(p_fdcanx->TSCV);

    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Get the timeout counter value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return uint16_t Current timeout counter value (0x0000..0xFFFF).
  */
uint16_t HAL_FDCAN_GetTimeoutCounter(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (uint16_t)STM32_READ_BIT(FDCAN_GET_INSTANCE(hfdcan)->TOCV, FDCAN_TOCV_TOC);
}

/**
  * @brief  Reset the timeout counter to its start value.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR Timeout counter source is not continuous.
  */
hal_status_t HAL_FDCAN_ResetTimeoutCounter(const hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  if (STM32_READ_BIT(p_fdcanx->TOCC, FDCAN_TOCC_TOS) == (uint32_t)HAL_FDCAN_TIMEOUT_CONTINUOUS)
  {
    STM32_CLEAR_REG(p_fdcanx->TOCV);

    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Get the high priority message status.
  * @param  hfdcan          Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_hp_msg_status Pointer to a high priority message status structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_hp_msg_status is NULL.
  */
hal_status_t HAL_FDCAN_GetHighPriorityMessageStatus(const hal_fdcan_handle_t *hfdcan,
                                                    hal_fdcan_high_prio_msg_status_t *p_hp_msg_status)
{
  uint32_t register_value;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_hp_msg_status != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_hp_msg_status == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  register_value = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->HPMS);

  p_hp_msg_status->filter_list             = (hal_fdcan_high_prio_filter_list_t)(uint32_t)
                                             STM32_READ_BIT(register_value, FDCAN_HPMS_FLST);
  p_hp_msg_status->filter_index            = (STM32_READ_BIT(register_value, FDCAN_HPMS_FIDX) >> FDCAN_HPMS_FIDX_Pos);
  p_hp_msg_status->message_location_status = (hal_fdcan_high_prio_message_storage_t)(uint32_t)
                                             STM32_READ_BIT(register_value, FDCAN_HPMS_MSI);
  p_hp_msg_status->message_index           = STM32_READ_BIT(register_value, FDCAN_HPMS_BIDX);

  return HAL_OK;
}

/**
  * @brief  Get the protocol status.
  * @param  hfdcan            Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_protocol_status Pointer to a protocol status structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_protocol_status is NULL.
  */
hal_status_t HAL_FDCAN_GetProtocolStatus(const hal_fdcan_handle_t *hfdcan,
                                         hal_fdcan_protocol_status_t *p_protocol_status)
{
  uint32_t reg_status;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_protocol_status != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_protocol_status == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  /* Read the protocol status register */
  reg_status = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->PSR);

  /* Fill the protocol status structure */
  p_protocol_status->last_error_code      = (hal_fdcan_protocol_error_code_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_LEC);
  p_protocol_status->data_last_error_code = (hal_fdcan_protocol_error_code_t)(uint32_t)
                                            (STM32_READ_BIT(reg_status, FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos);
  p_protocol_status->activity             = (hal_fdcan_communication_state_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_ACT);
  p_protocol_status->error_status         = (hal_fdcan_protocol_error_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_EP);
  p_protocol_status->error_warning        = (hal_fdcan_warning_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_EW);
  p_protocol_status->bus_off              = (hal_fdcan_bus_off_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_BO);
  p_protocol_status->rx_esi_flag          = (hal_fdcan_esi_flag_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status,  FDCAN_PSR_RESI);
  p_protocol_status->rx_brs_flag          = (hal_fdcan_brs_flag_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_RBRS);
  p_protocol_status->rx_fdf_flag          = (hal_fdcan_edl_flag_status_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_REDL);
  p_protocol_status->protocol_exception   = (hal_fdcan_protocol_exception_event_t)(uint32_t)
                                            STM32_READ_BIT(reg_status, FDCAN_PSR_PXE);
  p_protocol_status->tdc_value            = STM32_READ_BIT(reg_status, FDCAN_PSR_TDCV) >> FDCAN_PSR_TDCV_Pos;

  return HAL_OK;
}

/**
  * @brief  Get the error counter values.
  * @param  hfdcan           Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_error_counters Pointer to an error counters structure.
  * @retval HAL_OK            Operation completed successfully.
  * @retval HAL_INVALID_PARAM p_error_counters is NULL.
  */
hal_status_t HAL_FDCAN_GetErrorCounters(const hal_fdcan_handle_t *hfdcan,
                                        hal_fdcan_error_counters_t *p_error_counters)
{
  uint32_t error_counter_reg;

  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM((p_error_counters != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_error_counters == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  /* Read the error counters register */
  error_counter_reg = STM32_READ_REG(FDCAN_GET_INSTANCE(hfdcan)->ECR);

  /* Fill the error counters structure */
  p_error_counters->tx_error_cnt            = STM32_READ_BIT(error_counter_reg, FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
  p_error_counters->rx_error_cnt            = STM32_READ_BIT(error_counter_reg, FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
  p_error_counters->rx_error_passive_status = (hal_fdcan_rx_error_passive_level_t)(uint32_t)
                                              STM32_READ_BIT(error_counter_reg, FDCAN_ECR_RP);
  p_error_counters->global_cnt              = STM32_READ_BIT(error_counter_reg, FDCAN_ECR_CEL) >> FDCAN_ECR_CEL_Pos;

  return HAL_OK;
}

/**
  * @brief  Recover the FDCAN bus-off error.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_Recover(const hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_ACTIVE);

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  /* If the controller is in bus-off state (FDCAN_PSR_BO set), clear the INIT bit in CCCR to start bus-off recovery.
     This allows the FDCAN to synchronize to the CAN bus and resume normal operation after the bus-off recovery
     sequence. */
  if (STM32_READ_BIT(p_fdcanx->PSR, FDCAN_PSR_BO) != (uint32_t)HAL_FDCAN_BUS_OFF_FLAG_RESET)
  {
    STM32_CLEAR_BIT(p_fdcanx->CCCR, FDCAN_CCCR_INIT);
  }

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup FDCAN_Exported_Functions_Group6
  * @{
A set of functions allowing to deal with interruptions of the peripheral:
    - HAL_FDCAN_IRQHandler()                            : FDCAN interrupt request handler.
    - HAL_FDCAN_Line0_IRQHandler()                      : FDCAN interrupt request handler for line 0 only.
    - HAL_FDCAN_Line1_IRQHandler()                      : FDCAN interrupt request handler for line 1 only.
    - HAL_FDCAN_EnableInterrupts()                      : Enable interrupt sources.
    - HAL_FDCAN_DisableInterrupts()                     : Disable interrupt sources.
    - HAL_FDCAN_IsEnabledInterrupt()                    : Check if a given interrupt source is enabled.
    - HAL_FDCAN_EnableTxBufferCompleteInterrupts()      : Enable interrupt for Tx buffer complete.
    - HAL_FDCAN_DisableTxBufferCompleteInterrupts()     : Disable interrupt for Tx buffer complete.
    - HAL_FDCAN_IsEnabledTxBufferCompleteInterrupt()    : Check if interrupt for Tx buffer complete is enabled.
    - HAL_FDCAN_EnableTxBufferCancellationInterrupts()  : Enable interrupt for Tx buffer cancellation finished.
    - HAL_FDCAN_DisableTxBufferCancellationInterrupts() : Disable interrupt for Tx buffer cancellation finished.
    - HAL_FDCAN_IsEnabledTxBufferCancellationInterrupt(): Check if interrupt for Tx buffer cancellation finished.
                                                          is enabled.
    - HAL_FDCAN_SetInterruptGroupsToLine()              : Assign interrupt groups to either interrupt line 0 or 1.
    - HAL_FDCAN_GetLineFromInterruptGroup()             : Get the line associated to an interrupt group.
    - HAL_FDCAN_EnableInterruptLines()                  : Enable the given interrupt line.
    - HAL_FDCAN_DisableInterruptLines()                 : Disable the given interrupt line.
    - HAL_FDCAN_IsEnabledInterruptLine()                : Check if a given interrupt line is enabled.

  The FDCAN interrupt handler processes events in the following order:
    - Error events (bus-off, error passive, warning, protocol errors, RAM watchdog, etc).
    - High priority message event.
    - Rx FIFO 0 events (new message, full, message lost).
    - Rx FIFO 1 events (new message, full, message lost).
    - Tx Event FIFO events (new entry, full, element lost).
    - Tx FIFO empty event.
    - Transmission complete and transmission cancellation events.
    - Timestamp wraparound event.

  The following callbacks are triggered by the FDCAN interrupt handler when the corresponding event occurs and the
  relevant interrupts are enabled.

  @note For all callbacks below, the following HAL APIs must be called as part of the initialization sequence:
    - HAL_FDCAN_EnableInterrupts()         : Enable the relevant interrupt sources (e.g., Tx, Rx, error, etc.).
    - HAL_FDCAN_SetInterruptGroupsToLine() : Map the interrupts group to an interrupt line (if required).
    - HAL_FDCAN_EnableInterruptLines()     : Enable the interrupt line(s).
    - HAL_FDCAN_Start()                    : Start the FDCAN peripheral.

The table below lists the additional, callback-specific API(s) required to trigger each callback:

   | Callback                                | Additional Required API(s) (in order)                  |
   |-----------------------------------------|--------------------------------------------------------|
   | HAL_FDCAN_TxBufferCompleteCallback()    | HAL_FDCAN_EnableTxBufferCompleteInterrupts(),          |
   |                                         | HAL_FDCAN_ReqTransmitMsgFromFIFOQ()                    |
   | HAL_FDCAN_TxBufferAbortCallback()       | HAL_FDCAN_EnableTxBufferCancellationInterrupts(),      |
   |                                         | HAL_FDCAN_ReqAbortOfTxBuffer()                         |
   | HAL_FDCAN_TxEventFifoCallback()         | HAL_FDCAN_ReqTransmitMsgFromFIFOQ()                    |
   | HAL_FDCAN_TxFifoEmptyCallback()         | HAL_FDCAN_ReqTransmitMsgFromFIFOQ()                    |
   | HAL_FDCAN_RxFifo0Callback()             | (No additional API; triggered by incoming CAN frames)  |
   | HAL_FDCAN_RxFifo1Callback()             | (No additional API; triggered by incoming CAN frames)  |
   | HAL_FDCAN_HighPriorityMessageCallback() | (No additional API; triggered by incoming CAN frames)  |
   | HAL_FDCAN_TimestampWraparoundCallback() | (No additional API; triggered by timestamp wraparound) |
   | HAL_FDCAN_ErrorCallback()               | (No additional API; triggered by any error event)      |
  @note  HAL_FDCAN_ErrorCallback is called for any error event, including bus errors, protocol errors,
         access errors, or timeouts.

  The actual callback triggered depends on the enabled interrupts and the process API in use.

  @note  Recommended interrupt group dispatching when using
         HAL_FDCAN_SetInterruptGroupsToLine() with dual-line NVIC integration:

         | Interrupt Group                   | Representative Flags  | Recommended Line | Reason                    |
         |-----------------------------------|-----------------------|------------------|---------------------------|
         | HAL_FDCAN_IT_GROUP_RX_FIFO_0      | RF0N/RF0F/RF0L        | Line 1           | Latency critical Rx path  |
         | HAL_FDCAN_IT_GROUP_RX_FIFO_1      | RF1N/RF1F/RF1L        | Line 1           | Latency critical Rx path  |
         | HAL_FDCAN_IT_GROUP_STATUS_MSG     | HPM/TC/TCF            | Line 1           | Prioritize HPM latency    |
         | HAL_FDCAN_IT_GROUP_MISC           | TSW/MRAF/TOO          | Line 1           | Prioritize MRAF handling  |
         | HAL_FDCAN_IT_GROUP_TX_FIFO_ERROR  | TFE/TEFN/TEFF/TEFL    | Line 0           | Tx service and logging    |
         | HAL_FDCAN_IT_GROUP_BIT_LINE_ERROR | ELO/EP                | Line 0           | Diagnostic                |
         | HAL_FDCAN_IT_GROUP_PROTOCOL_ERROR | EW/BO/WDI/PEA/PED/ARA | Line 0           | Diagnostic and recovery   |

  @note  NVIC does not give interrupt line 1 a higher priority than line 0 by itself.
    The effective priority remains application-defined through the NVIC configuration of
    the IRQs serving FDCANx_IT0 and FDCANx_IT1.
    When following the recommendations above, assign FDCANx_IT1 a higher NVIC priority
    than FDCANx_IT0, that is, use a lower priority value for FDCANx_IT1.

  @note  For dual-line NVIC integration, prefer HAL_FDCAN_Line0_IRQHandler() and HAL_FDCAN_Line1_IRQHandler() to get
         line-focused interrupt dispatching, clearer event attribution, and improved ISR readability.
  */

/**
  * @brief   Processes all enabled FDCAN interrupt requests (line-agnostic behavior).
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @warning Use HAL_FDCAN_Line0_IRQHandler() / HAL_FDCAN_Line1_IRQHandler() instead of this function when
  *          line-specific interrupt dispatching is required.
  */
void HAL_FDCAN_IRQHandler(hal_fdcan_handle_t *hfdcan)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t it_flags;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  /* Retrieve all the current flags, independently of the configured line */
  it_flags = STM32_READ_REG(p_fdcanx->IR);
  it_flags &= STM32_READ_REG(p_fdcanx->IE);

  FDCAN_HandleInterruptFlags(hfdcan, it_flags);
}

/**
  * @brief  Processes only line 0 mapped and enabled FDCAN interrupt requests.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  */
void HAL_FDCAN_Line0_IRQHandler(hal_fdcan_handle_t *hfdcan)
{
  uint32_t it_flags;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  it_flags = FDCAN_GetLineSpecificInterruptFlags(FDCAN_GET_INSTANCE(hfdcan), HAL_FDCAN_IT_LINE_0);

  FDCAN_HandleInterruptFlags(hfdcan, it_flags);
}

/**
  * @brief  Processes only line 1 mapped and enabled FDCAN interrupt requests.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  */
void HAL_FDCAN_Line1_IRQHandler(hal_fdcan_handle_t *hfdcan)
{
  uint32_t it_flags;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  it_flags = FDCAN_GetLineSpecificInterruptFlags(FDCAN_GET_INSTANCE(hfdcan), HAL_FDCAN_IT_LINE_1);

  FDCAN_HandleInterruptFlags(hfdcan, it_flags);
}

/**
  * @brief  Enable the interrupts.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  interrupts Interrupt signal(s) to enable, can be OR-ed with different interrupt signals.
  *         This parameter can be any combination of @ref FDCAN_Interrupt_Sources.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t interrupts)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_IT(interrupts));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->IE, interrupts);

  return HAL_OK;
}

/**
  * @brief  Disable the interrupts.
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  interrupts Interrupt signal(s) to disable - can be OR-ed with other interrupt signals.
  *         This parameter can be any combination of @ref FDCAN_Interrupt_Sources.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t interrupts)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_IT(interrupts));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->IE, interrupts);

  return HAL_OK;
}

/**
  * @brief  Check the interrupt status. Apply to one single interrupt signal.
  * @param  hfdcan    Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  interrupt Interrupt signal to check. Only one single interrupt signal can be checked at a time.
  *         This parameter must be one unique sample of @ref FDCAN_Interrupt_Sources.
  * @return Current interrupt status.
  */
hal_fdcan_it_status_t HAL_FDCAN_IsEnabledInterrupt(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check that the IT is valid */
  ASSERT_DBG_PARAM(IS_FDCAN_IT(interrupt));
  /* Check if only one it source was passed as parameter */
  ASSERT_DBG_PARAM(IS_FDCAN_SINGLE_BIT_SET(interrupt));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_it_status_t)STM32_IS_BIT_SET(FDCAN_GET_INSTANCE(hfdcan)->IE, interrupt);
}

/**
  * @brief  Enable the transmission buffer complete interrupt.
  * @param  hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffers_idx Buffers to select to enable the transmission complete interrupt - can be OR-ed.
  *         This parameter can be any combination of @ref FDCAN_IT_Tx_Complete_Buffers_Select
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableTxBufferCompleteInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check the buffer selection */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_COMPLETE_BUFFERS(tx_buffers_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXBTIE, (tx_buffers_idx & HAL_FDCAN_IT_TX_CPLT_BUFFER_ALL));

  return HAL_OK;
}

/**
  * @brief  Disable the transmission buffer complete interrupt.
  * @param  hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffers_idx Buffers to select to disable the transmission complete interrupt - can be OR-ed.
  *         This parameter can be any combination of @ref FDCAN_IT_Tx_Complete_Buffers_Select
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableTxBufferCompleteInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check the buffer selection */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_COMPLETE_BUFFERS(tx_buffers_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXBTIE, (tx_buffers_idx & HAL_FDCAN_IT_TX_CPLT_BUFFER_ALL));

  return HAL_OK;
}

/**
  * @brief  Check the status of the buffer connected to transmission complete interrupt.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffer_idx Transmission buffer index to check with transmission complete interrupt status.
  *         This is applicable to a single buffer only.
  *         This parameter must be an unique sample of @ref FDCAN_IT_Tx_Complete_Buffers_Select
  * @return Current Tx buffer transmission complete interrupt status.
  */
hal_fdcan_it_tx_buffer_complete_status_t HAL_FDCAN_IsEnabledTxBufferCompleteInterrupt(const hal_fdcan_handle_t *hfdcan,
    uint32_t tx_buffer_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Validate that only one buffer has to be checked */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_COMPLETE_BUFFERS(tx_buffer_idx) && IS_FDCAN_SINGLE_BIT_SET(tx_buffer_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_it_tx_buffer_complete_status_t)STM32_IS_BIT_SET(FDCAN_GET_INSTANCE(hfdcan)->TXBTIE, tx_buffer_idx);
}

/**
  * @brief  Enable the transmission cancellation finished interrupt.
  * @param  hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffers_idx The buffers on which to enable the transmission cancellation finished interrupt.
  *         This parameter can be any combination of @ref FDCAN_IT_Tx_Abort_Buffers_Select
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableTxBufferCancellationInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check the buffer selection */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_ABORT_BUFFERS(tx_buffers_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXBCIE, (tx_buffers_idx & HAL_FDCAN_IT_TX_ABORT_BUFFER_ALL));

  return HAL_OK;
}

/**
  * @brief  Disable the transmission cancellation finished interrupt.
  * @param  hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffers_idx The buffers on which to enable the transmission cancellation finished interrupt.
  *         This parameter can be any combination of @ref FDCAN_IT_Tx_Abort_Buffers_Select
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableTxBufferCancellationInterrupts(const hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);
  /* Check the buffer selection */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_ABORT_BUFFERS(tx_buffers_idx));

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->TXBCIE, (tx_buffers_idx & HAL_FDCAN_IT_TX_ABORT_BUFFER_ALL));

  return HAL_OK;
}

/**
  * @brief  Check the status of the buffer connected to transmission cancellation finished interrupt.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  tx_buffer_idx Transmission buffer index to check with transmission cancellation finished status.
            This is applicable to a single buffer only.
  *         This parameter must be an unique sample of @ref FDCAN_IT_Tx_Abort_Buffers_Select
  * @return Current Tx buffer abort finished interrupt status.
  */
hal_fdcan_it_tx_buffer_abort_status_t HAL_FDCAN_IsEnabledTxBufferCancellationInterrupt(const hal_fdcan_handle_t *hfdcan,
    uint32_t tx_buffer_idx)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Validate that only one buffer has to be checked */
  ASSERT_DBG_PARAM(IS_FDCAN_TX_ABORT_BUFFERS(tx_buffer_idx) && IS_FDCAN_SINGLE_BIT_SET(tx_buffer_idx));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (hal_fdcan_it_tx_buffer_abort_status_t)STM32_IS_BIT_SET(FDCAN_GET_INSTANCE(hfdcan)->TXBCIE, tx_buffer_idx);
}

/**
  * @brief  Assign the interrupt group(s) to an interrupt line.
  * @param  hfdcan            Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  interrupt_groups  Interrupt group(s) to connect to the given interrupt line.
  *         This parameter can be any combination of @ref FDCAN_Interrupt_Groups.
  * @param  it_line           Indicates which interrupt line must be assigned to the interrupt groups.
  *         This parameter must be one unique item of @ref FDCAN_Interrupt_Lines.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_SetInterruptGroupsToLine(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt_groups,
                                                uint32_t it_line)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_IT_GROUP(interrupt_groups));
  ASSERT_DBG_PARAM(IS_FDCAN_IT_LINE(it_line));
  /* Check if the prequested parameter concerns only a single interrupt line */
  ASSERT_DBG_PARAM(IS_FDCAN_SINGLE_BIT_SET(it_line));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  (it_line == HAL_FDCAN_IT_LINE_0) ? STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->ILS, interrupt_groups)
  : STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->ILS, interrupt_groups);


  return HAL_OK;
}

/**
  * @brief  Get the interrupt line assigned to an interrupt group - applies to a single interrupt group.
  * @param  hfdcan           Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  interrupt_group  Interrupt group.
  *         This parameter must be one unique item of @ref FDCAN_Interrupt_Groups.
  * @retval HAL_FDCAN_IT_LINE_0 Interrupt group is assigned to line 0.
  * @retval HAL_FDCAN_IT_LINE_1 Interrupt group is assigned to line 1.
  */
uint32_t HAL_FDCAN_GetLineFromInterruptGroup(const hal_fdcan_handle_t *hfdcan, uint32_t interrupt_group)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  ASSERT_DBG_PARAM(IS_FDCAN_IT_GROUP(interrupt_group));
  /* Ensure only one interrupt group was passed as parameter */
  ASSERT_DBG_PARAM(IS_FDCAN_SINGLE_BIT_SET(interrupt_group));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  return (STM32_IS_BIT_SET(FDCAN_GET_INSTANCE(hfdcan)->ILS, interrupt_group) ? HAL_FDCAN_IT_LINE_1
          : HAL_FDCAN_IT_LINE_0);
}

/**
  * @brief  Enable the interrupt line.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  it_lines Interrupt line(s) to enable. Can be OR-ed.
  *         This parameter can be a combination of @ref FDCAN_Interrupt_Lines.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_EnableInterruptLines(const hal_fdcan_handle_t *hfdcan, uint32_t it_lines)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check if the passed interrupt line(s) is valid */
  ASSERT_DBG_PARAM(IS_FDCAN_IT_LINE(it_lines));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_SET_BIT(FDCAN_GET_INSTANCE(hfdcan)->ILE, it_lines);

  return HAL_OK;
}

/**
  * @brief  Disable the interrupt lines.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  it_lines Interrupt line(s) to disable.
  *         This parameter can be any combination of @ref FDCAN_Interrupt_Lines.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_FDCAN_DisableInterruptLines(const hal_fdcan_handle_t *hfdcan, uint32_t it_lines)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));
  /* Check if the passed interrupt line(s) is valid */
  ASSERT_DBG_PARAM(IS_FDCAN_IT_LINE(it_lines));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  STM32_CLEAR_BIT(FDCAN_GET_INSTANCE(hfdcan)->ILE, it_lines);

  return HAL_OK;
}

/**
  * @brief  Check the interrupt line status.
  * @param  hfdcan  Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  it_line Interrupt line to be checked.
  *         This parameter must be a unique value of @ref FDCAN_Interrupt_Lines.
  * @return Current interrupt lines status.
  */
hal_fdcan_it_lines_status_t HAL_FDCAN_IsEnabledInterruptLine(const hal_fdcan_handle_t *hfdcan, uint32_t it_line)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  ASSERT_DBG_STATE(hfdcan->global_state, (uint32_t)HAL_FDCAN_STATE_IDLE | (uint32_t)HAL_FDCAN_STATE_ACTIVE
                   | (uint32_t)HAL_FDCAN_STATE_POWER_DOWN);

  /* Check if the passed interrupt line(s) is valid */
  ASSERT_DBG_PARAM(IS_FDCAN_IT_LINE(it_line));
  /* Check if the prequested parameter concerns only a single interrupt line */
  ASSERT_DBG_PARAM(IS_FDCAN_SINGLE_BIT_SET(it_line));

  return (hal_fdcan_it_lines_status_t)STM32_IS_BIT_SET(FDCAN_GET_INSTANCE(hfdcan)->ILE, it_line);
}

/**
  * @}
  */

/** @addtogroup FDCAN_Exported_Functions_Group7
  * @{
A set of weak functions if USE_HAL_FDCAN_REGISTER_CALLBACKS is set to 0 (or custom callbacks functions if
USE_HAL_FDCAN_REGISTER_CALLBACKS is set to 1) which are used to asynchronously informed the application in non-blocking
modes:
    - HAL_FDCAN_TxEventFifoCallback()             : Transmission event FIFO callback.
    - HAL_FDCAN_RxFifo0Callback()                 : Reception FIFO 0 callback.
    - HAL_FDCAN_RxFifo1Callback()                 : Reception FIFO 1 callback.
    - HAL_FDCAN_TxFifoEmptyCallback()             : Transmission FIFO empty callback.
    - HAL_FDCAN_TxBufferCompleteCallback()        : Transmission completed callback.
    - HAL_FDCAN_TxBufferAbortCallback()           : Abort transmission callback.
    - HAL_FDCAN_HighPriorityMessageCallback()     : High priority message receiving callback.
    - HAL_FDCAN_TimestampWraparoundCallback()     : Timestamp wrap around callback.
    - HAL_FDCAN_ErrorCallback()                   : Global error callback.
  */

/**
  * @brief   Tx event callback.
  * @param   hfdcan                   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   tx_event_fifo_interrupts Indicates which Tx event FIFO interrupts are raised.
  *          This parameter can be any combination of the following values:
  *           @arg @ref HAL_FDCAN_FLAG_TX_EVT_FIFO_ELEM_LOST
  *           @arg @ref HAL_FDCAN_FLAG_TX_EVT_FIFO_FULL
  *           @arg @ref HAL_FDCAN_FLAG_TX_EVT_FIFO_NEW_DATA
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_TxEventFifoCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_event_fifo_interrupts)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);
  STM32_UNUSED(tx_event_fifo_interrupts);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_TxEventFifoCallback must be implemented in the user file.
   */
}

/**
  * @brief   Rx FIFO 0 callback.
  * @param   hfdcan              Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   rx_fifo0_interrupts Indicates which Rx FIFO 0 interrupts are raised.
  *          This parameter can be any combination of the following values:
  *           @arg @ref HAL_FDCAN_FLAG_RX_FIFO_0_MSG_LOST
  *           @arg @ref HAL_FDCAN_FLAG_RX_FIFO_0_FULL
  *           @arg @ref HAL_FDCAN_FLAG_RX_FIFO_0_NEW_MSG
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_RxFifo0Callback(hal_fdcan_handle_t *hfdcan, uint32_t rx_fifo0_interrupts)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);
  STM32_UNUSED(rx_fifo0_interrupts);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_RxFifo0Callback must be implemented in the user file.
   */
}

/**
  * @brief   Rx FIFO 1 callback.
  * @param   hfdcan              Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   rx_fifo1_interrupts Indicates which Rx FIFO 1 interrupts are raised.
  *          This parameter can be any combination of the following values:
  *          @arg @ref HAL_FDCAN_FLAG_RX_FIFO_1_MSG_LOST
  *          @arg @ref HAL_FDCAN_FLAG_RX_FIFO_1_FULL
  *          @arg @ref HAL_FDCAN_FLAG_RX_FIFO_1_NEW_MSG
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_RxFifo1Callback(hal_fdcan_handle_t *hfdcan, uint32_t rx_fifo1_interrupts)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);
  STM32_UNUSED(rx_fifo1_interrupts);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_RxFifo1Callback must be implemented in the user file.
   */
}

/**
  * @brief   Tx FIFO Empty callback.
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_TxFifoEmptyCallback(hal_fdcan_handle_t *hfdcan)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_TxFifoEmptyCallback must be implemented in the user file.
   */
}

/**
  * @brief   Transmission complete callback.
  * @param   hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   tx_buffers_idx Indexes of the transmitted buffers.
  *          This parameter can be any combination of @ref FDCAN_IT_Tx_Complete_Buffers_Select.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_TxBufferCompleteCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);
  STM32_UNUSED(tx_buffers_idx);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_TxBufferCompleteCallback must be implemented in the user file.
   */
}

/**
  * @brief   Transmission cancellation callback.
  * @param   hfdcan         Pointer to a @ref hal_fdcan_handle_t handle.
  * @param   tx_buffers_idx Indexes of the aborted buffers.
  *          This parameter can be any combination of @ref FDCAN_IT_Tx_Abort_Buffers_Select.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_TxBufferAbortCallback(hal_fdcan_handle_t *hfdcan, uint32_t tx_buffers_idx)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);
  STM32_UNUSED(tx_buffers_idx);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_TxBufferAbortCallback must be implemented in the user file.
   */
}

/**
  * @brief   High priority message callback.
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_HighPriorityMessageCallback(hal_fdcan_handle_t *hfdcan)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_HighPriorityMessageCallback must be implemented in the user file.
   */
}

/**
  * @brief   Timestamp wrap around callback.
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_TimestampWraparoundCallback(hal_fdcan_handle_t *hfdcan)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_TimestampWraparoundCallback must be implemented in the user file.
   */
}

/**
  * @brief   Error callback.
  * @param   hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @note    The hfdcan handle's last_error_codes parameter is updated by the FDCAN processes,
  *          and the user can use HAL_FDCAN_GetLastErrorCodes() to verify the most recent error that occurred.
  * @warning This weak function must not be modified. When the callback is needed,
  *          it must be implemented in the user file.
  */
__WEAK void HAL_FDCAN_ErrorCallback(hal_fdcan_handle_t *hfdcan)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hfdcan);

  /* WARNING: This function must not be modified. When the callback is needed,
           function HAL_FDCAN_ErrorCallback must be implemented in the user file.
   */
}


/**
  * @}
  */

/** @addtogroup FDCAN_Exported_Functions_Group8
  * @{
A set of functions allowing to process with the state and last process errors.
  - HAL_FDCAN_GetState()            : Get the FDCAN state.
  - HAL_FDCAN_GetLastErrorCodes()   : Get the last error codes limited to the last process.
  */

/**
  * @brief  Return the FDCAN state.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Current HAL FDCAN state.
  */
hal_fdcan_state_t HAL_FDCAN_GetState(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  /* Return FDCAN handle state */
  return hfdcan->global_state;
}

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
/**
  * @brief  Return the FDCAN error code.
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @return Last error code. This parameter can be any combination of @ref FDCAN_Error_Codes.
 */
uint32_t HAL_FDCAN_GetLastErrorCodes(const hal_fdcan_handle_t *hfdcan)
{
  ASSERT_DBG_PARAM((hfdcan != NULL));

  /* Return FDCAN error code */
  return hfdcan->last_error_codes;
}
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup FDCAN_Exported_Functions_Group9
  * @{
A set of functions allowing to Acquire/Release the bus based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
    - HAL_FDCAN_AcquireBus(): Acquire the FDCAN bus.
    - HAL_FDCAN_ReleaseBus(): Release the FDCAN bus.
  */

/**
  * @brief  Acquire the FDCAN bus through the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hfdcan     Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  timeout_ms Timeout duration in millisecond.
  * @note   HAL_FDCAN_AcquireBus() must be called from thread mode only (not from handler mode i.e from ISR).
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR Semaphore cannot be taken.
  */
hal_status_t HAL_FDCAN_AcquireBus(hal_fdcan_handle_t *hfdcan, uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  if (HAL_OS_SemaphoreTake(&hfdcan->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the FDCAN bus through the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hfdcan Pointer to a @ref hal_fdcan_handle_t handle.
  * @note   HAL_FDCAN_ReleaseBus() can be called from thread mode or from handler mode i.e from ISR.
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR Semaphore cannot be released.
  */
hal_status_t HAL_FDCAN_ReleaseBus(hal_fdcan_handle_t *hfdcan)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hfdcan != NULL));

  if (HAL_OS_SemaphoreRelease(&hfdcan->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined(USE_HAL_FDCAN_USER_DATA) && (USE_HAL_FDCAN_USER_DATA == 1)
/** @addtogroup FDCAN_Exported_Functions_Group10
  * @{
A set of functions allowing to manage a user data pointer stored to the FDCAN handle:
  - HAL_FDCAN_SetUserData() : Configure the user data into the handle
  - HAL_FDCAN_GetUserData() : Get the user data from the handle
  */

/**
  * @brief Set the user data pointer into the handle.
  * @param hfdcan      Pointer to a hal_fdcan_handle_t.
  * @param p_user_data Pointer to the user data.
  */
void HAL_FDCAN_SetUserData(hal_fdcan_handle_t *hfdcan, const void *p_user_data)
{
  /* Check the FDCAN handle allocation */
  ASSERT_DBG_PARAM((hfdcan != NULL));

  hfdcan->p_user_data = p_user_data;
}
/**
  * @brief  Get the user data pointer from the handle.
  * @param  hfdcan Pointer to a hal_fdcan_handle_t.
  * @return Current pointer to the user data.
  */
const void *HAL_FDCAN_GetUserData(const hal_fdcan_handle_t *hfdcan)
{
  /* Check the FDCAN handle allocation */
  ASSERT_DBG_PARAM((hfdcan != NULL));

  return (hfdcan->p_user_data);
}

/**
  * @}
  */
#endif /* USE_HAL_FDCAN_USER_DATA == 1 */


/**
  * @}
  */

/* Private function implementations ----------------------------------------------------------------------------------*/
/** @addtogroup FDCAN_Private_Functions
  * @{
This section contains static functions used internally by the FDCAN HAL module to:
     - Configure nominal and data bit timing parameters.
     - Compute message RAM block addresses based on configuration.
     - Copy transmit messages into the message RAM.
     - Retrieve and interpret the FDCAN operating mode.
     - Wait on hardware flags with timeout handling.
     - Manage clock stop requests and acknowledgments.

  These functions are not exposed to the user application and are intended solely for internal
  driver operations to ensure modularity, maintainability, and encapsulation of hardware-specific logic.

  Usage of these functions is limited to within the FDCAN HAL driver source file.
  */

/**
  * @brief  Set the FDCAN nominal bit timing configuration.
  * @param  fdcan Pointer to a FDCANx hardware instance.
  * @param  p_timing Pointer to nominal bit timing structure @ref hal_fdcan_nominal_bit_timing_t.
  */
static void FDCAN_SetNominalBitTiming(FDCAN_GlobalTypeDef *fdcan, const hal_fdcan_nominal_bit_timing_t *p_timing)
{
  uint32_t register_value = ((p_timing->nominal_prescaler - 1U) << FDCAN_NBTP_NBRP_Pos)
                            | ((p_timing->nominal_jump_width - 1U) << FDCAN_NBTP_NSJW_Pos)
                            | ((p_timing->nominal_time_seg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)
                            | ((p_timing->nominal_time_seg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos);

  STM32_WRITE_REG(fdcan->NBTP, register_value);
}

/**
  * @brief  Get the FDCAN nominal bit timing configuration.
  * @param  fdcan Pointer to a FDCANx hardware instance.
  * @param  p_timing Pointer to the nominal bit timing structure @ref hal_fdcan_nominal_bit_timing_t to be
  *         filled with current configuration.
  */
static void FDCAN_GetNominalBitTiming(const FDCAN_GlobalTypeDef *fdcan, hal_fdcan_nominal_bit_timing_t *p_timing)
{
  uint32_t register_value = STM32_READ_REG(fdcan->NBTP);

  p_timing->nominal_prescaler  = (STM32_READ_BIT(register_value, FDCAN_NBTP_NBRP) >> FDCAN_NBTP_NBRP_Pos) + 1U;
  p_timing->nominal_jump_width = (STM32_READ_BIT(register_value, FDCAN_NBTP_NSJW) >> FDCAN_NBTP_NSJW_Pos) + 1U;
  p_timing->nominal_time_seg1  = (STM32_READ_BIT(register_value, FDCAN_NBTP_NTSEG1) >> FDCAN_NBTP_NTSEG1_Pos) + 1U;
  p_timing->nominal_time_seg2  = (STM32_READ_BIT(register_value, FDCAN_NBTP_NTSEG2) >> FDCAN_NBTP_NTSEG2_Pos) + 1U;
}

/**
  * @brief  Set the FDCAN data bit timing configuration.
  * @param  fdcan Pointer to a FDCANx hardware instance.
  * @param  p_timing Pointer to data bit timing structure @ref hal_fdcan_data_bit_timing_t.
  */
static void FDCAN_SetDataBitTiming(FDCAN_GlobalTypeDef *fdcan, const hal_fdcan_data_bit_timing_t *p_timing)
{
  uint32_t register_value = ((uint32_t)(p_timing->data_prescaler - 1U) << FDCAN_DBTP_DBRP_Pos)
                            | ((uint32_t)(p_timing->data_jump_width - 1U) << FDCAN_DBTP_DSJW_Pos)
                            | ((uint32_t)(p_timing->data_time_seg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)
                            | ((uint32_t)(p_timing->data_time_seg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos);

  STM32_WRITE_REG(fdcan->DBTP, register_value);
}

/**
  * @brief  Get the FDCAN data bit timing configuration.
  * @param  fdcan Pointer to a FDCANx hardware instance.
  * @param  p_timing Pointer to the data bit timing structure @ref hal_fdcan_data_bit_timing_t to be
  *         filled with current configuration.
  */
static void FDCAN_GetDataBitTiming(const FDCAN_GlobalTypeDef *fdcan, hal_fdcan_data_bit_timing_t *p_timing)
{
  uint32_t register_value = STM32_READ_REG(fdcan->DBTP);

  p_timing->data_time_seg2  = (STM32_READ_BIT(register_value, FDCAN_DBTP_DTSEG2_Msk) >> FDCAN_DBTP_DTSEG2_Pos) + 1U;
  p_timing->data_time_seg1  = (STM32_READ_BIT(register_value, FDCAN_DBTP_DTSEG1_Msk) >> FDCAN_DBTP_DTSEG1_Pos) + 1U;
  p_timing->data_prescaler  = (STM32_READ_BIT(register_value, FDCAN_DBTP_DBRP_Msk) >> FDCAN_DBTP_DBRP_Pos) + 1U;
  p_timing->data_jump_width = (STM32_READ_BIT(register_value, FDCAN_DBTP_DSJW_Msk) >> FDCAN_DBTP_DSJW_Pos) + 1U;
}

/**
  * @brief  Calculate each RAM block start address and size.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_config Pointer to a global configuration structure.
 */
static void FDCAN_ComputeRAMBlockAddresses(hal_fdcan_handle_t *hfdcan, const hal_fdcan_config_t *p_config)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t ram_counter;
  uint32_t sram_can_instance_base = SRAMCAN_BASE;

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

#if defined(FDCAN2)
  if (p_fdcanx == FDCAN2)
  {
    sram_can_instance_base += FDCAN_RAM_SIZE;
  }
#endif /* FDCAN2 */

  /* Assign start addresses for each RAM block */
  hfdcan->msg_ram.std_filter_start_addr = sram_can_instance_base + FDCAN_RAM_FLSSA;
  hfdcan->msg_ram.ext_filter_start_addr = sram_can_instance_base + FDCAN_RAM_FLESA;
  hfdcan->msg_ram.rx_fifo0_start_addr   = sram_can_instance_base + FDCAN_RAM_RF0SA;
  hfdcan->msg_ram.rx_fifo1_start_addr   = sram_can_instance_base + FDCAN_RAM_RF1SA;
  hfdcan->msg_ram.tx_event_start_addr   = sram_can_instance_base + FDCAN_RAM_TEFSA;
  hfdcan->msg_ram.tx_fifo_start_addr    = sram_can_instance_base + FDCAN_RAM_TFQSA;

  /* Configure filter element numbers in hardware */
  STM32_MODIFY_REG(p_fdcanx->RXGFC, FDCAN_RXGFC_LSS, (p_config->std_filters_nbr << FDCAN_RXGFC_LSS_Pos));
  STM32_MODIFY_REG(p_fdcanx->RXGFC, FDCAN_RXGFC_LSE, (p_config->ext_filters_nbr << FDCAN_RXGFC_LSE_Pos));

  /* Flush the allocated message RAM area */
  for (ram_counter = sram_can_instance_base; ram_counter < (sram_can_instance_base + FDCAN_RAM_SIZE); ram_counter += 4U)
  {
    *(uint32_t *)(ram_counter) = 0x00000000U;
  }
}

/**
  * @brief  Get the FDCAN messgage RAM configuration.
  * @param  p_fdcan  Pointer to a FDCANx hardware instance.
  * @param  p_config Pointer to the configuration structure to be filled with current configuration.
  *                  filled with current configuration.
  */
static void FDCAN_GetMessageRAMConfig(const FDCAN_GlobalTypeDef *p_fdcan, hal_fdcan_config_t *p_config)
{
  uint32_t register_value = STM32_READ_REG(p_fdcan->RXGFC);

  p_config->std_filters_nbr = STM32_READ_BIT(register_value, FDCAN_RXGFC_LSS) >> FDCAN_RXGFC_LSS_Pos;
  p_config->ext_filters_nbr = STM32_READ_BIT(register_value, FDCAN_RXGFC_LSE) >> FDCAN_RXGFC_LSE_Pos;
}

/**
  * @brief  Copy Tx message to the message RAM.
  * @param  hfdcan        Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  p_tx_header   Pointer to a @ref hal_fdcan_tx_header_t structure containing the Tx header.
  * @param  p_tx_data     Pointer to a buffer containing the payload of the Tx frame.
  * @param  tx_buffer_idx Bit position of the Tx buffer to be filled (0 for buffer 0, 1 for buffer 1, etc).
 */
static void FDCAN_CopyMessageToRAM(const hal_fdcan_handle_t *hfdcan, const hal_fdcan_tx_header_t *p_tx_header,
                                   const uint8_t *p_tx_data, uint32_t tx_buffer_idx)
{
  uint32_t *tx_address;
  uint32_t word_count;
  uint32_t payload_bytes = FDCAN_GET_DATA_LENGTH_BYTES(p_tx_header->b.data_length);
  const uint8_t *p_data = p_tx_data;
  /* Calculate Tx element address */
  tx_address = (uint32_t *)(hfdcan->msg_ram.tx_fifo_start_addr + (tx_buffer_idx * FDCAN_RAM_TFQ_SIZE));

  /* Write the Tx header (64 bits = 2 x 32 bits) */
  tx_address[0U] = (uint32_t)(p_tx_header->d64 & 0x00000000FFFFFFFFU);
  tx_address[1U] = (uint32_t)(p_tx_header->d64 >> 32U);

  /* Move pointer past header */
  tx_address = &tx_address[2U];

  /* Write Tx payload to the message RAM in 32-bit words */
  word_count = payload_bytes >> 2U;
  while (word_count > 0U)
  {
    *tx_address = (((uint32_t)p_data[3U] << 24U)
                   | ((uint32_t)p_data[2U] << 16U)
                   | ((uint32_t)p_data[1U] << 8U)
                   | (uint32_t)p_data[0U]);

    p_data = &p_data[4U];
    tx_address++;
    word_count--;
  }

  /* Handle remaining bytes if Data Length Code (DLC) is not a multiple of 4 */
  switch (payload_bytes & 0x3U)
  {
    case 3U:
      *tx_address = ((uint32_t)p_data[2U] << 16U) | ((uint32_t)p_data[1U] << 8U) | (uint32_t)p_data[0U];
      break;

    case 2U:
      *tx_address = ((uint32_t)p_data[1U] << 8U) | (uint32_t)p_data[0U];
      break;

    case 1U:
      *tx_address = (uint32_t)p_data[0U];
      break;

    default:
      /* Nothing to write */
      break;
  }
}

/**
  * @brief  Retrieve the FDCAN mode.
  * @param  register_value Specifies the FDCAN_CCCR register value.
  * @retval Current fdcan operation mode.
  */
static hal_fdcan_mode_t FDCAN_GetMode(const uint32_t register_value)
{
  hal_fdcan_mode_t selection_mode;

  /* Read the ASM, MON, and TEST bits from FDCAN_CCCR register */
  selection_mode = (hal_fdcan_mode_t)(uint32_t)STM32_READ_BIT(register_value,
                                                              FDCAN_CCCR_ASM | FDCAN_CCCR_MON | FDCAN_CCCR_TEST);

  switch ((uint32_t)selection_mode)
  {
    case HAL_FDCAN_MODE_EXTERNAL_LOOPBACK:
    case HAL_FDCAN_MODE_INTERNAL_LOOPBACK:
    case HAL_FDCAN_MODE_NORMAL:
    case HAL_FDCAN_MODE_RESTRICTED_OPERATION:
    case HAL_FDCAN_MODE_BUS_MONITORING:
      /* Valid modes, do nothing */
      break;

    default:
      /* All other configs are invalid */
      selection_mode = HAL_FDCAN_MODE_INVALID;
      break;
  }

  return selection_mode;
}

/**
  * @brief  Wait for a flag to reach a given status until timeout.
  * @param  fdcan       Pointer to a FDCANx hardware instance.
  * @param  flag        Specifies the FDCAN flag to check in CCCR register.
  * @param  status      The new flag status (0U or the flag value in register).
  * @param  timeout_ms  Timeout duration in millisecond.
  * @retval HAL_TIMEOUT Timeout exceeded.
  * @retval HAL_OK      Operation completed successfully.
  */
static hal_status_t FDCAN_WaitOnFlagUntilTimeout(const FDCAN_GlobalTypeDef *fdcan, uint32_t flag, uint32_t status,
                                                 uint32_t timeout_ms)
{
  /* Init tickstart for timeout management */
  uint32_t tickstart = HAL_GetTick();

  hal_status_t tmp_status = HAL_OK;

  while (STM32_READ_BIT(fdcan->CCCR, flag) == status)
  {
    /* Check for the timeout */
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      if (STM32_READ_BIT(fdcan->CCCR, flag) == status)
      {
        tmp_status = HAL_TIMEOUT;
      }
      break;
    }
  }
  return tmp_status;
}

/**
  * @brief  Clock stop request and wait for acknowledge.
  * @param  fdcan      Pointer to a FDCANx hardware instance.
  * @retval HAL_ERROR  No clock stop acknowledged.
  * @retval HAL_OK     Clock stop acknowledged.
  */
static hal_status_t FDCAN_ResetClockStopRequest(FDCAN_GlobalTypeDef *fdcan)
{
  /* Reset clock stop request */
  STM32_CLEAR_BIT(fdcan->CCCR, FDCAN_CCCR_CSR);

  /* Wait until FDCAN clock stop acknowledged */
  if (FDCAN_WaitOnFlagUntilTimeout(fdcan, FDCAN_CCCR_CSA, FDCAN_CCCR_CSA, FDCAN_GLOBAL_TIMEOUT_MS) != HAL_TIMEOUT)
  {
    return HAL_OK;
  }

  return HAL_ERROR;
}

/**
  * @brief  Initialization request and wait for acknowledge.
  * @param  fdcan      Pointer to a FDCANx hardware instance.
  * @retval HAL_ERROR  No Initialization acknowledged.
  * @retval HAL_OK     Initialization acknowledged.
  */
static hal_status_t FDCAN_InitRequest(FDCAN_GlobalTypeDef *fdcan)
{
  /* Request initialisation */
  STM32_SET_BIT(fdcan->CCCR, FDCAN_CCCR_INIT);

  /* Wait until the initialisation is accepted */
  if (FDCAN_WaitOnFlagUntilTimeout(fdcan, FDCAN_CCCR_INIT, 0U, FDCAN_GLOBAL_TIMEOUT_MS) != HAL_TIMEOUT)
  {
    return HAL_OK;
  }

  return HAL_ERROR;
}


/**
  * @brief  Compute line-specific interrupt flags from current pending+enabled interrupts.
  * @param  p_fdcanx Pointer to FDCAN registers.
  * @param  it_line  Interrupt line selector.
  * @retval Filtered interrupt flags for the requested line.
  */
static uint32_t FDCAN_GetLineSpecificInterruptFlags(const FDCAN_GlobalTypeDef *p_fdcanx, uint32_t it_line)
{
  uint32_t it_flags;

  /* Read FDCAN interrupt register and keep only currently enabled interrupt sources. */
  it_flags = STM32_READ_REG(p_fdcanx->IR);
  it_flags &= STM32_READ_REG(p_fdcanx->IE);

  uint32_t line1_groups = STM32_READ_REG(p_fdcanx->ILS) & HAL_FDCAN_IT_GROUP_MSK;
  uint32_t line1_mask = 0U;

  if ((line1_groups & HAL_FDCAN_IT_GROUP_RX_FIFO_0) != 0U)
  {
    line1_mask |= FDCAN_RX_FIFO_0_MSK;
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_RX_FIFO_1) != 0U)
  {
    line1_mask |= FDCAN_RX_FIFO_1_MSK;
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_STATUS_MSG) != 0U)
  {
    line1_mask |= (FDCAN_IR_HPM | FDCAN_IR_TC | FDCAN_IR_TCF);
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_TX_FIFO_ERROR) != 0U)
  {
    line1_mask |= (FDCAN_IR_TFE | FDCAN_TX_EVENT_FIFO_MSK);
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_MISC) != 0U)
  {
    line1_mask |= (FDCAN_IR_TSW | FDCAN_IR_MRAF | FDCAN_IR_TOO);
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_BIT_LINE_ERROR) != 0U)
  {
    line1_mask |= (FDCAN_IR_ELO | FDCAN_IR_EP);
  }
  if ((line1_groups & HAL_FDCAN_IT_GROUP_PROTOCOL_ERROR) != 0U)
  {
    line1_mask |= (FDCAN_IR_EW | FDCAN_IR_BO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA);
  }

  return ((it_line == HAL_FDCAN_IT_LINE_1) ? (it_flags & line1_mask)
          : (it_flags & ((~line1_mask) & FDCAN_IR_MSK)));
}

/**
  * @brief  Process a prepared list of FDCAN interrupt flags.
  * @param  hfdcan   Pointer to a @ref hal_fdcan_handle_t handle.
  * @param  it_flags Interrupt flags to process.
  */
static void FDCAN_HandleInterruptFlags(hal_fdcan_handle_t *hfdcan, uint32_t it_flags)
{
  FDCAN_GlobalTypeDef *p_fdcanx;
  uint32_t specific_its;
  uint32_t transmitted_buffers;
  uint32_t aborted_buffers;

  p_fdcanx = FDCAN_GET_INSTANCE(hfdcan);

  /* High priority message interrupt management: FDCAN_IR_HPM */
  if (STM32_IS_BIT_SET(it_flags, FDCAN_IR_HPM) != 0U)
  {
    /* Clear the high priority message flag */
    STM32_WRITE_REG(p_fdcanx->IR, FDCAN_IR_HPM);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_high_priority_msg_cb(hfdcan);
#else
    /* High priority message callback */
    HAL_FDCAN_HighPriorityMessageCallback(hfdcan);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Read if there is an IT related to Rx FIFO 0 group:
        - Rx FIFO 0 new message interrupt       - RF0N
        - Rx FIFO 0 full interrupt              - RF0F
        - Rx FIFO 0 message lost interrupt      - RF0L          */
  specific_its = it_flags & FDCAN_RX_FIFO_0_MSK;

  /* Rx FIFO 0 interrupts management: FDCAN_IR_RF0L, FDCAN_IR_RF0F, FDCAN_IR_RF0N */
  if (specific_its != 0U)
  {
    /* Clear the Rx FIFO 0 flags */
    STM32_WRITE_REG(p_fdcanx->IR, specific_its);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_rx_fifo_0_cb(hfdcan, specific_its);
#else
    /* Rx FIFO 0 callback */
    HAL_FDCAN_RxFifo0Callback(hfdcan, specific_its);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Read if there is an IT related to Rx FIFO 1 group:
        - Rx FIFO 1 new message interrupt       - RF1N
        - Rx FIFO 1 full interrupt              - RF1F
        - Rx FIFO 1 message lost interrupt      - RF1L          */
  specific_its = it_flags & FDCAN_RX_FIFO_1_MSK;

  /* Rx FIFO 1 interrupts management: FDCAN_IR_RF1L, FDCAN_IR_RF1F, FDCAN_IR_RF1N */
  if (specific_its != 0U)
  {
    /* Clear the Rx FIFO 1 flags */
    STM32_WRITE_REG(p_fdcanx->IR, specific_its);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_rx_fifo_1_cb(hfdcan, specific_its);
#else
    /* Rx FIFO 1 callback */
    HAL_FDCAN_RxFifo1Callback(hfdcan, specific_its);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Transmission abort interrupt management: FDCAN_IE_TCFE */
  if (STM32_IS_BIT_SET(it_flags, FDCAN_IR_TCF) != 0U)
  {
    /* List of aborted monitored buffers */
    aborted_buffers = STM32_READ_REG(p_fdcanx->TXBCF);
    aborted_buffers &= STM32_READ_REG(p_fdcanx->TXBCIE);

    /* Clear the transmission cancellation flag */
    STM32_WRITE_REG(p_fdcanx->IR, FDCAN_IR_TCF);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_tx_buffer_abort_cb(hfdcan, aborted_buffers);
#else
    /* Transmission cancellation callback */
    HAL_FDCAN_TxBufferAbortCallback(hfdcan, aborted_buffers);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Read if there is an IT related to Tx event group:
        - Tx event FIFO new entry interrupt     - TEFN
        - Tx event FIFO full interrupt          - TEFF
        - Tx event FIFO element lost interrupt  - TEFL          */
  specific_its = it_flags & FDCAN_TX_EVENT_FIFO_MSK;

  /* Tx event FIFO interrupts management: FDCAN_IR_TEFL, FDCAN_IR_TEFF, FDCAN_IR_TEFN */
  if (specific_its != 0U)
  {
    /* Clear the Tx event FIFO flags */
    STM32_WRITE_REG(p_fdcanx->IR, specific_its);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_tx_event_fifo_cb(hfdcan, specific_its);
#else
    /* Tx event FIFO callback */
    HAL_FDCAN_TxEventFifoCallback(hfdcan, specific_its);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Tx FIFO empty interrupt management: FDCAN_IR_TFE */
  if (STM32_IS_BIT_SET(it_flags, FDCAN_IR_TFE) != 0U)
  {
    /* Clear the Tx FIFO empty flag */
    STM32_WRITE_REG(p_fdcanx->IR, FDCAN_IR_TFE);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_tx_fifo_empty_cb(hfdcan);
#else
    /* Tx FIFO empty callback */
    HAL_FDCAN_TxFifoEmptyCallback(hfdcan);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Transmission complete interrupt management: FDCAN_IR_TC */
  if (STM32_IS_BIT_SET(it_flags, FDCAN_IR_TC) != 0U)
  {
    /* List of transmitted monitored buffers */
    transmitted_buffers = STM32_READ_REG(p_fdcanx->TXBTO);
    transmitted_buffers &= STM32_READ_REG(p_fdcanx->TXBTIE);

    /* Clear the transmission complete flag */
    STM32_WRITE_REG(p_fdcanx->IR, FDCAN_IR_TC);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_tx_buffer_complete_cb(hfdcan, transmitted_buffers);
#else
    /* Transmission complete callback */
    HAL_FDCAN_TxBufferCompleteCallback(hfdcan, transmitted_buffers);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Timestamp wrap around interrupt management: FDCAN_IR_TSW */
  if (STM32_IS_BIT_SET(it_flags, FDCAN_IR_TSW) != 0U)
  {
    /* Clear the timestamp wrap around flag */
    STM32_WRITE_REG(p_fdcanx->IR, FDCAN_IR_TSW);

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_ts_wraparound_cb(hfdcan);
#else
    /* Timestamp wrap around callback */
    HAL_FDCAN_TimestampWraparoundCallback(hfdcan);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }

  /* Error FDCAN interrupts management:
        - Error logging overflow interrupt      - ELO
        - Watchdog interrupt                    - WDI
        - Protocol error in arbitration phase   - PEA
        - Protocol error in data phase          - PED
        - Access to reserved address            - ARA
        - Message RAM access failure            - MRAF
        - Timeout occurred                      - TOO
        - Bus_Off                               - BO
        - Warning status                        - EW
        - Error passive                         - EP  */

  specific_its = it_flags & (FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO | FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA
                             | FDCAN_IR_PED | FDCAN_IR_ARA | FDCAN_IR_TOO | FDCAN_IR_MRAF);

  if (specific_its != 0U)
  {
    /* Clear the error flags */
    STM32_WRITE_REG(p_fdcanx->IR, specific_its);

#if defined(USE_HAL_FDCAN_GET_LAST_ERRORS) && (USE_HAL_FDCAN_GET_LAST_ERRORS == 1)
    /* Update the last_error_codes according to the detected error flags */
    hfdcan->last_error_codes |= specific_its;
#endif /* USE_HAL_FDCAN_GET_LAST_ERRORS */

#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    /* Call registered callback */
    hfdcan->p_error_cb(hfdcan);
#else
    /* Error callback */
    HAL_FDCAN_ErrorCallback(hfdcan);
#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */
  }
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_FDCAN_MODULE */
#endif /* FDCAN1 || FDCAN2 */

/**
  * @}
  */
