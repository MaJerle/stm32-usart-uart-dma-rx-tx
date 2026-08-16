/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_dma.c
  * @brief   This file provides DMA (Direct Memory Access) peripheral services.
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

#if (defined (LPDMA1) || defined (LPDMA2))
#if defined (USE_HAL_DMA_MODULE) && (USE_HAL_DMA_MODULE == 1)

/** @addtogroup DMA
  * @{
  */
/** @defgroup DMA_Introduction DMA Introduction
  * @{

In STM32 microcontrollers, the Direct Memory Access (DMA) controller enables data transfers between peripherals
and memory or between memory regions.
It supports multiple channels, programmable priority levels, and circular buffer management (circular mode).
A transfer can be associated with a peripheral request.

Event generation and interrupt configuration are available per channel, based on various conditions.

Additional features can be activated, including triggered channels (where the trigger can be a peripheral event
or a DMA channel event) and data-handling operations.

Support for privileged and unprivileged DMA transfers is available independently at the channel level.

For more information on linked list operations and queue management, refer to the Q Drivers.

  */
/**
  * @}
  */

/** @defgroup DMA_How_To_Use DMA How To Use
  * @{

This file provides firmware functions to manage the following DMA peripheral functionalities:

- Initialization and de-initialization functions
- Configuration functions
- Linked list node management functions
- Process management functions
- Callback functions
- Status functions

The direct memory access (DMA) controller is a bus master and system peripheral. It performs programmable
data transfers between memory-mapped peripherals and/or memories via linked list upon the control of an off-loaded CPU.

# DMA main features

- DMA transfer modes are divided into two major categories (direct transfer and linked list transfer)

- The DMA channel can be programmed to allow one-shot transfers using direct mode transfer APIs

- Alternatively to the direct programming mode, a DMA channel can be programmed by a list of transfers, known as
  linked list (list of Node items).
  - Each node is defined by its data structure
  - Each node specifies a standalone DMA channel transfer
  When enabled, the DMA channel fetches the first linked list node from SRAM (known as head node).
  After execution, the next linked list node is fetched and executed.
  This operation is repeated until the end of the whole linked list queue.
  The linked list can be linear, where the last linked list queue node is not linked to another queue node,
  or circular, where the last linked list node is linked to any linked list queue node.

  - Linear linked list:
    A linear linked list is a finite list where the last node (also called the tail node) points to NULL.
    A linear linked list transfer execution is finite and ends at the last node.
    The DMA channel fetches and executes the DMA linked list queue from the first node (head node) to the last node
    (tail node) once.
    When the last node is completed, the DMA channel remains idle and another transfer can be launched.

  - Circular linked list:
    A circular linked list is a list where the last node points to one of the previous nodes of the list.
    A circular linked list transfer execution must loop from the last node (tail node) to the node
    where the tail node points to.
    The DMA channel fetches and executes the DMA linked list queue from the first node (head node) to the last node
    (tail node). When the circular node is executed, the DMA channel fetches the next node and repeats the same
    sequence in an infinite loop (circular transfer). To stop the DMA channel, an abort operation is required.

  - Use the stm32tnxx_hal_q module to create a DMA queue based on DMA transfer nodes

- To reduce linked list queue execution time and power consumption, the DMA channel supports executing the
  dynamic linked list format. In fact, the DMA supports the execution of two types of linked list formats: static and
  dynamic.

  - Static linked list:
    The static linked list format refers to the full linked list node where all DMA channel parameters are fetched and
    executed independently of the redundancy of information.

  - Dynamic linked list:
    The dynamic linked list format refers to the customized linked list node where only DMA channel necessary parameters
    are fetched and executed (Example: data size = 20 on previous node, and data size = 20 on the current node => no
    need to update it).

- For linked list transfers, the DMA channel can execute the linked list queue node by node when started. Enable the
  DMA channel once to fetch the head node from memory, then it stops. Enable the DMA channel again to execute the node.
  After that, keep enabling the DMA channel to execute each node until the end of the linked list queue. When the
  linked list queue is circular, enable the DMA channel in an infinite loop to keep the DMA channel running. This
  feature is useful for debugging purposes or asynchronously executing queue nodes.

- Each DMA channel transfer (direct or linked list), is highly configurable according to DMA channel instance
  integrated in devices. These configurations can be:

  - Trigger configuration:
    The DMA channel transfers can be conditioned by hardware signal edges (rising or falling) named hardware triggers.
    Trigger conditions can be applied at:
    - Single/Burst level   : Each single/burst data transmission is conditioned by a signal trigger hit.
    - Block level          : Each block data transmission is conditioned by a signal trigger hit.
    - Node level           : Each node execution is conditioned by a signal trigger hit.
    The DMA channel can report a trigger overrun when it detects more than 2 trigger signal edges before executing the
    current transfer.

  - Data handling configuration:
    The data handling feature can be:
    - Padding pattern: Pad with the selected pattern (zero padding or sign extension) when the source data width is
                        smaller than the destination data width at single level.
    - Truncation     : Truncate data from the source when the source data width is larger than the destination data
                        width at single level.

- Each DMA channel transfer (direct or linked list), when it is active, can be suspended and
  resumed at run time by the application.
  When suspending an ongoing transfer, the DMA channel is not suspended immediately but completes the current
  single/burst transfer, then stops.
  When the DMA channel is suspended, the current transfer can be resumed immediately.

- A DMA burst means moving several pieces of data at once, instead of one by one (single transfer).

- A DMA block is a larger chunk of data that the DMA will move from one place to another. It can be made up
  of several bursts or single transfers.

# How to use the DMA HAL module driver

## Initialization and de-initialization:

- For a given channel, call the function HAL_DMA_Init() to initialize the DMA channel handle and associate the physical
  channel instance as direct mode by default.

- Call the function HAL_DMA_DeInit() to de-initialize a DMA channel.
After this function is called, the DMA channel is in reset.
It is mandatory to reinitialize it for the next transfer.

## Transfer configuration:

### Set the DMA channel direct transfer configuration

- HAL_DMA_SetConfigDirectXfer() function for the direct transfer mode.

- Optional: set the DMA channel direct transfer feature configuration:

  - Call the function HAL_DMA_SetConfigDirectXferHardwareRequestMode() to set the transfer hardware request mode
    configuration
  - Call the function HAL_DMA_SetConfigDirectXferFlowControlMode() to set the transfer flow control mode configuration
  - Call the function HAL_DMA_SetConfigDirectXferTrigger() to set the transfer trigger configuration
  - Call the function HAL_DMA_SetConfigDirectXferDataHandling() to set the transfer data handling configuration

  - Use the reset functions to reset each feature configuration (for example, HAL_DMA_ResetConfigDirectXferTrigger)

  - Use the get functions to get the configuration of any feature (for example, HAL_DMA_GetConfigDirectXfer)

  - Call the function HAL_DMA_SetConfigPeriphDirectXfer() to set the direct peripheral transfer configuration

### Set the DMA channel linked list transfer configuration

- HAL_DMA_SetConfigLinkedListXfer() function for the linked list transfer mode

- Optional: set the DMA channel linked list transfer feature configuration:

  - Call the function HAL_DMA_SetLinkedListXferEventMode() to set the transfer event mode configuration
  - Call the function HAL_DMA_SetLinkedListXferPriority() to set the transfer priority configuration
  - Call the function HAL_DMA_SetLinkedListXferExecutionMode() to set the transfer execution mode configuration

  - Use the reset functions to reset each feature configuration (for example, HAL_DMA_ResetLinkedListXferEventMode)

  - Use the get functions to get the configuration of any feature (for example, HAL_DMA_GetConfigLinkedListXfer)

  - Call the function HAL_DMA_SetConfigPeriphLinkedListCircularXfer() to set linked list circular peripheral transfer
    configuration

## Linked list node management:

- The linked list node management is a software process independent of DMA channel hardware. Use it to fill,
  convert (to dynamic or to static) nodes and use the Q module services to:
  - Initialize the queue
  - Insert a node into the queue
  - Remove a node from the queue
  - Replace a node in the queue
  - Circularize queue in order to perform infinite transfers.
  Linked list APIs and types are adapted to reduce memory footprint.

- At node level, the operations are filling a new linked list node or getting linked list node information from a
  filled node. The linked list nodes have two forms based on 2D addressing capability. The linear addressing nodes
  contain the information of all DMA channel features except the 2D addressing features, and the 2D addressing nodes
  contain the information of all available features.

  - Call the function HAL_DMA_FillNodeConfig() to fill the DMA linked list node according to the specified parameters.
    The fill operation converts the specified parameters into values recognized by the DMA channel and stores them
    in memory.
    When placing the DMA linked list in SRAM, ensure compliance with the product specifications to guarantee proper
    memory access.
    The DMA linked list node parameter address must be 32-bit aligned and
    must not exceed the 64 KB addressable space.

  - Call the function HAL_DMA_GetNodeConfig() to get the specified configuration parameter on filling node.
    This API can be used when a few parameters need to be changed to fill a new node.

  - As optional, fill the DMA channel linked list node feature configuration :

    - Call the function HAL_DMA_FillNodeHardwareRequestMode() to fill the DMA linked list node request mode
      configuration
    - Call the function HAL_DMA_FillNodeFlowControlMode() to fill the DMA linked list node flow control mode
      configuration
    - Call the function HAL_DMA_FillNodeXferEventMode() to fill the DMA linked list node transfer event mode
      configuration
    - Call the function HAL_DMA_FillNodeTrigger() to fill the DMA linked list node trigger configuration
    - Call the function HAL_DMA_FillNodeDataHandling() to fill the DMA linked list node data handling configuration
    - Call the function HAL_DMA_FillNodeData() to fill the DMA linked list node data configuration
    - Call the function HAL_DMA_FillNodeDirectXfer() to fill the DMA linked list node direct transfer configuration

  - To optimize DMA channel linked list queue execution, convert the built linked list queue to dynamic format
    (static is the default format). When the linked list queue becomes dynamic, all queue nodes are optimized and
    only changed parameters are updated between nodes. As a result, the DMA fetches only changed parameters instead
    of the whole node.

    - Call the function HAL_DMA_ConvertQNodesToDynamic() to convert a linked list queue to dynamic format.
      - Call this API for static queue format.
      - Call this API as the last API before starting the DMA channel in linked list mode.

    - Call the function HAL_DMA_ConvertQNodesToStatic() to convert a linked list queue to static format.
       - Call this API for dynamic queue format.
       - If the execution is dynamic and an update is needed on the linked list queue, then:
        - If the execution is linear   : Call this API as the first API after the full execution of
          linked list queue.
        - If the execution is circular : Call this API as the first API after aborting the execution of
          the current linked list queue.

  - When converting a circular queue to dynamic format and when the first circular node is the last queue node, it is
    recommended to duplicate the last circular node to ensure full optimization when calling
    HAL_DMA_ConvertQNodesToDynamic() API. In this case, the updated information is only addresses, which reduces
    4 words of update for linear nodes per node execution and 6 words of update for 2D addressing nodes per
    node execution.

## Process and callback management :

### Silent mode IO operation:

- Call the function HAL_DMA_StartDirectXfer() to start a DMA transfer in direct mode after the configuration of source
  address, destination address and the size of data to be transferred.

- Call the function HAL_DMA_StartLinkedListXfer() to start a DMA transfer in linked list mode after the configuration of
  linked list queue.

- Call the function HAL_DMA_PollForXfer() to poll for selected transfer level. In this case, configure a fixed timeout
  based on the application.
  Transfer level can be :
  - HAL_DMA_XFER_HALF_COMPLETE
  - HAL_DMA_XFER_FULL_COMPLETE
  For circular transfer, this API returns HAL_INVALID_PARAM.

- Call the function HAL_DMA_Suspend() to suspend any ongoing DMA transfer in blocking mode.
  This API returns HAL_ERROR when there is no ongoing transfer or timeout is reached when disabling the DMA channel.
  Do not call this API from an interrupt service routine.

- Call the function HAL_DMA_Resume() to resume any suspended DMA transfer immediately.

- Call the function HAL_DMA_Abort() to abort any ongoing DMA transfer in blocking mode.
  This API returns HAL_ERROR when there is no ongoing transfer or timeout is reached when disabling the DMA channel.
  This API accepts the idle state when trying to abort a yet finished transfer. It returns HAL_ERROR in this case.
  Do not call this API from an interrupt service routine.

### Interrupt mode IO operation:

- Configure the DMA interrupt priority using HAL_CORTEX_NVIC_SetPriority() function

- Enable the DMA IRQ handler using HAL_CORTEX_NVIC_EnableIRQ() function

- Call the function HAL_DMA_RegisterXferHalfCpltCallback() to register half transfer complete user callbacks.
- Call the function HAL_DMA_RegisterXferCpltCallback() to register transfer complete user callbacks.
- Call the function HAL_DMA_RegisterXferAbortCallback() to register transfer abort user callbacks.
- Call the function HAL_DMA_RegisterXferSuspendCallback() to register transfer suspend user callbacks.
- Call the function HAL_DMA_RegisterXferErrorCallback() to register transfer error user callbacks.

- Call the function HAL_DMA_StartDirectXfer_IT() to start the DMA transfer in direct mode after enabling the DMA
  default optional interrupts and the configuration of source address, destination address and the size of data
  to be transferred.

- Call the function HAL_DMA_StartDirectXfer_IT_Opt() to start the DMA transfer in direct mode after enabling the DMA
  customized optional interrupts and the configuration of source address, destination address and the size of data
  to be transferred.

- Call the function HAL_DMA_StartPeriphXfer_IT_Opt() to start a DMA channel peripheral transfer in direct or circular
  mode according to source address, destination address and the size of data to be transferred in byte parameters after
  enabling of the DMA channel mandatory interrupts for the process and enabling or disabling of the DMA channel optional
  interrupts for the process.\n
  Note: This function is intended exclusively for internal use by HAL PPP drivers for transfers over DMA.

- Call the function HAL_DMA_StartLinkedListXfer_IT() to start a DMA transfer in linked list mode after enabling the DMA
  default optional interrupts and configuration of linked list queue.

- Call the function HAL_DMA_StartLinkedListXfer_IT_Opt() to start a DMA transfer in linked list mode after the enable
  of DMA customized optional interrupts and configuration of linked list queue.

- Use HAL_DMA_IRQHandler() in the DMA_IRQHandler interrupt handler to handle DMA interrupts.

- Call the function HAL_DMA_Suspend_IT() to suspend any ongoing DMA transfer in interrupt mode.
  This API suspends the DMA channel execution. When the transfer is effectively suspended, an interrupt
  is generated and HAL_DMA_IRQHandler() must reset the channel and executes the transfer suspend user callbacks.
  Call this API from an interrupt service routine.

- Call the function HAL_DMA_Resume() to resume any suspended DMA transfer immediately.

- Call the function HAL_DMA_Abort_IT() to abort any ongoing DMA transfer in interrupt mode.
  This API suspends the DMA channel execution.
  When the transfer is effectively suspended, an interrupt is generated and HAL_DMA_IRQHandler() must reset the channel
  and executes the transfer abort user callbacks.
  This API accepts the idle state when trying to abort a yet finished transfer. It returns HAL_ERROR in this case.
  This accounts for asynchronous update of the DMA state to idle within the IRQHandler when the transfer is completed.
  Call this API from an interrupt service routine.

## Status and errors

- Call the function HAL_DMA_SetUserData() to set the DMA user data
- Call the function HAL_DMA_GetUserData() to get the DMA user data
- Call the function HAL_DMA_GetDirectXferRemainingDataByte() to get the DMA remaining data in the current transfer
  in byte
- Call the function HAL_DMA_GetState() to get the DMA current state
- Call the function HAL_DMA_GetLastErrorCodes() to get last error codes

  */
/**
  * @}
  */

/** @defgroup DMA_Configuration_Table DMA Configuration Table
  * @{
# Configuration inside the DMA driver

Config defines              | Description     | Default value   | Note                                                 |
----------------------------| --------------- | --------------- | -----------------------------------------------------|
PRODUCT                     | from IDE        |        NA       | The selected device (e.g., STM32C5XXxx)            |
USE_HAL_DMA_MODULE          | from hal_conf.h |        1        | Allows to use HAL DMA module.                        |
USE_ASSERT_DBG_PARAM        | from IDE        |      None       | Allows to use the assert check parameters.           |
USE_ASSERT_DBG_STATE        | from IDE        |      None       | Allows to use the assert check states.               |
USE_HAL_CHECK_PARAM         | from hal_conf.h |        0        | Allows to use the run-time checks parameters.        |
USE_HAL_CHECK_PROCESS_STATE | from hal_conf.h |        0        | Allows to use the load and store exclusive.          |
USE_HAL_DMA_CLK_ENABLE_MODEL| from hal_conf.h |HAL_CLK_ENABLE_NO| Allows to use the clock interface management for DMA.|
USE_HAL_DMA_GET_LAST_ERRORS | from hal_conf.h |        0        | Allows to use error code mechanism.                  |
USE_HAL_DMA_USER_DATA       | from hal_conf.h |        0        | Allows to use user data.                             |
USE_HAL_DMA_LINKEDLIST      | from hal_conf.h |        0        | Allows to use linked list services.                  |
  */
/**
  * @}
  */

/* Private Macros-----------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Private_Macros DMA Private Macros
  * @{
  */

/**
  * @brief  Check DMA hardware request selection.
  * @param  value DMA hardware request to check.
  * @retval 1U if value is a valid DMA hardware request, 0U otherwise.
  */
#if defined (XSPI1)
#define IS_DMA_REQUEST(value)                                                 \
  (((value) == HAL_DMA_REQUEST_SW) || ((value) <= HAL_LPDMA1_REQUEST_XSPI1))
#elif defined (DAC1) && defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define IS_DMA_REQUEST(value)                                                 \
  (((value) == HAL_DMA_REQUEST_SW) || ((value) <= HAL_LPDMA1_REQUEST_DAC1_CH2))
#else
#define IS_DMA_REQUEST(value)                                                 \
  (((value) == HAL_DMA_REQUEST_SW) || ((value) <= HAL_LPDMA1_REQUEST_DAC1_CH1))
#endif /* XSPI1 */

/**
  * @brief  Check DMA hardware request mode.
  * @param  value DMA hardware request mode to check.
  * @retval 1U if value is a valid DMA hardware request mode, 0U otherwise.
  */
#define IS_DMA_HARDWARE_REQUEST_MODE(value)               \
  (((value) == (uint32_t)HAL_DMA_HARDWARE_REQUEST_BURST ) \
   || ((value) == (uint32_t)HAL_DMA_HARDWARE_REQUEST_BLOCK))

/**
  * @brief  Check DMA flow control mode.
  * @param  value DMA flow control mode to check.
  * @retval 1U if value is a valid DMA flow control mode, 0U otherwise.
  */
#define IS_DMA_FLOW_CONTROL_MODE(value)                 \
  (((value) == (uint32_t)HAL_DMA_FLOW_CONTROL_DMA)      \
   || ((value) == (uint32_t)HAL_DMA_FLOW_CONTROL_PERIPH))
/**
  * @brief  Check DMA transfer direction.
  * @param  value DMA transfer direction to check.
  * @retval 1U if value is a valid DMA transfer direction, 0U otherwise.
  */
#define IS_DMA_DIRECTION(value)                                 \
  (((value) == (uint32_t)HAL_DMA_DIRECTION_MEMORY_TO_MEMORY)    \
   || ((value) == (uint32_t)HAL_DMA_DIRECTION_PERIPH_TO_MEMORY) \
   || ((value) == (uint32_t)HAL_DMA_DIRECTION_MEMORY_TO_PERIPH))

/**
  * @brief  Check DMA source address increment mode.
  * @param  value DMA source address increment mode to check.
  * @retval 1U if value is a valid DMA source increment mode, 0U otherwise.
  */
#define IS_DMA_SRC_INC(value)                            \
  (((value) == (uint32_t)HAL_DMA_SRC_ADDR_FIXED)         \
   || ((value) == (uint32_t)HAL_DMA_SRC_ADDR_INCREMENTED))

/**
  * @brief  Check DMA destination address increment mode.
  * @param  value DMA destination address increment mode to check.
  * @retval 1U if value is a valid DMA destination increment mode, 0U otherwise.
  */
#define IS_DMA_DEST_INC(value)                            \
  (((value) == (uint32_t)HAL_DMA_DEST_ADDR_FIXED)         \
   || ((value) == (uint32_t)HAL_DMA_DEST_ADDR_INCREMENTED))

/**
  * @brief  Check DMA source data width.
  * @param  value DMA source data width to check.
  * @retval 1U if value is a valid DMA source data width, 0U otherwise.
  */
#define IS_DMA_SRC_DATA_WIDTH(value)                         \
  (((value) == (uint32_t)HAL_DMA_SRC_DATA_WIDTH_BYTE)        \
   || ((value) == (uint32_t)HAL_DMA_SRC_DATA_WIDTH_HALFWORD) \
   || ((value) == (uint32_t)HAL_DMA_SRC_DATA_WIDTH_WORD))

/**
  * @brief  Check DMA destination data width.
  * @param  value DMA destination data width to check.
  * @retval 1U if value is a valid DMA destination data width, 0U otherwise.
  */
#define IS_DMA_DEST_DATA_WIDTH(value)                         \
  (((value) == (uint32_t)HAL_DMA_DEST_DATA_WIDTH_BYTE)        \
   || ((value) == (uint32_t)HAL_DMA_DEST_DATA_WIDTH_HALFWORD) \
   || ((value) == (uint32_t)HAL_DMA_DEST_DATA_WIDTH_WORD))

/**
  * @brief  Check DMA priority.
  * @param  value DMA priority to check.
  * @retval 1U if value is a valid DMA priority, 0U otherwise.
  */
#define IS_DMA_PRIORITY(value)                                \
  (((value) == (uint32_t)HAL_DMA_PRIORITY_LOW_WEIGHT_LOW)     \
   || ((value) == (uint32_t)HAL_DMA_PRIORITY_LOW_WEIGHT_MID)  \
   || ((value) == (uint32_t)HAL_DMA_PRIORITY_LOW_WEIGHT_HIGH) \
   || ((value) == (uint32_t)HAL_DMA_PRIORITY_HIGH))

/**
  * @brief  Check DMA trigger source.
  * @param  value DMA trigger source to check.
  * @retval 1U if value is a valid DMA trigger source, 0U otherwise.
  */
#if defined (COMP2)
#define IS_DMA_TRIGGER_SOURCE(value)                \
  ((value) <= (uint32_t)HAL_LPDMA1_TRIGGER_COMP2_OUT)
#elif defined (LPDMA2_CH7)
#define IS_DMA_TRIGGER_SOURCE(value)                    \
  ((value) <= (uint32_t)HAL_LPDMA1_TRIGGER_LPDMA2_CH7_TC)
#else
#define IS_DMA_TRIGGER_SOURCE(value)                    \
  ((value) <= (uint32_t)HAL_LPDMA1_TRIGGER_LPDMA2_CH3_TC)
#endif /* COMP2 */

/**
  * @brief  Check DMA trigger polarity.
  * @param  value DMA trigger polarity to check.
  * @retval 1U if value is a valid DMA trigger polarity, 0U otherwise.
  */
#define IS_DMA_TRIGGER_POLARITY(value)                       \
  (((value) == (uint32_t)HAL_DMA_TRIGGER_POLARITY_MASKED)    \
   || ((value) == (uint32_t)HAL_DMA_TRIGGER_POLARITY_RISING) \
   || ((value) == (uint32_t)HAL_DMA_TRIGGER_POLARITY_FALLING))

/**
  * @brief  Check DMA trigger mode.
  * @param  value DMA trigger mode to check.
  * @retval 1U if value is a valid DMA trigger mode, 0U otherwise.
  */
#define IS_DMA_TRIGGER_MODE(value)                                \
  (((value) == (uint32_t)HAL_DMA_TRIGGER_BLOCK_TRANSFER)          \
   || ((value) == (uint32_t)HAL_DMA_TRIGGER_NODE_TRANSFER)        \
   || ((value) == (uint32_t)HAL_DMA_TRIGGER_SINGLE_BURST_TRANSFER))

/**
  * @brief  Check DMA destination data truncation and padding mode.
  * @param  value DMA destination data truncation and padding mode to check.
  * @retval 1U if value is a valid DMA destination truncation and padding mode, 0U otherwise.
  */
#define IS_DMA_DEST_DATA_TRUNC_PADD(value)                          \
  (((value) == (uint32_t)HAL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO)    \
   || ((value) == (uint32_t)HAL_DMA_DEST_DATA_TRUNC_RIGHT_PADD_SIGN))

/**
  * @brief  Check DMA linked list transfer event mode.
  * @param  value DMA linked list transfer event mode to check.
  * @retval 1U if value is a valid DMA linked list transfer event mode, 0U otherwise.
  */
#define IS_DMA_LINKEDLIST_XFER_EVENT_MODE(value)                \
  (((value) == (uint32_t)HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK)   \
   || ((value) == (uint32_t)HAL_DMA_LINKEDLIST_XFER_EVENT_NODE) \
   || ((value) == (uint32_t)HAL_DMA_LINKEDLIST_XFER_EVENT_Q))

/**
  * @brief  Check DMA linked list execution mode.
  * @param  value DMA linked list execution mode to check.
  * @retval 1U if value is a valid DMA linked list execution mode, 0U otherwise.
  */
#define IS_DMA_LINKEDLIST_EXEC_MODE(value)                    \
  (((value) == (uint32_t)HAL_DMA_LINKEDLIST_EXECUTION_Q)      \
   || ((value) == (uint32_t)HAL_DMA_LINKEDLIST_EXECUTION_NODE))


/**
  * @brief  Check DMA privilege attribute.
  * @param  value DMA privilege attribute to check.
  * @retval 1U if value is a valid DMA privilege attribute, 0U otherwise.
  */
#define IS_DMA_PRIV_ATTR(value)   \
  (((value) == HAL_DMA_NPRIV)     \
   || ((value) == HAL_DMA_PRIV))

/**
  * @brief  Check DMA optional interrupt selection.
  * @param  value DMA optional interrupt selection to check.
  * @retval 1U if value is a valid optional interrupt selection, 0U otherwise.
  */
#define IS_DMA_OPT_IT(value)              \
  (((value) == HAL_DMA_OPT_IT_NONE)       \
   || ((value) == HAL_DMA_OPT_IT_HT)      \
   || ((value) == HAL_DMA_OPT_IT_TO)      \
   || ((value) == HAL_DMA_OPT_IT_DEFAULT) \
   || ((value) == HAL_DMA_OPT_IT_SILENT))

/**
  * @brief  Check DMA transfer level.
  * @param  value DMA transfer level to check.
  * @retval 1U if value is a valid DMA transfer level, 0U otherwise.
  */
#define IS_DMA_XFER_LEVEL(value)                       \
  (((value) == (uint32_t)HAL_DMA_XFER_FULL_COMPLETE)   \
   || ((value) == (uint32_t)HAL_DMA_XFER_HALF_COMPLETE))

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief  Get the DMA node type.
  * @param  instance DMA channel instance.
  * @retval Linear addressing.
  */
#define DMA_GET_NODE_TYPE(instance)  HAL_DMA_NODE_LINEAR_ADDRESSING
#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief  Get the DMA channel instance.
  * @param  handle DMA handle.
  * @retval DMA channel instance.
  */
#define DMA_CHANNEL_GET_INSTANCE(handle)                \
  ((DMA_Channel_TypeDef *)((uint32_t)(handle)->instance))

/*! Macro to define DMA CTR1 register offset */
#define DMA_NODE_CTR1_REG_OFFSET  LL_DMA_NODE_CTR1_REG_OFFSET

/*! Macro to define DMA CTR2 register offset */
#define DMA_NODE_CTR2_REG_OFFSET  LL_DMA_NODE_CTR2_REG_OFFSET

/*! Macro to define DMA CBR1 register offset */
#define DMA_NODE_CBR1_REG_OFFSET  LL_DMA_NODE_CBR1_REG_OFFSET

/*! Macro to define DMA CSAR register offset */
#define DMA_NODE_CSAR_REG_OFFSET  LL_DMA_NODE_CSAR_REG_OFFSET

/*! Macro to define DMA CDAR register offset */
#define DMA_NODE_CDAR_REG_OFFSET  LL_DMA_NODE_CDAR_REG_OFFSET


/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup DMA_Private_Constants DMA Private Constants
  * @{
  */
#define DMA_SUSPEND_TIMEOUT (1U)          /*!< 1 ms is needed to suspend the DMA channel */

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define HAL_DMA_FLAG_ERROR (LL_DMA_FLAG_DTE | LL_DMA_FLAG_ULE | LL_DMA_FLAG_USE) /*!< DMA Flag error */
#else
#define HAL_DMA_FLAG_ERROR (LL_DMA_FLAG_DTE | LL_DMA_FLAG_USE)                   /*!< DMA Flag error */
#endif /* USE_HAL_DMA_LINKEDLIST */

#define DMA_NODE_CLLR_IDX            (0x0700U)                   /*!< DMA channel node CLLR index mask     */
#define DMA_NODE_CLLR_IDX_POS        (0x0008U)                   /*!< DMA channel node CLLR index position */
#define DMA_NODE_REGISTER_NUM        LL_DMA_NODE_REGISTER_NUM    /*!< DMA channel node register number     */
#define DMA_NODE_STATIC_FORMAT       (0x0000U)                   /*!< DMA channel node static format       */
#define DMA_NODE_DYNAMIC_FORMAT      (0x0001U)                   /*!< DMA channel node dynamic format      */
#define DMA_UPDATE_CLLR_POSITION     (0x0000U)                   /*!< DMA channel update CLLR position     */
#define DMA_UPDATE_CLLR_VALUE        (0x0001U)                   /*!< DMA channel update CLLR value        */
#define DMA_LASTNODE_ISNOT_CIRCULAR  (0x0000U)                   /*!< Last node is not first circular node */
#define DMA_LASTNODE_IS_CIRCULAR     (0x0001U)                   /*!< Last node is first circular node     */
#define DMA_NODE_CSAR_DEFAULT_OFFSET (0x0003U)                   /*!< CSAR default offset                  */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @addtogroup DMA_Private_Functions
  * @{
  */
static void DMA_SetConfigDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config);

static void DMA_GetConfigDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
static void DMA_SetConfigLinkedListXfer(hal_dma_handle_t *hdma, const hal_dma_linkedlist_xfer_config_t *p_config);

static void DMA_GetConfigLinkedListXfer(hal_dma_handle_t *hdma, hal_dma_linkedlist_xfer_config_t *p_config);

static void DMA_FillNodeConfig(hal_dma_node_t *p_node, const hal_dma_node_config_t *p_conf,
                               hal_dma_node_type_t node_type);

static void DMA_GetConfigNode(const hal_dma_node_t *p_node, hal_dma_node_config_t *p_conf,
                              hal_dma_node_type_t *p_node_type);

static void DMA_FillNodeDirectXfer(hal_dma_node_t *p_node, const hal_dma_direct_xfer_config_t *p_config,
                                   hal_dma_node_type_t node_type, hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode);

static void DMA_UpdateDataNode(hal_dma_node_t *p_node, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte);

static void DMA_ConvertQNodesToDynamic(hal_q_t *p_q);

static void DMA_ConvertQNodesToStatic(hal_q_t *p_q);

static void DMA_List_ConvertNodeToDynamic(uint32_t context_node_addr, uint32_t current_node_addr, uint32_t reg_nbr);

static void DMA_List_ConvertNodeToStatic(uint32_t context_node_addr, uint32_t current_node_addr, uint32_t reg_nbr);

static void DMA_List_UpdateDynamicQueueNodesCLLR(const hal_q_t *p_q, uint32_t last_node_is_circular);

static void DMA_List_UpdateStaticQueueNodesCLLR(hal_q_t *p_q, uint32_t operation);

static void DMA_List_GetCLLRNodeInfo(const hal_dma_node_t *p_node, uint32_t *p_cllr_mask, uint32_t *p_cllr_offset);

static void DMA_List_FormatNode(hal_dma_node_t *p_node, uint32_t reg_idx, uint32_t reg_nbr, uint32_t format);

static void DMA_List_ClearUnusedFields(hal_dma_node_t *p_node, uint32_t first_unused_field);
#endif /* USE_HAL_DMA_LINKEDLIST */

static void DMA_StartDirectXfer(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte,
                                uint32_t interrupts);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
static void DMA_StartLinkedListXfer(hal_dma_handle_t *hdma, const void *p_head_node, uint32_t interrupts);
#endif /* USE_HAL_DMA_LINKEDLIST */

static void DMA_HandleErrorIT(hal_dma_handle_t *hdma, uint32_t error_msk);
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup DMA_Exported_Functions
  * @{
  */

/** @addtogroup DMA_Exported_Functions_Group1
  * @{
This subsection provides a set of functions to initialize and de-initialize a DMA channel peripheral:

- Call the function HAL_DMA_Init() to initialize the DMA channel handle and associate a physical channel instance.
  (As optional, DMA clock is enabled inside the function)

- Call the function HAL_DMA_DeInit() to restore the physical and logical default configuration (after reset) of
  the selected DMA channel peripheral.

  */

/**
  * @brief  Initialize the DMA channel handle and associate physical channel instance.
  * @param  hdma              Pointer to DMA channel handle
  * @param  instance          DMA channel instance
  * @retval HAL_INVALID_PARAM Invalid parameter when hdma pointer is NULL
  * @retval HAL_OK            DMA channel is successfully initialized
  */
hal_status_t HAL_DMA_Init(hal_dma_handle_t *hdma, hal_dma_channel_t instance)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)instance)));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->instance = instance;

#if defined(USE_HAL_DMA_CLK_ENABLE_MODEL) && (USE_HAL_DMA_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  if (LL_DMA_GET_INSTANCE(hdma->instance) == LPDMA1)
  {
    HAL_RCC_LPDMA1_EnableClock();
  }
#if defined(LPDMA2)
  else
  {
    HAL_RCC_LPDMA2_EnableClock();
  }
#endif /* LPDMA2 */
#endif /* USE_HAL_DMA_CLK_ENABLE_MODEL */

  hdma->p_xfer_halfcplt_cb = HAL_DMA_XferHalfCpltCallback;
  hdma->p_xfer_cplt_cb     = HAL_DMA_XferCpltCallback;
  hdma->p_xfer_abort_cb    = HAL_DMA_XferAbortCallback;
  hdma->p_xfer_suspend_cb  = HAL_DMA_XferSuspendCallback;
  hdma->p_xfer_error_cb    = HAL_DMA_XferErrorCallback;

#if defined(USE_HAL_DMA_USER_DATA) && (USE_HAL_DMA_USER_DATA == 1)
  hdma->p_user_data = NULL;
#endif /* USE_HAL_DMA_USER_DATA */

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hdma->xfer_mode = HAL_DMA_XFER_MODE_DIRECT;
#endif /* USE_HAL_DMA_LINKEDLIST */

  hdma->global_state = HAL_DMA_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief Deinitialize the DMA channel handle and the associated physical channel instance.
  * @param hdma Pointer to DMA channel handle
  */
void HAL_DMA_DeInit(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE(DMA_CHANNEL_GET_INSTANCE(hdma)));

  LL_DMA_DisableChannel(DMA_CHANNEL_GET_INSTANCE(hdma));
  LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

  LL_DMA_WRITE_REG(DMA_CHANNEL_GET_INSTANCE(hdma), CCR, 0U);
  LL_DMA_WRITE_REG(DMA_CHANNEL_GET_INSTANCE(hdma), CTR1, 0U);
  LL_DMA_WRITE_REG(DMA_CHANNEL_GET_INSTANCE(hdma), CTR2, 0U);
  LL_DMA_WRITE_REG(DMA_CHANNEL_GET_INSTANCE(hdma), CBR1, 0U);
  LL_DMA_WRITE_REG(DMA_CHANNEL_GET_INSTANCE(hdma), CLLR, 0U);

  hdma->global_state = HAL_DMA_STATE_RESET;
}

/**
  * @brief Retrieve the HAL DMA channel instance.
  * @param hdma Pointer to DMA channel handle.
  * @retval DMA channel instance, element in @ref hal_dma_channel_t  enumeration.
  */
hal_dma_channel_t HAL_DMA_GetInstance(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);

  return hdma->instance;
}

/**
  * @brief Retrieve the LL DMA channel instance.
  * @param hdma Pointer to DMA channel handle.
  * @retval DMA channel instance.
  */
DMA_Channel_TypeDef *HAL_DMA_GetLLInstance(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  return ((DMA_Channel_TypeDef *)((uint32_t)((hdma)->instance)));
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group2
  * @{

This subsection provides a set of functions to configure the DMA channel peripheral:

  Basic transfer configuration

- Call the function HAL_DMA_SetConfigDirectXfer() to configure the DMA channel basic transfer according to configured
  parameter within hal_dma_direct_xfer_config_t structure

- Call the function HAL_DMA_GetConfigDirectXfer() to get the current configured basic transfer

  Hardware request mode transfer configuration

- Call the function HAL_DMA_SetConfigDirectXferHardwareRequestMode() to configure the DMA channel transfer hardware
  request mode according to configured parameters

- Call the function HAL_DMA_ResetConfigDirectXferHardwareRequestMode() to reset the DMA channel transfer hardware
  request mode configuration

- Call the function HAL_DMA_GetConfigDirectXferHardwareRequestMode() to get the current configured transfer hardware
  request mode

  Flow control mode configuration

- Call the function HAL_DMA_SetConfigDirectXferFlowControlMode() to configure the DMA channel flow control mode

- Call the function HAL_DMA_ResetConfigDirectXferFlowControlMode() to reset the DMA channel flow control mode

- Call the function HAL_DMA_GetConfigDirectXferFlowControlMode() to get the current configured transfer flow
  control mode

  Trigger transfer configuration

- Call the function HAL_DMA_SetConfigDirectXferTrigger() to configure the DMA channel trigger according to configured
  parameter within hal_dma_trigger_config_t structure

- Call the function HAL_DMA_ResetConfigDirectXferTrigger() to reset the DMA channel transfer trigger configuration

- Call the function HAL_DMA_GetConfigDirectXferTrigger() to get the current configured trigger

  Data handling transfer configuration

- Call the function HAL_DMA_SetConfigDirectXferDataHandling() to configure the DMA channel data handling according to
  configured parameter within hal_dma_data_handling_config_t structure

- Call the function HAL_DMA_ResetConfigDirectXferDataHandling() to reset the DMA channel transfer data handling
  configuration

- Call the function HAL_DMA_GetConfigDirectXferDataHandling() to get the current configured data handling


  Channel privilege attribute configuration

- Call the function HAL_DMA_SetPrivAttr() to configure the DMA channel privileged access level attribute

- Call the function HAL_DMA_GetPrivAttr() to get the DMA channel privileged access level attribute

  Channel lock attributes configuration

- Call the function HAL_DMA_LockAttr() to lock the DMA channel privileged access level attribute

- Call the function HAL_DMA_IsLockedAttr() to check the DMA channel privileged access level attribute lock

  Peripherals direct transfer configuration

- Call the function HAL_DMA_SetConfigPeriphDirectXfer() to configure the DMA channel peripheral direct transfer
  according to configured parameter within hal_dma_direct_xfer_config_t structure:

- Call the function HAL_DMA_GetConfigPeriphDirectXfer() to get the current configured direct transfer

  Linked list transfer configuration

- Call the function HAL_DMA_SetConfigLinkedListXfer() to configure the DMA channel linked list transfer according to
  configured parameter within hal_dma_linkedlist_xfer_config_t structure

- Call the function HAL_DMA_GetConfigLinkedListXfer() to get the current configured linked list transfer

  Event mode transfer configuration

- Call the function HAL_DMA_SetLinkedListXferEventMode() to configure the DMA channel event mode according to
  selected parameter within hal_dma_linkedlist_xfer_event_mode_t enumeration

- Call the function HAL_DMA_ResetLinkedListXferEventMode() to reset the DMA channel event mode configuration

- Call the function HAL_DMA_GetLinkedListXferEventMode() to get the current configured event mode

  Priority transfer configuration

- Call the function HAL_DMA_SetLinkedListXferPriority() to configure the DMA channel priority according to selected
  parameter within hal_dma_priority_t enumeration

- Call the function HAL_DMA_ResetLinkedListXferPriority() to reset the DMA channel priority configuration

- Call the function HAL_DMA_GetLinkedListXferPriority() to get the current configured priority

  Execution mode transfer configuration

- Call the function HAL_DMA_SetLinkedListXferExecutionMode() to configure the DMA channel execution mode according to
  selected parameter within hal_dma_linkedlist_execution_mode_t enumeration

- Call the function HAL_DMA_ResetLinkedListXferExecutionMode() to reset the DMA channel execution mode configuration

- Call the function HAL_DMA_GetLinkedListXferExecutionMode() to get the current configured execution mode

  Peripherals linked list circular transfer configuration

- Call the function HAL_DMA_SetConfigPeriphLinkedListCircularXfer() to configure the DMA channel peripheral linked list
  circular transfer according to configured parameter within hal_dma_direct_xfer_config_t structure

- Call the function HAL_DMA_GetConfigPeriphLinkedListCircularXfer() to get the current configured peripheral linked list
  circular transfer
  */

/**
  * @brief  Set the DMA channel direct transfer configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_config          Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_config pointer is NULL
  * @retval HAL_OK            Direct transfer is successfully configured
  */
hal_status_t HAL_DMA_SetConfigDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_REQUEST((uint32_t)p_config->request));
  ASSERT_DBG_PARAM(IS_DMA_DIRECTION((uint32_t)p_config->direction));
  ASSERT_DBG_PARAM(IS_DMA_SRC_INC((uint32_t)p_config->src_inc));
  ASSERT_DBG_PARAM(IS_DMA_DEST_INC((uint32_t)p_config->dest_inc));
  ASSERT_DBG_PARAM(IS_DMA_SRC_DATA_WIDTH((uint32_t)p_config->src_data_width));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_WIDTH((uint32_t)p_config->dest_data_width));
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)p_config->priority));
  ASSERT_DBG_STATE(hdma->global_state, (uint32_t)HAL_DMA_STATE_INIT | (uint32_t)HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_SetConfigDirectXfer(hdma, p_config);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hdma->xfer_mode = HAL_DMA_XFER_MODE_DIRECT;
#endif /* USE_HAL_DMA_LINKEDLIST */

  hdma->global_state = HAL_DMA_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get the DMA channel direct transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  */
void HAL_DMA_GetConfigDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  DMA_GetConfigDirectXfer(hdma, p_config);
}

/**
  * @brief  Set the DMA channel direct transfer hardware request mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  hw_request_mode   Element in @ref hal_dma_hardware_request_mode_t  enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_OK            Request mode is successfully configured
  */
hal_status_t HAL_DMA_SetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma,
                                                            hal_dma_hardware_request_mode_t  hw_request_mode)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(IS_DMA_HARDWARE_REQUEST_MODE((uint32_t)hw_request_mode));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_DMA_SetHWRequestMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)hw_request_mode);

  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel direct transfer hardware request mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_OK            Reset direct transfer request mode configuration is successful
  */
hal_status_t HAL_DMA_ResetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_DMA_SetHWRequestMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_HARDWARE_REQUEST_BURST);

  return HAL_OK;
}

/**
  * @brief Get the DMA channel direct transfer hardware request mode configuration.
  * @param hdma     Pointer to DMA channel handle
  * @retval Returned value can be one of the following values:
  *         @arg @ref HAL_DMA_HARDWARE_REQUEST_BURST
  *         @arg @ref HAL_DMA_HARDWARE_REQUEST_BLOCK
  */
hal_dma_hardware_request_mode_t  HAL_DMA_GetConfigDirectXferHardwareRequestMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  return ((hal_dma_hardware_request_mode_t)LL_DMA_GetHWRequestType(DMA_CHANNEL_GET_INSTANCE(hdma)));
}

/**
  * @brief  Set the DMA channel direct transfer flow control mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  flow_control_mode Element in @ref hal_dma_flow_control_mode_t enumeration
  * @retval HAL_OK            Direct transfer flow control mode is successfully configured
  */
hal_status_t HAL_DMA_SetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma,
                                                        hal_dma_flow_control_mode_t flow_control_mode)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_PFREQ_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)hdma->instance)));
  ASSERT_DBG_PARAM(IS_DMA_FLOW_CONTROL_MODE((uint32_t)flow_control_mode));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  LL_DMA_SetFlowControlMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)flow_control_mode);

  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel direct transfer flow control mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_OK            Direct transfer flow control mode is successfully configured
  */
hal_status_t HAL_DMA_ResetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_PFREQ_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)hdma->instance)));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  LL_DMA_SetFlowControlMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_FLOW_CONTROL_DMA);

  return HAL_OK;
}

/**
  * @brief  Get the DMA channel direct transfer flow control mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval Returned value can be one of the following values:
  *         @arg @ref HAL_DMA_FLOW_CONTROL_DMA
  *         @arg @ref HAL_DMA_FLOW_CONTROL_PERIPH
  */
hal_dma_flow_control_mode_t HAL_DMA_GetConfigDirectXferFlowControlMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_PFREQ_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)hdma->instance)));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  return ((hal_dma_flow_control_mode_t)LL_DMA_GetFlowControlMode(DMA_CHANNEL_GET_INSTANCE(hdma)));
}

/**
  * @brief  Set the DMA channel direct transfer trigger configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_config          Pointer to @ref hal_dma_trigger_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct or p_config pointer is
  *                           NULL
  * @retval HAL_OK            Direct transfer trigger is successfully configured
  */
hal_status_t HAL_DMA_SetConfigDirectXferTrigger(hal_dma_handle_t *hdma, const hal_dma_trigger_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_SOURCE((uint32_t)p_config->source));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_POLARITY((uint32_t)p_config->polarity));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_MODE((uint32_t)p_config->mode));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_ConfigChannelTrigger(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_config->source,
                              ((uint32_t)p_config->mode | (uint32_t)p_config->polarity));

  return HAL_OK;
}

/**
  * @brief Reset the DMA channel direct transfer trigger configuration.
  * @param hdma               Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_OK            Reset direct transfer trigger configuration is successful
  */
hal_status_t HAL_DMA_ResetConfigDirectXferTrigger(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  LL_DMA_ConfigChannelTrigger(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_LPDMA1_TRIGGER_EXTI0,
                              ((uint32_t)HAL_DMA_TRIGGER_BLOCK_TRANSFER | (uint32_t)HAL_DMA_TRIGGER_POLARITY_MASKED));

  return HAL_OK;
}

/**
  * @brief Get the DMA channel direct transfer trigger configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_trigger_config_t configuration structure
  */
void HAL_DMA_GetConfigDirectXferTrigger(hal_dma_handle_t *hdma, hal_dma_trigger_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  p_config->mode     = (hal_dma_trigger_mode_t)LL_DMA_GetTriggerMode(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->polarity = (hal_dma_trigger_polarity_t)LL_DMA_GetTriggerPolarity(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->source   = (hal_dma_trigger_source_t)LL_DMA_GetHWTrigger(DMA_CHANNEL_GET_INSTANCE(hdma));
}

/**
  * @brief  Set the DMA channel direct transfer data handling configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_config          Pointer to @ref hal_dma_data_handling_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct or p_config pointer is
  *                           NULL
  * @retval HAL_OK            Direct transfer data handling is successfully configured
  */
hal_status_t HAL_DMA_SetConfigDirectXferDataHandling(hal_dma_handle_t *hdma,
                                                     const hal_dma_data_handling_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_TRUNC_PADD((uint32_t)p_config->trunc_padd));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_ConfigDataHandling(DMA_CHANNEL_GET_INSTANCE(hdma),
                            (uint32_t)p_config->trunc_padd);
  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel direct transfer data handling configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_OK            Reset direct transfer data handling configuration is successful
  */
hal_status_t HAL_DMA_ResetConfigDirectXferDataHandling(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Reset DMA channel data handling configuration */
  LL_DMA_ConfigDataHandling(DMA_CHANNEL_GET_INSTANCE(hdma),
                            (uint32_t)HAL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO);

  return HAL_OK;
}

/**
  * @brief Get the DMA channel direct transfer data handling configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_data_handling_config_t configuration structure
  */
void HAL_DMA_GetConfigDirectXferDataHandling(hal_dma_handle_t *hdma, hal_dma_data_handling_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  p_config->trunc_padd             =
    (hal_dma_dest_data_trunc_padd_t)LL_DMA_GetDataTruncPadd(DMA_CHANNEL_GET_INSTANCE(hdma));
}

/**
  * @brief Set the DMA channel privileged access level attribute.
  * @param instance     DMA channel instance
  * @param priv_attr    Element in @ref hal_dma_priv_attr_t enumeration
  * @retval HAL_ERROR   Operation failed, not in privileged execution mode.
  * @retval HAL_OK      Privileged attribute set successfully.
  */
hal_status_t HAL_DMA_SetPrivAttr(hal_dma_channel_t instance, hal_dma_priv_attr_t priv_attr)
{
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)instance)));
  ASSERT_DBG_PARAM(IS_DMA_PRIV_ATTR(priv_attr));

  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  LL_DMA_SetPrivAttr(LL_DMA_GET_INSTANCE(instance), LL_DMA_GET_CHANNEL_IDX(instance), (uint32_t)priv_attr);

  return HAL_OK;
}

/**
  * @brief Get the DMA channel privileged access attribute configuration.
  * @param instance        DMA channel instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref HAL_DMA_PRIV
  *         @arg @ref HAL_DMA_NPRIV
  */
hal_dma_priv_attr_t HAL_DMA_GetPrivAttr(hal_dma_channel_t instance)
{
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)instance)));

  return (hal_dma_priv_attr_t)LL_DMA_GetPrivAttr(LL_DMA_GET_INSTANCE(instance),
                                                 LL_DMA_GET_CHANNEL_IDX(instance));
}


/**
  * @brief  Lock the privileged access levels attribute for item(s).
  * @param instance     DMA channel instance
  * @retval HAL_ERROR   Operation failed, not in privileged execution mode.
  * @retval HAL_OK      Privilege attribute locked successfully
  */
hal_status_t HAL_DMA_LockAttr(hal_dma_channel_t instance)
{
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)instance)));

  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  LL_DMA_LockAttr(LL_DMA_GET_INSTANCE(instance), LL_DMA_GET_CHANNEL_IDX(instance));

  return HAL_OK;
}


/**
  * @brief  Check if the DMA channel privilege attribute is locked.
  * @param instance       DMA channel instance
  * @retval Privilege attribute lock status, element in @ref hal_dma_attr_lock_status_t enumeration
  */
hal_dma_attr_lock_status_t HAL_DMA_IsLockedAttr(hal_dma_channel_t instance)
{
  ASSERT_DBG_PARAM(IS_DMA_ALL_INSTANCE((DMA_Channel_TypeDef *)((uint32_t)instance)));

  return (hal_dma_attr_lock_status_t)LL_DMA_IsLockedAttr(LL_DMA_GET_INSTANCE(instance),
                                                         LL_DMA_GET_CHANNEL_IDX(instance));
}

/**
  * @brief  Set the DMA channel peripheral direct transfer configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_config          Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_config pointer is NULL
  * @retval HAL_OK            Peripheral direct transfer is successfully configured
  */
hal_status_t HAL_DMA_SetConfigPeriphDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_REQUEST((uint32_t)p_config->request));
  ASSERT_DBG_PARAM(IS_DMA_DIRECTION((uint32_t)p_config->direction));
  ASSERT_DBG_PARAM(IS_DMA_SRC_INC((uint32_t)p_config->src_inc));
  ASSERT_DBG_PARAM(IS_DMA_DEST_INC((uint32_t)p_config->dest_inc));
  ASSERT_DBG_PARAM(IS_DMA_SRC_DATA_WIDTH((uint32_t)p_config->src_data_width));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_WIDTH((uint32_t)p_config->dest_data_width));
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)p_config->priority));
  ASSERT_DBG_STATE(hdma->global_state, (uint32_t)HAL_DMA_STATE_INIT | (uint32_t)HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_SetConfigDirectXfer(hdma, p_config);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  hdma->xfer_mode = HAL_DMA_XFER_MODE_DIRECT;
#endif /* USE_HAL_DMA_LINKEDLIST */

  hdma->global_state = HAL_DMA_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get the DMA channel peripheral direct transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  */
void HAL_DMA_GetConfigPeriphDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  DMA_GetConfigDirectXfer(hdma, p_config);
}

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief  Set the DMA channel linked list transfer configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_config          Pointer to @ref hal_dma_linkedlist_xfer_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_config pointer is NULL
  * @retval HAL_OK            Linked list transfer is successfully configured
  */
hal_status_t HAL_DMA_SetConfigLinkedListXfer(hal_dma_handle_t *hdma,
                                             const hal_dma_linkedlist_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma     != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)p_config->priority));
  ASSERT_DBG_PARAM(IS_DMA_LINKEDLIST_XFER_EVENT_MODE((uint32_t)p_config->xfer_event_mode));
  ASSERT_DBG_STATE(hdma->global_state, (uint32_t)HAL_DMA_STATE_INIT | (uint32_t)HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_SetConfigLinkedListXfer(hdma, p_config);

  hdma->xfer_mode = HAL_DMA_XFER_MODE_LINKEDLIST_LINEAR;

  hdma->global_state = HAL_DMA_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get the DMA channel linked list transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_linkedlist_xfer_config_t configuration structure
  */
void HAL_DMA_GetConfigLinkedListXfer(hal_dma_handle_t *hdma, hal_dma_linkedlist_xfer_config_t *p_config)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  DMA_GetConfigLinkedListXfer(hdma, p_config);
}

/**
  * @brief  Set the DMA channel linked list transfer event mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  xfer_event_mode   Element in @ref hal_dma_linkedlist_xfer_event_mode_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Linked list transfer event mode is successfully configured
  */
hal_status_t HAL_DMA_SetLinkedListXferEventMode(hal_dma_handle_t *hdma,
                                                hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_LINKEDLIST_XFER_EVENT_MODE((uint32_t)xfer_event_mode));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetTransferEventMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)xfer_event_mode);

  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel linked list transfer event mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Reset linked list transfer event mode configuration is successful
  */
hal_status_t HAL_DMA_ResetLinkedListXferEventMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetTransferEventMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK);

  return HAL_OK;
}

/**
  * @brief  Get the DMA channel linked list transfer event mode configuration.
  * @param  hdma                                         Pointer to DMA channel handle
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK          Linked list transfer event block
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_REPEATED_BLOCK Linked list transfer event repeated block
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_NODE           Linked list transfer event node
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_Q              Linked list transfer event Q
  */
hal_dma_linkedlist_xfer_event_mode_t HAL_DMA_GetLinkedListXferEventMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  return ((hal_dma_linkedlist_xfer_event_mode_t)LL_DMA_GetTransferEventMode(DMA_CHANNEL_GET_INSTANCE(hdma)));
}

/**
  * @brief  Set the DMA channel linked list transfer priority configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  priority          Element in @ref hal_dma_priority_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Linked list transfer priority is successfully configured
  */
hal_status_t HAL_DMA_SetLinkedListXferPriority(hal_dma_handle_t *hdma, hal_dma_priority_t priority)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)priority));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)priority);

  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel linked list transfer priority configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Reset linked list transfer priority configuration is successful
  */
hal_status_t HAL_DMA_ResetLinkedListXferPriority(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_PRIORITY_LOW_WEIGHT_LOW);

  return HAL_OK;
}

/**
  * @brief  Get the DMA channel linked list transfer priority configuration.
  * @param  hdma                             Pointer to DMA channel handle
  * @retval HAL_DMA_PRIORITY_LOW_WEIGHT_LOW  DMA channel priority low and weight low
  * @retval HAL_DMA_PRIORITY_LOW_WEIGHT_MID  DMA channel priority low and weight mid
  * @retval HAL_DMA_PRIORITY_LOW_WEIGHT_HIGH DMA channel priority low and weight high
  * @retval HAL_DMA_PRIORITY_HIGH            DMA channel priority high
  */
hal_dma_priority_t HAL_DMA_GetLinkedListXferPriority(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  return ((hal_dma_priority_t)LL_DMA_GetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma)));
}

/**
  * @brief  Set the DMA channel linked list transfer execution mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  exec_mode         Element in @ref hal_dma_linkedlist_execution_mode_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Linked list transfer execution mode is successfully configured
  */
hal_status_t HAL_DMA_SetLinkedListXferExecutionMode(hal_dma_handle_t *hdma,
                                                    hal_dma_linkedlist_execution_mode_t exec_mode)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_LINKEDLIST_EXEC_MODE((uint32_t)exec_mode));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetLinkStepMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)exec_mode);

  return HAL_OK;
}

/**
  * @brief  Reset the DMA channel linked list transfer execution mode configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is direct
  * @retval HAL_OK            Reset linked list transfer execution mode configuration is successful
  */
hal_status_t HAL_DMA_ResetLinkedListXferExecutionMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_DMA_SetLinkStepMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_LINKEDLIST_EXECUTION_Q);

  return HAL_OK;
}

/**
  * @brief  Get the DMA channel linked list transfer execution mode configuration.
  * @param  hdma                              Pointer to DMA channel handle
  * @retval HAL_DMA_LINKEDLIST_EXECUTION_Q    DMA channel is executed for the full linked list
  * @retval HAL_DMA_LINKEDLIST_NODE_EXECUTION DMA channel is executed once for the current linked list
  */
hal_dma_linkedlist_execution_mode_t HAL_DMA_GetLinkedListXferExecutionMode(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  return ((hal_dma_linkedlist_execution_mode_t)LL_DMA_GetLinkStepMode(DMA_CHANNEL_GET_INSTANCE(hdma)));
}

/**
  * @brief  Set the DMA channel peripheral linked list circular transfer configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_node            Pointer to @ref hal_dma_node_t structure
  * @param  p_node_config     Pointer to @ref hal_dma_direct_xfer_config_t structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when node or node_config pointer is NULL
  * @retval HAL_OK            Peripheral linked list circular transfer is successfully configured
  */
hal_status_t HAL_DMA_SetConfigPeriphLinkedListCircularXfer(hal_dma_handle_t *hdma, hal_dma_node_t *p_node,
                                                           const hal_dma_direct_xfer_config_t *p_node_config)
{
  hal_dma_node_type_t              node_type;
  hal_dma_linkedlist_xfer_config_t p_config;

  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_node_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_REQUEST((uint32_t)p_node_config->request));
  ASSERT_DBG_PARAM(IS_DMA_DIRECTION((uint32_t)p_node_config->direction));
  ASSERT_DBG_PARAM(IS_DMA_SRC_INC((uint32_t)p_node_config->src_inc));
  ASSERT_DBG_PARAM(IS_DMA_DEST_INC((uint32_t)p_node_config->dest_inc));
  ASSERT_DBG_PARAM(IS_DMA_SRC_DATA_WIDTH((uint32_t)p_node_config->src_data_width));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_WIDTH((uint32_t)p_node_config->dest_data_width));
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)p_node_config->priority));
  ASSERT_DBG_STATE(hdma->global_state, (uint32_t)HAL_DMA_STATE_INIT | (uint32_t)HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_node == NULL) || (p_node_config == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  node_type = DMA_GET_NODE_TYPE(hdma->instance);

  hdma->p_head_node = p_node;

  /* Set DMA channel linked list transfer configuration */
  p_config.priority   = p_node_config->priority;
  p_config.xfer_event_mode = HAL_DMA_LINKEDLIST_XFER_EVENT_Q;
  DMA_SetConfigLinkedListXfer(hdma, &p_config);

  /* Fill linked list node for periph circular transfer */
  DMA_FillNodeDirectXfer(p_node, p_node_config, node_type, HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK);

  /* Set circular link for DMA node */
  p_node->regs[node_type] = (((uint32_t)p_node & DMA_CLLR_LA) | LL_DMA_UPDATE_ALL);

  hdma->xfer_mode = HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR;

  hdma->global_state = HAL_DMA_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get the DMA channel peripheral linked list circular transfer configuration.
  * @param hdma          Pointer to DMA channel handle
  * @param p_node        Pointer to @ref hal_dma_node_t structure
  * @param p_node_config Pointer to @ref hal_dma_direct_xfer_config_t structure
  */
void HAL_DMA_GetConfigPeriphLinkedListCircularXfer(hal_dma_handle_t *hdma, hal_dma_node_t *p_node,
                                                   hal_dma_direct_xfer_config_t *p_node_config)
{
  hal_dma_linkedlist_xfer_config_t p_config;
  hal_dma_node_type_t              node_type;

  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_node_config != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  node_type = DMA_GET_NODE_TYPE(hdma->instance);

  /* Get the DMA channel configuration in linked list mode */
  DMA_GetConfigLinkedListXfer(hdma, &p_config);

  /* Get the linked list node for direct transfer */
  HAL_DMA_GetNodeDirectXfer(p_node, p_node_config, &node_type);

  /* Get the priority level for the linked list node */
  p_node_config->priority = p_config.priority;
}
#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @}
  */

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/** @addtogroup DMA_Exported_Functions_Group3
  * @{
This subsection provides a set of functions to configure the DMA channel peripheral:

  Node configuration

- Call the function HAL_DMA_FillNodeConfig() to fill the node according to configured parameter within
  hal_dma_node_config_t structure

- Call the function HAL_DMA_GetNodeConfig() to get the current node configuration

  Direct transfer node configuration

- Call the function HAL_DMA_FillNodeDirectXfer() to fill the node direct transfer according to configured parameter
  within hal_dma_direct_xfer_config_t structure

- Call the function HAL_DMA_GetNodeDirectXfer() to get the current node direct transfer configuration

  Hardware request mode node configuration

- Call the function HAL_DMA_FillNodeHardwareRequestMode() to fill the node hardware request mode according to selected
  request mode parameter

- Call the function HAL_DMA_GetNodeHardwareRequestMode() to get the current node hardware request mode selection

  Flow control mode node configuration

- Call the function HAL_DMA_FillNodeFlowControlMode() to fill the node flow control mode according to selected flow
  control mode parameter

- Call the function HAL_DMA_GetNodeFlowControlMode() to get the current node flow control mode selection

  Transfer event mode node configuration

- Call the function HAL_DMA_FillNodeXferEventMode() to fill the node transfer event mode according to selected
  transfer event mode parameter

- Call the function HAL_DMA_GetNodeXferEventMode() to get the current node transfer event mode configuration

  Trigger node configuration

- Call the function HAL_DMA_FillNodeTrigger() to fill the node trigger according to configured trigger parameters

- Call the function HAL_DMA_GetNodeTrigger() to get the current node trigger configuration

  Data handling node configuration

- Call the function HAL_DMA_FillNodeDataHandling() to fill the node data handling according to configured data handling
  parameters

- Call the function HAL_DMA_GetNodeDataHandling() to get the current node data handling configuration

  Data node configuration

- Call the function HAL_DMA_FillNodeData() to fill the node data according to configured data parameters

- Call the function HAL_DMA_GetNodeData() to get the current node data configuration

  Conversion Q nodes

- Call the function HAL_DMA_ConvertQNodesToDynamic() to Convert linked list queue associated to the handle to dynamic
  format

- Call the function HAL_DMA_ConvertQNodesToStatic() to Convert linked list queue associated to the handle to static
  format
  */

/**
  * @brief  Fill node configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  p_conf            Pointer to @ref hal_dma_node_config_t configuration structure
  * @param  node_type         Element in @ref hal_dma_node_type_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node or p_conf pointer is NULL
  * @retval HAL_OK            Fill node is successfully configured
  */
hal_status_t HAL_DMA_FillNodeConfig(hal_dma_node_t *p_node, const hal_dma_node_config_t *p_conf,
                                    hal_dma_node_type_t node_type)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_conf != NULL);
  ASSERT_DBG_PARAM(IS_DMA_REQUEST((uint32_t)p_conf->xfer.request));
  ASSERT_DBG_PARAM(IS_DMA_DIRECTION((uint32_t)p_conf->xfer.direction));
  ASSERT_DBG_PARAM(IS_DMA_SRC_INC((uint32_t)p_conf->xfer.src_inc));
  ASSERT_DBG_PARAM(IS_DMA_DEST_INC((uint32_t)p_conf->xfer.dest_inc));
  ASSERT_DBG_PARAM(IS_DMA_SRC_DATA_WIDTH((uint32_t)p_conf->xfer.src_data_width));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_WIDTH((uint32_t)p_conf->xfer.dest_data_width));
  ASSERT_DBG_PARAM(IS_DMA_HARDWARE_REQUEST_MODE((uint32_t)p_conf->hw_request_mode));
  ASSERT_DBG_PARAM(IS_DMA_FLOW_CONTROL_MODE((uint32_t)p_conf->flow_ctrl_mode));
  ASSERT_DBG_PARAM(IS_DMA_LINKEDLIST_XFER_EVENT_MODE((uint32_t)p_conf->xfer_event_mode));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_SOURCE((uint32_t)p_conf->trigger.source));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_POLARITY((uint32_t)p_conf->trigger.polarity));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_MODE((uint32_t)p_conf->trigger.mode));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_TRUNC_PADD((uint32_t)p_conf->data_handling.trunc_padd));
  ASSERT_DBG_PARAM(p_conf->size_byte <= 0xFFFFU);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_node == NULL) || (p_conf == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_FillNodeConfig(p_node, p_conf, node_type);

  return HAL_OK;
}

/**
  * @brief Get the configuration of node.
  * @param p_node      Pointer to @ref hal_dma_node_t node structure
  * @param p_conf      Pointer to @ref hal_dma_node_config_t configuration structure
  * @param p_node_type Element in @ref hal_dma_node_type_t enumeration
  */
void HAL_DMA_GetNodeConfig(const hal_dma_node_t *p_node, hal_dma_node_config_t *p_conf,
                           hal_dma_node_type_t *p_node_type)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_conf != NULL);

  DMA_GetConfigNode(p_node, p_conf, p_node_type);
}

/**
  * @brief  Fill node direct transfer configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  p_config          Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  * @param  node_type         Element in @ref hal_dma_node_type_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node or p_config pointer is NULL
  * @retval HAL_OK            Fill node direct transfer is successfully configured
  */
hal_status_t HAL_DMA_FillNodeDirectXfer(hal_dma_node_t *p_node, const hal_dma_direct_xfer_config_t *p_config,
                                        hal_dma_node_type_t node_type)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_REQUEST((uint32_t)p_config->request));
  ASSERT_DBG_PARAM(IS_DMA_DIRECTION((uint32_t)p_config->direction));
  ASSERT_DBG_PARAM(IS_DMA_SRC_INC((uint32_t)p_config->src_inc));
  ASSERT_DBG_PARAM(IS_DMA_DEST_INC((uint32_t)p_config->dest_inc));
  ASSERT_DBG_PARAM(IS_DMA_SRC_DATA_WIDTH((uint32_t)p_config->src_data_width));
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_WIDTH((uint32_t)p_config->dest_data_width));
  ASSERT_DBG_PARAM(IS_DMA_PRIORITY((uint32_t)p_config->priority));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_node == NULL) || (p_config == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_FillNodeDirectXfer(p_node, p_config, node_type, HAL_DMA_LINKEDLIST_XFER_EVENT_Q);

  return HAL_OK;
}

/**
  * @brief Get the configuration of node direct transfer.
  * @param p_node      Pointer to @ref hal_dma_node_t node structure
  * @param p_config    Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  * @param p_node_type Element in @ref hal_dma_node_type_t enumeration
  */
void HAL_DMA_GetNodeDirectXfer(const hal_dma_node_t *p_node, hal_dma_direct_xfer_config_t *p_config,
                               hal_dma_node_type_t *p_node_type)
{
  hal_dma_node_config_t p_conf;

  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  DMA_GetConfigNode(p_node, &p_conf, p_node_type);

  p_config->request         = p_conf.xfer.request;
  p_config->direction       = p_conf.xfer.direction;
  p_config->src_inc         = p_conf.xfer.src_inc;
  p_config->dest_inc        = p_conf.xfer.dest_inc;
  p_config->src_data_width  = p_conf.xfer.src_data_width;
  p_config->dest_data_width = p_conf.xfer.dest_data_width;
}

/**
  * @brief  Fill node hardware request mode configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  hw_request_mode   Element in @ref hal_dma_hardware_request_mode_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node pointer is NULL
  * @retval HAL_OK            Fill node hardware request mode is successfully configured
  */
hal_status_t HAL_DMA_FillNodeHardwareRequestMode(hal_dma_node_t *p_node,
                                                 hal_dma_hardware_request_mode_t hw_request_mode)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(IS_DMA_HARDWARE_REQUEST_MODE((uint32_t)hw_request_mode));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_MODIFY_REG(p_node->regs[DMA_NODE_CTR2_REG_OFFSET], DMA_CTR2_BREQ, (uint32_t)hw_request_mode);

  return HAL_OK;
}

/**
  * @brief  Get the configuration of node hardware request mode.
  * @param  p_node                         Pointer to @ref hal_dma_node_t node structure
  * @retval HAL_DMA_HARDWARE_REQUEST_BURST DMA channel hardware request mode is burst
  * @retval HAL_DMA_HARDWARE_REQUEST_BLOCK DMA channel hardware request mode is block
  */
hal_dma_hardware_request_mode_t HAL_DMA_GetNodeHardwareRequestMode(const hal_dma_node_t *p_node)
{
  ASSERT_DBG_PARAM(p_node != NULL);

  return (hal_dma_hardware_request_mode_t)((uint32_t)(p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_BREQ));
}
/**
  * @brief  Fill node flow control mode configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  flow_control_mode Element in @ref hal_dma_flow_control_mode_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node pointer is NULL
  * @retval HAL_OK            Fill node flow control request is successfully configured
  */
hal_status_t HAL_DMA_FillNodeFlowControlMode(hal_dma_node_t *p_node, hal_dma_flow_control_mode_t flow_control_mode)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(IS_DMA_FLOW_CONTROL_MODE((uint32_t)flow_control_mode));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_MODIFY_REG(p_node->regs[DMA_NODE_CTR2_REG_OFFSET], DMA_CTR2_PFREQ, (uint32_t)flow_control_mode);

  return HAL_OK;
}

/**
  * @brief  Get the configuration of node flow control mode .
  * @param  p_node                      Pointer to @ref hal_dma_node_t node structure
  * @retval HAL_DMA_FLOW_CONTROL_DMA    DMA request DMA channel flow control mode
  * @retval HAL_DMA_FLOW_CONTROL_PERIPH DMA request peripheral flow control mode
  */
hal_dma_flow_control_mode_t HAL_DMA_GetNodeFlowControlMode(const hal_dma_node_t *p_node)
{
  ASSERT_DBG_PARAM(p_node != NULL);

  return (hal_dma_flow_control_mode_t)((uint32_t)(p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_PFREQ));
}

/**
  * @brief  Fill node transfer event mode configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  xfer_event_mode   Element in @ref hal_dma_linkedlist_xfer_event_mode_t enumeration
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node is NULL
  * @retval HAL_OK            Fill node transfer event mode is successfully configured
  */
hal_status_t HAL_DMA_FillNodeXferEventMode(hal_dma_node_t *p_node, hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(IS_DMA_LINKEDLIST_XFER_EVENT_MODE((uint32_t)xfer_event_mode));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_MODIFY_REG(p_node->regs[DMA_NODE_CTR2_REG_OFFSET], DMA_CTR2_TCEM, (uint32_t)xfer_event_mode);

  return HAL_OK;
}

/**
  * @brief Get the configuration of node transfer event mode.
  * @param p_node  Pointer to @ref hal_dma_node_t node structure
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_BLOCK          DMA channel transfer event mode is at block level.
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_NODE           DMA channel transfer event mode is at each linked list item.
  * @retval HAL_DMA_LINKEDLIST_XFER_EVENT_Q              DMA channel transfer event mode is at last linked list item.
  */
hal_dma_linkedlist_xfer_event_mode_t HAL_DMA_GetNodeXferEventMode(const hal_dma_node_t *p_node)
{
  ASSERT_DBG_PARAM(p_node != NULL);

  return (hal_dma_linkedlist_xfer_event_mode_t)((uint32_t)(p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TCEM));
}

/**
  * @brief  Fill node trigger configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  p_config          Pointer to @ref hal_dma_trigger_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node or p_config pointer is NULL
  * @retval HAL_OK            Fill node trigger is successfully configured
  */
hal_status_t HAL_DMA_FillNodeTrigger(hal_dma_node_t *p_node, const hal_dma_trigger_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_SOURCE((uint32_t)p_config->source));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_POLARITY((uint32_t)p_config->polarity));
  ASSERT_DBG_PARAM(IS_DMA_TRIGGER_MODE((uint32_t)p_config->mode));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_node == NULL) || (p_config == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_MODIFY_REG(p_node->regs[DMA_NODE_CTR2_REG_OFFSET], (DMA_CTR2_TRIGM | DMA_CTR2_TRIGPOL | DMA_CTR2_TRIGSEL),
                   (uint32_t)p_config->mode | (uint32_t)p_config->polarity |
                   (((uint32_t)p_config->source << DMA_CTR2_TRIGSEL_Pos) & DMA_CTR2_TRIGSEL));

  return HAL_OK;
}

/**
  * @brief Get the configuration of node trigger.
  * @param p_node   Pointer to @ref hal_dma_node_t node structure
  * @param p_config Pointer to @ref hal_dma_trigger_config_t configuration structure
  */
void HAL_DMA_GetNodeTrigger(const hal_dma_node_t *p_node, hal_dma_trigger_config_t *p_config)
{
  uint32_t dummy;

  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGM;
  p_config->mode     = (hal_dma_trigger_mode_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGPOL;
  p_config->polarity = (hal_dma_trigger_polarity_t)dummy;
  dummy = (p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGSEL) >> DMA_CTR2_TRIGSEL_Pos;
  p_config->source   = (hal_dma_trigger_source_t)dummy;
}

/**
  * @brief  Fill node data handling configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  p_config          Pointer to @ref hal_dma_data_handling_config_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node or p_config pointer is NULL
  * @retval HAL_OK            Fill node data handling is successfully configured
  */
hal_status_t HAL_DMA_FillNodeDataHandling(hal_dma_node_t *p_node, const hal_dma_data_handling_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DMA_DEST_DATA_TRUNC_PADD((uint32_t)p_config->trunc_padd));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_node == NULL) || (p_config == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_MODIFY_REG(p_node->regs[DMA_NODE_CTR1_REG_OFFSET],
                   DMA_CTR1_PAM,
                   (uint32_t)p_config->trunc_padd);

  return HAL_OK;
}

/**
  * @brief Get the configuration of node data handling.
  * @param p_node   Pointer to @ref hal_dma_node_t node structure
  * @param p_config Pointer to @ref hal_dma_data_handling_config_t configuration structure
  */
void HAL_DMA_GetNodeDataHandling(const hal_dma_node_t *p_node, hal_dma_data_handling_config_t *p_config)
{
  uint32_t dummy;

  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);

  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_PAM_0;
  p_config->trunc_padd = (hal_dma_dest_data_trunc_padd_t)dummy;
}

/**
  * @brief  Fill node data configuration.
  * @param  p_node            Pointer to @ref hal_dma_node_t node structure
  * @param  src_addr          Source address
  * @param  dest_addr         Destination address
  * @param  size_byte         Size in byte
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_node pointer is NULL
  * @retval HAL_OK            Fill node data is successfully configured
  */
hal_status_t HAL_DMA_FillNodeData(hal_dma_node_t *p_node, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM((size_byte > 0U) && (size_byte <= 0xFFFFU));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_UpdateDataNode(p_node, src_addr, dest_addr, size_byte);

  return HAL_OK;
}

/**
  * @brief Get the configuration of node data.
  * @param p_node      Pointer to @ref hal_dma_node_t node structure
  * @param p_src_addr  Source address
  * @param p_dest_addr Destination address
  * @param p_size_byte Size in byte
  */
void HAL_DMA_GetNodeData(const hal_dma_node_t *p_node, uint32_t *p_src_addr, uint32_t *p_dest_addr,
                         uint32_t *p_size_byte)
{
  uint32_t *dummy;

  ASSERT_DBG_PARAM(p_node != NULL);
  ASSERT_DBG_PARAM(p_src_addr != NULL);
  ASSERT_DBG_PARAM(p_dest_addr != NULL);
  ASSERT_DBG_PARAM(p_size_byte != NULL);

  *p_size_byte = p_node->regs[DMA_NODE_CBR1_REG_OFFSET] & DMA_CBR1_BNDT;
  dummy  = p_src_addr;
  *dummy = p_node->regs[DMA_NODE_CSAR_REG_OFFSET];
  dummy  = p_dest_addr;
  *dummy = p_node->regs[DMA_NODE_CDAR_REG_OFFSET];
}

/**
  * @brief  Convert linked list queue associated to the handle to dynamic format.
  * @param  p_q               Pointer to hal_q_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_q pointer is NULL
  * @retval HAL_OK            Q nodes to dynamic conversion is successfully configured
  */
hal_status_t HAL_DMA_ConvertQNodesToDynamic(hal_q_t *p_q)
{
  ASSERT_DBG_PARAM(p_q != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_ConvertQNodesToDynamic(p_q);

  return HAL_OK;
}

/**
  * @brief  Convert linked list queue associated to the handle to static format.
  * @param  p_q               Pointer to hal_q_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_q pointer is NULL
  * @retval HAL_OK            Q nodes to static conversion is successfully configured
  */
hal_status_t HAL_DMA_ConvertQNodesToStatic(hal_q_t *p_q)
{
  ASSERT_DBG_PARAM(p_q != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DMA_ConvertQNodesToStatic(p_q);

  return HAL_OK;
}
/**
  * @}
  */
#endif /* USE_HAL_DMA_LINKEDLIST */

/** @addtogroup DMA_Exported_Functions_Group4
  * @{

This subsection provides a set of functions to configure the DMA channel peripheral:

- Call the function HAL_DMA_StartDirectXfer() to start direct DMA channel transfer in silent mode

- Call the function HAL_DMA_StartDirectXfer_IT() to start direct DMA channel transfer in interrupt mode
  with default optional interrupts configuration

- Call the function HAL_DMA_StartDirectXfer_IT_Opt() to start direct DMA channel transfer in interrupt mode
  with customized optional interrupts configuration

- Call the HAL_DMA_StartPeriphXfer_IT_Opt() function to start the DMA channel peripheral transfer with filtering
  optional interrupts.\n
  Note: This function is intended exclusively for internal use by HAL PPP drivers for transfers over DMA.

- Call the function HAL_DMA_StartLinkedListXfer() to start linked list DMA channel transfer in silent mode

- Call the function HAL_DMA_StartLinkedListXfer_IT() to start linked list DMA channel transfer in interrupt mode
  with default optional interrupts configuration

- Call the function HAL_DMA_StartLinkedListXfer_IT_Opt() to start linked list DMA channel transfer in interrupt mode
  with customized optional interrupts configuration

- Call the function HAL_DMA_Abort() to abort any ongoing DMA channel transfer in blocking mode

- Call the function HAL_DMA_Abort_IT() to abort any ongoing DMA channel transfer in interrupt mode

- Call the function HAL_DMA_Suspend() to suspend any ongoing DMA channel transfer in blocking mode

- Call the function HAL_DMA_Suspend_IT() to suspend any ongoing DMA channel transfer in interrupt mode

- Call the function HAL_DMA_Resume() to resume any suspended DMA channel transfer immediately.

- Call the function HAL_DMA_PollForXfer() to poll on any finite DMA channel transfer level selected through
  hal_dma_xfer_level_t

- Call the function HAL_DMA_IRQHandler() to handle any DMA channel interrupt. This API must be executed in handler mode
  */

/**
  * @brief  Start the DMA channel direct transfer in silent mode.
  * @param  hdma              Pointer to DMA channel handle
  * @param  src_addr          Source address
  * @param  dest_addr         Destination address
  * @param  size_byte         Size in byte
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Silent direct transfer is successfully started
  */
hal_status_t HAL_DMA_StartDirectXfer(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM((size_byte > 0U) && (size_byte <= 0xFFFFU));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  DMA_StartDirectXfer(hdma, src_addr, dest_addr, size_byte, HAL_DMA_OPT_IT_SILENT);

  return HAL_OK;
}

/**
  * @brief  Start the DMA channel direct transfer in interrupt mode with default optional interrupts configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  src_addr          Source address
  * @param  dest_addr         Destination address
  * @param  size_byte         Size in byte
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Interrupt direct transfer is successfully started
  */
hal_status_t HAL_DMA_StartDirectXfer_IT(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                        uint32_t size_byte)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM((size_byte > 0U) && (size_byte <= 0xFFFFU));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  DMA_StartDirectXfer(hdma, src_addr, dest_addr, size_byte, HAL_DMA_OPT_IT_DEFAULT);

  return HAL_OK;
}

/**
  * @brief  Start the DMA channel direct transfer in interrupt mode with customized optional interrupts configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  src_addr          Source address
  * @param  dest_addr         Destination address
  * @param  size_byte         Size in byte
  * @param  interrupts        DMA optional interrupts to be enabled.
  *                           This parameter can be one of @ref DMA_Optional_Interrupt group.
  * @retval HAL_INVALID_PARAM Invalid parameter return when transfer mode parameter is not direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Interrupt direct transfer is successfully started
  */
hal_status_t HAL_DMA_StartDirectXfer_IT_Opt(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                            uint32_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM((size_byte > 0U) && (size_byte <= 0xFFFFU));
  ASSERT_DBG_PARAM(IS_DMA_OPT_IT(interrupts));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode != HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  DMA_StartDirectXfer(hdma, src_addr, dest_addr, size_byte, interrupts);

  return HAL_OK;
}

/**
  * @brief  Start the DMA channel peripheral transfer.
  * @param  hdma       Pointer to DMA channel handle
  * @param  src_addr   Source address
  * @param  dest_addr  Destination address
  * @param  size_byte  Size in byte
  * @param  interrupts DMA optional interrupts to be enabled.
  *                    This parameter can be one of @ref DMA_Optional_Interrupt group.
  * @retval HAL_ERROR  Transfer mode parameter is linked list linear
  * @retval HAL_BUSY   DMA channel state is active when calling this API
  * @retval HAL_OK     Peripheral transfer is successfully started
  * @note   This function is intended exclusively for internal use by HAL PPP drivers for transfers over DMA.
  */
hal_status_t HAL_DMA_StartPeriphXfer_IT_Opt(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr,
                                            uint32_t size_byte, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM((size_byte > 0U) && (size_byte <= 0xFFFFU));
  ASSERT_DBG_PARAM(IS_DMA_OPT_IT(interrupts));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* Linear linked list mode is activated */
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_LINEAR)
  {
    hdma->global_state = HAL_DMA_STATE_IDLE;

    return HAL_ERROR;
  }
  /* Circular linked list mode is activated */
  else if (hdma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    DMA_UpdateDataNode(hdma->p_head_node, src_addr, dest_addr, size_byte);
    DMA_StartLinkedListXfer(hdma, hdma->p_head_node, interrupts);
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    DMA_StartDirectXfer(hdma, src_addr, dest_addr, size_byte, interrupts);
  }

  return HAL_OK;
}

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief  Start the DMA channel linked list transfer in silent mode.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_q               Pointer to hal_q_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_q pointer is NULL or transfer mode parameter is direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Silent linked list transfer is successfully started
  */
hal_status_t HAL_DMA_StartLinkedListXfer(hal_dma_handle_t *hdma, const hal_q_t *p_q)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  if (p_q->p_first_circular_node != NULL)
  {
    hdma->xfer_mode = HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR;
  }

  DMA_StartLinkedListXfer(hdma, p_q->p_head_node, HAL_DMA_OPT_IT_SILENT);

  return HAL_OK;
}

/**
  * @brief  Start the DMA channel linked list transfer in interrupt mode with default optional interrupts configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_q               Pointer to hal_q_t configuration structure
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_q pointer is NULL or transfer mode parameter is direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Interrupt linked list transfer is successfully started
  */
hal_status_t HAL_DMA_StartLinkedListXfer_IT(hal_dma_handle_t *hdma, const hal_q_t *p_q)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  if (p_q->p_first_circular_node != NULL)
  {
    hdma->xfer_mode = HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR;
  }

  DMA_StartLinkedListXfer(hdma, p_q->p_head_node, HAL_DMA_OPT_IT_DEFAULT);

  return HAL_OK;
}

/**
  * @brief  Start the DMA channel linked list transfer in interrupt mode with customized optional interrupts
  *         configuration.
  * @param  hdma              Pointer to DMA channel handle
  * @param  p_q               Pointer to hal_q_t configuration structure
  * @param  interrupts        DMA optional interrupts to be enabled.
  *                           This parameter can be one of @ref DMA_Optional_Interrupt group.
  * @retval HAL_INVALID_PARAM Invalid parameter return when p_q pointer is NULL or transfer mode parameter is direct
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  * @retval HAL_OK            Interrupt linked list transfer is successfully started
  */
hal_status_t HAL_DMA_StartLinkedListXfer_IT_Opt(hal_dma_handle_t *hdma, const hal_q_t *p_q, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(IS_DMA_OPT_IT(interrupts));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_DIRECT)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_IDLE, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  hdma->last_error_codes = HAL_DMA_ERROR_NONE;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  if (p_q->p_first_circular_node != NULL)
  {
    hdma->xfer_mode = HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR;
  }

  DMA_StartLinkedListXfer(hdma, p_q->p_head_node, interrupts);

  return HAL_OK;
}
#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief  Abort any ongoing DMA channel transfer in blocking mode.
  * @param  hdma      Pointer to DMA channel handle
  * @retval HAL_ERROR DMA channel not aborted
  * @retval HAL_OK    Transfer in blocking mode is successfully aborted
  */
hal_status_t HAL_DMA_Abort(hal_dma_handle_t *hdma)
{
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, ((uint32_t)HAL_DMA_STATE_IDLE | (uint32_t)HAL_DMA_STATE_ACTIVE));

  if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
  {
    hdma->global_state = HAL_DMA_STATE_ABORT;

    LL_DMA_SuspendChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

    tickstart = HAL_GetTick();
    while (LL_DMA_IsActiveFlag_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
    {
      if ((HAL_GetTick() - tickstart) > DMA_SUSPEND_TIMEOUT)
      {
        if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) != 0U)
        {
          LL_DMA_ResetChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

          hdma->global_state = HAL_DMA_STATE_IDLE;
        }

        /* No state change, stay in ABORT state */
        return HAL_ERROR;
      }
    }

    LL_DMA_ResetChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

    LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

    hdma->global_state = HAL_DMA_STATE_IDLE;
  }
  else
  {
    /* The channel was not transmitting upon abort request */
    /* Global state: transmission is already completed */
    hdma->global_state = HAL_DMA_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Abort any ongoing DMA channel transfer in interrupt mode.
  * @param  hdma      Pointer to DMA channel handle
  * @retval HAL_ERROR DMA channel not aborted
  * @retval HAL_OK    Transfer in interrupt mode is successfully aborted
  */
hal_status_t HAL_DMA_Abort_IT(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, (uint32_t)HAL_DMA_STATE_IDLE | (uint32_t)HAL_DMA_STATE_ACTIVE);

  if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
  {
    hdma->global_state = HAL_DMA_STATE_ABORT;

    LL_DMA_EnableIT_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma));

    LL_DMA_SuspendChannel(DMA_CHANNEL_GET_INSTANCE(hdma));
  }
  else
  {
    /* The channel was not transmitting upon abort request */
    /* Global state: transmission is already completed */
    hdma->global_state = HAL_DMA_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Suspend any ongoing DMA channel transfer in blocking mode.
  * @param  hdma      Pointer to DMA channel handle
  * @retval HAL_ERROR DMA channel not suspended
  * @retval HAL_OK    Transfer in blocking mode is successfully suspended
  */
hal_status_t HAL_DMA_Suspend(hal_dma_handle_t *hdma)
{
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_ACTIVE);

  if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
  {
    hdma->global_state = HAL_DMA_STATE_SUSPEND;

    LL_DMA_SuspendChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

    tickstart = HAL_GetTick();
    while (LL_DMA_IsActiveFlag_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
    {
      if ((HAL_GetTick() - tickstart) > DMA_SUSPEND_TIMEOUT)
      {
        return HAL_ERROR;
      }
    }
  }
  else
  {
    /* The channel was not transmitting upon suspend request */
    hdma->global_state = HAL_DMA_STATE_IDLE;

    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Suspend any ongoing DMA channel transfer in interrupt mode.
  * @param  hdma      Pointer to DMA channel handle
  * @retval HAL_ERROR DMA channel not suspended
  * @retval HAL_OK    Transfer in interrupt mode is successfully suspended
  */
hal_status_t HAL_DMA_Suspend_IT(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_ACTIVE);

  if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) == 0U)
  {
    hdma->global_state = HAL_DMA_STATE_SUSPEND;

    LL_DMA_EnableIT_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma));

    LL_DMA_SuspendChannel(DMA_CHANNEL_GET_INSTANCE(hdma));
  }
  else
  {
    /* The channel was not transmitting upon suspend request */
    hdma->global_state = HAL_DMA_STATE_IDLE;

    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Resume any suspended DMA channel transfer immediately.
  * @param  hdma     Pointer to DMA channel handle
  * @retval HAL_BUSY DMA channel state is active when calling this API
  * @retval HAL_OK   Transfer is successfully resumed
  */
hal_status_t HAL_DMA_Resume(hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_SUSPEND);

  HAL_CHECK_UPDATE_STATE(hdma, global_state, HAL_DMA_STATE_SUSPEND, HAL_DMA_STATE_ACTIVE);

  LL_DMA_ResumeChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

  return HAL_OK;
}

/**
  * @brief  Polling for transfer status for finite DMA channel silent transfers.
  * @param  hdma         Pointer to DMA channel handle
  * @param  xfer_level   DMA channel transfer level
  * @param  timeout_ms   User timeout in milliseconds
  * @retval HAL_TIMEOUT  User timeout
  * @retval HAL_ERROR    DMA channel error
  * @retval HAL_OK       Polling for transfer is successfully configured
  */
hal_status_t HAL_DMA_PollForXfer(hal_dma_handle_t *hdma, hal_dma_xfer_level_t xfer_level, uint32_t timeout_ms)
{
  uint32_t tickstart;
  uint32_t tmp_csr;

  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(IS_DMA_XFER_LEVEL((uint32_t)xfer_level));
  ASSERT_DBG_STATE(hdma->global_state, HAL_DMA_STATE_ACTIVE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
#endif /* USE_HAL_DMA_LINKEDLIST */

  tmp_csr = LL_DMA_READ_REG((DMA_CHANNEL_GET_INSTANCE(hdma)), CSR);

  if ((tmp_csr & LL_DMA_FLAG_TO) != 0U)
  {
#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
    hdma->last_error_codes |= HAL_DMA_ERROR_TO;
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

    LL_DMA_ClearFlag_TO(DMA_CHANNEL_GET_INSTANCE(hdma));
  }

  if ((tmp_csr & HAL_DMA_FLAG_ERROR) != 0U)
  {
#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
    /* Check the data transfer error flag */
    if ((tmp_csr & LL_DMA_FLAG_DTE) != 0U)
    {
      hdma->last_error_codes |= HAL_DMA_ERROR_DTE;
    }

    /* Check the user setting error flag */
    if ((tmp_csr & LL_DMA_FLAG_USE) != 0U)
    {
      hdma->last_error_codes |= HAL_DMA_ERROR_USE;
    }

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    /* Check the update link error flag */
    if ((tmp_csr & LL_DMA_FLAG_ULE) != 0U)
    {
      hdma->last_error_codes |= HAL_DMA_ERROR_ULE;
    }
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

    LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

    LL_DMA_ResetChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

    hdma->global_state = HAL_DMA_STATE_IDLE;

    return HAL_ERROR;
  }

  /* Wait for transfer level */
  tickstart = HAL_GetTick();
  while ((LL_DMA_READ_REG((DMA_CHANNEL_GET_INSTANCE(hdma)), CSR) & (uint32_t)xfer_level) == 0U)
  {
    /* Check for the timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        if ((LL_DMA_READ_REG((DMA_CHANNEL_GET_INSTANCE(hdma)), CSR) & (uint32_t)xfer_level) == 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }

  /* Clear transfer level flags */
  if (xfer_level == HAL_DMA_XFER_HALF_COMPLETE)
  {
    LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_HT);
  }
  else
  {
    LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), (LL_DMA_FLAG_TC | LL_DMA_FLAG_HT));
  }

  hdma->global_state = HAL_DMA_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Handle any DMA channel interrupt.
  * @param hdma Pointer to DMA channel handle
  */
void HAL_DMA_IRQHandler(hal_dma_handle_t *hdma)
{
  DMA_TypeDef *instance;
  uint32_t    flags;
  uint32_t    its;
  uint32_t    channel;

  ASSERT_DBG_PARAM(hdma != NULL);

  instance = LL_DMA_GET_INSTANCE(hdma->instance);
  channel  = LL_DMA_GET_CHANNEL_IDX(hdma->instance);
  its      = LL_DMA_READ_REG((DMA_CHANNEL_GET_INSTANCE(hdma)), CCR);

  /* Check DMA channel active interrupts */
  {
    if (LL_DMA_IsActiveFlag_MIS(instance, channel) == 0U)
    {
      return; /* The global interrupt flag is low; nothing to handle */
    }
  }

  flags = LL_DMA_READ_REG((DMA_CHANNEL_GET_INSTANCE(hdma)), CSR);

  /* Half Transfer Complete Interrupt management **********************************************************************/
  if (STM32_READ_BIT((flags & its), LL_DMA_FLAG_HT) != 0U)
  {
    LL_DMA_ClearFlag_HT(DMA_CHANNEL_GET_INSTANCE(hdma));

    hdma->p_xfer_halfcplt_cb(hdma);

    if (STM32_READ_BIT((flags & its), LL_DMA_FLAG_TC) == 0U)
    {
      return;
    }
  }

  /* Transfer Complete Interrupt management ***************************************************************************/
  if (STM32_READ_BIT((flags & its), LL_DMA_FLAG_TC) != 0U)
  {
    LL_DMA_ClearFlag_TC(DMA_CHANNEL_GET_INSTANCE(hdma));

    /* Check if there are remaining data */
    if (LL_DMA_IsActiveFlag_IDLE(DMA_CHANNEL_GET_INSTANCE(hdma)) != 0U)
    {
      LL_DMA_ClearFlag_HT(DMA_CHANNEL_GET_INSTANCE(hdma));

      LL_DMA_DisableIT(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_IT_ALL);

      hdma->global_state = HAL_DMA_STATE_IDLE;
    }

    hdma->p_xfer_cplt_cb(hdma);

    return;
  }

  /* Suspend Transfer Interrupt management ****************************************************************************/
  if (STM32_READ_BIT((flags & its), LL_DMA_FLAG_SUSP) != 0U)
  {
    LL_DMA_ClearFlag_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma));

    if (hdma->global_state == HAL_DMA_STATE_ABORT)
    {
      LL_DMA_ResetChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

      LL_DMA_DisableIT(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_IT_ALL);

      hdma->global_state = HAL_DMA_STATE_IDLE;

      hdma->p_xfer_abort_cb(hdma);
    }
    else
    {
      LL_DMA_DisableIT_SUSP(DMA_CHANNEL_GET_INSTANCE(hdma));

      hdma->global_state = HAL_DMA_STATE_SUSPEND;

      hdma->p_xfer_suspend_cb(hdma);
    }

    return;
  }

  /* Error Interrupt management ***************************************************************************************/
  DMA_HandleErrorIT(hdma, STM32_READ_BIT((flags & its), (HAL_DMA_FLAG_ERROR | LL_DMA_FLAG_TO)));
}
/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group5
  * @{

This subsection provides a set of functions to register the DMA channel process and error callbacks:

- Call the function HAL_DMA_RegisterXferHalfCpltCallback() to register the DMA channel half transfer complete callback

- Call the function HAL_DMA_RegisterXferCpltCallback() to register the DMA channel transfer complete callback

- Call the function HAL_DMA_RegisterXferAbortCallback() to register the DMA channel abort callback

- Call the function HAL_DMA_RegisterXferSuspendCallback() to register the DMA channel suspend callback

- Call the function HAL_DMA_RegisterXferErrorCallback() to register the DMA channel error callback

- Call the function HAL_DMA_SetUserData() to set a user data in handle

- Call the function HAL_DMA_GetUserData() to get a user data from handle
  */

/**
  * @brief  Store the given callback into the DMA handle.
  * @param  hdma              Pointer to DMA channel handle
  * @param  callback          Half transfer complete callback
  * @retval HAL_INVALID_PARAM Invalid parameter return when callback pointer is NULL
  * @retval HAL_OK            DMA channel half transfer complete callback is successfully stored
  */
hal_status_t HAL_DMA_RegisterXferHalfCpltCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->p_xfer_halfcplt_cb = callback;

  return HAL_OK;
}

/**
  * @brief  Store the given callback into the DMA handle.
  * @param  hdma              Pointer to DMA channel handle
  * @param  callback          Transfer complete callback
  * @retval HAL_INVALID_PARAM Invalid parameter return when callback pointer is NULL
  * @retval HAL_OK            DMA channel transfer complete callback is successfully stored
  */
hal_status_t HAL_DMA_RegisterXferCpltCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->p_xfer_cplt_cb = callback;

  return HAL_OK;
}

/**
  * @brief  Store the given callback into the DMA handle.
  * @param  hdma              Pointer to DMA channel handle
  * @param  callback          Abort callback
  * @retval HAL_INVALID_PARAM Invalid parameter return when callback pointer is NULL
  * @retval HAL_OK            DMA channel abort transfer callback is successfully stored
  */
hal_status_t HAL_DMA_RegisterXferAbortCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->p_xfer_abort_cb = callback;

  return HAL_OK;
}

/**
  * @brief  Store the given callback into the DMA handle.
  * @param  hdma              Pointer to DMA channel handle
  * @param  callback          Suspend callback
  * @retval HAL_INVALID_PARAM Invalid parameter return when callback pointer is NULL
  * @retval HAL_OK            DMA channel suspend transfer callback is successfully stored
  */
hal_status_t HAL_DMA_RegisterXferSuspendCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->p_xfer_suspend_cb = callback;

  return HAL_OK;
}

/**
  * @brief  Store the given callback into the DMA handle.
  * @param  hdma              Pointer to DMA channel handle
  * @param  callback          Error callback
  * @retval HAL_INVALID_PARAM Invalid parameter return when callback pointer is NULL
  * @retval HAL_OK            DMA channel error transfer callback is successfully stored
  */
hal_status_t HAL_DMA_RegisterXferErrorCallback(hal_dma_handle_t *hdma, hal_dma_cb_t callback)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_PARAM(callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdma->p_xfer_error_cb = callback;

  return HAL_OK;
}

/**
  * @brief DMA channel half transfer complete default callback.
  * @param hdma Pointer to DMA channel handle
  */
__WEAK void HAL_DMA_XferHalfCpltCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /*! <b>NOTE:</b> This is a weak function and must not be modified, when the callback is needed, the
                   HAL_DMA_RegisterXferHalfCpltCallback() must be implemented in the user file */
}

/**
  * @brief DMA channel transfer complete default callback.
  * @param hdma Pointer to DMA channel handle
  */
__WEAK void HAL_DMA_XferCpltCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /*! <b>NOTE:</b> This is a weak function and must not be modified, when the callback is needed, the
                   HAL_DMA_RegisterXferCpltCallback() must be implemented in the user file */
}

/**
  * @brief DMA channel abort default callback.
  * @param hdma Pointer to DMA channel handle
  */
__WEAK void HAL_DMA_XferAbortCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /*! <b>NOTE:</b> This is a weak function and must not be modified, when the callback is needed, the
                   HAL_DMA_RegisterXferAbortCallback() must be implemented in the user file */
}

/**
  * @brief DMA channel suspend default callback.
  * @param hdma Pointer to DMA channel handle
  */
__WEAK void HAL_DMA_XferSuspendCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /*! <b>NOTE:</b> This is a weak function and must not be modified, when the callback is needed, the
                   HAL_DMA_RegisterXferSuspendCallback() must be implemented in the user file */
}

/**
  * @brief DMA channel suspend default callback.
  * @param hdma Pointer to DMA channel handle
  */
__WEAK void HAL_DMA_XferErrorCallback(hal_dma_handle_t *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdma);

  /*! <b>NOTE:</b> This is a weak function and must not be modified, when the callback is needed, the
                   HAL_DMA_RegisterXferErrorCallback() must be implemented in the user file */
}

#if defined(USE_HAL_DMA_USER_DATA) && (USE_HAL_DMA_USER_DATA == 1)
/**
  * @brief Store the user data into the DMA channel handle.
  * @param hdma        Pointer to DMA channel handle
  * @param p_user_data Pointer to the user data
  */
void HAL_DMA_SetUserData(hal_dma_handle_t *hdma, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hdma != NULL);

  hdma->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve the user data from the DMA channel handle.
  * @param  hdma Pointer to DMA channel handle
  * @retval Pointer to the user data
  */
const void *HAL_DMA_GetUserData(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);

  return (hdma->p_user_data);
}
#endif /* USE_HAL_DMA_USER_DATA */
/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group6
  * @{

This subsection provides a set of functions to get the DMA channel data information and status:

- Call the function HAL_DMA_GetDirectXferRemainingDataByte() to get the DMA channel remaining data within the current
  transfer in byte

- Call the function HAL_DMA_GetState() to get the DMA channel current state

- Call the function HAL_DMA_GetLastErrorCodes() to get the DMA channel last errors codes
  */

/**
  * @brief  Get the DMA channel remaining data in the current transfer in byte.
  * @param  hdma     Pointer to DMA channel handle
  * @retval uint32_t Direct transfer remaining data in byte
  */
uint32_t HAL_DMA_GetDirectXferRemainingDataByte(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);
  ASSERT_DBG_STATE(hdma->global_state,
                   (uint32_t)HAL_DMA_STATE_IDLE | (uint32_t)HAL_DMA_STATE_ACTIVE | (uint32_t)HAL_DMA_STATE_SUSPEND);

  return (LL_DMA_GetBlkDataLength(DMA_CHANNEL_GET_INSTANCE(hdma))
         );
}

/**
  * @brief  Get the DMA channel current state.
  * @param  hdma            Pointer to DMA channel handle
  * @retval hal_dma_state_t DMA channel state
  */
hal_dma_state_t HAL_DMA_GetState(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);

  return (hdma->global_state);
}

#if defined (USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
/**
  * @brief  Get last error codes.
  * @param  hdma     Pointer to DMA channel handle
  * @retval uint32_t Last error codes which can be a combination of @ref DMA_Error_Code
  */
uint32_t HAL_DMA_GetLastErrorCodes(const hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hdma != NULL);

  return (hdma->last_error_codes);
}
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */
/**
  * @}
  */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief Set the DMA channel transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  */
static void DMA_SetConfigDirectXfer(hal_dma_handle_t *hdma, const hal_dma_direct_xfer_config_t *p_config)
{
  LL_DMA_SetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_config->priority);
  LL_DMA_ConfigTransfer(DMA_CHANNEL_GET_INSTANCE(hdma),
                        ((uint32_t)p_config->dest_inc | (uint32_t)p_config->dest_data_width |
                         (uint32_t)p_config->src_inc  | (uint32_t)p_config->src_data_width));

  if (p_config->direction != HAL_DMA_DIRECTION_MEMORY_TO_MEMORY)
  {
    LL_DMA_SetPeriphRequest(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_config->request);
  }

  LL_DMA_SetDataTransferDirection(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_config->direction);
  LL_DMA_SetHWRequestMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)HAL_DMA_HARDWARE_REQUEST_BURST);
}

/**
  * @brief Get the DMA channel transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_direct_xfer_config_t configuration structure
  */
static void DMA_GetConfigDirectXfer(hal_dma_handle_t *hdma, hal_dma_direct_xfer_config_t *p_config)
{
  p_config->request         = (hal_dma_request_source_t)LL_DMA_GetPeriphRequest(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->direction       = (hal_dma_direction_t)LL_DMA_GetDataTransferDirection(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->src_inc         = (hal_dma_src_addr_increment_t)LL_DMA_GetSrcIncMode(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->dest_inc        = (hal_dma_dest_addr_increment_t)LL_DMA_GetDestIncMode(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->src_data_width  = (hal_dma_src_data_width_t)LL_DMA_GetSrcDataWidth(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->dest_data_width = (hal_dma_dest_data_width_t)LL_DMA_GetDestDataWidth(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->priority        = (hal_dma_priority_t)LL_DMA_GetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma));
}

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief Set the DMA channel linked list transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_linkedlist_xfer_config_t configuration structure
  */
static void DMA_SetConfigLinkedListXfer(hal_dma_handle_t *hdma, const hal_dma_linkedlist_xfer_config_t *p_config)
{
  LL_DMA_ConfigControl(DMA_CHANNEL_GET_INSTANCE(hdma), ((uint32_t)p_config->priority));
  LL_DMA_SetTransferEventMode(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_config->xfer_event_mode);
  LL_DMA_SetBlkDataLength(DMA_CHANNEL_GET_INSTANCE(hdma), 0U);
}

/**
  * @brief Get the DMA channel linked list transfer configuration.
  * @param hdma     Pointer to DMA channel handle
  * @param p_config Pointer to @ref hal_dma_linkedlist_xfer_config_t configuration structure
  */
static void DMA_GetConfigLinkedListXfer(hal_dma_handle_t *hdma, hal_dma_linkedlist_xfer_config_t *p_config)
{
  p_config->priority = (hal_dma_priority_t)LL_DMA_GetChannelPriorityLevel(DMA_CHANNEL_GET_INSTANCE(hdma));
  p_config->xfer_event_mode = (hal_dma_linkedlist_xfer_event_mode_t)
                              LL_DMA_GetTransferEventMode(DMA_CHANNEL_GET_INSTANCE(hdma));
}

/**
  * @brief Fill the DMA channel linked list node configuration.
  * @param p_conf    Pointer to @ref hal_dma_node_config_t configuration structure
  * @param p_node    Pointer to @ref hal_dma_node_t node structure
  * @param node_type Element in @ref hal_dma_node_type_t enumeration
  */
static void DMA_FillNodeConfig(hal_dma_node_t *p_node, const hal_dma_node_config_t *p_conf,
                               hal_dma_node_type_t node_type)
{
  uint32_t idx   = 0U;
  uint32_t dummy;

  /* Update CTR1 register value */
  dummy = (uint32_t)p_conf->xfer.src_inc | (uint32_t)p_conf->xfer.dest_inc | (uint32_t)p_conf->xfer.src_data_width |
          (uint32_t)p_conf->xfer.dest_data_width |
          (uint32_t)p_conf->data_handling.trunc_padd;

  STM32_WRITE_REG(p_node->regs[idx], dummy);


  idx++;

  /* Update CTR2 register value */
  dummy = (uint32_t)p_conf->hw_request_mode                                     |
          (uint32_t)p_conf->flow_ctrl_mode                                      |
          (uint32_t)p_conf->xfer_event_mode                                     |
          (uint32_t)p_conf->xfer.direction                                      |
          ((uint32_t)p_conf->xfer.request & (DMA_CTR2_REQSEL | DMA_CTR2_SWREQ)) |
          (uint32_t)p_conf->trigger.mode                                        |
          (uint32_t)p_conf->trigger.polarity                                    |
          (((uint32_t)p_conf->trigger.source << DMA_CTR2_TRIGSEL_Pos) & DMA_CTR2_TRIGSEL);

  STM32_WRITE_REG(p_node->regs[idx], dummy);

  idx++;

  /* Update CBR1 register value */
  STM32_WRITE_REG(p_node->regs[idx], (uint32_t)(p_conf->size_byte & DMA_CBR1_BNDT));

  idx++;

  /* Update CSAR register value */
  p_node->regs[idx] = (uint32_t)p_conf->src_addr;

  idx++;

  /* Update CDAR register value */
  p_node->regs[idx] = (uint32_t)p_conf->dest_addr;

  idx++;

  /* Reset CLLR register value */
  STM32_WRITE_REG(p_node->regs[idx], 0U);

  /* Set node type */
  p_node->info = (uint32_t)node_type;
}

/**
  * @brief Get node configuration of DMA channel linked list.
  * @param p_node      Pointer to @ref hal_dma_node_t node structure
  * @param p_conf      Pointer to @ref hal_dma_node_config_t configuration structure
  * @param p_node_type Pointer to @ref hal_dma_node_type_t enumeration
  */
static void DMA_GetConfigNode(const hal_dma_node_t *p_node, hal_dma_node_config_t *p_conf,
                              hal_dma_node_type_t *p_node_type)
{
  uint32_t dummy;

  /* Get node type */
  *p_node_type = (hal_dma_node_type_t)p_node->info;

  /* Get CTR1 fields values */
  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_SINC;
  p_conf->xfer.src_inc = (hal_dma_src_addr_increment_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_DINC;
  p_conf->xfer.dest_inc = (hal_dma_dest_addr_increment_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_SDW_LOG2;
  p_conf->xfer.src_data_width = (hal_dma_src_data_width_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_DDW_LOG2;
  p_conf->xfer.dest_data_width = (hal_dma_dest_data_width_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR1_REG_OFFSET] & DMA_CTR1_PAM_0;
  p_conf->data_handling.trunc_padd = (hal_dma_dest_data_trunc_padd_t)dummy;


  /* Get CTR2 fields values */
  if ((p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_SWREQ) != 0U)
  {
    p_conf->xfer.request   = HAL_DMA_REQUEST_SW;
    p_conf->xfer.direction = HAL_DMA_DIRECTION_MEMORY_TO_MEMORY;
  }
  else
  {
    dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_REQSEL;
    p_conf->xfer.request = (hal_dma_request_source_t)dummy;
    p_conf->xfer.direction = HAL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  }

  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_BREQ;
  p_conf->hw_request_mode = (hal_dma_hardware_request_mode_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_PFREQ;
  p_conf->flow_ctrl_mode = (hal_dma_flow_control_mode_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGM;
  p_conf->trigger.mode = (hal_dma_trigger_mode_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGPOL;
  p_conf->trigger.polarity = (hal_dma_trigger_polarity_t)dummy;
  dummy = (p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TRIGSEL) >> DMA_CTR2_TRIGSEL_Pos;
  p_conf->trigger.source = (hal_dma_trigger_source_t)dummy;
  dummy = p_node->regs[DMA_NODE_CTR2_REG_OFFSET] & DMA_CTR2_TCEM;
  p_conf->xfer_event_mode = (hal_dma_linkedlist_xfer_event_mode_t)dummy;

  /* Get CBR1 fields */
  p_conf->size_byte = p_node->regs[DMA_NODE_CBR1_REG_OFFSET] & DMA_CBR1_BNDT;

  /* Get CSAR field */
  p_conf->src_addr  = p_node->regs[DMA_NODE_CSAR_REG_OFFSET];

  /* Get CDAR field */
  p_conf->dest_addr = p_node->regs[DMA_NODE_CDAR_REG_OFFSET];

  STM32_UNUSED(dummy);
}

/**
  * @brief Fill the DMA channel linked list node direct transfer.
  * @param p_node          Pointer to @ref hal_dma_node_t configuration structure
  * @param p_config        Pointer to @ref hal_dma_direct_xfer_config_t node structure
  * @param node_type       Element in @ref hal_dma_node_type_t enumeration
  * @param xfer_event_mode Element in @ref hal_dma_linkedlist_xfer_event_mode_t enumeration
  */
static void DMA_FillNodeDirectXfer(hal_dma_node_t *p_node, const hal_dma_direct_xfer_config_t *p_config,
                                   hal_dma_node_type_t node_type, hal_dma_linkedlist_xfer_event_mode_t xfer_event_mode)
{

  hal_dma_node_config_t p_conf;

  /* Set direct transfer configuration */
  p_conf.xfer.request                         = p_config->request;
  p_conf.hw_request_mode                      = HAL_DMA_HARDWARE_REQUEST_BURST;
  p_conf.flow_ctrl_mode                       = HAL_DMA_FLOW_CONTROL_DMA;
  p_conf.xfer.direction                       = p_config->direction;
  p_conf.xfer.src_inc                         = p_config->src_inc;
  p_conf.xfer.dest_inc                        = p_config->dest_inc;
  p_conf.xfer.src_data_width                  = p_config->src_data_width;
  p_conf.xfer.dest_data_width                 = p_config->dest_data_width;
  p_conf.xfer_event_mode                      = xfer_event_mode;
  p_conf.trigger.source                       = HAL_LPDMA1_TRIGGER_EXTI0;
  p_conf.trigger.mode                         = HAL_DMA_TRIGGER_SINGLE_BURST_TRANSFER;
  p_conf.trigger.polarity                     = HAL_DMA_TRIGGER_POLARITY_MASKED;
  p_conf.data_handling.trunc_padd             = HAL_DMA_DEST_DATA_TRUNC_LEFT_PADD_ZERO;
  p_conf.src_addr                             = 0U;
  p_conf.dest_addr                            = 0U;
  p_conf.size_byte                            = 0U;


  DMA_FillNodeConfig(p_node, &p_conf, node_type);
}

/**
  * @brief Update the DMA channel linked list node.
  * @param p_node    Pointer to @ref hal_dma_node_t node structure
  * @param src_addr  Source address
  * @param dest_addr Destination address
  * @param size_byte Size in byte
  */
static void DMA_UpdateDataNode(hal_dma_node_t *p_node, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte)
{
  p_node->regs[DMA_NODE_CBR1_REG_OFFSET] = size_byte;
  p_node->regs[DMA_NODE_CSAR_REG_OFFSET] = src_addr;
  p_node->regs[DMA_NODE_CDAR_REG_OFFSET] = dest_addr;
}

/**
  * @brief Convert linked list queue associated to the handle to dynamic format.
  * @param p_q Pointer to hal_q_t configuration structure
  */
static void DMA_ConvertQNodesToDynamic(hal_q_t *p_q)
{
  uint32_t cllr_offset;
  uint32_t currentnode_position = 0U;
  uint32_t currentnode_address  = 0U;

  uint32_t currentnode_addr;
  hal_dma_node_t context_node = {0};

  cllr_offset = ((hal_dma_node_t *)(p_q->p_head_node))->info;

  /* Check unexpected node format */
  if (cllr_offset == (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
  {
    /* Check queue circularity */
    if (p_q->p_first_circular_node != 0U)
    {
      /* Check that previous node is linked to the selected queue */
      while (currentnode_position < p_q->node_nbr)
      {
        /* Get head node address */
        if (currentnode_position == 0U)
        {
          currentnode_address = (uint32_t)p_q->p_head_node & DMA_CLLR_LA;
        }
        /* Calculate nodes addresses */
        else
        {
          currentnode_address =
            ((hal_dma_node_t *)(currentnode_address + ((uint32_t)p_q->p_head_node & DMA_CLBAR_LBA)))->regs[cllr_offset]
            & DMA_CLLR_LA;
        }

        currentnode_position++;
      }

      currentnode_address = currentnode_address | ((uint32_t)p_q->p_head_node & DMA_CLBAR_LBA);
    }

    currentnode_addr = (uint32_t)p_q->p_head_node;

    /* Store register value */
    for (uint32_t reg_idx = 0U; reg_idx < DMA_NODE_REGISTER_NUM; reg_idx++)
    {
      context_node.regs[reg_idx] = ((hal_dma_node_t *)p_q->p_head_node)->regs[reg_idx];
    }

    context_node.info = ((hal_dma_node_t *)p_q->p_head_node)->info;

    /* Convert all nodes to dyncamic (Bypass head node) */
    for (uint32_t node_count = 1U; node_count < p_q->node_nbr; node_count++)
    {
      STM32_MODIFY_REG(currentnode_addr, DMA_CLLR_LA, (context_node.regs[cllr_offset] & DMA_CLLR_LA));

      /* Bypass the first circular node when first circular node is not the last queue node */
      if (((uint32_t)p_q->p_first_circular_node != 0U)
          && ((uint32_t)p_q->p_first_circular_node != currentnode_address)
          && ((uint32_t)p_q->p_first_circular_node == currentnode_addr))
      {
        /* Copy first circular node to context node */
        for (uint32_t reg_idx = 0U; reg_idx < DMA_NODE_REGISTER_NUM; reg_idx++)
        {
          context_node.regs[reg_idx] = ((hal_dma_node_t *)p_q->p_first_circular_node)->regs[reg_idx];
        }

        context_node.info = ((hal_dma_node_t *)p_q->p_first_circular_node)->info;
      }
      else
      {
        DMA_List_ConvertNodeToDynamic((uint32_t)&context_node, currentnode_addr, (cllr_offset + 1U));
      }
    }

    /* Check if first circular node is the last node queue */
    if (((uint32_t)p_q->p_first_circular_node != 0U)
        && ((uint32_t)p_q->p_first_circular_node != currentnode_address))
    {
      DMA_List_UpdateDynamicQueueNodesCLLR(p_q, DMA_LASTNODE_ISNOT_CIRCULAR);
    }
    else
    {
      DMA_List_UpdateDynamicQueueNodesCLLR(p_q, DMA_LASTNODE_IS_CIRCULAR);
    }
  }
}

/**
  * @brief Convert linked list queue associated to the handle to static format.
  * @param p_q Pointer to hal_q_t configuration structure
  */
static void DMA_ConvertQNodesToStatic(hal_q_t *p_q)
{
  uint32_t cllr_offset;
  uint32_t currentnode_addr;
  hal_dma_node_t context_node = {0};

  currentnode_addr = (uint32_t)p_q->p_head_node;

  cllr_offset = ((hal_dma_node_t *)(p_q->p_head_node))->info;

  /* Check unexpected node format */
  if (cllr_offset == (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
  {
    DMA_List_UpdateStaticQueueNodesCLLR(p_q, DMA_UPDATE_CLLR_POSITION);

    /* Convert all nodes to static (Bypass head node) */
    for (uint32_t node_count = 1U; node_count < p_q->node_nbr; node_count++)
    {
      /* Update context node register values */
      for (uint32_t reg_idx = 0U; reg_idx < DMA_NODE_REGISTER_NUM; reg_idx++)
      {
        context_node.regs[reg_idx] = ((hal_dma_node_t *)currentnode_addr)->regs[reg_idx];
      }

      context_node.info = ((hal_dma_node_t *)currentnode_addr)->info;

      STM32_MODIFY_REG(currentnode_addr, DMA_CLLR_LA, (context_node.regs[cllr_offset] & DMA_CLLR_LA));

      DMA_List_ConvertNodeToStatic((uint32_t)&context_node, currentnode_addr, (cllr_offset + 1U));
    }

    DMA_List_UpdateStaticQueueNodesCLLR(p_q, DMA_UPDATE_CLLR_VALUE);
  }
}

/**
  * @brief Convert node to dynamic.
  * @param context_node_addr The context node address
  * @param current_node_addr The current node address to be converted
  * @param reg_nbr           The register number to be converted
  */
static void DMA_List_ConvertNodeToDynamic(uint32_t context_node_addr, uint32_t current_node_addr, uint32_t reg_nbr)
{
  /* Check unexpected node format */
  if (reg_nbr == ((uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING + (uint32_t)1U))
  {
    uint32_t currentnode_reg_counter = 0U;
    uint32_t contextnode_reg_counter = 0U;
    uint32_t cllr_idx = reg_nbr - 1U;
    uint32_t update_link[DMA_NODE_REGISTER_NUM] = {DMA_CLLR_UT1, DMA_CLLR_UT2, DMA_CLLR_UB1, DMA_CLLR_USA,
                                                   DMA_CLLR_UDA, DMA_CLLR_ULL
                                                  };
    hal_dma_node_t *context_node = (hal_dma_node_t *)context_node_addr;
    hal_dma_node_t *current_node = (hal_dma_node_t *)current_node_addr;

    /* Update ULL position according to register number */
    update_link[cllr_idx] = update_link[DMA_NODE_REGISTER_NUM - 1U];

    /* Repeat for all node registers */
    while (contextnode_reg_counter != reg_nbr)
    {
      /* Check if register values are equal (exception for CSAR, CDAR and CLLR registers) */
      if ((context_node->regs[contextnode_reg_counter]  == current_node->regs[currentnode_reg_counter])
          && (contextnode_reg_counter != DMA_NODE_CSAR_DEFAULT_OFFSET)
          && (contextnode_reg_counter != (reg_nbr - 1U)))
      {
        DMA_List_FormatNode(current_node, currentnode_reg_counter, reg_nbr, DMA_NODE_DYNAMIC_FORMAT);

        /* Out of bounds check */
        if (cllr_idx == 0U)
        {
          break;
        }
        cllr_idx--;
        current_node->regs[cllr_idx] &= ~update_link[contextnode_reg_counter];
      }
      else
      {
        context_node->regs[contextnode_reg_counter] = current_node->regs[currentnode_reg_counter];

        current_node->regs[cllr_idx] |= update_link[contextnode_reg_counter];

        currentnode_reg_counter++;
      }

      contextnode_reg_counter++;
    }

    STM32_MODIFY_REG(current_node->info, DMA_NODE_CLLR_IDX, ((currentnode_reg_counter - 1U) << DMA_NODE_CLLR_IDX_POS));

    DMA_List_ClearUnusedFields(current_node, currentnode_reg_counter);
  }
}

/**
  * @brief Convert node to static.
  * @param context_node_addr The context node address.
  * @param current_node_addr The current node address to be converted.
  * @param reg_nbr           The register number to be converted.
  */
static void DMA_List_ConvertNodeToStatic(uint32_t context_node_addr, uint32_t current_node_addr, uint32_t reg_nbr)
{
  /* Check unexpected node format */
  if (reg_nbr == ((uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING + (uint32_t)1U))
  {
    hal_dma_node_t *context_node = (hal_dma_node_t *)context_node_addr;
    hal_dma_node_t *current_node = (hal_dma_node_t *)current_node_addr;
    uint32_t contextnode_reg_counter = 0U;
    uint32_t update_link[DMA_NODE_REGISTER_NUM] = {DMA_CLLR_UT1, DMA_CLLR_UT2, DMA_CLLR_UB1, DMA_CLLR_USA,
                                                   DMA_CLLR_UDA, DMA_CLLR_ULL
                                                  };
    uint32_t cllr_mask;
    uint8_t  cllr_idx;

    /* Update ULL position according to register number */
    update_link[reg_nbr - 1U] = update_link[DMA_NODE_REGISTER_NUM - 1U];

    /* Get context node CLLR information */
    cllr_idx  = (uint8_t)context_node->info & 0x7U;

    /* Check node info is HAL_DMA_NODE_LINEAR_ADDRESSING */
    /* Act as a defensive check in the event node info has been corrupted in memory */
    if (cllr_idx != (uint8_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
    {
      /* Node type is unexpected. Set to default addressing mode (linear) */
      cllr_idx = (uint8_t)HAL_DMA_NODE_LINEAR_ADDRESSING;
    }

    current_node->info = cllr_idx;
    cllr_mask = (uint32_t)context_node->regs[cllr_idx];

    while (contextnode_reg_counter != reg_nbr)
    {
      /* Check if node field is dynamic */
      if ((cllr_mask & update_link[contextnode_reg_counter]) == 0U)
      {
        DMA_List_FormatNode(current_node, contextnode_reg_counter, reg_nbr, DMA_NODE_STATIC_FORMAT);

        current_node->regs[contextnode_reg_counter] = context_node->regs[contextnode_reg_counter];
      }

      contextnode_reg_counter++;
    }
  }
}

/**
  * @brief Update CLLR for all dynamic queue nodes.
  * @param p_q                   Pointer to a hal_q_t structure that contains queue information
  * @param last_node_is_circular The first circular node is the last queue node or not
  */
static void DMA_List_UpdateDynamicQueueNodesCLLR(const hal_q_t *p_q, uint32_t last_node_is_circular)
{
  uint32_t previous_cllr_offset;
  uint32_t current_cllr_offset = 0U;
  uint32_t previousnode_addr;
  uint32_t currentnode_addr = (uint32_t)p_q->p_head_node;
  uint32_t cllr_mask = LL_DMA_UPDATE_ALL;
  uint32_t node_idx = 0U;

  /*  Repeat for all register nodes */
  while (node_idx < p_q->node_nbr)
  {
    /* Get head node address */
    if (node_idx == 0U)
    {
      current_cllr_offset = ((hal_dma_node_t *)currentnode_addr)->info;
    }
    /* Calculate nodes addresses */
    else
    {
      previousnode_addr = currentnode_addr;
      previous_cllr_offset = current_cllr_offset;

      currentnode_addr = (((hal_dma_node_t *)(previousnode_addr))->regs[previous_cllr_offset] & DMA_CLLR_LA) +
                         ((uint32_t)p_q->p_head_node & DMA_CLBAR_LBA);
      if (((hal_dma_node_t *)currentnode_addr)->info == (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
      {
        current_cllr_offset = ((hal_dma_node_t *)currentnode_addr)->info;
      }
      else
      {
        current_cllr_offset = (((hal_dma_node_t *)currentnode_addr)->info >> 8U);
      }

      cllr_mask = (((hal_dma_node_t *)currentnode_addr)->regs[current_cllr_offset] & ~DMA_CLLR_LA) |
                  (((hal_dma_node_t *)(previousnode_addr))->regs[previous_cllr_offset] & DMA_CLLR_LA);

      ((hal_dma_node_t *)(previousnode_addr))->regs[previous_cllr_offset] = cllr_mask;
    }

    node_idx++;
  }

  /* Check queue circularity */
  if (p_q->p_first_circular_node != 0U)
  {
    /* First circular queue is not last queue node */
    if (last_node_is_circular == 0U)
    {
      DMA_List_GetCLLRNodeInfo(((hal_dma_node_t *)currentnode_addr), &cllr_mask, NULL);

      ((hal_dma_node_t *)currentnode_addr)->regs[current_cllr_offset] =
        ((uint32_t)p_q->p_first_circular_node & DMA_CLLR_LA) | cllr_mask;
    }
    /* First circular queue is last queue node */
    else
    {
      /* Disable CLLR updating */
      ((hal_dma_node_t *)currentnode_addr)->regs[current_cllr_offset] &= ~DMA_CLLR_ULL;
    }
  }
  else
  {
    /* Clear CLLR register for last node */
    ((hal_dma_node_t *)currentnode_addr)->regs[current_cllr_offset] = 0U;
  }
}

/**
  * @brief Update CLLR for all static queue nodes.
  * @param p_q       Pointer to a hal_q_t structure that contains queue information
  * @param operation The operation type
  */
static void DMA_List_UpdateStaticQueueNodesCLLR(hal_q_t *p_q, uint32_t operation)
{
  uint32_t currentnode_addr = (uint32_t)p_q->p_head_node;
  uint32_t current_cllr_offset = (uint32_t)((hal_dma_node_t *)p_q->p_head_node)->info;
  uint32_t cllr_default_offset = (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING;
  uint32_t cllr_default_mask = 0U;
  uint32_t cllr_mask;
  uint32_t node_idx = 0U;

  DMA_List_GetCLLRNodeInfo((const hal_dma_node_t *)p_q->p_head_node, &cllr_default_mask, &cllr_default_offset);

  /*  Repeat for all register nodes (Bypass last queue node) */
  while (node_idx < p_q->node_nbr)
  {
    if (operation == DMA_UPDATE_CLLR_POSITION)
    {
      cllr_mask = ((hal_dma_node_t *)currentnode_addr)->regs[current_cllr_offset];
    }
    else
    {
      cllr_mask = (((hal_dma_node_t *)currentnode_addr)->regs[((hal_dma_node_t *)currentnode_addr)->info] &
                   DMA_CLLR_LA) | cllr_default_mask;
    }

    /* Set new CLLR value to default position */
    if ((node_idx == (p_q->node_nbr - 1U)) && (p_q->p_first_circular_node == NULL))
    {
      ((hal_dma_node_t *)(currentnode_addr))->regs[cllr_default_offset] = 0U;
    }
    else
    {
      ((hal_dma_node_t *)(currentnode_addr))->regs[cllr_default_offset] = cllr_mask;
    }

    currentnode_addr = (currentnode_addr & DMA_CLBAR_LBA) | (cllr_mask & DMA_CLLR_LA);

    /* Update current CLLR offset with next CLLR offset */
    if (((hal_dma_node_t *)currentnode_addr)->info == (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
    {
      current_cllr_offset = ((hal_dma_node_t *)currentnode_addr)->info;
    }
    else
    {
      current_cllr_offset = (((hal_dma_node_t *)currentnode_addr)->info >> 8U);
    }

    node_idx++;
  }
}

/**
  * @brief Check nodes types compatibility.
  * @param p_node     Pointer to a @ref hal_dma_node_t structure that contains linked list node registers configurations
  * @param p_cllr_mask   Pointer to CLLR register mask value
  * @param p_cllr_offset Pointer to CLLR register offset value
  */
static void DMA_List_GetCLLRNodeInfo(const hal_dma_node_t *p_node, uint32_t *p_cllr_mask, uint32_t *p_cllr_offset)
{
  if ((p_node->info & (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING) == (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING)
  {
    *p_cllr_mask = DMA_CLLR_UT1 | DMA_CLLR_UT2 | DMA_CLLR_UB1 | DMA_CLLR_USA | DMA_CLLR_UDA | DMA_CLLR_ULL;

    if (p_cllr_offset != NULL)
    {
      *p_cllr_offset = (uint32_t)HAL_DMA_NODE_LINEAR_ADDRESSING;
    }
  }
}

/**
  * @brief Format the node according to unused registers.
  * @param p_node  Pointer to a @ref hal_dma_node_t structure that contains linked list node registers configurations
  * @param reg_idx The first register index to be formatted
  * @param reg_nbr The number of node registers
  * @param format  The format type
  */
static void DMA_List_FormatNode(hal_dma_node_t *p_node, uint32_t reg_idx, uint32_t reg_nbr, uint32_t format)
{
  if (format == DMA_NODE_DYNAMIC_FORMAT)
  {
    for (uint32_t reg_id = reg_idx; reg_id < (reg_nbr - 1U); reg_id++)
    {
      p_node->regs[reg_id] = p_node->regs[reg_id + 1U];
    }
  }
  else
  {
    for (uint32_t reg_id = (reg_nbr - 2U); reg_id > reg_idx; reg_id--)
    {
      p_node->regs[reg_id] = p_node->regs[reg_id - 1U];
    }
  }
}

/**
  * @brief Clear unused register fields.
  * @param p_node             Pointer to a @ref hal_dma_node_t structure that contains linked list node registers
  *                           configurations
  * @param first_unused_field The first unused field to be cleared
  */
static void DMA_List_ClearUnusedFields(hal_dma_node_t *p_node, uint32_t first_unused_field)
{
  for (uint32_t reg_idx = first_unused_field; reg_idx < DMA_NODE_REGISTER_NUM; reg_idx++)
  {
    p_node->regs[reg_idx] = 0U;
  }
}

#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief Start the DMA channel direct transfer.
  * @param hdma       Pointer to DMA channel handle
  * @param src_addr   Source address
  * @param dest_addr  Destination address
  * @param size_byte  Size in byte
  * @param interrupts DMA optional interrupts to be enabled.
  *                   This parameter can be one of @ref DMA_Optional_Interrupt group.
  */
static void DMA_StartDirectXfer(hal_dma_handle_t *hdma, uint32_t src_addr, uint32_t dest_addr, uint32_t size_byte,
                                uint32_t interrupts)
{
  LL_DMA_ConfigAddresses(DMA_CHANNEL_GET_INSTANCE(hdma), src_addr, dest_addr);
  LL_DMA_SetBlkDataLength(DMA_CHANNEL_GET_INSTANCE(hdma), size_byte);

  LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

  LL_DMA_DisableIT(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_IT_ALL);

  if (interrupts != HAL_DMA_OPT_IT_SILENT)
  {
    LL_DMA_EnableIT(DMA_CHANNEL_GET_INSTANCE(hdma), (LL_DMA_IT_TC  | LL_DMA_IT_DTE | LL_DMA_IT_ULE | LL_DMA_IT_USE |
                                                     interrupts));
  }

  LL_DMA_ConfigLinkUpdate(DMA_CHANNEL_GET_INSTANCE(hdma), 0U, 0U);

  LL_DMA_EnableChannel(DMA_CHANNEL_GET_INSTANCE(hdma));
}

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief Start the DMA channel linked list transfer.
  * @param hdma        Pointer to DMA channel handle
  * @param p_head_node Pointer to hal_q_t node structure
  * @param interrupts  DMA optional interrupts to be enabled.
  *                    This parameter can be one of @ref DMA_Optional_Interrupt group.
  */
static void DMA_StartLinkedListXfer(hal_dma_handle_t *hdma, const void *p_head_node, uint32_t interrupts)
{
  uint32_t update_bits = LL_DMA_UPDATE_CTR1 | LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 |
                         LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;

  LL_DMA_SetLinkedListBaseAddr(DMA_CHANNEL_GET_INSTANCE(hdma), (uint32_t)p_head_node);

  LL_DMA_ConfigLinkUpdate(DMA_CHANNEL_GET_INSTANCE(hdma), update_bits, ((uint32_t)p_head_node & DMA_CLLR_LA));

  LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

  LL_DMA_DisableIT(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_IT_ALL);

  if (interrupts != HAL_DMA_OPT_IT_SILENT)
  {
    LL_DMA_EnableIT(DMA_CHANNEL_GET_INSTANCE(hdma), (LL_DMA_IT_TC  | LL_DMA_IT_DTE | LL_DMA_IT_ULE | LL_DMA_IT_USE |
                                                     interrupts));
  }

  LL_DMA_SetBlkDataLength(DMA_CHANNEL_GET_INSTANCE(hdma), 0U);

  LL_DMA_EnableChannel(DMA_CHANNEL_GET_INSTANCE(hdma));
}
#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief  Handle the DMA channel error interrupt.
  * @param  hdma      Pointer to DMA channel handle
  * @param  error_msk Mask of errors flags
  */
static void DMA_HandleErrorIT(hal_dma_handle_t *hdma, uint32_t error_msk)
{
#if defined(USE_HAL_DMA_GET_LAST_ERRORS) && (USE_HAL_DMA_GET_LAST_ERRORS == 1)
  /* Check the data transfer error flag */
  if ((error_msk & LL_DMA_FLAG_DTE) != 0U)
  {
    hdma->last_error_codes |= HAL_DMA_ERROR_DTE;
  }

  /* Check the user setting error flag */
  if ((error_msk & LL_DMA_FLAG_USE) != 0U)
  {
    hdma->last_error_codes |= HAL_DMA_ERROR_USE;
  }

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  /* Check the update link error flag */
  if ((error_msk & LL_DMA_FLAG_ULE) != 0U)
  {
    hdma->last_error_codes |= HAL_DMA_ERROR_ULE;
  }
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Check trigger overrun flag */
  if ((error_msk & LL_DMA_FLAG_TO) != 0U)
  {
    hdma->last_error_codes |= HAL_DMA_ERROR_TO;
  }
#endif /* USE_HAL_DMA_GET_LAST_ERRORS */

  LL_DMA_ClearFlag(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_FLAG_ALL);

  /* Check error flags */
  if ((error_msk & HAL_DMA_FLAG_ERROR) != 0U)
  {
    /* This code is not run when only DMA trigger overrun error occurs */

    LL_DMA_ResetChannel(DMA_CHANNEL_GET_INSTANCE(hdma));

    LL_DMA_DisableIT(DMA_CHANNEL_GET_INSTANCE(hdma), LL_DMA_IT_ALL);

    hdma->global_state = HAL_DMA_STATE_IDLE;
  }

  hdma->p_xfer_error_cb(hdma);

}
/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_DMA_MODULE */
#endif /* (defined (LPDMA1) || defined (LPDMA2)) */

/**
  * @}
  */
