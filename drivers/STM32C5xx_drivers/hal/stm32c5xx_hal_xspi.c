/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_xspi.c
  * @brief   XSPI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functions of the Extended Serial Peripheral Interface (XSPI) peripheral:
  *           + Initialization and deinitialization functions
  *           + I/O operation functions
  *           + Peripheral control functions
  *           + Peripheral state functions.
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

/** @addtogroup XSPI
  * @{
  */
/** @defgroup XSPI_Introduction XSPI Introduction
  * @{

  - The **XSPI** hardware abstraction layer provides a set of APIs to interface with the STM32 XSPI (Extended Serial
    Peripheral Interface) peripheral.

  - It simplifies the configuration, initialization, and management of **XSPI** communication.
    Memory configuration and data transfer can be performed in different modes:
    - Indirect access mode: polling, interrupt, DMA for configuration and data transfer.
    - Direct access mode: memory mapped for data transfer only.

  - This abstraction layer ensures portability and ease of use across different STM32 series.

  */
/**
  * @}
  */

/** @defgroup XSPI_How_To_Use XSPI How To Use
  * @{

An explanation of XSPI overall usage:
- XSPI is the abbreviation of Expanded Serial Peripheral Interface. It is an interface that supports most external
  serial memories such as serial PSRAMs, serial NAND and serial NOR flash memories, HyperRAM and HyperFlash memories,
  with different functional modes.

This file provides firmware functions to manage the following functions of the XSPI peripheral:

- Initialization and deinitialization functions
- Configuration functions
- Command and I/O operation functions
- IRQHandler, link DMA, and callback functions
- Status functions
- Delay Block functions
- High-speed interface and calibration functions
- Interrupt functions

# How to use the XSPI HAL module driver

## Initialization and deinitialization functions:

- Declare a hal_xspi_handle_t handle structure, for example: hal_xspi_handle_t hxspi
- Use HAL_XSPI_Init() function to initialize the XSPI handle and associate the physical instance
- Use HAL_XSPI_DeInit() function to abort any ongoing operation then reset the state

## Configuration functions

- Use HAL_XSPI_SetConfig() function to configure the Regular/Hyperbus parameters of the XSPI peripheral

- Use HAL_XSPI_GetConfig() function to retrieve the current configuration for the XSPI instance


- Once a global configuration is applied (using HAL_XSPI_SetConfig()), optionally use the following unitary functions to
  update the different parameters individually:

  - HAL_XSPI_SetFifoThreshold()          Configure the FIFO threshold according to the user parameters.
  - HAL_XSPI_GetFifoThreshold()          Retrieve the FIFO threshold configuration.
  - HAL_XSPI_SetPrescaler()              Configure the prescaler according to the user parameters.
  - HAL_XSPI_GetPrescaler()              Retrieve the prescaler configuration.
  - HAL_XSPI_SetMemorySize()             Configure the XSPI memory size according to the user parameters.
  - HAL_XSPI_GetMemorySize()             Retrieve the XSPI memory size configuration.
  - HAL_XSPI_SetMemoryType()             Configure the XSPI memory type according to the user parameters.
  - HAL_XSPI_GetMemoryType()             Retrieve the XSPI memory type configuration.
  - HAL_XSPI_SetMemorySelection()        Configure the nCS used for memory selection.
  - HAL_XSPI_GetMemorySelection()        Retrieve the nCS used for memory selection.
  - HAL_XSPI_EnableFreeRunningClock()    Enable the free-running clock.
  - HAL_XSPI_DisableFreeRunningClock()   Disable the free-running clock.
  - HAL_XSPI_IsEnabledFreeRunningClock() Check whether the free-running clock is enabled.
  - HAL_XSPI_EnablePrefetchData()        Enable automatic prefetch in the external memory (reset value).
  - HAL_XSPI_DisablePrefetchData()       Disable automatic prefetch in the external memory.
  - HAL_XSPI_IsEnabledPrefetchData()     Check the state of automatic prefetch in the external memory.

## Command and I/O operation functions

- In Regular mode, use HAL_XSPI_SendRegularCmd()
  or
  HAL_XSPI_SendRegularCmd_IT() function to configure the command sequence.

- In Hyperbus mode, use HAL_XSPI_SendHyperbusCmd() function to configure the command sequence.

- If no data is required for the command (only for Regular Command mode, not for Hyperbus mode), it is sent directly to
  the memory:

  - In polling mode, the function returns when the command is complete.
  - In interrupt mode, the function starts the process and returns, the process continues in the background
    (based on the various interrupts). The HAL_XSPI_CmdCpltCallback() is called when the process is complete.

- For the indirect write mode, use HAL_XSPI_Transmit(), HAL_XSPI_Transmit_DMA(), HAL_XSPI_Transmit_DMA_Opt()
  or HAL_XSPI_Transmit_IT() after the command configuration:

  - In polling mode, the function returns when the transfer is complete.
  - In interrupt mode, the function starts the process and returns, the process continues in the background
    (based on the various interrupts). The HAL_XSPI_TxCpltCallback() is called when the process is complete.
  - In DMA mode:
      - HAL_XSPI_TxHalfCpltCallback() is called at the half transfer and
        HAL_XSPI_TxCpltCallback() is called when the transfer is complete.
      - The half transfer is filtered with HAL_XSPI_Transmit_DMA_Opt() and
        HAL_XSPI_TxCpltCallback() is called when the transfer is complete.

- For the indirect read mode, use HAL_XSPI_Receive(), HAL_XSPI_Receive_DMA(), HAL_XSPI_Receive_DMA_Opt()
  or HAL_XSPI_Receive_IT() after the command configuration:

  - In polling mode, the function returns when the receive is complete.
  - In interrupt mode, the function starts the process and returns, the process continues in the background
    (based on the various interrupts). The HAL_XSPI_RxCpltCallback() is called when the process is complete.
  - In DMA mode:
      - HAL_XSPI_RxHalfCpltCallback() is called at the half transfer and
        HAL_XSPI_RxCpltCallback() is called when the transfer is complete.
      - The half transfer is filtered with HAL_XSPI_Receive_DMA_Opt() and
        HAL_XSPI_RxCpltCallback() is called when the transfer is complete.

- Use HAL_XSPI_ExecRegularAutoPoll() or HAL_XSPI_ExecRegularAutoPoll_IT() function to configure
  the auto-polling functional mode:

  - In polling mode, the function returns when the status match is reached. The automatic stop is
    activated to avoid an infinite loop.
  - In interrupt mode, HAL_XSPI_StatusMatchCallback() is called each time the status match is reached.

- Use HAL_XSPI_Abort()
  or
  HAL_XSPI_Abort_IT() function to abort any ongoing operation and to flush the FIFO:

  - In polling mode, the function returns when the transfer-complete bit is set and the busy bit is cleared.
  - In interrupt mode, the function starts the process and returns, the process continues in the background
    (based on the various interrupts). The HAL_XSPI_AbortCpltCallback() is called when the process is complete.

- Use HAL_XSPI_StartMemoryMappedMode() function to start the memory-mapped functional mode.

- Use HAL_XSPI_StopMemoryMappedMode() function to stop the memory-mapped functional mode.

## XSPI IRQHandler, link DMA, and callback functions

- XSPI IRQHandler

  - Use HAL_XSPI_IRQHandler() function called under XSPI_IRQHandler interrupt subroutine to handle any XSPI interrupt.

- Link DMA

  - Use HAL_XSPI_SetTxDMA() function to link/store Tx HAL DMA handle into the HAL XSPI handle.
  - Use HAL_XSPI_SetRxDMA() function to link/store Rx HAL DMA handle into the HAL XSPI handle.

- Callback functions

  - Call the function HAL_XSPI_RegisterErrorCallback() to register the XSPI error callback to be used instead of
    the weak HAL_XSPI_ErrorCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterAbortCpltCallback() to register the XSPI abort-complete callback to be used
    instead of the weak HAL_XSPI_AbortCpltCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterFifoThresholdCallback() to register the XSPI FIFO Threshold callback to be
    used instead of the weak HAL_XSPI_FifoThresholdCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterStatusMatchCallback() to register the XSPI Status Match callback to be used
    instead of the weak HAL_XSPI_StatusMatchCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterTxCpltCallback() to register the XSPI Transfer-Complete callback to be used
    instead of the weak HAL_XSPI_TxCpltCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterTxHalfCpltCallback() to register the XSPI Half-Transfer complete callback to be
    used instead of the weak HAL_XSPI_TxHalfCpltCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterRxCpltCallback() to register the XSPI Receive-Complete callback to be used
    instead of the weak HAL_XSPI_RxCpltCallback() predefined callback.

  - Call the function HAL_XSPI_RegisterRxHalfCpltCallback() to register the XSPI Half-Receive complete callback to be
    used instead of the weak HAL_XSPI_RxHalfCpltCallback() predefined callback.

## State functions

- Use HAL_XSPI_GetState() function to get the current state of the HAL XSPI driver.

## Clock frequency of the XSPI peripheral

- Use HAL_XSPI_GetClockFreq() to retrieve the current clock frequency of the XSPI peripheral.

## XSPI Delay Block functions

- The delay block (DLYB) is used to generate an output clock that is dephased from the input clock.

  - Use HAL_XSPI_DLYB_SetConfigDelay() to set the delay configuration of the delay block peripheral.
  - Use HAL_XSPI_DLYB_GetConfigDelay() to get the delay output clock phase of the delay block peripheral.
  - Use HAL_XSPI_DLYB_CalculateMaxClockPhase() to calculate the maximum output clock phase of the
    delay block peripheral.
  - Use HAL_XSPI_DLYB_Enable() to enable the delay block peripheral.
  - Use HAL_XSPI_DLYB_Disable() to disable the delay block peripheral.
  - Use HAL_XSPI_DLYB_IsEnabled() to check if the delay block peripheral is enabled or not.


## Interrupt functions

- Use HAL_XSPI_EnableIT() function to enable the interrupts.

- Use HAL_XSPI_DisableIT() function to disable the interrupts.

- Use HAL_XSPI_IsEnabledIT() function to get the interrupt source.

- Use HAL_XSPI_IsActiveFlag() function to get flags.

- Use HAL_XSPI_ClearFlag() function to clear flags.
  */
/**
  * @}
  */

/** @defgroup XSPI_Configuration_Table XSPI Configuration Table
  * @{
## Configuration inside the XSPI driver

Config defines                  | Description     | Default value     | Note
------------------------------- | --------------- | ----------------- | ------------------------------------------------
PRODUCT                         | from IDE        |        NA         | The selected device (Ex: STM32C5XX)
USE_HAL_XSPI_MODULE             | from hal_conf.h |        1U         | Allows use of the HAL XSPI module
USE_ASSERT_DBG_PARAM            | from IDE        |        NA         | Allows use of the assert check parameters
USE_ASSERT_DBG_STATE            | from IDE        |        NA         | Allows use of the assert check states
USE_HAL_CHECK_PARAM             | from hal_conf.h |        0U         | Allows use of the run-time check parameters
USE_HAL_CHECK_PROCESS_STATE     | from hal_conf.h |        0U         | Allows load and store exclusive operations
USE_HAL_XSPI_DMA                | from hal_conf.h |        1U         | Allows use of DMA mode
USE_HAL_XSPI_HYPERBUS           | from hal_conf.h |        1U         | Allows use of the HYPERBUS protocol
USE_HAL_XSPI_REGISTER_CALLBACKS | from hal_conf.h |        0U         | Allows use of register callbacks
USE_HAL_XSPI_CLK_ENABLE_MODEL   | from hal_conf.h | HAL_CLK_ENABLE_NO | Allows use of the clock enable model
 */
/**
  * @}
  */
#if defined(XSPI1)

#if defined (USE_HAL_XSPI_MODULE) && (USE_HAL_XSPI_MODULE == 1U)

/** @defgroup XSPI_Private_Constants XSPI Private Constants
  * @{
  */
#define XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE 0x00000000U     /*!< Indirect write mode               */
#define XSPI_FUNCTIONAL_MODE_INDIRECT_READ  XSPI_CR_FMODE_0 /*!< Indirect read mode                */
#define XSPI_FUNCTIONAL_MODE_AUTO_POLLING   XSPI_CR_FMODE_1 /*!< Automatic polling mode            */
#define XSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  XSPI_CR_FMODE   /*!< Memory-mapped mode                */

#define XSPI_TIMEOUT_DEFAULT_VALUE          5U              /*!< XSPI timeout 5 seconds            */

#define XSPI_FIFO_FULL_SIZE                 64U             /*!< Data pass through 64-byte FIFO    */
#define XSPI_FIFO_MEDIUM_SIZE               32U             /*!< Data pass through 32-byte FIFO    */
#define XSPI_IO_SELECT_MSK                  XSPI_CR_MSEL    /*!< IO memory selection               */
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup XSPI_Private_Macros XSPI Private Macros
  * @{
  */
/**
  * @brief Get the XSPI instance.
  */
#define XSPI_GET_INSTANCE(handle) ((XSPI_TypeDef *)((uint32_t)(handle)->instance))

/**
  * @brief Get the XSPI instance within the Delay Block.
  */
#define XSPI_DLYB_GET_INSTANCE(instance) \
  ({                                     \
    STM32_UNUSED((instance));            \
    DLYB_XSPI1;                          \
  })

/**
  * @brief Check the FIFO threshold.
  */
#define IS_XSPI_FIFO_THRESHOLD_BYTE(instance, threshold) ((IS_XSPI_FULL_FIFO_SIZE(instance)) ?      \
                                                          (((threshold) >= 1U)                      \
                                                           && ((threshold) <= XSPI_FIFO_FULL_SIZE)) \
                                                          :(((threshold) >= 1U)                     \
                                                            && ((threshold) <= XSPI_FIFO_MEDIUM_SIZE)))

/**
  * @brief Check the Memory mode
  */
#define IS_XSPI_MEMORY_MODE(mode)                 (((mode) == HAL_XSPI_MEMORY_SINGLE) \
                                                   || ((mode) == HAL_XSPI_MEMORY_DUAL))

/**
  * @brief Check the Memory type
  */
#if defined(XSPI_DCR1_MTYP_2)
#define IS_XSPI_MEMORY_TYPE(type)                 (((type) == HAL_XSPI_MEMORY_TYPE_MICRON)          \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_MACRONIX)     \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_APMEM)        \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_MACRONIX_RAM) \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_HYPERBUS))
#else
#define IS_XSPI_MEMORY_TYPE(type)                 (((type) == HAL_XSPI_MEMORY_TYPE_MICRON)          \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_MACRONIX)     \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_APMEM)        \
                                                   || ((type) == HAL_XSPI_MEMORY_TYPE_MACRONIX_RAM))
#endif /* XSPI_DCR1_MTYP_2 */

/**
  * @brief Check the Memory size
  */
#define IS_XSPI_MEMORY_SIZE(size)                 (((size) == HAL_XSPI_MEMORY_SIZE_16BIT)      \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_32BIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_64BIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_128BIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_256BIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_512BIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_1KBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_2KBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_4KBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_8KBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_16KBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_32KBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_64KBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_128KBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_256KBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_512KBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_1MBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_2MBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_4MBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_8MBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_16MBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_32MBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_64MBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_128MBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_256MBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_512MBIT) \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_1GBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_2GBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_4GBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_8GBIT)   \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_16GBIT)  \
                                                   || ((size) == HAL_XSPI_MEMORY_SIZE_32GBIT))

/**
  * @brief Check the Chip select high time cycle
  */
#define IS_XSPI_CS_HIGH_TIME_CYCLE(time)          (((time) >= 1U) && ((time) <= 64U))


#if defined(XSPI_DCR2_WRAPSIZE)
/**
  * @brief Check the Wrap size
  */
#define IS_XSPI_WRAP_SIZE(size)                   (((size) == HAL_XSPI_WRAP_NOT_SUPPORTED) \
                                                   || ((size) == HAL_XSPI_WRAP_16BYTE)     \
                                                   || ((size) == HAL_XSPI_WRAP_32BYTE)     \
                                                   || ((size) == HAL_XSPI_WRAP_64BYTE)     \
                                                   || ((size) == HAL_XSPI_WRAP_128BYTE))
#endif /* XSPI_DCR2_WRAPSIZE */

/**
  * @brief Check the prescaler factor
  */
#define IS_XSPI_CLOCK_PRESCALER(prescaler)        ((prescaler) <= 255U)

/**
  * @brief Check the Sample shift
  */
#define IS_XSPI_SAMPLE_SHIFT(cycle)               (((cycle) == HAL_XSPI_SAMPLE_SHIFT_NONE) \
                                                   || ((cycle) == HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE))

/**
  * @brief Check the delay hold
  */
#define IS_XSPI_DELAY_HOLD(cycle)                 (((cycle) == HAL_XSPI_DELAY_HOLD_NONE) \
                                                   || ((cycle) == HAL_XSPI_DELAY_HOLD_QUARTCYCLE))

#if defined(XSPI_DCR3_CSBOUND)
/**
  * @brief Check the chip select boundary
  */
#define IS_XSPI_CS_BOUNDARY(size)                 (((size) == HAL_XSPI_CS_BOUNDARY_NONE)       \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_16BIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_32BIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_64BIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_128BIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_256BIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_512BIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_1KBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_2KBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_4KBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_8KBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_16KBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_32KBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_64KBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_128KBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_256KBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_512KBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_1MBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_2MBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_4MBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_8MBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_16MBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_32MBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_64MBIT)  \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_128MBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_256MBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_512MBIT) \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_1GBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_2GBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_4GBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_8GBIT)   \
                                                   || ((size) == HAL_XSPI_CS_BOUNDARY_16GBIT))
#endif /* XSPI_DCR3_CSBOUND */

/**
  * @brief Check the delay block bypass
  */
#define IS_XSPI_DLYB_BYPASS(dlyb)                 (((dlyb) == HAL_XSPI_DLYB_ON) \
                                                   || ((dlyb) == HAL_XSPI_DLYB_BYPASS))

/**
  * @brief Check the memory selection
  */
#define IS_XSPI_MEMORY_SELECTION(select)          (((select) == HAL_XSPI_MEMORY_SELECTION_NCS1) \
                                                   || ((select) == HAL_XSPI_MEMORY_SELECTION_NCS2))

/**
  * @brief Check the Operation type
  */
#if defined(XSPI_WIR_INSTRUCTION)
#define IS_XSPI_OPERATION_TYPE(type)              (((type) == HAL_XSPI_OPERATION_READ_CFG)     \
                                                   || ((type) == HAL_XSPI_OPERATION_WRITE_CFG) \
                                                   || ((type) == HAL_XSPI_OPERATION_WRAP_CFG))
#else
#define IS_XSPI_OPERATION_TYPE(type)              ((type) == HAL_XSPI_OPERATION_READ_CFG)
#endif /* XSPI_WIR_INSTRUCTION */

/**
  * @brief Check the I/O select
  */
#define IS_XSPI_IO_SELECT(instance, memsel)       (((memsel) == HAL_XSPI_IO_3_0)    \
                                                   || ((memsel) == HAL_XSPI_IO_7_4) \
                                                   || ((memsel) == HAL_XSPI_IO_7_0))

/**
  * @brief Check the Instruction mode
  */
#define IS_XSPI_INSTRUCTION_MODE(mode)            (((mode) == HAL_XSPI_INSTRUCTION_NONE)      \
                                                   || ((mode) == HAL_XSPI_INSTRUCTION_1LINE)  \
                                                   || ((mode) == HAL_XSPI_INSTRUCTION_2LINES) \
                                                   || ((mode) == HAL_XSPI_INSTRUCTION_4LINES) \
                                                   || ((mode) == HAL_XSPI_INSTRUCTION_8LINES))

/**
  * @brief Check the Instruction width
  */
#define IS_XSPI_INSTRUCTION_WIDTH(width)          (((width) == HAL_XSPI_INSTRUCTION_8BIT)     \
                                                   || ((width) == HAL_XSPI_INSTRUCTION_16BIT) \
                                                   || ((width) == HAL_XSPI_INSTRUCTION_24BIT) \
                                                   || ((width) == HAL_XSPI_INSTRUCTION_32BIT))

/**
  * @brief Check the Instruction dtr mode
  */
#define IS_XSPI_INSTRUCTION_DTR_MODE(mode)        (((mode) == HAL_XSPI_INSTRUCTION_DTR_DISABLED) \
                                                   || ((mode) == HAL_XSPI_INSTRUCTION_DTR_ENABLED))

/**
  * @brief Check the Address mode
  */
#define IS_XSPI_ADDR_MODE(mode)                   (((mode) == HAL_XSPI_ADDR_NONE)      \
                                                   || ((mode) == HAL_XSPI_ADDR_1LINE)  \
                                                   || ((mode) == HAL_XSPI_ADDR_2LINES) \
                                                   || ((mode) == HAL_XSPI_ADDR_4LINES) \
                                                   || ((mode) == HAL_XSPI_ADDR_8LINES))

/**
  * @brief Check the Address width
  */
#define IS_XSPI_ADDR_WIDTH(width)                 (((width) == HAL_XSPI_ADDR_8BIT)     \
                                                   || ((width) == HAL_XSPI_ADDR_16BIT) \
                                                   || ((width) == HAL_XSPI_ADDR_24BIT) \
                                                   || ((width) == HAL_XSPI_ADDR_32BIT))

/**
  * @brief Check the Address dtr mode
  */
#define IS_XSPI_ADDR_DTR_MODE(mode)               (((mode) == HAL_XSPI_ADDR_DTR_DISABLED) \
                                                   || ((mode) == HAL_XSPI_ADDR_DTR_ENABLED))

/**
  * @brief Check the Alternate bytes mode
  */
#define IS_XSPI_ALTERNATE_BYTES_MODE(mode)        (((mode) == HAL_XSPI_ALTERNATE_BYTES_NONE)      \
                                                   || ((mode) == HAL_XSPI_ALTERNATE_BYTES_1LINE)  \
                                                   || ((mode) == HAL_XSPI_ALTERNATE_BYTES_2LINES) \
                                                   || ((mode) == HAL_XSPI_ALTERNATE_BYTES_4LINES) \
                                                   || ((mode) == HAL_XSPI_ALTERNATE_BYTES_8LINES))

/**
  * @brief Check the Alternate bytes width
  */
#define IS_XSPI_ALTERNATE_BYTES_WIDTH(width)      (((width) == HAL_XSPI_ALTERNATE_BYTES_8BIT)     \
                                                   || ((width) == HAL_XSPI_ALTERNATE_BYTES_16BIT) \
                                                   || ((width) == HAL_XSPI_ALTERNATE_BYTES_24BIT) \
                                                   || ((width) == HAL_XSPI_ALTERNATE_BYTES_32BIT))

/**
  * @brief Check the Alternate bytes dtr mode
  */
#define IS_XSPI_ALTERNATE_BYTES_DTR_MODE(mode)    (((mode) == HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED) \
                                                   || ((mode) == HAL_XSPI_ALTERNATE_BYTES_DTR_ENABLED))

/**
  * @brief Check the Regular data mode
  */
#define IS_XSPI_REGULAR_DATA_MODE(instance, mode) (((mode) == HAL_XSPI_REGULAR_DATA_NONE)      \
                                                   || ((mode) == HAL_XSPI_REGULAR_DATA_1LINE)  \
                                                   || ((mode) == HAL_XSPI_REGULAR_DATA_2LINES) \
                                                   || ((mode) == HAL_XSPI_REGULAR_DATA_4LINES) \
                                                   || ((mode) == HAL_XSPI_REGULAR_DATA_8LINES))

/**
  * @brief Check the Hyperbus data mode
  */
#define IS_XSPI_HYPERBUS_DATA_MODE(instance, mode) (((mode) == HAL_XSPI_HYPERBUS_DATA_8LINES))

/**
  * @brief Check the Data length
  */
#define IS_XSPI_DATA_LENGTH(number)               ((number) >= 1U)

/**
  * @brief Check the Data dtr mode
  */
#define IS_XSPI_DATA_DTR_MODE(mode)               (((mode) == HAL_XSPI_DATA_DTR_DISABLED) \
                                                   || ((mode) == HAL_XSPI_DATA_DTR_ENABLED))

/**
  * @brief Check the Dummy cycles
  */
#define IS_XSPI_DUMMY_CYCLES(number)              ((number) <= 31U)

/**
  * @brief Check the Dqs mode
  */
#define IS_XSPI_DQS_MODE(mode)                    (((mode) == HAL_XSPI_DQS_DISABLED) \
                                                   || ((mode) == HAL_XSPI_DQS_ENABLED))

/**
  * @brief Check the RW recovery time cycle
  */
#define IS_XSPI_RW_RECOVERY_TIME_CYCLE(cycle)     ((cycle) <= 255U)

/**
  * @brief Check the Access time cycle
  */
#define IS_XSPI_ACCESS_TIME_CYCLE(cycle)          ((cycle) <= 255U)

/**
  * @brief Check the Write zero latency
  */
#define IS_XSPI_WRITE_ZERO_LATENCY(mode)          (((mode) == HAL_XSPI_WRITE_ZERO_LATENCY_DISABLED) \
                                                   || ((mode) == HAL_XSPI_WRITE_ZERO_LATENCY_ENABLED))
/**
  * @brief Check the Latency mode
  */
#define IS_XSPI_LATENCY_MODE(mode)                (((mode) == HAL_XSPI_LATENCY_VARIABLE) \
                                                   || ((mode) == HAL_XSPI_LATENCY_FIXED))

/**
  * @brief Check the Address space
  */
#define IS_XSPI_ADDRESS_SPACE(SPACE)              (((SPACE) == HAL_XSPI_ADDR_MEMORY) \
                                                   || ((SPACE) == HAL_XSPI_ADDR_REGISTER))

/**
  * @brief Check the Match mode
  */
#define IS_XSPI_MATCH_MODE(mode)                  (((mode) == HAL_XSPI_MATCH_MODE_AND) \
                                                   || ((mode) == HAL_XSPI_MATCH_MODE_OR))

/**
  * @brief Check the Autromatic stop
  */
#define IS_XSPI_AUTOMATIC_STOP(mode)              (((mode) == HAL_XSPI_AUTOMATIC_STOP_ENABLED) \
                                                   || ((mode) == HAL_XSPI_AUTOMATIC_STOP_DISABLED))

/**
  * @brief Check the Interval time
  */
#define IS_XSPI_INTERVAL(interval)                ((interval) <= 0xFFFFU)

/**
  * @brief Check the Status bytes size
  */
#define IS_XSPI_STATUS_BYTES_SIZE(size)           (((size) >= 1U) && ((size) <= 4U))

/**
  * @brief Check the Timeout activation
  */
#define IS_XSPI_TIMEOUT_ACTIVATION(mode)          (((mode) == HAL_XSPI_TIMEOUT_DISABLE) \
                                                   || ((mode) == HAL_XSPI_TIMEOUT_ENABLE))

/**
  * @brief Check the Timeout period
  */
#define IS_XSPI_TIMEOUT_PERIOD(period)            ((period) <= 0xFFFFU)


/**
  * @brief Check XSPI optional interrupt
  */
#define IS_XSPI_OPT_IT(value)                     (((value) == HAL_XSPI_OPT_IT_NONE)  \
                                                   || ((value) == HAL_XSPI_OPT_IT_HT) \
                                                   || ((value) == HAL_XSPI_OPT_IT_DEFAULT))

/**
  * @}
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/** @defgroup XSPI_Private_Types XSPI Private Types
  * @{
  */
/*! XSPI Interrupt state*/
typedef enum
{
  XSPI_INTERRUPT_DISABLE = 0U, /*!< HAL XSPI interrupt disabled */
  XSPI_INTERRUPT_ENABLE  = 1U, /*!< HAL XSPI interrupt enabled  */

} hal_xspi_interrupt_state_t;
/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @defgroup XSPI_Private_Functions XSPI Private Functions
  * @{
  */
static hal_status_t XSPI_WaitFlagStateUntilTimeout(hal_xspi_handle_t *hxspi, uint32_t flag,
                                                   hal_xspi_flag_status_t state, uint32_t timeout_ms);
static hal_status_t XSPI_SendRegularCmd(hal_xspi_handle_t *hxspi, const hal_xspi_regular_cmd_t *p_cmd,
                                        uint32_t timeout_ms, hal_xspi_interrupt_state_t it_state);
#if defined(XSPI_SR_SMF)
static hal_status_t XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi, const hal_xspi_auto_polling_config_t *p_config,
                                             uint32_t timeout_ms, hal_xspi_interrupt_state_t it_state);
#else
static hal_status_t XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi, const hal_xspi_auto_polling_config_t *p_config,
                                             uint32_t timeout_ms);
#endif /* XSPI_SR_SMF */
static hal_status_t XSPI_Abort(hal_xspi_handle_t *hxspi, uint32_t timeout_ms);

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
static void  XSPI_DMACplt(hal_dma_handle_t *hdma);
static void  XSPI_DMAHalfCplt(hal_dma_handle_t *hdma);
static void  XSPI_DMAError(hal_dma_handle_t *hdma);
static void  XSPI_DMAAbort(hal_dma_handle_t *hdma);
static void  XSPI_DMAAbortOnError(hal_dma_handle_t *hdma);
#endif /* USE_HAL_XSPI_DMA */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup XSPI_Exported_Functions
  * @{
  */

/** @addtogroup XSPI_Exported_Functions_Group1
  * @{
This subsection provides a set of functions allowing to initialize and deinitialize the XSPIx peripheral:
- Call the function HAL_XSPI_Init() to initialize the selected HAL XSPI handle and associate an XSPI
  peripheral instance.
- Call the function HAL_XSPI_DeInit() to de-initialize the given HAL XSPI instance by stopping any ongoing process and
  resetting the state machine.

  */

/**
  * @brief  Initialize the XSPI according to the associated instance.
  * @param  instance XSPI instance, can be one of the XSPI instances as defined in the CMSIS device
  *                  header file.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @note   The XSPI clock can be activated within the HAL_XSPI_Init() function by setting
  *         the <b> USE_XSPI_CLK_ENABLE_MODEL </b> flag to **HAL_CLK_ENABLE_XSPI_ONLY**
  *         in the configuration file ** stm32tnxx_hal_conf.h **.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            hxspi instance has been correctly Initialized.
  */
hal_status_t HAL_XSPI_Init(hal_xspi_handle_t *hxspi, hal_xspi_t instance)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_ALL_INSTANCE((XSPI_TypeDef *)((uint32_t)instance)));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hxspi == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif  /* USE_HAL_CHECK_PARAM */

  /* Associate physical instance to logical object. */
  hxspi->instance = instance;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
  hxspi->p_error_cb          = HAL_XSPI_ErrorCallback;          /* Error callback reset.            */
  hxspi->p_abort_cplt_cb     = HAL_XSPI_AbortCpltCallback;      /* Abort callback reset.            */
  hxspi->p_fifo_threshold_cb = HAL_XSPI_FifoThresholdCallback;  /* FIFO Threshold callback reset.   */
  hxspi->p_cmd_cplt_cb       = HAL_XSPI_CmdCpltCallback;        /* Command Complete callback reset. */
  hxspi->p_rx_cplt_cb        = HAL_XSPI_RxCpltCallback;         /* Rx Complete callback reset.      */
  hxspi->p_tx_cplt_cb        = HAL_XSPI_TxCpltCallback;         /* Tx Complete callback reset.      */
  hxspi->p_rx_half_cplt_cb   = HAL_XSPI_RxHalfCpltCallback;     /* Rx Half Complete callback reset  */
  hxspi->p_tx_half_cplt_cb   = HAL_XSPI_TxHalfCpltCallback;     /* Tx Half Complete callback reset  */
  hxspi->p_status_match_cb   = HAL_XSPI_StatusMatchCallback;    /* Status Match callback reset      */
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */

#if defined(USE_HAL_XSPI_CLK_ENABLE_MODEL) && (USE_HAL_XSPI_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  /* Enable the XSPI Peripheral Clock. */
  switch (hxspi->instance)
  {
    case HAL_XSPI1:
      /* Enable Clock. */
      HAL_RCC_XSPI1_EnableClock();
      break;
    default:
      break;
  }
#endif /* USE_HAL_XSPI_CLK_ENABLE_MODEL */

#if defined (USE_HAL_XSPI_USER_DATA) && (USE_HAL_XSPI_USER_DATA == 1U)
  hxspi->p_user_data = NULL;
#endif /* USE_HAL_XSPI_USER_DATA */

#if defined (USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  hxspi->global_state = HAL_XSPI_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  De-Initialize the XSPI peripheral.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  */
void HAL_XSPI_DeInit(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_ALL_INSTANCE(XSPI_GET_INSTANCE(hxspi)));

  /* Abort the current XSPI operation. */
  (void)XSPI_Abort(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE);

  /* Disable XSPI Instances. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_EN);

  hxspi->global_state = HAL_XSPI_STATE_RESET;
}

/**
  * @}
  */

/** @addtogroup XSPI_Exported_Functions_Group2
  * @{
This subsection provides a set of functions to configure the XSPIx peripheral:

There are three categories of HAL configuration APIs:

- Global configuration APIs:
  - HAL_XSPI_SetConfig()  Set the HAL peripheral instance into a ready-to-use state (idle)
  according to the user parameters
  - HAL_XSPI_GetConfig()  Retrieve the HAL peripheral configuration
- Unitary configuration APIs:
  - HAL_XSPI_SetFifoThreshold()          Configure the FIFO threshold according to the user parameters.
  - HAL_XSPI_GetFifoThreshold()          Retrieve the FIFO threshold configuration.
  - HAL_XSPI_SetPrescaler()              Configure the prescaler according to the user parameters.
  - HAL_XSPI_GetPrescaler()              Retrieve the prescaler configuration.
  - HAL_XSPI_SetMemorySize()             Configure the XSPI memory size according to the user parameters.
  - HAL_XSPI_GetMemorySize()             Retrieve the XSPI memory size configuration.
  - HAL_XSPI_SetMemoryType()             Configure the XSPI memory type according to the user parameters.
  - HAL_XSPI_GetMemoryType()             Retrieve the XSPI memory type configuration.
  - HAL_XSPI_SetMemorySelection()        Configure the nCS used for memory selection.
  - HAL_XSPI_GetMemorySelection()        Retrieve the nCS used for memory selection.
  - HAL_XSPI_EnableFreeRunningClock()    Enable the free-running clock.
  - HAL_XSPI_DisableFreeRunningClock()   Disable the free-running clock.
  - HAL_XSPI_IsEnabledFreeRunningClock() Check whether the free-running clock is enabled.
  - HAL_XSPI_EnablePrefetchData()        Enable automatic prefetch in the external memory (reset value).
  - HAL_XSPI_DisablePrefetchData()       Disable automatic prefetch in the external memory.
  - HAL_XSPI_IsEnabledPrefetchData()     Check the state of automatic prefetch in the external memory.

These APIs are intended to dynamically modify/retrieve a unitary item, meaning that a global configuration has already
been applied.
Unitary configuration APIs must first check whether the driver is in the IDLE state (meaning a global configuration was
applied) in order to modify or retrieve a single item.
Items that can alter other configuration parameters must not be handled within unitary APIs.
  */

/**
  * @brief  Configure the XSPI according to the user parameters.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_config          Pointer to the hal_xspi_config_t structure.
  * @retval HAL_ERROR         XSPI instance is already configured and can not be modified.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            XSPI instance has been correctly configured.
  */
hal_status_t HAL_XSPI_SetConfig(hal_xspi_handle_t *hxspi, const hal_xspi_config_t *p_config)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_MODE(p_config->memory.mode));
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_TYPE(p_config->memory.type));
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_SIZE(p_config->memory.size_bit));
#if defined(XSPI_DCR2_WRAPSIZE)
  ASSERT_DBG_PARAM(IS_XSPI_WRAP_SIZE(p_config->memory.wrap_size_byte));
#endif /* XSPI_DCR2_WRAPSIZE */
#if defined(XSPI_DCR3_CSBOUND)
  ASSERT_DBG_PARAM(IS_XSPI_CS_BOUNDARY(p_config->memory.cs_boundary));
#endif /* XSPI_DCR3_CSBOUND */
  ASSERT_DBG_PARAM(IS_XSPI_CS_HIGH_TIME_CYCLE(p_config->timing.cs_high_time_cycle));
  ASSERT_DBG_PARAM(IS_XSPI_CLOCK_PRESCALER(p_config->timing.clk_prescaler));
  ASSERT_DBG_PARAM(IS_XSPI_SAMPLE_SHIFT(p_config->timing.shift));
  ASSERT_DBG_PARAM(IS_XSPI_DELAY_HOLD(p_config->timing.hold));
  ASSERT_DBG_PARAM(IS_XSPI_DLYB_BYPASS(p_config->timing.dlyb_state));
#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  if (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
  {
    ASSERT_DBG_PARAM(IS_XSPI_WRITE_ZERO_LATENCY(p_config->hyperbus.write_zero_latency));
    ASSERT_DBG_PARAM(IS_XSPI_RW_RECOVERY_TIME_CYCLE(p_config->hyperbus.rw_recovery_time_cycle));
    ASSERT_DBG_PARAM(IS_XSPI_ACCESS_TIME_CYCLE(p_config->hyperbus.access_time_cycle));
    ASSERT_DBG_PARAM(IS_XSPI_LATENCY_MODE(p_config->hyperbus.latency_mode));
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_INIT | HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_EN);

  /* Configure memory type, device size, chip select high time cycle, clock mode. */
  hxspi->type = p_config->memory.type ;
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR1,
                   (XSPI_DCR1_MTYP | XSPI_DCR1_DEVSIZE | XSPI_DCR1_CSHT |
                    XSPI_DCR1_CKMODE),
                   ((uint32_t)(p_config->memory.type) |
                    ((uint32_t)(p_config->memory.size_bit)) |
                    (((uint32_t)(p_config->timing.cs_high_time_cycle) - 1U) << XSPI_DCR1_CSHT_Pos)));

  /* Configure delay block bypass. */
  if (IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)))
  {
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_DLYBYP, (uint32_t)(p_config->timing.dlyb_state));
  }

#if defined(XSPI_DCR2_WRAPSIZE)
  /* Configure wrap size. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR2, XSPI_DCR2_WRAPSIZE, (uint32_t)(p_config->memory.wrap_size_byte));
#endif /* XSPI_DCR2_WRAPSIZE */

#if defined(XSPI_DCR3_CSBOUND)
  /* Configure chip select boundary. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR3, XSPI_DCR3_CSBOUND, ((uint32_t)(p_config->memory.cs_boundary)
                                                                       << XSPI_DCR3_CSBOUND_Pos));
#endif /* XSPI_DCR3_CSBOUND */

#if defined(XSPI_DCR4_REFRESH)
  /* Configure refresh. */
  XSPI_GET_INSTANCE(hxspi)->DCR4 = p_config->timing.cs_refresh_time_cycle;
#endif /* XSPI_DCR4_REFRESH */

  /* Configure new FIFO threshold. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FTHRES, 0U);
  hxspi->fifo_threshold = 1U;

  /* Wait until the busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
  {
    /* Configure clock prescaler. */
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR2, XSPI_DCR2_PRESCALER, ((p_config->timing.clk_prescaler)
                                                                           << XSPI_DCR2_PRESCALER_Pos));

    /* Configure the memory mode. */
    hxspi->mode = p_config->memory.mode ;
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMM, (uint32_t)(p_config->memory.mode));

    /* Configure sample shifting and delay hold quarter cycle. */
    hxspi->hold = p_config->timing.hold;
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->TCR, (XSPI_TCR_SSHIFT | XSPI_TCR_DHQC),
                     ((uint32_t)p_config->timing.shift | (uint32_t)p_config->timing.hold));
  }
  else
  {
    hxspi->global_state = HAL_XSPI_STATE_INIT;

    return HAL_ERROR;
  }
#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  /* Configure Hyperbus Memory. */
  if (p_config->memory.type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
  {
    /* Wait till busy flag is reset. */
    if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                       XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
    {
      /* Configure Hyperbus configuration Latency register. */
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->HLCR,
                      (((uint32_t)(p_config->hyperbus.rw_recovery_time_cycle) << XSPI_HLCR_TRWR_Pos) |
                       ((uint32_t)(p_config->hyperbus.access_time_cycle) << XSPI_HLCR_TACC_Pos)      |
                       (uint32_t)(p_config->hyperbus.write_zero_latency)                             |
                       (uint32_t)(p_config->hyperbus.latency_mode)));
    }
    else
    {
      hxspi->global_state = HAL_XSPI_STATE_INIT;

      return HAL_ERROR;
    }
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

  /* Enable XSPI. */
  STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_EN);

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the XSPI configuration.
  * @param  hxspi    Pointer to a \ref hal_xspi_handle_t structure that contains
  *                  the handle information for the specified XSPI instance.
  * @param  p_config Pointer to the hal_xspi_config_t structure.
  */
void HAL_XSPI_GetConfig(hal_xspi_handle_t *hxspi, hal_xspi_config_t *p_config)
{
  uint32_t tmp_dcr1_reg;
  uint32_t tmp_dcr2_reg;
#if defined(XSPI_DCR3_CSBOUND)
  uint32_t tmp_dcr3_reg;
#endif /* XSPI_DCR3_CSBOUND */
#if defined(XSPI_DCR4_REFRESH)
  uint32_t tmp_dcr4_reg;
#endif /* XSPI_DCR4_REFRESH */
  uint32_t tmp_tcr_reg;
  uint32_t tmp_cr_reg;
  uint32_t tmp_reg;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  tmp_dcr1_reg = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DCR1);
  tmp_dcr2_reg = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DCR2);
#if defined(XSPI_DCR3_CSBOUND)
  tmp_dcr3_reg = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DCR3);
#endif /* XSPI_DCR3_CSBOUND */
#if defined(XSPI_DCR4_REFRESH)
  tmp_dcr4_reg = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DCR4);
#endif /* XSPI_DCR4_REFRESH */
  tmp_cr_reg   = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->CR) ;
  tmp_tcr_reg  = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->TCR);

#if defined(XSPI_DCR3_CSBOUND)
  /* Retrieve the value of chip select boundary */
  tmp_reg = STM32_READ_BIT(tmp_dcr3_reg, XSPI_DCR3_CSBOUND) >> XSPI_DCR3_CSBOUND_Pos;
  p_config->memory.cs_boundary = (hal_xspi_cs_boundary_t)tmp_reg;
#endif /* XSPI_DCR3_CSBOUND */

  /* Retrieve the value of Memory mode */
  tmp_reg = STM32_READ_BIT(tmp_cr_reg, XSPI_CR_DMM);
  p_config->memory.mode = (hal_xspi_memory_mode_t)tmp_reg;
  hxspi->mode         = (hal_xspi_memory_mode_t)tmp_reg;

  /* Retrieve the value of Memory type */
  tmp_reg = STM32_READ_BIT(tmp_dcr1_reg, XSPI_DCR1_MTYP);
  p_config->memory.type = (hal_xspi_memory_type_t)tmp_reg;
  hxspi->type           = (hal_xspi_memory_type_t)tmp_reg;

  /* Retrieve the value of device size*/
  tmp_reg = STM32_READ_BIT(tmp_dcr1_reg, XSPI_DCR1_DEVSIZE);
  p_config->memory.size_bit = (hal_xspi_memory_size_t)tmp_reg;

#if defined(XSPI_DCR2_WRAPSIZE)
  /* Retrieve the value of wrap size */
  tmp_reg = STM32_READ_BIT(tmp_dcr2_reg, XSPI_DCR2_WRAPSIZE);
  p_config->memory.wrap_size_byte = (hal_xspi_wrap_size_t)tmp_reg;
#endif /* XSPI_DCR2_WRAPSIZE */

  /* Retrieve the value of chip select high time */
  tmp_reg = (STM32_READ_BIT(tmp_dcr1_reg, XSPI_DCR1_CSHT) >> XSPI_DCR1_CSHT_Pos) + 1U;
  p_config->timing.cs_high_time_cycle = tmp_reg;

  /* Retrieve the value of clock prescaler */
  tmp_reg = STM32_READ_BIT(tmp_dcr2_reg, XSPI_DCR2_PRESCALER) >> XSPI_DCR2_PRESCALER_Pos;
  p_config->timing.clk_prescaler = tmp_reg;

  /* Retrieve the value of sample shifting */
  tmp_reg = STM32_READ_BIT(tmp_tcr_reg, XSPI_TCR_SSHIFT);
  p_config->timing.shift = (hal_xspi_sample_shift_t)tmp_reg;

  /* Retrieve the value of delay hold quarter cycle */
  tmp_reg = STM32_READ_BIT(tmp_tcr_reg, XSPI_TCR_DHQC);
  p_config->timing.hold = (hal_xspi_delay_hold_t)tmp_reg;
  hxspi->hold           = (hal_xspi_delay_hold_t)tmp_reg;

  if (IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)))
  {
    /* Retrieve the value of delay block bypass */
    tmp_reg = STM32_READ_BIT(tmp_dcr1_reg, XSPI_DCR1_DLYBYP);
    p_config->timing.dlyb_state = (hal_xspi_dlyb_state_t)tmp_reg;
  }

#if defined(XSPI_DCR4_REFRESH)
  /* Retrieve the value of refresh */
  tmp_reg = STM32_READ_BIT(tmp_dcr4_reg, XSPI_DCR4_REFRESH);
  p_config->timing.cs_refresh_time_cycle = tmp_reg;
#endif /* XSPI_DCR4_REFRESH */

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  uint32_t tmp_HLCR = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->HLCR);
  /* Retrieve the XSPI hyperbus configuration */
  if (p_config->memory.type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
  {
    tmp_reg = STM32_READ_BIT(tmp_HLCR, XSPI_HLCR_TRWR);
    p_config->hyperbus.rw_recovery_time_cycle = tmp_reg;

    tmp_reg = STM32_READ_BIT(tmp_HLCR, XSPI_HLCR_TACC);
    p_config->hyperbus.access_time_cycle = tmp_reg;

    tmp_reg = STM32_READ_BIT(tmp_HLCR, XSPI_HLCR_WZL) >> XSPI_HLCR_WZL_Pos;
    p_config->hyperbus.write_zero_latency = (hal_xspi_write_zero_latency_status_t)tmp_reg;

    tmp_reg = STM32_READ_BIT(tmp_HLCR, XSPI_HLCR_LM);
    p_config->hyperbus.latency_mode = (hal_xspi_latency_mode_t)tmp_reg;
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */
}

/** @brief  Set the XSPI FIFO threshold.
  * @param  hxspi     Pointer to a \ref hal_xspi_handle_t structure that contains
  *                   the handle information for the specified XSPI instance.
  * @param  threshold Threshold of the FIFO can be a value from 0 to 31.
  * @retval HAL_OK    FIFO threshold has been correctly configured.
  */
hal_status_t HAL_XSPI_SetFifoThreshold(hal_xspi_handle_t *hxspi, uint32_t threshold)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_FIFO_THRESHOLD_BYTE(XSPI_GET_INSTANCE(hxspi), threshold));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Set the XSPI FIFO threshold. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FTHRES, ((threshold - 1U) << XSPI_CR_FTHRES_Pos));

  hxspi->fifo_threshold = threshold;

  return HAL_OK;
}

/** @brief  Get the XSPI FIFO threshold.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the FIFO threshold value.
  */
uint32_t HAL_XSPI_GetFifoThreshold(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the XSPI FIFO threshold. */
  return ((STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FTHRES) >> XSPI_CR_FTHRES_Pos) + 1U);
}

/** @brief  Set XSPI Clock Prescaler.
  * @param  hxspi         Pointer to a \ref hal_xspi_handle_t structure that contains
  *                       the handle information for the specified XSPI instance.
  * @param  clk_prescaler Prescaler generating the external clock can be a value from 0 to 255.
  * @retval HAL_ERROR     An error has occurred.
  * @retval HAL_OK        Clock Prescaler has been correctly configured.
  */
hal_status_t HAL_XSPI_SetPrescaler(hal_xspi_handle_t *hxspi, uint32_t clk_prescaler)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_CLOCK_PRESCALER(clk_prescaler));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
  {
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR2, XSPI_DCR2_PRESCALER,
                     (clk_prescaler << XSPI_DCR2_PRESCALER_Pos));

  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/** @brief  Get XSPI Clock Prescaler.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the value clock prescaler.
  */
uint32_t HAL_XSPI_GetPrescaler(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the XSPI prescaler. */
  return (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->DCR2, XSPI_DCR2_PRESCALER) >> XSPI_DCR2_PRESCALER_Pos);
}

/** @brief  Configure device memory size.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @param  size   The size of the external device connected to the XSPI.
  * @retval HAL_OK Size has been correctly configured.
  */
hal_status_t HAL_XSPI_SetMemorySize(hal_xspi_handle_t *hxspi, hal_xspi_memory_size_t size)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_SIZE(size));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Set the XSPI memory size. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_DEVSIZE, (uint32_t)size);

  return HAL_OK;
}

/** @brief  Get XSPI Memory Size.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the value device memory size.
  */
hal_xspi_memory_size_t HAL_XSPI_GetMemorySize(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the XSPI memory size. */
  return ((hal_xspi_memory_size_t)(uint32_t)STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_DEVSIZE));
}

/** @brief  Set XSPI Memory Type.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @param  type   The type of the external device connected to the XSPI.
  * @retval HAL_OK Type has been correctly configured.
  */
hal_status_t HAL_XSPI_SetMemoryType(hal_xspi_handle_t *hxspi, hal_xspi_memory_type_t type)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_TYPE(type));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Set the XSPI memory type. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_MTYP, (uint32_t)type);

  hxspi->type = type;

  return HAL_OK;
}

/** @brief  Get XSPI Memory Type.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @return Retrieve the type of the external device connected to the XSPI.
  */
hal_xspi_memory_type_t HAL_XSPI_GetMemoryType(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the XSPI memory type. */
  return (hxspi->type);
}

/** @brief  Set XSPI Memory Selection.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @param  select The nCS use by the XSPI to select the external device.
  * @retval HAL_OK Type has been correctly configured.
  */
hal_status_t HAL_XSPI_SetMemorySelection(hal_xspi_handle_t *hxspi, hal_xspi_memory_selection_t select)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_MEMORY_SELECTION(select));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Set the XSPI memory type. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_CSSEL, (uint32_t)select);

  return HAL_OK;
}

/** @brief  Get XSPI Memory Selection.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @return Retrieve the nCS used to select the external device connected to the XSPI.
  */
hal_xspi_memory_selection_t HAL_XSPI_GetMemorySelection(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the XSPI memory type. */
  return ((hal_xspi_memory_selection_t)(uint32_t)STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_CSSEL));
}


/** @brief  Get Base Address of XSPI in Memory-Mapped mode.
  * @param  hxspi  XSPI handle.
  * @return Retrieve the base address of the XSPI peripheral (or 0U in case of error).
  */
uint32_t HAL_XSPI_GetMemoryMappedBaseAddress(const hal_xspi_handle_t *hxspi)
{
  uint32_t base_address;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_ALL_INSTANCE(XSPI_GET_INSTANCE(hxspi)));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE | HAL_XSPI_STATE_INIT
                   | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE);

  switch (hxspi->instance)
  {
    case HAL_XSPI1:
      base_address = XSPI1_BASE;
      break;
    default:
      /* Return 0U (error) if instance doesn't exist. */
      base_address = 0U;
      break;
  }

  return base_address;
}

/** @brief  Enable the free running clock.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @retval HAL_OK free running clock has been correctly enabled.
  */
hal_status_t HAL_XSPI_EnableFreeRunningClock(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Enable the free running clock. */
  STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  return HAL_OK;
}

/** @brief  Disable the free running clock.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @retval HAL_OK free running clock has been correctly disabled.
  */
hal_status_t HAL_XSPI_DisableFreeRunningClock(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Disable the free running clock. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  return HAL_OK;
}

/** @brief  Check whether the free running clock is enabled or disabled.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the state of the free running clock.
  */
hal_xspi_free_running_clk_status_t HAL_XSPI_IsEnabledFreeRunningClock(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Get the free running clock. */
  return ((STM32_READ_BIT((XSPI_GET_INSTANCE(hxspi))->DCR1, XSPI_DCR1_FRCK) == 0UL) ? HAL_XSPI_FREE_RUNNING_CLK_DISABLED
          : HAL_XSPI_FREE_RUNNING_CLK_ENABLED);
}

/** @brief  Enable automatic prefetch in the external memory (Enabled by default).
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @retval HAL_OK automatic prefetch has been correctly enabled.
  */
hal_status_t HAL_XSPI_EnablePrefetchData(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Enable the automatic prefetch. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_NOPREF);

  return HAL_OK;
}
/** @brief  Disable automatic prefetch in the external memory.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @retval HAL_OK automatic prefetch has been correctly disabled.
  */
hal_status_t HAL_XSPI_DisablePrefetchData(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  /* Disable the automatic prefetch. */
  STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_NOPREF);

  return HAL_OK;
}
/** @brief  Check whether the automatic prefetch is enabled or disabled.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the state of the automatic prefetch.
  */
hal_xspi_prefetch_data_status_t HAL_XSPI_IsEnabledPrefetchData(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE);

  /* Get the automatic prefetch status. */
  return ((STM32_READ_BIT((XSPI_GET_INSTANCE(hxspi))->CR, XSPI_CR_NOPREF) == 0UL) ? HAL_XSPI_PREFETCH_DATA_ENABLED
          : HAL_XSPI_PREFETCH_DATA_DISABLED);
}
/**
  * @}
  */

/** @addtogroup XSPI_Exported_Functions_Group3
  * @{

This subsection provides a set of functions allowing to manage the data
  transfer from/to external memory

- HAL_XSPI_StartMemoryMappedMode() Allowing to start a XSPI Memory-Mapped mode according to the user parameters
- HAL_XSPI_StopMemoryMappedMode() Allowing to stop a XSPI Memory-Mapped mode

There are 3 categories of HAL functions APIs to manage the data transfer

- Blocking mode: Polling
  - HAL_XSPI_SendRegularCmd()
  - HAL_XSPI_SendHyperbusCmd()
  - HAL_XSPI_ExecRegularAutoPoll()
  - HAL_XSPI_Transmit()
  - HAL_XSPI_Receive()
  - HAL_XSPI_Abort()

- Non-Blocking mode: IT
 - HAL_XSPI_SendRegularCmd_IT()
 - HAL_XSPI_ExecRegularAutoPoll_IT()
 - HAL_XSPI_Transmit_IT()
 - HAL_XSPI_Receive_IT()
 - HAL_XSPI_Abort_IT()

- Non-Blocking mode: DMA
 - HAL_XSPI_Transmit_DMA()
 - HAL_XSPI_Receive_DMA()
 - HAL_XSPI_Transmit_DMA_Opt()
 - HAL_XSPI_Receive_DMA_Opt()
  */

/**
  * @brief  Start the Memory Mapped mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_config          Pointer to structure that contains the memory mapped configuration information.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            XSPI instance has been correctly configured.
  */
hal_status_t HAL_XSPI_StartMemoryMappedMode(hal_xspi_handle_t *hxspi, const hal_xspi_memory_mapped_config_t *p_config)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_TIMEOUT_ACTIVATION(p_config->timeout_activation));
  ASSERT_DBG_PARAM(IS_XSPI_TIMEOUT_PERIOD(p_config->timeout_period_cycle));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE);

  /* Start the memory mapped mode. */
  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
  {
    if (p_config->timeout_activation == HAL_XSPI_TIMEOUT_ENABLE)
    {
      /* Configure LPTR register. */
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->LPTR, p_config->timeout_period_cycle);

      /* Clear Timeout flag. */
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TO);

      /* Enable interrupt on the timeout flag. */
      HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TO);
    }
    /* Set functional mode as memory-mapped. */
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, (XSPI_CR_TCEN | XSPI_CR_FMODE),
                     ((uint32_t)p_config->timeout_activation | XSPI_FUNCTIONAL_MODE_MEMORY_MAPPED));
  }
  else
  {
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Stop the Memory Mapped mode.
  * @param  hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                the handle information for the specified XSPI instance.
  * @note   This function is used only in Memory mapped Mode.
  * @retval HAL_ERROR An error has occurred.
  * @retval HAL_OK    XSPI instance has been correctly configured.
  */
hal_status_t HAL_XSPI_StopMemoryMappedMode(hal_xspi_handle_t *hxspi)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE);

  /* Abort the current XSPI operation if exist */
  status = XSPI_Abort(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE);

  if (status == HAL_OK)
  {
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
  }
  else
  {
    status = HAL_ERROR;
  }

  return status;
}
/**
  * @brief  Set the Regular command configuration.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  p_cmd       Structure that contains the Regular command configuration information.
  * @param  timeout_ms  Timeout duration.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       In case of user timeout.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_SendRegularCmd(hal_xspi_handle_t *hxspi,
                                     const hal_xspi_regular_cmd_t *p_cmd,
                                     uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_cmd != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_OPERATION_TYPE(p_cmd->operation_type));
  ASSERT_DBG_PARAM(IS_XSPI_IO_SELECT(XSPI_GET_INSTANCE(hxspi), p_cmd->io_select));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_MODE(p_cmd->instruction_mode));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_WIDTH(p_cmd->instruction_width));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_DTR_MODE(p_cmd->instruction_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_MODE(p_cmd->addr_mode));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_WIDTH(p_cmd->addr_width));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_DTR_MODE(p_cmd->addr_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_MODE(p_cmd->alternate_bytes_mode));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_WIDTH(p_cmd->alternate_bytes_width));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_DTR_MODE(p_cmd->alternate_bytes_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_REGULAR_DATA_MODE(XSPI_GET_INSTANCE(hxspi), p_cmd->data_mode));
  ASSERT_DBG_PARAM(IS_XSPI_DATA_DTR_MODE(p_cmd->data_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_DUMMY_CYCLES(p_cmd->dummy_cycle));
  ASSERT_DBG_PARAM(IS_XSPI_DQS_MODE(p_cmd->dqs_mode_status));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  if ((p_cmd == NULL) || (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS))
  {
    return HAL_INVALID_PARAM;
  }
#else
  if (p_cmd == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_CHECK_PARAM */

  /* Check Data Length only if data are inside command. */
  if (p_cmd->data_mode != HAL_XSPI_REGULAR_DATA_NONE)
  {
    ASSERT_DBG_PARAM(IS_XSPI_DATA_LENGTH(p_cmd->size_byte));
  }

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_CMD_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Send regular command in blocking mode. */
  status = XSPI_SendRegularCmd(hxspi, p_cmd, timeout_ms, XSPI_INTERRUPT_DISABLE);

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return status;
}

/**
  * @brief  Set the Regular command configuration in interrupt mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_cmd             Structure that contains the Regular command configuration information.
  * @note   This function is used only in Indirect Read or Write Modes.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_SendRegularCmd_IT(hal_xspi_handle_t *hxspi, const hal_xspi_regular_cmd_t *p_cmd)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_cmd != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_OPERATION_TYPE(p_cmd->operation_type));
  ASSERT_DBG_PARAM(IS_XSPI_IO_SELECT(XSPI_GET_INSTANCE(hxspi), p_cmd->io_select));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_MODE(p_cmd->instruction_mode));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_WIDTH(p_cmd->instruction_width));
  ASSERT_DBG_PARAM(IS_XSPI_INSTRUCTION_DTR_MODE(p_cmd->instruction_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_MODE(p_cmd->addr_mode));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_WIDTH(p_cmd->addr_width));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_DTR_MODE(p_cmd->addr_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_MODE(p_cmd->alternate_bytes_mode));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_WIDTH(p_cmd->alternate_bytes_width));
  ASSERT_DBG_PARAM(IS_XSPI_ALTERNATE_BYTES_DTR_MODE(p_cmd->alternate_bytes_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_REGULAR_DATA_MODE(XSPI_GET_INSTANCE(hxspi), p_cmd->data_mode));
  ASSERT_DBG_PARAM(IS_XSPI_DATA_DTR_MODE(p_cmd->data_dtr_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_DUMMY_CYCLES(p_cmd->dummy_cycle));
  ASSERT_DBG_PARAM(IS_XSPI_DQS_MODE(p_cmd->dqs_mode_status));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  if ((p_cmd == NULL)
      || (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
      || (p_cmd->operation_type != HAL_XSPI_OPERATION_COMMON_CFG))
  {
    return HAL_INVALID_PARAM;
  }
#else
  if ((p_cmd == NULL) || (p_cmd->operation_type != HAL_XSPI_OPERATION_COMMON_CFG))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_CHECK_PARAM */

  /* Check Data Length only if data are inside command. */
  if (p_cmd->data_mode != HAL_XSPI_REGULAR_DATA_NONE)
  {
    ASSERT_DBG_PARAM(IS_XSPI_DATA_LENGTH(p_cmd->size_byte));
  }

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_CMD_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Send regular command in non-blocking mode. */
  if (XSPI_SendRegularCmd(hxspi, p_cmd, XSPI_TIMEOUT_DEFAULT_VALUE, XSPI_INTERRUPT_ENABLE) != HAL_OK)
  {
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

#if defined(USE_HAL_XSPI_HYPERBUS) && (USE_HAL_XSPI_HYPERBUS == 1U)
#if defined(XSPI_DCR1_MTYP_2)
/**
  * @brief  Set the Hyperbus command configuration.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_cmd             Structure containing the Hyperbus command.
  * @param  timeout_ms        Timeout duration.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       In case of user timeout.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_SendHyperbusCmd(hal_xspi_handle_t *hxspi,
                                      const hal_xspi_hyperbus_cmd_t *p_cmd,
                                      uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_cmd != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_DATA_LENGTH(p_cmd->size_byte));
  ASSERT_DBG_PARAM(IS_XSPI_ADDRESS_SPACE(p_cmd->addr_space));
  ASSERT_DBG_PARAM(IS_XSPI_ADDR_WIDTH(p_cmd->addr_width));
  ASSERT_DBG_PARAM(IS_XSPI_DQS_MODE(p_cmd->dqs_mode_status));
  ASSERT_DBG_PARAM(IS_XSPI_HYPERBUS_DATA_MODE(XSPI_GET_INSTANCE(hxspi), p_cmd->data_mode));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((p_cmd == NULL) || (hxspi->type != HAL_XSPI_MEMORY_TYPE_HYPERBUS))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_CMD_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Send hyperbus command in blocking mode. */
  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     timeout_ms) == HAL_OK)
  {
    /* Re-initialize the value of the functional mode. */
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, 0U);

    /* Configure the address space. */
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->DCR1,  XSPI_DCR1_MTYP_0, (uint32_t)p_cmd->addr_space);

    /* Set the following configurations :
    - address size
    - DQS signal enabled (used as RWDS)
    - DTR mode enabled on address and data
    - address and data */
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->CCR, ((uint32_t)p_cmd->dqs_mode_status | XSPI_CCR_DDTR |
                                                    (uint32_t)p_cmd->data_mode | (uint32_t)p_cmd->addr_width |
                                                    XSPI_CCR_ADDTR | XSPI_CCR_ADMODE_2));
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->WCCR, ((uint32_t)p_cmd->dqs_mode_status | XSPI_WCCR_DDTR |
                                                     (uint32_t)p_cmd->data_mode | (uint32_t)p_cmd->addr_width |
                                                     XSPI_WCCR_ADDTR | XSPI_WCCR_ADMODE_2));

    /* Configure the number of data. */
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->DLR, (p_cmd->size_byte - 1U));

    /* Configure the address value. */
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, p_cmd->addr);
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
    {
      hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
    }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    return HAL_TIMEOUT;
  }

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return HAL_OK;
}
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_XSPI_HYPERBUS */

/**
  * @brief  Execute the XSPI Automatic Polling Mode in blocking mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_config          Pointer to structure that contains the polling configuration information.
  * @param  timeout_ms        Timeout duration.
  * @note   This function is used only in Automatic Polling Mode for Regular protocol.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       In case of user timeout.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */

hal_status_t HAL_XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi,
                                          const hal_xspi_auto_polling_config_t *p_config,
                                          uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_STATUS_BYTES_SIZE(XSPI_GET_INSTANCE(hxspi)->DLR + 1U));
#if defined(XSPI_SR_SMF)
  ASSERT_DBG_PARAM(IS_XSPI_MATCH_MODE(p_config->match_mode));
#endif /* XSPI_SR_SMF */
  ASSERT_DBG_PARAM(IS_XSPI_INTERVAL(p_config->interval_cycle));
  ASSERT_DBG_PARAM(IS_XSPI_AUTOMATIC_STOP(p_config->automatic_stop_status));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
#if defined(XSPI_DCR1_MTYP_2)
  if ((p_config == NULL)
      || (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
      || (p_config->automatic_stop_status != HAL_XSPI_AUTOMATIC_STOP_ENABLED))
  {
    return HAL_INVALID_PARAM;
  }
#else
  if ((p_config == NULL)
      || (p_config->automatic_stop_status != HAL_XSPI_AUTOMATIC_STOP_ENABLED))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* XSPI_DCR1_MTYP_2 */
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_AUTO_POLLING_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Execute regular auto-polling in blocking mode */
#if defined(XSPI_SR_SMF)
  status = XSPI_ExecRegularAutoPoll(hxspi, p_config, timeout_ms, XSPI_INTERRUPT_DISABLE);
#else
  status = XSPI_ExecRegularAutoPoll(hxspi, p_config, timeout_ms);
#endif /* XSPI_SR_SMF */

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return status;
}

#if defined(XSPI_SR_SMF)
/**
  * @brief  Execute the XSPI Automatic Polling Mode in non-blocking mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_config          Pointer to structure that contains the polling configuration information
  * @note   This function is used only in Automatic Polling Mode for Regular protocol.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_ExecRegularAutoPoll_IT(hal_xspi_handle_t *hxspi, const hal_xspi_auto_polling_config_t *p_config)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_STATUS_BYTES_SIZE(XSPI_GET_INSTANCE(hxspi)->DLR + 1U));
#if defined(XSPI_SR_SMF)
  ASSERT_DBG_PARAM(IS_XSPI_MATCH_MODE(p_config->match_mode));
#endif /* XSPI_SR_SMF */
  ASSERT_DBG_PARAM(IS_XSPI_INTERVAL(p_config->interval_cycle));
  ASSERT_DBG_PARAM(IS_XSPI_AUTOMATIC_STOP(p_config->automatic_stop_status));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((p_config == NULL) || (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_AUTO_POLLING_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Execute regular auto-polling in non-blocking mode */
  if (XSPI_ExecRegularAutoPoll(hxspi, p_config, XSPI_TIMEOUT_DEFAULT_VALUE,
                               XSPI_INTERRUPT_ENABLE) != HAL_OK)
  {
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* XSPI_SR_SMF */

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_data            Pointer to data buffer.
  * @param  timeout_ms        Timeout duration.
  * @note   This function is used only in Indirect Write Mode.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       In case of user timeout.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Transfer completed.
  */
hal_status_t HAL_XSPI_Transmit(hal_xspi_handle_t *hxspi, const void *p_data, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  volatile uint32_t *p_data_reg = &XSPI_GET_INSTANCE(hxspi)->DR;

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_TX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Configure counters and size. */
  hxspi->xfer_count = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DLR) + 1U;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Configure the functional mode as indirect write. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

  /* Repeat for all data. */
  do
  {
    /* Wait until the FIFO threshold flag is set to send data. */
    status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_FT, HAL_XSPI_FLAG_ACTIVE,
                                            timeout_ms);

    if (status != HAL_OK)
    {
      break;
    }

    *((volatile uint8_t *)p_data_reg) = *hxspi->p_buffer;
    hxspi->p_buffer++;
    hxspi->xfer_count--;

  } while (hxspi->xfer_count > 0U);

  if (status == HAL_OK)
  {
    /* Wait till transfer complete flag is set to go back in idle state. */
    status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_TC, HAL_XSPI_FLAG_ACTIVE,
                                            timeout_ms);

    if (status == HAL_OK)
    {
      /* Clear transfer complete flag. */
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);
    }
  }

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return status;
}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_data            Pointer to data buffer.
  * @param  timeout_ms        Timeout duration.
  * @note   This function is used only in Indirect Read Mode.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_TIMEOUT       In case of user timeout.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Receive(hal_xspi_handle_t *hxspi, void *p_data, uint32_t timeout_ms)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

  volatile uint32_t *p_data_reg = &XSPI_GET_INSTANCE(hxspi)->DR;
  uint32_t addr_reg             = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg               = XSPI_GET_INSTANCE(hxspi)->IR;

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_RX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Configure counters and size. */
  hxspi->xfer_count = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DLR) + 1U;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Configure the functional mode as indirect read. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

  /* Trig the transfer by re-writing address or instruction register. */
#if defined(XSPI_DCR1_MTYP_2)
  if (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
  }
  else
  {
    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
    }
  }
#else
  if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
  }
  else
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
  }
#endif /* XSPI_DCR1_MTYP_2 */

  do
  {
    /* Wait until the FIFO threshold or transfer complete flags are set to read received data. */
    status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_FT | (uint32_t)HAL_XSPI_FLAG_TC,
                                            HAL_XSPI_FLAG_ACTIVE,  timeout_ms);
    if (status != HAL_OK)
    {
      break;
    }

    *hxspi->p_buffer = *((volatile uint8_t *)p_data_reg);
    hxspi->p_buffer++;
    hxspi->xfer_count--;

  } while (hxspi->xfer_count > 0U);

  if (status == HAL_OK)
  {
    /* Wait till transfer complete flag is set to go back in idle state. */
    status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_TC, HAL_XSPI_FLAG_ACTIVE,
                                            timeout_ms);

    if (status == HAL_OK)
    {
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);
    }
  }

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return status;
}

/**
  * @brief  Send an amount of data in non-blocking mode with interrupt.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_data            Pointer to data buffer.
  * @note   This function is used only in Indirect Write Mode.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Transfer completed.
  */
hal_status_t HAL_XSPI_Transmit_IT(hal_xspi_handle_t *hxspi, const void *p_data)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_TX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  /* Store counters and size. */
  hxspi->xfer_count = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DLR) + 1U;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect write. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Enable the transfer complete, FIFO threshold, and transfer error interrupts. */
  HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC | (uint32_t)HAL_XSPI_IT_FT | (uint32_t)HAL_XSPI_IT_TE);

  return HAL_OK;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with interrupt.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_data            Pointer to data buffer.
  * @note   This function is used only in Indirect Read Mode.
  * @retval HAL_ERROR         An error has occurred.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          XSPI state is active when calling this API.
  * @retval HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Receive_IT(hal_xspi_handle_t *hxspi, void *p_data)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_RX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  uint32_t addr_reg = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg = XSPI_GET_INSTANCE(hxspi)->IR;

  /* Store counters and size. */
  hxspi->xfer_count = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DLR) + 1U;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect read. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Enable the transfer complete, FIFO threshold, and transfer error interrupts. */
  HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC | (uint32_t)HAL_XSPI_IT_FT | (uint32_t)HAL_XSPI_IT_TE);

  /* Trig the transfer by re-writing address or instruction register. */
#if defined(XSPI_DCR1_MTYP_2)
  if (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
  }
  else
  {
    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
    }
  }
#else
  if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
  }
  else
  {
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
  }
#endif /* XSPI_DCR1_MTYP_2 */

  return HAL_OK;
}

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
/**
  * @brief   Send an amount of data in non-blocking mode with DMA.
  * @param   hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                 the handle information for the specified XSPI instance.
  * @param   p_data Pointer to data buffer.
  * @note    This function is used only in Indirect Write Mode.
  * @warning If DMA peripheral access is configured as halfword, the number of data and the FIFO threshold must be
  *          aligned on halfword.
  * @warning If DMA peripheral access is configured as word, the number
  *          of data and the FIFO threshold must be aligned on word.
  * @retval  HAL_ERROR         An error has occurred.
  * @retval  HAL_INVALID_PARAM Invalid parameter.
  * @retval  HAL_BUSY          XSPI state is active when calling this API.
  * @retval  HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Transmit_DMA(hal_xspi_handle_t *hxspi, const void *p_data)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_TX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  uint32_t size_byte = XSPI_GET_INSTANCE(hxspi)->DLR + 1U;

  /* Configure counters and size. */
  hxspi->xfer_count = size_byte;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect write. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Set the DMA transfer complete callback. */
  hxspi->hdma_tx->p_xfer_cplt_cb = XSPI_DMACplt;

  /* Set the DMA Half transfer complete callback. */
  hxspi->hdma_tx->p_xfer_halfcplt_cb = XSPI_DMAHalfCplt;

  /* Set the DMA error callback. */
  hxspi->hdma_tx->p_xfer_error_cb = XSPI_DMAError;

  /* Start DMA peripheral. */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hxspi->hdma_tx,
                                     (uint32_t)p_data,
                                     (uint32_t)&XSPI_GET_INSTANCE(hxspi)->DR,
                                     hxspi->xfer_size,
                                     HAL_DMA_OPT_IT_DEFAULT) == HAL_OK)
  {
    /* Enable the transfer error interrupt. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TE);

    /* Enable the DMA transfer. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes = HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief   Send an amount of data with DMA in interrupt mode with optional interrupts.
  * @param   hxspi      Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param   p_data     Pointer to data buffer.
  * @param   interrupts Specifies the DMA optional interrupt to be enabled.
  *                     This parameter can be one of @ref XSPI_Optional_Interrupt group.
  * @note    This function is used only in Indirect Write Mode.
  * @warning If DMA peripheral access is configured as halfword, the number of data and the FIFO threshold must be
  *          aligned on halfword.
  * @warning If DMA peripheral access is configured as word, the number
  *          of data and the FIFO threshold must be aligned on word.
  * @retval  HAL_ERROR         An error has occurred.
  * @retval  HAL_INVALID_PARAM Invalid parameter.
  * @retval  HAL_BUSY          XSPI state is active when calling this API.
  * @retval  HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Transmit_DMA_Opt(hal_xspi_handle_t *hxspi, const void *p_data, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_OPT_IT(interrupts));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_TX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  uint32_t size_byte = XSPI_GET_INSTANCE(hxspi)->DLR + 1U;

  /* Store counters and size. */
  hxspi->xfer_count = size_byte;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect write. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Set the DMA transfer complete callback. */
  hxspi->hdma_tx->p_xfer_cplt_cb = XSPI_DMACplt;

  if ((interrupts & HAL_XSPI_OPT_IT_HT) != 0U)
  {
    /* Set the DMA Half transfer complete callback. */
    hxspi->hdma_tx->p_xfer_halfcplt_cb = XSPI_DMAHalfCplt;
  }

  /* Set the DMA error callback. */
  hxspi->hdma_tx->p_xfer_error_cb = XSPI_DMAError;

  /* Start DMA peripheral. */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hxspi->hdma_tx,
                                     (uint32_t)p_data,
                                     (uint32_t)&XSPI_GET_INSTANCE(hxspi)->DR,
                                     hxspi->xfer_size,
                                     interrupts) == HAL_OK)
  {
    /* Enable the transfer error interrupt. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TE);

    /* Enable the DMA transfer. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes = HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief   Receive an amount of data in non-blocking mode with DMA.
  * @param   hxspi  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                 the handle information for the specified XSPI instance.
  * @param   p_data Pointer to data buffer.
  * @note    This function is used only in Indirect Read Mode.
  * @warning If DMA peripheral access is configured as halfword, the number
  *          of data and the FIFO threshold must be aligned on halfword.
  * @warning If DMA peripheral access is configured as word, the number
  *          of data and the FIFO threshold must be aligned on word.
  * @retval  HAL_ERROR         An error has occurred.
  * @retval  HAL_INVALID_PARAM Invalid parameter.
  * @retval  HAL_BUSY          XSPI state is active when calling this API.
  * @retval  HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Receive_DMA(hal_xspi_handle_t *hxspi, void *p_data)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_RX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  uint32_t size_byte = XSPI_GET_INSTANCE(hxspi)->DLR + 1U;
  uint32_t addr_reg = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg = XSPI_GET_INSTANCE(hxspi)->IR;

  /* Set counters and size. */
  hxspi->xfer_count = size_byte;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect read. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Set the DMA transfer complete callback. */
  hxspi->hdma_rx->p_xfer_cplt_cb = XSPI_DMACplt;

  /* Set the DMA Half transfer complete callback. */
  hxspi->hdma_rx->p_xfer_halfcplt_cb = XSPI_DMAHalfCplt;

  /* Set the DMA error callback. */
  hxspi->hdma_rx->p_xfer_error_cb = XSPI_DMAError;

  /* Start DMA peripheral. */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hxspi->hdma_rx,
                                     (uint32_t)&XSPI_GET_INSTANCE(hxspi)->DR,
                                     (uint32_t)p_data,
                                     hxspi->xfer_size,
                                     HAL_DMA_OPT_IT_DEFAULT) == HAL_OK)
  {
    /* Enable the transfer error interrupt. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TE);

    /* Trig the transfer by re-writing address or instruction register. */
#if defined(XSPI_DCR1_MTYP_2)
    if (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
      }
      else
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
      }
    }
#else
    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
    }
#endif /* XSPI_DCR1_MTYP_2 */

    /* Enable the DMA transfer. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes = HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief   Receive an amount of data with DMA in interrupt mode with optional interrupts.
  * @param   hxspi      Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param   p_data     Pointer to data buffer.
  * @param   interrupts Specifies the DMA optional interrupt to be enabled.
  *                     This parameter can be one of @ref XSPI_Optional_Interrupt group.
  * @note    This function is used only in Indirect Read Mode.
  * @warning If DMA peripheral access is configured as halfword, the number
  *          of data and the FIFO threshold must be aligned on halfword.
  * @warning If DMA peripheral access is configured as word, the number
  *          of data and the FIFO threshold must be aligned on word.
  * @retval  HAL_ERROR         An error has occurred.
  * @retval  HAL_INVALID_PARAM Invalid parameter.
  * @retval  HAL_BUSY          XSPI state is active when calling this API.
  * @retval  HAL_OK            Operation completed.
  */
hal_status_t HAL_XSPI_Receive_DMA_Opt(hal_xspi_handle_t *hxspi, void *p_data, uint32_t interrupts)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_OPT_IT(interrupts));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hxspi, global_state, HAL_XSPI_STATE_IDLE, HAL_XSPI_STATE_RX_ACTIVE);

#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
  hxspi->last_error_codes = HAL_XSPI_ERROR_NONE;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

  uint32_t size_byte = XSPI_GET_INSTANCE(hxspi)->DLR + 1U;
  uint32_t addr_reg = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg = XSPI_GET_INSTANCE(hxspi)->IR;

  /* Set counters and size. */
  hxspi->xfer_count = size_byte;
  hxspi->xfer_size  = hxspi->xfer_count;
  hxspi->p_buffer   = (uint8_t *)((uint32_t)p_data);

  /* Set functional mode as indirect read. */
  STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

  /* Clear flags related to interrupt. */
  HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

  /* Set the DMA transfer complete callback. */
  hxspi->hdma_rx->p_xfer_cplt_cb = XSPI_DMACplt;

  if ((interrupts & HAL_XSPI_OPT_IT_HT) != 0U)
  {
    /* Set the DMA Half transfer complete callback. */
    hxspi->hdma_rx->p_xfer_halfcplt_cb = XSPI_DMAHalfCplt;
  }

  /* Set the DMA error callback. */
  hxspi->hdma_rx->p_xfer_error_cb = XSPI_DMAError;

  /* Start DMA peripheral. */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hxspi->hdma_rx,
                                     (uint32_t)&XSPI_GET_INSTANCE(hxspi)->DR,
                                     (uint32_t)p_data,
                                     hxspi->xfer_size,
                                     interrupts) == HAL_OK)
  {
    /* Enable the transfer error interrupt. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TE);

    /* Trig the transfer by re-writing address or instruction register. */
#if defined(XSPI_DCR1_MTYP_2)
    if (hxspi->type == HAL_XSPI_MEMORY_TYPE_HYPERBUS)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
      }
      else
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
      }
    }
#else
    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
    }
#endif /* XSPI_DCR1_MTYP_2 */

    /* Enable the DMA transfer. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes = HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
    hxspi->global_state = HAL_XSPI_STATE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}
#endif /* USE_HAL_XSPI_DMA */

/**
  * @brief  Abort the current transmission.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  timeout_ms  Timeout duration.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_OK      Operation completed.
  */
hal_status_t HAL_XSPI_Abort(hal_xspi_handle_t *hxspi, uint32_t timeout_ms)
{
  hal_status_t status ;

  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state,
                   HAL_XSPI_STATE_IDLE | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE |
                   HAL_XSPI_STATE_CMD_ACTIVE | HAL_XSPI_STATE_AUTO_POLLING_ACTIVE |
                   HAL_XSPI_STATE_TX_ACTIVE | HAL_XSPI_STATE_RX_ACTIVE);

  hxspi->global_state = HAL_XSPI_STATE_ABORT;

  status = XSPI_Abort(hxspi, timeout_ms);

  /* Return to indirect mode. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE);

  hxspi->global_state = HAL_XSPI_STATE_IDLE;

  return status;
}

/**
  * @brief  Abort the current transmission (non-blocking function).
  * @param  hxspi     Pointer to a \ref hal_xspi_handle_t structure that contains
  *                   the handle information for the specified XSPI instance.
  * @retval HAL_ERROR An error has occurred.
  * @retval HAL_OK    Operation completed.
  */
hal_status_t HAL_XSPI_Abort_IT(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_STATE(hxspi->global_state,
                   HAL_XSPI_STATE_IDLE | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE |
                   HAL_XSPI_STATE_CMD_ACTIVE | HAL_XSPI_STATE_AUTO_POLLING_ACTIVE |
                   HAL_XSPI_STATE_TX_ACTIVE | HAL_XSPI_STATE_RX_ACTIVE);

  /* Disable all interrupts. */
  HAL_XSPI_DisableIT(hxspi, (uint32_t)HAL_XSPI_IT_ALL);

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
  if ((XSPI_GET_INSTANCE(hxspi)->CR & XSPI_CR_DMAEN) != 0U)
  {
    /* Disable the DMA transfer on the XSPI side. */
    STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);

    if (hxspi->global_state == HAL_XSPI_STATE_TX_ACTIVE)
    {
      hxspi->global_state = HAL_XSPI_STATE_ABORT;

      /* Disable the DMA transmit on the DMA side. */
      hxspi->hdma_tx->p_xfer_abort_cb = XSPI_DMAAbort;
      (void)HAL_DMA_Abort_IT(hxspi->hdma_tx);
    }
    else if (hxspi->global_state == HAL_XSPI_STATE_RX_ACTIVE)
    {
      hxspi->global_state = HAL_XSPI_STATE_ABORT;

      /* Disable the DMA receive on the DMA side. */
      hxspi->hdma_rx->p_xfer_abort_cb = XSPI_DMAAbort;
      (void)HAL_DMA_Abort_IT(hxspi->hdma_rx);
    }
    else
    {
      return HAL_OK;
    }
  }
  else
#endif /* USE_HAL_XSPI_DMA */
  {
    if (HAL_XSPI_IsActiveFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY) != HAL_XSPI_FLAG_NOT_ACTIVE)
    {
      hxspi->global_state = HAL_XSPI_STATE_ABORT;

      /* Clear transfer complete flag. */
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);

      /* Enable the transfer complete interrupts. */
      HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC);

      /* Perform an abort of the XSPI. */
      STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_ABORT);

      /* Return to indirect mode. */
      STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE);
    }
    else
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
/**
  * @}
  */

/** @addtogroup XSPI_Exported_Functions_Group4
  * @{

This subsection provides a set callback functions allowing to manage the data
  transfer from/to external memory
  */

/**
  * @brief  Handle the XSPI interrupt request.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  */
void HAL_XSPI_IRQHandler(hal_xspi_handle_t *hxspi)
{
  volatile uint32_t *p_data_reg = &XSPI_GET_INSTANCE(hxspi)->DR;
  uint32_t it_flag              = XSPI_GET_INSTANCE(hxspi)->SR;
  uint32_t it_source            = XSPI_GET_INSTANCE(hxspi)->CR;
  uint32_t it_active            = it_flag & (it_source >> XSPI_CR_TEIE_Pos);
  uint32_t threshold            = hxspi->fifo_threshold;
  hal_xspi_state_t state        = hxspi->global_state;

  /* XSPI FIFO threshold interrupt occurred --------------------------------------------------------------------------*/
  if ((it_active & (uint32_t)HAL_XSPI_FLAG_FT) != 0U)
  {
    if (state == HAL_XSPI_STATE_RX_ACTIVE)
    {
      while (threshold > 0U)
      {
        *hxspi->p_buffer = *((volatile uint8_t *)p_data_reg);
        hxspi->p_buffer++;
        hxspi->xfer_count--;
        threshold--;
      }
    }

    if (state == HAL_XSPI_STATE_TX_ACTIVE)
    {
      while (threshold > 0U)
      {
        *((volatile uint8_t *)p_data_reg) = *hxspi->p_buffer;
        hxspi->p_buffer++;
        hxspi->xfer_count--;
        threshold--;
      }
    }

    /* All data have been received or transmitted for the transfer */
    if (hxspi->xfer_count == 0U)
    {
      /* Disable the interrupt on the FIFO threshold flag. */
      HAL_XSPI_DisableIT(hxspi, (uint32_t)HAL_XSPI_IT_FT);
    }

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_fifo_threshold_cb(hxspi);
#else
    HAL_XSPI_FifoThresholdCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }

  /* XSPI transfer complete interrupt occurred -----------------------------------------------------------------------*/
  if ((it_active & (uint32_t)HAL_XSPI_FLAG_TC) != 0U)
  {
    /* Clear transfer complete flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);

    /* Disable the interrupts on the FIFO threshold and the transfer complete flags. */
    HAL_XSPI_DisableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC | (uint32_t)HAL_XSPI_IT_FT | (uint32_t)HAL_XSPI_IT_TE);

    if (state == HAL_XSPI_STATE_RX_ACTIVE)
    {
      uint32_t Fifo = STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->SR, XSPI_SR_FLEVEL) >>
                      XSPI_SR_FLEVEL_Pos;
      if ((hxspi->xfer_count > 0U) && (Fifo != 0U))
      {
        while (hxspi->xfer_count != 0U)
        {
          /* Read the last data received in the FIFO. */
          *hxspi->p_buffer = *((volatile uint8_t *)p_data_reg);
          hxspi->p_buffer++;
          hxspi->xfer_count--;
        }
      }
      hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
      hxspi->p_rx_cplt_cb(hxspi);
#else
      HAL_XSPI_RxCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
    }
    else
    {
      if (state == HAL_XSPI_STATE_TX_ACTIVE)
      {
        hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
        hxspi->p_tx_cplt_cb(hxspi);
#else
        HAL_XSPI_TxCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
      }

      if (state == HAL_XSPI_STATE_CMD_ACTIVE)
      {
        hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
        hxspi->p_cmd_cplt_cb(hxspi);
#else
        HAL_XSPI_CmdCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
      }

      if (state == HAL_XSPI_STATE_ABORT)
      {
        hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
        if (hxspi->is_dma_error == 1U)
        {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
          hxspi->last_error_codes |= HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

          /* Abort due to an error (eg : DMA error) */
#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
          hxspi->p_error_cb(hxspi);
#else
          HAL_XSPI_ErrorCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
        }
        else
#endif /* USE_HAL_XSPI_DMA */
        {
#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
          hxspi->p_abort_cplt_cb(hxspi);
#else
          HAL_XSPI_AbortCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
        }
      }
    }
  }

#if defined(XSPI_SR_SMF)
  /* XSPI status match interrupt occurred ----------------------------------------------------------------------------*/
  if ((it_active & (uint32_t)HAL_XSPI_FLAG_SM) != 0U)
  {
    /* Clear status match flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_SM);

    /* Check if automatic poll mode stop is activated. */
    if ((XSPI_GET_INSTANCE(hxspi)->CR & XSPI_CR_APMS) != 0U)
    {
      /* Disable the interrupts on the status match and the transfer error flags. */
      HAL_XSPI_DisableIT(hxspi, ((uint32_t)HAL_XSPI_IT_SM | (uint32_t)HAL_XSPI_IT_TE));
      hxspi->global_state = HAL_XSPI_STATE_IDLE;
    }

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_status_match_cb(hxspi);
#else
    HAL_XSPI_StatusMatchCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }
#endif /* XSPI_SR_SMF */

  /* XSPI transfer error interrupt occurred --------------------------------------------------------------------------*/
  if ((it_active & (uint32_t)HAL_XSPI_FLAG_TE) != 0U)
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes |= HAL_XSPI_ERROR_TRANSFER;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    /* Clear Transfer error flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE);

    /* Disable all interrupts. */
    HAL_XSPI_DisableIT(hxspi, (uint32_t)HAL_XSPI_IT_ALL);

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
    if ((XSPI_GET_INSTANCE(hxspi)->CR & XSPI_CR_DMAEN) != 0U)
    {
      /* Disable the DMA transfer on the XSPI side. */
      STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);

      hxspi->is_dma_error = 1U;

      if (state == HAL_XSPI_STATE_TX_ACTIVE)
      {
        /* Disable the DMA transmit on the DMA side. */
        hxspi->hdma_tx->p_xfer_abort_cb = XSPI_DMAAbortOnError;

        /* Abort the DMA channel. */
        (void)HAL_DMA_Abort_IT(hxspi->hdma_tx);
      }

      if (state == HAL_XSPI_STATE_RX_ACTIVE)
      {
        /* Disable the DMA receive on the DMA side. */
        hxspi->hdma_rx->p_xfer_abort_cb = XSPI_DMAAbortOnError;

        /* Abort the DMA channel. */
        (void)HAL_DMA_Abort_IT(hxspi->hdma_rx);
      }
    }
    else
#endif /* USE_HAL_XSPI_DMA */
    {
      hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
      hxspi->p_error_cb(hxspi);
#else
      HAL_XSPI_ErrorCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
    }
  }

  /* XSPI timeout interrupt occurred ---------------------------------------------------------------------------------*/
  if ((it_active & (uint32_t)HAL_XSPI_FLAG_TO) != 0U)
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes |= HAL_XSPI_ERROR_TIMEOUT;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    /* Clear timeout flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TO);

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_error_cb(hxspi);
#else
    HAL_XSPI_ErrorCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }
}

/**
  * @brief Error callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_ErrorCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning. */
  STM32_UNUSED(hxspi);
  /* NOTE : This function must not be modified, when the callback is needed,
  the HAL_XSPI_ErrorCallback could be implemented in the user file
  */
}

/**
  * @brief Abort completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_AbortCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning. */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_AbortCpltCallback could be implemented in the user file
  */
}

/**
  * @brief FIFO Threshold callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_FifoThresholdCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE : This function must not be modified, when the callback is needed,
  the HAL_XSPI_FIFOThresholdCallback could be implemented in the user file
  */
}

/**
  * @brief Command completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_CmdCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_CmdCpltCallback could be implemented in the user file
  */
}

/**
  * @brief Rx Transfer completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_RxCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_RxCpltCallback could be implemented in the user file
  */
}

/**
  * @brief Tx Transfer completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_TxCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_TxCpltCallback could be implemented in the user file
  */
}

/**
  * @brief Rx Half Transfer completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_RxHalfCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_RxHalfCpltCallback could be implemented in the user file
  */
}

/**
  * @brief Tx Half Transfer completed callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_TxHalfCpltCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE: This function must not be modified, when the callback is needed,
  the HAL_XSPI_TxHalfCpltCallback could be implemented in the user file
  */
}

#if defined(XSPI_SR_SMF)
/**
  * @brief Status Match callback.
  * @param hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *              the handle information for the specified XSPI instance.
  */
__WEAK void HAL_XSPI_StatusMatchCallback(hal_xspi_handle_t *hxspi)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hxspi);

  /* NOTE : This function must not be modified, when the callback is needed,
  the HAL_XSPI_StatusMatchCallback could be implemented in the user file
  */
}
#endif /* XSPI_SR_SMF */

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register the XSPI Error Callback to be used instead of
  *         the weak HAL_XSPI_ErrorCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the error callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterErrorCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI command complete Callback TO be used instead of
  *         the weak HAL_XSPI_CmdCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the command complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterCmdCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_cmd_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI Receive complete Callback TO be used instead of
  *         the weak HAL_XSPI_RxCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback          Specifies the receive complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterRxCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI Transfer complete Callback TO be used instead of
  *         the weak HAL_XSPI_TxCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the transfer complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterTxCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI Receive Half complete Callback TO be used instead of
  *         the weak HAL_XSPI_RxHalfCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the half receive complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterRxHalfCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_rx_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI Transfer Half complete Callback TO be used instead of
  *         the weak HAL_XSPI_TxHalfCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the half transfer complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterTxHalfCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_tx_half_cplt_cb = p_callback;

  return HAL_OK;
}

#if defined(XSPI_SR_SMF)
/**
  * @brief  Register the XSPI Status Match Callback TO be used instead of
  *         the weak HAL_XSPI_StatusMatchCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the status match callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterStatusMatchCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_status_match_cb = p_callback;

  return HAL_OK;
}
#endif /* XSPI_SR_SMF */

/**
  * @brief  Register the XSPI Abort complete Callback TO be used instead of
  *         the weak HAL_XSPI_AbortCpltCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the abort complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterAbortCpltCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the XSPI FIFO threshold callback to be used instead of
  *         the weak HAL_XSPI_FifoThresholdCallback() predefined callback.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_callback        Specifies the FIFO threshold complete callback.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            Register completed successfully.
  */
hal_status_t HAL_XSPI_RegisterFifoThresholdCallback(hal_xspi_handle_t *hxspi, hal_xspi_cb_t  p_callback)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Store the callback function within handle */
  hxspi->p_fifo_threshold_cb = p_callback;

  return HAL_OK;
}
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */

#if defined (USE_HAL_XSPI_USER_DATA) && (USE_HAL_XSPI_USER_DATA == 1U)
/**
  * @brief  Store User Data pointer into the handle.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  p_user_data Pointer to the user data.
  */
void HAL_XSPI_SetUserData(hal_xspi_handle_t *hxspi, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  hxspi->p_user_data = p_user_data;
}
/**
  * @brief  Retrieve User Data pointer from the handle.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Pointer to the user data.
  */
const void *HAL_XSPI_GetUserData(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  return (hxspi->p_user_data);
}
#endif /* USE_HAL_XSPI_USER_DATA == 1U */

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
/**
  * @brief  link/store Tx HAL DMA handle into the HAL XSPI handle.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  hdma_tx           Pointer to a hal_dma_handle_t.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            The DMA Tx handle has been successfully linked and stored in the XSPI handle.
  */
hal_status_t HAL_XSPI_SetTxDMA(hal_xspi_handle_t *hxspi, hal_dma_handle_t *hdma_tx)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(hdma_tx != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_INIT | HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hdma_tx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hxspi->hdma_tx    = hdma_tx;
  hdma_tx->p_parent = hxspi;

  return HAL_OK;
}

/**
  * @brief  link/store Rx HAL DMA handle into the HAL XSPI handle.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  hdma_rx           Pointer to a hal_dma_handle_t.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_OK            The DMA Rx handle has been successfully linked and stored in the XSPI handle.
  */
hal_status_t HAL_XSPI_SetRxDMA(hal_xspi_handle_t *hxspi, hal_dma_handle_t *hdma_rx)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(hdma_rx != NULL);

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_INIT | HAL_XSPI_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (hdma_rx == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hxspi->hdma_rx    = hdma_rx;
  hdma_rx->p_parent = hxspi;

  return HAL_OK;
}
#endif /* USE_HAL_XSPI_DMA */
/**
  * @}
  */

/** @addtogroup XSPI_Exported_Functions_Group5
  * @{

This subsection provides a set of functions allowing to read peripheral
current frequency, state and last occurred errors.
  */

/** @brief  Return the peripheral clock frequency for XSPI.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @retval uint32_t Frequency in Hz.
  * @retval 0 source clock of the hxspi not configured or not ready.
  */
uint32_t HAL_XSPI_GetClockFreq(const hal_xspi_handle_t *hxspi)
{
  /* Check the XSPI handle & config allocation. */
  ASSERT_DBG_PARAM((hxspi != NULL));

  /* Check the global state, the driver must be at least configured. */
  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_INIT | HAL_XSPI_STATE_IDLE
                   | HAL_XSPI_STATE_CMD_ACTIVE | HAL_XSPI_STATE_AUTO_POLLING_ACTIVE
                   | HAL_XSPI_STATE_TX_ACTIVE | HAL_XSPI_STATE_RX_ACTIVE
                   | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE | HAL_XSPI_STATE_ABORT);

  return HAL_RCC_XSPI_GetKernelClkFreq((XSPI_TypeDef *)((uint32_t)(hxspi->instance)));
}

/**
  * @brief  Retrieve the HAL XSPI Global State.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @return Retrieve the XSPI global state.
  */
hal_xspi_state_t HAL_XSPI_GetState(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  return hxspi->global_state;
}

#if defined (USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
/**
  * @brief  Return the XSPI error code.
  * @param  hxspi Pointer to a \ref hal_xspi_handle_t structure that contains
  *               the handle information for the specified XSPI instance.
  * @retval XSPI error code
  */
uint32_t HAL_XSPI_GetLastErrorCodes(const hal_xspi_handle_t *hxspi)
{
  return hxspi->last_error_codes;
}
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
/**
  * @}
  */

/** @addtogroup XSPI_Exported_Functions_Group6
  * @{

This subsection provides a set of functions allowing to configure the Delay Block :

- Call the function HAL_XSPI_DLYB_SetConfigDelay() to set the delay configuration of the delay block peripheral
- Call the function HAL_XSPI_DLYB_GetConfigDelay() to get the delay output clock phase of the delay block peripheral
- Call the function HAL_XSPI_DLYB_CalculateMaxClockPhase() to calculate the maximum output clock phase of the
  delay block peripheral
- Call the function HAL_XSPI_DLYB_Enable() to enable the delay block peripheral
- Call the function HAL_XSPI_DLYB_Disable() to disable the delay block peripheral
- Call the function HAL_XSPI_DLYB_IsEnabled() to check if the delay block peripheral is enabled or not
  */

/**
  * @brief  Set the delay configuration of the delay block peripheral.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  clock_phase_value The desired output clock phase value.
  * @retval HAL_INVALID_PARAM When no valid XSPI.
  * @retval HAL_OK            The delay is correctly configured.
  */
hal_status_t HAL_XSPI_DLYB_SetConfigDelay(hal_xspi_handle_t *hxspi, uint32_t clock_phase_value)
{
  hal_status_t status = HAL_ERROR;
  DLYB_TypeDef *dlyb_instance;
  dlyb_state_t state;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)));
  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)) == 0U)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  dlyb_instance = XSPI_DLYB_GET_INSTANCE(hxspi->instance);

  state = DLYB_IsEnabled(dlyb_instance);
  DLYB_Enable(dlyb_instance);

  /* Enable XSPI Free Running Clock (mandatory). */
  STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  if (DLYB_ConfigureUnitDelay(dlyb_instance) == DLYB_CORE_OK)
  {
    DLYB_SetOutputClockPhase(dlyb_instance, clock_phase_value);
    status = HAL_OK;
  }

  (void)XSPI_Abort(hxspi, XSPI_TIMEOUT_DEFAULT_VALUE);

  /* Disable Free Running Clock. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  if (state == DLYB_DISABLED)
  {
    DLYB_Disable(dlyb_instance);
  }

  return status;
}

/**
  * @brief  Get the delay output clock phase of the delay block peripheral.
  * @param  hxspi              Pointer to a \ref hal_xspi_handle_t structure that contains
  *                            the handle information for the specified XSPI instance.
  * @param  p_clock_phase      Pointer to the variable where the selected output clock phase value will be stored.
  * @retval HAL_INVALID_PARAM When no valid XSPI.
  * @retval HAL_OK            When the register reading was successful.
  */
hal_status_t HAL_XSPI_DLYB_GetConfigDelay(const hal_xspi_handle_t *hxspi, uint32_t *p_clock_phase)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)));
  ASSERT_DBG_PARAM(p_clock_phase != NULL);
  ASSERT_DBG_STATE(hxspi->global_state,
                   HAL_XSPI_STATE_IDLE | HAL_XSPI_STATE_CMD_ACTIVE
                   | HAL_XSPI_STATE_AUTO_POLLING_ACTIVE | HAL_XSPI_STATE_TX_ACTIVE
                   | HAL_XSPI_STATE_RX_ACTIVE | HAL_XSPI_STATE_MEMORY_MAPPED_ACTIVE
                   | HAL_XSPI_STATE_ABORT);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)) == 0U) || (p_clock_phase == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  *p_clock_phase = DLYB_GetOutputClockPhase(XSPI_DLYB_GET_INSTANCE(hxspi->instance));

  return HAL_OK;
}

/**
  * @brief  Calculate maximum output clock phase of the delay block peripheral.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @param  p_max_clock_phase Pointer to the variable where the maximum clock phase value will be stored.
  * @retval HAL_ERROR         The max clock phase is not correctly calculated.
  * @retval HAL_INVALID_PARAM When no valid XSPI or invalid p_max_clock_phase parameter.
  * @retval HAL_OK            The max clock phase is correctly calculated.
 */
hal_status_t HAL_XSPI_DLYB_CalculateMaxClockPhase(hal_xspi_handle_t *hxspi, uint32_t *p_max_clock_phase)
{
  hal_status_t status = HAL_ERROR;
  DLYB_TypeDef *dlyb_instance;
  uint32_t sel;
  uint32_t unit;
  dlyb_state_t state;

  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)));
  ASSERT_DBG_PARAM(p_max_clock_phase != NULL);
  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if ((IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)) == 0U) || (p_max_clock_phase == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  dlyb_instance = XSPI_DLYB_GET_INSTANCE(hxspi->instance);

  state = DLYB_IsEnabled(dlyb_instance);
  DLYB_Enable(dlyb_instance);

  /* Enable XSPI Free Running Clock (mandatory). */
  STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  DLYB_GetConfig(dlyb_instance, &unit, &sel);

  if (DLYB_ConfigureUnitDelay(dlyb_instance) == DLYB_CORE_OK)
  {
    *p_max_clock_phase = DLYB_CalculateMaxOutputClockPhase(dlyb_instance);
    status = HAL_OK;
  }

  DLYB_SetConfig(dlyb_instance, unit, sel);

  /* Disable XSPI Free Running Clock. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->DCR1, XSPI_DCR1_FRCK);

  if (state == DLYB_DISABLED)
  {
    DLYB_Disable(dlyb_instance);
  }

  return status;
}

/**
  * @brief  Enable the delay block peripheral.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @retval HAL_ERROR         The delay is not correctly configured.
  * @retval HAL_INVALID_PARAM When no valid XSPI instance.
  * @retval HAL_OK            The delay is correctly configured.
  */
hal_status_t HAL_XSPI_DLYB_Enable(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  ASSERT_DBG_PARAM(IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)));
  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)) == 0U)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DLYB_Enable(XSPI_DLYB_GET_INSTANCE(hxspi->instance));

  return HAL_OK;
}

/**
  * @brief  Disable the delay block peripheral.
  * @param  hxspi             Pointer to a \ref hal_xspi_handle_t structure that contains
  *                           the handle information for the specified XSPI instance.
  * @retval HAL_INVALID_PARAM When no valid XSPI instance.
  * @retval HAL_OK            The delay block is disabled.
  */
hal_status_t HAL_XSPI_DLYB_Disable(hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);
  ASSERT_DBG_PARAM(IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)));

  ASSERT_DBG_STATE(hxspi->global_state, HAL_XSPI_STATE_IDLE);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1U)
  if (IS_XSPI_DLYB_INSTANCE(XSPI_GET_INSTANCE(hxspi)) == 0U)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  DLYB_Disable(XSPI_DLYB_GET_INSTANCE(hxspi->instance));

  return HAL_OK;
}

/**
  * @brief  Check if the delay block peripheral is enabled or not.
  * @param  hxspi                  Pointer to a \ref hal_xspi_handle_t structure that contains
  *                                the handle information for the specified XSPI instance.
  * @retval HAL_XSPI_DLYB_ENABLED  The delay block is enabled.
  * @retval HAL_XSPI_DLYB_DISABLED The delay block is disabled.
  */
hal_xspi_dlyb_status_t HAL_XSPI_DLYB_IsEnabled(const hal_xspi_handle_t *hxspi)
{
  ASSERT_DBG_PARAM(hxspi != NULL);

  return (hal_xspi_dlyb_status_t)DLYB_IsEnabled(XSPI_DLYB_GET_INSTANCE(hxspi->instance));
}
/**
  * @}
  */


/**
  * @}
  */

/** @addtogroup XSPI_Private_Functions XSPI Private Functions
  * @{
  */

/**
  * @brief  Wait for a flag state until timeout in non-blocking mode.
  * @param  hxspi      Pointer to a \ref hal_xspi_handle_t structure that contains
  *                    the handle information for the specified XSPI instance.
  * @param  flag       Flag checked.
  * @param  state      Value of the flag expected.
  * @param  timeout_ms Timeout duration.
  * @retval HAL_ERROR  An error has occurred.
  * @retval HAL_OK     Flag is correctly set.
  */
static hal_status_t XSPI_WaitFlagStateUntilTimeout(hal_xspi_handle_t *hxspi,
                                                   uint32_t flag,
                                                   hal_xspi_flag_status_t state,
                                                   uint32_t timeout_ms)
{
  uint32_t tickstart = HAL_GetTick();

  while (HAL_XSPI_IsActiveFlag(hxspi, flag) != state)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (HAL_XSPI_IsActiveFlag(hxspi, flag) != state)
      {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
        if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
        {
          hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
        }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
/**
  * @brief DMA XSPI process complete callback.
  * @param hdma Pointer to DMA handle.
  */
static void XSPI_DMACplt(hal_dma_handle_t *hdma)
{
  hal_xspi_handle_t *hxspi = (hal_xspi_handle_t *)(hdma->p_parent);
  hxspi->xfer_count = 0U;

  /* Disable the DMA transfer on the XSPI side. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);

  LL_DMA_DisableChannel((DMA_Channel_TypeDef *)((uint32_t)hdma->instance));

  /* Enable the XSPI transfer complete interrupt. */
  HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC);
}

/**
  * @brief DMA XSPI process half complete callback.
  * @param hdma Pointer to DMA handle.
  */
static void XSPI_DMAHalfCplt(hal_dma_handle_t *hdma)
{
  hal_xspi_handle_t *hxspi = (hal_xspi_handle_t *)(hdma->p_parent);
  hxspi->xfer_count = (hxspi->xfer_count >> 1U);

  if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE) == XSPI_FUNCTIONAL_MODE_INDIRECT_READ)
  {
#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_rx_half_cplt_cb(hxspi);
#else
    HAL_XSPI_RxHalfCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }
  else
  {
#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_tx_half_cplt_cb(hxspi);
#else
    HAL_XSPI_TxHalfCpltCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }
}

/**
  * @brief DMA XSPI communication error callback.
  * @param hdma Pointer to DMA handle.
  */
static void XSPI_DMAError(hal_dma_handle_t *hdma)
{
  hal_xspi_handle_t *hxspi = (hal_xspi_handle_t *)(hdma->p_parent);
  hxspi->xfer_count = 0U;

  /* Disable the DMA transfer on the XSPI side. */
  STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);

  /* Abort the XSPI. */
  if (HAL_XSPI_Abort_IT(hxspi) != HAL_OK)
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    hxspi->last_error_codes |= HAL_XSPI_ERROR_DMA;
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    HAL_XSPI_DisableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC | (uint32_t)HAL_XSPI_IT_FT | (uint32_t)HAL_XSPI_IT_TE);

    hxspi->global_state = HAL_XSPI_STATE_IDLE;

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U)
    hxspi->p_error_cb(hxspi);
#else
    HAL_XSPI_ErrorCallback(hxspi);
#endif /* (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1U) */
  }
}

/**
  * @brief DMA XSPI abort complete callback.
  * @param hdma Pointer to DMA handle.
  */
static void XSPI_DMAAbortOnError(hal_dma_handle_t *hdma)
{
  hal_xspi_handle_t *hxspi = (hal_xspi_handle_t *)(hdma->p_parent);
  hxspi->xfer_count = 0U;

  /* DMA abort called by XSPI abort. */
  if (HAL_XSPI_IsActiveFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY) != HAL_XSPI_FLAG_NOT_ACTIVE)
  {
    /* Clear transfer complete flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);

    /* Enable the transfer complete interrupts. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC);

    /* Perform an abort of the XSPI. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_ABORT);
  }
  else
  {
    return;
  }
}

/**
  * @brief DMA XSPI abort complete callback.
  * @param hdma Pointer to DMA handle.
  */
static void XSPI_DMAAbort(hal_dma_handle_t *hdma)
{
  hal_xspi_handle_t *hxspi = (hal_xspi_handle_t *)(hdma->p_parent);
  hxspi->xfer_count = 0U;

  /* DMA abort called by XSPI abort. */
  if (HAL_XSPI_IsActiveFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY) != HAL_XSPI_FLAG_NOT_ACTIVE)
  {
    /* Clear transfer complete flag. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);

    /* Enable the interrupt on the transfer complete flag. */
    HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC);

    /* Perform an abort of the XSPI. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_ABORT);
  }
  else
  {
    return;
  }
}
#endif /* USE_HAL_XSPI_DMA */

/**
  * @brief  Set the Regular command configuration.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  p_cmd       Structure that contains the Regular command configuration information.
  * @param  timeout_ms  Timeout duration.
  * @param  it_state    interrupt state.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t XSPI_SendRegularCmd(hal_xspi_handle_t *hxspi,
                                        const hal_xspi_regular_cmd_t *p_cmd,
                                        uint32_t timeout_ms,
                                        hal_xspi_interrupt_state_t it_state)
{
  hal_status_t status = HAL_OK;
  volatile uint32_t *p_ccr_reg;
  volatile uint32_t *p_tcr_reg;
  volatile uint32_t *p_ir_reg;
  volatile uint32_t *p_abr_reg;

  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     timeout_ms) == HAL_OK)
  {
    /* Clear transfer error and transfer complete flags. */
    HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_TC);

    /* Set functional mode. */
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, 0U);

    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_IO_SELECT_MSK, (uint32_t)p_cmd->io_select);

    p_ccr_reg = (volatile uint32_t *)((uint32_t)(&(XSPI_GET_INSTANCE(hxspi)->CCR)) + (uint32_t)p_cmd->operation_type);
    p_tcr_reg = (volatile uint32_t *)((uint32_t)(&(XSPI_GET_INSTANCE(hxspi)->TCR)) + (uint32_t)p_cmd->operation_type);
    p_ir_reg  = (volatile uint32_t *)((uint32_t)(&(XSPI_GET_INSTANCE(hxspi)->IR)) + (uint32_t)p_cmd->operation_type);
    p_abr_reg = (volatile uint32_t *)((uint32_t)(&(XSPI_GET_INSTANCE(hxspi)->ABR)) + (uint32_t)p_cmd->operation_type);

    /* Configure DQS modes. */
    *p_ccr_reg = (uint32_t)p_cmd->dqs_mode_status | (uint32_t)p_cmd->alternate_bytes_mode |
                 (uint32_t)p_cmd->alternate_bytes_dtr_mode_status | (uint32_t)p_cmd->alternate_bytes_width |
                 (uint32_t)p_cmd->instruction_mode | (uint32_t)p_cmd->instruction_dtr_mode_status
                 | (uint32_t)p_cmd->instruction_width |
                 (uint32_t)p_cmd->addr_mode | (uint32_t)p_cmd->addr_dtr_mode_status | (uint32_t)p_cmd->addr_width  |
                 (uint32_t)p_cmd->data_mode | (uint32_t)p_cmd->data_dtr_mode_status;

    /* Configure alternate bytes. */
    *p_abr_reg = p_cmd->alternate_bytes;

    /* Configure the number of dummy cycles. */
    STM32_MODIFY_REG((*p_tcr_reg), XSPI_TCR_DCYC, p_cmd->dummy_cycle);

    /* Configure the number of data. */
    XSPI_GET_INSTANCE(hxspi)->DLR = (p_cmd->size_byte - 1U);

    /* Configure the instruction value. */
    *p_ir_reg = p_cmd->instruction;

    /* Configure the address value. */
    XSPI_GET_INSTANCE(hxspi)->AR = p_cmd->addr;

    if (it_state == XSPI_INTERRUPT_DISABLE)
    {
      if (p_cmd->data_mode == HAL_XSPI_REGULAR_DATA_NONE)
      {
        /* When there is no data phase, the transfer starts as soon as the configuration is done.
        Wait until TC flag is set to go back in idle state. */
        if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                           timeout_ms) == HAL_OK)
        {
          HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);
        }
        else
        {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
          if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
          {
            hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
          }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

          status = HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Enable the interrupts on transfer complete and transfer error flags. */
      HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_TC | (uint32_t)HAL_XSPI_IT_TE);
    }
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
    {
      hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
    }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    status = HAL_TIMEOUT;
  }

  return status;
}

#if defined(XSPI_SR_SMF)
/**
  * @brief  Configure the XSPI Automatic Polling Mode for Regular protocol.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  p_config    Pointer to structure that contains the polling configuration information.
  * @param  timeout_ms  Timeout duration.
  * @param  it_state    Interrupt state.
  * @retval HAL_ERROR   An error has occurred.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_BUSY    XSPI state is active when calling this API.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi,
                                             const hal_xspi_auto_polling_config_t *p_config,
                                             uint32_t timeout_ms,
                                             hal_xspi_interrupt_state_t it_state)
{
  uint32_t addr_reg = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg = XSPI_GET_INSTANCE(hxspi)->IR;

  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     timeout_ms) == HAL_OK)
  {
    /* Set the following configurations :
    - match mask
    - match value
    - match mode
    - interval cycle
    - automatic stop */
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->PSMKR, p_config->match_mask);
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->PSMAR, p_config->match_value);
    STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->PIR,   p_config->interval_cycle);
    STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, (XSPI_CR_PMM | XSPI_CR_APMS | XSPI_CR_FMODE),
                     ((uint32_t)p_config->match_mode | (uint32_t)p_config->automatic_stop_status |
                      XSPI_FUNCTIONAL_MODE_AUTO_POLLING));

    if (it_state != XSPI_INTERRUPT_DISABLE)
    {
      /* Clear transfer error and status match flags. */
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TE | (uint32_t)HAL_XSPI_FLAG_SM);

      /* Enable the interrupts on the status match and transfer error flags. */
      HAL_XSPI_EnableIT(hxspi, (uint32_t)HAL_XSPI_IT_SM | (uint32_t)HAL_XSPI_IT_TE);
    }

    /* Trig the transfer by re-writing address or instruction register. */
    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
    }
    else
    {
      STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
    }

    if (it_state == XSPI_INTERRUPT_DISABLE)
    {
      /* Wait till status match flag is set to go back in idle state. */
      if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_SM, HAL_XSPI_FLAG_ACTIVE,
                                         timeout_ms) == HAL_OK)
      {
        /* Clear status match flag. */
        HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_SM);
      }
      else
      {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
        if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
        {
          hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
        }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

        return HAL_TIMEOUT;
      }
    }
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
    {
      hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
    }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    return HAL_TIMEOUT;
  }

  return HAL_OK;
}
#else
/**
  * @brief  Configure the XSPI Automatic Polling Mode for Regular protocol.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  p_config    Pointer to structure that contains the polling configuration information.
  * @param  timeout_ms  Timeout duration.
  * @retval HAL_ERROR   An error has occurred.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_BUSY    XSPI state is active when calling this API.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t XSPI_ExecRegularAutoPoll(hal_xspi_handle_t *hxspi,
                                             const hal_xspi_auto_polling_config_t *p_config,
                                             uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;
  uint32_t addr_reg = XSPI_GET_INSTANCE(hxspi)->AR;
  uint32_t ir_reg = XSPI_GET_INSTANCE(hxspi)->IR;
  volatile uint32_t *p_data_reg = &XSPI_GET_INSTANCE(hxspi)->DR;
  uint32_t buffer = ~(p_config->match_value);

  /* Wait till busy flag is reset. */
  if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                     timeout_ms) == HAL_OK)
  {
    /* Wait till status match flag is set to go back in idle state. */
    while ((buffer & p_config->match_mask) != (p_config->match_value & p_config->match_mask))
    {
      /* Configure counters and size. */
      hxspi->xfer_count = STM32_READ_REG(XSPI_GET_INSTANCE(hxspi)->DLR) + 1U;
      hxspi->xfer_size  = hxspi->xfer_count;
      hxspi->p_buffer   = (uint8_t *)&buffer;

      /* Configure the functional mode as indirect read. */
      STM32_MODIFY_REG(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

      /* Trig the transfer by re-writing address or instruction register. */
      if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CCR, XSPI_CCR_ADMODE) != (uint32_t)HAL_XSPI_ADDR_NONE)
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->AR, addr_reg);
      }
      else
      {
        STM32_WRITE_REG(XSPI_GET_INSTANCE(hxspi)->IR, ir_reg);
      }

      do
      {
        /* Wait till fifo threshold or transfer complete flags are set to read received data. */
        status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_FT | (uint32_t)HAL_XSPI_FLAG_TC,
                                                HAL_XSPI_FLAG_ACTIVE,  timeout_ms);
        if (status != HAL_OK)
        {
          break;
        }

        *hxspi->p_buffer = *((volatile uint8_t *)p_data_reg);
        hxspi->p_buffer++;
        hxspi->xfer_count--;

      } while (hxspi->xfer_count > 0U);

      if (status == HAL_OK)
      {
        /* Wait till transfer complete flag is set to go back in idle state. */
        status = XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_TC, HAL_XSPI_FLAG_ACTIVE,
                                                timeout_ms);

        if (status == HAL_OK)
        {
          HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);
        }
      }

      HAL_Delay(p_config->interval_cycle);
    };
  }
  else
  {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
    if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
    {
      hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
    }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

    return HAL_TIMEOUT;
  }

  return HAL_OK;
}
#endif /* XSPI_SR_SMF */

/**
  * @brief  Abort the current transmission.
  * @param  hxspi       Pointer to a \ref hal_xspi_handle_t structure that contains
  *                     the handle information for the specified XSPI instance.
  * @param  timeout_ms  Timeout duration.
  * @retval HAL_TIMEOUT In case of user timeout.
  * @retval HAL_OK      Operation completed.
  */
static hal_status_t XSPI_Abort(hal_xspi_handle_t *hxspi, uint32_t timeout_ms)
{
#if defined(USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1U)
  if ((XSPI_GET_INSTANCE(hxspi)->CR & XSPI_CR_DMAEN) != 0U)
  {
    /* Disable the DMA transfer on the XSPI side. */
    STM32_CLEAR_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_DMAEN);

    if (STM32_READ_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_FMODE) == XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE)
    {
      /* Disable the DMA transmit on the DMA side. */
      if (HAL_DMA_Abort(hxspi->hdma_tx) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Disable the DMA receive on the DMA side. */
      if (HAL_DMA_Abort(hxspi->hdma_rx) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }
#endif /* USE_HAL_XSPI_DMA */

  if (HAL_XSPI_IsActiveFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY) != HAL_XSPI_FLAG_NOT_ACTIVE)
  {
    /* Perform an abort of the XSPI. */
    STM32_SET_BIT(XSPI_GET_INSTANCE(hxspi)->CR, XSPI_CR_ABORT);

    /* Wait until the transfer complete flag is set to go back in idle state. */
    if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_TC, HAL_XSPI_FLAG_ACTIVE,
                                       timeout_ms) == HAL_OK)
    {
      /* Clear transfer complete flag. */
      HAL_XSPI_ClearFlag(hxspi, (uint32_t)HAL_XSPI_FLAG_TC);

      /* Wait until the busy flag is reset to go back in idle state. */
      if (XSPI_WaitFlagStateUntilTimeout(hxspi, (uint32_t)HAL_XSPI_FLAG_BUSY, HAL_XSPI_FLAG_NOT_ACTIVE,
                                         timeout_ms) != HAL_OK)
      {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
        if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
        {
          hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
        }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */

        return HAL_TIMEOUT;
      }
    }
    else
    {
#if defined(USE_HAL_XSPI_GET_LAST_ERRORS) && (USE_HAL_XSPI_GET_LAST_ERRORS == 1U)
      if (HAL_XSPI_IsActiveFlag(hxspi, HAL_XSPI_FLAG_TE) != HAL_XSPI_FLAG_NOT_ACTIVE)
      {
        hxspi->last_error_codes = HAL_XSPI_ERROR_TRANSFER;
      }
#endif /* USE_HAL_XSPI_GET_LAST_ERRORS */
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @}
  */

#endif /* USE_HAL_XSPI_MODULE */

#endif /* XSPI1 */
/**
  * @}
  */

/**
  * @}
  */
