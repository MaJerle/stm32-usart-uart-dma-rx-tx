/**
  ******************************************************************************
  * @file    stm32c5xx_hal_i2c.c
  * @brief   I2C HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Inter Integrated Circuit (I2C) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions.
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

#if defined(I2C1) || defined(I2C2)
#if defined(USE_HAL_I2C_MODULE) && (USE_HAL_I2C_MODULE == 1)

/** @addtogroup I2C I2C
  * @{
  */
/** @defgroup I2C_Introduction I2C Introduction
  * @{

  - The I2C hardware abstraction layer provides a set of APIs to interface with the STM32 I2C (Inter-integrated circuit
    interface) peripheral.

  - It simplifies the configuration, initialization and management of I2C communication, by supporting various modes
    such as polling, interrupt, and DMA for efficient data transfer.

  - This abstraction layer ensures portability and ease of use across different STM32 series.

  */
/**
  * @}
  */

/** @defgroup I2C_How_To_Use I2C How To Use
  * @{

# How to use the I2C HAL module driver

1. Declare a hal_i2c_handle_t handle structure and initialize the I2Cx driver
   with an I2C HW instance by calling the HAL_I2C_Init().
   The I2Cx clock is enabled inside the HAL_I2C_Init() if USE_HAL_I2C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.

2. Configure the low level hardware (GPIO, CLOCK, NVIC...etc):
  - Enable the I2Cx clock if USE_HAL_I2C_CLK_ENABLE_MODEL = HAL_CLK_ENABLE_NO
  - I2Cx pins configuration :
    - Enable the clock for the I2Cx GPIOs
      - Configure I2Cx pins as alternate function open-drain
    - NVIC configuration if you need to use interrupt process
      - Configure the I2Cx interrupt priority
      - Enable the NVIC I2Cx IRQ Channel
    - DMA configuration if you need to use DMA process
      - Declare a hal_dma_handle_t handle structure for the Transmit or Receive channel
      - Enable the DMAx clock
      - Configure the DMA transfer parameters for each direction, Transmit or Receive
      - Associate the initialized DMA handle to the hi2c DMA Tx or Rx handle respectively using HAL_I2C_SetTxDMA()
        or HAL_I2C_SetRxDMA()
      - For each DMA channel (Tx and Rx), configure the corresponding NVIC line priority and enable it

3. Configure the communication Clock Timing, Own Address1, Master Addressing mode by calling HAL_I2C_SetConfig().

4. Configure and/or enable advanced features. For instances, HAL_I2C_EnableAnalogFilter(), HAL_I2C_SetDigitalFilter(),
   HAL_I2C_SetConfigOwnAddress2(), HAL_I2C_EnableOwnAddress2(), etc.
   All these advanced configurations are optional (not mandatory).

5. For I2Cx IO and IO MEM operations, three operation modes, polling, interrupt and dma are available
   within this driver.
  - Polling mode IO operation
    - Transmit in master mode an amount of data in blocking mode using HAL_I2C_MASTER_Transmit()
    - Receive in master mode an amount of data in blocking mode using HAL_I2C_MASTER_Receive()
    - Transmit in slave mode an amount of data in blocking mode using HAL_I2C_SLAVE_Transmit()
    - Receive in slave mode an amount of data in blocking mode using HAL_I2C_SLAVE_Receive()

  - Polling mode IO MEM operation
    - Write an amount of data in blocking mode to a specific memory address using HAL_I2C_MASTER_MemWrite()
    - Read an amount of data in blocking mode from a specific memory address using HAL_I2C_MASTER_MemRead()

  - Interrupt mode IO operation
    - Transmit in master mode an amount of data in non-blocking mode using HAL_I2C_MASTER_Transmit_IT()
    - At transmission end of transfer, HAL_I2C_MASTER_TxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Receive in master mode an amount of data in non-blocking mode using HAL_I2C_MASTER_Receive_IT()
    - At reception end of transfer, HAL_I2C_MASTER_RxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Transmit in slave mode an amount of data in non-blocking mode using HAL_I2C_SLAVE_Transmit_IT()
    - At transmission end of transfer, HAL_I2C_SLAVE_TxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Receive in slave mode an amount of data in non-blocking mode using HAL_I2C_SLAVE_Receive_IT()
    - At reception end of transfer, HAL_I2C_SLAVE_RxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - In case of transfer error, HAL_I2C_ErrorCallback() function is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Abort a master or memory I2C process communication with interrupt using HAL_I2C_MASTER_Abort_IT()
    - End of abort process, HAL_I2C_AbortCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Discard a slave I2C process communication using HAL_I2C_SLAVE_Abort_IT().
      This action informs master to generate a Stop condition to discard the communication.

  - Interrupt mode or DMA mode IO sequential operation
      These interfaces allow to manage a sequential transfer with a repeated start condition when a direction change
      during transfer. A specific option field manage the different steps of a sequential transfer through
      hal_i2c_xfer_opt_t and are listed below

    - HAL_I2C_XFER_FIRST_AND_LAST_FRAME: No sequential, functional is same as associated interfaces
      in no sequential mode.

    - HAL_I2C_XFER_FIRST_FRAME: Sequential, this option allows to manage a sequence with start condition, address and
      data to transfer without a final stop condition.

    - HAL_I2C_XFER_FIRST_AND_NEXT_FRAME: Sequential (master only), this option allows to manage a sequence with start
      condition, address and data to transfer without a final stop condition, an then permit a call the same master
      sequential interface several times (like HAL_I2C_MASTER_SEQ_Transmit_IT() then HAL_I2C_MASTER_SEQ_Transmit_IT()
      or HAL_I2C_MASTER_SEQ_Transmit_DMA() then HAL_I2C_MASTER_SEQ_Transmit_DMA())

    - HAL_I2C_XFER_NEXT_FRAME: Sequential, this option allows to manage a sequence with a restart condition, address
      and with new data to transfer if the direction change or manage only the new data to transfer if no direction
      change and without a final stop condition in both cases.

    - HAL_I2C_XFER_LAST_FRAME: Sequential, this option allows to manage a sequence with a restart condition, address
      and with new data to transfer if the direction change or manage only the new data to transfer if no direction
      change and with a final stop condition in both cases.

    - HAL_I2C_XFER_LAST_FRAME_NO_STOP: Sequential (master only), this option allows to manage a restart condition
      after several call of the same master sequential interface several times (link with option
      HAL_I2C_XFER_FIRST_AND_NEXT_FRAME).
      User can transfer several bytes one by one using :
        - HAL_I2C_MASTER_SEQ_Transmit_IT()
        - HAL_I2C_MASTER_SEQ_Receive_IT()
        - HAL_I2C_MASTER_SEQ_Transmit_DMA()
        - HAL_I2C_MASTER_SEQ_Receive_DMA() with option HAL_I2C_XFER_FIRST_AND_NEXT_FRAME then HAL_I2C_XFER_NEXT_FRAME.
      Then usage of this option HAL_I2C_XFER_LAST_FRAME_NO_STOP at the last Transmit or Receive sequence permit to
      call the opposite interface Receive or Transmit without stopping the communication and so generate a restart
      condition.

    - HAL_I2C_XFER_OTHER_FRAME: Sequential (master only), this option allows to manage a restart condition after each
      call of the same master sequential interface. User can transfer several bytes one by one with a restart with
      slave address between each byte using
        - HAL_I2C_MASTER_SEQ_Transmit_IT()
        - HAL_I2C_MASTER_SEQ_Receive_IT()
        - HAL_I2C_MASTER_SEQ_Transmit_DMA()
        - HAL_I2C_MASTER_SEQ_Receive_DMA() with option HAL_I2C_XFER_FIRST_FRAME then HAL_I2C_XFER_OTHER_FRAME.
      Then usage of this option HAL_I2C_XFER_OTHER_AND_LAST_FRAME at the last frame to help automatic
      generation of STOP condition.

  - Different sequential I2C interfaces are listed below:
    - Sequential transmit in master mode an amount of data in non-blocking mode using HAL_I2C_MASTER_SEQ_Transmit_IT()
      or using HAL_I2C_MASTER_SEQ_Transmit_DMA().
    - At transmission end of current frame transfer, HAL_I2C_MASTER_TxCpltCallback() is executed and user can add his
      own code by overriding this weak callback function or by registering a callback function.

    - Sequential receive in master I2C mode an amount of data in non-blocking mode using HAL_I2C_MASTER_SEQ_Receive_IT()
      or using HAL_I2C_MASTER_SEQ_Receive_DMA()
    - At reception end of current frame transfer, HAL_I2C_MASTER_RxCpltCallback() is executed and user can add his
      own code by overriding this weak callback function or by registering a callback function.

    - Abort a master or memory IT or DMA I2C process communication with interrupt using HAL_I2C_MASTER_Abort_IT()
    - End of abort process, HAL_I2C_AbortCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Enable/disable the Address listen mode in slave I2C mode with HAL_I2C_SLAVE_EnableListen_IT() and
      HAL_I2C_SLAVE_DisableListen_IT()

    - When address slave I2C match, HAL_I2C_SLAVE_AddrCallback() is executed and users can add their own code to check
      the address Match Code and the transmission direction request by master(Write/Read).

    - At Listen mode end HAL_I2C_SLAVE_ListenCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Sequential transmit in slave I2C mode an amount of data in non-blocking mode using HAL_I2C_SLAVE_SEQ_Transmit_IT()
      or using HAL_I2C_SLAVE_SEQ_Transmit_DMA()

    - At transmission end of current frame transfer, HAL_I2C_SLAVE_TxCpltCallback() is executed and user can add his
      own code by overriding this weak callback function or by registering a callback function.

    - Sequential receive in slave I2C mode an amount of data in non-blocking mode using HAL_I2C_SLAVE_SEQ_Receive_IT()
      or using HAL_I2C_SLAVE_SEQ_Receive_DMA()

    - At reception end of current frame transfer, HAL_I2C_SLAVE_RxCpltCallback() is executed and user can add his
      own code by overriding this weak callback function or by registering a callback function.

    - In case of transfer error, HAL_I2C_ErrorCallback() function is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Discard a slave I2C process communication using HAL_I2C_SLAVE_Abort_IT() macro. This action informs master to
      generate a Stop condition to discard the communication.

  - Interrupt mode IO MEM operation
    - Write an amount of data in non-blocking mode with interrupt to a specific memory address using
      HAL_I2C_MASTER_MemWrite_IT()
    - At Memory end of write transfer, HAL_I2C_MASTER_MemTxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - Read an amount of data in non-blocking mode with interrupt from a specific memory address using
      HAL_I2C_MASTER_MemRead_IT()
    - At Memory end of read transfer, HAL_I2C_MASTER_MemRxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.
    - In case of transfer error, HAL_I2C_ErrorCallback() function is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

  - DMA mode IO operation
    - Transmit in master mode an amount of data in non-blocking mode (DMA) using HAL_I2C_MASTER_Transmit_DMA()

    - At transmission end of transfer, HAL_I2C_MASTER_TxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Receive in master mode an amount of data in non-blocking mode (DMA) using HAL_I2C_MASTER_Receive_DMA()

    - At reception end of transfer, HAL_I2C_MASTER_RxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Transmit in slave mode an amount of data in non-blocking mode (DMA) using HAL_I2C_SLAVE_Transmit_DMA()

    - At transmission end of transfer, HAL_I2C_SLAVE_TxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Receive in slave mode an amount of data in non-blocking mode (DMA) using HAL_I2C_SLAVE_Receive_DMA()

    - At reception end of transfer, HAL_I2C_SLAVE_RxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - In case of transfer error, HAL_I2C_ErrorCallback() function is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Abort a master or memory I2C process communication with interrupt using HAL_I2C_MASTER_Abort_IT()

    - End of abort process, HAL_I2C_AbortCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Discard a slave I2C process communication using HAL_I2C_SLAVE_Abort_IT() macro. This action informs master to
      generate a Stop condition to discard the communication.

  - DMA mode IO MEM operation

    - Write an amount of data in non-blocking mode with DMA to a specific memory address using
      HAL_I2C_MASTER_MemWrite_DMA()

    - At Memory end of write transfer, HAL_I2C_MASTER_MemTxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - Read an amount of data in non-blocking mode with DMA from a specific memory address using
      HAL_I2C_MASTER_MemRead_DMA()

    - At Memory end of read transfer, HAL_I2C_MASTER_MemRxCpltCallback() is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

    - In case of transfer error, HAL_I2C_ErrorCallback() function is executed and user can add his own code
      by overriding this weak callback function or by registering a callback function.

6. Callback registration definition in Interrupt
  - When the compilation define USE_HAL_I2C_REGISTER_CALLBACKS is set to 1, the user can configure dynamically the
    driver callbacks, via its own method:

  Callback name               | Default value                      | Callback registration function
  ----------------------------| ---------------------------------- | ---------------------------
  MASTER_TxCpltCallback       | HAL_I2C_MASTER_TxCpltCallback()    | HAL_I2C_MASTER_RegisterTxCpltCallback()
  MASTER_RxCpltCallback       | HAL_I2C_MASTER_RxCpltCallback()    | HAL_I2C_MASTER_RegisterRxCpltCallback()
  SLAVE_TxCpltCallback        | HAL_I2C_SLAVE_TxCpltCallback()     | HAL_I2C_SLAVE_RegisterTxCpltCallback()
  SLAVE_RxCpltCallback        | HAL_I2C_SLAVE_RxCpltCallback()     | HAL_I2C_SLAVE_RegisterRxCpltCallback()
  MASTER_MemTxCpltCallback    | HAL_I2C_MASTER_MemTxCpltCallback() | HAL_I2C_MASTER_RegisterMemTxCpltCallback()
  MASTER_MemRxCpltCallback    | HAL_I2C_MASTER_MemRxCpltCallback() | HAL_I2C_MASTER_RegisterMemRxCpltCallback()
  ListenCpltCallback          | HAL_I2C_SLAVE_ListenCpltCallback() | HAL_I2C_SLAVE_RegisterListenCpltCallback()
  AddrMatchCallback           | HAL_I2C_SLAVE_AddrCallback()       | HAL_I2C_SLAVE_RegisterAddrMatchCallback()
  AbortCpltCallback           | HAL_I2C_AbortCpltCallback()        | HAL_I2C_RegisterAbortCpltCallback()
  ErrorCallback               | HAL_I2C_ErrorCallback()            | HAL_I2C_RegisterErrorCallback()

  If one needs to unregister a callback, register the default callback via the registration function.

  By default, after the HAL_I2C_Init() and when the state is HAL_I2C_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Callbacks can be registered in handle global_state HAL_I2C_STATE_INIT and HAL_I2C_STATE_IDLE.

  When the compilation define USE_HAL_I2C_REGISTER_CALLBACKS is set to 0 or not defined, the callback registration
  feature is not available and weak callbacks are used, represented by the default value in the table above.

7. Acquire/Release the i2c bus
  - When the compilation flag USE_HAL_MUTEX is set to 1, it allows the user to acquire/reserve the whole I2C bus for
    executing process .
    The HAL Acquire/Release are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal) :
      - HAL_I2C_AcquireBus() for acquire the bus or wait for it.
      - HAL_I2C_ReleaseBus() for releasing the bus.

  - When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_I2C_AcquireBus() and HAL_I2C_ReleaseBus()
    are not available.
  */
/**
  * @}
  */

/** @defgroup I2C_Configuration_Table I2C Configuration Table
  * @{
8. Configuration inside the I2C driver

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_HAL_I2C_MODULE             |         1         | Enable HAL I2C driver module
USE_HAL_I2C_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_I2C_DMA                |         1         | Enable DMA code inside I2C
USE_HAL_CHECK_PARAM            |         0         | Enable runtime parameter check
USE_HAL_I2C_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE    |         0         | Enable atomicity of process state check
USE_HAL_MUTEX                  |         0         | Enable semaphore creation for OS
USE_HAL_I2C_USER_DATA          |         0         | Add a user data inside HAL I2C handle
USE_HAL_I2C_GET_LAST_ERRORS    |         0         | Enable retrieval of last processes error codes

Software configuration defined in preprocessor environment:
Preprocessor flags             |   Default value   | Comment
------------------------------ | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM           |    Not defined    | Enable check param for HAL and LL
USE_ASSERT_DBG_STATE           |    Not defined    | Enable check state for HAL

  */
/**
  * @}
  */

/*Private types ------------------------------------------------------------*/
/** @defgroup I2C_Private_Types I2C Private Types
  * @{
  */

/**
  * @brief I2C start or stop mode.
  */
typedef enum
{
  I2C_NO_STARTSTOP            = (0x00000000U),                                            /*!< No start no stop */
  I2C_GENERATE_STOP           = (uint32_t)(0x80000000U | I2C_CR2_STOP),                   /*!< Stop */
  I2C_GENERATE_START_READ     = (uint32_t)(0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN), /*!< Start read */
  I2C_GENERATE_START_WRITE    = (uint32_t)(0x80000000U | I2C_CR2_START),                  /*!< Start write */
} i2c_start_stop_mode_t;
/**
  * @}
  */

/* Private defines -----------------------------------------------------------*/
/** @defgroup I2C_Private_Constants I2C Private Constants
  * @{
  */
#define XFER_NO_OPTION          (0xFFFF0000U)  /*!< Sequential transfer options default/reset value */
#define I2C_DEFAULT_TIMEOUT_MS  (25U)          /*!< 25 ms */
#define MAX_NBYTE_SIZE          (255U)           /*!< 255 */
#define SLAVE_ADDR_SHIFT        (7U)             /*!< 7 */
#define SLAVE_ADDR_MSK          (0x06U)          /*!< 6 */

/* Private define for @ref PreviousState usage */
#define I2C_STATE_NONE            (0U)        /*!< Default Value */
#define I2C_STATE_MASTER_BUSY_TX  (1UL << 0U) /*!< Master Busy TX */
#define I2C_STATE_MASTER_BUSY_RX  (1UL << 1U) /*!< Master Busy RX */
#define I2C_STATE_SLAVE_BUSY_TX   (1UL << 2U) /*!< Slave Busy TX */
#define I2C_STATE_SLAVE_BUSY_RX   (1UL << 3U) /*!< Memory Busy TX */

/**
  * @brief Private define to centralize the enable/disable of interrupts.
  */
#define I2C_FLAG_MASK (0x0001FFFFU) /*!< Flag mask */

/** @defgroup I2C_Interrupt_configuration I2C interrupt configuration for disable
  * @{
  */
#define I2C_XFER_TX_IT      0x0001U /*!< Bitfield can be combined with @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_RX_IT      0x0002U /*!< Bitfield can be combined with @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_LISTEN_IT  0x8000U /*!< Bitfield can be combined with @ref I2C_XFER_TX_IT and @ref I2C_XFER_RX_IT */
#define I2C_XFER_ERROR_IT   0x0010U /*!< Bit definition to manage addition of global Error and NACK treatment */
#define I2C_XFER_CPLT_IT    0x0020U /*!< Bit definition to manage only STOP event */
#define I2C_XFER_RELOAD_IT  0x0040U /*!< Bit definition to manage only Reload of NBYTE */
/**
  * @}
  */

/** @defgroup I2C_Interrupt_configuration_mask I2C interrupt configuration mask
  * @{
  */
/*! Mask can be combined with @ref I2C_XFER_LISTEN_IT_MASK */
#define I2C_XFER_TX_IT_MASK       (LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE \
                                   | LL_I2C_CR1_NACKIE | LL_I2C_CR1_TXIE)
/*! Mask can be combined with @ref I2C_XFER_LISTEN_IT_MASK */
#define I2C_XFER_RX_IT_MASK       (LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE \
                                   | LL_I2C_CR1_NACKIE | LL_I2C_CR1_RXIE)
/*! Mask can be combined with @ref I2C_XFER_TX_IT_MASK and @ref I2C_XFER_RX_IT_MASK */
#define I2C_XFER_LISTEN_IT_MASK   (LL_I2C_CR1_ADDRIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_ERRIE)
/*! Mask to manage addition of global Error and NACK treatment */
#define I2C_XFER_ERROR_IT_MASK    (LL_I2C_CR1_ERRIE | LL_I2C_CR1_NACKIE)
/*! Mask to manage only STOP event */
#define I2C_XFER_CPLT_IT_MASK     LL_I2C_CR1_STOPIE
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/*! Mask to manage only STOP event with DMA */
#define I2C_XFER_CPLT_IT_DMA_MASK (LL_I2C_CR1_STOPIE | LL_I2C_CR1_TCIE)
#endif /* USE_HAL_I2C_DMA */
/*! Mask to manage only Reload of NBYTE */
#define I2C_XFER_RELOAD_IT_MASK  LL_I2C_CR1_TCIE
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup I2C_Private_Macros I2C Private Macros
  * @{
  */
/**
  * @brief Assert macro.
  */
#define IS_TRANSFER_REQUEST(request)  (((request) == I2C_GENERATE_STOP) \
                                       || ((request) == I2C_GENERATE_START_READ) \
                                       || ((request) == I2C_GENERATE_START_WRITE) \
                                       || ((request) == I2C_NO_STARTSTOP))

/**
  * @brief Assert macro.
  */
#define IS_I2C_ADDRESSING_MODE(mode) (((mode) == HAL_I2C_ADDRESSING_7BIT) \
                                      || ((mode) == HAL_I2C_ADDRESSING_10BIT))

/**
  * @brief Assert macro.
  */
#define IS_I2C_OWN_ADDRESS2_MASK(mask) (((mask) == HAL_I2C_OWN_ADDR2_NOMASK) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK01) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK02) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK03) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK04) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK05) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK06) \
                                        || ((mask) == HAL_I2C_OWN_ADDR2_MASK07))

/**
  * @brief Assert macro.
  */
#define IS_I2C_MEMADD_SIZE(size) (((size) == HAL_I2C_MEM_ADDR_8BIT) \
                                  || ((size) == HAL_I2C_MEM_ADDR_16BIT))

/**
  * @brief Assert macro.
  */
#define IS_TRANSFER_MODE(mode) (((mode) == LL_I2C_MODE_RELOAD) \
                                || ((mode) == LL_I2C_MODE_AUTOEND) \
                                || ((mode) == LL_I2C_MODE_SOFTEND))

/**
  * @brief Assert macro.
  */
#define IS_I2C_TRANSFER_OPTIONS_REQUEST(request) (((request) == HAL_I2C_XFER_FIRST_FRAME) \
                                                  || ((request) == HAL_I2C_XFER_FIRST_AND_NEXT_FRAME) \
                                                  || ((request) == HAL_I2C_XFER_NEXT_FRAME) \
                                                  || ((request) == HAL_I2C_XFER_FIRST_AND_LAST_FRAME) \
                                                  || ((request) == HAL_I2C_XFER_LAST_FRAME) \
                                                  || ((request) == HAL_I2C_XFER_LAST_FRAME_NO_STOP) \
                                                  || IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(request))

/**
  * @brief Assert macro.
  */
#define IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(request) (((request) == HAL_I2C_XFER_OTHER_FRAME) \
                                                        || ((request) == HAL_I2C_XFER_OTHER_AND_LAST_FRAME))
/**
  * @brief Reset CR2 macro.
  */
#define I2C_RESET_CR2(instance) ((instance)->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD   | I2C_CR2_HEAD10R \
                                                                           | I2C_CR2_NBYTES | I2C_CR2_RELOAD \
                                                                           | I2C_CR2_RD_WRN)))

/**
  * @brief Assert macro
  * The device 7-bit address value must be  shifted left by 1 bit.
  * In other words, an 8-bit value is required and the bit 0  is not considered.
  */
#define IS_I2C_OWN_ADDRESS_7BIT(ADDRESS) ((ADDRESS) <= 0x00000FFU)

/**
  * @brief Assert macro.
  */
#define IS_I2C_OWN_ADDRESS_10BIT(ADDRESS) ((ADDRESS) <= 0x000003FFU)

/**
  * @brief Mem address add MSB.
  */
#define I2C_MEM_ADD_MSB(__ADDRESS__)((uint8_t)((uint32_t)(((uint32_t)((__ADDRESS__) & (uint32_t)(0xFF00U))) >> 8U)))

/**
  * @brief Mem address add LSB.
  */
#define I2C_MEM_ADD_LSB(__ADDRESS__) ((uint8_t)((uint32_t)((__ADDRESS__) & (uint32_t)(0x00FFU))))

/**
  * @brief Generate start.
  */
#define I2C_GENERATE_START(__ADDMODE__,__ADDRESS__) (((__ADDMODE__) == LL_I2C_ADDRESSING_MODE_7BIT) ? \
                                                     (uint32_t)((((uint32_t)(__ADDRESS__) & (I2C_CR2_SADD)) \
                                                                 | (I2C_CR2_START) | (I2C_CR2_AUTOEND)) \
                                                                & (~I2C_CR2_RD_WRN)) : \
                                                     (uint32_t)((((uint32_t)(__ADDRESS__) & (I2C_CR2_SADD))\
                                                                 | (I2C_CR2_ADD10) | (I2C_CR2_START) \
                                                                 | (I2C_CR2_AUTOEND)) & (~I2C_CR2_RD_WRN)))

/**
  * @brief Check flag.
  */
#define I2C_CHECK_FLAG(isr, flag) ((((isr) & (flag)) == (flag)) ? 1U : 0U)

/**
  * @brief Check IT source.
  */
#define I2C_CHECK_IT_SOURCE(cr1, it)  ((((cr1) & (it)) == (it)) ? 1U : 0U)

/**
  * @brief Assert macro.
  */
#define IS_I2C_DIGITAL_FILTER(filter)  ((filter) <= 0x0000000FU)

/**
  * @brief Retrieve I2C instance from handle.
  */
#define I2C_GET_INSTANCE(handle) \
  ((I2C_TypeDef *)((uint32_t)((handle)->instance))) /*!< Retrieve I2C instance from handle */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/* Handle DMA transfer */
static void I2C_DMAMasterTransmitCplt(hal_dma_handle_t *hdma);
static void I2C_DMAMasterReceiveCplt(hal_dma_handle_t *hdma);
static void I2C_DMASlaveTransmitCplt(hal_dma_handle_t *hdma);
static void I2C_DMASlaveReceiveCplt(hal_dma_handle_t *hdma);
static void I2C_DMAError(hal_dma_handle_t *hdma);
static void I2C_DMAAbort(hal_dma_handle_t *hdma);
#endif /* USE_HAL_I2C_DMA */

/* Handle IT transfer */
static void I2C_ITAddrCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags);
static void I2C_ITMasterSeqCplt(hal_i2c_handle_t *hi2c);
static void I2C_ITSlaveSeqCplt(hal_i2c_handle_t *hi2c);
static void I2C_ITMasterCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags);
static void I2C_ITSlaveCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags);
static void I2C_ITListenCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags);
static void I2C_ITError(hal_i2c_handle_t *hi2c, uint32_t error_code);

/* Handle IT transfer */
static hal_status_t I2C_RequestMemoryWrite(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                           hal_i2c_mem_addr_size_t memory_addr_size,
                                           uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t I2C_RequestMemoryRead(hal_i2c_handle_t *hi2c, uint32_t device_addr,
                                          uint32_t memory_addr, hal_i2c_mem_addr_size_t memory_addr_size,
                                          uint32_t timeout_ms, uint32_t tick_start);

/* Private functions for I2C transfer IRQ handler */
static hal_status_t I2C_Master_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
static hal_status_t I2C_Mem_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
static hal_status_t I2C_Slave_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
static hal_status_t I2C_Master_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
static hal_status_t I2C_Mem_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
static hal_status_t I2C_Slave_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources);
#endif /* USE_HAL_I2C_DMA */

/* Handle flags during polling transfer */
static hal_status_t I2C_WaitOnFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t flag, uint32_t status,
                                               uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t I2C_WaitOnTXISFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t I2C_WaitOnRXNEFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t I2C_WaitOnSTOPFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start);
static hal_status_t I2C_IsErrorOccurred(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t timeout_ms,
                                        uint32_t tick_start);

/* Centralize the disable of interrupts */
static void I2C_Disable_IRQ(hal_i2c_handle_t *hi2c, uint32_t it_request);

/* Handle different error callbacks */
static void I2C_TreatErrorCallback(hal_i2c_handle_t *hi2c);

/* Flush TXDR register */
static void I2C_Flush_TXDR(I2C_TypeDef *p_i2cx);

/* Handle start, restart, or stop a transfer */
static void I2C_TransferConfig(I2C_TypeDef *p_i2cx, uint32_t device_addr, uint32_t size_byte, uint32_t mode,
                               i2c_start_stop_mode_t request);

/* Convert specific options */
static void I2C_ConvertOtherXferOptions(hal_i2c_handle_t *hi2c);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_Exported_Functions
  * @{
  */

/** @addtogroup I2C_Exported_Functions_Group1
  * @{
A set of functions to initialize and deinitialize the I2Cx peripheral:
 - HAL_I2C_Init(): initialize the selected device with the I2C instance.
 - HAL_I2C_DeInit(): restore the default configuration of the selected I2Cx peripheral.
  */

/**
  * @brief  Initialize the I2C according to the associated handle.
  * @param  hi2c Pointer to a hal_i2c_handle_t.
  * @param  instance HAL I2C instance.
  * @retval HAL_OK HAL I2C instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM HAL I2C instance is NULL.
  * @retval HAL_ERROR HAL I2C semaphore creation failed (USE_HAL_MUTEX is set to 1).
  */
hal_status_t HAL_I2C_Init(hal_i2c_handle_t *hi2c, hal_i2c_t instance)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM(IS_I2C_ALL_INSTANCE((I2C_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hi2c == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->instance = instance;

#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  /* I2C Callbacks to the weak function */
  hi2c->p_master_tx_cplt_cb    = HAL_I2C_MASTER_TxCpltCallback;
  hi2c->p_master_rx_cplt_cb    = HAL_I2C_MASTER_RxCpltCallback;
  hi2c->p_slave_tx_cplt_cb     = HAL_I2C_SLAVE_TxCpltCallback;
  hi2c->p_slave_rx_cplt_cb     = HAL_I2C_SLAVE_RxCpltCallback;
  hi2c->p_slave_listen_cplt_cb = HAL_I2C_SLAVE_ListenCpltCallback;
  hi2c->p_mem_tx_cplt_cb       = HAL_I2C_MASTER_MemTxCpltCallback;
  hi2c->p_mem_rx_cplt_cb       = HAL_I2C_MASTER_MemRxCpltCallback;
  hi2c->p_abort_cplt_cb        = HAL_I2C_AbortCpltCallback;
  hi2c->p_error_cb             = HAL_I2C_ErrorCallback;
  hi2c->p_slave_addr_cb        = HAL_I2C_SLAVE_AddrCallback;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

  /* Private fields */
  hi2c->p_buf_rx = (uint8_t *) NULL;
  hi2c->p_buf_tx = (const uint8_t *) NULL;
  hi2c->xfer_size = 0U;
  hi2c->xfer_count = 0U;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) 0U;
  hi2c->xfer_isr = NULL;
  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;
  hi2c->addr_event_count = 0U;
  hi2c->dev_addr = 0U;
  hi2c->mem_addr = 0U;

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  hi2c->hdma_tx = (hal_dma_handle_t *) NULL;
  hi2c->hdma_rx = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_I2C_DMA */

#if defined (USE_HAL_I2C_USER_DATA) && (USE_HAL_I2C_USER_DATA == 1)
  hi2c->p_user_data = (void *) NULL;
#endif /* USE_HAL_I2C_USER_DATA */

#if defined(USE_HAL_I2C_CLK_ENABLE_MODEL) && (USE_HAL_I2C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  /* Enable I2Cx Clock */
  switch (instance)
  {
    case HAL_I2C1:
      HAL_RCC_I2C1_EnableClock();
      break;
#if defined(I2C2)
    case HAL_I2C2:
      HAL_RCC_I2C2_EnableClock();
      break;
#endif /* I2C2 */
    default:
      break;
  }
#endif /* USE_HAL_I2C_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  if (HAL_OS_SemaphoreCreate(&hi2c->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  hi2c->global_state = HAL_I2C_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the HAL I2C driver for the given handle and disable the peripheral.
  * @param  hi2c pointer to a hal_i2c_handle_t structure.
  */
void HAL_I2C_DeInit(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;
  hal_i2c_state_t temp_state;
  uint32_t count = I2C_DEFAULT_TIMEOUT_MS;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_INIT        \
                                        | HAL_I2C_STATE_IDLE      \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  temp_state = hi2c->global_state;

  if ((temp_state != HAL_I2C_STATE_IDLE) && (temp_state != HAL_I2C_STATE_INIT))
  {
#if defined(USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
    if (LL_I2C_IsEnabledDMAReq_TX(p_i2cx) != 0U)
    {
      if (hi2c->hdma_tx != NULL)
      {
        (void)HAL_DMA_Abort(hi2c->hdma_tx);
      }
    }

    if (LL_I2C_IsEnabledDMAReq_RX(p_i2cx) != 0U)
    {
      if (hi2c->hdma_rx != NULL)
      {
        (void)HAL_DMA_Abort(hi2c->hdma_rx);
      }
    }
#endif /* USE_HAL_I2C_DMA */

    if (hi2c->mode == (hal_i2c_mode_t)HAL_I2C_MODE_SLAVE)
    {
      LL_I2C_AcknowledgeNextData(p_i2cx, LL_I2C_NACK);
    }
    else
    {
      LL_I2C_GenerateStopCondition(p_i2cx);
    }
    /* Wait for the transfer to stop */
    do
    {
      count--;
      if (count == 0U)
      {
        break;
      }
    } while (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U);
  }

  LL_I2C_Disable(p_i2cx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  (void)HAL_OS_SemaphoreDelete(&hi2c->semaphore);
#endif /* USE_HAL_MUTEX */

  hi2c->global_state = HAL_I2C_STATE_RESET;
}
/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group2
  * @{
A set of functions allowing to configure the I2Cx peripheral:

- Global configuration :
  - HAL_I2C_SetConfig()
  - HAL_I2C_GetConfig()

- Unitary configuration :
  - HAL_I2C_SetTiming()
  - HAL_I2C_GetTiming()

- Filter mode:
  - HAL_I2C_EnableAnalogFilter()
  - HAL_I2C_DisableAnalogFilter()
  - HAL_I2C_IsEnabledAnalogFilter()
  - HAL_I2C_SetDigitalFilter()
  - HAL_I2C_GetDigitalFilter()

- Wake-up from Stop mode(s):
  - HAL_I2C_SLAVE_EnableWakeUp()
  - HAL_I2C_SLAVE_DisableWakeUp()
  - HAL_I2C_SLAVE_IsEnabledWakeUp()

- Fast mode plus driving capability:
  - HAL_I2C_EnableFastModePlus()
  - HAL_I2C_DisableFastModePlus()
  - HAL_I2C_IsEnabledFastModePlus()

  */

/**
  * @brief  Configure the I2C according to the user parameters.
  * @param  hi2c Pointer to a hal_i2c_handle_t.
  * @param  p_config Pointer to the configuration structure.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_I2C_SetConfig(hal_i2c_handle_t *hi2c, const hal_i2c_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_I2C_ADDRESSING_MODE(p_config->addressing_mode));
  ASSERT_DBG_PARAM((p_config->addressing_mode == HAL_I2C_ADDRESSING_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(p_config->own_address1) :
                   IS_I2C_OWN_ADDRESS_10BIT(p_config->own_address1));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);

  /* Configure I2Cx Frequency range */
  LL_I2C_SetTiming(p_i2cx, p_config->timing);

  /* Disable I2Cx Own Address1 and clear I2Cx Own Address1 mode */
  LL_I2C_DisableOwnAddress1AndMode(p_i2cx);

  /* Configure I2Cx Own Address1 and ack own address1 mode */
  if (p_config->addressing_mode == HAL_I2C_ADDRESSING_7BIT)
  {
    LL_I2C_ConfigOwnAddress1(p_i2cx, p_config->own_address1, LL_I2C_OWNADDRESS1_7BIT);
  }
  else /* HAL_I2C_ADDRESSING_10BIT */
  {
    LL_I2C_ConfigOwnAddress1(p_i2cx, p_config->own_address1, LL_I2C_OWNADDRESS1_10BIT);
  }

  /* Configure I2Cx addressing master mode */
  LL_I2C_SetMasterAddressingMode(p_i2cx, (uint32_t)p_config->addressing_mode);

  /* Enable the I2Cx AUTOEND by default, and enable NACK (must be disable only during slave process) */
  LL_I2C_WRITE_REG(p_i2cx, CR2, (LL_I2C_READ_REG(p_i2cx, CR2) | I2C_CR2_AUTOEND | I2C_CR2_NACK));

  LL_I2C_Enable(p_i2cx);

  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the I2C configuration.
  * @param  hi2c Pointer to a hal_i2c_handle_t.
  * @param  p_config Pointer to the configuration structure.
  */
void HAL_I2C_GetConfig(const hal_i2c_handle_t *hi2c, hal_i2c_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  p_config->timing = LL_I2C_GetTiming(p_i2cx);

  p_config->addressing_mode = (hal_i2c_addressing_mode_t)
                              LL_I2C_GetMasterAddressingMode(p_i2cx);

  p_config->own_address1 = LL_I2C_GetOwnAddress1(p_i2cx);
}

/**
  * @brief  Set the I2C Timing.
  * @param  hi2c  Pointer to a hal_i2c_handle_t
  * @param  value I2C timing
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SetTiming(hal_i2c_handle_t *hi2c, uint32_t value)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetTiming(p_i2cx, value);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the I2C Timing.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval uint32_t I2C timing value
  */
uint32_t HAL_I2C_GetTiming(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return LL_I2C_GetTiming(I2C_GET_INSTANCE(hi2c));
}

/**
  * @brief  Enable I2C Analog noise filter.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_EnableAnalogFilter(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableAnalogFilter(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable I2C Analog noise filter.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_DisableAnalogFilter(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableAnalogFilter(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check I2C analog noise filter status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_ANALOG_FILTER_ENABLED  Analog Filter is enabled
  * @retval HAL_I2C_ANALOG_FILTER_DISABLED Analog Filter is disabled
  */
hal_i2c_analog_filter_status_t HAL_I2C_IsEnabledAnalogFilter(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return (hal_i2c_analog_filter_status_t) LL_I2C_IsEnabledAnalogFilter(I2C_GET_INSTANCE(hi2c));
}

/**
  * @brief  Set the I2C Digital noise filter.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  noise_filtering_in_bus_clk_period Coefficient of digital noise filter between Min_Data=0x00
  *         and Max_Data=0x0F.
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SetDigitalFilter(hal_i2c_handle_t *hi2c, uint32_t noise_filtering_in_bus_clk_period)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM(IS_I2C_DIGITAL_FILTER(noise_filtering_in_bus_clk_period));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetDigitalFilter(p_i2cx, noise_filtering_in_bus_clk_period);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the I2C Digital noise filter.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval uint32_t Coefficient of digital noise filter between Min_Data=0x00 and Max_Data=0x0F.
  */
uint32_t HAL_I2C_GetDigitalFilter(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return LL_I2C_GetDigitalFilter(I2C_GET_INSTANCE(hi2c));
}

/**
  * @brief  Enable I2C slave wakeup from Stop mode(s).
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_EnableWakeUp(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableWakeUpFromStop(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable slave I2C wakeup from Stop mode(s).
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_DisableWakeUp(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableWakeUpFromStop(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check slave I2C wake up feature status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_SLAVE_WAKE_UP_ENABLED  Slave Wake Up is enabled
  * @retval HAL_I2C_SLAVE_WAKE_UP_DISABLED Slave Wake Up is disabled
  */
hal_i2c_slave_wake_up_status_t HAL_I2C_SLAVE_IsEnabledWakeUp(const hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  ASSERT_DBG_PARAM((hi2c != NULL));

  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return (hal_i2c_slave_wake_up_status_t) LL_I2C_IsEnabledWakeUpFromStop(p_i2cx);
}

/**
  * @brief  Enable I2C fast mode plus.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_EnableFastModePlus(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableFastModePlus(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable I2C fast mode plus.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_DisableFastModePlus(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableFastModePlus(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check I2C fast mode plus feature status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_FAST_MODE_PLUS_ENABLED    Fast mode plus enabled
  * @retval HAL_I2C_FAST_MODE_PLUS_DISABLED   Fast mode plus disabled
  */
hal_i2c_fast_mode_plus_status_t HAL_I2C_IsEnabledFastModePlus(const hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  return (hal_i2c_fast_mode_plus_status_t) LL_I2C_IsEnabledFastModePlus(p_i2cx);
}

/**
  * @brief  Enable slave I2C clock Stretching.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @note   The stretching mode is already enabled after a I2C HW reset
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_EnableClockStretching(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableClockStretching(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable slave I2C clock Stretching.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_DisableClockStretching(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableClockStretching(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check slave clock Stretching status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_SLAVE_STRETCH_ENABLED  Slave stretch mode enabled
  * @retval HAL_I2C_SLAVE_STRETCH_DISABLED Slave stretch mode disabled
  */
hal_i2c_slave_stretch_mode_status_t HAL_I2C_SLAVE_IsEnabledClockStretching(const hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  return (hal_i2c_slave_stretch_mode_status_t)LL_I2C_IsEnabledClockStretching(p_i2cx);
}

/**
  * @brief  Enable slave I2C Acknowledge General Call.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_EnableAckGeneralCall(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableGeneralCall(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable I2C slave Acknowledge General Call.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_DisableAckGeneralCall(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableGeneralCall(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check slave Acknowledge General Call status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_SLAVE_ACK_GENERAL_CALL_ENABLED  Slave Acknowledge General Call is enabled
  * @retval HAL_I2C_SLAVE_ACK_GENERAL_CALL_DISABLED Slave Acknowledge General Call is disabled
  */
hal_i2c_slave_ack_general_call_status_t HAL_I2C_SLAVE_IsEnabledAckGeneralCall(const hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  return (hal_i2c_slave_ack_general_call_status_t)LL_I2C_IsEnabledGeneralCall(p_i2cx);
}

/**
  * @brief  Set the I2C own address2 configuration.
  * @param  hi2c  Pointer to a hal_i2c_handle_t.
  * @param  addr  The second device own address. It is a 7-bit address but the value must be shifted left by 1 bit.
                  In other words, an 8-bit value is required and the bit 0 is not considered.
  * @param  mask  Acknowledge mask address second device own address.
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SetConfigOwnAddress2(hal_i2c_handle_t *hi2c, uint32_t addr, hal_i2c_own_addr2_mask_t mask)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM(IS_I2C_OWN_ADDRESS_7BIT(addr));
  ASSERT_DBG_PARAM(IS_I2C_OWN_ADDRESS2_MASK(mask));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetOwnAddress2(p_i2cx, addr, (uint32_t)mask);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the I2C own address2 configuration.
  * @param  hi2c  Pointer to a hal_i2c_handle_t.
  * @param  addr  The second device own address. It is a 7-bit address but the value must is shifted left by 1 bit.
                  In other words, an 8-bit value is returned and the bit 0 is not considered.
  * @param  mask  Acknowledge mask address second device own address.
  */
void HAL_I2C_GetConfigOwnAddress2(const hal_i2c_handle_t *hi2c, uint32_t *addr, hal_i2c_own_addr2_mask_t *mask)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((addr != NULL));
  ASSERT_DBG_PARAM((mask != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  *addr = LL_I2C_GetOwnAddress2(p_i2cx);
  *mask = (hal_i2c_own_addr2_mask_t) LL_I2C_GetOwnAddress2Mask(p_i2cx);
}

/**
  * @brief  Enable I2C Own Address2.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_EnableOwnAddress2(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableOwnAddress2(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable I2C Own Address2.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_DisableOwnAddress2(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_IDLE);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableOwnAddress2(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check own address 2 status.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_I2C_OWN_ADDR2_ENABLED   Dual addressing is enabled
  * @retval HAL_I2C_OWN_ADDR2_DISABLED  Dual addressing is disabled
  */
hal_i2c_own_addr2_status_t HAL_I2C_IsEnabledOwnAddress2(const hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  return (hal_i2c_own_addr2_status_t)LL_I2C_IsEnabledOwnAddress2(p_i2cx);
}

#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register the I2C master Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the master Tx transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_RegisterTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

  hi2c->p_master_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C master Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the master Rx transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_RegisterRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_master_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C slave Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the slave Tx transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_RegisterTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_slave_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C slave Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the slave Rx transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_RegisterRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_slave_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C Listen Complete callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C Listen Complete callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_RegisterListenCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_slave_listen_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C Memory Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C Memory Tx transfer completed callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_RegisterMemTxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_mem_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C Memory Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C Memory Rx transfer completed callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_RegisterMemRxCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_mem_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C Abort completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C Abort completed callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_RegisterAbortCpltCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C slave Address Match  callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C slave Address Match  callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_RegisterAddrMatchCallback(hal_i2c_handle_t *hi2c, hal_i2c_slave_addr_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_slave_addr_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the I2C Error callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_callback pointer to the I2C Error callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_RegisterErrorCallback(hal_i2c_handle_t *hi2c, hal_i2c_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hi2c->p_error_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Link the Transmit DMA handle to the I2C handle.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  hdma Pointer to a hal_dma_handle_t structure
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SetTxDMA(hal_i2c_handle_t *hi2c, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Link the DMA handle to the I2C handle */
  hi2c->hdma_tx = hdma;
  hdma->p_parent = hi2c;

  return HAL_OK;
}

/**
  * @brief  Link the Receive DMA handle to the I2C handle.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  hdma Pointer to a hal_dma_handle_t structure
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SetRxDMA(hal_i2c_handle_t *hi2c, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (uint32_t)HAL_I2C_STATE_INIT | (uint32_t)HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* link the DMA handle to the I2C handle */
  hi2c->hdma_rx = hdma;
  hdma->p_parent = hi2c;

  return HAL_OK;
}
#endif /* USE_HAL_I2C_DMA */
/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group3
  * @{
A set of functions allowing to manage the I2C data transfers.

- There are two modes of transfer:
  - Blocking mode : The communication is performed in the polling mode.
    The status of all data processing is returned by the same function after finishing transfer.
  - No-Blocking mode : The communication is performed using interrupts or DMA.
    These functions return the status of the transfer startup.
    The end of the data processing is indicated through the dedicated I2C IRQ when using interrupt
      mode or the DMA IRQ when using DMA mode.

- Blocking mode functions are :
  - HAL_I2C_MASTER_Transmit()
  - HAL_I2C_MASTER_Receive()
  - HAL_I2C_MASTER_MemWrite()
  - HAL_I2C_MASTER_MemRead()
  - HAL_I2C_MASTER_PollForSlaveReady()
  - HAL_I2C_SLAVE_Transmit()
  - HAL_I2C_SLAVE_Receive()

- No-Blocking mode functions with interrupt are :
  - HAL_I2C_MASTER_Transmit_IT()
  - HAL_I2C_MASTER_Receive_IT()
  - HAL_I2C_SLAVE_Transmit_IT()
  - HAL_I2C_SLAVE_Receive_IT()
  - HAL_I2C_MASTER_MemWrite_IT()
  - HAL_I2C_MASTER_MemRead_IT()
  - HAL_I2C_MASTER_SEQ_Transmit_IT()
  - HAL_I2C_MASTER_SEQ_Receive_IT()
  - HAL_I2C_SLAVE_SEQ_Transmit_IT()
  - HAL_I2C_SLAVE_SEQ_Receive_IT()
  - HAL_I2C_SLAVE_EnableListen_IT()
  - HAL_I2C_SLAVE_DisableListen_IT()
  - HAL_I2C_MASTER_Abort_IT()
  - HAL_I2C_SLAVE_Abort_IT()

- No-Blocking mode functions with DMA are :
  - HAL_I2C_MASTER_Transmit_DMA()
  - HAL_I2C_MASTER_Receive_DMA()
  - HAL_I2C_SLAVE_Transmit_DMA()
  - HAL_I2C_SLAVE_Receive_DMA()
  - HAL_I2C_MASTER_MemWrite_DMA()
  - HAL_I2C_MASTER_MemRead_DMA()
  - HAL_I2C_MASTER_SEQ_Transmit_DMA()
  - HAL_I2C_MASTER_SEQ_Receive_DMA()
  - HAL_I2C_SLAVE_SEQ_Transmit_DMA()
  - HAL_I2C_SLAVE_SEQ_Receive_DMA()

- A set of transfer weak complete callbacks are provided in non Blocking mode:
  - HAL_I2C_MASTER_TxCpltCallback()
  - HAL_I2C_MASTER_RxCpltCallback()
  - HAL_I2C_SLAVE_TxCpltCallback()
  - HAL_I2C_SLAVE_RxCpltCallback()
  - HAL_I2C_MASTER_MemTxCpltCallback()
  - HAL_I2C_MASTER_MemRxCpltCallback()
  - HAL_I2C_SLAVE_AddrCallback()
  - HAL_I2C_SLAVE_ListenCpltCallback()
  - HAL_I2C_ErrorCallback()
  - HAL_I2C_AbortCpltCallback()
  */

/**
  * @brief  Transmit in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_MASTER_Transmit(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                     uint32_t size_byte, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, I2C_DEFAULT_TIMEOUT_MS, tick_start);
  if (hal_status == HAL_OK)
  {
    hi2c->mode = HAL_I2C_MODE_MASTER;
    hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->p_buf_tx = (const uint8_t *)p_data;
    hi2c->xfer_count = size_byte;
    hi2c->xfer_isr = NULL;

    /* Send slave address */
    /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_WRITE);
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_WRITE);
    }

    while (hi2c->xfer_count > 0U)
    {
      hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        /* Write data to TXDR */
        LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
        hi2c->p_buf_tx++;
        hi2c->xfer_count--;
        hi2c->xfer_size--;

        if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
        {
          /* Wait until TCR flag is set */
          hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TCR, 0U, timeout_ms, tick_start);
          if (hal_status == HAL_OK)
          {
            if (hi2c->xfer_count > MAX_NBYTE_SIZE)
            {
              hi2c->xfer_size = MAX_NBYTE_SIZE;
              I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
            }
            else
            {
              hi2c->xfer_size = hi2c->xfer_count;
              I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
            }
          }
        }
      }

      if (hal_status != HAL_OK)
      {
        break;
      }
    } /* End of while */

    if (hal_status == HAL_OK)
    {
      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set */
      hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        LL_I2C_ClearFlag_STOP(p_i2cx);
        I2C_RESET_CR2(p_i2cx);
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Receive in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms  Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_MASTER_Receive(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                    uint32_t size_byte, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, I2C_DEFAULT_TIMEOUT_MS, tick_start);
  if (hal_status == HAL_OK)
  {
    hi2c->mode = HAL_I2C_MODE_MASTER;
    hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->p_buf_rx = (uint8_t *)p_data;
    hi2c->xfer_count = size_byte;
    hi2c->xfer_isr = NULL;

    /* Send slave address */
    /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ);
    }

    while (hi2c->xfer_count > 0U)
    {
      /* Wait until RXNE flag is set */
      hal_status = I2C_WaitOnRXNEFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        /* Read data from RXDR */
        *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
        hi2c->p_buf_rx++;
        hi2c->xfer_size--;
        hi2c->xfer_count--;

        if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
        {
          /* Wait until TCR flag is set */
          hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TCR, 0U, timeout_ms, tick_start);
          if (hal_status == HAL_OK)
          {
            if (hi2c->xfer_count > MAX_NBYTE_SIZE)
            {
              hi2c->xfer_size = MAX_NBYTE_SIZE;
              I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
            }
            else
            {
              hi2c->xfer_size = hi2c->xfer_count;
              I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
            }
          }
        }
      }

      if (hal_status != HAL_OK)
      {
        break;
      }
    } /* End of while */

    if (hal_status == HAL_OK)
    {
      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set */
      hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        LL_I2C_ClearFlag_STOP(p_i2cx);
        I2C_RESET_CR2(p_i2cx);
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Transmit in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_SLAVE_Transmit(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte,
                                    uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_isr = NULL;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Wait until ADDR flag is set */
  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_ADDR, 0U, timeout_ms, tick_start);
  if (hal_status == HAL_OK)
  {
    /* Preload TX data if no stretch enable */
    if (LL_I2C_IsEnabledClockStretching(p_i2cx) == 0UL)
    {
      /* Preload TX register and Write data to TXDR */
      LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
      hi2c->p_buf_tx++;
      hi2c->xfer_count--;
    }

    LL_I2C_ClearFlag_ADDR(p_i2cx);

    /* If 10bit addressing mode is selected */
    if ((hal_i2c_addressing_mode_t)LL_I2C_GetMasterAddressingMode(p_i2cx) == HAL_I2C_ADDRESSING_10BIT)
    {
      /* Wait until ADDR flag is set */
      hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_ADDR, 0U, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        LL_I2C_ClearFlag_ADDR(p_i2cx);
      }
    }

    if (hal_status == HAL_OK)
    {
      /* Wait until DIR flag is set Transmitter mode */
      hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_DIR, 0U, timeout_ms, tick_start);
      if (hal_status == HAL_OK)
      {
        while (hi2c->xfer_count > 0U)
        {
          hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
          if (hal_status != HAL_OK)
          {
            break;
          }

          /* Write data to TXDR */
          LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
          hi2c->p_buf_tx++;
          hi2c->xfer_count--;
        }

        /* Wait until AF flag is set */
        if (hal_status == HAL_OK)
        {
          hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_NACKF, 0U, timeout_ms, tick_start);
          if (hal_status == HAL_OK)
          {
            I2C_Flush_TXDR(p_i2cx);

            LL_I2C_ClearFlag_NACK(p_i2cx);

            /* Wait until STOP flag is set */
            hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
            if (hal_status == HAL_OK)
            {
              LL_I2C_ClearFlag_STOP(p_i2cx);

              /* Wait until BUSY flag is reset */
              hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, timeout_ms, tick_start);
            }
          }
        }
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_SLAVE_Receive(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_isr = NULL;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Wait until ADDR flag is set */
  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_ADDR, 0U, timeout_ms, tick_start);
  if (hal_status == HAL_OK)
  {
    LL_I2C_ClearFlag_ADDR(p_i2cx);

    /* Wait until DIR flag is reset receiver mode */
    hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_DIR, 1U, timeout_ms, tick_start);
    if (hal_status == HAL_OK)
    {
      while (hi2c->xfer_count > 0U)
      {
        /* Wait until RXNE flag is set */
        hal_status = I2C_WaitOnRXNEFlagUntilTimeout(hi2c, timeout_ms, tick_start);
        if (hal_status != HAL_OK)
        {
          /* Store last receive data if any */
          if (LL_I2C_IsActiveFlag_RXNE(p_i2cx) != 0U)
          {
            /* Read data from RXDR */
            *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
          }
          break;
        }

        /* Read data from RXDR */
        *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
        hi2c->p_buf_rx++;
        hi2c->xfer_count--;
        hi2c->xfer_size--;
      }

      if (hal_status == HAL_OK)
      {
        /* Wait until STOP flag is set */
        hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
        if (hal_status == HAL_OK)
        {
          LL_I2C_ClearFlag_STOP(p_i2cx);

          /* Wait until BUSY flag is reset */
          hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, timeout_ms, tick_start);
        }
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_Transmit_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                        uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Master_ISR_IT;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = LL_I2C_MODE_AUTOEND;
  }

  /* Send slave address */
  /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE */
  I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, I2C_GENERATE_START_WRITE);

  /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_Receive_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                       uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Master_ISR_IT;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = LL_I2C_MODE_AUTOEND;
  }

  /* Send slave address */
  /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE */
  I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, I2C_GENERATE_START_READ);

  /* Enable ERR, TC, STOP, NACK, RXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_Transmit_IT(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Slave_ISR_IT;

  /* Preload TX data if no stretch enable */
  if (LL_I2C_IsEnabledClockStretching(p_i2cx) == 0UL)
  {
    /* Preload TX register and Write data to TXDR */
    LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
    hi2c->p_buf_tx++;
    hi2c->xfer_count--;
    hi2c->xfer_size--;
  }

  /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK | I2C_XFER_LISTEN_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_Receive_IT(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Slave_ISR_IT;

  /* Enable ERR, TC, STOP, NACK, RXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK | I2C_XFER_LISTEN_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_Transmit_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                         uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Master_ISR_DMA;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = LL_I2C_MODE_AUTOEND;
  }

  if (hi2c->xfer_size > 0U)
  {
    if (hi2c->hdma_tx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdma_tx->p_xfer_cplt_cb = I2C_DMAMasterTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdma_tx->p_xfer_error_cb = I2C_DMAError;

      /* Enable the DMA channel */
      hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                                  (uint32_t) p_data,
                                                  LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                                  hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
    }

    if (hal_status == HAL_OK)
    {
      /* Send slave address */
      /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, I2C_GENERATE_START_WRITE);

      hi2c->xfer_count -= hi2c->xfer_size;

      /* Enable ERR and NACK interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

      LL_I2C_EnableDMAReq_TX(p_i2cx);
    }
    else
    {
      hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
      hi2c->mode = HAL_I2C_MODE_NONE;
      hi2c->global_state = HAL_I2C_STATE_IDLE;
      hal_status = HAL_ERROR;
    }
  }
  else
  {
    hi2c->xfer_isr = I2C_Master_ISR_IT;

    /* Send slave address */
    /* Set NBYTES to write and generate START condition */
    I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_WRITE);

    /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);
    hal_status = HAL_OK;
  }

  return hal_status;
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_Receive_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                        uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Master_ISR_DMA;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = LL_I2C_MODE_AUTOEND;
  }

  if (hi2c->xfer_size > 0U)
  {
    if (hi2c->hdma_rx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdma_rx->p_xfer_cplt_cb = I2C_DMAMasterReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdma_rx->p_xfer_error_cb = I2C_DMAError;

      /* Enable the DMA channel */
      hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                                  LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                                  (uint32_t)p_data,
                                                  hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
    }

    if (hal_status == HAL_OK)
    {
      /* Send slave address */
      /* Set NBYTES to read and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, I2C_GENERATE_START_READ);

      hi2c->xfer_count -= hi2c->xfer_size;

      /* Enable ERR and NACK interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

      LL_I2C_EnableDMAReq_RX(p_i2cx);
    }
    else
    {
      hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
      hi2c->mode = HAL_I2C_MODE_NONE;
      hi2c->global_state = HAL_I2C_STATE_IDLE;
      hal_status = HAL_ERROR;
    }
  }
  else
  {
    hi2c->xfer_isr = I2C_Master_ISR_IT;

    /* Send slave address */
    /* Set NBYTES to read and generate START condition */
    I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ);

    /* Enable ERR, TC, STOP, NACK, RXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK);
    hal_status = HAL_OK;
  }

  return hal_status;
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_Transmit_DMA(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  p_i2cx = I2C_GET_INSTANCE(hi2c);

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Slave_ISR_DMA;

  /* Preload TX data if no stretch enable */
  if (LL_I2C_IsEnabledClockStretching(p_i2cx) == 0UL)
  {
    /* Preload TX register and Write data to TXDR */
    LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
    hi2c->p_buf_tx++;
    hi2c->xfer_count--;
    hi2c->xfer_size--;
  }

  if (hi2c->xfer_count != 0U)
  {
    if (hi2c->hdma_tx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdma_tx->p_xfer_cplt_cb = I2C_DMASlaveTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdma_tx->p_xfer_error_cb = I2C_DMAError;

      /* Enable the DMA channel */
      hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                                  (uint32_t)hi2c->p_buf_tx,
                                                  LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                                  hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
    }


    if (hal_status == HAL_OK)
    {
      LL_I2C_AcknowledgeEnable(p_i2cx);

      /* Enable ERR, STOP, NACK, ADDR interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_LISTEN_IT_MASK);

      LL_I2C_EnableDMAReq_TX(p_i2cx);
    }
    else
    {
      hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
      hi2c->mode = HAL_I2C_MODE_NONE;
      hi2c->global_state = HAL_I2C_STATE_LISTEN;
      hal_status = HAL_ERROR;
    }
  }
  else
  {
    LL_I2C_AcknowledgeEnable(p_i2cx);

    /* Enable ERR, STOP, NACK, ADDR interrupts */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_LISTEN_IT_MASK);
    hal_status = HAL_OK;
  }

  return hal_status;
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_Receive_DMA(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Slave_ISR_DMA;

  if (hi2c->hdma_rx != NULL)
  {
    /* Set the I2C DMA transfer complete callback */
    hi2c->hdma_rx->p_xfer_cplt_cb = I2C_DMASlaveReceiveCplt;

    /* Set the DMA error callback */
    hi2c->hdma_rx->p_xfer_error_cb = I2C_DMAError;

    /* Enable the DMA channel */
    hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                                LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                                (uint32_t)p_data,
                                                hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
  }

  if (hal_status == HAL_OK)
  {
    LL_I2C_AcknowledgeEnable(p_i2cx);

    /* Enable ERR, STOP, NACK, ADDR interrupts */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_LISTEN_IT_MASK);
    LL_I2C_EnableDMAReq_RX(p_i2cx);
  }
  else
  {
    hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
    hi2c->mode = HAL_I2C_MODE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    hal_status = HAL_ERROR;
  }

  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_MASTER_MemWrite(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                     hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                     uint32_t size_byte, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();

  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, I2C_DEFAULT_TIMEOUT_MS, tick_start);
  if (hal_status == HAL_OK)
  {
    hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
    hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->p_buf_tx = (const uint8_t *)p_data;
    hi2c->xfer_count = size_byte;
    hi2c->xfer_isr   = NULL;

    /* Send slave address and memory address */
    hal_status = I2C_RequestMemoryWrite(hi2c, device_addr, memory_addr, memory_addr_size, timeout_ms, tick_start);
    if (hal_status == HAL_OK)
    {
      /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE */
      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
      }

      do
      {
        hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
        if (hal_status == HAL_OK)
        {

          /* Write data to TXDR */
          LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
          hi2c->p_buf_tx++;
          hi2c->xfer_count--;
          hi2c->xfer_size--;

          if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
          {
            /* Wait until TCR flag is set */
            hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TCR, 0U, timeout_ms, tick_start);
            if (hal_status == HAL_OK)
            {

              if (hi2c->xfer_count > MAX_NBYTE_SIZE)
              {
                hi2c->xfer_size = MAX_NBYTE_SIZE;
                I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
              }
              else
              {
                hi2c->xfer_size = hi2c->xfer_count;
                I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
              }
            }
          }
        }

      } while ((hi2c->xfer_count > 0U) && (hal_status == HAL_OK));

      if (hal_status == HAL_OK)
      {
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
        /* Wait until STOPF flag is reset */
        hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
        if (hal_status == HAL_OK)
        {
          LL_I2C_ClearFlag_STOP(p_i2cx);
          I2C_RESET_CR2(p_i2cx);
        }
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Operation completed successfully
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  */
hal_status_t HAL_I2C_MASTER_MemRead(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                    hal_i2c_mem_addr_size_t memory_addr_size, void *p_data,
                                    uint32_t size_byte, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  uint32_t tick_start;
  hal_status_t hal_status;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  /* Init tick_start for timeout management */
  tick_start = HAL_GetTick();
  hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_BUSY, 1U, I2C_DEFAULT_TIMEOUT_MS, tick_start);
  if (hal_status == HAL_OK)
  {
    hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
    hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->p_buf_rx  = (uint8_t *)p_data;
    hi2c->xfer_count = size_byte;
    hi2c->xfer_isr   = NULL;

    /* Send slave address and memory address */
    hal_status = I2C_RequestMemoryRead(hi2c, device_addr, memory_addr, memory_addr_size, timeout_ms, tick_start);
    if (hal_status == HAL_OK)
    {
      /* Send slave address */
      /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_READ);
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ);
      }

      do
      {
        /* Wait until RXNE flag is set */
        hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_RXNE, 0U, timeout_ms, tick_start);
        if (hal_status == HAL_OK)
        {

          /* Read data from RXDR */
          *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
          hi2c->p_buf_rx++;
          hi2c->xfer_size--;
          hi2c->xfer_count--;

          if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
          {
            /* Wait until TCR flag is set */
            hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TCR, 0U, timeout_ms, tick_start);
            if (hal_status == HAL_OK)
            {

              if (hi2c->xfer_count > MAX_NBYTE_SIZE)
              {
                hi2c->xfer_size = MAX_NBYTE_SIZE;
                I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
              }
              else
              {
                hi2c->xfer_size = hi2c->xfer_count;
                I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
              }
            }
          }
        }
      } while ((hi2c->xfer_count > 0U) && (hal_status == HAL_OK));

      if (hal_status == HAL_OK)
      {
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
        /* Wait until STOPF flag is reset */
        hal_status = I2C_WaitOnSTOPFlagUntilTimeout(hi2c, timeout_ms, tick_start);
        if (hal_status == HAL_OK)
        {
          LL_I2C_ClearFlag_STOP(p_i2cx);
          I2C_RESET_CR2(p_i2cx);
        }
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  return hal_status;
}
/**
  * @brief  Write an amount of data in non-blocking mode with interrupt to a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_MemWrite_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                        hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                        uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->xfer_size = 0U;
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Mem_ISR_IT;
  hi2c->dev_addr = device_addr;

  /* If memory address size is 8Bit */
  if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
  {
    /* Prefetch memory address */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));

    /* Reset mem_addr content */
    hi2c->mem_addr = 0xFFFFFFFFU;
  }
  /* If memory address size is 16Bit */
  else
  {
    /* Prefetch memory address (MSB part, LSB is managed through interrupt) */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

    /* Prepare mem_addr buffer for LSB part */
    hi2c->mem_addr = I2C_MEM_ADD_LSB(memory_addr);
  }
  /* Send slave address and memory address */
  I2C_TransferConfig(p_i2cx, device_addr, (uint32_t)memory_addr_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_WRITE);

  /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Read an amount of data in non-blocking mode with interrupt from a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_MemRead_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                       hal_i2c_mem_addr_size_t memory_addr_size, void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Mem_ISR_IT;
  hi2c->dev_addr = device_addr;

  /* If memory address size is 8Bit */
  if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
  {
    /* Prefetch memory address */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));

    /* Reset mem_addr content */
    hi2c->mem_addr = 0xFFFFFFFFU;
  }
  /* If memory address size is 16Bit */
  else
  {
    /* Prefetch memory address (MSB part, LSB is managed through interrupt) */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

    /* Prepare mem_addr buffer for LSB part */
    hi2c->mem_addr = I2C_MEM_ADD_LSB(memory_addr);
  }
  /* Send slave address and memory address */
  I2C_TransferConfig(p_i2cx, device_addr, (uint32_t) memory_addr_size, LL_I2C_MODE_SOFTEND, I2C_GENERATE_START_WRITE);

  /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_MemWrite_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                         hal_i2c_mem_addr_size_t memory_addr_size, const void *p_data,
                                         uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Mem_ISR_DMA;
  hi2c->dev_addr = device_addr;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
  }

  /* If memory address size is 8Bit */
  if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
  {
    /* Prefetch memory address */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));

    /* Reset mem_addr content */
    hi2c->mem_addr = 0xFFFFFFFFU;
  }
  /* If memory address size is 16Bit */
  else
  {
    /* Prefetch memory address (MSB part, LSB is managed through interrupt) */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

    /* Prepare mem_addr buffer for LSB part */
    hi2c->mem_addr = I2C_MEM_ADD_LSB(memory_addr);
  }

  if (hi2c->hdma_tx != NULL)
  {
    /* Set the I2C DMA transfer complete callback */
    hi2c->hdma_tx->p_xfer_cplt_cb = I2C_DMAMasterTransmitCplt;

    /* Set the DMA error callback */
    hi2c->hdma_tx->p_xfer_error_cb = I2C_DMAError;

    /* Enable the DMA channel */
    hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                                (uint32_t)p_data,
                                                LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                                hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
  }

  if (hal_status == HAL_OK)
  {
    /* Send slave address and memory address */
    I2C_TransferConfig(p_i2cx, device_addr, (uint32_t)memory_addr_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_WRITE);

    /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);
  }
  else
  {
    hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
    hi2c->mode = HAL_I2C_MODE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    hal_status = HAL_ERROR;
  }

  return hal_status;
}

/**
  * @brief  Read an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be read in bytes
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing or bus is busy
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_MemRead_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t memory_addr,
                                        hal_i2c_mem_addr_size_t memory_addr_size, void *p_data, uint32_t size_byte)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_MEMADD_SIZE(memory_addr_size));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  if (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
  {
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    return HAL_BUSY;
  }

  hi2c->mode = HAL_I2C_MODE_MASTER_MEM;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_isr = I2C_Mem_ISR_DMA;
  hi2c->dev_addr = device_addr;

  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
  }

  /* If memory address size is 8Bit */
  if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
  {
    /* Prefetch memory address */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));

    /* Reset mem_addr content */
    hi2c->mem_addr = 0xFFFFFFFFU;
  }
  /* If memory address size is 16Bit */
  else
  {
    /* Prefetch memory address (MSB part, LSB is managed through interrupt) */
    LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

    /* Prepare mem_addr buffer for LSB part */
    hi2c->mem_addr = I2C_MEM_ADD_LSB(memory_addr);
  }

  if (hi2c->hdma_rx != NULL)
  {
    /* Set the I2C DMA transfer complete callback */
    hi2c->hdma_rx->p_xfer_cplt_cb = I2C_DMAMasterReceiveCplt;

    /* Set the DMA error callback */
    hi2c->hdma_rx->p_xfer_error_cb = I2C_DMAError;

    /* Enable the DMA channel */
    hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                                LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                                (uint32_t)p_data,
                                                hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
  }

  if (hal_status == HAL_OK)
  {
    /* Send slave address and memory address */
    I2C_TransferConfig(p_i2cx, device_addr, (uint32_t)memory_addr_size, LL_I2C_MODE_SOFTEND, I2C_GENERATE_START_WRITE);

    /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);
  }
  else
  {
    hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
    hi2c->mode = HAL_I2C_MODE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;
    hal_status = HAL_ERROR;
  }

  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Check if target device is ready for communication.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  timeout_ms Timeout duration in millisecond
  * @note   This function is used with Memory devices
  * @retval HAL_OK            Target is ready for communication
  * @retval HAL_ERROR         Internal failure while waiting for hardware flags
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       User timeout elapsed: device not ready in time
  */
hal_status_t HAL_I2C_MASTER_PollForSlaveReady(hal_i2c_handle_t *hi2c, uint32_t device_addr, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t tick_start;
  uint32_t tmp1;
  uint32_t tmp2;

  ASSERT_DBG_PARAM((hi2c != NULL));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  while (hal_status == HAL_OK)
  {
    tick_start = HAL_GetTick();

    while (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        hi2c->global_state = HAL_I2C_STATE_IDLE;
        return HAL_TIMEOUT;
      }
    }

    /* Generate Start */
    LL_I2C_WRITE_REG(p_i2cx, CR2, (I2C_GENERATE_START(LL_I2C_GetMasterAddressingMode(p_i2cx), device_addr)));

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set or a NACK flag is set */

    tmp1 = LL_I2C_IsActiveFlag_STOP(p_i2cx);
    tmp2 = LL_I2C_IsActiveFlag_NACK(p_i2cx);

    while (((tmp1 == 0U) && (tmp2 == 0U)) && (hal_status == HAL_OK))
    {
      if (timeout_ms != HAL_MAX_DELAY)
      {
        if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
        {
          /* Unable to get a response on the i2c bus */
          hi2c->global_state = HAL_I2C_STATE_IDLE;
          hal_status = HAL_TIMEOUT;
        }
      }

      tmp1 = LL_I2C_IsActiveFlag_STOP(p_i2cx);
      tmp2 = LL_I2C_IsActiveFlag_NACK(p_i2cx);
    }

    if (hal_status == HAL_OK)
    {
      /* Check if the NACKF flag has not been set */
      if (LL_I2C_IsActiveFlag_NACK(p_i2cx) == 0U)
      {
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_STOPF, 0U, timeout_ms, tick_start) == HAL_OK)
        {
          /* A acknowledge appear during STOP Flag waiting process, this mean that device respond to its address */

          /* Clear STOP Flag */
          LL_I2C_ClearFlag_STOP(p_i2cx);
          break;
        }
      }
      else
      {
        /* A non acknowledge is detected, this mean that device not respond to its address,
        a new trial must be performed */

        /* Clear NACK Flag */
        LL_I2C_ClearFlag_NACK(p_i2cx);

        /* Wait until STOPF flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_STOPF, 0U, I2C_DEFAULT_TIMEOUT_MS, tick_start) == HAL_OK)
        {
          /* Clear STOP flag, auto generated with autoend */
          LL_I2C_ClearFlag_STOP(p_i2cx);
        }

        hi2c->mode = HAL_I2C_MODE_NONE;
        hal_status = HAL_ERROR;
      }
    }
  }

  hi2c->global_state = HAL_I2C_STATE_IDLE;

  /* If no response from device */
  return hal_status;
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_SEQ_Transmit_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                            uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  i2c_start_stop_mode_t xfer_request = I2C_GENERATE_START_WRITE;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Master_ISR_IT;

  /* If hi2c->xfer_count > MAX_NBYTE_SIZE, use reload mode */
  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = (uint32_t) hi2c->xfer_opt;
  }

  /* If transfer direction not change and there is no request to start another frame,
     do not generate Restart Condition */
  /* Mean Previous state is same as current state */
  if ((hi2c->previous_state == I2C_STATE_MASTER_BUSY_TX)
      && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(xfer_opt) == 0))
  {
    xfer_request = I2C_NO_STARTSTOP;
  }
  else
  {
    /* Convert OTHER_xxx xfer_opt if any */
    I2C_ConvertOtherXferOptions(hi2c);

    /* Update xfer_mode accordingly if no reload is necessary */
    if (hi2c->xfer_count <= MAX_NBYTE_SIZE)
    {
      xfer_mode = (uint32_t) hi2c->xfer_opt;
    }
  }

  /* Send slave address and set NBYTES to write */
  I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, xfer_request);

  /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
  /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
  /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_SEQ_Transmit_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, const void *p_data,
                                             uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  i2c_start_stop_mode_t xfer_request = I2C_GENERATE_START_WRITE;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */

  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_TX);

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Master_ISR_DMA;

  /* If hi2c->xfer_count > MAX_NBYTE_SIZE, use reload mode */
  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = (uint32_t) hi2c->xfer_opt;
  }

  /* If transfer direction not change and there is no request to start another frame,
     do not generate Restart Condition */
  /* Mean Previous state is same as current state */
  if ((hi2c->previous_state == I2C_STATE_MASTER_BUSY_TX)
      && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(xfer_opt) == 0))
  {
    xfer_request = I2C_NO_STARTSTOP;
  }
  else
  {
    /* Convert OTHER_xxx xfer_opt if any */
    I2C_ConvertOtherXferOptions(hi2c);

    /* Update xfer_mode accordingly if no reload is necessary */
    if (hi2c->xfer_count <= MAX_NBYTE_SIZE)
    {
      xfer_mode = (uint32_t) hi2c->xfer_opt;
    }
  }

  if (hi2c->xfer_size > 0U)
  {
    if (hi2c->hdma_tx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdma_tx->p_xfer_cplt_cb = I2C_DMAMasterTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdma_tx->p_xfer_error_cb = I2C_DMAError;

      /* Enable the DMA channel */
      hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                                  (uint32_t)p_data,
                                                  LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                                  hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
    }

    if (hal_status == HAL_OK)
    {
      /* Send slave address and set NBYTES to write */
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, xfer_request);

      hi2c->xfer_count -= hi2c->xfer_size;

      /* Enable ERR and NACK interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

      LL_I2C_EnableDMAReq_TX(p_i2cx);
    }
    else
    {
      hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
      hi2c->mode = HAL_I2C_MODE_NONE;
      hi2c->global_state = HAL_I2C_STATE_IDLE;
      hal_status = HAL_ERROR;
    }
  }
  else
  {
    hi2c->xfer_isr = I2C_Master_ISR_IT;

    /* Send slave address */
    /* Set NBYTES to write and generate START condition */
    I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_WRITE);

    /* Enable ERR, TC, STOP, NACK, TXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK);
    hal_status = HAL_OK;
  }

  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_SEQ_Receive_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                           uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  i2c_start_stop_mode_t xfer_request = I2C_GENERATE_START_READ;

  ASSERT_DBG_PARAM((hi2c != NULL));

  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Master_ISR_IT;

  /* If hi2c->xfer_count > MAX_NBYTE_SIZE, use reload mode */
  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = (uint32_t) hi2c->xfer_opt;
  }

  /* If transfer direction not change and there is no request to start another frame,
     do not generate Restart Condition */
  /* Mean Previous state is same as current state */
  if ((hi2c->previous_state == I2C_STATE_MASTER_BUSY_RX)
      && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(xfer_opt) == 0))
  {
    xfer_request = I2C_NO_STARTSTOP;
  }
  else
  {
    /* Convert OTHER_xxx xfer_opt if any */
    I2C_ConvertOtherXferOptions(hi2c);

    /* Update xfer_mode accordingly if no reload is necessary */
    if (hi2c->xfer_count <= MAX_NBYTE_SIZE)
    {
      xfer_mode = (uint32_t) hi2c->xfer_opt;
    }
  }

  /* Send slave address and set NBYTES to read */
  I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, xfer_request);

  LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_MASTER_SEQ_Receive_DMA(hal_i2c_handle_t *hi2c, uint32_t device_addr, void *p_data,
                                            uint32_t size_byte, hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  uint32_t xfer_mode;
  i2c_start_stop_mode_t xfer_request = I2C_GENERATE_START_READ;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0UL)); /* if size_byte is equals to 0 p_data equals to NULL
                                                               is allowed */
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  p_i2cx = I2C_GET_INSTANCE(hi2c);
  ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                   IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0UL)) /* if size_byte is equals to 0 p_data equals to NULL is allowed */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */
  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_RX);

  hi2c->mode = HAL_I2C_MODE_MASTER;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Master_ISR_DMA;

  /* If hi2c->xfer_count > MAX_NBYTE_SIZE, use reload mode */
  if (hi2c->xfer_count > MAX_NBYTE_SIZE)
  {
    hi2c->xfer_size = MAX_NBYTE_SIZE;
    xfer_mode = LL_I2C_MODE_RELOAD;
  }
  else
  {
    hi2c->xfer_size = hi2c->xfer_count;
    xfer_mode = (uint32_t) hi2c->xfer_opt;
  }

  /* If transfer direction not change and there is no request to start another frame,
     do not generate Restart Condition */
  /* Mean Previous state is same as current state */
  if ((hi2c->previous_state == I2C_STATE_MASTER_BUSY_RX)
      && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(xfer_opt) == 0))
  {
    xfer_request = I2C_NO_STARTSTOP;
  }
  else
  {
    /* Convert OTHER_xxx xfer_opt if any */
    I2C_ConvertOtherXferOptions(hi2c);

    /* Update xfer_mode accordingly if no reload is necessary */
    if (hi2c->xfer_count <= MAX_NBYTE_SIZE)
    {
      xfer_mode = (uint32_t) hi2c->xfer_opt;
    }
  }

  if (hi2c->xfer_size > 0U)
  {
    if (hi2c->hdma_rx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdma_rx->p_xfer_cplt_cb = I2C_DMAMasterReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdma_rx->p_xfer_error_cb = I2C_DMAError;

      /* Enable the DMA channel */
      hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                                  LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                                  (uint32_t)p_data,
                                                  hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
    }

    if (hal_status == HAL_OK)
    {
      /* Send slave address and set NBYTES to read */
      I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, xfer_mode, xfer_request);

      hi2c->xfer_count -= hi2c->xfer_size;

      /* Enable ERR and NACK interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

      LL_I2C_EnableDMAReq_RX(p_i2cx);
    }
    else
    {
      hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
      hi2c->mode = HAL_I2C_MODE_NONE;
      hi2c->global_state = HAL_I2C_STATE_IDLE;
      hal_status = HAL_ERROR;
    }
  }
  else
  {
    hi2c->xfer_isr = I2C_Master_ISR_IT;

    /* Send slave address */
    /* Set NBYTES to read and generate START condition */
    I2C_TransferConfig(p_i2cx, device_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ);

    /* Enable ERR, TC, STOP, NACK, RXI interrupt possible to enable all of these */
    /* LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE */
    /* | LL_I2C_CR1_ADDRIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_TXIE */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK);
    hal_status = HAL_OK;
  }

  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_SEQ_Transmit_IT(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte,
                                           hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_LISTEN      \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
  if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                       | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
  {
    return HAL_BUSY;
  }
#endif /* USE_HAL_CHECK_PROCESS_STATE */
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Disable interrupts, to prevent preemption during treatment in case of multicall */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

  /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
  /* and then toggle the HAL slave RX state to TX state */
  if (hi2c->global_state == HAL_I2C_STATE_RX_LISTEN)
  {
    /* Disable associated interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
    /* Abort DMA Xfer if any */
    if (LL_I2C_IsEnabledDMAReq_RX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_RX(p_i2cx);

      if (hi2c->hdma_rx != NULL)
      {
        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_rx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA RX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_rx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_rx->p_xfer_abort_cb(hi2c->hdma_rx);
        }
      }
    }
#endif /* USE_HAL_I2C_DMA */
  }

  hi2c->global_state = HAL_I2C_STATE_TX_LISTEN;
  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Slave_ISR_IT;

  tmp = LL_I2C_IsActiveFlag_ADDR(p_i2cx);
  if ((LL_I2C_GetTransferDirection(p_i2cx) == LL_I2C_DIRECTION_READ) && (tmp != 0U))
  {
    /* Clear ADDR flag after prepare the transfer parameters */
    /* This action generates an acknowledge to the master */
    LL_I2C_ClearFlag_ADDR(p_i2cx);
  }

  /* Re-enable ADDR interrupt */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_TX_IT_MASK | I2C_XFER_LISTEN_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_SEQ_Transmit_DMA(hal_i2c_handle_t *hi2c, const void *p_data, uint32_t size_byte,
                                            hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_LISTEN      \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
  if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                       | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
  {
    return HAL_BUSY;
  }
#endif /* USE_HAL_CHECK_PROCESS_STATE */
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Disable interrupts, to prevent preemption during treatment in case of multicall */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

  /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
  /* and then toggle the HAL slave RX state to TX state */
  if (hi2c->global_state == HAL_I2C_STATE_RX_LISTEN)
  {
    /* Disable associated interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

    if (LL_I2C_IsEnabledDMAReq_RX(p_i2cx) != 0UL)
    {
      /* Abort DMA Xfer if any */
      if (hi2c->hdma_rx != NULL)
      {
        LL_I2C_DisableDMAReq_RX(p_i2cx);

        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_rx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA RX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_rx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_rx->p_xfer_abort_cb(hi2c->hdma_rx);
        }
      }
    }
  }
  else if (hi2c->global_state == HAL_I2C_STATE_TX_LISTEN)
  {
    if (LL_I2C_IsEnabledDMAReq_TX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_TX(p_i2cx);

      /* Abort DMA Xfer if any */
      if (hi2c->hdma_tx != NULL)
      {
        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_tx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA TX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_tx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_tx->p_xfer_abort_cb(hi2c->hdma_tx);
        }
      }
    }
  }
  else
  {
    /* Nothing to do */
  }

  hi2c->global_state = HAL_I2C_STATE_TX_LISTEN;
  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_tx = (const uint8_t *)p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Slave_ISR_DMA;

  if (hi2c->hdma_tx != NULL)
  {
    /* Set the I2C DMA transfer complete callback */
    hi2c->hdma_tx->p_xfer_cplt_cb = I2C_DMASlaveTransmitCplt;

    /* Set the DMA error callback */
    hi2c->hdma_tx->p_xfer_error_cb = I2C_DMAError;

    /* Enable the DMA channel */
    hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                                (uint32_t)p_data,
                                                LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                                hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
  }

  if (hal_status == HAL_OK)
  {
    hi2c->xfer_count -= hi2c->xfer_size;
    hi2c->xfer_size = 0;

    tmp = LL_I2C_IsActiveFlag_ADDR(p_i2cx);
    if ((LL_I2C_GetTransferDirection(p_i2cx) == LL_I2C_DIRECTION_READ) && (tmp != 0U))
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action generates an acknowledge to the master */
      LL_I2C_ClearFlag_ADDR(p_i2cx);
    }

    LL_I2C_EnableDMAReq_TX(p_i2cx);

    /* Enable ERR, STOP, NACK, ADDR interrupts */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_LISTEN_IT_MASK);
  }
  else
  {
    hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
    hi2c->mode = HAL_I2C_MODE_NONE;
    hi2c->global_state = HAL_I2C_STATE_LISTEN;
    hal_status = HAL_ERROR;
  }


  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_SEQ_Receive_IT(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte,
                                          hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_LISTEN      \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
  if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                       | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
  {
    return HAL_BUSY;
  }
#endif /* USE_HAL_CHECK_PROCESS_STATE */
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Disable interrupts, to prevent preemption during treatment in case of multicall */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

  /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
  /* and then toggle the HAL slave TX state to RX state */
  if (hi2c->global_state == HAL_I2C_STATE_TX_LISTEN)
  {
    /* Disable associated interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
    if (LL_I2C_IsEnabledDMAReq_TX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_TX(p_i2cx);

      /* Abort DMA Xfer if any */
      if (hi2c->hdma_tx != NULL)
      {
        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_tx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA TX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_tx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_tx->p_xfer_abort_cb(hi2c->hdma_tx);
        }
      }
    }
#endif /* USE_HAL_I2C_DMA */
  }

  hi2c->global_state = HAL_I2C_STATE_RX_LISTEN;
  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Slave_ISR_IT;

  tmp = LL_I2C_IsActiveFlag_ADDR(p_i2cx);
  if ((LL_I2C_GetTransferDirection(p_i2cx) == LL_I2C_DIRECTION_WRITE) && (tmp != 0U))
  {
    /* Clear ADDR flag after prepare the transfer parameters */
    /* This action generates an acknowledge to the master */
    LL_I2C_ClearFlag_ADDR(p_i2cx);
  }

  /* Re-enable ADDR interrupt */
  LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK | I2C_XFER_LISTEN_IT_MASK);

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to be sent in bytes
  * @param  xfer_opt Options of transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Dma error
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_I2C_SLAVE_SEQ_Receive_DMA(hal_i2c_handle_t *hi2c, void *p_data, uint32_t size_byte,
                                           hal_i2c_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;
  hal_status_t hal_status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL) && (p_data != NULL) && (size_byte != 0UL));
  ASSERT_DBG_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(xfer_opt));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_LISTEN      \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
  if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                       | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
  {
    return HAL_BUSY;
  }
#endif /* USE_HAL_CHECK_PROCESS_STATE */
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Disable interrupts, to prevent preemption during treatment in case of multicall */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

  /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
  /* and then toggle the HAL slave TX state to RX state */
  if (hi2c->global_state == HAL_I2C_STATE_TX_LISTEN)
  {
    /* Disable associated interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    if (LL_I2C_IsEnabledDMAReq_TX(p_i2cx) != 0UL)
    {
      /* Abort DMA Xfer if any */
      if (hi2c->hdma_tx != NULL)
      {
        LL_I2C_DisableDMAReq_TX(p_i2cx);

        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_tx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA TX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_tx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_tx->p_xfer_abort_cb(hi2c->hdma_tx);
        }
      }
    }
  }
  else if (hi2c->global_state == HAL_I2C_STATE_RX_LISTEN)
  {
    if (LL_I2C_IsEnabledDMAReq_RX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_RX(p_i2cx);

      /* Abort DMA Xfer if any */
      if (hi2c->hdma_rx != NULL)
      {
        /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdma_rx->p_xfer_abort_cb = I2C_DMAAbort;

        /* Abort DMA RX */
        if (HAL_DMA_Abort_IT(hi2c->hdma_rx) != HAL_OK)
        {
          /* Call directly p_xfer_abort_cb function in case of error */
          hi2c->hdma_rx->p_xfer_abort_cb(hi2c->hdma_rx);
        }
      }
    }
  }
  else
  {
    /* Nothing to do */
  }

  hi2c->global_state = HAL_I2C_STATE_RX_LISTEN;
  hi2c->mode = HAL_I2C_MODE_SLAVE;
  hi2c->last_error_codes = HAL_I2C_ERROR_NONE;

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hi2c->p_buf_rx = (uint8_t *) p_data;
  hi2c->xfer_count = size_byte;
  hi2c->xfer_size = hi2c->xfer_count;
  hi2c->xfer_opt = xfer_opt;
  hi2c->xfer_isr = I2C_Slave_ISR_DMA;

  if (hi2c->hdma_rx != NULL)
  {
    /* Set the I2C DMA transfer complete callback */
    hi2c->hdma_rx->p_xfer_cplt_cb = I2C_DMASlaveReceiveCplt;

    /* Set the DMA error callback */
    hi2c->hdma_rx->p_xfer_error_cb = I2C_DMAError;

    /* Enable the DMA channel */
    hal_status = HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                                LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                                (uint32_t)p_data,
                                                hi2c->xfer_size, HAL_DMA_OPT_IT_NONE);
  }

  if (hal_status == HAL_OK)
  {
    hi2c->xfer_count -= hi2c->xfer_size;
    hi2c->xfer_size = 0;

    tmp = LL_I2C_IsActiveFlag_ADDR(p_i2cx);
    if ((LL_I2C_GetTransferDirection(p_i2cx) == LL_I2C_DIRECTION_WRITE) && (tmp != 0U))
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action generates an acknowledge to the master */
      LL_I2C_ClearFlag_ADDR(p_i2cx);
    }

    LL_I2C_EnableDMAReq_RX(p_i2cx);

    /* Re-enable ADDR interrupt */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK | I2C_XFER_LISTEN_IT_MASK);
  }
  else
  {
    hi2c->last_error_codes |= HAL_I2C_ERROR_DMA;
    hi2c->mode = HAL_I2C_MODE_NONE;
    hi2c->global_state = HAL_I2C_STATE_LISTEN;
    hal_status = HAL_ERROR;
  }


  return hal_status;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Enable the Address listen mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK    Operation started successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  */
hal_status_t HAL_I2C_SLAVE_EnableListen_IT(hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_IDLE, HAL_I2C_STATE_LISTEN);

  hi2c->xfer_isr = I2C_Slave_ISR_IT;

  /* Enable the Address Match interrupt */
  LL_I2C_EnableIT(I2C_GET_INSTANCE(hi2c), I2C_XFER_LISTEN_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Disable the Address listen mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  */
hal_status_t HAL_I2C_SLAVE_DisableListen_IT(hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));

  ASSERT_DBG_STATE(hi2c->global_state, HAL_I2C_STATE_LISTEN);
  HAL_CHECK_UPDATE_STATE(hi2c, global_state, HAL_I2C_STATE_LISTEN, HAL_I2C_STATE_IDLE);

  hi2c->previous_state = I2C_STATE_NONE;
  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->xfer_isr = NULL;

  /* Disable the Address Match interrupt */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

  return HAL_OK;
}

/**
  * @brief  Abort a master or memory I2C IT or DMA process communication with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Mode is not master
  * @retval HAL_BUSY          No process ongoing
  */
hal_status_t HAL_I2C_MASTER_Abort_IT(hal_i2c_handle_t *hi2c, uint32_t device_addr)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));

  hal_i2c_mode_t tmp_mode = hi2c->mode;

  if ((tmp_mode == HAL_I2C_MODE_MASTER) || (tmp_mode == HAL_I2C_MODE_MASTER_MEM))
  {
    p_i2cx = I2C_GET_INSTANCE(hi2c);
    ASSERT_DBG_PARAM((LL_I2C_GetMasterAddressingMode(p_i2cx) == LL_I2C_ADDRESSING_MODE_7BIT) ?
                     IS_I2C_OWN_ADDRESS_7BIT(device_addr) : IS_I2C_OWN_ADDRESS_10BIT(device_addr));

    ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_TX          \
                                          | HAL_I2C_STATE_RX        \
                                          | HAL_I2C_STATE_LISTEN    \
                                          | HAL_I2C_STATE_RX_LISTEN \
                                          | HAL_I2C_STATE_TX_LISTEN));

    /* Disable interrupts and store previous state */
    if (hi2c->global_state == HAL_I2C_STATE_TX)
    {
      I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);
      hi2c->previous_state = I2C_STATE_MASTER_BUSY_TX;
    }
    else if (hi2c->global_state == HAL_I2C_STATE_RX)
    {
      I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);
      hi2c->previous_state = I2C_STATE_MASTER_BUSY_RX;
    }
    else
    {
      /* Do nothing */
    }

    hi2c->global_state = HAL_I2C_STATE_ABORT;

    /* Set NBYTES to 1 to generate a dummy read on I2C peripheral */
    /* Set AUTOEND mode, this generates a NACK then STOP condition to abort the current transfer */
    I2C_TransferConfig(p_i2cx, device_addr, 1, LL_I2C_MODE_AUTOEND, I2C_GENERATE_STOP);

    LL_I2C_EnableIT(p_i2cx, I2C_XFER_CPLT_IT_MASK);

    return HAL_OK;
  }
  else
  {
    /* Wrong usage of abort function */
    /* This function must be used only in case of abort monitored by master device */
    return HAL_ERROR;
  }
}

/**
  * @brief  Abort a slave I2C IT or DMA process communication with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_I2C_SLAVE_Abort_IT(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  p_i2cx = I2C_GET_INSTANCE(hi2c);
  LL_I2C_AcknowledgeNextData(p_i2cx, LL_I2C_NACK);

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group4
  * @{
A set of function to handle the I2C interruptions :
  - I2C Event IRQ Handler : HAL_I2C_EV_IRQHandler()
  - I2C Error IRQ Handler : HAL_I2C_ERR_IRQHandler()

  Depending on the process function one's use, different callback might be triggered:

| Process API \n \ \n Callbacks    |    HAL_I2C_MASTER_Transmit_IT   |   HAL_I2C_MASTER_Receive_IT    |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_MASTER_TxCpltCallback    |                x                |                                |
| HAL_I2C_MASTER_RxCpltCallback    |                                 |                x               |
| HAL_I2C_ErrorCallback            |                x                |                x               |
| HAL_I2C_AbortCpltCallback*       |                x                |                x               |

| Process API \n \ \n Callbacks    |  HAL_I2C_MASTER_SEQ_Transmit_IT | HAL_I2C_MASTER_SEQ_Receive_IT  |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_MASTER_TxCpltCallback    |                x                |                                |
| HAL_I2C_MASTER_RxCpltCallback    |                                 |                x               |
| HAL_I2C_ErrorCallback            |                x                |                x               |
| HAL_I2C_AbortCpltCallback*       |                x                |                x               |

| Process API \n \ \n Callbacks    |   HAL_I2C_MASTER_Transmit_DMA   |   HAL_I2C_MASTER_Receive_DMA   |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_MASTER_TxCpltCallback    |                x                |                                |
| HAL_I2C_MASTER_RxCpltCallback    |                                 |                x               |
| HAL_I2C_ErrorCallback            |                x                |                x               |
| HAL_I2C_AbortCpltCallback*       |                x                |                x               |

| Process API \n \ \n Callbacks    | HAL_I2C_MASTER_SEQ_Transmit_DMA | HAL_I2C_MASTER_SEQ_Receive_DMA |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_MASTER_TxCpltCallback    |                x                |                                |
| HAL_I2C_MASTER_RxCpltCallback    |                                 |                x               |
| HAL_I2C_ErrorCallback            |                x                |                x               |
| HAL_I2C_AbortCpltCallback*       |                x                |                x               |
@note * HAL_I2C_AbortCpltCallback is called by the ISR when the abort is requested by the slave (using NACK) or
        the master (by generating STOP)

| Process API \n \ \n Callbacks    |  HAL_I2C_MASTER_MemWrite_IT  |  HAL_I2C_MASTER_MemRead_IT  |
|----------------------------------|:----------------------------:|:---------------------------:|
| HAL_I2C_MASTER_MemTxCpltCallback |              x               |                             |
| HAL_I2C_MASTER_MemRxCpltCallback |                              |              x              |

| Process API \n \ \n Callbacks    |  HAL_I2C_MASTER_MemWrite_DMA  |  HAL_I2C_MASTER_MemRead_DMA  |
|----------------------------------|:-----------------------------:|:----------------------------:|
| HAL_I2C_MASTER_MemTxCpltCallback |               x               |                              |
| HAL_I2C_MASTER_MemRxCpltCallback |                               |               x              |

| Process API \n \ \n Callbacks    |    HAL_I2C_SLAVE_Transmit_IT    |    HAL_I2C_SLAVE_Receive_IT    |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_SLAVE_TxCpltCallback     |                x                |                                |
| HAL_I2C_SLAVE_RxCpltCallback     |                                 |                  x             |
| HAL_I2C_SLAVE_ListenCpltCallback |                x                |                  x             |
| HAL_I2C_ErrorCallback            |                x                |                  x             |

| Process API \n \ \n Callbacks    |  HAL_I2C_SLAVE_SEQ_Transmit_IT  |  HAL_I2C_SLAVE_SEQ_Receive_IT  |
|----------------------------------|:-------------------------------:|:------------------------------:|
| HAL_I2C_SLAVE_TxCpltCallback     |                x                |                                |
| HAL_I2C_SLAVE_RxCpltCallback     |                                 |                  x             |
| HAL_I2C_SLAVE_ListenCpltCallback |                x                |                  x             |
| HAL_I2C_ErrorCallback            |                x                |                  x             |

| Process API \n \ \n Callbacks    |   HAL_I2C_SLAVE_Transmit_DMA    |   HAL_I2C_SLAVE_Receive_DMA   |
|----------------------------------|:-------------------------------:|:-----------------------------:|
| HAL_I2C_SLAVE_TxCpltCallback     |                x                |                               |
| HAL_I2C_SLAVE_RxCpltCallback     |                                 |                x              |
| HAL_I2C_SLAVE_ListenCpltCallback |                x                |                x              |
| HAL_I2C_ErrorCallback            |                x                |                x              |

| Process API \n \ \n Callbacks    |  HAL_I2C_SLAVE_SEQ_Transmit_DMA  |  HAL_I2C_SLAVE_SEQ_Receive_DMA  |
|----------------------------------|:--------------------------------:|:-------------------------------:|
| HAL_I2C_SLAVE_TxCpltCallback     |                 x                |                                 |
| HAL_I2C_SLAVE_RxCpltCallback     |                                  |                  x              |
| HAL_I2C_SLAVE_ListenCpltCallback |                 x                |                  x              |
| HAL_I2C_ErrorCallback            |                 x                |                  x              |
@note HAL_I2C_SLAVE_EnableListen_IT must be called before HAL_I2C_SLAVE_SEQ_Transmit_IT and
      HAL_I2C_SLAVE_SEQ_Receive_IT

| Process API \n \ \n Callbacks      |  HAL_I2C_SLAVE_EnableListen_IT  |
|------------------------------------|:-------------------------------:|
| HAL_I2C_SLAVE_AddrCallback         |                x                |

| Process API \n \ \n Callbacks    |    HAL_I2C_MASTER_Abort_IT    |     HAL_I2C_SLAVE_Abort_IT    |
|----------------------------------|:-----------------------------:|:-----------------------------:|
| HAL_I2C_AbortCpltCallback        |               x               |               x               |

  */

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  */
void HAL_I2C_EV_IRQHandler(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Get current IT flags and IT sources value */
  uint32_t it_flags   = LL_I2C_READ_REG(p_i2cx, ISR) & I2C_FLAG_MASK;
  uint32_t it_sources = LL_I2C_READ_REG(p_i2cx, CR1);

  /* I2C events treatment -------------------------------------*/
  if (hi2c->xfer_isr != NULL)
  {
    hi2c->xfer_isr(hi2c, it_flags, it_sources);
  }
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  */
void HAL_I2C_ERR_IRQHandler(hal_i2c_handle_t *hi2c)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hi2c != NULL));
  p_i2cx = I2C_GET_INSTANCE(hi2c);

  uint32_t it_flags = LL_I2C_READ_REG(p_i2cx, ISR) & I2C_FLAG_MASK;
  uint32_t it_sources = LL_I2C_READ_REG(p_i2cx, CR1);
  uint32_t tmp_error = HAL_I2C_ERROR_NONE;

  /* I2C bus error interrupt occurred ------------------------------------*/
  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_BERR) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_ERRIE) != 0U))
  {
    tmp_error |= HAL_I2C_ERROR_BERR;
    LL_I2C_ClearFlag_BERR(p_i2cx);
  }

  /* I2C over-run/under-run interrupt occurred ----------------------------------------*/
  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_OVR) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_ERRIE) != 0U))
  {
    tmp_error |= HAL_I2C_ERROR_OVR;
    LL_I2C_ClearFlag_OVR(p_i2cx);
  }

  /* I2C arbitration loss error interrupt occurred -------------------------------------*/
  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_ARLO) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_ERRIE) != 0U))
  {
    tmp_error |= HAL_I2C_ERROR_ARLO;
    LL_I2C_ClearFlag_ARLO(p_i2cx);
  }

  /* Call the error callback in case of error detected */
  if (tmp_error != HAL_I2C_ERROR_NONE)
  {
    I2C_ITError(hi2c, tmp_error);
  }
}

/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group5
  * @{
A set of Weak functions (or default Callbacks functions if USE_HAL_I2C_REGISTER_CALLBACKS is set to 1) which are used
to asynchronously informed the application in non blocking modes (Interrupt and DMA) :
 - HAL_I2C_MASTER_TxCpltCallback()     : Master Tx transfer completed callback
 - HAL_I2C_MASTER_RxCpltCallback()     : Master Rx transfer completed callback
 - HAL_I2C_SLAVE_TxCpltCallback()      : Slave Tx transfer completed callback
 - HAL_I2C_SLAVE_RxCpltCallback()      : Slave Rx transfer completed callback
 - HAL_I2C_MASTER_MemTxCpltCallback()  : Memory Tx transfer completed callback
 - HAL_I2C_MASTER_MemRxCpltCallback()  : Memory Rx transfer completed callback
 - HAL_I2C_SLAVE_AddrCallback()        : Slave Address Match callback
 - HAL_I2C_SLAVE_ListenCpltCallback()  : Listen Complete callback
 - HAL_I2C_ErrorCallback()             : I2C error callback
 - HAL_I2C_AbortCpltCallback()         : I2C abort complete callback
 */

/**
  * @brief  Master Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_MASTER_TxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Master Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_MASTER_RxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Slave Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_SLAVE_TxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Slave Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_SLAVE_RxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  xfer_direction Master request transfer Direction (Write/Read)
  * @param  addr_match_code Address Match Code
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_SLAVE_AddrCallback(hal_i2c_handle_t *hi2c,
                                       hal_i2c_slave_xfer_direction_t xfer_direction,
                                       uint32_t addr_match_code)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
  STM32_UNUSED(xfer_direction);
  STM32_UNUSED(addr_match_code);
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_SLAVE_ListenCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Memory Tx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_MASTER_MemTxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  Memory Rx transfer completed callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_MASTER_MemRxCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_ErrorCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @brief  I2C abort complete callback.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_I2C_AbortCpltCallback(hal_i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hi2c);
}

/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group6
  * @{
A set of functions allowing to retrieve peripheral state, mode and last process errors.
 - HAL_I2C_GetState() : Return the I2C handle state
 - HAL_I2C_GetMode() : Returns the functional I2C mode. master, slave, memory or no mode.
 - HAL_I2C_GetLastErrorCodes() : Returns errors limited to the last process.
 - HAL_I2C_GetClockFreq() : Retrieve the HAL I2C instance kernel clock frequency.
  */

/**
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval hal_i2c_state_t HAL I2C state
  */
hal_i2c_state_t HAL_I2C_GetState(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_INIT        \
                                        | HAL_I2C_STATE_IDLE      \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return hi2c->global_state;
}

/**
  * @brief  Returns the functional I2C mode. master, slave, memory or no mode.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval hal_i2c_mode_t HAL I2C mode
  */
hal_i2c_mode_t HAL_I2C_GetMode(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_TX          \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return hi2c->mode;
}

#if defined (USE_HAL_I2C_GET_LAST_ERRORS) && (USE_HAL_I2C_GET_LAST_ERRORS == 1)
/**
  * @brief  Returns errors limited to the last process.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval uint32_t last error code. It can be HAL_I2C_ERROR_NONE or a combinaison of the following values:
  *         HAL_I2C_ERROR_BERR
  *         HAL_I2C_ERROR_ARLO
  *         HAL_I2C_ERROR_AF
  *         HAL_I2C_ERROR_OVR
  *         HAL_I2C_ERROR_DMA
  *         HAL_I2C_ERROR_SIZE
  */
uint32_t HAL_I2C_GetLastErrorCodes(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_IDLE        \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return hi2c->last_error_codes;
}
#endif /* USE_HAL_I2C_GET_LAST_ERRORS */

/** @brief  Return the peripheral clock frequency for I2C.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @retval uint32_t Frequency in Hz.
  *                  0 if the source clock of the I2C is not configured or not ready.
  */
uint32_t HAL_I2C_GetClockFreq(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_INIT        \
                                        | HAL_I2C_STATE_IDLE      \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  return HAL_RCC_I2C_GetKernelClkFreq(I2C_GET_INSTANCE(hi2c));
}

/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup I2C_Exported_Functions_Group7
  * @{
A set of functions allowing to Acquire/Release the bus based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
 - HAL_I2C_AcquireBus() Acquire the I2C bus.
 - HAL_I2C_ReleaseBus() Release the I2C bus.
  */

/**
  * @brief  Acquire the I2C bus thanks to the the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  timeout_ms Timeout duration in millisecond.
  * @note   The HAL_I2C_AcquireBus must be called from thread mode only (not from handler mode i.e from ISR).
  * @retval HAL_OK    Operation completed successfully
  * @retval HAL_ERROR Operation completed with error
  */
hal_status_t HAL_I2C_AcquireBus(hal_i2c_handle_t *hi2c, uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_INIT        \
                                        | HAL_I2C_STATE_IDLE      \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  if (HAL_OS_SemaphoreTake(&hi2c->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the I2C bus thanks to the the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @note   The HAL_I2C_ReleaseBus can be called from thread mode or from handler mode i.e from ISR.
  * @retval HAL_OK    Operation completed successfully
  * @retval HAL_ERROR Operation completed with error
  */
hal_status_t HAL_I2C_ReleaseBus(hal_i2c_handle_t *hi2c)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hi2c != NULL));
  ASSERT_DBG_STATE(hi2c->global_state, (HAL_I2C_STATE_INIT        \
                                        | HAL_I2C_STATE_IDLE      \
                                        | HAL_I2C_STATE_TX        \
                                        | HAL_I2C_STATE_RX        \
                                        | HAL_I2C_STATE_LISTEN    \
                                        | HAL_I2C_STATE_RX_LISTEN \
                                        | HAL_I2C_STATE_TX_LISTEN \
                                        | HAL_I2C_STATE_ABORT));

  if (HAL_OS_SemaphoreRelease(&hi2c->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_I2C_USER_DATA) && (USE_HAL_I2C_USER_DATA == 1)
/** @addtogroup I2C_Exported_Functions_Group8
  * @{
A set of functions allowing to manage a user data pointer stored to the I2C handle:
 - HAL_I2C_SetUserData() Set the user data into the handle
 - HAL_I2C_GetUserData() Get the user data from the handle
  */

/**
  * @brief Set the user data pointer into the handle.
  * @param hi2c Pointer to a hal_i2c_handle_t
  * @param p_user_data Pointer to the user data.
  */
void HAL_I2C_SetUserData(hal_i2c_handle_t *hi2c, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hi2c != NULL);

  hi2c->p_user_data = p_user_data;
}

/**
  * @brief Get the user data pointer from the handle.
  * @param hi2c Pointer to a hal_i2c_handle_t
  * @retval void* Pointer to the user data.
  */
const void *HAL_I2C_GetUserData(const hal_i2c_handle_t *hi2c)
{
  ASSERT_DBG_PARAM(hi2c != NULL);

  return (hi2c->p_user_data);
}
/**
  * @}
  */
#endif /* USE_HAL_I2C_USER_DATA */

/**
  * @}
  */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Interrupt sub-routine which handle the interrupt flags master mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Master_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t dev_addr;
  uint32_t tmp_it_flags = it_flags;

  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_NACKF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    /* No need to generate STOP, it is automatically done */
    /* Error callback is send during stop flag treatment */
    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

    I2C_Flush_TXDR(p_i2cx);
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_RXNE) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_RXIE) != 0U))
  {
    /* Remove RXNE flag on temporary variable as read done */
    tmp_it_flags &= ~LL_I2C_ISR_RXNE;

    /* Read data from RXDR */
    *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
    hi2c->p_buf_rx++;
    hi2c->xfer_size--;
    hi2c->xfer_count--;
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TXIS) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TXIE) != 0U))
  {
    /* Write data to TXDR */
    /* Write data to TXDR */
    LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
    hi2c->p_buf_tx++;
    hi2c->xfer_size--;
    hi2c->xfer_count--;
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TCR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
    {
      dev_addr = LL_I2C_GetSlaveAddr(p_i2cx);

      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        I2C_TransferConfig(p_i2cx, dev_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        if (hi2c->xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION)
        {
          I2C_TransferConfig(p_i2cx, dev_addr, hi2c->xfer_size, (uint32_t)hi2c->xfer_opt, I2C_NO_STARTSTOP);
        }
        else
        {
          I2C_TransferConfig(p_i2cx, dev_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
        }
      }
    }
    else
    {
      /* Call TxCpltCallback() if auto end mode is set */
      if (LL_I2C_IsEnabledAutoEndMode(p_i2cx) == 0U)
      {
        /* Call I2C master Sequential complete process */
        I2C_ITMasterSeqCplt(hi2c);
      }
      else
      {
        /* Wrong size status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of end of transfer */
        I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TC) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    if (hi2c->xfer_count == 0U)
    {
      if (LL_I2C_IsEnabledAutoEndMode(p_i2cx) == 0U)
      {
        /* Generate a stop condition in case of no transfer option */
        if (hi2c->xfer_opt == (hal_i2c_xfer_opt_t) XFER_NO_OPTION)
        {
          LL_I2C_GenerateStopCondition(p_i2cx);
        }
        else
        {
          /* Call I2C master sequential complete process */
          I2C_ITMasterSeqCplt(hi2c);
        }
      }
    }
    else
    {
      /* Wrong size status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else
  {
    /* Nothing to do */
  }

  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_STOPF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C master complete process */
    I2C_ITMasterCplt(hi2c, tmp_it_flags);
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt sub-routine which handle the interrupt flags memory mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t that contains the configuration information for the specified I2C.
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Mem_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  i2c_start_stop_mode_t direction = I2C_GENERATE_START_WRITE;
  uint32_t tmp_it_flags = it_flags;

  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_NACKF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    /* No need to generate STOP, it is automatically done */
    /* Error callback is sent during stop flag treatment */
    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

    I2C_Flush_TXDR(p_i2cx);
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_RXNE) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_RXIE) != 0U))
  {
    /* Remove RXNE flag on temporary variable as read done */
    tmp_it_flags &= ~LL_I2C_ISR_RXNE;

    /* Read data from RXDR */
    *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
    hi2c->p_buf_rx++;
    hi2c->xfer_size--;
    hi2c->xfer_count--;
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TXIS) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TXIE) != 0U))
  {
    if (hi2c->mem_addr == 0xFFFFFFFFU)
    {
      /* Write data to TXDR */
      LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
      hi2c->p_buf_tx++;
      hi2c->xfer_size--;
      hi2c->xfer_count--;
    }
    else
    {
      /* Write LSB part of Memory Address */
      LL_I2C_TransmitData8(p_i2cx, (uint8_t) hi2c->mem_addr);

      /* Reset mem_addr content */
      hi2c->mem_addr = 0xFFFFFFFFU;
    }
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TCR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    if ((hi2c->xfer_count != 0U) && (hi2c->xfer_size == 0U))
    {
      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
      }
    }
    else
    {
      /* Wrong size status regarding TCR flag event */
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TC) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    /* Disable interrupt related to address step */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    /* Enable ERR, TC, STOP, NACK and RXI interrupts */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_RX_IT_MASK);

    if (hi2c->global_state == HAL_I2C_STATE_RX)
    {
      direction = I2C_GENERATE_START_READ;
    }

    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;

      /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, direction);
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;

      /* Set NBYTES to write and generate RESTART */
      I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, direction);
    }
  }
  else
  {
    /* Nothing to do */
  }

  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_STOPF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C master complete process */
    I2C_ITMasterCplt(hi2c, tmp_it_flags);
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt sub-routine which handle the interrupt flags slave mode with interrupt.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Slave_ISR_IT(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  hal_i2c_xfer_opt_t xfer_opt = hi2c->xfer_opt;
  uint32_t tmp_it_flags = it_flags;

  /* Check if STOPF is set */
  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_STOPF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C slave complete process */
    I2C_ITSlaveCplt(hi2c, tmp_it_flags);
  }

  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_NACKF) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when transfer is finished */
    /* Mean xfer_count == 0 */
    /* So clear flag NACKF only */
    if (hi2c->xfer_count == 0U)
    {
      if ((hi2c->global_state == HAL_I2C_STATE_LISTEN) && (xfer_opt == HAL_I2C_XFER_FIRST_AND_LAST_FRAME))
        /* Same action must be done for (xfer_opt == HAL_I2C_XFER_LAST_FRAME) which removed for
           Warning[Pa134]: left and right operands are identical */
      {
        /* Call I2C Listen complete process */
        I2C_ITListenCplt(hi2c, tmp_it_flags);
      }
      else if ((hi2c->global_state == HAL_I2C_STATE_TX_LISTEN) && (xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION))
      {
        LL_I2C_ClearFlag_NACK(p_i2cx);

        I2C_Flush_TXDR(p_i2cx);

        /* Last byte is transmitted */
        /* Call I2C slave sequential complete process */
        I2C_ITSlaveSeqCplt(hi2c);
      }
      else
      {
        LL_I2C_ClearFlag_NACK(p_i2cx);
      }
    }
    else
    {
      /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER */
      LL_I2C_ClearFlag_NACK(p_i2cx);

      /* Set error_code corresponding to a Non-Acknowledge */
      hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

      if ((xfer_opt == HAL_I2C_XFER_FIRST_FRAME) || (xfer_opt == HAL_I2C_XFER_NEXT_FRAME))
      {
        /* Call the corresponding callback to inform upper layer of end of transfer */
        I2C_ITError(hi2c, hi2c->last_error_codes);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_RXNE) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_RXIE) != 0U))
  {
    if (hi2c->xfer_count > 0U)
    {
      /* Read data from RXDR */
      *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
      hi2c->p_buf_rx++;
      hi2c->xfer_size--;
      hi2c->xfer_count--;
    }

    if ((hi2c->xfer_count == 0U)
        && (xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION))
    {
      /* Call I2C slave sequential complete process */
      I2C_ITSlaveSeqCplt(hi2c);
    }
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_ADDR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_ADDRIE) != 0U))
  {
    I2C_ITAddrCplt(hi2c, tmp_it_flags);
  }
  else if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_TXIS) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TXIE) != 0U))
  {
    /* Write data to TXDR only if xfer_count not reach "0" */
    /* A TXIS flag can be set, during STOP treatment      */
    /* Check if all Data have already been sent */
    /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
    if (hi2c->xfer_count > 0U)
    {
      /* Write data to TXDR */
      LL_I2C_TransmitData8(p_i2cx, *hi2c->p_buf_tx);
      hi2c->p_buf_tx++;
      hi2c->xfer_count--;
      hi2c->xfer_size--;
    }
    else
    {
      if ((xfer_opt == HAL_I2C_XFER_NEXT_FRAME) || (xfer_opt == HAL_I2C_XFER_FIRST_FRAME))
      {
        /* Last byte is transmitted */
        /* Call I2C slave sequential complete process */
        I2C_ITSlaveSeqCplt(hi2c);
      }
    }
  }
  else
  {
    /* Nothing to do */
  }

  return HAL_OK;
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  Interrupt sub-routine which handle the interrupt flags master mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Master_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t dev_addr;
  uint32_t xfer_mode;

  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_NACKF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

    /* No need to generate STOP, it is automatically done */
    /* But enable STOP interrupt, to treat it */
    /* Error callback is sent during stop flag treatment */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_CPLT_IT_DMA_MASK);

    I2C_Flush_TXDR(p_i2cx);
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_TCR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    /* Disable transfer complete interrupt */
    LL_I2C_DisableIT(p_i2cx, LL_I2C_CR1_TCIE);

    if (hi2c->xfer_count != 0U)
    {
      /* Recover slave address */
      dev_addr = LL_I2C_GetSlaveAddr(p_i2cx);

      /* Prepare the new xfer_size to transfer */
      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        xfer_mode = LL_I2C_MODE_RELOAD;
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        if (hi2c->xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION)
        {
          xfer_mode = (uint32_t) hi2c->xfer_opt;
        }
        else
        {
          xfer_mode = LL_I2C_MODE_AUTOEND;
        }
      }

      /* Set the new xfer_size in Nbytes register */
      I2C_TransferConfig(p_i2cx, dev_addr, hi2c->xfer_size, xfer_mode, I2C_NO_STARTSTOP);

      hi2c->xfer_count -= hi2c->xfer_size;

      if (hi2c->global_state == HAL_I2C_STATE_RX)
      {
        LL_I2C_EnableDMAReq_RX(p_i2cx);
      }
      else
      {
        LL_I2C_EnableDMAReq_TX(p_i2cx);
      }
    }
    else
    {
      /* Call TxCpltCallback() if auto end mode is set */
      if (LL_I2C_IsEnabledAutoEndMode(p_i2cx) == 0U)
      {
        /* Call I2C master Sequential complete process */
        I2C_ITMasterSeqCplt(hi2c);
      }
      else
      {
        /* Wrong size status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of end of transfer */
        I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_TC) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    if (hi2c->xfer_count == 0U)
    {
      if (LL_I2C_IsEnabledAutoEndMode(p_i2cx) == 0U)
      {
        /* Generate a stop condition in case of no transfer option */
        if (hi2c->xfer_opt == (hal_i2c_xfer_opt_t) XFER_NO_OPTION)
        {
          LL_I2C_GenerateStopCondition(p_i2cx);
        }
        else
        {
          /* Call I2C master Sequential complete process */
          I2C_ITMasterSeqCplt(hi2c);
        }
      }
    }
    else
    {
      /* Wrong size status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_STOPF) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C master complete process */
    I2C_ITMasterCplt(hi2c, it_flags);
  }
  else
  {
    /* Nothing to do */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt sub-routine which handle the interrupt flags memory mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Mem_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  i2c_start_stop_mode_t direction = I2C_GENERATE_START_WRITE;

  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_NACKF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

    /* No need to generate STOP, it is automatically done */
    /* But enable STOP interrupt, to treat it */
    /* Error callback is send during stop flag treatment */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_CPLT_IT_DMA_MASK);

    I2C_Flush_TXDR(p_i2cx);
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_TXIS) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TXIE) != 0U))
  {
    /* Write LSB part of Memory Address */
    LL_I2C_TransmitData8(p_i2cx, (uint8_t) hi2c->mem_addr);

    /* Reset mem_addr content */
    hi2c->mem_addr = 0xFFFFFFFFU;
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_TCR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    /* Disable interrupt related to address step */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    /* Enable only Error interrupt */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

    if (hi2c->xfer_count != 0U)
    {
      /* Prepare the new xfer_size to transfer */
      if (hi2c->xfer_count > MAX_NBYTE_SIZE)
      {
        hi2c->xfer_size = MAX_NBYTE_SIZE;
        I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, I2C_NO_STARTSTOP);
      }
      else
      {
        hi2c->xfer_size = hi2c->xfer_count;
        I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, I2C_NO_STARTSTOP);
      }

      hi2c->xfer_count -= hi2c->xfer_size;

      if (hi2c->global_state == HAL_I2C_STATE_RX)
      {
        LL_I2C_EnableDMAReq_RX(p_i2cx);
      }
      else
      {
        LL_I2C_EnableDMAReq_TX(p_i2cx);
      }
    }
    else
    {
      /* Wrong size status regarding TCR flag event */
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_TC) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_TCIE) != 0U))
  {
    /* Disable interrupt related to address step */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    /* Enable only Error and NACK interrupt for data transfer */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_ERROR_IT_MASK);

    if (hi2c->global_state == HAL_I2C_STATE_RX)
    {
      direction = I2C_GENERATE_START_READ;
    }

    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;

      /* Set NBYTES to write and reload if hi2c->xfer_count > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_RELOAD, direction);
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;

      /* Set NBYTES to write and generate RESTART */
      I2C_TransferConfig(p_i2cx, (uint32_t)hi2c->dev_addr, hi2c->xfer_size, LL_I2C_MODE_AUTOEND, direction);
    }

    hi2c->xfer_count -= hi2c->xfer_size;

    if (hi2c->global_state == HAL_I2C_STATE_RX)
    {
      LL_I2C_EnableDMAReq_RX(p_i2cx);
    }
    else
    {
      LL_I2C_EnableDMAReq_TX(p_i2cx);
    }
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_STOPF) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C master complete process */
    I2C_ITMasterCplt(hi2c, it_flags);
  }
  else
  {
    /* Nothing to do */
  }

  return HAL_OK;
}

/**
  * @brief  Interrupt sub-routine which handle the interrupt flags slave mode with DMA.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags Interrupt flags to handle
  * @param  it_sources Interrupt sources enabled
  * @retval HAL_OK Operation completed successfully
  */
static hal_status_t I2C_Slave_ISR_DMA(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t it_sources)
{
  hal_i2c_xfer_opt_t xfer_opt = hi2c->xfer_opt;
  uint32_t treat_dma_nack = 0U;
  hal_i2c_state_t tmp_state;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Check if STOPF is set */
  if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_STOPF) != 0U)
      && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    /* Call I2C slave complete process */
    I2C_ITSlaveCplt(hi2c, it_flags);
  }

  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_NACKF) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_NACKIE) != 0U))
  {
    /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when transfer is finished */
    /* Mean xfer_count == 0 */
    /* So clear flag NACKF only */
    if ((I2C_CHECK_IT_SOURCE(it_sources, I2C_CR1_TXDMAEN) != 0U)
        || (I2C_CHECK_IT_SOURCE(it_sources, I2C_CR1_RXDMAEN) != 0U))
    {
      /* Split check of hdma_rx, for MISRA compliance */
      if (hi2c->hdma_rx != NULL)
      {
        if (I2C_CHECK_IT_SOURCE(it_sources, I2C_CR1_RXDMAEN) != 0U)
        {
          if (HAL_DMA_GetDirectXferRemainingDataByte(hi2c->hdma_rx) == 0U)
          {
            treat_dma_nack = 1U;
          }
        }
      }

      /* Split check of hdma_tx, for MISRA compliance  */
      if (hi2c->hdma_tx != NULL)
      {
        if (I2C_CHECK_IT_SOURCE(it_sources, I2C_CR1_TXDMAEN) != 0U)
        {
          if (HAL_DMA_GetDirectXferRemainingDataByte(hi2c->hdma_tx) == 0U)
          {
            treat_dma_nack = 1U;
          }
        }
      }

      if (treat_dma_nack != 0U)
      {
        if ((hi2c->global_state == HAL_I2C_STATE_LISTEN) && (xfer_opt == HAL_I2C_XFER_FIRST_AND_LAST_FRAME))
          /* Same action must be done for (xfer_opt == HAL_I2C_XFER_LAST_FRAME) which removed for
             Warning[Pa134]: left and right operands are identical */
        {
          /* Call I2C Listen complete process */
          I2C_ITListenCplt(hi2c, it_flags);
        }
        else if ((hi2c->global_state == HAL_I2C_STATE_TX_LISTEN) && (xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION))
        {
          LL_I2C_ClearFlag_NACK(p_i2cx);

          I2C_Flush_TXDR(p_i2cx);

          /* Last byte is transmitted */
          /* Call I2C slave sequential complete process */
          I2C_ITSlaveSeqCplt(hi2c);
        }
        else
        {
          LL_I2C_ClearFlag_NACK(p_i2cx);
        }
      }
      else
      {
        /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER */
        LL_I2C_ClearFlag_NACK(p_i2cx);

        /* Set error_code corresponding to a Non-Acknowledge */
        hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

        /* Store current hi2c->global_state, solve MISRA2012-Rule-13.5 */
        tmp_state = hi2c->global_state;

        if ((xfer_opt == HAL_I2C_XFER_FIRST_FRAME) || (xfer_opt == HAL_I2C_XFER_NEXT_FRAME))
        {
          if ((tmp_state == HAL_I2C_STATE_TX) || (tmp_state == HAL_I2C_STATE_TX_LISTEN))
          {
            hi2c->previous_state = I2C_STATE_SLAVE_BUSY_TX;
          }
          else if ((tmp_state == HAL_I2C_STATE_RX) || (tmp_state == HAL_I2C_STATE_RX_LISTEN))
          {
            hi2c->previous_state = I2C_STATE_SLAVE_BUSY_RX;
          }
          else
          {
            /* Do nothing */
          }

          /* Call the corresponding callback to inform upper layer of end of transfer */
          I2C_ITError(hi2c, hi2c->last_error_codes);
        }
      }
    }
    else
    {
      /* Only Clear NACK flag, no DMA treatment is pending */
      LL_I2C_ClearFlag_NACK(p_i2cx);
    }
  }
  else if ((I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_ADDR) != 0U)
           && (I2C_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_ADDRIE) != 0U))
  {
    I2C_ITAddrCplt(hi2c, it_flags);
  }
  else
  {
    /* Nothing to do */
  }

  return HAL_OK;
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_RequestMemoryWrite(hal_i2c_handle_t *hi2c, uint32_t device_addr,
                                           uint32_t memory_addr, hal_i2c_mem_addr_size_t memory_addr_size,
                                           uint32_t timeout_ms, uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  hal_status_t hal_status;

  I2C_TransferConfig(p_i2cx, device_addr, (uint32_t)memory_addr_size, LL_I2C_MODE_RELOAD, I2C_GENERATE_START_WRITE);

  hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
  if (hal_status == HAL_OK)
  {
    /* If memory address size is 8Bit */
    if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
    {
      /* Send Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));
    }
    /* If memory address size is 16Bit */
    else
    {
      /* Send MSB of Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

      hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status != HAL_OK)
      {
        return hal_status;
      }

      /* Send LSB of Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));
    }

    /* Wait until TCR flag is set */
    hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TCR, 0U, timeout_ms, tick_start);
  }

  return hal_status;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  memory_addr Internal memory address
  * @param  memory_addr_size Size of internal memory address
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_RequestMemoryRead(hal_i2c_handle_t *hi2c, uint32_t device_addr,
                                          uint32_t memory_addr, hal_i2c_mem_addr_size_t memory_addr_size,
                                          uint32_t timeout_ms, uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  hal_status_t hal_status;

  I2C_TransferConfig(p_i2cx, device_addr, (uint32_t)memory_addr_size, LL_I2C_MODE_SOFTEND, I2C_GENERATE_START_WRITE);

  hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
  if (hal_status == HAL_OK)
  {
    /* If memory address size is 8Bit */
    if (memory_addr_size == HAL_I2C_MEM_ADDR_8BIT)
    {
      /* Send Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));
    }
    /* If memory address size is 16Bit */
    else
    {
      /* Send MSB of Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_MSB(memory_addr));

      hal_status = I2C_WaitOnTXISFlagUntilTimeout(hi2c, timeout_ms, tick_start);
      if (hal_status != HAL_OK)
      {
        return hal_status;
      }

      /* Send LSB of Memory Address */
      LL_I2C_TransmitData8(p_i2cx, I2C_MEM_ADD_LSB(memory_addr));
    }

    /* Wait until TC flag is set */
    hal_status = I2C_WaitOnFlagUntilTimeout(hi2c, LL_I2C_ISR_TC, 0U, timeout_ms, tick_start);
  }

  return hal_status;
}

/**
  * @brief  I2C Address complete process callback.
  * @param  hi2c I2C handle
  * @param  it_flags Interrupt flags to handle
  */
static void I2C_ITAddrCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t xfer_direction;
  uint32_t slave_addr_code;
  uint32_t own_addr1_code;
  uint32_t own_addr2_code;

  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(it_flags);

  /* In case of Listen state, need to inform upper layer of address match code event */
  if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                       | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) != 0U)
  {
    xfer_direction = LL_I2C_GetTransferDirection(p_i2cx);
    slave_addr_code = LL_I2C_GetAddressMatchCode(p_i2cx);
    own_addr1_code = LL_I2C_GetOwnAddress1(p_i2cx);
    own_addr2_code = LL_I2C_GetOwnAddress2(p_i2cx);

    /* If 10bits addressing mode is selected */
    if ((hal_i2c_addressing_mode_t)LL_I2C_GetMasterAddressingMode(p_i2cx) == HAL_I2C_ADDRESSING_10BIT)
    {
      if ((slave_addr_code & SLAVE_ADDR_MSK) == ((own_addr1_code >> SLAVE_ADDR_SHIFT) & SLAVE_ADDR_MSK))
      {
        slave_addr_code = own_addr1_code;
        hi2c->addr_event_count++;
        if (hi2c->addr_event_count == 2U)
        {
          hi2c->addr_event_count = 0U;
          LL_I2C_ClearFlag_ADDR(p_i2cx);

          /* Call slave Addr callback */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->p_slave_addr_cb(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#else
          HAL_I2C_SLAVE_AddrCallback(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
        }
      }
      else
      {
        slave_addr_code = own_addr2_code;
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        /* Call slave Addr callback */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->p_slave_addr_cb(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#else
        HAL_I2C_SLAVE_AddrCallback(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
      }
    }
    /* else 7 bits addressing mode is selected */
    else
    {
      /* Disable ADDR interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

      /* Call slave Addr callback */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->p_slave_addr_cb(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#else
      HAL_I2C_SLAVE_AddrCallback(hi2c, (hal_i2c_slave_xfer_direction_t)xfer_direction, slave_addr_code);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
  }
  /* Else clear address flag only */
  else
  {
    LL_I2C_ClearFlag_ADDR(p_i2cx);

  }
}

/**
  * @brief  I2C master sequential complete process.
  * @param  hi2c I2C handle
  */
static void I2C_ITMasterSeqCplt(hal_i2c_handle_t *hi2c)
{
  hi2c->mode = HAL_I2C_MODE_NONE;

  /* No Generate Stop, to permit restart mode */
  /* The stop is done at the end of transfer, when LL_I2C_MODE_AUTOEND enable */
  if (hi2c->global_state == HAL_I2C_STATE_TX)
  {
    hi2c->previous_state = I2C_STATE_MASTER_BUSY_TX;
    hi2c->xfer_isr = NULL;
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_master_tx_cplt_cb(hi2c);
#else
    HAL_I2C_MASTER_TxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  /* hi2c->global_state == HAL_I2C_STATE_RX */
  else
  {
    hi2c->previous_state = I2C_STATE_MASTER_BUSY_RX;
    hi2c->xfer_isr = NULL;
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_master_rx_cplt_cb(hi2c);
#else
    HAL_I2C_MASTER_RxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  I2C slave sequential complete process.
  * @param  hi2c I2C handle
  */
static void I2C_ITSlaveSeqCplt(hal_i2c_handle_t *hi2c)
{
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t tmp_cr1_value = LL_I2C_READ_REG(p_i2cx, CR1);
#endif /* USE_HAL_I2C_DMA */

  hi2c->mode = HAL_I2C_MODE_NONE;

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  /* If a DMA is ongoing, Update handle size context */
  if (I2C_CHECK_IT_SOURCE(tmp_cr1_value, I2C_CR1_TXDMAEN) != 0U)
  {
    LL_I2C_DisableDMAReq_TX(p_i2cx);
  }
  else if (I2C_CHECK_IT_SOURCE(tmp_cr1_value, I2C_CR1_RXDMAEN) != 0U)
  {
    LL_I2C_DisableDMAReq_RX(p_i2cx);
  }
  else
  {
    /* Do nothing */
  }
#endif /* USE_HAL_I2C_DMA */

  if (hi2c->global_state == HAL_I2C_STATE_TX_LISTEN)
  {
    /* Remove HAL_I2C_STATE_SLAVE_BUSY_TX, keep only HAL_I2C_STATE_LISTEN */
    hi2c->previous_state = I2C_STATE_SLAVE_BUSY_TX;

    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    hi2c->global_state = HAL_I2C_STATE_LISTEN;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_slave_tx_cplt_cb(hi2c);
#else
    HAL_I2C_SLAVE_TxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }

  else if (hi2c->global_state == HAL_I2C_STATE_RX_LISTEN)
  {
    /* Remove HAL_I2C_STATE_SLAVE_BUSY_RX, keep only HAL_I2C_STATE_LISTEN */
    hi2c->previous_state = I2C_STATE_SLAVE_BUSY_RX;

    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

    hi2c->global_state = HAL_I2C_STATE_LISTEN;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_slave_rx_cplt_cb(hi2c);
#else
    HAL_I2C_SLAVE_RxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  I2C master complete process.
  * @param  hi2c I2C handle
  * @param  it_flags Interrupt flags to handle
  */
static void I2C_ITMasterCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t tmp_error;
  uint32_t tmp_it_flags = it_flags;
  volatile uint32_t tmp_reg;

  LL_I2C_ClearFlag_STOP(p_i2cx);

  /* Disable interrupts and store previous state */
  if (hi2c->global_state == HAL_I2C_STATE_TX)
  {
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);
    hi2c->previous_state = I2C_STATE_MASTER_BUSY_TX;
  }
  else if (hi2c->global_state == HAL_I2C_STATE_RX)
  {
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);
    hi2c->previous_state = I2C_STATE_MASTER_BUSY_RX;
  }
  else
  {
    /* Do nothing */
  }

  I2C_RESET_CR2(p_i2cx);

  /* Reset handle parameters */
  hi2c->xfer_isr = NULL;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;

  if (I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_NACKF) != 0U)
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);
    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;
  }

  /* Fetch last receive data if any */
  if ((hi2c->global_state == HAL_I2C_STATE_ABORT) && (I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_RXNE) != 0U))
  {
    /* Read data from RXDR */
    tmp_reg = LL_I2C_ReceiveData8(p_i2cx);
    STM32_UNUSED(tmp_reg);
  }

  I2C_Flush_TXDR(p_i2cx);
  tmp_error = hi2c->last_error_codes;

  if ((hi2c->global_state == HAL_I2C_STATE_ABORT) || (tmp_error != HAL_I2C_ERROR_NONE))
  {
    /* Call the corresponding callback to inform upper layer of end of transfer */
    I2C_ITError(hi2c, hi2c->last_error_codes);
  }
  /* hi2c->global_state == HAL_I2C_STATE_TX */
  else if (hi2c->global_state == HAL_I2C_STATE_TX)
  {
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    if (hi2c->mode == HAL_I2C_MODE_MASTER_MEM)
    {
      hi2c->mode = HAL_I2C_MODE_NONE;

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->p_mem_tx_cplt_cb(hi2c);
#else
      HAL_I2C_MASTER_MemTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      hi2c->mode = HAL_I2C_MODE_NONE;

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->p_master_tx_cplt_cb(hi2c);
#else
      HAL_I2C_MASTER_TxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
  }
  /* hi2c->global_state == HAL_I2C_STATE_RX */
  else if (hi2c->global_state == HAL_I2C_STATE_RX)
  {
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    if (hi2c->mode == HAL_I2C_MODE_MASTER_MEM)
    {
      hi2c->mode = HAL_I2C_MODE_NONE;

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->p_mem_rx_cplt_cb(hi2c);
#else
      HAL_I2C_MASTER_MemRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      hi2c->mode = HAL_I2C_MODE_NONE;

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->p_master_rx_cplt_cb(hi2c);
#else
      HAL_I2C_MASTER_RxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  I2C slave complete process.
  * @param  hi2c I2C handle
  * @param  it_flags Interrupt flags to handle
  */
static void I2C_ITSlaveCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t tmp_cr1_value = LL_I2C_READ_REG(p_i2cx, CR1);
  uint32_t tmp_it_flags = it_flags;
  hal_i2c_xfer_opt_t xfer_opt = hi2c->xfer_opt;
  hal_i2c_state_t tmp_state = hi2c->global_state;

  LL_I2C_ClearFlag_STOP(p_i2cx);

  /* Disable interrupts and store previous state */
  if ((tmp_state == HAL_I2C_STATE_TX) || (tmp_state == HAL_I2C_STATE_TX_LISTEN)
      || (tmp_state == HAL_I2C_STATE_LISTEN))
  {
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);
    hi2c->previous_state = I2C_STATE_SLAVE_BUSY_TX;
  }
  else if ((tmp_state == HAL_I2C_STATE_RX) || (tmp_state == HAL_I2C_STATE_RX_LISTEN))
  {
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);
    hi2c->previous_state = I2C_STATE_SLAVE_BUSY_RX;
  }
  else
  {
    /* Do nothing */
  }

  I2C_RESET_CR2(p_i2cx);

  I2C_Flush_TXDR(p_i2cx);

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  /* If a DMA is ongoing, Update handle size context */
  if (I2C_CHECK_IT_SOURCE(tmp_cr1_value, I2C_CR1_TXDMAEN) != 0U)
  {
    LL_I2C_DisableDMAReq_TX(p_i2cx);

    if (hi2c->hdma_tx != NULL)
    {
      hi2c->xfer_count = (uint32_t)HAL_DMA_GetDirectXferRemainingDataByte(hi2c->hdma_tx);
    }
  }
  else if (I2C_CHECK_IT_SOURCE(tmp_cr1_value, I2C_CR1_RXDMAEN) != 0U)
  {
    LL_I2C_DisableDMAReq_RX(p_i2cx);

    if (hi2c->hdma_rx != NULL)
    {
      hi2c->xfer_count = (uint32_t)HAL_DMA_GetDirectXferRemainingDataByte(hi2c->hdma_rx);
    }
  }
  else
  {
    /* Do nothing */
  }
#endif /* USE_HAL_I2C_DMA */

  /* Store last receive data if any */
  if (I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_RXNE) != 0U)
  {
    /* Remove RXNE flag on temporary variable as read done */
    tmp_it_flags &= ~LL_I2C_ISR_RXNE;

    /* Read data from RXDR */
    *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
    hi2c->p_buf_rx++;
    if ((hi2c->xfer_size > 0U))
    {
      hi2c->xfer_size--;
      hi2c->xfer_count--;
    }
  }

  /* All data are not transferred, so set error code accordingly */
  if (hi2c->xfer_count != 0U)
  {
    /* Set error_code corresponding to a Non-Acknowledge */
    hi2c->last_error_codes |= HAL_I2C_ERROR_AF;
  }

  if ((I2C_CHECK_FLAG(tmp_it_flags, LL_I2C_ISR_NACKF) != 0U)
      && (I2C_CHECK_IT_SOURCE(tmp_cr1_value, LL_I2C_CR1_NACKIE) != 0U))
  {
    /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean xfer_count == 0*/
    /* So clear Flag NACKF only */
    if (hi2c->xfer_count == 0U)
    {
      if ((hi2c->global_state == HAL_I2C_STATE_LISTEN) && (xfer_opt == HAL_I2C_XFER_FIRST_AND_LAST_FRAME))
        /* Same action must be done for (xfer_opt == I2C_LAST_FRAME) which removed for
           Warning[Pa134]: left and right operands are identical */
      {
        /* Call I2C Listen complete process */
        I2C_ITListenCplt(hi2c, tmp_it_flags);
      }
      else if ((hi2c->global_state == HAL_I2C_STATE_TX_LISTEN) && (xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION))
      {
        /* Clear NACK Flag */
        LL_I2C_ClearFlag_NACK(p_i2cx);

        /* Flush TX register */
        I2C_Flush_TXDR(p_i2cx);

        /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
        I2C_ITSlaveSeqCplt(hi2c);
      }
      else
      {
        /* Clear NACK Flag */
        LL_I2C_ClearFlag_NACK(p_i2cx);
      }
    }
    else
    {
      /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
      LL_I2C_ClearFlag_NACK(p_i2cx);

      /* Set last_error_codes corresponding to a Non-Acknowledge */
      hi2c->last_error_codes |= HAL_I2C_ERROR_AF;

      if ((xfer_opt == HAL_I2C_XFER_FIRST_FRAME) || (xfer_opt == HAL_I2C_XFER_NEXT_FRAME))
      {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, hi2c->last_error_codes);
      }
    }
  }

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->xfer_isr = NULL;

  if (hi2c->last_error_codes != HAL_I2C_ERROR_NONE)
  {
    /* Call the corresponding callback to inform upper layer of end of transfer */
    I2C_ITError(hi2c, hi2c->last_error_codes);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    if (hi2c->global_state == HAL_I2C_STATE_LISTEN)
    {
      /* Call I2C Listen complete process */
      I2C_ITListenCplt(hi2c, tmp_it_flags);
    }
  }
  else if (hi2c->xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION)
  {
    /* Call the Sequential Complete callback, to inform upper layer of the end of transfer */
    I2C_ITSlaveSeqCplt(hi2c);

    hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_slave_listen_cplt_cb(hi2c);
#else
    HAL_I2C_SLAVE_ListenCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  /* Call the corresponding callback to inform upper layer of end of transfer */
  else if (hi2c->global_state == HAL_I2C_STATE_RX)
  {
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_slave_rx_cplt_cb(hi2c);
#else
    HAL_I2C_SLAVE_RxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_slave_tx_cplt_cb(hi2c);
#else
    HAL_I2C_SLAVE_TxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  I2C Listen complete process.
  * @param  hi2c I2C handle
  * @param  it_flags Interrupt flags to handle
  */
static void I2C_ITListenCplt(hal_i2c_handle_t *hi2c, uint32_t it_flags)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  /* Reset handle parameters */
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->previous_state = I2C_STATE_NONE;
  hi2c->xfer_isr = NULL;

  /* Store last receive data if any */
  if (I2C_CHECK_FLAG(it_flags, LL_I2C_ISR_RXNE) != 0U)
  {
    /* Read data from RXDR */
    *hi2c->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);
    hi2c->p_buf_rx++;
    if ((hi2c->xfer_size > 0U))
    {
      hi2c->xfer_size--;
      hi2c->xfer_count--;

      /* Set error_code corresponding to a Non-Acknowledge */
      hi2c->last_error_codes |= HAL_I2C_ERROR_AF;
    }
  }

  /* Disable all interrupts */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT | I2C_XFER_TX_IT);

  LL_I2C_ClearFlag_NACK(p_i2cx);

  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->global_state = HAL_I2C_STATE_IDLE;

  /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  hi2c->p_slave_listen_cplt_cb(hi2c);
#else
  HAL_I2C_SLAVE_ListenCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

/**
  * @brief  I2C interrupts error process.
  * @param  hi2c I2C handle
  * @param  error_code Error code to handle
  */
static void I2C_ITError(hal_i2c_handle_t *hi2c, uint32_t error_code)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  uint32_t tmppreviousstate;
#endif /* USE_HAL_I2C_DMA */
  hal_i2c_state_t tmp_state = hi2c->global_state;

  /* Reset handle parameters */
  hi2c->mode = HAL_I2C_MODE_NONE;
  hi2c->xfer_opt = (hal_i2c_xfer_opt_t) XFER_NO_OPTION;
  hi2c->xfer_count = 0U;

  /* Set new error code */
  hi2c->last_error_codes |= error_code;

  /* Disable interrupts */
  if (((uint32_t)tmp_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                              | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) != 0U)
  {
    /* Disable all interrupts, except interrupts related to LISTEN state */
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_TX_IT);

    /* keep HAL_I2C_STATE_LISTEN if set */
    hi2c->global_state = HAL_I2C_STATE_LISTEN;
    hi2c->xfer_isr = I2C_Slave_ISR_IT;
  }
  else
  {
    /* Disable all interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT | I2C_XFER_TX_IT);

    I2C_Flush_TXDR(p_i2cx);

    /* If state is an abort treatment ongoing, do not change state */
    /* This change is done later */
    if (hi2c->global_state != HAL_I2C_STATE_ABORT)
    {
      /* Set HAL_I2C_STATE_IDLE */
      hi2c->global_state = HAL_I2C_STATE_IDLE;
    }

    /* Check if a STOPF is detected */
    if (LL_I2C_IsActiveFlag_STOP(p_i2cx) != 0U)
    {
      if (LL_I2C_IsActiveFlag_NACK(p_i2cx) != 0U)
      {
        LL_I2C_ClearFlag_NACK(p_i2cx);
        hi2c->last_error_codes |= HAL_I2C_ERROR_AF;
      }

      LL_I2C_ClearFlag_STOP(p_i2cx);
    }

    hi2c->xfer_isr = NULL;
  }

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
  /* Abort DMA TX transfer if any */
  tmppreviousstate = hi2c->previous_state;
  if ((hi2c->hdma_tx != NULL) && ((tmppreviousstate == I2C_STATE_MASTER_BUSY_TX)
                                  || (tmppreviousstate == I2C_STATE_SLAVE_BUSY_TX)))
  {
    if (LL_I2C_IsEnabledDMAReq_TX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_TX(p_i2cx);
    }

    if (HAL_DMA_GetState(hi2c->hdma_tx) != HAL_DMA_STATE_IDLE)
    {
      /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
      hi2c->hdma_tx->p_xfer_abort_cb = I2C_DMAAbort;

      /* Abort DMA TX */
      if (HAL_DMA_Abort_IT(hi2c->hdma_tx) != HAL_OK)
      {
        /* Call directly p_xfer_abort_cb function in case of error */
        hi2c->hdma_tx->p_xfer_abort_cb(hi2c->hdma_tx);
      }
    }
    else
    {
      I2C_TreatErrorCallback(hi2c);
    }
  }
  /* Abort DMA RX transfer if any */
  else if ((hi2c->hdma_rx != NULL) && ((tmppreviousstate == I2C_STATE_MASTER_BUSY_RX)
                                       || (tmppreviousstate == I2C_STATE_SLAVE_BUSY_RX)))
  {
    if (LL_I2C_IsEnabledDMAReq_RX(p_i2cx) != 0UL)
    {
      LL_I2C_DisableDMAReq_RX(p_i2cx);
    }

    if (HAL_DMA_GetState(hi2c->hdma_rx) != HAL_DMA_STATE_IDLE)
    {
      /* Set the I2C DMA abort callback with HAL_I2C_ErrorCallback() at end of DMA abort procedure */
      hi2c->hdma_rx->p_xfer_abort_cb = I2C_DMAAbort;

      /* Abort DMA RX */
      if (HAL_DMA_Abort_IT(hi2c->hdma_rx) != HAL_OK)
      {
        /* Call directly hi2c->hdma_rx->p_xfer_abort_cb function in case of error */
        hi2c->hdma_rx->p_xfer_abort_cb(hi2c->hdma_rx);
      }
    }
    else
    {
      I2C_TreatErrorCallback(hi2c);
    }
  }
  else
#endif /* USE_HAL_I2C_DMA */
  {
    I2C_TreatErrorCallback(hi2c);
  }
}

/**
  * @brief  I2C Error callback treatment.
  * @param  hi2c I2C handle
  */
static void I2C_TreatErrorCallback(hal_i2c_handle_t *hi2c)
{
  if (hi2c->global_state == HAL_I2C_STATE_ABORT)
  {
    hi2c->previous_state = I2C_STATE_NONE;
    hi2c->global_state = HAL_I2C_STATE_IDLE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_abort_cplt_cb(hi2c);
#else
    HAL_I2C_AbortCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    hi2c->previous_state = I2C_STATE_NONE;

    /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_I2C_REGISTER_CALLBACKS) && (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->p_error_cb(hi2c);
#else
    HAL_I2C_ErrorCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  I2C Tx data register flush process.
  * @param  p_i2cx CMSIS handle
  */
static void I2C_Flush_TXDR(I2C_TypeDef *p_i2cx)
{
  /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
  if (LL_I2C_IsActiveFlag_TXIS(p_i2cx) != 0U)
  {
    LL_I2C_TransmitData8(p_i2cx, 0x00U);
  }

  if (LL_I2C_IsActiveFlag_TXE(p_i2cx) == 0U)
  {
    LL_I2C_ClearFlag_TXE(p_i2cx);
  }
}

#if defined (USE_HAL_I2C_DMA) && (USE_HAL_I2C_DMA == 1)
/**
  * @brief  DMA I2C master transmit process complete callback.
  * @param  hdma DMA handle
  */
static void I2C_DMAMasterTransmitCplt(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_DisableDMAReq_TX(p_i2cx);

  /* If last transfer, enable STOP interrupt */
  if (hi2c->xfer_count == 0U)
  {
    /* Enable STOP interrupt */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_CPLT_IT_DMA_MASK);
  }
  /* else prepare a new DMA transfer and enable TCReload interrupt */
  else
  {
    hi2c->p_buf_tx += hi2c->xfer_size;
    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;
    }

    /* Enable the DMA channel */
    if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_tx,
                                       (uint32_t)hi2c->p_buf_tx,
                                       LL_I2C_DMA_GetRegAddrTx(p_i2cx),
                                       hi2c->xfer_size, HAL_DMA_OPT_IT_NONE) != HAL_OK)
    {
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_DMA);
    }
    else
    {
      /* Enable TC interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_RELOAD_IT_MASK);
    }
  }
}

/**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  hdma DMA handle
  */
static void I2C_DMASlaveTransmitCplt(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  hal_i2c_xfer_opt_t xfer_opt = hi2c->xfer_opt;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  if ((xfer_opt == HAL_I2C_XFER_NEXT_FRAME) || (xfer_opt == HAL_I2C_XFER_FIRST_FRAME))
  {
    LL_I2C_DisableDMAReq_TX(p_i2cx);

    /* Last byte is transmitted */
    /* Call I2C slave sequential complete process */
    I2C_ITSlaveSeqCplt(hi2c);
  }
  else
  {
    /* No specific action, master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition must be managed through interrupt treatment */
  }
}

/**
  * @brief  DMA I2C master receive process complete callback.
  * @param  hdma DMA handle
  */
static void I2C_DMAMasterReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  LL_I2C_DisableDMAReq_RX(p_i2cx);

  /* If last transfer, enable STOP interrupt */
  if (hi2c->xfer_count == 0U)
  {
    /* Enable STOP interrupt */
    LL_I2C_EnableIT(p_i2cx, I2C_XFER_CPLT_IT_DMA_MASK);
  }
  /* else prepare a new DMA transfer and enable TCReload interrupt */
  else
  {
    hi2c->p_buf_rx += hi2c->xfer_size;
    if (hi2c->xfer_count > MAX_NBYTE_SIZE)
    {
      hi2c->xfer_size = MAX_NBYTE_SIZE;
    }
    else
    {
      hi2c->xfer_size = hi2c->xfer_count;
    }

    /* Enable the DMA channel */
    if (HAL_DMA_StartPeriphXfer_IT_Opt(hi2c->hdma_rx,
                                       LL_I2C_DMA_GetRegAddrRx(p_i2cx),
                                       (uint32_t)hi2c->p_buf_rx,
                                       hi2c->xfer_size, HAL_DMA_OPT_IT_NONE) != HAL_OK)
    {
      /* Call the corresponding callback to inform upper layer of end of transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_DMA);
    }
    else
    {
      /* Enable TC interrupts */
      LL_I2C_EnableIT(p_i2cx, I2C_XFER_RELOAD_IT_MASK);
    }
  }
}

/**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  hdma DMA handle
  */
static void I2C_DMASlaveReceiveCplt(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  hal_i2c_xfer_opt_t xfer_opt = hi2c->xfer_opt;

  if ((HAL_DMA_GetDirectXferRemainingDataByte(hi2c->hdma_rx) == 0U)
      && (xfer_opt != (hal_i2c_xfer_opt_t) XFER_NO_OPTION))
  {
    LL_I2C_DisableDMAReq_RX(p_i2cx);

    /* Call I2C slave sequential complete process */
    I2C_ITSlaveSeqCplt(hi2c);
  }
  else
  {
    /* No specific action, master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition must be managed through interrupt treatment */
  }
}

/**
  * @brief  DMA I2C communication error callback.
  * @param hdma DMA handle
  */
static void I2C_DMAError(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  /* Call the corresponding callback to inform upper layer of end of transfer */
  I2C_ITError(hi2c, HAL_I2C_ERROR_DMA);
}

/**
  * @brief  DMA I2C communication abort callback.
  *         (To be called at end of DMA abort procedure).
  * @param  hdma DMA handle.
  */
static void I2C_DMAAbort(hal_dma_handle_t *hdma)
{
  hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)(((hal_dma_handle_t *)hdma)->p_parent);

  I2C_TreatErrorCallback(hi2c);
}
#endif /* USE_HAL_I2C_DMA */

/**
  * @brief  This function handles I2C Communication timeout.
  *         It waits until a flag is no longer in the specified status.
  * @param  hi2c Pointer to a I2C_TypeDef
  * @param  flag Specifies the I2C flag to check
  * @param  status The actual flag status (SET or RESET)
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_WaitOnFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t flag, uint32_t status,
                                               uint32_t timeout_ms, uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t it_flags = LL_I2C_READ_REG(p_i2cx, ISR);
  while (LL_I2C_IsActiveFlag(p_i2cx, flag) == status)
  {
    /* Check if an error is detected */
    if (I2C_IsErrorOccurred(hi2c, it_flags, timeout_ms, tick_start) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        if ((LL_I2C_IsActiveFlag(p_i2cx, flag) == status))
        {
          return HAL_TIMEOUT;
        }
      }
    }
    it_flags = LL_I2C_READ_REG(p_i2cx, ISR);
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication timeout for specific usage of TXIS flag.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_WaitOnTXISFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start)
{
  hal_status_t hal_status;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t it_flags = LL_I2C_READ_REG(p_i2cx, ISR);

  while (LL_I2C_IsActiveFlag_TXIS(p_i2cx) == 0U)
  {
    /* Check if an error is detected */
    hal_status = I2C_IsErrorOccurred(hi2c, it_flags, timeout_ms, tick_start);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }

    /* Check for the timeout */
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        if (LL_I2C_IsActiveFlag_TXIS(p_i2cx) == 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    it_flags = LL_I2C_READ_REG(p_i2cx, ISR);
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication timeout for specific usage of STOP flag.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_WaitOnSTOPFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start)
{
  hal_status_t hal_status;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t it_flags = LL_I2C_READ_REG(p_i2cx, ISR);

  while (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U)
  {
    /* Check if an error is detected */
    hal_status = I2C_IsErrorOccurred(hi2c, it_flags, timeout_ms, tick_start);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }

    /* Check for the timeout */
    if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
    {
      if (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U)
      {
        return HAL_TIMEOUT;
      }
    }
    it_flags = LL_I2C_READ_REG(p_i2cx, ISR);
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication timeout for specific usage of RXNE flag.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK      Operation completed successfully
  * @retval HAL_ERROR   Operation completed with error
  * @retval HAL_TIMEOUT Operation exceeds user timeout
  */
static hal_status_t I2C_WaitOnRXNEFlagUntilTimeout(hal_i2c_handle_t *hi2c, uint32_t timeout_ms, uint32_t tick_start)
{
  hal_status_t hal_status;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  uint32_t it_flags = LL_I2C_READ_REG(p_i2cx, ISR);

  while (LL_I2C_IsActiveFlag_RXNE(p_i2cx) == 0U)
  {
    /* Check if an error is detected */
    hal_status = I2C_IsErrorOccurred(hi2c, it_flags, timeout_ms, tick_start);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }

    /* Check if a STOPF is detected */
    if (LL_I2C_IsActiveFlag_STOP(p_i2cx) != 0U)
    {
      /* Check if an RXNE is pending */
      /* Store last receive data if any */
      if ((LL_I2C_IsActiveFlag_RXNE(p_i2cx) != 0U) && (hi2c->xfer_size > 0U))
      {
        /* The Reading of data from RXDR is done in caller function */
        return HAL_OK;
      }
      else
      {
        if (LL_I2C_IsActiveFlag_NACK(p_i2cx) != 0U)
        {
          LL_I2C_ClearFlag_NACK(p_i2cx);
          hi2c->last_error_codes = HAL_I2C_ERROR_AF;
        }

        LL_I2C_ClearFlag_STOP(p_i2cx);
        I2C_RESET_CR2(p_i2cx);
        return HAL_ERROR;
      }
    }

    /* Check for the timeout */
    if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
    {
      if (LL_I2C_IsActiveFlag_RXNE(p_i2cx) == 0U)
      {
        return HAL_TIMEOUT;
      }
    }
    it_flags = LL_I2C_READ_REG(p_i2cx, ISR);
  } /* End of while */

  return HAL_OK;
}

/**
  * @brief  This function handles errors detection during an I2C Communication.
  * @param  hi2c Pointer to a hal_i2c_handle_t
  * @param  it_flags flag register before function call
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK     Operation completed successfully
  * @retval HAL_ERROR  Operation completed with error
  */
static hal_status_t I2C_IsErrorOccurred(hal_i2c_handle_t *hi2c, uint32_t it_flags, uint32_t timeout_ms,
                                        uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);
  hal_status_t status = HAL_OK;
  uint32_t error_codes = 0;
  uint32_t tick_start_local = tick_start;
  uint32_t tmp_register;
  uint32_t tmp_it_flags = it_flags;
  hal_i2c_mode_t tmp_mode;

  if (STM32_IS_BIT_SET(tmp_it_flags, LL_I2C_ISR_NACKF))
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    /* Wait until STOP flag is set or timeout occurred */
    /* AutoEnd must be initiated after AF */
    while ((LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U) && (status == HAL_OK))
    {
      /* Check for the timeout */
      if (timeout_ms != HAL_MAX_DELAY)
      {
        if (((HAL_GetTick() - tick_start_local) > timeout_ms) || (timeout_ms == 0U))
        {
          tmp_register = (uint32_t)(LL_I2C_READ_REG(p_i2cx, CR2) & I2C_CR2_STOP);
          tmp_mode = hi2c->mode;

          /* In case of I2C still busy, try to regenerate a STOP manually */
          if ((LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
              && (tmp_register != I2C_CR2_STOP)
              && (tmp_mode != HAL_I2C_MODE_SLAVE))
          {
            LL_I2C_GenerateStopCondition(p_i2cx);

            /* Update Tick with new reference */
            tick_start_local = HAL_GetTick();
          }

          while (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U)
          {
            /* Check for the timeout */
            if ((HAL_GetTick() - tick_start_local) > I2C_DEFAULT_TIMEOUT_MS)
            {
              status = HAL_ERROR;
              break;
            }
          }
        }
      }
    }

    /* In case STOP flag is detected, clear it */
    if (status == HAL_OK)
    {
      LL_I2C_ClearFlag_STOP(p_i2cx);
    }

    error_codes |= HAL_I2C_ERROR_AF;
    status = HAL_ERROR;
  }

  /* Refresh Content of status register */
  tmp_it_flags = LL_I2C_READ_REG(p_i2cx, ISR);

  /* Verify if additional errors occur */
  /* Check if a Bus error occurred */
  if (STM32_IS_BIT_SET(tmp_it_flags, LL_I2C_ISR_BERR))
  {
    error_codes |= HAL_I2C_ERROR_BERR;

    LL_I2C_ClearFlag_BERR(p_i2cx);
    status = HAL_ERROR;
  }

  /* Check if an Over-Run/Under-Run error occurred */
  if (STM32_IS_BIT_SET(tmp_it_flags, LL_I2C_ISR_OVR))
  {
    error_codes |= HAL_I2C_ERROR_OVR;

    LL_I2C_ClearFlag_OVR(p_i2cx);
    status = HAL_ERROR;
  }

  /* Check if an arbitration loss error occurred */
  if (STM32_IS_BIT_SET(tmp_it_flags, LL_I2C_ISR_ARLO))
  {
    error_codes |= HAL_I2C_ERROR_ARLO;

    LL_I2C_ClearFlag_ARLO(p_i2cx);
    status = HAL_ERROR;
  }

  if (status != HAL_OK)
  {
    I2C_Flush_TXDR(p_i2cx);
    I2C_RESET_CR2(p_i2cx);

    hi2c->last_error_codes |= error_codes;
  }

  return status;
}

/**
  * @brief  Handle I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  p_i2cx Pointer to a I2C_TypeDef
  * @param  device_addr Specifies the slave address to be programmed
  * @param  size_byte Specifies the number of bytes to be programmed. It must be a value between 0 and 255.
  * @param  mode New state of the I2C START condition generation :
  *           Enable Reload mode
  *           Automatic end mode
  *           Enable Software end mode
  * @param  request New state of the I2C START condition generation :
  *           Don't Generate stop and start condition
  *           Generate stop condition (size_byte must be set to 0)
  *           Generate Restart for read request
  *           Generate Restart for write request
  */
static void I2C_TransferConfig(I2C_TypeDef *p_i2cx, uint32_t device_addr, uint32_t size_byte, uint32_t mode,
                               i2c_start_stop_mode_t request)
{
  ASSERT_DBG_PARAM(IS_TRANSFER_MODE(mode));
  ASSERT_DBG_PARAM(IS_TRANSFER_REQUEST(request));

  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp = ((uint32_t)(((uint32_t)device_addr & I2C_CR2_SADD)
                             | (((uint32_t)size_byte << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES)
                             | (uint32_t)mode | (uint32_t)request) & (~0x80000000U));

  STM32_MODIFY_REG(p_i2cx->CR2,
                   ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND
                     | (I2C_CR2_RD_WRN & ((uint32_t) request >> (31U - I2C_CR2_RD_WRN_Pos)))
                     | I2C_CR2_START | I2C_CR2_STOP)), tmp);
}

/**
  * @brief  Manage the disabling of interrupts.
  * @param  hi2c Pointer to a hal_i2c_handle_t that contains the configuration information for the specified I2C
  * @param  it_request Value of @ref I2C_Interrupt_configuration.
  */
static void I2C_Disable_IRQ(hal_i2c_handle_t *hi2c, uint32_t it_request)
{
  uint32_t tmp_isr = 0U;
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hi2c);

  if ((it_request & I2C_XFER_TX_IT) == I2C_XFER_TX_IT)
  {
    /* Disable TC and TXI interrupts */
    tmp_isr |= LL_I2C_CR1_TCIE | LL_I2C_CR1_TXIE;

    if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                         | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
    {
      /* Disable NACK and STOP interrupts */
      tmp_isr |= LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_ERRIE;
    }
  }

  if ((it_request & I2C_XFER_RX_IT) == I2C_XFER_RX_IT)
  {
    /* Disable TC and RXI interrupts */
    tmp_isr |= LL_I2C_CR1_TCIE | LL_I2C_CR1_RXIE;

    if (((uint32_t)hi2c->global_state & ((uint32_t)HAL_I2C_STATE_LISTEN | (uint32_t)HAL_I2C_STATE_RX_LISTEN \
                                         | (uint32_t)HAL_I2C_STATE_TX_LISTEN)) == 0U)
    {
      /* Disable NACK and STOP interrupts */
      tmp_isr |= LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_ERRIE;
    }
  }

  if ((it_request & I2C_XFER_LISTEN_IT) == I2C_XFER_LISTEN_IT)
  {
    /* Disable ADDR, NACK and STOP interrupts */
    tmp_isr |= I2C_XFER_LISTEN_IT_MASK;
  }

  /* Disable interrupts only at the end to avoid a breaking situation */
  /* like at "t" time all disable interrupts request are not done */
  LL_I2C_DisableIT(p_i2cx, tmp_isr);
}

/**
  * @brief  Convert I2Cx OTHER_xxx xfer_opt to functional xfer_opt.
  * @param  hi2c I2C handle
  */
static void I2C_ConvertOtherXferOptions(hal_i2c_handle_t *hi2c)
{
  /* if user set xfer_opt to HAL_I2C_XFER_OTHER_FRAME */
  /* it request implicitly to generate a restart condition */
  /* set xfer_opt to HAL_I2C_XFER_FIRST_FRAME */
  if (hi2c->xfer_opt == HAL_I2C_XFER_OTHER_FRAME)
  {
    hi2c->xfer_opt = HAL_I2C_XFER_FIRST_FRAME;
  }
  /* else if user set xfer_opt to HAL_I2C_XFER_OTHER_AND_LAST_FRAME */
  /* it request implicitly to generate a restart condition */
  /* then generate a stop condition at the end of transfer */
  /* set xfer_opt to HAL_I2C_XFER_FIRST_AND_LAST_FRAME */
  else if (hi2c->xfer_opt == HAL_I2C_XFER_OTHER_AND_LAST_FRAME)
  {
    hi2c->xfer_opt = HAL_I2C_XFER_FIRST_AND_LAST_FRAME;
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_I2C_MODULE */
#endif /* I2C1 || I2C2 */

/**
  * @}
  */
