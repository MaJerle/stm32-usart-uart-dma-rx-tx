/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_smbus.c
  * @brief   SMBUS HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the System Management Bus (SMBus) of the I2Cx peripheral,
  *          based on I2C principles of operation :
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions.
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
  **********************************************************************************************************************
  */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(I2C1) || defined(I2C2)
#if defined(USE_HAL_SMBUS_MODULE) && (USE_HAL_SMBUS_MODULE == 1)

/** @addtogroup SMBUS SMBUS
  * @{
  */
/** @defgroup SMBUS_Introduction SMBUS Introduction
  * @{

  - The SMBUS hardware abstraction layer provides a set of APIs to interface with the STM32 SMBUS (System management
    bus) peripheral.

  - It simplifies the configuration, initialization and management of I2C communication, by supporting various modes
    such as polling and interrupt for data transfer.

  - This peripheral is compatible with the SMBUS specification version 3.0.

  - This abstraction layer ensures portability and ease of use across different STM32 series.

  */
/**
  * @}
  */

/** @defgroup SMBUS_How_To_Use SMBUS How To Use
  * @{

# How to use the SMBUS HAL module driver

1. Declare a hal_smbus_handle_t handle structure and initialize the SMBUSx driver
   with an I2C HW instance by calling the HAL_SMBUS_Init().
   The SMBUSx clock is enabled inside the HAL_SMBUS_Init() if USE_HAL_SMBUS_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO.

2. Configure the low level hardware (GPIO, CLOCK, NVIC...etc):
  - Enable the SMBUSx clock if USE_HAL_SMBUS_CLK_ENABLE_MODEL = HAL_CLK_ENABLE_NO
  - SMBUSx pins configuration :
    - Enable the clock for the SMBUSx GPIOs
      - Configure SMBUSx pins as alternate function open-drain
    - NVIC configuration if you need to use interrupt process
      - Configure the SMBUSx interrupt priority
      - Enable the NVIC SMBUSx IRQ Channel

3. Configure the Communication Clock Timing (same calculation as I2C), Own Address1, Device mode by calling
   HAL_SMBUS_SetConfig.

4. Configure and/or enable advanced features. For instance, HAL_SMBUS_EnableAnalogFilter(),
   HAL_SMBUS_SetDigitalFilter(), HAL_SMBUS_SetConfigOwnAddress2(), HAL_SMBUS_EnableOwnAddress2(), ...APIs.
   All these advanced configurations are optional (not mandatory).

5. For SMBUSx IO operations modes, only interrupt is available within this driver as the SMBUS protocol requires the
   application to perform exchange with a byte granularity within the slave device.

  - Interrupt mode IO sequential operation.
      These interfaces allow to manage a sequential transfer with a repeated start condition when a direction change
      during transfer. A specific option field manages the different steps of a sequential transfer through
      @ref hal_smbus_xfer_opt_t and are listed below

    - XXXXXXXX_WITH_PEC suffix: Those options are activated by enabling Packet Error Check with
      HAL_SMBUS_EnablePacketErrorCheck() and allows for the Hardware PEC comparison to happen

    - XXXXXXXX_NO_PEC suffix: Those options are activated by default or by disabling Packet Error Check using
      HAL_SMBUS_DisablePacketErrorCheck() and avoid the Hardware PEC comparison to happen

    - HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME: No sequential, means that a stop condition is automatically generated at the
      end of the single sequence.

    - HAL_SMBUS_XFER_FIRST_FRAME: Sequential, this option allows to manage a sequence with start condition, address and
      data to transfer without a final stop condition.

    - HAL_SMBUS_XFER_FIRST_AND_NEXT_FRAME: Sequential (Master only), this option allows to manage a sequence with start
      condition, address and data to transfer without a final stop condition. This allows a call to the same master
      sequential interface several times (HAL_SMBUS_MASTER_SEQ_Transmit_IT() followed by another call to
      HAL_SMBUS_MASTER_SEQ_Transmit_IT())

    - HAL_SMBUS_XFER_NEXT_FRAME: Sequential, this option allows to manage a sequence with a restart condition, address
      and with new data to transfer if the direction change or manage only the new data to transfer if no direction
      change and without a final stop condition in both cases.

    - HAL_SMBUS_XFER_LAST_FRAME: Sequential, same as HAL_SMBUS_XFER_NEXT_FRAME but with a final stop condition in
      both cases.

    - HAL_SMBUS_XFER_OTHER_FRAME: Sequential (Master only), this option allows to manage a restart condition after each
      call of the same master sequential interface. User can transfer several bytes one by one with a restart with slave
      address between each byte using
        - HAL_SMBUS_MASTER_SEQ_Transmit_IT()
        - HAL_SMBUS_MASTER_SEQ_Receive_IT()
      Then usage of this option HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME at the last frame to help automatic
      generation of STOP condition.

  - Different sequential SMBUS interfaces are listed below:
    - Sequential transmit in master mode an amount of data in non-blocking mode using HAL_SMBUS_MASTER_SEQ_Transmit_IT()
    - At transmission end of current frame transfer, HAL_SMBUS_MASTER_TxCpltCallback() is executed and users can add
      their own code by customization of function pointer HAL_SMBUS_MASTER_TxCpltCallback()

    - Sequential receive in master SMBUS mode an amount of data in non-blocking mode using
      HAL_SMBUS_MASTER_SEQ_Receive_IT()
    - At reception end of current frame transfer, HAL_SMBUS_MASTER_RxCpltCallback() is executed and users can add their
      own code by customization of function pointer HAL_SMBUS_MASTER_RxCpltCallback()

    - Abort a master IT SMBUS process communication with Interrupt using HAL_SMBUS_MASTER_Abort_IT()
    - End of abort process, HAL_SMBUS_AbortCpltCallback() is executed and users can add their own code by customization
      of function pointer HAL_SMBUS_AbortCpltCallback()

    - Enable/disable the Address listen mode in slave SMBUS mode with HAL_SMBUS_SLAVE_EnableListen_IT() and
      HAL_SMBUS_SLAVE_DisableListen_IT()

    - When address slave SMBUS match, HAL_SMBUS_SLAVE_AddrCallback() is executed and users can add their own code to
      check the address Match Code and the transmission direction request by master(Write/Read).

    - At Listen mode end HAL_SMBUS_SLAVE_ListenCpltCallback() is executed and users can add their own code by
      customization of function pointer HAL_SMBUS_SLAVE_ListenCpltCallback()

    - Sequential transmit in slave SMBUS mode an amount of data in non-blocking mode using
      HAL_SMBUS_SLAVE_SEQ_Transmit_IT()

    - At transmission end of current frame transfer, HAL_SMBUS_SLAVE_TxCpltCallback() is executed and users can add
      their own code by customization of function pointer HAL_SMBUS_SLAVE_TxCpltCallback()

    - Sequential receive in slave SMBUS mode an amount of data in non-blocking mode using
      HAL_SMBUS_SLAVE_SEQ_Receive_IT()

    - At reception end of current frame transfer, HAL_SMBUS_SLAVE_RxCpltCallback() is executed and users can add their
      own code by customization of function pointer HAL_SMBUS_SLAVE_RxCpltCallback()

    - In case of transfer Error, HAL_SMBUS_ErrorCallback() function is executed and users can add their own code by
      customization of function pointer HAL_SMBUS_ErrorCallback()

    - Discard a slave SMBUS process communication using HAL_SMBUS_SLAVE_Abort_IT() macro. This action informs the Master
      to generate a Stop condition to discard the communication.

6. Callbacks definition in Interrupt
  - When the compilation define USE_HAL_SMBUS_REGISTER_CALLBACKS is set to 1, the user can configure dynamically the
    driver callbacks, via its own method:

  Callback name               | Default value                        | Callback registration function
  ----------------------------| ------------------------------------ | ---------------------------
  MASTER_TxCpltCallback       | HAL_SMBUS_MASTER_TxCpltCallback()    | HAL_SMBUS_MASTER_RegisterTxCpltCallback()
  MASTER_RxCpltCallback       | HAL_SMBUS_MASTER_RxCpltCallback()    | HAL_SMBUS_MASTER_RegisterRxCpltCallback()
  SLAVE_TxCpltCallback        | HAL_SMBUS_SLAVE_TxCpltCallback()     | HAL_SMBUS_SLAVE_RegisterTxCpltCallback()
  SLAVE_RxCpltCallback        | HAL_SMBUS_SLAVE_RxCpltCallback()     | HAL_SMBUS_SLAVE_RegisterRxCpltCallback()
  AddrMatchCallback           | HAL_SMBUS_SLAVE_AddrCallback()       | HAL_SMBUS_SLAVE_RegisterAddrMatchCallback()
  ListenCpltCallback          | HAL_SMBUS_SLAVE_ListenCpltCallback() | HAL_SMBUS_SLAVE_RegisterListenCpltCallback()
  AbortCpltCallback           | HAL_SMBUS_AbortCpltCallback()        | HAL_SMBUS_RegisterAbortCpltCallback()
  ErrorCallback               | HAL_SMBUS_ErrorCallback()            | HAL_SMBUS_RegisterErrorCallback()

  If one needs to unregister a callback, register the default callback via the registration function.

  By default, after the HAL_SMBUS_Init() and when the state is HAL_SMBUS_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Callbacks can be registered in handle global_state HAL_SMBUS_STATE_INIT and HAL_SMBUS_STATE_IDLE.

  When the compilation define USE_HAL_SMBUS_REGISTER_CALLBACKS is set to 0 or not defined, the callback registration
  feature is not available and weak callbacks are used, represented by the default value in the table above.

7. Acquire/Release the SMBUS bus
  - When the compilation flag USE_HAL_MUTEX is set to 1, it allows the user to acquire/reserve the whole I2C bus for
    executing process .
    The HAL Acquire/Release are based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal) :
      - HAL_SMBUS_AcquireBus() for acquire the bus or wait for it.
      - HAL_SMBUS_ReleaseBus() for releasing the bus.

  - When the compilation flag USE_HAL_MUTEX is set to 0 or not defined, HAL_SMBUS_AcquireBus() and
    HAL_SMBUS_ReleaseBus() are not available.
  */
/**
  * @}
  */

/** @defgroup SMBUS_Configuration_Table SMBUS Configuration Table
  * @{
8. Configuration inside the SMBUS driver

Software configuration defined in stm32c5xx_hal_conf.h:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_HAL_SMBUS_MODULE             |         1         | Enable HAL SMBUS driver module
USE_HAL_SMBUS_REGISTER_CALLBACKS |         0         | Allow the user to define their own callback
USE_HAL_CHECK_PARAM              |         0         | Enable runtime parameter check
USE_HAL_SMBUS_CLK_ENABLE_MODEL   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
USE_HAL_CHECK_PROCESS_STATE      |         0         | Enable atomicity of process state check
USE_HAL_MUTEX                    |         0         | Enable semaphore creation for OS
USE_HAL_SMBUS_USER_DATA          |         0         | Add a user data inside HAL SMBUS handle
USE_HAL_SMBUS_GET_LAST_ERRORS    |         0         | Enable retrieval of last processes error codes

Software configuration defined in preprocessor environment:
Preprocessor flags               |   Default value   | Comment
-------------------------------- | ----------------- | ------------------------------------------------
USE_ASSERT_DBG_PARAM             |    Not defined    | Enable check param for HAL and LL
USE_ASSERT_DBG_STATE             |    Not defined    | Enable check state for HAL

  */

/**
  * @}
  */

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/** @defgroup SMBUS_Private_Types SMBUS Private Types
  * @{
  */

/**
  * @brief SMBUS Start or Stop Mode.
  */
typedef enum
{
  SMBUS_NO_STARTSTOP            = (0x00000000U),                               /*!< No start no stop */
  SMBUS_GENERATE_STOP           = (0x80000000U | I2C_CR2_STOP),                /*!< Stop */
  SMBUS_GENERATE_START_READ     = (0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN), /*!< Start read */
  SMBUS_GENERATE_START_WRITE    = (0x80000000U | I2C_CR2_START),               /*!< Start write */
} smbus_start_stop_mode_t;
/**
  * @}
  */
/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup SMBUS_Private_Constants SMBUS Private Constants
  * @{
  */
#define TIMING_CLEAR_MASK           (0xF0FFFFFFUL)              /*!< SMBUS TIMING clear register Mask */
#define SMBUS_DEFAULT_TIMEOUT_MS    (25U)                       /*!< 25 ms */
#define MAX_NBYTE_SIZE              (255U)                      /*!< SMBUS Max NBYTES */

/* Private define for @ref PreviousState usage */
#define SMBUS_STATE_NONE            (0U)                        /*!< Default Value */
#define SMBUS_STATE_MASTER_BUSY_TX  (1UL << 0U)                 /*!< Master Busy TX */
#define SMBUS_STATE_MASTER_BUSY_RX  (1UL << 1U)                 /*!< Master Busy RX */
#define SMBUS_STATE_SLAVE_BUSY_TX   (1UL << 2U)                 /*!< Slave Busy TX */
#define SMBUS_STATE_SLAVE_BUSY_RX   (1UL << 3U)                 /*!< Slave Busy RX */

/** @defgroup SMBUS_ReloadEndMode_definition SMBUS ReloadEndMode definition
  * @{
  */

#define  SMBUS_SOFTEND_MODE                     (0x00000000U)   /*!< Software end mode and Reload mode */
#define  SMBUS_RELOAD_MODE                      I2C_CR2_RELOAD  /*!< Reload mode */
#define  SMBUS_AUTOEND_MODE                     I2C_CR2_AUTOEND /*!< Hardware auto end and reload mode */
#define  SMBUS_SENDPEC_MODE                     I2C_CR2_PECBYTE /*!< Packet Error Calculation mode */
/**
  * @}
  */

/** @defgroup SMBUS_Interrupt_configuration_mask SMBUS interrupt configuration mask
  * @{
  */

/**
  * @brief Interrupt Mask for error, Tx cplt, Stop, NACK and Tx.
  */
#define SMBUS_TX_IT_MASK                        (LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | \
                                                 LL_I2C_CR1_NACKIE | LL_I2C_CR1_TXIE)
/**
  * @brief Interrupt Mask for error, Tx cplt, NACK and Rx.
  */
#define SMBUS_RX_IT_MASK                        (LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_NACKIE | \
                                                 LL_I2C_CR1_RXIE)
/**
  * @brief Interrupt Mask for error.
  */
#define SMBUS_ALERT_IT_MASK                     (LL_I2C_CR1_ERRIE)

/**
  * @brief Interrupt Mask for error, Tx cplt, NACK and Rx.
  */
#define SMBUS_ADDR_IT_MASK                      (LL_I2C_CR1_ADDRIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE)
/**
  * @}
  */
/**
  * @}
  */

/* Private macros -------------------------------------------------------------*/
/** @defgroup SMBUS_Private_Macros SMBUS Private Macros
  * @{
  */
/**
  * @brief  Retrieve I2C instance from handle.
  * @param  handle from which instance has to be retrieved, must be from @ref hal_smbus_handle_t.
  * @retval Instance pointer
  */
#define I2C_GET_INSTANCE(handle)        ((I2C_TypeDef *)((uint32_t)(handle)->instance))

/**
  * @brief  Ensure that transfer request is valid.
  * @param  request Request to test, must be from @ref smbus_start_stop_mode_t.
  * @retval 1U (request is valid) or 0U (request is invalid)
  */
#define IS_TRANSFER_REQUEST(request)    (((request) == SMBUS_GENERATE_STOP)               \
                                         || ((request) == SMBUS_GENERATE_START_READ)      \
                                         || ((request) == SMBUS_GENERATE_START_WRITE)     \
                                         || ((request) == SMBUS_NO_STARTSTOP))

/**
  * @brief  Ensure that SMBUS mode is valid.
  * @param  mode mode to test, must be from @ref hal_smbus_mode_t.
  * @retval 1U (mode is valid) or 0U (mode is invalid)
  */
#define IS_SMBUS_MODE(mode)          (((mode) == HAL_SMBUS_PERIPHERAL_MODE_HOST)         \
                                      || ((mode) == HAL_SMBUS_PERIPHERAL_MODE_SLAVE)     \
                                      || ((mode) == HAL_SMBUS_PERIPHERAL_MODE_SLAVE_ARP))

/**
  * @brief  Ensure that SMBUS timeout is valid.
  * @param  timeout timeout to test, must be from @ref hal_smbus_timeout_t.
  * @retval 1U (timeout is valid) or 0U (timeout is invalid)
  */
#define IS_SMBUS_TIMEOUT(timeout)    (((timeout) == HAL_SMBUS_TIMEOUT_NONE)    \
                                      || ((timeout) == HAL_SMBUS_TIMEOUT_A)    \
                                      || ((timeout) == HAL_SMBUS_TIMEOUT_B)    \
                                      || ((timeout) == HAL_SMBUS_TIMEOUT_ALL))

/**
  * @brief  Ensure that SMBUS timeout value is valid.
  * @param  value timeout value to test.
  * @retval 1U (value is valid) or 0U (value is invalid)
  */
#define IS_SMBUS_TIMEOUT_VALUE(value)    ((value) <= 0x0000FFFU)

/**
  * @brief  Ensure that SMBUS timeout A mode is valid.
  * @param  mode timeout mode to test, must be from @ref hal_smbus_timeout_a_mode_t.
  * @retval 1U (mode is valid) or 0U (mode is invalid)
  */
#define IS_SMBUS_TIMEOUT_MODE(mode)   (((mode) == HAL_SMBUS_TIMEOUTA_MODE_SCL_LOW)     \
                                       || ((mode) == HAL_SMBUS_TIMEOUTA_MODE_SDA_SCL_HIGH))

/**
  * @brief  Ensure that SMBUS own address mask is valid.
  * @param  mask mask to test, must be from @ref hal_smbus_own_addr2_mask_t.
  * @retval 1U (mask is valid) or 0U (mask is invalid)
  */
#define IS_SMBUS_OWN_ADDRESS2_MASK(mask)  (((mask) == HAL_SMBUS_OWN_ADDR2_NOMASK)      \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK01)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK02)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK03)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK04)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK05)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK06)   \
                                           || ((mask) == HAL_SMBUS_OWN_ADDR2_MASK07))

/**
  * @brief  Ensure that SMBUS transfer request is valid.
  * @param  req transfer request to test, must be from @ref hal_smbus_xfer_opt_t.
  * @retval 1U (request is valid) or 0U (request is invalid)
  */
#define IS_SMBUS_TRANSFER_OPTIONS_REQUEST(req)(((req) == HAL_SMBUS_XFER_FIRST_FRAME)              \
                                               || ((req) == HAL_SMBUS_XFER_NEXT_FRAME)            \
                                               ||IS_SMBUS_TRANSFER_NOPEC_OPTIONS_REQUEST(req)     \
                                               ||IS_SMBUS_TRANSFER_PEC_OPTIONS_REQUEST(req))

/**
  * @brief  Ensure that SMBUS transfer request has PEC enabled.
  * @param  req transfer request to test, must be from @ref hal_smbus_xfer_opt_t.
  * @retval 1U (request with PEC) or 0U (request without PEC)
  */
#define IS_SMBUS_TRANSFER_PEC_OPTIONS_REQUEST(req) (((req) == HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_WITH_PEC)    \
                                                    || ((req) == HAL_SMBUS_XFER_OTHER_FRAME_WITH_PEC)          \
                                                    || ((req) == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_WITH_PEC))

/**
  * @brief  Ensure that SMBUS transfer request has PEC disabled.
  * @param  req transfer request to test, must be from @ref hal_smbus_xfer_opt_t.
  * @retval 1U (request without PEC) or 0U (request with PEC)
  */
#define IS_SMBUS_TRANSFER_NOPEC_OPTIONS_REQUEST(req) (((req) == HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_NO_PEC)    \
                                                      || ((req) == HAL_SMBUS_XFER_OTHER_FRAME_NO_PEC)          \
                                                      || ((req) == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_NO_PEC))

/**
  * @brief  Ensure that SMBUS transfer request is of OTHER type.
  * @param  req transfer request to test, must be from @ref hal_smbus_xfer_opt_t.
  * @retval 1U (request with OTHER) or 0U (request without OTHER)
  */
#define IS_SMBUS_TRANSFER_OTHER_OPTIONS_REQUEST(req) (((req) == HAL_SMBUS_XFER_OTHER_FRAME_NO_PEC)              \
                                                      || ((req) == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_NO_PEC)  \
                                                      || ((req) == HAL_SMBUS_XFER_OTHER_FRAME_WITH_PEC)         \
                                                      || ((req) == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_WITH_PEC))

/**
  * @brief  Get Address match macro.
  * @param  handle SMBUS handle.
  * @retval address match
  */
#define SMBUS_GET_ADDR_MATCH(handle) \
  ((uint32_t)((((I2C_TypeDef *)((uint32_t)(handle)->instance))->ISR & I2C_ISR_ADDCODE)  >> I2C_ISR_ADDCODE_Pos))

/**
  * @brief  Get dir macro.
  * @param  handle SMBUS handle.
  * @retval dir value
  */
#define SMBUS_GET_DIR(handle) \
  ((uint8_t)((((I2C_TypeDef *)((uint32_t)(handle)->instance))->ISR & I2C_ISR_DIR) >> 16U))

/**
  * @brief  Get Stop mode macro.
  * @param  handle SMBUS handle.
  * @retval Stop mode
  */
#define SMBUS_GET_STOP_MODE(handle) (((I2C_TypeDef *)((uint32_t)(handle)->instance))->CR2 & I2C_CR2_AUTOEND)

/**
  * @brief  Get own address1 macro.
  * @param  handle SMBUS handle.
  * @retval own address1
  */
#define SMBUS_GET_OWN_ADDRESS1(handle) \
  ((uint32_t)(((I2C_TypeDef *)((uint32_t)(handle)->instance))->OAR1 & I2C_OAR1_OA1))

/**
  * @brief  Get own address2 macro.
  * @param  handle SMBUS handle.
  * @retval own address2
  */
#define SMBUS_GET_OWN_ADDRESS2(handle) \
  ((uint32_t)(((I2C_TypeDef *)((uint32_t)(handle)->instance))->OAR2 & I2C_OAR2_OA2))

/**
  * @brief  Ensure that SMBUS address is valid.
  * @param  address SMBUS address.
  * @retval 1U (address is valid) or 0U (address is invalid)
  */
#define IS_SMBUS_ADDRESS(address)    ((address) <= 0x00000FFU)

/**
  * @brief  Ensure that SMBUS digital filter is valid.
  * @param  filter SMBUS filter value.
  * @retval 1U (filter is valid) or 0U (filter is invalid)
  */
#define IS_SMBUS_DIGITAL_FILTER(filter)   ((filter) <= 0x0000000FU)


/**
  * @brief  Check if the given flag is raised in the given ISR register.
  * @param  isr ISR register.
  * @param  flag ISR flag to check.
  * @retval 1U (flag is raised) or 0U (flag is not raised)
  */
#define SMBUS_CHECK_FLAG(isr, flag)         ((((isr) & ((flag) & (0x0001FFFFU)))  \
                                              == ((flag) & 0x0001FFFFU)) ? 1U : 0U)

/**
  * @brief  Check if the given IT is enabled in the given CR1 register.
  * @param  cr1 CR1 register.
  * @param  it IT to check.
  * @retval 1U (IT is enabled) or 0U (it is disabled)
  */
#define SMBUS_CHECK_IT_SOURCE(cr1, it)      ((((cr1) & (it)) == (it)) ? 1U : 0U)

/**
  * @brief  Reset the CR1 register of the given instance.
  * @param  instance SMBUS instance.
  */
#define I2C_RESET_CR2(instance) ((instance)->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD   | I2C_CR2_HEAD10R  \
                                                                           | I2C_CR2_NBYTES | I2C_CR2_RELOAD \
                                                                           | I2C_CR2_RD_WRN)))
/**
  * @}
  */
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------------------------------------------*/
/** @addtogroup SMBUS_Private_Functions SMBUS Private Functions
  * @{
  */
/* Private functions to handle flags during polling transfer */
static hal_status_t SMBUS_WaitOnFlagUntilTimeout(hal_smbus_handle_t *hsmbus, uint32_t flag, uint32_t status,
                                                 uint32_t timeout_ms, uint32_t tick_start);

/* Private functions for SMBUS transfer IRQ handler */
static hal_status_t SMBUS_Master_ISR(hal_smbus_handle_t *hsmbus, uint32_t it_flags, uint32_t it_sources);
static hal_status_t SMBUS_Slave_ISR(hal_smbus_handle_t *hsmbus, uint32_t it_flags, uint32_t it_sources);
static void SMBUS_ITErrorHandler(hal_smbus_handle_t *hsmbus);

/* Private functions to centralize the enable/disable of Interrupts */
static void SMBUS_Enable_IRQ(hal_smbus_handle_t *hsmbus, uint32_t it_request);
static void SMBUS_Disable_IRQ(hal_smbus_handle_t *hsmbus, uint32_t it_request);

/* Private function to flush TXDR register */
static void SMBUS_Flush_TXDR(hal_smbus_handle_t *hsmbus);

/* Private function to handle start, restart or stop a transfer */
static void SMBUS_TransferConfig(I2C_TypeDef *p_i2cx,  uint32_t device_addr, uint32_t size_byte,
                                 uint32_t mode, smbus_start_stop_mode_t request);

/* Private function to convert specific options */
static void SMBUS_ConvertOtherXferOptions(hal_smbus_handle_t *hsmbus);
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/

/** @addtogroup SMBUS_Exported_Functions
  * @{
  */

/** @addtogroup SMBUS_Exported_Functions_Group1
  * @{
A set of functions to initialize and deinitialize the SMBUSx functionality in the I2Cx peripheral:
 - HAL_SMBUS_Init(): initialize the selected device with the SMBUS instance.
 - HAL_SMBUS_DeInit(): restore the default configuration of the selected SMBUSx functionality in the I2Cx peripheral.
  */

/**
  * @brief  Initialize the SMBUS according to the associated handle.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  instance HAL SMBUS instance.
  * @retval HAL_OK HAL SMBUS instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM HAL SMBUS instance is NULL.
  * @retval HAL_ERROR HAL SMBUS semaphore creation failed (USE_HAL_MUTEX is set to 1).
  */
hal_status_t HAL_SMBUS_Init(hal_smbus_handle_t *hsmbus, hal_smbus_t instance)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_ALL_INSTANCE((I2C_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hsmbus == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Instance */
  hsmbus->instance = instance;

#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
  /* I2C Callbacks to the weak function */
  hsmbus->p_master_tx_cplt_cb    = HAL_SMBUS_MASTER_TxCpltCallback;
  hsmbus->p_master_rx_cplt_cb    = HAL_SMBUS_MASTER_RxCpltCallback;
  hsmbus->p_slave_tx_cplt_cb     = HAL_SMBUS_SLAVE_TxCpltCallback;
  hsmbus->p_slave_rx_cplt_cb     = HAL_SMBUS_SLAVE_RxCpltCallback;
  hsmbus->p_slave_listen_cplt_cb = HAL_SMBUS_SLAVE_ListenCpltCallback;
  hsmbus->p_slave_addr_cb        = HAL_SMBUS_SLAVE_AddrCallback;
  hsmbus->p_abort_cplt_cb        = HAL_SMBUS_AbortCpltCallback;
  hsmbus->p_error_cb             = HAL_SMBUS_ErrorCallback;
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */

  /* Other internal fields */
  hsmbus->p_buf_tx = (uint8_t *) NULL;
  hsmbus->p_buf_rx = (uint8_t *) NULL;
  hsmbus->xfer_size = 0U;
  hsmbus->xfer_count = 0U;
  hsmbus->xfer_opt = (hal_smbus_xfer_opt_t) 0U;
  hsmbus->xfer_isr = NULL;

  hsmbus->last_error_codes = HAL_SMBUS_ERROR_NONE;

#if defined (USE_HAL_SMBUS_USER_DATA) && (USE_HAL_SMBUS_USER_DATA == 1)
  hsmbus->p_user_data = (void *) NULL;
#endif /* USE_HAL_SMBUS_USER_DATA */

#if defined(USE_HAL_SMBUS_CLK_ENABLE_MODEL) && (USE_HAL_SMBUS_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  /* Enable I2Cx Clock */
  switch (instance)
  {
    case HAL_SMBUS1:
      HAL_RCC_I2C1_EnableClock();
      break;
#if defined(I2C2)
    case HAL_SMBUS2:
      HAL_RCC_I2C2_EnableClock();
      break;
#endif /* I2C2 */
    default:
      break;
  }
#endif /* USE_HAL_SMBUS_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Create the SMBUS semaphore */
  if (HAL_OS_SemaphoreCreate(&hsmbus->semaphore) != HAL_OS_OK)
  {
    return HAL_ERROR;
  }
#endif /* USE_HAL_MUTEX */

  hsmbus->global_state = HAL_SMBUS_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the HAL SMBUS driver for the given handle and disable the SMBUSx functionality in
  *         the I2Cx peripheral.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure.
  */
void HAL_SMBUS_DeInit(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;
  hal_smbus_state_t temp_state;
  uint32_t count = SMBUS_DEFAULT_TIMEOUT_MS;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_ALL_INSTANCE((I2C_TypeDef *)((uint32_t)hsmbus->instance)));

  /* Get the I2Cx CMSIS handle */
  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  temp_state = hsmbus->global_state;

  if ((temp_state != HAL_SMBUS_STATE_IDLE) && (temp_state != HAL_SMBUS_STATE_INIT))
  {
    if (LL_I2C_GetMode(p_i2cx) == (uint32_t)HAL_SMBUS_PERIPHERAL_MODE_HOST)
    {
      LL_I2C_GenerateStopCondition(p_i2cx);
    }
    else
    {
      LL_I2C_AcknowledgeNextData(p_i2cx, LL_I2C_NACK);
    }

    do
    {
      count--;
      if (count == 0U)
      {
        break;
      }
    } while (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U);
  }

  /* Disable the I2C instance */
  LL_I2C_Disable(p_i2cx);

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
  /* Delete the SMBUS semaphore */
  (void)HAL_OS_SemaphoreDelete(&hsmbus->semaphore);
#endif /* USE_HAL_MUTEX */

  /* Reset the state */
  hsmbus->global_state = HAL_SMBUS_STATE_RESET;
}
/**
  * @}
  */

/** @addtogroup SMBUS_Exported_Functions_Group2
  * @{
A set of functions to configure the I2Cx peripheral in SMBUS:

- Global configuration:
  - HAL_SMBUS_SetConfig()
  - HAL_SMBUS_GetConfig()

- Unitary configuration:
  - HAL_SMBUS_SetTiming()
  - HAL_SMBUS_GetTiming()

- Timeout configuration:
  - HAL_SMBUS_SetConfigTimeout()
  - HAL_SMBUS_GetConfigTimeout()
  - HAL_SMBUS_EnableTimeout()
  - HAL_SMBUS_DisableTimeout()
  - HAL_SMBUS_IsEnabledTimeoutA()
  - HAL_SMBUS_IsEnabledTimeoutB()

- Filter mode:
  - HAL_SMBUS_EnableAnalogFilter()
  - HAL_SMBUS_DisableAnalogFilter()
  - HAL_SMBUS_IsEnabledAnalogFilter()
  - HAL_SMBUS_SetDigitalFilter()
  - HAL_SMBUS_GetDigitalFilter()

- Acknowledge General Call:
  - HAL_SMBUS_SLAVE_EnableAckGeneralCall()
  - HAL_SMBUS_SLAVE_DisableAckGeneralCall()
  - HAL_SMBUS_SLAVE_IsEnabledAckGeneralCall()

- Second Own Address configuration :
  - HAL_SMBUS_SetConfigOwnAddress2()
  - HAL_SMBUS_GetConfigOwnAddress2()
  - HAL_SMBUS_EnableOwnAddress2()
  - HAL_SMBUS_DisableOwnAddress2()
  - HAL_SMBUS_IsEnabledOwnAddress2()

- Packet Error Check :
  - HAL_SMBUS_EnablePacketErrorCheck()
  - HAL_SMBUS_DisablePacketErrorCheck()
  - HAL_SMBUS_IsEnabledPacketErrorCheck()

- Alert interrupt:
  - HAL_SMBUS_MASTER_EnableAlertIT()
  - HAL_SMBUS_MASTER_DisableAlertIT()
  - HAL_SMBUS_MASTER_IsEnabledAlertIT()

- Wakeup from Stop mode(s) :
  - HAL_SMBUS_SLAVE_EnableWakeUp()
  - HAL_SMBUS_SLAVE_DisableWakeUp()
  - HAL_SMBUS_SLAVE_IsEnabledWakeUp()

- Fast mode plus driving capability:
  - HAL_SMBUS_EnableFastModePlus()
  - HAL_SMBUS_DisableFastModePlus()
  - HAL_SMBUS_IsEnabledFastModePlus()

  */

/**
  * @brief  Configure the SMBUS according to the user parameters.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  p_config Pointer to the configuration structure.
  * @retval HAL_OK Operation completed successfully.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  */
hal_status_t HAL_SMBUS_SetConfig(hal_smbus_handle_t *hsmbus, const hal_smbus_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_MODE(p_config->device_mode));
  ASSERT_DBG_PARAM(IS_SMBUS_ADDRESS(p_config->own_address1));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i2cx = I2C_GET_INSTANCE(hsmbus);
  LL_I2C_Disable(p_i2cx);

  LL_I2C_SetTiming(p_i2cx, p_config->timing);
  LL_I2C_SetMode(p_i2cx, (uint32_t)p_config->device_mode);
  LL_I2C_DisableOwnAddress1AndMode(p_i2cx);
  LL_I2C_ConfigOwnAddress1(p_i2cx, p_config->own_address1, LL_I2C_OWNADDRESS1_7BIT);

  /* Enable the I2Cx AUTOEND by default, and enable NACK (must be disabled only during slave process). */
  LL_I2C_WRITE_REG(p_i2cx, CR2, (LL_I2C_READ_REG(p_i2cx, CR2) | I2C_CR2_AUTOEND | I2C_CR2_NACK));

  LL_I2C_Enable(p_i2cx);

  hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Retrieve the SMBUS configuration.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  p_config Pointer to the configuration structure.
  */
void HAL_SMBUS_GetConfig(const hal_smbus_handle_t *hsmbus, hal_smbus_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  p_config->timing = LL_I2C_GetTiming(p_i2cx);

  p_config->device_mode = (hal_smbus_mode_t)LL_I2C_GetMode(p_i2cx);

  p_config->own_address1 = LL_I2C_GetOwnAddress1(p_i2cx);
}

/**
  * @brief  Set the SMBUS Timing.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  value SMBUS timing
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SetTiming(hal_smbus_handle_t *hsmbus, uint32_t value)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetTiming(p_i2cx, value);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the SMBUS Timing.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval uint32_t SMBUS timing value
  */
uint32_t HAL_SMBUS_GetTiming(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return LL_I2C_GetTiming(p_i2cx);
}

/**
  * @brief  Enable SMBUS Analog noise filter.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_EnableAnalogFilter(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableAnalogFilter(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable SMBUS Analog noise filter.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_DisableAnalogFilter(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableAnalogFilter(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS analog noise filter status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_ANALOG_FILTER_ENABLED  Analog Filter is enabled
  * @retval HAL_SMBUS_ANALOG_FILTER_DISABLED Analog Filter is disabled
  */
hal_smbus_analog_filter_status_t HAL_SMBUS_IsEnabledAnalogFilter(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_analog_filter_status_t) LL_I2C_IsEnabledAnalogFilter(p_i2cx);
}

/**
  * @brief  Set the SMBUS Digital noise filter.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  noise_filtering_in_bus_clk_period Filtering period between Min_Data=0x00 and Max_Data=0x0F.
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SetDigitalFilter(hal_smbus_handle_t *hsmbus, uint32_t noise_filtering_in_bus_clk_period)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_DIGITAL_FILTER(noise_filtering_in_bus_clk_period));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetDigitalFilter(p_i2cx, noise_filtering_in_bus_clk_period);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the SMBUS Digital noise filter.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval uint32_t Coefficient of digital noise filter between Min_Data=0x00 and Max_Data=0x0F.
  */
uint32_t HAL_SMBUS_GetDigitalFilter(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return LL_I2C_GetDigitalFilter(p_i2cx);
}

/**
  * @brief  Enable SMBUS Slave wakeup from Stop mode(s).
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SLAVE_EnableWakeUp(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableWakeUpFromStop(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable Slave SMBUS wakeup from Stop mode(s).
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SLAVE_DisableWakeUp(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableWakeUpFromStop(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS slave wake up status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_SLAVE_WAKE_UP_ENABLED  Slave Wake Up is enabled
  * @retval HAL_SMBUS_SLAVE_WAKE_UP_DISABLED Slave Wake Up is disabled
  */
hal_smbus_slave_wake_up_status_t HAL_SMBUS_SLAVE_IsEnabledWakeUp(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_slave_wake_up_status_t)LL_I2C_IsEnabledWakeUpFromStop(p_i2cx);
}

/**
  * @brief  Set hardware timeout configuration.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  p_config Pointer to hal_smbus_timeout_config_t containing both TimeoutA and B configuration.
  * @retval HAL_OK Operation completed successfully.
  */
hal_status_t HAL_SMBUS_SetConfigTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_TIMEOUT_VALUE(p_config->timeout_a));
  ASSERT_DBG_PARAM(IS_SMBUS_TIMEOUT_VALUE(p_config->timeout_b));
  ASSERT_DBG_PARAM(IS_SMBUS_TIMEOUT_MODE(p_config->timeout_a_mode));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_ConfigSMBusTimeout(p_i2cx, p_config->timeout_a, (uint32_t)p_config->timeout_a_mode, p_config->timeout_b);

  return HAL_OK;
}

/**
  * @brief  Get hardware timeout configuration.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  p_config Pointer to hal_smbus_timeout_config_t containing both TimeoutA and B configuration.
  */
void HAL_SMBUS_GetConfigTimeout(const hal_smbus_handle_t *hsmbus, hal_smbus_timeout_config_t *p_config)
{
  I2C_TypeDef *p_i2cx;
  uint32_t timeoutr_reg;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  timeoutr_reg = LL_I2C_READ_REG(p_i2cx, TIMEOUTR);

  p_config->timeout_a = (timeoutr_reg & I2C_TIMEOUTR_TIMEOUTA) >> I2C_TIMEOUTR_TIMEOUTA_Pos;
  p_config->timeout_a_mode = (hal_smbus_timeout_a_mode_t)(timeoutr_reg & I2C_TIMEOUTR_TIDLE);
  p_config->timeout_b = (timeoutr_reg & I2C_TIMEOUTR_TIMEOUTB) >> I2C_TIMEOUTR_TIMEOUTB_Pos;
}

/**
  * @brief  Enable SMBUS timeout feature.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  timeout Timeout to Enable
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_EnableTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_t timeout)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_TIMEOUT(timeout));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_EnableSMBusTimeout(p_i2cx, (uint32_t)timeout);

  return HAL_OK;
}

/**
  * @brief  Disable SMBUS timeout feature.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  timeout Timeout to disable
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_DisableTimeout(hal_smbus_handle_t *hsmbus, const hal_smbus_timeout_t timeout)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_TIMEOUT(timeout));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_DisableSMBusTimeout(p_i2cx, (uint32_t)timeout);

  return HAL_OK;
}

/**
  * @brief  Get SMBUS Timeout A status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_TIMEOUT_NONE Timeout A is disabled
  * @retval HAL_SMBUS_SELECT_TIMEOUT_A Timeout A is enabled
  */
hal_smbus_timeout_t HAL_SMBUS_IsEnabledTimeoutA(const hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_timeout_t)((uint32_t)STM32_READ_BIT(p_i2cx->TIMEOUTR, I2C_TIMEOUTR_TIMOUTEN));
}

/**
  * @brief  Get SMBUS Timeout B status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_TIMEOUT_NONE Timeout B is disabled
  * @retval HAL_SMBUS_SELECT_TIMEOUT_A Timeout B is enabled
  */
hal_smbus_timeout_t HAL_SMBUS_IsEnabledTimeoutB(const hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_timeout_t)((uint32_t)STM32_READ_BIT(p_i2cx->TIMEOUTR, I2C_TIMEOUTR_TEXTEN));
}

/**
  * @brief  Enable SMBUS slave acknowledge general call address.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SLAVE_EnableAckGeneralCall(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableGeneralCall(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable SMBUS slave acknowledge general call address.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SLAVE_DisableAckGeneralCall(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableGeneralCall(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS slave acknowledge general call status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_SLAVE_ACK_GENERAL_CALL_ENABLED    Slave Acknowledge General Call is enabled
  * @retval HAL_SMBUS_SLAVE_ACK_GENERAL_CALL_DISABLED   Slave Acknowledge General Call is disabled
  */
hal_smbus_slave_ack_general_call_status_t HAL_SMBUS_SLAVE_IsEnabledAckGeneralCall(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_slave_ack_general_call_status_t)LL_I2C_IsEnabledGeneralCall(p_i2cx);
}

/**
  * @brief  Enable Packet Error Check.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_EnablePacketErrorCheck(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_EnableSMBusPEC(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable Packet Error Check.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_DisablePacketErrorCheck(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_DisableSMBusPEC(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS packet error check(PEC) status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_PEC_ENABLED     Packet Error Check enabled
  * @retval HAL_SMBUS_PEC_DISABLED    Packet Error Check disabled
  */
hal_smbus_pec_status_t HAL_SMBUS_IsEnabledPacketErrorCheck(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_pec_status_t) LL_I2C_IsEnabledSMBusPEC(p_i2cx);
}

/**
  * @brief  Enable Alert Interruption.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_MASTER_EnableAlertIT(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_EnableSMBusAlert(p_i2cx);
  LL_I2C_ClearSMBusFlag_ALERT(p_i2cx);

  /* Enable Alert Interrupt */
  SMBUS_Enable_IRQ(hsmbus, SMBUS_ALERT_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Disable Alert Interruption.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_MASTER_DisableAlertIT(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  /* Disable Alert Interrupt */
  SMBUS_Disable_IRQ(hsmbus, SMBUS_ALERT_IT_MASK);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* Disable SMBus alert */
  LL_I2C_DisableSMBusAlert(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS Alert interruption status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_ALERT_ENABLED      Alert interruption enabled
  * @retval HAL_SMBUS_ALERT_DISABLED     Alert interruption disabled
  */
hal_smbus_alert_status_t HAL_SMBUS_MASTER_IsEnabledAlertIT(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_alert_status_t) LL_I2C_IsEnabledSMBusAlert(p_i2cx);
}

/**
  * @brief  Set the SMBUS own address2 configuration.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  addr  The second device own address. It is a 7-bit address but the value must be shifted left by 1 bit.
  *               In other words, an 8-bit value is required and the bit 0 is not considered.
  * @param  mask  Acknowledge mask address second device own address.
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SetConfigOwnAddress2(hal_smbus_handle_t *hsmbus, uint32_t addr, hal_smbus_own_addr2_mask_t mask)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_ADDRESS(addr));
  ASSERT_DBG_PARAM(IS_SMBUS_OWN_ADDRESS2_MASK(mask));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_SetOwnAddress2(p_i2cx, addr, (uint32_t)mask);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Get the SMBUS own address2 configuration.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  p_addr The second device own address. It is a 7-bit address but the value is shifted left by 1 bit.
  *                In other words, an 8-bit value is returned and the bit 0 is not considered.
  * @param  p_mask Acknowledge mask address second device own address.
  */
void HAL_SMBUS_GetConfigOwnAddress2(const hal_smbus_handle_t *hsmbus, uint32_t *p_addr,
                                    hal_smbus_own_addr2_mask_t *p_mask)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_addr != NULL));
  ASSERT_DBG_PARAM((p_mask != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  *p_addr = LL_I2C_GetOwnAddress2(p_i2cx);
  *p_mask = (hal_smbus_own_addr2_mask_t)LL_I2C_GetOwnAddress2Mask(p_i2cx);
}

/**
  * @brief  Enable SMBUS Own Address2.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_EnableOwnAddress2(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableOwnAddress2(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable SMBUS Own Address2.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_DisableOwnAddress2(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableOwnAddress2(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS own address 2 status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_OWN_ADDR2_ENABLED   Dual addressing is enabled
  * @retval HAL_SMBUS_OWN_ADDR2_DISABLED  Dual addressing is disabled
  */
hal_smbus_own_addr2_status_t HAL_SMBUS_IsEnabledOwnAddress2(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_own_addr2_status_t)LL_I2C_IsEnabledOwnAddress2(p_i2cx);
}

/**
  * @brief  Set the functional SMBUS mode(Host, Slave or Slave ARP).
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  mode   Mode to set from hal_smbus_mode_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_SetMode(hal_smbus_handle_t *hsmbus, const hal_smbus_mode_t mode)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_MODE(mode));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_SetMode(p_i2cx, (uint32_t)mode);

  return HAL_OK;
}

/**
  * @brief  Return the functional SMBUS mode. Host, Slave or Slave ARP.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL mode
  */
hal_smbus_mode_t HAL_SMBUS_GetMode(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_mode_t)LL_I2C_GetMode(p_i2cx);
}

/**
  * @brief  Enable the SMBUS fast mode plus driving capability.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_EnableFastModePlus(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  /* Check the parameter */
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_EnableFastModePlus(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Disable the SMBUS fast mode plus driving capability.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @retval HAL_OK Operation completed successfully
  */
hal_status_t HAL_SMBUS_DisableFastModePlus(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  /* Check the parameter */
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  LL_I2C_Disable(p_i2cx);
  LL_I2C_DisableFastModePlus(p_i2cx);
  LL_I2C_Enable(p_i2cx);

  return HAL_OK;
}

/**
  * @brief  Check SMBUS fast mode plus feature status.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_SMBUS_FAST_MODE_PLUS_ENABLED    Fast mode plus enabled
  * @retval HAL_SMBUS_FAST_MODE_PLUS_DISABLED   Fast mode plus disabled
  */
hal_smbus_fast_mode_plus_status_t HAL_SMBUS_IsEnabledFastModePlus(const hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  return (hal_smbus_fast_mode_plus_status_t)LL_I2C_IsEnabledFastModePlus(p_i2cx);
}

#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register the SMBUS Master Tx Transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the Master Tx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_MASTER_RegisterTxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

  hsmbus->p_master_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Master Rx Transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the Master Rx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_MASTER_RegisterRxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_master_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Slave Tx Transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the Slave Tx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_RegisterTxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_slave_tx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Slave Rx Transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the Slave Rx Transfer completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_RegisterRxCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_slave_rx_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Slave Listen completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the SMBUS slave listen completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_RegisterListenCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_slave_listen_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Abort completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the SMBUS Abort completed callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_RegisterAbortCpltCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_abort_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Slave Address Match callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the SMBUS Slave Address Match callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_RegisterAddrMatchCallback(hal_smbus_handle_t *hsmbus, hal_smbus_slave_addr_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_slave_addr_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register the SMBUS Error callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_callback Pointer to the SMBUS Error callback function
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_RegisterErrorCallback(hal_smbus_handle_t *hsmbus, hal_smbus_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hsmbus->p_error_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup SMBUS_Exported_Functions_Group3
  * @{
A set of functions to manage the SMBUS data transfers.

- There are two modes of transfer:
  - Blocking mode: The communication is performed in polling mode.
    The status of all data processing is returned by the same function after finishing transfer.
  - Non-blocking mode: The communication is performed using interrupts because DMA is not supported due to SMBUS need
    to perform exchange with a byte granularity within the slave device.
    These functions return the status of the transfer startup.
    The end of the data processing will be indicated through the dedicated SMBUS IRQ when using interrupt mode.

- Blocking mode functions are:
  - HAL_SMBUS_MASTER_PollForSlaveReady()

- Non-Blocking mode functions with Interrupt are :
  - HAL_SMBUS_MASTER_SEQ_Transmit_IT()
  - HAL_SMBUS_MASTER_SEQ_Receive_IT()
  - HAL_SMBUS_SLAVE_SEQ_Transmit_IT()
  - HAL_SMBUS_SLAVE_SEQ_Receive_IT()
  - HAL_SMBUS_SLAVE_EnableListen_IT()
  - HAL_SMBUS_SLAVE_DisableListen_IT()
  - HAL_SMBUS_MASTER_Abort_IT()
  - HAL_SMBUS_SLAVE_Abort_IT()

- A set of Transfer weak complete callbacks are provided in non-blocking mode:
  - HAL_SMBUS_MASTER_TxCpltCallback()
  - HAL_SMBUS_MASTER_RxCpltCallback()
  - HAL_SMBUS_SLAVE_TxCpltCallback()
  - HAL_SMBUS_SLAVE_RxCpltCallback()
  - HAL_SMBUS_SLAVE_AddrCallback()
  - HAL_SMBUS_SLAVE_ListenCpltCallback()
  - HAL_SMBUS_ErrorCallback()
  - HAL_SMBUS_AbortCpltCallback()
  */

/**
  * @brief  Check if slave device is ready for communication.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  timeout_ms Timeout duration in millisecond
  * @retval HAL_OK            Target is ready for communication
  * @retval HAL_ERROR         Internal failure while waiting for hardware flags
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_TIMEOUT       User timeout elapsed: device not ready in time
  */
hal_status_t HAL_SMBUS_MASTER_PollForSlaveReady(hal_smbus_handle_t *hsmbus, uint32_t device_addr, uint32_t timeout_ms)
{
  I2C_TypeDef *p_i2cx;
  hal_status_t hal_status = HAL_OK;
  uint32_t tick_start;
  uint32_t tmp1;
  uint32_t tmp2;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_PARAM(IS_SMBUS_ADDRESS(device_addr));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_IDLE, HAL_SMBUS_STATE_TX);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  while (hal_status == HAL_OK)
  {
    tick_start = HAL_GetTick();

    while (LL_I2C_IsActiveFlag_BUSY(p_i2cx) != 0U)
    {
      if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
      {
        hsmbus->global_state = HAL_SMBUS_STATE_IDLE;
        return HAL_TIMEOUT;
      }
    }

    /* Generate Start */
    LL_I2C_WRITE_REG(p_i2cx, CR2, (uint32_t)((((uint32_t)(device_addr) & (I2C_CR2_SADD))
                                              | (I2C_CR2_START) | (I2C_CR2_AUTOEND)) & (~I2C_CR2_RD_WRN)));

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set and a NACK flag is set*/
    tmp1 = LL_I2C_IsActiveFlag_STOP(p_i2cx);
    tmp2 = LL_I2C_IsActiveFlag_NACK(p_i2cx);

    while (((tmp1 == 0U) && (tmp2 == 0U)) && (hal_status == HAL_OK))
    {
      if (timeout_ms != HAL_MAX_DELAY)
      {
        if (((HAL_GetTick() - tick_start) > timeout_ms) || (timeout_ms == 0U))
        {
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;
          hal_status = HAL_TIMEOUT;
        }
      }

      tmp1 = LL_I2C_IsActiveFlag_STOP(p_i2cx);
      tmp2 = LL_I2C_IsActiveFlag_NACK(p_i2cx);
    }

    if (hal_status == HAL_OK)
    {
      if (LL_I2C_IsActiveFlag_NACK(p_i2cx) == 0U)
      {
        /* Wait until STOPF flag is reset */
        if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, LL_I2C_ISR_STOPF, 0U, timeout_ms, tick_start) == HAL_OK)
        {
          /* An acknowledge appear during STOP Flag waiting process, this mean that device respond to its address */
          LL_I2C_ClearFlag_STOP(p_i2cx);
          break;
        }
        else
        {
          /* A non acknowledge appear during STOP Flag waiting process, a new trial must be performed */
          /* Clear STOP Flag */
          LL_I2C_ClearFlag_STOP(p_i2cx);

          hal_status = HAL_ERROR;
        }
      }
      else
      {
        /* A non acknowledge is detected, this mean that device not respond to its address,
          a new trial must be performed */

        LL_I2C_ClearFlag_NACK(p_i2cx);

        /* Wait until STOPF flag is reset */
        if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, LL_I2C_ISR_STOPF, 0U, SMBUS_DEFAULT_TIMEOUT_MS, tick_start) == HAL_OK)
        {
          /* Clear STOP flag, auto generated with autoend */
          LL_I2C_ClearFlag_STOP(p_i2cx);
        }

        hal_status = HAL_ERROR;
      }
    }
  }

  hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

  return hal_status;
}

/**
  * @brief  Sequential transmit in master SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to send in bytes
  * @param  xfer_opt Options of Transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  */
hal_status_t HAL_SMBUS_MASTER_SEQ_Transmit_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr, const void *p_data,
                                              uint32_t size_byte, hal_smbus_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;

  hal_smbus_xfer_opt_t tmp;
  smbus_start_stop_mode_t request;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  /* Allowed parameters, if size_byte is equals to 0 and p_data is equals to NULL */
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0U));
  ASSERT_DBG_PARAM(IS_SMBUS_ADDRESS(device_addr));
  ASSERT_DBG_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(xfer_opt));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0U)) /* p_data can be NULL if size_byte = 0 */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_IDLE, HAL_SMBUS_STATE_TX);

  /* Prepare transfer parameters */
  hsmbus->p_buf_tx = (const uint8_t *)p_data;
  hsmbus->xfer_count = size_byte;
  hsmbus->xfer_opt = xfer_opt;
  hsmbus->xfer_isr = SMBUS_Master_ISR;
  hsmbus->last_error_codes = HAL_SMBUS_ERROR_NONE;

  /* In case of Quick command, remove autoend mode */
  /* Manage the stop generation by software */
  if (hsmbus->p_buf_tx == NULL)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_FRAME;
  }

  if (size_byte > MAX_NBYTE_SIZE)
  {
    hsmbus->xfer_size = MAX_NBYTE_SIZE;
  }
  else
  {
    hsmbus->xfer_size = size_byte;
  }

  /* Send Slave Address */
  /* Set NBYTES to write and reload if size > MAX_NBYTE_SIZE and generate RESTART */
  if ((hsmbus->xfer_size < hsmbus->xfer_count) && (hsmbus->xfer_size == MAX_NBYTE_SIZE))
  {
    SMBUS_TransferConfig(p_i2cx, device_addr, hsmbus->xfer_size,
                         SMBUS_RELOAD_MODE | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE),
                         SMBUS_GENERATE_START_WRITE);
  }
  else
  {
    /* If transfer direction not changed, do not generate Restart Condition */
    /* Mean Previous state is same as current state */

    /* Store current volatile xfer_opt, misra rule */
    tmp = hsmbus->xfer_opt;

    if ((hsmbus->previous_state == (uint32_t)HAL_SMBUS_STATE_TX) && (IS_SMBUS_TRANSFER_OTHER_OPTIONS_REQUEST(tmp) == 0))
    {
      request = SMBUS_NO_STARTSTOP;
    }
    /* Else transfer direction change, so generate Restart with new transfer direction */
    else
    {
      /* Convert OTHER_xxx xfer_opt if any */
      SMBUS_ConvertOtherXferOptions(hsmbus);

      /* Handle Transfer */
      request = SMBUS_GENERATE_START_WRITE;
    }

    SMBUS_TransferConfig(p_i2cx, device_addr, hsmbus->xfer_size, (uint32_t)hsmbus->xfer_opt, request);

    /* If PEC mode is enabled, size to transmit managed by SW part must be Size-1 byte, corresponding to PEC byte */
    /* PEC byte is automatically sent by HW block, no need to manage it in Transmit process */
    if (LL_I2C_IsEnabledSMBusPECCompare(p_i2cx) != 0U)
    {
      if (hsmbus->xfer_size > 0U)
      {
        hsmbus->xfer_size--;
        hsmbus->xfer_count--;
      }
      else
      {
        hsmbus->global_state = HAL_SMBUS_STATE_IDLE;
        return HAL_ERROR;
      }
    }
  }

  SMBUS_Enable_IRQ(hsmbus, SMBUS_TX_IT_MASK);
  return HAL_OK;
}

/**
  * @brief  Sequential receive in master SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to receive in bytes
  * @param  xfer_opt Options of Transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_MASTER_SEQ_Receive_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr, void *p_data,
                                             uint32_t size_byte, hal_smbus_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;
  smbus_start_stop_mode_t request;
  hal_smbus_xfer_opt_t tmp;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  /* Allowed parameters, if size_byte is equals to 0 and p_data is equals to NULL */
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0U));
  ASSERT_DBG_PARAM(IS_SMBUS_ADDRESS(device_addr));
  ASSERT_DBG_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(xfer_opt));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0U)) /* p_data can be NULL if size_byte = 0 */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_IDLE, HAL_SMBUS_STATE_RX);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* Prepare transfer parameters */
  hsmbus->p_buf_rx = (uint8_t *)p_data;
  hsmbus->xfer_count = size_byte;
  hsmbus->xfer_opt = xfer_opt;
  hsmbus->xfer_isr = SMBUS_Master_ISR;
  hsmbus->last_error_codes = HAL_SMBUS_ERROR_NONE;

  /* In case of Quick command, remove autoend mode */
  /* Manage the stop generation by software */
  if (hsmbus->p_buf_rx == NULL)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_FRAME;
  }

  if (size_byte > MAX_NBYTE_SIZE)
  {
    hsmbus->xfer_size = MAX_NBYTE_SIZE;
  }
  else
  {
    hsmbus->xfer_size = size_byte;
  }

  /* Send Slave Address */
  /* Set NBYTES to write and reload if size > MAX_NBYTE_SIZE and generate RESTART */
  if ((hsmbus->xfer_size < hsmbus->xfer_count) && (hsmbus->xfer_size == MAX_NBYTE_SIZE))
  {
    request = SMBUS_GENERATE_START_READ;
    SMBUS_TransferConfig(p_i2cx, device_addr, hsmbus->xfer_size,
                         (SMBUS_RELOAD_MODE | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE)), request);
  }
  else
  {
    /* If transfer direction not changed, do not generate Restart Condition */
    /* Mean Previous state is same as current state */

    /* Store current volatile XferOptions, Misra rule */
    tmp = hsmbus->xfer_opt;

    if ((hsmbus->previous_state == (uint32_t)HAL_SMBUS_STATE_RX)
        && (IS_SMBUS_TRANSFER_OTHER_OPTIONS_REQUEST(tmp) == 0U))
    {
      request = SMBUS_NO_STARTSTOP;
    }
    /* Else transfer direction change, so generate Restart with new transfer direction */
    else
    {
      /* Convert OTHER_xxx XferOptions if any */
      SMBUS_ConvertOtherXferOptions(hsmbus);

      /* Handle Transfer */
      request = SMBUS_GENERATE_START_READ;
    }
    SMBUS_TransferConfig(p_i2cx, device_addr, hsmbus->xfer_size,
                         (uint32_t)hsmbus->xfer_opt,
                         request);
  }

  SMBUS_Enable_IRQ(hsmbus, SMBUS_RX_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Sequential transmit in slave/device SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to send in bytes
  * @param  xfer_opt Options of Transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_SEQ_Transmit_IT(hal_smbus_handle_t *hsmbus, const void *p_data, uint32_t size_byte,
                                             hal_smbus_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  /* Allowed parameters, if size_byte is equals to 0 and p_data is equals to NULL */
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0U));
  ASSERT_DBG_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(xfer_opt));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) && (size_byte != 0U)) /* p_data can be NULL if size_byte = 0 */
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_LISTEN, HAL_SMBUS_STATE_TX_LISTEN);

  /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
  SMBUS_Disable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK | SMBUS_TX_IT_MASK);

  /* Set SBC bit in CR1 to manage Acknowledge at each bit */
  LL_I2C_EnableSlaveByteControl(p_i2cx);

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hsmbus->p_buf_tx = (const uint8_t *)p_data;
  hsmbus->xfer_count = size_byte;
  hsmbus->xfer_opt = xfer_opt;
  hsmbus->xfer_isr = SMBUS_Slave_ISR;
  hsmbus->last_error_codes = HAL_SMBUS_ERROR_NONE;

  SMBUS_ConvertOtherXferOptions(hsmbus);

  if (size_byte > MAX_NBYTE_SIZE)
  {
    hsmbus->xfer_size = MAX_NBYTE_SIZE;
  }
  else
  {
    hsmbus->xfer_size = size_byte;
  }

  /* Set NBYTES to write and reload if size > MAX_NBYTE_SIZE and generate RESTART */
  if ((hsmbus->xfer_size < hsmbus->xfer_count) && (hsmbus->xfer_size == MAX_NBYTE_SIZE))
  {
    SMBUS_TransferConfig(p_i2cx, 0, hsmbus->xfer_size,
                         SMBUS_RELOAD_MODE | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE),
                         SMBUS_NO_STARTSTOP);
  }
  else
  {
    /* Set NBYTE to transmit */
    SMBUS_TransferConfig(p_i2cx, 0, hsmbus->xfer_size, (uint32_t)hsmbus->xfer_opt,
                         SMBUS_NO_STARTSTOP);

    /* If PEC mode is enabled, size to transmit must be size_byte-1 byte, corresponding to PEC byte */
    /* PEC byte is automatically sent by HW block, no need to manage it in Transmit process */
    if (LL_I2C_IsEnabledSMBusPEC(p_i2cx) != 0U)
    {
      if (hsmbus->xfer_size > 0U)
      {
        hsmbus->xfer_size--;
        hsmbus->xfer_count--;
      }
      else
      {
        hsmbus->global_state = HAL_SMBUS_STATE_IDLE;
        return HAL_ERROR;
      }
    }
  }

  /* Clear ADDR flag after prepare the transfer parameters */
  /* This action will generate an acknowledge to the HOST */
  LL_I2C_ClearFlag_ADDR(p_i2cx);

  /* Re-enable ADDR interrupt */
  SMBUS_Enable_IRQ(hsmbus, SMBUS_TX_IT_MASK | SMBUS_ADDR_IT_MASK);

  return HAL_OK;

}

/**
  * @brief  Sequential receive in slave/device SMBUS mode an amount of data in non-blocking mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  p_data Pointer to data buffer
  * @param  size_byte Amount of data to receive in bytes
  * @param  xfer_opt Options of Transfer
  * @note   This interface allows to manage repeated start condition when a direction change during transfer
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_SMBUS_SLAVE_SEQ_Receive_IT(hal_smbus_handle_t *hsmbus, void *p_data, uint32_t size_byte,
                                            hal_smbus_xfer_opt_t xfer_opt)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));
  /* Allowed parameters, if size_byte is equals to 0 and p_data is equals to NULL */
  ASSERT_DBG_PARAM((p_data != NULL) || (size_byte == 0U));
  ASSERT_DBG_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(xfer_opt));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 1)
  if (((uint32_t)hsmbus->global_state & ((uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                                         | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN)) == 0U)
  {
    return HAL_BUSY;
  }
#endif /* USE_HAL_CHECK_PROCESS_STATE */

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
  SMBUS_Disable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK | SMBUS_TX_IT_MASK);

  hsmbus->global_state = (HAL_SMBUS_STATE_RX_LISTEN);

  /* Set SBC bit in CR1 to manage Acknowledge at each bit */
  LL_I2C_EnableSlaveByteControl(p_i2cx);

  LL_I2C_AcknowledgeEnable(p_i2cx);

  /* Prepare transfer parameters */
  hsmbus->p_buf_rx = (uint8_t *)p_data;
  hsmbus->xfer_count = size_byte;
  hsmbus->xfer_size = size_byte;
  hsmbus->xfer_opt = xfer_opt;
  hsmbus->xfer_isr = SMBUS_Slave_ISR;
  hsmbus->last_error_codes = HAL_SMBUS_ERROR_NONE;

  /* Convert OTHER_xxx XferOptions if any */
  SMBUS_ConvertOtherXferOptions(hsmbus);

  /* If XferSize equal "1", or XferSize equal "2" with PEC requested (mean 1 data byte + 1 PEC byte */
  /* no need to set RELOAD bit mode, a ACK will be automatically generated in that case */
  /* else need to set RELOAD bit mode to generate an automatic ACK at each byte Received */
  /* This RELOAD bit will be reset for last BYTE to be receive in SMBUS_Slave_ISR */
  if (((LL_I2C_IsEnabledSMBusPEC(p_i2cx) != 0U) && (hsmbus->xfer_size == 2U)) || (hsmbus->xfer_size == 1U))
  {
    SMBUS_TransferConfig(p_i2cx, 0, (uint8_t)hsmbus->xfer_size, (uint32_t)hsmbus->xfer_opt,
                         SMBUS_NO_STARTSTOP);
  }
  else
  {
    SMBUS_TransferConfig(p_i2cx, 0, 1, (uint32_t)hsmbus->xfer_opt | SMBUS_RELOAD_MODE, SMBUS_NO_STARTSTOP);
  }

  /* Clear ADDR flag after prepare the transfer parameters */
  /* This action will generate an acknowledge to the HOST */
  LL_I2C_ClearFlag_ADDR(p_i2cx);

  /* Enable ADDR interrupt */
  SMBUS_Enable_IRQ(hsmbus, SMBUS_RX_IT_MASK | SMBUS_ADDR_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK    Operation started successfully
  * @retval HAL_BUSY  Concurrent process ongoing
  */
hal_status_t HAL_SMBUS_SLAVE_EnableListen_IT(hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_IDLE, HAL_SMBUS_STATE_LISTEN);

  hsmbus->xfer_isr = SMBUS_Slave_ISR;

  /* Enable the Address Match interrupt */
  SMBUS_Enable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK);

  return HAL_OK;
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_BUSY          Concurrent process ongoing
  */
hal_status_t HAL_SMBUS_SLAVE_DisableListen_IT(hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, HAL_SMBUS_STATE_LISTEN);

  HAL_CHECK_UPDATE_STATE(hsmbus, global_state, HAL_SMBUS_STATE_LISTEN, HAL_SMBUS_STATE_IDLE);

  /* Disable the Address Match interrupt */
  SMBUS_Disable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK);

  hsmbus->previous_state = SMBUS_STATE_NONE;
  hsmbus->xfer_isr = NULL;

  return HAL_OK;
}

/**
  * @brief  Abort a master SMBUS process communication with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  device_addr Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Mode is not Master
  * @retval HAL_BUSY          No process ongoing
  */
hal_status_t HAL_SMBUS_MASTER_Abort_IT(hal_smbus_handle_t *hsmbus, uint32_t device_addr)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_TX | (uint32_t)HAL_SMBUS_STATE_RX
                   | (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN | (uint32_t)HAL_SMBUS_STATE_IDLE);

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if (LL_I2C_GetMode(p_i2cx) == (uint32_t)HAL_SMBUS_PERIPHERAL_MODE_HOST)
  {
    /* Disable Interrupts and Store Previous state */
    if (hsmbus->global_state == HAL_SMBUS_STATE_TX)
    {
      SMBUS_Enable_IRQ(hsmbus, SMBUS_TX_IT_MASK);
      hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_TX;
    }
    else if (hsmbus->global_state == HAL_SMBUS_STATE_RX)
    {
      SMBUS_Enable_IRQ(hsmbus, SMBUS_RX_IT_MASK);
      hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_RX;
    }
    else
    {
      /* Do nothing */
    }

    hsmbus->global_state = HAL_SMBUS_STATE_ABORT;

    /* Set NBYTES to 1 to generate a dummy read on SMBUS functionality in the I2Cx peripheral */
    /* Set AUTOEND mode, this will generate a NACK then STOP condition to abort the current transfer */
    SMBUS_TransferConfig(p_i2cx, device_addr, 1, LL_I2C_MODE_AUTOEND, SMBUS_GENERATE_STOP);
    LL_I2C_EnableIT(p_i2cx, LL_I2C_CR1_STOPIE);

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
  * @brief  Abort a slave SMBUS process communication with Interrupt.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval HAL_OK            Operation started successfully
  * @retval HAL_ERROR         Mode is not Slave
  */
hal_status_t HAL_SMBUS_SLAVE_Abort_IT(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_TX | (uint32_t)HAL_SMBUS_STATE_RX
                   | (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN | (uint32_t)HAL_SMBUS_STATE_IDLE);

  p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if ((hal_smbus_mode_t)LL_I2C_GetMode(p_i2cx) == HAL_SMBUS_PERIPHERAL_MODE_SLAVE)
  {
    hsmbus->global_state = HAL_SMBUS_STATE_ABORT;

    LL_I2C_AcknowledgeNextData(p_i2cx, LL_I2C_NACK);
    return HAL_OK;
  }
  else
  {
    /* Wrong usage of abort function */
    /* This function must be used only in case of abort monitored by slave device */
    return HAL_ERROR;
  }
}
/**
  * @}
  */

/** @addtogroup SMBUS_Exported_Functions_Group4
  * @{
  This subsection provides the function handling the interruption of the SMBUS.
  - SMBUS Event IRQ Handler : HAL_SMBUS_EV_IRQHandler()
  - SMBUS Error IRQ Handler : HAL_SMBUS_ERR_IRQHandler()

  Depending on the process function one's use, different callback might be triggered:

| Process API \n \ \n Callbacks      | HAL_SMBUS_MASTER_SEQ_Transmit_IT | HAL_SMBUS_MASTER_SEQ_Receive_IT |
|------------------------------------|:-------------------------------:|:------------------------------:|
| HAL_SMBUS_MASTER_TxCpltCallback    |                x                |                                |
| HAL_SMBUS_MASTER_RxCpltCallback    |                                 |                  x             |
| HAL_SMBUS_ErrorCallback            |                x                |                  x             |
| HAL_SMBUS_AbortCpltCallback*       |                x                |                  x             |
@note * HAL_SMBUS_AbortCpltCallback is called by the ISR when the abort is requested by the slave (using NACK) or
        the master (by generating STOP)

| Process API \n \ \n Callbacks      | HAL_SMBUS_SLAVE_SEQ_Transmit_IT  |  HAL_SMBUS_SLAVE_SEQ_Receive_IT |
|------------------------------------|:-------------------------------:|:------------------------------:|
| HAL_SMBUS_SLAVE_TxCpltCallback     |                x                |                                |
| HAL_SMBUS_SLAVE_RxCpltCallback     |                                 |                  x             |
| HAL_SMBUS_SLAVE_ListenCpltCallback |                x                |                  x             |
| HAL_SMBUS_ErrorCallback            |                x                |                  x             |
@note HAL_SMBUS_SLAVE_EnableListen_IT must be called before HAL_SMBUS_SLAVE_SEQ_Transmit_IT and
      HAL_SMBUS_SLAVE_SEQ_Receive_IT

| Process API \n \ \n Callbacks      | HAL_SMBUS_SLAVE_EnableListen_IT |
|------------------------------------|:-------------------------------:|
| HAL_SMBUS_SLAVE_AddrCallback       |                x                |

| Process API \n \ \n Callbacks      |    HAL_SMBUS_MASTER_Abort_IT    |    HAL_SMBUS_SLAVE_Abort_IT    |
|------------------------------------|:-------------------------------:|:------------------------------:|
| HAL_SMBUS_AbortCpltCallback        |                x                |                  x             |
  */

/**
  * @brief  Handle SMBUS event interrupt request.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  */
void HAL_SMBUS_EV_IRQHandler(hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* Get current IT Flags and IT sources value */
  uint32_t it_flags   = LL_I2C_READ_REG(p_i2cx, ISR);
  uint32_t it_sources = LL_I2C_READ_REG(p_i2cx, CR1);

  /* SMBUS events treatment */
  if (hsmbus->xfer_isr != NULL)
  {
    hsmbus->xfer_isr(hsmbus, it_flags, it_sources);
  }
}
/**
  * @brief  Handle SMBUS error interrupt request.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  */
void HAL_SMBUS_ERR_IRQHandler(hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  SMBUS_ITErrorHandler(hsmbus);
}

/**
  * @}
  */

/** @addtogroup SMBUS_Exported_Functions_Group5
  * @{
A set of weak functions (or default callbacks functions if USE_HAL_SMBUS_REGISTER_CALLBACKS is set to 1U) which are used
to asynchronously informed the application in non blocking modes (Interrupt) :
 - HAL_SMBUS_MASTER_TxCpltCallback()     : Master Tx Transfer completed callback
 - HAL_SMBUS_MASTER_RxCpltCallback()     : Master Rx Transfer completed callback
 - HAL_SMBUS_SLAVE_TxCpltCallback()      : Slave Tx Transfer completed callback
 - HAL_SMBUS_SLAVE_RxCpltCallback()      : Slave Rx Transfer completed callback
 - HAL_SMBUS_SLAVE_AddrCallback()        : Slave Address Match callback
 - HAL_SMBUS_SLAVE_ListenCpltCallback()  : Slave listen completed callback
 - HAL_SMBUS_ErrorCallback()             : SMBUS error callback
 - HAL_SMBUS_AbortCpltCallback()         : SMBUS abort complete callback
 */

/**
  * @brief  Master Tx transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_MASTER_TxCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_MASTER_TxCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Master Rx transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_MASTER_RxCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_MASTER_RxCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Slave Tx transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_SLAVE_TxCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_SLAVE_TxCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Slave Rx transfer completed callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_SLAVE_RxCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_SLAVE_RxCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Slave address match callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  xfer_direction Master request Transfer Direction (Write/Read)
  * @param  addr_match_code Address Match Code
  */
__WEAK void HAL_SMBUS_SLAVE_AddrCallback(hal_smbus_handle_t *hsmbus,
                                         hal_smbus_slave_xfer_direction_t xfer_direction,
                                         uint32_t addr_match_code)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);
  STM32_UNUSED(xfer_direction);
  STM32_UNUSED(addr_match_code);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_SLAVE_AddrCallback() could be implemented in the user file
    */
}

/**
  * @brief  Slave listen complete callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_SLAVE_ListenCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_SLAVE_ListenCpltCallback() could be implemented in the user file
    */
}

/**
  * @brief  SMBUS error callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_ErrorCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_ErrorCallback could be implemented in the user file
    */
}

/**
  * @brief  SMBUS abort complete callback.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
__WEAK void HAL_SMBUS_AbortCpltCallback(hal_smbus_handle_t *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hsmbus);

  /** @warning : This function must not be modified, when the callback is needed,
    *            the HAL_SMBUS_AbortCpltCallback could be implemented in the user file
    */
}

/**
  * @}
  */

/** @addtogroup SMBUS_Exported_Functions_Group6
  * @{
A set of functions to retrieve peripheral state and last process errors.
 - HAL_SMBUS_GetState(): Return the SMBUS handle state.
 - HAL_SMBUS_GetLastErrorCodes(): Return errors limited to the last process.
 - HAL_SMBUS_GetClockFreq(): Retrieve the HAL SMBUS instance kernel clock frequency.
  */

/**
  * @brief  Return the SMBUS handle state.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @retval HAL state.
  */
hal_smbus_state_t HAL_SMBUS_GetState(const hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  /* Return SMBUS handle state */
  return hsmbus->global_state;
}

#if defined (USE_HAL_SMBUS_GET_LAST_ERRORS) && (USE_HAL_SMBUS_GET_LAST_ERRORS == 1)
/**
  * @brief  Return errors limited to the last process.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval uint32_t Last error code. It can be NULL or a combinaison of the following values:
  *         @arg @ref HAL_SMBUS_ERROR_BERR,
  *         @arg @ref HAL_SMBUS_ERROR_ARLO,
  *         @arg @ref HAL_SMBUS_ERROR_ACKF,
  *         @arg @ref HAL_SMBUS_ERROR_OVR,
  *         @arg @ref HAL_SMBUS_ERROR_BUSTIMEOUT,
  *         @arg @ref HAL_SMBUS_ERROR_ALERT,
  *         @arg @ref HAL_SMBUS_ERROR_PECERR,
  */
uint32_t HAL_SMBUS_GetLastErrorCodes(const hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_IDLE | (uint32_t)HAL_SMBUS_STATE_TX
                   | (uint32_t)HAL_SMBUS_STATE_RX | (uint32_t)HAL_SMBUS_STATE_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_ABORT);

  return hsmbus->last_error_codes;
}
#endif /* USE_HAL_SMBUS_GET_LAST_ERRORS */

/** @brief  Return the peripheral clock frequency for SMBUS.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @retval uint32_t Frequency in Hz.
  *                  0 if the source clock of the SMBUS is not configured or not ready.
  */
uint32_t HAL_SMBUS_GetClockFreq(const hal_smbus_handle_t *hsmbus)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM((hsmbus != NULL));
  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE
                   | (uint32_t)HAL_SMBUS_STATE_TX | (uint32_t)HAL_SMBUS_STATE_RX
                   | (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN | (uint32_t)HAL_SMBUS_STATE_ABORT);

  return HAL_RCC_I2C_GetKernelClkFreq((I2C_TypeDef *)((uint32_t)hsmbus->instance));
}
/**
  * @}
  */

#if defined(USE_HAL_MUTEX) && (USE_HAL_MUTEX == 1)
/** @addtogroup SMBUS_Exported_Functions_Group7
  * @{
A set of functions to acquire and release the bus based on the HAL OS abstraction layer (stm32_hal_os.c/.h osal):
 - HAL_SMBUS_AcquireBus(): Acquire the SMBUS bus.
 - HAL_SMBUS_ReleaseBus(): Release the SMBUS bus.
  */

/**
  * @brief  Acquire the SMBUS bus using the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param  timeout_ms Timeout duration in milliseconds.
  * @note   Call HAL_SMBUS_AcquireBus from thread mode only (not from handler mode, e.g., from ISR).
  * @retval HAL_OK    Operation completed successfully.
  * @retval HAL_ERROR Operation completed with error.
  */
hal_status_t HAL_SMBUS_AcquireBus(hal_smbus_handle_t *hsmbus, uint32_t timeout_ms)
{
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE
                   | (uint32_t)HAL_SMBUS_STATE_TX | (uint32_t)HAL_SMBUS_STATE_RX
                   | (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN | (uint32_t)HAL_SMBUS_STATE_ABORT);

  if (HAL_OS_SemaphoreTake(&hsmbus->semaphore, timeout_ms) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Release the SMBUS bus thanks to the the HAL OS abstraction layer (stm32_hal_os.c/.h osal).
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @note   The HAL_SMBUS_ReleaseBus can be called from thread mode or from handler mode e.g from ISR.
  * @retval HAL_OK    Operation completed successfully
  * @retval HAL_ERROR Operation completed with error
  */
hal_status_t HAL_SMBUS_ReleaseBus(hal_smbus_handle_t *hsmbus)
{
  hal_status_t status = HAL_ERROR;
  ASSERT_DBG_PARAM((hsmbus != NULL));

  ASSERT_DBG_STATE(hsmbus->global_state, (uint32_t)HAL_SMBUS_STATE_INIT | (uint32_t)HAL_SMBUS_STATE_IDLE
                   | (uint32_t)HAL_SMBUS_STATE_TX | (uint32_t)HAL_SMBUS_STATE_RX
                   | (uint32_t)HAL_SMBUS_STATE_LISTEN | (uint32_t)HAL_SMBUS_STATE_RX_LISTEN
                   | (uint32_t)HAL_SMBUS_STATE_TX_LISTEN | (uint32_t)HAL_SMBUS_STATE_ABORT);

  if (HAL_OS_SemaphoreRelease(&hsmbus->semaphore) == HAL_OS_OK)
  {
    status = HAL_OK;
  }

  return status;
}

/**
  * @}
  */
#endif /* USE_HAL_MUTEX */

#if defined (USE_HAL_SMBUS_USER_DATA) && (USE_HAL_SMBUS_USER_DATA == 1)
/** @addtogroup SMBUS_Exported_Functions_Group8
  * @{
A set of functions to manage a user data pointer stored in the SMBUS handle:
 - HAL_SMBUS_SetUserData(): Set the user data in the handle.
 - HAL_SMBUS_GetUserData(): Get the user data from the handle.
  */

/**
  * @brief Set the user data pointer in the handle.
  * @param hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @param p_user_data Pointer to the user data.
  */
void HAL_SMBUS_SetUserData(hal_smbus_handle_t *hsmbus, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hsmbus != NULL);

  hsmbus->p_user_data = p_user_data;
}

/**
  * @brief  Get the user data pointer from the handle.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t.
  * @retval Pointer to the user data.
  */
const void *HAL_SMBUS_GetUserData(const hal_smbus_handle_t *hsmbus)
{
  ASSERT_DBG_PARAM(hsmbus != NULL);

  return (hsmbus->p_user_data);
}
/**
  * @}
  */
#endif /* USE_HAL_SMBUS_USER_DATA */
/**
  * @}
  */

/** @addtogroup SMBUS_Private_Functions SMBUS Private Functions
  * @{
  */

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  it_flags Value of Interrupt Flags.
  * @param  it_sources Interrupt sources enabled.
  * @retval HAL status
  */
static hal_status_t SMBUS_Master_ISR(hal_smbus_handle_t *hsmbus, uint32_t it_flags, uint32_t it_sources)
{
  uint16_t device_addr;

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_NACKF) != 0U)
  {
    LL_I2C_ClearFlag_NACK(p_i2cx);

    /* Set corresponding error code */
    /* No need to generate STOP, it is automatically done */
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_ACKF;

    SMBUS_Flush_TXDR(hsmbus);

    /* Call the error callback to inform upper layer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->p_error_cb(hsmbus);
#else
    HAL_SMBUS_ErrorCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_STOPF) != 0U)
  {
    /* Check and treat errors if errors occurs during STOP process */
    SMBUS_ITErrorHandler(hsmbus);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    if (hsmbus->global_state == HAL_SMBUS_STATE_TX)
    {
      SMBUS_Disable_IRQ(hsmbus, SMBUS_TX_IT_MASK);

      LL_I2C_ClearFlag_STOP(p_i2cx);

      /* Clear configuration register 2 */
      STM32_CLEAR_BIT(p_i2cx->CR2, I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
                      | LL_I2C_MODE_RELOAD | LL_I2C_REQUEST_READ);

      /* Flush remaining data in Fifo register in case of error occurs before TXEmpty */
      /* Disable the selected SMBUS functionality in the I2Cx peripheral */
      LL_I2C_Disable(p_i2cx);

      hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_IDLE;
      hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

      /* Re-enable the selected SMBUS functionality in the I2Cx peripheral */
      LL_I2C_Enable(p_i2cx);

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_master_tx_cplt_cb(hsmbus);
#else
      HAL_SMBUS_MASTER_TxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
    else if (hsmbus->global_state == HAL_SMBUS_STATE_RX)
    {
      /* Store last receive data if any */
      if ((SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_RXNE) != 0U)
          && (SMBUS_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_RXIE) != 0U))
      {
        /* Read data from RXDR */
        *hsmbus->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);

        /* Increment buffer pointer */
        hsmbus->p_buf_rx++;

        if ((hsmbus->xfer_size > 0U))
        {
          hsmbus->xfer_size--;
          hsmbus->xfer_count--;
        }
      }

      SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK);

      LL_I2C_ClearFlag_STOP(p_i2cx);

      /* Clear configuration register 2 */
      STM32_CLEAR_BIT(p_i2cx->CR2, I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
                      | LL_I2C_MODE_RELOAD | LL_I2C_REQUEST_READ);

      hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_IDLE;
      hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_master_rx_cplt_cb(hsmbus);
#else
      HAL_SMBUS_MASTER_RxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
    else if (hsmbus->global_state == HAL_SMBUS_STATE_ABORT)
    {
      SMBUS_Disable_IRQ(hsmbus, (SMBUS_TX_IT_MASK | SMBUS_RX_IT_MASK));

      LL_I2C_ClearFlag_STOP(p_i2cx);

      /* Clear Configuration Register 2 */
      STM32_CLEAR_BIT(p_i2cx->CR2, I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
                      | LL_I2C_MODE_RELOAD | LL_I2C_REQUEST_READ);

      /* Flush remaining data in Fifo register in case of error occurs before TXEmpty */
      /* Disable the selected SMBUS functionality in the I2Cx peripheral */
      LL_I2C_Disable(p_i2cx);

      hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_IDLE;
      hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

      SMBUS_Flush_TXDR(hsmbus);

      /* Re-enable the selected SMBUS functionality in the I2Cx peripheral */
      LL_I2C_Enable(p_i2cx);

#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_abort_cplt_cb(hsmbus);
#else
      HAL_SMBUS_AbortCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
    else
    {
      /* Nothing to do */
    }
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_RXNE) != 0U)
  {
    /* Read data from RXDR */
    *hsmbus->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);

    /* Increment buffer pointer */
    hsmbus->p_buf_rx++;

    /* Increment size counter */
    hsmbus->xfer_size--;
    hsmbus->xfer_count--;
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_TXIS) != 0U)
  {
    /* Write data to TXDR */
    LL_I2C_TransmitData8(p_i2cx, *hsmbus->p_buf_tx);

    /* Increment buffer pointer */
    hsmbus->p_buf_tx++;

    /* Increment size counter */
    hsmbus->xfer_size--;
    hsmbus->xfer_count--;
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_TCR) != 0U)
  {
    if ((hsmbus->xfer_count != 0U) && (hsmbus->xfer_size == 0U))
    {
      device_addr = (uint16_t)LL_I2C_GetSlaveAddr(p_i2cx);

      if (hsmbus->xfer_count > MAX_NBYTE_SIZE)
      {
        SMBUS_TransferConfig(p_i2cx, device_addr, MAX_NBYTE_SIZE,
                             (SMBUS_RELOAD_MODE | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE)),
                             SMBUS_NO_STARTSTOP);
        hsmbus->xfer_size = MAX_NBYTE_SIZE;
      }
      else
      {
        hsmbus->xfer_size = hsmbus->xfer_count;
        SMBUS_TransferConfig(p_i2cx, device_addr, hsmbus->xfer_size, (uint32_t)hsmbus->xfer_opt,
                             SMBUS_NO_STARTSTOP);
        /* If PEC mode is enabled, size to transmit must be Size-1 byte, corresponding to PEC byte */
        /* PEC byte is automatically sent by HW block, no need to manage it in transmit process */
        if (LL_I2C_IsEnabledSMBusPEC(p_i2cx) != 0U)
        {
          hsmbus->xfer_size--;
          hsmbus->xfer_count--;
        }
      }
    }
    else if ((hsmbus->xfer_count == 0U) && (hsmbus->xfer_size == 0U))
    {
      /* Call TxCpltCallback() if no stop mode is set */
      if (SMBUS_GET_STOP_MODE(hsmbus) != SMBUS_AUTOEND_MODE)
      {
        /* Call the corresponding callback to inform upper layer of end of transfer */
        if (hsmbus->global_state == HAL_SMBUS_STATE_TX)
        {
          SMBUS_Disable_IRQ(hsmbus, SMBUS_TX_IT_MASK);
          hsmbus->previous_state = (uint32_t)hsmbus->global_state;
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

          /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
          hsmbus->p_master_tx_cplt_cb(hsmbus);
#else
          HAL_SMBUS_MASTER_TxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
        }
        else if (hsmbus->global_state == HAL_SMBUS_STATE_RX)
        {
          SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK);
          hsmbus->previous_state = (uint32_t)hsmbus->global_state;
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

          /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
          hsmbus->p_master_rx_cplt_cb(hsmbus);
#else
          HAL_SMBUS_MASTER_RxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
        }
        else
        {
          /* Nothing to do */
        }
      }
    }
    else
    {
      /* Nothing to do */
    }
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_TC) != 0U)
  {
    if (hsmbus->xfer_count == 0U)
    {
      /* Specific use case for quick command */
      if (hsmbus->p_buf_tx == NULL)
      {
        LL_I2C_GenerateStopCondition(p_i2cx);
      }
      /* Call TxCpltCallback() if no stop mode is set */
      else if (SMBUS_GET_STOP_MODE(hsmbus) != SMBUS_AUTOEND_MODE)
      {
        /* No generate stop, to allow restart mode */
        /* The stop will be done at the end of transfer, when SMBUS_AUTOEND_MODE enable */

        /* Call the corresponding callback to inform upper layer of end of transfer */
        if (hsmbus->global_state == HAL_SMBUS_STATE_TX)
        {
          SMBUS_Disable_IRQ(hsmbus, SMBUS_TX_IT_MASK);
          hsmbus->previous_state = (uint32_t)hsmbus->global_state;
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

          /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
          hsmbus->p_master_tx_cplt_cb(hsmbus);
#else
          HAL_SMBUS_MASTER_TxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
        }
        else if (hsmbus->global_state == HAL_SMBUS_STATE_RX)
        {
          SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK);
          hsmbus->previous_state = (uint32_t)hsmbus->global_state;
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

          /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
          hsmbus->p_master_rx_cplt_cb(hsmbus);
#else
          HAL_SMBUS_MASTER_RxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
        }
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        /* Nothing to do */
      }
    }
  }
  else
  {
    /* Nothing to do */
  }

  return HAL_OK;
}
/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  it_flags Value of Interrupt Flags.
  * @param  it_sources Interrupt sources enabled.
  * @retval HAL status
  */
static hal_status_t SMBUS_Slave_ISR(hal_smbus_handle_t *hsmbus, uint32_t it_flags, uint32_t it_sources)
{
  hal_smbus_slave_xfer_direction_t TransferDirection;
  uint32_t SlaveAddrCode;

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_NACKF) != 0U)
  {
    /* Check that SMBUS transfer finished */
    /* if yes, normal usecase, a NACK is sent by the HOST when Transfer is finished */
    /* Mean XferCount == 0 */
    /* So clear Flag NACKF only */
    if (hsmbus->xfer_count == 0U)
    {
      LL_I2C_ClearFlag_NACK(p_i2cx);

      SMBUS_Flush_TXDR(hsmbus);

      hsmbus->global_state = HAL_SMBUS_STATE_LISTEN;
    }
    else
    {
      /* If no, error usecase, a non-acknowledge of last data is generated by the HOST */
      LL_I2C_ClearFlag_NACK(p_i2cx);

      /* Set HAL state to "Idle" state, mean to LISTEN state */
      /* So reset slave busy state */
      hsmbus->previous_state = (uint32_t)hsmbus->global_state;
      hsmbus->global_state = HAL_SMBUS_STATE_LISTEN;

      /* Disable RX/TX interrupts, keep only ADDR interrupt */
      SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK | SMBUS_TX_IT_MASK);

      /* Set error code corresponding to a non-acknowledge */
      hsmbus->last_error_codes |= HAL_SMBUS_ERROR_ACKF;

      SMBUS_Flush_TXDR(hsmbus);

      /* Call the error callback to inform upper layer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_error_cb(hsmbus);
#else
      HAL_SMBUS_ErrorCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_ADDR) != 0U)
  {
    TransferDirection = (hal_smbus_slave_xfer_direction_t)(SMBUS_GET_DIR(hsmbus));
    SlaveAddrCode = SMBUS_GET_ADDR_MATCH(hsmbus);

    /* Disable ADDR interrupt to prevent multiple ADDRInterrupt */
    /* Other ADDRInterrupt will be treat in next listen usecase */
    SMBUS_Disable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK);

    /* Call slave addr callback */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->p_slave_addr_cb(hsmbus, TransferDirection, SlaveAddrCode);
#else
    HAL_SMBUS_SLAVE_AddrCallback(hsmbus, TransferDirection, SlaveAddrCode);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
  }
  else if ((SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_RXNE) != 0U)
           || (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_TCR) != 0U))
  {
    if (hsmbus->global_state == HAL_SMBUS_STATE_RX_LISTEN)
    {
      /* Read data from RXDR */
      *hsmbus->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);

      /* Increment buffer pointer */
      hsmbus->p_buf_rx++;

      hsmbus->xfer_size--;
      hsmbus->xfer_count--;

      if (hsmbus->xfer_count == 1U)
      {
        /* Receive last byte, can be PEC byte in case of PEC BYTE enabled */
        /* or only the last byte of transfer */
        /* So reset the RELOAD bit mode */
        hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_FRAME;
        SMBUS_TransferConfig(p_i2cx, 0, 1, (uint32_t)hsmbus->xfer_opt, SMBUS_NO_STARTSTOP);
      }
      else if (hsmbus->xfer_count == 0U)
      {
        /* Last byte is received, disable interrupt */
        SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK);

        /* Remove HAL_SMBUS_STATE_SLAVE_BUSY_RX, keep only HAL_SMBUS_STATE_LISTEN */
        hsmbus->previous_state = (uint32_t)hsmbus->global_state;
        hsmbus->global_state = HAL_SMBUS_STATE_LISTEN;

        /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
        hsmbus->p_slave_rx_cplt_cb(hsmbus);
#else
        HAL_SMBUS_SLAVE_RxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
      }
      else
      {
        /* Set reload for next bytes */
        SMBUS_TransferConfig(p_i2cx, 0, 1,
                             SMBUS_RELOAD_MODE  | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE),
                             SMBUS_NO_STARTSTOP);

        /* Ack last byte read */
        LL_I2C_AcknowledgeEnable(p_i2cx);
      }
    }
    else if (hsmbus->global_state == HAL_SMBUS_STATE_TX_LISTEN)
    {
      if ((hsmbus->xfer_count != 0U) && (hsmbus->xfer_size == 0U))
      {
        if (hsmbus->xfer_count > MAX_NBYTE_SIZE)
        {
          SMBUS_TransferConfig(p_i2cx, 0, MAX_NBYTE_SIZE,
                               (SMBUS_RELOAD_MODE | ((uint32_t)hsmbus->xfer_opt & SMBUS_SENDPEC_MODE)),
                               SMBUS_NO_STARTSTOP);
          hsmbus->xfer_size = MAX_NBYTE_SIZE;
        }
        else
        {
          hsmbus->xfer_size = hsmbus->xfer_count;
          SMBUS_TransferConfig(p_i2cx, 0, (uint32_t)hsmbus->xfer_size, (uint32_t)hsmbus->xfer_opt,
                               SMBUS_NO_STARTSTOP);
          /* If PEC mode is enabled, size to transmit must be size-1 byte, corresponding to PEC byte */
          /* PEC byte is automatically sent by HW block, no need to manage it in transmit process */
          if (LL_I2C_IsEnabledSMBusPEC(p_i2cx) != 0U)
          {
            hsmbus->xfer_size--;
            hsmbus->xfer_count--;
          }
        }
      }
    }
    else
    {
      /* Nothing to do */
    }
  }
  else if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_TXIS) != 0U)
  {
    /* Write data to TXDR only if XferCount not reach "0" */
    /* A TXIS flag can be set, during STOP treatment */
    /* Check if all data have already been sent */
    /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
    if (hsmbus->xfer_count > 0U)
    {
      /* Write data to TXDR */
      LL_I2C_TransmitData8(p_i2cx, *hsmbus->p_buf_tx);

      /* Increment buffer pointer */
      hsmbus->p_buf_tx++;

      hsmbus->xfer_count--;
      hsmbus->xfer_size--;
    }

    if (hsmbus->xfer_count == 0U)
    {
      /* Last byte is Transmitted */
      /* Remove HAL_SMBUS_STATE_SLAVE_BUSY_TX, keep only HAL_SMBUS_STATE_LISTEN */
      SMBUS_Disable_IRQ(hsmbus, SMBUS_TX_IT_MASK);
      hsmbus->previous_state = (uint32_t)hsmbus->global_state;
      hsmbus->global_state = HAL_SMBUS_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of end of transfer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_slave_tx_cplt_cb(hsmbus);
#else
      HAL_SMBUS_SLAVE_TxCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Nothing to do */
  }

  /* Check if STOPF is set */
  if ((SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_STOPF) != 0U)
      && (SMBUS_CHECK_IT_SOURCE(it_sources, LL_I2C_CR1_STOPIE) != 0U))
  {
    uint32_t tmp_state = (uint32_t)hsmbus->global_state;
    if ((tmp_state == (uint32_t)HAL_SMBUS_STATE_LISTEN)
        || (tmp_state == (uint32_t)HAL_SMBUS_STATE_TX_LISTEN)
        || (tmp_state == (uint32_t)HAL_SMBUS_STATE_RX_LISTEN)
        || (tmp_state == (uint32_t)HAL_SMBUS_STATE_ABORT))
    {
      /* Store last receive data if any */
      if (SMBUS_CHECK_FLAG(it_flags, LL_I2C_ISR_RXNE) != 0U)
      {
        /* Read data from RXDR */
        *hsmbus->p_buf_rx = LL_I2C_ReceiveData8(p_i2cx);

        /* Increment buffer pointer */
        hsmbus->p_buf_rx++;

        if ((hsmbus->xfer_size > 0U))
        {
          hsmbus->xfer_size--;
          hsmbus->xfer_count--;
        }
      }

      /* Disable RX and TXiInterrupts */
      SMBUS_Disable_IRQ(hsmbus, SMBUS_RX_IT_MASK | SMBUS_TX_IT_MASK);

      /* Disable ADDR interrupt */
      SMBUS_Disable_IRQ(hsmbus, SMBUS_ADDR_IT_MASK);

      /* Disable address acknowledge */
      LL_I2C_AcknowledgeDisable(p_i2cx);

      /* Clear configuration register 2 */
      STM32_CLEAR_BIT(p_i2cx->CR2, I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
                      | LL_I2C_MODE_RELOAD | LL_I2C_REQUEST_READ);

      LL_I2C_ClearFlag_STOP(p_i2cx);

      LL_I2C_ClearFlag_ADDR(p_i2cx);

      hsmbus->xfer_opt = (hal_smbus_xfer_opt_t)0U;
      hsmbus->previous_state = (uint32_t)hsmbus->global_state;
      hsmbus->global_state = HAL_SMBUS_STATE_IDLE;

      /* Call the listen complete callback, to inform upper layer of the end of listen usecase */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->p_slave_listen_cplt_cb(hsmbus);
#else
      HAL_SMBUS_SLAVE_ListenCpltCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
    }
  }

  return HAL_OK;
}
/**
  * @brief  Manage the enabling of Interrupts.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  it_request Value of @ref SMBUS_Interrupt_configuration_mask.
  */
static void SMBUS_Enable_IRQ(hal_smbus_handle_t *hsmbus, uint32_t it_request)
{
  uint32_t tmpisr = 0U;

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if ((it_request & SMBUS_ALERT_IT_MASK) == SMBUS_ALERT_IT_MASK)
  {
    /* Enable ERR interrupt */
    tmpisr |= LL_I2C_CR1_ERRIE;
  }

  if ((it_request & SMBUS_ADDR_IT_MASK) == SMBUS_ADDR_IT_MASK)
  {
    /* Enable ADDR, STOP interrupt */
    tmpisr |= LL_I2C_CR1_ADDRIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_ERRIE;
  }

  if ((it_request & SMBUS_TX_IT_MASK) == SMBUS_TX_IT_MASK)
  {
    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    tmpisr |= LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_TXIE;
  }

  if ((it_request & SMBUS_RX_IT_MASK) == SMBUS_RX_IT_MASK)
  {
    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    tmpisr |= LL_I2C_CR1_ERRIE | LL_I2C_CR1_TCIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE | LL_I2C_CR1_RXIE;
  }

  /* Enable interrupts only at the end */
  /* to avoid the risk of SMBUS interrupt handle execution before */
  /* all interrupts requested done */
  LL_I2C_EnableIT(p_i2cx, tmpisr);
}

/**
  * @brief  Manage the disabling of interrupts.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  it_request Value of @ref SMBUS_Interrupt_configuration_mask.
  */
static void SMBUS_Disable_IRQ(hal_smbus_handle_t *hsmbus, uint32_t it_request)
{
  uint32_t tmpisr = 0U;
  hal_smbus_state_t tmpstate = hsmbus->global_state;

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  if ((tmpstate == HAL_SMBUS_STATE_IDLE) && ((it_request & SMBUS_ALERT_IT_MASK) == SMBUS_ALERT_IT_MASK))
  {
    /* Disable ERR interrupt */
    tmpisr |= LL_I2C_CR1_ERRIE;
  }

  if ((it_request & SMBUS_TX_IT_MASK) == SMBUS_TX_IT_MASK)
  {
    /* Disable TC, STOP, NACK and TXI interrupt */
    tmpisr |= LL_I2C_CR1_TCIE | LL_I2C_CR1_TXIE;

    if ((LL_I2C_IsEnabledSMBusAlert(p_i2cx) != 0U)
        && ((tmpstate != HAL_SMBUS_STATE_LISTEN)))
    {
      /* Disable ERR interrupt */
      tmpisr |= LL_I2C_CR1_ERRIE;
    }

    if ((tmpstate != HAL_SMBUS_STATE_TX_LISTEN) && (tmpstate != HAL_SMBUS_STATE_LISTEN))
    {
      /* Disable STOP and NACK interrupt */
      tmpisr |= LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE;
    }
  }

  if ((it_request & SMBUS_RX_IT_MASK) == SMBUS_RX_IT_MASK)
  {
    /* Disable TC, STOP, NACK and RXI interrupt */
    tmpisr |= LL_I2C_CR1_TCIE | LL_I2C_CR1_RXIE;

    if ((LL_I2C_IsEnabledSMBusAlert(p_i2cx) != 0U)
        && ((tmpstate != HAL_SMBUS_STATE_RX_LISTEN) && (tmpstate != HAL_SMBUS_STATE_LISTEN)))
    {
      /* Disable ERR interrupt */
      tmpisr |= LL_I2C_CR1_ERRIE;
    }

    if (((tmpstate != HAL_SMBUS_STATE_RX_LISTEN) && (tmpstate != HAL_SMBUS_STATE_LISTEN)))
    {
      /* Disable STOP and NACK interrupt */
      tmpisr |= LL_I2C_CR1_STOPIE | LL_I2C_CR1_NACKIE;
    }
  }

  if ((it_request & SMBUS_ADDR_IT_MASK) == SMBUS_ADDR_IT_MASK)
  {
    /* Disable ADDR and NACK interrupt */
    tmpisr |= LL_I2C_CR1_ADDRIE | LL_I2C_CR1_NACKIE;

    if (LL_I2C_IsEnabledSMBusAlert(p_i2cx) != 0U)
    {
      /* Disable ERR interrupt */
      tmpisr |= LL_I2C_CR1_ERRIE;
    }
  }

  /* Disable interrupts only at the end */
  /* to avoid a breaking situation like at "t" time */
  /* all disable interrupts request are not done */
  LL_I2C_DisableIT(p_i2cx, tmpisr);
}

/**
  * @brief  SMBUS interrupts error handler.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
static void SMBUS_ITErrorHandler(hal_smbus_handle_t *hsmbus)
{

  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  uint32_t itflags   = LL_I2C_READ_REG(p_i2cx, ISR);
  uint32_t itsources = LL_I2C_READ_REG(p_i2cx, CR1);
  hal_smbus_state_t tmpstate;
  uint32_t tmperror;

  /* SMBUS Bus error interrupt occurred ------------------------------------*/
  if (((itflags & LL_I2C_ISR_BERR) == LL_I2C_ISR_BERR)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_BERR;

    LL_I2C_ClearFlag_BERR(p_i2cx);
  }

  /* SMBUS Over-Run/Under-Run interrupt occurred ----------------------------------------*/
  if (((itflags & LL_I2C_ISR_OVR) == LL_I2C_ISR_OVR)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_OVR;

    LL_I2C_ClearFlag_OVR(p_i2cx);
  }

  /* SMBUS Arbitration Loss error interrupt occurred ------------------------------------*/
  if (((itflags & LL_I2C_ISR_ARLO) == LL_I2C_ISR_ARLO)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_ARLO;

    LL_I2C_ClearFlag_ARLO(p_i2cx);
  }

  /* SMBUS Timeout error interrupt occurred ---------------------------------------------*/
  if (((itflags & LL_I2C_ISR_TIMEOUT) == LL_I2C_ISR_TIMEOUT)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_BUSTIMEOUT;

    LL_I2C_ClearSMBusFlag_TIMEOUT(p_i2cx);
  }

  /* SMBUS Alert error interrupt occurred -----------------------------------------------*/
  if (((itflags & LL_I2C_ISR_ALERT) == LL_I2C_ISR_ALERT)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_ALERT;

    LL_I2C_ClearSMBusFlag_ALERT(p_i2cx);
  }

  /* SMBUS Packet Error Check error interrupt occurred ----------------------------------*/
  if (((itflags & LL_I2C_ISR_PECERR) == LL_I2C_ISR_PECERR)
      && ((itsources & LL_I2C_CR1_ERRIE) == LL_I2C_CR1_ERRIE))
  {
    hsmbus->last_error_codes |= HAL_SMBUS_ERROR_PECERR;

    LL_I2C_ClearSMBusFlag_PECERR(p_i2cx);
  }

  if (hsmbus->last_error_codes != HAL_SMBUS_ERROR_NONE)
  {
    SMBUS_Flush_TXDR(hsmbus);
  }

  /* Store current volatile hsmbus->last_error_codes, misra rule */
  tmperror = hsmbus->last_error_codes;

  /* Call the Error Callback in case of Error detected */
  if ((tmperror != 0U) && (tmperror != HAL_SMBUS_ERROR_ACKF))
  {
    /* Do not Reset the HAL state in case of ALERT error */
    if ((tmperror & HAL_SMBUS_ERROR_ALERT) != HAL_SMBUS_ERROR_ALERT)
    {
      /* Store current volatile hsmbus->global_state, misra rule */
      tmpstate = hsmbus->global_state;

      if ((tmpstate == HAL_SMBUS_STATE_RX_LISTEN) || (tmpstate == HAL_SMBUS_STATE_TX_LISTEN)
          || (tmpstate == HAL_SMBUS_STATE_ABORT))
      {
        /* Reset only HAL_SMBUS_STATE_SLAVE_BUSY_XX */
        /* keep HAL_SMBUS_STATE_LISTEN if set */
        hsmbus->previous_state = (uint32_t)HAL_SMBUS_STATE_IDLE;
        hsmbus->global_state = HAL_SMBUS_STATE_LISTEN;
      }
    }

    /* Call the Error callback to inform upper layer */
#if defined (USE_HAL_SMBUS_REGISTER_CALLBACKS) && (USE_HAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->p_error_cb(hsmbus);
#else
    HAL_SMBUS_ErrorCallback(hsmbus);
#endif /* USE_HAL_SMBUS_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  This function handles errors detection during a SMBUS communication.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  * @param  timeout_ms Timeout duration in millisecond
  * @param  tick_start Tick start value
  * @retval HAL_OK     Operation completed successfully
  * @retval HAL_ERROR  Operation completed with error
  */
static hal_status_t SMBUS_IsErrorOccurred(hal_smbus_handle_t *hsmbus, uint32_t timeout_ms, uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);
  hal_status_t status = HAL_OK;
  uint32_t it_flag = LL_I2C_READ_REG(p_i2cx, ISR);
  uint32_t error_codes = 0U;
  uint32_t tick_start_local = tick_start;
  uint32_t tmp_register;
  hal_smbus_mode_t tmp_mode;

  if (STM32_IS_BIT_SET(it_flag, LL_I2C_ISR_NACKF))
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
          tmp_mode = (hal_smbus_mode_t)LL_I2C_GetMode(p_i2cx);

          /* In case of I2C still busy, try to regenerate a STOP manually */
          if ((LL_I2C_IsActiveFlag_BUSY(p_i2cx) != (uint32_t)0U)
              && (tmp_register != I2C_CR2_STOP)
              && (tmp_mode == HAL_SMBUS_PERIPHERAL_MODE_HOST))
          {
            LL_I2C_GenerateStopCondition(p_i2cx);

            /* Update tick with new reference */
            tick_start_local = HAL_GetTick();
          }

          while (LL_I2C_IsActiveFlag_STOP(p_i2cx) == 0U)
          {
            /* Check for the timeout */
            if ((HAL_GetTick() - tick_start_local) > SMBUS_DEFAULT_TIMEOUT_MS)
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
    error_codes |= HAL_SMBUS_ERROR_ACKF;
    status = HAL_ERROR;
  }

  /* Refresh content of status register */
  it_flag = LL_I2C_READ_REG(p_i2cx, ISR);

  /* Verify if additional errors occur */
  /* Check if a Bus error occurred */
  if (STM32_IS_BIT_SET(it_flag, LL_I2C_ISR_BERR))
  {
    error_codes |= HAL_SMBUS_ERROR_BERR;
    LL_I2C_ClearFlag_BERR(p_i2cx);
    status = HAL_ERROR;
  }

  /* Check if an Over-Run/Under-Run error occurred */
  if (STM32_IS_BIT_SET(it_flag, LL_I2C_ISR_OVR))
  {
    error_codes |= HAL_SMBUS_ERROR_OVR;
    LL_I2C_ClearFlag_OVR(p_i2cx);
    status = HAL_ERROR;
  }

  /* Check if an arbitration loss error occurred */
  if (STM32_IS_BIT_SET(it_flag, LL_I2C_ISR_ARLO))
  {
    error_codes |= HAL_SMBUS_ERROR_ARLO;

    LL_I2C_ClearFlag_ARLO(p_i2cx);
    status = HAL_ERROR;
  }

  if (status != HAL_OK)
  {
    SMBUS_Flush_TXDR(hsmbus);
    I2C_RESET_CR2(p_i2cx);

    hsmbus->last_error_codes |= error_codes;
  }

  return status;
}

/**
  * @brief  Handle SMBUS communication timeout.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  flag Specifies the SMBUS flag to check.
  * @param  status The new flag status (SET or RESET).
  * @param  timeout_ms Timeout duration
  * @param  tick_start Tick start value
  * @retval HAL status
  */
static hal_status_t SMBUS_WaitOnFlagUntilTimeout(hal_smbus_handle_t *hsmbus, uint32_t flag, uint32_t status,
                                                 uint32_t timeout_ms, uint32_t tick_start)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* Wait until flag is set */
  while ((uint32_t)(LL_I2C_IsActiveFlag(p_i2cx, flag)) == status)
  {
    /* Check if an error is detected */
    if (SMBUS_IsErrorOccurred(hsmbus, timeout_ms, tick_start) != HAL_OK)
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
          hsmbus->previous_state = (uint32_t)hsmbus->global_state;
          hsmbus->global_state = HAL_SMBUS_STATE_IDLE;
          return HAL_TIMEOUT;
        }
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  SMBUS Tx data register flush process.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
static void SMBUS_Flush_TXDR(hal_smbus_handle_t *hsmbus)
{
  I2C_TypeDef *p_i2cx = I2C_GET_INSTANCE(hsmbus);

  /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
  if (LL_I2C_IsActiveFlag_TXIS(p_i2cx) != 0U)
  {
    LL_I2C_TransmitData8(p_i2cx, 0x00U);
  }

  /* Flush TX register if not empty */
  if (LL_I2C_IsActiveFlag_TXE(p_i2cx) == 0U)
  {
    LL_I2C_ClearFlag_TXE(p_i2cx);
  }
}

/**
  * @brief  Handle SMBUS communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  p_i2cx Pointer to a I2C_TypeDef
  * @param  device_addr Specifies the slave address to be programmed
  * @param  size_byte Specifies the number of bytes to be programmed. It must be a value between 0 and 255.
  * @param  mode New state of the SMBUS START condition generation :
  *     @arg @ref SMBUS_RELOAD_MODE Enable Reload mode.
  *     @arg @ref SMBUS_AUTOEND_MODE Automatic end mode.
  *     @arg @ref SMBUS_SOFTEND_MODE Enable Software end mode.
  *     @arg @ref SMBUS_SENDPEC_MODE Enable Packet Error Calculation mode.
  * @param  request New state of the SMBUS START condition generation :
  *     @arg @ref SMBUS_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref SMBUS_GENERATE_STOP Generate stop condition (size_byte must be set to 0).
  *     @arg @ref SMBUS_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref SMBUS_GENERATE_START_WRITE Generate Restart for write request.
  */
static void SMBUS_TransferConfig(I2C_TypeDef *p_i2cx,  uint32_t device_addr, uint32_t size_byte,
                                 uint32_t mode, smbus_start_stop_mode_t request)
{
  /* Update CR2 register */
  STM32_MODIFY_REG(p_i2cx->CR2,
                   ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
                     (I2C_CR2_RD_WRN & ((uint32_t)request >> (31UL - I2C_CR2_RD_WRN_Pos))) | I2C_CR2_START |
                     I2C_CR2_STOP | I2C_CR2_PECBYTE)),
                   (uint32_t)((device_addr & I2C_CR2_SADD) | ((size_byte << I2C_CR2_NBYTES_Pos)
                                                              & I2C_CR2_NBYTES) | mode | (uint32_t)request));
}

/**
  * @brief  Convert SMBUSx OTHER_xxx from @ref hal_smbus_xfer_opt_t to their functional equivalent.
  * @param  hsmbus Pointer to a @ref hal_smbus_handle_t
  */
static void SMBUS_ConvertOtherXferOptions(hal_smbus_handle_t *hsmbus)
{
  /* If user set xfer_opt to HAL_SMBUS_XFER_OTHER_FRAME_NO_PEC   */
  /* it request implicitly to generate a restart condition */
  /* set xfer_opt to HAL_SMBUS_XFER_FIRST_FRAME                  */
  if (hsmbus->xfer_opt == HAL_SMBUS_XFER_OTHER_FRAME_NO_PEC)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_FRAME;
  }
  /* else if user set xfer_opt to HAL_SMBUS_XFER_OTHER_FRAME_WITH_PEC */
  /* it request implicitly to generate a restart condition      */
  /* set xfer_opt to HAL_SMBUS_XFER_FIRST_FRAME_WITH_PEC  */
  else if (hsmbus->xfer_opt == HAL_SMBUS_XFER_OTHER_FRAME_WITH_PEC)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_FRAME_WITH_PEC;
  }
  /* else if user set xfer_opt to HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_NO_PEC */
  /* it request implicitly to generate a restart condition             */
  /* then generate a stop condition at the end of transfer             */
  /* set xfer_opt to HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_NO_PEC              */
  else if (hsmbus->xfer_opt == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_NO_PEC)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_NO_PEC;
  }
  /* else if user set xfer_opt to HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_WITH_PEC */
  /* it request implicitly to generate a restart condition               */
  /* then generate a stop condition at the end of transfer               */
  /* set xfer_opt to HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_WITH_PEC              */
  else if (hsmbus->xfer_opt == HAL_SMBUS_XFER_OTHER_AND_LAST_FRAME_WITH_PEC)
  {
    hsmbus->xfer_opt = HAL_SMBUS_XFER_FIRST_AND_LAST_FRAME_WITH_PEC;
  }
  else
  {
    /* Nothing to do */
  }
}
/**
  * @}
  */

#endif /* USE_HAL_SMBUS_MODULE */
#endif /* I2C1 || I2C2 */

/**
  * @}
  */

/**
  * @}
  */
