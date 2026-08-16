/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_adc.c
  * @brief   ADC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the ADC peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral state and errors functions
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
  **********************************************************************************************************************
  */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5XX_HAL_Driver
  * @{
  */
#if defined(ADC1) || defined(ADC2) || defined(ADC3)
#if defined(USE_HAL_ADC_MODULE) && (USE_HAL_ADC_MODULE == 1)

/** @addtogroup ADC
  * @{
  */
/** @defgroup ADC_Introduction ADC Introduction
  * @{

  - The ADC (analog to digital converter) hardware abstraction layer provides a set of APIs to interface with the
    STM32 ADC peripheral.
  - It simplifies the initialization, configuration and process of peripheral features.
    (including various modes such as polling, interrupt, DMA for efficient data transfer)
  - This abstraction layer ensures portability and ease of use across different STM32 series.

  The HAL ADC driver includes the following features:
   - Structures and functions organized in subblocks to use only the features needed
     - Base: ADC instance, groups regular and injected, channels
     - Optional: analog watchdog, oversampling, offset, post processing, multimode (combine multiple ADC instances)
   - Supports polling, interrupt, DMA for efficient data transfer

  */

/**
  * @}
  */

/** @defgroup ADC_How_To_Use ADC How To Use
  * @{

# How to use the ADC HAL module driver

## HAL ADC driver usage

- ADC configuration
  - System configuration (out of HAL ADC driver)
    - RCC to provide ADC kernel clock
    - GPIO to connect ADC channels to device pins (if ADC usage with channel other than internal ones)
    - CPU Cortex NVIC to configure interrupts lines (if ADC usage with interrupt)
    - DMA channel (if ADC usage with data transfer by DMA)
  - ADC peripheral configuration
    - ADC peripheral is structured in subblocks with each a specific scope.
      HAL ADC follows this structure with a configuration structure and associated function for each subblock.
        - Mandatory subblocks, all must be configured:
          - ADC instance
          - ADC channel
        - Mandatory subblocks, at least one must be configured:
          - ADC group regular (prefix REG): main group available on all STM32 series, all ADC instances.
            Intended for regular data stream.
          - ADC group injected (prefix INJ): alternate group, availability depending on STM32 series and ADC instances.
            Intended for occasional or priority conversions.
        - Optional subblocks
          - Analog watchdog
          - Oversampling
          - Offset
          - Multimode (prefix MM): encompass multiple ADC instances (one master, some slaves) for synchronized
            operation.
    - ADC instances can belong to an ADC common instance, in this case they can share features (clock configuration,
      multimode capability, ...). HAL ADC driver provides a mechanism to link HAL ADC handles
      and manage shared features.
  - HAL ADC configuration steps:
    1. Configure system
    2. Initialize HAL ADC handle using HAL_ADC_Init()
    3. Case of multiple ADC instances used: Link HAL ADC handles using HAL_ADC_SetLinkNextHandle()
       (for more details, refer to function description).
    4. Configure ADC subblocks using functions HAL_ADC_[INJ|REG|MM]_SetConfig{Features}()

- ADC operation
  - Activation and deactivation
    - ADC peripheral requires a specific procedure for activation (internal analog circuitry control, delay for
      stabilization time).
      Note: From activation step, ADC internal analog hardware is enabled, inducing current consumption.
            Therefore, after ADC usage, ADC must be deactivated for power optimization.
  - Calibration
    - ADC conversion accuracy can be improved by running a self calibration.
  - ADC conversions management
    - Conversions can be performed using three programming models:
      - Polling mode (blocking): using HAL_ADC_[INJ|REG|MM]_StartConv(), HAL_ADC_[INJ|REG|MM]_PollForConv()
      - Interrupt mode: using HAL_ADC_[INJ|REG|MM]_StartConv_IT(), HAL_ADC_IRQHandler_[INJ|REG|AWD]()
        and callback functions
      - Data transfer by DMA mode: using HAL_ADC_[INJ|REG|MM]_StartConv_DMA()
      - Note: IT and DMA mode can be used with optional interruptions (analog watchdog, ...) using functions
              HAL_ADC_[INJ|REG|MM]_StartConv_IT_Opt(), HAL_ADC_[INJ|REG|MM]_StartConv_DMA_Opt()
  - HAL ADC operation steps:
    1. Activate ADC using HAL_ADC_Start()
    2. Perform ADC calibration using HAL_ADC_Calibrate()
    3. Start ADC conversions using functions HAL_ADC_[INJ|REG|MM]_StartConv...() (refer to list above)
    4. Process conversion data using HAL_ADC_[INJ|REG|MM]_ReadConversionData(), IRQ handler and callback functions,
       DMA buffers
    5. Stop ADC conversions using functions HAL_ADC_[INJ|REG|MM]_StopConv...()
       Note: With trigger SW start, conversions iterations without conversion stop operation is possible using function
             HAL_ADC_[INJ|REG|MM]_TrigNextConv().
    6. Deactivate ADC using HAL_ADC_Stop()

## Callback registration

When the compilation flag USE_HAL_ADC_REGISTER_CALLBACKS is set to 1,
functions HAL_ADC_Register...Callback() allow to register following callbacks:
  - @ref HAL_ADC_ErrorCallback() : ADC error callback
  - @ref HAL_ADC_REG_EndOfSamplingCallback() : ADC group regular end of sampling phase callback
  - @ref HAL_ADC_REG_UnitaryConvCpltCallback() : ADC group regular end of unitary conversion callback
  - @ref HAL_ADC_REG_SequenceConvCpltCallback() : ADC group regular end of sequence conversions callback
  - @ref HAL_ADC_REG_DataTransferHalfCallback() : ADC group regular conversion data buffer half transfer callback
    (under condition of USE_HAL_ADC_DMA activated)
  - @ref HAL_ADC_REG_DataTransferCpltCallback() : ADC group regular conversion data buffer transfer complete callback
    (under condition of USE_HAL_ADC_DMA activated)
  - @ref HAL_ADC_REG_DataTransferStopCallback() : ADC group regular conversion data transfer abort callback
    (under condition of USE_HAL_ADC_DMA activated)
  - @ref HAL_ADC_INJ_UnitaryConvCpltCallback() : ADC group injected end of unitary conversion callback
  - @ref HAL_ADC_INJ_SequenceConvCpltCallback() : ADC group injected end of sequence conversions callback
  - @ref HAL_ADC_AnalogWD_OutOfWindowCallback() : ADC analog watchdog out of window event callback

When the compilation flag USE_HAL_ADC_REGISTER_CALLBACKS is set to 0 or not defined,
the callback registration feature is not available and all callbacks are set to the corresponding weak functions.

  */

/**
  * @}
  */

/** @defgroup ADC_Configuration_Table ADC Configuration Table
  * @{

## Configuration inside the ADC driver

Config defines                  | Description           | Default value | Note
------------------------------- | --------------------- | ------------- | --------------------------------------------
USE_HAL_ADC_MODULE              | from hal_conf.h       | 1  | When set, HAL ADC module is enabled
USE_HAL_ADC_DMA                 | from hal_conf.h       | 1  | Enable DMA code inside ADC
USE_HAL_ADC_REGISTER_CALLBACKS  | from hal_conf.h       | 0  | When defined, enable the register callbacks assert
USE_HAL_ADC_CLK_ENABLE_MODEL    | from hal_conf.h       | HAL_CLK_ENABLE_NO | Enable gating of the peripheral clock
USE_HAL_CHECK_PARAM             | from hal_conf.h       | 0  | Parameters (pointers or sizes) are checked in runtime
USE_HAL_CHECK_PROCESS_STATE     | from hal_conf.h       | 0  | When set, enable atomic access to process state check
USE_ASSERT_DBG_PARAM            | from PreProcessor env | None | When defined, enable the params assert
USE_ASSERT_DBG_STATE            | from PreProcessor env | None | When defined, enable the state assert
ADC_INST_IN_COMMON_COUNT        | from CMSIS | None | When defined and value > 2, multiple ADC handles can be linked
ADC_MULTIMODE_SUPPORT           | from CMSIS | None | When defined, multimode features available
  */
/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup ADC_Private_Constants ADC Private Constants
  * @{
  */

/* Definition of HAL ADC handle table "group_state" indexes */
#define ADC_GROUP_REGULAR               ((uint8_t)HAL_ADC_GROUP_REGULAR - 1U)  /*!< ADC group regular index
                                        in HAL ADC handle table "group_state" */
#define ADC_GROUP_INJECTED              ((uint8_t)HAL_ADC_GROUP_INJECTED - 1U) /*!< ADC group injected index
                                        in HAL ADC handle table "group_state" */

#if defined(ADC_MULTIMODE_SUPPORT)
/* Definition of HAL ADC handle table "common" indexes
   Multimode ADC instances on this STM32 series: ADC1 and ADC2. */
#define ADC_MM_INDEX_MASTER             (0U) /*!< Multimode ADC instance master index
                                        in HAL ADC handle table "common" */
#define ADC_MM_INDEX_SLAVE              (1U) /*!< Multimode ADC instance slave index
                                        in HAL ADC handle table "common" */
#define ADC_MM_INST_COUNT               (2U) /*!< Multimode ADC instances count */
#endif /* ADC_MULTIMODE_SUPPORT */

#define ADC_GAIN_COMPENSATION_VAL_UNIT  (1000UL) /*!< HAL ADC gain compensation value corresponding to unitary gain */

/* Delay values for ADC operation */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32U) /*!< Delay between ADC end of
                                        calibration and ADC enable. Delay estimation in CPU cycles: Case of ADC enable
                                        done immediately after ADC calibration, ADC clock setting slow (presc 32). */

/* Timeout values for ADC operations (enable settling time,                   */
/*   disable settling time, ...).                                             */
/*   Values defined to be higher than worst cases: low clock frequency,       */
/*   maximum prescalers.                                                      */
#define ADC_ENABLE_TIMEOUT_MS           (2UL) /*!< ADC enable timeout value (unit: milliseconds) */
#define ADC_DISABLE_TIMEOUT_MS          (2UL) /*!< ADC disable timeout value (unit: milliseconds) */
#define ADC_CONV_STOP_TIMEOUT_MS        (2UL) /*!< ADC conversion stop timeout value (unit: milliseconds) */

/* Timeout value for ADC calibration.                                         */
/* Refer to datasheet params "tCAL" and "fADC":                               */
/* Calibration time typ = tCAL / fADC                                         */
/* Calibration time max = tCAL max / fADC min = 31849 / 0.14 = 228ms          */
#define ADC_CALIBRATION_TIMEOUT_MS      (228UL) /*!< ADC calibration timeout value (unit: milliseconds) */

/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup ADC_Private_Macros ADC Private Macros
  * @{
  */

/*! Get ADC instance from the selected HAL ADC handle */
#define ADC_GET_INSTANCE(hadc) ((ADC_TypeDef *)((uint32_t)(hadc)->instance))

/*! Convert HAL ADC group (hal_adc_group_t) to group state (hal_adc_group_state_t) */
#define ADC_GROUP_TO_STATE(group) ((hal_adc_group_state_t)(((uint32_t)group) - 1U))

/**
  * @brief Wait for approximate delay in us.
  * @param delay_us Delay to wait for (unit: us)
  * @note  Compute number of CPU cycles to wait for, using CMSIS global variable "SystemCoreClock".
  * @note  Delay is approximate (depends on compilation optimization).
  * @note  Computation: variable is divided by 2 to compensate partially CPU processing cycles of wait loop
  *        (total shift right of 21 bits, including conversion from frequency in MHz).
  *        If system core clock frequency is below 500kHz, delay is fulfilled by few CPU processing cycles.
  */
#define ADC_DELAY_US(delay_us) \
  do { \
    volatile uint32_t wait_loop_index = (((delay_us) * (SystemCoreClock >> 19U)) >> 2U); \
    while (wait_loop_index != 0U) { \
      wait_loop_index--; \
    } \
  } while(0)

/**
  * @brief IS_ADC macro assert definitions
  * @{
  */
/*! Assert definitions of ADC resolution parameters */
#define IS_ADC_RESOLUTION(resolution) (((resolution) == HAL_ADC_RESOLUTION_12_BIT) \
                                       || ((resolution) == HAL_ADC_RESOLUTION_10_BIT) \
                                       || ((resolution) == HAL_ADC_RESOLUTION_8_BIT) \
                                       || ((resolution) == HAL_ADC_RESOLUTION_6_BIT))

/*! Assert definitions of ADC trigger frequency mode */
#define IS_ADC_TRIGGER_FREQ_MODE(trigger_freq_mode) (((trigger_freq_mode) == HAL_ADC_TRIGGER_FREQ_HIGH) \
                                                     || ((trigger_freq_mode) == HAL_ADC_TRIGGER_FREQ_LOW))

/*! Assert definitions of ADC sampling mode */
#define IS_ADC_SAMPLING_MODE(sampling_mode) (((sampling_mode) == HAL_ADC_SAMPLING_MODE_NORMAL) \
                                             || ((sampling_mode) == HAL_ADC_SAMPLING_MODE_BULB) \
                                             || ((sampling_mode) == HAL_ADC_SAMPLING_MODE_TRIGGER_CTRL))

/*! Assert definitions of ADC group */
#define IS_ADC_GROUP(group) (((group) == HAL_ADC_GROUP_REGULAR) \
                             || ((group) == HAL_ADC_GROUP_INJECTED) \
                             || ((group) == HAL_ADC_GROUP_REGULAR_INJECTED) \
                             || ((group) == HAL_ADC_GROUP_NONE))

/*! Assert definitions of ADC group regular conversion trigger source (ADC instance specific) */
#if defined(STM32C591xx) || defined(STM32C593xx) || defined(STM32C5A3xx)
#define IS_ADC_REG_TRIGGER_SRC_ADC123(trigger_src)  (((trigger_src) == HAL_ADC_REG_TRIG_SOFTWARE) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_EXTI11) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC1) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC3) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_TRGO2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM2_OC2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM2_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM3_OC4) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM3_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM4_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM4_OC4) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM5_OC4) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM5_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM6_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM8_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM8_TRGO2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM15_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_LPTIM1_OC1))
#else
#define IS_ADC_REG_TRIGGER_SRC_ADC123(trigger_src)  (((trigger_src) == HAL_ADC_REG_TRIG_SOFTWARE) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_EXTI11) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC1) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_OC3) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM1_TRGO2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM2_OC2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM2_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM5_OC4) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM5_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM6_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM8_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM8_TRGO2) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_TIM15_TRGO) \
                                                     || ((trigger_src) == HAL_ADC_REG_TRIG_LPTIM1_OC1))
#endif /* STM32C591xx, STM32C593xx, STM32C5A3xx */

/*! Assert definitions of ADC group regular conversion trigger source */
#define IS_ADC_REG_TRIGGER_SRC(instance, trigger_src) (IS_ADC_REG_TRIGGER_SRC_ADC123(trigger_src))


/*! Assert definitions of ADC group regular conversion trigger edge */
#define IS_ADC_REG_TRIGGER_EDGE(trigger_edge) (((trigger_edge) == HAL_ADC_REG_TRIG_EDGE_NONE) \
                                               || ((trigger_edge) == HAL_ADC_REG_TRIG_EDGE_RISING) \
                                               || ((trigger_edge) == HAL_ADC_REG_TRIG_EDGE_FALLING) \
                                               || ((trigger_edge) == HAL_ADC_REG_TRIG_EDGE_RISING_FALLING))

/*! Assert definitions of ADC group regular sequencer length */
#define IS_ADC_REG_SEQUENCER_LENGTH(sequencer_length) ((1U <= (sequencer_length)) && ((sequencer_length) <= 16U))

/*! Assert definitions of ADC group regular sequencer discontinuous mode */
#define IS_ADC_REG_SEQ_DISCONT(sequencer_discont) (((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_DISABLE) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_1RANK) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_2RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_3RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_4RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_5RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_6RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_7RANKS) \
                                                   || ((sequencer_discont) == HAL_ADC_REG_SEQ_DISCONT_8RANKS))

/*! Assert definitions of ADC group regular continuous mode */
#define IS_ADC_REG_CONTINUOUS_MODE(continuous) (((continuous) == HAL_ADC_REG_CONV_SINGLE) \
                                                || ((continuous) == HAL_ADC_REG_CONV_CONTINUOUS))

/*! Assert definitions of ADC group regular overrun mode */
#define IS_ADC_REG_OVERRUN_MODE(overrun) (((overrun) == HAL_ADC_REG_OVR_DATA_PRESERVED) \
                                          || ((overrun) == HAL_ADC_REG_OVR_DATA_OVERWRITTEN))

/*! Assert definitions of ADC group injected conversion trigger source (ADC instance specific) */
#if defined(STM32C591xx) || defined(STM32C593xx) || defined(STM32C5A3xx)
#define IS_ADC_INJ_TRIGGER_SRC_ADC12(trigger_src)  (((trigger_src) == HAL_ADC_INJ_TRIG_SOFTWARE) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_EXTI15) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_OC4) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_TRGO2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM2_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM2_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM3_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM3_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM3_OC3) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM4_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC3) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM7_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_OC4) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM12_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM15_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_LPTIM1_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_FROM_REGULAR))
#else
#define IS_ADC_INJ_TRIGGER_SRC_ADC12(trigger_src)  (((trigger_src) == HAL_ADC_INJ_TRIG_SOFTWARE) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_EXTI15) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_OC4) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM1_TRGO2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM2_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM2_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_OC3) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM5_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM7_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_OC4) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO2) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM8_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM12_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_TIM15_TRGO) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_LPTIM1_OC1) \
                                                    || ((trigger_src) == HAL_ADC_INJ_TRIG_FROM_REGULAR))
#endif /* STM32C591xx, STM32C593xx, STM32C5A3xx */

/*! Assert definitions of ADC group injected conversion trigger source */
#define IS_ADC_INJ_TRIGGER_SRC(instance, trigger_src) (IS_ADC_INJ_TRIGGER_SRC_ADC12(trigger_src))

/*! Assert definitions of ADC group injected conversion trigger edge */
#define IS_ADC_INJ_TRIGGER_EDGE(trigger_edge) (((trigger_edge) == HAL_ADC_INJ_TRIG_EDGE_NONE) \
                                               || ((trigger_edge) == HAL_ADC_INJ_TRIG_EDGE_RISING) \
                                               || ((trigger_edge) == HAL_ADC_INJ_TRIG_EDGE_FALLING) \
                                               || ((trigger_edge) == HAL_ADC_INJ_TRIG_EDGE_RISING_FALLING))

/*! Assert definitions of ADC group injected sequencer length */
#define IS_ADC_INJ_SEQUENCER_LENGTH(sequencer_length) ((1U <= (sequencer_length)) && ((sequencer_length) <= 4U))

/*! Assert definitions of ADC group injected sequencer discontinuous mode */
#define IS_ADC_INJ_SEQ_DISCONT(sequencer_discont) (((sequencer_discont) == HAL_ADC_INJ_SEQ_DISCONT_DISABLE) \
                                                   || ((sequencer_discont) == HAL_ADC_INJ_SEQ_DISCONT_1RANK))

/*! Assert definitions of ADC channel (specific ADC1) */
#define IS_ADC_CHANNEL_ADC1(channel) (((channel) == HAL_ADC_CHANNEL_0) \
                                      || ((channel) == HAL_ADC_CHANNEL_1) \
                                      || ((channel) == HAL_ADC_CHANNEL_2) \
                                      || ((channel) == HAL_ADC_CHANNEL_3) \
                                      || ((channel) == HAL_ADC_CHANNEL_4) \
                                      || ((channel) == HAL_ADC_CHANNEL_5) \
                                      || ((channel) == HAL_ADC_CHANNEL_6) \
                                      || ((channel) == HAL_ADC_CHANNEL_7) \
                                      || ((channel) == HAL_ADC_CHANNEL_8) \
                                      || ((channel) == HAL_ADC_CHANNEL_9) \
                                      || ((channel) == HAL_ADC_CHANNEL_10) \
                                      || ((channel) == HAL_ADC_CHANNEL_11) \
                                      || ((channel) == HAL_ADC_CHANNEL_12) \
                                      || ((channel) == HAL_ADC_CHANNEL_13) \
                                      || ((channel) == HAL_ADC_CHANNEL_VREFINT) \
                                      || ((channel) == HAL_ADC_CHANNEL_TEMPSENSOR) \
                                      || ((channel) == HAL_ADC_CHANNEL_NONE) \
                                      || ((channel) == HAL_ADC_CHANNEL_ALL))

#if defined(ADC2)
/*! Assert definitions of ADC channel (specific ADC2) */
#define IS_ADC_CHANNEL_ADC2(channel) (((channel) == HAL_ADC_CHANNEL_0) \
                                      || ((channel) == HAL_ADC_CHANNEL_1) \
                                      || ((channel) == HAL_ADC_CHANNEL_2) \
                                      || ((channel) == HAL_ADC_CHANNEL_3) \
                                      || ((channel) == HAL_ADC_CHANNEL_4) \
                                      || ((channel) == HAL_ADC_CHANNEL_5) \
                                      || ((channel) == HAL_ADC_CHANNEL_6) \
                                      || ((channel) == HAL_ADC_CHANNEL_7) \
                                      || ((channel) == HAL_ADC_CHANNEL_8) \
                                      || ((channel) == HAL_ADC_CHANNEL_9) \
                                      || ((channel) == HAL_ADC_CHANNEL_10) \
                                      || ((channel) == HAL_ADC_CHANNEL_11) \
                                      || ((channel) == HAL_ADC_CHANNEL_12) \
                                      || ((channel) == HAL_ADC_CHANNEL_13) \
                                      || ((channel) == HAL_ADC_CHANNEL_NONE) \
                                      || ((channel) == HAL_ADC_CHANNEL_ALL))
#endif /* ADC2 */

#if defined(ADC3)
/*! Assert definitions of ADC channel (specific ADC3) */
#define IS_ADC_CHANNEL_ADC3(channel) (((channel) == HAL_ADC_CHANNEL_0) \
                                      || ((channel) == HAL_ADC_CHANNEL_1) \
                                      || ((channel) == HAL_ADC_CHANNEL_2) \
                                      || ((channel) == HAL_ADC_CHANNEL_3) \
                                      || ((channel) == HAL_ADC_CHANNEL_4) \
                                      || ((channel) == HAL_ADC_CHANNEL_5) \
                                      || ((channel) == HAL_ADC_CHANNEL_6) \
                                      || ((channel) == HAL_ADC_CHANNEL_7) \
                                      || ((channel) == HAL_ADC_CHANNEL_8) \
                                      || ((channel) == HAL_ADC_CHANNEL_9) \
                                      || ((channel) == HAL_ADC_CHANNEL_10) \
                                      || ((channel) == HAL_ADC_CHANNEL_11) \
                                      || ((channel) == HAL_ADC_CHANNEL_12) \
                                      || ((channel) == HAL_ADC_CHANNEL_13) \
                                      || ((channel) == HAL_ADC_CHANNEL_NONE) \
                                      || ((channel) == HAL_ADC_CHANNEL_ALL))
#endif /* ADC3 */

/*! Assert definitions of ADC channel */
#if defined(ADC3)
#define IS_ADC_CHANNEL(instance, channel)  (((instance) == HAL_ADC1) ? IS_ADC_CHANNEL_ADC1(channel) : \
                                            ((instance) == HAL_ADC2) ? IS_ADC_CHANNEL_ADC2(channel) : \
                                            IS_ADC_CHANNEL_ADC3(channel))
#elif defined(ADC2)
#define IS_ADC_CHANNEL(instance, channel) (((instance) == HAL_ADC2) ? \
                                           IS_ADC_CHANNEL_ADC2(channel) :\
                                           IS_ADC_CHANNEL_ADC1(channel))
#else
#define IS_ADC_CHANNEL(instance, channel) (IS_ADC_CHANNEL_ADC1(channel))
#endif /* ADC2 */

/*! Assert definitions of ADC sampling time */
#define IS_ADC_SAMPLING_TIME(sampling_time) (((sampling_time) ==  HAL_ADC_SAMPLING_TIME_3CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_5CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_8CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_13CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_25CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_48CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_139CYCLES) \
                                             || ((sampling_time) ==  HAL_ADC_SAMPLING_TIME_289CYCLES))

/*! Assert definitions of ADC channel ending mode */
#define IS_ADC_CHANNEL_INPUT_MODE(input_mode) ((input_mode) == HAL_ADC_IN_SINGLE_ENDED)


/*! Assert definitions of ADC multimode mode */
#define IS_ADC_MM_MODE(mode) (((mode) == HAL_ADC_MM_INDEPENDENT) \
                              || ((mode) == HAL_ADC_MM_DUAL_REG_SIMULT) \
                              || ((mode) == HAL_ADC_MM_DUAL_REG_INTERL) \
                              || ((mode) == HAL_ADC_MM_DUAL_INJ_SIMULT) \
                              || ((mode) == HAL_ADC_MM_DUAL_INJ_ALTERN) \
                              || ((mode) == HAL_ADC_MM_DUAL_REG_SIM_INJ_SIM) \
                              || ((mode) == HAL_ADC_MM_DUAL_REG_SIM_INJ_ALT) \
                              || ((mode) == HAL_ADC_MM_DUAL_REG_INT_INJ_SIM))

/*! Assert definitions of ADC multimode data format */
#define IS_ADC_MM_REG_DATA_FORMAT(reg_data_format) (((reg_data_format) == HAL_ADC_MM_REG_DATA_EACH_ADC) \
                                                    || ((reg_data_format) == HAL_ADC_MM_REG_DATA_PACK_32_BITS) \
                                                    || ((reg_data_format) == HAL_ADC_MM_REG_DATA_PACK_16_BITS))

/*! Assert definitions of ADC multimode data format */
#define IS_ADC_MM_REG_DATA_TRANSFER_PACKING(reg_data_transfer) \
  (((reg_data_transfer) == HAL_ADC_MM_REG_DATA_TRANSFER_PACK) \
   || ((reg_data_transfer) == HAL_ADC_MM_REG_DATA_TRANSFER_UNPACK))

/*! Assert definitions of ADC multimode interleaved delay */
#define IS_ADC_MM_INTERL_DELAY(interl_delay) (((interl_delay) == HAL_ADC_MM_INTERL_DELAY_1CYCLE) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_2CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_3CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_4CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_5CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_6CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_7CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_8CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_9CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_10CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_11CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_12CYCLES) \
                                              || ((interl_delay) == HAL_ADC_MM_INTERL_DELAY_13CYCLES))

/*! Assert definitions of ADC data shift left */
#define IS_ADC_LEFT_BIT_SHIFT(left_bit_shift) (((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_NONE) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_1_BIT) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_2_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_3_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_4_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_5_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_6_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_7_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_8_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_9_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_10_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_11_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_12_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_13_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_14_BITS) \
                                               || ((left_bit_shift) == HAL_ADC_LEFT_BIT_SHIFT_15_BITS))

/*! Assert definitions of ADC gain compensation */
#define IS_ADC_GAIN_COMPENSATION(gain_compensation) ((gain_compensation) <= 3999UL)

/*! Assert definitions of ADC low power mode auto wait */
#define IS_ADC_LP_AUTOWAIT(lp_auto_wait) (((lp_auto_wait) == HAL_ADC_LP_AUTO_WAIT_DISABLE) \
                                          || ((lp_auto_wait) == HAL_ADC_LP_AUTO_WAIT_ENABLE))

/*! Assert definitions of ADC analog watchdog instance */
#define IS_ADC_AWD_INSTANCE(awd_instance) (((awd_instance) == HAL_ADC_AWD_1) \
                                           || ((awd_instance) == HAL_ADC_AWD_2) \
                                           || ((awd_instance) == HAL_ADC_AWD_3))

/*! Assert definitions of ADC analog watchdog threshold selection */
#define IS_ADC_AWD_THRESHOLD_SEL(threshold_sel) (((threshold_sel) == HAL_ADC_AWD_THRESHOLD_HIGH) \
                                                 || ((threshold_sel) == HAL_ADC_AWD_THRESHOLD_LOW))

/*! Assert definitions of ADC analog watchdog threshold value */
#define IS_ADC_AWD_THRESHOLD(threshold) (((threshold) >= (0x00400000L * (-1L))) && ((threshold) <= 0x003FFFFFL))

/*! Assert definitions of ADC analog watchdog filtering */
#define IS_ADC_AWD_FILTERING(filtering) (((filtering) == HAL_ADC_AWD_FILTERING_NONE) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_2SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_3SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_4SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_5SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_6SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_7SAMPLES) \
                                         || ((filtering) == HAL_ADC_AWD_FILTERING_8SAMPLES))

/*! Assert definitions of ADC oversampling instance */
#define IS_ADC_OVS_INSTANCE(ovs_instance) ((ovs_instance) == HAL_ADC_OVS_1)

/*! Assert definitions of ADC oversampling scope */
#define IS_ADC_OVS_SCOPE(scope) (((scope) == HAL_ADC_OVS_DISABLE) \
                                 || ((scope) == HAL_ADC_OVS_REG_CONTINUED) \
                                 || ((scope) == HAL_ADC_OVS_REG_RESUMED) \
                                 || ((scope) == HAL_ADC_OVS_INJ) \
                                 || ((scope) == HAL_ADC_OVS_INJ_REG_RESUMED))

/*! Assert definitions of ADC oversampling discontinuous mode */
#define IS_ADC_OVS_DISCONT(discont) (((discont) == HAL_ADC_OVS_CONT) \
                                     || ((discont) == HAL_ADC_OVS_DISCONT))

/*! Assert definitions of ADC oversampling ratio */
#define IS_ADC_OVS_RATIO(ratio) ((1UL <= (ratio)) && ((ratio) <= (1UL << 10U) /* 1024 */))

/*! Assert definitions of ADC oversampling ratio power of 2 */
#define IS_ADC_OVS_RATIO_POW_2(ratio) (((ratio) == (1UL << 0U))     /* 1 */ \
                                       || ((ratio) == (1UL << 1U))  /* 2 */ \
                                       || ((ratio) == (1UL << 2U))  /* 4 */ \
                                       || ((ratio) == (1UL << 3U))  /* 8 */ \
                                       || ((ratio) == (1UL << 4U))  /* 16 */ \
                                       || ((ratio) == (1UL << 5U))  /* 32 */ \
                                       || ((ratio) == (1UL << 6U))  /* 64 */ \
                                       || ((ratio) == (1UL << 7U))  /* 128 */ \
                                       || ((ratio) == (1UL << 8U))  /* 256 */ \
                                       || ((ratio) == (1UL << 9U))  /* 512 */ \
                                       || ((ratio) == (1UL << 10U)))/* 1024 */

/*! Assert definitions of ADC oversampling shift */
#define IS_ADC_OVS_SHIFT(shift) ((shift) <= 11UL)

/*! Assert definitions of ADC offset instance */
#define IS_ADC_OFFSET_INSTANCE(offset_instance) (((offset_instance) == HAL_ADC_OFFSET_1) \
                                                 || ((offset_instance) == HAL_ADC_OFFSET_2) \
                                                 || ((offset_instance) == HAL_ADC_OFFSET_3) \
                                                 || ((offset_instance) == HAL_ADC_OFFSET_4))

/*! Assert definitions of ADC offset level value */
#define IS_ADC_OFFSET_LEVEL(level) (((level) >= (0x003FFFFFL * (-1L))) && ((level) <= 0x003FFFFFL))

/*! Assert definitions of ADC offset signed saturation */
#define IS_ADC_OFFSET_SAT_SIGN(saturation_signed) (((saturation_signed) == HAL_ADC_OFFSET_SAT_SIGNED_DISABLE) \
                                                   || ((saturation_signed) == HAL_ADC_OFFSET_SAT_SIGNED_ENABLE))

/*! Assert definitions of ADC offset unsigned saturation */
#define IS_ADC_OFFSET_SAT_UNSIGN(sat_unsigned) (((sat_unsigned) == HAL_ADC_OFFSET_SAT_UNSIGNED_DISABLE) \
                                                || ((sat_unsigned) == HAL_ADC_OFFSET_SAT_UNSIGNED_ENABLE))

/*! Assert definitions of ADC event */
#define IS_ADC_EVENT(event) (((event) == HAL_ADC_EVENT_EOC) \
                             || ((event) == HAL_ADC_EVENT_EOS) \
                             || ((event) == HAL_ADC_EVENT_OVR) \
                             || ((event) == HAL_ADC_EVENT_EOSMP) \
                             || ((event) == HAL_ADC_EVENT_JEOC) \
                             || ((event) == HAL_ADC_EVENT_JEOS) \
                             || ((event) == HAL_ADC_EVENT_AWD_1) \
                             || ((event) == HAL_ADC_EVENT_AWD_2) \
                             || ((event) == HAL_ADC_EVENT_AWD_3))

/*! Assert configuration of ADC data transfer by DMA in silent mode: DMA must be in circular mode */
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_ADC_DMA_VALID_SILENT_MODE(hadc, interrupts) \
  (((interrupts) != HAL_ADC_OPT_DMA_IT_SILENT) || (hadc->hdma_reg->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR))
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_ADC_DMA */

/*! Assert definitions of ADC optional interruptions for regular conversions */
#define IS_ADC_OPT_IT_REG(event) (((event) & (HAL_ADC_OPT_IT_NONE \
                                              | HAL_ADC_OPT_IT_REG_EOSMP \
                                              | HAL_ADC_OPT_IT_REG_EOC \
                                              | HAL_ADC_OPT_IT_REG_EOS \
                                              | HAL_ADC_OPT_IT_REG_OVR \
                                              | HAL_ADC_OPT_IT_AWD_1 \
                                              | HAL_ADC_OPT_IT_AWD_2 \
                                              | HAL_ADC_OPT_IT_AWD_3)) == (event))

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/*! Assert definitions of ADC optional interruptions for regular conversions with data transfer by DMA */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define IS_ADC_OPT_IT_REG_DMA(event) ((((event) & (HAL_ADC_OPT_IT_NONE \
                                                   | HAL_ADC_OPT_IT_REG_EOSMP \
                                                   | HAL_ADC_OPT_IT_REG_EOC \
                                                   | HAL_ADC_OPT_IT_REG_EOS \
                                                   | HAL_ADC_OPT_IT_REG_OVR \
                                                   | HAL_ADC_OPT_IT_AWD_1 \
                                                   | HAL_ADC_OPT_IT_AWD_2 \
                                                   | HAL_ADC_OPT_IT_AWD_3 \
                                                   | HAL_ADC_OPT_DMA_IT_NONE \
                                                   | HAL_ADC_OPT_DMA_IT_HT \
                                                   | HAL_ADC_OPT_DMA_IT_DEFAULT)) == (event)) \
                                      || ((event) == HAL_ADC_OPT_DMA_IT_SILENT))
#else
#define IS_ADC_OPT_IT_REG_DMA(event) (((event) & (HAL_ADC_OPT_IT_NONE \
                                                  | HAL_ADC_OPT_IT_REG_EOSMP \
                                                  | HAL_ADC_OPT_IT_REG_EOC \
                                                  | HAL_ADC_OPT_IT_REG_EOS \
                                                  | HAL_ADC_OPT_IT_REG_OVR \
                                                  | HAL_ADC_OPT_IT_AWD_1 \
                                                  | HAL_ADC_OPT_IT_AWD_2 \
                                                  | HAL_ADC_OPT_IT_AWD_3 \
                                                  | HAL_ADC_OPT_DMA_IT_NONE \
                                                  | HAL_ADC_OPT_DMA_IT_HT \
                                                  | HAL_ADC_OPT_DMA_IT_DEFAULT)) == (event))
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_ADC_DMA */

/*! Assert definitions of ADC optional interruptions for injected conversions */
#define IS_ADC_OPT_IT_INJ(event) (((event) & (HAL_ADC_OPT_IT_NONE \
                                              | HAL_ADC_OPT_IT_INJ_EOC \
                                              | HAL_ADC_OPT_IT_INJ_EOS \
                                              | HAL_ADC_OPT_IT_AWD_1 \
                                              | HAL_ADC_OPT_IT_AWD_2 \
                                              | HAL_ADC_OPT_IT_AWD_3)) == (event))

/**
  * @}
  */

/**
  * @}
  */

/* Private functions -------------------------------------------------------------------------------------------------*/
/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
static void adc_reg_dma_data_transfer_half_callback(hal_dma_handle_t *hdma);
static void adc_reg_dma_data_transfer_cplt_callback(hal_dma_handle_t *hdma);
static void adc_reg_dma_data_transfer_stop_callback(hal_dma_handle_t *hdma);
#if defined(ADC_MULTIMODE_SUPPORT)
static void adc_mm_reg_dma_data_transfer_stop_callback(hal_dma_handle_t *hdma);
#endif /* ADC_MULTIMODE_SUPPORT */
static void adc_reg_dma_data_transfer_error_callback(hal_dma_handle_t *hdma);
#endif /* USE_HAL_ADC_DMA */

static hal_status_t adc_activate(hal_adc_handle_t *hadc);
static hal_status_t adc_deactivate(hal_adc_handle_t *hadc);
static hal_status_t adc_calibrate(hal_adc_handle_t *hadc);
static hal_status_t adc_reg_stop_conversion(hal_adc_handle_t *hadc);
static hal_status_t adc_inj_stop_conversion(hal_adc_handle_t *hadc);

#if defined(ADC_MULTIMODE_SUPPORT)
static void adc_assert_state_mm_inst(const hal_adc_handle_t *hadc,
                                     const hal_adc_common_state_t common_state_expected,
                                     const hal_adc_state_t instance_state_expected);
static void adc_assert_state_mm_reg(const hal_adc_handle_t *hadc,
                                    const hal_adc_group_state_t group_state_expected);
static void adc_assert_state_mm_inj(const hal_adc_handle_t *hadc,
                                    const hal_adc_group_state_t group_state_expected);
static void adc_mm_set_state_inst(hal_adc_handle_t *hadc,
                                  hal_adc_common_state_t common_state,
                                  hal_adc_state_t instance_state);
static void adc_mm_set_state_inst_reg(hal_adc_handle_t *hadc,
                                      hal_adc_common_state_t common_state,
                                      hal_adc_state_t instance_state,
                                      hal_adc_group_state_t group_state);
static void adc_mm_set_state_inst_inj(hal_adc_handle_t *hadc,
                                      hal_adc_common_state_t common_state,
                                      hal_adc_state_t instance_state,
                                      hal_adc_group_state_t group_state);
static hal_status_t adc_mm_check_set_state_group(hal_adc_handle_t *hadc,
                                                 uint8_t group_index,
                                                 hal_adc_group_state_t group_state_conditional,
                                                 hal_adc_group_state_t group_state_new);
#endif /* ADC_MULTIMODE_SUPPORT */
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup ADC_Exported_Functions
  * @{
  */

/** @addtogroup ADC_Exported_Functions_Group1
  * @{
    Set of functions allowing to initialize and deinitialize the ADCx peripheral:
      + HAL_ADC_Init(): associate HAL ADC handle with selected ADC instance.
      + HAL_ADC_DeInit(): restore the default configuration of the HAL ADC handle.
      + HAL_ADC_SetLinkNextHandle(): Link HAL ADC handles belonging to the same ADC common instance.
      + HAL_ADC_REG_SetDMA(): Link a HAL ADC handle and a HAL DMA handle for conversion data from ADC group regular.
  */

/**
  * @brief  Initialize HAL ADC handle and associate it to the selected ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  instance ADC instance.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    HAL ADC handle has been correctly initialized.
  */
hal_status_t HAL_ADC_Init(hal_adc_handle_t *hadc, hal_adc_t instance)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_ALL_INSTANCE((ADC_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hadc == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->instance = instance;

#if defined(ADC_INST_IN_COMMON_COUNT) && (ADC_INST_IN_COMMON_COUNT > 1)
  hadc->p_link_next_handle = (hal_adc_handle_t *) NULL;
#endif /* ADC_INST_IN_COMMON_COUNT */

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
  hadc->hdma_reg = (hal_dma_handle_t *) NULL;
#if defined(ADC_MULTIMODE_SUPPORT)
  hadc->mm_reg_data_transfer_packing = HAL_ADC_MM_REG_DATA_TRANSFER_PACK;
#endif /* ADC_MULTIMODE_SUPPORT */
#endif /* USE_HAL_ADC_DMA */

#if defined(USE_HAL_ADC_USER_DATA) && (USE_HAL_ADC_USER_DATA == 1)
  hadc->p_user_data = (void *) NULL;
#endif /* USE_HAL_ADC_USER_DATA */

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1U)
  /* Init HAL ADC callback settings */
  hadc->p_error_cb = HAL_ADC_ErrorCallback;
  hadc->p_reg_end_of_sampling_cb = HAL_ADC_REG_EndOfSamplingCallback;
  hadc->p_reg_eoc_cb = HAL_ADC_REG_UnitaryConvCpltCallback;
  hadc->p_reg_eos_cb = HAL_ADC_REG_SequenceConvCpltCallback;
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
  hadc->p_reg_xfer_half_cb = HAL_ADC_REG_DataTransferHalfCallback;
  hadc->p_reg_xfer_cplt_cb = HAL_ADC_REG_DataTransferCpltCallback;
  hadc->p_reg_xfer_stop_cb = HAL_ADC_REG_DataTransferStopCallback;
#endif /* USE_HAL_ADC_DMA */
  hadc->p_inj_eoc_cb = HAL_ADC_INJ_UnitaryConvCpltCallback;
  hadc->p_inj_eos_cb = HAL_ADC_INJ_SequenceConvCpltCallback;
  hadc->p_awd_out_window_cb = HAL_ADC_AnalogWD_OutOfWindowCallback;
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
  hadc->last_error_codes = HAL_ADC_ERROR_NONE;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

#if defined(USE_HAL_ADC_CLK_ENABLE_MODEL) && (USE_HAL_ADC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
#if defined(USE_HAL_ADC_CLK_ENABLE_MODEL) && (USE_HAL_ADC_CLK_ENABLE_MODEL == HAL_CLK_ENABLE_PERIPH_PWR_SYSTEM)
  /* Enable the independent analog power supply */
#endif /* USE_HAL_ADC_CLK_ENABLE_MODEL */

  /* Enable peripheral clock */
#if defined(ADC3)
  if (instance == HAL_ADC3)
  {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC3);
  }
  else
  {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  }
#else
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
#endif /* ADC3 */
#endif /* USE_HAL_ADC_CLK_ENABLE_MODEL */

  /* Initialize HAL ADC state machine */
  hadc->global_state = HAL_ADC_STATE_INIT;
  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_RESET;
  hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_RESET;
  hadc->common_state = HAL_ADC_COMMON_STATE_INDEPT;

  return HAL_OK;
}

/**
  * @brief  Deinitialize the ADC peripheral.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
void HAL_ADC_DeInit(hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_ALL_INSTANCE(ADC_GET_INSTANCE(hadc)));

  /* Stop the current operations */
  if (hadc->global_state == HAL_ADC_STATE_ACTIVE)
  {
    if (hadc->group_state[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_STATE_ACTIVE)
    {
      (void)HAL_ADC_REG_StopConv(hadc);
    }

    if (hadc->group_state[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_STATE_ACTIVE)
    {
      (void)HAL_ADC_INJ_StopConv(hadc);
    }

    (void)HAL_ADC_Stop(hadc);
  }

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
  hadc->hdma_reg = (hal_dma_handle_t *) NULL;
#endif /* USE_HAL_ADC_DMA */

#if defined(USE_HAL_ADC_USER_DATA) && (USE_HAL_ADC_USER_DATA == 1)
  hadc->p_user_data = NULL;
#endif /* USE_HAL_ADC_USER_DATA */

#if defined(ADC_INST_IN_COMMON_COUNT) && (ADC_INST_IN_COMMON_COUNT > 1)
  /* Check whether handle is registered in a handles daisy chain */
  if (hadc->p_link_next_handle != NULL)
  {
    /* Remove handle from daisy chain: Parse handles through links of daisy chain until loop back to previous handle */
    uint32_t index = ADC_MM_INST_COUNT; /* Maximum number of linked handles (to prevent infinite loop in case of
                                           pointer issue. In this case, daisy chain not updated.) */
    hal_adc_handle_t *p_handle_current = hadc;
    while (index != 0U)
    {
      if (p_handle_current->p_link_next_handle == hadc)
      {
#if (ADC_MM_INST_COUNT > 2)
        /* Daisy chain can be left with other handles linked */
        if (p_handle_current == hadc->p_link_next_handle)
        {
          /* Daisy chain remaining without link: other ADC handle no more linked */
          p_handle_current->common_state = HAL_ADC_COMMON_STATE_INDEPT;
          p_handle_current->p_link_next_handle = NULL;
        }
        else
        {
          /* Daisy chain update: Set link from previous handle to next handle */
          p_handle_current->p_link_next_handle = hadc->p_link_next_handle;
        }
#else
        /* Daisy chain remaining without link: other ADC handle no more linked */
        p_handle_current->common_state = HAL_ADC_COMMON_STATE_INDEPT;
        p_handle_current->p_link_next_handle = NULL;
#endif /* ADC_MM_INST_COUNT */

        hadc->p_link_next_handle = NULL;
        break;
      }
      else
      {
        /* Go to next handle */
        ASSERT_DBG_PARAM(p_handle_current->p_link_next_handle != NULL);
        p_handle_current = p_handle_current->p_link_next_handle;
      }
      index--;
    }
  }
#endif /* ADC_INST_IN_COMMON_COUNT */

  hadc->global_state = HAL_ADC_STATE_RESET;
  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_RESET;
  hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_RESET;
  hadc->common_state = HAL_ADC_COMMON_STATE_INDEPT;
}

#if defined(ADC_INST_IN_COMMON_COUNT) && (ADC_INST_IN_COMMON_COUNT > 1)
/**
  * @brief  Link HAL ADC handles belonging to the same ADC common instance.
  * @param  hadc_a Pointer to hal_adc_handle_t structure of an ADC instance
  *                (not yet linked or already linked to a chain)
  * @param  hadc_b Pointer to hal_adc_handle_t structure of another ADC instance sharing the same ADC common instance
  *                (not yet linked: to be added to an existing chain)
  * @note   Requirement: The selected device must have at least 2 ADC instances sharing the same ADC common instance.
  *         Refer to device reference manual or macro "ADC_COMMON_INSTANCE()" returning
  *         ADC common instance from ADC instance.
  * @note   Link can be performed even if not using the common features.
  *         It is recommended to systematically link handles (when compliant instances): this allows functions
  *         to perform all cross instances checks possible, for optimal HAL ADC driver usage.
  * @note   Links are used to access multiple HAL ADC handles (daisy chain: from one to another and circular).
  *         Used by functions configuring parameters impacting multiple ADC instances.
  * @note   A handle can be removed from a chain using function @ref HAL_ADC_DeInit().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetLinkNextHandle(hal_adc_handle_t *hadc_a, hal_adc_handle_t *hadc_b)
{
  hal_status_t status = HAL_OK;

  ASSERT_DBG_PARAM((hadc_a != NULL));
  ASSERT_DBG_PARAM((hadc_b != NULL));

  /* Check whether selected ADC instances are different */
  ASSERT_DBG_PARAM(hadc_a->instance != hadc_b->instance);

  /* Ensure new handle is not already linked */
  ASSERT_DBG_PARAM(hadc_b->p_link_next_handle == NULL);

  /* Check whether selected ADC instances belong to the same ADC common instance */
  ASSERT_DBG_PARAM(ADC_COMMON_INSTANCE(ADC_GET_INSTANCE(hadc_a))
                   == ADC_COMMON_INSTANCE(ADC_GET_INSTANCE(hadc_b)));

  ASSERT_DBG_STATE(hadc_a->common_state,
                   (uint32_t)HAL_ADC_COMMON_STATE_RESET |
                   (uint32_t)HAL_ADC_COMMON_STATE_INDEPT |
                   (uint32_t)HAL_ADC_COMMON_STATE_LINKED);
  ASSERT_DBG_STATE(hadc_b->common_state,
                   (uint32_t)HAL_ADC_COMMON_STATE_RESET |
                   (uint32_t)HAL_ADC_COMMON_STATE_INDEPT);
  ASSERT_DBG_STATE(hadc_a->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                                           | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                                           | (uint32_t)HAL_ADC_STATE_CALIB
                                                           | (uint32_t)HAL_ADC_STATE_IDLE
                                                           | (uint32_t)HAL_ADC_STATE_ACTIVE));
  ASSERT_DBG_STATE(hadc_b->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                                           | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                                           | (uint32_t)HAL_ADC_STATE_CALIB
                                                           | (uint32_t)HAL_ADC_STATE_IDLE
                                                           | (uint32_t)HAL_ADC_STATE_ACTIVE));

  if (hadc_a->p_link_next_handle == NULL)
  {
    /* First link */
    hadc_b->p_link_next_handle = hadc_a;
  }
  else
  {
    /* Additional link */
    hadc_b->p_link_next_handle = hadc_a->p_link_next_handle;
  }
  hadc_a->p_link_next_handle = hadc_b;

  hadc_a->common_state = HAL_ADC_COMMON_STATE_LINKED;
  hadc_b->common_state = HAL_ADC_COMMON_STATE_LINKED;

  return status;
}
#endif /* ADC_INST_IN_COMMON_COUNT */

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief Link a HAL ADC handle and a HAL DMA handle for conversion data from ADC group regular.
  * @param  hadc Pointer to HAL ADC handle.
  * @param  hdma Pointer to HAL DMA handle.
  * @retval HAL_OK    HAL ADC handle has been correctly configured.
  */
hal_status_t HAL_ADC_REG_SetDMA(hal_adc_handle_t *hadc, hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_STATE(hadc->global_state, ((uint32_t)HAL_ADC_STATE_INIT |
                                        (uint32_t)HAL_ADC_STATE_CONFIGURING |
                                        (uint32_t)HAL_ADC_STATE_IDLE));
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR], ((uint32_t)HAL_ADC_GROUP_STATE_RESET |
                                                          (uint32_t)HAL_ADC_GROUP_STATE_IDLE));

  hadc->hdma_reg = hdma;
  hdma->p_parent = hadc;

  return HAL_OK;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group2_1
  * @{
    Set of functions allowing to configure ADCx peripheral (mandatory features):
    Mandatory features for some process functions (at least some of these functions must be used but
    not necessarily all, depending on process functions used).
    @note For necessary configurations functions, refer to state machine diagram
          or "How to use the ADC HAL module driver" section).
  */

/**
  * @brief  Configure ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_config_t structure containing ADC configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfig(hal_adc_handle_t *hadc, const hal_adc_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_RESOLUTION(p_config->resolution));
  ASSERT_DBG_PARAM(IS_ADC_SAMPLING_MODE(p_config->sampling_mode));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_INIT |
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetResolution(p_instance, (uint32_t)p_config->resolution);
  LL_ADC_SetSamplingMode(p_instance, (uint32_t)p_config->sampling_mode);

  if (hadc->global_state == HAL_ADC_STATE_INIT)
  {
    hadc->global_state = HAL_ADC_STATE_CONFIGURING;
  }

  return status;
}

/**
  * @brief  Get the ADC instance configuration.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_config_t structure containing ADC configuration.
  */
void HAL_ADC_GetConfig(const hal_adc_handle_t *hadc, hal_adc_config_t *p_config)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->resolution = (hal_adc_resolution_t)LL_ADC_GetResolution(p_instance);
  p_config->sampling_mode = (hal_adc_sampling_mode_t)LL_ADC_GetSamplingMode(p_instance);
}

/**
  * @brief  Configure ADC group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_reg_config_t structure containing ADC group regular configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_SetConfig(hal_adc_handle_t *hadc, const hal_adc_reg_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t reg_config;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_REG_TRIGGER_SRC(hadc->instance, p_config->trigger_src));
  if (p_config->trigger_src != HAL_ADC_REG_TRIG_SOFTWARE)
  {
    ASSERT_DBG_PARAM(IS_ADC_REG_TRIGGER_EDGE(p_config->trigger_edge));
  }
  ASSERT_DBG_PARAM(IS_ADC_REG_CONTINUOUS_MODE(p_config->continuous));
  ASSERT_DBG_PARAM(IS_ADC_REG_OVERRUN_MODE(p_config->overrun));
  ASSERT_DBG_PARAM(IS_ADC_REG_SEQUENCER_LENGTH(p_config->sequencer_length));
  ASSERT_DBG_PARAM(IS_ADC_REG_SEQ_DISCONT(p_config->sequencer_discont));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Verify configuration compliance versus hardware constraints */
  if (p_config->continuous != HAL_ADC_REG_CONV_SINGLE)
  {
    if (LL_ADC_GetSamplingMode(p_instance) != LL_ADC_SAMPLING_MODE_NORMAL)
    {
      status = HAL_ERROR;
    }
  }

  LL_ADC_REG_SetTriggerSource(p_instance, (uint32_t)p_config->trigger_src);
  if (p_config->trigger_src != HAL_ADC_REG_TRIG_SOFTWARE)
  {
    LL_ADC_REG_SetTriggerEdge(p_instance, (uint32_t)p_config->trigger_edge);
  }

  LL_ADC_REG_SetSequencerLength(p_instance, LL_ADC_DECIMAL_NB_TO_REG_SEQ_LENGTH((uint32_t)p_config->sequencer_length));

  /* Set ADC group regular in a single register write access
     (equivalent to successive calls of configuration functions LL_ADC_REG_Set...()) */
  reg_config = LL_ADC_READ_REG(p_instance, CFGR1);
  reg_config &= ~(ADC_CFGR1_CONT
                  | ADC_CFGR1_DISCEN | ADC_CFGR1_DISCNUM
                  | ADC_CFGR1_DMNGT
                  | ADC_CFGR1_OVRMOD);
  reg_config |= ((uint32_t)p_config->continuous
                 | (uint32_t)p_config->sequencer_discont
                 | (uint32_t)p_config->overrun);
  LL_ADC_WRITE_REG(p_instance, CFGR1, reg_config);

  if ((p_config->trigger_src == HAL_ADC_REG_TRIG_SOFTWARE)
      && (p_config->continuous == HAL_ADC_REG_CONV_SINGLE))
  {
    hadc->group_conv_per_start[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_CONV_UNIT;
  }
  else
  {
    hadc->group_conv_per_start[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_CONV_MULTIPLE;
  }

  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

/**
  * @brief  Get configuration of ADC group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_reg_config_t structure containing ADC group regular configuration.
  */
void HAL_ADC_REG_GetConfig(const hal_adc_handle_t *hadc, hal_adc_reg_config_t *p_config)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->trigger_src = (hal_adc_reg_trig_src_t)LL_ADC_REG_GetTriggerSource(p_instance);

  if (p_config->trigger_src == HAL_ADC_REG_TRIG_SOFTWARE)
  {
    p_config->trigger_edge = HAL_ADC_REG_TRIG_EDGE_NONE;
  }
  else
  {
    p_config->trigger_edge = (hal_adc_reg_trig_edge_t)LL_ADC_REG_GetTriggerEdge(p_instance);
  }

  p_config->sequencer_length = (uint8_t)LL_ADC_REG_SEQ_LENGTH_TO_DECIMAL_NB(LL_ADC_REG_GetSequencerLength(p_instance));
  p_config->sequencer_discont = (hal_adc_reg_seq_discont_length_t)LL_ADC_REG_GetSequencerDiscont(p_instance);
  p_config->continuous = (hal_adc_reg_continuous_mode_t)LL_ADC_REG_GetContinuousMode(p_instance);
  p_config->overrun = (hal_adc_reg_overrun_mode_t)LL_ADC_REG_GetOverrun(p_instance);
}

/**
  * @brief  Configure ADC group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_inj_config_t structure containing ADC group injected configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_SetConfig(hal_adc_handle_t *hadc, const hal_adc_inj_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_INJ_TRIGGER_SRC(hadc->instance, p_config->trigger_src));
  if ((p_config->trigger_src != HAL_ADC_INJ_TRIG_SOFTWARE)
      && (p_config->trigger_src != HAL_ADC_INJ_TRIG_FROM_REGULAR))
  {
    ASSERT_DBG_PARAM(IS_ADC_INJ_TRIGGER_EDGE(p_config->trigger_edge));
  }
  ASSERT_DBG_PARAM(IS_ADC_INJ_SEQUENCER_LENGTH(p_config->sequencer_length));
  ASSERT_DBG_PARAM(IS_ADC_INJ_SEQ_DISCONT(p_config->sequencer_discont));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_INJ_SetTriggerSource(p_instance, (uint32_t)p_config->trigger_src & ~LL_ADC_INJ_TRIG_FROM_REGULAR);

  if (p_config->trigger_src != HAL_ADC_INJ_TRIG_SOFTWARE)
  {
    if (p_config->trigger_src == HAL_ADC_INJ_TRIG_FROM_REGULAR)
    {
      LL_ADC_INJ_SetTrigAuto(p_instance, LL_ADC_INJ_TRIG_FROM_REGULAR);
    }
    else
    {
      LL_ADC_INJ_SetTriggerEdge(p_instance, (uint32_t)p_config->trigger_edge);
    }
  }

  LL_ADC_INJ_SetSequencerLength(p_instance, LL_ADC_DECIMAL_NB_TO_INJ_SEQ_LENGTH((uint32_t)p_config->sequencer_length));
  LL_ADC_INJ_SetSequencerDiscont(p_instance, (uint32_t)p_config->sequencer_discont);

  if (p_config->trigger_src == HAL_ADC_INJ_TRIG_SOFTWARE)
  {
    hadc->group_conv_per_start[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_CONV_UNIT;
  }
  else
  {
    hadc->group_conv_per_start[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_CONV_MULTIPLE;
  }

  hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

/**
  * @brief  Get configuration of ADC group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_inj_config_t structure containing ADC group injected configuration.
  */
void HAL_ADC_INJ_GetConfig(const hal_adc_handle_t *hadc, hal_adc_inj_config_t *p_config)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->trigger_src = (hal_adc_inj_trig_src_t)LL_ADC_INJ_GetTriggerSource(p_instance);

  if (p_config->trigger_src == HAL_ADC_INJ_TRIG_SOFTWARE)
  {
    p_config->trigger_src = (hal_adc_inj_trig_src_t)LL_ADC_INJ_GetTrigAuto(p_instance);

    p_config->trigger_edge = HAL_ADC_INJ_TRIG_EDGE_NONE;
  }
  else
  {
    p_config->trigger_edge = (hal_adc_inj_trig_edge_t)LL_ADC_INJ_GetTriggerEdge(p_instance);
  }

  p_config->sequencer_length = (uint8_t)LL_ADC_INJ_SEQ_LENGTH_TO_DECIMAL_NB(LL_ADC_INJ_GetSequencerLength(p_instance));
  p_config->sequencer_discont = (hal_adc_inj_seq_discont_length_t)LL_ADC_INJ_GetSequencerDiscont(p_instance);
}

/**
  * @brief  Configure the selected ADC channel.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  channel ADC channel
  * @param  p_config Pointer to a hal_adc_channel_config_t structure containing ADC channel configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigChannel(hal_adc_handle_t *hadc, hal_adc_channel_t channel,
                                      const hal_adc_channel_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t sequencer_rank_ll_format;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_CHANNEL(hadc->instance, channel));
  ASSERT_DBG_PARAM((p_config->group == HAL_ADC_GROUP_REGULAR) || (p_config->group == HAL_ADC_GROUP_INJECTED));
  if (p_config->group == HAL_ADC_GROUP_REGULAR)
  {
    ASSERT_DBG_PARAM(IS_ADC_REG_SEQUENCER_LENGTH(p_config->sequencer_rank));
  }
  else
  {
    ASSERT_DBG_PARAM(IS_ADC_INJ_SEQUENCER_LENGTH(p_config->sequencer_rank));
  }
  ASSERT_DBG_PARAM(IS_ADC_SAMPLING_TIME(p_config->sampling_time));
  ASSERT_DBG_PARAM(IS_ADC_CHANNEL_INPUT_MODE(p_config->input_mode));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  if (p_config->group == HAL_ADC_GROUP_REGULAR)
  {
    sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_REG_SEQ_RANK((uint32_t)p_config->sequencer_rank);

    LL_ADC_REG_SetSequencerRanks(p_instance, sequencer_rank_ll_format, (uint32_t)channel);
  }
  else
  {
    sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_INJ_SEQ_RANK((uint32_t)p_config->sequencer_rank);
    LL_ADC_INJ_SetSequencerRanks(p_instance, sequencer_rank_ll_format, (uint32_t)channel);
  }

  LL_ADC_SetChannelSamplingTime(p_instance, (uint32_t)channel, (uint32_t)p_config->sampling_time);
  LL_ADC_SetChannelSingleDiff(p_instance, (uint32_t)channel, (uint32_t)p_config->input_mode);

  if (LL_ADC_IS_CHANNEL_INTERNAL((uint32_t)channel))
  {
    /* Channel internal */
    if (((uint32_t)channel & LL_ADC_COMMON_PATH_INTERNAL_MASK) != 0UL)
    {
      LL_ADC_SetCommonPathInternalChAdd(ADC_COMMON_INSTANCE(p_instance), (uint32_t)channel);
    }
  }
  else
  {
    /* Channel connected to GPIO */
    LL_ADC_SetChannelPreselection(p_instance, (uint32_t)channel);
  }

  hadc->global_state = HAL_ADC_STATE_IDLE;

  return status;
}

/**
  * @brief  Get configuration of the selected ADC channel.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  channel ADC channel
  * @param  p_config Pointer to a hal_adc_channel_config_t structure containing ADC channel configuration.
  * @note   Specific cases:
  *         - in case of channel set in ADC groups regular and injected, returned value of ADC group is
  *           @ref HAL_ADC_GROUP_REGULAR_INJECTED and sequencer rank referring to group regular.
  *         - in case of channel not set in any ADC group sequencer, returned value of ADC group is
  *           @ref HAL_ADC_GROUP_NONE. Value of sequencer rank is not relevant.
  *         - in case of channel set multiple times in a ADC group sequencer, returned value of sequencer rank is the
  *           lowest one.
  */
void HAL_ADC_GetConfigChannel(const hal_adc_handle_t *hadc, hal_adc_channel_t channel,
                              hal_adc_channel_config_t *p_config)
{
  ADC_TypeDef *p_instance;
  uint32_t sequencer_rank_ll_format;
  uint8_t sequencer_length;
  uint8_t index;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_CHANNEL(hadc->instance, channel));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_IDLE |
                   (uint32_t)HAL_ADC_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->group = HAL_ADC_GROUP_NONE;
  p_config->sequencer_rank = 0U; /* Initialization to sequencer rank out of bound */
  sequencer_length = (uint8_t)LL_ADC_REG_SEQ_LENGTH_TO_DECIMAL_NB(LL_ADC_REG_GetSequencerLength(p_instance));

  /* Search channel in ADC group regular sequencer */
  for (index = 1U; index <= sequencer_length; index++)
  {
    sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_REG_SEQ_RANK(index);
    if (LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_REG_GetSequencerRanks(p_instance, sequencer_rank_ll_format))
        == LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)channel))
    {
      p_config->group = HAL_ADC_GROUP_REGULAR;
      p_config->sequencer_rank = index;
      break;
    }
  }

  /* Search channel in ADC group injected sequencer */
  sequencer_length = (uint8_t)LL_ADC_INJ_SEQ_LENGTH_TO_DECIMAL_NB(LL_ADC_INJ_GetSequencerLength(p_instance));
  for (index = 1U; index <= sequencer_length; index++)
  {
    sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_INJ_SEQ_RANK(index);
    if (LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_INJ_GetSequencerRanks(p_instance, sequencer_rank_ll_format))
        == LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)channel))
    {
      if (p_config->group == HAL_ADC_GROUP_REGULAR)
      {
        p_config->group = HAL_ADC_GROUP_REGULAR_INJECTED;
        /* Note: in this case, field "sequencer_rank" kept referring to group regular */
      }
      else
      {
        p_config->group = HAL_ADC_GROUP_INJECTED;
        p_config->sequencer_rank = index;
      }
      break;
    }
  }

  p_config->input_mode = (hal_adc_in_mode_t)LL_ADC_GetChannelSingleDiff(p_instance, (uint32_t)channel);
  p_config->sampling_time = (hal_adc_sampling_time_t)LL_ADC_GetChannelSamplingTime(p_instance, (uint32_t)channel);
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Configure ADC multimode.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  p_config Pointer to a hal_adc_mm_config_t structure containing ADC multimode configuration.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle().
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_SetConfig(hal_adc_handle_t *hadc, const hal_adc_mm_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  ADC_Common_TypeDef *p_common_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_MM_MODE(p_config->mode));
  if (p_config->mode != HAL_ADC_MM_INDEPENDENT)
  {
    if ((p_config->mode != HAL_ADC_MM_DUAL_INJ_SIMULT)
        && (p_config->mode != HAL_ADC_MM_DUAL_INJ_ALTERN))
    {
      ASSERT_DBG_PARAM(IS_ADC_MM_REG_DATA_FORMAT(p_config->reg_data_format));

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
      if (p_config->reg_data_format != HAL_ADC_MM_REG_DATA_EACH_ADC)
      {
        ASSERT_DBG_PARAM(IS_ADC_MM_REG_DATA_TRANSFER_PACKING(p_config->reg_data_transfer_packing));
      }
#endif /* USE_HAL_ADC_DMA */
    }

    if ((p_config->mode == HAL_ADC_MM_DUAL_REG_INTERL)
        || (p_config->mode == HAL_ADC_MM_DUAL_REG_INT_INJ_SIM))
    {
      ASSERT_DBG_PARAM(IS_ADC_MM_INTERL_DELAY(p_config->interl_delay));
    }
  }

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc,
                           (hal_adc_common_state_t)((uint32_t)HAL_ADC_COMMON_STATE_LINKED |
                                                    (uint32_t)HAL_ADC_COMMON_STATE_MM),
                           HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);
  p_common_instance = ADC_COMMON_INSTANCE(p_instance);

  /* Variable can be unused depending on assert param and ADC_COMMON_INSTANCE() macro content */
  STM32_UNUSED(p_instance);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance) == p_instance);

#if defined(USE_ASSERT_DBG_PARAM)
  /* Check constraints for mode interleaved */
  if ((p_config->mode == HAL_ADC_MM_DUAL_REG_INTERL)
      || (p_config->mode == HAL_ADC_MM_DUAL_REG_INT_INJ_SIM))
  {
    /* Interleaved delay dependency to ADC resolution */
    uint32_t adc_resolution = LL_ADC_GetResolution(p_instance);
    if (adc_resolution == LL_ADC_RESOLUTION_10B)
    {
      ASSERT_DBG_PARAM(p_config->interl_delay <= HAL_ADC_MM_INTERL_DELAY_11CYCLES);
    }
    else if (adc_resolution == LL_ADC_RESOLUTION_8B)
    {
      ASSERT_DBG_PARAM(p_config->interl_delay <= HAL_ADC_MM_INTERL_DELAY_9CYCLES);
    }
    else if (adc_resolution == LL_ADC_RESOLUTION_6B)
    {
      ASSERT_DBG_PARAM(p_config->interl_delay <= HAL_ADC_MM_INTERL_DELAY_7CYCLES);
    }
    else
    {
      /* No action */
    }
  }
#endif /* USE_ASSERT_DBG_PARAM */

  LL_ADC_SetMultimode(p_common_instance, (uint32_t)p_config->mode);

  if (p_config->mode != HAL_ADC_MM_INDEPENDENT)
  {
    if ((p_config->mode != HAL_ADC_MM_DUAL_INJ_SIMULT)
        && (p_config->mode != HAL_ADC_MM_DUAL_INJ_ALTERN))
    {
      LL_ADC_SetMultiDMATransfer(p_common_instance, (uint32_t)p_config->reg_data_format);

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
      if (p_config->reg_data_format != HAL_ADC_MM_REG_DATA_EACH_ADC)
      {
        hadc->mm_reg_data_transfer_packing = p_config->reg_data_transfer_packing;
      }
#endif /* USE_HAL_ADC_DMA */
    }

    if ((p_config->mode == HAL_ADC_MM_DUAL_REG_INTERL)
        || (p_config->mode == HAL_ADC_MM_DUAL_REG_INT_INJ_SIM))
    {
      LL_ADC_SetMultiTwoSamplingDelay(p_common_instance, (uint32_t)p_config->interl_delay);
    }
  }

  /* Update state of all HAL ADC handles part of multimode */
  adc_mm_set_state_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_IDLE);

  return status;
}

/**
  * @brief  Get configuration of ADC multimode.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  p_config Pointer to a hal_adc_mm_config_t structure containing ADC multimode configuration.
  */
void HAL_ADC_MM_GetConfig(const hal_adc_handle_t *hadc, hal_adc_mm_config_t *p_config)
{
  ADC_Common_TypeDef *p_common_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(ADC_GET_INSTANCE(hadc)) == ADC_GET_INSTANCE(hadc));

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc,
                           (hal_adc_common_state_t)((uint32_t)HAL_ADC_COMMON_STATE_LINKED |
                                                    (uint32_t)HAL_ADC_COMMON_STATE_MM),
                           (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_IDLE
                                             | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_common_instance = ADC_COMMON_INSTANCE(ADC_GET_INSTANCE(hadc));

  p_config->mode = (hal_adc_mm_mode_t)LL_ADC_GetMultimode(p_common_instance);
  p_config->reg_data_format = (hal_adc_mm_reg_data_format_t)LL_ADC_GetMultiDMATransfer(p_common_instance);
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
  p_config->reg_data_transfer_packing = hadc->mm_reg_data_transfer_packing;
#endif /* USE_HAL_ADC_DMA */
  p_config->interl_delay = (hal_adc_mm_interl_delay_t)LL_ADC_GetMultiTwoSamplingDelay(p_common_instance);
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Multimode configuration: conversion data of all ADC instances part of multimode are
  *         transferred using multiple DMA channels (one DMA channel assigned to each ADC).
  *         This function must be called by each HAL ADC handle part of multimode.
  *         Then, multimode conversion is started by HAL_ADC_MM_REG_StartConvM_DMA() / M_DMA_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_data     Pointer to the data buffer (data transfer from ADC to buffer, through DMA).
  * @param  size_byte  Data buffer size (in bytes).
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_SetMultiDMA(hal_adc_handle_t *hadc, uint8_t *p_data, uint32_t size_byte)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;
  hal_dma_handle_t *hdma;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(hadc->hdma_reg != NULL); /* Pointer set by HAL_ADC_REG_SetDMA() */
  ASSERT_DBG_PARAM(p_data != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hadc->hdma_reg == NULL) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check state of all HAL ADC handles part of multimode */
  ASSERT_DBG_STATE(hadc->common_state, HAL_ADC_COMMON_STATE_MM);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);
  hdma = hadc->hdma_reg;

  /* Set DMA channel callback functions pointers */
  hdma->p_xfer_error_cb     = adc_reg_dma_data_transfer_error_callback;
  hdma->p_xfer_halfcplt_cb  = adc_reg_dma_data_transfer_half_callback;
  hdma->p_xfer_cplt_cb      = adc_reg_dma_data_transfer_cplt_callback;

  /* Start DMA transfer in IT mode */
  /* Note: DMA transfer interruptions selection is updated by HAL_ADC_MM_REG_StartConvM_DMA() / DMA_Opt() */
  status = HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                          LL_ADC_DMA_GetRegAddr(p_instance, LL_ADC_DMA_REG_REGULAR_DATA),
                                          (uint32_t)p_data, size_byte, HAL_ADC_OPT_DMA_IT_NONE);

#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
  if (status != HAL_OK)
  {
    hadc->last_error_codes |= HAL_ADC_REG_ERROR_DMA;
  }
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

  /* ADC DMA requests come from each ADC instance */
  LL_ADC_SetMultiDMATransfer(ADC_COMMON_INSTANCE(p_instance), LL_ADC_MULTI_REG_DMA_EACH_ADC);

  return status;
}
#endif /* USE_HAL_ADC_DMA */

#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group2_2
  * @{
    Set of functions allowing to configure ADCx peripheral (optional features):
    Same as other configuration functions for optional features: the non usage of these functions will not block
    any process function.
  */

/**
  * @brief  Configure ADC instance advanced features: conversion data post processing.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_post_processing_config_t structure containing ADC configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigPostProcessing(hal_adc_handle_t *hadc, const hal_adc_post_processing_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_LEFT_BIT_SHIFT(p_config->left_bit_shift));
  ASSERT_DBG_PARAM(IS_ADC_GAIN_COMPENSATION(p_config->gain_compensation_x1000));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetLeftBitShift(p_instance, (uint32_t)p_config->left_bit_shift);

  /* Gain coefficient computation: convert from HAL ADC gain in decimal value to ADC peripheral digital value */
  LL_ADC_SetGainCompensation(p_instance,
                             (uint32_t)((p_config->gain_compensation_x1000 * LL_ADC_GAIN_COMPENSATION_DIV)
                                        / ADC_GAIN_COMPENSATION_VAL_UNIT));

  return status;
}

/**
  * @brief  Get the ADC instance configuration for advanced features: conversion data post processing.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_post_processing_config_t structure containing ADC configuration.
  * @note   Returned value of hal_adc_post_processing_config_t field "gain_compensation_x1000"
  *         can differ from value set in HAL_ADC_SetConfigPostProcessing() due to computation rounding.
  */
void HAL_ADC_GetConfigPostProcessing(const hal_adc_handle_t *hadc, hal_adc_post_processing_config_t *p_config)
{
  ADC_TypeDef *p_instance;
  uint32_t gain_compensation;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->left_bit_shift = (hal_adc_left_bit_shift_t)LL_ADC_GetLeftBitShift(p_instance);

  gain_compensation = LL_ADC_GetGainCompensation(p_instance);
  if (gain_compensation == LL_ADC_GAIN_COMPENSATION_DIV)
  {
    p_config->gain_compensation_x1000 = ADC_GAIN_COMPENSATION_VAL_UNIT;
  }
  else
  {
    /* Gain coefficient computation: convert from ADC peripheral digital value to HAL ADC gain in decimal value */
    p_config->gain_compensation_x1000 = ((gain_compensation * ADC_GAIN_COMPENSATION_VAL_UNIT)
                                         / LL_ADC_GAIN_COMPENSATION_DIV);
    /* Update value to compensate computation rounding */
    if (gain_compensation != 0UL)
    {
      p_config->gain_compensation_x1000 += 1UL;
    }
  }
}

/**
  * @brief  Configure ADC instance advanced features: low power.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_low_power_config_t structure containing ADC configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigLowPower(hal_adc_handle_t *hadc, const hal_adc_low_power_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_LP_AUTOWAIT(p_config->lp_auto_wait));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetLowPowerMode(p_instance, (uint32_t)p_config->lp_auto_wait);

  return status;
}

/**
  * @brief  Get the ADC instance configuration for advanced features: low power.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_config Pointer to a hal_adc_low_power_config_t structure containing ADC configuration.
  */
void HAL_ADC_GetConfigLowPower(const hal_adc_handle_t *hadc, hal_adc_low_power_config_t *p_config)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->lp_auto_wait = (hal_adc_lp_auto_wait_state_t)LL_ADC_GetLowPowerMode(p_instance);
}


/**
  * @brief  Configure ADC analog watchdog.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @param  p_config Pointer to a hal_adc_awd_config_t structure containing ADC analog watchdog configuration.
  * @note   Specific configurations:
  *         - to monitor all channels, use following parameters values of hal_adc_awd_config_t:
  *             .group = definition of group regular and-or injected
  *             .channel = HA_ADC_CHANNEL_ALL
  *         - to monitor a list of channels (AWD2 and AWD3 only): call this function once to configure
  *           all parameters with one channel, then call function @ref HAL_ADC_SetAnalogWDChannel() for channels to add.
  *         - to disable ADC analog watchdog, use following parameters values:
  *             .group = HAL_ADC_GROUP_NONE
  *             .channel = HAL_ADC_CHANNEL_NONE
  *           (parameters HAL_ADC_GROUP_NONE and HAL_ADC_CHANNEL_NONE must be used together, not separately)
  * @note   Analog watchdog instances specificities: refer to description of "hal_adc_awd_instance_t".
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigAnalogWD(hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance,
                                       const hal_adc_awd_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t adc_resolution;
  int32_t threshold_high_res;
  int32_t threshold_low_res;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));
  ASSERT_DBG_PARAM(IS_ADC_GROUP(p_config->group));
  ASSERT_DBG_PARAM(IS_ADC_CHANNEL(hadc->instance, p_config->channel));
  if (p_config->channel != HAL_ADC_CHANNEL_NONE)
  {
    ASSERT_DBG_PARAM(IS_ADC_AWD_THRESHOLD(p_config->threshold_high));
    ASSERT_DBG_PARAM(IS_ADC_AWD_THRESHOLD(p_config->threshold_low));

    /* Case filtering selected */
    ASSERT_DBG_PARAM(IS_ADC_AWD_FILTERING(p_config->filtering));
    if (awd_instance != HAL_ADC_AWD_1)
    {
      ASSERT_DBG_PARAM(p_config->filtering == HAL_ADC_AWD_FILTERING_NONE);
    }
  }

  /* Check parameters values dependent to specific conditions */
  /* Case analog watchdog disable: if group set to HAL_ADC_GROUP_NONE, channel must be set to HAL_ADC_CHANNEL_NONE */
  if (p_config->channel == HAL_ADC_CHANNEL_NONE)
  {
    ASSERT_DBG_PARAM(p_config->group == HAL_ADC_GROUP_NONE);
  }
  if (p_config->group == HAL_ADC_GROUP_NONE)
  {
    ASSERT_DBG_PARAM(p_config->channel == HAL_ADC_CHANNEL_NONE);
  }

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Apply analog watchdog configuration */
  LL_ADC_SetAnalogWDScope(p_instance, (uint32_t)awd_instance, (uint32_t)p_config->group, (uint32_t)p_config->channel);

  if (p_config->channel != HAL_ADC_CHANNEL_NONE)
  {
    /* Thresholds computation: convert from numerical value to ADC peripheral digital value */
    adc_resolution = LL_ADC_GetResolution(p_instance);
    threshold_high_res = (int32_t)LL_ADC_ANALOGWD_SET_THRESHOLD_RES(adc_resolution, p_config->threshold_high);
    threshold_low_res = (int32_t)LL_ADC_ANALOGWD_SET_THRESHOLD_RES(adc_resolution, p_config->threshold_low);

    LL_ADC_SetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, LL_ADC_AWD_THRESHOLD_HIGH, threshold_high_res);
    LL_ADC_SetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, LL_ADC_AWD_THRESHOLD_LOW, threshold_low_res);

    if (awd_instance == HAL_ADC_AWD_1)
    {
      LL_ADC_SetAnalogWDFiltering(p_instance, (uint32_t)awd_instance, (uint32_t)p_config->filtering);
    }
  }

  return status;
}

/**
  * @brief  Get configuration of ADC analog watchdog.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @param  p_config Pointer to a hal_adc_awd_config_t structure containing ADC analog watchdog configuration.
  * @note   Usage of the returned structure field "channel":
  *         To reinject this channel into another function HAL_ADC_xxx: the returned channel number
  *         is only partly formatted on definition of literals HAL_ADC_CHANNEL_x.
  *         Therefore, it has to be compared with literals HAL_ADC_CHANNEL_x using
  *         helper macro LL_ADC_CHANNEL_TO_DECIMAL_NB().
  * @note   Case of multiple channels monitored (AWD2 and AWD3 only): the channel value returned corresponds
  *         to the lowest one. If needed to check a specific channel, use LL_ADC_IsAnalogWDChannelMonitored().
  */
void HAL_ADC_GetConfigAnalogWD(const hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance,
                               hal_adc_awd_config_t *p_config)
{
  ADC_TypeDef *p_instance;
  uint32_t adc_resolution;
  int32_t threshold_high;
  int32_t threshold_low;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->group = (hal_adc_group_t)LL_ADC_GetAnalogWDScopeGroup(p_instance, (uint32_t)awd_instance);
  p_config->channel = (hal_adc_channel_t)LL_ADC_GetAnalogWDScopeChannel(p_instance, (uint32_t)awd_instance);

  if ((p_config->channel != HAL_ADC_CHANNEL_ALL) && (p_config->channel != HAL_ADC_CHANNEL_NONE))
  {
    /* Case unitary channel */
    p_config->channel = (hal_adc_channel_t)
                        ((uint32_t)LL_ADC_DECIMAL_NB_TO_CHANNEL(
                           LL_ADC_CHANNEL_TO_DECIMAL_NB(((uint32_t)p_config->channel))));
  }

  /* Thresholds computation: convert from numerical value to ADC peripheral digital value */
  threshold_high = LL_ADC_GetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, LL_ADC_AWD_THRESHOLD_HIGH);
  threshold_low = LL_ADC_GetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, LL_ADC_AWD_THRESHOLD_LOW);
  adc_resolution = LL_ADC_GetResolution(p_instance);
  p_config->threshold_high = (int32_t)LL_ADC_ANALOGWD_GET_THRESHOLD_RES(adc_resolution, threshold_high);
  p_config->threshold_low = (int32_t)LL_ADC_ANALOGWD_GET_THRESHOLD_RES(adc_resolution, threshold_low);

  if (awd_instance == HAL_ADC_AWD_1)
  {
    p_config->filtering = (hal_adc_awd_filtering_t)LL_ADC_GetAnalogWDFiltering(p_instance, (uint32_t)awd_instance);
  }
  else
  {
    p_config->filtering = HAL_ADC_AWD_FILTERING_NONE;
  }
}

/**
  * @brief  Configure ADC analog watchdog parameter: thresholds.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @param  awd_threshold_sel Analog watchdog threshold selection (high or low)
  * @param  awd_threshold_value Analog watchdog threshold value. Value is signed and can exceed ADC resolution
  *         with post-processing computation (offset, oversampling, data shift, ...).
  *         Value between Min_Data=-4194304 (two's complement 0xFFC00000) and Max_Data=+4194303 (0x003FFFFF)
  * @note   Function intended to update analog watchdog thresholds whatever ADC conversion state, on the fly
  *         (other analog watchdog parameters require ADC deactivated or no conversion ongoing)
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetAnalogWDThresholds(hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance,
                                           hal_adc_awd_threshold_sel_t awd_threshold_sel, int32_t awd_threshold_value)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t adc_resolution;
  int32_t threshold_value_res;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));
  ASSERT_DBG_PARAM(IS_ADC_AWD_THRESHOLD_SEL(awd_threshold_sel));
  ASSERT_DBG_PARAM(IS_ADC_AWD_THRESHOLD(awd_threshold_value));

  /* Analog watchdog thresholds can be updated even during ADC conversion */
  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE |
                   (uint32_t)HAL_ADC_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Thresholds computation: convert from numerical value to ADC peripheral digital value */
  adc_resolution = LL_ADC_GetResolution(p_instance);
  threshold_value_res = (int32_t)LL_ADC_ANALOGWD_SET_THRESHOLD_RES(adc_resolution, awd_threshold_value);

  LL_ADC_SetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, (uint32_t)awd_threshold_sel, threshold_value_res);

  return status;
}

/**
  * @brief  Get ADC analog watchdog parameter: thresholds.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @param  awd_threshold_sel Analog watchdog threshold selection (high or low)
  * @note   Function intended to update analog watchdog thresholds whatever ADC conversion state, on the fly
  *         (other analog watchdog parameters require ADC deactivated or no conversion ongoing)
  * @retval Analog watchdog threshold value. Value is signed and can exceed ADC resolution
  *         with post-processing computation (offset, oversampling, data shift, ...).
  *         Value between Min_Data=-4194304 (two's complement 0xFFC00000) and Max_Data=+4194303 (0x003FFFFF)
  */
int32_t HAL_ADC_GetAnalogWDThresholds(const hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance,
                                      hal_adc_awd_threshold_sel_t awd_threshold_sel)
{
  ADC_TypeDef *p_instance;
  uint32_t adc_resolution;
  int32_t threshold_value;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));
  ASSERT_DBG_PARAM(IS_ADC_AWD_THRESHOLD_SEL(awd_threshold_sel));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Thresholds computation: convert from numerical value to ADC peripheral digital value */
  threshold_value = LL_ADC_GetAnalogWDThresholds(p_instance, (uint32_t)awd_instance, (uint32_t)awd_threshold_sel);
  adc_resolution = LL_ADC_GetResolution(p_instance);
  threshold_value = (int32_t)LL_ADC_ANALOGWD_GET_THRESHOLD_RES(adc_resolution, threshold_value);

  return threshold_value;
}

/**
  * @brief  Configure ADC analog watchdog parameter: ADC channel.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @param  channel ADC channel to be monitored
  * @note   This function keeps unchanged ADC group configuration monitored by analog watchdog, must have been
  *         previously configured.
  *         For analog watchdog monitoring to be effective, the selected ADC channel must be converted by
  *         this ADC group.
  *         If needed to update it, use function @ref HAL_ADC_SetConfigAnalogWD().
  * @note   To monitor a list of channels (AWD2 and AWD3 only): call @ref HAL_ADC_SetConfigAnalogWD() once to configure
  *         all parameters with one channel, then this functions for each channels to add.
  *         Channels list can be flushed with parameter HAL_ADC_CHANNEL_NONE.
  * @note   Analog watchdog instances specificities: refer to description of "hal_adc_awd_instance_t".
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetAnalogWDChannel(hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance,
                                        hal_adc_channel_t channel)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t group;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));
  ASSERT_DBG_PARAM(IS_ADC_CHANNEL(hadc->instance, channel));

  /* Required state: no conversion ongoing (implies global state can be idle or active) */
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  if (awd_instance == HAL_ADC_AWD_1)
  {
    if (channel == HAL_ADC_CHANNEL_NONE)
    {
      /* Case analog watchdog disable */
      group = LL_ADC_GROUP_NONE;
    }
    else
    {
      /* Case channel monitored update */
      group = (uint32_t)LL_ADC_GetAnalogWDScopeGroup(p_instance, (uint32_t)awd_instance);
    }

    /* Apply analog watchdog configuration */
    LL_ADC_SetAnalogWDScope(p_instance, (uint32_t)awd_instance, group, (uint32_t)channel);
  }
  else /* HAL_ADC_AWD_2, HAL_ADC_AWD_3 */
  {
    if (channel == HAL_ADC_CHANNEL_NONE)
    {
      /* Case analog watchdog disable */
      LL_ADC_SetAnalogWDChannelRem(p_instance, (uint32_t)awd_instance, LL_ADC_CHANNEL_ALL);
    }
    else
    {
      /* Case channel monitored update: add channel to list of monitored channels */
      LL_ADC_SetAnalogWDChannelAdd(p_instance, (uint32_t)awd_instance, (uint32_t)channel);
    }
  }

  return status;
}

/**
  * @brief  Get ADC analog watchdog parameter: ADC channel.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance.
  * @note   This function does not return information of ADC group configuration monitored by analog watchdog.
  *         For analog watchdog monitoring to be effective, the selected ADC channel must be converted by
  *         this ADC group.
  *         If needed to retrieve it, use function @ref HAL_ADC_GetConfigAnalogWD().
  * @note   Usage of the returned channel value:
  *         To reinject this channel into another function HAL_ADC_xxx: the returned channel number
  *         is only partly formatted on definition of literals HAL_ADC_CHANNEL_x.
  *         Therefore, it has to be compared with literals HAL_ADC_CHANNEL_x using
  *         helper macro LL_ADC_CHANNEL_TO_DECIMAL_NB().
  * @note   Case of multiple channels monitored (AWD2 and AWD3 only): the channel value returned corresponds
  *         to the lowest one. If needed to check a specific channel, use LL_ADC_IsAnalogWDChannelMonitored().
  * @note   Analog watchdog instances specificities: refer to description of "hal_adc_awd_instance_t".
  * @retval ADC channel partially corresponding to literals of hal_adc_channel_t
  *         (the returned value is only partially formatted on definition of hal_adc_channel_t.
  *         It can be reinjected in other functions or used with LL_ADC_CHANNEL_TO_DECIMAL_NB())
  */
hal_adc_channel_t HAL_ADC_GetAnalogWDChannel(const hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance)
{
  ADC_TypeDef *p_instance;
  hal_adc_channel_t channel;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_AWD_INSTANCE(awd_instance));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  channel = (hal_adc_channel_t)LL_ADC_GetAnalogWDScopeChannel(p_instance, (uint32_t)awd_instance);

  if ((channel != HAL_ADC_CHANNEL_ALL) && (channel != HAL_ADC_CHANNEL_NONE))
  {
    /* Case unitary channel */
    channel = (hal_adc_channel_t)
              ((uint32_t)LL_ADC_DECIMAL_NB_TO_CHANNEL(LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)channel)));
  }

  return channel;
}

/**
  * @brief  Configure ADC oversampling.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  ovs_instance Oversampling instance.
  * @param  p_config Pointer to a hal_adc_ovs_config_t structure containing ADC oversampling configuration.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigOverSampling(hal_adc_handle_t *hadc, hal_adc_ovs_instance_t ovs_instance,
                                           const hal_adc_ovs_config_t *p_config)
{
  STM32_UNUSED(ovs_instance); /* Function argument not used on this STM32 series */
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_OVS_INSTANCE(ovs_instance));
  ASSERT_DBG_PARAM(IS_ADC_OVS_SCOPE(p_config->scope));
  if (p_config->scope != HAL_ADC_OVS_DISABLE)
  {
    ASSERT_DBG_PARAM(IS_ADC_OVS_DISCONT(p_config->discont));
    ASSERT_DBG_PARAM(IS_ADC_OVS_RATIO(p_config->ratio));
    ASSERT_DBG_PARAM(IS_ADC_OVS_SHIFT(p_config->shift));
  }

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetOverSamplingScope(p_instance, (uint32_t)p_config->scope);

  if (p_config->scope != HAL_ADC_OVS_DISABLE)
  {
    LL_ADC_SetOverSamplingDiscont(p_instance, (uint32_t)p_config->discont);
    LL_ADC_ConfigOverSamplingRatioShift(p_instance, (uint32_t)p_config->ratio, ((uint32_t)p_config->shift));
  }

  return status;
}

/**
  * @brief  Get configuration of ADC oversampling.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  ovs_instance Oversampling instance.
  * @param  p_config Pointer to a hal_adc_ovs_config_t structure containing ADC oversampling configuration.
  */
void HAL_ADC_GetConfigOverSampling(const hal_adc_handle_t *hadc, hal_adc_ovs_instance_t ovs_instance,
                                   hal_adc_ovs_config_t *p_config)
{
  STM32_UNUSED(ovs_instance); /* Function argument not used on this STM32 series */
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_OVS_INSTANCE(ovs_instance));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->scope = (hal_adc_ovs_scope_t)LL_ADC_GetOverSamplingScope(p_instance);
  p_config->discont = (hal_adc_ovs_discont_t)LL_ADC_GetOverSamplingDiscont(p_instance);
  p_config->ratio = (uint16_t)LL_ADC_GetOverSamplingRatio(p_instance);
  p_config->shift = (uint8_t)LL_ADC_GetOverSamplingShift(p_instance);
}

/**
  * @brief  Compute ADC oversampling right bit shift value in function of ratio to have oversampling data
  *         keeping current resolution (example: to keep data resolution, ratio x8 requires right shift of 3 bits).
  * @param  ratio ADC oversampling ratio, value must be from 1 to 1024 and a power of 2: {1; 2; 4; 8; ...; 1024}.
  * @note   Value intended to be used for parameter "shift" of @ref hal_adc_ovs_config_t.
  * @retval Value of right bit shift (number between Min_Data = 1 and Max_Data = 10)
  */
uint32_t HAL_ADC_GetOverSamplingShiftKeepRes(uint32_t ratio)
{
  uint32_t right_bit_shift;

  ASSERT_DBG_PARAM(IS_ADC_OVS_RATIO_POW_2(ratio));

  right_bit_shift = LL_ADC_OVS_SHIFT_KEEP_RES(ratio);

  return right_bit_shift;
}

/**
  * @brief  Configure ADC offset subblock features (offset level, sign, saturation, ...).
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  offset_instance Offset instance.
  * @param  p_config Pointer to a hal_adc_offset_config_t structure containing ADC offset configuration.
  * @note   To remove a channel from an offset instance, set configuration with channel to HAL_ADC_CHANNEL_NONE.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetConfigOffset(hal_adc_handle_t *hadc, hal_adc_offset_instance_t offset_instance,
                                     const hal_adc_offset_config_t *p_config)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t offset_sign;
  uint32_t adc_resolution;
  uint32_t offset_level_processed;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_ADC_OFFSET_INSTANCE(offset_instance));
  ASSERT_DBG_PARAM(IS_ADC_CHANNEL(hadc->instance, p_config->channel));
  if (p_config->channel != HAL_ADC_CHANNEL_NONE)
  {
    ASSERT_DBG_PARAM(p_config->channel != HAL_ADC_CHANNEL_ALL);
    ASSERT_DBG_PARAM(IS_ADC_OFFSET_LEVEL(p_config->level));
    ASSERT_DBG_PARAM(IS_ADC_OFFSET_SAT_SIGN(p_config->saturation_signed));
    ASSERT_DBG_PARAM(IS_ADC_OFFSET_SAT_UNSIGN(p_config->saturation_unsigned));
  }

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetOffsetChannel(p_instance, (uint32_t)offset_instance, (uint32_t)p_config->channel);

  if (p_config->channel == HAL_ADC_CHANNEL_NONE)
  {
    LL_ADC_SetOffsetLevel(p_instance, (uint32_t)offset_instance, 0UL);
  }
  else
  {
    /* Manage offset level sign */
    if (p_config->level < 0L)
    {
      offset_level_processed = (uint32_t)(-p_config->level);
      offset_sign = LL_ADC_OFFSET_SIGN_NEGATIVE;
    }
    else
    {
      offset_level_processed = (uint32_t)p_config->level;
      offset_sign = LL_ADC_OFFSET_SIGN_POSITIVE;
    }

    /* Offset level computation: convert from numerical value to ADC peripheral digital value depending on resolution */
    adc_resolution = LL_ADC_GetResolution(p_instance);
    offset_level_processed = LL_ADC_OFFSET_SET_LEVEL_RES(adc_resolution, offset_level_processed);

    LL_ADC_SetOffsetLevel(p_instance, (uint32_t)offset_instance, offset_level_processed);
    LL_ADC_SetOffsetSign(p_instance, (uint32_t)offset_instance, offset_sign);
    LL_ADC_SetOffsetSignedSaturation(p_instance, (uint32_t)offset_instance, (uint32_t)p_config->saturation_signed);
    LL_ADC_SetOffsetUnsignedSaturation(p_instance, (uint32_t)offset_instance, (uint32_t)p_config->saturation_unsigned);
  }

  return status;
}

/**
  * @brief  Get configuration of ADC offset subblock features (offset level, sign, saturation, ...).
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  offset_instance Offset instance.
  * @param  p_config Pointer to a hal_adc_offset_config_t structure containing ADC offset configuration.
  * @note   Usage of the returned structure field "channel":
  *         To reinject this channel into another function HAL_ADC_xxx: the returned channel number
  *         is only partly formatted on definition of literals HAL_ADC_CHANNEL_x.
  *         Therefore, it has to be compared with literals HAL_ADC_CHANNEL_x using
  *         helper macro LL_ADC_CHANNEL_TO_DECIMAL_NB().
  */
void HAL_ADC_GetConfigOffset(const hal_adc_handle_t *hadc,  hal_adc_offset_instance_t offset_instance,
                             hal_adc_offset_config_t *p_config)
{
  ADC_TypeDef *p_instance;
  uint32_t offset_sign;
  uint32_t adc_resolution;
  uint32_t offset_level_processed;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM((p_config != NULL));
  ASSERT_DBG_PARAM(IS_ADC_OFFSET_INSTANCE(offset_instance));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  p_config->channel = (hal_adc_channel_t)(LL_ADC_GetOffsetChannel(p_instance, (uint32_t) offset_instance));
  adc_resolution = LL_ADC_GetResolution(p_instance);
  offset_level_processed = LL_ADC_GetOffsetLevel(p_instance, (uint32_t)offset_instance);
  offset_level_processed = LL_ADC_OFFSET_GET_LEVEL_RES(adc_resolution, offset_level_processed);

  /* Manage offset level sign */
  offset_level_processed = (offset_level_processed & ADC_OFR_OFFSET); /* Explicitly limit value to register
  bitfield range to avoid compiler warning of potential overflow */
  offset_sign = LL_ADC_GetOffsetSign(p_instance, (uint32_t)offset_instance);
  if (offset_sign == LL_ADC_OFFSET_SIGN_NEGATIVE)
  {
    p_config->level = -((int32_t)offset_level_processed);
  }
  else
  {
    p_config->level = (int32_t)offset_level_processed;
  }

  p_config->saturation_signed = (hal_adc_offset_sat_sign_state_t)
                                LL_ADC_GetOffsetSignedSaturation(p_instance, (uint32_t)offset_instance);
  p_config->saturation_unsigned = (hal_adc_offset_sat_unsign_state_t)
                                  LL_ADC_GetOffsetUnsignedSaturation(p_instance, (uint32_t)offset_instance);
}

/**
  * @brief  Set ADC offset subblock parameter: offset level.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  offset_instance Offset instance.
  * @param  offset_level ADC offset level (signed value between Min_Data= -1*0x003FFFFF (two's complement 0xFFC00001)
  *         and Max_Data=0x003FFFFF).
  * @note   Other ADC offset subblock parameters can be set using @ref HAL_ADC_SetConfigOffset().
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetOffsetLevel(hal_adc_handle_t *hadc,  hal_adc_offset_instance_t offset_instance,
                                    int32_t offset_level)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t offset_sign;
  uint32_t adc_resolution;
  uint32_t offset_level_processed;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OFFSET_INSTANCE(offset_instance));
  ASSERT_DBG_PARAM(IS_ADC_OFFSET_LEVEL(offset_level));

  ASSERT_DBG_STATE(hadc->global_state,
                   (uint32_t)HAL_ADC_STATE_CONFIGURING |
                   (uint32_t)HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Manage offset level sign */
  if (offset_level < 0L)
  {
    offset_level_processed = (uint32_t)(-offset_level);
    offset_sign = LL_ADC_OFFSET_SIGN_NEGATIVE;
  }
  else
  {
    offset_level_processed = (uint32_t)offset_level;
    offset_sign = LL_ADC_OFFSET_SIGN_POSITIVE;
  }

  /* Offset level computation: convert from numerical value to ADC peripheral digital value depending on resolution */
  adc_resolution = LL_ADC_GetResolution(p_instance);
  offset_level_processed = LL_ADC_OFFSET_SET_LEVEL_RES(adc_resolution, offset_level_processed);

  LL_ADC_SetOffsetLevel(p_instance, (uint32_t)offset_instance, offset_level_processed);
  LL_ADC_SetOffsetSign(p_instance, (uint32_t)offset_instance, offset_sign);

  return status;
}

/**
  * @brief  Get ADC offset subblock parameter: offset level.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  offset_instance Offset instance.
  * @note   Other ADC offset subblock parameters can be retrieved using @ref HAL_ADC_GetConfigOffset().
  * @retval ADC offset level (signed value between Min_Data= -1*0x003FFFFF (two's complement 0xFFC00001)
  *         and Max_Data=0x003FFFFF).
  */
int32_t HAL_ADC_GetOffsetLevel(const hal_adc_handle_t *hadc, hal_adc_offset_instance_t offset_instance)
{
  ADC_TypeDef *p_instance;
  uint32_t offset_sign;
  uint32_t adc_resolution;
  uint32_t offset_level_processed;
  int32_t offset_level;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OFFSET_INSTANCE(offset_instance));

  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Thresholds computation: convert from ADC peripheral digital value to numerical value depending on resolution */
  adc_resolution = LL_ADC_GetResolution(p_instance);
  offset_level_processed = LL_ADC_GetOffsetLevel(p_instance, (uint32_t)offset_instance);
  offset_level_processed = LL_ADC_OFFSET_GET_LEVEL_RES(adc_resolution, offset_level_processed);

  /* Manage offset level sign */
  offset_sign = LL_ADC_GetOffsetSign(p_instance, (uint32_t)offset_instance);
  if (offset_sign == LL_ADC_OFFSET_SIGN_NEGATIVE)
  {
    offset_level = -((int32_t)offset_level_processed);
  }
  else
  {
    offset_level = (int32_t)offset_level_processed;
  }

  return offset_level;
}

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group3
  * @{
    Set of function to handle the ADC interruptions :
      + HAL_ADC_IRQHandler(): Handle all ADC interrupt requests
      + HAL_ADC_IRQHandler_REG(): Handle ADC interrupt requests optimized: specific to ADC group regular
      + HAL_ADC_IRQHandler_INJ(): Handle ADC interrupt requests optimized: specific to ADC group injected
      + HAL_ADC_IRQHandler_AWD(): Handle ADC interrupt requests optimized: specific to ADC analog watchdog
  */

/**
  * @brief  Handle all ADC interrupt requests.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   For optimized process of specific interrupts, refer to other HAL_ADC_IRQHandler_x functions.
  */
void HAL_ADC_IRQHandler(hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  p_instance = ADC_GET_INSTANCE(hadc);

  uint32_t flag_status = LL_ADC_READ_REG(p_instance, ISR);
  uint32_t it_sources = LL_ADC_READ_REG(p_instance, IER);
  uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND with flags status and interrupts sources
                                                             (registers bitfields aligned) */

  if ((flag_status_masked & LL_ADC_FLAG_EOC) != 0UL)
  {
    LL_ADC_ClearFlag_EOC(p_instance);

    if (hadc->group_conv_per_start[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_CONV_UNIT)
    {
      hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
    }

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_eoc_cb(hadc);
#else
    HAL_ADC_REG_UnitaryConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_EOS) != 0UL)
  {
    LL_ADC_ClearFlag_EOS(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_eos_cb(hadc);
#else
    HAL_ADC_REG_SequenceConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_OVR) != 0UL)
  {
    LL_ADC_ClearFlag_OVR(p_instance);
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    hadc->last_error_codes |= HAL_ADC_REG_ERROR_OVR;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_error_cb(hadc);
#else
    HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_EOSMP) != 0UL)
  {
    LL_ADC_ClearFlag_EOSMP(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_end_of_sampling_cb(hadc);
#else
    HAL_ADC_REG_EndOfSamplingCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_JEOC) != 0UL)
  {
    LL_ADC_ClearFlag_JEOC(p_instance);

    if (hadc->group_conv_per_start[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_CONV_UNIT)
    {
      hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
    }

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_inj_eoc_cb(hadc);
#else
    HAL_ADC_INJ_UnitaryConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_JEOS) != 0UL)
  {
    LL_ADC_ClearFlag_JEOS(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_inj_eos_cb(hadc);
#else
    HAL_ADC_INJ_SequenceConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_AWD1) != 0UL)
  {
    LL_ADC_ClearFlag_AWD1(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_1);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_1);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_AWD2) != 0UL)
  {
    LL_ADC_ClearFlag_AWD2(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_2);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_2);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_AWD3) != 0UL)
  {
    LL_ADC_ClearFlag_AWD3(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_3);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_3);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Handle ADC interrupt requests optimized: specific to ADC group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   For generic process of all ADC interrupts request, use function HAL_ADC_IRQHandler().
  */
void HAL_ADC_IRQHandler_REG(hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  p_instance = ADC_GET_INSTANCE(hadc);

  uint32_t flag_status = LL_ADC_READ_REG(p_instance, ISR);
  uint32_t it_sources = LL_ADC_READ_REG(p_instance, IER);
  uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND with flags status and interrupts sources
                                                             (registers bitfields aligned) */

  if ((flag_status_masked & LL_ADC_FLAG_EOC) != 0UL)
  {
    LL_ADC_ClearFlag_EOC(p_instance);

    if (hadc->group_conv_per_start[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_CONV_UNIT)
    {
      hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
    }

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_eoc_cb(hadc);
#else
    HAL_ADC_REG_UnitaryConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_EOS) != 0UL)
  {
    LL_ADC_ClearFlag_EOS(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_eos_cb(hadc);
#else
    HAL_ADC_REG_SequenceConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_OVR) != 0UL)
  {
    LL_ADC_ClearFlag_OVR(p_instance);
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    hadc->last_error_codes |= HAL_ADC_REG_ERROR_OVR;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_error_cb(hadc);
#else
    HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_EOSMP) != 0UL)
  {
    LL_ADC_ClearFlag_EOSMP(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_reg_end_of_sampling_cb(hadc);
#else
    HAL_ADC_REG_EndOfSamplingCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Handle ADC interrupt requests optimized: specific to ADC group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   For generic process of all ADC interrupts request, use function HAL_ADC_IRQHandler().
  */
void HAL_ADC_IRQHandler_INJ(hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  p_instance = ADC_GET_INSTANCE(hadc);

  uint32_t flag_status = LL_ADC_READ_REG(p_instance, ISR);
  uint32_t it_sources = LL_ADC_READ_REG(p_instance, IER);
  uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND with flags status and interrupts sources
                                                             (registers bitfields aligned) */

  if ((flag_status_masked & LL_ADC_FLAG_JEOC) != 0UL)
  {
    LL_ADC_ClearFlag_JEOC(p_instance);

    if (hadc->group_conv_per_start[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_CONV_UNIT)
    {
      hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
    }

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_inj_eoc_cb(hadc);
#else
    HAL_ADC_INJ_UnitaryConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_JEOS) != 0UL)
  {
    LL_ADC_ClearFlag_JEOS(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_inj_eos_cb(hadc);
#else
    HAL_ADC_INJ_SequenceConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Handle ADC interrupt requests optimized: specific to ADC analog watchdog.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   For generic process of all ADC interrupts request, use function HAL_ADC_IRQHandler().
  */
void HAL_ADC_IRQHandler_AWD(hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  p_instance = ADC_GET_INSTANCE(hadc);

  uint32_t flag_status = LL_ADC_READ_REG(p_instance, ISR);
  uint32_t it_sources = LL_ADC_READ_REG(p_instance, IER);
  uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND with flags status and interrupts sources
                                                             (registers bitfields aligned) */

  if ((flag_status_masked & LL_ADC_FLAG_AWD1) != 0UL)
  {
    LL_ADC_ClearFlag_AWD1(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_1);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_1);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_AWD2) != 0UL)
  {
    LL_ADC_ClearFlag_AWD2(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_2);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_2);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_ADC_FLAG_AWD3) != 0UL)
  {
    LL_ADC_ClearFlag_AWD3(p_instance);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->p_awd_out_window_cb(hadc, HAL_ADC_AWD_3);
#else
    HAL_ADC_AnalogWD_OutOfWindowCallback(hadc, HAL_ADC_AWD_3);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  HAL ADC error callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_ErrorCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_ErrorCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular end of sampling phase callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   ADC conversion has 2 phases (sampling, SAR conversion). Conversion data is available upon
  *         conversion complete callbacks.
  * @note   Sampling phase duration can be set using ADC channel sampling time.
  */
__WEAK void HAL_ADC_REG_EndOfSamplingCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_EndOfSamplingCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular end of unitary conversion callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_REG_UnitaryConvCpltCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_UnitaryConvCpltCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular end of sequence conversions callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_REG_SequenceConvCpltCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_SequenceConvCpltCallback() can be implemented in the user file. */
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  ADC group regular conversion data buffer half transfer.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_REG_DataTransferHalfCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_DataTransferHalfCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular conversion data buffer transfer complete.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_REG_DataTransferCpltCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_DataTransferCpltCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular conversion data transfer abort.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_REG_DataTransferStopCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_REG_DataTransferStopCallback() can be implemented in the user file. */
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  ADC group injected end of unitary conversion callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_INJ_UnitaryConvCpltCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_INJ_UnitaryConvCpltCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group injected end of sequence conversions callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  */
__WEAK void HAL_ADC_INJ_SequenceConvCpltCallback(hal_adc_handle_t *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_INJ_SequenceConvCpltCallback() can be implemented in the user file. */
}

/**
  * @brief  ADC group regular analog watchdog out of window event callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  awd_instance Analog watchdog instance
  */
__WEAK void HAL_ADC_AnalogWD_OutOfWindowCallback(hal_adc_handle_t *hadc, hal_adc_awd_instance_t awd_instance)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hadc);
  STM32_UNUSED(awd_instance);

  /* Warning : This function must not be modified. When the callback is needed, function
               HAL_ADC_AnalogWD_OutOfWindowCallback() can be implemented in the user file. */
}

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register ADC error callback function to be used in place of
  *         the weak HAL_ADC_ErrorCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterErrorCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_error_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group regular end of sampling phase callback function to be used in place of
  *         the weak HAL_ADC_REG_EndOfSamplingCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterRegEndOfSamplingCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_end_of_sampling_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group regular end of unitary conversion callback to be used in place of
  *         the weak HAL_ADC_REG_UnitaryConvCpltCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterRegUnitaryConvCpltCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_eoc_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group regular end of sequence conversions callback function to be used in place of
  *         the weak HAL_ADC_REG_SequenceConvCpltCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterRegSequenceConvCpltCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_eos_cb = p_callback;

  return HAL_OK;
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Register ADC group regular conv data buffer half transfer callback function to be used in place of
  *         the weak HAL_ADC_REG_DataTransferHalfCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterDataTransferHalfCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_xfer_half_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group regular conv data buffer transfer complete callback function to be used in place of
  *         the weak HAL_ADC_REG_DataTransferCpltCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterDataTransferCpltCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_xfer_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group regular conv data transfer abort callback function to be used in place of
  *         the weak HAL_ADC_REG_DataTransferStopCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterDataTransferStopCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_reg_xfer_stop_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Register ADC group injected end of unitary conversion callback to be used in place of
  *         the weak HAL_ADC_INJ_UnitaryConvCpltCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterInjUnitaryConvCpltCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_inj_eoc_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC group injected end of sequence conversions callback function to be used in place of
  *         the weak HAL_ADC_INJ_SequenceConvCpltCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterInjSequenceConvCpltCallback(hal_adc_handle_t *hadc, hal_adc_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_inj_eos_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register ADC analog watchdog out of window event callback function to be used in place of
  *         the weak HAL_ADC_AnalogWD_OutOfWindowCallback() predefined callback.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_callback Pointer to the hal_adc_awd_cb_t callback function.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_OK    Register completed successfully.
  * @retval HAL_ERROR Register completed with error.
  */
hal_status_t HAL_ADC_RegisterAwdOutOfWindowCallback(hal_adc_handle_t *hadc, hal_adc_awd_cb_t p_callback)
{
  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_callback != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_callback == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hadc->p_awd_out_window_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group4
  * @{
    Set of function to handle the HAL ADC driver state, errors, kernel clock frequency:
      + HAL_ADC_GetState(): Retrieve the HAL ADC global state
      + HAL_ADC_GetStateGroup(): Retrieve the HAL ADC groups (regular, injected) state
      + HAL_ADC_GetStateCommon(): Retrieve the HAL ADC handle link to common instance state
      + HAL_ADC_GetLastErrorCodes(): Retrieve the HAL ADC last error codes
      + HAL_ADC_GetClockFreq(): Retrieve the HAL ADC instance kernel clock frequency
  */

/**
  * @brief  Retrieve the HAL ADC global state.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval hal_adc_state_t HAL ADC global state
  */
hal_adc_state_t HAL_ADC_GetState(const hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM((hadc != NULL));

  return hadc->global_state;
}

/**
  * @brief  Retrieve the HAL ADC groups (regular, injected) state.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  group ADC group from which conversion data is compared to thresholds
  *         This parameter can be one of the following values:
  *         @arg @ref HAL_ADC_GROUP_REGULAR
  *         @arg @ref HAL_ADC_GROUP_INJECTED
  * @retval hal_adc_group_state_t HAL ADC global state
  */
hal_adc_group_state_t HAL_ADC_GetStateGroup(const hal_adc_handle_t *hadc, hal_adc_group_t group)
{
  ASSERT_DBG_PARAM((hadc != NULL));

  return hadc->group_state[(uint8_t)group - 1U];
}

/**
  * @brief  Retrieve the HAL ADC handle link to common instance state.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval hal_adc_common_state_t HAL ADC global state
  */
hal_adc_common_state_t HAL_ADC_GetStateCommon(const hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM((hadc != NULL));

  return hadc->common_state;
}

#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
/**
  * @brief  Retrieve the HAL ADC last error codes.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval uint32_t last error code.
  *         This parameter can be a combination (bitfields) of the following values: \n
  *         @arg @ref HAL_ADC_ERROR_NONE
  *         @arg @ref HAL_ADC_ERROR_INTERNAL
  *         @arg @ref HAL_ADC_REG_ERROR_OVR
  *         @arg @ref HAL_ADC_REG_ERROR_DMA
  */
uint32_t HAL_ADC_GetLastErrorCodes(const hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM((hadc != NULL));

  return hadc->last_error_codes;
}
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

/** @brief  Return the peripheral clock frequency for ADC.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   Clock frequency corresponds to ADC kernel clock, including clock source and prescaler configurations
  *         from all related modules (RCC and ADC).
  * @retval uint32_t Frequency in Hz.
  *                  0 if the source clock of the ADC is not configured or not ready.
  */
uint32_t HAL_ADC_GetClockFreq(const hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM((hadc != NULL));
  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                                         | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_CALIB
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

#if !defined(USE_ASSERT_DBG_STATE) && !defined(USE_ASSERT_DBG_PARAM)
  STM32_UNUSED(hadc);
#endif /* USE_ASSERT_DBG_STATE or USE_ASSERT_DBG_PARAM */

  return HAL_RCC_ADC_GetKernelClkFreq(ADC_GET_INSTANCE(hadc));
}

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group5
  * @{
    Set of functions allowing to operate ADCx peripheral.
      + Activation and deactivation
      + Calibration
      + ADC conversions management
    @note For more details, refer to state machine diagram or "How to use the ADC HAL module driver" section).
  */

/**
  * @brief  Activate ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_Start(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Verify configuration compliance to hardware constraints:
     with operation on group injected, specific sampling mode cannot be used */
  if (hadc->group_state[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_STATE_IDLE)
  {
    if (LL_ADC_GetSamplingMode(p_instance) != LL_ADC_SAMPLING_MODE_NORMAL)
    {
      status = HAL_ERROR;
    }
  }

  if (status == HAL_OK)
  {
    HAL_CHECK_UPDATE_STATE(hadc, global_state,
                           HAL_ADC_STATE_IDLE,
                           HAL_ADC_STATE_ACTIVE);

    if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
    {
      HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_INJECTED],
                             HAL_ADC_GROUP_STATE_IDLE,
                             HAL_ADC_GROUP_STATE_ACTIVE);
    }

    status = adc_activate(hadc);
  }

  return status;
}

/**
  * @brief  Deactivate ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_Stop(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);

  p_instance = ADC_GET_INSTANCE(hadc);

  if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
  {
    ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                     (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                     (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                     (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);
  }
  else
  {
    ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                     (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                     (uint32_t)HAL_ADC_GROUP_STATE_IDLE);
  }

  status = adc_deactivate(hadc);

  /* Clear flags and interruptions */
  LL_ADC_ClearFlag(p_instance, LL_ADC_FLAG_ALL);
  LL_ADC_DisableIT(p_instance, LL_ADC_IT_ALL);

  if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
  {
    hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
  }

  hadc->global_state = HAL_ADC_STATE_IDLE;

  return status;
}

/**
  * @brief  Perform self-calibration of ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_Calibrate(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, global_state, HAL_ADC_STATE_ACTIVE, HAL_ADC_STATE_CALIB);

  status = adc_calibrate(hadc);

  if (status == HAL_OK)
  {
    hadc->global_state = HAL_ADC_STATE_ACTIVE;
  }

  return status;
}

/**
  * @brief  Get ADC instance calibration factors.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_calib Pointer to a hal_adc_calib_t structure containing calibration data.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_GetCalibrationFactor(hal_adc_handle_t *hadc, hal_adc_calib_t *p_calib)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_calib != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_calib == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, global_state, HAL_ADC_STATE_ACTIVE, HAL_ADC_STATE_CALIB);

  p_instance = ADC_GET_INSTANCE(hadc);

  p_calib->factors[0] = LL_ADC_GetCalibrationFactor(p_instance, LL_ADC_IN_SINGLE_ENDED);

  hadc->global_state = HAL_ADC_STATE_ACTIVE;

  return status;
}

/**
  * @brief  Set ADC instance calibration factors.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_calib Pointer to a hal_adc_calib_t structure containing calibration data.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_SetCalibrationFactor(hal_adc_handle_t *hadc, const hal_adc_calib_t *p_calib)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(p_calib != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_calib == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_RESET |
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, global_state, HAL_ADC_STATE_ACTIVE, HAL_ADC_STATE_CALIB);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_SetCalibrationFactor(p_instance, LL_ADC_IN_SINGLE_ENDED, p_calib->factors[0]);

  hadc->global_state = HAL_ADC_STATE_ACTIVE;

  return status;
}

/**
  * @brief  Poll for ADC event.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  event Value of hal_adc_event_t
  * @param  timeout_ms ADC conversion timeout value (unit: ms)
  * @note   HAL ADC state machine is not updated by this function
  *         (on the contrary to other polling functions: HAL_ADC_REG_PollForConv(), ...)
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_PollForEvent(hal_adc_handle_t *hadc, hal_adc_event_t event, uint32_t timeout_ms)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_EVENT(event));

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_IDLE | HAL_ADC_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);
  tickstart = HAL_GetTick();

  /* Wait until selected flag is raised */
  while (LL_ADC_IsActiveFlag(p_instance, (uint32_t)event) == 0UL)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_ADC_IsActiveFlag(p_instance, (uint32_t)event) == 0UL)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  LL_ADC_ClearFlag(p_instance, (uint32_t)event);

  return status;
}

/**
  * @brief  Start conversion on ADC group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StartConv(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE, HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  LL_ADC_REG_StartConversion(p_instance);

  return status;
}

/**
  * @brief  Start conversion on ADC group regular with interruption: default interruptions.
  *         Default interruptions used: end of unitary conversion, overrun. To use other interruptions,
  *         refer to HAL_ADC_REG_StartConv_IT_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   Callback functions "HAL_ADC_REG_...Callback()" (corresponding to these interruptions) will be triggered.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StartConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  status = HAL_ADC_REG_StartConv_IT_Opt(hadc, HAL_ADC_OPT_IT_REG_EOC | HAL_ADC_OPT_IT_REG_OVR);

  return status;
}

/**
  * @brief  Start conversion on ADC group regular with interruption: selected optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOSMP
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOC
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOS
  *         @arg @ref HAL_ADC_OPT_IT_REG_OVR
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  * @note   Callback functions "HAL_ADC_REG_...Callback()" (corresponding to these interruptions) will be triggered.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StartConv_IT_Opt(hal_adc_handle_t *hadc, uint32_t it_opt)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_REG(it_opt));

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE, HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Manage optional interruptions */
  LL_ADC_ClearFlag(p_instance, it_opt);
  LL_ADC_EnableIT(p_instance, it_opt);

  LL_ADC_REG_StartConversion(p_instance);

  return status;
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Start conversion on ADC group regular with data transfer by DMA.
  *         Default interruptions used: buffer half transfer and transfer complete. To use other interruptions,
  *         refer to HAL_ADC_REG_StartConv_DMA_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_data     Pointer to the data buffer (data transfer from ADC to buffer, through DMA).
  * @param  size_byte  Data buffer size (in bytes).
  * @note   Callback functions @ref HAL_ADC_REG_DataTransferHalfCallback and @ref HAL_ADC_REG_DataTransferCpltCallback
  *         (corresponding to these interruptions) will be triggered.
  * @note   This function configures automatically ADC data transfer by DMA mode limited or unlimited in function of
  *         DMA configuration one shot or circular mode.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StartConv_DMA(hal_adc_handle_t *hadc, uint8_t *p_data, uint32_t size_byte)
{
  hal_status_t status;

  status = HAL_ADC_REG_StartConv_DMA_Opt(hadc, p_data, size_byte, HAL_ADC_OPT_DMA_IT_HT);

  return status;
}

/**
  * @brief  Start conversion on ADC group regular with data transfer by DMA and selected optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  p_data     Pointer to the data buffer (data transfer from ADC to buffer, through DMA).
  * @param  size_byte  Data buffer size (in bytes).
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOSMP
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOS
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  *         @arg @ref HAL_ADC_OPT_DMA_IT_NONE
  *         @arg @ref HAL_ADC_OPT_DMA_IT_HT
  *         @arg @ref HAL_ADC_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *         @arg @ref HAL_ADC_OPT_DMA_IT_SILENT (1)
  *
  *         (1) If mode silent selected, then all other interruptions are disabled
  * @endif
  * @note   Callback functions "HAL_ADC_REG_DataTransfer...Callback" (corresponding to these interruptions) will
  *         be triggered.
  * @note   This function configure automatically ADC data transfer by DMA mode limited or unlimited in function of
  *         DMA configuration one shot or circular mode.
  * @note   Optional interruptions not applicable: HAL_ADC_OPT_IT_REG_EOC (flag cleared by DMA)
  *         and HAL_ADC_OPT_IT_REG_OVR (always enabled: overrun event is an error in case of DMA transfer)
  * @warning  Prerequisite: Function @ref HAL_ADC_REG_SetDMA() must be called prior to this function,
  *           to set link to DMA.
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StartConv_DMA_Opt(hal_adc_handle_t *hadc, uint8_t *p_data, uint32_t size_byte,
                                           uint32_t it_opt)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;
  hal_dma_handle_t *hdma;
  uint32_t hal_dma_opt_it = it_opt;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(hadc->hdma_reg != NULL); /* Pointer set by HAL_ADC_REG_SetDMA() */
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_REG_DMA(it_opt));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hadc->hdma_reg == NULL) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR], HAL_ADC_GROUP_STATE_IDLE);

#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
  if (it_opt == HAL_ADC_OPT_DMA_IT_SILENT)
  {
    HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_REGULAR],
                           HAL_ADC_GROUP_STATE_IDLE,
                           HAL_ADC_GROUP_STATE_ACTIVE_SILENT);
  }
  else
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
  {
    HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_REGULAR],
                           HAL_ADC_GROUP_STATE_IDLE,
                           HAL_ADC_GROUP_STATE_ACTIVE);
  }

  p_instance = ADC_GET_INSTANCE(hadc);

  hdma = hadc->hdma_reg;

  /* Set DMA channel callback functions pointers */
  hdma->p_xfer_error_cb     = adc_reg_dma_data_transfer_error_callback;
  hdma->p_xfer_halfcplt_cb  = adc_reg_dma_data_transfer_half_callback;
  hdma->p_xfer_cplt_cb      = adc_reg_dma_data_transfer_cplt_callback;

  /* Manage optional interruptions specific to HAL DMA */
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
  if (hal_dma_opt_it != HAL_ADC_OPT_DMA_IT_SILENT)
  {
    hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
  }
  ASSERT_DBG_PARAM(IS_ADC_DMA_VALID_SILENT_MODE(hadc, hal_dma_opt_it));
#else
  hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */

  /* Start DMA transfer in IT mode */
  status = HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                          LL_ADC_DMA_GetRegAddr(p_instance, LL_ADC_DMA_REG_REGULAR_DATA),
                                          (uint32_t)p_data, size_byte, hal_dma_opt_it);

  if (status != HAL_OK)
  {
    status = HAL_ERROR;
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    hadc->last_error_codes |= HAL_ADC_REG_ERROR_DMA;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */

    /* Start operation aborted, restore state machine initial state */
    hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
  }
  else
  {
    uint32_t dma_mode;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    if (hdma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
    {
      dma_mode = 1U;
    }
    else
#endif /* USE_HAL_DMA_LINKEDLIST */
    {
      dma_mode = 0U;
    }

    if (dma_mode == 1U)
    {
      LL_ADC_REG_SetDataTransferMode(p_instance, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    }
    else
    {
      LL_ADC_REG_SetDataTransferMode(p_instance, LL_ADC_REG_DMA_TRANSFER_LIMITED);
    }

    /* Clear flags */
    LL_ADC_ClearFlag(p_instance, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);

    /* Enable interruptions */
    LL_ADC_EnableIT_OVR(p_instance);

    /* Manage optional interruptions */
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
    if (it_opt != HAL_ADC_OPT_DMA_IT_SILENT)
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
    {
      LL_ADC_ClearFlag(p_instance, (it_opt & LL_ADC_FLAG_ALL));
      LL_ADC_EnableIT(p_instance, (it_opt & LL_ADC_IT_ALL));
    }

    LL_ADC_REG_StartConversion(p_instance);
  }

  return status;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Trigger conversion (SW start) on ADC group regular for a conversion process ongoing.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   This function can be used to iterate a conversion process ongoing initiated by HAL_ADC_REG_StartConv...()
  *         (for example, sequence in discontinuous mode or DMA transfer from unitary conversions by SW start).
  * @warning Necessary condition: previous conversion must be completed (state HAL_ADC_GROUP_STATE_IDLE, ensured by
  *         polling (HAL_ADC_REG_PollForConv()) or interruption (HAL_ADC_IRQHandler()).
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_TrigNextConv(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance = ADC_GET_INSTANCE(hadc);
  ASSERT_DBG_PARAM(LL_ADC_REG_IsTriggerSourceSWStart(p_instance) != 0U);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE_SILENT |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);
#else
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);
#endif /* USE_HAL_ADC_DMA */

  if (LL_ADC_REG_IsConversionOngoing(p_instance) != 0U)
  {
    status = HAL_ERROR;
  }
  else
  {
    if (hadc->group_state[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_STATE_IDLE)
    {
      HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_REGULAR],
                             HAL_ADC_GROUP_STATE_IDLE,
                             HAL_ADC_GROUP_STATE_ACTIVE);
    }

    LL_ADC_REG_StartConversion(p_instance);
  }

  return status;
}

/**
  * @brief  Stop conversion on ADC group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StopConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);

  status = adc_reg_stop_conversion(hadc);

  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

/**
  * @brief  Stop conversion on ADC group regular with interruption.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   This function disable interruptions enabled by start conversion function except
  *         analog watchdog interrutpions possibly used by group injected. To disable them, use @ref HAL_ADC_Stop().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StopConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  status = adc_reg_stop_conversion(hadc);

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance, LL_ADC_IT_EOSMP | LL_ADC_IT_EOC | LL_ADC_IT_EOS | LL_ADC_IT_OVR);

  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Stop conversion on ADC group regular with data transfer by DMA.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   This function disable interruptions enabled by start conversion function except
  *         analog watchdog interrutpions possibly used by group injected. To disable them, use @ref HAL_ADC_Stop().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_StopConv_DMA(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_REGULAR],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE_SILENT);

  p_instance = ADC_GET_INSTANCE(hadc);

  status = adc_reg_stop_conversion(hadc);

  LL_ADC_REG_SetDataTransferMode(p_instance, LL_ADC_REG_DR_TRANSFER);

  if (hadc->group_state[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_STATE_ACTIVE_SILENT)
  {
    (void)HAL_DMA_Abort(hadc->hdma_reg);
    /* Error flag clearing when error in silent mode */

    adc_reg_dma_data_transfer_stop_callback(hadc->hdma_reg);
  }
  else
  {
    hadc->hdma_reg->p_xfer_abort_cb = adc_reg_dma_data_transfer_stop_callback;

    if (HAL_DMA_Abort_IT(hadc->hdma_reg) != HAL_OK)
    {
      adc_reg_dma_data_transfer_stop_callback(hadc->hdma_reg);
    }
  }

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance, LL_ADC_IT_EOSMP | LL_ADC_IT_EOC | LL_ADC_IT_EOS | LL_ADC_IT_OVR);

  /* Note: HAL ADC state machine is updated in function adc_reg_dma_data_transfer_stop_callback() */

  return status;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Wait for conversion on ADC group regular to be completed.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  timeout_ms ADC conversion timeout value (unit: ms)
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_REG_PollForConv(hal_adc_handle_t *hadc, uint32_t timeout_ms)
{
  ADC_TypeDef *p_instance;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);
  tickstart = HAL_GetTick();

  /* Wait until end of unitary conversion flag is raised */
  while (LL_ADC_IsActiveFlag_EOC(p_instance) == 0UL)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_ADC_IsActiveFlag_EOC(p_instance) == 0UL)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /* Clear flag */
  LL_ADC_ClearFlag_EOC(p_instance);

  if (hadc->group_conv_per_start[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_CONV_UNIT)
  {
    hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
  }

  return HAL_OK;
}

/**
  * @brief  Get ADC group regular conversion data.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval conversion data (signed value (can be negative after post-processing computation: offset feature)
  *         between Min_Data=-2147483648 (two's complement 0x80000000) and Max_Data=+2147483647 (0x7FFFFFFF))
  */
int32_t HAL_ADC_REG_ReadConversionData(const hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                                         | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_CALIB
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);
  int32_t data = LL_ADC_REG_ReadConversionData(p_instance);

  return data;
}

/**
  * @brief  Start conversion on ADC group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_StartConv(hal_adc_handle_t *hadc)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED], HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_INJECTED], HAL_ADC_GROUP_STATE_IDLE, HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  ASSERT_DBG_PARAM(LL_ADC_INJ_GetTrigAuto(p_instance) != LL_ADC_INJ_TRIG_FROM_REGULAR);

  LL_ADC_INJ_StartConversion(p_instance);

  return HAL_OK;
}

/**
  * @brief  Start conversion on ADC group injected with interruption: default interruptions.
  *         Default interruptions used: end of unitary conversion. To use other interruptions,
  *         refer to HAL_ADC_INJ_StartConv_IT_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   Callback functions "HAL_ADC_INJ_...Callback()" (corresponding to these interruptions) will be triggered.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_StartConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  status = HAL_ADC_INJ_StartConv_IT_Opt(hadc, HAL_ADC_OPT_IT_INJ_EOC);

  return status;
}

/**
  * @brief  Start conversion on ADC group injected with interruption: selected optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_INJ_EOC
  *         @arg @ref HAL_ADC_OPT_IT_INJ_EOS
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  * @note   Callback functions "HAL_ADC_INJ_...Callback()" (corresponding to these interruptions) will be triggered.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_StartConv_IT_Opt(hal_adc_handle_t *hadc, uint32_t it_opt)
{
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_INJ(it_opt));

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED], HAL_ADC_GROUP_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_INJECTED], HAL_ADC_GROUP_STATE_IDLE, HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  ASSERT_DBG_PARAM(LL_ADC_INJ_GetTrigAuto(p_instance) != LL_ADC_INJ_TRIG_FROM_REGULAR);

  /* Manage optional interruptions */
  LL_ADC_ClearFlag(p_instance, it_opt);
  LL_ADC_EnableIT(p_instance, it_opt);

  LL_ADC_INJ_StartConversion(p_instance);

  return HAL_OK;
}

/**
  * @brief  Trigger conversion (SW start) on ADC group injected for a conversion process ongoing.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   This function can be used to iterate a conversion process ongoing initiated by HAL_ADC_INJ_StartConv...()
  *         (for example, sequence in discontinuous mode).
  * @warning Necessary condition: previous conversion must be completed (state HAL_ADC_GROUP_STATE_IDLE, ensured by
  *         polling (HAL_ADC_INJ_PollForConv()) or interruption (HAL_ADC_IRQHandler()).
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_TrigNextConv(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance = ADC_GET_INSTANCE(hadc);
  ASSERT_DBG_PARAM(LL_ADC_INJ_IsTriggerSourceSWStart(p_instance) != 0U);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);

  if (LL_ADC_INJ_IsConversionOngoing(p_instance) != 0U)
  {
    status = HAL_ERROR;
  }
  else
  {
    if (hadc->group_state[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_STATE_IDLE)
    {
      HAL_CHECK_UPDATE_STATE(hadc, group_state[ADC_GROUP_INJECTED],
                             HAL_ADC_GROUP_STATE_IDLE,
                             HAL_ADC_GROUP_STATE_ACTIVE);
    }

    LL_ADC_INJ_StartConversion(p_instance);
  }

  return status;
}

/**
  * @brief  Stop conversion on ADC group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_StopConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);

  status = adc_inj_stop_conversion(hadc);

  hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

/**
  * @brief  Stop conversion on ADC group injected with interruption.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @note   This function disable interruptions enabled by start conversion function except
  *         analog watchdog interrutpions possibly used by group regular. To disable them, use @ref HAL_ADC_Stop().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_StopConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);
  ASSERT_DBG_STATE(hadc->group_state[ADC_GROUP_INJECTED],
                   (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                   (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  status = adc_inj_stop_conversion(hadc);

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance, LL_ADC_IT_JEOC | LL_ADC_IT_JEOS);

  hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;

  return status;
}

/**
  * @brief  Wait for conversion on ADC group injected to be completed.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  timeout_ms ADC conversion timeout value (unit: ms)
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_INJ_PollForConv(hal_adc_handle_t *hadc, uint32_t timeout_ms)
{
  ADC_TypeDef *p_instance;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hadc != NULL);

  ASSERT_DBG_STATE(hadc->global_state, HAL_ADC_STATE_ACTIVE);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Get tick count */
  tickstart = HAL_GetTick();

  /* Wait until end of unitary conversion flag is raised */
  while (LL_ADC_IsActiveFlag_JEOC(p_instance) == 0UL)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_ADC_IsActiveFlag_JEOC(p_instance) == 0UL)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /* Clear flag */
  LL_ADC_ClearFlag_JEOC(p_instance);

  if (hadc->group_conv_per_start[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_CONV_UNIT)
  {
    hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
  }

  return HAL_OK;
}

/**
  * @brief  Get ADC group injected conversion data.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  sequencer_rank ADC group injected sequencer rank.
  *                        Can be a number between Min_Data = 1, Max_Data = 4
  * @retval conversion data (signed value (can be negative after post-processing computation: offset feature)
  *         between Min_Data=-2147483648 (two's complement 0x80000000) and Max_Data=+2147483647 (0x7FFFFFFF))
  */
int32_t HAL_ADC_INJ_ReadConversionDataRank(const hal_adc_handle_t *hadc, uint8_t sequencer_rank)
{
  ADC_TypeDef *p_instance;
  uint32_t sequencer_rank_ll_format;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_INJ_SEQUENCER_LENGTH(sequencer_rank));
  ASSERT_DBG_STATE(hadc->global_state, (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                                         | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                                         | (uint32_t)HAL_ADC_STATE_CALIB
                                                         | (uint32_t)HAL_ADC_STATE_IDLE
                                                         | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance = ADC_GET_INSTANCE(hadc);
  sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_INJ_SEQ_RANK(sequencer_rank);
  int32_t data = LL_ADC_INJ_ReadConversionDataRank(p_instance, sequencer_rank_ll_format);

  return data;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Multimode operation: Activate ADC instances part of multimode.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_Start(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  hal_adc_handle_t *p_handle_current;
  uint32_t index;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance) == p_instance);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_IDLE);

  adc_mm_set_state_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
  {
    status = adc_mm_check_set_state_group(hadc, ADC_GROUP_INJECTED,
                                          HAL_ADC_GROUP_STATE_IDLE,
                                          HAL_ADC_GROUP_STATE_ACTIVE);
  }

  if (status == HAL_OK)
  {
    /* Verify configuration compliance to hardware constraints */
    if (hadc->group_state[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_STATE_IDLE)
    {
      if (LL_ADC_GetSamplingMode(p_instance) != LL_ADC_SAMPLING_MODE_NORMAL)
      {
        status = HAL_ERROR;
      }
    }

    if (status == HAL_OK)
    {
      /* Parse multimode instances through links of daisy chain (starting from ADC master) */
      p_handle_current = hadc;
      for (index = 0; index < ADC_MM_INST_COUNT; index++)
      {
        status = adc_activate(p_handle_current);

        if (status != HAL_OK)
        {
          break;
        }

        p_handle_current = p_handle_current->p_link_next_handle;
      }
    }
  }

  return status;
}

/**
  * @brief  Multimode operation: Deactivate ADC instances part of multimode.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_Stop(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance;
  hal_adc_handle_t *p_handle_current;
  uint32_t index;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance) == p_instance);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);

  if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
  {
    adc_assert_state_mm_inj(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_RESET |
                                                          (uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                          (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));
  }
  else
  {
    adc_assert_state_mm_inj(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_RESET |
                                                          (uint32_t)HAL_ADC_GROUP_STATE_IDLE));
  }

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  p_handle_current = hadc;
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    status = adc_deactivate(p_handle_current);

    /* Clear flags and interruptions */
    LL_ADC_ClearFlag(ADC_GET_INSTANCE(p_handle_current), LL_ADC_FLAG_ALL);
    LL_ADC_DisableIT(ADC_GET_INSTANCE(p_handle_current), LL_ADC_IT_ALL);

    if (status != HAL_OK)
    {
      break;
    }

    p_handle_current = p_handle_current->p_link_next_handle;
  }

  if (LL_ADC_INJ_GetTrigAuto(p_instance) == LL_ADC_INJ_TRIG_FROM_REGULAR)
  {
    adc_mm_set_state_inst_inj(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_IDLE, HAL_ADC_GROUP_STATE_IDLE);
  }
  else
  {
    adc_mm_set_state_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_IDLE);
  }

  return status;
}

/**
  * @brief  Multimode operation: Perform self-calibration of ADC instances part of multimode.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_Calibrate(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  hal_adc_handle_t *p_handle_current;
  uint32_t index;

  ASSERT_DBG_PARAM(hadc != NULL);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(ADC_GET_INSTANCE(hadc)) == ADC_GET_INSTANCE(hadc));

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);
  adc_assert_state_mm_inj(hadc, HAL_ADC_GROUP_STATE_IDLE);

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  p_handle_current = hadc;
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    HAL_CHECK_UPDATE_STATE(p_handle_current, global_state, HAL_ADC_STATE_ACTIVE, HAL_ADC_STATE_CALIB);

    status = adc_calibrate(p_handle_current);

    if (status != HAL_OK)
    {
      break;
    }

    p_handle_current = p_handle_current->p_link_next_handle;
  }

  adc_mm_set_state_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular.
  *         Callback functions "HAL_ADC_INJ_...Callback" (depending on ADC interrupt selected) will be triggered.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance) == p_instance);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);

  status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                        HAL_ADC_GROUP_STATE_IDLE,
                                        HAL_ADC_GROUP_STATE_ACTIVE);

  if (status == HAL_OK)
  {
    LL_ADC_REG_StartConversion(p_instance);
  }

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with interruption: default
  *         interruptions.
  *         Default interruptions used: end of unitary conversion, overrun. To use other interruptions,
  *         refer to HAL_ADC_REG_StartConv_IT_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   Callback functions "HAL_ADC_REG_...Callback()" (corresponding to these interruptions) will be triggered.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  status = HAL_ADC_MM_REG_StartConv_IT_Opt(hadc, (HAL_ADC_OPT_IT_REG_EOC | HAL_ADC_OPT_IT_REG_OVR));

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with interruption: selected
  *         optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOSMP
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOC
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOS
  *         @arg @ref HAL_ADC_OPT_IT_REG_OVR
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  * @note   Callback functions "HAL_ADC_REG_...Callback()" (corresponding to these interruptions) will be triggered.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConv_IT_Opt(hal_adc_handle_t *hadc, uint32_t it_opt)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_REG(it_opt));

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);

  status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                        HAL_ADC_GROUP_STATE_IDLE,
                                        HAL_ADC_GROUP_STATE_ACTIVE);

  if (status == HAL_OK)
  {
    /* Manage optional interruptions */
    LL_ADC_ClearFlag(p_instance_mst, it_opt);
    LL_ADC_ClearFlag(p_instance_slv, it_opt);
    LL_ADC_EnableIT(p_instance_mst, it_opt);

    LL_ADC_REG_StartConversion(p_instance_mst);
  }

  return status;
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with data transfer by DMA.
  *         In this mode, multimode conversion data of all ADC instances part of multimode are concatenated and
  *         transferred using only one DMA channel (the one assigned to ADC master).
  *         Default interruptions used: buffer half transfer and transfer complete. To use other interruptions,
  *         refer to HAL_ADC_MM_REG_StartConv_DMA_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  p_data     Pointer to the data buffer (data transfer from ADC to buffer, through DMA).
  * @param  size_byte  Data buffer size (in bytes).
  * @note   Callback functions @ref HAL_ADC_REG_DataTransferHalfCallback and @ref HAL_ADC_REG_DataTransferCpltCallback
  *         (corresponding to these interruptions) will be triggered.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConv_DMA(hal_adc_handle_t *hadc, uint8_t *p_data, uint32_t size_byte)
{
  hal_status_t status;

  status = HAL_ADC_MM_REG_StartConv_DMA_Opt(hadc, p_data, size_byte, HAL_ADC_OPT_DMA_IT_HT);

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with data transfer by DMA
  *         and selected optional interruptions.
  *         In this mode, multimode conversion data of all ADC instances part of multimode are concatenated and
  *         transferred using only one DMA channel (the one assigned to ADC master).
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  p_data     Pointer to the data buffer (data transfer from ADC to buffer, through DMA).
  * @param  size_byte  Data buffer size (in bytes).
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOSMP
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOS
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  *         @arg @ref HAL_ADC_OPT_DMA_IT_NONE
  *         @arg @ref HAL_ADC_OPT_DMA_IT_HT
  *         @arg @ref HAL_ADC_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *         @arg @ref HAL_ADC_OPT_DMA_IT_SILENT (1)
  *
  *         (1) If mode silent selected, then all other interruptions are disabled
  * @endif
  * @note   Callback functions "HAL_ADC_REG_DataTransfer...Callback" (corresponding to these interruptions) will
  *         be triggered.
  * @note   Function @ref HAL_ADC_REG_SetDMA() must be called prior to this function, to set link to DMA.

  * @note   Optional interruptions not applicable: HAL_ADC_OPT_IT_REG_EOC (flag cleared by DMA)
  *         and HAL_ADC_OPT_IT_REG_OVR (always enabled: overrun event is an error in case of DMA transfer)
  * @warning  Prerequisite: Function @ref HAL_ADC_REG_SetDMA() must be called prior to this function,
  *           to set link to DMA.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_INVALID_PARAM Invalid parameter
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConv_DMA_Opt(hal_adc_handle_t *hadc, uint8_t *p_data, uint32_t size_byte,
                                              uint32_t it_opt)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;
  hal_dma_handle_t *hdma;
  uint32_t hal_dma_opt_it = it_opt;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(hadc->hdma_reg != NULL); /* Pointer set by HAL_ADC_REG_SetDMA() */
  ASSERT_DBG_PARAM(p_data != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_REG_DMA(it_opt));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hadc->hdma_reg == NULL) || (p_data == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);

#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
  if (it_opt == HAL_ADC_OPT_DMA_IT_SILENT)
  {
    status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                          HAL_ADC_GROUP_STATE_IDLE,
                                          HAL_ADC_GROUP_STATE_ACTIVE_SILENT);
  }
  else
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
  {
    status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                          HAL_ADC_GROUP_STATE_IDLE,
                                          HAL_ADC_GROUP_STATE_ACTIVE);
  }

  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);
  hdma = hadc->hdma_reg;

  /* Set DMA channel callback functions pointers */
  hdma->p_xfer_error_cb     = adc_reg_dma_data_transfer_error_callback;
  hdma->p_xfer_halfcplt_cb  = adc_reg_dma_data_transfer_half_callback;
  hdma->p_xfer_cplt_cb      = adc_reg_dma_data_transfer_cplt_callback;

  if (status == HAL_OK)
  {
    /* Manage optional interruptions specific to HAL DMA */
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
    if (hal_dma_opt_it != HAL_ADC_OPT_DMA_IT_SILENT)
    {
      hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
    }
    ASSERT_DBG_PARAM(IS_ADC_DMA_VALID_SILENT_MODE(hadc, hal_dma_opt_it));
#else
    hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */

    /* Start DMA transfer in IT mode */
    status = HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                            LL_ADC_DMA_GetRegAddr(p_instance_mst,
                                                                  (uint32_t)hadc->mm_reg_data_transfer_packing),
                                            (uint32_t)p_data, size_byte, hal_dma_opt_it);
  }
  else
  {
    status = HAL_ERROR;
  }

  if (status != HAL_OK)
  {
    status = HAL_ERROR;
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    hadc->last_error_codes |= HAL_ADC_REG_ERROR_DMA;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
  }
  else
  {
    uint32_t dma_mode;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
    if (hdma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
    {
      dma_mode = 1U;
    }
    else
#endif /* USE_HAL_DMA_LINKEDLIST */
    {
      dma_mode = 0U;
    }

    if (dma_mode == 1U)
    {
      LL_ADC_REG_SetDataTransferMode(p_instance_mst, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    }
    else
    {
      LL_ADC_REG_SetDataTransferMode(p_instance_mst, LL_ADC_REG_DMA_TRANSFER_LIMITED);
    }

    /* Clear flags */
    LL_ADC_ClearFlag(p_instance_mst, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);
    LL_ADC_ClearFlag(p_instance_slv, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);

    /* Enable interruptions */
    LL_ADC_EnableIT_OVR(p_instance_mst);
    LL_ADC_EnableIT_OVR(p_instance_slv);

    /* Manage optional interruptions */
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
    if (it_opt != HAL_ADC_OPT_DMA_IT_SILENT)
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
    {
      LL_ADC_ClearFlag(p_instance_mst, (it_opt & LL_ADC_FLAG_ALL));
      LL_ADC_EnableIT(p_instance_mst, (it_opt & LL_ADC_IT_ALL));
    }

    LL_ADC_REG_StartConversion(p_instance_mst);
  }

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with data transfer by DMA
  *         using multiple DMA channels ("M_DMA" stands for multiple DMA).
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   In this mode, multimode conversion data of all ADC instances part of multimode are
  *         transferred using multiple DMA channels (the one assigned to each ADC).
  *         DMA transfer must have been configured using @ref HAL_ADC_MM_REG_SetMultiDMA().
  *         Default interruptions used: buffer half transfer and transfer complete. To use other interruptions,
  *         refer to @ref HAL_ADC_MM_REG_StartConvM_DMA_Opt().
  * @note   Callback functions "HAL_ADC_REG_DataTransfer...Callback" (corresponding to these interruptions) will
  *         be triggered.
  * @note   To stop conversion data and transfer by DMA of all ADC instances part of multimode use function
  *         @ref HAL_ADC_MM_REG_StopConv_DMA(). To restart new conversion DMA transfer must have been re-configured
  *         using @ref HAL_ADC_MM_REG_SetMultiDMA() before calling this function.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConvM_DMA(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  status = HAL_ADC_MM_REG_StartConvM_DMA_Opt(hadc, HAL_ADC_OPT_DMA_IT_NONE);

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group regular with data transfer by DMA
  *         using multiple DMA channels ("M_DMA" stands for multiple DMA), and selected optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOSMP
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOC
  *         @arg @ref HAL_ADC_OPT_IT_REG_EOS
  *         @arg @ref HAL_ADC_OPT_IT_REG_OVR
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  *         @arg @ref HAL_ADC_OPT_DMA_IT_NONE
  *         @arg @ref HAL_ADC_OPT_DMA_IT_HT
  *         @arg @ref HAL_ADC_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *         @arg @ref HAL_ADC_OPT_DMA_IT_SILENT (1)
  *
  *         (1) If mode silent selected, then all other interruptions are disabled
  * @endif
  * @note   In this mode, multimode conversion data of all ADC instances part of multimode are
  *         transferred using multiple DMA channels (the one assigned to each ADC).
  *         DMA transfer must have been configured using @ref HAL_ADC_MM_REG_SetMultiDMA().
  * @note   Callback functions "HAL_ADC_REG_DataTransfer...Callback" (corresponding to these interruptions) will
  *         be triggered.
  * @note   Optional interruptions not applicable: HAL_ADC_OPT_IT_REG_EOC (flag cleared by DMA)
  *         and HAL_ADC_OPT_IT_REG_OVR (always enabled: overrun event is an error in case of DMA transfer)
  * @note   To stop conversion data and transfer by DMA of all ADC instances part of multimode use function
  *         @ref HAL_ADC_MM_REG_StopConv_DMA(). To restart new conversion DMA transfer must have been re-configured
  *         using @ref HAL_ADC_MM_REG_SetMultiDMA() before calling this function.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StartConvM_DMA_Opt(hal_adc_handle_t *hadc, uint32_t it_opt)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_REG_DMA(it_opt));

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, HAL_ADC_GROUP_STATE_IDLE);

#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
  if (it_opt == HAL_ADC_OPT_DMA_IT_SILENT)
  {
    status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                          HAL_ADC_GROUP_STATE_IDLE,
                                          HAL_ADC_GROUP_STATE_ACTIVE_SILENT);
  }
  else
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
  {
    status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                          HAL_ADC_GROUP_STATE_IDLE,
                                          HAL_ADC_GROUP_STATE_ACTIVE);
  }

  if ((it_opt & (HAL_ADC_OPT_DMA_IT_HT | HAL_ADC_OPT_DMA_IT_DEFAULT)) != 0U)
  {
    /* Manage optional interruptions specific to HAL DMA */
    uint32_t hal_dma_opt_it = it_opt;
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
    if (hal_dma_opt_it != HAL_ADC_OPT_DMA_IT_SILENT)
    {
      hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
    }
    ASSERT_DBG_PARAM(IS_ADC_DMA_VALID_SILENT_MODE(hadc, hal_dma_opt_it));
#else
    hal_dma_opt_it = ((it_opt >> HAL_ADC_OPT_DMA_SHIFT) & HAL_DMA_OPT_IT_DEFAULT);
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */

    /* Update DMA transfer interruptions selection */
    LL_DMA_DisableIT((DMA_Channel_TypeDef *)((uint32_t)hadc->hdma_reg->instance), LL_DMA_IT_ALL);
    LL_DMA_DisableIT((DMA_Channel_TypeDef *)((uint32_t)hadc->p_link_next_handle->hdma_reg->instance), LL_DMA_IT_ALL);
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
    if (hal_dma_opt_it != HAL_ADC_OPT_DMA_IT_SILENT)
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
    {
      LL_DMA_EnableIT((DMA_Channel_TypeDef *)((uint32_t)hadc->hdma_reg->instance),
                      (LL_DMA_IT_TC  | LL_DMA_IT_DTE | LL_DMA_IT_ULE | LL_DMA_IT_USE | hal_dma_opt_it));
      LL_DMA_EnableIT((DMA_Channel_TypeDef *)((uint32_t)hadc->p_link_next_handle->hdma_reg->instance),
                      (LL_DMA_IT_TC  | LL_DMA_IT_DTE | LL_DMA_IT_ULE | LL_DMA_IT_USE | hal_dma_opt_it));
    }
  }

  uint32_t dma_mode;
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (hadc->hdma_reg->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR)
  {
    dma_mode = 1U;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    dma_mode = 0U;
  }

  if (dma_mode == 1U)
  {
    LL_ADC_REG_SetDataTransferMode(p_instance_mst, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetDataTransferMode(p_instance_slv, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  }
  else
  {
    LL_ADC_REG_SetDataTransferMode(p_instance_mst, LL_ADC_REG_DMA_TRANSFER_LIMITED);
    LL_ADC_REG_SetDataTransferMode(p_instance_slv, LL_ADC_REG_DMA_TRANSFER_LIMITED);
  }

  /* Clear flags */
  LL_ADC_ClearFlag(p_instance_mst, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);
  LL_ADC_ClearFlag(p_instance_slv, LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS | LL_ADC_FLAG_OVR);

  /* Enable interruptions */
  LL_ADC_EnableIT_OVR(p_instance_mst);
  LL_ADC_EnableIT_OVR(p_instance_slv);

  /* Manage optional interruptions */
#if defined(HAL_ADC_OPT_DMA_IT_SILENT)
  if (it_opt != HAL_ADC_OPT_DMA_IT_SILENT)
#endif /* HAL_ADC_OPT_DMA_IT_SILENT */
  {
    LL_ADC_ClearFlag(p_instance_mst, (it_opt & LL_ADC_FLAG_ALL));
    LL_ADC_EnableIT(p_instance_mst, (it_opt & LL_ADC_IT_ALL));
  }

  LL_ADC_REG_StartConversion(p_instance_mst);

  return status;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Trigger conversion (SW start) on multimode ADC instances group regular for a conversion process ongoing.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   This function can be used to iterate a conversion process ongoing initiated
  *         by HAL_ADC_MM_REG_StartConv...() (for example, sequence in discontinuous mode or DMA transfer
  *         from unitary conversions by SW start).
  * @note   This function is not compliant with multimode regular interleaved.
  * @warning Necessary condition: previous conversion must be completed (state HAL_ADC_GROUP_STATE_IDLE, ensured by
  *          polling (HAL_ADC_REG_PollForConv()) or interruption (HAL_ADC_IRQHandler()).
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_TrigNextConv(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance_mst;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  ASSERT_DBG_PARAM(LL_ADC_REG_IsTriggerSourceSWStart(p_instance_mst) != 0U);

#if defined(USE_ASSERT_DBG_PARAM)
  uint32_t multimode = LL_ADC_GetMultimode(ADC_COMMON_INSTANCE(p_instance_mst));
  ASSERT_DBG_PARAM((multimode != LL_ADC_MULTI_DUAL_REG_INTERL) && (multimode != LL_ADC_MULTI_DUAL_REG_INT_INJ_SIM));
#endif /* USE_ASSERT_DBG_PARAM */

  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_reg(hadc, (hal_adc_group_state_t)
                          ((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
                           (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE_SILENT |
#endif /* USE_HAL_ADC_DMA */
                           (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  if (LL_ADC_REG_IsConversionOngoing(p_instance_mst) != 0U)
  {
    status = HAL_ERROR;
  }
  else
  {
    if (hadc->group_state[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_STATE_IDLE)
    {
      status = adc_mm_check_set_state_group(hadc, ADC_GROUP_REGULAR,
                                            HAL_ADC_GROUP_STATE_IDLE,
                                            HAL_ADC_GROUP_STATE_ACTIVE);
    }

    LL_ADC_REG_StartConversion(p_instance_mst);
  }

  return status;
}

/**
  * @brief  Multimode operation: Stop conversion on multimode ADC instances group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StopConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  /* Multimode conversions controlled by ADC master only */
  status = adc_reg_stop_conversion(hadc);

  /* Disable features enabled by all start conversion functions */
  LL_ADC_DisableIT_EOC(p_instance_mst);
  LL_ADC_DisableIT_EOS(p_instance_mst);

  adc_mm_set_state_inst_reg(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE, HAL_ADC_GROUP_STATE_IDLE);

  return status;
}

/**
  * @brief  Multimode operation: Stop conversion on multimode ADC instances group regular with interruption.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StopConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  /* Multimode conversions controlled by ADC master only */
  status = adc_reg_stop_conversion(hadc);

  /* Disable features enabled by all start conversion functions */
  LL_ADC_DisableIT_EOC(p_instance_mst);
  LL_ADC_DisableIT_EOS(p_instance_mst);

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance_mst, LL_ADC_IT_EOSMP | LL_ADC_IT_EOC | LL_ADC_IT_EOS | LL_ADC_IT_OVR);
  LL_ADC_DisableIT(p_instance_slv, LL_ADC_IT_OVR);

  adc_mm_set_state_inst_reg(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE, HAL_ADC_GROUP_STATE_IDLE);

  return status;
}

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  Multimode operation: Stop conversion on multimode ADC instances group regular with data transfer by DMA.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   This function disable interruptions enabled by start conversion function except
  *         analog watchdog interrutpions possibly used by group injected. To disable them, use @ref HAL_ADC_MM_Stop().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_StopConv_DMA(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;
  hal_adc_handle_t *hadc_stop_last;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_reg(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE_SILENT));

  /* Multimode conversions controlled by ADC master only */
  status = adc_reg_stop_conversion(hadc);

  /* Disable features enabled by all start conversion functions */
  LL_ADC_DisableIT_EOC(p_instance_mst);
  LL_ADC_DisableIT_EOS(p_instance_mst);

  LL_ADC_REG_SetDataTransferMode(p_instance_mst, LL_ADC_REG_DR_TRANSFER);
  LL_ADC_REG_SetDataTransferMode(p_instance_slv, LL_ADC_REG_DR_TRANSFER);

  /* Case multiple buffers (started by HAL_ADC_MM_REG_StartConvM_DMA() / M_DMA_Opt()) */
  if (hadc->p_link_next_handle->hdma_reg != NULL)
  {
    hadc_stop_last = hadc->p_link_next_handle;
    (void)HAL_DMA_Abort(hadc->hdma_reg);
  }
  else
  {
    hadc_stop_last = hadc;
  }

  if (hadc->group_state[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_STATE_ACTIVE_SILENT)
  {
    (void)HAL_DMA_Abort(hadc_stop_last->hdma_reg);
    /* Error flag clearing when error in silent mode */

    adc_mm_reg_dma_data_transfer_stop_callback(hadc_stop_last->hdma_reg);
  }
  else
  {
    hadc_stop_last->hdma_reg->p_xfer_abort_cb = adc_mm_reg_dma_data_transfer_stop_callback;

    if (HAL_DMA_Abort_IT(hadc_stop_last->hdma_reg) != HAL_OK)
    {
      adc_mm_reg_dma_data_transfer_stop_callback(hadc_stop_last->hdma_reg);
    }
  }

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance_mst, LL_ADC_IT_EOSMP | LL_ADC_IT_EOC | LL_ADC_IT_EOS | LL_ADC_IT_OVR);
  LL_ADC_DisableIT(p_instance_slv, LL_ADC_IT_OVR);

  /* Note: HAL ADC state machine is updated in function adc_reg_dma_data_transfer_stop_callback() */

  return status;
}
#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Multimode operation: Wait for conversion on ADC group regular to be completed
  *         (for all ADC instances part of multimode).
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  timeout_ms ADC conversion timeout value (unit: ms)
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_REG_PollForConv(hal_adc_handle_t *hadc, uint32_t timeout_ms)
{
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;
  uint32_t flag_eoc_mst;
  uint32_t flag_eoc_slv;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hadc != NULL);

  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Get tick count */
  tickstart = HAL_GetTick();

  /* Wait until end of unitary conversion flag is raised */
  flag_eoc_mst = LL_ADC_IsActiveFlag_EOC(p_instance_mst);
  flag_eoc_slv = LL_ADC_IsActiveFlag_EOC(p_instance_slv);
  while ((flag_eoc_mst == 0UL) && (flag_eoc_slv == 0UL))
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      flag_eoc_mst = LL_ADC_IsActiveFlag_EOC(p_instance_mst);
      flag_eoc_slv = LL_ADC_IsActiveFlag_EOC(p_instance_slv);
      if ((flag_eoc_mst == 0UL) && (flag_eoc_slv == 0UL))
      {
        return HAL_TIMEOUT;
      }
    }
    flag_eoc_mst = LL_ADC_IsActiveFlag_EOC(p_instance_mst);
    flag_eoc_slv = LL_ADC_IsActiveFlag_EOC(p_instance_slv);
  }

  /* Clear flag */
  LL_ADC_ClearFlag_EOC(p_instance_mst);
  LL_ADC_ClearFlag_EOC(p_instance_slv);

  if (hadc->group_conv_per_start[ADC_GROUP_REGULAR] == HAL_ADC_GROUP_CONV_UNIT)
  {
    hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
    /* ADC slave state machine update */
    hadc->p_link_next_handle->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;
  }

  return HAL_OK;
}

/**
  * @brief  Multimode operation: Get ADC group regular conversion data of ADC master, ADC slave
  *         or ADC master and slave concatenated.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  multi_inst Value of hal_adc_mm_inst_t:
  *         @arg @ref HAL_ADC_MM_MASTER       (1)
  *         @arg @ref HAL_ADC_MM_SLAVE        (1)
  *         @arg @ref HAL_ADC_MM_MASTER_SLAVE
  *
  *         (1) Parameter available only if ADC multimode group regular data format with packing option on 32 bits
  *             (refer to @ref hal_adc_mm_reg_data_format_t).
  * @note   This function is relevant only for ADC multimode group regular data format with packing: each ADC conversion
  *         data concatenated in a single register (refer to @ref hal_adc_mm_reg_data_format_t).
  * @note   Each ADC conversion data width is limited to 8 or 16 bits depending on data packing setting.
  *         If expected data width is wider (this can be the case with features extending data width (oversampling,
  *         data shift,...), others services must be used:
  *         - function "HAL_ADC_REG_ReadConversionData()" for each ADC instance part of multimode.
  *         - multimode functions with data transfer by DMA.
  * @note   Returned value is unsigned, due to concatenation of multiple data.
  *         In case of signed data expected (with features changing data sign: offset),
  *         use function "HAL_ADC_REG_ReadConversionData()" for each ADC instance part of multimode.
  * @retval conversion data (unsigned value between Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF)
  */
uint32_t HAL_ADC_MM_REG_ReadConversionData(const hal_adc_handle_t *hadc, hal_adc_mm_inst_t multi_inst)
{
  ASSERT_DBG_PARAM(hadc != NULL);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(ADC_GET_INSTANCE(hadc)) == ADC_GET_INSTANCE(hadc));

  ASSERT_DBG_PARAM(LL_ADC_GetMultiDMATransfer(ADC_COMMON_INSTANCE(ADC_GET_INSTANCE(hadc)))
                   != LL_ADC_MULTI_REG_DMA_EACH_ADC);
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM,
                           (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                             | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                             | (uint32_t)HAL_ADC_STATE_CALIB
                                             | (uint32_t)HAL_ADC_STATE_IDLE
                                             | (uint32_t)HAL_ADC_STATE_ACTIVE));

  uint32_t data = LL_ADC_REG_ReadMultiConversionData32(ADC_COMMON_INSTANCE(ADC_GET_INSTANCE(hadc)),
                                                       (uint32_t)multi_inst);

  return data;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_StartConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_inj(hadc, HAL_ADC_GROUP_STATE_IDLE);

  status = adc_mm_check_set_state_group(hadc, ADC_GROUP_INJECTED,
                                        HAL_ADC_GROUP_STATE_IDLE,
                                        HAL_ADC_GROUP_STATE_ACTIVE);

  if (status == HAL_OK)
  {
    LL_ADC_INJ_StartConversion(p_instance_mst);
  }

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group injected with interruption: default
  *         interruptions.
  *         Default interruptions used: end of unitary conversion, overrun. To use other interruptions,
  *         refer to HAL_ADC_MM_INJ_StartConv_IT_Opt().
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   Callback functions "HAL_ADC_INJ_...Callback()" (corresponding to these interruptions) will be triggered.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_StartConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  status = HAL_ADC_MM_INJ_StartConv_IT_Opt(hadc, HAL_ADC_OPT_IT_INJ_EOC);

  return status;
}

/**
  * @brief  Multimode operation: Start conversion on multimode ADC instances group injected with interruption: selected
  *         optional interruptions.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  it_opt Can be a combination of values (subset of @ref ADC_optional_interruptions):
  *         @arg @ref HAL_ADC_OPT_IT_NONE
  *         @arg @ref HAL_ADC_OPT_IT_INJ_EOC
  *         @arg @ref HAL_ADC_OPT_IT_INJ_EOS
  *         @arg @ref HAL_ADC_OPT_IT_AWD_1
  *         @arg @ref HAL_ADC_OPT_IT_AWD_2
  *         @arg @ref HAL_ADC_OPT_IT_AWD_3
  * @note   Callback functions "HAL_ADC_INJ_...Callback()" (corresponding to these interruptions) will be triggered.
  * @warning  Prerequisite: HAL ADC handles part of multimode setup must have been linked using function
  *           @ref HAL_ADC_SetLinkNextHandle()
  *           and multimode must have been configured using function @ref HAL_ADC_MM_SetConfig().
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_StartConv_IT_Opt(hal_adc_handle_t *hadc, uint32_t it_opt)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;

  ASSERT_DBG_PARAM(hadc != NULL);
  ASSERT_DBG_PARAM(IS_ADC_OPT_IT_INJ(it_opt));

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_inj(hadc, HAL_ADC_GROUP_STATE_IDLE);

  status = adc_mm_check_set_state_group(hadc, ADC_GROUP_INJECTED,
                                        HAL_ADC_GROUP_STATE_IDLE,
                                        HAL_ADC_GROUP_STATE_ACTIVE);

  if (status == HAL_OK)
  {
    /* Manage optional interruptions */
    LL_ADC_ClearFlag(p_instance_mst, it_opt);
    LL_ADC_ClearFlag(p_instance_slv, it_opt);
    LL_ADC_EnableIT(p_instance_mst, it_opt);

    LL_ADC_INJ_StartConversion(p_instance_mst);
  }

  return status;
}

/**
  * @brief  Trigger conversion (SW start) on multimode ADC instances group injected for a conversion process ongoing.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   This function can be used to iterate a conversion process ongoing initiated
  *         by HAL_ADC_MM_INJ_StartConv...() (for example, sequence in discontinuous mode or DMA transfer
  *         from unitary conversions by SW start).
  * @warning Necessary condition: previous conversion must be completed (state HAL_ADC_GROUP_STATE_IDLE, ensured by
  *         polling (HAL_ADC_INJ_PollForConv()) or interruption (HAL_ADC_IRQHandler()).
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_TrigNextConv(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance_mst;

  ASSERT_DBG_PARAM(hadc != NULL);

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  ASSERT_DBG_PARAM(LL_ADC_INJ_IsTriggerSourceSWStart(p_instance_mst) != 0U);

  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inj(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  if (LL_ADC_INJ_IsConversionOngoing(p_instance_mst) != 0U)
  {
    status = HAL_ERROR;
  }
  else
  {
    if (hadc->group_state[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_STATE_IDLE)
    {
      status = adc_mm_check_set_state_group(hadc, ADC_GROUP_INJECTED,
                                            HAL_ADC_GROUP_STATE_IDLE,
                                            HAL_ADC_GROUP_STATE_ACTIVE);
    }

    LL_ADC_INJ_StartConversion(p_instance_mst);
  }

  return status;
}

/**
  * @brief  Multimode operation: Stop conversion on multimode ADC instances group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_StopConv(hal_adc_handle_t *hadc)
{
  hal_status_t status;

  ASSERT_DBG_PARAM(hadc != NULL);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_inj(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(ADC_GET_INSTANCE(hadc)) == ADC_GET_INSTANCE(hadc));

  /* Multimode conversions controlled by ADC master only */
  status = adc_inj_stop_conversion(hadc);

  adc_mm_set_state_inst_inj(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE, HAL_ADC_GROUP_STATE_IDLE);

  return status;
}

/**
  * @brief  Multimode operation: Stop conversion on multimode ADC instances group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @note   This function disable interruptions enabled by start conversion function except
  *         analog watchdog interrutpions possibly used by group regular. To disable them, use @ref HAL_ADC_Stop().
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_StopConv_IT(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance_mst;

  ASSERT_DBG_PARAM(hadc != NULL);

  /* Check state of all HAL ADC handles part of multimode */
  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);
  adc_assert_state_mm_inj(hadc, (hal_adc_group_state_t)((uint32_t)HAL_ADC_GROUP_STATE_IDLE |
                                                        (uint32_t)HAL_ADC_GROUP_STATE_ACTIVE));

  p_instance_mst = ADC_GET_INSTANCE(hadc);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Multimode conversions controlled by ADC master only */
  status = adc_inj_stop_conversion(hadc);

  /* Disable interruptions */
  LL_ADC_DisableIT(p_instance_mst, LL_ADC_IT_JEOC | LL_ADC_IT_JEOS);

  adc_mm_set_state_inst_inj(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE, HAL_ADC_GROUP_STATE_IDLE);

  return status;
}

/**
  * @brief  Multimode operation: Wait for conversion on ADC group injected to be completed
  *         (for all ADC instances part of multimode).
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  timeout_ms ADC conversion timeout value (unit: ms)
  * @retval HAL_TIMEOUT       Operation exceeds user timeout
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
hal_status_t HAL_ADC_MM_INJ_PollForConv(hal_adc_handle_t *hadc, uint32_t timeout_ms)
{
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;
  uint32_t flag_eoc_mst;
  uint32_t flag_eoc_slv;
  uint32_t tickstart;

  ASSERT_DBG_PARAM(hadc != NULL);

  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE);

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(p_instance_mst) == p_instance_mst);

  /* Get tick count */
  tickstart = HAL_GetTick();

  /* Wait until end of unitary conversion flag is raised */
  flag_eoc_mst = LL_ADC_IsActiveFlag_JEOC(p_instance_mst);
  flag_eoc_slv = LL_ADC_IsActiveFlag_JEOC(p_instance_slv);
  while ((flag_eoc_mst == 0UL) && (flag_eoc_slv == 0UL))
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      /* New check to avoid false timeout detection in case of preemption */
      flag_eoc_mst = LL_ADC_IsActiveFlag_JEOC(p_instance_mst);
      flag_eoc_slv = LL_ADC_IsActiveFlag_JEOC(p_instance_slv);
      if ((flag_eoc_mst == 0UL) && (flag_eoc_slv == 0UL))
      {
        return HAL_TIMEOUT;
      }
    }
    flag_eoc_mst = LL_ADC_IsActiveFlag_JEOC(p_instance_mst);
    flag_eoc_slv = LL_ADC_IsActiveFlag_JEOC(p_instance_slv);
  }

  /* Clear flag */
  LL_ADC_ClearFlag_JEOC(p_instance_mst);
  LL_ADC_ClearFlag_JEOC(p_instance_slv);

  if (hadc->group_conv_per_start[ADC_GROUP_INJECTED] == HAL_ADC_GROUP_CONV_UNIT)
  {
    hadc->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
    /* ADC slave state machine update */
    hadc->p_link_next_handle->group_state[ADC_GROUP_INJECTED] = HAL_ADC_GROUP_STATE_IDLE;
  }

  return HAL_OK;
}

/**
  * @brief  Multimode operation: Get ADC group injected conversion data of ADC master, ADC slave
  *         or ADC master and slave concatenated.
  * @param  hadc Pointer to a hal_adc_handle_t structure.
  * @param  multi_inst Value of hal_adc_mm_inst_t
  * @param  sequencer_rank ADC group injected sequencer rank.
  *                        Can be a number between Min_Data = 1, Max_Data = 4
  * @note   With ADC master and slave concatenated: Data width is limited to 16 bits (due to register 32 bits containing
  *         conversion data of two ADC instances: ADC master and slave data in ranges [15:0] and [31:16] respectively).
  *         If expected data width is wider (this can be the case with features extending data width (oversampling,
  *         data shift,...), others services must be used:
  *         - this function with parameter HAL_ADC_MM_MASTER or HAL_ADC_MM_SLAVE
  *         - function "HAL_ADC_INJ_ReadConversionDataRank()" for each ADC instance part of multimode.
  * @note   Returned value is unsigned, due to concatenation of multiple data.
  *         In case of signed data expected (with features changing data sign: offset),
  *         use function "HAL_ADC_INJ_ReadConversionDataRank()" for each ADC instance part of multimode.
  * @retval conversion data (unsigned value between Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF)
  */
uint32_t HAL_ADC_MM_INJ_ReadConversionDataRank(const hal_adc_handle_t *hadc, hal_adc_mm_inst_t multi_inst,
                                               uint8_t sequencer_rank)
{
  ADC_TypeDef *p_instance_mst;
  ADC_TypeDef *p_instance_slv;
  uint32_t sequencer_rank_ll_format;
  uint32_t data;

  ASSERT_DBG_PARAM(hadc != NULL);

  /* Check whether HAL ADC handle is the one of ADC master */
  ASSERT_DBG_PARAM(ADC_MULTI_INSTANCE_MASTER(ADC_GET_INSTANCE(hadc)) == ADC_GET_INSTANCE(hadc));

  adc_assert_state_mm_inst(hadc, HAL_ADC_COMMON_STATE_MM,
                           (hal_adc_state_t)((uint32_t)HAL_ADC_STATE_INIT
                                             | (uint32_t)HAL_ADC_STATE_CONFIGURING
                                             | (uint32_t)HAL_ADC_STATE_CALIB
                                             | (uint32_t)HAL_ADC_STATE_IDLE
                                             | (uint32_t)HAL_ADC_STATE_ACTIVE));

  p_instance_mst = ADC_GET_INSTANCE(hadc);
  p_instance_slv = ADC_GET_INSTANCE(hadc->p_link_next_handle);
  sequencer_rank_ll_format = LL_ADC_DECIMAL_NB_TO_INJ_SEQ_RANK(sequencer_rank);

  if (multi_inst == HAL_ADC_MM_MASTER)
  {
    data = LL_ADC_INJ_ReadConversionData32Rank(p_instance_mst, sequencer_rank_ll_format);
  }
  else if (multi_inst == HAL_ADC_MM_SLAVE)
  {
    data = LL_ADC_INJ_ReadConversionData32Rank(p_instance_slv, sequencer_rank_ll_format);
  }
  else
  {
    /* Concatenate data in 32 bits registers */
    data = LL_ADC_INJ_ReadConversionData32Rank(p_instance_mst, sequencer_rank_ll_format);
    data |= (LL_ADC_INJ_ReadConversionData32Rank(p_instance_slv, sequencer_rank_ll_format) << 16U);
  }

  return data;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @}
  */

/** @addtogroup ADC_Exported_Functions_Group6
  * @{
    This subsection provides functions allowing to:
      + Set a user data pointer (ex: a user context) in a HAL ADC handle,
      + Get a user data pointer (ex: a user context) from a HAL ADC handle.
    @note A typical usage is to set user data pointer before starting a data conversion,
          then retrieve it within the user conversion completion callback.
  */
#if defined(USE_HAL_ADC_USER_DATA) && (USE_HAL_ADC_USER_DATA == 1)
/**
  * @brief  Store  user data pointer into the adc handle.
  * @param  hadc Pointer to a hal_adc_handle_t.
  * @param  p_user_data Pointer to the user data.
  */
void HAL_ADC_SetUserData(hal_adc_handle_t *hadc, const void *p_user_data)
{
  ASSERT_DBG_PARAM(hadc != NULL);

  hadc->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve user data pointer from the adc handle.
  * @param  hadc Pointer to a hal_adc_handle_t.
  * @retval (void*) the pointer to the user data, when previously set by HAL_ADC_SetUserData().
  * @retval NULL other way.
  */
const void *HAL_ADC_GetUserData(const hal_adc_handle_t *hadc)
{
  ASSERT_DBG_PARAM(hadc != NULL);

  return (hadc->p_user_data);
}

#endif /* USE_HAL_ADC_USER_DATA */
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup ADC_Private_Functions
  * @{
  */

#if defined(USE_HAL_ADC_DMA) && (USE_HAL_ADC_DMA == 1)
/**
  * @brief  DMA data half transfer callback for ADC group regular.
  * @param  hdma pointer to a hal_dma_handle_t structure
  */
static void adc_reg_dma_data_transfer_half_callback(hal_dma_handle_t *hdma)
{
  hal_adc_handle_t *hadc = (hal_adc_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->p_reg_xfer_half_cb(hadc);
#else
  HAL_ADC_REG_DataTransferHalfCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback for ADC group regular.
  * @param  hdma pointer to a hal_dma_handle_t structure
  */
static void adc_reg_dma_data_transfer_cplt_callback(hal_dma_handle_t *hdma)
{
  hal_adc_handle_t *hadc = (hal_adc_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->p_reg_xfer_cplt_cb(hadc);
#else
  HAL_ADC_REG_DataTransferCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer stop callback for ADC group regular.
  * @param  hdma pointer to a hal_dma_handle_t structure
  */
static void adc_reg_dma_data_transfer_stop_callback(hal_dma_handle_t *hdma)
{
  hal_adc_handle_t *hadc = (hal_adc_handle_t *)(hdma->p_parent);

  hadc->group_state[ADC_GROUP_REGULAR] = HAL_ADC_GROUP_STATE_IDLE;

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->p_reg_xfer_stop_cb(hadc);
#else
  HAL_ADC_REG_DataTransferStopCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  DMA data transfer stop callback for ADC group regular.
  * @param  hdma pointer to a hal_dma_handle_t structure
  */
static void adc_mm_reg_dma_data_transfer_stop_callback(hal_dma_handle_t *hdma)
{
  hal_adc_handle_t *hadc = (hal_adc_handle_t *)(hdma->p_parent);

  adc_mm_set_state_inst_reg(hadc, HAL_ADC_COMMON_STATE_MM, HAL_ADC_STATE_ACTIVE, HAL_ADC_GROUP_STATE_IDLE);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->p_reg_xfer_stop_cb(hadc);
#else
  HAL_ADC_REG_DataTransferStopCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @brief  DMA data transfer error callback for ADC group regular.
  * @param  hdma pointer to a hal_dma_handle_t structure
  */
static void adc_reg_dma_data_transfer_error_callback(hal_dma_handle_t *hdma)
{
  hal_adc_handle_t *hadc = (hal_adc_handle_t *)(hdma->p_parent);

#if defined(USE_HAL_ADC_REGISTER_CALLBACKS) && (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->p_error_cb(hadc);
#else
  HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}

#endif /* USE_HAL_ADC_DMA */

/**
  * @brief  Activate the selected ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
static hal_status_t adc_activate(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance = ADC_GET_INSTANCE(hadc);
  uint32_t tickstart;
  uint32_t enabled_internal_channel;
  uint32_t internal_channel_stab_time_us;

  /* ADC enable and wait for ADC ready (in case of ADC disabled or enabling phase not yet completed: flag ADC ready
     not yet set).
     Timeout implemented to not be stuck if ADC cannot be enabled (possible causes: ADC clock not running, ...). */
  if (LL_ADC_IsEnabled(p_instance) == 0UL)
  {
    if (LL_ADC_IsInternalRegulatorEnabled(p_instance) == 0U)
    {
      /* Disable ADC deep power down (enabled by default after reset state) */
      LL_ADC_DisableDeepPowerDown(p_instance);

      /* Enable ADC internal voltage regulator */
      LL_ADC_EnableInternalRegulator(p_instance);

      /* Delay for ADC internal voltage regulator stabilization */
      ADC_DELAY_US(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
    }

    /* Check if conditions to enable the ADC are fulfilled */
    if ((p_instance->CR & (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART
                           | ADC_CR_ADDIS | ADC_CR_ADEN)) != 0UL)
    {
      status = HAL_ERROR;
    }
    else
    {
      LL_ADC_ClearFlag_ADRDY(p_instance);

      LL_ADC_Enable(p_instance);

      /* Wait for ADC effectively enabled */
      tickstart = HAL_GetTick();
      /* Poll for ADC ready flag raised */
      while (LL_ADC_IsActiveFlag_ADRDY(p_instance) == 0UL)
      {
        /* If ADEN bit is set less than 4 ADC clock cycles after the ADCAL bit has been cleared (after a calibration),
           ADEN bit is reset by the calibration logic.
           The workaround is to continue setting ADEN until ADRDY is becomes 1.
           Additionally, ADC_ENABLE_TIMEOUT_MS is defined to encompass this 4 ADC clock cycle duration */
        /* Note: Test of ADC enabled required due to hardware constraint to not enable ADC if already enabled. */
        if (LL_ADC_IsEnabled(p_instance) == 0UL)
        {
          LL_ADC_Enable(p_instance);
        }

        if ((HAL_GetTick() - tickstart) > ADC_ENABLE_TIMEOUT_MS)
        {
          /* New check to avoid false timeout detection in case of preemption */
          if (LL_ADC_IsActiveFlag_ADRDY(p_instance) == 0UL)
          {
            status = HAL_ERROR;
            break;
          }
        }
      }

      /* Delay for ADC internal channel voltage stabilization */
      enabled_internal_channel = LL_ADC_GetCommonPathInternalCh(ADC_COMMON_INSTANCE(p_instance));
      if ((enabled_internal_channel & (LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR)) != 0UL)
      {
        if ((enabled_internal_channel & LL_ADC_PATH_INTERNAL_TEMPSENSOR) != 0UL)
        {
          /* Case single internal channel enabled (temperature sensor) or multiple internal channels */
          /* Note: Temperature sensor stabilization delay encompasses VrefInt stabilization delay */
          internal_channel_stab_time_us = LL_ADC_DELAY_TEMPSENSOR_STAB_US;
        }
        else
        {
          /* Case single internal channel enabled (VrefInt) */
          internal_channel_stab_time_us = LL_ADC_DELAY_VREFINT_STAB_US;
        }

        /* Delay for ADC internal channel stabilization */
        ADC_DELAY_US(internal_channel_stab_time_us);
      }
    }

#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    if (status != HAL_OK)
    {
      hadc->last_error_codes |= HAL_ADC_ERROR_INTERNAL;
    }
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
  }

  return status;
}

/**
  * @brief  Deactivate the selected ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
static hal_status_t adc_deactivate(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance = ADC_GET_INSTANCE(hadc);
  uint32_t tickstart;
  const uint32_t tmp_adc_is_disable_on_going = LL_ADC_IsDisableOngoing(p_instance);

  /* Verification if ADC is not already disabled */
  /* Note: forbidden to disable ADC (set bit ADC_CR_ADDIS) if ADC is already
           disabled. */
  if ((LL_ADC_IsEnabled(p_instance) != 0UL)
      && (tmp_adc_is_disable_on_going == 0UL)
     )
  {
    /* Check if conditions to disable the ADC are fulfilled */
    if ((p_instance->CR & (ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADEN)) != ADC_CR_ADEN)
    {
      status = HAL_ERROR;
    }
    else
    {
      /* Disable the ADC peripheral */
      LL_ADC_Disable(p_instance);

      LL_ADC_ClearFlag_ADRDY(p_instance);

      /* Wait for ADC effectively disabled */
      /* Get tick count */
      tickstart = HAL_GetTick();

      while (LL_ADC_IsEnabled(p_instance) != 0UL)
      {
        if ((HAL_GetTick() - tickstart) > ADC_DISABLE_TIMEOUT_MS)
        {
          /* New check to avoid false timeout detection in case of preemption */
          if (LL_ADC_IsEnabled(p_instance) != 0UL)
          {
            status = HAL_ERROR;
            break;
          }
        }
      }
    }

    if (status == HAL_OK)
    {
      /* Set ADC instance to deepest disable level.
         Note: Except case of basic disable state needed by internal process */
      if (hadc->global_state != HAL_ADC_STATE_CALIB)
      {
        LL_ADC_DisableInternalRegulator(p_instance);
        LL_ADC_EnableDeepPowerDown(p_instance);
      }
    }
    else
    {
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
      hadc->last_error_codes |= HAL_ADC_ERROR_INTERNAL;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
    }
  }

  return status;
}

/**
  * @brief  Calibrate the selected ADC instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
static hal_status_t adc_calibrate(hal_adc_handle_t *hadc)
{
  hal_status_t status;
  ADC_TypeDef *p_instance;
  uint32_t tickstart;
  __IO uint32_t wait_loop_index;

  p_instance = ADC_GET_INSTANCE(hadc);

  /* Set ADC state for self-calibration */
  status = adc_deactivate(hadc);

  if (status == HAL_OK)
  {
    /* Perform ADC self-calibration */
    LL_ADC_StartCalibration(p_instance, LL_ADC_IN_SINGLE_ENDED);

    /* Poll for ADC calibration completion */
    tickstart = HAL_GetTick();
    while (LL_ADC_IsCalibrationOnGoing(p_instance) != 0UL)
    {
      if ((HAL_GetTick() - tickstart) > ADC_CALIBRATION_TIMEOUT_MS)
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (LL_ADC_IsCalibrationOnGoing(p_instance) != 0UL)
        {
          status = HAL_ERROR;
          break;
        }
      }
    }

    /* Calibration end phase */
    if (status == HAL_OK)
    {
      /* Restore ADC state */
      /* 1. Delay between ADC end of calibration and ADC enable */
      /* Note: Variable divided by 2 to compensate partially CPU processing cycles (depends on compilation
               optimization) */
      wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1U);
      while (wait_loop_index != 0U)
      {
        wait_loop_index--;
      }

      /* 2. Activate ADC */
      status = adc_activate(hadc);
    }
  }

  if (status != HAL_OK)
  {
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
    hadc->last_error_codes |= HAL_ADC_ERROR_INTERNAL;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
  }

  return status;
}

/**
  * @brief  Stop conversion on ADC group regular (low level).
  * @param  hadc Pointer to a hal_adc_handle_t structure
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
static hal_status_t adc_reg_stop_conversion(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance = ADC_GET_INSTANCE(hadc);
  uint32_t tickstart;

  /* Stop ADC conversion */
  LL_ADC_REG_StopConversion(p_instance);

  /* Wait for ADC conversion effectively stopped */
  tickstart = HAL_GetTick();
  while (LL_ADC_REG_IsConversionOngoing(p_instance) != 0UL)
  {
    if ((HAL_GetTick() - tickstart) > ADC_CONV_STOP_TIMEOUT_MS)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_ADC_REG_IsConversionOngoing(p_instance) != 0UL)
      {
        status = HAL_ERROR;
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
        hadc->last_error_codes |= HAL_ADC_ERROR_INTERNAL;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
        break;
      }
    }
  }

  return status;
}

/**
  * @brief  Stop conversion on ADC group injected (low level).
  * @param  hadc Pointer to a hal_adc_handle_t structure
  * @retval HAL_ERROR         Operation completed with error
  * @retval HAL_OK            Operation completed successfully
  */
static hal_status_t adc_inj_stop_conversion(hal_adc_handle_t *hadc)
{
  hal_status_t status = HAL_OK;
  ADC_TypeDef *p_instance = ADC_GET_INSTANCE(hadc);
  uint32_t tickstart;

  /* Stop ADC conversion */
  LL_ADC_INJ_StopConversion(p_instance);

  /* Wait for ADC conversion effectively stopped */
  tickstart = HAL_GetTick();
  while (LL_ADC_INJ_IsConversionOngoing(p_instance) != 0UL)
  {
    if ((HAL_GetTick() - tickstart) > ADC_CONV_STOP_TIMEOUT_MS)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_ADC_INJ_IsConversionOngoing(p_instance) != 0UL)
      {
        status = HAL_ERROR;
#if defined(USE_HAL_ADC_GET_LAST_ERRORS) && (USE_HAL_ADC_GET_LAST_ERRORS == 1)
        hadc->last_error_codes |= HAL_ADC_ERROR_INTERNAL;
#endif /* USE_HAL_ADC_GET_LAST_ERRORS */
        break;
      }
    }
  }

  return status;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  For all HAL ADC handles part of multimode, check handles state: multimode, instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  common_state_expected  Expected state of HAL handle related to ADC common instance
  * @param  instance_state_expected  Expected state of HAL handle related to ADC instance
  */
static void adc_assert_state_mm_inst(const hal_adc_handle_t *hadc,
                                     const hal_adc_common_state_t common_state_expected,
                                     const hal_adc_state_t instance_state_expected)
{
#if defined(USE_ASSERT_DBG_STATE)
  hal_adc_handle_t *p_handle_current = (hal_adc_handle_t *)hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    ASSERT_DBG_STATE(p_handle_current->common_state, common_state_expected);
    ASSERT_DBG_STATE(p_handle_current->global_state, instance_state_expected);
    p_handle_current = p_handle_current->p_link_next_handle;
  }
#else
  STM32_UNUSED(hadc);
  STM32_UNUSED(common_state_expected);
  STM32_UNUSED(instance_state_expected);
#endif /* USE_ASSERT_DBG_STATE */
}

/**
  * @brief  For all HAL ADC handles part of multimode, check handles state: group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  group_state_expected  Expected state of HAL handle related to ADC group regular
  */
static void adc_assert_state_mm_reg(const hal_adc_handle_t *hadc,
                                    const hal_adc_group_state_t group_state_expected)
{
#if defined(USE_ASSERT_DBG_STATE)
  hal_adc_handle_t *p_handle_current = (hal_adc_handle_t *)hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    ASSERT_DBG_STATE(p_handle_current->group_state[ADC_GROUP_REGULAR], group_state_expected);

    p_handle_current = p_handle_current->p_link_next_handle;
  }
#else
  STM32_UNUSED(hadc);
  STM32_UNUSED(group_state_expected);
#endif /* USE_ASSERT_DBG_STATE */
}

/**
  * @brief  For all HAL ADC handles part of multimode, check handles state: group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  group_state_expected  Expected state of HAL handle related to ADC group injected
  */
static void adc_assert_state_mm_inj(const hal_adc_handle_t *hadc,
                                    const hal_adc_group_state_t group_state_expected)
{
#if defined(USE_ASSERT_DBG_STATE)
  hal_adc_handle_t *p_handle_current = (hal_adc_handle_t *)hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    ASSERT_DBG_STATE(p_handle_current->group_state[ADC_GROUP_INJECTED], group_state_expected);

    p_handle_current = p_handle_current->p_link_next_handle;
  }
#else
  STM32_UNUSED(hadc);
  STM32_UNUSED(group_state_expected);
#endif /* USE_ASSERT_DBG_STATE */
}

/**
  * @brief  For all HAL ADC handles part of multimode, set handles state: multimode, instance.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  common_state  State of HAL handle related to ADC common instance
  * @param  instance_state  State of HAL handle related to ADC instance
  */
static void adc_mm_set_state_inst(hal_adc_handle_t *hadc,
                                  hal_adc_common_state_t common_state,
                                  hal_adc_state_t instance_state)
{
  hal_adc_handle_t *p_handle_current = hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    p_handle_current->common_state = common_state;
    p_handle_current->global_state = instance_state;

    p_handle_current = p_handle_current->p_link_next_handle;
  }
}

/**
  * @brief  For all HAL ADC handles part of multimode, set handles state: multimode, instance, group regular.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  common_state  State of HAL handle related to ADC common instance
  * @param  instance_state  State of HAL handle related to ADC instance
  * @param  group_state  State of HAL handle related to ADC group regular
  */
static void adc_mm_set_state_inst_reg(hal_adc_handle_t *hadc,
                                      hal_adc_common_state_t common_state,
                                      hal_adc_state_t instance_state,
                                      hal_adc_group_state_t group_state)
{
  hal_adc_handle_t *p_handle_current = hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    p_handle_current->common_state = common_state;
    p_handle_current->global_state = instance_state;
    p_handle_current->group_state[ADC_GROUP_REGULAR] = group_state;

    p_handle_current = p_handle_current->p_link_next_handle;
  }
}

/**
  * @brief  For all HAL ADC handles part of multimode, set handles state: multimode, instance, group injected.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  common_state  State of HAL handle related to ADC common instance
  * @param  instance_state  State of HAL handle related to ADC instance
  * @param  group_state  State of HAL handle related to ADC group injected
  */
static void adc_mm_set_state_inst_inj(hal_adc_handle_t *hadc,
                                      hal_adc_common_state_t common_state,
                                      hal_adc_state_t instance_state,
                                      hal_adc_group_state_t group_state)
{
  hal_adc_handle_t *p_handle_current = hadc;
  uint32_t index;

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    p_handle_current->common_state = common_state;
    p_handle_current->global_state = instance_state;
    p_handle_current->group_state[ADC_GROUP_INJECTED] = group_state;

    p_handle_current = p_handle_current->p_link_next_handle;
  }
}

/**
  * @brief  For all HAL ADC handles part of multimode, check and set handles state: group.
  * @param  hadc Pointer to a hal_adc_handle_t structure. Must be the handle of ADC master.
  * @param  group_index  handle state "group_state[]" index (ADC_GROUP_REGULAR, ADC_GROUP_INJECTED)
  * @param  group_state_conditional  State to be checked to authorize moving to the new state.
  * @param  group_state_new          New state to be set.
  * @retval HAL_BUSY          HAL ADC state machine not in expected initial state
  * @retval HAL_OK            ADC instance has been correctly configured.
  */
static hal_status_t adc_mm_check_set_state_group(hal_adc_handle_t *hadc,
                                                 uint8_t group_index,
                                                 hal_adc_group_state_t group_state_conditional,
                                                 hal_adc_group_state_t group_state_new)
{
  hal_adc_handle_t *p_handle_current = hadc;
  uint32_t index;
#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 0)
  hal_status_t status = HAL_OK;
  STM32_UNUSED(group_state_conditional);
#else
  hal_status_t status;
#endif /* USE_HAL_CHECK_PROCESS_STATE == 0 */

  /* Parse multimode instances through links of daisy chain (starting from ADC master) */
  for (index = 0; index < ADC_MM_INST_COUNT; index++)
  {
    HAL_CHECK_UPDATE_STATE(p_handle_current, group_state[group_index],
                           group_state_conditional,
                           group_state_new);

    p_handle_current = p_handle_current->p_link_next_handle;
  }

#if defined(USE_HAL_CHECK_PROCESS_STATE) && (USE_HAL_CHECK_PROCESS_STATE == 0)
  /* No action */
#else
  status = HAL_OK; /* Value update if no early return by HAL_CHECK_UPDATE_STATE() */
#endif /* USE_HAL_CHECK_PROCESS_STATE == 0 */

  return status;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_ADC_MODULE */
#endif /* ADC1 || ADC2 || ADC3 */
/**
  * @}
  */
