/**
  ******************************************************************************
  * @file    stm32c5xx_hal_dac.c
  * @brief   DAC HAL module driver.
  *
  *         This file provides firmware functions to manage the following
  *         functionalities of the Digital-to-Analog Converter (DAC) peripheral:
  *           + Initialization and de-initialization functions
  *           + Input and output operation functions
  *           + Peripheral control functions
  *           + Peripheral state and error functions
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
#if defined(DAC1)
#if defined(USE_HAL_DAC_MODULE) && (USE_HAL_DAC_MODULE == 1)

/** @defgroup DAC DAC
  * @{
  */
/** @defgroup DAC_Introduction DAC Introduction
  * @{

  - The DAC (Digital-to-Analog Converter) hardware abstraction layer provides a set of APIs to interface with the STM32
    DAC peripheral.
  - It simplifies the configuration, initialization, and management of analog output functionality, supporting
    various modes such as polling, interrupt, and DMA for efficient data transfer.
  - This abstraction layer ensures portability and ease of use across different STM32 series. The HAL DAC driver
    abstracts device-specific features.

  The HAL DAC driver includes the following features:
   - Output buffer management for driving external loads
   - Calibration features for improved analog output accuracy
   - Support for single and dual-channel operation
   - Advanced signal generation:
     - Noise
     - Triangle
   - User-defined buffer transfer via DMA
   - Sample-and-hold mode for low-power applications
  */
/**
  * @}
  */

/** @defgroup DAC_How_To_Use DAC How To Use
  * @{

  # DAC peripheral main features

  ## DAC channels

    On this STM32 series, DAC instances feature one or two channels depending on the device.
    @note Differentiation is made by the literal DAC_NB_OF_CHANNEL (defined in the DFP of each device).

    DAC channels have configurable resolution: 12-bit or 8-bit.
    They can be connected to device GPIO or on-chip peripherals.

    DAC channel conversion can be triggered by:
      - External event: EXTI Line
      - Other peripherals (timers, low-power timers, ...)
      - Software

    @note The trigger selection depends on the PWR mode, in STOP modes, only the triggers relative to peripherals
          that are functional in STOP modes are possible (ex: EXTI and LPTIM).

  ## DAC output buffer mode feature

    Each DAC channel integrates an output buffer that can be used to
    reduce the output impedance and drive external loads directly
    without adding an external amplifier.
    To enable the output buffer, use HAL_DAC_SetConfigChannel() to set HAL_DAC_OUTPUT_BUFFER_ENABLED.

    @note Refer to the device datasheet for more details about output
          impedance value with and without output buffer.

  ## Channel in separate mode, input and output operation

    + Use HAL_DAC_StartChannel() to enable one channel and start conversion
      for this channel.<br>
      Caution: when software trigger is selected, the call to HAL_DAC_StartChannel() starts
      the first conversion with the data previously present in DAC_DHRx register.
      Therefore, call HAL_DAC_SetChannelData() first to set the initial value of DAC_DHRx.
      Then subsequent conversions use data set with HAL_DAC_SetChannelData().
    + Use HAL_DAC_StopChannel() to disable one channel and stop conversion for this channel.
    + Use HAL_DAC_StartChannel_DMA() to enable one channel and start conversion
      for this channel using DMA to feed DAC converters.<br>
      Caution: the first issued trigger starts the conversion with the data previously present in DAC_DHRx register.
      Therefore, call HAL_DAC_SetChannelData() first to set the initial value of DAC_DHRx.<br>
    + Use HAL_DAC_StopChannel_DMA() to disable the channel and stop conversion for that channel using DMA.
    + Use HAL_DAC_SetChannelData() to set digital data to be converted.
    + Use HAL_DAC_GetChannelData() to get digital data to be converted.

  ## Dual-channel mode, input and output operation

    Dual mode allows the use of two DAC channels for simultaneous operation.

    + Use HAL_DAC_StartDualChannel() to enable both channels and start conversion
      for dual mode operation.<br>
      Caution: when software trigger is selected, the call to HAL_DAC_StartDualChannel() starts
      the first conversion with the data previously present in DAC_DHRx register.
      Therefore, a first call to HAL_DAC_SetDualChannelData is required in order to fix the initial value of DAC_DHRx.
      Then subsequent conversions use data set with HAL_DAC_SetDualChannelData().
    + Use HAL_DAC_StopDualChannel() to disable both channel and stop conversion
        for dual mode operation.
    + Use HAL_DAC_StartDualChannel_DMA() to enable both channels and start conversion
      for dual mode operation using DMA to feed DAC converters.<br>
      Caution: the first issued trigger starts the conversion with the data previously present in DAC_DHRx register.
      Therefore, a first call to HAL_DAC_SetDualChannelData is required in order to fix the initial value of DAC_DHRx.
      The same callbacks used in single mode are called in dual mode to notify
      transfer completion (half or complete), errors, or underrun.
    + Use HAL_DAC_StopDualChannel_DMA() to disable both channel and stop conversion
        for dual mode operation using DMA to feed DAC converters.

    + When dual-channel mode is enabled (i.e. DAC channel 1 and channel 2 are used simultaneously):
      use HAL_DAC_GetDualChannelData() to get digital data to be converted and use
        HAL_DAC_SetDualChannelData() to set digital data to be converted simultaneously in
        channel 1 and channel 2.

  ## DAC sample and hold feature

    For each converter, two modes are supported: "normal mode" and
    "sample and hold" mode (i.e. low-power mode).
    In the "sample and hold" mode, the DAC core converts data and then holds the
    converted voltage on a capacitor. If the DAC output is connected to an on-chip peripheral, this capacitor is
    internal. If the DAC output is connected to a pin, an external capacitor must be connected to the output pin.<br>
    When not converting, the DAC cores and buffer are completely turned off between samples and the DAC output is
    tri-stated, therefore reducing the overall power consumption. A new
    stabilization period is needed before each new conversion.

    The sample and hold mode allows setting internal or external voltage with a low-power consumption cost
    (output value can be changed at any given rate either by CPU or DMA).

    The sample and hold block and registers use LSI and run in
    several power modes: run mode, sleep mode, low-power run, low-power sleep
    mode and stop1 mode.

    Low power stop1 mode allows only static conversion.

    To enable sample and hold mode, enable LSI using HAL_RCC_LSI_Enable().

    Use HAL_DAC_SetConfigChannelSampleAndHold() to set sample_time_cycle, hold_time_cycle, and refresh_time_cycle.
    Use HAL_DAC_EnableChannelSampleAndHold() or HAL_DAC_DisableChannelSampleAndHold()
    to enable or disable sample and hold mode.

  ## DAC calibration feature
    - DAC channels have calibration capabilities: aim to correct some offset in the output buffer.
    - The DAC uses either factory calibration settings or user-defined
        calibration settings (i.e. trimming mode).
    - The user-defined settings can be determined using self-calibration
        handled by HAL_DAC_CalibrateChannelBuffer().
    - Use HAL_DAC_CalibrateChannelBuffer() to:
       - Calibrate one DAC channel and also to set automatically the trimming value.
       - Run the calibration automatically.
       - Enable the user trimming mode.
       - Update a structure with trimming values using fresh calibration
         results.
    - Use HAL_DAC_GetChannelBufferCalibrationValue() to retrieve the trimming value (the trimming factory setting
         after reset, or the user setting if HAL_DAC_SetChannelBufferCalibrationValue() has been used
         at least once after reset).<br>
         The user might store the calibration results for later usage (for instance: monitoring the trimming
         as a function of temperature).
    - Use HAL_DAC_SetChannelBufferCalibrationValue() to set the trimming value.

  ## DAC wave generation feature

    Both DAC channels can be used to add a noise wave or a triangle wave.

    + Use HAL_DAC_EnableChannelAddingTriangleWave() to enable adding triangle wave signal.
    + Use HAL_DAC_DisableChannelAddingTriangleWave() to disable adding triangle wave signal.
    + Use HAL_DAC_EnableChannelAddingNoiseWave() to enable adding noise wave signal.
    + Use HAL_DAC_DisableChannelAddingNoiseWave() to disable adding noise wave signal.

  ## DAC data format

    The DAC data alignment format can be:
    -  8-bit right alignment using HAL_DAC_DATA_ALIGN_8_BITS_RIGHT
    - 12-bit left alignment using HAL_DAC_DATA_ALIGN_12_BITS_LEFT
    - 12-bit right alignment using HAL_DAC_DATA_ALIGN_12_BITS_RIGHT

  ## DAC data value to voltage correspondence

     The analog output voltage on each DAC channel pin is determined
     by the following equation:
            DAC_OUTx = (Vref+) x (DOR / 4095) <br>
    where DOR is the Data Output Register, and Vref+ is the input voltage reference (refer to the device datasheet)<br>
     e.g.:
       - Assuming that Vref+ is equal to 3.3V, to set DAC_OUT1 to 0.7 V, set data value 868 in DOR register:
       - DAC_OUT1 = (3.3) x (868 / 4095) = 0.7 V

         @note A helper macro is available to calculate the DAC conversion data (unit: digital value) corresponding
           to a voltage (unit: mVolt). Refer to the LL DAC driver: LL_DAC_CALC_VOLTAGE_TO_DATA

  ## DMA requests

      A DAC channel can operate with data transfer by DMA using HAL_DAC_StartChannel_DMA().
      DMA requests are generated when an external trigger (not a software trigger) occurs.

  ## High frequency interface mode

    The high frequency interface informs the DAC instance about the bus frequency in use.
    This is mandatory information for the DAC (as internal timing of the DAC is bus-frequency-dependent).
    Provide this information with the high_frequency_mode parameter handled in HAL_DAC_SetConfig().<br>

    The optimum frequency interface mode for the DAC peripheral can be determined by
    calling HAL_DAC_GetOptimumFrequencyMode().
    Then set the optimum high frequency interface mode (HFSEL bits) with HAL_DAC_SetConfig().<br>

    The high frequency mode is the same for all converters of the same DAC instance.


# How to use the HAL DAC module driver

## The DAC HAL driver can be used as follows (in separate channel configuration):

  ### Initialize the DAC low-level resources:
    + Enable the DAC APB clock to get write access to the DAC.
      Enable the DACx interface clock.<br>
      Note: the clock is enabled inside HAL_DAC_Init() whenever USE_HAL_DAC_CLK_ENABLE_MODEL is not set
            to HAL_CLK_ENABLE_NO.

    + Declare a hal_dac_handle_t handle structure, for example: hal_dac_handle_t hdac;
    + Initialize the DAC instance using HAL_DAC_Init().
    + Configure the DAC output GPIO pins for the used channels in analog mode.
    + Configure the DAC instance with HAL_DAC_SetConfig().
    + Configure the DAC channel with HAL_DAC_SetConfigChannel().
    + If needed, link the DAC to DMA with HAL_DAC_SetChannelDMA().
    + Enable the DAC channel with HAL_DAC_StartChannel() or HAL_DAC_StartChannel_DMA().

    ### Polling mode, input and output operation

       + Start the DAC peripheral on a given channel with HAL_DAC_StartChannel().
       + To change the data output value, use the HAL_DAC_SetChannelData() function.
       + To read the last DAC data output value, use the HAL_DAC_GetChannelData() function.
       + Stop the DAC peripheral on a given channel with HAL_DAC_StopChannel().

    ### DMA mode, input and output operation

       + Start the DAC peripheral on a given channel using DMA with HAL_DAC_StartChannel_DMA(),
         and specify a data buffer and the data size to be converted through DMA.<br>
         Caution: when software trigger is selected, the call to HAL_DAC_StartDualChannel() starts
         the first conversion with the data previously present in DAC_DHRx register.
         Therefore, a first call to HAL_DAC_SetChannelData() is required in order to fix the initial value of DAC_DHRx.
         + At the midpoint of the data transfer, HAL_DAC_ConvHalfCpltCallback()
           function is executed (the user can add custom code by overriding this weak function).
         + At the end of the data transfer, HAL_DAC_ConvCpltCallback() function is executed
           (the user can add custom code by overriding this weak function).
         + If a transfer error occurs, HAL_DAC_ErrorCallback() function is executed (the user can add custom code
           by overriding this weak function).

         + If a DMA underrun occurs, a DAC interrupt triggers and executes the internal function HAL_DAC_IRQHandler().
           HAL_DAC_ErrorCallback() function is executed (the user can add custom code
           by overriding this weak function).

       + Stop the DAC peripheral using HAL_DAC_StopChannel_DMA().

    ### Callback registration

      The compilation define, "USE_HAL_DAC_REGISTER_CALLBACKS", when set to 1,
      allows the user to configure driver callbacks dynamically.

       + Use HAL_DAC_RegisterConvHalfCpltCallback() to register
         HAL_DAC_ConvHalfCpltCallback(): callback when a half transfer is completed on a channel.
       + Use HAL_DAC_RegisterConvCpltCallback() to register
         HAL_DAC_ConvCpltCallback(): callback when a transfer is completed on a channel.
       + Use HAL_DAC_RegisterErrorCallback() to register
         HAL_DAC_ErrorCallback(): callback when an error occurs on a channel.

      These functions take the HAL peripheral handle and a pointer to the user callback function as parameters.

       By default, after the first HAL_DAC_Init(), all callbacks are reset to the corresponding
       legacy weak (overridden) functions.
      Register callbacks only in
       state HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED or HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED.
      When the compilation define USE_HAL_DAC_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registering feature is not available,
      and weak (overridden) callbacks are used.
  */
/**
  * @}
  */

/** @defgroup DAC_Configuration_Table DAC Configuration Table
  * @{
# Configuration in the DAC driver

  Config defines               | Description        | Default value | Note                                     |
-------------------------------| -------------------| ------------- | ------------------------------------------
USE_HAL_DAC_MODULE             | hal_conf.h         | 1 | HAL DAC module is enabled.
USE_HAL_DAC_USER_DATA          | hal_conf.h         | 0 | Enable user data.
USE_HAL_DAC_REGISTER_CALLBACKS | hal_conf.h         | 0 | Enable register callback assertions.
USE_HAL_DAC_CLK_ENABLE_MODEL | hal_conf.h | HAL_CLK_ENABLE_NO | Otherwise, the clock is enabled inside HAL_DAC_Init().
USE_HAL_DAC_DMA                | hal_conf.h         | 1 | Use DMA with the DAC.
USE_HAL_CHECK_PARAM            | hal_conf.h         | 0 | Enable run-time checks on function parameters.
USE_HAL_CHECK_PROCESS_STATE    | hal_conf.h         | 0 | Enable run-time checks on the state during processing.
USE_HAL_DAC_GET_LAST_ERRORS    | hal_conf.h         | 0 | Record errors that occur during DAC channel processing.
USE_HAL_DAC_DUAL_CHANNEL       | hal_conf.h         | 1 | Enable DAC dual channel mode (if DAC_NB_OF_CHANNEL > 1).
USE_ASSERT_DBG_PARAM           | Pre-processor env  | None | Enable assert checks on function parameters.
USE_ASSERT_DBG_STATE           | Pre-processor env  | None | Enable assert checks on module state.
DAC_NB_OF_CHANNEL              | DFP                | NA | DAC channel count (value "2" allows dual mode).

  ******************************************************************************
  */
/**
  * @}
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @addtogroup DAC_Private_Constants DAC Private Constants
  * @{
  */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
#define DAC_STATE_ALL (((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED) \
                       | ((uint32_t) HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED)   \
                       | ((uint32_t) HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE))           /*!< DAC all states except IDLE */

#define DAC_STATE_CONFIG (((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED) \
                          | ((uint32_t) HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED))    /*!< DAC all CONFIGURED states */
#else
#define DAC_STATE_ALL ((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED)       /*!< DAC all states except IDLE */

#define DAC_STATE_CONFIG ((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED)    /*!< DAC CONFIGURED states */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#else
#define DAC_STATE_ALL ((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED)       /*!< DAC all states except IDLE */

#define DAC_STATE_CONFIG ((uint32_t) HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED)    /*!< DAC CONFIGURED states */
#endif /* DAC_NB_OF_CHANNEL */


#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
#define DAC_CHANNEL_STATE_ALL (((uint32_t) HAL_DAC_CHANNEL_STATE_IDLE)     \
                               | ((uint32_t) HAL_DAC_CHANNEL_STATE_ACTIVE) \
                               | ((uint32_t) HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT)) /*!< DAC CHANNEL all states */
#else
#define DAC_CHANNEL_STATE_ALL (((uint32_t) HAL_DAC_CHANNEL_STATE_IDLE)     \
                               | ((uint32_t) HAL_DAC_CHANNEL_STATE_ACTIVE)) /*!< DAC CHANNEL all states */
#endif /* USE_HAL_DAC_DMA  */

#define DAC_TIMEOUT_FOR_BWST_MS  1UL  /*!< Timeout of 1 ms after writing in DAC_SHSRx register */

/* Delay for DAC minimum trimming time.                                       */
/* Note: minimum time needed between two calibration steps                    */
/*       The delay below is specified under conditions:                       */
/*        - DAC channel output buffer enabled                                 */
/* Literal set to maximum value (refer to device datasheet,                   */
/* electrical characteristics, parameter "tTRIM").                            */
/* Unit: us                                                                   */
#define DAC_DELAY_TRIM_US      50UL  /*!< Delay of 50 us for DAC minimum trimming time */

/* High frequency interface mode selection. */
#define DAC_HFSEL_ENABLE_THRESHOLD_80MHZ   80000000U  /*!< High frequency clock selection:  80 MHz */
#define DAC_HFSEL_ENABLE_THRESHOLD_160MHZ  160000000U /*!< High frequency clock selection: 160 MHz */

/**
  * Delay for DAC channel voltage settling time from DAC channel startup
  * (transition from disable to enable).
  * Note: DAC channel startup time depends on board application environment:
  *       impedance connected to DAC channel output.
  *       The delay below is specified under conditions:
  *        - voltage maximum transition (lowest to highest value)
  *        - until voltage reaches final value +-1LSB
  *        - DAC channel output buffer enabled
  *        - load impedance of 5kOhm (min), 50pF (max)
  * Literal set to maximum value (refer to device datasheet,
  * parameter "tWAKEUP").
  * Unit: us
  */
#define DAC_DELAY_STARTUP_US    (15UL)  /*!< Delay of 15 us for DAC channel voltage settling time from DAC channel
                                             startup (transition from disable to enable) */
/**
  * @}
  */

/* Private macros -------------------------------------------------------------*/
/** @defgroup DAC_Private_Macros DAC Private Macros
  * @{
  */

#define DAC_GET_INSTANCE(hdac) ((DAC_TypeDef *)((uint32_t)((hdac)->instance)))/*!< Retrieve DAC instance from handle */

#define DAC_GET_DMA_PARENT(hdma) ((hal_dac_handle_t *)((hdma)->p_parent))/*!< Retrieve DMA parent from handle */

#define DAC_GET_ALIGNMENT_FROM_DHR_ADDRESS(dhr_reg_base, dhr_reg)              \
  (((dhr_reg_base) >= (dhr_reg))                                               \
   ? (hal_dac_data_alignment_t)((uint32_t)(((dhr_reg_base) - (dhr_reg)) >> 2)) \
   : (HAL_DAC_DATA_ALIGN_12_BITS_RIGHT)                                        \
  )  /*!< Retrieve alignment from DHR register addresses */

#define DAC_GET_ALIGNMENT_CHANNEL(hdac, channel)                                                             \
  (DAC_GET_ALIGNMENT_FROM_DHR_ADDRESS((uint32_t)((hdac)->channel_dhr_address[(uint32_t)((channel))]),        \
                                      (uint32_t)(((uint32_t) &(DAC_GET_INSTANCE(hdac)->DHR12R1))             \
                                                 + (uint32_t)(3UL* sizeof(uint32_t) * (uint32_t)((channel))) \
                                                )                                                            \
                                     ))  /*!< Retrieve alignment from register addresses for a channel */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
#define DAC_GET_ALIGNMENT_DUAL(hdac)                                                                          \
  (DAC_GET_ALIGNMENT_FROM_DHR_ADDRESS((uint32_t)((hdac)->channel_dhr_address[(uint32_t)(HAL_DAC_CHANNEL_1)]), \
                                      (uint32_t)((uint32_t) &(DAC_GET_INSTANCE(hdac)->DHR12RD))               \
                                     ))  /*!< Retrieve alignment from register addresses for dual channel */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */

/*! Triggers */
#define IS_DAC_TRIGGER(hdac, trigger)  (((trigger) == HAL_DAC_TRIGGER_NONE)          \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM1_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM2_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM5_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM6_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM7_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM8_TRGO)  \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM12_TRGO) \
                                        || ((trigger) == HAL_DAC_TRIGGER_TIM15_TRGO) \
                                        || ((trigger) == HAL_DAC_TRIGGER_LPTIM1_OC1) \
                                        || ((trigger) == HAL_DAC_TRIGGER_EXTI9)      \
                                        || ((trigger) == HAL_DAC_TRIGGER_SOFTWARE))

/*! High Frequencies mode selection */
#define  IS_DAC_HIGH_FREQUENCY_MODE(mode) (((mode) == HAL_DAC_HIGH_FREQ_MODE_DISABLED)       \
                                           || ((mode) == HAL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ) \
                                           || ((mode) == HAL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ))

/*! Sample time when "sample and hold mode" is enabled */
#define IS_DAC_SAMPLE_TIME(time)  ((time) <= 0x000003FFU)

/*! Hold time when "sample and hold mode" is enabled */
#define IS_DAC_HOLD_TIME(time)    ((time) <= 0x000003FFU)

/*! Refresh time when "sample and hold mode" is enabled */
#define IS_DAC_REFRESH_TIME(time) ((time) <= 0x000000FFUL)

/*! Sample and hold mode: enabled / disabled */
#define IS_DAC_SAMPLEANDHOLD(mode) (((mode) == HAL_DAC_SAMPLE_AND_HOLD_DISABLED) \
                                    || ((mode) == HAL_DAC_SAMPLE_AND_HOLD_ENABLED))

/*! Output connection */
#define IS_DAC_CHIP_CONNECTION(connect) (((connect) == HAL_DAC_OUTPUT_CONNECTION_EXTERNAL) \
                                         || ((connect) == HAL_DAC_OUTPUT_CONNECTION_INTERNAL))

/*! Calibration: check trimming value range */
#define IS_DAC_TRIMMINGVALUE(trimming_value) ((trimming_value) <= 0x1FU)

/*! Amplitude for triangle wave or pseudo noise wave (generated with LFSR) */
#define IS_DAC_WAVE_AMPLITUDE(value) (((value) == HAL_DAC_WAVE_AMPLITUDE_1)       \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_3)    \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_7)    \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_15)   \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_31)   \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_63)   \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_127)  \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_255)  \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_511)  \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_1023) \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_2047) \
                                      || ((value) == HAL_DAC_WAVE_AMPLITUDE_4095))

/*!  DMA double data mode (enable/disable) */
#define IS_DAC_DMA_DOUBLE_MODE(state) (((state) == HAL_DAC_DMA_DOUBLE_DATA_MODE_ENABLED) \
                                       || ((state) == HAL_DAC_DMA_DOUBLE_DATA_MODE_DISABLED))

/*! Data format (signed/unsigned) */
#define IS_DAC_SIGN_FORMAT(state) (((state) == HAL_DAC_SIGN_FORMAT_SIGNED) \
                                   || ((state) == HAL_DAC_SIGN_FORMAT_UNSIGNED))

/*! State of the channel output buffer (enable/disable) */
#define IS_DAC_OUTPUT_BUFFER_STATE(state) (((state) == HAL_DAC_OUTPUT_BUFFER_ENABLED) \
                                           || ((state) == HAL_DAC_OUTPUT_BUFFER_DISABLED))

/*! Check DAC channel */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define IS_DAC_CHANNEL(hdac, channel) (((channel) == HAL_DAC_CHANNEL_1) \
                                       || ((channel) == HAL_DAC_CHANNEL_2))
#else
#define IS_DAC_CHANNEL(hdac, channel) ((channel) == HAL_DAC_CHANNEL_1)
#endif /* DAC_NB_OF_CHANNEL */

/*! To choose the alignment of the data */
#define IS_DAC_ALIGN(align) (((align) == HAL_DAC_DATA_ALIGN_12_BITS_RIGHT) \
                             || ((align) == HAL_DAC_DATA_ALIGN_12_BITS_LEFT) \
                             || ((align) == HAL_DAC_DATA_ALIGN_8_BITS_RIGHT))

/*! Check the data range */
#define IS_DAC_DATA(data) ((data) <= 0x0000FFF0UL)  /* Maximum value of 12 bit left alignment */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/*! Check the dual data range */
#define IS_DAC_DATA_DUAL(data) ((data) <= 0xFFF0FFF0UL) /* Maximum value of 12 bit left alignment */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */

/*! Check the double mode data range */
#define IS_DAC_DATA_DOUBLE_MODE(data) ((data) <= 0xFFF0FFF0UL) /* Maximum value of 12 bit left alignment */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/*! Check the compatibility between HAL_DAC_OPT_DMA_IT_SILENT and DMA configuration */
#define IS_DAC_DMA_VALID_SILENT_MODE(handle_dma, interrupts) \
  (((interrupts) != HAL_DAC_OPT_DMA_IT_SILENT) \
   || (handle_dma->xfer_mode == HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR))
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_DAC_DMA */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup DAC_Private_Variables DAC Private Variables
  * @{
  */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/*! Lookup table for channel identifier, to convert channel index from HAL_DAC_CHANNEL_x to LL_DAC_CHANNEL_x */
static const uint32_t lut_ch[DAC_NB_OF_CHANNEL] = {LL_DAC_CHANNEL_1,
                                                   LL_DAC_CHANNEL_2
                                                  };

/*! Lookup table for channel bit offset mask */
static const uint32_t lut_ch_shift[DAC_NB_OF_CHANNEL] = {(LL_DAC_CHANNEL_1 & DAC_CR_CHX_BITOFFSET_MASK),
                                                         (LL_DAC_CHANNEL_2 & DAC_CR_CHX_BITOFFSET_MASK)
                                                        };

/*! Lookup table for channel DMA underrun interrupt */
static const uint32_t lut_ch_dma_underrun_it[DAC_NB_OF_CHANNEL] = {LL_DAC_IT_DMAUDRIE1,
                                                                   LL_DAC_IT_DMAUDRIE2
                                                                  };

#else
/*! Lookup table for channel identifier, to convert channel index from HAL_DAC_CHANNEL_x to LL_DAC_CHANNEL_x */
static const uint32_t lut_ch[DAC_NB_OF_CHANNEL] = {LL_DAC_CHANNEL_1};

/*! Lookup table for channel bit offset mask */
static const uint32_t lut_ch_shift[DAC_NB_OF_CHANNEL] = {LL_DAC_CHANNEL_1 & DAC_CR_CHX_BITOFFSET_MASK};

/*! Lookup table for channel DMA underrun interrupt */
static const uint32_t lut_ch_dma_underrun_it[DAC_NB_OF_CHANNEL] = {LL_DAC_IT_DMAUDRIE1};

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @defgroup DAC_Private_Functions DAC Private Functions
  * @{
  */

static void DAC_WaitMicroSecond(uint32_t delay_us);

static void DAC_SetChannelAlignment(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                    hal_dac_data_alignment_t alignment);

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
static void DAC_SetDualChannelAlignment(hal_dac_handle_t *hdac, hal_dac_data_alignment_t alignment);

#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
static void DAC_SetChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma, hal_dac_channel_t channel);
#endif /* USE_HAL_DAC_DMA */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/* DAC_DMA_ChConvCplt / DAC_DMA_ChHalfConvCplt / DAC_DMA_ChError */
/* are set in HAL_DAC_StartChannel_DMA */
static void DAC_DMA_ChConvCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_ChHalfConvCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_ChStopCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_ChError(hal_dma_handle_t *hdma);
static hal_status_t DAC_StartChannel_DMA_Opt(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                             uint32_t size_byte, uint32_t dac_opt_interrupt);
#endif /* USE_HAL_DAC_DMA */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/* DAC_DMA_DualChannelConvCplt / DAC_DMA_DualChannelHalfConvCplt / DAC_DMA_DualChannelError */
/* are set in HAL_DAC_StartDualChannel_DMA */
static void DAC_DMA_DualChannelConvCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_DualChannelHalfConvCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_DualChannelStopCplt(hal_dma_handle_t *hdma);
static void DAC_DMA_DualChannelError(hal_dma_handle_t *hdma);
hal_status_t DAC_StartDualChannel_DMA_Opt(hal_dac_handle_t *hdac, const void *p_data,
                                          uint32_t size_byte, uint32_t dac_opt_interrupt);
#endif /* USE_HAL_DAC_DMA */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
__STATIC_INLINE void DAC_IRQHandlerCH(hal_dac_handle_t *hdac, hal_dac_channel_t channel);

/**
  * @}
  */

/* Exported functions -------------------------------------------------------*/

/** @defgroup DAC_Exported_Functions HAL DAC Functions
  * @{
  */

/** @defgroup DAC_Exported_Functions_Group1_1 Initialization, de-initialization, configuration, calibration functions
    This section provides functions allowing to:
      + Initialize the DAC,
      + De-initialize the DAC,
      + Configure the DAC,
      + Chose optimum frequency,
      + Calibration setting.

  * @{
  */

/**
  * @brief  Initialize the DAC peripheral handle with a DAC instance.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  instance A DAC hardware peripheral base address.
  * @note   After calling this function, the DAC jumps to HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED, and it is possible
  *         to call directly HAL_DAC_StartCHannel() without calling HAL_DAC_SetConfig() or HAL_DAC_SetConfigChannel().
  *         In this case the DAC default configuration parameters are:
  *         + no DMA linked with DAC
  *         + HAL_DAC_HIGH_FREQ_MODE_DISABLED
  *         + HAL_DAC_SAMPLE_AND_HOLD_DISABLED
  *         + HAL_DAC_AM_DISABLED
  *         + HAL_DAC_DMA_DOUBLE_DATA_MODE_DISABLED
  *         + HAL_DAC_DATA_ALIGN_12_BITS_RIGHT
  *         + HAL_DAC_SIGN_FORMAT_UNSIGNED
  *         + HAL_DAC_TRIGGER_NONE
  *         + HAL_DAC_OUTPUT_BUFFER_ENABLED
  * @retval HAL_OK DAC instance has been correctly initialized.
  * @retval HAL_INVALID_PARAM A parameter is invalid.
  */
hal_status_t HAL_DAC_Init(hal_dac_handle_t *hdac, hal_dac_t instance)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_ALL_INSTANCE((DAC_TypeDef *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdac == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->instance = instance;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_conv_cplt_cb      = HAL_DAC_ConvCpltCallback;
  hdac->p_conv_half_cplt_cb = HAL_DAC_ConvHalfCpltCallback;
  hdac->p_error_cb          = HAL_DAC_ErrorCallback;
  hdac->p_stop_cplt_cb      = HAL_DAC_StopCpltCallback;
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
  hdac->p_dual_channel_conv_cplt_cb      = HAL_DAC_DualChannelConvCpltCallback;
  hdac->p_dual_channel_conv_half_cplt_cb = HAL_DAC_DualChannelConvHalfCpltCallback;
  hdac->p_dual_channel_stop_cplt_cb      = HAL_DAC_DualChannelStopCpltCallback;
#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

#if defined(USE_HAL_DAC_CLK_ENABLE_MODEL) && (USE_HAL_DAC_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  HAL_RCC_DAC1_EnableClock();
#endif /* USE_HAL_DAC_CLK_ENABLE_MODEL */

  for (uint32_t index = 0; index < DAC_NB_OF_CHANNEL; index++)
  {
    hdac->channel_state[index] = HAL_DAC_CHANNEL_STATE_IDLE;

    /* Set default alignment (12-bit right-aligned) for each channel */
    DAC_SetChannelAlignment(hdac, (hal_dac_channel_t)index, HAL_DAC_DATA_ALIGN_12_BITS_RIGHT);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
    hdac->last_error_codes[(uint32_t)index] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
  }

  hdac->global_state = HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED;
  return HAL_OK;
}

/**
  * @brief  Deinitialize the DAC peripheral.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @note   Stop the DAC and restore the state machine to reset state.
  */
void HAL_DAC_DeInit(hal_dac_handle_t *hdac)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  p_instance = DAC_GET_INSTANCE(hdac);
  ASSERT_DBG_PARAM(IS_DAC_ALL_INSTANCE(p_instance));

#if defined (USE_HAL_DAC_USER_DATA) && (USE_HAL_DAC_USER_DATA == 1)
  hdac->p_user_data = NULL;
#endif /* USE_HAL_DAC_USER_DATA */

  for (uint32_t index = 0; index < DAC_NB_OF_CHANNEL; index++)
  {
    LL_DAC_Disable(p_instance, lut_ch[index]);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
    hdac->last_error_codes[(uint32_t)index] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

    hdac->channel_state[index] = HAL_DAC_CHANNEL_STATE_IDLE;
  }

  hdac->global_state = HAL_DAC_STATE_RESET;
}

/**
  * @brief  Get the optimum frequency interface mode for the DAC peripheral.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @note   After calling this function, set the optimum high frequency interface mode (HFSEL bits)
  *         with HAL_DAC_SetConfig().
  * @retval hal_dac_high_freq_mode_t Optimum frequency interface mode.
  */
hal_dac_high_freq_mode_t HAL_DAC_GetOptimumFrequencyMode(const hal_dac_handle_t *hdac)
{
  hal_dac_high_freq_mode_t optimum_freq_mode;
  uint32_t hclkfreq;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  STM32_UNUSED(hdac);

  /* Get the system core clock frequency */
  hclkfreq = HAL_RCC_GetHCLKFreq();

  if (hclkfreq > DAC_HFSEL_ENABLE_THRESHOLD_160MHZ)
  {
    /* HCLK higher than 160 Mhz is available */
    optimum_freq_mode = HAL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ;
  }
  else if (hclkfreq > DAC_HFSEL_ENABLE_THRESHOLD_80MHZ)
  {
    /* HCLK higher than 80 Mhz is available */
    optimum_freq_mode = HAL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ;
  }
  else
  {
    /* HCLK lower than 80 Mhz, so disable high frequency mode */
    optimum_freq_mode = HAL_DAC_HIGH_FREQ_MODE_DISABLED;
  }

  return optimum_freq_mode;
}

/**
  * @brief  Configure the DAC peripheral according to the specified parameters in the hal_dac_config_t.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_config  The configuration that contains information for the specified DAC.
  * @note   By calling this function, the high frequency interface mode (HFSEL bits) is set.
  *         Optionally, before calling this function, the optimum high frequency interface mode (HFSEL bits)
  *         could be determined with HAL_DAC_GetOptimumFrequencyMode().
  * @retval HAL_OK DAC instance has been correctly configured.
  * @retval HAL_INVALID_PARAM A parameter is invalid.
  */
hal_status_t HAL_DAC_SetConfig(hal_dac_handle_t *hdac, const hal_dac_config_t *p_config)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_DAC_HIGH_FREQUENCY_MODE(p_config->high_frequency_mode));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

#endif /* DAC_NB_OF_CHANNEL */
#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }

#endif /* USE_HAL_CHECK_PARAM */
  p_instance = DAC_GET_INSTANCE(hdac);
  LL_DAC_SetHighFrequencyMode(p_instance, (uint32_t)(p_config->high_frequency_mode));

  return HAL_OK;
}

/**
  * @brief  Return the configuration parameters of the DAC peripheral
  *         in the hal_dac_config_t.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_config The configuration parameters.
  */
void HAL_DAC_GetConfig(const hal_dac_handle_t *hdac, hal_dac_config_t *p_config)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Set the high frequency interface mode selection */
  p_config->high_frequency_mode = (hal_dac_high_freq_mode_t) LL_DAC_GetHighFrequencyMode(p_instance);
}

/**
  * @brief  Reset the configuration parameters of the DAC and its channels.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @note   Configuration parameters of the DAC are reset to:
  *         + DAC is disabled and no more DMA is running with DAC
  *         + normal mode operation (ie: not in calibration mode)
  *         + high frequency disabled
  *         + sample and hold mode is disabled
  *         + no wave generation
  *         + DMA double data mode is disabled
  *         + unsigned data
  *         + 12 bits right alignment
  *         + trigger none
  *         + output buffer enabled
  *         + external pin connection
  */
void HAL_DAC_ResetConfig(hal_dac_handle_t *hdac)
{
  DAC_TypeDef *p_instance;
  uint32_t reg_value;
  uint32_t reg_mask;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

#endif /* DAC_NB_OF_CHANNEL */

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Reset the configuration items to their default/HW reset values */
  for (uint32_t index = 0; index < DAC_NB_OF_CHANNEL; index++)
  {
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
    hdac->last_error_codes[(uint32_t)index] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
    hdac->dma_ch[(uint32_t)index]  = NULL;
#endif /* USE_HAL_DAC_DMA */

    reg_value = LL_DAC_READ_REG(p_instance, CR);

    reg_mask = ((DAC_CR_EN1)          /* To disable the DAC channel */
                | (DAC_CR_DMAEN1)     /* To disable the selected DAC channel DMA request */
                | (DAC_CR_DMAUDRIE1)  /* To disable the DAC DMA underrun interrupt */
                | (DAC_CR_CEN1)       /* To set operation mode normal */
                | (DAC_CR_WAVE1)      /* To disable the wave generation */
                | (DAC_CR_MAMP1)      /* To disable the DAC channel mask/amplitude selector */
                | (DAC_CR_TSEL1)      /* To set trigger selection to software */
                | (DAC_CR_TEN1)       /* To disable the trigger (ie TRIGGER_NONE) */
                | (DAC_MCR_HFSEL)     /* To disable the High Frequency Mode */
               ) << lut_ch_shift[index];
    reg_value &= ~reg_mask ;
    reg_value |= ((uint32_t)(LL_DAC_MODE_NORMAL_OPERATION)
                  | (uint32_t)(LL_DAC_WAVE_AUTO_GENERATION_NONE)
                  | (uint32_t)(LL_DAC_TRIGGER_SOFTWARE)
                  | (uint32_t)(LL_DAC_HIGH_FREQ_MODE_DISABLED)
                 ) << lut_ch_shift[index];

    LL_DAC_Disable(p_instance, lut_ch[index]);   /* first write CR to set EN bit to 0 */
    LL_DAC_WRITE_REG(p_instance, CR, reg_value); /* then write again CR to set CEN bit to 0 after EN bit is set to 0 */

    reg_value = LL_DAC_READ_REG(p_instance, MCR);

    reg_mask = ((DAC_MCR_SINFORMAT1)
                | (DAC_MCR_MODE1_0)
                | (DAC_MCR_MODE1_1)
                | (DAC_MCR_MODE1_2)
                | (DAC_MCR_DMADOUBLE1)
               ) << lut_ch_shift[index];
    reg_value &= ~reg_mask ;
    reg_value |= ((uint32_t)(LL_DAC_SIGN_FORMAT_UNSIGNED)
                  | (uint32_t)(LL_DAC_OUTPUT_CONNECT_EXTERNAL)
                  | (uint32_t)(LL_DAC_OUTPUT_BUFFER_ENABLE)
                  | (uint32_t)(LL_DAC_OUTPUT_MODE_NORMAL)
                 ) << lut_ch_shift[index];

    /* Set MCR bits after CR bits because writing to MODE[] bits needs that EN and CEN bits are set to 0 */
    LL_DAC_WRITE_REG(p_instance, MCR, reg_value);

    /* Set default alignment for each channel */
    DAC_SetChannelAlignment(hdac, (hal_dac_channel_t)index, HAL_DAC_DATA_ALIGN_12_BITS_RIGHT);
  }
  hdac->global_state = HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED;
}

/**
  * @brief  Run the calibration of one DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The DAC channel.
  * @note   Calibration runs about 2 ms per channel.
  * @retval HAL_OK When HAL_OK the channel buffer has been calibrated correctly.
  * @retval HAL_INVALID_PARAM A parameter is invalid.
  */
hal_status_t HAL_DAC_CalibrateChannelBuffer(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  hal_status_t status = HAL_OK;
  DAC_TypeDef *p_instance;

  uint32_t output_buffer_mode;
  uint32_t trimming_value;
  uint32_t delta;
  uint32_t flag;

  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdac == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Store output buffer configuration */
  output_buffer_mode = LL_DAC_GetOutputBuffer(p_instance, lut_ch[channel]);

  LL_DAC_Disable(p_instance, lut_ch[channel]);

  /* Set mode in MCR for calibration */
  LL_DAC_SetOutputBuffer(p_instance, lut_ch[channel], LL_DAC_OUTPUT_BUFFER_ENABLE);

  /* Enable the selected DAC channel calibration */
  LL_DAC_SetMode(p_instance, lut_ch[channel], LL_DAC_MODE_CALIBRATION);

  /* Init trimming counter */
  /* Medium value */
  trimming_value = 0x10UL;
  delta = 0x08UL;
  while (delta != 0UL)
  {
    /* Set candidate trimming */
    LL_DAC_SetTrimmingValue(p_instance, lut_ch[channel], (trimming_value & DAC_CCR_OTRIM1));

    /* Wait minimum time needed between two calibration steps (OTRIM) */
    DAC_WaitMicroSecond(DAC_DELAY_TRIM_US);

    flag = LL_DAC_IsActiveFlag_CAL(p_instance, lut_ch[channel]);

    if (flag == 1U)
    {
      /* DAC_SR_CAL_FLAGx is HIGH try lower trimming */
      trimming_value -= delta;
    }
    else
    {
      /* DAC_SR_CAL_FLAGx is LOW try higher trimming */
      trimming_value += delta;
    }
    delta >>= 1UL;
  }

  /* Still need to check if  trimming_value calibration is current value or one step below */
  /* Indeed the first value that causes the DAC_SR_CAL_FLAGx bit to change from 0 to 1  */
  /* Set candidate trimming */
  LL_DAC_SetTrimmingValue(p_instance, lut_ch[channel], (trimming_value & DAC_CCR_OTRIM1));

  /* Wait minimum time needed between two calibration steps (OTRIM) */
  DAC_WaitMicroSecond(DAC_DELAY_TRIM_US);

  flag = LL_DAC_IsActiveFlag_CAL(p_instance, lut_ch[channel]);

  if (flag == 0UL)
  {
    /* Check trimming value below maximum */
    if (trimming_value < DAC_CCR_OTRIM1)
    {
      /* Trimming is actually one value more */
      trimming_value++;
    }
    /* Set right trimming */
    LL_DAC_SetTrimmingValue(p_instance, lut_ch[channel], (trimming_value & DAC_CCR_OTRIM1));
  }

  /* Disable the DAC channel calibration */
  LL_DAC_SetMode(p_instance, lut_ch[channel], LL_DAC_MODE_NORMAL_OPERATION);

  /* Restore configuration */
  LL_DAC_SetOutputBuffer(p_instance, lut_ch[channel], output_buffer_mode);

  return status;
}

/**
  * @brief  Set a trimming offset value.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  value DAC offset trimming value to be set. This parameter must be a number the range [1,31].
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetChannelBufferCalibrationValue(hal_dac_handle_t *hdac, hal_dac_channel_t channel, uint32_t value)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));
  ASSERT_DBG_PARAM(IS_DAC_TRIMMINGVALUE(value));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  p_instance = DAC_GET_INSTANCE(hdac);

  LL_DAC_SetTrimmingValue(p_instance, lut_ch[channel], (value & DAC_CCR_OTRIM1));

  return HAL_OK;
}

/**
  * @brief  Return the DAC trimming value.
  * @param  hdac DAC handle
  * @param  channel The selected DAC channel.
  * @retval Trimming value in range [0, 31].
  *
 */
uint32_t HAL_DAC_GetChannelBufferCalibrationValue(const hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  return LL_DAC_GetTrimmingValue(p_instance, lut_ch[channel]);
}

/**
  * @}
  */  /* Endgroup DAC_Exported_Functions_Group1_1 */

/** @defgroup DAC_Exported_Functions_Group1_2  Separate channel mode configuration functions
    This section provides functions allowing to:
      + Configure the DAC channels in separate channel mode.
  * @{
  */

/**
  * @brief  Configure the selected DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_config DAC configuration structure.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK  DAC instance has been correctly configured.
  * @retval HAL_INVALID_PARAM A parameter is invalid.
  */
hal_status_t HAL_DAC_SetConfigChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                      const hal_dac_channel_config_t *p_config)
{
  DAC_TypeDef *p_instance;
  uint32_t reg_value;
  uint32_t reg_mask;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  p_instance = DAC_GET_INSTANCE(hdac);
  ASSERT_DBG_PARAM(IS_DAC_ALL_INSTANCE(p_instance));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_TRIGGER(hdac, p_config->trigger));
  ASSERT_DBG_PARAM(IS_DAC_OUTPUT_BUFFER_STATE(p_config->output_buffer));
  ASSERT_DBG_PARAM(IS_DAC_CHIP_CONNECTION(p_config->output_connection));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));
  ASSERT_DBG_PARAM(IS_DAC_SIGN_FORMAT(p_config->data_sign_format));
  ASSERT_DBG_PARAM(IS_DAC_ALIGN(p_config->alignment));
  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  /* Calculate and store the channel data hold register address from the given channel and alignment */
  DAC_SetChannelAlignment(hdac, channel, p_config->alignment);

  /* Configure mode and trigger */

  reg_value = LL_DAC_READ_REG(p_instance, CR);
  reg_mask = ((DAC_CR_CEN1)     /* To set operation mode normal (disable calibration)*/
              | (DAC_CR_TSEL1)  /* To set trigger selection to HAL_DAC_TRIGGER_SOFTWARE */
              | (DAC_CR_TEN1)   /* To disable the trigger (ie TRIGGER_NONE) */
             ) << lut_ch_shift[channel];
  reg_value &= ~reg_mask ;

  if (p_config->trigger ==  HAL_DAC_TRIGGER_NONE)
  {
    /* Is set to HAL_DAC_TRIGGER_SOFTWARE to reset previous trigger */
  }
  else
  {
    reg_value |= ((uint32_t)(p_config->trigger)
                  | (DAC_CR_TEN1)   /* To enable the trigger */
                 ) << lut_ch_shift[channel];
  }

  LL_DAC_WRITE_REG(p_instance, CR, reg_value);

  /* Configure DAC channel signed format and output mode */
  /* Set MCR bits after CR bits because writing to MODE[] bits needs that EN and CEN bits are set to 0 */

  reg_value = LL_DAC_READ_REG(p_instance, MCR);

  reg_mask = ((DAC_MCR_SINFORMAT1) | (DAC_MCR_MODE1_1) | (DAC_MCR_MODE1_0)) << lut_ch_shift[channel];
  reg_value &= ~reg_mask ;
  reg_value |= ((uint32_t)(p_config->data_sign_format)
                | (uint32_t)(p_config->output_buffer)
                | (uint32_t)(p_config->output_connection)
               ) << lut_ch_shift[channel];

  LL_DAC_WRITE_REG(p_instance, MCR, reg_value);

  hdac->global_state = HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED;
  hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get configuration for a channel.
  * @param hdac Pointer to a hal_dac_handle_t structure.
  * @param channel The selected DAC channel.
  * @param p_config Pointer to a channel configuration structure parameters.
  */
void HAL_DAC_GetConfigChannel(const hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                              hal_dac_channel_config_t *p_config)
{
  DAC_TypeDef *p_instance;
  uint32_t reg_value;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], ((uint32_t)DAC_CHANNEL_STATE_ALL));

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Fill config structure parameter */
  reg_value = LL_DAC_READ_REG(p_instance, MCR);
  reg_value >>= (lut_ch_shift[channel]);

  p_config->data_sign_format  = (hal_dac_sign_format_t)((uint32_t)(reg_value & (DAC_MCR_SINFORMAT1)));
  p_config->output_buffer     = (hal_dac_output_buffer_status_t)((uint32_t)(reg_value & (DAC_MCR_MODE1_1)));
  p_config->output_connection = (hal_dac_output_connection_t)((uint32_t)(reg_value & (DAC_MCR_MODE1_0)));

  p_config->trigger = HAL_DAC_TRIGGER_NONE;
  if (LL_DAC_IsTriggerEnabled(p_instance, lut_ch[channel]) != 0UL)
  {
    p_config->trigger = (hal_dac_trigger_t) LL_DAC_GetTriggerSource(p_instance, lut_ch[channel]);
  }

  /* Calculate and return the alignment from the stored data hold register address */
  p_config->alignment = DAC_GET_ALIGNMENT_CHANNEL(hdac, channel);
}

/**
  * @brief  Set the data width and alignment for the DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  alignment  The alignment and data width:
  *                    12 bits right alignment, 12 bits left alignment, 8 bits right alignment.
  * @retval HAL_OK DAC instance has been correctly configured.
  */
hal_status_t HAL_DAC_SetChannelAlignment(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                         hal_dac_data_alignment_t alignment)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_ALIGN(alignment));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  DAC_SetChannelAlignment(hdac, channel, alignment);
  return HAL_OK;
}

/**
  * @brief  Get the data width and alignment for the DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval hal_dac_data_alignment_t The data width and alignment for the DAC channel.
  */
hal_dac_data_alignment_t HAL_DAC_GetChannelAlignment(const hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  hal_dac_data_alignment_t alignment;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  /* Calculate and return the alignment from the stored data hold register address */
  alignment = DAC_GET_ALIGNMENT_CHANNEL(hdac, channel);

  return alignment;
}

/**
  * @}
  */  /* Endgroup DAC_Exported_Functions_Group1_2 */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/** @defgroup DAC_Exported_Functions_Group1_3 Dual channel mode configuration functions
  *     This section provides functions allowing to:
      + Configure the DAC in dual channel mode.
  * @{
  */

/**
  * @brief Set dual channel configuration.
  * @param hdac Pointer to a hal_dac_handle_t structure.
  * @param p_config Pointer to a dual channel configuration structure parameters.
  * @retval HAL_OK DAC instance has been correctly configured.
  * @retval HAL_INVALID_PARAM A parameter is invalid.
  */
hal_status_t HAL_DAC_SetConfigDualChannel(hal_dac_handle_t *hdac, const hal_dac_dual_channel_config_t *p_config)
{
  DAC_TypeDef *p_instance;
  uint32_t reg_value;
  uint32_t reg_mask;
  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_ALIGN(p_config->alignment));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Calculate and store the dual data hold register address from the given alignment */
  /* Stored in [HAL_DAC_CHANNEL_1] as separate channel is not running when in dual channel */
  DAC_SetDualChannelAlignment(hdac, p_config->alignment);

  /* Configure mode and trigger */

  reg_value = LL_DAC_READ_REG(p_instance, CR);
  reg_mask = ((DAC_CR_CEN1)     /* To set operation mode normal (disable calibration)*/
              | (DAC_CR_TSEL1)  /* To set trigger selection to HAL_DAC_TRIGGER_SOFTWARE */
              | (DAC_CR_TEN1)   /* To disable the trigger (ie TRIGGER_NONE) */
             ) << lut_ch_shift[HAL_DAC_CHANNEL_1];
  reg_mask |= ((DAC_CR_CEN1)     /* To set operation mode normal (disable calibration)*/
               | (DAC_CR_TSEL1)  /* To set trigger selection to HAL_DAC_TRIGGER_SOFTWARE */
               | (DAC_CR_TEN1)   /* To disable the trigger (ie TRIGGER_NONE) */
              ) << lut_ch_shift[HAL_DAC_CHANNEL_2];
  reg_value &= ~reg_mask ;

  if (p_config->channel1_config.trigger == HAL_DAC_TRIGGER_NONE)
  {
    /* Is set to HAL_DAC_TRIGGER_SOFTWARE to reset previous trigger */
  }
  else
  {
    reg_value |= ((uint32_t)(p_config->channel1_config.trigger)
                  | (DAC_CR_TEN1)   /* To enable the trigger */
                 ) << lut_ch_shift[HAL_DAC_CHANNEL_1];
  }

  if (p_config->channel2_config.trigger == HAL_DAC_TRIGGER_NONE)
  {
    /* Is set to HAL_DAC_TRIGGER_SOFTWARE to reset previous trigger */
  }
  else
  {
    reg_value |= ((uint32_t)(p_config->channel2_config.trigger)
                  | (DAC_CR_TEN1)   /* To enable the trigger */
                 ) << lut_ch_shift[HAL_DAC_CHANNEL_2];
  }
  LL_DAC_WRITE_REG(p_instance, CR, reg_value);

  /* Configure DAC channel signed format and output mode */
  /* Set MCR bits after CR bits because writing to MODE[] bits needs that EN and CEN bits are set to 0 */

  reg_value = LL_DAC_READ_REG(p_instance, MCR);

  reg_mask  = ((DAC_MCR_SINFORMAT1) | (DAC_MCR_MODE1_1) | (DAC_MCR_MODE1_0)) << lut_ch_shift[HAL_DAC_CHANNEL_1];
  reg_mask |= ((DAC_MCR_SINFORMAT1) | (DAC_MCR_MODE1_1) | (DAC_MCR_MODE1_0)) << lut_ch_shift[HAL_DAC_CHANNEL_2];

  reg_value &= ~reg_mask ; /* Reset bits to be modified */
  reg_value |= ((uint32_t)(p_config->channel1_config.data_sign_format)
                | (uint32_t)(p_config->channel1_config.output_buffer)
                | (uint32_t)(p_config->channel1_config.output_connection)
               ) << lut_ch_shift[HAL_DAC_CHANNEL_1];

  reg_value |= ((uint32_t)(p_config->channel2_config.data_sign_format)
                | (uint32_t)(p_config->channel2_config.output_buffer)
                | (uint32_t)(p_config->channel2_config.output_connection)
               ) << lut_ch_shift[HAL_DAC_CHANNEL_2];

  LL_DAC_WRITE_REG(p_instance, MCR, reg_value);

  hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;

  return HAL_OK;
}

/**
  * @brief Get dual channel configuration.
  * @param hdac Pointer to a hal_dac_handle_t structure.
  * @param p_config Pointer to a dual channel configuration structure parameters.
  */
void HAL_DAC_GetConfigDualChannel(const hal_dac_handle_t *hdac, hal_dac_dual_channel_config_t *p_config)
{
  DAC_TypeDef *p_instance;
  uint32_t reg_value;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Retrieve and fill config from register for channel 1 and channel 2 */

  reg_value = LL_DAC_READ_REG(p_instance, MCR);
  p_config->channel1_config.data_sign_format  = (hal_dac_sign_format_t)((uint32_t)(reg_value & (DAC_MCR_SINFORMAT1)));
  p_config->channel1_config.output_buffer     = (hal_dac_output_buffer_status_t)
                                                ((uint32_t)(reg_value & (DAC_MCR_MODE1_1)));
  p_config->channel1_config.output_connection = (hal_dac_output_connection_t)
                                                ((uint32_t)(reg_value & (DAC_MCR_MODE1_0)));

  reg_value >>= (lut_ch_shift[HAL_DAC_CHANNEL_2]);
  p_config->channel2_config.data_sign_format  = (hal_dac_sign_format_t)((uint32_t)(reg_value & (DAC_MCR_SINFORMAT1)));
  p_config->channel2_config.output_buffer     = (hal_dac_output_buffer_status_t)
                                                ((uint32_t)(reg_value & (DAC_MCR_MODE1_1)));
  p_config->channel2_config.output_connection = (hal_dac_output_connection_t)
                                                ((uint32_t)(reg_value & (DAC_MCR_MODE1_0)));

  reg_value  = LL_DAC_READ_REG(p_instance, CR);
  p_config->channel1_config.trigger = HAL_DAC_TRIGGER_NONE;
  /* Update for HAL_DAC_TRIGGER_NONE (ie: trigger bit CR.TENx not enabled)*/
  if ((reg_value & (DAC_CR_TEN1)) != 0UL)
  {
    p_config->channel1_config.trigger = (hal_dac_trigger_t)((uint32_t)(reg_value & (DAC_CR_TSEL1)));
  }

  p_config->channel2_config.trigger = HAL_DAC_TRIGGER_NONE;
  if ((reg_value & (DAC_CR_TEN2)) != 0UL)
  {
    p_config->channel2_config.trigger = (hal_dac_trigger_t)((uint32_t)((reg_value & (DAC_CR_TSEL2))
                                                                       >> (lut_ch_shift[HAL_DAC_CHANNEL_2])));
  }

  /* Retrieve alignment */
  /* Calculate and return the alignment from the stored data hold register address */
  p_config->alignment = DAC_GET_ALIGNMENT_DUAL(hdac);
}

/**
  * @brief Set dual channel alignment.
  * @param hdac Pointer to a hal_dac_handle_t structure.
  * @param  alignment  The alignment and data width:
  *                    12 bits right alignment, 12 bits left alignment, 8 bits right alignment.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetDualChannelAlignment(hal_dac_handle_t *hdac,  hal_dac_data_alignment_t alignment)
{
  ASSERT_DBG_PARAM(IS_DAC_ALIGN(alignment));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  /* Calculate and store the dual data hold register address from the given alignment */
  /* Stored in [HAL_DAC_CHANNEL_1] as separate channel is not running when in dual channel */
  DAC_SetDualChannelAlignment(hdac, alignment);

  return HAL_OK;
}

/**
  * @brief Set dual channel alignment.
  * @param hdac Pointer to a hal_dac_handle_t structure.
  * @retval alignment  The alignment and data width:
  *                    12 bits right alignment, 12 bits left alignment, 8 bits right alignment.
  */
hal_dac_data_alignment_t HAL_DAC_GetDualChannelAlignment(const hal_dac_handle_t *hdac)
{
  hal_dac_data_alignment_t alignment;

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  /* Retrieve alignment from the dual data hold register address stored in [HAL_DAC_CHANNEL_1] */
  alignment = DAC_GET_ALIGNMENT_DUAL(hdac);

  return alignment ;
}

/**
  * @}
  */   /* Endgroup DAC_Exported_Functions_Group1_3 */

#endif /* USE_HAL_DAC_DUAL_CHANNEL */
#endif /* DAC_NB_OF_CHANNEL */

/** @defgroup DAC_Exported_Functions_Group2_1 Separate channel mode, input and output operation functions
     This section provides functions allowing to:
   + Enable a "software trigger" conversion,
   + Start conversion in separate channel mode,
   + Stop conversion in separate channel mode,
   + Start conversion and enable DMA transfer in separate channel mode,
   + Stop conversion and disable DMA transfer in separate channel mode,
   + Set the data in holding register for DAC channel in separate channel mode,
   + Get the converted data in separate channel mode.

  * @{
  */

/**
  * @brief  DAC channel software trigger conversion.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK or HAL_ERROR if software trigger is not enabled.
  */
hal_status_t HAL_DAC_TrigSWConversionChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  p_instance = DAC_GET_INSTANCE(hdac);

  if (LL_DAC_IsTriggerSWEnabled(p_instance, lut_ch[channel]) != 0U)
  {
    /* Enable the selected DAC software conversion */
    LL_DAC_TrigSWConversion(p_instance, lut_ch[channel]);
    status = HAL_OK;
  }

  return status;
}

/**
  * @brief  Enable DAC and start conversion of channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_StartChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  hdac->channel_state[(int32_t) channel] = HAL_DAC_CHANNEL_STATE_ACTIVE;

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  hdac->last_error_codes[channel] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

  LL_DAC_Enable(p_instance, lut_ch[channel]);
  /* Ensure minimum wait before using peripheral after enabling it */
  DAC_WaitMicroSecond(DAC_DELAY_STARTUP_US);

  return HAL_OK;
}

/**
  * @brief  Disable DAC and stop conversion of channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_StopChannel(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE);

  p_instance = DAC_GET_INSTANCE(hdac);

  LL_DAC_Disable(p_instance, lut_ch[channel]);

  hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Set the data holding register value for DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  data Data to be loaded in the selected data holding register.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetChannelData(hal_dac_handle_t *hdac, hal_dac_channel_t channel, uint32_t data)
{
  DAC_TypeDef *p_instance;
  uint32_t mask;
  uint32_t tmp_reg;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  p_instance = DAC_GET_INSTANCE(hdac);

  /* In case DMA Double data mode is activated, DATA range is almost full uin32_t one: no check */
  if (LL_DAC_IsDMADoubleDataModeEnabled(p_instance, lut_ch[channel]) == 0U)
  {
    ASSERT_DBG_PARAM(IS_DAC_DATA(data));
  }
  else
  {
    ASSERT_DBG_PARAM(IS_DAC_DATA_DOUBLE_MODE(data));
  }

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)DAC_CHANNEL_STATE_ALL);

  /**
    * Set the DAC channel selected data holding register.
    * The tmp_reg variable is needed to avoid changing other bits (ex: DAC_DHR12L2).
    * The mask takes into account those single mode alignments:
    * 0x000000FF for 8BR, 0x00000FFF for 12BR or 0x0000FFF0  for 12BL.
    */
  mask = 0x0000FFFF;
  tmp_reg = *(volatile uint32_t *) hdac->channel_dhr_address[channel];
  tmp_reg &= ~mask;
  tmp_reg = (tmp_reg | (data & mask));
  *(volatile uint32_t *) hdac->channel_dhr_address[channel] = tmp_reg;

  return HAL_OK;
}

/**
  * @brief  Return the last data output value of the selected DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval The selected DAC channel data output value.
  */
uint32_t HAL_DAC_GetChannelData(const hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  p_instance = DAC_GET_INSTANCE(hdac);
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)DAC_CHANNEL_STATE_ALL);

  return (LL_DAC_RetrieveOutputData(p_instance, lut_ch[channel]));
}

#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief  Set the link between DAC channel and a DMA handler.
  * @param  hdac Pointer to hal_dac_handle_t structure.
  * @param  hdma Pointer to the DMA handler to be linked with the DAC channel.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma, hal_dac_channel_t channel)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

  DAC_SetChannelDMA(hdac, hdma, channel);
  return HAL_OK;
}

/**
  * @brief  Enable DAC and start conversion of channel with DMA.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  p_data The source buffer address.
  * @param  size_byte The number of bytes of data to be transferred from memory to DAC peripheral
  * @retval HAL_OK DAC instance has been correctly configured with the DMA.
  * @retval HAL_ERROR An internal inconsistency error or a parameter is invalid.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API.
  */
hal_status_t HAL_DAC_StartChannel_DMA(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                      uint32_t size_byte)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hdac, channel_state[channel], HAL_DAC_CHANNEL_STATE_IDLE, HAL_DAC_CHANNEL_STATE_ACTIVE);

  status =  DAC_StartChannel_DMA_Opt(hdac, channel, p_data, size_byte, (uint32_t)HAL_DAC_OPT_DMA_IT_DEFAULT);

  if (status != HAL_OK)
  {
    /* Revert DAC channel state */
    hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Enable DAC and start conversion of channel with DMA and optional interruption.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  p_data The source buffer address.
  * @param  size_byte The number of bytes of data to be transferred from memory to DAC peripheral
  * @param  dac_opt_interrupt The DAC optional interrupt flag.
  * @retval HAL_OK DAC instance has been correctly configured with the DMA.
  * @retval HAL_ERROR An internal inconsistency error or a parameter is invalid.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API.
  */
hal_status_t HAL_DAC_StartChannel_DMA_Opt(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                          uint32_t size_byte, uint32_t dac_opt_interrupt)
{
  hal_status_t status;
  hal_dac_channel_state_t new_channel_state;

  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_DAC_DMA_VALID_SILENT_MODE(hdac->dma_ch[channel], dac_opt_interrupt));
#endif /* USE_HAL_DMA_LINKEDLIST */

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (dac_opt_interrupt == HAL_DAC_OPT_DMA_IT_SILENT)
  {
    new_channel_state = HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    new_channel_state = HAL_DAC_CHANNEL_STATE_ACTIVE;
  }
  HAL_CHECK_UPDATE_STATE(hdac, channel_state[channel], HAL_DAC_CHANNEL_STATE_IDLE, new_channel_state);

  status = DAC_StartChannel_DMA_Opt(hdac, channel, p_data, size_byte, dac_opt_interrupt);

  if (status != HAL_OK)
  {
    /* Revert DAC channel state */
    hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Disable DAC and stop conversion of channel with DMA.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK DMA successfully stopped.
  */
hal_status_t HAL_DAC_StopChannel_DMA(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;
  hal_dma_handle_t *p_hdma;
  hal_status_t status;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[channel],
                   (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE | (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT);

  p_instance = DAC_GET_INSTANCE(hdac);
  p_hdma = hdac->dma_ch[channel];

  LL_DAC_DisableDMAReq(p_instance, lut_ch[channel]);

  LL_DAC_DisableIT_DMAUDR(p_instance, lut_ch_dma_underrun_it[channel]);

  LL_DAC_Disable(p_instance, lut_ch[channel]);

  if (hdac->channel_state[channel] == HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT)
  {
    (void)HAL_DMA_Abort(p_hdma); /* (void) because irrelevant return status in silent mode */
    hdac->global_state = HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED;
    hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;
  }
  else
  {
    p_hdma->p_xfer_abort_cb = DAC_DMA_ChStopCplt;
    status = HAL_DMA_Abort_IT(p_hdma);
    /* DAC global_state and channel_state are changed inside DAC_DMA_ChStopCplt() */
    if (status != HAL_OK)
    {
      DAC_DMA_ChStopCplt(p_hdma);
    }
  }

  return HAL_OK;
}

#endif /* USE_HAL_DAC_DMA */

/**
  * @}
  */   /* Endgroup DAC_Exported_Functions_Group2_1 */

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/** @defgroup DAC_Exported_Functions_Group2_2 Dual mode, input and output operation functions
     This section provides functions allowing to:
   + Enable a "software trigger" for dual channel conversion,
   + Start conversion in dual channel mode,
   + Stop conversion in dual channel mode,
   + Start conversion and enable DMA transfer in dual channel mode,
   + Stop conversion and disable DMA transfer in dual channel mode,
   + Set the data in holding register for DAC channel in dual channel mode,
   + Get the converted data in dual channel mode.
  * @{
  */

#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief  DAC dual channel software trigger conversion.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval HAL_OK or HAL_ERROR if software trigger is not enabled.
  */
hal_status_t HAL_DAC_TrigSWConversionDualChannel(hal_dac_handle_t *hdac)
{
  uint32_t tmp_swtrig_ch = 0UL;
  DAC_TypeDef *p_instance;
  hal_status_t status = HAL_ERROR;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Check both channels to see whether the software trigger is enabled */
  if (LL_DAC_IsTriggerSWEnabled(p_instance, LL_DAC_CHANNEL_1) != 0UL)
  {
    tmp_swtrig_ch |= LL_DAC_CHANNEL_1;
    status = HAL_OK;
  }
  if (LL_DAC_IsTriggerSWEnabled(p_instance, LL_DAC_CHANNEL_2) != 0UL)
  {
    tmp_swtrig_ch |= LL_DAC_CHANNEL_2;
    status = HAL_OK;
  }

  /* Enable the selected DAC software conversion*/
  LL_DAC_TrigSWConversion(p_instance, (uint32_t)(tmp_swtrig_ch));

  return status;
}

/**
  * @brief  Enable DAC and start conversion of both channels in dual channel mode.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_StartDualChannel(hal_dac_handle_t *hdac)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE;
  hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_ACTIVE;
  hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_ACTIVE;

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  hdac->last_error_codes[HAL_DAC_CHANNEL_1] = (uint16_t)HAL_DAC_ERROR_NONE;
  hdac->last_error_codes[HAL_DAC_CHANNEL_2] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

  /* Enable the Peripheral */
  LL_DAC_DualChannelEnable(p_instance);

  /* Ensure minimum wait before using peripheral after enabling it */
  DAC_WaitMicroSecond(DAC_DELAY_STARTUP_US);

  return HAL_OK;
}

/**
  * @brief  Disable DAC and stop conversion of both channels.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_StopDualChannel(hal_dac_handle_t *hdac)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  p_instance = DAC_GET_INSTANCE(hdac);

  LL_DAC_DualChannelDisable(p_instance);

  hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;
  hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_IDLE;
  hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Set the specified data holding register value for dual DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure that contains
  *               the configuration information for the specified DAC.
  * @param  data Data for DAC channel 1 and DAC channel 2 to be loaded in the selected data holding register.
  * @warning  In dual mode, a unique register access is required to write in both
  *           DAC channels at the same time. The data value given by the user must be a concatenation of channel 1
  *           data and channel 2 data, according to the used alignment as described in the reference manual.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetDualChannelData(hal_dac_handle_t *hdac, uint32_t data)
{
  volatile uint32_t *tmp;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM(IS_DAC_DATA_DUAL(data));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  /* Set the dual data holding register */
  tmp = hdac->channel_dhr_address[(uint32_t)HAL_DAC_CHANNEL_1]; /* dual dhr address is recorded in index 0 */
  *(volatile uint32_t *) tmp = data;

  return HAL_OK;
}

/**
  * @brief  Return the last dual data output value of the dual channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval The dual channel data output value.
  */
uint32_t HAL_DAC_GetDualChannelData(const hal_dac_handle_t *hdac)
{
  uint32_t tmp;
  const DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  p_instance = DAC_GET_INSTANCE(hdac);

  tmp  = LL_DAC_READ_REG(p_instance, DOR1);
  tmp |= LL_DAC_READ_REG(p_instance, DOR2) << 16UL;

  /* Returns the DAC dual channel data output value */
  return tmp;
}
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief  Set the link between DAC and a DMA handler in dual channel mode.
  * @param  hdac Pointer to hal_dac_handle_t structure.
  * @param  hdma Pointer to the DMA handler to be linked with the DAC in dual channel mode.
  * @param  dma_requester_channel The channel that does the request to the DMA.
  * @warning  DMA dual channel is set in dma_ch[HAL_DAC_CHANNEL_1] so it is not possible to used simultaneously
  *           DMA dual channel mode and DMA in single channel mode.
  *           To go back in DMA single channel mode, user must call HAL_DAC_SetChannelDMA().
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_SetDualChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma,
                                       hal_dac_channel_t dma_requester_channel)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, dma_requester_channel));

  hdac->dual_channel_dma_requester = dma_requester_channel;

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);

  DAC_SetChannelDMA(hdac, hdma, dma_requester_channel);
  return HAL_OK;
}

/**
  * @brief  Enable DAC, and start conversion with a DMA, of both channels of the same DAC, with optional interruption.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_data The destination peripheral Buffer address.
  * @param  size_byte The number of data to be transferred from memory to DAC peripheral.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  */
hal_status_t HAL_DAC_StartDualChannel_DMA(hal_dac_handle_t *hdac, const void *p_data, uint32_t size_byte)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hdac, global_state, HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED, HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  HAL_CHECK_UPDATE_STATE(hdac, channel_state[HAL_DAC_CHANNEL_1], HAL_DAC_CHANNEL_STATE_IDLE,
                         HAL_DAC_CHANNEL_STATE_ACTIVE);
  HAL_CHECK_UPDATE_STATE(hdac, channel_state[HAL_DAC_CHANNEL_2], HAL_DAC_CHANNEL_STATE_IDLE,
                         HAL_DAC_CHANNEL_STATE_ACTIVE);

  status = DAC_StartDualChannel_DMA_Opt(hdac, p_data, size_byte, (uint32_t)HAL_DAC_OPT_DMA_IT_DEFAULT);

  if (status != HAL_OK)
  {
    /* Revert DAC state and DAC channel state */
    hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;
    hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_IDLE;
    hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Enable DAC, and start conversion with a DMA, of both channels of the same DAC, with optional interruption.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_data The destination peripheral Buffer address.
  * @param  size_byte The number of data to be transferred from memory to DAC peripheral.
  * @param  dac_opt_interrupt The DAC optional interrupt flag.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  */
hal_status_t HAL_DAC_StartDualChannel_DMA_Opt(hal_dac_handle_t *hdac, const void *p_data, uint32_t size_byte,
                                              uint32_t dac_opt_interrupt)
{
  hal_status_t status;
  hal_dac_channel_state_t new_channel_state;

  ASSERT_DBG_PARAM((hdac != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_data == NULL) || (size_byte == 0UL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2], (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_DAC_DMA_VALID_SILENT_MODE(hdac->dma_ch[hdac->dual_channel_dma_requester], dac_opt_interrupt));
#endif /* USE_HAL_DMA_LINKEDLIST */

  HAL_CHECK_UPDATE_STATE(hdac, global_state, HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED, HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

#if defined (USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  if (dac_opt_interrupt == HAL_DAC_OPT_DMA_IT_SILENT)
  {
    new_channel_state = HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT;
  }
  else
#endif /* USE_HAL_DMA_LINKEDLIST */
  {
    new_channel_state = HAL_DAC_CHANNEL_STATE_ACTIVE;
  }
  HAL_CHECK_UPDATE_STATE(hdac, channel_state[HAL_DAC_CHANNEL_1], HAL_DAC_CHANNEL_STATE_IDLE, new_channel_state);
  HAL_CHECK_UPDATE_STATE(hdac, channel_state[HAL_DAC_CHANNEL_2], HAL_DAC_CHANNEL_STATE_IDLE, new_channel_state);

  status = DAC_StartDualChannel_DMA_Opt(hdac, p_data, size_byte, (uint32_t)dac_opt_interrupt);

  if (status != HAL_OK)
  {
    /* Revert DAC state and DAC channel state */
    hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;
    hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_IDLE;
    hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_IDLE;
  }

  return status;
}

/**
  * @brief  Disable DAC, and stop conversion with DMA, for both channels.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval HAL_OK DMA successfully stopped.
  */
hal_status_t HAL_DAC_StopDualChannel_DMA(hal_dac_handle_t *hdac)
{
  DAC_TypeDef *p_instance;
  hal_dma_handle_t *p_hdma;
  hal_status_t status;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED |
                   (uint32_t)HAL_DAC_STATE_DUAL_CHANNEL_ACTIVE);

  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_1],
                   (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE | (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT);
  ASSERT_DBG_STATE(hdac->channel_state[HAL_DAC_CHANNEL_2],
                   (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE | (uint32_t)HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT);

  p_instance = DAC_GET_INSTANCE(hdac);

  p_hdma = hdac->dma_ch[hdac->dual_channel_dma_requester];

  LL_DAC_DisableDMAReq(p_instance, lut_ch[hdac->dual_channel_dma_requester]);

  LL_DAC_DisableIT_DMAUDR(p_instance, lut_ch_dma_underrun_it[hdac->dual_channel_dma_requester]);

  LL_DAC_DualChannelDisable(p_instance);

  if (hdac->channel_state[HAL_DAC_CHANNEL_1] == HAL_DAC_CHANNEL_STATE_ACTIVE_SILENT) /* CHANNEL_2 has same state */
  {
    (void)HAL_DMA_Abort(p_hdma); /* (void) because irrelevant return status in silent mode */
    hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;
    hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_IDLE;
    hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_IDLE;
  }
  else
  {
    p_hdma->p_xfer_abort_cb = DAC_DMA_DualChannelStopCplt;
    status = HAL_DMA_Abort_IT(p_hdma);
    /* DAC global_state and channel_state are changed inside DAC_DMA_DualChannelStopCplt() */
    if (status != HAL_OK)
    {
      DAC_DMA_DualChannelStopCplt(p_hdma);
    }
  }
  return HAL_OK;
}

#endif /* USE_HAL_DAC_DMA */
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

/**
  * @}
  */   /* Endgroup DAC_Exported_Functions_Group2_2 */
#endif /* DAC_NB_OF_CHANNEL */

/** @defgroup DAC_Exported_Functions_Group3 Peripheral control functions
  This section provides functions allowing to set and configure the DAC main features:
    + the DMA double data mode,
    + the autonomous mode,
    + the configuration for the sample and hold mode,
    + the triangle wave addition,
    + the noise wave addition.
  * @{
  */
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief  Enable the DAC DMA double data mode.
  * @param  hdac Pointer to a hal_dac_handle_t.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK.
  */

hal_status_t HAL_DAC_EnableChannelDMADoubleDataMode(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  /* To change from double data to single data mode or vice versa: the DAC channel must be disabled */
  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE((hdac->channel_state[(uint32_t)channel]), (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Update the MCR register */
  LL_DAC_EnableDMADoubleDataMode(p_instance, lut_ch[channel]);
  return HAL_OK;
}

/**
  * @brief  Disable the DAC DMA double data mode.
  * @param  hdac Pointer to a hal_dac_handle_t.
  * @param  channel The selected DAC channel.
  * @retval HAL_OK.
  */
hal_status_t HAL_DAC_DisableChannelDMADoubleDataMode(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  /* To change from double data to single data mode or vice versa: the DAC channel must be disabled */
  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE((hdac->channel_state[(uint32_t)channel]), (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Update the MCR register */
  LL_DAC_DisableDMADoubleDataMode(p_instance, lut_ch[channel]);

  return HAL_OK;
}

/**
  * @brief  Check whether the DAC DMA double data mode is enabled.
  * @param  hdac Pointer to a hal_dac_handle_t.
  * @param  channel The selected DAC channel.
  * @retval hal_dac_dma_double_data_mode_status_t State of DMA double data mode:
            + HAL_DAC_DMA_DOUBLE_DATA_MODE_ENABLED
            + HAL_DAC_DMA_DOUBLE_DATA_MODE_DISABLED.
  */
hal_dac_dma_double_data_mode_status_t HAL_DAC_IsEnabledChannelDMADoubleDataMode(hal_dac_handle_t *hdac,
                                                                                hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  return ((hal_dac_dma_double_data_mode_status_t) LL_DAC_IsDMADoubleDataModeEnabled(p_instance, lut_ch[channel]));
}
#endif /* defined(USE_HAL_DAC_DMA) */

/**
  * @brief Set sample and hold configuration for a channel.
  * @param hdac Pointer to a hal_dac_handle_t structure that contains
  *        the configuration information for the specified DAC.
  * @param channel The selected DAC channel.
  * @param p_config Pointer to sample and hold mode structure parameters.
  * @retval HAL_OK DAC  Instance has been correctly configured.
  * @retval HAL_ERROR   Internal timeout (to long time before writing in DAC_SHSRx has been completed).
  * @retval HAL_INVALID_PARAM  If p_config is NULL.
  */
hal_status_t HAL_DAC_SetConfigChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                   const hal_dac_channel_sample_and_hold_config_t *p_config)
{
  DAC_TypeDef *p_instance;
  uint32_t tickstart;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_PARAM(IS_DAC_SAMPLE_TIME(p_config->sample_time_cycle));
  ASSERT_DBG_PARAM(IS_DAC_HOLD_TIME(p_config->hold_time_cycle));
  ASSERT_DBG_PARAM(IS_DAC_REFRESH_TIME(p_config->refresh_time_cycle));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Sample and hold configuration */
  tickstart = HAL_GetTick();
  /* SHSR1 for channel x can be written when BWSTx is cleared */
  while (LL_DAC_IsActiveFlag_BWST(p_instance, lut_ch[channel]) != 0UL)
  {
    /* Check for the Timeout */
    if ((HAL_GetTick() - tickstart) > DAC_TIMEOUT_FOR_BWST_MS)
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_DAC_IsActiveFlag_BWST(p_instance, lut_ch[channel]) != 0UL)
      {
        return HAL_ERROR;
      }
    }
  }

  LL_DAC_SetSampleAndHoldSampleTime(p_instance, lut_ch[channel],
                                    (p_config->sample_time_cycle & DAC_SHSR1_TSAMPLE1_Msk));

  LL_DAC_SetSampleAndHoldHoldTime(p_instance, lut_ch[channel],
                                  (p_config->hold_time_cycle & DAC_SHHR_THOLD1_Msk));

  LL_DAC_SetSampleAndHoldRefreshTime(p_instance, lut_ch[channel],
                                     (p_config->refresh_time_cycle & DAC_SHRR_TREFRESH1_Msk));

  return HAL_OK;
}

/**
  * @brief Get sample and hold  configuration for a channel.
  * @param hdac Pointer to a hal_dac_handle_t structure that contains
  *        the configuration information for the specified DAC.
  * @param channel The selected DAC channel.
  * @param p_config Pointer to sample and hold structure parameters.
  */
void HAL_DAC_GetConfigChannelSampleAndHold(const hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                           hal_dac_channel_sample_and_hold_config_t *p_config)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  p_config->sample_time_cycle  = LL_DAC_GetSampleAndHoldSampleTime(p_instance, lut_ch[channel]);
  p_config->hold_time_cycle    = LL_DAC_GetSampleAndHoldHoldTime(p_instance, lut_ch[channel]);
  p_config->refresh_time_cycle = LL_DAC_GetSampleAndHoldRefreshTime(p_instance, lut_ch[channel]);
}

/**
  * @brief Enable the DAC sample and hold mode for a channel.
  * @param hdac Pointer to a hal_dac_handle_t.
  * @param channel The selected DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_EnableChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE((hdac->channel_state[(uint32_t)channel]), (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Write the sample_and_hold_mode in DAC_MCR register */
  LL_DAC_SetOutputMode(p_instance, lut_ch[channel], (uint32_t)HAL_DAC_SAMPLE_AND_HOLD_ENABLED);

  return HAL_OK;
}

/**
  * @brief Disable the DAC sample and hold mode for a channel.
  * @param hdac Pointer to a hal_dac_handle_t.
  * @param channel The selected DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_DisableChannelSampleAndHold(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  /* Write the sample_and_hold_mode in DAC_MCR register */
  LL_DAC_SetOutputMode(p_instance, lut_ch[channel], (uint32_t)HAL_DAC_SAMPLE_AND_HOLD_DISABLED);

  return HAL_OK;
}

/**
  * @brief Check whether the DAC sample-and-hold mode is enabled for a channel.
  * @param hdac Pointer to a hal_dac_handle_t.
  * @param channel The selected DAC channel.
  * @retval hal_dac_sample_and_hold_status_t Sample and hold state:
            + HAL_DAC_SAMPLE_AND_HOLD_ENABLED
            + HAL_DAC_SAMPLE_AND_HOLD_DISABLED.
  */
hal_dac_sample_and_hold_status_t HAL_DAC_IsEnabledChannelSampleAndHold(hal_dac_handle_t *hdac,
                                                                       hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  p_instance = DAC_GET_INSTANCE(hdac);

  return ((hal_dac_sample_and_hold_status_t) LL_DAC_GetOutputMode(p_instance, lut_ch[channel]));
}

/**
  * @brief  Enable the DAC channel adding triangle wave.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The DAC channel.
  * @param  amplitude The triangle wave amplitude.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_EnableChannelAddingTriangleWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                     hal_dac_wave_amplitude_t amplitude)
{
  DAC_TypeDef *p_instance;
  uint32_t cr_value;
  uint32_t cr_mask;

  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));
  ASSERT_DBG_PARAM(IS_DAC_WAVE_AMPLITUDE(amplitude));

  /* The MAMPx[3:0] bits must be configured before enabling the DAC, otherwise they cannot be changed.
    So the DAC channel must be disabled */
  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE((hdac->channel_state[(uint32_t)channel]), (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  p_instance = DAC_GET_INSTANCE(hdac);

  /** Set the triangle wave generation amplitude for the DAC channel,
    * and enable the triangle wave generation for the DAC channel
    */
  cr_value = LL_DAC_READ_REG(p_instance, CR);

  cr_mask = ((DAC_CR_MAMP1) | (DAC_CR_WAVE1)) << lut_ch_shift[channel];
  cr_value &= ~cr_mask ;
  cr_value |= ((uint32_t)(amplitude) | (uint32_t)(LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE)) << lut_ch_shift[channel];

  LL_DAC_WRITE_REG(p_instance, CR, cr_value);

  return HAL_OK;
}

/**
  * @brief  Disable the DAC channel adding triangle wave.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_DisableChannelAddingTriangleWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  p_instance = DAC_GET_INSTANCE(hdac);

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  /* Disable the triangle wave generation for the DAC channel */
  LL_DAC_SetWaveAutoGeneration(p_instance, lut_ch[channel], LL_DAC_WAVE_AUTO_GENERATION_NONE);

  return HAL_OK;
}

/**
  * @brief  Enable the DAC channel adding noise wave.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The DAC channel.
  * @param  amplitude Noise amplitude used for pseudo noise wave generation.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_EnableChannelAddingNoiseWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                                  hal_dac_wave_amplitude_t amplitude)
{
  DAC_TypeDef *p_instance;
  uint32_t cr_value;
  uint32_t cr_mask;

  ASSERT_DBG_PARAM((hdac != NULL));
  p_instance = DAC_GET_INSTANCE(hdac);

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));
  ASSERT_DBG_PARAM(IS_DAC_WAVE_AMPLITUDE(amplitude));

  /* The MAMPx[3:0] bits must be configured before enabling the DAC, otherwise they cannot be changed.
    So the DAC channel must be disabled */
  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);
  ASSERT_DBG_STATE((hdac->channel_state[(uint32_t)channel]), (uint32_t)HAL_DAC_CHANNEL_STATE_IDLE);

  /** Set the amplitude for the DAC channel LFSR used for noise wave generation
    * and enable the noise wave generation for the DAC channel
    */
  cr_value = LL_DAC_READ_REG(p_instance, CR);

  cr_mask = ((DAC_CR_MAMP1) | (DAC_CR_WAVE1)) << lut_ch_shift[channel];
  cr_value &= ~cr_mask ;
  cr_value |= ((uint32_t)(amplitude) | (uint32_t)(LL_DAC_WAVE_AUTO_GENERATION_NOISE)) << lut_ch_shift[channel];

  LL_DAC_WRITE_REG(p_instance, CR, cr_value);

  return HAL_OK;
}

/**
  * @brief  Disable the DAC channel adding noise wave.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The DAC channel.
  * @retval HAL_OK
  */
hal_status_t HAL_DAC_DisableChannelAddingNoiseWave(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  DAC_TypeDef *p_instance;

  ASSERT_DBG_PARAM((hdac != NULL));
  p_instance = DAC_GET_INSTANCE(hdac);

  ASSERT_DBG_PARAM(IS_DAC_CHANNEL(hdac, channel));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_ALL);

  /* Disable the noise wave generation for the selected DAC channel */
  LL_DAC_SetWaveAutoGeneration(p_instance, lut_ch[channel], LL_DAC_WAVE_AUTO_GENERATION_NONE);

  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions_Group4 Callback functions and callback register functions
  This section is about callbacks functions:
    + The weak callbacks functions.
    + The register callback functions.
  * @{
  */

/**
  * @brief  Conversion complete callback in non-blocking mode for the channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  */
__WEAK void HAL_DAC_ConvCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);
  STM32_UNUSED(channel);

  /* NOTE: This function must not be modified when the callback is needed,
            the HAL_DAC_ConvCpltCallback could be implemented in the user file.
   */
}

/**
  * @brief  Conversion half DMA transfer callback in non-blocking mode for the channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  */
__WEAK void HAL_DAC_ConvHalfCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);
  STM32_UNUSED(channel);

  /* NOTE: This function must not be modified when the callback is needed,
            the HAL_DAC_ConvHalfCpltCallback could be implemented in the user file.
   */
}

/**
  * @brief  DAC stop callback.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  */
__WEAK void HAL_DAC_StopCpltCallback(hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);
  STM32_UNUSED(channel);

  /* NOTE: This function must not be modified when the callback is needed,
           the HAL_DAC_StopCpltCallback could be implemented in the user file.
   */
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief  Conversion complete callback in non-blocking mode for dual channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  */
__WEAK void HAL_DAC_DualChannelConvCpltCallback(hal_dac_handle_t *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);

  /* NOTE: This function must not be modified when the callback is needed,
            the HAL_DAC_DualChannelConvCpltCallback could be implemented in the user file.
   */
}

/**
  * @brief  Conversion half DMA transfer callback in non-blocking mode for dual channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  */
__WEAK void HAL_DAC_DualChannelConvHalfCpltCallback(hal_dac_handle_t *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);

  /* NOTE: This function must not be modified when the callback is needed,
            the HAL_DAC_DualChannelConvHalfCpltCallback could be implemented in the user file.
   */
}

/**
  * @brief  DAC dual channel stop callback.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  */
__WEAK void HAL_DAC_DualChannelStopCpltCallback(hal_dac_handle_t *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);

  /* NOTE: This function must not be modified when the callback is needed,
           the HAL_DAC_DualChannelStopCpltCallback could be implemented in the user file.
   */
}
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  DAC error callback.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  */
__WEAK void HAL_DAC_ErrorCallback(hal_dac_handle_t *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hdac);

  /* NOTE: This function must not be modified when the callback is needed,
           the HAL_DAC_ErrorCallback could be implemented in the user file.
   */
}

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a user DAC callback to manage the completion conversion.
  *         To be used instead of the weak (overridden) predefined callback.
  * @param  hdac DAC handle
  * @param  p_callback Pointer to channel converter complete callback function.
  *
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM In case of a NULL pointer for parameter.
  */
hal_status_t HAL_DAC_RegisterConvCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_conv_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a user DAC callback to manage the half completion conversion.
  *         To be used instead of the weak (overridden) predefined callback.
  * @param  hdac DAC handle
  * @param  p_callback Pointer to channel converter half complete callback function.
  *
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM In case of a NULL pointer for parameter.
  */
hal_status_t HAL_DAC_RegisterConvHalfCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_conv_half_cplt_cb = p_callback;
  return HAL_OK;
}

/**
  * @brief  Register a user DAC stop completed callback.
  * @param  hdac Pointer to a hal_dac_handle_t
  * @param  p_callback Pointer to the DAC Stop completed callback
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_DAC_RegisterStopCpltCallback(hal_dac_handle_t *hdac, hal_dac_cb_t p_callback)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  /* Check the state */
  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_stop_cplt_cb = p_callback;

  return HAL_OK;
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief  Register a user DAC callback to manage the dual completion conversion.
  *         To be used instead of the weak (overridden) predefined callback.
  * @param  hdac DAC handle
  * @param  p_callback Pointer to the dual channel converter complete callback function.
  *
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM In case of a NULL pointer for parameter.
  */
hal_status_t HAL_DAC_RegisterDualChannelCpltCallback(hal_dac_handle_t *hdac, hal_dac_dual_channel_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_dual_channel_conv_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a user DAC callback to manage the dual half completion conversion.
  *         To be used instead of the weak (overridden) predefined callback.
  * @param  hdac DAC handle
  * @param  p_callback  Pointer to the dual channel converter half complete callback function.
  *
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM In case of a NULL pointer for parameter.
  */
hal_status_t HAL_DAC_RegisterDualChannelHalfCpltCallback(hal_dac_handle_t *hdac,
                                                         hal_dac_dual_channel_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_dual_channel_conv_half_cplt_cb = p_callback;

  return HAL_OK;
}

/**
  * @brief  Register a user DAC dual channel stop completed callback.
  * @param  hdac Pointer to a hal_dac_handle_t
  * @param  p_callback Pointer to the DAC dual channel stop completed callback function.
  * @retval HAL_OK Operation completed successfully
  * @retval HAL_INVALID_PARAM Invalid parameter
  */
hal_status_t HAL_DAC_RegisterDualChannelStopCpltCallback(hal_dac_handle_t *hdac, hal_dac_dual_channel_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_dual_channel_stop_cplt_cb = p_callback;

  return HAL_OK;
}
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Register a user DAC callback to manage error.
  *         To be used instead of the weak (overridden) predefined callback.
  * @param  hdac DAC handle
  * @param  p_callback Pointer to the DAC channel error callback function.
  *
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM In case of a NULL pointer for parameter.
  */
hal_status_t HAL_DAC_RegisterErrorCallback(hal_dac_handle_t *hdac, hal_dac_error_cb_t p_callback)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_PARAM((p_callback != NULL));

  ASSERT_DBG_STATE(hdac->global_state, (uint32_t)DAC_STATE_CONFIG);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((hdac == NULL) || (p_callback == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hdac->p_error_cb = p_callback;
  return HAL_OK;
}
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions_Group5 Peripheral State, kernel clock frequency, IRQ and Errors functions
    This subsection provides functions allowing to:
      + Check the DAC state,
      + Check the DAC channels states,
      + Get the kernel clock frequency,
      + IRQ handler,
      + Check the DAC Errors.
  * @{
  */

/**
  * @brief  return the DAC handle state.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval hal_dac_state_t DAC global state.
  */
hal_dac_state_t HAL_DAC_GetState(const hal_dac_handle_t *hdac)
{
  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  return hdac->global_state;
}

/**
  * @brief  return the DAC channel state.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @retval hal_dac_channel_state_t  Channel state.
  */
hal_dac_channel_state_t HAL_DAC_GetChannelState(const hal_dac_handle_t *hdac, hal_dac_channel_t channel)
{
  ASSERT_DBG_PARAM((hdac != NULL));

  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  return (hdac->channel_state[(uint32_t)channel]);
}

/** @brief  Return the peripheral clock frequency for DAC.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval uint32_t Frequency in Hz.
  *         0 if the source clock of the DAC is not configured or not ready.
  */
uint32_t HAL_DAC_GetClockFreq(const hal_dac_handle_t *hdac)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

#if !defined(USE_ASSERT_DBG_STATE) && !defined(USE_ASSERT_DBG_PARAM)
  STM32_UNUSED(hdac);
#endif /* USE_ASSERT_DBG_STATE or USE_ASSERT_DBG_PARAM*/

  return HAL_RCC_DAC_GetKernelClkFreq(DAC_GET_INSTANCE(hdac));
}

/**
  * @brief  Handle DAC interrupt request.
  *         This function is called when an interruption for DMA underrun error occurs.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  */
void HAL_DAC_IRQHandler(hal_dac_handle_t *hdac)
{
  DAC_IRQHandlerCH(hdac, HAL_DAC_CHANNEL_1);
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL > 1)
  DAC_IRQHandlerCH(hdac, HAL_DAC_CHANNEL_2);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Returns the last DAC error codes in a bits field.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @retval Last error code, coded on an uint32_t bits field,
  *         it can be :
  *         + HAL_DAC_ERROR_NONE
  *         or a combinaison of the following values:
  *         + HAL_DAC_ERROR_DMA_UNDERRUN_CH1
  *         + HAL_DAC_ERROR_DMA_CH1
  *         + HAL_DAC_ERROR_DMA_UNDERRUN_CH2
  *         + HAL_DAC_ERROR_DMA_CH2
  */
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
uint32_t HAL_DAC_GetLastErrorCodes(const hal_dac_handle_t *hdac)
{
  uint32_t tmp;

  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  /* In two steps as last_error_codes is volatile */
  tmp  = hdac->last_error_codes[HAL_DAC_CHANNEL_1];
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  tmp |= hdac->last_error_codes[HAL_DAC_CHANNEL_2];
#endif /* DAC_NB_OF_CHANNEL */
  return tmp;
}
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

/**
  * @}
  */
/** @defgroup DAC_Exported_Functions_Group6  User Data API functions
    This subsection provides functions allowing to:
      + Set a user data pointer (ex: a user context) in a DAC handle,
      + Get a user data pointer (ex: a user context) from a DAC handle.
    @note A typical usage is to set user data pointer before starting a data conversion,<br>
          then retrieve it within the user conversion completion callback.
  * @{
  */
#if defined(USE_HAL_DAC_USER_DATA) && (USE_HAL_DAC_USER_DATA == 1)
/**
  * @brief  Store user data pointer into the DAC handle.
  * @param  hdac Pointer to a hal_dac_handle_t.
  * @param  p_user_data Pointer to the user data.
  */
void HAL_DAC_SetUserData(hal_dac_handle_t *hdac, const void *p_user_data)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  hdac->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve user data pointer from the DAC handle.
  * @param  hdac Pointer to a hal_dac_handle_t.
  * @retval (void*) the pointer to the user data, when previously set by HAL_DAC_SetUserData().
  * @retval NULL other way.
  */
const void *HAL_DAC_GetUserData(const hal_dac_handle_t *hdac)
{
  ASSERT_DBG_PARAM((hdac != NULL));
  ASSERT_DBG_STATE(hdac->global_state, DAC_STATE_ALL);

  return (hdac->p_user_data);
}
#endif /* USE_HAL_DAC_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DAC_Private_Functions
  * @{
  */

/**
  * @brief  Ensure a minimum wait of delay_us in microsecond.
  * @param  delay_us The delay in microsecond.
  */
static void DAC_WaitMicroSecond(uint32_t delay_us)
{
  volatile uint32_t wait_loop_index;

  /**
    * Wait loop initialization and execution.
    * Note: Variable divided by 2 to compensate partially CPU processing cycles, scaling in us split to not exceed 32
    *       bits register capacity and handle low frequency.
    */
  wait_loop_index = ((delay_us / 10UL) * ((SystemCoreClock / (100000UL * 2UL)) + 1UL));
  while (wait_loop_index > 0UL)
  {
    wait_loop_index--;
  }
}

/**
  * @brief  Set the data width and alignment for the DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  alignment  The alignment and data width:
  *                    12 bits right alignment, 12 bits left alignment, 8 bits right alignment.
  */
static void DAC_SetChannelAlignment(hal_dac_handle_t *hdac, hal_dac_channel_t channel,
                                    hal_dac_data_alignment_t alignment)
{
  DAC_TypeDef *p_instance = DAC_GET_INSTANCE(hdac);

  /**
    * Computes and stores the channel data hold register address from the given channel and alignment
    * One among those register addresses:
    * uint32_t DHR12R1;   DAC channel1 12-bit right aligned data holding register, Address offset: 0x08
    * uint32_t DHR12L1;   DAC channel1 12-bit left  aligned data holding register, Address offset: 0x0C
    * uint32_t DHR8R1;    DAC channel1  8-bit right aligned data holding register, Address offset: 0x10
    * uint32_t DHR12R2;   DAC channel2 12-bit right aligned data holding register, Address offset: 0x14
    * uint32_t DHR12L2;   DAC channel2 12-bit left  aligned data holding register, Address offset: 0x18
    * uint32_t DHR8R2;    DAC channel2  8-bit right aligned data holding register, Address offset: 0x1C
    */
  hdac->channel_dhr_address[(uint32_t)channel]  = (volatile uint32_t *)(&(p_instance->DHR12R1));
  hdac->channel_dhr_address[(uint32_t)channel] += (uint32_t)(3UL * (uint32_t)channel);
  hdac->channel_dhr_address[(uint32_t)channel] += (uint32_t)(alignment);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief  Set the data width and alignment for the DAC channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  alignment  The alignment and data width:
  *                    12 bits right alignment, 12 bits left alignment, 8 bits right alignment.
  */
static void DAC_SetDualChannelAlignment(hal_dac_handle_t *hdac, hal_dac_data_alignment_t alignment)
{
  DAC_TypeDef *p_instance = DAC_GET_INSTANCE(hdac);

  /**
    * Computes and stores the channel data hold register address for dual channel and alignment
    * Stored in [HAL_DAC_CHANNEL_1] as separate channel is not running when in dual channel
    * One among those register addresses:
    * uint32_t DHR12RD;   Dual DAC     12-bit right aligned data holding register, Address offset: 0x20
    * uint32_t DHR12LD;   Dual DAC     12-bit left  aligned data holding register, Address offset: 0x24
    * uint32_t DHR8RD;    Dual DAC      8-bit right aligned data holding register, Address offset: 0x28
    */
  hdac->channel_dhr_address[(uint32_t)HAL_DAC_CHANNEL_1]  = (volatile uint32_t *)(&(p_instance->DHR12RD));
  hdac->channel_dhr_address[(uint32_t)HAL_DAC_CHANNEL_1] += (uint32_t)(alignment);
}
#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
#if defined(USE_HAL_DAC_DMA) && (USE_HAL_DAC_DMA == 1)
/**
  * @brief  Set the link between DAC channel and a DMA handler.
  * @param  hdac Pointer to hal_dac_handle_t structure.
  * @param  hdma Pointer to the DMA handler to be linked with the DAC channel.
  * @param  channel The selected DAC channel.
  */
static void DAC_SetChannelDMA(hal_dac_handle_t *hdac, hal_dma_handle_t *hdma, hal_dac_channel_t channel)
{
  hdac->dma_ch[(uint32_t)channel] = hdma;
  hdma->p_parent = hdac;
}

/**
  * @brief  DMA conversion complete callback for the DAC channel.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *                the configuration information for the specified DMA module.
  */
static void DAC_DMA_ChConvCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);
  hal_dac_channel_t channel = HAL_DAC_CHANNEL_1;

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  if (hdac->dma_ch[HAL_DAC_CHANNEL_2] == hdma)
  {
    channel = HAL_DAC_CHANNEL_2;
  }
#endif /* DAC_NB_OF_CHANNEL */

  /* Call conversion complete callback */
#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_conv_cplt_cb(hdac, channel);
#else
  HAL_DAC_ConvCpltCallback(hdac, channel);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA half transfer complete callback for the DAC channel.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void DAC_DMA_ChHalfConvCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);
  hal_dac_channel_t channel = HAL_DAC_CHANNEL_1;

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  if (hdac->dma_ch[HAL_DAC_CHANNEL_2] == hdma)
  {
    channel = HAL_DAC_CHANNEL_2;
  }
#endif /* DAC_NB_OF_CHANNEL */

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_conv_half_cplt_cb(hdac, channel);
#else
  HAL_DAC_ConvHalfCpltCallback(hdac, channel);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA stop callback, when initiated by user by a call to HAL_DAC_StopChannel_DMA API
  *         (This callback is executed at end of DMA stop procedure following user stop request,
  *         and leads to user HAL_DAC_DMA_StopCallback() callback execution).
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void DAC_DMA_ChStopCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);
  hal_dac_channel_t channel = HAL_DAC_CHANNEL_1;

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  if (hdac->dma_ch[HAL_DAC_CHANNEL_2] == hdma)
  {
    channel = HAL_DAC_CHANNEL_2;
  }
#endif /* DAC_NB_OF_CHANNEL */

  hdac->global_state = HAL_DAC_STATE_SEPARATE_CHANNEL_CONFIGURED;
  hdac->channel_state[channel] = HAL_DAC_CHANNEL_STATE_IDLE;

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_stop_cplt_cb(hdac, channel);
#else
  HAL_DAC_StopCpltCallback(hdac, channel);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA error callback for the DAC channel.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void DAC_DMA_ChError(hal_dma_handle_t *const hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  if (hdac->dma_ch[HAL_DAC_CHANNEL_1] == hdma)
  {
    /* The hdma is used with DAC channel 1 */
    hdac->last_error_codes[HAL_DAC_CHANNEL_1] |= (uint16_t)(HAL_DAC_ERROR_DMA_CH1);
  }
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  else
  {
    /* The hdma is used with DAC channel 2 */
    hdac->last_error_codes[HAL_DAC_CHANNEL_2] |= (uint16_t)(HAL_DAC_ERROR_DMA_CH2);
  }
#endif /* DAC_NB_OF_CHANNEL */
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_error_cb(hdac);
#else
  HAL_DAC_ErrorCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  Enable DAC and start conversion of channel.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  * @param  p_data The source buffer address.
  * @param  size_byte The number of bytes of data to be transferred from memory to DAC peripheral
  * @param  dma_opt_interrupt The DMA optional interrupt flag.
  * @retval HAL_OK DAC instance has been correctly configured with the DMA.
  * @retval HAL_ERROR An internal inconsistency error or a parameter is invalid.
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API.
  */
static hal_status_t DAC_StartChannel_DMA_Opt(hal_dac_handle_t *hdac, hal_dac_channel_t channel, const void *p_data,
                                             uint32_t size_byte, uint32_t dma_opt_interrupt)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  static const uint16_t lut_ch_err_dma[DAC_NB_OF_CHANNEL] = {HAL_DAC_ERROR_DMA_CH1,
                                                             HAL_DAC_ERROR_DMA_CH2
                                                            };
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
#else
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  static const uint16_t lut_ch_err_dma[DAC_NB_OF_CHANNEL] = {HAL_DAC_ERROR_DMA_CH1};
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
#endif /* DAC_NB_OF_CHANNEL */
  hal_status_t status;
  DAC_TypeDef *p_instance;
  hal_dma_handle_t *p_hdma;

  p_instance = DAC_GET_INSTANCE(hdac);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  hdac->last_error_codes[channel] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

  p_hdma = hdac->dma_ch[channel];

  p_hdma->p_xfer_cplt_cb     = DAC_DMA_ChConvCplt;
  p_hdma->p_xfer_halfcplt_cb = DAC_DMA_ChHalfConvCplt;
  p_hdma->p_xfer_error_cb    = DAC_DMA_ChError;

  LL_DAC_EnableDMAReq(p_instance, lut_ch[channel]);

  LL_DAC_EnableIT_DMAUDR(p_instance, lut_ch_dma_underrun_it[channel]);

  /* Enable the DMA channel */
  status = HAL_DMA_StartPeriphXfer_IT_Opt(p_hdma, (uint32_t)p_data,
                                          (uint32_t)(hdac->channel_dhr_address[(uint32_t)channel]),
                                          size_byte, dma_opt_interrupt);

  if (status == HAL_OK)
  {
    LL_DAC_Enable(p_instance, lut_ch[channel]);

    /* Ensure minimum wait before using peripheral after enabling it */
    DAC_WaitMicroSecond(DAC_DELAY_STARTUP_US);
  }
  else
  {
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
    hdac->last_error_codes[(uint32_t)channel] |= (uint16_t)lut_ch_err_dma[channel];
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
  }

  return status;
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#if defined(USE_HAL_DAC_DUAL_CHANNEL) && (USE_HAL_DAC_DUAL_CHANNEL == 1)
/**
  * @brief  DMA conversion complete callback when in dual channel mode.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void DAC_DMA_DualChannelConvCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_dual_channel_conv_cplt_cb(hdac);
#else
  HAL_DAC_DualChannelConvCpltCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA half transfer complete callback when in dual channel mode.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void DAC_DMA_DualChannelHalfConvCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_dual_channel_conv_half_cplt_cb(hdac);
#else
  HAL_DAC_DualChannelConvHalfCpltCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA dual channel stop callback, when initiated by user by a call to HAL_DAC_DualChannelStop_DMA()
  *         (This callback is executed at end of DMA stop procedure following user stop request,
  *         and leads to user HAL_DAC_DualChannelStopCpltCallback() callback execution).
  * @param  hdma Pointer to a hal_dma_handle_t structure which contains a DMA instance.
  */
static void DAC_DMA_DualChannelStopCplt(hal_dma_handle_t *hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);

  hdac->global_state = HAL_DAC_STATE_DUAL_CHANNEL_CONFIGURED;
  hdac->channel_state[HAL_DAC_CHANNEL_1] = HAL_DAC_CHANNEL_STATE_IDLE;
  hdac->channel_state[HAL_DAC_CHANNEL_2] = HAL_DAC_CHANNEL_STATE_IDLE;

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_dual_channel_stop_cplt_cb(hdac);
#else
  HAL_DAC_DualChannelStopCpltCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA error callback when in dual channel mode.
  * @param  hdma Pointer to a hal_dma_handle_t structure that contains
  *              the configuration information for the specified DMA module.
  */
static void DAC_DMA_DualChannelError(hal_dma_handle_t *const hdma)
{
  hal_dac_handle_t *hdac = DAC_GET_DMA_PARENT(hdma);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  hdac->last_error_codes[HAL_DAC_CHANNEL_1] |= (uint16_t)(HAL_DAC_ERROR_DMA_CH1);
  hdac->last_error_codes[HAL_DAC_CHANNEL_2] |= (uint16_t)(HAL_DAC_ERROR_DMA_CH2);
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->p_error_cb(hdac);
#else
  HAL_DAC_ErrorCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

}

/**
  * @brief  Enable DAC, and start conversion with a DMA, of both channels of the same DAC.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  p_data The destination peripheral Buffer address.
  * @param  size_byte The number of data to be transferred from memory to DAC peripheral.
  * @param  dma_opt_interrupt The DMA optional interrupt flag.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM Invalid parameter.
  * @retval HAL_BUSY          DMA channel state is active when calling this API
  */
hal_status_t DAC_StartDualChannel_DMA_Opt(hal_dac_handle_t *hdac, const void *p_data, uint32_t size_byte,
                                          uint32_t dma_opt_interrupt)
{
  DAC_TypeDef *p_instance;
  hal_status_t status;
  hal_dma_handle_t *p_hdma;

  p_instance = DAC_GET_INSTANCE(hdac);

#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  hdac->last_error_codes[(uint32_t)HAL_DAC_CHANNEL_1] = (uint16_t)HAL_DAC_ERROR_NONE;
  hdac->last_error_codes[(uint32_t)HAL_DAC_CHANNEL_2] = (uint16_t)HAL_DAC_ERROR_NONE;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

  p_hdma = hdac->dma_ch[hdac->dual_channel_dma_requester];

  p_hdma->p_xfer_cplt_cb     = DAC_DMA_DualChannelConvCplt;
  p_hdma->p_xfer_halfcplt_cb = DAC_DMA_DualChannelHalfConvCplt;
  p_hdma->p_xfer_error_cb    = DAC_DMA_DualChannelError;

  LL_DAC_EnableDMAReq(p_instance, lut_ch[hdac->dual_channel_dma_requester]);
  LL_DAC_EnableIT_DMAUDR(p_instance, lut_ch_dma_underrun_it[hdac->dual_channel_dma_requester]);
  /* Enable the DMA channel, data holding register same that on HAL_DAC_CHANNEL_1*/
  status = HAL_DMA_StartPeriphXfer_IT_Opt(p_hdma, (uint32_t)p_data,
                                          (uint32_t)(hdac->channel_dhr_address[(uint32_t)HAL_DAC_CHANNEL_1]),
                                          size_byte, dma_opt_interrupt);

  if (status == HAL_OK)
  {
    LL_DAC_DualChannelEnable(p_instance);

    /* Ensure minimum wait before using peripheral after enabling it */
    DAC_WaitMicroSecond(DAC_DELAY_STARTUP_US);
  }
  else
  {
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
    hdac->last_error_codes[(uint32_t)HAL_DAC_CHANNEL_1] |= (uint16_t)HAL_DAC_ERROR_DMA_CH1;
    hdac->last_error_codes[(uint32_t)HAL_DAC_CHANNEL_2] |= (uint16_t)HAL_DAC_ERROR_DMA_CH2;
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
  }

  return status;
}

#endif /* USE_HAL_DAC_DUAL_CHANNEL */

#endif /* DAC_NB_OF_CHANNEL */
#endif /* USE_HAL_DAC_DMA */
/**
  * @brief  Handle DAC interrupt request management by channel.
  *         This function is called when an interruption for DMA underrun error occurs.
  * @param  hdac Pointer to a hal_dac_handle_t structure.
  * @param  channel The selected DAC channel.
  */
__STATIC_INLINE void DAC_IRQHandlerCH(hal_dac_handle_t *const hdac, hal_dac_channel_t channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  static const uint32_t lut_ch_dma_underrun_flag[DAC_NB_OF_CHANNEL] = {LL_DAC_FLAG_DMAUDR1,
                                                                       LL_DAC_FLAG_DMAUDR2
                                                                      };
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  static const uint16_t lut_ch_dma_underrun_error_code[DAC_NB_OF_CHANNEL] = {HAL_DAC_ERROR_DMA_UNDERRUN_CH1,
                                                                             HAL_DAC_ERROR_DMA_UNDERRUN_CH2
                                                                            };
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
#else
  static const uint32_t lut_ch_dma_underrun_flag[DAC_NB_OF_CHANNEL] = {LL_DAC_FLAG_DMAUDR1};
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
  static const uint16_t lut_ch_dma_underrun_error_code[DAC_NB_OF_CHANNEL] = {HAL_DAC_ERROR_DMA_UNDERRUN_CH1};
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */
#endif /* DAC_NB_OF_CHANNEL */
  DAC_TypeDef *p_instance;
  p_instance = DAC_GET_INSTANCE(hdac);

  if (LL_DAC_IsEnabledIT_DMAUDR(p_instance, lut_ch_dma_underrun_it[channel]) != 0UL)
  {
    if (LL_DAC_IsActiveFlag_DMAUDR(p_instance, lut_ch_dma_underrun_flag[channel]) != 0UL)
    {
      LL_DAC_ClearFlag_DMAUDR(p_instance, lut_ch_dma_underrun_flag[channel]);

      LL_DAC_DisableDMAReq(p_instance, lut_ch[channel]);

      /* Set DAC error code to channel1 DMA underrun error */
#if defined(USE_HAL_DAC_GET_LAST_ERRORS) && (USE_HAL_DAC_GET_LAST_ERRORS == 1)
      hdac->last_error_codes[channel] |= (uint16_t)lut_ch_dma_underrun_error_code[channel];
#endif /* USE_HAL_DAC_GET_LAST_ERRORS */

#if defined(USE_HAL_DAC_REGISTER_CALLBACKS) && (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
      hdac->p_error_cb(hdac);
#else
      HAL_DAC_ErrorCallback(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_DAC_MODULE */
#endif /* DAC1 || DAC2 */
/**
  * @}
  */
