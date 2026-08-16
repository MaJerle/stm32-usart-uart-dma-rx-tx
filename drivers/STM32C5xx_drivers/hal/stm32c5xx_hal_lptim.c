/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_lptim.c
  * @brief   LPTIM HAL module driver.
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
  *  @{
  */
#if defined (LPTIM1)
#if defined(USE_HAL_LPTIM_MODULE) && (USE_HAL_LPTIM_MODULE == 1)

/** @addtogroup LPTIM
  *  @{
  */
/** @defgroup LPTIM_Introduction LPTIM Introduction
  * @{
  - The LPTIM hardware abstraction layer (HAL) provides a set of APIs to interface with STM32 low-power timers.

  - STM32 low-power timers (LPTIM) provide ultra-low-power time base generation, periodic events, pulse counting,
    and low-frequency PWM while the device operates in low-power modes. They enable energy-efficient wake-up scheduling,
    low-speed signal measurement, and autonomous timing functions in battery-powered and power-constrained embedded
    systems.

  - The LPTIM HAL driver simplifies the configuration, initialization, and management of low-power timer operations by
    supporting polling, interrupt, and, where available, DMA modes, enabling flexible and efficient low-power timing
    control.

  - Additionally, it supports multiple LPTIM instances and features such as encoder mode, one-pulse mode, and external
    clocking, depending on the STM32 device, ensuring portability and consistency of low-power timing capabilities
    across different STM32 series.
  */
/**
  * @}
  */

/** @defgroup LPTIM_How_To_Use LPTIM How To Use
  * @{

  Use the LPTIM HAL driver as follows:
  ===============

  # (Non-optional call) Call HAL_LPTIM_Init() to initialize the LPTIM driver by establishing a link with
    the LPTIM physical instance.

  # (Non-optional call) Call HAL_LPTIM_SetConfig() to configure the timebase unit:
      <ul>
        <li> Select the clock source @ref hal_lptim_clk_src_t:
        <ul>
          <li> @ref HAL_LPTIM_CLK_INTERNAL : \n
            LPTIM is clocked by an internal clock source.
            The counter is incremented following each internal clock pulse.
          <li> @ref HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS : \n
            The LPTIM counter clock signal is generated from the external Input1 signal. \n
            The LPTIM external Input1 is sampled with the internal clock provided to the LPTIM. \n
            Configure LPTIM Input1 with @ref HAL_LPTIM_SetConfigInput1(). \n
          <li> @ref HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS : \n
            The LPTIM counter clock signal is generated from the external Input1 signal. \n
            Configure the LPTIM external Input1 by calling HAL_LPTIM_SetConfigInput1(). \n
          <li> @ref HAL_LPTIM_CLK_ENCODER_SUBMODE_1 to @ref HAL_LPTIM_CLK_ENCODER_SUBMODE_3 : \n
            The LPTIM counter clock signal is generated from the two external input signals, Input1 and Input2.
        </ul>
        <li> Select the <b>prescaler</b> division factor @ref hal_lptim_clk_src_presc_t
        <li> Set <b>period</b> value: number from 0 to 65535 (0x0000 to 0xFFFF)
        <li> Set <b>repetition counter</b> value: number from 0 to 255 (0x00 to 0xFF)
        <li> Select the functioning mode with @ref hal_lptim_mode_t:
        <ul>
          <li> @ref HAL_LPTIM_ONE_SHOT :
                When the LPTIM counter is stopped, a trigger event starts it.
                The counter is stopped on an update event.
          <li> @ref HAL_LPTIM_SET_ONCE :
                A first trigger event starts the LPTIM counter for a single one-shot cycle.
          <li> @ref HAL_LPTIM_CONTINUOUS :
                The LPTIM counter starts from a trigger event and never stops until the timer is disabled.
          <li> @ref HAL_LPTIM_TIMEOUT :
                The detection of an active edge on one selected trigger input can be used to reset the LPTIM counter.
        </ul>
      </ul>

  # If needed, call HAL_LPTIM_DeInit() to reset the driver.

  # When an external clock is used, configure LPTIM Input1 by calling HAL_LPTIM_SetConfigInput1().
    <ul>
      <li> Select the Input1 source with @ref hal_lptim_input1_src_t
      <li> Select the Input1 polarity with @ref hal_lptim_input1_polarity_t
      <li> Select the Input1 filter with @ref hal_lptim_filter_t
    </ul>

# Select the usage:
  - To use an LPTIM instance as a simple counter:
  --------------------------------------------
    - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_CONTINUOUS
      when calling HAL_LPTIM_SetConfig().
    - Start the LPTIM counter. Two execution modes are available:
      - Polling: HAL_LPTIM_Start()
      - Interrupt: HAL_LPTIM_Start_IT(), in this case, the update interrupt is enabled
    - Stop the LPTIM counter. Call HAL_LPTIM_Stop() and HAL_LPTIM_Stop_IT(), as per the selected execution mode.
    @note If needed, some configuration parameters can be changed on the fly (e.g., period @ref HAL_LPTIM_SetPeriod()).

  - To generate a PWM signal:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_CONTINUOUS
    when calling HAL_LPTIM_SetConfig().
  - Configure the output channel(s) by calling HAL_LPTIM_OC_SetConfigChannel().
  - To start PWM signal generation, first start the output channel, then the LPTIM time base unit
    by calling HAL_LPTIM_Start().
  Three execution modes are available:
    - Polling: HAL_LPTIM_OC_StartChannel()
    - Interrupt: HAL_LPTIM_OC_StartChannel_IT(), in this case, the compare match interrupt is enabled
  - Stop PWM signal generation. Call HAL_LPTIM_OC_StopChannel() or HAL_LPTIM_OC_StopChannel_IT(),
    as per the selected execution mode.
  @note If needed, some configuration parameters can be changed on the fly
    (e.g., PWM duty cycle @ref HAL_LPTIM_OC_SetChannelPulse()).

  - To generate a one-pulse signal:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_ONE_SHOT
    when calling HAL_LPTIM_SetConfig().
  - Configure the output channel(s) by calling HAL_LPTIM_OC_SetConfigChannel().
  - If the pulse generation is triggered when an active edge is detected on the external trigger input,
    configure the external trigger input by calling HAL_LPTIM_SetConfigExtTrigInput().
  - To start pulse generation, first start the output channel(s), then the LPTIM time base unit
    by calling HAL_LPTIM_Start().
  Three execution modes are available:
    - Polling: HAL_LPTIM_OC_StartChannel()
    - Interrupt: HAL_LPTIM_OC_StartChannel_IT(), in this case, the compare match interrupt is enabled
  - Stop signal generation. Call HAL_LPTIM_OC_StopChannel() or HAL_LPTIM_OC_StopChannel_IT(),
     as per the selected execution mode.
  @note In case of software start (external trigger not configured),
    call HAL_LPTIM_Start() to start the LPTIM counter for one-shot counting. \n
    In one-pulse mode, the output waveform is similar to the PWM mode for the first pulse.
      Then the output is permanently reset.

  - To generate a set once signal:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_SET_ONCE
    when calling HAL_LPTIM_SetConfig().
  - If the signal generation is triggered when an active edge is detected on the external trigger input,
      configure the external trigger input by calling HAL_LPTIM_SetConfigExtTrigInput().
  - Configure the output channel(s) by calling HAL_LPTIM_OC_SetConfigChannel().
  - To start signal generation, first start the channel(s), then the LPTIM time base unit by calling HAL_LPTIM_Start().
  Two execution modes are available:
    - Polling: HAL_LPTIM_OC_StartChannel()
    - Interrupt: HAL_LPTIM_OC_StartChannel_IT(), in this case, the compare match interrupt is enabled
  - Stop signal generation. Call HAL_LPTIM_OC_StopChannel() or HAL_LPTIM_OC_StopChannel_IT(),
    as per the selected execution mode.
    @note If needed, some configuration parameters can be changed on the fly
      (e.g., PWM duty cycle @ref HAL_LPTIM_OC_SetChannelPulse()).

  - To capture an input signal:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_CONTINUOUS
    when calling HAL_LPTIM_SetConfig().
  - Configure the input channel(s) by calling HAL_LPTIM_IC_SetConfigChannel().
  - To start a capture, first start the input channel, then the LPTIM time base unit (HAL_LPTIM_Start()).
  Three execution modes are available:
    - Polling: HAL_LPTIM_IC_StartChannel()
    - Interrupt: HAL_LPTIM_IC_StartChannel_IT(), in this case, the capture interrupt is enabled
    - DMA: HAL_LPTIM_IC_StartChannel_DMA(), in this case, the capture DMA request is enabled
  - Stop input signal capture. Call HAL_LPTIM_IC_StopChannel() or HAL_LPTIM_IC_StopChannel_IT()
    or HAL_LPTIM_IC_StopChannel_DMA(), as per the selected execution mode.

  - To use the LPTIM timeout feature:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_TIMEOUT
    when calling HAL_LPTIM_SetConfig().
  - Configure the external trigger input by calling HAL_LPTIM_SetConfigExtTrigInput().
  - Configure the timeout value by calling HAL_LPTIM_OC_SetPulse().
  Two execution modes are available:
    - Polling: HAL_LPTIM_Start()
    - Interrupt: HAL_LPTIM_Start_IT(), in this case, the update event is enabled
  - To stop pulse generation, call HAL_LPTIM_Stop() and HAL_LPTIM_Stop_IT(),
    as per the selected execution mode.
    @note If needed, some configuration parameters can be changed on the fly
      (e.g., PWM duty cycle @ref HAL_LPTIM_OC_SetChannelPulse()).

  - To use the LPTIM encoder interface:
  --------------------------------------------
  - Configure the LPTIM time base unit. The functioning mode must be set to @ref HAL_LPTIM_CONTINUOUS \n
  and encoder mode must be selected as the LPTIM clock source when calling HAL_LPTIM_SetConfig().
  - Configure the encoder interface (LPTIM Input1 and Input2) by calling HAL_LPTIM_SetConfigEncoder().
  Two execution modes are available:
    - Polling: HAL_LPTIM_Start()
    - Interrupt: HAL_LPTIM_Start_IT(), in this case, the update event is enabled
  - To stop the encoder interface, call HAL_LPTIM_Stop() and HAL_LPTIM_Stop_IT(), as per the selected execution mode.
    @note If needed, some configuration parameters can be changed on the fly (e.g., period @ref HAL_LPTIM_SetPeriod()).

  # Callbacks definition in interrupt or DMA mode
      - When the compilation define <b>USE_HAL_LPTIM_REGISTER_CALLBACKS</b> is set to 1,
      the user can configure the driver callbacks dynamically using their own method:

  Callback name                | Default callback                       | Register callback
  -----------------------------|----------------------------------------|----------------------------------------
  ErrorCallback                |  HAL_LPTIM_ErrorCallback               | HAL_LPTIM_RegisterErrorCallback
  StopCallback                 |  HAL_LPTIM_StopCallback                | HAL_LPTIM_RegisterStopCallback
  InputCaptureStopCallback     |  HAL_LPTIM_InputCaptureStopCallback    | HAL_LPTIM_RegisterChannelStopCallback
  UpdateCallback               |  HAL_LPTIM_UpdateCallback              | HAL_LPTIM_RegisterUpdateCallback
  UpdateHalfCpltCallback       |  HAL_LPTIM_UpdateHalfCpltCallback      | HAL_LPTIM_RegisterUpdateHalfCpltCallback
  RepUpdateCallback            |  HAL_LPTIM_RepUpdateCallback           | HAL_LPTIM_RegisterRepUpdateCallback
  TriggerCallback              |  HAL_LPTIM_TriggerCallback             | HAL_LPTIM_RegisterTriggerCallback
  InputCaptureCallback         |  HAL_LPTIM_InputCaptureCallback        | HAL_LPTIM_RegisterInputCaptureCallback
  InputCaptureHalfCpltCallback | HAL_LPTIM_InputCaptureHalfCpltCallback | HAL_LPTIM_RegisterInputCaptureHalfCpltCallback
  InputOverCaptureCallback     |  HAL_LPTIM_InputOverCaptureCallback    | HAL_LPTIM_RegisterOverCaptureCallback
  CompareMatchCallback         |  HAL_LPTIM_CompareMatchCallback        | HAL_LPTIM_RegisterCompareMatchCallback
  CompareUpdateCallback        |  HAL_LPTIM_CompareUpdateCallback       | HAL_LPTIM_RegisterCompareUpdateCallback
  AutoReloadMatchCallback      |  HAL_LPTIM_AutoReloadMatchCallback     | HAL_LPTIM_RegisterAutoReloadMatchCallback
  AutoReloadUpdateCallback     |  HAL_LPTIM_AutoReloadUpdateCallback    | HAL_LPTIM_RegisterAutoReloadUpdateCallback
  DirectionDownCallback        |  HAL_LPTIM_DirectionDownCallback       | HAL_LPTIM_RegisterDirectionDownCallback
  DirectionUpCallback          |  HAL_LPTIM_DirectionUpCallback         | HAL_LPTIM_RegisterDirectionUpCallback

  To unregister a callback, register the default callback via the registration function.

  By default, after HAL_LPTIM_Init() and when the state is @ref HAL_LPTIM_STATE_INIT, all callbacks are set to the
  corresponding default weak functions.

  Callbacks can be registered in the handle global state @ref HAL_LPTIM_STATE_INIT and @ref HAL_LPTIM_STATE_IDLE.

  When the compilation define <b>USE_HAL_LPTIM_REGISTER_CALLBACKS</b> is set to 0 or is not defined,
  the callback registration feature is not available and weak callbacks are used, represented by the default value in
  the table above.
  */
/**
  * @}
  */

/** @defgroup LPTIM_Configuration_Table LPTIM Configuration Table
  * @{
  ## Configuration inside the HAL LPTIM driver:

  Config defines                   | Where        | Default value     | Note
  -------------------------------  | -------------| ------------------| -------------------------------------------
  USE_HAL_LPTIM_MODULE             | hal_conf.h   |         1         | Enable the HAL LPTIM module
  USE_HAL_LPTIM_REGISTER_CALLBACKS | hal_conf.h   |         0         | Allow the user to define custom callbacks
  USE_HAL_LPTIM_CLK_ENABLE_MODEL   | hal_conf.h   | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
  USE_HAL_LPTIM_USER_DATA          | hal_conf.h   |         0         | Add user data inside the HAL LPTIM handle
  USE_HAL_LPTIM_GET_LAST_ERRORS    | hal_conf.h   |         0         | Enable retrieval of the last process error codes
  USE_HAL_LPTIM_DMA                | hal_conf.h   |         1         | Enable DMA support inside HAL LPTIM
  USE_HAL_CHECK_PARAM              | hal_conf.h   |         0         | Enable checking of vital parameters at runtime
  USE_HAL_MUTEX                    | hal_conf.h   |         0         | Enable semaphore usage in the HAL driver
  USE_HAL_CHECK_PROCESS_STATE      | hal_conf.h   |         0         | Enable atomic access to process state checks
  USE_ASSERT_DBG_PARAM             | PreProc env  |        NONE       | Enable parameter checks for HAL
  USE_ASSERT_DBG_STATE             | PreProc env  |        NONE       | Enable state checks for HAL

  **********************************************************************************************************************

  */
/**
  * @}
  */

/** @defgroup LPTIM_Private_Types LPTIM Private Types
  *  @{
  */

/**
  * @brief Alias for the CMSIS instance type definition.
  */
typedef LPTIM_TypeDef lptim_t;

/**
  *  @}
  */

/** @defgroup LPTIM_Private_Constants  LPTIM Private Constants
  *  @{
  */
/**
  * @brief Timeout
  */
#define LPTIM_TIMEOUT 1000UL /* 1s timeout */

/**
  * @brief Internal mapping helper used to translate HAL enumerations into
  *        low-level (LL) register selector values for external trigger and
  *        input capture sources.
  */
typedef struct
{
  /** LL selector value */
  uint32_t ll_value;

  /** HAL LPTIM instance this entry applies to. */
  hal_lptim_t instance;
  union
  {
    /** HAL external trigger source associated to ll_value. */
    hal_lptim_ext_trig_src_t  exttrig_src;

    /** HAL input capture channel source associated to ll_value. */
    hal_lptim_ic_src_t        channel_src;
  };
} lptim_mapping_t;

/**
  * @brief Mapping table translating HAL external trigger sources to LL TRIGSEL selector values.
  */
static const lptim_mapping_t extrig_mapping[] =
{
  {LL_LPTIM_TRIG_SOURCE_GPIO,          HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_GPIO}},
  {LL_LPTIM_TRIG_SOURCE_RTC_ALRA_TRG,  HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_RTC_ALRA_TRG}},
  {LL_LPTIM_TRIG_SOURCE_RTC_ALRB_TRG,  HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_RTC_ALRB_TRG}},
  {LL_LPTIM_TRIG_SOURCE_TAMP_TRG1,     HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_TAMP_TRG1}},
  {LL_LPTIM_TRIG_SOURCE_TAMP_TRG2,     HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_TAMP_TRG2}},
  {LL_LPTIM_TRIG_SOURCE_LPDMA_CH1_TC,  HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_LPDMA_CH1_TC}},
  {LL_LPTIM_TRIG_SOURCE_COMP1_OUT,     HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_COMP1_OUT}},
  {LL_LPTIM_TRIG_SOURCE_EVENTOUT,      HAL_LPTIM1, {.exttrig_src = HAL_LPTIM_EXT_TRIG_EVENTOUT}},

  /* End of map */
  {.ll_value = 0xFFFFFFFFU}
};


/**
  * @brief IC1 mapping: LPTIMx input capture 1 connections.
  */
static const lptim_mapping_t ic1_mapping[] =
{
  {LL_LPTIM_LPTIM1_IC1_RMP_GPIO,         HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_GPIO}},
  {LL_LPTIM_LPTIM1_IC1_RMP_COMP1_OUT,    HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_COMP1_OUT}},
  {LL_LPTIM_LPTIM1_IC1_RMP_EVENTOUT,     HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_EVENTOUT}},
  {LL_LPTIM_LPTIM1_IC1_RMP_MCO1,         HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_MCO1}},

  /* End of map */
  {.ll_value = 0xFFFFFFFFU}
};

/**
  * @brief IC2 mapping: LPTIMx input capture 2 connections.
  */
static const lptim_mapping_t ic2_mapping[] =
{
  {LL_LPTIM_LPTIM1_IC2_RMP_GPIO,         HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_GPIO}},
  {LL_LPTIM_LPTIM1_IC2_RMP_LSI,          HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_LSI}},
  {LL_LPTIM_LPTIM1_IC2_RMP_LSE,          HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_LSE}},
  {LL_LPTIM_LPTIM1_IC2_RMP_RCC_HSE_1MHZ, HAL_LPTIM1, {.channel_src = HAL_LPTIM_INPUT_RCC_HSE_1MHZ}},

  /* End of map */
  {.ll_value = 0xFFFFFFFFU}
};


/**
  * @brief Helper structure grouping compare-match and capture IT mask for one channel.
  */
typedef struct
{
  /** DIER bit enabling compare register OK interrupt for the channel. */
  uint32_t cmp;

  /** DIER bit enabling capture/compare interrupt for the channel. */
  uint32_t cc;
} lptim_channel_it_t;

/**
  * @brief Per-channel interrupt bit mapping for compare-match and capture events.
  */

static const lptim_channel_it_t channel_it[] =
{
  {LL_LPTIM_DIER_CMP1OKIE, LPTIM_DIER_CC1IE},
  {LL_LPTIM_DIER_CMP2OKIE, LPTIM_DIER_CC2IE},
};

/**
  * @brief Define the channel idle state, whether it is an OC or an IC channel.
  */
#define LPTIM_CHANNEL_STATE_IDLE \
  (HAL_LPTIM_OC_CHANNEL_STATE_IDLE | HAL_LPTIM_IC_CHANNEL_STATE_IDLE)

/**
  * @brief Mask for the polarity bits of the clock.
  */
#define LPTIM_CLOCK_POLARITY_MASK \
  (LL_LPTIM_CLK_POLARITY_FALLING | LL_LPTIM_CLK_POLARITY_RISING_FALLING)

/**
  * @brief Mask for the source bits in CFGR of the external trigger.
  */
#define LPTIM_ETR_SRC_MASK LPTIM_CFGR_TRIGSEL_Msk

/**
  * @brief Mask for the polarity bits in CFGR of the external trigger.
  */
#define LPTIM_ETR_POLARITY_MASK LL_LPTIM_TRIG_POLARITY_RISING_FALLING

/**
  * @brief Mask for all interrupt bits in DIER.
  */
#define LPTIM_DIER_INTERRUPTS_MASK (LL_LPTIM_DIER_ARROKIE | LL_LPTIM_DIER_ARRMIE | \
                                    LL_LPTIM_DIER_REPOKIE | LL_LPTIM_DIER_UEIE | \
                                    LL_LPTIM_DIER_UPIE | LL_LPTIM_DIER_DOWNIE | \
                                    LL_LPTIM_DIER_CC1IE | LPTIM_DIER_CC2IE)

/**
  * @brief Mask for the filter bits of the input trigger.
  */
#define LPTIM_ETR_FILTER_SHIFT 6U

/**
  * @brief Mask for the filter bits of the input trigger with shift.
  */
#define LPTIM_ETR_FILTER_MASK (3U << LPTIM_ETR_FILTER_SHIFT)

/**
  * @brief Mask for LPTIM_CFGR.
  */
#define LPTIM_MODE_CFGR_MASK \
  (LL_LPTIM_OC_WAVEFORM_SETONCE | LPTIM_CFGR_TIMOUT)

/**
  * @brief Mask for LPTIM_CR.
  */
#define LPTIM_MODE_CR_MASK \
  (LL_LPTIM_OPERATING_MODE_ONESHOT | LL_LPTIM_OPERATING_MODE_CONTINUOUS)

/**
  * @brief Mask to determine whether the clock source is internal, external, or encoder. \n
  *        The mask lets us select the ENC, the COUNTMODE, and the CKSEL of the CFGR register.
  */
#define LPTIM_CLOCK_TYPE_MASK       \
  (LL_LPTIM_CLK_SOURCE_EXTERNAL   | \
   LL_LPTIM_COUNTER_MODE_EXTERNAL | \
   LL_LPTIM_ENCODER_MODE_ENABLE)


/**
  *  @}
  */

/** @defgroup  LPTIM_Private_Macros LPTIM Private Macros
  *  @{
  */

/**
  * @brief  Check the validity of an output channel parameter.
  * @param  instance  LPTIM instance.
  * @param  channel   The channel to check (@ref hal_lptim_channel_t).
  * @retval SET (compare channel is valid) or RESET (compare channel is invalid).
  */
#define IS_LPTIM_OC_CHANNEL(instance, channel) \
  ((((channel) == HAL_LPTIM_CHANNEL_1) && IS_LPTIM_CC1_INSTANCE((instance)))      \
   || (((channel) == HAL_LPTIM_CHANNEL_2) && IS_LPTIM_CC2_INSTANCE((instance))))

/**
  * @brief  Check if the timeout period is expired.
  * @param  delta_ticks  Delta ticks to compare with the timeout period
  * @retval SET (timeout period is expired) or RESET (otherwise).
  */
#define LPTIM_TIMEOUT_PERIOD_EXPIRED(delta_ticks) \
  ((delta_ticks) > (uint32_t)LPTIM_TIMEOUT)

/**
  * @brief  Get the LPTIM instance from the handle.
  * @param  hlptim  LPTIM handle.
  * @retval LPTIM instance.
  */
#define LPTIM_INSTANCE(hlptim) ((lptim_t *)((uint32_t)((hlptim)->instance)))

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
/**
  * @brief  Macro that returns the global state depending on the DMA silent mode.
  * @param  interrupts  DMA interrupts.
  * @retval HAL_LPTIM_STATE_ACTIVE_SILENT (DMA silent mode is active) or
  *         HAL_LPTIM_STATE_ACTIVE (DMA silent mode is not active).
  */
#define LPTIM_STATE_ACTIVE(interrupts) \
  HAL_LPTIM_STATE_ACTIVE |             \
  ((((interrupts) & HAL_DMA_OPT_IT_SILENT) == HAL_DMA_OPT_IT_SILENT) ? LPTIM_ACTIVE_SILENT : LPTIM_ACTIVE_NOT_SILENT)

/**
  * @brief  Macro that returns the input channel state depending on the DMA silent mode.
  * @param  interrupts  DMA interrupts.
  * @retval HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE_SILENT (DMA silent mode is active) or
  *         HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE (DMA silent mode is not active).
  */
#define LPTIM_IC_CHANNEL_STATE_ACTIVE(interrupts) \
  ((STM32_IS_BIT_SET((interrupts), (uint32_t)HAL_DMA_OPT_IT_SILENT)) ? \
   HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE_SILENT : HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE)

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief  Macro for the control of DMA silent mode validity.
  * @param  handle     LPTIM handle.
  * @param  channel    DMA channel.
  * @param  interrupts DMA interrupts.
  * @retval SET (DMA silent mode is valid) or RESET (DMA silent mode is invalid).
  */
#define IS_LPTIM_DMA_VALID_SILENT_MODE(handle, channel, interrupts) \
  (((interrupts) == HAL_LPTIM_OPT_DMA_IT_SILENT)                    \
   && ((handle)->hdma[(channel)]->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)

#endif /* USE_HAL_DMA_LINKEDLIST */
/**
  * @brief  Tell whether the DMA silent mode is active.
  * @param  state  The state to check.
  * @retval SET (DMA silent mode is active) or RESET (otherwise).
  */
#define IS_LPTIM_ACTIVE_SILENT(state) \
  ((uint32_t)(state) & (uint32_t)LPTIM_ACTIVE_SILENT)

#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief  Check the validity of the channel.
  * @param  channel  The channel to check (@ref hal_lptim_channel_t).
  * @retval SET (channel is valid) or RESET (channel is invalid).
  */
#define IS_LPTIM_CHANNEL(channel)     \
  (((channel) == HAL_LPTIM_CHANNEL_1) \
   || ((channel) == HAL_LPTIM_CHANNEL_2))

/**
  * @brief  Check the validity of the LPTIM1 channel sources.
  * @param  channel   The channel to check (@ref hal_lptim_channel_t).
  * @param  src       The channel source to check (@ref hal_lptim_ic_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_LPTIM1_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_LPTIM_CHANNEL_1) \
    && (((src) == HAL_LPTIM_INPUT_GPIO) \
        || ((src) == HAL_LPTIM_INPUT_COMP1_OUT) \
        || ((src) == HAL_LPTIM_INPUT_EVENTOUT) \
        || ((src) == HAL_LPTIM_INPUT_MCO1))) \
   || (((channel) == HAL_LPTIM_CHANNEL_2) \
       && (((src) == HAL_LPTIM_INPUT_GPIO) \
           || ((src) == HAL_LPTIM_INPUT_LSI) \
           || ((src) == HAL_LPTIM_INPUT_LSE) \
           || ((src) == HAL_LPTIM_INPUT_RCC_HSE_1MHZ))))

/**
  * @brief  Check the validity of the channel sources.
  * @param  instance  LPTIM instance.
  * @param  channel   The channel to check (@ref hal_lptim_channel_t).
  * @param  src       The channel source to check (@ref hal_lptim_ic_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_LPTIM_CHANNEL_SRC(instance, channel, src) \
  ((((instance) == LPTIM1) && IS_LPTIM1_CHANNEL_SRC((channel), (src))))


/**
  * @brief  Check the validity of the input1 polarity.
  * @param  polarity  The input1 polarity to check (@ref hal_lptim_input1_polarity_t).
  * @retval SET (input1 polarity is valid) or RESET (input1 polarity is invalid).
  */
#define IS_LPTIM_INPUT1_POLARITY(polarity)     \
  (((polarity) == HAL_LPTIM_INPUT1_RISING)     \
   || ((polarity) == HAL_LPTIM_INPUT1_FALLING) \
   || ((polarity) == HAL_LPTIM_INPUT1_RISING_FALLING))

/**
  * @brief  Check the validity of the input1 source.
  * @param  src  The input1 source to check (@ref hal_lptim_input1_src_t).
  * @retval SET (input1 polarity is valid) or RESET (input1 polarity is invalid).
  */
#define IS_LPTIM_INPUT1_SRC(src)   \
  (((src) == HAL_LPTIM_INPUT1_GPIO)\
   || ((src) == HAL_LPTIM_INPUT1_COMP1_OUT))

/**
  * @brief Check input2 source
  */
#if defined(COMP2)
#define IS_LPTIM_INPUT2_SRC(src)       \
  (((src) == HAL_LPTIM_INPUT2_GPIO)    \
   || ((src) == HAL_LPTIM_INPUT2_COMP2_OUT))
#else
#define IS_LPTIM_INPUT2_SRC(src)       \
  (((src) == HAL_LPTIM_INPUT2_GPIO))
#endif /* COMP2 */

/**
  * @brief  Check the validity of the clock encoder submode.
  * @param  src  The clock encoder submode to check (@ref hal_lptim_clk_src_t).
  * @retval SET (clock encoder submode is valid) or RESET (clock encoder submode is invalid).
  */
#define IS_LPTIM_CLK_ENCODER(src)                \
  (((src) == HAL_LPTIM_CLK_ENCODER_SUBMODE_1)    \
   || ((src) == HAL_LPTIM_CLK_ENCODER_SUBMODE_2) \
   || ((src) == HAL_LPTIM_CLK_ENCODER_SUBMODE_3))

/**
  * @brief  Check the validity of the clock source with respect to the instance.
  * @param  instance  LPTIM instance.
  * @param  src  The clock source to check (@ref hal_lptim_clk_src_t).
  * @retval SET (clock source is valid) or RESET (clock source is invalid).
  */
#define IS_LPTIM_CLK_SRC(instance, src)              \
  (((src) == HAL_LPTIM_CLK_INTERNAL)                 \
   || ((src) == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS)  \
   || ((src) == HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS) \
   || (IS_LPTIM_CLK_ENCODER(src)                     \
       && (IS_LPTIM_ENCODER_INTERFACE_INSTANCE(instance))))

/**
  * @brief  Check the validity of the clock source prescaler with respect to the clock source.
  * @param  clock_source  The clock source to check prescaler (@ref hal_lptim_clk_src_t).
  * @param  prescaler     The prescaler to check (@ref hal_lptim_clk_src_presc_t).
  * @note   When the clock source is either HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS
  *         or HAL_LPTIM_CLK_ENCODER_SUBMODE_x the internal clock provided to
  *         the LPTIM must not be prescaled.
  * @retval SET (clock source and prescaler is valid) or RESET (clock source and prescaler is invalid).
  */
#define IS_LPTIM_CLK_SRC_PRESC(clock_source, prescaler)   \
  (((clock_source) == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS) \
   || (IS_LPTIM_CLK_ENCODER(clock_source))                \
   ? ((prescaler) == HAL_LPTIM_CLK_SRC_DIV1)              \
   : (((prescaler) == HAL_LPTIM_CLK_SRC_DIV1)             \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV2)          \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV4)          \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV8)          \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV16)         \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV32)         \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV64)         \
      || ((prescaler) == HAL_LPTIM_CLK_SRC_DIV128)))

/**
  * @brief  Check the validity of the clock source with respect to the counter mode.
  * @param  clock_source  The clock source to check counter mode (@ref hal_lptim_clk_src_t).
  * @param  mode          The counter mode to check (@ref hal_lptim_mode_t).
  * @note   When the clock source is a clock encoder, the mode provided must be CONTINUOUS.
  * @retval SET (clock source and counter mode is valid) or RESET (clock source and counter mode is invalid).
  */
#define IS_LPTIM_MODE(clock_source, mode) \
  ((IS_LPTIM_CLK_ENCODER(clock_source)) ? ((mode) == HAL_LPTIM_CONTINUOUS) : (((mode) == HAL_LPTIM_ONE_SHOT)      \
                                                                              || ((mode) == HAL_LPTIM_SET_ONCE)   \
                                                                              || ((mode) == HAL_LPTIM_CONTINUOUS) \
                                                                              || ((mode) == HAL_LPTIM_TIMEOUT)))

/**
  * @brief  Check the value to store in the auto-reload register (ARR).
  * @param  period    Period value.
  * @retval SET (period value valid) or RESET (period value invalid).
  */
#define IS_LPTIM_PERIOD(period) \
  (((period) > 0U) && ((period) <= 0x0000FFFFU))

/**
  * @brief  Check the value to store in the repetition counter register (RCR).
  * @param  rep  Repetition counter value.
  * @retval SET (repetition counter value valid) or RESET (repetition counter value invalid).
  */
#define IS_LPTIM_REPETITION_COUNTER(rep) \
  ((rep) <= 0x000000FFU)

/**
  * @brief  Check the validity of the DMA index.
  * @param  idx  The DMA index to check (@ref hal_lptim_dma_index_t).
  * @retval SET (DMA index is valid) or RESET (DMA index is invalid).
  */
#define IS_LPTIM_DMA_INDEX(idx)       \
  (((idx) == HAL_LPTIM_DMA_ID_UPDATE) \
   || ((idx) == HAL_LPTIM_DMA_ID_CC1) \
   || ((idx) == HAL_LPTIM_DMA_ID_CC2))

/**
  * @brief Check external trigger LPTIM1.
  */
#define IS_LPTIM1_EXT_TRIG_SRC(src)              \
  (((src) == HAL_LPTIM_EXT_TRIG_GPIO)            \
   || ((src) == HAL_LPTIM_EXT_TRIG_RTC_ALRA_TRG) \
   || ((src) == HAL_LPTIM_EXT_TRIG_RTC_ALRB_TRG) \
   || ((src) == HAL_LPTIM_EXT_TRIG_TAMP_TRG1)    \
   || ((src) == HAL_LPTIM_EXT_TRIG_TAMP_TRG2)    \
   || ((src) == HAL_LPTIM_EXT_TRIG_LPDMA_CH1_TC) \
   || ((src) == HAL_LPTIM_EXT_TRIG_COMP1_OUT)    \
   || ((src) == HAL_LPTIM_EXT_TRIG_EVENTOUT))


/**
  * @brief Check external trigger.
  */
#define IS_LPTIM_EXT_TRIG_SRC(instance, src)                   \
  ((((instance) == LPTIM1) && IS_LPTIM1_EXT_TRIG_SRC((src))))

/**
  * @brief  Check the validity of the input channel prescaler.
  * @param  prescaler  The input channel prescaler to check (@ref hal_lptim_ic_prescaler_t).
  * @retval SET (input channel prescaler is valid) or RESET (input channel prescaler is invalid).
  */
#define IS_LPTIM_IC_PRESCALER(prescaler) \
  (((prescaler) == HAL_LPTIM_IC_DIV1)    \
   || ((prescaler) == HAL_LPTIM_IC_DIV2) \
   || ((prescaler) == HAL_LPTIM_IC_DIV4) \
   || ((prescaler) == HAL_LPTIM_IC_DIV8))

/**
  * @brief  Check the validity of the external trigger polarity.
  * @param  polarity  The external trigger polarity to check (@ref hal_lptim_ext_trig_polarity_t).
  * @retval SET (external trigger polarity is valid) or RESET (external trigger polarity is invalid).
  */
#define IS_LPTIM_EXT_TRIG_POLARITY(polarity) \
  (((polarity) == HAL_LPTIM_TRIG_RISING)     \
   || ((polarity) == HAL_LPTIM_TRIG_FALLING) \
   || ((polarity) == HAL_LPTIM_TRIG_RISING_FALLING))

/**
  * @brief  Check the validity of the filter.
  * @param  fdiv  The filter division to check (@ref hal_lptim_filter_t).
  * @retval SET (filter division is valid) or RESET (filter division is invalid).
  */
#define IS_LPTIM_FILTER(fdiv)        \
  (((fdiv) == HAL_LPTIM_FDIV1)       \
   || ((fdiv) == HAL_LPTIM_FDIV1_N2) \
   || ((fdiv) == HAL_LPTIM_FDIV1_N4) \
   || ((fdiv) == HAL_LPTIM_FDIV1_N8))

/**
  * @brief Check the validity of the output compare pulse.
  * @param pulse The output compare pulse.
  * @retval SET (output compare pulse is valid) or RESET (output compare pulse is invalid).
  */
#define IS_LPTIM_OC_PULSE(pulse) \
  ((pulse) <= 0xFFFFU)

/**
  * @brief  Check the validity of the output channel polarity.
  * @param  polarity  The output channel polarity to check (@ref hal_lptim_oc_polarity_t).
  * @retval SET (output channel polarity is valid) or RESET (output channel polarity is invalid).
  */
#define IS_LPTIM_OC_POLARITY(polarity) \
  (((polarity) == HAL_LPTIM_OC_HIGH)   \
   || ((polarity) == HAL_LPTIM_OC_LOW))

/**
  * @brief  Check the validity of the input channel polarity.
  * @param  polarity  The input channel polarity to check (@ref hal_lptim_ic_polarity_t).
  * @retval SET (input channel polarity is valid) or RESET (input channel polarity is invalid).
  */
#define IS_LPTIM_IC_POLARITY(polarity)     \
  (((polarity) == HAL_LPTIM_IC_RISING)     \
   || ((polarity) == HAL_LPTIM_IC_FALLING) \
   || ((polarity) == HAL_LPTIM_IC_RISING_FALLING))

/**
  * @brief  Check the validity of clock source external asynchronous.
  * @param  clk  The clock source to check (@ref hal_lptim_clk_src_t)
  * @retval SET (Clock src is external asynchronous) or RESET (Clock src is not external asynchronous).
  */
#define IS_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS(clk) \
  ((((clk) & LL_LPTIM_CLK_SOURCE_EXTERNAL) != 0U) ? 1U : 0U)

/**
  * @brief  Check the validity of clock source encoder submode.
  * @param  clk The clock source to check (@ref hal_lptim_clk_src_t)
  * @retval SET (Clock src is an encoder submode type) or RESET (Clock src is not an encoder submode type).
  */
#define IS_LPTIM_CLOCK_TYPE_ENCODER(clk) \
  ((((clk) & LL_LPTIM_ENCODER_MODE_ENABLE) != 0U) ? 1U : 0U)

/**
  * @brief Extract the clock type (internal, external, encoder) from CFGR register.
  */
/**
  * @brief  Extract the clock type (internal, external, encoder) from the CFGR register.
  * @param  cfgr The register needs to extract the clock type.
  * @retval Clock type (internal, external, encoder).
  */
#define LPTIM_GET_CLOCK_TYPE(cfgr) \
  ((uint32_t)((cfgr) & LPTIM_CLOCK_TYPE_MASK))

/**
  * @brief  Extract the clock filter from instance.CFGR register.
  * @param  instance  LPTIM instance.
  * @retval Clock filter
  */
#define LPTIM_GET_CLOCK_FILTER(instance) \
  (uint32_t)(LL_LPTIM_GetClockFilter(instance) >> LPTIM_CFGR_CKFLT_Pos)

/**
  * @brief  Transform the HAL filter into a LL filter.
  * @param  filter  The HAL filter.
  * @retval The LL filter.
  */
#define LPTIM_CFGR_HAL2LL_FILTER(filter) \
  (uint32_t)((filter) << LPTIM_CFGR_CKFLT_Pos)

/**
  * @brief  Transform the HAL filter into a LL IC filter.
  * @param  filter  The HAL filter.
  * @retval The LL IC filter.
  */
#define LPTIM_IC_HAL2LL_FILTER(filter) \
  (uint32_t)((filter) << LPTIM_CCMR1_IC1F_Pos)

/**
  * @brief  Transform the LL filter into a HAL IC filter.
  * @param  filter  The HAL filter.
  * @retval The LL IC filter.
  */
#define LPTIM_IC_LL2HAL_FILTER(filter) \
  (uint32_t)((filter) >> LPTIM_CCMR1_IC1F_Pos)

/**
  * @brief Extract the trigger source from CFGR.
  * @param  cfgr  The CFGR register.
  * @retval The LL trigger source.
  */
#define LPTIM_GET_ETR_SOURCE(cfgr) \
  ((uint32_t)(((cfgr) & LPTIM_ETR_SRC_MASK)))

/**
  * @brief Extract the ETR filter value from CFGR.
  * @param  cfgr  The CFGR register.
  * @retval The LL trigger filter.
  */
#define LPTIM_GET_ETR_FILTER(cfgr) \
  ((uint32_t)(((cfgr) & LPTIM_ETR_FILTER_MASK)))

/**
  * @brief Extract the ETR polarity from CFGR.
  * @param  cfgr  The CFGR register.
  * @retval The LL trigger polarity.
  */
#define LPTIM_GET_ETR_POLARITY(cfgr) \
  ((uint32_t)((cfgr) & LPTIM_ETR_POLARITY_MASK))

/**
  * @brief Reset the clock source prescaler from CFGR register.
  * @param  cfgr  The CFGR register.
  * @retval The CFGR with Clock source prescaler reset.
  */
#define LPTIM_RESET_CLOCK_SOURCE_PRESCALER(cfgr) \
  ((cfgr) &= ~(uint32_t)(LPTIM_CFGR_PRESC))

/**
  * @brief  Get the low-power timer handle registered in the DMA handle.
  * @param  hdma  DMA handle.
  * @retval LPTIM handle.
  */
#define LPTIM_GET_HDMA_PARENT(hdma) \
  (hal_lptim_handle_t *)((hdma)->p_parent)

/**
  *  @}
  */

/** @defgroup LPTIM_Private_Functions LPTIM Private Functions
  *  @{
  */

#if defined(USE_HAL_LPTIM_CLK_ENABLE_MODEL) && (USE_HAL_LPTIM_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
/**
  * @brief  Clock enabling for a particular instance.
  * @param  instance HAL LPTIM instance
  */
__STATIC_FORCEINLINE void LPTIM_EnableClock(hal_lptim_t instance)
{
  switch (instance)
  {
    case HAL_LPTIM1:
      HAL_RCC_LPTIM1_EnableClock();
      break;
    default:
      break;
  }
}
#endif /* USE_HAL_LPTIM_CLK_ENABLE_MODEL */

/**
  * @brief    Wait for a given flag.
  * @param    p_lptim                  CMSIS LPTIM instance
  * @param    ll_lptim_is_active_flag  Is flag active
  * @warning  That is the responsibility of the caller to clear the flag.
  * @return   return                  state of flag
  * @retval 1                       flag not activate
  * @retval 0                       flag is correctly activate
  */
static hal_status_t LPTIM_WaitFlag(const lptim_t *p_lptim,
                                   uint32_t (*ll_lptim_is_active_flag)(const lptim_t *p_lptim))
{
  uint32_t tickstart = HAL_GetTick();
  uint8_t isActiveFlag = 0;

  while (isActiveFlag == 0U)
  {
    isActiveFlag = ll_lptim_is_active_flag(p_lptim);
    if (LPTIM_TIMEOUT_PERIOD_EXPIRED(HAL_GetTick() - tickstart))
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (isActiveFlag == 0U)
      {
        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Disable LPTIM HW instance.
  * @param  p_lptim   CMSIS LPTIM instance
  * @note   The following sequence is required to solve LPTIM disable HW limitation.
  *         Please check Errata Sheet ES0335 for more details under "MCU remain
  *         stuck in LPTIM interrupt when entering Stop mode" section.
  * @retval None
  */
__STATIC_INLINE hal_status_t LPTIM_CcDisable(lptim_t *p_lptim)
{
  /* save LPTIM Config */
  lptim_t cpyInstance = *p_lptim;

  /* Enter critical section */
  uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1);

  switch ((uint32_t)p_lptim)
  {
    case HAL_LPTIM1:
      HAL_RCC_LPTIM1_Reset();
      break;
    default:
      break;
  }

  uint32_t dier_reg = LL_LPTIM_READ_REG(&cpyInstance, DIER);
  uint32_t arr_reg = LL_LPTIM_READ_REG(&cpyInstance, ARR);

  if ((dier_reg != 0U) || (arr_reg != 0U))
  {
    LL_LPTIM_Enable(p_lptim);
    LL_LPTIM_WRITE_REG(p_lptim, DIER, LL_LPTIM_READ_REG(&cpyInstance, DIER));
    if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
    {
      return HAL_ERROR;
    }
    LL_LPTIM_ClearFlag_DIEROK(p_lptim);
    LL_LPTIM_SetAutoReload(p_lptim, LL_LPTIM_READ_REG(&cpyInstance, ARR));
    if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_ARROK) != HAL_OK)
    {
      return HAL_ERROR;
    }
    LL_LPTIM_ClearFlag_ARROK(p_lptim);
    LL_LPTIM_Disable(p_lptim);
  }

  LL_LPTIM_OC_SetCompareCH1(p_lptim, LL_LPTIM_READ_REG(&cpyInstance, CCR1));
  LL_LPTIM_OC_SetCompareCH2(p_lptim, LL_LPTIM_READ_REG(&cpyInstance, CCR2));
  LL_LPTIM_SetRepetition(p_lptim, LL_LPTIM_READ_REG(&cpyInstance, RCR));
  LL_LPTIM_WRITE_REG(p_lptim, CFGR, LL_LPTIM_READ_REG(&cpyInstance, CFGR));
  LL_LPTIM_WRITE_REG(p_lptim, CFGR2, LL_LPTIM_READ_REG(&cpyInstance, CFGR2));
  LL_LPTIM_WRITE_REG(p_lptim, CCMR1, LL_LPTIM_READ_REG(&cpyInstance, CCMR1));

  /* Restore LPTIM_Config */
  __set_PRIMASK(primask_bit);
  return HAL_OK;
}

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
/**
  * @brief Callbacks initialization function.
  * @param hlptim   Pointer to the handle of the LPTIM instance.
  */
__STATIC_FORCEINLINE void LPTIM_InitCallbacks(hal_lptim_handle_t *hlptim)
{
#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
  /* LPTIM Error Callback */
  hlptim->error_callback = HAL_LPTIM_ErrorCallback;

  hlptim->stop_callback = HAL_LPTIM_StopCallback;

  hlptim->input_capture_stop_callback = HAL_LPTIM_InputCaptureStopCallback;
#endif /* USE_HAL_LPTIM_DMA */

  /* LPTIM Period Elapsed Callback */
  hlptim->update_callback = HAL_LPTIM_UpdateCallback;

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
  /* LPTIM Period Elapsed half complete Callback */
  hlptim->update_half_cplt_callback = HAL_LPTIM_UpdateHalfCpltCallback;
#endif /* USE_HAL_LPTIM_DMA */

  /* LPTIM Auto Reload Update Callback */
  hlptim->auto_reload_update_callback = HAL_LPTIM_AutoReloadUpdateCallback;

  /* LPTIM Auto Reload Match Callback */
  hlptim->auto_reload_match_callback = HAL_LPTIM_AutoReloadMatchCallback;

  /* LPTIM Repetition Update Callback */
  hlptim->rep_update_callback = HAL_LPTIM_RepUpdateCallback;

  /* LPTIM Trigger Callback */
  hlptim->trigger_callback = HAL_LPTIM_TriggerCallback;

  /* LPTIM Output Compare Delay Elapsed Callback */
  hlptim->compare_match_callback = HAL_LPTIM_CompareMatchCallback;

  /* LPTIM Output Compare Update Callback */
  hlptim->compare_update_callback = HAL_LPTIM_CompareUpdateCallback;

  /* LPTIM Input Capture Callback */
  hlptim->input_capture_callback = HAL_LPTIM_InputCaptureCallback;

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
  /* LPTIM Input Capture half complete Callback */
  hlptim->input_capture_half_cplt_callback = HAL_LPTIM_InputCaptureHalfCpltCallback;
#endif /* USE_HAL_LPTIM_DMA */

  /* LPTIM Over capture Callback */
  hlptim->input_over_capture_callback = HAL_LPTIM_InputOverCaptureCallback;

  /* LPTIM Direction UP Change Callback */
  hlptim->direction_up_callback = HAL_LPTIM_DirectionUpCallback;

  /* LPTIM Direction DOWN Change Callback */
  hlptim->direction_down_callback = HAL_LPTIM_DirectionDownCallback;
}
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

/**
  * @brief  Convert external trigger sources to an LL value.
  * @param  hlptim       Pointer to the handle of the LPTIM instance.
  * @param  exttrig_src  External trigger source to convert to an LL value.
  * @return LL value selector.
  */
__STATIC_INLINE uint32_t LPTIM_ConvertHALToLLExttrig(const hal_lptim_handle_t *hlptim,
                                                     hal_lptim_ext_trig_src_t exttrig_src)
{
  uint32_t trigsel = LL_LPTIM_TRIG_SOURCE_GPIO;
  uint32_t i = 0;

  while (extrig_mapping[i].ll_value != (uint32_t)0xFFFFU)
  {
    if ((extrig_mapping[i].exttrig_src == exttrig_src)
        && ((extrig_mapping[i].instance == (hal_lptim_t)0xFFFFU)
            || (extrig_mapping[i].instance == hlptim->instance)))
    {
      trigsel = extrig_mapping[i].ll_value;
      break;
    }
    ++i;
  }
  return trigsel;
}

/**
  * @brief  Convert an LL external trigger source to a HAL source value.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @param  trigsel  LL value used to convert to an external trigger source.
  * @return @ref hal_lptim_ext_trig_src_t External trigger source value.
  */
__STATIC_INLINE hal_lptim_ext_trig_src_t LPTIM_ConvertLLToHALExttrig(const hal_lptim_handle_t *hlptim,
                                                                     uint32_t trigsel)
{
  hal_lptim_ext_trig_src_t exttrig_src = HAL_LPTIM_EXT_TRIG_GPIO;
  uint32_t i = 0;

  while (extrig_mapping[i].ll_value != (uint32_t)0xFFFFU)
  {
    if ((extrig_mapping[i].ll_value == trigsel)
        && ((extrig_mapping[i].instance == (hal_lptim_t)0xFFFFU)
            || (extrig_mapping[i].instance == hlptim->instance)))
    {
      exttrig_src = extrig_mapping[i].exttrig_src;
      break;
    }
    ++i;
  }

  return exttrig_src;
}

/**
  * @brief  Convert a HAL input channel source to an LL input source value.
  * @param  hlptim      Pointer to the handle of the LPTIM instance.
  * @param  channel     LPTIM channel identifier.
  * @param  channel_src Input channel source to convert to an LL value.
  * @return LL value selector.
  */
__STATIC_INLINE uint32_t LPTIM_ConvertHALToLLIcx(const hal_lptim_handle_t *hlptim,
                                                 hal_lptim_channel_t channel,
                                                 hal_lptim_ic_src_t channel_src)
{
  uint32_t icxsel = 0x00000000UL; /*ICx_GPIO */
  uint32_t i = 0;
  const lptim_mapping_t *icx_mapping;

  switch (channel)
  {
    case HAL_LPTIM_CHANNEL_1:
      icx_mapping = ic1_mapping;
      break;
    case HAL_LPTIM_CHANNEL_2:
      icx_mapping = ic2_mapping;
      break;
    default:
      icx_mapping = NULL;
      break;
  }

  if (icx_mapping != NULL)
  {
    while (icx_mapping[i].ll_value != (uint32_t)0xFFFFU)
    {
      if ((icx_mapping[i].channel_src == channel_src)
          && ((icx_mapping[i].instance == (hal_lptim_t)0xFFFFU)
              || (icx_mapping[i].instance == hlptim->instance)))
      {
        icxsel = icx_mapping[i].ll_value;
        break;
      }
      ++i;
    }
  }
  return icxsel;
}

/**
  * @brief  Convert an LL input source to a HAL input channel source value.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   LPTIM channel identifier.
  * @param  icxsel    LL value used to convert to input channel sources.
  * @return @ref hal_lptim_ic_src_t.
  */
__STATIC_INLINE hal_lptim_ic_src_t LPTIM_ConvertLLToHALIcx(const hal_lptim_handle_t *hlptim,
                                                           hal_lptim_channel_t channel,
                                                           uint32_t icxsel)
{
  uint32_t i = 0;
  hal_lptim_ic_src_t channel_src = HAL_LPTIM_INPUT_GPIO;
  const lptim_mapping_t *icx_mapping;

  switch (channel)
  {
    case HAL_LPTIM_CHANNEL_1:
      icx_mapping = ic1_mapping;
      break;
    case HAL_LPTIM_CHANNEL_2:
      icx_mapping = ic2_mapping;
      break;
    default:
      icx_mapping = NULL;
      break;
  }

  if (icx_mapping != NULL)
  {
    while (icx_mapping[i].ll_value != (uint32_t)0xFFFFU)
    {
      if ((icx_mapping[i].ll_value == icxsel)
          && ((icx_mapping[i].instance == (hal_lptim_t)0xFFFFU)
              || (icx_mapping[i].instance == hlptim->instance)))
      {
        channel_src = icx_mapping[i].channel_src;
        break;
      }
      ++i;
    }
  }
  return channel_src;

}

/**
  * @brief Get the clock source of the low-power timer time-base unit.
  * @param p_lptim      CMSIS LPTIM instance.
  * @return @ref hal_lptim_clk_src_t clock source mode.
  */
__STATIC_INLINE hal_lptim_clk_src_t LPTIM_GetClockSource(const lptim_t *p_lptim)
{
  volatile uint32_t cfgr = LL_LPTIM_READ_REG(p_lptim, CFGR);
  uint32_t clk_src = (uint32_t)LPTIM_GET_CLOCK_TYPE(cfgr);

  if (IS_LPTIM_CLOCK_TYPE_ENCODER((uint32_t)clk_src) != 0U)
  {
    /* For the encoder mode the polarity gives the submode. */
    clk_src |= (cfgr & LPTIM_CLOCK_POLARITY_MASK);

    /* COUNTMODE force clean... */
    clk_src &= ~LL_LPTIM_COUNTER_MODE_EXTERNAL;

    clk_src &= (cfgr | LPTIM_CFGR_CKPOL_Pos);

  }
  else
  {
    if (IS_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS((uint32_t)clk_src) != 0U)
    {
      /* Just to make sure that LL_LPTIM_COUNTER_MODE_EXTERNAL is 0. */
      clk_src &= ~LL_LPTIM_COUNTER_MODE_EXTERNAL;
    }
  }

  return (hal_lptim_clk_src_t)clk_src;
}

/**
  * @brief  Set the clock source of the low-power timer time-base unit.
  * @param  p_lptim  CMSIS LPTIM instance.
  * @note   Ensure the LPTIM instance is disabled.
  * @param  clk_src  Configured clock source.
  */
__STATIC_FORCEINLINE void LPTIM_SetClockSource(lptim_t *p_lptim,
                                               hal_lptim_clk_src_t clk_src)
{
  volatile uint32_t cfgr = LL_LPTIM_READ_REG(p_lptim, CFGR);

  /* Reset CKSEL (which is set to 1 only when clk_src is
     HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS */
  cfgr &= ~((uint32_t)(HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS));

  /* Reset COUNTMODE */
  cfgr &= ~LL_LPTIM_COUNTER_MODE_EXTERNAL;

  if (IS_LPTIM_CLOCK_TYPE_ENCODER((uint32_t)clk_src) != 0U)
  {
    /* Reset the polarity. */
    cfgr &= ~LPTIM_CLOCK_POLARITY_MASK;
  }

  cfgr |= (uint32_t)clk_src;

  LL_LPTIM_WRITE_REG(p_lptim, CFGR, cfgr);
}

/**
  * @brief  Program the pulse width for output channel mode.
  * @param  p_lptim     CMSIS LPTIM instance
  * @param  channel     Output channel of interest. \n
  *                       Must be one of the following values: \n
  *                       @arg @ref HAL_LPTIM_CHANNEL_1
  *                       @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  pulse       Config pulse.
  * @retval HAL_OK
  * @retval HAL_ERROR   LPTIM_CMP write operation failed.
  */
static hal_status_t LPTIM_OC_SetPulse(lptim_t *p_lptim,
                                      hal_lptim_channel_t channel,
                                      uint32_t pulse)
{
  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  if (channel == HAL_LPTIM_CHANNEL_1)
  {
    /* Clear the compare register 1 update flag */
    LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_ISR_CMP1OK);

    /* Write to CCR1 register the pulse value */
    LL_LPTIM_OC_SetCompareCH1(p_lptim, pulse);
    if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_CMP1OK) != HAL_OK)
    {
      return HAL_ERROR;
    }
    LL_LPTIM_ClearFlag_CMP1OK(p_lptim);
  }
  else /* HAL_LPTIM_CHANNEL_2 */
  {
    /* Clear the compare register 2 update flag */
    LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_ISR_CMP2OK);

    /* Write to CCR2 register the pulse value */
    LL_LPTIM_OC_SetCompareCH2(p_lptim, pulse);
    if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_CMP2OK) != HAL_OK)
    {
      return HAL_ERROR;
    }
    LL_LPTIM_ClearFlag_CMP2OK(p_lptim);
  }

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Start the low-power timer with the selected mode.
  * @param  p_lptim     CMSIS LPTIM instance.
  * @param  mode        Selection of the LPTIM modes (list here: @ref hal_lptim_mode_t).
  * @retval HAL_OK
  * @retval HAL_ERROR   Error: encoder clock initialized but continuous mode set.
  */
static hal_status_t LPTIM_Start(lptim_t *p_lptim, uint32_t mode)
{
  /* Ensure the LPTIM instance is disabled, but disable it again for safety. */
  LL_LPTIM_Disable(p_lptim);

  volatile uint32_t cfgr = LL_LPTIM_READ_REG(p_lptim, CFGR);

  if ((IS_LPTIM_CLOCK_TYPE_ENCODER(LPTIM_GET_CLOCK_TYPE(cfgr)) != 0U) && (mode != (uint32_t)HAL_LPTIM_CONTINUOUS))
  {
    return HAL_ERROR;
  }

  cfgr &= ~LPTIM_MODE_CFGR_MASK;
  cfgr |= (mode & LPTIM_MODE_CFGR_MASK);
  LL_LPTIM_WRITE_REG(p_lptim, CFGR, cfgr);

  return HAL_OK;
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)

/**
  * @brief  Get the channel associated with a DMA handler.
  * @param  hlptim Pointer to the handle of the LPTIM instance.
  * @param  hdma Pointer to the handle of the DMA instance.
  * @return @ref hal_lptim_channel_t Channel number of the handler.
  */
__STATIC_INLINE hal_lptim_channel_t LPTIM_GetCCxDmaHandler(const hal_lptim_handle_t *hlptim,
                                                           const hal_dma_handle_t *hdma)
{
  if (hdma == hlptim->hdma[HAL_LPTIM_DMA_ID_CC1])
  {
    return HAL_LPTIM_CHANNEL_1;
  }
  else
  {
    return HAL_LPTIM_CHANNEL_2;
  }
}


/**
  * @brief Capture half complete.
  * @param hdma Pointer to the DMA handler.
  */
static void LPTIM_DMACaptureHalfcpltCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  const hal_lptim_channel_t channel = LPTIM_GetCCxDmaHandler(hlptim, hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->input_capture_half_cplt_callback(hlptim, channel);
#else
  HAL_LPTIM_InputCaptureHalfCpltCallback(hlptim, channel);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief Capture complete.
  * @param hdma Pointer to the DMA handler.
  */
static void LPTIM_DMACaptureCpltCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_lptim_channel_t channel = LPTIM_GetCCxDmaHandler(hlptim, hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->input_capture_callback(hlptim, channel);
#else
  HAL_LPTIM_InputCaptureCallback(hlptim, channel);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief DMA transfer error callback.
  * @param hdma Pointer to the DMA handler.
  */
static void LPTIM_DMAErrorCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->error_callback(hlptim);
#else
  HAL_LPTIM_ErrorCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief DMA transfer stopped callback when triggered by an LPTIM update event.
  * @param hdma Pointer to the DMA handler.
  */
static void LPTIM_DMAStopCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->stop_callback(hlptim);
#else
  HAL_LPTIM_StopCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief Callback for DMA channel stop.
  * @param hdma Pointer to the DMA handler.
  */
static void LPTIM_DMAChannelStopCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_lptim_channel_t channel = LPTIM_GetCCxDmaHandler(hlptim, hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->input_capture_stop_callback(hlptim, channel);
#else
  HAL_LPTIM_InputCaptureStopCallback(hlptim, channel);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief Callback for DMA update half complete.
  * @param  hdma  Pointer to the DMA handle.
  */
static void LPTIM_DMAUpdateHalfcpltCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->update_half_cplt_callback(hlptim);
#else
  HAL_LPTIM_UpdateHalfCpltCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief Callback for DMA update complete.
  * @param  hdma  Pointer to the DMA handle.
  */
static void LPTIM_DMAUpdateCpltCallback(hal_dma_handle_t *hdma)
{
  hal_lptim_handle_t *hlptim = LPTIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  hlptim->update_callback(hlptim);
#else
  HAL_LPTIM_UpdateCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
}

/**
  * @brief  Start the low-power timer in DMA mode (optional DMA interrupts).
  * @param  hlptim       Pointer to the LPTIM handle.
  * @param  p_data       Pointer to data.
  * @param  size_byte    Size of data.
  * @param  interrupts  Selection of DMA interrupts. \n
  *                     Can be any of the following values:
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_NONE
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_HT
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_DEFAULT
  * @if (USE_HAL_DMA_LINKEDLIST)
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_SILENT
  * @endif
  * @retval HAL_OK
  * @retval HAL_ERROR    LPTIM_DIER write operation failed.
  */
static hal_status_t LPTIM_Start_DMA_Opt(hal_lptim_handle_t *hlptim,
                                        const uint8_t *p_data,
                                        uint32_t size_byte,
                                        uint32_t interrupts)
{

  hal_dma_handle_t *hdma = hlptim->hdma[HAL_LPTIM_DMA_ID_UPDATE];
  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((hdma != NULL));

  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_EnableDMAReq_UPDATE(p_lptim);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_LPTIM_DMA_VALID_SILENT_MODE(hlptim, HAL_LPTIM_DMA_ID_UPDATE, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set DMA channel callback function pointers */
  hdma->p_xfer_halfcplt_cb = LPTIM_DMAUpdateHalfcpltCallback;
  hdma->p_xfer_cplt_cb = LPTIM_DMAUpdateCpltCallback;
  hdma->p_xfer_error_cb = LPTIM_DMAErrorCallback;

  /* Start DMA transfer in IT mode: from Memory to ARR register */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma, (uint32_t)p_data,
                                     (uint32_t)((uint32_t *)(&p_lptim->ARR)),
                                     size_byte, interrupts) != HAL_OK)
  {
#if defined(USE_HAL_LPTIM_GET_LAST_ERRORS) && (USE_HAL_LPTIM_GET_LAST_ERRORS == 1)
    hlptim->last_error_codes |= HAL_LPTIM_ERROR_DMA;
#endif /* USE_HAL_LPTIM_GET_LAST_ERRORS */

    hlptim->global_state = HAL_LPTIM_STATE_IDLE;

    return HAL_ERROR;
  }

  LL_LPTIM_StartCounter(p_lptim, (LPTIM_MODE_CR_MASK & (uint32_t)hlptim->mode));

  return HAL_OK;
}

/**
  * @brief  Start input capture channel with DMA and interrupts.
  * @param  hlptim       Pointer to the handle of the LPTIM instance.
  * @param  channel      Channel ID.
  * @param  p_data       Pointer to data.
  * @param  size_byte    Size of data.
  * @param  interrupts  Selection of DMA interrupts. \n
  *                     Can be any of the following values:
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_NONE
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_HT
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_DEFAULT
  * @if (USE_HAL_DMA_LINKEDLIST)
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_SILENT
  * @endif
  * @retval HAL_ERROR    Input channel with DMA and interrupts did not start correctly.
  * @retval HAL_OK       Input channel with DMA and interrupts started correctly.
  */
__STATIC_INLINE hal_status_t LPTIM_IC_StartChannel_DMA_Opt(hal_lptim_handle_t *hlptim,
                                                           hal_lptim_channel_t channel,
                                                           const uint8_t *p_data,
                                                           uint32_t size_byte,
                                                           uint32_t interrupts)
{
  hal_dma_handle_t *hdma;
  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  typedef struct
  {
    uint32_t id_dma;
    uint32_t src_addr;
    void (*lptim_enable_dma_cb)(lptim_t *);
  } lptim_ic_start_mapping_dma_t;

  lptim_ic_start_mapping_dma_t mapping_dma[] =
  {
    {HAL_LPTIM_DMA_ID_CC1, (uint32_t)((uint32_t *)(&p_lptim->CCR1)), LL_LPTIM_EnableDMAReq_CC1},
    {HAL_LPTIM_DMA_ID_CC2, (uint32_t)((uint32_t *)(&p_lptim->CCR2)), LL_LPTIM_EnableDMAReq_CC2},
  };

  /* Temporarily enable the peripheral to modify DIER impact by EnableDMAReq. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  hdma = hlptim->hdma[mapping_dma[(uint32_t)channel].id_dma];

  ASSERT_DBG_PARAM((hdma != NULL));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
  ASSERT_DBG_PARAM(IS_LPTIM_DMA_VALID_SILENT_MODE(hlptim, mapping_dma[(uint32_t)channel].id_dma, interrupts));
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set DMA channel callback function pointers */
  hdma->p_xfer_halfcplt_cb = LPTIM_DMACaptureHalfcpltCallback;
  hdma->p_xfer_cplt_cb = LPTIM_DMACaptureCpltCallback;
  hdma->p_xfer_error_cb = LPTIM_DMAErrorCallback;

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                     mapping_dma[(uint32_t)channel].src_addr,
                                     (uint32_t)p_data,
                                     size_byte, interrupts) != HAL_OK)
  {
#if defined(USE_HAL_LPTIM_GET_LAST_ERRORS) && (USE_HAL_LPTIM_GET_LAST_ERRORS == 1)
    hlptim->last_error_codes |= HAL_LPTIM_ERROR_DMA;
#endif /* USE_HAL_LPTIM_GET_LAST_ERRORS */
    hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_IC_CHANNEL_STATE_IDLE;
    return HAL_ERROR;
  }

  mapping_dma[(uint32_t)channel].lptim_enable_dma_cb(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  LL_LPTIM_CC_EnableChannel(p_lptim, (uint32_t)channel);

  return HAL_OK;

}

/**
  * @brief  Abort any ongoing DMA channel transfer.
  * @param  hlptim     Pointer to the handle of the TIM instance.
  * @param  dma_idx  DMA handle index
  * @param  active_silent_mode  Status of the silent mode.
  */
__STATIC_INLINE void LPTIM_Abort_DMA(const hal_lptim_handle_t *hlptim,
                                     const hal_lptim_dma_index_t dma_idx,
                                     const uint32_t active_silent_mode)
{
  hal_dma_cb_t xfer_abort_cb;
  hal_dma_handle_t *hdma = hlptim->hdma[dma_idx];

  ASSERT_DBG_PARAM((hdma != NULL));

  if (active_silent_mode == LPTIM_ACTIVE_SILENT)
  {
    (void)HAL_DMA_Abort(hdma);

    return;
  }

  /* dma stop callback function pointer depends on the dma request source */
  if (dma_idx == HAL_LPTIM_DMA_ID_UPDATE)
  {
    xfer_abort_cb = LPTIM_DMAStopCallback;
  }
  else
  {
    xfer_abort_cb = LPTIM_DMAChannelStopCallback;
  }

  hdma->p_xfer_abort_cb = xfer_abort_cb;
  if (HAL_DMA_Abort_IT(hdma) != HAL_OK)
  {
    xfer_abort_cb(hdma);
  }
}

/**
  * @brief Stop a low-power timer input channel that was started in DMA mode.
  * @param hlptim   Pointer to the handle of the LPTIM instance.
  * @param channel  Channel number to disable.
  * @param active_silent_mode Active silent mode.
  */
static void LPTIM_IC_StopChannel_DMA(const hal_lptim_handle_t *hlptim,
                                     const hal_lptim_channel_t channel,
                                     const uint32_t active_silent_mode)
{
  typedef struct
  {
    hal_lptim_dma_index_t id_dma;
    void (*lptim_disable_dma_cb)(lptim_t *);
  } lptim_ic_stop_mapping_dma_t;

  lptim_ic_stop_mapping_dma_t mapping_dma[] =
  {
    {HAL_LPTIM_DMA_ID_CC1, LL_LPTIM_DisableDMAReq_CC1},
    {HAL_LPTIM_DMA_ID_CC2, LL_LPTIM_DisableDMAReq_CC2},
  };

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  /* Disable capture/compare channel match DMA request */
  mapping_dma[(uint32_t)channel].lptim_disable_dma_cb(p_lptim);

  /* Abort DMA */
  LPTIM_Abort_DMA(hlptim,
                  mapping_dma[(uint32_t)channel].id_dma,
                  active_silent_mode);

}

#endif /* USE_HAL_LPTIM_DMA */

/**
  *  @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup LPTIM_Exported_Functions
  *  @{
  */

/** @addtogroup LPTIM_Exported_Functions_Group1
  *  @{
     This section provides a set of function allowing to:
    - Initialize and deinitialize LPTIM with HAL_LPTIM_Init() and HAL_LPTIM_DeInit()
    - Associate DMA channels to LPTIM DMA requests with HAL_LPTIM_SetDMA()
  */

/**
  * @brief  Initialization function.
  *         Initialize the LPTIM handle and associate an instance.
  * @param  hlptim     Pointer to the handler of the LPTIM instance.
  * @param  instance   One of the values of the hal_lptim_t enumeration.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM Input parameter is invalid
  *                           (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_Init(hal_lptim_handle_t *hlptim, hal_lptim_t instance)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM(IS_LPTIM_INSTANCE((lptim_t *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hlptim == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the LPTIM instance */
  hlptim->instance = instance;

#if defined(USE_HAL_LPTIM_CLK_ENABLE_MODEL) && (USE_HAL_LPTIM_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  LPTIM_EnableClock(instance);
#endif /* USE_HAL_LPTIM_CLK_ENABLE_MODEL */

#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
  LPTIM_InitCallbacks(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
  for (uint32_t dma_idx = 0; dma_idx < LPTIM_DMA_REQUESTS; dma_idx++)
  {
    hlptim->hdma[dma_idx] = NULL;
  }
#endif /* USE_HAL_LPTIM_DMA */

  /* Init the handle internal parameters */

#if defined(USE_HAL_LPTIM_USER_DATA) && (USE_HAL_LPTIM_USER_DATA == 1)
  hlptim->p_user_data = NULL;
#endif /* USE_HAL_LPTIM_USER_DATA */

  /* Reset channels state */
  for (uint32_t i = 0; i < HAL_LPTIM_CHANNELS; ++i)
  {
    hlptim->channel_states[i] = HAL_LPTIM_CHANNEL_STATE_RESET;
  }

#if defined(USE_HAL_LPTIM_GET_LAST_ERRORS) && (USE_HAL_LPTIM_GET_LAST_ERRORS == 1)
  hlptim->last_error_codes = HAL_LPTIM_ERROR_NONE;
#endif /* USE_HAL_LPTIM_GET_LAST_ERRORS */

  hlptim->global_state = HAL_LPTIM_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief   Reset function.\n
  *          Stop all current operations and reset states.\n
  *          Hence: \n
  *          @arg stop the counter.
  *          @arg disable interrupts / DMA transfers.
  *          @arg clear status flags.
  *          @arg set channels' states to RESET.
  *          @arg set global state to RESET.
  *
  * @param   hlptim  Pointer to the handler of the LPTIM instance.
  * @warning Be careful if you used an external clock to have called HAL_LPTIM_SetConfigInput1() before Deinit!
  */
void HAL_LPTIM_DeInit(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_INSTANCE(p_lptim));

  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_WRITE_REG(p_lptim, DIER, 0U);

  (void)LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK);

  LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_FLAG_ALL);

  /* Disable the LPTIM instance */
  LL_LPTIM_Disable(p_lptim);

  /* Reset channels state */
  for (uint32_t channel_id = 0; channel_id < HAL_LPTIM_CHANNELS; ++channel_id)
  {
    LL_LPTIM_CC_DisableChannel(p_lptim, channel_id);
    hlptim->channel_states[channel_id] = HAL_LPTIM_CHANNEL_STATE_RESET;
  }

  hlptim->global_state = HAL_LPTIM_STATE_RESET;
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)

/**
  * @brief    Link a DMA handle to a DMA request.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    dma_idx    Index of the DMA request.
  * @param    hdma       Pointer to a handle of the DMA instance.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_SetDMA(hal_lptim_handle_t *hlptim,
                              hal_lptim_dma_index_t dma_idx,
                              hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  ASSERT_DBG_PARAM(IS_LPTIM_DMA_INDEX(dma_idx));

  /* Link the DMA handle to the LPTIM handle. */
  hlptim->hdma[(uint32_t)dma_idx] = hdma;
  hdma->p_parent = hlptim;

  return HAL_OK;
}

#endif /* USE_HAL_LPTIM_DMA */

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group2
  *  @{
  * This section provides a set of functions for state and error management.
  *  - Call HAL_LPTIM_GetState() to get information about the low-power timer state.
  *  - Call HAL_LPTIM_GetChannelState() to get the channel state.
  *  - Call HAL_LPTIM_GetLastErrorCodes() to get the last error codes.
  */

/**
  * @brief  Get the low-power timer state.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_state_t HAL LPTIM state.
  */
hal_lptim_state_t HAL_LPTIM_GetState(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  return hlptim->global_state;
}

/**
  * @brief  Get the state of a channel.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  channel Channel of interest.
  * @retval hal_lptim_channel_state_t LPTIM channel state.
  */
hal_lptim_channel_state_t HAL_LPTIM_GetChannelState(const hal_lptim_handle_t *hlptim,
                                                    hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));
  return hlptim->channel_states[(uint32_t)channel];
}

#if defined(USE_HAL_LPTIM_GET_LAST_ERRORS) && (USE_HAL_LPTIM_GET_LAST_ERRORS == 1)

/**
  * @brief  Retrieve the HAL LPTIM last errors.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @retval uint32_t HAL LPTIM bit-mapped last errors.\n
  *                  Values are:\n
  *                  @ref HAL_LPTIM_ERROR_NONE \n
  *                  @ref HAL_LPTIM_ERROR_DMA \n
  *                  @ref HAL_LPTIM_ERROR_TIMEOUT
  */
uint32_t HAL_LPTIM_GetLastErrorCodes(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  return hlptim->last_error_codes;
}

#endif /* USE_HAL_LPTIM_GET_LAST_ERRORS */

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group3
  *  @{
  *     This group contains the functions used to configure and control
  *           the time-base unit.
  *
  *           - Call HAL_LPTIM_SetConfig() to set the LPTIM configuration.
  *           - Call HAL_LPTIM_GetConfig() to get the LPTIM configuration.
  *           - Call HAL_LPTIM_SetMode() to set the LPTIM mode (CONTINUOUS or ONE-SHOT).
  *           - Call HAL_LPTIM_GetMode() to get the LPTIM mode.
  *           - Call HAL_LPTIM_SetClockSource() to set the LPTIM clock source (INTERNAL, EXTERNAL SYNCHRONOUS,
  *                                               EXTERNAL ASYNCHRONOUS, ENCODER SUBMODE 1 TO 3).
  *           - Call HAL_LPTIM_GetClockSource() to get the LPTIM clock source.
  *           - Call HAL_LPTIM_SetClockSourcePrescaler() to set the LPTIM clock source prescaler division.
  *           - Call HAL_LPTIM_GetClockSourcePrescaler() to get the LPTIM clock source prescaler division.
  *           - Call HAL_LPTIM_SetPeriod() to set the period.
  *           - Call HAL_LPTIM_GetPeriod() to get the period.
  *           - Call HAL_LPTIM_SetRepetitionCounter() to set the repetition counter.
  *           - Call HAL_LPTIM_GetRepetitionCounter() to get the repetition counter.
  *           - Call HAL_LPTIM_GetCounter() to get the counter.
  *           - Call HAL_LPTIM_ResetCounter() to reset the counter.
  *           - Call HAL_LPTIM_EnableResetCounterAfterRead() to enable reset counter after read.
  *           - Call HAL_LPTIM_DisableResetCounterAfterRead() to disable reset counter after read.
  *           - Call HAL_LPTIM_IsEnableResetCounterAfterRead() to check whether reset counter after read is enabled.
  *           - Call HAL_LPTIM_EnablePreload() to enable preload.
  *           - Call HAL_LPTIM_DisablePreload() to disable preload.
  *           - Call HAL_LPTIM_IsEnabledPreload() to check whether preload is enabled.
  *           - Call HAL_LPTIM_SetConfigInput1() to configure Input1.
  *           - Call HAL_LPTIM_GetConfigInput1() to get the configuration for Input1.
  *           - Call HAL_LPTIM_SetInput1Source() to set the Input1 source.
  *           - Call HAL_LPTIM_GetInput1Source() to get the Input1 source setup.
  *           - Call HAL_LPTIM_SetInput1Polarity() to set Input1 polarity (RISING, FALLING, or both).
  *           - Call HAL_LPTIM_GetInput1Polarity() to get the Input1 polarity setup.
  *           - Call HAL_LPTIM_SetInput1Filter() to set the Input1 filter.
  *           - Call HAL_LPTIM_GetInput1Filter() to get the Input1 filter setup.
  *
  * @note When the clock source is HAL_LPTIM_CLK_ENCODER_SUBMODE_[1|2|3],
  *       selection of the sources (two signals from quadrature encoders)
  *       is done with HAL_LPTIM_SetConfigEncoder().
  *
  */

/**
  * @brief  Configure the low-power timer time-base unit.
  * @param  hlptim     Pointer to the handle of the LPTIM instance.
  * @param  p_config   Pointer to the time-base unit configuration structure.
  * @note   If the clock source is set to HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or HAL_LPTIM_CLK_ENCODER_SUBMODE_[1|2|3],
  *         the prescaler is forced to HAL_LPTIM_CLK_SRC_DIV1.
  * @retval HAL_OK
  * @retval HAL_ERROR         LPTIM_ISR or LPTIM_ARR write operation failed.
  * @retval HAL_INVALID_PARAM Input parameter is invalid.
  *                           (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_SetConfig(hal_lptim_handle_t *hlptim,
                                 const hal_lptim_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  /* Check parameters */
  ASSERT_DBG_PARAM(IS_LPTIM_CLK_SRC(p_lptim, p_config->clock_source));
  ASSERT_DBG_PARAM(IS_LPTIM_MODE(p_config->clock_source,
                                 p_config->mode));
  ASSERT_DBG_PARAM(IS_LPTIM_CLK_SRC_PRESC(p_config->clock_source,
                                          p_config->prescaler));
  ASSERT_DBG_PARAM(IS_LPTIM_PERIOD(p_config->period));
  ASSERT_DBG_PARAM(IS_LPTIM_REPETITION_COUNTER(p_config->repetition_counter));

  /* Store the mode (configuration is done in the process function). */
  hlptim->mode = p_config->mode;

  LL_LPTIM_Enable(p_lptim);

  /* Clear all flags */
  LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_FLAG_ALL);

  /* Set the repetition counter. */
  LL_LPTIM_SetRepetition(p_lptim, p_config->repetition_counter);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_REPOK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_REPOK(p_lptim);

  /* Set the period. */
  LL_LPTIM_SetAutoReload(p_lptim, p_config->period);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_ARROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_ARROK(p_lptim);

  LL_LPTIM_Disable(p_lptim);

  /* Clock source configuration */
  LPTIM_SetClockSource(p_lptim, p_config->clock_source);

  /* If the clock source is HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or
     HAL_LPTIM_CLK_ENCODER_SUBMODE_[1|2|3] the prescaler must be
     HAL_LPTIM_PRESCALER_DIV1 (that is 000). */
  if ((p_config->clock_source == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS)
      || (IS_LPTIM_CLOCK_TYPE_ENCODER((uint32_t)p_config->clock_source) != 0U))
  {
    LL_LPTIM_SetPrescaler(p_lptim, (uint32_t)HAL_LPTIM_CLK_SRC_DIV1);
  }
  else
  {
    LL_LPTIM_SetPrescaler(p_lptim, (uint32_t)p_config->prescaler);
  }

  /* Reset channels (needed only in IDLE state but done by default). */
  for (uint32_t i = 0; i < HAL_LPTIM_CHANNELS; ++i)
  {
    hlptim->channel_states[i] = HAL_LPTIM_CHANNEL_STATE_RESET;
  }

  hlptim->global_state = HAL_LPTIM_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the low-power timer time-base unit.
  * @param  hlptim     Pointer to the handle of the LPTIM instance.
  * @param  p_config   Pointer to a time-base unit configuration structure to fill.
  */
void HAL_LPTIM_GetConfig(const hal_lptim_handle_t *hlptim,
                         hal_lptim_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  p_config->mode = hlptim->mode;
  p_config->clock_source = LPTIM_GetClockSource(p_lptim);
  p_config->prescaler = (hal_lptim_clk_src_presc_t)LL_LPTIM_GetPrescaler(p_lptim);
  p_config->period = LL_LPTIM_GetAutoReload(p_lptim);
  p_config->repetition_counter = LL_LPTIM_GetRepetition(p_lptim);
}

/**
  * @brief    Set the mode of the low-power timer time-base unit.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    mode    The counter mode selected.
  * @warning  Calling this function while the clock source is HAL_LPTIM_CLK_ENCODER_SUBMODE_x has no effect.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_SetMode(hal_lptim_handle_t *hlptim,
                               hal_lptim_mode_t mode)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = (lptim_t *)LPTIM_INSTANCE(hlptim);
  hal_lptim_clk_src_t clk_src = LPTIM_GetClockSource(p_lptim);

  if (IS_LPTIM_CLK_ENCODER(clk_src) == 0U)
  {
    hlptim->mode = mode;
  }
  return HAL_OK;
}

/**
  * @brief  Get the mode of the low-power timer time-base unit.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_mode_t  Mode in which the low-power timer runs.
  */
hal_lptim_mode_t HAL_LPTIM_GetMode(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return hlptim->mode;
}

/**
  * @brief  Set the clock source of the low-power timer time-base unit.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  clk_src   Clock source selection.
  * @retval HAL_OK
  *
  */
hal_status_t HAL_LPTIM_SetClockSource(const hal_lptim_handle_t *hlptim,
                                      hal_lptim_clk_src_t clk_src)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM(IS_LPTIM_CLK_SRC(LPTIM_INSTANCE(hlptim), clk_src));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  LPTIM_SetClockSource(LPTIM_INSTANCE(hlptim), clk_src);

  return HAL_OK;
}

/**
  * @brief  Get the clock source of the low-power timer time-base unit.
  * @param  hlptim                Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_clk_src_t   Clock source of the LPTIM instance.
  *
  */
hal_lptim_clk_src_t HAL_LPTIM_GetClockSource(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));
  return LPTIM_GetClockSource(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief  Set the clock source prescaler of the low-power timer time-base unit.
  * @param  hlptim          Pointer to the handle of the LPTIM instance.
  * @param  clk_src_presc   Clock source prescaler for the time base unit.
  * @note   Clock prescaler setting has no effect if the clock source is
  *         HAL_LPTIM_CLK_ENCODER_SUBMODE_[1|2|3].
  * @note   The prescaler must not be used (DIV1) when the LPTIM external Input1 is sampled with the
  *         internal clock (HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS).
  * @retval HAL_OK
  */

hal_status_t HAL_LPTIM_SetClockSourcePrescaler(const hal_lptim_handle_t *hlptim,
                                               hal_lptim_clk_src_presc_t clk_src_presc)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CLK_SRC_PRESC(LPTIM_GetClockSource(p_lptim), clk_src_presc));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  LL_LPTIM_SetPrescaler(p_lptim, (uint32_t)clk_src_presc);

  return HAL_OK;
}

/**
  * @brief  Get the clock source prescaler of the low-power timer.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_clk_src_presc_t  Clock source prescaler of the LPTIM instance.
  *
  */
hal_lptim_clk_src_presc_t HAL_LPTIM_GetClockSourcePrescaler(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_clk_src_presc_t)LL_LPTIM_GetPrescaler(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief  Set the period of the low-power timer time-base unit.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @param  period   Period for the time base unit.
  * @retval HAL_OK
  * @retval HAL_ERROR  LPTIM_ARR write operation failed.
  */
hal_status_t HAL_LPTIM_SetPeriod(const hal_lptim_handle_t *hlptim,
                                 uint32_t period)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_PERIOD(period));

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  /* Clear flag */
  LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_ISR_ARROK);

  /* Set the period and wait for the register to be updated. */
  LL_LPTIM_SetAutoReload(p_lptim, period);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_ARROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_ARROK(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Get the period of the low-power timer.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @return uint32_t  Period Value
  */
uint32_t HAL_LPTIM_GetPeriod(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return LL_LPTIM_GetAutoReload(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief  Set the repetition counter of the low-power timer time-base unit.
  * @param  hlptim              Pointer to the handle of the LPTIM instance.
  * @param  repetition_counter  Repetition value for the time base unit.
  * @retval HAL_OK
  * @retval HAL_ERROR  LPTIM_ISR write operation failed
  */
hal_status_t HAL_LPTIM_SetRepetitionCounter(const hal_lptim_handle_t *hlptim,
                                            uint32_t repetition_counter)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  ASSERT_DBG_PARAM(IS_LPTIM_REPETITION_COUNTER(repetition_counter));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  /* Clear flag */
  LL_LPTIM_WRITE_REG(p_lptim, ICR, LL_LPTIM_ISR_REPOK);

  /* Set the repetition counter and wait for the register to be updated. */
  LL_LPTIM_SetRepetition(p_lptim, repetition_counter);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_REPOK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_REPOK(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Get the value of the repetition counter of the low-power timer.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval uint32_t  Value of the repetition counter.
  */
uint32_t HAL_LPTIM_GetRepetitionCounter(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return LL_LPTIM_GetRepetition(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief    Get Counter Register (LPTIMx_CNT) value.
  * @param    hlptim    Pointer to the handle of the LPTIM instance.
  * @warning  When the LPTIM instance is running, reading
  *           the LPTIMx_CNT register can return unreliable values. So in this case
  *           it is necessary to perform two consecutive read accesses and verify
  *           that the two returned values are identical.
  * @return   Counter register value.
  */
uint32_t HAL_LPTIM_GetCounter(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return LL_LPTIM_GetCounter(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief  Reset Counter Register.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_ResetCounter(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  LL_LPTIM_ResetCounter(LPTIM_INSTANCE(hlptim));

  return HAL_OK;
}

/**
  * @brief  Enable Reset Counter After read.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_EnableResetCounterAfterRead(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_ACTIVE);

  LL_LPTIM_EnableResetAfterRead(LPTIM_INSTANCE(hlptim));

  return HAL_OK;
}

/**
  * @brief  Disable Reset Counter After read.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_DisableResetCounterAfterRead(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_ACTIVE);

  LL_LPTIM_DisableResetAfterRead(LPTIM_INSTANCE(hlptim));

  return HAL_OK;
}

/**
  * @brief  Check Reset Counter After read.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_reset_after_read_status_t Registers update mode.
  */
hal_lptim_reset_after_read_status_t HAL_LPTIM_IsEnableResetCounterAfterRead(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_reset_after_read_status_t)LL_LPTIM_IsEnabledResetAfterRead(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief  Enable the preload.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @note   registers ARR, RCR and CCRx will be updated only at the end of the current LPTIM period.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_EnablePreload(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  /* Disable LPTIM if it was enabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Disable(p_lptim);
  LL_LPTIM_SetUpdateMode(p_lptim, (uint32_t)HAL_LPTIM_PRELOAD_ENABLED);
  if (is_lptim_enabled != 0U)
  {
    LL_LPTIM_Enable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Disable the preload.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @note   registers ARR, RCR and CCR are updated after each APB bus access.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_DisablePreload(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  /* Disable LPTIM if it was enabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Disable(p_lptim);
  LL_LPTIM_SetUpdateMode(p_lptim, (uint32_t)HAL_LPTIM_PRELOAD_DISABLED);
  if (is_lptim_enabled != 0U)
  {
    LL_LPTIM_Enable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Check preload state.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_preload_status_t Registers update mode.
  */
hal_lptim_preload_status_t HAL_LPTIM_IsEnabledPreload(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_preload_status_t)LL_LPTIM_GetUpdateMode(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief    Configure Input1.
  * @param    hlptim    Pointer to the handle of the LPTIM instance.
  * @param    p_config  Pointer to the input1 configuration structure.
  * @warning  This function must be called only after the clock source is
  *           configured.
  * @warning  If the clock is HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS but the filter
  *           is not HAL_LPTIM_FDIV1, or the polarity is HAL_LPTIM_INPUT1_RISING_FALLING then an auxiliary clock
  *           (one of the Low power oscillator) must be active.
  * @retval   HAL_OK
  * @retval   HAL_ERROR When called with clock source different from
  *                     HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or
  *                     HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS.
  */

hal_status_t HAL_LPTIM_SetConfigInput1(const hal_lptim_handle_t *hlptim,
                                       const hal_lptim_input1_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_INSTANCE(p_lptim));
  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_SRC(p_config->src));
  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(p_config->filter));

  hal_lptim_clk_src_t clk_src = LPTIM_GetClockSource(p_lptim);

  if (!((clk_src == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS)
        || (clk_src == HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS)))
  {
    return HAL_ERROR;
  }

  LL_LPTIM_SetInput1Source(p_lptim, (uint32_t)p_config->src);

  /* Configure the polarity and the filter together (CKPOL and CKFLT in CFGR). */
  LL_LPTIM_ConfigClock(p_lptim, (uint32_t)(p_config->polarity), LPTIM_CFGR_HAL2LL_FILTER((uint32_t)p_config->filter));

  return HAL_OK;
}

/**
  * @brief  Get Input1 configuration.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  p_config  Pointer to an input1 configuration structure.
  */
void HAL_LPTIM_GetConfigInput1(const hal_lptim_handle_t *hlptim,
                               hal_lptim_input1_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  p_config->polarity = (hal_lptim_input1_polarity_t)LL_LPTIM_GetClockPolarity(p_lptim);
  p_config->filter = (hal_lptim_filter_t)(LPTIM_GET_CLOCK_FILTER(p_lptim));
  p_config->src = (hal_lptim_input1_src_t)LL_LPTIM_GetInput1Source(p_lptim);
}

/**
  * @brief    Configure the Input1 source.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    input1_src Source of Input1.
  * @warning  This function must be called only after the clock source is
  *           configured.
  * @retval   HAL_OK
  * @retval   HAL_ERROR When called with clock source different from
  *                     HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS.
  */
hal_status_t HAL_LPTIM_SetInput1Source(const hal_lptim_handle_t *hlptim,
                                       hal_lptim_input1_src_t input1_src)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_INSTANCE(p_lptim));
  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_SRC(input1_src));

  hal_lptim_clk_src_t clk_src = LPTIM_GetClockSource(p_lptim);

  if (!((clk_src == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS) || (clk_src == HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS)))
  {
    return HAL_ERROR;
  }

  LL_LPTIM_SetInput1Source(p_lptim, (uint32_t)input1_src);

  return HAL_OK;
}

/**
  * @brief  Get the input1 source.
  * @param  hlptim     Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_input1_src_t The input1 source.
  */
hal_lptim_input1_src_t HAL_LPTIM_GetInput1Source(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_input1_src_t)LL_LPTIM_GetInput1Source(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief    Set Input1 polarity.
  * @param    hlptim      Pointer to the handle of the LPTIM instance.
  * @param    polarity    polarity of Input1.
  * @warning  This function must be called only after the clock source is
  *           configured.
  * @warning  An auxiliary clock (one of the Low power oscillator) must be active
  *           if the polarity is HAL_LPTIM_INPUT1_RISING_FALLING.
  * @retval   HAL_OK
  * @retval   HAL_ERROR When called with clock source different from
  *                     HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS.
  */
hal_status_t HAL_LPTIM_SetInput1Polarity(const hal_lptim_handle_t *hlptim,
                                         hal_lptim_input1_polarity_t polarity)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);
  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_INSTANCE(p_lptim));
  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_POLARITY(polarity));

  hal_lptim_clk_src_t clk_src = LPTIM_GetClockSource(p_lptim);

  if (!((clk_src == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS) || (clk_src == HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS)))
  {
    return HAL_ERROR;
  }

  LL_LPTIM_SetClockPolarity(p_lptim, (uint32_t)polarity);

  return HAL_OK;
}

/**
  * @brief  Get Input1 polarity.
  * @param  hlptim     Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_input1_polarity_t Input1 polarity.
  */
hal_lptim_input1_polarity_t HAL_LPTIM_GetInput1Polarity(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_input1_polarity_t)LL_LPTIM_GetClockPolarity(LPTIM_INSTANCE(hlptim));
}

/**
  * @brief    Configure the Input1 filter.
  * @param    hlptim   Pointer to the handle of the LPTIM instance.
  * @param    filter   Filter for Input1.
  * @note     If filtering is used, an auxiliary clock must be active.
  * @warning  This function must be called only after the clock source is
  *           configured.
  * @warning  An auxiliary clock (one of the Low power oscillator) must be active
  *           if the value of the filter is different from HAL_LPTIM_FDIV1.
  * @retval   HAL_OK
  * @retval   HAL_ERROR When called with clock source different from
  *                     HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS or HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS.
  */

hal_status_t HAL_LPTIM_SetInput1Filter(const hal_lptim_handle_t *hlptim,
                                       hal_lptim_filter_t filter)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_INSTANCE(p_lptim));
  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(filter));

  hal_lptim_clk_src_t clk_src = LPTIM_GetClockSource(p_lptim);

  if (!((clk_src == HAL_LPTIM_CLK_EXTERNAL_SYNCHRONOUS) || (clk_src == HAL_LPTIM_CLK_EXTERNAL_ASYNCHRONOUS)))
  {
    return HAL_ERROR;
  }

  LL_LPTIM_SetClockFilter(p_lptim, LPTIM_CFGR_HAL2LL_FILTER((uint32_t)filter));

  return HAL_OK;
}

/**
  * @brief  Get the input1 filter.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @return @ref hal_lptim_filter_t Filter applied to Input1.
  */
hal_lptim_filter_t HAL_LPTIM_GetInput1Filter(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_filter_t)LPTIM_GET_CLOCK_FILTER(LPTIM_INSTANCE(hlptim));
}

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group4
  * @{
  * This section provides a set of functions to start and stop LPTIM services.
  * - Call HAL_LPTIM_Start() to start the low-power timer in polling mode.
  * - Call HAL_LPTIM_Stop() to stop the low-power timer.
  * - Call HAL_LPTIM_Start_IT() to start the low-power timer in interrupt mode.
  * - Call HAL_LPTIM_Stop_IT() to stop the low-power timer and disable interrupts.
  * - Call HAL_LPTIM_Start_DMA_Opt() to start the low-power timer in DMA mode with interrupt options.
  * - Call HAL_LPTIM_Start_DMA() to start the low-power timer in DMA mode.
  * - Call HAL_LPTIM_Stop_DMA() to stop the low-power timer in DMA mode.
  */

/**
  * @brief  Start the low-power timer in polling mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When there is a mismatch between the mode and the current clock source.
  */
hal_status_t HAL_LPTIM_Start(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hlptim, global_state, HAL_LPTIM_STATE_IDLE,
                         HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  if (LPTIM_Start(p_lptim, (uint32_t)hlptim->mode) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  LL_LPTIM_Enable(p_lptim);

  /* Start the counter in continuous or single counting mode (set the
     CNTSTRT bit or the SNGSTRT bit in CR.)
     Note that the counter starts only if TRIGEN is 00 in CFGR which is
     the case unless an external trigger source was set. */
  LL_LPTIM_StartCounter(p_lptim, (LPTIM_MODE_CR_MASK & (uint32_t)(hlptim->mode)));

  return HAL_OK;
}

/**
  * @brief  Stop the low-power timer that was started in polling mode.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_Stop(hal_lptim_handle_t *hlptim)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  status = LPTIM_CcDisable(p_lptim);

  /* Reset the WAVE bit and the TIMOUT bit in CFGR. */
  volatile uint32_t cfgr = LL_LPTIM_READ_REG(p_lptim, CFGR);
  cfgr &= ~LPTIM_MODE_CFGR_MASK;
  LL_LPTIM_WRITE_REG(p_lptim, CFGR, cfgr);

  hlptim->global_state = HAL_LPTIM_STATE_IDLE;

  return status;
}

/**
  * @brief  Start the low-power timer in interrupt mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When there is a mismatch between the mode
  *                   and the current clock source, or when enabling the
  *                   interrupts failed.
  */
hal_status_t HAL_LPTIM_Start_IT(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hlptim, global_state, HAL_LPTIM_STATE_IDLE,
                         HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  if (LPTIM_Start(p_lptim, (uint32_t)hlptim->mode) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  LL_LPTIM_Enable(p_lptim);

  uint32_t it_mask = 0U;

  if (LL_LPTIM_IsEnabledEncoderMode(p_lptim) != 0U)
  {
    it_mask |= (LL_LPTIM_DIER_UPIE | LL_LPTIM_DIER_DOWNIE);
  }
  else if (LL_LPTIM_IsEnabledTimeout(p_lptim) != 0U)
  {
    it_mask |= LL_LPTIM_DIER_CC1IE;
  }
  else
  {
    it_mask |= (LL_LPTIM_DIER_ARROKIE | LL_LPTIM_DIER_ARRMIE |
                LL_LPTIM_DIER_REPOKIE | LL_LPTIM_DIER_UEIE);
  }
  /* Check external trigger active*/
  if (LL_LPTIM_GetTriggerPolarity(p_lptim) != 0U)
  {
    it_mask |= LPTIM_DIER_EXTTRIGIE;
  }

  LL_LPTIM_EnableIT(p_lptim, it_mask);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  /* Start the counter in continuous or single counting mode (set the
     CNTSTRT bit or the SNGSTRT bit in CR.)
     Note that the counter starts only if TRIGEN is 00 in CFGR which is
     the case unless an external trigger source was set. */
  LL_LPTIM_StartCounter(p_lptim, (LPTIM_MODE_CR_MASK & (uint32_t)hlptim->mode));

  return HAL_OK;
}

/**
  * @brief  Stop the low-power timer that was started in interrupt mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  * @retval HAL_ERROR LPTIM_DIER write operation failed.
  */
hal_status_t HAL_LPTIM_Stop_IT(hal_lptim_handle_t *hlptim)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_DisableIT(p_lptim, LPTIM_DIER_INTERRUPTS_MASK);
  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  status = LPTIM_CcDisable(p_lptim);

  /* Reset the WAVE bit and the TIMOUT bit in CFGR. */
  volatile uint32_t cfgr = LL_LPTIM_READ_REG(p_lptim, CFGR);
  cfgr &= ~LPTIM_MODE_CFGR_MASK;
  LL_LPTIM_WRITE_REG(p_lptim, CFGR, cfgr);

  hlptim->global_state = HAL_LPTIM_STATE_IDLE;

  return status;
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
/**
  * @brief    Start the low-power timer in DMA mode.
  * @param    hlptim      Pointer to the handle of the LPTIM instance.
  * @param    p_data      Pointer to the data buffer.
  * @param    size_byte   Data buffer size (in bytes).
  * @param    interrupts  Selection of the DMA interrupts. \n
  *                     Can be any of the (meaningful) those values:
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_NONE
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_HT
  *                     - @ref HAL_LPTIM_OPT_DMA_IT_DEFAULT
  * @if (USE_HAL_DMA_LINKEDLIST)
  *                    - @ref HAL_LPTIM_OPT_DMA_IT_SILENT
  * @endif
  * @warning  HAL_LPTIM_SetDMA() is to be called with the correct DMA index (see
  *           hal_lptim_dma_index_t) before calling this function.
  * @retval   HAL_OK
  * @retval   HAL_ERROR           Failed to start the DMA transfer.
  * @retval   HAL_INVALID_PARAM   Input parameter is invalid
  *                               (only if USE_HAL_CHECK_PARAM == 1)
  *
  */
hal_status_t HAL_LPTIM_Start_DMA_Opt(hal_lptim_handle_t *hlptim,
                                     const uint8_t *p_data,
                                     uint32_t size_byte,
                                     uint32_t interrupts)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_LPTIM_DMA_INSTANCE(LPTIM_INSTANCE(hlptim)));

  HAL_CHECK_UPDATE_STATE(hlptim, global_state, HAL_LPTIM_STATE_IDLE,
                         HAL_LPTIM_STATE_ACTIVE);

  return LPTIM_Start_DMA_Opt(hlptim, p_data, size_byte, interrupts);
}

/**
  * @brief    Start the low-power timer in DMA mode.
  * @param    hlptim      Pointer to the handle of the LPTIM instance.
  * @param    p_data      Pointer to the data buffer.
  * @param    size_byte   Data buffer size (in bytes).
  * @warning  HAL_LPTIM_SetDMA() is to be called with the correct DMA index (see
  *           hal_lptim_dma_index_t) before calling this function.
  * @retval   HAL_OK
  * @retval   HAL_ERROR           Failed to start the DMA transfer.
  * @retval   HAL_INVALID_PARAM   Input parameter is invalid
  *                               (only if USE_HAL_CHECK_PARAM == 1)
  *
  */
hal_status_t HAL_LPTIM_Start_DMA(hal_lptim_handle_t *hlptim,
                                 const uint8_t *p_data,
                                 uint32_t size_byte)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  ASSERT_DBG_PARAM(IS_LPTIM_DMA_INSTANCE(LPTIM_INSTANCE(hlptim)));

  HAL_CHECK_UPDATE_STATE(hlptim, global_state, HAL_LPTIM_STATE_IDLE,
                         HAL_LPTIM_STATE_ACTIVE);

  return LPTIM_Start_DMA_Opt(hlptim, p_data, size_byte, HAL_LPTIM_OPT_DMA_IT_DEFAULT);
}

/**
  * @brief  Stop the timer that was started in DMA mode.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_Stop_DMA(hal_lptim_handle_t *hlptim)
{
  hal_status_t status;

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  if (LL_LPTIM_IsEnabledDMAReq_UPDATE(p_lptim) != 0U)
  {
    LL_LPTIM_DisableDMAReq_UPDATE(p_lptim);

    LPTIM_Abort_DMA(hlptim,
                    HAL_LPTIM_DMA_ID_UPDATE,
                    (IS_LPTIM_ACTIVE_SILENT(hlptim->global_state)));
  }

  LL_LPTIM_DisableDMAReq_UPDATE(p_lptim);

  status = LPTIM_CcDisable(p_lptim);

  hlptim->global_state = HAL_LPTIM_STATE_IDLE;

  return status;
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group5
  * @{
  *  This group contains the functions used to configure and control
  *    the output stage of the LP Timer's capture/compare channels.
  *
  *     - Call HAL_LPTIM_OC_SetConfigChannel() to set output channel's configuration
  *     - Call HAL_LPTIM_OC_GetConfigChannel() to get output channel's configuration
  *     - Call HAL_LPTIM_OC_SetChannelPolarity() to set output channel's polarity
  *     - Call HAL_LPTIM_OC_GetChannelPolarity() to get output channel's polarity
  *     - Call HAL_LPTIM_OC_SetChannelPulse() to set output channel's pulse width
  *     - Call HAL_LPTIM_OC_GetChannelPulse() to get output channel's pulse width
  *     - Call HAL_LPTIM_OC_StartChannel() to start output channel
  *     - Call HAL_LPTIM_OC_StopChannel() to stop output channel
  *     - Call HAL_LPTIM_OC_StartChannel_IT() to start output channel with compare IT
  *     - Call HAL_LPTIM_OC_StopChannel_IT() to stop output channel with compare IT
  */

/**
  * @brief  Configure the Output channel.
  * @param  hlptim              Pointer to the handle of the LPTIM instance.
  * @param  channel             Output channel of interest. \n
  *                               Must be one of the following values: \n
  *                               @arg @ref HAL_LPTIM_CHANNEL_1
  *                               @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  p_config            Pointer on @ref hal_lptim_oc_config_t
  * @retval HAL_OK
  * @retval HAL_ERROR           When low-power timer PWM setup fails.
  * @retval HAL_INVALID_PARAM   When p_config pointer is NULL
  *                             (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_OC_SetConfigChannel(hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel,
                                           const hal_lptim_oc_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_CHANNEL_STATE_RESET | LPTIM_CHANNEL_STATE_IDLE));

  ASSERT_DBG_PARAM(IS_LPTIM_OC_PULSE(p_config->pulse));
  ASSERT_DBG_PARAM(IS_LPTIM_OC_POLARITY(p_config->polarity));

  LL_LPTIM_OC_SetPolarity(p_lptim, (uint32_t)channel, (uint32_t)p_config->polarity);

  if (LPTIM_OC_SetPulse(p_lptim, channel, p_config->pulse) == HAL_ERROR)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_CC_SetChannelMode(p_lptim, (uint32_t)channel, LL_LPTIM_CCMODE_OUTPUT_PWM);

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of an Output Channel.
  * @param  hlptim     Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  p_config   Pointer on @ref hal_lptim_oc_config_t
  */
void HAL_LPTIM_OC_GetConfigChannel(hal_lptim_handle_t *hlptim,
                                   hal_lptim_channel_t channel,
                                   hal_lptim_oc_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_OC_CHANNEL_STATE_IDLE | HAL_LPTIM_OC_CHANNEL_STATE_ACTIVE));

  p_config->polarity = (hal_lptim_oc_polarity_t)LL_LPTIM_OC_GetPolarity(p_lptim, (uint32_t)channel);
  p_config->pulse = LL_LPTIM_OC_GetCompareValue(p_lptim, (uint32_t)channel);
}

/**
  * @brief    Set output channel's polarity.
  * @param    hlptim    Pointer to the handle of the LPTIM instance.
  * @param    channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param    polarity  Output channel's polarity
  * @warning  Ensure the channel is disabled.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_OC_SetChannelPolarity(hal_lptim_handle_t *hlptim,
                                             hal_lptim_channel_t channel,
                                             hal_lptim_oc_polarity_t polarity)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_PARAM(IS_LPTIM_OC_POLARITY(polarity));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (LPTIM_CHANNEL_STATE_IDLE));

  LL_LPTIM_OC_SetPolarity(p_lptim, (uint32_t)channel, (uint32_t)polarity);

  return HAL_OK;
}

/**
  * @brief  Get output channel's polarity.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @return hal_lptim_oc_polarity_t Output channel's polarity mode
  */
hal_lptim_oc_polarity_t HAL_LPTIM_OC_GetChannelPolarity(const hal_lptim_handle_t *hlptim,
                                                        hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_oc_polarity_t)LL_LPTIM_OC_GetPolarity(LPTIM_INSTANCE(hlptim), (uint32_t)channel);
}

/**
  * @brief  Program the pulse width of the output channel.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  pulse     Value pulse width needs to compare it.
  * @retval HAL_OK
  * @retval HAL_ERROR If pulse set failed.
  */
hal_status_t HAL_LPTIM_OC_SetChannelPulse(hal_lptim_handle_t *hlptim,
                                          hal_lptim_channel_t channel,
                                          uint32_t pulse)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_PARAM(IS_LPTIM_OC_PULSE(pulse));

  if (LPTIM_OC_SetPulse(p_lptim, channel, pulse) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  LL_LPTIM_CC_SetChannelMode(p_lptim, (uint32_t)channel, LL_LPTIM_CCMODE_OUTPUT_PWM);

  return HAL_OK;
}

/**
  * @brief  Get the pulse of the output channel.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @return Pulse value from channel number.
  */
uint32_t HAL_LPTIM_OC_GetChannelPulse(const hal_lptim_handle_t *hlptim,
                                      hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(LPTIM_INSTANCE(hlptim), channel)));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return LL_LPTIM_OC_GetCompareValue(LPTIM_INSTANCE(hlptim), (uint32_t)channel);
}

/**
  * @brief  Start a LP-Timer's output channel in polling mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_OC_StartChannel(hal_lptim_handle_t *hlptim,
                                       hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_OC_CHANNEL_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_OC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_OC_CHANNEL_STATE_ACTIVE);

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  LL_LPTIM_CC_SetChannelMode(p_lptim, (uint32_t)channel, LL_LPTIM_CCMODE_OUTPUT_PWM);

  LL_LPTIM_CC_EnableChannel(p_lptim, (uint32_t)channel);

  return HAL_OK;
}

/**
  * @brief  Stop the low-power timer output channel or output compare.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_OC_StopChannel(hal_lptim_handle_t *hlptim,
                                      hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_OC_CHANNEL_STATE_ACTIVE);

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Start the low-power timer output channel or output compare in interrupt mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_OK
  * @retval HAL_ERROR No flag has been given.
  */
hal_status_t HAL_LPTIM_OC_StartChannel_IT(hal_lptim_handle_t *hlptim,
                                          hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_OC_CHANNEL_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_OC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_OC_CHANNEL_STATE_ACTIVE);

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  /* Enable IT flag */
  uint32_t it_mask = 0U;
  it_mask |= channel_it[(uint32_t)channel].cmp;
  it_mask |= channel_it[(uint32_t)channel].cc;
  LL_LPTIM_EnableIT(p_lptim, it_mask);

  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  LL_LPTIM_CC_SetChannelMode(p_lptim, (uint32_t)channel, LL_LPTIM_CCMODE_OUTPUT_PWM);

  LL_LPTIM_CC_EnableChannel(p_lptim, (uint32_t)channel);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Stop the low-power timer output channel or output compare in interrupt mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_OK
  * @retval HAL_ERROR No flags have been given.
  */
hal_status_t HAL_LPTIM_OC_StopChannel_IT(hal_lptim_handle_t *hlptim,
                                         hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM((IS_LPTIM_OC_CHANNEL(p_lptim, channel)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_OC_CHANNEL_STATE_ACTIVE);

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  /* Disable IT flag */
  uint32_t it_mask = 0U;
  it_mask |= channel_it[(uint32_t)channel].cmp;
  it_mask |= channel_it[(uint32_t)channel].cc;
  LL_LPTIM_DisableIT(p_lptim, it_mask);

  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}


/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group6
  * @{
  *
  * This group contains the functions used to configure and control
  * the input stage of the timer's capture/compare channels.
  *    - Call HAL_LPTIM_IC_SetConfigChannel() to set input channel's configuration
  *    - Call HAL_LPTIM_IC_GetConfigChannel() to get input channel's configuration
  *    - Call HAL_LPTIM_IC_SetChannelSource() to set input channel's source
  *    - Call HAL_LPTIM_IC_GetChannelSource() to get input channel's source
  *    - Call HAL_LPTIM_IC_SetChannelPolarity() to set input channel's polarity
  *    - Call HAL_LPTIM_IC_GetChannelPolarity() to get input channel's polarity
  *    - Call HAL_LPTIM_IC_GetChannelFilter() to get input channel's filter
  *    - Call HAL_LPTIM_IC_SetChannelFilter() to set input channel's filter
  *    - Call HAL_LPTIM_IC_GetChannelPrescaler() to get input channel's prescaler
  *    - Call HAL_LPTIM_IC_SetChannelPrescaler() to set input channel's prescaler
  *    - Call HAL_LPTIM_IC_StartChannel() to start input channel
  *    - Call HAL_LPTIM_IC_StopChannel() to stop input channel
  *    - Call HAL_LPTIM_IC_StartChannel_IT() to start input channel with capture IT
  *    - Call HAL_LPTIM_IC_StopChannel_IT() to stop input channel with capture IT
  *    - Call HAL_LPTIM_IC_StartChannel_DMA() to start input channel with capture DMA
  *    - Call HAL_LPTIM_IC_StartChannel_DMA_Opt() to start input channel with capture DMA with DMA IT specific options
  *    - Call HAL_LPTIM_IC_StopChannel_DMA() to stop input channel with capture DMA
  *    - Call HAL_LPTIM_IC_ReadChannelCapturedValue() to read value captured of timer's input channel
  */

/**
  * @brief  Configure an input channel.
  * @param  hlptim              Pointer to the handle of the LPTIM instance.
  * @param  channel   Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  p_config            Pointer on @ref hal_lptim_ic_config_t.
  * @retval HAL_OK
  * @retval HAL_ERROR           When the low-power timer PWM setup failed.
  * @retval HAL_INVALID_PARAM   When p_config pointer is NULL
  *                             (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_IC_SetConfigChannel(hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel,
                                           const hal_lptim_ic_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL_SRC(p_lptim, channel, p_config->source));
  ASSERT_DBG_PARAM(IS_LPTIM_IC_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(p_config->filter));
  ASSERT_DBG_PARAM(IS_LPTIM_IC_PRESCALER(p_config->prescaler));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_CHANNEL_STATE_RESET | LPTIM_CHANNEL_STATE_IDLE));

  uint32_t ll_channelsource = LPTIM_ConvertHALToLLIcx(hlptim, channel, p_config->source);
  LL_LPTIM_SetRemap(p_lptim, ll_channelsource);

  uint32_t config = LPTIM_IC_HAL2LL_FILTER((uint32_t)p_config->filter) \
                    | (uint32_t)p_config->polarity \
                    | (uint32_t)p_config->prescaler \
                    | (uint32_t)LL_LPTIM_CCMODE_INPUTCAPTURE;
  LL_LPTIM_IC_Config(p_lptim, (uint32_t)channel, config);

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Get the configuration of an input channel.
  * @param hlptim     Pointer to the handle of the LPTIM instance.
  * @param channel    Input channel of interest to enable.
  * @param p_config   Pointer on @ref hal_lptim_ic_config_t.
  */
void HAL_LPTIM_IC_GetConfigChannel(const hal_lptim_handle_t *hlptim,
                                   hal_lptim_channel_t channel,
                                   hal_lptim_ic_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_IC_CHANNEL_STATE_IDLE | HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE));

  p_config->source = (hal_lptim_ic_src_t)LPTIM_ConvertLLToHALIcx(hlptim, channel, LL_LPTIM_GetRemap(p_lptim));

  uint32_t reg_config = LL_LPTIM_IC_GetConfig(p_lptim, (uint32_t)channel);
  p_config->polarity = (hal_lptim_ic_polarity_t)((uint32_t)(reg_config & LPTIM_CCMR1_CC1P));
  p_config->filter = (hal_lptim_filter_t)LPTIM_IC_LL2HAL_FILTER(reg_config & LPTIM_CCMR1_IC1F);
  p_config->prescaler = (hal_lptim_ic_prescaler_t)((uint32_t)((reg_config & LPTIM_CCMR1_IC1PSC)));
}

/**
  * @brief Set input channel's source.
  *
  * @param    hlptim    Pointer to the handle of the LPTIM instance.
  * @param    channel   Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param    source    Input source for the channel.
  * @warning  Ensure the channel is disabled.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_IC_SetChannelSource(const hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel,
                                           hal_lptim_ic_src_t source)
{
  STM32_UNUSED(channel);

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_IC_CHANNEL_STATE_IDLE | HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE));


  uint32_t ll_channelsource = LPTIM_ConvertHALToLLIcx(hlptim, channel, source);
  LL_LPTIM_SetRemap(p_lptim, ll_channelsource);

  return HAL_OK;
}

/**
  * @brief    Get the source of a input channel.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @warning  Ensure the channel is disabled.
  * @retval   hal_lptim_ic_src_t channel input sources
  */
hal_lptim_ic_src_t HAL_LPTIM_IC_GetChannelSource(const hal_lptim_handle_t *hlptim,
                                                 hal_lptim_channel_t channel)
{
  STM32_UNUSED(channel);

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (HAL_LPTIM_IC_CHANNEL_STATE_IDLE | HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE));

  return LPTIM_ConvertLLToHALIcx(hlptim, channel, LL_LPTIM_GetRemap(p_lptim));
}

/**
  * @brief Set input channel's polarity.
  *
  * @param hlptim   Pointer to the handle of the LPTIM instance.
  * @param channel  Input channel of interest. \n
  *                 Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @param polarity input channel's polarity.
  * @warning Ensure the channel is disabled.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_IC_SetChannelPolarity(const hal_lptim_handle_t *hlptim,
                                             hal_lptim_channel_t channel,
                                             hal_lptim_ic_polarity_t polarity)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_INIT);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(LPTIM_INSTANCE(hlptim)));

  ASSERT_DBG_PARAM(IS_LPTIM_IC_POLARITY(polarity));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (LPTIM_CHANNEL_STATE_IDLE));

  LL_LPTIM_IC_SetPolarity(LPTIM_INSTANCE(hlptim), (uint32_t)channel, (uint32_t)polarity);

  return HAL_OK;
}

/**
  * @brief    Get input channel's polarity.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @return   hal_lptim_ic_polarity_t The input polarity for the channel
  */
hal_lptim_ic_polarity_t HAL_LPTIM_IC_GetChannelPolarity(const hal_lptim_handle_t *hlptim,
                                                        hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_ic_polarity_t)LL_LPTIM_IC_GetPolarity(LPTIM_INSTANCE(hlptim),
                                                          (uint32_t)channel);
}

/**
  * @brief    Set input channel's filter.
  * @param    hlptim Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @param    filter  Input filter for the channel.
  * @warning  Ensure the channel is disabled.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_IC_SetChannelFilter(const hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel,
                                           hal_lptim_filter_t filter)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_INIT);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(filter));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (LPTIM_CHANNEL_STATE_IDLE));

  LL_LPTIM_IC_SetFilter(p_lptim, (uint32_t)channel, LPTIM_IC_HAL2LL_FILTER((uint32_t)filter));

  return HAL_OK;
}

/**
  * @brief    Get input channel's filter.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @return   hal_lptim_filter_t input filter for the channel.
  */
hal_lptim_filter_t HAL_LPTIM_IC_GetChannelFilter(const hal_lptim_handle_t *hlptim,
                                                 hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_filter_t)LPTIM_IC_LL2HAL_FILTER(LL_LPTIM_IC_GetFilter(LPTIM_INSTANCE(hlptim), (uint32_t)channel));
}

/**
  * @brief    Configure the prescaler of an input channel.
  * @param    hlptim Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @param    prescaler Input prescaler for the channel.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_IC_SetChannelPrescaler(const hal_lptim_handle_t *hlptim,
                                              hal_lptim_channel_t channel,
                                              hal_lptim_ic_prescaler_t prescaler)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_PARAM(IS_LPTIM_IC_PRESCALER(prescaler));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   (LPTIM_CHANNEL_STATE_IDLE));

  LL_LPTIM_IC_SetPrescaler(p_lptim, (uint32_t)channel, (uint32_t)prescaler);

  return HAL_OK;
}

/**
  * @brief    Get the prescaler of a input channel.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @return   hal_lptim_ic_prescaler_t The input prescaler for the channel.
  */
hal_lptim_ic_prescaler_t HAL_LPTIM_IC_GetChannelPrescaler(const hal_lptim_handle_t *hlptim,
                                                          hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_ic_prescaler_t)LL_LPTIM_IC_GetPrescaler(LPTIM_INSTANCE(hlptim), (uint32_t)channel);
}

/**
  * @brief    Start a low-power timer input channel in polling mode.
  * @param    hlptim     Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_IC_StartChannel(hal_lptim_handle_t *hlptim,
                                       hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_IC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  LL_LPTIM_CC_EnableChannel(p_lptim, (uint32_t)channel);

  return HAL_OK;
}

/**
  * @brief    Stop the low-power timer input channel.
  * @param    hlptim Pointer to the handle of the LPTIM instance.
  * @param    channel    Input channel of interest. \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_IC_StopChannel(hal_lptim_handle_t *hlptim,
                                      hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief Start the low-power timer input channel in interrupt mode.
  *
  * @param hlptim Pointer to the handle of the LPTIM instance.
  * @param    channel   Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_ERROR No flag has been given
  * @retval HAL_OK input channel has been correctly started
  */
hal_status_t HAL_LPTIM_IC_StartChannel_IT(hal_lptim_handle_t *hlptim,
                                          hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_IC_CHANNEL_STATE_IDLE);
  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_IC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_EnableIT(p_lptim, channel_it[(uint32_t)channel].cc);

  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  LL_LPTIM_CC_SetChannelMode(p_lptim, (uint32_t)channel, LL_LPTIM_CCMODE_INPUTCAPTURE);

  LL_LPTIM_CC_EnableChannel(p_lptim, (uint32_t)channel);

  return HAL_OK;
}

/**
  * @brief  Stop the low-power timer input channel in interrupt mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_ERROR No flags have been given.
  * @retval HAL_OK input channel has been correctly stopped
  */
hal_status_t HAL_LPTIM_IC_StopChannel_IT(hal_lptim_handle_t *hlptim,
                                         hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel], HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  /* Enable LPTIM if it was disabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Enable(p_lptim);

  LL_LPTIM_DisableIT(p_lptim, channel_it[(uint32_t)channel].cc);

  if (LPTIM_WaitFlag(p_lptim, LL_LPTIM_IsActiveFlag_DIEROK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  LL_LPTIM_ClearFlag_DIEROK(p_lptim);

  if (is_lptim_enabled == 0U)
  {
    LL_LPTIM_Disable(p_lptim);
  }

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Start the low-power timer input channel in DMA mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_LPTIM_CHANNEL_1
  *                     @arg @ref HAL_LPTIM_CHANNEL_2
  * @param p_data     Pointer to the data buffer
  * @param size_byte  Data buffer size
  * @retval HAL_ERROR No flag has been given
  * @retval HAL_OK
  */
#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
hal_status_t HAL_LPTIM_IC_StartChannel_DMA(hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel,
                                           uint8_t *p_data,
                                           uint32_t size_byte)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(LPTIM_INSTANCE(hlptim)));

  ASSERT_DBG_PARAM((IS_LPTIM_DMA_INSTANCE(LPTIM_INSTANCE(hlptim))));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_IC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  return LPTIM_IC_StartChannel_DMA_Opt(hlptim, channel, p_data, size_byte, HAL_LPTIM_OPT_DMA_IT_DEFAULT);
}

/**
  * @brief  Start the low-power timer input channel in DMA mode.
  * @param  hlptim Pointer to the handle of the LPTIM instance.
  * @param  channel    Input channel of interest. \n
  *                    Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @param  p_data pointer to the data buffer
  * @param  size_byte data buffer size
  * @param  interrupts Selection of the DMA interrupts.
  *                    Can be any of the (meaningful) those values:
  *                    - @ref HAL_LPTIM_OPT_DMA_IT_NONE
  *                    - @ref HAL_LPTIM_OPT_DMA_IT_HT
  *                    - @ref HAL_LPTIM_OPT_DMA_IT_DEFAULT
  * @if (USE_HAL_DMA_LINKEDLIST)
  *                    - @ref HAL_LPTIM_OPT_DMA_IT_SILENT
  * @endif
  * @retval HAL_OK
  * @retval HAL_ERROR No flag has been given
  */
hal_status_t HAL_LPTIM_IC_StartChannel_DMA_Opt(hal_lptim_handle_t *hlptim,
                                               hal_lptim_channel_t channel,
                                               uint8_t *p_data,
                                               uint32_t size_byte,
                                               uint32_t interrupts)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(LPTIM_INSTANCE(hlptim)));

  ASSERT_DBG_PARAM(IS_LPTIM_DMA_INSTANCE(LPTIM_INSTANCE(hlptim)));

  ASSERT_DBG_STATE(hlptim->channel_states[(uint32_t)channel],
                   HAL_LPTIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(hlptim, channel_states[(uint32_t)channel],
                         HAL_LPTIM_IC_CHANNEL_STATE_IDLE,
                         HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  return LPTIM_IC_StartChannel_DMA_Opt(hlptim, channel, p_data, size_byte, interrupts);
}

/**
  * @brief  Stop a timer's input channel that was started in DMA mode.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel    Input channel of interest. \n
  *                    Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_IC_StopChannel_DMA(hal_lptim_handle_t *hlptim,
                                          hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT_CAPTURE_INSTANCE(p_lptim));

  ASSERT_DBG_PARAM(IS_LPTIM_DMA_INSTANCE(p_lptim));

  hal_lptim_channel_state_t channel_state = hlptim->channel_states[(uint32_t)channel];

  ASSERT_DBG_STATE(channel_state,
                   HAL_LPTIM_IC_CHANNEL_STATE_ACTIVE);

  LPTIM_IC_StopChannel_DMA(hlptim, channel,
                           (IS_LPTIM_ACTIVE_SILENT(channel_state)));

  LL_LPTIM_CC_DisableChannel(p_lptim, (uint32_t)channel);

  hlptim->channel_states[(uint32_t)channel] = HAL_LPTIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief  Read the captured value for a low-power timer input channel.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Input channel of interest. \n
  *                   Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @retval uint32_t  Captured value
  */
uint32_t HAL_LPTIM_IC_ReadChannelCapturedValue(const hal_lptim_handle_t *hlptim,
                                               hal_lptim_channel_t channel)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  return LL_LPTIM_IC_GetCapturedValue(p_lptim, (uint32_t)channel);
}

/**
  * @brief  Get the input capture glitch filter latency.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  channel   Input channel of interest. \n
  *                   Must be one of the following values: \n
  *                      @arg @ref HAL_LPTIM_CHANNEL_1
  *                      @arg @ref HAL_LPTIM_CHANNEL_2
  * @note  The LPTIM glitch filter and prescaler cause a latency offset in captured values.
  * @note  The real capture value corresponding to the input capture trigger can be calculated using the below formula:
  *        Real capture value = captured(LPTIM_CCRx) - filter latency
  * @retval uint8_t  Glitch filter latency in counter step unit
  */
uint8_t HAL_LPTIM_IC_GetChannelFilterLatency(const hal_lptim_handle_t *hlptim,
                                             hal_lptim_channel_t channel)
{
  uint8_t filter_latency;

  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_CHANNEL(channel));

  filter_latency = LL_LPTIM_IC_GetOffset(p_lptim, (uint32_t) channel);

  return filter_latency;
}

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group7
  *  @{
  *  This group contains the functions used to configure the encoder stage of the timer.
  *     - Call HAL_LPTIM_SetConfigEncoder() to set the config for the encoder.
  *     - Call HAL_LPTIM_GetConfigEncoder() to get the config for the encoder.
  */

/**
  * @brief    Configure the encoder interface.
  * @param    hlptim    Pointer to the handle of the LPTIM instance.
  * @param    p_encoder Pointer to the encoder configuration structure.
  * @warning  The signals frequency on both input1 and input2 must not exceed the LPTIM internal clock frequency divided
  *           by 4.
  * @retval   HAL_OK
  */
hal_status_t HAL_LPTIM_SetConfigEncoder(const hal_lptim_handle_t *hlptim,
                                        const hal_lptim_encoder_config_t *p_encoder)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_encoder != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(p_encoder->filter));

  ASSERT_DBG_PARAM(IS_LPTIM_ENCODER_INTERFACE_INSTANCE(p_lptim));

  ASSERT_DBG_PARAM(IS_LPTIM_INPUT1_SRC(p_encoder->input1));
  ASSERT_DBG_PARAM(IS_LPTIM_INPUT2_SRC(p_encoder->input2));

  /* The signals frequency on both Input1 and Input2 inputs must not exceed the LPTIM internal
    clock frequency divided by 4.*/
  ASSERT_DBG_PARAM((p_encoder->filter <= HAL_LPTIM_FDIV1_N4));

  LL_LPTIM_SetInput1Source(p_lptim, (uint32_t)p_encoder->input1);
  LL_LPTIM_SetInput2Source(p_lptim, (uint32_t)p_encoder->input2);
  LL_LPTIM_SetClockFilter(p_lptim, LPTIM_CFGR_HAL2LL_FILTER((uint32_t)p_encoder->filter));

  /* Disable LPTIM if it was enabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Disable(p_lptim);

  LL_LPTIM_EnableEncoderMode(p_lptim);

  if (is_lptim_enabled != 0U)
  {
    LL_LPTIM_Enable(p_lptim);
  }
  return HAL_OK;
}

/**
  * @brief Get the configuration of the low-power timer encoder interface.
  * @param hlptim     Pointer to the handle of the LPTIM instance.
  * @param p_encoder  Pointer to the encoder configuration structure.
  */
void HAL_LPTIM_GetConfigEncoder(const hal_lptim_handle_t *hlptim,
                                hal_lptim_encoder_config_t *p_encoder)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_encoder != NULL));

  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE);

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  p_encoder->input1 = (hal_lptim_input1_src_t)LL_LPTIM_GetInput1Source(p_lptim);
  p_encoder->input2 = (hal_lptim_input2_src_t)LL_LPTIM_GetInput2Source(p_lptim);
  p_encoder->filter = (hal_lptim_filter_t)(LPTIM_GET_CLOCK_FILTER(p_lptim));
}

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group8
  *  @{
  *  This section provides a set of function to configure external trigger
  *      - Call HAL_LPTIM_SetConfigExtTrigInput() Set External trigger input
  *      - Call HAL_LPTIM_GetConfigExtTrigInput() Get Configuration external trigger input
  *      - Call HAL_LPTIM_SetExtTrigInputSource() Set external trigger input Source
  *      - Call HAL_LPTIM_GetExtTrigInputSource() Get External trigger input Source
  *      - Call HAL_LPTIM_SetExtTrigInputPolarity() Set External trigger input Polarity
  *      - Call HAL_LPTIM_GetExtTrigInputPolarity() Get external trigger input Polarity
  *      - Call HAL_LPTIM_SetExtTrigInputFilter() Set External trigger input Filter
  *      - Call HAL_LPTIM_GetExtTrigInputFilter() Get external trigger input Filter
  */

/**
  * @brief  Configure External Trigger input.
  * @param  hlptim    Pointer to the handle of the LPTIM instance.
  * @param  p_config  Pointer to an external trigger input configuration structure.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_SetConfigExtTrigInput(const hal_lptim_handle_t *hlptim,
                                             const hal_lptim_ext_trig_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_EXT_TRIG_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(p_config->filter));
  ASSERT_DBG_PARAM(IS_LPTIM_EXT_TRIG_SRC(p_lptim, p_config->source));

  /* Disable LPTIM if it was enabled when entering this function. */
  uint32_t is_lptim_enabled = LL_LPTIM_IsEnabled(p_lptim);
  LL_LPTIM_Disable(p_lptim);

  uint32_t trig_src = LPTIM_ConvertHALToLLExttrig(hlptim, p_config->source);
  LL_LPTIM_ConfigTrigger(p_lptim,
                         (uint32_t)trig_src,
                         (uint32_t)(p_config->filter) << LPTIM_CFGR_TRGFLT_Pos,
                         (uint32_t)p_config->polarity);

  if (is_lptim_enabled != 0U)
  {
    LL_LPTIM_Enable(p_lptim);
  }

  return HAL_OK;
}

/**
  * @brief  Get the External Trigger input configuration.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @param  p_config Pointer to an external trigger input configuration structure.
  */
void HAL_LPTIM_GetConfigExtTrigInput(const hal_lptim_handle_t *hlptim,
                                     hal_lptim_ext_trig_config_t *p_config)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  volatile uint32_t cfgr = LL_LPTIM_READ_REG(LPTIM_INSTANCE(hlptim), CFGR);

  p_config->polarity = (hal_lptim_ext_trig_polarity_t)LPTIM_GET_ETR_POLARITY(cfgr);
  p_config->filter = (hal_lptim_filter_t)(uint32_t)(LPTIM_GET_ETR_FILTER(cfgr) >> LPTIM_CFGR_TRGFLT_Pos);
  p_config->source = (hal_lptim_ext_trig_src_t)LPTIM_ConvertLLToHALExttrig(hlptim, LPTIM_GET_ETR_SOURCE(cfgr));
}

/**
  * @brief  Set the External Trigger input source.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  source  Source selection for the external trigger.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_SetExtTrigInputSource(const hal_lptim_handle_t *hlptim,
                                             hal_lptim_ext_trig_src_t source)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_EXT_TRIG_SRC(p_lptim, source));

  uint32_t trig_src = LPTIM_ConvertHALToLLExttrig(hlptim, source);

  LL_LPTIM_SetTriggerSource(p_lptim, (uint32_t)trig_src);

  return HAL_OK;
}

/**
  * @brief  Get the External Trigger input source.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_ext_trig_src_t Source selected for the external trigger.
  */
hal_lptim_ext_trig_src_t HAL_LPTIM_GetExtTrigInputSource(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  uint32_t trig_src = LL_LPTIM_GetTriggerSource(LPTIM_INSTANCE(hlptim));

  return (LPTIM_ConvertLLToHALExttrig(hlptim, trig_src));
}

/**
  * @brief  Set the External Trigger input polarity.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @param  polarity Polarity of the ETR input.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_SetExtTrigInputPolarity(const hal_lptim_handle_t *hlptim,
                                               hal_lptim_ext_trig_polarity_t polarity)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_EXT_TRIG_POLARITY(polarity));

  LL_LPTIM_SetTriggerPolarity(p_lptim, (uint32_t)polarity);

  return HAL_OK;
}

/**
  * @brief  Get the External Trigger input polarity.
  * @param  hlptim   Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_ext_trig_polarity_t External trigger polarity.
  */
hal_lptim_ext_trig_polarity_t HAL_LPTIM_GetExtTrigInputPolarity(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return ((hal_lptim_ext_trig_polarity_t)
          LL_LPTIM_GetTriggerPolarity(LPTIM_INSTANCE(hlptim)));
}

/**
  * @brief  Set the External Trigger input filter.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  filter  Filter the external trigger input.
  * @retval HAL_OK
  */
hal_status_t HAL_LPTIM_SetExtTrigInputFilter(const hal_lptim_handle_t *hlptim,
                                             hal_lptim_filter_t filter)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  ASSERT_DBG_PARAM(IS_LPTIM_FILTER(filter));

  LL_LPTIM_SetTriggerFilter(p_lptim, (uint32_t)(filter) << LPTIM_CFGR_TRGFLT_Pos);

  return HAL_OK;
}

/**
  * @brief  Get the External Trigger input filter.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @retval hal_lptim_filter_t Filter for the external trigger input.
  */
hal_lptim_filter_t HAL_LPTIM_GetExtTrigInputFilter(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  ASSERT_DBG_STATE(hlptim->global_state,
                   (HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE));

  return (hal_lptim_filter_t)(uint32_t)(LL_LPTIM_GetTriggerFilter(LPTIM_INSTANCE(hlptim)) >> LPTIM_CFGR_TRGFLT_Pos);
}

/**
  *  @}
  */

/** @addtogroup LPTIM_Exported_Functions_Group9 Low-power timer IRQ handler and callback functions
  *
  *  This section provides LPTIM IRQ Handler and callback function called within the IRQ Handler

  *     IRQ Handler:
       - Call HAL_LPTIM_IRQHandler() LPTIM interrupt global request handler
       - Call HAL_LPTIM_UPD_IRQHandler() LPTIM interrupt Update request handler
       - Call HAL_LPTIM_CC_IRQHandler() LPTIM interrupt Capture/Compare request handler
       - Call HAL_LPTIM_TRGI_DIR_IRQHandler() LPTIM interrupt Trigger Direction request handler

        Weak Callback:
       - Call HAL_LPTIM_ErrorCallback() Error Callback
       - Call HAL_LPTIM_StopCallback() Stop Callback
       - Call HAL_LPTIM_InputCaptureStopCallback() Channel Stop Callback
       - Call HAL_LPTIM_UpdateCallback() Update Callback
       - Call HAL_LPTIM_UpdateHalfCpltCallback() Update Half Cplt Callback
       - Call HAL_LPTIM_RepUpdateCallback() Rep Update Callback
       - Call HAL_LPTIM_TriggerCallback() Trigger Callback
       - Call HAL_LPTIM_InputCaptureCallback() Input Capture Callback
       - Call HAL_LPTIM_InputCaptureHalfCpltCallback() Input Capture Half Cplt Callback
       - Call HAL_LPTIM_InputOverCaptureCallback() Input Over Capture Callback
       - Call HAL_LPTIM_CompareMatchCallback() Compare Match Callback
       - Call HAL_LPTIM_CompareMatchHalfCpltCallback() Compare Match Half Cplt Callback
       - Call HAL_LPTIM_CompareUpdateCallback() Compare Update Callback
       - Call HAL_LPTIM_AutoReloadMatchCallback() AutoReload Match Callback
       - Call HAL_LPTIM_AutoReloadUpdateCallback() AutoReload Update Callback
       - Call HAL_LPTIM_DirectionUpCallback() Direction Up Callback
       - Call HAL_LPTIM_DirectionDownCallback() Direction Down Callback

       And there register callback:
       - Call HAL_LPTIM_RegisterErrorCallback() Error Callback
       - Call HAL_LPTIM_RegisterStopCallback() Stop Callback
       - Call HAL_LPTIM_RegisterChannelStopCallback() Stop Callback
       - Call HAL_LPTIM_RegisterUpdateCallback() Update Callback
       - Call HAL_LPTIM_RegisterUpdateHalfCpltCallback() Update Half Cplt Callback
       - Call HAL_LPTIM_RegisterRepUpdateCallback() Rep Update Callback
       - Call HAL_LPTIM_RegisterTriggerCallback() Trigger Callback
       - Call HAL_LPTIM_RegisterInputCaptureCallback() Input Capture Callback
       - Call HAL_LPTIM_RegisterInputCaptureHalfCpltCallback() Input Capture Half Cplt Callback
       - Call HAL_LPTIM_RegisterOverCaptureCallback() Input Over Capture Callback
       - Call HAL_LPTIM_RegisterCompareMatchCallback() Compare Match Callback
       - Call HAL_LPTIM_RegisterCompareUpdateCallback() Compare Update Callback
       - Call HAL_LPTIM_RegisterAutoReloadMatchCallback() AutoReload Match Callback
       - Call HAL_LPTIM_RegisterAutoReloadUpdateCallback() AutoReload Update Callback
       - Call HAL_LPTIM_RegisterDirectionUpCallback() Direction Up Callback
       - Call HAL_LPTIM_RegisterDirectionDownCallback() Direction Down Callback
  *  @{
  */

/**
  * @brief This function handles LPTIM generic interrupts requests.
  * @param hlptim  Pointer to the handle of the LPTIM instance.
  * @note  Handle all low-power timer interrupt requests.
  */
void HAL_LPTIM_IRQHandler(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  const uint32_t flag_status = LL_LPTIM_READ_REG(p_lptim, ISR);
  const uint32_t it_sources = LL_LPTIM_READ_REG(p_lptim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources;

  /* Capture compare 1 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC1IF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC1(p_lptim);
    /* input capture catching an event in*/
    if (LL_LPTIM_CC_GetChannelMode(p_lptim, (uint32_t)HAL_LPTIM_CHANNEL_1) == LL_LPTIM_CCMODE_INPUTCAPTURE)
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->input_capture_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
      HAL_LPTIM_InputCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
    else
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->compare_match_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
      HAL_LPTIM_CompareMatchCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
  }
  /* Capture compare 2 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC2IF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC2(p_lptim);
    if (LL_LPTIM_CC_GetChannelMode(p_lptim, (uint32_t)HAL_LPTIM_CHANNEL_2) == LL_LPTIM_CCMODE_INPUTCAPTURE)
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->input_capture_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
      HAL_LPTIM_InputCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
    else
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->compare_match_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
      HAL_LPTIM_CompareMatchCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
  }
  /* Compare update interrupt Channel 1*/
  if ((flag_status_masked & LL_LPTIM_ISR_CMP1OK) != 0U)
  {
    LL_LPTIM_ClearFlag_CMP1OK(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->compare_update_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
    HAL_LPTIM_CompareUpdateCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Compare update interrupt Channel 2*/
  if ((flag_status_masked & LL_LPTIM_ISR_CMP2OK) != 0U)
  {
    LL_LPTIM_ClearFlag_CMP2OK(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->compare_update_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
    HAL_LPTIM_CompareUpdateCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Over capture 1 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC1OF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC1O(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->input_over_capture_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
    HAL_LPTIM_InputOverCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Over capture 2 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC2OF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC2O(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->input_over_capture_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
    HAL_LPTIM_InputOverCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Autoreload matched interrupt */
  if ((flag_status_masked & LL_LPTIM_ISR_ARRM) != 0U)
  {
    LL_LPTIM_ClearFlag_ARRM(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->auto_reload_match_callback(hlptim);
#else
    HAL_LPTIM_AutoReloadMatchCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Autoreload update interrupt */
  if ((flag_status_masked & LL_LPTIM_ISR_ARROK) != 0U)
  {
    LL_LPTIM_ClearFlag_ARROK(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->auto_reload_update_callback(hlptim);
#else
    HAL_LPTIM_AutoReloadUpdateCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Trigger detected interrupt */
  if ((flag_status_masked & LL_LPTIM_ISR_EXTTRIG) != 0U)
  {
    LL_LPTIM_ClearFlag_EXTTRIG(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->trigger_callback(hlptim);
#else
    HAL_LPTIM_TriggerCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Direction counter changed from up to down */
  if ((flag_status_masked & LL_LPTIM_ISR_DOWN) != 0U)
  {
    LL_LPTIM_ClearFlag_DOWN(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->direction_down_callback(hlptim);
#else
    HAL_LPTIM_DirectionDownCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Direction counter changed from down to up */
  if ((flag_status_masked & LL_LPTIM_ISR_UP) != 0U)
  {
    LL_LPTIM_ClearFlag_UP(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->direction_up_callback(hlptim);
#else
    HAL_LPTIM_DirectionUpCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Repetition counter underflowed or == 0 and LPTIM counter overflow */
  if ((flag_status_masked & LL_LPTIM_ISR_UE) != 0U)
  {
    LL_LPTIM_ClearFlag_UE(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->update_callback(hlptim);
#else
    HAL_LPTIM_UpdateCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Successful APB bus write to repetition counter register */
  if ((flag_status_masked & LL_LPTIM_ISR_REPOK) != 0U)
  {
    LL_LPTIM_ClearFlag_REPOK(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->rep_update_callback(hlptim);
#else
    HAL_LPTIM_RepUpdateCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Low-power timer capture/compare handler.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  */
void HAL_LPTIM_CC_IRQHandler(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  const uint32_t flag_status = LL_LPTIM_READ_REG(p_lptim, ISR);
  const uint32_t it_sources = LL_LPTIM_READ_REG(p_lptim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources;

  /* Capture compare 1 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC1IF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC1(p_lptim);
    /* input capture catching an event in*/
    if (LL_LPTIM_CC_GetChannelMode(p_lptim, (uint32_t)HAL_LPTIM_CHANNEL_1) == LL_LPTIM_CCMODE_INPUTCAPTURE)
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->input_capture_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
      HAL_LPTIM_InputCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
    else
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->compare_match_callback(hlptim, HAL_LPTIM_CHANNEL_1);
#else
      HAL_LPTIM_CompareMatchCallback(hlptim, HAL_LPTIM_CHANNEL_1);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
  }
  /* Capture compare 2 interrupt caught */
  if ((flag_status_masked & LL_LPTIM_ISR_CC2IF) != 0U)
  {
    LL_LPTIM_ClearFlag_CC2(p_lptim);
    if (LL_LPTIM_CC_GetChannelMode(p_lptim, (uint32_t)HAL_LPTIM_CHANNEL_2) == LL_LPTIM_CCMODE_INPUTCAPTURE)
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->input_capture_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
      HAL_LPTIM_InputCaptureCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
    else
    {
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
      hlptim->compare_match_callback(hlptim, HAL_LPTIM_CHANNEL_2);
#else
      HAL_LPTIM_CompareMatchCallback(hlptim, HAL_LPTIM_CHANNEL_2);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  Low-power timer update handler.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  */
void HAL_LPTIM_UPD_IRQHandler(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  const uint32_t flag_status = LL_LPTIM_READ_REG(p_lptim, ISR);
  const uint32_t it_sources = LL_LPTIM_READ_REG(p_lptim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources;

  /* Repetition counter underflowed or == 0 and LPTIM counter overflow */
  if ((flag_status_masked & LL_LPTIM_ISR_UE) != 0U)
  {
    LL_LPTIM_ClearFlag_UE(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->update_callback(hlptim);
#else
    HAL_LPTIM_UpdateCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Low-power timer trigger input handler.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  */
void HAL_LPTIM_TRGI_DIR_IRQHandler(hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  lptim_t *p_lptim = LPTIM_INSTANCE(hlptim);

  const uint32_t flag_status = LL_LPTIM_READ_REG(p_lptim, ISR);
  const uint32_t it_sources = LL_LPTIM_READ_REG(p_lptim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources;

  /* Direction counter changed from up to down */
  if ((flag_status_masked & LL_LPTIM_ISR_DOWN) != 0U)
  {
    LL_LPTIM_ClearFlag_DOWN(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->direction_down_callback(hlptim);
#else
    HAL_LPTIM_DirectionDownCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Direction counter changed from down to up */
  if ((flag_status_masked & LL_LPTIM_ISR_UP) != 0U)
  {
    LL_LPTIM_ClearFlag_UP(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->direction_up_callback(hlptim);
#else
    HAL_LPTIM_DirectionUpCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
  /* Trigger detected interrupt */
  if ((flag_status_masked & LL_LPTIM_ISR_EXTTRIG) != 0U)
  {
    LL_LPTIM_ClearFlag_EXTTRIG(p_lptim);
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
    hlptim->trigger_callback(hlptim);
#else
    HAL_LPTIM_TriggerCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
  }
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
/**
  * @brief    Update Half Complete callback.\n
  *           Function called when the DMA transfer triggered by the timer update
  *           DMA request is half completed.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_UpdateHalfCpltCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    DMA Error callback. \n
  *           This function is called in case of a DMA transfer error.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_ErrorCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    DMA Stop callback. \n
  *           This function is called after stopping a DMA transfer triggered
  *           by the timer update event.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_StopCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    DMA Channel Stop callback. \n
  *           This function is called after stopping a DMA transfer triggered
  *           by a capture/compare event.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel Input Channel of interest.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_InputCaptureStopCallback(hal_lptim_handle_t *hlptim,
                                               hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief    Update callback. \n
  *           Function called when the timer update interrupt is generated or when
  *           the DMA transfer triggered by the timer update DMA request is completed.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_UpdateCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Repetition Update Callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_RepUpdateCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Trigger callback. \n
  *           Function called when the timer trigger interrupt is generated or when
  *           the DMA transfer triggered by the timer trigger DMA request is completed.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_TriggerCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Input Capture Callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel    input channel of interest to enable.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_InputCaptureCallback(hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
/**
  * @brief    Callback for the DMA Half Complete transfer triggered by an Input Capture event.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel    input channel of interest to enable.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_InputCaptureHalfCpltCallback(hal_lptim_handle_t *hlptim,
                                                   hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief    Input Over Capture callback. \n
  *           Function called when an input over capture interrupt is generated.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel    input channel of interest to enable.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_InputOverCaptureCallback(hal_lptim_handle_t *hlptim,
                                               hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}

/**
  * @brief    Compare match Callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel    Output channel of interest to enable.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_CompareMatchCallback(hal_lptim_handle_t *hlptim,
                                           hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}

/**
  * @brief    Compare update Callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @param    channel    Output channel of interest to enable.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_CompareUpdateCallback(hal_lptim_handle_t *hlptim,
                                            hal_lptim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
  STM32_UNUSED(channel);
}

/**
  * @brief    AutoReload Match callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_AutoReloadMatchCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Autoreload Update callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_AutoReloadUpdateCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Direction UP callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_DirectionUpCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/**
  * @brief    Direction Down callback.
  * @param    hlptim  Pointer to the handle of the LPTIM instance.
  * @warning  This weak function must not be modified. When the callback is needed, it is overridden in the user file.
  */
__WEAK void HAL_LPTIM_DirectionDownCallback(hal_lptim_handle_t *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(hlptim);
}

/* Interfaces for registering callbacks ***************************************/
#if defined(USE_HAL_LPTIM_REGISTER_CALLBACKS) && (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)

/**
  * @brief  Callback registration for the DMA Error.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterErrorCallback(hal_lptim_handle_t *hlptim,
                                             hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->error_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Stop callback.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterStopCallback(hal_lptim_handle_t *hlptim,
                                            hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->stop_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Channel Stop callback.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterChannelStopCallback(hal_lptim_handle_t *hlptim,
                                                   hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->input_capture_stop_callback = fct;

  return HAL_OK;
}

#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief  Callback registration for the Update Event.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterUpdateCallback(hal_lptim_handle_t *hlptim,
                                              hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->update_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)

/**
  * @brief  Callback registration for the DMA Half Complete transfer
  *            triggered on Update event.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterUpdateHalfCpltCallback(hal_lptim_handle_t *hlptim,
                                                      hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->update_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief  Callback registration for Repetition Update.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterRepUpdateCallback(hal_lptim_handle_t *hlptim,
                                                 hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->rep_update_callback = fct;

  return HAL_OK;
}
/**
  * @brief  Callback registration for the Trigger event.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterTriggerCallback(hal_lptim_handle_t *hlptim,
                                               hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->trigger_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Input Capture event.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterInputCaptureCallback(hal_lptim_handle_t *hlptim,
                                                    hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->input_capture_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_LPTIM_DMA) && (USE_HAL_LPTIM_DMA == 1)
/**
  * @brief  Callback registration for the Input Capture Half Complete.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterInputCaptureHalfCpltCallback(hal_lptim_handle_t *hlptim,
                                                            hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->input_capture_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_LPTIM_DMA */

/**
  * @brief  Callback registration for Over Capture.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterOverCaptureCallback(hal_lptim_handle_t *hlptim,
                                                   hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->input_over_capture_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Compare Match.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterCompareMatchCallback(hal_lptim_handle_t *hlptim,
                                                    hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->compare_match_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Compare Update.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterCompareUpdateCallback(hal_lptim_handle_t *hlptim,
                                                     hal_lptim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->compare_update_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Autoreload Update.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterAutoReloadUpdateCallback(hal_lptim_handle_t *hlptim,
                                                        hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->auto_reload_update_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Autoreload Match.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterAutoReloadMatchCallback(hal_lptim_handle_t *hlptim,
                                                       hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->auto_reload_match_callback = fct;

  return HAL_OK;
}
/**
  * @brief  Callback registration for the Direction UP changes.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterDirectionUpCallback(hal_lptim_handle_t *hlptim,
                                                   hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->direction_up_callback = fct;

  return HAL_OK;
}
/**
  * @brief  Callback registration for the Direction Down changes.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK Register correctly setup
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_LPTIM_RegisterDirectionDownCallback(hal_lptim_handle_t *hlptim,
                                                     hal_lptim_cb_t fct)
{
  ASSERT_DBG_PARAM((hlptim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hlptim->direction_down_callback = fct;

  return HAL_OK;
}

#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

/**
  *  @}
  */

#if defined(USE_HAL_LPTIM_USER_DATA) && (USE_HAL_LPTIM_USER_DATA == 1)
/** @addtogroup LPTIM_Exported_Functions_Group10
  *  The user data pointer, *p_user_data, in the HAL LPTIM handle allows user to associate applicative user
  *  data to the HAL LPTIM handle.
  *  The two functions in this group give an application the
  *  possibility to store and retrieve user data pointer into and
  *  from the handle.
  *  - Call HAL_LPTIM_SetUserData() Set user data
  *  - Call HAL_LPTIM_GetUserData() Get user data
  *  @{
  */

/**
  * @brief  Store User Data pointer into the handle.
  * @param  hlptim  Pointer to the handle of the LPTIM instance.
  * @param  p_user_data Pointer to the user data.
  */
void HAL_LPTIM_SetUserData(hal_lptim_handle_t *hlptim, const void *p_user_data)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  hlptim->p_user_data = p_user_data;
}

/**
  * @brief Retrieve User Data pointer from the handle.
  * @param hlptim  Pointer to the handle of the LPTIM instance.
  * @retval Pointer to the user data.
  */
const void *HAL_LPTIM_GetUserData(const hal_lptim_handle_t *hlptim)
{
  ASSERT_DBG_PARAM((hlptim != NULL));

  return hlptim->p_user_data;
}
/**
  *  @}
  */
#endif /* USE_HAL_LPTIM_USER_DATA */

/** @addtogroup LPTIM_Exported_Functions_Group11
  *  Get clock frequency depends on instance used.
  *  - Call HAL_LPTIM_GetClockFreq() Get the LPTIM instance kernel clock frequency
  *  @{
  */

/**
  * @brief  Return the peripheral clock frequency for LPTIMx.
  * @param  hlptim Pointer to the handle of the LPTIM instance.
  * @retval uint32_t Frequency in Hz
  * @retval 0 source clock of the lptimx not configured or not ready
  */
uint32_t HAL_LPTIM_GetClockFreq(const hal_lptim_handle_t *hlptim)
{
  STM32_UNUSED(hlptim);

  /* Check the LPTIM handle */
  ASSERT_DBG_PARAM((hlptim != NULL));

  /* Check the global state, the driver need to be at least configured */
  ASSERT_DBG_STATE(hlptim->global_state, HAL_LPTIM_STATE_INIT | HAL_LPTIM_STATE_IDLE | HAL_LPTIM_STATE_ACTIVE);
  return HAL_RCC_LPTIM1_GetKernelClkFreq();
}

/**
  *  @}
  */

/**
  *  @}
  */

/**
  *  @}
  */

#endif /* USE_HAL_LPTIM_MODULE */
#endif /* LPTIM1 */
/**
  *  @}
  */
