/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_tim.c
  * @brief   TIM HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Timer (TIM) peripheral:
  *           + TIM initialization/de-initialization
  *           + TIM state and error functions
  *           + TIM time base functions
  *           + TIM output channel functions
  *           + TIM input channel functions
  *           + TIM one-pulse functions
  *           + TIM encoder functions
  *           + TIM external trigger configuration
  *           + TIM master/slave functions
  *           + TIM OCRef clear functions
  *           + TIM DMA burst functions
  *           + TIM break functions
  *           + TIM dead-time functions
  *           + TIM protection
  *           + TIM commutation feature control
  *           + TIM software event generation
  *           + TIM IRQ handler and callback functions
  *           + TIM setter and getter for user data
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
#if defined (TIM1)  \
 || defined (TIM2)  \
 || defined (TIM3)  \
 || defined (TIM4)  \
 || defined (TIM5)  \
 || defined (TIM6)  \
 || defined (TIM7)  \
 || defined (TIM8)  \
 || defined (TIM12) \
 || defined (TIM15) \
 || defined (TIM16) \
 || defined (TIM17)
#if defined (USE_HAL_TIM_MODULE) && (USE_HAL_TIM_MODULE == 1)

/** @addtogroup TIM
  *  @{
  */
/** @defgroup TIM_Introduction TIM Introduction
  * @{

  - The TIM hardware abstraction layer (HAL) provides a set of APIs to interface with STM32 timers.

  - STM32 timers (TIM) are used for precise time base generation, pulse width modulation (PWM), and event
    measurement, such as input capture and output compare. They enable motor control, signal timing, and encoder
    decoding in embedded systems.

  - The TIM HAL driver simplifies the configuration, initialization, and management of timer operations by supporting
    various modes such as polling, interrupt, and DMA, enabling flexible and efficient timer control.

  - Additionally, it supports multiple timer types (advanced-control, general-purpose, lite, and basic timers)
    depending on the STM32 device, ensuring portability and consistency across different STM32 series.
  */
/**
  * @}
  */

/** @defgroup TIM_How_To_Use TIM How To Use
  * @{

  # TIMER Generic features
  Depending on the timer's type (Basic, Lite, General-purpose, Advanced), the timer features include:
  - 16-bit or 32-bit up, down, up/down auto-reload counter.
  - 16-bit programmable prescaler allowing division (also on the fly) of the
  counter clock frequency by any factor between 1 and 65536.
  - Up to 7 independent channels for:
    - Input Capture
    - Output Compare
    - PWM generation (Edge and Center-aligned Mode)
    - One-pulse mode output
  - Complementary outputs with programmable dead-time
  - Synchronization circuit to control the timer with external signals and to interconnect
  several timers together.
  - Repetition counter to update the timer registers only after a given number of cycles
  of the counter.
  - Encoder interface mode
  - Preload feature:\n
    The preload feature is available for:
    - The auto-reload timer register (TIMx_ARR)
    - The timer prescaler register (TIMx_PSC) (cannot be turned off)
    - The timer channel registers (TIMx_CCRy)
    - ...

  # Callback registration

  The compilation flag USE_HAL_TIM_REGISTER_CALLBACKS, when set to 1,
  allows the user to configure the driver callbacks dynamically.

  Use the function HAL_TIM_Register<Callback_name>() to register a callback.

  It takes the HAL peripheral handle and a pointer to the user callback function.

  These functions allow registration of the following callbacks:
  @if defined(USE_HAL_TIM_REGISTER_CALLBACKS)
    - @ref HAL_TIM_RegisterErrorCallback                : TIM Error Callback.
    - @ref HAL_TIM_RegisterUpdateCallback               : TIM Period Elapsed Callback.
    - @ref HAL_TIM_RegisterUpdateHalfCpltCallback       : TIM Period Elapsed half complete Callback.
    - @ref HAL_TIM_RegisterTriggerCallback              : TIM Trigger Callback.
    - @ref HAL_TIM_RegisterTriggerHalfCpltCallback      : TIM Trigger half complete Callback.
    - @ref HAL_TIM_RegisterInputCaptureCallback         : TIM Input Capture Callback.
    - @ref HAL_TIM_RegisterInputCaptureHalfCpltCallback : TIM Input Capture half complete Callback.
    - @ref HAL_TIM_RegisterCompareMatchCallback         : TIM Compare Match Callback.
    - @ref HAL_TIM_RegisterCompareMatchHalfCpltCallback : TIM Compare Match half complete Callback.
    - @ref HAL_TIM_RegisterCommutationCallback          : TIM Commutation Callback.
    - @ref HAL_TIM_RegisterCommutationHalfCpltCallback  : TIM Commutation half complete Callback.
    - @ref HAL_TIM_RegisterBreakCallback                : TIM Break Callback.
    - @ref HAL_TIM_RegisterBreak2Callback               : TIM Break2 Callback.
    - @ref HAL_TIM_RegisterSystemBreakCallback          : TIM System Break Callback.
    - @ref HAL_TIM_RegisterSoftwareBreakCallback        : TIM Software Break Callback.
    - @ref HAL_TIM_RegisterEncoderIndexCallback         : TIM Encoder Index Callback.
    - @ref HAL_TIM_RegisterDirectionChangeCallback      : TIM Direction Change Callback.
    - @ref HAL_TIM_RegisterIndexErrorCallback           : TIM Index Error Callback.
    - @ref HAL_TIM_RegisterTransitionErrorCallback      : TIM Transition Error Callback.
  @endif


  By default, after initialization and when the state is HAL_TIM_STATE_INIT,
  all interrupt callbacks are set to the corresponding weak functions HAL_TIM_<Callback_name>().

  Callbacks can be registered only in the @ref HAL_TIM_STATE_IDLE state.

  When the compilation flag USE_HAL_TIM_REGISTER_CALLBACKS is set to 0 or
  not defined, the callback registration feature is not available, and all callbacks
  are set to the corresponding weak functions.

  # How to use this driver

  Use the TIM driver for purposes including:
    - Time base generation
    - Measuring the pulse lengths and duty cycle of input signals (input capture)
    - Generating output waveforms (output compare, PWM, complementary PWM with dead-time insertion)
    - Pulse generation
    - Determining rotor speed/position feedback provided by a quadrature encoder or a hall sensor

  Use the following programming sequence:
    - Initialize the TIM handle (registration of a particular instance).
    - Configure the different resources of the timer depending on the usage.
    - Start channels, if needed, and then start the timer counter.
  */
/**
  * @}
  */

/** @defgroup TIM_Configuration_Table TIM Configuration Table
  * @{
  # Configuration inside the TIM driver

  Config defines                 | Where            | Default value     | Note
  ------------------------------ | ---------------- | ----------------- | -------------------------------------------
  USE_HAL_TIM_MODULE             | hal_conf.h       | 1                 | HAL TIM module is enabled
  USE_HAL_TIM_DMA                | hal_conf.h       | 1                 | Enable the DMA code inside TIM
  USE_HAL_TIM_REGISTER_CALLBACKS | hal_conf.h       | 0                 | Enable register callback feature
  USE_HAL_TIM_CLK_ENABLE_MODEL   | hal_conf.h       | HAL_CLK_ENABLE_NO | Enable the gating of the peripheral clock
  USE_HAL_CHECK_PARAM            | hal_conf.h       | 0                 | Enable run-time parameter check
  USE_HAL_CHECK_PROCESS_STATE    | hal_conf.h       | 0                 | Enable atomic access to process state check
  USE_ASSERT_DBG_PARAM           | PreProcessor env | NA                | Enable the parameter assert
  USE_ASSERT_DBG_STATE           | PreProcessor env | NA                | Enable the state assert
  USE_HAL_TIM_GET_LAST_ERRORS    | hal_conf.h       | 0                 | Enable retrieving the error codes
  USE_HAL_TIM_USER_DATA          | hal_conf.h       | 0                 | Add an user data inside HAL TIM handle
  */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @defgroup TIM_Private_Types TIM Private Types
  *  @{
  */

/** Alias for the CMSIS instance type definition */
typedef TIM_TypeDef tim_t;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief TIM channels DMA request structure definition.
  */
typedef struct
{
  /** DMA request for the channel */
  uint32_t dma_req;

  /** DMA handle index for the channel */
  hal_tim_dma_index_t dma_idx;

} tim_cc_dma_config_t;

/**
  * @brief DMA handle configuration structure definition.
  */
typedef struct
{
  /** DMA request for the channel */
  uint32_t dma_req;

  /** DMA data half transfer complete callback */
  hal_dma_cb_t halfcplt_cb;

  /** DMA data transfer complete callback */
  hal_dma_cb_t cplt_cb;

  /** DMA handle index for the channel */
  hal_tim_dma_index_t dma_idx;

} tim_dma_config_t;
#endif /* USE_HAL_TIM_DMA */
/**
  *  @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup TIM_Private_Constants  TIM Private Constants
  *  @{
  */

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Number of TIM channels supporting DMA requests.
  */
#define NB_TIM_CC_DMA_CONFIG  (4U)

/**
  * @brief LUT to associate a DMA request and ID for a TIM channel.
  */
static const tim_cc_dma_config_t dma_channel_info[NB_TIM_CC_DMA_CONFIG] =
{
  {LL_TIM_DIER_CC1DE, HAL_TIM_DMA_ID_CC1},
  {LL_TIM_DIER_CC2DE, HAL_TIM_DMA_ID_CC2},
  {LL_TIM_DIER_CC3DE, HAL_TIM_DMA_ID_CC3},
  {LL_TIM_DIER_CC4DE, HAL_TIM_DMA_ID_CC4}
};
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief LL TIM Channels lookup table.
  * @note  Indices are given by @ref hal_tim_channel_t or @ref hal_tim_oc_compare_unit_t.
  */
static const uint32_t ll_tim_channels[HAL_TIM_CHANNELS] =
{
  LL_TIM_CHANNEL_CH1,
  LL_TIM_CHANNEL_CH2,
  LL_TIM_CHANNEL_CH3,
  LL_TIM_CHANNEL_CH4,
  LL_TIM_CHANNEL_CH5,
  LL_TIM_CHANNEL_CH6,
  LL_TIM_CHANNEL_CH7,
  LL_TIM_CHANNEL_CH1N,
  LL_TIM_CHANNEL_CH2N,
  LL_TIM_CHANNEL_CH3N,
  LL_TIM_CHANNEL_CH4N,
};

/**
  * @brief Mask for all LL channels.
  */
#define TIM_ALL_LL_CHANNELS (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | \
                             LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | \
                             LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N | \
                             LL_TIM_CHANNEL_CH4 | LL_TIM_CHANNEL_CH4N | \
                             LL_TIM_CHANNEL_CH5 | LL_TIM_CHANNEL_CH6  | \
                             LL_TIM_CHANNEL_CH7)

/**
  * @brief Define channel state idle, whether it is an OC or an IC channel.
  */
#define TIM_CHANNEL_STATE_IDLE (HAL_TIM_OC_CHANNEL_STATE_IDLE | \
                                HAL_TIM_IC_CHANNEL_STATE_IDLE)

/**
  * @brief Timeout for break input rearm.
  */
#define TIM_BREAK_INPUT_REARM_TIMEOUT_MS  (5U)

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Indexes for tim_dma_config_t table in TIM_Start_DMA_Opt().
  */
/** Update index */
#define UPDATE_DMA_REQ_IDX       (0U)
/** Commutation index */
#define COMMUTATION_DMA_REQ_IDX  (1U)
/** Trigger index */
#define TRIGGER_DMA_REQ_IDX      (2U)

/**
  * @brief Shift to switch from dma burst source to dma index.
  */
#define TIM_DMABURST_DMAINDEX_SHIFT  (TIM_DCR_DBSS_Pos)

/**
  * @brief mask for all dma requests.
  */
#define TIM_DMAREQ_REQUESTS_MASK     (LL_TIM_DIER_UDE | LL_TIM_DIER_CC1DE | LL_TIM_DIER_CC2DE \
                                      | LL_TIM_DIER_CC3DE | LL_TIM_DIER_CC4DE | LL_TIM_DIER_COMDE \
                                      | LL_TIM_DIER_TDE)
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Shift to switch from HAL filter to LL specific filter and vice versa.
  */
/** HAL filter to LL IC filter and vice versa */
#define TIM_IC_FILTER_SHIFT  (24U - LL_TIM_IC_CONFIG_POS)  /* Bit position (HAL_TIM_FDIV1_N2 - LL_TIM_IC_CONFIG_POS) */
/** HAL filter to LL ETR filter and vice versa */
#define TIM_ETR_FILTER_SHIFT  (20U)  /* Bit position (HAL_TIM_FDIV1_N2 - LL_TIM_ETR_FILTER_FDIV1_N2) */
/** HAL break filter to LL break filter */
#define TIM_BREAK_FILTER_SHIFT  (12U)  /* Bit position (HAL_TIM_FDIV1_N2 - LL_TIM_BREAK_FILTER_FDIV1_N2) */
/** HAL break2 filter to LL break2 filter and vice versa */
#define TIM_BREAK2_FILTER_SHIFT  (8U)   /* Bit position (HAL_TIM_FDIV1_N2 - LL_TIM_BREAK2_FILTER_FDIV1_N2) */

/**
  * @brief Shift to switch from HAL break polarity to LL break2 polarity and vice versa.
  */
#define TIM_BREAK2_POLARITY_SHIFT  (12U)  /* Bit position (LL_TIM_BREAK2_POLARITY_HIGH - LL_TIM_BREAK_POLARITY_HIGH) */

/**
  * @brief Mask for the breaks af mode.
  */
#define TIM_BRK_BRK2_MODE_MASK  (0x30000000U)  /* Bit position (TIMx_BDTR.BKBID & TIMx_BDTR.BK2BID) */

/**
  * @brief Mask for the dithered bits in ARR and CCR registers.
  */
#define TIM_DITHERING_MASK  (0xFU)  /* Bit position (TIMx_ARR[3:0] & TIMx_CCRy[3:0]) */

/**
  * @brief All optional interrupts mask.
  */
#define TIM_OPTIONAL_INTERRUPTS_MASK \
  (HAL_TIM_OPT_IT_UPDATE | HAL_TIM_OPT_IT_COMMUTATION | HAL_TIM_OPT_IT_TRIGGER_INPUT \
   | HAL_TIM_OPT_IT_BREAK | HAL_TIM_OPT_IT_ENCODER_INDEX | HAL_TIM_OPT_IT_ENCODER_DIRECTION \
   | HAL_TIM_OPT_IT_ENCODER_INDEX_ERROR | HAL_TIM_OPT_IT_ENCODER_TRANSITION_ERROR)

/**
  * @brief Encoder optional interrupts mask.
  */
#define TIM_ENCODER_OPTIONAL_INTERRUPTS_MASK \
  (HAL_TIM_OPT_IT_ENCODER_DIRECTION | HAL_TIM_OPT_IT_ENCODER_TRANSITION_ERROR)

/**
  * @brief Encoder index optional interrupts mask.
  */
#define TIM_ENCODER_IDX_OPTIONAL_INTERRUPTS_MASK \
  (HAL_TIM_OPT_IT_ENCODER_INDEX | HAL_TIM_OPT_IT_ENCODER_INDEX_ERROR)

/**
  * @brief Mask for all combined 3-phase pwm modes.
  */
#define TIM_GROUP_MASK \
  (HAL_TIM_GROUP_AND_OC1REFC | HAL_TIM_GROUP_AND_OC2REFC | HAL_TIM_GROUP_AND_OC3REFC \
   | HAL_TIM_GROUP_AND_OC4REFC | HAL_TIM_GROUP_OR_OC1REFC  | HAL_TIM_GROUP_OR_OC2REFC \
   | HAL_TIM_GROUP_OR_OC3REFC  | HAL_TIM_GROUP_OR_OC4REFC)

/**
  * @brief Mask for the breaks input sources polarizable.
  */
#define TIM_BRK_BRK2_POLARITY_MASK  (0x1FU)  /* Bit position (max TIMx_AF1.BKCMP4E or TIMx_AF2.BK2CMP4E) */

/**
  * @brief Mask for specific TIM1 break input sources.
  */
#if defined(TIM16)
#define TIM1_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM1_GPIO | HAL_TIM_BRK_TIM1_COMP1_OUT \
   | HAL_TIM_BRK_TIM1_TIM8_BKIN | HAL_TIM_BRK_TIM1_TIM15_BKIN \
   | HAL_TIM_BRK_TIM1_TIM16_BKIN | HAL_TIM_BRK_TIM1_TIM17_BKIN)
#elif defined(COMP2)
#define TIM1_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM1_GPIO | HAL_TIM_BRK_TIM1_COMP1_OUT \
   | HAL_TIM_BRK_TIM1_COMP2_OUT | HAL_TIM_BRK_TIM1_TIM8_BKIN \
   | HAL_TIM_BRK_TIM1_TIM15_BKIN)
#endif /* TIM20 && PLAY1 */

/**
  * @brief Mask for specific TIM8 break input sources.
  */
#if defined(TIM16)
#define TIM8_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM8_GPIO | HAL_TIM_BRK_TIM8_COMP1_OUT \
   | HAL_TIM_BRK_TIM8_TIM1_BKIN | HAL_TIM_BRK_TIM8_TIM15_BKIN \
   | HAL_TIM_BRK_TIM8_TIM16_BKIN | HAL_TIM_BRK_TIM8_TIM17_BKIN)
#elif defined(COMP2)
#define TIM8_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM8_GPIO | HAL_TIM_BRK_TIM8_COMP1_OUT \
   | HAL_TIM_BRK_TIM8_COMP2_OUT | HAL_TIM_BRK_TIM8_TIM1_BKIN \
   | HAL_TIM_BRK_TIM8_TIM15_BKIN)
#endif /* TIM20 && PLAY1 */

/**
  * @brief Mask for specific TIM15 break input sources.
  */
#if defined(TIM16)
#define TIM15_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM15_GPIO | HAL_TIM_BRK_TIM15_COMP1_OUT \
   | HAL_TIM_BRK_TIM15_TIM1_BKIN | HAL_TIM_BRK_TIM15_TIM8_BKIN \
   | HAL_TIM_BRK_TIM15_TIM16_BKIN | HAL_TIM_BRK_TIM15_TIM17_BKIN)
#elif defined(COMP2)
#define TIM15_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM15_GPIO | HAL_TIM_BRK_TIM15_COMP1_OUT \
   | HAL_TIM_BRK_TIM15_COMP2_OUT | HAL_TIM_BRK_TIM15_TIM1_BKIN \
   | HAL_TIM_BRK_TIM15_TIM8_BKIN)
#endif /* TIM20 && PLAY1 */

#if defined(TIM16)
/**
  * @brief Mask for specific TIM16 break input sources.
  */
#define TIM16_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM16_GPIO | HAL_TIM_BRK_TIM16_COMP1_OUT \
   | HAL_TIM_BRK_TIM16_TIM1_BKIN | HAL_TIM_BRK_TIM16_TIM8_BKIN \
   | HAL_TIM_BRK_TIM16_TIM15_BKIN | HAL_TIM_BRK_TIM16_TIM17_BKIN)
#endif /* TIM16 */

#if defined(TIM17)
/**
  * @brief Mask for specific TIM17 break input sources.
  */
#define TIM17_BRK_SOURCE_MASK \
  (HAL_TIM_BRK_TIM17_GPIO | HAL_TIM_BRK_TIM17_COMP1_OUT \
   | HAL_TIM_BRK_TIM17_TIM1_BKIN | HAL_TIM_BRK_TIM17_TIM8_BKIN \
   | HAL_TIM_BRK_TIM17_TIM15_BKIN | HAL_TIM_BRK_TIM17_TIM16_BKIN)
#endif /* TIM17 */


/**
  * @brief Mask for specific TIM1 break2 input sources.
  */
#if defined(COMP2)
#define TIM1_BRK2_SOURCE_MASK \
  (HAL_TIM_BRK2_TIM1_GPIO | HAL_TIM_BRK2_TIM1_COMP1_OUT \
   | HAL_TIM_BRK2_TIM1_COMP2_OUT | HAL_TIM_BRK2_TIM1_TIM8_BKIN2)
#else
#define TIM1_BRK2_SOURCE_MASK \
  (HAL_TIM_BRK2_TIM1_GPIO | HAL_TIM_BRK2_TIM1_COMP1_OUT \
   | HAL_TIM_BRK2_TIM1_TIM8_BKIN2)
#endif /* TIM20 && PLAY1 */

/**
  * @brief Mask for specific TIM8 break2 input sources.
  */
#if defined(COMP2)
#define TIM8_BRK2_SOURCE_MASK \
  (HAL_TIM_BRK2_TIM8_GPIO | HAL_TIM_BRK2_TIM8_COMP1_OUT \
   | HAL_TIM_BRK2_TIM8_COMP2_OUT | HAL_TIM_BRK2_TIM8_TIM1_BKIN2)
#else
#define TIM8_BRK2_SOURCE_MASK \
  (HAL_TIM_BRK2_TIM8_GPIO | HAL_TIM_BRK2_TIM8_COMP1_OUT \
   | HAL_TIM_BRK2_TIM8_TIM1_BKIN2)
#endif /* TIM20 && PLAY1 */


/**
  *  @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup  TIM_Private_Macros TIM Private Macros
  *  @{
  */

/**
  * @brief  Macro for the control of TIM optional interrupts validity (subset of @ref TIM_Optional_Interruptions).
  * @param  instance    TIM instance.
  * @param  interrupts  TIM optional interrupts.
  * @retval SET (optional interrupts are valid) or RESET (optional interrupts are invalid)
  */
#define IS_TIM_OPTIONAL_INTERRUPTS(instance, interrupts) \
  (((interrupts) != 0U) \
   && !((interrupts) & ~TIM_OPTIONAL_INTERRUPTS_MASK) \
   && !(((interrupts) & HAL_TIM_OPT_IT_COMMUTATION) && !IS_TIM_COMMUTATION_EVENT_INSTANCE((instance))) \
   && !(((interrupts) & HAL_TIM_OPT_IT_TRIGGER_INPUT) && !IS_TIM_SLAVE_INSTANCE((instance))) \
   && !(((interrupts) & HAL_TIM_OPT_IT_BREAK) && !IS_TIM_BREAK_INSTANCE((instance))) \
   && !(((interrupts) & TIM_ENCODER_OPTIONAL_INTERRUPTS_MASK) && !IS_TIM_ENCODER_INTERFACE_INSTANCE((instance))) \
   && !(((interrupts) & TIM_ENCODER_IDX_OPTIONAL_INTERRUPTS_MASK) && !IS_TIM_ENCODER_INTERFACE_INSTANCE((instance))) \
   && !(((interrupts) & TIM_ENCODER_IDX_OPTIONAL_INTERRUPTS_MASK) && !IS_TIM_ETR_INSTANCE((instance))))

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Macro that returns the global state depending on the DMA silent mode.
  * @param  interrupts  DMA interrupts.
  * @retval HAL_TIM_STATE_ACTIVE_SILENT (DMA silent mode is active) or
  *         HAL_TIM_STATE_ACTIVE (DMA silent mode is not active).
  */
#define TIM_STATE_ACTIVE(interrupts) \
  ((STM32_IS_BIT_SET((interrupts), (uint32_t)HAL_DMA_OPT_IT_SILENT)) ? \
   HAL_TIM_STATE_ACTIVE_SILENT : HAL_TIM_STATE_ACTIVE)

/**
  * @brief  Macro that returns the output channel state depending on the DMA silent mode.
  * @param  interrupts  DMA interrupts.
  * @retval HAL_TIM_OC_CHANNEL_STATE_ACTIVE_SILENT (DMA silent mode is active) or
  *         HAL_TIM_OC_CHANNEL_STATE_ACTIVE (DMA silent mode is not active).
  */
#define TIM_OC_CHANNEL_STATE_ACTIVE(interrupts) \
  ((STM32_IS_BIT_SET((interrupts), (uint32_t)HAL_DMA_OPT_IT_SILENT)) ? \
   HAL_TIM_OC_CHANNEL_STATE_ACTIVE_SILENT : HAL_TIM_OC_CHANNEL_STATE_ACTIVE)

/**
  * @brief  Macro that returns the input channel state depending on the DMA silent mode.
  * @param  interrupts  DMA interrupts.
  * @retval HAL_TIM_IC_CHANNEL_STATE_ACTIVE_SILENT (DMA silent mode is active) or
  *         HAL_TIM_IC_CHANNEL_STATE_ACTIVE (DMA silent mode is not active).
  */
#define TIM_IC_CHANNEL_STATE_ACTIVE(interrupts) \
  ((STM32_IS_BIT_SET((interrupts), (uint32_t)HAL_DMA_OPT_IT_SILENT)) ? \
   HAL_TIM_IC_CHANNEL_STATE_ACTIVE_SILENT : HAL_TIM_IC_CHANNEL_STATE_ACTIVE)

#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
/**
  * @brief  Macro for the control of DMA silent mode validity.
  * @param  handle     TIM handle.
  * @param  channel    DMA channel.
  * @param  interrupts DMA interrupts.
  * @retval SET (DMA silent mode is valid) or RESET (DMA silent mode is invalid).
  */
#define IS_TIM_DMA_VALID_SILENT_MODE(handle, channel, interrupts) \
  (((interrupts) == HAL_TIM_OPT_DMA_IT_SILENT)  \
   && (handle->hdma[channel]->xfer_mode != HAL_DMA_XFER_MODE_LINKEDLIST_CIRCULAR) ? 0U : 1U)
#endif /* USE_HAL_DMA_LINKEDLIST */

/**
  * @brief  Tell whether the DMA silent mode is active.
  * @param  state  The state to check.
  * @retval SET (DMA silent mode is active) or RESET (otherwise).
  */
#define IS_TIM_ACTIVE_SILENT(state) \
  ((uint32_t)(state) & (uint32_t)HAL_TIM_ACTIVE_SILENT)

#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Check if the break input rearm timeout period is expired.
  * @param  delta_ticks  Delta ticks to compare with the timeout period
  * @retval SET (timeout period is expired) or RESET (otherwise).
  */
#define TIM_BREAK_INPUT_REARM_TIMEOUT_PERIOD_EXPIRED(delta_ticks) \
  ((delta_ticks) > (uint32_t)TIM_BREAK_INPUT_REARM_TIMEOUT_MS)

/**
  * @brief  Get the mask for changing the channel source of a given channel
  *         in the register TISEL.
  * @param  channel  The channel to get the mask for.
  * @note   Input channel 1 -> input channel index 0 -> mask 0xF. \n
  *         Input channel 2 -> input channel index 1 -> mask 0xF00. \n
  *         Input channel 3 -> input channel index 2 -> mask 0xF0000. \n
  *         Input channel 4 -> input channel index 3 -> mask 0xF000000.
  * @retval The mask for changing the channel source of the given channel.
  */
#define MASK_TISEL(channel) ((0xFUL) << ((channel) << 3))

/**
  * @brief  Get the shift to switch from LL to HAL Break/Break2 polarity and vice versa, depending the Break input.
  * @param  brkin  The Break input.
  * @retval The shift to switch from LL to HAL and vice versa.
  */
#define TIM_BRK_BRK2_POLARITY_SHIFT(brkin) \
  (((brkin) << 2) + ((brkin) << 3))

/**
  * @brief  Get the shift to switch from LL to HAL Break/Break2 filter and vice versa, depending the Break input.
  * @param  brkin  The Break input.
  * @retval The shift to switch from LL to HAL and vice versa.
  */
#define TIM_BRK_BRK2_FILTER_SHIFT(brkin) \
  ((brkin) << 2)

/**
  * @brief  Get the timer handle registered in the dma handle.
  * @param  hdma  DMA handle.
  * @retval TIM handle.
  */
#define TIM_GET_HDMA_PARENT(hdma) \
  ((hal_tim_handle_t *)((hdma)->p_parent))

/**
  * @brief Check if the channel is configured as input channel.
  * @param  instance  TIM instance.
  * @param  channel   Channel of interest.
  *                   It can be any of the following values: \n
  *                   @arg @ref HAL_TIM_CHANNEL_1
  *                   @arg @ref HAL_TIM_CHANNEL_2
  *                   @arg @ref HAL_TIM_CHANNEL_3
  *                   @arg @ref HAL_TIM_CHANNEL_4
  * @retval SET (input channel) or RESET (output channel).
  */
#define TIM_IS_INPUT_CHANNEL(instance, channel) \
  (LL_TIM_IC_GetActiveInput((instance), ll_tim_channels[(channel)]) != 0U)

/**
  * @brief Check if all channels are disabled.
  * @param  instance  TIM instance.
  * @retval SET (all channels are disabled) or RESET (otherwise).
  */
#define TIM_ARE_ALL_CHANNELS_DISABLED(instance) \
  ((LL_TIM_READ_REG((instance), CCER) & TIM_ALL_LL_CHANNELS) == 0x0U)

/**
  * @brief  Transform the HAL filter into a LL IC filter.
  * @param  filter  The HAL filter.
  * @retval The LL IC filter.
  */
#define TIM_IC_HAL2LL_FILTER(filter) \
  (((uint32_t)(filter)) >> TIM_IC_FILTER_SHIFT)

/**
  * @brief  Transform the LL IC filter into a HAL filter.
  * @param  filter  The LL IC filter.
  * @retval The HAL filter.
  */
#define TIM_IC_LL2HAL_FILTER(filter) \
  ((hal_tim_filter_t)((uint32_t)((filter) << TIM_IC_FILTER_SHIFT)))

/**
  * @brief  Transform the HAL filter into a LL ETR filter.
  * @param  filter  The HAL filter.
  * @retval The LL ETR filter.
  */
#define TIM_ETR_HAL2LL_FILTER(filter) \
  (((uint32_t)(filter)) >> TIM_ETR_FILTER_SHIFT)

/**
  * @brief  Transform the LL ETR filter into a HAL filter.
  * @param  filter  The LL ETR filter.
  * @retval The HAL filter.
  */
#define TIM_ETR_LL2HAL_FILTER(filter) \
  ((hal_tim_filter_t)((uint32_t)((filter) << TIM_ETR_FILTER_SHIFT)))

/**
  * @brief  Transform the HAL filter into a LL Break filter.
  * @param  filter  The HAL filter.
  * @retval The LL Break filter.
  */
#define TIM_BREAK_HAL2LL_FILTER(filter) \
  (((uint32_t)(filter)) >> TIM_BREAK_FILTER_SHIFT)

/**
  * @brief  Transform the LL Break filter into a HAL filter.
  * @param  filter  The LL Break filter.
  * @retval The HAL filter.
  */
#define TIM_BREAK_LL2HAL_FILTER(filter) \
  ((hal_tim_filter_t)((uint32_t)((filter) << TIM_BREAK_FILTER_SHIFT)))

/**
  * @brief  Transform the HAL filter into a LL Break2 filter.
  * @param  filter  The HAL filter.
  * @retval The LL Break2 filter.
  */
#define TIM_BREAK2_HAL2LL_FILTER(filter) \
  (((uint32_t)(filter)) >> TIM_BREAK2_FILTER_SHIFT)

/**
  * @brief  Transform the LL Break2 filter into a HAL filter.
  * @param  filter  The LL Break2 filter.
  * @retval The HAL filter.
  */
#define TIM_BREAK2_LL2HAL_FILTER(filter) \
  ((hal_tim_filter_t)((uint32_t)((filter) << TIM_BREAK2_FILTER_SHIFT)))

/**
  * @brief  Transform the HAL filter into LL Break/Break2 filter depending on the Break Input.
  * @param  brkin   The Break input.
  * @param  filter  The HAL filter.
  * @retval The LL Break/Break2 filter.
  */
#define TIM_BRK_BRK2_HAL2LL_FILTER(brkin, filter) \
  ((((uint32_t)(filter)) >> TIM_BREAK_FILTER_SHIFT) \
   << TIM_BRK_BRK2_FILTER_SHIFT((brkin)))

/**
  * @brief  Transform the LL Break/Break2 filter into a HAL filter depending on the Break Input.
  * @param  brkin   The Break input.
  * @param  filter  The LL Break/Break2 filter.
  * @retval The HAL filter.
  */
#define TIM_BRK_BRK2_LL2HAL_FILTER(brkin, filter) \
  ((hal_tim_filter_t)((uint32_t)((filter) >> TIM_BRK_BRK2_FILTER_SHIFT((brkin)) \
                                 << TIM_BREAK_FILTER_SHIFT)))

/**
  * @brief  Transform the HAL Break Polarity into LL Break polarity for the Break input.
  * @param  polarity  The HAL Break polarity.
  * @retval The LL Break polarity.
  */
#define TIM_BREAK_HAL2LL_POLARITY(polarity) \
  ((uint32_t)(polarity))

/**
  * @brief Transform the HAL Break Polarity into LL Break polarity for the Break input 2.
  * @param  polarity  The HAL Break polarity.
  * @retval The LL Break polarity.
  */
#define TIM_BREAK2_HAL2LL_POLARITY(polarity) \
  (((uint32_t)(polarity)) << TIM_BREAK2_POLARITY_SHIFT)

/**
  * @brief  Transform the LL Break Polarity into HAL Break polarity for a Break input.
  * @param  polarity  The LL Break polarity.
  * @retval The HAL Break polarity.
  */
#define TIM_BREAK_LL2HAL_POLARITY(polarity) \
  ((hal_tim_break_input_polarity_t)(polarity))

/**
  * @brief  Transform the LL Break2 Polarity into HAL Break polarity for a Break input 2.
  * @param  polarity  The LL Break2 polarity.
  * @retval The HAL Break polarity.
  */
#define TIM_BREAK2_LL2HAL_POLARITY(polarity) \
  ((hal_tim_break_input_polarity_t)((uint32_t)((polarity)  >> TIM_BREAK2_POLARITY_SHIFT)))

/**
  * @brief  Transform the HAL Break/Break2 Polarity into LL Break/Break2 polarity depending on the Break input.
  * @param  brkin     The Break input.
  * @param  polarity  The HAL Break(2) polarity.
  * @retval The LL Break(2) polarity.
  */
#define TIM_BRK_BRK2_HAL2LL_POLARITY(brkin, polarity) \
  (((uint32_t)(polarity))  << TIM_BRK_BRK2_POLARITY_SHIFT(brkin))

/**
  * @brief Transform the LL Break/Break2 Polarity into HAL Break polarity depending on the Break input.
  * @param brkin     The Break input.
  * @param polarity  The LL Break(2) polarity.
  * @retval The HAL Break polarity.
  */
#define TIM_BRK_BRK2_LL2HAL_POLARITY(brkin, polarity) \
  ((hal_tim_break_input_polarity_t)((uint32_t)((polarity) >> TIM_BRK_BRK2_POLARITY_SHIFT(brkin))))

/**
  * @brief  Transform the HAL Break mode into a LL Break mode for the Break input.
  * @param  mode  The HAL Break mode.
  * @retval The LL Break mode.
  */
#define TIM_BREAK_HAL2LL_MODE(mode) \
  (((uint32_t)(mode)) & LL_TIM_BREAK_AFMODE_BIDIRECTIONAL)

/**
  * @brief  Transform the HAL Break mode into a LL Break2 mode for the Break input 2.
  * @param  mode  The HAL Break mode.
  * @retval The LL Break2 mode.
  */
#define TIM_BREAK2_HAL2LL_MODE(mode) \
  (((uint32_t)(mode)) & (LL_TIM_BREAK_AFMODE_BIDIRECTIONAL << 1))

/**
  * @brief  Transform the HAL Break mode into a LL Break/Break2 mode depending on the Break input.
  * @param  brkin  The Break input.
  * @param  mode   The HAL Break mode.
  * @retval The LL Break/Break2 mode.
  */
#define TIM_BRK_BRK2_HAL2LL_MODE(brkin, mode) \
  (((uint32_t)(mode)) & (LL_TIM_BREAK_AFMODE_BIDIRECTIONAL << (brkin)))

/**
  * @brief  Transform the LL Break/Break2 mode into a HAL Break mode for the Break input.
  * @param  brkin  The Break input.
  * @param  mode   The LL Break mode.
  * @retval The HAL Break mode.
  */
#define TIM_BRK_BRK2_LL2HAL_MODE(brkin, mode) \
  ((hal_tim_break_input_mode_t)((uint32_t)(((mode) | ((mode) >> 1) | ((mode) << 1)) & \
                                           TIM_BRK_BRK2_MODE_MASK)))

/**
  * @brief  Get the LL active input from HAL capture unit source.
  * @param  capture_unit_src  The HAL capture unit source (@ref hal_tim_ic_capture_unit_src_t).
  * @note   The upper bytes store the LL active input, and the lower bytes store the LL source polarity.
  * @retval The LL active input.
  */
#define TIM_LL_ACTIVE_INPUT(capture_unit_src) ((capture_unit_src) & 0xFFFF0000U)

/**
  * @brief  Get the LL source polarity from HAL capture unit source.
  * @param  capture_unit_src  The HAL capture unit source (@ref hal_tim_ic_capture_unit_src_t).
  * @note   The upper bytes store the LL active input, and the lower bytes store the LL source polarity.
  * @retval The LL source polarity.
  */
#define TIM_LL_IC_POLARITY(capture_unit_src) ((capture_unit_src) & 0x0000FFFFU)

/**
  * @brief  Check if the timer is in a slave mode.
  * @param  instance  TIM instance.
  * @retval SET (slave mode enabled) or RESET (slave mode disabled).
  */
#define IS_TIM_SLAVE_MODE_ENABLED(instance) \
  (LL_TIM_GetSlaveMode(instance) != (uint32_t)HAL_TIM_SLAVE_DISABLED)

/**
  * @brief  Check if the timer is in a slave mode that enables the counter.
  * @param  sms  The slave mode.
  * @retval SET (slave mode enabling the counter) or RESET (otherwise).
  */
#define IS_TIM_SLAVE_MODE_ENABLING_COUNTER(sms) \
  (((sms) == (uint32_t)HAL_TIM_SLAVE_TRIGGER) \
   || ((sms) == (uint32_t)HAL_TIM_SLAVE_COMBINED_RESET_TRIGGER))

/**
  * @brief  Check if the timer instance supports the slave mode selection preload.
  * @param  instance  TIM instance.
  * @retval SET (slave mode preload supported) or RESET (otherwise).
  */
#define IS_TIM_SMS_PRELOAD_INSTANCE(instance) \
  IS_TIM_ENCODER_INTERFACE_INSTANCE((instance))

/**
  * @brief  Check if the timer instance supports the ADC synchronization.
  * @param  instance  TIM instance.
  * @retval SET (ADC synchronization supported) or RESET (otherwise).
  */
#define IS_TIM_ADC_SYNCHRO_INSTANCE(instance) \
  IS_TIM_MASTER_INSTANCE((instance))

/**
  * @brief  Check if the timer instance supports the 'pulse on compare' feature.
  * @param  instance  TIM instance.
  * @retval SET (pulse on compare supported) or RESET (otherwise).
  */
#define IS_TIM_PULSE_ON_COMPARE_INSTANCE(instance)  \
  IS_TIM_CC3_INSTANCE((instance))

/**
  * @brief  Check if the timer instance supports the 'group' feature (combined 3-phase PWM mode).
  * @param  instance  TIM instance.
  * @retval SET (group feature supported) or RESET (otherwise).
  */
#define IS_TIM_GROUP_INSTANCE(instance)  \
  IS_TIM_CC5_INSTANCE((instance))

/**
  * @brief  Check if the 'group' combination for OC5REF signal is valid.
  * @param  group  The group combination.
  * @retval SET (group combination valid) or RESET (otherwise).
  */
#define IS_TIM_GROUP(group) (((group & ~(TIM_GROUP_MASK)) == 0U) ? 1U : 0U)

/**
  * @brief  Check if the timer instance supports the 'break' feature.
  * @param  instance  TIM instance.
  * @param  brkin     The Break input.
  * @retval SET (break feature supported) or RESET (otherwise).
  */
#define IS_TIM_BRKIN_INSTANCE(instance, brkin) \
  (((brkin) == HAL_TIM_BREAK_INPUT_1)? IS_TIM_BREAK_INSTANCE((instance)) : \
   IS_TIM_BKIN2_INSTANCE((instance)))

/**
  * @brief  Get the TIM instance from the handle.
  * @param  htim  TIM handle.
  * @retval TIM instance.
  */
#define TIM_INSTANCE(htim) ((tim_t *)((uint32_t)((htim)->instance)))

/**
  * @brief  Check the value to store in the counter register (CNT).
  * @param  instance  TIM instance.
  * @param  counter   Counter value.
  * @retval SET (counter value valid) or RESET (counter value invalid).
  */
#define IS_TIM_COUNTER(instance, counter) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   ((counter) <= 0x0000FFFFU) : (1U))

/**
  * @brief  Check the value to store in the auto-reload register (ARR).
  * @param  instance  TIM instance.
  * @param  period    Period value.
  * @note   For a 32-bit counter, the constraint is only to have a positive value (with
  *         the meaning of the value depending on dithering mode). \n
  *         For a 16-bit counter, only bits 0 to 15 are meaningful (0 to 19 in dithering mode).
  * @retval SET (period value valid) or RESET (period value invalid).
  */
#define IS_TIM_PERIOD(instance, period) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   (((period) > 0U) && ((period) <= 0x000FFFEFU)) : ((period) > 0U))

/**
  * @brief  Check the integer part of the period value when dithering enabled.
  * @param  instance  TIM instance.
  * @param  period    Period value.
  * @retval SET (integer period value valid) or RESET (integer period value invalid).
  */
#define IS_TIM_PERIOD_WITH_DITHERING(instance, period) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   (((period) > 0U) && ((period) <= 0x0000FFFEU)) : \
   (((period) > 0U) && ((period) <= 0x0FFFFFFEU)))

/**
  * @brief  Check the period value when dithering is disabled.
  * @param  instance  TIM instance.
  * @param  period    Period value.
  * @retval SET (period value valid) or RESET (period value invalid).
  */
#define IS_TIM_PERIOD_WITHOUT_DITHERING(instance, period) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   (((period) > 0U) && ((period) <= 0x0000FFFFU)) : ((period) > 0U))

/**
  * @brief  Check the value to store in the repetition counter register (RCR).
  * @param  instance            TIM instance.
  * @param  repetition_counter  Repetition counter value.
  * @retval SET (repetition counter value valid) or RESET (repetition counter value invalid).
  */
#define IS_TIM_REPETITION_COUNTER(instance, repetition_counter) \
  ((IS_TIM_16B_REPETITION_COUNTER_INSTANCE((instance)) == 0U) ? \
   ((repetition_counter) <= 0x000000FFU) :                      \
   ((repetition_counter) <= 0x0000FFFFU))

/**
  * @brief  Check the value to store in the prescaler register (PSC).
  * @param  prescaler  Prescaler value.
  * @retval SET (prescaler value valid) or RESET (prescaler value invalid).
  */
#define IS_TIM_PRESCALER(prescaler) ((prescaler) <= 0x0000FFFFU)

/**
  * @brief  Check the validity of the channel.
  * @param  channel  The channel to check (@ref hal_tim_channel_t).
  * @retval SET (channel is valid) or RESET (channel is invalid).
  */
#define IS_TIM_CHANNEL(channel) (((channel) == HAL_TIM_CHANNEL_1)  \
                                 ||((channel) == HAL_TIM_CHANNEL_2)  \
                                 ||((channel) == HAL_TIM_CHANNEL_3)  \
                                 ||((channel) == HAL_TIM_CHANNEL_4)  \
                                 ||((channel) == HAL_TIM_CHANNEL_5)  \
                                 ||((channel) == HAL_TIM_CHANNEL_6)  \
                                 ||((channel) == HAL_TIM_CHANNEL_7)  \
                                 ||((channel) == HAL_TIM_CHANNEL_1N) \
                                 ||((channel) == HAL_TIM_CHANNEL_2N) \
                                 ||((channel) == HAL_TIM_CHANNEL_3N) \
                                 ||((channel) == HAL_TIM_CHANNEL_4N))

/**
  * @brief  Check the validity of the counter mode.
  * @param  mode  The counter mode to check (@ref hal_tim_counter_mode_t).
  * @retval SET (counter mode is valid) or RESET (counter mode is invalid).
  */
#define IS_TIM_COUNTER_MODE(mode) (((mode) == HAL_TIM_COUNTER_UP)            \
                                   ||((mode) == HAL_TIM_COUNTER_DOWN)        \
                                   ||((mode) == HAL_TIM_COUNTER_CENTER_DOWN) \
                                   ||((mode) == HAL_TIM_COUNTER_CENTER_UP)   \
                                   ||((mode) == HAL_TIM_COUNTER_CENTER_UP_DOWN))

/**
  * @brief  Check the validity of the DTS clock prescaler.
  * @param  psc  The prescaler to check (@ref hal_tim_dts_prescaler_t).
  * @retval SET (DTS prescaler is valid) or RESET (DTS prescaler is invalid).
  */
#define IS_TIM_DTS_PRESCALER(psc) (((psc) == HAL_TIM_DTS_DIV1) \
                                   ||((psc) == HAL_TIM_DTS_DIV2) \
                                   ||((psc) == HAL_TIM_DTS_DIV4) \
                                   ||((psc) == HAL_TIM_DTS_DIV8))

/**
  * @brief  Check the validity of the DTS2 clock prescaler.
  * @param  psc  The prescaler to check (@ref hal_tim_dts2_prescaler_t).
  * @retval SET (DTS2 prescaler is valid) or RESET (DTS2 prescaler is invalid).
  */
#define IS_TIM_DTS2_PRESCALER(psc) (((psc) == HAL_TIM_DTS2_DIV1)     \
                                    ||((psc) == HAL_TIM_DTS2_DIV4)     \
                                    ||((psc) == HAL_TIM_DTS2_DIV16)    \
                                    ||((psc) == HAL_TIM_DTS2_DIV64)    \
                                    ||((psc) == HAL_TIM_DTS2_DIV256)   \
                                    ||((psc) == HAL_TIM_DTS2_DIV1024)  \
                                    ||((psc) == HAL_TIM_DTS2_DIV4096)  \
                                    ||((psc) == HAL_TIM_DTS2_DIV16384) \
                                    ||((psc) == HAL_TIM_DTS2_DIV65536) \
                                    ||((psc) == HAL_TIM_DTS2_DIV262144))

/**
  * @brief  Check if the clock source is an encoder mode.
  * @param  src  The clock source (@ref hal_tim_clk_src_t).
  * @retval SET (encoder mode clock source) or RESET (otherwise).
  */
#define TIM_IS_CLK_ENCODER(src) (((src) == HAL_TIM_CLK_ENCODER_X1_TI1)            \
                                 ||((src) == HAL_TIM_CLK_ENCODER_X1_TI2)            \
                                 ||((src) == HAL_TIM_CLK_ENCODER_X2_TI1)            \
                                 ||((src) == HAL_TIM_CLK_ENCODER_X2_TI2)            \
                                 ||((src) == HAL_TIM_CLK_ENCODER_X4_TI12)           \
                                 ||((src) == HAL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1)  \
                                 ||((src) == HAL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12) \
                                 ||((src) == HAL_TIM_CLK_ENCODER_CLK_PLUS_X2)       \
                                 ||((src) == HAL_TIM_CLK_ENCODER_CLK_PLUS_X1)       \
                                 ||((src) == HAL_TIM_CLK_ENCODER_DIR_CLK_X2)        \
                                 ||((src) == HAL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12))

/**
  * @brief  Check the validity of the clock source.
  * @param  src  The clock source to check (@ref hal_tim_clk_src_t).
  * @retval SET (clock source is valid) or RESET (clock source is invalid).
  */
#define IS_TIM_CLK_SRC(src) (((src) == HAL_TIM_CLK_INTERNAL)      \
                             ||((src) == HAL_TIM_CLK_EXTERNAL_MODE1)\
                             ||((src) == HAL_TIM_CLK_EXTERNAL_MODE2)\
                             ||TIM_IS_CLK_ENCODER(src))

/**
  * @brief  Check the validity of the update source.
  * @param  src  The update source to check (@ref hal_tim_update_src_t).
  * @retval SET (update source is valid) or RESET (update source is invalid).
  */
#define IS_TIM_UPDATE_SRC(src) (((src) == HAL_TIM_UPDATE_REGULAR) \
                                ||((src) == HAL_TIM_UPDATE_COUNTER))

/**
  * @brief  Check the validity of the filter.
  * @param  fdiv  The filter division to check (@ref hal_tim_filter_t).
  * @retval SET (filter division is valid) or RESET (filter division is invalid).
  */
#define IS_TIM_FILTER(fdiv) (((fdiv) == HAL_TIM_FDIV1)     \
                             ||((fdiv) == HAL_TIM_FDIV1_N2)  \
                             ||((fdiv) == HAL_TIM_FDIV1_N4)  \
                             ||((fdiv) == HAL_TIM_FDIV1_N8)  \
                             ||((fdiv) == HAL_TIM_FDIV2_N6)  \
                             ||((fdiv) == HAL_TIM_FDIV2_N8)  \
                             ||((fdiv) == HAL_TIM_FDIV4_N6)  \
                             ||((fdiv) == HAL_TIM_FDIV4_N8)  \
                             ||((fdiv) == HAL_TIM_FDIV8_N6)  \
                             ||((fdiv) == HAL_TIM_FDIV8_N8)  \
                             ||((fdiv) == HAL_TIM_FDIV16_N5) \
                             ||((fdiv) == HAL_TIM_FDIV16_N6) \
                             ||((fdiv) == HAL_TIM_FDIV16_N8) \
                             ||((fdiv) == HAL_TIM_FDIV32_N5) \
                             ||((fdiv) == HAL_TIM_FDIV32_N6) \
                             ||((fdiv) == HAL_TIM_FDIV32_N8))

/**
  * @brief  Check the validity of the trigger selection.
  * @param  instance  TIM instance.
  * @param  trigger   The trigger selection to check (@ref hal_tim_trig_sel_t).
  * @retval SET (trigger selection is valid) or RESET (trigger selection is invalid).
  */
#if defined(TIM4)
#define IS_TIM_TRIG_SEL(instance, trigger) \
  (((((trigger) == HAL_TIM_TRIG_ITR0) \
     || ((trigger) == HAL_TIM_TRIG_ITR1)) \
    && (((instance) == TIM2) || ((instance) == TIM3) \
        || ((instance) == TIM4) || ((instance) == TIM5) \
        || ((instance) == TIM8) || ((instance) == TIM12) \
        || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR2) \
       && (((instance) == TIM1) || ((instance) == TIM3) \
           || ((instance) == TIM4) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR3) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM4) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR4) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR5) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM4) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR6) \
        || ((trigger) == HAL_TIM_TRIG_ITR7)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM4) \
           || ((instance) == TIM5) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR8) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM4) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR9) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM4) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR10) \
        || ((trigger) == HAL_TIM_TRIG_ITR11)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM4) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_TI1F_ED) \
        || ((trigger) == HAL_TIM_TRIG_TI1FP1) \
        || ((trigger) == HAL_TIM_TRIG_TI2FP2)) \
       && (IS_TIM_SLAVE_INSTANCE((instance)))) \
   || (((trigger) == HAL_TIM_TRIG_ETRF) \
       && (IS_TIM_ETR_INSTANCE((instance)))))
#elif defined(TIM3)
#define IS_TIM_TRIG_SEL(instance, trigger) \
  (((((trigger) == HAL_TIM_TRIG_ITR0) \
     || ((trigger) == HAL_TIM_TRIG_ITR1)) \
    && (((instance) == TIM2) || ((instance) == TIM3) \
        || ((instance) == TIM5) || ((instance) == TIM8) \
        || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR2) \
       && (((instance) == TIM1) || ((instance) == TIM3) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR3) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR5) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR6) \
        || ((trigger) == HAL_TIM_TRIG_ITR7)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM5) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR8) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR9) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR10) \
        || ((trigger) == HAL_TIM_TRIG_ITR11)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM3) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_TI1F_ED) \
        || ((trigger) == HAL_TIM_TRIG_TI1FP1) \
        || ((trigger) == HAL_TIM_TRIG_TI2FP2)) \
       && (IS_TIM_SLAVE_INSTANCE((instance)))) \
   || (((trigger) == HAL_TIM_TRIG_ETRF) \
       && (IS_TIM_ETR_INSTANCE((instance)))))
#elif defined(TIM5)
#define IS_TIM_TRIG_SEL(instance, trigger) \
  (((((trigger) == HAL_TIM_TRIG_ITR0) \
     || ((trigger) == HAL_TIM_TRIG_ITR1)) \
    && (((instance) == TIM2) || ((instance) == TIM5) \
        || ((instance) == TIM8) || ((instance) == TIM12) \
        || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR2) \
       && (((instance) == TIM1) || ((instance) == TIM5) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR5) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM8) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR6) \
        || ((trigger) == HAL_TIM_TRIG_ITR7)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM5) || ((instance) == TIM12) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR8) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR9) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR10) \
        || ((trigger) == HAL_TIM_TRIG_ITR11)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM5) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_TI1F_ED) \
        || ((trigger) == HAL_TIM_TRIG_TI1FP1) \
        || ((trigger) == HAL_TIM_TRIG_TI2FP2)) \
       && (IS_TIM_SLAVE_INSTANCE((instance)))) \
   || (((trigger) == HAL_TIM_TRIG_ETRF) \
       && (IS_TIM_ETR_INSTANCE((instance)))))
#else
#define IS_TIM_TRIG_SEL(instance, trigger) \
  (((((trigger) == HAL_TIM_TRIG_ITR0) \
     || ((trigger) == HAL_TIM_TRIG_ITR1)) \
    && (((instance) == TIM2) || ((instance) == TIM8) \
        || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR2) \
       && (((instance) == TIM1) || ((instance) == TIM8) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || ((((trigger) == HAL_TIM_TRIG_ITR6) \
        || ((trigger) == HAL_TIM_TRIG_ITR7)) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM12) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR8) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM8) || ((instance) == TIM15))) \
   || (((trigger) == HAL_TIM_TRIG_ITR9) \
       && (((instance) == TIM1) || ((instance) == TIM2) \
           || ((instance) == TIM8) || ((instance) == TIM12))) \
   || ((((trigger) == HAL_TIM_TRIG_TI1F_ED) \
        || ((trigger) == HAL_TIM_TRIG_TI1FP1) \
        || ((trigger) == HAL_TIM_TRIG_TI2FP2)) \
       && (IS_TIM_SLAVE_INSTANCE((instance)))) \
   || (((trigger) == HAL_TIM_TRIG_ETRF) \
       && (IS_TIM_ETR_INSTANCE((instance)))))
#endif /* TIM20 */

/**
  * @brief  Check if the timer instance supports external clock mode 1.
  * @param  instance  TIM instance.
  * @retval SET (external clock mode 1 supported) or RESET (external clock mode 1 not supported).
  */
#define IS_TIM_EXTERNAL_CLOCK_MODE1_INSTANCE(instance) \
  IS_TIM_SLAVE_INSTANCE((instance))

/**
  * @brief  Check if the timer instance supports external clock mode 2.
  * @param  instance  TIM instance.
  * @retval SET (external clock mode 2 supported) or RESET (external clock mode 2 not supported).
  */
#define IS_TIM_EXTERNAL_CLOCK_MODE2_INSTANCE(instance) \
  IS_TIM_ETR_INSTANCE((instance))

/**
  * @brief  Check the compatibility of a trigger for a slave mode.
  * @param  mode     The slave mode (@ref hal_tim_slave_mode_t).
  * @param  trigger  The trigger selection (@ref hal_tim_trig_sel_t).
  * @note   For gated mode or combined gated + reset mode, the trigger
  *         must not be a pulse.
  * @retval SET (trigger is valid) or RESET (trigger is invalid).
  */
#define IS_TIM_SLAVE_MODE_TRIGGER_VALID(mode, trigger) \
  (((((mode) == HAL_TIM_SLAVE_GATED) || ((mode) == HAL_TIM_SLAVE_COMBINED_GATED_RESET))  \
    && ((trigger) == HAL_TIM_TRIG_TI1F_ED)) ? 0 : 1)

/**
  * @brief  Check the validity of an internal output channel parameter.
  * @param  channel  The channel to check (@ref hal_tim_channel_t).
  * @retval SET (internal channel) or RESET (otherwise).
  */
#define IS_TIM_OC_INTERNAL_CHANNEL(channel) (((channel) == HAL_TIM_CHANNEL_5) \
                                             ||((channel) == HAL_TIM_CHANNEL_6) \
                                             ||((channel) == HAL_TIM_CHANNEL_7))

/**
  * @brief  Check the validity of an output channel parameter.
  * @param  instance  TIM instance.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @retval SET (output channel is valid) or RESET (output channel is invalid).
  */
#define IS_TIM_OC_CHANNEL(instance, channel) \
  ((((channel) == HAL_TIM_CHANNEL_1) && IS_TIM_CC1_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_2) && IS_TIM_CC2_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_3) && IS_TIM_CC3_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_4) && IS_TIM_CC4_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_5) && IS_TIM_CC5_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_6) && IS_TIM_CC6_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_7) && IS_TIM_CC7_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_1N) && IS_TIM_CC1N_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_2N) && IS_TIM_CC2N_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_3N) && IS_TIM_CC3N_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_4N) && IS_TIM_CC4N_INSTANCE((instance))))

/**
  * @brief  Check the validity of an output compare unit parameter.
  * @param  instance      TIM instance.
  * @param  compare_unit  The compare unit to check (@ref hal_tim_oc_compare_unit_t).
  * @retval SET (compare unit is valid) or RESET (compare unit is invalid).
  */
#define IS_TIM_OC_COMPARE_UNIT(instance, compare_unit) \
  ((((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_1) && IS_TIM_CC1_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_2) && IS_TIM_CC2_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_3) && IS_TIM_CC3_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_4) && IS_TIM_CC4_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_5) && IS_TIM_CC5_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_6) && IS_TIM_CC6_INSTANCE((instance))) \
   || (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_7) && IS_TIM_CC7_INSTANCE((instance))))

/**
  * @brief  Check the value to store in the Capture/Compare Register (CCRx).
  * @param  instance  TIM instance.
  * @param  pulse     Pulse value.
  * @note   For a 32-bit counter, the constraint is only to have a positive value. \n
  *         Otherwise, the maximum value is 0xFFFEF when dithering is enabled, and
  *         0xFFFF when dithering is not enabled.
  * @retval SET (pulse value is valid) or RESET (pulse value is invalid).
  */
#define IS_TIM_OC_PULSE(instance, pulse) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   ((pulse) <= 0x000FFFEFU) : 1U)

/**
  * @brief  Check the value to store in the Capture/Compare Register (CCRx) when dithering is enabled.
  * @param  instance  TIM instance.
  * @param  pulse     Pulse value.
  * @retval SET (pulse value is valid) or RESET (pulse value is invalid).
  */
#define IS_TIM_OC_PULSE_WITH_DITHERING(instance, pulse) \
  ((IS_TIM_32B_COUNTER_INSTANCE((instance)) == 0U) ?  \
   ((pulse) <= 0x0000FFFEU) : ((pulse) <= 0x0FFFFFFEU))

/**
  * @brief  Check the validity of the output channel unit mode.
  * @param  compare_unit  The channel unit to check.
  * @param  mode          The output channel mode to check (@ref hal_tim_oc_mode_t).
  * @note   HAL_TIM_OC_PULSE_ON_COMPARE and HAL_TIM_OC_DIRECTION_OUTPUT are
  *         available only for channel units 3 and 4.
  * @retval SET (output channel mode is valid) or RESET (output channel mode is invalid).
  */
#define IS_TIM_OC_MODE(compare_unit, mode) \
  (((mode) == HAL_TIM_OC_FROZEN)                            \
   ||((mode) == HAL_TIM_OC_ACTIVE_ON_MATCH)                 \
   ||((mode) == HAL_TIM_OC_INACTIVE_ON_MATCH)               \
   ||((mode) == HAL_TIM_OC_TOGGLE)                          \
   ||((mode) == HAL_TIM_OC_PWM1)                            \
   ||((mode) == HAL_TIM_OC_PWM2)                            \
   ||((mode) == HAL_TIM_OC_FORCED_ACTIVE)                   \
   ||((mode) == HAL_TIM_OC_FORCED_INACTIVE)                 \
   ||((mode) == HAL_TIM_OC_RETRIGERRABLE_OPM1)              \
   ||((mode) == HAL_TIM_OC_RETRIGERRABLE_OPM2)              \
   ||((mode) == HAL_TIM_OC_COMBINED_PWM1)                   \
   ||((mode) == HAL_TIM_OC_COMBINED_PWM2)                   \
   ||((mode) == HAL_TIM_OC_COMBINED_PWM3)                   \
   ||((mode) == HAL_TIM_OC_COMBINED_PWM4)                   \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM1)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM2)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM3)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM4)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM5)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM6)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM7)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM8)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM9)                 \
   ||((mode) == HAL_TIM_OC_ASYMMETRIC_PWM10)                \
   ||(((mode) == HAL_TIM_OC_PULSE_ON_COMPARE)               \
      && (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_3)     \
          ||((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_4))) \
   ||(((mode) == HAL_TIM_OC_DIRECTION_OUTPUT)               \
      && (((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_3)     \
          ||((compare_unit) == HAL_TIM_OC_COMPARE_UNIT_4))))

/**
  * @brief  Check the validity of the output channel polarity.
  * @param  polarity  The output channel polarity to check (@ref hal_tim_oc_polarity_t).
  * @retval SET (output channel polarity is valid) or RESET (output channel polarity is invalid).
  */
#define IS_TIM_OC_POLARITY(polarity) (((polarity) == HAL_TIM_OC_HIGH) \
                                      ||((polarity) == HAL_TIM_OC_LOW))

/**
  * @brief  Check the validity of the output channel idle state.
  * @param  state  The output channel idle state to check (@ref hal_tim_oc_idle_state_t).
  * @retval SET (output channel idle state is valid) or RESET (output channel idle state is invalid).
  */
#define IS_TIM_OC_IDLE_STATE(state) (((state) == HAL_TIM_OC_IDLE_STATE_RESET) \
                                     ||((state) == HAL_TIM_OC_IDLE_STATE_SET))

/**
  * @brief  Check the validity of the output channel override state.
  * @param  state  The output channel override state to check (@ref hal_tim_oc_override_state_t).
  * @retval SET (output channel override state is valid) or RESET (output channel override state is invalid).
  */
#define IS_TIM_OC_OVERRIDE_STATE(state) (((state) == HAL_TIM_OC_OVERRIDE_RESET) \
                                         ||((state) == HAL_TIM_OC_OVERRIDE_SET))

/**
  * @brief  Check the validity of the output channel break mode.
  * @param  break_mode  The output channel break mode to check (@ref hal_tim_oc_break_mode_t).
  * @retval SET (output channel break mode is valid) or RESET (output channel break mode is invalid).
  */
#define IS_TIM_OC_BREAK_MODE(break_mode) (((break_mode) == HAL_TIM_OC_BREAKMODE_IMMEDIATE) \
                                          ||((break_mode) == HAL_TIM_OC_BREAKMODE_DELAY1)    \
                                          ||((break_mode) == HAL_TIM_OC_BREAKMODE_DELAY2))

/**
  * @brief  Check the validity of the pulse prescaler for the pulse on compare.
  * @param  prescaler  The pulse prescaler to check (@ref hal_tim_pulse_prescaler_t).
  * @retval SET (pulse prescaler is valid) or RESET (pulse prescaler is invalid).
  */
#define IS_TIM_PULSE_PRESCALER(prescaler) (((prescaler) == HAL_TIM_PULSE_DIV1)  \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV2)  \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV4)  \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV8)  \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV16) \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV32) \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV64) \
                                           ||((prescaler) == HAL_TIM_PULSE_DIV128))

/**
  * @brief  Check the validity of the pulse width for the pulse on compare.
  * @param  pulse_width  The pulse width to check.
  * @retval SET (pulse width is valid) or RESET (pulse width is invalid).
  */
#define IS_TIM_OC_PULSE_WIDTH(pulse_width) \
  (((pulse_width) > 0U) && ((pulse_width) <= 0xFFU))

/**
  * @brief  Check the validity of the dithering pattern.
  * @param  pattern  The dithering pattern to check (@ref hal_tim_dithering_pattern_t).
  * @retval SET (dithering pattern is valid) or RESET (dithering pattern is invalid).
  */
#define IS_TIM_DITHERING_PATTERN(pattern) \
  (((pattern) == HAL_TIM_DITHERING_0_16)  \
   ||((pattern) == HAL_TIM_DITHERING_1_16)  \
   ||((pattern) == HAL_TIM_DITHERING_2_16)  \
   ||((pattern) == HAL_TIM_DITHERING_3_16)  \
   ||((pattern) == HAL_TIM_DITHERING_4_16)  \
   ||((pattern) == HAL_TIM_DITHERING_5_16)  \
   ||((pattern) == HAL_TIM_DITHERING_6_16)  \
   ||((pattern) == HAL_TIM_DITHERING_7_16)  \
   ||((pattern) == HAL_TIM_DITHERING_8_16)  \
   ||((pattern) == HAL_TIM_DITHERING_9_16)  \
   ||((pattern) == HAL_TIM_DITHERING_10_16) \
   ||((pattern) == HAL_TIM_DITHERING_11_16) \
   ||((pattern) == HAL_TIM_DITHERING_12_16) \
   ||((pattern) == HAL_TIM_DITHERING_13_16) \
   ||((pattern) == HAL_TIM_DITHERING_14_16) \
   ||((pattern) == HAL_TIM_DITHERING_15_16))

/**
  * @brief  Check the validity of an input channel parameter.
  * @param  instance  TIM instance.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @retval SET (input channel is valid) or RESET (input channel is invalid).
  */
#define IS_TIM_IC_CHANNEL(instance, channel) \
  ((((channel) == HAL_TIM_CHANNEL_1) && IS_TIM_CC1_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_2) && IS_TIM_CC2_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_3) && IS_TIM_CC3_INSTANCE((instance))) \
   || (((channel) == HAL_TIM_CHANNEL_4) && IS_TIM_CC4_INSTANCE((instance))))

/**
  * @brief  Check the validity of an input capture unit parameter.
  * @param  instance      TIM instance.
  * @param  capture_unit  The capture unit to check (@ref hal_tim_ic_capture_unit_t).
  * @retval SET (input capture unit is valid) or RESET (input capture unit is invalid).
  */
#define IS_TIM_IC_CAPTURE_UNIT(instance, capture_unit) \
  ((((capture_unit) == HAL_TIM_IC_CAPTURE_UNIT_1) && IS_TIM_CC1_INSTANCE((instance))) \
   || (((capture_unit) == HAL_TIM_IC_CAPTURE_UNIT_2) && IS_TIM_CC2_INSTANCE((instance))) \
   || (((capture_unit) == HAL_TIM_IC_CAPTURE_UNIT_3) && IS_TIM_CC3_INSTANCE((instance))) \
   || (((capture_unit) == HAL_TIM_IC_CAPTURE_UNIT_4) && IS_TIM_CC4_INSTANCE((instance))))

/**
  * @brief  Check the validity of the input channel polarity.
  * @param  polarity  The input channel polarity to check (@ref hal_tim_ic_polarity_t).
  * @retval SET (input channel polarity is valid) or RESET (input channel polarity is invalid).
  */
#define IS_TIM_IC_POLARITY(polarity)  \
  (((polarity) == HAL_TIM_IC_RISING)  \
   ||((polarity) == HAL_TIM_IC_FALLING) \
   ||((polarity) == HAL_TIM_IC_RISING_FALLING))

/**
  * @brief  Check the validity of the input capture unit source.
  * @param  src  The input capture unit source to check (@ref hal_tim_ic_capture_unit_src_t).
  * @retval SET (input capture unit source is valid) or RESET (input capture unit source is invalid).
  */
#define IS_TIM_IC_CAPTURE_UNIT_SRC(src) \
  (((src) == HAL_TIM_IC_DIRECT)                  \
   ||((src) == HAL_TIM_IC_INDIRECT_RISING)         \
   ||((src) == HAL_TIM_IC_INDIRECT_FALLING)        \
   ||((src) == HAL_TIM_IC_INDIRECT_RISING_FALLING) \
   ||((src) == HAL_TIM_IC_TRC))

/**
  * @brief  Check the validity of the input capture unit prescaler.
  * @param  prescaler  The input capture unit prescaler to check (@ref hal_tim_ic_capture_unit_prescaler_t).
  * @retval SET (input capture unit prescaler is valid) or RESET (input capture unit prescaler is invalid
  */
#define IS_TIM_IC_CAPTURE_UNIT_PRESCALER(prescaler) \
  (((prescaler) == HAL_TIM_IC_DIV1) \
   ||((prescaler) == HAL_TIM_IC_DIV2) \
   ||((prescaler) == HAL_TIM_IC_DIV4) \
   ||((prescaler) == HAL_TIM_IC_DIV8))

/**
  * @brief  Check the validity of the XOR gate position.
  * @param  pos  The XOR gate position to check (@ref hal_tim_ic_xor_gate_position_t).
  * @retval SET (XOR gate position is valid) or RESET (XOR gate position is invalid).
  */
#define IS_TIM_XOR_GATE_POSITION(pos) (((pos) == HAL_TIM_IC_XOR_GATE_POS_DIRECT) \
                                       ||((pos) == HAL_TIM_IC_XOR_GATE_POS_FILTERED))

/**
  * @brief  Check the validity of the XOR gate channel.
  * @param  instance  TIM instance.
  * @param  channel   The XOR gate channel to check (@ref hal_tim_channel_t).
  * @note   Channels 1, 2, and 3 can be used as XOR gate channels, but not channel 4.
  * @retval SET (XOR gate channel is valid) or RESET (XOR gate channel is invalid).
  */
#define IS_TIM_XOR_GATE_CHANNEL(instance, channel) ((IS_TIM_IC_CHANNEL(instance, channel)) \
                                                    && (channel != HAL_TIM_CHANNEL_4))

/**
  * @brief  Check the validity of the encoder index direction.
  * @param  dir  The encoder index direction to check (@ref hal_tim_encoder_index_dir_t).
  * @retval SET (encoder index direction is valid) or RESET (encoder index direction is invalid).
  */
#define IS_TIM_ENCODER_INDEX_DIR(dir) (((dir) == HAL_TIM_ENCODER_INDEX_UP_DOWN) \
                                       ||((dir) == HAL_TIM_ENCODER_INDEX_UP)      \
                                       ||((dir) == HAL_TIM_ENCODER_INDEX_DOWN))

/**
  * @brief  Check the validity of the encoder index blanking mode.
  * @param  blanking  The encoder index blanking mode to check (@ref hal_tim_encoder_index_blank_mode_t).
  * @retval SET (encoder index blanking mode is valid) or RESET (encoder index blanking mode is invalid).
  */
#define IS_TIM_ENCODER_INDEX_BLANK_MODE(blanking) \
  (((blanking) == HAL_TIM_ENCODER_INDEX_BLANK_ALWAYS) \
   ||((blanking) == HAL_TIM_ENCODER_INDEX_BLANK_TI3)  \
   ||((blanking) == HAL_TIM_ENCODER_INDEX_BLANK_TI4))

/**
  * @brief  Check the validity of the encoder index position selection.
  * @param  pos  The encoder index position selection to check (@ref hal_tim_encoder_index_pos_sel_t).
  * @retval SET (encoder index position selection is valid) or RESET (encoder index position selection is invalid).
  */
#define IS_TIM_ENCODER_INDEX_POS_SEL(pos) \
  (((pos) == HAL_TIM_ENCODER_INDEX_POS_DOWN_DOWN) \
   ||((pos) == HAL_TIM_ENCODER_INDEX_POS_DOWN_UP) \
   ||((pos) == HAL_TIM_ENCODER_INDEX_POS_UP_DOWN) \
   ||((pos) == HAL_TIM_ENCODER_INDEX_POS_UP_UP)   \
   ||((pos) == HAL_TIM_ENCODER_INDEX_POS_DOWN)    \
   ||((pos) == HAL_TIM_ENCODER_INDEX_POS_UP))

/**
  * @brief  Check the validity of the encoder index selection.
  * @param  sel  The encoder index selection to check (@ref hal_tim_encoder_index_sel_t).
  * @retval SET (encoder index selection is valid) or RESET (encoder index selection is invalid).
  */
#define IS_TIM_ENCODER_INDEX_SEL(sel) \
  (((sel) == HAL_TIM_ENCODER_INDEX_ALL)  \
   ||((sel) == HAL_TIM_ENCODER_INDEX_FIRST_ONLY))

/**
  * @brief  Check the validity of the external trigger polarity.
  * @param  polarity  The external trigger polarity to check (@ref hal_tim_ext_trig_polarity_t).
  * @retval SET (external trigger polarity is valid) or RESET (external trigger polarity is invalid).
  */
#define IS_TIM_EXT_TRIG_POLARITY(polarity) \
  (((polarity) == HAL_TIM_EXT_TRIG_NONINVERTED) \
   ||((polarity) == HAL_TIM_EXT_TRIG_INVERTED))

/**
  * @brief  Check the validity of the external trigger prescaler.
  * @param  prescaler  The external trigger prescaler to check (@ref hal_tim_ext_trig_prescaler_t).
  * @retval SET (external trigger prescaler is valid) or RESET (external trigger prescaler is invalid).
  */
#define IS_TIM_EXT_TRIG_PRESCALER(prescaler) \
  (((prescaler) == HAL_TIM_EXT_TRIG_DIV1)  \
   ||((prescaler) == HAL_TIM_EXT_TRIG_DIV2)  \
   ||((prescaler) == HAL_TIM_EXT_TRIG_DIV4)  \
   ||((prescaler) == HAL_TIM_EXT_TRIG_DIV8))

/**
  * @brief  Check the validity of the external trigger synchronous prescaler.
  * @param  sync_prescaler  The external trigger synchronous prescaler to check
  *                         (@ref hal_tim_ext_trig_sync_prescaler_t).
  * @retval SET (external trigger synchronous prescaler is valid) or
  *         RESET (external trigger synchronous prescaler is invalid).
  */
#define IS_TIM_EXT_TRIG_SYNC_PRESCALER(sync_prescaler) \
  (((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV1)    \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV2)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV3)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV4)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV5)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV6)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV7)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV8)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV9)  \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV10) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV11) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV12) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV13) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV14) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV15) \
   ||((sync_prescaler) == HAL_TIM_EXT_TRIG_SYNC_DIV16))

/**
  * @brief  Check the validity of the TIM1 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM1 external trigger source.
  */
#if defined(COMP2)
#define IS_TIM1_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM1_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_COMP2_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD3))
#else
#define IS_TIM1_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM1_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD3))
#endif /* PLAY1 && COMP3 */

/**
  * @brief  Check the validity of the TIM2 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM2 external trigger source.
  */
#if defined(TIM4)
#define IS_TIM2_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM2_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD3) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_LSE) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_MCO1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_TIM3_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_TIM4_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_TIM5_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ETH1_PTP_PPS_OUT))
#elif defined(TIM5)
#define IS_TIM2_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM2_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD3) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_LSE) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_MCO1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_TIM5_ETR))
#elif defined(COMP2)
#define IS_TIM2_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM2_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_COMP2_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD3) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_LSE) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM2_MCO1))
#endif /* PLAY1 */

#if defined(TIM3)
/**
  * @brief  Check the validity of the TIM3 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM3 external trigger source.
  */
#if defined(TIM4)
#define IS_TIM3_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM3_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM3_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM3_TIM2_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM3_TIM4_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM3_TIM5_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM3_ETH1_PTP_PPS_OUT))
#endif /* PLAY1 */
#endif /* TIM3 */

#if defined(TIM4)
/**
  * @brief  Check the validity of the TIM4 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM4 external trigger source.
  */
#define IS_TIM4_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM4_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD3) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_TIM2_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_TIM3_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM4_TIM5_ETR))
#endif /* TIM4 */

#if defined(TIM5)
/**
  * @brief  Check the validity of the TIM5 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM5 external trigger source.
  */
#if defined(TIM4)
#define IS_TIM5_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM5_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM2_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM3_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM4_ETR))
#elif defined(TIM3)
#define IS_TIM5_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM5_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM2_ETR) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM3_ETR))
#else
#define IS_TIM5_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM5_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM5_TIM2_ETR))
#endif /* COMP3 */
#endif /* TIM5 */

/**
  * @brief  Check the validity of the TIM8 external trigger sources.
  * @param  src  The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of TIM8 external trigger source.
  */
#if defined(ADC3)
#define IS_TIM8_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM8_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD3) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD3))
#elif defined(ADC2)
#define IS_TIM8_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM8_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD3))
#elif defined(COMP2)
#define IS_TIM8_EXT_TRG_SRC(src) \
  (((src) == HAL_TIM_EXT_TRIG_TIM8_GPIO) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_COMP1_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_COMP2_OUT) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD1) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD2) \
   || ((src) == HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD3))
#endif /* PLAY1 */


/**
  * @brief  Check the validity of the external trigger sources.
  * @param  instance  TIM instance.
  * @param  src       The external trigger source to check (@ref hal_tim_ext_trig_src_t).
  * @return Validity of external trigger source.
  */
#if defined(TIM4)
#define IS_TIM_EXT_TRIG_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_EXT_TRG_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_EXT_TRG_SRC((src))) \
   || (((instance) == TIM3) && IS_TIM3_EXT_TRG_SRC((src))) \
   || (((instance) == TIM4) && IS_TIM4_EXT_TRG_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_EXT_TRG_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_EXT_TRG_SRC((src))))
#elif defined(TIM3)
#define IS_TIM_EXT_TRIG_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_EXT_TRG_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_EXT_TRG_SRC((src))) \
   || (((instance) == TIM3) && IS_TIM3_EXT_TRG_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_EXT_TRG_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_EXT_TRG_SRC((src))))
#elif defined(TIM5)
#define IS_TIM_EXT_TRIG_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_EXT_TRG_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_EXT_TRG_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_EXT_TRG_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_EXT_TRG_SRC((src))))
#else
#define IS_TIM_EXT_TRIG_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_EXT_TRG_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_EXT_TRG_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_EXT_TRG_SRC((src))))
#endif /* TIM20 */

/**
  * @brief  Check the validity of the TIM1 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(COMP2)
#define IS_TIM1_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM1_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM1_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM1_TI1_COMP2_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI4_GPIO)))
#else
#define IS_TIM1_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM1_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM1_TI1_COMP1_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM1_TI4_GPIO)))
#endif /* PLAY1 */

/**
  * @brief  Check the validity of the TIM2 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(TIM5) && defined(FDCAN2)
#define IS_TIM2_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM2_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_ETH1_PTP_PPS_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSI) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_TIM5_CH1) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_FDCAN1_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_HSE_RTC) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO1) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO2) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_FDCAN1_TXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI3_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI3_FDCAN2_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI4_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_FDCAN2_TXEOF_EVT))))
#elif defined(TIM5) && defined(FDCAN1)
#define IS_TIM2_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM2_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSI) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_TIM5_CH1) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_FDCAN1_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_HSE_RTC) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO1) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO2) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_FDCAN1_TXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM2_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI4_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT))))
#elif defined(TIM5)
#define IS_TIM2_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM2_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSI) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_TIM5_CH1))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_HSE_RTC) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO1) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO2))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM2_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI4_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT))))
#elif defined(FDCAN1)
#define IS_TIM2_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM2_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP2_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSI) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_FDCAN1_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_HSE_RTC) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO1) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO2) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_FDCAN1_TXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI3_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI3_FDCAN2_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI4_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP2_OUT) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_FDCAN2_TXEOF_EVT))))
#else
#define IS_TIM2_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM2_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_COMP2_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSI) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_HSE_RTC) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO1) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI2_MCO2))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM2_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && (((src) == HAL_TIM_INPUT_TIM2_TI4_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT) \
           || ((src) == HAL_TIM_INPUT_TIM2_TI4_COMP2_OUT))))
#endif /* PLAY1 */

#if defined(TIM3)
/**
  * @brief  Check the validity of the TIM3 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(FDCAN2)
#define IS_TIM3_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM3_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM3_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM3_TI1_ETH1_PTP_PPS_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM3_TI1_FDCAN2_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM3_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM3_TI2_FDCAN2_TXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM3_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM3_TI4_GPIO)))
#else
#define IS_TIM3_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM3_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM3_TI1_COMP1_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM3_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM3_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM3_TI4_GPIO)))
#endif /* PLAY1 */
#endif /* TIM3 */

#if defined(TIM4)
/**
  * @brief  Check the validity of the TIM4 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_TIM4_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM4_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM4_TI1_COMP1_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM4_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM4_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM4_TI4_GPIO)))
#endif /* TIM4 */

#if defined(TIM5)
/**
  * @brief  Check the validity of the TIM5 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_TIM5_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM5_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM5_TI1_COMP1_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM5_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM5_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM5_TI4_GPIO)))
#endif /* TIM5 */

/**
  * @brief  Check the validity of the TIM8 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(COMP2)
#define IS_TIM8_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM8_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM8_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM8_TI1_COMP2_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI4_GPIO)))
#else
#define IS_TIM8_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM8_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM8_TI1_COMP1_OUT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI2_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_3) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI3_GPIO)) \
   || (((channel) == HAL_TIM_CHANNEL_4) \
       && ((src) == HAL_TIM_INPUT_TIM8_TI4_GPIO)))
#endif /* PLAY1 */

/**
  * @brief  Check the validity of the TIM12 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(COMP2)
#define IS_TIM12_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM12_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_COMP2_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_MCO1) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_MCO2) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_HSE_RTC) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_I3C1_IBI_ACK))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM12_TI2_GPIO)))
#else
#define IS_TIM12_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM12_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_MCO1) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_MCO2) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_HSE_RTC) \
        || ((src) == HAL_TIM_INPUT_TIM12_TI1_I3C1_IBI_ACK))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM12_TI2_GPIO)))
#endif /* PLAY1 */

/**
  * @brief  Check the validity of the TIM15 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(COMP2) && defined(FDCAN2)
#define IS_TIM15_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM15_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP2_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_FDCAN2_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM15_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM15_TI2_FDCAN2_TXEOF_EVT))))
#elif defined(COMP2)
#define IS_TIM15_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM15_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP2_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_LSE))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM15_TI2_GPIO)))
#elif defined(FDCAN2)
#define IS_TIM15_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM15_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_LSE) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_FDCAN2_RXEOF_EVT))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && (((src) == HAL_TIM_INPUT_TIM15_TI2_GPIO) \
           || ((src) == HAL_TIM_INPUT_TIM15_TI2_FDCAN2_TXEOF_EVT))))
#else
#define IS_TIM15_CHANNEL_SRC(channel, src) \
  ((((channel) == HAL_TIM_CHANNEL_1) \
    && (((src) == HAL_TIM_INPUT_TIM15_TI1_GPIO) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_COMP1_OUT) \
        || ((src) == HAL_TIM_INPUT_TIM15_TI1_LSE))) \
   || (((channel) == HAL_TIM_CHANNEL_2) \
       && ((src) == HAL_TIM_INPUT_TIM15_TI2_GPIO)))
#endif /* PLAY1 */

#if defined(TIM16)
/**
  * @brief  Check the validity of the TIM16 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_TIM16_CHANNEL_SRC(channel, src) \
  (((channel) == HAL_TIM_CHANNEL_1) \
   && (((src) == HAL_TIM_INPUT_TIM16_TI1_GPIO) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_COMP1_OUT) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_LSI) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_LSE) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_RTC_WUT_TRG) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_MCO1) \
       || ((src) == HAL_TIM_INPUT_TIM16_TI1_MCO2)))

/**
  * @brief  Check the validity of the TIM17 channel sources.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#define IS_TIM17_CHANNEL_SRC(channel, src) \
  (((channel) == HAL_TIM_CHANNEL_1) \
   && (((src) == HAL_TIM_INPUT_TIM17_TI1_GPIO) \
       || ((src) == HAL_TIM_INPUT_TIM17_TI1_COMP1_OUT) \
       || ((src) == HAL_TIM_INPUT_TIM17_TI1_HSE_RTC) \
       || ((src) == HAL_TIM_INPUT_TIM17_TI1_MCO1) \
       || ((src) == HAL_TIM_INPUT_TIM17_TI1_MCO2) \
       || ((src) == HAL_TIM_INPUT_TIM17_TI1_I3C1_IBI_ACK)))
#endif /* TIM16 */


/**
  * @brief  Check the validity of the channel sources.
  * @param  instance  TIM instance.
  * @param  channel   The channel to check (@ref hal_tim_channel_t).
  * @param  src       The channel source to check (@ref hal_tim_channel_src_t).
  * @retval SET (channel source is valid) or RESET (channel source is invalid).
  */
#if defined(TIM4)
#define IS_TIM_CHANNEL_SRC(instance, channel, src) \
  ((((instance) == TIM1) && IS_TIM1_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM2) && IS_TIM2_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM3) && IS_TIM3_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM4) && IS_TIM4_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM5) && IS_TIM5_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM8) && IS_TIM8_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM12) && IS_TIM12_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM15) && IS_TIM15_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM16) && IS_TIM16_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM17) && IS_TIM17_CHANNEL_SRC((channel), (src))))
#elif defined(TIM3)
#define IS_TIM_CHANNEL_SRC(instance, channel, src) \
  ((((instance) == TIM1) && IS_TIM1_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM2) && IS_TIM2_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM3) && IS_TIM3_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM5) && IS_TIM5_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM8) && IS_TIM8_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM12) && IS_TIM12_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM15) && IS_TIM15_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM16) && IS_TIM16_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM17) && IS_TIM17_CHANNEL_SRC((channel), (src))))
#elif defined(TIM5)
#define IS_TIM_CHANNEL_SRC(instance, channel, src) \
  ((((instance) == TIM1) && IS_TIM1_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM2) && IS_TIM2_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM5) && IS_TIM5_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM8) && IS_TIM8_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM12) && IS_TIM12_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM15) && IS_TIM15_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM16) && IS_TIM16_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM17) && IS_TIM17_CHANNEL_SRC((channel), (src))))
#else
#define IS_TIM_CHANNEL_SRC(instance, channel, src) \
  ((((instance) == TIM1) && IS_TIM1_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM2) && IS_TIM2_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM8) && IS_TIM8_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM12) && IS_TIM12_CHANNEL_SRC((channel), (src))) \
   || (((instance) == TIM15) && IS_TIM15_CHANNEL_SRC((channel), (src))))
#endif /* TIM20 && TIM4 */

/**
  * @brief  Check the validity of the slave mode.
  * @param  mode  The slave mode to check (@ref hal_tim_slave_mode_t).
  * @retval SET (slave mode is valid) or RESET (slave mode is invalid).
  */
#define IS_TIM_SLAVE_MODE(mode) \
  (((mode) == HAL_TIM_SLAVE_DISABLED)                 \
   ||((mode) == HAL_TIM_SLAVE_RESET)                  \
   ||((mode) == HAL_TIM_SLAVE_GATED)                  \
   ||((mode) == HAL_TIM_SLAVE_TRIGGER)                \
   ||((mode) == HAL_TIM_SLAVE_COMBINED_RESET_TRIGGER) \
   ||((mode) == HAL_TIM_SLAVE_COMBINED_GATED_RESET))

/**
  * @brief  Check the validity of the trigger output source.
  * @param  src  The trigger output source to check (@ref hal_tim_trigger_output_source_t).
  * @retval SET (trigger output source is valid) or RESET (trigger output source is invalid).
  */
#define IS_TIM_TRIGGER_OUTPUT_SOURCE(src) \
  (((src) == HAL_TIM_TRGO_RESET)    \
   ||((src) == HAL_TIM_TRGO_ENABLE) \
   ||((src) == HAL_TIM_TRGO_UPDATE) \
   ||((src) == HAL_TIM_TRGO_CC1IF)  \
   ||((src) == HAL_TIM_TRGO_OC1)    \
   ||((src) == HAL_TIM_TRGO_OC2)    \
   ||((src) == HAL_TIM_TRGO_OC3)    \
   ||((src) == HAL_TIM_TRGO_OC4)    \
   ||((src) == HAL_TIM_TRGO_ENCODER_CLK))

/**
  * @brief  Check the validity of the trigger output2 source.
  * @param  src  The trigger output2 source to check (@ref hal_tim_trigger_output2_source_t).
  * @retval SET (trigger output2 source is valid) or RESET (trigger output2 source is invalid).
  */
#define IS_TIM_TRIGGER_OUTPUT2_SOURCE(src) \
  (((src) == HAL_TIM_TRGO2_RESET)                    \
   ||((src) == HAL_TIM_TRGO2_ENABLE)                 \
   ||((src) == HAL_TIM_TRGO2_UPDATE)                 \
   ||((src) == HAL_TIM_TRGO2_CC1F)                   \
   ||((src) == HAL_TIM_TRGO2_OC1)                    \
   ||((src) == HAL_TIM_TRGO2_OC2)                    \
   ||((src) == HAL_TIM_TRGO2_OC3)                    \
   ||((src) == HAL_TIM_TRGO2_OC4)                    \
   ||((src) == HAL_TIM_TRGO2_OC5)                    \
   ||((src) == HAL_TIM_TRGO2_OC6)                    \
   ||((src) == HAL_TIM_TRGO2_OC7)                    \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC7_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC6_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC6_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC6_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC7_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC6_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC7_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_OC7_FALLING))

/**
  * @brief  Check the validity of the trigger output2 postscaler source.
  * @param  src  The trigger output2 postscaler source to check (@ref hal_tim_trigger_output2_source_t).
  * @note   The postscaler is only applicable when tim_trgo2 transfers a pulse (reset, update, compare pulse).
  * @retval SET (trigger output2 postscaler source is valid) or RESET (trigger output2 postscaler source is invalid).
  */
#define IS_TIM_TRIGGER_OUTPUT2_PSC_SOURCE(src) \
  (((src) == HAL_TIM_TRGO2_RESET)                    \
   ||((src) == HAL_TIM_TRGO2_UPDATE)                 \
   ||((src) == HAL_TIM_TRGO2_CC1F)                   \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC7_RISING_FALLING)     \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC6_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC6_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_OC7_RISING)  \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC6_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC4_RISING_OC7_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC6_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC5_RISING_OC7_FALLING) \
   ||((src) == HAL_TIM_TRGO2_OC6_RISING_OC7_FALLING))

/**
  * @brief  Check the validity of the trigger output2 postscaler.
  * @param  psc  The trigger output2 postscaler to check.
  * @retval SET (trigger output2 postscaler is valid) or RESET (trigger output2 postscaler is invalid).
  */
#define IS_TIM_TRIGGER_OUTPUT2_PSC(psc)  ((psc) <= 0x1FU)

/**
  * @brief  Check the validity of the slave mode preload source.
  * @param  src  The slave mode preload source to check (@ref hal_tim_slave_mode_preload_src_t).
  * @retval SET (slave mode preload source is valid) or RESET (slave mode preload source is invalid).
  */
#define IS_TIM_SLAVE_MODE_PRELOAD_SRC(src) (((src) == HAL_TIM_SLAVE_MODE_PRELOAD_UPDATE)  \
                                            ||((src) == HAL_TIM_SLAVE_MODE_PRELOAD_INDEX))


/**
  * @brief  Check the validity of the TIM1 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#if defined(COMP2)
#define IS_TIM1_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM1_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM1_COMP1_OUT) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM1_COMP2_OUT))
#else
#define IS_TIM1_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM1_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM1_COMP1_OUT))
#endif /* PLAY1 */

/**
  * @brief  Check the validity of the TIM2 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#if defined(COMP2)
#define IS_TIM2_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM2_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM2_COMP1_OUT) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM2_COMP2_OUT))
#else
#define IS_TIM2_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM2_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM2_COMP1_OUT))
#endif /* PLAY1 */

#if defined(TIM3)
/**
  * @brief  Check the validity of the TIM3 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#define IS_TIM3_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM3_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM3_COMP1_OUT))
#endif /* TIM3 */

#if defined(TIM4)
/**
  * @brief  Check the validity of the TIM4 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#define IS_TIM4_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM4_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM4_COMP1_OUT))
#endif /* TIM4 */

#if defined(TIM5)
/**
  * @brief  Check the validity of the TIM5 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#define IS_TIM5_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM5_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM5_COMP1_OUT))
#endif /* TIM5 */

/**
  * @brief  Check the validity of the TIM8 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#if defined(COMP2)
#define IS_TIM8_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM8_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM8_COMP1_OUT) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM8_COMP2_OUT))
#else
#define IS_TIM8_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM8_ETR) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM8_COMP1_OUT))
#endif /* PLAY1 */

/**
  * @brief  Check the validity of the TIM15 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#if defined(COMP2)
#define IS_TIM15_OCREF_CLR_SRC(src) \
  (((src) == HAL_TIM_OCREF_CLR_TIM15_COMP1_OUT) \
   || ((src) == HAL_TIM_OCREF_CLR_TIM15_COMP2_OUT))
#else
#define IS_TIM15_OCREF_CLR_SRC(src) \
  ((src) == HAL_TIM_OCREF_CLR_TIM15_COMP1_OUT)
#endif /* PLAY1 */

#if defined(TIM16)
/**
  * @brief  Check the validity of the TIM16 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#define IS_TIM16_OCREF_CLR_SRC(src) \
  ((src) == HAL_TIM_OCREF_CLR_TIM16_COMP1_OUT)

/**
  * @brief  Check the validity of the TIM17 OCref clear sources.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#define IS_TIM17_OCREF_CLR_SRC(src) \
  ((src) == HAL_TIM_OCREF_CLR_TIM17_COMP1_OUT)
#endif /* TIM16 */


/**
  * @brief  Check the validity of the OCref clear sources.
  * @param  instance  TIM instance.
  * @param  src  The OCref clear source to check (@ref hal_tim_ocref_clr_src_t).
  * @retval SET (OCref clear source is valid) or RESET (OCref clear source is invalid).
  */
#if defined(TIM4)
#define IS_TIM_OCREF_CLR_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM3) && IS_TIM3_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM4) && IS_TIM4_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM15) && IS_TIM15_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM16) && IS_TIM16_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM17) && IS_TIM17_OCREF_CLR_SRC((src))))
#elif defined(TIM3)
#define IS_TIM_OCREF_CLR_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM3) && IS_TIM3_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM15) && IS_TIM15_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM16) && IS_TIM16_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM17) && IS_TIM17_OCREF_CLR_SRC((src))))
#elif defined(TIM5)
#define IS_TIM_OCREF_CLR_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM5) && IS_TIM5_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM15) && IS_TIM15_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM16) && IS_TIM16_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM17) && IS_TIM17_OCREF_CLR_SRC((src))))
#else
#define IS_TIM_OCREF_CLR_SRC(instance, src) \
  ((((instance) == TIM1) && IS_TIM1_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM2) && IS_TIM2_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM8) && IS_TIM8_OCREF_CLR_SRC((src))) \
   || (((instance) == TIM15) && IS_TIM15_OCREF_CLR_SRC((src))))
#endif /* TIM20 && TIM4 */

/**
  * @brief  Check the validity of the DMA index.
  * @param  index  The DMA index to check (@ref hal_tim_dma_index_t).
  * @retval SET (DMA index is valid) or RESET (DMA index is invalid).
  */
#define IS_TIM_DMA_INDEX(index) (((index) == HAL_TIM_DMA_ID_UPD)   \
                                 ||((index) == HAL_TIM_DMA_ID_CC1) \
                                 ||((index) == HAL_TIM_DMA_ID_CC2) \
                                 ||((index) == HAL_TIM_DMA_ID_CC3) \
                                 ||((index) == HAL_TIM_DMA_ID_CC4) \
                                 ||((index) == HAL_TIM_DMA_ID_COM) \
                                 ||((index) == HAL_TIM_DMA_ID_TRGI))

/**
  * @brief  Check the validity of the DMA burst base address register.
  * @param  address  The DMA burst base address register to check (@ref hal_tim_dmaburst_base_addr_reg_t).
  * @retval SET (DMA burst base address register is valid) or RESET (DMA burst base address register is invalid).
  */
#define IS_TIM_DMABURST_BASE_ADDR_REG(address) \
  (((address) == HAL_TIM_DMABURST_BASE_ADDR_CR1)     \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CR2)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_SMCR)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_DIER)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_SR)    \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_EGR)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCMR1) \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCMR2) \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCER)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CNT)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_PSC)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_ARR)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_RCR)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR1)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR2)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR3)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR4)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_BDTR)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR5)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR6)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCMR3) \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_DTR2)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_ECR)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_TISEL) \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_AF1)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_AF2)   \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCR7)  \
   ||((address) == HAL_TIM_DMABURST_BASE_ADDR_CCMR4))

/**
  * @brief  Check the validity of the DMA burst source.
  * @param  instance  The TIM instance to check.
  * @param  source    The DMA burst source to check (@ref hal_tim_dmaburst_source_t).
  * @retval SET (DMA burst source is valid) or RESET (DMA burst source is invalid).
  */
#define IS_TIM_DMABURST_SRC(instance, source) \
  (((source) == HAL_TIM_DMABURST_UPD)   \
   || (((source) == HAL_TIM_DMABURST_CC1)  && IS_TIM_CC1_INSTANCE((instance)))               \
   || (((source) == HAL_TIM_DMABURST_CC2)  && IS_TIM_CC2_INSTANCE((instance)))               \
   || (((source) == HAL_TIM_DMABURST_CC3)  && IS_TIM_CC3_INSTANCE((instance)))               \
   || (((source) == HAL_TIM_DMABURST_CC4)  && IS_TIM_CC4_INSTANCE((instance)))               \
   || (((source) == HAL_TIM_DMABURST_COM)  && IS_TIM_COMMUTATION_EVENT_INSTANCE((instance))) \
   || (((source) == HAL_TIM_DMABURST_TRGI) && IS_TIM_SLAVE_INSTANCE((instance))))

/**
  * @brief  Check the validity of the DMA burst length.
  * @param  size  The DMA burst length to check (@ref hal_tim_dmaburst_length_t).
  * @retval SET (DMA burst length is valid) or RESET (DMA burst length is invalid).
  */
#define IS_TIM_DMABURST_LENGTH(size) \
  (((size) == HAL_TIM_DMABURST_1TRANSFER)     \
   ||((size) == HAL_TIM_DMABURST_2TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_3TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_4TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_5TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_6TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_7TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_8TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_9TRANSFERS)  \
   ||((size) == HAL_TIM_DMABURST_10TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_11TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_12TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_13TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_14TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_15TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_16TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_17TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_18TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_19TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_20TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_21TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_22TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_23TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_24TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_25TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_26TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_27TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_28TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_29TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_30TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_31TRANSFERS) \
   ||((size) == HAL_TIM_DMABURST_32TRANSFERS))

/**
  * @brief  Check the validity of the DMA burst direction.
  * @param  dir  The DMA burst direction to check (@ref hal_tim_dmaburst_direction_t).
  * @retval SET (DMA burst direction is valid) or RESET (DMA burst direction is invalid).
  */
#define IS_TIM_DMABURST_DIR(dir) (((dir) == HAL_TIM_DMABURST_READ) \
                                  || ((dir) == HAL_TIM_DMABURST_WRITE))

/**
  * @brief  Check the validity of the break input.
  * @param  id  The break input to check (@ref hal_tim_break_input_id_t).
  * @retval SET (break input is valid) or RESET (break input is invalid).
  */
#define IS_TIM_BREAK_INPUT_ID(id) \
  (((id) == HAL_TIM_BREAK_INPUT_1) || ((id) == HAL_TIM_BREAK_INPUT_2))

/**
  * @brief  Check the validity of the break input polarity.
  * @param  polarity  The break input polarity to check (@ref hal_tim_break_input_polarity_t).
  * @retval SET (break input polarity is valid) or RESET (break input polarity is invalid).
  */
#define IS_TIM_BREAK_INPUT_POLARITY(polarity) \
  (((polarity) == HAL_TIM_BREAK_INPUT_LOW) \
   ||((polarity) == HAL_TIM_BREAK_INPUT_HIGH))

/**
  * @brief  Check the validity of the break input mode.
  * @param  mode  The break input mode to check (@ref hal_tim_break_input_mode_t).
  * @retval SET (break input mode is valid) or RESET (break input mode is invalid).
  */
#define IS_TIM_BREAK_INPUT_MODE(mode) \
  (((mode) == HAL_TIM_BREAK_INPUT_MODE_INPUT) \
   ||((mode) == HAL_TIM_BREAK_INPUT_MODE_BIDIRECTIONAL))

/**
  * @brief  Check the validity of the break input source combination.
  * @param  instance  The TIM instance to check.
  * @param  id        The break input id to check (@ref hal_tim_break_input_id_t).
  * @param  brkinsrc  The break input source combination to check.
  * @retval SET (break input source combination is valid) or RESET (break input source combination is invalid).
  */
#if defined(TIM16)
#define IS_TIM_BREAK_INPUT_ALL_SRC(instance, id, brkinsrc) \
  (((brkinsrc) != 0U) \
   && ((((id) == HAL_TIM_BREAK_INPUT_1) \
        && ((((instance) == TIM1) && (((brkinsrc) & ~(TIM1_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM8) && (((brkinsrc) & ~(TIM8_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM15) && (((brkinsrc) & ~(TIM15_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM16) && (((brkinsrc) & ~(TIM16_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM17) && (((brkinsrc) & ~(TIM17_BRK_SOURCE_MASK)) == 0U)))) \
       || (((id) == HAL_TIM_BREAK_INPUT_2) \
           && ((((instance) == TIM1) && (((brkinsrc) & ~(TIM1_BRK2_SOURCE_MASK)) == 0U)) \
               || (((instance) == TIM8) && (((brkinsrc) & ~(TIM8_BRK2_SOURCE_MASK)) == 0U))))))
#else
#define IS_TIM_BREAK_INPUT_ALL_SRC(instance, id, brkinsrc) \
  (((brkinsrc) != 0U) \
   && ((((id) == HAL_TIM_BREAK_INPUT_1) \
        && ((((instance) == TIM1) && (((brkinsrc) & ~(TIM1_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM8) && (((brkinsrc) & ~(TIM8_BRK_SOURCE_MASK)) == 0U)) \
            || (((instance) == TIM15) && (((brkinsrc) & ~(TIM15_BRK_SOURCE_MASK)) == 0U)))) \
       || (((id) == HAL_TIM_BREAK_INPUT_2) \
           && ((((instance) == TIM1) && (((brkinsrc) & ~(TIM1_BRK2_SOURCE_MASK)) == 0U)) \
               || (((instance) == TIM8) && (((brkinsrc) & ~(TIM8_BRK2_SOURCE_MASK)) == 0U))))))
#endif /* TIM20 */

/**
  * @brief  Check that only one break input source is selected and supported.
  * @param  instance  The TIM instance to check.
  * @param  id        The break input id to check (@ref hal_tim_break_input_id_t).
  * @param  brkinsrc  The break input source to check.
  * @retval SET (only one break input source is selected and supported) or RESET (otherwise).
  */
#define IS_TIM_BREAK_INPUT_SRC(instance, id, brkinsrc) \
  ((((brkinsrc) & ((brkinsrc) - 1U)) == 0U) \
   && IS_TIM_BREAK_INPUT_ALL_SRC((instance), (id), (brkinsrc)))

/**
  * @brief  Check the validity of the break input source polarity for
  *         break input source that have a polarity configuration.
  * @param  brkinsrc  The break input source.
  * @param  polarity  The break input source polarity to check (@ref hal_tim_break_input_src_polarity_t).
  * @retval SET (break input source polarity is valid) or RESET (break input source polarity is invalid).
  */
#define IS_TIM_BREAK_INPUT_SRC_POLARITY(brkinsrc, polarity) \
  ((((brkinsrc) & TIM_BRK_BRK2_POLARITY_MASK) != 0U) \
   && (((polarity) == HAL_TIM_BREAK_INPUT_SRC_NONINVERTED) \
       || ((polarity) == HAL_TIM_BREAK_INPUT_SRC_INVERTED)))

/**
  * @brief  Check the validity of the break delay.
  * @param  break_delay  The break delay to check (@ref hal_tim_break_delay_t).
  * @retval SET (break delay is valid) or RESET (break delay is invalid).
  */
#define IS_TIM_BREAK_DELAY(break_delay)  \
  (((break_delay) == HAL_TIM_BREAK_DELAY1) \
   || ((break_delay) == HAL_TIM_BREAK_DELAY2))

/**
  * @brief  Check the validity of the break delay duration.
  * @param  delay  The break delay duration to check.
  * @retval SET (break delay duration is valid) or RESET (break delay duration is invalid).
  */
#define IS_TIM_BREAK_DELAY_DURATION(delay) ((delay) <= 0xFFU)

/**
  * @brief  Check the validity of the off state run.
  * @param  off_state_run  The off state run to check (@ref hal_tim_off_state_run_t).
  * @retval SET (off state run is valid) or RESET (off state run is invalid).
  */
#define IS_TIM_OFF_STATE_RUN(off_state_run) \
  (((off_state_run) == HAL_TIM_OFF_STATE_RUN_DISABLE) \
   || ((off_state_run) == HAL_TIM_OFF_STATE_RUN_ENABLE))

/**
  * @brief  Check the validity of the off state idle.
  * @param  off_state_idle  The off state idle to check (@ref hal_tim_off_state_idle_t).
  * @retval SET (off state idle is valid) or RESET (off state idle is invalid).
  */
#define IS_TIM_OFF_STATE_IDLE(off_state_idle) \
  (((off_state_idle) == HAL_TIM_OFF_STATE_IDLE_DISABLE) \
   || ((off_state_idle) == HAL_TIM_OFF_STATE_IDLE_ENABLE))

/**
  * @brief  Check the validity of the deadtime.
  * @param  deadtime  The deadtime to check.
  * @retval SET (deadtime is valid) or RESET (deadtime is invalid).
  */
#define IS_TIM_DEADTIME(deadtime) ((deadtime) <= 0xFFU)

/**
  * @brief  Check the validity of the lock level.
  * @param  level  The lock level to check (@ref hal_tim_lock_level_t).
  * @retval SET (lock level is valid) or RESET (lock level is invalid).
  */
#define IS_TIM_LOCK_LEVEL(level) (((level) == HAL_TIM_LOCK_OFF)   \
                                  ||((level) == HAL_TIM_LOCK_1)   \
                                  ||((level) == HAL_TIM_LOCK_2)   \
                                  ||((level) == HAL_TIM_LOCK_3))

/**
  * @brief  Check the validity of the commutation source.
  * @param  src  The commutation source to check (@ref hal_tim_commutation_src_t).
  * @retval SET (commutation source is valid) or RESET (commutation source is invalid).
  */
#define IS_TIM_COMMUTATION_SRC(src) \
  (((src) == HAL_TIM_COMMUTATION_SOFTWARE) \
   || ((src) == HAL_TIM_COMMUTATION_SOFTWARE_AND_TRIGGER))

/**
  * @brief  Check the validity of the DMA request source for the capture/compare channel.
  * @param  src  The DMA request source to check (@ref hal_tim_cc_dmareq_src_t).
  * @retval SET (DMA request source is valid) or RESET (DMA request source is invalid).
  */
#define IS_TIM_CC_DMAREQ_SRC(src) \
  (((src) == HAL_TIM_CC_DMAREQ_CC) \
   || ((src) == HAL_TIM_CC_DMAREQ_UPD))

/**
  * @brief  Check the validity of the software event.
  * @param  instance  The TIM instance to check.
  * @param  event_id  The software event to check (@ref hal_tim_sw_event_id_t).
  * @retval SET (software event is valid) or RESET (software event is invalid).
  */
#define IS_TIM_SW_EVENT_ID(instance, event_id) \
  (((event_id) == HAL_TIM_SW_EVENT_UPD) \
   || (((event_id) == HAL_TIM_SW_EVENT_CC1)  && IS_TIM_CC1_INSTANCE(instance))               \
   || (((event_id) == HAL_TIM_SW_EVENT_CC2)  && IS_TIM_CC2_INSTANCE(instance))               \
   || (((event_id) == HAL_TIM_SW_EVENT_CC3)  && IS_TIM_CC3_INSTANCE(instance))               \
   || (((event_id) == HAL_TIM_SW_EVENT_CC4)  && IS_TIM_CC4_INSTANCE(instance))               \
   || (((event_id) == HAL_TIM_SW_EVENT_COM)  && IS_TIM_COMMUTATION_EVENT_INSTANCE(instance)) \
   || (((event_id) == HAL_TIM_SW_EVENT_TRGI) && IS_TIM_SLAVE_INSTANCE(instance))             \
   || (((event_id) == HAL_TIM_SW_EVENT_BRK)  && IS_TIM_BREAK_INSTANCE(instance))             \
   || (((event_id) == HAL_TIM_SW_EVENT_BRK2) && IS_TIM_BKIN2_INSTANCE(instance)))

/**
  *  @}
  */

/* Private functions --------------------------------------------------------*/
/** @defgroup TIM_Private_Functions TIM Private Functions
  *  @{
  */

/**
  * @brief  Set the clock source of the timer's time-base unit.
  * @param  p_tim      Pointer to the handle of the TIM instance.
  * @param  p_clk_sel  Pointer to the clock selection.
  */
__STATIC_INLINE void TIM_SetClockSource(tim_t *p_tim,
                                        const hal_tim_clock_sel_t *p_clk_sel)
{
  const hal_tim_clk_src_t clock_source = p_clk_sel->clock_source;
  const hal_tim_trig_sel_t trigger = p_clk_sel->trigger;

  switch (clock_source)
  {
    case HAL_TIM_CLK_INTERNAL:
    {
      if (IS_TIM_SLAVE_INSTANCE(p_tim))
      {
        /* Disable the slave mode controller */
        LL_TIM_SetClockSource(p_tim, (uint32_t)clock_source);
      }
      break;
    }

    case HAL_TIM_CLK_EXTERNAL_MODE1:
    {
      ASSERT_DBG_PARAM(IS_TIM_EXTERNAL_CLOCK_MODE1_INSTANCE(p_tim));
      ASSERT_DBG_PARAM(IS_TIM_TRIG_SEL(p_tim, trigger));

      LL_TIM_SetClockSource(p_tim, (uint32_t)clock_source);
      /* Set the external trigger that is used as clock source */
      LL_TIM_SetTriggerInput(p_tim, (uint32_t)trigger);
      break;
    }

    case HAL_TIM_CLK_EXTERNAL_MODE2:
    {
      ASSERT_DBG_PARAM(IS_TIM_EXTERNAL_CLOCK_MODE2_INSTANCE(p_tim));

      LL_TIM_SetClockSource(p_tim, (uint32_t)clock_source);

      break;
    }

    default:
      /*
        HAL_TIM_CLK_ENCODER_X1_TI1:
        HAL_TIM_CLK_ENCODER_X1_TI2:
        HAL_TIM_CLK_ENCODER_X2_TI1:
        HAL_TIM_CLK_ENCODER_X2_TI2:
        HAL_TIM_CLK_ENCODER_X4_TI12:
        HAL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1:
        HAL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12:
        HAL_TIM_CLK_ENCODER_CLK_PLUS_X2:
        HAL_TIM_CLK_ENCODER_CLK_PLUS_X1:
        HAL_TIM_CLK_ENCODER_DIR_CLK_X2:
        HAL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12:
      */
    {
      ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));

      LL_TIM_SetClockSource(p_tim, (uint32_t)clock_source);

      break;
    }
  }
}

/**
  * @brief  Get the clock source of the timer's time-base unit.
  * @param  p_tim      Pointer to the handle of the TIM instance.
  * @param  p_clk_sel  Pointer to the clock selection.
  */
__STATIC_INLINE void TIM_GetClockSource(const tim_t *p_tim,
                                        hal_tim_clock_sel_t *p_clk_sel)
{
  hal_tim_clk_src_t clk_src = (hal_tim_clk_src_t)LL_TIM_GetClockSource(p_tim);

  p_clk_sel->clock_source = clk_src;

  if (clk_src == HAL_TIM_CLK_EXTERNAL_MODE1)
  {
    p_clk_sel->trigger = (hal_tim_trig_sel_t)LL_TIM_GetTriggerInput(p_tim);
  }
}

/**
  * @brief  Set a channel source.
  * @param  p_tim        Pointer to the handle of the TIM instance.
  * @param  channel      Channel to configure.
  * @param  channel_src  Source of the channel.
  * @note   This function calls LL_TIM_IC_SetSource(), which completely rewrites the content
  *         of the TISEL register. Hence, the TISEL register is first read and modified
  *         with the new source for the channel.
  */
__STATIC_INLINE void TIM_SetRemap(tim_t *p_tim,
                                  hal_tim_channel_t channel,
                                  hal_tim_channel_src_t channel_src)
{
  uint32_t tisel = LL_TIM_READ_REG(p_tim, TISEL);
  tisel &= ~(MASK_TISEL((uint32_t)channel));
  tisel |= (uint32_t)channel_src;

  LL_TIM_IC_SetSource(p_tim, tisel);
}

/**
  * @brief  Start the timer in interrupt mode.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  interrupts  Selection of the TIM interrupts (subset of @ref TIM_Optional_Interruptions).
  * @note   This function is the core of @ref HAL_TIM_Start_IT().
  *         and @ref HAL_TIM_Start_IT_Opt().
  * @retval HAL_OK
  */
static hal_status_t TIM_Start_IT_Opt(hal_tim_handle_t *htim,
                                     uint32_t interrupts)
{
  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Enable interrupts */
  LL_TIM_EnableIT(p_tim, interrupts);

  /* Enable TIMx counter except in trigger and 'combined reset + trigger modes'
     where enable is automatically done with trigger */
  uint32_t slave_mode = LL_TIM_GetSlaveMode(p_tim);

  if (!(IS_TIM_SLAVE_INSTANCE(p_tim) && IS_TIM_SLAVE_MODE_ENABLING_COUNTER(slave_mode)))
  {
    LL_TIM_EnableCounter(p_tim);
  }

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)

/**
  * @brief  Get the channel associated to a DMA channel.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  hdma  Pointer to the handle of the DMA instance.
  * @retval hal_tim_channel_t  The channel associated to the DMA channel.
  */
__STATIC_INLINE hal_tim_channel_t TIM_GetCCxDMAHandler(hal_tim_handle_t *htim, hal_dma_handle_t *hdma)
{
  hal_tim_channel_t channel;

  if (hdma == htim->hdma[HAL_TIM_DMA_ID_CC1])
  {
    channel = HAL_TIM_CHANNEL_1;
  }
  else if (hdma == htim->hdma[HAL_TIM_DMA_ID_CC2])
  {
    channel = HAL_TIM_CHANNEL_2;
  }
  else if (hdma == htim->hdma[HAL_TIM_DMA_ID_CC3])
  {
    channel = HAL_TIM_CHANNEL_3;
  }
  else
  {
    channel = HAL_TIM_CHANNEL_4;
  }

  return channel;
}

/**
  * @brief  DMA transfer error callback.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMAErrorCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->error_callback(htim);
#else
  HAL_TIM_ErrorCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA transfer stopped callback when triggered by an update event (UDE),
  *         a commutation event (COMDE), a trigger event (TDE), or when using
  *         TIM DMA Burst from any burst source.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMAStopCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->stop_callback(htim);
#else
  HAL_TIM_StopCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA transfer stopped callback when triggered by a timer capture/compare event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMAChannelStopCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_tim_channel_t channel = TIM_GetCCxDMAHandler(htim, hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->channel_stop_callback(htim, channel);
#else
  HAL_TIM_ChannelStopCallback(htim, channel);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data half transfer complete callback when triggered by a timer update event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMAUpdateHalfCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->update_half_cplt_callback(htim);
#else
  HAL_TIM_UpdateHalfCpltCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback when triggered by a timer update event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMAUpdateCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->update_callback(htim);
#else
  HAL_TIM_UpdateCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data half transfer complete callback when triggered by a timer compare match event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACompareMatchHalfCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_tim_channel_t channel = TIM_GetCCxDMAHandler(htim, hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->compare_match_half_cplt_callback(htim, channel);
#else
  HAL_TIM_CompareMatchHalfCpltCallback(htim, channel);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback when triggered by a timer compare match event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACompareMatchCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_tim_channel_t channel = TIM_GetCCxDMAHandler(htim, hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->compare_match_callback(htim, channel);
#else
  HAL_TIM_CompareMatchCallback(htim, channel);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data half transfer complete callback when triggered by a timer capture event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACaptureHalfCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_tim_channel_t channel = TIM_GetCCxDMAHandler(htim, hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->input_capture_half_cplt_callback(htim, channel);
#else
  HAL_TIM_InputCaptureHalfCpltCallback(htim, channel);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback when triggered by a timer capture event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACaptureCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

  /* Identify the channel */
  hal_tim_channel_t channel = TIM_GetCCxDMAHandler(htim, hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->input_capture_callback(htim, channel);
#else
  HAL_TIM_InputCaptureCallback(htim, channel);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data half transfer complete callback when triggered by a timer trigger event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMATriggerHalfCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->trigger_half_cplt_callback(htim);
#else
  HAL_TIM_TriggerHalfCpltCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback when triggered by a timer trigger event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMATriggerCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->trigger_callback(htim);
#else
  HAL_TIM_TriggerCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data half transfer complete callback when triggered by a timer commutation event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACommutationHalfCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->commutation_half_cplt_callback(htim);
#else
  HAL_TIM_CommutationHalfCpltCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA data transfer complete callback when triggered by a timer commutation event.
  * @param  hdma  Pointer to the DMA handle.
  */
static void TIM_DMACommutationCpltCallback(hal_dma_handle_t *hdma)
{
  hal_tim_handle_t *htim = TIM_GET_HDMA_PARENT(hdma);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->commutation_callback(htim);
#else
  HAL_TIM_CommutationCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
}

/**
  * @brief  Get the DMA index from the DMA request.
  * @param  dma_req  DMA request.
  * @retval hal_tim_dma_index_t  DMA index.
  */
__STATIC_INLINE hal_tim_dma_index_t TIM_DMARequestToDMAIndex(uint32_t dma_req)
{
  hal_tim_dma_index_t dma_index = HAL_TIM_DMA_ID_UPD;

  switch (dma_req)
  {
    case LL_TIM_DIER_CC1DE:
    {
      dma_index = HAL_TIM_DMA_ID_CC1;
      break;
    }
    case LL_TIM_DIER_CC2DE:
    {
      dma_index = HAL_TIM_DMA_ID_CC2;
      break;
    }
    case LL_TIM_DIER_CC3DE:
    {
      dma_index = HAL_TIM_DMA_ID_CC3;
      break;
    }
    case LL_TIM_DIER_CC4DE:
    {
      dma_index = HAL_TIM_DMA_ID_CC4;
      break;
    }
    case LL_TIM_DIER_COMDE:
    {
      dma_index = HAL_TIM_DMA_ID_COM;
      break;
    }
    case LL_TIM_DIER_TDE:
    {
      dma_index = HAL_TIM_DMA_ID_TRGI;
      break;
    }
    default:
    {
      /* LL_TIM_DIER_UDE:
           dma_index initialized with HAL_TIM_DMA_ID_UPD */
      break;
    }
  }

  return dma_index;
}

/**
  * @brief  Configure a DMA handle for a DMA transfer.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  dma_config  Pointer to the DMA configuration.
  * @param  interrupts  Selection of the DMA interrupts.
  * @note   This function is called by @ref TIM_Start_DMA_Opt(),
  *         @ref TIM_OC_StartChannel_DMA_Opt() and @ref TIM_IC_StartChannel_DMA_Opt().
  * @retval hal_dma_handle_t  Pointer to the DMA handle.
  */
__STATIC_INLINE hal_dma_handle_t *TIM_Config_DMA(hal_tim_handle_t *htim, tim_dma_config_t *dma_config,
                                                 uint32_t interrupts)
{
  hal_dma_handle_t *hdma = htim->hdma[dma_config->dma_idx];

  ASSERT_DBG_PARAM((hdma != NULL));
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1) && defined(USE_ASSERT_DBG_PARAM)
  ASSERT_DBG_PARAM(IS_TIM_DMA_VALID_SILENT_MODE(htim, dma_config->dma_idx, interrupts));
#else
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(interrupts);
#endif /* USE_HAL_DMA_LINKEDLIST */

  /* Set DMA channel callback function pointers */
  hdma->p_xfer_halfcplt_cb = dma_config->halfcplt_cb;
  hdma->p_xfer_cplt_cb = dma_config->cplt_cb;
  hdma->p_xfer_error_cb = TIM_DMAErrorCallback;

  LL_TIM_EnableDMAReq(TIM_INSTANCE(htim), dma_config->dma_req);

  return hdma;
}

/**
  * @brief  Start the timer in DMA mode with optional DMA interrupts.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in bytes).
  * @param  interrupts  Selection of the DMA interrupts.
  * @note   This function is the core of @ref HAL_TIM_Start_DMA()
  *         and @ref HAL_TIM_Start_DMA_Opt().
  * @retval HAL_OK
  * @retval HAL_ERROR  Failed to start the DMA transfer.
  */
static hal_status_t TIM_Start_DMA_Opt(hal_tim_handle_t *htim,
                                      const uint8_t *p_data,
                                      uint32_t size_byte,
                                      uint32_t interrupts)
{
  tim_t *p_tim = TIM_INSTANCE(htim);
  uint32_t is_slave_instance = (IS_TIM_SLAVE_INSTANCE(p_tim)) ? 1U : 0U;
  uint32_t is_slave_mode_enabled = (IS_TIM_SLAVE_MODE_ENABLED(p_tim)) ? 1U : 0U;
  uint32_t is_preload_enabled = LL_TIM_CC_IsEnabledPreload(p_tim);
  hal_dma_handle_t *hdma = NULL;
  /* index 0: update dma request
     index 1: commutation dma request
     index 2: trigger dma request */
  static const tim_dma_config_t dma_configurations[] =
  {
    {LL_TIM_DIER_UDE, TIM_DMAUpdateHalfCpltCallback, TIM_DMAUpdateCpltCallback, HAL_TIM_DMA_ID_UPD},
    {LL_TIM_DIER_COMDE, TIM_DMACommutationHalfCpltCallback, TIM_DMACommutationCpltCallback, HAL_TIM_DMA_ID_COM},
    {LL_TIM_DIER_TDE, TIM_DMATriggerHalfCpltCallback, TIM_DMATriggerCpltCallback, HAL_TIM_DMA_ID_TRGI}
  };
  uint32_t dma_config_idx = UPDATE_DMA_REQ_IDX; /* default update dma request */

  if (IS_TIM_COMMUTATION_EVENT_INSTANCE(p_tim) && (is_preload_enabled != 0U))
  {
    dma_config_idx = COMMUTATION_DMA_REQ_IDX;
  }
  else if ((is_slave_instance != 0U) && (is_slave_mode_enabled != 0U))
  {
    dma_config_idx = TRIGGER_DMA_REQ_IDX;
  }
  else
  {
    /* Nothing to do, already at UPDATE_DMA_REQ_IDX */
  }

  tim_dma_config_t dma_config = dma_configurations[dma_config_idx];
  hdma = TIM_Config_DMA(htim, &dma_config, interrupts);

  /* Start DMA transfer in IT mode: from Memory to ARR register */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                     (uint32_t)p_data,
                                     (uint32_t)((uint32_t *)(&p_tim->ARR)),
                                     size_byte, interrupts) != HAL_OK)
  {
#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
    htim->last_error_codes |= HAL_TIM_ERROR_DMA;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */
    htim->global_state = HAL_TIM_STATE_IDLE;

    return HAL_ERROR;
  }

  /* Enable TIMx counter except in 'trigger' and 'combined reset + trigger modes'
     where enable is automatically done with trigger */
  uint32_t slave_mode = LL_TIM_GetSlaveMode(p_tim);

  if ((is_slave_instance != 0U) && IS_TIM_SLAVE_MODE_ENABLING_COUNTER(slave_mode))
  {
    return HAL_OK;
  }

  LL_TIM_EnableCounter(p_tim);

  return HAL_OK;
}

/**
  * @brief  Start a timer's Output Channel in DMA mode with optional DMA interrupts.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  channel     Output channel of interest.
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in bytes).
  * @param  interrupts  Selection of the DMA interrupts.
  * @note   This function is the core of @ref HAL_TIM_OC_StartChannel_DMA()
  *         and @ref HAL_TIM_OC_StartChannel_DMA_Opt().
  * @retval HAL_OK
  * @retval HAL_ERROR  Failed to start the DMA transfer.
  */
__STATIC_INLINE hal_status_t TIM_OC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                                         hal_tim_channel_t channel,
                                                         const uint8_t *p_data,
                                                         uint32_t size_byte,
                                                         uint32_t interrupts)
{
  tim_t *p_tim = TIM_INSTANCE(htim);
  hal_dma_handle_t *hdma = NULL;
  tim_dma_config_t dma_config = {0};
  uint32_t channel_idx = (uint32_t)channel % (uint32_t)HAL_TIM_CHANNEL_1N;

  /* Check the validity of channel_idx value */
  if (channel_idx >= NB_TIM_CC_DMA_CONFIG)
  {
    return HAL_ERROR;
  }

  tim_cc_dma_config_t cc_dma_config = dma_channel_info[channel_idx];

  dma_config.dma_req = cc_dma_config.dma_req;
  dma_config.halfcplt_cb = TIM_DMACompareMatchHalfCpltCallback;
  dma_config.cplt_cb = TIM_DMACompareMatchCpltCallback;
  dma_config.dma_idx = cc_dma_config.dma_idx;

  hdma = TIM_Config_DMA(htim, &dma_config, interrupts);

  uint32_t dest_addr = (uint32_t)(&p_tim->CCR1) + LL_TIM_OFFSET_TAB_CCRx[channel_idx];

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                     (uint32_t)p_data,
                                     dest_addr,
                                     size_byte,
                                     interrupts) != HAL_OK)
  {
#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
    htim->last_error_codes |= HAL_TIM_ERROR_DMA;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */
    htim->channel_states[channel] = HAL_TIM_OC_CHANNEL_STATE_IDLE;

    return HAL_ERROR;
  }

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    LL_TIM_EnableAllOutputs(p_tim);
  }

  return HAL_OK;
}

/**
  * @brief  Start a timer's Input Channel in DMA mode with optional DMA interrupts.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  channel     Input channel of interest.
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in bytes).
  * @param  interrupts  Selection of the DMA interrupts.
  * @note   This function is the core of @ref HAL_TIM_IC_StartChannel_DMA()
  *         and @ref HAL_TIM_IC_StartChannel_DMA_Opt().
  * @retval HAL_OK
  * @retval HAL_ERROR  Failed to start the DMA transfer.
  */
__STATIC_INLINE hal_status_t TIM_IC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                                         hal_tim_channel_t channel,
                                                         uint8_t *p_data,
                                                         uint32_t size_byte,
                                                         uint32_t interrupts)
{
  tim_t *p_tim = TIM_INSTANCE(htim);
  hal_dma_handle_t *hdma = NULL;
  tim_dma_config_t dma_config = {0};

  /* Check the validity of channel value */
  if ((uint32_t)channel >= NB_TIM_CC_DMA_CONFIG)
  {
    return HAL_ERROR;
  }

  tim_cc_dma_config_t cc_dma_config = dma_channel_info[channel];

  dma_config.dma_req = cc_dma_config.dma_req;
  dma_config.halfcplt_cb = TIM_DMACaptureHalfCpltCallback;
  dma_config.cplt_cb = TIM_DMACaptureCpltCallback;
  dma_config.dma_idx = cc_dma_config.dma_idx;

  hdma = TIM_Config_DMA(htim, &dma_config, interrupts);

  uint32_t src_addr = (uint32_t)(&p_tim->CCR1) + LL_TIM_OFFSET_TAB_CCRx[channel];

  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma,
                                     src_addr,
                                     (uint32_t)p_data,
                                     size_byte,
                                     interrupts) != HAL_OK)
  {
#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
    htim->last_error_codes |= HAL_TIM_ERROR_DMA;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */
    htim->channel_states[channel] = HAL_TIM_IC_CHANNEL_STATE_IDLE;

    return HAL_ERROR;
  }

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  return HAL_OK;
}

/**
  * @brief  Abort any ongoing DMA channel transfer.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  dma_idx  DMA handle index
  * @param  active_silent_mode  Status of the silent mode.
  */
__STATIC_INLINE void TIM_Abort_DMA(hal_tim_handle_t *htim,
                                   hal_tim_dma_index_t dma_idx,
                                   uint32_t active_silent_mode)
{
  hal_dma_cb_t xfer_abort_cb;
  hal_dma_handle_t *hdma = htim->hdma[dma_idx];

  ASSERT_DBG_PARAM((hdma != NULL));

  if (active_silent_mode == HAL_TIM_ACTIVE_SILENT)
  {
    (void)HAL_DMA_Abort(hdma);
    return;
  }

  /* DMA stop callback function pointer depends on the DMA request source */
  if ((dma_idx == HAL_TIM_DMA_ID_UPD) || (dma_idx == HAL_TIM_DMA_ID_COM) || (dma_idx == HAL_TIM_DMA_ID_TRGI))
  {
    xfer_abort_cb = TIM_DMAStopCallback;
  }
  else
  {
    xfer_abort_cb = TIM_DMAChannelStopCallback;
  }

  hdma->p_xfer_abort_cb = xfer_abort_cb;
  if (HAL_DMA_Abort_IT(hdma) != HAL_OK)
  {
    xfer_abort_cb(hdma);
  }
}

/**
  * @brief  Stop DMA transfer and disable the DMA request.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  p_tim    Pointer to the TIM instance.
  * @param  channel  Channel of interest. \n
  *                  For an input channel, the channel parameter can take one of the value: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4 \n
  *                  For an output channel, the channel parameter can take one of the value: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *                  @arg @ref HAL_TIM_CHANNEL_1N
  *                  @arg @ref HAL_TIM_CHANNEL_2N
  *                  @arg @ref HAL_TIM_CHANNEL_3N
  *                  @arg @ref HAL_TIM_CHANNEL_4N
  * @param  active_silent_mode  Status of the silent mode.
  * @note   The validity of the channel is checked in the caller.
  * @retval HAL_OK
  * @retval HAL_ERROR  Invalid channel for DMA.
  */
__STATIC_INLINE hal_status_t TIM_StopChannel_DMA(hal_tim_handle_t *htim,
                                                 tim_t *p_tim,
                                                 hal_tim_channel_t channel,
                                                 uint32_t active_silent_mode)
{
  uint32_t channel_idx = (uint32_t)channel % (uint32_t)HAL_TIM_CHANNEL_1N;

  /* Check the validity of channel_idx value */
  if (channel_idx >= NB_TIM_CC_DMA_CONFIG)
  {
    return HAL_ERROR;
  }
  tim_cc_dma_config_t cc_dma_config = dma_channel_info[channel_idx];

  TIM_Abort_DMA(htim,
                cc_dma_config.dma_idx,
                active_silent_mode);

  LL_TIM_DisableDMAReq(p_tim, cc_dma_config.dma_req);

  return HAL_OK;
}

#endif /* USE_HAL_TIM_DMA */

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Callbacks initialization function.
  * @param  htim  Pointer to the handle of the TIM instance.
  */
__STATIC_FORCEINLINE void TIM_InitCallbacks(hal_tim_handle_t *htim)
{
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Error Callback */
  htim->error_callback = HAL_TIM_ErrorCallback;

  /* TIM Update DMA stop callback */
  htim->stop_callback = HAL_TIM_StopCallback;

  /* TIM capture/Compare DMA stop callback */
  htim->channel_stop_callback = HAL_TIM_ChannelStopCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Period Elapsed Callback */
  htim->update_callback = HAL_TIM_UpdateCallback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Period Elapsed half complete Callback */
  htim->update_half_cplt_callback = HAL_TIM_UpdateHalfCpltCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Trigger Callback */
  htim->trigger_callback = HAL_TIM_TriggerCallback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Trigger half complete Callback */
  htim->trigger_half_cplt_callback = HAL_TIM_TriggerHalfCpltCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Input Capture Callback */
  htim->input_capture_callback = HAL_TIM_InputCaptureCallback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Input Capture half complete Callback */
  htim->input_capture_half_cplt_callback = HAL_TIM_InputCaptureHalfCpltCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Output Compare Delay Elapsed Callback */
  htim->compare_match_callback = HAL_TIM_CompareMatchCallback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Output Compare Delay Elapsed Callback */
  htim->compare_match_half_cplt_callback = HAL_TIM_CompareMatchHalfCpltCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Commutation Callback */
  htim->commutation_callback = HAL_TIM_CommutationCallback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* TIM Commutation half complete Callback */
  htim->commutation_half_cplt_callback = HAL_TIM_CommutationHalfCpltCallback;
#endif /* USE_HAL_TIM_USER_DATA */

  /* TIM Break Callback */
  htim->break_callback = HAL_TIM_BreakCallback;

  /* TIM Break2 Callback */
  htim->break2_callback = HAL_TIM_Break2Callback;

  /* TIM Break2 Callback */
  htim->system_break_callback = HAL_TIM_SystemBreakCallback;

  /* TIM Software Break Callback */
  htim->software_break_callback = HAL_TIM_SoftwareBreakCallback;

  /* TIM Encoder Index Callback */
  htim->encoder_index_callback = HAL_TIM_EncoderIndexCallback;

  /* TIM Direction Change Callback */
  htim->direction_change_callback = HAL_TIM_DirectionChangeCallback;

  /* TIM Index Error Callback */
  htim->index_error_callback = HAL_TIM_IndexErrorCallback;

  /* TIM Transition Error Callback */
  htim->transition_error_callback = HAL_TIM_TransitionErrorCallback;
}
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

#if defined(USE_HAL_TIM_CLK_ENABLE_MODEL) && (USE_HAL_TIM_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
/**
  * @brief  Clock enabling for a particular instance.
  * @param  instance  HAL TIM instance
  */
__STATIC_FORCEINLINE void TIM_EnableClock(hal_tim_t instance)
{
  switch (instance)
  {
    case HAL_TIM1:
      HAL_RCC_TIM1_EnableClock();
      break;
    case HAL_TIM2:
      HAL_RCC_TIM2_EnableClock();
      break;
#if defined(TIM3)
    case HAL_TIM3:
      HAL_RCC_TIM3_EnableClock();
      break;
#endif /* TIM3 */
#if defined(TIM4)
    case HAL_TIM4:
      HAL_RCC_TIM4_EnableClock();
      break;
#endif /* TIM4 */
#if defined(TIM5)
    case HAL_TIM5:
      HAL_RCC_TIM5_EnableClock();
      break;
#endif /* TIM5 */
    case HAL_TIM6:
      HAL_RCC_TIM6_EnableClock();
      break;
    case HAL_TIM7:
      HAL_RCC_TIM7_EnableClock();
      break;
    case HAL_TIM8:
      HAL_RCC_TIM8_EnableClock();
      break;
    case HAL_TIM12:
      HAL_RCC_TIM12_EnableClock();
      break;
    case HAL_TIM15:
      HAL_RCC_TIM15_EnableClock();
      break;
#if defined(TIM16)
    case HAL_TIM16:
      HAL_RCC_TIM16_EnableClock();
      break;
    case HAL_TIM17:
      HAL_RCC_TIM17_EnableClock();
      break;
#endif /* TIM16 */
    default:
      break;
  }
}
#endif /* USE_HAL_TIM_CLK_ENABLE_MODEL */

/**
  *  @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup TIM_Exported_Functions
  *  @{
  */

/** @addtogroup TIM_Exported_Functions_Group1
  *  @brief This group gathers functions for the initialization/deinitialization of
  *         a timer instance.
  *  @{
  */

/**
  * @brief  Initialization function.
  *         Initialize the TIM handle and associate an instance.
  * @param  htim      Pointer to the handler of the TIM instance.
  * @param  instance  One of the value of the @ref hal_tim_t enumeration.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_Init(hal_tim_handle_t *htim, hal_tim_t instance)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM(IS_TIM_INSTANCE((tim_t *)((uint32_t)instance)));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (htim == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Register the TIM instance */
  htim->instance = instance;

#if defined(USE_HAL_TIM_CLK_ENABLE_MODEL) && (USE_HAL_TIM_CLK_ENABLE_MODEL > HAL_CLK_ENABLE_NO)
  TIM_EnableClock(instance);
#endif /* USE_HAL_TIM_CLK_ENABLE_MODEL */

#if defined (USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  TIM_InitCallbacks(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

  /* Init the handle internal parameters */

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  htim->dmaburst_source = TIM_DMABURST_NONE;
#endif /* USE_HAL_TIM_DMA */

#if defined (USE_HAL_TIM_USER_DATA) && (USE_HAL_TIM_USER_DATA == 1)
  htim->p_user_data = NULL;
#endif /* USE_HAL_TIM_USER_DATA */

  /* Reset channels state */
  for (uint32_t i = 0; i < HAL_TIM_CHANNELS; ++i)
  {
    htim->channel_states[i] = HAL_TIM_CHANNEL_STATE_RESET;
  }

#if defined(USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
  htim->last_error_codes = HAL_TIM_ERROR_NONE;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */

  htim->global_state = HAL_TIM_STATE_INIT;

  return HAL_OK;
}

/**
  * @brief   De-initialize the TIM instance handle.
  * @param   htim  Pointer to the handler of the TIM instance.
  * @note    Stop all current operations and reset states: \n
  *          @arg stop the counter
  *          @arg disable interrupts / DMA transfers
  *          @arg clear status flags
  *          @arg set channels' states to RESET
  *          @arg set global state to RESET
  * @note    HAL_TIM_DeInit does not reset all TIM registers.
  *          The Application must call RCC API to force the reset of all TIM registers.
  * @warning If a DMA transfer is ongoing, HAL_TIM_DeInit must be performed before
  *          calling HAL_DMA_DeInit to avoid asserts.
  */
void HAL_TIM_DeInit(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_INSTANCE(p_tim));

#if defined (USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /* Abort potential DMA in progress */
  uint32_t dma_req = LL_TIM_READ_REG(p_tim, DIER) & TIM_DMAREQ_REQUESTS_MASK;
#endif /* USE_HAL_TIM_DMA */

  LL_TIM_DisableCounter(p_tim);

  LL_TIM_WRITE_REG(p_tim, DIER, 0U);

  LL_TIM_WRITE_REG(p_tim, SR, 0U);

#if defined (USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  while (dma_req != 0U)
  {
    uint32_t active_dma_req = dma_req & (~dma_req + 1U);
    hal_tim_dma_index_t dma_idx = TIM_DMARequestToDMAIndex(active_dma_req);
    hal_dma_handle_t *hdma = htim->hdma[(uint32_t)dma_idx];

    if (hdma != NULL)
    {
      (void)HAL_DMA_Abort(hdma);
    }

    dma_req &= ~active_dma_req;
  }
#endif /* USE_HAL_TIM_DMA */

  /* Reset channels state */
  for (uint32_t i = 0; i < HAL_TIM_CHANNELS; ++i)
  {
    uint32_t ll_channel = ll_tim_channels[i];
    LL_TIM_CC_DisableChannel(p_tim, ll_channel);
    htim->channel_states[i] = HAL_TIM_CHANNEL_STATE_RESET;
  }

  htim->global_state = HAL_TIM_STATE_RESET;
}


#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Link a DMA handle to a DMA request.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param dma_idx  Index of the DMA request.
  * @param hdma     Pointer to a handle of the DMA instance.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetDMA(hal_tim_handle_t *htim,
                            hal_tim_dma_index_t dma_idx,
                            hal_dma_handle_t *hdma)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((hdma != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_INIT | HAL_TIM_STATE_IDLE));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (hdma == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  ASSERT_DBG_PARAM(IS_TIM_DMA_INDEX(dma_idx));

  /* link the DMA handle to the TIM handle */
  htim->hdma[(uint32_t)dma_idx] = hdma;
  hdma->p_parent = htim;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  *  @}
  */

/** @addtogroup TIM_Exported_Functions_Group2
  *  @{
  */

/**
  * @brief  Get the timer state.
  * @param  htim  Pointer to the handler of the TIM instance.
  * @retval hal_tim_state_t  HAL TIM state.
  */
hal_tim_state_t HAL_TIM_GetState(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));
  return htim->global_state;
}

/**
  * @brief  Get the state of a channel.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Channel of interest
  * @retval hal_tim_channel_state_t  TIM channel state
  */
hal_tim_channel_state_t HAL_TIM_GetChannelState(const hal_tim_handle_t *htim,
                                                hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM(IS_TIM_CHANNEL(channel));
  return htim->channel_states[channel];
}

#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
/**
  * @brief  Retrieve the HAL TIM Last Errors.
  * @param  htim  Pointer to the handler of the TIM instance.
  * @retval uint32_t  last error code. \n
  *                   Values can be: \n
  *                   @arg @ref HAL_TIM_ERROR_NONE
  *                   @arg @ref HAL_TIM_ERROR_DMA
  */
uint32_t HAL_TIM_GetLastErrorCodes(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));
  return htim->last_error_codes;
}

#endif /* USE_HAL_TIM_GET_LAST_ERRORS */

/** @brief  Return the peripheral clock frequency for TIM.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  Frequency in Hz.
  *                   0 if the source clock of the TIM is not configured or not ready.
  */
uint32_t HAL_TIM_GetClockFreq(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_INIT | HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return HAL_RCC_TIM_GetKernelClkFreq(TIM_INSTANCE(htim));
}

/**
  * @}
  */

/** @addtogroup TIM_Exported_Functions_Group3
  *  @{
  */

/**
  * @brief  Configure the timer's time-base unit.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to the time-base unit configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetConfig(hal_tim_handle_t *htim,
                               const hal_tim_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_INIT | HAL_TIM_STATE_IDLE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check parameters that are common to all instances */
  ASSERT_DBG_PARAM(IS_TIM_PRESCALER(p_config->prescaler));
  ASSERT_DBG_PARAM(IS_TIM_PERIOD(p_tim, p_config->period));
  ASSERT_DBG_PARAM(IS_TIM_CLK_SRC(p_config->clock_sel.clock_source));

  /* Apply the configuration */

  if (IS_TIM_REPETITION_COUNTER_INSTANCE(p_tim))
  {
    ASSERT_DBG_PARAM(IS_TIM_REPETITION_COUNTER(p_tim, p_config->repetition_counter));
    LL_TIM_SetRepetitionCounter(p_tim, p_config->repetition_counter);
  }

  if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(p_tim))
  {
    ASSERT_DBG_PARAM(IS_TIM_COUNTER_MODE(p_config->counter_mode));
    LL_TIM_SetCounterMode(p_tim, (uint32_t)p_config->counter_mode);
  }

  LL_TIM_SetAutoReload(p_tim, p_config->period);

  LL_TIM_SetPrescaler(p_tim, p_config->prescaler);

  TIM_SetClockSource(p_tim, &(p_config->clock_sel));

  uint32_t update_source = LL_TIM_GetUpdateSource(p_tim);

  if (update_source == LL_TIM_UPDATESOURCE_REGULAR)
  {
    /* Disable update event (UEV) with update generation (UG)
      by changing update request source (URS) to avoid update flag (UIF) */
    LL_TIM_SetUpdateSource(p_tim, LL_TIM_UPDATESOURCE_COUNTER);

    /* Generate an update event to reload the prescaler
      and the repetition counter (if applicable) values immediately */
    LL_TIM_GenerateEvent_UPDATE(p_tim);

    /* Put back the update event source */
    LL_TIM_SetUpdateSource(p_tim, LL_TIM_UPDATESOURCE_REGULAR);
  }
  else
  {
    /* Generate an update event to reload the prescaler
      and the repetition counter (if applicable) values immediately */
    LL_TIM_GenerateEvent_UPDATE(p_tim);
  }

  /* Reset channels (needed only if in IDLE state but done by default) */
  for (uint32_t i = 0; i < HAL_TIM_CHANNELS; ++i)
  {
    htim->channel_states[i] = HAL_TIM_CHANNEL_STATE_RESET;
  }

  htim->global_state = HAL_TIM_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the the timer's time-base unit.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a time-base unit configuration structure to fill.
  */
void HAL_TIM_GetConfig(const hal_tim_handle_t *htim,
                       hal_tim_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  if (IS_TIM_REPETITION_COUNTER_INSTANCE(p_tim))
  {
    p_config->repetition_counter = LL_TIM_GetRepetitionCounter(p_tim);
  }

  if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(p_tim))
  {
    p_config->counter_mode = (hal_tim_counter_mode_t)LL_TIM_GetCounterMode(p_tim);
  }

  p_config->period = LL_TIM_GetAutoReload(p_tim);

  p_config->prescaler = LL_TIM_GetPrescaler(p_tim);

  /* Get the clock source (and trigger input in case of external clock signal) */
  TIM_GetClockSource(p_tim, &(p_config->clock_sel));
}

/**
  * @brief  Set the period of the timer's time-base unit.
  * @param  htim    Pointer to the handle of the TIM instance.
  * @param  period  Period for the time base unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetPeriod(hal_tim_handle_t *htim,
                               uint32_t period)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_PERIOD_WITHOUT_DITHERING(p_tim, period));

  LL_TIM_SetAutoReload(p_tim, period);

  return HAL_OK;
}

/**
  * @brief  Get the period of the timer's time-base unit.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  Period of the timer's time-base unit.
  */
uint32_t HAL_TIM_GetPeriod(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return LL_TIM_GetAutoReload(TIM_INSTANCE(htim));
}

/**
  * @brief   Set the period and dithering pattern of the timer's
  *          time-base unit.
  * @param   htim    Pointer to the handle of the TIM instance.
  * @param   period  Period for the time base unit(integer part).
  * @param   period_dithering_pattern  Dithering pattern for the period (period
  *          fractional part)
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_SetDitheredPeriod(hal_tim_handle_t *htim,
                                       uint32_t period,
                                       hal_tim_dithering_pattern_t period_dithering_pattern)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM(IS_TIM_DITHERING_PATTERN(period_dithering_pattern));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the period can be shifted */
  ASSERT_DBG_PARAM(IS_TIM_PERIOD_WITH_DITHERING(p_tim, period));

  /* Set in ARR the integer period and the dithering part */
  LL_TIM_SetAutoReload(p_tim, HAL_TIM_COMPUTE_DITHERED_PERIOD(period,
                                                              (uint32_t)period_dithering_pattern));

  return HAL_OK;
}

/**
  * @brief  Get the period and its dithering pattern of the timer's
  *         time-base unit.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_period  Pointer for the period for the time base unit
  *         (period's integer part).
  * @param  p_period_dithering_pattern  Dithering pattern for the period
  *         (period's fractional part)
  */
void HAL_TIM_GetDitheredPeriod(const hal_tim_handle_t *htim,
                               uint32_t *p_period,
                               hal_tim_dithering_pattern_t *p_period_dithering_pattern)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  const tim_t *p_tim = TIM_INSTANCE(htim);

  /* Get in ARR the integer period and the dithering part */
  uint32_t arr = LL_TIM_GetAutoReload(p_tim);
  *p_period = (arr & ~TIM_DITHERING_MASK) >> HAL_TIM_DITHERING_SHIFT;
  *p_period_dithering_pattern = (hal_tim_dithering_pattern_t)((uint32_t)(arr & TIM_DITHERING_MASK));
}

/**
  * @brief  Set the prescaler of the timer's time-base unit.
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  prescaler  Prescaler for the time base unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetPrescaler(hal_tim_handle_t *htim,
                                  uint32_t prescaler)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_PRESCALER(prescaler));

  LL_TIM_SetPrescaler(p_tim, prescaler);

  return HAL_OK;
}

/**
  * @brief  Get the prescaler value of the timer's time-base unit.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  Prescaler value of the timer's time-base unit.
  */
uint32_t HAL_TIM_GetPrescaler(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return LL_TIM_GetPrescaler(TIM_INSTANCE(htim));
}

/**
  * @brief  Set the counter mode of the timer's time-base unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  counter_mode  Counter mode.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetCounterMode(hal_tim_handle_t *htim,
                                    hal_tim_counter_mode_t counter_mode)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports mode selection */
  ASSERT_DBG_PARAM(IS_TIM_COUNTER_MODE_SELECT_INSTANCE(p_tim));

  /* Check counter mode validity */
  ASSERT_DBG_PARAM(IS_TIM_COUNTER_MODE(counter_mode));

  LL_TIM_SetCounterMode(p_tim, (uint32_t)counter_mode);

  return HAL_OK;
}

/**
  * @brief  Get the counter mode of the timer's time-base unit.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_counter_mode_t  Counter mode of the timer's time-base unit.
  */
hal_tim_counter_mode_t HAL_TIM_GetCounterMode(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports mode selection */
  ASSERT_DBG_PARAM(IS_TIM_COUNTER_MODE_SELECT_INSTANCE(p_tim));

  return (hal_tim_counter_mode_t)LL_TIM_GetCounterMode(p_tim);
}

/**
  * @brief  Set the DTS clock prescaler.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  dts_prescaler  The DTS clock prescaler.
  * @note   The prescaler set the division ratio between the timer kernel clock (tim_ker_ck)
  *         and the DTS sampling clock (DTS_ck).
  * @note   The DTS sampling clock is used, when supported, by the dead-time
  *         generator, the break/break2 filters and the delayed break.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetDTSPrescaler(hal_tim_handle_t *htim,
                                     hal_tim_dts_prescaler_t dts_prescaler)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports clock division */
  ASSERT_DBG_PARAM(IS_TIM_CLOCK_DIVISION_INSTANCE(p_tim));

  /* Check clock division validity */
  ASSERT_DBG_PARAM(IS_TIM_DTS_PRESCALER(dts_prescaler));

  LL_TIM_SetClockDivision(p_tim, (uint32_t)dts_prescaler);

  return HAL_OK;
}

/**
  * @brief  Get the DTS clock prescaler.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_dts_prescaler_t  DTS clock prescaler.
  */
hal_tim_dts_prescaler_t HAL_TIM_GetDTSPrescaler(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports clock division */
  ASSERT_DBG_PARAM(IS_TIM_CLOCK_DIVISION_INSTANCE(p_tim));

  return (hal_tim_dts_prescaler_t)LL_TIM_GetClockDivision(p_tim);
}

/**
  * @brief  Set the DTS2 clock prescaler.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  dts2_prescaler  The DTS2 clock prescaler.
  * @note   The prescaler sets the division ratio between the DTS sampling clock (DTS_ck)
  *         and the DTS2 sampling clock (DTS2_ck).
  * @note   The DTS2 sampling clock is used by the digital filters (tim_etr_in, tim_tix).
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetDTS2Prescaler(hal_tim_handle_t *htim,
                                      hal_tim_dts2_prescaler_t dts2_prescaler)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports clock division */
  ASSERT_DBG_PARAM(IS_TIM_CLOCK_DIVISION_INSTANCE(p_tim));

  /* Check clock division validity */
  ASSERT_DBG_PARAM(IS_TIM_DTS2_PRESCALER(dts2_prescaler));

  LL_TIM_SetClockDivision2(p_tim, (uint32_t)dts2_prescaler);

  return HAL_OK;
}

/**
  * @brief  Get the DTS2 clock prescaler.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_dts2_prescaler_t  DTS2 clock prescaler.
  */
hal_tim_dts2_prescaler_t HAL_TIM_GetDTS2Prescaler(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports clock division */
  ASSERT_DBG_PARAM(IS_TIM_CLOCK_DIVISION_INSTANCE(p_tim));

  return (hal_tim_dts2_prescaler_t)LL_TIM_GetClockDivision2(p_tim);
}

/**
  * @brief  Set the repetition counter value of the timer's time-base unit.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  repetition_counter  Value of the repetition counter.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetRepetitionCounter(hal_tim_handle_t *htim,
                                          uint32_t repetition_counter)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports repetition counter */
  ASSERT_DBG_PARAM(IS_TIM_REPETITION_COUNTER_INSTANCE(p_tim));

  /* Check repetition counter validity */
  ASSERT_DBG_PARAM(IS_TIM_REPETITION_COUNTER(p_tim, repetition_counter));

  LL_TIM_SetRepetitionCounter(p_tim, repetition_counter);

  return HAL_OK;
}

/**
  * @brief  Get the repetition counter value of the timer's time-base unit.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  Repetition counter value of the timer's time-base unit.
  */
uint32_t HAL_TIM_GetRepetitionCounter(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Ensure that the instance supports repetition counter */
  ASSERT_DBG_PARAM(IS_TIM_REPETITION_COUNTER_INSTANCE(p_tim));

  return LL_TIM_GetRepetitionCounter(p_tim);

}

/**
  * @brief  Set the clock source of the timer's time-base unit.
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  p_clk_sel  Pointer to the clock selection.
  *                    Clock selection is used to set the clock source of the
  *                    timer's time-base unit.
  *                    If the clock source is @ref HAL_TIM_CLK_EXTERNAL_MODE1 then
  *                    the external trigger that is used as clock signal is also
  *                    specified.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetClockSource(hal_tim_handle_t *htim,
                                    const hal_tim_clock_sel_t *p_clk_sel)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_clk_sel != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_clk_sel == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE));

  ASSERT_DBG_PARAM(IS_TIM_CLK_SRC(p_clk_sel->clock_source));

  TIM_SetClockSource(TIM_INSTANCE(htim), p_clk_sel);

  return HAL_OK;
}

/**
  * @brief  Get the clock source of the timer's time-base unit.
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  p_clk_sel  Pointer to the clock selection that gather 2 parameters: \n
  *                    @arg clock_source  for the clock source of the timer's time-base unit.
  *                    @arg trigger       which is meaningful only in the case where the
  *                                       clock source is @ref HAL_TIM_CLK_EXTERNAL_MODE1.
  *                                       Then, it stores the value of the external
  *                                       trigger that is used as clock signal.
  */
void HAL_TIM_GetClockSource(const hal_tim_handle_t *htim,
                            hal_tim_clock_sel_t *p_clk_sel)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_clk_sel != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  TIM_GetClockSource(TIM_INSTANCE(htim), p_clk_sel);
}

/**
  * @brief   Set Counter Register (TIMx_CNT) value at runtime.
  * @param   htim           Pointer to the handle of the TIM instance.
  * @param   counter_value  Counter register new value.
  * @warning When UIF bit remapping is enabled (see @ref HAL_TIM_EnableUpdateFlagRemap),
  *          bit 31 of the timer counter register is read-only. This can affect the
  *          counter range for a 32-bit counter TIM instance.
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_SetCounter(hal_tim_handle_t *htim,
                                uint32_t counter_value)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_COUNTER(p_tim, counter_value));

  LL_TIM_SetCounter(p_tim, counter_value);

  return HAL_OK;
}

/**
  * @brief  Get Counter Register (TIMx_CNT) value at runtime.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   When UIF bit remapping is enabled (see @ref HAL_TIM_EnableUpdateFlagRemap),
  *         bit 31 of the returned value contains a copy of the update interrupt flag (UIF).
  * @retval uint32_t  16-bit or 32-bit value of the timer counter register (TIMx_CNT).
  */
uint32_t HAL_TIM_GetCounter(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return LL_TIM_GetCounter(TIM_INSTANCE(htim));
}

/**
  * @brief  Enable update event generation.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableUpdateGeneration(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_EnableUpdateEvent(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Disable update event generation.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   Once update event generation has been disabled, no update event
  *         occurs until @ref HAL_TIM_EnableUpdateGeneration is called.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableUpdateGeneration(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_DisableUpdateEvent(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Tell whether update event generation is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_update_generation_status_t  Update Event Generation Status (Disabled/Enabled)
  */
hal_tim_update_generation_status_t HAL_TIM_IsEnabledUpdateGeneration(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_update_generation_status_t)
         (LL_TIM_IsEnabledUpdateEvent((tim_t *)((uint32_t)htim->instance)));
}

/**
  * @brief  Set update event source.
  * @param  htim           Pointer to the handle of the TIM instance.
  * @param  update_source  Source for the Update event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetUpdateSource(hal_tim_handle_t *htim,
                                     hal_tim_update_src_t update_source)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM(IS_TIM_UPDATE_SRC(update_source));

  LL_TIM_SetUpdateSource(TIM_INSTANCE(htim), (uint32_t)update_source);

  return HAL_OK;
}

/**
  * @brief  Get update event source.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_update_src_t  Source of the update event.
  */
hal_tim_update_src_t HAL_TIM_GetUpdateSource(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_update_src_t)(LL_TIM_GetUpdateSource(TIM_INSTANCE(htim)));
}

/**
  * @brief  Force a continuous copy of the update interrupt flag (UIF)
  *         into the timer counter register (bit 31).
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableUpdateFlagRemap(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_EnableUIFRemap(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Disable the copy of the update interrupt flag (UIF)
  *         into the timer counter register (bit 31).
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableUpdateFlagRemap(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_DisableUIFRemap(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Tell whether the copy of the update interrupt flag (UIF) into the
  *         timer counter register is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_update_flag_remap_status_t  Update interrupt flag copy
  *                                             status (Disabled/Enabled)
  */
hal_tim_update_flag_remap_status_t HAL_TIM_IsEnabledUpdateFlagRemap(const hal_tim_handle_t *htim)

{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_update_flag_remap_status_t)(LL_TIM_IsEnabledUIFRemap(TIM_INSTANCE(htim)));
}

/**
  * @brief  Enable the auto-reload preload.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   When autoreload preload is enabled, autoreload (TIMx_ARR) preload
  *         value isn't taken into account immediately. \n
  *         It is loaded in the active register at next update event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableAutoReloadPreload(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_EnableARRPreload(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Disable the auto-reload preload.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableAutoReloadPreload(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_DisableARRPreload(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Tell whether autoreload preload is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_auto_reload_preload_status_t  Auto-reload preload status of TIM.
  */
hal_tim_auto_reload_preload_status_t HAL_TIM_IsEnabledAutoReloadPreload(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return ((hal_tim_auto_reload_preload_status_t)
          LL_TIM_IsEnabledARRPreload(TIM_INSTANCE(htim)));
}

/**
  * @brief  Enable dithering for the timer.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableDithering(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_EnableDithering(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Disable dithering for the timer.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   It is recommended to follow this sequence when disabling the dithering:
  *          1. The Counter must be stopped @ref HAL_TIM_Stop(_IT/_DMA) and
  *             Auto Reload preload disabled @ref HAL_TIM_DisableAutoReloadPreload
  *          2. The new Period without dithering must be set @ref HAL_TIM_SetPeriod
  *          3. The new Pulse values without dithering must be set @ref HAL_TIM_OC_SetCompareUnitPulse
  *          3. The Dithering must be disabled @ref HAL_TIM_DisableDithering
  *          4. Capture/compare interrupt flags must be cleared LL_TIM_ClearFlag_CC1 (for each channel)
  *          5. The Counter can be re-enabled @ref HAL_TIM_Start(_IT/_DMA)
  *             (eventually with Auto Reload preload).
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableDithering(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_DisableDithering(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Tell whether dithering is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_dithering_status_t  Dithering status of TIM.
  */
hal_tim_dithering_status_t HAL_TIM_IsEnabledDithering(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_dithering_status_t)LL_TIM_IsEnabledDithering(TIM_INSTANCE(htim));
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Set the source that triggers the capture/compare DMA request.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  cc_dmareq_source  Source for the Capture/Compare DMA request.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetCaptureCompareDMAReqSource(hal_tim_handle_t *htim,
                                                   hal_tim_cc_dmareq_src_t cc_dmareq_source)
{

  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM(IS_TIM_CC_DMAREQ_SRC(cc_dmareq_source));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_DMA_CC_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_CC_SetDMAReqTrigger(p_tim, (uint32_t)cc_dmareq_source);

  return HAL_OK;
}

/**
  * @brief  Get the source that triggers the capture/compare DMA request.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_cc_dmareq_src_t  The source that triggers the DMA request.
  */
hal_tim_cc_dmareq_src_t HAL_TIM_GetCaptureCompareDMAReqSource(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_DMA_CC_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_cc_dmareq_src_t)LL_TIM_CC_GetDMAReqTrigger(p_tim);

}
#endif /* USE_HAL_TIM_DMA */


/**
  * @brief  Start the timer in polling mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Start(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, global_state, HAL_TIM_STATE_IDLE,
                         HAL_TIM_STATE_ACTIVE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Enable TIMx counter except in 'trigger' and 'combined reset + trigger modes'
     where enable is automatically done with trigger.
     Thus it is not mandatory to call HAL_TIM_Start() for these modes. */
  uint32_t slave_mode = LL_TIM_GetSlaveMode(p_tim);

  if (!(IS_TIM_SLAVE_INSTANCE(p_tim) && IS_TIM_SLAVE_MODE_ENABLING_COUNTER(slave_mode)))
  {
    LL_TIM_EnableCounter(p_tim);
  }

  return HAL_OK;
}

/**
  * @brief  Stop the timer that was started in polling mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Stop(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_ACTIVE);

  LL_TIM_DisableCounter(p_tim);

  htim->global_state = HAL_TIM_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Start the timer in interrupt mode (default TIM update interrupt).
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Start_IT(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, global_state, HAL_TIM_STATE_IDLE,
                         HAL_TIM_STATE_ACTIVE);

  return TIM_Start_IT_Opt(htim, HAL_TIM_OPT_IT_UPDATE);
}

/**
  * @brief  Start the timer in interrupt mode.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  interrupts  Selection of the TIM interrupts (subset of @ref TIM_Optional_Interruptions). \n
  *                     Can be any of the (meaningful) ored values: \n
  *                     @arg @ref HAL_TIM_OPT_IT_UPDATE
  *                     @arg @ref HAL_TIM_OPT_IT_COMMUTATION
  *                     @arg @ref HAL_TIM_OPT_IT_TRIGGER_INPUT
  *                     @arg @ref HAL_TIM_OPT_IT_BREAK
  *                     @arg @ref HAL_TIM_OPT_IT_ENCODER_INDEX
  *                     @arg @ref HAL_TIM_OPT_IT_ENCODER_DIRECTION
  *                     @arg @ref HAL_TIM_OPT_IT_ENCODER_INDEX_ERROR
  *                     @arg @ref HAL_TIM_OPT_IT_ENCODER_TRANSITION_ERROR
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Start_IT_Opt(hal_tim_handle_t *htim, uint32_t interrupts)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, global_state, HAL_TIM_STATE_IDLE,
                         HAL_TIM_STATE_ACTIVE);

  /* Check that all the interrupts selected are supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_OPTIONAL_INTERRUPTS(TIM_INSTANCE(htim), interrupts));

  return TIM_Start_IT_Opt(htim, interrupts);
}

/**
  * @brief  Stop the timer that was started in interrupt mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Stop_IT(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_ACTIVE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Disable all interrupts by default */
  LL_TIM_DisableIT(p_tim, TIM_OPTIONAL_INTERRUPTS_MASK);

  LL_TIM_DisableCounter(p_tim);

  htim->global_state = HAL_TIM_STATE_IDLE;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Start the timer in DMA mode (default DMA interrupts).
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  p_data     Pointer to the data buffer.
  * @param  size_byte  Data buffer size (in bytes).
  * @note   One data will be transferred from the buffer to the autoreload
  *         register (TIMx_ARR) at each update event. \n
  *         DMA transfer ends when all the data of the buffer have been
  *         transferred.
  * @note   @ref HAL_TIM_SetDMA() must be called with the correct DMA index
  *         (see @ref hal_tim_dma_index_t) before calling this function.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_Start_DMA(hal_tim_handle_t *htim,
                               const uint8_t *p_data,
                               uint32_t size_byte)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  HAL_CHECK_UPDATE_STATE(htim, global_state, HAL_TIM_STATE_IDLE,
                         HAL_TIM_STATE_ACTIVE);

  return TIM_Start_DMA_Opt(htim, p_data, size_byte, HAL_TIM_OPT_DMA_IT_DEFAULT);
}

/**
  * @brief  Start the timer in DMA mode (optional DMA interrupts).
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in bytes).
  * @param  interrupts  Selection of the DMA interrupts (subset of @ref TIM_Optional_Interruptions). \n
  *                     Can be any of the (meaningful) ored values: \n
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_NONE
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_HT
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_SILENT
  * @endif
  * @note   One data will be transferred from the buffer to the autoreload
  *         register (TIMx_ARR) at each update event. \n
  *         DMA transfer ends when all the data of the buffer have been
  *         transferred.
  * @note   @ref HAL_TIM_SetDMA() must be called with the correct DMA index
  *         (see @ref hal_tim_dma_index_t) before calling this function.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_Start_DMA_Opt(hal_tim_handle_t *htim,
                                   const uint8_t *p_data,
                                   uint32_t size_byte,
                                   uint32_t interrupts)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  HAL_CHECK_UPDATE_STATE(htim, global_state, HAL_TIM_STATE_IDLE,
                         (TIM_STATE_ACTIVE(interrupts)));

  return TIM_Start_DMA_Opt(htim, p_data, size_byte, interrupts);
}

/**
  * @brief  Stop the timer that was started in DMA mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_Stop_DMA(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_ACTIVE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(p_tim));

  uint32_t dier = LL_TIM_READ_REG(p_tim, DIER);

  /* Retrieve dma requests already enabled (update, commutation and trigger) */
  uint32_t dma_req = dier & (LL_TIM_DIER_UDE | LL_TIM_DIER_COMDE | LL_TIM_DIER_TDE);

  /* Check that almost one dma request is enabled.
     Otherwise, it means that no HAL_TIM_Start_DMA() has been done.
     When using the driver, no more than 2 dma requests must be enabled.
     So, no check for this. */
  ASSERT_DBG_PARAM(dma_req != 0U);

  if (htim->dmaburst_source != TIM_DMABURST_NONE)
  {
    /* Calculate the dma request associated to the dma burst source
      (-1U because dma burst source starts at 1 (0 is reserved)) */
    uint32_t dmaburst_req = LL_TIM_DIER_UDE << (((uint32_t)htim->dmaburst_source >> TIM_DMABURST_DMAINDEX_SHIFT) - 1U);

    /* Disable dma requests except if used by the dma burst */
    dma_req &= ~dmaburst_req;
  }

  LL_TIM_DisableDMAReq(p_tim, dma_req);

  /* Calculate the dma index from the dma request */
  hal_tim_dma_index_t dma_index = TIM_DMARequestToDMAIndex(dma_req);

  TIM_Abort_DMA(htim,
                dma_index,
                (IS_TIM_ACTIVE_SILENT(htim->global_state)));

  LL_TIM_DisableCounter(p_tim);

  htim->global_state = HAL_TIM_STATE_IDLE;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @}
  */


/** @addtogroup TIM_Exported_Functions_Group4
  *  @{
  */

/**
  * @brief  Configure an output compare unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit to configure.
  * @param  p_config      Pointer to an output compare unit configuration structure.
  * @note   If dithering is activated, the value of pulse is split in two parts:
  *         bits[31:4] holds the integer part and bits[3:0] the fractional part.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_OC_SetConfigCompareUnit(hal_tim_handle_t *htim,
                                             hal_tim_oc_compare_unit_t compare_unit,
                                             const hal_tim_oc_compare_unit_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check compare unit configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_OC_MODE(compare_unit, p_config->mode));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_OC_PULSE(p_tim, p_config->pulse));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_SetMode(p_tim, ll_tim_channels[(uint32_t)compare_unit], (uint32_t)p_config->mode);

  LL_TIM_OC_SetCompareValue(p_tim, (uint32_t)compare_unit, p_config->pulse);

  return HAL_OK;
}

/**
  * @brief  Get the configuration of an output compare unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @param  p_config      Pointer to an output compare unit configuration structure.
  * @note   If dithering is activated, pay attention to the returned value interpretation.
  */
void HAL_TIM_OC_GetConfigCompareUnit(const hal_tim_handle_t *htim,
                                     hal_tim_oc_compare_unit_t compare_unit,
                                     hal_tim_oc_compare_unit_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  p_config->mode = (hal_tim_oc_mode_t)LL_TIM_OC_GetMode(p_tim, ll_tim_channels[(uint32_t)compare_unit]);

  p_config->pulse = LL_TIM_OC_GetCompareValue(p_tim, (uint32_t)compare_unit);
}

/**
  * @brief  Set the pulse of an output compare unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @param  pulse         Compare match value.
  * @note   The pulse value can also include the fractional part for the dithering mode.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_SetCompareUnitPulse(hal_tim_handle_t *htim,
                                            hal_tim_oc_compare_unit_t compare_unit,
                                            uint32_t pulse)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);
  ASSERT_DBG_PARAM(IS_TIM_OC_PULSE(p_tim, pulse));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_SetCompareValue(p_tim, (uint32_t)compare_unit, pulse);

  return HAL_OK;
}

/**
  * @brief  Get the pulse of an output compare unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval uint32_t  Compare match value
  */
uint32_t HAL_TIM_OC_GetCompareUnitPulse(const hal_tim_handle_t *htim,
                                        hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  return LL_TIM_OC_GetCompareValue(p_tim, (uint32_t)compare_unit);
}

/**
  * @brief   Set the pulse and dithering pattern of an output compare unit.
  * @param   htim          Pointer to the handle of the TIM instance.
  * @param   compare_unit  Output compare unit to configure.
  * @param   pulse         Compare match value (integer part).
  * @param   pulse_dithering_pattern  Dithering pattern for the pulse (fractional part).
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_OC_SetCompareUnitDitheredPulse(hal_tim_handle_t *htim,
                                                    hal_tim_oc_compare_unit_t compare_unit,
                                                    uint32_t pulse,
                                                    hal_tim_dithering_pattern_t pulse_dithering_pattern)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM(IS_TIM_DITHERING_PATTERN(pulse_dithering_pattern));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_OC_PULSE_WITH_DITHERING(p_tim, pulse));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_SetCompareValue(p_tim, (uint32_t)compare_unit,
                            HAL_TIM_COMPUTE_DITHERED_PULSE(pulse,
                                                           (uint32_t)pulse_dithering_pattern));

  return HAL_OK;
}

/**
  * @brief  Get the pulse and dithering pattern of an output compare unit.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @param  p_pulse       Pointer to compare match value (integer part).
  * @param  p_pulse_dithering_pattern  Pointer to dithering pattern for the pulse
  *                                    (fractional part)
  */
void HAL_TIM_OC_GetCompareUnitDitheredPulse(const hal_tim_handle_t *htim,
                                            hal_tim_oc_compare_unit_t compare_unit,
                                            uint32_t *p_pulse,
                                            hal_tim_dithering_pattern_t *p_pulse_dithering_pattern)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_pulse != NULL));
  ASSERT_DBG_PARAM((p_pulse_dithering_pattern != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  uint32_t compare_match_value = LL_TIM_OC_GetCompareValue(p_tim, (uint32_t)compare_unit);

  *p_pulse = ((compare_match_value & ~TIM_DITHERING_MASK)
              >> HAL_TIM_DITHERING_SHIFT);

  *p_pulse_dithering_pattern = (hal_tim_dithering_pattern_t)(uint32_t)(compare_match_value &
                                                                       TIM_DITHERING_MASK);
}

/**
  * @brief  Enable compare register (TIMx_CCRy) preload of an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @note   When output compare preload is enabled, compare (TIMx_CCRy) preload
  *         value isn't taken into account immediately. \n
  *         It is loaded in the active register at next update event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_EnableComparePreload(hal_tim_handle_t *htim,
                                             hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_EnablePreload(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Disable register (TIMx_CCRy) compare preload of an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_DisableComparePreload(hal_tim_handle_t *htim,
                                              hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_DisablePreload(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Tell whether output compare preload is enabled or not for an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval hal_tim_oc_compare_preload_status_t  Compare preload status.
  */
hal_tim_oc_compare_preload_status_t HAL_TIM_OC_IsEnabledComparePreload(const hal_tim_handle_t *htim,
                                                                       hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  return (hal_tim_oc_compare_preload_status_t)LL_TIM_OC_IsEnabledPreload(p_tim,
                                                                         ll_tim_channels[compare_unit]);
}

/**
  * @brief  Enable fast mode for an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_EnableCompareFastMode(hal_tim_handle_t *htim,
                                              hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_EnableFast(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Disable fast mode for an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_DisableCompareFastMode(hal_tim_handle_t *htim,
                                               hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_DisableFast(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Tell whether fast mode is enabled or not for an output channel.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval hal_tim_oc_compare_fast_mode_status_t  Fast Mode status.
  */
hal_tim_oc_compare_fast_mode_status_t HAL_TIM_OC_IsEnabledCompareFastMode(const hal_tim_handle_t *htim,
                                                                          hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  return (hal_tim_oc_compare_fast_mode_status_t)LL_TIM_OC_IsEnabledFast(p_tim,
                                                                        ll_tim_channels[compare_unit]);
}

/**
  * @brief  Configure an output channel.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  channel   Output channel to configure.
  * @param  p_config  Pointer to an output channel configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_OC_SetConfigChannel(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const hal_tim_oc_channel_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_PARAM(IS_TIM_OC_POLARITY(p_config->polarity));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_CHANNEL_STATE_RESET | TIM_CHANNEL_STATE_IDLE));

  LL_TIM_OC_SetPolarity(p_tim, ll_tim_channels[channel], (uint32_t)p_config->polarity);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    ASSERT_DBG_PARAM(IS_TIM_OC_IDLE_STATE(p_config->idle_state));
    LL_TIM_OC_SetIdleState(p_tim, ll_tim_channels[channel], (uint32_t)p_config->idle_state);
    if (!(IS_TIM_OC_INTERNAL_CHANNEL(channel)))
    {
      ASSERT_DBG_PARAM(IS_TIM_OC_OVERRIDE_STATE(p_config->override_state));
      ASSERT_DBG_PARAM(IS_TIM_OC_BREAK_MODE(p_config->break_mode));
      LL_TIM_OC_SetOverrideState(p_tim, ll_tim_channels[channel], (uint32_t)p_config->override_state);
      LL_TIM_OC_SetBreakMode(p_tim, ll_tim_channels[channel], (uint32_t)p_config->break_mode);
    }
  }

  htim->channel_states[channel] = HAL_TIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of an Output Channel.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  channel   Output channel.
  * @param  p_config  Pointer to an output channel configuration structure.
  * @note   There is no check that the channel direction is indeed output.
  */
void HAL_TIM_OC_GetConfigChannel(const hal_tim_handle_t *htim,
                                 hal_tim_channel_t channel,
                                 hal_tim_oc_channel_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_CHANNEL_STATE_RESET | HAL_TIM_OC_CHANNEL_STATE_IDLE \
                    | HAL_TIM_OC_CHANNEL_STATE_ACTIVE));

  uint32_t ll_channel = ll_tim_channels[channel];

  p_config->polarity = (hal_tim_oc_polarity_t)LL_TIM_OC_GetPolarity(p_tim, ll_channel);

  p_config->idle_state = (hal_tim_oc_idle_state_t)LL_TIM_OC_GetIdleState(p_tim, ll_channel);

  p_config->override_state = (hal_tim_oc_override_state_t)LL_TIM_OC_GetOverrideState(p_tim, ll_tim_channels[channel]);

  p_config->break_mode = (hal_tim_oc_break_mode_t)LL_TIM_OC_GetBreakMode(p_tim, ll_tim_channels[channel]);
}

/**
  * @brief   Enable output override.
  * @param   htim  Pointer to the handle of the TIM instance.
  * @note    Outputs are forced in an override state defined in @ref HAL_TIM_OC_SetConfigChannel().
  * @warning This function can only be used when the outputs are in idle state (MOE = 0).
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_OC_EnableOutputOverride(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_OC_EnableOutputOverride(p_tim);

  return HAL_OK;
}

/**
  * @brief   Disable output override.
  * @param   htim  Pointer to the handle of the TIM instance.
  * @note    Outputs return to the default idle state.
  * @warning This function can only be used when the outputs are in idle state (MOE = 0).
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_OC_DisableOutputOverride(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_OC_DisableOutputOverride(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether the output override is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_oc_output_override_status_t  Output override status.
  */
hal_tim_oc_output_override_status_t HAL_TIM_OC_IsEnabledOutputOverride(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_oc_output_override_status_t)LL_TIM_OC_IsEnabledOutputOverride(p_tim);
}

/**
  * @brief  Program the pulse width and prescaler when the output channel
  *         operates in pulse on compare mode.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a pulse generation configuration structure.
  * @note   Pulse on compare mode is only available on channel 3 and channel 4.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_OC_SetPulseGenerator(hal_tim_handle_t *htim,
                                          const hal_tim_pulse_generator_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Make sure pulse on compare is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_PULSE_ON_COMPARE_INSTANCE(p_tim));

  /* Check pulse generator configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_PULSE_PRESCALER(p_config->prescaler));
  ASSERT_DBG_PARAM(IS_TIM_OC_PULSE_WIDTH(p_config->pulse_width));

  LL_TIM_OC_SetPulseWidthPrescaler(p_tim, (uint32_t)p_config->prescaler);

  LL_TIM_OC_SetPulseWidth(p_tim, p_config->pulse_width);

  return HAL_OK;
}

/**
  * @brief  Get the pulse width and prescaler of an output channel
  *         operating in pulse on compare mode.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a pulse generation configuration structure.
  * @note   Pulse on compare mode is only available on channel 3 and channel 4.
  */
void HAL_TIM_OC_GetPulseGenerator(const hal_tim_handle_t *htim,
                                  hal_tim_pulse_generator_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  p_config->prescaler = (hal_tim_pulse_prescaler_t)LL_TIM_OC_GetPulseWidthPrescaler(p_tim);

  p_config->pulse_width = LL_TIM_OC_GetPulseWidth(p_tim);
}

/**
  * @brief   Select on which reference signal the OC5REF (i.e. output compare of channel 5)
  *          is combined to.
  * @param   htim   Pointer to the handle of the TIM instance.
  * @param   group  This parameter can be a combination of the following values: \n
  *                 @arg @ref HAL_TIM_GROUP_NONE
  *                 @arg @ref HAL_TIM_GROUP_AND_OC1REFC
  *                 @arg @ref HAL_TIM_GROUP_AND_OC2REFC
  *                 @arg @ref HAL_TIM_GROUP_AND_OC3REFC
  *                 @arg @ref HAL_TIM_GROUP_AND_OC4REFC
  *                 @arg @ref HAL_TIM_GROUP_OR_OC1REFC
  *                 @arg @ref HAL_TIM_GROUP_OR_OC2REFC
  *                 @arg @ref HAL_TIM_GROUP_OR_OC3REFC
  *                 @arg @ref HAL_TIM_GROUP_OR_OC4REFC
  * @note    When OC5REF is grouped with OCxREF, resulting tim_ocxrefc is made of
  *          an AND/OR logical combination of two reference PWMs.
  * @warning If both HAL_TIM_GROUP_AND_OCxREFC and HAL_TIM_GROUP_OR_OCxREFC are selected,
  *          HAL_TIM_GROUP_AND_OCxREFC overrides HAL_TIM_GROUP_OR_OCxREFC programming.
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_OC_SetGroupChannel(hal_tim_handle_t *htim,
                                        uint32_t group)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports OC5REF and group is a valid combination */
  ASSERT_DBG_PARAM(IS_TIM_GROUP_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_GROUP(group));

  LL_TIM_SetCH5CombinedChannels(p_tim, group);

  return HAL_OK;
}

/**
  * @brief  Get the group configuration of OC5REF signal of timer.
  *         That is, it returns a bitfield that informs if any of the output
  *         channels 1, 2 and 3 is combined with output channel 5.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  OC5REF signal which is a combination of the following values: \n
  *                   @arg @ref HAL_TIM_GROUP_NONE
  *                   @arg @ref HAL_TIM_GROUP_AND_OC1REFC
  *                   @arg @ref HAL_TIM_GROUP_AND_OC2REFC
  *                   @arg @ref HAL_TIM_GROUP_AND_OC3REFC
  *                   @arg @ref HAL_TIM_GROUP_AND_OC4REFC
  *                   @arg @ref HAL_TIM_GROUP_OR_OC1REFC
  *                   @arg @ref HAL_TIM_GROUP_OR_OC2REFC
  *                   @arg @ref HAL_TIM_GROUP_OR_OC3REFC
  *                   @arg @ref HAL_TIM_GROUP_OR_OC4REFC
  */
uint32_t HAL_TIM_OC_GetGroupChannel(const hal_tim_handle_t *htim)
{
  /* Check the TIM handle and config allocation */
  ASSERT_DBG_PARAM((htim != NULL));

  /* Check the global state */
  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports OC5REF */
  ASSERT_DBG_PARAM(IS_TIM_GROUP_INSTANCE(p_tim));

  return LL_TIM_GetCH5CombinedChannels(p_tim);
}

/**
  * @brief  Start a timer's output channel in polling mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Output channel of interest.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_StartChannel(hal_tim_handle_t *htim,
                                     hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel)));

  /* Check and update the channel state */
  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_OC_CHANNEL_STATE_IDLE,
                         HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    LL_TIM_EnableAllOutputs(p_tim);
  }

  return HAL_OK;
}

/**
  * @brief  Stop a timer's output channel that was started in polling mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Output channel of interest.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_StopChannel(hal_tim_handle_t *htim,
                                    hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  uint32_t ll_channel = ll_tim_channels[channel];
  LL_TIM_CC_DisableChannel(p_tim, ll_channel);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    if (TIM_ARE_ALL_CHANNELS_DISABLED(p_tim))
    {
      LL_TIM_DisableAllOutputs(p_tim);
    }
  }

  htim->channel_states[channel] = HAL_TIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Start a timer's output channel in interrupt mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Output channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *                  @arg @ref HAL_TIM_CHANNEL_1N
  *                  @arg @ref HAL_TIM_CHANNEL_2N
  *                  @arg @ref HAL_TIM_CHANNEL_3N
  *                  @arg @ref HAL_TIM_CHANNEL_4N
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_StartChannel_IT(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the channel is supported by the instance and
     is not an internal channel */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel))
                   && !(IS_TIM_OC_INTERNAL_CHANNEL(channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_OC_CHANNEL_STATE_IDLE,
                         HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  /* Enable compare match interrupt */
  uint32_t it_shift = (uint32_t)channel % (uint32_t)HAL_TIM_CHANNEL_1N;
  LL_TIM_EnableIT(p_tim, LL_TIM_DIER_CC1IE << it_shift);

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    LL_TIM_EnableAllOutputs(p_tim);
  }

  return HAL_OK;
}

/**
  * @brief  Stop a timer's output channel that was started in interrupt mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Output channel of interest.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_StopChannel_IT(hal_tim_handle_t *htim,
                                       hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the channel is supported by the instance and
     is not an internal channel */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel))
                   && !(IS_TIM_OC_INTERNAL_CHANNEL(channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  /* Disable compare match interrupt */
  uint32_t it_shift = (uint32_t)channel % (uint32_t)HAL_TIM_CHANNEL_1N;
  LL_TIM_DisableIT(p_tim, LL_TIM_DIER_CC1IE << it_shift);

  LL_TIM_CC_DisableChannel(p_tim, ll_tim_channels[channel]);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    if (TIM_ARE_ALL_CHANNELS_DISABLED(p_tim))
    {
      LL_TIM_DisableAllOutputs(p_tim);
    }
  }

  htim->channel_states[channel] = HAL_TIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Start a timer's Output Channel in DMA mode (default DMA interrupts).
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  channel    Output channel of interest. \n
  *                    Must be one of the following values: \n
  *                    @arg @ref HAL_TIM_CHANNEL_1
  *                    @arg @ref HAL_TIM_CHANNEL_2
  *                    @arg @ref HAL_TIM_CHANNEL_3
  *                    @arg @ref HAL_TIM_CHANNEL_4
  *                    @arg @ref HAL_TIM_CHANNEL_1N
  *                    @arg @ref HAL_TIM_CHANNEL_2N
  *                    @arg @ref HAL_TIM_CHANNEL_3N
  *                    @arg @ref HAL_TIM_CHANNEL_4N
  * @param  p_data     Pointer to the data buffer.
  * @param  size_byte  Data buffer size (in byte).
  * @note   One data will be transferred from the buffer to the compare
  *         register (TIMx_CCRy) at each compare match. \n
  *         DMA transfer ends when all the data of the buffer have been transferred.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_OC_StartChannel_DMA(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const uint8_t *p_data,
                                         uint32_t size_byte)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  /* Check if the channel is supported by the instance and
     is not an internal channel */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(TIM_INSTANCE(htim), channel))
                   && !(IS_TIM_OC_INTERNAL_CHANNEL(channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_OC_CHANNEL_STATE_IDLE,
                         HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  return TIM_OC_StartChannel_DMA_Opt(htim, channel, p_data, size_byte, HAL_TIM_OPT_DMA_IT_DEFAULT);
}

/**
  * @brief  Start a timer's Output Channel in DMA mode (optional DMA interrupts).
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  channel     Output channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_TIM_CHANNEL_1
  *                     @arg @ref HAL_TIM_CHANNEL_2
  *                     @arg @ref HAL_TIM_CHANNEL_3
  *                     @arg @ref HAL_TIM_CHANNEL_4
  *                     @arg @ref HAL_TIM_CHANNEL_1N
  *                     @arg @ref HAL_TIM_CHANNEL_2N
  *                     @arg @ref HAL_TIM_CHANNEL_3N
  *                     @arg @ref HAL_TIM_CHANNEL_4N
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in byte).
  * @param  interrupts  Selection of the DMA interrupts. \n
  *                     Can be any of the (meaningful) ored values: \n
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_NONE
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_HT
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_SILENT
  * @endif
  * @note   One data will be transferred from the buffer to the compare
  *         register (TIMx_CCRy) at each compare match. \n
  *         DMA transfer ends when all the data of the buffer have been transferred.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_OC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel,
                                             const uint8_t *p_data,
                                             uint32_t size_byte,
                                             uint32_t interrupts)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  /* Check if the channel is supported by the instance and
     is not an internal channel */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(TIM_INSTANCE(htim), channel))
                   && !(IS_TIM_OC_INTERNAL_CHANNEL(channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_IDLE);

  /* Move to state HAL_TIM_OC_CHANNEL_STATE_ACTIVE or
     HAL_TIM_OC_CHANNEL_STATE_ACTIVE_SILENT */
  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_OC_CHANNEL_STATE_IDLE,
                         TIM_OC_CHANNEL_STATE_ACTIVE(interrupts));

  return TIM_OC_StartChannel_DMA_Opt(htim, channel, p_data, size_byte, interrupts);
}

/**
  * @brief  Stop a timer's output channel that was started in DMA mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Output Channel of interest.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_OC_StopChannel_DMA(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(p_tim));

  /* Check if the channel is supported by the instance and
     is not an internal channel */
  ASSERT_DBG_PARAM((IS_TIM_OC_CHANNEL(p_tim, channel))
                   && !(IS_TIM_OC_INTERNAL_CHANNEL(channel)));

  /* Ensure that the channel is in active or active silent mode */
  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_OC_CHANNEL_STATE_ACTIVE);

  /* Stop DMA transfer and disable compare match DMA request */
  if (TIM_StopChannel_DMA(htim, p_tim, channel,
                          (IS_TIM_ACTIVE_SILENT(htim->channel_states[channel]))) != HAL_OK)
  {
    return HAL_ERROR;
  }

  uint32_t ll_channel = ll_tim_channels[channel];
  LL_TIM_CC_DisableChannel(p_tim, ll_channel);

  if (IS_TIM_BREAK_INSTANCE(p_tim))
  {
    if (TIM_ARE_ALL_CHANNELS_DISABLED(p_tim))
    {
      LL_TIM_DisableAllOutputs(p_tim);
    }
  }

  htim->channel_states[channel] = HAL_TIM_OC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group5
  *  @{
  */

/**
  * @brief   Configure an input channel.
  * @param   htim      Pointer to the handle of the TIM instance.
  * @param   channel   Input channel to configure. \n
  *                    Must be one of the following values: \n
  *                    @arg @ref HAL_TIM_CHANNEL_1
  *                    @arg @ref HAL_TIM_CHANNEL_2
  *                    @arg @ref HAL_TIM_CHANNEL_3
  *                    @arg @ref HAL_TIM_CHANNEL_4
  * @param   p_config  Pointer to an input channel configuration structure.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_IC_SetConfigChannel(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const hal_tim_ic_channel_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_CHANNEL_STATE_RESET | TIM_CHANNEL_STATE_IDLE));

  /* Check channel configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_CHANNEL_SRC(p_tim, channel, p_config->source));
  ASSERT_DBG_PARAM(IS_TIM_IC_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_TIM_FILTER(p_config->filter));

  const uint32_t ll_channel = ll_tim_channels[channel];
  const hal_tim_channel_src_t source = p_config->source;

  /* Configure the channel */
  TIM_SetRemap(p_tim, channel, source);

  LL_TIM_IC_SetPolarity(p_tim, ll_channel, (uint32_t)p_config->polarity);

  LL_TIM_IC_SetFilter(p_tim, ll_channel, TIM_IC_HAL2LL_FILTER(p_config->filter));

  htim->channel_states[channel] = HAL_TIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of an input channel.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  channel   Input channel of interest \n
  *                   Must be one of the following values: \n
  *                   @arg @ref HAL_TIM_CHANNEL_1
  *                   @arg @ref HAL_TIM_CHANNEL_2
  *                   @arg @ref HAL_TIM_CHANNEL_3
  *                   @arg @ref HAL_TIM_CHANNEL_4
  * @param  p_config  Pointer to an input channel configuration structure.
  */
void HAL_TIM_IC_GetConfigChannel(const hal_tim_handle_t *htim,
                                 hal_tim_channel_t channel,
                                 hal_tim_ic_channel_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_CHANNEL_STATE_RESET | HAL_TIM_IC_CHANNEL_STATE_IDLE \
                    | HAL_TIM_IC_CHANNEL_STATE_ACTIVE));

  uint32_t ll_channel = ll_tim_channels[channel];

  p_config->source = (hal_tim_channel_src_t)LL_TIM_IC_GetSource(p_tim, ll_channel);

  p_config->polarity = (hal_tim_ic_polarity_t)LL_TIM_IC_GetPolarity(p_tim,
                                                                    ll_channel);

  uint32_t ll_filter = LL_TIM_IC_GetFilter(p_tim, ll_channel);
  p_config->filter = TIM_IC_LL2HAL_FILTER(ll_filter);
}

/**
  * @brief  Configure the source of an input channel.
  * @param  htim         Pointer to the handle of the TIM instance.
  * @param  channel      Input channel of interest \n
  *                      Must be one of the following values: \n
  *                      @arg @ref HAL_TIM_CHANNEL_1
  *                      @arg @ref HAL_TIM_CHANNEL_2
  *                      @arg @ref HAL_TIM_CHANNEL_3
  *                      @arg @ref HAL_TIM_CHANNEL_4
  * @param  channel_src  Input source for the channel.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_SetChannelSource(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         hal_tim_channel_src_t channel_src)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  /* Check the source is valid for the instance */
  ASSERT_DBG_PARAM(IS_TIM_CHANNEL_SRC(p_tim, channel, channel_src));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  TIM_SetRemap(p_tim, channel, channel_src);

  return HAL_OK;
}

/**
  * @brief  Get the source of a input channel.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval hal_tim_channel_src_t  The input source for the channel.
  */
hal_tim_channel_src_t HAL_TIM_IC_GetChannelSource(const hal_tim_handle_t *htim,
                                                  hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE | HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  return (hal_tim_channel_src_t)LL_TIM_IC_GetSource(p_tim, ll_tim_channels[channel]);
}

/**
  * @brief   Configure a capture unit.
  * @param   htim          Pointer to the handle of the TIM instance.
  * @param   capture_unit  Identify the capture unit to configure.
  * @param   p_config      Pointer to a capture unit configuration structure.
  * @warning When the adjacent timer input channel is selected as the source of the capture unit
  *          (i.e. input channel 1 is captured by capture unit 2) then the polarity of the adjacent
  *          input channel is overwritten by this function as per the source field value.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM  Input parameter is invalid
  *                             (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_IC_SetConfigCaptureUnit(hal_tim_handle_t *htim,
                                             hal_tim_ic_capture_unit_t capture_unit,
                                             const hal_tim_ic_capture_unit_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the capture unit is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_IC_CAPTURE_UNIT(p_tim, capture_unit));

  /* Check the config parameters */
  ASSERT_DBG_PARAM((IS_TIM_IC_CAPTURE_UNIT_SRC(p_config->source)));
  ASSERT_DBG_PARAM((IS_TIM_IC_CAPTURE_UNIT_PRESCALER(p_config->prescaler)));

  hal_tim_ic_capture_unit_src_t source = p_config->source;
  LL_TIM_IC_SetActiveInput(p_tim, (uint32_t)capture_unit,
                           TIM_LL_ACTIVE_INPUT((uint32_t)source));

  if ((source != HAL_TIM_IC_DIRECT) && (source != HAL_TIM_IC_TRC))
  {
    LL_TIM_IC_SetPolarity(p_tim, (uint32_t)capture_unit,
                          TIM_LL_IC_POLARITY((uint32_t)source));
  }

  LL_TIM_IC_SetPrescaler(p_tim, (uint32_t)capture_unit,
                         (uint32_t)p_config->prescaler);

  return HAL_OK;
}

/**
  * @brief  Get a capture unit configuration.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  capture_unit  Identify the capture unit.
  * @param  p_config      Pointer to a capture unit configuration structure.
  */
void HAL_TIM_IC_GetConfigCaptureUnit(const hal_tim_handle_t *htim,
                                     hal_tim_ic_capture_unit_t capture_unit,
                                     hal_tim_ic_capture_unit_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the capture unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CAPTURE_UNIT(p_tim, capture_unit)));

  uint32_t source = LL_TIM_IC_GetActiveInput(p_tim, (uint32_t)capture_unit);
  if ((source != (uint32_t)HAL_TIM_IC_DIRECT)
      && (source != (uint32_t)HAL_TIM_IC_TRC))
  {
    source |= LL_TIM_IC_GetPolarity(p_tim, (uint32_t)capture_unit);
  }
  p_config->source = (hal_tim_ic_capture_unit_src_t)source;

  p_config->prescaler = (hal_tim_ic_capture_unit_prescaler_t)LL_TIM_IC_GetPrescaler(p_tim,
                        (uint32_t)capture_unit);
}

/**
  * @brief  Configure the XOR gate position.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  xor_position  XOR gate position.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_SetXORGatePosition(hal_tim_handle_t *htim,
                                           hal_tim_ic_xor_gate_position_t xor_position)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate and has at least 3 channels */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim) && IS_TIM_CC3_INSTANCE(p_tim)));

  /* Check the XOR gate position is valid */
  ASSERT_DBG_PARAM((IS_TIM_XOR_GATE_POSITION(xor_position)));

  LL_TIM_IC_SetXORGatePosition(p_tim, (uint32_t)xor_position);

  return HAL_OK;
}

/**
  * @brief  Get the XOR gate position configuration.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_ic_xor_gate_position_t  The XOR gate position.
  */
hal_tim_ic_xor_gate_position_t HAL_TIM_IC_GetXORGatePosition(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate and has at least 3 channels */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim) && IS_TIM_CC3_INSTANCE(p_tim)));

  return (hal_tim_ic_xor_gate_position_t)LL_TIM_IC_GetXORGatePosition(p_tim);
}

/**
  * @brief  Enable the signal inversion of a XOR gate input channel.
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  channel    Input channel to configure. \n
  *                    Must be one of the following values: \n
  *                    @arg @ref HAL_TIM_CHANNEL_1
  *                    @arg @ref HAL_TIM_CHANNEL_2
  *                    @arg @ref HAL_TIM_CHANNEL_3
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_EnableXORGateInputInversion(hal_tim_handle_t *htim, hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  /* Check the channel is supported by the instance and is a XOR input */
  ASSERT_DBG_PARAM((IS_TIM_XOR_GATE_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  LL_TIM_IC_EnableXORGateInputInversion(p_tim, ll_tim_channels[channel]);

  return HAL_OK;
}

/**
  * @brief  Disable the signal inversion of a XOR gate input channel.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel to configure. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_DisableXORGateInputInversion(hal_tim_handle_t *htim, hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  /* Check the channel is supported by the instance and is a XOR input */
  ASSERT_DBG_PARAM((IS_TIM_XOR_GATE_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  LL_TIM_IC_DisableXORGateInputInversion(p_tim, ll_tim_channels[channel]);

  return HAL_OK;
}

/**
  * @brief  Tell whether the XOR gate input channel signal is inverted or not.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel to configure. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  * @retval hal_tim_ic_xor_gate_input_inversion_status_t  The XOR gate inversion status for the input channel.
  */
hal_tim_ic_xor_gate_input_inversion_status_t HAL_TIM_IC_IsEnabledXORGateInputInversion(const hal_tim_handle_t *htim,
    hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  /* Check the channel is supported by the instance and is a XOR input */
  ASSERT_DBG_PARAM((IS_TIM_XOR_GATE_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE | HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  return (hal_tim_ic_xor_gate_input_inversion_status_t)LL_TIM_IC_IsEnabledXORGateInputInversion(p_tim,
         ll_tim_channels[channel]);
}

/**
  * @brief  Enable the XOR gate.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_EnableXORGate(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  /* Make sure all channels connected to the XOR gate are in IDLE state */
  ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_1],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_2],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);
  if (IS_TIM_CC3_INSTANCE(p_tim))
  {
    ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_3],
                     HAL_TIM_IC_CHANNEL_STATE_IDLE);
  }

  LL_TIM_IC_EnableXORCombination(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable the XOR gate.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_DisableXORGate(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  /* Make sure all channels connected to the XOR gate are in IDLE state */
  ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_1],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);
  ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_2],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);
  if (IS_TIM_CC3_INSTANCE(p_tim))
  {
    ASSERT_DBG_STATE(htim->channel_states[HAL_TIM_CHANNEL_3],
                     HAL_TIM_IC_CHANNEL_STATE_IDLE);
  }

  LL_TIM_IC_DisableXORCombination(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether XOR gate is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_ic_xor_gate_status_t  XOR gate status of timer.
  */
hal_tim_ic_xor_gate_status_t HAL_TIM_IC_IsEnabledXORGate(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that the instance supports XOR gate */
  ASSERT_DBG_PARAM((IS_TIM_XOR_INSTANCE(p_tim)));

  return (hal_tim_ic_xor_gate_status_t)LL_TIM_IC_IsEnabledXORCombination(p_tim);
}

/**
  * @brief  Read the captured value for an input channel of timer.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval uint32_t  Captured value
  */
uint32_t HAL_TIM_IC_ReadChannelCapturedValue(const hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_IC_CHANNEL_STATE_IDLE | HAL_TIM_IC_CHANNEL_STATE_ACTIVE));

  return LL_TIM_IC_GetCapturedValue(p_tim, ll_tim_channels[channel]);
}

/**
  * @brief  Indicate the input signal level of a channel (after the digital filtering stage),
  *         for polling purpose.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval hal_tim_ic_channel_level_t  Input channel signal level
  */
hal_tim_ic_channel_level_t HAL_TIM_IC_GetChannelLevel(const hal_tim_handle_t *htim,
                                                      hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   (HAL_TIM_IC_CHANNEL_STATE_IDLE | HAL_TIM_IC_CHANNEL_STATE_ACTIVE));

  return (hal_tim_ic_channel_level_t)LL_TIM_IC_GetInputStatus(p_tim, ll_tim_channels[channel]);
}

/**
  * @brief  Start a timer's input channel in polling mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_StartChannel(hal_tim_handle_t *htim,
                                     hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_IC_CHANNEL_STATE_IDLE,
                         HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  return HAL_OK;
}

/**
  * @brief  Stop a timer's input channel that was started in polling mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_StopChannel(hal_tim_handle_t *htim,
                                    hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  LL_TIM_CC_DisableChannel(p_tim, ll_tim_channels[channel]);

  htim->channel_states[channel] = HAL_TIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

/**
  * @brief  Start a timer's input channel in interrupt mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_StartChannel_IT(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_IC_CHANNEL_STATE_IDLE,
                         HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  /* Enable capture interrupt */
  LL_TIM_EnableIT(p_tim, LL_TIM_DIER_CC1IE << (uint32_t)channel);

  LL_TIM_CC_EnableChannel(p_tim, ll_tim_channels[channel]);

  return HAL_OK;
}

/**
  * @brief  Stop a timer's input channel that was started in interrupt mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_StopChannel_IT(hal_tim_handle_t *htim,
                                       hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  /* Disable capture interrupt */
  LL_TIM_DisableIT(p_tim, LL_TIM_DIER_CC1IE << (uint32_t)channel);

  LL_TIM_CC_DisableChannel(p_tim, ll_tim_channels[channel]);

  htim->channel_states[channel] = HAL_TIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Start a timer's Input Channel in DMA mode (default DMA interrupts).
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  channel    Input channel of interest. \n
  *                    Must be one of the following values: \n
  *                    @arg @ref HAL_TIM_CHANNEL_1
  *                    @arg @ref HAL_TIM_CHANNEL_2
  *                    @arg @ref HAL_TIM_CHANNEL_3
  *                    @arg @ref HAL_TIM_CHANNEL_4
  * @param  p_data     Pointer to the data buffer.
  * @param  size_byte  Data buffer size (in bytes).
  * @note   One data will be transferred from the capture register (TIMx_CCRy) to
  *         the buffer at each capture event. \n
  *         DMA transfer ends when all the data have been transferred to the buffer.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_IC_StartChannel_DMA(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         uint8_t *p_data,
                                         uint32_t size_byte)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(TIM_INSTANCE(htim), channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_IC_CHANNEL_STATE_IDLE,
                         HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  return TIM_IC_StartChannel_DMA_Opt(htim, channel, p_data, size_byte, HAL_TIM_OPT_DMA_IT_DEFAULT);
}

/**
  * @brief  Start a timer's Input Channel in DMA mode (optional DMA interrupts).
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  channel     Input channel of interest. \n
  *                     Must be one of the following values: \n
  *                     @arg @ref HAL_TIM_CHANNEL_1
  *                     @arg @ref HAL_TIM_CHANNEL_2
  *                     @arg @ref HAL_TIM_CHANNEL_3
  *                     @arg @ref HAL_TIM_CHANNEL_4
  * @param  p_data      Pointer to the data buffer.
  * @param  size_byte   Data buffer size (in bytes).
  * @param  interrupts  Selection of the DMA interrupts. \n
  *                     Can be any of the (meaningful) ored values: \n
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_NONE
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_HT
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_DEFAULT
  * @if USE_HAL_DMA_LINKEDLIST
  *                     @arg @ref HAL_TIM_OPT_DMA_IT_SILENT
  * @endif
  * @note   One data will be transferred from the capture register (TIMx_CCRy) to
  *         the buffer at each capture event. \n
  *         DMA transfer ends when all the data have been transferred to the buffer.
  * @retval HAL_OK
  * @retval HAL_ERROR          Failed to start the DMA transfer.
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_IC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel,
                                             uint8_t *p_data,
                                             uint32_t size_byte,
                                             uint32_t interrupts)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(TIM_INSTANCE(htim)));

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(TIM_INSTANCE(htim), channel)));

  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_IDLE);

  /* Move to state HAL_TIM_IC_CHANNEL_STATE_ACTIVE or
     HAL_TIM_IC_CHANNEL_STATE_ACTIVE_SILENT */
  HAL_CHECK_UPDATE_STATE(htim, channel_states[channel],
                         HAL_TIM_IC_CHANNEL_STATE_IDLE,
                         TIM_IC_CHANNEL_STATE_ACTIVE(interrupts));

  return TIM_IC_StartChannel_DMA_Opt(htim, channel, p_data, size_byte, interrupts);
}

/**
  * @brief  Stop a timer's input channel that was started in DMA mode.
  * @param  htim     Pointer to the handle of the TIM instance.
  * @param  channel  Input channel of interest. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_IC_StopChannel_DMA(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that DMA is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_DMA_INSTANCE(p_tim));

  /* Check the channel is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_IC_CHANNEL(p_tim, channel)));

  /* Ensure that the channel is in active or active silent mode */
  ASSERT_DBG_STATE(htim->channel_states[channel],
                   HAL_TIM_IC_CHANNEL_STATE_ACTIVE);

  /* Stop DMA transfer and disable capture DMA request */
  if (TIM_StopChannel_DMA(htim, p_tim, channel,
                          (IS_TIM_ACTIVE_SILENT(htim->channel_states[channel]))) != HAL_OK)
  {
    return HAL_ERROR;
  }

  LL_TIM_CC_DisableChannel(p_tim, ll_tim_channels[channel]);

  htim->channel_states[channel] = HAL_TIM_IC_CHANNEL_STATE_IDLE;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_USER_DATA */
/**
  * @}
  */


/** @addtogroup TIM_Exported_Functions_Group6
  *  @{
  */

/**
  * @brief  Enable the one-pulse mode of timer (single pulse).
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableOnePulseMode(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_EnableOnePulseMode(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Disable the one-pulse mode of timer.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableOnePulseMode(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_DisableOnePulseMode(TIM_INSTANCE(htim));

  return HAL_OK;
}

/**
  * @brief  Tell whether one-pulse mode is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_one_pulse_mode_status_t  One-pulse mode status.
  */
hal_tim_one_pulse_mode_status_t HAL_TIM_IsEnabledOnePulseMode(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_one_pulse_mode_status_t)LL_TIM_IsEnabledOnePulseMode(TIM_INSTANCE(htim));
}

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group7
  *  @{
  */

/**
  * @brief  Configure the index input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to the encoder index configuration structure.
  * @note   The index input is a pulse coming from an encoder.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetConfigEncoderIndex(hal_tim_handle_t *htim,
                                           const hal_tim_encoder_index_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Check configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INDEX_DIR(p_config->dir));
  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INDEX_POS_SEL(p_config->pos));
  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INDEX_BLANK_MODE(p_config->blanking));
  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INDEX_SEL(p_config->idx));

  uint32_t encoder_idx_cfg = (uint32_t)p_config->dir | (uint32_t)p_config->pos |
                             (uint32_t)p_config->blanking | (uint32_t)p_config->idx;

  LL_TIM_ConfigEncoderIndex(p_tim, encoder_idx_cfg);

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the index input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to an encoder index configuration structure.
  */
void HAL_TIM_GetConfigEncoderIndex(const hal_tim_handle_t *htim,
                                   hal_tim_encoder_index_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  uint32_t ecr = LL_TIM_READ_REG(p_tim, ECR);

  p_config->dir = (hal_tim_encoder_index_dir_t)(uint32_t)(ecr & TIM_ECR_IDIR);

  p_config->pos = (hal_tim_encoder_index_pos_sel_t)(uint32_t)(ecr & TIM_ECR_IPOS);

  p_config->blanking = (hal_tim_encoder_index_blank_mode_t)(uint32_t)(ecr & TIM_ECR_IBLK);

  p_config->idx = (hal_tim_encoder_index_sel_t)(uint32_t)(ecr & TIM_ECR_FIDX);
}

/**
  * @brief  Enable the index input.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   when the index input is enabled, the encoder index signal
  *         connected to the timer's external trigger can reset the counter
  *         as per index input configuration.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableEncoderIndex(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_EnableEncoderIndex(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable the index input.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableEncoderIndex(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_DisableEncoderIndex(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether index input is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_encoder_index_status_t  Encoder index status.
  */
hal_tim_encoder_index_status_t HAL_TIM_IsEnabledEncoderIndex(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_ENCODER_INTERFACE_INSTANCE(p_tim));
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  return (hal_tim_encoder_index_status_t)LL_TIM_IsEnabledEncoderIndex(p_tim);
}

/**
  * @}
  */


/** @addtogroup TIM_Exported_Functions_Group8
  *  @{
  */

/**
  * @brief  Configure the external trigger input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to an external trigger input configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetExternalTriggerInput(hal_tim_handle_t *htim,
                                             const hal_tim_ext_trig_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check ETR source selection is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_EXT_TRIG_SRC(p_tim, p_config->source));
  ASSERT_DBG_PARAM(IS_TIM_EXT_TRIG_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_TIM_EXT_TRIG_PRESCALER(p_config->prescaler));
  ASSERT_DBG_PARAM(IS_TIM_EXT_TRIG_SYNC_PRESCALER(p_config->sync_prescaler));
  ASSERT_DBG_PARAM(IS_TIM_FILTER(p_config->filter));

  LL_TIM_SetETRSource(p_tim, (uint32_t)p_config->source);

  uint32_t presc = (uint32_t)(p_config->prescaler) | (uint32_t)(p_config->sync_prescaler);

  LL_TIM_ConfigETR(p_tim,
                   (uint32_t)p_config->polarity,
                   presc,
                   TIM_ETR_HAL2LL_FILTER(p_config->filter));

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the external trigger input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to an external trigger input configuration structure.
  */
void HAL_TIM_GetExternalTriggerInput(const hal_tim_handle_t *htim,
                                     hal_tim_ext_trig_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check ETR source selection is supported by the instance */
  ASSERT_DBG_PARAM(IS_TIM_ETR_INSTANCE(p_tim));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  p_config->source = (hal_tim_ext_trig_src_t)LL_TIM_GetETRSource(p_tim);

  uint32_t polarity;
  uint32_t prescaler;
  uint32_t filter;

  LL_TIM_GetConfigETR(p_tim, &polarity, &prescaler, &filter);

  p_config->polarity = (hal_tim_ext_trig_polarity_t)polarity;
  p_config->prescaler = (hal_tim_ext_trig_prescaler_t)((uint32_t)(prescaler & TIM_SMCR_ETPS));
  p_config->sync_prescaler = (hal_tim_ext_trig_sync_prescaler_t)((uint32_t)(prescaler & TIM_SMCR_SETPS));
  p_config->filter = TIM_ETR_LL2HAL_FILTER(filter);
}

/**
  * @}
  */


/** @addtogroup TIM_Exported_Functions_Group9
  *  @{
  */

/**
  * @brief  Configure the slave mode controller.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a slave mode controller configuration structure.
  * @note   The selection of the event triggering the transfer of the preloaded
  *         slave mode configuration to the active register is done with
  *         @ref HAL_TIM_EnableSlaveModePreload().
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetSynchroSlave(hal_tim_handle_t *htim,
                                     const hal_tim_slave_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_INSTANCE(p_tim)));

  /* Check configuration parameters */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_MODE(p_config->mode)));
  ASSERT_DBG_PARAM((IS_TIM_TRIG_SEL(p_tim, p_config->trigger)));

  /* Make sure that a pulse trigger is not used
     in gated or combined gated + reset mode   */
  ASSERT_DBG_PARAM(IS_TIM_SLAVE_MODE_TRIGGER_VALID(p_config->mode,
                                                   p_config->trigger));

  LL_TIM_SetSlaveMode(p_tim, (uint32_t)p_config->mode);

  LL_TIM_SetTriggerInput(p_tim, (uint32_t)p_config->trigger);

  return HAL_OK;
}

/**
  * @brief  Get the slave mode controller configuration.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to the slave mode controller configuration structure.
  */
void HAL_TIM_GetSynchroSlave(const hal_tim_handle_t *htim,
                             hal_tim_slave_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_INSTANCE(p_tim)));

  p_config->mode = (hal_tim_slave_mode_t)LL_TIM_GetSlaveMode(p_tim);

  p_config->trigger = (hal_tim_trig_sel_t)LL_TIM_GetTriggerInput(p_tim);
}

/**
  * @brief  Set the trigger output source of master mode controller.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  trgo_src  Trigger-output source.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetTriggerOutput(hal_tim_handle_t *htim,
                                      hal_tim_trigger_output_source_t trgo_src)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a master timer */
  ASSERT_DBG_PARAM((IS_TIM_MASTER_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM(IS_TIM_TRIGGER_OUTPUT_SOURCE(trgo_src));

  LL_TIM_SetTriggerOutput(p_tim, (uint32_t)trgo_src);

  return HAL_OK;
}

/**
  * @brief  Get the trigger output source of the master mode controller configuration.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_trigger_output_source_t  Trigger-output source.
  */
hal_tim_trigger_output_source_t HAL_TIM_GetTriggerOutput(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a master timer */
  ASSERT_DBG_PARAM((IS_TIM_MASTER_INSTANCE(p_tim)));

  return (hal_tim_trigger_output_source_t)LL_TIM_GetTriggerOutput(p_tim);
}

/**
  * @brief   Configure the trigger output2.
  * @param   htim      Pointer to the handle of the TIM instance.
  * @param   p_config  Pointer to a trigger output2 configuration structure.
  * @warning The postscaler is only applicable when tim_trgo2 transfers a pulse
  *          (reset, update, compare pulse). When the tim_trgo2 outputs a level-based information
  *          (compare signal or enable bit for gated mode), the postscaler must be set set to 0.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM  Input parameter is invalid
  *                             (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetConfigTriggerOutput2(hal_tim_handle_t *htim,
                                             hal_tim_trigger_output2_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM(IS_TIM_TRIGGER_OUTPUT2_SOURCE(p_config->trgo2_src));

  ASSERT_DBG_PARAM(IS_TIM_TRIGGER_OUTPUT2_PSC(p_config->postscaler));

  LL_TIM_SetTriggerOutput2(p_tim, (uint32_t)p_config->trgo2_src);

  /* Set trgo2 postscaler only for specific sources */
  if (IS_TIM_TRIGGER_OUTPUT2_PSC_SOURCE(p_config->trgo2_src))
  {
    LL_TIM_SetTriggerOutput2Postscaler(p_tim, p_config->postscaler);
  }
  else
  {
    LL_TIM_SetTriggerOutput2Postscaler(p_tim, 0U);
  }

  return HAL_OK;
}

/**
  * @brief  Get the trigger output2 configuration.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a trgo2 configuration structure.
  */
void HAL_TIM_GetConfigTriggerOutput2(const hal_tim_handle_t *htim,
                                     hal_tim_trigger_output2_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  p_config->trgo2_src = (hal_tim_trigger_output2_source_t)LL_TIM_GetTriggerOutput2(p_tim);

  p_config->postscaler = LL_TIM_GetTriggerOutput2Postscaler(p_tim);
}

/**
  * @brief  Set the trigger output2 source of the master mode controller.
  * @param  htim       Pointer to the handle of the TIM instance.
  * @param  trgo2_src  Trigger-output2 source.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetTriggerOutput2(hal_tim_handle_t *htim,
                                       hal_tim_trigger_output2_source_t trgo2_src)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM(IS_TIM_TRIGGER_OUTPUT2_SOURCE(trgo2_src));

  LL_TIM_SetTriggerOutput2(p_tim, (uint32_t)trgo2_src);

  return HAL_OK;
}

/**
  * @brief  Get the trigger output2 source of the master mode controller.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_trigger_output2_source_t  Trigger-output2 source.
  */
hal_tim_trigger_output2_source_t HAL_TIM_GetTriggerOutput2(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  return (hal_tim_trigger_output2_source_t)LL_TIM_GetTriggerOutput2(p_tim);
}

/**
  * @brief   Set the trigger output2 postscaler of the master mode controller.
  * @param   htim        Pointer to the handle of the TIM instance.
  * @param   postscaler  Trigger-output2 postscaler (number between 0x00 and 0x1F).
  * @warning The postscaler is only applicable when tim_trgo2 transfers a pulse
  *          (reset, update, compare pulse). When the tim_trgo2 outputs a level-based information
  *          (compare signal or enable bit for gated mode), the postscaler must be set set to 0.
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_SetTriggerOutput2Postscaler(hal_tim_handle_t *htim,
                                                 uint32_t postscaler)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM(IS_TIM_TRIGGER_OUTPUT2_PSC(postscaler));

  LL_TIM_SetTriggerOutput2Postscaler(p_tim, postscaler);

  return HAL_OK;
}

/**
  * @brief  Get the trigger output2 postscaler of the master mode controller.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval uint32_t  Trigger-output2 postscaler (number between 0x00 and 0x1F).
  */
uint32_t HAL_TIM_GetTriggerOutput2Postscaler(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_TRGO2_INSTANCE(p_tim)));

  return LL_TIM_GetTriggerOutput2Postscaler(p_tim);
}

/**
  * @brief  Enable slave mode preload.
  * @param  htim         Pointer to the handle of the TIM instance.
  * @param  preload_src  Slave mode preload source.
  * @note   When slave mode preload is enabled, slave mode selection
  *         (TIMx_SMCR.SMS) preload value isn't taken into account immediately. \n
  *         It is loaded in the active register at next update event or at next
  *         index event as per chosen slave mode preload source.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableSlaveModePreload(hal_tim_handle_t *htim,
                                            const hal_tim_slave_mode_preload_src_t preload_src)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SMS_PRELOAD_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM(IS_TIM_SLAVE_MODE_PRELOAD_SRC(preload_src));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_SetSMSPreloadSource(p_tim, (uint32_t)preload_src);

  LL_TIM_EnableSMSPreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable slave mode preload.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableSlaveModePreload(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SMS_PRELOAD_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_DisableSMSPreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether slave mode preload is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_slave_mode_preload_status_t  Slave Preload Status.
  */
hal_tim_slave_mode_preload_status_t HAL_TIM_IsEnabledSlaveModePreload(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SMS_PRELOAD_INSTANCE(p_tim)));

  return (hal_tim_slave_mode_preload_status_t)LL_TIM_IsEnabledSMSPreload(p_tim);
}

/**
  * @brief  Enable master-slave mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   When the Master/slave mode is enabled, the effect of an event on the
  *         trigger input (TRGI) is delayed to allow a perfect synchronization
  *         between the current timer and its slaves (through TRGO).
  *         It is not mandatory in case of timer synchronization mode.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableMasterSlaveMode(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_INSTANCE(p_tim)));

  LL_TIM_EnableMasterSlaveMode(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable master-slave mode.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableMasterSlaveMode(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_INSTANCE(p_tim)));

  LL_TIM_DisableMasterSlaveMode(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether Master/Slave mode is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_master_slave_mode_status_t  Master-slave mode status.
  */
hal_tim_master_slave_mode_status_t HAL_TIM_IsEnabledMasterSlaveMode(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check the instance can operate as a slave timer */
  ASSERT_DBG_PARAM((IS_TIM_SLAVE_INSTANCE(p_tim)));

  return (hal_tim_master_slave_mode_status_t)LL_TIM_IsEnabledMasterSlaveMode(p_tim);
}

/**
  * @brief   Enable ADC synchronization.
  * @param   htim  Pointer to the handle of the TIM instance.
  * @note    It is mandatory to follow the procedure below to use the ADC synchronization:
  *           1. Enable the destination ADC clock.
  *           2. Configure the timer and set the ADSYNC bit.
  *           3. Configure the ADC and enable it (using ADSTART and/or JADSTART bits).
  *           4. Start the timer (with the CEN counter enable bit).
  * @warning The ADC synchronization feature must not be modified during run-time.
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_EnableADCSynchronization(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_ADC_SYNCHRO_INSTANCE(p_tim)));

  LL_TIM_EnableADCSynchronization(p_tim);

  return HAL_OK;
}

/**
  * @brief   Disable ADC synchronization.
  * @param   htim  Pointer to the handle of the TIM instance.
  * @warning The ADC synchronization feature must not be modified during run-time.
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_DisableADCSynchronization(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_ADC_SYNCHRO_INSTANCE(p_tim)));

  LL_TIM_DisableADCSynchronization(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether ADC synchronization is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_adc_synchronization_status_t  ADC synchronization status.
  */
hal_tim_adc_synchronization_status_t HAL_TIM_IsEnabledADCSynchronization(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_ADC_SYNCHRO_INSTANCE(p_tim)));

  return (hal_tim_adc_synchronization_status_t)LL_TIM_IsEnabledADCSynchronization(p_tim);
}

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group10
  *  @{
  */

/**
  * @brief   Set the OCRef clear source.
  * @param   htim    Pointer to the handle of the TIM instance.
  * @param   source  OCRef clear source.
  * @warning This function can only be used in specific output modes (output compare and PWM)
  *          configured in @ref HAL_TIM_OC_SetConfigCompareUnit().
  * @retval  HAL_OK
  */
hal_status_t HAL_TIM_SetOCRefClearSource(hal_tim_handle_t *htim,
                                         hal_tim_ocref_clr_src_t source)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Control that OCRefClear is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OCXREF_CLEAR_INSTANCE(p_tim)));

  ASSERT_DBG_PARAM((IS_TIM_OCREF_CLR_SRC(p_tim, source)));

  LL_TIM_SetOCRefClearInputSource(p_tim, (uint32_t)source);

  return HAL_OK;
}

/**
  * @brief  Get the OCRef clear source.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_ocref_clr_src_t  OCRef clear source.
  */
hal_tim_ocref_clr_src_t HAL_TIM_GetOCRefClearSource(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Control that OCRefClear is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OCXREF_CLEAR_INSTANCE(p_tim)));

  return (hal_tim_ocref_clr_src_t)LL_TIM_GetOCRefClearInputSource(p_tim);
}

/**
  * @brief  Enable clearing of the OCxRef signal by the OCRef clear input.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableCompareUnitOCRefClear(hal_tim_handle_t *htim,
                                                 hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Control that OCRefClear is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OCXREF_CLEAR_INSTANCE(p_tim)));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_EnableClear(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Disable clearing of the OCxRef signal by the OCRef clear input.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableCompareUnitOCRefClear(hal_tim_handle_t *htim,
                                                  hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Control that OCRefClear is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OCXREF_CLEAR_INSTANCE(p_tim)));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  LL_TIM_OC_DisableClear(p_tim, ll_tim_channels[compare_unit]);

  return HAL_OK;
}

/**
  * @brief  Tell whether OCxRef signal can be cleared by the OCRef clear input or not.
  * @param  htim          Pointer to the handle of the TIM instance.
  * @param  compare_unit  Output compare unit.
  * @retval hal_tim_ocref_clr_status_t  OCRefClear status.
  */
hal_tim_ocref_clr_status_t HAL_TIM_IsEnabledCompareUnitOCRefClear(const hal_tim_handle_t *htim,
                                                                  hal_tim_oc_compare_unit_t compare_unit)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Control that OCRefClear is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OCXREF_CLEAR_INSTANCE(p_tim)));

  /* Check if the compare unit is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_OC_COMPARE_UNIT(p_tim, compare_unit)));

  return (hal_tim_ocref_clr_status_t)LL_TIM_OC_IsEnabledClear(p_tim,
                                                              ll_tim_channels[compare_unit]);
}

/**
  *  @}
  */

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/** @addtogroup TIM_Exported_Functions_Group11
  *  @{
  */

/**
  * @brief Configure the DMA Burst.
  * @param htim      Pointer to the handle of the TIM instance.
  * @param p_config  Pointer to the DMA burst configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_SetConfigDMABurst(hal_tim_handle_t *htim,
                                       hal_tim_dmaburst_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_INSTANCE(p_tim));

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_BASE_ADDR_REG(p_config->address));
  ASSERT_DBG_PARAM(IS_TIM_DMABURST_SRC(p_tim, p_config->source));
  ASSERT_DBG_PARAM(IS_TIM_DMABURST_LENGTH(p_config->length));

  /* Save the DMA burst source in the handle for the DMA Burst start/stop operations */
  htim->dmaburst_source = (tim_dmaburst_source_t)p_config->source;

  LL_TIM_ConfigDMABurst(p_tim, (uint32_t)p_config->address,
                        (uint32_t)p_config->length,
                        (uint32_t)p_config->source);

  return HAL_OK;
}

/**
  * @brief Get the DMA Burst configuration.
  * @param htim      Pointer to the handle of the TIM instance.
  * @param p_config  Pointer to the DMA burst configuration structure to fill.
  */
void HAL_TIM_GetConfigDMABurst(const hal_tim_handle_t *htim,
                               hal_tim_dmaburst_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_INSTANCE(p_tim));

  uint32_t address;
  uint32_t length;
  uint32_t source;

  LL_TIM_GetConfigDMABurst(p_tim, &address, &length, &source);

  p_config->source = (hal_tim_dmaburst_source_t)source;
  p_config->address = (hal_tim_dmaburst_base_addr_reg_t)address;
  p_config->length = (hal_tim_dmaburst_length_t)length;
}

/**
  * @brief   Start the timer DMA Burst operation.
  * @param   htim                Pointer to the handle of the TIM instance.
  * @param   dmaburst_direction  DMA burst transfer direction.
  * @param   p_data              Pointer to the data buffer.
  * @param   size_byte           Number of byte data to transfer from memory to register.
  * @warning This function can only be called after DMA burst configuration, i.e.
  *          calling @ref HAL_TIM_SetConfigDMABurst().
  * @retval  HAL_OK
  * @retval  HAL_ERROR          Failed to start the DMA transfer.
  * @retval  HAL_INVALID_PARAM  Input parameter is invalid
  *                             (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_StartDMABurst(hal_tim_handle_t *htim,
                                   hal_tim_dmaburst_direction_t dmaburst_direction,
                                   const uint8_t *p_data,
                                   uint32_t size_byte)
{
  /* LUT to retrieve callbacks associated to the dma burst source
     (format: {half complete callback, complete callback}) */
  static const hal_dma_cb_t dma_burst_cb[][2] =
  {
    {TIM_DMAUpdateHalfCpltCallback, TIM_DMAUpdateCpltCallback},
    {TIM_DMACompareMatchHalfCpltCallback, TIM_DMACompareMatchCpltCallback},
    {TIM_DMACompareMatchHalfCpltCallback, TIM_DMACompareMatchCpltCallback},
    {TIM_DMACompareMatchHalfCpltCallback, TIM_DMACompareMatchCpltCallback},
    {TIM_DMACompareMatchHalfCpltCallback, TIM_DMACompareMatchCpltCallback},
    {TIM_DMACommutationHalfCpltCallback, TIM_DMACommutationCpltCallback},
    {TIM_DMATriggerHalfCpltCallback, TIM_DMATriggerCpltCallback}
  };
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_data != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_data == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_INSTANCE(p_tim));

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_DIR(dmaburst_direction));

  /* Retrieve the dma burst source configured from the handle */
  tim_dmaburst_source_t dma_burst_src = htim->dmaburst_source;

  /* Check that the retrieved dma burst source is correct */
  ASSERT_DBG_PARAM(IS_TIM_DMABURST_SRC(p_tim, (hal_tim_dmaburst_source_t)dma_burst_src));

  /* Calculate the dma request associated to the dma burst source
     (-1U because dma burst source starts at 1 (0 is reserved)) */
  hal_tim_dma_index_t dma_index = (hal_tim_dma_index_t)(uint32_t)(((uint32_t)dma_burst_src \
                                                                   >> TIM_DMABURST_DMAINDEX_SHIFT) - 1U);

  hal_dma_handle_t *hdma = htim->hdma[dma_index];
  ASSERT_DBG_PARAM((hdma != NULL));

  /* Set DMA channel callback function pointers */
  hdma->p_xfer_halfcplt_cb = dma_burst_cb[dma_index][0];
  hdma->p_xfer_cplt_cb = dma_burst_cb[dma_index][1];
  hdma->p_xfer_error_cb = TIM_DMAErrorCallback;
  hdma->p_xfer_abort_cb = TIM_DMAStopCallback;

  if ((dma_index >= HAL_TIM_DMA_ID_CC1) && (dma_index <= HAL_TIM_DMA_ID_CC4))
  {
    /* Calculate the tim channel associated to the dma index */
    hal_tim_channel_t channel = (hal_tim_channel_t)((uint32_t)((uint32_t)dma_index - (uint32_t)HAL_TIM_DMA_ID_CC1));

    if (TIM_IS_INPUT_CHANNEL(p_tim, channel))
    {
      /* Use capture callbacks if the channel is in input mode
         (compare callbacks by default) */
      hdma->p_xfer_halfcplt_cb = TIM_DMACaptureHalfCpltCallback;
      hdma->p_xfer_cplt_cb = TIM_DMACaptureCpltCallback;
    }
  }

  /* Enable the DMA request */
  uint32_t dma_req = LL_TIM_DIER_UDE << (uint32_t)dma_index;

  LL_TIM_EnableDMAReq(p_tim, dma_req);

  uint32_t src_addr;
  uint32_t dest_addr;

  /* Update the source and destination addresses depending
     the DMA burst transfer direction */
  if (dmaburst_direction == HAL_TIM_DMABURST_READ)
  {
    src_addr = (uint32_t)((uint32_t *)(&p_tim->DMAR));
    dest_addr = (uint32_t)p_data;
  }
  else
  {
    src_addr = (uint32_t)p_data;
    dest_addr = (uint32_t)((uint32_t *)(&p_tim->DMAR));
  }

  /* Start DMA transfer in interrupt mode */
  if (HAL_DMA_StartPeriphXfer_IT_Opt(hdma, src_addr,
                                     dest_addr, size_byte,
                                     HAL_TIM_OPT_DMA_IT_DEFAULT) != HAL_OK)
  {
#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
    htim->last_error_codes |= HAL_TIM_ERROR_DMA;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */

    return HAL_ERROR;
  }

  return HAL_OK;
}


/**
  * @brief  Stop the timer DMA Burst operation.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_StopDMABurst(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM(IS_TIM_DMABURST_INSTANCE(TIM_INSTANCE(htim)));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Retrieve the dma burst source configured */
  tim_dmaburst_source_t dma_burst_src = htim->dmaburst_source;

  /* Check that the retrieved dma burst source is correct */
  ASSERT_DBG_PARAM(IS_TIM_DMABURST_SRC(p_tim, (hal_tim_dmaburst_source_t)dma_burst_src));

  /* Calculate the dma request associated to the dma burst source
     (-1U because dma burst source starts at 1 (0 is reserved)) */
  hal_tim_dma_index_t dma_index = (hal_tim_dma_index_t)(uint32_t)(((uint32_t)dma_burst_src \
                                                                   >> TIM_DMABURST_DMAINDEX_SHIFT) - 1U);

  /* Calculate the dma request associated to the dma burst source */
  uint32_t dma_req = LL_TIM_DIER_UDE << (uint32_t)dma_index;

  hal_dma_handle_t *hdma = htim->hdma[dma_index];
  ASSERT_DBG_PARAM((hdma != NULL));

  (void)HAL_DMA_Abort_IT(hdma);

  LL_TIM_DisableDMAReq(p_tim, dma_req);

  return HAL_OK;
}


/**
  *  @}
  */
#endif /* USE_HAL_TIM_DMA */


/** @addtogroup TIM_Exported_Functions_Group12
  *  @{
  */

/**
  * @brief  Configure the break input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input to configure.
  * @param  p_config  Pointer to the break input configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_BREAK_SetConfigInput(hal_tim_handle_t *htim,
                                          hal_tim_break_input_id_t brkin,
                                          const hal_tim_break_input_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check break input configuration parameters */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_POLARITY(p_config->polarity)));
  ASSERT_DBG_PARAM((IS_TIM_FILTER(p_config->filter)));
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_MODE(p_config->mode)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  if (brkin == HAL_TIM_BREAK_INPUT_1)
  {
    LL_TIM_ConfigBRK(p_tim,
                     TIM_BREAK_HAL2LL_POLARITY(p_config->polarity),
                     TIM_BREAK_HAL2LL_FILTER(p_config->filter),
                     TIM_BREAK_HAL2LL_MODE(p_config->mode));
  }
  else
  {
    LL_TIM_ConfigBRK2(p_tim,
                      TIM_BREAK2_HAL2LL_POLARITY(p_config->polarity),
                      TIM_BREAK2_HAL2LL_FILTER(p_config->filter),
                      TIM_BREAK2_HAL2LL_MODE(p_config->mode));
  }

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the break input.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input of interest.
  * @param  p_config  Pointer to the break input configuration structure.
  */
void HAL_TIM_BREAK_GetConfigInput(const hal_tim_handle_t *htim,
                                  hal_tim_break_input_id_t brkin,
                                  hal_tim_break_input_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  uint32_t polarity;
  uint32_t filter;
  uint32_t mode;

  if (brkin == HAL_TIM_BREAK_INPUT_1)
  {
    LL_TIM_GetConfigBRK(p_tim, &polarity, &filter, &mode);

    p_config->polarity = TIM_BREAK_LL2HAL_POLARITY(polarity);
    p_config->filter = TIM_BREAK_LL2HAL_FILTER(filter);
  }
  else
  {
    LL_TIM_GetConfigBRK2(p_tim, &polarity, &filter, &mode);

    p_config->polarity = TIM_BREAK2_LL2HAL_POLARITY(polarity);
    p_config->filter = TIM_BREAK2_LL2HAL_FILTER(filter);
  }

  p_config->mode = (mode != LL_TIM_BREAK_AFMODE_INPUT) ? \
                   HAL_TIM_BREAK_INPUT_MODE_BIDIRECTIONAL : HAL_TIM_BREAK_INPUT_MODE_INPUT;
}

/**
  * @brief  Configure the timer's break input polarity.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input to configure.
  * @param  polarity  Polarity for the break input.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_SetInputPolarity(hal_tim_handle_t *htim,
                                            hal_tim_break_input_id_t brkin,
                                            hal_tim_break_input_polarity_t polarity)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_POLARITY(polarity)));

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  LL_TIM_SetBreakInputPolarity(p_tim, (uint32_t)brkin,
                               TIM_BRK_BRK2_HAL2LL_POLARITY((uint32_t)brkin,
                                                            (uint32_t)polarity));

  return HAL_OK;
}

/**
  * @brief  Get the polarity of the timer's break input.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input of interest.
  * @retval hal_tim_break_input_polarity_t  Polarity of the break input.
  */
hal_tim_break_input_polarity_t HAL_TIM_BREAK_GetInputPolarity(const hal_tim_handle_t *htim,
                                                              hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  uint32_t polarity = LL_TIM_GetBreakInputPolarity(p_tim, (uint32_t)brkin);

  return TIM_BRK_BRK2_LL2HAL_POLARITY((uint32_t)brkin, (uint32_t)polarity);
}

/**
  * @brief  Configure the timer's break input filter.
  * @param  htim    Pointer to the handle of the TIM instance.
  * @param  brkin   The break input to configure.
  * @param  filter  Filter to apply to the break input.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_SetInputFilter(hal_tim_handle_t *htim,
                                          hal_tim_break_input_id_t brkin,
                                          hal_tim_filter_t filter)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  ASSERT_DBG_PARAM((IS_TIM_FILTER(filter)));

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  LL_TIM_SetBreakInputFilter(p_tim, (uint32_t)brkin,
                             TIM_BRK_BRK2_HAL2LL_FILTER((uint32_t)brkin,
                                                        (uint32_t)filter));

  return HAL_OK;
}

/**
  * @brief  Get the filter applied to the timer's break input.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input of interest.
  * @retval hal_tim_filter_t  Filter applied to the break input.
  */
hal_tim_filter_t HAL_TIM_BREAK_GetInputFilter(const hal_tim_handle_t *htim,
                                              hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  uint32_t filter = LL_TIM_GetBreakInputFilter(p_tim, (uint32_t)brkin);

  return TIM_BRK_BRK2_LL2HAL_FILTER((uint32_t)brkin, filter);
}

/**
  * @brief  Configure the timer's break input AF mode (input versus bidirectional).
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input to configure.
  * @param  mode   Mode (input or bidirectional) for the break input.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_SetInputMode(hal_tim_handle_t *htim,
                                        hal_tim_break_input_id_t brkin,
                                        hal_tim_break_input_mode_t mode)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_MODE(mode)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  LL_TIM_SetBreakInputAFMode(p_tim, (uint32_t)brkin,
                             TIM_BRK_BRK2_HAL2LL_MODE((uint32_t)brkin,
                                                      (uint32_t)mode));

  return HAL_OK;
}

/**
  * @brief  Get the timer's break input mode.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input to configure.
  * @retval hal_tim_break_input_mode_dir_t  Break input mode.
  */
hal_tim_break_input_mode_t HAL_TIM_BREAK_GetInputMode(const hal_tim_handle_t *htim,
                                                      hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  return TIM_BRK_BRK2_LL2HAL_MODE(brkin,
                                  LL_TIM_GetBreakInputAFMode(p_tim,
                                                             (uint32_t)brkin));
}

/**
  * @brief  Enable a break input.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input to enable.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_EnableInput(hal_tim_handle_t *htim,
                                       hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  LL_TIM_EnableBreakInput(p_tim, (uint32_t)brkin);

  return HAL_OK;
}

/**
  * @brief  Disable a break input.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input to disable.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_DisableInput(hal_tim_handle_t *htim,
                                        hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  LL_TIM_DisableBreakInput(p_tim, (uint32_t)brkin);

  return HAL_OK;
}

/**
  * @brief  Tell whether a break input is enabled or not.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input.
  * @retval hal_tim_break_input_status_t  Status of the break input
  */
hal_tim_break_input_status_t HAL_TIM_BREAK_IsEnabledInput(const hal_tim_handle_t *htim,
                                                          hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  return (hal_tim_break_input_status_t)LL_TIM_IsEnabledBreakInput(p_tim,
                                                                  (uint32_t)brkin);
}

/**
  * @brief  Re-arm the break input after a break event.
  *         This function must be called to re-activate the break circuitry
  *         after a break (break2) event.
  * @param  htim   Pointer to the handle of the TIM instance.
  * @param  brkin  The break input to re-arm.
  * @note   The system break condition must have disappeared and the system
  *         break flag must have been cleared.
  * @note   If this function succeeds then @ref HAL_TIM_BREAK_EnableMainOutput
  *         can be called to re-enable the outputs.
  * @retval HAL_OK
  * @retval HAL_ERROR  Break input condition still present.
  */
hal_status_t HAL_TIM_BREAK_RearmInput(hal_tim_handle_t *htim,
                                      hal_tim_break_input_id_t brkin)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BRKIN_INSTANCE(p_tim, brkin)));

  /* Note: release of the output control is meaningful only in
     bidirectional mode but it is done by default. */

  /* Release the output control */
  LL_TIM_DisarmBreakInput(p_tim, (uint32_t)brkin);

  /* Break input is re-armed automatically by hardware.
     Poll to check whether application break condition disappeared. */
  uint32_t tickstart = HAL_GetTick();
  while (LL_TIM_IsDisarmedBreakInput(p_tim, (uint32_t)brkin) != 0U)
  {
    if (TIM_BREAK_INPUT_REARM_TIMEOUT_PERIOD_EXPIRED(HAL_GetTick() - tickstart))
    {
      /* New check to avoid false timeout detection in case of preemption */
      if (LL_TIM_IsDisarmedBreakInput(p_tim, (uint32_t)brkin) != 0U)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Configure the break input source polarity.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input to configure.
  * @param  brkinsrc  This parameter can be one of the following values: \n
  *         The description below summarizes "Timer Instance" and "BREAK(2) input source" possibilities:
  *
  *         TIM1: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM1_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            @arg @ref HAL_TIM_BRK2_TIM1_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM8: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM8_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            @arg @ref HAL_TIM_BRK2_TIM8_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM15: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM15_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP4_OUT (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM16_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM17_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM20_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP4_OUT (*)
  * @endif
  *
  *            @arg @ref HAL_TIM_BRK2_TIM20_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices. \n
  *         (**) Timer instance not available on all devices. \n
  * @param  polarity  Polarity for the break input source.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_SetInputSourcePolarity(hal_tim_handle_t *htim,
                                                  hal_tim_break_input_id_t brkin,
                                                  uint32_t brkinsrc,
                                                  hal_tim_break_input_src_polarity_t polarity)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  /* Check if the instance supports the break input source */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_SRC(p_tim, brkin, brkinsrc)));
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_SRC_POLARITY(brkinsrc, polarity)));

  LL_TIM_SetBreakInputSourcePolarity(p_tim, (uint32_t)brkin,
                                     (uint32_t)brkinsrc, (uint32_t)polarity);
  return HAL_OK;
}

/**
  * @brief  Get the polarity of the break input source.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input of interest.
  * @param  brkinsrc  This parameter can be one of the following values: \n
  *         The description below summarizes "Timer Instance" and "BREAK(2) input source" possibilities:
  *
  *         TIM1: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM1_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM1_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            @arg @ref HAL_TIM_BRK2_TIM1_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM1_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM8: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM8_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM8_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            @arg @ref HAL_TIM_BRK2_TIM8_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM8_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM15: one of the following values:
  *            @arg @ref HAL_TIM_BRK_TIM15_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP1_OUT
  * @if COMP2
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM15_COMP4_OUT (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM16_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM16_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM17_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM17_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *            @arg @ref HAL_TIM_BRK_TIM20_GPIO
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK_TIM20_COMP4_OUT (*)
  * @endif
  *
  *            @arg @ref HAL_TIM_BRK2_TIM20_GPIO
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP1_OUT
  * @if COMP3
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP2_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP3_OUT (*)
  *            @arg @ref HAL_TIM_BRK2_TIM20_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices. \n
  *         (**) Timer instance not available on all devices. \n
  * @retval hal_tim_break_input_src_polarity_t  The break input source polarity.
  */
hal_tim_break_input_src_polarity_t HAL_TIM_BREAK_GetInputSourcePolarity(const hal_tim_handle_t *htim,
                                                                        hal_tim_break_input_id_t brkin,
                                                                        uint32_t brkinsrc)
{
  ASSERT_DBG_PARAM((htim != NULL));

  /* Check the global state */
  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  const tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  /* Check if the instance supports the break input source */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_SRC(p_tim, brkin, brkinsrc)));

  return ((hal_tim_break_input_src_polarity_t)
          LL_TIM_GetBreakInputSourcePolarity(p_tim, (uint32_t)brkin,
                                             (uint32_t)brkinsrc));
}

/**
  * @brief  Enable a break input source.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input.
  * @param  brkinsrc  This parameter can be a combination of @ref TIM_Break_Input_Sources values.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_EnableInputSource(hal_tim_handle_t *htim,
                                             hal_tim_break_input_id_t brkin,
                                             uint32_t brkinsrc)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  /* Check if the instance supports the break input source(s) */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ALL_SRC(p_tim, brkin, brkinsrc)));

  LL_TIM_EnableBreakInputSource(p_tim, (uint32_t)brkin, (uint32_t)brkinsrc);

  return HAL_OK;
}

/**
  * @brief  Disable a break input source.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input.
  * @param  brkinsrc  This parameter can be a combination of @ref TIM_Break_Input_Sources values.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_DisableInputSource(hal_tim_handle_t *htim,
                                              hal_tim_break_input_id_t brkin,
                                              uint32_t brkinsrc)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  /* Check if the instance supports the break input source(s) */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ALL_SRC(p_tim, brkin, brkinsrc)));

  LL_TIM_DisableBreakInputSource(p_tim, (uint32_t)brkin, (uint32_t)brkinsrc);

  return HAL_OK;
}

/**
  * @brief  Tell whether a break input source is enabled or not.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  brkin     The break input.
  * @param  brkinsrc  This parameter can be one of @ref TIM_Break_Input_Sources values.
  * @retval hal_tim_break_input_src_status_t  Break input source status
  */
hal_tim_break_input_src_status_t HAL_TIM_BREAK_IsEnabledInputSource(const hal_tim_handle_t *htim,
                                                                    hal_tim_break_input_id_t brkin,
                                                                    uint32_t brkinsrc)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_ID(brkin)));
  /* Check if the instance supports the break input source */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INPUT_SRC(p_tim, brkin, brkinsrc)));

  return ((hal_tim_break_input_src_status_t)
          LL_TIM_IsEnabledBreakInputSource(p_tim, (uint32_t)brkin,
                                           (uint32_t)brkinsrc));
}

/**
  * @brief  Enable main output.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_EnableMainOutput(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_EnableAllOutputs(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable main output.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_DisableMainOutput(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_DisableAllOutputs(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether the main output is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_break_main_output_status_t  Main output status.
  */
hal_tim_break_main_output_status_t HAL_TIM_BREAK_IsEnabledMainOutput(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_break_main_output_status_t)LL_TIM_IsEnabledAllOutputs(p_tim);
}

/**
  * @brief  Enable automatic output.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   Main output is cleared by hardware as soon as one of the break inputs
  *         is active. \n
  *         When the break input is not active anymore, main output is
  *         automatically set by hardware if automatic output is enabled.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_EnableAutomaticOutput(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_EnableAutomaticOutput(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable automatic output.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_DisableAutomaticOutput(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_DisableAutomaticOutput(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether the automatic output is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_break_automatic_output_status_t  Status of the automatic output.
  */
hal_tim_break_automatic_output_status_t HAL_TIM_BREAK_IsEnabledAutomaticOutput(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_break_automatic_output_status_t)LL_TIM_IsEnabledAutomaticOutput(p_tim);
}

/**
  * @brief  Configure the delay duration for a specific break delay.
  * @param  htim         Pointer to the handle of the TIM instance.
  * @param  break_delay  Output channel break delay.
  * @param  delay        Delay duration (number between 0x00 and 0xFF).
  * @note   Delayed break (DBKx[7:0]) preload value isn't taken into account
  *         immediately. It is loaded in the active register at next update event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_BREAK_SetBreakDelay(hal_tim_handle_t *htim,
                                         hal_tim_break_delay_t break_delay,
                                         uint32_t delay)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  /* Check that it is a break delay */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_DELAY(break_delay)));

  /* Check the delay duration */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_DELAY_DURATION(delay)));

  LL_TIM_SetBreakDelay(p_tim, (uint32_t)break_delay, delay);

  return HAL_OK;
}

/**
  * @brief  Get the delay duration for a specific break delay.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  break_delay Output channel break delay. \n
  * @retval uint32_t  Delay duration (number between 0x00 and 0xFF).
  */
uint32_t HAL_TIM_BREAK_GetBreakDelay(const hal_tim_handle_t *htim,
                                     hal_tim_break_delay_t break_delay)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  /* Check that it is a break delay */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_DELAY(break_delay)));

  return LL_TIM_GetBreakDelay(p_tim, (uint32_t)break_delay);
}

/**
  * @brief  Configure the off-state of the timer's outputs for both RUN mode
  *         (when main output is enabled) and IDLE mode (when main output is disabled).
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a off states configuration structure.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM  Input parameter is invalid
  *                            (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_BREAK_SetOutputOffStates(hal_tim_handle_t *htim,
                                              const hal_tim_off_states_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that off-state (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  /* Check off-state configuration parameters */
  ASSERT_DBG_PARAM(IS_TIM_OFF_STATE_IDLE(p_config->off_state_idle));
  ASSERT_DBG_PARAM(IS_TIM_OFF_STATE_RUN(p_config->off_state_run));

  LL_TIM_SetOffStates(p_tim, (uint32_t)p_config->off_state_idle,
                      (uint32_t)p_config->off_state_run);

  return HAL_OK;
}

/**
  * @brief  Get the off-state configuration.
  * @param  htim      Pointer to the handle of the TIM instance.
  * @param  p_config  Pointer to a off states configuration structure.
  */
void HAL_TIM_BREAK_GetOutputOffStates(const hal_tim_handle_t *htim,
                                      hal_tim_off_states_config_t *p_config)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((p_config != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that off-state (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  uint32_t off_state_run;
  uint32_t off_state_idle;

  LL_TIM_GetOffStates(p_tim, &off_state_idle, &off_state_run);

  p_config->off_state_idle = (hal_tim_off_state_idle_t)off_state_idle;
  p_config->off_state_run = (hal_tim_off_state_run_t)off_state_run;
}

/**
  * @brief  Indicate the global output state when a break or break2 event
  *         occurred, to discriminate the source.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_output_disable_status_t  Output disable status.
  */
hal_tim_output_disable_status_t HAL_TIM_BREAK_GetOutputDisableStatus(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check if the instance supports break input */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_output_disable_status_t)LL_TIM_GetOutputDisableStatus(p_tim);
}
/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group13
  *  @{
  */
/**
  * @brief  Configure the deadtime inserted between two complementary outputs.
  * @param  htim                   Pointer to the handle of the TIM instance.
  * @param  rising_edge_deadtime   Deadtime value for rising edge (number between 0x00 and 0xFF)
  * @param  falling_edge_deadtime  Deadtime value for falling edge (number between 0x00 and 0xFF)
  * @note   For asymmetrical deadtime HAL_TIM_EnableAsymmetricalDeadTime must be called.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetDeadtime(hal_tim_handle_t *htim,
                                 uint32_t rising_edge_deadtime,
                                 uint32_t falling_edge_deadtime)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  /* Check the deadtime values */
  ASSERT_DBG_PARAM((IS_TIM_DEADTIME(rising_edge_deadtime)));
  ASSERT_DBG_PARAM((IS_TIM_DEADTIME(falling_edge_deadtime)));

  LL_TIM_OC_SetDeadTime(p_tim, rising_edge_deadtime);
  LL_TIM_SetFallingDeadTime(p_tim, falling_edge_deadtime);

  return HAL_OK;
}

/**
  * @brief  Get the deadtime configuration.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  p_rising_edge_deadtime   Pointer to a storage for the deadtime value for rising edge
  *                                  (number between 0x00 and 0xFF).
  * @param  p_falling_edge_deadtime  Pointer to a storage for the deadtime value for falling edge
  *                                  (number between 0x00 and 0xFF).
  */
void HAL_TIM_GetDeadtime(const hal_tim_handle_t *htim,
                         uint32_t *p_rising_edge_deadtime,
                         uint32_t *p_falling_edge_deadtime)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_PARAM((p_rising_edge_deadtime != NULL));
  ASSERT_DBG_PARAM((p_falling_edge_deadtime != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  *p_rising_edge_deadtime = LL_TIM_OC_GetDeadTime(p_tim);
  *p_falling_edge_deadtime = LL_TIM_GetFallingDeadTime(p_tim);
}

/**
  * @brief  Enable the deadtime configuration preload
  *         (DTG[7:0] and DTGF[7:0] bitfields).
  * @param  htim  Pointer to the handle of the TIM instance.
  * @note   When deadtime preload is enabled, rising and falling deatime
  *         (TIMx_BDTR.DTG and TIMx_DTR2.DTGF) preload values aren't taken
  *         into account immediately. \n
  *         They are loaded in the active register at next update event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableDeadtimePreload(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_EnableDeadTimePreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable the deadtime configuration preload.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableDeadtimePreload(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_DisableDeadTimePreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether the deadtime configuration preload is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_deadtime_preload_status_t  Deadtime preload status.
  */
hal_tim_deadtime_preload_status_t HAL_TIM_IsEnabledDeadtimePreload(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_deadtime_preload_status_t)LL_TIM_IsEnabledDeadTimePreload(p_tim);
}

/**
  * @brief  Enable asymmetrical deadtime.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableAsymmetricalDeadtime(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_EnableAsymmetricalDeadTime(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable asymmetrical deadtime.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableAsymmetricalDeadtime(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  LL_TIM_DisableAsymmetricalDeadTime(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether asymmetrical deadtime is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_asymmetrical_deadtime_status_t  Asymmetrical deadtime status.
  */
hal_tim_asymmetrical_deadtime_status_t HAL_TIM_IsEnabledAsymmetricalDeadtime(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that deadtime (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  return (hal_tim_asymmetrical_deadtime_status_t)LL_TIM_IsEnabledAsymmetricalDeadTime(p_tim);
}

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group14
  *  @{
  */

/**
  * @brief  Set the timer lock level.
  * @param  htim        Pointer to the handle of the TIM instance.
  * @param  lock_level  Lock level.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_SetLockLevel(hal_tim_handle_t *htim,
                                  hal_tim_lock_level_t lock_level)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  ASSERT_DBG_PARAM(IS_TIM_LOCK_LEVEL(lock_level));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that lock (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  LL_TIM_CC_SetLockLevel(p_tim, (uint32_t)lock_level);

  return HAL_OK;
}

/**
  * @brief  Get the timer lock level.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_lock_level_t  Lock level.
  */
hal_tim_lock_level_t HAL_TIM_GetLockLevel(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check that lock (<=>break input) is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_BREAK_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_lock_level_t)LL_TIM_CC_GetLockLevel(p_tim);
}

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group15
  *  @{
  */

/**
  * @brief  Enable the commutation and set the commutation event source.
  * @param  htim                Pointer to the handle of the TIM instance.
  * @param  commutation_source  Commutation source.
  * @note    when commutation is enabled, CCxE, CCxNE and OCxM bit are preloaded. \n
  *          They are loaded in the active register when the commutation event occurs. \n
  *          Commutation event can be triggered by software or both by software and trigger
  *          input as per chosen commutation source.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_EnableCommutation(hal_tim_handle_t *htim,
                                       hal_tim_commutation_src_t commutation_source)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check commutation is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_COMMUTATION_EVENT_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Check commutation source */
  ASSERT_DBG_PARAM((IS_TIM_COMMUTATION_SRC(commutation_source)));

  /* Configure the commutation event source */
  LL_TIM_CC_SetUpdate(p_tim, (uint32_t)commutation_source);

  /* Enable the capture/compare control bits (CCxE, CCxNE and OCxM) preload */
  LL_TIM_CC_EnablePreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Disable the commutation feature.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_DisableCommutation(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check commutation is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_COMMUTATION_EVENT_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state, HAL_TIM_STATE_IDLE);

  /* Disable the capture/compare control bits (CCxE, CCxNE and OCxM) preload */
  LL_TIM_CC_DisablePreload(p_tim);

  return HAL_OK;
}

/**
  * @brief  Tell whether the commutation is enabled or not.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_commutation_status_t  Status (Enabled/Disabled) of the commutation feature.
  */
hal_tim_commutation_status_t HAL_TIM_IsEnabledCommutation(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check commutation is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_COMMUTATION_EVENT_INSTANCE(p_tim)));

  return (hal_tim_commutation_status_t)LL_TIM_CC_IsEnabledPreload(p_tim);
}

/**
  * @brief  Get the commutation event source.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval hal_tim_commutation_src_t  Source (software only or software and trigger input)
  *                                    of the commutation feature.
  */
hal_tim_commutation_src_t HAL_TIM_GetCommutationSource(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  /* Check commutation is supported by the instance */
  ASSERT_DBG_PARAM((IS_TIM_COMMUTATION_EVENT_INSTANCE(p_tim)));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  return (hal_tim_commutation_src_t)LL_TIM_CC_GetUpdate(p_tim);
}

/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group16
  *  @{
  */

/**
  * @brief  Generate a software event for the timer.
  * @param  htim         Pointer to the handle of the TIM instance.
  * @param  sw_event_id  The source of the event.
  * @retval HAL_OK
  */
hal_status_t HAL_TIM_GenerateEvent(hal_tim_handle_t *htim,
                                   hal_tim_sw_event_id_t sw_event_id)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  ASSERT_DBG_PARAM((IS_TIM_SW_EVENT_ID(p_tim, sw_event_id)));

  ASSERT_DBG_STATE(htim->global_state,
                   (HAL_TIM_STATE_IDLE | HAL_TIM_STATE_ACTIVE));

  LL_TIM_GenerateEvent(p_tim, (uint32_t)sw_event_id);

  return HAL_OK;
}
/**
  *  @}
  */


/** @addtogroup TIM_Exported_Functions_Group17
  *  @{
  */

/**
  * @brief This function handles TIM generic interrupts requests.
  * @param htim  Pointer to the handle of the TIM instance.
  * @note  Handle all the timer interrupt requests.
  */
void HAL_TIM_IRQHandler(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  const uint32_t flag_status = LL_TIM_READ_REG(p_tim, SR);
  const uint32_t it_sources = LL_TIM_READ_REG(p_tim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND between flags status and interrupts
                                                                   sources enabled (for registers bitfields aligned) */
  const uint32_t break_it_source = (uint32_t)STM32_IS_BIT_SET(it_sources, LL_TIM_DIER_BIE); /* for break registers
                                                                                               bitfields not aligned */

  if ((flag_status_masked & LL_TIM_SR_UIF) != 0UL)
  {
    LL_TIM_ClearFlag_UPDATE(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->update_callback(htim);
#else
    HAL_TIM_UpdateCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_CC1IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC1(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_1))
    {
      /* Channel 1 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_1);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 1 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_1);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC2IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC2(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_2))
    {
      /* Channel 2 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_2);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 2 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_2);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC3IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC3(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_3))
    {
      /* Channel 3 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_3);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_3);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 3 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_3);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_3);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC4IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC4(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_4))
    {
      /* Channel 4 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_4);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_4);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 4 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_4);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_4);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_SBIF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_SYSBRK(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->system_break_callback(htim);
#else
    HAL_TIM_SystemBreakCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_BGF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_BG(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->software_break_callback(htim, HAL_TIM_BREAK_INPUT_1);
#else
    HAL_TIM_SoftwareBreakCallback(htim, HAL_TIM_BREAK_INPUT_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_B2GF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_B2G(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->software_break_callback(htim, HAL_TIM_BREAK_INPUT_2);
#else
    HAL_TIM_SoftwareBreakCallback(htim, HAL_TIM_BREAK_INPUT_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_BIF) != 0UL)
  {
    LL_TIM_ClearFlag_BRK(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->break_callback(htim);
#else
    HAL_TIM_BreakCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, TIM_SR_B2IF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_BRK2(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->break2_callback(htim);
#else
    HAL_TIM_Break2Callback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_TERRF) != 0UL)
  {
    LL_TIM_ClearFlag_TERR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->transition_error_callback(htim);
#else
    HAL_TIM_TransitionErrorCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_IERRF) != 0UL)
  {
    LL_TIM_ClearFlag_IERR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->index_error_callback(htim);
#else
    HAL_TIM_IndexErrorCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_TIF) != 0UL)
  {
    LL_TIM_ClearFlag_TRIG(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->trigger_callback(htim);
#else
    HAL_TIM_TriggerCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_COMIF) != 0UL)
  {
    LL_TIM_ClearFlag_COM(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->commutation_callback(htim);
#else
    HAL_TIM_CommutationCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_DIRF) != 0UL)
  {
    LL_TIM_ClearFlag_DIR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->direction_change_callback(htim);
#else
    HAL_TIM_DirectionChangeCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_IDXF) != 0UL)
  {
    LL_TIM_ClearFlag_IDX(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->encoder_index_callback(htim);
#else
    HAL_TIM_EncoderIndexCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }
}

/**
  * @brief Timer update interrupt handler.
  * @param htim  Pointer to the handle of the TIM instance.
  */
void HAL_TIM_UPD_IRQHandler(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  if (LL_TIM_IsEnabledIT_UPDATE(p_tim) == 1U)
  {
    LL_TIM_ClearFlag_UPDATE(p_tim);
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->update_callback(htim);
#else
    HAL_TIM_UpdateCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }
}

/**
  * @brief Timer Capture/Compare interrupt handler.
  * @param htim  Pointer to the handle of the TIM instance.
  */
void HAL_TIM_CC_IRQHandler(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  const uint32_t flag_status = LL_TIM_READ_REG(p_tim, SR);
  const uint32_t it_sources = LL_TIM_READ_REG(p_tim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND between flags status and interrupts
                                                                   sources enabled (for registers bitfields aligned) */

  if ((flag_status_masked & LL_TIM_SR_CC1IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC1(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_1))
    {
      /* Channel 1 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_1);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 1 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_1);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC2IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC2(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_2))
    {
      /* Channel 2 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_2);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 2 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_2);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC3IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC3(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_3))
    {
      /* Channel 3 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_3);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_3);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 3 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_3);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_3);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }

  if ((flag_status_masked & LL_TIM_SR_CC4IF) != 0UL)
  {
    LL_TIM_ClearFlag_CC4(p_tim);

    if (TIM_IS_INPUT_CHANNEL(p_tim, HAL_TIM_CHANNEL_4))
    {
      /* Channel 4 is configured as input */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->input_capture_callback(htim, HAL_TIM_CHANNEL_4);
#else
      HAL_TIM_InputCaptureCallback(htim, HAL_TIM_CHANNEL_4);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
    else
    {
      /* Channel 4 is configured as output */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
      htim->compare_match_callback(htim, HAL_TIM_CHANNEL_4);
#else
      HAL_TIM_CompareMatchCallback(htim, HAL_TIM_CHANNEL_4);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief Timer Break, Transition error and Index error interrupt handler.
  * @param htim  Pointer to the handle of the TIM instance.
  */
void HAL_TIM_BRK_TERR_IERR_IRQHandler(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  const uint32_t flag_status = LL_TIM_READ_REG(p_tim, SR);
  const uint32_t it_sources = LL_TIM_READ_REG(p_tim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND between flags status and interrupts
                                                                   sources enabled (for registers bitfields aligned) */
  const uint32_t break_it_source = (uint32_t)STM32_IS_BIT_SET(it_sources, LL_TIM_DIER_BIE); /* for break registers
                                                                                               bitfields not aligned */

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_SBIF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_SYSBRK(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->system_break_callback(htim);
#else
    HAL_TIM_SystemBreakCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_BGF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_BG(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->software_break_callback(htim, HAL_TIM_BREAK_INPUT_1);
#else
    HAL_TIM_SoftwareBreakCallback(htim, HAL_TIM_BREAK_INPUT_1);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_B2GF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_B2G(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->software_break_callback(htim, HAL_TIM_BREAK_INPUT_2);
#else
    HAL_TIM_SoftwareBreakCallback(htim, HAL_TIM_BREAK_INPUT_2);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_BIF) != 0UL)
  {
    LL_TIM_ClearFlag_BRK(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->break_callback(htim);
#else
    HAL_TIM_BreakCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if (STM32_IS_BIT_SET(flag_status, LL_TIM_SR_B2IF) && (break_it_source != 0U))
  {
    LL_TIM_ClearFlag_BRK2(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->break2_callback(htim);
#else
    HAL_TIM_Break2Callback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_TERRF) != 0UL)
  {
    LL_TIM_ClearFlag_TERR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->transition_error_callback(htim);
#else
    HAL_TIM_TransitionErrorCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_IERRF) != 0UL)
  {
    LL_TIM_ClearFlag_IERR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->index_error_callback(htim);
#else
    HAL_TIM_IndexErrorCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }
}

/**
  * @brief Timer Trigger, Commutation, Direction change and Index interrupt handler.
  * @param htim  Pointer to the handle of the TIM instance.
  */
void HAL_TIM_TRGI_COM_DIR_IDX_IRQHandler(hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  tim_t *p_tim = TIM_INSTANCE(htim);

  const uint32_t flag_status = LL_TIM_READ_REG(p_tim, SR);
  const uint32_t it_sources = LL_TIM_READ_REG(p_tim, DIER);
  const uint32_t flag_status_masked = flag_status & it_sources; /* Logical AND between flags status and interrupts
                                                                   sources enabled (for registers bitfields aligned) */

  if ((flag_status_masked & LL_TIM_SR_TIF) != 0UL)
  {
    LL_TIM_ClearFlag_TRIG(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->trigger_callback(htim);
#else
    HAL_TIM_TriggerCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_COMIF) != 0UL)
  {
    LL_TIM_ClearFlag_COM(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->commutation_callback(htim);
#else
    HAL_TIM_CommutationCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_DIRF) != 0UL)
  {
    LL_TIM_ClearFlag_DIR(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->direction_change_callback(htim);
#else
    HAL_TIM_DirectionChangeCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  if ((flag_status_masked & LL_TIM_SR_IDXF) != 0UL)
  {
    LL_TIM_ClearFlag_IDX(p_tim);

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->encoder_index_callback(htim);
#else
    HAL_TIM_EncoderIndexCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief DMA Error callback \n
  *        This function is called in case of a DMA transfer error.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_ErrorCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_ErrorCallback can be implemented in the user file.
   */
}

/**
  * @brief DMA Stop callback \n
  *        This function is called after stopping a DMA transfer either triggered
  *        by the timer update event, the commutation event or the trigger event.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_StopCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_StopCallback can be implemented in the user file.
   */
}

/**
  * @brief DMA Channel Stop callback \n
  *        This function is called after stopping a DMA transfer triggered
  *        by a capture/compare event.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param channel  Channel stopped for the capture/compare.
  */
__WEAK void HAL_TIM_ChannelStopCallback(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(channel);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_ChannelStopCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Update callback. \n
  *        Function called when the timer update interrupt is generated or when
  *        the DMA transfer triggered by the timer update DMA request is completed.
  *
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_UpdateCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_UpdateCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Update Half Complete callback. \n
  *        Function called when the DMA transfer triggered by the timer update
  *        DMA request is half completed.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_UpdateHalfCpltCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_UpdateHalfCpltCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Trigger callback. \n
  *        Function called when the timer trigger interrupt is generated or when
  *        the DMA transfer triggered by the timer trigger DMA request is completed.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_TriggerCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_TriggerCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Trigger Half Complete callback. \n
  *        Function called when the DMA transfer triggered by the timer trigger
  *        DMA request is half completed.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_TriggerHalfCpltCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_TriggerHalfCpltCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Input Capture callback. \n
  *        Function called when an input capture interrupt is generated or when
  *        the DMA transfer triggered by the an input capture DMA request is completed.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param channel  Channel for the input capture.
  */
__WEAK void HAL_TIM_InputCaptureCallback(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(channel);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_InputCaptureCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Input Capture Half Complete callback. \n
  *        Function called when the DMA transfer triggered by an input capture
  *        DMA request is half completed.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param channel  Channel for the input capture.
  */
__WEAK void HAL_TIM_InputCaptureHalfCpltCallback(hal_tim_handle_t *htim,
                                                 hal_tim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(channel);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_InputCaptureHalfCpltCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Compare Match callback. \n
  *        Function called when a compare match interrupt is generated or when the
  *        DMA transfer triggered by the compare match DMA request is completed.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param channel  Channel for the output compare.
  */
__WEAK void HAL_TIM_CompareMatchCallback(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(channel);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_CompareMatchCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)

/**
  * @brief Compare Match Half Complete callback. \n
  *        Function called when the DMA transfer triggered by compare matche DMA
  *        request is half completed.
  * @param htim     Pointer to the handle of the TIM instance.
  * @param channel  Channel for the output compare.
  */
__WEAK void HAL_TIM_CompareMatchHalfCpltCallback(hal_tim_handle_t *htim,
                                                 hal_tim_channel_t channel)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(channel);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_CompareMatchHalfCpltCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Commutation callback. \n
  *        Function called when the timer commutation interrupt is generated or
  *        when the DMA transfer triggered by the commutation DMA request is completed.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_CommutationCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_CommutationCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief Commutation Half Complete callback. \n
  *        Function called when the DMA transfer triggered by the commutation DMA
  *        request is half completed.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_CommutationHalfCpltCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_CommutationHalfCpltCallback can be implemented in the user file.
   */
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief Break callback. \n
  *        Function called when the break interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_BreakCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_BreakCallback can be implemented in the user file.
   */
}

/**
  * @brief Break2 callback. \n
  *        Function called when the break2 interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_Break2Callback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_Break2Callback can be implemented in the user file.
   */
}

/**
  * @brief System Break callback. \n
  *        Function called when the system break interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_SystemBreakCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_SystemBreakCallback can be implemented in the user file.
   */
}

/**
  * @brief Software Break callback. \n
  *        Function called when the software break interrupt is generated.
  * @param htim   Pointer to the handle of the TIM instance.
  * @param brkin  The break input of interest.
  */
__WEAK void HAL_TIM_SoftwareBreakCallback(hal_tim_handle_t *htim, hal_tim_break_input_id_t brkin)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);
  STM32_UNUSED(brkin);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_SoftwareBreakCallback can be implemented in the user file.
   */
}

/**
  * @brief Encoder Index callback. \n
  *        Could be renamed HAL_TIM_IndexCallback
  *        Function called when the index interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_EncoderIndexCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_EncoderIndexCallback can be implemented in the user file.
   */
}

/**
  * @brief Encoder Direction Change callback. \n
  *        Function called when the direction change interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_DirectionChangeCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_DirectionChangeCallback can be implemented in the user file.
   */
}

/**
  * @brief Index Error callback. \n
  *        Function called when the index error interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_IndexErrorCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_IndexErrorCallback can be implemented in the user file.
   */
}

/**
  * @brief Transition Error callback. \n
  *        Function called when the transition error interrupt is generated.
  * @param htim  Pointer to the handle of the TIM instance.
  */
__WEAK void HAL_TIM_TransitionErrorCallback(hal_tim_handle_t *htim)
{
  /* Prevent unused argument(s) compilation warning */
  STM32_UNUSED(htim);

  /*
   * WARNING : This function must preferably not be modified, when the callback is needed,
   *           HAL_TIM_TransitionErrorCallback can be implemented in the user file.
   */
}

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the DMA Error.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterErrorCallback(hal_tim_handle_t *htim,
                                           hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->error_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the DMA stop callback.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterStopCallback(hal_tim_handle_t *htim,
                                          hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->stop_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the DMA channel stop callback.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterChannelStopCallback(hal_tim_handle_t *htim,
                                                 hal_tim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->channel_stop_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Update event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterUpdateCallback(hal_tim_handle_t *htim,
                                            hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->update_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the DMA Half Complete transfer
  *         triggered on Update event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterUpdateHalfCpltCallback(hal_tim_handle_t *htim,
                                                    hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->update_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Trigger event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterTriggerCallback(hal_tim_handle_t *htim,
                                             hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->trigger_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the DMA Half Complete transfer
  *         triggered by a Trigger event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterTriggerHalfCpltCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->trigger_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Input Capture event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterInputCaptureCallback(hal_tim_handle_t *htim,
                                                  hal_tim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->input_capture_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the DMA Half Complete transfer
  *         triggered by an Input Capture event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterInputCaptureHalfCpltCallback(hal_tim_handle_t *htim,
                                                          hal_tim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->input_capture_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Compare Match event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterCompareMatchCallback(hal_tim_handle_t *htim,
                                                  hal_tim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->compare_match_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the Half Complete DMA transfer
  *         triggered by a Compare Match event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterCompareMatchHalfCpltCallback(hal_tim_handle_t *htim,
                                                          hal_tim_channel_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->compare_match_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Commutation event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterCommutationCallback(hal_tim_handle_t *htim,
                                                 hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->commutation_callback = fct;

  return HAL_OK;
}

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief  Callback registration for the DMA Half Complete transfer
  *         triggered by a Commutation event.
  *         for the timer.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterCommutationHalfCpltCallback(hal_tim_handle_t *htim,
                                                         hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->commutation_half_cplt_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Callback registration for the Break event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterBreakCallback(hal_tim_handle_t *htim,
                                           hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->break_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Break 2 event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterBreak2Callback(hal_tim_handle_t *htim,
                                            hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->break2_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the System Break event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterSystemBreakCallback(hal_tim_handle_t *htim,
                                                 hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->system_break_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Software Break event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterSoftwareBreakCallback(hal_tim_handle_t *htim,
                                                   hal_tim_break_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->software_break_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Encoder Index event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterEncoderIndexCallback(hal_tim_handle_t *htim,
                                                  hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->encoder_index_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Encoder Direction Change event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterDirectionChangeCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->direction_change_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Encoder Index Error event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterIndexErrorCallback(hal_tim_handle_t *htim,
                                                hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->index_error_callback = fct;

  return HAL_OK;
}

/**
  * @brief  Callback registration for the Encoder Transition Error event.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @param  fct   Function to register as callback.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM fct is NULL (only if USE_HAL_CHECK_PARAM == 1)
  */
hal_status_t HAL_TIM_RegisterTransitionErrorCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct)
{
  ASSERT_DBG_PARAM((htim != NULL));
  ASSERT_DBG_PARAM((fct != NULL));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM ==1)
  if (fct == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  htim->transition_error_callback = fct;

  return HAL_OK;
}
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

/**
  *  @}
  */

/** @addtogroup TIM_Exported_Functions_Group18
  * @brief The user data pointer, *p_user_data, in the HAL TIM handle allows
  *        user to associate applicative user data to the HAL TIM handle.
  *        Thus, the two functions in this group give an application the
  *        possibility to store and retrieve user data pointer into and
  *        from the handle.
  *
  * @{
  */
#if defined (USE_HAL_TIM_USER_DATA) && (USE_HAL_TIM_USER_DATA == 1)
/**
  * @brief Store User Data pointer into the handle.
  * @param htim         Pointer to the handle of the TIM instance.
  * @param p_user_data  Pointer to the user data.
  */
void HAL_TIM_SetUserData(hal_tim_handle_t *htim, const void *p_user_data)
{
  ASSERT_DBG_PARAM((htim != NULL));

  htim->p_user_data = p_user_data;
}

/**
  * @brief  Retrieve User Data pointer from the handle.
  * @param  htim  Pointer to the handle of the TIM instance.
  * @retval (void*) Pointer to the user data, when previously
  *         set by @ref HAL_TIM_SetUserData().
  * @retval NULL otherwise.
  */
const void *HAL_TIM_GetUserData(const hal_tim_handle_t *htim)
{
  ASSERT_DBG_PARAM((htim != NULL));

  return htim->p_user_data;
}
#endif /* USE_HAL_TIM_USER_DATA */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* USE_HAL_TIM_MODULE */
#endif /* TIM1 || TIM2 || TIM3 || TIM4 || TIM5 || TIM6 || TIM7 || TIM8 || TIM12 || TIM15 || TIM16 || TIM17 */
/**
  * @}
  */
