/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_tim.h
  * @brief   Header file for the TIM HAL module.
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

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_TIM_H
#define STM32C5XX_HAL_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_tim.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
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

/** @defgroup TIM TIM
  * @{
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @addtogroup TIM_Private_Constants
  * @{
  */

/**
  * @brief Shift to apply to a value (period or pulse)
  *        to obtain the equivalent value when dithering is enabled.
  */
#define HAL_TIM_DITHERING_SHIFT  (4U)

/**
  * @brief Number of TIM channels.
  * @note  Number of fields in the @ref hal_tim_channel_t.
  */
#define HAL_TIM_CHANNELS  (11U)

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/*  */
/**
  * @brief Number of DMA requests.
  * @note  Number of fields in the @ref hal_tim_dma_index_t.
  */
#define HAL_TIM_DMA_REQUESTS  (7U)

/** @defgroup TIM_Substates Substate of an active state
  * @{
  */
/** Not silent substate */
#define HAL_TIM_ACTIVE_NOT_SILENT  (0U)
/** Silent substate */
#define HAL_TIM_ACTIVE_SILENT      (1U)
/**
  * @}
  */
#endif /* USE_HAL_TIM_DMA */

/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/
/**
  * @brief Macro to compute a value with a dithered value.
  */
#define HAL_TIM_COMPUTE_DITHERED_VALUE(value, dithering_pattern) \
  (((value) << HAL_TIM_DITHERING_SHIFT) | (dithering_pattern))

/**
  * @brief Macro to compute the period with a dithered value.
  */
#define HAL_TIM_COMPUTE_DITHERED_PERIOD(period, period_dithering_pattern) \
  HAL_TIM_COMPUTE_DITHERED_VALUE(period, period_dithering_pattern)

/**
  * @brief Macro to compute the pulse with a dithered value.
  */
#define HAL_TIM_COMPUTE_DITHERED_PULSE(pulse, pulse_dithering_pattern) \
  HAL_TIM_COMPUTE_DITHERED_VALUE(pulse, pulse_dithering_pattern)

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_Exported_Constants HAL TIM Constants
  * @{
  */

#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
/** @defgroup  TIM_Error_Code Error Code definition reflecting asynchronous process errors
  * @{
  */
#define HAL_TIM_ERROR_NONE  (0U)    /*!< No error            */
#define HAL_TIM_ERROR_DMA   (1U)    /*!< DMA transfer error  */
/**
  * @}
  */
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */


/** @defgroup TIM_Optional_Interruptions Optional interruptions
  * @note     To be used as parameters of functions @ref HAL_TIM_Start_IT_Opt(),
  *           @ref HAL_TIM_Start_DMA_Opt()
  * @{
  */
/**
  * @brief HAL TIM interrupts.
  *        The interrupts are grouped in the following categories:
  *        - Update:       Update interrupt
  *        - Commutation:  Commutation interrupt
  *        - Trigger:      Trigger interrupt
  *        - Break:        Break interrupt
  *        - Encoder:      Encoder interrupts
  * @note  These interrupts are used in @ref HAL_TIM_Start_IT_Opt().
  */
/** TIM optional update interrupt */
#define HAL_TIM_OPT_IT_UPDATE                    (LL_TIM_DIER_UIE)
/** TIM optional commutation interrupt */
#define HAL_TIM_OPT_IT_COMMUTATION               (LL_TIM_DIER_COMIE)
/** TIM optional trigger interrupt */
#define HAL_TIM_OPT_IT_TRIGGER_INPUT             (LL_TIM_DIER_TIE)
/** TIM optional break interrupt */
#define HAL_TIM_OPT_IT_BREAK                     (LL_TIM_DIER_BIE)
/** TIM optional encoder index interrupt */
#define HAL_TIM_OPT_IT_ENCODER_INDEX             (LL_TIM_DIER_IDXIE)
/** TIM optional encoder direction interrupt */
#define HAL_TIM_OPT_IT_ENCODER_DIRECTION         (LL_TIM_DIER_DIRIE)
/** TIM optional encoder index error interrupt */
#define HAL_TIM_OPT_IT_ENCODER_INDEX_ERROR       (LL_TIM_DIER_IERRIE)
/** TIM optional encoder transition error interrupt */
#define HAL_TIM_OPT_IT_ENCODER_TRANSITION_ERROR  (LL_TIM_DIER_TERRIE)

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief HAL TIM Filters for the DMA interrupts.
  *        By default the half transfer complete callbacks are disabled.
  * @note  These interrupts are used in @ref HAL_TIM_Start_DMA_Opt().
  */
/** Disable DMA half transfer complete callbacks */
#define HAL_TIM_OPT_DMA_IT_NONE       (HAL_DMA_OPT_IT_NONE)
/** Enable the half-transfer callbacks */
#define HAL_TIM_OPT_DMA_IT_HT         (HAL_DMA_OPT_IT_HT)
/** Enable the half-transfer and transfer complete callbacks */
#define HAL_TIM_OPT_DMA_IT_DEFAULT    (HAL_DMA_OPT_IT_DEFAULT)

/** All interrupts are filtered */
#if defined(USE_HAL_DMA_LINKEDLIST) && (USE_HAL_DMA_LINKEDLIST == 1)
#define HAL_TIM_OPT_DMA_IT_SILENT     (HAL_DMA_OPT_IT_SILENT)
#endif /* USE_HAL_DMA_LINKEDLIST */
#endif /* USE_HAL_TIM_DMA */
/**
  * @}
  */

/** @defgroup TIM_Group_Channel5 Group Channel 5
  * @note     To be used as parameters of function @ref HAL_TIM_OC_SetGroupChannel()
  * @{
  */
/**
  * @brief HAL TIM Group Channel 5.
  */
/** No effect of channel 5 on channels 1, 2 and 3 */
#define HAL_TIM_GROUP_NONE         (LL_TIM_GROUPCH5_NONE)

/** Group channel 5 and channel 1 (ANDed) */
#define HAL_TIM_GROUP_AND_OC1REFC  (LL_TIM_GROUPCH5_AND_OC1REFC)

/** Group channel 5 and channel 2 (ANDed) */
#define HAL_TIM_GROUP_AND_OC2REFC  (LL_TIM_GROUPCH5_AND_OC2REFC)

/** Group channel 5 and channel 3 (ANDed) */
#define HAL_TIM_GROUP_AND_OC3REFC  (LL_TIM_GROUPCH5_AND_OC3REFC)

/** Group channel 5 and channel 4 (ANDed) */
#define HAL_TIM_GROUP_AND_OC4REFC  (LL_TIM_GROUPCH5_AND_OC4REFC)

/** Group channel 5 and channel 1 (ORed) */
#define HAL_TIM_GROUP_OR_OC1REFC   (LL_TIM_GROUPCH5_OR_OC1REFC)

/** Group channel 5 and channel 2 (ORed) */
#define HAL_TIM_GROUP_OR_OC2REFC   (LL_TIM_GROUPCH5_OR_OC2REFC)

/** Group channel 5 and channel 3 (ORed) */
#define HAL_TIM_GROUP_OR_OC3REFC   (LL_TIM_GROUPCH5_OR_OC3REFC)

/** Group channel 5 and channel 4 (ORed) */
#define HAL_TIM_GROUP_OR_OC4REFC   (LL_TIM_GROUPCH5_OR_OC4REFC)
/**
  * @}
  */


/**
  * @defgroup TIM_Break_Input_Sources Break Input Sources
  * @{
  */
/**
  * @brief  HAL TIM Break Input Sources.
  * @note   The pattern of the defines is: HAL_TIM_BRK_TIMx_<source>
  */
/** TIM1 break input is connected to TIM1_BKIN */
#define HAL_TIM_BRK_TIM1_GPIO         (LL_TIM_TIM1_BRK_GPIO)
/** TIM1 break input is connected to comp1_out */
#define HAL_TIM_BRK_TIM1_COMP1_OUT    (LL_TIM_TIM1_BRK_COMP1_OUT)
#if defined(COMP2)
/** TIM1 break input is connected to comp2_out */
#define HAL_TIM_BRK_TIM1_COMP2_OUT    (LL_TIM_TIM1_BRK_COMP2_OUT)
#endif /* COMP2 */
/** TIM1 break input is connected to TIM8_BKIN */
#define HAL_TIM_BRK_TIM1_TIM8_BKIN    (LL_TIM_TIM1_BRK_TIM8_BKIN)
/** TIM1 break input is connected to TIM15_BKIN */
#define HAL_TIM_BRK_TIM1_TIM15_BKIN   (LL_TIM_TIM1_BRK_TIM15_BKIN)
#if defined(TIM16)
/** TIM1 break input is connected to TIM16_BKIN */
#define HAL_TIM_BRK_TIM1_TIM16_BKIN   (LL_TIM_TIM1_BRK_TIM16_BKIN)
/** TIM1 break input is connected to TIM17_BKIN */
#define HAL_TIM_BRK_TIM1_TIM17_BKIN   (LL_TIM_TIM1_BRK_TIM17_BKIN)
#endif /* TIM16 */


/** TIM8 break input is connected to TIM8_BKIN */
#define HAL_TIM_BRK_TIM8_GPIO         (LL_TIM_TIM8_BRK_GPIO)
/** TIM8 break input is connected to comp1_out */
#define HAL_TIM_BRK_TIM8_COMP1_OUT    (LL_TIM_TIM8_BRK_COMP1_OUT)
#if defined(COMP2)
/** TIM8 break input is connected to comp2_out */
#define HAL_TIM_BRK_TIM8_COMP2_OUT    (LL_TIM_TIM8_BRK_COMP2_OUT)
#endif /* COMP2 */
/** TIM8 break input is connected to TIM1_BKIN */
#define HAL_TIM_BRK_TIM8_TIM1_BKIN    (LL_TIM_TIM8_BRK_TIM1_BKIN)
/** TIM8 break input is connected to TIM15_BKIN */
#define HAL_TIM_BRK_TIM8_TIM15_BKIN   (LL_TIM_TIM8_BRK_TIM15_BKIN)
#if defined(TIM16)
/** TIM8 break input is connected to TIM16_BKIN */
#define HAL_TIM_BRK_TIM8_TIM16_BKIN   (LL_TIM_TIM8_BRK_TIM16_BKIN)
/** TIM8 break input is connected to TIM17_BKIN */
#define HAL_TIM_BRK_TIM8_TIM17_BKIN   (LL_TIM_TIM8_BRK_TIM17_BKIN)
#endif /* TIM16 */


/** TIM15 break input is connected to TIM15_BKIN */
#define HAL_TIM_BRK_TIM15_GPIO         (LL_TIM_TIM15_BRK_GPIO)
/** TIM15 break input is connected to comp1_out */
#define HAL_TIM_BRK_TIM15_COMP1_OUT    (LL_TIM_TIM15_BRK_COMP1_OUT)
#if defined(COMP2)
/** TIM15 break input is connected to comp2_out */
#define HAL_TIM_BRK_TIM15_COMP2_OUT    (LL_TIM_TIM15_BRK_COMP2_OUT)
#endif /* COMP2 */
/** TIM15 break input is connected to TIM1_BKIN */
#define HAL_TIM_BRK_TIM15_TIM1_BKIN    (LL_TIM_TIM15_BRK_TIM1_BKIN)
/** TIM15 break input is connected to TIM8_BKIN */
#define HAL_TIM_BRK_TIM15_TIM8_BKIN    (LL_TIM_TIM15_BRK_TIM8_BKIN)
#if defined(TIM16)
/** TIM15 break input is connected to TIM16_BKIN */
#define HAL_TIM_BRK_TIM15_TIM16_BKIN   (LL_TIM_TIM15_BRK_TIM16_BKIN)
/** TIM15 break input is connected to TIM17_BKIN */
#define HAL_TIM_BRK_TIM15_TIM17_BKIN   (LL_TIM_TIM15_BRK_TIM17_BKIN)
#endif /* TIM16 */


#if defined(TIM16)
/** TIM16 break input is connected to TIM16_BKIN */
#define HAL_TIM_BRK_TIM16_GPIO         (LL_TIM_TIM16_BRK_GPIO)
/** TIM16 break input is connected to comp1_out */
#define HAL_TIM_BRK_TIM16_COMP1_OUT    (LL_TIM_TIM16_BRK_COMP1_OUT)
/** TIM16 break input is connected to TIM1_BKIN */
#define HAL_TIM_BRK_TIM16_TIM1_BKIN    (LL_TIM_TIM16_BRK_TIM1_BKIN)
/** TIM16 break input is connected to TIM8_BKIN */
#define HAL_TIM_BRK_TIM16_TIM8_BKIN    (LL_TIM_TIM16_BRK_TIM8_BKIN)
/** TIM16 break input is connected to TIM15_BKIN */
#define HAL_TIM_BRK_TIM16_TIM15_BKIN   (LL_TIM_TIM16_BRK_TIM15_BKIN)
/** TIM16 break input is connected to TIM17_BKIN */
#define HAL_TIM_BRK_TIM16_TIM17_BKIN   (LL_TIM_TIM16_BRK_TIM17_BKIN)
#endif /* TIM16 */


#if defined(TIM17)
/** TIM17 break input is connected to TIM17_BKIN */
#define HAL_TIM_BRK_TIM17_GPIO         (LL_TIM_TIM17_BRK_GPIO)
/** TIM17 break input is connected to comp1_out */
#define HAL_TIM_BRK_TIM17_COMP1_OUT    (LL_TIM_TIM17_BRK_COMP1_OUT)
/** TIM17 break input is connected to TIM1_BKIN */
#define HAL_TIM_BRK_TIM17_TIM1_BKIN    (LL_TIM_TIM17_BRK_TIM1_BKIN)
/** TIM17 break input is connected to TIM8_BKIN */
#define HAL_TIM_BRK_TIM17_TIM8_BKIN    (LL_TIM_TIM17_BRK_TIM8_BKIN)
/** TIM17 break input is connected to TIM15_BKIN */
#define HAL_TIM_BRK_TIM17_TIM15_BKIN   (LL_TIM_TIM17_BRK_TIM15_BKIN)
/** TIM17 break input is connected to TIM16_BKIN */
#define HAL_TIM_BRK_TIM17_TIM16_BKIN   (LL_TIM_TIM17_BRK_TIM16_BKIN)
#endif /* TIM17 */

/**
  * @brief  HAL TIM Break2 Input Sources.
  * @note   The pattern of the defines is: HAL_TIM_BRK2_TIMx_<source>
  */
/** TIM1 break2 input is connected to TIM1_BKIN2 */
#define HAL_TIM_BRK2_TIM1_GPIO         (LL_TIM_TIM1_BRK2_GPIO)
/** TIM1 break2 input is connected to comp1_out */
#define HAL_TIM_BRK2_TIM1_COMP1_OUT    (LL_TIM_TIM1_BRK2_COMP1_OUT)
#if defined(COMP2)
/** TIM1 break2 input is connected to comp2_out */
#define HAL_TIM_BRK2_TIM1_COMP2_OUT    (LL_TIM_TIM1_BRK2_COMP2_OUT)
#endif /* COMP2 */
/** TIM1 break2 input is connected to TIM8_BKIN2 */
#define HAL_TIM_BRK2_TIM1_TIM8_BKIN2   (LL_TIM_TIM1_BRK2_TIM8_BKIN2)


/** TIM8 break2 input is connected to TIM8_BKIN2 */
#define HAL_TIM_BRK2_TIM8_GPIO         (LL_TIM_TIM8_BRK2_GPIO)
/** TIM8 break2 input is connected to comp1_out */
#define HAL_TIM_BRK2_TIM8_COMP1_OUT    (LL_TIM_TIM8_BRK2_COMP1_OUT)
#if defined(COMP2)
/** TIM8 break2 input is connected to comp2_out */
#define HAL_TIM_BRK2_TIM8_COMP2_OUT    (LL_TIM_TIM8_BRK2_COMP2_OUT)
#endif /* COMP2 */
/** TIM8 break2 input is connected to TIM1_BKIN2 */
#define HAL_TIM_BRK2_TIM8_TIM1_BKIN2   (LL_TIM_TIM8_BRK2_TIM1_BKIN2)

/**
  * @}
  */
/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_Exported_Types HAL TIM Types
  * @{
  */

/**
  * @brief HAL TIM instance.
  */
typedef enum
{
  HAL_TIM1  = TIM1_BASE,

  HAL_TIM2  = TIM2_BASE,

#if defined(TIM3)
  HAL_TIM3  = TIM3_BASE,
#endif /* TIM3 */

#if defined(TIM4)
  HAL_TIM4  = TIM4_BASE,
#endif /* TIM4 */

#if defined(TIM5)
  HAL_TIM5  = TIM5_BASE,
#endif /* TIM5 */

  HAL_TIM6  = TIM6_BASE,

  HAL_TIM7  = TIM7_BASE,

  HAL_TIM8  = TIM8_BASE,

  HAL_TIM12 = TIM12_BASE,

  HAL_TIM15 = TIM15_BASE,

#if defined(TIM16)
  HAL_TIM16 = TIM16_BASE,

  HAL_TIM17 = TIM17_BASE,
#endif /* TIM16 */


} hal_tim_t;


/**
  * @brief  HAL TIM Global States definition.
  */
typedef enum
{
  /** Peripheral not yet initialized                     */
  HAL_TIM_STATE_RESET   = 0U,

  /** Peripheral initialized but not yet configured      */
  HAL_TIM_STATE_INIT    = (1UL << 31U),

  /** Peripheral initialized and a global config applied */
  HAL_TIM_STATE_IDLE    = (1UL << 30U),

  /** Counter is running */
  HAL_TIM_STATE_ACTIVE  = (1UL << 29U),

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  HAL_TIM_STATE_ACTIVE_SILENT = (HAL_TIM_STATE_ACTIVE | HAL_TIM_ACTIVE_SILENT),
#endif /* USE_HAL_TIM_DMA */

} hal_tim_state_t;


/**
  * @brief  TIM Channel States definition.
  */
typedef enum
{
  /** TIM Channel initial state                                */
  HAL_TIM_CHANNEL_STATE_RESET       = (1UL << 31U),

  /** TIM Channel ready for use as output channel              */
  HAL_TIM_OC_CHANNEL_STATE_IDLE     = (1UL << 30U),

  /** An internal process is ongoing on the TIM output channel */
  HAL_TIM_OC_CHANNEL_STATE_ACTIVE   = (1UL << 29U),

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** An internal process is ongoing on the TIM output channel in DMA silent mode */
  HAL_TIM_OC_CHANNEL_STATE_ACTIVE_SILENT   = (HAL_TIM_OC_CHANNEL_STATE_ACTIVE \
                                              | HAL_TIM_ACTIVE_SILENT),
#endif /* USE_HAL_TIM_DMA */

  /** TIM Channel ready for use as input channel               */
  HAL_TIM_IC_CHANNEL_STATE_IDLE     = (1UL << 28U),

  /** An internal process is ongoing on the TIM input channel */
  HAL_TIM_IC_CHANNEL_STATE_ACTIVE   = (1UL << 27U),

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)

  /** An internal process is ongoing on the TIM input channel in DMA silent mode */
  HAL_TIM_IC_CHANNEL_STATE_ACTIVE_SILENT   = (HAL_TIM_IC_CHANNEL_STATE_ACTIVE \
                                              | HAL_TIM_ACTIVE_SILENT),
#endif /* USE_HAL_TIM_DMA */

} hal_tim_channel_state_t;


/**
  * @brief  HAL TIM Channels identifier definition.
  */
typedef enum
{
  /** Timer input/output channel 1         */
  HAL_TIM_CHANNEL_1   = 0U,

  /** Timer input/output channel 2         */
  HAL_TIM_CHANNEL_2   = 1U,

  /** Timer input/output channel 3         */
  HAL_TIM_CHANNEL_3   = 2U,

  /** Timer input/output channel 4         */
  HAL_TIM_CHANNEL_4   = 3U,

  /** Timer output channel 5               */
  HAL_TIM_CHANNEL_5   = 4U,

  /** Timer output channel 6               */
  HAL_TIM_CHANNEL_6   = 5U,

  /** Timer output channel 7               */
  HAL_TIM_CHANNEL_7   = 6U,

  /** Timer complementary output channel 1 */
  HAL_TIM_CHANNEL_1N  = 7U,

  /** Timer complementary output channel 2 */
  HAL_TIM_CHANNEL_2N  = 8U,

  /** Timer complementary output channel 3 */
  HAL_TIM_CHANNEL_3N  = 9U,

  /** Timer complementary output channel 4 */
  HAL_TIM_CHANNEL_4N  = 10U,

} hal_tim_channel_t;


/**
  * @brief HAL TIM Counter Mode.
  */
typedef enum
{
  /** Counter used as up-counter                                     */
  HAL_TIM_COUNTER_UP   = LL_TIM_COUNTERMODE_UP,

  /** Counter used as down-counter                                   */
  HAL_TIM_COUNTER_DOWN = LL_TIM_COUNTERMODE_DOWN,

  /** Center-aligned mode 1                                          */
  HAL_TIM_COUNTER_CENTER_DOWN = LL_TIM_COUNTERMODE_CENTER_DOWN,

  /** Center-aligned mode 2                                          */
  HAL_TIM_COUNTER_CENTER_UP = LL_TIM_COUNTERMODE_CENTER_UP,

  /** Center-aligned mode 3                                          */
  HAL_TIM_COUNTER_CENTER_UP_DOWN = LL_TIM_COUNTERMODE_CENTER_UP_DOWN,

} hal_tim_counter_mode_t;


/**
  * @brief TIM DTS Clock Prescaler.
  */
typedef enum
{
  /** tDTS=tTIM_KER_CK                        */
  HAL_TIM_DTS_DIV1 = LL_TIM_CLOCKDIVISION_DIV1,

  /** tDTS=2*tTIM_KER_CK                      */
  HAL_TIM_DTS_DIV2 = LL_TIM_CLOCKDIVISION_DIV2,

  /** tDTS=4*tTIM_KER_CK                      */
  HAL_TIM_DTS_DIV4 = LL_TIM_CLOCKDIVISION_DIV4,

  /** tDTS=8*tTIM_KER_CK                      */
  HAL_TIM_DTS_DIV8 = LL_TIM_CLOCKDIVISION_DIV8,

} hal_tim_dts_prescaler_t;


/**
  * @brief TIM DTS2 Clock Prescaler.
  */
typedef enum
{
  /** tDTS2=tDTS                                          */
  HAL_TIM_DTS2_DIV1 = LL_TIM_CLOCKDIVISION2_DIV1,

  /** tDTS2=4*tDTS                                        */
  HAL_TIM_DTS2_DIV4 = LL_TIM_CLOCKDIVISION2_DIV4,

  /** tDTS2=16*tDTS                                       */
  HAL_TIM_DTS2_DIV16 = LL_TIM_CLOCKDIVISION2_DIV16,

  /** tDTS2=64*tDTS                                       */
  HAL_TIM_DTS2_DIV64 = LL_TIM_CLOCKDIVISION2_DIV64,

  /** tDTS2=256*tDTS                                      */
  HAL_TIM_DTS2_DIV256 = LL_TIM_CLOCKDIVISION2_DIV256,

  /** tDTS2=1024*tDTS                                     */
  HAL_TIM_DTS2_DIV1024 = LL_TIM_CLOCKDIVISION2_DIV1024,

  /** tDTS2=4096*tDTS                                     */
  HAL_TIM_DTS2_DIV4096 = LL_TIM_CLOCKDIVISION2_DIV4096,

  /** tDTS2=16384*tDTS                                    */
  HAL_TIM_DTS2_DIV16384 = LL_TIM_CLOCKDIVISION2_DIV16384,

  /** tDTS2=65536*tDTS                                    */
  HAL_TIM_DTS2_DIV65536 = LL_TIM_CLOCKDIVISION2_DIV65536,

  /** tDTS2=262144*tDTS                                   */
  HAL_TIM_DTS2_DIV262144 = LL_TIM_CLOCKDIVISION2_DIV262144,

} hal_tim_dts2_prescaler_t;


/**
  * @brief HAL TIM Clock Source definition.
  */
typedef enum
{
  /** Internal clock source (timer kernel clock)                                   */
  HAL_TIM_CLK_INTERNAL = LL_TIM_CLK_INTERNAL,

  /** External clock source mode 1                                                 */
  HAL_TIM_CLK_EXTERNAL_MODE1 = LL_TIM_CLK_EXTERNAL_MODE1,

  /** External clock source mode 2                                                 */
  HAL_TIM_CLK_EXTERNAL_MODE2 = LL_TIM_CLK_EXTERNAL_MODE2,

  /** Quadrature encoder mode: x1 mode, counting on TI1FP1 edges only,
      edge sensitivity is set by CC1P                                              */
  HAL_TIM_CLK_ENCODER_X1_TI1 = LL_TIM_CLK_ENCODER_X1_TI1,

  /** Quadrature encoder mode: x1 mode, counting on TI2FP2 edges only,
      edge sensitivity is set by CC2P                                              */
  HAL_TIM_CLK_ENCODER_X1_TI2 = LL_TIM_CLK_ENCODER_X1_TI2,

  /** Quadrature encoder mode 1: x2 mode, counts up/down on TI1FP1
      edge depending on TI2FP2 level                                               */
  HAL_TIM_CLK_ENCODER_X2_TI1 = LL_TIM_CLK_ENCODER_X2_TI1,

  /** Quadrature encoder mode 2: x2 mode, counts up/down on TI2FP2
      edge depending on TI1FP1 level                                               */
  HAL_TIM_CLK_ENCODER_X2_TI2 = LL_TIM_CLK_ENCODER_X2_TI2,

  /** Quadrature encoder mode 3: x4 mode, counts up/down on both
      TI1FP1 and TI2FP2 edges depending on the level of the other input            */
  HAL_TIM_CLK_ENCODER_X4_TI12 = LL_TIM_CLK_ENCODER_X4_TI12,

  /** Quadrature encoder with built-in debouncer: x2 mode, counts up/down
      on TI1FP1 edge depending on TI2FP2 level                                     */
  HAL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1 = LL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1,

  /** Quadrature encoder with built-in debouncer: x4 mode, counts up/down on
      both TI1FP1 and TI2FP2 edges depending on the level of the other input       */
  HAL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12 = LL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12,

  /** Encoder mode: Clock plus direction, x2 mode                                  */
  HAL_TIM_CLK_ENCODER_CLK_PLUS_X2 = LL_TIM_CLK_ENCODER_CLK_PLUS_X2,

  /** Encoder mode: Clock plus direction, x1 mode,
      TI2FP2 edge sensitivity is set by CC2P                                       */
  HAL_TIM_CLK_ENCODER_CLK_PLUS_X1 = LL_TIM_CLK_ENCODER_CLK_PLUS_X1,

  /** Encoder mode: Directional Clock, x2 mode                                     */
  HAL_TIM_CLK_ENCODER_DIR_CLK_X2 = LL_TIM_CLK_ENCODER_DIR_CLK_X2,

  /** Encoder mode: Directional Clock, x1 mode,
      TI1FP1 and TI2FP2 edge sensitivity is set by CC1P and CC2P                   */
  HAL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12 = LL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12,

} hal_tim_clk_src_t;


/**
  * @brief HAL TIM Trigger Selection.
  */
typedef enum
{
  /** Internal Trigger 0 (ITR0) */
  HAL_TIM_TRIG_ITR0    = LL_TIM_TS_ITR0,

  /** Internal Trigger 1 (ITR1) */
  HAL_TIM_TRIG_ITR1    = LL_TIM_TS_ITR1,

  /** Internal Trigger 2 (ITR2) */
  HAL_TIM_TRIG_ITR2    = LL_TIM_TS_ITR2,

#if defined(TIM3)
  /** Internal Trigger 3 (ITR3) */
  HAL_TIM_TRIG_ITR3    = LL_TIM_TS_ITR3,
#endif /* TIM3 */

#if defined(TIM4)
  /** Internal Trigger 4 (ITR4) */
  HAL_TIM_TRIG_ITR4    = LL_TIM_TS_ITR4,
#endif /* TIM4 */

#if defined(TIM5)
  /** Internal Trigger 5 (ITR5) */
  HAL_TIM_TRIG_ITR5    = LL_TIM_TS_ITR5,
#endif /* TIM5 */

  /** Internal Trigger 6 (ITR6) */
  HAL_TIM_TRIG_ITR6    = LL_TIM_TS_ITR6,

  /** Internal Trigger 7 (ITR7) */
  HAL_TIM_TRIG_ITR7    = LL_TIM_TS_ITR7,

  /** Internal Trigger 8 (ITR8) */
  HAL_TIM_TRIG_ITR8    = LL_TIM_TS_ITR8,

  /** Internal Trigger 9 (ITR9) */
  HAL_TIM_TRIG_ITR9    = LL_TIM_TS_ITR9,

#if defined(TIM16)
  /** Internal Trigger 10 (ITR10) */
  HAL_TIM_TRIG_ITR10   = LL_TIM_TS_ITR10,

  /** Internal Trigger 11 (ITR11) */
  HAL_TIM_TRIG_ITR11   = LL_TIM_TS_ITR11,
#endif /* TIM16 */


  /** Timer Input 1 Edge Detector (TI1F_ED)  */
  HAL_TIM_TRIG_TI1F_ED = LL_TIM_TS_TI1F_ED,

  /** Filtered Timer Input 1 (TI1FP1)        */
  HAL_TIM_TRIG_TI1FP1  = LL_TIM_TS_TI1FP1,

  /** Filtered Timer Input 2 (TI2FP2)        */
  HAL_TIM_TRIG_TI2FP2  = LL_TIM_TS_TI2FP2,

  /** Filtered External Trigger input (ETRF) */
  HAL_TIM_TRIG_ETRF    = LL_TIM_TS_ETRF,

} hal_tim_trig_sel_t;


/**
  * @brief HAL TIM Clock selection.
  */
typedef struct
{
  /** TIM clock source. \n
      Specifies the source of the clock feeding the timer's prescaler. */
  hal_tim_clk_src_t clock_source;

  /** Input Trigger source. \n
      Specifies the trigger input to be used to synchronize the counter
      when HAL_TIM_CLK_EXTERNAL_MODE1 is selected as clock source. */
  hal_tim_trig_sel_t trigger;

} hal_tim_clock_sel_t;

/**
  * @brief HAL TIM Update Event Generation Status.
  */
typedef enum
{
  /** Update event is not generated */
  HAL_TIM_UPDATE_GENERATION_DISABLED = 0U,

  /** Update event is generated as per configured update event source */
  HAL_TIM_UPDATE_GENERATION_ENABLED  = 1U,

} hal_tim_update_generation_status_t;


/**
  * @brief  Update Event Source.
  */
typedef enum
{
  /** Update event is generated when:
      - The counter reaches overflow/underflow
      - The TIMx_EGR.UG bit is set by software
      - An internal/external trigger is active
        (through the slave mode controller)           */
  HAL_TIM_UPDATE_REGULAR = LL_TIM_UPDATESOURCE_REGULAR,

  /** Update event is generated only when the counter
      reaches overflow/underflow                      */
  HAL_TIM_UPDATE_COUNTER = LL_TIM_UPDATESOURCE_COUNTER,

} hal_tim_update_src_t;


/**
  * @brief HAL TIM Update Flag Remap Status.
  */
typedef enum
{
  /** UIF status bit is not copied to TIMx_CNT register bit 31  */
  HAL_TIM_UPDATE_FLAG_REMAP_DISABLED = 0U,

  /** UIF status bit is copied to TIMx_CNT register bit 31      */
  HAL_TIM_UPDATE_FLAG_REMAP_ENABLED  = 1U,

} hal_tim_update_flag_remap_status_t;


/**
  * @brief HAL TIM Auto-Reload Preload Status.
  */
typedef enum
{
  /** TIMx_ARR register is not preloaded  */
  HAL_TIM_AUTO_RELOAD_PRELOAD_DISABLED = 0U,

  /** TIMx_ARR register is preloaded      */
  HAL_TIM_AUTO_RELOAD_PRELOAD_ENABLED  = 1U,

} hal_tim_auto_reload_preload_status_t;


/**
  * @brief HAL TIM Digital Filter.
  */
typedef enum
{
  /** No filter, sampling is done at fDTS                       */
  HAL_TIM_FDIV1    = 0x00000000U,

  /** fSAMPLING=fTIM_KER_CK, N=2                                */
  HAL_TIM_FDIV1_N2 = 0x10000000U,

  /** fSAMPLING=fTIM_KER_CK, N=4                                */
  HAL_TIM_FDIV1_N4 = 0x20000000U,

  /** fSAMPLING=fTIM_KER_CK, N=8                                */
  HAL_TIM_FDIV1_N8 = 0x30000000U,

  /** fSAMPLING=fDTS/2, N=6                                     */
  HAL_TIM_FDIV2_N6 = 0x40000000U,

  /** fSAMPLING=fDTS/2, N=8                                     */
  HAL_TIM_FDIV2_N8 = 0x50000000U,

  /** fSAMPLING=fDTS/4, N=6                                     */
  HAL_TIM_FDIV4_N6 = 0x60000000U,

  /** fSAMPLING=fDTS/4, N=8                                     */
  HAL_TIM_FDIV4_N8 = 0x70000000U,

  /** fSAMPLING=fDTS/8, N=6                                     */
  HAL_TIM_FDIV8_N6 = 0x80000000U,

  /** fSAMPLING=fDTS/8, N=8                                     */
  HAL_TIM_FDIV8_N8 = 0x90000000U,

  /** fSAMPLING=fDTS/16, N=5                                    */
  HAL_TIM_FDIV16_N5 = 0xA0000000U,

  /** fSAMPLING=fDTS/16, N=6                                    */
  HAL_TIM_FDIV16_N6 = 0xB0000000U,

  /** fSAMPLING=fDTS/16, N=8                                    */
  HAL_TIM_FDIV16_N8 = 0xC0000000U,

  /** fSAMPLING=fDTS/32, N=5                                    */
  HAL_TIM_FDIV32_N5 = 0xD0000000U,

  /** fSAMPLING=fDTS/32, N=6                                    */
  HAL_TIM_FDIV32_N6 = 0xE0000000U,

  /** fSAMPLING=fDTS/32, N=8                                    */
  HAL_TIM_FDIV32_N8 = 0xF0000000U,

} hal_tim_filter_t;


/**
  * @brief  HAL TIM Output Compare Unit identifier definition.
  */
typedef enum
{
  /** Timer output compare unit 1      */
  HAL_TIM_OC_COMPARE_UNIT_1   = LL_TIM_OC_COMPARE_UNIT_1,

  /** Timer output compare unit 2      */
  HAL_TIM_OC_COMPARE_UNIT_2   = LL_TIM_OC_COMPARE_UNIT_2,

  /** Timer output compare unit 3      */
  HAL_TIM_OC_COMPARE_UNIT_3   = LL_TIM_OC_COMPARE_UNIT_3,

  /** Timer output compare unit 4      */
  HAL_TIM_OC_COMPARE_UNIT_4   = LL_TIM_OC_COMPARE_UNIT_4,

  /** Timer output compare unit 5      */
  HAL_TIM_OC_COMPARE_UNIT_5   = LL_TIM_OC_COMPARE_UNIT_5,

  /** Timer output compare unit 6      */
  HAL_TIM_OC_COMPARE_UNIT_6   = LL_TIM_OC_COMPARE_UNIT_6,

  /** Timer output compare unit 7      */
  HAL_TIM_OC_COMPARE_UNIT_7   = LL_TIM_OC_COMPARE_UNIT_7,

} hal_tim_oc_compare_unit_t;


/**
  * @brief HAL TIM Output Channel Mode.
  */
typedef enum
{
  /** The comparison between the output compare register TIMx_CCRy and
      the counter TIMx_CNT has no effect on the output channel level   */
  HAL_TIM_OC_FROZEN = LL_TIM_OCMODE_FROZEN,

  /** Set channel to active level on match                             */
  HAL_TIM_OC_ACTIVE_ON_MATCH = LL_TIM_OCMODE_ACTIVE_ON_MATCH,

  /** Set channel to inactive level on match                           */
  HAL_TIM_OC_INACTIVE_ON_MATCH = LL_TIM_OCMODE_INACTIVE_ON_MATCH,

  /** Toggle mode                                                      */
  HAL_TIM_OC_TOGGLE = LL_TIM_OCMODE_TOGGLE,

  /** PWM mode 1                                                       */
  HAL_TIM_OC_PWM1 = LL_TIM_OCMODE_PWM1,

  /** PWM mode 2                                                       */
  HAL_TIM_OC_PWM2 = LL_TIM_OCMODE_PWM2,

  /** Force active level                                               */
  HAL_TIM_OC_FORCED_ACTIVE = LL_TIM_OCMODE_FORCED_ACTIVE,

  /** Force inactive level                                             */
  HAL_TIM_OC_FORCED_INACTIVE = LL_TIM_OCMODE_FORCED_INACTIVE,

  /** Retrigerrable OPM mode 1                                         */
  HAL_TIM_OC_RETRIGERRABLE_OPM1 = LL_TIM_OCMODE_RETRIGERRABLE_OPM1,

  /** Retrigerrable OPM mode 2                                         */
  HAL_TIM_OC_RETRIGERRABLE_OPM2 = LL_TIM_OCMODE_RETRIGERRABLE_OPM2,

  /** Combined PWM mode 1                                              */
  HAL_TIM_OC_COMBINED_PWM1 = LL_TIM_OCMODE_COMBINED_PWM1,

  /** Combined PWM mode 2                                              */
  HAL_TIM_OC_COMBINED_PWM2 = LL_TIM_OCMODE_COMBINED_PWM2,

  /** Combined PWM mode 3                                              */
  HAL_TIM_OC_COMBINED_PWM3 = LL_TIM_OCMODE_COMBINED_PWM3,

  /** Combined PWM mode 4                                              */
  HAL_TIM_OC_COMBINED_PWM4 = LL_TIM_OCMODE_COMBINED_PWM4,

  /** Asymmetric PWM mode 1                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM1 = LL_TIM_OCMODE_ASYMMETRIC_PWM1,

  /** Asymmetric PWM mode 2                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM2 = LL_TIM_OCMODE_ASYMMETRIC_PWM2,

  /** Asymmetric PWM mode 3                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM3 = LL_TIM_OCMODE_ASYMMETRIC_PWM3,

  /** Asymmetric PWM mode 4                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM4 = LL_TIM_OCMODE_ASYMMETRIC_PWM4,

  /** Asymmetric PWM mode 5                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM5 = LL_TIM_OCMODE_ASYMMETRIC_PWM5,

  /** Asymmetric PWM mode 6                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM6 = LL_TIM_OCMODE_ASYMMETRIC_PWM6,

  /** Asymmetric PWM mode 7                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM7 = LL_TIM_OCMODE_ASYMMETRIC_PWM7,

  /** Asymmetric PWM mode 8                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM8 = LL_TIM_OCMODE_ASYMMETRIC_PWM8,

  /** Asymmetric PWM mode 9                                            */
  HAL_TIM_OC_ASYMMETRIC_PWM9 = LL_TIM_OCMODE_ASYMMETRIC_PWM9,

  /** Asymmetric PWM mode 10                                           */
  HAL_TIM_OC_ASYMMETRIC_PWM10 = LL_TIM_OCMODE_ASYMMETRIC_PWM10,

  /** Pulse on compare (CH3 and CH4 only)                              */
  HAL_TIM_OC_PULSE_ON_COMPARE = LL_TIM_OCMODE_PULSE_ON_COMPARE,

  /** Direction output (CH3 and CH4 only)                              */
  HAL_TIM_OC_DIRECTION_OUTPUT = LL_TIM_OCMODE_DIRECTION_OUTPUT,

} hal_tim_oc_mode_t;


/**
  * @brief HAL TIM Output Channel Polarity.
  */
typedef enum
{
  /** Output Channel (complementary output channel) active high  */
  HAL_TIM_OC_HIGH = LL_TIM_OCPOLARITY_HIGH,

  /** Output Channel (complementary output channel) active low   */
  HAL_TIM_OC_LOW = LL_TIM_OCPOLARITY_LOW,

} hal_tim_oc_polarity_t;


/**
  * @brief HAL TIM Output Channel Idle State.
  */
typedef enum
{
  /** Output Idle state: OCx=0/OCxN=0 when MOE=0        */
  HAL_TIM_OC_IDLE_STATE_RESET = LL_TIM_OCIDLESTATE_RESET,

  /** Output Idle state: OCx=1/OCxN=1 when MOE=0       */
  HAL_TIM_OC_IDLE_STATE_SET   = LL_TIM_OCIDLESTATE_SET,

} hal_tim_oc_idle_state_t;


/**
  * @brief HAL TIM Output Channel Override State.
  */
typedef enum
{
  /** Output Override: OCx=0/OCxN=0 when OOC=1       */
  HAL_TIM_OC_OVERRIDE_RESET = LL_TIM_OCOVERRIDE_RESET,

  /** Output Override: OCx=1/OCxN=1 when OOC=1      */
  HAL_TIM_OC_OVERRIDE_SET   = LL_TIM_OCOVERRIDE_SET,

} hal_tim_oc_override_state_t;


/**
  * @brief HAL TIM Output Channel Break Mode.
  */
typedef enum
{
  /** Output break mode: Immediate break                     */
  HAL_TIM_OC_BREAKMODE_IMMEDIATE = LL_TIM_OCBREAKMODE_IMMEDIATE,

  /** Output break mode: Delayed 1 break                     */
  HAL_TIM_OC_BREAKMODE_DELAY1    = LL_TIM_OCBREAKMODE_DELAY1,

  /** Output break mode: Delayed 2 break                     */
  HAL_TIM_OC_BREAKMODE_DELAY2    = LL_TIM_OCBREAKMODE_DELAY2,

} hal_tim_oc_break_mode_t;


/**
  * @brief HAL TIM Output Channel Override Status.
  */
typedef enum
{
  /** Output override is disabled  */
  HAL_TIM_OC_OVERRIDE_DISABLED = 0U,

  /** Output override is enabled   */
  HAL_TIM_OC_OVERRIDE_ENABLED  = 1U,

} hal_tim_oc_output_override_status_t;


/**
  * @brief HAL TIM Output Compare Preload Status.
  */
typedef enum
{
  /** Output Compare preload is disabled  */
  HAL_TIM_OC_COMPARE_PRELOAD_DISABLED = 0U,

  /** Output Compare preload is enabled   */
  HAL_TIM_OC_COMPARE_PRELOAD_ENABLED  = 1U,

} hal_tim_oc_compare_preload_status_t;

/**
  * @brief HAL TIM Output Channel Fast Mode Status.
  */
typedef enum
{
  /** Output Compare fast mode is disabled  */
  HAL_TIM_OC_COMPARE_FAST_MODE_DISABLED = 0U,

  /** Output Compare fast mode is enabled   */
  HAL_TIM_OC_COMPARE_FAST_MODE_ENABLED  = 1U,

} hal_tim_oc_compare_fast_mode_status_t;


/**
  * @brief HAL TIM Pulse generator prescaler.
  */
typedef enum
{
  /** Pulse prescaler: tPWG = tTIM_KER_CK            */
  HAL_TIM_PULSE_DIV1   = LL_TIM_PWPRSC_DIV1,

  /** Pulse prescaler 2: tPWG = 2*tTIM_KER_CK        */
  HAL_TIM_PULSE_DIV2   = LL_TIM_PWPRSC_DIV2,

  /** Pulse prescaler 4: tPWG = 4*tTIM_KER_CK        */
  HAL_TIM_PULSE_DIV4   = LL_TIM_PWPRSC_DIV4,

  /** Pulse prescaler 8: tPWG = 8*tTIM_KER_CK        */
  HAL_TIM_PULSE_DIV8   = LL_TIM_PWPRSC_DIV8,

  /** Pulse prescaler 16: tPWG = 16*tTIM_KER_CK      */
  HAL_TIM_PULSE_DIV16  = LL_TIM_PWPRSC_DIV16,

  /** Pulse prescaler 32: tPWG = 32*tTIM_KER_CK      */
  HAL_TIM_PULSE_DIV32  = LL_TIM_PWPRSC_DIV32,

  /** Pulse prescaler 64: tPWG = 64*tTIM_KER_CK      */
  HAL_TIM_PULSE_DIV64  = LL_TIM_PWPRSC_DIV64,

  /** Pulse prescaler 128: tPWG = 128*tTIM_KER_CK    */
  HAL_TIM_PULSE_DIV128 = LL_TIM_PWPRSC_DIV128,

} hal_tim_pulse_prescaler_t;


/**
  * @brief HAL TIM Dithering pattern.
  */
typedef enum
{
  /** 0 duty cycle and / or period change over 16 consecutive periods */
  HAL_TIM_DITHERING_0_16 = 0U,

  /** 1 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_1_16,

  /** 2 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_2_16,

  /** 3 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_3_16,

  /** 4 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_4_16,

  /** 5 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_5_16,

  /** 6 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_6_16,

  /** 7 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_7_16,

  /** 8 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_8_16,

  /** 9 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_9_16,

  /** 10 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_10_16,

  /** 11 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_11_16,

  /** 12 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_12_16,

  /** 13 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_13_16,

  /** 14 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_14_16,

  /** 15 duty cycle and / or period changes over 16 consecutive periods */
  HAL_TIM_DITHERING_15_16,

} hal_tim_dithering_pattern_t;

/**
  * @brief HAL TIM Dithering status.
  */
typedef enum
{
  /** Dithering is disabled      */
  HAL_TIM_DITHERING_DISABLED = 0U,

  /** Dithering is enabled       */
  HAL_TIM_DITHERING_ENABLED  = 1U,

} hal_tim_dithering_status_t;


/**
  * @brief HAL TIM Input Capture Unit.
  */
typedef enum
{
  /** Input capture unit 1 */
  HAL_TIM_IC_CAPTURE_UNIT_1 = LL_TIM_CHANNEL_CH1,

  /** Input capture unit 2 */
  HAL_TIM_IC_CAPTURE_UNIT_2 = LL_TIM_CHANNEL_CH2,

  /** Input capture unit 3 */
  HAL_TIM_IC_CAPTURE_UNIT_3 = LL_TIM_CHANNEL_CH3,

  /** Input capture unit 4 */
  HAL_TIM_IC_CAPTURE_UNIT_4 = LL_TIM_CHANNEL_CH4,

} hal_tim_ic_capture_unit_t;


/**
  * @brief HAL TIM Input Channel Polarity.
  */
typedef enum
{
  /** Rising edges are detected on input channel                  */
  HAL_TIM_IC_RISING         = LL_TIM_IC_POLARITY_RISING,

  /** Falling edges are detected on input channel                 */
  HAL_TIM_IC_FALLING        = LL_TIM_IC_POLARITY_FALLING,

  /** Both rising and falling edges are detected on input channel */
  HAL_TIM_IC_RISING_FALLING = LL_TIM_IC_POLARITY_RISING_FALLING,

} hal_tim_ic_polarity_t;


/**
  * @brief HAL TIM Input Capture Source.
  */
typedef enum
{
  /** TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2,
      IC3 or IC4, respectively                                          */
  HAL_TIM_IC_DIRECT = LL_TIM_ACTIVEINPUT_DIRECT,

  /** TIM Input 1, 2, 3 or 4 is selected to be connected to
      IC2, IC1, IC4 or IC3, respectively with trigger on rising edge    */
  HAL_TIM_IC_INDIRECT_RISING = LL_TIM_ACTIVEINPUT_INDIRECT | LL_TIM_IC_POLARITY_RISING,

  /** TIM Input 1, 2, 3 or 4 edge is selected to be connected to
      IC2, IC1, IC4 or IC3, respectively with trigger on falling edge   */
  HAL_TIM_IC_INDIRECT_FALLING = LL_TIM_ACTIVEINPUT_INDIRECT | LL_TIM_IC_POLARITY_FALLING,

  /** TIM Input 1, 2, 3 or 4 rising edge is selected to be connected to
      IC2, IC1, IC4 or IC3, respectively with trigger on both edges     */
  HAL_TIM_IC_INDIRECT_RISING_FALLING = LL_TIM_ACTIVEINPUT_INDIRECT | LL_TIM_IC_POLARITY_RISING_FALLING,

  /** TIM Input 1, 2, 3 or 4 is selected to be connected to TRC         */
  HAL_TIM_IC_TRC = LL_TIM_ACTIVEINPUT_TRC,

} hal_tim_ic_capture_unit_src_t;


/**
  * @brief HAL TIM Input Capture Unit Prescaler.
  */
typedef enum
{
  /** Capture performed each time an edge is detected on the capture input */
  HAL_TIM_IC_DIV1 = LL_TIM_ICPSC_DIV1,

  /** Capture performed once every 2 events                                */
  HAL_TIM_IC_DIV2 = LL_TIM_ICPSC_DIV2,

  /** Capture performed once every 4 events                                */
  HAL_TIM_IC_DIV4 = LL_TIM_ICPSC_DIV4,

  /** Capture performed once every 8 events                                */
  HAL_TIM_IC_DIV8 = LL_TIM_ICPSC_DIV8,

} hal_tim_ic_capture_unit_prescaler_t;


/**
  * @brief HAL TIM Input Channel XOR Gate Position.
  */
typedef enum
{
  /** XOR gate placed before TI1 filter                            */
  HAL_TIM_IC_XOR_GATE_POS_DIRECT   = LL_TIM_IC_XOR_GATE_POS_DIRECT,

  /** XOR gate placed after TI1, TI2 and TI3 filters,
      edge detector placed on XOR output                           */
  HAL_TIM_IC_XOR_GATE_POS_FILTERED = LL_TIM_IC_XOR_GATE_POS_FILTERED,

} hal_tim_ic_xor_gate_position_t;


/**
  * @brief HAL TIM Input Channel XOR Gate Input Signal Inversion status.
  */
typedef enum
{
  /** ICx is non-inverted on XOR gate input  */
  HAL_TIM_IC_XOR_GATE_INPUT_INV_DISABLED = 0U,

  /** ICx is inverted on XOR gate input      */
  HAL_TIM_IC_XOR_GATE_INPUT_INV_ENABLED  = 1U,

} hal_tim_ic_xor_gate_input_inversion_status_t;


/**
  * @brief HAL TIM Input Channel XOR Gate status.
  */
typedef enum
{
  /** XOR gate is disabled         */
  HAL_TIM_IC_XOR_GATE_DISABLED = 0U,

  /** XOR gate is enabled          */
  HAL_TIM_IC_XOR_GATE_ENABLED  = 1U,

} hal_tim_ic_xor_gate_status_t;


/**
  * @brief HAL TIM Input Channel Signal level.
  */
typedef enum
{
  /** Input channel signal level is low                */
  HAL_TIM_IC_CHANNEL_LEVEL_LOW  = LL_TIM_IC_SIGNAL_LOW,

  /** Input channel signal level is high               */
  HAL_TIM_IC_CHANNEL_LEVEL_HIGH = LL_TIM_IC_SIGNAL_HIGH,

} hal_tim_ic_channel_level_t;


/**
  * @brief TIM One-Pulse Mode status.
  */
typedef enum
{
  /** One-Pulse Mode is disabled      */
  HAL_TIM_ONE_PULSE_MODE_DISABLED = 0U,

  /** One-Pulse Mode is enabled       */
  HAL_TIM_ONE_PULSE_MODE_ENABLED  = 1U,

} hal_tim_one_pulse_mode_status_t;


/**
  * @brief TIM Encoder Index Direction (index_dir).
  */
typedef enum
{
  /** Index resets the counter whatever the direction  */
  HAL_TIM_ENCODER_INDEX_UP_DOWN = LL_TIM_INDEX_UP_DOWN,

  /** Index resets the counter when up-counting only   */
  HAL_TIM_ENCODER_INDEX_UP      = LL_TIM_INDEX_UP,

  /** Index resets the counter when down-counting only */
  HAL_TIM_ENCODER_INDEX_DOWN    = LL_TIM_INDEX_DOWN,

} hal_tim_encoder_index_dir_t;


/**
  * @brief TIM Encoder Index Blanking selection.
  */
typedef enum
{
  /** Index always active                                           */
  HAL_TIM_ENCODER_INDEX_BLANK_ALWAYS =  LL_TIM_INDEX_BLANK_ALWAYS,

  /** Index disabled when TI3 input is active, as per CC3P bitfield */
  HAL_TIM_ENCODER_INDEX_BLANK_TI3    =  LL_TIM_INDEX_BLANK_TI3,

  /** Index disabled when TI4 input is active, as per CC4P bitfield */
  HAL_TIM_ENCODER_INDEX_BLANK_TI4    =  LL_TIM_INDEX_BLANK_TI4,

} hal_tim_encoder_index_blank_mode_t;


/**
  * @brief TIM Encoder Index Positioning selection.
  */
typedef enum
{
  /** In quadrature encoder mode, the index event resets the counter
      when AB = 00                                                     */
  HAL_TIM_ENCODER_INDEX_POS_DOWN_DOWN = LL_TIM_INDEX_POSITION_DOWN_DOWN,

  /** In quadrature encoder mode, the index event resets the counter
      when AB = 01                                                     */
  HAL_TIM_ENCODER_INDEX_POS_DOWN_UP   = LL_TIM_INDEX_POSITION_DOWN_UP,

  /** In quadrature encoder mode, the index event resets the counter
      when AB = 10                                                     */
  HAL_TIM_ENCODER_INDEX_POS_UP_DOWN   = LL_TIM_INDEX_POSITION_UP_DOWN,

  /** In quadrature encoder mode, the index event resets the counter
      when AB = 11                                                     */
  HAL_TIM_ENCODER_INDEX_POS_UP_UP     = LL_TIM_INDEX_POSITION_UP_UP,

  /** In directional clock mode or clock plus direction mode, the index
      event resets the counter when clock is 0                         */
  HAL_TIM_ENCODER_INDEX_POS_DOWN      = LL_TIM_INDEX_POSITION_DOWN,

  /** In directional clock mode or clock plus direction mode, the index
      event resets the counter when clock is 1                         */
  HAL_TIM_ENCODER_INDEX_POS_UP        = LL_TIM_INDEX_POSITION_UP,

} hal_tim_encoder_index_pos_sel_t;


/**
  * @brief TIM Encoder Index selection.
  */
typedef enum
{
  /** Index is always active                                */
  HAL_TIM_ENCODER_INDEX_ALL = 0U,

  /** The first Index only resets the counter               */
  HAL_TIM_ENCODER_INDEX_FIRST_ONLY = LL_TIM_INDEX_FIRST_ONLY,

} hal_tim_encoder_index_sel_t;


/**
  * @brief TIM Encoder Index status.
  */
typedef enum
{
  /** Index input is disabled         */
  HAL_TIM_ENCODER_INDEX_DISABLED = 0U,

  /** Index input is enabled          */
  HAL_TIM_ENCODER_INDEX_ENABLED  = 1U,

} hal_tim_encoder_index_status_t;


/**
  * @brief HAL TIM External Trigger Polarity.
  */
typedef enum
{
  /** ETR input is active at high level or rising edge           */
  HAL_TIM_EXT_TRIG_NONINVERTED = LL_TIM_ETR_POLARITY_NONINVERTED,

  /** ETR input is active at low level or falling edge           */
  HAL_TIM_EXT_TRIG_INVERTED    = LL_TIM_ETR_POLARITY_INVERTED,

} hal_tim_ext_trig_polarity_t;


/**
  * @brief HAL TIM External Trigger Prescaler.
  */
typedef enum
{
  /** No prescaler is used                                                  */
  HAL_TIM_EXT_TRIG_DIV1 = LL_TIM_ETR_PRESCALER_DIV1,

  /** Prescaler for External Trigger: Capture performed once every 2 events */
  HAL_TIM_EXT_TRIG_DIV2 = LL_TIM_ETR_PRESCALER_DIV2,

  /** Prescaler for External Trigger: Capture performed once every 4 events */
  HAL_TIM_EXT_TRIG_DIV4 = LL_TIM_ETR_PRESCALER_DIV4,

  /** Prescaler for External Trigger: Capture performed once every 8 events */
  HAL_TIM_EXT_TRIG_DIV8 = LL_TIM_ETR_PRESCALER_DIV8,

} hal_tim_ext_trig_prescaler_t;


/**
  * @brief HAL TIM External Trigger Synchronous Prescaler.
  */
typedef enum
{
  /** No prescaler is used                                                  */
  HAL_TIM_EXT_TRIG_SYNC_DIV1 = LL_TIM_ETR_SYNC_PRESCALER_DIV1,

  /** Prescaler for External Trigger: Capture performed once every 2 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV2 = LL_TIM_ETR_SYNC_PRESCALER_DIV2,

  /** Prescaler for External Trigger: Capture performed once every 3 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV3 = LL_TIM_ETR_SYNC_PRESCALER_DIV3,

  /** Prescaler for External Trigger: Capture performed once every 4 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV4 = LL_TIM_ETR_SYNC_PRESCALER_DIV4,

  /** Prescaler for External Trigger: Capture performed once every 5 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV5 = LL_TIM_ETR_SYNC_PRESCALER_DIV5,

  /** Prescaler for External Trigger: Capture performed once every 6 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV6 = LL_TIM_ETR_SYNC_PRESCALER_DIV6,

  /** Prescaler for External Trigger: Capture performed once every 7 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV7 = LL_TIM_ETR_SYNC_PRESCALER_DIV7,

  /** Prescaler for External Trigger: Capture performed once every 8 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV8 = LL_TIM_ETR_SYNC_PRESCALER_DIV8,

  /** Prescaler for External Trigger: Capture performed once every 9 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV9 = LL_TIM_ETR_SYNC_PRESCALER_DIV9,

  /** Prescaler for External Trigger: Capture performed once every 10 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV10 = LL_TIM_ETR_SYNC_PRESCALER_DIV10,

  /** Prescaler for External Trigger: Capture performed once every 11 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV11 = LL_TIM_ETR_SYNC_PRESCALER_DIV11,

  /** Prescaler for External Trigger: Capture performed once every 12 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV12 = LL_TIM_ETR_SYNC_PRESCALER_DIV12,

  /** Prescaler for External Trigger: Capture performed once every 13 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV13 = LL_TIM_ETR_SYNC_PRESCALER_DIV13,

  /** Prescaler for External Trigger: Capture performed once every 14 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV14 = LL_TIM_ETR_SYNC_PRESCALER_DIV14,

  /** Prescaler for External Trigger: Capture performed once every 15 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV15 = LL_TIM_ETR_SYNC_PRESCALER_DIV15,

  /** Prescaler for External Trigger: Capture performed once every 16 events */
  HAL_TIM_EXT_TRIG_SYNC_DIV16 = LL_TIM_ETR_SYNC_PRESCALER_DIV16,

} hal_tim_ext_trig_sync_prescaler_t;


/**
  * @brief HAL TIM External Trigger Source.
  * @note  The pattern of the enum constants is: HAL_TIM_EXT_TRIG_TIMx_<source>
  */
typedef enum
{
  /** TIM1 external trigger is connected to TIM1_ETR */
  HAL_TIM_EXT_TRIG_TIM1_GPIO = LL_TIM_TIM1_ETR_IN_GPIO,

  /** TIM1 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM1_COMP1_OUT = LL_TIM_TIM1_ETR_IN_COMP1_OUT,

#if defined(COMP2)
  /** TIM1 external trigger is connected to comp2_out */
  HAL_TIM_EXT_TRIG_TIM1_COMP2_OUT = LL_TIM_TIM1_ETR_IN_COMP2_OUT,
#endif /* COMP2 */


  /** TIM1 external trigger is connected to adc1_awd1 */
  HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD1 = LL_TIM_TIM1_ETR_IN_ADC1_AWD1,

  /** TIM1 external trigger is connected to adc1_awd2 */
  HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD2 = LL_TIM_TIM1_ETR_IN_ADC1_AWD2,

  /** TIM1 external trigger is connected to adc1_awd3 */
  HAL_TIM_EXT_TRIG_TIM1_ADC1_AWD3 = LL_TIM_TIM1_ETR_IN_ADC1_AWD3,


  /** TIM2 external trigger is connected to TIM2_ETR */
  HAL_TIM_EXT_TRIG_TIM2_GPIO = LL_TIM_TIM2_ETR_IN_GPIO,

  /** TIM2 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM2_COMP1_OUT = LL_TIM_TIM2_ETR_IN_COMP1_OUT,

#if defined(COMP2)
  /** TIM2 external trigger is connected to comp2_out */
  HAL_TIM_EXT_TRIG_TIM2_COMP2_OUT = LL_TIM_TIM2_ETR_IN_COMP2_OUT,
#endif /* COMP2 */


  /** TIM2 external trigger is connected to adc1_awd1 */
  HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD1 = LL_TIM_TIM2_ETR_IN_ADC1_AWD1,

  /** TIM2 external trigger is connected to adc1_awd2 */
  HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD2 = LL_TIM_TIM2_ETR_IN_ADC1_AWD2,

  /** TIM2 external trigger is connected to adc1_awd3 */
  HAL_TIM_EXT_TRIG_TIM2_ADC1_AWD3 = LL_TIM_TIM2_ETR_IN_ADC1_AWD3,

  /** TIM2 external trigger is connected to LSE */
  HAL_TIM_EXT_TRIG_TIM2_LSE = LL_TIM_TIM2_ETR_IN_LSE,

  /** TIM2 external trigger is connected to MCO1 */
  HAL_TIM_EXT_TRIG_TIM2_MCO1 = LL_TIM_TIM2_ETR_IN_MCO1,

#if defined(TIM3)
  /** TIM2 external trigger is connected to TIM3_ETR */
  HAL_TIM_EXT_TRIG_TIM2_TIM3_ETR = LL_TIM_TIM2_ETR_IN_TIM3_ETR,
#endif /* TIM3 */

#if defined(TIM4)
  /** TIM2 external trigger is connected to TIM4_ETR */
  HAL_TIM_EXT_TRIG_TIM2_TIM4_ETR = LL_TIM_TIM2_ETR_IN_TIM4_ETR,
#endif /* TIM4 */

#if defined(TIM5)
  /** TIM2 external trigger is connected to TIM5_ETR */
  HAL_TIM_EXT_TRIG_TIM2_TIM5_ETR = LL_TIM_TIM2_ETR_IN_TIM5_ETR,
#endif /* TIM5 */

#if defined(ETH1)
  /** TIM2 external trigger is connected to eth1_ptp_pps_out */
  HAL_TIM_EXT_TRIG_TIM2_ETH1_PTP_PPS_OUT = LL_TIM_TIM2_ETR_IN_ETH1_PTP_PPS_OUT,
#endif /* ETH1 */


#if defined(TIM3)
  /** TIM3 external trigger is connected to TIM3_ETR */
  HAL_TIM_EXT_TRIG_TIM3_GPIO  = LL_TIM_TIM3_ETR_IN_GPIO,

  /** TIM3 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM3_COMP1_OUT = LL_TIM_TIM3_ETR_IN_COMP1_OUT,

  /** TIM3 external trigger is connected to TIM2_ETR */
  HAL_TIM_EXT_TRIG_TIM3_TIM2_ETR = LL_TIM_TIM3_ETR_IN_TIM2_ETR,

#if defined(TIM4)
  /** TIM3 external trigger is connected to TIM4_ETR */
  HAL_TIM_EXT_TRIG_TIM3_TIM4_ETR = LL_TIM_TIM3_ETR_IN_TIM4_ETR,
#endif /* TIM4 */

  /** TIM3 external trigger is connected to TIM5_ETR */
  HAL_TIM_EXT_TRIG_TIM3_TIM5_ETR = LL_TIM_TIM3_ETR_IN_TIM5_ETR,


#if defined(ETH1)
  /** TIM3 external trigger is connected to eth1_ptp_pps_out */
  HAL_TIM_EXT_TRIG_TIM3_ETH1_PTP_PPS_OUT = LL_TIM_TIM3_ETR_IN_ETH1_PTP_PPS_OUT,
#endif /* ETH1 */
#endif /* TIM3 */


#if defined(TIM4)
  /** TIM4 external trigger is connected to TIM4_ETR */
  HAL_TIM_EXT_TRIG_TIM4_GPIO  = LL_TIM_TIM4_ETR_IN_GPIO,

  /** TIM4 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM4_COMP1_OUT = LL_TIM_TIM4_ETR_IN_COMP1_OUT,

  /** TIM4 external trigger is connected to adc3_awd1 */
  HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD1 = LL_TIM_TIM4_ETR_IN_ADC3_AWD1,

  /** TIM4 external trigger is connected to adc3_awd2 */
  HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD2 = LL_TIM_TIM4_ETR_IN_ADC3_AWD2,

  /** TIM4 external trigger is connected to adc3_awd3 */
  HAL_TIM_EXT_TRIG_TIM4_ADC3_AWD3 = LL_TIM_TIM4_ETR_IN_ADC3_AWD3,

  /** TIM4 external trigger is connected to TIM2_ETR */
  HAL_TIM_EXT_TRIG_TIM4_TIM2_ETR = LL_TIM_TIM4_ETR_IN_TIM2_ETR,

  /** TIM4 external trigger is connected to TIM3_ETR */
  HAL_TIM_EXT_TRIG_TIM4_TIM3_ETR = LL_TIM_TIM4_ETR_IN_TIM3_ETR,

  /** TIM4 external trigger is connected to TIM5_ETR */
  HAL_TIM_EXT_TRIG_TIM4_TIM5_ETR = LL_TIM_TIM4_ETR_IN_TIM5_ETR,
#endif /* TIM4 */


#if defined(TIM5)
  /** TIM5 external trigger is connected to TIM5_ETR */
  HAL_TIM_EXT_TRIG_TIM5_GPIO  = LL_TIM_TIM5_ETR_IN_GPIO,

  /** TIM5 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM5_COMP1_OUT = LL_TIM_TIM5_ETR_IN_COMP1_OUT,

  /** TIM5 external trigger is connected to TIM2_ETR */
  HAL_TIM_EXT_TRIG_TIM5_TIM2_ETR = LL_TIM_TIM5_ETR_IN_TIM2_ETR,

#if defined(TIM3)
  /** TIM5 external trigger is connected to TIM3_ETR */
  HAL_TIM_EXT_TRIG_TIM5_TIM3_ETR = LL_TIM_TIM5_ETR_IN_TIM3_ETR,
#endif /* TIM3 */

#if defined(TIM4)
  /** TIM5 external trigger is connected to TIM4_ETR */
  HAL_TIM_EXT_TRIG_TIM5_TIM4_ETR = LL_TIM_TIM5_ETR_IN_TIM4_ETR,
#endif /* TIM4 */


#endif /* TIM5 */


  /** TIM8 external trigger is connected to TIM8_ETR */
  HAL_TIM_EXT_TRIG_TIM8_GPIO  = LL_TIM_TIM8_ETR_IN_GPIO,

  /** TIM8 external trigger is connected to comp1_out */
  HAL_TIM_EXT_TRIG_TIM8_COMP1_OUT = LL_TIM_TIM8_ETR_IN_COMP1_OUT,

#if defined(COMP2)
  /** TIM8 external trigger is connected to comp2_out */
  HAL_TIM_EXT_TRIG_TIM8_COMP2_OUT = LL_TIM_TIM8_ETR_IN_COMP2_OUT,
#endif /* COMP2 */


#if defined(ADC1) && defined(ADC2)
  /** TIM8 external trigger is connected to adc2_awd1 */
  HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD1 = LL_TIM_TIM8_ETR_IN_ADC2_AWD1,

  /** TIM8 external trigger is connected to adc2_awd2 */
  HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD2 = LL_TIM_TIM8_ETR_IN_ADC2_AWD2,

  /** TIM8 external trigger is connected to adc2_awd3 */
  HAL_TIM_EXT_TRIG_TIM8_ADC2_AWD3 = LL_TIM_TIM8_ETR_IN_ADC2_AWD3,

#elif defined(ADC1)
  /** TIM8 external trigger is connected to adc1_awd1 */
  HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD1 = LL_TIM_TIM8_ETR_IN_ADC1_AWD1,

  /** TIM8 external trigger is connected to adc1_awd2 */
  HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD2 = LL_TIM_TIM8_ETR_IN_ADC1_AWD2,

  /** TIM8 external trigger is connected to adc1_awd3 */
  HAL_TIM_EXT_TRIG_TIM8_ADC1_AWD3 = LL_TIM_TIM8_ETR_IN_ADC1_AWD3,
#endif /* ADC1 && ADC2 */

#if defined(ADC3)
  /** TIM8 external trigger is connected to adc3_awd1 */
  HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD1 = LL_TIM_TIM8_ETR_IN_ADC3_AWD1,

  /** TIM8 external trigger is connected to adc3_awd2 */
  HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD2 = LL_TIM_TIM8_ETR_IN_ADC3_AWD2,

  /** TIM8 external trigger is connected to adc3_awd3 */
  HAL_TIM_EXT_TRIG_TIM8_ADC3_AWD3 = LL_TIM_TIM8_ETR_IN_ADC3_AWD3,
#endif /* ADC3 */

} hal_tim_ext_trig_src_t;


/**
  * @brief HAL TIM Input sources.
  * @note  The pattern of the enum constants is: HAL_TIM_INPUT_TIMx_TIy_<source>
  */
typedef enum
{
  /* TIM1 */
  /** TIM1 TI1 is connected to TIM1_CH1 */
  HAL_TIM_INPUT_TIM1_TI1_GPIO = LL_TIM_TIM1_TI1_GPIO,

  /** TIM1 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM1_TI1_COMP1_OUT = LL_TIM_TIM1_TI1_COMP1_OUT,

#if defined(COMP2)
  /** TIM1 TI1 is connected to comp2_out */
  HAL_TIM_INPUT_TIM1_TI1_COMP2_OUT = LL_TIM_TIM1_TI1_COMP2_OUT,
#endif /* COMP2 */

  /** TIM1 TI2 is connected to TIM1_CH2 */
  HAL_TIM_INPUT_TIM1_TI2_GPIO = LL_TIM_TIM1_TI2_GPIO,

  /** TIM1 TI3 is connected to TIM1_CH3 */
  HAL_TIM_INPUT_TIM1_TI3_GPIO = LL_TIM_TIM1_TI3_GPIO,

  /** TIM1 TI4 is connected to TIM1_CH4 */
  HAL_TIM_INPUT_TIM1_TI4_GPIO = LL_TIM_TIM1_TI4_GPIO,

  /* TIM2 */
  /** TIM2 TI1 is connected to TIM2_CH1 */
  HAL_TIM_INPUT_TIM2_TI1_GPIO = LL_TIM_TIM2_TI1_GPIO,

  /** TIM2 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM2_TI1_COMP1_OUT = LL_TIM_TIM2_TI1_COMP1_OUT,

#if defined(COMP2)
  /** TIM2 TI1 is connected to comp2_out */
  HAL_TIM_INPUT_TIM2_TI1_COMP2_OUT = LL_TIM_TIM2_TI1_COMP2_OUT,
#endif /* COMP2 */

#if defined(ETH1)
  /** TIM2 TI1 is connected to eth1_ptp_pps_out */
  HAL_TIM_INPUT_TIM2_TI1_ETH1_PTP_PPS_OUT = LL_TIM_TIM2_TI1_ETH1_PTP_PPS_OUT,
#endif /* ETH1 */

  /** TIM2 TI1 is connected to LSI */
  HAL_TIM_INPUT_TIM2_TI1_LSI = LL_TIM_TIM2_TI1_LSI,

  /** TIM2 TI1 is connected to LSE */
  HAL_TIM_INPUT_TIM2_TI1_LSE = LL_TIM_TIM2_TI1_LSE,

  /** TIM2 TI1 is connected to rtc_wut_trg */
  HAL_TIM_INPUT_TIM2_TI1_RTC_WUT_TRG = LL_TIM_TIM2_TI1_RTC_WUT_TRG,

#if defined(TIM5)
  /** TIM2 TI1 is connected to TIM5_CH1 */
  HAL_TIM_INPUT_TIM2_TI1_TIM5_CH1 = LL_TIM_TIM2_TI1_TIM5_CH1,
#endif /* TIM5 */

#if defined(FDCAN1)
  /** TIM2 TI1 is connected to fdcan1_rxeof_evt */
  HAL_TIM_INPUT_TIM2_TI1_FDCAN1_RXEOF_EVT = LL_TIM_TIM2_TI1_FDCAN1_RXEOF_EVT,
#endif /* FDCAN1 */

  /** TIM2 TI2 is connected to TIM2_CH2 */
  HAL_TIM_INPUT_TIM2_TI2_GPIO = LL_TIM_TIM2_TI2_GPIO,

  /** TIM2 TI2 is connected to hse_1M_ck */
  HAL_TIM_INPUT_TIM2_TI2_HSE_RTC = LL_TIM_TIM2_TI2_HSE_RTC,

  /** TIM2 TI2 is connected to MCO1 */
  HAL_TIM_INPUT_TIM2_TI2_MCO1 = LL_TIM_TIM2_TI2_MCO1,

  /** TIM2 TI2 is connected to MCO2 */
  HAL_TIM_INPUT_TIM2_TI2_MCO2 = LL_TIM_TIM2_TI2_MCO2,

#if defined(FDCAN1)
  /** TIM2 TI2 is connected to fdcan1_txeof_evt */
  HAL_TIM_INPUT_TIM2_TI2_FDCAN1_TXEOF_EVT = LL_TIM_TIM2_TI2_FDCAN1_TXEOF_EVT,
#endif /* FDCAN1 */

  /** TIM2 TI3 is connected to TIM2_CH3 */
  HAL_TIM_INPUT_TIM2_TI3_GPIO = LL_TIM_TIM2_TI3_GPIO,

#if defined(FDCAN2)
  /** TIM2 TI3 is connected to fdcan2_rxeof_evt */
  HAL_TIM_INPUT_TIM2_TI3_FDCAN2_RXEOF_EVT = LL_TIM_TIM2_TI3_FDCAN2_RXEOF_EVT,
#endif /* FDCAN2 */

  /** TIM2 TI4 is connected to TIM2_CH4 */
  HAL_TIM_INPUT_TIM2_TI4_GPIO = LL_TIM_TIM2_TI4_GPIO,

  /** TIM2 TI4 is connected to comp1_out */
  HAL_TIM_INPUT_TIM2_TI4_COMP1_OUT = LL_TIM_TIM2_TI4_COMP1_OUT,

#if defined(COMP2)
  /** TIM2 TI4 is connected to comp2_out */
  HAL_TIM_INPUT_TIM2_TI4_COMP2_OUT = LL_TIM_TIM2_TI4_COMP2_OUT,
#endif /* COMP2 */

#if defined(FDCAN2)
  /** TIM2 TI4 is connected to fdcan2_txeof_evt */
  HAL_TIM_INPUT_TIM2_TI4_FDCAN2_TXEOF_EVT = LL_TIM_TIM2_TI4_FDCAN2_TXEOF_EVT,
#endif /* FDCAN2 */

#if defined(TIM3)
  /* TIM3 */
  /** TIM3 TI1 is connected to TIM3_CH1 */
  HAL_TIM_INPUT_TIM3_TI1_GPIO = LL_TIM_TIM3_TI1_GPIO,

  /** TIM3 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM3_TI1_COMP1_OUT = LL_TIM_TIM3_TI1_COMP1_OUT,

#if defined(FDCAN2)
  /** TIM3 TI1 is connected to eth1_ptp_pps_out */
  HAL_TIM_INPUT_TIM3_TI1_ETH1_PTP_PPS_OUT = LL_TIM_TIM3_TI1_ETH1_PTP_PPS_OUT,

  /** TIM3 TI1 is connected to fdcan2_rxeof_evt */
  HAL_TIM_INPUT_TIM3_TI1_FDCAN2_RXEOF_EVT = LL_TIM_TIM3_TI1_FDCAN2_RXEOF_EVT,
#endif /* FDCAN2 */

  /** TIM3 TI2 is connected to TIM3_CH2 */
  HAL_TIM_INPUT_TIM3_TI2_GPIO = LL_TIM_TIM3_TI2_GPIO,

#if defined(FDCAN2)
  /** TIM3 TI2 is connected to fdcan2_txeof_evt */
  HAL_TIM_INPUT_TIM3_TI2_FDCAN2_TXEOF_EVT = LL_TIM_TIM3_TI2_FDCAN2_TXEOF_EVT,
#endif /* FDCAN2 */

  /** TIM3 TI3 is connected to TIM3_CH3 */
  HAL_TIM_INPUT_TIM3_TI3_GPIO = LL_TIM_TIM3_TI3_GPIO,

  /** TIM3 TI4 is connected to TIM3_CH4 */
  HAL_TIM_INPUT_TIM3_TI4_GPIO = LL_TIM_TIM3_TI4_GPIO,
#endif /* TIM3 */

#if defined(TIM4)
  /* TIM4 */
  /** TIM4 TI1 is connected to TIM4_CH1 */
  HAL_TIM_INPUT_TIM4_TI1_GPIO = LL_TIM_TIM4_TI1_GPIO,

  /** TIM4 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM4_TI1_COMP1_OUT = LL_TIM_TIM4_TI1_COMP1_OUT,

  /** TIM4 TI2 is connected to TIM4_CH2 */
  HAL_TIM_INPUT_TIM4_TI2_GPIO = LL_TIM_TIM4_TI2_GPIO,

  /** TIM4 TI3 is connected to TIM4_CH3 */
  HAL_TIM_INPUT_TIM4_TI3_GPIO = LL_TIM_TIM4_TI3_GPIO,

  /** TIM4 TI4 is connected to TIM4_CH4 */
  HAL_TIM_INPUT_TIM4_TI4_GPIO = LL_TIM_TIM4_TI4_GPIO,
#endif /* TIM4 */

#if defined(TIM5)
  /* TIM5 */
  /** TIM5 TI1 is connected to TIM5_CH1 */
  HAL_TIM_INPUT_TIM5_TI1_GPIO = LL_TIM_TIM5_TI1_GPIO,

  /** TIM5 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM5_TI1_COMP1_OUT = LL_TIM_TIM5_TI1_COMP1_OUT,

  /** TIM5 TI2 is connected to TIM5_CH2 */
  HAL_TIM_INPUT_TIM5_TI2_GPIO = LL_TIM_TIM5_TI2_GPIO,

  /** TIM5 TI3 is connected to TIM5_CH3 */
  HAL_TIM_INPUT_TIM5_TI3_GPIO = LL_TIM_TIM5_TI3_GPIO,

  /** TIM5 TI4 is connected to TIM5_CH4 */
  HAL_TIM_INPUT_TIM5_TI4_GPIO = LL_TIM_TIM5_TI4_GPIO,
#endif /* TIM5 */

  /* TIM8 */
  /** TIM8 TI1 is connected to TIM8_CH1 */
  HAL_TIM_INPUT_TIM8_TI1_GPIO = LL_TIM_TIM8_TI1_GPIO,

  /** TIM8 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM8_TI1_COMP1_OUT = LL_TIM_TIM8_TI1_COMP1_OUT,

#if defined(COMP2)
  /** TIM8 TI1 is connected to comp2_out */
  HAL_TIM_INPUT_TIM8_TI1_COMP2_OUT = LL_TIM_TIM8_TI1_COMP2_OUT,
#endif /* COMP2 */

  /** TIM8 TI2 is connected to TIM8_CH2 */
  HAL_TIM_INPUT_TIM8_TI2_GPIO = LL_TIM_TIM8_TI2_GPIO,

  /** TIM8 TI3 is connected to TIM8_CH3 */
  HAL_TIM_INPUT_TIM8_TI3_GPIO = LL_TIM_TIM8_TI3_GPIO,

  /** TIM8 TI4 is connected to TIM8_CH4 */
  HAL_TIM_INPUT_TIM8_TI4_GPIO = LL_TIM_TIM8_TI4_GPIO,

  /* TIM12 */
  /** TIM12 TI1 is connected to TIM12_CH1 */
  HAL_TIM_INPUT_TIM12_TI1_GPIO = LL_TIM_TIM12_TI1_GPIO,

  /** TIM12 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM12_TI1_COMP1_OUT = LL_TIM_TIM12_TI1_COMP1_OUT,

#if defined(COMP2)
  /** TIM12 TI1 is connected to comp2_out */
  HAL_TIM_INPUT_TIM12_TI1_COMP2_OUT = LL_TIM_TIM12_TI1_COMP2_OUT,
#endif /* COMP2 */

  /** TIM12 TI1 is connected to MCO1 */
  HAL_TIM_INPUT_TIM12_TI1_MCO1 = LL_TIM_TIM12_TI1_MCO1,

  /** TIM12 TI1 is connected to MCO2 */
  HAL_TIM_INPUT_TIM12_TI1_MCO2 = LL_TIM_TIM12_TI1_MCO2,

  /** TIM12 TI1 is connected to hse_1M_ck */
  HAL_TIM_INPUT_TIM12_TI1_HSE_RTC = LL_TIM_TIM12_TI1_HSE_RTC,

  /** TIM12 TI1 is connected to i3c1_ibi_ack */
  HAL_TIM_INPUT_TIM12_TI1_I3C1_IBI_ACK = LL_TIM_TIM12_TI1_I3C1_IBI_ACK,

  /** TIM12 TI2 is connected to TIM12_CH2 */
  HAL_TIM_INPUT_TIM12_TI2_GPIO = LL_TIM_TIM12_TI2_GPIO,

  /* TIM15 */
  /** TIM15 TI1 is connected to TIM15_CH1 */
  HAL_TIM_INPUT_TIM15_TI1_GPIO = LL_TIM_TIM15_TI1_GPIO,

  /** TIM15 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM15_TI1_COMP1_OUT = LL_TIM_TIM15_TI1_COMP1_OUT,

#if defined(COMP2)
  /** TIM15 TI1 is connected to comp2_out */
  HAL_TIM_INPUT_TIM15_TI1_COMP2_OUT = LL_TIM_TIM15_TI1_COMP2_OUT,
#endif /* COMP2 */

  /** TIM15 TI1 is connected to LSE */
  HAL_TIM_INPUT_TIM15_TI1_LSE = LL_TIM_TIM15_TI1_LSE,

#if defined(FDCAN2)
  /** TIM15 TI1 is connected to fdcan2_rxeof_evt */
  HAL_TIM_INPUT_TIM15_TI1_FDCAN2_RXEOF_EVT = LL_TIM_TIM15_TI1_FDCAN2_RXEOF_EVT,
#endif /* FDCAN2 */

  /** TIM15 TI2 is connected to TIM15_CH2 */
  HAL_TIM_INPUT_TIM15_TI2_GPIO = LL_TIM_TIM15_TI2_GPIO,

#if defined(FDCAN2)
  /** TIM15 TI2 is connected to fdcan2_txeof_evt */
  HAL_TIM_INPUT_TIM15_TI2_FDCAN2_TXEOF_EVT = LL_TIM_TIM15_TI2_FDCAN2_TXEOF_EVT,
#endif /* FDCAN2 */

#if defined(TIM16)
  /* TIM16 */
  /** TIM16 TI1 is connected to TIM16_CH1 */
  HAL_TIM_INPUT_TIM16_TI1_GPIO = LL_TIM_TIM16_TI1_GPIO,

  /** TIM16 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM16_TI1_COMP1_OUT = LL_TIM_TIM16_TI1_COMP1_OUT,

  /** TIM16 TI1 is connected to LSI */
  HAL_TIM_INPUT_TIM16_TI1_LSI = LL_TIM_TIM16_TI1_LSI,

  /** TIM16 TI1 is connected to LSE */
  HAL_TIM_INPUT_TIM16_TI1_LSE = LL_TIM_TIM16_TI1_LSE,

  /** TIM16 TI1 is connected to rtc_wut_trg */
  HAL_TIM_INPUT_TIM16_TI1_RTC_WUT_TRG = LL_TIM_TIM16_TI1_RTC_WUT_TRG,

  /** TIM16 TI1 is connected to MCO1 */
  HAL_TIM_INPUT_TIM16_TI1_MCO1 = LL_TIM_TIM16_TI1_MCO1,

  /** TIM16 TI1 is connected to MCO2 */
  HAL_TIM_INPUT_TIM16_TI1_MCO2 = LL_TIM_TIM16_TI1_MCO2,


  /* TIM17 */
  /** TIM17 TI1 is connected to TIM17_CH1 */
  HAL_TIM_INPUT_TIM17_TI1_GPIO = LL_TIM_TIM17_TI1_GPIO,

  /** TIM17 TI1 is connected to comp1_out */
  HAL_TIM_INPUT_TIM17_TI1_COMP1_OUT = LL_TIM_TIM17_TI1_COMP1_OUT,

  /** TIM17 TI1 is connected to hse_1M_ck */
  HAL_TIM_INPUT_TIM17_TI1_HSE_RTC = LL_TIM_TIM17_TI1_HSE_RTC,

  /** TIM17 TI1 is connected to MCO1 */
  HAL_TIM_INPUT_TIM17_TI1_MCO1 = LL_TIM_TIM17_TI1_MCO1,

  /** TIM17 TI1 is connected to MCO2 */
  HAL_TIM_INPUT_TIM17_TI1_MCO2 = LL_TIM_TIM17_TI1_MCO2,

  /** TIM17 TI1 is connected to i3c1_ibi_ack */
  HAL_TIM_INPUT_TIM17_TI1_I3C1_IBI_ACK = LL_TIM_TIM17_TI1_I3C1_IBI_ACK,
#endif /* TIM16 */

} hal_tim_channel_src_t;


/**
  * @brief HAL TIM Slave mode.
  */
typedef enum
{
  /** Slave mode disabled                                                         */
  HAL_TIM_SLAVE_DISABLED = LL_TIM_SLAVEMODE_DISABLED,

  /** Reset Mode
    *  Rising edge of the selected trigger input (TRGI) reinitializes the counter */
  HAL_TIM_SLAVE_RESET   = LL_TIM_SLAVEMODE_RESET,

  /** Gated Mode
    *  The counter clock is enabled when the trigger input (TRGI) is high         */
  HAL_TIM_SLAVE_GATED   = LL_TIM_SLAVEMODE_GATED,

  /** Trigger Mode
    *  The counter starts at a rising edge of the trigger TRGI                    */
  HAL_TIM_SLAVE_TRIGGER = LL_TIM_SLAVEMODE_TRIGGER,

  /** Combined reset + trigger mode
    *  Rising edge of the selected trigger input (TRGI) reinitializes the
    *  counter, generates an update of the registers and starts the counter       */
  HAL_TIM_SLAVE_COMBINED_RESET_TRIGGER = LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER,

  /** Combined gated + reset mode
    *  The counter clock is enabled when the trigger input (TRGI) is high.
    *  The counter stops and is reset as soon as the trigger becomes low.
    *  Both start and stop of the counter are controlled.                         */
  HAL_TIM_SLAVE_COMBINED_GATED_RESET   = LL_TIM_SLAVEMODE_COMBINED_GATED_RESET,

} hal_tim_slave_mode_t;


/**
  * @brief HAL TIM Master Mode Selection of Trigger Output source.
  */
typedef enum
{
  /** TIMx_EGR.UG bit is used as trigger output (TRGO)               */
  HAL_TIM_TRGO_RESET = LL_TIM_TRGO_RESET,

  /** TIMx_CR1.CEN bit is used as trigger output (TRGO)              */
  HAL_TIM_TRGO_ENABLE = LL_TIM_TRGO_ENABLE,

  /** Update event is used as trigger output (TRGO)                  */
  HAL_TIM_TRGO_UPDATE = LL_TIM_TRGO_UPDATE,

  /** Capture or a compare match 1 is used as trigger output (TRGO)  */
  HAL_TIM_TRGO_CC1IF = LL_TIM_TRGO_CC1IF,

  /** OC1REFC signal is used as trigger output (TRGO)                */
  HAL_TIM_TRGO_OC1 = LL_TIM_TRGO_OC1,

  /** OC2REFC signal is used as trigger output (TRGO)                */
  HAL_TIM_TRGO_OC2 = LL_TIM_TRGO_OC2,

  /** OC3REFC signal is used as trigger output (TRGO)                */
  HAL_TIM_TRGO_OC3 = LL_TIM_TRGO_OC3,

  /** OC4REFC signal is used as trigger output (TRGO)                */
  HAL_TIM_TRGO_OC4 = LL_TIM_TRGO_OC4,

  /** Encoder clock is used as trigger output (TRGO)                 */
  HAL_TIM_TRGO_ENCODER_CLK = LL_TIM_TRGO_ENCODER_CLK,

} hal_tim_trigger_output_source_t;


/**
  * @brief HAL TIM Master Mode Selection of Trigger Output 2 source.
  */
typedef enum
{
  /** TIMx_EGR.UG bit is used as trigger output (TRGO2)               */
  HAL_TIM_TRGO2_RESET = LL_TIM_TRGO2_RESET,

  /** TIMx_CR1.CEN bit is used as trigger output (TRGO2)              */
  HAL_TIM_TRGO2_ENABLE = LL_TIM_TRGO2_ENABLE,

  /** Update event is used as trigger output (TRGO2)                  */
  HAL_TIM_TRGO2_UPDATE = LL_TIM_TRGO2_UPDATE,

  /** Capture or a compare match 1 is used as trigger output (TRGO2)  */
  HAL_TIM_TRGO2_CC1F = LL_TIM_TRGO2_CC1F,

  /** OC1REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC1 = LL_TIM_TRGO2_OC1,

  /** OC2REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC2 = LL_TIM_TRGO2_OC2,

  /** OC3REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC3 = LL_TIM_TRGO2_OC3,

  /** OC4REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC4 = LL_TIM_TRGO2_OC4,

  /** OC5REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC5 = LL_TIM_TRGO2_OC5,

  /** OC6REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC6 = LL_TIM_TRGO2_OC6,

  /** OC7REFC signal is used as trigger output (TRGO2)                 */
  HAL_TIM_TRGO2_OC7 = LL_TIM_TRGO2_OC7,

  /** OC4REFC rising or falling edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC4_RISING_FALLING = LL_TIM_TRGO2_OC4_RISING_FALLING,

  /** OC6REFC rising or falling edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC6_RISING_FALLING = LL_TIM_TRGO2_OC6_RISING_FALLING,

  /** OC7REFC rising or falling edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC7_RISING_FALLING = LL_TIM_TRGO2_OC7_RISING_FALLING,

  /** OC4REFC or OC6REFC rising edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC4_RISING_OC6_RISING = LL_TIM_TRGO2_OC4_RISING_OC6_RISING,

  /** OC4REFC or OC7REFC rising edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC4_RISING_OC7_RISING = LL_TIM_TRGO2_OC4_RISING_OC7_RISING,

  /** OC5REFC or OC6REFC rising edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC5_RISING_OC6_RISING = LL_TIM_TRGO2_OC5_RISING_OC6_RISING,

  /** OC5REFC or OC7REFC rising edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC5_RISING_OC7_RISING = LL_TIM_TRGO2_OC5_RISING_OC7_RISING,

  /** OC6REFC or OC7REFC rising edges generate pulses on TRGO2         */
  HAL_TIM_TRGO2_OC6_RISING_OC7_RISING = LL_TIM_TRGO2_OC6_RISING_OC7_RISING,

  /** OC4REFC rising or OC6REFC falling edges generate pulses on TRGO2 */
  HAL_TIM_TRGO2_OC4_RISING_OC6_FALLING = LL_TIM_TRGO2_OC4_RISING_OC6_FALLING,

  /** OC4REFC rising or OC7REFC falling edges generate pulses on TRGO2 */
  HAL_TIM_TRGO2_OC4_RISING_OC7_FALLING = LL_TIM_TRGO2_OC4_RISING_OC7_FALLING,

  /** OC5REFC rising or OC6REFC falling edges generate pulses on TRGO2 */
  HAL_TIM_TRGO2_OC5_RISING_OC6_FALLING = LL_TIM_TRGO2_OC5_RISING_OC6_FALLING,

  /** OC5REFC rising or OC7REFC falling edges generate pulses on TRGO2 */
  HAL_TIM_TRGO2_OC5_RISING_OC7_FALLING = LL_TIM_TRGO2_OC5_RISING_OC7_FALLING,

  /** OC6REFC rising or OC7REFC falling edges generate pulses on TRGO2 */
  HAL_TIM_TRGO2_OC6_RISING_OC7_FALLING = LL_TIM_TRGO2_OC6_RISING_OC7_FALLING,

} hal_tim_trigger_output2_source_t;


/**
  * @brief HAL TIM Slave Mode Preload Status.
  */
typedef enum
{
  /** Slave mode selection (SMS[3:0]) isn't preloaded */
  HAL_TIM_SLAVE_MODE_PRELOAD_DISABLED = 0U,

  /** Slave mode selection (SMS[3:0]) is preloaded    */
  HAL_TIM_SLAVE_MODE_PRELOAD_ENABLED  = 1U,

} hal_tim_slave_mode_preload_status_t;


/**
  * @brief Slave Mode Preload Source.
  */
typedef enum
{
  /** The transfer is triggered by the timer's Update event           */
  HAL_TIM_SLAVE_MODE_PRELOAD_UPDATE = LL_TIM_SLAVE_MODE_PRELOAD_UPDATE,

  /** The transfer is triggered by the Index event                    */
  HAL_TIM_SLAVE_MODE_PRELOAD_INDEX  = LL_TIM_SLAVE_MODE_PRELOAD_INDEX,

} hal_tim_slave_mode_preload_src_t;


/**
  * @brief TIM Master/Slave Mode.
  */
typedef enum
{
  /** No action */
  HAL_TIM_MASTER_SLAVE_MODE_DISABLED = 0x00000000U,

  /** Master/Slave mode is selected */
  HAL_TIM_MASTER_SLAVE_MODE_ENABLED  = 0x00000001U,

} hal_tim_master_slave_mode_status_t;


/**
  * @brief HAL TIM ADC Synchronization Status.
  */
typedef enum
{
  /** The timer operates independently from the ADC   */
  HAL_TIM_ADC_SYNCHRONIZATION_DISABLED = 0U,

  /** The timer operation is synchronized with the
    * ADC clock to provide jitter-free sampling point */
  HAL_TIM_ADC_SYNCHRONIZATION_ENABLED  = 1U,

} hal_tim_adc_synchronization_status_t;


/**
  * @brief  HAL TIM Output Compare Clear (ocrefclr) Status.
  */
typedef enum
{
  HAL_TIM_OCREFCLEAR_DISABLED = 0U,

  HAL_TIM_OCREFCLEAR_ENABLED  = 1U,

} hal_tim_ocref_clr_status_t;


/**
  * @brief HAL TIM Output Compare Clear (ocrefclr) Sources.
  * @note  The pattern of the enum constants is: HAL_TIM_OCREF_CLR_TIMx_<source>
  */
typedef enum
{
  /** TIM1 OCREF clear input is connected to tim1_etrf */
  HAL_TIM_OCREF_CLR_TIM1_ETR = LL_TIM_TIM1_OCREF_CLR_INT_ETR,

  /** TIM1 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM1_COMP1_OUT = LL_TIM_TIM1_OCREF_CLR_INT_COMP1_OUT,

#if defined(COMP2)
  /** TIM1 OCREF clear input is connected to comp2_out */
  HAL_TIM_OCREF_CLR_TIM1_COMP2_OUT = LL_TIM_TIM1_OCREF_CLR_INT_COMP2_OUT,
#endif /* COMP2 */


  /** TIM2 OCREF clear input is connected to tim2_etrf */
  HAL_TIM_OCREF_CLR_TIM2_ETR = LL_TIM_TIM2_OCREF_CLR_INT_ETR,

  /** TIM2 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM2_COMP1_OUT = LL_TIM_TIM2_OCREF_CLR_INT_COMP1_OUT,

#if defined(COMP2)
  /** TIM2 OCREF clear input is connected to comp2_out */
  HAL_TIM_OCREF_CLR_TIM2_COMP2_OUT = LL_TIM_TIM2_OCREF_CLR_INT_COMP2_OUT,
#endif /* COMP2 */


#if defined(TIM3)
  /** TIM3 OCREF clear input is connected to tim3_etrf */
  HAL_TIM_OCREF_CLR_TIM3_ETR = LL_TIM_TIM3_OCREF_CLR_INT_ETR,

  /** TIM3 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM3_COMP1_OUT = LL_TIM_TIM3_OCREF_CLR_INT_COMP1_OUT,
#endif /* TIM3 */


#if defined(TIM4)
  /** TIM4 OCREF clear input is connected to tim4_etrf */
  HAL_TIM_OCREF_CLR_TIM4_ETR = LL_TIM_TIM4_OCREF_CLR_INT_ETR,

  /** TIM4 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM4_COMP1_OUT = LL_TIM_TIM4_OCREF_CLR_INT_COMP1_OUT,
#endif /* TIM4 */


#if defined(TIM5)
  /** TIM5 OCREF clear input is connected to tim5_etrf */
  HAL_TIM_OCREF_CLR_TIM5_ETR = LL_TIM_TIM5_OCREF_CLR_INT_ETR,

  /** TIM5 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM5_COMP1_OUT = LL_TIM_TIM5_OCREF_CLR_INT_COMP1_OUT,
#endif /* TIM5 */


  /** TIM8 OCREF clear input is connected to tim8_etrf */
  HAL_TIM_OCREF_CLR_TIM8_ETR = LL_TIM_TIM8_OCREF_CLR_INT_ETR,

  /** TIM8 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM8_COMP1_OUT = LL_TIM_TIM8_OCREF_CLR_INT_COMP1_OUT,

#if defined(COMP2)
  /** TIM8 OCREF clear input is connected to comp2_out */
  HAL_TIM_OCREF_CLR_TIM8_COMP2_OUT = LL_TIM_TIM8_OCREF_CLR_INT_COMP2_OUT,
#endif /* COMP2 */


  /** TIM15 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM15_COMP1_OUT = LL_TIM_TIM15_OCREF_CLR_INT_COMP1_OUT,

#if defined(COMP2)
  /** TIM15 OCREF clear input is connected to comp2_out */
  HAL_TIM_OCREF_CLR_TIM15_COMP2_OUT = LL_TIM_TIM15_OCREF_CLR_INT_COMP2_OUT,
#endif /* COMP2 */


#if defined(TIM16)
  /** TIM16 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM16_COMP1_OUT = LL_TIM_TIM16_OCREF_CLR_INT_COMP1_OUT,


  /** TIM17 OCREF clear input is connected to comp1_out */
  HAL_TIM_OCREF_CLR_TIM17_COMP1_OUT = LL_TIM_TIM17_OCREF_CLR_INT_COMP1_OUT,
#endif /* TIM16 */

} hal_tim_ocref_clr_src_t;


#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  *  @brief TIM DMA Handle Index
  */
typedef enum
{
  /** Index of the DMA handle used for Update DMA requests            */
  HAL_TIM_DMA_ID_UPD  = 0U,

  /** Index of the DMA handle used for Capture/Compare 1 DMA requests */
  HAL_TIM_DMA_ID_CC1  = 1U,

  /** Index of the DMA handle used for Capture/Compare 2 DMA requests */
  HAL_TIM_DMA_ID_CC2  = 2U,

  /** Index of the DMA handle used for Capture/Compare 3 DMA requests */
  HAL_TIM_DMA_ID_CC3  = 3U,

  /** Index of the DMA handle used for Capture/Compare 4 DMA requests */
  HAL_TIM_DMA_ID_CC4  = 4U,

  /** Index of the DMA handle used for Commutation DMA requests       */
  HAL_TIM_DMA_ID_COM  = 5U,

  /** Index of the DMA handle used for Trigger DMA requests           */
  HAL_TIM_DMA_ID_TRGI = 6U,

} hal_tim_dma_index_t;


/**
  * @brief  HAL TIM DMA Burst Base Address.
  */
typedef enum
{
  /** TIMx_CR1 register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_CR1   = LL_TIM_DMABURST_BASEADDR_CR1,

  /** TIMx_CR2 register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_CR2   = LL_TIM_DMABURST_BASEADDR_CR2,

  /** TIMx_SMCR register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_SMCR  = LL_TIM_DMABURST_BASEADDR_SMCR,

  /** TIMx_DIER register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_DIER  = LL_TIM_DMABURST_BASEADDR_DIER,

  /** TIMx_SR register is the DMA base address for DMA burst          */
  HAL_TIM_DMABURST_BASE_ADDR_SR    = LL_TIM_DMABURST_BASEADDR_SR,

  /** TIMx_EGR register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_EGR   = LL_TIM_DMABURST_BASEADDR_EGR,

  /** TIMx_CCMR1 register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_CCMR1 = LL_TIM_DMABURST_BASEADDR_CCMR1,

  /** TIMx_CCMR2 register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_CCMR2 = LL_TIM_DMABURST_BASEADDR_CCMR2,

  /** TIMx_CCER register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCER  = LL_TIM_DMABURST_BASEADDR_CCER,

  /** TIMx_CNT register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CNT   = LL_TIM_DMABURST_BASEADDR_CNT,

  /** TIMx_PSC register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_PSC   = LL_TIM_DMABURST_BASEADDR_PSC,

  /** TIMx_ARR register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_ARR   = LL_TIM_DMABURST_BASEADDR_ARR,

  /** TIMx_RCR register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_RCR   = LL_TIM_DMABURST_BASEADDR_RCR,

  /** TIMx_CCR1 register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_CCR1  = LL_TIM_DMABURST_BASEADDR_CCR1,

  /** TIMx_CCR2 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR2  = LL_TIM_DMABURST_BASEADDR_CCR2,

  /** TIMx_CCR3 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR3  = LL_TIM_DMABURST_BASEADDR_CCR3,

  /** TIMx_CCR4 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR4  = LL_TIM_DMABURST_BASEADDR_CCR4,

  /** TIMx_BDTR register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_BDTR  = LL_TIM_DMABURST_BASEADDR_BDTR,

  /** TIMx_CCR5 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR5  = LL_TIM_DMABURST_BASEADDR_CCR5,

  /** TIMx_CCR6 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR6  = LL_TIM_DMABURST_BASEADDR_CCR6,

  /** TIMx_CCMR3 register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_CCMR3 = LL_TIM_DMABURST_BASEADDR_CCMR3,

  /** TIMx_DTR2 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_DTR2  = LL_TIM_DMABURST_BASEADDR_DTR2,

  /** TIMx_ECR register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_ECR   = LL_TIM_DMABURST_BASEADDR_ECR,

  /** TIMx_TISEL register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_TISEL = LL_TIM_DMABURST_BASEADDR_TISEL,

  /** TIMx_AF1 register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_AF1   = LL_TIM_DMABURST_BASEADDR_AF1,

  /** TIMx_AF2 register is the DMA base address for DMA burst         */
  HAL_TIM_DMABURST_BASE_ADDR_AF2   = LL_TIM_DMABURST_BASEADDR_AF2,

  /** TIMx_CCR7 register is the DMA base address for DMA burst        */
  HAL_TIM_DMABURST_BASE_ADDR_CCR7  = LL_TIM_DMABURST_BASEADDR_CCR7,

  /** TIMx_CCMR4 register is the DMA base address for DMA burst       */
  HAL_TIM_DMABURST_BASE_ADDR_CCMR4 = LL_TIM_DMABURST_BASEADDR_CCMR4,

} hal_tim_dmaburst_base_addr_reg_t;


/**
  * @brief HAL TIM DMA Burst triggering sources.
  */
typedef enum
{
  /** DMA burst is triggered by the update event                   */
  HAL_TIM_DMABURST_UPD  = LL_TIM_DMABURST_UPD,

  /** DMA burst is triggered by the capture/compare match 1 event  */
  HAL_TIM_DMABURST_CC1  = LL_TIM_DMABURST_CC1,

  /** DMA burst is triggered by the capture/compare match 2 event  */
  HAL_TIM_DMABURST_CC2  = LL_TIM_DMABURST_CC2,

  /** DMA burst is triggered by the capture/compare match 3 event  */
  HAL_TIM_DMABURST_CC3  = LL_TIM_DMABURST_CC3,

  /** DMA burst is triggered by the capture/compare match 4 event  */
  HAL_TIM_DMABURST_CC4  = LL_TIM_DMABURST_CC4,

  /** DMA burst is triggered by the commutation event              */
  HAL_TIM_DMABURST_COM  = LL_TIM_DMABURST_COM,

  /** DMA burst is triggered by the trigger event                  */
  HAL_TIM_DMABURST_TRGI = LL_TIM_DMABURST_TRGI,

} hal_tim_dmaburst_source_t;


/**
  * @brief TIM DMA Burst triggering sources.
  * @note  DMA Burst sources mapped on hal_tim_dmaburst_source_t
  *        for internal usage.
  */
typedef enum
{
  /** DMA burst is not used                                             */
  TIM_DMABURST_NONE = 0U,

  /** DMA burst is triggered by the update event                        */
  TIM_DMABURST_UPD  = HAL_TIM_DMABURST_UPD,

  /** DMA burst is triggered by the capture/compare match 1 event       */
  TIM_DMABURST_CC1  = HAL_TIM_DMABURST_CC1,

  /** DMA burst is triggered by the capture/compare match 2 event event */
  TIM_DMABURST_CC2  = HAL_TIM_DMABURST_CC2,

  /** DMA burst is triggered by the capture/compare match 3 event event */
  TIM_DMABURST_CC3  = HAL_TIM_DMABURST_CC3,

  /** DMA burst is triggered by the capture/compare match 4 event event */
  TIM_DMABURST_CC4  = HAL_TIM_DMABURST_CC4,

  /** DMA burst is triggered by the commutation event                   */
  TIM_DMABURST_COM  = HAL_TIM_DMABURST_COM,

  /** DMA burst is triggered by the trigger event                       */
  TIM_DMABURST_TRGI = HAL_TIM_DMABURST_TRGI,

} tim_dmaburst_source_t;


/**
  * @brief HAL TIM DMA Burst Length.
  */
typedef enum
{
  /** The transfer is done to 1 register starting from the DMA burst base address   */
  HAL_TIM_DMABURST_1TRANSFER   = LL_TIM_DMABURST_LENGTH_1TRANSFER,

  /** The transfer is done to 2 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_2TRANSFERS  = LL_TIM_DMABURST_LENGTH_2TRANSFERS,

  /** The transfer is done to 3 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_3TRANSFERS  = LL_TIM_DMABURST_LENGTH_3TRANSFERS,

  /** The transfer is done to 4 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_4TRANSFERS  = LL_TIM_DMABURST_LENGTH_4TRANSFERS,

  /** The transfer is done to 5 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_5TRANSFERS  = LL_TIM_DMABURST_LENGTH_5TRANSFERS,

  /** The transfer is done to 6 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_6TRANSFERS  = LL_TIM_DMABURST_LENGTH_6TRANSFERS,

  /** The transfer is done to 7 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_7TRANSFERS  = LL_TIM_DMABURST_LENGTH_7TRANSFERS,

  /** The transfer is done to 8 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_8TRANSFERS  = LL_TIM_DMABURST_LENGTH_8TRANSFERS,

  /** The transfer is done to 9 registers starting from the DMA burst base address  */
  HAL_TIM_DMABURST_9TRANSFERS  = LL_TIM_DMABURST_LENGTH_9TRANSFERS,

  /** The transfer is done to 10 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_10TRANSFERS = LL_TIM_DMABURST_LENGTH_10TRANSFERS,

  /** The transfer is done to 11 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_11TRANSFERS = LL_TIM_DMABURST_LENGTH_11TRANSFERS,

  /** The transfer is done to 12 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_12TRANSFERS = LL_TIM_DMABURST_LENGTH_12TRANSFERS,

  /** The transfer is done to 13 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_13TRANSFERS = LL_TIM_DMABURST_LENGTH_13TRANSFERS,

  /** The transfer is done to 14 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_14TRANSFERS = LL_TIM_DMABURST_LENGTH_14TRANSFERS,

  /** The transfer is done to 15 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_15TRANSFERS = LL_TIM_DMABURST_LENGTH_15TRANSFERS,

  /** The transfer is done to 16 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_16TRANSFERS = LL_TIM_DMABURST_LENGTH_16TRANSFERS,

  /** The transfer is done to 17 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_17TRANSFERS = LL_TIM_DMABURST_LENGTH_17TRANSFERS,

  /** The transfer is done to 18 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_18TRANSFERS = LL_TIM_DMABURST_LENGTH_18TRANSFERS,

  /** The transfer is done to 19 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_19TRANSFERS = LL_TIM_DMABURST_LENGTH_19TRANSFERS,

  /** The transfer is done to 20 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_20TRANSFERS = LL_TIM_DMABURST_LENGTH_20TRANSFERS,

  /** The transfer is done to 21 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_21TRANSFERS = LL_TIM_DMABURST_LENGTH_21TRANSFERS,

  /** The transfer is done to 22 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_22TRANSFERS = LL_TIM_DMABURST_LENGTH_22TRANSFERS,

  /** The transfer is done to 23 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_23TRANSFERS = LL_TIM_DMABURST_LENGTH_23TRANSFERS,

  /** The transfer is done to 24 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_24TRANSFERS = LL_TIM_DMABURST_LENGTH_24TRANSFERS,

  /** The transfer is done to 25 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_25TRANSFERS = LL_TIM_DMABURST_LENGTH_25TRANSFERS,

  /** The transfer is done to 26 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_26TRANSFERS = LL_TIM_DMABURST_LENGTH_26TRANSFERS,

  /** The transfer is done to 27 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_27TRANSFERS = LL_TIM_DMABURST_LENGTH_27TRANSFERS,

  /** The transfer is done to 28 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_28TRANSFERS = LL_TIM_DMABURST_LENGTH_28TRANSFERS,

  /** The transfer is done to 29 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_29TRANSFERS = LL_TIM_DMABURST_LENGTH_29TRANSFERS,

  /** The transfer is done to 30 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_30TRANSFERS = LL_TIM_DMABURST_LENGTH_30TRANSFERS,

  /** The transfer is done to 31 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_31TRANSFERS = LL_TIM_DMABURST_LENGTH_31TRANSFERS,

  /** The transfer is done to 32 registers starting from the DMA burst base address */
  HAL_TIM_DMABURST_32TRANSFERS = LL_TIM_DMABURST_LENGTH_32TRANSFERS,

} hal_tim_dmaburst_length_t;

/**
  * @brief HAL TIM DMA Burst Direction.
  */
typedef enum
{
  /** DMA Burst read operation to transfer Data from the TIM peripheral to the memory */
  HAL_TIM_DMABURST_READ  = 0U,

  /** DMA Burst write operation to transfer Data from the memory to the TIM peripheral */
  HAL_TIM_DMABURST_WRITE = 1U,

} hal_tim_dmaburst_direction_t;
#endif /* USE_HAL_TIM_DMA */


/**
  * @brief  HAL TIM Break Input.
  */
typedef enum
{
  /** Timer break input 1                          */
  HAL_TIM_BREAK_INPUT_1  = LL_TIM_BREAK_INPUT_1,

  /** Timer break input 2                          */
  HAL_TIM_BREAK_INPUT_2  = LL_TIM_BREAK_INPUT_2,

} hal_tim_break_input_id_t;


/**
  * @brief HAL TIM Break Input Polarity.
  */
typedef enum
{
  /** Break input is active low                        */
  HAL_TIM_BREAK_INPUT_LOW  = LL_TIM_BREAK_POLARITY_LOW,

  /** Break input is active high                       */
  HAL_TIM_BREAK_INPUT_HIGH = LL_TIM_BREAK_POLARITY_HIGH,

} hal_tim_break_input_polarity_t;


/**
  * @brief HAL TIM Break Input Function Mode.
  */
typedef enum
{
  /** Break input in input mode                                                */
  HAL_TIM_BREAK_INPUT_MODE_INPUT = LL_TIM_BREAK_AFMODE_INPUT,

  /** Break input in bidirectional mode.
    *  In bidirectional mode the Break input is configured
    *  both in input mode and in open drain output mode.
    *  Any active Break event will assert a low logic level on the Break
    *  input to indicate an internal break event to external devices.          */
  HAL_TIM_BREAK_INPUT_MODE_BIDIRECTIONAL = (LL_TIM_BREAK_AFMODE_BIDIRECTIONAL |
                                            LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL),

} hal_tim_break_input_mode_t;


/**
  * @brief HAL TIM Break Input status.
  */
typedef enum
{
  /** Break input is disabled      */
  HAL_TIM_BREAK_INPUT_DISABLED = 0U,

  /** Break input is enabled       */
  HAL_TIM_BREAK_INPUT_ENABLED  = 1U,

} hal_tim_break_input_status_t;


/**
  * @brief HAL TIM Break Input Source Polarity.
  */
typedef enum
{
  /** Break input source polarity is not inverted                         */
  HAL_TIM_BREAK_INPUT_SRC_NONINVERTED = LL_TIM_BREAK_INPUT_SRC_NONINVERTED,

  /** Break input source polarity is inverted                             */
  HAL_TIM_BREAK_INPUT_SRC_INVERTED = LL_TIM_BREAK_INPUT_SRC_INVERTED,

} hal_tim_break_input_src_polarity_t;


/**
  * @brief HAL TIM Break Input Source Status.
  */
typedef enum
{
  /** Break input source is disabled   */
  HAL_TIM_BREAK_INPUT_SRC_DISABLED = 0U,

  /** Break input source is enabled    */
  HAL_TIM_BREAK_INPUT_SRC_ENABLED = 1U,

} hal_tim_break_input_src_status_t;


/**
  * @brief HAL TIM Main Output status.
  */
typedef enum
{
  /** Main output is disabled            */
  HAL_TIM_BREAK_MAIN_OUTPUT_DISABLED = 0U,

  /** Main output is enabled             */
  HAL_TIM_BREAK_MAIN_OUTPUT_ENABLED  = 1U,

} hal_tim_break_main_output_status_t;


/**
  * @brief HAL TIM Automatic Output status.
  */
typedef enum
{
  /** Main output can only be enabled by software                       */
  HAL_TIM_BREAK_AUTOMATIC_OUTPUT_DISABLED = 0U,

  /** Main output can be enabled by software or automatically at the next
      update event (if none of the break inputs BRK and BRK2 is active) */
  HAL_TIM_BREAK_AUTOMATIC_OUTPUT_ENABLED = 1U,

} hal_tim_break_automatic_output_status_t;

/**
  * @brief HAL TIM Break Delay.
  */
typedef enum
{
  /** Delayed 1 break                       */
  HAL_TIM_BREAK_DELAY1 = LL_TIM_BREAK_DELAY1,

  /** Delayed 2 break                       */
  HAL_TIM_BREAK_DELAY2 = LL_TIM_BREAK_DELAY2,

} hal_tim_break_delay_t;


/**
  * @brief Off-state selection for run (ossr) mode.
  */
typedef enum
{
  /** When inactive, OCx/OCxN outputs are disabled (forced to Hi-Z state) */
  HAL_TIM_OFF_STATE_RUN_DISABLE = LL_TIM_OSSR_DISABLE,

  /** When inactive, OCx/OCxN outputs are enabled with their inactive
      level as soon as CCxE=1 or CCxNE=1                                  */
  HAL_TIM_OFF_STATE_RUN_ENABLE  = LL_TIM_OSSR_ENABLE,

} hal_tim_off_state_run_t;


/**
  * @brief Off-state selection for idle (ossi) mode.
  */
typedef enum
{
  /** When inactive, OCx/OCxN outputs are disabled (forced to Hi-Z state)  */
  HAL_TIM_OFF_STATE_IDLE_DISABLE = LL_TIM_OSSI_DISABLE,

  /** When inactive, OxC/OCxN outputs are first forced with their inactive
      level then forced to their idle level after the deadtime             */
  HAL_TIM_OFF_STATE_IDLE_ENABLE = LL_TIM_OSSI_ENABLE,

} hal_tim_off_state_idle_t;


/**
  * @brief HAL TIM output disable status.
  */
typedef enum
{
  /** Break was triggered (or MOE was written to 0), tim_ocx and
      tim_ocxn are forced to their Output idle state (OIS)       */
  HAL_TIM_OUTPUT_IDLE_STATE = LL_TIM_OUTPUT_IDLE_STATE,

  /** Break2 was triggered, tim_ocx and tim_ocxn outputs are
      forced to their inactive level                             */
  HAL_TIM_OUTPUT_DISABLED_STATE = LL_TIM_OUTPUT_DISABLED_STATE,

} hal_tim_output_disable_status_t;


/**
  * @brief HAL TIM deadtime preload status.
  */
typedef enum
{
  /** Deadtime preload is disabled       */
  HAL_TIM_DEADTIME_PRELOAD_DISABLED = 0U,

  /** Deadtime preload is enabled        */
  HAL_TIM_DEADTIME_PRELOAD_ENABLED  = 1U,

} hal_tim_deadtime_preload_status_t;

/**
  * @brief HAL TIM asymmetrical deadtime status.
  */
typedef enum
{
  /** Asymmetrical deadtime is disabled       */
  HAL_TIM_ASYMMETRICAL_DEADTIME_DISABLED = 0U,

  /** Asymmetrical deadtime is enabled        */
  HAL_TIM_ASYMMETRICAL_DEADTIME_ENABLED  = 1U,

} hal_tim_asymmetrical_deadtime_status_t;


/**
  * @brief  HAL TIM write protection levels definition.
  */
typedef enum
{
  /** LOCK OFF - No bit is write protected */
  HAL_TIM_LOCK_OFF = LL_TIM_LOCKLEVEL_OFF,

  /** LOCK Level 1                         */
  HAL_TIM_LOCK_1   = LL_TIM_LOCKLEVEL_1,

  /** LOCK Level 2                         */
  HAL_TIM_LOCK_2   = LL_TIM_LOCKLEVEL_2,

  /** LOCK Level 3                         */
  HAL_TIM_LOCK_3   = LL_TIM_LOCKLEVEL_3,

} hal_tim_lock_level_t;


/**
  * @brief HAL TIM Commutation trigger selection.
  */
typedef enum
{
  /** Capture/compare control bits are updated by setting the COMG bit only            */
  HAL_TIM_COMMUTATION_SOFTWARE = LL_TIM_CCUPDATESOURCE_SOFTWARE,

  /** Capture/compare control bits are updated by setting the COMG bit or
      when a rising edge occurs on trigger input                                       */
  HAL_TIM_COMMUTATION_SOFTWARE_AND_TRIGGER = LL_TIM_CCUPDATESOURCE_SOFTWARE_AND_TRIGGER,

} hal_tim_commutation_src_t;


/**
  * @brief HAL TIM Commutation status.
  */
typedef enum
{
  /** Commutation is disabled      */
  HAL_TIM_COMMUTATION_DISABLED = 0U,

  /** Commutation is enabled       */
  HAL_TIM_COMMUTATION_ENABLED = 1U,

} hal_tim_commutation_status_t;


#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief HAL TIM Capture/Compare DMA Request Source.
  */
typedef enum
{
  /** CCx DMA request sent when CCx event occurs     */
  HAL_TIM_CC_DMAREQ_CC = LL_TIM_CCDMAREQUEST_CC,

  /** CCx DMA requests sent when update event occurs */
  HAL_TIM_CC_DMAREQ_UPD = LL_TIM_CCDMAREQUEST_UPD,

} hal_tim_cc_dmareq_src_t;
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  HAL TIM Software Event definition.
  */
typedef enum
{
  /** Reinitialize the counter and generates an update of the registers */
  HAL_TIM_SW_EVENT_UPD  = LL_TIM_SW_EVENT_UPD,

  /** A capture/compare event is generated on channel 1                 */
  HAL_TIM_SW_EVENT_CC1  = LL_TIM_SW_EVENT_CC1,

  /** A capture/compare event is generated on channel 2                 */
  HAL_TIM_SW_EVENT_CC2  = LL_TIM_SW_EVENT_CC2,

  /** A capture/compare event is generated on channel 3                 */
  HAL_TIM_SW_EVENT_CC3  = LL_TIM_SW_EVENT_CC3,

  /** A capture/compare event is generated on channel 4                 */
  HAL_TIM_SW_EVENT_CC4  = LL_TIM_SW_EVENT_CC4,

  /** A commutation event is generated                                  */
  HAL_TIM_SW_EVENT_COM  = LL_TIM_SW_EVENT_COM,

  /** A trigger event is generated                                      */
  HAL_TIM_SW_EVENT_TRGI = LL_TIM_SW_EVENT_TRGI,

  /** A break event is generated                                        */
  HAL_TIM_SW_EVENT_BRK  = LL_TIM_SW_EVENT_BRK,

  /** A break 2 event is generated                                      */
  HAL_TIM_SW_EVENT_BRK2 = LL_TIM_SW_EVENT_BRK2,

} hal_tim_sw_event_id_t;


/**
  * @brief HAL TIM Time Base Configuration Structure definition.
  * @note :
  *        The update event period is calculated as follows:
  *        update_event = TIM_CLK/((prescaler + 1)*(period + 1)*(repetition + 1))
  */
typedef struct
{
  /** Specifies the prescaler value used to divide the timer kernel clock. \n
      This parameter can be a number between Min_Data = 0x0000
      and Max_Data = 0xFFFF. */
  uint32_t prescaler;

  /** Specifies the counter mode */
  hal_tim_counter_mode_t counter_mode;

  /** Specifies the period value to be loaded into the active
      Auto-Reload Register. \n
      For a counter with a 16-bit resolution, this parameter can be a number
      between Min_Data = 0x0001 and Max_Data = 0xFFFF (or 0xFFFEF if dithering
      is activated). In non-dithering mode, only bits 15:0 hold the period
      value. In dithering mode, the integer part of the period is in bits 19:4
      and bits 3:0 hold the dithering part. \n
      For a counter with a 32-bit resolution, this parameter can be a number
      between Min_Data = 0x00000001 and Max_Data = 0xFFFFFFFF (or 0xFFFFFFEF if
      dithering is activated). The register holds the period value in
      non-dithering mode. In dithering mode, the integer part is in ARR[31:4] and
      ARR[3:0] bitfield contains the dithered part. */
  uint32_t period;

  /** Specifies the repetition counter value for instances that support it. \n
      If the repetition counter is used, the update event (UEV) is generated
      after upcounting repeats the number of times programmed in the
      repetition counter register (RCR). \n
      Otherwise, the update event is generated at each counter overflow.
      The value is encoded on 8 or 16 bits depending on the instance. */
  uint32_t repetition_counter;

  /** TIM clock selection. \n
      Specifies the source of the clock feeding the timer's prescaler.
      Also specifies the trigger input to be used to synchronize the counter
      in case the clock source is external mode 1. */
  hal_tim_clock_sel_t clock_sel;

} hal_tim_config_t;


/**
  * @brief HAL TIM Output Channel Configuration Structure definition.
  */
typedef struct
{
  /** Specifies the output channel mode */
  hal_tim_oc_mode_t mode;

  /** Specifies the pulse value to be loaded into the Capture/Compare Register. \n
      For a 16 bits counter,this parameter can be a number between
      Min_Data = 0x0000 and Max_Data = 0xFFFF (or 0xFFFEF if dithering is
      activated in which case bits[3:0] represent the dithered part and
      bits[19:4] the integer part).
      For a 32 bits counter,this parameter can be a number between
      Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF (or 0xFFFFFFEF if
      dithering is activated in which case bits[3:0] represent the dithered part
      and bits[31:4] the integer part). */
  uint32_t pulse;

} hal_tim_oc_compare_unit_config_t;


/**
  * @brief HAL TIM Output Channel Configuration Structure definition.
  */
typedef struct
{
  /** Specifies the output channel (CHx or CHxN) polarity */
  hal_tim_oc_polarity_t polarity;

  /** Specifies the output channel (CHx or CHxN) state during Idle state. \n
      This parameter is only valid for timer instances supporting break feature. */
  hal_tim_oc_idle_state_t idle_state;

  /** Specifies the output channel (CHx or CHxN) state when it is disabled. \n
      This parameter is only valid for timer instances supporting the break feature
      and is not applicable to internal channels. */
  hal_tim_oc_override_state_t override_state;

  /** Specifies the output channel (CHx or CHxN) break mode. \n
      This parameter is only valid for timer instances supporting the break feature
      and is not applicable to internal channels. */
  hal_tim_oc_break_mode_t break_mode;

} hal_tim_oc_channel_config_t;


/**
  * @brief HAL TIM Input Channel Configuration Structure definition.
  */
typedef struct
{
  /** Specifies the input source                    */
  hal_tim_channel_src_t source;

  /** Specifies the active edge of the input signal */
  hal_tim_ic_polarity_t polarity;

  /** Specifies the input channel filter            */
  hal_tim_filter_t filter;

} hal_tim_ic_channel_config_t;

/**
  * @brief HAL TIM Input Channel Capture Configuration Structure definition.
  */
typedef struct
{

  /** Specifies the signal to capture          */
  hal_tim_ic_capture_unit_src_t source;

  /** Specifies the input capture prescaler    */
  hal_tim_ic_capture_unit_prescaler_t prescaler;

} hal_tim_ic_capture_unit_config_t;


/**
  * @brief  TIM Index configuration Structure definition.
  * @note   Index input (ETR input polarity, prescaler and filter)
  *         is configured separately
  */
typedef struct
{
  /** Specifies in which counter direction
      the index event resets the counter                    */
  hal_tim_encoder_index_dir_t dir;

  /** Specifies in which AB input configuration
      the index event resets the counter                    */
  hal_tim_encoder_index_pos_sel_t pos;

  /** Specifies whether or not the index event
      is conditioned by TI3 or TI4 input                    */
  hal_tim_encoder_index_blank_mode_t blanking;

  /** Specifies whether index is always active or only once */
  hal_tim_encoder_index_sel_t idx;

} hal_tim_encoder_index_config_t;


/**
  * @brief  TIM ETR configuration Structure definition.
  */
typedef struct
{
  /** Specifies the external trigger input source                */
  hal_tim_ext_trig_src_t source;

  /** Specifies the external trigger input polarity              */
  hal_tim_ext_trig_polarity_t polarity;

  /** Specifies the external trigger input filter                */
  hal_tim_filter_t filter;

  /** Specifies the external trigger input prescaler             */
  hal_tim_ext_trig_prescaler_t prescaler;

  /** Specifies the external trigger input synchronous prescaler */
  hal_tim_ext_trig_sync_prescaler_t sync_prescaler;

} hal_tim_ext_trig_config_t;


/**
  * @brief  TIM Slave mode controller configuration Structure definition.
  */
typedef struct
{
  /** Specifies the slave mode                          */
  hal_tim_slave_mode_t mode;

  /** Specifies the slave mode controller trigger input */
  hal_tim_trig_sel_t trigger;

} hal_tim_slave_config_t;


/**
  * @brief TIM Trigger Output 2 configuration Structure definition.
  */
typedef struct
{
  /** Specifies the trigger-output2 source                                */
  hal_tim_trigger_output2_source_t trgo2_src;

  /** Specifies the trigger-output2 postscaler. \n
      The number of events on tim_trgo2 is divided by (TGO2PSC[4:0] + 1). */
  uint32_t postscaler;

} hal_tim_trigger_output2_config_t;


#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/**
  * @brief TIM DMA Burst operation specification Structure definition.
  */
typedef struct
{
  /** Specifies the DMA burst base address */
  hal_tim_dmaburst_base_addr_reg_t address;

  /** Specifies the DMA burst source       */
  hal_tim_dmaburst_source_t source;

  /** Specifies the DMA burst length       */
  hal_tim_dmaburst_length_t length;

} hal_tim_dmaburst_config_t;
#endif /* USE_HAL_TIM_DMA */


/**
  * @brief  TIM Break input(s) configuration Structure definition.
  * @note   2 break inputs can be configured (BKIN and BKIN2) with
  *         configurable filter, polarity and mode (input or bidirectional).
  */
typedef struct
{
  /** Specifies the break input polarity                        */
  hal_tim_break_input_polarity_t polarity;

  /** Specifies the break input filter                          */
  hal_tim_filter_t filter;

  /** Specifies whether the break input is bidirectional or not.
      (only for instances that support it)                       */
  hal_tim_break_input_mode_t mode;

} hal_tim_break_input_config_t;


/**
  * @brief Off-state configuration for RUN and IDLE modes.
  */
typedef struct
{
  /** Specifies the state of the output channel
      when the main output is enabled          */
  hal_tim_off_state_run_t off_state_run;

  /** Specifies the state of the output channel
      when the main output is disabled         */
  hal_tim_off_state_idle_t off_state_idle;

} hal_tim_off_states_config_t;


/**
  * @brief HAL TIM Pulse Generator Configuration Structure definition.
  */
typedef struct
{
  /** Specifies the pulse width prescaler                  */
  hal_tim_pulse_prescaler_t prescaler;

  /** Specifies the pulse width.
      This parameter can be a number between 0x00 and 0xFF */
  uint32_t pulse_width;

} hal_tim_pulse_generator_config_t;


/**
  * @brief HAL TIM Time Base Handle type definition.
  */

typedef struct hal_tim_handle_s hal_tim_handle_t;

/*
 * Types definitions for the callbacks
 */
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
/** HAL TIM generic callback pointer definition */
typedef void (* hal_tim_cb_t)(hal_tim_handle_t *);
/** HAL TIM callback pointer definition with channel parameter */
typedef void (* hal_tim_channel_cb_t)(hal_tim_handle_t *, hal_tim_channel_t);
/** HAL TIM callback pointer definition with break input parameter */
typedef void (* hal_tim_break_cb_t)(hal_tim_handle_t *, hal_tim_break_input_id_t);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

/**
  * @brief HAL TIM Time Base Handle Structure definition.
  */
struct hal_tim_handle_s
{
  /** HAL TIM instance */
  hal_tim_t instance;

  /** TIM global state */
  volatile hal_tim_state_t global_state;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** DMA Handlers array. (@ref hal_tim_dma_index_t) */
  hal_dma_handle_t *hdma[HAL_TIM_DMA_REQUESTS];

  /** DMA Burst source */
  volatile tim_dmaburst_source_t dmaburst_source;
#endif /* USE_HAL_TIM_DMA */

  /** TIM channels state array */
  volatile hal_tim_channel_state_t channel_states[HAL_TIM_CHANNELS];

#if defined(USE_HAL_TIM_USER_DATA) && (USE_HAL_TIM_USER_DATA == 1)
  /** User Data Pointer  */
  const void *p_user_data;
#endif /* USE_HAL_TIM_USER_DATA */

#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)
  /** Store last error code */
  volatile uint32_t last_error_codes;
#endif /* USE_HAL_TIM_GET_LAST_ERRORS */

#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Error callback */
  hal_tim_cb_t error_callback;

  /** TIM Update DMA stop callback */
  hal_tim_cb_t stop_callback;

  /** TIM capture/Compare DMA stop callback */
  hal_tim_channel_cb_t channel_stop_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Update callback */
  hal_tim_cb_t update_callback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Update Half Complete callback */
  hal_tim_cb_t update_half_cplt_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Trigger callback */
  hal_tim_cb_t trigger_callback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Trigger Half Complete callback */
  hal_tim_cb_t trigger_half_cplt_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Input Capture callback */
  hal_tim_channel_cb_t input_capture_callback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Input Capture Half Complete callback */
  hal_tim_channel_cb_t input_capture_half_cplt_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Compare Match callback */
  hal_tim_channel_cb_t compare_match_callback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Compare Match Half Complete callback */
  hal_tim_channel_cb_t compare_match_half_cplt_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Commutation callback */
  hal_tim_cb_t commutation_callback;

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
  /** TIM Commutation Half Complete callback */
  hal_tim_cb_t commutation_half_cplt_callback;
#endif /* USE_HAL_TIM_DMA */

  /** TIM Break callback */
  hal_tim_cb_t break_callback;

  /** TIM Break2 callback */
  hal_tim_cb_t break2_callback;

  /** TIM System Break callback */
  hal_tim_cb_t system_break_callback;

  /** TIM Software Break callback */
  hal_tim_break_cb_t software_break_callback;

  /** TIM Encoder Index callback */
  hal_tim_cb_t encoder_index_callback;

  /** TIM Direction Change callback */
  hal_tim_cb_t direction_change_callback;

  /** TIM Index Error callback */
  hal_tim_cb_t index_error_callback;

  /** TIM Transition Error callback */
  hal_tim_cb_t transition_error_callback;

#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
};


/**
  * @}
  */
/* End of exported types ---------------------------------------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_Exported_Functions HAL TIM Functions
  * @{
  */


/** @defgroup TIM_Exported_Functions_Group1 Timer Initialization/Deinitialization function
  * @{
  */
hal_status_t HAL_TIM_Init(hal_tim_handle_t *htim, hal_tim_t instance);
void HAL_TIM_DeInit(hal_tim_handle_t *htim);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_SetDMA(hal_tim_handle_t *htim,
                            hal_tim_dma_index_t dma_idx,
                            hal_dma_handle_t *hdma);
#endif /* USE_HAL_TIM_DMA */
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group2 Timer Peripheral State, Error functions, Kernel Clock Frequency.
  *           This group gathers the functions for state, error and kernel clock management.
  * @{
  */
hal_tim_state_t HAL_TIM_GetState(const hal_tim_handle_t *htim);

hal_tim_channel_state_t HAL_TIM_GetChannelState(const hal_tim_handle_t *htim,
                                                hal_tim_channel_t channel);

#if defined (USE_HAL_TIM_GET_LAST_ERRORS) && (USE_HAL_TIM_GET_LAST_ERRORS == 1)

uint32_t HAL_TIM_GetLastErrorCodes(const hal_tim_handle_t *htim);

#endif /* USE_HAL_TIM_GET_LAST_ERRORS */

uint32_t HAL_TIM_GetClockFreq(const hal_tim_handle_t *htim);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group3 Timer Timebase configuration and control
  *            This group contains the functions used to configure and control the
  *            time base unit.
  *
  *            The configuration includes:
  *             - Clock source selection
  *               - internal
  *               - external mode 1
  *               - external mode 2
  *               - encoder mode
  *             - Autoreload setting
  *             - Prescaler setting
  *             - Repetition counter setting
  *             - Dithering management
  *
  *            Control functions start or stop the timer's counter. \n
  *            Three execution modes are proposed:
  *             - polling: \n
  *               Neither update interrupt nor update DMA request are enabled. \n
  *               It is up to the application to read/clear the update flag if
  *               needed.
  *             - interrupt mode: \n
  *               Update interrupt is enabled.
  *             - DMA mode (commutation disabled): \n
  *               A DMA transfer is started in interrupt mode and Update DMA request
  *               is enabled. \n
  *               At every update event, the DMA transfers one word from the memory
  *               to the autoreload register.
  *
  *            @note
  *             - When the slave mode controller is configured in trigger mode
  *               the timer's counter isn't started by the software.
  *               (the timer's counter starts in response to an event on a selected trigger input)
  *
  *             - When commutation is enabled
  *                  - interrupt mode: \n
  *                    Commutation interrupt is enabled
  *                  - DMA mode: \n
  *                    Commutation DMA request is enabled. \n
  *                    At every commutation event, the DMA transfer takes place as
  *                    per DMA transfer configuration. \n
  *                    Note that in that case the start of the DMA transfer is under
  *                    application's responsibility. \n
  *
  *             - When the slave node is enabled
  *                  - interrupt mode: \n
  *                    Trigger interrupt is enabled. \n
  *
  * @{
  */
hal_status_t HAL_TIM_SetConfig(hal_tim_handle_t *htim,
                               const hal_tim_config_t *p_config);
void HAL_TIM_GetConfig(const hal_tim_handle_t *htim,
                       hal_tim_config_t *p_config);

hal_status_t HAL_TIM_SetPeriod(hal_tim_handle_t *htim,
                               uint32_t period);
uint32_t HAL_TIM_GetPeriod(const hal_tim_handle_t *htim);

hal_status_t
HAL_TIM_SetDitheredPeriod(hal_tim_handle_t *htim,
                          uint32_t period,
                          hal_tim_dithering_pattern_t period_dithering_pattern);
void HAL_TIM_GetDitheredPeriod(const hal_tim_handle_t *htim,
                               uint32_t *p_period,
                               hal_tim_dithering_pattern_t *p_period_dithering_pattern);

hal_status_t HAL_TIM_SetPrescaler(hal_tim_handle_t *htim,
                                  uint32_t prescaler);
uint32_t HAL_TIM_GetPrescaler(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetCounterMode(hal_tim_handle_t *htim,
                                    hal_tim_counter_mode_t counter_mode);
hal_tim_counter_mode_t HAL_TIM_GetCounterMode(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetDTSPrescaler(hal_tim_handle_t *htim,
                                     hal_tim_dts_prescaler_t dts_prescaler);
hal_tim_dts_prescaler_t HAL_TIM_GetDTSPrescaler(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetDTS2Prescaler(hal_tim_handle_t *htim,
                                      hal_tim_dts2_prescaler_t dts2_prescaler);
hal_tim_dts2_prescaler_t HAL_TIM_GetDTS2Prescaler(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetRepetitionCounter(hal_tim_handle_t *htim,
                                          uint32_t repetition_counter);
uint32_t HAL_TIM_GetRepetitionCounter(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetClockSource(hal_tim_handle_t *htim,
                                    const hal_tim_clock_sel_t *p_clk_sel);
void HAL_TIM_GetClockSource(const hal_tim_handle_t *htim,
                            hal_tim_clock_sel_t *p_clk_sel);

hal_status_t HAL_TIM_SetCounter(hal_tim_handle_t *htim, uint32_t counter_value);

uint32_t HAL_TIM_GetCounter(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableUpdateGeneration(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableUpdateGeneration(hal_tim_handle_t *htim);
hal_tim_update_generation_status_t HAL_TIM_IsEnabledUpdateGeneration(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetUpdateSource(hal_tim_handle_t *htim,
                                     hal_tim_update_src_t update_source);
hal_tim_update_src_t HAL_TIM_GetUpdateSource(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableUpdateFlagRemap(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableUpdateFlagRemap(hal_tim_handle_t *htim);
hal_tim_update_flag_remap_status_t HAL_TIM_IsEnabledUpdateFlagRemap(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableAutoReloadPreload(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableAutoReloadPreload(hal_tim_handle_t *htim);
hal_tim_auto_reload_preload_status_t HAL_TIM_IsEnabledAutoReloadPreload(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableDithering(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableDithering(hal_tim_handle_t *htim);
hal_tim_dithering_status_t HAL_TIM_IsEnabledDithering(const hal_tim_handle_t *htim);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_SetCaptureCompareDMAReqSource(hal_tim_handle_t *htim,
                                                   hal_tim_cc_dmareq_src_t cc_dmareq_source);
hal_tim_cc_dmareq_src_t HAL_TIM_GetCaptureCompareDMAReqSource(const hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_DMA */

hal_status_t HAL_TIM_Start(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_Stop(hal_tim_handle_t *htim);

hal_status_t HAL_TIM_Start_IT(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_Start_IT_Opt(hal_tim_handle_t *htim, uint32_t interrupts);
hal_status_t HAL_TIM_Stop_IT(hal_tim_handle_t *htim);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_Start_DMA(hal_tim_handle_t *htim,
                               const uint8_t *p_data,
                               uint32_t size_byte);
hal_status_t HAL_TIM_Start_DMA_Opt(hal_tim_handle_t *htim,
                                   const uint8_t *p_data,
                                   uint32_t size_byte,
                                   uint32_t interrupts);
hal_status_t HAL_TIM_Stop_DMA(hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_DMA */
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group4 Timer Output Channel functions
  *            This group contains the functions used to configure and control
  *            the output stage of the timer's capture/compare channels.
  *
  *            The output stage of a timer can be used to
  *             - control an output waveform
  *             - generate complementary PWM signals with deadtime insertion
  *             - indicate when a period has elapsed
  *
  *            The configuration of an output channel includes:
  *             - selection of the output mode (e.g. output compare, PWM, ...)
  *             - selection of the channel polarity
  *             - setting of the compare value
  *
  *            Control functions enable or disable the timer's output channel. \n
  *            Three execution modes are available:
  *             - polling \n
  *               Neither compare match interrupt nor compare match DMA request
  *               are enabled. \n
  *               Read/clear the compare match flag if needed.
  *             - interrupt mode \n
  *               Compare match interrupt is enabled for the concerned channel.
  *             - DMA \n
  *               A DMA transfer is started in interrupt mode and compare match
  *               DMA request is enabled. \n
  *               At every compare match event, the DMA transfers one word from
  *               the memory to the capture/compare register.
  *
  *
  * @{
  */
hal_status_t HAL_TIM_OC_SetConfigCompareUnit(hal_tim_handle_t *htim,
                                             hal_tim_oc_compare_unit_t compare_unit,
                                             const hal_tim_oc_compare_unit_config_t *p_config);
void HAL_TIM_OC_GetConfigCompareUnit(const hal_tim_handle_t *htim,
                                     hal_tim_oc_compare_unit_t compare_unit,
                                     hal_tim_oc_compare_unit_config_t *p_config);

hal_status_t HAL_TIM_OC_SetCompareUnitPulse(hal_tim_handle_t *htim,
                                            hal_tim_oc_compare_unit_t compare_unit,
                                            uint32_t pulse);
uint32_t HAL_TIM_OC_GetCompareUnitPulse(const hal_tim_handle_t *htim,
                                        hal_tim_oc_compare_unit_t compare_unit);

hal_status_t HAL_TIM_OC_SetCompareUnitDitheredPulse(hal_tim_handle_t *htim,
                                                    hal_tim_oc_compare_unit_t compare_unit,
                                                    uint32_t pulse,
                                                    hal_tim_dithering_pattern_t pulse_dithering_pattern);
void HAL_TIM_OC_GetCompareUnitDitheredPulse(const hal_tim_handle_t *htim,
                                            hal_tim_oc_compare_unit_t compare_unit,
                                            uint32_t *p_pulse,
                                            hal_tim_dithering_pattern_t *p_pulse_dithering_pattern);

hal_status_t HAL_TIM_OC_EnableComparePreload(hal_tim_handle_t *htim,
                                             hal_tim_oc_compare_unit_t compare_unit);
hal_status_t HAL_TIM_OC_DisableComparePreload(hal_tim_handle_t *htim,
                                              hal_tim_oc_compare_unit_t compare_unit);
hal_tim_oc_compare_preload_status_t HAL_TIM_OC_IsEnabledComparePreload(const hal_tim_handle_t *htim,
                                                                       hal_tim_oc_compare_unit_t compare_unit);

hal_status_t HAL_TIM_OC_EnableCompareFastMode(hal_tim_handle_t *htim,
                                              hal_tim_oc_compare_unit_t compare_unit);
hal_status_t HAL_TIM_OC_DisableCompareFastMode(hal_tim_handle_t *htim,
                                               hal_tim_oc_compare_unit_t compare_unit);
hal_tim_oc_compare_fast_mode_status_t HAL_TIM_OC_IsEnabledCompareFastMode(const hal_tim_handle_t *htim,
                                                                          hal_tim_oc_compare_unit_t compare_unit);

hal_status_t HAL_TIM_OC_SetConfigChannel(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const hal_tim_oc_channel_config_t *p_config);
void HAL_TIM_OC_GetConfigChannel(const hal_tim_handle_t *htim,
                                 hal_tim_channel_t channel,
                                 hal_tim_oc_channel_config_t *p_config);

hal_status_t HAL_TIM_OC_EnableOutputOverride(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_OC_DisableOutputOverride(hal_tim_handle_t *htim);
hal_tim_oc_output_override_status_t HAL_TIM_OC_IsEnabledOutputOverride(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_OC_SetPulseGenerator(hal_tim_handle_t *htim,
                                          const hal_tim_pulse_generator_config_t *p_config);
void HAL_TIM_OC_GetPulseGenerator(const hal_tim_handle_t *htim,
                                  hal_tim_pulse_generator_config_t *p_config);

hal_status_t HAL_TIM_OC_SetGroupChannel(hal_tim_handle_t *htim,
                                        uint32_t group);
uint32_t HAL_TIM_OC_GetGroupChannel(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_OC_StartChannel(hal_tim_handle_t *htim,
                                     hal_tim_channel_t channel);
hal_status_t HAL_TIM_OC_StopChannel(hal_tim_handle_t *htim,
                                    hal_tim_channel_t channel);

hal_status_t HAL_TIM_OC_StartChannel_IT(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel);
hal_status_t HAL_TIM_OC_StopChannel_IT(hal_tim_handle_t *htim,
                                       hal_tim_channel_t channel);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_OC_StartChannel_DMA(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const uint8_t *p_data,
                                         uint32_t size_byte);
hal_status_t HAL_TIM_OC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel,
                                             const uint8_t *p_data,
                                             uint32_t size_byte,
                                             uint32_t interrupts);
hal_status_t HAL_TIM_OC_StopChannel_DMA(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel);
#endif /* USE_HAL_TIM_DMA */

/**
  * @brief  Helper function to get a compare unit from an output channel.
  * @param  channel  Output channel. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *                  @arg @ref HAL_TIM_CHANNEL_5
  *                  @arg @ref HAL_TIM_CHANNEL_6
  *                  @arg @ref HAL_TIM_CHANNEL_7
  *                  @arg @ref HAL_TIM_CHANNEL_1N
  *                  @arg @ref HAL_TIM_CHANNEL_2N
  *                  @arg @ref HAL_TIM_CHANNEL_3N
  *                  @arg @ref HAL_TIM_CHANNEL_4N
  * @retval hal_tim_oc_compare_unit_t  Compare unit corresponding to the
  *         output channel.
  */
__STATIC_INLINE hal_tim_oc_compare_unit_t
hal_tim_oc_channel_to_compare_unit(hal_tim_channel_t channel)
{
  const hal_tim_oc_compare_unit_t ch_to_cmp_unit[] =
  {
    HAL_TIM_OC_COMPARE_UNIT_1,  /* HAL_TIM_CHANNEL_1  */
    HAL_TIM_OC_COMPARE_UNIT_2,  /* HAL_TIM_CHANNEL_2  */
    HAL_TIM_OC_COMPARE_UNIT_3,  /* HAL_TIM_CHANNEL_3  */
    HAL_TIM_OC_COMPARE_UNIT_4,  /* HAL_TIM_CHANNEL_4  */
    HAL_TIM_OC_COMPARE_UNIT_5,  /* HAL_TIM_CHANNEL_5  */
    HAL_TIM_OC_COMPARE_UNIT_6,  /* HAL_TIM_CHANNEL_6  */
    HAL_TIM_OC_COMPARE_UNIT_7,  /* HAL_TIM_CHANNEL_7  */
    HAL_TIM_OC_COMPARE_UNIT_1,  /* HAL_TIM_CHANNEL_1N */
    HAL_TIM_OC_COMPARE_UNIT_2,  /* HAL_TIM_CHANNEL_2N */
    HAL_TIM_OC_COMPARE_UNIT_3,  /* HAL_TIM_CHANNEL_3N */
    HAL_TIM_OC_COMPARE_UNIT_4,  /* HAL_TIM_CHANNEL_4N */
  };

  return ch_to_cmp_unit[channel];
}

/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group5 Timer Input Channel functions
  *            This group contains the functions used to configure and control
  *            the input stage of the timer's capture/compare channels.
  *
  *            The input stage of a timer can be used to
  *             - inject an external clock
  *             - detect a trigger
  *             - measure the period and duty cycle of a PWM signal
  *             - interface with an incremental (quadrature) decoder
  *             - interface with a hall sensor
  *
  *            The configuration of an input stage includes:
  *             - selection of a channel input source
  *             - selection of the input polarity
  *             - selection of the filter
  *             - configuration of the capture unit
  *               - input capture source selection
  *               - capture prescaler
  *
  *            The configuration of the XOR gate (if available) includes:
  *             - position (global)
  *             - per-input inversion (typically TI1, TI2 and TI3)
  *             - enable/disable
  *
  *            Control functions enable or disable the timer's input channel. \n
  *            Three execution modes are proposed:
  *             - polling \n
  *               Neither capture interrupt nor capture DMA request are enabled. \n
  *               It is up to the application to read/clear the capture flag if
  *               needed.
  *             - interrupt mode \n
  *               Capture interrupt is enabled for the concerned channel.
  *             - DMA \n
  *               A DMA transfer is started in interrupt mode and capture DMA
  *               request is enabled. \n
  *               At every capture event, the DMA transfers one word from the
  *               capture/compare register to the memory.
  *
  *
  * @{
  */
hal_status_t HAL_TIM_IC_SetConfigChannel(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         const hal_tim_ic_channel_config_t *p_config);
void HAL_TIM_IC_GetConfigChannel(const hal_tim_handle_t *htim,
                                 hal_tim_channel_t channel,
                                 hal_tim_ic_channel_config_t *p_config);

hal_status_t HAL_TIM_IC_SetChannelSource(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         hal_tim_channel_src_t channel_src);
hal_tim_channel_src_t HAL_TIM_IC_GetChannelSource(const hal_tim_handle_t *htim,
                                                  hal_tim_channel_t channel);

hal_status_t HAL_TIM_IC_SetConfigCaptureUnit(hal_tim_handle_t *htim,
                                             hal_tim_ic_capture_unit_t capture_unit,
                                             const hal_tim_ic_capture_unit_config_t *p_config);
void HAL_TIM_IC_GetConfigCaptureUnit(const hal_tim_handle_t *htim,
                                     hal_tim_ic_capture_unit_t capture_unit,
                                     hal_tim_ic_capture_unit_config_t *p_config);

hal_status_t HAL_TIM_IC_SetXORGatePosition(hal_tim_handle_t *htim,
                                           hal_tim_ic_xor_gate_position_t xor_position);
hal_tim_ic_xor_gate_position_t HAL_TIM_IC_GetXORGatePosition(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_IC_EnableXORGateInputInversion(hal_tim_handle_t *htim, hal_tim_channel_t channel);
hal_status_t HAL_TIM_IC_DisableXORGateInputInversion(hal_tim_handle_t *htim, hal_tim_channel_t channel);
hal_tim_ic_xor_gate_input_inversion_status_t HAL_TIM_IC_IsEnabledXORGateInputInversion(const hal_tim_handle_t *htim,
    hal_tim_channel_t channel);

hal_status_t HAL_TIM_IC_EnableXORGate(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_IC_DisableXORGate(hal_tim_handle_t *htim);
hal_tim_ic_xor_gate_status_t HAL_TIM_IC_IsEnabledXORGate(const hal_tim_handle_t *htim);

uint32_t HAL_TIM_IC_ReadChannelCapturedValue(const hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel);

hal_tim_ic_channel_level_t HAL_TIM_IC_GetChannelLevel(const hal_tim_handle_t *htim,
                                                      hal_tim_channel_t channel);

hal_status_t HAL_TIM_IC_StartChannel(hal_tim_handle_t *htim,
                                     hal_tim_channel_t channel);
hal_status_t HAL_TIM_IC_StopChannel(hal_tim_handle_t *htim,
                                    hal_tim_channel_t channel);

hal_status_t HAL_TIM_IC_StartChannel_IT(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel);
hal_status_t HAL_TIM_IC_StopChannel_IT(hal_tim_handle_t *htim,
                                       hal_tim_channel_t channel);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_IC_StartChannel_DMA(hal_tim_handle_t *htim,
                                         hal_tim_channel_t channel,
                                         uint8_t *p_data,
                                         uint32_t size_byte);
hal_status_t HAL_TIM_IC_StartChannel_DMA_Opt(hal_tim_handle_t *htim,
                                             hal_tim_channel_t channel,
                                             uint8_t *p_data,
                                             uint32_t size_byte,
                                             uint32_t interrupts);
hal_status_t HAL_TIM_IC_StopChannel_DMA(hal_tim_handle_t *htim,
                                        hal_tim_channel_t channel);
#endif /* USE_HAL_TIM_DMA */


/**
  * @brief  Helper function to get a direct capture unit from an input channel.
  * @param  channel  Input channel. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *
  * @retval hal_tim_ic_capture_unit_t  Capture unit corresponding to the
  *         input channel.
  */
__STATIC_INLINE hal_tim_ic_capture_unit_t
hal_tim_ic_channel_to_direct_capture_unit(hal_tim_channel_t channel)
{
  const hal_tim_ic_capture_unit_t ch_to_dir_capt_unit[] =
  {
    HAL_TIM_IC_CAPTURE_UNIT_1,  /* HAL_TIM_CHANNEL_1  */
    HAL_TIM_IC_CAPTURE_UNIT_2,  /* HAL_TIM_CHANNEL_2  */
    HAL_TIM_IC_CAPTURE_UNIT_3,  /* HAL_TIM_CHANNEL_3  */
    HAL_TIM_IC_CAPTURE_UNIT_4,  /* HAL_TIM_CHANNEL_4  */
  };

  return ch_to_dir_capt_unit[channel];
}

/**
  * @brief  Helper function to get an indirect capture unit from an input channel.
  * @param  channel  Input channel. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *
  * @retval hal_tim_ic_capture_unit_t  Indirect capture unit corresponding to the
  *         input channel.
  */
__STATIC_INLINE hal_tim_ic_capture_unit_t
hal_tim_ic_channel_to_indirect_capture_unit(hal_tim_channel_t channel)
{
  const hal_tim_ic_capture_unit_t ch_to_ind_capt_unit[] =
  {
    HAL_TIM_IC_CAPTURE_UNIT_2,  /* HAL_TIM_CHANNEL_1  */
    HAL_TIM_IC_CAPTURE_UNIT_1,  /* HAL_TIM_CHANNEL_2  */
    HAL_TIM_IC_CAPTURE_UNIT_4,  /* HAL_TIM_CHANNEL_3  */
    HAL_TIM_IC_CAPTURE_UNIT_3,  /* HAL_TIM_CHANNEL_4  */
  };

  return ch_to_ind_capt_unit[channel];
}

/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group6 Timer One-Pulse functions
  * @{
  *            Functions in this group form the API for the control
  *            of the one-pulse mode (single pulse or repetitive).
  *
  *            The one-pulse mode of a timer can be used to generate a pulse
  *            with a programmable length after a programmable delay on trigger
  *            detection \n(hence, an input channel and an output channel have to
  *            be configured.)
  *
  *            The control functions to enable/disable the one-pulse mode.
  *
  *
  */
hal_status_t HAL_TIM_EnableOnePulseMode(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableOnePulseMode(hal_tim_handle_t *htim);
hal_tim_one_pulse_mode_status_t HAL_TIM_IsEnabledOnePulseMode(const hal_tim_handle_t *htim);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group7 Timer Encoder Index functions
  * @{
  *            Functions in this group form the API for the configuration
  *            and control of the encoder index.
  *
  *            Basically, the timer is used to determine a rotor speed/position
  *            when a quadrature encoder, connected to the timer's input channels 1
  *            and 2, is used as an external clock. \n
  *            The counter can be reset by an index signal coming from the encoder
  *            connected to the timer external trigger input (ETR).
  *
  *            The configuration of the encoder index:
  *             - specifies in which counter direction the index event resets the counter
  *             - specifies in which AB input configuration the index event resets the counter
  *               - gated with A and B (i.e. channel 1 and 2)
  *               - gated with A (or with B)
  *               - ungated
  *             - index blanking (the Index event can be blanked using TI3 or TI4 input) \n
  *               During the blanking window, the index events are no longer resetting the counter)
  *             - specifies if this index is active only the first time or always
  *
  *            The control functions to enable/disable the encoder index.
  *
  */
hal_status_t HAL_TIM_SetConfigEncoderIndex(hal_tim_handle_t *htim,
                                           const hal_tim_encoder_index_config_t *p_config);
void HAL_TIM_GetConfigEncoderIndex(const hal_tim_handle_t *htim,
                                   hal_tim_encoder_index_config_t *p_config);

hal_status_t HAL_TIM_EnableEncoderIndex(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableEncoderIndex(hal_tim_handle_t *htim);
hal_tim_encoder_index_status_t HAL_TIM_IsEnabledEncoderIndex(const hal_tim_handle_t *htim);

/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group8 Timer External Trigger configuration
  * @{
  *            Functions in this group can be used to configure the external trigger input.
  *
  *            The configuration of the external trigger (ETR) input implies:
  *             -  selection the input source for the ETR
  *             -  selection the input polarity
  *             -  setting of the prescaler
  *             -  setting of the filter
  *
  *
  */
hal_status_t HAL_TIM_SetExternalTriggerInput(hal_tim_handle_t *htim,
                                             const hal_tim_ext_trig_config_t *p_config);
void HAL_TIM_GetExternalTriggerInput(const hal_tim_handle_t *htim,
                                     hal_tim_ext_trig_config_t *p_config);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group9 Timer Master/Slave functions
  * @{
  *            Functions in this group form the API for the configuration
  *            and control of a timer in master or/and slave mode
  *            (a timer instance can act as both a master and a slave).
  *
  *            This interface is typically used when one wants to cascade timer
  *            instances or to synchronize a timer with the DAC or the ADC.
  *
  *            The slave configuration sets:
  *             - the slave mode
  *             - the trigger input to be used to synchronize the counter
  *
  *            The master configuration sets:
  *             - the trigger output (TRGO and if supported TRGO2)
  *
  *            The control enables/disables:
  *             - the master/slave mode \n
  *               When enabled, the effect of an event on the trigger input (trgi)
  *               is delayed to allow a perfect synchronization between the
  *               current timer and its slaves (through trgo). \n It is useful if
  *               we want to synchronize several timers on a single external event.
  *             - the slave mode preload \n
  *               When enabled, the transfer from SMS[3:0] preload to active
  *               value is triggered either by the timer's Update event or
  *               Index event.
  *
  *
  */
hal_status_t HAL_TIM_SetSynchroSlave(hal_tim_handle_t *htim,
                                     const hal_tim_slave_config_t *p_config);
void HAL_TIM_GetSynchroSlave(const hal_tim_handle_t *htim,
                             hal_tim_slave_config_t *p_config);

hal_status_t HAL_TIM_SetTriggerOutput(hal_tim_handle_t *htim,
                                      hal_tim_trigger_output_source_t trgo_src);
hal_tim_trigger_output_source_t
HAL_TIM_GetTriggerOutput(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetConfigTriggerOutput2(hal_tim_handle_t *htim,
                                             hal_tim_trigger_output2_config_t *p_config);
void HAL_TIM_GetConfigTriggerOutput2(const hal_tim_handle_t *htim,
                                     hal_tim_trigger_output2_config_t *p_config);

hal_status_t HAL_TIM_SetTriggerOutput2(hal_tim_handle_t *htim,
                                       hal_tim_trigger_output2_source_t trgo2_src);
hal_tim_trigger_output2_source_t
HAL_TIM_GetTriggerOutput2(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_SetTriggerOutput2Postscaler(hal_tim_handle_t *htim,
                                                 uint32_t postscaler);
uint32_t HAL_TIM_GetTriggerOutput2Postscaler(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableSlaveModePreload(hal_tim_handle_t *htim,
                                            const hal_tim_slave_mode_preload_src_t preload_src);
hal_status_t HAL_TIM_DisableSlaveModePreload(hal_tim_handle_t *htim);
hal_tim_slave_mode_preload_status_t HAL_TIM_IsEnabledSlaveModePreload(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableMasterSlaveMode(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableMasterSlaveMode(hal_tim_handle_t *htim);
hal_tim_master_slave_mode_status_t HAL_TIM_IsEnabledMasterSlaveMode(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableADCSynchronization(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableADCSynchronization(hal_tim_handle_t *htim);
hal_tim_adc_synchronization_status_t HAL_TIM_IsEnabledADCSynchronization(const hal_tim_handle_t *htim);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group10 Timer OCRef Clear functions
  * @{
  *            Functions in this group configure and control a timer's feature
  *            known as "OCRef Clear". \n
  *            This feature is used to force output compare signals to active
  *            or inactive level independently of any comparison between the
  *            output compare register and the counter. \n
  *            The source of the OCRef clear can be external trigger
  *            or an internal signal (e.g. comparator output).
  *            The configuration consist of:
  *             - selection of the source:
  *               - external (ETR)
  *               - internal (COMP1 or COMP2)
  *            The control enables/disables the "clear" function for a given
  *            channel.
  *
  */
hal_status_t HAL_TIM_SetOCRefClearSource(hal_tim_handle_t *htim,
                                         hal_tim_ocref_clr_src_t source);
hal_tim_ocref_clr_src_t HAL_TIM_GetOCRefClearSource(const hal_tim_handle_t *htim);


hal_status_t HAL_TIM_EnableCompareUnitOCRefClear(hal_tim_handle_t *htim,
                                                 hal_tim_oc_compare_unit_t compare_unit);
hal_status_t HAL_TIM_DisableCompareUnitOCRefClear(hal_tim_handle_t *htim,
                                                  hal_tim_oc_compare_unit_t compare_unit);
hal_tim_ocref_clr_status_t HAL_TIM_IsEnabledCompareUnitOCRefClear(const hal_tim_handle_t *htim,
                                                                  hal_tim_oc_compare_unit_t compare_unit);
/**
  *  @}
  */

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
/** @defgroup TIM_Exported_Functions_Group11 Timer DMA Burst functions
  * @{
  *            Functions in this group configure and control a timer's feature
  *            known as "DMA burst".
  *
  *            The two main usages of this feature are:
  *            - DMA burst write:
  *              re-program part of the timer peripheral each time a given
  *              timer's event is triggered (e.g. modification on the fly of an
  *              output waveform).
  *            - DMA burts read:
  *              read several registers in a row, at regular intervals.
  *
  *            When a DMA burst operation is configured, the application must
  *            provide the DMA burst configuration which consist of:
  *            - Selection of the DMA burst start address
  *            - Length of the DMA burst
  *            - Selection of the DMA burst source (triggering event )
  *
  *            When a DMA burst operation is started, the application must
  *            provide:
  *            - Direction of the DMA burst (read or write)
  *            - Pointer to the data buffer
  *            - Size (in bytes) of the DMA transfer
  *
  *            The control consist of starting/stopping a DMA burst read/write.
  *
  */
hal_status_t HAL_TIM_SetConfigDMABurst(hal_tim_handle_t *htim,
                                       hal_tim_dmaburst_config_t *p_config);
void HAL_TIM_GetConfigDMABurst(const hal_tim_handle_t *htim,
                               hal_tim_dmaburst_config_t *p_config);

hal_status_t HAL_TIM_StartDMABurst(hal_tim_handle_t *htim,
                                   hal_tim_dmaburst_direction_t dmaburst_direction,
                                   const uint8_t *p_data,
                                   uint32_t size_byte);
hal_status_t HAL_TIM_StopDMABurst(hal_tim_handle_t *htim);

/**
  * @brief  Helper function to get the DMA burst base address register for a given channel.
  * @param  channel  Channel. \n
  *                  Must be one of the following values: \n
  *                  @arg @ref HAL_TIM_CHANNEL_1
  *                  @arg @ref HAL_TIM_CHANNEL_2
  *                  @arg @ref HAL_TIM_CHANNEL_3
  *                  @arg @ref HAL_TIM_CHANNEL_4
  *                  @arg @ref HAL_TIM_CHANNEL_5
  *                  @arg @ref HAL_TIM_CHANNEL_6
  *                  @arg @ref HAL_TIM_CHANNEL_7
  *                  @arg @ref HAL_TIM_CHANNEL_1N
  *                  @arg @ref HAL_TIM_CHANNEL_2N
  *                  @arg @ref HAL_TIM_CHANNEL_3N
  *                  @arg @ref HAL_TIM_CHANNEL_4N
  * @retval hal_tim_dmaburst_base_addr_reg_t  DMA burst base address register
  *         corresponding to the channel.
  */
__STATIC_INLINE hal_tim_dmaburst_base_addr_reg_t
hal_tim_channel_to_dmaburst_base_address(hal_tim_channel_t channel)
{
  const hal_tim_dmaburst_base_addr_reg_t ch_to_dmaburst_base_addr[] =
  {
    HAL_TIM_DMABURST_BASE_ADDR_CCR1,  /* HAL_TIM_CHANNEL_1  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR2,  /* HAL_TIM_CHANNEL_2  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR3,  /* HAL_TIM_CHANNEL_3  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR4,  /* HAL_TIM_CHANNEL_4  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR5,  /* HAL_TIM_CHANNEL_5  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR6,  /* HAL_TIM_CHANNEL_6  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR7,  /* HAL_TIM_CHANNEL_7  */
    HAL_TIM_DMABURST_BASE_ADDR_CCR1,  /* HAL_TIM_CHANNEL_1N */
    HAL_TIM_DMABURST_BASE_ADDR_CCR2,  /* HAL_TIM_CHANNEL_2N */
    HAL_TIM_DMABURST_BASE_ADDR_CCR3,  /* HAL_TIM_CHANNEL_3N */
    HAL_TIM_DMABURST_BASE_ADDR_CCR4,  /* HAL_TIM_CHANNEL_4N */
  };

  return ch_to_dmaburst_base_addr[channel];
}

/**
  *  @}
  */
#endif /* USE_HAL_TIM_DMA */

/** @defgroup TIM_Exported_Functions_Group12 Timer Break functions
  * @{
  *            Functions in this group configure and control the "break"
  *            feature that is present on certain timers that feature
  *            complementary outputs. \n
  *
  *            The Break utility acts on the output stage of timer channels
  *            configured in output mode. \n
  *            As soon as an active edge is detected on the break input,
  *            the outputs of timer channels configured in output mode are
  *            either turned off or forced to a predefined safe state.
  *
  *            Use the configuration API to:
  *             - for a break input (BKIN or BKIN2)
  *               - set the polarity
  *               - set the filter
  *               - set the alternate function
  *             - for a break input source
  *               - set the polarity
  *             - set the off state selection in idle and in run mode
  *
  *            Use the control API to:
  *             - enable/disable a break input (BKIN or BKIN2)
  *             - rearm a break input (BKIN or BKIN2) when it is configured
  *               in bi-directional mode
  *             - enable/disable a break input source
  *             - enable/disable the main output \n
  *               Control all outputs
  *             - enable/disable the automatic output \n
  *               Specify whether the main output can only be enabled by software
  *               or if it can be automatically re-enabled by hardware at next
  *               update event following the break condition disappearance
  *
  *
  */
hal_status_t HAL_TIM_BREAK_SetConfigInput(hal_tim_handle_t *htim,
                                          hal_tim_break_input_id_t brkin,
                                          const hal_tim_break_input_config_t *p_config);
void HAL_TIM_BREAK_GetConfigInput(const hal_tim_handle_t *htim,
                                  hal_tim_break_input_id_t brkin,
                                  hal_tim_break_input_config_t *p_config);

hal_status_t HAL_TIM_BREAK_SetInputPolarity(hal_tim_handle_t *htim,
                                            hal_tim_break_input_id_t brkin,
                                            hal_tim_break_input_polarity_t polarity);
hal_tim_break_input_polarity_t HAL_TIM_BREAK_GetInputPolarity(const hal_tim_handle_t *htim,
                                                              hal_tim_break_input_id_t brkin);

hal_status_t HAL_TIM_BREAK_SetInputFilter(hal_tim_handle_t *htim,
                                          hal_tim_break_input_id_t brkin,
                                          hal_tim_filter_t filter);
hal_tim_filter_t HAL_TIM_BREAK_GetInputFilter(const hal_tim_handle_t *htim,
                                              hal_tim_break_input_id_t brkin);

hal_status_t HAL_TIM_BREAK_SetInputMode(hal_tim_handle_t *htim,
                                        hal_tim_break_input_id_t brkin,
                                        hal_tim_break_input_mode_t mode);
hal_tim_break_input_mode_t HAL_TIM_BREAK_GetInputMode(const hal_tim_handle_t *htim,
                                                      hal_tim_break_input_id_t brkin);

hal_status_t HAL_TIM_BREAK_EnableInput(hal_tim_handle_t *htim,
                                       hal_tim_break_input_id_t brkin);
hal_status_t HAL_TIM_BREAK_DisableInput(hal_tim_handle_t *htim,
                                        hal_tim_break_input_id_t brkin);
hal_tim_break_input_status_t HAL_TIM_BREAK_IsEnabledInput(const hal_tim_handle_t *htim,
                                                          hal_tim_break_input_id_t brkin);

hal_status_t HAL_TIM_BREAK_RearmInput(hal_tim_handle_t *htim,
                                      hal_tim_break_input_id_t brkin);

hal_status_t HAL_TIM_BREAK_SetInputSourcePolarity(hal_tim_handle_t *htim,
                                                  hal_tim_break_input_id_t brkin,
                                                  uint32_t brkinsrc,
                                                  hal_tim_break_input_src_polarity_t polarity);
hal_tim_break_input_src_polarity_t HAL_TIM_BREAK_GetInputSourcePolarity(const hal_tim_handle_t *htim,
                                                                        hal_tim_break_input_id_t brkin,
                                                                        uint32_t brkinsrc);

hal_status_t HAL_TIM_BREAK_EnableInputSource(hal_tim_handle_t *htim,
                                             hal_tim_break_input_id_t brkin,
                                             uint32_t brkinsrc);
hal_status_t HAL_TIM_BREAK_DisableInputSource(hal_tim_handle_t *htim,
                                              hal_tim_break_input_id_t brkin,
                                              uint32_t brkinsrc);
hal_tim_break_input_src_status_t HAL_TIM_BREAK_IsEnabledInputSource(const hal_tim_handle_t *htim,
                                                                    hal_tim_break_input_id_t brkin,
                                                                    uint32_t brkinsrc);

hal_status_t HAL_TIM_BREAK_EnableMainOutput(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_BREAK_DisableMainOutput(hal_tim_handle_t *htim);
hal_tim_break_main_output_status_t HAL_TIM_BREAK_IsEnabledMainOutput(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_BREAK_EnableAutomaticOutput(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_BREAK_DisableAutomaticOutput(hal_tim_handle_t *htim);
hal_tim_break_automatic_output_status_t HAL_TIM_BREAK_IsEnabledAutomaticOutput(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_BREAK_SetBreakDelay(hal_tim_handle_t *htim,
                                         hal_tim_break_delay_t break_delay,
                                         uint32_t delay);
uint32_t HAL_TIM_BREAK_GetBreakDelay(const hal_tim_handle_t *htim,
                                     hal_tim_break_delay_t break_delay);

hal_status_t HAL_TIM_BREAK_SetOutputOffStates(hal_tim_handle_t *htim,
                                              const hal_tim_off_states_config_t *p_config);
void HAL_TIM_BREAK_GetOutputOffStates(const hal_tim_handle_t *htim,
                                      hal_tim_off_states_config_t *p_config);

hal_tim_output_disable_status_t HAL_TIM_BREAK_GetOutputDisableStatus(const hal_tim_handle_t *htim);

/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group13 Timer Deadtime functions
  * @{
  *            Functions in this group configure and control the "deadtime"
  *            feature.
  *
  *            Dead-time manages the switching-off and the switching-on instants
  *            of the outputs of two complementary signals.
  *
  *            Configuration:
  *            - set the deadtime for the rising edge
  *            - set the deadtime for the falling edge
  *
  *            Control:
  *            - When enabled, the transfer from DTG(F) preload to active value is
  *              triggered  by the timer's Update event.
  *
  *
  */
hal_status_t HAL_TIM_SetDeadtime(hal_tim_handle_t *htim,
                                 uint32_t rising_edge_deadtime,
                                 uint32_t falling_edge_deadtime);
void HAL_TIM_GetDeadtime(const hal_tim_handle_t *htim,
                         uint32_t *p_rising_edge_deadtime,
                         uint32_t *p_falling_edge_deadtime);

hal_status_t HAL_TIM_EnableDeadtimePreload(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableDeadtimePreload(hal_tim_handle_t *htim);
hal_tim_deadtime_preload_status_t HAL_TIM_IsEnabledDeadtimePreload(const hal_tim_handle_t *htim);

hal_status_t HAL_TIM_EnableAsymmetricalDeadtime(hal_tim_handle_t *htim);
hal_status_t HAL_TIM_DisableAsymmetricalDeadtime(hal_tim_handle_t *htim);
hal_tim_asymmetrical_deadtime_status_t HAL_TIM_IsEnabledAsymmetricalDeadtime(const hal_tim_handle_t *htim);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group14 Timer Protection
  * @{
  *            This group contains only two functions that are the setter and
  *            getter of the lock level.
  *
  *            The lock feature is a write protection that freezes the
  *            configuration of safety critical parameters (e.g. break inputs
  *            and deadtime configuration).
  *
  */
hal_status_t HAL_TIM_SetLockLevel(hal_tim_handle_t *htim,
                                  hal_tim_lock_level_t lock_level);
hal_tim_lock_level_t HAL_TIM_GetLockLevel(const hal_tim_handle_t *htim);
/**
  *  @}
  */

/** @defgroup TIM_Exported_Functions_Group15 Timer Commutation control
  * @{
  *            This group contains functions required to manage the commutation
  *            feature. \n
  *            The commutation feature is used in combination with the preload
  *            mechanism to change the timer channel configuration (e.g. output
  *            mode, channel enabled/disabled) \n in perfect synchronization with
  *            timer external events generated by software or through a trigger
  *            input (internal (ITRx) or external (ETR, TI1, or TI2)). \n
  *            \n
  *            When the commutation feature is enabled, capture/compare control
  *            bits (CCxE, CCxNE and OCxM) are preloaded and \n the commutation
  *            event trigger is set as per application's choice (software and/or
  *            trigger input).
  *
  */
hal_status_t HAL_TIM_EnableCommutation(hal_tim_handle_t *htim,
                                       hal_tim_commutation_src_t commutation_source);
hal_status_t HAL_TIM_DisableCommutation(hal_tim_handle_t *htim);
hal_tim_commutation_status_t HAL_TIM_IsEnabledCommutation(const hal_tim_handle_t *htim);

hal_tim_commutation_src_t HAL_TIM_GetCommutationSource(const hal_tim_handle_t *htim);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group16 Timer SW Event Generation
  * @{
  *            This group contains only one function that is used to control,
  *            by software, the generation of a timer's event.
  *
  */
hal_status_t HAL_TIM_GenerateEvent(hal_tim_handle_t *htim,
                                   hal_tim_sw_event_id_t sw_event_id);
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group17 Timer IRQ Handler and Callbacks functions
  * @{
  */

/* Interrupt Handler functions   **************************************************************************************/
void HAL_TIM_IRQHandler(hal_tim_handle_t *htim);
void HAL_TIM_UPD_IRQHandler(hal_tim_handle_t *htim);
void HAL_TIM_CC_IRQHandler(hal_tim_handle_t *htim);
void HAL_TIM_BRK_TERR_IERR_IRQHandler(hal_tim_handle_t *htim);
void HAL_TIM_TRGI_COM_DIR_IDX_IRQHandler(hal_tim_handle_t *htim);

/* Declaration for the weak callbacks *********************************************************************************/
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_ErrorCallback(hal_tim_handle_t *htim);
void HAL_TIM_StopCallback(hal_tim_handle_t *htim);
void HAL_TIM_ChannelStopCallback(hal_tim_handle_t *htim,
                                 hal_tim_channel_t channel);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_UpdateCallback(hal_tim_handle_t *htim);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_UpdateHalfCpltCallback(hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_TriggerCallback(hal_tim_handle_t *htim);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_TriggerHalfCpltCallback(hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_InputCaptureCallback(hal_tim_handle_t *htim,
                                  hal_tim_channel_t channel);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_InputCaptureHalfCpltCallback(hal_tim_handle_t *htim,
                                          hal_tim_channel_t channel);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_CompareMatchCallback(hal_tim_handle_t *htim,
                                  hal_tim_channel_t channel);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_CompareMatchHalfCpltCallback(hal_tim_handle_t *htim,
                                          hal_tim_channel_t channel);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_CommutationCallback(hal_tim_handle_t *htim);

#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
void HAL_TIM_CommutationHalfCpltCallback(hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_DMA */

void HAL_TIM_BreakCallback(hal_tim_handle_t *htim);
void HAL_TIM_Break2Callback(hal_tim_handle_t *htim);
void HAL_TIM_SystemBreakCallback(hal_tim_handle_t *htim);
void HAL_TIM_SoftwareBreakCallback(hal_tim_handle_t *htim,
                                   hal_tim_break_input_id_t bkin);

void HAL_TIM_EncoderIndexCallback(hal_tim_handle_t *htim);
void HAL_TIM_DirectionChangeCallback(hal_tim_handle_t *htim);
void HAL_TIM_IndexErrorCallback(hal_tim_handle_t *htim);
void HAL_TIM_TransitionErrorCallback(hal_tim_handle_t *htim);

/* Interfaces for registering callbacks *******************************************************************************/
#if defined(USE_HAL_TIM_REGISTER_CALLBACKS) && (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterErrorCallback(hal_tim_handle_t *htim,
                                           hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterStopCallback(hal_tim_handle_t *htim,
                                          hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterChannelStopCallback(hal_tim_handle_t *htim,
                                                 hal_tim_channel_cb_t fct);
#endif /* USE_HAL_TIM_DMA */

hal_status_t HAL_TIM_RegisterUpdateCallback(hal_tim_handle_t *htim,
                                            hal_tim_cb_t fct);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterUpdateHalfCpltCallback(hal_tim_handle_t *htim,
                                                    hal_tim_cb_t fct);
#endif /* USE_HAL_TIM_DMA */
hal_status_t HAL_TIM_RegisterTriggerCallback(hal_tim_handle_t *htim,
                                             hal_tim_cb_t fct);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterTriggerHalfCpltCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct);
#endif /* USE_HAL_TIM_DMA */
hal_status_t HAL_TIM_RegisterInputCaptureCallback(hal_tim_handle_t *htim,
                                                  hal_tim_channel_cb_t fct);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterInputCaptureHalfCpltCallback(hal_tim_handle_t *htim,
                                                          hal_tim_channel_cb_t fct);
#endif /* USE_HAL_TIM_DMA */
hal_status_t HAL_TIM_RegisterCompareMatchCallback(hal_tim_handle_t *htim,
                                                  hal_tim_channel_cb_t fct);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterCompareMatchHalfCpltCallback(hal_tim_handle_t *htim,
                                                          hal_tim_channel_cb_t fct);
#endif /* USE_HAL_TIM_DMA */
hal_status_t HAL_TIM_RegisterCommutationCallback(hal_tim_handle_t *htim,
                                                 hal_tim_cb_t fct);
#if defined(USE_HAL_TIM_DMA) && (USE_HAL_TIM_DMA == 1)
hal_status_t HAL_TIM_RegisterCommutationHalfCpltCallback(hal_tim_handle_t *htim,
                                                         hal_tim_cb_t fct);
#endif /* USE_HAL_TIM_DMA */
hal_status_t HAL_TIM_RegisterBreakCallback(hal_tim_handle_t *htim,
                                           hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterBreak2Callback(hal_tim_handle_t *htim,
                                            hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterSystemBreakCallback(hal_tim_handle_t *htim,
                                                 hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterSoftwareBreakCallback(hal_tim_handle_t *htim,
                                                   hal_tim_break_cb_t fct);

hal_status_t HAL_TIM_RegisterEncoderIndexCallback(hal_tim_handle_t *htim,
                                                  hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterDirectionChangeCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterIndexErrorCallback(hal_tim_handle_t *htim,
                                                hal_tim_cb_t fct);
hal_status_t HAL_TIM_RegisterTransitionErrorCallback(hal_tim_handle_t *htim,
                                                     hal_tim_cb_t fct);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
/**
  *  @}
  */


/** @defgroup TIM_Exported_Functions_Group18 Timer Setter and Getter of the user data
  * @{
  */
#if defined (USE_HAL_TIM_USER_DATA) && (USE_HAL_TIM_USER_DATA == 1)
void HAL_TIM_SetUserData(hal_tim_handle_t *htim, const void *p_user_data);
const void *HAL_TIM_GetUserData(const hal_tim_handle_t *htim);
#endif /* USE_HAL_TIM_USER_DATA */
/**
  *  @}
  */

/**
  *  @}
  */

/**
  *  @}
  */

#endif /* TIM1 || TIM2 || TIM3 || TIM4 || TIM5 || TIM6 || TIM7 || TIM8 || TIM12 || TIM15 || TIM16 || TIM17 */
/**
  *  @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C5XX_HAL_TIM_H */
