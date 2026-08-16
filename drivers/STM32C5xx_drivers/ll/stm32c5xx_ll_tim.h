/**
  **********************************************************************************************************************
  * @file    stm32c5xx_ll_tim.h
  * @brief   Header file for the TIM LL module.
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
#ifndef STM32C5XX_LL_TIM_H
#define STM32C5XX_LL_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
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

/** @defgroup TIM_LL TIM
  * @{
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Variables TIM Private Variables
  * @{
  */
static const uint8_t LL_TIM_OFFSET_TAB_CCMRx[] =
{
  0x00U   /* 0: TIMx_CH1  */
  , 0x00U   /* 1: TIMx_CH1N */
  , 0x00U   /* 2: TIMx_CH2  */
  , 0x00U   /* 3: TIMx_CH2N */
  , 0x04U   /* 4: TIMx_CH3  */
  , 0x04U   /* 5: TIMx_CH3N */
  , 0x04U   /* 6: TIMx_CH4  */
  , 0x04U   /* 7: TIMx_CH4N */
  , 0x38U   /* 8: TIMx_CH5  */
  , 0x38U    /* 9: TIMx_CH6  */
  , 0x60U    /* 10: TIMx_CH7 */
};

static const uint8_t LL_TIM_OFFSET_TAB_CCRx[] =
{
  0x00U             /* 0: CCR1 */
  , 0x04U           /* 1: CCR2 */
  , 0x08U           /* 2: CCR3 */
  , 0x0CU           /* 3: CCR4 */
  , 0x14U           /* 4: CCR5 */
  , (0x14U + 0x4U)  /* 5: CCR6 */
  , 0x3CU   /* 6: CCR7 */
};

static const uint8_t LL_TIM_SHIFT_TAB_OCxx[] =
{
  0U             /* 0: OC1M, OC1FE, OC1PE  */
  , 0U           /* 1: - NA */
  , 8U           /* 2: OC2M, OC2FE, OC2PE  */
  , 0U           /* 3: - NA */
  , 0U           /* 4: OC3M, OC3FE, OC3PE  */
  , 0U           /* 5: - NA */
  , 8U           /* 6: OC4M, OC4FE, OC4PE  */
  , 0U           /* 7: - NA */
  , 0U           /* 8: OC5M, OC5FE, OC5PE  */
  , 8U           /* 9: OC6M, OC6FE, OC6PE  */
  , 0U           /* 10: OC7M, OC7FE, OC7PE */
};

static const uint8_t LL_TIM_SHIFT_TAB_ICxx[] =
{
  0U             /* 0: CC1S, IC1PSC, IC1F */
  , 0U           /* 1: - NA */
  , 8U           /* 2: CC2S, IC2PSC, IC2F */
  , 0U           /* 3: - NA */
  , 0U           /* 4: CC3S, IC3PSC, IC3F */
  , 0U           /* 5: - NA */
  , 8U           /* 6: CC4S, IC4PSC, IC4F */
  , 0U           /* 7: - NA */
  , 0U           /* 8: - NA */
  , 0U           /* 9: - NA */
  , 0U           /* 10: - NA */
};

static const uint8_t LL_TIM_SHIFT_TAB_CCxP[] =
{
  0U             /* 0: CC1P  */
  , 2U           /* 1: CC1NP */
  , 4U           /* 2: CC2P  */
  , 6U           /* 3: CC2NP */
  , 8U           /* 4: CC3P  */
  , 10U          /* 5: CC3NP */
  , 12U          /* 6: CC4P  */
  , 14U          /* 7: CC4NP */
  , 16U          /* 8: CC5P  */
  , 20U          /* 9: CC6P  */
  , 24U          /* 10: CC7P */
};

static const uint8_t LL_TIM_SHIFT_TAB_OISx[] =
{
  0U             /* 0: OIS1  */
  , 1U           /* 1: OIS1N */
  , 2U           /* 2: OIS2  */
  , 3U           /* 3: OIS2N */
  , 4U           /* 4: OIS3  */
  , 5U           /* 5: OIS3N */
  , 6U           /* 6: OIS4  */
  , 7U           /* 7: OIS4N */
  , 8U           /* 8: OIS5  */
  , 10U          /* 9: OIS6  */
  , 11U          /* 10: OIS7 */
};

static const uint32_t LL_TIM_MASK_TAB_BKxE[] =
{
  TIM_BDTR_BKE,  /* 0: BKIN */
  TIM_BDTR_BK2E  /* 1: BKIN2 */
};

static const uint32_t LL_TIM_MASK_TAB_BKxP[] =
{
  TIM_BDTR_BKP,  /* 0: BKIN */
  TIM_BDTR_BK2P  /* 1: BKIN2 */
};

static const uint32_t LL_TIM_MASK_TAB_BKxF[] =
{
  TIM_BDTR_BKF,  /* 0: BKIN */
  TIM_BDTR_BK2F  /* 1: BKIN2 */
};

static const uint32_t LL_TIM_MASK_TAB_BKxBID[] =
{
  TIM_BDTR_BKBID,  /* 0: BKIN */
  TIM_BDTR_BK2BID  /* 1: BKIN2 */
};

/* Shift for IC config */
#define LL_TIM_IC_CONFIG_POS  (16U)
/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Constants TIM Private Constants
  * @{
  */


#define TIM_CCR5_GC5X  (TIM_CCR5_GC5C3 | TIM_CCR5_GC5C2 | TIM_CCR5_GC5C1 | TIM_CCR5_GC5C4 | \
                        TIM_CCR5_GC5C4O | TIM_CCR5_GC5C3O | TIM_CCR5_GC5C2O | TIM_CCR5_GC5C1O)

/* Mask used to set the TDG[x:0] of the DTG bits of the TIMx_BDTR register */
#define LL_TIM_DT_DELAY_1 ((uint8_t)0x7F)
#define LL_TIM_DT_DELAY_2 ((uint8_t)0x3F)
#define LL_TIM_DT_DELAY_3 ((uint8_t)0x1F)
#define LL_TIM_DT_DELAY_4 ((uint8_t)0x1F)

/* Mask used to set the DTG[7:5] bits of the DTG bits of the TIMx_BDTR register */
#define LL_TIM_DT_RANGE_1 ((uint8_t)0x00)
#define LL_TIM_DT_RANGE_2 ((uint8_t)0x80)
#define LL_TIM_DT_RANGE_3 ((uint8_t)0xC0)
#define LL_TIM_DT_RANGE_4 ((uint8_t)0xE0)

/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Private_Macros TIM Private Macros
  * @{
  */

/* Defines used for the bit position in the register and perform offsets */
#define LL_TIM_POSITION_BRK_SOURCE(source)  \
  (STM32_POSITION_VAL((source)) & 0x1FUL)

/** @brief  Convert channel id into channel index.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval Channel index.
  */
#define LL_TIM_GET_CHANNEL_INDEX(channel) (((channel) == LL_TIM_CHANNEL_CH1) ? 0U  :\
                                           ((channel) == LL_TIM_CHANNEL_CH1N) ? 1U :\
                                           ((channel) == LL_TIM_CHANNEL_CH2) ? 2U  :\
                                           ((channel) == LL_TIM_CHANNEL_CH2N) ? 3U :\
                                           ((channel) == LL_TIM_CHANNEL_CH3) ? 4U  :\
                                           ((channel) == LL_TIM_CHANNEL_CH3N) ? 5U :\
                                           ((channel) == LL_TIM_CHANNEL_CH4) ? 6U  :\
                                           ((channel) == LL_TIM_CHANNEL_CH4N) ? 7U :\
                                           ((channel) == LL_TIM_CHANNEL_CH5) ? 8U  :\
                                           ((channel) == LL_TIM_CHANNEL_CH6) ? 9U  : 10U)

/** @brief  Calculate the deadtime sampling period (in ps).
  * @param  tim_clk timer input clock frequency (in Hz).
  * @param  clk_div This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV8
  * @retval Deadtime sampling period (in ps).
  */
#define LL_TIM_CALC_DTS(tim_clk, clk_div) (((clk_div) == LL_TIM_CLOCKDIVISION_DIV1) ? \
                                           ((uint64_t)1000000000000U/(tim_clk)) : \
                                           ((clk_div) == LL_TIM_CLOCKDIVISION_DIV2) ? \
                                           ((uint64_t)1000000000000U/((tim_clk) >> 1U)) : \
                                           ((clk_div) == LL_TIM_CLOCKDIVISION_DIV4) ? \
                                           ((uint64_t)1000000000000U/((tim_clk) >> 2U)) : \
                                           ((uint64_t)1000000000000U/((tim_clk) >> 3U)))

/** Legacy definitions for compatibility purpose
  * @cond LEGACY_DEFINITIONS
  */
#define LL_TIM_TIM_POSITION_BRK_SOURCE(source)  LL_TIM_POSITION_BRK_SOURCE(source)
#define LL_TIM_TIM_GET_CHANNEL_INDEX(channel)  LL_TIM_GET_CHANNEL_INDEX(channel)
#define LL_TIM_TIM_CALC_DTS(tim_clk, clk_div)  LL_TIM_CALC_DTS(tim_clk, clk_div)
/**
  * @endcond
  */
/**
  * @}
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Constants LL TIM Constants
  * @{
  */

/** @defgroup TIM_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_TIM_READ_REG function.
  * @{
  */
#define LL_TIM_SR_UIF              TIM_SR_UIF              /*!< Update interrupt flag */
#define LL_TIM_SR_CC1IF            TIM_SR_CC1IF            /*!< Capture/compare 1 interrupt flag */
#define LL_TIM_SR_CC2IF            TIM_SR_CC2IF            /*!< Capture/compare 2 interrupt flag */
#define LL_TIM_SR_CC3IF            TIM_SR_CC3IF            /*!< Capture/compare 3 interrupt flag */
#define LL_TIM_SR_CC4IF            TIM_SR_CC4IF            /*!< Capture/compare 4 interrupt flag */
#define LL_TIM_SR_CC5IF            TIM_SR_CC5IF            /*!< Capture/compare 5 interrupt flag */
#define LL_TIM_SR_CC6IF            TIM_SR_CC6IF            /*!< Capture/compare 6 interrupt flag */
#define LL_TIM_SR_CC7IF            TIM_SR_CC7IF            /*!< Capture/compare 7 interrupt flag */
#define LL_TIM_SR_COMIF            TIM_SR_COMIF            /*!< COM interrupt flag */
#define LL_TIM_SR_TIF              TIM_SR_TIF              /*!< Trigger interrupt flag */
#define LL_TIM_SR_BIF              TIM_SR_BIF              /*!< Break interrupt flag */
#define LL_TIM_SR_B2IF             TIM_SR_B2IF             /*!< Second break interrupt flag */
#define LL_TIM_SR_SBIF             TIM_SR_SBIF             /*!< System Break interrupt flag  */
#define LL_TIM_SR_BGF              TIM_SR_BGF              /*!< Break Generation flag  */
#define LL_TIM_SR_B2GF             TIM_SR_B2GF             /*!< Break2 Generation flag  */
#define LL_TIM_SR_CC1OF            TIM_SR_CC1OF            /*!< Capture/Compare 1 overcapture flag */
#define LL_TIM_SR_CC2OF            TIM_SR_CC2OF            /*!< Capture/Compare 2 overcapture flag */
#define LL_TIM_SR_CC3OF            TIM_SR_CC3OF            /*!< Capture/Compare 3 overcapture flag */
#define LL_TIM_SR_CC4OF            TIM_SR_CC4OF            /*!< Capture/Compare 4 overcapture flag */
#define LL_TIM_SR_IDXF             TIM_SR_IDXF             /*!< Index interrupt flag  */
#define LL_TIM_SR_DIRF             TIM_SR_DIRF             /*!< Direction Change interrupt flag  */
#define LL_TIM_SR_IERRF            TIM_SR_IERRF            /*!< Index Error flag  */
#define LL_TIM_SR_TERRF            TIM_SR_TERRF            /*!< Transition Error flag  */
#define LL_TIM_SR_UIOVRF           TIM_SR_UIOVRF           /*!< Update interrupt overrun flag */
#define LL_TIM_SR_ODS              TIM_SR_ODS              /*!< Output disable status */
#define LL_TIM_SR_TI1FS            TIM_SR_TI1FS            /*!< Capture 1 signal status */
#define LL_TIM_SR_TI2FS            TIM_SR_TI2FS            /*!< Capture 2 signal status */
#define LL_TIM_SR_TI3FS            TIM_SR_TI3FS            /*!< Capture 3 signal status */
#define LL_TIM_SR_TI4FS            TIM_SR_TI4FS            /*!< Capture 4 signal status */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_ENABLE Break Enable
  * @{
  */
#define LL_TIM_BREAK_DISABLE            0x00000000U        /*!< Break function disabled */
#define LL_TIM_BREAK_ENABLE             TIM_BDTR_BKE       /*!< Break function enabled */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_ENABLE Break2 Enable
  * @{
  */
#define LL_TIM_BREAK2_DISABLE            0x00000000U       /*!< Break2 function disabled */
#define LL_TIM_BREAK2_ENABLE             TIM_BDTR_BK2E     /*!< Break2 function enabled */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_AUTOMATICOUTPUT_ENABLE Automatic output enable
  * @{
  */
#define LL_TIM_AUTOMATICOUTPUT_DISABLE      0x00000000U    /*!< MOE can be set only by software */
#define LL_TIM_AUTOMATICOUTPUT_ENABLE       TIM_BDTR_AOE   /*!< MOE can be set by software or automatically
                                                             at the next update event */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_TIM_READ_REG and LL_TIM_WRITE_REG functions.
  * @{
  */
#define LL_TIM_DIER_UIE                        TIM_DIER_UIE         /*!< Update interrupt enable */
#define LL_TIM_DIER_CC1IE                      TIM_DIER_CC1IE       /*!< Capture/compare 1 interrupt enable */
#define LL_TIM_DIER_CC2IE                      TIM_DIER_CC2IE       /*!< Capture/compare 2 interrupt enable */
#define LL_TIM_DIER_CC3IE                      TIM_DIER_CC3IE       /*!< Capture/compare 3 interrupt enable */
#define LL_TIM_DIER_CC4IE                      TIM_DIER_CC4IE       /*!< Capture/compare 4 interrupt enable */
#define LL_TIM_DIER_COMIE                      TIM_DIER_COMIE       /*!< COM interrupt enable */
#define LL_TIM_DIER_TIE                        TIM_DIER_TIE         /*!< Trigger interrupt enable */
#define LL_TIM_DIER_BIE                        TIM_DIER_BIE         /*!< Break interrupt enable */
#define LL_TIM_DIER_IDXIE                      TIM_DIER_IDXIE       /*!< Index interrupt enable */
#define LL_TIM_DIER_DIRIE                      TIM_DIER_DIRIE       /*!< Direction Change interrupt enable */
#define LL_TIM_DIER_IERRIE                     TIM_DIER_IERRIE      /*!< Index Error interrupt enable */
#define LL_TIM_DIER_TERRIE                     TIM_DIER_TERRIE      /*!< Transition Error interrupt enable */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMA  DMA request Defines
  * @brief    DMA request defines which can be used with LL_TIM_READ_REG and LL_TIM_WRITE_REG functions.
  * @{
  */
#define LL_TIM_DIER_UDE             TIM_DIER_UDE                    /*!< Update DMA request enable */
#define LL_TIM_DIER_CC1DE           TIM_DIER_CC1DE                  /*!< Capture/compare 1 DMA request enable */
#define LL_TIM_DIER_CC2DE           TIM_DIER_CC2DE                  /*!< Capture/compare 2 DMA request enable */
#define LL_TIM_DIER_CC3DE           TIM_DIER_CC3DE                  /*!< Capture/compare 3 DMA request enable */
#define LL_TIM_DIER_CC4DE           TIM_DIER_CC4DE                  /*!< Capture/compare 4 DMA request enable */
#define LL_TIM_DIER_COMDE           TIM_DIER_COMDE                  /*!< COM DMA request enable */
#define LL_TIM_DIER_TDE             TIM_DIER_TDE                    /*!< Trigger DMA request enable */
/**
  * @}
  *
  */

/** @defgroup TIM_LL_EC_UPDATESOURCE Update Source
  * @{
  */
#define LL_TIM_UPDATESOURCE_REGULAR   0x00000000U    /*!< Counter overflow/underflow, Setting the UG bit or Update
                                                      generation through the slave mode controller
                                                      generates an update request */
#define LL_TIM_UPDATESOURCE_COUNTER   TIM_CR1_URS    /*!< Only counter overflow/underflow generates an update request */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_COUNTERMODE Counter Mode
  * @{
  */
#define LL_TIM_COUNTERMODE_UP              0x00000000U       /*!< Counter used as upcounter */
#define LL_TIM_COUNTERMODE_DOWN            TIM_CR1_DIR       /*!< Counter used as downcounter */
#define LL_TIM_COUNTERMODE_CENTER_DOWN     TIM_CR1_CMS_0     /*!< The counter counts up and down alternatively.
                                                              Output compare interrupt flags of output channels
                                                              are set only when the counter is counting down. */
#define LL_TIM_COUNTERMODE_CENTER_UP       TIM_CR1_CMS_1     /*!< The counter counts up and down alternatively.
                                                              Output compare interrupt flags of output channels
                                                              are set only when the counter is counting up */
#define LL_TIM_COUNTERMODE_CENTER_UP_DOWN  TIM_CR1_CMS       /*!< The counter counts up and down alternatively.
                                                              Output compare interrupt flags of output channels
                                                              are set only when the counter is counting up or down. */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CLOCKDIVISION_DIV Clock Division
  * @{
  */
#define LL_TIM_CLOCKDIVISION_DIV1              0x00000000U                         /*!< tDTS=tTIM_KER_CK   */
#define LL_TIM_CLOCKDIVISION_DIV2              TIM_CR1_CKD_0                       /*!< tDTS=2*tTIM_KER_CK */
#define LL_TIM_CLOCKDIVISION_DIV4              TIM_CR1_CKD_1                       /*!< tDTS=4*tTIM_KER_CK */
#define LL_TIM_CLOCKDIVISION_DIV8              (TIM_CR1_CKD_1 | TIM_CR1_CKD_0)     /*!< tDTS=8*tTIM_KER_CK */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CLOCKDIVISION2_DIV Clock Division 2
  * @{
  */
#define LL_TIM_CLOCKDIVISION2_DIV1        0x00000000U                                       /*!< tDTS2=tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV4        TIM_CR1_CKD2_0                                    /*!< tDTS2=4*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV16       TIM_CR1_CKD2_1                                    /*!< tDTS2=16*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV64       (TIM_CR1_CKD2_1 | TIM_CR1_CKD2_0)                 /*!< tDTS2=64*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV256      TIM_CR1_CKD2_2                                    /*!< tDTS2=256*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV1024     (TIM_CR1_CKD2_2 | TIM_CR1_CKD2_0)                 /*!< tDTS2=1024*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV4096     (TIM_CR1_CKD2_2 | TIM_CR1_CKD2_1)                 /*!< tDTS2=4096*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV16384    (TIM_CR1_CKD2_2 |TIM_CR1_CKD2_1 | TIM_CR1_CKD2_0) /*!< tDTS2=16384*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV65536    TIM_CR1_CKD2_3                                    /*!< tDTS2=65536*tDTS */
#define LL_TIM_CLOCKDIVISION2_DIV262144   (TIM_CR1_CKD2_3 | TIM_CR1_CKD2_0)                 /*!< tDTS2=262144*tDTS */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_COUNTERDIRECTION Counter Direction
  * @{
  */
#define LL_TIM_COUNTERDIRECTION_UP        0x00000000U /*!< Timer counter counts up */
#define LL_TIM_COUNTERDIRECTION_DOWN      TIM_CR1_DIR /*!< Timer counter counts down */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CCUPDATESOURCE Capture Compare Update Source
  * @{
  */
#define LL_TIM_CCUPDATESOURCE_SOFTWARE               0x00000000U   /*!< Capture/compare control bits are updated
                                                                        by setting the COMG bit only */
#define LL_TIM_CCUPDATESOURCE_SOFTWARE_AND_TRIGGER   TIM_CR2_CCUS  /*!< Capture/compare control bits are updated
                                                                        by setting the COMG bit or when a rising edge
                                                                        occurs on trigger input (TRGI) */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CCDMAREQUEST Capture Compare DMA Request
  * @{
  */
#define LL_TIM_CCDMAREQUEST_CC             0x00000000U      /*!< CCx DMA request sent when CCx event occurs */
#define LL_TIM_CCDMAREQUEST_UPD            TIM_CR2_CCDS     /*!< CCx DMA requests sent when update event occurs */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_LOCKLEVEL Lock Level
  * @{
  */
#define LL_TIM_LOCKLEVEL_OFF                   0x00000000U          /*!< LOCK OFF - No bit is write protected */
#define LL_TIM_LOCKLEVEL_1                     TIM_BDTR_LOCK_0      /*!< LOCK Level 1 */
#define LL_TIM_LOCKLEVEL_2                     TIM_BDTR_LOCK_1      /*!< LOCK Level 2 */
#define LL_TIM_LOCKLEVEL_3                     TIM_BDTR_LOCK        /*!< LOCK Level 3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CHANNEL Channel
  * @{
  */
#define LL_TIM_CHANNEL_CH1             TIM_CCER_CC1E             /*!< Timer input/output channel 1 */
#define LL_TIM_CHANNEL_CH1N            TIM_CCER_CC1NE            /*!< Timer complementary output channel 1 */
#define LL_TIM_CHANNEL_CH2             TIM_CCER_CC2E             /*!< Timer input/output channel 2 */
#define LL_TIM_CHANNEL_CH2N            TIM_CCER_CC2NE            /*!< Timer complementary output channel 2 */
#define LL_TIM_CHANNEL_CH3             TIM_CCER_CC3E             /*!< Timer input/output channel 3 */
#define LL_TIM_CHANNEL_CH3N            TIM_CCER_CC3NE            /*!< Timer complementary output channel 3 */
#define LL_TIM_CHANNEL_CH4             TIM_CCER_CC4E             /*!< Timer input/output channel 4 */
#define LL_TIM_CHANNEL_CH4N            TIM_CCER_CC4NE            /*!< Timer complementary output channel 4 */
#define LL_TIM_CHANNEL_CH5             TIM_CCER_CC5E             /*!< Timer output channel 5 */
#define LL_TIM_CHANNEL_CH6             TIM_CCER_CC6E             /*!< Timer output channel 6 */
#define LL_TIM_CHANNEL_CH7             TIM_CCER_CC7E             /*!< Timer output channel 7 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCSTATE Output Configuration State
  * @{
  */
#define LL_TIM_OCSTATE_DISABLE          0x00000000U      /*!< OCx is not active */
#define LL_TIM_OCSTATE_ENABLE           TIM_CCER_CC1E    /*!< OCx signal is output on the corresponding output pin */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCMODE Output Configuration Mode
  * @{
  */
#define LL_TIM_OCMODE_FROZEN             0x00000000U                            /*!<The comparison between the output
                                                                                 compare register TIMx_CCRy and
                                                                                 the counter TIMx_CNT has no effect on
                                                                                 the output channel level */
#define LL_TIM_OCMODE_ACTIVE_ON_MATCH    TIM_CCMR1_OC1M_0                       /*!<OCyREF is forced high on compare
                                                                                 match */
#define LL_TIM_OCMODE_INACTIVE_ON_MATCH  TIM_CCMR1_OC1M_1                       /*!<OCyREF is forced low on compare
                                                                                 match */
#define LL_TIM_OCMODE_TOGGLE             (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0)  /*!<OCyREF toggles on compare match */
#define LL_TIM_OCMODE_FORCED_INACTIVE    TIM_CCMR1_OC1M_2                       /*!<OCyREF is forced low */
#define LL_TIM_OCMODE_FORCED_ACTIVE      (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)  /*!<OCyREF is forced high */
#define LL_TIM_OCMODE_PWM1               (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)  /*!<In upcounting, channel y is active
                                                                                as long as TIMx_CNT<TIMx_CCRy else
                                                                                inactive.In downcounting, channel y
                                                                                is inactive as long as
                                                                                TIMx_CNT>TIMx_CCRy else active */
#define LL_TIM_OCMODE_PWM2               (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 \
                                          | TIM_CCMR1_OC1M_0)                   /*!<In upcounting, channel y is
                                                                                inactive as long as TIMx_CNT<TIMx_CCRy
                                                                                else active. In downcounting, channel y
                                                                                is active as long as TIMx_CNT>TIMx_CCRy
                                                                                else inactive */
#define LL_TIM_OCMODE_RETRIGERRABLE_OPM1  TIM_CCMR1_OC1M_3                       /*!<Retrigerrable OPM mode 1 */
#define LL_TIM_OCMODE_RETRIGERRABLE_OPM2  (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_0)  /*!<Retrigerrable OPM mode 2 */
#define LL_TIM_OCMODE_COMBINED_PWM1      (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2)   /*!<Combined PWM mode 1 */
#define LL_TIM_OCMODE_COMBINED_PWM2      (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_0)                    /*!<Combined PWM mode 2 */
#define LL_TIM_OCMODE_COMBINED_PWM3      TIM_CCMR1_OC1M_4                        /*!<Combined PWM mode 3 */
#define LL_TIM_OCMODE_COMBINED_PWM4      (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_0)   /*!<Combined PWM mode 4 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM1    (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_1)                    /*!<Asymmetric PWM mode 1 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM2    (TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0) /*!<Asymmetric PWM mode 2 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM3    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_1)   /*!<Asymmetric PWM mode 3 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM4    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_1 \
                                          | TIM_CCMR1_OC1M_0)                    /*!<Asymmetric PWM mode 4 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM5    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_2)   /*!<Asymmetric PWM mode 5 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM6    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_0)                    /*!<Asymmetric PWM mode 6 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM7    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_1)                    /*!<Asymmetric PWM mode 7 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM8    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_2 \
                                          | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0) /*!<Asymmetric PWM mode 8 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM9    (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_3)   /*!<Asymmetric PWM mode 9 */
#define LL_TIM_OCMODE_ASYMMETRIC_PWM10   (TIM_CCMR1_OC1M_4 | TIM_CCMR1_OC1M_3 \
                                          | TIM_CCMR1_OC1M_0)                    /*!<Asymmetric PWM mode 10 */
#define LL_TIM_OCMODE_PULSE_ON_COMPARE   (TIM_CCMR2_OC3M_3 | TIM_CCMR2_OC3M_1)   /*!<Pulse on Compare mode */
#define LL_TIM_OCMODE_DIRECTION_OUTPUT   (TIM_CCMR2_OC3M_3 | TIM_CCMR2_OC3M_1 \
                                          | TIM_CCMR2_OC3M_0)                    /*!<Direction output mode */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCPOLARITY Output Configuration Polarity
  * @{
  */
#define LL_TIM_OCPOLARITY_HIGH                 0x00000000U                        /*!< OCx active high */
#define LL_TIM_OCPOLARITY_LOW                  TIM_CCER_CC1P                      /*!< OCx active low */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCIDLESTATE Output Configuration Idle State
  * @{
  */
#define LL_TIM_OCIDLESTATE_RESET               0x00000000U             /*!< OCx/OCxN=0 (after a dead-time
                                                                       if OC is implemented) when MOE=0 */
#define LL_TIM_OCIDLESTATE_SET                 TIM_CR2_OIS1            /*!< OCx/OCxN=1 (after a dead-time
                                                                       if OC is implemented) when MOE=0 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCOVERRIDE Output Configuration Idle State Override
  * @{
  */
#define LL_TIM_OCOVERRIDE_RESET                0x00000000U             /*!< OCx/OCxN=0 when OOC=1 */
#define LL_TIM_OCOVERRIDE_SET                  TIM_OOR_OOS1            /*!< OCx/OCxN=1 when OOC=1 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OCBREAKMODE Output Configuration Break Mode
  * @{
  */
#define LL_TIM_OCBREAKMODE_IMMEDIATE           0x00000000U             /*!< Immediate break */
#define LL_TIM_OCBREAKMODE_DELAY1              TIM_MPR1_BK1M_0         /*!< Delayed 1 break */
#define LL_TIM_OCBREAKMODE_DELAY2              TIM_MPR1_BK1M_1         /*!< Delayed 2 break */
/**
  * @}
  */


/** @defgroup TIM_LL_EC_OC_COMPARE_UNIT Compare Unit
  * @{
  */
#define LL_TIM_OC_COMPARE_UNIT_1             0U             /*!< Timer compare unit 1 */
#define LL_TIM_OC_COMPARE_UNIT_2             1U             /*!< Timer compare unit 2 */
#define LL_TIM_OC_COMPARE_UNIT_3             2U             /*!< Timer compare unit 3 */
#define LL_TIM_OC_COMPARE_UNIT_4             3U             /*!< Timer compare unit 4 */
#define LL_TIM_OC_COMPARE_UNIT_5             4U             /*!< Timer compare unit 5 */
#define LL_TIM_OC_COMPARE_UNIT_6             5U             /*!< Timer compare unit 6 */
#define LL_TIM_OC_COMPARE_UNIT_7             6U             /*!< Timer compare unit 7 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_GROUPCH5 GROUPCH5
  * @{
  */
#define LL_TIM_GROUPCH5_NONE            0x00000000U        /*!< No effect of OC5REF on OC1REFC, OC2REFC and OC3REFC */
#define LL_TIM_GROUPCH5_AND_OC1REFC     TIM_CCR5_GC5C1     /*!< OC1REFC is the logical AND of OC1REFC  and OC5REF */
#define LL_TIM_GROUPCH5_AND_OC2REFC     TIM_CCR5_GC5C2     /*!< OC2REFC is the logical AND of OC2REFC and OC5REF */
#define LL_TIM_GROUPCH5_AND_OC3REFC     TIM_CCR5_GC5C3     /*!< OC3REFC is the logical AND of OC3REFC and OC5REF */
#define LL_TIM_GROUPCH5_AND_OC4REFC     TIM_CCR5_GC5C4     /*!< OC4REFC is the logical AND of OC4REFC and OC5REF */
#define LL_TIM_GROUPCH5_OR_OC1REFC      TIM_CCR5_GC5C1O    /*!< OC1REFC is the logical OR of OC1REFC and OC5REF */
#define LL_TIM_GROUPCH5_OR_OC2REFC      TIM_CCR5_GC5C2O    /*!< OC2REFC is the logical OR of OC2REFC and OC5REF */
#define LL_TIM_GROUPCH5_OR_OC3REFC      TIM_CCR5_GC5C3O    /*!< OC3REFC is the logical OR of OC3REFC and OC5REF */
#define LL_TIM_GROUPCH5_OR_OC4REFC      TIM_CCR5_GC5C4O    /*!< OC4REFC is the logical OR of OC4REFC and OC5REF */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ACTIVEINPUT Active Input Selection
  * @{
  */
#define LL_TIM_ACTIVEINPUT_DIRECT       (TIM_CCMR1_CC1S_0 << LL_TIM_IC_CONFIG_POS)  /*!< ICx is mapped on TIx */
#define LL_TIM_ACTIVEINPUT_INDIRECT     (TIM_CCMR1_CC1S_1 << LL_TIM_IC_CONFIG_POS)  /*!< ICx is mapped on TIy */
#define LL_TIM_ACTIVEINPUT_TRC          (TIM_CCMR1_CC1S << LL_TIM_IC_CONFIG_POS)    /*!< ICx is mapped on TRC */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ICPSC Input Configuration Prescaler
  * @{
  */
#define LL_TIM_ICPSC_DIV1    0x00000000U                                  /*!< No prescaler, capture is done each time
                                                                          an edge is detected on the capture input */
#define LL_TIM_ICPSC_DIV2    (TIM_CCMR1_IC1PSC_0 << LL_TIM_IC_CONFIG_POS) /*!< Capture is done once every 2 events */
#define LL_TIM_ICPSC_DIV4    (TIM_CCMR1_IC1PSC_1 << LL_TIM_IC_CONFIG_POS) /*!< Capture is done once every 4 events */
#define LL_TIM_ICPSC_DIV8    (TIM_CCMR1_IC1PSC << LL_TIM_IC_CONFIG_POS)   /*!< Capture is done once every 8 events */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_FILTER Input Configuration Filter
  * @{
  */
#define LL_TIM_IC_FILTER_FDIV1          0x00000000U                                     /*!< No filter, sampling is
                                                                                        done at fDTS */
#define LL_TIM_IC_FILTER_FDIV1_N2       (TIM_CCMR1_IC1F_0 << LL_TIM_IC_CONFIG_POS)      /*!< fSAMPLING=fCK_INT, N=2 */
#define LL_TIM_IC_FILTER_FDIV1_N4       (TIM_CCMR1_IC1F_1 << LL_TIM_IC_CONFIG_POS)      /*!< fSAMPLING=fCK_INT, N=4 */
#define LL_TIM_IC_FILTER_FDIV1_N8       ((TIM_CCMR1_IC1F_1 \
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fCK_INT, N=8 */
#define LL_TIM_IC_FILTER_FDIV2_N6       (TIM_CCMR1_IC1F_2 << LL_TIM_IC_CONFIG_POS)      /*!< fSAMPLING=fDTS/2, N=6 */
#define LL_TIM_IC_FILTER_FDIV2_N8       ((TIM_CCMR1_IC1F_2 \
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/2, N=8 */
#define LL_TIM_IC_FILTER_FDIV4_N6       ((TIM_CCMR1_IC1F_2 \
                                          | TIM_CCMR1_IC1F_1) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/4, N=6 */
#define LL_TIM_IC_FILTER_FDIV4_N8       ((TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1\
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/4, N=8 */
#define LL_TIM_IC_FILTER_FDIV8_N6       (TIM_CCMR1_IC1F_3 << LL_TIM_IC_CONFIG_POS)      /*!< fSAMPLING=fDTS/8, N=6 */
#define LL_TIM_IC_FILTER_FDIV8_N8       ((TIM_CCMR1_IC1F_3 \
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/8, N=8 */
#define LL_TIM_IC_FILTER_FDIV16_N5      ((TIM_CCMR1_IC1F_3 \
                                          | TIM_CCMR1_IC1F_1) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/16, N=5 */
#define LL_TIM_IC_FILTER_FDIV16_N6      ((TIM_CCMR1_IC1F_3| TIM_CCMR1_IC1F_1 \
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/16, N=6 */
#define LL_TIM_IC_FILTER_FDIV16_N8      ((TIM_CCMR1_IC1F_3 \
                                          | TIM_CCMR1_IC1F_2) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/16, N=8 */
#define LL_TIM_IC_FILTER_FDIV32_N5      ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 \
                                          | TIM_CCMR1_IC1F_0) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/32, N=5 */
#define LL_TIM_IC_FILTER_FDIV32_N6      ((TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 \
                                          | TIM_CCMR1_IC1F_1) << LL_TIM_IC_CONFIG_POS)  /*!< fSAMPLING=fDTS/32, N=6 */
#define LL_TIM_IC_FILTER_FDIV32_N8      (TIM_CCMR1_IC1F << LL_TIM_IC_CONFIG_POS)        /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_POLARITY Input Configuration Polarity
  * @{
  */
#define LL_TIM_IC_POLARITY_RISING           0x00000000U            /*!< The circuit is sensitive to TIxFP1 rising edge,
                                                                   TIxFP1 is not inverted */
#define LL_TIM_IC_POLARITY_FALLING          TIM_CCER_CC1P          /*!< The circuit is sensitive to TIxFP1 falling edge,
                                                                   TIxFP1 is inverted */
#define LL_TIM_IC_POLARITY_RISING_FALLING   (TIM_CCER_CC1P \
                                             | TIM_CCER_CC1NP)     /*!< The circuit is sensitive to both TIxFP1 rising
                                                                   and falling edges, TIxFP1 is not inverted */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_XOR_GATE_POS XOR Gate Position Configuration
  * @{
  */
#define LL_TIM_IC_XOR_GATE_POS_DIRECT           0x00000000U      /*!< XOR gate placed before TI1 filter */
#define LL_TIM_IC_XOR_GATE_POS_FILTERED         TIM_CR2_XORPS    /*!< XOR gate placed after TI1, TI2 and TI3 filters,
                                                                 edge detector placed on XOR output */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_IC_SIGNAL Input Channel Signal Status
  * @{
  */
#define LL_TIM_IC_SIGNAL_LOW                    0x00000000U                 /*!< ICx signal is low */
#define LL_TIM_IC_SIGNAL_HIGH                   TIM_SR_TI1FS                /*!< ICx signal is high */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_CLK Clock Sources
  * @{
  */
#define LL_TIM_CLK_INTERNAL                   0x00000000U                 /*!< The timer is clocked by the internal
                                                                          clock provided from the RCC */
#define LL_TIM_CLK_EXTERNAL_MODE1             (TIM_SMCR_SMS_2 \
                                               | TIM_SMCR_SMS_1 \
                                               | TIM_SMCR_SMS_0)          /*!< Counter counts at each rising or
                                                                          falling edge on a selected input*/
#define LL_TIM_CLK_EXTERNAL_MODE2             TIM_SMCR_ECE                /*!< Counter counts at each rising or falling
                                                                          edge on the external trigger input ETR */
#define LL_TIM_CLK_ENCODER_X1_TI1             (TIM_SMCR_SMS_3 \
                                               | TIM_SMCR_SMS_2 \
                                               | TIM_SMCR_SMS_1)          /*!< Quadrature encoder mode:
                                                                          x1 mode, counting on TI1FP1 edges only,
                                                                          edge sensitivity is set by CC1P */
#define LL_TIM_CLK_ENCODER_X1_TI2             (TIM_SMCR_SMS_3 \
                                               | TIM_SMCR_SMS_2 \
                                               | TIM_SMCR_SMS_1 \
                                               | TIM_SMCR_SMS_0)          /*!< Quadrature encoder mode: x1 mode,
                                                                          counting on TI2FP2 edges only,
                                                                          edge sensitivity is set by CC1P */
#define LL_TIM_CLK_ENCODER_X2_TI1             TIM_SMCR_SMS_0              /*!< Quadrature encoder mode 1:
                                                                          x2 mode, Counter counts up/down
                                                                          on TI1FP1 edge depending on TI2FP2 level */
#define LL_TIM_CLK_ENCODER_X2_TI2             TIM_SMCR_SMS_1              /*!< Quadrature encoder mode 2:
                                                                          x2 mode, Counter counts up/down
                                                                          on TI2FP2 edge depending on TI1FP1 level */
#define LL_TIM_CLK_ENCODER_X4_TI12            (TIM_SMCR_SMS_1 \
                                               | TIM_SMCR_SMS_0)          /*!< Quadrature encoder mode 3:
                                                                          x4 mode, Counter counts up/down
                                                                          on both TI1FP1 and TI2FP2 edges
                                                                          depending on the level of the other input */
#define LL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1   TIM_SMCR_SMS_4              /*!< Quadrature encoder with built-in
                                                                          debouncer: x2 mode, Counter counts up/down
                                                                          on TI1FP1 edge depending on TI2FP2 level */
#define LL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12  (TIM_SMCR_SMS_4 \
                                               | TIM_SMCR_SMS_0)          /*!< Quadrature encoder with built-in
                                                                          debouncer:x4 mode, Counter counts up/down on
                                                                          both TI1FP1 and TI2FP2 edges depending on
                                                                          the level of the other input */
#define LL_TIM_CLK_ENCODER_CLK_PLUS_X2        (TIM_SMCR_SMS_3 \
                                               | TIM_SMCR_SMS_1)          /*!< Encoder mode: Clock plus direction,
                                                                          x2 mode */
#define LL_TIM_CLK_ENCODER_CLK_PLUS_X1        (TIM_SMCR_SMS_3 \
                                               | TIM_SMCR_SMS_1 \
                                               | TIM_SMCR_SMS_0)          /*!< Encoder mode:Clock plus direction,
                                                                          x1 mode, TI2FP2 edge sensitivity is set
                                                                          by CC2P */
#define LL_TIM_CLK_ENCODER_DIR_CLK_X2         (TIM_SMCR_SMS_3 \
                                               | TIM_SMCR_SMS_2)          /*!< Encoder mode: Directional Clock,
                                                                          x2 mode */
#define LL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12    (TIM_SMCR_SMS_3 | TIM_SMCR_SMS_2 \
                                               | TIM_SMCR_SMS_0)          /*!< Encoder mode: Directional Clock,
                                                                          x1 mode, TI1FP1 and TI2FP2 edge sensitivity
                                                                          is set by CC1P and CC2P */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TRGO Trigger Output
  * @{
  */
#define LL_TIM_TRGO_RESET        0x00000000U       /*!< UG bit from the TIMx_EGR register is used as trigger output */
#define LL_TIM_TRGO_ENABLE       TIM_CR2_MMS_0     /*!< Counter Enable signal (CNT_EN) is used as trigger output */
#define LL_TIM_TRGO_UPDATE       TIM_CR2_MMS_1                    /*!< Update event is used as trigger output */
#define LL_TIM_TRGO_CC1IF        (TIM_CR2_MMS_1 | TIM_CR2_MMS_0)  /*!< CC1 capture or a compare match is used as trigger
                                                                   output */
#define LL_TIM_TRGO_OC1          TIM_CR2_MMS_2                    /*!< OC1REFC signal is used as trigger output */
#define LL_TIM_TRGO_OC2          (TIM_CR2_MMS_2 | TIM_CR2_MMS_0)  /*!< OC2REFC signal is used as trigger output */
#define LL_TIM_TRGO_OC3          (TIM_CR2_MMS_2 | TIM_CR2_MMS_1)  /*!< OC3REFC signal is used as trigger output */
#define LL_TIM_TRGO_OC4          (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 \
                                  | TIM_CR2_MMS_0)                /*!< OC4REFC signal is used as trigger output */
#define LL_TIM_TRGO_ENCODER_CLK  TIM_CR2_MMS_3                    /*!< Encoder clock signal is used as trigger output */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TRGO2 Trigger Output 2
  * @{
  */
#define LL_TIM_TRGO2_RESET      0x00000000U      /*!< UG bit from the TIMx_EGR register is used as trigger output 2 */
#define LL_TIM_TRGO2_ENABLE     TIM_CR2_MMS2_0   /*!< Counter Enable signal (CNT_EN) is used as trigger output 2 */
#define LL_TIM_TRGO2_UPDATE     TIM_CR2_MMS2_1   /*!< Update event is used as trigger output 2 */
#define LL_TIM_TRGO2_CC1F       (TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0)     /*!< CC1 capture or a compare match
                                                                      is used as trigger output 2 */
#define LL_TIM_TRGO2_OC1        TIM_CR2_MMS2_2                        /*!< OC1REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC2        (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_0)     /*!< OC2REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC3        (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1)     /*!< OC3REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC4        (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1 \
                                 | TIM_CR2_MMS2_0)                    /*!< OC4REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC5        TIM_CR2_MMS2_3                        /*!< OC5REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC6        (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_0)     /*!< OC6REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC7        TIM_CR2_MMS2_4                        /*!< OC7REFC signal is used as trigger output 2 */
#define LL_TIM_TRGO2_OC4_RISING_FALLING      (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_1)    /*!< OC4REFC rising or falling edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC6_RISING_FALLING      (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_1 \
                                              | TIM_CR2_MMS2_0)                   /*!< OC6REFC rising or falling edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC7_RISING_FALLING      (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_0)    /*!< OC7REFC rising or falling edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC4_RISING_OC6_RISING   (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2)    /*!< OC4REFC or OC6REFC rising edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC4_RISING_OC7_RISING   (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_1)    /*!< OC4REFC or OC7REFC rising edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC5_RISING_OC6_RISING   (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_1)                   /*!< OC5REFC or OC6REFC rising edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC5_RISING_OC7_RISING   (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_2)    /*!< OC5REFC or OC7REFC rising edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC6_RISING_OC7_RISING   (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_1)                   /*!< OC6REFC or OC7REFC rising edges
                                                                                  are used as trigger output 2 */
#define LL_TIM_TRGO2_OC4_RISING_OC6_FALLING  (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_0)                  /*!< OC4REFC rising or OC6REFC falling
                                                                                 edges are used as trigger output 2 */
#define LL_TIM_TRGO2_OC4_RISING_OC7_FALLING  (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_1 \
                                              | TIM_CR2_MMS2_0)                  /*!< OC4REFC rising or OC7REFC falling
                                                                                 edges are used as trigger output 2 */
#define LL_TIM_TRGO2_OC5_RISING_OC6_FALLING  (TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0) /*!< OC5REFC rising or OC6REFC falling
                                                                                 edges are used as trigger output 2 */
#define LL_TIM_TRGO2_OC5_RISING_OC7_FALLING  (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_0)                  /*!< OC5REFC rising or OC7REFC falling
                                                                                 edges are used as trigger output 2 */
#define LL_TIM_TRGO2_OC6_RISING_OC7_FALLING  (TIM_CR2_MMS2_4 | TIM_CR2_MMS2_2 \
                                              | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0) /*!< OC6REFC rising or OC7REFC falling
                                                                                 edges are used as trigger output 2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_SLAVEMODE Slave Mode
  * @{
  */
#define LL_TIM_SLAVEMODE_DISABLED      0x00000000U                        /*!< Slave mode disabled */
#define LL_TIM_SLAVEMODE_RESET         TIM_SMCR_SMS_2                     /*!< Reset Mode - Rising edge of
                                                                          the selected trigger input (TRGI)
                                                                          reinitializes the counter */
#define LL_TIM_SLAVEMODE_GATED         (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0)  /*!< Gated Mode - The counter clock
                                                                          is enabled when the trigger input
                                                                          (TRGI) is high */
#define LL_TIM_SLAVEMODE_TRIGGER       (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1)  /*!< Trigger Mode - The counter starts
                                                                          at a rising edge of the trigger TRGI */
#define LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER  TIM_SMCR_SMS_3           /*!< Combined reset + trigger mode - Rising
                                                                          edge of the selected trigger input (TRGI)
                                                                          reinitializes the counter, generates an update
                                                                          of the registers and starts the counter */
#define LL_TIM_SLAVEMODE_COMBINED_GATED_RESET    (TIM_SMCR_SMS_3 | TIM_SMCR_SMS_0) /*!< Combined gated + reset mode
                                                                                  - The counter clock is enabled when
                                                                                  the trigger input (TRGI) is high.
                                                                                  The counter stops and is reset) as
                                                                                  soon as the trigger becomes low.Both
                                                                                  startand stop of the counter
                                                                                  are controlled. */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_SMS_PRELOAD_SOURCE SMS Preload Source
  * @{
  */
#define LL_TIM_SLAVE_MODE_PRELOAD_UPDATE   0x00000000U     /*!< The SMS preload transfer is triggered
                                                                by the Timer's Update event */
#define LL_TIM_SLAVE_MODE_PRELOAD_INDEX    TIM_SMCR_SMSPS  /*!< The SMS preload transfer is triggered
                                                                by the Index event */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TS Trigger Selection
  * @{
  */
#define LL_TIM_TS_ITR0      0x00000000U                            /*!< Internal Trigger 0 (ITR0) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR1      TIM_SMCR_TS_0                          /*!< Internal Trigger 1 (ITR1) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR2      TIM_SMCR_TS_1                          /*!< Internal Trigger 2 (ITR2) is used as
                                                                   trigger input */
#if defined(TIM3)
#define LL_TIM_TS_ITR3      (TIM_SMCR_TS_0 | TIM_SMCR_TS_1)        /*!< Internal Trigger 3 (ITR3) is used as
                                                                   trigger input */
#endif /* TIM3 */
#if defined(TIM4)
#define LL_TIM_TS_ITR4      (TIM_SMCR_TS_3)                        /*!< Internal Trigger 4 (ITR4) is used as
                                                                   trigger input */
#endif /* TIM4 */
#if defined(TIM5)
#define LL_TIM_TS_ITR5      (TIM_SMCR_TS_0 | TIM_SMCR_TS_3)        /*!< Internal Trigger 5 (ITR5) is used as
                                                                   trigger input */
#endif /* TIM5 */
#define LL_TIM_TS_ITR6      (TIM_SMCR_TS_1 | TIM_SMCR_TS_3)        /*!< Internal Trigger 6 (ITR6) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR7      (TIM_SMCR_TS_0 | TIM_SMCR_TS_1 \
                             | TIM_SMCR_TS_3)                      /*!< Internal Trigger 7 (ITR7) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR8      (TIM_SMCR_TS_2 | TIM_SMCR_TS_3)        /*!< Internal Trigger 8 (ITR8) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR9      (TIM_SMCR_TS_0 | TIM_SMCR_TS_2 \
                             | TIM_SMCR_TS_3)                      /*!< Internal Trigger 9 (ITR9) is used as
                                                                   trigger input */
#if defined(TIM16)
#define LL_TIM_TS_ITR10     (TIM_SMCR_TS_1 | TIM_SMCR_TS_2 \
                             | TIM_SMCR_TS_3)                      /*!< Internal Trigger 10 (ITR10) is used as
                                                                   trigger input */
#define LL_TIM_TS_ITR11     (TIM_SMCR_TS_0 | TIM_SMCR_TS_1 \
                             | TIM_SMCR_TS_2 | TIM_SMCR_TS_3)      /*!< Internal Trigger 11 (ITR11) is used as
                                                                   trigger input */
#endif /* TIM16 */
#define LL_TIM_TS_TI1F_ED   TIM_SMCR_TS_2                          /*!< TI1 Edge Detector (TI1F_ED) is used as
                                                                   trigger input */
#define LL_TIM_TS_TI1FP1    (TIM_SMCR_TS_2 | TIM_SMCR_TS_0)        /*!< Filtered Timer Input 1 (TI1FP1) is used as
                                                                   trigger input */
#define LL_TIM_TS_TI2FP2    (TIM_SMCR_TS_2 | TIM_SMCR_TS_1)        /*!< Filtered Timer Input 2 (TI12P2) is used as
                                                                   trigger input */
#define LL_TIM_TS_ETRF      (TIM_SMCR_TS_2 | TIM_SMCR_TS_1 \
                             | TIM_SMCR_TS_0)                      /*!< Filtered external Trigger (ETRF) is used as
                                                                   trigger input */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_POLARITY External Trigger Polarity
  * @{
  */
#define LL_TIM_ETR_POLARITY_NONINVERTED   0x00000000U   /*!< ETR is non-inverted, active at high level or rising edge */
#define LL_TIM_ETR_POLARITY_INVERTED      TIM_SMCR_ETP  /*!< ETR is inverted, active at low level or falling edge */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_PRESCALER External Trigger Prescaler
  * @{
  */
#define LL_TIM_ETR_PRESCALER_DIV1              0x00000000U             /*!< ETR prescaler OFF */
#define LL_TIM_ETR_PRESCALER_DIV2              TIM_SMCR_ETPS_0         /*!< ETR frequency is divided by 2 */
#define LL_TIM_ETR_PRESCALER_DIV4              TIM_SMCR_ETPS_1         /*!< ETR frequency is divided by 4 */
#define LL_TIM_ETR_PRESCALER_DIV8              TIM_SMCR_ETPS           /*!< ETR frequency is divided by 8 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_SYNCPRESCALER External Trigger Synchronous Prescaler
  * @{
  */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV1    0x00000000U                             /*!< ETR synchronous prescaler OFF */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV2    TIM_SMCR_SETPS_0                        /*!< ETR frequency is divided by 2 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV3    TIM_SMCR_SETPS_1                        /*!< ETR frequency is divided by 3 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV4    (TIM_SMCR_SETPS_1 | TIM_SMCR_SETPS_0)   /*!< ETR frequency is divided by 4 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV5    TIM_SMCR_SETPS_2                        /*!< ETR frequency is divided by 5 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV6    (TIM_SMCR_SETPS_2 | TIM_SMCR_SETPS_0)   /*!< ETR frequency is divided by 6 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV7    (TIM_SMCR_SETPS_2 | TIM_SMCR_SETPS_1)   /*!< ETR frequency is divided by 7 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV8    (TIM_SMCR_SETPS_2 | TIM_SMCR_SETPS_1 \
                                           | TIM_SMCR_SETPS_0)                    /*!< ETR frequency is divided by 8 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV9    TIM_SMCR_SETPS_3                        /*!< ETR frequency is divided by 9 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV10   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_0)   /*!< ETR frequency is divided by 10 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV11   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_1)   /*!< ETR frequency is divided by 11 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV12   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_1 \
                                           | TIM_SMCR_SETPS_0)                    /*!< ETR frequency is divided by 12 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV13   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_2)   /*!< ETR frequency is divided by 13 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV14   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_2 \
                                           | TIM_SMCR_SETPS_0)                    /*!< ETR frequency is divided by 14 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV15   (TIM_SMCR_SETPS_3 | TIM_SMCR_SETPS_2 \
                                           | TIM_SMCR_SETPS_1)                    /*!< ETR frequency is divided by 15 */
#define LL_TIM_ETR_SYNC_PRESCALER_DIV16   TIM_SMCR_SETPS                          /*!< ETR frequency is divided by 16 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_ETR_FILTER External Trigger Filter
  * @{
  */
#define LL_TIM_ETR_FILTER_FDIV1         0x00000000U                          /*!< No filter, sampling is done at fDTS */
#define LL_TIM_ETR_FILTER_FDIV1_N2      TIM_SMCR_ETF_0                       /*!< fSAMPLING=fCK_INT, N=2 */
#define LL_TIM_ETR_FILTER_FDIV1_N4      TIM_SMCR_ETF_1                       /*!< fSAMPLING=fCK_INT, N=4 */
#define LL_TIM_ETR_FILTER_FDIV1_N8      (TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0)    /*!< fSAMPLING=fCK_INT, N=8 */
#define LL_TIM_ETR_FILTER_FDIV2_N6      TIM_SMCR_ETF_2                       /*!< fSAMPLING=fDTS/2, N=6 */
#define LL_TIM_ETR_FILTER_FDIV2_N8      (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0)    /*!< fSAMPLING=fDTS/2, N=8 */
#define LL_TIM_ETR_FILTER_FDIV4_N6      (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1)    /*!< fSAMPLING=fDTS/4, N=6 */
#define LL_TIM_ETR_FILTER_FDIV4_N8      (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 \
                                         | TIM_SMCR_ETF_0)                   /*!< fSAMPLING=fDTS/4, N=8 */
#define LL_TIM_ETR_FILTER_FDIV8_N6      TIM_SMCR_ETF_3                       /*!< fSAMPLING=fDTS/8, N=6 */
#define LL_TIM_ETR_FILTER_FDIV8_N8      (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_0)    /*!< fSAMPLING=fDTS/16, N=8 */
#define LL_TIM_ETR_FILTER_FDIV16_N5     (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1)    /*!< fSAMPLING=fDTS/16, N=5 */
#define LL_TIM_ETR_FILTER_FDIV16_N6     (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1 \
                                         | TIM_SMCR_ETF_0)                   /*!< fSAMPLING=fDTS/16, N=6 */
#define LL_TIM_ETR_FILTER_FDIV16_N8     (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2)    /*!< fSAMPLING=fDTS/16, N=8 */
#define LL_TIM_ETR_FILTER_FDIV32_N5     (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 \
                                         | TIM_SMCR_ETF_0)                   /*!< fSAMPLING=fDTS/32, N=5 */
#define LL_TIM_ETR_FILTER_FDIV32_N6     (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 \
                                         | TIM_SMCR_ETF_1)                   /*!< fSAMPLING=fDTS/32, N=6 */
#define LL_TIM_ETR_FILTER_FDIV32_N8     TIM_SMCR_ETF                         /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_ETR_IN  TIM1 External Trigger Input
  * @{
  */
#define LL_TIM_TIM1_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim1_etr_in is connected to TIM1_ETR */
#define LL_TIM_TIM1_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim1_etr_in is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM1_ETR_IN_COMP2_OUT         TIM_AF1_ETRSEL_1                                                               /*!< tim1_etr_in is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
#define LL_TIM_TIM1_ETR_IN_ADC1_AWD1         (TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                                          /*!< tim1_etr_in is connected to adc1_awd1 */
#define LL_TIM_TIM1_ETR_IN_ADC1_AWD2         TIM_AF1_ETRSEL_2                                                               /*!< tim1_etr_in is connected to adc1_awd2 */
#define LL_TIM_TIM1_ETR_IN_ADC1_AWD3         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_0)                                          /*!< tim1_etr_in is connected to adc1_awd3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_ETR_IN  TIM2 External Trigger Input
  * @{
  */
#define LL_TIM_TIM2_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim2_etr_in is connected to TIM2_ETR */
#define LL_TIM_TIM2_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim2_etr_in is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM2_ETR_IN_COMP2_OUT         TIM_AF1_ETRSEL_1                                                               /*!< tim2_etr_in is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
#define LL_TIM_TIM2_ETR_IN_ADC1_AWD1         (TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                                          /*!< tim2_etr_in is connected to adc1_awd1 */
#define LL_TIM_TIM2_ETR_IN_ADC1_AWD2         TIM_AF1_ETRSEL_2                                                               /*!< tim2_etr_in is connected to adc1_awd2 */
#define LL_TIM_TIM2_ETR_IN_ADC1_AWD3         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_0)                                          /*!< tim2_etr_in is connected to adc1_awd3 */
#define LL_TIM_TIM2_ETR_IN_LSE               (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1)                                          /*!< tim2_etr_in is connected to LSE */
#define LL_TIM_TIM2_ETR_IN_MCO1              (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                       /*!< tim2_etr_in is connected to MCO1 */
#if defined(TIM3)
#define LL_TIM_TIM2_ETR_IN_TIM3_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_0)                                          /*!< tim2_etr_in is connected to TIM3_ETR */
#endif /* TIM3 */
#if defined(TIM4)
#define LL_TIM_TIM2_ETR_IN_TIM4_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1)                                          /*!< tim2_etr_in is connected to TIM4_ETR */
#endif /* TIM4 */
#if defined(TIM5)
#define LL_TIM_TIM2_ETR_IN_TIM5_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                       /*!< tim2_etr_in is connected to TIM5_ETR */
#endif /* TIM5 */
#if defined(ETH1)
#define LL_TIM_TIM2_ETR_IN_ETH1_PTP_PPS_OUT  (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1)                       /*!< tim2_etr_in is connected to eth1_ptp_pps_out */
#endif /* ETH1 */
/**
  * @}
  */

#if defined(TIM3)
/** @defgroup TIM_LL_EC_TIM3_ETR_IN  TIM3 External Trigger Input
  * @{
  */
#define LL_TIM_TIM3_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim3_etr_in is connected to TIM3_ETR */
#define LL_TIM_TIM3_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim3_etr_in is connected to comp1_out */
#define LL_TIM_TIM3_ETR_IN_TIM2_ETR          TIM_AF1_ETRSEL_3                                                               /*!< tim3_etr_in is connected to TIM2_ETR */
#if defined(TIM4)
#define LL_TIM_TIM3_ETR_IN_TIM4_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1)                                          /*!< tim3_etr_in is connected to TIM4_ETR */
#endif /* TIM4 */
#define LL_TIM_TIM3_ETR_IN_TIM5_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                       /*!< tim3_etr_in is connected to TIM5_ETR */
#if defined(ETH1)
#define LL_TIM_TIM3_ETR_IN_ETH1_PTP_PPS_OUT  (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1)                       /*!< tim3_etr_in is connected to eth1_ptp_pps_out */
#endif /* ETH1 */
/**
  * @}
  */
#endif /* TIM3 */

#if defined(TIM4)
/** @defgroup TIM_LL_EC_TIM4_ETR_IN  TIM4 External Trigger Input
  * @{
  */
#define LL_TIM_TIM4_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim4_etr_in is connected to TIM4_ETR */
#define LL_TIM_TIM4_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim4_etr_in is connected to comp1_out */
#define LL_TIM_TIM4_ETR_IN_ADC3_AWD1         (TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                                          /*!< tim4_etr_in is connected to adc3_awd1 */
#define LL_TIM_TIM4_ETR_IN_ADC3_AWD2         TIM_AF1_ETRSEL_2                                                               /*!< tim4_etr_in is connected to adc3_awd2 */
#define LL_TIM_TIM4_ETR_IN_ADC3_AWD3         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_0)                                          /*!< tim4_etr_in is connected to adc3_awd3 */
#define LL_TIM_TIM4_ETR_IN_TIM2_ETR          TIM_AF1_ETRSEL_3                                                               /*!< tim4_etr_in is connected to TIM2_ETR */
#define LL_TIM_TIM4_ETR_IN_TIM3_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_0)                                          /*!< tim4_etr_in is connected to TIM3_ETR */
#define LL_TIM_TIM4_ETR_IN_TIM5_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                       /*!< tim4_etr_in is connected to TIM5_ETR */
/**
  * @}
  */
#endif /* TIM4 */

#if defined(TIM5)
/** @defgroup TIM_LL_EC_TIM5_ETR_IN  TIM5 External Trigger Input
  * @{
  */
#define LL_TIM_TIM5_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim5_etr_in is connected to TIM5_ETR */
#define LL_TIM_TIM5_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim5_etr_in is connected to comp1_out */
#define LL_TIM_TIM5_ETR_IN_TIM2_ETR          TIM_AF1_ETRSEL_3                                                               /*!< tim5_etr_in is connected to TIM2_ETR */
#if defined(TIM3)
#define LL_TIM_TIM5_ETR_IN_TIM3_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_0)                                          /*!< tim5_etr_in is connected to TIM3_ETR */
#endif /* TIM3 */
#if defined(TIM4)
#define LL_TIM_TIM5_ETR_IN_TIM4_ETR          (TIM_AF1_ETRSEL_3 | TIM_AF1_ETRSEL_1)                                          /*!< tim5_etr_in is connected to TIM4_ETR */
#endif /* TIM4 */
/**
  * @}
  */
#endif /* TIM5 */

/** @defgroup TIM_LL_EC_TIM8_ETR_IN  TIM8 External Trigger Input
  * @{
  */
#define LL_TIM_TIM8_ETR_IN_GPIO              0x00000000U                                                                    /*!< tim8_etr_in is connected to TIM8_ETR */
#define LL_TIM_TIM8_ETR_IN_COMP1_OUT         TIM_AF1_ETRSEL_0                                                               /*!< tim8_etr_in is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM8_ETR_IN_COMP2_OUT         TIM_AF1_ETRSEL_1                                                               /*!< tim8_etr_in is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
#if defined(ADC1) && defined(ADC2)
#define LL_TIM_TIM8_ETR_IN_ADC2_AWD1         (TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                                          /*!< tim8_etr_in is connected to adc2_awd1 */
#define LL_TIM_TIM8_ETR_IN_ADC2_AWD2         TIM_AF1_ETRSEL_2                                                               /*!< tim8_etr_in is connected to adc2_awd2 */
#define LL_TIM_TIM8_ETR_IN_ADC2_AWD3         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_0)                                          /*!< tim8_etr_in is connected to adc2_awd3 */
#elif defined(ADC1)
#define LL_TIM_TIM8_ETR_IN_ADC1_AWD1         (TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                                          /*!< tim8_etr_in is connected to adc1_awd1 */
#define LL_TIM_TIM8_ETR_IN_ADC1_AWD2         TIM_AF1_ETRSEL_2                                                               /*!< tim8_etr_in is connected to adc1_awd2 */
#define LL_TIM_TIM8_ETR_IN_ADC1_AWD3         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_0)                                          /*!< tim8_etr_in is connected to adc1_awd3 */
#endif /* ADC1 && ADC2 */
#if defined(ADC3)
#define LL_TIM_TIM8_ETR_IN_ADC3_AWD1         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1)                                          /*!< tim8_etr_in is connected to adc3_awd1 */
#define LL_TIM_TIM8_ETR_IN_ADC3_AWD2         (TIM_AF1_ETRSEL_2 | TIM_AF1_ETRSEL_1 | TIM_AF1_ETRSEL_0)                       /*!< tim8_etr_in is connected to adc3_awd2 */
#define LL_TIM_TIM8_ETR_IN_ADC3_AWD3         TIM_AF1_ETRSEL_3                                                               /*!< tim8_etr_in is connected to adc3_awd3 */
#endif /* ADC3 */
/**
  * @}
  */


/** @defgroup TIM_LL_EC_BREAK_POLARITY break polarity
  * @{
  */
#define LL_TIM_BREAK_POLARITY_LOW              0x00000000U               /*!< Break input BRK is active low */
#define LL_TIM_BREAK_POLARITY_HIGH             TIM_BDTR_BKP              /*!< Break input BRK is active high */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_FILTER break filter
  * @{
  */
#define LL_TIM_BREAK_FILTER_FDIV1              0x00000000U   /*!< No filter, BRK acts asynchronously */
#define LL_TIM_BREAK_FILTER_FDIV1_N2           0x00010000U   /*!< fSAMPLING=fCK_INT, N=2 */
#define LL_TIM_BREAK_FILTER_FDIV1_N4           0x00020000U   /*!< fSAMPLING=fCK_INT, N=4 */
#define LL_TIM_BREAK_FILTER_FDIV1_N8           0x00030000U   /*!< fSAMPLING=fCK_INT, N=8 */
#define LL_TIM_BREAK_FILTER_FDIV2_N6           0x00040000U   /*!< fSAMPLING=fDTS/2, N=6 */
#define LL_TIM_BREAK_FILTER_FDIV2_N8           0x00050000U   /*!< fSAMPLING=fDTS/2, N=8 */
#define LL_TIM_BREAK_FILTER_FDIV4_N6           0x00060000U   /*!< fSAMPLING=fDTS/4, N=6 */
#define LL_TIM_BREAK_FILTER_FDIV4_N8           0x00070000U   /*!< fSAMPLING=fDTS/4, N=8 */
#define LL_TIM_BREAK_FILTER_FDIV8_N6           0x00080000U   /*!< fSAMPLING=fDTS/8, N=6 */
#define LL_TIM_BREAK_FILTER_FDIV8_N8           0x00090000U   /*!< fSAMPLING=fDTS/8, N=8 */
#define LL_TIM_BREAK_FILTER_FDIV16_N5          0x000A0000U   /*!< fSAMPLING=fDTS/16, N=5 */
#define LL_TIM_BREAK_FILTER_FDIV16_N6          0x000B0000U   /*!< fSAMPLING=fDTS/16, N=6 */
#define LL_TIM_BREAK_FILTER_FDIV16_N8          0x000C0000U   /*!< fSAMPLING=fDTS/16, N=8 */
#define LL_TIM_BREAK_FILTER_FDIV32_N5          0x000D0000U   /*!< fSAMPLING=fDTS/32, N=5 */
#define LL_TIM_BREAK_FILTER_FDIV32_N6          0x000E0000U   /*!< fSAMPLING=fDTS/32, N=6 */
#define LL_TIM_BREAK_FILTER_FDIV32_N8          0x000F0000U   /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_POLARITY BREAK2 POLARITY
  * @{
  */
#define LL_TIM_BREAK2_POLARITY_LOW             0x00000000U             /*!< Break input BRK2 is active low */
#define LL_TIM_BREAK2_POLARITY_HIGH            TIM_BDTR_BK2P           /*!< Break input BRK2 is active high */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_FILTER BREAK2 FILTER
  * @{
  */
#define LL_TIM_BREAK2_FILTER_FDIV1             0x00000000U   /*!< No filter, BRK acts asynchronously */
#define LL_TIM_BREAK2_FILTER_FDIV1_N2          0x00100000U   /*!< fSAMPLING=fCK_INT, N=2 */
#define LL_TIM_BREAK2_FILTER_FDIV1_N4          0x00200000U   /*!< fSAMPLING=fCK_INT, N=4 */
#define LL_TIM_BREAK2_FILTER_FDIV1_N8          0x00300000U   /*!< fSAMPLING=fCK_INT, N=8 */
#define LL_TIM_BREAK2_FILTER_FDIV2_N6          0x00400000U   /*!< fSAMPLING=fDTS/2, N=6 */
#define LL_TIM_BREAK2_FILTER_FDIV2_N8          0x00500000U   /*!< fSAMPLING=fDTS/2, N=8 */
#define LL_TIM_BREAK2_FILTER_FDIV4_N6          0x00600000U   /*!< fSAMPLING=fDTS/4, N=6 */
#define LL_TIM_BREAK2_FILTER_FDIV4_N8          0x00700000U   /*!< fSAMPLING=fDTS/4, N=8 */
#define LL_TIM_BREAK2_FILTER_FDIV8_N6          0x00800000U   /*!< fSAMPLING=fDTS/8, N=6 */
#define LL_TIM_BREAK2_FILTER_FDIV8_N8          0x00900000U   /*!< fSAMPLING=fDTS/8, N=8 */
#define LL_TIM_BREAK2_FILTER_FDIV16_N5         0x00A00000U   /*!< fSAMPLING=fDTS/16, N=5 */
#define LL_TIM_BREAK2_FILTER_FDIV16_N6         0x00B00000U   /*!< fSAMPLING=fDTS/16, N=6 */
#define LL_TIM_BREAK2_FILTER_FDIV16_N8         0x00C00000U   /*!< fSAMPLING=fDTS/16, N=8 */
#define LL_TIM_BREAK2_FILTER_FDIV32_N5         0x00D00000U   /*!< fSAMPLING=fDTS/32, N=5 */
#define LL_TIM_BREAK2_FILTER_FDIV32_N6         0x00E00000U   /*!< fSAMPLING=fDTS/32, N=6 */
#define LL_TIM_BREAK2_FILTER_FDIV32_N8         0x00F00000U   /*!< fSAMPLING=fDTS/32, N=8 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OSSI OSSI
  * @{
  */
#define LL_TIM_OSSI_DISABLE      0x00000000U       /*!< When inactive, OCx/OCxN outputs are disabled */
#define LL_TIM_OSSI_ENABLE       TIM_BDTR_OSSI     /*!< When inactive, OxC/OCxN outputs are first forced with their
                                                        inactive level then forced to their idle level after the
                                                        deadtime */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OSSR OSSR
  * @{
  */
#define LL_TIM_OSSR_DISABLE      0x00000000U       /*!< When inactive, OCx/OCxN outputs are disabled */
#define LL_TIM_OSSR_ENABLE       TIM_BDTR_OSSR     /*!< When inactive, OCx/OCxN outputs are enabled with their inactive
                                                        level as soon as CCxE=1 or CCxNE=1 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_OUTPUT OUTPUT DISABLE STATUS
  * @{
  */
#define LL_TIM_OUTPUT_IDLE_STATE               0x00000000U   /*!< Break was triggered (or MOE was written to 0) and
                                                             OIS states are forced on tim_ocx and tim_ocxn outputs */
#define LL_TIM_OUTPUT_DISABLED_STATE           TIM_SR_ODS    /*!< Break2 was triggered, tim_ocx and
                                                             tim_ocxn outputs are inactive */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_INPUT BREAK INPUT
  * @{
  */
#define LL_TIM_BREAK_INPUT_1                   0x00000000U  /*!< TIMx_BKIN input */
#define LL_TIM_BREAK_INPUT_2                   0x00000001U  /*!< TIMx_BKIN2 input */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BKIN_SOURCE BKIN SOURCE
  * @{
  */
/** @defgroup TIM_LL_EC_TIM1_BRK  TIM1 Break Input
  * @{
  */
#define LL_TIM_TIM1_BRK_GPIO              TIM_AF1_BKINE      /*!< tim1_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM1_BRK_COMP1_OUT         TIM_AF1_BKCMP1E    /*!< tim1_brk isconnected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM1_BRK_COMP2_OUT         TIM_AF1_BKCMP2E    /*!< tim1_brk isconnected to comp2_out */
#endif /* COMP2 */
#define LL_TIM_TIM1_BRK_TIM8_BKIN         TIM_AF1_BKCMP6E    /*!< tim1_brk is connected to TIM8_BKIN */
#define LL_TIM_TIM1_BRK_TIM15_BKIN        TIM_AF1_BKCMP8E    /*!< tim1_brk is connected to TIM15_BKIN */
#if defined(TIM16)
#define LL_TIM_TIM1_BRK_TIM16_BKIN        TIM_AF1_BKCMP9E    /*!< tim1_brk is connected to TIM16_BKIN */
#define LL_TIM_TIM1_BRK_TIM17_BKIN        TIM_AF1_BKCMP10E   /*!< tim1_brk is connected to TIM17_BKIN */
#endif /* TIM16 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_BRK  TIM8 Break Input
  * @{
  */
#define LL_TIM_TIM8_BRK_GPIO              TIM_AF1_BKINE      /*!< tim8_brk is connected to TIM8_BKIN */
#define LL_TIM_TIM8_BRK_COMP1_OUT         TIM_AF1_BKCMP1E    /*!< tim8_brk isconnected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM8_BRK_COMP2_OUT         TIM_AF1_BKCMP2E    /*!< tim8_brk isconnected to comp2_out */
#endif /* COMP2 */
#define LL_TIM_TIM8_BRK_TIM1_BKIN         TIM_AF1_BKCMP5E    /*!< tim8_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM8_BRK_TIM15_BKIN        TIM_AF1_BKCMP8E    /*!< tim8_brk is connected to TIM15_BKIN */
#if defined(TIM16)
#define LL_TIM_TIM8_BRK_TIM16_BKIN        TIM_AF1_BKCMP9E    /*!< tim8_brk is connected to TIM16_BKIN */
#define LL_TIM_TIM8_BRK_TIM17_BKIN        TIM_AF1_BKCMP10E   /*!< tim8_brk is connected to TIM17_BKIN */
#endif /* TIM16 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_BRK  TIM15 Break Input
  * @{
  */
#define LL_TIM_TIM15_BRK_GPIO             TIM_AF1_BKINE      /*!< tim15_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM15_BRK_COMP1_OUT        TIM_AF1_BKCMP1E    /*!< tim15_brk isconnected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM15_BRK_COMP2_OUT        TIM_AF1_BKCMP2E    /*!< tim15_brk isconnected to comp2_out */
#endif /* COMP2 */
#define LL_TIM_TIM15_BRK_TIM1_BKIN        TIM_AF1_BKCMP5E    /*!< tim15_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM15_BRK_TIM8_BKIN        TIM_AF1_BKCMP6E    /*!< tim15_brk is connected to TIM8_BKIN */
#if defined(TIM16)
#define LL_TIM_TIM15_BRK_TIM16_BKIN       TIM_AF1_BKCMP9E    /*!< tim15_brk is connected to TIM16_BKIN */
#define LL_TIM_TIM15_BRK_TIM17_BKIN       TIM_AF1_BKCMP10E   /*!< tim15_brk is connected to TIM17_BKIN */
#endif /* TIM16 */
/**
  * @}
  */

#if defined(TIM16)
/** @defgroup TIM_LL_EC_TIM16_BRK  TIM16 Break Input
  * @{
  */
#define LL_TIM_TIM16_BRK_GPIO             TIM_AF1_BKINE      /*!< tim16_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM16_BRK_COMP1_OUT        TIM_AF1_BKCMP1E    /*!< tim16_brk isconnected to comp1_out */
#define LL_TIM_TIM16_BRK_TIM1_BKIN        TIM_AF1_BKCMP5E    /*!< tim16_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM16_BRK_TIM8_BKIN        TIM_AF1_BKCMP6E    /*!< tim16_brk is connected to TIM8_BKIN */
#define LL_TIM_TIM16_BRK_TIM15_BKIN       TIM_AF1_BKCMP8E    /*!< tim16_brk is connected to TIM15_BKIN */
#define LL_TIM_TIM16_BRK_TIM17_BKIN       TIM_AF1_BKCMP10E   /*!< tim16_brk is connected to TIM17_BKIN */
/**
  * @}
  */
#endif /* TIM16 */

#if defined(TIM17)
/** @defgroup TIM_LL_EC_TIM17_BRK  TIM17 Break Input
  * @{
  */
#define LL_TIM_TIM17_BRK_GPIO             TIM_AF1_BKINE      /*!< tim17_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM17_BRK_COMP1_OUT        TIM_AF1_BKCMP1E    /*!< tim17_brk isconnected to comp1_out */
#define LL_TIM_TIM17_BRK_TIM1_BKIN        TIM_AF1_BKCMP5E    /*!< tim17_brk is connected to TIM1_BKIN */
#define LL_TIM_TIM17_BRK_TIM8_BKIN        TIM_AF1_BKCMP6E    /*!< tim17_brk is connected to TIM8_BKIN */
#define LL_TIM_TIM17_BRK_TIM15_BKIN       TIM_AF1_BKCMP8E    /*!< tim17_brk is connected to TIM15_BKIN */
#define LL_TIM_TIM17_BRK_TIM16_BKIN       TIM_AF1_BKCMP9E    /*!< tim17_brk is connected to TIM16_BKIN */
/**
  * @}
  */
#endif /* TIM17 */


/** @defgroup TIM_LL_EC_TIM1_BRK2  TIM1 Break2 Input
  * @{
  */
#define LL_TIM_TIM1_BRK2_GPIO             TIM_AF2_BK2INE     /*!< tim1_brk2 is connected to TIM1_BKIN2 */
#define LL_TIM_TIM1_BRK2_COMP1_OUT        TIM_AF2_BK2CMP1E   /*!< tim1_brk2 isconnected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM1_BRK2_COMP2_OUT        TIM_AF2_BK2CMP2E   /*!< tim1_brk2 isconnected to comp2_out */
#endif /* COMP2 */
#define LL_TIM_TIM1_BRK2_TIM8_BKIN2       TIM_AF2_BK2CMP6E   /*!< tim1_brk2 is connected to TIM8_BKIN2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_BRK2  TIM8 Break2 Input
  * @{
  */
#define LL_TIM_TIM8_BRK2_GPIO             TIM_AF2_BK2INE     /*!< tim8_brk2 is connected to TIM8_BKIN2 */
#define LL_TIM_TIM8_BRK2_COMP1_OUT        TIM_AF2_BK2CMP1E   /*!< tim8_brk2 isconnected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM8_BRK2_COMP2_OUT        TIM_AF2_BK2CMP2E   /*!< tim8_brk2 isconnected to comp2_out */
#endif /* COMP2 */
#define LL_TIM_TIM8_BRK2_TIM1_BKIN2       TIM_AF2_BK2CMP5E   /*!< tim8_brk2 is connected to TIM1_BKIN2 */
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup TIM_LL_EC_BKIN_POLARITY BKIN POLARITY
  * @{
  */
#define LL_TIM_BREAK_INPUT_SRC_NONINVERTED     0x00000000U              /*!< BRK BKIN input is active high */
#define LL_TIM_BREAK_INPUT_SRC_INVERTED        TIM_AF1_BKINP            /*!< BRK BKIN input is active low */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_AFMODE BREAK AF MODE
  * @{
  */
#define LL_TIM_BREAK_AFMODE_INPUT              0x00000000U              /*!< Break input BRK in input mode */
#define LL_TIM_BREAK_AFMODE_BIDIRECTIONAL      TIM_BDTR_BKBID           /*!< Break input BRK in bidirectional mode */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK2_AFMODE BREAK2 AF MODE
  * @{
  */
#define LL_TIM_BREAK2_AFMODE_INPUT             0x00000000U             /*!< Break2 input BRK2 in input mode */
#define LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL     TIM_BDTR_BK2BID         /*!< Break2 input BRK2 in bidirectional mode */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_BREAK_DELAY BREAK DELAY
  * @{
  */
#define LL_TIM_BREAK_DELAY1                    0x00000000U             /*!< Delayed 1 break */
#define LL_TIM_BREAK_DELAY2                    0x00000001U             /*!< Delayed 2 break */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMABURST_BASEADDR DMA Burst Base Address
  * @{
  */
#define LL_TIM_DMABURST_BASEADDR_CR1            0x00000000U                       /*!< TIMx_CR1 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CR2            TIM_DCR_DBA_0                     /*!< TIMx_CR2 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_SMCR           TIM_DCR_DBA_1                     /*!< TIMx_SMCR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_DIER           (TIM_DCR_DBA_1 |  TIM_DCR_DBA_0)  /*!< TIMx_DIER register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_SR             TIM_DCR_DBA_2                     /*!< TIMx_SR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_EGR            (TIM_DCR_DBA_2 | TIM_DCR_DBA_0)   /*!< TIMx_EGR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCMR1          (TIM_DCR_DBA_2 | TIM_DCR_DBA_1)   /*!< TIMx_CCMR1 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCMR2          (TIM_DCR_DBA_2 | TIM_DCR_DBA_1 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_CCMR2 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCER           TIM_DCR_DBA_3                     /*!< TIMx_CCER register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CNT            (TIM_DCR_DBA_3 | TIM_DCR_DBA_0)   /*!< TIMx_CNT register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_PSC            (TIM_DCR_DBA_3 | TIM_DCR_DBA_1)   /*!< TIMx_PSC register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_ARR            (TIM_DCR_DBA_3 | TIM_DCR_DBA_1 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_ARR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_RCR            (TIM_DCR_DBA_3 | TIM_DCR_DBA_2)   /*!< TIMx_RCR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR1           (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_CCR1 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR2           (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_1)                 /*!< TIMx_CCR2 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR3           (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0) /*!< TIMx_CCR3 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR4           TIM_DCR_DBA_4                     /*!< TIMx_CCR4 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_BDTR           (TIM_DCR_DBA_4 | TIM_DCR_DBA_0)   /*!< TIMx_BDTR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR5           (TIM_DCR_DBA_4 | TIM_DCR_DBA_1)   /*!< TIMx_CCR5 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR6           (TIM_DCR_DBA_4 | TIM_DCR_DBA_1 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_CCR6 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCMR3          (TIM_DCR_DBA_4 | TIM_DCR_DBA_2)   /*!< TIMx_CCMR3 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_DTR2           (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_DTR2 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_ECR            (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_1)                 /*!< TIMx_ECR register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_TISEL          (TIM_DCR_DBA_4 | TIM_DCR_DBA_2 \
                                                 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0) /*!< TIMx_TISEL register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_AF1            (TIM_DCR_DBA_4 | TIM_DCR_DBA_3)   /*!< TIMx_AF1 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_AF2            (TIM_DCR_DBA_4 | TIM_DCR_DBA_3 \
                                                 | TIM_DCR_DBA_0)                 /*!< TIMx_AF2 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCR7           (TIM_DCR_DBA_4 | TIM_DCR_DBA_3 \
                                                 | TIM_DCR_DBA_2)                 /*!< TIMx_CCR7 register is the DMA
                                                                                       base address for DMA burst */
#define LL_TIM_DMABURST_BASEADDR_CCMR4          (TIM_DCR_DBA_4 | TIM_DCR_DBA_3 \
                                                 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1) /*!< TIMx_CCMR4 register is the DMA
                                                                                       base address for DMA burst */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMABURST_LENGTH DMA Burst Length
  * @{
  */
#define LL_TIM_DMABURST_LENGTH_1TRANSFER       0x00000000U                        /*!< Transfer is done to 1 register
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_2TRANSFERS      TIM_DCR_DBL_0                      /*!< Transfer is done to 2 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_3TRANSFERS      TIM_DCR_DBL_1                      /*!< Transfer is done to 3 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_4TRANSFERS      (TIM_DCR_DBL_1 | TIM_DCR_DBL_0)    /*!< Transfer is done to 4 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_5TRANSFERS      TIM_DCR_DBL_2                      /*!< Transfer is done to 5 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_6TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_0)    /*!< Transfer is done to 6 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_7TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_1)    /*!< Transfer is done to 7 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_8TRANSFERS      (TIM_DCR_DBL_2 | TIM_DCR_DBL_1 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 1 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_9TRANSFERS      TIM_DCR_DBL_3                      /*!< Transfer is done to 9 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_10TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_0)    /*!< Transfer is done to 10 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_11TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_1)    /*!< Transfer is done to 11 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_12TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_1 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 12 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_13TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2)    /*!< Transfer is done to 13 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_14TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 14 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_15TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_1)                  /*!< Transfer is done to 15 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_16TRANSFERS     (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_1 | TIM_DCR_DBL_0)  /*!< Transfer is done to 16 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_17TRANSFERS     TIM_DCR_DBL_4                      /*!< Transfer is done to 17 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_18TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_0)    /*!< Transfer is done to 18 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_19TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_1)    /*!< Transfer is done to 19 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_20TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_1 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 20 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_21TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_2)    /*!< Transfer is done to 21 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_22TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 22 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_23TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_1)                  /*!< Transfer is done to 23 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_24TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_2 \
                                                | TIM_DCR_DBL_1 | TIM_DCR_DBL_0)  /*!< Transfer is done to 24 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_25TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3)    /*!< Transfer is done to 25 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_26TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_0)                  /*!< Transfer is done to 26 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_27TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_1)                  /*!< Transfer is done to 27 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_28TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_1 | TIM_DCR_DBL_0)  /*!< Transfer is done to 28 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_29TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_2)                  /*!< Transfer is done to 29 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_30TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_2 | TIM_DCR_DBL_0)  /*!< Transfer is done to 30 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_31TRANSFERS     (TIM_DCR_DBL_4 | TIM_DCR_DBL_3 \
                                                | TIM_DCR_DBL_2 | TIM_DCR_DBL_1)  /*!< Transfer is done to 31 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
#define LL_TIM_DMABURST_LENGTH_32TRANSFERS     TIM_DCR_DBL                        /*!< Transfer is done to 32 registers
                                                                                  starting from the DMA burst base
                                                                                  address */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_DMABURST_SOURCE DMA Burst Source
  * @{
  */
#define LL_TIM_DMABURST_UPD               TIM_DCR_DBSS_0                      /*!< Transfer source is update event */
#define LL_TIM_DMABURST_CC1               TIM_DCR_DBSS_1                      /*!< Transfer source is CC1 event */
#define LL_TIM_DMABURST_CC2               (TIM_DCR_DBSS_1 |  TIM_DCR_DBSS_0)  /*!< Transfer source is CC2 event */
#define LL_TIM_DMABURST_CC3               TIM_DCR_DBSS_2                      /*!< Transfer source is CC3 event */
#define LL_TIM_DMABURST_CC4               (TIM_DCR_DBSS_2 | TIM_DCR_DBSS_0)   /*!< Transfer source is CC4 event */
#define LL_TIM_DMABURST_COM               (TIM_DCR_DBSS_2 | TIM_DCR_DBSS_1)   /*!< Transfer source is COM event */
#define LL_TIM_DMABURST_TRGI              (TIM_DCR_DBSS_2 | TIM_DCR_DBSS_1 \
                                           | TIM_DCR_DBSS_0)                  /*!< Transfer source is trigger event */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_TI1  TIM1 Input Ch1
  * @{
  */
#define LL_TIM_TIM1_TI1_GPIO                 0x00000000U                                                     /*!< tim1_ti1 is connected to TIM1_CH1 */
#define LL_TIM_TIM1_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim1_ti1 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM1_TI1_COMP2_OUT            TIM_TISEL_TI1SEL_1                                              /*!< tim1_ti1 is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_TI2  TIM1 Input Ch2
  * @{
  */
#define LL_TIM_TIM1_TI2_GPIO                 0x00000000U                                                     /*!< tim1_ti2 is connected to TIM1_CH2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_TI3  TIM1 Input Ch3
  * @{
  */
#define LL_TIM_TIM1_TI3_GPIO                 0x00000000U                                                     /*!< tim1_ti3 is connected to TIM1_CH3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM1_TI4  TIM1 Input Ch4
  * @{
  */
#define LL_TIM_TIM1_TI4_GPIO                 0x00000000U                                                     /*!< tim1_ti4 is connected to TIM1_CH4 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_TI1  TIM2 Input Ch1
  * @{
  */
#define LL_TIM_TIM2_TI1_GPIO                 0x00000000U                                                     /*!< tim2_ti1 is connected to TIM2_CH1 */
#define LL_TIM_TIM2_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim2_ti1 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM2_TI1_COMP2_OUT            TIM_TISEL_TI1SEL_1                                              /*!< tim2_ti1 is connected to comp2_out */
#elif defined(ETH1)
#define LL_TIM_TIM2_TI1_ETH1_PTP_PPS_OUT     TIM_TISEL_TI1SEL_1                                              /*!< tim2_ti1 is connected to eth1_ptp_pps_out */
#endif /* COMP2 && !COMP3 */
#define LL_TIM_TIM2_TI1_LSI                  (TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)                       /*!< tim2_ti1 is connected to LSI */
#define LL_TIM_TIM2_TI1_LSE                  TIM_TISEL_TI1SEL_2                                              /*!< tim2_ti1 is connected to LSE */
#define LL_TIM_TIM2_TI1_RTC_WUT_TRG          (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_0)                       /*!< tim2_ti1 is connected to rtc_wut_trg */
#if defined(TIM5)
#define LL_TIM_TIM2_TI1_TIM5_CH1             (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1)                       /*!< tim2_ti1 is connected to TIM5_CH1 */
#endif /* TIM5 */
#if defined(FDCAN1)
#define LL_TIM_TIM2_TI1_FDCAN1_RXEOF_EVT     (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim2_ti1 is connected to fdcan1_rxeof_evt */
#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_TI2  TIM2 Input Ch2
  * @{
  */
#define LL_TIM_TIM2_TI2_GPIO                 0x00000000U                                                     /*!< tim2_ti2 is connected to TIM2_CH2 */
#define LL_TIM_TIM2_TI2_HSE_RTC              (TIM_TISEL_TI2SEL_1 | TIM_TISEL_TI2SEL_0)                       /*!< tim2_ti2 is connected to hse_1M_ck */
#define LL_TIM_TIM2_TI2_MCO1                 TIM_TISEL_TI2SEL_2                                              /*!< tim2_ti2 is connected to MCO1 */
#define LL_TIM_TIM2_TI2_MCO2                 (TIM_TISEL_TI2SEL_2 | TIM_TISEL_TI2SEL_0)                       /*!< tim2_ti2 is connected to MCO2 */
#if defined(FDCAN1)
#define LL_TIM_TIM2_TI2_FDCAN1_TXEOF_EVT     (TIM_TISEL_TI2SEL_2 | TIM_TISEL_TI2SEL_1 | TIM_TISEL_TI2SEL_0)  /*!< tim2_ti2 is connected to fdcan1_txeof_evt */
#endif /* FDCAN1 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_TI3  TIM2 Input Ch3
  * @{
  */
#define LL_TIM_TIM2_TI3_GPIO                 0x00000000U                                                     /*!< tim2_ti3 is connected to TIM2_CH3 */
#if defined(FDCAN2)
#define LL_TIM_TIM2_TI3_FDCAN2_RXEOF_EVT     (TIM_TISEL_TI3SEL_2 | TIM_TISEL_TI3SEL_1 | TIM_TISEL_TI3SEL_0)  /*!< tim2_ti3 is connected to fdcan2_rxeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_TI4  TIM2 Input Ch4
  * @{
  */
#define LL_TIM_TIM2_TI4_GPIO                 0x00000000U                                                     /*!< tim2_ti4 is connected to TIM2_CH4 */
#define LL_TIM_TIM2_TI4_COMP1_OUT            TIM_TISEL_TI4SEL_0                                              /*!< tim2_ti4 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM2_TI4_COMP2_OUT            TIM_TISEL_TI4SEL_1                                              /*!< tim2_ti4 is connected to comp2_out */
#endif /* COMP2 */
#if defined(FDCAN2)
#define LL_TIM_TIM2_TI4_FDCAN2_TXEOF_EVT     (TIM_TISEL_TI4SEL_2 | TIM_TISEL_TI4SEL_1 | TIM_TISEL_TI4SEL_0)  /*!< tim2_ti4 is connected to fdcan2_txeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

#if defined(TIM3)
/** @defgroup TIM_LL_EC_TIM3_TI1  TIM3 Input Ch1
  * @{
  */
#define LL_TIM_TIM3_TI1_GPIO                 0x00000000U                                                     /*!< tim3_ti1 is connected to TIM3_CH1 */
#define LL_TIM_TIM3_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim3_ti1 is connected to comp1_out */
#if defined(FDCAN2)
#define LL_TIM_TIM3_TI1_ETH1_PTP_PPS_OUT     TIM_TISEL_TI1SEL_1                                              /*!< tim3_ti1 is connected to eth1_ptp_pps_out */
#define LL_TIM_TIM3_TI1_FDCAN2_RXEOF_EVT     (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim3_ti1 is connected to fdcan2_rxeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM3_TI2  TIM3 Input Ch2
  * @{
  */
#define LL_TIM_TIM3_TI2_GPIO                 0x00000000U                                                     /*!< tim3_ti2 is connected to TIM3_CH2 */
#if defined(FDCAN2)
#define LL_TIM_TIM3_TI2_FDCAN2_TXEOF_EVT     (TIM_TISEL_TI2SEL_2 | TIM_TISEL_TI2SEL_1 | TIM_TISEL_TI2SEL_0)  /*!< tim3_ti2 is connected to fdcan2_txeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM3_TI3  TIM3 Input Ch3
  * @{
  */
#define LL_TIM_TIM3_TI3_GPIO                 0x00000000U                                                     /*!< tim3_ti3 is connected to TIM3_CH3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM3_TI4  TIM3 Input Ch4
  * @{
  */
#define LL_TIM_TIM3_TI4_GPIO                 0x00000000U                                                     /*!< tim3_ti4 is connected to TIM3_CH4 */
/**
  * @}
  */

#endif /* TIM3 */

#if defined(TIM4)
/** @defgroup TIM_LL_EC_TIM4_TI1  TIM4 Input Ch1
  * @{
  */
#define LL_TIM_TIM4_TI1_GPIO                 0x00000000U                                                     /*!< tim4_ti1 is connected to TIM4_CH1 */
#define LL_TIM_TIM4_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim4_ti1 is connected to comp1_out */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM4_TI2  TIM4 Input Ch2
  * @{
  */
#define LL_TIM_TIM4_TI2_GPIO                 0x00000000U                                                     /*!< tim4_ti2 is connected to TIM4_CH2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM4_TI3  TIM4 Input Ch3
  * @{
  */
#define LL_TIM_TIM4_TI3_GPIO                 0x00000000U                                                     /*!< tim4_ti3 is connected to TIM4_CH3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM4_TI4  TIM4 Input Ch4
  * @{
  */
#define LL_TIM_TIM4_TI4_GPIO                 0x00000000U                                                     /*!< tim4_ti4 is connected to TIM4_CH4 */
/**
  * @}
  */
#endif /* TIM4 */

#if defined(TIM5)
/** @defgroup TIM_LL_EC_TIM5_TI1  TIM5 Input Ch1
  * @{
  */
#define LL_TIM_TIM5_TI1_GPIO                 0x00000000U                                                     /*!< tim5_ti1 is connected to TIM5_CH1 */
#define LL_TIM_TIM5_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim5_ti1 is connected to comp1_out */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM5_TI2  TIM5 Input Ch2
  * @{
  */
#define LL_TIM_TIM5_TI2_GPIO                 0x00000000U                                                     /*!< tim5_ti2 is connected to TIM5_CH2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM5_TI3  TIM5 Input Ch3
  * @{
  */
#define LL_TIM_TIM5_TI3_GPIO                 0x00000000U                                                     /*!< tim5_ti3 is connected to TIM5_CH3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM5_TI4  TIM5 Input Ch4
  * @{
  */
#define LL_TIM_TIM5_TI4_GPIO                 0x00000000U                                                     /*!< tim5_ti4 is connected to TIM5_CH4 */
/**
  * @}
  */
#endif /* TIM5 */

/** @defgroup TIM_LL_EC_TIM8_TI1  TIM8 Input Ch1
  * @{
  */
#define LL_TIM_TIM8_TI1_GPIO                 0x00000000U                                                     /*!< tim8_ti1 is connected to TIM8_CH1 */
#define LL_TIM_TIM8_TI1_COMP1_OUT            TIM_TISEL_TI1SEL_0                                              /*!< tim8_ti1 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM8_TI1_COMP2_OUT            TIM_TISEL_TI1SEL_1                                              /*!< tim8_ti1 is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_TI2  TIM8 Input Ch2
  * @{
  */
#define LL_TIM_TIM8_TI2_GPIO                 0x00000000U                                                     /*!< tim8_ti2 is connected to TIM8_CH2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_TI3  TIM8 Input Ch3
  * @{
  */
#define LL_TIM_TIM8_TI3_GPIO                 0x00000000U                                                     /*!< tim8_ti3 is connected to TIM8_CH3 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM8_TI4  TIM8 Input Ch4
  * @{
  */
#define LL_TIM_TIM8_TI4_GPIO                 0x00000000U                                                     /*!< tim8_ti4 is connected to TIM8_CH4 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM12_TI1  TIM12 Input Ch1
  * @{
  */
#define LL_TIM_TIM12_TI1_GPIO                0x00000000U                                                     /*!< tim12_ti1 is connected to TIM12_CH1 */
#define LL_TIM_TIM12_TI1_COMP1_OUT           TIM_TISEL_TI1SEL_0                                              /*!< tim12_ti1 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM12_TI1_COMP2_OUT           TIM_TISEL_TI1SEL_1                                              /*!< tim12_ti1 is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
#define LL_TIM_TIM12_TI1_MCO1                (TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)                       /*!< tim12_ti1 is connected to MCO1 */
#define LL_TIM_TIM12_TI1_MCO2                TIM_TISEL_TI1SEL_2                                              /*!< tim12_ti1 is connected to MCO2 */
#define LL_TIM_TIM12_TI1_HSE_RTC             (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_0)                       /*!< tim12_ti1 is connected to hse_1M_ck */
#define LL_TIM_TIM12_TI1_I3C1_IBI_ACK        (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim12_ti1 is connected to i3c1_ibi_ack */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM12_TI2  TIM12 Input Ch2
  * @{
  */
#define LL_TIM_TIM12_TI2_GPIO                0x00000000U                                                     /*!< tim12_ti2 is connected to TIM12_CH2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_TI1  TIM15 Input Ch1
  * @{
  */
#define LL_TIM_TIM15_TI1_GPIO                0x00000000U                                                     /*!< tim15_ti1 is connected to TIM15_CH1 */
#define LL_TIM_TIM15_TI1_COMP1_OUT           TIM_TISEL_TI1SEL_0                                              /*!< tim15_ti1 is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM15_TI1_COMP2_OUT           TIM_TISEL_TI1SEL_1                                              /*!< tim15_ti1 is connected to comp2_out */
#endif /* COMP2 && !COMP3 */
#define LL_TIM_TIM15_TI1_LSE                 (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_0)                       /*!< tim15_ti1 is connected to LSE */
#if defined(FDCAN2)
#define LL_TIM_TIM15_TI1_FDCAN2_RXEOF_EVT    (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim15_ti1 is connected to fdcan2_rxeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_TI2  TIM15 Input Ch2
  * @{
  */
#define LL_TIM_TIM15_TI2_GPIO                0x00000000U                                                     /*!< tim15_ti2 is connected to TIM15_CH2 */
#if defined(FDCAN2)
#define LL_TIM_TIM15_TI2_FDCAN2_TXEOF_EVT    (TIM_TISEL_TI2SEL_2 | TIM_TISEL_TI2SEL_1 | TIM_TISEL_TI2SEL_0)  /*!< tim15_ti2 is connected to fdcan2_txeof_evt */
#endif /* FDCAN2 */
/**
  * @}
  */

#if defined(TIM16)
/** @defgroup TIM_LL_EC_TIM16_TI1  TIM16 Input Ch1
  * @{
  */
#define LL_TIM_TIM16_TI1_GPIO                0x00000000U                                                     /*!< tim16_ti1 is connected to TIM16_CH1 */
#define LL_TIM_TIM16_TI1_COMP1_OUT           TIM_TISEL_TI1SEL_0                                              /*!< tim16_ti1 is connected to comp1_out */
#define LL_TIM_TIM16_TI1_LSI                 (TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)                       /*!< tim16_ti1 is connected to LSI */
#define LL_TIM_TIM16_TI1_LSE                 TIM_TISEL_TI1SEL_2                                              /*!< tim16_ti1 is connected to LSE */
#define LL_TIM_TIM16_TI1_RTC_WUT_TRG         (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_0)                       /*!< tim16_ti1 is connected to rtc_wut_trg */
#define LL_TIM_TIM16_TI1_MCO1                (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1)                       /*!< tim16_ti1 is connected to MCO1 */
#define LL_TIM_TIM16_TI1_MCO2                (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim16_ti1 is connected to MCO2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM17_TI1  TIM17 Input Ch1
  * @{
  */
#define LL_TIM_TIM17_TI1_GPIO                0x00000000U                                                     /*!< tim17_ti1 is connected to TIM17_CH1 */
#define LL_TIM_TIM17_TI1_COMP1_OUT           TIM_TISEL_TI1SEL_0                                              /*!< tim17_ti1 is connected to comp1_out */
#define LL_TIM_TIM17_TI1_HSE_RTC             TIM_TISEL_TI1SEL_2                                              /*!< tim17_ti1 is connected to hse_rtc_ck */
#define LL_TIM_TIM17_TI1_MCO1                (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_0)                       /*!< tim17_ti1 is connected to MCO1 */
#define LL_TIM_TIM17_TI1_MCO2                (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1)                       /*!< tim17_ti1 is connected to MCO2 */
#define LL_TIM_TIM17_TI1_I3C1_IBI_ACK        (TIM_TISEL_TI1SEL_2 | TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0)  /*!< tim17_ti1 is connected to i3c1_ibi_ack */
/**
  * @}
  */
#endif /* TIM16 */


/** @defgroup TIM_LL_EC_OCREF_CLR_INT OCREF clear input selection
  * @{
  */
/** @defgroup TIM_LL_EC_TIM1_OCREF_CLR_INT  TIM1 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM1_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim1_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM1_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim1_ocref_clr is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM1_OCREF_CLR_INT_COMP2_OUT    TIM_AF2_OCRSEL_0     /*!< tim1_ocref_clr is connected to comp2_out */
#endif /* COMP2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM2_OCREF_CLR_INT  TIM2 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM2_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim2_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM2_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim2_ocref_clr is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM2_OCREF_CLR_INT_COMP2_OUT    TIM_AF2_OCRSEL_0     /*!< tim2_ocref_clr is connected to comp2_out */
#endif /* COMP2 */
/**
  * @}
  */

#if defined(TIM3)
/** @defgroup TIM_LL_EC_TIM3_OCREF_CLR_INT  TIM3 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM3_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim3_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM3_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim3_ocref_clr is connected to comp1_out */
/**
  * @}
  */
#endif /* TIM3 */

#if defined(TIM4)
/** @defgroup TIM_LL_EC_TIM4_OCREF_CLR_INT  TIM4 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM4_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim4_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM4_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim4_ocref_clr is connected to comp1_out */
/**
  * @}
  */
#endif /* TIM4 */

#if defined(TIM5)
/** @defgroup TIM_LL_EC_TIM5_OCREF_CLR_INT  TIM5 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM5_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim5_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM5_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim5_ocref_clr is connected to comp1_out */
/**
  * @}
  */
#endif /* TIM5 */

/** @defgroup TIM_LL_EC_TIM8_OCREF_CLR_INT  TIM8 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM8_OCREF_CLR_INT_ETR          TIM_SMCR_OCCS        /*!< tim8_ocref_clr is connected to tim_etrf */
#define LL_TIM_TIM8_OCREF_CLR_INT_COMP1_OUT    0x00000000U          /*!< tim8_ocref_clr is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM8_OCREF_CLR_INT_COMP2_OUT    TIM_AF2_OCRSEL_0     /*!< tim8_ocref_clr is connected to comp2_out */
#endif /* COMP2 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM15_OCREF_CLR_INT  TIM15 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM15_OCREF_CLR_INT_COMP1_OUT   0x00000000U          /*!< tim15_ocref_clr is connected to comp1_out */
#if defined(COMP2)
#define LL_TIM_TIM15_OCREF_CLR_INT_COMP2_OUT   TIM_AF2_OCRSEL_0     /*!< tim15_ocref_clr is connected to comp2_out */
#endif /* COMP2 */
/**
  * @}
  */

#if defined(TIM16)
/** @defgroup TIM_LL_EC_TIM16_OCREF_CLR_INT  TIM16 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM16_OCREF_CLR_INT_COMP1_OUT   0x00000000U          /*!< tim16_ocref_clr is connected to comp1_out */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_TIM17_OCREF_CLR_INT  TIM16 OCRef Clear Input
  * @{
  */
#define LL_TIM_TIM17_OCREF_CLR_INT_COMP1_OUT   0x00000000U          /*!< tim17_ocref_clr is connected to comp1_out */
/**
  * @}
  */
#endif /* TIM16 */

/**
  * @}
  */

/** @defgroup TIM_LL_EC_INDEX_DIR index direction selection
  * @{
  */
#define LL_TIM_INDEX_UP_DOWN     0x00000000U         /*!< Index resets the counter whatever the direction */
#define LL_TIM_INDEX_UP          TIM_ECR_IDIR_0      /*!< Index resets the counter when up-counting only */
#define LL_TIM_INDEX_DOWN        TIM_ECR_IDIR_1      /*!< Index resets the counter when down-counting only */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_INDEX_BLANK index blanking selection
  * @{
  */
#define LL_TIM_INDEX_BLANK_ALWAYS     0x00000000U         /*!< Index always active */
#define LL_TIM_INDEX_BLANK_TI3        TIM_ECR_IBLK_0      /*!< Index disabled when TI3 input is active,
                                                           as per CC3P bitfield */
#define LL_TIM_INDEX_BLANK_TI4        TIM_ECR_IBLK_1      /*!< Index disabled when TI4 input is active,
                                                           as per CC4P bitfield */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_INDEX_POSITION index positioning selection
  * @{
  */
#define LL_TIM_INDEX_POSITION_DOWN_DOWN    0x00000000U                  /*!< Index resets the counter when AB = 00 */
#define LL_TIM_INDEX_POSITION_DOWN_UP      TIM_ECR_IPOS_0               /*!< Index resets the counter when AB = 01 */
#define LL_TIM_INDEX_POSITION_UP_DOWN      TIM_ECR_IPOS_1               /*!< Index resets the counter when AB = 10 */
#define LL_TIM_INDEX_POSITION_UP_UP        (TIM_ECR_IPOS_1 \
                                            | TIM_ECR_IPOS_0)           /*!< Index resets the counter when AB = 11 */
#define LL_TIM_INDEX_POSITION_DOWN         0x00000000U                  /*!< Index resets the counter when clock is 0 */
#define LL_TIM_INDEX_POSITION_UP           TIM_ECR_IPOS_0               /*!< Index resets the counter when clock is 1 */
/**
  * @}
  */

/** @defgroup TIM_LL_EC_FIRST_INDEX first index selection
  * @{
  */
#define LL_TIM_INDEX_ALL           0x00000000U                           /*!< Index is always active */
#define LL_TIM_INDEX_FIRST_ONLY    TIM_ECR_FIDX                          /*!< The first Index only resets the counter */
/**
  * @}
  */
/** @defgroup TIM_LL_EC_PWPRSC Pulse on compare pulse width prescaler
  * @{
  */
#define LL_TIM_PWPRSC_DIV1     0x00000000U                           /*!< Pulse on compare pulse width prescaler 1 */
#define LL_TIM_PWPRSC_DIV2     TIM_ECR_PWPRSC_0                      /*!< Pulse on compare pulse width prescaler 2 */
#define LL_TIM_PWPRSC_DIV4     TIM_ECR_PWPRSC_1                      /*!< Pulse on compare pulse width prescaler 4 */
#define LL_TIM_PWPRSC_DIV8     (TIM_ECR_PWPRSC_1 | TIM_ECR_PWPRSC_0) /*!< Pulse on compare pulse width prescaler 8 */
#define LL_TIM_PWPRSC_DIV16    TIM_ECR_PWPRSC_2                      /*!< Pulse on compare pulse width prescaler 16 */
#define LL_TIM_PWPRSC_DIV32    (TIM_ECR_PWPRSC_2 | TIM_ECR_PWPRSC_0) /*!< Pulse on compare pulse width prescaler 32 */
#define LL_TIM_PWPRSC_DIV64    (TIM_ECR_PWPRSC_2 | TIM_ECR_PWPRSC_1) /*!< Pulse on compare pulse width prescaler 64 */
#define LL_TIM_PWPRSC_DIV128   (TIM_ECR_PWPRSC_2 | TIM_ECR_PWPRSC_1 \
                                | TIM_ECR_PWPRSC_0)                  /*!< Pulse on compare pulse width prescaler 128 */
/**
  * @}
  */


/** @defgroup TIM_LL_EC_SW_EVENT Software Event
  * @{
  */
#define LL_TIM_SW_EVENT_UPD       TIM_EGR_UG      /*!< Update generation */
#define LL_TIM_SW_EVENT_CC1       TIM_EGR_CC1G    /*!< Capture/Compare 1 generation */
#define LL_TIM_SW_EVENT_CC2       TIM_EGR_CC2G    /*!< Capture/Compare 2 generation */
#define LL_TIM_SW_EVENT_CC3       TIM_EGR_CC3G    /*!< Capture/Compare 3 generation */
#define LL_TIM_SW_EVENT_CC4       TIM_EGR_CC4G    /*!< Capture/Compare 4 generation */
#define LL_TIM_SW_EVENT_COM       TIM_EGR_COMG    /*!< Commutation generation */
#define LL_TIM_SW_EVENT_TRGI      TIM_EGR_TG      /*!< Trigger generation */
#define LL_TIM_SW_EVENT_BRK       TIM_EGR_BG      /*!< Break generation */
#define LL_TIM_SW_EVENT_BRK2      TIM_EGR_B2G     /*!< Break 2 generation */

/** Legacy definitions for compatibility purpose
  * @cond LEGACY_DEFINITIONS
  */
#define LL_TIM_BKIN_SOURCE_DFBK  LL_TIM_BKIN_SOURCE_DF1BK
/**
  * @endcond
  */

/**
  * @}
  */

/* Exported macro ----------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Macros LL TIM Macros
  * @{
  */

/** @defgroup TIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/**
  * @brief  Write a value in TIM register.
  * @param  instance TIM Instance
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_TIM_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value in TIM register.
  * @param  instance TIM Instance
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_TIM_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)
/**
  * @}
  */

/**
  * @brief  HELPER macro that retrieves the UIFCPY flag from the counter value.
  * @param  cnt Counter value
  * @note   e.g., LL_TIM_GETFLAG_UIFCPY(@ref LL_TIM_GetCounter()).
  * @note   Relevant only if UIF flag remapping has been enabled (UIF status bit is copied
  *         to TIMx_CNT register bit 31).
  * @retval UIF status bit
  */
#define LL_TIM_GETFLAG_UIFCPY(cnt)  \
  (STM32_READ_BIT((cnt), TIM_CNT_UIFCPY) >> TIM_CNT_UIFCPY_Pos)

/**
  * @brief  HELPER macro calculating DTG[0:7] in the TIMx_BDTR register to achieve the requested dead time duration.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  clk_div This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV8
  * @param  dt deadtime duration (in ns)
  * @note   e.g., LL_TIM_CALC_DEADTIME(80000000, @ref LL_TIM_GetClockDivision(), 120).
  * @retval DTG[0:7]
  */
#define LL_TIM_CALC_DEADTIME(tim_clk, clk_div, dt)  \
  ( (((uint64_t)((dt)*1000U)) < ((LL_TIM_DT_DELAY_1+1U) * LL_TIM_CALC_DTS((tim_clk), (clk_div))))    ?  \
    (uint8_t)(((uint64_t)((dt)*1000U) / LL_TIM_CALC_DTS((tim_clk), (clk_div)))  & LL_TIM_DT_DELAY_1) :      \
    (((uint64_t)((dt)*1000U)) < ((64U + (LL_TIM_DT_DELAY_2+1U)) * 2U * LL_TIM_CALC_DTS((tim_clk), (clk_div))))  ?  \
    (uint8_t)(LL_TIM_DT_RANGE_2 | ((uint8_t)((uint8_t)((((uint64_t)((dt)*1000U))/ LL_TIM_CALC_DTS((tim_clk),   \
                                                        (clk_div))) >> 1U) - (uint8_t) 64) & LL_TIM_DT_DELAY_2)) :\
    (((uint64_t)((dt)*1000U)) < ((32U + (LL_TIM_DT_DELAY_3+1U)) * 8U * LL_TIM_CALC_DTS((tim_clk), (clk_div))))  ?  \
    (uint8_t)(LL_TIM_DT_RANGE_3 | ((uint8_t)((uint8_t)(((((uint64_t)(dt)*1000U))/ LL_TIM_CALC_DTS((tim_clk),  \
                                                        (clk_div))) >> 3U) - (uint8_t) 32) & LL_TIM_DT_DELAY_3)) :\
    (((uint64_t)((dt)*1000U)) < ((32U + (LL_TIM_DT_DELAY_4+1U)) * 16U * LL_TIM_CALC_DTS((tim_clk), (clk_div)))) ?  \
    (uint8_t)(LL_TIM_DT_RANGE_4 | ((uint8_t)((uint8_t)(((((uint64_t)(dt)*1000U))/ LL_TIM_CALC_DTS((tim_clk),  \
                                                        (clk_div))) >> 4U) - (uint8_t) 32) & LL_TIM_DT_DELAY_4)) :\
    0U)

/**
  * @brief  HELPER macro calculating the prescaler value to achieve the required counter clock frequency.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  cnt_clk counter clock frequency (in Hz)
  * @note   e.g., LL_TIM_CALC_PSC(80000000, 1000000).
  * @retval Prescaler value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_PSC(tim_clk, cnt_clk)   \
  (((tim_clk) >= (cnt_clk)) ? (uint32_t)((((tim_clk) + (cnt_clk)/2U)/(cnt_clk)) - 1U) : 0U)

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the required output signal frequency.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  freq output signal frequency (in Hz)
  * @note   e.g., LL_TIM_CALC_ARR(1000000, @ref LL_TIM_GetPrescaler(), 10000).
  * @retval Auto-reload value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_ARR(tim_clk, psc, freq) \
  ((((tim_clk)/((psc) + 1U)) >= (freq)) ? (((tim_clk)/((freq) * ((psc) + 1U))) - 1U) : 0U)

/**
  * @brief  HELPER macro calculating the auto-reload value, with dithering feature enabled, to achieve the required
  *         output signal frequency.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  freq output signal frequency (in Hz)
  * @note   e.g., LL_TIM_CALC_ARR_DITHER(1000000, @ref LL_TIM_GetPrescaler(), 10000).
  * @retval Auto-reload value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_ARR_DITHER(tim_clk, psc, freq) \
  ((((tim_clk)/((psc) + 1U)) >= (freq)) ? \
   (uint32_t)((((uint64_t)(tim_clk) * 16U/((freq) * ((psc) + 1U))) - 16U)) : 0U)

/**
  * @brief  HELPER macro calculating the compare value required to achieve the required timer output compare
  *         active/inactive delay.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  delay timer output compare active/inactive delay (in us)
  * @note   e.g., LL_TIM_CALC_DELAY(1000000, @ref LL_TIM_GetPrescaler(), 10).
  * @retval Compare value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_DELAY(tim_clk, psc, delay)  \
  ((uint32_t)(((uint64_t)(tim_clk) * (uint64_t)(delay)) \
              / ((uint64_t)1000000U * (uint64_t)((psc) + 1U))))

/**
  * @brief  HELPER macro calculating the compare value, with dithering feature enabled, to achieve the required timer
  *         output compare active/inactive delay.
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  delay timer output compare active/inactive delay (in us)
  * @note   e.g., LL_TIM_CALC_DELAY_DITHER(1000000, @ref LL_TIM_GetPrescaler(), 10).
  * @retval Compare value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_DELAY_DITHER(tim_clk, psc, delay)  \
  ((uint32_t)(((uint64_t)(tim_clk) * (uint64_t)(delay) * 16U) \
              / ((uint64_t)1000000U * (uint64_t)((psc) + 1U))))

/**
  * @brief  HELPER macro calculating the auto-reload value to achieve the required pulse duration
  *         (when the timer operates in one pulse mode).
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  delay timer output compare active/inactive delay (in us)
  * @param  pulse pulse duration (in us)
  * @note   e.g., LL_TIM_CALC_PULSE(1000000, @ref LL_TIM_GetPrescaler(), 10, 20).
  * @retval Auto-reload value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_PULSE(tim_clk, psc, delay, pulse)  \
  ((uint32_t)(LL_TIM_CALC_DELAY((tim_clk), (psc), (pulse)) \
              + LL_TIM_CALC_DELAY((tim_clk), (psc), (delay))))

/**
  * @brief  HELPER macro calculating the auto-reload value, with dithering feature enabled, to achieve the required
  *         pulse duration (when the timer operates in one pulse mode).
  * @param  tim_clk timer input clock frequency (in Hz)
  * @param  psc prescaler
  * @param  delay timer output compare active/inactive delay (in us)
  * @param  pulse pulse duration (in us)
  * @note   e.g., LL_TIM_CALC_PULSE_DITHER(1000000, @ref LL_TIM_GetPrescaler(), 10, 20).
  * @retval Auto-reload value (between Min_Data=0 and Max_Data=65535)
  */
#define LL_TIM_CALC_PULSE_DITHER(tim_clk, psc, delay, pulse)  \
  ((uint32_t)(LL_TIM_CALC_DELAY_DITHER((tim_clk), (psc), (pulse)) \
              + LL_TIM_CALC_DELAY_DITHER((tim_clk), (psc), (delay))))

/**
  * @brief  HELPER macro retrieving the ratio of the input capture prescaler.
  * @param  ic_psc This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  * @note   e.g., LL_TIM_GET_ICPSC_RATIO(@ref LL_TIM_IC_GetPrescaler()).
  * @retval Input capture prescaler ratio (1, 2, 4, or 8).
  */
#define LL_TIM_GET_ICPSC_RATIO(ic_psc)  \
  ((uint32_t)(0x01U << (((ic_psc) >> 16U) >> TIM_CCMR1_IC1PSC_Pos)))


/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Functions LL TIM Functions
  * @{
  */

/** @defgroup TIM_LL_EF_Time_Base Time Base configuration
  * @{
  */
/**
  * @brief  Enable timer counter.
  * @rmtoll
  *  CR1          CEN           LL_TIM_EnableCounter
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableCounter(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_CEN);
}

/**
  * @brief  Disable timer counter.
  * @rmtoll
  *  CR1          CEN           LL_TIM_DisableCounter
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableCounter(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_CEN);
}

/**
  * @brief  Indicates whether the timer counter is enabled.
  * @rmtoll
  *  CR1          CEN           LL_TIM_IsEnabledCounter
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledCounter(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_CEN) == (TIM_CR1_CEN)) ? 1UL : 0UL);
}

/**
  * @brief  Enable update event generation.
  * @rmtoll
  *  CR1          UDIS          LL_TIM_EnableUpdateEvent
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableUpdateEvent(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_UDIS);
}

/**
  * @brief  Disable update event generation.
  * @rmtoll
  *  CR1          UDIS          LL_TIM_DisableUpdateEvent
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableUpdateEvent(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_UDIS);
}

/**
  * @brief  Indicates whether update event generation is enabled.
  * @rmtoll
  *  CR1          UDIS          LL_TIM_IsEnabledUpdateEvent
  * @param  timx Timer instance
  * @retval Inverted state of bit (0 or 1).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledUpdateEvent(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_UDIS) == 0U) ? 1UL : 0UL);
}

/**
  * @brief  Set update event source.
  * @rmtoll
  *  CR1          URS           LL_TIM_SetUpdateSource
  * @param  timx Timer instance
  * @param  update_source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  * @note Update event source set to LL_TIM_UPDATESOURCE_REGULAR: any of the following events
  *       generate an update interrupt or DMA request if enabled:
  *        - Counter overflow/underflow
  *        - Setting the UG bit
  *        - Update generation through the slave mode controller
  * @note Update event source set to LL_TIM_UPDATESOURCE_COUNTER: only counter
  *       overflow/underflow generates an update interrupt or DMA request if enabled.
  */
__STATIC_INLINE void LL_TIM_SetUpdateSource(TIM_TypeDef *timx, uint32_t update_source)
{
  STM32_MODIFY_REG(timx->CR1, TIM_CR1_URS, update_source);
}

/**
  * @brief  Get actual event update source.
  * @rmtoll
  *  CR1          URS           LL_TIM_GetUpdateSource
  * @param  timx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  */
__STATIC_INLINE uint32_t LL_TIM_GetUpdateSource(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_URS));
}

/**
  * @brief  Enable one-pulse mode (OPM).
  * @rmtoll
  *  CR1          OPM           LL_TIM_EnableOnePulseMode
  * @param  timx Timer instance
  * @note   When OPM is set, the timer stops counting at the next update event (UEV).
  */
__STATIC_INLINE void LL_TIM_EnableOnePulseMode(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_OPM);
}

/**
  * @brief  Disable one-pulse mode (OPM).
  * @rmtoll
  *  CR1          OPM           LL_TIM_DisableOnePulseMode
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableOnePulseMode(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_OPM);
}

/**
  * @brief  Indicates whether one-pulse mode (OPM) is enabled.
  * @rmtoll
  *  CR1          OPM           LL_TIM_IsEnabledOnePulseMode
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledOnePulseMode(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_OPM) == (TIM_CR1_OPM)) ? 1UL : 0UL);
}

/**
  * @brief  Set the timer counter counting mode.
  * @rmtoll
  *  CR1          DIR           LL_TIM_SetCounterMode \n
  *  CR1          CMS           LL_TIM_SetCounterMode
  * @param  timx Timer instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  * @note Macro IS_TIM_COUNTER_MODE_SELECT_INSTANCE(timx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @note Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *       requires a timer reset to avoid unexpected direction
  *       due to DIR bit readonly in center aligned mode.
  */
__STATIC_INLINE void LL_TIM_SetCounterMode(TIM_TypeDef *timx, uint32_t mode)
{
  STM32_MODIFY_REG(timx->CR1, (TIM_CR1_DIR | TIM_CR1_CMS), mode);
}

/**
  * @brief  Get actual counter mode.
  * @rmtoll
  *  CR1          DIR           LL_TIM_GetCounterMode \n
  *  CR1          CMS           LL_TIM_GetCounterMode
  * @param  timx Timer instance
  * @note Macro IS_TIM_COUNTER_MODE_SELECT_INSTANCE(timx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  */
__STATIC_INLINE uint32_t LL_TIM_GetCounterMode(const TIM_TypeDef *timx)
{
  uint32_t counter_mode;

  counter_mode = (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_CMS));

  if (counter_mode == 0U)
  {
    counter_mode = (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_DIR));
  }

  return counter_mode;
}

/**
  * @brief  Enable auto-reload (ARR) preload.
  * @rmtoll
  *  CR1          ARPE          LL_TIM_EnableARRPreload
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableARRPreload(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_ARPE);
}

/**
  * @brief  Disable auto-reload (ARR) preload.
  * @rmtoll
  *  CR1          ARPE          LL_TIM_DisableARRPreload
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableARRPreload(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_ARPE);
}

/**
  * @brief  Indicates whether auto-reload (ARR) preload is enabled.
  * @rmtoll
  *  CR1          ARPE          LL_TIM_IsEnabledARRPreload
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledARRPreload(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_ARPE) == (TIM_CR1_ARPE)) ? 1UL : 0UL);
}

/**
  * @brief  Set the division ratio between the timer kernel clock (tim_ker_ck) and the DTS sampling clock (DTS_ck)
  *         used by the dead-time generators (when supported) and the break/break2 filters.
  * @rmtoll
  *  CR1          CKD           LL_TIM_SetClockDivision
  * @param  timx Timer instance
  * @param  clock_division This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(timx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV8
  */
__STATIC_INLINE void LL_TIM_SetClockDivision(TIM_TypeDef *timx, uint32_t clock_division)
{
  STM32_MODIFY_REG(timx->CR1, TIM_CR1_CKD, clock_division);
}

/**
  * @brief  Get the actual division ratio between the timer kernel clock (tim_ker_ck) and the DTS sampling clock
  *         (DTS_ck) used by the dead-time generators (when supported) and the break/break2 filters.
  * @rmtoll
  *  CR1          CKD           LL_TIM_GetClockDivision
  * @param  timx Timer instance
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(timx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV8
  */
__STATIC_INLINE uint32_t LL_TIM_GetClockDivision(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_CKD));
}

/**
  * @brief  Set the division ratio between the DTS sampling clock (DTS_ck)
  *         and the DTS2 sampling clock (DTS2_ck) used by the digital filters.
  * @rmtoll
  *  CR1          CKD2          LL_TIM_SetClockDivision2
  * @param  timx Timer instance
  * @param  clock_division2 This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV4
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV16
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV64
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV256
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV1024
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV4096
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV16384
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV65536
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(timx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  */
__STATIC_INLINE void LL_TIM_SetClockDivision2(TIM_TypeDef *timx, uint32_t clock_division2)
{
  STM32_MODIFY_REG(timx->CR1, TIM_CR1_CKD2, clock_division2);
}

/**
  * @brief  Get the actual division ratio between the DTS sampling clock (DTS_ck)
  *         and the DTS2 sampling clock (DTS2_ck) used by the digital filters.
  * @rmtoll
  *  CR1          CKD2          LL_TIM_GetClockDivision2
  * @param  timx Timer instance
  * @note Macro IS_TIM_CLOCK_DIVISION_INSTANCE(timx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV4
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV16
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV64
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV256
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV1024
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV4096
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV16384
  *         @arg @ref LL_TIM_CLOCKDIVISION2_DIV65536
  */
__STATIC_INLINE uint32_t LL_TIM_GetClockDivision2(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_CKD2));
}

/**
  * @brief  Set the counter value.
  * @rmtoll
  *  CNT          CNT           LL_TIM_SetCounter
  * @param  timx Timer instance
  * @param  counter Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note If dithering is activated, pay attention to the Counter value interpretation
  */
__STATIC_INLINE void LL_TIM_SetCounter(TIM_TypeDef *timx, uint32_t counter)
{
  STM32_WRITE_REG(timx->CNT, counter);
}

/**
  * @brief  Get the counter value.
  * @rmtoll
  *  CNT          CNT           LL_TIM_GetCounter
  * @param  timx Timer instance
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note If dithering is activated, pay attention to the Counter value interpretation
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  */
__STATIC_INLINE uint32_t LL_TIM_GetCounter(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CNT));
}

/**
  * @brief  Get the current direction of the counter.
  * @rmtoll
  *  CR1          DIR           LL_TIM_GetDirection
  * @param  timx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERDIRECTION_UP
  *         @arg @ref LL_TIM_COUNTERDIRECTION_DOWN
  */
__STATIC_INLINE uint32_t LL_TIM_GetDirection(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR1, TIM_CR1_DIR));
}

/**
  * @brief  Set the prescaler value.
  * @rmtoll
  *  PSC          PSC           LL_TIM_SetPrescaler
  * @param  timx Timer instance
  * @param  prescaler between Min_Data=0 and Max_Data=65535
  * @note The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
  * @note The prescaler can be changed on the fly as this control register is buffered. The new
  *       prescaler ratio is taken into account at the next update event.
  * @note Helper macro @ref LL_TIM_CALC_PSC can be used to calculate the prescaler parameter
  */
__STATIC_INLINE void LL_TIM_SetPrescaler(TIM_TypeDef *timx, uint32_t prescaler)
{
  STM32_WRITE_REG(timx->PSC, prescaler);
}

/**
  * @brief  Get the prescaler value.
  * @rmtoll
  *  PSC          PSC           LL_TIM_GetPrescaler
  * @param  timx Timer instance
  * @retval  Prescaler value between Min_Data=0 and Max_Data=65535
  */
__STATIC_INLINE uint32_t LL_TIM_GetPrescaler(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->PSC));
}

/**
  * @brief  Set the auto-reload value.
  * @rmtoll
  *  ARR          ARR           LL_TIM_SetAutoReload
  * @param  timx Timer instance
  * @param  auto_reload between Min_Data=0 and Max_Data=65535
  * @note The counter is blocked while the auto-reload value is null.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Helper macro @ref LL_TIM_CALC_ARR can be used to calculate the auto_reload parameter
  *       In case dithering is activated,macro LL_TIM_CALC_ARR_DITHER can be used instead, to calculate the auto_reload
  *       parameter.
  */
__STATIC_INLINE void LL_TIM_SetAutoReload(TIM_TypeDef *timx, uint32_t auto_reload)
{
  STM32_WRITE_REG(timx->ARR, auto_reload);
}

/**
  * @brief  Get the auto-reload value.
  * @rmtoll
  *  ARR          ARR           LL_TIM_GetAutoReload
  * @param  timx Timer instance
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note If dithering is activated, pay attention to the returned value interpretation
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t LL_TIM_GetAutoReload(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->ARR));
}

/**
  * @brief  Set the repetition counter value.
  * @rmtoll
  *  RCR          REP           LL_TIM_SetRepetitionCounter
  * @param  timx Timer instance
  * @param  repetition_counter between Min_Data=0 and Max_Data=255 or 65535 for advanced timer.
  * @note Macro IS_TIM_REPETITION_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  */
__STATIC_INLINE void LL_TIM_SetRepetitionCounter(TIM_TypeDef *timx, uint32_t repetition_counter)
{
  STM32_WRITE_REG(timx->RCR, repetition_counter);
}

/**
  * @brief  Get the repetition counter value.
  * @rmtoll
  *  RCR          REP           LL_TIM_GetRepetitionCounter
  * @param  timx Timer instance
  * @note Macro IS_TIM_REPETITION_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  * @retval Repetition counter value
  */
__STATIC_INLINE uint32_t LL_TIM_GetRepetitionCounter(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->RCR));
}

/**
  * @brief  Force a continuous copy of the update interrupt flag (UIF) into the timer counter register (bit 31).
  * @rmtoll
  *  CR1          UIFREMAP      LL_TIM_EnableUIFRemap
  * @param  timx Timer instance
  * @note This allows both the counter value and a potential roll-over condition signalled by the UIFCPY flag to be read
  *       in an atomic way.
  */
__STATIC_INLINE void LL_TIM_EnableUIFRemap(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_UIFREMAP);
}

/**
  * @brief  Disable update interrupt flag (UIF) remapping.
  * @rmtoll
  *  CR1          UIFREMAP      LL_TIM_DisableUIFRemap
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableUIFRemap(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_UIFREMAP);
}

/**
  * @brief  Indicates whether the update interrupt flag (UIF)
  *         remapping is enabled.
  * @rmtoll
  *  CR1          UIFREMAP      LL_TIM_IsEnabledUIFRemap
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledUIFRemap(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_UIFREMAP) == (TIM_CR1_UIFREMAP)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate whether update interrupt flag (UIF) copy is set.
  * @param  Counter Counter value
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveUIFCPY(uint32_t Counter)
{
  return ((STM32_READ_BIT(Counter, TIM_CNT_UIFCPY) == (TIM_CNT_UIFCPY)) ? 1UL : 0UL);
}

/**
  * @brief  Enable dithering.
  * @rmtoll
  *  CR1          DITHEN          LL_TIM_EnableDithering
  * @param  timx Timer instance
  * @warning Dithering can only be enabled when the counter is disabled.
  */
__STATIC_INLINE void LL_TIM_EnableDithering(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR1, TIM_CR1_DITHEN);
}

/**
  * @brief  Disable dithering.
  * @rmtoll
  *  CR1          DITHEN          LL_TIM_DisableDithering
  * @param  timx Timer instance
  * @warning Dithering can only be disabled when the counter is disabled.
  */
__STATIC_INLINE void LL_TIM_DisableDithering(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR1, TIM_CR1_DITHEN);
}

/**
  * @brief  Indicates whether dithering is activated.
  * @rmtoll
  *  CR1          DITHEN          LL_TIM_IsEnabledDithering
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDithering(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR1, TIM_CR1_DITHEN) == (TIM_CR1_DITHEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Capture_Compare Capture Compare configuration
  * @{
  */
/**
  * @brief  Enable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @rmtoll
  *  CR2          CCPC          LL_TIM_CC_EnablePreload
  * @param  timx Timer instance
  * @note CCxE, CCxNE and OCxM bits are preloaded, after having been written,
  *       they are updated only when a commutation event (COM) occurs.
  * @note Only on channels that have a complementary output.
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(timx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  */
__STATIC_INLINE void LL_TIM_CC_EnablePreload(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR2, TIM_CR2_CCPC);
}

/**
  * @brief  Disable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @rmtoll
  *  CR2          CCPC          LL_TIM_CC_DisablePreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(timx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  */
__STATIC_INLINE void LL_TIM_CC_DisablePreload(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR2, TIM_CR2_CCPC);
}

/**
  * @brief  Indicates whether the capture/compare control bits (CCxE, CCxNE and OCxM) preload is enabled.
  * @rmtoll
  *  CR2          CCPC          LL_TIM_CC_IsEnabledPreload
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_CC_IsEnabledPreload(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR2, TIM_CR2_CCPC) == (TIM_CR2_CCPC)) ? 1UL : 0UL);
}

/**
  * @brief  Set the updated source of the capture/compare control bits (CCxE, CCxNE and OCxM).
  * @rmtoll
  *  CR2          CCUS          LL_TIM_CC_SetUpdate
  * @param  timx Timer instance
  * @param  cc_update_source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCUPDATESOURCE_SOFTWARE
  *         @arg @ref LL_TIM_CCUPDATESOURCE_SOFTWARE_AND_TRIGGER
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(timx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  */
__STATIC_INLINE void LL_TIM_CC_SetUpdate(TIM_TypeDef *timx, uint32_t cc_update_source)
{
  STM32_MODIFY_REG(timx->CR2, TIM_CR2_CCUS, cc_update_source);
}

/**
  * @brief  Get the updated source of the capture/compare control bits (CCxE, CCxNE and OCxM).
  * @rmtoll
  *  CR2          CCUS          LL_TIM_CC_GetUpdate
  * @param  timx Timer instance
  * @note Macro IS_TIM_COMMUTATION_EVENT_INSTANCE(timx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @retval The returned value can be one of the following values:
  *         @arg @ref LL_TIM_CCUPDATESOURCE_SOFTWARE
  *         @arg @ref LL_TIM_CCUPDATESOURCE_SOFTWARE_AND_TRIGGER
  */
__STATIC_INLINE uint32_t LL_TIM_CC_GetUpdate(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR2, TIM_CR2_CCUS));
}

/**
  * @brief  Set the trigger of the capture/compare DMA request.
  * @rmtoll
  *  CR2          CCDS          LL_TIM_CC_SetDMAReqTrigger
  * @param  timx Timer instance
  * @param  dma_req_trigger This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCDMAREQUEST_CC
  *         @arg @ref LL_TIM_CCDMAREQUEST_UPD
  */
__STATIC_INLINE void LL_TIM_CC_SetDMAReqTrigger(TIM_TypeDef *timx, uint32_t dma_req_trigger)
{
  STM32_MODIFY_REG(timx->CR2, TIM_CR2_CCDS, dma_req_trigger);
}

/**
  * @brief  Get actual trigger of the capture/compare DMA request.
  * @rmtoll
  *  CR2          CCDS          LL_TIM_CC_GetDMAReqTrigger
  * @param  timx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CCDMAREQUEST_CC
  *         @arg @ref LL_TIM_CCDMAREQUEST_UPD
  */
__STATIC_INLINE uint32_t LL_TIM_CC_GetDMAReqTrigger(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR2, TIM_CR2_CCDS));
}

/**
  * @brief  Set the lock level to freeze the
  *         configuration of several capture/compare parameters.
  * @rmtoll
  *  BDTR         LOCK          LL_TIM_CC_SetLockLevel
  * @param  timx Timer instance
  * @param  lock_level This parameter can be one of the following values:
  *         @arg @ref LL_TIM_LOCKLEVEL_OFF
  *         @arg @ref LL_TIM_LOCKLEVEL_1
  *         @arg @ref LL_TIM_LOCKLEVEL_2
  *         @arg @ref LL_TIM_LOCKLEVEL_3
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       the lock mechanism is supported by a timer instance.
  */
__STATIC_INLINE void LL_TIM_CC_SetLockLevel(TIM_TypeDef *timx, uint32_t lock_level)
{
  STM32_MODIFY_REG(timx->BDTR, TIM_BDTR_LOCK, lock_level);
}

/**
  * @brief  Get the lock level that freezes the
  *         configuration of several capture/compare parameters.
  * @rmtoll
  *  BDTR         LOCK          LL_TIM_CC_GetLockLevel
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       the lock mechanism is supported by a timer instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_LOCKLEVEL_OFF
  *         @arg @ref LL_TIM_LOCKLEVEL_1
  *         @arg @ref LL_TIM_LOCKLEVEL_2
  *         @arg @ref LL_TIM_LOCKLEVEL_3
  */
__STATIC_INLINE uint32_t LL_TIM_CC_GetLockLevel(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->BDTR, TIM_BDTR_LOCK));
}

/**
  * @brief  Enable capture/compare channels.
  * @rmtoll
  *  CCER         CC1E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC1NE         LL_TIM_CC_EnableChannel \n
  *  CCER         CC2E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC2NE         LL_TIM_CC_EnableChannel \n
  *  CCER         CC3E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC3NE         LL_TIM_CC_EnableChannel \n
  *  CCER         CC4E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC4NE         LL_TIM_CC_EnableChannel \n
  *  CCER         CC5E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC6E          LL_TIM_CC_EnableChannel \n
  *  CCER         CC7E          LL_TIM_CC_EnableChannel
  * @param  timx Timer instance
  * @param  channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  */
__STATIC_INLINE void LL_TIM_CC_EnableChannel(TIM_TypeDef *timx, uint32_t channels)
{
  STM32_SET_BIT(timx->CCER, channels);
}

/**
  * @brief  Disable capture/compare channels.
  * @rmtoll
  *  CCER         CC1E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC1NE         LL_TIM_CC_DisableChannel \n
  *  CCER         CC2E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC2NE         LL_TIM_CC_DisableChannel \n
  *  CCER         CC3E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC3NE         LL_TIM_CC_DisableChannel \n
  *  CCER         CC4E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC4NE         LL_TIM_CC_DisableChannel \n
  *  CCER         CC5E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC6E          LL_TIM_CC_DisableChannel \n
  *  CCER         CC7E          LL_TIM_CC_DisableChannel
  * @param  timx Timer instance
  * @param  channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  */
__STATIC_INLINE void LL_TIM_CC_DisableChannel(TIM_TypeDef *timx, uint32_t channels)
{
  STM32_CLEAR_BIT(timx->CCER, channels);
}

/**
  * @brief  Indicate whether channel(s) is(are) enabled.
  * @rmtoll
  *  CCER         CC1E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC1NE         LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC2E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC2NE         LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC3E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC3NE         LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC4E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC4NE         LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC5E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC6E          LL_TIM_CC_IsEnabledChannel \n
  *  CCER         CC7E          LL_TIM_CC_IsEnabledChannel
  * @param  timx Timer instance
  * @param  channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_CC_IsEnabledChannel(const TIM_TypeDef *timx, uint32_t channels)
{
  return ((STM32_READ_BIT(timx->CCER, channels) == (channels)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Output_Channel Output channel configuration
  * @{
  */
/**
  * @brief  Configure an output channel.
  * @rmtoll
  *  CCMR1        CC1S          LL_TIM_OC_ConfigOutput \n
  *  CCMR1        CC2S          LL_TIM_OC_ConfigOutput \n
  *  CCMR2        CC3S          LL_TIM_OC_ConfigOutput \n
  *  CCMR2        CC4S          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC1P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC2P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC3P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC4P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC5P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC6P          LL_TIM_OC_ConfigOutput \n
  *  CCER         CC7P          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS1          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS2          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS3          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS4          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS5          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS6          LL_TIM_OC_ConfigOutput \n
  *  CR2          OIS7          LL_TIM_OC_ConfigOutput
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @param  configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH or @ref LL_TIM_OCPOLARITY_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_RESET or @ref LL_TIM_OCIDLESTATE_SET
  */
__STATIC_INLINE void LL_TIM_OC_ConfigOutput(TIM_TypeDef *timx, uint32_t channel, uint32_t configuration)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_CLEAR_BIT(*pReg, (TIM_CCMR1_CC1S << LL_TIM_SHIFT_TAB_OCxx[ichannel]));
  STM32_MODIFY_REG(timx->CCER, (TIM_CCER_CC1P << LL_TIM_SHIFT_TAB_CCxP[ichannel]),
                   (configuration & TIM_CCER_CC1P) << LL_TIM_SHIFT_TAB_CCxP[ichannel]);
  STM32_MODIFY_REG(timx->CR2, (TIM_CR2_OIS1 << LL_TIM_SHIFT_TAB_OISx[ichannel]),
                   (configuration & TIM_CR2_OIS1) << LL_TIM_SHIFT_TAB_OISx[ichannel]);
}

/**
  * @brief  Define the behavior of the output reference signal OCxREF from which
  *         OCx and OCxN (when relevant) are derived.
  * @rmtoll
  *  CCMR1        OC1M          LL_TIM_OC_SetMode \n
  *  CCMR1        OC2M          LL_TIM_OC_SetMode \n
  *  CCMR2        OC3M          LL_TIM_OC_SetMode \n
  *  CCMR2        OC4M          LL_TIM_OC_SetMode \n
  *  CCMR3        OC5M          LL_TIM_OC_SetMode \n
  *  CCMR3        OC6M          LL_TIM_OC_SetMode \n
  *  CCMR4        OC7M          LL_TIM_OC_SetMode
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE_ON_MATCH
  *         @arg @ref LL_TIM_OCMODE_INACTIVE_ON_MATCH
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIGERRABLE_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIGERRABLE_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM3
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM4
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM3
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM4
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM5
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM6
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM7
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM8
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM9
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM10
  *         @arg @ref LL_TIM_OCMODE_PULSE_ON_COMPARE   (for channel 3 or channel 4 only)
  *         @arg @ref LL_TIM_OCMODE_DIRECTION_OUTPUT   (for channel 3 or channel 4 only)
  */
__STATIC_INLINE void LL_TIM_OC_SetMode(TIM_TypeDef *timx, uint32_t channel, uint32_t mode)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << LL_TIM_SHIFT_TAB_OCxx[ichannel]),
                   mode << LL_TIM_SHIFT_TAB_OCxx[ichannel]);
}

/**
  * @brief  Get the output compare mode of an output channel.
  * @rmtoll
  *  CCMR1        OC1M          LL_TIM_OC_GetMode \n
  *  CCMR1        OC2M          LL_TIM_OC_GetMode \n
  *  CCMR2        OC3M          LL_TIM_OC_GetMode \n
  *  CCMR2        OC4M          LL_TIM_OC_GetMode \n
  *  CCMR3        OC5M          LL_TIM_OC_GetMode \n
  *  CCMR3        OC6M          LL_TIM_OC_GetMode \n
  *  CCMR4        OC7M          LL_TIM_OC_GetMode
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE_ON_MATCH
  *         @arg @ref LL_TIM_OCMODE_INACTIVE_ON_MATCH
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIGERRABLE_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIGERRABLE_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM3
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM4
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM3
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM4
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM5
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM6
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM7
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM8
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM9
  *         @arg @ref LL_TIM_OCMODE_ASYMMETRIC_PWM10
  *         @arg @ref LL_TIM_OCMODE_PULSE_ON_COMPARE   (for channel 3 or channel 4 only)
  *         @arg @ref LL_TIM_OCMODE_DIRECTION_OUTPUT   (for channel 3 or channel 4 only)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetMode(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  return (STM32_READ_BIT(*pReg, ((TIM_CCMR1_OC1M | TIM_CCMR1_CC1S) << LL_TIM_SHIFT_TAB_OCxx[ichannel])) \
          >> LL_TIM_SHIFT_TAB_OCxx[ichannel]);
}

/**
  * @brief  Set the polarity of an output channel.
  * @rmtoll
  *  CCER         CC1P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC1NP         LL_TIM_OC_SetPolarity \n
  *  CCER         CC2P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC2NP         LL_TIM_OC_SetPolarity \n
  *  CCER         CC3P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC3NP         LL_TIM_OC_SetPolarity \n
  *  CCER         CC4P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC4NP         LL_TIM_OC_SetPolarity \n
  *  CCER         CC5P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC6P          LL_TIM_OC_SetPolarity \n
  *  CCER         CC7P          LL_TIM_OC_SetPolarity
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @param  polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  */
__STATIC_INLINE void LL_TIM_OC_SetPolarity(TIM_TypeDef *timx, uint32_t channel, uint32_t polarity)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  STM32_MODIFY_REG(timx->CCER, (TIM_CCER_CC1P << LL_TIM_SHIFT_TAB_CCxP[ichannel]),
                   polarity << LL_TIM_SHIFT_TAB_CCxP[ichannel]);
}

/**
  * @brief  Get the polarity of an output channel.
  * @rmtoll
  *  CCER         CC1P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC1NP         LL_TIM_OC_GetPolarity \n
  *  CCER         CC2P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC2NP         LL_TIM_OC_GetPolarity \n
  *  CCER         CC3P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC3NP         LL_TIM_OC_GetPolarity \n
  *  CCER         CC4P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC4NP         LL_TIM_OC_GetPolarity \n
  *  CCER         CC5P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC6P          LL_TIM_OC_GetPolarity \n
  *  CCER         CC7P          LL_TIM_OC_GetPolarity
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetPolarity(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  return (STM32_READ_BIT(timx->CCER, (TIM_CCER_CC1P << LL_TIM_SHIFT_TAB_CCxP[ichannel])) \
          >> LL_TIM_SHIFT_TAB_CCxP[ichannel]);
}

/**
  * @brief  Set the idle state of an output channel.
  * @rmtoll
  *  CR2         OIS1          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS2N         LL_TIM_OC_SetIdleState \n
  *  CR2         OIS2          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS2N         LL_TIM_OC_SetIdleState \n
  *  CR2         OIS3          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS3N         LL_TIM_OC_SetIdleState \n
  *  CR2         OIS4          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS4N         LL_TIM_OC_SetIdleState \n
  *  CR2         OIS5          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS6          LL_TIM_OC_SetIdleState \n
  *  CR2         OIS7          LL_TIM_OC_SetIdleState
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @param  idle_state This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_RESET
  *         @arg @ref LL_TIM_OCIDLESTATE_SET
  * @note This function is significant only for the timer instances
  *       supporting the break feature. Macro IS_TIM_BREAK_INSTANCE(timx)
  *       can be used to check whether or not a timer instance provides
  *       a break input.
  */
__STATIC_INLINE void LL_TIM_OC_SetIdleState(TIM_TypeDef *timx, uint32_t channel, uint32_t idle_state)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  STM32_MODIFY_REG(timx->CR2, (TIM_CR2_OIS1 << LL_TIM_SHIFT_TAB_OISx[ichannel]), \
                   idle_state << LL_TIM_SHIFT_TAB_OISx[ichannel]);
}

/**
  * @brief  Get the idle state of an output channel.
  * @rmtoll
  *  CR2         OIS1          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS2N         LL_TIM_OC_GetIdleState \n
  *  CR2         OIS2          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS2N         LL_TIM_OC_GetIdleState \n
  *  CR2         OIS3          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS3N         LL_TIM_OC_GetIdleState \n
  *  CR2         OIS4          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS4N         LL_TIM_OC_GetIdleState \n
  *  CR2         OIS5          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS6          LL_TIM_OC_GetIdleState \n
  *  CR2         OIS7          LL_TIM_OC_GetIdleState
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_RESET
  *         @arg @ref LL_TIM_OCIDLESTATE_SET
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetIdleState(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  return (STM32_READ_BIT(timx->CR2, (TIM_CR2_OIS1 << LL_TIM_SHIFT_TAB_OISx[ichannel])) \
          >> LL_TIM_SHIFT_TAB_OISx[ichannel]);
}

/**
  * @brief  Set the override state of a disabled output channel.
  * @rmtoll
  *  OOR         OOS1          LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS1N         LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS2          LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS2N         LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS3          LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS3N         LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS4          LL_TIM_OC_SetOverrideState \n
  *  OOR         OOS4N         LL_TIM_OC_SetOverrideState
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  * @param  override_state This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCOVERRIDE_RESET
  *         @arg @ref LL_TIM_OCOVERRIDE_SET
  * @note Only applicable to external channels and their complementary (1, 2, 3 and 4).
  * @note This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been programmed
  *       (LOCK bits in TIMx_BDTR register).
  */
__STATIC_INLINE void LL_TIM_OC_SetOverrideState(TIM_TypeDef *timx, uint32_t channel, uint32_t override_state)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  STM32_MODIFY_REG(timx->OOR, (TIM_OOR_OOS1 << ichannel), (override_state << ichannel));
}

/**
  * @brief  Get the override state of a disabled output channel.
  * @rmtoll
  *  OOR         OOS1          LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS1N         LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS2          LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS2N         LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS3          LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS3N         LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS4          LL_TIM_OC_GetOverrideState \n
  *  OOR         OOS4N         LL_TIM_OC_GetOverrideState
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCOVERRIDE_RESET
  *         @arg @ref LL_TIM_OCOVERRIDE_SET
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetOverrideState(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  return (STM32_READ_BIT(timx->OOR, (TIM_OOR_OOS1 << ichannel)) >> ichannel);
}

/**
  * @brief  Enable output override (outputs are forced in an idle state defined with OOSx/OOSxN bits).
  * @rmtoll
  *  OOR          OCC           LL_TIM_OC_EnableOutputOverride
  * @param  timx Timer instance
  * @note This function can only be used when the outputs are in idle state (MOE = 0).
  */
__STATIC_INLINE void LL_TIM_OC_EnableOutputOverride(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->OOR, TIM_OOR_OOC);
}

/**
  * @brief  Disable output override (outputs are in the default idle state).
  * @rmtoll
  *  OOR          OCC           LL_TIM_OC_DisableOutputOverride
  * @param  timx Timer instance
  * @note This function can only be used when the outputs are in idle state (MOE = 0).
  */
__STATIC_INLINE void LL_TIM_OC_DisableOutputOverride(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->OOR, TIM_OOR_OOC);
}

/**
  * @brief  Indicate whether output override is enabled.
  * @rmtoll
  *  OOR          OCC           LL_TIM_OC_IsEnabledOutputOverride
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_OC_IsEnabledOutputOverride(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->OOR, TIM_OOR_OOC) == (TIM_OOR_OOC)) ? 1UL : 0UL);
}

/**
  * @brief  Enable fast mode for the output channel.
  * @rmtoll
  *  CCMR1        OC1FE          LL_TIM_OC_EnableFast \n
  *  CCMR1        OC2FE          LL_TIM_OC_EnableFast \n
  *  CCMR2        OC3FE          LL_TIM_OC_EnableFast \n
  *  CCMR2        OC4FE          LL_TIM_OC_EnableFast \n
  *  CCMR3        OC5FE          LL_TIM_OC_EnableFast \n
  *  CCMR3        OC6FE          LL_TIM_OC_EnableFast \n
  *  CCMR4        OC7FE          LL_TIM_OC_EnableFast
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @note Acts only if the channel is configured in PWM1 or PWM2 mode.
  */
__STATIC_INLINE void LL_TIM_OC_EnableFast(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_SET_BIT(*pReg, (TIM_CCMR1_OC1FE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));

}

/**
  * @brief  Disable fast mode for the output channel.
  * @rmtoll
  *  CCMR1        OC1FE          LL_TIM_OC_DisableFast \n
  *  CCMR1        OC2FE          LL_TIM_OC_DisableFast \n
  *  CCMR2        OC3FE          LL_TIM_OC_DisableFast \n
  *  CCMR2        OC4FE          LL_TIM_OC_DisableFast \n
  *  CCMR3        OC5FE          LL_TIM_OC_DisableFast \n
  *  CCMR3        OC6FE          LL_TIM_OC_DisableFast \n
  *  CCMR4        OC7FE          LL_TIM_OC_DisableFast
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  */
__STATIC_INLINE void LL_TIM_OC_DisableFast(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_CLEAR_BIT(*pReg, (TIM_CCMR1_OC1FE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));

}

/**
  * @brief  Indicates whether fast mode is enabled for the output channel.
  * @rmtoll
  *  CCMR1        OC1FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR1        OC2FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR2        OC3FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR2        OC4FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR3        OC5FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR3        OC6FE          LL_TIM_OC_IsEnabledFast \n
  *  CCMR4        OC7FE          LL_TIM_OC_IsEnabledFast
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_OC_IsEnabledFast(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  uint32_t bitfield = TIM_CCMR1_OC1FE << LL_TIM_SHIFT_TAB_OCxx[ichannel];
  return ((STM32_READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Enable compare register (TIMx_CCRx) preload for the output channel.
  * @rmtoll
  *  CCMR1        OC1PE          LL_TIM_OC_EnablePreload \n
  *  CCMR1        OC2PE          LL_TIM_OC_EnablePreload \n
  *  CCMR2        OC3PE          LL_TIM_OC_EnablePreload \n
  *  CCMR2        OC4PE          LL_TIM_OC_EnablePreload \n
  *  CCMR3        OC5PE          LL_TIM_OC_EnablePreload \n
  *  CCMR3        OC6PE          LL_TIM_OC_EnablePreload \n
  *  CCMR4        OC7PE          LL_TIM_OC_EnablePreload
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  */
__STATIC_INLINE void LL_TIM_OC_EnablePreload(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_SET_BIT(*pReg, (TIM_CCMR1_OC1PE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));
}

/**
  * @brief  Disable compare register (TIMx_CCRx) preload for the output channel.
  * @rmtoll
  *  CCMR1        OC1PE          LL_TIM_OC_DisablePreload \n
  *  CCMR1        OC2PE          LL_TIM_OC_DisablePreload \n
  *  CCMR2        OC3PE          LL_TIM_OC_DisablePreload \n
  *  CCMR2        OC4PE          LL_TIM_OC_DisablePreload \n
  *  CCMR3        OC5PE          LL_TIM_OC_DisablePreload \n
  *  CCMR3        OC6PE          LL_TIM_OC_DisablePreload \n
  *  CCMR4        OC7PE          LL_TIM_OC_DisablePreload
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  */
__STATIC_INLINE void LL_TIM_OC_DisablePreload(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));
}

/**
  * @brief  Indicates whether compare register (TIMx_CCRx) preload is enabled for the output channel.
  * @rmtoll
  *  CCMR1        OC1PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR1        OC2PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR2        OC3PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR2        OC4PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR3        OC5PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR3        OC6PE          LL_TIM_OC_IsEnabledPreload \n
  *  CCMR4        OC7PE          LL_TIM_OC_IsEnabledPreload
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_OC_IsEnabledPreload(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  uint32_t bitfield = TIM_CCMR1_OC1PE << LL_TIM_SHIFT_TAB_OCxx[ichannel];
  return ((STM32_READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Enable clearing the output channel on an external event.
  * @rmtoll
  *  CCMR1        OC1CE          LL_TIM_OC_EnableClear \n
  *  CCMR1        OC2CE          LL_TIM_OC_EnableClear \n
  *  CCMR2        OC3CE          LL_TIM_OC_EnableClear \n
  *  CCMR2        OC4CE          LL_TIM_OC_EnableClear \n
  *  CCMR3        OC5CE          LL_TIM_OC_EnableClear \n
  *  CCMR3        OC6CE          LL_TIM_OC_EnableClear \n
  *  CCMR4        OC7CE          LL_TIM_OC_EnableClear
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro IS_TIM_OCXREF_CLEAR_INSTANCE(timx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  */
__STATIC_INLINE void LL_TIM_OC_EnableClear(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_SET_BIT(*pReg, (TIM_CCMR1_OC1CE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));
}

/**
  * @brief  Disable clearing the output channel on an external event.
  * @rmtoll
  *  CCMR1        OC1CE          LL_TIM_OC_DisableClear \n
  *  CCMR1        OC2CE          LL_TIM_OC_DisableClear \n
  *  CCMR2        OC3CE          LL_TIM_OC_DisableClear \n
  *  CCMR2        OC4CE          LL_TIM_OC_DisableClear \n
  *  CCMR3        OC5CE          LL_TIM_OC_DisableClear \n
  *  CCMR3        OC6CE          LL_TIM_OC_DisableClear \n
  *  CCMR4        OC7CE          LL_TIM_OC_DisableClear
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @note Macro IS_TIM_OCXREF_CLEAR_INSTANCE(timx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  */
__STATIC_INLINE void LL_TIM_OC_DisableClear(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_CLEAR_BIT(*pReg, (TIM_CCMR1_OC1CE << LL_TIM_SHIFT_TAB_OCxx[ichannel]));
}

/**
  * @brief  Indicates clearing the output channel on an external event is enabled for the output channel.
  * @rmtoll
  *  CCMR1        OC1CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR1        OC2CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR2        OC3CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR2        OC4CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR3        OC5CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR3        OC6CE          LL_TIM_OC_IsEnabledClear \n
  *  CCMR4        OC7CE          LL_TIM_OC_IsEnabledClear
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  *         @arg @ref LL_TIM_CHANNEL_CH7
  * @note This function enables clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro IS_TIM_OCXREF_CLEAR_INSTANCE(timx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_OC_IsEnabledClear(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  uint32_t bitfield = TIM_CCMR1_OC1CE << LL_TIM_SHIFT_TAB_OCxx[ichannel];
  return ((STM32_READ_BIT(*pReg, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Set the break channel output mode BKxM.
  * @rmtoll
  *  MPR1          BKM1          LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM1N         LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM2          LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM2N         LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM3          LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM3N         LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM4          LL_TIM_OC_SetBreakMode \n
  *  MPR1          BKM4N         LL_TIM_OC_SetBreakMode
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  * @param  break_mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCBREAKMODE_IMMEDIATE
  *         @arg @ref LL_TIM_OCBREAKMODE_DELAY1
  *         @arg @ref LL_TIM_OCBREAKMODE_DELAY2
  * @note This bit must be modified only when the counter is disabled.
  * @note The break2 feature must be disabled when the delayed mode is enabled.
  */
__STATIC_INLINE void LL_TIM_OC_SetBreakMode(TIM_TypeDef *timx, uint32_t channel, uint32_t break_mode)
{
  uint32_t ichannel_shift = (uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) << 1U;
  STM32_MODIFY_REG(timx->MPR1, (TIM_MPR1_BK1M << ichannel_shift), break_mode << ichannel_shift);
}

/**
  * @brief  Get the break channel output mode BKxM.
  * @rmtoll
  *  MPR1          BKM1          LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM1N         LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM2          LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM2N         LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM3          LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM3N         LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM4          LL_TIM_OC_GetBreakMode \n
  *  MPR1          BKM4N         LL_TIM_OC_GetBreakMode
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH4N
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCBREAKMODE_IMMEDIATE
  *         @arg @ref LL_TIM_OCBREAKMODE_DELAY1
  *         @arg @ref LL_TIM_OCBREAKMODE_DELAY2
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetBreakMode(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel_shift = (uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) << 1U;
  return (uint32_t)((STM32_READ_BIT(timx->MPR1, (TIM_MPR1_BK1M << ichannel_shift))) >> ichannel_shift);
}

/**
  * @brief  Set the dead-time delay (delay inserted between the rising edge of the OCxREF signal and the rising edge of
  *         the Ocx and OCxN signals).
  * @rmtoll
  *  BDTR         DTG           LL_TIM_OC_SetDeadTime
  * @param  timx Timer instance
  * @param  deadtime between Min_Data=0 and Max_Data=255
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       dead-time insertion feature is supported by a timer instance.
  * @note Helper macro @ref LL_TIM_CALC_DEADTIME can be used to calculate the deadtime parameter
  */
__STATIC_INLINE void LL_TIM_OC_SetDeadTime(TIM_TypeDef *timx, uint32_t deadtime)
{
  STM32_MODIFY_REG(timx->BDTR, TIM_BDTR_DTG, deadtime);
}

/**
  * @brief  Get the dead-time delay (delay inserted between the rising edge of the
  *         OCxREF signal and the rising edge of the Ocx and OCxN signals).
  * @rmtoll
  *  BDTR         DTG           LL_TIM_OC_GetDeadTime
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       dead-time insertion feature is supported by a timer instance.
  * @note Helper macro @ref LL_TIM_CALC_DEADTIME can be used to calculate the deadtime parameter
  * @retval deadtime between Min_Data=0 and Max_Data=255
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetDeadTime(const TIM_TypeDef *timx)
{
  return (STM32_READ_REG(timx->BDTR) & TIM_BDTR_DTG);
}

/**
  * @brief  Set compare value for output channel 1 (TIMx_CCR1).
  * @rmtoll
  *  CCR1         CCR1          LL_TIM_OC_SetCompareCH1
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC1_INSTANCE(timx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH1(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR1, compare_value);
}

/**
  * @brief  Set compare value for output channel 2 (TIMx_CCR2).
  * @rmtoll
  *  CCR2         CCR2          LL_TIM_OC_SetCompareCH2
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC2_INSTANCE(timx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH2(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR2, compare_value);
}

/**
  * @brief  Set compare value for output channel 3 (TIMx_CCR3).
  * @rmtoll
  *  CCR3         CCR3          LL_TIM_OC_SetCompareCH3
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC3_INSTANCE(timx) can be used to check whether or not
  *       output channel is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH3(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR3, compare_value);
}

/**
  * @brief  Set compare value for output channel 4 (TIMx_CCR4).
  * @rmtoll
  *  CCR4         CCR4          LL_TIM_OC_SetCompareCH4
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC4_INSTANCE(timx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH4(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR4, compare_value);
}

/**
  * @brief  Set compare value for output channel 5 (TIMx_CCR5).
  * @rmtoll
  *  CCR5         CCR5          LL_TIM_OC_SetCompareCH5
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note Macro IS_TIM_CC5_INSTANCE(timx) can be used to check whether or not
  *       output channel 5 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH5(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_MODIFY_REG(timx->CCR5, TIM_CCR5_CCR5, compare_value);
}

/**
  * @brief  Set compare value for output channel 6 (TIMx_CCR6).
  * @rmtoll
  *  CCR6         CCR6          LL_TIM_OC_SetCompareCH6
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note Macro IS_TIM_CC6_INSTANCE(timx) can be used to check whether or not
  *       output channel 6 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH6(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR6, compare_value);
}

/**
  * @brief  Set compare value for output channel 7 (TIMx_CCR7).
  * @rmtoll
  *  CCR7         CCR7          LL_TIM_OC_SetCompareCH7
  * @param  timx Timer instance
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note Macro IS_TIM_CC7_INSTANCE(timx) can be used to check whether or not
  *       output channel 7 is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareCH7(TIM_TypeDef *timx, uint32_t compare_value)
{
  STM32_WRITE_REG(timx->CCR7, compare_value);
}

/**
  * @brief  Set compare value for the selected compare unit.
  * @rmtoll
  *  CCR7         CCR7          LL_TIM_OC_SetCompareValue \n
  *  CCR1         CCR1          LL_TIM_OC_SetCompareValue \n
  *  CCR2         CCR2          LL_TIM_OC_SetCompareValue \n
  *  CCR3         CCR3          LL_TIM_OC_SetCompareValue \n
  *  CCR4         CCR4          LL_TIM_OC_SetCompareValue \n
  *  CCR5         CCR5          LL_TIM_OC_SetCompareValue \n
  *  CCR6         CCR6          LL_TIM_OC_SetCompareValue
  * @param  timx Timer instance
  * @param  compare_unit This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_1
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_2
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_3
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_4
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_5
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_6
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_7
  * @param  compare_value between Min_Data=0 and Max_Data=65535
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CCx_INSTANCE(timx) can be used to check whether or not
  *       capture unit x is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER .
  */
__STATIC_INLINE void LL_TIM_OC_SetCompareValue(TIM_TypeDef *timx, uint32_t compare_unit, uint32_t compare_value)
{
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCR1) + LL_TIM_OFFSET_TAB_CCRx[compare_unit]));
  uint32_t compare_mask = (compare_unit < LL_TIM_OC_COMPARE_UNIT_5) ? TIM_CCR1_CCR1 : TIM_CCR5_CCR5;
  STM32_MODIFY_REG(*pReg, compare_mask, compare_value);
}

/**
  * @brief  Get compare value (TIMx_CCR1) set for output channel 1.
  * @rmtoll
  *  CCR1         CCR1          LL_TIM_OC_GetCompareCH1
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC1_INSTANCE(timx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH1(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR1));
}

/**
  * @brief  Get compare value (TIMx_CCR2) set for output channel 2.
  * @rmtoll
  *  CCR2         CCR2          LL_TIM_OC_GetCompareCH2
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC2_INSTANCE(timx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH2(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR2));
}

/**
  * @brief  Get compare value (TIMx_CCR3) set for output channel 3.
  * @rmtoll
  *  CCR3         CCR3          LL_TIM_OC_GetCompareCH3
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC3_INSTANCE(timx) can be used to check whether or not
  *       output channel 3 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH3(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR3));
}

/**
  * @brief  Get compare value (TIMx_CCR4) set for output channel 4.
  * @rmtoll
  *  CCR4         CCR4          LL_TIM_OC_GetCompareCH4
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC4_INSTANCE(timx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH4(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR4));
}

/**
  * @brief  Get compare value (TIMx_CCR5) set for output channel 5.
  * @rmtoll
  *  CCR5         CCR5          LL_TIM_OC_GetCompareCH5
  * @param  timx Timer instance
  * @note Macro IS_TIM_CC5_INSTANCE(timx) can be used to check whether or not
  *       output channel 5 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH5(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CCR5, TIM_CCR5_CCR5));
}

/**
  * @brief  Get compare value (TIMx_CCR6) set for output channel 6.
  * @rmtoll
  *  CCR6         CCR6          LL_TIM_OC_GetCompareCH6
  * @param  timx Timer instance
  * @note Macro IS_TIM_CC6_INSTANCE(timx) can be used to check whether or not
  *       output channel 6 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH6(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR6));
}

/**
  * @brief  Get compare value (TIMx_CCR7) set for output channel 7.
  * @rmtoll
  *  CCR7         CCR7          LL_TIM_OC_GetCompareCH7
  * @param  timx Timer instance
  * @note Macro IS_TIM_CC7_INSTANCE(timx) can be used to check whether or not
  *       output channel 7 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareCH7(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR7));
}

/**
  * @brief  Get compare value for the selected compare unit.
  * @rmtoll
  *  CCR1         CCR1          LL_TIM_OC_GetCompareValue \n
  *  CCR2         CCR2          LL_TIM_OC_GetCompareValue \n
  *  CCR3         CCR3          LL_TIM_OC_GetCompareValue \n
  *  CCR4         CCR4          LL_TIM_OC_GetCompareValue \n
  *  CCR5         CCR5          LL_TIM_OC_GetCompareValue \n
  *  CCR6         CCR6          LL_TIM_OC_GetCompareValue \n
  *  CCR7         CCR7          LL_TIM_OC_GetCompareValue
  * @param  timx Timer instance
  * @param  compare_unit This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_1
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_2
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_3
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_4
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_5
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_6
  *         @arg @ref LL_TIM_OC_COMPARE_UNIT_7
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CCx_INSTANCE(timx) can be used to check whether or not
  *       capture unit x is supported by a timer instance.
  * @note If dithering is activated, compare_value can be calculated with macro @ref LL_TIM_CALC_DELAY_DITHER.
  * @retval compare_value (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetCompareValue(TIM_TypeDef *timx, uint32_t compare_unit)
{
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCR1) + \
                                                           LL_TIM_OFFSET_TAB_CCRx[compare_unit]));
  uint32_t compare_mask = (compare_unit < LL_TIM_OC_COMPARE_UNIT_5) ? TIM_CCR1_CCR1 : TIM_CCR5_CCR5;
  return (uint32_t)(STM32_READ_BIT(*pReg, compare_mask));
}

/**
  * @brief  Select on which reference signal the OC5REF is combined to.
  * @rmtoll
  *  CCR5         GC5C1          LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C2          LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C3          LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C4          LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C1O         LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C2O         LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C3O         LL_TIM_SetCH5CombinedChannels \n
  *  CCR5         GC5C4O         LL_TIM_SetCH5CombinedChannels
  * @param  timx Timer instance
  * @param  group_ch5 This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_GROUPCH5_NONE
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC3REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC4REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC3REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC4REFC
  * @note Macro IS_TIM_COMBINED3PHASEPWM_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the combined 3-phase PWM mode.
  */
__STATIC_INLINE void LL_TIM_SetCH5CombinedChannels(TIM_TypeDef *timx, uint32_t group_ch5)
{
  STM32_MODIFY_REG(timx->CCR5, (TIM_CCR5_GC5C1 | TIM_CCR5_GC5C2 | TIM_CCR5_GC5C3 | TIM_CCR5_GC5C4 | \
                                TIM_CCR5_GC5C1O | TIM_CCR5_GC5C2O | TIM_CCR5_GC5C3O | TIM_CCR5_GC5C4O), group_ch5);
}

/**
  * @brief  Get on which reference signal the OC5REF is combined to.
  * @param  timx Timer instance
  * @note Macro IS_TIM_COMBINED3PHASEPWM_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the combined 3-phase PWM mode.
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_TIM_GROUPCH5_NONE
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC3REFC
  *         @arg @ref LL_TIM_GROUPCH5_AND_OC4REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC3REFC
  *         @arg @ref LL_TIM_GROUPCH5_OR_OC4REFC
  */
__STATIC_INLINE uint32_t LL_TIM_GetCH5CombinedChannels(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CCR5, TIM_CCR5_GC5X));
}

/**
  * @brief  Set the pulse on compare pulse width prescaler.
  * @rmtoll
  *  ECR          PWPRSC           LL_TIM_OC_SetPulseWidthPrescaler
  * @param  timx Timer instance
  * @param  pulse_width_prescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_PWPRSC_DIV1
  *         @arg @ref LL_TIM_PWPRSC_DIV2
  *         @arg @ref LL_TIM_PWPRSC_DIV4
  *         @arg @ref LL_TIM_PWPRSC_DIV8
  *         @arg @ref LL_TIM_PWPRSC_DIV16
  *         @arg @ref LL_TIM_PWPRSC_DIV32
  *         @arg @ref LL_TIM_PWPRSC_DIV64
  *         @arg @ref LL_TIM_PWPRSC_DIV128
  * @note Macro IS_TIM_PULSEONCOMPARE_INSTANCE(timx) can be used to check
  *       whether or not the pulse on compare feature is supported by the timer
  *       instance.
  */
__STATIC_INLINE void LL_TIM_OC_SetPulseWidthPrescaler(TIM_TypeDef *timx, uint32_t pulse_width_prescaler)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_PWPRSC, pulse_width_prescaler);
}

/**
  * @brief  Get the pulse on compare pulse width prescaler.
  * @rmtoll
  *  ECR          PWPRSC           LL_TIM_OC_GetPulseWidthPrescaler
  * @param  timx Timer instance
  * @note Macro IS_TIM_PULSEONCOMPARE_INSTANCE(timx) can be used to check
  *       whether or not the pulse on compare feature is supported by the timer
  *       instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_PWPRSC_DIV1
  *         @arg @ref LL_TIM_PWPRSC_DIV2
  *         @arg @ref LL_TIM_PWPRSC_DIV4
  *         @arg @ref LL_TIM_PWPRSC_DIV8
  *         @arg @ref LL_TIM_PWPRSC_DIV16
  *         @arg @ref LL_TIM_PWPRSC_DIV32
  *         @arg @ref LL_TIM_PWPRSC_DIV64
  *         @arg @ref LL_TIM_PWPRSC_DIV128
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetPulseWidthPrescaler(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->ECR, TIM_ECR_PWPRSC));
}

/**
  * @brief  Set the pulse on compare pulse width duration.
  * @rmtoll
  *  ECR          PW           LL_TIM_OC_SetPulseWidth
  * @param  timx Timer instance
  * @param  pulse_width This parameter can be between Min_Data=0 and Max_Data=255
  * @note Macro IS_TIM_PULSEONCOMPARE_INSTANCE(timx) can be used to check
  *       whether or not the pulse on compare feature is supported by the timer
  *       instance.
  */
__STATIC_INLINE void LL_TIM_OC_SetPulseWidth(TIM_TypeDef *timx, uint32_t pulse_width)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_PW, pulse_width << TIM_ECR_PW_Pos);
}

/**
  * @brief  Get the pulse on compare pulse width duration.
  * @rmtoll
  *  ECR          PW           LL_TIM_OC_GetPulseWidth
  * @param  timx Timer instance
  * @note Macro IS_TIM_PULSEONCOMPARE_INSTANCE(timx) can be used to check
  *       whether or not the pulse on compare feature is supported by the timer
  *       instance.
  * @retval Returned value can be between Min_Data=0 and Max_Data=255:
  */
__STATIC_INLINE uint32_t LL_TIM_OC_GetPulseWidth(const TIM_TypeDef *timx)
{
  return (uint32_t)((STM32_READ_BIT(timx->ECR, TIM_ECR_PW)) >> TIM_ECR_PW_Pos);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Input_Channel Input channel configuration
  * @{
  */
/**
  * @brief  Configure input channel.
  * @rmtoll
  *  CCMR1        CC1S          LL_TIM_IC_Config \n
  *  CCMR1        IC1PSC        LL_TIM_IC_Config \n
  *  CCMR1        IC1F          LL_TIM_IC_Config \n
  *  CCMR1        CC2S          LL_TIM_IC_Config \n
  *  CCMR1        IC2PSC        LL_TIM_IC_Config \n
  *  CCMR1        IC2F          LL_TIM_IC_Config \n
  *  CCMR2        CC3S          LL_TIM_IC_Config \n
  *  CCMR2        IC3PSC        LL_TIM_IC_Config \n
  *  CCMR2        IC3F          LL_TIM_IC_Config \n
  *  CCMR2        CC4S          LL_TIM_IC_Config \n
  *  CCMR2        IC4PSC        LL_TIM_IC_Config \n
  *  CCMR2        IC4F          LL_TIM_IC_Config \n
  *  CCER         CC1P          LL_TIM_IC_Config \n
  *  CCER         CC1NP         LL_TIM_IC_Config \n
  *  CCER         CC2P          LL_TIM_IC_Config \n
  *  CCER         CC2NP         LL_TIM_IC_Config \n
  *  CCER         CC3P          LL_TIM_IC_Config \n
  *  CCER         CC3NP         LL_TIM_IC_Config \n
  *  CCER         CC4P          LL_TIM_IC_Config \n
  *  CCER         CC4NP         LL_TIM_IC_Config
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECT or @ref LL_TIM_ACTIVEINPUT_INDIRECT or @ref LL_TIM_ACTIVEINPUT_TRC
  *         @arg @ref LL_TIM_ICPSC_DIV1 or ... or @ref LL_TIM_ICPSC_DIV8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1 or ... or @ref LL_TIM_IC_FILTER_FDIV32_N8
  *         @arg @ref LL_TIM_IC_POLARITY_RISING or @ref LL_TIM_IC_POLARITY_FALLING or
                 @ref LL_TIM_IC_POLARITY_RISING_FALLING
  */
__STATIC_INLINE void LL_TIM_IC_Config(TIM_TypeDef *timx, uint32_t channel, uint32_t configuration)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_MODIFY_REG(*pReg, ((TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC | TIM_CCMR1_CC1S) << LL_TIM_SHIFT_TAB_ICxx[ichannel]),
                   ((configuration >> 16U) & (TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC | TIM_CCMR1_CC1S))                \
                   << LL_TIM_SHIFT_TAB_ICxx[ichannel]);
  STM32_MODIFY_REG(timx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << LL_TIM_SHIFT_TAB_CCxP[ichannel]),
                   (configuration & (TIM_CCER_CC1NP | TIM_CCER_CC1P)) << LL_TIM_SHIFT_TAB_CCxP[ichannel]);
}

/**
  * @brief  Select the input channel source.
  * @rmtoll
  *  TISEL    TI1SEL      LL_TIM_IC_SetSource \n
  *  TISEL    TI2SEL      LL_TIM_IC_SetSource \n
  *  TISEL    TI3SEL      LL_TIM_IC_SetSource \n
  *  TISEL    TI4SEL      LL_TIM_IC_SetSource
  * @param  timx Timer instance
  * @param  ti_source The timer input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *         Otherwise, see the Reference Manual description of the TISEL registers.
  *
  *         The description below summarizes "Timer Instance" and "Timer Input" parameter combinations:
  *
  *         TIM1: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM1_TI1_GPIO:                tim1_ti1 is connected to TIM1_CH1
  *            LL_TIM_TIM1_TI1_COMP1_OUT:           tim1_ti1 is connected to comp1_out
  *            LL_TIM_TIM1_TI1_COMP2_OUT:           tim1_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM1_TI1_COMP3_OUT:           tim1_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM1_TI1_COMP4_OUT:           tim1_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM1_TI1_PLAY1_OUT0:          tim1_ti1 is connected to play1_out0 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM1_TI2_GPIO:                tim1_ti2 is connected to TIM1_CH2
  *            LL_TIM_TIM1_TI2_PLAY1_OUT2:          tim1_ti2 is connected to play1_out2 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM1_TI3_GPIO:                tim1_ti3 is connected to TIM1_CH3
  *            LL_TIM_TIM1_TI3_PLAY1_OUT1:          tim1_ti3 is connected to play1_out1 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM1_TI4_GPIO:                tim1_ti4 is connected to TIM1_CH4
  *            LL_TIM_TIM1_TI4_PLAY1_OUT8:          tim1_ti4 is connected to play1_out8 (*)
  *
  *         TIM2: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM2_TI1_GPIO:                tim2_ti1 is connected to TIM2_CH1
  *            LL_TIM_TIM2_TI1_COMP1_OUT:           tim2_ti1 is connected to comp1_out
  *            LL_TIM_TIM2_TI1_COMP2_OUT:           tim2_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM2_TI1_ETH1_PTP_PPS_OUT:    tim2_ti1 is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM2_TI1_LSI:                 tim2_ti1 is connected to LSI
  *            LL_TIM_TIM2_TI1_LSE:                 tim2_ti1 is connected to LSE
  *            LL_TIM_TIM2_TI1_RTC_WUT_TRG:         tim2_ti1 is connected to rtc_wut_trg
  *            LL_TIM_TIM2_TI1_TIM5_CH1:            tim2_ti1 is connected to TIM5_CH1 (*)
  *            LL_TIM_TIM2_TI1_FDCAN1_RXEOF_EVT:    tim2_ti1 is connected to fdcan1_rxeof_evt (*)
  *            LL_TIM_TIM2_TI1_COMP3_OUT:           tim2_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM2_TI1_COMP4_OUT:           tim2_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM2_TI1_PLAY1_OUT15:         tim2_ti1 is connected to play1_out15 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM2_TI2_GPIO:                tim2_ti2 is connected to TIM2_CH2
  *            LL_TIM_TIM2_TI2_HSE_RTC:             tim2_ti2 is connected to hse_1M_ck
  *            LL_TIM_TIM2_TI2_MCO1:                tim2_ti2 is connected to MCO1
  *            LL_TIM_TIM2_TI2_MCO2:                tim2_ti2 is connected to MCO2
  *            LL_TIM_TIM2_TI2_FDCAN1_TXEOF_EVT:    tim2_ti2 is connected to fdcan1_txeof_evt (*)
  *            LL_TIM_TIM2_TI2_PLAY1_OUT15:         tim2_ti2 is connected to play1_out15 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM2_TI3_GPIO:                tim2_ti3 is connected to TIM2_CH3
  *            LL_TIM_TIM2_TI3_PLAY1_OUT3:          tim2_ti3 is connected to play1_out3 (*)
  *            LL_TIM_TIM2_TI3_FDCAN2_RXEOF_EVT:    tim2_ti3 is connected to fdcan2_rxeof_evt (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM2_TI4_GPIO:                tim2_ti4 is connected to TIM2_CH4
  *            LL_TIM_TIM2_TI4_COMP1_OUT:           tim2_ti4 is connected to comp1_out
  *            LL_TIM_TIM2_TI4_COMP2_OUT:           tim2_ti4 is connected to comp2_out (*)
  *            LL_TIM_TIM2_TI4_PLAY1_OUT0:          tim2_ti4 is connected to play1_out0 (*)
  *            LL_TIM_TIM2_TI4_FDCAN2_TXEOF_EVT:    tim2_ti4 is connected to fdcan2_txeof_evt (*)
  *
  *         TIM3: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM3_TI1_GPIO:                tim3_ti1 is connected to TIM3_CH1
  *            LL_TIM_TIM3_TI1_COMP1_OUT:           tim3_ti1 is connected to comp1_out
  *            LL_TIM_TIM3_TI1_ETH1_PTP_PPS_OUT:    tim3_ti1 is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM3_TI1_FDCAN2_RXEOF_EVT:    tim3_ti1 is connected to fdcan2_rxeof_evt (*)
  *            LL_TIM_TIM3_TI1_COMP2_OUT:           tim3_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM3_TI1_COMP3_OUT:           tim3_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM3_TI1_COMP4_OUT:           tim3_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM3_TI1_PLAY1_OUT8:          tim3_ti1 is connected to play1_out8 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM3_TI2_GPIO:                tim3_ti2 is connected to TIM3_CH2
  *            LL_TIM_TIM3_TI2_FDCAN2_TXEOF_EVT:    tim3_ti2 is connected to fdcan2_txeof_evt (*)
  *            LL_TIM_TIM3_TI2_PLAY1_OUT9:          tim3_ti2 is connected to play1_out9 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM3_TI3_GPIO:                tim3_ti3 is connected to TIM3_CH3
  *            LL_TIM_TIM3_TI3_PLAY1_OUT10:         tim3_ti3 is connected to play1_out10 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM3_TI4_GPIO:                tim3_ti4 is connected to TIM3_CH4
  *            LL_TIM_TIM3_TI4_PLAY1_OUT11:         tim3_ti4 is connected to play1_out11 (*)
  *
  *         TIM4: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM4_TI1_GPIO:                tim4_ti1 is connected to TIM4_CH1
  *            LL_TIM_TIM4_TI1_COMP1_OUT:           tim4_ti1 is connected to comp1_out
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM4_TI2_GPIO:                tim4_ti2 is connected to TIM4_CH2
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM4_TI3_GPIO:                tim4_ti3 is connected to TIM4_CH3
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM4_TI4_GPIO:                tim4_ti4 is connected to TIM4_CH4
  *
  *         TIM5: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM5_TI1_GPIO:                tim5_ti1 is connected to TIM5_CH1
  *            LL_TIM_TIM5_TI1_COMP1_OUT:           tim5_ti1 is connected to comp1_out
  *            LL_TIM_TIM5_TI1_COMP2_OUT:           tim5_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM5_TI1_COMP3_OUT:           tim5_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM5_TI1_COMP4_OUT:           tim5_ti1 is connected to comp4_out (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM5_TI2_GPIO:                tim5_ti2 is connected to TIM5_CH2
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM5_TI3_GPIO:                tim5_ti3 is connected to TIM5_CH3
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM5_TI4_GPIO:                tim5_ti4 is connected to TIM5_CH4
  *
  *         TIM8: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM8_TI1_GPIO:                tim8_ti1 is connected to TIM8_CH1
  *            LL_TIM_TIM8_TI1_COMP1_OUT:           tim8_ti1 is connected to comp1_out
  *            LL_TIM_TIM8_TI1_COMP2_OUT:           tim8_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM8_TI1_COMP3_OUT:           tim8_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM8_TI1_COMP4_OUT:           tim8_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM8_TI1_PLAY1_OUT14:         tim8_ti1 is connected to play1_out14 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM8_TI2_GPIO:                tim8_ti2 is connected to TIM8_CH2
  *            LL_TIM_TIM8_TI2_PLAY1_OUT10:         tim8_ti2 is connected to play1_out10 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM8_TI3_GPIO:                tim8_ti3 is connected to TIM8_CH3
  *            LL_TIM_TIM8_TI3_PLAY1_OUT12:         tim8_ti3 is connected to play1_out12 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM8_TI4_GPIO:                tim8_ti4 is connected to TIM8_CH4
  *            LL_TIM_TIM8_TI4_PLAY1_OUT5:          tim8_ti4 is connected to play1_out5 (*)
  *
  *         TIM12: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM12_TI1_GPIO:               tim12_ti1 is connected to TIM12_CH1
  *            LL_TIM_TIM12_TI1_COMP1_OUT:          tim12_ti1 is connected to comp1_out
  *            LL_TIM_TIM12_TI1_COMP2_OUT:          tim12_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM12_TI1_MCO1:               tim12_ti1 is connected to MCO1
  *            LL_TIM_TIM12_TI1_MCO2:               tim12_ti1 is connected to MCO2
  *            LL_TIM_TIM12_TI1_HSE_RTC:            tim12_ti1 is connected to hse_1M_ck
  *            LL_TIM_TIM12_TI1_I3C1_IBI_ACK:       tim12_ti1 is connected to i3c1_ibi_ack
  *            LL_TIM_TIM12_TI1_COMP3_OUT:          tim12_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM12_TI1_COMP4_OUT:          tim12_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM12_TI1_PLAY1_OUT7:         tim12_ti1 is connected to play1_out7 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM12_TI2_GPIO:               tim12_ti2 is connected to TIM12_CH2
  *            LL_TIM_TIM12_TI2_PLAY1_OUT14:        tim12_ti2 is connected to play1_out14 (*)
  *
  *         TIM15: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM15_TI1_GPIO:               tim15_ti1 is connected to TIM15_CH1
  *            LL_TIM_TIM15_TI1_COMP1_OUT:          tim15_ti1 is connected to comp1_out
  *            LL_TIM_TIM15_TI1_COMP2_OUT:          tim15_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM15_TI1_LSE:                tim15_ti1 is connected to LSE
  *            LL_TIM_TIM15_TI1_FDCAN2_RXEOF_EVT:   tim15_ti1 is connected to fdcan2_rxeof_evt (*)
  *            LL_TIM_TIM15_TI1_COMP3_OUT:          tim15_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM15_TI1_COMP4_OUT:          tim15_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM15_TI1_PLAY1_OUT4:         tim15_ti1 is connected to play1_out4 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM15_TI2_GPIO:               tim15_ti2 is connected to TIM15_CH2
  *            LL_TIM_TIM15_TI2_FDCAN2_TXEOF_EVT:   tim15_ti2 is connected to fdcan2_txeof_evt (*)
  *            LL_TIM_TIM15_TI2_PLAY1_OUT5:         tim15_ti2 is connected to play1_out5 (*)
  *
  *         TIM16: one of the following values: (**)
  *            LL_TIM_TIM16_TI1_GPIO:               tim16_ti1 is connected to TIM16_CH1
  *            LL_TIM_TIM16_TI1_COMP1_OUT:          tim16_ti1 is connected to comp1_out
  *            LL_TIM_TIM16_TI1_LSI:                tim16_ti1 is connected to LSI
  *            LL_TIM_TIM16_TI1_LSE:                tim16_ti1 is connected to LSE
  *            LL_TIM_TIM16_TI1_RTC_WUT_TRG:        tim16_ti1 is connected to rtc_wut_trg
  *            LL_TIM_TIM16_TI1_MCO1:               tim16_ti1 is connected to MCO1
  *            LL_TIM_TIM16_TI1_MCO2:               tim16_ti1 is connected to MCO2
  *            LL_TIM_TIM16_TI1_COMP2_OUT:          tim16_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM16_TI1_COMP3_OUT:          tim16_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM16_TI1_COMP4_OUT:          tim16_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM16_TI1_PLAY1_OUT5:         tim16_ti1 is connected to play1_out5 (*)
  *
  *         TIM17: one of the following values: (**)
  *            LL_TIM_TIM17_TI1_GPIO:               tim17_ti1 is connected to TIM17_CH1
  *            LL_TIM_TIM17_TI1_COMP1_OUT:          tim17_ti1 is connected to comp1_out
  *            LL_TIM_TIM17_TI1_HSE_RTC:            tim17_ti1 is connected to hse_1M_ck
  *            LL_TIM_TIM17_TI1_MCO1:               tim17_ti1 is connected to MCO1
  *            LL_TIM_TIM17_TI1_MCO2:               tim17_ti1 is connected to MCO2
  *            LL_TIM_TIM17_TI1_I3C1_IBI_ACK:       tim17_ti1 is connected to i3c1_ibi_ack
  *            LL_TIM_TIM17_TI1_COMP2_OUT:          tim17_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM17_TI1_COMP3_OUT:          tim17_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM17_TI1_COMP4_OUT:          tim17_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM17_TI1_PLAY1_OUT9:         tim17_ti1 is connected to play1_out9 (*)
  *
  *         TIM20: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM20_TI1_GPIO:               tim20_ti1 is connected to TIM20_CH1
  *            LL_TIM_TIM20_TI1_COMP1_OUT:          tim20_ti1 is connected to comp1_out
  *            LL_TIM_TIM20_TI1_COMP2_OUT:          tim20_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM20_TI1_COMP3_OUT:          tim20_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM20_TI1_COMP4_OUT:          tim20_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM20_TI1_PLAY1_OUT3:         tim20_ti1 is connected to play1_out3 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM20_TI2_GPIO:               tim20_ti2 is connected to TIM20_CH2
  *            LL_TIM_TIM20_TI2_PLAY1_OUT7:         tim20_ti2 is connected to play1_out7 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM20_TI3_GPIO:               tim20_ti3 is connected to TIM20_CH3
  *            LL_TIM_TIM20_TI3_PLAY1_OUT8:         tim20_ti3 is connected to play1_out8 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM20_TI4_GPIO:               tim20_ti4 is connected to TIM20_CH4
  *            LL_TIM_TIM20_TI4_PLAY1_OUT9:         tim20_ti4 is connected to play1_out9 (*)
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.
  * @note Macro IS_TIM_REMAP_INSTANCE(timx) can be used to check whether or not
  *       a some timer inputs can be remapped.
  */
__STATIC_INLINE void LL_TIM_IC_SetSource(TIM_TypeDef *timx, uint32_t ti_source)
{
  STM32_MODIFY_REG(timx->TISEL, (TIM_TISEL_TI1SEL | TIM_TISEL_TI2SEL | TIM_TISEL_TI3SEL | TIM_TISEL_TI4SEL), ti_source);
}

/**
  * @brief  Get the source of the input channel.
  * @rmtoll
  *  TISEL    TI1SEL      LL_TIM_IC_GetSource \n
  *  TISEL    TI2SEL      LL_TIM_IC_GetSource \n
  *  TISEL    TI3SEL      LL_TIM_IC_GetSource \n
  *  TISEL    TI4SEL      LL_TIM_IC_GetSource
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *           @arg @ref LL_TIM_CHANNEL_CH1
  *           @arg @ref LL_TIM_CHANNEL_CH2
  *           @arg @ref LL_TIM_CHANNEL_CH3
  *           @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *
  *         TIM1: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM1_TI1_GPIO:                tim1_ti1 is connected to TIM1_CH1
  *            LL_TIM_TIM1_TI1_COMP1_OUT:           tim1_ti1 is connected to comp1_out
  *            LL_TIM_TIM1_TI1_COMP2_OUT:           tim1_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM1_TI1_COMP3_OUT:           tim1_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM1_TI1_COMP4_OUT:           tim1_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM1_TI1_PLAY1_OUT0:          tim1_ti1 is connected to play1_out0 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM1_TI2_GPIO:                tim1_ti2 is connected to TIM1_CH2
  *            LL_TIM_TIM1_TI2_PLAY1_OUT2:          tim1_ti2 is connected to play1_out2 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM1_TI3_GPIO:                tim1_ti3 is connected to TIM1_CH3
  *            LL_TIM_TIM1_TI3_PLAY1_OUT1:          tim1_ti3 is connected to play1_out1 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM1_TI4_GPIO:                tim1_ti4 is connected to TIM1_CH4
  *            LL_TIM_TIM1_TI4_PLAY1_OUT8:          tim1_ti4 is connected to play1_out8 (*)
  *
  *         TIM2: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM2_TI1_GPIO:                tim2_ti1 is connected to TIM2_CH1
  *            LL_TIM_TIM2_TI1_COMP1_OUT:           tim2_ti1 is connected to comp1_out
  *            LL_TIM_TIM2_TI1_COMP2_OUT:           tim2_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM2_TI1_ETH1_PTP_PPS_OUT:    tim2_ti1 is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM2_TI1_LSI:                 tim2_ti1 is connected to LSI
  *            LL_TIM_TIM2_TI1_LSE:                 tim2_ti1 is connected to LSE
  *            LL_TIM_TIM2_TI1_RTC_WUT_TRG:         tim2_ti1 is connected to rtc_wut_trg
  *            LL_TIM_TIM2_TI1_TIM5_CH1:            tim2_ti1 is connected to TIM5_CH1 (*)
  *            LL_TIM_TIM2_TI1_FDCAN1_RXEOF_EVT:    tim2_ti1 is connected to fdcan1_rxeof_evt (*)
  *            LL_TIM_TIM2_TI1_COMP3_OUT:           tim2_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM2_TI1_COMP4_OUT:           tim2_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM2_TI1_PLAY1_OUT15:         tim2_ti1 is connected to play1_out15 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM2_TI2_GPIO:                tim2_ti2 is connected to TIM2_CH2
  *            LL_TIM_TIM2_TI2_HSE_RTC:             tim2_ti2 is connected to hse_1M_ck
  *            LL_TIM_TIM2_TI2_MCO1:                tim2_ti2 is connected to MCO1
  *            LL_TIM_TIM2_TI2_MCO2:                tim2_ti2 is connected to MCO2
  *            LL_TIM_TIM2_TI2_FDCAN1_TXEOF_EVT:    tim2_ti2 is connected to fdcan1_txeof_evt (*)
  *            LL_TIM_TIM2_TI2_PLAY1_OUT15:         tim2_ti2 is connected to play1_out15 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM2_TI3_GPIO:                tim2_ti3 is connected to TIM2_CH3
  *            LL_TIM_TIM2_TI3_PLAY1_OUT3:          tim2_ti3 is connected to play1_out3 (*)
  *            LL_TIM_TIM2_TI3_FDCAN2_RXEOF_EVT:    tim2_ti3 is connected to fdcan2_rxeof_evt (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM2_TI4_GPIO:                tim2_ti4 is connected to TIM2_CH4
  *            LL_TIM_TIM2_TI4_COMP1_OUT:           tim2_ti4 is connected to comp1_out
  *            LL_TIM_TIM2_TI4_COMP2_OUT:           tim2_ti4 is connected to comp2_out (*)
  *            LL_TIM_TIM2_TI4_PLAY1_OUT0:          tim2_ti4 is connected to play1_out0 (*)
  *            LL_TIM_TIM2_TI4_FDCAN2_TXEOF_EVT:    tim2_ti4 is connected to fdcan2_txeof_evt (*)
  *
  *         TIM3: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM3_TI1_GPIO:                tim3_ti1 is connected to TIM3_CH1
  *            LL_TIM_TIM3_TI1_COMP1_OUT:           tim3_ti1 is connected to comp1_out
  *            LL_TIM_TIM3_TI1_ETH1_PTP_PPS_OUT:    tim3_ti1 is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM3_TI1_FDCAN2_RXEOF_EVT:    tim3_ti1 is connected to fdcan2_rxeof_evt (*)
  *            LL_TIM_TIM3_TI1_COMP2_OUT:           tim3_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM3_TI1_COMP3_OUT:           tim3_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM3_TI1_COMP4_OUT:           tim3_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM3_TI1_PLAY1_OUT8:          tim3_ti1 is connected to play1_out8 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM3_TI2_GPIO:                tim3_ti2 is connected to TIM3_CH2
  *            LL_TIM_TIM3_TI2_FDCAN2_TXEOF_EVT:    tim3_ti2 is connected to fdcan2_txeof_evt (*)
  *            LL_TIM_TIM3_TI2_PLAY1_OUT9:          tim3_ti2 is connected to play1_out9 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM3_TI3_GPIO:                tim3_ti3 is connected to TIM3_CH3
  *            LL_TIM_TIM3_TI3_PLAY1_OUT10:         tim3_ti3 is connected to play1_out10 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM3_TI4_GPIO:                tim3_ti4 is connected to TIM3_CH4
  *            LL_TIM_TIM3_TI4_PLAY1_OUT11:         tim3_ti4 is connected to play1_out11 (*)
  *
  *         TIM4: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM4_TI1_GPIO:                tim4_ti1 is connected to TIM4_CH1
  *            LL_TIM_TIM4_TI1_COMP1_OUT:           tim4_ti1 is connected to comp1_out
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM4_TI2_GPIO:                tim4_ti2 is connected to TIM4_CH2
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM4_TI3_GPIO:                tim4_ti3 is connected to TIM4_CH3
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM4_TI4_GPIO:                tim4_ti4 is connected to TIM4_CH4
  *
  *         TIM5: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM5_TI1_GPIO:                tim5_ti1 is connected to TIM5_CH1
  *            LL_TIM_TIM5_TI1_COMP1_OUT:           tim5_ti1 is connected to comp1_out
  *            LL_TIM_TIM5_TI1_COMP2_OUT:           tim5_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM5_TI1_COMP3_OUT:           tim5_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM5_TI1_COMP4_OUT:           tim5_ti1 is connected to comp4_out (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM5_TI2_GPIO:                tim5_ti2 is connected to TIM5_CH2
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM5_TI3_GPIO:                tim5_ti3 is connected to TIM5_CH3
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM5_TI4_GPIO:                tim5_ti4 is connected to TIM5_CH4
  *
  *         TIM8: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM8_TI1_GPIO:                tim8_ti1 is connected to TIM8_CH1
  *            LL_TIM_TIM8_TI1_COMP1_OUT:           tim8_ti1 is connected to comp1_out
  *            LL_TIM_TIM8_TI1_COMP2_OUT:           tim8_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM8_TI1_COMP3_OUT:           tim8_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM8_TI1_COMP4_OUT:           tim8_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM8_TI1_PLAY1_OUT14:         tim8_ti1 is connected to play1_out14 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM8_TI2_GPIO:                tim8_ti2 is connected to TIM8_CH2
  *            LL_TIM_TIM8_TI2_PLAY1_OUT10:         tim8_ti2 is connected to play1_out10 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM8_TI3_GPIO:                tim8_ti3 is connected to TIM8_CH3
  *            LL_TIM_TIM8_TI3_PLAY1_OUT12:         tim8_ti3 is connected to play1_out12 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM8_TI4_GPIO:                tim8_ti4 is connected to TIM8_CH4
  *            LL_TIM_TIM8_TI4_PLAY1_OUT5:          tim8_ti4 is connected to play1_out5 (*)
  *
  *         TIM12: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM12_TI1_GPIO:               tim12_ti1 is connected to TIM12_CH1
  *            LL_TIM_TIM12_TI1_COMP1_OUT:          tim12_ti1 is connected to comp1_out
  *            LL_TIM_TIM12_TI1_COMP2_OUT:          tim12_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM12_TI1_MCO1:               tim12_ti1 is connected to MCO1
  *            LL_TIM_TIM12_TI1_MCO2:               tim12_ti1 is connected to MCO2
  *            LL_TIM_TIM12_TI1_HSE_RTC:            tim12_ti1 is connected to hse_1M_ck
  *            LL_TIM_TIM12_TI1_I3C1_IBI_ACK:       tim12_ti1 is connected to i3c1_ibi_ack
  *            LL_TIM_TIM12_TI1_COMP3_OUT:          tim12_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM12_TI1_COMP4_OUT:          tim12_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM12_TI1_PLAY1_OUT7:         tim12_ti1 is connected to play1_out7 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM12_TI2_GPIO:               tim12_ti2 is connected to TIM12_CH2
  *            LL_TIM_TIM12_TI2_PLAY1_OUT14:        tim12_ti2 is connected to play1_out14 (*)
  *
  *         TIM15: one of the following values:
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM15_TI1_GPIO:               tim15_ti1 is connected to TIM15_CH1
  *            LL_TIM_TIM15_TI1_COMP1_OUT:          tim15_ti1 is connected to comp1_out
  *            LL_TIM_TIM15_TI1_COMP2_OUT:          tim15_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM15_TI1_LSE:                tim15_ti1 is connected to LSE
  *            LL_TIM_TIM15_TI1_FDCAN2_RXEOF_EVT:   tim15_ti1 is connected to fdcan2_rxeof_evt (*)
  *            LL_TIM_TIM15_TI1_COMP3_OUT:          tim15_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM15_TI1_COMP4_OUT:          tim15_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM15_TI1_PLAY1_OUT4:         tim15_ti1 is connected to play1_out4 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM15_TI2_GPIO:               tim15_ti2 is connected to TIM15_CH2
  *            LL_TIM_TIM15_TI2_FDCAN2_TXEOF_EVT:   tim15_ti2 is connected to fdcan2_txeof_evt (*)
  *            LL_TIM_TIM15_TI2_PLAY1_OUT5:         tim15_ti2 is connected to play1_out5 (*)
  *
  *         TIM16: one of the following values: (**)
  *            LL_TIM_TIM16_TI1_GPIO:               tim16_ti1 is connected to TIM16_CH1
  *            LL_TIM_TIM16_TI1_COMP1_OUT:          tim16_ti1 is connected to comp1_out
  *            LL_TIM_TIM16_TI1_LSI:                tim16_ti1 is connected to LSI
  *            LL_TIM_TIM16_TI1_LSE:                tim16_ti1 is connected to LSE
  *            LL_TIM_TIM16_TI1_RTC_WUT_TRG:        tim16_ti1 is connected to rtc_wut_trg
  *            LL_TIM_TIM16_TI1_MCO1:               tim16_ti1 is connected to MCO1
  *            LL_TIM_TIM16_TI1_MCO2:               tim16_ti1 is connected to MCO2
  *            LL_TIM_TIM16_TI1_COMP2_OUT:          tim16_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM16_TI1_COMP3_OUT:          tim16_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM16_TI1_COMP4_OUT:          tim16_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM16_TI1_PLAY1_OUT5:         tim16_ti1 is connected to play1_out5 (*)
  *
  *         TIM17: one of the following values: (**)
  *            LL_TIM_TIM17_TI1_GPIO:               tim17_ti1 is connected to TIM17_CH1
  *            LL_TIM_TIM17_TI1_COMP1_OUT:          tim17_ti1 is connected to comp1_out
  *            LL_TIM_TIM17_TI1_HSE_RTC:            tim17_ti1 is connected to hse_1M_ck
  *            LL_TIM_TIM17_TI1_MCO1:               tim17_ti1 is connected to MCO1
  *            LL_TIM_TIM17_TI1_MCO2:               tim17_ti1 is connected to MCO2
  *            LL_TIM_TIM17_TI1_I3C1_IBI_ACK:       tim17_ti1 is connected to i3c1_ibi_ack
  *            LL_TIM_TIM17_TI1_COMP2_OUT:          tim17_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM17_TI1_COMP3_OUT:          tim17_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM17_TI1_COMP4_OUT:          tim17_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM17_TI1_PLAY1_OUT9:         tim17_ti1 is connected to play1_out9 (*)
  *
  *         TIM20: one of the following values: (**)
  *
  *            . . TI1_RMP can be one of the following values
  *            LL_TIM_TIM20_TI1_GPIO:               tim20_ti1 is connected to TIM20_CH1
  *            LL_TIM_TIM20_TI1_COMP1_OUT:          tim20_ti1 is connected to comp1_out
  *            LL_TIM_TIM20_TI1_COMP2_OUT:          tim20_ti1 is connected to comp2_out (*)
  *            LL_TIM_TIM20_TI1_COMP3_OUT:          tim20_ti1 is connected to comp3_out (*)
  *            LL_TIM_TIM20_TI1_COMP4_OUT:          tim20_ti1 is connected to comp4_out (*)
  *            LL_TIM_TIM20_TI1_PLAY1_OUT3:         tim20_ti1 is connected to play1_out3 (*)
  *
  *            . . TI2_RMP can be one of the following values
  *            LL_TIM_TIM20_TI2_GPIO:               tim20_ti2 is connected to TIM20_CH2
  *            LL_TIM_TIM20_TI2_PLAY1_OUT7:         tim20_ti2 is connected to play1_out7 (*)
  *
  *            . . TI3_RMP can be one of the following values
  *            LL_TIM_TIM20_TI3_GPIO:               tim20_ti3 is connected to TIM20_CH3
  *            LL_TIM_TIM20_TI3_PLAY1_OUT8:         tim20_ti3 is connected to play1_out8 (*)
  *
  *            . . TI4_RMP can be one of the following values
  *            LL_TIM_TIM20_TI4_GPIO:               tim20_ti4 is connected to TIM20_CH4
  *            LL_TIM_TIM20_TI4_PLAY1_OUT9:         tim20_ti4 is connected to play1_out9 (*)
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.
  * @note Macro IS_TIM_REMAP_INSTANCE(timx) can be used to check whether or not
  *       a some timer inputs can be remapped.
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetSource(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  uint32_t shift = (ichannel << 2U) & 0x18U;
  return (uint32_t)(STM32_READ_BIT(timx->TISEL, TIM_TISEL_TI1SEL << shift));
}

/**
  * @brief  Set the active input.
  * @rmtoll
  *  CCMR1        CC1S          LL_TIM_IC_SetActiveInput \n
  *  CCMR1        CC2S          LL_TIM_IC_SetActiveInput \n
  *  CCMR2        CC3S          LL_TIM_IC_SetActiveInput \n
  *  CCMR2        CC4S          LL_TIM_IC_SetActiveInput
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ic_active_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECT
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECT
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  */
__STATIC_INLINE void LL_TIM_IC_SetActiveInput(TIM_TypeDef *timx, uint32_t channel, uint32_t ic_active_input)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_MODIFY_REG(*pReg, ((TIM_CCMR1_CC1S) << LL_TIM_SHIFT_TAB_ICxx[ichannel]),
                   (ic_active_input >> LL_TIM_IC_CONFIG_POS) << LL_TIM_SHIFT_TAB_ICxx[ichannel]);
}

/**
  * @brief  Get the current active input.
  * @rmtoll
  *  CCMR1        CC1S          LL_TIM_IC_GetActiveInput \n
  *  CCMR1        CC2S          LL_TIM_IC_GetActiveInput \n
  *  CCMR2        CC3S          LL_TIM_IC_GetActiveInput \n
  *  CCMR2        CC4S          LL_TIM_IC_GetActiveInput
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECT
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECT
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetActiveInput(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  return ((STM32_READ_BIT(*pReg, ((TIM_CCMR1_CC1S) << LL_TIM_SHIFT_TAB_ICxx[ichannel])) \
           >> LL_TIM_SHIFT_TAB_ICxx[ichannel]) << LL_TIM_IC_CONFIG_POS);
}

/**
  * @brief  Set the prescaler of input channel.
  * @rmtoll
  *  CCMR1        IC1PSC        LL_TIM_IC_SetPrescaler \n
  *  CCMR1        IC2PSC        LL_TIM_IC_SetPrescaler \n
  *  CCMR2        IC3PSC        LL_TIM_IC_SetPrescaler \n
  *  CCMR2        IC4PSC        LL_TIM_IC_SetPrescaler
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ic_prescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  */
__STATIC_INLINE void LL_TIM_IC_SetPrescaler(TIM_TypeDef *timx, uint32_t channel, uint32_t ic_prescaler)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_MODIFY_REG(*pReg, ((TIM_CCMR1_IC1PSC) << LL_TIM_SHIFT_TAB_ICxx[ichannel]),
                   (ic_prescaler >> LL_TIM_IC_CONFIG_POS) << LL_TIM_SHIFT_TAB_ICxx[ichannel]);
}

/**
  * @brief  Get the current prescaler value acting on an  input channel.
  * @rmtoll
  *  CCMR1        IC1PSC        LL_TIM_IC_GetPrescaler \n
  *  CCMR1        IC2PSC        LL_TIM_IC_GetPrescaler \n
  *  CCMR2        IC3PSC        LL_TIM_IC_GetPrescaler \n
  *  CCMR2        IC4PSC        LL_TIM_IC_GetPrescaler
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetPrescaler(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  return ((STM32_READ_BIT(*pReg, ((TIM_CCMR1_IC1PSC) << LL_TIM_SHIFT_TAB_ICxx[ichannel])) \
           >> LL_TIM_SHIFT_TAB_ICxx[ichannel]) << LL_TIM_IC_CONFIG_POS);
}

/**
  * @brief  Set the input filter duration.
  * @rmtoll
  *  CCMR1        IC1F          LL_TIM_IC_SetFilter \n
  *  CCMR1        IC2F          LL_TIM_IC_SetFilter \n
  *  CCMR2        IC3F          LL_TIM_IC_SetFilter \n
  *  CCMR2        IC4F          LL_TIM_IC_SetFilter
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ic_filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  */
__STATIC_INLINE void LL_TIM_IC_SetFilter(TIM_TypeDef *timx, uint32_t channel, uint32_t ic_filter)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  STM32_MODIFY_REG(*pReg, ((TIM_CCMR1_IC1F) << LL_TIM_SHIFT_TAB_ICxx[ichannel]),
                   (ic_filter >> LL_TIM_IC_CONFIG_POS) << LL_TIM_SHIFT_TAB_ICxx[ichannel]);
}

/**
  * @brief  Get the input filter duration.
  * @rmtoll
  *  CCMR1        IC1F          LL_TIM_IC_GetFilter \n
  *  CCMR1        IC2F          LL_TIM_IC_GetFilter \n
  *  CCMR2        IC3F          LL_TIM_IC_GetFilter \n
  *  CCMR2        IC4F          LL_TIM_IC_GetFilter
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetFilter(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCMR1) + \
                                                           LL_TIM_OFFSET_TAB_CCMRx[ichannel]));
  return ((STM32_READ_BIT(*pReg, ((TIM_CCMR1_IC1F) << LL_TIM_SHIFT_TAB_ICxx[ichannel])) \
           >> LL_TIM_SHIFT_TAB_ICxx[ichannel]) << LL_TIM_IC_CONFIG_POS);
}

/**
  * @brief  Set the input channel polarity.
  * @rmtoll
  *  CCER         CC1P          LL_TIM_IC_SetPolarity \n
  *  CCER         CC1NP         LL_TIM_IC_SetPolarity \n
  *  CCER         CC2P          LL_TIM_IC_SetPolarity \n
  *  CCER         CC2NP         LL_TIM_IC_SetPolarity \n
  *  CCER         CC3P          LL_TIM_IC_SetPolarity \n
  *  CCER         CC3NP         LL_TIM_IC_SetPolarity \n
  *  CCER         CC4P          LL_TIM_IC_SetPolarity \n
  *  CCER         CC4NP         LL_TIM_IC_SetPolarity
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ic_polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_RISING_FALLING
  */
__STATIC_INLINE void LL_TIM_IC_SetPolarity(TIM_TypeDef *timx, uint32_t channel, uint32_t ic_polarity)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  STM32_MODIFY_REG(timx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << LL_TIM_SHIFT_TAB_CCxP[ichannel]),
                   ic_polarity << LL_TIM_SHIFT_TAB_CCxP[ichannel]);
}

/**
  * @brief  Get the current input channel polarity.
  * @rmtoll
  *  CCER         CC1P          LL_TIM_IC_GetPolarity \n
  *  CCER         CC1NP         LL_TIM_IC_GetPolarity \n
  *  CCER         CC2P          LL_TIM_IC_GetPolarity \n
  *  CCER         CC2NP         LL_TIM_IC_GetPolarity \n
  *  CCER         CC3P          LL_TIM_IC_GetPolarity \n
  *  CCER         CC3NP         LL_TIM_IC_GetPolarity \n
  *  CCER         CC4P          LL_TIM_IC_GetPolarity \n
  *  CCER         CC4NP         LL_TIM_IC_GetPolarity
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_RISING_FALLING
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetPolarity(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  return (STM32_READ_BIT(timx->CCER, ((TIM_CCER_CC1NP | TIM_CCER_CC1P) << LL_TIM_SHIFT_TAB_CCxP[ichannel])) >>
          LL_TIM_SHIFT_TAB_CCxP[ichannel]);
}

/**
  * @brief  Set the XOR gate position.
  * @rmtoll
  *  CR2          XORPS          LL_TIM_IC_SetXORGatePosition
  * @param  timx Timer instance
  * @param  xor_position This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_XOR_GATE_POS_DIRECT
  *         @arg @ref LL_TIM_IC_XOR_GATE_POS_FILTERED
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a XOR input.
  * @note Macro IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance has at least 3 channels.
  */
__STATIC_INLINE void LL_TIM_IC_SetXORGatePosition(TIM_TypeDef *timx, uint32_t xor_position)
{
  STM32_MODIFY_REG(timx->CR2, TIM_CR2_XORPS, xor_position);
}

/**
  * @brief  Get the XOR gate position.
  * @rmtoll
  *  CR2          XORPS          LL_TIM_IC_SetXORGatePosition
  * @param  timx Timer instance
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a XOR input.
  * @note Macro IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance has at least 3 channels.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_XOR_GATE_POS_DIRECT
  *         @arg @ref LL_TIM_IC_XOR_GATE_POS_FILTERED
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetXORGatePosition(TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->CR2, TIM_CR2_XORPS));
}

/**
  * @brief  Enable the signal inversion of a XOR gate input channel.
  * @rmtoll
  *  CR2         TI1INV          LL_TIM_IC_EnableXORGateInputInversion \n
  *  CR2         TI2INV          LL_TIM_IC_EnableXORGateInputInversion \n
  *  CR2         TI3INV          LL_TIM_IC_EnableXORGateInputInversion
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a XOR input.
  */
__STATIC_INLINE void LL_TIM_IC_EnableXORGateInputInversion(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel_shift = ((uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) >> 1U) % 3U;
  STM32_SET_BIT(timx->CR2, (TIM_CR2_TI1INV << ichannel_shift));
}

/**
  * @brief  Disable the signal inversion of a XOR gate input channel.
  * @rmtoll
  *  CR2         TI1INV          LL_TIM_IC_DisableXORGateInputInversion \n
  *  CR2         TI2INV          LL_TIM_IC_DisableXORGateInputInversion \n
  *  CR2         TI3INV          LL_TIM_IC_DisableXORGateInputInversion
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a XOR input.
  */
__STATIC_INLINE void LL_TIM_IC_DisableXORGateInputInversion(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel_shift = ((uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) >> 1U) % 3U;
  STM32_CLEAR_BIT(timx->CR2, (TIM_CR2_TI1INV << ichannel_shift));
}

/**
  * @brief  Indicates whether the signal inversion of a XOR gate input channel is enabled.
  * @rmtoll
  *  CR2         TI1INV          LL_TIM_IC_IsEnabledXORGateInputInversion \n
  *  CR2         TI2INV          LL_TIM_IC_IsEnabledXORGateInputInversion \n
  *  CR2         TI3INV          LL_TIM_IC_IsEnabledXORGateInputInversion
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  * @note Macro IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a XOR input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IC_IsEnabledXORGateInputInversion(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel_shift = ((uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) >> 1U) % 3U;
  uint32_t inv = TIM_CR2_TI1INV << ichannel_shift;
  return ((STM32_READ_BIT(timx->CR2, inv) == inv) ? 1UL : 0UL);
}

/**
  * @brief  Connect the TIMx_CH1, CH2 and CH3 pins to the TI1 input (XOR combination).
  * @rmtoll
  *  CR2          TI1S          LL_TIM_IC_EnableXORCombination
  * @param  timx Timer instance
  * @note Macro IS_TIM_XOR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  */
__STATIC_INLINE void LL_TIM_IC_EnableXORCombination(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR2, TIM_CR2_TI1S);
}

/**
  * @brief  Disconnect the TIMx_CH1, CH2 and CH3 pins from the TI1 input.
  * @rmtoll
  *  CR2          TI1S          LL_TIM_IC_DisableXORCombination
  * @param  timx Timer instance
  * @note Macro IS_TIM_XOR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  */
__STATIC_INLINE void LL_TIM_IC_DisableXORCombination(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR2, TIM_CR2_TI1S);
}

/**
  * @brief  Indicates whether the TIMx_CH1, CH2 and CH3 pins are connectected to the TI1 input.
  * @rmtoll
  *  CR2          TI1S          LL_TIM_IC_IsEnabledXORCombination
  * @param  timx Timer instance
  * @note Macro IS_TIM_XOR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IC_IsEnabledXORCombination(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR2, TIM_CR2_TI1S) == (TIM_CR2_TI1S)) ? 1UL : 0UL);
}

/**
  * @brief  Get captured value for input channel 1.
  * @rmtoll
  *  CCR1         CCR1          LL_TIM_IC_GetCaptureCH1
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC1_INSTANCE(timx) can be used to check whether or not
  *       input channel 1 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH1(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR1));
}

/**
  * @brief  Get captured value for input channel 2.
  * @rmtoll
  *  CCR2         CCR2          LL_TIM_IC_GetCaptureCH2
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC2_INSTANCE(timx) can be used to check whether or not
  *       input channel 2 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH2(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR2));
}

/**
  * @brief  Get captured value for input channel 3.
  * @rmtoll
  *  CCR3         CCR3          LL_TIM_IC_GetCaptureCH3
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC3_INSTANCE(timx) can be used to check whether or not
  *       input channel 3 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH3(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR3));
}

/**
  * @brief  Get captured value for input channel 4.
  * @rmtoll
  *  CCR4         CCR4          LL_TIM_IC_GetCaptureCH4
  * @param  timx Timer instance
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note Macro IS_TIM_CC4_INSTANCE(timx) can be used to check whether or not
  *       input channel 4 is supported by a timer instance.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCaptureCH4(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CCR4));
}

/**
  * @brief  Get captured value for the selected capture unit.
  * @rmtoll
  *  CCR1         CCR1          LL_TIM_IC_GetCapturedValue \n
  *  CCR2         CCR2          LL_TIM_IC_GetCapturedValue \n
  *  CCR3         CCR3          LL_TIM_IC_GetCapturedValue \n
  *  CCR4         CCR4          LL_TIM_IC_GetCapturedValue
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro IS_TIM_32B_COUNTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports a 32-bit counter.
  * @note If dithering is activated, pay attention to the returned value interpretation.
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetCapturedValue(const TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel = LL_TIM_GET_CHANNEL_INDEX(channel);
  ichannel >>= 1U; /* Divide by 2 to comply with LL_TIM_OFFSET_TAB_CCRx indexes */
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->CCR1) + \
                                                           LL_TIM_OFFSET_TAB_CCRx[ichannel]));
  return (uint32_t)(STM32_READ_REG(*pReg));
}

/**
  * @brief  Indicate the level of input signals (after the digital filtering stage), for polling purposes.
  * @rmtoll
  *  SR           TI1FS         LL_TIM_IC_GetInputStatus \n
  *  SR           TI2FS         LL_TIM_IC_GetInputStatus \n
  *  SR           TI3FS         LL_TIM_IC_GetInputStatus \n
  *  SR           TI4FS         LL_TIM_IC_GetInputStatus
  * @param  timx Timer instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_SIGNAL_LOW
  *         @arg @ref LL_TIM_IC_SIGNAL_HIGH
  */
__STATIC_INLINE uint32_t LL_TIM_IC_GetInputStatus(TIM_TypeDef *timx, uint32_t channel)
{
  uint32_t ichannel_shift = ((uint32_t)(LL_TIM_GET_CHANNEL_INDEX(channel)) >> 1U) & 0x3U;
  uint32_t status_shifted = TIM_SR_TI1FS << ichannel_shift;
  return ((STM32_READ_BIT(timx->SR, status_shifted) == status_shifted) ? LL_TIM_IC_SIGNAL_HIGH : LL_TIM_IC_SIGNAL_LOW);
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_Clock_Selection Counter clock selection
  * @{
  */
/**
  * @brief  Enable external clock mode 2.
  * @rmtoll
  *  SMCR         ECE           LL_TIM_EnableExternalClock
  * @param  timx Timer instance
  * @note When external clock mode 2 is enabled the counter is clocked by any active edge on the ETRF signal.
  * @note Macro IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  */
__STATIC_INLINE void LL_TIM_EnableExternalClock(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->SMCR, TIM_SMCR_ECE);
}

/**
  * @brief  Disable external clock mode 2.
  * @rmtoll
  *  SMCR         ECE           LL_TIM_DisableExternalClock
  * @param  timx Timer instance
  * @note Macro IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  */
__STATIC_INLINE void LL_TIM_DisableExternalClock(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->SMCR, TIM_SMCR_ECE);
}

/**
  * @brief  Indicate whether external clock mode 2 is enabled.
  * @rmtoll
  *  SMCR         ECE           LL_TIM_IsEnabledExternalClock
  * @param  timx Timer instance
  * @note Macro IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledExternalClock(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SMCR, TIM_SMCR_ECE) == (TIM_SMCR_ECE)) ? 1UL : 0UL);
}

/**
  * @brief  Set the clock source of the counter clock.
  * @rmtoll
  *  SMCR         SMS           LL_TIM_SetClockSource \n
  *  SMCR         ECE           LL_TIM_SetClockSource
  * @param  timx Timer instance
  * @param  clock_source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLK_INTERNAL
  *         @arg @ref LL_TIM_CLK_EXTERNAL_MODE1
  *         @arg @ref LL_TIM_CLK_EXTERNAL_MODE2
  *         @arg @ref LL_TIM_CLK_ENCODER_X1_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_X1_TI2
  *         @arg @ref LL_TIM_CLK_ENCODER_X2_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_X2_TI2
  *         @arg @ref LL_TIM_CLK_ENCODER_X4_TI12
  *         @arg @ref LL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12
  *         @arg @ref LL_TIM_CLK_ENCODER_CLK_PLUS_X2
  *         @arg @ref LL_TIM_CLK_ENCODER_CLK_PLUS_X1
  *         @arg @ref LL_TIM_CLK_ENCODER_DIR_CLK_X2
  *         @arg @ref LL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12
  * @note when selected clock source is external clock mode 1, the timer input
  *       the external clock is applied is selected by calling the @ref LL_TIM_SetTriggerInput()
  *       function. This timer input must be configured by calling
  *       the @ref LL_TIM_IC_Config() function.
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports external clock mode1.
  * @note Macro IS_TIM_ETR_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @note Macro IS_TIM_ENCODER_INTERFACE_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the encoder mode.
  */
__STATIC_INLINE void LL_TIM_SetClockSource(TIM_TypeDef *timx, uint32_t clock_source)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, clock_source);
}

/**
  * @brief  Get the clock source of the counter clock.
  * @rmtoll
  *  SMCR         SMS           LL_TIM_GetClockSource \n
  *  SMCR         ECE           LL_TIM_GetClockSource
  * @param  timx Timer instance
  * @note If external clock mode 1 and external clock mode 2 are enabled
  *       at the same time, the external clock input is tim_etrf.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CLK_INTERNAL
  *         @arg @ref LL_TIM_CLK_EXTERNAL_MODE1
  *         @arg @ref LL_TIM_CLK_EXTERNAL_MODE2
  *         @arg @ref LL_TIM_CLK_ENCODER_X1_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_X1_TI2
  *         @arg @ref LL_TIM_CLK_ENCODER_X2_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_X2_TI2
  *         @arg @ref LL_TIM_CLK_ENCODER_X4_TI12
  *         @arg @ref LL_TIM_CLK_ENCODER_DEBOUNCER_X2_TI1
  *         @arg @ref LL_TIM_CLK_ENCODER_DEBOUNCER_X4_TI12
  *         @arg @ref LL_TIM_CLK_ENCODER_CLK_PLUS_X2
  *         @arg @ref LL_TIM_CLK_ENCODER_CLK_PLUS_X1
  *         @arg @ref LL_TIM_CLK_ENCODER_DIR_CLK_X2
  *         @arg @ref LL_TIM_CLK_ENCODER_DIR_CLK_X1_TI12
  */
__STATIC_INLINE uint32_t LL_TIM_GetClockSource(const TIM_TypeDef *timx)
{
  uint32_t smcr = STM32_READ_REG(timx->SMCR);

  return (uint32_t)(((smcr & TIM_SMCR_ECE) == LL_TIM_CLK_EXTERNAL_MODE2) ? \
                    LL_TIM_CLK_EXTERNAL_MODE2 : (smcr & TIM_SMCR_SMS));
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_Timer_Synchronization Timer synchronisation configuration
  * @{
  */
/**
  * @brief  Set the trigger output (TRGO) used for timer synchronization.
  * @rmtoll
  *  CR2          MMS           LL_TIM_SetTriggerOutput
  * @param  timx Timer instance
  * @param  timer_synchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO_RESET
  *         @arg @ref LL_TIM_TRGO_ENABLE
  *         @arg @ref LL_TIM_TRGO_UPDATE
  *         @arg @ref LL_TIM_TRGO_CC1IF
  *         @arg @ref LL_TIM_TRGO_OC1
  *         @arg @ref LL_TIM_TRGO_OC2
  *         @arg @ref LL_TIM_TRGO_OC3
  *         @arg @ref LL_TIM_TRGO_OC4
  *         @arg @ref LL_TIM_TRGO_ENCODER_CLK
  * @note Macro IS_TIM_MASTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance can operate as a master timer.
  */
__STATIC_INLINE void LL_TIM_SetTriggerOutput(TIM_TypeDef *timx, uint32_t timer_synchronization)
{
  STM32_MODIFY_REG(timx->CR2, TIM_CR2_MMS, timer_synchronization);
}

/**
  * @brief  Get the source of the trigger output (TRGO).
  * @rmtoll
  *  CR2          MMS           LL_TIM_GetTriggerOutput
  * @param  timx Timer instance
  * @note Macro IS_TIM_MASTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance can operate as a master timer.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_TRGO_RESET
  *         @arg @ref LL_TIM_TRGO_ENABLE
  *         @arg @ref LL_TIM_TRGO_UPDATE
  *         @arg @ref LL_TIM_TRGO_CC1IF
  *         @arg @ref LL_TIM_TRGO_OC1
  *         @arg @ref LL_TIM_TRGO_OC2
  *         @arg @ref LL_TIM_TRGO_OC3
  *         @arg @ref LL_TIM_TRGO_OC4
  *         @arg @ref LL_TIM_TRGO_ENCODER_CLK
  */
__STATIC_INLINE uint32_t LL_TIM_GetTriggerOutput(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CR2) & TIM_CR2_MMS);
}

/**
  * @brief  Set the trigger output 2 (TRGO2) used for ADC synchronization .
  * @rmtoll
  *  CR2          MMS2          LL_TIM_SetTriggerOutput2
  * @param  timx Timer Instance
  * @param  adc_synchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO2_RESET
  *         @arg @ref LL_TIM_TRGO2_ENABLE
  *         @arg @ref LL_TIM_TRGO2_UPDATE
  *         @arg @ref LL_TIM_TRGO2_CC1F
  *         @arg @ref LL_TIM_TRGO2_OC1
  *         @arg @ref LL_TIM_TRGO2_OC2
  *         @arg @ref LL_TIM_TRGO2_OC3
  *         @arg @ref LL_TIM_TRGO2_OC4
  *         @arg @ref LL_TIM_TRGO2_OC5
  *         @arg @ref LL_TIM_TRGO2_OC6
  *         @arg @ref LL_TIM_TRGO2_OC7
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC7_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC7_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC7_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_OC7_FALLING
  * @note Macro IS_TIM_TRGO2_INSTANCE(timx) can be used to check
  *       whether or not a timer instance can be used for ADC synchronization.
  */
__STATIC_INLINE void LL_TIM_SetTriggerOutput2(TIM_TypeDef *timx, uint32_t adc_synchronization)
{
  STM32_MODIFY_REG(timx->CR2, TIM_CR2_MMS2, adc_synchronization);
}

/**
  * @brief  Get the source of the trigger output 2 (TRGO2).
  * @rmtoll
  *  CR2          MMS2          LL_TIM_GetTriggerOutput2
  * @param  timx Timer Instance
  * @note Macro IS_TIM_TRGO2_INSTANCE(timx) can be used to check
  *       whether or not a timer instance can be used for ADC synchronization.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_TRGO2_RESET
  *         @arg @ref LL_TIM_TRGO2_ENABLE
  *         @arg @ref LL_TIM_TRGO2_UPDATE
  *         @arg @ref LL_TIM_TRGO2_CC1F
  *         @arg @ref LL_TIM_TRGO2_OC1
  *         @arg @ref LL_TIM_TRGO2_OC2
  *         @arg @ref LL_TIM_TRGO2_OC3
  *         @arg @ref LL_TIM_TRGO2_OC4
  *         @arg @ref LL_TIM_TRGO2_OC5
  *         @arg @ref LL_TIM_TRGO2_OC6
  *         @arg @ref LL_TIM_TRGO2_OC7
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC7_RISING_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_OC7_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC7_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC7_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISING_OC7_FALLING
  */
__STATIC_INLINE uint32_t LL_TIM_GetTriggerOutput2(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->CR2) & TIM_CR2_MMS2);
}

/**
  * @brief  Set the trigger output 2 (TRGO2) post-scaler value.
  * @rmtoll
  *  CR1          TGO2PSC        LL_TIM_SetTriggerOutput2Postscaler
  * @param  timx Timer instance
  * @param  postscaler (between Min_Data=0 and Max_Data=31)
  * @note The TRGO2 clock frequency tgo2_cktim is equal to tim_ocxrefc / (TGO2PSC[4:0] + 1).
  * @note The post-scaler can be changed on the fly as this control register is buffered.
  *       The new post-scaler ratio is taken into account at the next update event.
  */
__STATIC_INLINE void LL_TIM_SetTriggerOutput2Postscaler(TIM_TypeDef *timx, uint32_t postscaler)
{
  STM32_MODIFY_REG(timx->CR1, TIM_CR1_TGO2PSC, postscaler << TIM_CR1_TGO2PSC_Pos);
}

/**
  * @brief  Get the trigger output 2 (TRGO2) post-scaler value.
  * @rmtoll
  *  CR1          TGO2PSC        LL_TIM_GetTriggerOutput2Postscaler
  * @param  timx Timer instance
  * @retval postscaler (between Min_Data=0 and Max_Data=31)
  */
__STATIC_INLINE uint32_t LL_TIM_GetTriggerOutput2Postscaler(const TIM_TypeDef *timx)
{
  return (uint32_t)((STM32_READ_REG(timx->CR1) & TIM_CR1_TGO2PSC) >> TIM_CR1_TGO2PSC_Pos);
}

/**
  * @brief  Set the synchronization mode of a slave timer.
  * @rmtoll
  *  SMCR         SMS           LL_TIM_SetSlaveMode
  * @param  timx Timer instance
  * @param  slave_mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_SLAVEMODE_DISABLED
  *         @arg @ref LL_TIM_SLAVEMODE_RESET
  *         @arg @ref LL_TIM_SLAVEMODE_GATED
  *         @arg @ref LL_TIM_SLAVEMODE_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_GATED_RESET
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  */
__STATIC_INLINE void LL_TIM_SetSlaveMode(TIM_TypeDef *timx, uint32_t slave_mode)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_SMS, slave_mode);
}

/**
  * @brief  Get the synchronization mode of a slave timer.
  * @rmtoll
  *  SMCR         SMS           LL_TIM_GetSlaveMode
  * @param  timx Timer instance
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_SLAVEMODE_DISABLED
  *         @arg @ref LL_TIM_SLAVEMODE_RESET
  *         @arg @ref LL_TIM_SLAVEMODE_GATED
  *         @arg @ref LL_TIM_SLAVEMODE_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_RESET_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_GATED_RESET
  */
__STATIC_INLINE uint32_t LL_TIM_GetSlaveMode(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->SMCR) & TIM_SMCR_SMS);
}

/**
  * @brief  Set the selects the trigger input to be used to synchronize the counter.
  * @rmtoll
  *  SMCR         TS            LL_TIM_SetTriggerInput
  * @param  timx Timer instance
  * @param  trigger_input This parameter can be one of the following values:
  *         LL_TIM_TS_ITR0
  *         LL_TIM_TS_ITR1
  *         LL_TIM_TS_ITR2
  * @if TIM3
  *         LL_TIM_TS_ITR3
  * @endif
  * @if TIM4
  *         LL_TIM_TS_ITR4
  * @endif
  * @if TIM5
  *         LL_TIM_TS_ITR5
  * @endif
  *         LL_TIM_TS_ITR6
  *         LL_TIM_TS_ITR7
  *         LL_TIM_TS_ITR8
  *         LL_TIM_TS_ITR9
  * @if TIM16
  *         LL_TIM_TS_ITR10
  * @endif
  * @if TIM17
  *         LL_TIM_TS_ITR11
  * @endif
  * #if TIM20
  *         LL_TIM_TS_ITR12
  *         LL_TIM_TS_ITR13
  * @endif
  *         LL_TIM_TS_TI1F_ED
  *         LL_TIM_TS_TI1FP1
  *         LL_TIM_TS_TI2FP2
  *         LL_TIM_TS_ETRF
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  */
__STATIC_INLINE void LL_TIM_SetTriggerInput(TIM_TypeDef *timx, uint32_t trigger_input)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_TS, trigger_input);
}

/**
  * @brief  Get the trigger input used to synchronize the counter.
  * @rmtoll
  *  SMCR         TS            LL_TIM_GetTriggerInput
  * @param  timx Timer instance
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @retval Returned value can be one of the following values:
  *         LL_TIM_TS_ITR0
  *         LL_TIM_TS_ITR1
  *         LL_TIM_TS_ITR2
  * @if TIM3
  *         LL_TIM_TS_ITR3
  * @endif
  * @if TIM4
  *         LL_TIM_TS_ITR4
  * @endif
  * @if TIM5
  *         LL_TIM_TS_ITR5
  * @endif
  *         LL_TIM_TS_ITR6
  *         LL_TIM_TS_ITR7
  *         LL_TIM_TS_ITR8
  *         LL_TIM_TS_ITR9
  * @if TIM16
  *         LL_TIM_TS_ITR10
  * @endif
  * @if TIM17
  *         LL_TIM_TS_ITR11
  * @endif
  * #if TIM20
  *         LL_TIM_TS_ITR12
  *         LL_TIM_TS_ITR13
  * @endif
  *         LL_TIM_TS_TI1F_ED
  *         LL_TIM_TS_TI1FP1
  *         LL_TIM_TS_TI2FP2
  *         LL_TIM_TS_ETRF
  */
__STATIC_INLINE uint32_t LL_TIM_GetTriggerInput(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->SMCR) & TIM_SMCR_TS);
}

/**
  * @brief  Enable the Master/Slave mode.
  * @rmtoll
  *  SMCR         MSM           LL_TIM_EnableMasterSlaveMode
  * @param  timx Timer instance
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  */
__STATIC_INLINE void LL_TIM_EnableMasterSlaveMode(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->SMCR, TIM_SMCR_MSM);
}

/**
  * @brief  Disable the Master/Slave mode.
  * @rmtoll
  *  SMCR         MSM           LL_TIM_DisableMasterSlaveMode
  * @param  timx Timer instance
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  */
__STATIC_INLINE void LL_TIM_DisableMasterSlaveMode(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->SMCR, TIM_SMCR_MSM);
}

/**
  * @brief Indicates whether the Master/Slave mode is enabled.
  * @rmtoll
  *  SMCR         MSM           LL_TIM_IsEnabledMasterSlaveMode
  * @param  timx Timer instance
  * @note Macro IS_TIM_SLAVE_INSTANCE(timx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledMasterSlaveMode(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SMCR, TIM_SMCR_MSM) == (TIM_SMCR_MSM)) ? 1UL : 0UL);
}

/**
  * @brief  Configure the external trigger (ETR) input.
  * @rmtoll
  *  SMCR         ETP           LL_TIM_ConfigETR \n
  *  SMCR         ETPS          LL_TIM_ConfigETR \n
  *  SMCR         SETPS         LL_TIM_ConfigETR \n
  *  SMCR         ETF           LL_TIM_ConfigETR
  * @param  timx Timer instance
  * @param  etr_polarity This parameter can be one of the following values: \n
  *         @arg @ref LL_TIM_ETR_POLARITY_NONINVERTED
  *         @arg @ref LL_TIM_ETR_POLARITY_INVERTED
  * @param  etr_prescaler This parameter must be a combination of the following values: \n
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV1 or ... or @ref LL_TIM_ETR_PRESCALER_DIV8
  *         @arg @ref LL_TIM_ETR_SYNC_PRESCALER_DIV1 or ... or @ref LL_TIM_ETR_SYNC_PRESCALER_DIV16
  * @param  etr_filter This parameter can be one of the following values: \n
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N8
  * @note Macro IS_TIM_ETR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an external trigger input.
  */
__STATIC_INLINE void LL_TIM_ConfigETR(TIM_TypeDef *timx, uint32_t etr_polarity, uint32_t etr_prescaler,
                                      uint32_t etr_filter)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_ETP | TIM_SMCR_ETPS | TIM_SMCR_SETPS | TIM_SMCR_ETF,
                   etr_polarity | etr_prescaler | etr_filter);
}

/**
  * @brief  Get the external trigger (ETR) input configuration.
  * @rmtoll
  *  SMCR         ETP           LL_TIM_GetConfigETR \n
  *  SMCR         ETPS          LL_TIM_GetConfigETR \n
  *  SMCR         SETPS         LL_TIM_GetConfigETR \n
  *  SMCR         ETF           LL_TIM_GetConfigETR
  * @param  timx Timer instance
  * @param  p_etr_polarity Pointer to a storage for ETR polarity. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_ETR_POLARITY_NONINVERTED
  *         @arg @ref LL_TIM_ETR_POLARITY_INVERTED
  * @param  p_etr_prescaler Pointer to a storage for ETR prescalers (asynchronous & synchronous). \n
  *         The value can be a combination of the following values: \n
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV1 or ... or @ref LL_TIM_ETR_PRESCALER_DIV8
  *         @arg @ref LL_TIM_ETR_SYNC_PRESCALER_DIV1 or ... or @ref LL_TIM_ETR_SYNC_PRESCALER_DIV16
  * @param  p_etr_filter Pointer to a storage for ETR filter. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N8
  * @note Macro IS_TIM_ETR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an external trigger input.
  */
__STATIC_INLINE void LL_TIM_GetConfigETR(TIM_TypeDef *timx, uint32_t *p_etr_polarity, uint32_t *p_etr_prescaler,
                                         uint32_t *p_etr_filter)
{
  const __IO uint32_t smcr = STM32_READ_REG(timx->SMCR);

  *p_etr_polarity = smcr & TIM_SMCR_ETP;
  *p_etr_prescaler = smcr & (TIM_SMCR_ETPS | TIM_SMCR_SETPS);
  *p_etr_filter = smcr & TIM_SMCR_ETF;
}

/**
  * @brief  Select the external trigger (ETR) input source.
  * @rmtoll
  *  AF1          ETRSEL        LL_TIM_SetETRSource
  * @param  timx Timer instance
  * @param  etr_source This parameter can be one of the following values:
  *
  *         TIM1: one of the following values:
  *            LL_TIM_TIM1_ETR_IN_GPIO:              tim1_etr_in is connected to TIM1_ETR
  *            LL_TIM_TIM1_ETR_IN_COMP1_OUT:         tim1_etr_in is connected to comp1_out
  *            LL_TIM_TIM1_ETR_IN_COMP2_OUT:         tim1_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD1:         tim1_etr_in is connected to adc1_awd1
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD2:         tim1_etr_in is connected to adc1_awd2
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD3:         tim1_etr_in is connected to adc1_awd3
  *            LL_TIM_TIM1_ETR_IN_COMP3_OUT:         tim1_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM1_ETR_IN_COMP4_OUT:         tim1_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM1_ETR_IN_PLAY1_OUT6:        tim1_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM1_ETR_IN_PLAY1_OUT8:        tim1_etr_in is connected to play1_out8 (*)
  *
  *         TIM2: one of the following values:
  *            LL_TIM_TIM2_ETR_IN_GPIO:              tim2_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM2_ETR_IN_COMP1_OUT:         tim2_etr_in is connected to comp1_out
  *            LL_TIM_TIM2_ETR_IN_COMP2_OUT:         tim2_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD1:         tim2_etr_in is connected to adc1_awd1
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD2:         tim2_etr_in is connected to adc1_awd2
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD3:         tim2_etr_in is connected to adc1_awd3
  *            LL_TIM_TIM2_ETR_IN_LSE:               tim2_etr_in is connected to LSE
  *            LL_TIM_TIM2_ETR_IN_MCO1:              tim2_etr_in is connected to MCO1
  *            LL_TIM_TIM2_ETR_IN_TIM3_ETR:          tim2_etr_in is connected to TIM3_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_TIM4_ETR:          tim2_etr_in is connected to TIM4_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_TIM5_ETR:          tim2_etr_in is connected to TIM5_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_ETH1_PTP_PPS_OUT:  tim2_etr_in is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM2_ETR_IN_COMP3_OUT:         tim2_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM2_ETR_IN_COMP4_OUT:         tim2_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM2_ETR_IN_PLAY1_OUT4:        tim2_etr_in is connected to play1_out4 (*)
  *            LL_TIM_TIM2_ETR_IN_PLAY1_OUT2:        tim2_etr_in is connected to play1_out2 (*)
  *
  *         TIM3: one of the following values: (**)
  *            LL_TIM_TIM3_ETR_IN_GPIO:              tim3_etr_in is connected to TIM3_ETR
  *            LL_TIM_TIM3_ETR_IN_COMP1_OUT:         tim3_etr_in is connected to comp1_out
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD1:         tim3_etr_in is connected to adc2_awd1 (*)
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD2:         tim3_etr_in is connected to adc2_awd2 (*)
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD3:         tim3_etr_in is connected to adc2_awd3 (*)
  *            LL_TIM_TIM3_ETR_IN_TIM2_ETR:          tim3_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM3_ETR_IN_TIM4_ETR:          tim3_etr_in is connected to TIM4_ETR
  *            LL_TIM_TIM3_ETR_IN_TIM5_ETR:          tim3_etr_in is connected to TIM5_ETR
  *            LL_TIM_TIM3_ETR_IN_ETH1_PTP_PPS_OUT:  tim3_etr_in is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP2_OUT:         tim3_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP3_OUT:         tim3_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP4_OUT:         tim3_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM3_ETR_IN_PLAY1_OUT4:        tim3_etr_in is connected to play1_out4 (*)
  *            LL_TIM_TIM3_ETR_IN_PLAY1_OUT13:       tim3_etr_in is connected to play1_out13 (*)
  *
  *         TIM4: one of the following values: (**)
  *            LL_TIM_TIM4_ETR_IN_GPIO:              tim4_etr_in is connected to TIM4_ETR
  *            LL_TIM_TIM4_ETR_IN_COMP1_OUT:         tim4_etr_in is connected to comp1_out
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD1:         tim4_etr_in is connected to adc3_awd1
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD2:         tim4_etr_in is connected to adc3_awd2
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD3:         tim4_etr_in is connected to adc3_awd3
  *            LL_TIM_TIM4_ETR_IN_TIM2_ETR:          tim4_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM4_ETR_IN_TIM3_ETR:          tim4_etr_in is connected to TIM3_ETR
  *            LL_TIM_TIM4_ETR_IN_TIM5_ETR:          tim4_etr_in is connected to TIM5_ETR
  *
  *         TIM5: one of the following values: (**)
  *            LL_TIM_TIM5_ETR_IN_GPIO:              tim5_etr_in is connected to TIM5_ETR
  *            LL_TIM_TIM5_ETR_IN_COMP1_OUT:         tim5_etr_in is connected to comp1_out
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD1:         tim5_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD2:         tim5_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD3:         tim5_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM5_ETR_IN_TIM2_ETR:          tim5_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM5_ETR_IN_TIM3_ETR:          tim5_etr_in is connected to TIM3_ETR (*)
  *            LL_TIM_TIM5_ETR_IN_TIM4_ETR:          tim5_etr_in is connected to TIM4_ETR (*)
  *            LL_TIM_TIM5_ETR_IN_COMP2_OUT:         tim5_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM5_ETR_IN_COMP3_OUT:         tim5_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM5_ETR_IN_COMP4_OUT:         tim5_etr_in is connected to comp4_out (*)
  *
  *         TIM8: one of the following values:
  *            LL_TIM_TIM8_ETR_IN_GPIO:              tim8_etr_in is connected to TIM8_ETR
  *            LL_TIM_TIM8_ETR_IN_COMP1_OUT:         tim8_etr_in is connected to comp1_out
  *            LL_TIM_TIM8_ETR_IN_COMP2_OUT:         tim8_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD1:         tim8_etr_in is connected to adc1_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD2:         tim8_etr_in is connected to adc1_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD3:         tim8_etr_in is connected to adc1_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD1:         tim8_etr_in is connected to adc2_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD2:         tim8_etr_in is connected to adc2_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD3:         tim8_etr_in is connected to adc2_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD1:         tim8_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD2:         tim8_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD3:         tim8_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_COMP3_OUT:         tim8_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM8_ETR_IN_COMP4_OUT:         tim8_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM8_ETR_IN_PLAY1_OUT6:        tim8_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM8_ETR_IN_PLAY1_OUT10:       tim8_etr_in is connected to play1_out10 (*)
  *
  *         TIM20: one of the following values: (**)
  *            LL_TIM_TIM20_ETR_IN_GPIO:             tim20_etr_in is connected to TIM20_ETR
  *            LL_TIM_TIM20_ETR_IN_COMP1_OUT:        tim20_etr_in is connected to comp1_out
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD1:        tim20_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD2:        tim20_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD3:        tim20_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM20_ETR_IN_COMP2_OUT:        tim20_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM20_ETR_IN_COMP3_OUT:        tim20_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM20_ETR_IN_COMP4_OUT:        tim20_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM20_ETR_IN_PLAY1_OUT6:       tim20_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM20_ETR_IN_PLAY1_OUT14:      tim20_etr_in is connected to play1_out14 (*)
  *
  *         (*) Value not defined in all devices.
  *         (**) Timer instance not available on all devices.
  * @note Macro IS_TIM_ETRSEL_INSTANCE(timx) can be used to check whether or
  *       not a timer instance supports ETR source selection.
  */
__STATIC_INLINE void LL_TIM_SetETRSource(TIM_TypeDef *timx, uint32_t etr_source)
{
  STM32_MODIFY_REG(timx->AF1, TIM_AF1_ETRSEL, etr_source);
}

/**
  * @brief  Get the source of the external trigger input (ETR).
  * @rmtoll
  *  AF1          ETRSEL        LL_TIM_GetETRSource
  * @param  timx Timer instance
  * @note Macro IS_TIM_ETRSEL_INSTANCE(timx) can be used to check whether or
  *       not a timer instance supports ETR source selection.
  * @retval  ETR source that can be one of the following values:
  *
  *         TIM1: one of the following values:
  *            LL_TIM_TIM1_ETR_IN_GPIO:              tim1_etr_in is connected to TIM1_ETR
  *            LL_TIM_TIM1_ETR_IN_COMP1_OUT:         tim1_etr_in is connected to comp1_out
  *            LL_TIM_TIM1_ETR_IN_COMP2_OUT:         tim1_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD1:         tim1_etr_in is connected to adc1_awd1
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD2:         tim1_etr_in is connected to adc1_awd2
  *            LL_TIM_TIM1_ETR_IN_ADC1_AWD3:         tim1_etr_in is connected to adc1_awd3
  *            LL_TIM_TIM1_ETR_IN_COMP3_OUT:         tim1_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM1_ETR_IN_COMP4_OUT:         tim1_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM1_ETR_IN_PLAY1_OUT6:        tim1_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM1_ETR_IN_PLAY1_OUT8:        tim1_etr_in is connected to play1_out8 (*)
  *
  *         TIM2: one of the following values:
  *            LL_TIM_TIM2_ETR_IN_GPIO:              tim2_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM2_ETR_IN_COMP1_OUT:         tim2_etr_in is connected to comp1_out
  *            LL_TIM_TIM2_ETR_IN_COMP2_OUT:         tim2_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD1:         tim2_etr_in is connected to adc1_awd1
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD2:         tim2_etr_in is connected to adc1_awd2
  *            LL_TIM_TIM2_ETR_IN_ADC1_AWD3:         tim2_etr_in is connected to adc1_awd3
  *            LL_TIM_TIM2_ETR_IN_LSE:               tim2_etr_in is connected to LSE
  *            LL_TIM_TIM2_ETR_IN_MCO1:              tim2_etr_in is connected to MCO1
  *            LL_TIM_TIM2_ETR_IN_TIM3_ETR:          tim2_etr_in is connected to TIM3_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_TIM4_ETR:          tim2_etr_in is connected to TIM4_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_TIM5_ETR:          tim2_etr_in is connected to TIM5_ETR (*)
  *            LL_TIM_TIM2_ETR_IN_ETH1_PTP_PPS_OUT:  tim2_etr_in is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM2_ETR_IN_COMP3_OUT:         tim2_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM2_ETR_IN_COMP4_OUT:         tim2_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM2_ETR_IN_PLAY1_OUT4:        tim2_etr_in is connected to play1_out4 (*)
  *            LL_TIM_TIM2_ETR_IN_PLAY1_OUT2:        tim2_etr_in is connected to play1_out2 (*)
  *
  *         TIM3: one of the following values: (**)
  *            LL_TIM_TIM3_ETR_IN_GPIO:              tim3_etr_in is connected to TIM3_ETR
  *            LL_TIM_TIM3_ETR_IN_COMP1_OUT:         tim3_etr_in is connected to comp1_out
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD1:         tim3_etr_in is connected to adc2_awd1 (*)
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD2:         tim3_etr_in is connected to adc2_awd2 (*)
  *            LL_TIM_TIM3_ETR_IN_ADC2_AWD3:         tim3_etr_in is connected to adc2_awd3 (*)
  *            LL_TIM_TIM3_ETR_IN_TIM2_ETR:          tim3_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM3_ETR_IN_TIM4_ETR:          tim3_etr_in is connected to TIM4_ETR
  *            LL_TIM_TIM3_ETR_IN_TIM5_ETR:          tim3_etr_in is connected to TIM5_ETR
  *            LL_TIM_TIM3_ETR_IN_ETH1_PTP_PPS_OUT:  tim3_etr_in is connected to eth1_ptp_pps_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP2_OUT:         tim3_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP3_OUT:         tim3_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM3_ETR_IN_COMP4_OUT:         tim3_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM3_ETR_IN_PLAY1_OUT4:        tim3_etr_in is connected to play1_out4 (*)
  *            LL_TIM_TIM3_ETR_IN_PLAY1_OUT13:       tim3_etr_in is connected to play1_out13 (*)
  *
  *         TIM4: one of the following values: (**)
  *            LL_TIM_TIM4_ETR_IN_GPIO:              tim4_etr_in is connected to TIM4_ETR
  *            LL_TIM_TIM4_ETR_IN_COMP1_OUT:         tim4_etr_in is connected to comp1_out
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD1:         tim4_etr_in is connected to adc3_awd1
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD2:         tim4_etr_in is connected to adc3_awd2
  *            LL_TIM_TIM4_ETR_IN_ADC3_AWD3:         tim4_etr_in is connected to adc3_awd3
  *            LL_TIM_TIM4_ETR_IN_TIM2_ETR:          tim4_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM4_ETR_IN_TIM3_ETR:          tim4_etr_in is connected to TIM3_ETR
  *            LL_TIM_TIM4_ETR_IN_TIM5_ETR:          tim4_etr_in is connected to TIM5_ETR
  *
  *         TIM5: one of the following values: (**)
  *            LL_TIM_TIM5_ETR_IN_GPIO:              tim5_etr_in is connected to TIM5_ETR
  *            LL_TIM_TIM5_ETR_IN_COMP1_OUT:         tim5_etr_in is connected to comp1_out
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD1:         tim5_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD2:         tim5_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM5_ETR_IN_ADC3_AWD3:         tim5_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM5_ETR_IN_TIM2_ETR:          tim5_etr_in is connected to TIM2_ETR
  *            LL_TIM_TIM5_ETR_IN_TIM3_ETR:          tim5_etr_in is connected to TIM3_ETR (*)
  *            LL_TIM_TIM5_ETR_IN_TIM4_ETR:          tim5_etr_in is connected to TIM4_ETR (*)
  *            LL_TIM_TIM5_ETR_IN_COMP2_OUT:         tim5_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM5_ETR_IN_COMP3_OUT:         tim5_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM5_ETR_IN_COMP4_OUT:         tim5_etr_in is connected to comp4_out (*)
  *
  *         TIM8: one of the following values:
  *            LL_TIM_TIM8_ETR_IN_GPIO:              tim8_etr_in is connected to TIM8_ETR
  *            LL_TIM_TIM8_ETR_IN_COMP1_OUT:         tim8_etr_in is connected to comp1_out
  *            LL_TIM_TIM8_ETR_IN_COMP2_OUT:         tim8_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD1:         tim8_etr_in is connected to adc1_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD2:         tim8_etr_in is connected to adc1_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC1_AWD3:         tim8_etr_in is connected to adc1_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD1:         tim8_etr_in is connected to adc2_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD2:         tim8_etr_in is connected to adc2_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC2_AWD3:         tim8_etr_in is connected to adc2_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD1:         tim8_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD2:         tim8_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM8_ETR_IN_ADC3_AWD3:         tim8_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM8_ETR_IN_COMP3_OUT:         tim8_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM8_ETR_IN_COMP4_OUT:         tim8_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM8_ETR_IN_PLAY1_OUT6:        tim8_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM8_ETR_IN_PLAY1_OUT10:       tim8_etr_in is connected to play1_out10 (*)
  *
  *         TIM20: one of the following values: (**)
  *            LL_TIM_TIM20_ETR_IN_GPIO:             tim20_etr_in is connected to TIM20_ETR
  *            LL_TIM_TIM20_ETR_IN_COMP1_OUT:        tim20_etr_in is connected to comp1_out
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD1:        tim20_etr_in is connected to adc3_awd1 (*)
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD2:        tim20_etr_in is connected to adc3_awd2 (*)
  *            LL_TIM_TIM20_ETR_IN_ADC3_AWD3:        tim20_etr_in is connected to adc3_awd3 (*)
  *            LL_TIM_TIM20_ETR_IN_COMP2_OUT:        tim20_etr_in is connected to comp2_out (*)
  *            LL_TIM_TIM20_ETR_IN_COMP3_OUT:        tim20_etr_in is connected to comp3_out (*)
  *            LL_TIM_TIM20_ETR_IN_COMP4_OUT:        tim20_etr_in is connected to comp4_out (*)
  *            LL_TIM_TIM20_ETR_IN_PLAY1_OUT6:       tim20_etr_in is connected to play1_out6 (*)
  *            LL_TIM_TIM20_ETR_IN_PLAY1_OUT14:      tim20_etr_in is connected to play1_out14 (*)
  *
  *         (*) Value not defined in all devices.
  *         (**) Timer instance not available on all devices.
  */
__STATIC_INLINE uint32_t LL_TIM_GetETRSource(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_REG(timx->AF1) & TIM_AF1_ETRSEL);
}

/**
  * @brief  Enable SMS preload.
  * @rmtoll
  *  SMCR         SMSPE           LL_TIM_EnableSMSPreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_SMS_PRELOAD_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the preload of SMS field in SMCR register.
  */
__STATIC_INLINE void LL_TIM_EnableSMSPreload(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->SMCR, TIM_SMCR_SMSPE);
}

/**
  * @brief  Disable SMS preload.
  * @rmtoll
  *  SMCR         SMSPE           LL_TIM_DisableSMSPreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_SMS_PRELOAD_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the preload of SMS field in SMCR register.
  */
__STATIC_INLINE void LL_TIM_DisableSMSPreload(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->SMCR, TIM_SMCR_SMSPE);
}

/**
  * @brief  Indicate whether  SMS preload is enabled.
  * @rmtoll
  *  SMCR         SMSPE           LL_TIM_IsEnabledSMSPreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_SMS_PRELOAD_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the preload of SMS field in SMCR register.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledSMSPreload(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SMCR, TIM_SMCR_SMSPE) == (TIM_SMCR_SMSPE)) ? 1UL : 0UL);
}

/**
  * @brief  Set the preload source of SMS.
  * @rmtoll
  *  SMCR         SMSPS        LL_TIM_SetSMSPreloadSource
  * @param  timx Timer instance
  * @param  preload_source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_SLAVE_MODE_PRELOAD_UPDATE
  *         @arg @ref LL_TIM_SLAVE_MODE_PRELOAD_INDEX
  * @note Macro IS_TIM_SMS_PRELOAD_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the preload of SMS field in SMCR register.
  */
__STATIC_INLINE void LL_TIM_SetSMSPreloadSource(TIM_TypeDef *timx, uint32_t preload_source)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_SMSPS, preload_source);
}

/**
  * @brief  Get the preload source of SMS.
  * @rmtoll
  *  SMCR         SMSPS        LL_TIM_GetSMSPreloadSource
  * @param  timx Timer instance
  * @note Macro IS_TIM_SMS_PRELOAD_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports the preload of SMS field in SMCR register.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_SLAVE_MODE_PRELOAD_UPDATE
  *         @arg @ref LL_TIM_SLAVE_MODE_PRELOAD_INDEX
  */
__STATIC_INLINE uint32_t LL_TIM_GetSMSPreloadSource(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->SMCR, TIM_SMCR_SMSPS));
}


/**
  * @brief  Enable ADC synchronization.
  * @rmtoll
  *  CR2         ADSYNC           LL_TIM_EnableADCSynchronization
  * @param  timx Timer instance
  * @note Macro IS_TIM_MASTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports ADC synchronization.
  */
__STATIC_INLINE void LL_TIM_EnableADCSynchronization(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->CR2, TIM_CR2_ADSYNC);
}

/**
  * @brief  Enable ADC synchronization.
  * @rmtoll
  *  CR2         ADSYNC           LL_TIM_DisableADCSynchronization
  * @param  timx Timer instance
  * @note Macro IS_TIM_MASTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports ADC synchronization.
  */
__STATIC_INLINE void LL_TIM_DisableADCSynchronization(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->CR2, TIM_CR2_ADSYNC);
}

/**
  * @brief  Indicate whether ADC sycnhronization is enabled.
  * @rmtoll
  *  CR2         ADSYNC           LL_TIM_IsEnabledADCSynchronization
  * @param  timx Timer instance
  * @note Macro IS_TIM_MASTER_INSTANCE(timx) can be used to check
  *       whether or not a timer instance supports ADC synchronization.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledADCSynchronization(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->CR2, TIM_CR2_ADSYNC) == (TIM_CR2_ADSYNC)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_Break_Function Break function configuration
  * @{
  */
/**
  * @brief  Enable the break function.
  * @rmtoll
  *  BDTR         BKE           LL_TIM_EnableBRK
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_EnableBRK(TIM_TypeDef *timx)
{
  __IO uint32_t tmpreg;
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_BKE);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Disable the break function.
  * @rmtoll
  *  BDTR         BKE           LL_TIM_DisableBRK
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_DisableBRK(TIM_TypeDef *timx)
{
  __IO uint32_t tmpreg;
  STM32_CLEAR_BIT(timx->BDTR, TIM_BDTR_BKE);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Configure the break input.
  * @rmtoll
  *  BDTR         BKP           LL_TIM_ConfigBRK \n
  *  BDTR         BKF           LL_TIM_ConfigBRK \n
  *  BDTR         BKBID         LL_TIM_ConfigBRK
  * @param  timx Timer instance
  * @param  break_polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH
  * @param  break_filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8
  * @param  break_afmode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK_AFMODE_BIDIRECTIONAL
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @note Bidirectional mode is only supported by advanced timer instances.
  * @note In bidirectional mode (BKBID bit set), the Break input is configured both
  *        in input mode and in open drain output mode. Any active Break event will
  *        assert a low logic level on the Break input to indicate an internal break
  *        event to external devices.
  * @note When bidirectional mode isn't supported, break_afmode must be set to
  *       LL_TIM_BREAK_AFMODE_INPUT.
  */
__STATIC_INLINE void LL_TIM_ConfigBRK(TIM_TypeDef *timx, uint32_t break_polarity, uint32_t break_filter,
                                      uint32_t break_afmode)
{
  __IO uint32_t tmpreg;
  STM32_MODIFY_REG(timx->BDTR, TIM_BDTR_BKP | TIM_BDTR_BKF | TIM_BDTR_BKBID, \
                   break_polarity | break_filter | break_afmode);
  /* Note: Any write operation to BKP bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Get the break input configuration.
  * @rmtoll
  *  BDTR         BKP           LL_TIM_GetConfigBRK \n
  *  BDTR         BKF           LL_TIM_GetConfigBRK \n
  *  BDTR         BKBID         LL_TIM_GetConfigBRK
  * @param  timx Timer instance
  * @param  p_break_polarity Pointer to a storage for break polarity. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH
  * @param  p_break_filter  Pointer to a storage for break filter. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8
  * @param  p_break_afmode Pointer to a storage for break afmode. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK_AFMODE_BIDIRECTIONAL
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_GetConfigBRK(TIM_TypeDef *timx, uint32_t *p_break_polarity, uint32_t *p_break_filter,
                                         uint32_t *p_break_afmode)
{
  const __IO uint32_t bdtr = STM32_READ_REG(timx->BDTR);

  *p_break_polarity = bdtr & TIM_BDTR_BKP;
  *p_break_filter = bdtr & TIM_BDTR_BKF;
  *p_break_afmode = bdtr & TIM_BDTR_BKBID;
}

/**
  * @brief  Disarm the break input (when it operates in bidirectional mode).
  * @rmtoll
  *  BDTR         BKDSRM        LL_TIM_DisarmBRK
  * @param  timx Timer instance
  * @note  The break input can be disarmed only when it is configured in
  *        bidirectional mode and when when MOE is reset.
  * @note  Purpose is to be able to have the input voltage back to high-state,
  *        whatever the time constant on the output .
  */
__STATIC_INLINE void LL_TIM_DisarmBRK(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_BKDSRM);
}

/**
  * @brief  Indicates whether the break input is disarmed.
  * @rmtoll
  *  BDTR         BKDSRM        LL_TIM_IsDisarmedBRK
  * @param  timx Timer instance
  * @retval Status of the break input (0: armed, 1: disarmed)
  */
__STATIC_INLINE uint32_t LL_TIM_IsDisarmedBRK(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->BDTR, TIM_BDTR_BKDSRM) == (TIM_BDTR_BKDSRM)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the break 2 function.
  * @rmtoll
  *  BDTR         BK2E          LL_TIM_EnableBRK2
  * @param  timx Timer instance
  * @note Macro IS_TIM_BKIN2_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a second break input.
  */
__STATIC_INLINE void LL_TIM_EnableBRK2(TIM_TypeDef *timx)
{
  __IO uint32_t tmpreg;
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_BK2E);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Disable the break  2 function.
  * @rmtoll
  *  BDTR         BK2E          LL_TIM_DisableBRK2
  * @param  timx Timer instance
  * @note Macro IS_TIM_BKIN2_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a second break input.
  */
__STATIC_INLINE void LL_TIM_DisableBRK2(TIM_TypeDef *timx)
{
  __IO uint32_t tmpreg;
  STM32_CLEAR_BIT(timx->BDTR, TIM_BDTR_BK2E);
  /* Note: Any write operation to this bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Configure the break 2 input.
  * @rmtoll
  *  BDTR         BK2P          LL_TIM_ConfigBRK2 \n
  *  BDTR         BK2F          LL_TIM_ConfigBRK2 \n
  *  BDTR         BK2BID        LL_TIM_ConfigBRK2
  * @param  timx Timer instance
  * @param  break2_polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK2_POLARITY_HIGH
  * @param  break2_filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  * @param  break2_afmode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL
  * @note Macro IS_TIM_BKIN2_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a second break input.
  * @note Bidirectional mode is only supported by advanced timer instances.
  * @note In bidirectional mode (BK2BID bit set), the Break 2 input is configured both
  *        in input mode and in open drain output mode. Any active Break event will
  *        assert a low logic level on the Break 2 input to indicate an internal break
  *        event to external devices.
  * @note When bidirectional mode isn't supported, break2_afmode must be set to
  *       LL_TIM_BREAK2_AFMODE_INPUT.
  */
__STATIC_INLINE void LL_TIM_ConfigBRK2(TIM_TypeDef *timx, uint32_t break2_polarity, uint32_t break2_filter,
                                       uint32_t break2_afmode)
{
  __IO uint32_t tmpreg;
  STM32_MODIFY_REG(timx->BDTR, TIM_BDTR_BK2P | TIM_BDTR_BK2F | TIM_BDTR_BK2BID,
                   break2_polarity | break2_filter | break2_afmode);
  /* Note: Any write operation to BK2P bit takes a delay of 1 APB clock cycle to become effective. */
  tmpreg = STM32_READ_REG(timx->BDTR);
  (void)(tmpreg);
}

/**
  * @brief  Get the break 2 input configuration.
  * @rmtoll
  *  BDTR         BK2P           LL_TIM_GetConfigBRK2 \n
  *  BDTR         BK2F           LL_TIM_GetConfigBRK2 \n
  *  BDTR         BK2BID         LL_TIM_GetConfigBRK2
  * @param  timx Timer instance
  * @param  p_break2_polarity Pointer to a storage for break 2 polarity. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK2_POLARITY_HIGH
  * @param  p_break2_filter  Pointer to a storage for break 2 filter. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  * @param  p_break2_afmode Pointer to a storage for break 2 afmode. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK2_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL
  * @note Macro IS_TIM_BKIN2_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a second break input.
  */
__STATIC_INLINE void LL_TIM_GetConfigBRK2(TIM_TypeDef *timx, uint32_t *p_break2_polarity, uint32_t *p_break2_filter,
                                          uint32_t *p_break2_afmode)
{
  const __IO uint32_t bdtr = STM32_READ_REG(timx->BDTR);

  *p_break2_polarity = bdtr & TIM_BDTR_BK2P;
  *p_break2_filter = bdtr & TIM_BDTR_BK2F;
  *p_break2_afmode = bdtr & TIM_BDTR_BK2BID;
}

/**
  * @brief  Disarm the break 2 input (when it operates in bidirectional mode).
  * @rmtoll
  *  BDTR         BK2DSRM       LL_TIM_DisarmBRK2
  * @param  timx Timer instance
  * @note  The break 2 input can be disarmed only when it is configured in
  *        bidirectional mode and when when MOE is reset.
  * @note  Purpose is to be able to have the input voltage back to high-state,
  *        whatever the time constant on the output.
  */
__STATIC_INLINE void LL_TIM_DisarmBRK2(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_BK2DSRM);
}

/**
  * @brief  Indicates whether the break input 2 is disarmed.
  * @rmtoll
  *  BDTR         BK2DSRM       LL_TIM_IsDisarmedBRK2
  * @param  timx Timer instance
  * @retval Status of the break input 2 (0: armed, 1: disarmed)
  */
__STATIC_INLINE uint32_t LL_TIM_IsDisarmedBRK2(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->BDTR, TIM_BDTR_BK2DSRM) == (TIM_BDTR_BK2DSRM)) ? 1UL : 0UL);
}

/**
  * @brief  Disarm the break input.
  * @rmtoll
  *  BDTR         BKDSRM        LL_TIM_DisarmBreakInput \n
  *  BDTR         BK2DSRM       LL_TIM_DisarmBreakInput
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @note  The break input can be disarmed only when it is configured in
  *        bidirectional mode and when when MOE is reset.
  */
__STATIC_INLINE void LL_TIM_DisarmBreakInput(TIM_TypeDef *timx, uint32_t break_input)
{
  STM32_SET_BIT(timx->BDTR, (TIM_BDTR_BKDSRM << break_input));
}

/**
  * @brief  Indicates whether the break input 2 is disarmed.
  * @rmtoll
  *  BDTR         BKDSRM        LL_TIM_IsDisarmedBreakInput \n
  *  BDTR         BK2DSRM       LL_TIM_IsDisarmedBreakInput
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @retval Status of the break input 2 (0: armed, 1: disarmed)
  */
__STATIC_INLINE uint32_t LL_TIM_IsDisarmedBreakInput(const TIM_TypeDef *timx, uint32_t break_input)
{
  uint32_t disarm_bit = (TIM_BDTR_BKDSRM << break_input);
  return ((STM32_READ_BIT(timx->BDTR, disarm_bit) == (disarm_bit)) ? 1UL : 0UL);
}

/**
  * @brief  Select the outputs off state (enabled v.s. disabled) in Idle and Run modes.
  * @rmtoll
  *  BDTR         OSSI          LL_TIM_SetOffStates \n
  *  BDTR         OSSR          LL_TIM_SetOffStates
  * @param  timx Timer instance
  * @param  offstate_idle This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSI_DISABLE
  *         @arg @ref LL_TIM_OSSI_ENABLE
  * @param  offstate_run This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSR_DISABLE
  *         @arg @ref LL_TIM_OSSR_ENABLE
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_SetOffStates(TIM_TypeDef *timx, uint32_t offstate_idle, uint32_t offstate_run)
{
  STM32_MODIFY_REG(timx->BDTR, TIM_BDTR_OSSI | TIM_BDTR_OSSR, offstate_idle | offstate_run);
}

/**
  * @brief  Get actual outputs off state (enabled v.s. disabled) in Idle and Run modes.
  * @rmtoll
  *  BDTR         OSSI          LL_TIM_GetOffStates \n
  *  BDTR         OSSR          LL_TIM_GetOffStates
  * @param  timx Timer instance
  * @param  offstate_idle This parameter can store one of the following values:
  *         @arg @ref LL_TIM_OSSI_DISABLE
  *         @arg @ref LL_TIM_OSSI_ENABLE
  * @param  offstate_run This parameter can store of the following values:
  *         @arg @ref LL_TIM_OSSR_DISABLE
  *         @arg @ref LL_TIM_OSSR_ENABLE
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_GetOffStates(const TIM_TypeDef *timx, uint32_t *offstate_idle, uint32_t *offstate_run)
{
  const __IO uint32_t reg = STM32_READ_REG(timx->BDTR);
  *offstate_idle = reg & TIM_BDTR_OSSI;
  *offstate_run = reg & TIM_BDTR_OSSR;
}

/**
  * @brief  Indicate the global output state when a break or break2 event occurred, to discriminate the source.
  * @rmtoll
  *  SR           ODS            LL_TIM_GetOutputDisableStatus
  * @param  timx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OUTPUT_IDLE_STATE
  *         @arg @ref LL_TIM_OUTPUT_DISABLED_STATE
  */
__STATIC_INLINE uint32_t LL_TIM_GetOutputDisableStatus(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->SR, TIM_SR_ODS));
}

/**
  * @brief  Enable automatic output (MOE can be set by software or automatically when a break input is active).
  * @rmtoll
  *  BDTR         AOE           LL_TIM_EnableAutomaticOutput
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_EnableAutomaticOutput(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_AOE);
}

/**
  * @brief  Disable automatic output (MOE can be set only by software).
  * @rmtoll
  *  BDTR         AOE           LL_TIM_DisableAutomaticOutput
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_DisableAutomaticOutput(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->BDTR, TIM_BDTR_AOE);
}

/**
  * @brief  Indicate whether automatic output is enabled.
  * @rmtoll
  *  BDTR         AOE           LL_TIM_IsEnabledAutomaticOutput
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledAutomaticOutput(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->BDTR, TIM_BDTR_AOE) == (TIM_BDTR_AOE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the outputs (set the MOE bit in TIMx_BDTR register).
  * @rmtoll
  *  BDTR         MOE           LL_TIM_EnableAllOutputs
  * @param  timx Timer instance
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_EnableAllOutputs(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->BDTR, TIM_BDTR_MOE);
}

/**
  * @brief  Disable the outputs (reset the MOE bit in TIMx_BDTR register).
  * @rmtoll
  *  BDTR         MOE           LL_TIM_DisableAllOutputs
  * @param  timx Timer instance
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event.
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  */
__STATIC_INLINE void LL_TIM_DisableAllOutputs(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->BDTR, TIM_BDTR_MOE);
}

/**
  * @brief  Indicates whether outputs are enabled.
  * @rmtoll
  *  BDTR         MOE           LL_TIM_IsEnabledAllOutputs
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledAllOutputs(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->BDTR, TIM_BDTR_MOE) == (TIM_BDTR_MOE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable a break input.
  * @rmtoll
  *  BDTR         BKE     LL_TIM_EnableBreakInput \n
  *  BDTR         BK2E    LL_TIM_EnableBreakInput
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) (IS_TIM_BKIN2_INSTANCE(timx))
  *       can be used to check whether or not a timer instance provides
  *       a break input (resp. a break2 input).
  */
__STATIC_INLINE void LL_TIM_EnableBreakInput(TIM_TypeDef *timx, uint32_t break_input)
{
  STM32_SET_BIT(timx->BDTR, LL_TIM_MASK_TAB_BKxE[break_input]);
}

/**
  * @brief  Disable a break input.
  * @rmtoll
  *  BDTR         BKE     LL_TIM_DisableBreakInput \n
  *  BDTR         BK2E    LL_TIM_DisableBreakInput
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) (IS_TIM_BKIN2_INSTANCE(timx))
  *       can be used to check whether or not a timer instance provides
  *       a break input (resp. a break2 input).
  */
__STATIC_INLINE void LL_TIM_DisableBreakInput(TIM_TypeDef *timx, uint32_t break_input)
{
  STM32_CLEAR_BIT(timx->BDTR, LL_TIM_MASK_TAB_BKxE[break_input]);
}

/**
  * @brief  Indicates whether the input is enabled or not.
  * @rmtoll
  *  BDTR         BKE     LL_TIM_IsEnabledBreakInput \n
  *  BDTR         BK2E    LL_TIM_IsEnabledBreakInput
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) (IS_TIM_BKIN2_INSTANCE(timx))
  *       can be used to check whether or not a timer instance provides
  *       a break input (resp. a break2 input).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledBreakInput(const TIM_TypeDef *timx, uint32_t break_input)
{
  uint32_t bitfield = LL_TIM_MASK_TAB_BKxE[break_input];
  return ((STM32_READ_BIT(timx->BDTR, bitfield) == bitfield) ? 1UL : 0UL);
}

/**
  * @brief  Set the polarity of a break input.
  * @rmtoll
  *  BDTR         BKP     LL_TIM_SetBreakInputPolarity \n
  *  BDTR         BK2P    LL_TIM_SetBreakInputPolarity
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  break_polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW  or LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH or LL_TIM_BREAK2_POLARITY_HIGH
  */
__STATIC_INLINE void LL_TIM_SetBreakInputPolarity(TIM_TypeDef *timx, uint32_t break_input, uint32_t break_polarity)
{
  STM32_MODIFY_REG(timx->BDTR, LL_TIM_MASK_TAB_BKxP[break_input], break_polarity);
}

/**
  * @brief  Get the polarity of a break input.
  * @rmtoll
  *  BDTR         BKP     LL_TIM_GetBreakInputPolarity \n
  *  BDTR         BK2P    LL_TIM_GetBreakInputPolarity
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW  or @arg @ref LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH or @arg @ref LL_TIM_BREAK2_POLARITY_HIGH
  */
__STATIC_INLINE uint32_t LL_TIM_GetBreakInputPolarity(const TIM_TypeDef *timx, uint32_t break_input)
{
  return (uint32_t)(STM32_READ_BIT(timx->BDTR, LL_TIM_MASK_TAB_BKxP[break_input]));
}

/**
  * @brief  Set the digital filter of a break input.
  * @rmtoll
  *  BDTR         BKF     LL_TIM_SetBreakInputFilter \n
  *  BDTR         BK2F    LL_TIM_SetBreakInputFilter
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  break_filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1     or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  */
__STATIC_INLINE void LL_TIM_SetBreakInputFilter(TIM_TypeDef *timx, uint32_t break_input, uint32_t break_filter)
{
  STM32_MODIFY_REG(timx->BDTR, LL_TIM_MASK_TAB_BKxF[break_input], break_filter);
}

/**
  * @brief  Get the digital filter of a break input.
  * @rmtoll
  *  BDTR         BKF     LL_TIM_GetBreakInputFilter \n
  *  BDTR         BK2F    LL_TIM_GetBreakInputFilter
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @retval  Returned value can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1     or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8  or @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8 or @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  */
__STATIC_INLINE uint32_t LL_TIM_GetBreakInputFilter(const TIM_TypeDef *timx, uint32_t break_input)
{
  return (uint32_t)(STM32_READ_BIT(timx->BDTR, LL_TIM_MASK_TAB_BKxF[break_input]));
}

/**
  * @brief  Set the mode of a break input.
  * @rmtoll
  *  BDTR         BKBID     LL_TIM_SetBreakInputAFMode \n
  *  BDTR         BK2BID    LL_TIM_SetBreakInputAFMode
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  break_afmode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_AFMODE_INPUT         or @arg @ref LL_TIM_BREAK2_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK_AFMODE_BIDIRECTIONAL or @arg @ref LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL
  */
__STATIC_INLINE void LL_TIM_SetBreakInputAFMode(TIM_TypeDef *timx, uint32_t break_input, uint32_t break_afmode)
{
  STM32_MODIFY_REG(timx->BDTR, LL_TIM_MASK_TAB_BKxBID[break_input], break_afmode);
}

/**
  * @brief  Get the mode of a break input.
  * @rmtoll
  *  BDTR         BKBID     LL_TIM_SetBreakInputAFMode \n
  *  BDTR         BK2BID    LL_TIM_SetBreakInputAFMode
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @retval  Returned value can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_AFMODE_INPUT         or @arg @ref LL_TIM_BREAK2_AFMODE_INPUT
  *         @arg @ref LL_TIM_BREAK_AFMODE_BIDIRECTIONAL or @arg @ref LL_TIM_BREAK2_AFMODE_BIDIRECTIONAL
  */
__STATIC_INLINE uint32_t LL_TIM_GetBreakInputAFMode(const TIM_TypeDef *timx, uint32_t break_input)
{
  return (uint32_t)(STM32_READ_BIT(timx->BDTR, LL_TIM_MASK_TAB_BKxBID[break_input]));
}

/**
  * @brief  Enable the signals connected to the designated timer break input.
  * @rmtoll
  *  AF1          BKINE         LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP1E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP2E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP3E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP4E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP5E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP6E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP8E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP9E       LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP10E      LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP11E      LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP13E      LL_TIM_EnableBreakInputSource \n
  *  AF1          BKCMP14E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2INE        LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP1E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP2E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP3E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP4E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP5E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP6E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP7E      LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP13E     LL_TIM_EnableBreakInputSource \n
  *  AF2          BK2CMP14E     LL_TIM_EnableBreakInputSource
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  source The break and break2 input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *
  *         The description below summarizes specific "Timer Instance" and "BREAK(2) input source"
  *         parameter possibilities:
  *
  *         TIM1: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM1_BRK_GPIO
  *            LL_TIM_TIM1_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK_TIM8_BKIN
  *            LL_TIM_TIM1_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM1_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM1_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM1_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM1_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM1_BRK_PLAY1_OUT1 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM1_BRK2_GPIO
  *            LL_TIM_TIM1_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK2_TIM8_BKIN2
  *            LL_TIM_TIM1_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT3 (*)
  * @endif
  * @endif
  *
  *         TIM8: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM8_BRK_GPIO
  *            LL_TIM_TIM8_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK_TIM1_BKIN
  *            LL_TIM_TIM8_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM8_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM8_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM8_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM8_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM8_BRK_PLAY1_OUT13 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM8_BRK2_GPIO
  *            LL_TIM_TIM8_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM8_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT15 (*)
  * @endif
  * @endif
  *
  *         TIM15: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM15_BRK_GPIO
  *            LL_TIM_TIM15_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM15_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM15_BRK_TIM1_BKIN
  *            LL_TIM_TIM15_BRK_TIM8_BKIN
  * @if TIM16
  *            LL_TIM_TIM15_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM15_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM15_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM15_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM15_BRK_PLAY1_OUT5 (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM16_BRK_GPIO
  *            LL_TIM_TIM16_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM16_BRK_TIM1_BKIN
  *            LL_TIM_TIM16_BRK_TIM8_BKIN
  *            LL_TIM_TIM16_BRK_TIM15_BKIN
  *            LL_TIM_TIM16_BRK_TIM17_BKIN
  * @if TIM20
  *            LL_TIM_TIM16_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM16_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM16_BRK_PLAY1_OUT7 (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM17_BRK_GPIO
  *            LL_TIM_TIM17_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM17_BRK_TIM1_BKIN
  *            LL_TIM_TIM17_BRK_TIM8_BKIN
  *            LL_TIM_TIM17_BRK_TIM15_BKIN
  *            LL_TIM_TIM17_BRK_TIM16_BKIN
  * @if TIM20
  *            LL_TIM_TIM17_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM17_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM17_BRK_PLAY1_OUT12 (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM20_BRK_GPIO
  *            LL_TIM_TIM20_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK_TIM1_BKIN
  *            LL_TIM_TIM20_BRK_TIM8_BKIN
  *            LL_TIM_TIM20_BRK_TIM15_BKIN
  *            LL_TIM_TIM20_BRK_TIM16_BKIN
  *            LL_TIM_TIM20_BRK_TIM17_BKIN
  * @if PLAY1
  *            LL_TIM_TIM20_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM20_BRK_PLAY1_OUT9 (*)
  * @endif
  *
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM20_BRK2_GPIO
  *            LL_TIM_TIM20_BRK2_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK2_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM20_BRK2_TIM8_BKIN2
  * @if PLAY1
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT11 (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.


  */
__STATIC_INLINE void LL_TIM_EnableBreakInputSource(TIM_TypeDef *timx, uint32_t break_input, uint32_t source)
{
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->AF1) + (break_input << 2)));
  STM32_SET_BIT(*pReg, source);
}

/**
  * @brief  Disable the signals connected to the designated timer break input.
  * @rmtoll
  *  AF1          BKINE         LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP1E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP2E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP3E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP4E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP5E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP6E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP8E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP9E       LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP10E      LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP11E      LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP13E      LL_TIM_DisableBreakInputSource \n
  *  AF1          BKCMP14E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2INE        LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP1E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP2E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP3E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP4E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP5E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP6E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP7E      LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP13E     LL_TIM_DisableBreakInputSource \n
  *  AF2          BK2CMP14E     LL_TIM_DisableBreakInputSource
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  source The break and break2 input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *
  *         The description below summarizes specific "Timer Instance" and "BREAK(2) input source"
  *         parameter possibilities:
  *
  *         TIM1: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM1_BRK_GPIO
  *            LL_TIM_TIM1_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK_TIM8_BKIN
  *            LL_TIM_TIM1_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM1_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM1_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM1_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM1_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM1_BRK_PLAY1_OUT1 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM1_BRK2_GPIO
  *            LL_TIM_TIM1_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK2_TIM8_BKIN2
  *            LL_TIM_TIM1_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT3 (*)
  * @endif
  * @endif
  *
  *         TIM8: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM8_BRK_GPIO
  *            LL_TIM_TIM8_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK_TIM1_BKIN
  *            LL_TIM_TIM8_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM8_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM8_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM8_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM8_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM8_BRK_PLAY1_OUT13 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM8_BRK2_GPIO
  *            LL_TIM_TIM8_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM8_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT15 (*)
  * @endif
  * @endif
  *
  *         TIM15: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM15_BRK_GPIO
  *            LL_TIM_TIM15_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM15_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM15_BRK_TIM1_BKIN
  *            LL_TIM_TIM15_BRK_TIM8_BKIN
  * @if TIM16
  *            LL_TIM_TIM15_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM15_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM15_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM15_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM15_BRK_PLAY1_OUT5 (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM16_BRK_GPIO
  *            LL_TIM_TIM16_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM16_BRK_TIM1_BKIN
  *            LL_TIM_TIM16_BRK_TIM8_BKIN
  *            LL_TIM_TIM16_BRK_TIM15_BKIN
  *            LL_TIM_TIM16_BRK_TIM17_BKIN
  * @if TIM20
  *            LL_TIM_TIM16_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM16_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM16_BRK_PLAY1_OUT7 (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM17_BRK_GPIO
  *            LL_TIM_TIM17_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM17_BRK_TIM1_BKIN
  *            LL_TIM_TIM17_BRK_TIM8_BKIN
  *            LL_TIM_TIM17_BRK_TIM15_BKIN
  *            LL_TIM_TIM17_BRK_TIM16_BKIN
  * @if TIM20
  *            LL_TIM_TIM17_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM17_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM17_BRK_PLAY1_OUT12 (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM20_BRK_GPIO
  *            LL_TIM_TIM20_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK_TIM1_BKIN
  *            LL_TIM_TIM20_BRK_TIM8_BKIN
  *            LL_TIM_TIM20_BRK_TIM15_BKIN
  *            LL_TIM_TIM20_BRK_TIM16_BKIN
  *            LL_TIM_TIM20_BRK_TIM17_BKIN
  * @if PLAY1
  *            LL_TIM_TIM20_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM20_BRK_PLAY1_OUT9 (*)
  * @endif
  *
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM20_BRK2_GPIO
  *            LL_TIM_TIM20_BRK2_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK2_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM20_BRK2_TIM8_BKIN2
  * @if PLAY1
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT11 (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.


  */
__STATIC_INLINE void LL_TIM_DisableBreakInputSource(TIM_TypeDef *timx, uint32_t break_input, uint32_t source)
{
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->AF1) + (break_input << 2)));
  STM32_CLEAR_BIT(*pReg, source);
}

/**
  * @brief  Indicates whether a break input source is enabled or not.
  * @rmtoll
  *  AF1          BKINE         LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP1E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP2E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP3E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP4E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP5E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP6E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP8E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP9E       LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP10E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP11E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP13E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF1          BKCMP14E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2INE        LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP1E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP2E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP3E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP4E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP5E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP6E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP7E      LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP13E     LL_TIM_IsEnabledBreakInputSource \n
  *  AF2          BK2CMP14E     LL_TIM_IsEnabledBreakInputSource
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  source The break and break2 input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *
  *         The description below summarizes specific "Timer Instance" and "BREAK(2) input source"
  *         parameter possibilities:
  *
  *         TIM1: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM1_BRK_GPIO
  *            LL_TIM_TIM1_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK_TIM8_BKIN
  *            LL_TIM_TIM1_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM1_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM1_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM1_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM1_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM1_BRK_PLAY1_OUT1 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM1_BRK2_GPIO
  *            LL_TIM_TIM1_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM1_BRK2_TIM8_BKIN2
  *            LL_TIM_TIM1_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM1_BRK2_PLAY1_OUT3 (*)
  * @endif
  * @endif
  *
  *         TIM8: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM8_BRK_GPIO
  *            LL_TIM_TIM8_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK_TIM1_BKIN
  *            LL_TIM_TIM8_BRK_TIM15_BKIN
  * @if TIM16
  *            LL_TIM_TIM8_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM8_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM8_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM8_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM8_BRK_PLAY1_OUT13 (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM8_BRK2_GPIO
  *            LL_TIM_TIM8_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM8_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM8_BRK2_TIM20_BKIN2 (*)
  * @if PLAY1
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM8_BRK2_PLAY1_OUT15 (*)
  * @endif
  * @endif
  *
  *         TIM15: combination of the following values:
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM15_BRK_GPIO
  *            LL_TIM_TIM15_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM15_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM15_BRK_TIM1_BKIN
  *            LL_TIM_TIM15_BRK_TIM8_BKIN
  * @if TIM16
  *            LL_TIM_TIM15_BRK_TIM16_BKIN (*)
  *            LL_TIM_TIM15_BRK_TIM17_BKIN (*)
  * @endif
  * @if TIM20
  *            LL_TIM_TIM15_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM15_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM15_BRK_PLAY1_OUT5 (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM16_BRK_GPIO
  *            LL_TIM_TIM16_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM16_BRK_TIM1_BKIN
  *            LL_TIM_TIM16_BRK_TIM8_BKIN
  *            LL_TIM_TIM16_BRK_TIM15_BKIN
  *            LL_TIM_TIM16_BRK_TIM17_BKIN
  * @if TIM20
  *            LL_TIM_TIM16_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM16_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM16_BRK_PLAY1_OUT7 (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM17_BRK_GPIO
  *            LL_TIM_TIM17_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM17_BRK_TIM1_BKIN
  *            LL_TIM_TIM17_BRK_TIM8_BKIN
  *            LL_TIM_TIM17_BRK_TIM15_BKIN
  *            LL_TIM_TIM17_BRK_TIM16_BKIN
  * @if TIM20
  *            LL_TIM_TIM17_BRK_TIM20_BKIN (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM17_BRK_PLAY1_OUT4 (*)
  *            LL_TIM_TIM17_BRK_PLAY1_OUT12 (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: combination of the following values: (**)
  *
  *            . . BREAK can be a combination of the following values
  *            LL_TIM_TIM20_BRK_GPIO
  *            LL_TIM_TIM20_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK_TIM1_BKIN
  *            LL_TIM_TIM20_BRK_TIM8_BKIN
  *            LL_TIM_TIM20_BRK_TIM15_BKIN
  *            LL_TIM_TIM20_BRK_TIM16_BKIN
  *            LL_TIM_TIM20_BRK_TIM17_BKIN
  * @if PLAY1
  *            LL_TIM_TIM20_BRK_PLAY1_OUT0 (*)
  *            LL_TIM_TIM20_BRK_PLAY1_OUT9 (*)
  * @endif
  *
  *            . . BREAK2 can be a combination of the following values
  *            LL_TIM_TIM20_BRK2_GPIO
  *            LL_TIM_TIM20_BRK2_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK2_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP4_OUT (*)
  * @endif
  *            LL_TIM_TIM20_BRK2_TIM1_BKIN2
  *            LL_TIM_TIM20_BRK2_TIM8_BKIN2
  * @if PLAY1
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT2 (*)
  *            LL_TIM_TIM20_BRK2_PLAY1_OUT11 (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.


  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledBreakInputSource(const TIM_TypeDef *timx, uint32_t break_input,
                                                          uint32_t source)
{
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->AF1) + (break_input << 2)));
  return ((STM32_READ_BIT(*pReg, source) == (source)) ? 1UL : 0UL);
}

/**
  * @brief  Set the polarity of the break signal for the timer break input.
  * @rmtoll
  *  AF1          BKINP         LL_TIM_SetBreakInputSourcePolarity \n
  *  AF1          BKCMP1P       LL_TIM_SetBreakInputSourcePolarity \n
  *  AF1          BKCMP2P       LL_TIM_SetBreakInputSourcePolarity \n
  *  AF1          BKCMP3P       LL_TIM_SetBreakInputSourcePolarity \n
  *  AF1          BKCMP4P       LL_TIM_SetBreakInputSourcePolarity \n
  *  AF2          BK2INP        LL_TIM_SetBreakInputSourcePolarity \n
  *  AF2          BK2CMP1P      LL_TIM_SetBreakInputSourcePolarity \n
  *  AF2          BK2CMP2P      LL_TIM_SetBreakInputSourcePolarity \n
  *  AF2          BK2CMP3P      LL_TIM_SetBreakInputSourcePolarity \n
  *  AF2          BK2CMP4P      LL_TIM_SetBreakInputSourcePolarity
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  source The break and break2 input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *
  *         The description below summarizes specific "Timer Instance" and "BREAK(2) input source"
  *         parameter possibilities:
  *
  *         TIM1: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM1_BRK_GPIO
  *            LL_TIM_TIM1_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM1_BRK2_GPIO
  *            LL_TIM_TIM1_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM8: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM8_BRK_GPIO
  *            LL_TIM_TIM8_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM8_BRK2_GPIO
  *            LL_TIM_TIM8_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM15: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM15_BRK_GPIO
  *            LL_TIM_TIM15_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM15_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM16_BRK_GPIO
  *            LL_TIM_TIM16_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM17_BRK_GPIO
  *            LL_TIM_TIM17_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM20_BRK_GPIO
  *            LL_TIM_TIM20_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP4_OUT (*)
  * @endif
  *
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM20_BRK2_GPIO
  *            LL_TIM_TIM20_BRK2_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK2_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.
  * @param  polarity This parameter can be one of the following values:
  *         LL_TIM_BREAK_INPUT_SRC_NONINVERTED
  *         LL_TIM_BREAK_INPUT_SRC_INVERTED
  *


  */
__STATIC_INLINE void LL_TIM_SetBreakInputSourcePolarity(TIM_TypeDef *timx, uint32_t break_input, uint32_t source,
                                                        uint32_t polarity)
{
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->AF1) + (break_input << 2)));
  STM32_MODIFY_REG(*pReg, (TIM_AF1_BKINP << LL_TIM_POSITION_BRK_SOURCE(source)), \
                   (polarity << LL_TIM_POSITION_BRK_SOURCE(source)));
}

/**
  * @brief  Get the polarity of the break signal for the timer break input.
  * @rmtoll
  *  AF1          BKINP         LL_TIM_GetBreakInputSourcePolarity \n
  *  AF1          BKCMP1P       LL_TIM_GetBreakInputSourcePolarity \n
  *  AF1          BKCMP2P       LL_TIM_GetBreakInputSourcePolarity \n
  *  AF1          BKCMP3P       LL_TIM_GetBreakInputSourcePolarity \n
  *  AF1          BKCMP4P       LL_TIM_GetBreakInputSourcePolarity \n
  *  AF2          BK2INP        LL_TIM_GetBreakInputSourcePolarity \n
  *  AF2          BK2CMP1P      LL_TIM_GetBreakInputSourcePolarity \n
  *  AF2          BK2CMP2P      LL_TIM_GetBreakInputSourcePolarity \n
  *  AF2          BK2CMP3P      LL_TIM_GetBreakInputSourcePolarity \n
  *  AF2          BK2CMP4P      LL_TIM_GetBreakInputSourcePolarity
  * @param  timx Timer instance
  * @param  break_input This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_INPUT_1
  *         @arg @ref LL_TIM_BREAK_INPUT_2
  * @param  source The break and break2 input source parameter depends on timx. Description is available only
  *         in the CHM version of the User Manual (not in the PDF).
  *
  *         The description below summarizes specific "Timer Instance" and "BREAK(2) input source"
  *         parameter possibilities:
  *
  *         TIM1: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM1_BRK_GPIO
  *            LL_TIM_TIM1_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM1_BRK2_GPIO
  *            LL_TIM_TIM1_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM1_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM8: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM8_BRK_GPIO
  *            LL_TIM_TIM8_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM20
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM8_BRK2_GPIO
  *            LL_TIM_TIM8_BRK2_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_BRK2_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM8_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         TIM15: one of the following values:
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM15_BRK_GPIO
  *            LL_TIM_TIM15_BRK_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_BRK_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM15_BRK_COMP4_OUT (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM16_BRK_GPIO
  *            LL_TIM_TIM16_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM16_BRK_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM17
  *         TIM17: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM17_BRK_GPIO
  *            LL_TIM_TIM17_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM17_BRK_COMP4_OUT (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *
  *            . . BREAK can be one of the following values
  *            LL_TIM_TIM20_BRK_GPIO
  *            LL_TIM_TIM20_BRK_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK_COMP4_OUT (*)
  * @endif
  *
  *            . . BREAK2 can be one of the following values
  *            LL_TIM_TIM20_BRK2_GPIO
  *            LL_TIM_TIM20_BRK2_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_BRK2_COMP2_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP3_OUT (*)
  *            LL_TIM_TIM20_BRK2_COMP4_OUT (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.


  * @retval  Returned value can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_INPUT_SRC_NONINVERTED
  *         @arg @ref LL_TIM_BREAK_INPUT_SRC_INVERTED
  */
__STATIC_INLINE uint32_t LL_TIM_GetBreakInputSourcePolarity(const TIM_TypeDef *timx,
                                                            uint32_t break_input,
                                                            uint32_t source)
{
  const __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&timx->AF1) + (break_input << 2)));
  uint32_t bitfield = TIM_AF1_BKINP << LL_TIM_POSITION_BRK_SOURCE(source);
  return ((STM32_READ_BIT(*pReg, bitfield) == bitfield) ? \
          LL_TIM_BREAK_INPUT_SRC_INVERTED : LL_TIM_BREAK_INPUT_SRC_NONINVERTED);
}

/**
  * @brief  Set the delay duration for a specific break delay.
  * @rmtoll
  *  MPR2          DBK1          LL_TIM_SetBreakDelay \n
  *  MPR2          DBK2          LL_TIM_SetBreakDelay
  * @param  timx Timer instance
  * @param  break_delay This parameter can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_DELAY1
  *         @arg @ref LL_TIM_BREAK_DELAY2
  * @param  delay       Delay duration (between Min_Data=0 and Max_Data=255)
  * @note The delay duration can be changed on the fly as this control register is buffered. The new
  *       value is taken into account at the next update event.
  */
__STATIC_INLINE void LL_TIM_SetBreakDelay(TIM_TypeDef *timx, uint32_t break_delay, uint32_t delay)
{
  uint32_t dbk_shift = break_delay << 4U;
  STM32_MODIFY_REG(timx->MPR2, (TIM_MPR2_DBK1 << dbk_shift), (delay << dbk_shift));
}

/**
  * @brief  Get the delay duration for a specific break delay.
  * @rmtoll
  *  MPR2          DBK1          LL_TIM_GetBreakDelay \n
  *  MPR2          DBK2          LL_TIM_GetBreakDelay
  * @param  timx Timer instance
  * @param  break_delay This parameter can be one of the following values: \n
  *         @arg @ref LL_TIM_BREAK_DELAY1
  *         @arg @ref LL_TIM_BREAK_DELAY2
  * @retval Delay duration (between Min_Data=0 and Max_Data=255)
  */
__STATIC_INLINE uint32_t LL_TIM_GetBreakDelay(const TIM_TypeDef *timx, uint32_t break_delay)
{
  uint32_t dbk_shift = break_delay << 4U;
  return (uint32_t)((STM32_READ_BIT(timx->MPR2, (TIM_MPR2_DBK1 << dbk_shift))) >> dbk_shift);
}

/**
  * @brief  Enable asymmetrical deadtime.
  * @rmtoll
  *  DTR2          DTAE          LL_TIM_EnableAsymmetricalDeadTime
  * @param  timx Timer instance
  * @note Macro IS_TIM_DEADTIME_ASYMMETRICAL_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides asymmetrical deadtime.
  */
__STATIC_INLINE void LL_TIM_EnableAsymmetricalDeadTime(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DTR2, TIM_DTR2_DTAE);
}

/**
  * @brief  Disable asymmetrical dead-time.
  * @rmtoll
  *  DTR2          DTAE          LL_TIM_DisableAsymmetricalDeadTime
  * @param  timx Timer instance
  * @note Macro IS_TIM_DEADTIME_ASYMMETRICAL_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides asymmetrical deadtime.
  */
__STATIC_INLINE void LL_TIM_DisableAsymmetricalDeadTime(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DTR2, TIM_DTR2_DTAE);
}

/**
  * @brief  Indicates whether asymmetrical deadtime is activated.
  * @rmtoll
  *  DTR2          DTAE          LL_TIM_IsEnabledAsymmetricalDeadTime
  * @param  timx Timer instance
  * @note Macro IS_TIM_DEADTIME_ASYMMETRICAL_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides asymmetrical deadtime.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledAsymmetricalDeadTime(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DTR2, TIM_DTR2_DTAE) == (TIM_DTR2_DTAE)) ? 1UL : 0UL);
}

/**
  * @brief  Set the falling edge dead-time delay (delay inserted between the falling edge of the OCxREF signal and the
  *         rising edge of OCxN signals).
  * @rmtoll
  *  DTR2         DTGF           LL_TIM_SetFallingDeadTime
  * @param  timx Timer instance
  * @param  deadtime between Min_Data=0 and Max_Data=255
  * @note Macro IS_TIM_DEADTIME_ASYMMETRICAL_INSTANCE(timx) can be used to check whether or not
  *       asymmetrical dead-time insertion feature is supported by a timer instance.
  * @note Helper macro @ref LL_TIM_CALC_DEADTIME can be used to calculate the deadtime parameter
  * @note This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been programmed
  *       (LOCK bits in TIMx_BDTR register).
  */
__STATIC_INLINE void LL_TIM_SetFallingDeadTime(TIM_TypeDef *timx, uint32_t deadtime)
{
  STM32_MODIFY_REG(timx->DTR2, TIM_DTR2_DTGF, deadtime);
}

/**
  * @brief  Get the falling edge dead-time delay (delay inserted between the falling edge of the OCxREF signal and
  *         the rising edge of OCxN signals).
  * @rmtoll
  *  DTR2          DTGF           LL_TIM_GetFallingDeadTime
  * @param  timx Timer instance
  * @note Macro IS_TIM_DEADTIME_ASYMMETRICAL_INSTANCE(timx) can be used to check whether or not
  *       asymmetrical dead-time insertion feature is supported by a timer instance.
  * @note This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been programmed
  *       (LOCK bits in TIMx_BDTR register).
  * @retval Returned value can be between Min_Data=0 and Max_Data=255.
  */
__STATIC_INLINE uint32_t LL_TIM_GetFallingDeadTime(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->DTR2, TIM_DTR2_DTGF));
}

/**
  * @brief  Enable deadtime preload.
  * @rmtoll
  *  DTR2          DTPE          LL_TIM_EnableDeadTimePreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides deadtime preload.
  */
__STATIC_INLINE void LL_TIM_EnableDeadTimePreload(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DTR2, TIM_DTR2_DTPE);
}

/**
  * @brief  Disable dead-time preload.
  * @rmtoll
  *  DTR2          DTPE          LL_TIM_DisableDeadTimePreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides deadtime preload.
  */
__STATIC_INLINE void LL_TIM_DisableDeadTimePreload(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DTR2, TIM_DTR2_DTPE);
}

/**
  * @brief  Indicates whether deadtime preload is activated.
  * @rmtoll
  *  DTR2          DTPE          LL_TIM_IsEnabledDeadTimePreload
  * @param  timx Timer instance
  * @note Macro IS_TIM_BREAK_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides deadtime preload.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDeadTimePreload(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DTR2, TIM_DTR2_DTPE) == (TIM_DTR2_DTPE)) ? 1UL : 0UL);
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_DMA_Burst_Mode DMA burst mode configuration
  * @{
  */
/**
  * @brief  Configures the timer DMA burst feature.
  * @rmtoll
  *  DCR          DBSS          LL_TIM_ConfigDMABurst \n
  *  DCR          DBL           LL_TIM_ConfigDMABurst \n
  *  DCR          DBA           LL_TIM_ConfigDMABurst
  * @param  timx Timer instance
  * @param  dmaburst_base_address This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SMCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DIER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_EGR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CNT
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_PSC
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ARR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_RCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR4
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_BDTR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR5
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR6
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DTR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ECR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_TISEL
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR7
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR4
  * @param  dmaburst_length This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref LL_TIM_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_18TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_19TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_20TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_21TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_22TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_23TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_24TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_25TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_26TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_27TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_28TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_29TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_30TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_31TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_32TRANSFERS
  * @param  dmaburst_source This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_UPD
  *         @arg @ref LL_TIM_DMABURST_CC1
  *         @arg @ref LL_TIM_DMABURST_CC2  (*)
  *         @arg @ref LL_TIM_DMABURST_CC3  (*)
  *         @arg @ref LL_TIM_DMABURST_CC4  (*)
  *         @arg @ref LL_TIM_DMABURST_COM  (*)
  *         @arg @ref LL_TIM_DMABURST_TRGI (*)
  *
  *         (*)  Value not defined for all timer instances.
  * @note Macro IS_TIM_DMABURST_INSTANCE(timx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  */
__STATIC_INLINE void LL_TIM_ConfigDMABurst(TIM_TypeDef *timx, uint32_t dmaburst_base_address, uint32_t dmaburst_length,
                                           uint32_t dmaburst_source)
{
  STM32_MODIFY_REG(timx->DCR, (TIM_DCR_DBL | TIM_DCR_DBA | TIM_DCR_DBSS),
                   (dmaburst_base_address | dmaburst_length | dmaburst_source));
}

/**
  * @brief  Get the timer DMA burst configuration.
  * @rmtoll
  *  DCR          DBSS          LL_TIM_GetConfigDMABurst \n
  *  DCR          DBL           LL_TIM_GetConfigDMABurst \n
  *  DCR          DBA           LL_TIM_GetConfigDMABurst
  * @param  timx Timer instance
  * @param  p_dmaburst_base_address Pointer to a storage for DMA burst base address. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SMCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DIER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_EGR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CNT
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_PSC
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ARR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_RCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR4
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_BDTR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR5
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR6
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DTR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ECR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_TISEL
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR7
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR4
  * @param  p_dmaburst_length Pointer to a storage for DMA burst length. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref LL_TIM_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_18TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_19TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_20TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_21TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_22TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_23TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_24TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_25TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_26TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_27TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_28TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_29TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_30TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_31TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_32TRANSFERS
  * @param  p_dmaburst_source Pointer to a storage for DMA burst source. \n
  *         The value can be one of the following values: \n
  *         @arg @ref LL_TIM_DMABURST_UPD
  *         @arg @ref LL_TIM_DMABURST_CC1
  *         @arg @ref LL_TIM_DMABURST_CC2  (*)
  *         @arg @ref LL_TIM_DMABURST_CC3  (*)
  *         @arg @ref LL_TIM_DMABURST_CC4  (*)
  *         @arg @ref LL_TIM_DMABURST_COM  (*)
  *         @arg @ref LL_TIM_DMABURST_TRGI (*)
  *
  *         (*)  Value not defined for all timer instances.
  * @note Macro IS_TIM_DMABURST_INSTANCE(timx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  */
__STATIC_INLINE void LL_TIM_GetConfigDMABurst(TIM_TypeDef *timx, uint32_t *p_dmaburst_base_address,
                                              uint32_t *p_dmaburst_length, uint32_t *p_dmaburst_source)
{
  const __IO uint32_t dcr = STM32_READ_REG(timx->DCR);

  *p_dmaburst_base_address = (dcr & TIM_DCR_DBA);
  *p_dmaburst_length = (dcr & TIM_DCR_DBL);
  *p_dmaburst_source = (dcr & TIM_DCR_DBSS);
}

/**
  * @brief  Get the DMA burst source.
  * @rmtoll
  *  DCR          DBSS           LL_TIM_GetDMABurstSource
  * @param  timx Timer instance
  * @note Macro IS_TIM_DMABURST_INSTANCE(timx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_UPD
  *         @arg @ref LL_TIM_DMABURST_CC1
  *         @arg @ref LL_TIM_DMABURST_CC2  (*)
  *         @arg @ref LL_TIM_DMABURST_CC3  (*)
  *         @arg @ref LL_TIM_DMABURST_CC4  (*)
  *         @arg @ref LL_TIM_DMABURST_COM  (*)
  *         @arg @ref LL_TIM_DMABURST_TRGI (*)
  *
  *         (*)  Value not defined for all timer instances.
  */
__STATIC_INLINE uint32_t LL_TIM_GetDMABurstSource(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->DCR, TIM_DCR_DBSS));
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_Encoder Encoder configuration
  * @{
  */

/**
  * @brief  Enable encoder index.
  * @rmtoll
  *  ECR         IE           LL_TIM_EnableEncoderIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_EnableEncoderIndex(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->ECR, TIM_ECR_IE);
}

/**
  * @brief  Disable encoder index.
  * @rmtoll
  *  ECR         IE           LL_TIM_DisableEncoderIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_DisableEncoderIndex(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->ECR, TIM_ECR_IE);
}

/**
  * @brief  Indicate whether encoder index is enabled.
  * @rmtoll
  *  ECR         IE           LL_TIM_IsEnabledEncoderIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledEncoderIndex(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->ECR, TIM_ECR_IE) == (TIM_ECR_IE)) ? 1UL : 0UL);
}

/**
  * @brief  Set index direction.
  * @rmtoll
  *  ECR          IDIR           LL_TIM_SetIndexDirection
  * @param  timx Timer instance
  * @param  index_direction This parameter can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_UP_DOWN
  *         @arg @ref LL_TIM_INDEX_UP
  *         @arg @ref LL_TIM_INDEX_DOWN
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_SetIndexDirection(TIM_TypeDef *timx, uint32_t index_direction)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_IDIR, index_direction);
}

/**
  * @brief  Get actual index direction.
  * @rmtoll
  *  ECR          IDIR           LL_TIM_GetIndexDirection
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_UP_DOWN
  *         @arg @ref LL_TIM_INDEX_UP
  *         @arg @ref LL_TIM_INDEX_DOWN
  */
__STATIC_INLINE uint32_t LL_TIM_GetIndexDirection(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->ECR, TIM_ECR_IDIR));
}

/**
  * @brief  Set index blanking.
  * @rmtoll
  *  ECR          IBLK           LL_TIM_SetIndexBlanking
  * @param  timx Timer instance
  * @param  index_blanking This parameter can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_BLANK_ALWAYS
  *         @arg @ref LL_TIM_INDEX_BLANK_TI3
  *         @arg @ref LL_TIM_INDEX_BLANK_TI4
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_SetIndexBlanking(TIM_TypeDef *timx, uint32_t index_blanking)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_IBLK, index_blanking);
}

/**
  * @brief  Get actual index blanking.
  * @rmtoll
  *  ECR          IBLK           LL_TIM_GetIndexBlanking
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_BLANK_ALWAYS
  *         @arg @ref LL_TIM_INDEX_BLANK_TI3
  *         @arg @ref LL_TIM_INDEX_BLANK_TI4
  */
__STATIC_INLINE uint32_t LL_TIM_GetIndexBlanking(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->ECR, TIM_ECR_IBLK));
}


/**
  * @brief  Enable first index.
  * @rmtoll
  *  ECR          FIDX          LL_TIM_EnableFirstIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_EnableFirstIndex(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->ECR, TIM_ECR_FIDX);
}

/**
  * @brief  Disable first index.
  * @rmtoll
  *  ECR          FIDX          LL_TIM_DisableFirstIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_DisableFirstIndex(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->ECR, TIM_ECR_FIDX);
}

/**
  * @brief  Indicates whether first index is enabled.
  * @rmtoll
  *  ECR          FIDX          LL_TIM_IsEnabledFirstIndex
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledFirstIndex(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->ECR, TIM_ECR_FIDX) == (TIM_ECR_FIDX)) ? 1UL : 0UL);
}

/**
  * @brief  Set index positioning.
  * @rmtoll
  *  ECR          IPOS           LL_TIM_SetIndexPositionning
  * @param  timx Timer instance
  * @param  index_positioning This parameter can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN_UP
  *         @arg @ref LL_TIM_INDEX_POSITION_UP_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_UP_UP
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_UP
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_SetIndexPositionning(TIM_TypeDef *timx, uint32_t index_positioning)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_IPOS, index_positioning);
}

/**
  * @brief  Get actual index positioning.
  * @rmtoll
  *  ECR          IPOS           LL_TIM_GetIndexPositionning
  * @param  timx Timer instance
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN_UP
  *         @arg @ref LL_TIM_INDEX_POSITION_UP_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_UP_UP
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN
  *         @arg @ref LL_TIM_INDEX_POSITION_UP
  */
__STATIC_INLINE uint32_t LL_TIM_GetIndexPositionning(const TIM_TypeDef *timx)
{
  return (uint32_t)(STM32_READ_BIT(timx->ECR, TIM_ECR_IPOS));
}

/**
  * @brief  Configure encoder index.
  * @rmtoll
  *  ECR          IDIR          LL_TIM_ConfigEncoderIndex \n
  *  ECR          IBLK          LL_TIM_ConfigEncoderIndex \n
  *  ECR          FIDX          LL_TIM_ConfigEncoderIndex \n
  *  ECR          IPOS          LL_TIM_ConfigEncoderIndex
  * @param  timx Timer instance
  * @param  configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_INDEX_UP or @ref LL_TIM_INDEX_DOWN or @ref LL_TIM_INDEX_UP_DOWN
  *         @arg @ref LL_TIM_INDEX_BLANK_ALWAYS or @ref LL_TIM_INDEX_BLANK_TI3 or @ref LL_TIM_INDEX_BLANK_TI4
  *         @arg @ref LL_TIM_INDEX_ALL or @ref LL_TIM_INDEX_FIRST_ONLY
  *         @arg @ref LL_TIM_INDEX_POSITION_DOWN_DOWN or ... or @ref LL_TIM_INDEX_POSITION_UP
  * @note Macro IS_TIM_INDEX_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides an index input.
  */
__STATIC_INLINE void LL_TIM_ConfigEncoderIndex(TIM_TypeDef *timx, uint32_t configuration)
{
  STM32_MODIFY_REG(timx->ECR, TIM_ECR_IDIR | TIM_ECR_IBLK | TIM_ECR_FIDX | TIM_ECR_IPOS, configuration);
}

/**
  * @}
  */


/** @defgroup TIM_LL_EF_OCREF_Clear OCREF_Clear_Management
  * @{
  */
/**
  * @brief  Set the OCREF clear input source.
  * @rmtoll
  *  AF2           OCRSEL              LL_TIM_SetOCRefClearInputSource \n
  *  SMCR          OCCS                LL_TIM_SetOCRefClearInputSource
  * @param  timx Timer instance
  * @param  ocrefclear_input_source This parameter can be one of the following values:
  *         The description below summarizes specific "Timer Instance" and "OCRef clear input source"
  *         parameter possibilities:
  *
  *         TIM1: one of the following values:
  *            LL_TIM_TIM1_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM1_OCREF_CLR_INT_PLAY1_OUT0 (*)
  *            LL_TIM_TIM1_OCREF_CLR_INT_PLAY1_OUT6 (*)
  * @endif
  *
  *         TIM2: one of the following values:
  *            LL_TIM_TIM2_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM2_OCREF_CLR_INT_PLAY1_OUT15 (*)
  *            LL_TIM_TIM2_OCREF_CLR_INT_PLAY1_OUT7 (*)
  * @endif
  *
  *         TIM3: one of the following values: (**)
  *            LL_TIM_TIM3_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM3_OCREF_CLR_INT_PLAY1_OUT3 (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_PLAY1_OUT2 (*)
  * @endif
  *
  *         TIM4: one of the following values: (**)
  *            LL_TIM_TIM4_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM4_OCREF_CLR_INT_COMP1_OUT
  *
  *         TIM5: one of the following values: (**)
  *            LL_TIM_TIM5_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  *
  *         TIM8: one of the following values:
  *            LL_TIM_TIM8_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM8_OCREF_CLR_INT_PLAY1_OUT5 (*)
  *            LL_TIM_TIM8_OCREF_CLR_INT_PLAY1_OUT14 (*)
  * @endif
  *
  *         TIM15: one of the following values:
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM15_OCREF_CLR_INT_PLAY1_OUT7 (*)
  *            LL_TIM_TIM15_OCREF_CLR_INT_PLAY1_OUT8 (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM16_OCREF_CLR_INT_PLAY1_OUT9 (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_PLAY1_OUT10 (*)
  * @endif
  *
  *         TIM17: one of the following values: (**)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM17_OCREF_CLR_INT_PLAY1_OUT11 (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_PLAY1_OUT13 (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM20_OCREF_CLR_INT_PLAY1_OUT12 (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_PLAY1_OUT1 (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.

  * @note The OCxREF signal of a given channel can be cleared when a high level is applied on the OCREF_CLR_INPUT
  * @note This function can only be used in Output compare and PWM modes.
  */
__STATIC_INLINE void LL_TIM_SetOCRefClearInputSource(TIM_TypeDef *timx, uint32_t ocrefclear_input_source)
{
  STM32_MODIFY_REG(timx->SMCR, TIM_SMCR_OCCS, (ocrefclear_input_source & TIM_SMCR_OCCS));
  STM32_MODIFY_REG(timx->AF2, TIM_AF2_OCRSEL, (ocrefclear_input_source & TIM_AF2_OCRSEL));
}

/**
  * @brief  Get the OCREF clear input source.
  * @rmtoll
  *  AF2           OCRSEL              LL_TIM_GetOCRefClearInputSource \n
  *  SMCR          OCCS                LL_TIM_GetOCRefClearInputSource
  * @param  timx Timer instance
  * @retval Returned value can be one of the following values:
  *         The description below summarizes specific "Timer Instance" and "OCRef clear input source"
  *         parameter possibilities:
  *
  *         TIM1: one of the following values:
  *            LL_TIM_TIM1_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM1_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM1_OCREF_CLR_INT_PLAY1_OUT0 (*)
  *            LL_TIM_TIM1_OCREF_CLR_INT_PLAY1_OUT6 (*)
  * @endif
  *
  *         TIM2: one of the following values:
  *            LL_TIM_TIM2_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM2_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM2_OCREF_CLR_INT_PLAY1_OUT15 (*)
  *            LL_TIM_TIM2_OCREF_CLR_INT_PLAY1_OUT7 (*)
  * @endif
  *
  *         TIM3: one of the following values: (**)
  *            LL_TIM_TIM3_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM3_OCREF_CLR_INT_PLAY1_OUT3 (*)
  *            LL_TIM_TIM3_OCREF_CLR_INT_PLAY1_OUT2 (*)
  * @endif
  *
  *         TIM4: one of the following values: (**)
  *            LL_TIM_TIM4_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM4_OCREF_CLR_INT_COMP1_OUT
  *
  *         TIM5: one of the following values: (**)
  *            LL_TIM_TIM5_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM5_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  *
  *         TIM8: one of the following values:
  *            LL_TIM_TIM8_OCREF_CLR_INT_ETR
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM8_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM8_OCREF_CLR_INT_PLAY1_OUT5 (*)
  *            LL_TIM_TIM8_OCREF_CLR_INT_PLAY1_OUT14 (*)
  * @endif
  *
  *         TIM15: one of the following values:
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP1_OUT
  * @if COMP2
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP2_OUT (*)
  * @endif
  * @if COMP3
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM15_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM15_OCREF_CLR_INT_PLAY1_OUT7 (*)
  *            LL_TIM_TIM15_OCREF_CLR_INT_PLAY1_OUT8 (*)
  * @endif
  *
  * @if TIM16
  *         TIM16: one of the following values: (**)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM16_OCREF_CLR_INT_PLAY1_OUT9 (*)
  *            LL_TIM_TIM16_OCREF_CLR_INT_PLAY1_OUT10 (*)
  * @endif
  *
  *         TIM17: one of the following values: (**)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM17_OCREF_CLR_INT_PLAY1_OUT11 (*)
  *            LL_TIM_TIM17_OCREF_CLR_INT_PLAY1_OUT13 (*)
  * @endif
  * @endif
  *
  * @if TIM20
  *         TIM20: one of the following values: (**)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP1_OUT
  * @if COMP3
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP2_OUT (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP3_OUT (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_COMP4_OUT (*)
  * @endif
  * @if PLAY1
  *            LL_TIM_TIM20_OCREF_CLR_INT_PLAY1_OUT12 (*)
  *            LL_TIM_TIM20_OCREF_CLR_INT_PLAY1_OUT1 (*)
  * @endif
  * @endif
  *
  *         (*)  Value not defined in all devices.
  *         (**) Timer instance not available on all devices.

  */
__STATIC_INLINE uint32_t LL_TIM_GetOCRefClearInputSource(const TIM_TypeDef *timx)
{
  uint32_t src = (uint32_t)(STM32_READ_BIT(timx->SMCR, TIM_SMCR_OCCS));
  src |= (uint32_t)(STM32_READ_BIT(timx->AF2, TIM_AF2_OCRSEL));
  return src;
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_FLAG_Management FLAG-Management
  * @{
  */
/**
  * @brief  Clear the update interrupt flag (UIF).
  * @rmtoll
  *  SR           UIF           LL_TIM_ClearFlag_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_UPDATE(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_UIF));
}

/**
  * @brief  Indicate whether update interrupt flag (UIF) is set (update interrupt is pending).
  * @rmtoll
  *  SR           UIF           LL_TIM_IsActiveFlag_UPDATE
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_UPDATE(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_UIF) == (TIM_SR_UIF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 1 interrupt flag (CC1F).
  * @rmtoll
  *  SR           CC1IF         LL_TIM_ClearFlag_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC1(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC1IF));
}

/**
  * @brief  Indicate whether Capture/Compare 1 interrupt flag (CC1F) is set (Capture/Compare 1 interrupt is pending).
  * @rmtoll
  *  SR           CC1IF         LL_TIM_IsActiveFlag_CC1
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC1(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC1IF) == (TIM_SR_CC1IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 2 interrupt flag (CC2F).
  * @rmtoll
  *  SR           CC2IF         LL_TIM_ClearFlag_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC2(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC2IF));
}

/**
  * @brief  Indicate whether Capture/Compare 2 interrupt flag (CC2F) is set (Capture/Compare 2 interrupt is pending).
  * @rmtoll
  *  SR           CC2IF         LL_TIM_IsActiveFlag_CC2
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC2(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC2IF) == (TIM_SR_CC2IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 3 interrupt flag (CC3F).
  * @rmtoll
  *  SR           CC3IF         LL_TIM_ClearFlag_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC3(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC3IF));
}

/**
  * @brief  Indicate whether Capture/Compare 3 interrupt flag (CC3F) is set (Capture/Compare 3 interrupt is pending).
  * @rmtoll
  *  SR           CC3IF         LL_TIM_IsActiveFlag_CC3
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC3(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC3IF) == (TIM_SR_CC3IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 4 interrupt flag (CC4F).
  * @rmtoll
  *  SR           CC4IF         LL_TIM_ClearFlag_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC4(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC4IF));
}

/**
  * @brief  Indicate whether Capture/Compare 4 interrupt flag (CC4F) is set (Capture/Compare 4 interrupt is pending).
  * @rmtoll
  *  SR           CC4IF         LL_TIM_IsActiveFlag_CC4
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC4(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC4IF) == (TIM_SR_CC4IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 5 interrupt flag (CC5F).
  * @rmtoll
  *  SR           CC5IF         LL_TIM_ClearFlag_CC5
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC5(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC5IF));
}

/**
  * @brief  Indicate whether Capture/Compare 5 interrupt flag (CC5F) is set (Capture/Compare 5 interrupt is pending).
  * @rmtoll
  *  SR           CC5IF         LL_TIM_IsActiveFlag_CC5
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC5(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC5IF) == (TIM_SR_CC5IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 6 interrupt flag (CC6F).
  * @rmtoll
  *  SR           CC6IF         LL_TIM_ClearFlag_CC6
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC6(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC6IF));
}

/**
  * @brief  Indicate whether Capture/Compare 6 interrupt flag (CC6F) is set (Capture/Compare 6 interrupt is pending).
  * @rmtoll
  *  SR           CC6IF         LL_TIM_IsActiveFlag_CC6
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC6(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC6IF) == (TIM_SR_CC6IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 7 interrupt flag (CC7F).
  * @rmtoll
  *  SR           CC7IF         LL_TIM_ClearFlag_CC7
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC7(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC7IF));
}

/**
  * @brief  Indicate whether Capture/Compare 7 interrupt flag (CC7F) is set (Capture/Compare 7 interrupt is pending).
  * @rmtoll
  *  SR           CC7IF         LL_TIM_IsActiveFlag_CC7
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC7(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC7IF) == (TIM_SR_CC7IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the commutation interrupt flag (COMIF).
  * @rmtoll
  *  SR           COMIF         LL_TIM_ClearFlag_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_COM(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_COMIF));
}

/**
  * @brief  Indicate whether commutation interrupt flag (COMIF) is set (commutation interrupt is pending).
  * @rmtoll
  *  SR           COMIF         LL_TIM_IsActiveFlag_COM
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_COM(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_COMIF) == (TIM_SR_COMIF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the trigger interrupt flag (TIF).
  * @rmtoll
  *  SR           TIF           LL_TIM_ClearFlag_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_TRIG(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_TIF));
}

/**
  * @brief  Indicate whether trigger interrupt flag (TIF) is set (trigger interrupt is pending).
  * @rmtoll
  *  SR           TIF           LL_TIM_IsActiveFlag_TRIG
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_TRIG(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_TIF) == (TIM_SR_TIF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the break interrupt flag (BIF).
  * @rmtoll
  *  SR           BIF           LL_TIM_ClearFlag_BRK
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_BRK(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_BIF));
}

/**
  * @brief  Indicate whether break interrupt flag (BIF) is set (break interrupt is pending).
  * @rmtoll
  *  SR           BIF           LL_TIM_IsActiveFlag_BRK
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_BRK(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_BIF) == (TIM_SR_BIF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the break 2 interrupt flag (B2IF).
  * @rmtoll
  *  SR           B2IF          LL_TIM_ClearFlag_BRK2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_BRK2(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_B2IF));
}

/**
  * @brief  Indicate whether break 2 interrupt flag (B2IF) is set (break 2 interrupt is pending).
  * @rmtoll
  *  SR           B2IF          LL_TIM_IsActiveFlag_BRK2
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_BRK2(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_B2IF) == (TIM_SR_B2IF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the system break interrupt flag (SBIF).
  * @rmtoll
  *  SR           SBIF          LL_TIM_ClearFlag_SYSBRK
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_SYSBRK(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_SBIF));
}

/**
  * @brief  Indicate whether system break interrupt flag (SBIF) is set (system break interrupt is pending).
  * @rmtoll
  *  SR           SBIF          LL_TIM_IsActiveFlag_SYSBRK
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_SYSBRK(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_SBIF) == (TIM_SR_SBIF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the break generation flag (BGF).
  * @rmtoll
  *  SR           BGF           LL_TIM_ClearFlag_BG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_BG(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_BGF));
}

/**
  * @brief  Indicate whether break generation flag (BGF) is set (break interrupt is pending).
  * @rmtoll
  *  SR           BGF         LL_TIM_IsActiveFlag_BG
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_BG(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_BGF) == (TIM_SR_BGF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the break 2 generation flag (B2GF).
  * @rmtoll
  *  SR           B2GF          LL_TIM_ClearFlag_B2G
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_B2G(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_B2GF));
}

/**
  * @brief  Indicate whether break 2 generation flag (B2GF) is set (break 2 interrupt is pending).
  * @rmtoll
  *  SR           B2GF        LL_TIM_IsActiveFlag_B2G
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_B2G(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_B2GF) == (TIM_SR_B2GF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 1 over-capture interrupt flag (CC1OF).
  * @rmtoll
  *  SR           CC1OF         LL_TIM_ClearFlag_CC1OVR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC1OVR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC1OF));
}

/**
  * @brief  Indicate whether Capture/Compare 1 over-capture interrupt flag (CC1OF) is set
  *         (Capture/Compare 1 interrupt is pending).
  * @rmtoll
  *  SR           CC1OF         LL_TIM_IsActiveFlag_CC1OVR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC1OVR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC1OF) == (TIM_SR_CC1OF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 2 over-capture interrupt flag (CC2OF).
  * @rmtoll
  *  SR           CC2OF         LL_TIM_ClearFlag_CC2OVR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC2OVR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC2OF));
}

/**
  * @brief  Indicate whether Capture/Compare 2 over-capture interrupt flag (CC2OF) is set
  *         (Capture/Compare 2 over-capture interrupt is pending).
  * @rmtoll
  *  SR           CC2OF         LL_TIM_IsActiveFlag_CC2OVR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC2OVR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC2OF) == (TIM_SR_CC2OF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 3 over-capture interrupt flag (CC3OF).
  * @rmtoll
  *  SR           CC3OF         LL_TIM_ClearFlag_CC3OVR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC3OVR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC3OF));
}

/**
  * @brief  Indicate whether Capture/Compare 3 over-capture interrupt flag (CC3OF) is set
  *         (Capture/Compare 3 over-capture interrupt is pending).
  * @rmtoll
  *  SR           CC3OF         LL_TIM_IsActiveFlag_CC3OVR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC3OVR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC3OF) == (TIM_SR_CC3OF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the Capture/Compare 4 over-capture interrupt flag (CC4OF).
  * @rmtoll
  *  SR           CC4OF         LL_TIM_ClearFlag_CC4OVR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_CC4OVR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_CC4OF));
}

/**
  * @brief  Indicate whether Capture/Compare 4 over-capture interrupt flag (CC4OF) is set
  *         (Capture/Compare 4 over-capture interrupt is pending).
  * @rmtoll
  *  SR           CC4OF         LL_TIM_IsActiveFlag_CC4OVR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_CC4OVR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_CC4OF) == (TIM_SR_CC4OF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the transition error interrupt flag (TERRF).
  * @rmtoll
  *  SR           TERRF           LL_TIM_ClearFlag_TERR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_TERR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_TERRF));
}

/**
  * @brief  Indicate whether transition error interrupt flag (TERRF) is set (transition error interrupt is pending).
  * @rmtoll
  *  SR           TERRF           LL_TIM_IsActiveFlag_TERR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_TERR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_TERRF) == (TIM_SR_TERRF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the index error interrupt flag (IERRF).
  * @rmtoll
  *  SR           IERRF           LL_TIM_ClearFlag_IERR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_IERR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_IERRF));
}

/**
  * @brief  Indicate whether index error interrupt flag (IERRF) is set (index error interrupt is pending).
  * @rmtoll
  *  SR           IERRF           LL_TIM_IsActiveFlag_IERR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_IERR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_IERRF) == (TIM_SR_IERRF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the direction change interrupt flag (DIRF).
  * @rmtoll
  *  SR           DIRF           LL_TIM_ClearFlag_DIR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_DIR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_DIRF));
}

/**
  * @brief  Indicate whether direction change interrupt flag (DIRF) is set (direction change interrupt is pending).
  * @rmtoll
  *  SR           DIRF           LL_TIM_IsActiveFlag_DIR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_DIR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_DIRF) == (TIM_SR_DIRF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the index interrupt flag (IDXF).
  * @rmtoll
  *  SR           IDXF           LL_TIM_ClearFlag_IDX
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_IDX(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_IDXF));
}

/**
  * @brief  Indicate whether index interrupt flag (IDXF) is set (index interrupt is pending).
  * @rmtoll
  *  SR           IDXF           LL_TIM_IsActiveFlag_IDX
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_IDX(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_IDXF) == (TIM_SR_IDXF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear the overrun flag (UIOVRF).
  * @rmtoll
  *  SR           UIOVRF         LL_TIM_ClearFlag_UIOVR
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_ClearFlag_UIOVR(TIM_TypeDef *timx)
{
  STM32_WRITE_REG(timx->SR, ~(TIM_SR_UIOVRF));
}

/**
  * @brief  Indicate whether overrun flag (UIOVRF) is set (another interrupt is pending).
  * @rmtoll
  *  SR           UIOVRF         LL_TIM_IsActiveFlag_UIOVR
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsActiveFlag_UIOVR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->SR, TIM_SR_UIOVRF) == (TIM_SR_UIOVRF)) ? 1UL : 0UL);
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_IT_Management IT-Management
  * @{
  */
/**
  * @brief  Enable update interrupt (UIE).
  * @rmtoll
  *  DIER         UIE           LL_TIM_EnableIT_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_UPDATE(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_UIE);
}

/**
  * @brief  Disable update interrupt (UIE).
  * @rmtoll
  *  DIER         UIE           LL_TIM_DisableIT_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_UPDATE(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_UIE);
}

/**
  * @brief  Indicates whether the update interrupt (UIE) is enabled.
  * @rmtoll
  *  DIER         UIE           LL_TIM_IsEnabledIT_UPDATE
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_UPDATE(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_UIE) == (TIM_DIER_UIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 1 interrupt (CC1IE).
  * @rmtoll
  *  DIER         CC1IE         LL_TIM_EnableIT_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC1(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC1IE);
}

/**
  * @brief  Disable capture/compare 1  interrupt (CC1IE).
  * @rmtoll
  *  DIER         CC1IE         LL_TIM_DisableIT_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC1(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC1IE);
}

/**
  * @brief  Indicates whether the capture/compare 1 interrupt (CC1IE) is enabled.
  * @rmtoll
  *  DIER         CC1IE         LL_TIM_IsEnabledIT_CC1
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC1(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC1IE) == (TIM_DIER_CC1IE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 2 interrupt (CC2IE).
  * @rmtoll
  *  DIER         CC2IE         LL_TIM_EnableIT_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC2(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC2IE);
}

/**
  * @brief  Disable capture/compare 2  interrupt (CC2IE).
  * @rmtoll
  *  DIER         CC2IE         LL_TIM_DisableIT_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC2(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC2IE);
}

/**
  * @brief  Indicates whether the capture/compare 2 interrupt (CC2IE) is enabled.
  * @rmtoll
  *  DIER         CC2IE         LL_TIM_IsEnabledIT_CC2
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC2(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC2IE) == (TIM_DIER_CC2IE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 3 interrupt (CC3IE).
  * @rmtoll
  *  DIER         CC3IE         LL_TIM_EnableIT_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC3(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC3IE);
}

/**
  * @brief  Disable capture/compare 3  interrupt (CC3IE).
  * @rmtoll
  *  DIER         CC3IE         LL_TIM_DisableIT_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC3(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC3IE);
}

/**
  * @brief  Indicates whether the capture/compare 3 interrupt (CC3IE) is enabled.
  * @rmtoll
  *  DIER         CC3IE         LL_TIM_IsEnabledIT_CC3
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC3(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC3IE) == (TIM_DIER_CC3IE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 4 interrupt (CC4IE).
  * @rmtoll
  *  DIER         CC4IE         LL_TIM_EnableIT_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_CC4(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC4IE);
}

/**
  * @brief  Disable capture/compare 4  interrupt (CC4IE).
  * @rmtoll
  *  DIER         CC4IE         LL_TIM_DisableIT_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_CC4(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC4IE);
}

/**
  * @brief  Indicates whether the capture/compare 4 interrupt (CC4IE) is enabled.
  * @rmtoll
  *  DIER         CC4IE         LL_TIM_IsEnabledIT_CC4
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_CC4(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC4IE) == (TIM_DIER_CC4IE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable commutation interrupt (COMIE).
  * @rmtoll
  *  DIER         COMIE         LL_TIM_EnableIT_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_COM(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_COMIE);
}

/**
  * @brief  Disable commutation interrupt (COMIE).
  * @rmtoll
  *  DIER         COMIE         LL_TIM_DisableIT_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_COM(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_COMIE);
}

/**
  * @brief  Indicates whether the commutation interrupt (COMIE) is enabled.
  * @rmtoll
  *  DIER         COMIE         LL_TIM_IsEnabledIT_COM
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_COM(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_COMIE) == (TIM_DIER_COMIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable trigger interrupt (TIE).
  * @rmtoll
  *  DIER         TIE           LL_TIM_EnableIT_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_TRIG(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_TIE);
}

/**
  * @brief  Disable trigger interrupt (TIE).
  * @rmtoll
  *  DIER         TIE           LL_TIM_DisableIT_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_TRIG(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_TIE);
}

/**
  * @brief  Indicates whether the trigger interrupt (TIE) is enabled.
  * @rmtoll
  *  DIER         TIE           LL_TIM_IsEnabledIT_TRIG
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_TRIG(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_TIE) == (TIM_DIER_TIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable break interrupt (BIE).
  * @rmtoll
  *  DIER         BIE           LL_TIM_EnableIT_BRK
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableIT_BRK(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_BIE);
}

/**
  * @brief  Disable break interrupt (BIE).
  * @rmtoll
  *  DIER         BIE           LL_TIM_DisableIT_BRK
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableIT_BRK(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_BIE);
}

/**
  * @brief  Indicates whether the break interrupt (BIE) is enabled.
  * @rmtoll
  *  DIER         BIE           LL_TIM_IsEnabledIT_BRK
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_BRK(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_BIE) == (TIM_DIER_BIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable transition error interrupt (TERRIE).
  * @rmtoll
  *  DIER         TERRIE           LL_TIM_EnableIT_TERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  */
__STATIC_INLINE void LL_TIM_EnableIT_TERR(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_TERRIE);
}

/**
  * @brief  Disable transition error interrupt (TERRIE).
  * @rmtoll
  *  DIER         TERRIE           LL_TIM_DisableIT_TERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  */
__STATIC_INLINE void LL_TIM_DisableIT_TERR(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_TERRIE);
}

/**
  * @brief  Indicates whether the transition error interrupt (TERRIE) is enabled.
  * @rmtoll
  *  DIER         TERRIE           LL_TIM_IsEnabledIT_TERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_TERR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_TERRIE) == (TIM_DIER_TERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable index error interrupt (IERRIE).
  * @rmtoll
  *  DIER         IERRIE           LL_TIM_EnableIT_IERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  */
__STATIC_INLINE void LL_TIM_EnableIT_IERR(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_IERRIE);
}

/**
  * @brief  Disable index error interrupt (IERRIE).
  * @rmtoll
  *  DIER         IERRIE           LL_TIM_DisableIT_IERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  */
__STATIC_INLINE void LL_TIM_DisableIT_IERR(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_IERRIE);
}

/**
  * @brief  Indicates whether the index error interrupt (IERRIE) is enabled.
  * @rmtoll
  *  DIER         IERRIE           LL_TIM_IsEnabledIT_IERR
  * @param  timx Timer instance
  * @note Macro IS_TIM_ENCODER_ERROR_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder error management.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_IERR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_IERRIE) == (TIM_DIER_IERRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable direction change interrupt (DIRIE).
  * @rmtoll
  *  DIER         DIRIE           LL_TIM_EnableIT_DIR
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  */
__STATIC_INLINE void LL_TIM_EnableIT_DIR(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_DIRIE);
}

/**
  * @brief  Disable direction change interrupt (DIRIE).
  * @rmtoll
  *  DIER         DIRIE           LL_TIM_DisableIT_DIR
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  */
__STATIC_INLINE void LL_TIM_DisableIT_DIR(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_DIRIE);
}

/**
  * @brief  Indicates whether the direction change interrupt (DIRIE) is enabled.
  * @rmtoll
  *  DIER         DIRIE           LL_TIM_IsEnabledIT_DIR
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_DIR(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_DIRIE) == (TIM_DIER_DIRIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable index interrupt (IDXIE).
  * @rmtoll
  *  DIER         IDXIE           LL_TIM_EnableIT_IDX
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  */
__STATIC_INLINE void LL_TIM_EnableIT_IDX(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_IDXIE);
}

/**
  * @brief  Disable index interrupt (IDXIE).
  * @rmtoll
  *  DIER         IDXIE           LL_TIM_DisableIT_IDX
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  */
__STATIC_INLINE void LL_TIM_DisableIT_IDX(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_IDXIE);
}

/**
  * @brief  Indicates whether the index interrupt (IDXIE) is enabled.
  * @rmtoll
  *  DIER         IDXIE           LL_TIM_IsEnabledIT_IDX
  * @param  timx Timer instance
  * @note Macro IS_TIM_FUNCTINONAL_ENCODER_INTERRUPT_INSTANCE(timx) can be used to check whether or not
  *       a timer instance provides encoder interrupt management.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledIT_IDX(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_IDXIE) == (TIM_DIER_IDXIE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the interrupt(s).
  * @rmtoll
  *  DIER         BIE             LL_TIM_EnableIT \n
  *  DIER         UIE             LL_TIM_EnableIT \n
  *  DIER         CC1IE           LL_TIM_EnableIT \n
  *  DIER         CC2IE           LL_TIM_EnableIT \n
  *  DIER         CC3IE           LL_TIM_EnableIT \n
  *  DIER         CC4IE           LL_TIM_EnableIT \n
  *  DIER         COMIE           LL_TIM_EnableIT \n
  *  DIER         IDXIE           LL_TIM_EnableIT \n
  *  DIER         DIRIE           LL_TIM_EnableIT \n
  *  DIER         IERRIE          LL_TIM_EnableIT \n
  *  DIER         TERRIE          LL_TIM_EnableIT \n
  *  DIER         TIE             LL_TIM_EnableIT
  * @param  timx Timer instance
  * @param  it_mask specifies the interrupt source(s) to enable.
  *         This parameter can be any combination of the following values:
  *         @arg @ref LL_TIM_DIER_UIE
  *         @arg @ref LL_TIM_DIER_CC1IE
  *         @arg @ref LL_TIM_DIER_CC2IE
  *         @arg @ref LL_TIM_DIER_CC3IE
  *         @arg @ref LL_TIM_DIER_CC4IE
  *         @arg @ref LL_TIM_DIER_COMIE
  *         @arg @ref LL_TIM_DIER_TIE
  *         @arg @ref LL_TIM_DIER_BIE
  *         @arg @ref LL_TIM_DIER_IDXIE
  *         @arg @ref LL_TIM_DIER_DIRIE
  *         @arg @ref LL_TIM_DIER_IERRIE
  *         @arg @ref LL_TIM_DIER_TERRIE
  */
__STATIC_INLINE void LL_TIM_EnableIT(TIM_TypeDef *timx, uint32_t it_mask)
{
  STM32_SET_BIT(timx->DIER, it_mask);
}

/**
  * @brief  Disable the interrupt(s).
  * @rmtoll
  *  DIER         BIE             LL_TIM_DisableIT \n
  *  DIER         UIE             LL_TIM_DisableIT \n
  *  DIER         CC1IE           LL_TIM_DisableIT \n
  *  DIER         CC2IE           LL_TIM_DisableIT \n
  *  DIER         CC3IE           LL_TIM_DisableIT \n
  *  DIER         CC4IE           LL_TIM_DisableIT \n
  *  DIER         COMIE           LL_TIM_DisableIT \n
  *  DIER         IDXIE           LL_TIM_DisableIT \n
  *  DIER         DIRIE           LL_TIM_DisableIT \n
  *  DIER         IERRIE          LL_TIM_DisableIT \n
  *  DIER         TERRIE          LL_TIM_DisableIT \n
  *  DIER         TIE             LL_TIM_DisableIT
  * @param  timx Timer instance
  * @param  it_mask specifies the interrupt source(s) to disable.
  *         This parameter can be any combination of the following values:
  *         @arg @ref LL_TIM_DIER_UIE
  *         @arg @ref LL_TIM_DIER_CC1IE
  *         @arg @ref LL_TIM_DIER_CC2IE
  *         @arg @ref LL_TIM_DIER_CC3IE
  *         @arg @ref LL_TIM_DIER_CC4IE
  *         @arg @ref LL_TIM_DIER_COMIE
  *         @arg @ref LL_TIM_DIER_TIE
  *         @arg @ref LL_TIM_DIER_BIE
  *         @arg @ref LL_TIM_DIER_IDXIE
  *         @arg @ref LL_TIM_DIER_DIRIE
  *         @arg @ref LL_TIM_DIER_IERRIE
  *         @arg @ref LL_TIM_DIER_TERRIE
  */
__STATIC_INLINE void LL_TIM_DisableIT(TIM_TypeDef *timx, uint32_t it_mask)
{
  STM32_CLEAR_BIT(timx->DIER, it_mask);
}
/**
  * @}
  */

/** @defgroup TIM_LL_EF_DMA_Management DMA Management
  * @{
  */
/**
  * @brief  Enable update DMA request (UDE).
  * @rmtoll
  *  DIER         UDE           LL_TIM_EnableDMAReq_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_UPDATE(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_UDE);
}

/**
  * @brief  Disable update DMA request (UDE).
  * @rmtoll
  *  DIER         UDE           LL_TIM_DisableDMAReq_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_UPDATE(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_UDE);
}

/**
  * @brief  Indicates whether the update DMA request  (UDE) is enabled.
  * @rmtoll
  *  DIER         UDE           LL_TIM_IsEnabledDMAReq_UPDATE
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_UPDATE(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_UDE) == (TIM_DIER_UDE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 1 DMA request (CC1DE).
  * @rmtoll
  *  DIER         CC1DE         LL_TIM_EnableDMAReq_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC1(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC1DE);
}

/**
  * @brief  Disable capture/compare 1  DMA request (CC1DE).
  * @rmtoll
  *  DIER         CC1DE         LL_TIM_DisableDMAReq_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC1(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC1DE);
}

/**
  * @brief  Indicates whether the capture/compare 1 DMA request (CC1DE) is enabled.
  * @rmtoll
  *  DIER         CC1DE         LL_TIM_IsEnabledDMAReq_CC1
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC1(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC1DE) == (TIM_DIER_CC1DE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 2 DMA request (CC2DE).
  * @rmtoll
  *  DIER         CC2DE         LL_TIM_EnableDMAReq_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC2(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC2DE);
}

/**
  * @brief  Disable capture/compare 2  DMA request (CC2DE).
  * @rmtoll
  *  DIER         CC2DE         LL_TIM_DisableDMAReq_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC2(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC2DE);
}

/**
  * @brief  Indicates whether the capture/compare 2 DMA request (CC2DE) is enabled.
  * @rmtoll
  *  DIER         CC2DE         LL_TIM_IsEnabledDMAReq_CC2
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC2(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC2DE) == (TIM_DIER_CC2DE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 3 DMA request (CC3DE).
  * @rmtoll
  *  DIER         CC3DE         LL_TIM_EnableDMAReq_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC3(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC3DE);
}

/**
  * @brief  Disable capture/compare 3  DMA request (CC3DE).
  * @rmtoll
  *  DIER         CC3DE         LL_TIM_DisableDMAReq_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC3(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC3DE);
}

/**
  * @brief  Indicates whether the capture/compare 3 DMA request (CC3DE) is enabled.
  * @rmtoll
  *  DIER         CC3DE         LL_TIM_IsEnabledDMAReq_CC3
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC3(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC3DE) == (TIM_DIER_CC3DE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable capture/compare 4 DMA request (CC4DE).
  * @rmtoll
  *  DIER         CC4DE         LL_TIM_EnableDMAReq_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_CC4(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_CC4DE);
}

/**
  * @brief  Disable capture/compare 4  DMA request (CC4DE).
  * @rmtoll
  *  DIER         CC4DE         LL_TIM_DisableDMAReq_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_CC4(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_CC4DE);
}

/**
  * @brief  Indicates whether the capture/compare 4 DMA request (CC4DE) is enabled.
  * @rmtoll
  *  DIER         CC4DE         LL_TIM_IsEnabledDMAReq_CC4
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_CC4(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_CC4DE) == (TIM_DIER_CC4DE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable commutation DMA request (COMDE).
  * @rmtoll
  *  DIER         COMDE         LL_TIM_EnableDMAReq_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_COM(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_COMDE);
}

/**
  * @brief  Disable commutation DMA request (COMDE).
  * @rmtoll
  *  DIER         COMDE         LL_TIM_DisableDMAReq_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_COM(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_COMDE);
}

/**
  * @brief  Indicates whether the commutation DMA request (COMDE) is enabled.
  * @rmtoll
  *  DIER         COMDE         LL_TIM_IsEnabledDMAReq_COM
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_COM(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_COMDE) == (TIM_DIER_COMDE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable trigger interrupt (TDE).
  * @rmtoll
  *  DIER         TDE           LL_TIM_EnableDMAReq_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq_TRIG(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->DIER, TIM_DIER_TDE);
}

/**
  * @brief  Disable trigger interrupt (TDE).
  * @rmtoll
  *  DIER         TDE           LL_TIM_DisableDMAReq_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq_TRIG(TIM_TypeDef *timx)
{
  STM32_CLEAR_BIT(timx->DIER, TIM_DIER_TDE);
}

/**
  * @brief  Indicates whether the trigger interrupt (TDE) is enabled.
  * @rmtoll
  *  DIER         TDE           LL_TIM_IsEnabledDMAReq_TRIG
  * @param  timx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_TIM_IsEnabledDMAReq_TRIG(const TIM_TypeDef *timx)
{
  return ((STM32_READ_BIT(timx->DIER, TIM_DIER_TDE) == (TIM_DIER_TDE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the selected dma request(s).
  * @rmtoll
  *  DIER         UDE             LL_TIM_EnableDMAReq \n
  *  DIER         CC1DE           LL_TIM_EnableDMAReq \n
  *  DIER         CC2DE           LL_TIM_EnableDMAReq \n
  *  DIER         CC3DE           LL_TIM_EnableDMAReq \n
  *  DIER         CC4DE           LL_TIM_EnableDMAReq \n
  *  DIER         COMDE           LL_TIM_EnableDMAReq \n
  *  DIER         TDE             LL_TIM_EnableDMAReq
  * @param  timx Timer instance
  * @param  dma_mask specifies the DMA request source(s) to enable.
  *         This parameter can be any combination of the following values:
  *         @arg @ref LL_TIM_DIER_UDE
  *         @arg @ref LL_TIM_DIER_CC1DE
  *         @arg @ref LL_TIM_DIER_CC2DE
  *         @arg @ref LL_TIM_DIER_CC3DE
  *         @arg @ref LL_TIM_DIER_CC4DE
  *         @arg @ref LL_TIM_DIER_COMDE
  *         @arg @ref LL_TIM_DIER_TDE
  */
__STATIC_INLINE void LL_TIM_EnableDMAReq(TIM_TypeDef *timx, uint32_t dma_mask)
{
  STM32_SET_BIT(timx->DIER, dma_mask);
}

/**
  * @brief  Disable the selected dma request(s).
  * @rmtoll
  *  DIER         UDE             LL_TIM_DisableDMAReq \n
  *  DIER         CC1DE           LL_TIM_DisableDMAReq \n
  *  DIER         CC2DE           LL_TIM_DisableDMAReq \n
  *  DIER         CC3DE           LL_TIM_DisableDMAReq \n
  *  DIER         CC4DE           LL_TIM_DisableDMAReq \n
  *  DIER         COMDE           LL_TIM_DisableDMAReq \n
  *  DIER         TDE             LL_TIM_DisableDMAReq
  * @param  timx Timer instance
  * @param  dma_mask specifies the DMA request source(s) to disable.
  *         This parameter can be any combination of the following values:
  *         @arg @ref LL_TIM_DIER_UDE
  *         @arg @ref LL_TIM_DIER_CC1DE
  *         @arg @ref LL_TIM_DIER_CC2DE
  *         @arg @ref LL_TIM_DIER_CC3DE
  *         @arg @ref LL_TIM_DIER_CC4DE
  *         @arg @ref LL_TIM_DIER_COMDE
  *         @arg @ref LL_TIM_DIER_TDE
  */
__STATIC_INLINE void LL_TIM_DisableDMAReq(TIM_TypeDef *timx, uint32_t dma_mask)
{
  STM32_CLEAR_BIT(timx->DIER, dma_mask);
}

/**
  * @}
  */

/** @defgroup TIM_LL_EF_EVENT_Management EVENT-Management
  * @{
  */
/**
  * @brief  Generate an update event.
  * @rmtoll
  *  EGR          UG            LL_TIM_GenerateEvent_UPDATE
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_UPDATE(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_UG);
}

/**
  * @brief  Generate Capture/Compare 1 event.
  * @rmtoll
  *  EGR          CC1G          LL_TIM_GenerateEvent_CC1
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC1(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_CC1G);
}

/**
  * @brief  Generate Capture/Compare 2 event.
  * @rmtoll
  *  EGR          CC2G          LL_TIM_GenerateEvent_CC2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC2(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_CC2G);
}

/**
  * @brief  Generate Capture/Compare 3 event.
  * @rmtoll
  *  EGR          CC3G          LL_TIM_GenerateEvent_CC3
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC3(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_CC3G);
}

/**
  * @brief  Generate Capture/Compare 4 event.
  * @rmtoll
  *  EGR          CC4G          LL_TIM_GenerateEvent_CC4
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_CC4(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_CC4G);
}

/**
  * @brief  Generate commutation event.
  * @rmtoll
  *  EGR          COMG          LL_TIM_GenerateEvent_COM
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_COM(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_COMG);
}

/**
  * @brief  Generate trigger event.
  * @rmtoll
  *  EGR          TG            LL_TIM_GenerateEvent_TRIG
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_TRIG(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_TG);
}

/**
  * @brief  Generate break event.
  * @rmtoll
  *  EGR          BG            LL_TIM_GenerateEvent_BRK
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_BRK(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_BG);
}

/**
  * @brief  Generate break 2 event.
  * @rmtoll
  *  EGR          B2G           LL_TIM_GenerateEvent_BRK2
  * @param  timx Timer instance
  */
__STATIC_INLINE void LL_TIM_GenerateEvent_BRK2(TIM_TypeDef *timx)
{
  STM32_SET_BIT(timx->EGR, TIM_EGR_B2G);
}


/**
  * @brief  Generate software event(s).
  * @rmtoll
  *  EGR          UG            LL_TIM_GenerateEvent \n
  *  EGR          CC1G          LL_TIM_GenerateEvent \n
  *  EGR          CC2G          LL_TIM_GenerateEvent \n
  *  EGR          CC3G          LL_TIM_GenerateEvent \n
  *  EGR          CC4G          LL_TIM_GenerateEvent \n
  *  EGR          COMG          LL_TIM_GenerateEvent \n
  *  EGR          TG            LL_TIM_GenerateEvent \n
  *  EGR          BG            LL_TIM_GenerateEvent \n
  *  EGR          B2G           LL_TIM_GenerateEvent
  * @param  timx Timer instance
  * @param  software_event specifies the software event source(s) to generate.
  *         This parameter can be any combination of the following values:
  *         @arg @ref LL_TIM_SW_EVENT_UPD
  *         @arg @ref LL_TIM_SW_EVENT_CC1
  *         @arg @ref LL_TIM_SW_EVENT_CC2
  *         @arg @ref LL_TIM_SW_EVENT_CC3
  *         @arg @ref LL_TIM_SW_EVENT_CC4
  *         @arg @ref LL_TIM_SW_EVENT_COM
  *         @arg @ref LL_TIM_SW_EVENT_TRGI
  *         @arg @ref LL_TIM_SW_EVENT_BRK
  *         @arg @ref LL_TIM_SW_EVENT_BRK2
  */
__STATIC_INLINE void LL_TIM_GenerateEvent(TIM_TypeDef *timx, uint32_t software_event)
{
  STM32_SET_BIT(timx->EGR, software_event);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* TIM1 || TIM2 || TIM3 || TIM4 || TIM5 || TIM6 || TIM7 || TIM8 || TIM12 || TIM15 || TIM16 || TIM17 */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_TIM_H */

