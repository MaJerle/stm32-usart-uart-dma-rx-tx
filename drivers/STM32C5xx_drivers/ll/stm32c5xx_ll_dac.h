/**
  ******************************************************************************
  * @file    stm32c5xx_ll_dac.h
  * @brief   Header file for the DAC LL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32C5XX_LL_DAC_H
#define STM32C5XX_LL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(DAC1)
/** @defgroup DAC_LL DAC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup DAC_LL_Private_Constants DAC Private Constants
  * @{
  */

/* Internal masks for DAC channels definition */
/* To select into literal LL_DAC_CHANNEL_x the relevant bits for:             */
/* - channel bits position into registers CR, MCR, CCR, SHHR, SHRR            */
/* - channel bits position into register SWTRIG                               */
/* - channel register offset of data holding register DHRx                    */
/* - channel register offset of data output register DORx                     */
/* - channel register offset of sample-and-hold sample time register SHSRx    */
#define DAC_CR_CH1_BITOFFSET           0UL   /* Position of channel bits into registers
                                                CR, MCR, CCR, SHHR, SHRR of channel 1 */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define DAC_CR_CH2_BITOFFSET           16UL  /* Position of channel bits into registers
                                                CR, MCR, CCR, SHHR, SHRR of channel 2 */
#define DAC_CR_CHX_BITOFFSET_MASK      (DAC_CR_CH1_BITOFFSET | DAC_CR_CH2_BITOFFSET)
#else
#define DAC_CR_CHX_BITOFFSET_MASK      (DAC_CR_CH1_BITOFFSET)
#endif /* DAC_NB_OF_CHANNEL == 2 */
#define DAC_SWTR_CH1                   (DAC_SWTRGR_SWTRIG1) /* Channel bit into register SWTRIGR of channel 1. */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define DAC_SWTR_CH2                   (DAC_SWTRGR_SWTRIG2) /* Channel bit into register SWTRIGR of channel 2. */
#define DAC_SWTR_CHX_MASK              (DAC_SWTR_CH1 | DAC_SWTR_CH2)
#else
#define DAC_SWTR_CHX_MASK              (DAC_SWTR_CH1)
#endif /* DAC_NB_OF_CHANNEL == 2 */

#define DAC_REG_DHR12R1_REGOFFSET      0x00000000UL            /* Register DHR12Rx channel 1 taken as reference */
#define DAC_REG_DHR12L1_REGOFFSET      0x00100000UL            /* Register offset of DHR12Lx channel 1 versus
                                                                  DHR12Rx channel 1 (shifted left of 20 bits)   */
#define DAC_REG_DHR8R1_REGOFFSET       0x02000000UL            /* Register offset of DHR8Rx  channel 1 versus
                                                                  DHR12Rx channel 1 (shifted left of 24 bits)   */
#define DAC_REG_DHR12R2_REGOFFSET      0x30000000UL            /* Register offset of DHR12Rx channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 28 bits)   */
#define DAC_REG_DHR12L2_REGOFFSET      0x00400000UL            /* Register offset of DHR12Lx channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 20 bits)   */
#define DAC_REG_DHR8R2_REGOFFSET       0x05000000UL            /* Register offset of DHR8Rx  channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 24 bits)   */
#define DAC_REG_DHR12RX_REGOFFSET_MASK 0xF0000000UL
#define DAC_REG_DHR12LX_REGOFFSET_MASK 0x00F00000UL
#define DAC_REG_DHR8RX_REGOFFSET_MASK  0x0F000000UL
#define DAC_REG_DHRX_REGOFFSET_MASK    (DAC_REG_DHR12RX_REGOFFSET_MASK\
                                        | DAC_REG_DHR12LX_REGOFFSET_MASK | DAC_REG_DHR8RX_REGOFFSET_MASK)

#define DAC_REG_DOR1_REGOFFSET         0x00000000UL            /* Register DORx channel 1 taken as reference */
#define DAC_REG_DOR2_REGOFFSET         0x00000020UL            /* Register offset of DORx channel 1 versus
                                                                  DORx channel 2 (shifted left of 5 bits)    */
#define DAC_REG_DORX_REGOFFSET_MASK    (DAC_REG_DOR1_REGOFFSET | DAC_REG_DOR2_REGOFFSET)
#define DAC_REG_SHSR1_REGOFFSET        0x00000000UL            /* Register SHSRx channel 1 taken as reference */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define DAC_REG_SHSR2_REGOFFSET        0x00000040UL            /* Register offset of SHSRx channel 1 versus
                                                                  SHSRx channel 2 (shifted left of 6 bits)    */
#define DAC_REG_SHSRX_REGOFFSET_MASK   (DAC_REG_SHSR1_REGOFFSET | DAC_REG_SHSR2_REGOFFSET)
#else
#define DAC_REG_SHSRX_REGOFFSET_MASK   (DAC_REG_SHSR1_REGOFFSET)
#endif /* DAC_NB_OF_CHANNEL == 2 */

#define DAC_REG_DHR_REGOFFSET_MASK_POSBIT0         0x0000000FUL /* Mask of data hold registers offset (DHR12Rx,
                                                                   DHR12Lx, DHR8Rx, ...) when shifted to position 0 */
#define DAC_REG_DORX_REGOFFSET_MASK_POSBIT0        0x00000001UL /* Mask of DORx registers offset when shifted
                                                                   to position 0                                    */
#define DAC_REG_SHSRX_REGOFFSET_MASK_POSBIT0       0x00000001UL /* Mask of SHSRx registers offset when shifted
                                                                   to position 0                                    */
#define DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS           28UL  /* Position of bits register offset of DHR12Rx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 28 bits)                   */

#define DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS           20UL  /* Position of bits register offset of DHR12Lx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 20 bits)                   */
#define DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS            24UL  /* Position of bits register offset of DHR8Rx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 24 bits)                   */
#define DAC_REG_DORX_REGOFFSET_BITOFFSET_POS               5UL  /* Position of bits register offset of DORx
                                                                   channel 1 or 2 versus DORx channel 1
                                                                   (shifted left of 5 bits)                    */
#define DAC_REG_SHSRX_REGOFFSET_BITOFFSET_POS              6UL  /* Position of bits register offset of SHSRx
                                                                   channel 1 or 2 versus SHSRx channel 1
                                                                   (shifted left of 6 bits)                    */

/* DAC registers bits positions */
#define DAC_DHR12RD_DACC2DHR_BITOFFSET_POS                DAC_DHR12RD_DACC2DHR_Pos
#define DAC_DHR12LD_DACC2DHR_BITOFFSET_POS                DAC_DHR12LD_DACC2DHR_Pos
#define DAC_DHR8RD_DACC2DHR_BITOFFSET_POS                 DAC_DHR8RD_DACC2DHR_Pos

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DAC_LL_Private_Macros DAC Private Macros
  * @{
  */

/**
  * @brief  Driver macro reserved for internal use: set a pointer to
  *         a register from a register basis from which an offset
  *         is applied.
  * @param  reg Register basis from which the offset is applied.
  * @param  reg_offset Offset to be applied (unit: number of registers).
  * @retval Pointer to register address
  */
#define DAC_PTR_REG_OFFSET(reg, reg_offset)  \
  ((uint32_t *)((uint32_t) ((uint32_t)(&(reg)) + ((reg_offset) << 2UL))))

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DAC_LL_Exported_Constants LL DAC Constants
  * @{
  */

/** @defgroup DAC_LL_EC_GET_FLAG DAC flags
  * @brief    Flags defines which can be used with LL_DAC_READ_REG function.
  * @{
  */
/* DAC channel 1 flags */
#define LL_DAC_FLAG_DMAUDR1                (DAC_SR_DMAUDR1)   /*!< DAC channel 1 flag DMA underrun */
#define LL_DAC_FLAG_CAL1                   (DAC_SR_CAL_FLAG1) /*!< DAC channel 1 flag offset calibration status */
#define LL_DAC_FLAG_BWST1                  (DAC_SR_BWST1)     /*!< DAC channel 1 flag busy writing sample time */
#define LL_DAC_FLAG_DAC1RDY                (DAC_SR_DAC1RDY)   /*!< DAC channel 1 flag ready */
#define LL_DAC_FLAG_DORSTAT1               (DAC_SR_DORSTAT1)  /*!< DAC channel 1 flag output register */

/* DAC channel 2 flags */
#define LL_DAC_FLAG_DMAUDR2                (DAC_SR_DMAUDR2)   /*!< DAC channel 2 flag DMA underrun */
#define LL_DAC_FLAG_CAL2                   (DAC_SR_CAL_FLAG2) /*!< DAC channel 2 flag offset calibration status */
#define LL_DAC_FLAG_BWST2                  (DAC_SR_BWST2)     /*!< DAC channel 2 flag busy writing sample time */
#define LL_DAC_FLAG_DAC2RDY                (DAC_SR_DAC2RDY)   /*!< DAC channel 2 flag ready */
#define LL_DAC_FLAG_DORSTAT2               (DAC_SR_DORSTAT2)  /*!< DAC channel 2 flag output register */
/**
  * @}
  */

/** @defgroup DAC_LL_DMA_EN DAC channel DMA enable
  * @brief    DMA channel enable which can be used with LL_DAC_READ_REG and  LL_DAC_WRITE_REG functions.
  * @{
  */
#define LL_DAC_DMAEN1                (DAC_CR_DMAEN1) /*!< DAC channel 1 DMA enable*/
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define LL_DAC_DMAEN2                (DAC_CR_DMAEN2) /*!< DAC channel 2 DMA enable */
#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/** @defgroup DAC_LL_EC_IT DAC interruptions
  * @brief    IT defines which can be used with LL_DAC_READ_REG and  LL_DAC_WRITE_REG functions.
  * @{
  */
#define LL_DAC_IT_DMAUDRIE1                (DAC_CR_DMAUDRIE1) /*!< DAC channel 1 interruption DMA underrun */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define LL_DAC_IT_DMAUDRIE2                (DAC_CR_DMAUDRIE2) /*!< DAC channel 2 interruption DMA underrun */
#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/** @defgroup DAC_LL_EC_CHANNEL DAC channels
  * @{
  */
#define LL_DAC_CHANNEL_1    (DAC_REG_SHSR1_REGOFFSET | DAC_REG_DOR1_REGOFFSET | DAC_REG_DHR12R1_REGOFFSET  \
                             | DAC_REG_DHR12L1_REGOFFSET | DAC_REG_DHR8R1_REGOFFSET  \
                             | DAC_CR_CH1_BITOFFSET | DAC_SWTR_CH1) /*!< DAC channel 1 */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define LL_DAC_CHANNEL_2    (DAC_REG_SHSR2_REGOFFSET | DAC_REG_DOR2_REGOFFSET | DAC_REG_DHR12R2_REGOFFSET  \
                             | DAC_REG_DHR12L2_REGOFFSET | DAC_REG_DHR8R2_REGOFFSET  \
                             | DAC_CR_CH2_BITOFFSET | DAC_SWTR_CH2) /*!< DAC channel 2 */
#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/** @defgroup DAC_LL_EC_HIGH_FREQUENCY_MODE DAC high frequency interface mode
  * @brief    High frequency interface mode defines that can be used
  *           with LL_DAC_SetHighFrequencyMode and LL_DAC_GetHighFrequencyMode.
  * @{
  */
/*!< High frequency interface mode disabled */
#define LL_DAC_HIGH_FREQ_MODE_DISABLED        0x00000000UL
/*!< High frequency interface mode compatible to AHB>80MHz enabled */
#define LL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ     (DAC_MCR_HFSEL_0)
/*!< High frequency interface mode compatible to AHB>160MHz enabled */
#define LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ    (DAC_MCR_HFSEL_1)
/**
  * @}
  */

/** @defgroup DAC_LL_EC_OPERATING_MODE DAC operating mode
  * @{
  */
#define LL_DAC_MODE_NORMAL_OPERATION       0x00000000UL        /*!< DAC channel in mode normal operation */
#define LL_DAC_MODE_CALIBRATION            (DAC_CR_CEN1)       /*!< DAC channel in mode calibration */
/**
  * @}
  */

/** @defgroup DAC_LL_EC_TRIGGER_SOURCE DAC trigger source
  * @{
  */
/*!< DAC channel conversion software trigger (SW start) */
#define LL_DAC_TRIGGER_SOFTWARE     0x00000000UL

/*!< DAC channel conversion trigger from external peripheral: TIM1 TRGO. */
#define LL_DAC_TRIGGER_TIM1_TRGO    (                                                    DAC_CR_TSEL1_0 )

/*!< DAC channel conversion trigger from external peripheral: TIM2 TRGO. */
#define LL_DAC_TRIGGER_TIM2_TRGO    (                                   DAC_CR_TSEL1_1                  )


/*!< DAC channel conversion trigger from external peripheral: TIM5 TRGO. */
#define LL_DAC_TRIGGER_TIM5_TRGO    (                  DAC_CR_TSEL1_2                  | DAC_CR_TSEL1_0 )

/*!< DAC channel conversion trigger from external peripheral: TIM6 TRGO. */
#define LL_DAC_TRIGGER_TIM6_TRGO    (                  DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1                  )

/*!< DAC channel conversion trigger from external peripheral: TIM7 TRGO. */
#define LL_DAC_TRIGGER_TIM7_TRGO    (                  DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 )

/*!< DAC channel conversion trigger from external peripheral: TIM8 TRGO. */
#define LL_DAC_TRIGGER_TIM8_TRGO    ( DAC_CR_TSEL1_3                                                    )

/*!< DAC channel conversion trigger from external peripheral: TIM12 TRGO. */
#define LL_DAC_TRIGGER_TIM12_TRGO   ( DAC_CR_TSEL1_3 |                                   DAC_CR_TSEL1_0 )

/*!< DAC channel conversion trigger from external peripheral: TIM15 TRGO. */
#define LL_DAC_TRIGGER_TIM15_TRGO   ( DAC_CR_TSEL1_3 |                  DAC_CR_TSEL1_1                  )

#if defined(LPTIM1)
/*!< DAC channel conversion trigger from external peripheral: LPTIM1 CH1. */
#define LL_DAC_TRIGGER_LPTIM1_OC1   ( DAC_CR_TSEL1_3 |                  DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 )
#endif /* LPTIM1 */

/*!< DAC channel conversion trigger from external peripheral: external interrupt line 9. */
#define LL_DAC_TRIGGER_EXTI9        ( DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2                                   )


/**
  * @}
  */

/** @defgroup DAC_LL_EC_WAVE_AUTO_GENERATION_MODE DAC waveform automatic generation mode
  * @{
  */
/*!< DAC channel wave auto generation mode disabled. */
#define LL_DAC_WAVE_AUTO_GENERATION_NONE     0x00000000UL

/*!< DAC channel wave auto generation mode enabled, set generated noise waveform. */
#define LL_DAC_WAVE_AUTO_GENERATION_NOISE    (               DAC_CR_WAVE1_0)

/*!< DAC channel wave auto generation mode enabled, set generated triangle waveform. */
#define LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE (DAC_CR_WAVE1_1               )

/**
  * @}
  */

/** @defgroup DAC_LL_EC_WAVE_NOISE_LFSR_UNMASK_BITS DAC wave generation - Noise LFSR unmask bits
  * @{
  */
/*!< Noise wave generation, unmask LFSR bit0, for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BIT0      0x00000000UL

/*!< Noise wave generation, unmask LFSR bits[1:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS1_0   (                                                   DAC_CR_MAMP1_0)

/*!< Noise wave generation, unmask LFSR bits[2:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS2_0   (                                  DAC_CR_MAMP1_1                 )

/*!< Noise wave generation, unmask LFSR bits[3:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS3_0   (                                  DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/*!< Noise wave generation, unmask LFSR bits[4:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS4_0   (                 DAC_CR_MAMP1_2                                  )

/*!< Noise wave generation, unmask LFSR bits[5:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS5_0   (                 DAC_CR_MAMP1_2                  | DAC_CR_MAMP1_0)

/*!< Noise wave generation, unmask LFSR bits[6:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS6_0   (                 DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1                 )

/*!< Noise wave generation, unmask LFSR bits[7:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS7_0   (                 DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/*!< Noise wave generation, unmask LFSR bits[8:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS8_0   (DAC_CR_MAMP1_3                                                   )

/*!< Noise wave generation, unmask LFSR bits[9:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS9_0   (DAC_CR_MAMP1_3                                   | DAC_CR_MAMP1_0)

/*!< Noise wave generation, unmask LFSR bits[10:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS10_0  (DAC_CR_MAMP1_3                  | DAC_CR_MAMP1_1                 )

/*!< Noise wave generation, unmask LFSR bits[11:0], for the selected DAC channel */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS11_0  (DAC_CR_MAMP1_3                  | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/**
  * @}
  */

/** @defgroup DAC_LL_EC_WAVE_TRIANGLE_AMPLITUDE DAC wave generation - Triangle amplitude
  * @{
  */
/*!< Triangle wave generation, amplitude of 1 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_1        0x00000000UL

/*!< Triangle wave generation, amplitude of 3 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_3        (                                                   DAC_CR_MAMP1_0)

/*!< Triangle wave generation, amplitude of 7 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_7        (                                  DAC_CR_MAMP1_1                 )

/*!< Triangle wave generation, amplitude of 15 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_15       (                                  DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/*!< Triangle wave generation, amplitude of 31 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_31       (                 DAC_CR_MAMP1_2                                  )

/*!< Triangle wave generation, amplitude of 63 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_63       (                 DAC_CR_MAMP1_2                  | DAC_CR_MAMP1_0)

/*!< Triangle wave generation, amplitude of 127 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_127      (                 DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1                 )

/*!< Triangle wave generation, amplitude of 255 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_255      (                 DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/*!< Triangle wave generation, amplitude of 512 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_511      (DAC_CR_MAMP1_3                                                   )

/*!< Triangle wave generation, amplitude of 1023 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_1023     (DAC_CR_MAMP1_3                                   | DAC_CR_MAMP1_0)

/*!< Triangle wave generation, amplitude of 2047 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_2047     (DAC_CR_MAMP1_3                  | DAC_CR_MAMP1_1                 )

/*!< Triangle wave generation, amplitude of 4095 LSB of DAC output range, for the selected DAC channel */
#define LL_DAC_TRIANGLE_AMPLITUDE_4095     (DAC_CR_MAMP1_3                  | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)

/**
  * @}
  */

/** @defgroup DAC_LL_EC_OUTPUT_MODE DAC channel output mode
  * @{
  */
/*!< The selected DAC channel output is in normal mode. */
#define LL_DAC_OUTPUT_MODE_NORMAL          0x00000000UL

/*!< The selected DAC channel output is in sample-and-hold mode. Sample-and-hold mode requires an external capacitor;
refer to the description of function @ref LL_DAC_ConfigOutput() or @ref LL_DAC_SetOutputMode(). */
#define LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD (DAC_MCR_MODE1_2)
/**
  * @}
  */

/** @defgroup DAC_LL_EC_OUTPUT_BUFFER DAC channel output buffer
  * @{
  */
/*!< The selected DAC channel output is buffered: higher drive current capability,
     but also higher current consumption */
#define LL_DAC_OUTPUT_BUFFER_ENABLE        0x00000000UL

/*!< The selected DAC channel output is not buffered: lower drive current capability,
     but also lower current consumption */
#define LL_DAC_OUTPUT_BUFFER_DISABLE       (DAC_MCR_MODE1_1)
/**
  * @}
  */

/** @defgroup DAC_LL_EC_OUTPUT_CONNECTION DAC channel output connection
  * @{
  */
/*!< The selected DAC channel output is connected to an external pin.
  Note: Depending on other parameters (normal mode or sample-and-hold mode, output buffer state),
  output can also be connected to on-chip peripherals; refer to the reference manual or
  comments of function @ref LL_DAC_SetOutputConnection(). */
#define LL_DAC_OUTPUT_CONNECT_EXTERNAL     0x00000000UL

/*!< The selected DAC channel output is connected to on-chip peripherals (via internal paths).
  Note: Depending on other parameters (normal mode or sample-and-hold mode, output buffer state),
  output can also be connected to an external pin; refer to the reference manual or
  comments of function @ref LL_DAC_SetOutputConnection(). */
#define LL_DAC_OUTPUT_CONNECT_INTERNAL     (DAC_MCR_MODE1_0)
/**
  * @}
  */

/** @defgroup DAC_LL_EC_SIGNED_FORMAT DAC channel data signed format
  * @{
  */
/*!< The selected DAC channel data format is not signed */
#define LL_DAC_SIGN_FORMAT_UNSIGNED       0x00000000UL
/*!< The selected DAC channel data format is signed */
#define LL_DAC_SIGN_FORMAT_SIGNED        (DAC_MCR_SINFORMAT1)
/**
  * @}
  */

/** @defgroup DAC_LL_EC_RESOLUTION  DAC channel output resolution
  * @{
  */
#define LL_DAC_RESOLUTION_12B              0x00000000UL            /*!< DAC channel resolution 12 bits */
#define LL_DAC_RESOLUTION_8B               0x00000002UL            /*!< DAC channel resolution 8 bits */
/**
  * @}
  */

/** @defgroup DAC_LL_EC_REGISTERS  DAC registers compliant with specific purpose
  * @{
  */
/* List of DAC registers intended to be used (most commonly) with             */
/* DMA transfer.                                                              */
/* Refer to function @ref LL_DAC_DMA_GetRegAddr().                            */

/*!< DAC channel data holding register 12 bits right aligned */
#define LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED  DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS

/*!< DAC channel data holding register 12 bits left aligned */
#define LL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED   DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS

/*!< DAC channel data holding register 8 bits right aligned */
#define LL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED   DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS

/**
  * @}
  */

/** @defgroup DAC_LL_EC_HW_DELAYS  Definitions of DAC hardware constraints delays
  * @note   Only DAC peripheral HW delays are defined in DAC LL driver driver,
  *         not timeout values.
  *         For details on delays values, refer to descriptions in source code
  *         above each literal definition.
  * @{
  */

/* Delay for DAC channel voltage settling time at DAC channel startup         */
/* (transition from disabled to enabled).                                     */
/* Note: DAC channel startup time depends on board application environment:   */
/*       impedance connected to DAC channel output.                           */
/*       The delay below is specified under conditions:                       */
/*        - voltage maximum transition (lowest to highest value)              */
/*        - until voltage reaches final value +-1LSB                          */
/*        - DAC channel output buffer enabled                                 */
/*        - load impedance of 5kOhm (min), 50pF (max)                         */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tWAKEUP").                                                      */
/* Unit: us                                                                   */
/*!< Delay for DAC channel voltage settling time at DAC channel startup (transition from disabled to enabled) */
#define LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US             8UL

/* Delay for DAC channel voltage settling time.                               */
/* Note: DAC channel startup time depends on board application environment:   */
/*       impedance connected to DAC channel output.                           */
/*       The delay below is specified under conditions:                       */
/*        - voltage maximum transition (lowest to highest value)              */
/*        - until voltage reaches final value +-1LSB                          */
/*        - DAC channel output buffer enabled                                 */
/*        - load impedance of 5kOhm min, 50pF max                             */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSETTLING").                                                    */
/* Unit: us                                                                   */
#define LL_DAC_DELAY_VOLTAGE_SETTLING_US                     3UL /*!< Delay for DAC channel voltage settling time */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DAC_LL_Exported_Macros LL DAC Macros
  * @{
  */

/** @defgroup DAC_LL_EM_WRITE_READ Common write and read registers macros
  * @{
  */

/**
  * @brief  Write a value to a DAC register.
  * @param  instance DAC Instance
  * @param  reg Register to write
  * @param  value Value to write to the register
  */
#define LL_DAC_WRITE_REG(instance, reg, value) STM32_WRITE_REG((instance)->reg, (value))

/**
  * @brief  Read a value from a DAC register.
  * @param  instance DAC Instance
  * @param  reg Register to read
  * @retval Register value
  */
#define LL_DAC_READ_REG(instance, reg) STM32_READ_REG((instance)->reg)

/**
  * @}
  */

/** @defgroup DAC_LL_EM_HELPER_MACRO DAC helper macro
  * @{
  */

/**
  * @brief  Helper macro to get DAC channel number in decimal format
  *         from literals LL_DAC_CHANNEL_x.
  *         Example:
  *            LL_DAC_CHANNEL_TO_DECIMAL_NB(LL_DAC_CHANNEL_1) returns decimal number "1".
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @note   The input can be a value from functions where a channel
  *         number is returned.
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval 1...2
  */
#define LL_DAC_CHANNEL_TO_DECIMAL_NB(channel)  ((channel) & DAC_SWTR_CHX_MASK)

/**
  * @brief  Helper macro to get DAC channel in literal format LL_DAC_CHANNEL_x
  *         from number in decimal format.
  *         Example:
  *           LL_DAC_DECIMAL_NB_TO_CHANNEL(1) returns a data equivalent to "LL_DAC_CHANNEL_1".
  * @param  decimal_nb 1...2 (value "2" depending on DAC channel 2 availability)
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  */
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
#define LL_DAC_DECIMAL_NB_TO_CHANNEL(decimal_nb) \
  (((decimal_nb) == 1UL) \
   ? ( \
       LL_DAC_CHANNEL_1 \
     ) \
   : \
   (((decimal_nb) == 2UL) \
    ? ( \
        LL_DAC_CHANNEL_2 \
      ) \
    : \
    ( \
      0UL \
    ) \
   ) \
  )
#else
#define LL_DAC_DECIMAL_NB_TO_CHANNEL(decimal_nb) \
  (((decimal_nb) == 1UL) \
   ? ( \
       LL_DAC_CHANNEL_1 \
     ) \
   : \
   ( \
     0UL \
   ) \
  )
#endif /* DAC_NB_OF_CHANNEL == 2 */

/**
  * @brief  Helper macro to define the DAC conversion data full-scale digital
  *         value corresponding to the selected DAC resolution.
  * @param  dac_resolution This parameter can be one of the following values:
  *         @arg @ref LL_DAC_RESOLUTION_12B
  *         @arg @ref LL_DAC_RESOLUTION_8B
  * @note   DAC conversion data full-scale corresponds to voltage range
  *         determined by analog voltage references Vref+ and Vref-
  *         (refer to reference manual).
  * @retval DAC conversion data equivalent voltage value (unit: mVolt)
  */
#define LL_DAC_DIGITAL_SCALE(dac_resolution)  ((0x00000FFFUL) >> ((dac_resolution) << 1UL))

/**
  * @brief  Helper macro to calculate the DAC conversion data (unit: digital
  *         value) corresponding to a voltage (unit: mVolt).
  * @param  vrefanalog_voltage Analog reference voltage (unit: mV)
  * @param  dac_voltage Voltage to be generated by DAC channel
  *                         (unit: mVolt).
  * @param  dac_resolution This parameter can be one of the following values:
  *         @arg @ref LL_DAC_RESOLUTION_12B
  *         @arg @ref LL_DAC_RESOLUTION_8B
  * @note   This helper macro is intended to provide input data in voltage
  *         rather than digital value,
  *         to be used with LL DAC functions such as
  *         @ref LL_DAC_ConvertData12RightAligned().
  * @note   Analog reference voltage (Vref+) must be either known from
  *         user board environment or can be calculated using ADC measurement
  *         and ADC helper macro LL_ADC_CALC_VREFANALOG_VOLTAGE().
  * @retval DAC conversion data (unit: digital value)
  */
#define LL_DAC_CALC_VOLTAGE_TO_DATA(vrefanalog_voltage, dac_voltage, dac_resolution)  \
  ((dac_voltage) * LL_DAC_DIGITAL_SCALE(dac_resolution) / (vrefanalog_voltage))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup DAC_LL_Exported_Functions LL DAC Functions
  * @{
  */
/** @defgroup DAC_LL_EF_Channel_Configuration Configuration of DAC instance
  * @{
  */
/**
  * @brief  Set the high frequency interface mode for the selected DAC instance.
  * @rmtoll
  *  MCR      HFSEL          LL_DAC_SetHighFrequencyMode
  * @param  dacx DAC instance
  * @param  high_freq_mode This parameter can be one of the following values:
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_DISABLED
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ
  */
__STATIC_INLINE void LL_DAC_SetHighFrequencyMode(DAC_TypeDef *dacx, uint32_t high_freq_mode)
{
  STM32_MODIFY_REG(dacx->MCR, DAC_MCR_HFSEL, high_freq_mode);
}

/**
  * @brief  Get the high frequency interface mode for the selected DAC instance.
  * @rmtoll
  *  MCR      HFSEL          LL_DAC_GetHighFrequencyMode
  * @param  dacx DAC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_DISABLED
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ
  *         @arg @ref LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ
  */
__STATIC_INLINE uint32_t LL_DAC_GetHighFrequencyMode(const DAC_TypeDef *dacx)
{
  return (uint32_t)(STM32_READ_BIT(dacx->MCR, DAC_MCR_HFSEL));
}
/**
  * @}
  */

/** @defgroup DAC_LL_EF_Configuration Configuration of DAC channels
  * @{
  */

/**
  * @brief  Set the operating mode for the selected DAC channel:
  *         calibration or normal operating mode.
  * @rmtoll
  *  CR       CEN1           LL_DAC_SetMode \n
  *  CR       CEN2           LL_DAC_SetMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  channel_mode This parameter can be one of the following values:
  *         @arg @ref LL_DAC_MODE_NORMAL_OPERATION
  *         @arg @ref LL_DAC_MODE_CALIBRATION
  */
__STATIC_INLINE void LL_DAC_SetMode(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t channel_mode)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_CEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   channel_mode << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_CEN1,
                   channel_mode);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the operating mode for the selected DAC channel:
  *         calibration or normal operating mode.
  * @rmtoll
  *  CR       CEN1           LL_DAC_GetMode \n
  *  CR       CEN2           LL_DAC_GetMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_MODE_NORMAL_OPERATION
  *         @arg @ref LL_DAC_MODE_CALIBRATION
  */
__STATIC_INLINE uint32_t LL_DAC_GetMode(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_CEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_CEN1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the offset trimming value for the selected DAC channel.
  *         Trimming has an impact when output buffer is enabled
  *         and is intended to replace factory calibration default values.
  * @rmtoll
  *  CCR      OTRIM1         LL_DAC_SetTrimmingValue \n
  *  CCR      OTRIM2         LL_DAC_SetTrimmingValue
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  trimming_value Value between Min_Data=0x00 and Max_Data=0x1F
  */
__STATIC_INLINE void LL_DAC_SetTrimmingValue(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t trimming_value)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CCR,
                   DAC_CCR_OTRIM1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   trimming_value << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CCR,
                   DAC_CCR_OTRIM1,
                   trimming_value);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the offset trimming value for the selected DAC channel.
  *         Trimming has an impact when output buffer is enabled
  *         and is intended to replace factory calibration default values.
  * @rmtoll
  *  CCR      OTRIM1         LL_DAC_GetTrimmingValue \n
  *  CCR      OTRIM2         LL_DAC_GetTrimmingValue
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Trimming value between Min_Data=0x00 and Max_Data=0x1F.
  */
__STATIC_INLINE uint32_t LL_DAC_GetTrimmingValue(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CCR, DAC_CCR_OTRIM1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CCR, DAC_CCR_OTRIM1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the conversion trigger source for the selected DAC channel.
  * @rmtoll
  *  CR       TSEL1          LL_DAC_SetTriggerSource \n
  *  CR       TSEL2          LL_DAC_SetTriggerSource
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  trigger_source This parameter can be one of the following values:
  *         @arg @ref LL_DAC_TRIGGER_SOFTWARE
  *         @arg @ref LL_DAC_TRIGGER_TIM1_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM2_TRGO
  *         @arg LL_DAC_TRIGGER_TIM3_TRGO (1)
  *         @arg @ref LL_DAC_TRIGGER_TIM5_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM6_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM7_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM8_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM12_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM15_TRGO
  *         @arg LL_DAC_TRIGGER_LPTIM1_OC1 (3)
  *         @arg @ref LL_DAC_TRIGGER_EXTI9
  *         @arg LL_DAC_TRIGGER_TIM20_TRGO (2)
  *         @arg LL_DAC_TRIGGER_PLAY_OUT (2)
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx and STM32C5J1xx/5K1xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available if LPTIMER1 instance is supported
  * @note   For conversion trigger source to be effective, DAC trigger
  *         must be enabled using function @ref LL_DAC_EnableTrigger().
  * @note   To set conversion trigger source, DAC channel must be disabled.
  *         Otherwise, the setting is discarded.
  * @note   Availability of parameters of trigger sources from timer
  *         depends on timers availability on the selected device.
  */
__STATIC_INLINE void LL_DAC_SetTriggerSource(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t trigger_source)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_TSEL1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   trigger_source << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_TSEL1,
                   trigger_source);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the conversion trigger source for the selected DAC channel.
  * @rmtoll
  *  CR       TSEL1          LL_DAC_GetTriggerSource \n
  *  CR       TSEL2          LL_DAC_GetTriggerSource
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   For conversion trigger source to be effective, DAC trigger
  *         must be enabled using function @ref LL_DAC_EnableTrigger().
  * @note   Availability of parameters of trigger sources from timer
  *         depends on timers availability on the selected device.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_TRIGGER_SOFTWARE
  *         @arg @ref LL_DAC_TRIGGER_TIM1_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM2_TRGO
  *         @arg LL_DAC_TRIGGER_TIM3_TRGO (1)
  *         @arg @ref LL_DAC_TRIGGER_TIM5_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM6_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM7_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM8_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM12_TRGO
  *         @arg @ref LL_DAC_TRIGGER_TIM15_TRGO
  *         @arg LL_DAC_TRIGGER_LPTIM1_OC1 (3)
  *         @arg @ref LL_DAC_TRIGGER_EXTI9
  *         @arg LL_DAC_TRIGGER_TIM20_TRGO (2)
  *         @arg LL_DAC_TRIGGER_PLAY_OUT (2)
  *         - (1) Parameters only available on STM32C5P1xx/5Q1xx and STM32C5J1xx/5K1xx devices
  *         - (2) Parameters only available on STM32C5P1xx/5Q1xx devices
  *         - (3) Parameters only available if LPTIMER1 instance is supported
  */
__STATIC_INLINE uint32_t LL_DAC_GetTriggerSource(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_TSEL1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_TSEL1));

#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the waveform automatic generation mode
  *         for the selected DAC channel.
  * @rmtoll
  *  CR       WAVE1          LL_DAC_SetWaveAutoGeneration \n
  *  CR       WAVE2          LL_DAC_SetWaveAutoGeneration
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  wave_auto_generation This parameter can be one of the following values:
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_NONE
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_NOISE
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE
  */
__STATIC_INLINE void LL_DAC_SetWaveAutoGeneration(DAC_TypeDef *dacx, uint32_t dac_channel,
                                                  uint32_t wave_auto_generation)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_WAVE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   wave_auto_generation << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_WAVE1,
                   wave_auto_generation);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the waveform automatic generation mode
  *         for the selected DAC channel.
  * @rmtoll
  *  CR       WAVE1          LL_DAC_GetWaveAutoGeneration \n
  *  CR       WAVE2          LL_DAC_GetWaveAutoGeneration
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_NONE
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_NOISE
  *         @arg @ref LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE
  */
__STATIC_INLINE uint32_t LL_DAC_GetWaveAutoGeneration(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_WAVE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_WAVE1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the noise waveform generation for the selected DAC channel:
  *         Noise mode and parameters LFSR (linear feedback shift register).
  * @rmtoll
  *  CR       MAMP1          LL_DAC_SetWaveNoiseLFSR \n
  *  CR       MAMP2          LL_DAC_SetWaveNoiseLFSR
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  noise_lfsr_mask This parameter can be one of the following values:
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BIT0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS1_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS2_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS3_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS4_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS5_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS6_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS7_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS8_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS9_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS10_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS11_0
  * @note   For wave generation to be effective, DAC channel
  *         wave generation mode must be enabled using
  *         function @ref LL_DAC_SetWaveAutoGeneration().
  * @note   This setting can be set when the selected DAC channel is disabled
  *         (otherwise, the setting operation is ignored).
  */
__STATIC_INLINE void LL_DAC_SetWaveNoiseLFSR(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t noise_lfsr_mask)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_MAMP1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   noise_lfsr_mask << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_MAMP1,
                   noise_lfsr_mask);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the noise waveform generation for the selected DAC channel:
  *         Noise mode and parameters LFSR (linear feedback shift register).
  * @rmtoll
  *  CR       MAMP1          LL_DAC_GetWaveNoiseLFSR \n
  *  CR       MAMP2          LL_DAC_GetWaveNoiseLFSR
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BIT0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS1_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS2_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS3_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS4_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS5_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS6_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS7_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS8_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS9_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS10_0
  *         @arg @ref LL_DAC_NOISE_LFSR_UNMASK_BITS11_0
  */
__STATIC_INLINE uint32_t LL_DAC_GetWaveNoiseLFSR(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_MAMP1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_MAMP1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the triangle waveform generation for the selected DAC channel:
  *         triangle mode and amplitude.
  * @rmtoll
  *  CR       MAMP1          LL_DAC_SetWaveTriangleAmplitude \n
  *  CR       MAMP2          LL_DAC_SetWaveTriangleAmplitude
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  triangle_amplitude This parameter can be one of the following values:
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_1
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_3
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_7
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_15
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_31
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_63
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_127
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_255
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_511
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_1023
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_2047
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_4095
  * @note   For wave generation to be effective, DAC channel
  *         wave generation mode must be enabled using
  *         function @ref LL_DAC_SetWaveAutoGeneration().
  * @note   This setting can be set when the selected DAC channel is disabled
  *         (otherwise, the setting operation is ignored).
  */
__STATIC_INLINE void LL_DAC_SetWaveTriangleAmplitude(DAC_TypeDef *dacx, uint32_t dac_channel,
                                                     uint32_t triangle_amplitude)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_MAMP1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   triangle_amplitude << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->CR,
                   DAC_CR_MAMP1,
                   triangle_amplitude);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the triangle waveform generation for the selected DAC channel:
  *         triangle mode and amplitude.
  * @rmtoll
  *  CR       MAMP1          LL_DAC_GetWaveTriangleAmplitude \n
  *  CR       MAMP2          LL_DAC_GetWaveTriangleAmplitude
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_1
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_3
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_7
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_15
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_31
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_63
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_127
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_255
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_511
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_1023
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_2047
  *         @arg @ref LL_DAC_TRIANGLE_AMPLITUDE_4095
  */
__STATIC_INLINE uint32_t LL_DAC_GetWaveTriangleAmplitude(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_MAMP1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->CR, DAC_CR_MAMP1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the output for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_ConfigOutput \n
  *  CR       MODE2          LL_DAC_ConfigOutput
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  output_mode This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_MODE_NORMAL
  *         @arg @ref LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD
  * @param  output_buffer This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_ENABLE
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_DISABLE
  * @param  output_connection This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_EXTERNAL
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_INTERNAL
  * @note   This function set several features:
  *         - mode normal or sample-and-hold
  *         - buffer
  *         - connection to GPIO or internal path.
  *         These features can also be set individually using
  *         dedicated functions:
  *         - @ref LL_DAC_SetOutputBuffer()
  *         - @ref LL_DAC_SetOutputMode()
  *         - @ref LL_DAC_SetOutputConnection()
  * @note   On this STM32 series, output connection depends on output mode
  *         (normal or sample and hold) and output buffer state.
  *         - if output connection is set to internal path and output buffer
  *           is enabled (whatever output mode):
  *           output connection is also connected to GPIO pin
  *           (both connections to GPIO pin and internal path).
  *         - if output connection is set to GPIO pin, output buffer
  *           is disabled, output mode set to sample and hold:
  *           output connection is also connected to internal path
  *           (both connections to GPIO pin and internal path).
  * @note   Mode sample-and-hold requires an external capacitor
  *         to be connected between DAC channel output and ground.
  *         Capacitor value depends on load on DAC channel output and
  *         sample-and-hold timings configured.
  *         As indication, capacitor typical value is 100nF
  *         (refer to device datasheet, parameter "CSH").
  */
__STATIC_INLINE void LL_DAC_ConfigOutput(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t output_mode,
                                         uint32_t output_buffer, uint32_t output_connection)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->MCR,
                   (DAC_MCR_MODE1_2 | DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0) << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   (output_mode | output_buffer | output_connection) << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->MCR,
                   (DAC_MCR_MODE1_2 | DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0),
                   (output_mode | output_buffer | output_connection));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the output mode normal or sample-and-hold
  *         for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_SetOutputMode \n
  *  CR       MODE2          LL_DAC_SetOutputMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  output_mode This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_MODE_NORMAL
  *         @arg @ref LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD
  * @note   Mode sample-and-hold requires an external capacitor
  *         to be connected between DAC channel output and ground.
  *         Capacitor value depends on load on DAC channel output and
  *         sample-and-hold timings configured.
  *         As indication, capacitor typical value is 100nF
  *         (refer to device datasheet, parameter "CSH").
  */
__STATIC_INLINE void LL_DAC_SetOutputMode(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t output_mode)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->MCR,
                   DAC_MCR_MODE1_2 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   output_mode << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->MCR,
                   DAC_MCR_MODE1_2,
                   output_mode);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the output mode normal or sample-and-hold for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_GetOutputMode \n
  *  CR       MODE2          LL_DAC_GetOutputMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_MODE_NORMAL
  *         @arg @ref LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD
  */
__STATIC_INLINE uint32_t LL_DAC_GetOutputMode(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_2 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_2));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the output buffer for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_SetOutputBuffer \n
  *  CR       MODE2          LL_DAC_SetOutputBuffer
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  output_buffer This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_ENABLE
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_DISABLE
  * @note   On this STM32 series, when buffer is enabled, its offset can be
  *         trimmed: factory calibration default values can be
  *         replaced by user trimming values, using function
  *         @ref LL_DAC_SetTrimmingValue().
  */
__STATIC_INLINE void LL_DAC_SetOutputBuffer(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t output_buffer)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->MCR,
                   (uint32_t)DAC_MCR_MODE1_1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   output_buffer << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->MCR,
                   (uint32_t)DAC_MCR_MODE1_1,
                   output_buffer);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the output buffer state for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_GetOutputBuffer \n
  *  CR       MODE2          LL_DAC_GetOutputBuffer
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_ENABLE
  *         @arg @ref LL_DAC_OUTPUT_BUFFER_DISABLE
  */
__STATIC_INLINE uint32_t LL_DAC_GetOutputBuffer(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the output connection for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_SetOutputConnection \n
  *  CR       MODE2          LL_DAC_SetOutputConnection
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  output_connection This parameter can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_EXTERNAL
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_INTERNAL
  * @note   On this STM32 series, output connection depends on output mode (normal or
  *         sample and hold) and output buffer state.
  *         - if output connection is set to internal path and output buffer
  *           is enabled (whatever output mode):
  *           output connection is also connected to GPIO pin
  *           (both connections to GPIO pin and internal path).
  *         - if output connection is set to GPIO pin, output buffer
  *           is disabled, output mode set to sample and hold:
  *           output connection is also connected to internal path
  *           (both connections to GPIO pin and internal path).
  */
__STATIC_INLINE void LL_DAC_SetOutputConnection(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t output_connection)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->MCR,
                   (uint32_t)DAC_MCR_MODE1_0 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   output_connection << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->MCR,
                   (uint32_t)DAC_MCR_MODE1_0,
                   output_connection);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the output connection for the selected DAC channel.
  * @rmtoll
  *  CR       MODE1          LL_DAC_GetOutputConnection \n
  *  CR       MODE2          LL_DAC_GetOutputConnection
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   On this STM32 series, output connection depends on output mode (normal or
  *         sample and hold) and output buffer state.
  *         - if output connection is set to internal path and output buffer
  *           is enabled (whatever output mode):
  *           output connection is also connected to GPIO pin
  *           (both connections to GPIO pin and internal path).
  *         - if output connection is set to GPIO pin, output buffer
  *           is disabled, output mode set to sample and hold:
  *           output connection is also connected to internal path
  *           (both connections to GPIO pin and internal path).
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_EXTERNAL
  *         @arg @ref LL_DAC_OUTPUT_CONNECT_INTERNAL
  */
__STATIC_INLINE uint32_t LL_DAC_GetOutputConnection(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_0 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->MCR, (uint32_t)DAC_MCR_MODE1_0));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the sample-and-hold timing for the selected DAC channel:
  *         sample time.
  * @rmtoll
  *  SHSR1    TSAMPLE1       LL_DAC_SetSampleAndHoldSampleTime \n
  *  SHSR2    TSAMPLE2       LL_DAC_SetSampleAndHoldSampleTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  sample_time Value between Min_Data=0x000 and Max_Data=0x3FF
  * @note   Sample time must be set when DAC channel is disabled
  *         or during DAC operation when DAC channel flag BWSTx is reset,
  *         otherwise the setting is ignored.
  *         Check BWSTx flag state using function "LL_DAC_IsActiveFlag_BWSTx()".
  */
__STATIC_INLINE void LL_DAC_SetSampleAndHoldSampleTime(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t sample_time)
{
  __IO uint32_t *preg = DAC_PTR_REG_OFFSET(dacx->SHSR1, (dac_channel >> DAC_REG_SHSRX_REGOFFSET_BITOFFSET_POS)
                                           & DAC_REG_SHSRX_REGOFFSET_MASK_POSBIT0);

  STM32_MODIFY_REG(*preg, DAC_SHSR1_TSAMPLE1, sample_time);
}

/**
  * @brief  Get the sample-and-hold timing for the selected DAC channel:
  *         sample time.
  * @rmtoll
  *  SHSR1    TSAMPLE1       LL_DAC_GetSampleAndHoldSampleTime \n
  *  SHSR2    TSAMPLE2       LL_DAC_GetSampleAndHoldSampleTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Value between Min_Data=0x000 and Max_Data=0x3FF
  */
__STATIC_INLINE uint32_t LL_DAC_GetSampleAndHoldSampleTime(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
  __IO uint32_t const *preg = DAC_PTR_REG_OFFSET(dacx->SHSR1, (dac_channel >> DAC_REG_SHSRX_REGOFFSET_BITOFFSET_POS)
                                                 & DAC_REG_SHSRX_REGOFFSET_MASK_POSBIT0);

  return (uint32_t) STM32_READ_BIT(*preg, DAC_SHSR1_TSAMPLE1);
}

/**
  * @brief  Set the sample-and-hold timing for the selected DAC channel:
  *         hold time.
  * @rmtoll
  *  SHHR     THOLD1         LL_DAC_SetSampleAndHoldHoldTime \n
  *  SHHR     THOLD2         LL_DAC_SetSampleAndHoldHoldTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  hold_time Value between Min_Data=0x000 and Max_Data=0x3FF
  */
__STATIC_INLINE void LL_DAC_SetSampleAndHoldHoldTime(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t hold_time)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->SHHR,
                   DAC_SHHR_THOLD1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   hold_time << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->SHHR,
                   DAC_SHHR_THOLD1,
                   hold_time);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the sample-and-hold timing for the selected DAC channel:
  *         hold time.
  * @rmtoll
  *  SHHR     THOLD1         LL_DAC_GetSampleAndHoldHoldTime \n
  *  SHHR     THOLD2         LL_DAC_GetSampleAndHoldHoldTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Value between Min_Data=0x000 and Max_Data=0x3FF
  */
__STATIC_INLINE uint32_t LL_DAC_GetSampleAndHoldHoldTime(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->SHHR, DAC_SHHR_THOLD1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->SHHR, DAC_SHHR_THOLD1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the sample-and-hold timing for the selected DAC channel:
  *         refresh time.
  * @rmtoll
  *  SHRR     TREFRESH1      LL_DAC_SetSampleAndHoldRefreshTime \n
  *  SHRR     TREFRESH2      LL_DAC_SetSampleAndHoldRefreshTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  refresh_time Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_DAC_SetSampleAndHoldRefreshTime(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t refresh_time)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->SHRR,
                   DAC_SHRR_TREFRESH1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   refresh_time << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->SHRR,
                   DAC_SHRR_TREFRESH1,
                   refresh_time);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the sample-and-hold timing for the selected DAC channel:
  *         refresh time.
  * @rmtoll
  *  SHRR     TREFRESH1      LL_DAC_GetSampleAndHoldRefreshTime \n
  *  SHRR     TREFRESH2      LL_DAC_GetSampleAndHoldRefreshTime
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t LL_DAC_GetSampleAndHoldRefreshTime(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->SHRR, DAC_SHRR_TREFRESH1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->SHRR, DAC_SHRR_TREFRESH1));
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Set the signed format for the selected DAC channel.
  * @rmtoll
  *  MCR      SINFORMAT1     LL_DAC_SetSignedFormat \n
  *  MCR      SINFORMAT2     LL_DAC_SetSignedFormat
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  signed_format This parameter can be one of the following values:
  *         @arg @ref LL_DAC_SIGN_FORMAT_SIGNED
  *         @arg @ref LL_DAC_SIGN_FORMAT_UNSIGNED
  * @note   With signed format an offset of half the amplitude (0x800) is added because DAC output can provide
  *         only positive value.
  * @note   On this STM32 series, signed format can be used to inject
  *         Q1.15, Q1.11, Q1.7 signed format data to DAC.
  *         Ex when using 12bits data format (Q1.11 is used):
  *             0x800  outputs 0v level
  *             0xFFF  outputs mid-scale level
  *             0x000  outputs mid-scale level
  *             0x7FF  outputs full-scale level
  */
__STATIC_INLINE void LL_DAC_SetSignedFormat(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t signed_format)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_MODIFY_REG(dacx->MCR,
                   DAC_MCR_SINFORMAT1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK),
                   signed_format << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_MODIFY_REG(dacx->MCR,
                   DAC_MCR_SINFORMAT1,
                   signed_format);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get the signed format state for the selected DAC channel.
  * @rmtoll
  *  MCR      SINFORMAT1     LL_DAC_GetSignedFormat \n
  *  MCR      SINFORMAT2     LL_DAC_GetSignedFormat
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DAC_SIGN_FORMAT_SIGNED
  *         @arg @ref LL_DAC_SIGN_FORMAT_UNSIGNED
  */
__STATIC_INLINE uint32_t LL_DAC_GetSignedFormat(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return (uint32_t)(STM32_READ_BIT(dacx->MCR, DAC_MCR_SINFORMAT1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
                    >> (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return (uint32_t)(STM32_READ_BIT(dacx->MCR, DAC_MCR_SINFORMAT1));
#endif /* DAC_NB_OF_CHANNEL */

}

/**
  * @}
  */

/** @defgroup DAC_LL_EF_DMA_Management DMA Management
  * @{
  */

/**
  * @brief  Enable DAC DMA transfer request of the selected channel.
  * @rmtoll
  *  CR       DMAEN1         LL_DAC_EnableDMAReq \n
  *  CR       DMAEN2         LL_DAC_EnableDMAReq
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   To configure DMA source address (peripheral address),
  *         use function @ref LL_DAC_DMA_GetRegAddr().
  */
__STATIC_INLINE void LL_DAC_EnableDMAReq(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_SET_BIT(dacx->CR,
                DAC_CR_DMAEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_SET_BIT(dacx->CR,
                DAC_CR_DMAEN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Disable DAC DMA transfer request of the selected channel.
  * @rmtoll
  *  CR       DMAEN1         LL_DAC_DisableDMAReq \n
  *  CR       DMAEN2         LL_DAC_DisableDMAReq
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   To configure DMA source address (peripheral address),
  *         use function @ref LL_DAC_DMA_GetRegAddr().
  */
__STATIC_INLINE void LL_DAC_DisableDMAReq(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_CLEAR_BIT(dacx->CR,
                  DAC_CR_DMAEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_CLEAR_BIT(dacx->CR,
                  DAC_CR_DMAEN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC DMA transfer request state of the selected channel.
  *         (0: DAC DMA transfer request is disabled, 1: DAC DMA transfer request is enabled)
  * @rmtoll
  *  CR       DMAEN1         LL_DAC_IsDMAReqEnabled \n
  *  CR       DMAEN2         LL_DAC_IsDMAReqEnabled
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsDMAReqEnabled(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->CR,
                          DAC_CR_DMAEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (DAC_CR_DMAEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->CR,
                          DAC_CR_DMAEN1)
           == (DAC_CR_DMAEN1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Enable DAC DMA Double data mode of the selected channel.
  * @rmtoll
  *  MCR      DMADOUBLE1     LL_DAC_EnableDMADoubleDataMode \n
  *  MCR      DMADOUBLE2     LL_DAC_EnableDMADoubleDataMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  */
__STATIC_INLINE void LL_DAC_EnableDMADoubleDataMode(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_SET_BIT(dacx->MCR,
                DAC_MCR_DMADOUBLE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_SET_BIT(dacx->MCR,
                DAC_MCR_DMADOUBLE1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Disable DAC DMA Double data mode of the selected channel.
  * @rmtoll
  *  MCR      DMADOUBLE1     LL_DAC_DisableDMADoubleDataMode \n
  *  MCR      DMADOUBLE2     LL_DAC_DisableDMADoubleDataMode
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  */
__STATIC_INLINE void LL_DAC_DisableDMADoubleDataMode(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_CLEAR_BIT(dacx->MCR,
                  DAC_MCR_DMADOUBLE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_CLEAR_BIT(dacx->MCR,
                  DAC_MCR_DMADOUBLE1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC DMA double data mode state of the selected channel.
  *         (0: DAC DMA double data mode is disabled, 1: DAC DMA double data mode is enabled)
  * @rmtoll
  *  MCR      DMADOUBLE1     LL_DAC_IsDMADoubleDataModeEnabled \n
  *  MCR      DMADOUBLE2     LL_DAC_IsDMADoubleDataModeEnabled
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsDMADoubleDataModeEnabled(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->MCR,
                          DAC_MCR_DMADOUBLE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (DAC_MCR_DMADOUBLE1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->MCR,
                          DAC_MCR_DMADOUBLE1)
           == (DAC_MCR_DMADOUBLE1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Function to help to configure DMA transfer to DAC: retrieve the
  *         DAC register address from DAC instance and a list of DAC registers
  *         intended to be used (most commonly) with DMA transfer.
  * @rmtoll
  *  DHR12R1  DACC1DHR       LL_DAC_DMA_GetRegAddr \n
  *  DHR12L1  DACC1DHR       LL_DAC_DMA_GetRegAddr \n
  *  DHR8R1   DACC1DHR       LL_DAC_DMA_GetRegAddr \n
  *  DHR12R2  DACC2DHR       LL_DAC_DMA_GetRegAddr \n
  *  DHR12L2  DACC2DHR       LL_DAC_DMA_GetRegAddr \n
  *  DHR8R2   DACC2DHR       LL_DAC_DMA_GetRegAddr
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  reg_addr This parameter can be one of the following values:
  *         @arg @ref LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED
  *         @arg @ref LL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED
  *         @arg @ref LL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED
  * @note   These DAC registers are data holding registers:
  *         when DAC conversion is requested, DAC generates a DMA transfer
  *         request to have data available in DAC data holding registers.
  * @note   This macro is intended to be used with LL DMA driver, refer to
  *         function "LL_DMA_ConfigAddresses()".
  *         Example:
  *           LL_DMA_ConfigAddresses(DMA1,
  *                                  LL_DMA_CHANNEL_1,
  *                                  (uint32_t)&< array or variable >,
  *                                  LL_DAC_DMA_GetRegAddr(DAC1, LL_DAC_CHANNEL_1,
  *                                  LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED),
  *                                  LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  * @retval DAC register address
  */
__STATIC_INLINE uint32_t LL_DAC_DMA_GetRegAddr(const DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t reg_addr)
{
  /* Retrieve address of reg_addr DHR12Rx, DHR12Lx or DHR8Rx depending on */
  /* DAC channel selected. */
  return ((uint32_t)(DAC_PTR_REG_OFFSET((dacx)->DHR12R1, ((dac_channel >> (reg_addr & 0x1FUL))
                                                          & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0))));
}
/**
  * @}
  */

/** @defgroup DAC_LL_EF_Operation Operation on DAC channels
  * @{
  */

/**
  * @brief  Enable DAC selected channel.
  * @rmtoll
  *  CR       EN1            LL_DAC_Enable \n
  *  CR       EN2            LL_DAC_Enable
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   After enable from off state, DAC channel requires a delay
  *         for output voltage to reach accuracy +/- 1 LSB.
  *         Refer to device datasheet, parameter "tWAKEUP".
  */
__STATIC_INLINE void LL_DAC_Enable(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_SET_BIT(dacx->CR, DAC_CR_EN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_SET_BIT(dacx->CR, DAC_CR_EN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Disable DAC selected channel.
  * @rmtoll
  *  CR       EN1            LL_DAC_Disable \n
  *  CR       EN2            LL_DAC_Disable
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  */
__STATIC_INLINE void LL_DAC_Disable(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_CLEAR_BIT(dacx->CR, DAC_CR_EN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_CLEAR_BIT(dacx->CR, DAC_CR_EN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC enable state of the selected channel
  *         (0: DAC channel is disabled, 1: DAC channel is enabled).
  * @rmtoll
  *  CR       EN1            LL_DAC_IsEnabled \n
  *  CR       EN2            LL_DAC_IsEnabled
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsEnabled(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->CR, DAC_CR_EN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (DAC_CR_EN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->CR, DAC_CR_EN1)
           == DAC_CR_EN1) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Enable DAC dual channel.
  * @rmtoll
  *  CR       EN1            LL_DAC_DualChannelEnable \n
  *  CR       EN2            LL_DAC_DualChannelEnable
  * @param  dacx DAC instance
  * @note   After enable from off state, DAC channel requires a delay
  *         for output voltage to reach accuracy +/- 1 LSB.
  *         Refer to device datasheet, parameter "tWAKEUP".
  */
__STATIC_INLINE void LL_DAC_DualChannelEnable(DAC_TypeDef *dacx)
{
  STM32_SET_BIT(dacx->CR, (DAC_CR_EN1 | DAC_CR_EN2));
}

/**
  * @brief  Disable DAC selected DualChan channel.
  * @rmtoll
  *  CR       EN1            LL_DAC_DualChannelDisable \n
  *  CR       EN2            LL_DAC_DualChannelDisable
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_DualChannelDisable(DAC_TypeDef *dacx)
{
  STM32_CLEAR_BIT(dacx->CR, (DAC_CR_EN1 | DAC_CR_EN2));
}
#endif /* DAC_NB_OF_CHANNEL */

/**
  * @brief  Get DAC ready for conversion state of the selected channel
  *         (0: DAC channel is not ready, 1: DAC channel is ready).
  * @rmtoll
  *  SR       DAC1RDY        LL_DAC_IsReady \n
  *  SR       DAC2RDY        LL_DAC_IsReady
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsReady(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->SR,
                          DAC_SR_DAC1RDY << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (DAC_SR_DAC1RDY << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->SR,
                          DAC_SR_DAC1RDY)
           == (DAC_SR_DAC1RDY)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Enable DAC trigger of the selected channel.
  * @rmtoll
  *  CR       TEN1           LL_DAC_EnableTrigger \n
  *  CR       TEN2           LL_DAC_EnableTrigger
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   - If DAC trigger is disabled, DAC conversion is performed
  *           automatically once the data holding register is updated,
  *           using functions "LL_DAC_ConvertData{8; 12}{Right; Left} Aligned()":
  *           @ref LL_DAC_ConvertData12RightAligned(), ...
  *         - If DAC trigger is enabled, DAC conversion is performed
  *           only when a hardware of software trigger event is occurring.
  *           Select trigger source using
  *           function @ref LL_DAC_SetTriggerSource().
  */
__STATIC_INLINE void LL_DAC_EnableTrigger(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_SET_BIT(dacx->CR,
                DAC_CR_TEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_SET_BIT(dacx->CR,
                DAC_CR_TEN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Disable DAC trigger of the selected channel.
  * @rmtoll
  *  CR       TEN1           LL_DAC_DisableTrigger \n
  *  CR       TEN2           LL_DAC_DisableTrigger
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  */
__STATIC_INLINE void LL_DAC_DisableTrigger(DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  STM32_CLEAR_BIT(dacx->CR,
                  DAC_CR_TEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK));
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  STM32_CLEAR_BIT(dacx->CR,
                  DAC_CR_TEN1);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC trigger state of the selected channel.
  *         (0: DAC trigger is disabled, 1: DAC trigger is enabled)
  * @rmtoll
  *  CR       TEN1           LL_DAC_IsTriggerEnabled \n
  *  CR       TEN2           LL_DAC_IsTriggerEnabled
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsTriggerEnabled(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->CR,
                          DAC_CR_TEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (DAC_CR_TEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->CR,
                          DAC_CR_TEN1)
           == (DAC_CR_TEN1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC software trigger state of the selected channel.
  *         (0: DAC software trigger is disabled, 1: DAC software trigger is enabled)
  * @rmtoll
  *  CR       TEN1           LL_DAC_IsTriggerSWEnabled \n
  *  CR       TEN2           LL_DAC_IsTriggerSWEnabled
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval Software trigger enabled (1) or disabled (0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsTriggerSWEnabled(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((((STM32_READ_REG(dacx->CR)) & ((DAC_CR_TEN1 | DAC_CR_TSEL1) << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK)))
           == (DAC_CR_TEN1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((((STM32_READ_REG(dacx->CR)) & (DAC_CR_TEN1 | DAC_CR_TSEL1))
           == (DAC_CR_TEN1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Trig DAC conversion by software for the selected DAC channel.
  * @rmtoll
  *  SWTRGR  SWTRIG1        LL_DAC_TrigSWConversion \n
  *  SWTRGR  SWTRIG2        LL_DAC_TrigSWConversion
  * @param  dacx DAC instance
  * @param  dac_channel  This parameter can a combination of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   Preliminarily, DAC trigger must be set to software trigger
  *         using function
  *           @ref LL_DAC_SetTriggerSource()
  *         with parameter "LL_DAC_TRIGGER_SOFTWARE".
  *         and DAC trigger must be enabled using
  *         function @ref LL_DAC_EnableTrigger().
  * @note   For devices featuring DAC with 2 channels: this function
  *         can perform a SW start of both DAC channels simultaneously.
  *         Two channels can be selected as parameter.
  *         Example: (LL_DAC_CHANNEL_1 | LL_DAC_CHANNEL_2)
  */
__STATIC_INLINE void LL_DAC_TrigSWConversion(DAC_TypeDef *dacx, uint32_t dac_channel)
{
  STM32_SET_BIT(dacx->SWTRGR,
                (dac_channel & DAC_SWTR_CHX_MASK));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (LSB aligned on bit 0),
  *         for the selected DAC channel.
  * @rmtoll
  *  DHR12R1  DACC1DHR       LL_DAC_ConvertData12RightAligned \n
  *  DHR12R2  DACC2DHR       LL_DAC_ConvertData12RightAligned
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  data Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE void LL_DAC_ConvertData12RightAligned(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t data)
{
  __IO uint32_t *preg = DAC_PTR_REG_OFFSET(dacx->DHR12R1, (dac_channel >> DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS)
                                           & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  STM32_MODIFY_REG(*preg, DAC_DHR12R1_DACC1DHR, data);
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (MSB aligned on bit 15),
  *         for the selected DAC channel.
  * @rmtoll
  *  DHR12L1  DACC1DHR       LL_DAC_ConvertData12LeftAligned \n
  *  DHR12L2  DACC2DHR       LL_DAC_ConvertData12LeftAligned
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  data Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE void LL_DAC_ConvertData12LeftAligned(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t data)
{
  __IO uint32_t *preg = DAC_PTR_REG_OFFSET(dacx->DHR12R1, (dac_channel >> DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS)
                                           & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  STM32_MODIFY_REG(*preg, DAC_DHR12L1_DACC1DHR, data);
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 8 bits left alignment (LSB aligned on bit 0),
  *         for the selected DAC channel.
  * @rmtoll
  *  DHR8R1   DACC1DHR       LL_DAC_ConvertData8RightAligned \n
  *  DHR8R2   DACC2DHR       LL_DAC_ConvertData8RightAligned
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @param  data Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_DAC_ConvertData8RightAligned(DAC_TypeDef *dacx, uint32_t dac_channel, uint32_t data)
{
  __IO uint32_t *preg = DAC_PTR_REG_OFFSET(dacx->DHR12R1, (dac_channel >> DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS)
                                           & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  STM32_MODIFY_REG(*preg, DAC_DHR8R1_DACC1DHR, data);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (LSB aligned on bit 0),
  *         for both DAC channels.
  * @rmtoll
  *  DHR12RD  DACC1DHR       LL_DAC_ConvertDualData12RightAligned \n
  *  DHR12RD  DACC2DHR       LL_DAC_ConvertDualData12RightAligned
  * @param  dacx DAC instance
  * @param  data_channel1 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @param  data_channel2 Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE void LL_DAC_ConvertDualData12RightAligned(DAC_TypeDef *dacx, uint32_t data_channel1,
                                                          uint32_t data_channel2)
{
  STM32_MODIFY_REG(dacx->DHR12RD,
                   (DAC_DHR12RD_DACC2DHR | DAC_DHR12RD_DACC1DHR),
                   ((data_channel2 << DAC_DHR12RD_DACC2DHR_BITOFFSET_POS) | data_channel1));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (MSB aligned on bit 15),
  *         for both DAC channels.
  * @rmtoll
  *  DHR12LD  DACC1DHR       LL_DAC_ConvertDualData12LeftAligned \n
  *  DHR12LD  DACC2DHR       LL_DAC_ConvertDualData12LeftAligned
  * @param  dacx DAC instance
  * @param  data_channel1 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @param  data_channel2 Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE void LL_DAC_ConvertDualData12LeftAligned(DAC_TypeDef *dacx, uint32_t data_channel1,
                                                         uint32_t data_channel2)
{
  /* Note: The DAC channel 2 shift value is reduced by 4 because the data is  */
  /*       16 bits and the DAC channel 2 bit field is in the 12 MSBs.         */
  /*       The 4 LSBs must be taken into account for the shift value.             */
  STM32_MODIFY_REG(dacx->DHR12LD,
                   (DAC_DHR12LD_DACC2DHR | DAC_DHR12LD_DACC1DHR),
                   ((data_channel2 << (DAC_DHR12LD_DACC2DHR_BITOFFSET_POS - 4U)) | data_channel1));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 8 bits left alignment (LSB aligned on bit 0),
  *         for both DAC channels.
  * @rmtoll
  *  DHR8RD  DACC1DHR       LL_DAC_ConvertDualData8RightAligned \n
  *  DHR8RD  DACC2DHR       LL_DAC_ConvertDualData8RightAligned
  * @param  dacx DAC instance
  * @param  data_channel1 Value between Min_Data=0x00 and Max_Data=0xFF
  * @param  data_channel2 Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE void LL_DAC_ConvertDualData8RightAligned(DAC_TypeDef *dacx, uint32_t data_channel1,
                                                         uint32_t data_channel2)
{
  STM32_MODIFY_REG(dacx->DHR8RD,
                   (DAC_DHR8RD_DACC2DHR | DAC_DHR8RD_DACC1DHR),
                   ((data_channel2 << DAC_DHR8RD_DACC2DHR_BITOFFSET_POS) | data_channel1));
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Retrieve output data currently generated for the selected DAC channel.
  * @rmtoll
  *  DOR1     DACC1DOR       LL_DAC_RetrieveOutputData \n
  *  DOR2     DACC2DOR       LL_DAC_RetrieveOutputData
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @note   Whatever alignment and resolution settings
  *         (using functions "LL_DAC_ConvertData{8; 12}{Right; Left} Aligned()":
  *         @ref LL_DAC_ConvertData12RightAligned(), ...),
  *         output data format is 12 bits right aligned (LSB aligned on bit 0).
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t LL_DAC_RetrieveOutputData(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
  __IO uint32_t const *preg = DAC_PTR_REG_OFFSET(dacx->DOR1, (dac_channel >> DAC_REG_DORX_REGOFFSET_BITOFFSET_POS)
                                                 & DAC_REG_DORX_REGOFFSET_MASK_POSBIT0);

  return (uint16_t) STM32_READ_BIT(*preg, DAC_DOR1_DACC1DOR);
}

/**
  * @}
  */

/** @defgroup DAC_LL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Get DAC calibration offset flag for DAC channel 1.
  * @rmtoll
  *  SR       CAL_FLAG1      LL_DAC_IsActiveFlag_CAL1
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_CAL1(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_CAL1) == (LL_DAC_FLAG_CAL1)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Get DAC calibration offset flag for DAC channel 2.
  * @rmtoll
  *  SR       CAL_FLAG2      LL_DAC_IsActiveFlag_CAL2
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_CAL2(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_CAL2) == (LL_DAC_FLAG_CAL2)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Get DAC calibration offset flag for DAC channel.
  * @rmtoll
  *  SR       CAL_FLAG1       LL_DAC_IsActiveFlag_CAL
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_CAL(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_CAL1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (LL_DAC_FLAG_CAL1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_CAL1)
           == (LL_DAC_FLAG_CAL1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC busy writing sample time flag for DAC channel 1.
  * @rmtoll
  *  SR       BWST1          LL_DAC_IsActiveFlag_BWST1
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_BWST1(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_BWST1) == (LL_DAC_FLAG_BWST1)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Get DAC busy writing sample time flag for DAC channel 2.
  * @rmtoll
  *  SR       BWST2          LL_DAC_IsActiveFlag_BWST2
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_BWST2(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_BWST2) == (LL_DAC_FLAG_BWST2)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Get DAC busy writing sample time flag for DAC channel.
  * @rmtoll
  *  SR       BWST1       LL_DAC_IsActiveFlag_BWST
  * @param  dacx DAC instance
  * @param  dac_channel This parameter can be one of the following values:
  *         @arg @ref LL_DAC_CHANNEL_1
  * @if LL_DAC_CHANNEL_2
  *         @arg @ref LL_DAC_CHANNEL_2
  * @endif
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_BWST(const DAC_TypeDef *dacx, uint32_t dac_channel)
{
#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_BWST1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))
           == (LL_DAC_FLAG_BWST1 << (dac_channel & DAC_CR_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
#else
  /* Prevent unused argument(s) compilation warning */
  (void)dac_channel;

  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_BWST1)
           == (LL_DAC_FLAG_BWST1)) ? 1UL : 0UL);
#endif /* DAC_NB_OF_CHANNEL */
}

/**
  * @brief  Get DAC ready status flag for DAC channel 1.
  * @rmtoll
  *  SR       DAC1RDY        LL_DAC_IsActiveFlag_DAC1RDY
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DAC1RDY(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DAC1RDY) == (LL_DAC_FLAG_DAC1RDY)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)

/**
  * @brief  Get DAC ready status flag for DAC channel 2.
  * @rmtoll
  *  SR       DAC2RDY        LL_DAC_IsActiveFlag_DAC2RDY
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DAC2RDY(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DAC2RDY) == (LL_DAC_FLAG_DAC2RDY)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Get DAC output register status flag for DAC channel 1.
  * @rmtoll
  *  SR       DORSTAT1       LL_DAC_IsActiveFlag_DORSTAT1
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DORSTAT1(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DORSTAT1) == (LL_DAC_FLAG_DORSTAT1)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Get DAC output register status flag for DAC channel 2.
  * @rmtoll
  *  SR       DORSTAT2       LL_DAC_IsActiveFlag_DORSTAT2
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DORSTAT2(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DORSTAT2) == (LL_DAC_FLAG_DORSTAT2)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Get DAC underrun flag for DAC channel x.
  * @rmtoll
  *  SR       SR        LL_DAC_IsActiveFlag_DMAUDR
  * @param  dacx DAC instance
  * @param  flag  This parameter can be one of the following values:
  *         @arg @ref LL_DAC_FLAG_DMAUDR1
  *         @arg @ref LL_DAC_FLAG_DMAUDR2
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DMAUDR(const DAC_TypeDef *dacx, uint32_t flag)
{
  return ((STM32_READ_BIT(dacx->SR, flag) == (flag)) ? 1UL : 0UL);
}

/**
  * @brief  Get DAC underrun flag for DAC channel 1.
  * @rmtoll
  *  SR       DMAUDR1        LL_DAC_IsActiveFlag_DMAUDR1
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DMAUDR1(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DMAUDR1) == (LL_DAC_FLAG_DMAUDR1)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Get DAC underrun flag for DAC channel 2.
  * @rmtoll
  *  SR       DMAUDR2        LL_DAC_IsActiveFlag_DMAUDR2
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsActiveFlag_DMAUDR2(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->SR, LL_DAC_FLAG_DMAUDR2) == (LL_DAC_FLAG_DMAUDR2)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Clear some bits in DAC status register.
  * @rmtoll
  *  SR       DMAUDRx     LL_DAC_ClearFlag_DMAUDR
  * @param  dacx DAC instance
  * @param  flag  (bit at 1 to be reset) can be LL_DAC_FLAG_DMAUDR1 or LL_DAC_FLAG_DMAUDR2
  * @note   DMAUDRx can be DMAUDR1 or DMAUDR2
  */
__STATIC_INLINE void LL_DAC_ClearFlag_DMAUDR(DAC_TypeDef *dacx, uint32_t flag)
{
  STM32_SET_BIT(dacx->SR, flag);
}

/**
  * @brief  Clear DAC underrun flag for DAC channel 1.
  * @rmtoll
  *  SR       DMAUDR1        LL_DAC_ClearFlag_DMAUDR1
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_ClearFlag_DMAUDR1(DAC_TypeDef *dacx)
{
  STM32_SET_BIT(dacx->SR, LL_DAC_FLAG_DMAUDR1);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Clear DAC underrun flag for DAC channel 2.
  * @rmtoll
  *  SR       DMAUDR2        LL_DAC_ClearFlag_DMAUDR2
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_ClearFlag_DMAUDR2(DAC_TypeDef *dacx)
{
  STM32_SET_BIT(dacx->SR, LL_DAC_FLAG_DMAUDR2);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/** @defgroup DAC_LL_EF_IT_Management IT management
  * @{
  */

/**
  * @brief  Enable DMA "mask" interrupt (choice of channel is contained in mask).
  * @rmtoll
  *  CR       mask      LL_DAC_EnableIT_DMAUDR
  * @param  dacx DAC instance
  * @param  mask interrupt mask that specifies the DAC interrupt.
  *         This parameter can be any combination of the following values:
  *         LL_DAC_IT_DMAUDRIE1 :  DAC channel 1 DMA underrun interrupt
  *         LL_DAC_IT_DMAUDRIE2 :  DAC channel 2 DMA underrun interrupt
  */
__STATIC_INLINE void LL_DAC_EnableIT_DMAUDR(DAC_TypeDef *dacx, uint32_t mask)
{
  STM32_SET_BIT(dacx->CR, mask);
}

/**
  * @brief  Disable DMA "mask"  interrupt (choice of channel is contained in mask).
  * @rmtoll
  *  CR       mask      LL_DAC_DisableIT_DMAUDR
  * @param  dacx DAC instance
  * @param  mask interrupt mask that specifies the DAC interrupt.
  *         This parameter can be any combination of the following values:
  *         LL_DAC_IT_DMAUDRIE1 :  DAC channel 1 DMA underrun interrupt
  *         LL_DAC_IT_DMAUDRIE2 :  DAC channel 2 DMA underrun interrupt
  */
__STATIC_INLINE void LL_DAC_DisableIT_DMAUDR(DAC_TypeDef *dacx, uint32_t mask)
{
  STM32_CLEAR_BIT(dacx->CR, mask);
}

/**
  * @brief  Enable DMA underrun interrupt for DAC channel 1.
  * @rmtoll
  *  CR       DMAUDRIE1      LL_DAC_EnableIT_DMAUDR1
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_EnableIT_DMAUDR1(DAC_TypeDef *dacx)
{
  STM32_SET_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE1);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Enable DMA underrun interrupt for DAC channel 2.
  * @rmtoll
  *  CR       DMAUDRIE2      LL_DAC_EnableIT_DMAUDR2
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_EnableIT_DMAUDR2(DAC_TypeDef *dacx)
{
  STM32_SET_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE2);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Disable DMA underrun interrupt for DAC channel 1.
  * @rmtoll
  *  CR       DMAUDRIE1      LL_DAC_DisableIT_DMAUDR1
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_DisableIT_DMAUDR1(DAC_TypeDef *dacx)
{
  STM32_CLEAR_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE1);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Disable DMA underrun interrupt for DAC channel 2.
  * @rmtoll
  *  CR       DMAUDRIE2      LL_DAC_DisableIT_DMAUDR2
  * @param  dacx DAC instance
  */
__STATIC_INLINE void LL_DAC_DisableIT_DMAUDR2(DAC_TypeDef *dacx)
{
  STM32_CLEAR_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE2);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @brief  Get a specific peripheral interrupt status.
  * @rmtoll
  *  CR       DMAUDRIEx      LL_DAC_IsEnabledIT_DMAUDR
  * @param  dacx DAC instance
  * @param  mask interrupt mask that specifies the DAC interrupt.
  *         This parameter can be any combination of the following values:
  *         LL_DAC_IT_DMAUDRIE1 :  DAC channel 1 DMA underrun interrupt
  *         LL_DAC_IT_DMAUDRIE2 :  DAC channel 2 DMA underrun interrupt
  * @retval State of mask (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsEnabledIT_DMAUDR(const DAC_TypeDef *dacx, uint32_t mask)
{
  return ((STM32_READ_BIT(dacx->CR, mask) == (mask)) ? 1UL : 0UL);
}

/**
  * @brief  Get DMA underrun interrupt for DAC channel 1.
  * @rmtoll
  *  CR       DMAUDRIE1      LL_DAC_IsEnabledIT_DMAUDR1
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsEnabledIT_DMAUDR1(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE1) == (LL_DAC_IT_DMAUDRIE1)) ? 1UL : 0UL);
}

#if defined (DAC_NB_OF_CHANNEL) && (DAC_NB_OF_CHANNEL == 2)
/**
  * @brief  Get DMA underrun interrupt for DAC channel 2.
  * @rmtoll
  *  CR       DMAUDRIE2      LL_DAC_IsEnabledIT_DMAUDR2
  * @param  dacx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DAC_IsEnabledIT_DMAUDR2(const DAC_TypeDef *dacx)
{
  return ((STM32_READ_BIT(dacx->CR, LL_DAC_IT_DMAUDRIE2) == (LL_DAC_IT_DMAUDRIE2)) ? 1UL : 0UL);
}

#endif /* DAC_NB_OF_CHANNEL */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAC1 || DAC2 */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_LL_DAC_H */

