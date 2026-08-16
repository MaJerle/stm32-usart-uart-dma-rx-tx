/**
  ******************************************************************************
  * @file    stm32c5xx_ll_rtc.h
  * @brief   Header file of RTC LL module
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

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef STM32C5xx_LL_RTC_H
#define STM32C5xx_LL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx.h"

/** @addtogroup STM32C5xx_LL_Driver
  * @{
  */

#if defined(RTC)

/** @defgroup RTC_LL RTC
  * @{
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_LL_Private_Constants RTC Private Constants
  * @{
  */
/**
  * @brief Write protection defines
  */
#define RTC_WRITE_PROTECTION_DISABLE  (uint32_t)0xFF
#define RTC_WRITE_PROTECTION_ENABLE_1 (uint32_t)0xCA
#define RTC_WRITE_PROTECTION_ENABLE_2 (uint32_t)0x53

/**
  * @brief Defines used to combine date & time
  */
#define RTC_OFFSET_WEEKDAY 24U
#define RTC_OFFSET_DAY     16U
#define RTC_OFFSET_MONTH    8U
#define RTC_OFFSET_HOUR    16U
#define RTC_OFFSET_MINUTE   8U
#define RTC_OFFSET_FORMAT  24U

/**
  * @brief Defines offset between alarm A and B registers
  */
#define RTC_ALRBR_ALRAR_OFFSET       0x02U
#define RTC_ALRBSSR_ALRASSR_OFFSET   0x02U
#define RTC_ALRBBINR_ALRABINR_OFFSET 0x01U

/**
  * @brief Defines used to combine alarm mask subseconds and subseconds
  */
#define RTC_OFFSET_ALR_MASK_SUBS_SECONDS 16U
/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Constants LL RTC Constants
  * @{
  */

/** @defgroup RTC_LL_EC_ALM Alarm A and alarm B
  * @{
  */
#define LL_RTC_ALARM_A    0U /*!< Alarm A */
#define LL_RTC_ALARM_B    1U /*!< Alarm B */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMA_WEEKDAY_SELECTION RTC alarm A date weekday
  * @{
  */
#define LL_RTC_ALMA_DATEWEEKDAYSEL_DATE    0U               /*!< Alarm A Date is selected    */
#define LL_RTC_ALMA_DATEWEEKDAYSEL_WEEKDAY RTC_ALRMAR_WDSEL /*!< Alarm A WeekDay is selected */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMB_WEEKDAY_SELECTION RTC alarm B date weekday
  * @{
  */
#define LL_RTC_ALMB_DATEWEEKDAYSEL_DATE    0U               /*!< Alarm B Date is selected    */
#define LL_RTC_ALMB_DATEWEEKDAYSEL_WEEKDAY RTC_ALRMBR_WDSEL /*!< Alarm B WeekDay is selected */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIMESTAMP_FLAGS Timestamp flags defines
  * @brief    Timestamp flags defines
  * @{
  */
#define LL_RTC_SR_TSOVF RTC_SR_TSOVF /*!< Timestamp overflow event */
#define LL_RTC_SR_TSF   RTC_SR_TSF   /*!< Timestamp event          */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CLEAR_FLAG Clear flags defines
  * @brief    Flags defines which can be used with LL_RTC_WRITE_REG function
  * @{
  */
#define LL_RTC_SCR_SSRUF    RTC_SCR_CSSRUF /*!< Clear SSR underflow event      */
#define LL_RTC_SCR_TSOVF    RTC_SCR_CTSOVF /*!< Clear timestamp overflow event */
#define LL_RTC_SCR_TSF      RTC_SCR_CTSF   /*!< Clear timestamp event          */
#define LL_RTC_SCR_WUTF     RTC_SCR_CWUTF  /*!< Clear wake-up timer event      */
#define LL_RTC_SCR_ALRBF    RTC_SCR_CALRBF /*!< Clear alarm B event            */
#define LL_RTC_SCR_ALRAF    RTC_SCR_CALRAF /*!< Clear alarm A event            */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_GET_FLAG Get flags defines
  * @brief    Flags defines which can be used with LL_RTC_READ_REG function
  * @{
  */
#define LL_RTC_ICSR_RECALPF RTC_ICSR_RECALPF /*!< Recalibration pending event     */
#define LL_RTC_ICSR_INITF   RTC_ICSR_INITF   /*!< Initialization flag             */
#define LL_RTC_ICSR_RSF     RTC_ICSR_RSF     /*!< Registers synchronization event */
#define LL_RTC_ICSR_INITS   RTC_ICSR_INITS   /*!< Initialization status event     */
#define LL_RTC_ICSR_SHPF    RTC_ICSR_SHPF    /*!< Shift operation pending event   */
#define LL_RTC_ICSR_WUTWF   RTC_ICSR_WUTWF   /*!< Wake-up timer write event       */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_BCD BCD defines
  * @brief    BCD defines which can be used with LL_RTC_READ_REG and  LL_RTC_WRITE_REG functions
  * @{
  */
#define LL_RTC_ICSR_BCDU_2  RTC_ICSR_BCDU_2 /*!< 1s calendar increment is generated each time SS[9:0]  */
#define LL_RTC_ICSR_BCDU_1  RTC_ICSR_BCDU_1 /*!< 1s calendar increment is generated each time SS[8:0]  */
#define LL_RTC_ICSR_BCDU_0  RTC_ICSR_BCDU_0 /*!< 1s calendar increment is generated each time SS[7:0]  */
#define LL_RTC_ICSR_BIN_1   RTC_ICSR_BIN_1  /*!< Free-running Binary mode (BCD mode disabled)          */
#define LL_RTC_ICSR_BIN_0   RTC_ICSR_BIN_0  /*!< Free-running BCD calendar mode (Binary mode disabled) */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_IT IT defines
  * @brief    IT defines which can be  used with LL_RTC_READ_REG and  LL_RTC_WRITE_REG functions
  * @{
  */
#define LL_RTC_CR_TSIE   RTC_CR_TSIE   /*!< Timestamp on internal event enable */
#define LL_RTC_CR_WUTIE  RTC_CR_WUTIE  /*!< Wake-up timer interrupt enable     */
#define LL_RTC_CR_ALRBIE RTC_CR_ALRBIE /*!< Alarm B interrupt enable           */
#define LL_RTC_CR_ALRAIE RTC_CR_ALRAIE /*!< Alarm A interrupt enable           */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_WEEKDAY  Weekday
  * @{
  */
#define LL_RTC_WEEKDAY_MONDAY    (uint32_t)0x01 /*!< Monday    */
#define LL_RTC_WEEKDAY_TUESDAY   (uint32_t)0x02 /*!< Tuesday   */
#define LL_RTC_WEEKDAY_WEDNESDAY (uint32_t)0x03 /*!< Wednesday */
#define LL_RTC_WEEKDAY_THURSDAY  (uint32_t)0x04 /*!< Thursday  */
#define LL_RTC_WEEKDAY_FRIDAY    (uint32_t)0x05 /*!< Friday    */
#define LL_RTC_WEEKDAY_SATURDAY  (uint32_t)0x06 /*!< Saturday  */
#define LL_RTC_WEEKDAY_SUNDAY    (uint32_t)0x07 /*!< Sunday    */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_MONTH  Month
  * @{
  */
#define LL_RTC_MONTH_JANUARY   (uint32_t)0x01  /*!< January   */
#define LL_RTC_MONTH_FEBRUARY  (uint32_t)0x02  /*!< February  */
#define LL_RTC_MONTH_MARCH     (uint32_t)0x03  /*!< March     */
#define LL_RTC_MONTH_APRIL     (uint32_t)0x04  /*!< April     */
#define LL_RTC_MONTH_MAY       (uint32_t)0x05  /*!< May       */
#define LL_RTC_MONTH_JUNE      (uint32_t)0x06  /*!< June      */
#define LL_RTC_MONTH_JULY      (uint32_t)0x07  /*!< July      */
#define LL_RTC_MONTH_AUGUST    (uint32_t)0x08  /*!< August    */
#define LL_RTC_MONTH_SEPTEMBER (uint32_t)0x09  /*!< September */
#define LL_RTC_MONTH_OCTOBER   (uint32_t)0x10  /*!< October   */
#define LL_RTC_MONTH_NOVEMBER  (uint32_t)0x11  /*!< November  */
#define LL_RTC_MONTH_DECEMBER  (uint32_t)0x12  /*!< December  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_HOUR_FORMAT  Hour format
  * @{
  */
#define LL_RTC_HOUR_FORMAT_24HOUR 0U         /*!< 24 hour/day format */
#define LL_RTC_HOUR_FORMAT_AMPM   RTC_CR_FMT /*!< AM/PM hour format  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_REF_CLOCK Reference clock
  * @{
  */
#define LL_RTC_REF_CLOCK_DISABLE 0U             /*!< Reference clock detection disabled */
#define LL_RTC_REF_CLOCK_ENABLE  RTC_CR_REFCKON /*!< Reference clock detection enabled  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_SHADOW_REGISTER Shadow register
  * @{
  */
#define LL_RTC_SHADOW_REG_KEEP   0U             /*!< Bypass shadow register disabled */
#define LL_RTC_SHADOW_REG_BYPASS RTC_CR_BYPSHAD /*!< Bypass shadow register enabled  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_BKP_REGISTER Backup register for daylight saving time
  * @{
  */
#define LL_RTC_BKP_REGISTER_UNSET 0U          /*!< Daylight time change has not been performed */
#define LL_RTC_BKP_REGISTER_SET   RTC_CR_BKP  /*!< Daylight time change has been performed     */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARMOUT  Alarm output
  * @{
  */
#define LL_RTC_ALARMOUT_DISABLE 0U              /*!< Output disabled        */
#define LL_RTC_ALARMOUT_ALARM_A    RTC_CR_OSEL_0 /*!< Alarm A output enabled */
#define LL_RTC_ALARMOUT_ALARM_B    RTC_CR_OSEL_1 /*!< Alarm B output enabled */
#define LL_RTC_ALARMOUT_WAKEUP  RTC_CR_OSEL     /*!< Wake-up output enabled */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARM_OUTPUTTYPE  Alarm output type
  * @{
  */
#define LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL    0U                   /*!< RTC_ALARM is push-pull output  */
#define LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN   RTC_CR_TAMPALRM_TYPE /*!< RTC_ALARM is open-drain output */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARM_OUTPUT_PULLUP  Alarm output pull-up
  * @{
  */
#define LL_RTC_ALARM_OUTPUT_PULLUP_NONE   0U                 /*!< No pull-up is applied on TAMPALRM output */
#define LL_RTC_ALARM_OUTPUT_PULLUP_ON     RTC_CR_TAMPALRM_PU /*!< A pull-up is applied on TAMPALRM output  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARM_OUTPUT_REMAP  Alarm output remap
  * @{
  */
#define LL_RTC_ALARM_OUTPUT_REMAP_NONE  0U             /*!< RTC_OUT2 output disabled */
#define LL_RTC_ALARM_OUTPUT_REMAP_POS1  RTC_CR_OUT2EN  /*!< RTC_OUT2 output enabled  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_OUTPUTPOLARITY_PIN  Output polarity pin
  * @{
  */
#define LL_RTC_OUTPUTPOLARITY_PIN_HIGH  0U          /*!< Pin is high when selected TAMPALRM is asserted */
#define LL_RTC_OUTPUTPOLARITY_PIN_LOW   RTC_CR_POL  /*!< Pin is low when selected TAMPALRM is asserted  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIME_FORMAT Time format
  * @{
  */
#define LL_RTC_TIME_FORMAT_AM_24H     0U          /*!< AM or 24-hour format */
#define LL_RTC_TIME_FORMAT_PM         RTC_TR_PM   /*!< PM                   */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_SHIFT_SECOND  Shift second
  * @{
  */
#define LL_RTC_SHIFT_SECOND_DELAY   0U               /*!< Delay (seconds) = SUBFS / (PREDIV_S + 1)     */
#define LL_RTC_SHIFT_SECOND_ADVANCE RTC_SHIFTR_ADD1S /*!< Advance (seconds) = (1-(SUBFS/(PREDIV_S+1))) */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMA_MASK  ALARMA mask
  * @{
  */
#define LL_RTC_ALMA_MASK_NONE        0U               /*!< No masks applied on alarm A                */
#define LL_RTC_ALMA_MASK_DATEWEEKDAY RTC_ALRMAR_MSK4  /*!< Date/day ignored in alarm A comparison */
#define LL_RTC_ALMA_MASK_HOURS       RTC_ALRMAR_MSK3  /*!< Hours ignored in alarm A comparison    */
#define LL_RTC_ALMA_MASK_MINUTES     RTC_ALRMAR_MSK2  /*!< Minutes ignored in alarm A comparison  */
#define LL_RTC_ALMA_MASK_SECONDS     RTC_ALRMAR_MSK1  /*!< Seconds ignored in alarm A comparison  */
#define LL_RTC_ALMA_MASK_ALL         (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1)
/*!< Masks all */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMA_TIME_FORMAT  ALARMA time format
  * @{
  */
#define LL_RTC_ALMA_TIME_FORMAT_AM_24H  0U            /*!< AM or 24-hour format */
#define LL_RTC_ALMA_TIME_FORMAT_PM      RTC_ALRMAR_PM /*!< PM                   */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMA_AUTOCLR RTC alarm auto clear
  * @{
  */
#define LL_RTC_ALM_AUTOCLR_NO  0U              /*!< Alarm autoclear disabled */
#define LL_RTC_ALM_AUTOCLR_YES RTC_CR_ALRAFCLR /*!< Alarm autoclear enabled  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMA_SUBSECONDBIN_AUTOCLR  RTC alarm sub seconds with binary mode auto clear definitions
  * @{
  */
#define LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO  0UL                /*!< The synchronous binary counter \
                                                                    (SS[31:0] in RTC_SSR) is free-running */
#define LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES RTC_ALRMASSR_SSCLR /*!< The synchronous binary counter (SS[31:0] \
                                                                     in RTC_SSR) is running from 0xFFFF FFFF to \
                                                                     -> SS[31:0] value and is automatically \
                                                                     reloaded with 0xFFFF FFFF when reaching \
                                                                     RTC_ALRABINR -> SS[31:0] */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMB_MASK  ALARMB mask
  * @{
  */
#define LL_RTC_ALMB_MASK_NONE        0U              /*!< No masks applied on Alarm B                */
#define LL_RTC_ALMB_MASK_DATEWEEKDAY RTC_ALRMBR_MSK4 /*!< Date/day ignored in alarm B comparison */
#define LL_RTC_ALMB_MASK_HOURS       RTC_ALRMBR_MSK3 /*!< Hours ignored in alarm B comparison    */
#define LL_RTC_ALMB_MASK_MINUTES     RTC_ALRMBR_MSK2 /*!< Minutes ignored in alarm B comparison  */
#define LL_RTC_ALMB_MASK_SECONDS     RTC_ALRMBR_MSK1 /*!< Seconds ignored in alarm B comparison  */
#define LL_RTC_ALMB_MASK_ALL         (RTC_ALRMBR_MSK4 | RTC_ALRMBR_MSK3 | RTC_ALRMBR_MSK2 | RTC_ALRMBR_MSK1)
/*!< Masks all */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMB_TIME_FORMAT  ALARMB time format
  * @{
  */
#define LL_RTC_ALMB_TIME_FORMAT_AM_24H 0U        /*!< AM or 24-hour format */
#define LL_RTC_ALMB_TIME_FORMAT_PM RTC_ALRMBR_PM /*!< PM                   */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALMB_SUBSECONDBIN_AUTOCLR  Alarm sub seconds with binary mode auto clear definitions
  * @{
  */
#define LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_NO  0UL                /*!< The synchronous binary counter \
                                                                     (SS[31:0] in RTC_SSR) is free-running */
#define LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_YES RTC_ALRMBSSR_SSCLR /*!< The synchronous binary counter \
                                                                     (SS[31:0] in RTC_SSR) is running \
                                                                     from 0xFFFF FFFF to RTC_ALRABINR -> SS[31:0]\
                                                                     value and is automatically reloaded with \
                                                                     0xFFFF FFFF when reaching \
                                                                     RTC_ALRABINR -> SS[31:0] */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIMESTAMP_EDGE  Timestamp edge
  * @{
  */
#define LL_RTC_TIMESTAMP_EDGE_RISING  0U            /*!< RTC_TS input rising edge generates a time-stamp event */
#define LL_RTC_TIMESTAMP_EDGE_FALLING RTC_CR_TSEDGE /*!< RTC_TS input falling edge generates a time-stamp event */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIMESTAMP_PIN_ENABLE Timestamp pin source enable
  * @{
  */

#define LL_RTC_TIMESTAMP_PIN_DISABLE 0U         /*!< Timestamp disabled */
#define LL_RTC_TIMESTAMP_PIN_ENABLE  RTC_CR_TSE /*!< Timestamp enabled  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIMESTAMP_TAMPER_ENABLE Timestamp tamper source enable
  * @{
  */

#define LL_RTC_TIMESTAMP_TAMPER_DISABLE 0U            /*!< Disable timestamp on tamper detection event */
#define LL_RTC_TIMESTAMP_TAMPER_ENABLE  RTC_CR_TAMPTS /*!< Enable timestamp on tamper detection event  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TS_TIME_FORMAT  Timestamp time format
  * @{
  */
#define LL_RTC_TS_TIME_FORMAT_AM_24H 0U          /*!< AM or 24-hour format */
#define LL_RTC_TS_TIME_FORMAT_PM     RTC_TSTR_PM /*!< PM                   */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_WAKEUPCLOCK_DIV  Wakeup clock div
  * @{
  */
#define LL_RTC_WAKEUPCLOCK_DIV_16     0U                                    /*!< RTC/16 clock is selected */
#define LL_RTC_WAKEUPCLOCK_DIV_8      RTC_CR_WUCKSEL_0                      /*!< RTC/8 clock is selected  */
#define LL_RTC_WAKEUPCLOCK_DIV_4      RTC_CR_WUCKSEL_1                      /*!< RTC/4 clock is selected  */
#define LL_RTC_WAKEUPCLOCK_DIV_2      (RTC_CR_WUCKSEL_1 | RTC_CR_WUCKSEL_0) /*!< RTC/2 clock is selected  */
#define LL_RTC_WAKEUPCLOCK_CKSPRE     RTC_CR_WUCKSEL_2                      /*!< ck_spre (usually 1 Hz) clock is
                                                                                 selected */
#define LL_RTC_WAKEUPCLOCK_CKSPRE_WUT (RTC_CR_WUCKSEL_2 | RTC_CR_WUCKSEL_1) /*!< ck_spre (usually 1 Hz) clock is
                                                                                 selected and 2exp16 is added to the WUT
                                                                                 counter value */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TAMPER_OUTPUT Tamper event output
  * @{
  */
#define LL_RTC_OUTPUT_TAMPER_DISABLE 0U            /*!< Disable Tamper detection output */
#define LL_RTC_OUTPUT_TAMPER_ENABLE  RTC_CR_TAMPOE /*!< Enable Tamper detection output  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CALIB_FREQUENCY_OUTPUT Calibration frequency output
  * @{
  */
#define LL_RTC_CALIB_FREQUENCY_512HZ  0U           /*!< Calibration output is 512 Hz */
#define LL_RTC_CALIB_FREQUENCY_1HZ    RTC_CR_COSEL /*!< Calibration output is 1 Hz   */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CALIB_OUTPUT  Calibration output
  * @{
  */
#define LL_RTC_CALIB_OUTPUT_NONE    0U                          /*!< Calibration output disabled  */
#define LL_RTC_CALIB_OUTPUT_ENABLE  RTC_CR_COE                  /*!< Calibration output enabled with current
                                                                     configuration */
#define LL_RTC_CALIB_OUTPUT_1HZ     (RTC_CR_COE | RTC_CR_COSEL) /*!< Calibration output is 1 Hz   */
#define LL_RTC_CALIB_OUTPUT_512HZ   RTC_CR_COE                  /*!< Calibration output is 512 Hz */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CALIB_INSERTPULSE  Calibration pulse insertion
  * @{
  */
#define LL_RTC_CALIB_INSERTPULSE_NONE 0U            /*!< No RTCCLK pulses are added */
#define LL_RTC_CALIB_INSERTPULSE_SET  RTC_CALR_CALP /*!< One RTCCLK pulse is effectively inserted every 2exp11 pulses
                                                         (frequency increased by 488.5 ppm) */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CALIB_PERIOD  Calibration period
  * @{
  */
#define LL_RTC_CALIB_PERIOD_32SEC 0U              /*!< Use a 32-second calibration cycle period */
#define LL_RTC_CALIB_PERIOD_16SEC RTC_CALR_CALW16 /*!< Use a 16-second calibration cycle period */
#define LL_RTC_CALIB_PERIOD_8SEC  RTC_CALR_CALW8  /*!< Use an 8-second calibration cycle period  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_CALIB_LOWPOWER  Calibration low power
  * @{
  */
#define LL_RTC_CALIB_LOWPOWER_NONE 0U             /*!< High-consumption mode      */
#define LL_RTC_CALIB_LOWPOWER_SET  RTC_CALR_LPCAL /*!< Ultra-low consumption mode */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_BINARY_MODE  Binary mode (sub second register)
  * @{
  */
#define LL_RTC_BINARY_NONE 0U             /*!< Free-running BCD calendar mode (Binary mode disabled) */
#define LL_RTC_BINARY_ONLY RTC_ICSR_BIN_0 /*!< Free-running Binary mode (BCD mode disabled)          */
#define LL_RTC_BINARY_MIX  RTC_ICSR_BIN_1 /*!< Free-running BCD calendar and Binary mode enabled     */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_BINARY_MIX_BCDU  Calendar second incrementation in binary mix mode
  * @{
  */
#define LL_RTC_BINARY_MIX_BCDU_SHIFT RTC_ICSR_BCDU_Pos
#define LL_RTC_BINARY_MIX_BCDU_0     0U                           /*!< 1s calendar increment is generated each time
                                                                       SS[7:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_1     (0x1UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[8:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_2     (0x2UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[9:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_3     (0x3UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[10:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_4     (0x4UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[11:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_5     (0x5UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[12:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_6     (0x6UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[13:0] = 0 */
#define LL_RTC_BINARY_MIX_BCDU_7     (0x7UL << RTC_ICSR_BCDU_Pos) /*!< 1s calendar increment is generated each time
                                                                       SS[14:0] = 0 */
/**
  * @}
  */

/** @defgroup RTC_privilege_attributes_configuration_items RTC privilege attributes configuration items
  * @{
  */
#define LL_RTC_ATTR_NPRIV 0UL /*!< Non-privileged attribute */
#define LL_RTC_ATTR_PRIV  1UL /*!< Privileged attribute     */

#define LL_RTC_PRIV_ITEM_ALRAPRIV RTC_PRIVCFGR_ALRAPRIV /*!< Privilege attribute of Alarm A and underflow protection */
#define LL_RTC_PRIV_ITEM_ALRBPRIV RTC_PRIVCFGR_ALRBPRIV /*!< Privilege attribute of Alarm B protection               */
#define LL_RTC_PRIV_ITEM_WUTPRIV  RTC_PRIVCFGR_WUTPRIV  /*!< Privilege attribute of Wake-up timer protection         */
#define LL_RTC_PRIV_ITEM_TSPRIV   RTC_PRIVCFGR_TSPRIV   /*!< Privilege attribute of Timestamp protection             */
#define LL_RTC_PRIV_ITEM_CALPRIV  RTC_PRIVCFGR_CALPRIV  /*!< Privilege attribute of  Shift register, daylightcalibration
                                                          and reference clock protection                          */
#define LL_RTC_PRIV_ITEM_INITPRIV RTC_PRIVCFGR_INITPRIV /*!< Privilege attribute of Initialization protection        */
#define LL_RTC_PRIV_ITEM_PRIV     RTC_PRIVCFGR_PRIV     /*!< Privilege attribute of RTC global protection            */
#define LL_RTC_PRIV_ITEM_ALL     (RTC_PRIVCFGR_ALRAPRIV | RTC_PRIVCFGR_ALRBPRIV | RTC_PRIVCFGR_WUTPRIV  | \
                                  RTC_PRIVCFGR_TSPRIV   | RTC_PRIVCFGR_CALPRIV  | RTC_PRIVCFGR_INITPRIV | \
                                  RTC_PRIVCFGR_PRIV )  /*!< All RTC resources configuration item                    */
/*
  * @}
  */

/** @defgroup RTC_LL_EC_WAKEUP_TIMER_INTERRUPT Wakeup timer interrupt definition
  * @{
  */
#define LL_RTC_WAKEUP_TIMER_IT_DISABLE 0U           /*!< Wake-up timer interrupt disable */
#define LL_RTC_WAKEUP_TIMER_IT_ENABLE  RTC_CR_WUTIE /*!< Wake-up timer interrupt enable  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARMA_INTERRUPT Alarm A interrupt definition
  * @{
  */
#define LL_RTC_ALMA_IT_DISABLE 0U            /*!< Alarm A interrupt disable */
#define LL_RTC_ALMA_IT_ENABLE  RTC_CR_ALRAIE /*!< Alarm A interrupt enable  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_ALARMB_INTERRUPT Alarm B interrupt definition
  * @{
  */
#define LL_RTC_ALMB_IT_DISABLE 0U            /*!< Alarm B interrupt disable */
#define LL_RTC_ALMB_IT_ENABLE  RTC_CR_ALRBIE /*!< Alarm B interrupt enable  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_TIMESTAMP_INTERRUPT Timestamp interrupt definition
  * @{
  */
#define LL_RTC_TIMESTAMP_IT_DISABLE 0U          /*!< Timestamp interrupt disable */
#define LL_RTC_TIMESTAMP_IT_ENABLE  RTC_CR_TSIE /*!< Timestamp interrupt enable  */
/**
  * @}
  */

/** @defgroup RTC_LL_EC_SSRU_INTERRUPT Sub seconds register underflow interrupt definition
  * @{
  */
#define LL_RTC_SSRU_IT_DISABLE 0U            /*!< SSR underflow interrupt disable */
#define LL_RTC_SSRU_IT_ENABLE  RTC_CR_SSRUIE /*!< SSR underflow interrupt enable  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ----------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Macros LL RTC Macros
  * @{
  */

/** @defgroup RTC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RTC register.
  * @param  reg Register to be written
  * @param  value Value to be written in the register
  */
#define LL_RTC_WRITE_REG(reg, value) STM32_WRITE_REG(RTC->reg, (value))

/**
  * @brief  Read a value in RTC register.
  * @param  reg Register to be read
  * @retval Register value
  */
#define LL_RTC_READ_REG(reg) STM32_READ_REG(RTC->reg)

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Convert Convert helper Macros
  * @{
  */

/**
  * @brief  Helper macro to convert a value from 2 digit decimal format to BCD format.
  * @param  value Byte to be converted
  * @retval Converted byte
  */
#define LL_RTC_CONVERT_BIN2BCD(value) ((uint32_t)((((value) / 10U) << 4U) | ((value) % 10U)))

/**
  * @brief  Helper macro to convert a value from BCD format to 2 digit decimal format.
  * @param  value BCD value to be converted
  * @retval Converted byte
  */
#define LL_RTC_CONVERT_BCD2BIN(value) ((uint32_t)((((uint32_t)((value) & (uint32_t)0xF0U) >> \
                                                    (uint32_t)0x4U) * 10U) + ((value) & (uint32_t)0x0FU)))

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Global Global configuration helper Macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve the value of the asynchronous prescaler.
  * @param  value Value returned by @ref LL_RTC_GetPrescalers
  * @retval Value of the asynchronous prescaler
  */
#define LL_RTC_GET_ASYNCH_PRESCALER(value) (((value) & RTC_PRER_PREDIV_A) >> RTC_PRER_PREDIV_A_Pos)

/**
  * @brief  Helper macro to retrieve the value of the synchronous prescaler.
  * @param  value Value returned by @ref LL_RTC_GetPrescalers
  * @retval Value of the synchronous prescaler
  */
#define LL_RTC_GET_SYNCH_PRESCALER(value) (((value) & RTC_PRER_PREDIV_S) >> RTC_PRER_PREDIV_S_Pos)

/**
  * @}
  */

/**
  * @brief  Helper macro to retrieve the value of BCD update.
  * @param  value Value returned by @ref LL_RTC_GetConfigBinaryMode
  * @retval Value of the BCD update in mixed mode
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_0
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_1
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_2
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_3
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_4
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_5
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_6
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_7
  */
#define LL_RTC_GET_BCDU(value) (((value) & RTC_ICSR_BCDU))

/**
  * @brief  Helper macro to retrieve the mode of RTC.
  * @param  value Value returned by @ref LL_RTC_GetConfigBinaryMode
  * @retval Mode of the RTC can be one of the value:
  *         @arg @ref LL_RTC_BINARY_NONE
  *         @arg @ref LL_RTC_BINARY_ONLY
  *         @arg @ref LL_RTC_BINARY_MIX
  */
#define LL_RTC_GET_BIN(value) (((value) & RTC_ICSR_BIN))

/** @defgroup RTC_LL_EM_Calendar Calendar helper macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve the calendar format value.
  * @param  value CR register value
  * @retval Mode of the RTC can be one of the value:
  *         @arg @ref LL_RTC_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_TIME_FORMAT_PM
  */
#define LL_RTC_GET_CALENDAR_HOUR_FORMAT(value) (((value) & RTC_CR_FMT))

/**
  * @brief Helper macro to retrieve the state of the bypass shadow register.
  * @param value CR register value
  * @retval Mode of the RTC can be one of the value:
  *        @arg @ref LL_RTC_SHADOW_REG_KEEP
  *        @arg @ref LL_RTC_SHADOW_REG_BYPASS
  */
#define LL_RTC_GET_SHADOW_REG_BYPASS(value) (((value) & RTC_CR_BYPSHAD))

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Date Date helper Macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve weekday.
  * @param  rtc_date Date returned by @ref  LL_RTC_DATE_Get function
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  */
#define LL_RTC_GET_WEEKDAY(rtc_date) (((rtc_date) >> RTC_OFFSET_WEEKDAY) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Year in BCD format.
  * @param  rtc_date Value returned by @ref  LL_RTC_DATE_Get
  * @retval Year in BCD format (0x00 . . . 0x99)
  */
#define LL_RTC_GET_YEAR(rtc_date) ((rtc_date) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Month in BCD format.
  * @param  rtc_date Value returned by @ref  LL_RTC_DATE_Get
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_MONTH_JANUARY
  *         @arg @ref LL_RTC_MONTH_FEBRUARY
  *         @arg @ref LL_RTC_MONTH_MARCH
  *         @arg @ref LL_RTC_MONTH_APRIL
  *         @arg @ref LL_RTC_MONTH_MAY
  *         @arg @ref LL_RTC_MONTH_JUNE
  *         @arg @ref LL_RTC_MONTH_JULY
  *         @arg @ref LL_RTC_MONTH_AUGUST
  *         @arg @ref LL_RTC_MONTH_SEPTEMBER
  *         @arg @ref LL_RTC_MONTH_OCTOBER
  *         @arg @ref LL_RTC_MONTH_NOVEMBER
  *         @arg @ref LL_RTC_MONTH_DECEMBER
  */
#define LL_RTC_GET_MONTH(rtc_date) (((rtc_date) >>RTC_OFFSET_MONTH) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve Day in BCD format.
  * @param  rtc_date Value returned by @ref  LL_RTC_DATE_Get
  * @retval Day in BCD format (0x01 . . . 0x31)
  */
#define LL_RTC_GET_DAY(rtc_date) (((rtc_date) >>RTC_OFFSET_DAY) & 0x000000FFU)

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Time Time helper Macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve hour in BCD format.
  * @param  rtc_time RTC time returned by @ref LL_RTC_TIME_Get function
  * @retval Hours in BCD format (0x01. . .0x12 or between Min_Data=0x00 and Max_Data=0x23)
  */
#define LL_RTC_GET_HOUR(rtc_time) (((rtc_time) >> RTC_OFFSET_HOUR) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve minute in BCD format.
  * @param  rtc_time RTC time returned by @ref LL_RTC_TIME_Get function
  * @retval Minutes in BCD format (0x00. . .0x59)
  */
#define LL_RTC_GET_MINUTE(rtc_time) (((rtc_time) >> RTC_OFFSET_MINUTE) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve second in BCD format.
  * @param  rtc_time RTC time returned by @ref LL_RTC_TIME_Get function
  * @retval Seconds in BCD format (0x00. . .0x59)
  */
#define LL_RTC_GET_SECOND(rtc_time) ((rtc_time) & 0x000000FFU)

/**
  * @brief  Helper macro to retrieve format.
  * @param  rtc_time RTC time returned by @ref LL_RTC_TIME_Get function
  * @retval Format
  */
#define LL_RTC_GET_FORMAT(rtc_time) (((rtc_time) >> RTC_OFFSET_FORMAT) & 0x0000000FU)

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Output Output helper macros
  *  @{
  */

/**
  * @brief  Helper macro to retrieve the output polarity from the CR register.
  * @param  value CR register value
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_LOW
  */
#define LL_RTC_GET_OUTPUT_POLARITY(value) (((value) & RTC_CR_TAMPALRM_TYPE) >> RTC_CR_TAMPALRM_TYPE_Pos)

/**
  * @brief  Helper macro to retrieve the output type from the CR register.
  * @param  value CR register value
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  */
#define LL_RTC_GET_OUTPUT_TYPE(value) (((value) & RTC_CR_TAMPALRM_TYPE) >> RTC_CR_TAMPALRM_TYPE_Pos)

/**
  * @brief  Helper macro to retrieve the output pull-up status from the CR register.
  * @param  value CR register value
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUT_PULLUP_NONE
  *         @arg @ref LL_RTC_ALARM_OUTPUT_PULLUP_ON
  */
#define LL_RTC_GET_OUTPUT_PULLUP(value) (((value) & RTC_CR_POL) >> RTC_CR_POL_Pos)

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Alarm_Time alarm helper macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve alarm hour in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Hours in BCD format (0x01. . .0x12 or between Min_Data=0x00 and Max_Data=0x23)
  */
#define LL_RTC_GET_ALARM_HOUR(rtc_alarm_time_date) (((rtc_alarm_time_date) & \
                                                     (RTC_ALRMAR_HU_Msk | RTC_ALRMAR_HT_Msk)) >> RTC_ALRMAR_HU_Pos)

/**
  * @brief  Helper macro to retrieve alarm minute in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Minutes in BCD format (0x00. . .0x59)
  */
#define LL_RTC_GET_ALARM_MINUTE(rtc_alarm_time_date) (((rtc_alarm_time_date) & \
                                                       (RTC_ALRMAR_MNU_Msk | RTC_ALRMAR_MNT_Msk)) >>\
                                                      RTC_ALRMAR_MNU_Pos )

/**
  * @brief  Helper macro to retrieve alarm second in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Seconds in BCD format (0x00. . .0x59)
  */
#define LL_RTC_GET_ALARM_SECOND(rtc_alarm_time_date) (((rtc_alarm_time_date) & \
                                                       (RTC_ALRMAR_SU_Msk | RTC_ALRMAR_ST_Msk)) \
                                                      >> RTC_ALRMAR_SU_Pos )

/**
  * @brief  Helper macro to retrieve alarm time format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Format
  */
#define LL_RTC_GET_ALARM_FORMAT(rtc_alarm_time_date) (((rtc_alarm_time_date) & (RTC_ALRMAR_PM_Msk)))

/**
  * @brief  Helper macro to retrieve alarm day in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Day in BCD format (0x00. . .0x31)
  */
#define LL_RTC_GET_ALARM_DAY(rtc_alarm_time_date) (((rtc_alarm_time_date) & (RTC_ALRMAR_DU_Msk | RTC_ALRMAR_DT_Msk))\
                                                   >> RTC_ALRMAR_DU_Pos )

/**
  * @brief  Helper macro to retrieve alarm weekday selection in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Day weekday selection
  */
#define LL_RTC_GET_ALARM_DAY_WDAY_SEL(rtc_alarm_time_date) (((rtc_alarm_time_date) & (RTC_ALRMAR_WDSEL_Msk)))

/**
  * @brief  Helper macro to retrieve alarm mask selection in BCD format.
  * @param  rtc_alarm_time_date Alarm time and date returned
  * @retval Alarm mask selection
  */
#define LL_RTC_GET_ALARM_MASKS(rtc_alarm_time_date) ((rtc_alarm_time_date) & (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | \
                                                                              RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1))

/**
  * @brief  Helper macro to retrieve alarm subsecond  in BCD format.
  * @param  rtc_alarm_ss Alarm subseconds mask and value returned
  * @retval Alarm subseconds
  */
#define LL_RTC_ALARM_GET_SS(rtc_alarm_ss) ((rtc_alarm_ss) & 0xFFFFU)

/**
  * @brief  Helper macro to retrieve alarm subsecond mask selection in BCD format.
  * @param  rtc_alarm_ss Alarm subseconds mask and value returned
  * @retval Alarm subseconds mask selection
  */
#define LL_RTC_ALARM_GET_MASK_SS(rtc_alarm_ss) (((rtc_alarm_ss)>> RTC_OFFSET_ALR_MASK_SUBS_SECONDS) & 0xFFU)

/**
  * @brief  Helper macro to retrieve the alarm A flag.
  * @param  rtc_flags Flags retrieved from RTC_SR register
  * @retval 0U when the alarm flag A is unset
  * @retval 1U when the alarm flag A is set
  */
#define LL_RTC_ALARM_A_GET_FLAG(rtc_flags) ((((rtc_flags) & RTC_SR_ALRAF) == RTC_SR_ALRAF) ? 1U : 0U)

/**
  * @brief  Helper macro to retrieve the alarm B flag.
  * @param  rtc_flags Flags retrieved from RTC_SR register
  * @retval 0U when the alarm flag B is unset
  * @retval 1U when the alarm flag B is set
  */
#define LL_RTC_ALARM_B_GET_FLAG(rtc_flags) ((((rtc_flags) & RTC_SR_ALRBF) == RTC_SR_ALRBF) ? 1U : 0U)

/**
  * @brief  Helper macro to retrieve the wake-up timer flag.
  * @param  rtc_flags Flags retrieved from RTC_SR
  * @retval 0U when the wake-up timer flag is unset
  * @retval 1U when the wake-up timer flag is set
  */
#define LL_RTC_WAKEUP_GET_FLAG(rtc_flags) ((((rtc_flags) & RTC_SR_WUTF) == RTC_SR_WUTF) ? 1U : 0U)
/**
  * @brief  Helper macro to retrieve the timestamp flag.
  * @param  rtc_flags Timestamp flags retrieved from RTC_SR
  * @retval 0U when the timestamp is unset
  * @retval 1U when the timestamp is set
  */
#define LL_RTC_TIMESTAMP_GET_FLAG(rtc_flags) ((((rtc_flags) & RTC_SR_TSF) == RTC_SR_TSF) ? 1U : 0U)

/**
  * @brief  Helper macro to retrieve the SSR underflow flag.
  * @param  rtc_flags SSRU flags retrieved from RTC_SR
  * @retval 0U when the SRRU flag is unset
  * @retval 1U when the SRRU flag is set
  */
#define LL_RTC_SSRU_GET_FLAG(rtc_flags) ((((rtc_flags) & RTC_SR_SSRUF) == RTC_SR_SSRUF) ? 1U : 0U)

/**
  * @}
  */

/** @defgroup RTC_LL_EM_Wakeup_Time Wake-up timer helper macros
  * @{
  */

/**
  * @brief  Helper macro to retrieve the value of the wake-up auto-reload value.
  * @param  value WUTR register value
  * @retval Returned value between 0x0 and 0xFFFF
  */
#define LL_RTC_GET_WAKEUP_AUTORELOAD(value) (((value) & RTC_WUTR_WUT) >> RTC_WUTR_WUT_Pos)

/**
  * @brief  Helper macro to retrieve the value of the auto-clear value.
  * @param  value WUTR register value
  * @retval Returned value between 0x0 and 0xFFFF
  */
#define LL_RTC_GET_WAKEUP_AUTOCLEAR(value) (((value) & RTC_WUTR_WUTOCLR) >> RTC_WUTR_WUTOCLR_Pos)

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Functions LL RTC Functions
  * @{
  */

/** @defgroup RTC_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Set Hours format (24 hour/day or AM/PM hour format).
  * @rmtoll
  *  RTC_CR           FMT           LL_RTC_SetHourFormat
  * @param  HourFormat This parameter can be one of the following values:
  *         @arg @ref LL_RTC_HOUR_FORMAT_24HOUR
  *         @arg @ref LL_RTC_HOUR_FORMAT_AMPM
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_SetHourFormat(uint32_t HourFormat)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_FMT, HourFormat);
}

/**
  * @brief  Get Hours format (24 hour/day or AM/PM hour format).
  * @rmtoll
  *  RTC_CR           FMT           LL_RTC_GetHourFormat
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_HOUR_FORMAT_24HOUR
  *         @arg @ref LL_RTC_HOUR_FORMAT_AMPM
  */
__STATIC_INLINE uint32_t LL_RTC_GetHourFormat(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_FMT));
}

/**
  * @brief  Select the flag to be routed to RTC_ALARM output.
  * @rmtoll
  *  RTC_CR           OSEL          LL_RTC_SetAlarmOutEvent
  * @param  AlarmOutput This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARMOUT_DISABLE
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_A
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_B
  *         @arg @ref LL_RTC_ALARMOUT_WAKEUP
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_SetAlarmOutEvent(uint32_t AlarmOutput)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_OSEL, AlarmOutput);
}

/**
  * @brief  Get the flag to be routed to RTC_ALARM output.
  * @rmtoll
  *  RTC_CR           OSEL          LL_RTC_GetAlarmOutEvent
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALARMOUT_DISABLE
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_A
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_B
  *         @arg @ref LL_RTC_ALARMOUT_WAKEUP
  */
__STATIC_INLINE uint32_t LL_RTC_GetAlarmOutEvent(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_OSEL));
}

/**
  * @brief  Set RTC_ALARM output type (ALARM in push-pull or open-drain output).
  * @rmtoll
  *  RTC_CR           TAMPALRM_TYPE          LL_RTC_SetAlarmOutputType
  * @param  Output This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  */
__STATIC_INLINE void LL_RTC_SetAlarmOutputType(uint32_t Output)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_TAMPALRM_TYPE, Output);
}

/**
  * @brief  Get RTC_ALARM output type (ALARM in push-pull or open-drain output).
  * @rmtoll
  *  RTC_CR           TAMPALRM_TYPE          LL_RTC_GetAlarmOutputType
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  */
__STATIC_INLINE uint32_t LL_RTC_GetAlarmOutputType(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_TAMPALRM_TYPE));
}

/**
  * @brief  Enable initialization mode.
  * @rmtoll
  *  RTC_ICSR          INIT          LL_RTC_EnableInitMode
  * @note   Initialization mode is used to program time and date register (RTC_TR and RTC_DR)
  *         and prescaler register (RTC_PRER)
  *         Counters are stopped and start counting from the new value when INIT is reset
  */
__STATIC_INLINE void LL_RTC_EnableInitMode(void)
{
  /* Set the Initialization mode */
  STM32_SET_BIT(RTC->ICSR, RTC_ICSR_INIT);
}

/**
  * @brief  Disable initialization mode (Free-running mode).
  * @rmtoll
  *  RTC_ICSR          INIT          LL_RTC_DisableInitMode
  */
__STATIC_INLINE void LL_RTC_DisableInitMode(void)
{
  /* Exit Initialization mode */
  STM32_CLEAR_BIT(RTC->ICSR, RTC_ICSR_INIT);

}

/**
  * @brief  Set Binary mode (Sub Second Register).
  * @rmtoll
  *  RTC_ICSR           BIN           LL_RTC_SetBinaryMode
  * @param  BinaryMode can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_NONE
  *         @arg @ref LL_RTC_BINARY_ONLY
  *         @arg @ref LL_RTC_BINARY_MIX
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_SetBinaryMode(uint32_t BinaryMode)
{
  STM32_MODIFY_REG(RTC->ICSR, RTC_ICSR_BIN, BinaryMode);
}

/**
  * @brief  Get Binary mode (Sub Second Register).
  * @rmtoll
  *  RTC_ICSR           BIN           LL_RTC_GetBinaryMode
  * @retval This parameter can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_NONE
  *         @arg @ref LL_RTC_BINARY_ONLY
  *         @arg @ref LL_RTC_BINARY_MIX
  */
__STATIC_INLINE uint32_t LL_RTC_GetBinaryMode(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ICSR, RTC_ICSR_BIN));
}

/**
  * @brief  Set Binary Mix mode BCDU.
  * @rmtoll
  *  RTC_ICSR           BCDU          LL_RTC_SetBinMixBCDU
  * @param  BinMixBcdU can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_0
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_1
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_2
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_3
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_4
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_5
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_6
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_7
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_SetBinMixBCDU(uint32_t BinMixBcdU)
{
  STM32_MODIFY_REG(RTC->ICSR, RTC_ICSR_BCDU, BinMixBcdU);
}

/**
  * @brief  Get Binary Mix mode BCDU.
  * @rmtoll
  *  RTC_ICSR           BCDU          LL_RTC_GetBinMixBCDU
  * @retval This parameter can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_0
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_1
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_2
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_3
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_4
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_5
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_6
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_7
  */
__STATIC_INLINE uint32_t LL_RTC_GetBinMixBCDU(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ICSR, RTC_ICSR_BCDU));
}

/**
  * @brief  Set Binary mode (Sub Second Register) and Mix mode BCDU.
  * @rmtoll
  *  RTC_ICSR           BIN           LL_RTC_SetConfigBinaryMode \n
  *  RTC_ICSR           BCDU          LL_RTC_SetConfigBinaryMode
  * @param  BinaryMode can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_NONE
  *         @arg @ref LL_RTC_BINARY_ONLY
  *         @arg @ref LL_RTC_BINARY_MIX
  * @param  BinMixBcdU can be one of the following values:
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_0
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_1
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_2
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_3
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_4
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_5
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_6
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_7
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_SetConfigBinaryMode(uint32_t BinaryMode, uint32_t BinMixBcdU)
{
  STM32_MODIFY_REG(RTC->ICSR, RTC_ICSR_BIN | RTC_ICSR_BCDU, BinaryMode | BinMixBcdU);
}

/**
  * @brief  Get Binary mode (Sub Second Register) and Mix mode BCDU.
  * @rmtoll
  *  RTC_ICSR           BIN           LL_RTC_GetConfigBinaryMode \n
  *  RTC_ICSR           BCDU          LL_RTC_GetConfigBinaryMode
  * @retval  A combination one of the following values:
  *         @arg @ref LL_RTC_BINARY_NONE
  *         @arg @ref LL_RTC_BINARY_ONLY
  *         @arg @ref LL_RTC_BINARY_MIX
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_0
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_1
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_2
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_3
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_4
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_5
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_6
  *         @arg @ref LL_RTC_BINARY_MIX_BCDU_7
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE uint32_t LL_RTC_GetConfigBinaryMode(void)
{
  return (uint32_t) STM32_READ_BIT(RTC->ICSR, RTC_ICSR_BIN | RTC_ICSR_BCDU);
}

/**
  * @brief  Set Output polarity.
  * @rmtoll
  *  RTC_CR           POL           LL_RTC_SetOutputPolarity
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_LOW
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_SetOutputPolarity(uint32_t Polarity)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_POL, Polarity);
}

/**
  * @brief  Get Output polarity.
  * @rmtoll
  *  RTC_CR           POL           LL_RTC_GetOutputPolarity
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_LOW
  */
__STATIC_INLINE uint32_t LL_RTC_GetOutputPolarity(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_POL));
}

/**
  * @brief  Enable bypass of the shadow registers.
  * @rmtoll
  *  RTC_CR           BYPSHAD       LL_RTC_EnableBypassShadowReg
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   When bypass enable, Calendar values (when reading from RTC_SSR, RTC_TR, and RTC_DR) are taken
  *         directly from the calendar counters
  * @note   If the frequency of the APB clock is less than seven times the frequency of RTCCLK,
  *         BYPSHAD must be set to 1
  */
__STATIC_INLINE void LL_RTC_EnableBypassShadowReg(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_BYPSHAD);
}

/**
  * @brief  Disable bypass of the shadow registers.
  * @rmtoll
  *  RTC_CR           BYPSHAD       LL_RTC_DisableBypassShadowReg
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_DisableBypassShadowReg(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_BYPSHAD);
}

/**
  * @brief  Check if the bypass of the shadow registers is enabled or not.
  * @rmtoll
  *  RTC_CR           BYPSHAD       LL_RTC_IsEnabledBypassShadowReg
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledBypassShadowReg(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_BYPSHAD) == (RTC_CR_BYPSHAD)) ? 1UL : 0UL);
}

/**
  * @brief  Set Hours format (24 hour/day or AM/PM hour format) and bypass shadow registers.
  * @rmtoll
  *  RTC_CR           FMT           LL_RTC_SetHourFormatAndShadowRegBypass \n
  *  RTC_CR           BYPSHAD       LL_RTC_SetHourFormatAndShadowRegBypass
  * @param  HourFormat This parameter can be one of the following values:
  *         @arg @ref LL_RTC_HOUR_FORMAT_24HOUR
  *         @arg @ref LL_RTC_HOUR_FORMAT_AMPM
  * @param  Bypass This parameter can be one of the following values:
  *         @arg @ref LL_RTC_SHADOW_REG_KEEP
  *         @arg @ref LL_RTC_SHADOW_REG_BYPASS
  * @note   Bits are write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_SetHourFormatAndShadowRegBypass(uint32_t HourFormat, uint32_t Bypass)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_FMT | RTC_CR_BYPSHAD, HourFormat | Bypass);
}

/**
  * @brief  Enable RTC_REFIN reference clock detection (50 or 60 Hz).
  * @rmtoll
  *  RTC_CR           REFCKON       LL_RTC_EnableRefClock
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_EnableRefClock(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_REFCKON);
}

/**
  * @brief  Disable RTC_REFIN reference clock detection (50 or 60 Hz).
  * @rmtoll
  *  RTC_CR           REFCKON       LL_RTC_DisableRefClock
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_DisableRefClock(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_REFCKON);
}

/**
  * @brief  Check if reference clock is enabled or not.
  * @rmtoll
  *  RTC_CR           REFCKON       LL_RTC_IsEnabledRefClock
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledRefClock(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_REFCKON) == (RTC_CR_REFCKON)) ? 1UL : 0UL);
}

/**
  * @brief  Set the different prescalers factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_A      LL_RTC_SetPrescalers \n
  *  RTC_PRER         PREDIV_S      LL_RTC_SetPrescalers
  * @param  AsynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7F
  * @param  SynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7FFF
  */
__STATIC_INLINE void LL_RTC_SetPrescalers(uint32_t AsynchPrescaler, uint32_t SynchPrescaler)
{
  STM32_WRITE_REG(RTC->PRER, (SynchPrescaler | (AsynchPrescaler << RTC_PRER_PREDIV_A_Pos)));
}

/**
  * @brief  Get the different prescalers factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_A      LL_RTC_GetPrescalers \n
  *  RTC_PRER         PREDIV_S      LL_RTC_GetPrescalers
  * @retval AsynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7F and
  *         SynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7FFF
  */
__STATIC_INLINE uint32_t LL_RTC_GetPrescalers(void)
{
  return STM32_READ_REG(RTC->PRER);
}

/**
  * @brief  Set Asynchronous prescaler factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_A      LL_RTC_SetAsynchPrescaler
  * @param  AsynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7F
  */
__STATIC_INLINE void LL_RTC_SetAsynchPrescaler(uint32_t AsynchPrescaler)
{
  STM32_MODIFY_REG(RTC->PRER, RTC_PRER_PREDIV_A, AsynchPrescaler << RTC_PRER_PREDIV_A_Pos);
}

/**
  * @brief  Set Synchronous prescaler factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_S      LL_RTC_SetSynchPrescaler
  * @param  SynchPrescaler Value between Min_Data = 0 and Max_Data = 0x7FFF
  */
__STATIC_INLINE void LL_RTC_SetSynchPrescaler(uint32_t SynchPrescaler)
{
  STM32_MODIFY_REG(RTC->PRER, RTC_PRER_PREDIV_S, SynchPrescaler);
}

/**
  * @brief  Get Asynchronous prescaler factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_A      LL_RTC_GetAsynchPrescaler
  * @retval Value between Min_Data = 0 and Max_Data = 0x7F
  */
__STATIC_INLINE uint32_t LL_RTC_GetAsynchPrescaler(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->PRER, RTC_PRER_PREDIV_A) >> RTC_PRER_PREDIV_A_Pos);
}

/**
  * @brief  Get Synchronous prescaler factor.
  * @rmtoll
  *  RTC_PRER         PREDIV_S      LL_RTC_GetSynchPrescaler
  * @retval Value between Min_Data = 0 and Max_Data = 0x7FFF
  */
__STATIC_INLINE uint32_t LL_RTC_GetSynchPrescaler(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->PRER, RTC_PRER_PREDIV_S));
}

/**
  * @brief  Enable the write protection for RTC registers.
  * @rmtoll
  *  RTC_WPR          KEY           LL_RTC_EnableWriteProtection
  */
__STATIC_INLINE void LL_RTC_EnableWriteProtection(void)
{
  STM32_WRITE_REG(RTC->WPR, RTC_WRITE_PROTECTION_DISABLE);
}

/**
  * @brief  Disable the write protection for RTC registers.
  * @rmtoll
  *  RTC_WPR          KEY           LL_RTC_DisableWriteProtection
  */
__STATIC_INLINE void LL_RTC_DisableWriteProtection(void)
{
  STM32_WRITE_REG(RTC->WPR, RTC_WRITE_PROTECTION_ENABLE_1);
  STM32_WRITE_REG(RTC->WPR, RTC_WRITE_PROTECTION_ENABLE_2);
}

/**
  * @brief  Enable tamper output.
  * @rmtoll
  *  RTC_CR           TAMPOE       LL_RTC_EnableTamperOutput
  * @note   When the tamper output is enabled, all external and internal tamper flags
  *         are ORed and routed to the TAMPALRM output
  */
__STATIC_INLINE void LL_RTC_EnableTamperOutput(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_TAMPOE);
}

/**
  * @brief  Disable tamper output.
  * @rmtoll
  *  RTC_CR           TAMPOE       LL_RTC_DisableTamperOutput
  */
__STATIC_INLINE void LL_RTC_DisableTamperOutput(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_TAMPOE);
}

/**
  * @brief  Check if tamper output is enabled or not.
  * @rmtoll
  *  RTC_CR           TAMPOE       LL_RTC_IsEnabledTamperOutput
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledTamperOutput(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_TAMPOE) == (RTC_CR_TAMPOE)) ? 1UL : 0UL);
}

/**
  * @brief  Enable internal pull-up in output mode.
  * @rmtoll
  *  RTC_CR           TAMPALRM_PU       LL_RTC_EnableAlarmPullUp
  */
__STATIC_INLINE void LL_RTC_EnableAlarmPullUp(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_TAMPALRM_PU);
}

/**
  * @brief  Disable internal pull-up in output mode.
  * @rmtoll
  *  RTC_CR           TAMPALRM_PU       LL_RTC_DisableAlarmPullUp
  */
__STATIC_INLINE void LL_RTC_DisableAlarmPullUp(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_TAMPALRM_PU);
}

/**
  * @brief  Check if internal pull-up in output mode is enabled or not.
  * @rmtoll
  *  RTC_CR           TAMPALRM_PU       LL_RTC_IsEnabledAlarmPullUp
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledAlarmPullUp(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_TAMPALRM_PU) == (RTC_CR_TAMPALRM_PU)) ? 1UL : 0UL);
}

/**
  * @brief  Enable RTC_OUT2 output.
  * @rmtoll
  *  RTC_CR           OUT2EN       LL_RTC_EnableOutput2
  * @note   RTC_OUT2 mapping depends on both OSEL (@ref LL_RTC_SetAlarmOutEvent)
  *         and COE (@ref LL_RTC_CAL_SetOutputFreq) settings
  */
__STATIC_INLINE void LL_RTC_EnableOutput2(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_OUT2EN);
}

/**
  * @brief  Disable RTC_OUT2 output.
  * @rmtoll
  *  RTC_CR           OUT2EN       LL_RTC_DisableOutput2
  */
__STATIC_INLINE void LL_RTC_DisableOutput2(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_OUT2EN);
}

/**
  * @brief  Check if RTC_OUT2 output is enabled or not.
  * @rmtoll
  *  RTC_CR           OUT2EN       LL_RTC_IsEnabledOutput2
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledOutput2(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_OUT2EN) == (RTC_CR_OUT2EN)) ? 1UL : 0UL);
}


/**
  * @brief  Enable the output of the calibration signal or tampalarm signal.
  * @rmtoll
  *  RTC_CR        OUT2EN        LL_RTC_EnableOutput \n
  *  RTC_CR        TAMPOE        LL_RTC_EnableOutput \n
  *  RTC_CR        OSEL          LL_RTC_EnableOutput \n
  *  RTC_CR        COE           LL_RTC_EnableOutput \n
  *  RTC_CR        COSEL         LL_RTC_EnableOutput
  * @param  Output This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_A
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_B
  *         @arg @ref LL_RTC_ALARM_OUTPUT_REMAP_POS1
  *         @arg @ref LL_RTC_ALARMOUT_WAKEUP
  *         @arg @ref LL_RTC_OUTPUT_TAMPER_ENABLE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref LL_RTC_CALIB_OUTPUT_512HZ
  */
__STATIC_INLINE void LL_RTC_EnableOutput(uint32_t Output)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_OUT2EN | RTC_CR_TAMPOE | RTC_CR_OSEL_0 | RTC_CR_OSEL_1 | RTC_CR_COE | RTC_CR_COSEL,
                   Output);
}

/**
  * @brief  Disable the output of the calibration signal or tampalarm signal.
  * @rmtoll
  *  RTC_CR              OUT2EN        LL_RTC_DisableOutput \n
  *  RTC_CR              TAMPOE        LL_RTC_DisableOutput \n
  *  RTC_CR              OSEL          LL_RTC_DisableOutput \n
  *  RTC_CR              COE           LL_RTC_DisableOutput \n
  *  RTC_CR              COSEL         LL_RTC_DisableOutput
  */
__STATIC_INLINE void LL_RTC_DisableOutput(void)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_OUT2EN | RTC_CR_TAMPOE | RTC_CR_OSEL_0 | RTC_CR_OSEL_1 | RTC_CR_COE | RTC_CR_COSEL,
                   0U);
}

/**
  * @brief  Get the output status of the calibration signal or tampalarm signal.
  * @rmtoll
  *  RTC_CR              OUT2EN        LL_RTC_IsEnabledOutput \n
  *  RTC_CR              TAMPOE        LL_RTC_IsEnabledOutput \n
  *  RTC_CR              OSEL          LL_RTC_IsEnabledOutput \n
  *  RTC_CR              COE           LL_RTC_IsEnabledOutput \n
  *  RTC_CR              COSEL         LL_RTC_IsEnabledOutput
  * @retval 0 output disabled
  * @retval 1 output enabled
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledOutput(void)
{
  return (((STM32_READ_REG(RTC->CR) & (RTC_CR_OUT2EN | RTC_CR_TAMPOE | RTC_CR_OSEL_0 | RTC_CR_OSEL_1 | RTC_CR_COE |
                                       RTC_CR_COSEL)) == 0U) ? 0UL : 1UL);
}

/**
  * @brief  Get the output detailed status of the calibration signal or tampalarm signal.
  * @rmtoll
  *  RTC_CR              OUT2EN        LL_RTC_IsEnabledDetailedOutput \n
  *  RTC_CR              AMPOE         LL_RTC_IsEnabledDetailedOutput \n
  *  RTC_CR              OSEL          LL_RTC_IsEnabledDetailedOutput \n
  *  RTC_CR              COE           LL_RTC_IsEnabledDetailedOutput \n
  *  RTC_CR              COSEL         LL_RTC_IsEnabledDetailedOutput
  * @param  Output This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_A
  *         @arg @ref LL_RTC_ALARMOUT_ALARM_B
  *         @arg @ref LL_RTC_ALARM_OUTPUT_REMAP_POS1
  *         @arg @ref LL_RTC_ALARMOUT_WAKEUP
  *         @arg @ref LL_RTC_OUTPUT_TAMPER_ENABLE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref LL_RTC_CALIB_OUTPUT_512HZ
  * @retval 0 output disabled, 1 output enabled
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledDetailedOutput(const uint32_t Output)
{
  return (((STM32_READ_REG(RTC->CR) & (RTC_CR_OUT2EN | RTC_CR_TAMPOE | RTC_CR_OSEL_0 | RTC_CR_OSEL_1 |
                                       RTC_CR_COE | RTC_CR_COSEL)) == Output) ? 1UL : 0UL);
}

/**
  * @brief  Configure the Output polarity and pull-up.
  * @rmtoll
  *  RTC_CR           POL                      LL_RTC_ConfigTampalarm \n
  *  RTC_CR           TAMPALRM_TYPE            LL_RTC_ConfigTampalarm \n
  *  RTC_CR           TAMPALRM_PU              LL_RTC_ConfigTampalarm
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_HIGH
  *         @arg @ref LL_RTC_OUTPUTPOLARITY_PIN_LOW
  * @param  Type This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN
  *         @arg @ref LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL
  * @param  PullUp This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_OUTPUT_PULLUP_NONE
  *         @arg @ref LL_RTC_ALARM_OUTPUT_PULLUP_ON
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_ConfigTampalarm(uint32_t Polarity,
                                            uint32_t Type, uint32_t PullUp)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_POL | RTC_CR_TAMPALRM_TYPE | RTC_CR_TAMPALRM_PU, Polarity | Type | PullUp);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Time Time
  * @{
  */

/**
  * @brief  Set time format (AM/24-hour or PM notation).
  * @rmtoll
  *  RTC_TR           PM            LL_RTC_TIME_SetFormat
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref LL_RTC_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_TIME_FORMAT_PM
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  */
__STATIC_INLINE void LL_RTC_TIME_SetFormat(uint32_t TimeFormat)
{
  STM32_MODIFY_REG(RTC->TR, RTC_TR_PM, TimeFormat);
}

/**
  * @brief  Get time format (AM/24-hour or PM notation).
  * @rmtoll
  *  RTC_TR           PM            LL_RTC_TIME_GetFormat
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_TIME_FORMAT_PM
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   Read either RTC_SSR or RTC_TR locks the values in the higher-order calendar
  *         shadow registers until RTC_DR is read (LL_RTC_READ_REG(DR))
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetFormat(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TR, RTC_TR_PM));
}

/**
  * @brief  Set Hours in BCD format.
  * @rmtoll
  *  RTC_TR           HT            LL_RTC_TIME_SetHour \n
  *  RTC_TR           HU            LL_RTC_TIME_SetHour
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert hour from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_TIME_SetHour(uint32_t Hours)
{
  STM32_MODIFY_REG(RTC->TR, (RTC_TR_HT | RTC_TR_HU),
                   (((Hours & 0xF0U) << (RTC_TR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_TR_HU_Pos)));
}

/**
  * @brief  Get Hours in BCD format.
  * @rmtoll
  *  RTC_TR           HT            LL_RTC_TIME_GetHour \n
  *  RTC_TR           HU            LL_RTC_TIME_GetHour
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   Read either RTC_SSR or RTC_TR locks the values in the higher-order calendar
  *         shadow registers until RTC_DR is read (LL_RTC_READ_REG(DR))
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert hour from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetHour(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->TR, (RTC_TR_HT | RTC_TR_HU))) >> RTC_TR_HU_Pos);
}

/**
  * @brief  Set Minutes in BCD format.
  * @rmtoll
  *  RTC_TR           MNT           LL_RTC_TIME_SetMinute \n
  *  RTC_TR           MNU           LL_RTC_TIME_SetMinute
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Minutes from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_TIME_SetMinute(uint32_t Minutes)
{
  STM32_MODIFY_REG(RTC->TR, (RTC_TR_MNT | RTC_TR_MNU),
                   (((Minutes & 0xF0U) << (RTC_TR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_TR_MNU_Pos)));
}

/**
  * @brief  Get Minutes in BCD format.
  * @rmtoll
  *  RTC_TR           MNT           LL_RTC_TIME_GetMinute \n
  *  RTC_TR           MNU           LL_RTC_TIME_GetMinute
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   Read either RTC_SSR or RTC_TR locks the values in the higher-order calendar
  *         shadow registers until RTC_DR is read (LL_RTC_READ_REG(DR))
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert minute from BCD
  *         to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetMinute(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TR, (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
}

/**
  * @brief  Set Seconds in BCD format.
  * @rmtoll
  *  RTC_TR           ST            LL_RTC_TIME_SetSecond \n
  *  RTC_TR           SU            LL_RTC_TIME_SetSecond
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Seconds from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_TIME_SetSecond(uint32_t Seconds)
{
  STM32_MODIFY_REG(RTC->TR, (RTC_TR_ST | RTC_TR_SU),
                   (((Seconds & 0xF0U) << (RTC_TR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_TR_SU_Pos)));
}

/**
  * @brief  Get Seconds in BCD format.
  * @rmtoll
  *  RTC_TR           ST            LL_RTC_TIME_GetSecond \n
  *  RTC_TR           SU            LL_RTC_TIME_GetSecond
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   Read either RTC_SSR or RTC_TR locks the values in the higher-order calendar
  *         shadow registers until RTC_DR is read (LL_RTC_READ_REG(DR))
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD
  *         to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TR, (RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos);
}

/**
  * @brief  Set time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_TR           PM            LL_RTC_TIME_Config \n
  *  RTC_TR           HT            LL_RTC_TIME_Config \n
  *  RTC_TR           HU            LL_RTC_TIME_Config \n
  *  RTC_TR           MNT           LL_RTC_TIME_Config \n
  *  RTC_TR           MNU           LL_RTC_TIME_Config \n
  *  RTC_TR           ST            LL_RTC_TIME_Config \n
  *  RTC_TR           SU            LL_RTC_TIME_Config
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref LL_RTC_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnableInitMode function)
  * @note   TimeFormat and Hours must preferably follow the same format
  */
__STATIC_INLINE void LL_RTC_TIME_Config(uint32_t Format12_24, uint32_t Hours, uint32_t Minutes,
                                        uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24                                                                                    | \
         (((Hours & 0xF0U) << (RTC_TR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_TR_HU_Pos))     | \
         (((Minutes & 0xF0U) << (RTC_TR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_TR_MNU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_TR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_TR_SU_Pos));
  STM32_WRITE_REG(RTC->TR, temp);
}

/**
  * @brief  Get time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_TR           HT            LL_RTC_TIME_Get \n
  *  RTC_TR           HU            LL_RTC_TIME_Get \n
  *  RTC_TR           MNT           LL_RTC_TIME_Get \n
  *  RTC_TR           MNU           LL_RTC_TIME_Get \n
  *  RTC_TR           ST            LL_RTC_TIME_Get \n
  *  RTC_TR           SU            LL_RTC_TIME_Get
  * @retval Combination of hours, minutes and seconds (Format: 0x00HHMMSS)
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   Read either RTC_SSR or RTC_TR locks the values in the higher-order calendar
  *         shadow registers until RTC_DR is read (LL_RTC_READ_REG(DR))
  * @note   helper macros LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE and LL_RTC_GET_SECOND
  *         are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_Get(void)
{
  uint32_t temp;

  temp = STM32_READ_BIT(RTC->TR, (RTC_TR_HT | RTC_TR_HU | RTC_TR_MNT | RTC_TR_MNU | RTC_TR_ST | RTC_TR_SU));
  return (uint32_t)((((((temp & RTC_TR_HT) >> RTC_TR_HT_Pos) << 4U) | ((temp & RTC_TR_HU) >> RTC_TR_HU_Pos)) \
                     << RTC_OFFSET_HOUR) | (((((temp & RTC_TR_MNT) >> RTC_TR_MNT_Pos) << 4U) | \
                                             ((temp & RTC_TR_MNU) >> RTC_TR_MNU_Pos)) << RTC_OFFSET_MINUTE) | \
                    ((((temp & RTC_TR_ST) >> RTC_TR_ST_Pos) << 4U) | ((temp & RTC_TR_SU) >> RTC_TR_SU_Pos)));
}

/**
  * @brief  Get time (hour, minute and second) in BCD format and time format.
  * @rmtoll
  *  RTC_TR        PM            LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        HT            LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        HU            LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        MNT           LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        MNU           LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        ST            LL_RTC_TIME_GetTimeAndFormat \n
  *  RTC_TR        SU            LL_RTC_TIME_GetTimeAndFormat
  * @retval Combination of format, hours, minutes and seconds (Format: 0x0FHHMMSS)
  * @note   helper macros LL_RTC_GET_FORMAT LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE
  *         and LL_RTC_GET_SECOND are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetTimeAndFormat(void)
{
  uint32_t temp;

  temp = STM32_READ_REG(RTC->TR);
  return (uint32_t)((((temp & RTC_TR_PM) >> RTC_TR_PM_Pos) << RTC_OFFSET_FORMAT) | \
                    (((((temp & RTC_TR_HT) >> RTC_TR_HT_Pos) << 4U) | ((temp & RTC_TR_HU) >> RTC_TR_HU_Pos)) \
                     << RTC_OFFSET_HOUR) | (((((temp & RTC_TR_MNT) >> RTC_TR_MNT_Pos) << 4U) | \
                                             ((temp & RTC_TR_MNU) >> RTC_TR_MNU_Pos)) << RTC_OFFSET_MINUTE) | \
                    ((((temp & RTC_TR_ST) >> RTC_TR_ST_Pos) << 4U) | ((temp & RTC_TR_SU) >> RTC_TR_SU_Pos)));
}

/**
  * @brief  Memorize whether the daylight saving time change has been performed.
  * @rmtoll
  *  RTC_CR           BKP           LL_RTC_TIME_EnableDayLightStore
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_TIME_EnableDayLightStore(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_BKP);
}

/**
  * @brief  Disable memorization whether the daylight saving time change has been performed.
  * @rmtoll
  *  RTC_CR           BKP           LL_RTC_TIME_DisableDayLightStore
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_TIME_DisableDayLightStore(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_BKP);
}

/**
  * @brief  Check if RTC Day Light Saving stored operation is enabled or not.
  * @rmtoll
  *  RTC_CR           BKP           LL_RTC_TIME_IsEnabledDayLightStore
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_IsEnabledDayLightStore(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_BKP) == (RTC_CR_BKP)) ? 1UL : 0UL);
}

/**
  * @brief  Subtract 1 hour (winter time change).
  * @rmtoll
  *  RTC_CR           SUB1H         LL_RTC_TIME_DecHour
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_TIME_DecHour(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_SUB1H);
}

/**
  * @brief  Add 1 hour (summer time change).
  * @rmtoll
  *  RTC_CR           ADD1H         LL_RTC_TIME_IncHour
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_TIME_IncHour(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_ADD1H);
}

/**
  * @brief  Get Sub second value in the synchronous prescaler counter.
  * @rmtoll
  *  RTC_SSR          SS            LL_RTC_TIME_GetSubSecond
  * @retval If binary mode is none, Value between Min_Data=0x0 and Max_Data=0x7FFF
  *         else Value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  * @note   Use both SubSeconds value and SecondFraction (PREDIV_S through the
  *         LL_RTC_GetSynchPrescaler function) terms returned to convert the calendar
  *         SubSeconds value to a second fraction ratio with the following time unit formula:
  *         ==> Seconds fraction ratio * time_unit= [(SecondFraction-SubSeconds)/(SecondFraction+1)] * time_unit
  *         This conversion can be performed only if no shift operation is pending (ie. SHFP=0) when PREDIV_S >= SS
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_GetSubSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->SSR, RTC_SSR_SS));
}

/**
  * @brief  Synchronize to a remote clock with a high degree of precision.
  * @rmtoll
  *  RTC_SHIFTR       ADD1S         LL_RTC_TIME_Synchronize \n
  *  RTC_SHIFTR       SUBFS         LL_RTC_TIME_Synchronize
  * @param  ShiftSecond This parameter can be one of the following values:
  *         @arg @ref LL_RTC_SHIFT_SECOND_DELAY
  *         @arg @ref LL_RTC_SHIFT_SECOND_ADVANCE
  * @param  Fraction Number of Seconds Fractions (any value from 0 to 0x7FFF)
  * @note   This operation effectively subtracts from (delays) or advance the clock of a fraction of a second
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  * @note   When REFCKON is set, firmware must not write to Shift control register
  */
__STATIC_INLINE void LL_RTC_TIME_Synchronize(uint32_t ShiftSecond, uint32_t Fraction)
{
  STM32_WRITE_REG(RTC->SHIFTR, ShiftSecond | Fraction);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Date Date
  * @{
  */

/**
  * @brief  Set Year in BCD format.
  * @rmtoll
  *  RTC_DR           YT            LL_RTC_DATE_SetYear \n
  *  RTC_DR           YU            LL_RTC_DATE_SetYear
  * @param  Year Value between Min_Data=0x00 and Max_Data=0x99
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Year from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_DATE_SetYear(uint32_t Year)
{
  STM32_MODIFY_REG(RTC->DR, (RTC_DR_YT | RTC_DR_YU),
                   (((Year & 0xF0U) << (RTC_DR_YT_Pos - 4U)) | ((Year & 0x0FU) << RTC_DR_YU_Pos)));
}

/**
  * @brief  Get Year in BCD format.
  * @rmtoll
  *  RTC_DR           YT            LL_RTC_DATE_GetYear \n
  *  RTC_DR           YU            LL_RTC_DATE_GetYear
  * @retval Value between Min_Data=0x00 and Max_Data=0x99
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Year from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_DATE_GetYear(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->DR, (RTC_DR_YT | RTC_DR_YU))) >> RTC_DR_YU_Pos);
}

/**
  * @brief  Set weekday.
  * @rmtoll
  *  RTC_DR           WDU           LL_RTC_DATE_SetWeekDay
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE void LL_RTC_DATE_SetWeekDay(uint32_t WeekDay)
{
  STM32_MODIFY_REG(RTC->DR, RTC_DR_WDU, WeekDay << RTC_DR_WDU_Pos);
}

/**
  * @brief  Get weekday.
  * @rmtoll
  *  RTC_DR           WDU           LL_RTC_DATE_GetWeekDay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  */
__STATIC_INLINE uint32_t LL_RTC_DATE_GetWeekDay(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->DR, RTC_DR_WDU) >> RTC_DR_WDU_Pos);
}

/**
  * @brief  Set Month in BCD format.
  * @rmtoll
  *  RTC_DR           MT            LL_RTC_DATE_SetMonth \n
  *  RTC_DR           MU            LL_RTC_DATE_SetMonth
  * @param  Month This parameter can be one of the following values:
  *         @arg @ref LL_RTC_MONTH_JANUARY
  *         @arg @ref LL_RTC_MONTH_FEBRUARY
  *         @arg @ref LL_RTC_MONTH_MARCH
  *         @arg @ref LL_RTC_MONTH_APRIL
  *         @arg @ref LL_RTC_MONTH_MAY
  *         @arg @ref LL_RTC_MONTH_JUNE
  *         @arg @ref LL_RTC_MONTH_JULY
  *         @arg @ref LL_RTC_MONTH_AUGUST
  *         @arg @ref LL_RTC_MONTH_SEPTEMBER
  *         @arg @ref LL_RTC_MONTH_OCTOBER
  *         @arg @ref LL_RTC_MONTH_NOVEMBER
  *         @arg @ref LL_RTC_MONTH_DECEMBER
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Month from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_DATE_SetMonth(uint32_t Month)
{
  STM32_MODIFY_REG(RTC->DR, (RTC_DR_MT | RTC_DR_MU),
                   (((Month & 0xF0U) << (RTC_DR_MT_Pos - 4U)) | ((Month & 0x0FU) << RTC_DR_MU_Pos)));
}

/**
  * @brief  Get Month in BCD format.
  * @rmtoll
  *  RTC_DR           MT            LL_RTC_DATE_GetMonth \n
  *  RTC_DR           MU            LL_RTC_DATE_GetMonth
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_MONTH_JANUARY
  *         @arg @ref LL_RTC_MONTH_FEBRUARY
  *         @arg @ref LL_RTC_MONTH_MARCH
  *         @arg @ref LL_RTC_MONTH_APRIL
  *         @arg @ref LL_RTC_MONTH_MAY
  *         @arg @ref LL_RTC_MONTH_JUNE
  *         @arg @ref LL_RTC_MONTH_JULY
  *         @arg @ref LL_RTC_MONTH_AUGUST
  *         @arg @ref LL_RTC_MONTH_SEPTEMBER
  *         @arg @ref LL_RTC_MONTH_OCTOBER
  *         @arg @ref LL_RTC_MONTH_NOVEMBER
  *         @arg @ref LL_RTC_MONTH_DECEMBER
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Month from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_DATE_GetMonth(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->DR, (RTC_DR_MT | RTC_DR_MU))) >> RTC_DR_MU_Pos);
}

/**
  * @brief  Set Day in BCD format.
  * @rmtoll
  *  RTC_DR           DT            LL_RTC_DATE_SetDay \n
  *  RTC_DR           DU            LL_RTC_DATE_SetDay
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_DATE_SetDay(uint32_t Day)
{
  STM32_MODIFY_REG(RTC->DR, (RTC_DR_DT | RTC_DR_DU),
                   (((Day & 0xF0U) << (RTC_DR_DT_Pos - 4U)) | ((Day & 0x0FU) << RTC_DR_DU_Pos)));
}

/**
  * @brief  Get Day in BCD format.
  * @rmtoll
  *  RTC_DR           DT            LL_RTC_DATE_GetDay \n
  *  RTC_DR           DU            LL_RTC_DATE_GetDay
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_DATE_GetDay(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->DR, (RTC_DR_DT | RTC_DR_DU))) >> RTC_DR_DU_Pos);
}

/**
  * @brief  Set date (WeekDay, Day, Month and Year) in BCD format.
  * @rmtoll
  *  RTC_DR           WDU           LL_RTC_DATE_Config \n
  *  RTC_DR           MT            LL_RTC_DATE_Config \n
  *  RTC_DR           MU            LL_RTC_DATE_Config \n
  *  RTC_DR           DT            LL_RTC_DATE_Config \n
  *  RTC_DR           DU            LL_RTC_DATE_Config \n
  *  RTC_DR           YT            LL_RTC_DATE_Config \n
  *  RTC_DR           YU            LL_RTC_DATE_Config
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @param  Month This parameter can be one of the following values:
  *         @arg @ref LL_RTC_MONTH_JANUARY
  *         @arg @ref LL_RTC_MONTH_FEBRUARY
  *         @arg @ref LL_RTC_MONTH_MARCH
  *         @arg @ref LL_RTC_MONTH_APRIL
  *         @arg @ref LL_RTC_MONTH_MAY
  *         @arg @ref LL_RTC_MONTH_JUNE
  *         @arg @ref LL_RTC_MONTH_JULY
  *         @arg @ref LL_RTC_MONTH_AUGUST
  *         @arg @ref LL_RTC_MONTH_SEPTEMBER
  *         @arg @ref LL_RTC_MONTH_OCTOBER
  *         @arg @ref LL_RTC_MONTH_NOVEMBER
  *         @arg @ref LL_RTC_MONTH_DECEMBER
  * @param  Year Value between Min_Data=0x00 and Max_Data=0x99
  */
__STATIC_INLINE void LL_RTC_DATE_Config(uint32_t WeekDay, uint32_t Day, uint32_t Month,
                                        uint32_t Year)
{
  uint32_t temp;

  temp = ((WeekDay & 0x0FU)  << RTC_DR_WDU_Pos)                                           | \
         (((Year & 0xF0U) << (RTC_DR_YT_Pos - 4U)) | ((Year & 0x0FU) << RTC_DR_YU_Pos))   | \
         (((Month & 0xF0U) << (RTC_DR_MT_Pos - 4U)) | ((Month & 0x0FU) << RTC_DR_MU_Pos)) | \
         (((Day & 0xF0U) << (RTC_DR_DT_Pos - 4U)) | ((Day & 0x0FU) << RTC_DR_DU_Pos));

  STM32_WRITE_REG(RTC->DR, temp);
}

/**
  * @brief  Get date (WeekDay, Day, Month and Year) in BCD format.
  * @rmtoll
  *  RTC_DR           WDU           LL_RTC_DATE_Get \n
  *  RTC_DR           MT            LL_RTC_DATE_Get \n
  *  RTC_DR           MU            LL_RTC_DATE_Get \n
  *  RTC_DR           DT            LL_RTC_DATE_Get \n
  *  RTC_DR           DU            LL_RTC_DATE_Get \n
  *  RTC_DR           YT            LL_RTC_DATE_Get \n
  *  RTC_DR           YU            LL_RTC_DATE_Get
  * @retval Combination of WeekDay, Day, Month and Year (Format: 0xWWDDMMYY)
  * @note   if RTC shadow registers are not bypassed (BYPSHAD=0), need to check if RSF flag is set
  *         before reading this bit
  * @note   helper macros LL_RTC_GET_WEEKDAY, LL_RTC_GET_YEAR, LL_RTC_GET_MONTH,
  *         and LL_RTC_GET_DAY are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_DATE_Get(void)
{
  uint32_t temp;

  temp = STM32_READ_BIT(RTC->DR, (RTC_DR_WDU | RTC_DR_MT | RTC_DR_MU | RTC_DR_DT | RTC_DR_DU | RTC_DR_YT | RTC_DR_YU));
  return (uint32_t)((((temp & RTC_DR_WDU) >> RTC_DR_WDU_Pos) << RTC_OFFSET_WEEKDAY) | \
                    (((((temp & RTC_DR_DT) >> RTC_DR_DT_Pos) << 4U) | ((temp & RTC_DR_DU) >> RTC_DR_DU_Pos)) \
                     << RTC_OFFSET_DAY)   | (((((temp & RTC_DR_MT) >> RTC_DR_MT_Pos) << 4U) | \
                                              ((temp & RTC_DR_MU) >> RTC_DR_MU_Pos)) << RTC_OFFSET_MONTH) | \
                    ((((temp & RTC_DR_YT) >> RTC_DR_YT_Pos) << 4U) | ((temp & RTC_DR_YU) >> RTC_DR_YU_Pos)));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_ALARM ALARMA ALARMB
  * @{
  */

/**
  * @brief  Set alarm A or B Binary mode auto clear.
  * @rmtoll
  *  RTC_ALRMBSSR    SSCLR        LL_RTC_ALM_SetBinAutoClr \n
  *  RTC_ALRMASSR    SSCLR        LL_RTC_ALM_SetBinAutoClr
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  BinaryAutoClr This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES
  * @note   This register can be written only when ALRAE or ALRBE is reset in RTC_CR register,
  *         or in initialization mode
  * @note   SSCLR must be kept to 0 when BCD or mixed mode is used (BIN = 00, 10 or 11).
  */
__STATIC_INLINE void LL_RTC_ALM_SetBinAutoClr(uint32_t Alarm, uint32_t BinaryAutoClr)
{
  STM32_MODIFY_REG(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)), RTC_ALRMASSR_SSCLR, BinaryAutoClr);
}

/**
  * @brief  Get alarm A or B Binary mode auto clear.
  * @rmtoll
  *  RTC_ALRMBSSR    SSCLR        LL_RTC_ALM_GetBinAutoClr \n
  *  RTC_ALRMASSR    SSCLR        LL_RTC_ALM_GetBinAutoClr
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetBinAutoClr(const uint32_t Alarm)
{
  return (uint32_t)(STM32_READ_BIT(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)), \
                                   RTC_ALRMASSR_SSCLR));
}

/**
  * @brief  Set alarm A or B flag automatic clear.
  * @rmtoll
  *  RTC_CR   ALRBFCLR        LL_RTC_ALM_SetFlagAutoClr \n
  *  RTC_CR   ALRAFCLR        LL_RTC_ALM_SetFlagAutoClr
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  AutoClr This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALM_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALM_AUTOCLR_YES
  * @note   This register can be written only when ALRAE or ALRBE is reset in RTC_CR register,
  *         or in initialization mode
  */
__STATIC_INLINE void LL_RTC_ALM_SetFlagAutoClr(uint32_t Alarm, uint32_t AutoClr)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_ALRAFCLR << Alarm, AutoClr << Alarm);
}

/**
  * @brief  Get alarm A or B flag automatic clear.
  * @rmtoll
  *  RTC_CR   ALRBFCLR        LL_RTC_ALM_GetFlagAutoClr \n
  *  RTC_CR   ALRAFCLR        LL_RTC_ALM_GetFlagAutoClr
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALM_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALM_AUTOCLR_YES
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetFlagAutoClr(const uint32_t Alarm)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, (RTC_CR_ALRAFCLR << Alarm)) >> Alarm);
}

/**
  * @brief  Set alarm A or B Time (hour, minute and second) in BCD format, time format,
  *         Day (weekday or day), weekday or day selection and masks.
  * @rmtoll
  *  RTC_ALRMBR       PM            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       HT            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       HU            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MNT           LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MNU           LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       ST            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       SU            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MSK4          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MSK3          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MSK2          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       MSK1          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       WDSEL         LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       DT            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMBR       DU            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       PM            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       HT            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       HU            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MNT           LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MNU           LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       ST            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       SU            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MSK4          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MSK3          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MSK2          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       MSK1          LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       WDSEL         LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       DT            LL_RTC_ALM_SetConfigDateTime \n
  *  RTC_ALRMAR       DU            LL_RTC_ALM_SetConfigDateTime
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  Mask This parameter can be a combination of the following values:
  *         @arg @ref LL_RTC_ALMA_MASK_NONE
  *         @arg @ref LL_RTC_ALMA_MASK_DATEWEEKDAY
  *         @arg @ref LL_RTC_ALMA_MASK_HOURS
  *         @arg @ref LL_RTC_ALMA_MASK_MINUTES
  *         @arg @ref LL_RTC_ALMA_MASK_SECONDS
  *         @arg @ref LL_RTC_ALMA_MASK_ALL
  * @param  DayWeekDaySelection This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_DATEWEEKDAYSEL_DATE
  *         @arg @ref LL_RTC_ALMA_DATEWEEKDAYSEL_WEEKDAY
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31 if the weekday is not selected
  *                   if weekday is selected, can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @note   This register can be written only when ALRAE or ALRBE is reset in RTC_CR register,
  *         or in initialization mode
  */
__STATIC_INLINE void LL_RTC_ALM_SetConfigDateTime(uint32_t Alarm,
                                                  uint32_t Mask, uint32_t DayWeekDaySelection,
                                                  uint32_t Day, uint32_t Format12_24,
                                                  uint32_t Hours, uint32_t Minutes,
                                                  uint32_t Seconds)
{
  STM32_WRITE_REG(*(&(RTC->ALRMAR) + (RTC_ALRBR_ALRAR_OFFSET * Alarm)), (Seconds << RTC_ALRMAR_SU_Pos)
                  | (Minutes << RTC_ALRMAR_MNU_Pos) | (Hours << RTC_ALRMAR_HU_Pos) | (Format12_24)
                  | (DayWeekDaySelection) | (Day << RTC_ALRMAR_DU_Pos) | Mask);
}

/**
  * @brief  Get alarm A or B Time (hour, minute and second) in BCD format, time format,
  *         Day (weekday or day), weekday or day selection and masks.
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval Content of the RTC_ALRMAR or RTC_ALRMBR register
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetConfigDateTime(const uint32_t Alarm)
{
  return STM32_READ_REG(*(&(RTC->ALRMAR) + (RTC_ALRBR_ALRAR_OFFSET * Alarm)));
}

/**
  * @brief  Set alarm A or B sub seconds mask and value.
  * @rmtoll
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALM_SetConfigSubSecond \n
  *  RTC_ALRMBSSR     SS            LL_RTC_ALM_SetConfigSubSecond \n
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALM_SetConfigSubSecond \n
  *  RTC_ALRMASSR     SS            LL_RTC_ALM_SetConfigSubSecond
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  Mask If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *              else Value between Min_Data=0x0 and Max_Data=0x3F
  * @param  Subsecond Value between Min_Data=0x00 and Max_Data=0x7FFF
  * @note   This register can be written only when ALRAE or ALRBE is reset in RTC_CR register,
  *         or in initialization mode
  */
__STATIC_INLINE void LL_RTC_ALM_SetConfigSubSecond(uint32_t Alarm,
                                                   uint32_t Mask,  uint32_t Subsecond)
{
  STM32_MODIFY_REG(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)),
                   RTC_ALRMASSR_MASKSS | RTC_ALRMASSR_SS, (Mask << RTC_ALRMASSR_MASKSS_Pos)
                   | Subsecond);
}

/**
  * @brief  Get alarm A or B sub seconds mask and value.
  * @rmtoll
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALM_GetConfigSubSecond \n
  *  RTC_ALRMASSR     SS            LL_RTC_ALM_GetConfigSubSecond \n
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALM_GetConfigSubSecond \n
  *  RTC_ALRMBSSR     SS            LL_RTC_ALM_GetConfigSubSecond
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval Mask If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *              else Value between Min_Data=0x0 and Max_Data=0x3F
  *         Subsecond Value between Min_Data=0x00 and Max_Data=0x7FFF
  *         Result is given in the form 0x00MMSSSS
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetConfigSubSecond(const uint32_t Alarm)
{
  uint32_t temp = STM32_READ_BIT(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)),
                                 RTC_ALRMASSR_MASKSS | RTC_ALRMASSR_SS);

  return ((((temp & RTC_ALRMASSR_MASKSS) >> RTC_ALRMASSR_MASKSS_Pos) << RTC_OFFSET_ALR_MASK_SUBS_SECONDS) | \
          ((temp & RTC_ALRMASSR_SS) >> RTC_ALRMASSR_SS_Pos));
}

/**
  * @brief  Start alarm A or B.
  * @rmtoll
  *  RTC_CR     ALRBIE        LL_RTC_ALM_Start \n
  *  RTC_CR     ALRBE         LL_RTC_ALM_Start \n
  *  RTC_CR     ALRAIE        LL_RTC_ALM_Start \n
  *  RTC_CR     ALRAE         LL_RTC_ALM_Start
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  Interruption This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_IT_DISABLE
  *         @arg @ref LL_RTC_ALMA_IT_ENABLE
  */
__STATIC_INLINE void LL_RTC_ALM_Start(uint32_t Alarm, uint32_t Interruption)
{
  STM32_MODIFY_REG(RTC->CR, (RTC_CR_ALRAIE << Alarm) |
                   (RTC_CR_ALRAE << Alarm),
                   (RTC_CR_ALRAE << Alarm) |
                   (Interruption << Alarm));
}

/**
  * @brief  Stop alarm A or B.
  * @rmtoll
  *  RTC_CR     ALRBIE        LL_RTC_ALM_Stop \n
  *  RTC_CR     ALRBE         LL_RTC_ALM_Stop \n
  *  RTC_CR     ALRAIE        LL_RTC_ALM_Stop \n
  *  RTC_CR     ALRAE         LL_RTC_ALM_Stop
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  */
__STATIC_INLINE void LL_RTC_ALM_Stop(uint32_t Alarm)
{
  STM32_MODIFY_REG(RTC->CR, (RTC_CR_ALRAIE << Alarm) | (RTC_CR_ALRAE << Alarm), 0UL);
}

/**
  * @brief  Is alarm A or B Enabled.
  * @rmtoll
  *  RTC_CR     ALRBIE        LL_RTC_ALM_IsStarted \n
  *  RTC_CR     ALRBE         LL_RTC_ALM_IsStarted \n
  *  RTC_CR     ALRAIE        LL_RTC_ALM_IsStarted \n
  *  RTC_CR     ALRAIE        LL_RTC_ALM_IsStarted
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_IsStarted(const uint32_t Alarm)
{
  return (((STM32_READ_REG(RTC->CR) & (RTC_CR_ALRAE << Alarm)) == 0U) ? 0UL : 1UL);
}

/**
  * @brief  Set alarm A or B Sub seconds value.
  * @rmtoll
  *  RTC_ALRABINR      SS            LL_RTC_ALM_SetBinarySubSecond \n
  *  RTC_ALRBBINR      SS            LL_RTC_ALM_SetBinarySubSecond
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  Subsecond If binary mode is none, Value between Min_Data=0x0 and Max_Data=0x7FFF
  *                   else Value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE void LL_RTC_ALM_SetBinarySubSecond(uint32_t Alarm, uint32_t Subsecond)
{
  STM32_WRITE_REG(*(&(RTC->ALRABINR) + (RTC_ALRBBINR_ALRABINR_OFFSET * Alarm)), Subsecond);
}

/**
  * @brief  Get alarm A or B Sub seconds value.
  * @rmtoll
  *  RTC_ALRABINR      SS            LL_RTC_ALM_GetBinarySubSecond \n
  *  RTC_ALRBBINR      SS            LL_RTC_ALM_GetBinarySubSecond
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval If binary mode is none, Value between Min_Data=0x0 and Max_Data=0x7FFF
  *         else Value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetBinarySubSecond(const uint32_t Alarm)
{
  return (uint32_t)(STM32_READ_REG(*(&(RTC->ALRABINR) + (RTC_ALRBBINR_ALRABINR_OFFSET * Alarm))));
}

/**
  * @brief  Set alarm A or B Sub seconds mask.
  * @rmtoll
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALM_SetSubSecondMask \n
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALM_SetSubSecondMask
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @param  Mask If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *              else Value between Min_Data=0x0 and Max_Data=0x3F
  * @note   This register can be written only when ALRAE or ALRBE is reset in RTC_CR register,
  *         or in initialization mode
  */
__STATIC_INLINE void LL_RTC_ALM_SetSubSecondMask(uint32_t Alarm, uint32_t Mask)
{
  STM32_MODIFY_REG(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)),
                   RTC_ALRMASSR_MASKSS, (Mask << RTC_ALRMASSR_MASKSS_Pos));
}

/**
  * @brief  Get alarm A or B sub seconds mask.
  * @rmtoll
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALM_GetSubSecondMask \n
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALM_GetSubSecondMask
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  * @retval Mask If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *              else Value between Min_Data=0x0 and Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_RTC_ALM_GetSubSecondMask(const uint32_t Alarm)
{
  uint32_t temp = STM32_READ_BIT(*(&(RTC->ALRMASSR) + (RTC_ALRBSSR_ALRASSR_OFFSET * Alarm)),
                                 RTC_ALRMASSR_MASKSS);

  return temp >> (RTC_ALRMASSR_MASKSS_Pos);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_ALARMA ALARMA
  * @{
  */

/**
  * @brief  Enable alarm A.
  * @rmtoll
  *  RTC_CR           ALRAE         LL_RTC_ALMA_Enable
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_ALMA_Enable(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_ALRAE);
}

/**
  * @brief  Disable alarm A.
  * @rmtoll
  *  RTC_CR           ALRAE         LL_RTC_ALMA_Disable
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_ALMA_Disable(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_ALRAE);
}

/**
  * @brief  Specify the alarm A masks.
  * @rmtoll
  *  RTC_ALRMAR       MSK4          LL_RTC_ALMA_SetMask \n
  *  RTC_ALRMAR       MSK3          LL_RTC_ALMA_SetMask \n
  *  RTC_ALRMAR       MSK2          LL_RTC_ALMA_SetMask \n
  *  RTC_ALRMAR       MSK1          LL_RTC_ALMA_SetMask
  * @param  Mask This parameter can be a combination of the following values:
  *         @arg @ref LL_RTC_ALMA_MASK_NONE
  *         @arg @ref LL_RTC_ALMA_MASK_DATEWEEKDAY
  *         @arg @ref LL_RTC_ALMA_MASK_HOURS
  *         @arg @ref LL_RTC_ALMA_MASK_MINUTES
  *         @arg @ref LL_RTC_ALMA_MASK_SECONDS
  *         @arg @ref LL_RTC_ALMA_MASK_ALL
  */
__STATIC_INLINE void LL_RTC_ALMA_SetMask(uint32_t Mask)
{
  STM32_MODIFY_REG(RTC->ALRMAR, RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1, Mask);
}

/**
  * @brief  Get the alarm A masks.
  * @rmtoll
  *  RTC_ALRMAR       MSK4          LL_RTC_ALMA_GetMask \n
  *  RTC_ALRMAR       MSK3          LL_RTC_ALMA_GetMask \n
  *  RTC_ALRMAR       MSK2          LL_RTC_ALMA_GetMask \n
  *  RTC_ALRMAR       MSK1          LL_RTC_ALMA_GetMask
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_RTC_ALMA_MASK_NONE
  *         @arg @ref LL_RTC_ALMA_MASK_DATEWEEKDAY
  *         @arg @ref LL_RTC_ALMA_MASK_HOURS
  *         @arg @ref LL_RTC_ALMA_MASK_MINUTES
  *         @arg @ref LL_RTC_ALMA_MASK_SECONDS
  *         @arg @ref LL_RTC_ALMA_MASK_ALL
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetMask(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMAR, RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1));
}

/**
  * @brief  Enable alarm A weekday selection (DU[3:0] represents the weekday. DT[1:0] is ignored).
  * @rmtoll
  *  RTC_ALRMAR       WDSEL         LL_RTC_ALMA_EnableWeekday
  */
__STATIC_INLINE void LL_RTC_ALMA_EnableWeekday(void)
{
  STM32_SET_BIT(RTC->ALRMAR, RTC_ALRMAR_WDSEL);
}

/**
  * @brief  Disable alarm A weekday selection (DU[3:0] represents the date).
  * @rmtoll
  *  RTC_ALRMAR       WDSEL         LL_RTC_ALMA_DisableWeekday
  */
__STATIC_INLINE void LL_RTC_ALMA_DisableWeekday(void)
{
  STM32_CLEAR_BIT(RTC->ALRMAR, RTC_ALRMAR_WDSEL);
}

/**
  * @brief  Check whether alarm A weekday selection is enabled.
  * @rmtoll
  *  RTC_ALRMAR       WDSEL         LL_RTC_ALMA_IsEnabledWeekday
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_IsEnabledWeekday(void)
{
  return ((STM32_READ_BIT(RTC->ALRMAR, RTC_ALRMAR_WDSEL) == (RTC_ALRMAR_WDSEL)) ? 1UL : 0UL);
}

/**
  * @brief  Set alarm A Day in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       DT            LL_RTC_ALMA_SetDay \n
  *  RTC_ALRMAR       DU            LL_RTC_ALMA_SetDay
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMA_SetDay(uint32_t Day)
{
  STM32_MODIFY_REG(RTC->ALRMAR, (RTC_ALRMAR_DT | RTC_ALRMAR_DU),
                   (((Day & 0xF0U) << (RTC_ALRMAR_DT_Pos - 4U)) | ((Day & 0x0FU) << RTC_ALRMAR_DU_Pos)));
}

/**
  * @brief  Get alarm A Day in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       DT            LL_RTC_ALMA_GetDay \n
  *  RTC_ALRMAR       DU            LL_RTC_ALMA_GetDay
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetDay(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMAR, (RTC_ALRMAR_DT | RTC_ALRMAR_DU))) >> RTC_ALRMAR_DU_Pos);
}

/**
  * @brief  Set alarm A Weekday.
  * @rmtoll
  *  RTC_ALRMAR       DU            LL_RTC_ALMA_SetWeekDay
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  * @note   DU in weekday mode only if WDSEL is enabled
  */
__STATIC_INLINE void LL_RTC_ALMA_SetWeekDay(uint32_t WeekDay)
{
  STM32_MODIFY_REG(RTC->ALRMAR, RTC_ALRMAR_DU, WeekDay << RTC_ALRMAR_DU_Pos);
}

/**
  * @brief  Get alarm A Weekday.
  * @rmtoll
  *  RTC_ALRMAR       DU            LL_RTC_ALMA_GetWeekDay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  * @note   DU in weekday mode only if WDSEL is enabled
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetWeekDay(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMAR, RTC_ALRMAR_DU) >> RTC_ALRMAR_DU_Pos);
}

/**
  * @brief  Set alarm A time format (AM/24-hour or PM notation).
  * @rmtoll
  *  RTC_ALRMAR       PM            LL_RTC_ALMA_SetTimeFormat
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_PM
  */
__STATIC_INLINE void LL_RTC_ALMA_SetTimeFormat(uint32_t TimeFormat)
{
  STM32_MODIFY_REG(RTC->ALRMAR, RTC_ALRMAR_PM, TimeFormat);
}

/**
  * @brief  Get alarm A time format (AM or PM notation).
  * @rmtoll
  *  RTC_ALRMAR       PM            LL_RTC_ALMA_GetTimeFormat
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetTimeFormat(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMAR, RTC_ALRMAR_PM));
}

/**
  * @brief  Set alarm A Hours in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       HT            LL_RTC_ALMA_SetHour \n
  *  RTC_ALRMAR       HU            LL_RTC_ALMA_SetHour
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Hours from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMA_SetHour(uint32_t Hours)
{
  STM32_MODIFY_REG(RTC->ALRMAR, (RTC_ALRMAR_HT | RTC_ALRMAR_HU),
                   (((Hours & 0xF0U) << (RTC_ALRMAR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMAR_HU_Pos)));
}

/**
  * @brief  Get alarm A Hours in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       HT            LL_RTC_ALMA_GetHour \n
  *  RTC_ALRMAR       HU            LL_RTC_ALMA_GetHour
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Hours from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetHour(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMAR, (RTC_ALRMAR_HT | RTC_ALRMAR_HU))) >> RTC_ALRMAR_HU_Pos);
}

/**
  * @brief  Set alarm A Minutes in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       MNT           LL_RTC_ALMA_SetMinute \n
  *  RTC_ALRMAR       MNU           LL_RTC_ALMA_SetMinute
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Minutes from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMA_SetMinute(uint32_t Minutes)
{
  STM32_MODIFY_REG(RTC->ALRMAR, (RTC_ALRMAR_MNT | RTC_ALRMAR_MNU),
                   (((Minutes & 0xF0U) << (RTC_ALRMAR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMAR_MNU_Pos)));
}

/**
  * @brief  Get alarm A Minutes in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       MNT           LL_RTC_ALMA_GetMinute \n
  *  RTC_ALRMAR       MNU           LL_RTC_ALMA_GetMinute
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Minutes from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetMinute(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMAR, (RTC_ALRMAR_MNT | RTC_ALRMAR_MNU))) >> RTC_ALRMAR_MNU_Pos);
}

/**
  * @brief  Set alarm A Seconds in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       ST            LL_RTC_ALMA_SetSecond \n
  *  RTC_ALRMAR       SU            LL_RTC_ALMA_SetSecond
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Seconds from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMA_SetSecond(uint32_t Seconds)
{
  STM32_MODIFY_REG(RTC->ALRMAR, (RTC_ALRMAR_ST | RTC_ALRMAR_SU),
                   (((Seconds & 0xF0U) << (RTC_ALRMAR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMAR_SU_Pos)));
}

/**
  * @brief  Get alarm A Seconds in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       ST            LL_RTC_ALMA_GetSecond \n
  *  RTC_ALRMAR       SU            LL_RTC_ALMA_GetSecond
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Seconds from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetSecond(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMAR, (RTC_ALRMAR_ST | RTC_ALRMAR_SU))) >> RTC_ALRMAR_SU_Pos);
}

/**
  * @brief  Set alarm A Time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       PM            LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       HT            LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       HU            LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       MNT           LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       MNU           LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       ST            LL_RTC_ALMA_ConfigTime \n
  *  RTC_ALRMAR       SU            LL_RTC_ALMA_ConfigTime
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMA_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE void LL_RTC_ALMA_ConfigTime(uint32_t Format12_24, uint32_t Hours, uint32_t Minutes,
                                            uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24 | (((Hours & 0xF0U) << (RTC_ALRMAR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMAR_HU_Pos))    | \
         (((Minutes & 0xF0U) << (RTC_ALRMAR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMAR_MNU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_ALRMAR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMAR_SU_Pos));

  STM32_MODIFY_REG(RTC->ALRMAR, RTC_ALRMAR_PM | RTC_ALRMAR_HT | RTC_ALRMAR_HU | RTC_ALRMAR_MNT | RTC_ALRMAR_MNU | \
                   RTC_ALRMAR_ST | RTC_ALRMAR_SU, temp);
}

/**
  * @brief  Get alarm A Time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_ALRMAR       HT            LL_RTC_ALMA_GetTime \n
  *  RTC_ALRMAR       HU            LL_RTC_ALMA_GetTime \n
  *  RTC_ALRMAR       MNT           LL_RTC_ALMA_GetTime \n
  *  RTC_ALRMAR       MNU           LL_RTC_ALMA_GetTime \n
  *  RTC_ALRMAR       ST            LL_RTC_ALMA_GetTime \n
  *  RTC_ALRMAR       SU            LL_RTC_ALMA_GetTime
  * @retval Combination of hours, minutes and seconds
  * @note   helper macros LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE and LL_RTC_GET_SECOND
  *         are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetTime(void)
{
  return (uint32_t)((LL_RTC_ALMA_GetHour() << RTC_OFFSET_HOUR) | (LL_RTC_ALMA_GetMinute() << \
                                                                  RTC_OFFSET_MINUTE) | LL_RTC_ALMA_GetSecond());
}

/**
  * @brief  Set alarm A Mask the most-significant bits starting at this bit.
  * @rmtoll
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALMA_SetSubSecondMask
  * @param  Mask If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *              else Value between Min_Data=0x0 and Max_Data=0x3F
  * @note   This register can be written only when ALRAE is reset in RTC_CR register,
  *         or in initialization mode
  */
__STATIC_INLINE void LL_RTC_ALMA_SetSubSecondMask(uint32_t Mask)
{
  STM32_MODIFY_REG(RTC->ALRMASSR, RTC_ALRMASSR_MASKSS, Mask << RTC_ALRMASSR_MASKSS_Pos);
}

/**
  * @brief  Get alarm A Mask the most-significant bits starting at this bit.
  * @rmtoll
  *  RTC_ALRMASSR     MASKSS        LL_RTC_ALMA_GetSubSecondMask
  * @retval If binary mode is none, Value between Min_Data=0x0 and Max_Data=0xF
  *         else Value between Min_Data=0x0 and Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetSubSecondMask(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMASSR, RTC_ALRMASSR_MASKSS) >> RTC_ALRMASSR_MASKSS_Pos);
}

/**
  * @brief  Set alarm A Binary mode auto clear.
  * @rmtoll
  *  RTC_ALRMASSR    SSCLR        LL_RTC_ALMA_SetBinAutoClr
  * @param  BinaryAutoClr This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES
  * @note   This register can be written only when ALRAE is reset in RTC_CR register,
  *         or in initialization mode
  * @note   SSCLR must be kept to 0 when BCD or mixed mode is used (BIN = 00, 10 or 11).
  */
__STATIC_INLINE void LL_RTC_ALMA_SetBinAutoClr(uint32_t BinaryAutoClr)
{
  STM32_MODIFY_REG(RTC->ALRMASSR, RTC_ALRMASSR_SSCLR, BinaryAutoClr);
}

/**
  * @brief  Get alarm A Binary mode auto clear.
  * @rmtoll
  *  RTC_ALRMASSR     SSCLR        LL_RTC_ALMA_GetBinAutoClr
  * @retval It can be one of the following values:
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetBinAutoClr(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMASSR, RTC_ALRMASSR_SSCLR));
}

/**
  * @brief  Set alarm A Sub seconds value.
  * @rmtoll
  *  RTC_ALRMASSR     SS            LL_RTC_ALMA_SetSubSecond
  * @param  Subsecond If binary mode is none, Value between Min_Data=0x0 and Max_Data=0x7FFF
  *                   else Value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE void LL_RTC_ALMA_SetSubSecond(uint32_t Subsecond)
{
  STM32_MODIFY_REG(RTC->ALRMASSR, RTC_ALRMASSR_SS, Subsecond);
}

/**
  * @brief  Get alarm A Sub seconds value.
  * @rmtoll
  *  RTC_ALRMASSR     SS            LL_RTC_ALMA_GetSubSecond
  * @retval If binary mode is none, Value between Min_Data=0x0 and Max_Data=0x7FFF
  *         else Value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_ALMA_GetSubSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMASSR, RTC_ALRMASSR_SS));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_ALARMB ALARMB
  * @{
  */

/**
  * @brief  Enable alarm B.
  * @rmtoll
  *  RTC_CR           ALRBE         LL_RTC_ALMB_Enable
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_ALMB_Enable(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_ALRBE);
}

/**
  * @brief  Disable alarm B.
  * @rmtoll
  *  RTC_CR           ALRBE         LL_RTC_ALMB_Disable
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function must preferably be called before
  */
__STATIC_INLINE void LL_RTC_ALMB_Disable(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_ALRBE);
}

/**
  * @brief  Specify the alarm B masks.
  * @rmtoll
  *  RTC_ALRMBR       MSK4          LL_RTC_ALMB_SetMask \n
  *  RTC_ALRMBR       MSK3          LL_RTC_ALMB_SetMask \n
  *  RTC_ALRMBR       MSK2          LL_RTC_ALMB_SetMask \n
  *  RTC_ALRMBR       MSK1          LL_RTC_ALMB_SetMask
  * @param  Mask This parameter can be a combination of the following values:
  *         @arg @ref LL_RTC_ALMB_MASK_NONE
  *         @arg @ref LL_RTC_ALMB_MASK_DATEWEEKDAY
  *         @arg @ref LL_RTC_ALMB_MASK_HOURS
  *         @arg @ref LL_RTC_ALMB_MASK_MINUTES
  *         @arg @ref LL_RTC_ALMB_MASK_SECONDS
  *         @arg @ref LL_RTC_ALMB_MASK_ALL
  */
__STATIC_INLINE void LL_RTC_ALMB_SetMask(uint32_t Mask)
{
  STM32_MODIFY_REG(RTC->ALRMBR, RTC_ALRMBR_MSK4 | RTC_ALRMBR_MSK3 | RTC_ALRMBR_MSK2 | RTC_ALRMBR_MSK1, Mask);
}

/**
  * @brief  Get the alarm B masks.
  * @rmtoll
  *  RTC_ALRMBR       MSK4          LL_RTC_ALMB_GetMask \n
  *  RTC_ALRMBR       MSK3          LL_RTC_ALMB_GetMask \n
  *  RTC_ALRMBR       MSK2          LL_RTC_ALMB_GetMask \n
  *  RTC_ALRMBR       MSK1          LL_RTC_ALMB_GetMask
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_RTC_ALMB_MASK_NONE
  *         @arg @ref LL_RTC_ALMB_MASK_DATEWEEKDAY
  *         @arg @ref LL_RTC_ALMB_MASK_HOURS
  *         @arg @ref LL_RTC_ALMB_MASK_MINUTES
  *         @arg @ref LL_RTC_ALMB_MASK_SECONDS
  *         @arg @ref LL_RTC_ALMB_MASK_ALL
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetMask(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBR, RTC_ALRMBR_MSK4 | RTC_ALRMBR_MSK3 | RTC_ALRMBR_MSK2 | RTC_ALRMBR_MSK1));
}

/**
  * @brief  Enable alarm B weekday selection (DU[3:0] represents the weekday. DT[1:0] is ignored).
  * @rmtoll
  *  RTC_ALRMBR       WDSEL         LL_RTC_ALMB_EnableWeekday
  */
__STATIC_INLINE void LL_RTC_ALMB_EnableWeekday(void)
{
  STM32_SET_BIT(RTC->ALRMBR, RTC_ALRMBR_WDSEL);
}

/**
  * @brief  Disable alarm B weekday selection (DU[3:0] represents the date).
  * @rmtoll
  *  RTC_ALRMBR       WDSEL         LL_RTC_ALMB_DisableWeekday
  */
__STATIC_INLINE void LL_RTC_ALMB_DisableWeekday(void)
{
  STM32_CLEAR_BIT(RTC->ALRMBR, RTC_ALRMBR_WDSEL);
}

/**
  * @brief  Check whether alarm B weekday selection is enabled.
  * @rmtoll
  *  RTC_ALRMBR       WDSEL         LL_RTC_ALMB_IsEnabledWeekday
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_IsEnabledWeekday(void)
{
  return ((STM32_READ_BIT(RTC->ALRMBR, RTC_ALRMBR_WDSEL) == (RTC_ALRMBR_WDSEL)) ? 1UL : 0UL);
}


/**
  * @brief  Set alarm B Day in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       DT            LL_RTC_ALMB_SetDay \n
  *  RTC_ALRMBR       DU            LL_RTC_ALMB_SetDay
  * @param  Day Value between Min_Data=0x01 and Max_Data=0x31
  * @note   helper macro LL_RTC_CONVERT_BIN2BCD is available to convert Day from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMB_SetDay(uint32_t Day)
{
  STM32_MODIFY_REG(RTC->ALRMBR, (RTC_ALRMBR_DT | RTC_ALRMBR_DU),
                   (((Day & 0xF0U) << (RTC_ALRMBR_DT_Pos - 4U)) | ((Day & 0x0FU) << RTC_ALRMBR_DU_Pos)));
}

/**
  * @brief  Get alarm B Day in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       DT            LL_RTC_ALMB_GetDay \n
  *  RTC_ALRMBR       DU            LL_RTC_ALMB_GetDay
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  * @note   helper macro LL_RTC_CONVERT_BCD2BIN is available to convert Day from BCD to Binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetDay(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMBR, (RTC_ALRMBR_DT | RTC_ALRMBR_DU))) >> RTC_ALRMBR_DU_Pos);
}

/**
  * @brief  Set alarm B Weekday.
  * @rmtoll
  *  RTC_ALRMBR       DU            LL_RTC_ALMB_SetWeekDay
  * @param  WeekDay This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE void LL_RTC_ALMB_SetWeekDay(uint32_t WeekDay)
{
  STM32_MODIFY_REG(RTC->ALRMBR, RTC_ALRMBR_DU, WeekDay << RTC_ALRMBR_DU_Pos);
}

/**
  * @brief  Get alarm B Weekday.
  * @rmtoll
  *  RTC_ALRMBR       DU            LL_RTC_ALMB_GetWeekDay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetWeekDay(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBR, RTC_ALRMBR_DU) >> RTC_ALRMBR_DU_Pos);
}

/**
  * @brief  Set alarm B time format (AM/24-hour or PM notation).
  * @rmtoll
  *  RTC_ALRMBR       PM            LL_RTC_ALMB_SetTimeFormat
  * @param  TimeFormat This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_PM
  */
__STATIC_INLINE void LL_RTC_ALMB_SetTimeFormat(uint32_t TimeFormat)
{
  STM32_MODIFY_REG(RTC->ALRMBR, RTC_ALRMBR_PM, TimeFormat);
}

/**
  * @brief  Get alarm B time format (AM/24-hour or PM notation).
  * @rmtoll
  *  RTC_ALRMBR       PM            LL_RTC_ALMB_GetTimeFormat
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetTimeFormat(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBR, RTC_ALRMBR_PM));
}

/**
  * @brief  Set alarm B hours in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       HT            LL_RTC_ALMB_SetHour \n
  *  RTC_ALRMBR       HU            LL_RTC_ALMB_SetHour
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   Helper macro LL_RTC_CONVERT_BIN2BCD is available to convert hours from
  *         binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMB_SetHour(uint32_t Hours)
{
  STM32_MODIFY_REG(RTC->ALRMBR, (RTC_ALRMBR_HT | RTC_ALRMBR_HU),
                   (((Hours & 0xF0U) << (RTC_ALRMBR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMBR_HU_Pos)));
}

/**
  * @brief  Get alarm B hours in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       HT            LL_RTC_ALMB_GetHour \n
  *  RTC_ALRMBR       HU            LL_RTC_ALMB_GetHour
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert hours from
  *         BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetHour(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMBR, (RTC_ALRMBR_HT | RTC_ALRMBR_HU))) >> RTC_ALRMBR_HU_Pos);
}

/**
  * @brief  Set alarm B minutes in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       MNT           LL_RTC_ALMB_SetMinute \n
  *  RTC_ALRMBR       MNU           LL_RTC_ALMB_SetMinute
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BIN2BCD is available to convert minutes
  *         from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMB_SetMinute(uint32_t Minutes)
{
  STM32_MODIFY_REG(RTC->ALRMBR, (RTC_ALRMBR_MNT | RTC_ALRMBR_MNU),
                   (((Minutes & 0xF0U) << (RTC_ALRMBR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMBR_MNU_Pos)));
}

/**
  * @brief  Get alarm B minutes in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       MNT           LL_RTC_ALMB_GetMinute \n
  *  RTC_ALRMBR       MNU           LL_RTC_ALMB_GetMinute
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert minutes
  *         from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetMinute(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMBR, (RTC_ALRMBR_MNT | RTC_ALRMBR_MNU))) >> RTC_ALRMBR_MNU_Pos);
}

/**
  * @brief  Set alarm B seconds in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       ST            LL_RTC_ALMB_SetSecond \n
  *  RTC_ALRMBR       SU            LL_RTC_ALMB_SetSecond
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BIN2BCD is available to convert seconds
  *         from binary to BCD format
  */
__STATIC_INLINE void LL_RTC_ALMB_SetSecond(uint32_t Seconds)
{
  STM32_MODIFY_REG(RTC->ALRMBR, (RTC_ALRMBR_ST | RTC_ALRMBR_SU),
                   (((Seconds & 0xF0U) << (RTC_ALRMBR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMBR_SU_Pos)));
}

/**
  * @brief  Get alarm B seconds in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       ST            LL_RTC_ALMB_GetSecond \n
  *  RTC_ALRMBR       SU            LL_RTC_ALMB_GetSecond
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert seconds
  *         from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetSecond(void)
{
  return (uint32_t)((STM32_READ_BIT(RTC->ALRMBR, (RTC_ALRMBR_ST | RTC_ALRMBR_SU))) >> RTC_ALRMBR_SU_Pos);
}

/**
  * @brief  Set alarm B time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       PM            LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       HT            LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       HU            LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       MNT           LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       MNU           LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       ST            LL_RTC_ALMB_ConfigTime \n
  *  RTC_ALRMBR       SU            LL_RTC_ALMB_ConfigTime
  * @param  Format12_24 This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_ALMB_TIME_FORMAT_PM
  * @param  Hours Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @param  Minutes Value between Min_Data=0x00 and Max_Data=0x59
  * @param  Seconds Value between Min_Data=0x00 and Max_Data=0x59
  */
__STATIC_INLINE void LL_RTC_ALMB_ConfigTime(uint32_t Format12_24, uint32_t Hours, uint32_t Minutes,
                                            uint32_t Seconds)
{
  uint32_t temp;

  temp = Format12_24 | (((Hours & 0xF0U) << (RTC_ALRMBR_HT_Pos - 4U)) | ((Hours & 0x0FU) << RTC_ALRMBR_HU_Pos))    | \
         (((Minutes & 0xF0U) << (RTC_ALRMBR_MNT_Pos - 4U)) | ((Minutes & 0x0FU) << RTC_ALRMBR_MNU_Pos)) | \
         (((Seconds & 0xF0U) << (RTC_ALRMBR_ST_Pos - 4U)) | ((Seconds & 0x0FU) << RTC_ALRMBR_SU_Pos));

  STM32_MODIFY_REG(RTC->ALRMBR, RTC_ALRMBR_PM | RTC_ALRMBR_HT | RTC_ALRMBR_HU | RTC_ALRMBR_MNT | RTC_ALRMBR_MNU | \
                   RTC_ALRMBR_ST | RTC_ALRMBR_SU, temp);
}

/**
  * @brief  Get alarm B time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_ALRMBR       HT            LL_RTC_ALMB_GetTime \n
  *  RTC_ALRMBR       HU            LL_RTC_ALMB_GetTime \n
  *  RTC_ALRMBR       MNT           LL_RTC_ALMB_GetTime \n
  *  RTC_ALRMBR       MNU           LL_RTC_ALMB_GetTime \n
  *  RTC_ALRMBR       ST            LL_RTC_ALMB_GetTime \n
  *  RTC_ALRMBR       SU            LL_RTC_ALMB_GetTime
  * @retval Combination of hours, minutes and seconds
  * @note   Helper macros LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE and LL_RTC_GET_SECOND
  *         are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetTime(void)
{
  return (uint32_t)((LL_RTC_ALMB_GetHour() << RTC_OFFSET_HOUR) | (LL_RTC_ALMB_GetMinute() << \
                                                                  RTC_OFFSET_MINUTE) | LL_RTC_ALMB_GetSecond());
}

/**
  * @brief  Set alarm B mask for the most significant bits starting at this bit.
  * @rmtoll
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALMB_SetSubSecondMask
  * @param  Mask If binary mode is not enabled, value between Min_Data=0x0 and Max_Data=0xF
  *              else value between Min_Data=0x0 and Max_Data=0x3F
  * @note   This register can be written only when ALRBE is reset in RTC_CR register
  *         or in initialization mode.
  */
__STATIC_INLINE void LL_RTC_ALMB_SetSubSecondMask(uint32_t Mask)
{
  STM32_MODIFY_REG(RTC->ALRMBSSR, RTC_ALRMBSSR_MASKSS, Mask << RTC_ALRMBSSR_MASKSS_Pos);
}

/**
  * @brief  Get alarm B mask for the most significant bits starting at this bit.
  * @rmtoll
  *  RTC_ALRMBSSR     MASKSS        LL_RTC_ALMB_GetSubSecondMask
  * @retval If binary mode is not enabled, value between Min_Data=0x0 and Max_Data=0xF
  *         else value between Min_Data=0x0 and Max_Data=0x3F
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetSubSecondMask(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBSSR, RTC_ALRMBSSR_MASKSS) >> RTC_ALRMBSSR_MASKSS_Pos);
}

/**
  * @brief  Set alarm B binary mode auto-clear.
  * @rmtoll
  *  RTC_ALRMBSSR     SSCLR        LL_RTC_ALMB_SetBinAutoClr
  * @param  BinaryAutoClr This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_YES
  * @note   This register can be written only when ALRBE is reset in RTC_CR register
  *         or in initialization mode.
  * @note   SSCLR must be kept at 0 when BCD or mixed mode is used (BIN = 00, 10 or 11).
  */
__STATIC_INLINE void LL_RTC_ALMB_SetBinAutoClr(uint32_t BinaryAutoClr)
{
  STM32_MODIFY_REG(RTC->ALRMBSSR, RTC_ALRMBSSR_SSCLR, BinaryAutoClr);
}

/**
  * @brief  Get alarm B binary mode auto-clear.
  * @rmtoll
  *  RTC_ALRMBSSR     SSCLR        LL_RTC_ALMB_GetBinAutoClr
  * @retval It can be one of the following values:
  *         @arg @ref LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_NO
  *         @arg @ref LL_RTC_ALMB_SUBSECONDBIN_AUTOCLR_YES
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetBinAutoClr(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBSSR, RTC_ALRMBSSR_SSCLR));
}

/**
  * @brief  Set alarm B subseconds value.
  * @rmtoll
  *  RTC_ALRMBSSR     SS            LL_RTC_ALMB_SetSubSecond
  * @param  Subsecond If binary mode is not enabled, value between Min_Data=0x0 and Max_Data=0x7FFF
  *                   else value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE void LL_RTC_ALMB_SetSubSecond(uint32_t Subsecond)
{
  STM32_MODIFY_REG(RTC->ALRMBSSR, RTC_ALRMBSSR_SS, Subsecond);
}

/**
  * @brief  Get alarm B subseconds value.
  * @rmtoll
  *  RTC_ALRMBSSR     SS            LL_RTC_ALMB_GetSubSecond
  * @retval If binary mode is not enabled, value between Min_Data=0x0 and Max_Data=0x7FFF
  *         else value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_ALMB_GetSubSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->ALRMBSSR, RTC_ALRMBSSR_SS));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Timestamp Timestamp
  * @{
  */

/**
  * @brief  Enable timestamp.
  * @rmtoll
  *  RTC_CR           TSE           LL_RTC_TS_Enable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_TS_Enable(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_TSE);
}

/**
  * @brief  Disable timestamp.
  * @rmtoll
  *  RTC_CR           TSE           LL_RTC_TS_Disable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_TS_Disable(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_TSE);
}

/**
  * @brief  Check whether timestamp is enabled.
  * @rmtoll
  *  RTC_CR           TSE          LL_RTC_TS_IsEnabled
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_TS_IsEnabled(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_TSE) == (RTC_CR_TSE)) ? 1UL : 0UL);
}

/**
  * @brief  Set timestamp event active edge.
  * @rmtoll
  *  RTC_CR           TSEDGE        LL_RTC_TS_SetActiveEdge
  * @param  Edge This parameter can be one of the following values:
  *         @arg @ref LL_RTC_TIMESTAMP_EDGE_RISING
  *         @arg @ref LL_RTC_TIMESTAMP_EDGE_FALLING
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   TSE must be reset when TSEDGE is changed to avoid unwanted TSF setting
  */
__STATIC_INLINE void LL_RTC_TS_SetActiveEdge(uint32_t Edge)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_TSEDGE, Edge);
}

/**
  * @brief  Get timestamp event active edge.
  * @rmtoll
  *  RTC_CR           TSEDGE        LL_RTC_TS_GetActiveEdge
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_TIMESTAMP_EDGE_RISING
  *         @arg @ref LL_RTC_TIMESTAMP_EDGE_FALLING
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetActiveEdge(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_TSEDGE));
}

/**
  * @brief  Get timestamp AM/PM notation (AM or 24-hour format).
  * @rmtoll
  *  RTC_TSTR         PM            LL_RTC_TS_GetTimeFormat
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_TS_TIME_FORMAT_AM_24H
  *         @arg @ref LL_RTC_TS_TIME_FORMAT_PM
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetTimeFormat(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSTR, RTC_TSTR_PM));
}

/**
  * @brief  Get timestamp hours in BCD format.
  * @rmtoll
  *  RTC_TSTR         HT            LL_RTC_TS_GetHour \n
  *  RTC_TSTR         HU            LL_RTC_TS_GetHour
  * @retval Value between Min_Data=0x01 and Max_Data=0x12 or between Min_Data=0x00 and Max_Data=0x23
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert hours from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetHour(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSTR, RTC_TSTR_HT | RTC_TSTR_HU) >> RTC_TSTR_HU_Pos);
}

/**
  * @brief  Get timestamp minutes in BCD format.
  * @rmtoll
  *  RTC_TSTR         MNT           LL_RTC_TS_GetMinute \n
  *  RTC_TSTR         MNU           LL_RTC_TS_GetMinute
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert minutes from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetMinute(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSTR, RTC_TSTR_MNT | RTC_TSTR_MNU) >> RTC_TSTR_MNU_Pos);
}

/**
  * @brief  Get timestamp seconds in BCD format.
  * @rmtoll
  *  RTC_TSTR         ST            LL_RTC_TS_GetSecond \n
  *  RTC_TSTR         SU            LL_RTC_TS_GetSecond
  * @retval Value between Min_Data=0x00 and Max_Data=0x59
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert seconds from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSTR, RTC_TSTR_ST | RTC_TSTR_SU));
}

/**
  * @brief  Get timestamp time (hour, minute and second) in BCD format.
  * @rmtoll
  *  RTC_TSTR         HT            LL_RTC_TS_GetTime \n
  *  RTC_TSTR         HU            LL_RTC_TS_GetTime \n
  *  RTC_TSTR         MNT           LL_RTC_TS_GetTime \n
  *  RTC_TSTR         MNU           LL_RTC_TS_GetTime \n
  *  RTC_TSTR         ST            LL_RTC_TS_GetTime \n
  *  RTC_TSTR         SU            LL_RTC_TS_GetTime
  * @retval Combination of hours, minutes and seconds
  * @note   Helper macros LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE and LL_RTC_GET_SECOND
  *         are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetTime(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSTR,
                                   RTC_TSTR_HT | RTC_TSTR_HU | RTC_TSTR_MNT
                                   | RTC_TSTR_MNU | RTC_TSTR_ST | RTC_TSTR_SU));
}

/**
  * @brief  Get timestamp time (hour, minute and second) in BCD format and time format.
  * @rmtoll
  *  RTC_TSTR        PM            LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        HT            LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        HU            LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        MNT           LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        MNU           LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        ST            LL_RTC_TS_GetTimeAndFormat \n
  *  RTC_TSTR        SU            LL_RTC_TS_GetTimeAndFormat
  * @retval Combination of format, hours, minutes and seconds (Format: 0x0FHHMMSS)
  * @note   Helper macros LL_RTC_GET_FORMAT, LL_RTC_GET_HOUR, LL_RTC_GET_MINUTE,
  *         and LL_RTC_GET_SECOND are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetTimeAndFormat(void)
{
  uint32_t temp;

  temp = STM32_READ_BIT(RTC->TSTR, (RTC_TSTR_PM | RTC_TSTR_HT | RTC_TSTR_HU | RTC_TSTR_MNT | RTC_TSTR_MNU | \
                                    RTC_TSTR_ST | RTC_TSTR_SU));
  return (uint32_t)((((temp & RTC_TSTR_PM) >> RTC_TSTR_PM_Pos) << RTC_OFFSET_FORMAT) | \
                    (((((temp & RTC_TSTR_HT) >> RTC_TSTR_HT_Pos) << 4U) | \
                      ((temp & RTC_TSTR_HU) >> RTC_TSTR_HU_Pos)) << RTC_OFFSET_HOUR) | \
                    (((((temp & RTC_TSTR_MNT) >> RTC_TSTR_MNT_Pos) << 4U) | \
                      ((temp & RTC_TSTR_MNU) >> RTC_TSTR_MNU_Pos)) << RTC_OFFSET_MINUTE) | \
                    ((((temp & RTC_TSTR_ST) >> RTC_TSTR_ST_Pos) << 4U) | ((temp & RTC_TSTR_SU) >> RTC_TSTR_SU_Pos)));
}

/**
  * @brief  Get timestamp weekday.
  * @rmtoll
  *  RTC_TSDR         WDU           LL_RTC_TS_GetWeekDay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WEEKDAY_MONDAY
  *         @arg @ref LL_RTC_WEEKDAY_TUESDAY
  *         @arg @ref LL_RTC_WEEKDAY_WEDNESDAY
  *         @arg @ref LL_RTC_WEEKDAY_THURSDAY
  *         @arg @ref LL_RTC_WEEKDAY_FRIDAY
  *         @arg @ref LL_RTC_WEEKDAY_SATURDAY
  *         @arg @ref LL_RTC_WEEKDAY_SUNDAY
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetWeekDay(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSDR, RTC_TSDR_WDU) >> RTC_TSDR_WDU_Pos);
}

/**
  * @brief  Get timestamp month in BCD format.
  * @rmtoll
  *  RTC_TSDR         MT            LL_RTC_TS_GetMonth \n
  *  RTC_TSDR         MU            LL_RTC_TS_GetMonth
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_MONTH_JANUARY
  *         @arg @ref LL_RTC_MONTH_FEBRUARY
  *         @arg @ref LL_RTC_MONTH_MARCH
  *         @arg @ref LL_RTC_MONTH_APRIL
  *         @arg @ref LL_RTC_MONTH_MAY
  *         @arg @ref LL_RTC_MONTH_JUNE
  *         @arg @ref LL_RTC_MONTH_JULY
  *         @arg @ref LL_RTC_MONTH_AUGUST
  *         @arg @ref LL_RTC_MONTH_SEPTEMBER
  *         @arg @ref LL_RTC_MONTH_OCTOBER
  *         @arg @ref LL_RTC_MONTH_NOVEMBER
  *         @arg @ref LL_RTC_MONTH_DECEMBER
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert month from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetMonth(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSDR, RTC_TSDR_MT | RTC_TSDR_MU) >> RTC_TSDR_MU_Pos);
}

/**
  * @brief  Get timestamp day in BCD format.
  * @rmtoll
  *  RTC_TSDR         DT            LL_RTC_TS_GetDay \n
  *  RTC_TSDR         DU            LL_RTC_TS_GetDay
  * @retval Value between Min_Data=0x01 and Max_Data=0x31
  * @note   Helper macro LL_RTC_CONVERT_BCD2BIN is available to convert day from BCD to binary format
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetDay(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSDR, RTC_TSDR_DT | RTC_TSDR_DU));
}

/**
  * @brief  Get timestamp date (weekday, day and month) in BCD format.
  * @rmtoll
  *  RTC_TSDR         WDU           LL_RTC_TS_GetDate \n
  *  RTC_TSDR         MT            LL_RTC_TS_GetDate \n
  *  RTC_TSDR         MU            LL_RTC_TS_GetDate \n
  *  RTC_TSDR         DT            LL_RTC_TS_GetDate \n
  *  RTC_TSDR         DU            LL_RTC_TS_GetDate
  * @retval Combination of weekday, day and month
  * @note   Helper macros LL_RTC_GET_WEEKDAY, LL_RTC_GET_MONTH, and LL_RTC_GET_DAY
  *         are available to get independently each parameter
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetDate(void)
{
  uint32_t temp;

  temp = STM32_READ_BIT(RTC->TSDR, (RTC_TSDR_WDU | RTC_TSDR_MT | RTC_TSDR_MU | RTC_TSDR_DT | RTC_TSDR_DU));
  return (uint32_t)((((temp & RTC_TSDR_WDU) >> RTC_TSDR_WDU_Pos) << RTC_OFFSET_WEEKDAY) | \
                    (((((temp & RTC_TSDR_DT) >> RTC_TSDR_DT_Pos) << 4U) | ((temp & RTC_TSDR_DU) >> RTC_TSDR_DU_Pos)) \
                     << RTC_OFFSET_DAY)   | (((((temp & RTC_TSDR_MT) >> RTC_TSDR_MT_Pos) << 4U) | \
                                              ((temp & RTC_TSDR_MU) >> RTC_TSDR_MU_Pos)) << RTC_OFFSET_MONTH));
}

/**
  * @brief  Get timestamp subsecond value.
  * @rmtoll
  *  RTC_TSSSR        SS            LL_RTC_TS_GetSubSecond
  * @retval If binary mode is not enabled, value between Min_Data=0x0 and Max_Data=0x7FFF
  *         else value between Min_Data=0x0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_TS_GetSubSecond(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->TSSSR, RTC_TSSSR_SS));
}
/**
  * @brief  Enable timestamp on tamper detection event.
  * @rmtoll
  *  RTC_CR       TAMPTS        LL_RTC_TS_EnableOnTamper
  */
__STATIC_INLINE void LL_RTC_TS_EnableOnTamper(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_TAMPTS);
}

/**
  * @brief  Disable timestamp on tamper detection event.
  * @rmtoll
  *  RTC_CR       TAMPTS        LL_RTC_TS_DisableOnTamper
  */
__STATIC_INLINE void LL_RTC_TS_DisableOnTamper(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_TAMPTS);
}

/**
  * @brief  Check whether timestamp on tamper detection is enabled.
  * @rmtoll
  *  RTC_CR           TSE          LL_RTC_TS_IsEnabledOnTamper
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_TS_IsEnabledOnTamper(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_TAMPTS) == (RTC_CR_TAMPTS)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Wakeup Wake-up
  * @{
  */

/**
  * @brief  Enable wake-up timer.
  * @rmtoll
  *  RTC_CR           WUTE          LL_RTC_WAKEUP_Enable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_WAKEUP_Enable(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_WUTE);
}

/**
  * @brief  Disable wake-up timer.
  * @rmtoll
  *  RTC_CR           WUTE          LL_RTC_WAKEUP_Disable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_WAKEUP_Disable(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_WUTE);
}

/**
  * @brief  Check whether wake-up timer is enabled.
  * @rmtoll
  *  RTC_CR           WUTE          LL_RTC_WAKEUP_IsEnabled
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_WAKEUP_IsEnabled(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_WUTE) == (RTC_CR_WUTE)) ? 1UL : 0UL);
}

/**
  * @brief  Start wake-up timer in interruption or polling mode.
  * @rmtoll
  *  RTC_CR          WUTE          LL_RTC_WAKEUP_Start \n
  *  RTC_CR          WUTIE         LL_RTC_WAKEUP_Start
  * @param  Interruption This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WAKEUP_TIMER_IT_DISABLE
  *         @arg @ref LL_RTC_WAKEUP_TIMER_IT_ENABLE
  */
__STATIC_INLINE void LL_RTC_WAKEUP_Start(uint32_t Interruption)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_WUTE | RTC_CR_WUTIE, RTC_CR_WUTE | Interruption);
}

/**
  * @brief  Stop wake-up timer.
  * @rmtoll
  *  RTC_CR          WUTE          LL_RTC_WAKEUP_Stop \n
  *  RTC_CR          WUTIE         LL_RTC_WAKEUP_Stop
  */
__STATIC_INLINE void LL_RTC_WAKEUP_Stop(void)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_WUTE | RTC_CR_WUTIE, 0U);
}

/**
  * @brief  Select wake-up clock.
  * @rmtoll
  *  RTC_CR           WUCKSEL       LL_RTC_WAKEUP_SetClock
  * @param  WakeupClock This parameter can be one of the following values:
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_16
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_8
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_4
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_2
  *         @arg @ref LL_RTC_WAKEUPCLOCK_CKSPRE
  *         @arg @ref LL_RTC_WAKEUPCLOCK_CKSPRE_WUT
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RTC_CR WUTE bit = 0 and RTC_ICSR WUTWF bit = 1
  */
__STATIC_INLINE void LL_RTC_WAKEUP_SetClock(uint32_t WakeupClock)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_WUCKSEL, WakeupClock);
}

/**
  * @brief  Get wake-up clock.
  * @rmtoll
  *  RTC_CR           WUCKSEL       LL_RTC_WAKEUP_GetClock
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_16
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_8
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_4
  *         @arg @ref LL_RTC_WAKEUPCLOCK_DIV_2
  *         @arg @ref LL_RTC_WAKEUPCLOCK_CKSPRE
  *         @arg @ref LL_RTC_WAKEUPCLOCK_CKSPRE_WUT
  */
__STATIC_INLINE uint32_t LL_RTC_WAKEUP_GetClock(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_WUCKSEL));
}

/**
  * @brief  Set wake-up auto-reload value.
  * @rmtoll
  *  RTC_WUTR         WUT           LL_RTC_WAKEUP_SetAutoReload
  * @param  Value Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @note   Bit can be written only when WUTWF is set to 1 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_WAKEUP_SetAutoReload(uint32_t Value)
{
  STM32_MODIFY_REG(RTC->WUTR, RTC_WUTR_WUT, Value);
}

/**
  * @brief  Get wake-up auto-reload value.
  * @rmtoll
  *  RTC_WUTR         WUT           LL_RTC_WAKEUP_GetAutoReload
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_WAKEUP_GetAutoReload(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->WUTR, RTC_WUTR_WUT));
}

/**
  * @brief  Set wake-up auto-reload clear value.
  * @rmtoll
  *  RTC_WUTR         WUTOCLR           LL_RTC_WAKEUP_SetAutoClear
  * @param  Value Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @note   Bit can be written only when WUTWF is set to 1 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_WAKEUP_SetAutoClear(uint32_t Value)
{
  STM32_MODIFY_REG(RTC->WUTR, RTC_WUTR_WUTOCLR, Value << RTC_WUTR_WUTOCLR_Pos);
}

/**
  * @brief  Get wake-up auto-reload clear value.
  * @rmtoll
  *  RTC_WUTR         WUTOCLR           LL_RTC_WAKEUP_GetAutoClear
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_WAKEUP_GetAutoClear(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->WUTR, RTC_WUTR_WUTOCLR) >> RTC_WUTR_WUTOCLR_Pos);
}

/**
  * @brief  Set wake-up auto-reload and auto-reload clear values.
  * @rmtoll
  *  RTC_WUTR         WUTOCLR           LL_RTC_WAKEUP_Config \n
  *  RTC_WUTR         WUT               LL_RTC_WAKEUP_Config
  * @param  Reload Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @param  Clear Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @note   Bit can be written only when WUTWF is set to 1 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_WAKEUP_Config(uint32_t Reload, uint32_t Clear)
{
  STM32_WRITE_REG(RTC->WUTR, ((Clear << RTC_WUTR_WUTOCLR_Pos) & RTC_WUTR_WUTOCLR_Msk) | (Reload & RTC_WUTR_WUT_Msk));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Calibration Calibration
  * @{
  */

/**
  * @brief  Set calibration output frequency (1 Hz or 512 Hz).
  * @rmtoll
  *  RTC_CR           COE           LL_RTC_CAL_SetOutputFreq \n
  *  RTC_CR           COSEL         LL_RTC_CAL_SetOutputFreq
  * @param  Frequency This parameter can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref LL_RTC_CALIB_OUTPUT_512HZ
  * @note   Bits are write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_CAL_SetOutputFreq(uint32_t Frequency)
{
  STM32_MODIFY_REG(RTC->CR, RTC_CR_COE | RTC_CR_COSEL, Frequency);
}

/**
  * @brief  Get calibration output frequency (1 Hz or 512 Hz).
  * @rmtoll
  *  RTC_CR           COE           LL_RTC_CAL_GetOutputFreq \n
  *  RTC_CR           COSEL         LL_RTC_CAL_GetOutputFreq
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_1HZ
  *         @arg @ref LL_RTC_CALIB_OUTPUT_512HZ
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_GetOutputFreq(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CR, RTC_CR_COE | RTC_CR_COSEL));
}

/**
  * @brief  Insert or do not insert one RTCCLK pulse every 2exp11 pulses (frequency increased by 488.5 ppm).
  * @rmtoll
  *  RTC_CALR         CALP          LL_RTC_CAL_SetPulse
  * @param  Pulse This parameter can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_INSERTPULSE_NONE
  *         @arg @ref LL_RTC_CALIB_INSERTPULSE_SET
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_CAL_SetPulse(uint32_t Pulse)
{
  STM32_MODIFY_REG(RTC->CALR, RTC_CALR_CALP, Pulse);
}

/**
  * @brief  Check whether one RTCCLK has been inserted every 2exp11 pulses (frequency increased by 488.5 ppm).
  * @rmtoll
  *  RTC_CALR         CALP          LL_RTC_CAL_IsPulseInserted
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_IsPulseInserted(void)
{
  return ((STM32_READ_BIT(RTC->CALR, RTC_CALR_CALP) == (RTC_CALR_CALP)) ? 1UL : 0UL);
}

/**
  * @brief  Set the calibration cycle period.
  * @rmtoll
  *  RTC_CALR         CALW8         LL_RTC_CAL_SetPeriod \n
  *  RTC_CALR         CALW16        LL_RTC_CAL_SetPeriod
  * @param  Period This parameter can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_PERIOD_32SEC
  *         @arg @ref LL_RTC_CALIB_PERIOD_16SEC
  *         @arg @ref LL_RTC_CALIB_PERIOD_8SEC
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_CAL_SetPeriod(uint32_t Period)
{
  STM32_MODIFY_REG(RTC->CALR, RTC_CALR_CALW8 | RTC_CALR_CALW16, Period);
}

/**
  * @brief  Get the calibration cycle period.
  * @rmtoll
  *  RTC_CALR         CALW8         LL_RTC_CAL_GetPeriod \n
  *  RTC_CALR         CALW16        LL_RTC_CAL_GetPeriod
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_PERIOD_32SEC
  *         @arg @ref LL_RTC_CALIB_PERIOD_16SEC
  *         @arg @ref LL_RTC_CALIB_PERIOD_8SEC
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_GetPeriod(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CALR, RTC_CALR_CALW8 | RTC_CALR_CALW16));
}

/**
  * @brief  Set calibration minus.
  * @rmtoll
  *  RTC_CALR         CALM          LL_RTC_CAL_SetMinus
  * @param  CalibMinus Value between Min_Data=0x00 and Max_Data=0x1FF
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RECALPF is set to 0 in RTC_ICSR
  */
__STATIC_INLINE void LL_RTC_CAL_SetMinus(uint32_t CalibMinus)
{
  STM32_MODIFY_REG(RTC->CALR, RTC_CALR_CALM, CalibMinus);
}

/**
  * @brief  Get calibration minus.
  * @rmtoll
  *  RTC_CALR         CALM          LL_RTC_CAL_GetMinus
  * @retval Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_GetMinus(void)
{
  return (uint32_t)(STM32_READ_BIT(RTC->CALR, RTC_CALR_CALM));
}

/**
  * @brief Set smooth calibration.
  * @rmtoll
  *  RTC_CALR         CALP          LL_RTC_CAL_SetSmoothCalibration \n
  *  RTC_CALR         CALW8         LL_RTC_CAL_SetSmoothCalibration \n
  *  RTC_CALR         CALW16        LL_RTC_CAL_SetSmoothCalibration \n
  *  RTC_CALR         CALM          LL_RTC_CAL_SetSmoothCalibration
  * @param SmoothCalibPeriod This parameter can be one of the following values:
  *        @arg @ref LL_RTC_CALIB_PERIOD_32SEC
  *        @arg @ref LL_RTC_CALIB_PERIOD_16SEC
  *        @arg @ref LL_RTC_CALIB_PERIOD_8SEC
  * @param SmoothCalibPlusPulses This parameter can be one of the following values:
  *        @arg @ref LL_RTC_CALIB_INSERTPULSE_NONE
  *        @arg @ref LL_RTC_CALIB_INSERTPULSE_SET
  * @param SmoothCalibMinusPulsesValue Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE void LL_RTC_CAL_SetSmoothCalibration(uint32_t SmoothCalibPeriod,
                                                     uint32_t SmoothCalibPlusPulses,
                                                     uint32_t SmoothCalibMinusPulsesValue)
{
  STM32_MODIFY_REG(RTC->CALR, (RTC_CALR_CALP | RTC_CALR_CALW8 | RTC_CALR_CALW16 | RTC_CALR_CALM),
                   (uint32_t)(SmoothCalibPeriod | SmoothCalibPlusPulses | SmoothCalibMinusPulsesValue));
}

/**
  * @brief  Check if the smooth calibration is enabled.
  * @rmtoll
  *  RTC_CALR         CALP          LL_RTC_CAL_IsEnabledSmoothCalibration \n
  *  RTC_CALR         CALM          LL_RTC_CAL_IsEnabledSmoothCalibration
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_IsEnabledSmoothCalibration(void)
{
  return ((STM32_READ_BIT(RTC->CALR, RTC_CALR_CALP | RTC_CALR_CALM) == 0U) ? 0UL : 1UL);
}

/**
  * @brief  Enable calibration low power.
  * @rmtoll
  *  RTC_CALR         LPCAL          LL_RTC_CAL_LowPower_Enable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RECALPF is set to 0
  */
__STATIC_INLINE void LL_RTC_CAL_LowPower_Enable(void)
{
  STM32_SET_BIT(RTC->CALR, RTC_CALR_LPCAL);
}

/**
  * @brief  Disable calibration low power.
  * @rmtoll
  *  RTC_CALR         LPCAL          LL_RTC_CAL_LowPower_Disable
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  * @note   Bit can be written only when RECALPF is set to 0
  */
__STATIC_INLINE void LL_RTC_CAL_LowPower_Disable(void)
{
  STM32_CLEAR_BIT(RTC->CALR, RTC_CALR_LPCAL);
}

/**
  * @brief  Check whether calibration low power is enabled.
  * @rmtoll
  *  RTC_CALR         LPCAL          LL_RTC_CAL_LowPower_IsEnabled
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_LowPower_IsEnabled(void)
{
  return ((STM32_READ_BIT(RTC->CALR, RTC_CALR_LPCAL) == (RTC_CALR_LPCAL)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get Recalibration pending flag.
  * @rmtoll
  *  RTC_ICSR          RECALPF       LL_RTC_IsActiveFlag_RECALP
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_RECALP(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_RECALPF) == (RTC_ICSR_RECALPF)) ? 1UL : 0UL);
}

/**
  * @brief  Get timestamp flag.
  * @rmtoll
  *  RTC_SR          TSF           LL_RTC_IsActiveFlag_TS
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TS(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_TSF) == (RTC_SR_TSF)) ? 1UL : 0UL);
}

/**
  * @brief  Get timestamp overflow flag.
  * @rmtoll
  *  RTC_SR          TSOVF         LL_RTC_IsActiveFlag_TSOV
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TSOV(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_TSOVF) == (RTC_SR_TSOVF)) ? 1UL : 0UL);
}

/**
  * @brief  Get wake-up timer flag.
  * @rmtoll
  *  RTC_SR          WUTF          LL_RTC_IsActiveFlag_WUT
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_WUT(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_WUTF) == (RTC_SR_WUTF)) ? 1UL : 0UL);
}

/**
  * @brief  Get alarm A flag.
  * @rmtoll
  *  RTC_SR          ALRAF         LL_RTC_IsActiveFlag_ALRA
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALRA(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_ALRAF) == (RTC_SR_ALRAF)) ? 1UL : 0UL);
}

/**
  * @brief  Get alarm B flag.
  * @rmtoll
  *  RTC_SR          ALRBF         LL_RTC_IsActiveFlag_ALRB
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALRB(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_ALRBF) == (RTC_SR_ALRBF)) ? 1UL : 0UL);
}

/**
  * @brief  Get selected alarm flag.
  * @rmtoll
  *  RTC_SR          ALRBF         LL_RTC_IsActiveFlag_ALR \n
  *  RTC_SR          ALRAF         LL_RTC_IsActiveFlag_ALR
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A
  *         @arg @ref LL_RTC_ALARM_B
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALR(const uint32_t Alarm)
{
  return ((STM32_READ_BIT(RTC->SR, (RTC_SR_ALRAF << Alarm)) == (RTC_SR_ALRAF << Alarm)) ? 1UL : 0UL);
}

/**
  * @brief  Get SSR Underflow flag.
  * @rmtoll
  *  RTC_SR          SSRUF         LL_RTC_IsActiveFlag_SSRU
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_SSRU(void)
{
  return ((STM32_READ_BIT(RTC->SR, RTC_SR_SSRUF) == (RTC_SR_SSRUF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Timestamp flag.
  * @rmtoll
  *  RTC_SR         CTSF           LL_RTC_ClearFlag_TS
  */
__STATIC_INLINE void LL_RTC_ClearFlag_TS(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CTSF);
}

/**
  * @brief  Clear Timestamp overflow flag.
  * @rmtoll
  *  RTC_SCR          CTSOVF         LL_RTC_ClearFlag_TSOV
  */
__STATIC_INLINE void LL_RTC_ClearFlag_TSOV(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CTSOVF);
}

/**
  * @brief  Clear wake-up timer flag.
  * @rmtoll
  *  RTC_SCR          CWUTF          LL_RTC_ClearFlag_WUT
  */
__STATIC_INLINE void LL_RTC_ClearFlag_WUT(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);
}

/**
  * @brief  Clear alarm A flag.
  * @rmtoll
  *  RTC_SCR          CALRAF         LL_RTC_ClearFlag_ALRA
  */
__STATIC_INLINE void LL_RTC_ClearFlag_ALRA(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CALRAF);
}

/**
  * @brief  Clear alarm B flag.
  * @rmtoll
  *  RTC_SCR          CALRBF         LL_RTC_ClearFlag_ALRB
  */
__STATIC_INLINE void LL_RTC_ClearFlag_ALRB(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CALRBF);
}

/**
  * @brief  Clear the selected alarm flag.
  * @rmtoll
  *         RTC_SCR          CALRBF         LL_RTC_ClearFlag_ALR \n
  *  RTC_SCR          CALRAF         LL_RTC_ClearFlag_ALR
  * @param  Alarm This parameter can be one of the following values:
  *         @arg @ref LL_RTC_ALARM_A Alarm A
  *         @arg @ref LL_RTC_ALARM_B Alarm B
  */
__STATIC_INLINE void LL_RTC_ClearFlag_ALR(uint32_t Alarm)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CALRAF << Alarm);
}

/**
  * @brief  Clear SSR Underflow flag.
  * @rmtoll
  *  RTC_SCR          CSSRUF         LL_RTC_ClearFlag_SSRU
  */
__STATIC_INLINE void LL_RTC_ClearFlag_SSRU(void)
{
  STM32_WRITE_REG(RTC->SCR, RTC_SCR_CSSRUF);
}

/**
  * @brief  Get Initialization flag.
  * @rmtoll
  *  RTC_ICSR          INITF         LL_RTC_IsActiveFlag_INIT
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_INIT(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_INITF) == (RTC_ICSR_INITF)) ? 1UL : 0UL);
}

/**
  * @brief  Get Registers synchronization flag.
  * @rmtoll
  *  RTC_ICSR          RSF           LL_RTC_IsActiveFlag_RS
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_RS(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_RSF) == (RTC_ICSR_RSF)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Registers synchronization flag.
  * @rmtoll
  *  RTC_ICSR          RSF           LL_RTC_ClearFlag_RS
  */
__STATIC_INLINE void LL_RTC_ClearFlag_RS(void)
{
  STM32_CLEAR_BIT(RTC->ICSR, RTC_ICSR_RSF);
}

/**
  * @brief  Get Initialization status flag.
  * @rmtoll
  *  RTC_ICSR          INITS         LL_RTC_IsActiveFlag_INITS
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_INITS(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_INITS) == (RTC_ICSR_INITS)) ? 1UL : 0UL);
}

/**
  * @brief  Get Shift operation pending flag.
  * @rmtoll
  *  RTC_ICSR          SHPF          LL_RTC_IsActiveFlag_SHP
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_SHP(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_SHPF) == (RTC_ICSR_SHPF)) ? 1UL : 0UL);
}

/**
  * @brief  Get wake-up timer write flag.
  * @rmtoll
  *  RTC_ICSR          WUTWF         LL_RTC_IsActiveFlag_WUTW
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_WUTW(void)
{
  return ((STM32_READ_BIT(RTC->ICSR, RTC_ICSR_WUTWF) == (RTC_ICSR_WUTWF)) ? 1UL : 0UL);
}

/**
  * @brief  Get alarm A masked flag.
  * @rmtoll
  *  RTC_MISR          ALRAMF        LL_RTC_IsActiveFlag_ALRAM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALRAM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_ALRAMF) == (RTC_MISR_ALRAMF)) ? 1UL : 0UL);
}

/**
  * @brief  Get SSR Underflow masked flag.
  * @rmtoll
  *  RTC_MISR          SSRUMF        LL_RTC_IsActiveFlag_SSRUM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_SSRUM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_SSRUMF) == (RTC_MISR_SSRUMF)) ? 1UL : 0UL);
}

/**
  * @brief  Get alarm B masked flag.
  * @rmtoll
  *  RTC_MISR          ALRBMF        LL_RTC_IsActiveFlag_ALRBM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALRBM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_ALRBMF) == (RTC_MISR_ALRBMF)) ? 1UL : 0UL);
}

/**
  * @brief  Get wake-up timer masked flag.
  * @rmtoll
  *  RTC_MISR          WUTMF        LL_RTC_IsActiveFlag_WUTM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_WUTM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_WUTMF) == (RTC_MISR_WUTMF)) ? 1UL : 0UL);
}

/**
  * @brief  Get timestamp masked flag.
  * @rmtoll
  *  RTC_MISR          TSMF        LL_RTC_IsActiveFlag_TSM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TSM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_TSMF) == (RTC_MISR_TSMF)) ? 1UL : 0UL);
}

/**
  * @brief  Get timestamp overflow masked flag.
  * @rmtoll
  *  RTC_MISR          TSOVMF        LL_RTC_IsActiveFlag_TSOVM
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TSOVM(void)
{
  return ((STM32_READ_BIT(RTC->MISR, RTC_MISR_TSOVMF) == (RTC_MISR_TSOVMF)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Privilege_Services Privilege Services
  * @{
  */

/**
  * @brief  Set privilege attribute configuration.
  * @rmtoll
  *  PRIVCFGR           ALRAPRIV            LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           ALRBPRIV            LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           WUTPRIV             LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           TSPRIV              LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           CALPRIV             LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           INITPRIV            LL_RTC_SetPrivAttr \n
  *  PRIVCFGR           PRIV                LL_RTC_SetPrivAttr
  * @param item This parameter can be one or a combination of the following values:
  *        @arg @ref LL_RTC_PRIV_ITEM_ALRAPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_ALRBPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_WUTPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_TSPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_CALPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_INITPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_PRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_ALL
  * @param priv_attr This parameter can be one of the following values:
  *        @arg @ref LL_RTC_ATTR_PRIV
  *        @arg @ref LL_RTC_ATTR_NPRIV
  */
__STATIC_INLINE void LL_RTC_SetPrivAttr(uint32_t item, uint32_t priv_attr)
{
  STM32_MODIFY_REG(RTC->PRIVCFGR, item, (item & ((~priv_attr) + 1U)));
}

/**
  * @brief  Get privilege attribute configuration.
  * @rmtoll
  *  PRIVCFGR           ALRAPRIV            LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           ALRBPRIV            LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           WUTPRIV             LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           TSPRIV              LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           CALPRIV             LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           INITPRIV            LL_RTC_GetPrivAttr \n
  *  PRIVCFGR           PRIV                LL_RTC_GetPrivAttr
  * @param  item This parameter can be one of the following values:
  *        @arg @ref LL_RTC_PRIV_ITEM_ALRAPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_ALRBPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_WUTPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_TSPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_CALPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_INITPRIV
  *        @arg @ref LL_RTC_PRIV_ITEM_PRIV
  * @retval Status of bit (1 or 0).
  *         - 1: Privileged attribute enabled for the item.
  *         - 0: Non-privileged attribute for the item.
  */
__STATIC_INLINE uint32_t LL_RTC_GetPrivAttr(uint32_t item)
{
  return ((STM32_READ_BIT(RTC->PRIVCFGR, item) == (item)) ? LL_RTC_ATTR_PRIV : LL_RTC_ATTR_NPRIV);
}
/**
  * @}
  */

/** @defgroup RTC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable timestamp interrupt.
  * @rmtoll
  *  RTC_CR          TSIE         LL_RTC_EnableIT_TS
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_EnableIT_TS(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_TSIE);
}

/**
  * @brief  Disable timestamp interrupt.
  * @rmtoll
  *  RTC_CR          TSIE         LL_RTC_DisableIT_TS
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_DisableIT_TS(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_TSIE);
}

/**
  * @brief  Enable wake-up timer interrupt.
  * @rmtoll
  *  RTC_CR          WUTIE         LL_RTC_EnableIT_WUT
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_EnableIT_WUT(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_WUTIE);
}

/**
  * @brief  Disable wake-up timer interrupt.
  * @rmtoll
  *  RTC_CR          WUTIE         LL_RTC_DisableIT_WUT
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_DisableIT_WUT(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_WUTIE);
}

/**
  * @brief  Enable alarm B interrupt.
  * @rmtoll
  *  RTC_CR           ALRBIE        LL_RTC_EnableIT_ALRB
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_EnableIT_ALRB(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_ALRBIE);
}

/**
  * @brief  Disable alarm B interrupt.
  * @rmtoll
  *  RTC_CR           ALRBIE        LL_RTC_DisableIT_ALRB
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_DisableIT_ALRB(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_ALRBIE);
}

/**
  * @brief  Enable alarm A interrupt.
  * @rmtoll
  *  RTC_CR           ALRAIE        LL_RTC_EnableIT_ALRA
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_EnableIT_ALRA(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_ALRAIE);
}

/**
  * @brief  Disable alarm A interrupt.
  * @rmtoll
  *  RTC_CR           ALRAIE        LL_RTC_DisableIT_ALRA
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_DisableIT_ALRA(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_ALRAIE);
}

/**
  * @brief  Enable SSR underflow interrupt.
  * @rmtoll
  *  RTC_CR           SSRUIE        LL_RTC_EnableIT_SSRU
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_EnableIT_SSRU(void)
{
  STM32_SET_BIT(RTC->CR, RTC_CR_SSRUIE);
}

/**
  * @brief  Disable SSR underflow interrupt.
  * @rmtoll
  *  RTC_CR           SSRUIE        LL_RTC_DisableIT_SSRU
  * @note   Bit is write-protected. Call @ref LL_RTC_DisableWriteProtection before.
  */
__STATIC_INLINE void LL_RTC_DisableIT_SSRU(void)
{
  STM32_CLEAR_BIT(RTC->CR, RTC_CR_SSRUIE);
}

/**
  * @brief  Check whether timestamp interrupt is enabled.
  * @rmtoll
  *  RTC_CR           TSIE          LL_RTC_IsEnabledIT_TS
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_TS(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_TSIE) == (RTC_CR_TSIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether wake-up timer interrupt is enabled.
  * @rmtoll
  *  RTC_CR           WUTIE         LL_RTC_IsEnabledIT_WUT
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_WUT(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_WUTIE) == (RTC_CR_WUTIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether alarm A interrupt is enabled.
  * @rmtoll
  *  RTC_CR           ALRAIE        LL_RTC_IsEnabledIT_ALRA
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_ALRA(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_ALRAIE) == (RTC_CR_ALRAIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether alarm B interrupt is enabled.
  * @rmtoll
  *  RTC_CR           ALRBIE        LL_RTC_IsEnabledIT_ALRB
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_ALRB(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_ALRBIE) == (RTC_CR_ALRBIE)) ? 1UL : 0UL);
}

/**
  * @brief  Check whether SSR underflow interrupt is enabled.
  * @rmtoll
  *  RTC_CR           SSRUIE        LL_RTC_IsEnabledIT_SSRU
  * @retval State of bit (1 or 0)
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_SSRU(void)
{
  return ((STM32_READ_BIT(RTC->CR, RTC_CR_SSRUIE) == (RTC_CR_SSRUIE)) ? 1UL : 0UL);
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

#endif /* defined(RTC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5xx_LL_RTC_H */
