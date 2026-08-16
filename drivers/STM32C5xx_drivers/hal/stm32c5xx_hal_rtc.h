/**
  ******************************************************************************
  * @file    stm32c5xx_hal_rtc.h
  * @brief   Header file of RTC HAL module.
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

#ifndef STM32C5xx_HAL_RTC_H
#define STM32C5xx_HAL_RTC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_rtc.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup RTC RTC
  * @{
  */

/** @defgroup RTC_Exported_Types HAL RTC types
  * @{
  */

/* Global exported enumeration ---------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Enums_Config RTC exported global configuration enumerations.
  * @{
  */

/**
  * @brief RTC mode definitions.
  */
typedef enum
{
  HAL_RTC_MODE_BCD    = LL_RTC_BINARY_NONE, /*!< RTC is set to BCD mode only */
  HAL_RTC_MODE_BINARY = LL_RTC_BINARY_ONLY, /*!< RTC is set to binary mode only */
  HAL_RTC_MODE_MIX    = LL_RTC_BINARY_MIX   /*!< RTC is set to mixed mode (BCD and binary mode together) */
} hal_rtc_mode_t;

/**
  * @brief BCD update using the subseconds register (SS) least significant bits.
  *        This mode is also used to feed the wake-up timer based on WUCKSEL[2:0] when
  *        the RTC mode is HAL_RTC_MODE_BINARY or HAL_RTC_MODE_MIX.
  */
typedef enum
{
  HAL_RTC_BCD_UPDATE_8BITS  = LL_RTC_BINARY_MIX_BCDU_0, /*!< Incremented when SS[7:0] = 0 */
  HAL_RTC_BCD_UPDATE_9BITS  = LL_RTC_BINARY_MIX_BCDU_1, /*!< Incremented when SS[8:0] = 0 */
  HAL_RTC_BCD_UPDATE_10BITS = LL_RTC_BINARY_MIX_BCDU_2, /*!< Incremented when SS[9:0] = 0 */
  HAL_RTC_BCD_UPDATE_11BITS = LL_RTC_BINARY_MIX_BCDU_3, /*!< Incremented when SS[10:0] = 0 */
  HAL_RTC_BCD_UPDATE_12BITS = LL_RTC_BINARY_MIX_BCDU_4, /*!< Incremented when SS[11:0] = 0 */
  HAL_RTC_BCD_UPDATE_13BITS = LL_RTC_BINARY_MIX_BCDU_5, /*!< Incremented when SS[12:0] = 0 */
  HAL_RTC_BCD_UPDATE_14BITS = LL_RTC_BINARY_MIX_BCDU_6, /*!< Incremented when SS[13:0] = 0 */
  HAL_RTC_BCD_UPDATE_15BITS = LL_RTC_BINARY_MIX_BCDU_7  /*!< Incremented when SS[14:0] = 0 */
} hal_rtc_bcd_update_t;

/**
  * @}
  */

/** @defgroup RTC_Exported_Enums_Low_Power RTC exported low power configuration enumerations.
  * @{
  */

/**
  * @brief Ultra low power status definitions.
  */
typedef enum
{
  HAL_RTC_ULTRA_LOW_POWER_DISABLED = 0U, /*!< Ultra low power disabled */
  HAL_RTC_ULTRA_LOW_POWER_ENABLED  = 1U  /*!< Ultra low power enabled */
} hal_rtc_ultra_low_power_mode_status_t;

/**
  * @}
  */

/* Calendar exported enumerations ------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Enums_Calendar RTC exported calendar enumerations.
  * @{
  */

/**
  * @brief Hour formats definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_HOUR_FORMAT_24   = LL_RTC_HOUR_FORMAT_24HOUR, /*!< 24-hour format */
  HAL_RTC_CALENDAR_HOUR_FORMAT_AMPM = LL_RTC_HOUR_FORMAT_AMPM    /*!< 12-hour format */
} hal_rtc_calendar_hour_format_t;

/**
  * @brief Bypass shadow register definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_SHADOW_REG_KEEP   = LL_RTC_SHADOW_REG_KEEP,  /*!< Keep shadow registers */
  HAL_RTC_CALENDAR_SHADOW_REG_BYPASS = LL_RTC_SHADOW_REG_BYPASS /*!< Bypass shadow registers */
} hal_rtc_calendar_shadow_reg_bypass_t;

/**
  * @brief Reference clock definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_REF_CLOCK_DISABLED = 0U, /*!< Reference clock disabled */
  HAL_RTC_CALENDAR_REF_CLOCK_ENABLED  = 1U  /*!< Reference clock enabled */
} hal_rtc_calendar_reference_clock_status_t;

/**
  * @brief Summer time definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_SUMMER_TIME_DISABLED = 0U, /*!< Summer time disabled */
  HAL_RTC_CALENDAR_SUMMER_TIME_ENABLED  = 1U  /*!< Summer time enabled */
} hal_rtc_calendar_summer_time_status_t;

/**
  * @brief Calendar mode interruption underflow status definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_IT_UNDERFLOW_DISABLED = 0U, /*!< Subseconds counter underflow interruption disabled */
  HAL_RTC_CALENDAR_IT_UNDERFLOW_ENABLED  = 1U  /*!< Subseconds counter underflow interruption enabled */
} hal_rtc_calendar_it_underflow_status_t;

/**
  * @}
  */

/* Date and time  exported enumerations ------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Enums_Date_Time RTC exported date and time enumerations.
  * @{
  */

/**
  * @brief AM PM definitions.
  */
typedef enum
{
  HAL_RTC_TIME_FORMAT_AM_24H = LL_RTC_TIME_FORMAT_AM_24H, /*!< If 12-hour format, hour is a.m. */
  HAL_RTC_TIME_FORMAT_PM     = LL_RTC_TIME_FORMAT_PM      /*!< Hour is p.m. */
} hal_rtc_time_format_am_pm_t;

/**
  * @brief  Month definitions.
  */
typedef enum
{
  HAL_RTC_MONTH_JANUARY   = LL_RTC_MONTH_JANUARY,   /*!< January */
  HAL_RTC_MONTH_FEBRUARY  = LL_RTC_MONTH_FEBRUARY,  /*!< February */
  HAL_RTC_MONTH_MARCH     = LL_RTC_MONTH_MARCH,     /*!< March */
  HAL_RTC_MONTH_APRIL     = LL_RTC_MONTH_APRIL,     /*!< April */
  HAL_RTC_MONTH_MAY       = LL_RTC_MONTH_MAY,       /*!< May */
  HAL_RTC_MONTH_JUNE      = LL_RTC_MONTH_JUNE,      /*!< June */
  HAL_RTC_MONTH_JULY      = LL_RTC_MONTH_JULY,      /*!< July */
  HAL_RTC_MONTH_AUGUST    = LL_RTC_MONTH_AUGUST,    /*!< August */
  HAL_RTC_MONTH_SEPTEMBER = LL_RTC_MONTH_SEPTEMBER, /*!< September */
  HAL_RTC_MONTH_OCTOBER   = LL_RTC_MONTH_OCTOBER,   /*!< October */
  HAL_RTC_MONTH_NOVEMBER  = LL_RTC_MONTH_NOVEMBER,  /*!< November */
  HAL_RTC_MONTH_DECEMBER  = LL_RTC_MONTH_DECEMBER   /*!< December */
} hal_rtc_month_t;

/**
  * @brief Weekday definitions.
  */
typedef enum
{
  HAL_RTC_WEEKDAY_MONDAY    = LL_RTC_WEEKDAY_MONDAY,    /*!< Monday */
  HAL_RTC_WEEKDAY_TUESDAY   = LL_RTC_WEEKDAY_TUESDAY,   /*!< Tuesday */
  HAL_RTC_WEEKDAY_WEDNESDAY = LL_RTC_WEEKDAY_WEDNESDAY, /*!< Wednesday */
  HAL_RTC_WEEKDAY_THURSDAY  = LL_RTC_WEEKDAY_THURSDAY,  /*!< Thursday */
  HAL_RTC_WEEKDAY_FRIDAY    = LL_RTC_WEEKDAY_FRIDAY,    /*!< Friday */
  HAL_RTC_WEEKDAY_SATURDAY  = LL_RTC_WEEKDAY_SATURDAY,  /*!< Saturday */
  HAL_RTC_WEEKDAY_SUNDAY    = LL_RTC_WEEKDAY_SUNDAY     /*!< Sunday */
} hal_rtc_weekday_t;

/**
  * @}
  */

/** @defgroup RTC_Exported_Enums_Init  RTC exported calendar init enumerations
  * @{
  */

/**
  * @brief RTC calendar initialization status definitions.
  */
typedef enum
{
  HAL_RTC_CALENDAR_NOT_INITIALIZED = 0U, /*!< RTC is not initialized */
  HAL_RTC_CALENDAR_INITIALIZED     = 1U  /*!< RTC is initialized */
} hal_rtc_calendar_status_t;

/**
  * @}
  */

/* Output exported enumerations --------------------------------------------------------------------------------------*/

/** @defgroup RTC_Exported_Enums_Output RTC exported output enumerations
  * @{
  */

/**
  * @brief Tampalarm output polarity definitions.
  */
typedef enum
{
  HAL_RTC_OUTPUT_TAMPALARM_POLARITY_HIGH = LL_RTC_OUTPUTPOLARITY_PIN_HIGH, /*!< Tampalarm output polarity is high */
  HAL_RTC_OUTPUT_TAMPALARM_POLARITY_LOW  = LL_RTC_OUTPUTPOLARITY_PIN_LOW   /*!< Tampalarm output polarity is low */
} hal_rtc_output_tampalarm_polarity_t;

/**
  * @brief  Tampalarm output type definitions.
  *
  */
typedef enum
{
  HAL_RTC_OUTPUT_TAMPALARM_TYPE_PUSHPULL  = LL_RTC_ALARM_OUTPUTTYPE_PUSHPULL,  /*!< Tampalarm output is a push-pull */
  HAL_RTC_OUTPUT_TAMPALARM_TYPE_OPENDRAIN = LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN, /*!< Tampalarm output is an open-drain */
} hal_rtc_output_tampalarm_type_t;

/**
  * @brief Tampalarm output pull-up definitions.
  */
typedef enum
{
  HAL_RTC_OUTPUT_TAMPALARM_PULLUP_DISABLE = LL_RTC_ALARM_OUTPUT_PULLUP_NONE, /*!< Tampalarm output pull-up disable */
  HAL_RTC_OUTPUT_TAMPALARM_PULLUP_ENABLE  = LL_RTC_ALARM_OUTPUT_PULLUP_ON    /*!< Tampalarm output pull-up enable */
} hal_rtc_output_tampalarm_pullup_t;

/**
  * @brief  Calibration output frequency definitions.
  */
typedef enum
{
  HAL_RTC_OUTPUT_CALIBRATION_SYNCHRONOUS        = LL_RTC_CALIB_FREQUENCY_1HZ,  /*!< 1Hz for RTCCLK at 32768 Hz and prescalers at their default values (PREDIV_A = 127 and PREDIV_S = 255) */
  HAL_RTC_OUTPUT_CALIBRATION_ASYNCHRONOUS_DIV64 = LL_RTC_CALIB_FREQUENCY_512HZ /*!< 512Hz for RTCCLK at 32768 Hz and prescalers at their default values (PREDIV_A = 127 and PREDIV_S = 255) */
} hal_rtc_output_calibration_frequency_t;

/**
  * @brief Output definitions.
  */
typedef enum
{
  HAL_RTC_OUTPUT_OUT1_ALARMA                 = LL_RTC_ALARMOUT_ALARM_A,  /*!< Alarm A event is routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_ALARMB                 = LL_RTC_ALARMOUT_ALARM_B,  /*!< Alarm B event is routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_WAKEUP                 = LL_RTC_ALARMOUT_WAKEUP,  /*!< Wake-up timer event is routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_TAMP                   = LL_RTC_OUTPUT_TAMPER_ENABLE,  /*!< Tamper event is routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_CALIB                  = LL_RTC_CALIB_OUTPUT_ENABLE,  /*!< Calibration is routed to output 1 */
  HAL_RTC_OUTPUT_OUT2_ALARMA                 = (LL_RTC_ALARMOUT_ALARM_A | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm A event is routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_ALARMB                 = (LL_RTC_ALARMOUT_ALARM_B | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm B event is routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_WAKEUP                 = (LL_RTC_ALARMOUT_WAKEUP | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Wake-up timer event is routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_TAMP                   = (LL_RTC_OUTPUT_TAMPER_ENABLE | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Tamper event is routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_CALIB                  = (LL_RTC_CALIB_OUTPUT_ENABLE | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Calibration is routed to output 2 */
  HAL_RTC_OUTPUT_OUT1_ALARMA_TAMP            = (LL_RTC_ALARMOUT_ALARM_A | LL_RTC_OUTPUT_TAMPER_ENABLE),  /*!< Alarm A and tamper event are routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_ALARMB_TAMP            = (LL_RTC_ALARMOUT_ALARM_B | LL_RTC_OUTPUT_TAMPER_ENABLE),  /*!< Alarm B and tamper event are routed to output 1 */
  HAL_RTC_OUTPUT_OUT1_WAKEUP_TAMP            = (LL_RTC_ALARMOUT_WAKEUP | LL_RTC_OUTPUT_TAMPER_ENABLE),  /*!< Wake-up timer and tamper event are routed to output 1 */
  HAL_RTC_OUTPUT_OUT2_ALARMA_TAMP            = (LL_RTC_ALARMOUT_ALARM_A | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),   /*!< Alarm A and tamper event are routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_ALARMB_TAMP            = (LL_RTC_ALARMOUT_ALARM_B | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),   /*!< Alarm B and tamper event are routed to output 2 */
  HAL_RTC_OUTPUT_OUT2_WAKEUP_TAMP            = (LL_RTC_ALARMOUT_WAKEUP | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),   /*!< Wake-up timer and tamper event are routed to output 2 */
  HAL_RTC_OUTPUT_OUT1_ALARMA_OUT2_CALIB      = (LL_RTC_ALARMOUT_ALARM_A | LL_RTC_CALIB_OUTPUT_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm A event is routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_ALARMB_OUT2_CALIB      = (LL_RTC_ALARMOUT_ALARM_B | LL_RTC_CALIB_OUTPUT_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm B event is routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_WAKEUP_OUT2_CALIB      = (LL_RTC_ALARMOUT_WAKEUP | LL_RTC_CALIB_OUTPUT_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Wake-up timer event is routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_TAMP_OUT2_CALIB        = (LL_RTC_OUTPUT_TAMPER_ENABLE | LL_RTC_CALIB_OUTPUT_ENABLE |
                                                LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Tamper event is routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_ALARMA_TAMP_OUT2_CALIB = (LL_RTC_ALARMOUT_ALARM_A | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_CALIB_OUTPUT_ENABLE | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm A and tamper event are routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_ALARMB_TAMP_OUT2_CALIB = (LL_RTC_ALARMOUT_ALARM_B | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_CALIB_OUTPUT_ENABLE | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Alarm B and tamper event are routed to output 1 and calibration to output 2 */
  HAL_RTC_OUTPUT_OUT1_WAKEUP_TAMP_OUT2_CALIB = (LL_RTC_ALARMOUT_WAKEUP | LL_RTC_OUTPUT_TAMPER_ENABLE |
                                                LL_RTC_CALIB_OUTPUT_ENABLE | LL_RTC_ALARM_OUTPUT_REMAP_POS1),  /*!< Wake-up timer and tamper event are routed to output 1 and calibration to output 2 */
} hal_rtc_output_t;

/**
  * @brief Output status definitions.
  */
typedef enum
{
  HAL_RTC_OUTPUT_DISABLED = 0U, /*!< Output disabled */
  HAL_RTC_OUTPUT_ENABLED  = 1U  /*!< Output enabled */
} hal_rtc_output_status_t;

/**
  * @}
  */

/* Alarm exported enumerations ---------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Enums_Alarm RTC exported alarm enumerations
  * @{
  */

/**
  * @brief Alarm date weekday definitions.
  */
typedef enum
{
  HAL_RTC_ALARM_DAY_TYPE_SEL_MONTHDAY = LL_RTC_ALMA_DATEWEEKDAYSEL_DATE, /*!< Alarm day corresponds to the month day */
  HAL_RTC_ALARM_DAY_TYPE_SEL_WEEKDAY  = LL_RTC_ALMA_DATEWEEKDAYSEL_WEEKDAY /*!< Alarm day corresponds to the weekday */
} hal_rtc_alarm_day_type_selection_t;

/**
  * Alarm definitions.
  */
typedef enum
{
  HAL_RTC_ALARM_A = LL_RTC_ALARM_A, /*!< Alarm A */
  HAL_RTC_ALARM_B = LL_RTC_ALARM_B  /*!< Alarm B */
} hal_rtc_alarm_t;

/**
  * @brief Alarm flag autoclear definitions.
  */
typedef enum
{
  HAL_ALARM_AUTO_CLEAR_DISABLE = LL_RTC_ALM_AUTOCLR_NO, /*!< Autoclear of the alarm flag is disabled */
  HAL_ALARM_AUTO_CLEAR_ENABLE  = LL_RTC_ALM_AUTOCLR_YES /*!< Autoclear of the alarm flag is enabled */
} hal_rtc_alarm_auto_clear_t;

/**
  * @brief Subseconds register auto-reload on alarm definitions. Enable only in binary mode.
  */
typedef enum
{
  HAL_RTC_ALARM_SUBSECONDS_AUTO_RELOAD_DISABLE = LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_NO, /*!< Disables the autoreload of calendar subseconds register */
  HAL_RTC_ALARM_SUBSECONDS_AUTO_RELOAD_ENABLE = LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES  /*!< Enables the autoreload of the subseconds register */
} hal_rtc_alarm_subseconds_auto_reload_t;

/**
  * @}
  */

/* Timestamp exported enumerations -----------------------------------------------------------------------------------*/

/** @defgroup RTC_Exported_Enums_Timestamp RTC exported timestamp enumerations.
  * @{
  */

/**
  * @brief Timestamp event on pin active edge definitions.
  */
typedef enum
{
  HAL_RTC_TIMESTAMP_EDGE_RISING  = LL_RTC_TIMESTAMP_EDGE_RISING, /*!< Create a timestamp event when a rising edge is detected in the input pin */
  HAL_RTC_TIMESTAMP_EDGE_FALLING = LL_RTC_TIMESTAMP_EDGE_FALLING /*!< Create a timestamp event when a falling edge is detected in the input pin */
} hal_rtc_timestamp_source_pin_edge_t;

/**
  * @brief Timestamp interruption status.
  */
typedef enum
{
  HAL_RTC_TIMESTAMP_IT_DISABLED = 0U, /*!< Timestamp interruption disabled */
  HAL_RTC_TIMESTAMP_IT_ENABLED  = 1U  /*!< Timestamp interruption enabled */
} hal_rtc_timestamp_it_status_t;

/**
  * @brief Timestamp on tamper status.
  */
typedef enum
{
  HAL_RTC_TIMESTAMP_TAMPER_DISABLED = 0U, /*!< Timestamp on tamper disabled */
  HAL_RTC_TIMESTAMP_TAMPER_ENABLED  = 1U  /*!< Timestamp on tamper enabled */
} hal_rtc_timestamp_tamper_status_t;


/**
  * @brief Timestamp status.
  */
typedef enum
{
  HAL_RTC_TIMESTAMP_DISABLED = 0U, /*!< Timestamp disabled */
  HAL_RTC_TIMESTAMP_ENABLED  = 1U  /*!< Timestamp enabled */
} hal_rtc_timestamp_status_t;

/**
  * @brief Timestamp event definitions.
  */
typedef enum
{
  HAL_RTC_TIMESTAMP_NO_EVENT       = 0U,               /*!< No timestamp event */
  HAL_RTC_TIMESTAMP_EVENT          = LL_RTC_SR_TSF,    /*!< Timestamp event */
  HAL_RTC_TIMESTAMP_OVERFLOW_EVENT = LL_RTC_SR_TSOVF,  /*!< Timestamp overflow event */
} hal_rtc_timestamp_event_flag_t;


/**
  * @}
  */

/* Wake-up timer exported enumerations -------------------------------------------------------------------------------*/

/** @defgroup RTC_Exported_Enums_WakeUp_Timer RTC exported wake-up timer enumerations.
  * @{
  */

/**
  * @brief Wake-up timer clock definitions.
  */
typedef enum
{
  HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV2   = LL_RTC_WAKEUPCLOCK_DIV_2,  /*!< Wakeup timer clock is the RTCCLK divided by 2 */
  HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV4   = LL_RTC_WAKEUPCLOCK_DIV_4,  /*!< Wakeup timer clock is the RTCCLK divided by 4 */
  HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV8   = LL_RTC_WAKEUPCLOCK_DIV_8,  /*!< Wakeup timer clock is the RTCCLK divided by 8 */
  HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV16  = LL_RTC_WAKEUPCLOCK_DIV_16, /*!< Wakeup timer clock is the RTCCLK divided by 16 */
  HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE    = LL_RTC_WAKEUPCLOCK_CKSPRE, /*!< Wakeup timer clock is the BCD update */
  HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT = LL_RTC_WAKEUPCLOCK_CKSPRE_WUT /*!< Wakeup timer clock is the BCD update and 1 bit is added to the wakeup timer */
} hal_rtc_wakeup_timer_clock_t;

/**
  * @}
  */

/** @defgroup RTC_Exported_Enums_Calibration RTC exported calibration enumerations.
  * @{
  */

/**
  * @brief Calibration cycle period definitions.
  */
typedef enum
{
  HAL_RTC_CALIBRATION_PERIOD_8SEC  = LL_RTC_CALIB_PERIOD_8SEC,  /*!< Calibration cycle period is set to 8 seconds  */
  HAL_RTC_CALIBRATION_PERIOD_16SEC = LL_RTC_CALIB_PERIOD_16SEC, /*!< Calibration cycle period is set to 16 seconds */
  HAL_RTC_CALIBRATION_PERIOD_32SEC = LL_RTC_CALIB_PERIOD_32SEC  /*!< Calibration cycle period is set to 32 seconds */
} hal_rtc_calibration_period_t;

/**
  * @brief Calibration increase frequency definitions.
  */
typedef enum
{
  HAL_RTC_CALIBRATION_PULSE_NOT_INSERTED = LL_RTC_CALIB_INSERTPULSE_NONE, /*!< No increase of the frequency */
  HAL_RTC_CALIBRATION_PULSE_INSERTED     = LL_RTC_CALIB_INSERTPULSE_SET   /*!< Increase of the frequency by one pulse every 2^11 pulses */
} hal_rtc_calibration_pulse_t;

/**
  * @brief Calibration seconds shifts definitions.
  */
typedef enum
{
  HAL_RTC_CALIBRATION_SHIFT_SECOND_DELAY   = LL_RTC_SHIFT_SECOND_DELAY,  /*!< Delay the calendar by one second */
  HAL_RTC_CALIBRATION_SHIFT_SECOND_ADVANCE = LL_RTC_SHIFT_SECOND_ADVANCE /*!< Advance the calendar by one second */
} hal_rtc_calibration_shift_second_t;

/**
  * @brief Calibration status definitions.
  */
typedef enum
{
  HAL_RTC_CALIBRATION_DISABLED = 0U, /*!< Timestamp disabled */
  HAL_RTC_CALIBRATION_ENABLED  = 1U  /*!< Timestamp enabled */
} hal_rtc_calibration_status_t;

/**
  * @}
  */


/*! HAL RTC Privilege attribute enumeration definition */
typedef enum
{
  HAL_RTC_NPRIV = LL_RTC_ATTR_NPRIV, /*!< RTC Non-privileged attribute */
  HAL_RTC_PRIV  = LL_RTC_ATTR_PRIV   /*!< RTC privileged attribute */
} hal_rtc_priv_attr_t;

/* Exported Unions ---------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Unions HAL RTC unions
  * @{
  */

/**
  * @brief RTC Alarm weekday and day union.
  */
typedef union
{
  uint32_t          mday; /*!< Day of the month */
  hal_rtc_weekday_t wday; /*!< Day of the week */
} hal_rtc_alarm_day_t;

/**
  * @}
  */

/* Exported Structure -----------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Structures HAL RTC exported structures
  * @{
  */

/**
  * @brief RTC configuration structure.
  */
typedef struct
{

  hal_rtc_mode_t mode;             /*!< RTC mode */

  uint32_t  asynch_prediv;         /*!< Asynchronous prescaler register value. Real prescaler = asynch_prediv + 1.
                                        This parameter must be a number between 0x00 and 0x7F */

  uint32_t  synch_prediv;          /*!< Synchronous prescaler register value. Real prescaler = synch_prediv + 1.
                                        This parameter must be a number between 0x00 and 0x7FFF.
                                        This parameter is used when the mode is HAL_RTC_MODE_BCD. */

  hal_rtc_bcd_update_t bcd_update; /*!< BCD update. This parameter is when the mode is HAL_RTC_MODE_BINARY or HAL_RTC_MODE_MIX */
} hal_rtc_config_t;

/**
  * @brief  Calendar configuration structure.
  */
typedef struct
{
  hal_rtc_calendar_hour_format_t       hour_format;            /*!< Hour format of the calendar */
  hal_rtc_calendar_shadow_reg_bypass_t bypass_shadow_register; /*!< Keep or bypass the shadow registers */
} hal_rtc_calendar_config_t;

/**
  * @brief Tampalarm polarity output structure.
  */
typedef struct
{
  hal_rtc_output_tampalarm_polarity_t polarity; /*!< Tampalarm output polarity */
  hal_rtc_output_tampalarm_type_t     type;     /*!< Tampalarm output type */
  hal_rtc_output_tampalarm_pullup_t   pullup;   /*!< Tampalarm output pull-up */
} hal_rtc_output_tampalarm_config_t;

/**
  * @brief Calibration output frequency structure.
  */
typedef struct
{
  hal_rtc_output_calibration_frequency_t frequency; /*!< Calibration frequency */
} hal_rtc_output_calib_config_t;

/**
  * @brief Time structure.
  */
typedef struct
{
  hal_rtc_time_format_am_pm_t am_pm;    /*!< Time is a.m. or p.m. */

  uint32_t                    subsec;   /*!< Subseconds register content that can have two functions:
                                             In BCD mode this parameter corresponds to a time unit range between[0-1]
                                             second with [1 sec/(SecondFraction+1)] granularity.
                                             This parameter corresponds to the free-running 32-bit counter in Binary and
                                             Mixed mode.
                                             This field is not used by the @ref HAL_RTC_CALENDAR_SetTime and @ref
                                             HAL_RTC_CALENDAR_SetDateTime functions.
                                             This parameter must be a number between 0x0 and 0x7FFF when configuring the
                                             alarm time in BCD or Mixed mode */

  uint32_t                    microsec; /*!< Time microseconds.
                                             This parameter must be a number between 0 and 999.
                                             It is only used when configuring the wake-up timer time */

  uint32_t                    millisec; /*!< Time milliseconds.
                                             This parameter must be a number between 0 and 999.
                                             It is only used when configuring the wake-up timer time */

  uint32_t                    hour;     /*!< Time hour. This parameter must be a number between:
                                             0 and 12 if the calendar hour format is 12 hours.
                                             0 and 24 if the calendar hour format is 24 hours.
                                             0 and 36 when using it with the wake-up timer */

  uint32_t                    min;      /*!< Time minutes. This parameter must be a number between 0 and 59 */

  uint32_t                    sec;      /*!< Time seconds */
} hal_rtc_time_t;

/**
  * @brief Date structure.
  */
typedef struct
{
  hal_rtc_weekday_t wday; /*!< Weekday */
  hal_rtc_month_t   mon;  /*!< Month */
  uint32_t          mday; /*!< Day. This parameter must be a number between 1 and 31 */
  uint32_t          year; /*!< Year. This parameter must be a number between 0 and 99.
                               Due to register limitations, the year is forced to 0 when reading the timestamp. */
} hal_rtc_date_t;

/**
  * @brief Alarm structure.
  */
typedef struct
{
  hal_rtc_alarm_subseconds_auto_reload_t subsec_auto_reload; /*!< Subsecond register reload. Enable is only allowed in binary mode */
  hal_rtc_alarm_auto_clear_t             auto_clear;         /*!< Alarm event automatic clear by hardware */
} hal_rtc_alarm_config_t;

/**
  * @brief Alarm time structure.
  */
typedef struct
{
  hal_rtc_time_t                     time;                /*!< Time of the alarm */

  uint32_t                           mask;                /*!< Alarm masks.
                                                               This parameter can be a combination of
                                                               @ref RTC_Exported_Constants_Alarm_Mask which includes
                                                               day, hours, minutes and seconds */

  uint32_t                           subsec_mask;         /*!< Alarm subseconds mask.
                                                               The most significant bits starting at this bit
                                                               are masked.
                                                               This parameter must be a number between 0 and 63.
                                                               From 32 to 63 all bits of the subseconds
                                                               register are compared to activate the alarm */

  hal_rtc_alarm_day_type_selection_t mday_wday_selection; /*!< Day mode of the alarm */

  hal_rtc_alarm_day_t                wday_mday;           /*!< Alarm day or day of week */
} hal_rtc_alarm_date_time_t;

/**
  * @brief Timestamp configuration structure.
  */
typedef struct
{
  hal_rtc_timestamp_source_pin_edge_t input_edge_polarity; /*!< Timestamp input edge polarity */
} hal_rtc_timestamp_config_t;

/**
  * @brief Timestamp information structure.
  */
typedef struct
{
  hal_rtc_timestamp_event_flag_t   flag;    /*!< Timestamp flag event   */
} hal_rtc_timestamp_information_t;


/**
  * @brief Wake-up structure.
  */
typedef struct
{
  hal_rtc_wakeup_timer_clock_t clock; /*!< Wake-up timer clock source */
} hal_rtc_wakeup_config_t;

/**
  * @}
  */

/**
  * @}
  */

/* Exported defines --------------------------------------------------------------------------------------------------*/

/** @defgroup RTC_Exported_Constants HAL RTC constants
  * @{
  */

/** @defgroup RTC_Exported_Constants_Alarm_Mask RTC alarm mask defines.
  * @{
  */

#define HAL_RTC_ALARM_MASK_NONE     LL_RTC_ALMA_MASK_NONE         /*!< All day and time values are considered to trigger the alarm */
#define HAL_RTC_ALARM_MASK_DAY      LL_RTC_ALMA_MASK_DATEWEEKDAY  /*!< The day/weekday value is ignored */
#define HAL_RTC_ALARM_MASK_HOURS    LL_RTC_ALMA_MASK_HOURS        /*!< The hour value is ignored */
#define HAL_RTC_ALARM_MASK_MINUTES  LL_RTC_ALMA_MASK_MINUTES      /*!< The minute value is ignored */
#define HAL_RTC_ALARM_MASK_SECONDS  LL_RTC_ALMA_MASK_SECONDS      /*!< The second value is ignored */
#define HAL_RTC_ALARM_MASK_ALL      LL_RTC_ALMA_MASK_ALL          /*!< All day and time values are ignored */

/**
  * @}
  */

/** @defgroup RTC_Exported_Constants_Wakeup_Timer_Interrupts RTC wake-up timer interrupt defines.
  * @{
  */

#define HAL_RTC_WAKEUP_IT_DISABLE LL_RTC_WAKEUP_TIMER_IT_DISABLE /*!< Wake-up interrupts are disabled */
#define HAL_RTC_WAKEUP_IT_ENABLE  LL_RTC_WAKEUP_TIMER_IT_ENABLE  /*!< Wake-up interrupts are enabled */

/**
  * @}
  */

/** @defgroup RTC_Exported_Constants_Alarm_Interrupts RTC alarm interrupt defines.
  * @{
  */

#define HAL_RTC_ALARM_IT_DISABLE LL_RTC_ALMA_IT_DISABLE /*!< Alarm interrupts are disabled */
#define HAL_RTC_ALARM_IT_ENABLE  LL_RTC_ALMA_IT_ENABLE  /*!< Alarm interrupts are enabled */

/**
  * @}
  */


/** @defgroup RTC_privilege_attributes_configuration_items RTC privilege attributes configuration items
  * @{
  */

#define HAL_RTC_PRIV_ITEM_ALRAPRIV LL_RTC_PRIV_ITEM_ALRAPRIV /*!< Privilege attribute of alarm A and underflow protection */
#define HAL_RTC_PRIV_ITEM_ALRBPRIV LL_RTC_PRIV_ITEM_ALRBPRIV /*!< Privilege attribute of alarm B protection */
#define HAL_RTC_PRIV_ITEM_WUTPRIV  LL_RTC_PRIV_ITEM_WUTPRIV  /*!< Privilege attribute of wake-up timer protection */
#define HAL_RTC_PRIV_ITEM_TSPRIV   LL_RTC_PRIV_ITEM_TSPRIV   /*!< Privilege attribute of timestamp protection */
#define HAL_RTC_PRIV_ITEM_CALPRIV  LL_RTC_PRIV_ITEM_CALPRIV  /*!< Privilege attribute of shift register, daylight saving, calibration and reference clock protection  */
#define HAL_RTC_PRIV_ITEM_INITPRIV LL_RTC_PRIV_ITEM_INITPRIV /*!< Privilege attribute of initialization protection */
#define HAL_RTC_PRIV_ITEM_PRIV     LL_RTC_PRIV_ITEM_PRIV     /*!< Privilege attribute of RTC global protection */
#define HAL_RTC_PRIV_ITEM_ALL      LL_RTC_PRIV_ITEM_ALL      /*!< Privilege attribute of all RTC resources */

/**
  * @}
  */


/**
  * @}
  */

/* Exported macros ---------------------------------------------------------------------------------------------------*/

/** @defgroup RTC_Exported_Macros HAL RTC macros
  * @{
  */

/**
  * @brief  Helper macro to convert a value from 2-digit decimal format to BCD format.
  * @param  value Byte to be converted
  * @return Converted byte
  */
#define HAL_RTC_CONVERT_DEC2BCD(value) LL_RTC_CONVERT_BIN2BCD(value)

/**
  * @brief  Helper macro to convert a value from BCD format to 2-digit decimal format.
  * @param  value BCD value to be converted
  * @return Converted byte
  */
#define HAL_RTC_CONVERT_BCD2DEC(value) LL_RTC_CONVERT_BCD2BIN(value)
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_Exported_Functions HAL RTC functions
  * @{
  */

/** @defgroup RTC_Exported_Functions_Write_Init RTC exported write protection and initialization mode functions.
  * @{
  */
hal_status_t HAL_RTC_EnableWriteProtection(void);
hal_status_t HAL_RTC_DisableWriteProtection(void);

hal_status_t HAL_RTC_EnterInitMode(void);
hal_status_t HAL_RTC_ExitInitMode(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Config RTC exported configuration functions
  * @{
  */
hal_status_t HAL_RTC_SetConfig(const hal_rtc_config_t *p_config);
void HAL_RTC_GetConfig(hal_rtc_config_t *p_config);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Low_Power RTC exported low power configuration functions
  * @{
  */
hal_status_t HAL_RTC_EnableUltraLowPowerMode(void);
hal_status_t HAL_RTC_DisableUltraLowPowerMode(void);
hal_rtc_ultra_low_power_mode_status_t HAL_RTC_IsEnabledUltraLowPowerMode(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Calendar RTC exported calendar functions
  *  @{
  */
hal_status_t HAL_RTC_CALENDAR_SetConfig(const hal_rtc_calendar_config_t *p_config_calendar);
void HAL_RTC_CALENDAR_GetConfig(hal_rtc_calendar_config_t *p_config_calendar);

hal_status_t HAL_RTC_CALENDAR_SetDateTime(const hal_rtc_date_t *p_date, const hal_rtc_time_t *p_time);
hal_status_t HAL_RTC_CALENDAR_GetDateTime(hal_rtc_date_t *p_date, hal_rtc_time_t *p_time);

hal_status_t HAL_RTC_CALENDAR_SetTime(const hal_rtc_time_t *p_time);
hal_status_t HAL_RTC_CALENDAR_GetTime(hal_rtc_time_t *p_time);

hal_status_t HAL_RTC_CALENDAR_SetDate(const hal_rtc_date_t *p_date);
hal_status_t HAL_RTC_CALENDAR_GetDate(hal_rtc_date_t *p_date);

hal_status_t HAL_RTC_CALENDAR_EnableReferenceClock(void);
hal_status_t HAL_RTC_CALENDAR_DisableReferenceClock(void);

hal_status_t HAL_RTC_CALENDAR_EnableSummerTimeMemorization(void);
hal_status_t HAL_RTC_CALENDAR_DisableSummerTimeMemorization(void);

hal_status_t HAL_RTC_CALENDAR_AddOneHour(void);
hal_status_t HAL_RTC_CALENDAR_SubtractOneHour(void);

uint32_t HAL_RTC_CALENDAR_GetBinaryTime(void);

hal_rtc_calendar_status_t HAL_RTC_CALENDAR_IsInitialized(void);
hal_rtc_calendar_reference_clock_status_t HAL_RTC_CALENDAR_IsEnabledReferenceClock(void);
hal_rtc_calendar_summer_time_status_t HAL_RTC_CALENDAR_IsEnabledSummerTimeMemorization(void);

hal_status_t HAL_RTC_CALENDAR_EnableITSubSecondsUnderflow(void);
hal_status_t HAL_RTC_CALENDAR_DisableITSubSecondsUnderflow(void);
hal_rtc_calendar_it_underflow_status_t HAL_RTC_CALENDAR_IsEnabledITSubSecondsUnderflow(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Output RTC exported output functions
  * @{
  */
hal_status_t HAL_RTC_OUTPUT_SetConfigTampalarm(const hal_rtc_output_tampalarm_config_t *p_config);
void HAL_RTC_OUTPUT_GetConfigTampalarm(hal_rtc_output_tampalarm_config_t *p_config);

hal_status_t HAL_RTC_OUTPUT_SetConfigCalib(const hal_rtc_output_calib_config_t *p_config);
void HAL_RTC_OUTPUT_GetConfigCalib(hal_rtc_output_calib_config_t *p_config);

hal_status_t HAL_RTC_OUTPUT_Enable(hal_rtc_output_t output);
hal_status_t HAL_RTC_OUTPUT_Disable(void);
hal_rtc_output_status_t HAL_RTC_OUTPUT_IsEnabled(hal_rtc_output_t output);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Calibration RTC exported calibration functions
  * @{
  */
hal_status_t HAL_RTC_EnableCalibration(hal_rtc_calibration_period_t calibration_period,
                                       hal_rtc_calibration_pulse_t pulse_add,
                                       uint32_t subtracted_pulses);
hal_status_t HAL_RTC_DisableCalibration(void);
hal_rtc_calibration_status_t HAL_RTC_IsEnabledCalibration(void);

hal_status_t HAL_RTC_ShiftCalibration(hal_rtc_calibration_shift_second_t add_one_sec,
                                      uint32_t fraction_sec_to_subtract);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Alarms RTC exported alarm functions
  * @{
  */
hal_status_t HAL_RTC_ALARM_SetConfig(hal_rtc_alarm_t alarm, const hal_rtc_alarm_config_t *p_config_alarm);
void HAL_RTC_ALARM_GetConfig(hal_rtc_alarm_t alarm, hal_rtc_alarm_config_t *p_config_alarm);

hal_status_t HAL_RTC_ALARM_SetDateTime(hal_rtc_alarm_t alarm, const hal_rtc_alarm_date_time_t *p_date_time);
void HAL_RTC_ALARM_GetDateTime(hal_rtc_alarm_t alarm, hal_rtc_alarm_date_time_t *p_date_time);

hal_status_t HAL_RTC_ALARM_Start(hal_rtc_alarm_t alarm, uint32_t interruption);
hal_status_t HAL_RTC_ALARM_Stop(hal_rtc_alarm_t alarm);
hal_status_t HAL_RTC_ALARM_PollForEvent(hal_rtc_alarm_t alarm, uint32_t timeout_ms);

hal_status_t HAL_RTC_ALARM_SetBinaryTime(hal_rtc_alarm_t alarm, uint32_t alarm_subsecond);
uint32_t HAL_RTC_ALARM_GetBinaryTime(hal_rtc_alarm_t alarm);
hal_status_t HAL_RTC_ALARM_SetBinarySubSecondMask(hal_rtc_alarm_t alarm, uint32_t alarm_subsecond_mask);
uint32_t HAL_RTC_ALARM_GetBinarySubSecondMask(hal_rtc_alarm_t alarm);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Timestamp RTC exported timestamp functions
  * @{
  */
hal_status_t HAL_RTC_TIMESTAMP_SetConfig(const hal_rtc_timestamp_config_t *p_config_timestamp);
void HAL_RTC_TIMESTAMP_GetConfig(hal_rtc_timestamp_config_t *p_config_timestamp);
hal_status_t HAL_RTC_TIMESTAMP_EnablePinSource(void);
hal_status_t HAL_RTC_TIMESTAMP_DisablePinSource(void);
hal_rtc_timestamp_status_t HAL_RTC_TIMESTAMP_IsEnabledPinSource(void);

hal_status_t HAL_RTC_TIMESTAMP_EnableTamperSource(void);
hal_status_t HAL_RTC_TIMESTAMP_DisableTamperSource(void);
hal_rtc_timestamp_tamper_status_t HAL_RTC_TIMESTAMP_IsEnabledTamperSource(void);

hal_status_t HAL_RTC_TIMESTAMP_EnableIT(void);
hal_status_t HAL_RTC_TIMESTAMP_DisableIT(void);
hal_rtc_timestamp_it_status_t HAL_RTC_TIMESTAMP_IsEnabledIT(void);

hal_status_t HAL_RTC_TIMESTAMP_GetDateTime(hal_rtc_time_t *p_time, hal_rtc_date_t *p_date,
                                           hal_rtc_timestamp_information_t *p_info);

hal_status_t HAL_RTC_TIMESTAMP_PollForEvent(uint32_t timeout_ms);

hal_status_t HAL_RTC_TIMESTAMP_GetBinaryTime(uint32_t *p_time_subseconds, hal_rtc_timestamp_information_t *p_info);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_WakeUp_Timer RTC exported wake-up timer functions
  * @{
  */
hal_status_t HAL_RTC_WAKEUP_SetConfig(const hal_rtc_wakeup_config_t *p_config_wakeup_timer);
void HAL_RTC_WAKEUP_GetConfig(hal_rtc_wakeup_config_t *p_config_wakeup_timer);

hal_status_t HAL_RTC_WAKEUP_SetPeriod(const hal_rtc_time_t *p_auto_reload_time,
                                      const hal_rtc_time_t *p_auto_clear_time);
void HAL_RTC_WAKEUP_GetPeriod(hal_rtc_time_t *p_auto_reload_time,
                              hal_rtc_time_t *p_auto_clear_time);

hal_status_t HAL_RTC_WAKEUP_Start(uint32_t interruption);
hal_status_t HAL_RTC_WAKEUP_Stop(void);
hal_status_t HAL_RTC_WAKEUP_PollForEvent(uint32_t timeout_ms);

hal_status_t HAL_RTC_WAKEUP_SetAutoReloadAndAutoClear(uint32_t wakeup_timer_auto_reload,
                                                      uint32_t wakeup_timer_auto_clear);
uint32_t HAL_RTC_WAKEUP_GetAutoReload(void);
uint32_t HAL_RTC_WAKEUP_GetAutoClear(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_IRQ RTC exported IRQ functions
  * @{
  */
void HAL_RTC_IRQHandler(void);
void HAL_RTC_ALARM_IRQHandler(void);
void HAL_RTC_TIMESTAMP_IRQHandler(void);
void HAL_RTC_WAKEUP_IRQHandler(void);
void HAL_RTC_SubSecondsUnderflow_IRQHandler(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Callback RTC exported callback functions
  * @{
  */
void HAL_RTC_AlarmAEventCallback(void);
void HAL_RTC_AlarmBEventCallback(void);
void HAL_RTC_TimestampEventCallback(void);
void HAL_RTC_WakeUpTimerEventCallback(void);
void HAL_RTC_SubSecondsUnderflowEventCallback(void);
/**
  * @}
  */

/** @defgroup RTC_Exported_Functions_Attributes RTC exported attribute management functions
  * @{
  */

hal_status_t HAL_RTC_SetPrivAttr(uint32_t item, hal_rtc_priv_attr_t priv_attr);
hal_rtc_priv_attr_t HAL_RTC_GetPrivAttr(uint32_t item);
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

#ifdef __cplusplus
}
#endif

#endif /* STM32C5xx_HAL_RTC_H */
