/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_rtc.c
  * @brief   RTC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Real-Time Clock (RTC) peripheral:
  *           * RTC configuration.
  *           * Calendar (Time and Date) configuration.
  *           * Alarms (alarm A and alarm B) configuration.
  *           * Wake-up timer configuration.
  *           * Timestamp configuration.
  *           * RTC output pin configuration.
  *           * Interrupts and flags management.
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
  *********************************************************************************************************************/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @addtogroup RTC
  * @{
  */

/** @defgroup RTC_Introduction RTC introduction
  * @{

  - The real-time clock (RTC) peripheral provides accurate timekeeping and calendar functionality.
    It operates independently of the main microcontroller, ensuring continuous time tracking even
    during low-power modes or system resets.
    The RTC peripheral supports a wide range of features, including programmable alarms, periodic
    wake-up events, and timestamping capabilities.

  * @}
  */

/** @defgroup RTC_How_To_Use RTC how to use
  * @{

  * The Real-Time Clock (RTC) is an independent BCD/Binary timer/counter.
  *
  * The RTC provides:
  *   - Time-of-day clock/calendar
  *   - Programmable alarms
  *   - Timestamp feature
  *   - Automatic wake-up timer
  *
  * The RTC clock, named RTCCLK, can come from different sources:
  *   - HSE
  *   - LSE
  *   - LSI
  *
  * By default, the RTC prescalers are configured to work with the LSE clock at 32.768 kHz.
  *
  * # RTC and microcontroller low power modes
  *
  * The MCU can be woken up from low power mode by RTC functions.
  *
  * The RTC functions are:
  *   - Alarm A event.
  *   - Alarm B event.
  *   - Timestamp event.
  *   - Wake-up timer event.
  *
  */

/**
  * @}
  */

/** @defgroup RTC_Configuration_Table RTC configuration table
  * @{

# Configuration inside the RTC driver

This configuration table is under development.

Configuration defines             | Description      | Default value   | Note                                         |
----------------------------------| ---------------- | --------------- | ---------------------------------------------|
PRODUCT                           | from IDE         |        NA       | The selected device (e.g. STM32ynxx)         |
USE_HAL_RTC_MODULE                | from hal_conf.h  |        1        | Allows use of the HAL RTC module             |
USE_ASSERT_DBG_PARAM              | from IDE         |      None       | Allows use of the assert check parameters    |
USE_HAL_CHECK_PARAM               | from hal_conf.h  |        0        | Allows use of the run-time check parameters  |
  */

/**
  * @}
  */

#if defined(USE_HAL_RTC_MODULE) && (USE_HAL_RTC_MODULE == 1)

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/* Private define ----------------------------------------------------------------------------------------------------*/

/** @defgroup RTC_Private_Constants RTC private constants
  * @{
  */
#define RTC_MICROSECONDS                        1000000U /*!< Number of microseconds in one second */
#define RTC_MILLISECONDS                        1000U    /*!< Number of milliseconds in one second */
#define RTC_MILLIMICROSECONDS                   1000U    /*!< Number of microseconds in one millisecond */
#define RTC_HOUR_SECONDS                        3600U    /*!< Number of seconds per hour */
#define RTC_MIN_SECONDS                         60U      /*!< Number of seconds per minute */
#define RTC_MAX_WAKEUP_VALUE_16BITS             0xFFFFU  /*!< Maximum value of the wake-up timer in 16 bits */
#define RTC_MIN_WAKEUP_VALUE_16BITS             0x0001U  /*!< Minimum value of the wake-up timer in 16 bits */
#define RTC_MAX_WAKEUP_VALUE_17BITS             0x1FFFFU /*!< Maximum value of the wake-up timer in 17 bits */
#define RTC_MIN_WAKEUP_VALUE_17BITS             0x10001U /*!< Minimum value of the wake-up timer in 17 bits */
#define RTC_TIMEOUT_VALUE                       512U     /*!< RTC timeout duration in ms */
#define RTC_WAKEUP_TIMER_CLOCK_SECONDS_ADD_1BIT 0x10000U /*!< RTC wake-up timer Clock seconds 1BIT */
/**
  * @}
  */

/* Private macro -----------------------------------------------------------------------------------------------------*/
/** @defgroup RTC_Private_Macros RTC private macros
  * @{
  */

/** @defgroup RTC_Private_Macros_Assert_Config RTC private macros for global enumerations.
  * @{
  */

/**
  * @brief Test RTC mode.
  */
#define IS_RTC_MODE(mode) (((mode) == HAL_RTC_MODE_BCD)       \
                           || ((mode) == HAL_RTC_MODE_BINARY) \
                           || ((mode) == HAL_RTC_MODE_MIX))

/**
  * @brief Test BCD seconds increment value if mode is not HAL_RTC_MODE_BCD.
  */
#define IS_RTC_BCD_UPDATE(increment, mode) (((mode) == HAL_RTC_MODE_BCD)                           \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_8BITS)  \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_9BITS)  \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_10BITS) \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_11BITS) \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_12BITS) \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_13BITS) \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_14BITS) \
                                            || ((increment) == HAL_RTC_BCD_UPDATE_15BITS))

/**
  * Test asynchronous prescaler value.
  */
#define IS_RTC_ASYNCH_PREDIV(prediv) ((prediv) <= 0x7FU)

/**
  * Test synchronous prescaler value if mode is HAL_RTC_MODE_BCD.
  */
#define IS_RTC_SYNCH_PREDIV(prediv, mode) (((mode) != HAL_RTC_MODE_BCD) || ((prediv) <= 0x7FFFU))

/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Calendar RTC private macros for calendar enumerations.
  * @{
  */

/**
  * @brief Test calendar hour format.
  */
#define IS_RTC_CALENDAR_HOUR_FORMAT(format) (((format) == HAL_RTC_CALENDAR_HOUR_FORMAT_24) \
                                             || ((format) == HAL_RTC_CALENDAR_HOUR_FORMAT_AMPM))

/**
  * @brief Test shadow registers bypass.
  */
#define IS_RTC_CALENDAR_SHADOW_REG(shadow) (((shadow) == HAL_RTC_CALENDAR_SHADOW_REG_KEEP) \
                                            || ((shadow) == HAL_RTC_CALENDAR_SHADOW_REG_BYPASS))
/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Date_Time RTC private macros for time and date enumerations.
  * @{
  */

/**
  * @brief Test time format.
  */
#define IS_RTC_TIME_FORMAT(format) (((format) == HAL_RTC_TIME_FORMAT_AM_24H) \
                                    || ((format) == HAL_RTC_TIME_FORMAT_PM))

/**
  * @brief Test date month.
  */
#define IS_RTC_MONTH(month) (((month) == HAL_RTC_MONTH_JANUARY)      \
                             || ((month) == HAL_RTC_MONTH_FEBRUARY)  \
                             || ((month) == HAL_RTC_MONTH_MARCH)     \
                             || ((month) == HAL_RTC_MONTH_APRIL)     \
                             || ((month) == HAL_RTC_MONTH_MAY)       \
                             || ((month) == HAL_RTC_MONTH_JUNE)      \
                             || ((month) == HAL_RTC_MONTH_JULY)      \
                             || ((month) == HAL_RTC_MONTH_AUGUST)    \
                             || ((month) == HAL_RTC_MONTH_SEPTEMBER) \
                             || ((month) == HAL_RTC_MONTH_OCTOBER)   \
                             || ((month) == HAL_RTC_MONTH_NOVEMBER)  \
                             || ((month) == HAL_RTC_MONTH_DECEMBER))

/**
  * @brief Test date week-day.
  */
#define IS_RTC_WEEKDAY(wday) (((wday) == HAL_RTC_WEEKDAY_MONDAY)       \
                              || ((wday) == HAL_RTC_WEEKDAY_TUESDAY)   \
                              || ((wday) == HAL_RTC_WEEKDAY_WEDNESDAY) \
                              || ((wday) == HAL_RTC_WEEKDAY_THURSDAY)  \
                              || ((wday) == HAL_RTC_WEEKDAY_FRIDAY)    \
                              || ((wday) == HAL_RTC_WEEKDAY_SATURDAY)  \
                              || ((wday) == HAL_RTC_WEEKDAY_SUNDAY))

/**
  * @brief Test date year.
  */
#define IS_RTC_YEAR(year) ((year) <= 99U)

/**
  * @brief Test date month-day.
  */
#define IS_RTC_MONTHDAY_NBR(mday) ((mday) <= 31U)

/**
  * @brief Test time hour depending on the time format.
  */
#define IS_RTC_HOUR(hour, format) (((((format) == HAL_RTC_CALENDAR_HOUR_FORMAT_24) ? \
                                     ((hour) <= 24U) : ((hour) <= 12U))))

/**
  * @brief Test time minutes.
  */
#define IS_RTC_MIN(min) ((min) <= 59U)

/**
  * @brief Test time seconds.
  */
#define IS_RTC_SEC(sec) ((sec) <= 59U)

/**
  * @brief Test time hours for wake-up timer.
  */
#define IS_RTC_HOUR_36(hour) ((hour) <= 36U)

/**
  * @brief Test microseconds.
  */
#define IS_RTC_MICROSEC(micro) ((micro) <= 999U)

/**
  * @brief Test milliseconds.
  */
#define IS_RTC_MILLISEC(micro) ((micro) <= 999U)
/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Output RTC private macros for output enumerations.
  * @{
  */

/**
  * @brief Test alarms or wake-up output.
  */
#define IS_RTC_OUTPUT_ALARM_WAKEUP(type) (((type) == HAL_RTC_OUTPUT_ALARM_WAKEUP_DISABLE)   \
                                          || ((type) == HAL_RTC_OUTPUT_ALARM_WAKEUP_ALARMA) \
                                          || ((type) == HAL_RTC_OUTPUT_ALARM_WAKEUP_ALARMB) \
                                          || ((type) == HAL_RTC_OUTPUT_ALARM_WAKEUP_WAKEUP))

/**
  * @brief Test tampalarm output polarity.
  */
#define IS_RTC_OUTPUT_TAMPALARM_POLARITY(polarity) (((polarity) == LL_RTC_OUTPUTPOLARITY_PIN_HIGH) \
                                                    || ((polarity) == LL_RTC_OUTPUTPOLARITY_PIN_LOW))

/**
  * @brief Test tampalarm output type.
  */
#define IS_RTC_OUTPUT_TAMPALARM_TYPE(type) (((type) == HAL_RTC_OUTPUT_TAMPALARM_TYPE_PUSHPULL) \
                                            || ((type) == HAL_RTC_OUTPUT_TAMPALARM_TYPE_OPENDRAIN))

/**
  * @brief Test tampalarm output pull-up.
  */
#define IS_RTC_OUTPUT_TAMPALARM_PULLUP(pullup) (((pullup) == HAL_RTC_OUTPUT_TAMPALARM_PULLUP_DISABLE) \
                                                || ((pullup) == HAL_RTC_OUTPUT_TAMPALARM_PULLUP_ENABLE))

/**
  * @brief Test calibration output frequency.
  */
#define IS_RTC_OUTPUT_CALIB_FREQ(freq) (((freq) == HAL_RTC_OUTPUT_CALIBRATION_SYNCHRONOUS) \
                                        || ((freq) == HAL_RTC_OUTPUT_CALIBRATION_ASYNCHRONOUS_DIV64))

/**
  * @brief Test output.
  */
#define IS_RTC_OUTPUT(output) (((output) == HAL_RTC_OUTPUT_OUT1_ALARMA)                    \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMB)                 \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_WAKEUP)                 \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_TAMP)                   \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_CALIB)                  \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_ALARMA)                 \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_ALARMB)                 \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_WAKEUP)                 \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_TAMP)                   \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_CALIB)                  \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMA_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMB_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_WAKEUP_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_ALARMA_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_ALARMB_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT2_WAKEUP_TAMP)            \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMA_OUT2_CALIB)      \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMB_OUT2_CALIB)      \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_WAKEUP_OUT2_CALIB)      \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_TAMP_OUT2_CALIB)        \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMA_TAMP_OUT2_CALIB) \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_ALARMB_TAMP_OUT2_CALIB) \
                               || ((output) == HAL_RTC_OUTPUT_OUT1_WAKEUP_TAMP_OUT2_CALIB))


/*! Privilege item attribute check macro */
#define IS_RTC_ITEM_PRIV_ATTR(attribute) ((attribute == HAL_RTC_PRIV) || (attribute == HAL_RTC_NPRIV))

/*! Set privilege item attribute check macro */
#define IS_RTC_PRIV_SET_ITEM(item) ((((item) & (HAL_RTC_PRIV_ITEM_ALL)) != 0x0U) \
                                    && (((item) & ~(HAL_RTC_PRIV_ITEM_ALL)) == 0x0U))

/*! Get privilege item attribute check macro */
#define IS_RTC_PRIV_GET_ITEM(item) (((item) == HAL_RTC_PRIV_ITEM_ALRAPRIV)    \
                                    || ((item) == HAL_RTC_PRIV_ITEM_ALRBPRIV) \
                                    || ((item) == HAL_RTC_PRIV_ITEM_WUTPRIV)  \
                                    || ((item) == HAL_RTC_PRIV_ITEM_TSPRIV)   \
                                    || ((item) == HAL_RTC_PRIV_ITEM_CALPRIV)  \
                                    || ((item) == HAL_RTC_PRIV_ITEM_INITPRIV) \
                                    || ((item) == HAL_RTC_PRIV_ITEM_PRIV))
/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Alarms RTC private macros for alarm enumerations.
  * @{
 */

/**
  * @brief Test alarm type.
  */
#define IS_RTC_ALARM_DAY_TYPE(alarm) (((alarm) == HAL_RTC_ALARM_DAY_TYPE_SEL_MONTHDAY) \
                                      || ((alarm) == HAL_RTC_ALARM_DAY_TYPE_SEL_WEEKDAY))

/**
  * @brief Test alarm.
  */
#define IS_RTC_ALARM(alarm) (((alarm) == HAL_RTC_ALARM_A) \
                             || ((alarm) == HAL_RTC_ALARM_B))

/**
  * @brief Test alarm auto-clear.
  */
#define IS_RTC_ALARM_AUTO_CLEAR(clear) (((clear) == HAL_ALARM_AUTO_CLEAR_DISABLE) \
                                        || ((clear) == HAL_ALARM_AUTO_CLEAR_ENABLE))

/**
  * @brief Test alarm subseconds auto-reload; enable is allowed only in binary mode.
  */
#define IS_RTC_ALARM_BINARY_AUTO_RELOAD(reload) (((reload) == HAL_RTC_ALARM_SUBSECONDS_AUTO_RELOAD_DISABLE) \
                                                 || (((reload) == HAL_RTC_ALARM_SUBSECONDS_AUTO_RELOAD_ENABLE) \
                                                     && (LL_RTC_GetBinaryMode() == LL_RTC_BINARY_ONLY)))

/*! Test alarm subseconds mask depending on the mode */
#define IS_RTC_ALARM_SUBSECONDS_MASK(mask) ((LL_RTC_GetBinaryMode() == LL_RTC_BINARY_NONE) \
                                            ? ((mask) <= 0xFU) \
                                            : ((mask) <= 0x3FU))

/*! Test alarm subseconds value depending on the mode */
#define IS_RTC_ALARM_SUBSECONDS(subsec) (((LL_RTC_GetBinaryMode() == LL_RTC_BINARY_NONE) \
                                          ? ((subsec) <= 0x7FFFU) \
                                          : 1U))

/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Timestamp RTC private macros for timestamp enumerations.
  * @{
  */

/**
  * @brief Test timestamp pin edge.
  */
#define IS_RTC_TIMESTAMP_SOURCE_PIN_EDGE(edge) (((edge) == HAL_RTC_TIMESTAMP_EDGE_RISING) \
                                                || ((edge) == HAL_RTC_TIMESTAMP_EDGE_FALLING))

/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Wakeup_Timer RTC private macros for wake-up timer enumerations.
  * @{
  */

/**
  * @brief Test wake-up timer clock source.
  */
#define IS_RTC_WAKEUP_TIMER_CLOCK(clock) (((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV2)             \
                                          || ((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV4)          \
                                          || ((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV8)          \
                                          || ((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_RTCCLK_DIV16)         \
                                          || ((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE)           \
                                          || ((clock) == HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT))

/*! Test wake-up timer value */
#define IS_RTC_WAKEUP_VALUE(value) ((value) <= 0xFFFFU)

/*! Test wake-up timer auto-clear value */
#define IS_RTC_WAKEUP_AUTOCLEAR_VALUE(value) ((value) <= 0xFFFFU)

/**
  * @brief Test wake-up timer auto-reload and auto-clear value comparison.
  */
#define IS_RTC_WAKEUP_AUTORELOAD_AUTOCLEAR(auto_reload, auto_clear) ((auto_clear) <= (auto_reload))
/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Calibration RTC private macros for calibration enumerations.
  * @{
  */

/**
  * @brief Test calibration period.
  */
#define IS_RTC_CALIBRATION_PERIOD(period) (((period) == HAL_RTC_CALIBRATION_PERIOD_8SEC)     \
                                           || ((period) == HAL_RTC_CALIBRATION_PERIOD_16SEC) \
                                           || ((period) == HAL_RTC_CALIBRATION_PERIOD_32SEC))

/**
  * @brief Test calibration pulse.
  */
#define IS_RTC_CALIBRATION_PULSE(pulse) (((pulse) == HAL_RTC_CALIBRATION_PULSE_NOT_INSERTED) \
                                         || ((pulse) == HAL_RTC_CALIBRATION_PULSE_INSERTED))

/*! Test calibration subtracted pulses */
#define IS_RTC_CALIBRATION_SUBTRACTED_PULSES(pulses) ((pulses)<= 0x1FFU)

/**
  * @brief Test calibration shift second.
  */
#define IS_RTC_CALIBRATION_SHIFT_SECOND(shift) (((shift) == HAL_RTC_CALIBRATION_SHIFT_SECOND_DELAY) \
                                                || ((shift) == HAL_RTC_CALIBRATION_SHIFT_SECOND_ADVANCE))

/*! Test calibration shift fractions of seconds */
#define IS_RTC_CALIBRATION_SHIFT_FRACTIONS(shifts) ((shifts) <= 0x7FFFU)

/**
  * @}
  */

/** @defgroup RTC_Private_Macros_Assert_Combination_Constants RTC private macros combination defines.
  * @{
  */

/**
  * @brief Test alarm mask combination.
  */
#define IS_RTC_ALARM_MASK(mask) (((mask) == HAL_RTC_ALARM_MASK_NONE)                                      \
                                 || (((mask) & HAL_RTC_ALARM_MASK_DAY) == HAL_RTC_ALARM_MASK_DAY)         \
                                 || (((mask) & HAL_RTC_ALARM_MASK_HOURS) == HAL_RTC_ALARM_MASK_HOURS)     \
                                 || (((mask) & HAL_RTC_ALARM_MASK_MINUTES) == HAL_RTC_ALARM_MASK_MINUTES) \
                                 || (((mask) & HAL_RTC_ALARM_MASK_SECONDS) == HAL_RTC_ALARM_MASK_SECONDS) \
                                 || (((mask) & HAL_RTC_ALARM_MASK_ALL) == HAL_RTC_ALARM_MASK_ALL))

/**
  * @brief Test wake-up timer interruption.
  */
#define IS_RTC_WAKEUP_IT(it) (((it) == HAL_RTC_WAKEUP_IT_DISABLE) \
                              || ((it) == HAL_RTC_WAKEUP_IT_ENABLE))

/**
  * @brief Test Alarm interruption.
  */
#define IS_RTC_ALARM_IT(it) (((it) == HAL_RTC_ALARM_IT_DISABLE) \
                             || ((it) == HAL_RTC_ALARM_IT_ENABLE))
/**
  * @}
  */

/**
  * @}
  */

/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------------------------------------------*/
/** @defgroup RTC_Private_Functions RTC private functions
  * @brief RTC private functions.
  * @{
  */

static hal_status_t          RTC_WaitSynchro_RS(void);
static hal_status_t          RTC_WaitSynchro_SHP(void);
static hal_status_t          RTC_WaitSynchro_RECALP(void);
static hal_status_t          RTC_WaitSynchro_WUTW(void);
__STATIC_INLINE hal_status_t RTC_GetTime(hal_rtc_time_t *p_time);
__STATIC_INLINE hal_status_t RTC_GetDate(hal_rtc_date_t *p_date);
__STATIC_INLINE uint32_t     RTC_GetWakeUpClockFrequency(hal_rtc_wakeup_timer_clock_t clock_prescaler_wakeup);
__STATIC_INLINE uint32_t     RTC_GetRTCClockCalibrated(void);
__STATIC_INLINE uint32_t     RTC_GetRTCClockAfterPrescalerS(uint32_t frequency);
__STATIC_INLINE uint32_t     RTC_ConvertSecSubsecToBits(uint32_t seconds, uint32_t milliseconds,
                                                        uint32_t microseconds, uint32_t frequency);
__STATIC_INLINE void         RTC_ConvertBitsToTime(hal_rtc_time_t *p_time, uint32_t bits, uint32_t frequency);
/**
  * @}
  */
/* Functions Definition ----------------------------------------------------------------------------------------------*/

/** @addtogroup RTC_Exported_Functions
  * @{
  */

/** @addtogroup RTC_Exported_Functions_Write_Init
  * @brief Write protection and initialization mode.
  * @{
  *
  * # RTC write protection.
  *
  * The RTC peripheral has its own write protection.
  * After backup domain reset, some of the RTC registers are protected against parasitic write access.
  *
  * Calling @ref HAL_RTC_DisableWriteProtection and @ref HAL_RTC_EnableWriteProtection
  * disables and enables the write protection respectively.
  *
  * # Initialization mode
  *
  * The initialization mode of the RTC permits to initialize and configure the RTC calendar.
  * Entering this mode stops the calendar counter.
  *
  * Calling @ref HAL_RTC_EnterInitMode and @ref HAL_RTC_ExitInitMode makes the RTC enter and
  * exit initialization mode, respectively.
  */

/**
  * @brief  Enable the RTC registers write protection.
  * @note   Not all RTC registers are concerned by the write protection.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_EnableWriteProtection(void)
{
  LL_RTC_EnableWriteProtection();

  return HAL_OK;
}

/**
  * @brief  Disable the RTC registers write protection.
  * @note   Not all RTC registers are concerned by the write protection.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_DisableWriteProtection(void)
{
  LL_RTC_DisableWriteProtection();

  return HAL_OK;
}

/**
  * @brief   Enter the RTC initialization mode.
  * @warning The RTC initialization mode is write protected.
  *          Use the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  * @retval  HAL_ERROR RTC does not enter initialization mode before the RTC timeout duration.
  */
hal_status_t HAL_RTC_EnterInitMode(void)
{
  uint32_t tickstart;

  if (LL_RTC_IsActiveFlag_INIT() == 0U)
  {
    LL_RTC_EnableInitMode();
    tickstart = HAL_GetTick();
    while ((LL_RTC_IsActiveFlag_INIT() == 0U))
    {
      if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief   Exit the RTC initialization mode.
  * @warning The RTC initialization mode is write protected.
  *          Use the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  * @retval  HAL_ERROR Calendar resynchronization is still pending after timeout duration.
  */
hal_status_t HAL_RTC_ExitInitMode(void)
{
  LL_RTC_DisableInitMode();

  if (RTC_WaitSynchro_RS() != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Config
  * @brief  Configuration functions for prescaler values and RTC mode.
  * @{
  *
  * # RTC clock and prescalers
  *
  * The RTC clock, named RTCCLK, is configured by the RCC driver using HAL_RCC_RTC_SetKernelClkSource.
  * The RTCCLK source can be LSE, LSI, or HSE.
  * The RTC peripheral has a programmable prescaler stage that can be configured to generate a 1 Hz clock.
  * To minimize consumption, the prescaler is split into two programmable prescalers:
  * * A 7-bit asynchronous prescaler (PREDIV_A).
  * * A 15-bit synchronous prescaler (PREDIV_S).
  *
  * When using both prescalers, it is recommended to configure the asynchronous
  * prescaler to a high value to minimize consumption.
  * By default, the asynchronous and synchronous prescalers are set to 128 and 256
  * to obtain a clock frequency of 1 Hz with an RTCCLK configured to an LSE of 32.768 kHz.
  *
  * The following equation gives the clock frequency after the first prescaler:
  * freq_async_clk = freq_rtcclk/(1+@ref hal_rtc_config_t::asynch_prediv)
  *
  * The following equation gives the clock frequency after the two prescalers:
  * freq_sync_clk = freq_rtcclk/((1+ @ref hal_rtc_config_t::asynch_prediv)(1+ @ref hal_rtc_config_t::synch_prediv))
  *
  * The values of the prescalers can be changed by calling @ref HAL_RTC_SetConfig and
  * setting the value of @ref hal_rtc_config_t::asynch_prediv and @ref hal_rtc_config_t::synch_prediv.
  * To change the values of the prescalers, the RTC needs to be in
  * initialization mode by calling @ref HAL_RTC_EnterInitMode.
  *
  * # RTC running modes.
  *
  * RTC supports three operating modes:
  * * BCD mode for Binary-Coded Decimal
  * * Binary mode
  * * Mixed mode
  *
  * The mode is chosen by selecting the value of @ref hal_rtc_config_t::mode and calling @ref HAL_RTC_SetConfig.
  *
  * ## Binary-Coded Decimal mode
  *
  * The BCD mode is a feature that allows the RTC to store and display time and date information in a BCD format.
  * BCD is a way of representing decimal numbers using four bits per digit, with each digit ranging from 0 to 9.
  * In BCD mode, the subseconds down-counter is clocked by the signal after the first prescaler.
  * When the down-counter reaches 0, it is reloaded with the value of the synchronous prescaler.
  * Only the first 16 bits of the subsecond register are used in BCD mode.
  *
  * ## Binary mode
  *
  * In binary mode, the subseconds down-counter is clocked by the signal after the first prescaler.
  * The subsecond register is extended to 32-bit length and is free running.
  * The value of the synchronous register is not used, and the time and date registers are not used.
  * In this mode, users can use all functions that include the keyword Binary in their names that are optimized for it.
  *
  * ## Mixed mode
  *
  * This mode, as stated by its name, is a mix between the two previous modes.
  * The subseconds down-counter is extended to 32-bit length and is free running.
  * The time and date calendar in BCD are also functional.
  * The down-counter is still clocked by the signal after the asynchronous prescaler,
  * but the seconds of the calendar are incremented each time the least significant
  * bits of the subsecond register reach 0. The number of bits taken into account
  * are defined by configuring @ref HAL_RTC_SetConfig and changing the value of @ref hal_rtc_config_t::bcd_update.
  *
  * To change the RTC mode, the RTC needs to be in initialization mode by calling @ref HAL_RTC_EnterInitMode.
  */

/**
  * @brief   Program the RTC prescaler values and the RTC mode according to the specified parameters.
  * @param   p_config pointer to an RTC configuration instance.
  * @warning The RTC configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this
  *          function.
  * @warning The RTC configuration must be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK.
  * @retval  HAL_INVALID_PARAM p_config is NULL.
  */
hal_status_t HAL_RTC_SetConfig(const hal_rtc_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_RTC_ASYNCH_PREDIV(p_config->asynch_prediv));
  ASSERT_DBG_PARAM(IS_RTC_MODE(p_config->mode));
  ASSERT_DBG_PARAM(IS_RTC_SYNCH_PREDIV(p_config->synch_prediv, p_config->mode));
  ASSERT_DBG_PARAM(IS_RTC_BCD_UPDATE(p_config->bcd_update, p_config->mode));
#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)

  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (p_config->mode == HAL_RTC_MODE_BCD)
  {
    LL_RTC_SetPrescalers(p_config->asynch_prediv, p_config->synch_prediv);
    LL_RTC_SetBinaryMode(LL_RTC_BINARY_NONE);
  }
  else
  {
    LL_RTC_SetAsynchPrescaler(p_config->asynch_prediv);
    LL_RTC_SetConfigBinaryMode((uint32_t) p_config->mode, (uint32_t) p_config->bcd_update);
  }

  return HAL_OK;
}

/**
  * @brief Retrieve the RTC prescaler values and the RTC mode.
  * @param p_config pointer to an RTC configuration instance.
  */
void HAL_RTC_GetConfig(hal_rtc_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);

  uint32_t prescalers_values = LL_RTC_GetPrescalers();
  uint32_t binary_mode_bcdu = LL_RTC_GetConfigBinaryMode();
  uint32_t binary_mode = LL_RTC_GET_BIN(binary_mode_bcdu);
  uint32_t bcd_update = LL_RTC_GET_BCDU(binary_mode_bcdu);
  p_config->asynch_prediv = LL_RTC_GET_ASYNCH_PRESCALER(prescalers_values);
  p_config->synch_prediv = LL_RTC_GET_SYNCH_PRESCALER(prescalers_values);
  p_config->mode = (hal_rtc_mode_t) binary_mode;
  p_config->bcd_update = (hal_rtc_bcd_update_t) bcd_update;
}

/**
  * @}
 */

/** @addtogroup RTC_Exported_Functions_Low_Power
  * @brief   Low power configuration functions.
  * @{
  *
  * It is possible to drastically reduce the RTC power consumption by setting the RTC to
  * low power mode. In this configuration the whole RTC is clocked by the ck_apre.
  *
  * When the division factor of the asynchronous prescaler is not
  * a power of 2 (( @ref hal_rtc_config_t::asynch_prediv +1) % 2 == 0),
  * the low power mode cannot be activated.
  *
  */

/**
  * @brief   Enable the RTC ultra low power mode.
  * @note    When (@ref hal_rtc_config_t::asynch_prediv + 1) is not a power of 2, calling this function does not
  *          enable low power mode.
  * @warning The RTC low power configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this
  *          function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_EnableUltraLowPowerMode(void)
{
  LL_RTC_CAL_LowPower_Enable();

  return HAL_OK;
}

/**
  * @brief   Disable the RTC ultra low power mode.
  * @warning The RTC configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this
  *          function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_DisableUltraLowPowerMode(void)
{
  LL_RTC_CAL_LowPower_Disable();

  return HAL_OK;
}

/**
  * @brief  Check whether the RTC ultra low power mode is enabled.
  * @retval hal_rtc_ultra_low_power_mode_status_t Ultra
  *         low power mode status.
  */
hal_rtc_ultra_low_power_mode_status_t HAL_RTC_IsEnabledUltraLowPowerMode(void)
{
  return (hal_rtc_ultra_low_power_mode_status_t) LL_RTC_CAL_LowPower_IsEnabled();
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Calendar
  * @brief   Exported calendar functions.
  * @{
  *
  * The RTC calendar stores and displays time and date information in a BCD format.
  * It includes subseconds, seconds, minutes, hours, weekday, date, month, and year.
  * The calendar works only in BCD or mixed mode; otherwise, only the subseconds down-counter register works.
  *
  * # Calendar configuration
  *
  * ## Calendar hour format and shadow registers.
  *
  * Set the hour format to 24-hour or 12-hour format by programming
  * @ref hal_rtc_calendar_config_t::hour_format and calling @ref HAL_RTC_CALENDAR_SetConfig.
  *
  * ## Shadow registers
  *
  * The shadow registers mirror the values of the calendar date and time registers.
  * When the shadow registers are not bypassed, the application reads the values of
  * date and time in the shadow registers. Reading the time locks the values in
  * calendar shadow registers to ensure consistency between time and date values;
  * reading the date unlocks them. Call @ref HAL_RTC_CALENDAR_GetTime before
  * @ref HAL_RTC_CALENDAR_GetDate, or call @ref HAL_RTC_CALENDAR_GetDateTime to retrieve the time and date correctly.
  * The application can access the date and time registers directly by bypassing
  * the shadow registers. Use @ref hal_rtc_calendar_config_t::bypass_shadow_register
  * to configure the shadow register bypass.
  *
  * # Other calendar services
  *
  * ## Reference clock detection
  *
  * In BCD mode only, the update of the RTC calendar can be synchronized to a reference clock.
  * This reference clock is used to compensate for the imprecision of the calendar seconds update frequency.
  * This detection is only enabled if the prescalers are set to their default values,
  * which are 128 and 256 for @ref hal_rtc_config_t::asynch_prediv and @ref hal_rtc_config_t::synch_prediv respectively.
  * ## Binary mode
  *
  * A dedicated API is provided to optimize the code when using binary mode.
  * The function @ref HAL_RTC_CALENDAR_GetBinaryTime retrieves the value of the subseconds down-counter register.
  * ## Summer time and hour increment and decrement
  *
  * The functions @ref HAL_RTC_CALENDAR_EnableSummerTimeMemorization and
  * @ref HAL_RTC_CALENDAR_DisableSummerTimeMemorization allow an application to
  * keep information about the summer time status. These functions do not alter the hour registers.
  * Increment or decrement the hours by using @ref HAL_RTC_CALENDAR_AddOneHour and @ref HAL_RTC_CALENDAR_SubtractOneHour
  * respectively, which are independent of the summer time memorization bit.
  * ## Subseconds underflow
  *
  * In binary or mixed mode, enable the subsecond underflow interrupt.
  * This interrupt triggers when the subseconds down-counter underflows.
  */

/**
  * @brief   Program the bypass shadow registers and calendar hour format according to the specified parameters.
  * @param   p_config_calendar pointer to an RTC calendar configuration instance.
  * @warning The RTC calendar configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC calendar configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_config_calendar is NULL.
  */
hal_status_t HAL_RTC_CALENDAR_SetConfig(const hal_rtc_calendar_config_t *p_config_calendar)
{
  ASSERT_DBG_PARAM(p_config_calendar != NULL);
  ASSERT_DBG_PARAM(IS_RTC_CALENDAR_HOUR_FORMAT(p_config_calendar->hour_format));
  ASSERT_DBG_PARAM(IS_RTC_CALENDAR_SHADOW_REG(p_config_calendar->bypass_shadow_register));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config_calendar == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_SetHourFormatAndShadowRegBypass((uint32_t) p_config_calendar->hour_format,
                                         (uint32_t) p_config_calendar->bypass_shadow_register);

  return HAL_OK;
}

/**
  * @brief Retrieve the bypass shadow registers and calendar hour format configuration.
  * @param p_config_calendar pointer to an RTC calendar configuration instance.
  */
void HAL_RTC_CALENDAR_GetConfig(hal_rtc_calendar_config_t *p_config_calendar)
{
  ASSERT_DBG_PARAM(p_config_calendar != NULL);

  uint32_t value_format_shadow_reg = LL_RTC_READ_REG(CR);
  uint32_t bypass_shadow_register = LL_RTC_GET_SHADOW_REG_BYPASS(value_format_shadow_reg);
  uint32_t hour_format = LL_RTC_GET_CALENDAR_HOUR_FORMAT(value_format_shadow_reg);

  p_config_calendar->bypass_shadow_register = (hal_rtc_calendar_shadow_reg_bypass_t)
                                              bypass_shadow_register;
  p_config_calendar->hour_format = (hal_rtc_calendar_hour_format_t) hour_format;

}

/**
  * @brief   Program the RTC time.
  * @param   p_time pointer to an RTC time instance.
  * @note    @ref hal_rtc_time_t::microsec field is ignored because it is not used in the SetTime function.
  * @note    @ref hal_rtc_time_t::millisec field is ignored because it is not used in the SetTime function.
  * @note    @ref hal_rtc_time_t::subsec field is ignored because it is not used in the SetTime function.
  * @warning The RTC time configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC time configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_time is NULL.
  */
hal_status_t HAL_RTC_CALENDAR_SetTime(const hal_rtc_time_t *p_time)
{
  ASSERT_DBG_PARAM(p_time != NULL);
  ASSERT_DBG_PARAM(IS_RTC_TIME_FORMAT(p_time->am_pm));
  ASSERT_DBG_PARAM(IS_RTC_HOUR(p_time->hour, (hal_rtc_calendar_hour_format_t) LL_RTC_GetHourFormat()));
  ASSERT_DBG_PARAM(IS_RTC_MIN(p_time->min));
  ASSERT_DBG_PARAM(IS_RTC_SEC(p_time->sec));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_time == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_TIME_Config((uint32_t) p_time->am_pm, HAL_RTC_CONVERT_DEC2BCD(p_time->hour),
                     HAL_RTC_CONVERT_DEC2BCD(p_time->min), HAL_RTC_CONVERT_DEC2BCD(p_time->sec));

  return HAL_OK;
}

/**
  * @brief  Retrieve the RTC time.
  * @param  p_time pointer to an RTC time instance.
  * @note   When using the shadow registers (default):
  *         Reading RTC current time locks the values in calendar shadow
  *         registers to ensure consistency between time and date values.
  *         Call @ref HAL_RTC_CALENDAR_GetDate to unlock the time value.
  * @retval HAL_OK
  * @retval HAL_ERROR When a timeout occurs on the shadow register flag
  */
hal_status_t HAL_RTC_CALENDAR_GetTime(hal_rtc_time_t *p_time)
{
  ASSERT_DBG_PARAM(p_time != NULL);
  return RTC_GetTime(p_time);
}

/**
  * @brief   Program the RTC date.
  * @param   p_date pointer to an RTC date instance.
  * @warning The RTC date configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC date configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_date is NULL.
  */
hal_status_t HAL_RTC_CALENDAR_SetDate(const hal_rtc_date_t *p_date)
{
  ASSERT_DBG_PARAM(p_date != NULL);
  ASSERT_DBG_PARAM(IS_RTC_YEAR(p_date->year));
  ASSERT_DBG_PARAM(IS_RTC_MONTH(p_date->mon));
  ASSERT_DBG_PARAM(IS_RTC_WEEKDAY(p_date->wday));
  ASSERT_DBG_PARAM(IS_RTC_MONTHDAY_NBR(p_date->mday));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_date == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* No need to use HAL_RTC_CONVERT_DEC2BCD for wday because the max value is less than 10; BCD equals decimal */
  LL_RTC_DATE_Config((uint32_t)p_date->wday, HAL_RTC_CONVERT_DEC2BCD(p_date->mday),
                     (uint32_t)p_date->mon, HAL_RTC_CONVERT_DEC2BCD(p_date->year));

  return HAL_OK;
}

/**
  * @brief  Retrieve the RTC date.
  * @param  p_date pointer to an RTC date instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When a timeout occurs on the shadow register flag
  */
hal_status_t HAL_RTC_CALENDAR_GetDate(hal_rtc_date_t *p_date)
{
  ASSERT_DBG_PARAM(p_date != NULL);

  return RTC_GetDate(p_date);
}

/**
  * @brief   Program the RTC date and time.
  * @param   p_date pointer to an RTC date instance.
  * @param   p_time pointer to an RTC time instance.
  * @note    @ref hal_rtc_time_t::microsec field is ignored because it is not used in the SetDateTime function.
  * @note    @ref hal_rtc_time_t::millisec field is ignored because it is not used in the SetDateTime function.
  * @note    @ref hal_rtc_time_t::subsec field is ignored because it is not used in the SetDateTime function.
  * @warning The RTC date and time configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC date and time configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_date or p_time are NULL.
  */
hal_status_t HAL_RTC_CALENDAR_SetDateTime(const hal_rtc_date_t *p_date, const hal_rtc_time_t *p_time)
{
  ASSERT_DBG_PARAM(p_date != NULL);
  ASSERT_DBG_PARAM(p_time != NULL);
  ASSERT_DBG_PARAM(IS_RTC_YEAR(p_date->year));
  ASSERT_DBG_PARAM(IS_RTC_MONTH(p_date->mon));
  ASSERT_DBG_PARAM(IS_RTC_WEEKDAY(p_date->wday));
  ASSERT_DBG_PARAM(IS_RTC_MONTHDAY_NBR(p_date->mday));
  ASSERT_DBG_PARAM(IS_RTC_TIME_FORMAT(p_time->am_pm));
  ASSERT_DBG_PARAM(IS_RTC_HOUR(p_time->hour, (hal_rtc_calendar_hour_format_t) LL_RTC_GetHourFormat()));
  ASSERT_DBG_PARAM(IS_RTC_MIN(p_time->min));
  ASSERT_DBG_PARAM(IS_RTC_SEC(p_time->sec));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_date == NULL) || (p_time == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_TIME_Config((uint32_t) p_time->am_pm, HAL_RTC_CONVERT_DEC2BCD(p_time->hour),
                     HAL_RTC_CONVERT_DEC2BCD(p_time->min), HAL_RTC_CONVERT_DEC2BCD(p_time->sec));

  /* No need to use HAL_RTC_CONVERT_DEC2BCD for wday because the max value is less than 10; BCD equals decimal */
  LL_RTC_DATE_Config((uint32_t)p_date->wday, HAL_RTC_CONVERT_DEC2BCD(p_date->mday),
                     (uint32_t)p_date->mon, HAL_RTC_CONVERT_DEC2BCD(p_date->year));

  return HAL_OK;
}

/**
  * @brief  Retrieve the RTC date and time.
  * @param  p_date pointer to an RTC date instance.
  * @param  p_time pointer to an RTC time instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When a timeout occurs on the shadow register flag
  */
hal_status_t HAL_RTC_CALENDAR_GetDateTime(hal_rtc_date_t *p_date, hal_rtc_time_t *p_time)
{
  ASSERT_DBG_PARAM(p_date != NULL);
  ASSERT_DBG_PARAM(p_time != NULL);

  hal_status_t status_time;
  hal_status_t status_date;

  /* When using the shadow registers always get time first and then date.
     Even if there is a timeout in time read the date in any case to unlock the shadow registers. */
  status_time = RTC_GetTime(p_time);
  status_date = RTC_GetDate(p_date);

  if (status_time != HAL_OK)
  {
    return status_time;
  }

  return status_date;
}

/**
  * @brief   Enable the reference clock detection.
  * @note    The detection only works when RTC is set to BCD mode only
  *          @ref HAL_RTC_MODE_BCD, the synchronous prescaler
  *          @ref hal_rtc_config_t::synch_prediv and the asynchronous prescaler @ref hal_rtc_config_t::asynch_prediv
  *          are set to 0x00FF (default value) and to 0x007F (default-value) respectively.
  * @warning The RTC reference clock configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC reference clock configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_EnableReferenceClock(void)
{
  LL_RTC_EnableRefClock();

  return HAL_OK;
}

/**
  * @brief   Disable the reference clock detection.
  * @warning The RTC reference clock configuration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning The RTC reference clock configuration needs to be called when RTC is in initialization mode,
  *          use @ref HAL_RTC_EnterInitMode to enter initialization mode.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_DisableReferenceClock(void)
{
  LL_RTC_DisableRefClock();

  return HAL_OK;
}

/**
  * @brief  Check if the reference clock detection is enabled or disabled.
  * @retval hal_rtc_calendar_reference_clock_status_t Reference clock detection status
  */
hal_rtc_calendar_reference_clock_status_t HAL_RTC_CALENDAR_IsEnabledReferenceClock(void)
{
  return (hal_rtc_calendar_reference_clock_status_t) LL_RTC_IsEnabledRefClock();
}

/**
  * @brief   Add one hour to the calendar in a single operation without going
  *          through the initialization procedure.
  * @warning The RTC hour increment is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_AddOneHour(void)
{
  LL_RTC_TIME_IncHour();

  return HAL_OK;
}

/**
  * @brief   Subtract one hour from the calendar in a single operation without
  *          through the initialization procedure.
  * @warning The RTC hour decrement is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_SubtractOneHour(void)
{
  LL_RTC_TIME_DecHour();

  return HAL_OK;
}

/**
  * @brief   Enable the summer time memorization bit status.
  * @warning The RTC summer time memorization bit is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_EnableSummerTimeMemorization(void)
{
  LL_RTC_TIME_EnableDayLightStore();

  return HAL_OK;
}

/**
  * @brief   Disable the summer time memorization bit status.
  * @warning The RTC summer time memorization bit is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_DisableSummerTimeMemorization(void)
{
  LL_RTC_TIME_DisableDayLightStore();

  return HAL_OK;
}

/**
  * @brief  Check if the summer time memorization bit is enabled or disabled.
  * @retval hal_rtc_calendar_summer_time_status_t Summer time memorization bit status.
  */
hal_rtc_calendar_summer_time_status_t HAL_RTC_CALENDAR_IsEnabledSummerTimeMemorization(void)
{
  return (hal_rtc_calendar_summer_time_status_t)LL_RTC_TIME_IsEnabledDayLightStore();
}

/**
  * @brief  Retrieve the subsecond register value.
  * @retval uint32_t Value of the subsecond register.
  */
uint32_t HAL_RTC_CALENDAR_GetBinaryTime(void)
{
  return LL_RTC_TIME_GetSubSecond();
}

/**
  * @brief  Enable the subsecond register underflow interruption.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_EnableITSubSecondsUnderflow(void)
{
  LL_RTC_EnableIT_SSRU();

  return HAL_OK;
}

/**
  * @brief  Disable the subsecond register underflow interruption.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_CALENDAR_DisableITSubSecondsUnderflow(void)
{
  LL_RTC_DisableIT_SSRU();

  return HAL_OK;
}

/**
  * @brief  Check if the subsecond register underflow interruption is enabled or disabled.
  * @retval hal_rtc_calendar_it_underflow_status_t Subsecond register underflow interruption status
  */
hal_rtc_calendar_it_underflow_status_t HAL_RTC_CALENDAR_IsEnabledITSubSecondsUnderflow(void)
{
  return (hal_rtc_calendar_it_underflow_status_t) LL_RTC_IsEnabledIT_SSRU();
}

/**
  * @brief  Check if the calendar is initialized.
  * @retval hal_rtc_calendar_status_t Calendar initialization status.
  */
hal_rtc_calendar_status_t HAL_RTC_CALENDAR_IsInitialized(void)
{
  return (hal_rtc_calendar_status_t) LL_RTC_IsActiveFlag_INITS();
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Output
  * @brief Exported output functions.
  * @{
  *
  * To enable the output signals, call @ref HAL_RTC_OUTPUT_Enable. To use RTC_OUT1 as input,
  * all outputs must be disabled on RTC_OUT1. Disable all outputs by calling @ref HAL_RTC_OUTPUT_Disable,
  * or configure the outputs on RTC_OUT2 by calling @ref HAL_RTC_OUTPUT_Enable with @ref hal_rtc_output_t values that
  * contain only OUT2 in their names.
  *
  * # Configuration of the TAMPALRM output signal
  *
  * The TAMPALRM output is the OR between rtc_tamp_evt and ALARM output.
  *
  * ALARM output can be selected between alarm A, alarm B, or wake-up outputs.
  *
  * Depending on the application needs, configure the polarity, the output type, and the use of a pull-up
  * by using @ref HAL_RTC_OUTPUT_SetConfigTampalarm.
  * When configuring the output to open-drain, set the GPIO as input by using
  * HAL_GPIO_Init from the GPIO driver.
  *
  * # Configuration of the calibration output signal
  *
  * The calibration output signal corresponds to the RTCCLK clock signal after the
  * 6th stage of the asynchronous prescaler or the 8th stage of the synchronous
  * prescaler (and all the asynchronous prescaler).
  *
  * To choose between these two configurations, call @ref HAL_RTC_OUTPUT_SetConfigCalib.
  *
  * ## Asynchronous clock calibration
  *
  * The calibration clock is the output of the 6th stage of the asynchronous prescaler.
  * Thus, only the first 6 bits are used to define the calibration frequency.
  * The frequency of the output is (freq_RTCCLK / (PREDIV_A[5:0]+1)),
  * PREDIV_A given by @ref hal_rtc_config_t::asynch_prediv.
  *
  * If low power mode is disabled and the value of the asynchronous prescaler is
  * strictly below 0x20, the calibration signal output does not work.
  *
  * With the prescaler at default values and using LSE, the clock frequency is 512 Hz.
  *
  * ## Synchronous clock calibration
  *
  * The calibration output is the output of the 8th stage of the synchronous prescaler.
  * Thus, only the first 8 bits are used to define the calibration frequency.
  * The frequency of the output is (freq_RTCCLK/((PREDIV_A+1) * (PREDIV_S[7:0] +1)).
  * PREDIV_S and PREDIV_A given by @ref hal_rtc_config_t::synch_prediv and @ref hal_rtc_config_t::asynch_prediv
  * respectively.
  *
  * With the prescaler at default values and using LSE, the clock frequency is 1 Hz.
  *
  */

/**
  * @brief   Program the tampalarm configuration.
  * @param   p_config pointer to the tampalarm configuration instance.
  * @warning When configuring tampalarm in open-drain mode
  *          @ref HAL_RTC_OUTPUT_TAMPALARM_TYPE_OPENDRAIN
  *          The RTC_OUT1 GPIO must be set to input in GPIO driver.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_config is NULL.
  */
hal_status_t HAL_RTC_OUTPUT_SetConfigTampalarm(const hal_rtc_output_tampalarm_config_t  *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT_TAMPALARM_POLARITY(p_config->polarity));
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT_TAMPALARM_TYPE(p_config->type));
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT_TAMPALARM_PULLUP(p_config->pullup));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_ConfigTampalarm((uint32_t) p_config->polarity, (uint32_t) p_config->type,
                         (uint32_t) p_config->pullup);

  return HAL_OK;
}

/**
  * @brief Retrieve the tampalarm configuration.
  * @param p_config pointer to the tampalarm configuration instance.
  */
void HAL_RTC_OUTPUT_GetConfigTampalarm(hal_rtc_output_tampalarm_config_t  *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);

  uint32_t value_output_tampalarm = LL_RTC_READ_REG(CR);
  uint32_t polarity_temp = value_output_tampalarm & LL_RTC_OUTPUTPOLARITY_PIN_LOW;
  uint32_t type_temp = value_output_tampalarm & LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN;
  uint32_t pullup_temp = value_output_tampalarm & LL_RTC_ALARM_OUTPUT_PULLUP_ON;

  p_config->polarity = (hal_rtc_output_tampalarm_polarity_t) polarity_temp;
  p_config->type = (hal_rtc_output_tampalarm_type_t) type_temp;
  p_config->pullup = (hal_rtc_output_tampalarm_pullup_t) pullup_temp;
}

/**
  * @brief   Program the calibration output configuration.
  * @param   p_config pointer to the calibration output configuration instance.
  * @note    When using the synchronous calibration (1Hz by default), the calibration output is the output of the 8th
  *          stage of the synchronous prescaler. Thus only the 8 first bits are used to define the calibration frequency
  *          The frequency of the output is (f_RTCCLK/((PREDIV_A+1) * (PREDIV_S[7:0] +1)).
  * @note    When using the asynchronous calibration (512Hz by default), the calibration output is the output
  *          of the asynchronous prescaler 6th stage. Thus only the 6 first bits are
  *          used to define the calibration frequency. The frequency of the output is (f_RTCCLK / (PREDIV_A[5:0]+1)).
  * @warning If low power mode is disabled and the value of the asynchronous prescaler
  *          @ref hal_rtc_config_t::asynch_prediv is strictly below 0x20, the calibration signal output does not work.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_config is NULL.
  */
hal_status_t HAL_RTC_OUTPUT_SetConfigCalib(const hal_rtc_output_calib_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT_CALIB_FREQ(p_config->frequency));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_CAL_SetOutputFreq((uint32_t) p_config->frequency);

  return HAL_OK;
}

/**
  * @brief Retrieve the calibration output configuration.
  * @param p_config pointer to the calibration output configuration instance.
  */
void HAL_RTC_OUTPUT_GetConfigCalib(hal_rtc_output_calib_config_t *p_config)
{
  ASSERT_DBG_PARAM(p_config != NULL);

  p_config->frequency = (hal_rtc_output_calibration_frequency_t) LL_RTC_CAL_GetOutputFreq();
}

/**
  * @brief  Enable the output of the calibration signal or/and tampalarm signal.
  * @param  output Value of @ref hal_rtc_output_t.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_OUTPUT_Enable(hal_rtc_output_t output)
{
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT(output));

  uint32_t reg_value = LL_RTC_READ_REG(CR);

  uint32_t mask = (LL_RTC_CALIB_OUTPUT_512HZ   |
                   LL_RTC_OUTPUT_TAMPER_ENABLE |
                   LL_RTC_ALARMOUT_WAKEUP      |
                   LL_RTC_ALARM_OUTPUT_REMAP_POS1);
  reg_value |= (mask & (uint32_t)output);

  LL_RTC_WRITE_REG(CR, (reg_value));

  return HAL_OK;
}

/**
  * @brief  Disable the output RTC output.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_OUTPUT_Disable(void)
{
  LL_RTC_DisableOutput();

  return HAL_OK;
}

/**
  * @brief  Check if the RTC output is enabled or disabled.
  * @param  output Value of @ref hal_rtc_output_t representing the output configuration to be checked.
  * @retval hal_rtc_output_status_t:HAL_RTC_OUTPUT_DISABLED RTC output status.
  */
hal_rtc_output_status_t HAL_RTC_OUTPUT_IsEnabled(hal_rtc_output_t output)
{
  ASSERT_DBG_PARAM(IS_RTC_OUTPUT(output));
  return (hal_rtc_output_status_t) LL_RTC_IsEnabledDetailedOutput((uint32_t) output);
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Calibration
  * @brief Exported calibration functions.
  * @{
  *
  * # Smooth digital calibration
  *
  * The RTC frequency can be calibrated with a very small resolution of about 0.954 ppm with a range
  * from -487.1 ppm to 488.5 ppm. This adjustment is made on the RTCCLK or the clock after
  * the asynchronous prescaler when low power mode is enabled.
  *
  * To activate the smooth calibration, one needs to call  @ref HAL_RTC_EnableCalibration with
  * the parameters pulse_add
  * or subtracted_pulses set to @ref HAL_RTC_CALIBRATION_PULSE_INSERTED
  * or a value different from 0 respectively.
  * The subtracted_pulses specifies the number of pulses of the clock to be masked during the calibration cycle
  * chosen by the calibration_period. If calibration_period is set to
  * @ref HAL_RTC_CALIBRATION_PERIOD_16SEC or
  * @ref HAL_RTC_CALIBRATION_PERIOD_8SEC the first bit and the two first bit
  * are stuck at 0 respectively.
  *
  * Setting the pulse_add parameter inserts one extra pulse
  * every 2^11 cycles which doesn't dependent on the calibration period.
  *
  * The frequency after calibration is given by this formula
  * freq_calib = freq_RTCCLK x [1 + ( pulse_add x 512 - subtracted_pulses) /
  * (2^20 + subtracted_pulses - pulse_add x 512)]
  *
  * When the value of the asynchronous prescaler @ref hal_rtc_config_t::asynch_prediv is inferior to 3
  * no pulses are inserted, thus making the parameter pulse_add is ignored.
  *
  * To stop the calibration call  @ref HAL_RTC_DisableCalibration.
  * To retrieve the status of the calibration call @ref HAL_RTC_IsEnabledCalibration.
  *
  * # Synchronization of RTC the calendar
  *
  * After reading the subsecond register and calculating an offset with the remote clock, the goal of this feature
  * is to synchronize the RTC by adding/subtracting the offset to synchronize the RTC with the remote clock.
  * The shift operation consists of adding or subtracting some
  * subseconds depending on the parameters of the @ref HAL_RTC_ShiftCalibration.
  * The param add_one_sec adds one second to the time register and the param fraction_sec_to_subtract subtract
  * some subseconds to the subsecond register.
  *
  * The shift operation does not work properly in the following cases:
  * 1. In BCD mode :HAL_RTC_MODE_BCD, if the reference clock is enabled.
  * 2. In BCD mode :HAL_RTC_MODE_BCD, the 15th value of the subsecond register is
  *    equal to 1.
  * 3. In mixed mode :HAL_RTC_MODE_MIX, the fraction_sec_to_subtract
  *    [14, @ref hal_rtc_config_t::bcd_update ] must be equal to 0.
  * 4. In mixed mode :HAL_RTC_MODE_MIX, the BCD increment value
  *    @ref hal_rtc_config_t::bcd_update in the subsecond register must be equal to 0.
  *
  */

/**
  * @brief   Enable the smooth calibration parameters.
  * @param   calibration_period Select the smooth calibration period, values from @ref hal_rtc_calibration_period_t.
  * @param   pulse_add Choose between increasing the frequency by one pulse every 2^11 pulses
  *          or no pulse insertion (@ref hal_rtc_calibration_pulse_t).
  * @param   subtracted_pulses Number of pulses to subtract out of 2^20 pulses.
  * @note    To activate the smooth calibration, the parameters pulse_add
  *          or subtracted_pulses must be set to @ref HAL_RTC_CALIBRATION_PULSE_INSERTED
  *          or a value different from 0, respectively.
  *          If calibration_period is set to @ref HAL_RTC_CALIBRATION_PERIOD_16SEC or
  *          @ref HAL_RTC_CALIBRATION_PERIOD_8SEC the first bit and the first two bits
  *          are stuck at 0, respectively.
  *          If PREDIV_A (@ref hal_rtc_config_t::asynch_prediv) is less than 3,
  *          the bit CALP set by pulse_add is always equal to 0, thus pulse_add is ignored.
  *          The frequency after calibration is given by this formula:
  *          F_CAL = F_RTCCLK x [1 + ( pulse_add x 512 - subtracted_pulses) /
  *          (2^20 + subtracted_pulses - pulse_add x 512)]
  * @warning The RTC calibration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning When low power mode is enabled (use @ref HAL_RTC_EnableUltraLowPowerMode), the calibration mechanism
  *          is applied to the RTCCLK after the asynchronous prescaler.
  * @retval  HAL_OK
  * @retval  HAL_ERROR If a recalibration operation is still ongoing after RTC timeout duration.
  */
hal_status_t HAL_RTC_EnableCalibration(hal_rtc_calibration_period_t calibration_period,
                                       hal_rtc_calibration_pulse_t pulse_add,
                                       uint32_t subtracted_pulses)
{
  ASSERT_DBG_PARAM(IS_RTC_CALIBRATION_PERIOD(calibration_period));
  ASSERT_DBG_PARAM(IS_RTC_CALIBRATION_PULSE(pulse_add));
  ASSERT_DBG_PARAM(IS_RTC_CALIBRATION_SUBTRACTED_PULSES(subtracted_pulses));

  if (RTC_WaitSynchro_RECALP() != HAL_OK)
  {
    return HAL_ERROR;
  }

  LL_RTC_CAL_SetSmoothCalibration((uint32_t) calibration_period, (uint32_t) pulse_add, subtracted_pulses);

  return HAL_OK;
}

/**
  * @brief   Disable the smooth calibration parameters.
  * @warning The RTC calibration is write protected, use
  *          the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @retval  HAL_OK
  * @retval  HAL_ERROR If a recalibration operation is still ongoing after RTC timeout duration.
  */
hal_status_t HAL_RTC_DisableCalibration(void)
{
  if (RTC_WaitSynchro_RECALP() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* This is the only way to deactivate the calibration */
  LL_RTC_CAL_SetSmoothCalibration(0U, 0U, 0U);

  return HAL_OK;
}

/**
  * @brief  Check if the smooth calibration is enabled.
  * @retval hal_rtc_calibration_status_t RTC smooth calibration status.
  */
hal_rtc_calibration_status_t HAL_RTC_IsEnabledCalibration(void)
{
  return (hal_rtc_calibration_status_t) LL_RTC_CAL_IsEnabledSmoothCalibration();
}

/**
  * @brief   Shift the RTC clock (Subsecond Register) by a fraction of a second.
  * @param   add_one_sec Choose between adding one second to the clock.
  * @param   fraction_sec_to_subtract Number of fraction of a second to subtract.
  * @note    the RTC calibration is write protected.
  *          Use the @ref HAL_RTC_DisableWriteProtection before calling this function.
  * @warning This feature is not compatible with the reference clock detection feature enabled
  *          by @ref HAL_RTC_CALENDAR_EnableReferenceClock
  * @warning This function will not decrease the number of seconds nor overflow the subsecond register.
  * @retval  HAL_OK
  * @retval  HAL_ERROR The shift operation is still pending after RTC timeout duration.
  *                    RTC_SHIFTR must not be written when the reference clock detection is enabled.
  *                    In BCD mode, SS[15] must be 0 before writing RTC_SHIFTR to avoid overflow.
  *                    In mixed mode, SUBFS[14:BCDU+8] must be written with 0.
  *                    In mixed mode, SS[BCDU+8] must be 0 before writing RTC_SHIFTR.
  */
hal_status_t HAL_RTC_ShiftCalibration(hal_rtc_calibration_shift_second_t add_one_sec,
                                      uint32_t fraction_sec_to_subtract)
{
  ASSERT_DBG_PARAM(IS_RTC_CALIBRATION_SHIFT_SECOND(add_one_sec));
  ASSERT_DBG_PARAM(IS_RTC_CALIBRATION_SHIFT_FRACTIONS(fraction_sec_to_subtract));
  uint32_t value_sec_tmp;
  hal_status_t status = HAL_OK;
  uint32_t binary_mode = LL_RTC_GetBinaryMode();

  if (binary_mode == LL_RTC_BINARY_NONE)
  {
    /* Check if the 15th value of the subsecond register is equal to 0 or 1 in BCD mode */
    value_sec_tmp = LL_RTC_TIME_GetSubSecond();
    /* This is because of the shadow register */
    (void)LL_RTC_DATE_Get();

    if (((value_sec_tmp >> 15) & 1U) == 1U)
    {
      /* In BCD mode, SS[15] must be 0 before writing RTC_SHIFTR to avoid overflow. */
      status = HAL_ERROR;
    }
    if (LL_RTC_IsEnabledRefClock() == 1U)
    {
      /* RTC_SHIFTR must not be written when the reference clock detection is enabled. */
      status = HAL_ERROR;
    }
  }
  else if (binary_mode == LL_RTC_BINARY_MIX)
  {
    uint32_t bcd_increment = (LL_RTC_GetBinMixBCDU() >> LL_RTC_BINARY_MIX_BCDU_SHIFT) + 8U;

    if ((fraction_sec_to_subtract >> bcd_increment) != 0U)
    {
      /* In mixed mode, SUBFS[14:BCDU+8] must be written with 0. */
      status = HAL_ERROR;
    }

    value_sec_tmp = LL_RTC_TIME_GetSubSecond();
    /* This is because of the shadow register */
    (void)LL_RTC_DATE_Get();

    if (((value_sec_tmp >> bcd_increment) & 1U) == 1U)
    {
      /* In mixed mode, SS[BCDU+8] must be 0 before writing RTC_SHIFTR. */
      status = HAL_ERROR;
    }
  }
  else
  {
    /* In binary mode, no check */
  }

  if (status == HAL_OK)
  {
    /* Check that there is no shift ongoing */
    if (RTC_WaitSynchro_SHP() != HAL_OK)
    {
      return HAL_ERROR;
    }

    LL_RTC_TIME_Synchronize((uint32_t) add_one_sec, fraction_sec_to_subtract);

    if (RTC_WaitSynchro_SHP() != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (RTC_WaitSynchro_RS() != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return status;
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Alarms
  * @brief Exported alarm functions.
  * @{
  *
  * The RTC unit provides two programmable alarms. The alarms trigger when the calendar subseconds, seconds,
  * minutes, hours, day of month, or weekday match the values of the alarm.
  * Each field can be independently masked by setting @ref hal_rtc_alarm_date_time_t::mask.
  * When the alarm triggers, it can be automatically cleared by setting the parameter
  * @ref hal_rtc_alarm_config_t::auto_clear to @ref HAL_ALARM_AUTO_CLEAR_ENABLE.
  *
  * To update the alarms configuration, RTC must be in initialization mode or the
  * alarms must be disabled by using @ref HAL_RTC_EnterInitMode or @ref HAL_RTC_ALARM_Stop respectively.
  *
  * # Binary mode
  *
  * When using the binary mode only the subsecond registers are compared. A dedicated API is given
  * to optimize the code. The subseconds down-counter register can be reloaded every time the alarms triggers which
  * means that the subseconds down-counter register is running from 0xFFFF FFFF to the value set in the subseconds
  * registers of the alarm. This can only be used in binary mode.
  * This is configured by setting @ref hal_rtc_alarm_config_t::subsec_auto_reload to
  *  @ref ::HAL_RTC_ALARM_SUBSECONDS_AUTO_RELOAD_ENABLE.
  *
  */

/**
  * @brief  Configure the alarm parameters.
  * @param  alarm Select the alarm A or B.
  * @param  p_config_alarm Pointer to the alarm configuration instance.
  * @note   @ref hal_rtc_alarm_config_t::subsec_auto_reload can be enabled only in binary mode.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM when p_config_alarm is NULL.
  * @retval HAL_ERROR If the alarm is started and the RTC is not in initialization mode.
  *                   or if p_config_alarm->subsec_auto_reload is set to LL_RTC_ALMA_SUBSECONDBIN_AUTOCLR_YES
  *                   and RTC is in BCD or Mixed mode.
  */
hal_status_t HAL_RTC_ALARM_SetConfig(hal_rtc_alarm_t alarm, const hal_rtc_alarm_config_t *p_config_alarm)
{
  ASSERT_DBG_PARAM(p_config_alarm != NULL);
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_BINARY_AUTO_RELOAD(p_config_alarm->subsec_auto_reload));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_AUTO_CLEAR(p_config_alarm->auto_clear));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config_alarm == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  if (LL_RTC_ALM_IsStarted((uint32_t)alarm) == 1U)
  {
    if (LL_RTC_IsActiveFlag_INIT() == 0U)
    {
      return HAL_ERROR;
    }
  }

  LL_RTC_ALM_SetBinAutoClr((uint32_t) alarm, (uint32_t) p_config_alarm->subsec_auto_reload);
  LL_RTC_ALM_SetFlagAutoClr((uint32_t) alarm, (uint32_t)p_config_alarm->auto_clear);

  return HAL_OK;
}

/**
  * @brief Retrieve the alarm configuration.
  * @param alarm Select the alarm A or B.
  * @param p_config_alarm Pointer to the alarm configuration instance.
  */
void HAL_RTC_ALARM_GetConfig(hal_rtc_alarm_t alarm, hal_rtc_alarm_config_t *p_config_alarm)
{
  ASSERT_DBG_PARAM(p_config_alarm != NULL);
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  p_config_alarm->subsec_auto_reload = (hal_rtc_alarm_subseconds_auto_reload_t)
                                       LL_RTC_ALM_GetBinAutoClr((uint32_t) alarm);
  p_config_alarm->auto_clear = (hal_rtc_alarm_auto_clear_t) LL_RTC_ALM_GetFlagAutoClr((uint32_t) alarm);
}

/**
  * @brief   Configure the alarm date and time.
  * @param   alarm Select the alarm A or B.
  * @param   p_date_time Pointer to the alarm date and time instance.
  * @warning The synchronous prescaler value, @ref hal_rtc_config_t::synch_prediv,
  *          must be greater than 3 when seconds are unmasked.
  * @retval  HAL_OK
  * @retval  HAL_INVALID_PARAM when p_date_time is NULL.
  * @retval  HAL_ERROR if the alarm is started and the RTC is not in initialization mode.
  */
hal_status_t HAL_RTC_ALARM_SetDateTime(hal_rtc_alarm_t alarm, const hal_rtc_alarm_date_time_t *p_date_time)
{
  ASSERT_DBG_PARAM(p_date_time != NULL);
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_MASK(p_date_time->mask));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_DAY_TYPE(p_date_time->mday_wday_selection));
  ASSERT_DBG_PARAM(IS_RTC_WEEKDAY(p_date_time->wday_mday.wday) || IS_RTC_MONTHDAY_NBR(p_date_time->wday_mday.mday));
  ASSERT_DBG_PARAM(IS_RTC_TIME_FORMAT(p_date_time->time.am_pm));
  ASSERT_DBG_PARAM(IS_RTC_HOUR(p_date_time->time.hour, (hal_rtc_calendar_hour_format_t) LL_RTC_GetHourFormat()));
  ASSERT_DBG_PARAM(IS_RTC_MIN(p_date_time->time.min));
  ASSERT_DBG_PARAM(IS_RTC_SEC(p_date_time->time.sec));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_SUBSECONDS(p_date_time->time.subsec));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_SUBSECONDS_MASK(p_date_time->subsec_mask));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_date_time == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  uint32_t alarm_wday_mday;

  if (LL_RTC_ALM_IsStarted((uint32_t)alarm) == 1U)
  {
    if (LL_RTC_IsActiveFlag_INIT() == 0U)
    {
      return HAL_ERROR;
    }
  }

  if (p_date_time->mday_wday_selection == HAL_RTC_ALARM_DAY_TYPE_SEL_MONTHDAY)
  {
    alarm_wday_mday = p_date_time->wday_mday.mday;
  }
  else
  {
    alarm_wday_mday = (uint32_t) p_date_time->wday_mday.wday;
  }

  LL_RTC_ALM_SetConfigDateTime((uint32_t) alarm, p_date_time->mask,
                               (uint32_t) p_date_time->mday_wday_selection,
                               HAL_RTC_CONVERT_DEC2BCD(alarm_wday_mday),
                               (uint32_t) p_date_time->time.am_pm,
                               HAL_RTC_CONVERT_DEC2BCD(p_date_time->time.hour),
                               HAL_RTC_CONVERT_DEC2BCD(p_date_time->time.min),
                               HAL_RTC_CONVERT_DEC2BCD(p_date_time->time.sec));

  if (LL_RTC_GetBinaryMode() == LL_RTC_BINARY_MIX)
  {
    LL_RTC_ALM_SetBinarySubSecond((uint32_t) alarm, p_date_time->time.subsec);
    LL_RTC_ALM_SetSubSecondMask((uint32_t) alarm, p_date_time->subsec_mask);
  }
  else
  {
    LL_RTC_ALM_SetConfigSubSecond((uint32_t) alarm, p_date_time->subsec_mask,
                                  p_date_time->time.subsec);
  }


  return HAL_OK;
}

/**
  * @brief Retrieve the alarm date and time.
  * @param alarm Select the alarm A or B.
  * @param p_date_time Pointer to the alarm date and time instance.
  */
void HAL_RTC_ALARM_GetDateTime(hal_rtc_alarm_t alarm, hal_rtc_alarm_date_time_t *p_date_time)
{
  ASSERT_DBG_PARAM(p_date_time != NULL);
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  uint32_t alarm_register_value = LL_RTC_ALM_GetConfigDateTime((uint32_t)alarm);
  uint32_t alarm_day_wday_sel = LL_RTC_GET_ALARM_DAY_WDAY_SEL(alarm_register_value);
  uint32_t alarm_am_pm_value = LL_RTC_GET_ALARM_FORMAT(alarm_register_value);

  p_date_time->mask = LL_RTC_GET_ALARM_MASKS(alarm_register_value);
  p_date_time->mday_wday_selection = (hal_rtc_alarm_day_type_selection_t) alarm_day_wday_sel;

  uint32_t day_alarm_value = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_ALARM_DAY(alarm_register_value));

  if (p_date_time->mday_wday_selection == HAL_RTC_ALARM_DAY_TYPE_SEL_MONTHDAY)
  {
    p_date_time->wday_mday.mday = day_alarm_value;
  }
  else
  {
    p_date_time->wday_mday.wday = (hal_rtc_weekday_t) day_alarm_value;
  }

  p_date_time->time.am_pm = (hal_rtc_time_format_am_pm_t) alarm_am_pm_value;
  p_date_time->time.hour = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_ALARM_HOUR(alarm_register_value));
  p_date_time->time.min = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_ALARM_MINUTE(alarm_register_value));
  p_date_time->time.sec = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_ALARM_SECOND(alarm_register_value));

  uint32_t alarm_ss_register_value = LL_RTC_ALM_GetConfigSubSecond((uint32_t) alarm);

  if (LL_RTC_GetBinaryMode() == LL_RTC_BINARY_MIX)
  {
    p_date_time->time.subsec = LL_RTC_ALM_GetSubSecondMask((uint32_t) alarm);
  }
  else
  {
    p_date_time->time.subsec = LL_RTC_ALARM_GET_SS(alarm_ss_register_value);
  }

  p_date_time->subsec_mask = LL_RTC_ALARM_GET_MASK_SS(alarm_ss_register_value);
}

/**
  * @brief  Start the alarm.
  * @param  alarm Select the alarm A or B.
  * @param  interruption Enable the interrupt.
  *         @arg @ref HAL_RTC_ALARM_IT_DISABLE
  *         @arg @ref HAL_RTC_ALARM_IT_ENABLE
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_ALARM_Start(hal_rtc_alarm_t alarm, uint32_t interruption)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_IT(interruption));

  LL_RTC_ALM_Start((uint32_t) alarm, interruption);

  return HAL_OK;
}

/**
  * @brief  Stop the alarm.
  * @param  alarm Select the alarm A or B.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_ALARM_Stop(hal_rtc_alarm_t alarm)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  LL_RTC_ALM_Stop((uint32_t) alarm);

  return HAL_OK;
}

/**
  * @brief  Poll for alarm event.
  * @param  alarm Select the alarm A or B.
  * @param  timeout_ms Timeout duration.
  * @retval HAL_OK
  * @retval HAL_TIMEOUT when reaching the timeout during polling.
  */
hal_status_t HAL_RTC_ALARM_PollForEvent(hal_rtc_alarm_t alarm, uint32_t timeout_ms)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM((uint32_t) alarm));

  uint32_t tickstart = HAL_GetTick();

  while (LL_RTC_IsActiveFlag_ALR((uint32_t) alarm) == 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (LL_RTC_IsActiveFlag_ALR((uint32_t) alarm) == 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }

  LL_RTC_ClearFlag_ALR((uint32_t) alarm);

  return HAL_OK;
}

/**
  * @brief  Program the alarm subseconds.
  * @param  alarm Select the alarm A or B.
  * @param  alarm_subsecond Alarm subsecond value. Value between 0x0 and 0xFFFFFFFF.
  * @note   Use in binary mode only. Prefer @ref HAL_RTC_ALARM_SetDateTime to program the alarm in BCD or mixed mode.
  * @retval HAL_OK
  * @retval HAL_ERROR
  */
hal_status_t HAL_RTC_ALARM_SetBinaryTime(hal_rtc_alarm_t alarm, uint32_t alarm_subsecond)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  if (LL_RTC_ALM_IsStarted((uint32_t)alarm) == 1U)
  {
    if (LL_RTC_IsActiveFlag_INIT() == 0U)
    {
      return HAL_ERROR;
    }
  }

  LL_RTC_ALM_SetBinarySubSecond((uint32_t) alarm, alarm_subsecond);

  return HAL_OK;
}

/**
  * @brief  Retrieve the alarm subseconds.
  * @param  alarm Select the alarm A or B.
  * @note   Use in binary mode only. Prefer @ref HAL_RTC_ALARM_GetDateTime to
  *         retrieve alarm subseconds in BCD or mixed mode.
  * @retval uint32_t Alarm subseconds.
  */
uint32_t HAL_RTC_ALARM_GetBinaryTime(hal_rtc_alarm_t alarm)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  return LL_RTC_ALM_GetBinarySubSecond((uint32_t) alarm);
}

/**
  * @brief  Program the alarm subseconds mask.
  * @param  alarm Select the alarm A or B.
  * @param  alarm_subsecond_mask Alarm subsecond mask. Value between 0x0 and 0x3F.
  * @note   Use in binary mode only. Prefer @ref HAL_RTC_ALARM_SetDateTime to program the alarm in BCD or mixed mode.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_ALARM_SetBinarySubSecondMask(hal_rtc_alarm_t alarm, uint32_t alarm_subsecond_mask)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));
  ASSERT_DBG_PARAM(IS_RTC_ALARM_SUBSECONDS_MASK(alarm_subsecond_mask));

  if (LL_RTC_ALM_IsStarted((uint32_t)alarm) == 1U)
  {
    if (LL_RTC_IsActiveFlag_INIT() == 0U)
    {
      return HAL_ERROR;
    }
  }

  LL_RTC_ALM_SetSubSecondMask((uint32_t) alarm, alarm_subsecond_mask);

  return HAL_OK;
}

/**
  * @brief  Retrieve the alarm subseconds mask.
  * @param  alarm Select the alarm A or B.
  * @note   Use in binary mode only. Prefer @ref HAL_RTC_ALARM_GetDateTime to
  *         retrieve alarm subseconds mask in BCD or mixed mode.
  * @retval uint32_t Alarm subseconds mask.
  */
uint32_t HAL_RTC_ALARM_GetBinarySubSecondMask(hal_rtc_alarm_t alarm)
{
  ASSERT_DBG_PARAM(IS_RTC_ALARM(alarm));

  return LL_RTC_ALM_GetSubSecondMask((uint32_t) alarm);
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Timestamp
  * @brief Exported timestamp functions.
  * @{
  *
  * Timestamp is enabled by setting the TSE bit of RTC_CR register to 1.
  * When TSE is set, the calendar is saved in the timestamp registers (RTC_TSSSR, RTC_TSTR, RTC_TSDR)
  * when a timestamp event is detected on the RTC_TS pin.
  *
  * When TAMPTS is set:
  * The calendar is saved in the timestamp registers (RTC_TSSSR, RTC_TSTR, RTC_TSDR)
  * when an internal or external tamper event is detected.
  *
  * After a timestamp event, by calling the @ref HAL_RTC_TIMESTAMP_GetDateTime, the application
  * can retrieve the date and time of the last timestamp event and clear the timestamp registers.
  *
  */

/**
  * @brief  Configure the RTC timestamp.
  * @param  p_config_timestamp pointer to an RTC timestamp configuration instance.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM when p_config_timestamp is NULL.
  */
hal_status_t HAL_RTC_TIMESTAMP_SetConfig(const hal_rtc_timestamp_config_t *p_config_timestamp)
{
  ASSERT_DBG_PARAM(p_config_timestamp != NULL);
  ASSERT_DBG_PARAM(IS_RTC_TIMESTAMP_SOURCE_PIN_EDGE(p_config_timestamp->input_edge_polarity));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config_timestamp == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  LL_RTC_TS_SetActiveEdge((uint32_t) p_config_timestamp->input_edge_polarity);

  return HAL_OK;
}

/**
  * @brief Retrieve the RTC timestamp configuration.
  * @param p_config_timestamp pointer to an RTC timestamp configuration instance.
  */
void HAL_RTC_TIMESTAMP_GetConfig(hal_rtc_timestamp_config_t *p_config_timestamp)
{
  ASSERT_DBG_PARAM(p_config_timestamp != NULL);

  p_config_timestamp->input_edge_polarity = (hal_rtc_timestamp_source_pin_edge_t) LL_RTC_TS_GetActiveEdge();
}

/**
  * @brief  Enable the RTC timestamp from the external pin (RTC_TS).
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_EnablePinSource(void)
{
  LL_RTC_TS_Enable();

  return HAL_OK;
}

/**
  * @brief  Disable the RTC timestamp from the external pin (RTC_TS).
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_DisablePinSource(void)
{
  LL_RTC_TS_Disable();

  return HAL_OK;
}

/**
  * @brief  Check if the RTC timestamps from the external pin (RTC_TS) is enabled.
  * @retval hal_rtc_timestamp_status_t RTC timestamp from the external pin status.
  */
hal_rtc_timestamp_status_t HAL_RTC_TIMESTAMP_IsEnabledPinSource(void)
{
  return (hal_rtc_timestamp_status_t) LL_RTC_TS_IsEnabled();
}


/**
  * @brief  Enable the RTC timestamp on tamper event.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_EnableTamperSource(void)
{
  LL_RTC_TS_EnableOnTamper();

  return HAL_OK;
}

/**
  * @brief  Disable the RTC timestamp on tamper event.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_DisableTamperSource(void)
{
  LL_RTC_TS_DisableOnTamper();

  return HAL_OK;
}

/**
  * @brief  Check whether the RTC timestamp on tamper event is enabled.
  * @retval hal_rtc_timestamp_tamper_status_t RTC timestamp on tamper event status.
  */
hal_rtc_timestamp_tamper_status_t HAL_RTC_TIMESTAMP_IsEnabledTamperSource(void)
{
  return (hal_rtc_timestamp_tamper_status_t) LL_RTC_TS_IsEnabledOnTamper();
}

/**
  * @brief  Enable the RTC timestamp interrupt.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_EnableIT(void)
{
  LL_RTC_EnableIT_TS();

  return HAL_OK;
}

/**
  * @brief  Disable the RTC timestamp interrupt.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_TIMESTAMP_DisableIT(void)
{
  LL_RTC_DisableIT_TS();

  return HAL_OK;
}

/**
  * @brief  Check if the RTC timestamp interrupt is enabled.
  * @retval hal_rtc_timestamp_it_status_t RTC timestamp interrupt status.
  */
hal_rtc_timestamp_it_status_t HAL_RTC_TIMESTAMP_IsEnabledIT(void)
{
  return (hal_rtc_timestamp_it_status_t) LL_RTC_IsEnabledIT_TS();
}

/**
  * @brief  Retrieve the RTC timestamp time and the source of the timestamp event.
  *         Due to register limitations, the year is forced to 0 when reading the timestamp.
  * @param  p_time pointer to an RTC timestamp time instance.
  * @param  p_date pointer to an RTC timestamp date instance.
  * @param  p_info pointer to a @ref hal_rtc_timestamp_information_t value.
  *         It is used to retrieve the flag of the timestamp event.
  * @retval HAL_OK
  * @retval HAL_INVALID_PARAM when p_date or p_time are NULL.
  * @retval HAL_ERROR When an overflow occurred
  */
hal_status_t HAL_RTC_TIMESTAMP_GetDateTime(hal_rtc_time_t *p_time, hal_rtc_date_t *p_date,
                                           hal_rtc_timestamp_information_t *p_info)
{
  ASSERT_DBG_PARAM(p_time != NULL);
  ASSERT_DBG_PARAM(p_date != NULL);
  ASSERT_DBG_PARAM(p_info != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_date == NULL) || (p_time == NULL) || (p_info == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  uint32_t time_and_format = LL_RTC_TS_GetTimeAndFormat();
  uint32_t format = LL_RTC_GET_FORMAT(time_and_format);
  uint32_t date = LL_RTC_TS_GetDate();
  uint32_t week_day = LL_RTC_GET_WEEKDAY(date);
  uint32_t month = LL_RTC_GET_MONTH(date);
  hal_status_t status = HAL_OK;

  /* Get timestamp time information */
  p_time->am_pm = (hal_rtc_time_format_am_pm_t) format;
  p_time->hour = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_HOUR(time_and_format));
  p_time->min = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_MINUTE(time_and_format));
  p_time->sec = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_SECOND(time_and_format));
  p_time->subsec = LL_RTC_TS_GetSubSecond();

  /* Get timestamp date information */
  p_date->mon = (hal_rtc_month_t) month;
  p_date->wday = (hal_rtc_weekday_t) week_day;
  p_date->mday = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_DAY(date));
  p_date->year = 0U;

  p_info->flag = (hal_rtc_timestamp_event_flag_t)((uint32_t)(LL_RTC_READ_REG(SR) & (LL_RTC_SR_TSF | LL_RTC_SR_TSOVF)));

  if (((uint32_t)p_info->flag & (uint32_t)HAL_RTC_TIMESTAMP_EVENT) == (uint32_t)HAL_RTC_TIMESTAMP_EVENT)
  {
    /* It is recommended to check and then clear TSOVF only after clearing the TSF bit */
    LL_RTC_ClearFlag_TS();

    if (LL_RTC_IsActiveFlag_TSOV() != 0U)
    {
      p_info->flag = (hal_rtc_timestamp_event_flag_t)((uint32_t)((uint32_t)p_info->flag \
                                                                 | (uint32_t)(HAL_RTC_TIMESTAMP_OVERFLOW_EVENT)));
      status = HAL_ERROR;
    }
  }

  /* Clear all timestamp flags */
  LL_RTC_WRITE_REG(SCR, LL_RTC_SCR_TSF | LL_RTC_SCR_TSOVF);

  return status;
}

/**
  * @brief Poll for timestamp event.
  * @param timeout_ms Timeout duration
  * @retval HAL_OK
  * @retval HAL_TIMEOUT when the timeout is reached.
  */
hal_status_t HAL_RTC_TIMESTAMP_PollForEvent(uint32_t timeout_ms)
{
  uint32_t tickstart = HAL_GetTick();

  while (LL_RTC_IsActiveFlag_TS() == 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (LL_RTC_IsActiveFlag_TS() == 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Retrieve the RTC timestamp subsecond register value.
  * @param  p_time_subseconds pointer to a uint32_t to retrieve the value of the subseconds of the timestamp.
  * @param  p_info pointer to a @ref hal_rtc_timestamp_information_t value.
  *         It is used to retrieve the flag of the timestamp event.
  * @retval HAL_OK
  * @retval HAL_ERROR If an overflow occurred
  */
hal_status_t HAL_RTC_TIMESTAMP_GetBinaryTime(uint32_t *p_time_subseconds, hal_rtc_timestamp_information_t *p_info)
{
  ASSERT_DBG_PARAM(p_time_subseconds != NULL);
  ASSERT_DBG_PARAM(p_info != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_time_subseconds == NULL) || (p_info == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hal_status_t status = HAL_OK;

  /* Get timestamp subseconds */
  *p_time_subseconds = LL_RTC_TS_GetSubSecond();

  p_info->flag = (hal_rtc_timestamp_event_flag_t)((uint32_t)(LL_RTC_READ_REG(SR) & (LL_RTC_SR_TSF | LL_RTC_SR_TSOVF)));

  if (((uint32_t)p_info->flag & (uint32_t)HAL_RTC_TIMESTAMP_EVENT) == (uint32_t)HAL_RTC_TIMESTAMP_EVENT)
  {
    /* It is recommended to check and then clear TSOVF only after clearing the TSF bit */
    LL_RTC_ClearFlag_TS();

    if (LL_RTC_IsActiveFlag_TSOV() != 0U)
    {
      p_info->flag = (hal_rtc_timestamp_event_flag_t)((uint32_t)((uint32_t)p_info->flag \
                                                                 | (uint32_t)(HAL_RTC_TIMESTAMP_OVERFLOW_EVENT)));
      status = HAL_ERROR;
    }
  }

  /* Clear all timestamp flags */
  LL_RTC_WRITE_REG(SCR, LL_RTC_SCR_TSF | LL_RTC_SCR_TSOVF);

  return status;
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_WakeUp_Timer
  * @brief Exported wake-up timer functions.
  * @{
  *
  * The wake-up timer is a 16-bit auto-reload down-counter. The wake-up timer clock input can be one of:
  * * The RTCCLK divided by a dedicated prescaler of 2, 4, 8, or 16.
  * * The synchronous clock that is configured by changing the values of the RTC
  *   prescalers @ref hal_rtc_config_t::asynch_prediv and @ref hal_rtc_config_t::synch_prediv.
  *
  * Call @ref HAL_RTC_WAKEUP_SetConfig and configure @ref hal_rtc_wakeup_config_t::clock
  * to choose the input clock of the wake-up timer. When using the synchronous clock
  * input, the wake-up timer can behave like a 17-bit auto-reload down-counter by
  * setting the value @ref ::HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT to
  * @ref hal_rtc_wakeup_config_t::clock. In this case, the 16-bit down-counter
  * reaches 0 two times; the first time it reloads automatically but triggers a
  * wake-up event. The second time it does not reload automatically but also
  * triggers a wake-up event.
  *
  * By setting the parameter p_auto_clear_time to a non-zero value in function
  * @ref HAL_RTC_WAKEUP_SetPeriod, the wake-up timer flag is cleared by hardware
  * when the auto-reload down-counter reaches this value.
  *
  * To configure the wake-up timer without using physical time units, call
  * @ref HAL_RTC_WAKEUP_SetAutoReloadAndAutoClear.
  */

/**
  * @brief  Configure the RTC wake-up timer.
  * @param  p_config_wakeup_timer Pointer to an RTC wake-up timer configuration instance.
  * @retval HAL_OK
  * @retval HAL_ERROR If not in initialization mode and wake-up timer is started.
  *                   If not in initialization mode and WUTWF is still unset after RTC timeout duration.
  * @retval HAL_INVALID_PARAM When p_config_wakeup_timer is NULL.
  */
hal_status_t HAL_RTC_WAKEUP_SetConfig(const hal_rtc_wakeup_config_t *p_config_wakeup_timer)
{
  ASSERT_DBG_PARAM(p_config_wakeup_timer != NULL);
  ASSERT_DBG_PARAM(IS_RTC_WAKEUP_TIMER_CLOCK(p_config_wakeup_timer->clock));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_config_wakeup_timer == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hal_status_t synchro_status = RTC_WaitSynchro_WUTW();

  if (synchro_status != HAL_OK)
  {
    return synchro_status;
  }

  LL_RTC_WAKEUP_SetClock((uint32_t) p_config_wakeup_timer->clock);

  return HAL_OK;
}

/**
  * @brief Retrieve the configuration of the RTC wake-up timer.
  * @param p_config_wakeup_timer Pointer to an RTC wake-up timer configuration instance.
  */
void HAL_RTC_WAKEUP_GetConfig(hal_rtc_wakeup_config_t *p_config_wakeup_timer)
{
  ASSERT_DBG_PARAM(p_config_wakeup_timer != NULL);

  p_config_wakeup_timer->clock = (hal_rtc_wakeup_timer_clock_t) LL_RTC_WAKEUP_GetClock();
}

/**
  * @brief  Program the RTC wake-up timer auto-reload time and auto-reload clear flag time.
  * @param  p_auto_reload_time Pointer to an RTC time instance representing the wake-up reload period.
  * @param  p_auto_clear_time Pointer to an RTC time instance representing the wake-up auto-clear flag time.
  * @note   @ref hal_rtc_time_t::am_pm field is ignored because it is not used in the SetPeriod function.
  * @note   @ref hal_rtc_time_t::subsec field is ignored because it is not used in the SetPeriod function.
  * @retval HAL_OK
  * @retval HAL_ERROR The wake-up timer is still in use.
  *                   The wake-up timer frequency is smaller than 1 Hz;
  *                   use @ref HAL_RTC_WAKEUP_SetAutoReloadAndAutoClear.
  *                   Auto-reload time is shorter than auto-reload clear flag time.
  *                   The value of the auto-reload time is too big or too small or the wake-up timer counter.
  *                   The wake-up write flag is still cleared after timeout.
  * @retval HAL_INVALID_PARAM p_period is NULL.
  */
hal_status_t HAL_RTC_WAKEUP_SetPeriod(const hal_rtc_time_t *p_auto_reload_time, const hal_rtc_time_t *p_auto_clear_time)
{
  ASSERT_DBG_PARAM(p_auto_reload_time != NULL);
  ASSERT_DBG_PARAM(p_auto_clear_time != NULL);
  ASSERT_DBG_PARAM(IS_RTC_HOUR_36(p_auto_reload_time->hour));
  ASSERT_DBG_PARAM(IS_RTC_MIN(p_auto_reload_time->min));
  ASSERT_DBG_PARAM(IS_RTC_SEC(p_auto_reload_time->sec));
  ASSERT_DBG_PARAM(IS_RTC_MICROSEC(p_auto_reload_time->microsec));
  ASSERT_DBG_PARAM(IS_RTC_MILLISEC(p_auto_reload_time->millisec));

  ASSERT_DBG_PARAM(IS_RTC_HOUR_36(p_auto_clear_time->hour));
  ASSERT_DBG_PARAM(IS_RTC_MIN(p_auto_clear_time->min));
  ASSERT_DBG_PARAM(IS_RTC_SEC(p_auto_clear_time->sec));
  ASSERT_DBG_PARAM(IS_RTC_MICROSEC(p_auto_clear_time->microsec));
  ASSERT_DBG_PARAM(IS_RTC_MILLISEC(p_auto_clear_time->millisec));

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_auto_reload_time == NULL) || (p_auto_clear_time == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  hal_status_t status;
  uint32_t seconds_auto_reload;
  uint32_t seconds_auto_clear;
  uint32_t microseconds_auto_reload;
  uint32_t microseconds_auto_clear;
  uint32_t frequency_wakeup;
  uint32_t bits_wakeup;
  uint32_t bits_auto_clear;
  uint32_t max_seconds;
  uint32_t min_seconds;
  hal_rtc_wakeup_timer_clock_t clock_prescaler_wakeup;

  if (LL_RTC_WAKEUP_IsEnabled() == 1U)
  {
    return HAL_ERROR;
  }

  seconds_auto_reload = (p_auto_reload_time->hour * RTC_HOUR_SECONDS) +
                        (p_auto_reload_time->min * RTC_MIN_SECONDS) +
                        (p_auto_reload_time->sec);
  seconds_auto_clear = (p_auto_clear_time->hour * RTC_HOUR_SECONDS) +
                       (p_auto_clear_time->min * RTC_MIN_SECONDS) +
                       (p_auto_clear_time->sec);

  microseconds_auto_reload = (p_auto_reload_time->microsec) +
                             (p_auto_reload_time->millisec * RTC_MILLIMICROSECONDS);

  microseconds_auto_clear = (p_auto_clear_time->microsec) +
                            (p_auto_clear_time->millisec * RTC_MILLIMICROSECONDS);

  /* We verify that the flag auto clear time is smaller than the auto-reload time */
  if ((seconds_auto_clear > seconds_auto_reload)
      || ((seconds_auto_clear == seconds_auto_reload) && (microseconds_auto_reload < microseconds_auto_clear)))
  {
    return HAL_ERROR;
  }

  /* Get the frequency of the wake-up timer clock to perform seconds/microsecond conversion into bits */
  clock_prescaler_wakeup = (hal_rtc_wakeup_timer_clock_t) LL_RTC_WAKEUP_GetClock();
  frequency_wakeup = RTC_GetWakeUpClockFrequency(clock_prescaler_wakeup);

  /* Frequency slower than 1Hz triggers an error */
  if (frequency_wakeup == 0U)
  {
    return HAL_ERROR;
  }

  /* We check that we do not overflow the values. We only check the reload time because
  we checked that the flag_auto_clear time is smaller than the reload time */
  if (clock_prescaler_wakeup == HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT)
  {
    max_seconds = RTC_MAX_WAKEUP_VALUE_17BITS / frequency_wakeup;
    min_seconds = RTC_MIN_WAKEUP_VALUE_17BITS / frequency_wakeup;
  }
  else
  {
    min_seconds = RTC_MIN_WAKEUP_VALUE_16BITS / frequency_wakeup;
    max_seconds = RTC_MAX_WAKEUP_VALUE_16BITS / frequency_wakeup;
  }

  if ((seconds_auto_reload >= max_seconds) || (seconds_auto_reload <= min_seconds))
  {
    return HAL_ERROR;
  }

  bits_wakeup = RTC_ConvertSecSubsecToBits(seconds_auto_reload,
                                           p_auto_reload_time->millisec,
                                           p_auto_reload_time->microsec,
                                           frequency_wakeup);
  bits_auto_clear = RTC_ConvertSecSubsecToBits(seconds_auto_clear,
                                               p_auto_clear_time->millisec,
                                               p_auto_clear_time->microsec,
                                               frequency_wakeup);
  status = RTC_WaitSynchro_WUTW();
  if (status != HAL_OK)
  {
    return status;
  }

  LL_RTC_WAKEUP_Config(bits_wakeup, bits_auto_clear);

  return HAL_OK;
}

/**
  * @brief Retrieve the RTC wake-up timer auto-reload and auto-reload clear value.
  * @param p_auto_reload_time pointer to an RTC time instance representing the wake-up reload period.
  * @param p_auto_clear_time pointer to an RTC time instance representing the wake-up auto-clear flag time.
  */
void HAL_RTC_WAKEUP_GetPeriod(hal_rtc_time_t *p_auto_reload_time, hal_rtc_time_t *p_auto_clear_time)
{
  ASSERT_DBG_PARAM(p_auto_reload_time != NULL);
  ASSERT_DBG_PARAM(p_auto_clear_time != NULL);

  uint32_t bits_auto_reload;
  uint32_t bits_flag_auto_clear;
  uint32_t frequency_wakeup;
  uint32_t value_reg_wakeup;
  hal_rtc_wakeup_timer_clock_t clock_prescaler_wakeup;

  clock_prescaler_wakeup = (hal_rtc_wakeup_timer_clock_t) LL_RTC_WAKEUP_GetClock();
  frequency_wakeup = RTC_GetWakeUpClockFrequency(clock_prescaler_wakeup);

  /* When the wake-up clock selection is set to HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT, add one bit to the value
    of the wake-up auto-reload time from the RTC_WUTR register */

  value_reg_wakeup = LL_RTC_READ_REG(WUTR);

  bits_auto_reload = LL_RTC_GET_WAKEUP_AUTORELOAD(value_reg_wakeup);
  bits_flag_auto_clear = LL_RTC_GET_WAKEUP_AUTOCLEAR(value_reg_wakeup);

  if (clock_prescaler_wakeup == HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE_ADD_1BIT)
  {
    bits_auto_reload += RTC_WAKEUP_TIMER_CLOCK_SECONDS_ADD_1BIT;
    bits_flag_auto_clear += RTC_WAKEUP_TIMER_CLOCK_SECONDS_ADD_1BIT;
  }

  RTC_ConvertBitsToTime(p_auto_reload_time, bits_auto_reload, frequency_wakeup);
  RTC_ConvertBitsToTime(p_auto_clear_time, bits_flag_auto_clear, frequency_wakeup);
}

/**
  * @brief  Start the wake-up timer.
  * @param  interruption Specifies the wake-up timer mode.
  *         @arg @ref HAL_RTC_WAKEUP_IT_DISABLE
  *         @arg @ref HAL_RTC_WAKEUP_IT_ENABLE
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_WAKEUP_Start(uint32_t interruption)
{
  ASSERT_DBG_PARAM(IS_RTC_WAKEUP_IT(interruption));
  LL_RTC_WAKEUP_Start(interruption);

  return HAL_OK;
}

/**
  * @brief Stops the wake-up timer.
  * @retval HAL_OK
  */
hal_status_t HAL_RTC_WAKEUP_Stop(void)
{
  LL_RTC_WAKEUP_Stop();

  return HAL_OK;
}

/**
  * @brief  Poll for wake-up timer event.
  * @param  timeout_ms Timeout duration.
  * @retval HAL_OK
  * @retval HAL_TIMEOUT If WUTF is unset after timeout duration.
  */
hal_status_t HAL_RTC_WAKEUP_PollForEvent(uint32_t timeout_ms)
{
  uint32_t tickstart = HAL_GetTick();

  while (LL_RTC_IsActiveFlag_WUT() == 0U)
  {
    if (timeout_ms != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tickstart) > timeout_ms) || (timeout_ms == 0U))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if (LL_RTC_IsActiveFlag_WUT() == 0U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }

  LL_RTC_ClearFlag_WUT();

  return HAL_OK;
}

/**
  * @brief  Program the RTC wake-up timer auto-reload and auto-reload clear value in binary.
  * @param  wakeup_timer_auto_reload Wake-up timer auto-reload time in binary
  * @param  wakeup_timer_auto_clear Wake-up timer auto-reload clear value in binary
  * @retval HAL_OK
  * @retval HAL_ERROR If WUTF is unset after timeout duration.
  */
hal_status_t HAL_RTC_WAKEUP_SetAutoReloadAndAutoClear(uint32_t wakeup_timer_auto_reload,
                                                      uint32_t wakeup_timer_auto_clear)
{
  ASSERT_DBG_PARAM(IS_RTC_WAKEUP_VALUE(wakeup_timer_auto_reload));
  ASSERT_DBG_PARAM(IS_RTC_WAKEUP_AUTOCLEAR_VALUE(wakeup_timer_auto_clear));
  ASSERT_DBG_PARAM(IS_RTC_WAKEUP_AUTORELOAD_AUTOCLEAR(wakeup_timer_auto_reload, wakeup_timer_auto_clear));

  hal_status_t status = RTC_WaitSynchro_WUTW();

  if (status != HAL_OK)
  {
    return status;
  }

  LL_RTC_WAKEUP_Config(wakeup_timer_auto_reload, wakeup_timer_auto_clear);

  return HAL_OK;
}

/**
  * @brief  Retrieve the wake-up timer auto-reload value.
  * @retval uint32_t Auto-reload value
  */
uint32_t HAL_RTC_WAKEUP_GetAutoReload(void)
{
  return LL_RTC_WAKEUP_GetAutoReload();
}

/**
  * @brief  Retrieve wake-up timer auto-reload output clear value.
  * @retval uint32_t Auto-reload output clear value.
  */
uint32_t HAL_RTC_WAKEUP_GetAutoClear(void)
{
  return LL_RTC_WAKEUP_GetAutoClear();
}

/**
  * @}
  */
/** @addtogroup RTC_Exported_Functions_IRQ
  * @brief IRQ handler exported functions.
  * @{
  *
  * IRQ handler functions to manage the different interrupts:
  * * Alarm A and alarm B
  * * Timestamps
  * * Wake-up timer
  * * Subsecond register underflow in binary mode.
  */

/**
  * @brief Handle RTC interrupt request.
  */
void HAL_RTC_IRQHandler(void)
{
  uint32_t flags = LL_RTC_READ_REG(SR);

  if (LL_RTC_ALARM_A_GET_FLAG(flags) != 0U)
  {
    LL_RTC_ClearFlag_ALRA();
    HAL_RTC_AlarmAEventCallback();
  }

  if (LL_RTC_ALARM_B_GET_FLAG(flags) != 0U)
  {
    LL_RTC_ClearFlag_ALRB();
    HAL_RTC_AlarmBEventCallback();
  }

  if (LL_RTC_WAKEUP_GET_FLAG(flags) != 0U)
  {
    LL_RTC_ClearFlag_WUT();
    HAL_RTC_WakeUpTimerEventCallback();
  }

  if (LL_RTC_SSRU_GET_FLAG(flags) != 0U)
  {
    LL_RTC_ClearFlag_SSRU();
    HAL_RTC_SubSecondsUnderflowEventCallback();
  }

  if (LL_RTC_TIMESTAMP_GET_FLAG(flags) != 0U)
  {
    HAL_RTC_TimestampEventCallback();
  }
}

/**
  * @brief Handle alarm interrupt request.
  */
void HAL_RTC_ALARM_IRQHandler(void)
{
  uint32_t flags = LL_RTC_READ_REG(SR);

  if (LL_RTC_ALARM_A_GET_FLAG(flags) != 0U)
  {
    /* Clear the alarm A interrupt pending bit */
    LL_RTC_ClearFlag_ALRA();
    HAL_RTC_AlarmAEventCallback();

  }
  if ((LL_RTC_ALARM_B_GET_FLAG(flags)) != 0U)
  {
    /* Clear the alarm B interrupt pending bit */
    LL_RTC_ClearFlag_ALRB();
    HAL_RTC_AlarmBEventCallback();
  }
}

/**
  * @brief Handle timestamp request.
  * @note  Inside the callback, call @ref HAL_RTC_TIMESTAMP_GetDateTime
  *        to clear the different timestamp flags (TSF, TSOVF).
  */
void HAL_RTC_TIMESTAMP_IRQHandler(void)
{
  if (LL_RTC_IsActiveFlag_TS() != 0U)
  {
    HAL_RTC_TimestampEventCallback();
  }
}

/**
  * @brief Handle wake-up timer interrupt request.
  */
void HAL_RTC_WAKEUP_IRQHandler(void)
{
  if (LL_RTC_IsActiveFlag_WUT() != 0U)
  {
    LL_RTC_ClearFlag_WUT();
    HAL_RTC_WakeUpTimerEventCallback();
  }
}

/**
  * @brief Handle subsecond register underflow interrupt request.
  */
void HAL_RTC_SubSecondsUnderflow_IRQHandler(void)
{
  /* Get the pending status of the SSR Underflow Interrupt */
  if (LL_RTC_IsActiveFlag_SSRU() != 0U)
  {
    /* Immediately clear SSR underflow flag */
    LL_RTC_ClearFlag_SSRU();

    /* SSRU callback */
    HAL_RTC_SubSecondsUnderflowEventCallback();
  }
}

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Callback
  * @brief Exported callback functions.
  * @{
  *  Callback functions that can be overwritten for the different interrupts:
  * * Alarm A
  * * Alarm B
  * * Wake-up timer
  * * Timestamp
  * * Subsecond register underflow only in binary mode.
  *
  * When rewriting the timestamp callback function, @ref HAL_RTC_TimestampEventCallback,
  * call @ref HAL_RTC_TIMESTAMP_GetDateTime or @ref HAL_RTC_TIMESTAMP_GetBinaryTime to clear
  * the timestamp flags (TSF, TSOVF).
  */

/**
  * @brief Alarm A callback.
  */
__WEAK void HAL_RTC_AlarmAEventCallback(void)
{
  /* NOTE: This function must not be modified in this file. When the callback is needed,
   HAL_RTC_AlarmAEventCallback() can be implemented in the application file. */
}

/**
  * @brief Alarm B callback.
  */
__WEAK void HAL_RTC_AlarmBEventCallback(void)
{
  /* NOTE: This function must not be modified in this file. When the callback is needed,
   HAL_RTC_AlarmBEventCallback() can be implemented in the application file. */
}

/**
  * @brief Timestamp callback.
  * @note  When rewriting this function, call @ref HAL_RTC_TIMESTAMP_GetDateTime or
  *        @ref HAL_RTC_TIMESTAMP_GetBinaryTime to clear the different timestamp flags.
  */
__WEAK void HAL_RTC_TimestampEventCallback(void)
{
  /* NOTE: This function must not be modified in this file. When the callback is needed,
   HAL_RTC_TimestampEventCallback() can be implemented in the application file. */
}

/**
  * @brief Wake-up timer callback.
  */
__WEAK void HAL_RTC_WakeUpTimerEventCallback(void)
{
  /* NOTE: This function must not be modified in this file. When the callback is needed,
   HAL_RTC_WakeUpTimerEventCallback() can be implemented in the application file. */
}

/**
  * @brief SSRU callback.
  */
__WEAK void HAL_RTC_SubSecondsUnderflowEventCallback(void)
{
  /* NOTE: This function must not be modified in this file. When the callback is needed,
   HAL_RTC_SubSecondsUnderflowEventCallback() can be implemented in the application file. */
}
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Attributes
  * @brief This section provides functions for managing security, privilege and lock configurations
  * @{
  * - Call HAL_RTC_SetPrivAttr() to set the privilege attribute of specified RTC item(s).
  * - Call HAL_RTC_GetPrivAttr() to get the privilege attribute state of specified RTC item.
  */

/**
  * @brief  Set RTC item(s) privilege configuration.
  * @param  item This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_RTC_PRIV_ITEM_ALRAPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_ALRBPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_WUTPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_TSPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_CALPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_INITPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_PRIV
  * @param priv_attr This parameter is an element of @ref hal_rtc_priv_attr_t enumeration.
  * @retval HAL_ERROR Non-Privileged write to a privilege-only register.
  * @retval HAL_OK    Privilege has been correctly configured.
  */
hal_status_t HAL_RTC_SetPrivAttr(uint32_t item, hal_rtc_priv_attr_t priv_attr)
{
  ASSERT_DBG_PARAM(IS_RTC_PRIV_SET_ITEM(item));
  ASSERT_DBG_PARAM(IS_RTC_ITEM_PRIV_ATTR(priv_attr));

  if (STM32_IS_PRIVILEGED_EXECUTION() == 0U)
  {
    return HAL_ERROR;
  }

  LL_RTC_SetPrivAttr(item, (uint32_t)priv_attr);

  return HAL_OK;
}

/**
  * @brief  Get RTC item privilege configuration.
  * @param  item This parameter can be one or a combination of the following values:
  *         @arg @ref HAL_RTC_PRIV_ITEM_ALRAPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_ALRBPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_WUTPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_TSPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_CALPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_INITPRIV
  *         @arg @ref HAL_RTC_PRIV_ITEM_PRIV
  * @retval Returned value is an element of @ref hal_rtc_priv_attr_t enumeration.
  */
hal_rtc_priv_attr_t HAL_RTC_GetPrivAttr(uint32_t item)
{
  ASSERT_DBG_PARAM(IS_RTC_PRIV_GET_ITEM(item));

  return (hal_rtc_priv_attr_t)LL_RTC_GetPrivAttr(item);
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup RTC_Private_Functions RTC private functions
  * @brief RTC private functions.
  * @{
  */

/**
  * @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
  *         synchronized with RTC APB clock.
  * @retval HAL_OK
  * @retval HAL_ERROR If resynchronization flag is still unset after RTC timeout duration.
  */
static hal_status_t RTC_WaitSynchro_RS(void)
{
  uint32_t tickstart;

  if (LL_RTC_IsEnabledBypassShadowReg() == 0U)
  {
    if (LL_RTC_IsActiveFlag_INIT() == 0U)
    {
      LL_RTC_ClearFlag_RS();
      tickstart = HAL_GetTick();

      while (LL_RTC_IsActiveFlag_RS() == 0U)
      {
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
          if (LL_RTC_IsActiveFlag_RS() == 0U)
          {
            return HAL_ERROR;
          }
        }
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Wait until there is no more shift operation ongoing.
  * @retval HAL_OK
  * @retval HAL_ERROR If a shift operation is still ongoing after RTC timeout duration.
  */
static hal_status_t RTC_WaitSynchro_SHP(void)
{
  uint32_t tickstart = HAL_GetTick();

  while (LL_RTC_IsActiveFlag_SHP() == 1U)
  {
    if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
    {
      if (LL_RTC_IsActiveFlag_SHP() == 1U)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Wait until there is no more recalibration operation ongoing.
  * @retval HAL_OK
  * @retval HAL_ERROR If a recalibration operation is still ongoing after RTC timeout duration.
  */
static hal_status_t RTC_WaitSynchro_RECALP(void)
{
  uint32_t tickstart = HAL_GetTick();

  while (LL_RTC_IsActiveFlag_RECALP() == 1U)
  {
    if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
    {
      if (LL_RTC_IsActiveFlag_RECALP() == 1U)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Wait until the WUTWF bit of ICSR register is set
  *         after the wake-up timer has been disabled.
  * @retval HAL_OK
  * @retval HAL_ERROR If WUTWF is still unset after RTC timeout duration.
  */
static hal_status_t RTC_WaitSynchro_WUTW(void)
{
  uint32_t tickstart;

  if (LL_RTC_IsActiveFlag_INIT() == 0U)
  {
    if (LL_RTC_WAKEUP_IsEnabled() == 1U)
    {
      return HAL_ERROR;
    }
    tickstart = HAL_GetTick();

    while (LL_RTC_IsActiveFlag_WUTW() == 0U)
    {
      if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
      {
        if (LL_RTC_IsActiveFlag_WUTW() == 0U)
        {
          return HAL_ERROR;
        }
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Retrieve the RTC current time.
  * @param  p_time Pointer to time instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When a timeout occurs on the shadow register flag.
  */
__STATIC_INLINE hal_status_t RTC_GetTime(hal_rtc_time_t *p_time)
{
  hal_status_t status;
  status = RTC_WaitSynchro_RS();

  if (status != HAL_OK)
  {
    return status;
  }

  uint32_t temp_time_format = LL_RTC_TIME_GetTimeAndFormat();
  uint32_t format_am_pm = (((uint32_t) HAL_RTC_TIME_FORMAT_PM) * LL_RTC_GET_FORMAT(temp_time_format));

  p_time->am_pm = (hal_rtc_time_format_am_pm_t) format_am_pm;

  p_time->hour = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_HOUR(temp_time_format));
  p_time->min = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_MINUTE(temp_time_format));
  p_time->sec =  HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_SECOND(temp_time_format));

  p_time->subsec = LL_RTC_TIME_GetSubSecond();

  return status;
}

/**
  * @brief  Retrieve the RTC current date.
  * @param  p_date Pointer to date instance.
  * @retval HAL_OK
  * @retval HAL_ERROR When a timeout occurs on the shadow register flag.
  */
__STATIC_INLINE hal_status_t RTC_GetDate(hal_rtc_date_t *p_date)
{
  uint32_t temp_value_date = LL_RTC_DATE_Get();
  uint32_t week_day = LL_RTC_GET_WEEKDAY(temp_value_date);
  uint32_t month = LL_RTC_GET_MONTH(temp_value_date);

  /* No need to use HAL_RTC_CONVERT_BCD2DEC for wday because the max value is less than 10; BCD equals decimal */
  p_date->wday = (hal_rtc_weekday_t) week_day;
  p_date->mday = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_DAY(temp_value_date));
  p_date->mon = (hal_rtc_month_t) month;
  p_date->year = HAL_RTC_CONVERT_BCD2DEC(LL_RTC_GET_YEAR(temp_value_date));

  return RTC_WaitSynchro_RS();
}

/**
  * @brief  Retrieve the value of the wake-up timer decrement frequency.
  * @param  clock_prescaler_wakeup Wake-up clock prescaler.
  * @retval Wake-up timer decrement frequency.
  */
__STATIC_INLINE uint32_t RTC_GetWakeUpClockFrequency(hal_rtc_wakeup_timer_clock_t clock_prescaler_wakeup)
{
  uint32_t frequency_wakeup;
  uint32_t prescaler_value;
  uint32_t frequency_rtcclk;

  /* RCC gives the value of the RTCCLK used; it can be HSE (with prescaler), LSE, or LSI (with prescaler) */
  frequency_rtcclk = RTC_GetRTCClockCalibrated();

  if (clock_prescaler_wakeup >= HAL_RTC_WAKEUP_TIMER_CLOCK_BCD_UPDATE)
  {
    frequency_wakeup = RTC_GetRTCClockAfterPrescalerS(frequency_rtcclk);
  }
  else
  {
    /* Values of the wake-up prescaler are 2, 4, 8, and 16 and are directly on RTCCLK */
    prescaler_value = 16UL >> ((uint32_t) clock_prescaler_wakeup);
    frequency_wakeup = frequency_rtcclk / prescaler_value;
  }

  return frequency_wakeup;
}

/**
  * @brief  Retrieve the RTCCLK frequency after calibration.
  * @return Value of the RTCCLK frequency after calibration.
  */
__STATIC_INLINE uint32_t RTC_GetRTCClockCalibrated(void)
{
  /* Check if low power mode is needed */
  uint32_t frequency_rtcclk;
  uint32_t calib_m;
  uint32_t calib_p;

  frequency_rtcclk = HAL_RCC_RTC_GetKernelClkFreq();

  calib_m = LL_RTC_CAL_GetMinus();
  calib_p = LL_RTC_CAL_IsPulseInserted();

  /*Corrected frequency depending on the calibration */
  frequency_rtcclk = frequency_rtcclk * (1U + (((calib_p * RTC_TIMEOUT_VALUE) - calib_m) /
                                               ((0x1UL << 20U) + calib_m - (calib_p * RTC_TIMEOUT_VALUE))));

  return frequency_rtcclk;
}

/**
  * @brief  Retrieve the RTCCLK frequency after the synchronous prescaler (second one of RTC).
  * @param  frequency input frequency of the RTCCLK
  * @return Value of the frequency after the synchronous prescaler
  */
__STATIC_INLINE uint32_t RTC_GetRTCClockAfterPrescalerS(uint32_t frequency)
{
  return frequency / ((1U + LL_RTC_GetAsynchPrescaler()) * (1U + LL_RTC_GetSynchPrescaler()));
}

/**
  * @brief  Convert seconds and microseconds into bits representing time depending on a frequency.
  * @param  seconds number of seconds.
  * @param  milliseconds number of milliseconds.
  * @param  microseconds number of microseconds.
  * @param  frequency frequency used to convert.
  * @retval Bits representing time depending on a frequency.
  */
__STATIC_INLINE uint32_t RTC_ConvertSecSubsecToBits(uint32_t seconds, uint32_t milliseconds,
                                                    uint32_t microseconds, uint32_t frequency)
{
  const uint32_t freq_per_millisecond = frequency / RTC_MILLISECONDS;
  const uint32_t freq_per_microsecond = frequency / RTC_MICROSECONDS;

  return (uint32_t)((seconds * frequency) + (milliseconds * freq_per_millisecond) +
                    (microseconds * freq_per_microsecond));

}

/**
  * @brief Convert time bits values to time values (hours, minutes, seconds and microseconds).
  * @param p_time pointer to a time instance.
  * @param bits value of the time in bits.
  * @param frequency frequency used to convert.
  */
__STATIC_INLINE void RTC_ConvertBitsToTime(hal_rtc_time_t *p_time, uint32_t bits, uint32_t frequency)
{
  const uint32_t microseconds_per_bit = RTC_MICROSECONDS / frequency;

  uint32_t total_seconds = bits / frequency;
  uint32_t remaining_bits = bits % frequency;
  uint32_t total_microseconds = remaining_bits * microseconds_per_bit;

  p_time->millisec = total_microseconds / RTC_MILLISECONDS;
  p_time->microsec = total_microseconds % RTC_MILLISECONDS;
  p_time->hour = total_seconds / RTC_HOUR_SECONDS;
  total_seconds %= RTC_HOUR_SECONDS;

  p_time->min = total_seconds / RTC_MIN_SECONDS;
  p_time->sec = total_seconds % RTC_MIN_SECONDS;
}

/**
  * @}
  */

#endif /* USE_HAL_RTC_MODULE */

/**
  * @}
  */

/**
  * @}
  */
