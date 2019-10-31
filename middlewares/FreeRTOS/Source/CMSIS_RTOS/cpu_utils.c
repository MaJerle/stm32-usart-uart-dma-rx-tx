/********************** NOTES **********************************************
To use this module, the following steps should be followed :

1- in the _OS_Config.h file (ex. FreeRTOSConfig.h) enable the following macros : 
      - #define configUSE_IDLE_HOOK        1
      - #define configUSE_TICK_HOOK        1

2- in the _OS_Config.h define the following macros : 
      - #define traceTASK_SWITCHED_IN()  extern void StartIdleMonitor(void); \
                                         StartIdleMonitor()
      - #define traceTASK_SWITCHED_OUT() extern void EndIdleMonitor(void); \
                                         EndIdleMonitor()
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private variables */
static osThreadId_t        idle_thread_id = NULL;
static volatile uint32_t   cpu_usage = 0;
static uint32_t            idle_start_time = 0;
static uint32_t            idle_total_time = 0;

/**
 * \brief           Application Idle Hook
 *
 * This function is called by FreeRTOS idle hook.
 * \ref configUSE_IDLE_HOOK must be enabled for this feature
 *
 * \note            Function name is fixed and cannot be changed
 */
void
vApplicationIdleHook(void) {
    /* Get idle thread ID */
    if (idle_thread_id == NULL) {
        idle_thread_id = (osThreadId_t)xTaskGetCurrentTaskHandle();
    }
}

/**
 * \brief           Application Tick Hook
 *
 * This function is called by FreeRTOS tick hook, every tick interrupt.
 * \ref configUSE_TICK_HOOK must be enabled for this feature
 *
 * \note            Function name is fixed and cannot be changed
 */
void
vApplicationTickHook(void) {
    static int tick = 0;
    
    /* We calculate usage only once per period */
    if (tick > CALCULATION_PERIOD) {
        tick = 0;

        if (idle_total_time > 1000) {
            idle_total_time = 1000;
        }
        cpu_usage = (100 - (idle_total_time * 100) / CALCULATION_PERIOD);
        idle_total_time = 0;
    } else {
        ++tick;
    }
}

/**
 * \brief           Start monitoring idle time
 *
 * This function is called when FreeRTOS thread is switched in.
 * It is configured using \ref traceTASK_SWITCHED_IN macro.
 */
void
StartIdleMonitor(void) {
    /* React only on idle thread */
    if (idle_thread_id == (osThreadId_t)xTaskGetCurrentTaskHandle()) {
        idle_start_time = xTaskGetTickCountFromISR();
    }
}

/**
 * \brief           Stop monitoring idle time
 *
 * This function is called when FreeRTOS thread is switched out.
 * It is configured using \ref traceTASK_SWITCHED_OUT macro.
 */
void
EndIdleMonitor(void) {
    /* React only on idle thread */
    if (idle_thread_id == (osThreadId_t)xTaskGetCurrentTaskHandle()) {
        idle_total_time += xTaskGetTickCountFromISR() - idle_start_time;
    }
}

/**
 * \brief  Stop Idle monitor
 * \param  None
 * \retval None
 */
uint16_t
osGetCPUUsage(void) {
    return (uint16_t)cpu_usage;
}

