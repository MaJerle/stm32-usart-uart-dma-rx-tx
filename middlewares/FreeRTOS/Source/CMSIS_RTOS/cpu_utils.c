/**
  ******************************************************************************
  * @file    cpu_utils.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-November-2014
  * @brief   Utilities for CPU Load calculation
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

xTaskHandle    xIdleHandle = NULL;
volatile uint32_t  osCPU_Usage = 0; 
uint32_t       osCPU_IdleStartTime = 0; 
uint32_t       osCPU_IdleSpentTime = 0; 
uint32_t       osCPU_TotalIdleTime = 0; 

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Application Idle Hook
  * @param  None 
  * @retval None
  */
void vApplicationIdleHook(void) 
{
  if( xIdleHandle == NULL )
  {
    /* Store the handle to the idle task. */
    xIdleHandle = xTaskGetCurrentTaskHandle();
  }
}

/**
  * @brief  Application Idle Hook
  * @param  None 
  * @retval None
  */
void vApplicationTickHook (void)
{
  static int tick = 0;
  
  if(tick ++ > CALCULATION_PERIOD)
  {
    tick = 0;
    
    if(osCPU_TotalIdleTime > 1000)
    {
      osCPU_TotalIdleTime = 1000;
    }
    osCPU_Usage = (100 - (osCPU_TotalIdleTime * 100) / CALCULATION_PERIOD);
    osCPU_TotalIdleTime = 0;
  }
}

/**
  * @brief  Start Idle monitor
  * @param  None 
  * @retval None
  */
void StartIdleMonitor (void)
{
  if( xTaskGetCurrentTaskHandle() == xIdleHandle ) 
  {
    osCPU_IdleStartTime = xTaskGetTickCountFromISR();
  }
}

/**
  * @brief  Stop Idle monitor
  * @param  None 
  * @retval None
  */
void EndIdleMonitor (void)
{
  if( xTaskGetCurrentTaskHandle() == xIdleHandle )
  {
    /* Store the handle to the idle task. */
    osCPU_IdleSpentTime = xTaskGetTickCountFromISR() - osCPU_IdleStartTime;
    osCPU_TotalIdleTime += osCPU_IdleSpentTime; 
  }
}

/**
  * @brief  Stop Idle monitor
  * @param  None 
  * @retval None
  */
uint16_t osGetCPUUsage (void)
{
  return (uint16_t)osCPU_Usage;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
