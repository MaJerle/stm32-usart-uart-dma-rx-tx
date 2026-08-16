/**
  ******************************************************************************
  * @file    startup_stm32c542xx.c
  * @brief   Startup File for STM32C542xx Devices
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

/* Includes ------------------------------------------------------------------*/
#include "stm32c542xx.h"

#ifdef __cplusplus
#error "startup_stm32c542xx.c file cannot be compiled with C++ compiler"
#endif /* __cplusplus */

/* External References -------------------------------------------------------*/
extern void SystemInit(void);
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/* Private function prototypes -----------------------------------------------*/
/* ISR headers */
__NO_RETURN void Reset_Handler(void) __attribute__((weak));
            void Default_IRQHandler(void);
__NO_RETURN void Default_IRQHandler_Hook(void) __attribute__((weak));

/* Cortex-M interrupts */
void NMI_Handler                      (void) __attribute__((weak, alias("Default_IRQHandler")));
void HardFault_Handler                (void) __attribute__((weak));
void MemManage_Handler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void BusFault_Handler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void UsageFault_Handler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void SVC_Handler                      (void) __attribute__((weak, alias("Default_IRQHandler")));
void DebugMon_Handler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void PendSV_Handler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void SysTick_Handler                  (void) __attribute__((weak, alias("Default_IRQHandler")));

/* Device interrupts */
void WWDG_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void PWR_PVD_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void RTC_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void TAMP_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void RAMCFG_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void FLASH_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void RCC_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI0_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI1_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI2_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI3_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI4_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI5_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI6_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI7_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI8_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI9_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI10_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI11_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI12_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI13_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI14_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void EXTI15_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA1_CH0_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA1_CH1_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA1_CH2_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA1_CH3_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void IWDG_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void ADC1_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void FDCAN1_IT0_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void FDCAN1_IT1_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM1_BRK_TERR_IERR_IRQHandler    (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM1_UPD_IRQHandler              (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM1_TRGI_COM_DIR_IDX_IRQHandler (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM1_CC_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM2_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM6_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM7_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void I2C1_EV_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void I2C1_ERR_IRQHandler              (void) __attribute__((weak, alias("Default_IRQHandler")));
void I3C1_EV_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void I3C1_ERR_IRQHandler              (void) __attribute__((weak, alias("Default_IRQHandler")));
void SPI1_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void SPI2_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void USART1_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void USART2_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void UART4_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void UART5_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPUART1_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPTIM1_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM12_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM15_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void USB_DRD_FS_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void CRS_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void RNG_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void FPU_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void ICACHE_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void CORDIC_IRQHandler                (void) __attribute__((weak, alias("Default_IRQHandler")));
void AES_IRQHandler                   (void) __attribute__((weak, alias("Default_IRQHandler")));
void HASH_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM8_BRK_TERR_IERR_IRQHandler    (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM8_UPD_IRQHandler              (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM8_TRGI_COM_DIR_IDX_IRQHandler (void) __attribute__((weak, alias("Default_IRQHandler")));
void TIM8_CC_IRQHandler               (void) __attribute__((weak, alias("Default_IRQHandler")));
void COMP1_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));
void DAC1_IRQHandler                  (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA2_CH0_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA2_CH1_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA2_CH2_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void LPDMA2_CH3_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void FDCAN2_IT0_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void FDCAN2_IT1_IRQHandler            (void) __attribute__((weak, alias("Default_IRQHandler")));
void COMP2_IRQHandler                 (void) __attribute__((weak, alias("Default_IRQHandler")));

/* Private variables ---------------------------------------------------------*/
/**
  Vector table
  */

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif /* __GNUC__ */

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
  (VECTOR_TABLE_Type)(&__INITIAL_SP), /* The initial stack pointer */
  Reset_Handler,
  NMI_Handler,
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler,
  UsageFault_Handler,
  0,
  0,
  0,
  0,
  SVC_Handler,
  DebugMon_Handler,
  0,
  PendSV_Handler,
  SysTick_Handler,
  WWDG_IRQHandler,
  PWR_PVD_IRQHandler,
  RTC_IRQHandler,
  TAMP_IRQHandler,
  RAMCFG_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  EXTI5_IRQHandler,
  EXTI6_IRQHandler,
  EXTI7_IRQHandler,
  EXTI8_IRQHandler,
  EXTI9_IRQHandler,
  EXTI10_IRQHandler,
  EXTI11_IRQHandler,
  EXTI12_IRQHandler,
  EXTI13_IRQHandler,
  EXTI14_IRQHandler,
  EXTI15_IRQHandler,
  LPDMA1_CH0_IRQHandler,
  LPDMA1_CH1_IRQHandler,
  LPDMA1_CH2_IRQHandler,
  LPDMA1_CH3_IRQHandler,
  0,
  0,
  0,
  0,
  IWDG_IRQHandler,
  ADC1_IRQHandler,
  0,
  FDCAN1_IT0_IRQHandler,
  FDCAN1_IT1_IRQHandler,
  TIM1_BRK_TERR_IERR_IRQHandler,
  TIM1_UPD_IRQHandler,
  TIM1_TRGI_COM_DIR_IDX_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  0,
  TIM6_IRQHandler,
  TIM7_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ERR_IRQHandler,
  I3C1_EV_IRQHandler,
  I3C1_ERR_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  0,
  USART1_IRQHandler,
  USART2_IRQHandler,
  0,
  UART4_IRQHandler,
  UART5_IRQHandler,
  LPUART1_IRQHandler,
  LPTIM1_IRQHandler,
  TIM12_IRQHandler,
  TIM15_IRQHandler,
  0,
  0,
  USB_DRD_FS_IRQHandler,
  CRS_IRQHandler,
  RNG_IRQHandler,
  FPU_IRQHandler,
  ICACHE_IRQHandler,
  CORDIC_IRQHandler,
  AES_IRQHandler,
  HASH_IRQHandler,
  0,
  0,
  TIM8_BRK_TERR_IERR_IRQHandler,
  TIM8_UPD_IRQHandler,
  TIM8_TRGI_COM_DIR_IDX_IRQHandler,
  TIM8_CC_IRQHandler,
  COMP1_IRQHandler,
  DAC1_IRQHandler,
  LPDMA2_CH0_IRQHandler,
  LPDMA2_CH1_IRQHandler,
  LPDMA2_CH2_IRQHandler,
  LPDMA2_CH3_IRQHandler,
  0,
  0,
  0,
  0,
  FDCAN2_IT0_IRQHandler,
  FDCAN2_IT1_IRQHandler,
  COMP2_IRQHandler
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is the Reset Handler called on controller reset.
  * @param  None
  * @retval None
  */
__WEAK __NO_RETURN void Reset_Handler(void)
{
  __set_PSP((uint32_t)(&__INITIAL_SP));

  __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
  __set_PSPLIM((uint32_t)(&__STACK_LIMIT));

  SystemInit();         /* CMSIS System Initialization */
  __PROGRAM_START();    /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif /* __ARMCC_VERSION */

/**
  * @brief  Hard Fault Handler.
  * @param  None
  * @retval None
  */
__WEAK void HardFault_Handler(void)
{
  while (1);
}

/**
  * @brief  This function is the default IRQ handler
  *         when the IRQ line is not used by the application.
  * @param  None
  * @retval None
  */
void Default_IRQHandler(void)
{
  Default_IRQHandler_Hook();
}

/**
  * @brief  This function is the default IRQHandler implementation hook
  *         could be overridden by the application.
  * @param  None
  * @retval None
  */
__WEAK void Default_IRQHandler_Hook(void)
{
  while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#endif /* __ARMCC_VERSION */
