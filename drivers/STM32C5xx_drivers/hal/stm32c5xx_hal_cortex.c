/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_cortex.c
  * @brief   CORTEX HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the CORTEX:
  *           + Initialization and configuration functions
  *           + Peripheral control functions
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
  **********************************************************************************************************************
  */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(USE_HAL_CORTEX_MODULE) && (USE_HAL_CORTEX_MODULE == 1U)

/** @addtogroup CORTEX
  * @{
  */
/** @defgroup CORTEX_Introduction CORTEX Introduction
  * @{

  The CORTEX hardware abstraction layer provides a comprehensive set of APIs to interface with the Cortex core
  peripherals on STM32 microcontrollers. It enhances system control, interrupt management, and fault handling by
  offering flexible and efficient configuration of key core system blocks tailored to the ARMv8-M architecture.

  It simplifies the configuration, initialization, and management of the Cortex core components, including the Nested
  Vector Interrupt Controller (NVIC), System Timer (SYSTICK), Memory Protection Unit (MPU), and System Control Block
  (SCB). Features supported include interrupt priority grouping, fault exception control, timer configuration, memory
  region protection, interrupt handling, and system information retrieval.

  This abstraction layer guarantees portability and ease of use across different STM32 series.
  */
/**
  * @}
  */

/** @defgroup CORTEX_How_To_Use CORTEX How To Use
  * @{

# CORTEX main features

## The HAL CORTEX driver contains four main blocks:

1. NVIC: Nested Vector Interrupt Controller

  - The NVIC is an embedded interrupt controller that supports low-latency interrupt processing.
    - It contains a configurable interrupt handling ability. Configured items can be:
        - Priority grouping that specifies the range of preemption priority and sub priority.
        - Preemption priority ability between interrupts.
        - Subpriority ability between interrupts.

2. SYSTICK: System Timer

  - The SysTick is a 24-bit countdown timer. It can be used as a simple counter or as a tick timer in a real-time
    operating system (RTOS).

3. MPU: Memory Protection Unit

  - The MPU allows privileged software to define memory regions, assign memory access permissions and memory
    attributes to each of them to improve the system reliability.
  - The MPU in this device supports 8 regions.

4. SCB: System Control Block

  - The SCB provides system information and system control that includes configuration, control and reporting of
    system fault exceptions.

# How to use the CORTEX HAL module driver

## The CORTEX HAL driver can be used as follows:

This driver provides HAL_CORTEX driver functions to configure the NVIC, SYSTICK, MPU, and SCB blocks.

1. How to configure NVIC interrupts using the CORTEX HAL driver:

  - Configure the NVIC priority grouping using the HAL_CORTEX_NVIC_SetPriorityGrouping() function once at startup.
     - When the HAL_CORTEX_NVIC_PRIORITY_GROUP_0 is selected, IRQ preemption is no longer configurable. The pending IRQ
       priority is managed only by the sub-priority.
     - When the HAL_CORTEX_NVIC_PRIORITY_GROUP_1 is selected, there is one bit for preemption priority and three bits
       for sub-priority.
     - When the HAL_CORTEX_NVIC_PRIORITY_GROUP_2 is selected, there are two bits for preemption priority and two bits
       for sub-priority.
     - When the HAL_CORTEX_NVIC_PRIORITY_GROUP_3 is selected, there are three bits for preemption priority and one bit
       for sub-priority.
     - When the HAL_CORTEX_NVIC_PRIORITY_GROUP_4 is selected, IRQ sub-priority is no longer configurable. The pending
     IRQ priority is managed only by the preemption priority.

  - Configure the priority of the selected IRQ channels using HAL_CORTEX_NVIC_SetPriority() function :
     - IRQ priority order (sorted by highest to lowest priority):
        - The lowest is the preemption priority numerical value, the highest is the preemption priority.
        - The lowest is the subpriority numerical value, the highest is the subpriority.
     - Get the priority grouping using HAL_CORTEX_NVIC_GetPriorityGrouping() function.
     - Get the priority of an interrupt using HAL_CORTEX_NVIC_GetPriority() function.

  - Enable the selected IRQ channels using HAL_CORTEX_NVIC_EnableIRQ() function.

  - Disable the selected IRQ channels using HAL_CORTEX_NVIC_DisableIRQ() function.

  - To check if an IRQ channel is enabled or not, use HAL_CORTEX_NVIC_IsEnabledIRQ() function.

  - To check if an IRQ channel is active or not, use HAL_CORTEX_NVIC_IsActiveIRQ() function.

  - To set pending bit of an interrupt, use HAL_CORTEX_NVIC_SetPendingIRQ() function.

  - To check if the IRQn channel is in pending state or not, use HAL_CORTEX_NVIC_IsPendingIRQ() function. When pending,
    use HAL_CORTEX_NVIC_ClearPendingIRQ() to clear the event.

  - When a system reset is needed within the application, use HAL_CORTEX_NVIC_SystemReset() function.

2. How to configure SYSTICK using the CORTEX HAL driver:

  - Configure the SYSTICK notification frequency and its source clock using HAL_CORTEX_SYSTICK_SetFreq()
    and HAL_CORTEX_SYSTICK_SetClkSource() functions.

  - To suspend the SYSTICK, use the HAL_CORTEX_SYSTICK_Suspend() function. When suspending, use
    HAL_CORTEX_SYSTICK_Resume() to resume the SYSTICK.

  - To handle the SYSTICK interrupts, use HAL_CORTEX_SYSTICK_IRQHandler() function.

3. How to configure MPU using the CORTEX HAL driver:

  - To configure a device memory attribute, use HAL_CORTEX_MPU_SetDeviceMemAttr() and, to configure normal
    memory (cache memory), use HAL_CORTEX_MPU_SetCacheMemAttr().

  - To get the device memory attributes configuration, use HAL_CORTEX_MPU_GetDeviceMemAttr().

  - To get the cache memory attributes configuration, use HAL_CORTEX_MPU_GetCacheMemAttr().

  - To configure an MPU region, use HAL_CORTEX_MPU_SetConfigRegion().

  - To get the MPU region configuration, use HAL_CORTEX_MPU_GetConfigRegion().

  - To enable or disable an MPU region, use HAL_CORTEX_MPU_EnableRegion() or HAL_CORTEX_MPU_DisableRegion().

  - To enable or disable the MPU, use HAL_CORTEX_MPU_Enable() or HAL_CORTEX_MPU_Disable().

  - To check if the MPU is enabled or not, use HAL_CORTEX_MPU_IsEnabled().

  - To check if the given MPU region is enabled or not, use HAL_CORTEX_MPU_IsEnabledRegion().

4. How to configure SCB using the CORTEX HAL driver:

  - When you need to get the CPU ID information within the application, use HAL_CORTEX_SCB_GetInfo().

  - Some exceptions can be redirected to their own IRQ channels or to the HARDFAULT IRQ channel. These exceptions are:
     - USAGE FAULT
     - BUS FAULT
     - MEMORY MANAGEMENT FAULT

  - When you need to redirect any exception to a HardFault, use HAL_CORTEX_SCB_EnableHardFaultEscalation().

  - When you need to disable any HardFault redirection, use HAL_CORTEX_SCB_DisableHardFaultEscalation().
  */
/**
  * @}
  */

/** @defgroup CORTEX_Configuration_Table CORTEX Configuration Table
  * @{
## Configuration inside the CORTEX driver

Config defines                    | Description      | Default value | Note
--------------------------------- | ---------------- | ------------- | ------------------------------------------
PRODUCT                           | from IDE         |     NA        | The selected product.
USE_ASSERT_DBG_PARAM              | from IDE         |     None      | Allows to use the assert check parameters.
USE_HAL_CHECK_PARAM               | from hal_conf.h  |     0         | Allows to use the run-time checks parameters.
USE_HAL_CORTEX_MODULE             | from hal_conf.h  |     1         | Allows to use HAL CORTEX module.
  */
/**
  * @}
  */

/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
/* Private constants -------------------------------------------------------------------------------------------------*/
/** @defgroup CORTEX_Private_Constants CORTEX Private Constants
  * @{
  */
#define CORTEX_DEVICE_MASK      (0x0000000CU) /*!< Device memory mask          */
#define CORTEX_NORMAL_MASK      (0x000000F0U) /*!< Normal memory mask          */
#define CORTEX_ATTR_OUTER_MASK  (0x000000F0U) /*!< Outer attribute mask        */
#define CORTEX_ATTR_INNER_MASK  (0x0000000FU) /*!< Inner attribute mask        */
#define CORTEX_ATTR_REG_NUM     (0x00000004U) /*!< Attribute register number   */
#define CORTEX_ATTR_BITS_NUM    (0x00000008U) /*!< Attribute bits number       */
#define CORTEX_REGION_ADDR_MASK (0xFFFFFFE0U) /*!< Base and limit address mask */
/**
  * @}
  */

/* Private macros ----------------------------------------------------------------------------------------------------*/
/** @defgroup CORTEX_Private_Macros CORTEX Private Macros
  * @{
  */
/*! Macro to identify irq number (maximum value depends on the irq numbers on the device) */
#if defined(ADC3)
#define IS_IRQ_NUMBER(irq_number) ((irq_number) <= ADC3_IRQn)
#elif defined(COMP2)
#define IS_IRQ_NUMBER(irq_number) ((irq_number) <= COMP2_IRQn)
#else
#define IS_IRQ_NUMBER(irq_number) ((irq_number) <= LPDMA2_CH3_IRQn)
#endif /* PLAY1 */

/*! Macro to identify priority grouping */
#define IS_PRIORITY_GROUP(prio_grp) (((prio_grp) == HAL_CORTEX_NVIC_PRIORITY_GROUP_0)    \
                                     || ((prio_grp) == HAL_CORTEX_NVIC_PRIORITY_GROUP_1) \
                                     || ((prio_grp) == HAL_CORTEX_NVIC_PRIORITY_GROUP_2) \
                                     || ((prio_grp) == HAL_CORTEX_NVIC_PRIORITY_GROUP_3) \
                                     || ((prio_grp) == HAL_CORTEX_NVIC_PRIORITY_GROUP_4))

/*! Macro to identify preemption priority according the given priority group */
#define IS_PREEMP_PRIORITY(prio_grp, preemp_prio) (                                 \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_0)) ? (((preemp_prio)) == 0U): \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_1)) ? (((preemp_prio)) <= 1U): \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_2)) ? (((preemp_prio)) <= 3U): \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_3)) ? (((preemp_prio)) <= 7U): \
    (((preemp_prio) <= 15U)))

/*! Macro to identify sub-priority according the given priority group */
#define IS_SUB_PRIORITY(prio_grp, sub_prio) (                                      \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_0)) ? (((sub_prio) <= 15U)) : \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_1)) ? (((sub_prio) <= 7U))  : \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_2)) ? (((sub_prio) <= 3U))  : \
    (((prio_grp)) == (HAL_CORTEX_NVIC_PRIORITY_GROUP_3)) ? (((sub_prio) <= 1U))  : \
    (((sub_prio) == 0U)))

/*! Macro to identify the clock source */
#define IS_CLOCK_SOURCE(clk_src) (((clk_src) == HAL_CORTEX_SYSTICK_CLKSOURCE_EXTERNAL) \
                                  || ((clk_src) == HAL_CORTEX_SYSTICK_CLKSOURCE_INTERNAL))

/*! Macro to identify the HardFault NMI state */
#define IS_NMI_STATE(fault_nmi) (((fault_nmi) == HAL_CORTEX_MPU_HARDFAULT_NMI_DISABLE)  \
                                 || ((fault_nmi) == HAL_CORTEX_MPU_HARDFAULT_NMI_ENABLE))

/*! Macro to identify the Access Privilege */
#define IS_ACCESS_PRIV(access_priv) (((access_priv) == HAL_CORTEX_MPU_ACCESS_FAULT_ALL)          \
                                     || ((access_priv) == HAL_CORTEX_MPU_ACCESS_FAULT_ONLY_PRIV))

/*! Macro to identify device memory attribute */
#define IS_DEVICE_MEM_ATTR(device_attr) (((device_attr) == HAL_CORTEX_MPU_DEVICE_MEM_NGNRNE)    \
                                         || ((device_attr) == HAL_CORTEX_MPU_DEVICE_MEM_NGNRE)  \
                                         || ((device_attr) == HAL_CORTEX_MPU_DEVICE_MEM_NGRE)   \
                                         || ((device_attr) == HAL_CORTEX_MPU_DEVICE_MEM_GRE))

/*! Macro to identify the memory attribute */
#define IS_NORMAL_MEM_ATTR(mem_attr) (((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_NCACHEABLE)               \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WT_NOA)                \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WT_WA)                 \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WT_RA)                 \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WT_RWA)                \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WB_NOA)                \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WB_WA)                 \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WB_RA)                 \
                                      || ((mem_attr) == HAL_CORTEX_MPU_NORMAL_MEM_WB_RWA))

/*! Macro to identify the memory attribute index */
#define IS_MEM_ATTR_IDX(mem_attr_idx) (((mem_attr_idx)   == HAL_CORTEX_MPU_MEM_ATTR_0)            \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_1)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_2)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_3)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_4)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_5)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_6)           \
                                       || ((mem_attr_idx) == HAL_CORTEX_MPU_MEM_ATTR_7))

/*! Macro to identify MPU region index */
#define IS_MPU_REGION(mpu_region) (((mpu_region) == HAL_CORTEX_MPU_REGION_0)    \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_1) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_2) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_3) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_4) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_5) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_6) \
                                   || ((mpu_region) == HAL_CORTEX_MPU_REGION_7))

/*! Macro to identify fault exceptions */
#define IS_FAULT_EXCEPT(fault_except)                                              \
  ((((fault_except) & (HAL_CORTEX_SCB_USAGE_FAULT | HAL_CORTEX_SCB_BUS_FAULT  |    \
                       HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT)) != 0x00U)             \
   && (((fault_except) & ~(HAL_CORTEX_SCB_USAGE_FAULT | HAL_CORTEX_SCB_BUS_FAULT | \
                           HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT)) == 0x00U))

/*! Macro to identify region access attribute */
#define IS_ACCESS_ATTR(access_attr) (((access_attr) == HAL_CORTEX_MPU_REGION_ONLY_PRIV_RW)    \
                                     || ((access_attr) == HAL_CORTEX_MPU_REGION_ALL_RW)       \
                                     || ((access_attr) == HAL_CORTEX_MPU_REGION_ONLY_PRIV_RO) \
                                     || ((access_attr) == HAL_CORTEX_MPU_REGION_ALL_RO))

/*! Macro to identify the execution attribute */
#define IS_EXEC_ATTR(exec_attr)      (((exec_attr) == HAL_CORTEX_MPU_EXECUTION_ATTR_DISABLE) \
                                      || ((exec_attr) == HAL_CORTEX_MPU_EXECUTION_ATTR_ENABLE))

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup CORTEX_Exported_Functions
  * @{
  */

/** @addtogroup CORTEX_Exported_Functions_Group1
  * @{
- This subsection provides a set of functions to configure CORTEX NVIC block features.
  - Use HAL_CORTEX_NVIC_SetPriorityGrouping() to set the priority grouping.
  - Use HAL_CORTEX_NVIC_GetPriorityGrouping() to get the priority grouping.
  - Use HAL_CORTEX_NVIC_SetPriority() to set the interrupt preemption priority.
  - Use HAL_CORTEX_NVIC_GetPriority() to get the interrupt preemption priority.
  - Use HAL_CORTEX_NVIC_EnableIRQ() to enable an interrupt.
  - Use HAL_CORTEX_NVIC_DisableIRQ() to disable the interrupt.
  - Use HAL_CORTEX_NVIC_IsEnabledIRQ() to check whether an interrupt is enabled or not.
  - Use HAL_CORTEX_NVIC_IsActiveIRQ() to check whether an interrupt is active or not.
  - Use HAL_CORTEX_NVIC_SetPendingIRQ() to set an interrupt in pending state.
  - Use HAL_CORTEX_NVIC_ClearPendingIRQ() to clear a pending interrupt.
  - Use HAL_CORTEX_NVIC_IsPendingIRQ() to check whether an interrupt is pending or not.
  - Use HAL_CORTEX_NVIC_SystemReset() to perform a system reset.
  */

/**
  * @brief   Set the priority grouping field (preemption priority and subpriority)
  *          using the required unlock sequence.
  * @param   prio_grp: The priority grouping bits length.
  *          This parameter is an element of \ref hal_cortex_nvic_priority_group_t enumeration.
  * @note    When the HAL_CORTEX_NVIC_PRIORITY_GROUP_1 is selected, there is one bit for preemption priority and three
  *          bits for sub-priority.
  * @note    When the HAL_CORTEX_NVIC_PRIORITY_GROUP_2 is selected, there are two bits for preemption priority and two
  *          bits for sub-priority.
  * @note    When the HAL_CORTEX_NVIC_PRIORITY_GROUP_3 is selected, there are three bits for preemption priority and one
  *          bit for sub-priority.
  * @warning When the HAL_CORTEX_NVIC_PRIORITY_GROUP_0 is selected, IRQ preemption is no longer possible.
  *          The pending IRQ priority is managed only by the sub-priority.
  * @warning When the HAL_CORTEX_NVIC_PRIORITY_GROUP_4 is selected, IRQ sub-priority is no longer possible.
  *          The pending IRQ priority is managed only by the preemption.
  */
void HAL_CORTEX_NVIC_SetPriorityGrouping(hal_cortex_nvic_priority_group_t prio_grp)
{
  ASSERT_DBG_PARAM(IS_PRIORITY_GROUP(prio_grp));

  NVIC_SetPriorityGrouping((uint32_t)prio_grp);
}

/**
  * @brief  Get the priority grouping field from the NVIC Interrupt Controller.
  * @retval hal_cortex_nvic_priority_group_t Priority group value.
  */
hal_cortex_nvic_priority_group_t HAL_CORTEX_NVIC_GetPriorityGrouping(void)
{
  return ((hal_cortex_nvic_priority_group_t)NVIC_GetPriorityGrouping());
}

/**
  * @brief Set the priority of an interrupt.
  * @param irqn: The interrupt number.
  *        This parameter can be a value of IRQn_Type enumeration.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  * @param preemp_prio: Specify the preemption priority for the IRQn channel.
  *        This parameter is an element of \ref hal_cortex_nvic_preemp_priority_t enumeration.
  * @param sub_prio: Specify the subpriority level for the IRQ channel.
  *        This parameter is an element of \ref hal_cortex_nvic_sub_priority_t enumeration.
  */
void HAL_CORTEX_NVIC_SetPriority(IRQn_Type irqn, hal_cortex_nvic_preemp_priority_t preemp_prio,
                                 hal_cortex_nvic_sub_priority_t sub_prio)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));
  ASSERT_DBG_PARAM(IS_PREEMP_PRIORITY(NVIC_GetPriorityGrouping(), preemp_prio));
  ASSERT_DBG_PARAM(IS_SUB_PRIORITY(NVIC_GetPriorityGrouping(), sub_prio));

  /* Set the preemption priority and sub-priority according to the priority grouping meaning
     the number of allocated bits used respectively to encode the preemption and sub-priority */
  NVIC_SetPriority(irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), (uint32_t)preemp_prio, (uint32_t)sub_prio));
}

/**
  * @brief Get the priority of an interrupt.
  * @param irqn: The interrupt number.
  *         This parameter can be a value of IRQn_Type enumeration.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  * @param p_preemp_prio: Get the preemption priority for the IRQn channel.
  *        This parameter is an element of \ref hal_cortex_nvic_preemp_priority_t enumeration.
  * @param p_sub_prio: Get the subpriority level for the IRQ channel.
  *        This parameter is an element of \ref hal_cortex_nvic_sub_priority_t enumeration.
  */
void HAL_CORTEX_NVIC_GetPriority(IRQn_Type irqn, hal_cortex_nvic_preemp_priority_t *p_preemp_prio,
                                 hal_cortex_nvic_sub_priority_t *p_sub_prio)
{
  uint32_t preemp_prio = 0U;
  uint32_t sub_prio = 0U;

  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));
  ASSERT_DBG_PARAM(p_preemp_prio != NULL);
  ASSERT_DBG_PARAM(p_sub_prio != NULL);
  /* Retrieve the preemption priority and sub-priority according to the priority grouping meaning the number
     of allocated bits used respectively to encode the preemption and sub-priority */
  NVIC_DecodePriority(NVIC_GetPriority(irqn), (uint32_t)NVIC_GetPriorityGrouping(), &preemp_prio, &sub_prio);

  *p_preemp_prio = (hal_cortex_nvic_preemp_priority_t)preemp_prio;
  *p_sub_prio    = (hal_cortex_nvic_sub_priority_t)sub_prio;
}

/**
  * @brief Enable the specified interrupt in the NVIC interrupt controller.
  * @param irqn External interrupt number.
  *        This parameter can be a value of IRQn_Type.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  */
void HAL_CORTEX_NVIC_EnableIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  NVIC_EnableIRQ(irqn);
}

/**
  * @brief Disable the specified interrupt in the NVIC interrupt controller.
  * @param irqn External interrupt number.
  *        This parameter can be a value of IRQn_Type.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  */
void HAL_CORTEX_NVIC_DisableIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  NVIC_DisableIRQ(irqn);
}

/**
  * @brief  Check whether the specified IRQn is enabled or disabled.
  * @param  irqn: Specify the interrupt number.
  *         This parameter can be a value of IRQn_Type enumeration.
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *          CMSIS device file (stm32c5xxxx.h))
  * @retval hal_cortex_nvic_irq_status_t Interrupt enable status value.
  */
hal_cortex_nvic_irq_status_t HAL_CORTEX_NVIC_IsEnabledIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  return ((hal_cortex_nvic_irq_status_t)NVIC_GetEnableIRQ(irqn));
}

/**
  * @brief  Check whether an interrupt is active.
  * @param  irqn: Specify the interrupt number.
  *         This parameter can be a value of IRQn_Type.
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *          CMSIS device file (stm32c5xxxx.h))
  * @note   Reads the active register in NVIC and returns the active bit.
  * @retval hal_cortex_nvic_irq_active_status_t Interrupt active status value.
  */
hal_cortex_nvic_irq_active_status_t HAL_CORTEX_NVIC_IsActiveIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  return ((hal_cortex_nvic_irq_active_status_t)NVIC_GetActive(irqn));
}

/**
  * @brief Set the pending bit of an external interrupt.
  * @param irqn External interrupt number.
  *        This parameter can be a value of IRQn_Type enumeration.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  */
void HAL_CORTEX_NVIC_SetPendingIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  NVIC_SetPendingIRQ(irqn);
}

/**
  * @brief Clear the pending bit of an external interrupt.
  * @param irqn External interrupt number.
  *        This parameter can be a value of IRQn_Type enumeration.
  *        (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *         CMSIS device file (stm32c5xxxx.h))
  */
void HAL_CORTEX_NVIC_ClearPendingIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  NVIC_ClearPendingIRQ(irqn);
}

/**
  * @brief  Check pending interrupt.
  * @param  irqn External interrupt number.
  *         This parameter can be a value of IRQn_Type enumeration.
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate
  *          CMSIS device file (stm32c5xxxx.h))
  * @note   Reads the NVIC pending register and returns the pending bit for the specified interrupt.
  * @retval hal_cortex_nvic_irq_pending_status_t Interrupt pending status value.
  */
hal_cortex_nvic_irq_pending_status_t HAL_CORTEX_NVIC_IsPendingIRQ(IRQn_Type irqn)
{
  ASSERT_DBG_PARAM(IS_IRQ_NUMBER(irqn));

  return ((hal_cortex_nvic_irq_pending_status_t)NVIC_GetPendingIRQ(irqn));
}

/**
  * @brief  Initiate a system reset request to reset the MCU.
  */
__NO_RETURN void HAL_CORTEX_NVIC_SystemReset(void)
{
  NVIC_SystemReset();
}

/**
  * @}
  */

/** @addtogroup CORTEX_Exported_Functions_Group2
  * @{
- This subsection provides a set of functions to configure CORTEX SYSTICK block features.
  - Use HAL_CORTEX_SYSTICK_SetFreq() to configure SYSTICK block frequency.
  - Use HAL_CORTEX_SYSTICK_SetClkSource() to configure clock source.
  - Use HAL_CORTEX_SYSTICK_Suspend() to suspend the core ticks.
  - Use HAL_CORTEX_SYSTICK_Resume() to resume the core ticks.
  */

/**
  * @brief  Configure the SysTick frequency.
  * @param  ticks_freq: Specifies the frequency in Hz.
  * @retval HAL_OK              Function succeeded.
  * @retval HAL_ERROR           Function failed.
  * @retval HAL_INVALID_PARAM   Invalid parameter.
  */
hal_status_t HAL_CORTEX_SYSTICK_SetFreq(uint32_t ticks_freq)
{
  uint32_t ticks = 0U;

  /* Check the parameters */
  ASSERT_DBG_PARAM(ticks_freq > 0UL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (ticks_freq == 0UL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  /* Get ticks frequency according to the SysTick source clock frequency */
  if (SysTick_GetClkSource() == SysTick_CTRL_CLKSOURCE_Msk)
  {
    /* The SysTick clock source is the CPU clock, get the CPU clock frequency */
    ticks = HAL_RCC_GetHCLKFreq();
  }
  else
  {
    /* The SysTick clock source is external (RCC). Get its frequency */
    ticks = HAL_RCC_GetSysTickExternalClkFreq();
  }

  ticks /= ticks_freq;

  /* Set the SysTick reload counter according to the ticks frequency and requested frequency */
  if (SysTick_SetReload(ticks) == 1U)
  {
    /* Reload value impossible */
    return HAL_ERROR;
  }

  /* Load the SysTick counter value */
  SysTick_SetCounter(0UL);

  SysTick_Enable();
  SysTick_EnableIT();

  return HAL_OK;
}

/**
  * @brief Configure the SysTick clock source.
  * @param clk_src: Specifies the SysTick clock source.
  *        This parameter is an element of \ref hal_cortex_systick_clk_src_t enumeration.
  */
void HAL_CORTEX_SYSTICK_SetClkSource(hal_cortex_systick_clk_src_t clk_src)
{
  /* Check the parameters */
  ASSERT_DBG_PARAM(IS_CLOCK_SOURCE(clk_src));

  /* Configure the SysTick CPU clock source */
  SysTick_SetClkSource((uint32_t)clk_src);
}

/**
  * @brief Suspend core ticks.
  */
void HAL_CORTEX_SYSTICK_Suspend(void)
{
  SysTick_DisableIT();
}

/**
  * @brief Resume core ticks.
  */
void HAL_CORTEX_SYSTICK_Resume(void)
{
  SysTick_EnableIT();
}

/**
  * @brief Handle SYSTICK interrupt request.
  */
void HAL_CORTEX_SYSTICK_IRQHandler(void)
{
  HAL_CORTEX_SYSTICK_Callback();
}

/**
  * @brief SYSTICK callback.
  */
__WEAK void HAL_CORTEX_SYSTICK_Callback(void)
{
  /* Warning: Do not modify this function. When the callback is needed,
             implement HAL_CORTEX_SYSTICK_Callback in the user file.
   */
}

/**
  * @}
  */

/** @addtogroup CORTEX_Exported_Functions_Group3
  *  @{
- This subsection provides a set of functions to configure the CORTEX MPU block features.
  - Use HAL_CORTEX_MPU_Enable() to enable the MPU.
  - Use HAL_CORTEX_MPU_Disable() to disable the MPU.
  - Use HAL_CORTEX_MPU_IsEnabled() to check whether the MPU is enabled or disabled.
  - Use HAL_CORTEX_MPU_GetDeviceMemAttr() to get device memory attributes.
  - Use HAL_CORTEX_MPU_SetDeviceMemAttr() to set device memory attributes.
  - Use HAL_CORTEX_MPU_GetCacheMemAttr() to get normal memory (cache) attributes.
  - Use HAL_CORTEX_MPU_SetCacheMemAttr() to set normal memory (cache) attributes.
  - Use HAL_CORTEX_MPU_SetConfigRegion() to set region configuration.
  - Use HAL_CORTEX_MPU_GetConfigRegion() to get region configuration.
  - Use HAL_CORTEX_MPU_EnableRegion() to enable a region configuration.
  - Use HAL_CORTEX_MPU_DisableRegion() to disable a region configuration.
  - Use HAL_CORTEX_MPU_IsEnabledRegion() to check whether a region memory enabled or not.
  */

/**
  * @brief Enable MPU and set the control mode of the MPU during HardFault,
  *        NMI, FAULTMASK, and privileged access to the default memory.
  * @param fault_state: Configure the control mode during HardFault, NMI, and FAULTMASK.
  *        This parameter is an element of \ref hal_cortex_mpu_hardfault_nmi_state_t enumeration.
  * @param priv_default_state: Configure the privileged access to the default memory.
  *        This parameter is an element of \ref hal_cortex_mpu_unmapped_addr_fault_t enumeration.
  */
void HAL_CORTEX_MPU_Enable(hal_cortex_mpu_hardfault_nmi_state_t fault_state,
                           hal_cortex_mpu_unmapped_addr_fault_t priv_default_state)
{
  ASSERT_DBG_PARAM(IS_NMI_STATE(fault_state));
  ASSERT_DBG_PARAM(IS_ACCESS_PRIV(priv_default_state));

  ARM_MPU_Enable(((uint32_t)fault_state) | ((uint32_t)priv_default_state));
}

/**
  * @brief  Disable MPU
  */
void HAL_CORTEX_MPU_Disable(void)
{
  ARM_MPU_Disable();
}

/**
  * @brief  Check whether the MPU is enabled.
  * @retval hal_cortex_mpu_status_t MPU status value.
  */
hal_cortex_mpu_status_t HAL_CORTEX_MPU_IsEnabled(void)
{
  return ((STM32_READ_BIT(MPU->CTRL, MPU_CTRL_ENABLE_Msk) == MPU_CTRL_ENABLE_Msk)
          ? HAL_CORTEX_MPU_ENABLED : HAL_CORTEX_MPU_DISABLED);
}

/**
  * @brief Set the device memory attributes configuration.
  * @param attr_idx: Specify the attributes index.
  *        This parameter is an element of \ref hal_cortex_mpu_mem_attr_idx_t enumeration.
  * @param mem_attr: Specify the device.
  *        This parameter is an element of \ref hal_cortex_mpu_device_mem_attr_t enumeration.
  */
void HAL_CORTEX_MPU_SetDeviceMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx, hal_cortex_mpu_device_mem_attr_t mem_attr)
{
  ASSERT_DBG_PARAM(IS_MEM_ATTR_IDX(attr_idx));
  ASSERT_DBG_PARAM(IS_DEVICE_MEM_ATTR(mem_attr));

  ARM_MPU_SetMemAttr((uint8_t)attr_idx, (uint8_t)mem_attr);
}

/**
  * @brief  Get the device memory attributes configuration.
  * @param  attr_idx: Specify the attributes index.
  *         This parameter is an element of \ref hal_cortex_mpu_mem_attr_idx_t enumeration.
  * @retval hal_cortex_mpu_device_mem_attr_t Device memory attribute value.
  */
hal_cortex_mpu_device_mem_attr_t HAL_CORTEX_MPU_GetDeviceMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx)
{
  hal_cortex_mpu_device_mem_attr_t mem_attr;
  uint8_t attr_reg_idx = 0U;
  uint8_t attr_bits_pos = 0U;
  uint8_t dev_attr = 0U;

  ASSERT_DBG_PARAM(IS_MEM_ATTR_IDX(attr_idx));

  attr_reg_idx  = (uint8_t)attr_idx / CORTEX_ATTR_REG_NUM;
  attr_bits_pos = (((uint8_t)attr_idx % CORTEX_ATTR_REG_NUM) * CORTEX_ATTR_BITS_NUM);
  dev_attr      = (uint8_t)(MPU->MAIR[attr_reg_idx] >> attr_bits_pos);

  if ((dev_attr & (~CORTEX_DEVICE_MASK)) == 0U)
  {
    mem_attr = (hal_cortex_mpu_device_mem_attr_t)(dev_attr);
  }
  else
  {
    mem_attr = HAL_CORTEX_MPU_DEVICE_MEM_INVALID;
  }

  return (mem_attr);
}

/**
  * @brief Set cache memory attributes configuration.
  * @param attr_idx: Specify the attributes index.
  *        This parameter is an element of \ref hal_cortex_mpu_mem_attr_idx_t enumeration.
  * @param mem_attr: Specify the cache memory config.
  *        This parameter is an element of \ref hal_cortex_mpu_normal_mem_cache_attr_t enumeration.
  */
void HAL_CORTEX_MPU_SetCacheMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx,
                                    hal_cortex_mpu_normal_mem_cache_attr_t mem_attr)
{
  ASSERT_DBG_PARAM(IS_MEM_ATTR_IDX(attr_idx));
  ASSERT_DBG_PARAM(IS_NORMAL_MEM_ATTR(mem_attr));

  ARM_MPU_SetMemAttr((uint8_t)attr_idx, (CORTEX_ATTR_INNER_MASK | (uint8_t)mem_attr));
}

/**
  * @brief  Get the cache memory attributes configuration.
  * @param  attr_idx: Specify the attributes index.
  *         This parameter is an element of \ref hal_cortex_mpu_mem_attr_idx_t enumeration.
  * @retval hal_cortex_mpu_normal_mem_cache_attr_t Normal memory attribute value.
  */
hal_cortex_mpu_normal_mem_cache_attr_t HAL_CORTEX_MPU_GetCacheMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx)
{
  hal_cortex_mpu_normal_mem_cache_attr_t mem_attr;
  uint8_t attr_reg_idx = 0U;
  uint8_t attr_bits_pos = 0U;
  uint8_t normal_mem_attr = 0U;

  ASSERT_DBG_PARAM(IS_MEM_ATTR_IDX(attr_idx));

  attr_reg_idx    = ((uint8_t)attr_idx / CORTEX_ATTR_REG_NUM);
  attr_bits_pos   = (((uint8_t)attr_idx % CORTEX_ATTR_REG_NUM) * CORTEX_ATTR_BITS_NUM);
  normal_mem_attr = (uint8_t)(MPU->MAIR[attr_reg_idx] >> attr_bits_pos);

  /* Check if the normal memory mode is enabled */
  if ((normal_mem_attr & CORTEX_NORMAL_MASK) != 0U)
  {
    mem_attr = (hal_cortex_mpu_normal_mem_cache_attr_t)(uint32_t)((uint32_t)normal_mem_attr & CORTEX_ATTR_OUTER_MASK);
  }
  else
  {
    mem_attr = HAL_CORTEX_MPU_NORMAL_MEM_INVALID;
  }

  return (mem_attr);
}

/**
  * @brief  Set the MPU region configuration.
  * @param  region_idx: Specify the region index.
  *         This parameter is an element of \ref hal_cortex_mpu_region_idx_t enumeration.
  * @param  p_config: Pointer to the \ref hal_cortex_mpu_region_config_t structure
  *                   that contains the configuration for the MPU region.
  * @retval HAL_OK            Function succeeded.
  * @retval HAL_INVALID_PARAM Function failed (HAL_INVALID_PARAM).
  */
hal_status_t HAL_CORTEX_MPU_SetConfigRegion(hal_cortex_mpu_region_idx_t region_idx,
                                            const hal_cortex_mpu_region_config_t *p_config)
{
  ASSERT_DBG_PARAM(IS_MPU_REGION(region_idx));

  ASSERT_DBG_PARAM(p_config != NULL);
  ASSERT_DBG_PARAM(IS_MEM_ATTR_IDX(p_config->attr_idx));
  ASSERT_DBG_PARAM(IS_ACCESS_ATTR(p_config->access_attr));
  ASSERT_DBG_PARAM(IS_EXEC_ATTR(p_config->exec_attr));

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)

  if (p_config == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  ARM_MPU_SetRegion((uint32_t)region_idx, (((uint32_t)p_config->base_addr & CORTEX_REGION_ADDR_MASK)
                                           | ((uint32_t)p_config->access_attr << MPU_RBAR_AP_Pos)
                                           | ((uint32_t)p_config->exec_attr   << MPU_RBAR_XN_Pos)),
                    (((uint32_t)p_config->limit_addr & CORTEX_REGION_ADDR_MASK)
                     | ((uint32_t)p_config->attr_idx << MPU_RLAR_AttrIndx_Pos)));

  return HAL_OK;
}

/**
  * @brief Get the MPU region configuration.
  * @param region_idx: Specify the region index.
  *        This parameter is an element of \ref hal_cortex_mpu_region_idx_t enumeration.
  * @param p_config: Pointer to the \ref hal_cortex_mpu_region_config_t structure
  *                  that contains the configuration for the MPU region.
  */
void HAL_CORTEX_MPU_GetConfigRegion(hal_cortex_mpu_region_idx_t region_idx,
                                    hal_cortex_mpu_region_config_t *p_config)
{
  ASSERT_DBG_PARAM(IS_MPU_REGION(region_idx));
  ASSERT_DBG_PARAM(p_config != NULL);

  STM32_WRITE_REG(MPU->RNR, (uint32_t)region_idx);
  p_config->base_addr   = _FLD2VAL(MPU_RBAR_BASE, MPU->RBAR) << MPU_RBAR_BASE_Pos;
  p_config->access_attr = (hal_cortex_mpu_region_access_attr_t)_FLD2VAL(MPU_RBAR_AP, MPU->RBAR);
  p_config->exec_attr   = (hal_cortex_mpu_execution_attr_t)(uint32_t)STM32_READ_BIT(MPU->RBAR, MPU_RBAR_XN_Msk);
  p_config->limit_addr  = (_FLD2VAL(MPU_RLAR_LIMIT, MPU->RLAR) << MPU_RLAR_LIMIT_Pos) + 0x1FUL;
  p_config->attr_idx    = (hal_cortex_mpu_mem_attr_idx_t) _FLD2VAL(MPU_RLAR_AttrIndx, MPU->RLAR);
}

/**
  * @brief Enable the given MPU region.
  * @param region_idx: Specify the region index.
  *        This parameter is an element of \ref hal_cortex_mpu_region_idx_t enumeration.
  */
void HAL_CORTEX_MPU_EnableRegion(hal_cortex_mpu_region_idx_t region_idx)
{
  ASSERT_DBG_PARAM(IS_MPU_REGION(region_idx));

  STM32_WRITE_REG(MPU->RNR, (uint32_t)region_idx);

  STM32_SET_BIT(MPU->RLAR, MPU_RLAR_EN_Msk);
}

/**
  * @brief Disable the given MPU region.
  * @param region_idx: Specify the region index.
  *        This parameter is an element of \ref hal_cortex_mpu_region_idx_t enumeration.
  */
void HAL_CORTEX_MPU_DisableRegion(hal_cortex_mpu_region_idx_t region_idx)
{
  ASSERT_DBG_PARAM(IS_MPU_REGION(region_idx));

  STM32_WRITE_REG(MPU->RNR, (uint32_t)region_idx);

  STM32_CLEAR_BIT(MPU->RLAR, MPU_RLAR_EN_Msk);
}

/**
  * @brief  Check whether the given MPU region is enabled.
  * @param  region_idx: Specify the region index.
  *         This parameter is an element of \ref hal_cortex_mpu_region_idx_t enumeration.
  * @retval hal_cortex_mpu_region_status_t MPU region status value.
  */
hal_cortex_mpu_region_status_t HAL_CORTEX_MPU_IsEnabledRegion(hal_cortex_mpu_region_idx_t region_idx)
{
  ASSERT_DBG_PARAM(IS_MPU_REGION(region_idx));

  STM32_WRITE_REG(MPU->RNR, (uint32_t)region_idx);

  return ((STM32_READ_BIT(MPU->RLAR, MPU_RLAR_EN_Msk) == MPU_RLAR_EN_Msk)
          ? HAL_CORTEX_MPU_REGION_ENABLED : HAL_CORTEX_MPU_REGION_DISABLED);
}


/**
  * @}
  */

/** @addtogroup CORTEX_Exported_Functions_Group4
  *  @{
- This subsection provides a set of functions to configure CORTEX SCB block features.
  - Use HAL_CORTEX_SCB_GetInfo() to get the CPU ID information.
  - Use HAL_CORTEX_SCB_DisableHardFaultEscalation() to disable an exception fault escalation.
  - Use HAL_CORTEX_SCB_EnableHardFaultEscalation() to enable an exception fault escalation.
 */

/**
  * @brief Get CPU ID information.
  * @param p_info: Pointer to the \ref hal_cortex_scb_cpuid_info_t structure.
  */
void HAL_CORTEX_SCB_GetInfo(hal_cortex_scb_cpuid_info_t *p_info)
{
  ASSERT_DBG_PARAM(p_info != NULL);

  p_info->revision     = _FLD2VAL(SCB_CPUID_REVISION, SCB->CPUID);
  p_info->part_number  = _FLD2VAL(SCB_CPUID_PARTNO, SCB->CPUID);
  p_info->architecture = _FLD2VAL(SCB_CPUID_ARCHITECTURE, SCB->CPUID);
  p_info->variant      = _FLD2VAL(SCB_CPUID_VARIANT, SCB->CPUID);
  p_info->implementer  = _FLD2VAL(SCB_CPUID_IMPLEMENTER, SCB->CPUID);
}

/**
  * @brief Enable a fault escalation to HardFault.
  * @param faults This parameter can be a combination of the following values:
  *        @arg @ref HAL_CORTEX_SCB_USAGE_FAULT           Usage fault
  *        @arg @ref HAL_CORTEX_SCB_BUS_FAULT             Bus fault
  *        @arg @ref HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT  Memory management fault
  */
void HAL_CORTEX_SCB_EnableHardFaultEscalation(uint32_t faults)
{
  ASSERT_DBG_PARAM(IS_FAULT_EXCEPT(faults));

  SCB_DisableFault(faults);
}

/**
  * @brief Disable a fault escalation to HardFault.
  * @param faults This parameter can be a combination of the following values:
  *        @arg @ref HAL_CORTEX_SCB_USAGE_FAULT           Usage fault
  *        @arg @ref HAL_CORTEX_SCB_BUS_FAULT             Bus fault
  *        @arg @ref HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT  Memory management fault
  */
void HAL_CORTEX_SCB_DisableHardFaultEscalation(uint32_t faults)
{
  ASSERT_DBG_PARAM(IS_FAULT_EXCEPT(faults));

  SCB_EnableFault(faults);
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
#endif /* USE_HAL_CORTEX_MODULE */
/**
  * @}
  */
