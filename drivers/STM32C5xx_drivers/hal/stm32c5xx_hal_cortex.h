/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_cortex.h
  * @brief   Header file for the CORTEX HAL module.
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

/* Define to prevent recursive inclusion. ----------------------------------------------------------------------------*/
#ifndef STM32C5XX_HAL_CORTEX_H
#define STM32C5XX_HAL_CORTEX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup CORTEX CORTEX
  * @{
  */

/* Exported types ----------------------------------------------------------------------------------------------------*/
/** @defgroup CORTEX_Exported_Types HAL CORTEX Types
  * @{
  */

/*! CORTEX priority group enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_PRIORITY_GROUP_0 = 0x7U, /*!< 0 bit  for preemption priority, 4 bits for sub-priority */
  HAL_CORTEX_NVIC_PRIORITY_GROUP_1 = 0x6U, /*!< 1 bit  for preemption priority, 3 bits for sub-priority */
  HAL_CORTEX_NVIC_PRIORITY_GROUP_2 = 0x5U, /*!< 2 bits for preemption priority, 2 bits for sub-priority */
  HAL_CORTEX_NVIC_PRIORITY_GROUP_3 = 0x4U, /*!< 3 bits for preemption priority, 1 bit  for sub-priority */
  HAL_CORTEX_NVIC_PRIORITY_GROUP_4 = 0x3U  /*!< 4 bits for preemption priority, 0 bit  for sub-priority */
} hal_cortex_nvic_priority_group_t;

/*! CORTEX preemption priority enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_0  = 0x0U,  /*!< NVIC preemption priority 0  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_1  = 0x1U,  /*!< NVIC preemption priority 1  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_2  = 0x2U,  /*!< NVIC preemption priority 2  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_3  = 0x3U,  /*!< NVIC preemption priority 3  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_4  = 0x4U,  /*!< NVIC preemption priority 4  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_5  = 0x5U,  /*!< NVIC preemption priority 5  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_6  = 0x6U,  /*!< NVIC preemption priority 6  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_7  = 0x7U,  /*!< NVIC preemption priority 7  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_8  = 0x8U,  /*!< NVIC preemption priority 8  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_9  = 0x9U,  /*!< NVIC preemption priority 9  */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_10 = 0xAU,  /*!< NVIC preemption priority 10 */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_11 = 0xBU,  /*!< NVIC preemption priority 11 */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_12 = 0xCU,  /*!< NVIC preemption priority 12 */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_13 = 0xDU,  /*!< NVIC preemption priority 13 */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_14 = 0xEU,  /*!< NVIC preemption priority 14 */
  HAL_CORTEX_NVIC_PREEMP_PRIORITY_15 = 0xFU   /*!< NVIC preemption priority 15 */
} hal_cortex_nvic_preemp_priority_t;

/*! CORTEX sub-priority enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_SUB_PRIORITY_0  = 0x0U,  /*!< NVIC sub-priority 0  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_1  = 0x1U,  /*!< NVIC sub-priority 1  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_2  = 0x2U,  /*!< NVIC sub-priority 2  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_3  = 0x3U,  /*!< NVIC sub-priority 3  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_4  = 0x4U,  /*!< NVIC sub-priority 4  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_5  = 0x5U,  /*!< NVIC sub-priority 5  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_6  = 0x6U,  /*!< NVIC sub-priority 6  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_7  = 0x7U,  /*!< NVIC sub-priority 7  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_8  = 0x8U,  /*!< NVIC sub-priority 8  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_9  = 0x9U,  /*!< NVIC sub-priority 9  */
  HAL_CORTEX_NVIC_SUB_PRIORITY_10 = 0xAU,  /*!< NVIC sub-priority 10 */
  HAL_CORTEX_NVIC_SUB_PRIORITY_11 = 0xBU,  /*!< NVIC sub-priority 11 */
  HAL_CORTEX_NVIC_SUB_PRIORITY_12 = 0xCU,  /*!< NVIC sub-priority 12 */
  HAL_CORTEX_NVIC_SUB_PRIORITY_13 = 0xDU,  /*!< NVIC sub-priority 13 */
  HAL_CORTEX_NVIC_SUB_PRIORITY_14 = 0xEU,  /*!< NVIC sub-priority 14 */
  HAL_CORTEX_NVIC_SUB_PRIORITY_15 = 0xFU   /*!< NVIC sub-priority 15 */
} hal_cortex_nvic_sub_priority_t;

/*! CORTEX NVIC IRQ status enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_IRQ_DISABLED = 0U, /*!< NVIC IRQ disabled */
  HAL_CORTEX_NVIC_IRQ_ENABLED  = 1U  /*!< NVIC IRQ enabled  */
} hal_cortex_nvic_irq_status_t;

/*! CORTEX NVIC IRQ active status enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_IRQ_NOT_ACTIVE = 0U, /*!< NVIC IRQ not active */
  HAL_CORTEX_NVIC_IRQ_ACTIVE     = 1U  /*!< NVIC IRQ active     */
} hal_cortex_nvic_irq_active_status_t;

/*! CORTEX NVIC IRQ pending status enumeration definition */
typedef enum
{
  HAL_CORTEX_NVIC_IRQ_NOT_PENDING = 0U, /*!< NVIC IRQ not pending */
  HAL_CORTEX_NVIC_IRQ_PENDING     = 1U  /*!< NVIC IRQ pending     */
} hal_cortex_nvic_irq_pending_status_t;

/*! CORTEX SysTick clock source enumeration definition */
typedef enum
{
  HAL_CORTEX_SYSTICK_CLKSOURCE_INTERNAL = SysTick_CTRL_CLKSOURCE_Msk,      /*!< Internal clock selected as SysTick clock
                                                                           source */
  HAL_CORTEX_SYSTICK_CLKSOURCE_EXTERNAL = 0U                               /*!< External clock selected as SysTick clock
                                                                           source */
} hal_cortex_systick_clk_src_t;

/*! CORTEX MPU unmapped address fault enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_ACCESS_FAULT_ALL       = 0x00,                    /*!< All accesses to unmapped addresses result
                                                                        in faults */
  HAL_CORTEX_MPU_ACCESS_FAULT_ONLY_PRIV = MPU_CTRL_PRIVDEFENA_Msk  /*!< Enables the default memory map for privileged
                                                                        code      */
} hal_cortex_mpu_unmapped_addr_fault_t;

/*! CORTEX MPU HardFault NMI status enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_HARDFAULT_NMI_DISABLE = 0x00,                  /*!< HardFault and NMI handlers bypass MPU
                                                                      configuration as if MPU is disabled */
  HAL_CORTEX_MPU_HARDFAULT_NMI_ENABLE  = MPU_CTRL_HFNMIENA_Msk  /*!< MPU access rules apply to HardFault and NMI
                                                                      handlers                            */
} hal_cortex_mpu_hardfault_nmi_state_t;

/*! CORTEX MPU memory attributes index enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_MEM_ATTR_0 = 0x00U, /*!< MPU memory attributes index 0 */
  HAL_CORTEX_MPU_MEM_ATTR_1 = 0x01U, /*!< MPU memory attributes index 1 */
  HAL_CORTEX_MPU_MEM_ATTR_2 = 0x02U, /*!< MPU memory attributes index 2 */
  HAL_CORTEX_MPU_MEM_ATTR_3 = 0x03U, /*!< MPU memory attributes index 3 */
  HAL_CORTEX_MPU_MEM_ATTR_4 = 0x04U, /*!< MPU memory attributes index 4 */
  HAL_CORTEX_MPU_MEM_ATTR_5 = 0x05U, /*!< MPU memory attributes index 5 */
  HAL_CORTEX_MPU_MEM_ATTR_6 = 0x06U, /*!< MPU memory attributes index 6 */
  HAL_CORTEX_MPU_MEM_ATTR_7 = 0x07U  /*!< MPU memory attributes index 7 */
} hal_cortex_mpu_mem_attr_idx_t;

/*! CORTEX MPU device attributes enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_DEVICE_MEM_NGNRNE  = (0x00U << 2U), /*!< Device memory, noGather, noReorder, noEarly acknowledge */
  HAL_CORTEX_MPU_DEVICE_MEM_NGNRE   = (0x01U << 2U), /*!< Device memory, noGather, noReorder, Early acknowledge   */
  HAL_CORTEX_MPU_DEVICE_MEM_NGRE    = (0x02U << 2U), /*!< Device memory, noGather, Reorder, Early acknowledge     */
  HAL_CORTEX_MPU_DEVICE_MEM_GRE     = (0x03U << 2U), /*!< Device memory, Gather, Reorder, Early acknowledge       */
  HAL_CORTEX_MPU_DEVICE_MEM_INVALID = (0xFFU)        /*!< Returned when getting configuration of normal memory    */
} hal_cortex_mpu_device_mem_attr_t;

/*! CORTEX MPU normal attributes enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_NORMAL_MEM_NCACHEABLE    = (0x4U << 4U), /*!< Normal, non-cacheable                                   */
  HAL_CORTEX_MPU_NORMAL_MEM_WT_NOA        = (0x8U << 4U), /*!< Normal, write-through non-allocate non-transient        */
  HAL_CORTEX_MPU_NORMAL_MEM_WT_WA         = (0x9U << 4U), /*!< Normal, write-through write allocate non-transient      */
  HAL_CORTEX_MPU_NORMAL_MEM_WT_RA         = (0xAU << 4U), /*!< Normal, write-through read-allocate non-transient       */
  HAL_CORTEX_MPU_NORMAL_MEM_WT_RWA        = (0xBU << 4U), /*!< Normal, write-through read/write-allocate non-transient */
  HAL_CORTEX_MPU_NORMAL_MEM_WB_NOA        = (0xCU << 4U), /*!< Normal, write-back non-allocate non-transient           */
  HAL_CORTEX_MPU_NORMAL_MEM_WB_WA         = (0xDU << 4U), /*!< Normal, write-back write allocate non-transient         */
  HAL_CORTEX_MPU_NORMAL_MEM_WB_RA         = (0xEU << 4U), /*!< Normal, write-back read-allocate non-transient          */
  HAL_CORTEX_MPU_NORMAL_MEM_WB_RWA        = (0xFU << 4U), /*!< Normal, write-back read/write-allocate non-transient    */
  HAL_CORTEX_MPU_NORMAL_MEM_INVALID       = (0xFFU)       /*!< Returned when getting configuration of device memory    */
} hal_cortex_mpu_normal_mem_cache_attr_t;

/*! CORTEX MPU region index enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_REGION_0 = 0U, /*!< MPU region index 0 */
  HAL_CORTEX_MPU_REGION_1 = 1U, /*!< MPU region index 1 */
  HAL_CORTEX_MPU_REGION_2 = 2U, /*!< MPU region index 2 */
  HAL_CORTEX_MPU_REGION_3 = 3U, /*!< MPU region index 3 */
  HAL_CORTEX_MPU_REGION_4 = 4U, /*!< MPU region index 4 */
  HAL_CORTEX_MPU_REGION_5 = 5U, /*!< MPU region index 5 */
  HAL_CORTEX_MPU_REGION_6 = 6U, /*!< MPU region index 6 */
  HAL_CORTEX_MPU_REGION_7 = 7U  /*!< MPU region index 7 */
} hal_cortex_mpu_region_idx_t;

/*! CORTEX MPU region access attributes enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_REGION_ONLY_PRIV_RW = 0x00U, /*!< Read/write by privileged code only */
  HAL_CORTEX_MPU_REGION_ALL_RW       = 0x01U, /*!< Read/write by any privilege level  */
  HAL_CORTEX_MPU_REGION_ONLY_PRIV_RO = 0x02U, /*!< Read only by privileged code only  */
  HAL_CORTEX_MPU_REGION_ALL_RO       = 0x03U  /*!< Read only by any privilege level   */
} hal_cortex_mpu_region_access_attr_t;

/*! CORTEX MPU execution access enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_EXECUTION_ATTR_DISABLE = 0x01U, /*!< MPU execution attribute disable */
  HAL_CORTEX_MPU_EXECUTION_ATTR_ENABLE  = 0x00U  /*!< MPU execution attribute enable  */
} hal_cortex_mpu_execution_attr_t;

/*! CORTEX MPU status enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_DISABLED  = 0U, /*!< MPU disabled */
  HAL_CORTEX_MPU_ENABLED   = 1U  /*!< MPU enabled  */
} hal_cortex_mpu_status_t;

/*! CORTEX MPU region status enumeration definition */
typedef enum
{
  HAL_CORTEX_MPU_REGION_DISABLED  = 0U, /*!< MPU region disabled */
  HAL_CORTEX_MPU_REGION_ENABLED   = 1U  /*!< MPU region enabled  */
} hal_cortex_mpu_region_status_t;

/*! CORTEX MPU region configuration structure definition */
typedef struct
{
  uint32_t                              base_addr;      /*!< Base address of the region to protect  */
  uint32_t                              limit_addr;     /*!< Limit address of the region to protect */
  hal_cortex_mpu_region_access_attr_t   access_attr;    /*!< Region access permission               */
  hal_cortex_mpu_execution_attr_t       exec_attr;      /*!< Execution attribute status             */
  hal_cortex_mpu_mem_attr_idx_t         attr_idx;       /*!< Memory attributes index                */
} hal_cortex_mpu_region_config_t;

/*! CORTEX SCB CPU ID information structure definition */
typedef struct
{
  uint32_t revision     : 4U;  /*!< Revision number identifier */
  uint32_t part_number  : 12U; /*!< Cortex part number         */
  uint32_t architecture : 4U;  /*!< Architecture version       */
  uint32_t variant      : 4U;  /*!< Variant                    */
  uint32_t implementer  : 8U;  /*!< Implementer identifier     */
} hal_cortex_scb_cpuid_info_t;
/**
  * @}
  */

/* Exported constants ------------------------------------------------------------------------------------------------*/
/** @defgroup CORTEX_Exported_Constants HAL CORTEX Constants
  * @{
  */

/** @defgroup  CORTEX_Fault_Exceptions Fault Exceptions definition
  * @{
  */
#define HAL_CORTEX_SCB_USAGE_FAULT          SCB_SHCSR_USGFAULTENA_Msk    /*!< Usage fault             */
#define HAL_CORTEX_SCB_BUS_FAULT            SCB_SHCSR_BUSFAULTENA_Msk    /*!< Bus fault               */
#define HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT SCB_SHCSR_MEMFAULTENA_Msk    /*!< Memory management fault */
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @defgroup CORTEX_Exported_Functions HAL CORTEX Functions
  * @{
  */

/**
  * @defgroup CORTEX_Exported_Functions_Group1 NVIC management functions
  * @{
  */
/* Priority grouping functions */
void                             HAL_CORTEX_NVIC_SetPriorityGrouping(hal_cortex_nvic_priority_group_t prio_grp);
hal_cortex_nvic_priority_group_t HAL_CORTEX_NVIC_GetPriorityGrouping(void);

/* Set and get priority configuration functions */
void HAL_CORTEX_NVIC_SetPriority(IRQn_Type irqn, hal_cortex_nvic_preemp_priority_t preemp_prio,
                                 hal_cortex_nvic_sub_priority_t sub_prio);
void HAL_CORTEX_NVIC_GetPriority(IRQn_Type irqn, hal_cortex_nvic_preemp_priority_t *p_preemp_prio,
                                 hal_cortex_nvic_sub_priority_t *p_sub_prio);

/* NVIC IRQ management functions */
void                                 HAL_CORTEX_NVIC_EnableIRQ(IRQn_Type irqn);
void                                 HAL_CORTEX_NVIC_DisableIRQ(IRQn_Type irqn);
hal_cortex_nvic_irq_status_t         HAL_CORTEX_NVIC_IsEnabledIRQ(IRQn_Type irqn);
hal_cortex_nvic_irq_active_status_t  HAL_CORTEX_NVIC_IsActiveIRQ(IRQn_Type irqn);
void                                 HAL_CORTEX_NVIC_SetPendingIRQ(IRQn_Type irqn);
void                                 HAL_CORTEX_NVIC_ClearPendingIRQ(IRQn_Type irqn);
hal_cortex_nvic_irq_pending_status_t HAL_CORTEX_NVIC_IsPendingIRQ(IRQn_Type irqn);

/* NVIC system reset function */
__NO_RETURN void HAL_CORTEX_NVIC_SystemReset(void);

/**
  * @}
  */

/**
  * @defgroup CORTEX_Exported_Functions_Group2 SYSTICK management functions
  * @{
  */
hal_status_t HAL_CORTEX_SYSTICK_SetFreq(uint32_t ticks_freq);
void         HAL_CORTEX_SYSTICK_SetClkSource(hal_cortex_systick_clk_src_t clk_src);
void         HAL_CORTEX_SYSTICK_Suspend(void);
void         HAL_CORTEX_SYSTICK_Resume(void);
void         HAL_CORTEX_SYSTICK_IRQHandler(void);
void         HAL_CORTEX_SYSTICK_Callback(void);

/**
  * @}
  */

/**
  * @defgroup CORTEX_Exported_Functions_Group3 MPU management functions
  * @{
  */
/* MPU Enable/Disable functions */
void                    HAL_CORTEX_MPU_Enable(hal_cortex_mpu_hardfault_nmi_state_t fault_state,
                                              hal_cortex_mpu_unmapped_addr_fault_t priv_default_state);
void                    HAL_CORTEX_MPU_Disable(void);
hal_cortex_mpu_status_t HAL_CORTEX_MPU_IsEnabled(void);

/* MPU memory attributes functions */
void                             HAL_CORTEX_MPU_SetDeviceMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx,
                                                                 hal_cortex_mpu_device_mem_attr_t mem_attr);
hal_cortex_mpu_device_mem_attr_t HAL_CORTEX_MPU_GetDeviceMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx);
void                             HAL_CORTEX_MPU_SetCacheMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx,
                                                                hal_cortex_mpu_normal_mem_cache_attr_t mem_attr);
hal_cortex_mpu_normal_mem_cache_attr_t HAL_CORTEX_MPU_GetCacheMemAttr(hal_cortex_mpu_mem_attr_idx_t attr_idx);

/* MPU region management functions */
hal_status_t                   HAL_CORTEX_MPU_SetConfigRegion(hal_cortex_mpu_region_idx_t region_idx,
                                                              const hal_cortex_mpu_region_config_t *p_config);
void                           HAL_CORTEX_MPU_GetConfigRegion(hal_cortex_mpu_region_idx_t region_idx,
                                                              hal_cortex_mpu_region_config_t *p_config);
void                           HAL_CORTEX_MPU_EnableRegion(hal_cortex_mpu_region_idx_t region_idx);
void                           HAL_CORTEX_MPU_DisableRegion(hal_cortex_mpu_region_idx_t region_idx);
hal_cortex_mpu_region_status_t HAL_CORTEX_MPU_IsEnabledRegion(hal_cortex_mpu_region_idx_t region_idx);

/**
  * @}
  */

/**
  * @defgroup CORTEX_Exported_Functions_Group4 SCB management functions
  * @{
  */
/* SCB CPU ID information function */
void HAL_CORTEX_SCB_GetInfo(hal_cortex_scb_cpuid_info_t *p_info);

/* SCB Cortex Fault management functions */
void HAL_CORTEX_SCB_EnableHardFaultEscalation(uint32_t faults);
void HAL_CORTEX_SCB_DisableHardFaultEscalation(uint32_t faults);
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

#endif /* STM32C5XX_HAL_CORTEX_H */
