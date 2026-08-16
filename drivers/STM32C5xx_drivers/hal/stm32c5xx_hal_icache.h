/**
  ******************************************************************************
  * @file    stm32c5xx_hal_icache.h
  * @brief   Header file of ICACHE HAL module.
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
#ifndef STM32C5XX_HAL_ICACHE_H
#define STM32C5XX_HAL_ICACHE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_icache.h"

/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

#if defined(ICACHE)

/** @addtogroup ICACHE
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup ICACHE_Exported_Types HAL ICACHE Types
  * @{
  */

/** @defgroup ICACHE_Exported_Types_Group1 Enumerations
  * @{
  */

/**
  * @brief HAL ICACHE instance definitions.
  */
typedef enum
{
  HAL_ICACHE = ICACHE_BASE    /*!< Instance ICACHE   */
} hal_icache_t;


/**
  * @brief  HAL ICACHE associativity definitions.
  */
typedef enum
{
  HAL_ICACHE_ASSOCIATIVITY_1WAY  = LL_ICACHE_1WAY,        /*!< 1-way */
  HAL_ICACHE_ASSOCIATIVITY_2WAYS = LL_ICACHE_2WAYS        /*!< 2-ways */
} hal_icache_associativity_t;


/**
  * @brief HAL ICACHE master port definitions.
  */
typedef enum
{
#if defined(LL_ICACHE_MASTER2_PORT)
  HAL_ICACHE_MASTER1_PORT   = LL_ICACHE_MASTER1_PORT,  /*!< Master1 port */
  HAL_ICACHE_MASTER2_PORT   = LL_ICACHE_MASTER2_PORT   /*!< Master2 port */
#else
  HAL_ICACHE_MASTER1_PORT   = LL_ICACHE_MASTER1_PORT   /*!< Master1 port */
#endif /* LL_ICACHE_MASTER2_PORT */
} hal_icache_master_port_t;


/**
  * @brief HAL ICACHE output burst definitions.
  */
typedef enum
{
  HAL_ICACHE_BURST_WRAP = LL_ICACHE_OUTPUT_BURST_WRAP, /*!< Output WRAP */
  HAL_ICACHE_BURST_INCR = LL_ICACHE_OUTPUT_BURST_INCR  /*!< Output INCR */
} hal_icache_region_burst_t;


/**
  * @brief HAL ICACHE remap region definitions.
  */
typedef enum
{
  HAL_ICACHE_REGION_0  = LL_ICACHE_REGION_0,  /*!< Region number 0 */
  HAL_ICACHE_REGION_1  = LL_ICACHE_REGION_1,  /*!< Region number 1 */
  HAL_ICACHE_REGION_2  = LL_ICACHE_REGION_2,  /*!< Region number 2 */
  HAL_ICACHE_REGION_3  = LL_ICACHE_REGION_3   /*!< Region number 3 */
} hal_icache_region_t;


/**
  * @brief HAL ICACHE remap region size definitions.
  */
typedef enum
{
  HAL_ICACHE_REGION_SIZE_2MBYTES   = LL_ICACHE_REGIONSIZE_2MB,   /*!< Region size 2MB */
  HAL_ICACHE_REGION_SIZE_4MBYTES   = LL_ICACHE_REGIONSIZE_4MB,   /*!< Region size 4MB */
  HAL_ICACHE_REGION_SIZE_8MBYTES   = LL_ICACHE_REGIONSIZE_8MB,   /*!< Region size 8MB */
  HAL_ICACHE_REGION_SIZE_16MBYTES  = LL_ICACHE_REGIONSIZE_16MB,  /*!< Region size 16MB */
  HAL_ICACHE_REGION_SIZE_32MBYTES  = LL_ICACHE_REGIONSIZE_32MB,  /*!< Region size 32MB */
  HAL_ICACHE_REGION_SIZE_64MBYTES  = LL_ICACHE_REGIONSIZE_64MB,  /*!< Region size 64MB */
  HAL_ICACHE_REGION_SIZE_128MBYTES = LL_ICACHE_REGIONSIZE_128MB  /*!< Region size 128MB */
} hal_icache_region_size_t;

/**
  * @brief HAL ICACHE remap region status definitions.
  */
typedef enum
{
  HAL_ICACHE_REMAP_REGION_DISABLED = 0U,      /*!< Corresponding remap region is disabled */
  HAL_ICACHE_REMAP_REGION_ENABLED  = 1U       /*!< Corresponding remap region is enabled */
} hal_icache_remap_region_status_t;

/**
  * @brief HAL ICACHE state definitions.
  */
typedef enum
{
  HAL_ICACHE_STATE_RESET       = 0UL,           /*!< ICACHE driver not initialized and not started */
  HAL_ICACHE_STATE_IDLE        = (1UL << 31U),  /*!< ICACHE driver initialized and not started */
  HAL_ICACHE_STATE_ACTIVE      = (1UL << 30U),  /*!< ICACHE driver initialized and started */
  HAL_ICACHE_STATE_MAINTENANCE = (1UL << 29U)   /*!< ICACHE driver initialized, started and a maintenance operation is
                                                  ongoing */
} hal_icache_state_t;

/**
  * @}
  */

/** @defgroup ICACHE_Exported_Types_Group2 Configuration Structure
  * @{
  */

/**
  * @brief HAL ICACHE region configuration structure definition.
  */
typedef struct
{
  /*! Configures the base address of the region to be remapped. */
  uint32_t base_address;
  /*! Configures the remap address of the region to be remapped. */
  uint32_t remap_address;
  /*! Configures the region size. */
  hal_icache_region_size_t size;
  /*! Selects the master port. */
  hal_icache_master_port_t master_port;
  /*! Selects the output burst type. */
  hal_icache_region_burst_t output_burst;
} hal_icache_region_config_t;

/**
  * @}
  */

/** @defgroup ICACHE_Exported_Types_Group3 Handle Structure
  * @{
  */
typedef struct hal_icache_handle_s hal_icache_handle_t; /*!< ICACHE handle type definition */

#if defined(USE_HAL_ICACHE_REGISTER_CALLBACKS) && (USE_HAL_ICACHE_REGISTER_CALLBACKS == 1)
typedef  void (*hal_icache_cb_t)(hal_icache_handle_t *hicache); /*!< Pointer to an ICACHE callback function */
#endif /* USE_HAL_ICACHE_REGISTER_CALLBACKS */

/**
  * @brief HAL ICACHE handle structure definition.
  */
struct hal_icache_handle_s
{
  /*! Peripheral instance */
  hal_icache_t instance;

  /*! ICACHE global state */
  volatile hal_icache_state_t global_state;

#if defined(USE_HAL_ICACHE_GET_LAST_ERRORS) && (USE_HAL_ICACHE_GET_LAST_ERRORS == 1)
  /*! Variable storing the last errors */
  volatile uint32_t last_error_codes;
#endif /* USE_HAL_ICACHE_GET_LAST_ERRORS */

#if defined(USE_HAL_ICACHE_REGISTER_CALLBACKS) && (USE_HAL_ICACHE_REGISTER_CALLBACKS == 1)
  /*! Error Callback pointer */
  hal_icache_cb_t p_error_cb;
  /*! Invalidate complete Callback pointer */
  hal_icache_cb_t p_invalidate_cplt_cb;
#endif /* USE_HAL_ICACHE_REGISTER_CALLBACKS */

#if defined(USE_HAL_ICACHE_USER_DATA) && (USE_HAL_ICACHE_USER_DATA == 1)
  /*! ICACHE user data */
  const void *p_user_data;
#endif /* USE_HAL_ICACHE_USER_DATA */
};

/**
  * @}
  */

/**
  * @}
  */


/* Exported constants ---------------------------------------------------------*/
/** @defgroup ICACHE_Exported_Constants HAL ICACHE Constants
  * @{
  */
/** @defgroup Monitoring_Constants Monitoring Constants
  * @{
  */
#define HAL_ICACHE_MONITOR_HIT        LL_ICACHE_MONITOR_HIT    /*!< Read Hit monitor */
#define HAL_ICACHE_MONITOR_MISS       LL_ICACHE_MONITOR_MISS   /*!< Read Miss monitor */
#define HAL_ICACHE_MONITOR_ALL        LL_ICACHE_MONITOR_ALL    /*!< Read Miss/Hit monitor */
/**
  * @}
  */

/** @defgroup Interruptions_Constants Interrupts
  * @{
  */
#define HAL_ICACHE_IT_NONE        0U                       /*!< No interrupt */
#define HAL_ICACHE_IT_ERROR       LL_ICACHE_IER_ERRIE      /*!< Error interrupt */
/**
  * @}
  */

/** @defgroup Error_Codes Error Codes
  * @{
  */
#if defined(USE_HAL_ICACHE_GET_LAST_ERRORS) && (USE_HAL_ICACHE_GET_LAST_ERRORS == 1)
#define HAL_ICACHE_ERROR_NONE              0U          /*!< No error                */
#define HAL_ICACHE_ERROR_WRITE_INTRUSION   1U          /*!< Write access in executable cacheable region */
#endif /* USE_HAL_ICACHE_GET_LAST_ERRORS */
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup ICACHE_Exported_Functions HAL ICACHE Functions
  * @{
  */

/** @defgroup ICACHE_Exported_Functions_Group1 Initialization and Deinitialization functions
  * @{
  */
hal_status_t HAL_ICACHE_Init(hal_icache_handle_t *hicache, hal_icache_t instance);
void HAL_ICACHE_DeInit(hal_icache_handle_t *hicache);
/**
  * @}
  */


/** @defgroup ICACHE_Exported_Functions_Group2 Configuration functions
  * @{
  */
hal_status_t HAL_ICACHE_SetAssociativityMode(hal_icache_handle_t *hicache, hal_icache_associativity_t mode);
hal_icache_associativity_t HAL_ICACHE_GetAssociativityMode(const hal_icache_handle_t *hicache);
hal_status_t HAL_ICACHE_SetConfigRemapRegion(hal_icache_handle_t *hicache, hal_icache_region_t region,
                                             const hal_icache_region_config_t *p_region_config);
void HAL_ICACHE_GetConfigRemapRegion(const hal_icache_handle_t *hicache, hal_icache_region_t region,
                                     hal_icache_region_config_t *p_region_config);
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group3 Control functions
  * @{
  */
hal_status_t HAL_ICACHE_Start(hal_icache_handle_t *hicache, uint32_t interrupts);
hal_status_t HAL_ICACHE_Stop(hal_icache_handle_t *hicache);
hal_status_t HAL_ICACHE_EnableRemapRegion(hal_icache_handle_t *hicache, hal_icache_region_t region);
hal_status_t HAL_ICACHE_DisableRemapRegion(hal_icache_handle_t *hicache, hal_icache_region_t region);
hal_icache_remap_region_status_t HAL_ICACHE_IsEnabledRemapRegion(const hal_icache_handle_t *hicache,
                                                                 hal_icache_region_t region);
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group5 Monitoring functions
  * @{
  */
hal_status_t HAL_ICACHE_EnableMonitors(hal_icache_handle_t *hicache, uint32_t monitor_type);
hal_status_t HAL_ICACHE_DisableMonitors(hal_icache_handle_t *hicache, uint32_t monitor_type);
hal_status_t HAL_ICACHE_ResetMonitors(hal_icache_handle_t *hicache, uint32_t monitor_type);
uint32_t HAL_ICACHE_GetMonitorHitValue(const hal_icache_handle_t *hicache);
uint32_t HAL_ICACHE_GetMonitorMissValue(const hal_icache_handle_t *hicache);
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group6 Maintenance operation functions
  * @{
  */
hal_status_t HAL_ICACHE_Invalidate(hal_icache_handle_t *hicache);
hal_status_t HAL_ICACHE_Invalidate_IT(hal_icache_handle_t *hicache);
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group7 IRQ and callback functions
  * @{
  */
void HAL_ICACHE_IRQHandler(hal_icache_handle_t *hicache);
void HAL_ICACHE_ErrorCallback(hal_icache_handle_t *hicache);
void HAL_ICACHE_InvalidateCompleteCallback(hal_icache_handle_t *hicache);

#if defined (USE_HAL_ICACHE_REGISTER_CALLBACKS) && (USE_HAL_ICACHE_REGISTER_CALLBACKS == 1)
hal_status_t HAL_ICACHE_RegisterErrorCallback(hal_icache_handle_t *hicache, hal_icache_cb_t p_callback);
hal_status_t HAL_ICACHE_RegisterInvalidateCompleteCallback(hal_icache_handle_t *hicache, hal_icache_cb_t p_callback);
#endif /* USE_HAL_ICACHE_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group8 State function
  * @{
  */
hal_icache_state_t HAL_ICACHE_GetState(const hal_icache_handle_t *hicache);
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group9 Error function
  * @{
  */
#if defined(USE_HAL_ICACHE_GET_LAST_ERRORS) && (USE_HAL_ICACHE_GET_LAST_ERRORS == 1)
uint32_t HAL_ICACHE_GetLastErrorCodes(const hal_icache_handle_t *hicache);
#endif /* USE_HAL_ICACHE_GET_LAST_ERRORS */
/**
  * @}
  */

/** @defgroup ICACHE_Exported_Functions_Group10 Accessor and Instance functions
  * @{
  */
#if defined(USE_HAL_ICACHE_USER_DATA) && (USE_HAL_ICACHE_USER_DATA == 1)
void HAL_ICACHE_SetUserData(hal_icache_handle_t *hicache, const void *p_user_data);
const void *HAL_ICACHE_GetUserData(const hal_icache_handle_t *hicache);
#endif /* USE_HAL_ICACHE_USER_DATA */

hal_icache_t HAL_ICACHE_GetInstance(const hal_icache_handle_t *hicache);
ICACHE_TypeDef *HAL_ICACHE_GetLLInstance(const hal_icache_handle_t *hicache);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* ICACHE */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_ICACHE_H */
