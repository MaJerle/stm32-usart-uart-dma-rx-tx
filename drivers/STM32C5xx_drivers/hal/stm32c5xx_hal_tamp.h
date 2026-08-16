/**
  ******************************************************************************
  * @file    stm32c5xx_hal_tamp.h
  * @brief   Header file of TAMP HAL module.
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
#ifndef STM32C5XX_HAL_TAMP_H
#define STM32C5XX_HAL_TAMP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal_def.h"
#include "stm32c5xx_ll_tamp.h"


/** @addtogroup STM32C5xx_HAL_Driver
  * @{
  */

/** @defgroup TAMP TAMP
  * @{
  */

/* Exported enumerations --------------------------------------------------------*/
/** @defgroup TAMP_Exported_Enums HAL TAMP Enumerations
  * @{
  */

/** @defgroup TAMP_Exported_Enums_Passive HAL TAMP passive tamper enumerations
  * @{
  */

/**
  * @brief Passive tampers detection of edge level definition.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_TRIGGER_RISING  = LL_TAMP_ACTIVELEVEL_DEACTIVATE_ALL, /*!< Rising edge triggers a tamper detection  */
  HAL_TAMP_PASSIVE_TRIGGER_FALLING = LL_TAMP_ACTIVELEVEL_TAMP,           /*!< Falling edge triggers a tamper detection */
  HAL_TAMP_PASSIVE_TRIGGER_LOW     = LL_TAMP_ACTIVELEVEL_DEACTIVATE_ALL, /*!< Low level triggers a tamper detection */
  HAL_TAMP_PASSIVE_TRIGGER_HIGH    = LL_TAMP_ACTIVELEVEL_TAMP,           /*!< High level triggers a tamper detection */
} hal_tamp_passive_trigger_t;

/**
  * @brief Passive tampers erase mode definitions.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_SECRETS_ERASE    = LL_TAMP_ERASE_ACTIVATE_ALL, /*!< Confirmed mode: device secrets are erased when a tamper is detected */
  HAL_TAMP_PASSIVE_SECRETS_NO_ERASE = LL_TAMP_NOERASE_TAMPER      /*!< Potential mode: some device secrets are not erased but blocked when a tamper is detected */
} hal_tamp_passive_secrets_erase_t;

/**
  * @brief Passive tampers masked definition.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_UNMASKED = LL_TAMP_MASK_NONE,  /*!< Tampers are not masked and behave as expected */
  HAL_TAMP_PASSIVE_MASKED   = LL_TAMP_MASK_TAMPER /*!< Tampers are masked: a tamper detection does not trigger */
} hal_tamp_passive_mask_t;

/**
  * @brief Passive tampers pull-up (precharge) definition.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_DISABLE = LL_TAMP_PULL_UP_DISABLE, /*!< Disable tamper pull-up (precharge) */
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_ENABLE  = LL_TAMP_PULL_UP_ENABLE,  /*!< Enable tamper pull-up (precharge) */
} hal_tamp_passive_pull_up_precharge_state_t;

/**
  * @brief Passive tampers pull-up precharge duration definition.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_1_RTCCLK = LL_TAMP_DURATION_1RTCCLK, /*!< Tamper pins are precharged before sampling during 1 RTCCLK cycle */
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_2_RTCCLK = LL_TAMP_DURATION_2RTCCLK, /*!< Tamper pins are precharged before sampling during 2 RTCCLK cycle */
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_4_RTCCLK = LL_TAMP_DURATION_4RTCCLK, /*!< Tamper pins are precharged before sampling during 4 RTCCLK cycle */
  HAL_TAMP_PASSIVE_PULL_UP_PRECHARGE_8_RTCCLK = LL_TAMP_DURATION_8RTCCLK  /*!< Tamper pins are precharged before sampling during 8 RTCCLK cycle */
} hal_tamp_passive_pull_up_precharge_duration_t;

/**
  * @brief Passive tampers filter activation definition.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_FILTER_DISABLE   = LL_TAMP_FILTER_DISABLE,  /*!< Tamper filter is disabled */
  HAL_TAMP_PASSIVE_FILTER_2_SAMPLES = LL_TAMP_FILTER_2SAMPLES, /*!< Tamper is activated after 2 consecutive samples at the active level */
  HAL_TAMP_PASSIVE_FILTER_4_SAMPLES = LL_TAMP_FILTER_4SAMPLES, /*!< Tamper is activated after 4 consecutive samples at the active level */
  HAL_TAMP_PASSIVE_FILTER_8_SAMPLES = LL_TAMP_FILTER_8SAMPLES  /*!< Tamper is activated after 8 consecutive samples at the active level */
} hal_tamp_passive_filter_t;

/**
  * @brief Passive tampers sampling frequency definitions.
  */
typedef enum
{
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_256   = LL_TAMP_SAMPLFREQDIV_256,   /*!< Tampers pins are sampled at (RTCCLK / 256) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_512   = LL_TAMP_SAMPLFREQDIV_512,   /*!< Tampers pins are sampled at (RTCCLK / 512) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_1024  = LL_TAMP_SAMPLFREQDIV_1024,  /*!< Tampers pins are sampled at (RTCCLK / 1024) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_2048  = LL_TAMP_SAMPLFREQDIV_2048,  /*!< Tampers pins are sampled at (RTCCLK / 2048) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_4096  = LL_TAMP_SAMPLFREQDIV_4096,  /*!< Tampers pins are sampled at (RTCCLK / 4096) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_8192  = LL_TAMP_SAMPLFREQDIV_8192,  /*!< Tampers pins are sampled at (RTCCLK / 8192) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_16384 = LL_TAMP_SAMPLFREQDIV_16384, /*!< Tampers pins are sampled at (RTCCLK / 16384) Hz */
  HAL_TAMP_PASSIVE_SAMPLE_FREQ_DIV_32768 = LL_TAMP_SAMPLFREQDIV_32768, /*!< Tampers pins are sampled at (RTCCLK / 32768) Hz */
} hal_tamp_passive_sample_frequency_t;
/**
  * @}
  */

/** @defgroup TAMP_Exported_Enums_Internal HAL TAMP internal tampers enumerations
  * @{
  */

/**
  * @brief Internal tampers erase mode definitions.
  */
typedef enum
{
  HAL_TAMP_INTERNAL_SECRETS_ERASE    = LL_TAMP_ITAMP_ERASE_ACTIVE_ALL, /*!< Confirmed mode: device secrets are erased when a tamper is detected */
  HAL_TAMP_INTERNAL_SECRETS_NO_ERASE = LL_TAMP_ITAMP_NOERASE_TAMPER    /*!< Potential mode: some device secrets are not erased but blocked when a tamper is detected */
} hal_tamp_internal_secrets_erase_t;
/**
  * @}
  */

/** @defgroup TAMP_Exported_Enums_Device_Secrets HAL TAMP secrets enumerations
  * @{
  */

/**
  * @brief Device secrets access status definitions.
  */
typedef enum
{
  HAL_TAMP_SECRETS_ACCESS_UNBLOCKED = 0U, /*!< Device secrets can be accessed if no tamper flag is set. */
  HAL_TAMP_SECRETS_ACCESS_BLOCKED   = 1U  /*!< Device secrets cannot be accessed. */
} hal_tamp_secrets_status_t;
/**
  * @}
  */

/** @defgroup TAMP_Exported_Enums_Remap HAL TAMP remap enumerations
  * @{
  */

/**
  * @brief Remap tamp pin status definitions.
  */
typedef enum
{
  HAL_TAMP_REMAP_TAMP_PIN_DISABLED = 0U,  /*!< Remap is disabled. */
  HAL_TAMP_REMAP_TAMP_PIN_ENABLED  = 1U   /*!< Remap is enabled. */
} hal_tamp_remap_status_t;
/**
  * @}
  */

/*! HAL TAMP Privilege attribute enumeration definition */
typedef enum
{
  HAL_TAMP_NPRIV = LL_TAMP_ATTR_NPRIV, /*!< TAMP Non-privileged attribute */
  HAL_TAMP_PRIV  = LL_TAMP_ATTR_PRIV   /*!< TAMP privileged attribute */
} hal_tamp_priv_attr_t;
/**
  * @}
  */

/* Exported Unions ----------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/** @defgroup TAMP_Exported_Defines HAL TAMP Defines
  * @{
  */

/** @defgroup TAMP_Exported_Defines_Tamper TAMP tamper defines for passive and active.
  * @{
  */
#define HAL_TAMP_TAMPER_1    LL_TAMP_1   /*!< Tamper input 1 */
#define HAL_TAMP_TAMPER_2    LL_TAMP_2   /*!< Tamper input 2 */
#define HAL_TAMP_TAMPER_3    LL_TAMP_3   /*!< Tamper input 3 */
#define HAL_TAMP_TAMPER_ALL  LL_TAMP_ALL /*!< All tampers inputs */
/**
  * @}
  */

/** @defgroup TAMP_Exported_Defines_Internal_Tamper TAMP internal tamper defines.
  * @{
  */
#define HAL_TAMP_INTERNAL_TAMPER_3   LL_TAMP_ITAMP3    /*!< Internal tamper 3: LSE monitoring */
#define HAL_TAMP_INTERNAL_TAMPER_4   LL_TAMP_ITAMP4    /*!< Internal tamper 4: HSE monitoring */
#define HAL_TAMP_INTERNAL_TAMPER_5   LL_TAMP_ITAMP5    /*!< Internal tamper 5: RTC calendar overflow */
#define HAL_TAMP_INTERNAL_TAMPER_6   LL_TAMP_ITAMP6    /*!< Internal tamper 6: Unexpected debug activation */
#define HAL_TAMP_INTERNAL_TAMPER_9   LL_TAMP_ITAMP9    /*!< Internal tamper 9: TRNG fault */
#define HAL_TAMP_INTERNAL_TAMPER_11  LL_TAMP_ITAMP11   /*!< Internal tamper 11: IWDG reset when tamper flag is set */
#define HAL_TAMP_INTERNAL_ALL        LL_TAMP_ITAMP_ALL /*!< All internal tampers inputs */
/**
  * @}
  */

/** @defgroup TAMP_Exported_Defines_Tamper_Interruption TAMP tamper interruption defines
  * @{
  */
#define HAL_TAMP_IT_NONE       LL_TAMP_IT_NONE       /*!< All external tampers interrupts are disabled */
#define HAL_TAMP_IT_TAMPER_1   LL_TAMP_IT_TAMPER_1   /*!< External tamper 1 interrupt is enabled */
#define HAL_TAMP_IT_TAMPER_2   LL_TAMP_IT_TAMPER_2   /*!< External tamper 2 interrupt is enabled */
#define HAL_TAMP_IT_TAMPER_3   LL_TAMP_IT_TAMPER_3   /*!< External tamper 3 interrupt is enabled */
#define HAL_TAMP_IT_ALL        LL_TAMP_IT_ALL        /*!< All external tampers interrupts are enabled */
/**
  * @}
  */

/** @defgroup TAMP_Exported_Defines_Internal_Tamper_Interruption TAMP internal tamper interruption defines
  * @{
  */
#define HAL_TAMP_INTERNAL_IT_NONE       LL_TAMP_INTERNAL_IT_NONE       /*!< All internal tampers interrupts are disabled */
#define HAL_TAMP_INTERNAL_IT_TAMPER_3   LL_TAMP_INTERNAL_IT_TAMPER_3   /*!< Internal tamper 3 interrupt is enabled  */
#define HAL_TAMP_INTERNAL_IT_TAMPER_4   LL_TAMP_INTERNAL_IT_TAMPER_4   /*!< Internal tamper 4 interrupt is enabled  */
#define HAL_TAMP_INTERNAL_IT_TAMPER_5   LL_TAMP_INTERNAL_IT_TAMPER_5   /*!< Internal tamper 5 interrupt is enabled  */
#define HAL_TAMP_INTERNAL_IT_TAMPER_6   LL_TAMP_INTERNAL_IT_TAMPER_6   /*!< Internal tamper 6 interrupt is enabled  */
#define HAL_TAMP_INTERNAL_IT_TAMPER_9   LL_TAMP_INTERNAL_IT_TAMPER_9   /*!< Internal tamper 9 interrupt is enabled  */
#define HAL_TAMP_INTERNAL_IT_TAMPER_11  LL_TAMP_INTERNAL_IT_TAMPER_11  /*!< Internal tamper 11 interrupt is enabled */
#define HAL_TAMP_INTERNAL_IT_ALL        LL_TAMP_INTERNAL_IT_ALL        /*!< All internal tampers interrupts are enabled */
/**
  * @}
  */

/** @defgroup TAMP_Exported_Defines_Remap_Tamper_Pin TAMP remap tamp pin defines.
  * @{
  */
#define HAL_TAMP_REMAP_TAMP_IN2_PA0_TO_PC1     LL_TAMP_RMP_TAMP_IN2_PA0_TO_PC1  /*!< Remap Tamper input 2 from PA0 to PC1 */
#define HAL_TAMP_REMAP_TAMP_IN3_PA1_TO_PA2     LL_TAMP_RMP_TAMP_IN3_PA1_TO_PA2  /*!< Remap Tamper input 3 from PA1 to PA2 */
/**
  * @}
  */


/** @defgroup TAMP_privilege_attributes_configuration_items TAMP privilege attributes configuration items
  * @{
  */
#define HAL_TAMP_PRIV_ITEM_TAMP          LL_TAMP_PRIV_ITEM_TAMPPRIV  /*!< Privilege attribute excluding backup registers */
#define HAL_TAMP_PRIV_ITEM_BACKUP_ZONE_1 LL_TAMP_PRIV_ZONE_BKPRWPRIV /*!< Privilege attribute of zone 1 */
#define HAL_TAMP_PRIV_ITEM_BACKUP_ZONE_2 LL_TAMP_PRIV_ZONE_BKPWPRIV  /*!< Privilege attribute of zone 2. Even in privileged mode, the non privileged code can still read zone 2 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported structures --------------------------------------------------------*/

/** @defgroup TAMP_Exported_Structures HAL TAMP Structures
  * @{
  */

/** @defgroup TAMP_Exported_Structures_Passive TAMP passive tampers exported structures
  * @{
  */

/**
  * @brief Passive tamper configuration.
  */
typedef struct
{
  hal_tamp_passive_pull_up_precharge_state_t    precharge;          /*!< Activation of the pull-up (precharge) */
  hal_tamp_passive_pull_up_precharge_duration_t precharge_duration; /*!< Duration of the precharge in RTCCLK units */
  hal_tamp_passive_filter_t                     type_activation;    /*!< Activation type of the tamper */
  hal_tamp_passive_sample_frequency_t           sample_frequency;   /*!< Tamper sample frequency */
} hal_tamp_passive_config_t;

/**
  * @brief Passive tamper individual configuration.
  */
typedef struct
{
  hal_tamp_passive_trigger_t       trigger;       /*!< Trigger type for edge and level tamper detection */
  hal_tamp_passive_secrets_erase_t erase_secrets; /*!< Tamper erase mode */
  hal_tamp_passive_mask_t          masked;        /*!< That the tamper is masked or not */

} hal_tamp_passive_individual_config_t;
/**
  * @}
  */

/** @defgroup TAMP_Exported_Structures_Internal TAMP internal tampers exported structures
  * @{
  */

/**
  * @brief Internal tamper individual configuration.
  */
typedef struct
{
  hal_tamp_internal_secrets_erase_t erase_secrets; /*!< Internal tamper erase mode */
} hal_tamp_internal_individual_config_t;
/**
  * @}
  */

/**
  * @brief Backup registers enumeration definition.
  */
typedef enum
{
  HAL_TAMP_BACKUP_REG_0  = LL_TAMP_BKP_DR0,  /*!< Backup register 0 */
  HAL_TAMP_BACKUP_REG_1  = LL_TAMP_BKP_DR1,  /*!< Backup register 1 */
  HAL_TAMP_BACKUP_REG_2  = LL_TAMP_BKP_DR2,  /*!< Backup register 2 */
  HAL_TAMP_BACKUP_REG_3  = LL_TAMP_BKP_DR3,  /*!< Backup register 3 */
  HAL_TAMP_BACKUP_REG_4  = LL_TAMP_BKP_DR4,  /*!< Backup register 4 */
  HAL_TAMP_BACKUP_REG_5  = LL_TAMP_BKP_DR5,  /*!< Backup register 5 */
  HAL_TAMP_BACKUP_REG_6  = LL_TAMP_BKP_DR6,  /*!< Backup register 6 */
  HAL_TAMP_BACKUP_REG_7  = LL_TAMP_BKP_DR7,  /*!< Backup register 7 */
  HAL_TAMP_BACKUP_REG_8  = LL_TAMP_BKP_DR8,  /*!< Backup register 8 */
  HAL_TAMP_BACKUP_REG_9  = LL_TAMP_BKP_DR9,  /*!< Backup register 9 */
  HAL_TAMP_BACKUP_REG_10 = LL_TAMP_BKP_DR10, /*!< Backup register 10 */
  HAL_TAMP_BACKUP_REG_11 = LL_TAMP_BKP_DR11, /*!< Backup register 11 */
  HAL_TAMP_BACKUP_REG_12 = LL_TAMP_BKP_DR12, /*!< Backup register 12 */
  HAL_TAMP_BACKUP_REG_13 = LL_TAMP_BKP_DR13, /*!< Backup register 13 */
  HAL_TAMP_BACKUP_REG_14 = LL_TAMP_BKP_DR14, /*!< Backup register 14 */
  HAL_TAMP_BACKUP_REG_15 = LL_TAMP_BKP_DR15, /*!< Backup register 15 */
  HAL_TAMP_BACKUP_REG_16 = LL_TAMP_BKP_DR16, /*!< Backup register 16 */
  HAL_TAMP_BACKUP_REG_17 = LL_TAMP_BKP_DR17, /*!< Backup register 17 */
  HAL_TAMP_BACKUP_REG_18 = LL_TAMP_BKP_DR18, /*!< Backup register 18 */
  HAL_TAMP_BACKUP_REG_19 = LL_TAMP_BKP_DR19, /*!< Backup register 19 */
  HAL_TAMP_BACKUP_REG_20 = LL_TAMP_BKP_DR20, /*!< Backup register 20 */
  HAL_TAMP_BACKUP_REG_21 = LL_TAMP_BKP_DR21, /*!< Backup register 21 */
  HAL_TAMP_BACKUP_REG_22 = LL_TAMP_BKP_DR22, /*!< Backup register 22 */
  HAL_TAMP_BACKUP_REG_23 = LL_TAMP_BKP_DR23, /*!< Backup register 23 */
  HAL_TAMP_BACKUP_REG_24 = LL_TAMP_BKP_DR24, /*!< Backup register 24 */
  HAL_TAMP_BACKUP_REG_25 = LL_TAMP_BKP_DR25, /*!< Backup register 25 */
  HAL_TAMP_BACKUP_REG_26 = LL_TAMP_BKP_DR26, /*!< Backup register 26 */
  HAL_TAMP_BACKUP_REG_27 = LL_TAMP_BKP_DR27, /*!< Backup register 27 */
  HAL_TAMP_BACKUP_REG_28 = LL_TAMP_BKP_DR28, /*!< Backup register 28 */
  HAL_TAMP_BACKUP_REG_29 = LL_TAMP_BKP_DR29, /*!< Backup register 29 */
  HAL_TAMP_BACKUP_REG_30 = LL_TAMP_BKP_DR30, /*!< Backup register 30 */
  HAL_TAMP_BACKUP_REG_31 = LL_TAMP_BKP_DR31  /*!< Backup register 31 */
} hal_tamp_backup_register_idx_t;
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup TAMP_Exported_Functions HAL TAMP Functions
  * @{
  */

/** @defgroup TAMP_Exported_Functions_Passive HAL TAMP passive tamper mode functions
  * @{
  */
hal_status_t HAL_TAMP_PASSIVE_SetConfig(const hal_tamp_passive_config_t *p_config);
void HAL_TAMP_PASSIVE_GetConfig(hal_tamp_passive_config_t *p_config);

hal_status_t HAL_TAMP_PASSIVE_SetConfigTampers(uint32_t tampers, const hal_tamp_passive_individual_config_t *p_config);
void HAL_TAMP_PASSIVE_GetConfigTamper(uint32_t tamper, hal_tamp_passive_individual_config_t *p_config);

hal_status_t HAL_TAMP_PASSIVE_Start(uint32_t tampers, uint32_t interruption);
hal_status_t HAL_TAMP_PASSIVE_Stop(uint32_t tampers);

hal_status_t HAL_TAMP_PASSIVE_PollForEvent(uint32_t tampers, uint32_t timeout_ms);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_Internal HAL TAMP internal tamper mode functions
  * @{
  */
hal_status_t HAL_TAMP_INTERNAL_SetConfigTampers(uint32_t internal_tampers,
                                                const hal_tamp_internal_individual_config_t *p_config);
void HAL_TAMP_INTERNAL_GetConfigTamper(uint32_t internal_tamper, hal_tamp_internal_individual_config_t *p_config);

hal_status_t HAL_TAMP_INTERNAL_Start(uint32_t internal_tampers, uint32_t interruption);
hal_status_t HAL_TAMP_INTERNAL_Stop(uint32_t internal_tampers);

hal_status_t HAL_TAMP_INTERNAL_PollForEvent(uint32_t internal_tampers, uint32_t timeout_ms);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_IRQ HAL TAMP IRQ functions
  * @{
  */
void HAL_TAMP_IRQHandler(void);
void HAL_TAMP_InternalTamperIRQHandler(void);
void HAL_TAMP_TamperIRQHandler(void);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_Callback HAL TAMP callback functions
  * @{
  */
void HAL_TAMP_InternalTamperEventCallback(uint32_t internal_tampers);
void HAL_TAMP_TamperEventCallback(uint32_t tampers);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_Device_Secrets HAL TAMP device secrets functions
  * @{
  */
hal_status_t HAL_TAMP_WriteBackupRegisterValue(hal_tamp_backup_register_idx_t backup_register_index,
                                               uint32_t data_32bit);
uint32_t HAL_TAMP_ReadBackupRegisterValue(hal_tamp_backup_register_idx_t backup_register_index);

hal_status_t HAL_TAMP_UnblockDeviceSecretsAccess(void);
hal_status_t HAL_TAMP_BlockDeviceSecretsAccess(void);
hal_tamp_secrets_status_t HAL_TAMP_IsBlockedDeviceSecretsAccess(void);
hal_status_t HAL_TAMP_EraseDeviceSecrets(void);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_Remap HAL TAMP remap functions
  * @{
  */
void HAL_TAMP_EnableRemap(uint32_t tamp_remap);
void HAL_TAMP_DisableRemap(uint32_t tamp_remap);
hal_tamp_remap_status_t HAL_TAMP_IsEnabledRemap(uint32_t tamp_remap);
/**
  * @}
  */

/** @defgroup TAMP_Exported_Functions_Attributes management functions
  * @{
  */
hal_status_t HAL_TAMP_SetPrivAttr(uint32_t item, hal_tamp_priv_attr_t priv_attr);
hal_tamp_priv_attr_t HAL_TAMP_GetPrivAttr(uint32_t item);

hal_status_t HAL_TAMP_SetBackupRegisterZones(uint32_t zone1_backup_register_nbr,
                                             uint32_t zone2_backup_register_nbr);
hal_status_t HAL_TAMP_GetBackupRegisterZones(uint32_t *p_zone1_backup_register_nbr,
                                             uint32_t *p_zone2_backup_register_nbr);
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
#endif  /* __cplusplus */

#endif /* STM32C5XX_HAL_TAMP_H */
