/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CPU_UTILS_HDR_H
#define CPU_UTILS_HDR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

#define CALCULATION_PERIOD    1000

uint16_t osGetCPUUsage (void);

#ifdef __cplusplus
}
#endif

#endif /* CPU_UTILS_HDR_H */
