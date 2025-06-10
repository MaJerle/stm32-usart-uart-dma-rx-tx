#ifndef _GENERIC_STATUS_H
#define _GENERIC_STATUS_H

#include "basic_types.h"

/*!***************************************************************************
 * @defgroup    status_t Status Codes
 *
 * @brief       Status and Error Code Definitions
 *
 * @details     Defines status and error codes for function return values.
 *              Basic status number structure:
 *              - 0 is OK or no error.
 *              - negative values determine errors.
 *              - positive values determine warnings or status information.
 *              .
 *
 * @addtogroup  status_t
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief   Type used for all status and error return values.
 * @details Basic status number structure:
 *           - 0 is OK or no error.
 *           - negative values determine errors.
 *           - positive values determine warnings or status information.
 *           .
 *****************************************************************************/

typedef i8 status_t;

/*! API status and error return codes. */
enum Status
{
    /**********************************************************************************************
     ********** Generic Status ********************************************************************
     *********************************************************************************************/

    /*! 0: Status for success/no error. */
    STATUS_OK = 0,

    /*! 0: Status for device/module/hardware idle. Implies #STATUS_OK. */
    STATUS_IDLE = 0,

    /*! 1: Status to be ignored. */
    STATUS_IGNORE = 1,

    /*! 2: Status for device/module/hardware busy. */
    STATUS_BUSY = 2,

    /*! 3: Status for device/module/hardware is currently initializing. */
    STATUS_INITIALIZING = 3,

    /*! -1: Error for generic fail/error. */
    ERROR_FAIL = -1,

    /*! -2: Error for process aborted by user/external. */
    ERROR_ABORTED = -2,

    /*! -3: Error for invalid read only operations. */
    ERROR_READ_ONLY = -3,

    /*! -4: Error for out of range parameters. */
    ERROR_OUT_OF_RANGE = -4,

    /*! -5: Error for invalid argument passed to an function. */
    ERROR_INVALID_ARGUMENT = -5,

    /*! -6: Error for timeout occurred. */
    ERROR_TIMEOUT = -6,

    /*! -7: Error for not initialized modules. */
    ERROR_NOT_INITIALIZED = -7,

    /*! -8: Error for not supported. */
    ERROR_NOT_SUPPORTED = -8,

    /*! -9: Error for yet not implemented functions. */
    ERROR_NOT_IMPLEMENTED = -9,
};

/*! @} */

#endif /* _GENERIC_STATUS_H */
