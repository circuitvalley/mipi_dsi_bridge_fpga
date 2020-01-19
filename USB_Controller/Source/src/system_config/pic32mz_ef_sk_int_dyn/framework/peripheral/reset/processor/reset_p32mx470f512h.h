/* Created by plibgen $Revision: 1.31 $ */

#ifndef _RESET_P32MX470F512H_H
#define _RESET_P32MX470F512H_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    RESET_ID_0 = 0,
    RESET_NUMBER_OF_MODULES = 1

} RESET_MODULE_ID;

typedef enum {

    RESET_REASON_NONE = 0x00000000,
    RESET_REASON_POWERON = 0x00000003,
    RESET_REASON_BROWNOUT = 0x00000002,
    RESET_REASON_WDT_TIMEOUT = 0x00000010,
    RESET_REASON_SOFTWARE = 0x00000040,
    RESET_REASON_MCLR = 0x00000080,
    RESET_REASON_CONFIG_MISMATCH = 0x00000200,
    RESET_REASON_HIGH_VOLTAGE_DETECT = 0x20000000,
    RESET_REASON_ALL = 0x200002D3

} RESET_REASON;

typedef enum {

    RESET_CONFIG_REG_READ_ERROR_NONE

} RESET_CONFIG_REG_READ_ERROR;

typedef enum {

    RESET_NMI_REASON_NONE

} RESET_NMI_REASON;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/reset_ResetReasonStatus_MX_1.h"
#include "../templates/reset_SoftwareResetTrigger_Default.h"
#include "../templates/reset_ConfigRegReadError_Unsupported.h"
#include "../templates/reset_NmiControl_Unsupported.h"
#include "../templates/reset_WdtoInSleep_Unsupported.h"
#include "../templates/reset_NmiCounter_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_RESET_ExistsResetReasonStatus(RESET_MODULE_ID index)
{
     return RESET_ExistsResetReasonStatus_MX_1(index);
}

PLIB_INLINE_API RESET_REASON PLIB_RESET_ReasonGet(RESET_MODULE_ID index)
{
     return RESET_ReasonGet_MX_1(index);
}

PLIB_INLINE_API void PLIB_RESET_ReasonClear(RESET_MODULE_ID index, RESET_REASON reason)
{
     RESET_ReasonClear_MX_1(index, reason);
}

PLIB_INLINE_API bool PLIB_RESET_ExistsSoftwareResetTrigger(RESET_MODULE_ID index)
{
     return RESET_ExistsSoftwareResetTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_RESET_SoftwareResetEnable(RESET_MODULE_ID index)
{
     RESET_SoftwareResetEnable_Default(index);
}

PLIB_INLINE_API bool PLIB_RESET_ExistsConfigRegReadError(RESET_MODULE_ID index)
{
     return RESET_ExistsConfigRegReadError_Unsupported(index);
}

PLIB_INLINE_API RESET_CONFIG_REG_READ_ERROR _PLIB_UNSUPPORTED PLIB_RESET_ConfigRegReadErrorGet(RESET_MODULE_ID index)
{
     return RESET_ConfigRegReadErrorGet_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_RESET_ExistsNmiControl(RESET_MODULE_ID index)
{
     return RESET_ExistsNmiControl_Unsupported(index);
}

PLIB_INLINE_API RESET_NMI_REASON _PLIB_UNSUPPORTED PLIB_RESET_NmiReasonGet(RESET_MODULE_ID index)
{
     return RESET_NmiReasonGet_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_RESET_NmiEventTrigger(RESET_MODULE_ID index, RESET_NMI_REASON nmi_reason)
{
     RESET_NmiEventTrigger_Unsupported(index, nmi_reason);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_RESET_NmiEventClear(RESET_MODULE_ID index, RESET_NMI_REASON nmi_reason)
{
     RESET_NmiEventClear_Unsupported(index, nmi_reason);
}

PLIB_INLINE_API bool PLIB_RESET_ExistsWdtoInSleep(RESET_MODULE_ID index)
{
     return RESET_ExistsWdtoInSleep_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_RESET_WdtTimeOutHasOccurredInSleep(RESET_MODULE_ID index)
{
     return RESET_WdtTimeOutHasOccurredInSleep_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_RESET_ExistsNmiCounter(RESET_MODULE_ID index)
{
     return RESET_ExistsNmiCounter_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_RESET_NmiCounterValueSet(RESET_MODULE_ID index, RESET_NMI_COUNT_TYPE nmi_count)
{
     RESET_NmiCounterValueSet_Unsupported(index, nmi_count);
}

PLIB_INLINE_API RESET_NMI_COUNT_TYPE _PLIB_UNSUPPORTED PLIB_RESET_NmiCounterValueGet(RESET_MODULE_ID index)
{
     return RESET_NmiCounterValueGet_Unsupported(index);
}

#endif
