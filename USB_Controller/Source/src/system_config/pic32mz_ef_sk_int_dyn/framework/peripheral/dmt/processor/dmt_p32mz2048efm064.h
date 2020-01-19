/* Created by plibgen $Revision: 1.31 $ */

#ifndef _DMT_P32MZ2048EFM064_H
#define _DMT_P32MZ2048EFM064_H

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

    DMT_ID_0 = _DMT_BASE_ADDRESS,
    DMT_NUMBER_OF_MODULES = 1

} DMT_MODULE_ID;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/dmt_Enable_Default.h"
#include "../templates/dmt_DeadManTimerStatus_Default.h"
#include "../templates/dmt_Step1_Default.h"
#include "../templates/dmt_Step2_Default.h"
#include "../templates/dmt_Counter_Default.h"
#include "../templates/dmt_PostscalerValue_Default.h"
#include "../templates/dmt_PostscalerInterval_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_DMT_Enable(DMT_MODULE_ID index)
{
     DMT_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_DMT_Disable(DMT_MODULE_ID index)
{
     DMT_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_IsEnabled(DMT_MODULE_ID index)
{
     return DMT_IsEnabled_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsEnableControl(DMT_MODULE_ID index)
{
     return DMT_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_WindowIsOpen(DMT_MODULE_ID index)
{
     return DMT_WindowIsOpen_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_EventOccurred(DMT_MODULE_ID index)
{
     return DMT_EventOccurred_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_BAD2Get(DMT_MODULE_ID index)
{
     return DMT_BAD2Get_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_BAD1Get(DMT_MODULE_ID index)
{
     return DMT_BAD1Get_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStatus(DMT_MODULE_ID index)
{
     return DMT_ExistsStatus_Default(index);
}

PLIB_INLINE_API void PLIB_DMT_ClearStep1(DMT_MODULE_ID index)
{
     DMT_ClearStep1_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStep1(DMT_MODULE_ID index)
{
     return DMT_ExistsStep1_Default(index);
}

PLIB_INLINE_API void PLIB_DMT_ClearStep2(DMT_MODULE_ID index)
{
     DMT_ClearStep2_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStep2(DMT_MODULE_ID index)
{
     return DMT_ExistsStep2_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_DMT_CounterGet(DMT_MODULE_ID index)
{
     return DMT_CounterGet_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsCounter(DMT_MODULE_ID index)
{
     return DMT_ExistsCounter_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_DMT_PostscalerValueGet(DMT_MODULE_ID index)
{
     return DMT_PostscalerValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsPostscalerValue(DMT_MODULE_ID index)
{
     return DMT_ExistsPostscalerValue_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_DMT_PostscalerIntervalGet(DMT_MODULE_ID index)
{
     return DMT_PostscalerIntervalGet_Default(index);
}

PLIB_INLINE_API bool PLIB_DMT_ExistsPostscalerInterval(DMT_MODULE_ID index)
{
     return DMT_ExistsPostscalerInterval_Default(index);
}

#endif
