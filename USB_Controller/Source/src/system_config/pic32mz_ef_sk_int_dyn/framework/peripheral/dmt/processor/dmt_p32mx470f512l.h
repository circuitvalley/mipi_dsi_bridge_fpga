/* Created by plibgen $Revision: 1.31 $ */

#ifndef _DMT_P32MX470F512L_H
#define _DMT_P32MX470F512L_H

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

    DMT_NUMBER_OF_MODULES = 0

} DMT_MODULE_ID;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMT_Enable(DMT_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMT_Disable(DMT_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_DMT_IsEnabled(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_DMT_ExistsEnableControl(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_DMT_WindowIsOpen(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_DMT_EventOccurred(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_DMT_BAD2Get(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_DMT_BAD1Get(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStatus(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMT_ClearStep1(DMT_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStep1(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMT_ClearStep2(DMT_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_DMT_ExistsStep2(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_DMT_CounterGet(DMT_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_DMT_ExistsCounter(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_DMT_PostscalerValueGet(DMT_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_DMT_ExistsPostscalerValue(DMT_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_DMT_PostscalerIntervalGet(DMT_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_DMT_ExistsPostscalerInterval(DMT_MODULE_ID index)
{
     return (bool)0;
}

#endif
