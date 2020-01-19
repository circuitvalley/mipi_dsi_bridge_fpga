/* Created by plibgen $Revision: 1.31 $ */

#ifndef _WDT_P32MZ2048EFM064_H
#define _WDT_P32MZ2048EFM064_H

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

    WDT_ID_0 = 0,
    WDT_NUMBER_OF_MODULES = 1

} WDT_MODULE_ID;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/wdt_EnableControl_Default.h"
#include "../templates/wdt_WindowEnable_Default.h"
#include "../templates/wdt_TimerClear_WithKey.h"
#include "../templates/wdt_PostscalerValue_Default_1.h"
#include "../templates/wdt_SleepModePostscalerValue_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_WDT_ExistsEnableControl(WDT_MODULE_ID index)
{
     return WDT_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_WDT_Enable(WDT_MODULE_ID index)
{
     WDT_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_WDT_Disable(WDT_MODULE_ID index)
{
     WDT_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_WDT_IsEnabled(WDT_MODULE_ID index)
{
     return WDT_IsEnabled_Default(index);
}

PLIB_INLINE_API bool PLIB_WDT_ExistsWindowEnable(WDT_MODULE_ID index)
{
     return WDT_ExistsWindowEnable_Default(index);
}

PLIB_INLINE_API void PLIB_WDT_WindowEnable(WDT_MODULE_ID index)
{
     WDT_WindowEnable_Default(index);
}

PLIB_INLINE_API void PLIB_WDT_WindowDisable(WDT_MODULE_ID index)
{
     WDT_WindowDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_WDT_ExistsTimerClear(WDT_MODULE_ID index)
{
     return WDT_ExistsTimerClear_WithKey(index);
}

PLIB_INLINE_API void PLIB_WDT_TimerClear(WDT_MODULE_ID index)
{
     WDT_TimerClear_WithKey(index);
}

PLIB_INLINE_API bool PLIB_WDT_ExistsPostscalerValue(WDT_MODULE_ID index)
{
     return WDT_ExistsPostscalerValue_Default_1(index);
}

PLIB_INLINE_API char PLIB_WDT_PostscalerValueGet(WDT_MODULE_ID index)
{
     return WDT_PostscalerValueGet_Default_1(index);
}

PLIB_INLINE_API bool PLIB_WDT_ExistsSleepModePostscalerValue(WDT_MODULE_ID index)
{
     return WDT_ExistsSleepModePostscalerValue_Unsupported(index);
}

PLIB_INLINE_API char _PLIB_UNSUPPORTED PLIB_WDT_SleepModePostscalerValueGet(WDT_MODULE_ID index)
{
     return WDT_SleepModePostscalerValueGet_Unsupported(index);
}

#endif
