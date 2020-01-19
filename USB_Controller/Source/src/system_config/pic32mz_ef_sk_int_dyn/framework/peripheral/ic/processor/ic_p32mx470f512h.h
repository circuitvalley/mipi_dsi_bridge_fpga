/* Created by plibgen $Revision: 1.31 $ */

#ifndef _IC_P32MX470F512H_H
#define _IC_P32MX470F512H_H

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

    IC_ID_1 = _ICAP1_BASE_ADDRESS,
    IC_ID_2 = _ICAP2_BASE_ADDRESS,
    IC_ID_3 = _ICAP3_BASE_ADDRESS,
    IC_ID_4 = _ICAP4_BASE_ADDRESS,
    IC_ID_5 = _ICAP5_BASE_ADDRESS,
    IC_NUMBER_OF_MODULES = 5

} IC_MODULE_ID;

typedef enum {

    IC_INPUT_CAPTURE_DISABLE_MODE = 0,
    IC_INPUT_CAPTURE_EDGE_DETECT_MODE = 1,
    IC_INPUT_CAPTURE_FALLING_EDGE_MODE = 2,
    IC_INPUT_CAPTURE_RISING_EDGE_MODE = 3,
    IC_INPUT_CAPTURE_EVERY_4TH_EDGE_MODE = 4,
    IC_INPUT_CAPTURE_EVERY_16TH_EDGE_MODE = 5,
    IC_INPUT_CAPTURE_EVERY_EDGE_MODE = 6,
    IC_INPUT_CAPTURE_INTERRUPT_MODE = 7

} IC_INPUT_CAPTURE_MODES;

typedef enum {

    IC_BUFFER_SIZE_16BIT = 0,
    IC_BUFFER_SIZE_32BIT = 1

} IC_BUFFER_SIZE;

typedef enum {

    IC_EDGE_FALLING = 0,
    IC_EDGE_RISING = 1

} IC_EDGE_TYPES;

typedef enum {

    IC_TIMER_TMR3 = 0,
    IC_TIMER_TMR2 = 1

} IC_TIMERS;

typedef enum {

    IC_ALT_TIMERS_NONE

} IC_ALT_TIMERS;

typedef enum {

    IC_INTERRUPT_ON_EVERY_CAPTURE_EVENT = 0,
    IC_INTERRUPT_ON_EVERY_2ND_CAPTURE_EVENT = 1,
    IC_INTERRUPT_ON_EVERY_3RD_CAPTURE_EVENT = 2,
    IC_INTERRUPT_ON_EVERY_4TH_CAPTURE_EVENT = 3

} IC_EVENTS_PER_INTERRUPT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ic_EnableControl_Default.h"
#include "../templates/ic_StopInIdle_Default.h"
#include "../templates/ic_EdgeCapture_Default.h"
#include "../templates/ic_EventsPerInterruptSelect_Default.h"
#include "../templates/ic_BufferValue_32Bit_Variant.h"
#include "../templates/ic_BufferIsEmptyStatus_Default.h"
#include "../templates/ic_BufferOverflowStatus_Default.h"
#include "../templates/ic_CaptureMode_Default.h"
#include "../templates/ic_BufferSize_Default.h"
#include "../templates/ic_TimerSelect_Default.h"
#include "../templates/ic_AlternateClock_Unsupported.h"
#include "../templates/ic_AlternateTimerSelect_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_IC_Enable(IC_MODULE_ID index)
{
     IC_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_IC_Disable(IC_MODULE_ID index)
{
     IC_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsEnable(IC_MODULE_ID index)
{
     return IC_ExistsEnable_Default(index);
}

PLIB_INLINE_API void PLIB_IC_StopInIdleEnable(IC_MODULE_ID index)
{
     IC_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_IC_StopInIdleDisable(IC_MODULE_ID index)
{
     IC_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsStopInIdle(IC_MODULE_ID index)
{
     return IC_ExistsStopInIdle_Default(index);
}

PLIB_INLINE_API void PLIB_IC_FirstCaptureEdgeSelect(IC_MODULE_ID index, IC_EDGE_TYPES edgeType)
{
     IC_FirstCaptureEdgeSelect_Default(index, edgeType);
}

PLIB_INLINE_API bool PLIB_IC_ExistsEdgeCapture(IC_MODULE_ID index)
{
     return IC_ExistsEdgeCapture_Default(index);
}

PLIB_INLINE_API void PLIB_IC_EventsPerInterruptSelect(IC_MODULE_ID index, IC_EVENTS_PER_INTERRUPT event)
{
     IC_EventsPerInterruptSelect_Default(index, event);
}

PLIB_INLINE_API bool PLIB_IC_ExistsEventsPerInterruptSelect(IC_MODULE_ID index)
{
     return IC_ExistsEventsPerInterruptSelect_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_IC_Buffer32BitGet(IC_MODULE_ID index)
{
     return IC_Buffer32BitGet_32Bit_Variant(index);
}

PLIB_INLINE_API uint16_t PLIB_IC_Buffer16BitGet(IC_MODULE_ID index)
{
     return IC_Buffer16BitGet_32Bit_Variant(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsBufferValue(IC_MODULE_ID index)
{
     return IC_ExistsBufferValue_32Bit_Variant(index);
}

PLIB_INLINE_API bool PLIB_IC_BufferIsEmpty(IC_MODULE_ID index)
{
     return IC_BufferIsEmpty_Default(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsBufferIsEmptyStatus(IC_MODULE_ID index)
{
     return IC_ExistsBufferIsEmptyStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_IC_BufferOverflowHasOccurred(IC_MODULE_ID index)
{
     return IC_BufferOverflowHasOccurred_Default(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsBufferOverflowStatus(IC_MODULE_ID index)
{
     return IC_ExistsBufferOverflowStatus_Default(index);
}

PLIB_INLINE_API void PLIB_IC_ModeSelect(IC_MODULE_ID index, IC_INPUT_CAPTURE_MODES modeSel)
{
     IC_ModeSelect_Default(index, modeSel);
}

PLIB_INLINE_API bool PLIB_IC_ExistsCaptureMode(IC_MODULE_ID index)
{
     return IC_ExistsCaptureMode_Default(index);
}

PLIB_INLINE_API void PLIB_IC_BufferSizeSelect(IC_MODULE_ID index, IC_BUFFER_SIZE bufSize)
{
     IC_BufferSizeSelect_Default(index, bufSize);
}

PLIB_INLINE_API bool PLIB_IC_ExistsBufferSize(IC_MODULE_ID index)
{
     return IC_ExistsBufferSize_Default(index);
}

PLIB_INLINE_API void PLIB_IC_TimerSelect(IC_MODULE_ID index, IC_TIMERS tmr)
{
     IC_TimerSelect_Default(index, tmr);
}

PLIB_INLINE_API bool PLIB_IC_ExistsTimerSelect(IC_MODULE_ID index)
{
     return IC_ExistsTimerSelect_Default(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_IC_AlternateClockEnable(IC_MODULE_ID index)
{
     IC_AlternateClockEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_IC_AlternateClockDisable(IC_MODULE_ID index)
{
     IC_AlternateClockDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_IC_ExistsAlternateClock(IC_MODULE_ID index)
{
     return IC_ExistsAlternateClock_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_IC_AlternateTimerSelect(IC_MODULE_ID index, IC_ALT_TIMERS tmr)
{
     return IC_AlternateTimerSelect_Unsupported(index, tmr);
}

PLIB_INLINE_API bool PLIB_IC_ExistsAlternateTimerSelect(IC_MODULE_ID index)
{
     return IC_ExistsAlternateTimerSelect_Unsupported(index);
}

#endif
