/* Created by plibgen $Revision: 1.31 $ */

#ifndef _OC_P32MX470F512H_H
#define _OC_P32MX470F512H_H

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

    OC_ID_1 = _OCMP1_BASE_ADDRESS,
    OC_ID_2 = _OCMP2_BASE_ADDRESS,
    OC_ID_3 = _OCMP3_BASE_ADDRESS,
    OC_ID_4 = _OCMP4_BASE_ADDRESS,
    OC_ID_5 = _OCMP5_BASE_ADDRESS,
    OC_NUMBER_OF_MODULES = 5

} OC_MODULE_ID;

typedef enum {

    OC_COMPARE_TURN_OFF_MODE = 0,
    OC_SET_HIGH_SINGLE_PULSE_MODE = 1,
    OC_SET_LOW_SINGLE_PULSE_MODE = 2,
    OC_TOGGLE_CONTINUOUS_PULSE_MODE = 3,
    OC_DUAL_COMPARE_SINGLE_PULSE_MODE = 4,
    OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE = 5,
    OC_COMPARE_PWM_EDGE_ALIGNED_MODE = 6,
    OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION = 6,
    OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION = 7

} OC_COMPARE_MODES;

typedef enum {

    OC_BUFFER_SIZE_16BIT = 0,
    OC_BUFFER_SIZE_32BIT = 1

} OC_BUFFER_SIZE;

typedef enum {

    OC_TIMER_16BIT_TMR2 = 0,
    OC_TIMER_16BIT_TMR3 = 1

} OC_16BIT_TIMERS;

typedef enum {

    OC_ALT_TIMERS_NONE

} OC_ALT_TIMERS;

typedef enum {

    OC_FAULT_PRESET = 7,
    OC_FAULT_DISABLE = 6

} OC_FAULTS;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/oc_EnableControl_Default.h"
#include "../templates/oc_StopInIdle_Default.h"
#include "../templates/oc_FaultInput_Default.h"
#include "../templates/oc_FaultStatus_Default.h"
#include "../templates/oc_TimerSelect_Default.h"
#include "../templates/oc_AlternateClock_Unsupported.h"
#include "../templates/oc_AlternateTimerSelect_Unsupported.h"
#include "../templates/oc_BufferValue_32Bit_Variant.h"
#include "../templates/oc_PulseWidth_32Bit_Variant.h"
#include "../templates/oc_BufferSize_Default.h"
#include "../templates/oc_CompareModeSelect_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_OC_Enable(OC_MODULE_ID index)
{
     OC_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_OC_Disable(OC_MODULE_ID index)
{
     OC_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_OC_IsEnabled(OC_MODULE_ID index)
{
     return OC_IsEnabled_Default(index);
}

PLIB_INLINE_API bool PLIB_OC_ExistsEnableControl(OC_MODULE_ID index)
{
     return OC_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_OC_StopInIdleEnable(OC_MODULE_ID index)
{
     OC_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_OC_StopInIdleDisable(OC_MODULE_ID index)
{
     OC_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_OC_ExistsStopInIdle(OC_MODULE_ID index)
{
     return OC_ExistsStopInIdle_Default(index);
}

PLIB_INLINE_API void PLIB_OC_FaultInputSelect(OC_MODULE_ID index, OC_FAULTS flt)
{
     OC_FaultInputSelect_Default(index, flt);
}

PLIB_INLINE_API bool PLIB_OC_ExistsFaultInput(OC_MODULE_ID index)
{
     return OC_ExistsFaultInput_Default(index);
}

PLIB_INLINE_API bool PLIB_OC_FaultHasOccurred(OC_MODULE_ID index)
{
     return OC_FaultHasOccurred_Default(index);
}

PLIB_INLINE_API bool PLIB_OC_ExistsFaultStatus(OC_MODULE_ID index)
{
     return OC_ExistsFaultStatus_Default(index);
}

PLIB_INLINE_API void PLIB_OC_TimerSelect(OC_MODULE_ID index, OC_16BIT_TIMERS tmr)
{
     OC_TimerSelect_Default(index, tmr);
}

PLIB_INLINE_API bool PLIB_OC_ExistsTimerSelect(OC_MODULE_ID index)
{
     return OC_ExistsTimerSelect_Default(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_OC_AlternateClockEnable(OC_MODULE_ID index)
{
     OC_AlternateClockEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_OC_AlternateClockDisable(OC_MODULE_ID index)
{
     OC_AlternateClockDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_OC_ExistsAlternateClock(OC_MODULE_ID index)
{
     return OC_ExistsAlternateClock_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_OC_AlternateTimerSelect(OC_MODULE_ID index, OC_ALT_TIMERS tmr)
{
     return OC_AlternateTimerSelect_Unsupported(index, tmr);
}

PLIB_INLINE_API bool PLIB_OC_ExistsAlternateTimerSelect(OC_MODULE_ID index)
{
     return OC_ExistsAlternateTimerSelect_Unsupported(index);
}

PLIB_INLINE_API void PLIB_OC_Buffer32BitSet(OC_MODULE_ID index, uint32_t val32Bit)
{
     OC_Buffer32BitSet_32Bit_Variant(index, val32Bit);
}

PLIB_INLINE_API void PLIB_OC_Buffer16BitSet(OC_MODULE_ID index, uint16_t val16Bit)
{
     OC_Buffer16BitSet_32Bit_Variant(index, val16Bit);
}

PLIB_INLINE_API bool PLIB_OC_ExistsBufferValue(OC_MODULE_ID index)
{
     return OC_ExistsBufferValue_32Bit_Variant(index);
}

PLIB_INLINE_API void PLIB_OC_PulseWidth32BitSet(OC_MODULE_ID index, uint32_t pulseWidth)
{
     OC_PulseWidth32BitSet_32Bit_Variant(index, pulseWidth);
}

PLIB_INLINE_API void PLIB_OC_PulseWidth16BitSet(OC_MODULE_ID index, uint16_t pulseWidth)
{
     OC_PulseWidth16BitSet_32Bit_Variant(index, pulseWidth);
}

PLIB_INLINE_API bool PLIB_OC_ExistsPulseWidth(OC_MODULE_ID index)
{
     return OC_ExistsPulseWidth_32Bit_Variant(index);
}

PLIB_INLINE_API void PLIB_OC_BufferSizeSelect(OC_MODULE_ID index, OC_BUFFER_SIZE size)
{
     OC_BufferSizeSelect_Default(index, size);
}

PLIB_INLINE_API bool PLIB_OC_ExistsBufferSize(OC_MODULE_ID index)
{
     return OC_ExistsBufferSize_Default(index);
}

PLIB_INLINE_API void PLIB_OC_ModeSelect(OC_MODULE_ID index, OC_COMPARE_MODES cmpMode)
{
     OC_ModeSelect_Default(index, cmpMode);
}

PLIB_INLINE_API bool PLIB_OC_ExistsCompareModeSelect(OC_MODULE_ID index)
{
     return OC_ExistsCompareModeSelect_Default(index);
}

#endif
