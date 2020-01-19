/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PTG_P32MX470F512L_H
#define _PTG_P32MX470F512L_H

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

    PTG_NUMBER_OF_MODULES = 0

} PTG_MODULE_ID;

typedef enum {

    PTG_CLK_SRC_SEL_NONE

} PTG_CLK_SRC_SEL;

typedef enum {

    PTG_WDT_TIMEOUT_SEL_NONE

} PTG_WDT_TIMEOUT_SEL;

typedef enum {

    PTG_INPUT_MODE_NONE

} PTG_INPUT_MODE;

typedef enum {

    PTG_TIMER_SEL_NONE

} PTG_TIMER_SEL;

typedef enum {

    PTG_COUNTER_SEL_NONE

} PTG_COUNTER_SEL;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_ClockSourceSelect(PTG_MODULE_ID index, PTG_CLK_SRC_SEL clkSrcSel)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsClockSource(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_PrescaleSelect(PTG_MODULE_ID index, uint8_t preScaleSel)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PTG_PrescaleGet(PTG_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsPrescale(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_TriggerPulseWidthSet(PTG_MODULE_ID index, uint8_t trigOuputSel)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PTG_TriggerPulseWidthGet(PTG_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTriggerPulseWidth(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_WDTCountValueSet(PTG_MODULE_ID index, PTG_WDT_TIMEOUT_SEL wdtTimeOutSel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_DisableWDT(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API PTG_WDT_TIMEOUT_SEL _PLIB_UNSUPPORTED PLIB_PTG_WDTCountValueGet(PTG_MODULE_ID index)
{
     return (PTG_WDT_TIMEOUT_SEL)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsWDTCount(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_Enable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_Disable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsEnableControl(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_StopInIdleEnable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_StopInIdleDisable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStopInIdle(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_OutputTriggerToggle(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_OutputTriggerPulse(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsOutputTriggerMode(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_SWTEdgeTrigger(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_SWTLevelTrigger(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_SWTClear(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PTG_SWTGet(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsSWTControl(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_SingleStepEnable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_SingleStepDisable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsSingleStepControl(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_VisiblityEnable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_VisiblityDisable(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsVisibilityControl(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_ExecutionStart(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_ExecutionHalt(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStartExecution(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PTG_WDTStatusGet(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_WDTStatusClear(PTG_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PTG_ExistsWDTStatus(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PTG_IsBusy(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsPTGBusyStatus(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_InputTriggerModeSelect(PTG_MODULE_ID index, PTG_INPUT_MODE InputTrigMode)
{
     
}

PLIB_INLINE_API PTG_INPUT_MODE _PLIB_UNSUPPORTED PLIB_PTG_InputTriggerModeGet(PTG_MODULE_ID index)
{
     return (PTG_INPUT_MODE)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsInputTriggerMode(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_TriggerBroadcastMaskSet(PTG_MODULE_ID index, uint32_t broadcastMask)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PTG_TriggerBroadcastMaskGet(PTG_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTriggerBroadcastMask(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_HoldValueSet(PTG_MODULE_ID index, uint16_t holdValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_HoldValueGet(PTG_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsHoldValue(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_TimerLimitSet(PTG_MODULE_ID index, PTG_TIMER_SEL timerSel, uint16_t timerLimitValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_TimerLimitGet(PTG_MODULE_ID index, PTG_TIMER_SEL timerSel)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTimerLimit(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_StepDelaySet(PTG_MODULE_ID index, uint16_t stepDelayLimit)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_StepDelayGet(PTG_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStepDelay(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_CounterLimitSet(PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel, uint16_t counterLimit)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_CounterLimitGet(PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsCounterLimit(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_AdjustValueSet(PTG_MODULE_ID index, uint16_t adjustValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_AdjustValueGet(PTG_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsAdjustValue(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_LiteralStrobeValueSet(PTG_MODULE_ID index, uint16_t strobeValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PTG_LiteralStrobeValueGet(PTG_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsLiteralStrobe(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_QueuePointerSet(PTG_MODULE_ID index, uint8_t queueLoc)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PTG_QueuePointerGet(PTG_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsQueuePointer(PTG_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PTG_StepCommandSet(PTG_MODULE_ID index, uint8_t stepLoc, uint8_t command)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PTG_StepCommandGet(PTG_MODULE_ID index, uint8_t stepLoc)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStepCommand(PTG_MODULE_ID index)
{
     return (bool)0;
}

#endif
