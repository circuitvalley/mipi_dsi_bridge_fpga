/* Created by plibgen $Revision: 1.31 $ */

#ifndef _CTR_P32MX470F512L_H
#define _CTR_P32MX470F512L_H

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

    CTR_NUMBER_OF_MODULES = 0

} CTR_MODULE_ID;

typedef enum {

    CTR_SELECT_NONE

} CTR_SELECT;

typedef enum {

    CTR_LATCH_UNIT_SELECT_NONE

} CTR_LATCH_UNIT_SELECT;

typedef enum {

    CTR_ENABLE_CONTROL_NONE

} CTR_ENABLE_CONTROL;

typedef enum {

    CTR_MODE_SELECT_NONE

} CTR_MODE_SELECT;

typedef enum {

    CTR_LATCH_TRIGGER_SELECT_NONE

} CTR_LATCH_TRIGGER_SELECT;

typedef enum {

    CTR_LATCH_CTR_SELECT_NONE

} CTR_LATCH_CTR_SELECT;

typedef enum {

    CTR_ENABLE_LATCH_INT_GEN_NONE

} CTR_ENABLE_LATCH_INT_GEN;

typedef enum {

    CTR_LATCH_INT_MODE_NONE

} CTR_LATCH_INT_MODE;

typedef enum {

    CTR_TEST_BUS_SELECT_NONE

} CTR_TEST_BUS_SELECT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_CTR_Exists1394Mode(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_1394ModeSecondGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_1394ModeSecondSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t secVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_1394ModeCountGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_1394ModeCountSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t countVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_1394ModeOffsetGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_1394ModeOffsetSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t offsetVal)
{
     
}

PLIB_INLINE_API bool PLIB_CTR_ExistsUSMode(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_USModeSecondGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_USModeSecondSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t secUSVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_USModeValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_USModeValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t usVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_USMode10nsGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_USMode10nsSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t us10nsVal)
{
     
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLinearCTR(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LinearCTRGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsEnableCTR(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_EnableCTR(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_DisableCTR(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     
}

PLIB_INLINE_API CTR_ENABLE_CONTROL _PLIB_UNSUPPORTED PLIB_CTR_ModuleStatus(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (CTR_ENABLE_CONTROL)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRFormatSel(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CTRModeSelect(CTR_MODULE_ID index, CTR_SELECT ctrSel, CTR_MODE_SELECT modeVal)
{
     
}

PLIB_INLINE_API CTR_MODE_SELECT _PLIB_UNSUPPORTED PLIB_CTR_CTRModeStatus(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (CTR_MODE_SELECT)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRAdjustUS(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CTRAdjustValueInitialize(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t adjVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRAdjustValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftUS(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CTRDriftValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t driftVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftAccuUS(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRAccuUSDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRAdjustLIN(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CTRLinearAdjustInitialize(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t linAdjVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRLinearAdjustGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftLIN(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CTRLinearDriftSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t linDriftVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRLinearDriftGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftAccuLIN(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CTRAccuLinDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchTriggerSelect(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_LatchTriggerSelect(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, CTR_LATCH_TRIGGER_SELECT latTrigSrc)
{
     
}

PLIB_INLINE_API CTR_LATCH_TRIGGER_SELECT _PLIB_UNSUPPORTED PLIB_CTR_LatchTriggerGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (CTR_LATCH_TRIGGER_SELECT)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_LatchCTRSelect(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, CTR_LATCH_CTR_SELECT latctrVal)
{
     
}

PLIB_INLINE_API CTR_LATCH_CTR_SELECT _PLIB_UNSUPPORTED PLIB_CTR_LatchCTRGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (CTR_LATCH_CTR_SELECT)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_LatchDivSet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, uint32_t divVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LatchDivGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchValue(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LatchGetValue(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchStatus(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LatchGetStatus(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchTriggerCountValue(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LatchTriggerCountGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsTrigger(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_TriggerSelect(CTR_MODULE_ID index, CTR_LATCH_CTR_SELECT ctrTrigVal)
{
     
}

PLIB_INLINE_API CTR_LATCH_CTR_SELECT _PLIB_UNSUPPORTED PLIB_CTR_TriggerGet(CTR_MODULE_ID index)
{
     return (CTR_LATCH_CTR_SELECT)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_CycleOffsetValueSet(CTR_MODULE_ID index, uint32_t cycleOffsetVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_CycleOffsetValueGet(CTR_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsNValue(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_NValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueN)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_NValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsMValue(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_MValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueM)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_MValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLSBValue(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_LSBValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueLSB)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_LSBValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsInterrupt(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_IntModeLatchSelect(CTR_MODULE_ID index, CTR_LATCH_INT_MODE intMode, CTR_LATCH_UNIT_SELECT latNum)
{
     
}

PLIB_INLINE_API CTR_LATCH_INT_MODE _PLIB_UNSUPPORTED PLIB_CTR_IntModeLatchGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return (CTR_LATCH_INT_MODE)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_IntLatchSelect(CTR_MODULE_ID index, CTR_ENABLE_LATCH_INT_GEN enableLatchVal)
{
     
}

PLIB_INLINE_API CTR_ENABLE_LATCH_INT_GEN _PLIB_UNSUPPORTED PLIB_CTR_IntLatchGet(CTR_MODULE_ID index)
{
     return (CTR_ENABLE_LATCH_INT_GEN)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsSpare(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_SpareValueGet(CTR_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_SpareValueSet(CTR_MODULE_ID index, uint32_t spareVal)
{
     
}

PLIB_INLINE_API bool PLIB_CTR_ExistsTestBusSelect(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CTR_TestBusSelect(CTR_MODULE_ID index, CTR_TEST_BUS_SELECT testBusVal)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_TestBusGet(CTR_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_CTR_ExistsRevision(CTR_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_CTR_BlockRevisionGet(CTR_MODULE_ID index)
{
     return (uint32_t)0;
}

#endif
