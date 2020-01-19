/* Created by plibgen $Revision: 1.31 $ */

#ifndef _RTCC_P32MX470F512H_H
#define _RTCC_P32MX470F512H_H

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

    RTCC_ID_0 = _RTCC_BASE_ADDRESS,
    RTCC_NUMBER_OF_MODULES = 1

} RTCC_MODULE_ID;

typedef enum {

    RTCC_ALARM_EVERY_HALF_SECOND = 0x00,
    RTCC_ALARM_EVERY_SECOND = 0x01,
    RTCC_ALARM_EVERY_10_SECONDS = 0x02,
    RTCC_ALARM_EVERY_MINUTE = 0x03,
    RTCC_ALARM_EVERY_10_MINUTES = 0x04,
    RTCC_ALARM_EVERY_HOUR = 0x05,
    RTCC_ALARM_ONCE_A_DAY = 0x06,
    RTCC_ALARM_ONCE_A_WEEK = 0x07,
    RTCC_ALARM_ONCE_A_MONTH = 0x08,
    RTCC_ALARM_ONCE_A_YEAR = 0x09

} RTCC_ALARM_MASK_CONFIGURATION;

typedef enum {

    RTCC_OUTPUT_ALARM_PULSE = 0x00,
    RTCC_OUTPUT_SECONDS_CLOCK = 0x01

} RTCC_OUTPUT_SELECT;

typedef enum {

    RTCC_CLOCK_SOURCE_SELECT_NONE

} RTCC_CLOCK_SOURCE_SELECT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/rtcc_EnableControl_Default.h"
#include "../templates/rtcc_WriteEnable_Default.h"
#include "../templates/rtcc_StopInIdle_Default.h"
#include "../templates/rtcc_OutputSelect_Default.h"
#include "../templates/rtcc_ClockSelect_Unsupported.h"
#include "../templates/rtcc_ClockRunning_Default.h"
#include "../templates/rtcc_Calibration_Default.h"
#include "../templates/rtcc_Sync_Default.h"
#include "../templates/rtcc_HalfSecond_Default.h"
#include "../templates/rtcc_OutputControl_Default.h"
#include "../templates/rtcc_AlarmControl_Default.h"
#include "../templates/rtcc_AlarmChimeControl_Default.h"
#include "../templates/rtcc_AlarmPulseInitial_Default.h"
#include "../templates/rtcc_AlarmSync_Default.h"
#include "../templates/rtcc_AlarmMaskControl_Default.h"
#include "../templates/rtcc_AlarmRepeatControl_Default.h"
#include "../templates/rtcc_RTCTime_Default.h"
#include "../templates/rtcc_RTCDate_Default.h"
#include "../templates/rtcc_AlarmTime_Default.h"
#include "../templates/rtcc_AlarmDate_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_RTCC_Enable(RTCC_MODULE_ID index)
{
     RTCC_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_Disable(RTCC_MODULE_ID index)
{
     RTCC_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsEnableControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_WriteEnable(RTCC_MODULE_ID index)
{
     RTCC_WriteEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_WriteDisable(RTCC_MODULE_ID index)
{
     RTCC_WriteDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsWriteEnable(RTCC_MODULE_ID index)
{
     return RTCC_ExistsWriteEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_StopInIdleEnable(RTCC_MODULE_ID index)
{
     RTCC_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_StopInIdleDisable(RTCC_MODULE_ID index)
{
     RTCC_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsStopInIdleControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsStopInIdleControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_OutputSelect(RTCC_MODULE_ID index, RTCC_OUTPUT_SELECT data)
{
     RTCC_OutputSelect_Default(index, data);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsOutputSelect(RTCC_MODULE_ID index)
{
     return RTCC_ExistsOutputSelect_Default(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_RTCC_ClockSourceSelect(RTCC_MODULE_ID index, RTCC_CLOCK_SOURCE_SELECT source)
{
     RTCC_ClockSourceSelect_Unsupported(index, source);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsClockSelect(RTCC_MODULE_ID index)
{
     return RTCC_ExistsClockSelect_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ClockRunningStatus(RTCC_MODULE_ID index)
{
     return RTCC_ClockRunningStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsClockRunning(RTCC_MODULE_ID index)
{
     return RTCC_ExistsClockRunning_Default(index);
}

PLIB_INLINE_API uint16_t PLIB_RTCC_DriftCalibrateGet(RTCC_MODULE_ID index)
{
     return RTCC_DriftCalibrateGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_DriftCalibrateSet(RTCC_MODULE_ID index, uint16_t calibrationbits)
{
     RTCC_DriftCalibrateSet_Default(index, calibrationbits);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsCalibration(RTCC_MODULE_ID index)
{
     return RTCC_ExistsCalibration_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_RTCSyncStatusGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCSyncStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsSynchronization(RTCC_MODULE_ID index)
{
     return RTCC_ExistsSynchronization_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_HalfSecondStatusGet(RTCC_MODULE_ID index)
{
     return RTCC_HalfSecondStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsHalfSecond(RTCC_MODULE_ID index)
{
     return RTCC_ExistsHalfSecond_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_ClockOutputEnable(RTCC_MODULE_ID index)
{
     RTCC_ClockOutputEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_ClockOutputDisable(RTCC_MODULE_ID index)
{
     RTCC_ClockOutputDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsOutputControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsOutputControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmEnable(RTCC_MODULE_ID index)
{
     RTCC_AlarmEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmDisable(RTCC_MODULE_ID index)
{
     RTCC_AlarmDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmChimeEnable(RTCC_MODULE_ID index)
{
     RTCC_AlarmChimeEnable_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmChimeDisable(RTCC_MODULE_ID index)
{
     RTCC_AlarmChimeDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmChimeControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmChimeControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmPulseInitialSet(RTCC_MODULE_ID index, bool data)
{
     RTCC_AlarmPulseInitialSet_Default(index, data);
}

PLIB_INLINE_API bool PLIB_RTCC_AlarmPulseInitialGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmPulseInitialGet_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmPulseInitial(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmPulseInitial_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_AlarmSyncStatusGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmSyncStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmSynchronization(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmSynchronization_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmMaskModeSelect(RTCC_MODULE_ID index, RTCC_ALARM_MASK_CONFIGURATION data)
{
     RTCC_AlarmMaskModeSelect_Default(index, data);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmMaskControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmMaskControl_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmRepeatCountSet(RTCC_MODULE_ID index, uint8_t data)
{
     RTCC_AlarmRepeatCountSet_Default(index, data);
}

PLIB_INLINE_API uint8_t PLIB_RTCC_AlarmRepeatCountGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmRepeatCountGet_Default(index);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmRepeatControl(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmRepeatControl_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCTimeGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCTimeGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCTimeSet(RTCC_MODULE_ID index, uint32_t data)
{
     RTCC_RTCTimeSet_Default(index, data);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCHourGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCHourGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCHourSet(RTCC_MODULE_ID index, uint32_t hour)
{
     RTCC_RTCHourSet_Default(index, hour);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCMinuteGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCMinuteGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCMinuteSet(RTCC_MODULE_ID index, uint32_t minute)
{
     RTCC_RTCMinuteSet_Default(index, minute);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCSecondGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCSecondGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCSecondSet(RTCC_MODULE_ID index, uint32_t second)
{
     RTCC_RTCSecondSet_Default(index, second);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsRTCTime(RTCC_MODULE_ID index)
{
     return RTCC_ExistsRTCTime_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCDateGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCDateGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCDateSet(RTCC_MODULE_ID index, uint32_t data)
{
     RTCC_RTCDateSet_Default(index, data);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCYearGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCYearGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCYearSet(RTCC_MODULE_ID index, uint32_t year)
{
     RTCC_RTCYearSet_Default(index, year);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCMonthGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCMonthGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCMonthSet(RTCC_MODULE_ID index, uint32_t month)
{
     RTCC_RTCMonthSet_Default(index, month);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCDayGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCDayGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCDaySet(RTCC_MODULE_ID index, uint32_t day)
{
     RTCC_RTCDaySet_Default(index, day);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_RTCWeekDayGet(RTCC_MODULE_ID index)
{
     return RTCC_RTCWeekDayGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_RTCWeekDaySet(RTCC_MODULE_ID index, uint32_t weekday)
{
     RTCC_RTCWeekDaySet_Default(index, weekday);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsRTCDate(RTCC_MODULE_ID index)
{
     return RTCC_ExistsRTCDate_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmTimeGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmTimeGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmTimeSet(RTCC_MODULE_ID index, uint32_t data)
{
     RTCC_AlarmTimeSet_Default(index, data);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmHourGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmHourGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmHourSet(RTCC_MODULE_ID index, uint32_t hour)
{
     RTCC_AlarmHourSet_Default(index, hour);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmMinuteGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmMinuteGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmMinuteSet(RTCC_MODULE_ID index, uint32_t minute)
{
     RTCC_AlarmMinuteSet_Default(index, minute);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmSecondGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmSecondGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmSecondSet(RTCC_MODULE_ID index, uint32_t second)
{
     RTCC_AlarmSecondSet_Default(index, second);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmTime(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmTime_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmDateGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmDateGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmDateSet(RTCC_MODULE_ID index, uint32_t data)
{
     RTCC_AlarmDateSet_Default(index, data);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmMonthGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmMonthGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmMonthSet(RTCC_MODULE_ID index, uint32_t month)
{
     RTCC_AlarmMonthSet_Default(index, month);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmDayGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmDayGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmDaySet(RTCC_MODULE_ID index, uint32_t day)
{
     RTCC_AlarmDaySet_Default(index, day);
}

PLIB_INLINE_API uint32_t PLIB_RTCC_AlarmWeekDayGet(RTCC_MODULE_ID index)
{
     return RTCC_AlarmWeekDayGet_Default(index);
}

PLIB_INLINE_API void PLIB_RTCC_AlarmWeekDaySet(RTCC_MODULE_ID index, uint32_t weekday)
{
     RTCC_AlarmWeekDaySet_Default(index, weekday);
}

PLIB_INLINE_API bool PLIB_RTCC_ExistsAlarmDate(RTCC_MODULE_ID index)
{
     return RTCC_ExistsAlarmDate_Default(index);
}

#endif
