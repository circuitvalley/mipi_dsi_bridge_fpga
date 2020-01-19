/* Created by plibgen $Revision: 1.31 $ */

#ifndef _MCPWM_P32MZ2048EFM064_H
#define _MCPWM_P32MZ2048EFM064_H

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

    MCPWM_NUMBER_OF_MODULES = 0

} MCPWM_MODULE_ID;

typedef enum {

    MCPWM_CHANNEL_ID_NONE

} MCPWM_CHANNEL_ID;

typedef enum {

    MCPWM_TRIGGER_DIVIDER_NONE

} MCPWM_TRIGGER_DIVIDER;

typedef enum {

    MCPWM_CLOCK_DIVIDER_NONE

} MCPWM_CLOCK_DIVIDER;

typedef enum {

    MCPWM_CHOP_CLOCK_CONTROL_NONE

} MCPWM_CHOP_CLOCK_CONTROL;

typedef enum {

    MCPWM_TIME_BASE_SOURCE_NONE

} MCPWM_TIME_BASE_SOURCE;

typedef enum {

    MCPWM_TIME_BASE_MODE_NONE

} MCPWM_TIME_BASE_MODE;

typedef enum {

    MCPWM_ALIGNMENT_MODE_NONE

} MCPWM_ALIGNMENT_MODE;

typedef enum {

    MCPWM_OUTPUT_MODE_NONE

} MCPWM_OUTPUT_MODE;

typedef enum {

    MCPWM_PWMxH_OUTPUT_POLARITY_NONE

} MCPWM_PWMxH_OUTPUT_POLARITY;

typedef enum {

    MCPWM_PWMxL_OUTPUT_POLARITY_NONE

} MCPWM_PWMxL_OUTPUT_POLARITY;

typedef enum {

    MCPWM_DEADTIME_MODE_NONE

} MCPWM_DEADTIME_MODE;

typedef enum {

    MCPWM_DEADTIME_COMPENSATION_POLARITY_NONE

} MCPWM_DEADTIME_COMPENSATION_POLARITY;

typedef enum {

    MCPWM_TIMER_DIRECTION_NONE

} MCPWM_TIMER_DIRECTION;

typedef enum {

    MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT_NONE

} MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT;

typedef enum {

    MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT_NONE

} MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT;

typedef enum {

    MCPWM_ADC_TRIGGER_SOURCE_NONE

} MCPWM_ADC_TRIGGER_SOURCE;

typedef enum {

    MCPWM_TRIGGER_INTERRUPT_SOURCE_NONE

} MCPWM_TRIGGER_INTERRUPT_SOURCE;

typedef enum {

    MCPWM_FAULT_INPUT_LEB_CONTROL_NONE

} MCPWM_FAULT_INPUT_LEB_CONTROL;

typedef enum {

    MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL_NONE

} MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL;

typedef enum {

    MCPWM_CHOP_CLOCK_SOURCE_NONE

} MCPWM_CHOP_CLOCK_SOURCE;

typedef enum {

    MCPWM_PWMxH_CHOP_CONTROL_NONE

} MCPWM_PWMxH_CHOP_CONTROL;

typedef enum {

    MCPWM_PWMxL_CHOP_CONTROL_NONE

} MCPWM_PWMxL_CHOP_CONTROL;

typedef enum {

    MCPWM_CURRENTLIMIT_SOURCE_NONE

} MCPWM_CURRENTLIMIT_SOURCE;

typedef enum {

    MCPWM_CURRENTLIMIT_INPUT_POLARITY_NONE

} MCPWM_CURRENTLIMIT_INPUT_POLARITY;

typedef enum {

    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE_NONE

} MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE_NONE

} MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE;

typedef enum {

    MCPWM_CURRENTLIMIT_MODE_NONE

} MCPWM_CURRENTLIMIT_MODE;

typedef enum {

    MCPWM_FAULT_SOURCE_NONE

} MCPWM_FAULT_SOURCE;

typedef enum {

    MCPWM_FAULT_INPUT_POLARITY_NONE

} MCPWM_FAULT_INPUT_POLARITY;

typedef enum {

    MCPWM_FAULT_OVERRIDE_PWMxH_VALUE_NONE

} MCPWM_FAULT_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_FAULT_OVERRIDE_PWMxL_VALUE_NONE

} MCPWM_FAULT_OVERRIDE_PWMxL_VALUE;

typedef enum {

    MCPWM_FAULT_MODE_NONE

} MCPWM_FAULT_MODE;

typedef enum {

    MCPWM_OVERRIDE_PWMxH_VALUE_NONE

} MCPWM_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_OVERRIDE_PWMxL_VALUE_NONE

} MCPWM_OVERRIDE_PWMxL_VALUE;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_Enable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_Disable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_StopInIdleEnable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_StopInIdleDisable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ModuleIsReady(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsModuleControl(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_PrimarySpecialEventTriggerSetup(MCPWM_MODULE_ID index, uint16_t compare_value, MCPWM_TRIGGER_DIVIDER postscaler_value)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_PrimarySpecialEventTriggerInterruptIsPending(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_PrimarySpecialEventTriggerInterruptEnable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_PrimarySpecialEventTriggerInterruptDisable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsPrimarySpecialEventTrigger(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_SecondarySpecialEventTriggerSetup(MCPWM_MODULE_ID index, uint16_t compare_value, MCPWM_TRIGGER_DIVIDER postscaler_value)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_SecondarySpecialEventTriggerInterruptIsPending(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_SecondarySpecialEventTriggerInterruptEnable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_SecondarySpecialEventTriggerInterruptDisable(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsSecondarySpecialEventTrigger(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_PrimaryTimerSetup(MCPWM_MODULE_ID index, MCPWM_CLOCK_DIVIDER clock_div, uint16_t period_value)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_MCPWM_PrimaryTimerCountRead(MCPWM_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsPrimaryTimerSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_SecondaryTimerSetup(MCPWM_MODULE_ID index, MCPWM_CLOCK_DIVIDER clock_div, uint16_t period_value)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_MCPWM_SecondaryTimerCountRead(MCPWM_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsSecondaryTimerSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChopClockSetup(MCPWM_MODULE_ID index, uint16_t mcpwm_chop_clk_divider, MCPWM_CHOP_CLOCK_CONTROL mcpwm_chop_clk_control)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChopClockSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel_id, MCPWM_TIME_BASE_SOURCE time_base_source, MCPWM_TIME_BASE_MODE time_base_mode, MCPWM_ALIGNMENT_MODE mcpwm_alignment_mode, MCPWM_OUTPUT_MODE mcpwm_output_mode, MCPWM_PWMxH_OUTPUT_POLARITY mcpwm_pwmh_output_polarity, MCPWM_PWMxL_OUTPUT_POLARITY mcpwm_pwml_output_polarity, MCPWM_DEADTIME_MODE mcpwm_deadtime_mode, MCPWM_DEADTIME_COMPENSATION_POLARITY mcpwm_deadtime_compensation_polarity)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPrimaryDutyCycleSet(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSecondaryDutyCycleSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelSecondaryDutyCycleSet(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPhaseSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPhaseSet(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxHDeadtimeSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxLDeadtimeSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelDeadtimeCompSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelDeadtimeSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_TRIGGER_DIVIDER trigger_postscaler_value, MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT primary_trigger_cycle_select, MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT secondary_trigger_cycle_select, MCPWM_ADC_TRIGGER_SOURCE mcpwm_adc_trigger_source, MCPWM_TRIGGER_INTERRUPT_SOURCE mcpwm_trigger_interrupt_source, uint16_t primary_trigger_compare_value, uint16_t secondary_trigger_compare_value)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTriggerInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelTrigger(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_CURRENTLIMIT_SOURCE current_limit_source, MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE current_limit_override_pwmh_value, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE current_limit_override_pwml_value, MCPWM_CURRENTLIMIT_MODE current_limit_mode)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitCaptureRead(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelCurrentLimitIsAsserted(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelCurrentLimit(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_SOURCE mcpwm_fault_source, MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity, MCPWM_FAULT_OVERRIDE_PWMxH_VALUE mcpwm_fault_override_pwmh_value, MCPWM_FAULT_OVERRIDE_PWMxL_VALUE mcpwm_fault_override_pwml_value, MCPWM_FAULT_MODE mcpwm_fault_mode)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelFaultIsAsserted(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelFault(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelChopSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_CHOP_CLOCK_SOURCE chop_clock_source, MCPWM_PWMxH_CHOP_CONTROL mcpwm_pwmh_chop_control, MCPWM_PWMxL_CHOP_CONTROL mcpwm_pwml_chop_control)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelChopSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control, MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control, uint16_t leb_delay)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelLEBSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodResetInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodResetInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodResetInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodResetInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodResetInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPeriodResetInterrupt(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodMatchInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPeriodMatchInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxHOverrideEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxLOverrideEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxLOverrideDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelOverrideOutputSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_OVERRIDE_PWMxH_VALUE override_pwmh_value, MCPWM_OVERRIDE_PWMxL_VALUE override_pwml_value)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelOverrideSetup(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelLocalPWMTimerCountRead(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_IOCONxUnlock(MCPWM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelTimerDirectionGet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxHDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxLEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelPWMxLDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSwapHighLowEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_MCPWM_ChannelSwapHighLowDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelGeneralFunctions(MCPWM_MODULE_ID index)
{
     return (bool)0;
}

#endif
